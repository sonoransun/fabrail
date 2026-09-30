//! EEW fusion: AlertFeedKind sources, max-intensity plus S-ETA, capped log-odds, cost-loss decision; fault-injection Injector (FAB-EXO-801..810). Owner: X3 (API frozen by W0a).
//!
//! EXOTIC §X10; PHYSICS §20. Every alert in fabrail is synthetic or injected: fabrail is
//! not an EEW provider (live feeds need ShakeAlert LtO / JMA licensing and are roadmap).
//! The fusion is an estimator for the exotic tier and the choreographer; it is not a
//! rated safety function, and whatever it proposes passes the core `admit` filter.
//!
//! **Sources** ([`AlertFeedKind`]; the kind lives in fabrail-exo-safety because the alert
//! gate reads [`FusedAlert`] and safety crates may not depend on this crate):
//!
//! * trigger feeds `Network`, `OffshoreCabled` (S-net-like) and `Das`, pushed as
//!   [`Feed`]s (M, epicentral distance, depth, origin time, optional site MMI);
//! * the on-site P pick (`LocalP`) and the detector features, which arrive only through
//!   [`LocalDetection`] (a pushed `LocalP` feed is refused);
//! * `PegsMagnitudeAdvisory` (research tier): an unsaturated magnitude for an event that
//!   another feed already reports. It is never a trigger on its own (FAB-EXO-810); the
//!   on-site gradiometer exists only in the numpy frontier module.
//!
//! **Primary estimate** (exotic design §3.4, JMA-style): each source predicts the site
//! intensity I_k — the provider's MMI when given, else the intensity prediction
//! equation [`mmi_ipe`] (Atkinson & Wald 2007) at the hypocentral distance; the on-site
//! source uses Pd → PGV → MMI ([`pgv_from_pd_cmps`], [`mmi_from_pgv`]). The source with
//! the **maximum** I drives the event. Its S-wave ETA is t_S ~ N(t0 + R_hyp/V_s, σ)
//! ([`wave_eta`]; PHYSICS §20) and P(I ≥ I_crit) = Φ((I_max − I_crit)/σ_I) is the prior.
//!
//! **Secondary** (capped log-odds over the local detectors, [`capped_log_odds`]):
//! logit P = logit P0 + clamp(Σ w_i·ln LR_i, ±cap). The terms are the on-site P pick
//! confirming a remote alert, the *absence* of that pick 2σ after the predicted P arrival
//! (the concurrent-quake false-alarm signature), a τc level ≥ 2 and PGA ≥ 80 gal.
//!
//! **Decision** (cost–loss, Thompson 1952 / Murphy 1977): act when P > C/L. Two actions
//! with different costs give two thresholds: freeze (C_f/L, TRIGGERED, level 1) and
//! protect (C/L, PROTECT, level 2; level 3 when I_max ≥ I_crit + 1).
//!
//! **State.** NORMAL → TRIGGERED / PROTECT (decision) → SHAKING (local strong motion) →
//! QUIET; within an event the state and level never step down. The event returns to
//! NORMAL when its remote evidence is withdrawn and the site saw nothing: a cancel, a
//! downgrade update, or a *no-show* (no local pick or shaking by the S-ETA +
//! `no_show_s`: a concurrent-quake false alarm; FAB-EXO-805). Local physics outranks a
//! remote retraction, as in the core fusion: once the site picked P or shook, only
//! [`Fusion::reset`] (the INSPECT reset) ends the event.
//!
//! **Output**: [`FusedAlert`] = a synthetic core [`NetworkAlert`] (id `FUSED_ID_BASE` +
//! event number, updated every release, then one cancel on retraction) plus the exotic
//! extension (level, p_damaging, S-ETA μ/σ, long-period flag, tsunami ETA, feed set,
//! independent-feed count). [`core_alert`] selects what may enter the core alert link;
//! the exotic classes reach the core only through [`FusionPolicy`], the `ActionPolicy`
//! hook (RideThrough / CompleteClose / CloseAfterShaking and the land/lock mappings),
//! bounded by `fabrail_exo_safety::alert_gate` and the core `admit` filter.
//!
//! The `Injector` exists only under `cfg(feature = "fault-injection")`, which no
//! firmware feature set enables (FAB-EXO-307). Injected feeds may trigger FREEZE, LAND,
//! LOCK or SEISMIC_PASSIVE but never count as independent confirmation.

use fabrail_core::msg::SeismicReport;
use fabrail_core::seismic::action_policy::{ActionContext, ActionDecision, ActionPolicy};
use fabrail_exo_safety::alert_gate::{AlertAction, Confirmations, GateClass, gate};
use fabrail_exo_safety::presets;
use fabrail_exo_safety::{
    AlertFeedKind, DeviceCondition, ExoClass, FeedSet, FusedAlert, SafeStatePolicy, SafetyRank,
};
use fabrail_params::{SystemParams, derive};
use fabrail_types::{
    BASE_TICK_HZ, EgressRole, LevState, NetworkAlert, SeismicAction, SeismicLevel, SeismicState,
    Tick, fmath,
};

/// Feeds held at once.
pub const MAX_FEEDS: usize = 8;

/// Id base of the synthetic fused alerts ("FU" in the high half): distinct from scenario
/// ids and from the `Injector` range 0x8000_0000.
pub const FUSED_ID_BASE: u32 = 0x4655_0000;

// ---- Intensity model (EXOTIC §X10; PHYSICS §20) --------------------------------------

/// Atkinson & Wald (2007), "Did You Feel It? intensity data: a surprisingly good
/// measure of earthquake ground motion", SRL 78(3):362–368, California coefficients:
/// MMI = c1 + c2(M−6) + c3(M−6)² + c4·log R + c5·R + c6·B + c7·M·log R,
/// R = √(D² + h²), B = max(0, log(R/R_t)) \[UNCERTAIN: coefficients recalled from the
/// paper's Table 1; verify before any non-simulation use\].
const AW07_C1: f32 = 12.27;
/// See [`AW07_C1`].
const AW07_C2: f32 = 2.270;
/// See [`AW07_C1`].
const AW07_C3: f32 = 0.1304;
/// See [`AW07_C1`].
const AW07_C4: f32 = -1.30;
/// See [`AW07_C1`].
const AW07_C5: f32 = -0.000_707_0;
/// See [`AW07_C1`].
const AW07_C6: f32 = 1.95;
/// See [`AW07_C1`].
const AW07_C7: f32 = -0.577;
/// Near-source saturation depth h, km (Atkinson & Wald 2007, California).
const AW07_H_KM: f32 = 14.0;
/// Transition distance R_t of the B term, km (Atkinson & Wald 2007, California).
const AW07_RT_KM: f32 = 30.0;
/// Magnitude range the IPE is evaluated over (clamped; M9 is an extrapolation).
const IPE_M_MIN: f32 = 3.0;
/// See [`IPE_M_MIN`].
const IPE_M_MAX: f32 = 9.5;
/// Intensity scale bounds (MMI I–XII).
const MMI_MIN: f32 = 1.0;
/// See [`MMI_MIN`].
const MMI_MAX: f32 = 12.0;

/// Wu & Kanamori (2008), Sensors 8:1–9: log₁₀ PGV\[cm/s\] = 0.920·log₁₀ Pd\[cm\] + 1.642
/// (Taiwan, southern California and Japan records) \[UNCERTAIN: ±0.3 log units\].
const WK08_PGV_SLOPE: f32 = 0.920;
/// See [`WK08_PGV_SLOPE`].
const WK08_PGV_ICPT: f32 = 1.642;

/// Worden et al. (2012), BSSA 102(1):204–221, PGV ground-motion-to-intensity
/// conversion: MMI = c1 + c2·log₁₀ PGV for log₁₀ PGV ≤ t1, else c3 + c4·log₁₀ PGV
/// (PGV in cm/s; the branches meet at MMI ≈ 4.56).
const W12_C1: f32 = 3.78;
/// See [`W12_C1`].
const W12_C2: f32 = 1.47;
/// See [`W12_C1`].
const W12_C3: f32 = 2.89;
/// See [`W12_C1`].
const W12_C4: f32 = 3.16;
/// See [`W12_C1`].
const W12_T1: f32 = 0.53;

/// τc–magnitude regression log₁₀ τc = 0.296·M − 1.716 (Wu & Kanamori 2008, Sensors
/// 8:1–9; PHYSICS §20; the core LocalEta uses the same regression) \[UNCERTAIN: ±0.4 M\].
const TAUC_M_SLOPE: f32 = 0.296;
/// See [`TAUC_M_SLOPE`].
const TAUC_M_ICPT: f32 = -1.716;
/// On-site magnitude clamp (τc saturates for large events).
const LOCAL_M_MAX: f32 = 8.5;
/// On-site distance clamp for the Wu–Zhao inversion, km.
const LOCAL_R_MIN_KM: f32 = 1.0;
/// See [`LOCAL_R_MIN_KM`].
const LOCAL_R_MAX_KM: f32 = 500.0;

// ---- Log-odds evidence (secondary estimate) --------------------------------------------
//
// Likelihood ratios LR = P(evidence | damaging) / P(evidence | not damaging) of the local
// detectors, with weights w ≤ 1 discounting evidence correlated with the prior. All are
// calibration placeholders [UNCERTAIN: X7 sweeps them in the detector-ROC study]; the
// derivations below state what each value must achieve.

/// On-site P pick confirming a remote alert (the site sees an event) \[UNCERTAIN\].
const LR_PICK: f32 = 4.0;
/// No on-site pick 2σ after the predicted P arrival: the remote event is not at this
/// site (concurrent-quake mislocation, JMA 2011 false alarms) \[UNCERTAIN\]; 1/LR_PICK.
const LR_NO_PICK: f32 = 0.25;
/// On-site τc level ≥ 2 (τc ≥ 0.6 s: M ≳ 6.5) \[UNCERTAIN\].
const LR_LEVEL: f32 = 3.0;
/// Weight of the level term: τc comes from the same window as Pd, which already set the
/// on-site prior.
const W_LEVEL: f32 = 0.5;
/// On-site PGA ≥ 80 gal (strong shaking observed). Must lift the base prior over C/L on
/// its own, as the core PROTECT rule does: ln LR > logit(0.1) − logit(0.01) = 2.40, so
/// LR > 11 \[UNCERTAIN: 20 chosen\].
const LR_PGA: f32 = 20.0;
/// Prior P(damaging) for an on-site pick before Pd is available \[UNCERTAIN\]: most
/// STA/LTA triggers are small events.
const P_BASE_LOCAL: f32 = 0.01;
/// Probability clamp keeping logit finite.
const P_CLAMP: f32 = 1.0e-4;
/// The no-pick term applies this many σ after the predicted P arrival.
const NO_PICK_SIGMAS: f32 = 2.0;

// ---- Event flags -------------------------------------------------------------------------

/// Long-period flag: best magnitude ≥ Mw 7.5, where long-period (T > 2 s) ground motion
/// dominates tall and isolated structures (2003 Tokachi-oki, 2011 Tohoku)
/// \[UNCERTAIN: threshold\].
const LONG_PERIOD_MW: f32 = 7.5;
/// Tsunami ETA only for Mw ≥ 7.0 \[UNCERTAIN: JMA issues tsunami warnings for shallow
/// offshore events from about M 6.5–7\].
const TSUNAMI_MW: f32 = 7.0;
/// … and for shallow sources (depth ≤ 60 km) \[UNCERTAIN\].
const TSUNAMI_MAX_DEPTH_KM: f32 = 60.0;
/// Standard-normal 95 % quantile: the S-ETA 5 % quantile is μ − 1.645σ.
const Z_Q95: f32 = 1.644_853_6;
/// Deadline margin before the S-ETA 5 % quantile, s (plan §4b: quantile − 1 s).
pub const DEADLINE_MARGIN_S: f32 = 1.0;
/// SHAKING → QUIET after this long without new local strong motion, s \[UNCERTAIN\]:
/// longer than the gaps between successive PGA peaks during shaking.
const QUIET_HOLD_S: f32 = 2.0;

/// One early-warning feed message (synthetic or injected).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Feed {
    /// Source kind.
    pub kind: AlertFeedKind,
    /// Message id (updates reuse the id).
    pub id: u32,
    /// Base tick of reception.
    pub received: Tick,
    /// Origin time relative to reception, s (negative: the origin was earlier).
    pub origin_offset_s: f32,
    /// Moment magnitude estimate.
    pub magnitude: f32,
    /// Epicentral distance to the site, km.
    pub r_epi_km: f32,
    /// Hypocentral depth, km.
    pub depth_km: f32,
    /// Predicted intensity at the site (MMI); 0 when not provided.
    pub mmi: f32,
    /// This message cancels feed `id`.
    pub cancel: bool,
    /// Emitted by the fault-injection `Injector` (never counts as independent).
    pub injected: bool,
}

/// Fusion tuning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FusionCfg {
    /// P-wave speed, km/s (core §C.4: 6.0).
    pub v_p_kmps: f32,
    /// S-wave speed, km/s (core §C.4: 3.5).
    pub v_s_kmps: f32,
    /// S-ETA standard deviation, s (exotic design §3.4: 1–2 s; lower end).
    pub s_eta_sigma_s: f32,
    /// Damaging intensity threshold I_crit, MMI [UNCERTAIN: MMI VI, slight damage].
    pub mmi_crit: f32,
    /// Cost–loss ratio C/L for acting [UNCERTAIN: X3 calibrates per class].
    pub cost_loss: f32,
    /// Cap on the summed log-odds [UNCERTAIN: X3 calibrates].
    pub log_odds_cap: f32,
    /// Independent feeds needed for motion without a local P pick (plan §4: 2).
    pub min_independent_feeds: u8,
    /// Cost–loss ratio C_f/L of the cheaper freeze-only action (TRIGGERED, level 1)
    /// \[UNCERTAIN: a freeze costs a few seconds of interrupted motion, about 1/5 of a
    /// landing\].
    pub cost_loss_freeze: f32,
    /// Standard deviation of a predicted site intensity, MMI units \[UNCERTAIN: typical
    /// single-site IPE/GMICE scatter 0.6–0.8\].
    pub mmi_sigma: f32,
    /// S-ETA σ of an on-site-only estimate, s (the Wu–Zhao distance inversion is
    /// uncertain by about ±0.4 M; upper end of the 1–2 s range).
    pub local_eta_sigma_s: f32,
    /// A remote alert with no local P pick or shaking by S-ETA + this is a no-show (false
    /// alarm) and is withdrawn, s \[UNCERTAIN: 3σ at σ ≤ 2 s plus 4 s for the S onset\].
    pub no_show_s: f32,
    /// Origin times within this window belong to the same event (PEGS association), s.
    pub assoc_window_s: f32,
    /// A PEGS magnitude advisory expires this long after reception, s.
    pub advisory_ttl_s: f32,
    /// Local strong motion (SHAKING) and the PGA log-odds term at or above this, m/s²
    /// (80 gal, the core PROTECT PGA; plan §2b item 6).
    pub shaking_pga_mps2: f32,
    /// Effective water depth of the source-to-site tsunami path h̄, m, such that the
    /// travel time is R_epi/√(g·h̄) (EXOTIC §X12; the full ∫dx/√(g·h) is exo-sim
    /// `tsunami`); 0 = no coastal site (no tsunami ETA).
    pub tsunami_path_depth_m: f32,
}

impl Default for FusionCfg {
    fn default() -> FusionCfg {
        FusionCfg {
            v_p_kmps: 6.0,
            v_s_kmps: 3.5,
            s_eta_sigma_s: 1.0,
            mmi_crit: 6.0,
            cost_loss: 0.1,
            log_odds_cap: 4.0,
            min_independent_feeds: 2,
            cost_loss_freeze: 0.02,
            mmi_sigma: 0.7,
            local_eta_sigma_s: 2.0,
            no_show_s: 10.0,
            assoc_window_s: 30.0,
            advisory_ttl_s: 600.0,
            shaking_pga_mps2: 0.8,
            tsunami_path_depth_m: 0.0,
        }
    }
}

impl FusionCfg {
    /// The defaults with the wave speeds, S-ETA σ, independent-feed count and PGA
    /// threshold taken from the parameter set (`control.seismic.{fusion,detector}`), the
    /// single source the core fusion also reads.
    pub fn from_params(p: &SystemParams) -> FusionCfg {
        let f = &p.control.seismic.fusion;
        let d = &p.control.seismic.detector;
        FusionCfg {
            v_p_kmps: f.v_p_kmps as f32,
            v_s_kmps: f.v_s_kmps as f32,
            s_eta_sigma_s: f.eta_sigma_s as f32,
            min_independent_feeds: f.min_independent_feeds,
            shaking_pga_mps2: d.pga_alert_mps2 as f32,
            ..FusionCfg::default()
        }
    }
}

/// The on-site detector's contribution (from the core seismic report).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LocalDetection {
    /// Tick of the local P pick, if any.
    pub p_pick: Option<Tick>,
    /// Detector level.
    pub level: SeismicLevel,
    /// Peak displacement Pd, m.
    pub pd_m: f32,
    /// Average period τc, s.
    pub tau_c_s: f32,
    /// Peak ground acceleration seen, m/s².
    pub pga_mps2: f32,
}

impl LocalDetection {
    /// The on-site contribution of a core seismic report: the pick counts only when it is
    /// Z/H-confirmed (`local_p`, plan §4).
    pub fn from_report(r: &SeismicReport) -> LocalDetection {
        LocalDetection {
            p_pick: if r.local_p { r.triggered_at } else { None },
            level: r.level,
            pd_m: r.pd_m,
            tau_c_s: r.tau_c_s,
            pga_mps2: r.pga_mps2,
        }
    }
}

// ---- Pure helpers ----------------------------------------------------------------------

/// Predicted site intensity (MMI) from moment magnitude and hypocentral distance with the
/// Atkinson & Wald (2007) California IPE (EXOTIC §X10). M is clamped to 3–9.5 and the
/// result to MMI I–XII; non-finite inputs give MMI I. \[SIMPLIFIED: hypocentral, not
/// rupture, distance; no site term.\]
pub fn mmi_ipe(magnitude: f32, r_hyp_km: f32) -> f32 {
    if !(magnitude.is_finite() && r_hyp_km.is_finite()) {
        return MMI_MIN;
    }
    let m = fmath::clampf(magnitude, IPE_M_MIN, IPE_M_MAX);
    let d = fmath::fmaxf(r_hyp_km, 0.0);
    let r = fmath::hypotf(d, AW07_H_KM);
    let lr = fmath::log10f(r);
    let b = fmath::fmaxf(fmath::log10f(r / AW07_RT_KM), 0.0);
    let dm = m - 6.0;
    let i = AW07_C1
        + AW07_C2 * dm
        + AW07_C3 * dm * dm
        + AW07_C4 * lr
        + AW07_C5 * r
        + AW07_C6 * b
        + AW07_C7 * m * lr;
    fmath::clampf(i, MMI_MIN, MMI_MAX)
}

/// Peak ground velocity implied by the on-site Pd, cm/s (Wu & Kanamori 2008; PHYSICS
/// §20). Zero for a non-positive or non-finite Pd.
pub fn pgv_from_pd_cmps(pd_m: f32) -> f32 {
    if !(pd_m > 0.0 && pd_m.is_finite()) {
        return 0.0;
    }
    let lg = WK08_PGV_SLOPE * fmath::log10f(100.0 * pd_m) + WK08_PGV_ICPT;
    fmath::powf(10.0, lg)
}

/// MMI from PGV in cm/s (Worden et al. 2012), clamped to MMI I–XII.
pub fn mmi_from_pgv(pgv_cmps: f32) -> f32 {
    if !(pgv_cmps > 0.0 && pgv_cmps.is_finite()) {
        return MMI_MIN;
    }
    let lg = fmath::log10f(pgv_cmps);
    let i = if lg <= W12_T1 {
        W12_C1 + W12_C2 * lg
    } else {
        W12_C3 + W12_C4 * lg
    };
    fmath::clampf(i, MMI_MIN, MMI_MAX)
}

/// P(I ≥ I_crit) for a Gaussian intensity prediction: Φ((I − I_crit)/σ_I).
pub fn p_exceed(i_pred: f32, i_crit: f32, sigma: f32) -> f32 {
    if !(sigma > 0.0 && sigma.is_finite()) {
        return if i_pred >= i_crit { 1.0 } else { 0.0 };
    }
    let z = (i_pred - i_crit) / sigma;
    0.5 * (1.0 + fmath::erff(z * core::f32::consts::FRAC_1_SQRT_2))
}

/// ln(p/(1 − p)) with p clamped to [1e-4, 1 − 1e-4].
pub fn logit(p: f32) -> f32 {
    let q = if p.is_finite() {
        fmath::clampf(p, P_CLAMP, 1.0 - P_CLAMP)
    } else {
        0.5
    };
    fmath::lnf(q / (1.0 - q))
}

/// 1/(1 + e^(−x)).
pub fn logistic(x: f32) -> f32 {
    1.0 / (1.0 + fmath::expf(-x))
}

/// Σ w_i·ln LR_i over `(w, LR)` terms, clamped to ±`cap` (terms with a non-positive or
/// non-finite LR are skipped).
pub fn capped_log_odds(terms: &[(f32, f32)], cap: f32) -> f32 {
    let s: f32 = terms
        .iter()
        .filter(|(w, lr)| w.is_finite() && *lr > 0.0 && lr.is_finite())
        .map(|(w, lr)| w * fmath::lnf(*lr))
        .sum();
    let c = fmath::fmaxf(cap, 0.0);
    fmath::clampf(s, -c, c)
}

/// Seconds → base ticks (nearest; negative or non-finite → 0).
fn ticks_of_s(t_s: f32) -> u64 {
    let n = fmath::roundf(t_s * BASE_TICK_HZ as f32);
    if n.is_finite() && n > 0.0 {
        n as u64
    } else {
        0
    }
}

/// `t` shifted by a signed number of seconds, saturating at tick 0.
fn tick_offset(t: Tick, dt_s: f32) -> Tick {
    if dt_s >= 0.0 {
        t.plus(ticks_of_s(dt_s))
    } else {
        Tick(t.0.saturating_sub(ticks_of_s(-dt_s)))
    }
}

/// `a − b` in seconds (signed).
fn signed_s(a: Tick, b: Tick) -> f32 {
    if a >= b { a.since_s(b) } else { -b.since_s(a) }
}

/// Arrival of a wave with speed `v_kmps` at the site: anchor + origin_offset +
/// √(r_epi² + depth²)/v (PHYSICS §20: S-ETA = received + origin_offset_s + R_hyp/V_s;
/// with V_p it is the P arrival). `None` for a non-positive speed or non-finite inputs;
/// an arrival before tick 0 saturates to tick 0.
pub fn wave_eta(
    anchor: Tick,
    origin_offset_s: f32,
    r_epi_km: f32,
    depth_km: f32,
    v_kmps: f32,
) -> Option<Tick> {
    if !(v_kmps > 0.0 && v_kmps.is_finite()) {
        return None;
    }
    let dt = origin_offset_s + fmath::hypotf(r_epi_km, depth_km) / v_kmps;
    if !dt.is_finite() {
        return None;
    }
    Some(tick_offset(anchor, dt))
}

/// Motion deadline from a fused alert: the S-ETA 5 % quantile μ − 1.645σ minus
/// `margin_s` (plan §4b uses 1 s, [`DEADLINE_MARGIN_S`]). `None` without an S-ETA.
pub fn s_eta_deadline(a: &FusedAlert, margin_s: f32) -> Option<Tick> {
    let mu = a.s_eta?;
    let back = Z_Q95 * fmath::fmaxf(a.s_eta_sigma_s, 0.0) + fmath::fmaxf(margin_s, 0.0);
    if !back.is_finite() {
        return None;
    }
    Some(Tick(mu.0.saturating_sub(ticks_of_s(back))))
}

/// What of a fused alert may enter the core alert link (`SeismicUnit::step`): the core
/// treats any active network alert as PROTECT, so only protect-level (≥ 2) alerts with a
/// non-local contribution and cancels pass. A local-only event is already the core's own
/// detection; a freeze-only (level 1) alert stays advisory for the choreographer.
pub fn core_alert(a: &FusedAlert) -> Option<NetworkAlert> {
    if a.alert.cancel || (a.level >= SeismicLevel::L2 && a.non_local()) {
        Some(a.alert)
    } else {
        None
    }
}

/// True for the kinds that may trigger an event.
const fn is_trigger(k: AlertFeedKind) -> bool {
    matches!(
        k,
        AlertFeedKind::Network | AlertFeedKind::OffshoreCabled | AlertFeedKind::Das
    )
}

/// Wu–Zhao log₁₀ Pd\[cm\] = c0 + c_m·M − c_r·log₁₀ R, inverted for R (coefficients taken
/// at init from `params::derive::pd_wu_zhao`, the single source; PHYSICS §20).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct WuZhao {
    c0: f32,
    c_m: f32,
    c_r: f32,
}

impl WuZhao {
    fn new() -> WuZhao {
        let lg = |m: f64, r: f64| fmath::log10(100.0 * derive::pd_wu_zhao(m, r));
        let c0 = lg(0.0, 1.0);
        WuZhao {
            c0: c0 as f32,
            c_m: (lg(1.0, 1.0) - c0) as f32,
            c_r: (c0 - lg(0.0, 10.0)) as f32,
        }
    }

    /// Hypocentral distance implied by Pd (m) at magnitude `m`, km (clamped 1–500 km).
    fn r_hyp_km(&self, pd_m: f32, m: f32) -> Option<f32> {
        if !(pd_m > 0.0 && m.is_finite() && self.c_r > 0.0) {
            return None;
        }
        let lr = (self.c0 + self.c_m * m - fmath::log10f(100.0 * pd_m)) / self.c_r;
        let r = fmath::powf(10.0, lr);
        r.is_finite()
            .then(|| fmath::clampf(r, LOCAL_R_MIN_KM, LOCAL_R_MAX_KM))
    }
}

/// Magnitude implied by τc (Wu & Kanamori 2008), clamped to [`IPE_M_MIN`]..8.5.
fn tauc_magnitude(tau_c_s: f32) -> Option<f32> {
    if !(tau_c_s > 0.0 && tau_c_s.is_finite()) {
        return None;
    }
    let m = (fmath::log10f(tau_c_s) - TAUC_M_ICPT) / TAUC_M_SLOPE;
    Some(fmath::clampf(m, IPE_M_MIN, LOCAL_M_MAX))
}

/// One intensity source of the current event.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Source {
    /// Reception (feed) or pick (on-site) tick.
    anchor: Tick,
    /// Origin relative to `anchor`, s.
    origin_offset_s: f32,
    magnitude: f32,
    r_epi_km: f32,
    depth_km: f32,
    /// Predicted site intensity (0: unknown).
    mmi: f32,
    sigma_s: f32,
    s_eta: Option<Tick>,
    /// Predicted P arrival at the site (remote sources only).
    p_eta: Option<Tick>,
}

/// The fused estimate at one release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Estimate {
    driver: Source,
    p: f32,
    feeds: FeedSet,
    n_independent: u8,
    local_p: bool,
    injected: bool,
    m_best: f32,
    offshore: bool,
}

/// Decision of the cost–loss rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Decision {
    None,
    Freeze,
    Protect,
}

/// The fusion estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fusion {
    cfg: FusionCfg,
    feeds: [Option<Feed>; MAX_FEEDS],
    state: SeismicState,
    /// Recently cancelled (kind, id), a ring: a late re-send never revives a cancel.
    cancelled: [Option<(AlertFeedKind, u32)>; MAX_FEEDS],
    cancel_head: u8,
    /// Event counter (the synthetic alert id is `FUSED_ID_BASE + event`).
    event: u32,
    /// Level of the current event (never steps down within an event).
    level: SeismicLevel,
    /// An alert of the current event was returned (a retraction then emits a cancel).
    published: bool,
    /// First local strong-motion sample of the current event.
    shaking_start: Option<Tick>,
    /// Latest local strong-motion sample.
    last_activity: Tick,
    prev_pga: f32,
    /// Latest non-cancel output (template of the retraction message).
    last: Option<FusedAlert>,
    wz: WuZhao,
}

impl Fusion {
    /// An estimator in NORMAL with no feeds.
    pub fn new(cfg: FusionCfg) -> Fusion {
        Fusion {
            cfg,
            feeds: [None; MAX_FEEDS],
            state: SeismicState::Normal,
            cancelled: [None; MAX_FEEDS],
            cancel_head: 0,
            event: 0,
            level: SeismicLevel::L0,
            published: false,
            shaking_start: None,
            last_activity: Tick::ZERO,
            prev_pga: 0.0,
            last: None,
            wz: WuZhao::new(),
        }
    }

    /// Stores a feed (a same-id message replaces the older one); `false` when it is not
    /// stored: full, invalid (non-finite or negative fields), a pushed `LocalP` (the pick
    /// enters through [`LocalDetection`]), or a re-send of a cancelled id. A cancel
    /// message removes the matching (kind, id) and is always accepted; the next
    /// [`Fusion::step`] re-evaluates the event.
    pub fn push(&mut self, f: &Feed) -> bool {
        if f.cancel {
            for s in self.feeds.iter_mut() {
                if s.is_some_and(|o| o.id == f.id && o.kind == f.kind) {
                    *s = None;
                }
            }
            self.remember_cancel(f.kind, f.id);
            return true;
        }
        if f.kind == AlertFeedKind::LocalP
            || self.is_cancelled(f.kind, f.id)
            || !(f.origin_offset_s.is_finite()
                && f.magnitude.is_finite()
                && f.r_epi_km >= 0.0
                && f.r_epi_km.is_finite()
                && f.depth_km >= 0.0
                && f.depth_km.is_finite()
                && f.mmi >= 0.0
                && f.mmi.is_finite())
        {
            return false;
        }
        if let Some(slot) = self
            .feeds
            .iter_mut()
            .find(|s| s.is_some_and(|o| o.id == f.id && o.kind == f.kind))
        {
            *slot = Some(*f);
            return true;
        }
        if let Some(slot) = self.feeds.iter_mut().find(|s| s.is_none()) {
            *slot = Some(*f);
            return true;
        }
        // Full: a trigger feed evicts the oldest PEGS advisory, never another trigger.
        if is_trigger(f.kind) {
            let oldest = self
                .feeds
                .iter_mut()
                .filter(|s| s.is_some_and(|o| o.kind == AlertFeedKind::PegsMagnitudeAdvisory))
                .min_by_key(|s| s.map_or(Tick(u64::MAX), |o| o.received));
            if let Some(slot) = oldest {
                *slot = Some(*f);
                return true;
            }
        }
        false
    }

    /// Fuses at `t` with the local detection; returns the fused alert while an event is
    /// active (every release, with `alert.received = t`), then one message with
    /// `alert.cancel` set when the event is withdrawn, and `None` otherwise.
    pub fn step(&mut self, t: Tick, local: &LocalDetection) -> Option<FusedAlert> {
        let cfg = self.cfg;

        // PEGS advisories expire; they never keep an event alive.
        for s in self.feeds.iter_mut() {
            if s.is_some_and(|o| {
                o.kind == AlertFeedKind::PegsMagnitudeAdvisory
                    && !(t.since_s(o.received) <= cfg.advisory_ttl_s
                        && cfg.advisory_ttl_s.is_finite())
            }) {
                *s = None;
            }
        }

        // Local strong motion: a new PGA value at or above the threshold (works for both
        // a running event peak and a per-release window peak).
        let pga = if local.pga_mps2.is_finite() {
            local.pga_mps2
        } else {
            0.0
        };
        let active = pga >= cfg.shaking_pga_mps2 && pga != self.prev_pga;
        self.prev_pga = pga;
        if active {
            self.last_activity = t;
            if self.shaking_start.is_none() {
                self.shaking_start = Some(t);
            }
        }
        let local_evidence = local.p_pick.is_some() || self.shaking_start.is_some();

        // No-show: a remote alert whose S wave should have arrived long ago while the site
        // saw nothing is a false alarm (concurrent-quake mislocation; FAB-EXO-805).
        if !local_evidence {
            let (v_s, no_show) = (cfg.v_s_kmps, cfg.no_show_s);
            for s in self.feeds.iter_mut() {
                let late = s.is_some_and(|o| {
                    is_trigger(o.kind)
                        && wave_eta(o.received, o.origin_offset_s, o.r_epi_km, o.depth_km, v_s)
                            .is_some_and(|eta| signed_s(t, eta) > no_show)
                });
                if late {
                    *s = None;
                }
            }
        }

        let est = self.evaluate(t, local);
        let decision = est.map_or(Decision::None, |e| {
            if e.p > cfg.cost_loss {
                Decision::Protect
            } else if e.p > cfg.cost_loss_freeze {
                Decision::Freeze
            } else {
                Decision::None
            }
        });

        let prev = self.state;
        let mut next = match (prev, decision) {
            (SeismicState::Normal, Decision::None) => SeismicState::Normal,
            (SeismicState::Normal | SeismicState::Triggered, Decision::Freeze) => {
                SeismicState::Triggered
            }
            (SeismicState::Normal | SeismicState::Triggered, Decision::Protect) => {
                SeismicState::Protect
            }
            // Withdrawn remote evidence ends the event only if the site saw nothing.
            (_, Decision::None) if !local_evidence => SeismicState::Normal,
            // Never step down within an event.
            (SeismicState::Inspect, _) => SeismicState::Quiet,
            (s, _) => s,
        };
        if next != SeismicState::Normal {
            if active {
                next = SeismicState::Shaking;
            } else if next == SeismicState::Shaking && t.since_s(self.last_activity) >= QUIET_HOLD_S
            {
                next = SeismicState::Quiet;
            }
        }

        if next == SeismicState::Normal {
            let out = match (prev != SeismicState::Normal && self.published, self.last) {
                (true, Some(l)) => Some(FusedAlert {
                    alert: NetworkAlert {
                        received: t,
                        cancel: true,
                        ..l.alert
                    },
                    level: SeismicLevel::L0,
                    p_damaging: est.map_or(0.0, |e| e.p),
                    s_eta: None,
                    tsunami_eta: None,
                    ..l
                }),
                _ => None,
            };
            self.end_event();
            return out;
        }

        if prev == SeismicState::Normal {
            self.event = self.event.wrapping_add(1);
            self.published = false;
            self.level = SeismicLevel::L0;
        }
        self.state = next;

        let Some(e) = est else {
            // Latched by local evidence without a usable source: repeat the last output.
            let out = self.last.map(|l| FusedAlert {
                alert: NetworkAlert {
                    received: t,
                    ..l.alert
                },
                ..l
            });
            self.last = out;
            return out;
        };

        let level_now = match decision {
            Decision::None => SeismicLevel::L0,
            Decision::Freeze => SeismicLevel::L1,
            Decision::Protect if e.driver.mmi >= cfg.mmi_crit + 1.0 => SeismicLevel::L3,
            Decision::Protect => SeismicLevel::L2,
        };
        let floor = match next {
            SeismicState::Triggered => SeismicLevel::L1,
            _ => SeismicLevel::L2,
        };
        self.level = self.level.max(level_now).max(floor);

        let d = e.driver;
        let tsunami_eta = if e.offshore
            && e.m_best >= TSUNAMI_MW
            && d.depth_km <= TSUNAMI_MAX_DEPTH_KM
            && cfg.tsunami_path_depth_m > 0.0
        {
            // Shallow-water speed c = √(g·h̄) (EXOTIC §X12; 198 m/s at 4 km).
            let c_kmps = fmath::sqrtf(derive::G0_MPS2 as f32 * cfg.tsunami_path_depth_m) / 1000.0;
            wave_eta(d.anchor, d.origin_offset_s, d.r_epi_km, 0.0, c_kmps)
        } else {
            None
        };

        let out = FusedAlert {
            alert: NetworkAlert {
                id: FUSED_ID_BASE.wrapping_add(self.event),
                received: t,
                origin_offset_s: signed_s(d.anchor, t) + d.origin_offset_s,
                magnitude: d.magnitude,
                r_epi_km: d.r_epi_km,
                depth_km: d.depth_km,
                mmi: d.mmi,
                cancel: false,
            },
            level: self.level,
            p_damaging: e.p,
            s_eta: d.s_eta,
            s_eta_sigma_s: d.sigma_s,
            long_period: e.m_best >= LONG_PERIOD_MW,
            tsunami_eta,
            feeds: e.feeds,
            n_independent: e.n_independent,
            local_p: e.local_p,
            injected: e.injected,
        };
        self.published = true;
        self.last = Some(out);
        Some(out)
    }

    /// Seismic state as seen by the fusion.
    pub const fn state(&self) -> SeismicState {
        self.state
    }

    /// Clears every feed and returns to NORMAL (the INSPECT reset).
    pub fn reset(&mut self) {
        self.feeds = [None; MAX_FEEDS];
        self.cancelled = [None; MAX_FEEDS];
        self.cancel_head = 0;
        // `prev_pga` is kept: a running PGA peak that has not changed is not new motion.
        self.end_event();
    }

    /// The configuration.
    pub const fn cfg(&self) -> FusionCfg {
        self.cfg
    }

    /// Feeds currently held (in slot order).
    pub fn feeds(&self) -> impl Iterator<Item = &Feed> {
        self.feeds.iter().flatten()
    }

    /// Clears the per-event state (feeds other than the withdrawn ones are kept).
    fn end_event(&mut self) {
        self.state = SeismicState::Normal;
        self.level = SeismicLevel::L0;
        self.published = false;
        self.shaking_start = None;
        self.last = None;
    }

    fn remember_cancel(&mut self, kind: AlertFeedKind, id: u32) {
        let i = usize::from(self.cancel_head) % MAX_FEEDS;
        if let Some(s) = self.cancelled.get_mut(i) {
            *s = Some((kind, id));
        }
        self.cancel_head = ((i + 1) % MAX_FEEDS) as u8;
    }

    fn is_cancelled(&self, kind: AlertFeedKind, id: u32) -> bool {
        self.cancelled.contains(&Some((kind, id)))
    }

    /// PEGS-upgraded magnitude of trigger feed `f`: the largest associated advisory
    /// magnitude above the feed's own (the unsaturated magnitude; PEGSNet, Licciardi et
    /// al. 2022, Nature 606:319–324), and whether an advisory was used.
    fn pegs_magnitude(&self, f: &Feed) -> (f32, bool) {
        let mut m = f.magnitude;
        let mut used = false;
        for p in self.feeds.iter().flatten() {
            if p.kind != AlertFeedKind::PegsMagnitudeAdvisory
                || !(p.magnitude > m && p.magnitude.is_finite())
            {
                continue;
            }
            let d_origin = signed_s(f.received, p.received) + f.origin_offset_s - p.origin_offset_s;
            if fmath::absf(d_origin) <= self.cfg.assoc_window_s {
                m = p.magnitude;
                used = true;
            }
        }
        (m, used)
    }

    /// The on-site source: magnitude from τc, distance from the Wu–Zhao inversion of Pd,
    /// intensity from Pd → PGV → MMI; without Pd the intensity is unknown (0) and the S
    /// wave is taken as imminent (anchor = pick), the safe side.
    fn local_source(&self, t: Tick, local: &LocalDetection) -> Source {
        let cfg = self.cfg;
        let anchor = local.p_pick.or(self.shaking_start).unwrap_or(t);
        let m = tauc_magnitude(local.tau_c_s);
        let r = m.and_then(|m| self.wz.r_hyp_km(local.pd_m, m));
        let mmi = if local.pd_m > 0.0 {
            mmi_from_pgv(pgv_from_pd_cmps(local.pd_m))
        } else {
            0.0
        };
        let origin_offset_s = match r {
            Some(r) if cfg.v_p_kmps > 0.0 => -r / cfg.v_p_kmps,
            _ => 0.0,
        };
        let r_km = r.unwrap_or(0.0);
        Source {
            anchor,
            origin_offset_s,
            magnitude: m.unwrap_or(0.0),
            r_epi_km: r_km,
            depth_km: 0.0,
            mmi,
            sigma_s: cfg.local_eta_sigma_s,
            s_eta: if r.is_some() {
                wave_eta(anchor, origin_offset_s, r_km, 0.0, cfg.v_s_kmps)
            } else {
                Some(anchor)
            },
            p_eta: None,
        }
    }

    /// The fused estimate (primary max intensity, secondary capped log-odds).
    fn evaluate(&self, t: Tick, local: &LocalDetection) -> Option<Estimate> {
        let cfg = self.cfg;
        let mut best_remote: Option<Source> = None;
        let mut feeds = FeedSet::EMPTY;
        let mut independent = FeedSet::EMPTY;
        let mut injected = false;
        let mut offshore = false;
        let mut m_best = 0.0_f32;

        for f in self.feeds.iter().flatten() {
            if !is_trigger(f.kind) {
                continue;
            }
            let (m, pegs) = self.pegs_magnitude(f);
            let i_ipe = mmi_ipe(m, fmath::hypotf(f.r_epi_km, f.depth_km));
            let mmi = match (f.mmi > 0.0, pegs) {
                (true, true) => fmath::fmaxf(f.mmi, i_ipe),
                (true, false) => fmath::fminf(f.mmi, MMI_MAX),
                (false, _) => i_ipe,
            };
            feeds = feeds.with(f.kind);
            if pegs {
                feeds = feeds.with(AlertFeedKind::PegsMagnitudeAdvisory);
            }
            if f.injected {
                injected = true;
            } else {
                independent = independent.with(f.kind);
            }
            offshore |= f.kind == AlertFeedKind::OffshoreCabled;
            m_best = fmath::fmaxf(m_best, m);
            let src = Source {
                anchor: f.received,
                origin_offset_s: f.origin_offset_s,
                magnitude: m,
                r_epi_km: f.r_epi_km,
                depth_km: f.depth_km,
                mmi,
                sigma_s: cfg.s_eta_sigma_s,
                s_eta: wave_eta(
                    f.received,
                    f.origin_offset_s,
                    f.r_epi_km,
                    f.depth_km,
                    cfg.v_s_kmps,
                ),
                p_eta: wave_eta(
                    f.received,
                    f.origin_offset_s,
                    f.r_epi_km,
                    f.depth_km,
                    cfg.v_p_kmps,
                ),
            };
            // Ties keep the earlier slot (deterministic).
            if best_remote.is_none_or(|b| src.mmi > b.mmi) {
                best_remote = Some(src);
            }
        }
        let mut best = best_remote;

        let local_p = local.p_pick.is_some();
        if local_p || self.shaking_start.is_some() {
            let src = self.local_source(t, local);
            if local_p {
                feeds = feeds.with(AlertFeedKind::LocalP);
            }
            m_best = fmath::fmaxf(m_best, src.magnitude);
            if best.is_none_or(|b| src.mmi > b.mmi) {
                best = Some(src);
            }
        }
        let driver = best?;

        let p0 = if driver.mmi > 0.0 {
            p_exceed(driver.mmi, cfg.mmi_crit, cfg.mmi_sigma)
        } else {
            P_BASE_LOCAL
        };

        let mut terms = [(0.0_f32, 1.0_f32); 3];
        if let Some(r) = best_remote {
            let sigma = fmath::fmaxf(cfg.s_eta_sigma_s, 0.0);
            let overdue = r
                .p_eta
                .is_some_and(|tp| signed_s(t, tp) > NO_PICK_SIGMAS * sigma);
            if local_p {
                terms[0] = (1.0, LR_PICK);
            } else if overdue && self.shaking_start.is_none() {
                terms[0] = (1.0, LR_NO_PICK);
            }
        }
        if local.level >= SeismicLevel::L2 {
            terms[1] = (W_LEVEL, LR_LEVEL);
        }
        if local.pga_mps2 >= cfg.shaking_pga_mps2 {
            terms[2] = (1.0, LR_PGA);
        }
        let p = logistic(logit(p0) + capped_log_odds(&terms, cfg.log_odds_cap));

        let n = independent.count() + u32::from(local_p);
        Some(Estimate {
            driver,
            p,
            feeds,
            n_independent: u8::try_from(n).unwrap_or(u8::MAX),
            local_p,
            injected,
            m_best,
            offshore,
        })
    }
}

/// Fault-injection source of synthetic network feeds (never in firmware; FAB-EXO-307).
#[cfg(feature = "fault-injection")]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Injector {
    next_id: u32,
}

#[cfg(feature = "fault-injection")]
impl Injector {
    /// An injector starting at id 0x8000_0000 (distinct from scenario ids).
    pub const fn new() -> Injector {
        Injector {
            next_id: 0x8000_0000,
        }
    }

    /// An injected (spoof-test) network feed received at `t`.
    pub fn network(&mut self, t: Tick, magnitude: f32, r_epi_km: f32, depth_km: f32) -> Feed {
        let id = self.next_id;
        self.next_id = self.next_id.wrapping_add(1);
        Feed {
            kind: AlertFeedKind::Network,
            id,
            received: t,
            magnitude,
            r_epi_km,
            depth_km,
            injected: true,
            ..Feed::default()
        }
    }
}

/// Speed below which a device counts as stopped, m/s.
const V_STOPPED_MPS: f32 = 0.002;
/// A device within this of its closed position counts as closed, nm (1 mm).
const AT_CLOSED_NM: i64 = 1_000_000;

/// The exotic seam into the core supervisor (plan §4; FAB-EXO-102, 801): maps a sovereign
/// class's safe-state policy onto the core seismic action while the core state is not
/// NORMAL, using the latest fused alert for escalation, confirmation and deadlines.
///
/// | safe state | TRIGGERED | PROTECT / SHAKING | QUIET / INSPECT |
/// |---|---|---|---|
/// | RideThrough (SENTINEL) | RideThrough | RideThrough | RideThrough |
/// | CompleteClose (BASTION) | complete an in-progress close if feasible, else core | same, else Lock | Lock |
/// | CloseAfterShaking (TIDEGATE) | core (freeze) | Land | CloseAfterShaking on the external command, else Land |
/// | LandAndLock (SHIELD, ARENA, SELENE) | core | Lock (Land on egress) | Lock |
/// | BrakeAndLand (ROTATOR) | core | Land | Land |
/// | IsolateThenLandFps (HAVEN) | Isolate if enabled, confirmed and ClearVerified, else core | Isolate, else Land | same |
///
/// A fused level ≥ 2 escalates TRIGGERED to PROTECT. Every motion (CompleteClose,
/// CloseAfterShaking) first passes `fabrail_exo_safety::alert_gate::gate` as a closing
/// move to a strictly higher `SafetyRank` with independent confirmation (a local pick or
/// ≥ 2 non-injected feeds) and a `ClearVerified` zone; its speed cap never exceeds the
/// class v_max or, for a close in progress, the current speed (never speeding up). The
/// core `admit` filter still applies. `class: None` (core and pedestrian devices) never
/// overrides: the choreographer's `PlanPolicy` owns those.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FusionPolicy {
    /// Latest fused alert, if any.
    pub alert: Option<FusedAlert>,
    /// Sovereign class of the device (`None`: never overrides).
    pub class: Option<ExoClass>,
    /// Safe-state policy (from the class preset by [`FusionPolicy::for_class`]).
    pub safe_state: SafeStatePolicy,
    /// The guarded zone is `ClearVerified` (sovereign motion requires it).
    pub zone_clear_verified: bool,
    /// The closure-race planner decided CLOSE (BASTION; `closure::plan`).
    pub close_feasible: bool,
    /// The external decision system commanded the close (TIDEGATE).
    pub close_command: bool,
    /// Contactless isolation is enabled (research tier; false by default).
    pub isolation_enabled: bool,
}

impl FusionPolicy {
    /// A policy for sovereign class `c` with its preset safe state; zone, closure and
    /// isolation flags start false.
    pub const fn for_class(c: ExoClass) -> FusionPolicy {
        FusionPolicy {
            alert: None,
            class: Some(c),
            safe_state: presets::preset(c).safe_state,
            zone_clear_verified: false,
            close_feasible: false,
            close_command: false,
            isolation_enabled: false,
        }
    }

    /// Confirmations for the exotic gate: a local pick (core report or fused) or the fused
    /// independent-feed count (injected feeds never count; the core `n_feeds` cannot tell
    /// an injected link apart, so it is not used here).
    pub fn confirmations(&self, ctx: &ActionContext) -> Confirmations {
        let a = self.active_alert();
        Confirmations {
            local_p: ctx.report.local_p || a.is_some_and(|a| a.local_p),
            n_independent_feeds: a.map_or(0, |a| a.n_independent),
            zone_clear_verified: self.zone_clear_verified,
            obstruction_armed: false,
            within_low_energy_caps: false,
        }
    }

    /// The alert unless it is a cancel or below level 1.
    fn active_alert(&self) -> Option<FusedAlert> {
        self.alert
            .filter(|a| !a.alert.cancel && a.level > SeismicLevel::L0)
    }
}

/// Phase of the class action table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Triggered,
    Protect,
    Shaking,
    After,
}

/// The device's condition for the rank order.
fn condition_of(ctx: &ActionContext) -> DeviceCondition {
    let at_closed = ctx.x.diff_nm(ctx.x_closed).unsigned_abs() <= AT_CLOSED_NM.unsigned_abs();
    match ctx.lev {
        LevState::Seated => DeviceCondition::Seated,
        LevState::Landed | LevState::Manual => DeviceCondition::Landed,
        _ if fmath::absf(ctx.v_mps) > V_STOPPED_MPS => DeviceCondition::Moving,
        _ if at_closed => DeviceCondition::ClosedLevitated,
        _ => DeviceCondition::Hold,
    }
}

impl ActionPolicy for FusionPolicy {
    fn propose(&mut self, ctx: &ActionContext) -> Option<ActionDecision> {
        let class = self.class?;
        let alert = self.active_alert();
        let escalate = alert.is_some_and(|a| a.level >= SeismicLevel::L2);
        let phase = match ctx.seis {
            SeismicState::Normal => return None,
            SeismicState::Triggered if escalate => Phase::Protect,
            SeismicState::Triggered => Phase::Triggered,
            SeismicState::Protect => Phase::Protect,
            SeismicState::Shaking => Phase::Shaking,
            SeismicState::Quiet | SeismicState::Inspect => Phase::After,
        };
        let conf = self.confirmations(ctx);
        let close_move = AlertAction::Move {
            from: SafetyRank::of(condition_of(ctx), ctx.egress),
            to: SafetyRank::SeatedOrLocked,
            closing: true,
        };
        let may_close = gate(close_move, &conf, GateClass::Sovereign(class)).is_allowed();
        let v_max = presets::preset(class).kinematics.v_max_mmps as f32 / 1000.0;
        let lock_or_land = if ctx.egress == EgressRole::None {
            ActionDecision::of(SeismicAction::Lock)
        } else {
            ActionDecision::of(SeismicAction::Land)
        };
        let land = ActionDecision::of(SeismicAction::Land);

        match self.safe_state {
            // SENTINEL SEISMIC_PASSIVE: admissible on any alert (no motion target).
            SafeStatePolicy::RideThrough => Some(ActionDecision::of(SeismicAction::RideThrough)),
            SafeStatePolicy::LandAndLock => (phase != Phase::Triggered).then_some(lock_or_land),
            SafeStatePolicy::BrakeAndLand => (phase != Phase::Triggered).then_some(land),
            SafeStatePolicy::IsolateThenLandFps => {
                // Research tier: isolating moves the platform against the moat edges, so
                // it needs the pre-positioning gate (confirmation + ClearVerified;
                // FAB-EXO-707) as well as the enable.
                let may_isolate =
                    gate(AlertAction::PrePosition, &conf, GateClass::Sovereign(class)).is_allowed();
                if self.isolation_enabled && may_isolate {
                    Some(ActionDecision::of(SeismicAction::Isolate))
                } else {
                    (phase != Phase::Triggered).then_some(land)
                }
            }
            SafeStatePolicy::CompleteClose => {
                let closing = ctx.v_mps < -V_STOPPED_MPS && ctx.x.diff_nm(ctx.x_closed) > 0;
                if closing && self.close_feasible && may_close && phase != Phase::After {
                    Some(ActionDecision {
                        action: SeismicAction::CompleteClose,
                        target: Some(ctx.x_closed),
                        v_cap_mps: fmath::fminf(fmath::absf(ctx.v_mps), v_max),
                        deadline: alert.and_then(|a| s_eta_deadline(&a, DEADLINE_MARGIN_S)),
                    })
                } else {
                    (phase != Phase::Triggered).then_some(lock_or_land)
                }
            }
            SafeStatePolicy::CloseAfterShaking => match phase {
                Phase::Triggered => None,
                Phase::Protect | Phase::Shaking => Some(land),
                Phase::After if self.close_command && may_close => Some(ActionDecision {
                    action: SeismicAction::CloseAfterShaking,
                    target: Some(ctx.x_closed),
                    v_cap_mps: v_max,
                    deadline: alert.and_then(|a| a.tsunami_eta),
                }),
                Phase::After => Some(land),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_core::seismic::action_policy::{PolicyOutcome, RejectReason, admit, resolve};
    use fabrail_types::{AlertSource, PosNm, RailClass};

    const HZ: f64 = BASE_TICK_HZ as f64;

    fn tk(s: f64) -> Tick {
        Tick::from_seconds_f64(s)
    }

    fn net(kind: AlertFeedKind, id: u32, rx_s: f64, origin_s: f64, m: f32, r: f32, d: f32) -> Feed {
        Feed {
            kind,
            id,
            received: tk(rx_s),
            origin_offset_s: (origin_s - rx_s) as f32,
            magnitude: m,
            r_epi_km: r,
            depth_km: d,
            ..Feed::default()
        }
    }

    fn quiet() -> LocalDetection {
        LocalDetection::default()
    }

    /// Runs the fusion from `t0_s` to `t1_s` at 100 Hz, returning the last output.
    fn run(fu: &mut Fusion, t0_s: f64, t1_s: f64, local: &LocalDetection) -> Option<FusedAlert> {
        let mut out = None;
        let mut t = t0_s;
        while t <= t1_s + 1e-9 {
            out = fu.step(tk(t), local);
            t += 0.01;
        }
        out
    }

    /// Standard-normal CDF in f64 (test reference).
    fn phi(z: f64) -> f64 {
        0.5 * (1.0 + fmath::erf(z / core::f64::consts::SQRT_2))
    }

    fn logit64(p: f64) -> f64 {
        (p / (1.0 - p)).ln()
    }

    fn logistic64(x: f64) -> f64 {
        1.0 / (1.0 + (-x).exp())
    }

    fn ctx(seis: SeismicState) -> ActionContext {
        ActionContext {
            class: RailClass::Heavy,
            egress: EgressRole::None,
            seis,
            lev: LevState::Seismic,
            x: PosNm(3_000_000_000),
            x_closed: PosNm(0),
            default: ActionDecision::of(SeismicAction::Freeze),
            v_unattended_mps: 0.23,
            min_independent_feeds: 2,
            report: SeismicReport {
                state: seis,
                source: AlertSource::Network,
                n_feeds: 1,
                ..SeismicReport::default()
            },
            ..ActionContext::default()
        }
    }

    // req: FAB-EXO-801
    #[test]
    fn feed_kinds_and_injector_gating() {
        // The five kinds, in wire order (EXOTIC §3.8 FAB-EXO-801).
        assert_eq!(
            AlertFeedKind::ALL,
            &[
                AlertFeedKind::LocalP,
                AlertFeedKind::Network,
                AlertFeedKind::OffshoreCabled,
                AlertFeedKind::Das,
                AlertFeedKind::PegsMagnitudeAdvisory,
            ],
            "FAB-EXO-801: exactly {{LocalP, Network, OffshoreCabled, Das, PegsMagnitudeAdvisory}}"
        );
        // Only the three remote kinds trigger; the pick enters through LocalDetection.
        let triggers: [bool; 5] = [false, true, true, true, false];
        for (k, want) in AlertFeedKind::ALL.iter().zip(triggers) {
            assert_eq!(is_trigger(*k), want, "{k:?}");
        }
        let mut fu = Fusion::new(FusionCfg::default());
        let pushed_pick = Feed {
            kind: AlertFeedKind::LocalP,
            ..Feed::default()
        };
        assert!(!fu.push(&pushed_pick), "a pushed LocalP feed is refused");

        // The Injector is compiled only with `fault-injection` (source inspection; the
        // test module is excluded from the scan so its own literals do not count).
        let src = include_str!("fusion.rs");
        let body = src.split("#[cfg(test)]\nmod tests").next().unwrap_or("");
        let gate_attr = "#[cfg(feature = \"fault-injection\")]\n";
        for item in ["pub struct Injector", "impl Injector"] {
            let idx = body.find(item).expect("Injector item present");
            let before = &body[..idx];
            let attr_block_start = before.rfind(gate_attr).expect("cfg attribute precedes it");
            let between = &before[attr_block_start + gate_attr.len()..];
            assert!(
                between.lines().all(|l| l.trim_start().starts_with("#[")),
                "FAB-EXO-801: `{item}` must sit directly under {gate_attr:?}, got {between:?}"
            );
        }
        let manifest = include_str!("../Cargo.toml");
        assert!(
            manifest.contains("\nfault-injection = []"),
            "feature declared, empty"
        );
        assert!(
            !manifest
                .lines()
                .any(|l| l.trim_start().starts_with("default") && l.contains("fault-injection")),
            "FAB-EXO-801/307: fault-injection is never a default feature"
        );

        // Output: a synthetic core NetworkAlert whose S-ETA (PHYSICS §20:
        // received + origin_offset_s + R_hyp/V_s) equals the extension's S-ETA.
        let mut fu = Fusion::new(FusionCfg::default());
        assert!(fu.push(&net(AlertFeedKind::Network, 7, 20.0, 12.0, 7.0, 40.0, 10.0)));
        let a = fu
            .step(tk(20.0), &quiet())
            .expect("M7 at 40 km is an alert");
        assert_eq!(a.alert.id, FUSED_ID_BASE + 1, "synthetic id, event 1");
        assert!(!a.alert.cancel && a.non_local() && !a.injected);
        let core_eta_s = a.alert.received.seconds_f64()
            + f64::from(a.alert.origin_offset_s)
            + f64::from(a.alert.r_epi_km).hypot(f64::from(a.alert.depth_km)) / 3.5;
        let eta = a.s_eta.expect("S-ETA").seconds_f64();
        assert!(
            (eta - core_eta_s).abs() <= 1.0 / HZ + 1e-9,
            "core S-ETA from the synthetic alert {core_eta_s} vs extension {eta} (±1 tick)"
        );
        assert_eq!(
            core_alert(&a),
            Some(a.alert),
            "protect-level alerts reach the core link"
        );

        // … and it reaches the core only through the ActionPolicy hook.
        let mut pol = FusionPolicy::for_class(ExoClass::Sentinel);
        pol.alert = Some(a);
        let c = ctx(SeismicState::Protect);
        let (d, o) = resolve(&c, &mut pol);
        assert_eq!(o, PolicyOutcome::Admitted);
        assert_eq!(
            d.action,
            SeismicAction::RideThrough,
            "SENTINEL rides through"
        );

        #[cfg(feature = "fault-injection")]
        {
            // Injected feeds may trigger (bounded regret) but never confirm.
            let mut inj = Injector::new();
            let f = inj.network(tk(20.0), 7.5, 30.0, 10.0);
            assert!(f.injected && f.kind == AlertFeedKind::Network);
            let mut fu = Fusion::new(FusionCfg::default());
            assert!(fu.push(&f));
            let a = fu
                .step(tk(20.0), &quiet())
                .expect("an injected alert still freezes/lands");
            assert!(a.injected && a.non_local());
            assert_eq!(
                a.n_independent, 0,
                "injected feeds never count as independent"
            );
            let mut pol = FusionPolicy::for_class(ExoClass::Tidegate);
            pol.alert = Some(a);
            pol.zone_clear_verified = true;
            pol.close_command = true;
            let mut c = ctx(SeismicState::Quiet);
            c.lev = LevState::Landed;
            let p = pol.propose(&c).expect("TIDEGATE acts");
            assert_eq!(
                p.action,
                SeismicAction::Land,
                "no alert-triggered close on an injection"
            );
        }
    }

    // req: FAB-EXO-802
    #[test]
    fn max_intensity_eta_logodds_costloss() {
        let cfg = FusionCfg::default();
        let mut fu = Fusion::new(cfg);
        // Stated inputs: A Network M6.0 at 80 km, 10 km deep, no site MMI (IPE);
        // B OffshoreCabled M7.0 at 60 km, 20 km deep, provider MMI 6.4. Origin at 10 s.
        let a = net(AlertFeedKind::Network, 1, 16.0, 10.0, 6.0, 80.0, 10.0);
        let b = Feed {
            mmi: 6.4,
            ..net(
                AlertFeedKind::OffshoreCabled,
                2,
                15.0,
                10.0,
                7.0,
                60.0,
                20.0,
            )
        };
        assert!(fu.push(&a) && fu.push(&b));
        let i_a = mmi_ipe(6.0, 80.0_f32.hypot(10.0));
        let t = 16.0;
        let out = fu.step(tk(t), &quiet()).expect("alert");
        // Primary: the maximum predicted intensity drives the event.
        let i_max = i_a.max(6.4);
        assert_eq!(
            out.alert.mmi, i_max,
            "JMA-style max of {i_a} and 6.4 (exotic design §3.4)"
        );
        assert_eq!(out.feeds.count(), 2, "both remote kinds contribute");
        assert_eq!(out.n_independent, 2);
        // S-ETA of the driver ~ N(t0 + R_hyp/V_s, σ) (PHYSICS §20), analytic in f64.
        let (r, dpt) = if i_a > 6.4 {
            (80.0_f64, 10.0_f64)
        } else {
            (60.0, 20.0)
        };
        let mu = 10.0 + r.hypot(dpt) / 3.5;
        let got = out.s_eta.expect("S-ETA").seconds_f64();
        assert!(
            (got - mu).abs() <= 1.0 / HZ + 1e-9,
            "S-ETA {got} vs {mu} (±1 tick)"
        );
        assert_eq!(out.s_eta_sigma_s, 1.0, "σ from the configuration (1–2 s)");
        // Prior P0 = Φ((I − I_crit)/σ_I); no local evidence yet (before the P arrival).
        let p0 = phi((f64::from(i_max) - 6.0) / 0.7);
        assert!(
            (f64::from(out.p_damaging) - p0).abs() < 1e-5,
            "P0 {} vs Φ = {p0} (1e-5, f32 erf)",
            out.p_damaging
        );

        // Secondary: an on-site pick adds w·ln LR = ln 4.
        let pick = LocalDetection {
            p_pick: Some(tk(t)),
            ..quiet()
        };
        let out = fu.step(tk(t + 0.01), &pick).expect("alert");
        let want = logistic64(logit64(p0) + 4.0_f64.ln());
        assert!(
            (f64::from(out.p_damaging) - want).abs() < 1e-5,
            "pick LR 4: {} vs {want} (1e-5)",
            out.p_damaging
        );
        assert!(out.local_p && out.n_independent == 3 && out.feeds.contains(AlertFeedKind::LocalP));
        // The cap binds: pick (ln 4) + PGA ≥ 80 gal (ln 20) = 4.38 > 4.
        let strong = LocalDetection {
            pga_mps2: 1.0,
            ..pick
        };
        let out = fu.step(tk(t + 0.02), &strong).expect("alert");
        let want = logistic64(logit64(p0) + 4.0);
        assert!(
            (f64::from(out.p_damaging) - want).abs() < 1e-5,
            "capped at ±4 nats: {} vs {want} (1e-5)",
            out.p_damaging
        );
        assert_eq!(
            capped_log_odds(&[(1.0, 1e9), (1.0, 1e9)], 4.0),
            4.0,
            "cap is symmetric"
        );
        assert_eq!(capped_log_odds(&[(1.0, 1e-9)], 4.0), -4.0);

        // Cost–loss: act (PROTECT) iff P > C/L = 0.1; freeze iff P > C_f/L = 0.02.
        // Provider MMI chosen around the boundaries I = 6 + 0.7·Φ⁻¹(p).
        for (mmi, want_state, want_level) in [
            (5.12_f32, SeismicState::Protect, Some(SeismicLevel::L2)),
            (5.08, SeismicState::Triggered, Some(SeismicLevel::L1)),
            (4.40, SeismicState::Normal, None),
            (7.10, SeismicState::Protect, Some(SeismicLevel::L3)),
        ] {
            let p = phi((f64::from(mmi) - 6.0) / 0.7);
            let mut fu = Fusion::new(cfg);
            let f = Feed {
                mmi,
                ..net(AlertFeedKind::Network, 3, 16.0, 10.0, 6.5, 50.0, 10.0)
            };
            assert!(fu.push(&f));
            let out = fu.step(tk(16.0), &quiet());
            assert_eq!(
                fu.state(),
                want_state,
                "MMI {mmi}: P = {p:.4} vs C/L 0.1, C_f/L 0.02"
            );
            assert_eq!(out.map(|o| o.level), want_level, "MMI {mmi}");
            assert_eq!(
                out.and_then(|o| core_alert(&o)).is_some(),
                want_state == SeismicState::Protect,
                "only protect-level alerts enter the core link"
            );
        }
    }

    // req: FAB-EXO-805
    #[test]
    fn cancel_returns_normal() {
        let cfg = FusionCfg::default();
        // (1) Cancellation: M7 at 30 km → PROTECT; the cancel for its id → one cancel
        // message with the synthetic id, then NORMAL and silence.
        let mut fu = Fusion::new(cfg);
        let f = net(AlertFeedKind::Network, 11, 5.0, 2.0, 7.0, 30.0, 10.0);
        assert!(fu.push(&f));
        let a = fu.step(tk(5.0), &quiet()).expect("alert");
        assert_eq!(fu.state(), SeismicState::Protect);
        assert!(fu.push(&Feed { cancel: true, ..f }));
        let c = fu.step(tk(5.5), &quiet()).expect("one cancel message");
        assert!(
            c.alert.cancel && c.alert.id == a.alert.id,
            "cancel reuses the synthetic id"
        );
        assert_eq!(c.level, SeismicLevel::L0);
        assert_eq!(
            core_alert(&c),
            Some(c.alert),
            "the cancel reaches the core link"
        );
        assert_eq!(
            fu.state(),
            SeismicState::Normal,
            "FAB-EXO-805: cancel → NORMAL"
        );
        assert_eq!(fu.step(tk(5.51), &quiet()), None);
        assert!(
            !fu.push(&f),
            "a late re-send of a cancelled id never revives it"
        );
        assert_eq!(run(&mut fu, 5.52, 8.0, &quiet()), None);

        // (2) Concurrent-quake false alarm, downgrade: two simultaneous M5 events merged
        // by the network into one M7.3 at 40 km (the JMA 2011 mechanism). The update
        // (same id) corrects it to M4.5 → NORMAL.
        let mut fu = Fusion::new(cfg);
        let merged = net(AlertFeedKind::Network, 21, 4.0, 0.0, 7.3, 40.0, 10.0);
        assert!(fu.push(&merged));
        let a = fu
            .step(tk(4.0), &quiet())
            .expect("the false alarm acts: P > C/L");
        assert!(a.p_damaging > cfg.cost_loss && fu.state() == SeismicState::Protect);
        assert!(fu.push(&Feed {
            magnitude: 4.5,
            received: tk(6.0),
            origin_offset_s: -6.0,
            ..merged
        }));
        let c = fu.step(tk(6.0), &quiet()).expect("retraction message");
        assert!(c.alert.cancel);
        assert_eq!(fu.state(), SeismicState::Normal, "downgrade → NORMAL");

        // (3) Concurrent-quake false alarm, no update and no cancel: no pick 2σ after the
        // predicted P (LR 1/4), then a no-show by S-ETA + no_show_s → NORMAL.
        let mut fu = Fusion::new(cfg);
        assert!(fu.push(&merged));
        let mu_s = 0.0 + 40.0_f64.hypot(10.0) / 3.5;
        let t_p = 40.0_f64.hypot(10.0) / 6.0;
        let before = fu.step(tk(4.0), &quiet()).expect("alert").p_damaging;
        let after = run(&mut fu, 4.01, t_p + 2.5, &quiet())
            .expect("still active")
            .p_damaging;
        assert!(after < before, "missing pick lowers P ({before} → {after})");
        let last = run(&mut fu, t_p + 2.51, mu_s + 10.0 - 0.02, &quiet());
        assert!(
            last.is_some_and(|a| !a.alert.cancel),
            "held until the no-show time"
        );
        let mut t = mu_s + 10.0 - 0.01;
        let retract = loop {
            let o = fu.step(tk(t), &quiet());
            if fu.state() == SeismicState::Normal {
                break (t, o);
            }
            assert!(
                t < mu_s + 10.05,
                "no-show must fire by S-ETA + 10 s (one release)"
            );
            t += 0.01;
        };
        assert!(
            retract.0 <= mu_s + 10.0 + 0.011,
            "no-show at {} s vs S-ETA {mu_s:.3} + 10 s (one 10 ms release)",
            retract.0
        );
        assert!(
            retract.1.is_some_and(|c| c.alert.cancel),
            "one cancel message at the no-show"
        );
        assert_eq!(
            fu.step(tk(t + 0.01), &quiet()),
            None,
            "then silence (FAB-EXO-805)"
        );

        // (4) Local physics outranks a remote retraction: the site picked P and shook.
        let mut fu = Fusion::new(cfg);
        assert!(fu.push(&f));
        let shake = LocalDetection {
            p_pick: Some(tk(5.2)),
            level: SeismicLevel::L2,
            pd_m: 0.004,
            tau_c_s: 1.0,
            pga_mps2: 1.2,
        };
        fu.step(tk(5.3), &shake).expect("alert");
        assert_eq!(fu.state(), SeismicState::Shaking);
        assert!(fu.push(&Feed { cancel: true, ..f }));
        let a = fu
            .step(tk(5.4), &shake)
            .expect("still active on local evidence");
        assert!(!a.alert.cancel && !a.non_local(), "local-only now");
        assert_ne!(fu.state(), SeismicState::Normal);
        fu.reset();
        assert_eq!(
            fu.state(),
            SeismicState::Normal,
            "only the INSPECT reset ends it"
        );
        assert_eq!(fu.step(tk(6.0), &quiet()), None);
    }

    // req: FAB-EXO-810
    #[test]
    fn pegs_never_sole_trigger() {
        let cfg = FusionCfg::default();
        let pegs = Feed {
            mmi: 9.0, // even a PEGS message carrying an intensity is ignored
            ..net(
                AlertFeedKind::PegsMagnitudeAdvisory,
                90,
                60.0,
                0.0,
                9.0,
                300.0,
                20.0,
            )
        };
        let mut fu = Fusion::new(cfg);
        assert!(fu.push(&pegs));
        assert_eq!(
            run(&mut fu, 60.0, 90.0, &quiet()),
            None,
            "FAB-EXO-810: a PEGS advisory alone never triggers"
        );
        assert_eq!(fu.state(), SeismicState::Normal);

        // With a corroborating network alert (saturated M7.9 for the same origin) the
        // advisory supplies the unsaturated magnitude and the long-period flag.
        let netw = net(AlertFeedKind::Network, 91, 62.0, 0.0, 7.9, 300.0, 20.0);
        let mut plain = Fusion::new(cfg);
        assert!(plain.push(&netw));
        let base = plain.step(tk(62.0), &quiet());
        assert!(fu.push(&netw));
        let a = fu.step(tk(62.0), &quiet()).expect("network + PEGS acts");
        assert!(a.feeds.contains(AlertFeedKind::PegsMagnitudeAdvisory));
        assert_eq!(
            a.alert.magnitude, 9.0,
            "PEGS magnitude replaces the saturated one"
        );
        assert!(a.long_period, "Mw 9 far field is long-period");
        assert_eq!(
            a.n_independent, 1,
            "PEGS never counts as an independent feed"
        );
        let base_p = base.map_or(0.0, |b| b.p_damaging);
        assert!(
            a.p_damaging > base_p,
            "the advisory raises P ({base_p} → {})",
            a.p_damaging
        );
        let ipe = mmi_ipe(9.0, 300.0_f32.hypot(20.0));
        assert!(
            (a.alert.mmi - ipe).abs() < 1e-4,
            "PEGS provider MMI is ignored; the IPE at the upgraded M is used ({} vs {ipe})",
            a.alert.mmi
        );

        // An advisory for a different origin (outside the association window) is unused.
        let mut fu2 = Fusion::new(cfg);
        let far_origin = Feed {
            origin_offset_s: -200.0,
            ..pegs
        };
        // A second event (origin 50 s) near the site; its S wave is still to come.
        let near = net(AlertFeedKind::Network, 92, 62.0, 50.0, 7.9, 60.0, 20.0);
        assert!(fu2.push(&far_origin) && fu2.push(&near));
        let b = fu2
            .step(tk(62.0), &quiet())
            .expect("the near network alert alone");
        assert!(!b.feeds.contains(AlertFeedKind::PegsMagnitudeAdvisory));
        assert_eq!(
            b.alert.magnitude, 7.9,
            "unassociated advisory leaves M unchanged"
        );

        // Withdrawing the corroboration returns to NORMAL although PEGS remains.
        assert!(fu.push(&Feed {
            cancel: true,
            ..netw
        }));
        let c = fu.step(tk(63.0), &quiet()).expect("cancel message");
        assert!(c.alert.cancel);
        assert_eq!(run(&mut fu, 63.01, 80.0, &quiet()), None);
        assert_eq!(fu.state(), SeismicState::Normal, "PEGS alone again: NORMAL");
        assert!(
            fu.feeds()
                .any(|f| f.kind == AlertFeedKind::PegsMagnitudeAdvisory)
        );

        // Only the network advisory kind exists: no on-site gradiometer in Rust
        // (FRONTIER: numpy only).
        let src = include_str!("fusion.rs");
        let body = src.split("#[cfg(test)]\nmod tests").next().unwrap_or("");
        assert!(!body.contains("struct Gradiometer") && !body.contains("fn gradiometer"));
        assert_eq!(
            AlertFeedKind::ALL
                .iter()
                .filter(|k| k.name().to_ascii_lowercase().contains("pegs"))
                .count(),
            1
        );
    }

    #[test]
    fn intensity_models_reference_points() {
        // Atkinson & Wald 2007 (California) evaluated by hand in f64.
        let aw = |m: f64, d: f64| {
            let r = d.hypot(14.0);
            let lr = r.log10();
            let b = (r / 30.0).log10().max(0.0);
            12.27 + 2.270 * (m - 6.0) + 0.1304 * (m - 6.0).powi(2) - 1.30 * lr - 0.000_707 * r
                + 1.95 * b
                - 0.577 * m * lr
        };
        for (m, d) in [(7.0, 20.0), (5.0, 10.0), (7.2, 61.19), (8.0, 150.0)] {
            let got = f64::from(mmi_ipe(m as f32, d as f32));
            assert!(
                (got - aw(m, d)).abs() < 1e-4,
                "IPE M{m} D{d}: {got} vs {} (1e-4)",
                aw(m, d)
            );
        }
        // Villa warned case (plan §4b: M7.2, R_epi 60 km, 12 km deep) sits near MMI VI.
        let villa = mmi_ipe(7.2, 60.0_f32.hypot(12.0));
        assert!((5.5..6.5).contains(&villa), "villa MMI {villa}");
        // Monotone: larger M raises, larger R lowers.
        assert!(mmi_ipe(8.0, 200.0) > mmi_ipe(7.0, 200.0));
        assert!(mmi_ipe(7.0, 50.0) > mmi_ipe(7.0, 150.0));
        // Worden 2012 branches meet at log PGV = 0.53 (MMI ≈ 4.56, within 0.01).
        let lo: f64 = 3.78 + 1.47 * 0.53;
        let hi: f64 = 2.89 + 3.16 * 0.53;
        assert!((lo - hi).abs() < 0.01);
        let at = f64::from(mmi_from_pgv(10.0_f32.powf(0.53)));
        assert!((at - lo).abs() < 0.01, "GMICE at the knee {at}");
        // Wu & Kanamori 2008: Pd 0.35 cm (the core PROTECT Pd) → PGV 10^(0.92·log 0.35 + 1.642).
        let pgv = f64::from(pgv_from_pd_cmps(0.0035));
        let want = 10.0_f64.powf(0.920 * 0.35_f64.log10() + 1.642);
        assert!(
            (pgv / want - 1.0).abs() < 1e-5,
            "PGV {pgv} vs {want} (1e-5 rel)"
        );
        let i = mmi_from_pgv(pgv as f32);
        assert!(i > 6.0, "the core PROTECT Pd maps above I_crit (MMI {i})");
        assert_eq!(pgv_from_pd_cmps(0.0), 0.0);
        assert_eq!(mmi_ipe(f32::NAN, 10.0), 1.0);
        // Wu–Zhao inversion round-trips params::derive::pd_wu_zhao.
        let wz = WuZhao::new();
        let pd = derive::pd_wu_zhao(7.0, 20.0) as f32;
        let r = wz.r_hyp_km(pd, 7.0).expect("R");
        assert!(
            (r - 20.0).abs() < 1e-3,
            "Wu–Zhao inversion {r} km vs 20 km (1e-3)"
        );
    }

    #[test]
    fn eta_gain_per_km_and_blind_zone() {
        // Sensor-advance / site-distance gain at the lead-time anchors (EXOTIC §2: V_p 6.5,
        // V_s 3.7 → 0.15385 and 0.27027 s/km); f32 ticks resolve them to 1 tick per 100 km.
        let gain = |v: f32| {
            let a = wave_eta(Tick(0), 0.0, 100.0, 0.0, v).expect("eta");
            let b = wave_eta(Tick(0), 0.0, 200.0, 0.0, v).expect("eta");
            b.since_s(a) as f64 / 100.0
        };
        assert!(
            (gain(6.5) - 1.0 / 6.5).abs() <= 1.0 / HZ / 100.0 + 1e-9,
            "{}",
            gain(6.5)
        );
        assert!(
            (gain(3.7) - 1.0 / 3.7).abs() <= 1.0 / HZ / 100.0 + 1e-9,
            "{}",
            gain(3.7)
        );
        assert!(
            (1.0_f64 / 6.5 - 0.153_846).abs() < 1e-6 && (1.0_f64 / 3.7 - 0.270_270).abs() < 1e-6
        );
        assert_eq!(wave_eta(Tick(0), 0.0, 10.0, 0.0, 0.0), None);

        // Blind zone r_b = √((V_s·t_alert)² − h²): alert 5 s after origin, h = 10 km.
        let (v_s, t_alert, h) = (3.5_f32, 5.0_f32, 10.0_f32);
        let r_b = ((v_s * t_alert).powi(2) - h * h).sqrt();
        let cfg = FusionCfg::default();
        for (r, blind) in [(r_b - 1.0, true), (r_b + 1.0, false)] {
            let mut fu = Fusion::new(cfg);
            let f = Feed {
                mmi: 7.5,
                ..net(AlertFeedKind::Network, 5, 5.0, 0.0, 6.8, r, h)
            };
            assert!(fu.push(&f));
            let a = fu.step(tk(5.0), &quiet()).expect("alert");
            let w = signed_s(a.s_eta.expect("eta"), tk(5.0));
            assert_eq!(w < 0.0, blind, "r {r} km vs r_b {r_b} km: warning {w} s");
        }
    }

    #[test]
    fn offshore_m8_fused_warning_matches_analytic() {
        // Stated inputs: Mw 8.0 offshore, 20 km deep; site 150 km epicentral; an S-net-like
        // sensor above the source (R_sensor = 20 km) or a DAS cable 40 km from the
        // epicentre; latency 3.3 s (exotic design §2.8); V_p 6.5, V_s 3.7 (EXOTIC §2).
        let cfg = FusionCfg {
            v_p_kmps: 6.5,
            v_s_kmps: 3.7,
            ..FusionCfg::default()
        };
        let (r_site, depth, lat) = (150.0_f64, 20.0_f64, 3.3_f64);
        let origin = 100.0;
        for (kind, r_sensor) in [
            (AlertFeedKind::OffshoreCabled, 20.0_f64),
            (AlertFeedKind::Das, 40.0_f64.hypot(20.0)),
        ] {
            let t_alert = r_sensor / 6.5 + lat;
            let w_analytic = r_site.hypot(depth) / 3.7 - t_alert;
            let mut fu = Fusion::new(cfg);
            let f = net(
                kind,
                8,
                origin + t_alert,
                origin,
                8.0,
                r_site as f32,
                depth as f32,
            );
            assert!(fu.push(&f));
            let a = fu
                .step(tk(origin + t_alert), &quiet())
                .expect("M8 at 150 km alerts");
            let w = f64::from(signed_s(a.s_eta.expect("eta"), f.received));
            assert!(
                (w - w_analytic).abs() <= 1.0,
                "{kind:?}: fused warning {w} s vs analytic {w_analytic} s (±1 s, FAB-EXO-804 inputs)"
            );
        }
    }

    #[test]
    fn tsunami_eta_from_offshore_feed() {
        // c = √(g·h) = 198 m/s at a 4 km path depth (EXOTIC §X12 anchor).
        let cfg = FusionCfg {
            tsunami_path_depth_m: 4000.0,
            ..FusionCfg::default()
        };
        let c = (derive::G0_MPS2 * 4000.0).sqrt();
        assert!((c - 198.06).abs() < 0.01, "c {c}");
        let mut fu = Fusion::new(cfg);
        let f = net(
            AlertFeedKind::OffshoreCabled,
            4,
            10.0,
            0.0,
            8.0,
            150.0,
            20.0,
        );
        assert!(fu.push(&f));
        let a = fu.step(tk(10.0), &quiet()).expect("alert");
        let got = a.tsunami_eta.expect("tsunami ETA").seconds_f64();
        let want = 150_000.0 / c;
        assert!(
            (got - want).abs() < 0.01,
            "tsunami ETA {got} s vs {want} s (10 ms, f32)"
        );
        // Land network only, deep, or small: no tsunami ETA.
        for g in [
            net(AlertFeedKind::Network, 5, 10.0, 0.0, 8.0, 150.0, 20.0),
            net(
                AlertFeedKind::OffshoreCabled,
                6,
                10.0,
                0.0,
                8.0,
                150.0,
                300.0,
            ),
            Feed {
                mmi: 7.0,
                ..net(AlertFeedKind::OffshoreCabled, 7, 10.0, 0.0, 6.5, 30.0, 10.0)
            },
        ] {
            let mut fu = Fusion::new(cfg);
            assert!(fu.push(&g));
            let a = fu.step(tk(10.0), &quiet());
            assert_eq!(a.and_then(|a| a.tsunami_eta), None, "{g:?}");
        }
    }

    #[test]
    fn local_only_event_and_shaking_states() {
        let cfg = FusionCfg::default();
        let mut fu = Fusion::new(cfg);
        // A pick without Pd yet: base prior 0.01 < C_f/L → no fused event.
        let pick = LocalDetection {
            p_pick: Some(tk(1.0)),
            ..quiet()
        };
        assert_eq!(fu.step(tk(1.0), &pick), None);
        // Pd 0.35 cm, τc 1 s: on-site MMI > 6 → PROTECT from the site alone.
        let pd = LocalDetection {
            pd_m: 0.0035,
            tau_c_s: 1.0,
            level: SeismicLevel::L2,
            ..pick
        };
        let a = fu.step(tk(2.0), &pd).expect("local PROTECT");
        assert_eq!(fu.state(), SeismicState::Protect);
        assert!(!a.non_local() && a.feeds == FeedSet::EMPTY.with(AlertFeedKind::LocalP));
        assert_eq!(
            core_alert(&a),
            None,
            "a local-only event is the core's own detection"
        );
        let eta = a.s_eta.expect("local S-ETA");
        assert!(eta > tk(1.0), "S−P from the Wu–Zhao distance is positive");
        assert_eq!(a.s_eta_sigma_s, cfg.local_eta_sigma_s);
        // Strong motion → SHAKING; no new peak for 2 s → QUIET; again → SHAKING.
        let mut s = pd;
        for (i, pga) in [0.9_f32, 1.1, 1.3].iter().enumerate() {
            s.pga_mps2 = *pga;
            fu.step(tk(3.0 + 0.01 * i as f64), &s);
        }
        assert_eq!(fu.state(), SeismicState::Shaking);
        run(&mut fu, 3.1, 5.2, &s);
        assert_eq!(fu.state(), SeismicState::Quiet, "no new peak for 2 s");
        s.pga_mps2 = 1.4;
        fu.step(tk(5.3), &s);
        assert_eq!(fu.state(), SeismicState::Shaking);
        // The deadline is the S-ETA 5 % quantile minus 1 s (plan §4b).
        let fa = FusedAlert {
            s_eta: Some(tk(20.0)),
            s_eta_sigma_s: 1.5,
            ..FusedAlert::default()
        };
        let dl = s_eta_deadline(&fa, DEADLINE_MARGIN_S).expect("deadline");
        let want = 20.0 - 1.644_853_6 * 1.5 - 1.0;
        assert!(
            (dl.seconds_f64() - want).abs() <= 1.0 / HZ + 1e-9,
            "deadline {dl:?} vs {want} s"
        );
        let early = s_eta_deadline(&a, DEADLINE_MARGIN_S).expect("deadline");
        assert_eq!(
            early,
            Tick(0),
            "a deadline before tick 0 saturates (near-field)"
        );
        assert_eq!(s_eta_deadline(&FusedAlert::default(), 1.0), None);
        // from_report takes the pick only when Z/H-confirmed.
        let r = SeismicReport {
            triggered_at: Some(tk(1.0)),
            local_p: false,
            pga_mps2: 0.5,
            ..SeismicReport::default()
        };
        assert_eq!(LocalDetection::from_report(&r).p_pick, None);
        let r = SeismicReport { local_p: true, ..r };
        assert_eq!(LocalDetection::from_report(&r).p_pick, Some(tk(1.0)));
        let p = FusionCfg::from_params(&fabrail_params::presets::door());
        assert_eq!(
            (p.v_p_kmps, p.v_s_kmps, p.shaking_pga_mps2),
            (6.0, 3.5, 0.8)
        );
    }

    #[test]
    fn push_validation_and_capacity() {
        let mut fu = Fusion::new(FusionCfg::default());
        let bad = [
            Feed {
                r_epi_km: -1.0,
                ..net(AlertFeedKind::Network, 1, 1.0, 0.0, 6.0, 10.0, 5.0)
            },
            Feed {
                magnitude: f32::NAN,
                ..net(AlertFeedKind::Network, 1, 1.0, 0.0, 6.0, 10.0, 5.0)
            },
            Feed {
                origin_offset_s: f32::INFINITY,
                ..net(AlertFeedKind::Das, 1, 1.0, 0.0, 6.0, 10.0, 5.0)
            },
        ];
        for f in bad {
            assert!(!fu.push(&f), "{f:?} is refused");
        }
        for i in 0..MAX_FEEDS as u32 {
            assert!(fu.push(&net(
                AlertFeedKind::PegsMagnitudeAdvisory,
                i,
                1.0,
                0.0,
                8.0,
                100.0,
                10.0
            )));
        }
        assert!(!fu.push(&net(
            AlertFeedKind::PegsMagnitudeAdvisory,
            99,
            1.0,
            0.0,
            8.0,
            100.0,
            10.0
        )));
        assert!(
            fu.push(&net(AlertFeedKind::Network, 100, 2.0, 0.0, 7.0, 30.0, 10.0)),
            "a trigger evicts the oldest advisory"
        );
        assert!(fu.feeds().any(|f| f.id == 100));
    }

    #[test]
    fn exotic_safe_states_through_the_hook() {
        let mut fu = Fusion::new(FusionCfg::default());
        assert!(fu.push(&net(
            AlertFeedKind::OffshoreCabled,
            1,
            5.0,
            0.0,
            8.0,
            60.0,
            20.0
        )));
        assert!(fu.push(&net(AlertFeedKind::Das, 2, 5.5, 0.0, 8.0, 60.0, 20.0)));
        let two_feeds = fu.step(tk(5.5), &quiet()).expect("alert");
        assert_eq!(two_feeds.n_independent, 2);
        let mut fu1 = Fusion::new(FusionCfg::default());
        assert!(fu1.push(&net(AlertFeedKind::Network, 3, 5.0, 0.0, 8.0, 60.0, 20.0)));
        let one_feed = fu1.step(tk(5.0), &quiet()).expect("alert");

        // No class: never overrides (the choreographer owns core devices).
        let mut none = FusionPolicy {
            alert: Some(two_feeds),
            ..FusionPolicy::default()
        };
        assert_eq!(none.propose(&ctx(SeismicState::Protect)), None);

        // SENTINEL: RideThrough in every non-NORMAL phase, even network-only.
        let mut s = FusionPolicy::for_class(ExoClass::Sentinel);
        s.alert = Some(one_feed);
        for st in [
            SeismicState::Triggered,
            SeismicState::Shaking,
            SeismicState::Quiet,
        ] {
            let (d, o) = resolve(&ctx(st), &mut s);
            assert_eq!(
                (d.action, o),
                (SeismicAction::RideThrough, PolicyOutcome::Admitted)
            );
        }
        assert_eq!(s.propose(&ctx(SeismicState::Normal)), None);

        // TIDEGATE: land in PROTECT/SHAKING; close only after shaking, on the external
        // command, confirmed, with the zone ClearVerified; the deadline is the tsunami ETA.
        let mut tg = FusionPolicy::for_class(ExoClass::Tidegate);
        assert_eq!(tg.safe_state, SafeStatePolicy::CloseAfterShaking);
        tg.alert = Some(two_feeds);
        tg.close_command = true;
        tg.zone_clear_verified = true;
        let mut c = ctx(SeismicState::Shaking);
        c.lev = LevState::Landed;
        // The integrator reports the fused independent feeds to the core (see issues).
        c.report.n_feeds = 2;
        assert_eq!(
            tg.propose(&c).map(|d| d.action),
            Some(SeismicAction::Land),
            "never during shaking"
        );
        c.seis = SeismicState::Quiet;
        let (d, o) = resolve(&c, &mut tg);
        assert_eq!(o, PolicyOutcome::Admitted);
        assert_eq!(d.action, SeismicAction::CloseAfterShaking);
        assert_eq!(d.target, Some(PosNm(0)));
        assert!(d.v_cap_mps <= 0.23, "admit clamps to the unattended limit");
        let mut no_cmd = tg;
        no_cmd.close_command = false;
        assert_eq!(
            no_cmd.propose(&c).map(|d| d.action),
            Some(SeismicAction::Land)
        );
        let mut no_zone = tg;
        no_zone.zone_clear_verified = false;
        assert_eq!(
            no_zone.propose(&c).map(|d| d.action),
            Some(SeismicAction::Land)
        );
        let mut one = tg;
        one.alert = Some(one_feed);
        assert_eq!(
            one.propose(&c).map(|d| d.action),
            Some(SeismicAction::Land),
            "one feed without a local pick is not independent confirmation"
        );
        assert_eq!(
            tg.propose(&ctx(SeismicState::Triggered)).map(|d| d.action),
            Some(SeismicAction::Land),
            "a level ≥ 2 fused alert escalates TRIGGERED to PROTECT"
        );

        // BASTION: complete an in-progress, feasible close; otherwise land and lock.
        let mut b = FusionPolicy::for_class(ExoClass::Bastion);
        b.alert = Some(two_feeds);
        b.zone_clear_verified = true;
        b.close_feasible = true;
        let mut c = ctx(SeismicState::Protect);
        c.v_mps = -0.4;
        c.report.local_p = true;
        let (d, o) = resolve(&c, &mut b);
        assert_eq!(
            (d.action, o),
            (SeismicAction::CompleteClose, PolicyOutcome::Admitted)
        );
        assert!(d.v_cap_mps <= 0.4, "never speeds up");
        assert!(d.deadline.is_some(), "S-ETA 5 % quantile − 1 s");
        c.v_mps = 0.0;
        assert_eq!(
            b.propose(&c).map(|d| d.action),
            Some(SeismicAction::Lock),
            "not closing"
        );
        c.egress = EgressRole::NonRated;
        assert_eq!(
            b.propose(&c).map(|d| d.action),
            Some(SeismicAction::Land),
            "egress never locks"
        );

        // SHIELD lock, ROTATOR land, HAVEN land while isolation (research) is disabled.
        let c = ctx(SeismicState::Protect);
        for (x, want) in [
            (ExoClass::Shield, SeismicAction::Lock),
            (ExoClass::Rotator, SeismicAction::Land),
            (ExoClass::HavenR, SeismicAction::Land),
        ] {
            let mut p = FusionPolicy::for_class(x);
            p.alert = Some(one_feed);
            assert_eq!(resolve(&c, &mut p).0.action, want, "{x:?}");
        }
        let mut h = FusionPolicy::for_class(ExoClass::HavenR);
        h.isolation_enabled = true;
        h.zone_clear_verified = true;
        h.alert = Some(one_feed);
        assert_eq!(
            h.propose(&c).map(|d| d.action),
            Some(SeismicAction::Land),
            "unconfirmed"
        );
        h.alert = Some(two_feeds);
        h.zone_clear_verified = false;
        assert_eq!(
            h.propose(&c).map(|d| d.action),
            Some(SeismicAction::Land),
            "moat not clear"
        );
        h.zone_clear_verified = true;
        let p = h.propose(&c);
        assert_eq!(
            p.map(|d| d.action),
            Some(SeismicAction::Isolate),
            "fused 2 feeds confirm"
        );
        let (d, o) = admit(&c, p);
        assert_eq!(
            (d, o),
            (
                c.default,
                PolicyOutcome::Rejected(RejectReason::Unconfirmed)
            ),
            "core n_feeds 1: the core backstop still applies"
        );
        let mut c2 = c;
        c2.report.local_p = true;
        assert_eq!(admit(&c2, h.propose(&c2)).1, PolicyOutcome::Admitted);

        // A cancel message carries no confirmation.
        let mut cancel = two_feeds;
        cancel.alert.cancel = true;
        tg.alert = Some(cancel);
        let mut c = ctx(SeismicState::Quiet);
        c.lev = LevState::Landed;
        assert_eq!(tg.propose(&c).map(|d| d.action), Some(SeismicAction::Land));
    }
}
