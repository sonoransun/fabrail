//! Online KPI accumulators at the base rate. Owner: A7 (KPI names frozen by W0-B3).
//!
//! KPI names (core design §B.6) are consumed by the Python anchors
//! (`tests/data/anchors_seed42.json`), so they are frozen here; A7 may add KPIs but
//! never rename these.
//!
//! Conventions:
//!
//! * physical KPIs are measured on plant truth (the controller never sees truth);
//!   controller KPIs (governor energy, passivity, intent, levels) on the executive's
//!   latest published outputs;
//! * a KPI whose phase never happened in the run (no lift, no hand, no terminus, …) is
//!   ABSENT from the map, never NaN, so KPI maps serialise as strict JSON; the
//!   accumulator KPIs (maxima, counts, `level_max`, …) are always present;
//! * `metrics.windows` in the scenario adds a copy of every KPI restricted to the
//!   window, named `"<window>.<kpi>"`; cumulative KPIs (bumper impacts/energy) count
//!   only what happened inside the window.
//!
//! Definitions (PHYSICS § in brackets; thresholds are KPI definitions, not tunables):
//!
//! | KPI | Definition |
//! |---|---|
//! | `breakaway_n` | peak \|F_hand\| from hand contact until the body first exceeds 1 mm/s [§17] |
//! | `apparent_mass_err` | \|M − M_v\|/M_v, M from the least-squares fit F_hand = M·a + B·v over hand contact in `Admittance` [§17] |
//! | `soft_close_v_final` | \|v\| when the body first comes within 0.5 mm of the closed stop on its last approach [§16] |
//! | `overshoot_m` | largest travel past a motion segment's end point in its direction [§16] |
//! | `detent_peak_err` | \|peak(F_hand − M_v·a − B_v·v) − F_detent\|/F_detent inside detent zones [§17] |
//! | `passivity_min_j` | minimum passivity-observer energy [§17] |
//! | `force_noise_rms_n` | rms of the LSM thrust about its mean while levitated idle without a hand [§13, §17] |
//! | `ke_max_gov_j` | maximum governor energy ½·m_gov·v² [§16] |
//! | `momentum_max` | maximum \|m·v\| (truth), kg·m/s [§16] |
//! | `contact_force_max_n` | maximum obstacle (ISO/TS 15066 body) contact force [§22] |
//! | `guide_force_max_n` | maximum guide-fin or side-pad force [§6] |
//! | `gap_dev_max_m` | maximum \|g_i − g_zp(m)\| over lift elements while levitated idle or moving [§3, §15] |
//! | `coil_i_ss_a` | mean \|i\| over active elements in the last 0.25 s while levitated [§15] |
//! | `hover_power_w` | coil + LSM copper power while levitated idle [§14] |
//! | `lift_time_s` / `unseat_time_s` / `land_time_s` | directive Lift / Unseat / Land-or-Seat → truth levitated / out of the seat / landed-or-seated [§15] |
//! | `touchdown_v_mps` | heave speed at touchdown [§8, §15] |
//! | `trigger_t_s` / `freeze_t_s` | first STA/LTA trigger / first fast-group Freeze [§20] |
//! | `level_max` | highest detector level [§20] |
//! | `protect_margin_to_s_s` | S-ETA − time PROTECT was first reported [§20] |
//! | `stroke_max_m` | maximum \|x − x₀\|, x₀ = position at the first trigger (or t = 0) [§21] |
//! | `bumper_impacts` / `bumper_energy_j` | bumper contact onsets / largest single impact energy [§8] |
//! | `manual_force_n` | peak \|F_hand\| while the body is not levitated [§18] |
//! | `intent_onset_s` / `intent_confirm_s` | hand contact → first pending / confirmed intent flag [§17] |
//! | `reverse_latency_s` | first obstacle contact → first `Reverse` mode [§18] |
//! | `mu_eq` | ledger drag energy / (m·g₀·distance travelled) [§9] |
//!
//! Additions: `bumper_energy_total_j`, `upper_force_max_n`, `roller_force_max_n`,
//! `p_bus_max_w`, `e_hold_min_j`, `hover_gap_mean_m`, `hover_gap_rms_m`; terminus
//! (FAB-HAP-013…016, measured on truth while |a_floor,x| < 0.01 g): `terminus_count`,
//! `terminus_r2_min` (semilog fit of the distance-to-go over the exponential capture),
//! `terminus_tau_fit_s`, `terminus_overshoot_max_m`, `terminus_v_arrive_max_mps`,
//! `terminus_t_arrive_s` and `terminus_t_arrive_cf_s` (measured from the blend start
//! to d ≤ 0.02 mm, and the closed form t_b + τ·ln(d_b/d_ε) + T_q, of the last
//! terminus), `terminus_t_arrive_err_max`; inertial smoothing (FAB-HAP-017…019):
//! `inertial_acc_rms_mps2`, `inertial_floor_rms_mps2`, `inertial_ratio`,
//! `inertial_stroke_max_m`.

use std::collections::BTreeMap;

use fabrail_core::msg::IntentFlag;
use fabrail_core::msg::LevDirective;
use fabrail_params::SystemParams;
use fabrail_params::derive::{self, G0_MPS2};
use fabrail_plant::Truth;
use fabrail_rt::ExecSnapshot;
use fabrail_types::{LevState, MAX_MAGNETS, MotionMode, Segment, TravelDir, fmath};

use crate::recorder::drag_energy_j;

/// Frozen KPI names (core design §B.6).
pub const KPI_NAMES: &[&str] = &[
    "breakaway_n",
    "apparent_mass_err",
    "soft_close_v_final",
    "overshoot_m",
    "detent_peak_err",
    "passivity_min_j",
    "force_noise_rms_n",
    "ke_max_gov_j",
    "momentum_max",
    "contact_force_max_n",
    "guide_force_max_n",
    "gap_dev_max_m",
    "coil_i_ss_a",
    "hover_power_w",
    "lift_time_s",
    "unseat_time_s",
    "land_time_s",
    "touchdown_v_mps",
    "trigger_t_s",
    "freeze_t_s",
    "level_max",
    "protect_margin_to_s_s",
    "stroke_max_m",
    "bumper_impacts",
    "bumper_energy_j",
    "manual_force_n",
    "intent_onset_s",
    "intent_confirm_s",
    "reverse_latency_s",
    "mu_eq",
];

/// KPIs added by A7 beyond the frozen core list (terminus, inertial, power and
/// contact extras; see the module docs).
pub const KPI_EXTRA_NAMES: &[&str] = &[
    "bumper_energy_total_j",
    "upper_force_max_n",
    "roller_force_max_n",
    "p_bus_max_w",
    "e_hold_min_j",
    "hover_gap_mean_m",
    "hover_gap_rms_m",
    "terminus_count",
    "terminus_r2_min",
    "terminus_tau_fit_s",
    "terminus_overshoot_max_m",
    "terminus_v_arrive_max_mps",
    "terminus_t_arrive_s",
    "terminus_t_arrive_cf_s",
    "terminus_t_arrive_err_max",
    "inertial_acc_rms_mps2",
    "inertial_floor_rms_mps2",
    "inertial_ratio",
    "inertial_stroke_max_m",
];

/// Speed above which the body counts as moving (breakaway onset), m/s.
const V_ONSET_MPS: f64 = 1.0e-3;
/// Distance to the closed stop at which the soft-close final speed is taken, m.
const SOFT_CLOSE_ZONE_M: f64 = 0.5e-3;
/// Window before the end of the run for the steady coil current, s.
const COIL_SS_WINDOW_S: f64 = 0.25;
/// Terminus arrival tolerance on the distance-to-go (the HAP-015 overshoot bound,
/// plan §4b: x − x_term never crosses zero by more than 0.02 mm), m.
///
/// W1.5: the closed form t_arrive = t_b + τ·ln(d_b/d_ε) + T_q is the instant the
/// reference reaches the target, while the quintic's tail d = d_ε·(1 − σ)³·(1 + 2σ +
/// 3.5σ²) (T_q = τ) passes 20 µm 0.25·T_q ≈ 3.8 % of t_arrive earlier. The ±2 % check
/// (`terminus_t_arrive_err_max`) therefore compares the measured crossing with the
/// reference segment's own crossing of the same tolerance (evaluated with
/// `motion::terminus::eval`), falling back to the closed form only when the reference
/// is unavailable; `terminus_t_arrive_cf_s` still reports the closed form.
const TERMINUS_ARRIVE_TOL_M: f64 = 0.02e-3;
/// Floor-acceleration bound of the terminus criteria ("quiet", plan §4b: only while
/// |a_floor,x| < 0.01 g), m/s².
const TERMINUS_QUIET_MPS2: f64 = 0.01 * G0_MPS2;

/// Run constants the accumulators need.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MetricsCtx {
    /// Nominal zero-power mass m*, kg.
    pub m_star_kg: f64,
    /// Zero-power gap g* at m*, m.
    pub g_star_m: f64,
    /// Magnet reluctance length h′ = h_m/μ_rec, m.
    pub h_prime_m: f64,
    /// Active elements.
    pub n_magnets: usize,
    /// Lift elements (PM-biased, attracting up).
    pub lift_mask: [bool; MAX_MAGNETS],
    /// Closed stop, m.
    pub x_closed_m: f64,
    /// Run length, s.
    pub duration_s: f64,
}

impl MetricsCtx {
    /// Constants for parameter set `p` and a run of `duration_s`.
    pub fn new(p: &SystemParams, duration_s: f64) -> MetricsCtx {
        let s = &p.design.suspension;
        let n = (s.n_magnets as usize).min(MAX_MAGNETS);
        let mut lift_mask = [false; MAX_MAGNETS];
        for (i, m) in lift_mask.iter_mut().enumerate().take(n) {
            let st = &s.stations[i];
            *m = st.has_pm && st.normal_sign > 0.0;
        }
        MetricsCtx {
            m_star_kg: p.design.carriage.m_star_kg,
            g_star_m: s.g_star_m,
            h_prime_m: derive::h_prime(s.h_m_m, s.mu_rec),
            n_magnets: n,
            lift_mask,
            x_closed_m: p.design.rail.x_closed_m,
            duration_s,
        }
    }
}

/// One base-rate observation (after the plant step).
#[derive(Clone, Copy, Debug)]
pub struct Observation<'a> {
    /// Time of the truth, s.
    pub t_s: f64,
    /// Plant truth.
    pub truth: &'a Truth,
    /// Executive outputs.
    pub snap: &'a ExecSnapshot,
    /// Start time of the hand contact in progress, if any.
    pub hand_since: Option<f64>,
    /// Floor acceleration along x at `t_s`, m/s².
    pub floor_x_mps2: f64,
}

/// Welford mean/variance.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Welford {
    n: u64,
    mean: f64,
    m2: f64,
}

impl Welford {
    fn push(&mut self, x: f64) {
        self.n += 1;
        let d = x - self.mean;
        self.mean += d / self.n as f64;
        self.m2 += d * (x - self.mean);
    }
    fn rms_dev(&self) -> Option<f64> {
        (self.n >= 2).then(|| fmath::sqrt(self.m2 / self.n as f64))
    }
    fn mean(&self) -> Option<f64> {
        (self.n >= 1).then_some(self.mean)
    }
}

/// Running root-mean-square.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Rms {
    n: u64,
    sum_sq: f64,
}

impl Rms {
    fn push(&mut self, x: f64) {
        self.n += 1;
        self.sum_sq += x * x;
    }
    fn get(&self) -> Option<f64> {
        (self.n >= 1).then(|| fmath::sqrt(self.sum_sq / self.n as f64))
    }
}

/// Online simple linear regression y = α + β·t (co-moments; R² and slope).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct LinFit {
    n: u64,
    mt: f64,
    my: f64,
    ctt: f64,
    cyy: f64,
    cty: f64,
}

impl LinFit {
    fn push(&mut self, t: f64, y: f64) {
        self.n += 1;
        let k = self.n as f64;
        let dt = t - self.mt;
        let dy = y - self.my;
        self.mt += dt / k;
        self.my += dy / k;
        self.ctt += dt * (t - self.mt);
        self.cyy += dy * (y - self.my);
        self.cty += dt * (y - self.my);
    }
    fn r2(&self) -> Option<f64> {
        (self.n >= 3 && self.ctt > 0.0 && self.cyy > 0.0)
            .then(|| self.cty * self.cty / (self.ctt * self.cyy))
    }
    fn slope(&self) -> Option<f64> {
        (self.n >= 2 && self.ctt > 0.0).then(|| self.cty / self.ctt)
    }
}

/// One log-damped terminus being measured.
#[derive(Clone, Copy, Debug, PartialEq)]
struct TerminusTrack {
    t0_s: f64,
    target_m: f64,
    sign: f64,
    t_b_s: f64,
    d_eps_m: f64,
    t_cf_s: Option<f64>,
    fit: LinFit,
    overshoot_m: f64,
    arrive: Option<(f64, f64)>,
    /// The reference segment's own arrival within the tolerance, s after t0.
    ref_arrive: Option<f64>,
}

/// Every accumulator of one KPI set.
#[derive(Clone, Debug, Default, PartialEq)]
struct Acc {
    windowed: bool,
    seen: bool,
    // hand
    hand_t0: Option<f64>,
    hand_peak_pre_onset: f64,
    breakaway: Option<f64>,
    moved_since_hand: bool,
    am: [f64; 5],
    am_n: u64,
    m_v: f64,
    b_v: f64,
    detent_peak: Option<f64>,
    detent_f: f64,
    manual_force: Option<f64>,
    intent_onset: Option<f64>,
    intent_confirm: Option<f64>,
    // motion
    soft_close_v: Option<f64>,
    far_from_closed: bool,
    overshoot: Option<f64>,
    passivity_min: f64,
    ke_gov_max: f64,
    momentum_max: f64,
    contact_max: f64,
    guide_max: f64,
    upper_max: f64,
    roller_max: f64,
    force_noise: Welford,
    // levitation
    gap_dev_max: Option<f64>,
    coil_ss: Welford,
    hover_e_j: f64,
    hover_t_s: f64,
    hover_gap: Welford,
    prev_copper_j: Option<f64>,
    prev_lev_idle: bool,
    prev_t: Option<f64>,
    prev_dir: Option<u8>,
    lift_start: Option<f64>,
    unseat_start: Option<f64>,
    land_start: Option<f64>,
    lift_time: Option<f64>,
    unseat_time: Option<f64>,
    land_time: Option<f64>,
    touchdown_v: Option<f64>,
    prev_z: Option<(f64, f64)>,
    v_z: f64,
    prev_contact: Option<(bool, bool, bool)>,
    // seismic
    trigger_t: Option<f64>,
    freeze_t: Option<f64>,
    protect_margin: Option<f64>,
    level_max: u8,
    stroke_base: Option<f64>,
    stroke_max: f64,
    // obstruction
    obstacle_t: Option<f64>,
    reverse_latency: Option<f64>,
    // cumulative
    bumper_base: Option<(u32, f64)>,
    bumper_impacts: u32,
    bumper_total_j: f64,
    bumper_e_max: f64,
    drag_base: Option<f64>,
    drag_j: f64,
    distance_m: f64,
    prev_x: Option<f64>,
    mass_kg: f64,
    p_bus_max: f64,
    e_hold_min: Option<f64>,
    // terminus
    term: Option<TerminusTrack>,
    term_count: u32,
    term_r2_min: Option<f64>,
    term_tau: Option<f64>,
    term_over_max: Option<f64>,
    term_v_max: Option<f64>,
    term_t_last: Option<(f64, Option<f64>)>,
    term_err_max: Option<f64>,
    // inertial
    inertial_acc: Rms,
    inertial_floor: Rms,
    inertial_x0: Option<f64>,
    inertial_stroke: Option<f64>,
}

fn max_opt(a: Option<f64>, b: f64) -> Option<f64> {
    Some(a.map_or(b, |x| x.max(b)))
}

fn min_opt(a: Option<f64>, b: f64) -> Option<f64> {
    Some(a.map_or(b, |x| x.min(b)))
}

impl Acc {
    fn new(windowed: bool) -> Acc {
        Acc {
            windowed,
            passivity_min: f64::INFINITY,
            ..Acc::default()
        }
    }

    #[allow(clippy::too_many_lines)]
    fn observe(&mut self, ctx: &MetricsCtx, o: &Observation<'_>) {
        let t = o.t_s;
        let tr = o.truth;
        let fs = &o.snap.fast.status;
        let sup = &o.snap.sup;
        let rep = &o.snap.seis.report;
        let v = tr.v_mps;
        let first = !self.seen;
        self.seen = true;

        // ---- energies, forces, power (always present) ----
        self.ke_gov_max = self.ke_gov_max.max(fs.e_kin_gov_j as f64);
        self.passivity_min = self.passivity_min.min(fs.e_passivity_j as f64);
        self.momentum_max = self.momentum_max.max((tr.mass_kg * v).abs());
        let c = &tr.f_contact;
        self.contact_max = self.contact_max.max(c.obstacle_n);
        self.guide_max = self.guide_max.max(c.guide_fin_n.max(c.pads_n));
        self.upper_max = self.upper_max.max(c.upper_n);
        self.roller_max = self.roller_max.max(c.rollers_n);
        self.p_bus_max = self.p_bus_max.max(tr.power.p_bus_w);
        self.e_hold_min = min_opt(self.e_hold_min, tr.power.e_hold_j);
        self.mass_kg = tr.mass_kg;

        // ---- cumulative ledger quantities ----
        let led = &tr.e_diss;
        if first {
            self.bumper_base = Some(if self.windowed {
                (led.bumper_impacts, led.bumper_j)
            } else {
                (0, 0.0)
            });
            self.drag_base = Some(if self.windowed {
                drag_energy_j(tr)
            } else {
                0.0
            });
        }
        let (b0, e0) = self.bumper_base.unwrap_or((0, 0.0));
        let new_impacts = led.bumper_impacts.saturating_sub(b0);
        if new_impacts > self.bumper_impacts {
            // A new impact began: its energy is the kinetic energy at contact onset.
            self.bumper_e_max = self.bumper_e_max.max(0.5 * tr.mass_kg * v * v);
        }
        self.bumper_impacts = new_impacts;
        self.bumper_total_j = led.bumper_j - e0;
        if !self.windowed {
            self.bumper_e_max = self.bumper_e_max.max(led.bumper_e_max_j);
        }
        self.drag_j = drag_energy_j(tr) - self.drag_base.unwrap_or(0.0);
        if let Some(px) = self.prev_x {
            self.distance_m += (tr.x_m - px).abs();
        }
        self.prev_x = Some(tr.x_m);

        // ---- hand ----
        let hand_active = o.hand_since.is_some();
        if let Some(h0) = o.hand_since {
            if self.hand_t0 != Some(h0) {
                self.hand_t0 = Some(h0);
                self.hand_peak_pre_onset = 0.0;
                self.moved_since_hand = false;
            }
            let fh = tr.f_hand_n;
            if !self.moved_since_hand {
                self.hand_peak_pre_onset = self.hand_peak_pre_onset.max(fh.abs());
                if v.abs() > V_ONSET_MPS {
                    self.moved_since_hand = true;
                    if self.breakaway.is_none() {
                        self.breakaway = Some(self.hand_peak_pre_onset);
                    }
                }
            }
            let hp = &sup.directive.haptic;
            if tr.state.levitated && fs.mode == MotionMode::Admittance && hp.m_v_kg > 0.0 {
                let a = tr.a_mps2;
                self.am[0] += a * a;
                self.am[1] += a * v;
                self.am[2] += v * v;
                self.am[3] += fh * a;
                self.am[4] += fh * v;
                self.am_n += 1;
                self.m_v = hp.m_v_kg as f64;
            }
            if hp.n_detents > 0 && hp.detent_f_n > 0.0 && tr.state.levitated {
                let w = hp.detent_width_m as f64;
                let n = (hp.n_detents as usize).min(hp.detents.len());
                let inside = hp.detents[..n]
                    .iter()
                    .any(|d| (tr.x_m - d.to_m_f64()).abs() < w);
                if inside {
                    self.b_v = hp.b_v_n_s_per_m as f64;
                    let mv = hp.m_v_kg as f64;
                    let r = fh - mv * tr.a_mps2 - self.b_v * v;
                    self.detent_peak = max_opt(self.detent_peak, r.abs());
                    self.detent_f = hp.detent_f_n as f64;
                }
            }
            if !tr.state.levitated {
                self.manual_force = max_opt(self.manual_force, fh.abs());
            }
            if self.intent_onset.is_none()
                && matches!(
                    fs.intent,
                    IntentFlag::PendingOpen
                        | IntentFlag::PendingClose
                        | IntentFlag::Open
                        | IntentFlag::Close
                )
            {
                self.intent_onset = Some(t - h0);
            }
            if self.intent_confirm.is_none()
                && matches!(fs.intent, IntentFlag::Open | IntentFlag::Close)
            {
                self.intent_confirm = Some(t - h0);
            }
        }

        // ---- motion ----
        let d_closed = tr.x_m - ctx.x_closed_m;
        if d_closed > SOFT_CLOSE_ZONE_M {
            self.far_from_closed = true;
        } else if self.far_from_closed && v <= 0.0 {
            self.soft_close_v = Some(v.abs());
            self.far_from_closed = false;
        }
        let moving_mode = matches!(
            fs.mode,
            MotionMode::Trajectory
                | MotionMode::Terminus
                | MotionMode::SoftOpen
                | MotionMode::SoftClose
        );
        if moving_mode {
            let seg = &sup.directive.segment;
            let past = (tr.x_m - seg.end().to_m_f64()) * seg.dir().sign() as f64;
            self.overshoot = max_opt(self.overshoot, past.max(0.0));
        }
        let idle = sup.status.lev == LevState::LevitatedIdle && tr.state.levitated;
        if idle && !hand_active && matches!(fs.mode, MotionMode::Off | MotionMode::Hold) {
            self.force_noise.push(tr.f_lsm_n);
        }

        // ---- levitation ----
        let n = ctx.n_magnets;
        let lev_ok = tr.state.levitated
            && matches!(sup.status.lev, LevState::LevitatedIdle | LevState::Moving);
        if lev_ok && tr.mass_kg > 0.0 {
            let g_zp =
                derive::zero_power_gap(tr.mass_kg, ctx.g_star_m, ctx.h_prime_m, ctx.m_star_kg);
            if g_zp.is_finite() {
                for i in 0..n {
                    if ctx.lift_mask[i] {
                        self.gap_dev_max = max_opt(self.gap_dev_max, (tr.gaps_m[i] - g_zp).abs());
                    }
                }
            }
        }
        if tr.state.levitated && n > 0 && t >= ctx.duration_s - COIL_SS_WINDOW_S {
            let m: f64 = tr.coil_i_a[..n].iter().map(|i| i.abs()).sum::<f64>() / n as f64;
            self.coil_ss.push(m);
        }
        let copper = led.coil_copper_j + led.lsm_copper_j;
        if idle {
            // Copper energy over consecutive idle observations (PHYSICS §14).
            if self.prev_lev_idle
                && let (Some(pc), Some(pt)) = (self.prev_copper_j, self.prev_t)
                && t > pt
            {
                self.hover_e_j += copper - pc;
                self.hover_t_s += t - pt;
            }
            let lifts: Vec<f64> = (0..n)
                .filter(|&i| ctx.lift_mask[i])
                .map(|i| tr.gaps_m[i])
                .collect();
            if !lifts.is_empty() {
                self.hover_gap
                    .push(lifts.iter().sum::<f64>() / lifts.len() as f64);
            }
        }
        self.prev_copper_j = Some(copper);
        self.prev_lev_idle = idle;
        self.prev_t = Some(t);

        // Directive transitions start the lift / unseat / land clocks.
        let dir_code = match sup.directive.lev {
            LevDirective::Off => 0u8,
            LevDirective::Unseat => 1,
            LevDirective::Lift => 2,
            LevDirective::Levitate => 3,
            LevDirective::Land { .. } => 4,
            LevDirective::Seat => 5,
            LevDirective::Hold => 6,
        };
        if self.prev_dir != Some(dir_code) {
            match dir_code {
                1 => self.unseat_start = Some(t),
                2 => self.lift_start = Some(t),
                4 | 5 => self.land_start = Some(t),
                _ => {}
            }
            self.prev_dir = Some(dir_code);
        }
        // Heave speed from successive truths (z = g* − gap, up +).
        let z = tr.pose[2];
        if let Some((tz, pz)) = self.prev_z
            && tr.t_s > tz
        {
            self.v_z = (z - pz) / (tr.t_s - tz);
        }
        if self.prev_z.is_none_or(|(tz, _)| tr.t_s > tz) {
            self.prev_z = Some((tr.t_s, z));
        }
        let st = (tr.state.levitated, tr.state.seated, tr.state.landed);
        if let Some((plev, pseat, pland)) = self.prev_contact {
            if st.0 && !plev {
                if let Some(s0) = self.lift_start.take()
                    && self.lift_time.is_none()
                {
                    self.lift_time = Some(t - s0);
                }
                if !st.1
                    && let Some(s0) = self.unseat_start.take()
                    && self.unseat_time.is_none()
                {
                    self.unseat_time = Some(t - s0);
                }
            }
            if (st.1 && !pseat) || (st.2 && !pland) {
                self.touchdown_v = Some(self.v_z.abs());
                if let Some(s0) = self.land_start.take()
                    && self.land_time.is_none()
                {
                    self.land_time = Some(t - s0);
                }
            }
        }
        self.prev_contact = Some(st);

        // ---- seismic ----
        self.level_max = self.level_max.max(rep.level.as_u8());
        if self.trigger_t.is_none()
            && let Some(tt) = rep.triggered_at
        {
            self.trigger_t = Some(tt.seconds_f64());
            self.stroke_base = Some(tr.x_m);
            self.stroke_max = 0.0;
        }
        if self.freeze_t.is_none() && fs.mode == MotionMode::Freeze {
            self.freeze_t = Some(t);
        }
        if self.protect_margin.is_none()
            && rep.protect
            && let Some(eta) = rep.s_eta
        {
            self.protect_margin = Some(eta.seconds_f64() - t);
        }
        let base = *self.stroke_base.get_or_insert(tr.x_m);
        self.stroke_max = self.stroke_max.max((tr.x_m - base).abs());

        // ---- obstruction ----
        if self.obstacle_t.is_none() && c.obstacle_n > 0.0 {
            self.obstacle_t = Some(t);
        }
        if self.reverse_latency.is_none()
            && fs.mode == MotionMode::Reverse
            && let Some(t0) = self.obstacle_t
        {
            self.reverse_latency = Some(t - t0);
        }

        // ---- terminus (FAB-HAP-013…016) ----
        self.observe_terminus(o);

        // ---- inertial smoothing (FAB-HAP-017…019) ----
        if fs.mode == MotionMode::InertialSmooth {
            let x0 = *self.inertial_x0.get_or_insert(tr.x_m);
            self.inertial_acc.push(tr.a_mps2 + o.floor_x_mps2);
            self.inertial_floor.push(o.floor_x_mps2);
            self.inertial_stroke = max_opt(self.inertial_stroke, (tr.x_m - x0).abs());
        } else {
            self.inertial_x0 = None;
        }
    }

    fn observe_terminus(&mut self, o: &Observation<'_>) {
        let seg = &o.snap.sup.directive.segment;
        let tr = o.truth;
        let t = o.t_s;
        match seg {
            Segment::Terminus(ts) => {
                let t0 = ts.t0.seconds_f64();
                if self.term.is_none_or(|k| k.t0_s != t0) {
                    self.finish_terminus();
                    let t_cf = (ts.d_b_m > ts.d_eps_m && ts.d_eps_m > 0.0).then(|| {
                        ts.t_b_s as f64
                            + ts.tau_s as f64 * fmath::ln(ts.d_b_m as f64 / ts.d_eps_m as f64)
                            + ts.t_q_s as f64
                    });
                    self.term = Some(TerminusTrack {
                        t0_s: t0,
                        target_m: ts.target.to_m_f64(),
                        sign: match ts.dir {
                            TravelDir::Open => 1.0,
                            TravelDir::Close => -1.0,
                        },
                        t_b_s: ts.t_b_s as f64,
                        d_eps_m: ts.d_eps_m as f64,
                        t_cf_s: t_cf,
                        fit: LinFit::default(),
                        overshoot_m: 0.0,
                        arrive: None,
                        ref_arrive: None,
                    });
                }
                if let Some(k) = self.term.as_mut()
                    && k.ref_arrive.is_none()
                {
                    let (xr, _, _) = fabrail_core::motion::terminus::eval(
                        ts,
                        fabrail_types::Tick::from_seconds_f64(t),
                    );
                    if k.sign * (k.target_m - xr.to_m_f64()) <= TERMINUS_ARRIVE_TOL_M {
                        k.ref_arrive = Some(t - k.t0_s);
                    }
                }
            }
            Segment::Quintic(q) => {
                if self
                    .term
                    .is_some_and(|k| (k.target_m - q.target.to_m_f64()).abs() > 1e-9)
                {
                    self.finish_terminus();
                }
            }
            Segment::SCurve(_) => self.finish_terminus(),
        }
        let quiet = o.floor_x_mps2.abs() < TERMINUS_QUIET_MPS2;
        if let Some(k) = self.term.as_mut() {
            // Distance-to-go d = sign·(target − x) ≥ 0 while approaching.
            let d = k.sign * (k.target_m - tr.x_m);
            if quiet && t >= k.t0_s + k.t_b_s && d > k.d_eps_m {
                k.fit.push(t, fmath::ln(d));
            }
            k.overshoot_m = k.overshoot_m.max(-d);
            if k.arrive.is_none() && d <= TERMINUS_ARRIVE_TOL_M {
                k.arrive = Some((t - k.t0_s, tr.v_mps.abs()));
            }
        }
    }

    fn finish_terminus(&mut self) {
        let Some(k) = self.term.take() else {
            return;
        };
        self.term_count += 1;
        if let Some(r2) = k.fit.r2() {
            self.term_r2_min = min_opt(self.term_r2_min, r2);
        }
        if let Some(s) = k.fit.slope()
            && s < 0.0
        {
            self.term_tau = Some(-1.0 / s);
        }
        self.term_over_max = max_opt(self.term_over_max, k.overshoot_m.max(0.0));
        if let Some((ta, va)) = k.arrive {
            self.term_v_max = max_opt(self.term_v_max, va);
            self.term_t_last = Some((ta, k.t_cf_s));
            if let Some(cf) = k.ref_arrive.or(k.t_cf_s)
                && cf > 0.0
            {
                self.term_err_max = max_opt(self.term_err_max, (ta / cf - 1.0).abs());
            }
        }
    }

    fn finish(&self, out: &mut BTreeMap<String, f64>, prefix: &str) {
        let mut acc = self.clone();
        acc.finish_terminus();
        let mut put = |k: &str, v: Option<f64>| {
            if let Some(x) = v
                && x.is_finite()
            {
                out.insert(format!("{prefix}{k}"), x);
            }
        };
        put("breakaway_n", acc.breakaway);
        let am = acc.am;
        let det = am[0] * am[2] - am[1] * am[1];
        let m_fit = (acc.am_n >= 3 && det > 0.0 && acc.m_v > 0.0)
            .then(|| (am[3] * am[2] - am[4] * am[1]) / det);
        put(
            "apparent_mass_err",
            m_fit.map(|m| (m - acc.m_v).abs() / acc.m_v),
        );
        put("soft_close_v_final", acc.soft_close_v);
        put("overshoot_m", acc.overshoot);
        put(
            "detent_peak_err",
            acc.detent_peak
                .filter(|_| acc.detent_f > 0.0)
                .map(|p| (p - acc.detent_f).abs() / acc.detent_f),
        );
        put(
            "passivity_min_j",
            Some(if acc.passivity_min.is_finite() {
                acc.passivity_min
            } else {
                0.0
            }),
        );
        put("force_noise_rms_n", acc.force_noise.rms_dev());
        put("ke_max_gov_j", Some(acc.ke_gov_max));
        put("momentum_max", Some(acc.momentum_max));
        put("contact_force_max_n", Some(acc.contact_max));
        put("guide_force_max_n", Some(acc.guide_max));
        put("gap_dev_max_m", acc.gap_dev_max);
        put("coil_i_ss_a", acc.coil_ss.mean());
        put(
            "hover_power_w",
            (acc.hover_t_s > 0.0).then(|| acc.hover_e_j / acc.hover_t_s),
        );
        put("lift_time_s", acc.lift_time);
        put("unseat_time_s", acc.unseat_time);
        put("land_time_s", acc.land_time);
        put("touchdown_v_mps", acc.touchdown_v);
        put("trigger_t_s", acc.trigger_t);
        put("freeze_t_s", acc.freeze_t);
        put("level_max", Some(acc.level_max as f64));
        put("protect_margin_to_s_s", acc.protect_margin);
        put("stroke_max_m", Some(acc.stroke_max));
        put("bumper_impacts", Some(acc.bumper_impacts as f64));
        put("bumper_energy_j", Some(acc.bumper_e_max));
        put("manual_force_n", acc.manual_force);
        put("intent_onset_s", acc.intent_onset);
        put("intent_confirm_s", acc.intent_confirm);
        put("reverse_latency_s", acc.reverse_latency);
        put(
            "mu_eq",
            (acc.distance_m > 0.0 && acc.mass_kg > 0.0).then(|| {
                fabrail_plant::dissipation::mu_eq(acc.drag_j, acc.mass_kg, acc.distance_m)
            }),
        );
        // Additions.
        put("bumper_energy_total_j", Some(acc.bumper_total_j));
        put("upper_force_max_n", Some(acc.upper_max));
        put("roller_force_max_n", Some(acc.roller_max));
        put("p_bus_max_w", Some(acc.p_bus_max));
        put("e_hold_min_j", acc.e_hold_min);
        put("hover_gap_mean_m", acc.hover_gap.mean());
        put("hover_gap_rms_m", acc.hover_gap.rms_dev());
        if acc.term_count > 0 {
            put("terminus_count", Some(acc.term_count as f64));
        }
        put("terminus_r2_min", acc.term_r2_min);
        put("terminus_tau_fit_s", acc.term_tau);
        put("terminus_overshoot_max_m", acc.term_over_max);
        put("terminus_v_arrive_max_mps", acc.term_v_max);
        put("terminus_t_arrive_s", acc.term_t_last.map(|(t, _)| t));
        put(
            "terminus_t_arrive_cf_s",
            acc.term_t_last.and_then(|(_, c)| c),
        );
        put("terminus_t_arrive_err_max", acc.term_err_max);
        let ia = acc.inertial_acc.get();
        let ifl = acc.inertial_floor.get();
        put("inertial_acc_rms_mps2", ia);
        put("inertial_floor_rms_mps2", ifl);
        put(
            "inertial_ratio",
            ia.zip(ifl).filter(|(_, f)| *f > 0.0).map(|(a, f)| a / f),
        );
        put("inertial_stroke_max_m", acc.inertial_stroke);
    }
}

/// Online KPI accumulator.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Metrics {
    ctx: MetricsCtx,
    global: Acc,
    windows: Vec<(String, f64, f64, Acc)>,
    samples: u64,
}

impl Metrics {
    /// An accumulator for parameter set `p`, a run of `duration_s` and the named KPI
    /// windows `windows` (`[t0_s, t1_s]`).
    pub fn new(p: &SystemParams, duration_s: f64, windows: &BTreeMap<String, [f64; 2]>) -> Metrics {
        Metrics {
            ctx: MetricsCtx::new(p, duration_s),
            global: Acc::new(false),
            windows: windows
                .iter()
                .map(|(k, w)| (k.clone(), w[0], w[1], Acc::new(true)))
                .collect(),
            samples: 0,
        }
    }

    /// Base-rate samples seen.
    pub fn samples(&self) -> u64 {
        self.samples
    }

    /// Accumulates one base-rate observation.
    pub fn observe(&mut self, o: &Observation<'_>) {
        self.samples += 1;
        self.global.observe(&self.ctx, o);
        for (_, t0, t1, acc) in &mut self.windows {
            if o.t_s >= *t0 && o.t_s <= *t1 {
                acc.observe(&self.ctx, o);
            }
        }
    }

    /// Final KPIs (see the module docs; absent = phase not observed).
    pub fn finish(&self) -> BTreeMap<String, f64> {
        let mut out = BTreeMap::new();
        if self.samples == 0 {
            return out;
        }
        self.global.finish(&mut out, "");
        for (name, _, _, acc) in &self.windows {
            if acc.seen {
                acc.finish(&mut out, &format!("{name}."));
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kpi_names_unique_and_snake_case() {
        let set: std::collections::BTreeSet<_> = KPI_NAMES.iter().chain(KPI_EXTRA_NAMES).collect();
        assert_eq!(set.len(), KPI_NAMES.len() + KPI_EXTRA_NAMES.len());
        for n in KPI_NAMES.iter().chain(KPI_EXTRA_NAMES) {
            assert!(
                n.chars()
                    .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_'),
                "{n}"
            );
        }
    }

    #[test]
    fn linear_fit_recovers_an_exponential() {
        let mut f = LinFit::default();
        let tau = 0.25;
        for k in 0..1000 {
            let t = 1.0 + k as f64 * 1e-3;
            f.push(t, (0.03f64 * (-(t - 1.0) / tau).exp()).ln());
        }
        let r2 = f.r2().expect("fit");
        assert!((r2 - 1.0).abs() < 1e-12, "(E) R² of an exact line: {r2}");
        assert!((-1.0 / f.slope().expect("slope") - tau).abs() < 1e-9);
    }

    #[test]
    fn welford_and_rms() {
        let mut w = Welford::default();
        for x in [1.0, 2.0, 3.0, 4.0] {
            w.push(x);
        }
        assert_eq!(w.mean(), Some(2.5));
        assert!((w.rms_dev().expect("rms") - 1.25f64.sqrt()).abs() < 1e-15);
        let mut r = Rms::default();
        r.push(3.0);
        r.push(-4.0);
        assert!((r.get().expect("rms") - 12.5f64.sqrt()).abs() < 1e-15);
    }

    fn observe(m: &mut Metrics, t: f64, truth: &Truth, snap: &ExecSnapshot) {
        m.observe(&Observation {
            t_s: t,
            truth,
            snap,
            hand_since: None,
            floor_x_mps2: 0.0,
        });
    }

    #[test]
    fn terminus_kpis_from_an_ideal_capture() {
        use fabrail_types::{PosNm, TerminusSeg, Tick};
        let p = fabrail_params::presets::door();
        let mut m = Metrics::new(&p, 3.0, &BTreeMap::new());
        let (tau, d_b, d_eps, t_b, t_q) = (0.25, 0.03, 2.0e-4, 0.1, 0.25);
        let ts = TerminusSeg {
            target: PosNm(0),
            t0: Tick::from_seconds_f64(1.0),
            dir: TravelDir::Close,
            v_app_mps: 0.15,
            j_mps3: 10.0,
            tau_s: tau,
            t_b_s: t_b,
            d_b_m: d_b,
            d_0_m: 0.035,
            d_eps_m: d_eps,
            t_q_s: t_q,
        };
        let mut snap = ExecSnapshot::default();
        snap.sup.directive.segment = Segment::Terminus(ts);
        snap.fast.status.mode = MotionMode::Terminus;
        let (tau, d_b, d_eps, t_b, t_q) =
            (tau as f64, d_b as f64, d_eps as f64, t_b as f64, t_q as f64);
        let t_eps = 1.0 + t_b + tau * (d_b / d_eps).ln();
        let h = 1.0e-3;
        for k in 0..3000 {
            let t = k as f64 * h;
            // Closing toward x = 0: x = target + d. Ideal blend, capture, linear finish.
            let (d, v) = if t < 1.0 + t_b {
                let u = ((t - 1.0) / t_b).clamp(0.0, 1.0);
                (0.035 - 0.005 * u, -0.05)
            } else if t < t_eps {
                let d = d_b * (-(t - 1.0 - t_b) / tau).exp();
                (d, -d / tau)
            } else {
                ((d_eps * (1.0 - (t - t_eps) / t_q)).max(0.0), -d_eps / t_q)
            };
            let truth = Truth {
                t_s: t,
                x_m: d,
                v_mps: v,
                ..Truth::default()
            };
            observe(&mut m, t, &truth, &snap);
        }
        let k = m.finish();
        assert_eq!(k["terminus_count"], 1.0);
        let r2 = k["terminus_r2_min"];
        assert!(
            r2 > 0.999_999,
            "(E) semilog R² of an exact exponential: {r2}"
        );
        let tau_fit = k["terminus_tau_fit_s"];
        assert!(
            (tau_fit / tau - 1.0).abs() < 1e-6,
            "(E) τ from the slope: {tau_fit}"
        );
        assert_eq!(k["terminus_overshoot_max_m"], 0.0);
        // Arrival at d ≤ 0.02 mm on the linear finish: 0.9·T_q after t_ε.
        let t_arr = k["terminus_t_arrive_s"];
        let want = t_eps + 0.9 * t_q - 1.0;
        assert!(
            (t_arr - want).abs() <= 1.5e-3,
            "(I) one sample: {t_arr} vs {want}"
        );
        let cf = t_b + tau * (d_b / d_eps).ln() + t_q;
        assert!(
            (k["terminus_t_arrive_cf_s"] - cf).abs() < 1e-6,
            "closed form, PHYSICS §16.1"
        );
        assert!((k["terminus_v_arrive_max_mps"] - d_eps / t_q).abs() < 1e-12);
    }

    #[test]
    fn lift_land_timing_and_touchdown_speed() {
        let p = fabrail_params::presets::wall();
        let mut m = Metrics::new(&p, 2.0, &BTreeMap::new());
        let mut snap = ExecSnapshot::default();
        let h = 1.0e-3;
        for k in 0..2000 {
            let t = k as f64 * h;
            snap.sup.directive.lev = if t < 0.5 {
                LevDirective::Off
            } else if t < 1.0 {
                LevDirective::Lift
            } else {
                LevDirective::Land { rate_mps: 3.0e-3 }
            };
            let levitated = (0.7..1.3).contains(&t);
            // Heave falls at 3 mm/s during the landing.
            let z = if t >= 1.0 { -3.0e-3 * (t - 1.0) } else { 0.0 };
            let mut truth = Truth {
                t_s: t,
                ..Truth::default()
            };
            truth.pose[2] = z;
            truth.state.levitated = levitated;
            truth.state.landed = !levitated;
            observe(&mut m, t, &truth, &snap);
        }
        let k = m.finish();
        assert!(
            (k["lift_time_s"] - 0.2).abs() < 1.5e-3,
            "{}",
            k["lift_time_s"]
        );
        assert!(
            (k["land_time_s"] - 0.3).abs() < 1.5e-3,
            "{}",
            k["land_time_s"]
        );
        assert!(
            (k["touchdown_v_mps"] - 3.0e-3).abs() < 1e-9,
            "(E) heave speed at touchdown"
        );
        assert!(!k.contains_key("unseat_time_s"), "absent, never NaN");
        assert!(k.values().all(|v| v.is_finite()));
    }

    #[test]
    fn windows_restrict_the_accumulators() {
        let p = fabrail_params::presets::door();
        let w: BTreeMap<String, [f64; 2]> =
            [("push".to_string(), [0.5, 0.6])].into_iter().collect();
        let mut m = Metrics::new(&p, 1.0, &w);
        let snap = ExecSnapshot::default();
        for k in 0..1000 {
            let t = k as f64 * 1.0e-3;
            let mut truth = Truth {
                t_s: t,
                mass_kg: 100.0,
                v_mps: if t < 0.2 { 1.0 } else { 0.1 },
                ..Truth::default()
            };
            truth.power.p_bus_w = t;
            observe(&mut m, t, &truth, &snap);
        }
        let k = m.finish();
        assert!((k["momentum_max"] - 100.0).abs() < 1e-12);
        assert!((k["push.momentum_max"] - 10.0).abs() < 1e-12);
        assert!((k["push.p_bus_max_w"] - 0.6).abs() < 1e-12);
    }
}
