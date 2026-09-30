//! Log-damped terminus: constant-jerk blend, exponential capture d = d_b*exp(-t/tau), quintic from d_eps; hand re-plan (PHYSICS §16.1). Owner: E1.
//!
//! Frozen API (W0): [`TerminusPlanner::new`], [`TerminusPlanner::tau_eff`],
//! [`TerminusPlanner::plan`] (from the measured distance-to-go, speed and acceleration
//! with τ and j) → `Segment`, and the closed-form evaluators [`eval`] (a
//! `TerminusSeg`) and [`eval_quintic`] (a `QuinticSeg`), each returning (x, v, a).
//! Wave-1 additions (E1): [`TerminusPlanner::plan_detailed`] / [`TerminusPlanner::replan`]
//! → [`Plan`] (kind, τ used, arrival tick, and for a start from rest the terminus to
//! switch to), [`TerminusPlanner::nominal`], [`TerminusPlanner::approach`], [`blend`],
//! [`tau_min`], [`eval_d`] / [`eval_quintic_d`] (distance-to-go in continuous time),
//! [`phase`], [`integrator_frozen`], [`done`], [`arrive_tick`] and [`x_obs`].
//!
//! The law (PHYSICS §16.1; plan §4b), for approach speed v, jerk j, time constant τ and
//! capture distance d_ε:
//! 1. blend out of cruise with constant jerk: t_b = √(τ² + 2v/j) − τ, d_b = j·t_b·τ²,
//!    d_0 = d_b + v·t_b − j·t_b³/6 (`params::derive::terminus_blend`); speed and
//!    deceleration match the exponential at t_b (v − j·t_b²/2 = d_b/τ, j·t_b = d_b/τ²);
//! 2. exponential capture d(t) = d_b·e^(−t/τ): ln d is a straight line of slope −1/τ
//!    on a semilog plot (the "log-damped" definition);
//! 3. at d_ε a quintic from (d_ε, −d_ε/τ, d_ε/τ²) runs monotonically to v = a = 0 in
//!    T_q = τ (monotone for T_q/τ ∈ [1, 6]; FAB-HAP-014);
//! 4. arrival t_arrive = t_b + τ·ln(d_b/d_ε) + T_q (+0.2 s DRAWER seat step, applied by
//!    the supervisor).
//!
//! The reference is C² everywhere (cruise → blend → capture → quintic → rest); the jerk
//! steps at each joint. If d_b ≤ d_ε the capture is skipped and the quintic starts at
//! the blend end (d_b, −d_b/τ, d_b/τ²), which has the same normalised shape.
//!
//! **Evaluation.** Distances are distance-to-go d ≥ 0 from `target` against `dir`:
//! x = target − dir.sign()·d. Every segment is evaluated in closed form from its target
//! and start tick (the time since t0 is formed from the integer tick difference, then
//! cast), never as a running f32 sum (PHYSICS §24), so the result is independent of the
//! absolute position and tick. Before t0 a terminus extrapolates the cruise it blends out
//! of (d = d_0 + v_app·(t0 − t), constant speed): a supervisor can hand over from a
//! cruising S-curve at any release before t0 without a reference step. A quintic before
//! its t0 likewise extrapolates its start state at constant rate.
//!
//! τ ≥ τ_min = v_app/a_max − a_max/(2j) (validated): τ_min is exactly the τ at which the
//! blend's peak deceleration j·t_b equals a_max. At runtime τ_eff = max(τ,
//! τ_min(v_measured)) covers hand-thrown drawers.
//!
//! **Re-plans** ([`TerminusPlanner::plan_detailed`], [`PlanKind`]) from the measured
//! (d, v, a) after a hand release:
//! - d ≥ d_0(v, τ): cruise at v, then the nominal terminus (`Nominal`);
//! - d < d_0: τ is shortened (bisection on d_0(τ') = d, d_0 is increasing in τ) down to
//!   τ_min(v, a_brk, j) with a_brk = replan_brk_ratio·a_max = 2·a_max, so the peak
//!   deceleration stays ≤ a_brk (`Braked`), and not below 2/ω_v (the frozen-integrator
//!   error poles then stay at least as fast as the reference); otherwise `Infeasible`
//!   (the caller falls back to core SoftClose) with a best-effort quintic;
//! - (nearly) at rest (|v| below the class soft-close final speed of HAP-007, or d_ε/τ
//!   if larger): a jerk-limited S-curve launch to the fastest v_app ≤ v_cap whose launch
//!   plus d_0(v_app) fits, then the terminus (`Approach`; the caller switches to
//!   [`Plan::next`] once the launch is cruising). The launch starts from rest, so the
//!   reference speed steps by at most that final speed; a faster release cruises at its
//!   own speed instead, which bounds the cruise to d/v_final;
//! - inside d_ε, or receding: a stand-alone quintic from the measured state whose
//!   duration is searched near T_q for no overshoot, monotone approach and |d̈| ≤ a_brk
//!   (`Quintic`).
//!
//! A terminus re-plan cannot carry the measured acceleration (the blend starts at
//! d̈ = 0), so it is C¹ at the re-plan instant; quintic re-plans are C².
//!
//! **Integrator freeze.** During blend and capture ([`integrator_frozen`]) the cascade's
//! velocity integrator is frozen with bumpless hold: with K_v = m̂·ω_v and K_x = ω_v/4
//! the error poles are (s + ω_v/2)², ζ = 1; ω_v/2 ≥ 3/τ holds in every preset. It is
//! re-enabled at d_ε (the quintic).
//!
//! **Governor.** A commanded terminus is observed at x_obs = target + dir·overtravel
//! ([`x_obs`]), so the braking envelope never clips the exponential tail.
//!
//! Acceptance: FAB-HAP-013…016 (semilog R² ≥ 0.999, C², ζ ≥ 0.9, overshoot ≤ 0.02 mm,
//! arrival ≤ 1 mm/s, t_arrive within ±2 % of the closed form).

use core::f64::consts::PI;

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{
    BASE_TICK_HZ, PosNm, QuinticSeg, SCurveSeg, Segment, TerminusSeg, Tick, TravelDir, fmath,
};

/// Base ticks per second as f32 (20 000; exact).
const TICK_HZ_F32: f32 = BASE_TICK_HZ as f32;

/// Bisection steps for the braked τ and the approach speed (2⁻⁴⁰ of the bracket: below
/// f32 resolution).
const BISECT_STEPS: u32 = 40;

/// Samples per quintic feasibility check (plus the end point).
const QUINTIC_CHECK_SAMPLES: u16 = 32;

/// Quintic re-plan duration search: T = T_q·2^(k/4) for |k| ≤ this (T_q/64 … 64·T_q).
const QUINTIC_SEARCH_STEPS: i32 = 24;

/// Distance-to-go state in the segment's own frame (d ≥ 0 toward the target).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DState {
    /// Distance-to-go d, m.
    pub d_m: f32,
    /// ḋ (negative while approaching), m/s.
    pub rate_mps: f32,
    /// d̈ (positive while decelerating toward the target), m/s².
    pub accel_mps2: f32,
}

/// Blend quantities (PHYSICS §16.1): the f32 twin of `params::derive::terminus_blend`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Blend {
    /// Blend duration t_b, s.
    pub t_b_s: f32,
    /// Distance-to-go at the end of the blend d_b, m.
    pub d_b_m: f32,
    /// Distance-to-go at blend start d_0, m.
    pub d_0_m: f32,
}

/// Constant-jerk blend out of cruise at speed v with jerk j and time constant τ
/// (PHYSICS §16.1; plan §4b): t_b = √(τ² + 2v/j) − τ, written as (2v/j)/(√(τ² + 2v/j) + τ)
/// to avoid cancellation at small v; d_b = j·t_b·τ²; d_0 = d_b + v·t_b − j·t_b³/6.
/// Returns zeros for a non-positive or non-finite τ or j.
pub fn blend(v_mps: f32, j_mps3: f32, tau_s: f32) -> Blend {
    let v = fmath::absf(v_mps);
    if !(v.is_finite() && j_mps3.is_finite() && tau_s.is_finite() && j_mps3 > 0.0 && tau_s > 0.0) {
        return Blend::default();
    }
    let q = 2.0 * v / j_mps3;
    let t_b = q / (fmath::sqrtf(tau_s * tau_s + q) + tau_s);
    let d_b = j_mps3 * t_b * tau_s * tau_s;
    Blend {
        t_b_s: t_b,
        d_b_m: d_b,
        d_0_m: d_b + v * t_b - j_mps3 * t_b * t_b * t_b / 6.0,
    }
}

/// Minimum time constant τ_min = |v|/a − a/(2j) (PHYSICS §16.1; plan §4b): the τ at which
/// the blend's peak deceleration j·t_b equals `a_mps2`. `f32::INFINITY` when a or j is not
/// positive (mirrors `params::derive::tau_min`), s.
pub fn tau_min(v_mps: f32, a_mps2: f32, j_mps3: f32) -> f32 {
    if !(a_mps2 > 0.0 && j_mps3 > 0.0) {
        return f32::INFINITY;
    }
    fmath::absf(v_mps) / a_mps2 - a_mps2 / (2.0 * j_mps3)
}

/// Jerk-limited launch from rest to speed v with limits a, j (symmetric, so the
/// distance is v·T/2): T = a/j + v/a when v·j ≥ a², else 2·√(v/j) (PHYSICS §16).
/// Returns (T, distance).
fn launch(v_mps: f32, a_mps2: f32, j_mps3: f32) -> (f32, f32) {
    if !(v_mps > 0.0 && a_mps2 > 0.0 && j_mps3 > 0.0) {
        return (0.0, 0.0);
    }
    let t = if v_mps * j_mps3 >= a_mps2 * a_mps2 {
        a_mps2 / j_mps3 + v_mps / a_mps2
    } else {
        2.0 * fmath::sqrtf(v_mps / j_mps3)
    };
    (t, 0.5 * v_mps * t)
}

/// Signed seconds from `t0` to `t` (integer tick difference first, then cast).
#[inline]
fn rel_s(t: Tick, t0: Tick) -> f32 {
    if t >= t0 {
        t.since_s(t0)
    } else {
        -t0.since_s(t)
    }
}

/// Nearest whole number of base ticks in `s` seconds (0 for non-positive or NaN;
/// saturating).
#[inline]
fn ticks_round(s: f32) -> u64 {
    if s > 0.0 {
        fmath::roundf(s * TICK_HZ_F32) as u64
    } else {
        0
    }
}

/// Whole base ticks n such that the evaluators' time since t0 (`Tick::since_s`, the
/// value [`eval`] sees) is ≥ `s` (0 for non-positive or NaN; saturating).
#[inline]
fn ticks_ceil(s: f32) -> u64 {
    let positive = s > 0.0; // false for NaN
    if !positive {
        return 0;
    }
    let mut n = fmath::ceilf(s * TICK_HZ_F32) as u64;
    // The f32 product can round down across an integer: step past the shortfall.
    for _ in 0..4 {
        if Tick(n).since_s(Tick::ZERO) >= s {
            break;
        }
        n = n.saturating_add(1);
    }
    n
}

/// (x, v, a) along +x from a distance-to-go state: x = target − dir·d.
#[inline]
fn to_x(target: PosNm, dir: TravelDir, st: DState) -> (PosNm, f32, f32) {
    let sg = dir.sign();
    (
        target.offset_m(-sg * st.d_m),
        -sg * st.rate_mps,
        -sg * st.accel_mps2,
    )
}

/// Quintic in normalised time σ = s/T from (d0, ḋ0, d̈0) to (0, 0, 0) in T (the
/// standard minimum-jerk boundary solution; PHYSICS §16.1). Ending at rest with zero
/// acceleration puts a triple root at σ = 1, so it is evaluated in factored form
/// d = (1 − σ)³·(α + β·σ + γ·σ²) with α = d0, β = ḋ0·T + 3·d0 and
/// γ = d̈0·T²/2 + 3·ḋ0·T + 6·d0: the reference lands on the target exactly and ḋ keeps
/// its sign near the end in f32 (the expanded power form loses it to rounding).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Quintic {
    alpha: f32,
    beta: f32,
    gamma: f32,
    t: f32,
}

impl Quintic {
    fn new(d0: f32, v0: f32, a0: f32, t: f32) -> Quintic {
        let b1 = v0 * t;
        let b2 = 0.5 * a0 * t * t;
        Quintic {
            alpha: d0,
            beta: b1 + 3.0 * d0,
            gamma: b2 + 3.0 * b1 + 6.0 * d0,
            t,
        }
    }

    /// State at `s` ∈ [0, T] seconds (clamped): with w = 1 − σ and
    /// g = α + βσ + γσ², d = w³·g, ḋ = w²·(w·(β + 2γσ) − 3g)/T and
    /// d̈ = w·(6g − 6w·(β + 2γσ) + 2γ·w²)/T².
    fn at(&self, s: f32) -> DState {
        let valid = self.t > 0.0; // false for NaN
        if !valid {
            return DState::default();
        }
        let sg = fmath::clampf(s / self.t, 0.0, 1.0);
        let w = 1.0 - sg;
        let g = self.alpha + sg * (self.beta + sg * self.gamma);
        let gp = self.beta + 2.0 * self.gamma * sg;
        DState {
            d_m: w * w * w * g,
            rate_mps: w * w * (w * gp - 3.0 * g) / self.t,
            accel_mps2: w * (6.0 * g - 6.0 * w * gp + 2.0 * self.gamma * w * w) / (self.t * self.t),
        }
    }

    /// Re-plan feasibility on 33 samples: never past the target (d ≥ −1 nm), monotone when
    /// `monotone`, |d̈| ≤ `a_lim` and |ḋ| ≤ `v_lim` (when positive).
    fn feasible(&self, monotone: bool, a_lim: f32, v_lim: f32) -> bool {
        let n = f32::from(QUINTIC_CHECK_SAMPLES);
        let rate_tol = 1.0e-6 * (fmath::absf(self.beta) + fmath::absf(self.alpha)) / self.t;
        (0..=QUINTIC_CHECK_SAMPLES).all(|i| {
            let st = self.at(self.t * f32::from(i) / n);
            st.d_m >= -1.0e-9
                && (!monotone || st.rate_mps <= rate_tol)
                && fmath::absf(st.accel_mps2) <= a_lim
                && (v_lim <= 0.0 || fmath::absf(st.rate_mps) <= v_lim)
        })
    }
}

/// Handover distance d_h = min(d_ε, d_b) and capture duration t_e = τ·ln(d_b/d_h) of a
/// terminus (the capture is skipped when d_b ≤ d_ε).
#[inline]
fn capture(seg: &TerminusSeg) -> (f32, f32) {
    let d_h = if seg.d_eps_m > 0.0 {
        fmath::fminf(seg.d_eps_m, seg.d_b_m)
    } else {
        seg.d_b_m
    };
    let t_e = if d_h > 0.0 && seg.d_b_m > d_h {
        seg.tau_s * fmath::lnf(seg.d_b_m / d_h)
    } else {
        0.0
    };
    (d_h, t_e)
}

/// Phase boundaries measured from t0: (d_h, capture end t_b + t_e, arrival
/// t_b + t_e + T_q). [`eval_d`], [`phase`] and [`t_arrive_s`] all compare against these
/// same f32 values, so the reference is exactly at rest from `t_arrive_s` on.
#[inline]
fn bounds(seg: &TerminusSeg) -> (f32, f32, f32) {
    let (d_h, t_e) = capture(seg);
    let t_ce = seg.t_b_s + t_e;
    (d_h, t_ce, t_ce + seg.t_q_s)
}

/// A terminus is well formed: finite, τ > 0, T_q > 0, non-negative distances and blend
/// time, and j > 0 whenever there is a blend.
fn seg_valid(seg: &TerminusSeg) -> bool {
    let f = [
        seg.v_app_mps,
        seg.j_mps3,
        seg.tau_s,
        seg.t_b_s,
        seg.d_b_m,
        seg.d_0_m,
        seg.d_eps_m,
        seg.t_q_s,
    ];
    f.iter().all(|x| x.is_finite())
        && seg.tau_s > 0.0
        && seg.t_q_s > 0.0
        && seg.t_b_s >= 0.0
        && seg.d_b_m >= 0.0
        && seg.d_0_m >= 0.0
        && seg.v_app_mps >= 0.0
        && (seg.t_b_s == 0.0 || seg.j_mps3 > 0.0)
}

/// A quintic is well formed: finite, T > 0 and d0 ≥ 0.
fn quintic_valid(q: &QuinticSeg) -> bool {
    [q.d0_m, q.v0_mps, q.a0_mps2, q.t_q_s]
        .iter()
        .all(|x| x.is_finite())
        && q.t_q_s > 0.0
        && q.d0_m >= 0.0
}

/// Phase of a terminus reference.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum TerminusPhase {
    /// Before t0: cruise at v_app toward the blend start.
    #[default]
    Cruise = 0,
    /// Constant-jerk blend (0 ≤ s < t_b).
    Blend = 1,
    /// Exponential capture d = d_b·e^(−(s − t_b)/τ), down to d_ε.
    Capture = 2,
    /// Quintic from d_ε to rest.
    Quintic = 3,
    /// At the target at rest (also: a malformed segment).
    Arrived = 4,
}

/// Distance-to-go of a terminus `s` seconds after its t0, in continuous time (the
/// exact phase functions; [`eval`] samples it at a tick). A malformed segment holds its
/// blend-start distance at rest.
pub fn eval_d(seg: &TerminusSeg, s: f32) -> DState {
    if !seg_valid(seg) || !s.is_finite() {
        let d = if seg.d_0_m.is_finite() {
            fmath::fmaxf(seg.d_0_m, 0.0)
        } else {
            0.0
        };
        return DState {
            d_m: d,
            rate_mps: 0.0,
            accel_mps2: 0.0,
        };
    }
    let v = seg.v_app_mps;
    let j = seg.j_mps3;
    let tau = seg.tau_s;
    if s < 0.0 {
        // Cruise extrapolation before the blend.
        return DState {
            d_m: seg.d_0_m - v * s,
            rate_mps: -v,
            accel_mps2: 0.0,
        };
    }
    if s < seg.t_b_s {
        // Constant-jerk blend: d̈ = j·s, ḋ = −v + j·s²/2, d = d_0 − v·s + j·s³/6.
        return DState {
            d_m: seg.d_0_m - v * s + j * s * s * s / 6.0,
            rate_mps: -v + 0.5 * j * s * s,
            accel_mps2: j * s,
        };
    }
    let (d_h, t_ce, t_end) = bounds(seg);
    if s >= t_end {
        return DState::default();
    }
    if s >= t_ce {
        // Quintic hand-over from (d_h, −d_h/τ, d_h/τ²) to rest in T_q.
        return Quintic::new(d_h, -d_h / tau, d_h / (tau * tau), seg.t_q_s).at(s - t_ce);
    }
    // Exponential capture d = d_b·e^(−(s − t_b)/τ) (the log-damped law).
    let d = seg.d_b_m * fmath::expf(-(s - seg.t_b_s) / tau);
    DState {
        d_m: d,
        rate_mps: -d / tau,
        accel_mps2: d / (tau * tau),
    }
}

/// Distance-to-go of a quintic `s` seconds after its t0 (before t0: the start state
/// extrapolated at constant rate; after T: rest at the target). A malformed quintic holds
/// its start distance at rest.
pub fn eval_quintic_d(q: &QuinticSeg, s: f32) -> DState {
    if !quintic_valid(q) || !s.is_finite() {
        let d = if q.d0_m.is_finite() {
            fmath::fmaxf(q.d0_m, 0.0)
        } else {
            0.0
        };
        return DState {
            d_m: d,
            rate_mps: 0.0,
            accel_mps2: 0.0,
        };
    }
    if s < 0.0 {
        return DState {
            d_m: q.d0_m + q.v0_mps * s,
            rate_mps: q.v0_mps,
            accel_mps2: 0.0,
        };
    }
    if s >= q.t_q_s {
        return DState::default();
    }
    Quintic::new(q.d0_m, q.v0_mps, q.a0_mps2, q.t_q_s).at(s)
}

/// Evaluates a terminus (cruise before t0, blend, exponential capture, then the quintic
/// hand-over) at tick `t` in closed form: (x, v along +x, a along +x).
pub fn eval(seg: &TerminusSeg, t: Tick) -> (PosNm, f32, f32) {
    to_x(seg.target, seg.dir, eval_d(seg, rel_s(t, seg.t0)))
}

/// Evaluates a quintic capture at tick `t`: (x, v, a).
pub fn eval_quintic(q: &QuinticSeg, t: Tick) -> (PosNm, f32, f32) {
    to_x(q.target, q.dir, eval_quintic_d(q, rel_s(t, q.t0)))
}

/// Closed-form arrival time of a terminus from its start tick, s:
/// t_b + τ·ln(d_b/d_ε) + T_q (the capture term is 0 when d_b ≤ d_ε; 0 for a malformed
/// segment).
pub fn t_arrive_s(seg: &TerminusSeg) -> f32 {
    if !seg_valid(seg) {
        return 0.0;
    }
    bounds(seg).2
}

/// First tick at which a segment's reference is at its target at rest (a terminus:
/// t0 + ⌈t_arrive⌉; a quintic: t0 + ⌈T⌉; an S-curve: its t0, since its duration belongs
/// to `scurve`).
pub fn arrive_tick(seg: &Segment) -> Tick {
    match seg {
        Segment::Terminus(s) => s.t0.plus(ticks_ceil(t_arrive_s(s))),
        Segment::Quintic(q) if quintic_valid(q) => q.t0.plus(ticks_ceil(q.t_q_s)),
        Segment::Quintic(q) => q.t0,
        Segment::SCurve(s) => s.t0,
    }
}

/// Phase of a terminus at tick `t`.
pub fn phase(seg: &TerminusSeg, t: Tick) -> TerminusPhase {
    if !seg_valid(seg) {
        return TerminusPhase::Arrived;
    }
    let s = rel_s(t, seg.t0);
    if s < 0.0 {
        return TerminusPhase::Cruise;
    }
    if s < seg.t_b_s {
        return TerminusPhase::Blend;
    }
    let (_, t_ce, t_end) = bounds(seg);
    if s >= t_end {
        TerminusPhase::Arrived
    } else if s >= t_ce {
        TerminusPhase::Quintic
    } else {
        TerminusPhase::Capture
    }
}

/// Integrator-freeze request (plan §4b; FAB-HAP-015): true while a terminus is in its
/// blend or exponential capture (the cascade then holds its velocity integrator
/// bumplessly, error poles (s + ω_v/2)²); false in cruise, from d_ε on (the quintic
/// re-enables it) and for any other segment. The fast loop ANDs it with
/// `TerminusCfg::freeze_integrator`.
pub fn integrator_frozen(seg: &Segment, t: Tick) -> bool {
    match seg {
        Segment::Terminus(s) => {
            matches!(phase(s, t), TerminusPhase::Blend | TerminusPhase::Capture)
        }
        _ => false,
    }
}

/// The segment's reference has finished at tick `t` (terminus/quintic: at rest on the
/// target; S-curve: as `scurve::eval` reports).
pub fn done(seg: &Segment, t: Tick) -> bool {
    match seg {
        Segment::Terminus(s) => phase(s, t) == TerminusPhase::Arrived,
        Segment::Quintic(q) => !quintic_valid(q) || rel_s(t, q.t0) >= q.t_q_s,
        Segment::SCurve(s) => crate::motion::scurve::eval(s, t).done,
    }
}

/// Governor observation point for a commanded terminus: x_obs = target + dir·overtravel
/// (plan §4b), so the braking envelope √(2·a_brk·(x_obs − x − margin)) never clips the
/// exponential tail (overtravel > margin in every preset).
pub fn x_obs(target: PosNm, dir: TravelDir, overtravel_m: f32) -> PosNm {
    let o = if overtravel_m.is_finite() {
        fmath::fmaxf(overtravel_m, 0.0)
    } else {
        0.0
    };
    target.offset_m(dir.sign() * o)
}

/// How a plan was obtained (see the module docs).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum PlanKind {
    /// Nominal terminus (after a cruise at the measured speed when d > d_0).
    #[default]
    Nominal = 0,
    /// Terminus with τ shortened so that d_0 = d; peak deceleration in (a_max, a_brk].
    Braked = 1,
    /// From (nearly) rest: S-curve launch in `seg`, then the terminus in `next`.
    Approach = 2,
    /// Stand-alone quintic (inside d_ε, or receding).
    Quintic = 3,
    /// No log-damped plan fits within a_brk: the caller falls back to core SoftClose;
    /// `seg` is a best-effort quintic from the measured state.
    Infeasible = 4,
}

/// A planned reference.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Plan {
    /// The segment to run now.
    pub seg: Segment,
    /// How it was obtained.
    pub kind: PlanKind,
    /// τ of the terminus (τ_eff, or the braked τ′); for a quintic the τ that set T_q, s.
    pub tau_s: f32,
    /// Tick at which the reference reaches the target at rest.
    pub arrive: Tick,
    /// `Approach` only: the terminus to switch to once the launch S-curve cruises (at any
    /// release in [`Plan::switch_from`, next.t0]; the terminus extrapolates the cruise).
    pub next: Option<TerminusSeg>,
    /// `Approach` only: the first tick at which the launch cruises at v_app.
    pub switch_from: Tick,
}

/// Terminus planner (f32 copies of `control.motion.terminus` and the motion limits).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TerminusPlanner {
    /// Nominal τ, s.
    pub tau_s: f32,
    /// Capture distance d_ε, m.
    pub d_eps_m: f32,
    /// Blend jerk j, m/s³.
    pub j_mps3: f32,
    /// T_q/τ.
    pub t_q_ratio: f32,
    /// a_max, m/s² (τ_min rule).
    pub a_max_mps2: f32,
    /// Re-plan braking ratio a_brk/a_max (2).
    pub replan_brk_ratio: f32,
    /// Governor overtravel beyond the target (x_obs = target + overtravel), m.
    pub overtravel_m: f32,
    /// Shortest braked τ′ = 2/ω_v (the frozen-integrator error poles ω_v/2 stay at least
    /// as fast as the reference), s; 0 = no floor.
    pub tau_floor_s: f32,
    /// Class opening speed cap (quintic and approach speed limit), m/s.
    pub v_open_cap_mps: f32,
    /// Class closing speed cap, m/s.
    pub v_close_cap_mps: f32,
    /// Re-plans treat |v| below this as at rest (the soft-close final speed bound,
    /// HAP-007: 20 / 50 / 20 / 10 mm/s), m/s.
    pub v_rest_mps: f32,
}

/// `Ok(x as f32)` when `x` is finite and positive, else the error for `path`.
fn positive(x: f64, path: &'static str) -> Result<f32, ParamError> {
    if x.is_finite() && x > 0.0 {
        Ok(x as f32)
    } else {
        Err(ParamError::new(path, "must be finite and > 0"))
    }
}

impl TerminusPlanner {
    /// Planner from `control.motion` (terminus block, a_max, ω_v), the class speed caps
    /// and the soft-close final speed. Rejects a non-positive τ, d_ε, j, T_q/τ or a_max, a braking ratio below 1 or
    /// a negative overtravel.
    pub fn new(p: &SystemParams) -> Result<TerminusPlanner, ParamError> {
        let mo = &p.control.motion;
        let t = &mo.terminus;
        let tau_s = positive(t.tau_s, "control.motion.terminus.tau_s")?;
        let d_eps_m = positive(t.d_eps_m, "control.motion.terminus.d_eps_m")?;
        let j_mps3 = positive(t.j_mps3, "control.motion.terminus.j_mps3")?;
        let t_q_ratio = positive(t.t_q_ratio, "control.motion.terminus.t_q_ratio")?;
        let a_max_mps2 = positive(mo.a_max_mps2, "control.motion.a_max_mps2")?;
        if !(t.replan_brk_ratio.is_finite() && t.replan_brk_ratio >= 1.0) {
            return Err(ParamError::new(
                "control.motion.terminus.replan_brk_ratio",
                "must be finite and >= 1",
            ));
        }
        if !(t.overtravel_m.is_finite() && t.overtravel_m >= 0.0) {
            return Err(ParamError::new(
                "control.motion.terminus.overtravel_m",
                "must be finite and >= 0",
            ));
        }
        // 2/ω_v with ω_v = 2π·f_v: the braked τ′ keeps ω_v/2 ≥ 1/τ′ (plan §4b uses 3/τ
        // for the nominal τ; a re-plan relaxes it); design choice.
        let tau_floor_s = if mo.omega_v_hz.is_finite() && mo.omega_v_hz > 0.0 {
            (1.0 / (PI * mo.omega_v_hz)) as f32
        } else {
            0.0
        };
        let cap = |v: f64| {
            if v.is_finite() && v > 0.0 {
                v as f32
            } else {
                0.0
            }
        };
        Ok(TerminusPlanner {
            tau_s,
            d_eps_m,
            j_mps3,
            t_q_ratio,
            a_max_mps2,
            replan_brk_ratio: t.replan_brk_ratio as f32,
            overtravel_m: t.overtravel_m as f32,
            tau_floor_s,
            v_open_cap_mps: cap(p.safety.v_open_max_mps),
            v_close_cap_mps: cap(p.safety.v_close_max_mps),
            v_rest_mps: cap(p.control.haptic.soft_close_v_final_mps),
        })
    }

    /// τ_min(|v|) = |v|/a_max − a_max/(2j) with this planner's a_max and j, s.
    pub fn tau_min(&self, v_mps: f32) -> f32 {
        tau_min(v_mps, self.a_max_mps2, self.j_mps3)
    }

    /// τ_eff = max(τ, τ_min(|v_measured|)), s (plan §4b: covers hand-thrown drawers).
    pub fn tau_eff(&self, v_meas_mps: f32) -> f32 {
        let tm = self.tau_min(v_meas_mps);
        if tm.is_finite() {
            fmath::fmaxf(self.tau_s, tm)
        } else {
            self.tau_s
        }
    }

    /// Re-plan braking deceleration a_brk = replan_brk_ratio·a_max (2·a_max), m/s².
    pub fn a_brk_mps2(&self) -> f32 {
        fmath::fmaxf(self.replan_brk_ratio, 1.0) * self.a_max_mps2
    }

    /// Class speed cap in `dir`, m/s (0 = unknown).
    pub fn v_cap_mps(&self, dir: TravelDir) -> f32 {
        match dir {
            TravelDir::Open => self.v_open_cap_mps,
            TravelDir::Close => self.v_close_cap_mps,
        }
    }

    /// Governor observation point for a terminus toward `target` in `dir`
    /// (x_obs = target + dir·overtravel).
    pub fn x_obs(&self, target: PosNm, dir: TravelDir) -> PosNm {
        x_obs(target, dir, self.overtravel_m)
    }

    /// T_q for time constant τ (T_q/τ from the tuning; τ itself if the ratio is unset).
    fn t_q(&self, tau_s: f32) -> f32 {
        if self.t_q_ratio > 0.0 && self.t_q_ratio.is_finite() {
            self.t_q_ratio * tau_s
        } else {
            tau_s
        }
    }

    /// A terminus segment from its blend.
    #[allow(clippy::too_many_arguments)]
    fn terminus_seg(
        &self,
        target: PosNm,
        dir: TravelDir,
        t0: Tick,
        v_mps: f32,
        j_mps3: f32,
        tau_s: f32,
        b: Blend,
    ) -> TerminusSeg {
        TerminusSeg {
            target,
            t0,
            dir,
            v_app_mps: fmath::absf(v_mps),
            j_mps3,
            tau_s,
            t_b_s: b.t_b_s,
            d_b_m: b.d_b_m,
            d_0_m: b.d_0_m,
            d_eps_m: self.d_eps_m,
            t_q_s: self.t_q(tau_s),
        }
    }

    /// The nominal terminus out of a cruise at `v_app_mps` with τ_eff(v_app) and the
    /// tuned jerk, blending at `t_blend` (distance-to-go d_0 then). Before `t_blend` it
    /// extrapolates the cruise, so a supervisor may switch to it at any release while the
    /// cruise S-curve is at v_app.
    pub fn nominal(
        &self,
        target: PosNm,
        dir: TravelDir,
        t_blend: Tick,
        v_app_mps: f32,
    ) -> TerminusSeg {
        let v = fmath::absf(v_app_mps);
        let tau = self.tau_eff(v);
        self.terminus_seg(
            target,
            dir,
            t_blend,
            v,
            self.j_mps3,
            tau,
            blend(v, self.j_mps3, tau),
        )
    }

    /// Plans a terminus toward `target` in direction `dir`, starting at tick `t0` from
    /// distance-to-go `d_m` (≥ 0), approach speed magnitude `v_mps` and acceleration
    /// `a_mps2` (along `dir`), with time constant `tau_s` (callers pass
    /// [`Self::tau_eff`]) and blend jerk `j_mps3`. Returns `Segment::Terminus` (blend and
    /// capture), or `Segment::Quintic` when already inside d_ε ([`Self::plan_detailed`]
    /// has the kind; for a start from rest this returns the launch S-curve, whose own end
    /// is the target).
    #[allow(clippy::too_many_arguments)]
    pub fn plan(
        &self,
        target: PosNm,
        dir: TravelDir,
        t0: Tick,
        d_m: f32,
        v_mps: f32,
        a_mps2: f32,
        tau_s: f32,
        j_mps3: f32,
    ) -> Segment {
        self.plan_detailed(target, dir, t0, d_m, v_mps, a_mps2, tau_s, j_mps3)
            .seg
    }

    /// Re-plan from the measured (d, v, a) with τ_eff(v) and the tuned jerk (plan §4b:
    /// after a hand release; a_brk = 2·a_max if d < d_0; `Infeasible` → core SoftClose).
    pub fn replan(
        &self,
        target: PosNm,
        dir: TravelDir,
        t0: Tick,
        d_m: f32,
        v_mps: f32,
        a_mps2: f32,
    ) -> Plan {
        let tau = self.tau_eff(v_mps);
        self.plan_detailed(target, dir, t0, d_m, v_mps, a_mps2, tau, self.j_mps3)
    }

    /// [`Self::plan`] with the plan kind, τ used and arrival (see the module docs for the
    /// cases). `v_mps` > 0 approaches the target; `a_mps2` > 0 accelerates toward it.
    /// Non-finite τ or j fall back to the tuned values; a non-finite d gives
    /// `Infeasible` with a hold at the target.
    #[allow(clippy::too_many_arguments)]
    pub fn plan_detailed(
        &self,
        target: PosNm,
        dir: TravelDir,
        t0: Tick,
        d_m: f32,
        v_mps: f32,
        a_mps2: f32,
        tau_s: f32,
        j_mps3: f32,
    ) -> Plan {
        if !d_m.is_finite() {
            let hold = Segment::SCurve(SCurveSeg {
                anchor: target,
                t0,
                dir,
                ..SCurveSeg::default()
            });
            return Plan {
                seg: hold,
                kind: PlanKind::Infeasible,
                arrive: t0,
                ..Plan::default()
            };
        }
        let d = fmath::fmaxf(d_m, 0.0);
        let v = if v_mps.is_finite() { v_mps } else { 0.0 };
        let a = if a_mps2.is_finite() { a_mps2 } else { 0.0 };
        let tau = if tau_s.is_finite() && tau_s > 0.0 {
            tau_s
        } else {
            self.tau_s
        };
        let j = if j_mps3.is_finite() && j_mps3 > 0.0 {
            j_mps3
        } else {
            self.j_mps3
        };
        let d_eps = self.d_eps_m;
        if !(tau > 0.0 && j > 0.0 && d_eps > 0.0) {
            // Unusable tuning (a default planner): best-effort quintic from the state.
            let mut p = self.quintic_plan(target, dir, t0, d, v, a, tau);
            p.kind = PlanKind::Infeasible;
            return p;
        }
        if d <= d_eps {
            return self.quintic_plan(target, dir, t0, d, v, a, tau);
        }
        // Below the soft-close final speed (or d_ε/τ, the slowest speed a terminus commands
        // before its quintic, if larger) the carriage counts as at rest.
        let v_rest = fmath::fmaxf(d_eps / tau, self.v_rest_mps);
        if v >= v_rest {
            let b = blend(v, j, tau);
            if d >= b.d_0_m {
                // Cruise at v, then the nominal terminus.
                let start = t0.plus(ticks_round((d - b.d_0_m) / v));
                let seg = self.terminus_seg(target, dir, start, v, j, tau, b);
                return Plan {
                    seg: Segment::Terminus(seg),
                    kind: PlanKind::Nominal,
                    tau_s: tau,
                    arrive: arrive_tick(&Segment::Terminus(seg)),
                    ..Plan::default()
                };
            }
            // Inside d_0: shorten τ, keeping the peak deceleration j·t_b ≤ a_brk.
            let tau_lo = fmath::fmaxf(
                fmath::fmaxf(tau_min(v, self.a_brk_mps2(), j), self.tau_floor_s),
                1.0e-3 * tau,
            );
            if tau_lo < tau && blend(v, j, tau_lo).d_0_m <= d {
                let (mut lo, mut hi) = (tau_lo, tau);
                for _ in 0..BISECT_STEPS {
                    let mid = 0.5 * (lo + hi);
                    if blend(v, j, mid).d_0_m <= d {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                let seg = self.terminus_seg(target, dir, t0, v, j, lo, blend(v, j, lo));
                return Plan {
                    seg: Segment::Terminus(seg),
                    kind: PlanKind::Braked,
                    tau_s: lo,
                    arrive: arrive_tick(&Segment::Terminus(seg)),
                    ..Plan::default()
                };
            }
            let mut p = self.quintic_plan(target, dir, t0, d, v, a, tau);
            p.kind = PlanKind::Infeasible;
            return p;
        }
        if v > -v_rest
            && let Some(p) = self.approach(target, dir, t0, d, self.v_cap_mps(dir))
        {
            return p;
        }
        self.quintic_plan(target, dir, t0, d, v, a, tau)
    }

    /// Log-damped move from rest over distance-to-go `d_m`: a jerk-limited S-curve
    /// launch (a_max, j) to the fastest v_app ≤ `v_cap_mps` (the class cap if ≤ 0) for
    /// which launch distance + d_0(v_app, τ_eff(v_app)) ≤ d, cruising until the blend,
    /// then the nominal terminus. The launch is planned over the whole distance (so on
    /// its own it still ends at the target) and assumes the standard symmetric 7-segment
    /// profile (cruise reached after a/j + v/a, or 2·√(v/j) below a²/j). `None` when no
    /// positive v_app fits (d ≤ d_ε, or unusable limits).
    pub fn approach(
        &self,
        target: PosNm,
        dir: TravelDir,
        t0: Tick,
        d_m: f32,
        v_cap_mps: f32,
    ) -> Option<Plan> {
        let v_cap = if v_cap_mps.is_finite() && v_cap_mps > 0.0 {
            v_cap_mps
        } else {
            self.v_cap_mps(dir)
        };
        let (a, j) = (self.a_max_mps2, self.j_mps3);
        if !(d_m.is_finite() && d_m > self.d_eps_m && v_cap > 0.0 && a > 0.0 && j > 0.0) {
            return None;
        }
        let fits = |v: f32| launch(v, a, j).1 + blend(v, j, self.tau_eff(v)).d_0_m <= d_m;
        let v_app = if fits(v_cap) {
            v_cap
        } else {
            let (mut lo, mut hi) = (0.0f32, v_cap);
            for _ in 0..BISECT_STEPS {
                let mid = 0.5 * (lo + hi);
                if fits(mid) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            lo
        };
        let moving = v_app > 0.0; // false for NaN
        if !moving {
            return None;
        }
        let tau = self.tau_eff(v_app);
        let b = blend(v_app, j, tau);
        let (t_acc, s_acc) = launch(v_app, a, j);
        if b.d_0_m < s_acc {
            // The blend would start inside the launch S-curve's own deceleration.
            return None;
        }
        let t_cruise = fmath::fmaxf(d_m - s_acc - b.d_0_m, 0.0) / v_app;
        let term = self.terminus_seg(
            target,
            dir,
            t0.plus(ticks_round(t_acc + t_cruise)),
            v_app,
            j,
            tau,
            b,
        );
        let launch_seg = SCurveSeg {
            anchor: target.offset_m(-dir.sign() * d_m),
            t0,
            dir,
            dist_m: d_m,
            v_mps: v_app,
            a_mps2: a,
            j_mps3: j,
        };
        Some(Plan {
            seg: Segment::SCurve(launch_seg),
            kind: PlanKind::Approach,
            tau_s: tau,
            arrive: arrive_tick(&Segment::Terminus(term)),
            next: Some(term),
            switch_from: t0.plus(ticks_ceil(t_acc)),
        })
    }

    /// Stand-alone quintic from the measured state (d, ḋ = −v, d̈ = −a): T searched on
    /// T_q·2^(k/4), |k| ≤ 24, nearest T_q first, for no overshoot, a monotone approach
    /// (unless receding), |d̈| ≤ a_brk and |ḋ| ≤ max(v_cap, |v|). `Infeasible` with
    /// T = T_q when nothing fits.
    #[allow(clippy::too_many_arguments)]
    fn quintic_plan(
        &self,
        target: PosNm,
        dir: TravelDir,
        t0: Tick,
        d: f32,
        v: f32,
        a: f32,
        tau: f32,
    ) -> Plan {
        let t_base = if tau > 0.0 { self.t_q(tau) } else { 0.0 };
        let a_lim = self.a_brk_mps2();
        let cap = self.v_cap_mps(dir);
        let v_lim = if cap > 0.0 {
            fmath::fmaxf(cap, fmath::absf(v)) * 1.001
        } else {
            0.0
        };
        let monotone = v >= 0.0;
        let seg_for = |t: f32| QuinticSeg {
            target,
            t0,
            dir,
            d0_m: d,
            v0_mps: -v,
            a0_mps2: -a,
            t_q_s: t,
        };
        if t_base > 0.0 && a_lim > 0.0 {
            for step in 0..=(2 * QUINTIC_SEARCH_STEPS) {
                // 0, +1, −1, +2, −2, … quarter-octaves around T_q.
                let k = if step % 2 == 1 {
                    (step + 1) / 2
                } else {
                    -(step / 2)
                };
                let t = t_base * fmath::powf(2.0, k as f32 / 4.0);
                if Quintic::new(d, -v, -a, t).feasible(monotone, a_lim, v_lim) {
                    let q = Segment::Quintic(seg_for(t));
                    return Plan {
                        seg: q,
                        kind: PlanKind::Quintic,
                        tau_s: tau,
                        arrive: arrive_tick(&q),
                        ..Plan::default()
                    };
                }
            }
        }
        let t = if t_base > 0.0 { t_base } else { 1.0 };
        let q = Segment::Quintic(seg_for(t));
        Plan {
            seg: q,
            kind: PlanKind::Infeasible,
            tau_s: tau,
            arrive: arrive_tick(&q),
            ..Plan::default()
        }
    }

    /// Evaluates a planned segment (any kind) at `t`: (x, v, a).
    pub fn eval(&self, seg: &Segment, t: Tick) -> (PosNm, f32, f32) {
        let r = crate::motion::eval_segment(seg, t);
        (r.x, r.v_mps, r.a_mps2)
    }
}

/// Test-only linear algebra shared by the terminus and inertial pole tests (f64, std):
/// matrix exponential (ZOH discretisation), characteristic polynomial, polynomial roots
/// and the spectral radius.
#[cfg(test)]
#[allow(clippy::needless_range_loop)]
pub(crate) mod lin {
    /// Complex number (re, im).
    pub type C = (f64, f64);

    fn cmul(a: C, b: C) -> C {
        (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
    }

    fn cdiv(a: C, b: C) -> C {
        let den = b.0 * b.0 + b.1 * b.1;
        ((a.0 * b.0 + a.1 * b.1) / den, (a.1 * b.0 - a.0 * b.1) / den)
    }

    /// Dense n×n matrix (row-major).
    pub type Mat = Vec<Vec<f64>>;

    /// Zero matrix.
    pub fn zeros(n: usize) -> Mat {
        vec![vec![0.0; n]; n]
    }

    /// A·B.
    pub fn mul(a: &Mat, b: &Mat) -> Mat {
        let n = a.len();
        let mut c = zeros(n);
        for i in 0..n {
            for k in 0..n {
                let aik = a[i][k];
                if aik == 0.0 {
                    continue;
                }
                for j in 0..n {
                    c[i][j] += aik * b[k][j];
                }
            }
        }
        c
    }

    fn norm1(a: &Mat) -> f64 {
        let n = a.len();
        (0..n)
            .map(|j| (0..n).map(|i| a[i][j].abs()).sum::<f64>())
            .fold(0.0, f64::max)
    }

    /// e^A by scaling and squaring with a 20-term Taylor series (well within f64 for the
    /// ‖A‖ ≤ 1/2 it is applied at).
    pub fn expm(a: &Mat) -> Mat {
        let n = a.len();
        let nrm = norm1(a);
        let mut sq = 0i32;
        while nrm * 0.5f64.powi(sq) > 0.5 && sq < 200 {
            sq += 1;
        }
        let scale = 0.5f64.powi(sq);
        let x: Mat = a
            .iter()
            .map(|r| r.iter().map(|v| v * scale).collect())
            .collect();
        let mut e = zeros(n);
        let mut term = zeros(n);
        for i in 0..n {
            e[i][i] = 1.0;
            term[i][i] = 1.0;
        }
        for k in 1..=20 {
            term = mul(&term, &x);
            for row in term.iter_mut() {
                for v in row.iter_mut() {
                    *v /= f64::from(k);
                }
            }
            for i in 0..n {
                for j in 0..n {
                    e[i][j] += term[i][j];
                }
            }
        }
        for _ in 0..sq {
            e = mul(&e, &e);
        }
        e
    }

    /// Characteristic polynomial det(λI − A) by Faddeev–LeVerrier, coefficients from λ⁰
    /// up (the λⁿ coefficient is 1). Used on small (n ≤ 6) well-scaled matrices only.
    pub fn charpoly(a: &Mat) -> Vec<f64> {
        let n = a.len();
        let mut c = vec![0.0; n + 1];
        c[n] = 1.0;
        let mut m = zeros(n);
        for k in 1..=n {
            // M_k = A·M_{k−1} + c_{n−k+1}·I
            let mut mk = mul(a, &m);
            for (i, row) in mk.iter_mut().enumerate() {
                row[i] += c[n - k + 1];
            }
            let am = mul(a, &mk);
            let tr: f64 = (0..n).map(|i| am[i][i]).sum();
            c[n - k] = -tr / k as f64;
            m = mk;
        }
        c
    }

    /// Roots of a monic polynomial (coefficients from x⁰ up) by Aberth–Ehrlich iteration.
    pub fn roots(c: &[f64]) -> Vec<C> {
        let n = c.len() - 1;
        let lead = c[n];
        let cn: Vec<f64> = c.iter().map(|v| v / lead).collect();
        // Fujiwara bound 2·max_k |c_{n−k}|^(1/k) on the root moduli.
        let bound = 2.0
            * (1..=n)
                .map(|k| cn[n - k].abs().powf(1.0 / k as f64))
                .fold(1e-300f64, f64::max);
        let mut z: Vec<C> = (0..n)
            .map(|k| {
                let th = 2.0 * std::f64::consts::PI * (k as f64 + 0.25) / n as f64 + 0.4;
                (0.5 * bound * th.cos(), 0.5 * bound * th.sin())
            })
            .collect();
        let eval = |x: C| -> (C, C) {
            let mut p: C = (cn[n], 0.0);
            let mut dp: C = (0.0, 0.0);
            for k in (0..n).rev() {
                dp = cmul(dp, x);
                dp = (dp.0 + p.0, dp.1 + p.1);
                p = cmul(p, x);
                p = (p.0 + cn[k], p.1);
            }
            (p, dp)
        };
        for _ in 0..500 {
            let mut moved = 0.0f64;
            for i in 0..n {
                let (p, dp) = eval(z[i]);
                if p.0 == 0.0 && p.1 == 0.0 {
                    continue;
                }
                let ratio = cdiv(p, dp);
                let mut s: C = (0.0, 0.0);
                for (k, zk) in z.iter().enumerate() {
                    if k != i {
                        let d = (z[i].0 - zk.0, z[i].1 - zk.1);
                        let inv = cdiv((1.0, 0.0), d);
                        s = (s.0 + inv.0, s.1 + inv.1);
                    }
                }
                let den = (1.0 - cmul(ratio, s).0, -cmul(ratio, s).1);
                let w = cdiv(ratio, den);
                z[i] = (z[i].0 - w.0, z[i].1 - w.1);
                moved = moved.max(w.0.abs() + w.1.abs());
            }
            if moved < 1e-15 * bound {
                break;
            }
        }
        z
    }

    /// Spectral radius ρ(A) = lim ‖A^N‖^(1/N) by repeated normalised squaring (N = 2^k):
    /// robust for clustered slow poles where root finding on a characteristic polynomial
    /// is ill-conditioned. Error ≈ ln(cond)/N; with k = 26 that is < 1e-6 relative.
    pub fn spectral_radius(a: &Mat, k: u32) -> f64 {
        let mut m = a.clone();
        let mut log_scale = 0.0f64; // ln of the factor removed so far (per unit power)
        let mut power = 1.0f64;
        for _ in 0..k {
            let s = norm1(&m);
            if !s.is_finite() || s == 0.0 {
                return if s == 0.0 { 0.0 } else { f64::INFINITY };
            }
            for row in m.iter_mut() {
                for v in row.iter_mut() {
                    *v /= s;
                }
            }
            log_scale += s.ln() / power;
            m = mul(&m, &m);
            power *= 2.0;
        }
        let s = norm1(&m);
        (log_scale + s.ln() / power).exp()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;
    use fabrail_types::RailClass;

    const DT: f64 = 1.0 / 20_000.0;

    fn planner(c: RailClass) -> TerminusPlanner {
        TerminusPlanner::new(&presets::by_class(c)).expect("preset planner")
    }

    /// The nominal soft-close terminus of a preset at its closing cap.
    fn nominal_close(c: RailClass) -> TerminusSeg {
        let p = presets::by_class(c);
        planner(c).nominal(
            PosNm(123_456_789),
            TravelDir::Close,
            Tick(4_000),
            p.safety.v_close_max_mps as f32,
        )
    }

    #[test]
    fn plan_is_continuous_at_start() {
        // Any plan starts where the carriage is (C0 at t0), even from a default planner.
        let p = SystemParams::default();
        let tp = TerminusPlanner::new(&p).unwrap_or_default();
        let target = PosNm(0);
        let seg = tp.plan(
            target,
            TravelDir::Close,
            Tick(0),
            0.05,
            0.2,
            0.0,
            0.15,
            20.0,
        );
        let (x, v, a) = tp.eval(&seg, Tick(0));
        // Closing toward 0 from d = 50 mm: the carriage is at +50 mm.
        assert!(
            x.diff_nm(PosNm(50_000_000)).unsigned_abs() <= 1_000,
            "x(t0) = {x:?}"
        );
        assert!(v.is_finite() && a.is_finite());
        // A preset planner too, for every class and both directions.
        for c in presets::CLASSES {
            let tp = planner(c);
            for dir in [TravelDir::Open, TravelDir::Close] {
                let v0 = tp.v_cap_mps(dir);
                let pl = tp.replan(target, dir, Tick(7), 0.2, v0, 0.0);
                let (x, _, _) = tp.eval(&pl.seg, Tick(7));
                let want = target.offset_m(-dir.sign() * 0.2);
                // (E) within one base tick of cruise, PHYSICS §16.1 (tick-rounded t0).
                let tol = (f64::from(v0) * DT * 1e9) as u64 + 2;
                assert!(
                    x.diff_nm(want).unsigned_abs() <= tol,
                    "{c:?} {dir:?}: x(t0) off by {} nm > {tol} (one tick of cruise)",
                    x.diff_nm(want)
                );
            }
        }
    }

    // req: FAB-HAP-013
    #[test]
    fn exponential_capture_exact() {
        // During capture the reference IS d_b·e^(−(t − t0 − t_b)/τ): compare every base
        // tick with the f64 closed form (f32 evaluation, rel ≤ 2e-5 + 1 nm), then fit
        // ln d against t and require slope −1/τ (±1e-4) and R² ≥ 0.999 999
        // (FAB-HAP-013 needs ≥ 0.999 on plant truth; PHYSICS §16.1).
        for c in presets::CLASSES {
            let seg = nominal_close(c);
            let (d_h, t_e) = capture(&seg);
            assert!(
                (f64::from(d_h) - f64::from(seg.d_eps_m)).abs() < 1e-12 && t_e > 0.0,
                "{c:?}: capture runs down to d_ε (plan §4b)"
            );
            let tau = f64::from(seg.tau_s);
            let t_b = f64::from(seg.t_b_s);
            let k0 = (t_b / DT).ceil() as u64;
            let k1 = ((t_b + f64::from(t_e)) / DT).floor() as u64 - 1;
            let (mut n, mut sx, mut sy, mut sxx, mut sxy, mut syy) =
                (0.0f64, 0.0, 0.0, 0.0, 0.0, 0.0);
            for k in k0..=k1 {
                let t = seg.t0.plus(k);
                let (x, v, a) = eval(&seg, t);
                let d = x.diff_m(seg.target) as f64 * f64::from(-seg.dir.sign());
                let s = k as f64 * DT;
                let want = f64::from(seg.d_b_m) * (-(s - t_b) / tau).exp();
                assert!(
                    (d - want).abs() <= 2e-5 * want + 1.0e-9,
                    "{c:?} tick {k}: d = {d} vs d_b·e^(−t/τ) = {want} (rel 2e-5 + 1 nm; PHYSICS §16.1)"
                );
                // ḋ = −d/τ and d̈ = d/τ² (the exponential's own derivatives).
                // x = target − sg·d ⇒ ḋ = −sg·ẋ, d̈ = −sg·ẍ.
                let rate = -f64::from(seg.dir.sign()) * f64::from(v);
                let acc = -f64::from(seg.dir.sign()) * f64::from(a);
                assert!(
                    (rate + want / tau).abs() <= 2e-5 * want / tau,
                    "{c:?}: ḋ = −d/τ (rel 2e-5; PHYSICS §16.1)"
                );
                assert!(
                    (acc - want / (tau * tau)).abs() <= 2e-5 * want / (tau * tau),
                    "{c:?}: d̈ = d/τ² (rel 2e-5; PHYSICS §16.1)"
                );
                let yd = d.ln();
                n += 1.0;
                sx += s;
                sy += yd;
                sxx += s * s;
                sxy += s * yd;
                syy += yd * yd;
            }
            let slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
            let r2 = (n * sxy - sx * sy).powi(2) / ((n * sxx - sx * sx) * (n * syy - sy * sy));
            assert!(
                (slope * tau + 1.0).abs() <= 1e-4,
                "{c:?}: semilog slope {slope} vs −1/τ = {} (±1e-4; plan §4b)",
                -1.0 / tau
            );
            assert!(
                r2 >= 0.999_999,
                "{c:?}: semilog R² = {r2} ≥ 0.999999 (FAB-HAP-013 needs 0.999; plan §4b)"
            );
        }
    }

    /// Left/right limits of (d, ḋ, d̈) across `s_joint` (±δ): each jump must be within
    /// what a bounded higher derivative explains over 2δ, plus f32 rounding.
    fn assert_c2_at(seg: &TerminusSeg, s_joint: f32, what: &str) {
        let delta = 2.0e-5f32;
        let l = eval_d(seg, s_joint - delta);
        let r = eval_d(seg, s_joint + delta);
        let scale_d = f64::from(seg.d_0_m) + 1e-9;
        let v = f64::from(seg.v_app_mps) + f64::from(seg.d_b_m / seg.tau_s);
        let a_max = f64::from(seg.j_mps3 * seg.t_b_s)
            + f64::from(seg.d_b_m / (seg.tau_s * seg.tau_s))
            + 1e-9;
        // Jerk bound: blend j, capture d_b/τ³, and the quintic's third derivative
        // ≤ (6|c3| + 24|c4| + 60|c5|)/T³ ≤ 780·d0/T³ + 408·|ḋ0|/T² + 75·|d̈0|/T.
        let tq = f64::from(seg.t_q_s);
        let dh = f64::from(capture(seg).0);
        let tau = f64::from(seg.tau_s);
        let jerk = f64::from(seg.j_mps3)
            + f64::from(seg.d_b_m) / tau.powi(3)
            + 780.0 * dh / tq.powi(3)
            + 408.0 * dh / tau / tq.powi(2)
            + 75.0 * dh / (tau * tau) / tq;
        let w = 2.0 * f64::from(delta);
        let dd = f64::from(r.d_m - l.d_m).abs();
        let dv = f64::from(r.rate_mps - l.rate_mps).abs();
        let da = f64::from(r.accel_mps2 - l.accel_mps2).abs();
        assert!(
            dd <= w * v * 1.01 + 4e-7 * scale_d,
            "{what}: C0 jump {dd} m at s = {s_joint} (PHYSICS §16.1)"
        );
        assert!(
            dv <= w * a_max * 1.01 + 4e-7 * v,
            "{what}: C1 jump {dv} m/s at s = {s_joint} (PHYSICS §16.1)"
        );
        assert!(
            da <= w * jerk * 1.01 + 4e-7 * a_max,
            "{what}: C2 jump {da} m/s² at s = {s_joint} (PHYSICS §16.1)"
        );
    }

    // req: FAB-HAP-014
    #[test]
    fn reference_is_c2() {
        // (E) C²: position, velocity and acceleration are continuous at every joint
        // (cruise→blend, blend→capture, capture→quintic, quintic→rest) for every preset,
        // for d_b ≤ d_ε (capture skipped), and for T_q/τ ∈ {1, 3, 6}; the joint states
        // equal the analytic hand-over values (PHYSICS §16.1; plan §4b).
        let mut segs: Vec<(String, TerminusSeg)> = Vec::new();
        for c in presets::CLASSES {
            segs.push((format!("{c:?}"), nominal_close(c)));
            let mut tp = planner(c);
            for r in [3.0f32, 6.0] {
                tp.t_q_ratio = r;
                let p = presets::by_class(c);
                segs.push((
                    format!("{c:?} T_q/τ={r}"),
                    tp.nominal(
                        PosNm(0),
                        TravelDir::Open,
                        Tick(0),
                        p.safety.v_open_max_mps as f32,
                    ),
                ));
            }
        }
        // d_b below d_ε: a creeping approach, the quintic starts at the blend end.
        let tp = planner(RailClass::Drawer);
        segs.push((
            "creep".into(),
            tp.nominal(PosNm(0), TravelDir::Close, Tick(0), 0.001),
        ));
        for (name, seg) in &segs {
            let (d_h, t_e) = capture(seg);
            let tb = seg.t_b_s;
            let joints = [0.0, tb, tb + t_e, tb + t_e + seg.t_q_s];
            for s in joints {
                assert_c2_at(seg, s, name);
            }
            // Blend end equals the exponential's start (d_b, −d_b/τ, d_b/τ²).
            let e = eval_d(seg, tb);
            let tau = seg.tau_s;
            let rel = |x: f32, y: f32| f64::from((x - y).abs()) / (f64::from(y.abs()) + 1e-12);
            assert!(
                rel(e.d_m, seg.d_b_m) < 1e-5
                    && rel(e.rate_mps, -seg.d_b_m / tau) < 1e-4
                    && rel(e.accel_mps2, seg.d_b_m / (tau * tau)) < 1e-4,
                "{name}: blend end = (d_b, −d_b/τ, d_b/τ²) (rel 1e-4; plan §4b): {e:?}"
            );
            // Just after the hand-over: the quintic starts at (d_h, −d_h/τ, d_h/τ²).
            let q = eval_d(seg, tb + t_e + 1.0e-7);
            assert!(
                rel(q.d_m, d_h) < 1e-4 && rel(q.rate_mps, -d_h / tau) < 1e-3,
                "{name}: quintic starts at the hand-over state (rel 1e-3; plan §4b): {q:?}"
            );
            // End: exactly at rest on the target.
            let z = eval_d(seg, t_arrive_s(seg));
            assert_eq!(z, DState::default(), "{name}: (E) rest at t_arrive");
            let (x, v, a) = eval(seg, arrive_tick(&Segment::Terminus(*seg)));
            assert!(
                x == seg.target && v == 0.0 && a == 0.0,
                "{name}: (E) x = target exactly at the arrival tick, v = a = 0"
            );
        }
    }

    // req: FAB-HAP-014
    #[test]
    fn quintic_monotone() {
        // The hand-over quintic from (d_ε, −d_ε/τ, d_ε/τ²) is monotone (ḋ ≤ 0), never
        // passes the target (d ≥ 0) and lands exactly at v = a = 0 for T_q/τ ∈ [1, 6]
        // (5 000 samples per ratio, ratios in steps of 0.25; plan §4b, PHYSICS §16.1).
        // Exact property in f64 on the normalised polynomial q(σ) = d/d_ε with
        // r = T_q/τ: q' ≤ 1e-12. The f32 evaluator is then held to its rounding level:
        // ḋ ≤ 1e-7·d_ε/τ and d ≥ −1e-6·d_ε (the factored form (1 − σ)³·g keeps the sign
        // of ḋ = (1 − σ)²·(…) near its double root at σ = 1).
        for i in 0..=20 {
            let r = 1.0 + 0.25 * f64::from(i);
            let (b1, b2) = (-r, 0.5 * r * r);
            let c3 = -10.0 - 6.0 * b1 - 3.0 * b2;
            let c4 = 15.0 + 8.0 * b1 + 3.0 * b2;
            let c5 = -6.0 - 3.0 * b1 - b2;
            for k in 0..=20_000 {
                let sg = f64::from(k) / 20_000.0;
                let q = 1.0 + sg * (b1 + sg * (b2 + sg * (c3 + sg * (c4 + sg * c5))));
                let dq = b1 + sg * (2.0 * b2 + sg * (3.0 * c3 + sg * (4.0 * c4 + sg * 5.0 * c5)));
                assert!(
                    dq <= 1e-12 && q >= -1e-12,
                    "T_q/τ = {r}: q'({sg}) = {dq}, q = {q} (exact monotone; plan §4b)"
                );
            }
        }
        let d_eps = 2.0e-4f32;
        for tau in [0.15f32, 0.25, 0.6, 1.0] {
            let mut r = 1.0f32;
            while r <= 6.0 + 1e-6 {
                let q = QuinticSeg {
                    target: PosNm(0),
                    t0: Tick(0),
                    dir: TravelDir::Close,
                    d0_m: d_eps,
                    v0_mps: -d_eps / tau,
                    a0_mps2: d_eps / (tau * tau),
                    t_q_s: r * tau,
                };
                let n = 5_000;
                let mut prev = f32::INFINITY;
                for i in 0..=n {
                    let s = q.t_q_s * i as f32 / n as f32;
                    let st = eval_quintic_d(&q, s);
                    assert!(
                        st.d_m >= -1e-6 * d_eps,
                        "τ {tau}, T_q/τ {r}: overshoot d = {} (plan §4b)",
                        st.d_m
                    );
                    assert!(
                        st.rate_mps <= 1e-7 * d_eps / tau,
                        "τ {tau}, T_q/τ {r}: ḋ = {} > 0 at s = {s} (monotone; plan §4b)",
                        st.rate_mps
                    );
                    assert!(
                        st.d_m <= prev + 1e-6 * d_eps,
                        "τ {tau}, T_q/τ {r}: d increased"
                    );
                    prev = st.d_m;
                }
                // Exact boundary values: start state, and (0, 0, 0) at and after T_q.
                let s0 = eval_quintic_d(&q, 0.0);
                assert!(
                    s0.d_m == d_eps
                        && (s0.rate_mps + d_eps / tau).abs() <= 1e-6 * d_eps / tau
                        && (s0.accel_mps2 - d_eps / (tau * tau)).abs()
                            <= 1e-5 * d_eps / (tau * tau),
                    "τ {tau}, T_q/τ {r}: start state {s0:?} (E)"
                );
                let near = eval_quintic_d(&q, q.t_q_s * (1.0 - 1e-4));
                assert!(
                    near.d_m.abs() <= 1e-9
                        && near.rate_mps.abs() <= 1e-6
                        && near.accel_mps2.abs() <= 1e-3,
                    "τ {tau}, T_q/τ {r}: smooth landing {near:?}"
                );
                assert_eq!(
                    eval_quintic_d(&q, q.t_q_s),
                    DState::default(),
                    "(E) rest at T_q"
                );
                let (x, v, a) = eval_quintic(&q, Tick(ticks_ceil(q.t_q_s)));
                assert!(
                    x == q.target && v == 0.0 && a == 0.0,
                    "(E) exactly on target"
                );
                r += 0.25;
            }
        }
    }

    /// Discrete closed-loop poles s = ln(z)/T of the frozen-integrator cascade for one
    /// preset and payload-estimate ratio m̂/m: e = x_ref − x, F = K_v·(K_x·e + ė) with
    /// K_v = m̂·ω_v, K_x = kx_ratio·ω_v (the integrator held), applied one fast period
    /// later (LET) through the LSM current-loop lag τ_cl on a free mass (ZOH).
    fn frozen_poles(p: &SystemParams, m_ratio: f64) -> Vec<lin::C> {
        use fabrail_params::derive;
        let t = derive::fast_period_s(p);
        let tau_cl = derive::tau_current_loop_s(p);
        let m = p.design.carriage.m_rated_kg;
        let wv = 2.0 * PI * p.control.motion.omega_v_hz;
        let kv = m_ratio * m * wv;
        let kx = p.control.motion.kx_ratio * wv;
        // Continuous [e, ė, f] with input u (ZOH): ë = −f/m, ḟ = (u − f)/τ_cl.
        let mut aug = lin::zeros(4);
        aug[0][1] = 1.0;
        aug[1][2] = -1.0 / m;
        aug[2][2] = -1.0 / tau_cl;
        aug[2][3] = 1.0 / tau_cl;
        for row in aug.iter_mut() {
            for v in row.iter_mut() {
                *v *= t;
            }
        }
        let e = lin::expm(&aug);
        // Closed loop [e, ė, f, u_d]: u_d⁺ = K_v·(K_x·e + ė).
        let mut mcl = lin::zeros(4);
        for i in 0..3 {
            for j in 0..4 {
                mcl[i][j] = e[i][j];
            }
        }
        mcl[3][0] = kv * kx;
        mcl[3][1] = kv;
        // δ-operator matrix (M − I)/T: its eigenvalues are well separated near s.
        let mut dmat = mcl.clone();
        for (i, row) in dmat.iter_mut().enumerate() {
            row[i] -= 1.0;
            for v in row.iter_mut() {
                *v /= t;
            }
        }
        lin::roots(&lin::charpoly(&dmat))
            .into_iter()
            .map(|dl| {
                let z = (1.0 + t * dl.0, t * dl.1);
                let r = (z.0 * z.0 + z.1 * z.1).sqrt();
                (r.ln() / t, z.1.atan2(z.0) / t)
            })
            .collect()
    }

    // req: FAB-HAP-015
    #[test]
    fn integrator_freeze_poles() {
        // (1) The freeze request covers exactly blend + capture and is released at d_ε.
        for c in presets::CLASSES {
            let seg = nominal_close(c);
            let s = Segment::Terminus(seg);
            let (_, t_e) = capture(&seg);
            let at = |sec: f32| seg.t0.plus(ticks_round(sec));
            assert!(
                !integrator_frozen(&s, Tick(seg.t0.0 - 1)),
                "{c:?}: cruise integrates"
            );
            assert!(
                integrator_frozen(&s, seg.t0),
                "{c:?}: frozen from blend start"
            );
            assert!(
                integrator_frozen(&s, at(seg.t_b_s + 0.5 * t_e)),
                "{c:?}: frozen in capture"
            );
            assert!(
                !integrator_frozen(&s, at(seg.t_b_s + t_e + 0.5 * seg.t_q_s)),
                "{c:?}: re-enabled at d_ε (quintic; plan §4b)"
            );
            assert!(!integrator_frozen(&s, arrive_tick(&s)), "{c:?}: arrived");
            let q = Segment::Quintic(QuinticSeg::default());
            assert!(!integrator_frozen(&q, Tick(0)), "quintic never freezes");
        }
        // (2) Poles. Continuous: s² + K_v/m̂·s + K_v·K_x/m̂ = (s + ω_v/2)² for K_x = ω_v/4.
        // Discrete (with LET delay and τ_cl): the two error poles keep ζ ≥ 0.9 and
        // their product/sum within 5 % of (ω_v/2)² / ω_v at m̂/m ∈ {0.95, 1, 1.05}
        // (LEV-013 ±5 %); every pole |z| < 1; ω_v/2 ≥ 3/τ (plan §4b).
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            let wv = 2.0 * PI * p.control.motion.omega_v_hz;
            assert!(
                (p.control.motion.kx_ratio - 0.25).abs() < 1e-12,
                "{c:?}: K_x = ω_v/4 gives the double pole (core §C.4)"
            );
            assert!(
                wv / 2.0 >= 3.0 / p.control.motion.terminus.tau_s,
                "{c:?}: ω_v/2 = {} ≥ 3/τ (plan §4b)",
                wv / 2.0
            );
            for mr in [0.95, 1.0, 1.05] {
                let mut poles = frozen_poles(&p, mr);
                for s in &poles {
                    assert!(s.0 < 0.0, "{c:?} m̂/m {mr}: unstable pole {s:?} (|z| < 1)");
                }
                poles.sort_by(|a, b| a.0.hypot(a.1).total_cmp(&b.0.hypot(b.1)));
                let dom = &poles[..2];
                for s in dom {
                    let zeta = -s.0 / s.0.hypot(s.1);
                    assert!(
                        zeta >= 0.9,
                        "{c:?} m̂/m {mr}: error pole {s:?} has ζ = {zeta} < 0.9 (FAB-HAP-015)"
                    );
                    assert!(
                        s.0.hypot(s.1) <= 2.0 * wv,
                        "{c:?}: dominant pair is the error pair"
                    );
                }
                // Product and sum of the pair (real parts; imaginary parts cancel).
                let prod = dom[0].0 * dom[1].0 - dom[0].1 * dom[1].1;
                let sum = -(dom[0].0 + dom[1].0);
                let want = mr * wv * wv / 4.0;
                assert!(
                    (prod / want - 1.0).abs() <= 0.05 && (sum / (mr * wv) - 1.0).abs() <= 0.05,
                    "{c:?} m̂/m {mr}: pair {dom:?} vs (s + ω_v/2)² (±5 %; plan §4b)"
                );
            }
        }
    }

    #[test]
    fn blend_matches_params_derive() {
        // The f32 blend equals params::derive::terminus_blend (f64) to f32 precision, and
        // t_arrive_s equals derive::terminus_t_arrive, for the plan §4b table rows
        // (rel 1e-5; PHYSICS §16.1).
        use fabrail_params::derive;
        let rows = [
            (0.2654, 20.0, 0.15),
            (0.152, 5.0, 0.2),
            (0.152, 5.0, 0.25),
            (0.1838, 5.0, 0.25),
            (0.0446, 1.5, 0.6),
            (0.025, 1.0, 1.0),
            (1.0e-5, 20.0, 0.15),
        ];
        for (v, j, tau) in rows {
            let b64 = derive::terminus_blend(v, j, tau);
            let b = blend(v as f32, j as f32, tau as f32);
            let rel = |x: f32, y: f64| (f64::from(x) - y).abs() / y.abs().max(1e-30);
            assert!(
                rel(b.t_b_s, b64.t_b_s) < 1e-5
                    && rel(b.d_b_m, b64.d_b_m) < 1e-5
                    && rel(b.d_0_m, b64.d_0_m) < 1e-5,
                "blend({v}, {j}, {tau}) = {b:?} vs {b64:?} (rel 1e-5)"
            );
            let seg = TerminusSeg {
                v_app_mps: v as f32,
                j_mps3: j as f32,
                tau_s: tau as f32,
                t_b_s: b.t_b_s,
                d_b_m: b.d_b_m,
                d_0_m: b.d_0_m,
                d_eps_m: 2e-4,
                t_q_s: tau as f32,
                ..TerminusSeg::default()
            };
            let ta = derive::terminus_t_arrive(v, j, tau, 2e-4);
            assert!(
                rel(t_arrive_s(&seg), ta) < 1e-5,
                "t_arrive({v}, {j}, {tau}) = {} vs {ta} (rel 1e-5)",
                t_arrive_s(&seg)
            );
        }
        assert_eq!(blend(0.1, 0.0, 0.15), Blend::default());
        assert_eq!(blend(0.1, 20.0, f32::NAN), Blend::default());
    }

    #[test]
    fn malformed_segments_hold() {
        // Non-finite or degenerate segments never produce NaN (they hold at rest).
        let mut seg = nominal_close(RailClass::Drawer);
        seg.tau_s = f32::NAN;
        let (x, v, a) = eval(&seg, Tick(5_000));
        assert!(v == 0.0 && a == 0.0 && x.diff_m(seg.target).is_finite());
        assert_eq!(t_arrive_s(&seg), 0.0);
        assert_eq!(phase(&seg, Tick(0)), TerminusPhase::Arrived);
        let q = QuinticSeg {
            t_q_s: 0.0,
            d0_m: 0.01,
            ..QuinticSeg::default()
        };
        let (_, v, a) = eval_quintic(&q, Tick(3));
        assert!(v == 0.0 && a == 0.0);
        assert!(done(&Segment::Quintic(q), Tick(0)));
        let st = eval_d(&nominal_close(RailClass::Door), f32::NAN);
        assert!(st.d_m.is_finite() && st.rate_mps == 0.0);
    }

    #[test]
    fn lin_helpers_self_check() {
        // Roots of (x − 0.5)(x + 2)(x² + 1) = x⁴ + 1.5x³ + 1.5x − 1: the eigen helpers
        // the pole tests rely on.
        let c = [-1.0, 1.5, 0.0, 1.5, 1.0];
        let r = lin::roots(&c);
        let want = [(-2.0, 0.0), (0.0, -1.0), (0.0, 1.0), (0.5, 0.0)];
        for w in want {
            assert!(
                r.iter()
                    .any(|g| (g.0 - w.0).abs() < 1e-9 && (g.1 - w.1).abs() < 1e-9),
                "root {w:?} not found in {r:?}"
            );
        }
        // charpoly of diag(1, 2, 3) is (x−1)(x−2)(x−3) = x³ − 6x² + 11x − 6.
        let mut m = lin::zeros(3);
        m[0][0] = 1.0;
        m[1][1] = 2.0;
        m[2][2] = 3.0;
        let cp = lin::charpoly(&m);
        assert!(
            (cp[0] + 6.0).abs() < 1e-12
                && (cp[1] - 11.0).abs() < 1e-12
                && (cp[2] + 6.0).abs() < 1e-12
        );
        // expm of a rotation generator and the spectral radius of a scaled rotation.
        let mut g = lin::zeros(2);
        g[0][1] = -1.0;
        g[1][0] = 1.0;
        let e = lin::expm(&g);
        assert!((e[0][0] - 1f64.cos()).abs() < 1e-12 && (e[1][0] - 1f64.sin()).abs() < 1e-12);
        let mut rot = e.clone();
        for row in rot.iter_mut() {
            for v in row.iter_mut() {
                *v *= 0.9;
            }
        }
        let rho = lin::spectral_radius(&rot, 26);
        assert!((rho - 0.9).abs() < 1e-6, "ρ = {rho}");
    }
}
