//! HousePlan: deadline-constrained parallel scheduling under the bus power budget, P(t_S) 5 % quantile minus 1 s deadlines, QUIET seating. Owner: E2 (API frozen by W0a).
//!
//! Plan §4b. At alert time the choreographer computes one deterministic plan: every
//! device gets a policy, a target rank, a start tick and a deadline (the S-wave arrival
//! quantile `deadline_quantile` (5 %) minus `deadline_margin_s` (1 s)), staggered in
//! parallel under the bus power budget (serial would take about 17 s for six drawers).
//! Devices that cannot finish in time keep the core class action (freeze or seat) or
//! are reported `NotSecured` with a reason. The plan replays bit-identically from the
//! recorded `FusedAlert` and device status.
//!
//! **Deadline** (PHYSICS §20): t_S ~ N(μ, σ) from the building prediction (or the fused
//! alert when the prediction carries no S-ETA); the deadline is the q-quantile minus the
//! margin, μ + σ·Φ⁻¹(q) − margin ([`normal_quantile`], [`plan_deadline`]). For the
//! warned villa variant (t_S = 17.5 s) this is 14.86 s at σ = 1 s and 13.21 s at
//! σ = 2 s (plan §4b: 13.2–14.8 s).
//!
//! **Durations** (PHYSICS §16, §16.1): a close is a jerk-limited ramp to the unattended
//! speed, a cruise and the log-damped terminus (`params::derive::terminus_blend` and
//! `terminus_t_arrive`), plus the DRAWER z-seat step ([`motion_time_s`]); a stop is
//! v/a_brk + a_brk/j, then the landing descent (g_land − g*)/v_land.
//!
//! **Schedule.** Motions draw `p_move_w` from the bus while they run; protective actions
//! without x motion (land, lock, ride-through) start at once. Motions are placed longest
//! first (ties by registry index) at the earliest start where the summed power of the
//! motions already placed plus their own stays within `p_bus_max_w` over their whole
//! run (a list schedule: parallel wherever the budget allows). A motion that would
//! finish after the deadline is `NotSecured` — `Deadline` when it cannot finish even
//! alone from now, else `Power` — and the core class action stands for it.
//!
//! **Alert-motion rule.** Every decision is put to `fabrail_exo_safety::alert_gate`
//! ([`crate::choreo::policy::gate_decision`]); a refused motion becomes the core class
//! action (`NotSecured`/`Gate`). A planned motion waiting for its power slot proposes
//! FREEZE (hold in place, never an SS1 at the standoff). One attempt only: an
//! obstruction marks the device `NotSecured`/`Obstruction` and it then proposes FREEZE
//! (no re-close); [`replan`] keeps committed actions.
//!
//! **QUIET** ([`quiet_action`]): devices left at a standoff seat at unattended limits
//! after a gap/encoder plausibility check, otherwise they land and are flagged for a
//! manual push-to-seat.

use fabrail_core::seismic::action_policy::ActionDecision;
use fabrail_exo_safety::alert_gate::{DenyReason, GateVerdict};
use fabrail_exo_safety::{
    AnyClass, DeviceCondition, FusedAlert, SafetyRank, presets as exo_presets,
};
use fabrail_params::{SystemParams, derive, presets};
use fabrail_types::{
    BASE_TICK_HZ, EgressRole, PosNm, RailClass, RestPolicy, SeismicAction, Tick, fmath,
};

use crate::choreo::policy::{
    ChoreoPolicy, SmoothAdmissionInputs, admit_smooth_ride_through, gate_decision, latch_holds,
    resolve_auto,
};
use crate::choreo::registry::{DeviceEntry, DeviceRegistry, MAX_DEVICES};

/// Default deadline quantile of the S arrival (plan §4b: 5 %).
pub const DEFAULT_DEADLINE_QUANTILE: f32 = 0.05;

/// Default deadline margin, s (plan §4b: 1 s).
pub const DEFAULT_DEADLINE_MARGIN_S: f32 = 1.0;

/// The building-model prediction the plan is computed against.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BuildingPrediction {
    /// Planning tick (alert time).
    pub now: Tick,
    /// Mean S-wave arrival at the building.
    pub s_eta: Option<Tick>,
    /// S-wave arrival standard deviation, s.
    pub s_eta_sigma_s: f32,
    /// Deadline quantile of the S arrival (0.05).
    pub deadline_quantile: f32,
    /// Deadline margin, s (1 s).
    pub deadline_margin_s: f32,
    /// Bus power budget, W (≤ 0 or non-finite: no budget is enforced).
    pub p_bus_max_w: f32,
    /// Predicted peak floor acceleration (x in the device frame) per device, m/s².
    pub pfa_x_mps2: [f32; MAX_DEVICES],
    /// Predicted peak floor acceleration (y in the device frame) per device, m/s².
    pub pfa_y_mps2: [f32; MAX_DEVICES],
}

impl Default for BuildingPrediction {
    fn default() -> BuildingPrediction {
        BuildingPrediction {
            now: Tick(0),
            s_eta: None,
            s_eta_sigma_s: 0.0,
            deadline_quantile: DEFAULT_DEADLINE_QUANTILE,
            deadline_margin_s: DEFAULT_DEADLINE_MARGIN_S,
            p_bus_max_w: 0.0,
            pfa_x_mps2: [0.0; MAX_DEVICES],
            pfa_y_mps2: [0.0; MAX_DEVICES],
        }
    }
}

/// Planning status of one device.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[repr(u8)]
pub enum PlanStatus {
    /// No override: the core class action stands.
    #[default]
    CoreFallback = 0,
    /// A choreographed action was planned.
    Planned = 1,
    /// The device cannot reach its rank in time (reason given).
    NotSecured = 2,
}

impl PlanStatus {
    /// Snake-case name.
    pub const fn slug(self) -> &'static str {
        match self {
            PlanStatus::CoreFallback => "core_fallback",
            PlanStatus::Planned => "planned",
            PlanStatus::NotSecured => "not_secured",
        }
    }
}

/// Why a device is not secured.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[repr(u8)]
pub enum NotSecuredReason {
    /// Cannot finish before the deadline.
    Deadline = 0,
    /// The bus power budget is exhausted.
    Power = 1,
    /// An admission check failed.
    Admission = 2,
    /// Obstruction during the single attempt.
    Obstruction = 3,
    /// The alert gate refused the motion.
    Gate = 4,
}

impl NotSecuredReason {
    /// Snake-case name.
    pub const fn slug(self) -> &'static str {
        match self {
            NotSecuredReason::Deadline => "deadline",
            NotSecuredReason::Power => "power",
            NotSecuredReason::Admission => "admission",
            NotSecuredReason::Obstruction => "obstruction",
            NotSecuredReason::Gate => "gate",
        }
    }
}

/// One device's planned action.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlannedAction {
    /// Registry index.
    pub device: u8,
    /// Resolved policy.
    pub policy: ChoreoPolicy,
    /// Decision proposed through the core `ActionPolicy` hook (`SeismicAction::None`:
    /// no override, the core class action stands).
    pub decision: ActionDecision,
    /// Start tick.
    pub start: Tick,
    /// Target rank.
    pub target_rank: SafetyRank,
    /// Status.
    pub status: PlanStatus,
    /// Reason when `NotSecured`.
    pub reason: Option<NotSecuredReason>,
    /// Bus power while executing, W.
    pub p_w: f32,
    /// Predicted completion tick (start + duration; for a refused motion, when it would
    /// have finished alone from now). Additive to the W0 surface (E2).
    pub finish: Tick,
    /// A closing motion refused only for want of independent confirmation holds
    /// (FREEZE) until this tick — its latest start (deadline − motion time) — so a
    /// confirmation arriving in time (a local P pick) can still start it at the next
    /// re-plan; afterwards the core class action stands. `None` otherwise. Additive
    /// (Wave 2b, E5).
    pub hold_until: Option<Tick>,
}

impl PlannedAction {
    /// True when the decision moves the carriage along x.
    pub fn moves(&self) -> bool {
        self.decision.moves()
    }

    /// What the device's `PlanPolicy` proposes at tick `now` (`None`: the core class
    /// action stands): nothing before a non-moving action's start or for a core
    /// fallback; FREEZE while a planned motion waits for its power slot; the decision
    /// from its start on.
    pub fn proposal(&self, now: Tick) -> Option<ActionDecision> {
        if self.decision.action == SeismicAction::None {
            return None;
        }
        if let Some(h) = self.hold_until {
            // Awaiting confirmation: FREEZE until the latest start, then the core action.
            return (now < h).then_some(self.decision);
        }
        if now < self.start {
            if self.status == PlanStatus::Planned && self.moves() {
                return Some(ActionDecision::of(SeismicAction::Freeze));
            }
            return None;
        }
        Some(self.decision)
    }
}

/// The house plan (fixed capacity, registry order).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HousePlan {
    /// Planned actions; entries beyond `len` are unused.
    pub actions: [PlannedAction; MAX_DEVICES],
    /// Number of actions (= devices).
    pub len: u8,
    /// Common deadline, if an S arrival was predicted.
    pub deadline: Option<Tick>,
    /// Peak planned bus power, W.
    pub p_peak_w: f32,
    /// Alert the plan answers.
    pub alert_id: u32,
}

impl Default for HousePlan {
    fn default() -> HousePlan {
        HousePlan {
            actions: [PlannedAction::default(); MAX_DEVICES],
            len: 0,
            deadline: None,
            p_peak_w: 0.0,
            alert_id: 0,
        }
    }
}

impl HousePlan {
    /// The planned actions in registry order.
    pub fn as_slice(&self) -> &[PlannedAction] {
        self.actions.get(..self.len as usize).unwrap_or(&[])
    }

    /// Action of device `i`.
    pub fn get(&self, i: usize) -> Option<&PlannedAction> {
        self.as_slice().get(i)
    }

    /// What device `i` proposes at `now` ([`PlannedAction::proposal`]).
    pub fn proposal(&self, i: usize, now: Tick) -> Option<ActionDecision> {
        self.get(i).and_then(|a| a.proposal(now))
    }

    /// Marks device `i` not secured (e.g. an obstruction during its single attempt).
    /// An obstructed or refused motion then proposes FREEZE (hold, clamps unchanged; no
    /// re-close); a protective action without x motion keeps its decision.
    pub fn mark_not_secured(&mut self, i: usize, reason: NotSecuredReason) {
        let n = self.len as usize;
        if let Some(a) = self.actions.get_mut(..n).and_then(|s| s.get_mut(i)) {
            a.status = PlanStatus::NotSecured;
            a.reason = Some(reason);
            if a.decision.moves() {
                a.decision = ActionDecision::of(SeismicAction::Freeze);
                a.p_w = 0.0;
                a.hold_until = None;
            }
        }
    }
}

/// Kinematics the planner times one device's action with (PHYSICS §15, §16, §16.1).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DeviceKinematics {
    /// Acceleration limit a_max, m/s².
    pub a_max_mps2: f32,
    /// Jerk limit j_max, m/s³.
    pub j_max_mps3: f32,
    /// Terminus time constant τ, s.
    pub tau_s: f32,
    /// Terminus blend jerk, m/s³.
    pub terminus_j_mps3: f32,
    /// Terminus capture distance d_ε, m.
    pub d_eps_m: f32,
    /// DRAWER z-seat step after the terminus (0 for other classes), s.
    pub t_seat_s: f32,
    /// Landing descent time (g_land − g*)/v_land, s.
    pub t_land_s: f32,
    /// Closed-position tolerance, m.
    pub pos_tol_m: f32,
}

/// Landing descent time assumed for sovereign classes without a core parameter set, s
/// (\[UNCERTAIN\] design choice: HEAVY-like 2 mm/s over ≈ 4 mm; PHYSICS §15).
pub const EXO_T_LAND_S: f32 = 2.0;

/// Terminus τ assumed for sovereign classes, s (the HEAVY value of the PHYSICS §16.1
/// table, the slowest core class).
pub const EXO_TAU_S: f32 = 1.0;

impl DeviceKinematics {
    /// From a core parameter set (`control.motion`, `control.motion.terminus`,
    /// `control.levitation`, `design.suspension`).
    pub fn from_params(p: &SystemParams) -> DeviceKinematics {
        let mo = &p.control.motion;
        let lv = &p.control.levitation;
        let s = &p.design.suspension;
        let rest = if s.g_seat_m > 0.0 && matches!(p.design.rest_policy, RestPolicy::Seat) {
            s.g_seat_m
        } else {
            s.g_land_m
        };
        // Landing descent from the zero-power gap to the rest gap at the landing speed
        // (PHYSICS §15; LEV-012 landing ramp).
        let t_land = if lv.land_speed_mps > 0.0 {
            fmath::abs(rest - s.g_star_m) / lv.land_speed_mps
        } else {
            0.0
        };
        DeviceKinematics {
            a_max_mps2: mo.a_max_mps2 as f32,
            j_max_mps3: mo.j_max_mps3 as f32,
            tau_s: mo.terminus.tau_s as f32,
            terminus_j_mps3: mo.terminus.j_mps3 as f32,
            d_eps_m: mo.terminus.d_eps_m as f32,
            t_seat_s: if matches!(p.design.rest_policy, RestPolicy::Seat) {
                lv.seat_time_s as f32
            } else {
                0.0
            },
            t_land_s: t_land as f32,
            pos_tol_m: mo.pos_tol_closed_m as f32,
        }
    }

    /// The class preset's kinematics (core presets; sovereign presets from their
    /// `Kinematics` row with τ = [`EXO_TAU_S`] and landing [`EXO_T_LAND_S`]).
    pub fn for_class(c: AnyClass) -> DeviceKinematics {
        match c {
            AnyClass::Core(rc) => DeviceKinematics::from_params(&presets::by_class(rc)),
            AnyClass::Exo(x) => {
                let k = exo_presets::preset(x).kinematics;
                let a = if k.a_max_mmps2 > 0 {
                    k.a_max_mmps2 as f32 * 1.0e-3
                } else {
                    0.1
                };
                let j = if k.j_max_mmps3 > 0 {
                    k.j_max_mmps3 as f32 * 1.0e-3
                } else {
                    1.0
                };
                DeviceKinematics {
                    a_max_mps2: a,
                    j_max_mps3: j,
                    tau_s: EXO_TAU_S,
                    terminus_j_mps3: j,
                    d_eps_m: 2.0e-4,
                    t_seat_s: 0.0,
                    t_land_s: EXO_T_LAND_S,
                    pos_tol_m: 1.0e-3,
                }
            }
        }
    }
}

/// Planning options beyond the frozen [`plan`] inputs (additive, E2).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanOptions {
    /// Per-device kinematics (`None`: the class preset, [`DeviceKinematics::for_class`]).
    pub kinematics: [Option<DeviceKinematics>; MAX_DEVICES],
    /// SmoothRideThrough allowed (research tier; false by default).
    pub smoothing: bool,
    /// Per-device SmoothRideThrough admission inputs (`None`: not admissible).
    pub smooth: [Option<SmoothAdmissionInputs>; MAX_DEVICES],
    /// Per-device zone `ClearVerified` (sovereign classes; the HAVEN plinth pre-centres
    /// only with it).
    pub zone_clear_verified: [bool; MAX_DEVICES],
    /// Unattended automatic closing allowed (scene `choreo.unattended_close`).
    pub unattended_close: bool,
}

impl Default for PlanOptions {
    fn default() -> PlanOptions {
        PlanOptions {
            kinematics: [None; MAX_DEVICES],
            smoothing: false,
            smooth: [None; MAX_DEVICES],
            zone_clear_verified: [false; MAX_DEVICES],
            unattended_close: true,
        }
    }
}

// ---- Deadline math (PHYSICS §20) -------------------------------------------------------

/// Φ⁻¹(p), the standard normal quantile: Acklam's rational approximation (P. J. Acklam,
/// "An algorithm for computing the inverse normal cumulative distribution function",
/// 2003; relative error ≤ 1.15e-9) refined by one Halley step on Φ(x) = ½·erfc(−x/√2)
/// (PHYSICS §20). ±∞ at 0 and 1; NaN outside [0, 1].
pub fn normal_quantile(p: f64) -> f64 {
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    const P_LOW: f64 = 0.024_25;
    if !(0.0..=1.0).contains(&p) {
        return f64::NAN;
    }
    if p == 0.0 {
        return f64::NEG_INFINITY;
    }
    if p == 1.0 {
        return f64::INFINITY;
    }
    let tail = |q: f64| {
        (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    };
    let x = if p < P_LOW {
        tail(fmath::sqrt(-2.0 * fmath::ln(p)))
    } else if p <= 1.0 - P_LOW {
        let q = p - 0.5;
        let r = q * q;
        (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
            / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
    } else {
        -tail(fmath::sqrt(-2.0 * fmath::ln(1.0 - p)))
    };
    // One Halley step (Acklam 2003, "refinement"): e = Φ(x) − p.
    let e = 0.5 * fmath::erfc(-x / core::f64::consts::SQRT_2) - p;
    let u = e * fmath::sqrt(2.0 * core::f64::consts::PI) * fmath::exp(0.5 * x * x);
    let refined = x - u / (1.0 + 0.5 * x * u);
    if refined.is_finite() { refined } else { x }
}

/// Deadline for an S arrival N(`s_eta`, `sigma_s`): the `quantile` minus `margin_s`,
/// μ + σ·Φ⁻¹(q) − margin, rounded to the nearest base tick (saturating at tick 0; plan
/// §4b, PHYSICS §20). A quantile outside (0, 0.5] or non-finite inputs fall back to the
/// defaults (5 %, 1 s); a negative σ counts as 0.
pub fn s_quantile_deadline(s_eta: Tick, sigma_s: f32, quantile: f32, margin_s: f32) -> Tick {
    let q = if quantile.is_finite() && quantile > 0.0 && quantile <= 0.5 {
        quantile
    } else {
        DEFAULT_DEADLINE_QUANTILE
    };
    let m = if margin_s.is_finite() && margin_s >= 0.0 {
        margin_s
    } else {
        DEFAULT_DEADLINE_MARGIN_S
    };
    let sigma = if sigma_s.is_finite() && sigma_s > 0.0 {
        f64::from(sigma_s)
    } else {
        0.0
    };
    let t = s_eta.seconds_f64() + sigma * normal_quantile(f64::from(q)) - f64::from(m);
    Tick::from_seconds_f64(t)
}

/// The plan deadline: the S-ETA distribution of the building prediction when it has
/// one, else the fused alert's (μ, σ); `None` when neither predicts an S arrival.
pub fn plan_deadline(alert: &FusedAlert, pred: &BuildingPrediction) -> Option<Tick> {
    let (mu, sigma) = match pred.s_eta {
        Some(t) => (t, pred.s_eta_sigma_s),
        None => (alert.s_eta?, alert.s_eta_sigma_s),
    };
    Some(s_quantile_deadline(
        mu,
        sigma,
        pred.deadline_quantile,
        pred.deadline_margin_s,
    ))
}

// ---- Durations (PHYSICS §16, §16.1) -------------------------------------------------------

/// Jerk-limited ramp 0 → v at (a, j): (time, distance). t = v/a + a/j when v ≥ a²/j
/// (trapezoidal acceleration), else 2·√(v/j) (triangular); the ramp is point-symmetric
/// about v/2, so d = v·t/2 (PHYSICS §16).
fn ramp(v: f64, a: f64, j: f64) -> (f64, f64) {
    let t = if v >= a * a / j {
        v / a + a / j
    } else {
        2.0 * fmath::sqrt(v / j)
    };
    (t, 0.5 * v * t)
}

/// Time to travel `d_m` toward a stop from rest: a jerk-limited ramp to at most `v_mps`,
/// a cruise and the log-damped terminus (blend, exponential capture, quintic;
/// `derive::terminus_t_arrive` with T_q = τ). When `d_m` is too short to reach `v_mps`
/// the cruise speed is lowered (bisection) until ramp + terminus capture fit exactly
/// (PHYSICS §16, §16.1). The DRAWER seat step is not included. +∞ for non-positive
/// kinematics; 0 for d ≤ 0.
pub fn motion_time_s(d_m: f32, v_mps: f32, k: &DeviceKinematics) -> f32 {
    let (d, v) = (f64::from(d_m), f64::from(v_mps));
    let (a, j) = (f64::from(k.a_max_mps2), f64::from(k.j_max_mps3));
    let (tau, jt, eps) = (
        f64::from(k.tau_s),
        f64::from(k.terminus_j_mps3),
        f64::from(k.d_eps_m),
    );
    if !(d.is_finite() && v.is_finite()) || d <= 0.0 {
        return 0.0;
    }
    if !(v > 0.0 && a > 0.0 && j > 0.0 && tau > 0.0 && jt > 0.0) {
        return f32::INFINITY;
    }
    let need = |vc: f64| ramp(vc, a, j).1 + derive::terminus_blend(vc, jt, tau).d_0_m;
    let total = |vc: f64, cruise_m: f64| {
        ramp(vc, a, j).0 + cruise_m / vc + derive::terminus_t_arrive(vc, jt, tau, eps)
    };
    let t = if d >= need(v) {
        total(v, d - need(v))
    } else {
        // need(v) is increasing in v: bisect on (0, v].
        let (mut lo, mut hi) = (0.0f64, v);
        for _ in 0..48 {
            let mid = 0.5 * (lo + hi);
            if need(mid) > d {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        if lo > 0.0 { total(lo, 0.0) } else { tau }
    };
    t as f32
}

/// Time to stop from `v_mps` at deceleration `a_brk` with jerk `j`: v/a + a/j
/// (PHYSICS §16), 0 at rest.
pub fn stop_time_s(v_mps: f32, a_brk_mps2: f32, j_mps3: f32) -> f32 {
    let v = fmath::absf(v_mps);
    if !(v.is_finite() && v > 0.0) {
        return 0.0;
    }
    if !(a_brk_mps2 > 0.0 && j_mps3 > 0.0) {
        return f32::INFINITY;
    }
    v / a_brk_mps2 + a_brk_mps2 / j_mps3
}

fn secs_to_ticks(t_s: f32) -> u64 {
    if !(t_s.is_finite() && t_s > 0.0) {
        return if t_s.is_finite() { 0 } else { u64::MAX };
    }
    fmath::round(f64::from(t_s) * f64::from(BASE_TICK_HZ)) as u64
}

// ---- Per-device evaluation --------------------------------------------------------------

/// The resolved action of one device before scheduling.
#[derive(Clone, Copy, Debug)]
struct Eval {
    policy: ChoreoPolicy,
    target_rank: SafetyRank,
    decision: ActionDecision,
    dur_s: f32,
    p_w: f32,
    reason: Option<NotSecuredReason>,
    /// The closing motion was refused only for want of confirmation: hold (FREEZE)
    /// until its latest start (`PlannedAction::hold_until`).
    await_confirmation: bool,
}

/// Configured policy → effective policy (class/egress admissibility, latch capacity,
/// research-tier smoothing admission).
fn resolve_policy(e: &DeviceEntry, pfa_x: f32, i: usize, opts: &PlanOptions) -> ChoreoPolicy {
    let mut p = if e.policy == ChoreoPolicy::Auto || !e.policy.allowed(e.class, e.egress) {
        resolve_auto(e)
    } else {
        e.policy
    };
    let drawer = matches!(e.class, AnyClass::Core(RailClass::Drawer));
    let smooth_ok = opts.smoothing
        && opts
            .smooth
            .get(i)
            .copied()
            .flatten()
            .is_some_and(|s| admit_smooth_ride_through(&s));
    // Research tier: an Auto drawer may ride through only when admitted.
    if drawer && e.policy == ChoreoPolicy::Auto && smooth_ok {
        p = ChoreoPolicy::SmoothRideThrough;
    }
    if p == ChoreoPolicy::SmoothRideThrough && !smooth_ok {
        p = ChoreoPolicy::SeatAndLatch;
    }
    if p == ChoreoPolicy::SeatAndLatch && !latch_holds(e, pfa_x) {
        p = ChoreoPolicy::SeatAndHold;
    }
    p
}

fn at_closed(e: &DeviceEntry, k: &DeviceKinematics) -> bool {
    e.x.diff_m(e.x_closed) <= k.pos_tol_m
}

fn eval_device(
    i: usize,
    e: &DeviceEntry,
    alert: &FusedAlert,
    pred: &BuildingPrediction,
    opts: &PlanOptions,
    deadline: Option<Tick>,
) -> Eval {
    let k = opts
        .kinematics
        .get(i)
        .copied()
        .flatten()
        .unwrap_or_else(|| DeviceKinematics::for_class(e.class));
    let pfa_x = pred.pfa_x_mps2.get(i).copied().unwrap_or(0.0);
    let zone = opts.zone_clear_verified.get(i).copied().unwrap_or(false);
    let policy = resolve_policy(e, pfa_x, i, opts);
    let rank_now = SafetyRank::of(e.condition, e.egress);
    let c = &e.constraints;
    let v_move = if c.v_cap_mps > 0.0 {
        c.v_unattended_mps.min(c.v_cap_mps)
    } else {
        c.v_unattended_mps
    };
    let open_m = e.x.diff_m(e.x_closed);
    let stop_land = stop_time_s(e.v_mps, c.a_brk_mps2, k.j_max_mps3) + k.t_land_s;
    let hold = |d: ActionDecision, rank: SafetyRank, dur: f32| Eval {
        policy,
        target_rank: rank,
        decision: d,
        dur_s: dur,
        p_w: 0.0,
        reason: None,
        await_confirmation: false,
    };
    let go = |action: SeismicAction, target: PosNm, dist_m: f32, extra_s: f32, rank| Eval {
        policy,
        target_rank: rank,
        decision: ActionDecision {
            action,
            target: Some(target),
            v_cap_mps: v_move,
            deadline,
        },
        dur_s: motion_time_s(dist_m, v_move, &k) + extra_s,
        p_w: c.p_move_w.max(0.0),
        reason: None,
        await_confirmation: false,
    };
    let mut ev = match policy {
        ChoreoPolicy::SeatAndHold | ChoreoPolicy::SeatAndLatch | ChoreoPolicy::Auto => {
            let act = if policy == ChoreoPolicy::SeatAndLatch {
                SeismicAction::Lock
            } else {
                SeismicAction::Land
            };
            match e.condition {
                DeviceCondition::Seated | DeviceCondition::LandedLocked => {
                    hold(ActionDecision::of(act), SafetyRank::SeatedOrLocked, 0.0)
                }
                // On its rollers away from the seat: no automatic re-lift (F17; plan §4b
                // H-20); it stays landed and needs a manual push-to-seat.
                DeviceCondition::Landed if !at_closed(e, &k) => Eval {
                    reason: Some(NotSecuredReason::Admission),
                    ..hold(ActionDecision::of(SeismicAction::Land), rank_now, 0.0)
                },
                _ if at_closed(e, &k) => hold(
                    ActionDecision::of(act),
                    SafetyRank::SeatedOrLocked,
                    stop_time_s(e.v_mps, c.a_brk_mps2, k.j_max_mps3) + k.t_seat_s,
                ),
                _ => go(
                    act,
                    e.x_closed,
                    open_m,
                    k.t_seat_s,
                    SafetyRank::SeatedOrLocked,
                ),
            }
        }
        ChoreoPolicy::SmoothRideThrough => {
            let s_min = c.s_min_m.max(0.0);
            let target = e.x_closed.offset_m(s_min);
            if open_m > s_min + k.pos_tol_m {
                go(
                    SeismicAction::RideThrough,
                    target,
                    open_m - s_min,
                    0.0,
                    SafetyRank::StandoffHold,
                )
            } else {
                hold(
                    ActionDecision::of(SeismicAction::RideThrough),
                    SafetyRank::StandoffHold,
                    0.0,
                )
            }
        }
        ChoreoPolicy::StopAndLand => hold(
            ActionDecision::of(SeismicAction::Land),
            SafetyRank::of(DeviceCondition::Landed, e.egress),
            stop_land,
        ),
        ChoreoPolicy::LandAndLock => hold(
            ActionDecision::of(SeismicAction::Lock),
            SafetyRank::of(DeviceCondition::LandedLocked, e.egress),
            stop_land,
        ),
        ChoreoPolicy::RideThrough => {
            let isolator = matches!(
                e.class,
                AnyClass::Exo(
                    fabrail_exo_safety::ExoClass::HavenR | fabrail_exo_safety::ExoClass::HavenF
                )
            );
            if isolator && zone && !at_closed(e, &k) {
                // Pre-centring (plan §4b: only while the moat zone is ClearVerified).
                go(
                    SeismicAction::Center,
                    e.x_closed,
                    open_m,
                    0.0,
                    rank_now.max(SafetyRank::StandoffHold),
                )
            } else {
                hold(
                    ActionDecision::of(SeismicAction::RideThrough),
                    rank_now,
                    0.0,
                )
            }
        }
        ChoreoPolicy::CompleteClose => {
            if at_closed(e, &k) {
                hold(
                    ActionDecision::of(SeismicAction::Land),
                    SafetyRank::of(DeviceCondition::Landed, e.egress),
                    stop_land,
                )
            } else {
                go(
                    SeismicAction::CompleteClose,
                    e.x_closed,
                    open_m,
                    k.t_land_s,
                    SafetyRank::SeatedOrLocked,
                )
            }
        }
        // Closes only after the shaking, on the external command (TIDEGATE): nothing is
        // scheduled now; the core carries the action. Already closed: land.
        ChoreoPolicy::CloseAfterShaking => {
            if at_closed(e, &k) {
                hold(
                    ActionDecision::of(SeismicAction::Land),
                    SafetyRank::of(DeviceCondition::Landed, e.egress),
                    stop_land,
                )
            } else {
                hold(
                    ActionDecision::of(SeismicAction::CloseAfterShaking),
                    rank_now,
                    0.0,
                )
            }
        }
    };
    // Admission of automatic motion: unattended closing enabled and a usable limit.
    if ev.decision.moves()
        && ev.decision.target.is_some_and(|t| t != e.x)
        && !(opts.unattended_close && v_move.is_finite() && v_move > 0.0)
    {
        ev = fallback(ev, rank_now, NotSecuredReason::Admission);
    }
    // The alert gate is consulted for every command (plan §4b).
    let verdict = gate_decision(alert, e, &ev.decision, ev.target_rank, zone);
    if !verdict.is_allowed() {
        let moving = ev.decision.target.is_some_and(|t| t != e.x)
            || matches!(
                ev.decision.action,
                SeismicAction::CompleteClose | SeismicAction::CloseAfterShaking
            );
        if moving && verdict == GateVerdict::Denied(DenyReason::Unconfirmed) {
            // Only the confirmation is missing (a network alert before the local P pick):
            // hold in place (FREEZE, which the gate always allows) until the latest start,
            // so the re-plan on confirmation can still close it; never an SS1 landing at
            // the open position meanwhile (H-20). The core action follows the hold.
            ev = Eval {
                decision: ActionDecision::of(SeismicAction::Freeze),
                target_rank: rank_now,
                p_w: 0.0,
                reason: Some(NotSecuredReason::Gate),
                await_confirmation: true,
                ..ev
            };
        } else if moving {
            ev = fallback(ev, rank_now, NotSecuredReason::Gate);
        } else {
            // A refused protective action (e.g. LOCK on an egress path) degrades to LAND,
            // then FREEZE, which the gate always allows.
            let land = ActionDecision::of(SeismicAction::Land);
            let rank = SafetyRank::of(DeviceCondition::Landed, e.egress);
            ev.decision = if gate_decision(alert, e, &land, rank, zone).is_allowed() {
                ev.target_rank = rank;
                land
            } else {
                ev.target_rank = rank_now;
                ActionDecision::of(SeismicAction::Freeze)
            };
            ev.reason = Some(NotSecuredReason::Gate);
        }
    }
    ev
}

/// A refused motion: the core class action stands (freeze or seat).
fn fallback(ev: Eval, rank_now: SafetyRank, why: NotSecuredReason) -> Eval {
    Eval {
        decision: ActionDecision::of(SeismicAction::None),
        target_rank: rank_now,
        p_w: 0.0,
        reason: Some(why),
        await_confirmation: false,
        ..ev
    }
}

// ---- Schedule ----------------------------------------------------------------------------

/// A placed power interval [start, end) at `p_w`, ticks.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Slot {
    start: u64,
    end: u64,
    p_w: f32,
}

fn load_at(slots: &[Slot], t: u64) -> f32 {
    slots
        .iter()
        .filter(|s| s.start <= t && t < s.end)
        .fold(0.0f32, |acc, s| acc + s.p_w)
}

/// Peak summed power over [a, b).
fn peak_over(slots: &[Slot], a: u64, b: u64) -> f32 {
    let mut peak = load_at(slots, a);
    for s in slots {
        if s.start > a && s.start < b {
            peak = peak.max(load_at(slots, s.start));
        }
    }
    peak
}

/// Earliest start ≥ `now` where `p_w` fits over `dur` ticks under `budget`; `None` when
/// it never fits (p alone exceeds the budget).
fn earliest_fit(slots: &[Slot], now: u64, dur: u64, p_w: f32, budget: Option<f32>) -> Option<u64> {
    let Some(b) = budget else {
        return Some(now);
    };
    if p_w > b {
        return None;
    }
    let fits = |s: u64| peak_over(slots, s, s.saturating_add(dur.max(1))) + p_w <= b;
    if fits(now) {
        return Some(now);
    }
    // Candidate starts are the ends of placed intervals (the load only drops there).
    let mut best: Option<u64> = None;
    for s in slots {
        let c = s.end.max(now);
        if fits(c) && best.is_none_or(|b0| c < b0) {
            best = Some(c);
        }
    }
    best
}

/// Plans every device's response to `alert` with default [`PlanOptions`].
pub fn plan(alert: &FusedAlert, reg: &DeviceRegistry, pred: &BuildingPrediction) -> HousePlan {
    plan_with(alert, reg, pred, &PlanOptions::default())
}

/// Plans every device's response to `alert` (see the module docs).
pub fn plan_with(
    alert: &FusedAlert,
    reg: &DeviceRegistry,
    pred: &BuildingPrediction,
    opts: &PlanOptions,
) -> HousePlan {
    schedule(None, alert, reg, pred, opts)
}

/// Re-plans after new information (e.g. a local P pick confirming a network alert, or a
/// changed device status): actions already committed — protective actions in force,
/// motions whose start has passed, and obstructed devices (one attempt only) — are kept,
/// with the remaining power of running motions reserved; every other device is planned
/// again from its current status.
pub fn replan(
    prev: &HousePlan,
    alert: &FusedAlert,
    reg: &DeviceRegistry,
    pred: &BuildingPrediction,
    opts: &PlanOptions,
) -> HousePlan {
    schedule(Some(prev), alert, reg, pred, opts)
}

fn committed(a: &PlannedAction, now: Tick) -> bool {
    match a.status {
        PlanStatus::NotSecured => a.reason == Some(NotSecuredReason::Obstruction),
        PlanStatus::Planned => {
            if a.decision.moves() {
                a.start <= now
            } else {
                a.decision.action != SeismicAction::None
            }
        }
        PlanStatus::CoreFallback => false,
    }
}

fn schedule(
    prev: Option<&HousePlan>,
    alert: &FusedAlert,
    reg: &DeviceRegistry,
    pred: &BuildingPrediction,
    opts: &PlanOptions,
) -> HousePlan {
    let n = reg.len().min(MAX_DEVICES);
    let now = pred.now;
    let deadline = plan_deadline(alert, pred);
    let budget =
        (pred.p_bus_max_w.is_finite() && pred.p_bus_max_w > 0.0).then_some(pred.p_bus_max_w);
    let mut out = HousePlan {
        alert_id: alert.alert.id,
        deadline,
        len: n as u8,
        ..HousePlan::default()
    };
    let mut slots = [Slot::default(); MAX_DEVICES];
    let mut n_slots = 0usize;
    let mut evals: [Option<Eval>; MAX_DEVICES] = [None; MAX_DEVICES];

    for (i, e) in reg.iter().enumerate().take(n) {
        if let Some(p) = prev.and_then(|p| p.get(i)).filter(|a| committed(a, now)) {
            if let Some(slot) = out.actions.get_mut(i) {
                *slot = *p;
            }
            if p.decision.moves()
                && p.p_w > 0.0
                && p.finish > now
                && let Some(s) = slots.get_mut(n_slots)
            {
                *s = Slot {
                    start: now.0,
                    end: p.finish.0,
                    p_w: p.p_w,
                };
                n_slots += 1;
            }
            continue;
        }
        let ev = eval_device(i, e, alert, pred, opts, deadline);
        if let Some(slot) = evals.get_mut(i) {
            *slot = Some(ev);
        }
    }

    // Non-motion actions and refusals first (they need no power slot).
    let mut order = [0u8; MAX_DEVICES];
    let mut n_order = 0usize;
    for i in 0..n {
        let Some(ev) = evals.get(i).copied().flatten() else {
            continue;
        };
        let moving = ev.p_w > 0.0 || ev.decision.target.is_some();
        if ev.reason.is_none() && moving && ev.decision.moves() {
            if let Some(o) = order.get_mut(n_order) {
                *o = i as u8;
                n_order += 1;
            }
            continue;
        }
        let finish = now.plus(secs_to_ticks(ev.dur_s));
        let mut status = if ev.decision.action == SeismicAction::None {
            PlanStatus::NotSecured
        } else {
            PlanStatus::Planned
        };
        let mut reason = ev.reason;
        if ev.reason.is_some() {
            status = PlanStatus::NotSecured;
        } else if deadline.is_some_and(|d| finish > d) {
            // A protective action still applies; its completion is reported late.
            status = PlanStatus::NotSecured;
            reason = Some(NotSecuredReason::Deadline);
        }
        // Awaiting confirmation: hold until the latest start (deadline − motion time);
        // without a predicted deadline the hold lasts one motion time.
        let hold_until = ev.await_confirmation.then(|| {
            let dur = secs_to_ticks(ev.dur_s);
            deadline.map_or(now.plus(dur), |d| Tick(d.0.saturating_sub(dur)))
        });
        if let Some(slot) = out.actions.get_mut(i) {
            *slot = PlannedAction {
                device: i as u8,
                policy: ev.policy,
                decision: ev.decision,
                start: now,
                target_rank: ev.target_rank,
                status,
                reason,
                p_w: 0.0,
                finish,
                hold_until,
            };
        }
    }

    // Motions: longest first, ties by registry index (deterministic list schedule).
    let key = |i: u8| {
        evals
            .get(i as usize)
            .copied()
            .flatten()
            .map_or(0, |e| secs_to_ticks(e.dur_s))
    };
    for a in 1..n_order {
        let mut b = a;
        while b > 0 {
            let (Some(&x), Some(&y)) = (order.get(b - 1), order.get(b)) else {
                break;
            };
            let swap = key(y) > key(x) || (key(y) == key(x) && y < x);
            if !swap {
                break;
            }
            order.swap(b - 1, b);
            b -= 1;
        }
    }
    for &iu in order.iter().take(n_order) {
        let i = iu as usize;
        let Some(ev) = evals.get(i).copied().flatten() else {
            continue;
        };
        let dur = secs_to_ticks(ev.dur_s);
        let alone_finish = now.plus(dur);
        let placed = slots.get(..n_slots).unwrap_or(&[]);
        let start = earliest_fit(placed, now.0, dur, ev.p_w, budget);
        let (status, reason, start_t, finish) = match start {
            None => (
                PlanStatus::NotSecured,
                Some(NotSecuredReason::Power),
                now,
                alone_finish,
            ),
            Some(s) => {
                let f = Tick(s).plus(dur);
                if deadline.is_some_and(|d| alone_finish > d) {
                    (
                        PlanStatus::NotSecured,
                        Some(NotSecuredReason::Deadline),
                        now,
                        alone_finish,
                    )
                } else if deadline.is_some_and(|d| f > d) {
                    (
                        PlanStatus::NotSecured,
                        Some(NotSecuredReason::Power),
                        now,
                        f,
                    )
                } else {
                    (PlanStatus::Planned, None, Tick(s), f)
                }
            }
        };
        let planned = status == PlanStatus::Planned;
        if planned && let Some(s) = slots.get_mut(n_slots) {
            *s = Slot {
                start: start_t.0,
                end: finish.0,
                p_w: ev.p_w,
            };
            n_slots += 1;
        }
        if let Some(slot) = out.actions.get_mut(i) {
            *slot = PlannedAction {
                device: iu,
                policy: ev.policy,
                decision: if planned {
                    ev.decision
                } else {
                    ActionDecision::of(SeismicAction::None)
                },
                start: start_t,
                target_rank: if planned {
                    ev.target_rank
                } else {
                    SafetyRank::of(
                        reg.get(i).map_or(DeviceCondition::Moving, |e| e.condition),
                        reg.get(i).map_or(EgressRole::None, |e| e.egress),
                    )
                },
                status,
                reason,
                p_w: if planned { ev.p_w } else { 0.0 },
                finish,
                hold_until: None,
            };
        }
    }

    let placed = slots.get(..n_slots).unwrap_or(&[]);
    out.p_peak_w = placed
        .iter()
        .map(|s| load_at(placed, s.start))
        .fold(0.0f32, f32::max);
    out
}

// ---- QUIET (plan §4b) ---------------------------------------------------------------------

/// What a device does in QUIET.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum QuietAction {
    /// Nothing to do (already seated, landed, or not a drawer at a standoff).
    None,
    /// Seat: log-damped close to the seat at the unattended limit.
    Seat {
        /// Seat position.
        target: PosNm,
        /// Speed cap (unattended limit), m/s.
        v_cap_mps: f32,
    },
    /// The plausibility check failed: land in place and flag for a manual push-to-seat.
    LandAndFlag,
}

impl QuietAction {
    /// The decision the device's `PlanPolicy` proposes for this QUIET action: a seat is a
    /// LAND with the seat as target at the unattended limit; a failed plausibility check
    /// is a LAND in place.
    pub fn decision(&self) -> Option<ActionDecision> {
        match *self {
            QuietAction::None => None,
            QuietAction::Seat { target, v_cap_mps } => Some(ActionDecision {
                action: SeismicAction::Land,
                target: Some(target),
                v_cap_mps,
                deadline: None,
            }),
            QuietAction::LandAndFlag => Some(ActionDecision::of(SeismicAction::Land)),
        }
    }
}

/// QUIET: a DRAWER left levitated away from its seat (standoff, hold or frozen open)
/// seats at the unattended limit when the gap/encoder plausibility check passes,
/// otherwise it lands and is flagged for a manual push-to-seat (plan §4b). A user
/// motion in progress is never taken over.
pub fn quiet_action(e: &DeviceEntry, plausible: bool) -> QuietAction {
    let drawer = matches!(e.class, AnyClass::Core(RailClass::Drawer));
    let levitated_open = matches!(
        e.condition,
        DeviceCondition::Standoff
            | DeviceCondition::Hold
            | DeviceCondition::FrozenOpen
            | DeviceCondition::ClosedLevitated
    );
    if !drawer || !levitated_open || e.user_motion.is_some() {
        return QuietAction::None;
    }
    let c = &e.constraints;
    let v = if c.v_cap_mps > 0.0 {
        c.v_unattended_mps.min(c.v_cap_mps)
    } else {
        c.v_unattended_mps
    };
    if plausible && v.is_finite() && v > 0.0 && e.x.diff_nm(e.x_closed) >= 0 {
        QuietAction::Seat {
            target: e.x_closed,
            v_cap_mps: v,
        }
    } else {
        QuietAction::LandAndFlag
    }
}

/// Inputs of the gap/encoder plausibility check before a QUIET seat (plan §4b).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Plausibility {
    /// Encoder position.
    pub x: PosNm,
    /// Travel lower bound (closed stop).
    pub x_min: PosNm,
    /// Travel upper bound.
    pub x_max: PosNm,
    /// Position tolerance, m.
    pub tol_m: f32,
    /// Measured air gap, m.
    pub gap_m: f32,
    /// Safe gap band lower bound, m.
    pub gap_lo_m: f32,
    /// Safe gap band upper bound, m.
    pub gap_hi_m: f32,
    /// A gap, encoder or current sensor fault is active.
    pub sensor_fault: bool,
}

impl Plausibility {
    /// The encoder position lies inside the travel (with `tol_m`), the measured gap
    /// inside the safe band, and no sensor fault is active.
    pub fn ok(&self) -> bool {
        let in_travel =
            self.x.diff_m(self.x_min) >= -self.tol_m && self.x_max.diff_m(self.x) >= -self.tol_m;
        let gap_ok =
            self.gap_m.is_finite() && self.gap_m >= self.gap_lo_m && self.gap_m <= self.gap_hi_m;
        in_travel && gap_ok && !self.sensor_fault
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::choreo::registry::DeviceConstraints;
    use fabrail_exo_safety::{AlertFeedKind, FeedSet};
    use fabrail_types::{NetworkAlert, SeismicLevel};

    const S: u64 = BASE_TICK_HZ as u64;

    fn confirmed_alert(s_eta_s: f64, sigma: f32) -> FusedAlert {
        FusedAlert {
            alert: NetworkAlert {
                id: 3,
                received: Tick(150_000),
                magnitude: 7.2,
                r_epi_km: 60.0,
                depth_km: 12.0,
                ..NetworkAlert::default()
            },
            level: SeismicLevel::L2,
            s_eta: Some(Tick::from_seconds_f64(s_eta_s)),
            s_eta_sigma_s: sigma,
            feeds: FeedSet::EMPTY
                .with(AlertFeedKind::Network)
                .with(AlertFeedKind::LocalP),
            n_independent: 2,
            local_p: true,
            ..FusedAlert::default()
        }
    }

    fn open_drawer(x_m: f64, p_w: f32) -> DeviceEntry {
        DeviceEntry {
            class: AnyClass::Core(RailClass::Drawer),
            condition: DeviceCondition::FrozenOpen,
            x: PosNm::from_m_f64(x_m),
            m_gov_kg: 48.0,
            obstruction_armed: true,
            constraints: DeviceConstraints {
                v_cap_mps: 0.2654,
                v_unattended_mps: 0.2305,
                a_brk_mps2: 3.0,
                p_move_w: p_w,
                s_min_m: 0.05,
                latch_fitted: false,
                latch_retention_n: 0.0,
            },
            ..DeviceEntry::EMPTY
        }
    }

    #[test]
    fn deadline_quantile_math() {
        // Φ⁻¹ against tabulated values (Abramowitz & Stegun Table 26.5; 16 digits from
        // scipy.stats.norm.ppf), tolerance 1e-9.
        for (p, z) in [
            (0.05, -1.644_853_626_951_472_2),
            (0.5, 0.0),
            (0.975, 1.959_963_984_540_054),
            (0.001, -3.090_232_306_167_813_6),
            (0.2, -0.841_621_233_572_914_2),
        ] {
            let q = normal_quantile(p);
            assert!(
                (q - z).abs() < 1e-9,
                "Φ⁻¹({p}) = {q} vs {z} (±1e-9, A&S 26.5)"
            );
            assert!(
                (normal_quantile(1.0 - p) + z).abs() < 1e-9,
                "symmetry at {p}"
            );
        }
        assert!(normal_quantile(0.0).is_infinite() && normal_quantile(1.5).is_nan());
        // Warned villa variant (plan §4b): t_S = 17.5 s → deadline 14.855 s at σ 1 s and
        // 13.210 s at σ 2 s ("13.2–14.8 s"), ±1 tick.
        for (sigma, want) in [
            (1.0f32, 17.5 - 1.644_853_6 - 1.0),
            (2.0, 17.5 - 3.289_707 - 1.0),
        ] {
            let d = s_quantile_deadline(Tick::from_seconds_f64(17.5), sigma, 0.05, 1.0);
            assert!(
                (d.seconds_f64() - want).abs() <= 1.0 / S as f64,
                "deadline at σ {sigma}: {} vs {want} s (±1 tick; plan §4b)",
                d.seconds_f64()
            );
        }
        // The prediction's S-ETA wins over the alert's; without either there is none.
        let a = confirmed_alert(17.5, 1.0);
        let mut pred = BuildingPrediction::default();
        let from_alert = plan_deadline(&a, &pred).expect("alert S-ETA");
        assert_eq!(
            from_alert,
            s_quantile_deadline(Tick(350_000), 1.0, 0.05, 1.0)
        );
        pred.s_eta = Some(Tick::from_seconds_f64(20.0));
        pred.s_eta_sigma_s = 0.0;
        assert_eq!(plan_deadline(&a, &pred), Some(Tick::from_seconds_f64(19.0)));
        let no_eta = FusedAlert { s_eta: None, ..a };
        assert_eq!(plan_deadline(&no_eta, &BuildingPrediction::default()), None);
        // Bad quantile / margin fall back to 5 % / 1 s.
        assert_eq!(
            s_quantile_deadline(Tick(350_000), 1.0, f32::NAN, -3.0),
            s_quantile_deadline(Tick(350_000), 1.0, 0.05, 1.0)
        );
    }

    #[test]
    fn motion_time_closed_forms() {
        let k = DeviceKinematics::for_class(AnyClass::Core(RailClass::Drawer));
        // Long move: ramp + cruise + terminus exactly (PHYSICS §16, §16.1).
        let v = 0.2305f32;
        let (a, j) = (f64::from(k.a_max_mps2), f64::from(k.j_max_mps3));
        let (tr, dr) = ramp(f64::from(v), a, j);
        let b = derive::terminus_blend(
            f64::from(v),
            f64::from(k.terminus_j_mps3),
            f64::from(k.tau_s),
        );
        let ta = derive::terminus_t_arrive(
            f64::from(v),
            f64::from(k.terminus_j_mps3),
            f64::from(k.tau_s),
            f64::from(k.d_eps_m),
        );
        let want = tr + (0.30 - dr - b.d_0_m) / f64::from(v) + ta;
        let t = motion_time_s(0.30, v, &k);
        assert!(
            (f64::from(t) - want).abs() < 1e-5,
            "{t} vs {want} s (closed form)"
        );
        // Plan §4b: 0.30 m in about 2.0 s (+0.2 s seat) at the unattended limit.
        assert!(
            (1.6..2.6).contains(&t),
            "DRAWER 0.30 m close {t} s (plan §4b ≈ 2.0 s)"
        );
        assert!(
            (k.t_seat_s - 0.2).abs() < 1e-6,
            "seat step 0.2 s (PHYSICS §16.1)"
        );
        // Monotone in distance, continuous through the short-move branch.
        let mut last = 0.0f32;
        for mm in 1..=400 {
            let t = motion_time_s(mm as f32 * 1e-3, v, &k);
            assert!(
                t.is_finite() && t >= last - 1e-5,
                "monotone at {mm} mm: {t} < {last}"
            );
            last = t;
        }
        assert_eq!(motion_time_s(0.0, v, &k), 0.0);
        assert!(motion_time_s(0.1, 0.0, &k).is_infinite());
        assert!((stop_time_s(0.3, 3.0, 30.0) - (0.1 + 0.1)).abs() < 1e-6);
    }

    #[test]
    fn parallel_schedule_under_bus_power() {
        // Six open drawers at 0.30 m, 100 W each while moving (plan §4b).
        let mut reg = DeviceRegistry::new();
        for _ in 0..6 {
            assert!(reg.push(open_drawer(0.30, 100.0)).is_ok());
        }
        let a = confirmed_alert(30.0, 1.0);
        let now = a.alert.received;
        let mut pred = BuildingPrediction {
            now,
            p_bus_max_w: 250.0,
            ..BuildingPrediction::default()
        };
        let hp = plan(&a, &reg, &pred);
        let acts = hp.as_slice();
        assert!(
            acts.iter().all(|x| x.status == PlanStatus::Planned),
            "{acts:?}"
        );
        // At every start/finish instant the summed motion power stays within budget.
        for probe in acts.iter().flat_map(|x| [x.start, x.finish]) {
            let load: f32 = acts
                .iter()
                .filter(|x| x.start <= probe && probe < x.finish)
                .map(|x| x.p_w)
                .sum();
            assert!(
                load <= 250.0 + 1e-3,
                "load {load} W at tick {} > 250 W",
                probe.0
            );
        }
        assert!(
            hp.p_peak_w <= 250.0 && hp.p_peak_w >= 200.0,
            "peak {}",
            hp.p_peak_w
        );
        // Parallel: two at a time, so the makespan is 3 durations, not 6 (serial).
        let dur = acts[0].finish.since(acts[0].start);
        let makespan = acts.iter().map(|x| x.finish.0).max().unwrap_or(0) - now.0;
        assert_eq!(makespan, 3 * dur, "2 concurrent × 3 waves");
        assert_eq!(acts.iter().filter(|x| x.start == now).count(), 2);
        // Waiting motions hold (FREEZE) until their slot, then close.
        let late = acts.iter().find(|x| x.start > now).expect("staggered");
        assert_eq!(
            late.proposal(now),
            Some(ActionDecision::of(SeismicAction::Freeze))
        );
        assert_eq!(
            late.proposal(late.start).and_then(|d| d.target),
            Some(PosNm(0))
        );
        // Enough budget: all start at once.
        pred.p_bus_max_w = 600.0;
        let all = plan(&a, &reg, &pred);
        assert!(all.as_slice().iter().all(|x| x.start == now));
        assert!((all.p_peak_w - 600.0).abs() < 1e-3);
        // A tight deadline (t_S 5 % − 1 s ≈ 1.6 durations after now) secures only the
        // first wave; the rest are NotSecured(Power) and keep the core action.
        let d_s = now.seconds_f64() + 1.6 * dur as f64 / S as f64 + 1.0 + 1.644_853_6;
        let tight = confirmed_alert(d_s, 1.0);
        pred.p_bus_max_w = 250.0;
        let hp = plan(&tight, &reg, &pred);
        let ok = hp
            .as_slice()
            .iter()
            .filter(|x| x.status == PlanStatus::Planned)
            .count();
        assert_eq!(ok, 2, "{:?}", hp.as_slice());
        assert!(
            hp.as_slice()
                .iter()
                .filter(|x| x.status == PlanStatus::NotSecured)
                .all(|x| x.reason == Some(NotSecuredReason::Power) && x.proposal(now).is_none())
        );
        // A deadline shorter than one close: NotSecured(Deadline) for all.
        let d_s = now.seconds_f64() + 0.5 * dur as f64 / S as f64 + 1.0 + 1.644_853_6;
        let hp = plan(&confirmed_alert(d_s, 1.0), &reg, &pred);
        assert!(
            hp.as_slice()
                .iter()
                .all(|x| x.reason == Some(NotSecuredReason::Deadline))
        );
        // A single motion above the whole budget never fits.
        pred.p_bus_max_w = 50.0;
        let hp = plan(&a, &reg, &pred);
        assert!(
            hp.as_slice()
                .iter()
                .all(|x| x.reason == Some(NotSecuredReason::Power))
        );
    }

    #[test]
    fn replan_keeps_committed_and_upgrades_on_confirmation() {
        let mut reg = DeviceRegistry::new();
        assert!(reg.push(open_drawer(0.30, 100.0)).is_ok());
        let mut door = open_drawer(0.5, 0.0);
        door.class = AnyClass::Core(RailClass::Door);
        door.egress = EgressRole::NonRated;
        door.condition = DeviceCondition::Hold;
        assert!(reg.push(door).is_ok());
        // Network-only first: the drawer may not move, the door lands.
        let net = FusedAlert {
            feeds: FeedSet::EMPTY.with(AlertFeedKind::Network),
            n_independent: 1,
            local_p: false,
            ..confirmed_alert(17.5, 1.0)
        };
        let now = net.alert.received;
        let pred = BuildingPrediction {
            now,
            p_bus_max_w: 500.0,
            ..BuildingPrediction::default()
        };
        let p1 = plan(&net, &reg, &pred);
        assert_eq!(p1.as_slice()[0].reason, Some(NotSecuredReason::Gate));
        assert_eq!(
            p1.as_slice()[1].decision,
            ActionDecision::of(SeismicAction::Land)
        );
        // The local P pick confirms 2.7 s later: the drawer is now planned; the door's
        // landing is committed and kept as it was.
        let later = BuildingPrediction {
            now: now.plus(54_000),
            ..pred
        };
        let p2 = replan(
            &p1,
            &confirmed_alert(17.5, 1.0),
            &reg,
            &later,
            &PlanOptions::default(),
        );
        assert_eq!(p2.as_slice()[1], p1.as_slice()[1]);
        assert_eq!(p2.as_slice()[0].status, PlanStatus::Planned);
        assert_eq!(p2.as_slice()[0].start, later.now);
    }

    // req: FAB-EXA-016
    #[test]
    fn replay_bit_identical() {
        // A recorded FusedAlert and device status replay bit-identically through the
        // choreographer (plan §4b "calculated": deterministic plan, byte-identical
        // replay). The record is a flat little-endian byte image of every input field.
        fn put(v: &mut Vec<u8>, x: u64) {
            v.extend_from_slice(&x.to_le_bytes());
        }
        fn record(a: &FusedAlert, reg: &DeviceRegistry) -> Vec<u8> {
            let mut v = Vec::new();
            let n = &a.alert;
            for x in [
                u64::from(n.id),
                n.received.0,
                u64::from(n.origin_offset_s.to_bits()),
                u64::from(n.magnitude.to_bits()),
                u64::from(n.r_epi_km.to_bits()),
                u64::from(n.depth_km.to_bits()),
                u64::from(n.mmi.to_bits()),
                u64::from(n.cancel),
                u64::from(a.level.as_u8()),
                u64::from(a.p_damaging.to_bits()),
                a.s_eta.map_or(u64::MAX, |t| t.0),
                u64::from(a.s_eta_sigma_s.to_bits()),
                u64::from(a.long_period),
                a.tsunami_eta.map_or(u64::MAX, |t| t.0),
                u64::from(a.feeds.0),
                u64::from(a.n_independent),
                u64::from(a.local_p),
                u64::from(a.injected),
            ] {
                put(&mut v, x);
            }
            for e in reg.iter() {
                for x in [
                    u64::from(e.condition.as_u8()),
                    e.x.0 as u64,
                    u64::from(e.v_mps.to_bits()),
                    u64::from(e.m_gov_kg.to_bits()),
                    e.user_motion.map_or(9, |d| u64::from(d.as_u8())),
                ] {
                    put(&mut v, x);
                }
            }
            v
        }
        fn replay(bytes: &[u8], cfg: &DeviceRegistry) -> (FusedAlert, DeviceRegistry) {
            let w: Vec<u64> = bytes
                .chunks_exact(8)
                .map(|c| u64::from_le_bytes(c.try_into().unwrap_or([0; 8])))
                .collect();
            let f = |i: usize| f32::from_bits(w[i] as u32);
            let t = |i: usize| (w[i] != u64::MAX).then_some(Tick(w[i]));
            let a = FusedAlert {
                alert: NetworkAlert {
                    id: w[0] as u32,
                    received: Tick(w[1]),
                    origin_offset_s: f(2),
                    magnitude: f(3),
                    r_epi_km: f(4),
                    depth_km: f(5),
                    mmi: f(6),
                    cancel: w[7] != 0,
                },
                level: SeismicLevel::from_u8(w[8] as u8).unwrap_or_default(),
                p_damaging: f(9),
                s_eta: t(10),
                s_eta_sigma_s: f(11),
                long_period: w[12] != 0,
                tsunami_eta: t(13),
                feeds: FeedSet(w[14] as u8),
                n_independent: w[15] as u8,
                local_p: w[16] != 0,
                injected: w[17] != 0,
            };
            let mut reg = DeviceRegistry::new();
            for (k, e) in cfg.iter().enumerate() {
                let b = 18 + 5 * k;
                let mut e = *e;
                e.condition = DeviceCondition::from_u8(w[b] as u8).unwrap_or_default();
                e.x = PosNm(w[b + 1] as i64);
                e.v_mps = f(b + 2);
                e.m_gov_kg = f(b + 3);
                e.user_motion = fabrail_types::TravelDir::from_u8(w[b + 4] as u8);
                assert!(reg.push(e).is_ok());
            }
            (a, reg)
        }
        fn digest(p: &HousePlan) -> u64 {
            // FNV-1a 64 over every output bit (the golden selftest hash's digest).
            let mut h: u64 = 0xcbf2_9ce4_8422_2325;
            let mut eat = |x: u64| {
                for b in x.to_le_bytes() {
                    h ^= u64::from(b);
                    h = h.wrapping_mul(0x0000_0100_0000_01b3);
                }
            };
            eat(u64::from(p.len));
            eat(p.deadline.map_or(u64::MAX, |t| t.0));
            eat(u64::from(p.p_peak_w.to_bits()));
            eat(u64::from(p.alert_id));
            for a in p.as_slice() {
                eat(u64::from(a.device));
                eat(u64::from(a.policy as u8));
                eat(u64::from(a.decision.action.as_u8()));
                eat(a.decision.target.map_or(u64::MAX, |t| t.0 as u64));
                eat(u64::from(a.decision.v_cap_mps.to_bits()));
                eat(a.decision.deadline.map_or(u64::MAX, |t| t.0));
                eat(a.start.0);
                eat(a.finish.0);
                eat(u64::from(a.target_rank.as_u8()));
                eat(u64::from(a.status as u8));
                eat(a.reason.map_or(u64::MAX, |r| r as u64));
                eat(u64::from(a.p_w.to_bits()));
            }
            h
        }
        let mut reg = DeviceRegistry::new();
        for (k, x) in [0.30, 0.12, 0.0, 0.27, 0.05, 0.30].into_iter().enumerate() {
            let mut e = open_drawer(x, 60.0 + 7.0 * k as f32);
            if x == 0.0 {
                e.condition = DeviceCondition::Seated;
            }
            if k == 3 {
                e.user_motion = Some(fabrail_types::TravelDir::Open);
                e.condition = DeviceCondition::Moving;
                e.v_mps = 0.11;
            }
            assert!(reg.push(e).is_ok());
        }
        let mut door = open_drawer(0.6, 0.0);
        door.class = AnyClass::Core(RailClass::Door);
        door.egress = EgressRole::NonRated;
        door.v_mps = -0.12;
        assert!(reg.push(door).is_ok());
        let a = FusedAlert {
            alert: NetworkAlert {
                origin_offset_s: -7.5,
                mmi: 7.0,
                ..confirmed_alert(17.5, 1.3).alert
            },
            p_damaging: 0.731,
            ..confirmed_alert(17.5, 1.3)
        };
        let pred = BuildingPrediction {
            now: a.alert.received,
            p_bus_max_w: 150.0,
            ..BuildingPrediction::default()
        };
        let live = plan(&a, &reg, &pred);
        let tape = record(&a, &reg);
        let (a2, reg2) = replay(&tape, &reg);
        assert_eq!(record(&a2, &reg2), tape, "the record round-trips");
        let again = plan(&a2, &reg2, &pred);
        assert_eq!(again, live, "replayed plan equals the live plan");
        assert_eq!(
            digest(&again),
            digest(&live),
            "bit-identical (FNV-1a over every bit)"
        );
        for _ in 0..16 {
            assert_eq!(digest(&plan(&a2, &reg2, &pred)), digest(&live));
        }
        // The plan is not trivially constant: a different status changes it.
        let mut moved = reg2;
        if let Some(e) = moved.get_mut(1) {
            e.x = PosNm::from_m_f64(0.2);
        }
        assert_ne!(digest(&plan(&a2, &moved, &pred)), digest(&live));
    }

    #[test]
    fn quiet_seats_after_plausibility_else_flags() {
        let mut e = open_drawer(0.04, 0.0);
        e.condition = DeviceCondition::Standoff;
        assert_eq!(
            quiet_action(&e, true),
            QuietAction::Seat {
                target: PosNm(0),
                v_cap_mps: 0.2305
            }
        );
        assert_eq!(quiet_action(&e, false), QuietAction::LandAndFlag);
        e.user_motion = Some(fabrail_types::TravelDir::Close);
        assert_eq!(
            quiet_action(&e, true),
            QuietAction::None,
            "user motion kept"
        );
        e.user_motion = None;
        e.condition = DeviceCondition::Seated;
        assert_eq!(quiet_action(&e, true), QuietAction::None);
        let good = Plausibility {
            x: PosNm::from_m_f64(0.1),
            x_min: PosNm(0),
            x_max: PosNm::from_m_f64(0.3),
            tol_m: 1e-3,
            gap_m: 4.0e-3,
            gap_lo_m: 2.0e-3,
            gap_hi_m: 7.0e-3,
            sensor_fault: false,
        };
        assert!(good.ok());
        assert!(
            !Plausibility {
                gap_m: 9.0e-3,
                ..good
            }
            .ok()
        );
        assert!(
            !Plausibility {
                sensor_fault: true,
                ..good
            }
            .ok()
        );
        assert!(
            !Plausibility {
                x: PosNm::from_m_f64(0.4),
                ..good
            }
            .ok()
        );
    }
}
