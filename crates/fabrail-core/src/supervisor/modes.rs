//! MotionMode selection, segment planning (S-curve, terminus, quintic) and Directive assembly (PHYSICS §18). Owner: A4.
//!
//! Frozen API (W0): [`ModeInputs`], [`ModePlanner::new`], [`ModePlanner::directive`]
//! (everything in the `Directive` except `seq`, which the supervisor stamps). Planning
//! uses `motion::scurve::plan` for cruise and `motion::terminus::TerminusPlanner` for
//! the log-damped open/close termini (SoftOpen / Terminus modes), with live limits from
//! the class, sensing and seismic state (unattended limits for choreographed motion).
//!
//! Division of labour: the supervisor (`supervisor/mod.rs`) decides WHAT happens each
//! release (the levitation command and a [`MotionCmd`]); this planner decides HOW:
//!
//! * **Moves** ([`ModePlanner::start_move`]). A move to a travel end (the closed/seat
//!   stop or the open end) is a 7-segment S-curve cruise (`MotionMode::Trajectory`)
//!   handed over to the log-damped terminus (PHYSICS §16.1) at the tick `t_h` where the
//!   S-curve's distance-to-go equals d_0 = d_b + v·t_b − j·t_b³/6 (found by bisection
//!   on the closed-form `scurve::eval`). The terminus directive is published at the
//!   first supervisor release whose activation tick r + P ≥ t_h, with `t0 = t_h`: in
//!   the ≤ P window the fast group still tracks the cruise, which differs from the
//!   blend by j·Δt³/6 ≤ 3.3 nm for DRAWER (j 20 m/s³, Δt ≤ 1 ms). Both ends are at
//!   cruise speed there because d_0 exceeds the S-curve's own braking distance in
//!   every class (49.8 vs 33.4 mm DRAWER; plan §4b table). Interior targets (GoTo) and
//!   moves too short to reach cruise before d_0 use the S-curve alone. A drawer
//!   released by the hand while moving toward an end within the capture window gets a
//!   terminus planned from the measured (d, v, a) with τ_eff = max(τ, τ_min(v)).
//! * **Holds** (Hold, Freeze): a zero-length segment anchored once per episode at the
//!   natural stopping point x + sign(v)·d_stop(|v|, a, j) (a = a_max for Hold,
//!   a_brk = 2·a_max for Freeze; PHYSICS §16), so a stop never pulls the carriage back.
//! * **Live limits** (PHYSICS §16; SAF-001…004): unsensed caps unless PL d presence
//!   sensing is fitted, healthy and the zone is clear (then the sensed caps, a_max and
//!   150 N static); the unattended close limit min(v_cap, F_pinch/√(m_gov·k_series))
//!   for automatic unattended motion (plan §4b); zero speed caps at rest and whenever
//!   hold-to-run is required but not actuated; governor observation points at the
//!   travel ends, or terminus target + overtravel during a terminus move (plan §4b).
//! * **Profiles**: the haptic admittance profile (M_v = m̂/assist ratio, B_v, detents,
//!   soft-close, passivity; intent only in NORMAL), the terminus runtime configuration
//!   (hand yields only in NORMAL), and the research-tier inertial / isolation blocks,
//!   which are filled only when their preset `enable` flag is set (never in a preset).

use fabrail_params::{ParamError, RestPolicy, SystemParams};
use fabrail_types::{
    LevState, MotionMode, PosNm, PresenceSensing, SCurveSeg, Segment, SeismicState, Tick,
    TravelDir, UserCommand, fmath,
};

use crate::motion::{scurve, terminus::TerminusPlanner};
use crate::msg::{
    Directive, FastStatus, HapticProfile, InertialCfg, IsolateCfg, LevDirective, LiveLimits,
    TerminusCfg,
};
use crate::seismic::action_policy::ActionDecision;

/// Base ticks per second as f32 (PHYSICS §24: time is `Tick` at 20 kHz).
const TICK_HZ: f32 = fabrail_types::BASE_TICK_HZ as f32;

/// What the fast group should do with propulsion this release (chosen by the
/// supervisor; rendered into a `MotionMode` and segment by the planner).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum MotionCmd {
    /// Propulsion off (STO on the LSM; also "release to 0 N" after an obstruction).
    #[default]
    Off,
    /// Position hold (a controlled stop at a_max when moving).
    Hold,
    /// Seismic/fault freeze: brake at a_brk = 2·a_max, then hold.
    Freeze,
    /// Hand-driven haptic admittance.
    Admittance,
    /// Execute the active move plan (Trajectory, then Terminus/SoftOpen).
    Move,
    /// Obstruction reversal along the active reverse plan.
    Reverse,
    /// Contactless isolation (research tier).
    Isolate,
    /// Active mass damping (research tier).
    Damp,
    /// Inertial smoothing (research tier).
    InertialSmooth,
}

/// Why a move was planned (logging, limits and arrival handling).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum MoveKind {
    /// User command or confirmed intent.
    #[default]
    Command,
    /// Hand-released drawer captured by a terminus near a travel end.
    Capture,
    /// DRAWER seat-first SS1 from the standoff (plan §4b).
    SeatFirst,
    /// Admitted seismic target (unattended, closing only).
    Seismic,
    /// Egress fail-open (ESCAPE).
    Escape,
    /// Obstruction reversal (DOOR/WALL, 50 mm).
    Reverse,
}

/// The active move.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MovePlan {
    /// A plan is active.
    pub active: bool,
    /// Why.
    pub kind: MoveKind,
    /// Target position.
    pub target: PosNm,
    /// Direction toward the target.
    pub dir: TravelDir,
    /// Cruise speed cap, m/s.
    pub v_cap_mps: f32,
    /// Unattended automatic motion (unattended limits, obstruction armed).
    pub unattended: bool,
    /// DRAWER: z-seat step on arrival at the seat.
    pub seat_on_arrival: bool,
    /// Cruise S-curve.
    pub cruise: SCurveSeg,
    /// Tick at which the cruise hands over to the terminus, if any.
    pub handover: Option<Tick>,
    /// The terminus has been published.
    pub captured: bool,
    /// Segment the fast group tracks.
    pub segment: Segment,
    /// Mode that tracks `segment`.
    pub mode: MotionMode,
    /// Start tick.
    pub t_start: Tick,
    /// Closed-form expected end tick.
    pub t_end: Tick,
    /// Arrival position tolerance, m.
    pub tol_m: f32,
    /// Directive sequence number that first carried `segment`.
    pub seg_seq: u32,
    /// `segment` changed this release (the supervisor stamps `seg_seq`).
    pub seg_new: bool,
}

/// A move request for [`ModePlanner::start_move`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MoveStart {
    /// Why.
    pub kind: MoveKind,
    /// Target position.
    pub target: PosNm,
    /// Speed cap, m/s (> 0).
    pub v_cap_mps: f32,
    /// Acceleration limit, m/s².
    pub a_mps2: f32,
    /// Unattended automatic motion.
    pub unattended: bool,
    /// Activation tick of the directive that will carry the plan (release + P).
    pub act: Tick,
    /// Tick of the measurement (`FastStatus::tick`).
    pub t_meas: Tick,
    /// Measured position.
    pub x: PosNm,
    /// Measured velocity, m/s.
    pub v_mps: f32,
    /// Measured acceleration, m/s².
    pub a_meas_mps2: f32,
}

/// Inputs to mode selection for one supervisor release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModeInputs {
    /// Base tick of the release.
    pub tick: Tick,
    /// LevState after this release's transition.
    pub lev: LevState,
    /// Seismic state.
    pub seis: SeismicState,
    /// Seismic action in force (after the ActionPolicy admission filter).
    pub action: ActionDecision,
    /// Command accepted this release, if any.
    pub cmd: Option<UserCommand>,
    /// Latest fast status.
    pub fast: FastStatus,
    /// Hold-to-run actuated.
    pub hold_to_run: bool,
    /// Presence sensing healthy and fitted (sensed limits apply).
    pub sensed: bool,
    /// Floor velocity estimate, m/s.
    pub floor_v_mps: f32,
    /// Activation tick of this release's directive (tick + supervisor period; LET).
    pub act: Tick,
    /// Levitation command chosen by the supervisor.
    pub lev_cmd: LevDirective,
    /// Propulsion command chosen by the supervisor.
    pub motion: MotionCmd,
    /// Motion is hold-to-run only (HEAVY unsensed, PayloadMismatch, NotReferenced).
    pub hold_to_run_required: bool,
    /// Automatic unattended motion (unattended limits).
    pub unattended: bool,
    /// Intent detection enabled (NORMAL, allowed state).
    pub intent_enable: bool,
    /// DRAWER push-to-open from the seat enabled.
    pub push_to_open: bool,
}

/// Mode selector and segment planner.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModePlanner {
    /// Active segment start tick (re-plans stamp a new one).
    pub seg_t0: Tick,
    /// Active move.
    pub plan: MovePlan,
    /// Terminus planner (E1).
    pub term: TerminusPlanner,
    /// The fast group's energy/momentum governor (A2), so moves are planned inside its
    /// steady speed envelope (W1.5).
    pub gov: crate::motion::governor::Governor,
    /// Closed / seat stop.
    pub x_closed: PosNm,
    /// Open travel end.
    pub x_open: PosNm,
    /// Rest policy.
    pub rest: RestPolicy,
    /// PL d presence sensing fitted.
    pub presence_fitted: bool,
    /// Unsensed opening / closing caps, m/s.
    pub v_open_u: f32,
    /// Unsensed closing cap, m/s.
    pub v_close_u: f32,
    /// Sensed opening cap, m/s.
    pub v_open_s: f32,
    /// Sensed closing cap, m/s.
    pub v_close_s: f32,
    /// Unattended automatic-motion speed limit, m/s.
    pub v_unattended: f32,
    /// a_max unsensed, m/s².
    pub a_max: f32,
    /// a_max sensed, m/s².
    pub a_max_sensed: f32,
    /// Jerk limit, m/s³.
    pub j_max: f32,
    /// a_brk / a_max.
    pub a_brk_ratio: f32,
    /// Terminus blend jerk, m/s³.
    pub j_term: f32,
    /// Terminus capture distance d_ε, m.
    pub d_eps_m: f32,
    /// T_q / τ.
    pub t_q_ratio: f32,
    /// Governor overtravel beyond a terminus target, m.
    pub overtravel_m: f32,
    /// Re-plan braking ratio.
    pub replan_brk_ratio: f32,
    /// Hand onset during capture, s.
    pub hand_onset_s: f32,
    /// Closed / travel position tolerances, m.
    pub tol_closed_m: f32,
    /// Travel position tolerance, m.
    pub tol_travel_m: f32,
    /// Driving-force limit, N.
    pub f_drive_n: f32,
    /// Static force unsensed / sensed, N.
    pub f_static_u: f32,
    /// Static force sensed, N.
    pub f_static_s: f32,
    /// Closing force in the last zone, N.
    pub f_close_last50_n: f32,
    /// Rated mass, kg.
    pub m_rated_kg: f32,
    /// Assist ratio m/M_v.
    pub mass_ratio: f32,
    /// Virtual damping, N·s/m.
    pub b_v: f32,
    /// Active detents.
    pub n_detents: u8,
    /// Detent positions.
    pub detents: [PosNm; 4],
    /// Detent force, N.
    pub detent_f_n: f32,
    /// Detent width, m.
    pub detent_width_m: f32,
    /// Inertial smoothing (research tier) configuration; `enable` = preset flag.
    pub inertial: InertialCfg,
    /// Isolation / AMD (research tier) configuration template.
    pub isolate: IsolateCfg,
    /// Isolation enabled in the preset.
    pub isolation_enable: bool,
    /// AMD enabled in the preset.
    pub amd_enable: bool,
    /// Hold/freeze episode anchor.
    pub hold_anchor: Option<PosNm>,
    /// The episode anchor was computed for a freeze.
    pub hold_is_freeze: bool,
}

/// Jerk-limited stopping distance v²/(2a) + v·a/(2j) when v ≥ a²/j, else v·√(v/j)
/// (PHYSICS §16; `params::derive::stop_distance` in f32), m.
pub fn stop_distance_m(v_mps: f32, a_mps2: f32, j_mps3: f32) -> f32 {
    let v = fmath::absf(v_mps);
    if a_mps2 <= 0.0 || j_mps3 <= 0.0 || !v.is_finite() {
        return 0.0;
    }
    if v >= a_mps2 * a_mps2 / j_mps3 {
        v * v / (2.0 * a_mps2) + v * a_mps2 / (2.0 * j_mps3)
    } else {
        v * fmath::sqrtf(v / j_mps3)
    }
}

/// Fraction of the governor's steady speed envelope a move is planned at (design
/// choice, W1.5: 2 % headroom for the position loop's correction under the velocity
/// limiter).
pub const GOV_PLAN_FRAC: f32 = 0.98;

/// Terminus blend out of cruise (PHYSICS §16.1; plan §4b): t_b = √(τ² + 2v/j) − τ,
/// d_b = j·t_b·τ², d_0 = d_b + v·t_b − j·t_b³/6. Returns (t_b, d_b, d_0) in s, m, m.
pub fn terminus_blend(v_mps: f32, j_mps3: f32, tau_s: f32) -> (f32, f32, f32) {
    let v = fmath::absf(v_mps);
    if j_mps3 <= 0.0 || tau_s <= 0.0 || !v.is_finite() {
        return (0.0, 0.0, 0.0);
    }
    let t_b = fmath::sqrtf(tau_s * tau_s + 2.0 * v / j_mps3) - tau_s;
    let d_b = j_mps3 * t_b * tau_s * tau_s;
    (t_b, d_b, d_b + v * t_b - j_mps3 * t_b * t_b * t_b / 6.0)
}

/// Duration of a planned segment in closed form, s: S-curve `scurve::duration_s`;
/// terminus t_b + τ·ln(d_b/d_ε) + T_q (PHYSICS §16.1); quintic T_q.
pub fn segment_duration_s(seg: &Segment) -> f32 {
    match seg {
        Segment::SCurve(s) => scurve::duration_s(s),
        Segment::Terminus(t) => {
            let capture = if t.d_eps_m > 0.0 && t.d_b_m > t.d_eps_m {
                t.tau_s * fmath::lnf(t.d_b_m / t.d_eps_m)
            } else {
                0.0
            };
            t.t_b_s + capture + t.t_q_s
        }
        Segment::Quintic(q) => q.t_q_s,
    }
}

/// Seconds → ticks, rounded up, non-negative (PHYSICS §24).
pub fn ticks_ceil(t_s: f32) -> u64 {
    if t_s.is_finite() && t_s > 0.0 {
        fmath::ceilf(t_s * TICK_HZ) as u64
    } else {
        0
    }
}

/// Smallest tick in `[t_lo, t_hi]` at which the monotonically non-increasing
/// distance-to-go `dist(t)` is ≤ `d0_m` (bisection on integer ticks), or `None` when
/// even `t_hi` is still farther.
pub fn first_tick_within(
    t_lo: Tick,
    t_hi: Tick,
    d0_m: f32,
    mut dist: impl FnMut(Tick) -> f32,
) -> Option<Tick> {
    if t_hi < t_lo || dist(t_hi) > d0_m {
        return None;
    }
    if dist(t_lo) <= d0_m {
        return Some(t_lo);
    }
    let (mut lo, mut hi) = (t_lo.0, t_hi.0);
    while hi - lo > 1 {
        let mid = lo + (hi - lo) / 2;
        if dist(Tick(mid)) <= d0_m {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    Some(Tick(hi))
}

impl ModePlanner {
    /// Planner from the motion, haptic and safety blocks.
    pub fn new(p: &SystemParams) -> Result<ModePlanner, ParamError> {
        let d = &p.design;
        let mo = &p.control.motion;
        let hp = &p.control.haptic;
        let sf = &p.safety;
        let t = &mo.terminus;
        let x_closed = PosNm::from_m_f64(d.rail.x_closed_m);
        let x_open = PosNm::from_m_f64(d.rail.x_closed_m + d.rail.travel_m);
        let v_unattended = fabrail_params::derive::unattended_close_speed(
            sf.v_close_max_mps,
            sf.unattended.f_pinch_n,
            d.carriage.m_rated_kg,
            sf.unattended.k_series_n_per_m,
        ) as f32;
        let n_detents = hp.n_detents.min(4);
        let mut detents = [x_closed; 4];
        for (k, (slot, &dx)) in detents.iter_mut().zip(hp.detent_x_m.iter()).enumerate() {
            if (k as u8) < n_detents {
                *slot = PosNm::from_m_f64(d.rail.x_closed_m + dx);
            }
        }
        let inr = &mo.inertial;
        let standoff = PosNm::from_m_f64(d.rail.x_closed_m + inr.standoff_m);
        let inertial = InertialCfg {
            enable: inr.enable,
            f_c_hz: inr.f_c_hz as f32,
            zeta: inr.zeta as f32,
            hp_hz: inr.hp_hz as f32,
            standoff,
            stroke_m: fmath::fmax(inr.standoff_m - inr.stroke_margin_m, 0.0) as f32,
            f_cont_n: d.lsm.f_cont_n as f32,
            f_burst_n: inr.f_burst_n as f32,
            t_burst_s: inr.t_burst_s as f32,
        };
        // Isolation k = m·(2π/T)², c_sky = 2ζ·m·(2π/T) (PHYSICS §21).
        let iso = &p.control.seismic.isolation;
        let w = if iso.t_iso_s > 0.0 {
            2.0 * core::f64::consts::PI / iso.t_iso_s
        } else {
            0.0
        };
        let m = d.carriage.m_rated_kg;
        let isolate = IsolateCfg {
            k_n_per_m: (m * w * w) as f32,
            c_sky_n_s_per_m: (2.0 * iso.zeta * m * w) as f32,
            center: x_closed,
            stroke_m: iso.stroke_m as f32,
            f_max_n: d.lsm.f_cont_n as f32,
            c_dvf_n_s_per_m: p.control.seismic.amd.c_dvf_n_s_per_m as f32,
        };
        Ok(ModePlanner {
            seg_t0: Tick::ZERO,
            plan: MovePlan::default(),
            term: TerminusPlanner::new(p)?,
            gov: crate::motion::governor::Governor::new(p)?,
            x_closed,
            x_open,
            rest: d.rest_policy,
            presence_fitted: d.presence == PresenceSensing::PlD,
            v_open_u: sf.v_open_max_mps as f32,
            v_close_u: sf.v_close_max_mps as f32,
            v_open_s: sf.v_open_sensed_mps as f32,
            v_close_s: sf.v_close_sensed_mps as f32,
            v_unattended,
            a_max: mo.a_max_mps2 as f32,
            a_max_sensed: mo.a_max_sensed_mps2 as f32,
            j_max: mo.j_max_mps3 as f32,
            a_brk_ratio: mo.a_brk_ratio as f32,
            j_term: t.j_mps3 as f32,
            d_eps_m: t.d_eps_m as f32,
            t_q_ratio: t.t_q_ratio as f32,
            overtravel_m: t.overtravel_m as f32,
            replan_brk_ratio: t.replan_brk_ratio as f32,
            hand_onset_s: hp.hand_onset_s as f32,
            tol_closed_m: mo.pos_tol_closed_m as f32,
            tol_travel_m: mo.pos_tol_travel_m as f32,
            f_drive_n: sf.f_drive_max_n as f32,
            f_static_u: sf.f_static_n as f32,
            f_static_s: sf.f_static_sensed_n as f32,
            f_close_last50_n: sf.f_close_last50_n as f32,
            m_rated_kg: m as f32,
            mass_ratio: hp.mass_ratio as f32,
            b_v: hp.b_v_n_s_per_m as f32,
            n_detents,
            detents,
            detent_f_n: hp.detent_f_n as f32,
            detent_width_m: hp.detent_width_m as f32,
            inertial,
            isolate,
            isolation_enable: iso.enable,
            amd_enable: p.control.seismic.amd.enable,
            hold_anchor: None,
            hold_is_freeze: false,
        })
    }

    /// Speed caps (open, close) and a_max for the sensing state (SAF-001…003).
    pub fn caps(&self, sensed: bool, unattended: bool) -> (f32, f32, f32) {
        let (mut vo, mut vc, a) = if sensed {
            (self.v_open_s, self.v_close_s, self.a_max_sensed)
        } else {
            (self.v_open_u, self.v_close_u, self.a_max)
        };
        if unattended {
            // Unattended limit min(v_cap, F_pinch/√(m_gov·k_series)) (plan §4b).
            vo = fmath::fminf(vo, self.v_unattended);
            vc = fmath::fminf(vc, self.v_unattended);
        }
        (vo, vc, a)
    }

    /// Planned cruise speed inside the governor's steady envelope (W1.5; SAF-001): the
    /// governor limits the fast loop's velocity command to min(√(2E/m_gov), p/m_gov)
    /// − a_brk·t_react with m_gov = max(m_rated, 1.05·m̂) (`motion::governor`), which is
    /// below the class cap computed at m_rated. A plan above it would lag the reference
    /// by the speed difference times the cruise time (≈ 30 mm on a 0.8 m DOOR move) and
    /// the terminus would start behind. `GOV_PLAN_FRAC` leaves the position loop its
    /// correction headroom.
    pub fn governed_speed(&self, v_mps: f32, dir: TravelDir, m_hat_kg: f32, a_max: f32) -> f32 {
        let far = PosNm(i64::MAX / 4);
        let lim = LiveLimits {
            v_open_mps: v_mps,
            v_close_mps: v_mps,
            a_max_mps2: a_max,
            x_obs_open: far,
            x_obs_close: PosNm(-far.0),
            ..LiveLimits::default()
        };
        let v_gov = self.gov.v_lim_mps(PosNm::ZERO, dir, m_hat_kg, &lim);
        if v_gov > 0.0 {
            fmath::fminf(v_mps, GOV_PLAN_FRAC * v_gov)
        } else {
            v_mps
        }
    }

    /// Position tolerance for arrival at `target` (closed vs travel; PRP-006).
    pub fn tol_for(&self, target: PosNm) -> f32 {
        if target == self.x_closed {
            self.tol_closed_m
        } else {
            self.tol_travel_m
        }
    }

    /// Clamps a target into the travel [x_closed, x_open].
    pub fn clamp_target(&self, x: PosNm) -> PosNm {
        let (lo, hi) = if self.x_closed <= self.x_open {
            (self.x_closed, self.x_open)
        } else {
            (self.x_open, self.x_closed)
        };
        if x < lo {
            lo
        } else if x > hi {
            hi
        } else {
            x
        }
    }

    /// Plans a move (see the module docs). Returns the capture event payload
    /// (d_b, τ_eff) when a terminus was planned immediately (hand-released capture).
    pub fn start_move(&mut self, m: &MoveStart) -> Option<(f32, f32)> {
        let target = self.clamp_target(m.target);
        let dist = target.diff_m(m.x);
        let dir = if dist < 0.0 {
            TravelDir::Close
        } else {
            TravelDir::Open
        };
        let d = fmath::absf(dist);
        let tol = self.tol_for(target);
        let term_mode = match dir {
            TravelDir::Close => MotionMode::Terminus,
            TravelDir::Open => MotionMode::SoftOpen,
        };
        let v_cap = if m.v_cap_mps.is_finite() && m.v_cap_mps > 0.0 {
            m.v_cap_mps
        } else {
            0.0
        };
        let mut plan = MovePlan {
            active: true,
            kind: m.kind,
            target,
            dir,
            v_cap_mps: v_cap,
            unattended: m.unattended,
            seat_on_arrival: self.rest == RestPolicy::Seat && target == self.x_closed,
            t_start: m.act,
            tol_m: tol,
            seg_new: true,
            ..MovePlan::default()
        };
        let end_target = target == self.x_closed || target == self.x_open;
        let mut capture = None;
        if m.kind == MoveKind::Capture && end_target {
            // Terminus from the measured state (τ_eff covers hand-thrown drawers).
            let v_app = fmath::absf(m.v_mps);
            let tau = self.term.tau_eff(v_app);
            let a_along = m.a_meas_mps2 * dir.sign();
            let seg = self
                .term
                .plan(target, dir, m.t_meas, d, v_app, a_along, tau, self.j_term);
            plan.segment = seg;
            plan.mode = term_mode;
            plan.captured = true;
            plan.t_start = m.t_meas;
            plan.t_end = m.t_meas.plus(ticks_ceil(segment_duration_s(&seg)));
            capture = Some(match seg {
                Segment::Terminus(t) => (t.d_b_m, t.tau_s),
                _ => (d, tau),
            });
        } else {
            let a = if m.a_mps2 > 0.0 { m.a_mps2 } else { self.a_max };
            let cruise = scurve::plan(m.x, m.act, target, v_cap, a, self.j_max);
            let dur = scurve::duration_s(&cruise);
            let t_cruise_end = m.act.plus(ticks_ceil(dur));
            plan.cruise = cruise;
            plan.segment = Segment::SCurve(cruise);
            plan.mode = if m.kind == MoveKind::Reverse {
                MotionMode::Reverse
            } else {
                MotionMode::Trajectory
            };
            plan.t_end = t_cruise_end;
            let tau = self.term.tau_eff(v_cap);
            let (t_b, d_b, d_0) = terminus_blend(v_cap, self.j_term, tau);
            let room = d_0 + stop_distance_m(v_cap, a, self.j_max);
            if end_target && m.kind != MoveKind::Reverse && d >= room && dur > 0.0 {
                let h = first_tick_within(m.act, t_cruise_end, d_0, |t| {
                    target.diff_m(scurve::eval(&cruise, t).x) * dir.sign()
                });
                if let Some(h) = h {
                    plan.handover = Some(h);
                    let capture_s = if self.d_eps_m > 0.0 && d_b > self.d_eps_m {
                        tau * fmath::lnf(d_b / self.d_eps_m)
                    } else {
                        0.0
                    };
                    plan.t_end = h.plus(ticks_ceil(t_b + capture_s + self.t_q_ratio * tau));
                }
            }
        }
        self.seg_t0 = plan.segment.t0();
        self.plan = plan;
        capture
    }

    /// Hands the cruise over to the terminus once the directive activating at `act`
    /// reaches the handover tick. Returns (d_b, τ_eff) for the capture event.
    pub fn update(&mut self, act: Tick) -> Option<(f32, f32)> {
        let p = self.plan;
        let h = p.handover?;
        if !p.active || p.captured || act < h {
            return None;
        }
        let rs = scurve::eval(&p.cruise, h);
        let d = fmath::fmaxf(p.target.diff_m(rs.x) * p.dir.sign(), 0.0);
        let v_app = fmath::absf(rs.v_mps);
        let a_along = rs.a_mps2 * p.dir.sign();
        let tau = self.term.tau_eff(v_app);
        let seg = self
            .term
            .plan(p.target, p.dir, h, d, v_app, a_along, tau, self.j_term);
        let plan = &mut self.plan;
        plan.segment = seg;
        plan.mode = match p.dir {
            TravelDir::Close => MotionMode::Terminus,
            TravelDir::Open => MotionMode::SoftOpen,
        };
        plan.captured = true;
        plan.seg_new = true;
        plan.t_end = h.plus(ticks_ceil(segment_duration_s(&seg)));
        self.seg_t0 = h;
        Some(match seg {
            Segment::Terminus(t) => (t.d_b_m, t.tau_s),
            _ => (d, tau),
        })
    }

    /// Ends the active move.
    pub fn abort_move(&mut self) {
        self.plan.active = false;
        self.plan.handover = None;
    }

    /// True when the active move has arrived: the fast group reports `arrived` for a
    /// directive that carried the current segment, or the measured state is within the
    /// position tolerance at rest after the closed-form end tick.
    pub fn arrived(&self, s: &FastStatus, now: Tick, stall_v_mps: f32) -> bool {
        let p = &self.plan;
        if !p.active {
            return false;
        }
        let fresh = !p.seg_new && (s.directive_seq.wrapping_sub(p.seg_seq) as i32) >= 0;
        let close =
            fmath::absf(p.target.diff_m(s.x)) <= p.tol_m && fmath::absf(s.v_mps) <= stall_v_mps;
        (s.arrived && fresh && close) || (close && now >= p.t_end)
    }

    /// True when the active move is overdue: past its closed-form end by more than
    /// max(1 s, half its planned duration).
    pub fn overdue(&self, now: Tick) -> bool {
        let p = &self.plan;
        if !p.active {
            return false;
        }
        let span = p.t_end.since(p.t_start);
        let grace = core::cmp::max(fabrail_types::BASE_TICK_HZ as u64, span / 2);
        now.since(p.t_end) > grace
    }

    /// The hold segment for a Hold/Freeze episode (anchored at the stopping point).
    fn hold_segment(&mut self, i: &ModeInputs, freeze: bool) -> Segment {
        let a = if freeze {
            self.a_brk_ratio * self.a_max
        } else if i.sensed {
            self.a_max_sensed
        } else {
            self.a_max
        };
        let j = self.j_max;
        let anchor_now = || {
            let v = i.fast.v_mps;
            let d = stop_distance_m(v, a, j);
            i.fast.x.offset_m(if v < 0.0 { -d } else { d })
        };
        let escalate = freeze && !self.hold_is_freeze;
        let anchor = match self.hold_anchor {
            Some(x) if !escalate => x,
            Some(x) => {
                // Escalation Hold → Freeze: the nearer of the two stopping points.
                let n = anchor_now();
                if n.diff_nm(i.fast.x).unsigned_abs() < x.diff_nm(i.fast.x).unsigned_abs() {
                    n
                } else {
                    x
                }
            }
            None => anchor_now(),
        };
        if self.hold_anchor != Some(anchor) {
            self.seg_t0 = i.act;
        }
        self.hold_anchor = Some(anchor);
        self.hold_is_freeze = freeze;
        Segment::SCurve(SCurveSeg {
            anchor,
            t0: self.seg_t0,
            dir: TravelDir::Open,
            dist_m: 0.0,
            v_mps: 0.0,
            a_mps2: a,
            j_mps3: self.j_max,
        })
    }

    /// The live limits for this release (module docs; SAF-001…004, plan §4b).
    pub fn limits(&self, i: &ModeInputs) -> LiveLimits {
        let mut x_obs_open = self.x_open;
        let mut x_obs_close = self.x_closed;
        let unattended = i.unattended || (self.plan.active && self.plan.unattended);
        if matches!(i.motion, MotionCmd::Move) && self.plan.active {
            // Terminus + overtravel: the exponential tail is never clipped (plan §4b).
            let x_obs = self
                .plan
                .target
                .offset_m(self.plan.dir.sign() * self.overtravel_m);
            match self.plan.dir {
                TravelDir::Open => x_obs_open = x_obs,
                TravelDir::Close => x_obs_close = x_obs,
            }
        }
        let resting = matches!(i.lev_cmd, LevDirective::Off);
        let blocked = i.hold_to_run_required && !i.hold_to_run;
        let (v_open, v_close, a_max) = self.caps(i.sensed, unattended);
        let (v_open, v_close) = if resting || blocked || i.motion == MotionCmd::Off {
            (0.0, 0.0)
        } else {
            (v_open, v_close)
        };
        LiveLimits {
            v_open_mps: v_open,
            v_close_mps: v_close,
            a_max_mps2: a_max,
            f_drive_n: self.f_drive_n,
            f_static_n: if i.sensed {
                self.f_static_s
            } else {
                self.f_static_u
            },
            f_close_last50_n: self.f_close_last50_n,
            x_obs_open,
            x_obs_close,
            hold_to_run: i.hold_to_run_required,
            unattended,
        }
    }

    /// The directive for this release (`seq` left 0).
    pub fn directive(&mut self, i: &ModeInputs) -> Directive {
        let mut used_hold = false;
        let (mode, segment) = match i.motion {
            MotionCmd::Off => (MotionMode::Off, None),
            MotionCmd::Admittance => (MotionMode::Admittance, None),
            MotionCmd::Move | MotionCmd::Reverse if self.plan.active => {
                (self.plan.mode, Some(self.plan.segment))
            }
            m => {
                // Hold-type modes (and a move without a plan) keep one anchor per episode.
                used_hold = true;
                let mode = match m {
                    MotionCmd::Freeze => MotionMode::Freeze,
                    MotionCmd::Isolate => MotionMode::Isolate,
                    MotionCmd::Damp => MotionMode::Damp,
                    MotionCmd::InertialSmooth => MotionMode::InertialSmooth,
                    _ => MotionMode::Hold,
                };
                (mode, Some(self.hold_segment(i, m == MotionCmd::Freeze)))
            }
        };
        if !used_hold {
            self.hold_anchor = None;
            self.hold_is_freeze = false;
        }
        let segment = segment.unwrap_or(Segment::SCurve(SCurveSeg {
            anchor: i.fast.x,
            t0: i.act,
            ..SCurveSeg::default()
        }));
        let m_hat = i.fast.m_hat_kg;
        let m_ref = if m_hat.is_finite() && m_hat > 0.0 {
            m_hat
        } else {
            self.m_rated_kg
        };
        let haptic = HapticProfile {
            m_v_kg: if self.mass_ratio > 0.0 {
                m_ref / self.mass_ratio
            } else {
                m_ref
            },
            b_v_n_s_per_m: self.b_v,
            k_v_n_per_m: 0.0,
            x0: i.fast.x,
            n_detents: self.n_detents,
            detents: self.detents,
            detent_f_n: self.detent_f_n,
            detent_width_m: self.detent_width_m,
            soft_close: true,
            passivity: true,
            intent_enable: i.intent_enable && i.seis == SeismicState::Normal,
            push_to_open: i.push_to_open && i.seis == SeismicState::Normal,
        };
        let (_, _, a_max) = self.caps(i.sensed, false);
        let terminus = TerminusCfg {
            tau_s: self.term.tau_s,
            d_eps_m: self.d_eps_m,
            j_mps3: self.j_term,
            t_q_ratio: self.t_q_ratio,
            overtravel_m: self.overtravel_m,
            a_brk_replan_mps2: self.replan_brk_ratio * a_max,
            hand_onset_s: self.hand_onset_s,
            hand_yields: i.seis == SeismicState::Normal,
            seat_on_arrival: self.plan.active && self.plan.seat_on_arrival,
            freeze_integrator: true,
        };
        let inertial = if i.motion == MotionCmd::InertialSmooth && self.inertial.enable {
            self.inertial
        } else {
            InertialCfg::default()
        };
        let isolate = if matches!(i.motion, MotionCmd::Isolate | MotionCmd::Damp)
            && (self.isolation_enable || self.amd_enable)
        {
            IsolateCfg {
                center: self.hold_anchor.unwrap_or(i.fast.x),
                ..self.isolate
            }
        } else {
            IsolateCfg::default()
        };
        Directive {
            seq: 0,
            lev: i.lev_cmd,
            mode,
            segment,
            terminus,
            inertial,
            haptic,
            limits: self.limits(i),
            isolate,
            seis: i.seis,
            floor_v_mps: i.floor_v_mps,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    fn planner(p: &SystemParams) -> ModePlanner {
        match ModePlanner::new(p) {
            Ok(m) => m,
            Err(e) => panic!("planner: {e}"),
        }
    }

    /// Terminus blend in f32 against the plan §4b table (DRAWER τ 0.15 s, v 0.265,
    /// j 20: d_0 49.8 mm, d_b 32.1 mm). Tolerance (A) 0.1 mm, source plan §4b.
    #[test]
    fn blend_matches_table() {
        let (_, d_b, d_0) = terminus_blend(0.265, 20.0, 0.15);
        assert!(
            (d_0 - 0.0498).abs() < 1e-4,
            "(A) d_0 49.8 mm ±0.1 mm, got {d_0}"
        );
        assert!(
            (d_b - 0.0321).abs() < 1e-4,
            "(A) d_b 32.1 mm ±0.1 mm, got {d_b}"
        );
        // d_0 exceeds the S-curve braking distance in every class, so the handover
        // happens at cruise (module docs).
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let m = planner(&p);
            for v in [m.v_open_u, m.v_close_u] {
                let (_, _, d0) = terminus_blend(v, m.j_term, m.term.tau_s);
                let ds = stop_distance_m(v, m.a_max, m.j_max);
                assert!(d0 > ds, "{:?}: d_0 {d0} > d_stop {ds}", p.design.class);
            }
        }
    }

    /// d_stop in f32 equals the PRP-005 anchors 33.43 / 36.31 / 8.86 / 4.91 mm.
    /// Tolerance (A) 0.01 mm, source core design §E.5.
    #[test]
    fn stop_distance_anchors() {
        for (v, a, j, want) in [
            (0.2654f32, 1.5f32, 20.0f32, 33.43e-3f32),
            (0.1838, 0.75, 5.0, 36.31e-3),
            (0.0446, 0.15, 1.5, 8.86e-3),
            (0.025, 0.08, 1.0, 4.91e-3),
        ] {
            let d = stop_distance_m(v, a, j);
            assert!((d - want).abs() < 1e-5, "(A) {want} ±0.01 mm, got {d}");
        }
    }

    /// Bisection finds the first tick within d_0 of a monotone distance-to-go.
    #[test]
    fn handover_bisection() {
        // d(t) = 1 − t/1000 m: first ≤ 0.25 at t = 750.
        let f = |t: Tick| 1.0 - t.0 as f32 / 1000.0;
        assert_eq!(
            first_tick_within(Tick(0), Tick(1000), 0.25, f),
            Some(Tick(750))
        );
        assert_eq!(first_tick_within(Tick(0), Tick(500), 0.25, f), None);
        assert_eq!(
            first_tick_within(Tick(800), Tick(900), 0.25, f),
            Some(Tick(800))
        );
    }

    /// Unsensed and sensed caps, the unattended clamp, and zero caps when hold-to-run
    /// is required but not actuated (SAF-002/003; plan §4b).
    #[test]
    fn live_limits_gates() {
        let p = presets::door();
        let mut m = planner(&p);
        let base = ModeInputs {
            lev: LevState::LevitatedIdle,
            lev_cmd: LevDirective::Levitate,
            motion: MotionCmd::Admittance,
            ..ModeInputs::default()
        };
        let d = m.directive(&base);
        assert!(
            (d.limits.v_open_mps - 0.1838).abs() < 1e-3,
            "DOOR open 0.1838"
        );
        assert!(
            (d.limits.v_close_mps - 0.152).abs() < 1e-6,
            "DOOR close 0.152"
        );
        assert_eq!(d.limits.f_static_n, 67.0, "67 N unsensed (F4)");
        assert_eq!(d.mode, MotionMode::Admittance);
        let d = m.directive(&ModeInputs {
            sensed: true,
            ..base
        });
        assert!((d.limits.v_open_mps - 0.70).abs() < 1e-6, "sensed cap");
        assert_eq!(d.limits.f_static_n, 150.0);
        let d = m.directive(&ModeInputs {
            unattended: true,
            ..base
        });
        assert!(d.limits.unattended && d.limits.v_close_mps <= m.v_unattended + 1e-6);
        let d = m.directive(&ModeInputs {
            hold_to_run_required: true,
            ..base
        });
        assert_eq!(
            d.limits.v_open_mps, 0.0,
            "hold-to-run not actuated: no motion"
        );
        assert!(d.limits.hold_to_run);
        let d = m.directive(&ModeInputs {
            hold_to_run_required: true,
            hold_to_run: true,
            ..base
        });
        assert!(d.limits.v_open_mps > 0.0);
        let d = m.directive(&ModeInputs {
            lev_cmd: LevDirective::Off,
            motion: MotionCmd::Off,
            ..base
        });
        assert_eq!(d.limits.v_open_mps, 0.0, "rest: no motion");
        assert!(!d.inertial.enable && d.isolate == IsolateCfg::default());
        assert!(d.haptic.passivity && d.haptic.m_v_kg > 0.0);
    }

    /// A move toward a terminus sets x_obs = target + overtravel in the travel
    /// direction (plan §4b) and flags seat-on-arrival for a DRAWER close.
    #[test]
    fn terminus_observation_point() {
        let p = presets::drawer();
        let mut m = planner(&p);
        let x0 = m.x_open;
        m.start_move(&MoveStart {
            kind: MoveKind::Command,
            target: m.x_closed,
            v_cap_mps: m.v_close_u,
            a_mps2: m.a_max,
            act: Tick(20),
            x: x0,
            ..MoveStart::default()
        });
        assert!(m.plan.active && m.plan.seat_on_arrival);
        assert_eq!(m.plan.dir, TravelDir::Close);
        let d = m.directive(&ModeInputs {
            lev_cmd: LevDirective::Levitate,
            motion: MotionCmd::Move,
            act: Tick(20),
            fast: FastStatus {
                x: x0,
                ..FastStatus::default()
            },
            ..ModeInputs::default()
        });
        assert_eq!(
            d.limits.x_obs_close,
            m.x_closed.offset_m(-m.overtravel_m),
            "x_obs = terminus + overtravel"
        );
        assert!(d.terminus.seat_on_arrival && d.terminus.freeze_integrator);
        assert!(matches!(
            d.mode,
            MotionMode::Trajectory | MotionMode::Terminus
        ));
    }

    /// A hold episode keeps one anchor at the stopping point ahead of a moving carriage.
    #[test]
    fn hold_anchor_is_stopping_point() {
        let p = presets::door();
        let mut m = planner(&p);
        let i = ModeInputs {
            lev_cmd: LevDirective::Levitate,
            motion: MotionCmd::Hold,
            act: Tick(100),
            fast: FastStatus {
                x: PosNm(0),
                v_mps: 0.1838,
                ..FastStatus::default()
            },
            ..ModeInputs::default()
        };
        let d1 = m.directive(&i);
        let want = PosNm(0).offset_m(stop_distance_m(0.1838, m.a_max, m.j_max));
        assert_eq!(d1.segment.end(), want);
        let d2 = m.directive(&ModeInputs {
            fast: FastStatus {
                x: PosNm(5_000_000),
                v_mps: 0.05,
                ..FastStatus::default()
            },
            ..i
        });
        assert_eq!(d2.segment.end(), want, "anchor fixed for the episode");
        assert_eq!(d2.mode, MotionMode::Hold);
    }
}
