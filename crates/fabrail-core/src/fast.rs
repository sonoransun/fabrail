//! FastLoop: fast-group composition (frontend, levitation, motion/haptics, governor, clamp) at 10/5 kHz. Owner: A2.
//!
//! Frozen API (W0, core design §B.4): [`FastLoop::new`], [`FastLoop::step`] →
//! (`FastOutputs`, `FastStatus`), [`FastLoop::reset`]. The fast group is a pure function
//! of (FastRaw, FastIn): fabrail-rt converts the latched raw sample with
//! `frontend::Frontend::step` (merging its faults into the status), calls
//! `FastLoop::step` with the active `Directive`, then runs the monitor's `step_fast`
//! and the arbitration on the outputs. No HAL access here.
//!
//! Composition per fast release (PHYSICS §15–§17; core design §C.4):
//!
//! 1. `levitation::Levitation` → coil current references and m̂ (A1);
//! 2. kinematics: a third-order tracking observer on the integer encoder position gives
//!    v̂ and â (poles (s + ω_t)³, ω_t = 2π·max(5·f_v, 2·f_KO); design choice: ≥ 5× the
//!    velocity loop, so its lag at ω_v is < 1°);
//! 3. `haptics::observer` → F̂_h from v̂ and the modelled thrust of the measured i_q;
//!    `haptics::intent` on the raw F̂ (push-to-open at the DRAWER seat) feeds the lean
//!    bias estimator; a hand is |F̂| > F_th for hand_onset/2 (5 ms; 10 ms WALL);
//! 4. the reference, from the directive's mode: planned segment (`motion::eval_segment`:
//!    S-curve, log-damped terminus, quintic), admittance (`haptics::admittance` with
//!    detents, soft-close and the passivity controller in F_eff), hold, freeze (brake at
//!    a_brk, then hold), or a research-tier hook;
//! 5. `motion::governor` v_lim(x) on the cascade's velocity command and on the
//!    admittance velocity (dissipative only); `motion::cascade` → F*;
//!    `motion::clamp` (directional; bypassed in ISOLATE, where SAF-020's envelope
//!    replaces it; the last 50 mm closing zone is measured from
//!    `LiveLimits::x_obs_close`, the closing end the supervisor sets) → F;
//!    i_q* = F/K̂_f (ripple table);
//! 6. `haptics::obstruction` while moving under power (or unattended): STOP (brake at
//!    a_brk), then REVERSE `obstruction.reverse_m` (DOOR/WALL) or release to 0 N (LSM
//!    STO; DRAWER/HEAVY). The response is latched until the supervisor changes the
//!    command (mode or segment); `FastStatus::obstruction` reports it.
//!
//! Terminus hand-over (plan §4b): a hand during a Terminus/SoftOpen capture switches to
//! plain admittance (K_v = 0, passivity on) when `TerminusCfg::hand_yields`, else holds
//! (outside NORMAL). On release the terminus is re-planned from the measured (d, v, a)
//! with `TerminusPlanner::plan` at τ_eff. The velocity integrator is frozen (bumpless)
//! while the terminus distance-to-go exceeds d_ε.
//!
//! Wave-1.5 integration (closed loop on Carriage6):
//! * the motion loop controls the CoM: the encoder reads the carriage at the rail, a
//!   lever h = −c_z from the CoM, so x_c = x_enc − h·θ̂_y (levitation pitch estimate),
//!   and the LSM thrust's pitch torque h·F is cancelled by the levitation's
//!   thrust-moment feed-forward (`Levitation::thrust_ff`); without both a top-hung DOOR
//!   couples travel and pitch through the hand-force observer and diverges;
//! * a directive re-anchoring the placeholder segment of a hold or admittance mode at
//!   every supervisor release does not re-enter the mode (`command_segment`);
//! * the hand threshold grows with the commanded thrust (`HAND_TH_THRUST_FRAC`);
//! * the terminus hand re-plan uses E1's `TerminusPlanner::replan` (infeasible → core
//!   SoftClose toward the target; an approach from rest launches, then switches to its
//!   terminus), the integrator freeze follows `terminus::integrator_frozen`, and a
//!   terminus has arrived only when its reference has (`terminus::done`);
//! * inertial smoothing (research tier) exits through `InertialSmoother::exit_plan`.
//!
//! Still open: MotionMode::Damp holds on a carriage (AMD needs the structure state).
//!
//! Every position is used as an integer `PosNm` difference and every time as a tick
//! difference, so the outputs are bit-identical under x → x + L (L a multiple of the
//! electrical period 2τ_p while the ripple table is on) and tick → tick + offset
//! (`tests/invariance.rs`, FAB-PRP-008).

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{
    CoilCmd, LsmCmd, LsmMode, MotionMode, PosAccum, PosNm, Segment, SeismicState, Tick, TravelDir,
    fmath,
};

use crate::haptics::admittance::Admittance;
use crate::haptics::effects;
use crate::haptics::intent::IntentDetector;
use crate::haptics::observer::MomentumObserver;
use crate::haptics::obstruction::{ObstructionDetector, ObstructionOut};
use crate::haptics::passivity::PassivityController;
use crate::levitation::Levitation;
use crate::motion::cascade::Cascade;
use crate::motion::clamp::DirectionalClamp;
use crate::motion::governor::Governor;
use crate::motion::inertial::InertialSmoother;
use crate::motion::terminus::{self, PlanKind, TerminusPlanner};
use crate::motion::{self, RefState, scurve};
use crate::msg::{
    Directive, FastOutputs, FastStatus, HapticProfile, IntentFlag, LevDirective, Measurements,
    SeatStatus,
};
use crate::seismic::isolate::Isolator;

/// Encoder jump that re-initialises the tracking observer (homing, glitch), m
/// (design choice).
const TRACK_JUMP_M: f32 = 0.01;

/// Bound on the pitch used for the CoM correction of the encoder position, rad (design
/// choice: 5 mrad is several times the pitch at the upper stop of every class).
const PITCH_CORR_MAX_RAD: f32 = 5.0e-3;

/// Hand-detection threshold allowance per newton of commanded thrust (W1.5; design
/// choice). Under powered motion F̂ carries the unmodelled forces (cabinet air, rail-end
/// reluctance, thrust and mass model errors), measured at up to ≈ 15 % of the thrust in
/// a Carriage6 DRAWER terminus capture; the hand threshold is F_th + 0.25·|F_cmd| so a
/// braking terminus is not mistaken for a hand (the obstruction detector already
/// scales its threshold the same way with the floor term).
const HAND_TH_THRUST_FRAC: f32 = 0.25;

/// Third-order position tracking observer: x̂ in a `PosAccum`, v̂, â; error poles
/// (s + ω)³ (l = 3ω, 3ω², ω³), forward Euler at the fast period.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Tracker {
    x: PosAccum,
    v: f32,
    a: f32,
    l: [f32; 3],
    dt: f32,
    init: bool,
}

impl Tracker {
    fn new(omega: f32, dt: f32) -> Tracker {
        Tracker {
            l: [3.0 * omega, 3.0 * omega * omega, omega * omega * omega],
            dt,
            ..Tracker::default()
        }
    }

    /// One release on the measured position; returns (v̂, â).
    fn step(&mut self, x: PosNm) -> (f32, f32) {
        // Error as an integer difference first (PHYSICS §24), then the sub-nm fraction.
        let e = (x.diff_nm(self.x.pos()) as f32 - self.x.frac_nm) * 1.0e-9;
        let ae = fmath::absf(e);
        if !self.init || ae.is_nan() || ae > TRACK_JUMP_M {
            self.x.reset(x);
            self.v = 0.0;
            self.a = 0.0;
            self.init = true;
            return (0.0, 0.0);
        }
        let [l1, l2, l3] = self.l;
        let (v, a) = (self.v, self.a);
        self.x.advance(v + l1 * e, self.dt);
        self.v = v + (a + l2 * e) * self.dt;
        self.a = a + l3 * e * self.dt;
        (self.v, self.a)
    }

    fn reset(&mut self) {
        self.init = false;
        self.v = 0.0;
        self.a = 0.0;
    }
}

/// Constant-deceleration stop from a latched state, in closed form (STOP, Freeze and a
/// Hold entered while moving): x(τ) = x0 + v0·τ − s·a·τ²/2 until v = 0.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Brake {
    x0: PosNm,
    v0: f32,
    a: f32,
    t0: Tick,
}

impl Brake {
    fn eval(&self, t: Tick) -> RefState {
        let tau = t.since_s(self.t0);
        let s = if self.v0 >= 0.0 { 1.0 } else { -1.0 };
        let t_stop = if self.a > 0.0 {
            fmath::absf(self.v0) / self.a
        } else {
            0.0
        };
        if tau >= t_stop {
            return RefState {
                x: self.x0.offset_m(0.5 * self.v0 * t_stop),
                v_mps: 0.0,
                a_mps2: 0.0,
                done: true,
            };
        }
        RefState {
            x: self
                .x0
                .offset_m(self.v0 * tau - 0.5 * s * self.a * tau * tau),
            v_mps: self.v0 - s * self.a * tau,
            a_mps2: -s * self.a,
            done: false,
        }
    }
}

/// Where the reference comes from.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum Src {
    /// LSM off (STO); motion states follow the measurement.
    #[default]
    Idle,
    /// Hold at a latched position.
    Hold(PosNm),
    /// Decelerate at a_brk, then hold where it stops.
    Brake(Brake),
    /// The directive's planned segment.
    Directive,
    /// A segment planned in the fast group (terminus re-plan, obstruction reversal).
    Local(Segment),
    /// Admittance (hand-driven), `Directive::haptic` profile.
    Admit,
    /// Terminus hand take-over: admittance with K_v = 0 and passivity on.
    HandAdmit,
    /// Infeasible terminus re-plan after a hand release: core soft-close toward the
    /// directive's target (plan §4b).
    SoftCloseFallback,
    /// Obstruction release to 0 N (LSM STO).
    Released,
    /// Research tier: inertial smoothing (skyhook about the standoff; exits to the
    /// log-damped seat close).
    Inertial,
    /// Research tier: contactless isolation (A3's isolator law; SAF-020's envelope
    /// replaces the clamp).
    Isolate,
}

/// Obstruction response phase (PHYSICS §17; SAF-006).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ObstPhase {
    /// Braking to rest.
    #[default]
    Stop,
    /// Reversing `reverse_m` (DOOR/WALL).
    Reverse,
    /// Released to 0 N (DRAWER/HEAVY).
    Released,
    /// Holding after the reversal.
    Hold,
}

/// Latched obstruction response.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Obst {
    /// Direction of motion at detection (±1).
    sign: f32,
    phase: ObstPhase,
}

/// Motion modes whose reference is the directive's planned segment.
fn segment_mode(m: MotionMode) -> bool {
    matches!(
        m,
        MotionMode::Trajectory
            | MotionMode::Center
            | MotionMode::Reverse
            | MotionMode::Terminus
            | MotionMode::SoftOpen
    )
}

/// The part of the directive's segment that defines the command for its mode (W1.5):
/// the whole segment for planned modes, the target alone for SoftClose (its capture
/// point), nothing otherwise. The supervisor may re-anchor the placeholder segment of a
/// hold or admittance directive at every release; that must not re-enter the mode
/// (which would reset the admittance and the obstruction latch at 1 kHz).
fn command_segment(d: &Directive) -> Segment {
    match d.mode {
        m if segment_mode(m) => d.segment,
        MotionMode::SoftClose => Segment::SCurve(fabrail_types::SCurveSeg {
            anchor: d.segment.end(),
            ..fabrail_types::SCurveSeg::default()
        }),
        _ => Segment::default(),
    }
}

/// The fast-group controller.
#[derive(Clone, Debug, Default)]
pub struct FastLoop {
    /// Fast release sequence number (the core heartbeat).
    pub seq: u32,
    /// Fast period, s.
    pub dt_s: f32,
    /// Levitation (A1).
    pub lev: Levitation,
    /// Position/velocity cascade.
    pub cascade: Cascade,
    /// Energy/momentum/braking governor.
    pub governor: Governor,
    /// Directional force clamp.
    pub clamp: DirectionalClamp,
    /// Admittance.
    pub adm: Admittance,
    /// Momentum observer (F̂_h).
    pub obs: MomentumObserver,
    /// Passivity observer/controller.
    pub pc: PassivityController,
    /// Intent detector.
    pub intent: IntentDetector,
    /// Obstruction detector.
    pub obst_det: ObstructionDetector,
    /// Terminus re-planner (E1).
    pub terminus: TerminusPlanner,
    /// Inertial smoother (E1, research tier).
    pub inertial: InertialSmoother,
    /// Isolator (A3, research tier).
    pub isolator: Isolator,
    track: Tracker,
    /// Tracking observer on the raw encoder point (rail reference point, pitch rocking
    /// included): the speed the independent monitor measures (D4 governor damping).
    track_enc: Tracker,
    src: Src,
    sig: Option<(MotionMode, Segment)>,
    obst: Option<Obst>,
    last_ref: RefState,
    intent_ref: PosNm,
    hand: bool,
    hand_ticks: u32,
    m_nominal_kg: f32,
    stall_v_mps: f32,
    pos_tol_closed_m: f32,
    pos_tol_travel_m: f32,
    last_zone_m: f32,
    a_max_mps2: f32,
    j_max_mps3: f32,
    reverse_m: f32,
    release_to_zero: bool,
    hand_th_n: f32,
    hand_on_ticks: u32,
    hand_off_ticks: u32,
    sc_omega_rad_s: f32,
    sc_capture_m: f32,
    /// Travel speed estimate of the last release, m/s.
    last_v_mps: f32,
    /// Thrust commanded at the last release, N.
    last_f_n: f32,
    /// Largest hand force against the terminus direction during a hand take-over, N.
    hand_oppose_max_n: f32,
    /// Releases the hand has pushed against an unattended capture harder than the
    /// obstruction threshold (D6 pinch persistence).
    pinch_ticks: u32,
    /// Terminus to switch to after a re-planned approach launch (E1 `PlanKind::Approach`)
    /// and the first tick at which the switch may happen.
    pending_terminus: Option<(fabrail_types::TerminusSeg, Tick)>,
}

impl FastLoop {
    /// Builds every fast-group block from the parameters (f64 → f32 in the blocks'
    /// `new`). Fails only where a block fails (levitation design).
    pub fn new(p: &SystemParams) -> Result<FastLoop, ParamError> {
        let dt64 = fabrail_params::derive::fast_period_s(p);
        let dt = dt64 as f32;
        let mo = &p.control.motion;
        let hp = &p.control.haptic;
        let two_pi = 2.0 * core::f64::consts::PI;
        // Tracking-observer bandwidth ω_t = 2π·max(5·f_v, 2·f_KO) (module docs).
        let omega_t = (two_pi * fmath::fmax(5.0 * mo.omega_v_hz, 2.0 * hp.k_o_hz)) as f32;
        let ticks = |t_s: f64| -> u32 {
            if dt64 > 0.0 && t_s > 0.0 {
                let n = fmath::round(t_s / dt64);
                if n >= f64::from(u32::MAX) {
                    u32::MAX
                } else {
                    (n as u32).max(1)
                }
            } else {
                1
            }
        };
        let m_nominal = p.design.carriage.m_star_kg as f32;
        Ok(FastLoop {
            seq: 0,
            dt_s: dt,
            lev: Levitation::new(p)?,
            cascade: Cascade::new(p)?,
            governor: Governor::new(p)?,
            clamp: DirectionalClamp::new(p)?,
            adm: Admittance::new(p)?,
            obs: MomentumObserver::new(p)?,
            pc: PassivityController::new(p)?,
            intent: IntentDetector::new(p)?,
            obst_det: ObstructionDetector::new(p)?,
            terminus: TerminusPlanner::new(p)?,
            inertial: InertialSmoother::new(p)?,
            isolator: Isolator::new(p)?,
            track: Tracker::new(omega_t, dt),
            track_enc: Tracker::new(omega_t, dt),
            src: Src::Idle,
            sig: None,
            obst: None,
            last_ref: RefState::default(),
            intent_ref: PosNm::ZERO,
            hand: false,
            hand_ticks: 0,
            m_nominal_kg: if m_nominal > 0.0 { m_nominal } else { 1.0 },
            stall_v_mps: p.control.supervisor.stall_v_mps as f32,
            pos_tol_closed_m: mo.pos_tol_closed_m as f32,
            pos_tol_travel_m: mo.pos_tol_travel_m as f32,
            last_zone_m: p.safety.last_zone_m as f32,
            a_max_mps2: mo.a_max_mps2 as f32,
            j_max_mps3: mo.j_max_mps3 as f32,
            reverse_m: hp.obstruction.reverse_m as f32,
            release_to_zero: hp.obstruction.release_to_zero,
            // Hand = |F̂| above the intent force threshold (m̂ errors during powered
            // motion stay below it; design choice), for half the onset budget.
            hand_th_n: p.control.intent.f_th_n as f32,
            hand_on_ticks: ticks(0.5 * hp.hand_onset_s),
            hand_off_ticks: ticks(hp.hand_onset_s),
            sc_omega_rad_s: (two_pi * hp.soft_close_omega_hz) as f32,
            sc_capture_m: hp.soft_close_capture_m as f32,
            last_v_mps: 0.0,
            last_f_n: 0.0,
            pending_terminus: None,
            hand_oppose_max_n: 0.0,
            pinch_ticks: 0,
        })
    }

    /// Live acceleration limit (falls back to the preset a_max), m/s².
    fn a_max(&self, d: &Directive) -> f32 {
        if d.limits.a_max_mps2 > 0.0 {
            d.limits.a_max_mps2
        } else {
            self.a_max_mps2
        }
    }

    /// Brake from the measured state, or hold where the carriage stands (at the last
    /// reference if it was at rest nearby, so a finished move holds its target).
    fn stop_or_hold(&self, d: &Directive, x: PosNm, v: f32, t: Tick) -> Src {
        let a_brk = self.governor.a_brk_live_mps2(&d.limits);
        if fmath::absf(v) > self.stall_v_mps && a_brk > 0.0 {
            return Src::Brake(Brake {
                x0: x,
                v0: v,
                a: a_brk,
                t0: t,
            });
        }
        // The last reference exists only after a first command (`sig`): before it, the
        // default reference at PosNm(0) must not be held (offset invariance, PRP-008).
        let r = self.last_ref;
        let near = fmath::absf(r.x.diff_m(x)) <= self.pos_tol_travel_m;
        if self.sig.is_some() && fmath::absf(r.v_mps) <= self.stall_v_mps && near {
            Src::Hold(r.x)
        } else {
            Src::Hold(x)
        }
    }

    /// Sets up the reference source for a new command (mode or segment changed).
    fn enter(&mut self, d: &Directive, m: &Measurements, v: f32) {
        self.obst = None;
        self.pending_terminus = None;
        self.obst_det.reset();
        let t = m.tick;
        self.src = match d.mode {
            MotionMode::Off => Src::Idle,
            MotionMode::Hold | MotionMode::Damp => {
                // MotionMode::Damp (AMD, research tier; `seismic::amd`) needs the
                // structure state, which the fast group does not have (the Structure
                // tier runs AMD in the plant); on a carriage it holds. Documented gap.
                self.stop_or_hold(d, m.x, v, t)
            }
            MotionMode::Freeze => {
                let a_brk = self.governor.a_brk_live_mps2(&d.limits);
                Src::Brake(Brake {
                    x0: m.x,
                    v0: v,
                    a: a_brk,
                    t0: t,
                })
            }
            MotionMode::Admittance | MotionMode::SoftClose => {
                self.adm.reset(m.x, v);
                self.pc.reset();
                Src::Admit
            }
            MotionMode::InertialSmooth => {
                if d.inertial.enable && self.inertial.enable {
                    self.inertial.reset();
                    Src::Inertial
                } else {
                    self.stop_or_hold(d, m.x, v, t)
                }
            }
            MotionMode::Isolate => {
                self.isolator.reset();
                Src::Isolate
            }
            mode if segment_mode(mode) => Src::Directive,
            _ => self.stop_or_hold(d, m.x, v, t),
        };
    }

    /// Debounced hand detection on the corrected F̂ (threshold F_th plus
    /// `HAND_TH_THRUST_FRAC` of the last commanded thrust).
    fn hand_step(&mut self, f_hat: f32) -> bool {
        let a = fmath::absf(f_hat);
        let th = self.hand_th_n + HAND_TH_THRUST_FRAC * fmath::absf(self.last_f_n);
        if self.hand {
            self.hand_ticks = if a < 0.5 * th {
                self.hand_ticks.saturating_add(1)
            } else {
                0
            };
            if self.hand_ticks >= self.hand_off_ticks {
                self.hand = false;
                self.hand_ticks = 0;
            }
        } else {
            self.hand_ticks = if a > th || !a.is_finite() {
                self.hand_ticks.saturating_add(1)
            } else {
                0
            };
            if self.hand_ticks >= self.hand_on_ticks {
                self.hand = true;
                self.hand_ticks = 0;
            }
        }
        self.hand
    }

    /// Terminus hand-over and re-plan (plan §4b) while the directive's segment is a
    /// terminus (Terminus / SoftOpen modes).
    fn terminus_hand(&mut self, d: &Directive, m: &Measurements, v: f32, a: f32, f_hat: f32) {
        if !matches!(d.mode, MotionMode::Terminus | MotionMode::SoftOpen) || self.obst.is_some() {
            self.pinch_ticks = 0;
            return;
        }
        // Unattended close (choreographed or alert-initiated; Wave 2 lead decision D6): a
        // hand detected against the capture (the hand detector's onset/2 persistence,
        // with the estimate opposing the closing direction beyond the hand threshold) is
        // a pinch, flagged at the yield instant — the release at which the attended
        // capture would yield — not when the hand lets go. The obstruction latches: stop,
        // then release to 0 N (DRAWER/HEAVY) or reverse, never re-close (plan §4b).
        if d.limits.unattended && self.hand {
            let against = -f_hat * d.segment.dir().sign();
            self.pinch_ticks = if against.is_finite() && against > self.hand_th_n {
                self.pinch_ticks.saturating_add(1)
            } else {
                0
            };
            if self.pinch_ticks >= 1 {
                self.pinch_ticks = 0;
                self.obst = Some(Obst {
                    sign: d.segment.dir().sign(),
                    phase: ObstPhase::Stop,
                });
                self.src = self.stop_or_hold(d, m.x, v, m.tick);
                if let Src::Hold(x) = self.src {
                    self.obstruction_advance(d, x, m.tick);
                }
                return;
            }
        } else {
            self.pinch_ticks = 0;
        }
        match self.src {
            Src::Directive | Src::Local(_) if self.hand => {
                if d.terminus.hand_yields {
                    // Plain class admittance: K_v = 0, passivity on.
                    self.adm.reset(m.x, v);
                    self.pc.reset();
                    self.src = Src::HandAdmit;
                    self.hand_oppose_max_n = 0.0;
                } else {
                    // Outside SeismicState NORMAL a hand gives Hold (clamps unchanged).
                    self.src = self.stop_or_hold(d, m.x, v, m.tick);
                }
            }
            Src::HandAdmit if self.hand => {
                // Largest force the hand applied against the terminus direction.
                let against = -f_hat * d.segment.dir().sign();
                if against.is_finite() {
                    self.hand_oppose_max_n = fmath::fmaxf(self.hand_oppose_max_n, against);
                }
            }
            Src::HandAdmit if !self.hand && self.hand_oppose_max_n > self.obst_det.th0_n => {
                // The hand held the carriage back harder than the obstruction threshold:
                // a trapped or blocking hand. Obstruction overrides the target (plan
                // §4b): stop, then release to 0 N (DRAWER/HEAVY) or reverse, latched —
                // never re-close into it (W1.5; FAB-EXA unattended-pinch "no re-close").
                self.obst = Some(Obst {
                    sign: d.segment.dir().sign(),
                    phase: ObstPhase::Stop,
                });
                self.src = self.stop_or_hold(d, m.x, v, m.tick);
                if let Src::Hold(x) = self.src {
                    self.obstruction_advance(d, x, m.tick);
                }
            }
            Src::HandAdmit if !self.hand => {
                // Re-plan from the measured (d, v, a) at τ_eff with E1's planner
                // (a_brk = 2·a_max inside d_0; plan §4b). An infeasible plan falls back
                // to core SoftClose toward the target; an approach from rest runs its
                // launch S-curve, then switches to the terminus it carries. The DRAWER
                // seat step on arrival is the supervisor's (`seat_on_arrival`).
                let seg = d.segment;
                let dir = seg.dir();
                let s = dir.sign();
                let target = seg.end();
                let dist = s * target.diff_m(m.x);
                let plan = self
                    .terminus
                    .replan(target, dir, m.tick, dist, s * v, s * a);
                match plan.kind {
                    PlanKind::Infeasible => {
                        self.adm.reset(m.x, v);
                        self.pc.reset();
                        self.src = Src::SoftCloseFallback;
                    }
                    PlanKind::Approach => {
                        self.pending_terminus = plan.next.map(|n| (n, plan.switch_from));
                        self.src = Src::Local(plan.seg);
                    }
                    _ => self.src = Src::Local(plan.seg),
                }
            }
            _ => {}
        }
    }

    /// Advances the obstruction response after the brake has stopped.
    fn obstruction_advance(&mut self, d: &Directive, stop_at: PosNm, t: Tick) {
        let Some(mut ob) = self.obst else {
            return;
        };
        if ob.phase != ObstPhase::Stop {
            return;
        }
        if self.release_to_zero {
            ob.phase = ObstPhase::Released;
            self.src = Src::Released;
        } else if self.reverse_m > 0.0 {
            // Reverse away from the obstacle at the live limits of that direction.
            let (dir_v, v) = if ob.sign > 0.0 {
                (TravelDir::Close, d.limits.v_close_mps)
            } else {
                (TravelDir::Open, d.limits.v_open_mps)
            };
            let target = stop_at.offset_m(dir_v.sign() * self.reverse_m);
            let seg = scurve::plan(stop_at, t, target, v, self.a_max(d), self.j_max_mps3);
            ob.phase = ObstPhase::Reverse;
            self.src = Src::Local(Segment::SCurve(seg));
        } else {
            ob.phase = ObstPhase::Hold;
            self.src = Src::Hold(stop_at);
        }
        self.obst = Some(ob);
    }

    /// Admittance profile for the current source and mode.
    fn profile(&self, d: &Directive) -> HapticProfile {
        let mut prof = d.haptic;
        if self.src == Src::HandAdmit {
            prof.k_v_n_per_m = 0.0;
            prof.passivity = true;
        }
        prof
    }

    /// One fast release.
    pub fn step(&mut self, m: &Measurements, d: &Directive) -> (FastOutputs, FastStatus) {
        self.seq = self.seq.wrapping_add(1);
        let t = m.tick;
        let mut sat: u8 = 0;

        // 1. Levitation (the seat approach reads the previous release's travel speed).
        self.lev.travel_v_mps = self.last_v_mps;
        let lev = self.lev.step(m, &d.lev);
        if lev.saturated {
            sat |= FastStatus::SAT_COIL;
        }
        let m_hat = if lev.m_hat_kg.is_finite() && lev.m_hat_kg > 0.0 {
            lev.m_hat_kg
        } else {
            self.m_nominal_kg
        };
        // The encoder reads the carriage at the rail reference point, a lever h above
        // the CoM (h = −c_z): x_enc = x_c + h·θ_y. The motion loop controls the CoM
        // (x_c = x_enc − h·θ̂_y from the levitation's pitch estimate), so pitch motion
        // never enters the travel loop or the hand-force observer (W1.5; PHYSICS §5).
        let theta = if lev.pitch_rad.is_finite() {
            fmath::clampf(lev.pitch_rad, -PITCH_CORR_MAX_RAD, PITCH_CORR_MAX_RAD)
        } else {
            0.0
        };
        // Encoder-point speed (what the monitor's windowed encoder velocity sees; the
        // pitch rocking h·θ̇ of a top-hung leaf included), for the D4 damping rule.
        let v_enc = if m.enc_ok {
            self.track_enc.step(m.x).0
        } else {
            self.track_enc.reset();
            0.0
        };
        let m_c = Measurements {
            x: m.x.offset_m(-self.lev.thrust_arm_m * theta),
            ..*m
        };
        let m = &m_c;

        // 2. Kinematics from the integer encoder position.
        let enc = m.enc_ok;
        let (v, a) = if enc {
            self.track.step(m.x)
        } else {
            self.track.reset();
            (0.0, 0.0)
        };
        self.last_v_mps = v;

        // 3. Hand force, intent, lean bias, hand detection.
        let f_act = self.cascade.thrust_n(m.iq_a);
        let f_hat = self.obs.step(v, f_act, m_hat);
        let f_raw = self.obs.f_raw_n;
        self.intent.seat_mode = d.haptic.push_to_open && lev.seat == SeatStatus::Seated;
        if self.intent.is_idle() {
            self.intent_ref = m.x;
        }
        let intent_on = d.haptic.intent_enable && d.seis == SeismicState::Normal && enc;
        let flag = self
            .intent
            .step(f_raw, m.x.diff_m(self.intent_ref), v, intent_on);
        self.obs.set_lean(flag == IntentFlag::Lean);
        let hand = self.hand_step(f_hat);

        // 4. Command changes, local sources, terminus hand-over.
        if !enc {
            self.sig = None;
            self.src = Src::Idle;
            self.obst = None;
        } else {
            let sig = (d.mode, command_segment(d));
            if self.sig != Some(sig) {
                self.enter(d, m, v);
                self.sig = Some(sig);
            }
            self.terminus_hand(d, m, v, a, f_hat);
        }

        // 5. Obstruction detection while moving under power (or unattended).
        let a_floor = if m.rail_ok { m.rail_mps2[0] } else { 0.0 };
        let moving = fmath::absf(v) > self.stall_v_mps || fmath::absf(self.last_ref.v_mps) > 0.0;
        let powered = match self.src {
            Src::Directive | Src::Local(_) | Src::SoftCloseFallback => true,
            Src::Admit => d.mode == MotionMode::SoftClose,
            _ => false,
        };
        let armed = enc && self.obst.is_none() && (d.limits.unattended || (powered && moving));
        let ob: ObstructionOut = if self.obst.is_some() {
            ObstructionOut {
                detected: true,
                ..self.obst_det.step(f_hat, a_floor, m_hat, false)
            }
        } else {
            self.obst_det.step(f_hat, a_floor, m_hat, armed)
        };
        if ob.detected && self.obst.is_none() && enc {
            let s = if fmath::absf(v) > self.stall_v_mps {
                v
            } else {
                self.last_ref.v_mps
            };
            self.obst = Some(Obst {
                sign: if s >= 0.0 { 1.0 } else { -1.0 },
                phase: ObstPhase::Stop,
            });
            self.src = self.stop_or_hold(d, m.x, v, t);
            if let Src::Hold(x) = self.src {
                // Already at rest against the obstacle: reverse or release now.
                self.obstruction_advance(d, x, t);
            }
        }

        // 6. Governor envelope.
        let v_open = self
            .governor
            .v_lim_mps(m.x, TravelDir::Open, m_hat, &d.limits);
        let v_close = self
            .governor
            .v_lim_mps(m.x, TravelDir::Close, m_hat, &d.limits);
        self.cascade.set_speed_limits(v_open, v_close);
        self.adm.set_speed_limits(v_open, v_close);

        // 7. Reference and force.
        let mut freeze = false;
        let mut lsm_on = true;
        let mut bypass_clamp = false;
        let mut f_direct: Option<f32> = None;
        let mut mode = d.mode;
        let r = match self.src {
            Src::Idle => {
                lsm_on = false;
                mode = MotionMode::Off;
                RefState {
                    x: m.x,
                    ..RefState::default()
                }
            }
            Src::Hold(x) => {
                mode = MotionMode::Hold;
                RefState {
                    x,
                    ..RefState::default()
                }
            }
            Src::Brake(b) => {
                let r = b.eval(t);
                mode = if d.mode == MotionMode::Freeze && self.obst.is_none() {
                    MotionMode::Freeze
                } else {
                    MotionMode::Hold
                };
                if r.done {
                    self.src = Src::Hold(r.x);
                    self.obstruction_advance(d, r.x, t);
                }
                r
            }
            Src::Directive | Src::Local(_) => {
                // A re-planned approach launch hands over to its terminus once cruising.
                if let (Src::Local(_), Some((next, from))) = (self.src, self.pending_terminus)
                    && t >= from
                {
                    self.src = Src::Local(Segment::Terminus(next));
                    self.pending_terminus = None;
                }
                let seg = match self.src {
                    Src::Local(s) => s,
                    _ => d.segment,
                };
                let r = motion::eval_segment(&seg, t);
                // Integrator frozen (bumpless) during the blend and capture, re-enabled
                // at d_ε (E1 `terminus::integrator_frozen`; FAB-HAP-015).
                freeze = d.terminus.freeze_integrator && terminus::integrator_frozen(&seg, t);
                if let Some(ob) = self.obst {
                    mode = if ob.phase == ObstPhase::Reverse {
                        MotionMode::Reverse
                    } else {
                        MotionMode::Hold
                    };
                    if r.done && ob.phase == ObstPhase::Reverse {
                        self.obst = Some(Obst {
                            phase: ObstPhase::Hold,
                            ..ob
                        });
                        self.src = Src::Hold(seg.end());
                    }
                }
                r
            }
            Src::Admit | Src::HandAdmit | Src::SoftCloseFallback => {
                let prof = self.profile(d);
                let x_ref = self.adm.state().x;
                let mut f_eff = effects::detent_force_n(x_ref, &prof);
                let soft_close = (d.mode == MotionMode::SoftClose && self.src == Src::Admit)
                    || self.src == Src::SoftCloseFallback;
                if soft_close {
                    f_eff += effects::soft_close_force_n(
                        x_ref,
                        self.adm.v_mps,
                        d.segment.end(),
                        self.sc_omega_rad_s,
                        prof.m_v_kg,
                        self.sc_capture_m,
                    );
                }
                // Passivity: storage reference of the virtual element (½M_v·v² + ½K_v·e²).
                let e = x_ref.diff_m(prof.x0);
                let w = 0.5 * prof.m_v_kg * self.adm.v_mps * self.adm.v_mps
                    + 0.5 * fmath::fmaxf(prof.k_v_n_per_m, 0.0) * e * e;
                self.pc.set_stored_energy(w);
                let pco = self.pc.step(f_hat, v, prof.passivity);
                f_eff += pco.f_pc_n;
                mode = match self.src {
                    Src::HandAdmit => MotionMode::Admittance,
                    Src::SoftCloseFallback => MotionMode::SoftClose,
                    _ => d.mode,
                };
                let r = self.adm.step(f_hat, f_eff, &prof);
                if self.adm.governed {
                    sat |= FastStatus::SAT_GOVERNOR;
                }
                r
            }
            Src::Released => {
                lsm_on = false;
                mode = MotionMode::Hold;
                RefState {
                    x: m.x,
                    ..RefState::default()
                }
            }
            Src::Inertial => {
                // Research tier (FAB-HAP-017…019; `inertial.enable = false` in every
                // preset). E1's skyhook force about the standoff replaces the cascade
                // (x_rel, v_rel from the encoder; the rail accelerometer feeds the
                // optional feed-forward). Every latched exit hands over to the
                // log-damped close to the seat at a_brk = 2·a_max (`exit_plan`; plan
                // §4b), or brakes when no log-damped plan fits.
                let f = self.inertial.step(
                    m.imu_mps2[0],
                    a_floor,
                    v,
                    m.x.diff_m(d.inertial.standoff),
                    m_hat,
                );
                f_direct = Some(f);
                if self.inertial.exit() != crate::motion::inertial::ExitReason::None {
                    let plan = self.inertial.exit_plan(
                        &self.terminus,
                        self.lev.cfg.x_closed,
                        t,
                        m.x,
                        v,
                        a,
                    );
                    self.src = match plan.kind {
                        PlanKind::Infeasible => Src::Brake(Brake {
                            x0: m.x,
                            v0: v,
                            a: self.governor.a_brk_live_mps2(&d.limits),
                            t0: t,
                        }),
                        PlanKind::Approach => {
                            self.pending_terminus = plan.next.map(|n| (n, plan.switch_from));
                            Src::Local(plan.seg)
                        }
                        _ => Src::Local(plan.seg),
                    };
                }
                RefState {
                    x: m.x,
                    v_mps: v,
                    ..RefState::default()
                }
            }
            Src::Isolate => {
                // Research tier (SEI-015, SAF-020). A3's isolator law; its F_cont
                // saturation and the monitor's IsolateLimits replace the directional
                // clamp in ISOLATE.
                let io =
                    self.isolator
                        .step(m.x.diff_m(d.isolate.center), v, m.imu_mps2[0], &d.isolate);
                if io.saturated {
                    sat |= FastStatus::SAT_STROKE;
                }
                let f_max = fmath::fmaxf(d.isolate.f_max_n, 0.0);
                f_direct = Some(fmath::clampf(io.f_n, -f_max, f_max));
                bypass_clamp = true;
                RefState {
                    x: m.x,
                    v_mps: v,
                    ..RefState::default()
                }
            }
        };

        let mut f_thrust = 0.0f32;
        let iq = if lsm_on && enc {
            let f_star = match f_direct {
                Some(f) => f,
                None => {
                    let f = self.cascade.step(&r, m.x, v, m_hat, freeze);
                    if self.cascade.governed {
                        sat |= FastStatus::SAT_GOVERNOR;
                    }
                    if fmath::absf(f) >= self.cascade.f_max_n {
                        sat |= FastStatus::SAT_LSM;
                    }
                    f
                }
            };
            let in_last_zone = m.x.diff_m(d.limits.x_obs_close) <= self.last_zone_m;
            let co = if bypass_clamp {
                None
            } else {
                Some(self.clamp.step(f_star, v, &d.limits, in_last_zone))
            };
            let mut f = co.map_or(f_star, |c| c.f_n);
            if co.is_some_and(|c| c.saturated) {
                sat |= FastStatus::SAT_CLAMP;
            }
            // Governor damping only (Wave 2 lead decision D4): a drive force along the
            // velocity fades from full at the governor's planning cap (0.90·E, 0.95·p)
            // to zero at the full energy/momentum cap, so above the caps the drive only
            // brakes (F·v ≤ 0). Excess speed from a hand throw or a rail disturbance is
            // then only damped, which is what the independent monitor's positive-work
            // rule expects (it trips on cap exceedance only while the drive pushes along
            // v). Both the CoM speed and the encoder-point speed (the monitor's
            // measurement, including the pitch rocking of a top-hung leaf) are used.
            // Research-tier direct-force modes are exempt.
            if f_direct.is_none() {
                let mut k = 1.0f32;
                for vv in [v, v_enc] {
                    if fmath::absf(vv) > self.stall_v_mps && f * vv > 0.0 {
                        k = fmath::fminf(k, self.governor.drive_scale(vv, m_hat, &d.limits));
                    }
                }
                if k < 1.0 {
                    f *= k;
                    sat |= FastStatus::SAT_GOVERNOR;
                }
            }
            // Anti-windup against the directional clamp (W1.5): the cascade's own
            // conditional integration only knows F_peak, so while the clamp holds the
            // force below the demand (a 5 t HEAVY breaking away at the 67 N static cap)
            // the velocity integral would keep growing and the carriage would overshoot
            // the governor speed once the clamp opens (Overspeed/EnergyLimit trips). The
            // integral is backed off by the clipped excess, never past zero.
            let excess = f_star - f;
            if f_direct.is_none() && excess != 0.0 && excess.is_finite() {
                let i0 = self.cascade.integ_n;
                let i1 = i0 - excess;
                self.cascade.integ_n = if i0 * i1 < 0.0 { 0.0 } else { i1 };
            }
            f_thrust = f;
            let iq = self.cascade.iq_ref(f);
            if fmath::absf(iq) >= self.cascade.i_max_a && self.cascade.i_max_a > 0.0 {
                sat |= FastStatus::SAT_LSM;
            }
            iq
        } else {
            self.cascade.reset(0.0);
            self.clamp.reset();
            0.0
        };
        if !matches!(
            self.src,
            Src::Admit | Src::HandAdmit | Src::SoftCloseFallback
        ) {
            // Keep the admittance bumpless for the next entry.
            self.adm.reset(r.x, r.v_mps);
        }
        self.last_ref = r;

        // 8. Outputs.
        self.last_f_n = f_thrust;
        // Thrust-moment feed-forward (the LSM acts at the rail; W1.5, PHYSICS §5, §7).
        let mut coil_ref = lev.coil_ref_a;
        self.lev.thrust_ff(f_thrust, lev.gap_heave_m, &mut coil_ref);
        let coil = if d.lev == LevDirective::Off {
            CoilCmd::Off
        } else {
            CoilCmd::CurrentRef(coil_ref)
        };
        let lsm = if lsm_on && enc {
            LsmCmd {
                iq_ref_a: iq,
                id_ref_a: 0.0,
                mode: LsmMode::Run,
            }
        } else {
            LsmCmd {
                iq_ref_a: 0.0,
                id_ref_a: 0.0,
                mode: LsmMode::Sto,
            }
        };

        // 9. Status.
        let arrived = self.arrived(d, &r, m.x, v);
        let status = FastStatus {
            tick: t,
            seq: self.seq,
            directive_seq: d.seq,
            mode,
            x: m.x,
            referenced: m.referenced,
            v_mps: v,
            a_mps2: a,
            x_ref: r.x,
            v_ref_mps: r.v_mps,
            modal: lev.modal,
            gap_m: m.gap_m,
            coil_i_a: m.coil_i_a,
            iq_a: m.iq_a,
            f_hand_hat_n: f_hat,
            residual_n: ob.residual_n,
            m_hat_kg: m_hat,
            e_kin_gov_j: self.governor.e_kin_j(m_hat, v),
            e_passivity_j: self.pc.energy_j,
            a_floor_x_mps2: a_floor,
            intent: flag,
            hand,
            obstruction: self.obst.is_some(),
            obstruction_degraded: ob.degraded,
            arrived,
            lev: lev.phase,
            seat: lev.seat,
            sat_flags: sat,
            faults: lev.faults,
        };
        (FastOutputs { coil, lsm }, status)
    }

    /// The planned segment is complete and the carriage is at its end within the
    /// position tolerance (closed tolerance within the last zone of `x_obs_close`), at
    /// rest.
    fn arrived(&self, d: &Directive, r: &RefState, x: PosNm, v: f32) -> bool {
        let seg = match self.src {
            Src::Directive => d.segment,
            Src::Local(s) => s,
            _ => return false,
        };
        let end = seg.end();
        let tol = if end.diff_m(d.limits.x_obs_close) <= self.last_zone_m {
            self.pos_tol_closed_m
        } else {
            self.pos_tol_travel_m
        };
        let at_rest = fmath::absf(v) < self.stall_v_mps;
        // A log-damped terminus (and its quintic) is slower than the stall speed long
        // before it ends: it has arrived only when its reference has (W1.5; HAP-016).
        let ref_rest = match seg {
            Segment::SCurve(_) => r.done || fmath::absf(r.v_mps) < self.stall_v_mps,
            Segment::Terminus(_) | Segment::Quintic(_) => r.done,
        };
        ref_rest && at_rest && fmath::absf(end.diff_m(x)) <= tol
    }

    /// Returns every block to its initial state.
    pub fn reset(&mut self) {
        self.seq = 0;
        self.lev.reset();
        self.cascade.reset(0.0);
        self.cascade.set_speed_limits(f32::INFINITY, f32::INFINITY);
        self.clamp.reset();
        self.adm.reset(PosNm::ZERO, 0.0);
        self.obs.reset(0.0);
        self.obs.set_lean(false);
        self.pc.reset();
        self.intent.reset();
        self.obst_det.reset();
        self.inertial.reset();
        self.isolator.reset();
        self.track.reset();
        self.track_enc.reset();
        self.src = Src::Idle;
        self.sig = None;
        self.obst = None;
        self.last_ref = RefState::default();
        self.intent_ref = PosNm::ZERO;
        self.hand = false;
        self.hand_ticks = 0;
        self.last_v_mps = 0.0;
        self.last_f_n = 0.0;
        self.pending_terminus = None;
        self.hand_oppose_max_n = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::LiveLimits;
    use fabrail_params::presets;
    use fabrail_types::{SCurveSeg, TravelDir};

    #[test]
    fn stub_is_safe_and_heartbeat_advances() {
        let p = SystemParams::default();
        let mut f = FastLoop::new(&p).unwrap_or_default();
        let m = Measurements {
            tick: Tick(2),
            x: PosNm(42),
            ..Measurements::default()
        };
        let (o, s1) = f.step(&m, &Directive::default());
        assert_eq!(o.lsm.mode, LsmMode::Sto);
        assert_eq!(o.coil, CoilCmd::Off);
        let (_, s2) = f.step(&m, &Directive::default());
        assert_eq!(s2.seq, s1.seq.wrapping_add(1));
        assert_eq!(s2.x, PosNm(42));
    }

    /// 1-D carriage: m·ẍ = K_f·i_q − b·ẋ + F_ext(x, ẋ, t), 10 substeps per release,
    /// encoder quantised to `nm_per_count`, the LSM current loop ideal (i_q = i_q* one
    /// release later).
    struct Plant {
        m: f64,
        b: f64,
        k_f: f64,
        x: f64,
        v: f64,
        iq: f64,
        q_nm: i64,
        g_star: f32,
    }

    impl Plant {
        fn new(p: &SystemParams, m: f64, x: f64) -> Plant {
            Plant {
                m,
                b: p.design.damping.b_dist_n_s_per_m + p.design.damping.c_rail_n_s_per_m,
                k_f: p.design.lsm.k_f_n_per_a,
                x,
                v: 0.0,
                iq: 0.0,
                q_nm: i64::from(p.design.sensors.nm_per_count),
                g_star: p.design.suspension.g_star_m as f32,
            }
        }

        fn meas(&self, tick: u64) -> Measurements {
            let counts = (self.x * 1e9 / self.q_nm as f64).floor() as i64;
            Measurements {
                tick: Tick(tick),
                x: PosNm(counts * self.q_nm),
                gap_m: [self.g_star; fabrail_types::MAX_MAGNETS],
                gap_ok: 0xff,
                referenced: true,
                enc_ok: true,
                iq_a: self.iq as f32,
                ..Measurements::default()
            }
        }

        fn advance(&mut self, out: &FastOutputs, dt: f64, f_ext: impl Fn(f64, f64) -> f64) {
            self.iq = if out.lsm.mode == LsmMode::Run {
                f64::from(out.lsm.iq_ref_a)
            } else {
                0.0
            };
            let h = dt / 10.0;
            for _ in 0..10 {
                let f = self.k_f * self.iq - self.b * self.v + f_ext(self.x, self.v);
                self.v += f / self.m * h;
                self.x += self.v * h;
            }
        }
    }

    /// The loop's own payload estimate (levitation, A1) for a plant at rest at `x`: the
    /// 1-D plant uses it as its mass so these tests exercise motion and haptics only.
    fn loop_mass(p: &SystemParams, x: f64) -> f64 {
        let mut f = FastLoop::new(p).unwrap_or_default();
        let pl = Plant::new(p, 1.0, x);
        let d = Directive {
            lev: LevDirective::Hold,
            ..Directive::default()
        };
        f64::from(f.step(&pl.meas(0), &d).1.m_hat_kg)
    }

    fn limits(p: &SystemParams) -> LiveLimits {
        let sf = &p.safety;
        LiveLimits {
            v_open_mps: sf.v_open_max_mps as f32,
            v_close_mps: sf.v_close_max_mps as f32,
            a_max_mps2: p.control.motion.a_max_mps2 as f32,
            f_drive_n: sf.f_drive_max_n as f32,
            f_static_n: sf.f_static_n as f32,
            f_close_last50_n: sf.f_close_last50_n as f32,
            x_obs_open: PosNm::from_m_f64(p.design.rail.travel_m + 0.01),
            x_obs_close: PosNm::from_m_f64(-0.01),
            ..LiveLimits::default()
        }
    }

    fn traj(p: &SystemParams, from: f64, to: f64, t0: u64) -> Directive {
        let mo = &p.control.motion;
        let v = p.safety.v_open_max_mps.min(p.safety.v_close_max_mps) as f32 * 0.95;
        Directive {
            lev: LevDirective::Hold,
            mode: MotionMode::Trajectory,
            segment: Segment::SCurve(scurve::plan(
                PosNm::from_m_f64(from),
                Tick(t0),
                PosNm::from_m_f64(to),
                v,
                mo.a_max_mps2 as f32,
                mo.j_max_mps3 as f32,
            )),
            limits: limits(p),
            ..Directive::default()
        }
    }

    /// Closed loop on the 1-D plant: a DOOR S-curve 0.1 → 0.6 m arrives within the
    /// travel tolerance (±2 mm) and never exceeds the governor envelope by more than
    /// 10 %. (S) simplified 1-D plant; PRP-006 bound, PHYSICS §16.
    #[test]
    fn trajectory_arrives() {
        for p in [presets::door(), presets::drawer()] {
            let mut f = FastLoop::new(&p).unwrap_or_default();
            let decim = u64::from(p.control.rates.fast_decim);
            let dt = f64::from(f.dt_s);
            let travel = p.design.rail.travel_m;
            let m = loop_mass(&p, 0.2 * travel);
            let (a, b) = (0.2 * travel, 0.8 * travel);
            let mut pl = Plant::new(&p, m, a);
            let d = traj(&p, a, b, 0);
            let dur = f64::from(scurve::duration_s(match &d.segment {
                Segment::SCurve(s) => s,
                _ => unreachable!(),
            }));
            let n = ((dur + 1.0) / dt) as u64;
            let v_cap = p.safety.v_open_max_mps.max(p.safety.v_close_max_mps);
            let mut arrived = false;
            for k in 0..n {
                let (o, s) = f.step(&pl.meas(k * decim), &d);
                pl.advance(&o, dt, |_, _| 0.0);
                assert!(
                    pl.v.abs() <= 1.1 * v_cap,
                    "{:?}: v {} > cap",
                    p.design.class,
                    pl.v
                );
                arrived = s.arrived;
            }
            let tol = p.control.motion.pos_tol_travel_m;
            assert!(
                (pl.x - b).abs() <= tol,
                "{:?}: x {} vs {b}",
                p.design.class,
                pl.x
            );
            assert!(arrived, "{:?}: arrived flag", p.design.class);
        }
    }

    /// A DOOR closing into a stiff obstacle stops, reports the obstruction and reverses
    /// 50 mm; a DRAWER releases to 0 N (LSM STO). (T) SAF-006 reaction, PHYSICS §17.
    #[test]
    fn obstruction_stops_then_reverses_or_releases() {
        for p in [presets::door(), presets::drawer()] {
            let class = p.design.class;
            let mut f = FastLoop::new(&p).unwrap_or_default();
            let decim = u64::from(p.control.rates.fast_decim);
            let dt = f64::from(f.dt_s);
            let m = loop_mass(&p, 0.25);
            let mut pl = Plant::new(&p, m, 0.25);
            let d = traj(&p, 0.25, 0.0, 0);
            // Obstacle (hand, 75 N/mm through a 10 N/mm edge ≈ 8.8 N/mm) at x = 0.15 m.
            let k_obs = 8.8e3;
            let obstacle = |x: f64, _v: f64| if x < 0.15 { k_obs * (0.15 - x) } else { 0.0 };
            let mut detected_at = None;
            let mut min_x = f64::INFINITY;
            let mut sto_seen = false;
            for k in 0..((3.0 / dt) as u64) {
                let (o, s) = f.step(&pl.meas(k * decim), &d);
                pl.advance(&o, dt, obstacle);
                min_x = min_x.min(pl.x);
                if s.obstruction && detected_at.is_none() {
                    detected_at = Some(pl.x);
                }
                if s.obstruction && o.lsm.mode == LsmMode::Sto {
                    sto_seen = true;
                }
            }
            let x_det = detected_at.unwrap_or_else(|| panic!("{class:?}: no obstruction"));
            assert!(
                x_det < 0.15 && x_det > 0.14,
                "{class:?}: detected at {x_det}"
            );
            if p.control.haptic.obstruction.release_to_zero {
                assert!(sto_seen, "{class:?}: released to 0 N (LSM STO)");
            } else {
                // Reversed ≈ 50 mm away from the stop point.
                assert!(
                    pl.x > min_x + 0.04,
                    "{class:?}: reversed to {} from {min_x}",
                    pl.x
                );
            }
        }
    }

    /// Hand-driven admittance: a 10 N push on the DOOR accelerates the carriage at about
    /// F/M_v (M_v = m/3) and the governor caps the speed. (S) 1-D plant, HAP-001 trend.
    #[test]
    fn admittance_renders_virtual_mass() {
        let p = presets::door();
        let mut f = FastLoop::new(&p).unwrap_or_default();
        let decim = u64::from(p.control.rates.fast_decim);
        let dt = f64::from(f.dt_s);
        let m = loop_mass(&p, 0.5);
        let m_v = m / p.control.haptic.mass_ratio;
        let mut pl = Plant::new(&p, m, 0.5);
        let d = Directive {
            lev: LevDirective::Hold,
            mode: MotionMode::Admittance,
            haptic: HapticProfile {
                m_v_kg: m_v as f32,
                b_v_n_s_per_m: 0.0,
                passivity: true,
                ..HapticProfile::default()
            },
            limits: limits(&p),
            ..Directive::default()
        };
        let push = |_x: f64, _v: f64| 10.0;
        let n = (0.4 / dt) as u64;
        for k in 0..n {
            let (o, _) = f.step(&pl.meas(k * decim), &d);
            pl.advance(&o, dt, push);
        }
        // After 0.4 s at F/M_v: v ≈ 10/33.3·0.4 = 0.12 m/s (observer lag trims it).
        let v_ideal = 10.0 / m_v * 0.4;
        assert!(
            pl.v > 0.6 * v_ideal && pl.v < 1.1 * v_ideal,
            "v {} vs {v_ideal}",
            pl.v
        );
        // Keep pushing: capped near the opening envelope.
        for k in n..(n + (3.0 / dt) as u64) {
            let (o, _) = f.step(&pl.meas(k * decim), &d);
            pl.advance(&o, dt, push);
        }
        assert!(
            pl.v <= 1.1 * p.safety.v_open_max_mps,
            "v {} above the cap",
            pl.v
        );
    }

    #[test]
    fn encoder_fault_is_sto() {
        let p = presets::door();
        let mut f = FastLoop::new(&p).unwrap_or_default();
        let d = traj(&p, 0.1, 0.5, 0);
        let m = Measurements {
            enc_ok: false,
            ..Measurements::default()
        };
        let (o, s) = f.step(&m, &d);
        assert_eq!(o.lsm.mode, LsmMode::Sto);
        assert_eq!(s.mode, MotionMode::Off);
    }

    #[test]
    fn terminus_freezes_the_integrator() {
        let p = presets::drawer();
        let mut f = FastLoop::new(&p).unwrap_or_default();
        let decim = u64::from(p.control.rates.fast_decim);
        let dt = f64::from(f.dt_s);
        let mut pl = Plant::new(&p, loop_mass(&p, 0.05), 0.05);
        let tm = &p.control.motion.terminus;
        let seg = Segment::Terminus(fabrail_types::TerminusSeg {
            target: PosNm::ZERO,
            t0: Tick(0),
            dir: TravelDir::Close,
            v_app_mps: 0.0,
            j_mps3: tm.j_mps3 as f32,
            tau_s: tm.tau_s as f32,
            t_b_s: 0.0,
            d_b_m: 0.05,
            d_0_m: 0.05,
            d_eps_m: tm.d_eps_m as f32,
            t_q_s: tm.tau_s as f32,
        });
        let d = Directive {
            lev: LevDirective::Hold,
            mode: MotionMode::Terminus,
            segment: seg,
            terminus: crate::msg::TerminusCfg {
                freeze_integrator: true,
                d_eps_m: tm.d_eps_m as f32,
                ..crate::msg::TerminusCfg::default()
            },
            limits: limits(&p),
            ..Directive::default()
        };
        let mut integ = None;
        for k in 0..50u64 {
            let (o, _) = f.step(&pl.meas(k * decim), &d);
            pl.advance(&o, dt, |_, _| 0.0);
            let i = f.cascade.integ_n;
            if let Some(i0) = integ {
                assert_eq!(i, i0, "integrator frozen during capture (plan §4b)");
            }
            integ = Some(i);
        }
    }

    #[test]
    fn default_limits_do_not_move() {
        // Zero live limits: the governor pins the command, the clamp allows no drive.
        let p = presets::door();
        let mut f = FastLoop::new(&p).unwrap_or_default();
        let decim = u64::from(p.control.rates.fast_decim);
        let dt = f64::from(f.dt_s);
        let mut pl = Plant::new(&p, loop_mass(&p, 0.3), 0.3);
        let mut d = traj(&p, 0.3, 0.8, 0);
        d.limits = LiveLimits::default();
        for k in 0..((1.0 / dt) as u64) {
            let (o, _) = f.step(&pl.meas(k * decim), &d);
            pl.advance(&o, dt, |_, _| 0.0);
        }
        assert!((pl.x - 0.3).abs() < 1e-4, "moved to {}", pl.x);
        let _ = SCurveSeg::default();
    }

    /// D6: under unattended limits a hand during the log-damped capture raises the
    /// obstruction flag at the yield instant (≤ 20 ms after contact), the DRAWER releases
    /// to 0 N (no re-close), and the flag stays latched after the hand lets go. (S) 1-D
    /// plant; plan §4b unattended pinch, FAB-EXA "detected within 20 ms".
    #[test]
    fn unattended_pinch_flags_at_yield() {
        let p = presets::drawer();
        let mut f = FastLoop::new(&p).unwrap_or_default();
        let decim = u64::from(p.control.rates.fast_decim);
        let dt = f64::from(f.dt_s);
        let x0 = 0.28;
        let mut pl = Plant::new(&p, loop_mass(&p, x0), x0);
        let tm = &p.control.motion.terminus;
        let mut lim = limits(&p);
        lim.unattended = true;
        let base = Directive {
            lev: LevDirective::Hold,
            terminus: crate::msg::TerminusCfg {
                freeze_integrator: true,
                hand_yields: true,
                d_eps_m: tm.d_eps_m as f32,
                ..crate::msg::TerminusCfg::default()
            },
            limits: lim,
            ..Directive::default()
        };
        // Cruise toward the seat first (a long S-curve), then hand over to the
        // log-damped terminus planned from the measured state 60 mm before the target
        // (the supervisor's Trajectory → Terminus hand-over).
        let v_c = 0.12f32;
        let cruise = Directive {
            mode: MotionMode::Trajectory,
            segment: Segment::SCurve(scurve::plan(
                PosNm::from_m_f64(x0),
                Tick(0),
                PosNm::from_m_f64(-1.0),
                v_c,
                p.control.motion.a_max_mps2 as f32,
                p.control.motion.j_max_mps3 as f32,
            )),
            ..base
        };
        // A hand in the gap at x = 20 mm (stiff 20 kN/m contact) until t = 3 s.
        let x_hand = 0.02f64;
        let t_off = 3.0f64;
        let mut term: Option<Directive> = None;
        let mut t_contact = None;
        let mut t_flag = None;
        let mut peak = 0.0f64;
        for k in 0..25_000u64 {
            let t = k as f64 * dt;
            let tick = Tick(k * decim);
            if term.is_none() && pl.x <= 0.06 {
                let seg = f.terminus.plan(
                    PosNm::ZERO,
                    TravelDir::Close,
                    tick,
                    pl.x as f32,
                    -pl.v as f32,
                    0.0,
                    tm.tau_s as f32,
                    tm.j_mps3 as f32,
                );
                term = Some(Directive {
                    mode: MotionMode::Terminus,
                    segment: seg,
                    ..base
                });
            }
            let d = term.unwrap_or(cruise);
            let (o, s) = f.step(&pl.meas(k * decim), &d);
            assert!(
                term.is_some() || !s.obstruction,
                "no obstruction while cruising at {t}"
            );
            if s.obstruction && t_flag.is_none() {
                t_flag = Some(t);
            }
            let on = t < t_off;
            pl.advance(&o, dt, |x, _| {
                let pen = x_hand - x;
                if on && pen > 0.0 { 20_000.0 * pen } else { 0.0 }
            });
            if on && pl.x < x_hand {
                t_contact.get_or_insert(t);
                peak = peak.max(20_000.0 * (x_hand - pl.x));
            }
            if t > t_off + 0.2 {
                assert!(
                    s.obstruction,
                    "obstruction stays latched after the hand lets go (D6)"
                );
                assert!(
                    o.lsm.mode == LsmMode::Sto || o.lsm.iq_ref_a.abs() < 1e-3,
                    "DRAWER released to 0 N, no re-close (plan §4b)"
                );
            }
        }
        let t_contact = t_contact.expect("the drawer reaches the hand");
        let t_flag = t_flag.expect("obstruction flag raised (D6)");
        assert!(
            t_flag >= t_contact && t_flag - t_contact <= 0.020,
            "(S) flag {:.1} ms after contact, at the yield instant (≤ 20 ms; not at the \
             hand's release at {t_off} s; D6)",
            (t_flag - t_contact) * 1e3
        );
        assert!(
            peak <= 150.0,
            "(S) peak pinch force {peak} N ≤ 150 N (FAB-EXA unattended pinch)"
        );
        eprintln!(
            "unattended pinch: flag {:.1} ms after contact, peak {peak:.1} N",
            (t_flag - t_contact) * 1e3
        );
    }
}
