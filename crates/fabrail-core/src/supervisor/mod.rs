//! Supervisor (1 kHz): `Supervisor::step` → (Directive, SupStatus); LevState machine,
//! mode selection and segment planning, fault reactions, power and hold-up
//! (PHYSICS §18). Owner: A4 (module list frozen by W0a).
//!
//! Frozen API (W0): [`Supervisor::new`], [`Supervisor::step`] (core policy, what the
//! firmware runs) and [`Supervisor::step_with_policy`] (the `ActionPolicy` seam used by
//! the exotic tier and the house choreographer in simulation; plan §4, §4b).
//!
//! Per release the supervisor
//!
//! 1. converts the supply sample (`power`: hold-up energy ≥ 3× the landing need,
//!    MainsLost after a 2 ms debounce; SAF-009);
//! 2. computes the core land-first class action and passes any `ActionPolicy` proposal
//!    through the core-side admission filter (`seismic::action_policy::resolve`);
//! 3. raises its own faults (PayloadMismatch when m̂ > 1.05·m_rated for `m_hat_tau_s`,
//!    NotReferenced, ObstructionDegraded, ParamCrc on a CRC/diverse-copy mismatch,
//!    state timeouts), applies a `Reset`, latches faults and computes the reaction
//!    demand, which is never below the monitor's (SAF-013, SAF-016);
//! 4. accepts or rejects the command (hold-to-run gate, presence gate PL d, INSPECT
//!    gate: no lift or move outside NORMAL), detects intent edges, hand-released
//!    captures and the hover timeout;
//! 5. advances the pure LevState table (`levstate::transition`) and runs the entry
//!    actions and the sub-sequences of FAULT_REACTION (stop at a_brk → DRAWER seat-first
//!    from the standoff when only the supply failed → land; STO on the LSM only),
//!    SEISMIC (freeze → admitted target move under unattended limits → land / lock /
//!    hold / research mode) and ESCAPE (open to the travel end, then land, never lock);
//! 6. assembles the `Directive` through `modes::ModePlanner` (segments, live limits,
//!    haptic profile) and the `SupStatus`, and logs discrete events to the 64-entry
//!    ring that fabrail-rt drains to the Telemetry HAL.
//!
//! Timing (LET, PHYSICS §24): a directive published at release r activates at r + P
//! (P = `sup_decim` base ticks); segments are planned for that activation tick. A fast
//! status completes a state (touchdown, arrival) only when it was produced under a
//! directive of that state (`directive_seq` ≥ the state's first directive).
//!
//! Time limits are derived in `new()` from the parameters (each a design choice with a
//! 2× margin on the physical duration plus a fixed allowance for the LET delays): lift
//! 2·t_lift + 0.5 s; landing 2·(g_rest − g_up)/v_land + 1 s; seat step 5·t_seat + 0.5 s;
//! stop 2·v_max/a_brk + 0.5 s; fault reaction = stop + landing + seat step + seat-first
//! move + 1 s; parking-lock release 1 s.

pub mod faults;
pub mod levstate;
pub mod modes;
pub mod power;

use fabrail_params::{OperatingMode, ParamError, PolicyTuning, SystemParams, crc, derive};
use fabrail_types::{
    AuxCmd, EgressRole, Event, EventKind, FaultCode, LevState, MotionMode, PosNm, PresenceSensing,
    RailClass, RestPolicy, SafetyDemand, SeismicAction, SeismicState, Severity, Tick, TravelDir,
    UserCommand, fmath,
};

use crate::msg::{
    Directive, FastStatus, IntentFlag, LevDirective, LevPhase, SeatStatus, SeismicReport,
    SupInputs, SupStatus,
};
use crate::seismic::action_policy::{
    ActionContext, ActionDecision, ActionPolicy, CorePolicy, PolicyOutcome, resolve,
};
use crate::seismic::policy::{PolicyInputs, class_action};

use levstate::{LevInputs, is_levitated};
use modes::{MotionCmd, MoveKind, MoveStart};

/// Idle time in the seat slot after which a lifted but unmoved DRAWER is re-seated, s
/// (Wave 2 lead decision D7; design choice). Holding InSlot costs ≈ 3.7 A per coil at
/// 26 kg (87 W) and ≈ 10.5 A at 48 kg, so an unseat with no move does not wait for the
/// hover timeout; a pending move drives out instead (LevState Lifting → Moving).
pub const SLOT_IDLE_RESEAT_S: f64 = 2.0;

/// Supervisor-rate seconds → base ticks (rounded up).
fn ticks(t_s: f64) -> u64 {
    if t_s.is_finite() && t_s > 0.0 {
        fmath::ceil(t_s * f64::from(fabrail_types::BASE_TICK_HZ)) as u64
    } else {
        0
    }
}

/// Sub-phase of the multi-step sequences in FAULT_REACTION, SEISMIC and ESCAPE.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Sub {
    /// Controlled stop (Freeze at a_brk; ESCAPE uses a Hold stop).
    #[default]
    Brake,
    /// Executing a planned move (seat-first, admitted seismic target, escape).
    Move,
    /// Levitated hold (freeze-only alert, passive ride-through).
    Hold,
    /// Controlled landing (SS1 Safe Landing in FAULT_REACTION).
    Land,
    /// DRAWER z-seat step.
    Seat,
    /// Research-tier mode (isolation, AMD, inertial smoothing; disabled in presets).
    Research,
    /// Carriage already down (nothing to land).
    Down,
}

/// Why a STOPPING is in progress.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum StopKind {
    /// Stop command, hold-to-run released, re-plan, land/manual request, overdue move.
    #[default]
    User,
    /// Obstruction: reverse 50 mm (DOOR/WALL) or release to 0 N (DRAWER/HEAVY).
    Obstruction,
}

/// An accepted move that has not started yet.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MoveReq {
    /// Target position.
    pub target: PosNm,
    /// Why.
    pub kind: MoveKind,
}

/// Command rejection reasons (the `b` payload of `EventKind::Rejected`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum RejectReason {
    /// Seismic state is not NORMAL (INSPECT gate; plan §4b).
    NotNormal = 1,
    /// Hold-to-run required but not actuated (NotHoldToRun).
    HoldToRun = 2,
    /// The current state does not accept the command.
    State = 3,
    /// Lift refused (CRC, selftest, hold-up, fault reaction).
    LiftRefused = 4,
}

/// Wire index of a command for `EventKind::Command` / `Rejected` (a), with the GoTo
/// target in metres (b). Indices match the web engine's command codes where both
/// exist (open 1 … hold_to_run_off 8).
pub fn command_index(c: &UserCommand) -> (f32, f32) {
    match c {
        UserCommand::Open => (1.0, 0.0),
        UserCommand::Close => (2.0, 0.0),
        UserCommand::Stop => (3.0, 0.0),
        UserCommand::Lift => (4.0, 0.0),
        UserCommand::Land => (5.0, 0.0),
        UserCommand::Reset => (6.0, 0.0),
        UserCommand::HoldToRun(true) => (7.0, 0.0),
        UserCommand::HoldToRun(false) => (8.0, 0.0),
        UserCommand::GoTo(x) => (9.0, x.diff_m(PosNm::ZERO)),
        UserCommand::Manual => (10.0, 0.0),
    }
}

/// Constants derived from the parameters in `new()` (f32 and base ticks).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SupCfg {
    /// Supervisor period, base ticks (LET activation offset).
    pub p_sup: u64,
    /// Rated mass, kg.
    pub m_rated_kg: f32,
    /// PayloadMismatch ratio (1.05; F18).
    pub payload_ratio: f32,
    /// PayloadMismatch persistence (m̂ time constant), ticks.
    pub t_payload: u64,
    /// Stall speed (2 mm/s), m/s.
    pub stall_v_mps: f32,
    /// Lift time limit, ticks.
    pub t_lift: u64,
    /// Landing time limit, ticks.
    pub t_land: u64,
    /// Seat step time limit, ticks.
    pub t_seat: u64,
    /// Stop time limit, ticks.
    pub t_stop: u64,
    /// Fault-reaction time limit, ticks.
    pub t_fault_reaction: u64,
    /// Parking-lock release limit, ticks.
    pub t_lock: u64,
    /// Hover timeout (0 = none), ticks.
    pub t_hover: u64,
    /// Idle time in the seat slot before a re-seat ([`SLOT_IDLE_RESEAT_S`]), ticks.
    pub t_slot_idle: u64,
    /// Lift retries before LiftTimeout.
    pub lift_retries: u8,
    /// Landing speed, m/s.
    pub land_rate_mps: f32,
    /// Parking lock fitted (never used on an egress path).
    pub parking_lock: bool,
    /// Lock after a seismic landing.
    pub lock_after_land: bool,
    /// Seismic x-latch fitted.
    pub seismic_latch: bool,
    /// DRAWER push-to-open from the seat.
    pub push_to_open: bool,
    /// Automatic operation from rest by intent (not on hold-to-run classes).
    pub intent_from_rest: bool,
    /// Hold-to-run operating mode (HEAVY unsensed).
    pub hold_to_run_mode: bool,
    /// Seat-first window from the seat (inertial standoff), m.
    pub standoff_m: f32,
    /// Hand-release capture window (soft-close capture distance), m.
    pub capture_m: f32,
    /// Minimum hand-released speed for a capture (soft-close final speed), m/s.
    pub capture_v_mps: f32,
    /// Obstruction reversal distance (0 = release to 0 N), m.
    pub reverse_m: f32,
    /// Inertial smoothing enabled (research tier).
    pub inertial_enable: bool,
    /// Centring target (WALL/HEAVY, F11), if configured.
    pub x_center: Option<PosNm>,
    /// Time to land from the float gap (class policy input), s.
    pub t_land_s: f32,
    /// Speed available for centring, m/s.
    pub v_center_mps: f32,
    /// Actuator-veto acceleration threshold, m/s².
    pub veto_a_mps2: f32,
}

/// The 1 kHz supervisor.
#[derive(Clone, Debug)]
pub struct Supervisor {
    /// Release sequence number.
    pub seq: u32,
    /// LevState.
    pub lev: LevState,
    /// Class.
    pub class: RailClass,
    /// Rest policy.
    pub rest_policy: RestPolicy,
    /// Egress role.
    pub egress: EgressRole,
    /// PL d presence sensing fitted (sensed limits apply only when fitted AND healthy).
    pub presence_fitted: bool,
    /// Closed / seat position.
    pub x_closed: PosNm,
    /// Unattended automatic-close speed limit, m/s (plan §4b).
    pub v_unattended_mps: f32,
    /// Feeds required for alert-triggered motion without a local P pick.
    pub min_independent_feeds: u8,
    /// Class seismic policy tuning.
    pub policy: PolicyTuning,
    /// Fault latch.
    pub faults: faults::FaultManager,
    /// Supply monitor.
    pub supply: power::SupplyMonitor,
    /// Mode selector / planner.
    pub modes: modes::ModePlanner,
    /// Discrete events for telemetry (fabrail-rt drains them to the Telemetry HAL).
    pub events: crate::telemetry::EventRing,
    /// Parameter CRC and diverse copies valid (`crc::verify_seal`; SAF-015).
    pub crc_ok: bool,
    /// The last SELFTEST passed (restore after INSPECT requires it; plan §4b).
    pub selftest_passed: bool,
    /// Open travel end.
    pub x_open: PosNm,
    /// Operating mode.
    pub operating_mode: OperatingMode,
    /// Derived constants.
    pub cfg: SupCfg,
    /// Sub-phase of FAULT_REACTION / SEISMIC / ESCAPE.
    pub sub: Sub,
    /// Why the current STOPPING runs.
    pub stop_kind: StopKind,
    /// Accepted move not started yet.
    pub pending: Option<MoveReq>,
    /// Land requested (entered after the current stop).
    pub land_pending: bool,
    /// MANUAL requested (entered once down).
    pub manual_pending: bool,
    /// Tick the current LevState was entered.
    pub entered: Tick,
    /// Sequence number of the current state's first directive.
    pub state_seq: u32,
    /// A fast status has been seen since boot.
    pub fast_seen: bool,
    /// Hold-to-run latched from the command link (`UserCommand::HoldToRun`).
    pub htr_cmd: bool,
    /// Emergency stop latched until Reset with the button released.
    pub estop_latched: bool,
    /// Lift attempts in the current lift request.
    pub lift_attempts: u8,
    /// Retry the lift after an aborted attempt.
    pub retry_lift: bool,
    /// HOVER class: re-lift when permitted (boot, hold-up recovered).
    pub auto_lift: bool,
    /// Lift/unseat directive start tick (after the parking lock released).
    pub lift_t0: Option<Tick>,
    /// Current lift is a DRAWER unseat.
    pub unseat: bool,
    /// Engage the parking lock at rest after a seismic Lock.
    pub lock_request: bool,
    /// Since when m̂ has exceeded the PayloadMismatch threshold.
    pub payload_since: Option<Tick>,
    /// An obstruction ended an unattended move in this seismic/escape episode.
    pub obstructed: bool,
    /// ESCAPE: the open move has ended (land after the stop).
    pub escape_opened: bool,
    /// LEVITATED_IDLE activity reference for the hover timeout.
    pub idle_since: Tick,
    /// Re-run SELFTEST at rest (INSPECT reset returned to NORMAL).
    pub reselftest: bool,
    /// Previous release's intent (edge detection).
    pub prev_intent: IntentFlag,
    /// The pending move came from a confirmed intent: it starts once the hand has let
    /// go (push-and-go, W1.5), so the automatic move never drags a gripping hand.
    pub pending_from_intent: bool,
    /// Previous release's obstruction flag.
    pub prev_obstruction: bool,
    /// Previous seismic state.
    pub prev_seis: SeismicState,
    /// Previous commanded mode.
    pub prev_mode: MotionMode,
    /// Previous demand.
    pub prev_demand: SafetyDemand,
}

impl Supervisor {
    /// Builds the supervisor (range validation is `fabrail_rt::Executive::new`'s job;
    /// a CRC/diverse-copy mismatch is a runtime refuse-to-lift, SAF-015).
    pub fn new(p: &SystemParams) -> Result<Supervisor, ParamError> {
        let d = &p.design;
        let c = &p.control;
        let sf = &p.safety;
        let v_unattended = derive::unattended_close_speed(
            sf.v_close_max_mps,
            sf.unattended.f_pinch_n,
            d.carriage.m_rated_kg,
            sf.unattended.k_series_n_per_m,
        );
        let lt = &c.levitation;
        let mo = &c.motion;
        let s = &d.suspension;
        let a_brk = mo.a_brk_ratio * mo.a_max_mps2;
        let v_max = fmath::fmax(
            fmath::fmax(sf.v_open_max_mps, sf.v_close_max_mps),
            fmath::fmax(sf.v_open_sensed_mps, sf.v_close_sensed_mps),
        );
        let g_rest = fmath::fmax(s.g_land_m, s.g_seat_m);
        let t_land_phys = if lt.land_speed_mps > 0.0 {
            fmath::abs(g_rest - s.g_up_m) / lt.land_speed_mps
        } else {
            0.0
        };
        let t_stop_s = if a_brk > 0.0 { v_max / a_brk } else { 0.0 };
        let t_seat_first = if d.rest_policy == RestPolicy::Seat && v_unattended > 0.0 {
            derive::terminus_t_arrive(
                v_unattended,
                mo.terminus.j_mps3,
                mo.terminus.tau_s,
                mo.terminus.d_eps_m,
            ) + mo.inertial.standoff_m / v_unattended
        } else {
            0.0
        };
        let t_stop = 2.0 * t_stop_s + 0.5;
        let t_land = 2.0 * t_land_phys + 1.0;
        let t_seat = 5.0 * lt.seat_time_s + 0.5;
        let x_closed = PosNm::from_m_f64(d.rail.x_closed_m);
        let x_open = PosNm::from_m_f64(d.rail.x_closed_m + d.rail.travel_m);
        let po = &c.seismic.policy;
        let x_center = if po.center_if_reachable {
            Some(PosNm::from_m_f64(d.rail.x_closed_m + 0.5 * d.rail.travel_m))
        } else {
            None
        };
        let v_center = fmath::fmin(
            fmath::fmin(sf.v_open_max_mps, sf.v_close_max_mps),
            v_unattended,
        );
        let cfg = SupCfg {
            p_sup: u64::from(c.rates.sup_decim.max(1)),
            m_rated_kg: d.carriage.m_rated_kg as f32,
            payload_ratio: c.supervisor.payload_mismatch_ratio as f32,
            t_payload: ticks(lt.m_hat_tau_s),
            stall_v_mps: c.supervisor.stall_v_mps as f32,
            t_lift: ticks(2.0 * lt.lift_time_max_s + 0.5),
            t_land: ticks(t_land),
            t_seat: ticks(t_seat),
            t_stop: ticks(t_stop),
            t_fault_reaction: ticks(t_stop + t_land + t_seat + t_seat_first + 1.0),
            t_lock: ticks(1.0),
            t_hover: ticks(c.supervisor.hover_timeout_s),
            t_slot_idle: ticks(SLOT_IDLE_RESEAT_S),
            lift_retries: c.supervisor.lift_retries,
            land_rate_mps: lt.land_speed_mps as f32,
            parking_lock: d.landing.parking_lock && d.egress == EgressRole::None,
            lock_after_land: po.lock_after_land,
            seismic_latch: d.landing.seismic_latch.enabled,
            push_to_open: c.intent.push_to_open && d.rest_policy == RestPolicy::Seat,
            intent_from_rest: d.operating_mode != OperatingMode::HoldToRun,
            hold_to_run_mode: d.operating_mode == OperatingMode::HoldToRun,
            standoff_m: mo.inertial.standoff_m as f32,
            capture_m: c.haptic.soft_close_capture_m as f32,
            capture_v_mps: c.haptic.soft_close_v_final_mps as f32,
            reverse_m: if c.haptic.obstruction.release_to_zero {
                0.0
            } else {
                c.haptic.obstruction.reverse_m as f32
            },
            inertial_enable: mo.inertial.enable,
            x_center,
            t_land_s: (t_land_phys + t_stop_s) as f32,
            v_center_mps: v_center as f32,
            // Self-actuation veto above 10 % of a_max (design choice; SEI-005).
            veto_a_mps2: (0.1 * mo.a_max_mps2) as f32,
        };
        Ok(Supervisor {
            seq: 0,
            lev: LevState::Boot,
            class: d.class,
            rest_policy: d.rest_policy,
            egress: d.egress,
            presence_fitted: d.presence == PresenceSensing::PlD,
            x_closed,
            v_unattended_mps: v_unattended as f32,
            min_independent_feeds: c.seismic.fusion.min_independent_feeds,
            policy: c.seismic.policy,
            faults: faults::FaultManager::new(p)?,
            supply: power::SupplyMonitor::new(p)?,
            modes: modes::ModePlanner::new(p)?,
            events: crate::telemetry::EventRing::new(),
            crc_ok: crc::verify_seal(p).is_ok(),
            selftest_passed: false,
            x_open,
            operating_mode: d.operating_mode,
            cfg,
            sub: Sub::Brake,
            stop_kind: StopKind::User,
            pending: None,
            land_pending: false,
            manual_pending: false,
            entered: Tick::ZERO,
            state_seq: 0,
            fast_seen: false,
            htr_cmd: false,
            estop_latched: false,
            lift_attempts: 0,
            retry_lift: false,
            auto_lift: false,
            lift_t0: None,
            unseat: false,
            lock_request: false,
            payload_since: None,
            obstructed: false,
            escape_opened: false,
            idle_since: Tick::ZERO,
            reselftest: false,
            prev_intent: IntentFlag::None,
            pending_from_intent: false,
            prev_obstruction: false,
            prev_seis: SeismicState::Normal,
            prev_mode: MotionMode::Off,
            prev_demand: SafetyDemand::None,
        })
    }

    /// Simulation warm start: adopt the plant's initial support state (SEATED, LANDED
    /// or LEVITATED_IDLE) instead of booting. Refused (false) with a bad CRC (SAF-015)
    /// or for any other state; the fast group must be warm-started consistently.
    pub fn warm_start(&mut self, lev: LevState) -> bool {
        if !self.crc_ok
            || !matches!(
                lev,
                LevState::Seated | LevState::Landed | LevState::LevitatedIdle
            )
        {
            return false;
        }
        self.lev = lev;
        self.selftest_passed = true;
        self.entered = Tick::ZERO;
        self.state_seq = self.seq;
        self.idle_since = Tick::ZERO;
        true
    }

    /// One supervisor release with the core policy (no override).
    pub fn step(
        &mut self,
        s: &FastStatus,
        i: &SupInputs,
        r: &SeismicReport,
    ) -> (Directive, SupStatus) {
        self.step_with_policy(s, i, r, &mut CorePolicy)
    }

    fn ev(&mut self, t: Tick, kind: EventKind, a: f32, b: f32) {
        self.events.push(Event {
            tick: t,
            kind,
            a,
            b,
        });
    }

    fn reject(&mut self, t: Tick, c: &UserCommand, why: RejectReason) {
        let (a, _) = command_index(c);
        self.ev(t, EventKind::Rejected, a, f32::from(why as u8));
    }

    /// One supervisor release, consulting `policy` for the seismic action while the
    /// seismic state is not NORMAL. Proposals pass `action_policy::admit`.
    pub fn step_with_policy(
        &mut self,
        s: &FastStatus,
        i: &SupInputs,
        r: &SeismicReport,
        policy: &mut dyn ActionPolicy,
    ) -> (Directive, SupStatus) {
        use LevState as L;
        self.seq = self.seq.wrapping_add(1);
        let now = i.tick;
        let act = now.plus(self.cfg.p_sup);
        let supply = self.supply.step(&i.power);
        if s.seq != 0 {
            self.fast_seen = true;
        }
        let fresh = self.fast_seen && (s.directive_seq.wrapping_sub(self.state_seq) as i32) >= 0;
        let stopped = fmath::absf(s.v_mps) <= self.cfg.stall_v_mps;
        let at_closed = fmath::absf(self.x_closed.diff_m(s.x)) <= self.modes.tol_closed_m;
        if let Some(UserCommand::HoldToRun(b)) = i.cmd {
            self.htr_cmd = b;
        }
        let htr = i.safety_in.hold_to_run || self.htr_cmd;
        // Presence gate (SAF-003): sensed caps only with PL d fitted, healthy and the
        // zone clear (the monitor allows them when fitted and healthy; the core is
        // stricter, never looser).
        let sensed =
            self.presence_fitted && i.safety_in.presence_ok && !i.safety_in.presence_active;
        let normal = r.state == SeismicState::Normal;

        // ---- Seismic action: core class table, then the policy hook -----------------
        let pin = PolicyInputs {
            tick: now,
            class: self.class,
            rest_policy: self.rest_policy,
            egress: self.egress,
            seis: r.state,
            level: r.level,
            x: s.x,
            v_mps: s.v_mps,
            x_center: self.cfg.x_center,
            s_eta: r.s_eta,
            t_land_s: self.cfg.t_land_s,
            v_center_mps: self.cfg.v_center_mps,
            source: r.source,
            x_open: (self.egress == EgressRole::NonRated).then_some(self.x_open),
            v_open_mps: self.cfg.v_center_mps,
            a_mps2: self.modes.a_max,
        };
        let default = class_action(&pin, &self.policy);
        let (action, outcome) = if normal {
            (default, PolicyOutcome::Default)
        } else {
            let user_motion = if self.lev == L::Moving
                && self.modes.plan.active
                && self.modes.plan.kind == MoveKind::Command
            {
                Some(self.modes.plan.dir)
            } else if s.hand && s.v_mps > 0.0 {
                Some(TravelDir::Open)
            } else if s.hand && s.v_mps < 0.0 {
                Some(TravelDir::Close)
            } else {
                None
            };
            let ctx = ActionContext {
                tick: now,
                class: self.class,
                rest_policy: self.rest_policy,
                egress: self.egress,
                seis: r.state,
                lev: self.lev,
                mode: s.mode,
                x: s.x,
                v_mps: s.v_mps,
                x_closed: self.x_closed,
                user_motion,
                report: *r,
                default,
                v_unattended_mps: self.v_unattended_mps,
                min_independent_feeds: self.min_independent_feeds,
            };
            resolve(&ctx, policy)
        };

        // ---- Supervisor-own faults and state timers ----------------------------------
        let mut own = supply.faults;
        if !self.crc_ok {
            own.set(FaultCode::ParamCrc);
        }
        let levitated_now = s.lev == LevPhase::Levitated && is_levitated(self.lev);
        let m_hat = s.m_hat_kg;
        if levitated_now
            && m_hat.is_finite()
            && m_hat > self.cfg.payload_ratio * self.cfg.m_rated_kg
        {
            // PayloadMismatch: m̂ > 1.05·m_rated for the m̂ time constant (F18).
            let since = *self.payload_since.get_or_insert(now);
            if now.since(since) >= self.cfg.t_payload {
                own.set(FaultCode::PayloadMismatch);
            }
        } else {
            self.payload_since = None;
        }
        if self.fast_seen && !s.referenced && !matches!(self.lev, L::Boot | L::SelfTest) {
            own.set(FaultCode::NotReferenced);
        }
        if s.obstruction_degraded {
            own.set(FaultCode::ObstructionDegraded);
        }
        let obstruction_edge = s.obstruction && !self.prev_obstruction;
        if obstruction_edge {
            own.set(FaultCode::Obstruction);
            self.ev(now, EventKind::Obstruction, s.residual_n, 0.0);
        }
        let t_in = now.since(self.entered);
        let mut timeout = false;
        match self.lev {
            L::Lifting => {
                if self.cfg.parking_lock && i.safety_in.lock_engaged && self.lift_t0.is_none() {
                    if t_in > self.cfg.t_lock {
                        own.set(FaultCode::LockFailed);
                        timeout = true;
                    }
                } else {
                    let t0 = *self.lift_t0.get_or_insert(now);
                    if (fresh && s.lev == LevPhase::Timeout) || now.since(t0) > self.cfg.t_lift {
                        self.lift_attempts = self.lift_attempts.saturating_add(1);
                        if self.lift_attempts > self.cfg.lift_retries {
                            own.set(FaultCode::LiftTimeout);
                        } else {
                            timeout = true;
                            self.retry_lift = true;
                        }
                    }
                }
            }
            L::Landing if t_in > self.cfg.t_land || (fresh && s.lev == LevPhase::Timeout) => {
                own.set(FaultCode::LandTimeout);
            }
            L::Seating if t_in > self.cfg.t_seat || (fresh && s.seat == SeatStatus::Failed) => {
                own.set(FaultCode::SeatFailed);
            }
            L::Stopping => timeout = t_in > self.cfg.t_stop,
            L::Moving => timeout = self.modes.overdue(now),
            L::FaultReaction if t_in > self.cfg.t_fault_reaction => {
                own.set(FaultCode::LandTimeout);
                timeout = true;
            }
            _ => {}
        }

        // ---- Reset, fault latch and reaction demand ----------------------------------
        let reset = i.cmd == Some(UserCommand::Reset);
        let inspect_reset = reset && r.state == SeismicState::Inspect;
        if reset && self.lev != L::FaultReaction {
            let cleared = self.faults.latched;
            for c in cleared.iter() {
                self.ev(now, EventKind::FaultCleared, f32::from(c.as_u16()), 0.0);
            }
            self.faults.reset();
            if !i.safety_in.estop {
                self.estop_latched = false;
            }
            self.lift_attempts = 0;
            self.obstructed = false;
        }
        let reaction = self.faults.step(s.faults, r.faults, own, &i.verdict);
        for c in reaction.raised.iter() {
            self.ev(now, EventKind::FaultRaised, f32::from(c.as_u16()), 0.0);
        }
        if i.safety_in.estop {
            self.estop_latched = true;
        }
        let mut demand = reaction.demand;
        if self.estop_latched {
            // Emergency stop: stop category 1 (SS1) and a latched FAULT until Reset.
            demand = core::cmp::max(demand, SafetyDemand::SafeStop);
        }
        let mut reported = reaction.latched;
        let selftest_ok = !reaction.latched.contains(FaultCode::SelftestHash);
        let htr_required = (self.cfg.hold_to_run_mode && !sensed) || reaction.hold_to_run;
        let htr_ok = !htr_required || htr;

        // ---- Commands ---------------------------------------------------------------
        let mut lift_req = false;
        let mut stop_req = false;
        let mut land_req = false;
        let rest_state = matches!(self.lev, L::Seated | L::Landed);
        let accepts_move = matches!(
            self.lev,
            L::Seated | L::Landed | L::Lifting | L::LevitatedIdle | L::Moving | L::Stopping
        );
        if let Some(c) = i.cmd {
            let (a_idx, b_val) = command_index(&c);
            match c {
                UserCommand::Open | UserCommand::Close | UserCommand::GoTo(_) => {
                    let target = match c {
                        UserCommand::Open => self.x_open,
                        UserCommand::Close => self.x_closed,
                        UserCommand::GoTo(x) => self.modes.clamp_target(x),
                        _ => s.x,
                    };
                    if !normal {
                        self.reject(now, &c, RejectReason::NotNormal);
                    } else if !accepts_move || demand >= SafetyDemand::SafeStop {
                        self.reject(now, &c, RejectReason::State);
                    } else if !htr_ok {
                        reported.set(FaultCode::NotHoldToRun);
                        self.ev(
                            now,
                            EventKind::FaultRaised,
                            f32::from(FaultCode::NotHoldToRun.as_u16()),
                            0.0,
                        );
                        self.reject(now, &c, RejectReason::HoldToRun);
                    } else if rest_state && reaction.refuse_lift {
                        self.reject(now, &c, RejectReason::LiftRefused);
                    } else if self.lev == L::Seated && target == self.x_closed {
                        self.ev(now, EventKind::Command, a_idx, b_val); // already seated
                    } else {
                        self.ev(now, EventKind::Command, a_idx, b_val);
                        let same = self.lev == L::Moving
                            && self.modes.plan.active
                            && self.modes.plan.target == target;
                        if !same {
                            self.pending_from_intent = false;
                            self.pending = Some(MoveReq {
                                target,
                                kind: MoveKind::Command,
                            });
                            if self.lev == L::Moving {
                                stop_req = true; // stop, then re-plan
                                self.stop_kind = StopKind::User;
                            }
                        }
                    }
                }
                UserCommand::Stop => {
                    self.ev(now, EventKind::Command, a_idx, b_val);
                    self.pending = None;
                    self.land_pending = false;
                    if self.lev == L::Moving {
                        stop_req = true;
                        self.stop_kind = StopKind::User;
                    }
                }
                UserCommand::Lift => {
                    if !rest_state {
                        self.reject(now, &c, RejectReason::State);
                    } else if !normal {
                        self.reject(now, &c, RejectReason::NotNormal);
                    } else if reaction.refuse_lift {
                        self.reject(now, &c, RejectReason::LiftRefused);
                    } else {
                        self.ev(now, EventKind::Command, a_idx, b_val);
                        lift_req = true;
                    }
                }
                UserCommand::Land => {
                    self.ev(now, EventKind::Command, a_idx, b_val);
                    self.auto_lift = false;
                    self.pending = None;
                    match self.lev {
                        L::Lifting => land_req = true,
                        L::LevitatedIdle | L::Moving | L::Stopping => {
                            if self.rest_policy == RestPolicy::Seat && !at_closed {
                                // DRAWER "land" means seat: close first (F17).
                                self.pending_from_intent = false;
                                self.pending = Some(MoveReq {
                                    target: self.x_closed,
                                    kind: MoveKind::Command,
                                });
                                if self.lev == L::Moving {
                                    stop_req = true;
                                    self.stop_kind = StopKind::User;
                                }
                            } else {
                                self.land_pending = true;
                            }
                        }
                        L::Seismic if self.sub != Sub::Down => self.sub = Sub::Land,
                        _ => {}
                    }
                }
                UserCommand::Manual => {
                    if matches!(self.lev, L::Boot | L::SelfTest | L::FaultReaction) {
                        self.reject(now, &c, RejectReason::State);
                    } else {
                        self.ev(now, EventKind::Command, a_idx, b_val);
                        self.manual_pending = true;
                        self.pending = None;
                    }
                }
                UserCommand::Reset | UserCommand::HoldToRun(_) => {
                    self.ev(now, EventKind::Command, a_idx, b_val);
                }
            }
        }

        // ---- Intent, hand-released capture, hover timeout, hold-to-run release ------
        let intent_allowed = normal && htr_ok && demand < SafetyDemand::SafeStop;
        if s.intent.is_confirmed() && s.intent != self.prev_intent {
            let dir = s.intent.dir();
            let sign = dir.map_or(0.0, |d| d.sign());
            self.ev(now, EventKind::Intent, s.f_hand_hat_n, sign);
            let target = match dir {
                Some(TravelDir::Close) => self.x_closed,
                _ => self.x_open,
            };
            let ok_state = match self.lev {
                L::Seated => self.cfg.push_to_open && dir == Some(TravelDir::Open),
                L::Landed => self.cfg.intent_from_rest,
                L::LevitatedIdle => true,
                _ => false,
            };
            if intent_allowed
                && ok_state
                && self.pending.is_none()
                && !(rest_state && reaction.refuse_lift)
            {
                self.pending = Some(MoveReq {
                    target,
                    kind: MoveKind::Command,
                });
                self.pending_from_intent = true;
            }
        }
        if self.pending.is_none() {
            self.pending_from_intent = false;
        }
        if self.lev == L::LevitatedIdle {
            if s.hand || !stopped || s.intent != IntentFlag::None {
                self.idle_since = now;
            }
            // Hand-released capture: log-damped terminus from the measured state.
            let v = s.v_mps;
            if intent_allowed
                && self.pending.is_none()
                && !s.hand
                && fmath::absf(v) >= self.cfg.capture_v_mps
                && self.cfg.capture_v_mps > 0.0
            {
                let target = if v < 0.0 { self.x_closed } else { self.x_open };
                let d = fmath::absf(target.diff_m(s.x));
                let tau = self.modes.term.tau_eff(fmath::absf(v));
                let (_, _, d0) = modes::terminus_blend(v, self.modes.j_term, tau);
                if d <= fmath::fmaxf(d0, self.cfg.capture_m) && d > self.modes.tol_for(target) {
                    self.pending_from_intent = false;
                    self.pending = Some(MoveReq {
                        target,
                        kind: MoveKind::Capture,
                    });
                }
            }
            if self.cfg.t_hover > 0 && now.since(self.idle_since) >= self.cfg.t_hover {
                land_req = true; // land in place (SEAT class at the seat: seat)
            }
            // D7: a DRAWER lifted out of its seat but never driven out re-seats after
            // SLOT_IDLE_RESEAT_S idle in the slot (Land inside the slot is the seat step).
            if self.rest_policy == RestPolicy::Seat
                && s.seat == SeatStatus::InSlot
                && self.pending.is_none()
                && self.cfg.t_slot_idle > 0
                && now.since(self.idle_since) >= self.cfg.t_slot_idle
            {
                land_req = true;
            }
            if !supply.holdup_ok {
                // Hold-up below 3× need while levitated: land gracefully (SAF-009).
                land_req = true;
                self.auto_lift = self.rest_policy == RestPolicy::Hover;
            }
        }
        if self.lev == L::Moving && !htr_ok {
            stop_req = true;
            self.stop_kind = StopKind::User;
            self.pending = None;
        }
        if self.lev == L::Moving && s.obstruction {
            stop_req = true;
            self.stop_kind = StopKind::Obstruction;
            self.pending = None;
        }
        if normal
            && self.prev_seis == SeismicState::Inspect
            && matches!(self.lev, L::Seated | L::Landed | L::Manual)
        {
            self.reselftest = true; // restore needs INSPECT reset + selftest (plan §4b)
        }
        let escape = (i.safety_in.fire_alarm && self.egress == EgressRole::NonRated)
            || (!normal && action.action == SeismicAction::OpenAndStop);

        // ---- LevState transition ------------------------------------------------------
        let arrived = self.modes.arrived(s, now, self.cfg.stall_v_mps);
        // An obstruction reversal completes when its 50 mm reverse move has arrived.
        let stop_done = if self.lev == L::Stopping
            && self.stop_kind == StopKind::Obstruction
            && self.modes.plan.active
        {
            arrived || self.modes.overdue(now)
        } else {
            stopped
        };
        let li = LevInputs {
            cmd: i.cmd,
            phase: s.lev,
            seat: s.seat,
            demand,
            faults: reported,
            selftest_ok,
            crc_ok: self.crc_ok,
            referenced: s.referenced,
            seis: r.state,
            arrived,
            holdup_ok: supply.holdup_ok,
            fast_valid: self.fast_seen,
            fresh,
            action: action.action,
            lift_request: lift_req
                || self.retry_lift
                || (self.auto_lift && self.rest_policy == RestPolicy::Hover),
            // An intent-triggered move waits for the hand to let go (push-and-go): while
            // gripped, the admittance follows the hand (W1.5).
            move_request: self.pending.is_some() && !(self.pending_from_intent && s.hand),
            stop_request: stop_req,
            land_request: land_req || self.land_pending,
            manual_request: self.manual_pending,
            escape,
            stopped: stop_done,
            at_closed,
            timeout,
            selftest_request: self.reselftest
                && matches!(self.lev, L::Seated | L::Landed | L::Manual),
        };
        let old = self.lev;
        let next = levstate::transition(old, &li, self.rest_policy);
        if old == L::Moving && next != L::Moving && arrived {
            self.ev(now, EventKind::Arrived, fmath::absf(s.v_mps), 0.0);
        }
        if next != old {
            self.enter(next, old, s, i, &action, &li, sensed, act);
        }

        // ---- Sub-sequences ------------------------------------------------------------
        let research_ok = |a: SeismicAction, m: &modes::ModePlanner, inertial: bool| match a {
            SeismicAction::Isolate => m.isolation_enable,
            SeismicAction::Damp => m.amd_enable,
            SeismicAction::RideThrough => inertial,
            _ => false,
        };
        let no_x = reaction.latched.contains(FaultCode::EncoderLost)
            || reaction.latched.contains(FaultCode::EncoderJump)
            || reaction.latched.contains(FaultCode::NonFinite);
        match self.lev {
            L::FaultReaction => {
                // STO on the LSM, or no trustworthy x feedback: no braking or seat-first
                // move is possible; land immediately (the carriage coasts on its rollers).
                if (demand >= SafetyDemand::StoLsm || no_x)
                    && matches!(self.sub, Sub::Brake | Sub::Move)
                {
                    self.modes.abort_move();
                    self.sub = Sub::Land;
                }
                match self.sub {
                    Sub::Brake if stopped => {
                        if at_closed && self.rest_policy == RestPolicy::Seat {
                            self.sub = Sub::Seat;
                        } else if self.seat_first_ok(s, &reaction, demand) {
                            self.begin_move(
                                MoveKind::SeatFirst,
                                self.x_closed,
                                0.0,
                                true,
                                s,
                                sensed,
                                act,
                            );
                            self.sub = Sub::Move;
                        } else {
                            self.sub = Sub::Land;
                        }
                    }
                    Sub::Move => {
                        if self.modes.arrived(s, now, self.cfg.stall_v_mps) {
                            self.modes.abort_move();
                            self.sub = Sub::Seat;
                        } else if self.modes.overdue(now) || obstruction_edge {
                            self.modes.abort_move();
                            self.sub = Sub::Land;
                        }
                    }
                    Sub::Seat if fresh && s.seat == SeatStatus::Failed => self.sub = Sub::Land,
                    _ => {}
                }
            }
            L::Seismic => {
                if obstruction_edge && self.sub == Sub::Move {
                    // One attempt only: the device is not secured, no re-close (plan §4b).
                    self.obstructed = true;
                    self.modes.abort_move();
                    self.sub = Sub::Brake;
                }
                let target_ok = !self.obstructed
                    && !reaction.no_auto_motion
                    && !reaction.hold_to_run
                    && s.referenced;
                match self.sub {
                    Sub::Brake | Sub::Hold if self.sub == Sub::Hold || stopped => {
                        let n = self.after_stop(&action, target_ok, at_closed, s, sensed, act);
                        let n = if n == Sub::Research
                            && !research_ok(action.action, &self.modes, self.cfg.inertial_enable)
                        {
                            Sub::Hold
                        } else {
                            n
                        };
                        if self.sub == Sub::Brake || n != Sub::Hold {
                            self.sub = n;
                        }
                    }
                    Sub::Research => {
                        if !research_ok(action.action, &self.modes, self.cfg.inertial_enable) {
                            self.sub = Sub::Brake;
                        }
                    }
                    Sub::Move => {
                        if self.modes.arrived(s, now, self.cfg.stall_v_mps) {
                            self.modes.abort_move();
                            self.sub = if self.rest_policy == RestPolicy::Seat && at_closed {
                                Sub::Seat
                            } else {
                                match action.action {
                                    SeismicAction::RideThrough => {
                                        if self.cfg.inertial_enable {
                                            Sub::Research
                                        } else {
                                            Sub::Hold
                                        }
                                    }
                                    SeismicAction::Freeze | SeismicAction::None => Sub::Hold,
                                    _ => Sub::Land,
                                }
                            };
                        } else if self.modes.overdue(now) {
                            self.modes.abort_move();
                            self.sub = Sub::Brake;
                            self.obstructed = true;
                        }
                    }
                    Sub::Seat if fresh && s.seat == SeatStatus::Failed => self.sub = Sub::Land,
                    _ => {}
                }
            }
            L::Escape => match self.sub {
                Sub::Brake if stopped => {
                    if self.escape_opened || self.obstructed {
                        self.sub = Sub::Land;
                    } else {
                        self.begin_move(MoveKind::Escape, self.x_open, 0.0, true, s, sensed, act);
                        self.sub = Sub::Move;
                    }
                }
                Sub::Move => {
                    let late = action.deadline.is_some_and(|dl| now >= dl);
                    if self.modes.arrived(s, now, self.cfg.stall_v_mps) {
                        self.modes.abort_move();
                        self.escape_opened = true;
                        self.sub = Sub::Land;
                    } else if self.modes.overdue(now) || obstruction_edge || late {
                        self.modes.abort_move();
                        self.escape_opened = true;
                        self.obstructed |= obstruction_edge;
                        self.sub = Sub::Brake;
                    }
                }
                _ => {}
            },
            L::Moving => {
                if let Some((d_b, tau)) = self.modes.update(act) {
                    self.ev(now, EventKind::TerminusCapture, d_b, tau);
                }
            }
            _ => {}
        }

        // ---- Levitation and propulsion commands for the state ------------------------
        let land = LevDirective::Land {
            rate_mps: self.cfg.land_rate_mps,
        };
        let mut unattended = false;
        let (lev_cmd, motion) = match self.lev {
            L::Boot | L::SelfTest | L::Seated | L::Landed | L::Fault | L::Manual => {
                (LevDirective::Off, MotionCmd::Off)
            }
            L::Lifting => {
                let lev = if self.cfg.parking_lock && i.safety_in.lock_engaged {
                    LevDirective::Hold // wait for the parking lock to release
                } else if self.unseat {
                    LevDirective::Unseat
                } else {
                    LevDirective::Lift
                };
                (lev, MotionCmd::Hold)
            }
            L::LevitatedIdle => {
                let m = if htr_ok {
                    MotionCmd::Admittance
                } else {
                    MotionCmd::Hold
                };
                (LevDirective::Levitate, m)
            }
            L::Moving => (LevDirective::Levitate, MotionCmd::Move),
            L::Stopping => {
                let m = match self.stop_kind {
                    StopKind::Obstruction if self.cfg.reverse_m > 0.0 && self.modes.plan.active => {
                        MotionCmd::Reverse
                    }
                    StopKind::Obstruction if self.cfg.reverse_m <= 0.0 => MotionCmd::Off,
                    _ => MotionCmd::Hold,
                };
                (LevDirective::Levitate, m)
            }
            L::Landing => (land, MotionCmd::Hold),
            L::Seating => (LevDirective::Seat, MotionCmd::Hold),
            L::FaultReaction => {
                let sto = demand >= SafetyDemand::StoLsm;
                match self.sub {
                    Sub::Down => (LevDirective::Off, MotionCmd::Off),
                    Sub::Brake => (
                        LevDirective::Levitate,
                        if sto {
                            MotionCmd::Off
                        } else {
                            MotionCmd::Freeze
                        },
                    ),
                    Sub::Move => {
                        unattended = true;
                        (LevDirective::Levitate, MotionCmd::Move)
                    }
                    Sub::Seat => (LevDirective::Seat, MotionCmd::Hold),
                    _ => (
                        land,
                        if sto || no_x {
                            MotionCmd::Off
                        } else {
                            MotionCmd::Hold
                        },
                    ),
                }
            }
            L::Seismic => {
                unattended = true;
                match self.sub {
                    Sub::Brake => (LevDirective::Levitate, MotionCmd::Freeze),
                    Sub::Move => (LevDirective::Levitate, MotionCmd::Move),
                    Sub::Hold | Sub::Down => (LevDirective::Levitate, MotionCmd::Hold),
                    Sub::Research => (
                        LevDirective::Levitate,
                        match action.action {
                            SeismicAction::Isolate => MotionCmd::Isolate,
                            SeismicAction::Damp => MotionCmd::Damp,
                            _ => MotionCmd::InertialSmooth,
                        },
                    ),
                    Sub::Land => (land, MotionCmd::Hold),
                    Sub::Seat => (LevDirective::Seat, MotionCmd::Hold),
                }
            }
            L::Escape => {
                unattended = true;
                match self.sub {
                    Sub::Move => (LevDirective::Levitate, MotionCmd::Move),
                    Sub::Land | Sub::Down => (land, MotionCmd::Hold),
                    _ => (LevDirective::Levitate, MotionCmd::Hold),
                }
            }
        };

        // ---- Directive ----------------------------------------------------------------
        let intent_state = match self.lev {
            L::LevitatedIdle => true,
            L::Landed => self.cfg.intent_from_rest,
            L::Seated => self.cfg.push_to_open,
            _ => false,
        };
        let mi = modes::ModeInputs {
            tick: now,
            lev: self.lev,
            seis: r.state,
            action,
            cmd: i.cmd,
            fast: *s,
            hold_to_run: htr,
            sensed,
            floor_v_mps: r.floor_v_mps,
            act,
            lev_cmd,
            motion,
            hold_to_run_required: htr_required,
            unattended,
            intent_enable: intent_state && intent_allowed,
            push_to_open: self.cfg.push_to_open && self.lev == L::Seated && intent_allowed,
        };
        let mut d = self.modes.directive(&mi);
        d.seq = self.seq;
        if self.modes.plan.seg_new {
            self.modes.plan.seg_seq = self.seq;
            self.modes.plan.seg_new = false;
        }

        // ---- Aux, veto, events, status ------------------------------------------------
        let rest_lock = self.rest_policy == RestPolicy::LandLock || self.lock_request;
        let aux = AuxCmd {
            parking_lock: self.cfg.parking_lock
                && rest_lock
                && matches!(self.lev, L::Landed | L::Fault),
            seismic_latch: self.cfg.seismic_latch && self.lev == L::Seated && !normal,
        };
        let veto = matches!(
            lev_cmd,
            LevDirective::Lift
                | LevDirective::Unseat
                | LevDirective::Land { .. }
                | LevDirective::Seat
        ) || (matches!(
            motion,
            MotionCmd::Move | MotionCmd::Reverse | MotionCmd::Freeze | MotionCmd::Hold
        ) && fmath::absf(s.a_mps2) >= self.cfg.veto_a_mps2);
        if r.state != self.prev_seis {
            self.ev(
                now,
                EventKind::SeismicState,
                f32::from(r.state.as_u8()),
                f32::from(self.prev_seis.as_u8()),
            );
            if r.state == SeismicState::Protect {
                let eta = r.s_eta.map_or(-1.0, |t| t.since_s(now));
                self.ev(now, EventKind::Protect, f32::from(r.level.as_u8()), eta);
            }
        }
        if d.mode != self.prev_mode {
            self.ev(
                now,
                EventKind::Mode,
                f32::from(d.mode.as_u8()),
                f32::from(self.prev_mode.as_u8()),
            );
        }
        if demand != self.prev_demand {
            self.ev(
                now,
                EventKind::Demand,
                f32::from(demand.as_u8()),
                f32::from(self.prev_demand.as_u8()),
            );
        }
        self.prev_intent = s.intent;
        self.prev_obstruction = s.obstruction;
        self.prev_seis = r.state;
        self.prev_mode = d.mode;
        self.prev_demand = demand;

        let st = SupStatus {
            tick: now,
            seq: self.seq,
            lev: self.lev,
            seis: r.state,
            mode: d.mode,
            action: action.action,
            action_overridden: outcome == PolicyOutcome::Admitted,
            demand,
            faults: reported,
            aux,
            veto,
            inspect_reset,
            hold_to_run: htr,
            presence: i.safety_in.presence_active,
            mains_ok: supply.mains_ok,
            v_bus_v: supply.v_bus_v,
            v_hold_v: supply.v_hold_v,
            e_hold_j: supply.e_hold_j,
        };
        (d, st)
    }

    /// DRAWER seat-first SS1 from the standoff (plan §4b): only for a SEAT class within
    /// the standoff of the seat, referenced, with the LSM available (demand ≤ SafeLand)
    /// and every latched fault of severity ≥ SafeStop a supply fault (MainsLost,
    /// BusUnder); never after an e-stop or with automatic motion forbidden.
    fn seat_first_ok(
        &self,
        s: &FastStatus,
        reaction: &faults::FaultReaction,
        demand: SafetyDemand,
    ) -> bool {
        if self.rest_policy != RestPolicy::Seat
            || demand > SafetyDemand::SafeLand
            || self.estop_latched
            || reaction.no_auto_motion
            || !s.referenced
            || fmath::absf(self.x_closed.diff_m(s.x)) > self.cfg.standoff_m
        {
            return false;
        }
        reaction.latched.iter().all(|c| {
            c.severity() < Severity::SafeStop
                || matches!(c, FaultCode::MainsLost | FaultCode::BusUnder)
        })
    }

    /// Plans a move at the class caps (unattended limit when `unattended`), with
    /// `v_cap` (> 0) as an additional cap.
    #[allow(clippy::too_many_arguments)]
    fn begin_move(
        &mut self,
        kind: MoveKind,
        target: PosNm,
        v_cap: f32,
        unattended: bool,
        s: &FastStatus,
        sensed: bool,
        act: Tick,
    ) {
        let (vo, vc, a) = self.modes.caps(sensed, unattended);
        let mut v = if target >= s.x { vo } else { vc };
        if v_cap > 0.0 {
            v = fmath::fminf(v, v_cap);
        }
        let dir = if target >= s.x {
            TravelDir::Open
        } else {
            TravelDir::Close
        };
        v = self.modes.governed_speed(v, dir, s.m_hat_kg, a);
        let start = MoveStart {
            kind,
            target,
            v_cap_mps: v,
            a_mps2: a,
            unattended,
            act,
            t_meas: s.tick,
            x: s.x,
            v_mps: s.v_mps,
            a_meas_mps2: s.a_mps2,
        };
        if let Some((d_b, tau)) = self.modes.start_move(&start) {
            self.events.push(Event {
                tick: act,
                kind: EventKind::TerminusCapture,
                a: d_b,
                b: tau,
            });
        }
        if kind == MoveKind::Reverse {
            self.events.push(Event {
                tick: act,
                kind: EventKind::Reversal,
                a: 0.0,
                b: 0.0,
            });
        }
    }

    /// SEISMIC: what to do once stopped (or re-evaluated while holding).
    fn after_stop(
        &mut self,
        dec: &ActionDecision,
        target_ok: bool,
        at_closed: bool,
        s: &FastStatus,
        sensed: bool,
        act: Tick,
    ) -> Sub {
        if let Some(tgt) = dec.target {
            let tgt = self.modes.clamp_target(tgt);
            if target_ok && fmath::absf(tgt.diff_m(s.x)) > self.modes.tol_for(tgt) {
                self.begin_move(MoveKind::Seismic, tgt, dec.v_cap_mps, true, s, sensed, act);
                return Sub::Move;
            }
        }
        if self.rest_policy == RestPolicy::Seat && at_closed {
            return Sub::Seat;
        }
        match dec.action {
            SeismicAction::Land
            | SeismicAction::Lock
            | SeismicAction::Center
            | SeismicAction::CompleteClose
            | SeismicAction::OpenAndStop => Sub::Land,
            SeismicAction::Isolate | SeismicAction::Damp | SeismicAction::RideThrough => {
                Sub::Research
            }
            SeismicAction::Freeze | SeismicAction::None | SeismicAction::CloseAfterShaking => {
                Sub::Hold
            }
        }
    }

    /// Entry actions of `next` (events, timers, move planning).
    #[allow(clippy::too_many_arguments)]
    fn enter(
        &mut self,
        next: LevState,
        old: LevState,
        s: &FastStatus,
        i: &SupInputs,
        action: &ActionDecision,
        li: &LevInputs,
        sensed: bool,
        act: Tick,
    ) {
        use LevState as L;
        let now = i.tick;
        self.ev(
            now,
            EventKind::LevState,
            f32::from(next.as_u8()),
            f32::from(old.as_u8()),
        );
        let t_in = now.since_s(self.entered);
        self.lev = next;
        self.entered = now;
        self.state_seq = self.seq;
        if old == L::Lifting && matches!(next, L::LevitatedIdle | L::Moving) {
            self.lift_attempts = 0;
            if self.unseat {
                let i_peak = s
                    .coil_i_a
                    .iter()
                    .fold(0.0f32, |m, &c| fmath::fmaxf(m, fmath::absf(c)));
                self.ev(now, EventKind::Unseated, s.m_hat_kg, i_peak);
            } else {
                self.ev(now, EventKind::Lifted, t_in, 0.0);
            }
        }
        if old == L::SelfTest {
            self.selftest_passed = li.crc_ok && li.selftest_ok;
            self.ev(
                now,
                EventKind::Selftest,
                if self.selftest_passed { 1.0 } else { 0.0 },
                0.0,
            );
            if self.rest_policy == RestPolicy::Hover && next == L::Landed {
                self.auto_lift = true; // lift once hold-up and the alert state allow
            }
        }
        match next {
            L::Boot => {}
            L::SelfTest => {
                self.reselftest = false;
                self.modes.abort_move();
            }
            L::Lifting => {
                self.unseat = old == L::Seated;
                self.retry_lift = false;
                self.auto_lift = false;
                self.lift_t0 = None;
                self.lock_request = false;
                self.modes.abort_move();
            }
            L::LevitatedIdle => {
                self.idle_since = now;
                self.modes.abort_move();
            }
            L::Moving => {
                self.stop_kind = StopKind::User;
                let req = self.pending.take().unwrap_or(MoveReq {
                    target: s.x,
                    kind: MoveKind::Command,
                });
                self.pending_from_intent = false;
                self.begin_move(req.kind, req.target, 0.0, false, s, sensed, act);
            }
            L::Stopping => {
                self.modes.abort_move();
                if self.stop_kind == StopKind::Obstruction && self.cfg.reverse_m > 0.0 {
                    // Reverse 50 mm away from the obstruction (DOOR/WALL; SAF-006).
                    let back = if s.v_mps > 0.0 {
                        -self.cfg.reverse_m
                    } else {
                        self.cfg.reverse_m
                    };
                    let target = self.modes.clamp_target(s.x.offset_m(back));
                    self.begin_move(MoveKind::Reverse, target, 0.0, true, s, sensed, act);
                }
            }
            L::Landing => {
                self.land_pending = false;
                self.modes.abort_move();
            }
            L::Seating => {
                self.land_pending = false;
                self.modes.abort_move();
            }
            L::Seated => {
                self.modes.abort_move();
                self.ev(now, EventKind::Seated, 0.0, 0.0);
            }
            L::Landed => {
                self.modes.abort_move();
                if matches!(old, L::Seismic | L::Escape) {
                    let lock = action.action == SeismicAction::Lock
                        || (self.policy.lock_after_land
                            && matches!(
                                action.action,
                                SeismicAction::Land | SeismicAction::Center
                            ));
                    self.lock_request = lock && self.egress == EgressRole::None;
                }
                if matches!(old, L::Landing | L::Seismic | L::Escape) {
                    self.ev(now, EventKind::Landed, fmath::absf(s.v_mps), 0.0);
                }
            }
            L::FaultReaction => {
                self.pending = None;
                self.land_pending = false;
                // Down already: nothing to land. A carriage that was lifting is treated
                // as levitated (the lift may have started under LET) and lands.
                self.sub = if !is_levitated(old) {
                    Sub::Down
                } else if li.demand >= SafetyDemand::StoLsm {
                    Sub::Land
                } else {
                    Sub::Brake
                };
                self.modes.abort_move();
            }
            L::Fault => {
                self.modes.abort_move();
            }
            L::Seismic => {
                self.pending = None;
                self.obstructed = false;
                let p = self.modes.plan;
                let continue_close = p.active
                    && p.dir == TravelDir::Close
                    && (action.action == SeismicAction::CompleteClose
                        || action.target == Some(p.target));
                if continue_close {
                    // A drawer already gliding closed continues to its seat under the
                    // unattended limits (plan §4b near-field variant).
                    self.modes.plan.unattended = true;
                    self.sub = Sub::Move;
                } else {
                    self.modes.abort_move();
                    self.sub = Sub::Brake;
                    self.ev(now, EventKind::Freeze, 0.0, 0.0);
                }
            }
            L::Escape => {
                self.pending = None;
                self.escape_opened = false;
                self.obstructed = false;
                self.modes.abort_move();
                self.sub = Sub::Brake;
            }
            L::Manual => {
                self.manual_pending = false;
                self.pending = None;
                self.modes.abort_move();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::levstate::tests::{FakeFast, nominal_inputs, run};
    use super::*;
    use fabrail_params::presets;
    use fabrail_types::{SafetyVerdict, SeismicLevel};

    fn sup(p: &SystemParams) -> Supervisor {
        match Supervisor::new(p) {
            Ok(s) => s,
            Err(e) => panic!("supervisor: {e}"),
        }
    }

    /// The first directive is safe (propulsion off) and the core never lowers a monitor
    /// demand (SAF-016). (Default `SystemParams` are rejected by the terminus planner's
    /// range check, so a preset is used.)
    #[test]
    fn stub_directive_is_safe_and_demand_not_lowered() {
        let p = presets::door();
        let built = Supervisor::new(&p);
        assert!(built.is_ok(), "supervisor construction: {built:?}");
        let Ok(mut sup) = built else {
            return;
        };
        let i = SupInputs {
            verdict: SafetyVerdict {
                demand: SafetyDemand::StoLsm,
                ..SafetyVerdict::default()
            },
            ..SupInputs::default()
        };
        let (d, st) = sup.step(&FastStatus::default(), &i, &SeismicReport::default());
        assert_eq!(d.mode, MotionMode::Off);
        assert_eq!(d.seq, 1);
        assert!(
            st.demand >= SafetyDemand::StoLsm,
            "core never lowers the monitor demand"
        );
    }

    /// Boot reaches the class rest state: DRAWER SEATED, DOOR hovering (via LIFTING),
    /// WALL/HEAVY LANDED; HEAVY engages its parking lock at rest (PWR-001).
    #[test]
    fn boot_to_rest_state() {
        for (p, want) in [
            (presets::drawer(), LevState::Seated),
            (presets::door(), LevState::LevitatedIdle),
            (presets::wall(), LevState::Landed),
            (presets::heavy(), LevState::Landed),
        ] {
            let mut s = sup(&p);
            let mut ff = FakeFast::new(p.design.rest_policy);
            let mut t = Tick(0);
            let d = run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
            assert_eq!(s.lev, want, "{:?} rest state (PWR-001)", p.design.class);
            assert!(s.selftest_passed);
            if want == LevState::LevitatedIdle {
                assert_eq!(d.lev, LevDirective::Levitate);
                assert_eq!(d.mode, MotionMode::Admittance);
            } else {
                assert_eq!(d.lev, LevDirective::Off);
                assert_eq!(d.mode, MotionMode::Off);
            }
            let i = nominal_inputs(&p, &s, t);
            let (_, st) = s.step(&ff.s, &i, &SeismicReport::default());
            assert_eq!(
                st.aux.parking_lock,
                p.design.class == RailClass::Heavy,
                "{:?} lock at rest",
                p.design.class
            );
        }
    }

    // PayloadMismatch (plan §2b item 10; FAB-SAF-018 is verified in Python): m̂ >
    // 1.05·m_rated raises a Degrade fault; levitation continues, the demand stays None,
    // motion is refused without hold-to-run and accepted with it.
    #[test]
    fn payload_mismatch_degrades_to_hold_to_run() {
        let p = presets::door();
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
        assert_eq!(s.lev, LevState::LevitatedIdle);
        ff.s.m_hat_kg = 1.10 * p.design.carriage.m_rated_kg as f32; // +10 % step (LEV-004)
        let d = run(&mut s, &mut ff, &p, &mut t, 300, |_, _, _| {});
        assert!(
            s.faults.latched.contains(FaultCode::PayloadMismatch),
            "raised after the m̂ time constant (F18)"
        );
        assert_eq!(s.lev, LevState::LevitatedIdle, "levitation continues");
        assert_eq!(d.lev, LevDirective::Levitate);
        assert!(d.limits.hold_to_run, "hold-to-run regime");
        assert_eq!(d.limits.v_open_mps, 0.0, "no motion without hold-to-run");
        assert_eq!(d.mode, MotionMode::Hold);
        // Open without hold-to-run: rejected with NotHoldToRun.
        let i = SupInputs {
            cmd: Some(UserCommand::Open),
            ..nominal_inputs(&p, &s, t)
        };
        let (_, st) = s.step(&ff.s, &i, &SeismicReport::default());
        assert!(st.faults.contains(FaultCode::NotHoldToRun));
        assert_eq!(
            st.demand,
            SafetyDemand::None,
            "Degrade never demands a stop"
        );
        assert_eq!(s.lev, LevState::LevitatedIdle);
        let mut rejected = false;
        while let Some(e) = s.events.pop() {
            rejected |= e.kind == EventKind::Rejected;
        }
        assert!(rejected, "rejection logged");
        // With hold-to-run actuated the move is accepted.
        let i = SupInputs {
            cmd: Some(UserCommand::Open),
            ..nominal_inputs(&p, &s, t)
        };
        let mut i = i;
        i.safety_in.hold_to_run = true;
        let (d, st) = s.step(&ff.s, &i, &SeismicReport::default());
        assert_eq!(s.lev, LevState::Moving);
        assert!(st.hold_to_run && d.limits.v_open_mps > 0.0);
        // Releasing hold-to-run stops the move.
        let i = nominal_inputs(&p, &s, t.plus(20));
        s.step(&ff.s, &i, &SeismicReport::default());
        assert_eq!(s.lev, LevState::Stopping);
    }

    /// Mains loss while hovering: SS1 Safe Landing under power (levitation LAND, never
    /// coils Off while levitated), STO on the LSM only after touchdown, then FAULT.
    #[test]
    fn mains_loss_lands_under_power() {
        let p = presets::door();
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
        let mut saw_land = false;
        for _ in 0..15 {
            let mut i = nominal_inputs(&p, &s, t);
            i.power.mains_ok = false;
            let (d, st) = s.step(&ff.s, &i, &SeismicReport::default());
            if s.lev == LevState::FaultReaction {
                assert_ne!(d.lev, LevDirective::Off, "coils stay energised (C11)");
                saw_land |= matches!(d.lev, LevDirective::Land { .. });
                assert!(st.faults.contains(FaultCode::MainsLost));
            }
            ff.run(&d);
            t = t.plus(20);
        }
        assert!(saw_land, "SS1 Safe Landing");
        assert_eq!(s.lev, LevState::Fault);
    }

    /// INSPECT gate (plan §4b): after a seismic landing, lift and moves are rejected
    /// until the INSPECT manual reset; the reset is forwarded and a SELFTEST runs before
    /// the rest state is restored.
    #[test]
    fn inspect_gates_restore() {
        let p = presets::door();
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
        assert_eq!(s.lev, LevState::LevitatedIdle);
        let mut rep = SeismicReport {
            state: SeismicState::Protect,
            level: SeismicLevel::L3,
            protect: true,
            local_p: true,
            ..SeismicReport::default()
        };
        // Land-first, independent of the class table's state: a policy proposing Land
        // (admitted: it does not move the carriage along x).
        struct LandPolicy;
        impl ActionPolicy for LandPolicy {
            fn propose(&mut self, _: &ActionContext) -> Option<ActionDecision> {
                Some(ActionDecision::of(SeismicAction::Land))
            }
        }
        let mut entered_seismic = false;
        for _ in 0..20 {
            let i = nominal_inputs(&p, &s, t);
            let (d, st) = s.step_with_policy(&ff.s, &i, &rep, &mut LandPolicy);
            assert_eq!(st.action, SeismicAction::Land);
            entered_seismic |= s.lev == LevState::Seismic;
            ff.run(&d);
            t = t.plus(20);
        }
        assert!(
            entered_seismic,
            "alert takes the hovering door into SEISMIC"
        );
        assert_eq!(s.lev, LevState::Landed, "land-first");
        rep.state = SeismicState::Inspect;
        for cmd in [UserCommand::Lift, UserCommand::Open] {
            let i = SupInputs {
                cmd: Some(cmd),
                ..nominal_inputs(&p, &s, t)
            };
            let (d, st) = s.step(&ff.s, &i, &rep);
            assert_eq!(s.lev, LevState::Landed, "{cmd:?} gated in INSPECT");
            assert_eq!(d.lev, LevDirective::Off);
            assert!(!st.inspect_reset);
        }
        let i = SupInputs {
            cmd: Some(UserCommand::Reset),
            ..nominal_inputs(&p, &s, t)
        };
        let (_, st) = s.step(&ff.s, &i, &rep);
        assert!(
            st.inspect_reset,
            "Reset in INSPECT is forwarded to the seismic unit"
        );
        rep.state = SeismicState::Normal;
        let i = nominal_inputs(&p, &s, t);
        s.step(&ff.s, &i, &rep);
        assert_eq!(s.lev, LevState::SelfTest, "selftest before restore");
        run(&mut s, &mut ff, &p, &mut t, 10, |_, _, _| {});
        assert_eq!(s.lev, LevState::LevitatedIdle, "DOOR hovers again");
    }

    /// A seated drawer never unseats on an alert, and engages nothing it does not have.
    #[test]
    fn seated_drawer_stays_seated_on_alert() {
        let p = presets::drawer();
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
        assert_eq!(s.lev, LevState::Seated);
        let rep = SeismicReport {
            state: SeismicState::Protect,
            ..SeismicReport::default()
        };
        for k in 0..10 {
            let mut i = nominal_inputs(&p, &s, t);
            if k == 2 {
                i.cmd = Some(UserCommand::Open);
            }
            let (d, _) = s.step(&ff.s, &i, &rep);
            assert_eq!(d.lev, LevDirective::Off);
            ff.run(&d);
            t = t.plus(20);
        }
        assert_eq!(s.lev, LevState::Seated);
    }

    /// Open then Close on a DRAWER: unseat → move → seat (the fake fast group reports
    /// arrival), with the events logged; the directive is deterministic.
    #[test]
    fn drawer_open_close_cycle() {
        let p = presets::drawer();
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
        let x_open = s.x_open;
        let x_closed = s.x_closed;
        run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.cmd = Some(UserCommand::Open)
        });
        assert_eq!(s.lev, LevState::Lifting);
        run(&mut s, &mut ff, &p, &mut t, 6, |_, _, _| {});
        assert_eq!(s.lev, LevState::Moving, "unseated, driving out");
        assert!(s.modes.plan.active && s.modes.plan.target == x_open);
        // The carriage arrives (fake): at the open end, at rest, fast arrived flag.
        run(&mut s, &mut ff, &p, &mut t, 3, |_, _, f| {
            f.x = x_open;
            f.arrived = true;
        });
        assert_eq!(s.lev, LevState::LevitatedIdle, "open drawer hovers");
        ff.s.arrived = false;
        run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.cmd = Some(UserCommand::Close)
        });
        assert_eq!(s.lev, LevState::Moving);
        run(&mut s, &mut ff, &p, &mut t, 3, |_, _, f| {
            f.x = x_closed;
            f.arrived = true;
            f.seat = SeatStatus::InSlot;
        });
        assert_eq!(s.lev, LevState::Seating, "seat step on arrival");
        run(&mut s, &mut ff, &p, &mut t, 6, |_, _, _| {});
        assert_eq!(s.lev, LevState::Seated);
        let mut kinds = [false; 25];
        while let Some(e) = s.events.pop() {
            if let Some(k) = kinds.get_mut(e.kind.as_u8() as usize) {
                *k = true;
            }
        }
        for k in [
            EventKind::LevState,
            EventKind::Command,
            EventKind::Unseated,
            EventKind::Arrived,
            EventKind::Seated,
            EventKind::Selftest,
        ] {
            assert!(kinds[k.as_u8() as usize], "{k:?} logged");
        }
    }

    /// D7: a DRAWER lifted out of its seat with no move re-seats after
    /// SLOT_IDLE_RESEAT_S (2 s) idle in the slot, not at the 300 s hover timeout; a
    /// pending move drives out instead.
    #[test]
    fn unmoved_unseat_reseats_after_idle() {
        let p = presets::drawer();
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
        run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.cmd = Some(UserCommand::Lift)
        });
        run(&mut s, &mut ff, &p, &mut t, 8, |_, _, _| {});
        assert_eq!(
            s.lev,
            LevState::LevitatedIdle,
            "unseated, holding in the slot"
        );
        assert_eq!(ff.s.seat, SeatStatus::InSlot);
        let p_sup = s.cfg.p_sup.max(1);
        let n_idle = (s.cfg.t_slot_idle / p_sup) as u32;
        run(&mut s, &mut ff, &p, &mut t, n_idle - 20, |_, _, _| {});
        assert_eq!(
            s.lev,
            LevState::LevitatedIdle,
            "still holding before 2 s idle (D7)"
        );
        run(&mut s, &mut ff, &p, &mut t, 40, |_, _, _| {});
        assert!(
            matches!(s.lev, LevState::Seating | LevState::Seated),
            "re-seated after 2 s idle in the slot (D7): {:?}",
            s.lev
        );
        run(&mut s, &mut ff, &p, &mut t, 10, |_, _, _| {});
        assert_eq!(s.lev, LevState::Seated);
        // A pending move drives out of the slot instead (Lifting → Moving).
        run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.cmd = Some(UserCommand::Open)
        });
        run(&mut s, &mut ff, &p, &mut t, 6, |_, _, _| {});
        assert_eq!(s.lev, LevState::Moving, "a pending move drives out (D7)");
    }

    /// Boots a DRAWER and parks it levitated at `x_m` from the seat (fake arrival).
    fn drawer_open_at(x_m: f32) -> (SystemParams, Supervisor, FakeFast, Tick) {
        let p = presets::drawer();
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
        let target = s.x_closed.offset_m(x_m);
        run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.cmd = Some(UserCommand::GoTo(target))
        });
        run(&mut s, &mut ff, &p, &mut t, 6, |_, _, _| {});
        assert_eq!(s.lev, LevState::Moving);
        run(&mut s, &mut ff, &p, &mut t, 3, |_, _, f| {
            f.x = target;
            f.arrived = true;
            f.seat = SeatStatus::Clear;
        });
        assert_eq!(s.lev, LevState::LevitatedIdle);
        ff.s.arrived = false;
        (p, s, ff, t)
    }

    /// Mains loss with the DRAWER at the standoff (30 mm ≤ 50 mm): SS1 is seat-first, a
    /// log-damped close to the seat under the unattended limits, then the z-seat step
    /// (plan §4b); at 200 mm it lands in place on the travel rollers (F17).
    #[test]
    fn seat_first_from_standoff_on_mains_loss() {
        let (p, mut s, mut ff, mut t) = drawer_open_at(0.03);
        let x_closed = s.x_closed;
        let mut saw_move = false;
        for _ in 0..4 {
            let mut i = nominal_inputs(&p, &s, t);
            i.power.mains_ok = false;
            let (d, _) = s.step(&ff.s, &i, &SeismicReport::default());
            if s.lev == LevState::FaultReaction && s.sub == Sub::Move {
                saw_move = true;
                assert_eq!(d.lev, LevDirective::Levitate);
                assert!(d.limits.unattended, "unattended limits (plan §4b)");
                assert!(d.limits.v_close_mps <= s.v_unattended_mps + 1e-6);
                assert_eq!(s.modes.plan.target, x_closed);
                assert_eq!(s.modes.plan.kind, MoveKind::SeatFirst);
            }
            ff.run(&d);
            t = t.plus(20);
        }
        assert!(saw_move, "seat-first move planned");
        let mut seated = false;
        for _ in 0..12 {
            let mut i = nominal_inputs(&p, &s, t);
            i.power.mains_ok = false;
            ff.s.x = x_closed;
            ff.s.arrived = true;
            let (d, _) = s.step(&ff.s, &i, &SeismicReport::default());
            seated |= d.lev == LevDirective::Seat;
            ff.run(&d);
            t = t.plus(20);
        }
        assert!(seated, "z-seat step after the seat-first move");
        assert_eq!(s.lev, LevState::Fault);
        assert_eq!(ff.s.seat, SeatStatus::Seated);

        let (p, mut s, mut ff, mut t) = drawer_open_at(0.2);
        for _ in 0..4 {
            let mut i = nominal_inputs(&p, &s, t);
            i.power.mains_ok = false;
            let (d, _) = s.step(&ff.s, &i, &SeismicReport::default());
            if s.lev == LevState::FaultReaction {
                assert!(
                    matches!(d.lev, LevDirective::Land { .. }),
                    "open 200 mm: land in place"
                );
            }
            ff.run(&d);
            t = t.plus(20);
        }
    }

    /// A fire alarm on a non-rated egress door fails open (ESCAPE): open move under
    /// unattended limits, then land; never locks.
    #[test]
    fn fire_alarm_fails_open_on_egress() {
        let p = presets::door();
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
        let x_open = s.x_open;
        let d = run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.safety_in.fire_alarm = true
        });
        assert_eq!(s.lev, LevState::Escape);
        assert_eq!(s.modes.plan.target, x_open);
        assert!(d.limits.unattended && d.lev == LevDirective::Levitate);
        let mut landed = false;
        for _ in 0..10 {
            let mut i = nominal_inputs(&p, &s, t);
            i.safety_in.fire_alarm = true;
            ff.s.x = x_open;
            ff.s.arrived = true;
            let (d, st) = s.step(&ff.s, &i, &SeismicReport::default());
            landed |= matches!(d.lev, LevDirective::Land { .. });
            assert!(!st.aux.parking_lock, "egress never locks (SEI-011)");
            ff.run(&d);
            t = t.plus(20);
        }
        assert!(landed);
        assert_eq!(s.lev, LevState::Landed, "manually operable after landing");
    }

    /// HEAVY unsensed is hold-to-run (F5); PL d presence sensing, healthy and with the
    /// zone clear, enables automatic motion at the sensed caps (SAF-003); a person in the
    /// zone returns it to hold-to-run.
    #[test]
    fn presence_gate_and_hold_to_run() {
        let mut p = presets::heavy();
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, _, _| {});
        assert_eq!(s.lev, LevState::Landed);
        run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.cmd = Some(UserCommand::Open)
        });
        assert_eq!(s.lev, LevState::Landed, "unsensed HEAVY needs hold-to-run");
        run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.cmd = Some(UserCommand::Open);
            i.safety_in.hold_to_run = true;
        });
        assert_eq!(s.lev, LevState::Lifting, "hold-to-run actuated");

        p.design.presence = PresenceSensing::PlD;
        crc::seal(&mut p);
        let mut s = sup(&p);
        let mut ff = FakeFast::new(p.design.rest_policy);
        let mut t = Tick(0);
        run(&mut s, &mut ff, &p, &mut t, 12, |_, i, _| {
            i.safety_in.presence_ok = true
        });
        run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.safety_in.presence_ok = true;
            i.safety_in.presence_active = true;
            i.cmd = Some(UserCommand::Open);
        });
        assert_eq!(s.lev, LevState::Landed, "person in the zone: hold-to-run");
        let d = run(&mut s, &mut ff, &p, &mut t, 1, |_, i, _| {
            i.safety_in.presence_ok = true;
            i.cmd = Some(UserCommand::Open);
        });
        assert_eq!(s.lev, LevState::Lifting, "PL d sensed: automatic");
        assert!(!d.limits.hold_to_run);
        let d = run(&mut s, &mut ff, &p, &mut t, 6, |_, i, _| {
            i.safety_in.presence_ok = true
        });
        assert_eq!(s.lev, LevState::Moving);
        assert!(
            (d.limits.v_open_mps - p.safety.v_open_sensed_mps as f32).abs() < 1e-6,
            "sensed cap 0.10 m/s"
        );
    }

    /// An admitted seismic target (closing) runs under the unattended limits;
    /// ObstructionDegraded forbids it and the drawer lands in place (plan §4b).
    #[test]
    fn seismic_target_move_and_obstruction_degraded() {
        struct SeatPolicy(PosNm);
        impl ActionPolicy for SeatPolicy {
            fn propose(&mut self, _: &ActionContext) -> Option<ActionDecision> {
                Some(ActionDecision {
                    action: SeismicAction::Land,
                    target: Some(self.0),
                    v_cap_mps: 10.0,
                    deadline: None,
                })
            }
        }
        let rep = SeismicReport {
            state: SeismicState::Protect,
            local_p: true,
            ..SeismicReport::default()
        };
        for degraded in [false, true] {
            let (p, mut s, mut ff, mut t) = drawer_open_at(0.2);
            let mut pol = SeatPolicy(s.x_closed);
            ff.s.obstruction_degraded = degraded;
            let mut d = Directive::default();
            let mut st = SupStatus::default();
            for _ in 0..3 {
                let i = nominal_inputs(&p, &s, t);
                (d, st) = s.step_with_policy(&ff.s, &i, &rep, &mut pol);
                ff.run(&d);
                t = t.plus(20);
            }
            assert_eq!(s.lev, LevState::Seismic);
            assert!(st.action_overridden, "admitted (closing, confirmed)");
            if degraded {
                assert!(st.faults.contains(FaultCode::ObstructionDegraded));
                assert_eq!(s.sub, Sub::Land, "no automatic motion when degraded");
                assert!(matches!(d.lev, LevDirective::Land { .. }));
            } else {
                assert_eq!(s.sub, Sub::Move);
                assert!(d.limits.unattended);
                assert!(s.modes.plan.v_cap_mps <= s.v_unattended_mps + 1e-6);
                assert!(!d.terminus.hand_yields, "a hand gives Hold outside NORMAL");
                assert!(!d.haptic.intent_enable, "intent disabled in SEISMIC");
            }
        }
    }

    /// Two supervisors fed the same inputs produce identical directives (determinism).
    #[test]
    fn deterministic() {
        let p = presets::wall();
        let mut a = sup(&p);
        let mut b = sup(&p);
        let mut fa = FakeFast::new(p.design.rest_policy);
        let mut fb = FakeFast::new(p.design.rest_policy);
        let (mut ta, mut tb) = (Tick(0), Tick(0));
        let script = |k: u32, i: &mut SupInputs, _: &mut FastStatus| {
            i.cmd = match k {
                15 => Some(UserCommand::Open),
                40 => Some(UserCommand::Stop),
                _ => None,
            };
        };
        let da = run(&mut a, &mut fa, &p, &mut ta, 60, script);
        let db = run(&mut b, &mut fb, &p, &mut tb, 60, script);
        assert_eq!(da, db);
        assert_eq!(a.lev, b.lev);
    }
}
