//! Frozen inter-group messages: Measurements, FastOutputs, FastStatus, Directive (Segment, terminus, inertial), SupInputs, SupStatus, SeismicReport. Owner: W0a.
//!
//! These are the only values that cross rate-group boundaries (core design §A.5, §B.4).
//! They travel through fabrail-rt's LET mailboxes: a job released at tick r reads values
//! whose activation tick is ≤ r and publishes values that activate at r + P (P = its
//! own period). Every type is `Copy`, allocation-free and has a `Default` that is the
//! safe "nothing commanded yet" value a mailbox holds before its first publish
//! (levitation off, propulsion STO, zero speed caps).
//!
//! Data flow per base tick (core design §A.5):
//!
//! ```text
//! base (20 kHz) ── FastRaw ──> frontend::convert ── Measurements ──> FastLoop::step
//!   ^                                                                  │  (10/5 kHz)
//!   └── FastOutputs (arbitrated with the monitor verdict) <────────────┤
//!                                                                      └─ FastStatus ─┐
//! SeismicUnit::step (100 Hz) ── SeismicReport ──> Supervisor::step (1 kHz) <─ SupInputs┘
//!                                                     │
//!                                                     └── Directive ──> FastLoop (next P)
//!                                                     └── SupStatus ──> rt, telemetry
//! ```
//!
//! Units follow the house rule: SI, f32 in the controller, unit suffix in the field name
//! (`_m`, `_mps`, `_mps2`, `_n`, `_a`, `_kg`, `_j`, `_s`, `_hz`, `_n_per_m`,
//! `_n_s_per_m`). Absolute positions are [`PosNm`] and times are [`Tick`], never f32
//! (PHYSICS §24). Frames per PHYSICS §1: x is travel (+x opens), y cross-track, z up.

use fabrail_types::{
    AlertSource, AuxCmd, CoilCmd, FaultSet, LevState, LsmCmd, MAX_MAGNETS, MotionMode, PosNm,
    PowerRaw, SafetyDemand, SafetyInRaw, SafetyVerdict, Segment, SeismicAction, SeismicLevel,
    SeismicState, Tick, TravelDir, UserCommand,
};

/// Defines a `#[repr(u8)]` wire enum with explicit discriminants plus `ALL`, `as_u8`,
/// `from_u8` and `name` (the same contract as the `fabrail-types` state enums, which
/// keep their macro private).
macro_rules! wire_enum {
    (
        $(#[$meta:meta])*
        pub enum $name:ident {
            $( $(#[$vmeta:meta])* $var:ident = $val:expr ),+ $(,)?
        }
    ) => {
        $(#[$meta])*
        #[repr(u8)]
        pub enum $name {
            $( $(#[$vmeta])* $var = $val ),+
        }

        impl $name {
            /// Every variant, in discriminant order.
            pub const ALL: &'static [$name] = &[$($name::$var),+];

            /// The wire value (telemetry, replay, PyO3).
            #[inline]
            pub const fn as_u8(self) -> u8 {
                self as u8
            }

            /// Decodes a wire value; `None` for an unknown discriminant.
            pub const fn from_u8(v: u8) -> Option<Self> {
                match v {
                    $( x if x == $val => Some($name::$var), )+
                    _ => None,
                }
            }

            /// The variant name (event logs, Python enums).
            pub const fn name(self) -> &'static str {
                match self {
                    $( $name::$var => stringify!($var), )+
                }
            }
        }
    };
}

// ---- Fast group input ---------------------------------------------------------------

/// Calibrated measurements for one fast-group release, produced from the base task's
/// latched [`fabrail_types::FastRaw`] by `frontend::convert` (PHYSICS §13). The safety
/// monitor never sees these: it reads the raw codes with its own calibration copies.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Measurements {
    /// Base tick at which the raw sample was latched.
    pub tick: Tick,
    /// Air gap per magnet (dual-probe voted), m. Unused elements read 0.
    pub gap_m: [f32; MAX_MAGNETS],
    /// Bit i set = gap channel i valid after range/NaN checks and the dual-probe vote.
    pub gap_ok: u8,
    /// Coil current per magnet, A.
    pub coil_i_a: [f32; MAX_MAGNETS],
    /// Absolute position: encoder counts × nm_per_count, integer (PHYSICS §24).
    pub x: PosNm,
    /// Encoder referenced (homed) since boot.
    pub referenced: bool,
    /// Encoder signal healthy (no loss, no implausible jump).
    pub enc_ok: bool,
    /// LSM quadrature current from Clarke/Park at the exact electrical angle, A (§7).
    pub iq_a: f32,
    /// LSM direct-axis current, A.
    pub id_a: f32,
    /// Carriage IMU acceleration (x, y, z), m/s². Skyhook and fusion only, never EEW
    /// (core design F16).
    pub imu_mps2: [f32; 3],
    /// Carriage IMU present and plausible.
    pub imu_ok: bool,
    /// Rail-mounted floor accelerometer (x, y, z), m/s²: the only floor feed-forward
    /// source for inertial smoothing (PHYSICS §16.2).
    pub rail_mps2: [f32; 3],
    /// Rail accelerometer present and plausible.
    pub rail_ok: bool,
}

// ---- Fast group outputs ------------------------------------------------------------

/// Actuator commands from one fast-group release, BEFORE (FastLoop output) or AFTER
/// (rt arbitration) the monitor verdict is applied. In voltage coil mode the base task's
/// `current::CurrentLoops` turns `CoilCmd::CurrentRef` into bridge voltages; in lag mode
/// the reference passes through and SimBoard applies the lag (PHYSICS §4).
///
/// `Default` is coils off and the LSM in STO: the safe value before the first release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FastOutputs {
    /// Levitation coil command.
    pub coil: CoilCmd,
    /// LSM command (i_q* = F*/K̂_f, PHYSICS §7).
    pub lsm: LsmCmd,
}

wire_enum! {
    /// Intent state from `haptics::intent` (force × time or displacement, with direction
    /// consistency; plan §2b item 11; HAP-005/006).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum IntentFlag {
        /// No hand input above threshold.
        #[default]
        None = 0,
        /// Force above threshold toward +x, persistence not yet met.
        PendingOpen = 1,
        /// Force above threshold toward −x, persistence not yet met.
        PendingClose = 2,
        /// Intent to open confirmed.
        Open = 3,
        /// Intent to close confirmed.
        Close = 4,
        /// Sustained force without motion: classified as a lean and fed to the bias
        /// estimator, never as intent (HAP-006).
        Lean = 5,
    }
}

impl IntentFlag {
    /// True for a confirmed intent.
    #[inline]
    pub const fn is_confirmed(self) -> bool {
        matches!(self, IntentFlag::Open | IntentFlag::Close)
    }

    /// Direction of a pending or confirmed intent.
    #[inline]
    pub const fn dir(self) -> Option<TravelDir> {
        match self {
            IntentFlag::PendingOpen | IntentFlag::Open => Some(TravelDir::Open),
            IntentFlag::PendingClose | IntentFlag::Close => Some(TravelDir::Close),
            IntentFlag::None | IntentFlag::Lean => None,
        }
    }
}

wire_enum! {
    /// DRAWER captive-seat status from `levitation::seat` (PHYSICS §15; core design F17;
    /// plan §2b item 4). Classes without a seat report `NoSeat`.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum SeatStatus {
        /// The class has no captive seat.
        #[default]
        NoSeat = 0,
        /// Resting in the captive seat (g_seat 3.5 mm, constrained in z both ways).
        Seated = 1,
        /// Current ramp until lift-off is sensed (> 20 µm); m̂ from the closed form.
        Unseating = 2,
        /// Levitated but still inside the seat slot (the LSM is driving out).
        InSlot = 3,
        /// Levitated and clear of the slot.
        Clear = 4,
        /// z-seat step into the seat (+0.2 s after the close terminus).
        Seating = 5,
        /// Seat step failed: landed on the travel rollers; manual push-to-seat needed.
        Failed = 6,
    }
}

wire_enum! {
    /// Levitation phase reported by the fast group so the supervisor can advance the
    /// LevState machine (PHYSICS §15, §18).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum LevPhase {
        /// Coils idle (resting on seat, rollers or skids).
        #[default]
        Off = 0,
        /// Lift or unseat in progress.
        Lifting = 1,
        /// Bumpless transfer into zero-power control (|δg| < 50 µm).
        Transfer = 2,
        /// Levitated under zero-power control.
        Levitated = 3,
        /// Controlled descent at the landing speed.
        Landing = 4,
        /// Touchdown detected (ġ ≈ 0 with a current step).
        Touchdown = 5,
        /// Lift or land did not finish within its time limit.
        Timeout = 6,
    }
}

/// Fast-group status, published every fast release for the supervisor, telemetry and
/// the recorder. `seq` doubles as the core heartbeat the monitor supervises (SAF-013).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FastStatus {
    /// Base tick of the fast release that produced this status.
    pub tick: Tick,
    /// Fast-release sequence number (wrapping); the core heartbeat.
    pub seq: u32,
    /// Sequence number of the `Directive` that was executed.
    pub directive_seq: u32,
    /// Motion mode actually executed (the fast group may hold on a hand or obstruction).
    pub mode: MotionMode,
    /// Measured position.
    pub x: PosNm,
    /// Encoder referenced (NotReferenced → hold-to-run until homed).
    pub referenced: bool,
    /// Estimated velocity, m/s.
    pub v_mps: f32,
    /// Estimated acceleration, m/s².
    pub a_mps2: f32,
    /// Reference position.
    pub x_ref: PosNm,
    /// Reference velocity, m/s.
    pub v_ref_mps: f32,
    /// Modal deviation from the set point: [heave m, pitch rad, roll rad] (PHYSICS §15).
    pub modal: [f32; 3],
    /// Air gap per magnet, m.
    pub gap_m: [f32; MAX_MAGNETS],
    /// Coil current per magnet, A.
    pub coil_i_a: [f32; MAX_MAGNETS],
    /// LSM quadrature current, A.
    pub iq_a: f32,
    /// Momentum-observer hand-force estimate F̂_h, N (PHYSICS §17).
    pub f_hand_hat_n: f32,
    /// Obstruction residual r = F̂ − m̂·a_floor,x, N (PHYSICS §17; plan §4b).
    pub residual_n: f32,
    /// Payload estimate m̂, kg (non-safety; the monitor uses rated mass only).
    pub m_hat_kg: f32,
    /// Governor kinetic energy ½·m_gov·v², J (PHYSICS §16).
    pub e_kin_gov_j: f32,
    /// Passivity observer energy, J (PHYSICS §17; HAP-009).
    pub e_passivity_j: f32,
    /// Rail floor acceleration along x, m/s².
    pub a_floor_x_mps2: f32,
    /// Intent state.
    pub intent: IntentFlag,
    /// Hand detected (admittance onset ≤ 10 ms, 20 ms on WALL; plan §4b).
    pub hand: bool,
    /// Obstruction detected (stop, then reverse or release).
    pub obstruction: bool,
    /// Obstruction threshold degraded by floor shaking beyond 2·th0 (plan §4b).
    pub obstruction_degraded: bool,
    /// Current segment complete and at target within the position tolerance.
    pub arrived: bool,
    /// Levitation phase.
    pub lev: LevPhase,
    /// DRAWER seat status.
    pub seat: SeatStatus,
    /// `SAT_*` bits.
    pub sat_flags: u8,
    /// Faults raised by the frontend and the fast group.
    pub faults: FaultSet,
}

impl FastStatus {
    /// A coil current reference hit the I²t clamp.
    pub const SAT_COIL: u8 = 1 << 0;
    /// The LSM force hit its limit (core or verdict cap).
    pub const SAT_LSM: u8 = 1 << 1;
    /// The energy/momentum/braking governor limited the reference speed.
    pub const SAT_GOVERNOR: u8 = 1 << 2;
    /// The directional force clamp limited the drive force.
    pub const SAT_CLAMP: u8 = 1 << 3;
    /// A stroke governor (isolation, inertial smoothing) limited the motion.
    pub const SAT_STROKE: u8 = 1 << 4;
    /// DRAWER extension beyond the contactless envelope (LEV-016; plan §2b item 3).
    pub const SAT_EXTENSION: u8 = 1 << 5;
}

// ---- Supervisor → fast directive ----------------------------------------------------

/// Levitation command to the fast group (PHYSICS §15; core design §B.4).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum LevDirective {
    /// Coils idle.
    #[default]
    Off,
    /// DRAWER: ramp current until lift-off from the captive seat, then drive out of the
    /// slot (F17).
    Unseat,
    /// Lift off the landing rollers (lift time ≤ 0.1/0.3/1/2 s by class, LEV-012).
    Lift,
    /// Zero-power levitation at the set point.
    Levitate,
    /// Controlled landing at `rate_mps` (5/5/3/2 mm/s by class), then idle.
    Land {
        /// Descent speed, m/s (> 0).
        rate_mps: f32,
    },
    /// DRAWER: z-seat step into the captive seat.
    Seat,
    /// Keep the present support state (seated, landed or levitated) with no transition.
    Hold,
}

/// Haptic admittance and effects profile (PHYSICS §17):
/// M_v·ẍ + B_v·ẋ + K_v·(x − x0) = F̂_h + F_eff.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct HapticProfile {
    /// Rendered (virtual) mass M_v = m/assist ratio, kg.
    pub m_v_kg: f32,
    /// Virtual damping B_v, N·s/m.
    pub b_v_n_s_per_m: f32,
    /// Virtual stiffness K_v (0 during a hand take-over of a terminus), N/m.
    pub k_v_n_per_m: f32,
    /// Spring anchor x0.
    pub x0: PosNm,
    /// Number of active detents (≤ 4).
    pub n_detents: u8,
    /// Detent positions.
    pub detents: [PosNm; 4],
    /// Detent peak force, N.
    pub detent_f_n: f32,
    /// Detent width, m.
    pub detent_width_m: f32,
    /// Legacy soft-close spring enabled (HAP-007).
    pub soft_close: bool,
    /// Passivity controller enabled (HAP-009).
    pub passivity: bool,
    /// Intent detection enabled (disabled outside SeismicState NORMAL; plan §4b).
    pub intent_enable: bool,
    /// Push-to-open from the seat enabled (DRAWER; HAP-012).
    pub push_to_open: bool,
}

/// Live motion limits chosen by the supervisor for the current mode, sensing and
/// seismic state (PHYSICS §16; SAF-001..004). `Default` is all zeros: no motion allowed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LiveLimits {
    /// Opening speed cap V_mode (+x), m/s.
    pub v_open_mps: f32,
    /// Closing speed cap V_mode (−x), m/s. The unattended-close limit
    /// min(v_cap, f_pinch/√(m_gov·k_series)) is applied here (plan §4b).
    pub v_close_mps: f32,
    /// Acceleration limit, m/s² (braking uses a_brk = 2·a_max).
    pub a_max_mps2: f32,
    /// Dynamic driving-force limit (≤ 0.75 s), N.
    pub f_drive_n: f32,
    /// Static (stall) force limit: 67 N unsensed, 150 N sensed (F4), N.
    pub f_static_n: f32,
    /// Closing force allowed in the last 50 mm, N.
    pub f_close_last50_n: f32,
    /// Governor observation point when opening (travel end, or terminus + overtravel).
    pub x_obs_open: PosNm,
    /// Governor observation point when closing.
    pub x_obs_close: PosNm,
    /// Motion only while hold-to-run is actuated (HEAVY unsensed; PayloadMismatch).
    pub hold_to_run: bool,
    /// Unattended automatic motion (choreographed seismic close): obstruction armed,
    /// one attempt only (plan §4b).
    pub unattended: bool,
}

/// Runtime configuration of the log-damped terminus (PHYSICS §16.1; plan §4b). The
/// planned trajectory itself is `Directive::segment` (`Segment::Terminus` /
/// `Segment::Quintic`); this block tells the fast group how to re-plan and hand over.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TerminusCfg {
    /// Nominal τ; the runtime uses τ_eff = max(τ, τ_min(v_measured)), s.
    pub tau_s: f32,
    /// Capture distance d_ε where the quintic takes over, m.
    pub d_eps_m: f32,
    /// Blend jerk, m/s³.
    pub j_mps3: f32,
    /// Quintic duration ratio T_q/τ.
    pub t_q_ratio: f32,
    /// Governor overtravel beyond the terminus target (x_obs = target + overtravel), m.
    pub overtravel_m: f32,
    /// Braking deceleration for a re-plan inside d_0 after a hand release
    /// (a_brk = 2·a_max), m/s².
    pub a_brk_replan_mps2: f32,
    /// Hand detection onset during capture (10 ms; 20 ms WALL), s.
    pub hand_onset_s: f32,
    /// A hand during capture switches to plain admittance (K_v = 0, passivity on) and
    /// re-plans on release. False outside SeismicState NORMAL: a hand gives Hold.
    pub hand_yields: bool,
    /// DRAWER: z-seat step on arrival (+0.2 s).
    pub seat_on_arrival: bool,
    /// Velocity integrator frozen with bumpless hold during capture (error poles
    /// (s + ω_v/2)², ζ = 1); re-enabled at d_ε.
    pub freeze_integrator: bool,
}

/// Runtime configuration of inertial smoothing (PHYSICS §16.2; plan §4b; research tier,
/// `enable = false` in every preset). Skyhook absolute-velocity feedback from the
/// carriage IMU, c_sky = 2ζ·m̂·ω_c, following only floor motion below f_c.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InertialCfg {
    /// Mode admitted by the supervisor (never true unless the preset enables it).
    pub enable: bool,
    /// Crossover f_c, Hz.
    pub f_c_hz: f32,
    /// Skyhook damping ratio ζ.
    pub zeta: f32,
    /// IMU leaky-integral high-pass, Hz.
    pub hp_hz: f32,
    /// Standoff position the smoother is centred on (s_min from the seat).
    pub standoff: PosNm,
    /// Stroke available about the standoff (standoff − margin), m.
    pub stroke_m: f32,
    /// Continuous force budget, N.
    pub f_cont_n: f32,
    /// Burst force allowance, N.
    pub f_burst_n: f32,
    /// Burst duration limit, s.
    pub t_burst_s: f32,
}

/// Research-tier isolation and active-mass-damping configuration (PHYSICS §21):
/// F = −k·x_rel − c_sky·ẋ_abs, saturated at `f_max_n` with a stroke governor; in
/// `MotionMode::Damp`, direct velocity feedback `c_dvf`. Replaced limits per SAF-020.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IsolateCfg {
    /// Centring stiffness k = m·(2π/T_iso)², N/m.
    pub k_n_per_m: f32,
    /// Skyhook damping c_sky, N·s/m.
    pub c_sky_n_s_per_m: f32,
    /// Isolation centre.
    pub center: PosNm,
    /// Stroke envelope about the centre, m.
    pub stroke_m: f32,
    /// Force saturation (F_cont), N.
    pub f_max_n: f32,
    /// AMD direct velocity feedback gain, N·s/m.
    pub c_dvf_n_s_per_m: f32,
}

/// Supervisor command to the fast group for the next supervisor period.
///
/// `Default` is levitation off, `MotionMode::Off` (STO), a hold segment at the origin
/// and zero limits: the safe value before the first supervisor release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Directive {
    /// Supervisor release sequence number (wrapping).
    pub seq: u32,
    /// Levitation command.
    pub lev: LevDirective,
    /// Motion mode.
    pub mode: MotionMode,
    /// Reference segment (S-curve, log-damped terminus or quintic), planned in closed
    /// form relative to its anchor/target and start tick (PHYSICS §16, §16.1).
    pub segment: Segment,
    /// Terminus re-plan and hand-over configuration.
    pub terminus: TerminusCfg,
    /// Inertial-smoothing configuration (research tier).
    pub inertial: InertialCfg,
    /// Haptic profile.
    pub haptic: HapticProfile,
    /// Live limits.
    pub limits: LiveLimits,
    /// Isolation / AMD configuration (research tier).
    pub isolate: IsolateCfg,
    /// Seismic state (obstruction degradation, hand → Hold outside NORMAL).
    pub seis: SeismicState,
    /// Low-frequency floor velocity estimate from the seismic unit, m/s.
    pub floor_v_mps: f32,
}

// ---- Supervisor inputs and status ---------------------------------------------------

/// Supervisor-group inputs gathered by fabrail-rt for one supervisor release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SupInputs {
    /// Base tick of the supervisor release.
    pub tick: Tick,
    /// Power-path sample.
    pub power: PowerRaw,
    /// Safety-related inputs.
    pub safety_in: SafetyInRaw,
    /// User/building command polled this release, if any.
    pub cmd: Option<UserCommand>,
    /// Combined monitor verdict (fast and slow checks) active at this release. The core
    /// can never lower its demand (SAF-016).
    pub verdict: SafetyVerdict,
}

/// Supervisor status published every supervisor release (rt, telemetry, recorder).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SupStatus {
    /// Base tick of the supervisor release.
    pub tick: Tick,
    /// Supervisor release sequence number (wrapping).
    pub seq: u32,
    /// Levitation / carriage state.
    pub lev: LevState,
    /// Seismic state as acted on by the supervisor.
    pub seis: SeismicState,
    /// Commanded motion mode.
    pub mode: MotionMode,
    /// Seismic action in force (after the `ActionPolicy` hook and its admission filter).
    pub action: SeismicAction,
    /// The action came from an admitted `ActionPolicy` override (plan §4).
    pub action_overridden: bool,
    /// Supervisor fault-reaction demand (FAULT_REACTION is SS1 Safe Landing).
    pub demand: SafetyDemand,
    /// Latched faults (supervisor, fast, seismic and monitor).
    pub faults: FaultSet,
    /// Auxiliary actuator command (parking lock, seismic latch).
    pub aux: AuxCmd,
    /// Actuator veto for the seismic detector (self-actuation vibration; SEI-005).
    pub veto: bool,
    /// Forward a manual INSPECT reset to the seismic unit this release.
    pub inspect_reset: bool,
    /// Hold-to-run actuated.
    pub hold_to_run: bool,
    /// Presence sensing reports a person in the zone.
    pub presence: bool,
    /// Mains present.
    pub mains_ok: bool,
    /// DC bus voltage, V.
    pub v_bus_v: f32,
    /// Hold-up (supercap) voltage, V.
    pub v_hold_v: f32,
    /// Usable hold-up energy, J (SAF-009: ≥ 3 × landing need).
    pub e_hold_j: f32,
}

// ---- Seismic unit output ------------------------------------------------------------

/// Seismic-unit report, published every seismic release (100 Hz; PHYSICS §20).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SeismicReport {
    /// Base tick of the seismic release.
    pub tick: Tick,
    /// Seismic release sequence number (wrapping).
    pub seq: u32,
    /// Seismic state (NORMAL → TRIGGERED → PROTECT → SHAKING → QUIET → INSPECT).
    pub state: SeismicState,
    /// Detector level from the Pd/τc matrix.
    pub level: SeismicLevel,
    /// PROTECT rule met: level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal (plan §2b item 6).
    pub protect: bool,
    /// Tick of the STA/LTA trigger (local P pick), if any.
    pub triggered_at: Option<Tick>,
    /// Peak displacement Pd, m.
    pub pd_m: f32,
    /// Average period τc, s.
    pub tau_c_s: f32,
    /// Peak ground acceleration seen, m/s².
    pub pga_mps2: f32,
    /// Predicted S-wave arrival, if known (local S−P or network origin + R/V_s).
    pub s_eta: Option<Tick>,
    /// Where the decision came from.
    pub source: AlertSource,
    /// A local P pick contributed (independent confirmation, plan §4).
    pub local_p: bool,
    /// Number of independent feeds contributing (local P counts as one).
    pub n_feeds: u8,
    /// The active alert was cancelled this release.
    pub cancel: bool,
    /// Low-frequency floor velocity estimate (x), m/s.
    pub floor_v_mps: f32,
    /// STA/LTA triggers since boot (including non-PROTECT ones).
    pub raw_triggers: u32,
    /// Faults raised by the seismic unit (AccelLost, AccelStuck, VerticalSaturation).
    pub faults: FaultSet,
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_types::LsmMode;

    #[test]
    fn defaults_are_safe() {
        let o = FastOutputs::default();
        assert_eq!(o.coil, CoilCmd::Off);
        assert_eq!(
            o.lsm.mode,
            LsmMode::Sto,
            "propulsion STO before the first release"
        );
        let d = Directive::default();
        assert_eq!(d.lev, LevDirective::Off);
        assert_eq!(d.mode, MotionMode::Off);
        assert_eq!(d.limits.v_open_mps, 0.0);
        assert_eq!(d.limits.v_close_mps, 0.0);
        assert!(!d.inertial.enable);
        assert_eq!(d.segment.end(), PosNm::ZERO);
        let r = SeismicReport::default();
        assert_eq!(r.state, SeismicState::Normal);
        assert!(!r.protect);
    }

    #[test]
    fn wire_enums_roundtrip() {
        for f in IntentFlag::ALL {
            assert_eq!(IntentFlag::from_u8(f.as_u8()), Some(*f));
        }
        for s in SeatStatus::ALL {
            assert_eq!(SeatStatus::from_u8(s.as_u8()), Some(*s));
        }
        for p in LevPhase::ALL {
            assert_eq!(LevPhase::from_u8(p.as_u8()), Some(*p));
        }
        assert_eq!(IntentFlag::from_u8(200), None);
        assert!(IntentFlag::Open.is_confirmed());
        assert!(!IntentFlag::PendingOpen.is_confirmed());
        assert_eq!(IntentFlag::PendingClose.dir(), Some(TravelDir::Close));
        assert_eq!(IntentFlag::Lean.dir(), None);
        assert_eq!(SeatStatus::Failed.name(), "Failed");
    }
}
