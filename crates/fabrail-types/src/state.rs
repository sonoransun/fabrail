//! Classes, rest/egress/presence policy, the state machines' state enums, safety demands,
//! fault severities and the safety monitor's verdict (core design §B.1, §B.4;
//! plan §2b, §4, §4b additions).
//!
//! All enums are `#[repr(u8)]` with explicit discriminants: they are telemetry and replay
//! wire values and must never be renumbered.

use crate::FaultSet;

u8_enum! {
    /// Product class of a rail (core design §C.2). The core classes are pedestrian-regime
    /// devices; sovereign classes use `fabrail_exo_safety::ExoClass` and never extend this.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum RailClass {
        /// Cabinet drawer, 8–48 kg, captive SEAT rest.
        Drawer = 0,
        /// Sliding door, 100 kg, HOVER rest.
        #[default]
        Door = 1,
        /// Partition wall, 500 kg, LAND rest.
        Wall = 2,
        /// Infrastructure panel, 5 t, LAND + lock, hold-to-run unsensed.
        Heavy = 3,
    }
}

u8_enum! {
    /// What the carriage does at rest (PWR-001; F17 for the DRAWER seat).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum RestPolicy {
        /// Captive registration seat (DRAWER, g_seat 3.5 mm, constrained in z both ways).
        Seat = 0,
        /// Zero-power hover (DOOR).
        #[default]
        Hover = 1,
        /// Land on rollers at park (WALL).
        Land = 2,
        /// Land on rollers and engage the parking lock (HEAVY).
        LandLock = 3,
    }
}

u8_enum! {
    /// Role of the opening on a means of egress (IBC 1010.3; SEI-011).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum EgressRole {
        /// Not on an egress path (may land and lock).
        #[default]
        None = 0,
        /// Egress, non-fire-rated: never locks; may open only if stopped by t_S − 1 s.
        NonRated = 1,
        /// Egress, fire-rated: never locks and never opens on a seismic action.
        FireRated = 2,
    }
}

u8_enum! {
    /// Presence-sensing fitted (SAF-003: above the unsensed caps only with PL d sensing).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum PresenceSensing {
        /// Unsensed: low-energy caps (1.69 J, 22.3 kg·m/s, 67 N static).
        #[default]
        None = 0,
        /// ISO 13849-1 PL d presence sensing (declared target, not certified).
        PlD = 1,
    }
}

u8_enum! {
    /// Levitation / carriage state machine (supervisor/levstate.rs; TECHNICAL_DESIGN §4).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum LevState {
        #[default]
        Boot = 0,
        /// Golden hash, parameter CRC, referenced.
        SelfTest = 1,
        /// DRAWER rest on the captive seat.
        Seated = 2,
        /// Resting on rollers/skids.
        Landed = 3,
        /// Lift or unseat sequence.
        Lifting = 4,
        LevitatedIdle = 5,
        Moving = 6,
        Stopping = 7,
        Landing = 8,
        /// DRAWER z-seat step into the captive seat.
        Seating = 9,
        /// SS1 Safe Landing; STO on the LSM only.
        FaultReaction = 10,
        /// Latched; `UserCommand::Reset` clears it.
        Fault = 11,
        /// Seismic action in progress (delegates to the seismic policy).
        Seismic = 12,
        /// Egress fail-open / non-rated open-and-stop.
        Escape = 13,
        /// Unpowered on rollers, moved by hand.
        Manual = 14,
    }
}

u8_enum! {
    /// Seismic state machine (seismic/fusion.rs). A cancel from any state returns to
    /// NORMAL; INSPECT needs a manual reset (SEI-008).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum SeismicState {
        #[default]
        Normal = 0,
        /// Freeze.
        Triggered = 1,
        /// Class action (land-first).
        Protect = 2,
        Shaking = 3,
        /// 30 s without triggers.
        Quiet = 4,
        /// Awaiting manual reset.
        Inspect = 5,
    }
}

u8_enum! {
    /// Motion-control mode selected by the supervisor for the fast group.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum MotionMode {
        /// Propulsion off (STO).
        #[default]
        Off = 0,
        /// Position hold.
        Hold = 1,
        /// S-curve trajectory tracking.
        Trajectory = 2,
        /// Haptic admittance (hand-driven).
        Admittance = 3,
        /// Legacy soft-close spring (HAP-007; the looser bound).
        SoftClose = 4,
        /// Obstruction reversal (DOOR/WALL, 50 mm).
        Reverse = 5,
        /// Seismic freeze: decelerate at a_brk and hold.
        Freeze = 6,
        /// Seismic centring (only if reachable, F11).
        Center = 7,
        /// Contactless isolation (research tier, opt-in).
        Isolate = 8,
        /// Active mass damping (research tier, opt-in).
        Damp = 9,
        /// Log-damped terminus toward a close/seat target (PHYSICS §16.1).
        Terminus = 10,
        /// Log-damped terminus toward an open target (PHYSICS §16.1).
        SoftOpen = 11,
        /// Inertial smoothing: follow only low-frequency floor motion (research tier,
        /// PHYSICS §16.2; `inertial.enable = false` in every preset).
        InertialSmooth = 12,
    }
}

u8_enum! {
    /// Detector level from the Pd/τc matrix (Wu–Kanamori; SEI-005).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    pub enum SeismicLevel {
        #[default]
        L0 = 0,
        /// Freeze only.
        L1 = 1,
        /// PROTECT.
        L2 = 2,
        /// PROTECT (τc ≥ 0.6 s and Pd ≥ 0.5 cm).
        L3 = 3,
    }
}

u8_enum! {
    /// Seismic action a device takes in PROTECT. Land-first is the core default; the
    /// sovereign additions (RideThrough, CompleteClose, CloseAfterShaking) are selected
    /// only through the core `ActionPolicy` hook (plan §4).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum SeismicAction {
        #[default]
        None = 0,
        Freeze = 1,
        /// Centre, only if reachable (F11).
        Center = 2,
        Land = 3,
        /// Land and engage the parking lock (never on an egress path).
        Lock = 4,
        /// Contactless isolation (research tier).
        Isolate = 5,
        /// Active mass damping (research tier).
        Damp = 6,
        /// Non-rated egress door: open and stop by t_S − 1 s.
        OpenAndStop = 7,
        /// Stay levitated through the shaking (SENTINEL; drawer SmoothRideThrough).
        RideThrough = 8,
        /// Finish an in-progress close, else land (BASTION).
        CompleteClose = 9,
        /// Close only after shaking ends, on external command (TIDEGATE).
        CloseAfterShaking = 10,
    }
}

u8_enum! {
    /// Safety demand from the monitor, ordered by severity. `Sto*` act on PROPULSION only.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum SafetyDemand {
        #[default]
        None = 0,
        /// Cap the LSM force at `SafetyVerdict::force_cap_n`.
        LimitForce = 1,
        /// Controlled stop, then FAULT_REACTION at the next supervisor release.
        SafeStop = 2,
        /// SS1 Safe Landing.
        SafeLand = 3,
        /// Safe Torque Off on the LSM (+ Safe Landing).
        StoLsm = 4,
        /// STO + Active Short Circuit on the LSM (+ Safe Landing).
        StoLsmAsc = 5,
    }
}

u8_enum! {
    /// Default reaction class of a fault code, ordered.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum Severity {
        /// Logged only.
        #[default]
        Info = 0,
        /// Logged and reported.
        Warn = 1,
        /// Stays levitated; capability reduced (e.g. PayloadMismatch → hold-to-run only).
        Degrade = 2,
        /// Controlled stop.
        SafeStop = 3,
        /// SS1 Safe Landing.
        SafeLand = 4,
        /// STO on the LSM plus Safe Landing.
        Sto = 5,
    }
}

impl Severity {
    /// The monitor/supervisor demand this severity maps to.
    #[inline]
    pub const fn demand(self) -> SafetyDemand {
        match self {
            Severity::Info | Severity::Warn | Severity::Degrade => SafetyDemand::None,
            Severity::SafeStop => SafetyDemand::SafeStop,
            Severity::SafeLand => SafetyDemand::SafeLand,
            Severity::Sto => SafetyDemand::StoLsm,
        }
    }
}

/// The independent monitor's output for one step (fabrail-safety `Monitor::step_*`).
/// Defined here so `fabrail-core::msg::SupInputs` can carry the previous verdict without
/// core ever depending on the monitor crate.
///
/// `Default` is the most restrictive cap (0 N) with no demand: until the monitor has
/// published its first verdict the arbitration allows no propulsion force.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SafetyVerdict {
    /// Demand level; the core can never lower it within a latch window (SAF-016).
    pub demand: SafetyDemand,
    /// LSM force cap in newtons; arbitration applies min(core limit, this).
    pub force_cap_n: f32,
    /// Faults raised by the monitor.
    pub faults: FaultSet,
}
