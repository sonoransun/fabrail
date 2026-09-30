//! Actuator and user commands.
//!
//! Safe-state nuance (binding, core design C11): STO/ASC act on PROPULSION (the LSM)
//! only. Hybrid-PM coil STO is non-deterministic (at any gap below g* the magnet force
//! exceeds the weight), so levitation's safe state is always a controlled, energised
//! SS1 Safe Landing, backed by mechanical stops, rollers and the captive guideway.

use crate::{MAX_MAGNETS, PosNm};

/// Levitation coil command, one value per magnet.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CoilCmd {
    /// Bridges disabled (outputs off). Not a safe state while levitated (C11).
    #[default]
    Off,
    /// Current references in amperes (lag coil mode: SimBoard applies the lag).
    CurrentRef([f32; MAX_MAGNETS]),
    /// Bridge voltages in volts (voltage coil mode: the current PI runs in the base task).
    Voltage([f32; MAX_MAGNETS]),
}

impl CoilCmd {
    /// The per-magnet values (A or V); zeros for `Off`.
    #[inline]
    pub const fn values(&self) -> [f32; MAX_MAGNETS] {
        match self {
            CoilCmd::Off => [0.0; MAX_MAGNETS],
            CoilCmd::CurrentRef(v) | CoilCmd::Voltage(v) => *v,
        }
    }
}

u8_enum! {
    /// LSM inverter mode.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    pub enum LsmMode {
        /// Field-oriented current control.
        Run = 0,
        /// Safe Torque Off: gates disabled (IEC 61800-5-2 STO).
        #[default]
        Sto = 1,
        /// Active Short Circuit: low-side switches closed, back-EMF braking. Only above
        /// v_runaway or during a seismic landing; released after stop on egress classes
        /// (core design F9).
        Asc = 2,
    }
}

/// LSM command (dq current references in amperes).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LsmCmd {
    /// Quadrature (thrust) current reference, A. i_q* = F*/K̂_f (PHYSICS §7).
    pub iq_ref_a: f32,
    /// Direct-axis current reference, A.
    pub id_ref_a: f32,
    /// Inverter mode.
    pub mode: LsmMode,
}

/// Auxiliary actuators (supervisor group).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AuxCmd {
    /// Engage the parking lock (HEAVY; WALL "land + lock").
    pub parking_lock: bool,
    /// Engage the seismic x-latch (`landing.seismic_latch`, SeatAndLatch policy).
    pub seismic_latch: bool,
}

/// Commands from the user/building link (CAN or SimBoard events).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum UserCommand {
    /// Move to the open end (log-damped open terminus).
    Open,
    /// Move to the closed end (log-damped close terminus, then seat/land by rest policy).
    Close,
    /// Move to an absolute position.
    GoTo(PosNm),
    /// Controlled stop.
    Stop,
    /// Lift (DRAWER: unseat) to levitation.
    Lift,
    /// Land (DRAWER: seat).
    Land,
    /// Clear a latched FAULT / INSPECT after manual inspection.
    Reset,
    /// Enter MANUAL (unpowered, on rollers).
    Manual,
    /// Hold-to-run control state (HEAVY unsensed operation).
    HoldToRun(bool),
}
