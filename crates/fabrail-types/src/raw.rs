//! HAL raw samples: integer ADC codes, counts and flags exactly as the MCU peripherals
//! (or SimBoard's sensor models) deliver them (PHYSICS §13). The controller converts them
//! in `core::frontend`; the safety monitor reads them directly with its own calibration
//! copies, so a core conversion bug cannot blind the monitor.

use crate::{MAX_MAGNETS, Tick};

/// Gap sensors, one channel per magnet (eddy-current probes, 16-bit ADC over 20 mm).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GapRaw {
    /// Primary channel codes.
    pub code: [u16; MAX_MAGNETS],
    /// Redundant channel codes (0 when the redundant probe is absent).
    pub code_b: [u16; MAX_MAGNETS],
    /// Bit i set = channel i reported a valid conversion.
    pub valid: u8,
}

/// Coil current sense, one channel per magnet (12-bit, signed codes).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CurrentRaw {
    /// Signed ADC codes.
    pub code: [i16; MAX_MAGNETS],
}

/// Linear encoder (1 µm/count default), extended to 64 bits by the BSP.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EncoderRaw {
    /// Absolute counts; position = counts × nm_per_count.
    pub counts: i64,
    /// Homing / reference mark seen since boot.
    pub referenced: bool,
    /// Signal error (lost, illegal transition).
    pub err: bool,
}

/// LSM phase currents (signed codes) and driver fault line.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LsmRaw {
    /// Phase current codes a, b, c.
    pub i_abc_code: [i16; 3],
    /// Gate-driver fault (desaturation, UVLO, comparator trip).
    pub fault: bool,
}

/// One ADXL355 FIFO sample. The earthquake-early-warning accelerometer is mounted on the
/// STRUCTURE or rail, never on the carriage (SEI-001, core design F16).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AccelRaw {
    /// 20-bit two's-complement codes, sign-extended to i32 (x, y, z).
    pub xyz: [i32; 3],
    /// Sample sequence number (gaps reveal FIFO loss).
    pub seq: u32,
    /// FIFO overrun flag.
    pub overrun: bool,
}

/// Carriage IMU: skyhook absolute-velocity feedback and fusion only, never EEW (F16).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ImuRaw {
    /// Acceleration codes (x, y, z).
    pub xyz: [i32; 3],
    /// Sensor fitted and responding.
    pub present: bool,
}

/// Rail-mounted (base-group) floor accelerometer: the only source of floor feed-forward
/// for inertial smoothing (plan §4b; PHYSICS §16.2). The structure ADXL355 is used only
/// below 0.3 Hz.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RailAccelRaw {
    /// Acceleration codes (x, y, z).
    pub xyz: [i32; 3],
    /// Sensor fitted and responding.
    pub present: bool,
}

/// Power path sense (supervisor group).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PowerRaw {
    /// DC bus voltage code.
    pub v_bus_code: u16,
    /// Hold-up (supercap) voltage code.
    pub v_hold_code: u16,
    /// Bus current code (signed: negative = regeneration).
    pub i_bus_code: i16,
    /// Mains present.
    pub mains_ok: bool,
    /// Temperature codes: [coil hot-spot, magnet].
    pub temp_code: [u16; 2],
}

/// Discrete and analogue safety-related inputs (supervisor group).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SafetyInRaw {
    /// Presence-sensing device healthy (OSSD pair consistent).
    pub presence_ok: bool,
    /// Presence-sensing device reports a person in the zone.
    pub presence_active: bool,
    /// Handle load-cell code (optional sensor).
    pub handle_force_code: i16,
    /// Handle load cell fitted.
    pub has_handle_cell: bool,
    /// Building fire alarm input.
    pub fire_alarm: bool,
    /// Emergency stop pressed.
    pub estop: bool,
    /// Parking lock engaged (feedback switch).
    pub lock_engaged: bool,
    /// Hold-to-run control actuated (HEAVY unsensed operation).
    pub hold_to_run: bool,
}

/// Everything the base task latches for one fast-group release (FastGroup is a pure
/// function of `(FastRaw, FastIn)`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FastRaw {
    /// Base tick at which the sample was latched.
    pub tick: Tick,
    /// Gap sensors.
    pub gap: GapRaw,
    /// Coil currents.
    pub cur: CurrentRaw,
    /// Encoder.
    pub enc: EncoderRaw,
    /// LSM phase currents.
    pub lsm: LsmRaw,
    /// Carriage IMU.
    pub imu: ImuRaw,
    /// Rail-mounted floor accelerometer.
    pub rail: RailAccelRaw,
}

/// Power-stage driver status (coil H-bridges or LSM inverter).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DriverStatus {
    /// Gate drive enabled.
    pub enabled: bool,
    /// Latched driver fault.
    pub fault: bool,
    /// Hardware overcurrent comparator tripped.
    pub overcurrent: bool,
    /// Over-temperature.
    pub overtemp: bool,
    /// Gate-drive or bus undervoltage lock-out.
    pub undervoltage: bool,
}

u8_enum! {
    /// Why the MCU last reset (RCC CSR flags on the STM32G474; SimBoard reports PowerOn).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    pub enum ResetCause {
        #[default]
        PowerOn = 0,
        Pin = 1,
        Software = 2,
        IndependentWatchdog = 3,
        WindowWatchdog = 4,
        LowPower = 5,
        BrownOut = 6,
        /// Panic/HardFault handler wrote a crash record before resetting.
        Crash = 7,
        Unknown = 255,
    }
}
