//! Safety limits: the block the independent monitor (fabrail-safety) reads (core design
//! §B.2, §B.5; plan §2b items 4, 5, 8, 17; plan §4b unattended close).
//!
//! The block carries DIVERSE COPIES of the calibration values the monitor needs
//! (`*_copy`), so the monitor never trusts the controller's `SensorCal`. `crc::seal`
//! recomputes `crc` over the block; the diverse copies must equal the design values or
//! the carriage refuses to lift (SAF-015). All physical fields are f64 (plan §2b item 8);
//! the monitor converts them to integer thresholds in `Monitor::new()` and runs integer
//! arithmetic per tick.

use fabrail_types::PresenceSensing;

/// Explicit ISOLATE-mode envelope (SAF-020). In ISOLATE these limits REPLACE the clamp
/// and KE checks rather than disabling them (an R-tier deviation declared in SAFETY.md).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct IsolateLimits {
    /// Stroke envelope about the isolation centre, m.
    pub stroke_m: f64,
    /// Kinetic energy allowed at bumper contact, J.
    pub bumper_ke_j: f64,
    /// Continuous LSM force limit, N.
    pub f_cont_n: f64,
}

/// Unattended automatic close (choreographed seismic close; plan §4b):
/// v ≤ min(v_cap, f_pinch/√(m_gov·k_series)) (0.23 m/s at 48 kg), obstruction armed,
/// one attempt only.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct UnattendedClose {
    /// Peak pinch force limit (150 N, EN 16005 static), N.
    pub f_pinch_n: f64,
    /// Series stiffness of the body (hand 75 N/mm, ISO/TS 15066) with the closing-edge
    /// seal (10 N/mm), N/m.
    pub k_series_n_per_m: f64,
    /// Hand-in-gap detection deadline (20 ms), s.
    pub detect_max_s: f64,
    /// Close attempts before the device is reported NotSecured (1).
    pub max_attempts: u8,
}

/// The monitor's limits and calibration copies.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct SafetyLimits {
    /// Rated mass used by the energy/momentum checks (never m̂), kg.
    pub mass_rated_kg: f64,
    /// Kinetic-energy cap (1.69 J unsensed), J.
    pub e_max_j: f64,
    /// Momentum cap (22.3 kg·m/s; 0 = not applicable), kg·m/s.
    pub p_max_kgmps: f64,
    /// Unsensed opening speed cap, m/s.
    pub v_open_max_mps: f64,
    /// Unsensed closing speed cap, m/s.
    pub v_close_max_mps: f64,
    /// Opening speed cap with PL d presence sensing, m/s.
    pub v_open_sensed_mps: f64,
    /// Closing speed cap with PL d presence sensing, m/s.
    pub v_close_sensed_mps: f64,
    /// Dynamic driving-force limit (≤ `t_drive_max_s`), N.
    pub f_drive_max_n: f64,
    /// Dynamic driving-force window (0.75 s), s.
    pub t_drive_max_s: f64,
    /// Static (stall) force unsensed (67 N), N.
    pub f_static_n: f64,
    /// Static force when sensed (150 N), N.
    pub f_static_sensed_n: f64,
    /// Static force after `t_static_final_s` (< 80 N), N.
    pub f_static_final_n: f64,
    /// Time to reach the final static force (5 s), s.
    pub t_static_final_s: f64,
    /// Closing force allowed in the last zone (150 N), N.
    pub f_close_last50_n: f64,
    /// Length of the last closing zone (50 mm), m.
    pub last_zone_m: f64,
    /// Continuous coil current (I²t floor; mirrors `suspension.i_cont_a`), A.
    pub i_coil_cont_a: f64,
    /// Pulse coil current (mirrors `suspension.i_pulse_a`), A.
    pub i_coil_pulse_a: f64,
    /// Pulse duration (I²t budget (i_pulse² − i_cont²)·t_pulse), s.
    pub t_coil_pulse_s: f64,
    /// LSM phase current limit, A.
    pub i_lsm_max_a: f64,
    /// Minimum safe gap, m.
    pub gap_min_m: f64,
    /// Maximum safe gap, m.
    pub gap_max_m: f64,
    /// Safely-limited position, lower bound, m.
    pub x_min_m: f64,
    /// Safely-limited position, upper bound, m.
    pub x_max_m: f64,
    /// Overspeed trip ratio v/v_lim (1.1; SAF-008).
    pub overspeed_ratio: f64,
    /// Overspeed persistence (10 ms), s.
    pub overspeed_t_s: f64,
    /// Integer velocity window (≥ 10 ms; plan §2b item 17), s.
    pub vel_window_s: f64,
    /// ASC allowed only above this speed outside seismic landing (F9), m/s.
    pub v_runaway_mps: f64,
    /// Minimum hold-up voltage for a Safe Landing, V.
    pub v_hold_min_v: f64,
    /// Diverse copy of the LSM thrust constant, N/A.
    pub k_f_copy_n_per_a: f64,
    /// Diverse copy of the coil current scale, A/code.
    pub adc_a_per_code_copy: f64,
    /// Diverse copy of the LSM phase current scale, A/code.
    pub lsm_a_per_code_copy: f64,
    /// Diverse copy of the encoder resolution, nm/count.
    pub nm_per_count_copy: i32,
    /// Diverse copy of `design.presence`: the monitor allows the sensed caps
    /// (`v_*_sensed_mps`, `f_static_sensed_n`) only when this is `PlD` AND the presence
    /// input is healthy (SAF-003). The monitor never reads the design block.
    pub presence_copy: PresenceSensing,
    /// Watchdog timeout, base ticks.
    pub watchdog_ticks: u32,
    /// Heartbeat staleness limit, base ticks.
    pub heartbeat_max_ticks: u32,
    /// ISOLATE-mode replacement envelope (SAF-020).
    pub isolate: IsolateLimits,
    /// Unattended automatic close limits (plan §4b).
    pub unattended: UnattendedClose,
    /// CRC32 over the block (all fields except this one), set by `crc::seal`.
    pub crc: u32,
}
