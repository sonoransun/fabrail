//! fabrail-safety: the independent monitor channel (core design §B.5; SAFETY.md).
//!
//! Reads RAW samples (never core's conversions) with its own CRC-protected calibration
//! copies from `SafetyLimits`, and demands STO/ASC on PROPULSION only (levitation's safe
//! state is SS1 Safe Landing). Floats appear only in `Monitor::new()`, which converts
//! thresholds to integer codes; per-tick checks are integer-only (KE in u128 or
//! saturating arithmetic, velocity windowed over >= 10 ms, current/force checks on
//! integer |i_abc| codes). Depends on fabrail-types and fabrail-params ONLY: never on
//! fabrail-core (common-cause independence; transitive closure test).
//!
//! The crate-root constants below are the monitor's own monitoring tolerances. They are
//! not part of the sealed `SafetyLimits` block (which has no fields for them); each is a
//! documented design choice with its source, listed for the parameter owner to adopt.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing
    )
)]

pub mod energy;
pub mod envelope;
pub mod force;
pub mod heartbeat;
pub mod monitor;
pub mod position;
pub mod supply;
pub mod velocity;

pub use fabrail_types::SafetyVerdict;
// Glob re-exports of the frozen API modules (names must stay unique across them).
#[allow(unused_imports)]
pub use monitor::*;

/// Monitoring tolerance on measured force and current, numerator: a measurement may
/// exceed the limit in force by 10 % before the monitor faults (the same 1.1 ratio
/// SAF-008 uses for speed; covers the ±1 LSB current noise and current-loop overshoot of
/// PHYSICS §13 and §4). Design choice.
pub const MON_TOL_NUM: u64 = 11;

/// Monitoring tolerance, denominator (see [`MON_TOL_NUM`]).
pub const MON_TOL_DEN: u64 = 10;

/// Grace after a force cap is lowered before the new cap is enforced, base ticks
/// (2 ms: one command release plus ≈ 6 current-loop time constants τ_cl = 0.159 ms at
/// ω_c = 2π·1 kHz, PLAN §2b item 2, PHYSICS §4, plus one 1 ms plant step of the
/// Travel1d tier, whose LSM current follows a new reference only at its next step; with
/// 1 ms a 5 t HEAVY move tripped ForceLimit on Travel1d 1.2 ms after the arbitration had
/// already lowered the command, W2). Also the STO current-decay allowance. Negligible
/// against the 0.75 s / 5 s force-time windows. Design choice.
pub const CAP_GRACE_TICKS: u32 = 40;

/// Standstill speed, µm/s: |v| below 2 mm/s is a stall (core design §C.4 clamp stall
/// threshold `supervisor.stall_v_mps`, which the sealed safety block does not copy).
pub const STANDSTILL_UMPS: u64 = 2_000;

/// The LSM counts as energised (a possible energy source) when its measured force
/// exceeds F_drive_max / `LSM_ACTIVE_DIV` (5 %: above drag compensation, e.g. DRAWER
/// 0.6 N at 0.4 m/s against a 4.5 N threshold; PHYSICS §9). Design choice.
pub const LSM_ACTIVE_DIV: u32 = 20;

/// Sustain window for the sign-error evidence of positive drive work, base ticks
/// (0.5 s at 20 kHz; Wave 2 lead decision D4): with the LSM energised and a braking
/// command, a carriage that keeps speeding up over every persistence interval for this
/// long is being driven by the LSM (a commutation sign error), not by a hand or a rail
/// disturbance, which are brief. Design choice: a hand throw lasts ≤ 0.3 s.
pub const BRAKE_ACCEL_SUSTAIN_TICKS: u32 = 10_000;

/// LSM phase-sum tolerance and STO residual-current threshold as a fraction
/// 1/`LSM_SUM_DIV` of the phase current limit i_lsm_max (a star winding has
/// Σ i_abc = 0; ADC offsets and ±1 LSB noise stay far below 10 %). Design choice.
pub const LSM_SUM_DIV: u32 = 10;

/// Applies the monitoring tolerance to a squared threshold: x·(11/10)², saturating.
#[inline]
pub const fn tol_sq(x: u64) -> u64 {
    let num = MON_TOL_NUM as u128 * MON_TOL_NUM as u128;
    let den = MON_TOL_DEN as u128 * MON_TOL_DEN as u128;
    let v = x as u128 * num / den;
    if v > u64::MAX as u128 {
        u64::MAX
    } else {
        v as u64
    }
}

/// Applies the monitoring tolerance to a linear threshold: x·11/10, saturating.
#[inline]
pub const fn tol_lin(x: u64) -> u64 {
    let v = x as u128 * MON_TOL_NUM as u128 / MON_TOL_DEN as u128;
    if v > u64::MAX as u128 {
        u64::MAX
    } else {
        v as u64
    }
}
