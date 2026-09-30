//! fabrail-exo-sim: sovereign-tier f64 truth models (plan §4, §4b).
//!
//! The `SuspensionModel` trait and models (HybridEms adapter, PureEms, HTS frozen image
//! with play hysteresis, creep, Jc(T) and LN2; AirPad; FeCo as a parameter), energy
//! storage truth, host structures (2-DOF, modal tower), the HAVEN-R 3-DOF platform,
//! ground-motion additions, EEW timing, tsunami timing, XS/XC scenarios and the
//! multi-device building scenes (one-way building → devices coupling on
//! `fabrail_sim::Session`). Feature `py` exposes `py::register`, which adds the flat
//! `exo_*` / `exo_scene_*` functions to `fabrail._native`.
// The PyO3 macros used under feature `py` expand to `unsafe` FFI glue.
#![cfg_attr(not(feature = "py"), forbid(unsafe_code))]

pub mod eew_timing;
pub mod ground;
pub mod host;
pub mod platform;
pub mod scenarios;
pub mod scene;
pub mod storage;
pub mod suspension;
pub mod tsunami;

#[cfg(feature = "py")]
pub mod py;
