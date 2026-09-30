//! fabrail-core: the control core, i.e. the function channel (core design §B.4).
//!
//! Deterministic, allocation-free f32 control: levitation (zero-power hybrid EMS),
//! motion (S-curves, log-damped termini, governor, clamps), haptics (admittance,
//! intent, obstruction, passivity), seismic (detector, fusion, land-first policy) and
//! the supervisor. f64 appears only in init-time gain design (`new()`), via
//! `fabrail_params::derive`. All math goes through `fabrail_types::fmath`.
//!
//! Step API (frozen): `current::CurrentLoops` (base, 20 kHz), `fast::FastLoop`
//! (10/5 kHz, a pure function of its inputs), `supervisor::Supervisor` (1 kHz),
//! `seismic::SeismicUnit` (100 Hz). Groups exchange data only through the `msg` types
//! and fabrail-rt's LET mailboxes. This crate never depends on fabrail-safety.
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

pub mod current;
pub mod fast;
pub mod filters;
pub mod foc;
pub mod frontend;
pub mod haptics;
pub mod levitation;
pub mod linalg;
pub mod motion;
pub mod msg;
pub mod seismic;
pub mod supervisor;
pub mod telemetry;

// Glob re-exports of the frozen API modules (names must stay unique across them).
#[allow(unused_imports)]
pub use msg::*;
