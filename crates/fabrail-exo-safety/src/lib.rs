//! fabrail-exo-safety: sovereign-tier safety logic in integers (plan §2, §4).
//!
//! Plain-data exo types (`ExoClass`, `SafetyRegime`, safe-state policies, `SafetyRank`,
//! `FusedAlert`), integer units (kg as u32, mm/s as i32, 2·KE in u64 µJ, momentum in
//! u64 kg·mm/s; no Q16.16), the MVS 2oo3 voter and degradation ladder
//! (2oo3 → 1oo2D → 1oo1D → safe), the black channel, the zone state machine, the integer
//! envelope, the alert gate and the PFH calculator. "Designed toward, not certified."
//! Depends on fabrail-types and fabrail-params ONLY (FAB-EXO-003).
//!
//! The `pub` items of `types`, `units` and `rank` are re-exported at the crate root.
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

pub mod alert_gate;
pub mod black_channel;
pub mod envelope;
pub mod ladder;
pub mod pfh;
pub mod presets;
pub mod rank;
pub mod types;
pub mod units;
pub mod vote;
pub mod zone;

// Glob re-exports of the frozen API modules (names must stay unique across them).
#[allow(unused_imports)]
pub use rank::*;
#[allow(unused_imports)]
pub use types::*;
#[allow(unused_imports)]
pub use units::*;
