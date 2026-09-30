//! fabrail-rt: the exact executive the MCU runs (core design §B.5).
//!
//! LET mailboxes (a job released at tick r reads values active at <= r and publishes
//! values that activate at r + P; the base tick always runs first), the four rate groups
//! (base 20 kHz, fast 10/5 kHz, supervisor 1 kHz, seismic 100 Hz), the `Executive`
//! (`on_base_tick` for SIL, `split` for RTIC), safety arbitration (core can never lower
//! a monitor demand) and the golden selftest hash that proves SIL ≡ firmware.
//!
//! The `pub` items of `exec`, `groups`, `mailbox` and `selftest` are re-exported at the
//! crate root, so their names must be unique across those modules.
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

pub mod arbitrate;
pub mod exec;
pub mod groups;
pub mod mailbox;
pub mod selftest;

// Glob re-exports of the frozen API modules (names must stay unique across them).
#[allow(unused_imports)]
pub use exec::*;
#[allow(unused_imports)]
pub use groups::*;
#[allow(unused_imports)]
pub use mailbox::*;
#[allow(unused_imports)]
pub use selftest::*;
