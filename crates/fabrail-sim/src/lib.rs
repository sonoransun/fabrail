//! fabrail-sim: the SIL harness (core design §B.6).
//!
//! Scenario spec v1 (`fabrail.scenario/1`), SimBoard (every HAL trait over plant truth
//! with sensor noise and quantisation), the event executor, the base-tick loop (`run`),
//! batch runs in index order (`run_batch`, rayon behind the default `parallel`
//! feature), recorder and KPI metrics, trace replay (bit-identical), the detector
//! replay, and the steppable `Session` used by the exotic building scenes. All Rust
//! randomness lives in `rng` (SplitMix64 + Xoshiro256++, named substreams).
//!
//! The `pub` items of `scenario`, `session`, `scheduler`, `batch`, `detect` and
//! `recorder` are re-exported at the crate root (`fabrail_sim::run`,
//! `fabrail_sim::Session`, …), so their names must be unique across those modules.
//!
//! `run`, `replay::record` and `Session` share one private base-tick engine
//! (`engine`): the plant tier, the `fabrail-rt` executive at the 20 kHz base tick
//! (whatever the plant step), the `SimBoard`, the event executor, and the optional
//! recorder, metrics and HAL trace.
#![forbid(unsafe_code)]

mod engine;

pub mod batch;
pub mod detect;
pub mod events;
pub mod faults;
pub mod metrics;
pub mod recorder;
pub mod replay;
pub mod rng;
pub mod scenario;
pub mod scheduler;
pub mod sensors;
pub mod session;
pub mod simboard;

// Glob re-exports of the frozen API modules (names must stay unique across them).
#[allow(unused_imports)]
pub use batch::*;
#[allow(unused_imports)]
pub use detect::*;
#[allow(unused_imports)]
pub use recorder::*;
#[allow(unused_imports)]
pub use scenario::*;
#[allow(unused_imports)]
pub use scheduler::*;
#[allow(unused_imports)]
pub use session::*;
