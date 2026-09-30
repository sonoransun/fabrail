//! fabrail-plant: f64 truth physics (core design §B.6; PHYSICS §2–§14, §22–§24).
//!
//! Co-energy hybrid magnets with saturation, eddy lag and tempco; coils (lag or full
//! flux-linkage voltage mode); the LSM; rigid-body tiers (Carriage6, Heave1, Travel1d,
//! Structure); contacts with elasto-plastic bristle friction; dissipation, rail, air,
//! thermal, human interaction, stray field and power. Deterministic and RNG-free; RK4
//! at 50 µs (Travel1d/Structure 1 ms). Transcendentals go through
//! `fabrail_types::fmath` (native `f64::sqrt` is allowed here, plan §2b item 15).
//!
//! `model` holds the `PlantModel` trait, `PlantInputs`, `Truth` and `InitialState`,
//! re-exported at the crate root.
#![forbid(unsafe_code)]

pub mod air;
pub mod body;
pub mod coil;
pub mod contact;
pub mod dissipation;
pub mod friction;
pub mod guidance;
pub mod heave1;
pub mod integrator;
pub mod interaction;
pub mod lsm;
pub mod magnet;
pub mod model;
pub mod params;
pub mod power;
pub mod rail;
pub mod stability;
pub mod stray;
pub mod structure;
pub mod thermal;
pub mod travel1d;

// Glob re-exports of the frozen API modules (names must stay unique across them).
#[allow(unused_imports)]
pub use model::*;
