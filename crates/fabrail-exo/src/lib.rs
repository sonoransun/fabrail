//! fabrail-exo: sovereign-tier controllers and estimators in f32 (plan §4, §4b).
//!
//! HMD (LQR with the stroke → power → force → rate saturation stack, DVF fallback and
//! the mode ladder), TMD tuning helpers, HAVEN isolation skyhook, energy admission,
//! the closure-race planner, static pod allocation, EEW fusion and the house
//! choreographer (non-safety; not a rated safety function). Physics truth models live
//! in fabrail-exo-sim; integer safety logic lives in fabrail-exo-safety. The module
//! named `alloc` is pod force allocation: this crate never uses heap allocation.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod alloc;
pub mod choreo;
pub mod closure;
pub mod energy;
pub mod fusion;
pub mod hmd;
pub mod isolation;
pub mod tuning;
