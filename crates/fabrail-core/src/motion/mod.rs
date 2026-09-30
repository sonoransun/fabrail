//! Motion: reference segments, cascade, energy/momentum governor, directional clamps,
//! log-damped termini and inertial smoothing (PHYSICS §16, §16.1, §16.2).
//! Owner: A2 (scurve, cascade, governor, clamp) and E1 (terminus, inertial); module
//! list frozen by W0a.
//!
//! Frozen shared items (W0): [`RefState`] (the reference the cascade tracks) and
//! [`eval_segment`] (dispatches a `Segment` to `scurve::eval`, `terminus::eval` or
//! `terminus::eval_quintic`). Segments are always evaluated in closed form from their
//! anchor/target and start tick, never as a running f32 sum (PHYSICS §24).

pub mod cascade;
pub mod clamp;
pub mod governor;
pub mod inertial;
pub mod scurve;
pub mod terminus;

use fabrail_types::{PosNm, Segment, Tick};

/// Reference state at one tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RefState {
    /// Reference position.
    pub x: PosNm,
    /// Reference velocity, m/s.
    pub v_mps: f32,
    /// Reference acceleration, m/s².
    pub a_mps2: f32,
    /// The segment has finished (t ≥ its duration).
    pub done: bool,
}

/// Evaluates any segment at tick `t` (before `t0` the start state, after the end the
/// end state with zero velocity and acceleration).
pub fn eval_segment(seg: &Segment, t: Tick) -> RefState {
    match seg {
        Segment::SCurve(s) => scurve::eval(s, t),
        Segment::Terminus(s) => {
            let (x, v_mps, a_mps2) = terminus::eval(s, t);
            RefState {
                x,
                v_mps,
                a_mps2,
                done: terminus::done(seg, t),
            }
        }
        Segment::Quintic(q) => {
            let (x, v_mps, a_mps2) = terminus::eval_quintic(q, t);
            RefState {
                x,
                v_mps,
                a_mps2,
                done: terminus::done(seg, t),
            }
        }
    }
}
