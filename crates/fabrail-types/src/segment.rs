//! Motion reference segments (PHYSICS §16, §16.1), replacing the earlier `SCurveSeg`-only
//! directive field. The supervisor plans a segment in closed form relative to an anchor
//! `PosNm` and a start `Tick`; the fast group evaluates it (never as a running f32 sum).
//!
//! Log-damped terminus (PHYSICS §16.1, plan §4b), for approach speed v, jerk j, time
//! constant τ and capture distance d_ε:
//!
//! 1. constant-jerk blend out of cruise: t_b = √(τ² + 2v/j) − τ,
//!    d_b = j·t_b·τ², d_0 = d_b + v·t_b − j·t_b³/6;
//! 2. exponential capture d(t) = d_b·e^(−t/τ) (a straight line of slope −1/τ on a
//!    semilog plot: the "log-damped" definition);
//! 3. at d_ε a quintic from (d_ε, −d_ε/τ, d_ε/τ²) runs monotonically to v = a = 0 in
//!    T_q = τ;
//! 4. t_arrive = t_b + τ·ln(d_b/d_ε) + T_q.
//!
//! Distances in `TerminusSeg`/`QuinticSeg` are distance-to-go `d ≥ 0` measured from
//! `target` against `dir`: x = target − dir.sign()·d.

use crate::{PosNm, Tick};

u8_enum! {
    /// Direction of travel along x (+x opens; PHYSICS §1, SW-005).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum TravelDir {
        /// +x.
        #[default]
        Open = 0,
        /// −x.
        Close = 1,
    }
}

impl TravelDir {
    /// +1 for `Open`, −1 for `Close`.
    #[inline]
    pub const fn sign(self) -> f32 {
        match self {
            TravelDir::Open => 1.0,
            TravelDir::Close => -1.0,
        }
    }
}

/// 7-segment jerk-limited S-curve from `anchor`, evaluated in closed form (PHYSICS §16).
/// A zero `dist_m` segment is a hold at `anchor`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SCurveSeg {
    /// Start position.
    pub anchor: PosNm,
    /// Start tick.
    pub t0: Tick,
    /// Direction of travel.
    pub dir: TravelDir,
    /// Travel distance, m (≥ 0).
    pub dist_m: f32,
    /// Cruise speed limit, m/s.
    pub v_mps: f32,
    /// Acceleration limit, m/s².
    pub a_mps2: f32,
    /// Jerk limit, m/s³.
    pub j_mps3: f32,
}

/// Log-damped terminus (PHYSICS §16.1): constant-jerk blend, exponential capture, then
/// hand-over to a [`QuinticSeg`] at `d_eps_m`. Blend quantities are precomputed by the
/// supervisor with `params::derive::terminus_blend` (τ_eff = max(τ, τ_min(v_measured))).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct TerminusSeg {
    /// Terminal position (the seat/close stop or the remembered open position).
    pub target: PosNm,
    /// Tick at which the blend starts (distance-to-go = `d_0_m`).
    pub t0: Tick,
    /// Direction of travel toward `target`.
    pub dir: TravelDir,
    /// Approach speed magnitude at blend start, m/s.
    pub v_app_mps: f32,
    /// Blend jerk, m/s³.
    pub j_mps3: f32,
    /// Exponential time constant τ (effective, after the τ_min rule), s.
    pub tau_s: f32,
    /// Blend duration t_b, s.
    pub t_b_s: f32,
    /// Distance-to-go at the end of the blend d_b, m.
    pub d_b_m: f32,
    /// Distance-to-go at blend start d_0, m.
    pub d_0_m: f32,
    /// Capture distance d_ε where the quintic takes over, m.
    pub d_eps_m: f32,
    /// Quintic duration T_q (= τ by default), s.
    pub t_q_s: f32,
}

/// Quintic in distance-to-go from (d0, ḋ0, d̈0) to (0, 0, 0) in `t_q_s`: C² and
/// monotone for the terminus hand-over state (d_ε, −d_ε/τ, d_ε/τ²).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct QuinticSeg {
    /// Terminal position.
    pub target: PosNm,
    /// Start tick.
    pub t0: Tick,
    /// Direction of travel toward `target`.
    pub dir: TravelDir,
    /// Initial distance-to-go, m (≥ 0).
    pub d0_m: f32,
    /// Initial rate of distance-to-go ḋ, m/s (≤ 0 while approaching).
    pub v0_mps: f32,
    /// Initial d̈, m/s².
    pub a0_mps2: f32,
    /// Duration, s.
    pub t_q_s: f32,
}

/// The reference segment carried by a supervisor `Directive`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Segment {
    /// Cruise / point-to-point S-curve.
    SCurve(SCurveSeg),
    /// Log-damped soft-open / soft-close terminus.
    Terminus(TerminusSeg),
    /// Final quintic capture (also used stand-alone for short re-plans).
    Quintic(QuinticSeg),
}

impl Default for Segment {
    /// A zero-length S-curve at the origin (a hold).
    fn default() -> Segment {
        Segment::SCurve(SCurveSeg::default())
    }
}

impl Segment {
    /// Start tick of the segment.
    #[inline]
    pub const fn t0(&self) -> Tick {
        match self {
            Segment::SCurve(s) => s.t0,
            Segment::Terminus(s) => s.t0,
            Segment::Quintic(s) => s.t0,
        }
    }

    /// Direction of travel.
    #[inline]
    pub const fn dir(&self) -> TravelDir {
        match self {
            Segment::SCurve(s) => s.dir,
            Segment::Terminus(s) => s.dir,
            Segment::Quintic(s) => s.dir,
        }
    }

    /// The position the segment ends at (anchor + dir·dist for an S-curve).
    #[inline]
    pub fn end(&self) -> PosNm {
        match self {
            Segment::SCurve(s) => s.anchor.offset_m(s.dir.sign() * s.dist_m),
            Segment::Terminus(s) => s.target,
            Segment::Quintic(s) => s.target,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_accessors() {
        let s = Segment::SCurve(SCurveSeg {
            anchor: PosNm(1_000),
            t0: Tick(5),
            dir: TravelDir::Close,
            dist_m: 0.25,
            v_mps: 0.1,
            a_mps2: 1.0,
            j_mps3: 10.0,
        });
        assert_eq!(s.t0(), Tick(5));
        assert_eq!(s.dir(), TravelDir::Close);
        assert_eq!(s.end(), PosNm(1_000 - 250_000_000));
        let d = Segment::default();
        assert_eq!(d.end(), PosNm::ZERO);
        assert_eq!(TravelDir::Open.sign(), 1.0);
        assert_eq!(TravelDir::from_u8(1), Some(TravelDir::Close));
    }
}
