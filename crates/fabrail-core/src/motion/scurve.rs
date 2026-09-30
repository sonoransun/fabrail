//! 7-segment jerk-limited S-curve evaluated in closed form relative to its anchor (PHYSICS §16). Owner: A2.
//!
//! Frozen API (W0): [`plan`] (from anchor to target under v, a, j limits) → `SCurveSeg`,
//! [`eval`] (closed form at a tick, relative to the anchor so the result is exact at any
//! travel), [`duration_s`] and [`stop_distance_m`] (d_stop at the speed caps:
//! 33.43 / 36.31 / 8.86 / 4.91 mm, core design §E.5). Test:
//! `tests/scurve_props::limits_and_stop_distance` (proptest, 1e-6 f32 relative to the
//! anchor).
//!
//! Profile (PHYSICS §16; core design §C.4). A rest-to-rest move of distance D under
//! limits (V, A, J) is seven constant-jerk phases: +J, 0, −J (accelerate to the peak
//! speed v_p), cruise, then the time-reversed mirror. Per accelerating half, with
//! t_j the jerk time and t_c the constant-acceleration time:
//!
//! * trapezoidal (v_p ≥ A²/J): a_p = A, t_j = A/J, t_c = v_p/A − A/J;
//! * triangular (v_p < A²/J): a_p = √(v_p·J), t_j = √(v_p/J), t_c = 0;
//! * t_a = 2·t_j + t_c and the half distance d_a(v_p) = v_p·t_a/2 (the profile is
//!   point-symmetric about its midpoint).
//!
//! d_a at the cap is the jerk-limited stop distance
//! d_stop = v²/(2a) + v·a/(2j) for v ≥ a²/j, else v·√(v/j) (PRP-005; the f64 twin is
//! `params::derive::stop_distance`). When 2·d_a(V) > D the cruise vanishes and v_p
//! solves 2·d_a(v_p) = D: v_p = (A/2)·(√(A²/J² + 4D/A) − A/J) for D ≥ 2A³/J², else
//! v_p = (D²·J/4)^(1/3) (both branches meet at v_p = A²/J).
//!
//! Evaluation is closed form, never a running sum (PHYSICS §24): the first half is
//! measured from the anchor and the second half backwards from the end point
//! `anchor + dir·dist` (the same `PosNm` as `Segment::end`), so the reference lands on
//! the end exactly and never accumulates f32 error over a 15 m travel. Time is the
//! integer tick difference to `t0` converted once to f32 (`Tick::since_s`), so the
//! evaluation is invariant to tick offsets.

use fabrail_types::{PosNm, SCurveSeg, Tick, TravelDir, fmath};

use crate::motion::RefState;

/// Closed-form shape of a planned segment (derived from `SCurveSeg` on every call; the
/// frozen segment stores only the limits and the distance).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Shape {
    /// Jerk time of one jerk phase, s.
    t_j: f32,
    /// Constant-acceleration time, s.
    t_c: f32,
    /// Accelerating-half duration 2·t_j + t_c, s.
    t_a: f32,
    /// Cruise duration, s.
    t_v: f32,
    /// Peak speed, m/s.
    v_p: f32,
    /// Peak acceleration, m/s².
    a_p: f32,
    /// Jerk, m/s³.
    j: f32,
    /// Distance covered by the accelerating half, m.
    d_a: f32,
}

impl Shape {
    /// Total duration 2·t_a + t_v, s.
    fn total_s(&self) -> f32 {
        2.0 * self.t_a + self.t_v
    }

    /// Accelerating-half profile of a peak speed `v` (PHYSICS §16): (t_j, t_c, a_p).
    fn accel_half(v: f32, a: f32, j: f32) -> (f32, f32, f32) {
        if v * j >= a * a {
            // Trapezoidal acceleration: a reaches A.
            (a / j, v / a - a / j, a)
        } else {
            // Triangular acceleration: a peaks at √(v·J) < A.
            (fmath::sqrtf(v / j), 0.0, fmath::sqrtf(v * j))
        }
    }

    /// Shape for distance `d` (m) under limits `v`, `a`, `j`; `None` for a hold
    /// (zero or invalid distance or limits).
    fn new(d: f32, v: f32, a: f32, j: f32) -> Option<Shape> {
        let ok = |x: f32| x.is_finite() && x > 0.0;
        if !(ok(d) && ok(v) && ok(a) && ok(j)) {
            return None;
        }
        let (t_j, t_c, a_p) = Shape::accel_half(v, a, j);
        let t_a = 2.0 * t_j + t_c;
        let d_a = 0.5 * v * t_a;
        if 2.0 * d_a <= d {
            // The cap is reached: cruise for the remainder.
            return Some(Shape {
                t_j,
                t_c,
                t_a,
                t_v: (d - 2.0 * d_a) / v,
                v_p: v,
                a_p,
                j,
                d_a,
            });
        }
        // Peak speed below the cap: solve 2·d_a(v_p) = D (module docs).
        let v_p = if d >= 2.0 * a * a * a / (j * j) {
            let r = a / j;
            0.5 * a * (fmath::sqrtf(r * r + 4.0 * d / a) - r)
        } else {
            fmath::cbrtf(0.25 * d * d * j)
        };
        let v_p = fmath::fminf(v_p, v);
        let (t_j, t_c, a_p) = Shape::accel_half(v_p, a, j);
        let t_a = 2.0 * t_j + t_c;
        let d_a = 0.5 * v_p * t_a;
        // Rounding may leave a sub-nanometre remainder; it becomes a vanishing cruise.
        let t_v = if v_p > 0.0 {
            fmath::fmaxf((d - 2.0 * d_a) / v_p, 0.0)
        } else {
            0.0
        };
        Some(Shape {
            t_j,
            t_c,
            t_a,
            t_v,
            v_p,
            a_p,
            j,
            d_a,
        })
    }

    /// Accelerating half at local time `tau` ∈ [0, t_a]: (distance, speed, accel).
    fn accel_at(&self, tau: f32) -> (f32, f32, f32) {
        let (j, t_j, a_p) = (self.j, self.t_j, self.a_p);
        let v1 = 0.5 * j * t_j * t_j;
        let s1 = j * t_j * t_j * t_j / 6.0;
        if tau <= t_j {
            // Phase 1: jerk +J.
            return (j * tau * tau * tau / 6.0, 0.5 * j * tau * tau, j * tau);
        }
        let t2 = t_j + self.t_c;
        if tau <= t2 {
            // Phase 2: constant acceleration a_p.
            let u = tau - t_j;
            return (s1 + v1 * u + 0.5 * a_p * u * u, v1 + a_p * u, a_p);
        }
        // Phase 3: jerk −J (evaluated from phase 2's closed-form end state).
        let u = fmath::fminf(tau - t2, t_j);
        let v2 = v1 + a_p * self.t_c;
        let s2 = s1 + v1 * self.t_c + 0.5 * a_p * self.t_c * self.t_c;
        (
            s2 + v2 * u + 0.5 * a_p * u * u - j * u * u * u / 6.0,
            v2 + a_p * u - 0.5 * j * u * u,
            a_p - j * u,
        )
    }
}

/// Plans a point-to-point S-curve from `anchor` (at tick `t0`) to `target` with speed,
/// acceleration and jerk limits (all > 0).
///
/// The segment stores the limits and the travel distance |target − anchor| (integer
/// difference first, then f32; PHYSICS §24). Its end point `anchor + dir·dist` equals
/// `target` to the f32 rounding of `dist` (≤ 0.5 ulp: ≤ 0.5 µm at 15 m).
pub fn plan(
    anchor: PosNm,
    t0: Tick,
    target: PosNm,
    v_mps: f32,
    a_mps2: f32,
    j_mps3: f32,
) -> SCurveSeg {
    let diff_nm = target.diff_nm(anchor);
    let dir = if diff_nm < 0 {
        TravelDir::Close
    } else {
        TravelDir::Open
    };
    SCurveSeg {
        anchor,
        t0,
        dir,
        dist_m: (diff_nm.unsigned_abs() as f64 / PosNm::NM_PER_M) as f32,
        v_mps,
        a_mps2,
        j_mps3,
    }
}

/// Evaluates `seg` at tick `t` in closed form.
///
/// Before `t0` the anchor at rest; after the end the end point at rest with
/// `done = true`. A zero-length or degenerate (non-positive limit) segment is a hold at
/// the anchor, `done` once `t ≥ t0`. Velocity and acceleration are signed along +x.
pub fn eval(seg: &SCurveSeg, t: Tick) -> RefState {
    let hold = RefState {
        x: seg.anchor,
        v_mps: 0.0,
        a_mps2: 0.0,
        done: t >= seg.t0,
    };
    let Some(sh) = Shape::new(seg.dist_m, seg.v_mps, seg.a_mps2, seg.j_mps3) else {
        return hold;
    };
    let sign = seg.dir.sign();
    let tau = t.since_s(seg.t0);
    let total = sh.total_s();
    if tau >= total {
        return RefState {
            x: seg.anchor.offset_m(sign * seg.dist_m),
            v_mps: 0.0,
            a_mps2: 0.0,
            done: true,
        };
    }
    if tau <= 0.0 {
        return RefState {
            done: false,
            ..hold
        };
    }
    let end = seg.anchor.offset_m(sign * seg.dist_m);
    let decel_start = sh.t_a + sh.t_v;
    let (x, v, a) = if tau < sh.t_a {
        let (s, v, a) = sh.accel_at(tau);
        (seg.anchor.offset_m(sign * s), v, a)
    } else if tau < decel_start {
        // Cruise: first half from the anchor, second half back from the end.
        if tau <= 0.5 * total {
            let s = sh.d_a + sh.v_p * (tau - sh.t_a);
            (seg.anchor.offset_m(sign * s), sh.v_p, 0.0)
        } else {
            let r = sh.d_a + sh.v_p * (decel_start - tau);
            (end.offset_m(-sign * r), sh.v_p, 0.0)
        }
    } else {
        // Decelerating half: the time-reverse of the accelerating half, measured back
        // from the end (distance-to-go = accel distance at the time-to-go).
        let (r, v, a) = sh.accel_at(total - tau);
        (end.offset_m(-sign * r), v, -a)
    };
    RefState {
        x,
        v_mps: sign * v,
        a_mps2: sign * a,
        done: false,
    }
}

/// Total duration of `seg`, s (0 for a hold).
pub fn duration_s(seg: &SCurveSeg) -> f32 {
    match Shape::new(seg.dist_m, seg.v_mps, seg.a_mps2, seg.j_mps3) {
        Some(sh) => sh.total_s(),
        None => 0.0,
    }
}

/// Jerk-limited stopping distance from speed `v` with deceleration `a` and jerk `j`
/// (PHYSICS §16): v²/(2a) + v·a/(2j) when v ≥ a²/j, else v·√(v/j), m. Starts from zero
/// acceleration, i.e. it is the decelerating half of the S-curve at peak speed |v|.
/// Non-positive `a` or `j` gives +∞ (the carriage cannot be stopped by the planner).
pub fn stop_distance_m(v_mps: f32, a_mps2: f32, j_mps3: f32) -> f32 {
    if a_mps2.is_nan() || j_mps3.is_nan() || a_mps2 <= 0.0 || j_mps3 <= 0.0 {
        return f32::INFINITY;
    }
    let v = fmath::absf(v_mps);
    if v >= a_mps2 * a_mps2 / j_mps3 {
        v * v / (2.0 * a_mps2) + v * a_mps2 / (2.0 * j_mps3)
    } else {
        v * fmath::sqrtf(v / j_mps3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn planned_segment_starts_at_anchor() {
        let s = plan(PosNm(5), Tick(10), PosNm(-5), 0.1, 1.0, 10.0);
        assert_eq!(s.anchor, PosNm(5));
        assert_eq!(s.dir, TravelDir::Close);
        assert_eq!(eval(&s, Tick(10)).x, PosNm(5));
    }

    /// Both d_stop branches against the f64 closed form; (E) f32 rel 1e-6, PHYSICS §16.
    #[test]
    fn stop_distance_branches() {
        for &(v, a, j) in &[
            (0.2654_f32, 1.5_f32, 20.0_f32),
            (0.05, 1.5, 20.0),
            (0.025, 0.08, 1.0),
        ] {
            let got = stop_distance_m(v, a, j) as f64;
            let want = fabrail_params::derive::stop_distance(v as f64, a as f64, j as f64);
            assert!(
                ((got - want) / want).abs() <= 1e-6,
                "d_stop({v},{a},{j}) = {got} vs {want} ((E) f32 rel 1e-6, PHYSICS §16)"
            );
        }
        assert_eq!(stop_distance_m(0.1, 0.0, 1.0), f32::INFINITY);
    }

    /// The decelerating half of a capped profile is exactly d_stop; (E) f32, PHYSICS §16.
    #[test]
    fn decel_half_is_stop_distance() {
        let s = plan(PosNm(0), Tick(0), PosNm(300_000_000), 0.2654, 1.5, 20.0);
        let sh = Shape::new(s.dist_m, s.v_mps, s.a_mps2, s.j_mps3).unwrap_or_default();
        let d = stop_distance_m(0.2654, 1.5, 20.0);
        assert!(((sh.d_a - d) / d).abs() < 1e-6, "{} vs {d}", sh.d_a);
        assert!(sh.t_v > 0.0);
    }
}
