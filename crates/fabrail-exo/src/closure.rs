//! Closure-race planner: jerk-aware t_min, CLOSE iff feasible and the zone is clear (FAB-EXO-807). Owner: X2 (API frozen by W0a).
//!
//! EXOTIC §X11, §X12 (BASTION, TIDEGATE); exotic design §3.5. The planner decides CLOSE
//! iff the jerk-aware minimum close time plus seal time and margin fits before the
//! arrival:
//!
//! t_min(d, v0, a, v_max, j) + t_seal + margin ≤ t_a − t_detect,
//!
//! and the zone is clear (or a two-person override is active). Otherwise the device
//! uses its configured fallback (land in place / hold open landed). The arrival time
//! t_a is always an input (no scaled-distance or yield-to-distance law exists in
//! fabrail; the bound t_a ≤ R/c0 is only reported, [`arrival_bound_s`]). BASTION
//! closes 3 m in 5.246 s at a 0.48 m/s², j 2 m/s³ (5.031 s at a 0.53), so a
//! blast-lead close (5.25 s + 0.6 s seal and margin) needs R ≥ c0·5.85 s ≈ 2 km
//! ([`min_standoff_m`]). TIDEGATE closes only after the shaking ends and only on the
//! external decision-system command (the caller's gate; this planner only answers
//! "can it finish in time").
//!
//! Jerk-limited profile (PHYSICS §16; the rest-to-rest case equals
//! `fabrail_core::motion::scurve`): a speed change Δv with zero acceleration at both
//! ends takes t_j = a/j, t_c = Δv/a − a/j when Δv ≥ a²/j, else t_j = √(Δv/j), t_c = 0;
//! its duration is 2·t_j + t_c and, the velocity profile being point-symmetric, its
//! distance is the mean speed times the duration. The close is: v0 → v_p (≥ v0),
//! cruise at v_p, v_p → 0. With D(v_p) the distance without cruise (increasing in
//! v_p), v_p = v_max plus a cruise when D(v_max) ≤ d, else D(v_p) = d is solved by
//! bisection. Starting while opening (v0 < 0) first stops, then closes from rest over
//! the extra distance. A close that cannot stop within d from v0 has no solution: the
//! only escape is a reversal, i.e. an opening move, which an alert never commands, so
//! t_min is +∞ and the planner falls back.

use fabrail_types::fmath;

/// Speed of sound in dry air at 20 °C, m/s (c0 = 331.3·√(1 + θ/273.15) m/s, ISO
/// 9613-1). A blast front travels at c0 or faster, so R/c0 bounds the arrival time
/// from above (EXOTIC §X11).
pub const C0_AIR_MPS: f32 = 343.0;

/// Relative tolerance on v0 above v_max (sensor noise); beyond it the start state is
/// outside the planner's domain.
const V0_OVER_VMAX_TOL: f32 = 1.0e-3;

/// Bisection iterations for the peak speed (f32 resolution after ~30).
const PEAK_ITERS: u32 = 64;

/// Closure-race inputs (SI, f32).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ClosureInputs {
    /// Distance to close, m.
    pub d_m: f32,
    /// Current speed toward closed, m/s.
    pub v0_mps: f32,
    /// Acceleration limit, m/s².
    pub a_mps2: f32,
    /// Jerk limit, m/s³.
    pub j_mps3: f32,
    /// Speed limit, m/s.
    pub v_max_mps: f32,
    /// Seal (land-to-seal) time after arrival, s.
    pub t_seal_s: f32,
    /// Safety margin, s.
    pub margin_s: f32,
    /// Arrival time of the threat measured from now, s (an input).
    pub t_a_s: f32,
    /// Detection / decision latency already spent, s.
    pub t_detect_s: f32,
    /// The guarded zone is clear.
    pub zone_clear: bool,
    /// A two-person authority override is active (closure may win over presence).
    pub two_person_override: bool,
}

/// Why the planner did not close.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ClosureReject {
    /// t_min + t_seal + margin exceeds the available time.
    Infeasible = 0,
    /// The zone is not clear and no two-person override is active.
    ZoneNotClear = 1,
    /// A non-finite or negative input.
    InvalidInput = 2,
}

/// The planner's decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ClosureDecision {
    /// Close now; the planned close time (motion + seal), s.
    Close {
        /// Motion plus seal time, s.
        t_close_s: f32,
    },
    /// Do not start the race; land in place (or hold open landed).
    Fallback(ClosureReject),
}

/// Bang-bang (infinite-jerk, no speed limit) rest-to-rest time 2·√(d/a), s.
pub fn bang_bang_t_s(d_m: f32, a_mps2: f32) -> f32 {
    if !(d_m >= 0.0 && a_mps2 > 0.0) {
        return f32::INFINITY;
    }
    2.0 * fmath::sqrtf(d_m / a_mps2)
}

/// One jerk-limited speed change of |Δv| with zero acceleration at both ends:
/// (t_j, t_c, a_peak).
fn dv_phase(dv: f32, a: f32, j: f32) -> (f32, f32, f32) {
    let dv = fmath::fmaxf(dv, 0.0);
    if dv >= a * a / j {
        (a / j, dv / a - a / j, a)
    } else {
        let t_j = fmath::sqrtf(dv / j);
        (t_j, 0.0, j * t_j)
    }
}

/// Duration of a jerk-limited speed change of |Δv|, s.
fn dv_time(dv: f32, a: f32, j: f32) -> f32 {
    let (t_j, t_c, _) = dv_phase(dv, a, j);
    2.0 * t_j + t_c
}

/// Distance covered while changing speed from `v1` to `v2` (both ≥ 0), m.
fn dv_dist(v1: f32, v2: f32, a: f32, j: f32) -> f32 {
    0.5 * (v1 + v2) * dv_time(fmath::absf(v2 - v1), a, j)
}

/// A planned close: v0 → v_p, cruise, v_p → 0 (all constant-jerk segments).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SCurve {
    /// Start speed toward closed, m/s (≥ 0).
    pub v0_mps: f32,
    /// Peak (cruise) speed, m/s.
    pub v_peak_mps: f32,
    /// Acceleration limit, m/s².
    pub a_mps2: f32,
    /// Jerk limit, m/s³.
    pub j_mps3: f32,
    /// Speed-up phase duration (v0 → v_p), s.
    pub t_acc_s: f32,
    /// Cruise duration, s.
    pub t_cruise_s: f32,
    /// Stopping phase duration (v_p → 0), s.
    pub t_dec_s: f32,
    /// Distance, m.
    pub d_m: f32,
}

impl SCurve {
    /// Total motion time, s.
    pub fn total_s(&self) -> f32 {
        self.t_acc_s + self.t_cruise_s + self.t_dec_s
    }

    /// The seven constant-jerk segments (duration s, jerk m/s³).
    fn segments(&self) -> [(f32, f32); 7] {
        let (a, j) = (self.a_mps2, self.j_mps3);
        let (tj1, tc1, _) = dv_phase(self.v_peak_mps - self.v0_mps, a, j);
        let (tj2, tc2, _) = dv_phase(self.v_peak_mps, a, j);
        [
            (tj1, j),
            (tc1, 0.0),
            (tj1, -j),
            (self.t_cruise_s, 0.0),
            (tj2, -j),
            (tc2, 0.0),
            (tj2, j),
        ]
    }

    /// Position, speed and acceleration toward closed at `t_s` after the start
    /// (before 0: the start state; after the end: closed at rest).
    pub fn sample(&self, t_s: f32) -> (f32, f32, f32) {
        if t_s.is_nan() || t_s <= 0.0 {
            return (0.0, self.v0_mps, 0.0);
        }
        if t_s >= self.total_s() {
            return (self.d_m, 0.0, 0.0);
        }
        let (mut x, mut v, mut acc) = (0.0f32, self.v0_mps, 0.0f32);
        let mut left = t_s;
        for (dur, jerk) in self.segments() {
            let tau = fmath::fminf(left, dur);
            x += v * tau + 0.5 * acc * tau * tau + jerk * tau * tau * tau / 6.0;
            v += acc * tau + 0.5 * jerk * tau * tau;
            acc += jerk * tau;
            left -= tau;
            if left <= 0.0 {
                break;
            }
        }
        (x, v, acc)
    }
}

/// Plans the time-minimal jerk-limited close of `d_m` from `v0_mps` (≥ 0) to rest
/// under a, v_max, j (module doc). `None`: invalid limits, v0 above v_max, or no stop
/// within `d_m` without reversing.
pub fn s_curve(d_m: f32, v0_mps: f32, a_mps2: f32, v_max_mps: f32, j_mps3: f32) -> Option<SCurve> {
    let valid = d_m.is_finite()
        && d_m >= 0.0
        && v0_mps.is_finite()
        && v0_mps >= 0.0
        && a_mps2.is_finite()
        && a_mps2 > 0.0
        && j_mps3.is_finite()
        && j_mps3 > 0.0
        && v_max_mps.is_finite()
        && v_max_mps > 0.0;
    if !valid || v0_mps > v_max_mps * (1.0 + V0_OVER_VMAX_TOL) {
        return None;
    }
    let (a, j) = (a_mps2, j_mps3);
    let v0 = fmath::fminf(v0_mps, v_max_mps);
    let total_dist = |vp: f32| dv_dist(v0, vp, a, j) + dv_dist(vp, 0.0, a, j);
    if total_dist(v0) > d_m {
        return None;
    }
    let (v_p, cruise) = if total_dist(v_max_mps) <= d_m {
        (v_max_mps, (d_m - total_dist(v_max_mps)) / v_max_mps)
    } else {
        let (mut lo, mut hi) = (v0, v_max_mps);
        for _ in 0..PEAK_ITERS {
            let mid = 0.5 * (lo + hi);
            if mid <= lo || mid >= hi {
                break;
            }
            if total_dist(mid) <= d_m {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let rest = fmath::fmaxf(d_m - total_dist(lo), 0.0);
        let cruise = if lo > 0.0 { rest / lo } else { 0.0 };
        (lo, cruise)
    };
    Some(SCurve {
        v0_mps: v0,
        v_peak_mps: v_p,
        a_mps2: a,
        j_mps3: j,
        t_acc_s: dv_time(v_p - v0, a, j),
        t_cruise_s: cruise,
        t_dec_s: dv_time(v_p, a, j),
        d_m,
    })
}

/// Jerk-aware minimum time to cover `d_m` from `v0_mps` to rest, s (+∞ when no close
/// exists: invalid limits, v0 above v_max, or no stop within `d_m`). A negative `v0`
/// (opening) first stops, then closes from rest over the extra distance.
pub fn t_min_s(d_m: f32, v0_mps: f32, a_mps2: f32, v_max_mps: f32, j_mps3: f32) -> f32 {
    if v0_mps.is_finite() && v0_mps < 0.0 {
        if !(a_mps2 > 0.0 && j_mps3 > 0.0) {
            return f32::INFINITY;
        }
        let v = -v0_mps;
        let t_stop = dv_time(v, a_mps2, j_mps3);
        let d_stop = dv_dist(v, 0.0, a_mps2, j_mps3);
        return t_stop + t_min_s(d_m + d_stop, 0.0, a_mps2, v_max_mps, j_mps3);
    }
    match s_curve(d_m, v0_mps, a_mps2, v_max_mps, j_mps3) {
        Some(s) => s.total_s(),
        None => f32::INFINITY,
    }
}

/// Upper bound on the blast arrival time at range `r_m`, t_a ≤ R/c0, s (reported only;
/// the planner takes t_a as an input).
pub fn arrival_bound_s(r_m: f32) -> f32 {
    if !(r_m.is_finite() && r_m >= 0.0) {
        return f32::NAN;
    }
    r_m / C0_AIR_MPS
}

/// Range below which a close of `t_needed_s` (motion + seal + margin + detection)
/// cannot beat the front, R_min = c0·t_needed, m (necessary, not sufficient).
pub fn min_standoff_m(t_needed_s: f32) -> f32 {
    if !(t_needed_s.is_finite() && t_needed_s >= 0.0) {
        return f32::INFINITY;
    }
    C0_AIR_MPS * t_needed_s
}

/// Decides CLOSE iff feasible and the zone is clear (or overridden). `v0_mps` may be
/// negative (opening: stop, then close); every other input must be finite and ≥ 0.
pub fn plan(i: &ClosureInputs) -> ClosureDecision {
    let vals = [
        i.d_m,
        i.a_mps2,
        i.j_mps3,
        i.v_max_mps,
        i.t_seal_s,
        i.margin_s,
        i.t_a_s,
        i.t_detect_s,
    ];
    if vals.iter().any(|x| !x.is_finite() || *x < 0.0) || !i.v0_mps.is_finite() {
        return ClosureDecision::Fallback(ClosureReject::InvalidInput);
    }
    if !(i.zone_clear || i.two_person_override) {
        return ClosureDecision::Fallback(ClosureReject::ZoneNotClear);
    }
    let t_move = t_min_s(i.d_m, i.v0_mps, i.a_mps2, i.v_max_mps, i.j_mps3);
    let t_close = t_move + i.t_seal_s;
    if t_close + i.margin_s <= i.t_a_s - i.t_detect_s {
        ClosureDecision::Close { t_close_s: t_close }
    } else {
        ClosureDecision::Fallback(ClosureReject::Infeasible)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_core::motion::scurve;
    use fabrail_types::{PosNm, SCurveSeg, Tick, TravelDir};

    /// Rest-to-rest closed form (no cruise, v_p ≥ a²/j): v_p = (a/2)(√(a²/j² + 4d/a) −
    /// a/j), T = 2·(v_p/a + a/j) (PHYSICS §16), f64.
    fn rest_to_rest(d: f64, a: f64, j: f64) -> f64 {
        let vp = 0.5 * a * ((a * a / (j * j) + 4.0 * d / a).sqrt() - a / j);
        assert!(vp >= a * a / j);
        2.0 * (vp / a + a / j)
    }

    fn bastion(t_a: f32) -> ClosureInputs {
        ClosureInputs {
            d_m: 3.0,
            v0_mps: 0.0,
            a_mps2: 0.48,
            j_mps3: 2.0,
            v_max_mps: 1.2,
            t_seal_s: 0.4,
            margin_s: 0.2,
            t_a_s: t_a,
            t_detect_s: 0.0,
            zone_clear: true,
            two_person_override: false,
        }
    }

    // req: FAB-EXO-807
    #[test]
    fn close_iff_feasible() {
        // 1. BASTION anchors (EXOTIC §1, §2): 3 m in 5.246 s at a 0.48, j 2 (v_max
        //    1.2 m/s does not bind: v_p 1.144), 5.031 s at a 0.53; bang-bang 5.0 s is
        //    the infinite-jerk lower bound.
        let t = t_min_s(3.0, 0.0, 0.48, 1.2, 2.0);
        assert!(
            (t - 5.2457).abs() < 1e-3,
            "BASTION t_min {t} s (5.246 s, EXOTIC §1)"
        );
        assert!(((t as f64) - rest_to_rest(3.0, 0.48, 2.0)).abs() < 1e-4);
        let t53 = t_min_s(3.0, 0.0, 0.53, 1.2, 2.0);
        assert!((t53 - 5.031).abs() < 1e-3, "a 0.53: {t53} s (≈ 5.0 s)");
        let bb = bang_bang_t_s(3.0, 0.48);
        assert!((bb - 5.0).abs() < 1e-5 && t > bb);
        assert!(
            (t_min_s(3.0, 0.0, 0.48, 10.0, 1.0e6) - bb).abs() < 1e-3,
            "j → ∞"
        );

        // 2. Cross-check against the core rest-to-rest S-curve (independent code).
        for (d, v, a, j) in [
            (3.0f32, 1.2f32, 0.48f32, 2.0f32),
            (20.0, 0.5, 0.05, 0.1), // TIDEGATE preset: cruise at 0.5 m/s
            (0.01, 1.0, 1.0, 20.0), // triangular (a_peak < a)
            (8.0, 0.3, 0.02, 0.01), // ARENA-like limits
        ] {
            let seg = SCurveSeg {
                anchor: PosNm(0),
                t0: Tick(0),
                dir: TravelDir::Close,
                dist_m: d,
                v_mps: v,
                a_mps2: a,
                j_mps3: j,
            };
            let core_t = scurve::duration_s(&seg);
            let ours = t_min_s(d, 0.0, a, v, j);
            assert!(
                ((ours - core_t) / core_t).abs() < 1e-4,
                "d {d}: t_min {ours} vs core S-curve {core_t}"
            );
        }
        // TIDEGATE 20 m: 2·10.5 s ramps + 29.5 s cruise = 50.5 s.
        assert!((t_min_s(20.0, 0.0, 0.05, 0.5, 0.1) - 50.5).abs() < 1e-3);

        // 3. The planned profile is feasible: ends closed at rest, never exceeds v_max,
        //    a or j, never reverses; moving toward closed already only helps.
        for (d, v0, a, vm, j) in [
            (3.0f32, 0.0f32, 0.48f32, 1.2f32, 2.0f32),
            (3.0, 0.8, 0.48, 1.2, 2.0),
            (3.0, 1.2, 0.48, 1.2, 2.0),
            (20.0, 0.3, 0.05, 0.5, 0.1),
            (0.5, 0.1, 0.48, 1.2, 2.0),
        ] {
            let s = s_curve(d, v0, a, vm, j).unwrap_or_default();
            let n = 20_000;
            let dt = s.total_s() / n as f32;
            let (mut x_prev, mut a_prev) = (0.0f32, 0.0f32);
            for k in 0..=n {
                let (x, v, acc) = s.sample(k as f32 * dt);
                assert!(v <= vm * 1.0001 && v >= -1e-4, "{d}/{v0}: v {v}");
                assert!(acc.abs() <= a * 1.0001, "{d}/{v0}: a {acc}");
                assert!(x >= x_prev - 1e-5, "{d}/{v0}: reversal at {x}");
                if k > 0 {
                    assert!((acc - a_prev).abs() <= j * dt * 1.01 + 1e-5, "jerk");
                }
                x_prev = x;
                a_prev = acc;
            }
            let (x_end, v_end, a_end) = s.sample(s.total_s() * (1.0 - 1e-7));
            assert!(
                (x_end - d).abs() < 1e-3 * d.max(1.0),
                "{d}/{v0}: ends at {x_end}"
            );
            assert!(v_end.abs() < 1e-3 && a_end.abs() < 1e-3);
            if v0 > 0.0 {
                assert!(s.total_s() < t_min_s(d, 0.0, a, vm, j));
            }
        }
        // Opening (v0 < 0) costs the stop first; overshoot or v0 above v_max has no close.
        assert!(t_min_s(3.0, -0.5, 0.48, 1.2, 2.0) > t);
        assert_eq!(
            t_min_s(0.05, 1.2, 0.48, 1.2, 2.0),
            f32::INFINITY,
            "cannot stop in 5 cm"
        );
        assert_eq!(
            t_min_s(3.0, 2.0, 0.48, 1.2, 2.0),
            f32::INFINITY,
            "v0 > v_max"
        );
        assert_eq!(t_min_s(3.0, 0.0, 0.0, 1.2, 2.0), f32::INFINITY);
        assert_eq!(t_min_s(0.0, 0.0, 0.48, 1.2, 2.0), 0.0);

        // 4. CLOSE iff t_min + t_seal + margin ≤ t_a − t_detect (inclusive), and the zone
        //    is clear or a two-person override is active.
        let need = t + 0.4 + 0.2;
        match plan(&bastion(need)) {
            ClosureDecision::Close { t_close_s } => {
                assert!((t_close_s - (t + 0.4)).abs() < 1e-5, "{t_close_s}")
            }
            d => panic!("boundary must close: {d:?}"),
        }
        assert_eq!(
            plan(&bastion(need - 1e-3)),
            ClosureDecision::Fallback(ClosureReject::Infeasible)
        );
        let late = ClosureInputs {
            t_detect_s: 0.5,
            ..bastion(need + 0.4)
        };
        assert_eq!(
            plan(&late),
            ClosureDecision::Fallback(ClosureReject::Infeasible)
        );
        let blocked = ClosureInputs {
            zone_clear: false,
            ..bastion(60.0)
        };
        assert_eq!(
            plan(&blocked),
            ClosureDecision::Fallback(ClosureReject::ZoneNotClear)
        );
        let override_ = ClosureInputs {
            two_person_override: true,
            ..blocked
        };
        assert!(matches!(plan(&override_), ClosureDecision::Close { .. }));
        // The zone rule wins even when the close is infeasible anyway.
        let both = ClosureInputs {
            zone_clear: false,
            ..bastion(1.0)
        };
        assert_eq!(
            plan(&both),
            ClosureDecision::Fallback(ClosureReject::ZoneNotClear)
        );
        for bad in [
            ClosureInputs {
                d_m: -1.0,
                ..bastion(60.0)
            },
            ClosureInputs {
                t_a_s: f32::NAN,
                ..bastion(60.0)
            },
            ClosureInputs {
                v0_mps: f32::INFINITY,
                ..bastion(60.0)
            },
        ] {
            assert_eq!(
                plan(&bad),
                ClosureDecision::Fallback(ClosureReject::InvalidInput)
            );
        }
        // An infeasible profile (cannot stop in time) never closes, whatever t_a.
        let overshoot = ClosureInputs {
            d_m: 0.05,
            v0_mps: 1.2,
            ..bastion(1.0e6)
        };
        assert_eq!(
            plan(&overshoot),
            ClosureDecision::Fallback(ClosureReject::Infeasible)
        );
        // Opening at detection: the planner stops, then races (still feasible with time).
        let opening = ClosureInputs {
            v0_mps: -0.3,
            ..bastion(60.0)
        };
        assert!(matches!(plan(&opening), ClosureDecision::Close { .. }));

        // 5. Blast lead (t_a an input; t_a ≤ R/c0 reported): the BASTION close with
        //    0.6 s of seal and margin needs R ≥ c0·5.85 s ≈ 2 km (EXOTIC §2).
        let r_min = min_standoff_m(need);
        assert!(
            (2000.0..2020.0).contains(&r_min),
            "R_min {r_min} m (≥ 2 km, EXOTIC §2)"
        );
        assert!((arrival_bound_s(343.0) - 1.0).abs() < 1e-6);
        assert_eq!(
            plan(&bastion(arrival_bound_s(1500.0))),
            ClosureDecision::Fallback(ClosureReject::Infeasible),
            "1.5 km: the front arrives first"
        );
        assert!(matches!(
            plan(&bastion(arrival_bound_s(2100.0))),
            ClosureDecision::Close { .. }
        ));
    }
}
