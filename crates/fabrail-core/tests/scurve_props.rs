//! S-curve properties (PHYSICS §16; FAB-PRP-005): the 7-segment jerk-limited profile
//! respects its v, a and j limits, starts at its anchor and lands on its end point
//! exactly, and its decelerating half equals the closed-form stop distance
//! d_stop = v²/(2a) + v·a/(2j) (v ≥ a²/j) or v·√(v/j), including the class anchors
//! 33.43 / 36.31 / 8.86 / 4.91 mm at the speed caps (core design §E.5). Owner: A2.
//!
//! Tolerances: f32 evaluation relative to the anchor, 1e-6 relative (core design §E.2),
//! plus the 1 nm rounding of `PosNm`; limits within 1e-5 relative; d_stop against the
//! f64 twin `params::derive::stop_distance` to 1e-6 relative (f32 arithmetic).

use fabrail_core::motion::scurve::{self, duration_s, eval, stop_distance_m};
use fabrail_params::{derive, presets};
use fabrail_types::{BASE_TICK_HZ, PosNm, Segment, Tick};
use proptest::prelude::*;

/// Samples `seg` at `n + 1` evenly spaced ticks over its duration plus a margin.
fn sample(seg: &fabrail_types::SCurveSeg, n: u64) -> Vec<(Tick, fabrail_core::motion::RefState)> {
    let dur_ticks = (f64::from(duration_s(seg)) * f64::from(BASE_TICK_HZ)).ceil() as u64 + 2;
    let step = (dur_ticks / n).max(1);
    (0..=dur_ticks / step + 1)
        .map(|k| {
            let t = seg.t0.plus(k * step);
            (t, eval(seg, t))
        })
        .collect()
}

// req: FAB-PRP-005
proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]
    /// Random moves (1 µm to 15 m, anchors up to ±1000 km, start ticks up to 2⁴⁰) under
    /// random limits: |v| ≤ V, |a| ≤ A, |Δa/Δt| ≤ J, monotone progress, position
    /// consistent with the integral of v, exact start and end, and the decelerating
    /// half equal to d_stop. (E) f32 rel 1e-6 relative to the anchor, PHYSICS §16.
    #[test]
    fn limits_and_stop_distance(
        anchor_nm in -1_000_000_000_000_000i64..1_000_000_000_000_000,
        dist_um in prop_oneof![1u64..1_000, 1_000u64..100_000, 100_000u64..15_000_000],
        close in any::<bool>(),
        t0 in 0u64..(1u64 << 40),
        v in 0.01f32..1.0,
        a in 0.05f32..3.0,
        j in 0.5f32..40.0,
    ) {
        let anchor = PosNm(anchor_nm);
        let d_nm = dist_um as i64 * 1_000;
        let target = PosNm(anchor_nm + if close { -d_nm } else { d_nm });
        let seg = scurve::plan(anchor, Tick(t0), target, v, a, j);
        let end = Segment::SCurve(seg).end();
        // The end point is the target to the f32 rounding of the distance (≤ 0.5 ulp).
        let ulp_nm = (f64::from(seg.dist_m) * 1e9 * f64::from(f32::EPSILON)).max(1.0);
        prop_assert!((end.diff_nm(target).abs() as f64) <= ulp_nm,
            "end {:?} vs target {:?} (f32 rounding of dist)", end, target);

        // Start: anchor at rest, also before t0.
        let s0 = eval(&seg, Tick(t0));
        prop_assert_eq!(s0.x, anchor);
        prop_assert_eq!((s0.v_mps, s0.a_mps2), (0.0, 0.0));
        prop_assert_eq!(eval(&seg, Tick(t0.saturating_sub(7))).x, anchor);

        // End: exactly the end point at rest, done.
        let t_end = Tick(t0 + (f64::from(duration_s(&seg)) * f64::from(BASE_TICK_HZ)).ceil() as u64 + 1);
        let se = eval(&seg, t_end);
        prop_assert_eq!(se.x, end, "exact end point (PHYSICS §24)");
        prop_assert!(se.done && se.v_mps == 0.0 && se.a_mps2 == 0.0);

        let sign = seg.dir.sign();
        let dist = f64::from(seg.dist_m);
        let pos_tol_m = 1e-6 * dist + 2e-9;
        let samples = sample(&seg, 800);
        let mut prev: Option<(Tick, fabrail_core::motion::RefState)> = None;
        for &(t, r) in &samples {
            prop_assert!(f64::from(r.v_mps.abs()) <= f64::from(v) * (1.0 + 1e-5),
                "|v| {} > V {} (PRP-005)", r.v_mps, v);
            prop_assert!(f64::from(r.a_mps2.abs()) <= f64::from(a) * (1.0 + 1e-5),
                "|a| {} > A {} (PRP-005)", r.a_mps2, a);
            prop_assert!(r.v_mps * sign >= 0.0, "velocity never opposes the move");
            // Progress stays within [anchor, end].
            let s = f64::from(sign) * r.x.diff_nm(anchor) as f64 * 1e-9;
            prop_assert!(s >= -pos_tol_m && s <= dist + pos_tol_m, "s = {} outside [0, {}]", s, dist);
            if let Some((tp, rp)) = prev {
                let h = t.since_s(tp) as f64;
                // Jerk: |Δa| ≤ J·Δt (+ f32 rounding of a).
                let da = f64::from(r.a_mps2 - rp.a_mps2).abs();
                prop_assert!(da <= f64::from(j) * h * (1.0 + 1e-4) + 1e-5 * f64::from(a),
                    "|Δa/Δt| {} > J {} (PRP-005)", da / h, j);
                // Monotone progress.
                let ds = f64::from(sign) * r.x.diff_nm(rp.x) as f64 * 1e-9;
                prop_assert!(ds >= -pos_tol_m, "position went backwards by {}", -ds);
                // Δx matches the trapezoid of v to the O(J·h³) and f32 rounding level.
                let trap = 0.5 * f64::from(r.v_mps + rp.v_mps) * h;
                let tol = f64::from(j) * h * h * h / 6.0 + 2.0 * pos_tol_m + 1e-7 * f64::from(v) * h;
                prop_assert!((ds - f64::from(sign) * trap).abs() <= tol,
                    "Δx {} vs ∫v {} (tol {})", ds, trap * f64::from(sign), tol);
            }
            prev = Some((t, r));
        }

        // Stop distance: f32 closed form = f64 derive twin (1e-6 rel).
        let ds32 = f64::from(stop_distance_m(v, a, j));
        let ds64 = derive::stop_distance(f64::from(v), f64::from(a), f64::from(j));
        prop_assert!((ds32 - ds64).abs() <= 1e-6 * ds64, "d_stop f32 {} vs f64 {}", ds32, ds64);
        // When the cap is reached, the decelerating half covers exactly d_stop(V).
        if dist > 2.0 * ds64 * (1.0 + 1e-4) {
            let t_a_ticks = {
                // Accelerating half duration: v/A + A/J (trapezoidal) or 2√(v/J).
                let (vf, af, jf) = (f64::from(v), f64::from(a), f64::from(j));
                let t_a = if vf * jf >= af * af { vf / af + af / jf } else { 2.0 * (vf / jf).sqrt() };
                (t_a * f64::from(BASE_TICK_HZ)).round() as u64
            };
            let t_end_exact = f64::from(duration_s(&seg)) * f64::from(BASE_TICK_HZ);
            let t_dec = Tick(t0 + (t_end_exact.round() as u64).saturating_sub(t_a_ticks));
            let r = eval(&seg, t_dec);
            let remaining = f64::from(sign) * end.diff_nm(r.x) as f64 * 1e-9;
            // Tick rounding of both instants (≤ 1 tick) plus the f32 rounding of the
            // duration (½ ulp of T), at the cruise speed; plus f32 position rounding.
            let t_unc = 1.0 / f64::from(BASE_TICK_HZ) + t_end_exact / f64::from(BASE_TICK_HZ) * 2f64.powi(-23);
            let tol = f64::from(v) * t_unc + 1e-6 * ds64 + pos_tol_m;
            prop_assert!((remaining - ds64).abs() <= tol,
                "decelerating half {} vs d_stop {} (tol {}; PHYSICS §16)", remaining, ds64, tol);
            prop_assert!((f64::from(r.v_mps.abs()) - f64::from(v)).abs() <= 1e-4 * f64::from(v) + 1e-6);
        }
    }
}

/// d_stop at the class speed caps: 33.43 / 36.31 / 8.86 / 4.91 mm at the rounded caps
/// 0.2654 / 0.1838 / 0.0446 / 0.025 m/s (core design §E.5), f32 vs the f64 closed form
/// to 1e-6 relative at the exact preset caps and at the rounded anchor caps, and the
/// anchors to their 0.005 mm rounding.
#[test]
fn stop_distance_anchors() {
    let cases = [
        (presets::drawer(), 0.2654, 33.43),
        (presets::door(), 0.1838, 36.31),
        (presets::wall(), 0.0446, 8.86),
        (presets::heavy(), 0.025, 4.91),
    ];
    for (p, v_anchor, want_mm) in cases {
        let (a, j) = (p.control.motion.a_max_mps2, p.control.motion.j_max_mps3);
        for v in [p.safety.v_open_max_mps, v_anchor] {
            let d64 = derive::stop_distance(v, a, j);
            let d32 = f64::from(stop_distance_m(v as f32, a as f32, j as f32));
            assert!(
                (d32 - d64).abs() <= 1e-6 * d64,
                "{:?} v = {v}: f32 {d32} vs f64 {d64} ((E) f32 rel 1e-6, PHYSICS §16)",
                p.design.class
            );
        }
        let d = derive::stop_distance(v_anchor, a, j) * 1e3;
        assert!(
            (d - want_mm).abs() <= 0.005,
            "{:?}: d_stop {d} mm vs anchor {want_mm} mm (core design §E.5)",
            p.design.class
        );
    }
}
