//! Tsunami timing: arrival by integral dx/sqrt(g*h), Green's law, closure budget (FAB-EXO-809). Owner: X6 (API frozen by W0a).
//!
//! EXOTIC §X12 (timing only; no inundation, run-up or hydrodynamic load). Long-wave speed
//! c = √(g·h) (198 m/s at 4 km), arrival t = ∫dx/√(g·h(x)), Green's law amplitude ratio
//! A1/A0 = (h0/h1)^(1/4) (×4.47 from 4,000 m to 10 m), and the closure budget
//! t_arrival − shaking duration − latency.
//!
//! **Rail distortion** ([`RailDistortion`]). Coastal subsidence of 0.3–1.2 m comes with
//! differential settlement and tilt. A rigid gate levitated on N pods can follow a rail
//! that heaves and pitches (its gap loops take up a best-fit line), but not a step: the
//! per-pod residual about the least-squares line is the gap error ([`gap_check`]). The gap
//! check passes when the largest residual is within a fraction of the design gap; then
//! TIDEGATE closes by maglev, otherwise it lands on its wheels (or rack) and closes on
//! them at a reduced speed ([`plan_tidegate_close`]).
//!
//! **Decision rule** (plan §4; FAB-EXO-809): TIDEGATE closes only after the shaking ends
//! and only on the command of an external decision system (decision path designed toward
//! a SIL 3 target; SIL 4 is unclaimable). Without the command it stays landed. Debris and
//! salt in an attractive-EMS gap are an unsolved objection, and buoyant or hydraulic gates
//! beat maglev physically: the transferable product is the fused trigger stack.

use fabrail_exo::closure;
use fabrail_params::derive::G0_MPS2;
use fabrail_types::fmath;

/// Long-wave speed √(g·h), m/s.
pub fn wave_speed_mps(depth_m: f64) -> f64 {
    if depth_m <= 0.0 {
        return 0.0;
    }
    fmath::sqrt(G0_MPS2 * depth_m)
}

/// Green's-law amplitude ratio (h0/h1)^(1/4) from depth `h0_m` to `h1_m`.
pub fn greens_law_factor(h0_m: f64, h1_m: f64) -> f64 {
    if h0_m <= 0.0 || h1_m <= 0.0 {
        return f64::INFINITY;
    }
    fmath::sqrt(fmath::sqrt(h0_m / h1_m))
}

/// Arrival time over a bathymetry profile of (distance m, depth m) points by the
/// trapezoidal rule on 1/√(g·h), s (+∞ on a non-positive depth).
pub fn arrival_time_s(profile: &[(f64, f64)]) -> f64 {
    let mut t = 0.0;
    for w in profile.windows(2) {
        let [(x0, h0), (x1, h1)] = [w[0], w[1]];
        let (c0, c1) = (wave_speed_mps(h0), wave_speed_mps(h1));
        if c0 <= 0.0 || c1 <= 0.0 {
            return f64::INFINITY;
        }
        t += 0.5 * fmath::abs(x1 - x0) * (1.0 / c0 + 1.0 / c1);
    }
    t
}

/// Closure budget t_arrival − t_shaking − t_latency, s (negative: no time to close).
pub fn closure_budget_s(t_arrival_s: f64, t_shaking_s: f64, t_latency_s: f64) -> f64 {
    t_arrival_s - t_shaking_s - t_latency_s
}

/// Exact travel time over a linear slope from depth `h0_m` to `h1_m` across `l_m`:
/// ∫dx/√(g·(h0 + s·x)) = 2·(√(g·h1) − √(g·h0))/(g·s), s = (h1 − h0)/L (L/√(g·h0) on a
/// flat bottom), s. The closed form the trapezoid of [`arrival_time_s`] converges to.
pub fn linear_slope_time_s(l_m: f64, h0_m: f64, h1_m: f64) -> f64 {
    if !(h0_m > 0.0 && h1_m > 0.0 && l_m >= 0.0) {
        return f64::INFINITY;
    }
    let slope = (h1_m - h0_m) / l_m.max(f64::MIN_POSITIVE);
    if fmath::abs(h1_m - h0_m) <= 1.0e-12 * h0_m.max(h1_m) {
        return l_m / wave_speed_mps(h0_m);
    }
    2.0 * (wave_speed_mps(h1_m) - wave_speed_mps(h0_m)) / (G0_MPS2 * slope)
}

/// Resamples a piecewise-linear profile to at most `step_m` between points (so the
/// trapezoid converges on steep slopes).
pub fn refine_profile(profile: &[(f64, f64)], step_m: f64) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    if profile.is_empty() {
        return out;
    }
    out.push(profile[0]);
    for w in profile.windows(2) {
        let [(x0, h0), (x1, h1)] = [w[0], w[1]];
        let n = if step_m > 0.0 {
            fmath::ceil(fmath::abs(x1 - x0) / step_m).clamp(1.0, 1.0e6) as usize
        } else {
            1
        };
        for i in 1..=n {
            let f = i as f64 / n as f64;
            out.push((x0 + f * (x1 - x0), h0 + f * (h1 - h0)));
        }
    }
    out
}

/// A reference subduction-zone path from the source to the coast (distance m, depth m):
/// 150 km of 4,000 m deep ocean, a 50 km continental slope to 200 m, a 30 km shelf to
/// 50 m and the last 5 km to 10 m at the coast \[UNCERTAIN: illustrative bathymetry\].
pub fn reference_profile() -> Vec<(f64, f64)> {
    vec![
        (0.0, 4000.0),
        (150.0e3, 4000.0),
        (200.0e3, 200.0),
        (230.0e3, 50.0),
        (235.0e3, 10.0),
    ]
}

/// Rail distortion injected after the shaking (EXOTIC §X12; exotic design §2.8).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RailDistortion {
    /// Regional subsidence, m (0.3–1.2 m).
    pub subsidence_m: f64,
    /// Differential settlement across the gate span as a fraction of the subsidence (a
    /// step at `step_frac` of the span) \[UNCERTAIN: 5 % default\].
    pub differential_frac: f64,
    /// Position of the step along the span, fraction 0..1.
    pub step_frac: f64,
    /// Longitudinal tilt of the rail, rad (taken up by the gate's pitch).
    pub tilt_rad: f64,
}

impl RailDistortion {
    /// Vertical rail displacement at `x_m` along a span of `span_m`, m (down negative).
    pub fn rail_z_m(&self, x_m: f64, span_m: f64) -> f64 {
        let step = if x_m >= self.step_frac * span_m {
            -self.differential_frac * self.subsidence_m
        } else {
            0.0
        };
        -self.subsidence_m + step + self.tilt_rad * x_m
    }
}

/// Result of the post-shaking gap check.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GapCheck {
    /// Largest pod gap error after the gate's best-fit heave and pitch, m.
    pub max_gap_error_m: f64,
    /// Allowed error, m.
    pub tolerance_m: f64,
    /// The check passed (maglev close allowed).
    pub pass: bool,
}

/// Gap check of a rigid gate of span `span_m` on `n_pods` evenly spaced pods over a
/// distorted rail: the residual of the least-squares line through the pods' rail heights
/// (the gate heaves and pitches; a step cannot be followed) against
/// `tol_frac`·`gap_m`.
pub fn gap_check(
    d: &RailDistortion,
    span_m: f64,
    n_pods: usize,
    gap_m: f64,
    tol_frac: f64,
) -> GapCheck {
    let n = n_pods.max(2);
    let xs: Vec<f64> = (0..n).map(|i| span_m * i as f64 / (n - 1) as f64).collect();
    let zs: Vec<f64> = xs.iter().map(|x| d.rail_z_m(*x, span_m)).collect();
    let nf = n as f64;
    let mx = xs.iter().sum::<f64>() / nf;
    let mz = zs.iter().sum::<f64>() / nf;
    let sxx: f64 = xs.iter().map(|x| (x - mx) * (x - mx)).sum();
    let sxz: f64 = xs.iter().zip(&zs).map(|(x, z)| (x - mx) * (z - mz)).sum();
    let b = if sxx > 0.0 { sxz / sxx } else { 0.0 };
    let max_err = xs
        .iter()
        .zip(&zs)
        .map(|(x, z)| fmath::abs(z - (mz + b * (x - mx))))
        .fold(0.0_f64, f64::max);
    let tol = tol_frac * gap_m;
    GapCheck {
        max_gap_error_m: max_err,
        tolerance_m: tol,
        pass: max_err <= tol,
    }
}

/// How TIDEGATE closed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum CloseMode {
    /// Levitated close after a passed gap check.
    Maglev,
    /// Wheel (or rack) close after a failed gap check.
    Wheels,
    /// No close: no external command (the gate stays landed).
    #[default]
    NoCommand,
}

impl CloseMode {
    /// Report name.
    pub const fn name(self) -> &'static str {
        match self {
            CloseMode::Maglev => "maglev",
            CloseMode::Wheels => "wheels",
            CloseMode::NoCommand => "no_command",
        }
    }
}

/// Inputs of the TIDEGATE closure plan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TidegateInputs {
    /// Tsunami arrival after origin, s.
    pub t_arrival_s: f64,
    /// Shaking duration after origin, s.
    pub t_shaking_s: f64,
    /// Alert and decision latency, s.
    pub t_latency_s: f64,
    /// External decision-system close command after origin, s (`None`: never sent).
    pub t_command_s: Option<f64>,
    /// Gate travel to close, m.
    pub d_m: f64,
    /// Maglev speed, m/s.
    pub v_mps: f64,
    /// Maglev acceleration, m/s².
    pub a_mps2: f64,
    /// Maglev jerk, m/s³.
    pub j_mps3: f64,
    /// Wheel-mode speed as a fraction of the maglev speed \[UNCERTAIN\].
    pub wheel_speed_frac: f64,
    /// Time to land onto the wheels and engage the rack, s \[UNCERTAIN\].
    pub t_wheel_switch_s: f64,
    /// Seal time after arrival at closed, s.
    pub t_seal_s: f64,
    /// Margin before the arrival, s.
    pub margin_s: f64,
    /// Levitation gap, m.
    pub gap_m: f64,
    /// Allowed gap error as a fraction of the gap.
    pub gap_tol_frac: f64,
    /// Gate span, m.
    pub span_m: f64,
    /// Pods along the span.
    pub n_pods: usize,
    /// Rail distortion.
    pub distortion: RailDistortion,
}

impl TidegateInputs {
    /// TIDEGATE preset kinematics (20 m, 0.5 m/s, 0.05 m/s², 0.1 m/s³), a 22.5 mm gap with
    /// a 30 % error allowance, a 20 m span on 10 pods, wheels at half speed after a 10 s
    /// switch, 5 s seal, 60 s margin \[UNCERTAIN design picks\].
    pub fn tidegate(t_arrival_s: f64, t_shaking_s: f64, t_latency_s: f64) -> TidegateInputs {
        TidegateInputs {
            t_arrival_s,
            t_shaking_s,
            t_latency_s,
            t_command_s: Some(t_shaking_s + t_latency_s),
            d_m: 20.0,
            v_mps: 0.5,
            a_mps2: 0.05,
            j_mps3: 0.1,
            wheel_speed_frac: 0.5,
            t_wheel_switch_s: 10.0,
            t_seal_s: 5.0,
            margin_s: 60.0,
            gap_m: 0.0225,
            gap_tol_frac: 0.3,
            span_m: 20.0,
            n_pods: 10,
            distortion: RailDistortion::default(),
        }
    }
}

/// The TIDEGATE closure plan.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TidegatePlan {
    /// The gap check.
    pub gap: GapCheck,
    /// Close mode.
    pub mode: CloseMode,
    /// Close start after origin, s (∞ with no command).
    pub t_start_s: f64,
    /// Motion time (including the wheel switch), s.
    pub t_move_s: f64,
    /// Closed and sealed after origin, s.
    pub t_closed_s: f64,
    /// Closure budget t_arrival − t_shaking − t_latency, s.
    pub budget_s: f64,
    /// Closed and sealed with the margin before the arrival.
    pub within_budget: bool,
}

/// Plans the TIDEGATE close: the gap check after the shaking picks maglev or wheels; the
/// close starts at max(end of shaking, command); no command → no close.
pub fn plan_tidegate_close(i: &TidegateInputs) -> TidegatePlan {
    let gap = gap_check(&i.distortion, i.span_m, i.n_pods, i.gap_m, i.gap_tol_frac);
    let budget = closure_budget_s(i.t_arrival_s, i.t_shaking_s, i.t_latency_s);
    let Some(t_cmd) = i.t_command_s.filter(|t| t.is_finite()) else {
        return TidegatePlan {
            gap,
            mode: CloseMode::NoCommand,
            t_start_s: f64::INFINITY,
            t_move_s: f64::INFINITY,
            t_closed_s: f64::INFINITY,
            budget_s: budget,
            within_budget: false,
        };
    };
    let t_start = t_cmd.max(i.t_shaking_s);
    let (mode, t_move) = if gap.pass {
        let t = closure::t_min_s(
            i.d_m as f32,
            0.0,
            i.a_mps2 as f32,
            i.v_mps as f32,
            i.j_mps3 as f32,
        );
        (CloseMode::Maglev, f64::from(t))
    } else {
        let v = i.v_mps * i.wheel_speed_frac.clamp(0.0, 1.0);
        let t = closure::t_min_s(
            i.d_m as f32,
            0.0,
            i.a_mps2 as f32,
            v as f32,
            i.j_mps3 as f32,
        );
        (CloseMode::Wheels, f64::from(t) + i.t_wheel_switch_s)
    };
    let t_closed = t_start + t_move + i.t_seal_s;
    TidegatePlan {
        gap,
        mode,
        t_start_s: t_start,
        t_move_s: t_move,
        t_closed_s: t_closed,
        budget_s: budget,
        within_budget: t_closed + i.margin_s <= i.t_arrival_s,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    // req: FAB-EXO-809
    #[test]
    fn timing_greens_law_budget() {
        // c = √(g·h): 198 m/s at 4 km, 99 m/s at 1 km, 22 m/s at 50 m (EXOTIC §X12).
        let c4 = wave_speed_mps(4000.0);
        assert!(
            rel(c4, (G0_MPS2 * 4000.0).sqrt()) < 1e-9,
            "√(g·h) closed form rel 1e-9 (FAB-EXO-809)"
        );
        assert!(rel(c4, 198.0) < 1.0e-3, "198 m/s at 4 km (±0.1 %): {c4}");
        assert!(rel(wave_speed_mps(1000.0), 99.0) < 2.0e-3);
        assert!(rel(wave_speed_mps(50.0), 22.1) < 2.0e-3);
        // Green's law ×4.47 from 4,000 m to 10 m, (h0/h1)^(1/4) rel 1e-9.
        let gl = greens_law_factor(4000.0, 10.0);
        assert!(rel(gl, 400f64.powf(0.25)) < 1e-9, "Green's law rel 1e-9");
        assert!((gl - 4.47).abs() < 0.005, "×4.47 (FAB-EXO-809): {gl}");
        // ∫dx/√(g·h): 100 km over a uniform 1,000 m shelf takes about 17 min.
        let t_flat = arrival_time_s(&[(0.0, 1000.0), (100.0e3, 1000.0)]);
        assert!(
            rel(t_flat, 100.0e3 / (G0_MPS2 * 1000.0).sqrt()) < 1e-12
                && (t_flat / 60.0 - 17.0).abs() < 0.5,
            "100 km at 1,000 m ≈ 17 min: {:.1} min",
            t_flat / 60.0
        );
        // The trapezoid converges to the exact linear-slope integral.
        let exact = linear_slope_time_s(50.0e3, 4000.0, 200.0);
        let coarse = arrival_time_s(&[(0.0, 4000.0), (50.0e3, 200.0)]);
        let fine = arrival_time_s(&refine_profile(&[(0.0, 4000.0), (50.0e3, 200.0)], 100.0));
        println!(
            "slope 4,000 → 200 m over 50 km: exact {exact:.2} s, 1 panel {coarse:.2} s, 100 m panels {fine:.3} s"
        );
        assert!(
            rel(fine, exact) < 1e-5,
            "trapezoid → closed form (rel 1e-5)"
        );
        assert!(
            rel(coarse, exact) > rel(fine, exact),
            "refinement converges"
        );
        // Reference path and the closure budget t_arrival − t_shaking − t_latency.
        let t_arr = arrival_time_s(&refine_profile(&reference_profile(), 100.0));
        let (t_shake, t_lat) = (180.0, 30.0);
        let budget = closure_budget_s(t_arr, t_shake, t_lat);
        println!(
            "inputs: reference profile (235 km, 4,000 → 10 m), shaking {t_shake} s, latency {t_lat} s → arrival {:.1} min, budget {:.1} min, Green's ×{:.2}",
            t_arr / 60.0,
            budget / 60.0,
            greens_law_factor(4000.0, 10.0)
        );
        assert!(
            rel(budget, t_arr - t_shake - t_lat) < 1e-12,
            "budget closed form"
        );
        assert!(
            budget > 600.0,
            "minutes to spare at this path: time is not the problem"
        );
        // No distortion: the gap check passes and TIDEGATE closes by maglev within budget,
        // starting only after the shaking and on the external command.
        let mut inp = TidegateInputs::tidegate(t_arr, t_shake, t_lat);
        let p = plan_tidegate_close(&inp);
        assert_eq!(p.mode, CloseMode::Maglev);
        assert!(p.gap.pass && p.within_budget);
        assert!(p.t_start_s >= t_shake, "closes only after the shaking");
        // An early command still waits for the end of the shaking.
        inp.t_command_s = Some(10.0);
        assert!(plan_tidegate_close(&inp).t_start_s >= t_shake);
        // No external command: no close, whatever the timing.
        inp.t_command_s = None;
        let p = plan_tidegate_close(&inp);
        assert_eq!(
            p.mode,
            CloseMode::NoCommand,
            "closes only on the external command"
        );
        assert!(!p.within_budget && p.t_start_s.is_infinite());
        // 0.3–1.2 m subsidence with 5 % differential settlement: from 0.6 m the step
        // defeats the gap check (at 0.3 m it passes narrowly), so TIDEGATE falls back to
        // its wheels, and still closes in budget at this path.
        inp.t_command_s = Some(t_shake + t_lat);
        for sub in [0.3, 0.6, 1.2] {
            inp.distortion = RailDistortion {
                subsidence_m: sub,
                differential_frac: 0.05,
                step_frac: 0.5,
                tilt_rad: 2.0e-3,
            };
            let p = plan_tidegate_close(&inp);
            println!(
                "subsidence {sub} m: gap error {:.1} mm (tol {:.2} mm) → {} close in {:.0} s, closed at {:.0} s",
                p.gap.max_gap_error_m * 1e3,
                p.gap.tolerance_m * 1e3,
                p.mode.name(),
                p.t_move_s,
                p.t_closed_s
            );
            assert_eq!(
                p.mode == CloseMode::Wheels,
                !p.gap.pass,
                "wheels exactly when the gap check fails"
            );
            if sub >= 0.6 {
                assert_eq!(
                    p.mode,
                    CloseMode::Wheels,
                    "wheel fallback on rail distortion"
                );
            }
            assert!(p.within_budget);
        }
        // Uniform subsidence and tilt alone are taken up by heave and pitch.
        let g = gap_check(
            &RailDistortion {
                subsidence_m: 1.2,
                differential_frac: 0.0,
                step_frac: 0.5,
                tilt_rad: 5.0e-3,
            },
            20.0,
            10,
            0.0225,
            0.3,
        );
        assert!(
            g.pass && g.max_gap_error_m < 1e-9,
            "a plane is followed exactly"
        );
    }
}
