//! Mizuno current-integral zero-power law and the modified zero-power band outer PI (PHYSICS §15). Owner: A1.
//!
//! Frozen API (W0): [`ZeroPower`] per mode, i* = k_p·δg + k_d·δġ + κ·q with
//! q̇ = i − i_bias, and the modified-ZP outer PI (bandwidth `zp_outer_bw_hz`, 1 Hz)
//! active only outside `zp_band_m` (core design F1: DRAWER band 1.5–6.0 mm, +0.39 A
//! bias at 48 kg). Test: `bias_current_closed_form` (1e-4 vs
//! `params::derive::bias_current`).
//!
//! Law (Mizuno & Higuchi 1984; Mizuno et al., "Zero-power control", JSME 1990): the
//! integral of the MEASURED coil current enters the command, so at equilibrium the
//! current returns to i_bias (zero in the band) and the gap floats to the zero-power gap
//! of the load, m(g) = m*·((2g* + h′)/(2g + h′))² (PHYSICS §3). Order per release (the
//! discrete design of `derive::zp_gains_discrete`): u\[k\] = k_p·δg\[k\] + k_d·δġ\[k\] + κ·q\[k\],
//! then q\[k+1\] = q\[k\] + T·(i\[k\] − i_bias). Because q integrates the actual current, the
//! law has no integrator windup when the reference is clamped.
//!
//! Modified zero power (core design F1): outside the band [g_min, g_max] a slow outer
//! loop sets i_bias so the gap is held at the band edge. It is a one-sided PI with
//! integral gain K_I = ω_o·k_s/k_i (the quasi-static plant from i_bias to gap is
//! ∂g/∂i = k_i/k_s at a force balance, so the loop crosses over at ω_o = 2π·1 Hz, well
//! below the zero-power integral pole αω ≥ 24 rad/s) and proportional gain
//! K_P = 0.25·k_s/k_i (a loop-gain fraction of 0.25, design choice for noise). It
//! engages below g_min (bias ≥ 0) or above g_max (bias ≤ 0) and keeps regulating to that
//! edge until its bias has bled back to zero, so it never chatters at the edge and the
//! steady bias equals the closed form `derive::bias_current(g_edge, m)`.

use crate::levitation::design::ModeGains;
use fabrail_types::fmath;

/// Zero-power controller for one mode.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ZeroPower {
    /// Gains.
    pub gains: ModeGains,
    /// Current integral q, A·s.
    pub q_a_s: f32,
    /// Bias current i_bias from the modified-ZP outer loop, A.
    pub i_bias_a: f32,
    /// Band [g_min, g_max] deviation limits relative to the set point, m (disabled when
    /// g_min ≥ g_max).
    pub band_m: [f32; 2],
    /// Period, s.
    pub dt_s: f32,
    /// Outer-loop integral gain K_I, A/(m·s) (0 disables the band clamp).
    pub k_outer_i: f32,
    /// Outer-loop proportional gain K_P, A/m.
    pub k_outer_p: f32,
    /// Outer-loop integrator state (the bias it holds), A.
    pub x_outer_a: f32,
}

impl ZeroPower {
    /// A controller with `gains`, band limits and period.
    pub fn new(gains: ModeGains, band_m: [f32; 2], dt_s: f32) -> ZeroPower {
        ZeroPower {
            gains,
            band_m,
            dt_s,
            ..ZeroPower::default()
        }
    }

    /// Enables the modified zero-power band clamp with bandwidth `bw_hz` for a mode with
    /// per-element negative stiffness `k_s` (N/m) and current gain `k_i` (N/A).
    pub fn with_outer(mut self, bw_hz: f32, k_s_n_per_m: f32, k_i_n_per_a: f32) -> ZeroPower {
        if bw_hz > 0.0 && k_s_n_per_m > 0.0 && k_i_n_per_a > 0.0 && self.band_m[0] < self.band_m[1]
        {
            let r = k_s_n_per_m / k_i_n_per_a;
            self.k_outer_i = core::f32::consts::TAU * bw_hz * r;
            self.k_outer_p = 0.25 * r;
        }
        self
    }

    /// One update: modal deviation `dg` (m), rate `dg_dot` (m/s), measured modal current
    /// `i_a`; returns the modal current demand, A.
    pub fn step(&mut self, dg: f32, dg_dot: f32, i_a: f32) -> f32 {
        let i_bias = self.outer(dg);
        let g = &self.gains;
        let u = g.kp_a_per_m * dg + g.kd_a_s_per_m * dg_dot + g.kappa_per_s * self.q_a_s;
        self.q_a_s += self.dt_s * (i_a - i_bias);
        self.i_bias_a = i_bias;
        if u.is_finite() {
            u
        } else {
            self.reset(0.0);
            0.0
        }
    }

    /// The one-sided band outer loop; returns i_bias.
    fn outer(&mut self, dg: f32) -> f32 {
        if self.k_outer_i <= 0.0 {
            return 0.0;
        }
        let [lo, hi] = self.band_m;
        if dg < lo || self.x_outer_a > 0.0 {
            // Holding the lower edge (heavy load): bias ≥ 0.
            let eps = lo - dg;
            self.x_outer_a = fmath::fmaxf(self.x_outer_a + self.dt_s * self.k_outer_i * eps, 0.0);
            fmath::fmaxf(self.x_outer_a + self.k_outer_p * eps, 0.0)
        } else if dg > hi || self.x_outer_a < 0.0 {
            // Holding the upper edge (light load): bias ≤ 0.
            let eps = hi - dg;
            self.x_outer_a = fmath::fminf(self.x_outer_a + self.dt_s * self.k_outer_i * eps, 0.0);
            fmath::fminf(self.x_outer_a + self.k_outer_p * eps, 0.0)
        } else {
            0.0
        }
    }

    /// Bumpless re-initialisation of the integral so the demand equals `i_now_a` at the
    /// present estimate (`dg`, `dg_dot`), with the band bias preset to `i_bias_a`.
    pub fn bumpless(&mut self, i_now_a: f32, dg: f32, dg_dot: f32, i_bias_a: f32) {
        let g = &self.gains;
        self.x_outer_a = if self.k_outer_i > 0.0 { i_bias_a } else { 0.0 };
        self.i_bias_a = self.x_outer_a;
        self.q_a_s = if g.kappa_per_s != 0.0 {
            (i_now_a - g.kp_a_per_m * dg - g.kd_a_s_per_m * dg_dot) / g.kappa_per_s
        } else {
            0.0
        };
        if !self.q_a_s.is_finite() {
            self.q_a_s = 0.0;
        }
    }

    /// Bumpless re-initialisation of the integral so the demand equals `i_now_a` (at
    /// zero deviation and rate).
    pub fn reset(&mut self, i_now_a: f32) {
        self.bumpless(i_now_a, 0.0, 0.0, 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::levitation::test_plant::{HeavePlant, run_levitated};
    use fabrail_params::{derive, presets};

    #[test]
    fn bumpless_reset_matches_demand() {
        let g = ModeGains {
            kp_a_per_m: 20_000.0,
            kd_a_s_per_m: 300.0,
            kappa_per_s: 150.0,
        };
        let mut z = ZeroPower::new(g, [-1e-3, 3.5e-3], 1e-4);
        z.bumpless(1.5, 2e-4, -0.01, 0.0);
        let u = z.step(2e-4, -0.01, 1.5);
        assert!((u - 1.5).abs() < 1e-5, "bumpless: {u}");
    }

    // req: FAB-LEV-018
    #[test]
    fn bias_current_closed_form() {
        // DRAWER at 48 kg floats at 1.427 mm, below the 1.5 mm band edge (F1); the
        // modified zero-power outer loop holds 1.5 mm and the steady current equals the
        // closed form bias_current(1.5 mm, 48 kg) = +0.39 A (LEV-018). Nonlinear plant
        // with the co-energy force, τ_cl lag and τ_e eddy lag; 12 s at 10 kHz, current
        // averaged over the last second (the f32 loop jitters by ≈ ±0.3 mA).
        let p = presets::drawer();
        let want = derive::bias_current(&p.design, 1.5e-3, 48.0);
        assert!(
            (want - 0.39).abs() < 0.01,
            "closed form ≈ +0.39 A (F1): {want}"
        );
        let mut plant = HeavePlant::new(&p, 48.0, 1.5e-3);
        let (lev, trace) = run_levitated(&p, &mut plant, 12.0);
        let i_ss = f64::from(trace.i_mean_last);
        let g_ss = plant.g_m;
        assert!(
            (i_ss / want - 1.0).abs() < 1e-4,
            "steady bias current {i_ss} A vs closed form {want} A (tol 1e-4 rel; LEV-018 ±2 %)"
        );
        assert!(
            (g_ss - 1.5e-3).abs() < 1e-6,
            "gap held at the band edge 1.5 mm (±1 µm): {g_ss}"
        );
        let x = f64::from(lev.loops[0].zp.x_outer_a);
        assert!(
            (x / want - 1.0).abs() < 1e-4,
            "outer-loop state holds the bias: {x}"
        );
        // In the band the bias bleeds to zero: 26 kg floats at g* with i → 0.
        let mut plant = HeavePlant::new(&p, 26.0, 2.3e-3);
        let (lev, trace) = run_levitated(&p, &mut plant, 6.0);
        assert!(
            trace.i_mean_last.abs() < 1e-3,
            "zero power in band: {}",
            trace.i_mean_last
        );
        assert!(
            (plant.g_m - 2.5e-3).abs() < 2e-6,
            "floats at g* = 2.5 mm: {}",
            plant.g_m
        );
        assert_eq!(lev.loops[0].zp.x_outer_a, 0.0, "no bias inside the band");
    }
}
