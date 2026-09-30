//! Coil electrics: exact-discretised lag mode and the flux-linkage voltage mode with motional EMF (PHYSICS §4). Owner: A6a.
//!
//! Lag mode (sim-only): the current follows its reference through the closed current
//! loop τ_cl (plan §2b item 2: ω_c = 2π·1 kHz → τ_cl = 0.159 ms), clamped at |i| ≤
//! i_pulse (hardware clamp). Voltage mode: dψ/dt = v − R(T)·i with ψ(i, g) from the
//! magnet model, including motional EMF; SIL ≡ firmware is claimed in voltage mode only.
//!
//! As implemented (PHYSICS §4; physics perspective §1.4):
//! * Lag mode: within one plant step the current follows the exact solution of
//!   τ_cl·di/dt = i* − i ([`LagProfile`]); the RK4 stages read it at their stage time, so
//!   the lag is exact for any step. The change is slew-limited by the voltage headroom
//!   |di/dt| ≤ (V_bus − |R·i + e_mot|)/L_inc ([`slew_limit_a_per_s`]) and clamped at the
//!   driver limit (i_pulse, folding back to i_cont when the I²t budget
//!   (i_pulse² − i_cont²)·t_pulse is spent, [`I2tBudget`]; plan §2b item 4).
//! * Voltage mode: the current is an RK4 state,
//!   di/dt = (v − R(T)·i − N·(∂Φ/∂g·ġ + ∂Φ/∂y·ẏ + ∂Φ/∂x·ẋ + ∂Φ/∂T·Ṫ))/(N·∂Φ/∂i + L_σ),
//!   |v| ≤ V_bus ([`voltage_di_dt`]); at the driver limit the bridge chops (di/dt = 0
//!   while the voltage would push |i| further).
//! * Coils off (`CoilCmd::Off`): the current decays through the freewheel path at L/R
//!   (physics perspective §1.13).
//!
//! [`lag_step`] is complete (it is the exact solution of the first-order lag).

use fabrail_types::fmath;

/// Exact zero-order-hold step of the first-order lag di/dt = (i_ref − i)/τ over `h_s`:
/// i⁺ = i_ref + (i − i_ref)·e^(−h/τ) (PHYSICS §4). A non-positive τ returns `i_ref`.
pub fn lag_step(i_a: f64, i_ref_a: f64, tau_s: f64, h_s: f64) -> f64 {
    if tau_s <= 0.0 || tau_s.is_nan() {
        return i_ref_a;
    }
    i_ref_a + (i_a - i_ref_a) * fmath::exp(-h_s / tau_s)
}

/// Copper resistance at temperature: R(T) = R₂₀·(1 + α_R·(T − 20 °C)) (copper
/// α_R = +0.393 %/K; PHYSICS §12), Ω.
pub fn r_at(r20_ohm: f64, tempco_r_per_k: f64, t_c: f64) -> f64 {
    r20_ohm * (1.0 + tempco_r_per_k * (t_c - crate::magnet::T_REF_C))
}

/// Largest current slew the bridge can force, (V_bus − |R·i + e_mot|)/L_inc, A/s
/// (≥ 0; physics perspective §1.4). ∞ when L_inc ≤ 0.
pub fn slew_limit_a_per_s(v_bus_v: f64, r_ohm: f64, i_a: f64, e_mot_v: f64, l_inc_h: f64) -> f64 {
    if l_inc_h <= 0.0 || l_inc_h.is_nan() {
        return f64::INFINITY;
    }
    fmath::fmax(v_bus_v - fmath::abs(r_ohm * i_a + e_mot_v), 0.0) / l_inc_h
}

/// Voltage-mode current rate (v − R·i − e_mot)/L_inc, with the driver clamp: at
/// |i| ≥ `limit_a` the rate may only reduce |i| (PHYSICS §4), A/s.
pub fn voltage_di_dt(
    v_v: f64,
    r_ohm: f64,
    i_a: f64,
    e_mot_v: f64,
    l_inc_h: f64,
    limit_a: f64,
) -> f64 {
    if l_inc_h <= 0.0 || l_inc_h.is_nan() {
        return 0.0;
    }
    let di = (v_v - r_ohm * i_a - e_mot_v) / l_inc_h;
    if fmath::abs(i_a) >= limit_a && di * i_a > 0.0 {
        0.0
    } else {
        di
    }
}

/// One plant step of an exactly discretised current lag (PHYSICS §4): the unconstrained
/// lag i_lag(s) = i* + (i₀ − i*)·e^(−s/τ), its change from i₀ limited to ±slew·s, then
/// clamped to ±limit. `s` is the time since the step start.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LagProfile {
    /// Current at the step start, A.
    pub i0_a: f64,
    /// Target (reference, or 0 while decaying), A.
    pub i_ref_a: f64,
    /// Lag time constant (≤ 0 = instantaneous), s.
    pub tau_s: f64,
    /// Slew limit (∞ = none), A/s.
    pub slew_a_per_s: f64,
    /// Driver clamp (∞ = none), A.
    pub limit_a: f64,
}

impl LagProfile {
    /// A constant current.
    pub fn hold(i_a: f64) -> LagProfile {
        LagProfile {
            i0_a: i_a,
            i_ref_a: i_a,
            tau_s: 0.0,
            slew_a_per_s: f64::INFINITY,
            limit_a: f64::INFINITY,
        }
    }

    /// Unconstrained lag value and rate at `s`.
    fn lag(&self, s: f64) -> (f64, f64) {
        if self.tau_s > 0.0 {
            let k = fmath::exp(-s / self.tau_s);
            let d = self.i0_a - self.i_ref_a;
            (self.i_ref_a + d * k, -d * k / self.tau_s)
        } else {
            (self.i_ref_a, 0.0)
        }
    }

    /// Current at `s` into the step, A.
    pub fn at(&self, s: f64) -> f64 {
        let (il, _) = self.lag(s);
        let m = self.slew_a_per_s * s;
        let d = if m.is_finite() {
            fmath::clamp(il - self.i0_a, -m, m)
        } else {
            il - self.i0_a
        };
        fmath::clamp(self.i0_a + d, -self.limit_a, self.limit_a)
    }

    /// di/dt at `s` into the step (the slope of [`LagProfile::at`]), A/s.
    pub fn rate(&self, s: f64) -> f64 {
        let (il, r) = self.lag(s);
        let m = self.slew_a_per_s * s;
        let d = il - self.i0_a;
        let (i, di) = if m.is_finite() && fmath::abs(d) > m {
            (
                self.i0_a + fmath::copysign(m, d),
                fmath::copysign(self.slew_a_per_s, d),
            )
        } else {
            (il, r)
        };
        if fmath::abs(i) >= self.limit_a {
            0.0
        } else {
            di
        }
    }
}

/// Driver I²t protection (plan §2b item 4): the budget (i_pulse² − i_cont²)·t_pulse is
/// spent by ∫(i² − i_cont²)dt and recovers below i_cont; when it is spent the clamp
/// folds back from i_pulse to i_cont until the budget is fully recovered.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct I2tBudget {
    /// Spent budget ∫(i² − i_cont²)dt ≥ 0, A²·s.
    pub spent_a2s: f64,
    /// Folded back to i_cont.
    pub folded: bool,
}

impl I2tBudget {
    /// Present driver clamp, A (∞ when no pulse limit is configured).
    pub fn limit_a(&self, i_cont_a: f64, i_pulse_a: f64) -> f64 {
        if i_pulse_a <= 0.0 {
            f64::INFINITY
        } else if self.folded {
            fmath::fmin(i_cont_a, i_pulse_a)
        } else {
            i_pulse_a
        }
    }

    /// Accounts one step of `h_s` at current `i_a`.
    pub fn update(&mut self, i_a: f64, h_s: f64, i_cont_a: f64, i_pulse_a: f64, t_pulse_s: f64) {
        if i_pulse_a <= 0.0 {
            return;
        }
        let budget = fmath::fmax(i_pulse_a * i_pulse_a - i_cont_a * i_cont_a, 0.0) * t_pulse_s;
        self.spent_a2s = fmath::fmax(
            self.spent_a2s + (i_a * i_a - i_cont_a * i_cont_a) * h_s,
            0.0,
        );
        if self.spent_a2s >= budget {
            self.folded = true;
        } else if self.spent_a2s <= 0.0 {
            self.folded = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lag_step_is_exact() {
        // One time constant closes 1 − e⁻¹ of the error.
        let i = lag_step(0.0, 1.0, 1.0e-4, 1.0e-4);
        assert!((i - (1.0 - (-1.0f64).exp())).abs() < 1e-15);
        // Two half-steps equal one full step (exact discretisation composes).
        let a = lag_step(lag_step(0.2, 3.0, 2e-4, 5e-5), 3.0, 2e-4, 5e-5);
        let b = lag_step(0.2, 3.0, 2e-4, 1e-4);
        assert!((a - b).abs() < 1e-14);
        assert_eq!(lag_step(0.2, 3.0, 0.0, 1e-4), 3.0);
    }

    #[test]
    fn profile_matches_lag_step_and_its_rate() {
        // Without slew or clamp the profile is lag_step (PHYSICS §4), and rate() is its
        // derivative (FD, 1e-6 rel).
        let tau = 1.0 / (2.0 * core::f64::consts::PI * 1000.0); // τ_cl 0.159 ms, plan §2b item 2
        let p = LagProfile {
            i0_a: 0.5,
            i_ref_a: 2.0,
            tau_s: tau,
            slew_a_per_s: f64::INFINITY,
            limit_a: f64::INFINITY,
        };
        for s in [0.0, 1e-5, 2.5e-5, 5e-5] {
            assert!((p.at(s) - lag_step(0.5, 2.0, tau, s)).abs() < 1e-15);
            let fd =
                (p.at(s + 1e-9) - p.at((s - 1e-9).max(0.0))) / if s > 0.0 { 2e-9 } else { 1e-9 };
            assert!(
                (p.rate(s) - fd).abs() <= 1e-5 * fd.abs(),
                "{} vs {fd}",
                p.rate(s)
            );
        }
    }

    #[test]
    fn slew_and_clamp_bound_the_profile() {
        // WALL at 48 V: L ≈ 13 mH gives ≈ 3.7 kA/s (physics perspective §1.4); a 10 A step
        // over 50 µs moves at most slew·h, and the clamp holds ±i_pulse.
        let r = slew_limit_a_per_s(48.0, 0.7, 0.0, 0.0, 13.0e-3);
        assert!((r - 48.0 / 13.0e-3).abs() < 1e-9);
        let p = LagProfile {
            i0_a: 0.0,
            i_ref_a: 10.0,
            tau_s: 1.59e-4,
            slew_a_per_s: r,
            limit_a: 20.0,
        };
        assert!(
            (p.at(5e-5) - r * 5e-5).abs() < 1e-12,
            "slew-limited (PHYSICS §4)"
        );
        assert!((p.rate(2e-5) - r).abs() < 1e-9);
        let c = LagProfile {
            i0_a: 19.9,
            i_ref_a: 40.0,
            tau_s: 1e-4,
            slew_a_per_s: f64::INFINITY,
            limit_a: 20.0,
        };
        assert_eq!(c.at(1e-3), 20.0, "hardware clamp (PHYSICS §4)");
        assert_eq!(c.rate(1e-3), 0.0);
        // Headroom vanishes when R·i + e_mot reaches the bus.
        assert_eq!(slew_limit_a_per_s(24.0, 1.6, 10.0, 10.0, 2e-3), 0.0);
    }

    #[test]
    fn voltage_mode_rate_and_chopping() {
        assert!((voltage_di_dt(10.0, 1.0, 2.0, 1.0, 0.01, 20.0) - 700.0).abs() < 1e-9);
        assert_eq!(
            voltage_di_dt(30.0, 1.0, 20.0, 0.0, 0.01, 20.0),
            0.0,
            "clamped"
        );
        assert!(
            voltage_di_dt(-10.0, 1.0, 20.0, 0.0, 0.01, 20.0) < 0.0,
            "may reduce |i|"
        );
    }

    #[test]
    fn i2t_budget_folds_back_and_recovers() {
        // DRAWER: i_cont 4 A, i_pulse 12 A for 200 ms (plan §2b item 4).
        let mut b = I2tBudget::default();
        let h = 5e-5;
        let mut t = 0.0;
        while !b.folded {
            b.update(12.0, h, 4.0, 12.0, 0.2);
            t += h;
            assert!(t < 0.3, "must fold back");
        }
        assert!((t - 0.2).abs() <= 2.0 * h, "folds at t_pulse (got {t})");
        assert_eq!(b.limit_a(4.0, 12.0), 4.0);
        for _ in 0..200_000 {
            b.update(0.0, h, 4.0, 12.0, 0.2);
            if !b.folded {
                break;
            }
        }
        assert!(
            !b.folded && b.limit_a(4.0, 12.0) == 12.0,
            "recovers below i_cont"
        );
        assert!((r_at(1.6, 0.00393, 60.0) - 1.6 * (1.0 + 0.00393 * 40.0)).abs() < 1e-15);
    }
}
