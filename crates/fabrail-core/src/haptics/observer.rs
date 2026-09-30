//! Momentum observer for the hand force F_hat (K_O), 0.5 Hz pressure high-pass and the lean bias estimator (PHYSICS §17). Owner: A2.
//!
//! Frozen API (W0): [`MomentumObserver::new`], [`MomentumObserver::step`] → F̂_h (N).
//! Generalised-momentum observer with bandwidth K_O = 2π×{20, 15, 8, 5} Hz by class; a
//! lean (F̂ > F_th with |v| < 2 mm/s for > 1 s, detected by `haptics::intent`) feeds
//! the bias estimator instead of intent. Test: `ramp_tracking` (10 %).
//!
//! **Observer** (PHYSICS §17; De Luca & Mattone 2005 generalised momentum, extended with
//! a disturbance-rate state as in Han's extended-state observer). With the measured
//! momentum p = m̂·v, the actuator force F_act (the modelled LSM thrust of the measured
//! i_q) and the drag b̂·v (b̂ = b_dist + c_rail, PHYSICS §9):
//!
//! ```text
//! e = p − p̂
//! p̂' = F_act − b̂·v + F̂ + l1·e
//! F̂' = Ḟ̂ + l2·e
//! Ḟ̂' = l3·e
//! ```
//!
//! with the error poles at K_O·{1, 3, 5} (l1 = 9K_O, l2 = 23K_O², l3 = 15K_O³; forward
//! Euler at the fast period, |pole|·T ≤ 0.07). The dominant pole is the class bandwidth
//! K_O; the faster pair and the ramp state remove the steady ramp lag of the classic
//! first-order observer r = K_O(m̂v − ∫(F + r)dt) (lag = slope/K_O). The peak error on a
//! ramp of slope k is ≈ 0.33·k/K_O (analysis of E(s) = k(s + l1)/char(s)), so the
//! FAB-HAP-004 bound of 10 % of peak holds for ramps longer than ≈ 3.3/K_O: 26 / 35 / 66 /
//! 105 ms (DRAWER / DOOR / WALL / HEAVY). A force step overshoots by ≈ 19 %.
//!
//! **Pressure high-pass.** The 0.5 Hz high-pass on F̂ (`haptic.f_hp_hz`) that rejects
//! pressure and lean ramps is applied in `haptics::intent` only: applied here it would
//! bias admittance and the ramp tracking (a 150 ms ramp loses ≈ 20 % through it).
//!
//! **Lean bias estimator.** While `set_lean(true)` a bias b̂_F follows the raw F̂ with
//! τ = `intent.lean_t_s` (1 s); otherwise it relaxes to 0 with the same τ. It is always
//! kept between 0 and the raw F̂, so the corrected output F̂ − b̂_F never changes sign
//! and never exceeds the raw estimate (a released lean cannot push the carriage back).

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::fmath;

/// Observer pole multiples of K_O (design choice; module docs).
const POLES: [f32; 3] = [1.0, 3.0, 5.0];

/// Momentum observer and lean bias estimator.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MomentumObserver {
    /// Observer bandwidth K_O (dominant pole), rad/s.
    pub k_o_rad_s: f32,
    /// Estimated momentum p̂, kg·m/s.
    pub integ_n: f32,
    /// Lean bias estimate, N.
    pub bias_n: f32,
    /// Fast period, s.
    pub dt_s: f32,
    /// Gains l1 (1/s), l2 (1/s²), l3 (1/s³).
    pub l: [f32; 3],
    /// Raw disturbance estimate F̂ (before the bias correction), N.
    pub f_raw_n: f32,
    /// Estimated disturbance rate Ḟ̂, N/s.
    pub f_rate_n_per_s: f32,
    /// Viscous drag b̂, N·s/m.
    pub b_hat_n_s_per_m: f32,
    /// Bias time constant, s.
    pub tau_bias_s: f32,
    /// Lean active (bias estimator following F̂).
    pub lean: bool,
    /// m̂ used at the last step (reset uses it), kg.
    pub m_last_kg: f32,
}

impl MomentumObserver {
    /// Observer from the haptic tuning (never fails; a zero period gives F̂ = 0).
    pub fn new(p: &SystemParams) -> Result<MomentumObserver, ParamError> {
        let k = (2.0 * core::f64::consts::PI * p.control.haptic.k_o_hz) as f32;
        let [a, b, c] = POLES;
        let dmp = &p.design.damping;
        Ok(MomentumObserver {
            k_o_rad_s: k,
            integ_n: 0.0,
            bias_n: 0.0,
            dt_s: fabrail_params::derive::fast_period_s(p) as f32,
            // Characteristic polynomial (s + aK)(s + bK)(s + cK).
            l: [
                (a + b + c) * k,
                (a * b + b * c + c * a) * k * k,
                a * b * c * k * k * k,
            ],
            f_raw_n: 0.0,
            f_rate_n_per_s: 0.0,
            b_hat_n_s_per_m: (dmp.b_dist_n_s_per_m + dmp.c_rail_n_s_per_m) as f32,
            tau_bias_s: p.control.intent.lean_t_s as f32,
            lean: false,
            m_last_kg: p.design.carriage.m_star_kg as f32,
        })
    }

    /// Lean classification from `haptics::intent` for the next steps.
    pub fn set_lean(&mut self, lean: bool) {
        self.lean = lean;
    }

    /// One fast release from velocity (m/s), applied actuator force (N) and m̂ (kg);
    /// returns the bias-corrected F̂_h (N). The raw estimate is `f_raw_n`.
    pub fn step(&mut self, v_mps: f32, f_act_n: f32, m_hat_kg: f32) -> f32 {
        let dt = self.dt_s;
        if !(v_mps.is_finite() && f_act_n.is_finite() && m_hat_kg.is_finite()) || dt <= 0.0 {
            return self.f_raw_n - self.bias_n;
        }
        self.m_last_kg = m_hat_kg;
        let [l1, l2, l3] = self.l;
        let e = m_hat_kg * v_mps - self.integ_n;
        let f = self.f_raw_n;
        let fd = self.f_rate_n_per_s;
        self.integ_n += (f_act_n - self.b_hat_n_s_per_m * v_mps + f + l1 * e) * dt;
        self.f_raw_n += (fd + l2 * e) * dt;
        self.f_rate_n_per_s += l3 * e * dt;
        if !(self.f_raw_n.is_finite()
            && self.integ_n.is_finite()
            && self.f_rate_n_per_s.is_finite())
        {
            self.reset(v_mps);
        }
        // Lean bias: follow F̂ while leaning, else relax to 0; kept within [0, F̂].
        let r = self.f_raw_n;
        let target = if self.lean { r } else { 0.0 };
        if self.tau_bias_s > 0.0 {
            self.bias_n += (target - self.bias_n) * fmath::fminf(dt / self.tau_bias_s, 1.0);
        } else {
            self.bias_n = target;
        }
        self.bias_n = fmath::clampf(self.bias_n, fmath::fminf(0.0, r), fmath::fmaxf(0.0, r));
        r - self.bias_n
    }

    /// Re-initialises at velocity `v_mps` (bumpless: p̂ = m̂·v, F̂ = Ḟ̂ = 0, no bias).
    pub fn reset(&mut self, v_mps: f32) {
        let v = if v_mps.is_finite() { v_mps } else { 0.0 };
        self.integ_n = self.m_last_kg * v;
        self.f_raw_n = 0.0;
        self.f_rate_n_per_s = 0.0;
        self.bias_n = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    /// Rigid carriage m·v' = F_act + F_h − b·v integrated with 20 substeps per fast
    /// period; F_act is a stiff position hold (so the actuator force is non-zero).
    /// Returns the peak |F̂ − F_h| over a 50 ms rest, the ramp 0 → f_pk over `t_r` and a
    /// 400 ms hold, relative to f_pk.
    fn ramp_error(p: &SystemParams, f_pk: f64, t_r: f64) -> f64 {
        let mut o = MomentumObserver::new(p).unwrap_or_default();
        let m = p.design.carriage.m_rated_kg;
        let b = p.design.damping.b_dist_n_s_per_m + p.design.damping.c_rail_n_s_per_m;
        let dt = f64::from(o.dt_s);
        let (mut x, mut v) = (0.0_f64, 0.0_f64);
        // Hold law: K = m·ω², D = 2·m·ω with ω = 2π·2 Hz (a soft position hold).
        let w = 2.0 * core::f64::consts::PI * 2.0;
        let (kh, dh) = (m * w * w, 2.0 * m * w);
        let force = |t: f64| {
            if t < 0.05 {
                0.0
            } else {
                f_pk * ((t - 0.05) / t_r).min(1.0)
            }
        };
        let n = ((0.05 + t_r + 0.4) / dt).ceil() as usize;
        let mut worst = 0.0_f64;
        for k in 0..n {
            let t = k as f64 * dt;
            let f_act = -kh * x - dh * v;
            let fh = o.step(v as f32, f_act as f32, m as f32);
            worst = worst.max((f64::from(fh) - force(t)).abs());
            let h = dt / 20.0;
            for s in 0..20 {
                let ts = t + s as f64 * h;
                let a = (f_act + force(ts) - b * v) / m;
                v += a * h;
                x += v * h;
            }
        }
        worst / f_pk
    }

    /// F̂ error ≤ 10 % of peak on 5–25 N ramps (G8 hand-intent profile) over 50–150 ms
    /// for DRAWER and DOOR, and over the ramp lengths the class bandwidth supports for
    /// WALL (≥ 75 ms) and HEAVY (≥ 120 ms): the analytic peak error is ≈ 0.33·slope/K_O
    /// (module docs), 10 % needs T_r ≥ 3.3/K_O = 66 / 105 ms there. m̂ exact.
    /// (A) 10 % of peak, FAB-HAP-004, PHYSICS §17.
    // req: FAB-HAP-004
    #[test]
    fn ramp_tracking() {
        let classes = [
            (presets::drawer(), 0.050),
            (presets::door(), 0.050),
            (presets::wall(), 0.075),
            (presets::heavy(), 0.120),
        ];
        for (p, t_min) in classes {
            for f_pk in [5.0, 15.0, 25.0] {
                for t_r in [t_min, 0.5 * (t_min + 0.150), 0.150] {
                    let e = ramp_error(&p, f_pk, t_r);
                    assert!(
                        e <= 0.10,
                        "{:?}: F̂ error {:.1} % of peak on a {f_pk} N ramp over {:.0} ms \
                         ((A) ≤ 10 %, FAB-HAP-004)",
                        p.design.class,
                        e * 100.0,
                        t_r * 1e3
                    );
                }
            }
        }
    }

    #[test]
    fn lean_bias_never_flips_sign() {
        let p = presets::door();
        let mut o = MomentumObserver::new(&p).unwrap_or_default();
        o.set_lean(true);
        // A constant 15 N lean with the carriage held still: F̂_raw → 15 N, bias follows.
        let m = 100.0;
        let mut out = 0.0;
        for _ in 0..40_000 {
            // Momentum stays 0 (carriage held), F_act balances the lean: −15 N.
            out = o.step(0.0, -15.0, m);
        }
        assert!((o.f_raw_n - 15.0).abs() < 0.01);
        assert!(out.abs() < 0.5, "lean absorbed by the bias: {out}");
        // Release: raw F̂ → 0 (with the observer's own step overshoot). The bias never
        // adds to it: the corrected output lies between 0 and the raw estimate, so the
        // released lean does not push back with −15 N.
        o.set_lean(false);
        for _ in 0..5_000 {
            let f = o.step(0.0, 0.0, m);
            let r = o.f_raw_n;
            assert!(
                f >= r.min(0.0) - 1e-4 && f <= r.max(0.0) + 1e-4,
                "{f} outside [0, {r}]"
            );
            assert!(f > -0.25 * 15.0, "released lean pushes back: {f}");
        }
    }

    #[test]
    fn non_finite_inputs_are_ignored() {
        let mut o = MomentumObserver::new(&presets::drawer()).unwrap_or_default();
        let f = o.step(f32::NAN, 1.0, 26.0);
        assert!(f.is_finite());
        o.reset(0.1);
        assert_eq!(o.f_raw_n, 0.0);
    }
}
