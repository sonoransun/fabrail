//! Pd, tau_c and tau_p_max by double integration with a 0.075 Hz high-pass (PHYSICS §20; SEI-004). Owner: A3.
//!
//! Frozen API (W0): [`PdTauC::new`], [`PdTauC::start`] (at the P pick),
//! [`PdTauC::step`] → [`PdTc`]. Vertical acceleration is integrated twice with a
//! 0.075 Hz high-pass; Pd is the peak |u| and τc = 2π/√(∫u̇²/∫u²) over the 1 s and 3 s
//! windows; τp_max with α = 0.99 \[M\]. Wu–Zhao anchor Pd 0.7118 cm (core design §E.5).
//! Tests: `tauc_sinusoid` (1 %; fed u and u̇ directly over an integer number of
//! periods, plan §2b item 7), `pd_linear_in_amplitude`.
//!
//! **Chain, exactly as implemented** (Wu & Kanamori 2005; Wu & Zhao 2006; PHYSICS §20).
//! Every high-pass is a 2-pole Butterworth at `pdtc_hp_hz` (0.075 Hz), Tustin with the
//! corner prewarped (ω_a = 2f_s·tan(π·f_c/f_s)), stepped as a TDF-II `filters::Biquad`:
//!
//! ```text
//! a_b = HP(a_z − a_z,0)    baseline: removes gravity (+1 g on z) and sensor offset
//! v   = IHP(a_b)           = HP(∫a_b dt), trapezoidal ∫   (velocity stage)
//! u   = IHP(v)             = HP(∫v dt)                     (displacement stage)
//! u̇   = HP(v)              = d/dt u exactly (the same HP without the integrator)
//! ```
//!
//! IHP(s) = s/(s² + √2·ω_a·s + ω_a²) is the trapezoidal integrator and the HP fused into
//! one biquad (the Tustin map of a product is the product of the maps, so this is *exactly*
//! "integrate, then high-pass" without an unbounded integrator state in f32). The first
//! sample a_z,0 is subtracted before the baseline HP (exact in f32 for samples within a
//! factor 2 of it, Sterbenz): the 0.075 Hz section then carries mm/s²-scale states instead
//! of 9.8 m/s², which keeps its f32 rounding noise out of Pd (with 1 g on z the f32 chain
//! otherwise drifts ≈ 2 % from its gravity-free result). All stages start from rest.
//!
//! **Windows.** [`PdTauC::start`] opens the window at the P pick: Pd = max|u|,
//! τc = 2π·√(Σu²/Σu̇²) (the sample period cancels) accumulate for `win_long_s` (3 s) and
//! then freeze; [`PdTauC::short_done`] marks the `win_short_s` (1 s) early estimate that
//! the level matrix first uses. τp (Allen & Kanamori 2003): X ← αX + u̇²,
//! D ← αD + (Δu̇/Δt)², τp = 2π√(X/D), run continuously; τp_max is its maximum inside the
//! window. With α = 0.99 \[M\] at 100 Hz the recursion has a 1 s memory, so τp_max of a
//! sinusoid carries a ripple of about (1 − α)/|1 − α·e^(−2iωΔt)| (4 % at 0.5 s period);
//! τc has no such ripple over whole periods.

use fabrail_params::DetectorTuning;
use fabrail_types::fmath;

use crate::filters::Biquad;

/// 2π.
const TWO_PI: f32 = core::f32::consts::TAU;

/// Pd/τc estimate.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PdTc {
    /// Peak displacement Pd, m.
    pub pd_m: f32,
    /// Average period τc, s.
    pub tau_c_s: f32,
    /// Predominant period τp_max, s.
    pub tau_p_max_s: f32,
    /// Time since the P pick, s.
    pub t_since_p_s: f32,
    /// The long (3 s) window has completed.
    pub complete: bool,
}

/// Pd/τc estimator.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PdTauC {
    /// Sample period, s.
    pub dt_s: f32,
    /// Running output.
    pub out: PdTc,
    /// Window running.
    pub active: bool,
    /// Baseline high-pass on the vertical acceleration.
    pub hp_a: Biquad,
    /// Velocity stage (integrate, then high-pass).
    pub ihp_v: Biquad,
    /// Displacement stage (integrate, then high-pass).
    pub ihp_u: Biquad,
    /// u̇ = HP(v), the exact derivative of u.
    pub hp_ud: Biquad,
    /// The baseline a_z,0 has been taken.
    pub primed: bool,
    /// Baseline (first sample), m/s².
    pub a0_mps2: f32,
    /// Samples accumulated in the window.
    pub n: u32,
    /// Short-window length, samples.
    pub n_short: u32,
    /// Long-window length, samples.
    pub n_long: u32,
    /// Σu² over the window, m².
    pub sum_u2: f32,
    /// Σu̇² over the window, (m/s)².
    pub sum_ud2: f32,
    /// τp smoothing α.
    pub alpha: f32,
    /// τp numerator X.
    pub tp_x: f32,
    /// τp denominator D.
    pub tp_d: f32,
    /// Previous u̇ (τp derivative).
    pub ud_prev: f32,
}

impl PdTauC {
    /// Estimator at sample rate `fs_hz` from the detector tuning (`pdtc_hp_hz`,
    /// `win_short_s`, `win_long_s`, `taup_alpha`). A non-positive rate gives an inert
    /// estimator that never accumulates.
    pub fn new(t: &DetectorTuning, fs_hz: f32) -> PdTauC {
        let fs = f64::from(fs_hz);
        if !(fs.is_finite() && fs > 0.0) {
            return PdTauC::default();
        }
        let hp = highpass2(t.pdtc_hp_hz, fs);
        let ihp = integrating_highpass2(t.pdtc_hp_hz, fs);
        let n_of = |s: f64| {
            let n = fmath::round(s * fs);
            if n.is_finite() && n >= 1.0 {
                n as u32
            } else {
                1
            }
        };
        let n_short = n_of(t.win_short_s);
        PdTauC {
            dt_s: (1.0 / fs) as f32,
            hp_a: hp,
            ihp_v: ihp,
            ihp_u: ihp,
            hp_ud: hp,
            n_short,
            n_long: n_of(t.win_long_s).max(n_short),
            alpha: fmath::clamp(t.taup_alpha, 0.0, 1.0) as f32,
            ..PdTauC::default()
        }
    }

    /// Starts the windows at the P pick (the filters keep running).
    pub fn start(&mut self) {
        self.active = true;
        self.n = 0;
        self.sum_u2 = 0.0;
        self.sum_ud2 = 0.0;
        self.out = PdTc::default();
    }

    /// One vertical acceleration sample, m/s² (gravity included).
    pub fn step(&mut self, a_z_mps2: f32) -> PdTc {
        if !(self.dt_s > 0.0 && a_z_mps2.is_finite()) {
            return self.out;
        }
        if !self.primed {
            self.a0_mps2 = a_z_mps2;
            self.primed = true;
        }
        let a = self.hp_a.step(a_z_mps2 - self.a0_mps2);
        let v = self.ihp_v.step(a);
        let u = self.ihp_u.step(v);
        let ud = self.hp_ud.step(v);
        self.feed(u, ud)
    }

    /// Feeds displacement `u_m` and its derivative `udot_mps` directly (bypassing the
    /// integration chain): the window stage of [`PdTauC::step`], used by the τc test and
    /// by replay tools that already hold u and u̇.
    pub fn push_disp(&mut self, u_m: f32, udot_mps: f32) -> PdTc {
        if !(self.dt_s > 0.0 && u_m.is_finite() && udot_mps.is_finite()) {
            return self.out;
        }
        self.feed(u_m, udot_mps)
    }

    fn feed(&mut self, u: f32, ud: f32) -> PdTc {
        let d = (ud - self.ud_prev) / self.dt_s;
        self.ud_prev = ud;
        self.tp_x = self.alpha * self.tp_x + ud * ud;
        self.tp_d = self.alpha * self.tp_d + d * d;
        if !self.active || self.n >= self.n_long {
            return self.out;
        }
        self.n += 1;
        self.sum_u2 += u * u;
        self.sum_ud2 += ud * ud;
        let o = &mut self.out;
        o.pd_m = fmath::fmaxf(o.pd_m, fmath::absf(u));
        o.tau_c_s = if self.sum_u2 > 0.0 && self.sum_ud2 > 0.0 {
            TWO_PI * fmath::sqrtf(self.sum_u2 / self.sum_ud2)
        } else {
            0.0
        };
        if self.tp_d > 0.0 {
            let tp = TWO_PI * fmath::sqrtf(self.tp_x / self.tp_d);
            o.tau_p_max_s = fmath::fmaxf(o.tau_p_max_s, tp);
        }
        o.t_since_p_s = self.n as f32 * self.dt_s;
        o.complete = self.n >= self.n_long;
        *o
    }

    /// The short (1 s) window has been accumulated since the pick.
    #[inline]
    pub fn short_done(&self) -> bool {
        self.active && self.n >= self.n_short
    }

    /// Stops and clears the window outputs (the filters keep their state).
    pub fn reset(&mut self) {
        self.active = false;
        self.n = 0;
        self.sum_u2 = 0.0;
        self.sum_ud2 = 0.0;
        self.out = PdTc::default();
    }
}

/// Tustin coefficients of an analog biquad (n2·s² + n1·s + n0)/(s² + √2·ω_a·s + ω_a²),
/// ω_a prewarped at `fc_hz` (2-pole Butterworth denominator; PHYSICS §20). Init-time f64.
fn tustin_butter2(n: [f64; 3], fc_hz: f64, fs_hz: f64) -> Biquad {
    if !(fc_hz.is_finite() && fc_hz > 0.0 && fs_hz.is_finite() && fc_hz < 0.5 * fs_hz) {
        return Biquad::IDENTITY;
    }
    let k = 2.0 * fs_hz;
    let w = k * fmath::tan(core::f64::consts::PI * fc_hz / fs_hz);
    let sq2 = core::f64::consts::SQRT_2;
    let a0 = k * k + sq2 * w * k + w * w;
    let a1 = (2.0 * w * w - 2.0 * k * k) / a0;
    let a2 = (k * k - sq2 * w * k + w * w) / a0;
    let [n2, n1, n0] = n;
    // s → k(1 − z⁻¹)/(1 + z⁻¹), numerator multiplied by (1 + z⁻¹)².
    let b0 = (n2 * k * k + n1 * k + n0) / a0;
    let b1 = (2.0 * n0 - 2.0 * n2 * k * k) / a0;
    let b2 = (n2 * k * k - n1 * k + n0) / a0;
    Biquad::new([b0 as f32, b1 as f32, b2 as f32], [a1 as f32, a2 as f32])
}

/// 2-pole Butterworth high-pass s²/(s² + √2ω_a s + ω_a²), Tustin, prewarped (PHYSICS §20).
pub(crate) fn highpass2(fc_hz: f64, fs_hz: f64) -> Biquad {
    tustin_butter2([1.0, 0.0, 0.0], fc_hz, fs_hz)
}

/// Integrating high-pass s/(s² + √2ω_a s + ω_a²) = HP·(1/s), Tustin, prewarped: exactly a
/// trapezoidal integrator followed by [`highpass2`] (PHYSICS §20).
pub(crate) fn integrating_highpass2(fc_hz: f64, fs_hz: f64) -> Biquad {
    tustin_butter2([0.0, 1.0, 0.0], fc_hz, fs_hz)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    // req: FAB-SEI-004
    #[test]
    fn tauc_sinusoid() {
        let t = presets::wall().control.seismic.detector;
        let fs = 100.0_f32;
        let a = 3.0e-3_f32; // 3 mm displacement amplitude
        // Periods that fit a whole number of times into the 3 s window (and, for 0.5 s
        // and 1 s, into the 1 s window).
        for period in [0.5_f32, 0.75, 1.0, 1.5, 3.0] {
            let mut e = PdTauC::new(&t, fs);
            assert_eq!(
                (e.n_short, e.n_long),
                (100, 300),
                "1 s / 3 s windows at 100 Hz"
            );
            e.start();
            let w = TWO_PI / period;
            let mut out = PdTc::default();
            for k in 0..e.n_long {
                let tk = k as f32 / fs;
                out = e.push_disp(a * fmath::sinf(w * tk), a * w * fmath::cosf(w * tk));
                if k + 1 == e.n_short && (period == 0.5 || period == 1.0) {
                    let rel = (out.tau_c_s - period).abs() / period;
                    assert!(
                        rel <= 0.01,
                        "1 s window: τc {} vs T {period} (±1 %; SEI-004)",
                        out.tau_c_s
                    );
                }
            }
            assert!(out.complete, "3 s window complete");
            let rel = (out.tau_c_s - period).abs() / period;
            assert!(
                rel <= 0.01,
                "τc {} s vs period {period} s: rel {rel:.2e} (±1 %; Wu & Kanamori 2005; SEI-004)",
                out.tau_c_s
            );
            assert!(
                (out.pd_m - a).abs() <= 0.01 * a,
                "Pd = amplitude ±1 % (sampled peak)"
            );
            // The window is frozen after 3 s.
            let frozen = e.push_disp(1.0, 1.0);
            assert_eq!(frozen, out, "no accumulation after the long window");
        }
    }

    // req: FAB-SEI-004
    #[test]
    fn pd_linear_in_amplitude() {
        let t = presets::door().control.seismic.detector;
        let fs = 100.0_f32;
        // A tapered 1.5 Hz burst (zero mean, inside the detector band).
        let burst = |k: u32| {
            let tk = k as f32 / fs;
            let env = fmath::sinf(core::f32::consts::PI * fmath::fminf(tk / 4.0, 1.0));
            env * fmath::sinf(TWO_PI * 1.5 * tk)
        };
        let run = |amp: f32| {
            let mut e = PdTauC::new(&t, fs);
            for _ in 0..200 {
                e.step(0.0);
            }
            e.start();
            let mut out = PdTc::default();
            for k in 0..400 {
                out = e.step(amp * burst(k));
            }
            out
        };
        let base_amp = 1.0e-2_f32;
        let base = run(base_amp);
        assert!(base.pd_m > 0.0 && base.complete);
        // Power-of-two amplitude ratios: the chain has no amplitude-dependent logic, so
        // binary floating point scales it exactly (tolerance 0).
        for k in [-3_i32, -1, 1, 3, 6] {
            let g = fmath::powif(2.0, k);
            let o = run(base_amp * g);
            assert_eq!(
                o.pd_m,
                base.pd_m * g,
                "Pd(2^{k}·A) = 2^{k}·Pd(A) exactly (SEI-004)"
            );
            assert_eq!(o.tau_c_s, base.tau_c_s, "τc amplitude-invariant exactly");
        }
        // Other ratios: equal up to the f32 rounding noise of the 0.075 Hz chain
        // (f32 vs f64 Pd within 0.5 %, physics perspective §4.6).
        for amp in [1.0e-3_f32, 3.0e-2, 0.1, 1.0] {
            let o = run(amp);
            let ratio = o.pd_m / base.pd_m;
            let want = amp / base_amp;
            assert!(
                (ratio - want).abs() <= 5e-3 * want,
                "Pd({amp})/Pd(0.01) = {ratio} vs {want} (linear within 0.5 % f32; SEI-004)"
            );
            assert!(
                (o.tau_c_s - base.tau_c_s).abs() <= 5e-3 * base.tau_c_s,
                "τc amplitude-invariant within 0.5 %"
            );
        }
    }

    #[test]
    fn taup_of_sinusoid_within_recursive_ripple() {
        let t = presets::door().control.seismic.detector;
        let fs = 100.0_f32;
        let period = 0.5_f32;
        let w = TWO_PI / period;
        let mut e = PdTauC::new(&t, fs);
        let mut k = 0_u32;
        let mut feed = |e: &mut PdTauC, n: u32| {
            for _ in 0..n {
                let tk = k as f32 / fs;
                e.push_disp(fmath::sinf(w * tk) / w, fmath::cosf(w * tk));
                k += 1;
            }
        };
        feed(&mut e, 2000); // X, D in steady state (20 s ≫ 1 s memory)
        e.start();
        feed(&mut e, 300);
        // Ripple bound (1 − α)/|1 − α e^(−2iωΔt)| plus the backward-difference gain.
        let (al, x) = (0.99_f64, 2.0 * f64::from(w) / 100.0);
        let r = (1.0 - al) / ((1.0 - al * x.cos()).powi(2) + (al * x.sin()).powi(2)).sqrt();
        let tp = f64::from(e.out.tau_p_max_s);
        let p = f64::from(period);
        assert!(
            tp >= p * (1.0 - 0.01) && tp <= p * (1.0 + r + 0.01),
            "τp_max {tp:.4} s within [T, T(1 + {r:.3})] ±1 % (α = 0.99 [M]; Allen & Kanamori 2003)"
        );
    }

    #[test]
    fn hp_and_integrating_hp_are_a_matched_pair() {
        // IHP = trapezoid ∫ · HP: same poles; numerator K(1 − z⁻²) vs K²(1 − z⁻¹)².
        let hp = highpass2(0.075, 100.0);
        let ihp = integrating_highpass2(0.075, 100.0);
        assert_eq!(hp.a, ihp.a, "shared prewarped Butterworth poles");
        assert_eq!(hp.b[0], -0.5 * hp.b[1], "HP zeros at z = 1 (double)");
        assert_eq!(hp.b[0], hp.b[2]);
        assert_eq!(ihp.b[1], 0.0);
        assert_eq!(ihp.b[0], -ihp.b[2], "IHP zeros at z = ±1");
        let k = 200.0_f32;
        assert!(
            ((hp.b[0] / ihp.b[0]) - k).abs() <= 1e-3 * k,
            "b0 ratio = 2·f_s"
        );
        // Corner: |HP(e^{jωT})| = 1/√2 at f_c (prewarped exactly).
        let wt = 2.0 * core::f64::consts::PI * 0.075 / 100.0;
        let (c1, s1, c2, s2) = (wt.cos(), -wt.sin(), (2.0 * wt).cos(), -(2.0 * wt).sin());
        let (b, a) = (hp.b.map(f64::from), hp.a.map(f64::from));
        let nr = b[0] + b[1] * c1 + b[2] * c2;
        let ni = b[1] * s1 + b[2] * s2;
        let dr = 1.0 + a[0] * c1 + a[1] * c2;
        let di = a[0] * s1 + a[1] * s2;
        let mag = ((nr * nr + ni * ni) / (dr * dr + di * di)).sqrt();
        assert!(
            (mag - core::f64::consts::FRAC_1_SQRT_2).abs() < 2e-3,
            "−3 dB at 0.075 Hz (f32 coefficients)"
        );
        assert_eq!(
            highpass2(0.0, 100.0),
            Biquad::IDENTITY,
            "invalid corner → identity"
        );
    }
}
