//! Ground-motion additions: near-fault pulse (Baker 2007 Tp), fling step, basin resonance, long-duration M9. Owner: X4 (API frozen by W0a).
//!
//! EXOTIC §X15 (f64; PHYSICS §19 for the core Kanai–Tajimi/Clough–Penzien synthesis).
//! Deterministic generators complementing the Python synthesis (`fabrail.pwave`,
//! `fabrail.ground`):
//!
//! * [`NearFaultPulse`]: Mavroeidis–Papageorgiou (BSSA 93(3), 2003) velocity pulse
//!   v(t) = (A/2)·[1 + cos(2π·f_p·τ/γ)]·cos(2π·f_p·τ + ν), |τ| ≤ γ/(2f_p), τ = t − t0,
//!   with the pulse period from ln Tp = −6.45 + 1.11·Mw (Baker 2007, BSSA 97(5); σ_lnTp
//!   0.56) \[UNCERTAIN: coefficients as widely cited, not re-read from the primary
//!   source in this build\]. The same pulse, scaled to a peak of up to 1 g, is the
//!   vertical pulse of scenario XS05 ([`NearFaultPulse::scaled_to_peak_accel`]).
//! * [`FlingStep`]: a permanent displacement d over rise time t_r as one full cycle of
//!   sine acceleration (velocity returns to zero, displacement ends at d).
//! * [`basin_series`]: narrow-band basin motion, white noise through a Kanai–Tajimi
//!   filter (ω_g = 2π/6, ζ_g 0.05–0.15) and a Clough–Penzien high-pass (ω_f = 0.1·ω_g,
//!   ζ_f 0.6), 5 % cosine tapers, scaled numerically to the RMS.
//! * [`m9_long_duration_series`]: long-duration M9 (150–300 s) as Clough–Penzien
//!   filtered noise under a double Jennings–Housner–Tsai envelope (two wave packets, the
//!   second at 0.3·D with 0.9 of the amplitude, Tohoku-like) \[UNCERTAIN shape\], scaled
//!   numerically to the PGA.
//!
//! White noise comes from the in-repo Xoshiro256++ (`fabrail_sim::rng`), so every
//! series is a pure function of its seed on every host.

use fabrail_params::derive::G0_MPS2;
use fabrail_sim::rng::{NormalStream, Xoshiro256pp};
use fabrail_types::fmath;

/// Clough–Penzien high-pass frequency ratio ω_f/ω_g (PHYSICS §19: ω_f = 0.1·ω_g).
pub const CP_OMEGA_RATIO: f64 = 0.1;

/// Clough–Penzien high-pass damping for the narrow-band basin filter (chosen above the
/// basin ζ_g so the high-pass adds no second resonance) \[UNCERTAIN\].
pub const CP_ZETA_BASIN: f64 = 0.6;

/// Kanai–Tajimi ground frequency of the M9 record, rad/s (medium soil, PHYSICS §19).
pub const M9_OMEGA_G_RAD_S: f64 = 10.0;

/// Kanai–Tajimi ground damping of the M9 record (medium soil, PHYSICS §19).
pub const M9_ZETA_G: f64 = 0.4;

/// Largest vertical pulse the scenarios use, in g (XS05).
pub const VERTICAL_PULSE_MAX_G: f64 = 1.0;

/// Near-fault pulse period from moment magnitude, ln Tp = −6.45 + 1.11·Mw, s.
pub fn pulse_period_s(mw: f64) -> f64 {
    fmath::exp(-6.45 + 1.11 * mw)
}

/// Near-fault velocity pulse (Mavroeidis–Papageorgiou form).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NearFaultPulse {
    /// Peak velocity, m/s.
    pub v_p_mps: f64,
    /// Pulse period, s.
    pub t_p_s: f64,
    /// Oscillation parameter γ.
    pub gamma: f64,
    /// Phase ν, rad.
    pub nu_rad: f64,
    /// Pulse centre time, s.
    pub t0_s: f64,
}

impl NearFaultPulse {
    /// A pulse for magnitude `mw` (Baker 2007 period), peak velocity `v_p_mps` and centre
    /// `t0_s`, with γ = 2 and ν = 0 (a symmetric pulse with zero net displacement).
    pub fn baker2007(mw: f64, v_p_mps: f64, t0_s: f64) -> NearFaultPulse {
        NearFaultPulse {
            v_p_mps,
            t_p_s: pulse_period_s(mw),
            gamma: 2.0,
            nu_rad: 0.0,
            t0_s,
        }
    }

    /// Half-duration γ/(2·f_p) = γ·Tp/2, s.
    pub fn half_duration_s(&self) -> f64 {
        0.5 * self.gamma * self.t_p_s
    }

    /// Ground velocity at `t_s`, m/s.
    pub fn velocity_mps(&self, t_s: f64) -> f64 {
        if !(self.t_p_s > 0.0 && self.gamma > 0.0) {
            return 0.0;
        }
        let tau = t_s - self.t0_s;
        if fmath::abs(tau) > self.half_duration_s() {
            return 0.0;
        }
        let w = 2.0 * core::f64::consts::PI / self.t_p_s;
        0.5 * self.v_p_mps
            * (1.0 + fmath::cos(w * tau / self.gamma))
            * fmath::cos(w * tau + self.nu_rad)
    }

    /// Ground acceleration at `t_s` (the analytic derivative of the velocity), m/s².
    pub fn accel_mps2(&self, t_s: f64) -> f64 {
        if !(self.t_p_s > 0.0 && self.gamma > 0.0) {
            return 0.0;
        }
        let tau = t_s - self.t0_s;
        if fmath::abs(tau) > self.half_duration_s() {
            return 0.0;
        }
        let w = 2.0 * core::f64::consts::PI / self.t_p_s;
        let a = self.v_p_mps;
        let env = 1.0 + fmath::cos(w * tau / self.gamma);
        let d_env = -(w / self.gamma) * fmath::sin(w * tau / self.gamma);
        0.5 * a
            * (d_env * fmath::cos(w * tau + self.nu_rad)
                - env * w * fmath::sin(w * tau + self.nu_rad))
    }

    /// Largest |acceleration| over the pulse (sampled at Tp/2000), m/s².
    pub fn peak_accel_mps2(&self) -> f64 {
        let h = self.half_duration_s();
        if !(h > 0.0 && h.is_finite()) {
            return 0.0;
        }
        let n = 4000_u32;
        (0..=n)
            .map(|i| {
                let t = self.t0_s - h + 2.0 * h * f64::from(i) / f64::from(n);
                fmath::abs(self.accel_mps2(t))
            })
            .fold(0.0, f64::max)
    }

    /// The same pulse scaled so its peak acceleration is `a_peak_mps2` (the XS05 vertical
    /// pulse uses up to 1 g).
    pub fn scaled_to_peak_accel(&self, a_peak_mps2: f64) -> NearFaultPulse {
        let p = self.peak_accel_mps2();
        let s = if p > 0.0 { a_peak_mps2 / p } else { 0.0 };
        NearFaultPulse {
            v_p_mps: self.v_p_mps * s,
            ..*self
        }
    }
}

/// Fling step: permanent displacement `d_m` over rise time `t_r_s` starting at `t0_s`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlingStep {
    /// Permanent displacement, m (5–9 m for the Chi-Chi case).
    pub d_m: f64,
    /// Rise time, s.
    pub t_r_s: f64,
    /// Start time, s.
    pub t0_s: f64,
}

impl FlingStep {
    /// Normalised time τ = (t − t0)/t_r clamped to [0, 1].
    fn tau(&self, t_s: f64) -> f64 {
        if self.t_r_s <= 0.0 {
            return if t_s >= self.t0_s { 1.0 } else { 0.0 };
        }
        ((t_s - self.t0_s) / self.t_r_s).clamp(0.0, 1.0)
    }

    /// Ground displacement d·(τ − sin(2πτ)/(2π)), m.
    pub fn displacement_m(&self, t_s: f64) -> f64 {
        let tau = self.tau(t_s);
        let tp = 2.0 * core::f64::consts::PI;
        self.d_m * (tau - fmath::sin(tp * tau) / tp)
    }

    /// Ground velocity (d/t_r)·(1 − cos 2πτ), m/s.
    pub fn velocity_mps(&self, t_s: f64) -> f64 {
        if self.t_r_s <= 0.0 || t_s < self.t0_s || t_s > self.t0_s + self.t_r_s {
            return 0.0;
        }
        let tau = self.tau(t_s);
        self.d_m / self.t_r_s * (1.0 - fmath::cos(2.0 * core::f64::consts::PI * tau))
    }

    /// Ground acceleration (2π·d/t_r²)·sin 2πτ, m/s².
    pub fn accel_mps2(&self, t_s: f64) -> f64 {
        if self.t_r_s <= 0.0 || t_s < self.t0_s || t_s > self.t0_s + self.t_r_s {
            return 0.0;
        }
        let tau = self.tau(t_s);
        let tp = 2.0 * core::f64::consts::PI;
        tp * self.d_m / (self.t_r_s * self.t_r_s) * fmath::sin(tp * tau)
    }

    /// Peak acceleration 2π·d/t_r², m/s².
    pub fn peak_accel_mps2(&self) -> f64 {
        if self.t_r_s <= 0.0 {
            return f64::INFINITY;
        }
        2.0 * core::f64::consts::PI * fmath::abs(self.d_m) / (self.t_r_s * self.t_r_s)
    }
}

/// Narrow-band basin resonance (Kanai–Tajimi filtered noise).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BasinResonance {
    /// Basin angular frequency ω_g, rad/s (2π/6).
    pub omega_g_rad_s: f64,
    /// Basin damping ζ_g (0.05–0.15).
    pub zeta_g: f64,
    /// RMS acceleration, m/s².
    pub a_rms_mps2: f64,
    /// Duration, s (> 10 min).
    pub duration_s: f64,
}

/// Second-order filter state for the Kanai–Tajimi × Clough–Penzien cascade.
#[derive(Clone, Copy, Debug, Default)]
struct KtCp {
    wg: f64,
    zg: f64,
    wf: f64,
    zf: f64,
    // Kanai–Tajimi oscillator (relative response to the white-noise base input).
    x1: f64,
    v1: f64,
    // Clough–Penzien oscillator driven by the Kanai–Tajimi output.
    x2: f64,
    v2: f64,
}

impl KtCp {
    fn new(wg: f64, zg: f64, wf: f64, zf: f64) -> KtCp {
        KtCp {
            wg,
            zg,
            wf,
            zf,
            ..KtCp::default()
        }
    }

    /// Derivatives of (x1, v1, x2, v2) for noise input `w`.
    fn deriv(&self, s: [f64; 4], w: f64) -> [f64; 4] {
        let a1 = -2.0 * self.zg * self.wg * s[1] - self.wg * self.wg * s[0] - w;
        // Kanai–Tajimi absolute acceleration output.
        let a_kt = -2.0 * self.zg * self.wg * s[1] - self.wg * self.wg * s[0];
        let a2 = -2.0 * self.zf * self.wf * s[3] - self.wf * self.wf * s[2] - a_kt;
        [s[1], a1, s[3], a2]
    }

    /// One RK4 step with the noise held; returns the Clough–Penzien ground acceleration.
    fn step(&mut self, w: f64, h: f64) -> f64 {
        let s = [self.x1, self.v1, self.x2, self.v2];
        let add = |a: [f64; 4], b: [f64; 4], k: f64| -> [f64; 4] {
            [
                a[0] + k * b[0],
                a[1] + k * b[1],
                a[2] + k * b[2],
                a[3] + k * b[3],
            ]
        };
        let k1 = self.deriv(s, w);
        let k2 = self.deriv(add(s, k1, 0.5 * h), w);
        let k3 = self.deriv(add(s, k2, 0.5 * h), w);
        let k4 = self.deriv(add(s, k3, h), w);
        let n = [
            s[0] + h / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]),
            s[1] + h / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]),
            s[2] + h / 6.0 * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]),
            s[3] + h / 6.0 * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]),
        ];
        self.x1 = n[0];
        self.v1 = n[1];
        self.x2 = n[2];
        self.v2 = n[3];
        // Kanai–Tajimi: ẍ1 + 2ζ_g·ω_g·ẋ1 + ω_g²·x1 = −w, a_KT = −(2ζ_g·ω_g·ẋ1 + ω_g²·x1).
        // Clough–Penzien: ẍ2 + 2ζ_f·ω_f·ẋ2 + ω_f²·x2 = −a_KT; the ground acceleration is
        // the relative acceleration ẍ2, whose transfer from a_KT is the high-pass
        // r²/√((1 − r²)² + (2ζ_f·r)²), r = ω/ω_f (PHYSICS §19).
        let a_kt = -2.0 * self.zg * self.wg * self.v1 - self.wg * self.wg * self.x1;
        -2.0 * self.zf * self.wf * self.v2 - self.wf * self.wf * self.x2 - a_kt
    }
}

/// Filtered noise of `n` samples at `fs_hz` through the KT × CP cascade (unit-variance
/// Gaussian input per sample), unscaled.
fn kt_cp_noise(n: usize, fs_hz: f64, wg: f64, zg: f64, wf: f64, zf: f64, seed: u64) -> Vec<f64> {
    let mut out = Vec::with_capacity(n);
    if !(fs_hz > 0.0 && fs_hz.is_finite()) {
        return vec![0.0; n];
    }
    let h = 1.0 / fs_hz;
    let mut rng = NormalStream::new(Xoshiro256pp::from_seed(seed));
    let mut f = KtCp::new(wg, zg, wf, zf);
    // Warm-up of 5 KT time constants so the output starts stationary.
    let warm = if zg > 0.0 && wg > 0.0 {
        fmath::ceil(5.0 / (zg * wg) * fs_hz).min(1.0e6) as usize
    } else {
        0
    };
    for _ in 0..warm {
        let w = rng.next_normal() * fmath::sqrt(fs_hz);
        f.step(w, h);
    }
    for _ in 0..n {
        let w = rng.next_normal() * fmath::sqrt(fs_hz);
        out.push(f.step(w, h));
    }
    out
}

/// Scales `x` so its RMS equals `rms` (no-op for an all-zero series).
fn scale_to_rms(x: &mut [f64], rms: f64) {
    if x.is_empty() {
        return;
    }
    let m = x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64;
    if m > 0.0 {
        let s = rms / fmath::sqrt(m);
        for v in x.iter_mut() {
            *v *= s;
        }
    }
}

/// Samples a generator on a uniform grid (`n` samples at `fs_hz`) with seed `seed`,
/// m/s²: narrow-band basin motion (KT at ω_g, ζ_g; CP high-pass at 0.1·ω_g), 5 % cosine
/// tapers, scaled numerically to `a_rms_mps2`.
pub fn basin_series(b: &BasinResonance, fs_hz: f64, n: usize, seed: u64) -> Vec<f64> {
    if !(b.omega_g_rad_s > 0.0 && fs_hz > 0.0) || n == 0 {
        return vec![0.0; n];
    }
    let mut x = kt_cp_noise(
        n,
        fs_hz,
        b.omega_g_rad_s,
        b.zeta_g.max(1.0e-3),
        CP_OMEGA_RATIO * b.omega_g_rad_s,
        CP_ZETA_BASIN,
        seed,
    );
    let nt = (n / 20).max(1);
    for i in 0..nt.min(n) {
        let w = 0.5 * (1.0 - fmath::cos(core::f64::consts::PI * i as f64 / nt as f64));
        x[i] *= w;
        x[n - 1 - i] *= w;
    }
    scale_to_rms(&mut x, b.a_rms_mps2);
    x
}

/// Jennings–Housner–Tsai envelope: (t/t1)² before t1, 1 until t2, e^(−c(t − t2)) after.
pub fn jht_envelope(t_s: f64, t1_s: f64, t2_s: f64, c_per_s: f64) -> f64 {
    if t_s <= 0.0 {
        0.0
    } else if t_s < t1_s {
        (t_s / t1_s) * (t_s / t1_s)
    } else if t_s <= t2_s {
        1.0
    } else {
        fmath::exp(-c_per_s * (t_s - t2_s))
    }
}

/// Double envelope of the M9 record over duration `d_s`: packet A at 0 (rise 0.05·D,
/// plateau to 0.25·D) and packet B at 0.3·D with 0.9 of the amplitude (same shape),
/// decaying so the tail is 5 % at D; combined as √(e_A² + e_B²), normalised to ≤ 1.
pub fn m9_double_envelope(t_s: f64, d_s: f64) -> f64 {
    if !(d_s > 0.0 && d_s.is_finite()) {
        return 0.0;
    }
    let t1 = 0.05 * d_s;
    let t2 = 0.25 * d_s;
    let c = fmath::ln(20.0) / (0.45 * d_s);
    let ea = jht_envelope(t_s, t1, t2, c);
    let eb = 0.9 * jht_envelope(t_s - 0.3 * d_s, t1, t2, c);
    fmath::sqrt(ea * ea + eb * eb).min(1.0)
}

/// Long-duration M9 (double-envelope Clough–Penzien) record, m/s²: KT × CP noise (ω_g
/// 10 rad/s, ζ_g 0.4; ω_f = 0.1·ω_g, ζ_f = ζ_g as PHYSICS §19) under
/// [`m9_double_envelope`], scaled numerically so max|a| = `pga_mps2`.
pub fn m9_long_duration_series(duration_s: f64, fs_hz: f64, pga_mps2: f64, seed: u64) -> Vec<f64> {
    let n = if duration_s > 0.0 && fs_hz > 0.0 {
        fmath::round(duration_s * fs_hz) as usize
    } else {
        0
    };
    if n == 0 {
        return Vec::new();
    }
    let mut x = kt_cp_noise(
        n,
        fs_hz,
        M9_OMEGA_G_RAD_S,
        M9_ZETA_G,
        CP_OMEGA_RATIO * M9_OMEGA_G_RAD_S,
        M9_ZETA_G,
        seed,
    );
    for (i, v) in x.iter_mut().enumerate() {
        *v *= m9_double_envelope(i as f64 / fs_hz, duration_s);
    }
    let peak = x.iter().fold(0.0_f64, |a, v| a.max(fmath::abs(*v)));
    if peak > 0.0 {
        let s = pga_mps2 / peak;
        for v in x.iter_mut() {
            *v *= s;
        }
    }
    x
}

/// A vertical pulse of peak `a_peak_g`·g (≤ 1 g) for magnitude `mw`, centred at `t0_s`
/// (scenario XS05).
pub fn vertical_pulse(mw: f64, a_peak_g: f64, t0_s: f64) -> NearFaultPulse {
    let a = a_peak_g.clamp(0.0, VERTICAL_PULSE_MAX_G) * G0_MPS2;
    NearFaultPulse::baker2007(mw, 1.0, t0_s).scaled_to_peak_accel(a)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    #[test]
    fn near_fault_pulse_baker_and_mp_form() {
        // Baker 2007: ln Tp = −6.45 + 1.11·Mw → Tp(7) = e^1.32 = 3.743 s (E).
        assert!(rel(pulse_period_s(7.0), 1.32_f64.exp()) < 1e-12);
        let p = NearFaultPulse::baker2007(7.0, 1.2, 10.0);
        // ν = 0: the peak velocity is v_p at t0.
        assert!(rel(p.velocity_mps(10.0), 1.2) < 1e-12);
        assert_eq!(p.velocity_mps(10.0 + p.half_duration_s() + 1e-9), 0.0);
        assert_eq!(p.accel_mps2(10.0 - p.half_duration_s() - 1e-9), 0.0);
        // Acceleration is the derivative of velocity (central difference, rel 1e-6).
        for t in [8.0, 9.3, 10.4, 11.7] {
            let h = 1.0e-5;
            let fd = (p.velocity_mps(t + h) - p.velocity_mps(t - h)) / (2.0 * h);
            assert!(
                (fd - p.accel_mps2(t)).abs() <= 1e-6 * p.peak_accel_mps2(),
                "a = dv/dt at {t}"
            );
        }
        // γ = 2, ν = 0: no net displacement (trapezoid of v over the pulse, ≤ 1e-6 m).
        let (t0, t1) = (10.0 - p.half_duration_s(), 10.0 + p.half_duration_s());
        let n = 20_000;
        let h = (t1 - t0) / f64::from(n);
        let disp: f64 = (0..n)
            .map(|i| {
                0.5 * (p.velocity_mps(t0 + f64::from(i) * h)
                    + p.velocity_mps(t0 + f64::from(i + 1) * h))
                    * h
            })
            .sum();
        assert!(disp.abs() < 1e-6, "zero net displacement: {disp}");
        // The 1 g vertical pulse of XS05.
        let v = vertical_pulse(7.0, 1.0, 5.0);
        assert!(rel(v.peak_accel_mps2(), G0_MPS2) < 1e-9);
        assert!(
            rel(vertical_pulse(7.0, 3.0, 5.0).peak_accel_mps2(), G0_MPS2) < 1e-9,
            "capped at 1 g"
        );
    }

    #[test]
    fn fling_step_permanent_offset() {
        let f = FlingStep {
            d_m: 6.0,
            t_r_s: 4.0,
            t0_s: 2.0,
        };
        assert_eq!(f.displacement_m(1.0), 0.0);
        assert!(
            rel(f.displacement_m(6.0), 6.0) < 1e-12,
            "ends at d (Chi-Chi 5–9 m)"
        );
        assert!(
            rel(f.displacement_m(4.0), 3.0) < 1e-12,
            "half-way at mid-rise"
        );
        assert_eq!(f.velocity_mps(6.5), 0.0);
        assert!(
            rel(f.velocity_mps(4.0), 2.0 * 6.0 / 4.0) < 1e-12,
            "peak velocity 2d/t_r"
        );
        assert!(
            rel(
                f.peak_accel_mps2(),
                2.0 * core::f64::consts::PI * 6.0 / 16.0
            ) < 1e-12
        );
        // Integrating the acceleration twice recovers the step (RK-free trapezoid, 1e-6).
        let dt = 1.0e-4;
        let (mut v, mut x, mut t) = (0.0, 0.0, 0.0);
        while t < 8.0 {
            let a0 = f.accel_mps2(t);
            let a1 = f.accel_mps2(t + dt);
            let v1 = v + 0.5 * (a0 + a1) * dt;
            x += 0.5 * (v + v1) * dt;
            v = v1;
            t += dt;
        }
        assert!((x - 6.0).abs() < 1e-5 && v.abs() < 1e-9);
    }

    #[test]
    fn basin_series_narrow_band() {
        let b = BasinResonance {
            omega_g_rad_s: 2.0 * core::f64::consts::PI / 6.0,
            zeta_g: 0.08,
            a_rms_mps2: 0.4,
            duration_s: 900.0,
        };
        let fs = 20.0;
        let n = (b.duration_s * fs) as usize;
        let x = basin_series(&b, fs, n, 5);
        assert_eq!(x.len(), n);
        let rms = (x.iter().map(|v| v * v).sum::<f64>() / n as f64).sqrt();
        assert!(rel(rms, 0.4) < 1e-12, "scaled numerically to the RMS");
        assert_eq!(x[0], 0.0, "tapered start");
        assert_eq!(x, basin_series(&b, fs, n, 5), "deterministic per seed");
        assert_ne!(x, basin_series(&b, fs, n, 6));
        // Narrow band: the periodogram peaks near f_g = 1/6 Hz (DFT on a 2.5 mHz grid,
        // ±10 %), and the band 0.1–0.25 Hz holds most of the power.
        let power = |f: f64| -> f64 {
            let w = 2.0 * core::f64::consts::PI * f / fs;
            let (mut re, mut im) = (0.0, 0.0);
            for (k, v) in x.iter().enumerate() {
                re += v * (w * k as f64).cos();
                im -= v * (w * k as f64).sin();
            }
            re * re + im * im
        };
        let grid: Vec<f64> = (4..=200).map(|i| 0.0025 * f64::from(i)).collect();
        let spec: Vec<f64> = grid.iter().map(|f| power(*f)).collect();
        let (i_max, _) = spec
            .iter()
            .enumerate()
            .fold((0, 0.0), |a, (i, p)| if *p > a.1 { (i, *p) } else { a });
        let in_band: f64 = grid
            .iter()
            .zip(&spec)
            .filter(|(f, _)| (0.1..=0.25).contains(*f))
            .map(|(_, p)| p)
            .sum();
        let total: f64 = spec.iter().sum();
        println!(
            "basin periodogram peak {:.4} Hz vs f_g {:.4} Hz; 0.1–0.25 Hz share {:.2}",
            grid[i_max],
            1.0 / 6.0,
            in_band / total
        );
        assert!(rel(grid[i_max], 1.0 / 6.0) < 0.10, "dominant period ≈ 6 s");
        assert!(in_band / total > 0.5, "narrow band");
    }

    #[test]
    fn m9_double_envelope_record() {
        let d = 240.0;
        let fs = 50.0;
        let pga = 0.3 * G0_MPS2;
        let x = m9_long_duration_series(d, fs, pga, 3);
        assert_eq!(x.len(), (d * fs) as usize);
        let peak = x.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
        assert!(rel(peak, pga) < 1e-12, "scaled numerically to the PGA");
        assert_eq!(x, m9_long_duration_series(d, fs, pga, 3));
        // Two packets: energy in [0.3D, 0.55D] comparable to [0, 0.25D]; the tail is small.
        let energy = |a: f64, b: f64| -> f64 {
            x[(a * d * fs) as usize..(b * d * fs) as usize]
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
        };
        let (e1, e2, tail) = (energy(0.0, 0.25), energy(0.3, 0.55), energy(0.9, 1.0));
        assert!(
            e2 / e1 > 0.4 && e2 / e1 < 2.5,
            "second packet present: {:.2}",
            e2 / e1
        );
        assert!(tail / e1 < 0.1, "decayed tail");
        assert!(m9_double_envelope(0.0, d) == 0.0 && m9_double_envelope(0.2 * d, d) <= 1.0);
        assert!(m9_long_duration_series(0.0, fs, pga, 3).is_empty());
    }
}
