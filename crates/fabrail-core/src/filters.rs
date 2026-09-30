//! Discrete filters: biquad (Tustin with prewarp), first-order LP/HP, leaky integrator, filtered differentiator. Owner: A1.
//!
//! Frozen API (W0). Coefficients are designed in f64 at `new()` time (allowed: init-time
//! design, core design §A.4 item 1) through `fabrail_types::fmath`, then stored as f32;
//! `step` is f32 only. Biquads use transposed direct form II (TDF-II). The seismic
//! 0.5–20 Hz band-pass is a Butterworth [`Cascade`] (PHYSICS §20); the pressure
//! high-pass, IMU leaky integrator and observer differentiators are first order
//! (PHYSICS §16.2, §17).
//!
//! Design notes:
//! - Second-order sections use the bilinear (Tustin) map s = 2f_s·(1 − z⁻¹)/(1 + z⁻¹)
//!   with the corner prewarped, Ω = 2f_s·tan(π·f_c/f_s), so the digital −3 dB point sits
//!   exactly at f_c (Oppenheim & Schafer, "Discrete-Time Signal Processing", 3rd ed.,
//!   §7.1.2; the RBJ "Audio EQ Cookbook" coefficient forms).
//! - The Butterworth band-pass maps the n-th-order low-pass prototype poles
//!   p_k = e^{jπ(2k+n+1)/(2n)} through s = (p·B ± √(p²B² − 4Ω₀²))/2 with B = Ω_hi − Ω_lo,
//!   Ω₀ = √(Ω_lo·Ω_hi) (both edges prewarped), then bilinear-maps poles and zeros
//!   (s = 0 → z = 1, s = ∞ → z = −1); each section is normalised to unit gain at the
//!   digital centre frequency (Parks & Burrus, "Digital Filter Design", 1987, §7.3).
//!   Total order 2n uses n sections.
//! - First-order sections use the exact ZOH pole λ = e^{−2π·f_c/f_s} (step-invariant).
//!
//! Degenerate arguments (non-positive or non-finite frequencies, f_c ≥ f_s/2) give the
//! pass-through section for the biquads and a zero-gain filter for the integrator and
//! differentiator, so a misconfigured filter never injects energy.

use core::f64::consts::PI;
use fabrail_types::fmath;

/// Maximum biquad sections in a [`Cascade`] (an 8th-order band-pass; the preset
/// `bp_order` is 4 = 2 sections).
pub const MAX_SECTIONS: usize = 4;

/// Biquad y = (b0 + b1 z⁻¹ + b2 z⁻²)/(1 + a1 z⁻¹ + a2 z⁻²) in TDF-II.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Biquad {
    /// Numerator b0, b1, b2.
    pub b: [f32; 3],
    /// Denominator a1, a2 (a0 = 1).
    pub a: [f32; 2],
    /// TDF-II state.
    pub s: [f32; 2],
}

impl Default for Biquad {
    fn default() -> Biquad {
        Biquad::IDENTITY
    }
}

/// True when `f_hz` is a usable corner below Nyquist for sample rate `fs_hz`.
fn corner_ok(f_hz: f64, fs_hz: f64) -> bool {
    f_hz.is_finite() && fs_hz.is_finite() && f_hz > 0.0 && fs_hz > 0.0 && f_hz < 0.5 * fs_hz
}

impl Biquad {
    /// Pass-through section.
    pub const IDENTITY: Biquad = Biquad {
        b: [1.0, 0.0, 0.0],
        a: [0.0, 0.0],
        s: [0.0, 0.0],
    };

    /// A section from explicit coefficients (state zeroed).
    pub const fn new(b: [f32; 3], a: [f32; 2]) -> Biquad {
        Biquad { b, a, s: [0.0; 2] }
    }

    /// Section from f64 coefficients (a0 = 1).
    fn from_f64(b: [f64; 3], a: [f64; 2]) -> Biquad {
        Biquad::new(
            [b[0] as f32, b[1] as f32, b[2] as f32],
            [a[0] as f32, a[1] as f32],
        )
    }

    /// Second-order low-pass at `fc_hz` with quality `q`, Tustin with prewarp at fc:
    /// K = tan(π·f_c/f_s), b = K²·[1, 2, 1]/D, a = [2(K² − 1), 1 − K/Q + K²]/D,
    /// D = 1 + K/Q + K² (RBJ cookbook; Oppenheim & Schafer §7.1.2).
    pub fn lowpass(fc_hz: f64, q: f64, fs_hz: f64) -> Biquad {
        if !corner_ok(fc_hz, fs_hz) || q.is_nan() || q <= 0.0 {
            return Biquad::IDENTITY;
        }
        let k = fmath::tan(PI * fc_hz / fs_hz);
        let d = 1.0 + k / q + k * k;
        let b0 = k * k / d;
        Biquad::from_f64(
            [b0, 2.0 * b0, b0],
            [2.0 * (k * k - 1.0) / d, (1.0 - k / q + k * k) / d],
        )
    }

    /// Second-order high-pass at `fc_hz` with quality `q`, Tustin with prewarp at fc:
    /// b = [1, −2, 1]/D with the low-pass denominator.
    pub fn highpass(fc_hz: f64, q: f64, fs_hz: f64) -> Biquad {
        if !corner_ok(fc_hz, fs_hz) || q.is_nan() || q <= 0.0 {
            return Biquad::IDENTITY;
        }
        let k = fmath::tan(PI * fc_hz / fs_hz);
        let d = 1.0 + k / q + k * k;
        Biquad::from_f64(
            [1.0 / d, -2.0 / d, 1.0 / d],
            [2.0 * (k * k - 1.0) / d, (1.0 - k / q + k * k) / d],
        )
    }

    /// One sample (TDF-II: y = b0·x + s0; s0 ← b1·x − a1·y + s1; s1 ← b2·x − a2·y).
    #[inline]
    pub fn step(&mut self, x: f32) -> f32 {
        let y = self.b[0] * x + self.s[0];
        self.s[0] = self.b[1] * x - self.a[0] * y + self.s[1];
        self.s[1] = self.b[2] * x - self.a[1] * y;
        y
    }

    /// Zeroes the state.
    #[inline]
    pub fn reset(&mut self) {
        self.s = [0.0; 2];
    }

    /// Complex frequency response H(e^{jω}) at `f_hz` for sample rate `fs_hz`, as
    /// (re, im) in f64 (verification and plotting; not used on the fast path).
    pub fn response(&self, f_hz: f64, fs_hz: f64) -> (f64, f64) {
        let w = 2.0 * PI * f_hz / fs_hz;
        let z1 = Cx::new(fmath::cos(-w), fmath::sin(-w)); // z⁻¹
        let z2 = z1.mul(z1);
        let num = Cx::new(f64::from(self.b[0]), 0.0)
            .add(z1.scale(f64::from(self.b[1])))
            .add(z2.scale(f64::from(self.b[2])));
        let den = Cx::new(1.0, 0.0)
            .add(z1.scale(f64::from(self.a[0])))
            .add(z2.scale(f64::from(self.a[1])));
        let h = num.div(den);
        (h.re, h.im)
    }
}

/// Cascade of up to [`MAX_SECTIONS`] biquads (`n` active).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Cascade {
    /// Sections; only the first `n` are applied.
    pub sec: [Biquad; MAX_SECTIONS],
    /// Active sections.
    pub n: u8,
}

impl Cascade {
    /// One sample through the active sections.
    #[inline]
    pub fn step(&mut self, x: f32) -> f32 {
        let n = usize::from(self.n);
        self.sec.iter_mut().take(n).fold(x, |acc, s| s.step(acc))
    }

    /// Zeroes every section's state.
    pub fn reset(&mut self) {
        for s in self.sec.iter_mut() {
            s.reset();
        }
    }

    /// Magnitude |H(e^{jω})| of the active sections at `f_hz` (f64; verification).
    pub fn magnitude(&self, f_hz: f64, fs_hz: f64) -> f64 {
        let n = usize::from(self.n);
        self.sec
            .iter()
            .take(n)
            .map(|s| {
                let (re, im) = s.response(f_hz, fs_hz);
                fmath::hypot(re, im)
            })
            .product()
    }
}

/// Butterworth band-pass of total order `order` (even, ≤ 2·MAX_SECTIONS) between
/// `lo_hz` and `hi_hz`, prewarped Tustin, as a TDF-II cascade (PHYSICS §20; seismic
/// preset 4th order, 0.5–20 Hz at 100 Hz). Unit gain at the digital centre frequency
/// 2·atan(Ω₀/2f_s)·f_s/π; exactly −3.01 dB at `lo_hz` and `hi_hz`.
///
/// Returns an empty (pass-through) cascade for an odd, zero or too-large order, or for
/// band edges that are not 0 < lo < hi < f_s/2.
pub fn butterworth_bandpass(order: u8, lo_hz: f64, hi_hz: f64, fs_hz: f64) -> Cascade {
    let mut out = Cascade::default();
    let n = usize::from(order / 2);
    if order == 0
        || !order.is_multiple_of(2)
        || n > MAX_SECTIONS
        || !corner_ok(lo_hz, fs_hz)
        || !corner_ok(hi_hz, fs_hz)
        || lo_hz >= hi_hz
    {
        return out;
    }
    let fs2 = 2.0 * fs_hz;
    let w_lo = fs2 * fmath::tan(PI * lo_hz / fs_hz);
    let w_hi = fs2 * fmath::tan(PI * hi_hz / fs_hz);
    let bw = w_hi - w_lo;
    let w0 = fmath::sqrt(w_lo * w_hi);
    // Digital centre z0 = e^{jθ0}, θ0 = 2·atan(Ω0/2fs).
    let th0 = 2.0 * fmath::atan(w0 / fs2);
    let z0 = Cx::new(fmath::cos(th0), fmath::sin(th0));

    let mut k_sec = 0usize;
    let mut push = |p1: Cx, p2: Cx, sec: &mut [Biquad; MAX_SECTIONS]| {
        // Bilinear map of the two analog poles; zeros at z = 1 and z = −1.
        let z1 = bilinear(p1, fs2);
        let z2 = bilinear(p2, fs2);
        // Denominator (1 − z1 z⁻¹)(1 − z2 z⁻¹) = 1 − (z1 + z2) z⁻¹ + z1 z2 z⁻²; real for
        // conjugate or real pole pairs.
        let a1 = -(z1.add(z2)).re;
        let a2 = z1.mul(z2).re;
        // Numerator (1 − z⁻²)·g, g normalising |H(z0)| to 1.
        let zi = z0.inv();
        let zi2 = zi.mul(zi);
        let num = Cx::new(1.0, 0.0).sub(zi2);
        let den = Cx::new(1.0, 0.0).add(zi.scale(a1)).add(zi2.scale(a2));
        let mag = num.div(den).abs();
        let g = if mag > 0.0 { 1.0 / mag } else { 0.0 };
        if let Some(s) = sec.get_mut(k_sec) {
            *s = Biquad::from_f64([g, 0.0, -g], [a1, a2]);
        }
        k_sec += 1;
    };

    for k in 0..n {
        let ang = PI * (2.0 * k as f64 + n as f64 + 1.0) / (2.0 * n as f64);
        let p = Cx::new(fmath::cos(ang), fmath::sin(ang));
        if p.im > 1e-12 {
            // Upper-half-plane prototype pole: its two band-pass images and their
            // conjugates (from the conjugate prototype pole) form two sections.
            let (s1, s2) = lp_to_bp(p, bw, w0);
            push(s1, s1.conj(), &mut out.sec);
            push(s2, s2.conj(), &mut out.sec);
        } else if fmath::abs(p.im) <= 1e-12 {
            // Real prototype pole (odd n): one section from its two images.
            let (s1, s2) = lp_to_bp(Cx::new(p.re, 0.0), bw, w0);
            push(s1, s2, &mut out.sec);
        }
    }
    out.n = n as u8;
    out
}

/// Low-pass prototype pole → the two band-pass poles s = (pB ± √(p²B² − 4Ω₀²))/2.
fn lp_to_bp(p: Cx, bw: f64, w0: f64) -> (Cx, Cx) {
    let pb = p.scale(bw);
    let disc = pb.mul(pb).sub(Cx::new(4.0 * w0 * w0, 0.0)).csqrt();
    (pb.add(disc).scale(0.5), pb.sub(disc).scale(0.5))
}

/// Bilinear map z = (2f_s + s)/(2f_s − s).
fn bilinear(s: Cx, fs2: f64) -> Cx {
    Cx::new(fs2, 0.0).add(s).div(Cx::new(fs2, 0.0).sub(s))
}

/// First-order low-pass y += α(x − y), α from the exact ZOH pole e^(−2π·fc/fs).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FirstOrderLp {
    /// Smoothing factor α = 1 − e^(−2π·f_c/f_s).
    pub alpha: f32,
    /// Output state.
    pub y: f32,
}

impl FirstOrderLp {
    /// Designs the filter for corner `fc_hz` at sample rate `fs_hz` (α = 1, i.e.
    /// pass-through, for a degenerate corner).
    pub fn new(fc_hz: f64, fs_hz: f64) -> FirstOrderLp {
        let alpha = if fc_hz.is_finite() && fs_hz.is_finite() && fc_hz > 0.0 && fs_hz > 0.0 {
            -fmath::exp_m1(-2.0 * PI * fc_hz / fs_hz)
        } else {
            1.0
        };
        FirstOrderLp {
            alpha: alpha as f32,
            y: 0.0,
        }
    }

    /// One sample.
    #[inline]
    pub fn step(&mut self, x: f32) -> f32 {
        self.y += self.alpha * (x - self.y);
        self.y
    }

    /// Sets the state.
    #[inline]
    pub fn reset(&mut self, y0: f32) {
        self.y = y0;
    }
}

/// First-order high-pass (x − LP(x)) with the exact ZOH pole (F̂ pressure rejection at
/// 0.5 Hz, PHYSICS §17). Implemented as y\[k\] = λ·(y\[k−1\] + x\[k\] − x\[k−1\]) with
/// λ = e^(−2π·f_c/f_s), which equals x − LP(x) for the [`FirstOrderLp`] with the same
/// corner (substitute y_lp = x − y into the low-pass recursion).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FirstOrderHp {
    /// Pole factor λ = e^(−2π·f_c/f_s).
    pub alpha: f32,
    /// Previous input.
    pub x1: f32,
    /// Output state.
    pub y: f32,
}

impl FirstOrderHp {
    /// Designs the filter for corner `fc_hz` at sample rate `fs_hz` (λ = 1, i.e. a pure
    /// differencer-accumulator that passes everything but DC, for a degenerate corner).
    pub fn new(fc_hz: f64, fs_hz: f64) -> FirstOrderHp {
        let lambda = if fc_hz.is_finite() && fs_hz.is_finite() && fc_hz > 0.0 && fs_hz > 0.0 {
            fmath::exp(-2.0 * PI * fc_hz / fs_hz)
        } else {
            1.0
        };
        FirstOrderHp {
            alpha: lambda as f32,
            x1: 0.0,
            y: 0.0,
        }
    }

    /// One sample.
    #[inline]
    pub fn step(&mut self, x: f32) -> f32 {
        self.y = self.alpha * (self.y + x - self.x1);
        self.x1 = x;
        self.y
    }

    /// Zeroes the state.
    #[inline]
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.y = 0.0;
    }

    /// Zeroes the output and primes the previous input with `x0` (no startup step).
    #[inline]
    pub fn prime(&mut self, x0: f32) {
        self.x1 = x0;
        self.y = 0.0;
    }
}

/// Leaky integrator y ← λ·y + x·dt with λ = e^(−2π·f_hp·dt): an integrator high-passed
/// at `hp_hz` (IMU absolute velocity for skyhook, 0.1 Hz; PHYSICS §16.2, §21).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LeakyIntegrator {
    /// Leak factor λ.
    pub lambda: f32,
    /// Sample period, s.
    pub dt_s: f32,
    /// Output state.
    pub y: f32,
}

impl LeakyIntegrator {
    /// Designs the integrator for high-pass corner `hp_hz` at sample rate `fs_hz`
    /// (`hp_hz` = 0 gives a pure integrator; a non-positive `fs_hz` gives zero gain).
    pub fn new(hp_hz: f64, fs_hz: f64) -> LeakyIntegrator {
        if !fs_hz.is_finite() || fs_hz <= 0.0 || !hp_hz.is_finite() || hp_hz < 0.0 {
            return LeakyIntegrator::default();
        }
        let dt = 1.0 / fs_hz;
        LeakyIntegrator {
            lambda: fmath::exp(-2.0 * PI * hp_hz * dt) as f32,
            dt_s: dt as f32,
            y: 0.0,
        }
    }

    /// One sample.
    #[inline]
    pub fn step(&mut self, x: f32) -> f32 {
        self.y = self.lambda * self.y + x * self.dt_s;
        self.y
    }

    /// Sets the state.
    #[inline]
    pub fn reset(&mut self, y0: f32) {
        self.y = y0;
    }
}

/// Filtered differentiator s/(1 + s/ω_bw), Tustin with the corner prewarped
/// (observer fallbacks, touchdown and lift-off rates; PHYSICS §15):
/// y\[k\] = a·y\[k−1\] + g·(x\[k\] − x\[k−1\]), with Ω = 2f_s·tan(π·f_bw/f_s),
/// g = 2Ω/(2 + Ω·T), a = (2 − Ω·T)/(2 + Ω·T).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Differentiator {
    /// Coefficients [g (1/s), a].
    pub k: [f32; 2],
    /// Previous input.
    pub x1: f32,
    /// Output state.
    pub y: f32,
}

impl Differentiator {
    /// Designs the differentiator for bandwidth `bw_hz` at sample rate `fs_hz` (zero
    /// gain for a degenerate corner).
    pub fn new(bw_hz: f64, fs_hz: f64) -> Differentiator {
        if !corner_ok(bw_hz, fs_hz) {
            return Differentiator::default();
        }
        let t = 1.0 / fs_hz;
        let w = 2.0 * fs_hz * fmath::tan(PI * bw_hz / fs_hz);
        let d = 2.0 + w * t;
        Differentiator {
            k: [(2.0 * w / d) as f32, ((2.0 - w * t) / d) as f32],
            x1: 0.0,
            y: 0.0,
        }
    }

    /// One sample.
    #[inline]
    pub fn step(&mut self, x: f32) -> f32 {
        self.y = self.k[1] * self.y + self.k[0] * (x - self.x1);
        self.x1 = x;
        self.y
    }

    /// Zeroes the state and primes the previous input with `x0` (no startup kick).
    #[inline]
    pub fn reset(&mut self, x0: f32) {
        self.x1 = x0;
        self.y = 0.0;
    }
}

/// Minimal f64 complex arithmetic for init-time filter design.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Cx {
    re: f64,
    im: f64,
}

impl Cx {
    const fn new(re: f64, im: f64) -> Cx {
        Cx { re, im }
    }
    fn add(self, o: Cx) -> Cx {
        Cx::new(self.re + o.re, self.im + o.im)
    }
    fn sub(self, o: Cx) -> Cx {
        Cx::new(self.re - o.re, self.im - o.im)
    }
    fn mul(self, o: Cx) -> Cx {
        Cx::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
    fn scale(self, k: f64) -> Cx {
        Cx::new(self.re * k, self.im * k)
    }
    fn conj(self) -> Cx {
        Cx::new(self.re, -self.im)
    }
    fn abs(self) -> f64 {
        fmath::hypot(self.re, self.im)
    }
    fn div(self, o: Cx) -> Cx {
        let d = o.re * o.re + o.im * o.im;
        if d == 0.0 {
            return Cx::new(0.0, 0.0);
        }
        Cx::new(
            (self.re * o.re + self.im * o.im) / d,
            (self.im * o.re - self.re * o.im) / d,
        )
    }
    fn inv(self) -> Cx {
        Cx::new(1.0, 0.0).div(self)
    }
    /// Principal square root (named `csqrt` so the no_std float-method guard does not
    /// mistake it for `f64::sqrt`).
    fn csqrt(self) -> Cx {
        let r = self.abs();
        let re = fmath::sqrt(fmath::fmax(0.5 * (r + self.re), 0.0));
        let im = fmath::copysign(fmath::sqrt(fmath::fmax(0.5 * (r - self.re), 0.0)), self.im);
        Cx::new(re, im)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_biquad_passes_through() {
        let mut b = Biquad::IDENTITY;
        assert_eq!(b.step(1.25), 1.25);
        let mut c = Cascade::default();
        assert_eq!(c.step(-3.0), -3.0);
    }

    // req: FAB-SEI-002
    #[test]
    fn butterworth_prewarp() {
        // Seismic preset: 4-pole Butterworth band-pass 0.5–20 Hz at 100 Hz (PHYSICS §20).
        let fs = 100.0;
        let c = butterworth_bandpass(4, 0.5, 20.0, fs);
        assert_eq!(c.n, 2, "total order 4 = 2 sections");
        let inv_sqrt2 = core::f64::consts::FRAC_1_SQRT_2;
        for f in [0.5, 20.0] {
            let m = c.magnitude(f, fs);
            assert!(
                (m / inv_sqrt2 - 1.0).abs() < 1e-4,
                "prewarped edge {f} Hz at −3.01 dB: |H| = {m} (tol 1e-4 rel; SEI-002)"
            );
        }
        // Unit gain at the digital centre (geometric mean of the prewarped edges).
        let wl = 2.0 * fs * (PI * 0.5 / fs).tan();
        let wh = 2.0 * fs * (PI * 20.0 / fs).tan();
        let fc = (2.0 * ((wl * wh).sqrt() / (2.0 * fs)).atan()) * fs / (2.0 * PI);
        let m0 = c.magnitude(fc, fs);
        assert!((m0 - 1.0).abs() < 1e-4, "centre gain 1 (tol 1e-4): {m0}");
        // Butterworth band-pass magnitude from the analog prototype through the
        // prewarped map: |H|² = 1/(1 + ((Ω² − Ω0²)/(Ω·B))^(2n)), n = 2.
        for f in [0.05, 0.2, 1.0, 3.0, 10.0, 30.0, 45.0] {
            let w = 2.0 * fs * (PI * f / fs).tan();
            let x = (w * w - wl * wh) / (w * (wh - wl));
            let want = 1.0 / (1.0 + x.powi(4)).sqrt();
            let got = c.magnitude(f, fs);
            assert!(
                (got - want).abs() <= 1e-4 * want.max(1e-3),
                "{f} Hz: |H| = {got} vs analog-prototype {want} (tol 1e-4 rel; SEI-002 reference)"
            );
        }
        // Time domain: the f32 TDF-II cascade matches the f64 difference equations of
        // its own coefficients to 1e-4 relative over a chirp (SEI-002 f32 vs f64).
        let mut cf = c;
        let mut st = [[0.0f64; 2]; 2];
        let mut max_err = 0.0f64;
        let mut max_y = 0.0f64;
        for k in 0..4000 {
            let t = k as f64 / fs;
            let x = (2.0 * PI * (0.2 + 0.5 * t) * t).sin();
            let mut y = x;
            for (s, sec) in st.iter_mut().zip(c.sec.iter()) {
                let b: [f64; 3] = [sec.b[0].into(), sec.b[1].into(), sec.b[2].into()];
                let a: [f64; 2] = [sec.a[0].into(), sec.a[1].into()];
                let out = b[0] * y + s[0];
                s[0] = b[1] * y - a[0] * out + s[1];
                s[1] = b[2] * y - a[1] * out;
                y = out;
            }
            let yf = f64::from(cf.step(x as f32));
            max_err = max_err.max((yf - y).abs());
            max_y = max_y.max(y.abs());
        }
        assert!(
            max_err <= 1e-4 * max_y,
            "f32 cascade vs f64 recursion: {max_err} (tol 1e-4 of peak {max_y})"
        );
        // Degenerate designs are pass-through.
        assert_eq!(butterworth_bandpass(3, 0.5, 20.0, fs).n, 0);
        assert_eq!(butterworth_bandpass(4, 20.0, 0.5, fs).n, 0);
        assert_eq!(butterworth_bandpass(4, 0.5, 60.0, fs).n, 0);
        // Higher orders keep the −3 dB edges.
        for order in [2u8, 6, 8] {
            let c = butterworth_bandpass(order, 1.0, 10.0, fs);
            assert_eq!(usize::from(c.n), usize::from(order / 2));
            for f in [1.0, 10.0] {
                let m = c.magnitude(f, fs);
                assert!(
                    (m / inv_sqrt2 - 1.0).abs() < 1e-4,
                    "order {order} edge {f}: {m}"
                );
            }
        }
    }

    #[test]
    fn biquad_prewarped_corners() {
        let fs = 10_000.0;
        for fc in [10.0, 500.0, 3000.0] {
            let lp = Biquad::lowpass(fc, core::f64::consts::FRAC_1_SQRT_2, fs);
            let hp = Biquad::highpass(fc, core::f64::consts::FRAC_1_SQRT_2, fs);
            let (r, i) = lp.response(fc, fs);
            let (rh, ih) = hp.response(fc, fs);
            let m = (r * r + i * i).sqrt();
            let mh = (rh * rh + ih * ih).sqrt();
            assert!(
                (m - 0.5f64.sqrt()).abs() < 1e-5,
                "LP −3 dB at fc = {fc} (prewarp)"
            );
            assert!(
                (mh - 0.5f64.sqrt()).abs() < 1e-5,
                "HP −3 dB at fc = {fc} (prewarp)"
            );
            // DC gain 1 + a1 + a2 = 4K²/D cancels in f32 for fc ≪ fs (coefficient
            // quantisation 2⁻²⁴ against K² ≈ 1e-5 at 10 Hz / 10 kHz): tolerance scales
            // as 2⁻²⁴/K².
            let (r0, _) = lp.response(0.0, fs);
            let k = (PI * fc / fs).tan();
            let tol = (8.0 * 2f64.powi(-24) / (k * k)).max(1e-5);
            assert!(
                (r0 - 1.0).abs() < tol,
                "LP DC gain 1 (tol {tol:e}, f32 coefficients)"
            );
        }
        // Step response of the low-pass settles to 1.
        let mut lp = Biquad::lowpass(100.0, core::f64::consts::FRAC_1_SQRT_2, fs);
        let mut y = 0.0;
        for _ in 0..2000 {
            y = lp.step(1.0);
        }
        assert!((y - 1.0).abs() < 1e-4);
    }

    #[test]
    fn first_order_sections() {
        let fs = 1000.0;
        // LP: exact ZOH pole, so the step response is 1 − λ^k.
        let mut lp = FirstOrderLp::new(5.0, fs);
        let lam = (-2.0 * PI * 5.0 / fs).exp();
        let mut y = 0.0;
        for k in 1..=100 {
            y = lp.step(1.0);
            let want = 1.0 - lam.powi(k);
            assert!((f64::from(y) - want).abs() < 1e-5, "LP step at k = {k}");
        }
        assert!(y < 1.0);
        // HP = x − LP(x).
        let mut hp = FirstOrderHp::new(5.0, fs);
        let mut lp2 = FirstOrderLp::new(5.0, fs);
        for k in 0..200 {
            let x = ((k as f32) * 0.1).sin() + 0.5;
            let h = hp.step(x);
            let l = lp2.step(x);
            assert!((h - (x - l)).abs() < 1e-5, "HP ≡ x − LP(x) at k = {k}");
        }
        // Leaky integrator of a constant converges to x·dt/(1 − λ).
        let mut li = LeakyIntegrator::new(0.1, fs);
        let mut v = 0.0;
        for _ in 0..200_000 {
            v = li.step(1.0);
        }
        let want = (1.0 / fs) / (1.0 - (-2.0 * PI * 0.1 / fs).exp());
        assert!((f64::from(v) / want - 1.0).abs() < 1e-3, "leaky DC gain");
        // Pure integrator when hp = 0.
        let mut pi = LeakyIntegrator::new(0.0, fs);
        for _ in 0..1000 {
            v = pi.step(2.0);
        }
        assert!((v - 2.0).abs() < 1e-4, "∫2 dt over 1 s");
    }

    #[test]
    fn differentiator_tracks_ramp_slope() {
        let fs = 10_000.0;
        let mut d = Differentiator::new(200.0, fs);
        d.reset(0.0);
        let mut y = 0.0;
        for k in 1..=2000 {
            y = d.step(0.003 * k as f32 / fs as f32);
        }
        assert!((y - 0.003).abs() < 1e-6, "ramp slope 3 mm/s recovered: {y}");
        let z = Differentiator::new(0.0, fs);
        assert_eq!(z.k, [0.0, 0.0], "degenerate corner: zero gain");
    }
}
