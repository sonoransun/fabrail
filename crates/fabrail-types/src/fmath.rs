//! The ONLY math entry point for every fabrail crate (controller, monitor, plant, sim).
//!
//! All transcendental and rounding functions go through soft-float `libm` 0.2.16 with
//! `default-features = false, features = ["force-soft-floats"]`, so host (aarch64/x86),
//! MCU (thumbv7em, thumbv6m, armv7r) and wasm32 execute the same instruction-level
//! algorithm and produce the same bits (core design §A.4 item 4; PHYSICS §24).
//!
//! Banned in the no_std crates (grepped by `scripts/check_nostd.sh`): float method math
//! such as `.sin()`/`.exp()`/`.powi()`, `mul_add`, `num_traits::Float` and
//! `cfg(target_arch)`. `powi` is provided here as an explicit multiply loop because the
//! LLVM intrinsic is not guaranteed bit-identical across targets.
//!
//! Naming: `f32` functions carry an `f` suffix (`sqrtf`, `lnf`); `f64` functions are
//! bare (`sqrt`, `ln`). `ln` is the natural logarithm.

// ---------------------------------------------------------------------------------------
// f32 (controller)
// ---------------------------------------------------------------------------------------

/// Square root.
#[inline]
pub fn sqrtf(x: f32) -> f32 {
    libm::sqrtf(x)
}

/// Cube root.
#[inline]
pub fn cbrtf(x: f32) -> f32 {
    libm::cbrtf(x)
}

/// Sine (radians).
#[inline]
pub fn sinf(x: f32) -> f32 {
    libm::sinf(x)
}

/// Cosine (radians).
#[inline]
pub fn cosf(x: f32) -> f32 {
    libm::cosf(x)
}

/// Tangent (radians).
#[inline]
pub fn tanf(x: f32) -> f32 {
    libm::tanf(x)
}

/// Arcsine (radians).
#[inline]
pub fn asinf(x: f32) -> f32 {
    libm::asinf(x)
}

/// Arccosine (radians).
#[inline]
pub fn acosf(x: f32) -> f32 {
    libm::acosf(x)
}

/// Arctangent (radians).
#[inline]
pub fn atanf(x: f32) -> f32 {
    libm::atanf(x)
}

/// Four-quadrant arctangent of `y / x` (radians).
#[inline]
pub fn atan2f(y: f32, x: f32) -> f32 {
    libm::atan2f(y, x)
}

/// e^x.
#[inline]
pub fn expf(x: f32) -> f32 {
    libm::expf(x)
}

/// e^x − 1, accurate near 0.
#[inline]
pub fn exp_m1f(x: f32) -> f32 {
    libm::expm1f(x)
}

/// Natural logarithm.
#[inline]
pub fn lnf(x: f32) -> f32 {
    libm::logf(x)
}

/// ln(1 + x), accurate near 0.
#[inline]
pub fn ln_1pf(x: f32) -> f32 {
    libm::log1pf(x)
}

/// Base-10 logarithm.
#[inline]
pub fn log10f(x: f32) -> f32 {
    libm::log10f(x)
}

/// Base-2 logarithm.
#[inline]
pub fn log2f(x: f32) -> f32 {
    libm::log2f(x)
}

/// x^y.
#[inline]
pub fn powf(x: f32, y: f32) -> f32 {
    libm::powf(x, y)
}

/// x^n by explicit repeated multiplication (square-and-multiply); deterministic on
/// every target, unlike the `powi` intrinsic.
#[inline]
pub fn powif(x: f32, n: i32) -> f32 {
    let mut base = if n < 0 { 1.0 / x } else { x };
    let mut e = n.unsigned_abs();
    let mut acc = 1.0f32;
    while e > 0 {
        if e & 1 == 1 {
            acc *= base;
        }
        base *= base;
        e >>= 1;
    }
    acc
}

/// Hyperbolic tangent.
#[inline]
pub fn tanhf(x: f32) -> f32 {
    libm::tanhf(x)
}

/// Hyperbolic cosine.
#[inline]
pub fn coshf(x: f32) -> f32 {
    libm::coshf(x)
}

/// Hyperbolic sine.
#[inline]
pub fn sinhf(x: f32) -> f32 {
    libm::sinhf(x)
}

/// sqrt(x² + y²) without undue overflow.
#[inline]
pub fn hypotf(x: f32, y: f32) -> f32 {
    libm::hypotf(x, y)
}

/// Absolute value.
#[inline]
pub fn absf(x: f32) -> f32 {
    libm::fabsf(x)
}

/// Magnitude of `x` with the sign of `s`.
#[inline]
pub fn copysignf(x: f32, s: f32) -> f32 {
    libm::copysignf(x, s)
}

/// Largest integer ≤ x.
#[inline]
pub fn floorf(x: f32) -> f32 {
    libm::floorf(x)
}

/// Smallest integer ≥ x.
#[inline]
pub fn ceilf(x: f32) -> f32 {
    libm::ceilf(x)
}

/// Round half away from zero.
#[inline]
pub fn roundf(x: f32) -> f32 {
    libm::roundf(x)
}

/// Round toward zero.
#[inline]
pub fn truncf(x: f32) -> f32 {
    libm::truncf(x)
}

/// Floating remainder of x / y with the sign of x.
#[inline]
pub fn fmodf(x: f32, y: f32) -> f32 {
    libm::fmodf(x, y)
}

/// Minimum; returns the non-NaN operand if exactly one is NaN (IEEE minNum).
#[inline]
pub fn fminf(x: f32, y: f32) -> f32 {
    libm::fminf(x, y)
}

/// Maximum; returns the non-NaN operand if exactly one is NaN (IEEE maxNum).
#[inline]
pub fn fmaxf(x: f32, y: f32) -> f32 {
    libm::fmaxf(x, y)
}

/// Clamp `x` into `[lo, hi]` without panicking (unlike `f32::clamp`, which asserts
/// `lo <= hi`). NaN passes through unchanged so callers can detect it.
#[inline]
pub fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// Error function.
#[inline]
pub fn erff(x: f32) -> f32 {
    libm::erff(x)
}

// ---------------------------------------------------------------------------------------
// f64 (plant, params::derive, init-time gain design)
// ---------------------------------------------------------------------------------------

/// Square root.
#[inline]
pub fn sqrt(x: f64) -> f64 {
    libm::sqrt(x)
}

/// Cube root.
#[inline]
pub fn cbrt(x: f64) -> f64 {
    libm::cbrt(x)
}

/// Sine (radians).
#[inline]
pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}

/// Cosine (radians).
#[inline]
pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}

/// Tangent (radians).
#[inline]
pub fn tan(x: f64) -> f64 {
    libm::tan(x)
}

/// Arcsine (radians).
#[inline]
pub fn asin(x: f64) -> f64 {
    libm::asin(x)
}

/// Arccosine (radians).
#[inline]
pub fn acos(x: f64) -> f64 {
    libm::acos(x)
}

/// Arctangent (radians).
#[inline]
pub fn atan(x: f64) -> f64 {
    libm::atan(x)
}

/// Four-quadrant arctangent of `y / x` (radians).
#[inline]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

/// e^x.
#[inline]
pub fn exp(x: f64) -> f64 {
    libm::exp(x)
}

/// e^x − 1, accurate near 0.
#[inline]
pub fn exp_m1(x: f64) -> f64 {
    libm::expm1(x)
}

/// Natural logarithm.
#[inline]
pub fn ln(x: f64) -> f64 {
    libm::log(x)
}

/// ln(1 + x), accurate near 0.
#[inline]
pub fn ln_1p(x: f64) -> f64 {
    libm::log1p(x)
}

/// Base-10 logarithm.
#[inline]
pub fn log10(x: f64) -> f64 {
    libm::log10(x)
}

/// Base-2 logarithm.
#[inline]
pub fn log2(x: f64) -> f64 {
    libm::log2(x)
}

/// x^y.
#[inline]
pub fn pow(x: f64, y: f64) -> f64 {
    libm::pow(x, y)
}

/// x^n by explicit repeated multiplication (square-and-multiply); deterministic on
/// every target, unlike the `powi` intrinsic.
#[inline]
pub fn powi(x: f64, n: i32) -> f64 {
    let mut base = if n < 0 { 1.0 / x } else { x };
    let mut e = n.unsigned_abs();
    let mut acc = 1.0f64;
    while e > 0 {
        if e & 1 == 1 {
            acc *= base;
        }
        base *= base;
        e >>= 1;
    }
    acc
}

/// Hyperbolic tangent.
#[inline]
pub fn tanh(x: f64) -> f64 {
    libm::tanh(x)
}

/// Hyperbolic cosine.
#[inline]
pub fn cosh(x: f64) -> f64 {
    libm::cosh(x)
}

/// Hyperbolic sine.
#[inline]
pub fn sinh(x: f64) -> f64 {
    libm::sinh(x)
}

/// sqrt(x² + y²) without undue overflow.
#[inline]
pub fn hypot(x: f64, y: f64) -> f64 {
    libm::hypot(x, y)
}

/// Absolute value.
#[inline]
pub fn abs(x: f64) -> f64 {
    libm::fabs(x)
}

/// Magnitude of `x` with the sign of `s`.
#[inline]
pub fn copysign(x: f64, s: f64) -> f64 {
    libm::copysign(x, s)
}

/// Largest integer ≤ x.
#[inline]
pub fn floor(x: f64) -> f64 {
    libm::floor(x)
}

/// Smallest integer ≥ x.
#[inline]
pub fn ceil(x: f64) -> f64 {
    libm::ceil(x)
}

/// Round half away from zero.
#[inline]
pub fn round(x: f64) -> f64 {
    libm::round(x)
}

/// Round toward zero.
#[inline]
pub fn trunc(x: f64) -> f64 {
    libm::trunc(x)
}

/// Floating remainder of x / y with the sign of x.
#[inline]
pub fn fmod(x: f64, y: f64) -> f64 {
    libm::fmod(x, y)
}

/// Minimum; returns the non-NaN operand if exactly one is NaN (IEEE minNum).
#[inline]
pub fn fmin(x: f64, y: f64) -> f64 {
    libm::fmin(x, y)
}

/// Maximum; returns the non-NaN operand if exactly one is NaN (IEEE maxNum).
#[inline]
pub fn fmax(x: f64, y: f64) -> f64 {
    libm::fmax(x, y)
}

/// Clamp `x` into `[lo, hi]` without panicking (unlike `f64::clamp`). NaN passes
/// through unchanged.
#[inline]
pub fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// Error function.
#[inline]
pub fn erf(x: f64) -> f64 {
    libm::erf(x)
}

/// Complementary error function 1 − erf(x), accurate for large x.
#[inline]
pub fn erfc(x: f64) -> f64 {
    libm::erfc(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pinned output bits of soft-float libm 0.2.16 (`force-soft-floats`). Any libm
    /// upgrade or feature-unification change that alters results is caught here, before
    /// it silently shifts the golden selftest hash or the seed-42 anchors.
    /// Tolerance (E): bit-exact.
    #[test]
    fn known_values() {
        // IEEE-754 correctly rounded: sqrt(2) = 1.4142135623730951.
        assert_eq!(
            sqrt(2.0).to_bits(),
            0x3FF6_A09E_667F_3BCD,
            "(E) bit-exact sqrt(2) f64"
        );
        assert_eq!(
            sqrtf(2.0).to_bits(),
            0x3FB5_04F3,
            "(E) bit-exact sqrt(2) f32"
        );
        // libm (musl-derived) exp(1) = 2.7182818284590455, 1 ulp above the correctly
        // rounded e (0x4005BF0A8B145769); pinned as libm's actual output.
        assert_eq!(
            exp(1.0).to_bits(),
            0x4005_BF0A_8B14_576A,
            "(E) pinned libm exp(1)"
        );
        // ln 10 = 2.302585092994046 (correctly rounded).
        assert_eq!(
            ln(10.0).to_bits(),
            0x4002_6BB1_BBB5_5516,
            "(E) pinned libm ln(10)"
        );
        // atan2(1, 1) = pi/4 (correctly rounded).
        assert_eq!(
            atan2(1.0, 1.0).to_bits(),
            0x3FE9_21FB_5444_2D18,
            "(E) pinned pi/4"
        );
        // sin(0.5) = 0.479425538604203.
        assert_eq!(
            sin(0.5).to_bits(),
            0x3FDE_AEE8_744B_05F0,
            "(E) pinned libm sin(0.5)"
        );
        // f32 controller path.
        assert_eq!(expf(1.0).to_bits(), 0x402D_F854, "(E) pinned libm expf(1)");
        assert_eq!(
            tanhf(0.5).to_bits(),
            0x3EEC_9A9E,
            "(E) pinned libm tanhf(0.5)"
        );
        assert_eq!(cos(0.0), 1.0);
        assert_eq!(sin(0.0), 0.0);
        assert_eq!(tanh(0.0), 0.0);
        assert_eq!(expf(0.0), 1.0);
        assert_eq!(lnf(1.0), 0.0);
        assert_eq!(pow(2.0, 10.0), 1024.0);
        assert_eq!(log10(1000.0), 3.0);
    }

    #[test]
    fn powi_matches_repeated_multiplication() {
        assert_eq!(powi(3.0, 4), 81.0);
        assert_eq!(powi(2.0, -2), 0.25);
        assert_eq!(powi(7.5, 0), 1.0);
        assert_eq!(powif(1.5, 3), 3.375);
        assert_eq!(powif(2.0, -3), 0.125);
    }

    #[test]
    fn clamp_never_panics_and_passes_nan() {
        assert_eq!(clampf(5.0, 0.0, 1.0), 1.0);
        assert_eq!(clampf(-5.0, 0.0, 1.0), 0.0);
        // Inverted bounds: no panic (f32::clamp would assert).
        assert_eq!(clampf(0.5, 1.0, 0.0), 1.0);
        assert!(clampf(f32::NAN, 0.0, 1.0).is_nan());
        assert_eq!(clamp(2.0, -1.0, 1.0), 1.0);
        assert!(clamp(f64::NAN, -1.0, 1.0).is_nan());
    }

    #[test]
    fn rounding_family() {
        assert_eq!(round(2.5), 3.0);
        assert_eq!(round(-2.5), -3.0);
        assert_eq!(trunc(-2.7), -2.0);
        assert_eq!(floor(-2.2), -3.0);
        assert_eq!(ceilf(2.2), 3.0);
        assert_eq!(truncf(-0.9), -0.0);
    }
}
