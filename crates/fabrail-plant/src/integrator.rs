//! RK4 over state structs with floor acceleration at t, t + h/2, t + h (PHYSICS §24). Owner: A6a.
//!
//! Classic fourth-order Runge–Kutta (Butcher tableau 0 | ½ ½ | ½ 0 ½ | 1 0 0 1 |
//! ⅙ ⅓ ⅓ ⅙). The derivative closure receives the stage time; the harness supplies the
//! floor acceleration at t, t + h/2 and t + h (stages 2 and 3 share t + h/2).
//!
//! [`rk4_step`] is complete; tiers implement [`OdeState`] for their state structs.

/// A state vector the integrator can combine.
pub trait OdeState: Clone {
    /// Returns `self + k·d`.
    fn add_scaled(&self, k: f64, d: &Self) -> Self;
}

impl OdeState for f64 {
    fn add_scaled(&self, k: f64, d: &Self) -> Self {
        self + k * d
    }
}

impl<const N: usize> OdeState for [f64; N] {
    fn add_scaled(&self, k: f64, d: &Self) -> Self {
        let mut out = *self;
        for (o, di) in out.iter_mut().zip(d.iter()) {
            *o += k * di;
        }
        out
    }
}

/// One RK4 step of ẏ = f(stage, t, y) from `t` to `t + h`. `stage` is 0..=3 (0 at t,
/// 1 and 2 at t + h/2, 3 at t + h) so the caller can index its floor samples
/// `[a(t), a(t+h/2), a(t+h)]` as `[0, 1, 1, 2][stage]`.
pub fn rk4_step<S: OdeState>(y: &S, t: f64, h: f64, mut f: impl FnMut(usize, f64, &S) -> S) -> S {
    let k1 = f(0, t, y);
    let k2 = f(1, t + 0.5 * h, &y.add_scaled(0.5 * h, &k1));
    let k3 = f(2, t + 0.5 * h, &y.add_scaled(0.5 * h, &k2));
    let k4 = f(3, t + h, &y.add_scaled(h, &k3));
    y.add_scaled(h / 6.0, &k1)
        .add_scaled(h / 3.0, &k2)
        .add_scaled(h / 3.0, &k3)
        .add_scaled(h / 6.0, &k4)
}

/// Floor-sample index for an RK4 stage (see [`rk4_step`]).
pub const STAGE_FLOOR_INDEX: [usize; 4] = [0, 1, 1, 2];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rk4_exponential_decay_is_fourth_order() {
        // ẏ = −y from 1 over 1 s: global error ∝ h⁴ (PHYSICS §24).
        let run = |n: u32| {
            let h = 1.0 / n as f64;
            let mut y = 1.0_f64;
            for k in 0..n {
                y = rk4_step(&y, k as f64 * h, h, |_, _, y| -y);
            }
            (y - (-1.0f64).exp()).abs()
        };
        let (e1, e2) = (run(10), run(20));
        let order = (e1 / e2).log2();
        assert!((order - 4.0).abs() < 0.1, "observed order {order}");
    }

    #[test]
    fn rk4_harmonic_oscillator_arrays() {
        // ẍ = −ω²x with ω·h = 0.2: one period keeps amplitude within (ωh)⁴-order error.
        let w = 10.0;
        let h = 0.02;
        let n = (2.0 * core::f64::consts::PI / w / h).round() as u32;
        let mut y = [1.0, 0.0];
        for k in 0..n {
            y = rk4_step(&y, k as f64 * h, h, |_, _, s| [s[1], -w * w * s[0]]);
        }
        let amp = (y[0] * y[0] + (y[1] / w) * (y[1] / w)).sqrt();
        assert!((amp - 1.0).abs() < 1e-3, "amplitude {amp}");
    }
}
