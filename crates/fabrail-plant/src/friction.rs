//! Elasto-plastic bristle friction (Dupont 2002) for every tangential contact (PHYSICS §8). Owner: A6b.
//!
//! Dupont, Hayward, Armstrong & Altpeter, "Single state elastoplastic friction models",
//! IEEE TAC 47(5), 2002: a bristle state z with a purely elastic presliding regime below
//! the breakaway displacement z_ba, so there is no drift (ratchet) under vibration;
//! tanh Coulomb creeps and LuGre ratchets (core design §A.1). All coefficients are
//! normalised by the normal force: F_t = F_n·(σ0·z + σ1·ż + σ2·v).
//!
//! State law (PHYSICS §8; physics perspective §1.6):
//!
//! * μ(v) = μ_c + (μ_s − μ_c)·exp(−(v/v_s)²) (Stribeck, optional: μ_s = μ_c disables it);
//! * σ0 = μ_c/z_ss, the steady deflection z_ss(v) = μ(v)/σ0 and z_ba = r_ba·z_ss(v);
//! * ż = v·(1 − α(z, v)·z/z_ss(v)) with α = 0 while |z| ≤ z_ba or sgn z ≠ sgn v
//!   (purely elastic), α = 1 for |z| ≥ z_ss, and in between the blend
//!   α = z_ss·(|z| − z_ba)/(|z|·(z_ss − z_ba)), which makes ż linear in z across the
//!   transition. Dupont's conditions (α continuous in z, 0 < α < 1 on the transition)
//!   hold; the linear-rate blend is a design choice because it admits the exact per-step
//!   update [`bristle_advance`].
//!
//! Numerics (PHYSICS §24). In the elastic regime ż = v and nothing is stiff. On the
//! transition the relaxation rate is |v|/(z_ss − z_ba) ([`relax_rate_per_s`]): at
//! z_ss = 10 µm, r_ba = 0.7 and 0.265 m/s that is 8.8e4 s⁻¹, so an RK4 step of 50 µs
//! (λh = 4.4) exceeds RK4's real-axis limit 2.78. Tiers therefore either keep
//! `relax_rate_per_s·h` ≤ 2 when they integrate z inside RK4, or hold z fixed through
//! the RK4 body step (the force uses [`bristle`] at the frozen z and the stage velocity)
//! and then advance it exactly with [`bristle_advance`] over the step's slide distance.
//! The split update keeps stick exact (z follows the displacement one-for-one below
//! z_ba), which is what makes the no-ratchet property survive discretisation.

use fabrail_params::derive::G0_MPS2;
use fabrail_types::fmath;

/// Presliding displacement at μ_c on the 50 µs tiers (Carriage6, Heave1), m (core design
/// §C.5: 10 µm; PHYSICS §8).
pub const Z_SS_CARRIAGE6_M: f64 = 10.0e-6;

/// Presliding displacement at μ_c on the 1 ms Travel1d tier, m (core design §C.5: 50 µm;
/// the larger value keeps the bristle stick mode resolved at 1 ms; PHYSICS §8, §24).
pub const Z_SS_TRAVEL1D_M: f64 = 50.0e-6;

/// Breakaway ratio z_ba/z_ss: purely elastic below 0.7·z_ss (physics perspective §1.6,
/// "α = 0 for |z| < 0.7 z_ss"; PHYSICS §8).
pub const Z_BA_RATIO: f64 = 0.7;

/// Stribeck velocity, m/s (physics perspective §1.6 uses v_s = 1 mm/s as the reference
/// regularisation speed; only active when μ_s > μ_c; PHYSICS §8).
pub const V_STRIBECK_MPS: f64 = 1.0e-3;

/// Damping ratio of the bristle stick mode: critical (ζ = 1), the usual LuGre choice
/// σ1 = 2√(σ0·m) (Canudas de Wit et al., IEEE TAC 40(3), 1995), normalised by F_n = m·g₀
/// (PHYSICS §8).
pub const ZETA_BRISTLE: f64 = 1.0;

/// Normalised elasto-plastic parameters.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BristleParams {
    /// Coulomb (kinetic) coefficient μ_c (rollers 0.003, plan §2b item 19; UHMW 0.15).
    pub mu_c: f64,
    /// Static coefficient μ_s ≥ μ_c.
    pub mu_s: f64,
    /// Stribeck velocity, m/s.
    pub v_s_mps: f64,
    /// Steady-state presliding displacement at μ_c (10 µm Carriage6, 50 µm Travel1d;
    /// core design §C.5), m. σ0 = μ_c / z_ss.
    pub z_ss_m: f64,
    /// Breakaway ratio z_ba / z_ss in (0, 1).
    pub z_ba_ratio: f64,
    /// Bristle micro-damping σ1 (normalised), s/m.
    pub sigma1_s_per_m: f64,
    /// Viscous term σ2 (normalised), s/m.
    pub sigma2_s_per_m: f64,
}

impl BristleParams {
    /// Parameters for Coulomb μ_c, static μ_s (clamped to ≥ μ_c) and presliding
    /// displacement z_ss, with r_ba = [`Z_BA_RATIO`], v_s = [`V_STRIBECK_MPS`], no viscous
    /// term (viscous drag lives in the dissipation ledger, PHYSICS §9) and a critically
    /// damped stick mode for a weight-bearing contact: σ1 = 2ζ·√(σ0/g₀) (from
    /// c = σ1·F_n = 2ζ√(σ0·F_n·m) with F_n = m·g₀; PHYSICS §8).
    pub fn new(mu_c: f64, mu_s: f64, z_ss_m: f64) -> BristleParams {
        let mu_c = if mu_c.is_finite() { mu_c.max(0.0) } else { 0.0 };
        let mu_s = if mu_s.is_finite() {
            mu_s.max(mu_c)
        } else {
            mu_c
        };
        let z_ss = if z_ss_m.is_finite() {
            z_ss_m.max(0.0)
        } else {
            0.0
        };
        let sigma1 = if mu_c > 0.0 && z_ss > 0.0 {
            2.0 * ZETA_BRISTLE * fmath::sqrt(mu_c / z_ss / G0_MPS2)
        } else {
            0.0
        };
        BristleParams {
            mu_c,
            mu_s,
            v_s_mps: V_STRIBECK_MPS,
            z_ss_m: z_ss,
            z_ba_ratio: Z_BA_RATIO,
            sigma1_s_per_m: sigma1,
            sigma2_s_per_m: 0.0,
        }
    }

    /// True when the element carries no friction (μ_c ≤ 0, z_ss ≤ 0 or a non-finite
    /// coefficient): [`bristle`] returns zero force and a frozen state.
    pub fn is_inert(&self) -> bool {
        !(self.mu_c.is_finite() && self.z_ss_m.is_finite() && self.mu_c > 0.0 && self.z_ss_m > 0.0)
    }

    /// Normalised bristle stiffness σ0 = μ_c/z_ss, 1/m (0 when inert).
    pub fn sigma0_per_m(&self) -> f64 {
        if self.is_inert() {
            0.0
        } else {
            self.mu_c / self.z_ss_m
        }
    }

    /// Friction coefficient at sliding speed `v_mps`: μ_c + (μ_s − μ_c)·exp(−(v/v_s)²)
    /// (Stribeck; PHYSICS §8).
    pub fn mu_at(&self, v_mps: f64) -> f64 {
        let dmu = self.mu_s - self.mu_c;
        if !(dmu > 0.0 && self.v_s_mps > 0.0 && v_mps.is_finite()) {
            return self.mu_c;
        }
        let r = v_mps / self.v_s_mps;
        self.mu_c + dmu * fmath::exp(-r * r)
    }

    /// Steady presliding displacement magnitude z_ss(v) = μ(v)/σ0, m.
    pub fn z_ss_at(&self, v_mps: f64) -> f64 {
        if self.is_inert() {
            return 0.0;
        }
        self.z_ss_m * self.mu_at(v_mps) / self.mu_c
    }

    /// Breakaway ratio clamped to [0, 1].
    fn r_ba(&self) -> f64 {
        if self.z_ba_ratio.is_finite() {
            self.z_ba_ratio.clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    /// Largest tangential force carried purely elastically from rest, per unit normal
    /// force: F_ba/F_n = σ0·z_ba(0) = r_ba·μ_s. Below it the contact never slips (the
    /// no-ratchet bound).
    pub fn breakaway_ratio(&self) -> f64 {
        if self.is_inert() {
            return 0.0;
        }
        self.r_ba() * self.mu_at(0.0)
    }
}

/// Bristle state rate and tangential force for one step evaluation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BristleRates {
    /// dz/dt, m/s.
    pub z_dot_mps: f64,
    /// Tangential force on the body, N (opposes sliding; signed along the contact axis).
    pub f_t_n: f64,
}

/// dζ/dσ for ζ = z·sgn(v), the deflection measured in the sliding direction, per unit
/// slide distance σ (PHYSICS §8): 1 while elastic, (z_ss − ζ)/(z_ss − z_ba) on the
/// transition, 1 − ζ/z_ss beyond z_ss.
fn slide_rate(zeta: f64, z_ss: f64, z_ba: f64) -> f64 {
    if zeta <= z_ba {
        1.0
    } else if zeta < z_ss {
        (z_ss - zeta) / (z_ss - z_ba)
    } else {
        1.0 - zeta / z_ss
    }
}

/// Bristle state rate and friction force at bristle deflection `z_m`, sliding velocity
/// `v_mps` and normal load `f_n_n` ≥ 0 (PHYSICS §8): ż = v·(1 − α·z/z_ss(v)) and
/// F_t = −F_n·(σ0·z + σ1·ż + σ2·v). A non-positive or non-finite normal load gives zero
/// force (the state rate is still returned; callers reset z to 0 when the contact
/// opens). Non-finite state or velocity gives zero rates.
pub fn bristle(p: &BristleParams, z_m: f64, v_mps: f64, f_n_n: f64) -> BristleRates {
    if p.is_inert() || !(z_m.is_finite() && v_mps.is_finite()) {
        return BristleRates::default();
    }
    let z_dot = if v_mps == 0.0 {
        0.0
    } else {
        let z_ss = p.z_ss_at(v_mps);
        let z_ba = p.r_ba() * z_ss;
        let s = if v_mps > 0.0 { 1.0 } else { -1.0 };
        v_mps * slide_rate(z_m * s, z_ss, z_ba)
    };
    let f_n = if f_n_n.is_finite() {
        f_n_n.max(0.0)
    } else {
        0.0
    };
    let f_t = -f_n * (p.sigma0_per_m() * z_m + p.sigma1_s_per_m * z_dot + p.sigma2_s_per_m * v_mps);
    BristleRates {
        z_dot_mps: z_dot,
        f_t_n: f_t,
    }
}

/// Exact bristle update over a step with slide displacement `dx_m` at (constant) speed
/// `v_mps` (used only for the Stribeck z_ss(v)); returns the new deflection, m
/// (PHYSICS §8, §24).
///
/// With σ the slide distance, the rate law is autonomous in σ and piecewise linear, so
/// each regime has a closed form: ζ += σ while elastic (up to z_ba), then
/// ζ = z_ss − (z_ss − ζ₀)·e^(−σ/(z_ss − z_ba)) on the transition (the fixed point z_ss
/// is never crossed), or ζ = z_ss + (ζ₀ − z_ss)·e^(−σ/z_ss) above it. Unconditionally
/// stable for any step, and stick is exact: a displacement that stays below z_ba
/// returns to the same z when reversed.
pub fn bristle_advance(p: &BristleParams, z_m: f64, dx_m: f64, v_mps: f64) -> f64 {
    if !z_m.is_finite() {
        return 0.0;
    }
    if p.is_inert() || !dx_m.is_finite() || dx_m == 0.0 {
        return z_m;
    }
    let s = if dx_m > 0.0 { 1.0 } else { -1.0 };
    let v = if v_mps.is_finite() { v_mps } else { 0.0 };
    let z_ss = p.z_ss_at(v);
    let z_ba = p.r_ba() * z_ss;
    let mut zeta = z_m * s;
    let mut sigma = dx_m.abs();
    // Elastic presliding (includes deflection against the motion, α = 0).
    if zeta < z_ba {
        let room = z_ba - zeta;
        if sigma <= room {
            return s * (zeta + sigma);
        }
        zeta = z_ba;
        sigma -= room;
    }
    // Transition: exponential approach to z_ss with length z_ss − z_ba.
    if zeta < z_ss {
        let w = z_ss - z_ba;
        zeta = z_ss - (z_ss - zeta) * fmath::exp(-sigma / w);
        return s * zeta;
    }
    // Beyond z_ss (e.g. after a Stribeck drop): relaxes back with length z_ss.
    s * (z_ss + (zeta - z_ss) * fmath::exp(-sigma / z_ss))
}

/// Fastest relaxation rate of the bristle state at sliding speed `v_mps`,
/// |∂ż/∂z| = |v|/(z_ss(v)·(1 − r_ba)), s⁻¹ (the transition slope; PHYSICS §24). A tier
/// that integrates z inside RK4 needs this times h ≤ 2 (RK4's real-axis limit is 2.78);
/// otherwise it uses [`bristle_advance`]. The stick mode itself is a spring of
/// ω = √(σ0·g₀) for a weight-bearing contact and is not stiff.
pub fn relax_rate_per_s(p: &BristleParams, v_mps: f64) -> f64 {
    if p.is_inert() || !v_mps.is_finite() {
        return 0.0;
    }
    let w = p.z_ss_at(v_mps) * (1.0 - p.r_ba());
    if w <= 0.0 {
        return f64::INFINITY;
    }
    v_mps.abs() / w
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integrator::rk4_step;

    fn uhmw() -> BristleParams {
        BristleParams::new(0.15, 0.15, Z_SS_CARRIAGE6_M)
    }

    #[test]
    fn constructor_derives_sigma0_and_critical_sigma1() {
        let p = uhmw();
        assert!((p.sigma0_per_m() - 0.15 / 10.0e-6).abs() < 1e-6);
        // σ1 = 2√(σ0/g₀): (E) rel 1e-12, PHYSICS §8.
        let want = 2.0 * (0.15 / 10.0e-6 / G0_MPS2).sqrt();
        assert!(
            (p.sigma1_s_per_m - want).abs() <= 1e-12 * want,
            "σ1 {} vs {want} ((E) rel 1e-12, PHYSICS §8)",
            p.sigma1_s_per_m
        );
        assert!((p.breakaway_ratio() - 0.7 * 0.15).abs() < 1e-15);
        assert!(BristleParams::new(0.0, 0.0, 1e-5).is_inert());
        assert!(BristleParams::new(0.1, 0.1, 0.0).is_inert());
        assert_eq!(
            BristleParams::new(0.1, 0.05, 1e-5).mu_s,
            0.1,
            "μ_s is clamped to ≥ μ_c"
        );
    }

    #[test]
    fn elastic_regime_is_a_pure_spring() {
        let p = uhmw();
        // Below z_ba: ż = v exactly and the force is −F_n·(σ0 z + σ1 v).
        let r = bristle(&p, 3.0e-6, 0.01, 100.0);
        assert_eq!(r.z_dot_mps, 0.01);
        let want = -100.0 * (p.sigma0_per_m() * 3.0e-6 + p.sigma1_s_per_m * 0.01);
        assert!((r.f_t_n - want).abs() < 1e-9);
        // Moving back against the deflection is elastic too (α = 0 when sgn z ≠ sgn v).
        let r = bristle(&p, 9.9e-6, -0.2, 100.0);
        assert_eq!(r.z_dot_mps, -0.2);
        // Stick is exact under the split update: out and back returns to the start.
        let z1 = bristle_advance(&p, 1.0e-6, 4.0e-6, 0.01);
        let z2 = bristle_advance(&p, z1, -4.0e-6, -0.01);
        assert!((z1 - 5.0e-6).abs() < 1e-18 && (z2 - 1.0e-6).abs() < 1e-18);
    }

    #[test]
    fn advance_matches_fine_rk4_of_the_rate_law() {
        // The closed-form update equals a converged RK4 integration of ż = v·g(z)
        // (tolerance (I) 1e-9 m, PHYSICS §24) across every regime, both directions and
        // with Stribeck active.
        let mut p = BristleParams::new(0.15, 0.25, Z_SS_CARRIAGE6_M);
        p.v_s_mps = 0.02;
        let cases = [
            (0.0, 30.0e-6, 0.01),
            (-12.0e-6, 25.0e-6, 0.05),
            (8.0e-6, 3.0e-6, 0.002),
            (16.0e-6, 40.0e-6, 0.2),
            (5.0e-6, -60.0e-6, -0.03),
        ];
        for (z0, dx, v) in cases {
            let exact = bristle_advance(&p, z0, dx, v);
            let n = 20_000u32;
            let dt = dx / v / f64::from(n);
            let mut z = z0;
            for k in 0..n {
                z = rk4_step(&z, f64::from(k) * dt, dt, |_, _, zz| {
                    bristle(&p, *zz, v, 1.0).z_dot_mps
                });
            }
            assert!(
                (z - exact).abs() < 1e-9,
                "z0 {z0} dx {dx}: rk4 {z} vs exact {exact} ((I) 1e-9 m, PHYSICS §24)"
            );
        }
    }

    #[test]
    fn steady_sliding_is_coulomb_and_stribeck_raises_breakaway() {
        let mut p = BristleParams::new(0.1, 0.13, Z_SS_CARRIAGE6_M);
        p.v_s_mps = 0.01;
        // Far above v_s: z → z_ss and F → −μ_c·F_n.
        let z = bristle_advance(&p, 0.0, 1.0e-2, 0.5);
        assert!((z - p.z_ss_at(0.5)).abs() < 1e-12);
        let f = bristle(&p, z, 0.5, 200.0).f_t_n;
        assert!((f + 0.1 * 200.0).abs() < 1e-6 * 20.0, "F {f} vs −μ_c·F_n");
        // At rest the elastic range grows with μ_s (breakaway r_ba·μ_s).
        assert!((p.breakaway_ratio() - 0.7 * 0.13).abs() < 1e-12);
        assert!(p.z_ss_at(0.0) > p.z_ss_at(0.1));
    }

    #[test]
    fn relax_rate_and_degenerate_inputs() {
        let p = uhmw();
        // |v|/(z_ss·(1 − r_ba)) = 0.3/(10 µm·0.3) = 1e5 s⁻¹.
        assert!((relax_rate_per_s(&p, -0.3) - 1.0e5).abs() < 1e-6);
        let inert = BristleParams::default();
        assert_eq!(bristle(&inert, 1e-6, 1.0, 10.0), BristleRates::default());
        assert_eq!(bristle(&p, f64::NAN, 1.0, 10.0), BristleRates::default());
        assert_eq!(
            bristle(&p, 1e-6, 1.0, -5.0).f_t_n,
            0.0,
            "no force without load"
        );
        assert_eq!(bristle_advance(&p, f64::NAN, 1e-6, 0.1), 0.0);
        assert_eq!(bristle_advance(&inert, 2e-6, 1e-3, 0.1), 2e-6);
    }
}
