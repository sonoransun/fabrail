//! DRAWER compressible cavity with leak orifice and the DOOR smoke-control pressure force (PHYSICS §11). Owner: A6b.
//!
//! DRAWER: A_p 0.06 m², V 0.03 m³, A_leak 40 cm² (core design §C.1). Sign convention:
//! positive Δp pushes the body toward +x (cabinet pressure pushes outward, plan §2b
//! item 11).
//!
//! Cavity law (PHYSICS §11; physics perspective §1.9), adiabatic compression of the
//! enclosed air plus orifice outflow:
//!
//! dΔp/dt = (γ·(P₀ + Δp)/V(x))·(−A_p·ẋ − Q(Δp)),  V(x) = V₀ + A_p·x_open,
//! Q(Δp) = C_d·A_leak·√(2/ρ)·Δp/(Δp² + P_ε²)^¼,
//!
//! with the volume V₀ at the closed position (the drawer box displaces A_p per metre as
//! it closes). Q is the sharp-edged orifice law C_d·A·√(2|Δp|/ρ)·sgn Δp regularised
//! near Δp = 0 (P_ε = 1 Pa) so the rate is Lipschitz. A sealed cavity (A_leak = 0) is an
//! air spring of rate γ·P₀·A_p²/V₀ (17.0 kN/m for the DRAWER, physics perspective §1.9
//! \[calc\]); with the 40 cm² reveal the quasi-static piston force at 0.5 m/s is
//! ½ρ(v·A_p/(C_d·A_leak))²·A_p = 5.6 N (ENV-003; `derive::piston_force`).
//!
//! Stiffness (PHYSICS §24): near Δp = 0 the leak makes the state stiff,
//! |∂(dΔp/dt)/∂Δp| ≈ (γP₀/V)·C_d·A_leak·√(2/ρ)/√P_ε ≈ 1.5e4 s⁻¹ for 40 cm², i.e.
//! λh = 0.73 at 50 µs (stable) but 15 at 1 ms. Tiers read [`cavity_rate_jacobian`] and
//! use [`cavity_dp_quasi_static`] (the exact steady state) when λh > 1.

use fabrail_params::AirDesign;
use fabrail_params::derive::{ORIFICE_CD, RHO_AIR_KG_PER_M3};
use fabrail_types::fmath;

/// Standard atmospheric pressure, Pa (ISO 2533:1975 standard atmosphere; PHYSICS §11).
pub const P_ATM_PA: f64 = 101_325.0;

/// Ratio of specific heats of dry air (adiabatic cavity compression), dimensionless:
/// c_p/c_v = 7/5 for an ideal diatomic gas (1.40 for air at 20 °C, NIST Chemistry WebBook;
/// PHYSICS §11).
pub const GAMMA_AIR: f64 = 1.4;

/// Orifice-law regularisation pressure P_ε, Pa (physics perspective §1.9: P_ε = 1 Pa;
/// changes the 94 Pa ENV-003 operating point by 3e-5 relative; PHYSICS §11).
pub const P_EPS_PA: f64 = 1.0;

/// Smallest cavity volume as a fraction of V₀ (numerical floor for an overtravel into
/// the cabinet; never reached in the preset travel; PHYSICS §11), dimensionless.
const V_MIN_FRAC: f64 = 0.05;

/// Orifice conductance C_d·A_leak·√(2/ρ), m³/s per √Pa (PHYSICS §11).
fn orifice_k(leak_area_m2: f64) -> f64 {
    if !(leak_area_m2.is_finite() && leak_area_m2 > 0.0) {
        return 0.0;
    }
    ORIFICE_CD * leak_area_m2 * fmath::sqrt(2.0 / RHO_AIR_KG_PER_M3)
}

/// Orifice outflow Q(Δp) = C_d·A_leak·√(2/ρ)·Δp/(Δp² + P_ε²)^¼, m³/s (positive out of
/// the cavity; PHYSICS §11).
pub fn orifice_flow_m3ps(leak_area_m2: f64, dp_pa: f64) -> f64 {
    let k = orifice_k(leak_area_m2);
    if k == 0.0 || !dp_pa.is_finite() {
        return 0.0;
    }
    k * dp_pa / fmath::sqrt(fmath::sqrt(dp_pa * dp_pa + P_EPS_PA * P_EPS_PA))
}

/// Cavity volume at `x_open_m` from the closed stop: V₀ + A_p·x_open, floored at
/// 5 % of V₀, m³ (PHYSICS §11).
pub fn cavity_volume_m3(a: &AirDesign, x_open_m: f64) -> f64 {
    let v0 = a.cavity_volume_m3;
    (v0 + a.cavity_area_m2 * x_open_m).max(V_MIN_FRAC * v0)
}

/// True when the design has a cavity (positive piston area and volume).
pub fn has_cavity(a: &AirDesign) -> bool {
    a.cavity_area_m2 > 0.0 && a.cavity_volume_m3 > 0.0
}

/// Rate of the cavity overpressure dΔp/dt, Pa/s, for the front at `x_open_m` from the
/// closed stop moving at `v_mps`, with leak area `leak_area_m2` (adiabatic compression
/// plus orifice flow; PHYSICS §11). 0 when the design has no cavity.
pub fn cavity_dp_rate(
    a: &AirDesign,
    leak_area_m2: f64,
    dp_pa: f64,
    x_open_m: f64,
    v_mps: f64,
) -> f64 {
    if !has_cavity(a) || !dp_pa.is_finite() || !x_open_m.is_finite() || !v_mps.is_finite() {
        return 0.0;
    }
    let p_abs = (P_ATM_PA + dp_pa).max(0.0);
    let vol = cavity_volume_m3(a, x_open_m);
    GAMMA_AIR * p_abs / vol * (-a.cavity_area_m2 * v_mps - orifice_flow_m3ps(leak_area_m2, dp_pa))
}

/// ∂(dΔp/dt)/∂Δp at the operating point, s⁻¹ (≤ 0; the cavity state's eigenvalue for
/// the stability report, PHYSICS §24). Evaluated by a central difference of
/// [`cavity_dp_rate`] over ±1e-3·max(|Δp|, P_ε).
pub fn cavity_rate_jacobian(
    a: &AirDesign,
    leak_area_m2: f64,
    dp_pa: f64,
    x_open_m: f64,
    v_mps: f64,
) -> f64 {
    let e = 1.0e-3 * dp_pa.abs().max(P_EPS_PA);
    (cavity_dp_rate(a, leak_area_m2, dp_pa + e, x_open_m, v_mps)
        - cavity_dp_rate(a, leak_area_m2, dp_pa - e, x_open_m, v_mps))
        / (2.0 * e)
}

/// Quasi-static cavity overpressure at front speed `v_mps`: the exact root of
/// Q(Δp) = −A_p·v for the regularised orifice law, Pa (PHYSICS §11). With q = −A_p·v/K
/// (K the orifice conductance), Δp²/√(Δp² + P_ε²) = q⁴/q² gives
/// s = Δp² = (q⁴ + √(q⁸ + 4·q⁴·P_ε²))/2. 0 for a sealed cavity or no cavity (a sealed
/// cavity has no steady state under motion: use [`cavity_dp_rate`]).
pub fn cavity_dp_quasi_static(a: &AirDesign, leak_area_m2: f64, v_mps: f64) -> f64 {
    let k = orifice_k(leak_area_m2);
    if !has_cavity(a) || k == 0.0 || !v_mps.is_finite() {
        return 0.0;
    }
    let q = -a.cavity_area_m2 * v_mps / k;
    let q2 = q * q;
    let q4 = q2 * q2;
    let s = 0.5 * (q4 + fmath::sqrt(q4 * q4 + 4.0 * q4 * P_EPS_PA * P_EPS_PA));
    let dp = fmath::sqrt(s);
    if q < 0.0 { -dp } else { dp }
}

/// Sealed-cavity air-spring rate at the closed position, γ·P₀·A_p²/V₀, N/m (PHYSICS §11;
/// DRAWER 17.0 kN/m).
pub fn sealed_spring_rate_n_per_m(a: &AirDesign) -> f64 {
    if !has_cavity(a) {
        return 0.0;
    }
    GAMMA_AIR * P_ATM_PA * a.cavity_area_m2 * a.cavity_area_m2 / a.cavity_volume_m3
}

/// Force of the cavity overpressure on the DRAWER front, Δp·A_p, N (+x).
pub fn cavity_force_n(a: &AirDesign, dp_pa: f64) -> f64 {
    dp_pa * a.cavity_area_m2
}

/// Force of a smoke-control pressure difference on the DOOR leaf, Δp·A_leaf, N (+x).
pub fn smoke_force_n(a: &AirDesign, dp_pa: f64) -> f64 {
    dp_pa * a.leaf_area_m2
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integrator::rk4_step;
    use fabrail_params::derive::piston_force;
    use fabrail_params::presets;

    fn drawer_air() -> AirDesign {
        presets::drawer().design.air
    }

    #[test]
    fn pressure_forces() {
        let a = AirDesign {
            cavity_area_m2: 0.06,
            leaf_area_m2: 2.0,
            ..AirDesign::default()
        };
        assert!((cavity_force_n(&a, 25.0) - 1.5).abs() < 1e-12);
        assert!((smoke_force_n(&a, -25.0) + 50.0).abs() < 1e-12);
    }

    // req: FAB-ENV-003
    #[test]
    fn sealed_spring_rate() {
        let a = drawer_air();
        // (1) Sealed cavity: the linearised rate at Δp = 0 is the air spring
        // k = γ·P₀·A_p²/V₀ = 1.4·101325·0.06²/0.03 = 17.02 kN/m ((E) rel 1e-12 against
        // the rate law; physics perspective §1.9 "17 kN/m [calc]").
        let k = sealed_spring_rate_n_per_m(&a);
        let from_rate = -cavity_dp_rate(&a, 0.0, 0.0, 0.0, 1.0) * a.cavity_area_m2;
        assert!(
            (from_rate - k).abs() <= 1e-12 * k,
            "rate-law spring {from_rate} vs closed form {k} ((E) rel 1e-12, PHYSICS §11)"
        );
        assert!(
            (k - 17_023.0).abs() < 1.0,
            "k {k} N/m vs 17.0 kN/m (physics perspective §1.9)"
        );
        // A 1 mm sealed push-in integrated by RK4 reproduces k·δ to the adiabatic
        // nonlinearity (≈ γ·A_p·δ/V₀ ≈ 3e-3 relative; (I) 5e-3).
        let (v, h) = (-0.01, 5.0e-5);
        let mut dp = 0.0;
        let mut x = 0.0;
        for n in 0..2000u32 {
            let t = f64::from(n) * h;
            let x0 = x;
            dp = rk4_step(&dp, t, h, |_, ts, d| {
                cavity_dp_rate(&a, 0.0, *d, x0 + v * (ts - t), v)
            });
            x += v * h;
        }
        let f = cavity_force_n(&a, dp);
        assert!(
            (f - k * 1.0e-3).abs() < 5e-3 * k * 1.0e-3,
            "sealed 1 mm: F {f} vs k·δ"
        );
        // (2) With the 40 cm² reveal at 0.5 m/s closing the piston force is 5.6 N ±5 %
        // (FAB-ENV-003, core research C15) and equals derive::piston_force (the
        // unregularised orifice law) within 1e-4 relative.
        let dp_qs = cavity_dp_quasi_static(&a, a.leak_area_m2, -0.5);
        let f_qs = cavity_force_n(&a, dp_qs);
        assert!(
            (f_qs - 5.6).abs() <= 0.05 * 5.6,
            "piston force {f_qs} N vs 5.6 N ±5 % (ENV-003)"
        );
        let f_ref = piston_force(a.cavity_area_m2, 0.5, a.leak_area_m2);
        assert!(
            (f_qs - f_ref).abs() <= 1e-4 * f_ref,
            "{f_qs} vs derive::piston_force {f_ref}"
        );
        // The dynamic cavity (from rest, 0.5 m/s closing for 0.2 s) settles on it: the
        // quasi-static value is the exact fixed point of the rate law.
        let mut dp = 0.0;
        let mut x = 0.3;
        for n in 0..4000u32 {
            let t = f64::from(n) * h;
            let x0 = x;
            dp = rk4_step(&dp, t, h, |_, ts, d| {
                cavity_dp_rate(&a, a.leak_area_m2, *d, x0 - 0.5 * (ts - t), -0.5)
            });
            x -= 0.5 * h;
        }
        assert!(
            (dp - dp_qs).abs() < 1e-3 * dp_qs,
            "dynamic Δp {dp} vs quasi-static {dp_qs}"
        );
        assert!(
            cavity_dp_rate(&a, a.leak_area_m2, dp_qs, 0.1, -0.5).abs()
                < 1e-6 * GAMMA_AIR * P_ATM_PA
        );
    }

    #[test]
    fn signs_regularisation_and_stiffness() {
        let a = drawer_air();
        // Closing (v < 0) compresses: Δp rises and pushes the front outward (+x).
        assert!(cavity_dp_rate(&a, a.leak_area_m2, 0.0, 0.1, -0.2) > 0.0);
        assert!(
            cavity_dp_quasi_static(&a, a.leak_area_m2, 0.3) < 0.0,
            "opening sucks"
        );
        // Orifice law: odd, Lipschitz at 0, √Δp far from 0.
        assert_eq!(orifice_flow_m3ps(a.leak_area_m2, 0.0), 0.0);
        let q = orifice_flow_m3ps(a.leak_area_m2, 100.0);
        let q_ideal = ORIFICE_CD * a.leak_area_m2 * (2.0 * 100.0 / RHO_AIR_KG_PER_M3).sqrt();
        assert!((q - q_ideal).abs() < 1e-4 * q_ideal);
        assert_eq!(orifice_flow_m3ps(a.leak_area_m2, -100.0), -q);
        // The leak eigenvalue at Δp = 0: (γP₀/V₀)·C_d·A·√(2/ρ)/√P_ε ≈ 1.46e4 s⁻¹
        // (1 %, PHYSICS §24).
        let lam = cavity_rate_jacobian(&a, a.leak_area_m2, 0.0, 0.0, 0.0);
        let want = -GAMMA_AIR * P_ATM_PA / a.cavity_volume_m3 * orifice_k(a.leak_area_m2)
            / P_EPS_PA.sqrt();
        assert!((lam - want).abs() < 0.01 * want.abs(), "λ {lam} vs {want}");
        // No cavity (DOOR): inert.
        let door = presets::door().design.air;
        assert!(!has_cavity(&door));
        assert_eq!(cavity_dp_rate(&door, 0.0, 10.0, 0.0, -1.0), 0.0);
        assert_eq!(cavity_dp_quasi_static(&door, 1e-3, -1.0), 0.0);
        // Opening grows the volume.
        assert!((cavity_volume_m3(&a, 0.3) - (0.03 + 0.06 * 0.3)).abs() < 1e-15);
    }
}
