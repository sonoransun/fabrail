//! Passive lateral guidance: reluctance y/yaw stiffness, sway damping, UHMW pads (PHYSICS §6). Owner: A6b.
//!
//! The reluctance restoring stiffness itself comes from the magnet model (A6a,
//! `magnet`); this module adds the mechanical guidance: the bottom guide fin (±2 mm
//! clearance on DOOR/WALL/HEAVY), the UHMW side pads (μ 0.15) and the sway damping
//! c_sway (plan §2b item 16).
//!
//! Laws (PHYSICS §6; physics perspective §1.3):
//!
//! * sway damping F_y = −c_sway·ẏ (eddy fin / guide damping; the presets size c_sway for
//!   sway ζ ≈ 0.08 at the 0.49 Hz top-hung pendulum, LEV-011 needs ζ ≥ 0.05);
//! * the fin (or, without a fin, the side pads) is a pair of unilateral UHMW stops at
//!   ±clearance: Hunt–Crossley F_n = k·δ·(1 + hc·δ̇) with δ = |y| − clearance
//!   (`contact::K_UHMW_N_PER_M`, `contact::HC_UHMW_S_PER_M`); it only pushes back
//!   toward the centreline, and is contactless in nominal operation;
//! * friction along x at the stop, −μ·F_n·tanh(v_x/v_reg) (kinetic estimate; tiers
//!   carrying a fin bristle state use `friction::bristle` with
//!   `contact::contact_params(p, GuideFin)` instead);
//! * small-offset reluctance centring per element k_y = F_z/w (the co-energy model's
//!   first-order lateral stiffness, F_y ≈ −F_z·(g/w)·tanh(y/g); DOOR 12.3 kN/m per
//!   magnet) and yaw stiffness Σ k_y·(x_k − x_cg)² from the station spacing.

use fabrail_params::derive::pm_force;
use fabrail_params::{GuidanceDesign, SuspensionDesign};
use fabrail_types::{MAX_MAGNETS, fmath};

use crate::contact::{HC_UHMW_S_PER_M, K_UHMW_N_PER_M, hunt_crossley_n};

/// Regularisation speed of the stateless kinetic friction estimate, m/s (design choice:
/// 1 mm/s, the physics perspective's reference; tanh(3) = 0.995, so the drag is μ·N
/// within 0.5 % above 3 mm/s, inside LEV-011's ±5 %; PHYSICS §6).
pub const V_REG_MPS: f64 = 1.0e-3;

/// Guidance loads on the body.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GuideForces {
    /// Lateral force (y), N.
    pub fy_n: f64,
    /// Yaw moment (about z), N·m.
    pub mz_n_m: f64,
    /// Friction along x from fin/pad contact, N (opposes travel).
    pub fx_n: f64,
    /// Fin/pad normal force magnitude (for KPIs), N.
    pub normal_n: f64,
}

/// Fin, pad and sway-damping loads at lateral offset `y_m`, lateral rate `y_dot_mps`
/// and travel speed `v_x_mps` (PHYSICS §6), all at one guide point: the fin when
/// `fin_present`, else the side pads. `mz_n_m` is the moment of the x friction about
/// the guide centreline (−y·F_x); the caller adds r × F for the guide point's position.
pub fn guide(g: &GuidanceDesign, y_m: f64, y_dot_mps: f64, v_x_mps: f64) -> GuideForces {
    if !(y_m.is_finite() && y_dot_mps.is_finite() && v_x_mps.is_finite()) {
        return GuideForces::default();
    }
    let clearance = if g.fin_present {
        g.fin_clearance_m
    } else {
        g.pad_clearance_m
    };
    let n = lateral_stop_n(clearance, y_m, y_dot_mps);
    let mut out = GuideForces {
        fy_n: sway_damping_n(g, y_dot_mps),
        ..GuideForces::default()
    };
    if n > 0.0 {
        let side = if y_m > 0.0 { 1.0 } else { -1.0 };
        out.fy_n -= side * n;
        out.normal_n = n;
        out.fx_n = kinetic_friction_n(g.pad_mu, n, v_x_mps);
        out.mz_n_m = -y_m * out.fx_n;
    }
    out
}

/// Sway damping F_y = −c_sway·ẏ, N (plan §2b item 16; PHYSICS §6).
pub fn sway_damping_n(g: &GuidanceDesign, y_dot_mps: f64) -> f64 {
    -g.c_sway_n_s_per_m.max(0.0) * y_dot_mps
}

/// Normal force magnitude of a pair of unilateral UHMW stops at ±`clearance_m` for
/// lateral offset `y_m` and rate `y_dot_mps` (Hunt–Crossley with δ = |y| − clearance,
/// δ̇ = sgn(y)·ẏ), N ≥ 0 (PHYSICS §6, §8).
pub fn lateral_stop_n(clearance_m: f64, y_m: f64, y_dot_mps: f64) -> f64 {
    let c = if clearance_m.is_finite() {
        clearance_m.max(0.0)
    } else {
        0.0
    };
    let side = if y_m >= 0.0 { 1.0 } else { -1.0 };
    hunt_crossley_n(
        K_UHMW_N_PER_M,
        HC_UHMW_S_PER_M,
        y_m.abs() - c,
        side * y_dot_mps,
    )
}

/// Stateless kinetic friction along x at a lateral stop: −μ·N·tanh(v/v_reg), N
/// (PHYSICS §6).
pub fn kinetic_friction_n(mu: f64, normal_n: f64, v_x_mps: f64) -> f64 {
    -mu.max(0.0) * normal_n.max(0.0) * fmath::tanh(v_x_mps / V_REG_MPS)
}

/// Small-offset reluctance centring stiffness of one PM element, k_y = F_z(g, 0, i)/w,
/// N/m (first order of F_y ≈ −F_z·(g/w)·tanh(y/g); physics perspective §1.3, DOOR 12.3
/// kN/m per magnet). Design closed form (σ_leak = 0, no saturation).
pub fn reluctance_ky_n_per_m(s: &SuspensionDesign, g_m: f64, i_a: f64) -> f64 {
    if s.pole_w_m <= 0.0 {
        return 0.0;
    }
    pm_force(s, g_m, 0.0, i_a) / s.pole_w_m
}

/// Passive yaw stiffness Σ_k k_y·(x_k − x_cg)² over the active PM elements at gap `g_m`
/// and current `i_a`, N·m/rad (physics perspective §1.3).
pub fn yaw_stiffness_n_m_per_rad(s: &SuspensionDesign, cg_x_m: f64, g_m: f64, i_a: f64) -> f64 {
    let ky = reluctance_ky_n_per_m(s, g_m, i_a);
    let n = usize::from(s.n_magnets).min(MAX_MAGNETS);
    s.stations
        .iter()
        .take(n)
        .filter(|st| st.has_pm)
        .map(|st| ky * (st.x_m - cg_x_m) * (st.x_m - cg_x_m))
        .sum()
}

/// Sway damping ratio of a top-hung leaf treated as a simple pendulum of length L at the
/// CoM, ζ = c_sway/(2·m·√(g₀/L)) (the preset sizing; LEV-011 ζ ≥ 0.05).
pub fn sway_zeta(g: &GuidanceDesign, m_kg: f64, l_m: f64) -> f64 {
    if !(m_kg > 0.0 && l_m > 0.0) {
        return 0.0;
    }
    let w = fmath::sqrt(fabrail_params::derive::G0_MPS2 / l_m);
    g.c_sway_n_s_per_m / (2.0 * m_kg * w)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    fn door_guide() -> GuidanceDesign {
        presets::door().design.guidance
    }

    #[test]
    fn contactless_inside_the_clearance() {
        // Nominal operation never touches the fin (physics perspective §1.3).
        let g = door_guide();
        for y in [-1.9e-3, -1.0e-3, 0.0, 1.0e-3, 1.99e-3] {
            let f = guide(&g, y, 0.0, 0.2);
            assert_eq!((f.normal_n, f.fx_n, f.mz_n_m), (0.0, 0.0, 0.0), "y {y}");
        }
        // Sway damping is always there.
        let f = guide(&g, 0.0, 0.01, 0.0);
        assert!((f.fy_n + 50.0 * 0.01).abs() < 1e-12);
    }

    #[test]
    fn fin_pushes_back_and_drags_mu_n() {
        // A 20 N lateral push: the fin engages at 2 mm plus 20/k (4 µm), pushes back
        // toward the centreline and drags μ·N along x within 5 % while sliding (LEV-011).
        let g = door_guide();
        let y = 2.0e-3 + 20.0 / K_UHMW_N_PER_M;
        let f = guide(&g, y, 0.0, 0.1);
        assert!((f.normal_n - 20.0).abs() < 1e-9, "N {}", f.normal_n);
        assert!(f.fy_n < 0.0, "pushes toward the centreline");
        assert!(
            (f.fx_n + 0.15 * 20.0).abs() <= 0.05 * 0.15 * 20.0,
            "x drag {} vs μN 3 N (±5 %, LEV-011)",
            f.fx_n
        );
        assert!((f.mz_n_m + y * f.fx_n).abs() < 1e-15);
        let f = guide(&g, -y, 0.0, -0.1);
        assert!(f.fy_n > 0.0 && f.fx_n > 0.0, "mirror side");
        // Rebounding away from the fin never pulls.
        assert_eq!(lateral_stop_n(2.0e-3, 2.001e-3, -5.0), 0.0);
        assert_eq!(guide(&g, f64::NAN, 0.0, 0.0), GuideForces::default());
    }

    #[test]
    fn drawer_uses_side_pads() {
        let g = presets::drawer().design.guidance;
        assert!(!g.fin_present);
        assert_eq!(guide(&g, 0.9e-3, 0.0, 0.0).normal_n, 0.0);
        assert!(guide(&g, 1.1e-3, 0.0, 0.0).normal_n > 0.0, "pads at ±1 mm");
    }

    #[test]
    fn reluctance_stiffness_and_sway_anchors() {
        // DOOR k_y = F*/w = 245.2 N / 20 mm = 12.3 kN/m per magnet (physics perspective
        // §1.3 [calc]; tolerance 1 %).
        let d = presets::door().design;
        let ky = reluctance_ky_n_per_m(&d.suspension, d.suspension.g_star_m, 0.0);
        assert!((ky - 12.26e3).abs() < 0.01 * 12.26e3, "k_y {ky} N/m");
        // Yaw stiffness: 4 elements at x ±0.40 m about the CoM: 4·k_y·0.16.
        let kyaw = yaw_stiffness_n_m_per_rad(
            &d.suspension,
            d.carriage.cg_m[0],
            d.suspension.g_star_m,
            0.0,
        );
        assert!((kyaw - 4.0 * ky * 0.16).abs() < 1e-9 * kyaw);
        // Preset sway damping: DOOR ζ ≈ 0.08 at the 1.05 m pendulum (LEV-011 ≥ 0.05).
        let z = sway_zeta(&d.guidance, d.carriage.m_star_kg, d.carriage.cg_m[2].abs());
        assert!((z - 0.081).abs() < 0.002, "DOOR sway ζ {z}");
        let w = presets::wall().design;
        let zw = sway_zeta(&w.guidance, w.carriage.m_star_kg, w.carriage.cg_m[2].abs());
        assert!(zw >= 0.05, "WALL sway ζ {zw} (LEV-011)");
    }
}
