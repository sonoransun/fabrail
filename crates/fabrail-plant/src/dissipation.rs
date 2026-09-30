//! Dissipation ledger: distributed damping, progressive end zones, rail eddy drag and hysteresis, mu_eq (PHYSICS §9). Owner: A6b.
//!
//! Rail eddy drag c_rail·ẋ and hysteresis F_hyst both scale with ΣΦ²/ΣΦ0² (the flux
//! ratio); μ_eq ≈ 1e-3–1e-2 is reported, never "zero friction" (core design §C.1).
//!
//! Components (PHYSICS §9; physics perspective §1.7), each a signed force along +x that
//! opposes the travel velocity:
//!
//! * distributed eddy damping −b_dist·ẋ (b_dist 2 / 5 / 20 / 100 N·s/m; it also sets
//!   the passivity headroom);
//! * progressive end zones −c(x)·ẋ with c(x) = c_max·smoothstep(depth/L_ez), the depth
//!   measured beyond the nominal travel limits [x_closed, x_closed + travel] (SAF-014:
//!   nominal termini never engage them); linear eddy regime v ≪ v_c ≈ 9 m/s;
//! * rail eddy drag −c_rail·ẋ·r and hysteresis −F_hyst·r·tanh(ẋ/v_h), r = ΣΦ²/ΣΦ0²
//!   (truth values from `PlantParams`, \[UNCERTAIN, swept\] by the drag_ledger study);
//!   the stateless tanh creeps below v_h, so tiers that carry a hysteresis bristle state
//!   use [`rail_hyst_bristle`] instead;
//! * aerodynamic drag −½ρ·(C_D·A_front + C_f·2·A_leaf)·|ẋ|·ẋ (negligible, kept).

use fabrail_params::derive::{G0_MPS2, RHO_AIR_KG_PER_M3};
use fabrail_types::fmath;

use crate::contact::travel_limits_m;
use crate::friction::BristleParams;
use crate::model::EnergyLedger;
use crate::params::PlantParams;

/// Regularisation speed of the stateless hysteresis drag sign, m/s (design choice; the
/// same 1 mm/s reference as the stateless guide friction; PHYSICS §9).
pub const V_HYST_MPS: f64 = 1.0e-3;

/// Bluff-body drag coefficient of a front moving face-on (DRAWER front), dimensionless
/// (flat plate normal to the flow ≈ 1.17, cube 1.05; Hoerner, Fluid-Dynamic Drag, 1965;
/// 1.05 used; PHYSICS §9).
pub const CD_FRONT: f64 = 1.05;

/// Skin-friction coefficient of a leaf sliding in its own plane, dimensionless (laminar
/// Blasius 1.328/√Re at Re ≈ 1e4–2e4 gives ≈ 0.01; order of magnitude only; PHYSICS §9).
pub const CF_LEAF: f64 = 0.01;

/// Drag components along x, N (each opposes the travel velocity).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DragForces {
    /// Distributed eddy damping b_dist·ẋ.
    pub dist_n: f64,
    /// Progressive end zone.
    pub end_zone_n: f64,
    /// Rail eddy drag.
    pub rail_eddy_n: f64,
    /// Rail hysteresis drag.
    pub rail_hyst_n: f64,
    /// Air drag.
    pub air_n: f64,
}

impl DragForces {
    /// Sum of all components, N.
    pub fn total_n(&self) -> f64 {
        self.dist_n + self.end_zone_n + self.rail_eddy_n + self.rail_hyst_n + self.air_n
    }
}

/// Cubic smoothstep 3s² − 2s³ on s clamped to [0, 1] (C¹ ramp).
fn smoothstep(s: f64) -> f64 {
    let s = s.clamp(0.0, 1.0);
    s * s * (3.0 - 2.0 * s)
}

/// Depth beyond the nominal travel limits into either end zone, m (0 inside the travel).
pub fn end_zone_depth_m(p: &PlantParams, x_m: f64) -> f64 {
    let [lo, hi] = travel_limits_m(&p.design);
    if x_m < lo {
        lo - x_m
    } else if x_m > hi {
        x_m - hi
    } else {
        0.0
    }
}

/// End-zone damping c(x) = c_max·smoothstep(depth/L_ez), N·s/m: 0 inside the nominal
/// travel, c_max from one zone length beyond a limit (the bumper face) onward
/// (PHYSICS §9; SAF-014).
pub fn end_zone_c(p: &PlantParams, x_m: f64) -> f64 {
    let dmp = &p.design.damping;
    let depth = end_zone_depth_m(p, x_m);
    if depth <= 0.0 || dmp.ez_cmax_n_s_per_m <= 0.0 {
        return 0.0;
    }
    if dmp.ez_len_m <= 0.0 {
        return dmp.ez_cmax_n_s_per_m;
    }
    dmp.ez_cmax_n_s_per_m * smoothstep(depth / dmp.ez_len_m)
}

/// Aerodynamic C·A, m²: C_D·A_front + C_f·2·A_leaf with the DRAWER front taken as the
/// cavity piston area and the leaf area from `design.air` (PHYSICS §9, §11).
pub fn air_cda_m2(p: &PlantParams) -> f64 {
    let a = &p.design.air;
    CD_FRONT * a.cavity_area_m2.max(0.0) + CF_LEAF * 2.0 * a.leaf_area_m2.max(0.0)
}

/// Drag at travel position `x_m` and velocity `v_mps` with flux ratio ΣΦ²/ΣΦ0²
/// (PHYSICS §9). Every component is a signed force along +x opposing `v_mps`; the flux
/// ratio is clamped to ≥ 0 (levitated at g* it is 1; landed it drops with the flux).
pub fn drag(p: &PlantParams, x_m: f64, v_mps: f64, flux_ratio: f64) -> DragForces {
    if !(x_m.is_finite() && v_mps.is_finite()) {
        return DragForces::default();
    }
    let r = if flux_ratio.is_finite() {
        flux_ratio.max(0.0)
    } else {
        0.0
    };
    let dmp = &p.design.damping;
    DragForces {
        dist_n: -dmp.b_dist_n_s_per_m.max(0.0) * v_mps,
        end_zone_n: -end_zone_c(p, x_m) * v_mps,
        rail_eddy_n: -p.c_rail_n_s_per_m.max(0.0) * r * v_mps,
        rail_hyst_n: -p.f_hyst_n.max(0.0) * r * fmath::tanh(v_mps / V_HYST_MPS),
        air_n: -0.5 * RHO_AIR_KG_PER_M3 * air_cda_m2(p) * v_mps.abs() * v_mps,
    }
}

/// Elasto-plastic law for the rail hysteresis drag (physics perspective §1.7 "sign via
/// bristle"): μ = 1 with the "normal" load F_hyst·r, so
/// `friction::bristle(&rail_hyst_bristle(z_ss), z, v, p.f_hyst_n·r).f_t_n` replaces
/// `DragForces::rail_hyst_n` without creep.
pub fn rail_hyst_bristle(z_ss_m: f64) -> BristleParams {
    BristleParams::new(1.0, 1.0, z_ss_m)
}

/// Flux ratio ΣΦ²/ΣΦ0² over the active elements (`phi0_wb` the reference fluxes at g*,
/// m*), dimensionless; 1 when the reference is empty or zero (PHYSICS §9).
pub fn flux_ratio(phi_wb: &[f64], phi0_wb: &[f64]) -> f64 {
    let num: f64 = phi_wb.iter().map(|f| f * f).sum();
    let den: f64 = phi0_wb.iter().map(|f| f * f).sum();
    if den > 0.0 && num.is_finite() {
        num / den
    } else {
        1.0
    }
}

/// Adds one step's dissipated energy to the ledger: each component's −F·v·h (≥ 0 for a
/// drag that opposes the motion) into `dist_j`, `end_zone_j`, `rail_eddy_j`,
/// `rail_hyst_j` and `air_j` (PHYSICS §9). `v_mps` is the step-average velocity.
pub fn accumulate(ledger: &mut EnergyLedger, d: &DragForces, v_mps: f64, h_s: f64) {
    let w = |f: f64| (-f * v_mps * h_s).max(0.0);
    ledger.dist_j += w(d.dist_n);
    ledger.end_zone_j += w(d.end_zone_n);
    ledger.rail_eddy_j += w(d.rail_eddy_n);
    ledger.rail_hyst_j += w(d.rail_hyst_n);
    ledger.air_j += w(d.air_n);
}

/// Total drag energy in the ledger (distributed, end zones, rail, air and friction),
/// J: the numerator of [`mu_eq`] over a traverse.
pub fn drag_energy_j(l: &EnergyLedger) -> f64 {
    l.dist_j + l.end_zone_j + l.rail_eddy_j + l.rail_hyst_j + l.air_j + l.friction_j
}

/// Equivalent friction coefficient μ_eq = E_drag / (m·g₀·d) over a traverse of length
/// `distance_m` (PHYSICS §9; g₀ standard gravity). 0 when mass or distance is ≤ 0.
pub fn mu_eq(e_drag_j: f64, m_kg: f64, distance_m: f64) -> f64 {
    if m_kg <= 0.0 || distance_m <= 0.0 || !(m_kg.is_finite() && distance_m.is_finite()) {
        return 0.0;
    }
    e_drag_j / (m_kg * G0_MPS2 * distance_m)
}

/// Levitated μ_eq at constant speed `v_mps` mid-travel with the nominal flux
/// (r = 1): |ΣF_drag|/(m·g₀) for body mass `m_kg` (PHYSICS §9; the "zero friction" is
/// a direction, μ_eq ≈ 1e-3–1e-2).
pub fn mu_eq_levitated(p: &PlantParams, m_kg: f64, v_mps: f64) -> f64 {
    let [lo, hi] = travel_limits_m(&p.design);
    let d = drag(p, 0.5 * (lo + hi), v_mps, 1.0);
    if m_kg <= 0.0 {
        return 0.0;
    }
    d.total_n().abs() / (m_kg * G0_MPS2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::{Environment, Mismatch};
    use fabrail_params::presets;

    fn plant(sp: &fabrail_params::SystemParams) -> PlantParams {
        PlantParams::from_design(&sp.design, &Mismatch::default(), &Environment::default())
    }

    #[test]
    fn mu_eq_definition() {
        // 100 kg dragged 1 m against 9.80665 N of drag: μ_eq = 0.01.
        assert!((mu_eq(9.80665, 100.0, 1.0) - 0.01).abs() < 1e-15);
        assert_eq!(mu_eq(1.0, 0.0, 1.0), 0.0);
        let d = DragForces {
            dist_n: 1.0,
            end_zone_n: 2.0,
            rail_eddy_n: 3.0,
            rail_hyst_n: 4.0,
            air_n: 5.0,
        };
        assert_eq!(d.total_n(), 15.0);
    }

    #[test]
    fn components_oppose_motion_and_scale_with_flux() {
        let p = plant(&presets::door());
        let d = drag(&p, 0.5, 0.15, 1.0);
        for f in [d.dist_n, d.rail_eddy_n, d.rail_hyst_n, d.air_n] {
            assert!(f < 0.0, "{d:?}");
        }
        assert_eq!(d.end_zone_n, 0.0, "no end zone inside the travel (SAF-014)");
        // (b_dist + c_rail)·v + F_hyst = (5 + 5)·0.15 + 1·tanh(150) (E) PHYSICS §9.
        let lev = d.dist_n + d.rail_eddy_n + d.rail_hyst_n;
        assert!(
            (lev + 2.5).abs() < 1e-9,
            "levitated drag {lev} vs derive::drag_force 2.5 N"
        );
        let half = drag(&p, 0.5, 0.15, 0.5);
        assert!((half.rail_eddy_n - 0.5 * d.rail_eddy_n).abs() < 1e-15);
        assert!((half.rail_hyst_n - 0.5 * d.rail_hyst_n).abs() < 1e-15);
        assert_eq!(half.dist_n, d.dist_n, "b_dist does not scale with flux");
        let back = drag(&p, 0.5, -0.15, 1.0);
        assert!((back.total_n() + d.total_n()).abs() < 1e-12, "odd in v");
        assert_eq!(drag(&p, 0.5, 0.0, 1.0).total_n(), 0.0);
    }

    #[test]
    fn end_zones_are_progressive_and_outside_the_travel() {
        let p = plant(&presets::wall());
        // WALL travel [0, 6] m, zones 150 mm, c_max 1500 N·s/m.
        assert_eq!(end_zone_c(&p, 0.0), 0.0);
        assert_eq!(end_zone_c(&p, 6.0), 0.0);
        assert!(
            (end_zone_c(&p, 6.075) - 750.0).abs() < 1e-9,
            "smoothstep(½) = ½"
        );
        assert!((end_zone_c(&p, -0.15) - 1500.0).abs() < 1e-9);
        assert!(
            (end_zone_c(&p, -0.3) - 1500.0).abs() < 1e-9,
            "saturates at the bumper"
        );
        // Monotone in depth.
        let mut last = 0.0;
        for k in 0..=30 {
            let c = end_zone_c(&p, 6.0 + f64::from(k) * 0.005);
            assert!(c >= last);
            last = c;
        }
    }

    #[test]
    fn mu_eq_band_at_cap_speed() {
        // Levitated μ_eq at each class's unsensed speed cap and nominal drag lies in the
        // 1e-3–1e-2 band (NARRATIVE "zero friction is a direction"; PHYSICS §9).
        for sp in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let p = plant(&sp);
            let v = sp.safety.v_open_max_mps.max(sp.safety.v_close_max_mps);
            let mu = mu_eq_levitated(&p, sp.design.carriage.m_star_kg, v);
            assert!(
                (1.0e-3..=1.0e-2).contains(&mu),
                "{:?} μ_eq {mu} at {v} m/s outside 1e-3–1e-2 (PHYSICS §9)",
                sp.design.class
            );
        }
    }

    #[test]
    fn ledger_accumulates_dissipation() {
        let p = plant(&presets::drawer());
        let mut l = EnergyLedger::default();
        let v = 0.2;
        for _ in 0..1000 {
            let d = drag(&p, 0.1, v, 1.0);
            accumulate(&mut l, &d, v, 1.0e-3);
        }
        // 0.2 m at 0.2 m/s: E = |F|·d with |F| = (2 + 1)·0.2 + 0.2·tanh(200) + air.
        let f = drag(&p, 0.1, v, 1.0).total_n().abs();
        assert!((drag_energy_j(&l) - f * 0.2).abs() < 1e-12);
        assert!((mu_eq(drag_energy_j(&l), 26.0, 0.2) - f / (26.0 * G0_MPS2)).abs() < 1e-15);
        assert_eq!(flux_ratio(&[1.0, 1.0], &[1.0, 1.0]), 1.0);
        assert!((flux_ratio(&[0.5, 0.5], &[1.0, 1.0]) - 0.25).abs() < 1e-15);
        assert_eq!(flux_ratio(&[0.5], &[]), 1.0);
    }
}
