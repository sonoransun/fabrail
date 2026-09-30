//! PlantParams::from_design, Mismatch (truth vs nominal) and Environment. Owner: A6a
//! (signatures frozen by W0-B3).
//!
//! The controller is designed from the nominal [`DesignParams`]; the plant is the truth.
//! [`Mismatch`] perturbs the truth away from the nominal (scenario `plant` block) so
//! robustness and estimator tests have something to estimate. Every `Option` override
//! means "the design value" when `None`.

use fabrail_params::{ControlParams, DesignParams, presets};

use crate::model::PlantError;
use crate::rail::RailProfile;
use crate::structure::ShearBuilding;

/// Truth-vs-nominal perturbations (scenario `plant` block). `Default` is the identity:
/// the plant equals the design.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mismatch {
    /// Body mass scale: truth m_body = mass_scale · m* (payload comes on top).
    pub mass_scale: f64,
    /// Magnet remanence scale: truth B_r = br_scale · B_r,design (tolerance, ageing).
    pub br_scale: f64,
    /// Flux leakage σ_leak override (0 = ideal; realistic 0.25 in sensitivity studies
    /// only, core design §C.5).
    pub leak_sigma: Option<f64>,
    /// Rail eddy drag c_rail override, N·s/m [UNCERTAIN, swept].
    pub c_rail_n_s_per_m: Option<f64>,
    /// Rail hysteresis drag F_hyst override, N [UNCERTAIN, swept].
    pub f_hyst_n: Option<f64>,
    /// DRAWER cavity leak area override, m².
    pub leak_area_m2: Option<f64>,
}

impl Default for Mismatch {
    fn default() -> Self {
        Mismatch {
            mass_scale: 1.0,
            br_scale: 1.0,
            leak_sigma: None,
            c_rail_n_s_per_m: None,
            f_hyst_n: None,
            leak_area_m2: None,
        }
    }
}

impl Mismatch {
    /// Range checks: scales in (0, 10], σ_leak in [0, 1), drags and areas ≥ 0, all
    /// finite.
    pub fn validate(&self) -> Result<(), PlantError> {
        let scale_ok = |v: f64| v.is_finite() && v > 0.0 && v <= 10.0;
        if !scale_ok(self.mass_scale) {
            return Err(PlantError::Invalid(
                "plant.mass_scale",
                "must be in (0, 10]",
            ));
        }
        if !scale_ok(self.br_scale) {
            return Err(PlantError::Invalid("plant.br_scale", "must be in (0, 10]"));
        }
        if let Some(s) = self.leak_sigma
            && !(s.is_finite() && (0.0..1.0).contains(&s))
        {
            return Err(PlantError::Invalid("plant.leak_sigma", "must be in [0, 1)"));
        }
        let nonneg = |v: Option<f64>| v.is_none_or(|x| x.is_finite() && x >= 0.0);
        if !nonneg(self.c_rail_n_s_per_m) {
            return Err(PlantError::Invalid(
                "plant.c_rail_n_s_per_m",
                "must be finite and >= 0",
            ));
        }
        if !nonneg(self.f_hyst_n) {
            return Err(PlantError::Invalid(
                "plant.f_hyst_n",
                "must be finite and >= 0",
            ));
        }
        if !nonneg(self.leak_area_m2) {
            return Err(PlantError::Invalid(
                "plant.leak_area_m2",
                "must be finite and >= 0",
            ));
        }
        Ok(())
    }
}

/// The world around the device: rail irregularity and, for the Structure tier, the
/// host building.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Environment {
    /// Measured/synthesised rail profile (PHYSICS §10); `None` = ideal straight rail.
    pub rail: Option<RailProfile>,
    /// Shear building for the Structure tier (PHYSICS §21); `None` otherwise.
    pub structure: Option<ShearBuilding>,
}

/// Plant truth parameters: the nominal design plus resolved truth values.
#[derive(Clone, Debug, PartialEq)]
pub struct PlantParams {
    /// Nominal hardware (what the controller was designed for).
    pub design: DesignParams,
    /// The perturbation applied.
    pub mismatch: Mismatch,
    /// Environment.
    pub env: Environment,
    /// Truth body mass without payload: mass_scale · m*, kg.
    pub m_body_kg: f64,
    /// Truth magnet remanence at 20 °C: br_scale · B_r, T.
    pub b_r_t: f64,
    /// Truth flux leakage σ_leak.
    pub leak: f64,
    /// Truth rail eddy drag, N·s/m.
    pub c_rail_n_s_per_m: f64,
    /// Truth rail hysteresis drag, N.
    pub f_hyst_n: f64,
    /// Truth DRAWER cavity leak area, m².
    pub leak_area_m2: f64,
    /// Closed current-loop lag τ_cl = 1/(2π·current_bw_hz) of the Lag coil mode, s
    /// (plan §2b item 2: 1 kHz → 0.159 ms; PHYSICS §4).
    pub tau_cl_s: f64,
    /// Travel1d kinematic heave: land speed, m/s (core design §C.4: 5 / 5 / 3 / 2 mm/s).
    pub land_speed_mps: f64,
    /// Travel1d kinematic heave: lift / unseat time, s (core design §C.4: 0.1 / 0.3 / 1 /
    /// 2 s).
    pub lift_time_s: f64,
}

impl PlantParams {
    /// Resolves the truth from the nominal design, the mismatch and the environment.
    /// Inputs are validated upstream (`fabrail_params::validate`, the scenario spec and
    /// [`Mismatch::validate`]); this never fails. The current-loop lag and the Travel1d
    /// heave kinematics come from the class preset's control block
    /// (`fabrail_params::presets::by_class`); [`PlantParams::with_control`] replaces them
    /// with the run's actual control parameters.
    pub fn from_design(d: &DesignParams, mismatch: &Mismatch, env: &Environment) -> PlantParams {
        let lev = presets::by_class(d.class).control.levitation;
        PlantParams {
            design: *d,
            mismatch: *mismatch,
            env: env.clone(),
            m_body_kg: mismatch.mass_scale * d.carriage.m_star_kg,
            b_r_t: mismatch.br_scale * d.suspension.b_r_t,
            leak: mismatch.leak_sigma.unwrap_or(d.suspension.leak),
            c_rail_n_s_per_m: mismatch
                .c_rail_n_s_per_m
                .unwrap_or(d.damping.c_rail_n_s_per_m),
            f_hyst_n: mismatch.f_hyst_n.unwrap_or(d.damping.f_hyst_n),
            leak_area_m2: mismatch.leak_area_m2.unwrap_or(d.air.leak_area_m2),
            tau_cl_s: tau_from_bw(lev.current_bw_hz),
            land_speed_mps: lev.land_speed_mps,
            lift_time_s: lev.lift_time_max_s,
        }
    }

    /// The same truth with the current-loop lag and the heave kinematics taken from the
    /// run's control parameters `c` (τ_cl = 1/(2π·current_bw_hz), land speed, lift time).
    pub fn with_control(mut self, c: &ControlParams) -> PlantParams {
        let lev = &c.levitation;
        self.tau_cl_s = tau_from_bw(lev.current_bw_hz);
        self.land_speed_mps = lev.land_speed_mps;
        self.lift_time_s = lev.lift_time_max_s;
        self
    }
}

/// τ = 1/(2π·f), 0 for a non-positive bandwidth (instantaneous), s.
fn tau_from_bw(bw_hz: f64) -> f64 {
    if bw_hz > 0.0 {
        1.0 / (2.0 * core::f64::consts::PI * bw_hz)
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_mismatch_reproduces_the_design() {
        let mut d = DesignParams::default();
        d.carriage.m_star_kg = 26.0;
        d.suspension.b_r_t = 1.3;
        d.damping.c_rail_n_s_per_m = 1.0;
        d.damping.f_hyst_n = 0.2;
        d.air.leak_area_m2 = 4.0e-3;
        let p = PlantParams::from_design(&d, &Mismatch::default(), &Environment::default());
        assert_eq!(p.m_body_kg, 26.0);
        assert_eq!(p.b_r_t, 1.3);
        assert_eq!(p.c_rail_n_s_per_m, 1.0);
        assert_eq!(p.f_hyst_n, 0.2);
        assert_eq!(p.leak_area_m2, 4.0e-3);
        assert_eq!(p.leak, 0.0);
    }

    #[test]
    fn overrides_and_scales_apply() {
        let mut d = DesignParams::default();
        d.carriage.m_star_kg = 100.0;
        d.suspension.b_r_t = 1.2;
        let m = Mismatch {
            mass_scale: 1.1,
            br_scale: 0.95,
            leak_sigma: Some(0.25),
            c_rail_n_s_per_m: Some(7.0),
            f_hyst_n: Some(2.0),
            leak_area_m2: Some(1.0e-3),
        };
        m.validate().expect("valid");
        let p = PlantParams::from_design(&d, &m, &Environment::default());
        assert!((p.m_body_kg - 110.0).abs() < 1e-12);
        assert!((p.b_r_t - 1.14).abs() < 1e-12);
        assert_eq!(
            (p.leak, p.c_rail_n_s_per_m, p.f_hyst_n, p.leak_area_m2),
            (0.25, 7.0, 2.0, 1.0e-3)
        );
    }

    #[test]
    fn mismatch_validation_rejects_out_of_range() {
        let bad = [
            Mismatch {
                mass_scale: 0.0,
                ..Mismatch::default()
            },
            Mismatch {
                br_scale: f64::NAN,
                ..Mismatch::default()
            },
            Mismatch {
                leak_sigma: Some(1.0),
                ..Mismatch::default()
            },
            Mismatch {
                c_rail_n_s_per_m: Some(-1.0),
                ..Mismatch::default()
            },
            Mismatch {
                f_hyst_n: Some(f64::INFINITY),
                ..Mismatch::default()
            },
            Mismatch {
                leak_area_m2: Some(-1e-3),
                ..Mismatch::default()
            },
        ];
        for m in bad {
            assert!(m.validate().is_err(), "{m:?}");
        }
    }
}
