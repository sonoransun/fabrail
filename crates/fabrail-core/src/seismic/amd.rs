//! Active mass damping: direct velocity feedback or injected LQR gains; research tier (PHYSICS §21). Owner: A3.
//!
//! Frozen API (W0): [`Amd::new`], [`Amd::step`] → force (N). Direct velocity feedback
//! F = −c_dvf·ẋ_s, or LQR state feedback on [x_s, ẋ_s, x_r, ẋ_r] with gains injected
//! from Python (scipy CARE; `seismic.amd.k_lqr`). Mass dampers are mainly wind devices;
//! AMD is research tier (`amd.enable = false` in every preset). SEI-016/021 (closed-loop
//! AMD/TMD on the 2-DOF Structure model).
//!
//! **Sign convention** (PHYSICS §21). `step` returns the LSM force on the **carriage**
//! (the reaction mass), +x, like every core force demand; the structure feels the
//! reaction −F. The damping laws are written for the force on the structure, F_s:
//!
//! * DVF (all `k_lqr` zero): F_s = −c_dvf·ẋ_s, so F = +c_dvf·ẋ_s;
//! * LQR: the injected gains act on the carriage force directly, F = −K·[x_s, ẋ_s, x_r,
//!   ẋ_r], with x_s, ẋ_s the host storey and x_r, ẋ_r the carriage relative to it (the
//!   Structure tier's `Truth::x_m`); the Python CARE design uses this input sign.
//!
//! F is clamped to ±`amd.f_max_n` (≤ F_cont, `validate`), and the carriage's relative
//! stroke is governed exactly as in isolation (`isolate::stroke_governor`, envelope
//! min(`isolation.stroke_m`, `IsolateLimits::stroke_m`), planning with
//! F_max/`a_brk_ratio`). No integrator, so no windup. The mass ratio μ = m/M is what
//! decides the benefit (SEI-021: negligible at μ = 0.05 %).

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::fmath;

use crate::seismic::isolate::stroke_governor;

/// AMD controller.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Amd {
    /// Mode available.
    pub enable: bool,
    /// LQR gains (f32 copy).
    pub k_lqr: [f32; 4],
    /// DVF gain, N·s/m.
    pub c_dvf_n_s_per_m: f32,
    /// Force limit, N.
    pub f_max_n: f32,
    /// Carriage stroke envelope about the centre, m (0: ungoverned).
    pub stroke_m: f32,
    /// Governor mass (rated), kg.
    pub m_kg: f32,
    /// Governor planning ratio.
    pub brk_ratio: f32,
    /// The last output was clamped or governed.
    pub saturated: bool,
}

impl Amd {
    /// AMD from `control.seismic.amd` (plus the isolation stroke envelope and the rated
    /// mass for the stroke governor).
    pub fn new(p: &SystemParams) -> Result<Amd, ParamError> {
        let a = &p.control.seismic.amd;
        let s_iso = p.control.seismic.isolation.stroke_m;
        let s_lim = p.safety.isolate.stroke_m;
        let stroke = if s_lim > 0.0 {
            fmath::fmin(s_iso, s_lim)
        } else {
            s_iso
        };
        Ok(Amd {
            enable: a.enable,
            k_lqr: a.k_lqr.map(|k| k as f32),
            c_dvf_n_s_per_m: a.c_dvf_n_s_per_m as f32,
            f_max_n: a.f_max_n as f32,
            stroke_m: fmath::fmax(stroke, 0.0) as f32,
            m_kg: p.design.carriage.m_rated_kg as f32,
            brk_ratio: p.control.motion.a_brk_ratio as f32,
            saturated: false,
        })
    }

    /// One fast release from the state [x_s m, ẋ_s m/s, x_r m, ẋ_r m/s]; returns the LSM
    /// force on the carriage, N (0 when disabled or on a non-finite state).
    pub fn step(&mut self, state: &[f32; 4]) -> f32 {
        self.saturated = false;
        if !(self.enable && state.iter().all(|v| v.is_finite())) {
            return 0.0;
        }
        let [_, v_s, x_r, v_r] = *state;
        let f = if self.k_lqr.iter().all(|&k| k == 0.0) {
            self.c_dvf_n_s_per_m * v_s
        } else {
            -self
                .k_lqr
                .iter()
                .zip(state.iter())
                .fold(0.0_f32, |acc, (k, s)| acc + k * s)
        };
        let f_max = fmath::fmaxf(self.f_max_n, 0.0);
        if !f.is_finite() {
            return 0.0;
        }
        let clamped = fmath::clampf(f, -f_max, f_max);
        let (out, governed) = stroke_governor(
            x_r,
            v_r,
            clamped,
            f_max,
            self.stroke_m,
            self.m_kg,
            self.brk_ratio,
        );
        self.saturated = governed || clamped != f;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    fn heavy_on() -> SystemParams {
        let mut p = presets::heavy();
        p.control.seismic.amd.enable = true;
        p.control.seismic.amd.c_dvf_n_s_per_m = 2000.0;
        p
    }

    #[test]
    fn dvf_damps_the_structure_and_saturates() {
        let mut off = Amd::new(&presets::heavy()).expect("amd");
        assert_eq!(
            off.step(&[0.0, 0.1, 0.0, 0.0]),
            0.0,
            "research tier off in the preset"
        );
        let p = heavy_on();
        let mut a = Amd::new(&p).expect("amd");
        assert_eq!(a.f_max_n, 500.0, "F_max = F_cont (HEAVY 500 N)");
        let f = a.step(&[0.0, 0.1, 0.0, 0.0]);
        assert!(
            (f - 200.0).abs() < 1e-3,
            "carriage force +c·ẋ_s; structure feels −c·ẋ_s"
        );
        assert!(!a.saturated);
        assert_eq!(a.step(&[0.0, 1.0, 0.0, 0.0]), 500.0, "clamped at F_max");
        assert!(a.saturated);
        assert_eq!(
            a.step(&[0.0, f32::NAN, 0.0, 0.0]),
            0.0,
            "non-finite state → 0 N"
        );
    }

    #[test]
    fn lqr_gains_and_stroke_governor() {
        let mut p = heavy_on();
        p.control.seismic.amd.k_lqr = [1000.0, 200.0, 50.0, 10.0];
        let mut a = Amd::new(&p).expect("amd");
        let f = a.step(&[0.01, 0.1, 0.02, 0.0]);
        assert!((f + (10.0 + 20.0 + 1.0)).abs() < 1e-3, "F = −K·x");
        // Carriage near the envelope moving outward fast: governed braking.
        let f = a.step(&[0.0, 0.0, 0.19, 0.3]);
        assert_eq!(f, -500.0, "stroke governor brakes the carriage");
        assert!(a.saturated);
    }
}
