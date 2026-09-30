//! Contactless isolation F = -k*x_rel - c_sky*v_abs with a stroke governor; research tier (PHYSICS §21). Owner: A3.
//!
//! Frozen API (W0): [`Isolator::new`], [`Isolator::step`] → force (N). F = −k·x_rel −
//! c_sky·ẋ_abs with ẋ_abs from the leaky-integrated carriage IMU (high-pass 0.1 Hz),
//! saturated at F_cont plus a stroke governor; T = max(2 s, 1.1·T_min) with T_min from
//! F_cont (WALL 6.3 s, so WALL isolation largely saturates; plan §2b item 5). In
//! ISOLATE the monitor swaps its clamp/KE checks for `SafetyLimits::isolate`
//! (SAF-020). Research tier: `isolation.enable = false` in every preset. Test:
//! `stroke_governor`.
//!
//! **Law** (Karnopp skyhook; PHYSICS §21). k = m·ω², c_sky = 2ζ·m·ω with ω = 2π/T_iso
//! and m the rated mass ([`isolate_cfg`]); T_iso = `isolation.t_iso_s`, which `validate`
//! holds at ≥ max(2 s, 1.1·T_min), T_min = 2π√(m·s/F_cont): at T_min the spring alone
//! needs F_cont at full stroke, so the 1.1 margin leaves 17 % of F_cont for damping and
//! the governor. The force is clamped to ±F_max = min(F_cont, `IsolateLimits::f_cont_n`)
//! and the stroke to min(`isolation.stroke_m`, `IsolateLimits::stroke_m`), the envelope
//! the monitor enforces in ISOLATE (SAF-020).
//!
//! **Stroke governor** (`stroke_governor`). Moving outward with r = s − |x| left, the
//! force needed to stop inside the envelope is F_stop = m·v²/(2r). Once
//! F_stop ≥ F_max/`a_brk_ratio` (plan with half the capacity, the motion governor's
//! a_brk = 2·a_max rule, core design §C.4) the governor overrides the law with full
//! braking −sign(v)·F_max, which then stops the carriage within half the remaining
//! stroke in the floor frame; at or beyond the envelope it pushes back at F_max unless
//! already returning. Floor acceleration above F_max/(2m) can still drive the carriage
//! to the envelope: that is why WALL isolation saturates and the bumpers and IsolateLimits
//! stay in force (SEI-015).

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{PosNm, fmath};

use crate::filters::LeakyIntegrator;
use crate::msg::IsolateCfg;

/// Isolation controller output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct IsolateOut {
    /// Force demand, N.
    pub f_n: f32,
    /// The force or stroke governor saturated.
    pub saturated: bool,
}

/// Skyhook isolation controller.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Isolator {
    /// Mode available.
    pub enable: bool,
    /// Leaky-integrated absolute velocity, m/s.
    pub v_abs_mps: f32,
    /// Governor mass (rated), kg.
    pub m_kg: f32,
    /// Governor planning ratio (plan with F_max / ratio).
    pub brk_ratio: f32,
    /// IMU leaky integrator (0.1 Hz high-pass) at the fast rate.
    pub integ: LeakyIntegrator,
}

/// The research-tier isolation configuration for `p` about `center` (init-time, f64):
/// k = m·(2π/T)², c_sky = 2ζ·m·(2π/T), stroke and force within `IsolateLimits`
/// (SAF-020), AMD DVF gain passed through. Zero gains when T is not positive.
pub fn isolate_cfg(p: &SystemParams, center: PosNm) -> IsolateCfg {
    let iso = &p.control.seismic.isolation;
    let lim = &p.safety.isolate;
    let m = p.design.carriage.m_rated_kg;
    let w = if iso.t_iso_s > 0.0 {
        2.0 * core::f64::consts::PI / iso.t_iso_s
    } else {
        0.0
    };
    let tighter = |a: f64, b: f64| if b > 0.0 { fmath::fmin(a, b) } else { a };
    IsolateCfg {
        k_n_per_m: (m * w * w) as f32,
        c_sky_n_s_per_m: (2.0 * iso.zeta * m * w) as f32,
        center,
        stroke_m: tighter(iso.stroke_m, lim.stroke_m) as f32,
        f_max_n: tighter(p.design.lsm.f_cont_n, lim.f_cont_n) as f32,
        c_dvf_n_s_per_m: p.control.seismic.amd.c_dvf_n_s_per_m as f32,
    }
}

/// The stroke governor (see the module docs): returns the governed force and whether it
/// intervened. `x_m`/`v_mps` are relative to the centre; `f_n` is the law's (already
/// clamped) force; `m_kg` the governor mass.
pub(crate) fn stroke_governor(
    x_m: f32,
    v_mps: f32,
    f_n: f32,
    f_max_n: f32,
    stroke_m: f32,
    m_kg: f32,
    brk_ratio: f32,
) -> (f32, bool) {
    if !(stroke_m > 0.0 && f_max_n > 0.0) {
        return (f_n, false);
    }
    let outward = x_m * v_mps > 0.0;
    let ax = fmath::absf(x_m);
    if ax >= stroke_m {
        // At or beyond the envelope: push back unless already returning.
        if v_mps * x_m >= 0.0 {
            return (-fmath::copysignf(f_max_n, x_m), true);
        }
        return (f_n, false);
    }
    if outward {
        let r = stroke_m - ax;
        let f_stop = m_kg * v_mps * v_mps / (2.0 * r);
        if f_stop * fmath::fmaxf(brk_ratio, 1.0) >= f_max_n {
            return (-fmath::copysignf(f_max_n, v_mps), true);
        }
    }
    (f_n, false)
}

impl Isolator {
    /// Isolator from `control.seismic.isolation` (enable, IMU high-pass), the rated mass
    /// and the fast-group rate. Degenerate rates give an isolator whose integrator stays
    /// at zero.
    pub fn new(p: &SystemParams) -> Result<Isolator, ParamError> {
        let r = &p.control.rates;
        let fs = if r.fast_decim > 0 {
            f64::from(r.base_hz) / f64::from(r.fast_decim)
        } else {
            0.0
        };
        Ok(Isolator {
            enable: p.control.seismic.isolation.enable,
            v_abs_mps: 0.0,
            m_kg: p.design.carriage.m_rated_kg as f32,
            brk_ratio: p.control.motion.a_brk_ratio as f32,
            integ: LeakyIntegrator::new(p.control.seismic.isolation.hp_hz, fs),
        })
    }

    /// One fast release from the relative position to the centre (m), relative
    /// velocity (m/s) and IMU x-acceleration (m/s²). Disabled (research tier off) or
    /// non-finite inputs give 0 N.
    pub fn step(
        &mut self,
        x_rel_m: f32,
        v_rel_mps: f32,
        imu_x_mps2: f32,
        cfg: &IsolateCfg,
    ) -> IsolateOut {
        if imu_x_mps2.is_finite() {
            self.v_abs_mps = self.integ.step(imu_x_mps2);
        }
        if !(self.enable && x_rel_m.is_finite() && v_rel_mps.is_finite()) {
            return IsolateOut::default();
        }
        let f_max = fmath::fmaxf(cfg.f_max_n, 0.0);
        let law = -cfg.k_n_per_m * x_rel_m - cfg.c_sky_n_s_per_m * self.v_abs_mps;
        if !law.is_finite() {
            return IsolateOut::default();
        }
        let f = fmath::clampf(law, -f_max, f_max);
        let clamped = fmath::absf(law) > f_max;
        let (f, governed) = stroke_governor(
            x_rel_m,
            v_rel_mps,
            f,
            f_max,
            cfg.stroke_m,
            self.m_kg,
            self.brk_ratio,
        );
        IsolateOut {
            f_n: f,
            saturated: clamped || governed,
        }
    }

    /// Clears the integrator.
    pub fn reset(&mut self) {
        self.v_abs_mps = 0.0;
        self.integ.reset(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::{derive, presets};

    fn wall_on() -> SystemParams {
        let mut p = presets::wall();
        p.control.seismic.isolation.enable = true;
        p
    }

    #[test]
    fn cfg_from_t_iso_and_limits() {
        let p = wall_on();
        let t_min = derive::t_min_iso(500.0, 0.2, 100.0);
        assert!(
            (t_min - core::f64::consts::TAU).abs() < 1e-12,
            "WALL T_min = 2π√(500·0.2/100) = 6.283 s from F_cont (plan §2b item 5)"
        );
        assert!(
            (p.control.seismic.isolation.t_iso_s - 1.1 * t_min).abs() < 1e-9,
            "T = max(2, 1.1·T_min)"
        );
        let c = isolate_cfg(&p, PosNm(0));
        let w = 2.0 * core::f64::consts::PI / (1.1 * t_min);
        assert!(
            (f64::from(c.k_n_per_m) - 500.0 * w * w).abs() < 1e-3 * 500.0 * w * w,
            "k = m·ω²"
        );
        assert!(
            (f64::from(c.c_sky_n_s_per_m) - 2.0 * 0.3 * 500.0 * w).abs() < 1e-2,
            "c = 2ζmω"
        );
        assert_eq!(
            (c.stroke_m, c.f_max_n),
            (0.2, 100.0),
            "IsolateLimits envelope (SAF-020)"
        );
        // At full stroke the spring needs F_cont/1.21: saturation-free centring.
        assert!(c.k_n_per_m * c.stroke_m < c.f_max_n);
    }

    #[test]
    fn stroke_governor() {
        let p = wall_on();
        let cfg = isolate_cfg(&p, PosNm(0));
        let mut iso = Isolator::new(&p).expect("isolator");
        // Disabled → 0 N (research tier off in every preset).
        let mut off = Isolator::new(&presets::wall()).expect("isolator");
        assert_eq!(off.step(0.1, 0.1, 1.0, &cfg), IsolateOut::default());
        // Inside the envelope and slow: the skyhook law (IMU still → v_abs = 0).
        let o = iso.step(0.05, 0.01, 0.0, &cfg);
        assert!(!o.saturated);
        assert!(
            (o.f_n + cfg.k_n_per_m * 0.05).abs() < 1e-4,
            "F = −k·x_rel (v_abs 0)"
        );
        // Moving outward fast near the edge: full braking against the motion.
        let o = iso.step(0.18, 0.3, 0.0, &cfg);
        assert!(o.saturated, "governor engaged");
        assert_eq!(o.f_n, -cfg.f_max_n, "−sign(v)·F_max");
        let o = iso.step(-0.18, -0.3, 0.0, &cfg);
        assert_eq!(o.f_n, cfg.f_max_n, "symmetric");
        // Beyond the envelope: push back; returning: the law.
        assert_eq!(iso.step(0.21, 0.0, 0.0, &cfg).f_n, -cfg.f_max_n);
        let back = iso.step(0.21, -0.05, 0.0, &cfg);
        assert!(!back.saturated, "already returning: no push");
        assert!(
            (back.f_n + cfg.k_n_per_m * 0.21).abs() < 1e-4,
            "the law (−k·x within F_max)"
        );
        // The force never exceeds F_max = F_cont (SAF-020).
        for &(x, v) in &[(0.0_f32, 5.0_f32), (0.19, -2.0), (-0.3, 0.4), (0.1, 0.0)] {
            assert!(iso.step(x, v, 50.0, &cfg).f_n.abs() <= cfg.f_max_n);
        }

        // Closed loop, floor at rest, carriage launched outward at v0 with ½mv0² below
        // F_max·s: the governed carriage stays inside the stroke envelope.
        let fs = 20_000.0 / f32::from(p.control.rates.fast_decim);
        let dt = 1.0 / fs;
        for v0 in [0.15_f32, 0.2, 0.25, 0.28] {
            let mut iso = Isolator::new(&p).expect("isolator");
            let (mut x, mut v, mut a) = (0.0_f32, v0, 0.0_f32);
            let mut peak = 0.0_f32;
            let mut engaged = false;
            for _ in 0..(fs as u32 * 20) {
                let o = iso.step(x, v, a, &cfg);
                engaged |= o.saturated;
                a = o.f_n / 500.0;
                v += a * dt;
                x += v * dt;
                peak = peak.max(x.abs());
            }
            assert!(
                peak <= cfg.stroke_m,
                "v0 {v0}: peak {peak:.4} m within the ±{} m envelope (SAF-020)",
                cfg.stroke_m
            );
            if v0 >= 0.25 {
                assert!(engaged, "v0 {v0}: governor engaged");
            }
        }
    }
}
