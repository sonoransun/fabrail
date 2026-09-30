//! Passivity observer and controller (Hannaford-Ryu) and the K <= 2b/(T + 2*T_d) bound (PHYSICS §17; HAP-009). Owner: A2.
//!
//! Frozen API (W0): [`PassivityController::new`], [`PassivityController::step`] →
//! [`PassivityOut`], and [`k_max`] (Colgate–Schenkel bound K_max = 2b/(T + 2·T_d),
//! plan §2b item 19; f32 twin of `params::derive::passivity_k_max`). The controller's
//! variable damper is limited to α_max = 5·b_dist. Tests: `stiffness_bound`,
//! `pc_bounds_energy`.
//!
//! **Bound** (Colgate & Schenkel 1997; plan §2b item 19). A virtual spring K rendered by
//! a sampled-data loop with period T and delay T_d on a device with physical damping b
//! is passive iff b > K·(T/2 + T_d), i.e. K < 2b/(T + 2T_d). The presets render
//! K_v = 0.3·2b/T with T_d = T, below the bound 2b/(3T) (`validate`).
//!
//! **Observer and controller** (Hannaford & Ryu 2002), per fast release at the haptic
//! port with port force f (the force the user/environment applies into the device) and
//! port velocity v:
//!
//! ```text
//! E ← E + f·v·T                 (energy delivered into the device; passive ⇔ E ≥ 0)
//! if E < 0 and v ≠ 0:  α = min(−E/(T·v²), α_max),  f_pc = −α·v,  E ← E + α·v²·T
//! ```
//!
//! `f_pc` is added to the rendered force (the admittance's F_eff in `fast::FastLoop`),
//! so the controller dissipates exactly the energy the loop generated, one release
//! late, up to α_max = `passivity_alpha_ratio`·b_dist (5·b_dist).
//!
//! **Energy reference** (Ryu, Preusche, Hannaford & Hirzinger 2005, reference energy
//! following). The plain observer sees generated energy only once it exceeds what the
//! virtual element currently stores, so the controller acts in bursts at the storage
//! minima and, with α capped, lets E dip (−2 mJ in the 3·K_max test). When the caller
//! knows the virtual storage W ≥ 0 (½M_v·v² + ½K_v·e² of the admittance) it passes it
//! with [`PassivityController::set_stored_energy`] and the controller acts on E − W < 0
//! instead: the generated energy is removed as it appears, and E ≥ W ≥ 0 follows. With
//! W = 0 this is the classic Hannaford–Ryu controller.

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::fmath;

/// Speeds below this do not activate the damper (α = −E/(T·v²) would blow up), m/s.
const V_EPS_MPS: f32 = 1.0e-6;

/// Passivity controller output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PassivityOut {
    /// Damping force to add, N.
    pub f_pc_n: f32,
    /// Variable damper α, N·s/m (≤ α_max).
    pub alpha_n_s_per_m: f32,
    /// Observed port energy, J.
    pub energy_j: f32,
}

/// Hannaford–Ryu passivity observer/controller.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PassivityController {
    /// α_max, N·s/m.
    pub alpha_max_n_s_per_m: f32,
    /// Observed energy, J.
    pub energy_j: f32,
    /// Fast period, s.
    pub dt_s: f32,
    /// Energy stored in the virtual element (reference), J (0 = classic controller).
    pub stored_j: f32,
}

impl PassivityController {
    /// Controller from the haptic tuning and b_dist (never fails).
    pub fn new(p: &SystemParams) -> Result<PassivityController, ParamError> {
        Ok(PassivityController {
            // α_max = 5·b_dist (core design §C.4).
            alpha_max_n_s_per_m: (p.control.haptic.passivity_alpha_ratio
                * p.design.damping.b_dist_n_s_per_m) as f32,
            energy_j: 0.0,
            dt_s: fabrail_params::derive::fast_period_s(p) as f32,
            stored_j: 0.0,
        })
    }

    /// Energy currently stored in the rendered virtual element (≥ 0), J; the controller
    /// then keeps E_obs ≥ W (module docs). Non-finite or negative values count as 0.
    pub fn set_stored_energy(&mut self, w_j: f32) {
        self.stored_j = if w_j.is_finite() {
            fmath::fmaxf(w_j, 0.0)
        } else {
            0.0
        };
    }

    /// One fast release from the port force (N) and velocity (m/s). With `enabled`
    /// false the observer still integrates the energy but no damping is added.
    pub fn step(&mut self, f_n: f32, v_mps: f32, enabled: bool) -> PassivityOut {
        let dt = self.dt_s;
        if !(f_n.is_finite() && v_mps.is_finite()) || dt <= 0.0 {
            return PassivityOut {
                energy_j: self.energy_j,
                ..PassivityOut::default()
            };
        }
        self.energy_j += f_n * v_mps * dt;
        let mut out = PassivityOut::default();
        let deficit = self.energy_j - self.stored_j;
        if enabled && deficit < 0.0 && fmath::absf(v_mps) > V_EPS_MPS {
            let alpha = fmath::fminf(
                -deficit / (dt * v_mps * v_mps),
                fmath::fmaxf(self.alpha_max_n_s_per_m, 0.0),
            );
            out.alpha_n_s_per_m = alpha;
            out.f_pc_n = -alpha * v_mps;
            // The damper's dissipation is part of the port from now on.
            self.energy_j += alpha * v_mps * v_mps * dt;
        }
        out.energy_j = self.energy_j;
        out
    }

    /// Clears the observed energy and the storage reference.
    pub fn reset(&mut self) {
        self.energy_j = 0.0;
        self.stored_j = 0.0;
    }
}

/// Stiffness bound K_max = 2b/(T + 2·T_d) for damping b (N·s/m), sample period T (s)
/// and delay T_d (s), N/m (+∞ for a non-positive denominator).
pub fn k_max(b_n_s_per_m: f32, t_s: f32, t_d_s: f32) -> f32 {
    let den = t_s + 2.0 * t_d_s;
    if den > 0.0 {
        2.0 * b_n_s_per_m / den
    } else {
        f32::INFINITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::{derive, presets};

    /// Mass m with physical damping b against a sampled virtual spring K (ZOH, period T,
    /// delay d samples), optionally with the passivity controller at the port. The mass
    /// starts at the spring's rest point moving at 30 mm/s, so the virtual element holds
    /// no energy initially (the observer's E_obs = 0 reference). Returns (mechanical
    /// energy ½mv² + ½Kx² at start, at the end, minimum observed E_obs).
    fn wall_run(
        m: f64,
        b: f64,
        k: f64,
        t: f64,
        d: usize,
        secs: f64,
        pc: Option<&mut PassivityController>,
    ) -> (f64, f64, f64) {
        let sub = 10;
        let h = t / sub as f64;
        let (mut x, mut v) = (0.0_f64, 0.03_f64);
        let mut hist = vec![x; d + 1];
        let e0 = 0.5 * m * v * v;
        let mut e_min = 0.0_f64;
        let mut pc = pc;
        let n = (secs / t) as usize;
        for _ in 0..n {
            hist.rotate_right(1);
            hist[0] = x;
            let x_d = hist[d];
            let mut f_wall = -k * x_d;
            if let Some(c) = pc.as_deref_mut() {
                // Storage reference: the rendered spring's energy ½K·x² at this sample.
                c.set_stored_energy((0.5 * k * x * x) as f32);
                // Port force into the virtual wall is −f_wall (the mass pushes on it).
                let o = c.step((-f_wall) as f32, v as f32, true);
                f_wall += f64::from(o.f_pc_n);
                e_min = e_min.min(f64::from(o.energy_j));
            }
            for _ in 0..sub {
                let a = (f_wall - b * v) / m;
                v += a * h;
                x += v * h;
            }
        }
        (e0, 0.5 * m * v * v + 0.5 * k * x * x, e_min)
    }

    /// K_max = 2b/(T + 2T_d) matches the f64 twin; every preset's rendered K_v
    /// (k_virtual_frac·2b/T) is below the bound with T_d = T; and a sampled virtual
    /// spring on the DRAWER mass decays at 0.8·K_max but grows at 3·K_max (the
    /// violation reproduced). (A) f32 rel 1e-6; (I) energy trend, HAP-009, PHYSICS §17.
    // req: FAB-HAP-009
    #[test]
    fn stiffness_bound() {
        for (b, t, td) in [
            (2.0_f32, 1e-4_f32, 1e-4_f32),
            (5.0, 1e-4, 0.0),
            (100.0, 2e-4, 2e-4),
        ] {
            let k = f64::from(k_max(b, t, td));
            let k64 = derive::passivity_k_max(f64::from(b), f64::from(t), f64::from(td));
            assert!(
                ((k - k64) / k64).abs() <= 1e-6,
                "{k} vs {k64} ((A) rel 1e-6)"
            );
        }
        assert_eq!(k_max(1.0, 0.0, 0.0), f32::INFINITY);
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let b = p.design.damping.b_dist_n_s_per_m;
            let t = derive::fast_period_s(&p);
            let k_v = p.control.haptic.k_virtual_frac * 2.0 * b / t;
            let bound = derive::passivity_k_max(b, t, t);
            assert!(
                k_v < bound,
                "{:?}: K_v {k_v} ≥ K_max {bound} (HAP-009)",
                p.design.class
            );
        }
        // DRAWER: m 48 kg, b 2 N·s/m, T 100 µs, one-sample delay T_d = T.
        let (m, b, t) = (48.0, 2.0, 1e-4);
        let km = derive::passivity_k_max(b, t, t);
        let (e0, e_end, _) = wall_run(m, b, 0.8 * km, t, 1, 20.0, None);
        assert!(
            e_end < 0.95 * e0,
            "0.8·K_max must be dissipative: {e0} → {e_end}"
        );
        let (e0, e_end, _) = wall_run(m, b, 3.0 * km, t, 1, 20.0, None);
        assert!(
            e_end > 1.5 * e0,
            "3·K_max must reproduce the violation: {e0} → {e_end}"
        );
    }

    /// With the passivity controller on (energy-reference form, α ≤ 5·b_dist), the
    /// 3·K_max spring keeps E_obs ≥ −1 mJ and its energy bounded; with it off (observer
    /// only) E_obs falls below −1 mJ. (S) E_obs ≥ −1 mJ, HAP-009, PHYSICS §17.
    // req: FAB-HAP-009
    #[test]
    fn pc_bounds_energy() {
        let p = presets::drawer();
        let (m, b, t) = (48.0, 2.0, 1e-4);
        let km = derive::passivity_k_max(b, t, t);
        let mut pc = PassivityController::new(&p).unwrap_or_default();
        assert_eq!(
            pc.alpha_max_n_s_per_m, 10.0,
            "α_max = 5·b_dist (core design §C.4)"
        );
        let (e0, e_end, e_min) = wall_run(m, b, 3.0 * km, t, 1, 20.0, Some(&mut pc));
        assert!(
            e_min >= -1.0e-3,
            "E_obs {e_min} J < −1 mJ with the PC on (HAP-009)"
        );
        assert!(
            e_end <= e0 * 1.01,
            "energy grew with the PC on: {e0} → {e_end}"
        );
        // Observer only: the generated energy shows up as E_obs < −1 mJ.
        let mut po = PassivityController::new(&p).unwrap_or_default();
        let mut e_obs_min = 0.0_f64;
        {
            let (mut x, mut v) = (0.0_f64, 0.03_f64);
            let k = 3.0 * km;
            let mut x_d = x;
            for _ in 0..200_000 {
                let f_wall = -k * x_d;
                x_d = x;
                let o = po.step((-f_wall) as f32, v as f32, false);
                e_obs_min = e_obs_min.min(f64::from(o.energy_j));
                for _ in 0..10 {
                    v += (f_wall - b * v) / m * (t / 10.0);
                    x += v * (t / 10.0);
                }
            }
        }
        assert!(
            e_obs_min < -1.0e-3,
            "PC off: E_obs {e_obs_min} should fall below −1 mJ"
        );
    }
}
