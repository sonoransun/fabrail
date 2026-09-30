//! Position/velocity cascade with m_hat*a + b_hat*v + ripple feed-forward; iq* = F*/Kf_hat (PHYSICS §16). Owner: A2.
//!
//! Frozen API (W0): [`Cascade::new`], [`Cascade::step`] → force demand F* (N),
//! [`Cascade::iq_ref`] (F*/K̂_f). Velocity PI K_v = m̂·ω_v, K_vi = K_v·ω_v/4 with
//! ω_v = 2π×{8, 6, 3, 2} Hz; position gain K_x = ω_v/4; feed-forward m̂a + b̂v + ripple
//! LUT. In terminus capture the velocity integrator is frozen with bumpless hold (error
//! poles (s + ω_v/2)², ζ = 1) and re-enabled at d_ε (plan §4b).
//!
//! Law (PHYSICS §16; core design §C.4), per fast release with period T:
//!
//! * position loop: v_cmd = v_ref + K_x·(x_ref − x), the error taken as an integer
//!   `PosNm` difference first (PHYSICS §24), then limited to the governor's speed
//!   envelope (`set_speed_limits`, applied by `fast::FastLoop` on planner references);
//! * velocity PI: F = K_v·(v_cmd − v) + I, I ← I + K_vi·(v_cmd − v)·T, with
//!   conditional-integration anti-windup at the thrust limit F_peak;
//! * feed-forward m̂·a_ref + b̂·v_ref (b̂ = b_dist + c_rail, PHYSICS §9); the
//!   acceleration term is dropped while the governor limits the command (it would
//!   push past the envelope);
//! * FOC: i_q* = (F* + F̂_cog(θ_e))/(K̂_f·(1 + r_6·cos 6θ_e)) (PHYSICS §7 ripple
//!   table), clamped to ±i_max.
//!
//! With the integrator frozen the tracking error obeys m·ë + K_v·ė + K_v·K_x·e = 0,
//! i.e. s² + ω_v·s + ω_v²/4 = (s + ω_v/2)²: critically damped (plan §4b).
//!
//! Ripple table convention (PHYSICS §7): thrust = K_f·i_q·(1 + r_6·cos 6θ_e) +
//! F_cog·sin 6θ_e with θ_e from `foc::elec_angle` (exact integer modulo of 2τ_p). W1.5
//! aligned it with the plant's `lsm::thrust`: the cogging force is +F_c·sin 6θ_e, the
//! derivative −dU/dx of U = F_c·τ_p/(6π)·cos 6θ_e (the earlier − sign would have
//! doubled HEAVY's cogging instead of cancelling it).

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{PosNm, fmath};

use crate::foc;
use crate::motion::RefState;

/// 2π, rad.
const TWO_PI: f32 = core::f32::consts::TAU;

/// Reference speed at the freeze below which the held integral is kept unscaled, m/s
/// (design choice: a terminus planned from rest has no cruise resistance to unwind).
const HOLD_V_MIN_MPS: f32 = 1.0e-3;

/// Position/velocity cascade.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Cascade {
    /// ω_v, rad/s.
    pub omega_v_rad_s: f32,
    /// K_x/ω_v.
    pub kx_ratio: f32,
    /// K_vi/(K_v·ω_v).
    pub ki_ratio: f32,
    /// Estimated viscous drag b̂, N·s/m.
    pub b_hat_n_s_per_m: f32,
    /// Thrust constant K̂_f, N/A.
    pub k_f_n_per_a: f32,
    /// Velocity integrator, N.
    pub integ_n: f32,
    /// Fast period, s.
    pub dt_s: f32,
    /// Thrust limit F_peak, N.
    pub f_max_n: f32,
    /// Phase current limit i_max, A.
    pub i_max_a: f32,
    /// Ripple/cogging feed-forward enabled (`motion.ripple_ff`).
    pub ripple_ff: bool,
    /// 6th-harmonic thrust ripple fraction r_6.
    pub ripple_frac: f32,
    /// Cogging amplitude F_cog (iron-core only), N.
    pub cogging_n: f32,
    /// Pole pitch τ_p, nm (electrical angle).
    pub tau_p_nm: i64,
    /// Governor speed envelope toward +x, m/s (≥ 0; +∞ = none).
    pub v_lim_open_mps: f32,
    /// Governor speed envelope toward −x, m/s (≥ 0; +∞ = none).
    pub v_lim_close_mps: f32,
    /// The last step's velocity command was limited by the envelope.
    pub governed: bool,
    /// Electrical angle at the last step, rad.
    pub theta_e: f32,
    /// The integrator is frozen (terminus capture).
    pub frozen: bool,
    /// |v_ref| when the integrator froze, m/s (the held-integral scale reference).
    pub freeze_v_ref_mps: f32,
    /// Scale applied to the held integral this release (1 when not frozen).
    pub hold_scale: f32,
}

impl Cascade {
    /// Cascade from the motion tuning and LSM design (f64 → f32 cast here; never fails:
    /// degenerate parameters give a zero-force cascade, and `validate` rejects them).
    pub fn new(p: &SystemParams) -> Result<Cascade, ParamError> {
        let mo = &p.control.motion;
        let l = &p.design.lsm;
        let dmp = &p.design.damping;
        let dt = fabrail_params::derive::fast_period_s(p) as f32;
        Ok(Cascade {
            // ω_v = 2π·f_v (core design §C.4).
            omega_v_rad_s: (2.0 * core::f64::consts::PI * mo.omega_v_hz) as f32,
            kx_ratio: mo.kx_ratio as f32,
            ki_ratio: mo.ki_ratio as f32,
            // b̂ = b_dist + c_rail (PHYSICS §9 dissipation ledger).
            b_hat_n_s_per_m: (dmp.b_dist_n_s_per_m + dmp.c_rail_n_s_per_m) as f32,
            k_f_n_per_a: l.k_f_n_per_a as f32,
            integ_n: 0.0,
            dt_s: dt,
            f_max_n: l.f_peak_n as f32,
            i_max_a: l.i_max_a as f32,
            ripple_ff: mo.ripple_ff,
            ripple_frac: l.ripple_frac as f32,
            cogging_n: l.cogging_n as f32,
            tau_p_nm: fmath::round(l.tau_p_m * PosNm::NM_PER_M) as i64,
            v_lim_open_mps: f32::INFINITY,
            v_lim_close_mps: f32::INFINITY,
            governed: false,
            theta_e: 0.0,
            frozen: false,
            freeze_v_ref_mps: 0.0,
            hold_scale: 1.0,
        })
    }

    /// Sets the governor's speed envelope for the next steps (magnitudes, m/s; the
    /// velocity command is limited to [−v_close, +v_open]). `fast::FastLoop` refreshes it
    /// every release from `motion::governor` (SAF-001; plan: applied to planner
    /// references).
    pub fn set_speed_limits(&mut self, v_open_mps: f32, v_close_mps: f32) {
        self.v_lim_open_mps = fmath::fmaxf(v_open_mps, 0.0);
        self.v_lim_close_mps = fmath::fmaxf(v_close_mps, 0.0);
    }

    /// One fast release: force demand (N) to track `r` from the measured position and
    /// velocity with payload estimate `m_hat_kg`. `freeze_integrator` holds the
    /// velocity integrator (terminus capture) with a bumpless hold: its value is kept,
    /// so the force stays continuous on freeze and on release.
    pub fn step(
        &mut self,
        r: &RefState,
        x: PosNm,
        v_mps: f32,
        m_hat_kg: f32,
        freeze_integrator: bool,
    ) -> f32 {
        self.theta_e = foc::elec_angle(x, self.tau_p_nm);
        let m = if m_hat_kg.is_finite() && m_hat_kg > 0.0 {
            m_hat_kg
        } else {
            0.0
        };
        let w = self.omega_v_rad_s;
        let k_x = self.kx_ratio * w;
        let k_v = m * w;
        let k_vi = k_v * w * self.ki_ratio;
        // Position loop on the integer difference (exact at any travel; PHYSICS §24).
        let e_x = r.x.diff_m(x);
        let v_raw = r.v_mps + k_x * e_x;
        let v_cmd = fmath::clampf(v_raw, -self.v_lim_close_mps, self.v_lim_open_mps);
        self.governed = v_cmd != v_raw;
        let e_v = v_cmd - v_mps;
        // Feed-forward m̂·a_ref + b̂·v_ref; the acceleration term only when it does not
        // push the command further into a governor limit.
        let a_ff = if self.governed && r.a_mps2 * (v_raw - v_cmd) > 0.0 {
            0.0
        } else {
            m * r.a_mps2
        };
        let ff = a_ff + self.b_hat_n_s_per_m * r.v_mps;
        // Held integral during the terminus capture (W1.5, PHYSICS §16.1): the value at
        // the freeze balanced the velocity-dependent resistance of the cruise (drag,
        // cabinet air), so it is held scaled by |v_ref|/|v_ref at freeze| (≤ 1). A
        // constant hold would keep pushing with the cruise force as the reference slows
        // and lead the exponential by F_I/(K_v·K_x) (≈ 0.1 mm on a Carriage6 DRAWER,
        // bending the semilog line near d_ε). The scale depends on the reference only,
        // so the error dynamics stay (s + ω_v/2)² (FAB-HAP-015); on release the
        // integrator restarts from the scaled value (bumpless).
        let v_ref_abs = fmath::absf(r.v_mps);
        if freeze_integrator {
            if !self.frozen {
                self.frozen = true;
                self.freeze_v_ref_mps = v_ref_abs;
            }
            self.hold_scale = if self.freeze_v_ref_mps > HOLD_V_MIN_MPS {
                fmath::clampf(v_ref_abs / self.freeze_v_ref_mps, 0.0, 1.0)
            } else {
                1.0
            };
        } else if self.frozen {
            self.frozen = false;
            self.integ_n *= self.hold_scale;
            self.hold_scale = 1.0;
        }
        let integ = if self.frozen {
            self.integ_n * self.hold_scale
        } else {
            self.integ_n
        };
        let f_unsat = k_v * e_v + integ + ff;
        let f_max = self.f_max_n;
        let saturated_same_dir = (f_unsat > f_max && e_v > 0.0) || (f_unsat < -f_max && e_v < 0.0);
        if !freeze_integrator && !saturated_same_dir {
            self.integ_n += k_vi * e_v * self.dt_s;
            self.integ_n = fmath::clampf(self.integ_n, -f_max, f_max);
        }
        let integ = if self.frozen {
            self.integ_n * self.hold_scale
        } else {
            self.integ_n
        };
        let f = k_v * e_v + integ + ff;
        if f.is_finite() {
            fmath::clampf(f, -f_max, f_max)
        } else {
            0.0
        }
    }

    /// Modelled thrust multiplier 1 + r_6·cos 6θ_e and cogging force +F_cog·sin 6θ_e at
    /// the last step's angle (PHYSICS §7); (1, 0) when the table is disabled.
    fn ripple_model(&self) -> (f32, f32) {
        if !self.ripple_ff {
            return (1.0, 0.0);
        }
        let th6 = 6.0 * self.theta_e;
        (
            1.0 + self.ripple_frac * fmath::cosf(th6),
            self.cogging_n * fmath::sinf(th6),
        )
    }

    /// Modelled thrust of the measured quadrature current `iq_a` at the last step's
    /// angle: K̂_f·i_q·(1 + r_6·cos 6θ_e) + F_cog·sin 6θ_e, N (the momentum observer's
    /// actuator force, consistent with [`Self::iq_ref`]).
    pub fn thrust_n(&self, iq_a: f32) -> f32 {
        let (mult, cog) = self.ripple_model();
        self.k_f_n_per_a * iq_a * mult + cog
    }

    /// Quadrature current for force `f_n`: i_q* = (F* − F̂_cog)/(K̂_f·(1 + r_6·cos 6θ_e))
    /// (PHYSICS §7; F*/K̂_f with the table off), clamped to ±i_max, A.
    pub fn iq_ref(&self, f_n: f32) -> f32 {
        let (mult, cog) = self.ripple_model();
        let den = self.k_f_n_per_a * mult;
        if den.is_nan() || den <= 0.0 || !f_n.is_finite() {
            return 0.0;
        }
        let i = (f_n - cog) / den;
        let i_max = fmath::fmaxf(self.i_max_a, 0.0);
        fmath::clampf(i, -i_max, i_max)
    }

    /// Bumpless reset of the integrator to produce `f_now_n` (with zero tracking error
    /// and zero feed-forward the next output equals `f_now_n`).
    pub fn reset(&mut self, f_now_n: f32) {
        let f = if f_now_n.is_finite() { f_now_n } else { 0.0 };
        self.integ_n = fmath::clampf(f, -self.f_max_n, self.f_max_n);
        self.governed = false;
        self.frozen = false;
        self.hold_scale = 1.0;
    }

    /// Velocity-loop bandwidth in Hz (ω_v/2π).
    pub fn omega_v_hz(&self) -> f32 {
        self.omega_v_rad_s / TWO_PI
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    fn door() -> Cascade {
        Cascade::new(&presets::door()).unwrap_or_default()
    }

    #[test]
    fn gains_follow_design() {
        let c = door();
        // ω_v = 2π·6 Hz (core design §C.4); K_x/ω_v = K_vi/(K_v·ω_v) = 1/4.
        assert!((c.omega_v_hz() - 6.0).abs() < 1e-5);
        assert_eq!(c.kx_ratio, 0.25);
        assert_eq!(c.ki_ratio, 0.25);
    }

    #[test]
    fn frozen_integrator_is_bumpless() {
        let mut c = door();
        let r = RefState {
            x: PosNm(1_000_000),
            v_mps: 0.1,
            a_mps2: 0.0,
            done: false,
        };
        for _ in 0..100 {
            c.step(&r, PosNm(0), 0.05, 100.0, false);
        }
        let held = c.integ_n;
        let f1 = c.step(&r, PosNm(0), 0.05, 100.0, true);
        assert_eq!(c.integ_n, held, "frozen integrator holds its value");
        let f2 = c.step(&r, PosNm(0), 0.05, 100.0, false);
        assert!((f2 - f1).abs() < 1.0, "no bump on release: {f1} → {f2}");
    }

    #[test]
    fn governor_limit_bounds_command() {
        let mut c = door();
        c.set_speed_limits(0.1, 0.1);
        let r = RefState {
            x: PosNm(50_000_000),
            v_mps: 0.3,
            a_mps2: 0.5,
            done: false,
        };
        let f = c.step(&r, PosNm(0), 0.1, 100.0, true);
        assert!(c.governed);
        // At v = v_lim the P term is zero and the acceleration FF is dropped: only b̂·v_ref.
        assert!((f - c.b_hat_n_s_per_m * 0.3).abs() < 1e-3, "f = {f}");
    }

    #[test]
    fn iq_ref_inverts_thrust() {
        let c = door();
        let f = 42.0;
        let i = c.iq_ref(f);
        assert!((c.thrust_n(i) - f).abs() < 1e-4);
        assert!(c.iq_ref(1e9) <= c.i_max_a);
        assert_eq!(c.iq_ref(f32::NAN), 0.0);
    }
}
