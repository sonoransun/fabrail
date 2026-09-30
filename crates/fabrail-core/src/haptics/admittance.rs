//! Admittance M_v*x'' + B_v*x' + K_v*(x - x0) = F_hat + F_eff integrated in a PosAccum (PHYSICS §17). Owner: A2.
//!
//! Frozen API (W0): [`Admittance::new`], [`Admittance::step`] → `RefState`,
//! [`Admittance::reset`]. The rendered mass is M_v = m/assist ratio (1.5 / 3 / 5 / 5;
//! HAP-002); the position integrates in a `PosAccum` so it does not drift over the
//! track (PHYSICS §24). Rendered lightness does not reduce impact energy (non-claim 6):
//! the governor and monitor act on m_gov and rated mass.
//!
//! Integration (PHYSICS §17), per fast release T with F = F̂_h + F_eff − K_v·(x − x0):
//! the damper is implicit and the spring semi-implicit (symplectic Euler),
//!
//! ```text
//! v ← (v + T·F/M_v) / (1 + T·B_v/M_v)      (unconditionally dissipative in B_v)
//! v ← clamp(v, −v_lim,close, +v_lim,open)  (governor: removes energy only)
//! x ← x + v·T                             (PosAccum: integer base + sub-nm fraction)
//! ```
//!
//! The spring K_v ≤ K_max = 2·b_dist/(T + 2T_d) (HAP-009, `haptics::passivity`) is
//! stable here for ω_v·T = √(K_v/M_v)·T ≪ 2. The governor limit (`set_speed_limits`,
//! refreshed by `fast::FastLoop` from `motion::governor`) only ever reduces |v|, so under
//! hand-driven motion the governor adds damping and never injects energy (PHYSICS §16).
//! The spring anchor, detents and the reference are `PosNm`; only differences are cast
//! to f32, so the rendering is identical anywhere on a 15 m track.

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{PosAccum, PosNm, fmath};

use crate::motion::RefState;
use crate::msg::HapticProfile;

/// Virtual mass–damper–spring.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Admittance {
    /// Integrated reference position.
    pub x: PosAccum,
    /// Reference velocity, m/s.
    pub v_mps: f32,
    /// Fast period, s.
    pub dt_s: f32,
    /// Reference acceleration at the last step, m/s².
    pub a_mps2: f32,
    /// Governor speed limit toward +x (≥ 0; +∞ = none), m/s.
    pub v_lim_open_mps: f32,
    /// Governor speed limit toward −x (≥ 0; +∞ = none), m/s.
    pub v_lim_close_mps: f32,
    /// The governor limited the velocity at the last step.
    pub governed: bool,
}

impl Admittance {
    /// Admittance from the haptic tuning (the profile itself arrives per release in the
    /// `Directive`); starts at the origin at rest.
    pub fn new(p: &SystemParams) -> Result<Admittance, ParamError> {
        Ok(Admittance {
            dt_s: fabrail_params::derive::fast_period_s(p) as f32,
            v_lim_open_mps: f32::INFINITY,
            v_lim_close_mps: f32::INFINITY,
            ..Admittance::default()
        })
    }

    /// Sets the governor's speed envelope (magnitudes, m/s) for the next steps.
    pub fn set_speed_limits(&mut self, v_open_mps: f32, v_close_mps: f32) {
        self.v_lim_open_mps = fmath::fmaxf(v_open_mps, 0.0);
        self.v_lim_close_mps = fmath::fmaxf(v_close_mps, 0.0);
    }

    /// One fast release with hand force `f_hand_n` and effect force `f_eff_n`.
    ///
    /// A non-positive or non-finite M_v, or non-finite forces, hold the reference at
    /// rest (no energy is injected).
    pub fn step(&mut self, f_hand_n: f32, f_eff_n: f32, prof: &HapticProfile) -> RefState {
        let m = prof.m_v_kg;
        let dt = self.dt_s;
        let ok = m.is_finite() && m > 0.0 && f_hand_n.is_finite() && f_eff_n.is_finite();
        if !ok || dt <= 0.0 {
            self.v_mps = 0.0;
            self.a_mps2 = 0.0;
            self.governed = false;
            return self.state();
        }
        // Spring on the integer difference to the anchor (PHYSICS §24), sub-nm fraction
        // included.
        let e = self.x.pos().diff_m(prof.x0) + self.x.frac_nm * 1.0e-9;
        let k = fmath::fmaxf(prof.k_v_n_per_m, 0.0);
        let b = fmath::fmaxf(prof.b_v_n_s_per_m, 0.0);
        let f = f_hand_n + f_eff_n - k * e;
        let v_old = self.v_mps;
        let v_new = (v_old + dt * f / m) / (1.0 + dt * b / m);
        let v_lim = fmath::clampf(v_new, -self.v_lim_close_mps, self.v_lim_open_mps);
        self.governed = v_lim != v_new;
        self.v_mps = if v_lim.is_finite() { v_lim } else { 0.0 };
        self.a_mps2 = (self.v_mps - v_old) / dt;
        self.x.advance(self.v_mps, dt);
        self.state()
    }

    /// The current reference state.
    pub fn state(&self) -> RefState {
        RefState {
            x: self.x.pos(),
            v_mps: self.v_mps,
            a_mps2: self.a_mps2,
            done: false,
        }
    }

    /// Re-anchors at the measured state (bumpless entry into admittance).
    pub fn reset(&mut self, x: PosNm, v_mps: f32) {
        self.x.reset(x);
        self.v_mps = if v_mps.is_finite() { v_mps } else { 0.0 };
        self.a_mps2 = 0.0;
        self.governed = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    fn prof(m_v: f32, b_v: f32, k_v: f32) -> HapticProfile {
        HapticProfile {
            m_v_kg: m_v,
            b_v_n_s_per_m: b_v,
            k_v_n_per_m: k_v,
            ..HapticProfile::default()
        }
    }

    #[test]
    fn free_mass_follows_newton() {
        let mut a = Admittance::new(&presets::door()).unwrap_or_default();
        let pr = prof(100.0 / 3.0, 0.0, 0.0);
        // 10 N for 1 s on M_v = 33.3 kg: v = 0.3 m/s, x = 0.15 m (Euler, T = 100 µs).
        let n = (1.0 / a.dt_s).round() as usize;
        for _ in 0..n {
            a.step(10.0, 0.0, &pr);
        }
        assert!((a.v_mps - 0.3).abs() < 1e-4, "v = {}", a.v_mps);
        let x = a.x.pos_m_f64();
        assert!((x - 0.15).abs() < 1e-4, "x = {x}");
    }

    #[test]
    fn governor_only_removes_energy() {
        let mut a = Admittance::new(&presets::door()).unwrap_or_default();
        a.set_speed_limits(0.1, 0.1);
        let pr = prof(20.0, 5.0, 0.0);
        let mut e_prev = 0.0;
        for _ in 0..20_000 {
            let r = a.step(50.0, 0.0, &pr);
            assert!(r.v_mps <= 0.1 + 1e-7);
            let e = 0.5 * pr.m_v_kg * r.v_mps * r.v_mps;
            assert!(e >= e_prev - 1e-9 || a.governed);
            e_prev = e;
        }
        assert!(a.governed);
    }

    #[test]
    fn spring_is_offset_invariant() {
        // The same motion 12 m further along the track is bit-identical (PHYSICS §24).
        let run = |base: i64| {
            let mut a = Admittance::new(&presets::wall()).unwrap_or_default();
            a.reset(PosNm(base + 5_000_000), 0.0);
            let mut pr = prof(100.0, 450.0, 2_000.0);
            pr.x0 = PosNm(base);
            let mut out = [0u32; 3];
            for _ in 0..5_000 {
                let r = a.step(3.0, 0.0, &pr);
                out = [r.v_mps.to_bits(), r.a_mps2.to_bits(), (r.x.0 - base) as u32];
            }
            out
        };
        assert_eq!(run(0), run(12_000_000_000));
    }

    #[test]
    fn invalid_profile_holds() {
        let mut a = Admittance::new(&presets::drawer()).unwrap_or_default();
        a.reset(PosNm(7), 0.2);
        let r = a.step(10.0, 0.0, &prof(0.0, 1.0, 0.0));
        assert_eq!((r.x, r.v_mps), (PosNm(7), 0.0));
    }
}
