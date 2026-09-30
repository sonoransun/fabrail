//! Energy/momentum/braking envelope v_lim(x) with m_gov = max(m_rated, 1.05*m_hat) (PHYSICS §16; SAF-001, SAF-007). Owner: A2.
//!
//! Frozen API (W0): [`Governor::new`], [`Governor::m_gov_kg`], [`Governor::v_lim_mps`]
//! and [`Governor::e_kin_j`].
//!
//! v_lim(x) = min(V_mode, √(2·0.90·E/m_gov), 0.95·p_max/m_gov,
//! √(2·a_brk·(x_obs − x − margin))) − a_brk·t_react (the 0.90/0.95 planning margins
//! against the monitor's caps are Wave 2 lead decision D4), with a_brk = 2·a_max and
//! t_react = 2 fast periods; m_gov =
//! max(m_rated, 1.05·m̂) (core design F19). A commanded terminus is observed at
//! x_obs = terminus + overtravel so the exponential tail is never clipped (plan §4b).
//! Tests: `rated_mass_invariance` (proptest), `braking_envelope`, `terminus_not_clipped`.
//!
//! Notes (PHYSICS §16; plan §2b and §4b):
//!
//! * The envelope uses the rated mass (the larger of `design.carriage.m_rated_kg` and the
//!   monitor's `safety.mass_rated_kg`); m̂ can only raise m_gov (F19), so under-reporting
//!   m̂ (sabotage to 0.5·m) changes nothing.
//! * Braking term: after the reaction time t_react at v_lim the carriage brakes at
//!   a_brk; the distance used is v_lim·t_react + v_lim²/(2a_brk) = d − a_brk·t_react²/2
//!   ≤ d = x_obs − x − margin, so it always stops before x_obs − margin.
//! * `fast::FastLoop` applies v_lim to planner references (the cascade's velocity
//!   command) and to the admittance output, where limiting the virtual velocity only
//!   removes energy: under hand-driven motion the governor only adds damping.
//! * `a_brk` uses the live `LiveLimits::a_max_mps2` (sensed vs unsensed), falling back
//!   to `control.motion.a_max_mps2` when the live value is not set.

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{PosNm, TravelDir, fmath};

use crate::msg::LiveLimits;

/// Fraction of the kinetic-energy cap the governor plans at (Wave 2 lead decision D4):
/// v_e = √(2·0.90·E/m_gov) ≈ 0.949·√(2E/m_gov). The independent monitor trips at the
/// cap itself, so tracking overshoot, rail-profile disturbances and hand assistance get
/// a 10 % energy (≈ 5 % speed) margin instead of the W1.5 ≈ 0.3 mm/s.
pub const GOV_KE_PLAN_FRAC: f32 = 0.90;

/// Fraction of the momentum cap the governor plans at (D4): v_p = 0.95·p_max/m_gov.
pub const GOV_P_PLAN_FRAC: f32 = 0.95;

/// Start of the governor-damping fade as a fraction of the full energy/momentum cap
/// speed (D4; design choice): a drive force along v is unrestricted below 0.975·v_cap and
/// fades linearly to zero at v_cap, where the monitor trips while the drive does positive
/// work. The band sits above the 0.949·v_cap planning cap, so tracking and rail noise
/// around a cruise at the planning cap never touch it (a fade from the planning cap
/// chattered the DRAWER drive at 0.25 m/s on the rail traverse).
pub const GOV_DRIVE_FADE_FRAC: f32 = 0.975;

/// Energy/momentum/braking governor.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Governor {
    /// Rated mass, kg.
    pub m_rated_kg: f32,
    /// m_gov ratio on m̂ (1.05).
    pub m_gov_ratio: f32,
    /// Energy cap E, J.
    pub e_max_j: f32,
    /// Momentum cap p_max (0 = not applicable), kg·m/s.
    pub p_max_kgmps: f32,
    /// Braking deceleration a_brk at the preset a_max (fallback), m/s².
    pub a_brk_mps2: f32,
    /// Reaction time t_react, s.
    pub t_react_s: f32,
    /// Stopping margin, m.
    pub margin_m: f32,
    /// Braking ratio a_brk/a_max (2).
    pub a_brk_ratio: f32,
}

impl Governor {
    /// Governor from the safety limits and motion/supervisor tuning (never fails;
    /// degenerate parameters give v_lim = 0).
    pub fn new(p: &SystemParams) -> Result<Governor, ParamError> {
        let mo = &p.control.motion;
        let t_fast = fabrail_params::derive::fast_period_s(p);
        Ok(Governor {
            // Rated mass, never m̂ (F19); the larger of the design and monitor copies.
            m_rated_kg: fmath::fmax(p.design.carriage.m_rated_kg, p.safety.mass_rated_kg) as f32,
            m_gov_ratio: p.control.supervisor.m_gov_ratio as f32,
            e_max_j: p.safety.e_max_j as f32,
            p_max_kgmps: p.safety.p_max_kgmps as f32,
            // a_brk = 2·a_max (core design §C.4, SAF-007).
            a_brk_mps2: (mo.a_brk_ratio * mo.a_max_mps2) as f32,
            // t_react = 2 fast periods (core design §C.4, SAF-007).
            t_react_s: (f64::from(mo.t_react_fast_periods) * t_fast) as f32,
            margin_m: mo.gov_margin_m as f32,
            a_brk_ratio: mo.a_brk_ratio as f32,
        })
    }

    /// m_gov = max(m_rated, ratio·m̂), kg (F19). A non-finite m̂ is ignored.
    pub fn m_gov_kg(&self, m_hat_kg: f32) -> f32 {
        // fmaxf returns the non-NaN operand, so a NaN m̂ yields m_rated.
        fmath::fmaxf(self.m_rated_kg, self.m_gov_ratio * m_hat_kg)
    }

    /// Braking deceleration for the live limits: a_brk_ratio·a_max(live), or the preset
    /// a_brk when the live value is not set, m/s².
    pub fn a_brk_live_mps2(&self, limits: &LiveLimits) -> f32 {
        if limits.a_max_mps2 > 0.0 {
            self.a_brk_ratio * limits.a_max_mps2
        } else {
            self.a_brk_mps2
        }
    }

    /// Speed limit at `x` moving in `dir` under `limits`, m/s (≥ 0).
    pub fn v_lim_mps(&self, x: PosNm, dir: TravelDir, m_hat_kg: f32, limits: &LiveLimits) -> f32 {
        let m_gov = self.m_gov_kg(m_hat_kg);
        if m_gov.is_nan() || m_gov <= 0.0 {
            return 0.0;
        }
        let (v_mode, x_obs) = match dir {
            TravelDir::Open => (limits.v_open_mps, limits.x_obs_open),
            TravelDir::Close => (limits.v_close_mps, limits.x_obs_close),
        };
        // Energy cap √(2·0.90·E/m_gov) (PHYSICS §16; SAF-001; D4 planning margin).
        let v_e = fmath::sqrtf(2.0 * GOV_KE_PLAN_FRAC * fmath::fmaxf(self.e_max_j, 0.0) / m_gov);
        // Momentum cap 0.95·p_max/m_gov where applicable (A156.10; 0 = not applicable;
        // D4 planning margin).
        let v_p = if self.p_max_kgmps > 0.0 {
            GOV_P_PLAN_FRAC * self.p_max_kgmps / m_gov
        } else {
            f32::INFINITY
        };
        // Braking envelope √(2·a_brk·(x_obs − x − margin)), distance along `dir` taken
        // as an integer difference first (PHYSICS §24).
        let a_brk = self.a_brk_live_mps2(limits);
        let d = dir.sign() * x_obs.diff_m(x) - self.margin_m;
        let v_b = fmath::sqrtf(2.0 * fmath::fmaxf(a_brk, 0.0) * fmath::fmaxf(d, 0.0));
        let v = fmath::fminf(fmath::fminf(v_mode, v_e), fmath::fminf(v_p, v_b))
            - fmath::fmaxf(a_brk, 0.0) * self.t_react_s;
        if v.is_finite() {
            fmath::fmaxf(v, 0.0)
        } else {
            0.0
        }
    }

    /// Planning cap without the braking envelope: min(V_mode, √(2·0.90·E/m_gov),
    /// 0.95·p_max/m_gov) in direction `dir`, m/s (D4).
    pub fn v_plan_cap_mps(&self, dir: TravelDir, m_hat_kg: f32, limits: &LiveLimits) -> f32 {
        let m_gov = self.m_gov_kg(m_hat_kg);
        if m_gov.is_nan() || m_gov <= 0.0 {
            return 0.0;
        }
        let v_mode = match dir {
            TravelDir::Open => limits.v_open_mps,
            TravelDir::Close => limits.v_close_mps,
        };
        let (v_e, v_p) = self.caps_at(m_gov, GOV_KE_PLAN_FRAC, GOV_P_PLAN_FRAC);
        fmath::fmaxf(fmath::fminf(v_mode, fmath::fminf(v_e, v_p)), 0.0)
    }

    /// Energy and momentum speed caps at m_gov with fractions (ke, p) of E and p_max
    /// (momentum ∞ when not applicable), m/s.
    fn caps_at(&self, m_gov: f32, ke_frac: f32, p_frac: f32) -> (f32, f32) {
        let v_e = fmath::sqrtf(2.0 * ke_frac * fmath::fmaxf(self.e_max_j, 0.0) / m_gov);
        let v_p = if self.p_max_kgmps > 0.0 {
            p_frac * self.p_max_kgmps / m_gov
        } else {
            f32::INFINITY
        };
        (v_e, v_p)
    }

    /// Governor damping (D4): the fraction k ∈ [0, 1] of a drive force ALONG the velocity
    /// `v_mps` that is still allowed. k = 1 up to [`GOV_DRIVE_FADE_FRAC`] of the full
    /// energy/momentum cap v_cap = min(√(2E/m_gov), p_max/m_gov), falling linearly to 0 at
    /// v_cap (where the independent monitor trips while the drive does positive work);
    /// braking forces are never scaled. Above the caps the drive may only remove energy,
    /// so hand-thrown or rail-disturbance excess is met by damping.
    pub fn drive_scale(&self, v_mps: f32, m_hat_kg: f32, _limits: &LiveLimits) -> f32 {
        if !v_mps.is_finite() {
            return 0.0;
        }
        let m_gov = self.m_gov_kg(m_hat_kg);
        if m_gov.is_nan() || m_gov <= 0.0 {
            return 0.0;
        }
        let (v_e, v_p) = self.caps_at(m_gov, 1.0, 1.0);
        let hi = fmath::fminf(v_e, v_p);
        let lo = GOV_DRIVE_FADE_FRAC * hi;
        let s = fmath::absf(v_mps);
        if s <= lo {
            1.0
        } else if hi > lo {
            fmath::clampf((hi - s) / (hi - lo), 0.0, 1.0)
        } else {
            0.0
        }
    }

    /// Governor kinetic energy ½·m_gov·v², J.
    pub fn e_kin_j(&self, m_hat_kg: f32, v_mps: f32) -> f32 {
        0.5 * self.m_gov_kg(m_hat_kg) * v_mps * v_mps
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::{derive, presets};
    use proptest::prelude::*;

    /// Live limits at the unsensed caps with observation points far away (so only the
    /// energy/momentum/mode terms bind unless a test moves them).
    fn cap_limits(p: &SystemParams) -> LiveLimits {
        LiveLimits {
            v_open_mps: p.safety.v_open_max_mps as f32,
            v_close_mps: p.safety.v_close_max_mps as f32,
            a_max_mps2: p.control.motion.a_max_mps2 as f32,
            x_obs_open: PosNm::from_m_f64(100.0),
            x_obs_close: PosNm::from_m_f64(-100.0),
            ..LiveLimits::default()
        }
    }

    fn all() -> [SystemParams; 4] {
        [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ]
    }

    // req: FAB-SAF-001
    proptest! {
        #![proptest_config(ProptestConfig::with_cases(256))]
        /// v_lim never depends on m̂ ≤ m_rated/1.05 (bit-identical; sabotage 0.5·m changes
        /// nothing) and never increases with m̂ above it. (E) bit-exact, F19, SAF-001.
        #[test]
        fn rated_mass_invariance(
            class in 0usize..4,
            m_frac in 0.0f64..0.999,
            m_over in 1.0f64..3.0,
            x_mm in -500.0f64..15_000.0,
            obs_mm in -600.0f64..16_000.0,
            open in any::<bool>(),
            v_scale in 0.0f64..2.0,
        ) {
            let p = all()[class];
            let g = Governor::new(&p).unwrap_or_default();
            let mut lim = cap_limits(&p);
            lim.v_open_mps *= v_scale as f32;
            lim.v_close_mps *= v_scale as f32;
            lim.x_obs_open = PosNm::from_m_f64(obs_mm * 1e-3);
            lim.x_obs_close = PosNm::from_m_f64(-obs_mm * 1e-3);
            let dir = if open { TravelDir::Open } else { TravelDir::Close };
            let x = PosNm::from_m_f64(x_mm * 1e-3);
            let m_r = p.design.carriage.m_rated_kg;
            let v_ref = g.v_lim_mps(x, dir, 0.0, &lim);
            // Any m̂ up to m_rated/1.05, including the 0.5·m sabotage.
            let m_low = (m_frac * m_r / 1.05) as f32;
            for m_hat in [m_low, (0.5 * m_r) as f32, 1.0e-3, f32::NAN] {
                let v = g.v_lim_mps(x, dir, m_hat, &lim);
                prop_assert_eq!(v.to_bits(), v_ref.to_bits(),
                    "m̂ = {} changed v_lim ((E) bit-exact, F19, SAF-001)", m_hat);
            }
            // Above the rated mass m̂ can only lower the envelope.
            let v_hi = g.v_lim_mps(x, dir, (m_over * m_r) as f32, &lim);
            prop_assert!(v_hi <= v_ref, "m̂ above m_rated raised v_lim (F19)");
            // And the envelope never exceeds the rated-mass energy/momentum caps.
            let cap = derive::unsensed_speed_cap(m_r, p.safety.e_max_j, p.safety.p_max_kgmps);
            prop_assert!(f64::from(v_ref) <= cap * (1.0 + 1e-6),
                "v_lim {} above the rated-mass cap {} (SAF-001)", v_ref, cap);
        }
    }

    /// v_lim uses a_brk = 2·a_max and t_react = 2 fast periods, stops before x_obs −
    /// margin, and is 0 inside the margin. (E) f32 rel 1e-5, SAF-007, PHYSICS §16.
    // req: FAB-SAF-007
    #[test]
    fn braking_envelope() {
        for p in all() {
            let g = Governor::new(&p).unwrap_or_default();
            let a_max = p.control.motion.a_max_mps2;
            let t_fast = derive::fast_period_s(&p);
            assert_eq!(
                g.a_brk_mps2,
                (2.0 * a_max) as f32,
                "a_brk = 2·a_max (SAF-007)"
            );
            assert_eq!(
                g.t_react_s,
                (2.0 * t_fast) as f32,
                "t_react = 2 T_fast (SAF-007)"
            );
            let mut lim = cap_limits(&p);
            // Mode cap high so the braking term binds near x_obs.
            lim.v_open_mps = 10.0;
            lim.v_close_mps = 10.0;
            lim.x_obs_open = PosNm::from_m_f64(1.0);
            lim.x_obs_close = PosNm::from_m_f64(-1.0);
            let m = 0.0;
            let margin = p.control.motion.gov_margin_m;
            let a_brk = 2.0 * a_max;
            let t_r = 2.0 * t_fast;
            // Planning caps at 0.90·E and 0.95·p_max (D4).
            let v_e = derive::unsensed_speed_cap(
                p.design.carriage.m_rated_kg,
                f64::from(GOV_KE_PLAN_FRAC) * p.safety.e_max_j,
                f64::from(GOV_P_PLAN_FRAC) * p.safety.p_max_kgmps,
            );
            for d_mm in [0.5_f64, 1.0, 3.0, 5.0, 10.0, 20.0] {
                let d = d_mm * 1e-3;
                let x = PosNm::from_m_f64(1.0 - margin - d);
                let want_b = (2.0 * a_brk * d).sqrt();
                let want = (want_b.min(v_e) - a_brk * t_r).max(0.0);
                let got = f64::from(g.v_lim_mps(x, TravelDir::Open, m, &lim));
                assert!(
                    (got - want).abs() <= 1e-5 * want.max(1e-3),
                    "{:?} d = {d_mm} mm: v_lim {got} vs {want} ((E) f32 rel 1e-5, SAF-007)",
                    p.design.class
                );
                // The stop from v_lim after t_react at a_brk ends before x_obs − margin.
                let stop = got * t_r + got * got / (2.0 * a_brk);
                assert!(stop <= d + 1e-9, "stop {stop} > {d} (SAF-007)");
                // Mirror image when closing.
                let xc = PosNm::from_m_f64(-1.0 + margin + d);
                let gc = f64::from(g.v_lim_mps(xc, TravelDir::Close, m, &lim));
                assert!((gc - got).abs() <= 1e-6, "closing mirror {gc} vs {got}");
            }
            // Inside the margin and past x_obs: no motion toward x_obs.
            let inside = PosNm::from_m_f64(1.0 - 0.5 * margin);
            assert_eq!(g.v_lim_mps(inside, TravelDir::Open, m, &lim), 0.0);
            assert_eq!(
                g.v_lim_mps(PosNm::from_m_f64(1.2), TravelDir::Open, m, &lim),
                0.0
            );
            // Moving away from x_obs is not braking-limited.
            let away = g.v_lim_mps(inside, TravelDir::Close, m, &lim);
            assert!(away > 0.0);
        }
    }

    /// Log-damped terminus reference (plan §4b; PHYSICS §16.1) in closed form, f64:
    /// distance-to-go d(t) and speed |ḋ(t)| over blend, capture and quintic.
    fn terminus_profile(v_app: f64, j: f64, tau: f64, d_eps: f64, n: usize) -> Vec<(f64, f64)> {
        let b = derive::terminus_blend(v_app, j, tau);
        let (t_b, d_b, d_0) = (b.t_b_s, b.d_b_m, b.d_0_m);
        let t_c = tau * (d_b / d_eps).ln();
        let t_q = tau;
        // Quintic in d from (d_ε, −d_ε/τ, d_ε/τ²) to (0, 0, 0) in T_q.
        let (q0, q1, q2) = (d_eps, -d_eps / tau, d_eps / (tau * tau));
        let tq = t_q;
        // d(s) = q0 + q1 s + q2/2 s² + c3 s³ + c4 s⁴ + c5 s⁵ with d(T)=ḋ(T)=d̈(T)=0.
        let (a0, a1, a2) = (q0, q1, 0.5 * q2);
        let c3 = (-20.0 * a0 - 12.0 * a1 * tq - 6.0 * a2 * tq * tq) / (2.0 * tq.powi(3));
        let c4 = (30.0 * a0 + 16.0 * a1 * tq + 6.0 * a2 * tq * tq) / (2.0 * tq.powi(4));
        let c5 = (-12.0 * a0 - 6.0 * a1 * tq - 2.0 * a2 * tq * tq) / (2.0 * tq.powi(5));
        let total = t_b + t_c + t_q;
        (0..=n)
            .map(|k| {
                let t = total * k as f64 / n as f64;
                if t <= t_b {
                    (
                        d_0 - v_app * t + j * t * t * t / 6.0,
                        v_app - 0.5 * j * t * t,
                    )
                } else if t <= t_b + t_c {
                    let d = d_b * (-(t - t_b) / tau).exp();
                    (d, d / tau)
                } else {
                    let s = (t - t_b - t_c).min(tq);
                    let d =
                        a0 + a1 * s + a2 * s * s + c3 * s.powi(3) + c4 * s.powi(4) + c5 * s.powi(5);
                    let dd = a1
                        + 2.0 * a2 * s
                        + 3.0 * c3 * s * s
                        + 4.0 * c4 * s.powi(3)
                        + 5.0 * c5 * s.powi(4);
                    (d.max(0.0), (-dd).max(0.0))
                }
            })
            .collect()
    }

    /// With x_obs = terminus + overtravel the governor never clips the log-damped
    /// terminus (blend, exponential tail and quintic) for any class, closing or opening,
    /// for m̂ from 0 to 1.2·m_rated, when the approach speed is the governor's own
    /// cruise envelope far from x_obs (the speed the supervisor plans at).
    /// (E) f32, plan §4b, PHYSICS §16.1.
    // req: FAB-HAP-015
    #[test]
    fn terminus_not_clipped() {
        for p in all() {
            let g = Governor::new(&p).unwrap_or_default();
            let tm = &p.control.motion.terminus;
            let target = PosNm::from_m_f64(0.123_456_789);
            for dir in [TravelDir::Close, TravelDir::Open] {
                let mut lim = cap_limits(&p);
                let over = tm.overtravel_m as f32;
                match dir {
                    TravelDir::Close => lim.x_obs_close = target.offset_m(-over),
                    TravelDir::Open => lim.x_obs_open = target.offset_m(over),
                }
                let m_r = p.design.carriage.m_rated_kg as f32;
                for m_hat in [0.0, m_r / 1.05, m_r, 1.2 * m_r] {
                    let far = target.offset_m(-dir.sign() * 1.0);
                    let v_app = f64::from(g.v_lim_mps(far, dir, m_hat, &lim));
                    let prof = terminus_profile(v_app, tm.j_mps3, tm.tau_s, tm.d_eps_m, 4000);
                    for (k, &(d, v)) in prof.iter().enumerate() {
                        let x = target.offset_m(-dir.sign() * d as f32);
                        let v_lim = f64::from(g.v_lim_mps(x, dir, m_hat, &lim));
                        assert!(
                            v <= v_lim * (1.0 + 1e-6) + 1e-9,
                            "{:?} {dir:?} m̂ {m_hat} sample {k}: d = {:.4} mm, v_ref {v} > \
                             v_lim {v_lim} (tail clipped; plan §4b)",
                            p.design.class,
                            d * 1e3
                        );
                    }
                    // The tail is well inside the braking envelope near the target.
                    let x_eps = target.offset_m(-dir.sign() * tm.d_eps_m as f32);
                    assert!(g.v_lim_mps(x_eps, dir, m_hat, &lim) > (tm.d_eps_m / tm.tau_s) as f32);
                }
            }
        }
    }

    #[test]
    fn energy_uses_governor_mass() {
        let p = presets::door();
        let g = Governor::new(&p).unwrap_or_default();
        // m̂ below rated: ½·m_rated·v²; above: ½·1.05·m̂·v².
        assert!((g.e_kin_j(50.0, 0.1) - 0.5).abs() < 1e-6);
        assert!((g.e_kin_j(200.0, 0.1) - 0.5 * 210.0 * 0.01).abs() < 1e-4);
    }

    /// D4: the governor's cruise envelope keeps KE ≤ 0.90·E_max and p ≤ 0.95·p_max at
    /// m_gov (the monitor trips at the caps themselves), for every class and m̂.
    /// (E) f32 rel 1e-5, Wave 2 lead decision D4, SAF-001, SAF-012 (no SPEC token: no
    /// req marker).
    #[test]
    fn planning_margin_below_caps() {
        for p in all() {
            let g = Governor::new(&p).unwrap_or_default();
            let mut lim = cap_limits(&p);
            // Mode caps high: only the energy/momentum terms bind far from x_obs.
            lim.v_open_mps = 10.0;
            lim.v_close_mps = 10.0;
            let m_r = p.design.carriage.m_rated_kg as f32;
            for m_hat in [0.0, m_r, 1.3 * m_r] {
                for dir in [TravelDir::Open, TravelDir::Close] {
                    let v = g.v_lim_mps(PosNm::ZERO, dir, m_hat, &lim);
                    let m_gov = g.m_gov_kg(m_hat);
                    let v_top = v + g.a_brk_live_mps2(&lim) * g.t_react_s;
                    let ke = 0.5 * m_gov * v_top * v_top;
                    assert!(
                        ke <= GOV_KE_PLAN_FRAC * g.e_max_j * (1.0 + 1e-5),
                        "{:?}: planned KE {ke} J > 0.90·E_max {} (D4)",
                        p.design.class,
                        g.e_max_j
                    );
                    if g.p_max_kgmps > 0.0 {
                        assert!(
                            m_gov * v_top <= GOV_P_PLAN_FRAC * g.p_max_kgmps * (1.0 + 1e-5),
                            "{:?}: planned momentum above 0.95·p_max (D4)",
                            p.design.class
                        );
                    }
                    // Speed margin ≥ 5 % below the energy cap speed √(2E/m_gov) (≈ 0.949).
                    let v_cap = fmath::sqrtf(2.0 * g.e_max_j / m_gov);
                    assert!(
                        v_top <= 0.95 * v_cap,
                        "{:?}: ≥ 5 % speed margin (D4)",
                        p.design.class
                    );
                }
            }
        }
    }

    /// D4 governor damping: full drive up to 0.975 of the full cap, none at the full
    /// cap, linear in between; every class, both directions.
    #[test]
    fn drive_scale_fades_to_zero_at_cap() {
        for p in all() {
            let g = Governor::new(&p).unwrap_or_default();
            let lim = cap_limits(&p);
            let m_r = p.design.carriage.m_rated_kg as f32;
            for sgn in [1.0f32, -1.0] {
                let dir = if sgn > 0.0 {
                    TravelDir::Open
                } else {
                    TravelDir::Close
                };
                let m_gov = g.m_gov_kg(m_r);
                let hi = fmath::sqrtf(2.0 * g.e_max_j / m_gov).min(if g.p_max_kgmps > 0.0 {
                    g.p_max_kgmps / m_gov
                } else {
                    f32::INFINITY
                });
                let lo = GOV_DRIVE_FADE_FRAC * hi;
                assert!(
                    g.v_plan_cap_mps(dir, m_r, &lim) < lo,
                    "{:?}: the planning cap lies below the fade band (D4)",
                    p.design.class
                );
                assert_eq!(g.drive_scale(sgn * 0.5 * lo, m_r, &lim), 1.0);
                assert!(g.drive_scale(sgn * 0.999 * lo, m_r, &lim) == 1.0);
                assert!(
                    g.drive_scale(sgn * hi * 1.0001, m_r, &lim) == 0.0,
                    "{:?}: no drive work at the cap (D4)",
                    p.design.class
                );
                if hi > lo * 1.001 {
                    let mid = g.drive_scale(sgn * 0.5 * (lo + hi), m_r, &lim);
                    assert!(
                        (mid - 0.5).abs() < 1e-3,
                        "{:?}: linear fade {mid}",
                        p.design.class
                    );
                }
            }
        }
    }
}
