//! Init-time levitation gain design (f64 to f32) from params::derive::zp_gains_discrete (PHYSICS §15). Owner: A1.
//!
//! Frozen API (W0): [`design`] → [`LevGains`]. f64 is allowed here (init-time gain
//! design, core design §A.4 item 1); the result is cast to f32 once. Stable for
//! τ_e ≤ 2 ms in every class (LEV-020); discrete poles within 2 % of the design, ζ ≥ 0.5
//! (LEV-007 as corrected by plan §2b item 1).
//!
//! Per controlled mode (heave, pitch; roll on DRAWER and HEAVY) this module produces:
//! - the zero-power gains (k_p, k_d, κ) of `derive::zp_gains_discrete` at
//!   ω = omega_ratio·max(p_mode, p_heave) = 2.5·max(p_mode, p_heave), designed on the
//!   ZOH plant with the LET delay, τ_cl = 1/(2π·current_bw_hz) and τ_e,nom;
//! - the exact ZOH model (Φ, Γ) of the four-state design plant [δg, δġ, i, i_e] (the
//!   same model, derived here by the matrix exponential), from which the Luenberger
//!   observer at observer_ratio·ω = 4ω is built (`observer::GapObserver::from_model`).
//!   Because the observer propagates the design model exactly (measured coil current,
//!   known LET-delayed command), the separation principle holds and the closed-loop
//!   poles are the design poles plus the observer poles (LEV-007 tests this on the f32
//!   implementation);
//! - servo gains for the lift/land/seat gap-tracking phases, from the continuous
//!   identity between the Mizuno law and a gap PID (PHYSICS §15): with q = ∫i and the
//!   modal equation δg̈ = p²δg − b·i, i* = k_p·δg + k_d·δġ + κ·q is equivalent to
//!   i* = k_p·δg + (k_d − κ/b)·δġ + (κ·p²/b)·∫δg (same closed-loop polynomial; the
//!   servo integral holds the gap instead of the current at zero).
//!
//! [`ElementModel`] carries the f32 closed forms of one hybrid element used at run time
//! (feed-forward current, lift-off mass, k_i): F = μ₀A_p·(M_pm + N·i)²/(2g + h′)²
//! (PHYSICS §2–§3 at y = 0, σ_leak = 0).

use fabrail_params::derive::{self, G0_MPS2, LevMode, MU0_H_PER_M};
use fabrail_params::{ParamError, SystemParams};
use fabrail_types::fmath;

/// Zero-power gains for one mode, per magnet (f32 copy of `derive::ZpGains`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModeGains {
    /// Gap gain k_p, A/m.
    pub kp_a_per_m: f32,
    /// Gap-rate gain k_d, A·s/m.
    pub kd_a_s_per_m: f32,
    /// Current-integral gain κ, 1/s.
    pub kappa_per_s: f32,
}

/// Exact ZOH model of one mode's design plant x = [δg, δġ, i, i_e] with input u (the
/// modal current command applied over the period): x⁺ = Φx + Γu (PHYSICS §15).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModePlant {
    /// Φ (f64, init-time).
    pub phi: [[f64; 4]; 4],
    /// Γ (f64, init-time).
    pub gam: [f64; 4],
}

/// Everything the run-time loop of one mode needs (f32, made once).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModeDesign {
    /// The mode gets a zero-power loop (heave, pitch; roll when `roll_active`).
    pub active: bool,
    /// Zero-power gains.
    pub zp: ModeGains,
    /// Servo gains [k_p (A/m), k_d − κ/b (A·s/m), κ·p²/b (A/(m·s))] for gap tracking.
    pub servo: [f32; 3],
    /// Target natural frequency ω, rad/s.
    pub omega_rad_s: f32,
    /// Open-loop pole squared p² = k_eff/m_eff, 1/s².
    pub p2_per_s2: f32,
    /// Input gain b = k_i/m_eff per element, m/(A·s²).
    pub b_m_per_a_s2: f32,
    /// Effective mass per element m_eff, kg (heave m*/n; pitch/roll J/Σr²).
    pub m_eff_kg: f32,
    /// Current gain per element k_i(g*, 0), N/A.
    pub k_i_n_per_a: f32,
    /// Observer bandwidth observer_ratio·ω, rad/s.
    pub observer_rad_s: f32,
    /// Sampled design plant (for the observer).
    pub plant: ModePlant,
    /// Coupled roll–sway extension (roll on a class with strong roll–sway coupling; D1).
    pub sway: SwayDesign,
}

/// Coupled roll–sway extension of the roll design (PHYSICS §6, §15; Wave 2 decision D1):
/// from `derive::roll_sway_design`, the sway feedback gains and the coupled observer's
/// model and gains. Inactive (all zero) for every other mode and class.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SwayDesign {
    /// The roll loop runs the coupled design.
    pub active: bool,
    /// Sway gains [k_Y (A/m), k_Ẏ (A·s/m)] on the CoM sway estimate.
    pub k: [f32; 2],
    /// Coupled observer correction gains for [u, u̇, Y, Ẏ].
    pub l: [f64; 4],
    /// Sampled roll–sway design plant Φ over [u, u̇, Y, Ẏ, i, i_e].
    pub phi: [[f64; 6]; 6],
    /// Sampled roll–sway design plant Γ.
    pub gam: [f64; 6],
    /// Passive sway frequency ω_y, rad/s.
    pub omega_y_rad_s: f32,
}

/// Full levitation gain set.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LevGains {
    /// Heave.
    pub heave: ModeGains,
    /// Pitch.
    pub pitch: ModeGains,
    /// Roll (zero when roll is passive: top-hung DOOR/WALL).
    pub roll: ModeGains,
    /// Roll actively controlled.
    pub roll_active: bool,
    /// Observer bandwidth 4·ω of the heave mode, rad/s (per-mode values in `modes`).
    pub observer_rad_s: f32,
    /// Fast-group period, s.
    pub dt_s: f32,
    /// Per-mode designs, index 0 heave, 1 pitch, 2 roll.
    pub modes: [ModeDesign; 3],
    /// Observer damping ratio (the loop's ζ, 0.7).
    pub observer_zeta: f32,
    /// Current-loop time constant τ_cl, s.
    pub tau_cl_s: f32,
    /// Design eddy lag τ_e,nom, s.
    pub tau_e_s: f32,
}

/// f32 closed forms of one hybrid PM–EMS lift element at y = 0 (PHYSICS §2–§3).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ElementModel {
    /// μ₀·A_p, H·m.
    pub mu0_ap: f32,
    /// Magnet MMF M_pm, A-turns.
    pub m_pm: f32,
    /// Coil turns N.
    pub turns: f32,
    /// Magnet reluctance length h′ = h_m/μ_rec, m.
    pub h_prime_m: f32,
    /// Lift elements carrying the weight.
    pub n_lift: f32,
}

impl ElementModel {
    /// From the suspension design (σ_leak = 0, full engagement).
    pub fn new(p: &SystemParams) -> ElementModel {
        let s = &p.design.suspension;
        ElementModel {
            mu0_ap: (MU0_H_PER_M * s.pole_w_m * s.pole_l_m) as f32,
            m_pm: derive::m_pm(s.b_r_t, s.h_m_m, s.mu_rec) as f32,
            turns: s.turns as f32,
            h_prime_m: derive::h_prime(s.h_m_m, s.mu_rec) as f32,
            n_lift: derive::n_lift(s) as f32,
        }
    }

    /// Element force F = μ₀A_p·(M_pm + N·i)²/(2g + h′)², N.
    pub fn force(&self, g_m: f32, i_a: f32) -> f32 {
        let d = 2.0 * g_m + self.h_prime_m;
        let mmf = self.m_pm + self.turns * i_a;
        if d > 0.0 {
            self.mu0_ap * mmf * mmf / (d * d)
        } else {
            0.0
        }
    }

    /// Current for element force `f_n` at gap `g_m`:
    /// i = ((2g + h′)·√(F/(μ₀A_p)) − M_pm)/N (F ≤ 0 gives the PM-cancelling current).
    pub fn current_for(&self, g_m: f32, f_n: f32) -> f32 {
        if self.turns <= 0.0 || self.mu0_ap <= 0.0 {
            return 0.0;
        }
        let d = 2.0 * g_m + self.h_prime_m;
        let root = fmath::sqrtf(fmath::fmaxf(f_n, 0.0) / self.mu0_ap);
        (d * root - self.m_pm) / self.turns
    }

    /// Mass held by all lift elements at gap `g_m` and equal current `i_a`:
    /// m = n·F/g₀, kg.
    pub fn mass(&self, g_m: f32, i_a: f32) -> f32 {
        self.n_lift * self.force(g_m, i_a) / G0_MPS2 as f32
    }

    /// Current gain k_i = 2N·μ₀A_p·(M_pm + N·i)/(2g + h′)², N/A.
    pub fn k_i(&self, g_m: f32, i_a: f32) -> f32 {
        let d = 2.0 * g_m + self.h_prime_m;
        if d > 0.0 {
            2.0 * self.turns * self.mu0_ap * (self.m_pm + self.turns * i_a) / (d * d)
        } else {
            0.0
        }
    }

    /// Zero-power gap of mass `m_kg`: ((2g* + h′)·√(m*/m) − h′)/2 via the element force
    /// balance at i = 0, m.
    pub fn zero_power_gap(&self, m_kg: f32) -> f32 {
        if m_kg <= 0.0 || self.n_lift <= 0.0 {
            return f32::INFINITY;
        }
        // n·μ₀A_p·M²/(2g + h′)² = m·g₀.
        let d = self.m_pm * fmath::sqrtf(self.n_lift * self.mu0_ap / (m_kg * G0_MPS2 as f32));
        0.5 * (d - self.h_prime_m)
    }
}

/// Designs every mode's gains for `p`.
///
/// Errors when the rates are degenerate, heave is not controllable, or an active mode's
/// design is singular or non-finite (never a silent zero gain).
pub fn design(p: &SystemParams) -> Result<LevGains, ParamError> {
    let lt = &p.control.levitation;
    let t = derive::fast_period_s(p);
    let tau_cl = derive::tau_current_loop_s(p);
    let tau_e = fmath::fmax(lt.tau_e_nominal_s, 0.0);
    if t.is_nan() || t <= 0.0 || tau_cl.is_nan() || tau_cl <= 0.0 {
        return Err(ParamError::new(
            "control.rates.fast_decim",
            "fast period and current bandwidth must be > 0",
        ));
    }
    let mut out = LevGains {
        roll_active: lt.roll_active,
        dt_s: t as f32,
        observer_zeta: lt.zeta as f32,
        tau_cl_s: tau_cl as f32,
        tau_e_s: tau_e as f32,
        ..LevGains::default()
    };
    for (md, mode) in out.modes.iter_mut().zip(LevMode::ALL.iter()) {
        if !derive::mode_active(p, *mode) {
            continue;
        }
        let mm = derive::mode_model(p, *mode);
        let mut k = derive::zp_gains_discrete(p, *mode);
        // D1: a roll loop with strong roll–sway coupling (HEAVY) uses the coupled design;
        // its gap, rate and integral gains replace the single-DOF ones.
        let mut sway = SwayDesign::default();
        if matches!(mode, LevMode::Roll) && derive::roll_sway_coupled(p) {
            let rs = derive::roll_sway_design(p).ok_or_else(|| {
                ParamError::new(
                    "design.guidance.c_sway_n_s_per_m",
                    "coupled roll–sway design failed",
                )
            })?;
            k = derive::ZpGains {
                kp_a_per_m: rs.k[0],
                kd_a_s_per_m: rs.k[1],
                kappa_per_s: rs.k[4],
            };
            sway = SwayDesign {
                active: true,
                k: [rs.k[2] as f32, rs.k[3] as f32],
                l: rs.l,
                phi: rs.phi,
                gam: rs.gam,
                omega_y_rad_s: rs.omega_y_rad_s as f32,
            };
        }
        let w = derive::zp_omega(p, *mode);
        let p2 = mm.k_eff_n_per_m / mm.m_eff_kg;
        let b = mm.k_i_n_per_a / mm.m_eff_kg;
        let ok = [k.kp_a_per_m, k.kd_a_s_per_m, k.kappa_per_s, p2, b, w]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0);
        if !ok {
            return Err(ParamError::new(
                "control.levitation.omega_ratio",
                "zero-power design failed for an active mode",
            ));
        }
        let plant = zoh_plant(p2, b, t, tau_cl, tau_e);
        *md = ModeDesign {
            active: true,
            zp: ModeGains {
                kp_a_per_m: k.kp_a_per_m as f32,
                kd_a_s_per_m: k.kd_a_s_per_m as f32,
                kappa_per_s: k.kappa_per_s as f32,
            },
            servo: [
                k.kp_a_per_m as f32,
                (k.kd_a_s_per_m - k.kappa_per_s / b) as f32,
                (k.kappa_per_s * p2 / b) as f32,
            ],
            omega_rad_s: w as f32,
            p2_per_s2: p2 as f32,
            b_m_per_a_s2: b as f32,
            m_eff_kg: mm.m_eff_kg as f32,
            k_i_n_per_a: mm.k_i_n_per_a as f32,
            observer_rad_s: (lt.observer_ratio * w) as f32,
            plant,
            sway,
        };
    }
    if !out.modes[0].active {
        return Err(ParamError::new(
            "design.suspension",
            "heave is not controllable (no unstable pole or no lift element)",
        ));
    }
    out.heave = out.modes[0].zp;
    out.pitch = out.modes[1].zp;
    out.roll = out.modes[2].zp;
    out.observer_rad_s = out.modes[0].observer_rad_s;
    Ok(out)
}

/// Exact ZOH of the design plant (PHYSICS §15; the model of `derive::zp_gains_discrete`):
/// δg̈ = p²·δg − b·i_e, τ_cl·i̇ = u − i, τ_e·i̇_e = i − i_e (with τ_e = 0 the force follows
/// i directly and i_e is a decoupled stable dummy). Van Loan: exp([[A, B], [0, 0]]·T)
/// holds Φ and Γ.
pub fn zoh_plant(p2: f64, b: f64, t: f64, tau_cl: f64, tau_e: f64) -> ModePlant {
    let mut a = [[0.0f64; 5]; 5];
    a[0][1] = 1.0;
    a[1][0] = p2;
    a[2][2] = -1.0 / tau_cl;
    a[2][4] = 1.0 / tau_cl;
    if tau_e > 0.0 {
        a[1][3] = -b;
        a[3][2] = 1.0 / tau_e;
        a[3][3] = -1.0 / tau_e;
    } else {
        a[1][2] = -b;
        a[3][3] = -1.0 / tau_cl;
    }
    for row in a.iter_mut() {
        for v in row.iter_mut() {
            *v *= t;
        }
    }
    let e = expm5(&a);
    let mut out = ModePlant::default();
    for ((prow, g), erow) in out.phi.iter_mut().zip(out.gam.iter_mut()).zip(e.iter()) {
        for (d, s) in prow.iter_mut().zip(erow.iter()) {
            *d = *s;
        }
        *g = erow[4];
    }
    out
}

type M5 = [[f64; 5]; 5];

fn matmul5(a: &M5, b: &M5) -> M5 {
    let mut c = [[0.0f64; 5]; 5];
    for (crow, arow) in c.iter_mut().zip(a.iter()) {
        for (aik, brow) in arow.iter().zip(b.iter()) {
            for (cij, bkj) in crow.iter_mut().zip(brow.iter()) {
                *cij += aik * bkj;
            }
        }
    }
    c
}

/// Matrix exponential by scaling and squaring with an 18-term Taylor series (Moler &
/// Van Loan, "Nineteen Dubious Ways…", SIAM Review 45(1), 2003, method 3): scale so
/// ‖A/2^s‖∞ ≤ 0.5, where the truncation error is below 0.5¹⁹/19! ≈ 1.6e-23.
fn expm5(a: &M5) -> M5 {
    let norm = a
        .iter()
        .map(|r| r.iter().map(|v| fmath::abs(*v)).sum::<f64>())
        .fold(0.0, fmath::fmax);
    let mut s = 0u32;
    let mut scale = 1.0f64;
    while norm * scale > 0.5 && s < 60 {
        scale *= 0.5;
        s += 1;
    }
    let mut b = *a;
    for row in b.iter_mut() {
        for v in row.iter_mut() {
            *v *= scale;
        }
    }
    let mut term = identity5();
    let mut sum = identity5();
    for k in 1..=18u32 {
        term = matmul5(&term, &b);
        let inv_k = 1.0 / f64::from(k);
        for row in term.iter_mut() {
            for v in row.iter_mut() {
                *v *= inv_k;
            }
        }
        for (srow, trow) in sum.iter_mut().zip(term.iter()) {
            for (sv, tv) in srow.iter_mut().zip(trow.iter()) {
                *sv += tv;
            }
        }
    }
    for _ in 0..s {
        sum = matmul5(&sum, &sum);
    }
    sum
}

fn identity5() -> M5 {
    let mut m = [[0.0f64; 5]; 5];
    for (r, row) in m.iter_mut().enumerate() {
        for (c, v) in row.iter_mut().enumerate() {
            if r == c {
                *v = 1.0;
            }
        }
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    #[test]
    fn design_matches_derive_for_every_class() {
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let g = design(&p).expect("preset design");
            for (md, mode) in g.modes.iter().zip(LevMode::ALL.iter()) {
                assert_eq!(md.active, derive::mode_active(&p, *mode));
                let coupled = matches!(mode, LevMode::Roll) && derive::roll_sway_coupled(&p);
                assert_eq!(
                    md.sway.active, coupled,
                    "coupled roll–sway only where selected (D1)"
                );
                let k = if coupled {
                    // D1: the coupled design's gap, rate and integral gains.
                    let rs = derive::roll_sway_design(&p).expect("coupled design");
                    assert!(
                        (f64::from(md.sway.k[0]) / rs.k[2] - 1.0).abs() < 1e-6
                            && (f64::from(md.sway.k[1]) / rs.k[3] - 1.0).abs() < 1e-6,
                        "sway gains copied from derive::roll_sway_design"
                    );
                    derive::ZpGains {
                        kp_a_per_m: rs.k[0],
                        kd_a_s_per_m: rs.k[1],
                        kappa_per_s: rs.k[4],
                    }
                } else {
                    derive::zp_gains_discrete(&p, *mode)
                };
                assert!(
                    (f64::from(md.zp.kp_a_per_m) / k.kp_a_per_m.max(1e-30) - 1.0).abs() < 1e-6
                        || !md.active
                );
                if md.active {
                    assert!((f64::from(md.zp.kappa_per_s) / k.kappa_per_s - 1.0).abs() < 1e-6);
                    let w = derive::zp_omega(&p, *mode);
                    assert!(
                        (f64::from(md.observer_rad_s) / (4.0 * w) - 1.0).abs() < 1e-6,
                        "observer at 4ω (core §C.4)"
                    );
                    assert!(
                        md.servo[1] > 0.0 && md.servo[2] > 0.0,
                        "servo gains positive"
                    );
                }
            }
            assert_eq!(g.roll_active, p.control.levitation.roll_active);
            assert!(g.modes[1].active, "pitch active in every class (LEV-001)");
        }
        assert!(
            design(&SystemParams::default()).is_err(),
            "degenerate params rejected"
        );
    }

    #[test]
    fn zoh_matches_closed_form_for_pure_mechanics() {
        // With b = 0 the mechanical block is exp([[0,1],[p²,0]]T) = [[cosh, sinh/p], [p·sinh, cosh]].
        let (p2, t) = (4828.0f64, 1e-4);
        let m = zoh_plant(p2, 0.0, t, 1.59e-4, 1e-3);
        let p = p2.sqrt();
        assert!((m.phi[0][0] - (p * t).cosh()).abs() < 1e-14);
        assert!((m.phi[0][1] - (p * t).sinh() / p).abs() < 1e-16);
        assert!((m.phi[1][0] - p * (p * t).sinh()).abs() < 1e-11);
        // Current lag: i⁺ = e^{−T/τ}·i + (1 − e^{−T/τ})·u.
        let a = (-t / 1.59e-4).exp();
        assert!((m.phi[2][2] - a).abs() < 1e-14 && (m.gam[2] - (1.0 - a)).abs() < 1e-14);
    }

    #[test]
    fn element_model_matches_derive() {
        let p = presets::drawer();
        let e = ElementModel::new(&p);
        let s = &p.design.suspension;
        for (g, i) in [
            (2.5e-3, 0.0),
            (3.5e-3, 11.2),
            (1.5e-3, 0.39),
            (3.5e-3, -4.99),
        ] {
            let f = derive::pm_force(s, g, 0.0, i);
            assert!((f64::from(e.force(g as f32, i as f32)) / f - 1.0).abs() < 1e-5);
            let ki = derive::k_i(s, g, i);
            assert!((f64::from(e.k_i(g as f32, i as f32)) / ki - 1.0).abs() < 1e-5);
        }
        let ib = derive::bias_current(&p.design, 3.5e-3, 48.0);
        let f = 48.0 * G0_MPS2 / 4.0;
        assert!((f64::from(e.current_for(3.5e-3, f as f32)) - ib).abs() < 1e-4);
        assert!((f64::from(e.mass(3.5e-3, ib as f32)) - 48.0).abs() < 1e-3);
        let gz = derive::zero_power_gap(48.0, s.g_star_m, derive::h_prime(s.h_m_m, s.mu_rec), 26.0);
        assert!((f64::from(e.zero_power_gap(48.0)) - gz).abs() < 1e-8);
    }
}
