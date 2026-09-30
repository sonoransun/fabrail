//! Co-energy hybrid PM-EMS element: permeance with lateral fringing, tanh saturation, eddy lag, tempco, leakage (PHYSICS §2). Owner: A6a.
//!
//! W′ = ½(M_eff + N·i)²·P(g, y, x), P = e(x)·μ0·A_p / (2g/f(g, y) + h′),
//! f = 1 − (g/w)·ln cosh(y/g); saturation Φ = Φ_sat·tanh(Φ_lin/Φ_sat) at B_sat;
//! tempco −0.12 %/K on B_r. Forces are the co-energy gradient
//! F_q = ½·(Φ/P)²·∂P/∂q (generalised force along +q), i.e. the attraction
//! −∂W′/∂g closes the gap and ∂W′/∂y restores the lateral offset (conservative by
//! construction while saturation and eddy lag are inactive; physics perspective §1.2).
//!
//! Model as implemented (PHYSICS §2):
//! * M_eff = k_σ·B_r(T)·h_m/(μ0·μ_rec), k_σ = 1/(1 + σ_leak), B_r(T) = B_r·(1 + α_Br·(T − 20 °C));
//!   h′ = h_m/μ_rec. Pure-EMS hold-down elements have M = 0 and h′ = 0, so
//!   F = μ0·A_p·N²·i²/(4g²).
//! * Saturation acts on the flux through the pole face, Φ_sat = B_sat·A_p (the same cap as
//!   `params::derive::element_force_max`; the rail flange is sized separately by
//!   LEV-015). The tanh shape is applied above a knee at [`SAT_KNEE_FRAC`]·Φ_sat, so the
//!   design envelope (B_g 0.5–0.6 T at g*, lift currents) stays exactly linear and the
//!   zero-power anchors of `params::derive` hold; see [`saturate`].
//! * The eddy lag τ_e·Φ̇_eff = Φ − Φ_eff acts on the force-producing flux only
//!   [heuristic, PHYSICS §2]; the tier integrates Φ_eff and passes it to
//!   [`ElementEval::loads`].

use fabrail_params::SuspensionDesign;
use fabrail_params::derive::{MU0_H_PER_M, h_prime, m_pm};
use fabrail_types::fmath;

/// Floor of the lateral overlap factor f(g, y), the same floor as `params::derive`, so a
/// large lateral offset cannot drive the permeance through zero (PHYSICS §2).
pub const FRINGE_F_MIN: f64 = 1.0e-3;

/// Smallest gap at which the permeance is evaluated, m. It guards 2g/f + h′ → 0 for
/// pure-EMS hold-down elements (h′ = 0); contacts keep real gaps ≥ g_up (PHYSICS §2).
pub const GAP_MIN_M: f64 = 1.0e-6;

/// Saturation knee as a fraction of Φ_sat: below it Φ = Φ_lin exactly, above it the
/// flux approaches Φ_sat along a tanh (C² at the knee). Design choice \[UNCERTAIN\]:
/// electrical steel keeps a high permeability up to ≈ 1.0–1.1 T (≈ 0.7·B_sat at
/// B_sat = 1.5 T); a pure tanh from zero would already cut the force at B_g = 0.55 T by
/// ≈ 8 % and move every zero-power gap by ≈ 0.4 mm (PHYSICS §2).
pub const SAT_KNEE_FRAC: f64 = 0.7;

/// Reference temperature of B_r and the coil resistance, °C (datasheet reference;
/// PHYSICS §12).
pub const T_REF_C: f64 = 20.0;

/// One element's loads and flux.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ElementForce {
    /// Attraction toward the rail (+ closes the gap), N.
    pub f_attract_n: f64,
    /// Lateral force on the element along +y (restoring: opposite in sign to the offset),
    /// N.
    pub fy_n: f64,
    /// Force-producing flux Φ, Wb.
    pub flux_wb: f64,
    /// Incremental flux linkage ∂ψ/∂i (the coil's apparent inductance), H.
    pub dpsi_di_h: f64,
}

/// Element loads at gap `g_m`, lateral offset `y_m`, coil current `i_a`, truth
/// remanence `b_r_t` and engagement `e` ∈ [0, 1] (PHYSICS §2): a PM-biased element at
/// the design leakage `s.leak`, 20 °C, with saturation and no eddy lag.
pub fn element_force(
    s: &SuspensionDesign,
    b_r_t: f64,
    g_m: f64,
    y_m: f64,
    i_a: f64,
    e: f64,
) -> ElementForce {
    let model = ElementModel::new(s, b_r_t, s.leak, true);
    let ev = model.eval(
        &ElementGeom {
            g_m,
            y_m,
            e,
            de_dx_per_m: 0.0,
        },
        i_a,
        T_REF_C,
    );
    let loads = ev.loads(ev.phi_wb);
    ElementForce {
        f_attract_n: loads.f_attract_n,
        fy_n: loads.fy_n,
        flux_wb: ev.phi_wb,
        dpsi_di_h: model.dpsi_di_h(&ev),
    }
}

/// ln cosh(u), overflow-free: |u| + ln(1 + e^{−2|u|}) − ln 2 (PHYSICS §2).
pub fn ln_cosh(u: f64) -> f64 {
    let a = fmath::abs(u);
    if a < 1.0e-4 {
        // Series u²/2 − u⁴/12 (next term u⁶/45 < 3e-26): exact to f64 and cheap.
        let u2 = a * a;
        return u2 * (0.5 - u2 / 12.0);
    }
    a + fmath::ln_1p(fmath::exp(-2.0 * a)) - core::f64::consts::LN_2
}

/// Flux saturation with a linear region up to the knee K = [`SAT_KNEE_FRAC`]·Φ_sat and a
/// tanh approach to Φ_sat above it: Φ = sign·(K + (Φ_sat − K)·tanh((|Φ_lin| − K)/(Φ_sat − K)))
/// (PHYSICS §2). Returns (Φ, dΦ/dΦ_lin). A non-finite or non-positive Φ_sat disables
/// saturation.
pub fn saturate(phi_lin_wb: f64, phi_sat_wb: f64) -> (f64, f64) {
    if !(phi_sat_wb.is_finite() && phi_sat_wb > 0.0) {
        return (phi_lin_wb, 1.0);
    }
    let knee = SAT_KNEE_FRAC * phi_sat_wb;
    let a = fmath::abs(phi_lin_wb);
    if a <= knee {
        return (phi_lin_wb, 1.0);
    }
    let span = phi_sat_wb - knee;
    let t = fmath::tanh((a - knee) / span);
    (fmath::copysign(knee + span * t, phi_lin_wb), 1.0 - t * t)
}

/// Element geometry at one evaluation (rail frame, PHYSICS §2, §5).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ElementGeom {
    /// Air gap g > 0, m.
    pub g_m: f64,
    /// Lateral offset of the pole from its rail track, m.
    pub y_m: f64,
    /// Engagement e(x) ∈ [0, 1] (DRAWER extension, rail ends; PHYSICS §10).
    pub e: f64,
    /// de/dx along the element's rail coordinate, 1/m.
    pub de_dx_per_m: f64,
}

/// Permeance P(g, y, x) and its partial derivatives (PHYSICS §2).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Permeance {
    /// P = e·μ0·A_p/D, H.
    pub p_h: f64,
    /// ∂P/∂g, H/m.
    pub dp_dg: f64,
    /// ∂P/∂y, H/m.
    pub dp_dy: f64,
    /// ∂P/∂x (engagement), H/m.
    pub dp_dx: f64,
}

/// Generalised loads of one element (PHYSICS §2).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ElementLoads {
    /// Attraction toward the rail, −∂W′/∂g (+ closes the gap), N.
    pub f_attract_n: f64,
    /// Force along +y, ∂W′/∂y (restoring), N.
    pub fy_n: f64,
    /// Force along +x from the engagement gradient, ∂W′/∂x (pulls into engagement), N.
    pub fx_n: f64,
}

/// Magnetic state of one element at one evaluation (PHYSICS §2, §4).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ElementEval {
    /// Permeance and gradient.
    pub perm: Permeance,
    /// Total MMF M_eff(T) + N·i, A-turns.
    pub mmf_at: f64,
    /// Unsaturated flux Φ_lin = MMF·P, Wb.
    pub phi_lin_wb: f64,
    /// Saturated flux Φ, Wb.
    pub phi_wb: f64,
    /// dΦ/dΦ_lin (1 below the saturation knee).
    pub sat_slope: f64,
    /// ∂Φ/∂i, Wb/A.
    pub dphi_di: f64,
    /// ∂Φ/∂g, Wb/m.
    pub dphi_dg: f64,
    /// ∂Φ/∂y, Wb/m.
    pub dphi_dy: f64,
    /// ∂Φ/∂x, Wb/m.
    pub dphi_dx: f64,
    /// ∂Φ/∂T_mag through the B_r tempco, Wb/K.
    pub dphi_dt: f64,
}

impl ElementEval {
    /// Loads from the force-producing flux `phi_force_wb` (Φ, or the eddy-lagged Φ_eff):
    /// F_q = ½·(Φ_f/P)²·∂P/∂q (PHYSICS §2). With Φ_f = Φ below the saturation knee this is
    /// exactly ∂W′/∂q at constant current.
    pub fn loads(&self, phi_force_wb: f64) -> ElementLoads {
        let p = self.perm.p_h;
        let mmf_eq = if p > 0.0 {
            phi_force_wb / p
        } else {
            self.mmf_at
        };
        let half = 0.5 * mmf_eq * mmf_eq;
        ElementLoads {
            f_attract_n: -half * self.perm.dp_dg,
            fy_n: half * self.perm.dp_dy,
            fx_n: half * self.perm.dp_dx,
        }
    }

    /// Linear co-energy W′ = ½·MMF²·P, J (PHYSICS §2; exact below the saturation knee).
    pub fn coenergy_j(&self) -> f64 {
        0.5 * self.mmf_at * self.mmf_at * self.perm.p_h
    }

    /// Stored magnetic energy ½·Φ²/P, J (linear; PHYSICS §2, §4 power balance).
    pub fn stored_j(&self) -> f64 {
        if self.perm.p_h > 0.0 {
            0.5 * self.phi_wb * self.phi_wb / self.perm.p_h
        } else {
            0.0
        }
    }
}

/// Constant parameters of one element (PHYSICS §2).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ElementModel {
    /// Pole face area A_p = w·l, m².
    pub a_p_m2: f64,
    /// Pole face width w (across track), m.
    pub pole_w_m: f64,
    /// Magnet reluctance length h′ (0 for pure EMS), m.
    pub h_prime_m: f64,
    /// Coil turns N.
    pub turns: f64,
    /// Coil leakage inductance L_σ, H.
    pub l_sigma_h: f64,
    /// PM MMF at 20 °C including the leakage factor k_σ (0 for pure EMS), A-turns.
    pub m_pm_at: f64,
    /// Remanence temperature coefficient α_Br, 1/K.
    pub tempco_br_per_k: f64,
    /// Saturation flux Φ_sat = B_sat·A_p (∞ = no saturation), Wb.
    pub phi_sat_wb: f64,
}

impl ElementModel {
    /// The element of `s` with truth remanence `b_r_t` and leakage `leak`; `has_pm` false
    /// gives a pure-EMS hold-down element (M = 0, h′ = 0) (PHYSICS §2).
    pub fn new(s: &SuspensionDesign, b_r_t: f64, leak: f64, has_pm: bool) -> ElementModel {
        let a_p = s.pole_w_m * s.pole_l_m;
        let (m, hp) = if has_pm {
            // k_σ = 1/(1 + σ_leak) (physics perspective §1.2).
            let k_sigma = 1.0 / (1.0 + fmath::fmax(leak, 0.0));
            (
                k_sigma * m_pm(b_r_t, s.h_m_m, s.mu_rec),
                h_prime(s.h_m_m, s.mu_rec),
            )
        } else {
            (0.0, 0.0)
        };
        let phi_sat = if s.b_sat_t > 0.0 && a_p > 0.0 {
            s.b_sat_t * a_p
        } else {
            f64::INFINITY
        };
        ElementModel {
            a_p_m2: a_p,
            pole_w_m: s.pole_w_m,
            h_prime_m: hp,
            turns: s.turns,
            l_sigma_h: s.l_sigma_h,
            m_pm_at: m,
            tempco_br_per_k: s.tempco_br_per_k,
            phi_sat_wb: phi_sat,
        }
    }

    /// The same element with saturation disabled (analytic anchors and conservation
    /// tests).
    pub fn unsaturated(mut self) -> ElementModel {
        self.phi_sat_wb = f64::INFINITY;
        self
    }

    /// PM MMF at magnet temperature `t_mag_c`: M_eff·(1 + α_Br·(T − 20 °C)), A-turns
    /// (PHYSICS §2, §12).
    pub fn m_pm_at_temp(&self, t_mag_c: f64) -> f64 {
        self.m_pm_at * (1.0 + self.tempco_br_per_k * (t_mag_c - T_REF_C))
    }

    /// Incremental inductance ∂ψ/∂i = N·∂Φ/∂i + L_σ, H (PHYSICS §4).
    pub fn dpsi_di_h(&self, ev: &ElementEval) -> f64 {
        self.turns * ev.dphi_di + self.l_sigma_h
    }

    /// Permeance at gap `g_m` (floored at [`GAP_MIN_M`]), offset `y_m`, engagement `e`
    /// and gradient `de_dx` (PHYSICS §2):
    /// D = 2g/f + h′, ∂D/∂g = 2/f − 2g·f_g/f², ∂D/∂y = −2g·f_y/f²,
    /// f_g = −(1/w)·ln cosh(u) + (u/w)·tanh(u), f_y = −tanh(u)/w, u = y/g.
    pub fn permeance(&self, g_m: f64, y_m: f64, e: f64, de_dx: f64) -> Permeance {
        let g = fmath::fmax(g_m, GAP_MIN_M);
        let w = self.pole_w_m;
        let (f, f_g, f_y) = if w > 0.0 && y_m != 0.0 {
            let u = y_m / g;
            let lc = ln_cosh(u);
            let f_raw = 1.0 - (g / w) * lc;
            if f_raw > FRINGE_F_MIN {
                let th = fmath::tanh(u);
                (f_raw, -lc / w + (u / w) * th, -th / w)
            } else {
                (FRINGE_F_MIN, 0.0, 0.0)
            }
        } else {
            (1.0, 0.0, 0.0)
        };
        let d = 2.0 * g / f + self.h_prime_m;
        let d_g = 2.0 / f - 2.0 * g * f_g / (f * f);
        let d_y = -2.0 * g * f_y / (f * f);
        let p0 = MU0_H_PER_M * self.a_p_m2 / d;
        let dp0_dd = -p0 / d;
        Permeance {
            p_h: e * p0,
            dp_dg: e * dp0_dd * d_g,
            dp_dy: e * dp0_dd * d_y,
            dp_dx: de_dx * p0,
        }
    }

    /// Full magnetic evaluation at `geom`, current `i_a` and magnet temperature
    /// `t_mag_c` (PHYSICS §2).
    pub fn eval(&self, geom: &ElementGeom, i_a: f64, t_mag_c: f64) -> ElementEval {
        let perm = self.permeance(geom.g_m, geom.y_m, geom.e, geom.de_dx_per_m);
        let mmf = self.m_pm_at_temp(t_mag_c) + self.turns * i_a;
        let phi_lin = mmf * perm.p_h;
        let (phi, s) = saturate(phi_lin, self.phi_sat_wb);
        let dm_dt = self.m_pm_at * self.tempco_br_per_k;
        ElementEval {
            perm,
            mmf_at: mmf,
            phi_lin_wb: phi_lin,
            phi_wb: phi,
            sat_slope: s,
            dphi_di: self.turns * perm.p_h * s,
            dphi_dg: mmf * perm.dp_dg * s,
            dphi_dy: mmf * perm.dp_dy * s,
            dphi_dx: mmf * perm.dp_dx * s,
            dphi_dt: dm_dt * perm.p_h * s,
        }
    }

    /// Linear co-energy W′(g, y, e, i) at temperature `t_mag_c`, J (PHYSICS §2).
    pub fn coenergy_j(&self, g_m: f64, y_m: f64, e: f64, i_a: f64, t_mag_c: f64) -> f64 {
        let p = self.permeance(g_m, y_m, e, 0.0).p_h;
        let mmf = self.m_pm_at_temp(t_mag_c) + self.turns * i_a;
        0.5 * mmf * mmf * p
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::derive::{self, G0_MPS2};
    use fabrail_params::presets;

    fn door() -> SuspensionDesign {
        presets::door().design.suspension
    }

    /// Smooth engagement for the x-gradient check: e(x) = 0.6 + 0.3·sin(x/0.02).
    fn e_of(x: f64) -> (f64, f64) {
        (0.6 + 0.3 * (x / 0.02).sin(), 0.3 / 0.02 * (x / 0.02).cos())
    }

    // req: FAB-LEV-002
    #[test]
    fn coenergy_force_consistency() {
        // F_q = ∂W′/∂q by central finite differences (tolerance 1e-7 rel, core §E.2 plant
        // row; physics perspective A4) for g, y and x, with saturation off, across
        // presets, currents and offsets.
        for (name, p) in [
            ("drawer", presets::drawer()),
            ("door", presets::door()),
            ("wall", presets::wall()),
            ("heavy", presets::heavy()),
        ] {
            let s = p.design.suspension;
            let m = ElementModel::new(&s, s.b_r_t, 0.0, true).unsaturated();
            for &(g, y, x, i) in &[
                (s.g_star_m, 0.0, 0.0, 0.0),
                (s.g_star_m * 0.9, 0.3e-3, 0.004, 1.5),
                (s.g_star_m * 1.1, -1.2e-3, -0.007, -2.0),
                (s.g_up_m, 2.0e-3, 0.011, 3.0),
            ] {
                let (e, de) = e_of(x);
                let ev = m.eval(
                    &ElementGeom {
                        g_m: g,
                        y_m: y,
                        e,
                        de_dx_per_m: de,
                    },
                    i,
                    T_REF_C,
                );
                let l = ev.loads(ev.phi_wb);
                let w = |g: f64, y: f64, x: f64| m.coenergy_j(g, y, e_of(x).0, i, T_REF_C);
                let dg = 1.0e-7 * g;
                let dy = 1.0e-7;
                let dx = 1.0e-7;
                let fd_g = -(w(g + dg, y, x) - w(g - dg, y, x)) / (2.0 * dg);
                let fd_y = (w(g, y + dy, x) - w(g, y - dy, x)) / (2.0 * dy);
                let fd_x = (w(g, y, x + dx) - w(g, y, x - dx)) / (2.0 * dx);
                let scale = l.f_attract_n.abs();
                assert!(
                    (l.f_attract_n - fd_g).abs() <= 1e-7 * scale,
                    "{name} F_attract {} vs FD {fd_g} (1e-7 rel, PHYSICS §2)",
                    l.f_attract_n
                );
                assert!(
                    (l.fy_n - fd_y).abs() <= 1e-7 * scale,
                    "{name} F_y {} vs FD {fd_y} (1e-7 of F_z, PHYSICS §2)",
                    l.fy_n
                );
                assert!(
                    (l.fx_n - fd_x).abs() <= 1e-7 * scale,
                    "{name} F_x {} vs FD {fd_x} (1e-7 of F_z, PHYSICS §2)",
                    l.fx_n
                );
            }
        }
    }

    // req: FAB-LEV-002
    #[test]
    fn closed_form_y0() {
        // At y = 0, e = 1: F = μ0·A_p·(M + N·i)²/(2g + h′)² (E, rel 1e-12, PHYSICS §2),
        // equal to params::derive::pm_force; hold-down: μ0·A_p·N²·i²/(4g²).
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let s = p.design.suspension;
            let m = ElementModel::new(&s, s.b_r_t, 0.0, true);
            let hp = derive::h_prime(s.h_m_m, s.mu_rec);
            let mpm = derive::m_pm(s.b_r_t, s.h_m_m, s.mu_rec);
            for &(g, i) in &[
                (s.g_star_m, 0.0),
                (s.g_up_m, 0.0),
                (s.g_land_m, 0.0),
                (s.g_star_m, 2.0),
                (s.g_star_m, -1.0),
            ] {
                let ev = m.eval(
                    &ElementGeom {
                        g_m: g,
                        y_m: 0.0,
                        e: 1.0,
                        de_dx_per_m: 0.0,
                    },
                    i,
                    T_REF_C,
                );
                let f = ev.loads(ev.phi_wb).f_attract_n;
                let mmf = mpm + s.turns * i;
                let d = 2.0 * g + hp;
                let want = MU0_H_PER_M * s.pole_w_m * s.pole_l_m * mmf * mmf / (d * d);
                assert!(
                    (f - want).abs() <= 1e-12 * want,
                    "F {f} vs closed form {want} (E rel 1e-12, PHYSICS §2)"
                );
                let dr = derive::pm_force(&s, g, 0.0, i);
                assert!(
                    (f - dr).abs() <= 1e-12 * want,
                    "F {f} vs derive::pm_force {dr} (E rel 1e-12, PHYSICS §2)"
                );
                assert_eq!(ev.loads(ev.phi_wb).fy_n, 0.0, "no lateral force at y = 0");
                // The public frozen helper agrees.
                let ef = element_force(&s, s.b_r_t, g, 0.0, i, 1.0);
                assert!((ef.f_attract_n - want).abs() <= 1e-12 * want);
            }
            let hd = ElementModel::new(&s, s.b_r_t, 0.0, false);
            let (g, i) = (s.g_star_m, 3.0);
            let ev = hd.eval(
                &ElementGeom {
                    g_m: g,
                    y_m: 0.0,
                    e: 1.0,
                    de_dx_per_m: 0.0,
                },
                i,
                T_REF_C,
            );
            let want =
                MU0_H_PER_M * s.pole_w_m * s.pole_l_m * (s.turns * i).powi(2) / (4.0 * g * g);
            let f = ev.loads(ev.phi_wb).f_attract_n;
            assert!(
                (f - want).abs() <= 1e-12 * want,
                "hold-down F {f} vs μ0·A_p·N²i²/(4g²) {want} (E, PHYSICS §2)"
            );
        }
    }

    // req: FAB-LEV-002
    #[test]
    fn saturation_bound() {
        // |Φ| ≤ Φ_sat = B_sat·A_p for any MMF; the attraction at y = 0 is then bounded by
        // B_sat²·A_p/μ0 (= derive::element_force_max's cap); below the knee the model is
        // exactly linear; the curve is monotone (PHYSICS §2).
        let s = door();
        let m = ElementModel::new(&s, s.b_r_t, 0.0, true);
        let a_p = s.pole_w_m * s.pole_l_m;
        let phi_sat = s.b_sat_t * a_p;
        assert!((m.phi_sat_wb - phi_sat).abs() <= 1e-15);
        let f_cap = s.b_sat_t * s.b_sat_t * a_p / MU0_H_PER_M;
        let mut last_phi = f64::NEG_INFINITY;
        for k in -400..=400 {
            let i = k as f64 * 0.5; // −200 … 200 A (far beyond i_pulse)
            let ev = m.eval(
                &ElementGeom {
                    g_m: 0.5e-3,
                    y_m: 0.0,
                    e: 1.0,
                    de_dx_per_m: 0.0,
                },
                i,
                T_REF_C,
            );
            assert!(
                ev.phi_wb.abs() <= phi_sat * (1.0 + 1e-12),
                "|Φ| {} ≤ Φ_sat {phi_sat} at i {i} (PHYSICS §2)",
                ev.phi_wb
            );
            assert!(ev.phi_wb >= last_phi, "Φ monotone in i (PHYSICS §2)");
            last_phi = ev.phi_wb;
            let f = ev.loads(ev.phi_wb).f_attract_n;
            assert!(
                f <= f_cap * (1.0 + 1e-12),
                "F {f} ≤ B_sat²·A_p/μ0 {f_cap} (PHYSICS §2)"
            );
            assert!(ev.sat_slope >= 0.0 && ev.sat_slope <= 1.0);
        }
        // Deep saturation approaches the cap from below.
        let ev = m.eval(
            &ElementGeom {
                g_m: 0.2e-3,
                y_m: 0.0,
                e: 1.0,
                de_dx_per_m: 0.0,
            },
            2.0e3,
            T_REF_C,
        );
        assert!(ev.phi_wb > 0.999 * phi_sat, "Φ → Φ_sat (tanh, PHYSICS §2)");
        // The design operating point is below the knee: exactly linear.
        let ev = m.eval(
            &ElementGeom {
                g_m: s.g_star_m,
                y_m: 0.0,
                e: 1.0,
                de_dx_per_m: 0.0,
            },
            0.0,
            T_REF_C,
        );
        assert_eq!(
            ev.phi_wb, ev.phi_lin_wb,
            "linear below the knee (PHYSICS §2)"
        );
        assert!(ev.phi_lin_wb < SAT_KNEE_FRAC * phi_sat);
        // Knee continuity (C¹): the slope is 1 on both sides.
        let knee = SAT_KNEE_FRAC * phi_sat;
        let (a, sa) = saturate(knee * (1.0 - 1e-12), phi_sat);
        let (b, sb) = saturate(knee * (1.0 + 1e-12), phi_sat);
        assert!((a - b).abs() < 1e-11 * phi_sat && (sa - sb).abs() < 1e-9);
    }

    // req: FAB-LEV-001
    #[test]
    fn stiffness_sum_nonpositive() {
        // Earnshaw: with fixed currents the magnetic potential U = −W′ has a translational
        // stiffness trace K_gg + K_yy + K_xx ≤ 0, so no passive stable levitation; the
        // lateral term is the stabilising reluctance (K_yy > 0) and the gap term is
        // destabilising (K_gg < 0). FD second derivatives, per element and per preset
        // (PHYSICS §2; Earnshaw 1842).
        for (name, p) in [
            ("drawer", presets::drawer()),
            ("door", presets::door()),
            ("wall", presets::wall()),
            ("heavy", presets::heavy()),
        ] {
            let s = p.design.suspension;
            let m = ElementModel::new(&s, s.b_r_t, 0.0, true).unsaturated();
            for &(g, y, i) in &[
                (s.g_star_m, 0.0, 0.0),
                (s.g_star_m, 0.5e-3, 0.0),
                (s.zp_band_m[0], -0.2e-3, 1.0),
                (s.zp_band_m[1], 1.0e-3, -1.0),
            ] {
                let u = |g: f64, y: f64, x: f64| -m.coenergy_j(g, y, e_of(x).0, i, T_REF_C);
                let x = 0.003;
                let hg = 1e-6;
                let hy = 1e-6;
                let hx = 1e-5;
                let k_gg = (u(g + hg, y, x) - 2.0 * u(g, y, x) + u(g - hg, y, x)) / (hg * hg);
                let k_yy = (u(g, y + hy, x) - 2.0 * u(g, y, x) + u(g, y - hy, x)) / (hy * hy);
                let k_xx = (u(g, y, x + hx) - 2.0 * u(g, y, x) + u(g, y, x - hx)) / (hx * hx);
                assert!(
                    k_gg < 0.0,
                    "{name}: K_gg {k_gg} < 0 (gap unstable, PHYSICS §2)"
                );
                assert!(
                    k_yy > 0.0 || y != 0.0,
                    "{name}: K_yy {k_yy} > 0 at y = 0 (reluctance centring)"
                );
                let sum = k_gg + k_yy + k_xx;
                assert!(
                    sum <= 1e-6 * k_gg.abs(),
                    "{name}: K_gg + K_yy + K_xx = {sum} ≤ 0 (Earnshaw, PHYSICS §2)"
                );
            }
        }
        // DOOR per element at g*: lateral reluctance k_y = F/w = 12.3 kN/m (physics
        // perspective §1.3) against the vertical k_s = 4F/(2g* + h′); the sum is ≤ 0
        // because 2g* + h′ ≤ 4w (PHYSICS §2).
        let s = door();
        let ks = derive::k_s(&s, s.g_star_m, 0.0);
        let f = derive::pm_force(&s, s.g_star_m, 0.0, 0.0);
        let k_y = f / s.pole_w_m;
        assert!(
            (k_y - 12.26e3).abs() < 0.05e3,
            "DOOR k_y {k_y} ≈ 12.3 kN/m (physics perspective §1.3)"
        );
        assert!(ks > k_y, "vertical {ks} > lateral {k_y} N/m (Earnshaw)");
        assert!(f * 4.0 / G0_MPS2 > 0.0);
    }

    #[test]
    fn small_y_lateral() {
        // For small offsets F_y/F_z = −(g/w)·tanh(y/g) within 1 % for |y| ≤ 0.2·g
        // (physics perspective A4; PHYSICS §2).
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let s = p.design.suspension;
            let m = ElementModel::new(&s, s.b_r_t, 0.0, true);
            let g = s.g_star_m;
            for k in 1..=10 {
                let y = 0.02 * k as f64 * g;
                for y in [y, -y] {
                    let ev = m.eval(
                        &ElementGeom {
                            g_m: g,
                            y_m: y,
                            e: 1.0,
                            de_dx_per_m: 0.0,
                        },
                        0.0,
                        T_REF_C,
                    );
                    let l = ev.loads(ev.phi_wb);
                    let want = -(g / s.pole_w_m) * (y / g).tanh();
                    let got = l.fy_n / l.f_attract_n;
                    assert!(
                        (got - want).abs() <= 0.01 * want.abs(),
                        "F_y/F_z {got} vs −(g/w)·tanh(y/g) {want} (1 %, PHYSICS §2)"
                    );
                }
            }
        }
    }

    #[test]
    fn tempco_and_leakage_scale_the_mmf() {
        // B_r(T) = B_r·(1 − 0.0012·ΔT) and k_σ = 1/(1 + σ) (PHYSICS §2, §12).
        let s = door();
        let m0 = ElementModel::new(&s, s.b_r_t, 0.0, true);
        let m1 = ElementModel::new(&s, s.b_r_t, 0.25, true);
        assert!((m1.m_pm_at - m0.m_pm_at / 1.25).abs() <= 1e-12 * m0.m_pm_at);
        let hot = m0.m_pm_at_temp(60.0);
        assert!((hot - m0.m_pm_at * (1.0 - 0.0012 * 40.0)).abs() <= 1e-12 * m0.m_pm_at);
        // ∂Φ/∂T matches a finite difference.
        let geom = ElementGeom {
            g_m: s.g_star_m,
            y_m: 0.0,
            e: 1.0,
            de_dx_per_m: 0.0,
        };
        let ev = m0.eval(&geom, 0.5, 30.0);
        let fd = (m0.eval(&geom, 0.5, 30.001).phi_wb - m0.eval(&geom, 0.5, 29.999).phi_wb) / 0.002;
        assert!((ev.dphi_dt - fd).abs() <= 1e-9 * fd.abs());
    }
}
