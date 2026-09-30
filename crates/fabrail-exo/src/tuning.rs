//! TMD tuning helpers: Den Hartog; Warburton in base-acceleration and force (wind) forms; zeta normalised to omega_s (FAB-EXO-601). Owner: X2 (complete in Wave 0, W0-C1; extended in Wave 1).
//!
//! EXOTIC §X6, §X7; plan §4 "Controllers: tuning helpers". Design-time f64 closed forms
//! (gains are computed once, then cast to f32 by the HMD in `new()`). The closed forms
//! are the single-source implementations in `fabrail_params::derive` (PHYSICS §21);
//! this module names them for the damper, records the frequency each published ζ is
//! normalised to, and adds the dimensional design.
//!
//! | Form | f = ω_d/ω_s | ζ as published | basis | μ 0.01 (f / ζ / ζ_s) | μ 0.02 (f / ζ / ζ_s) |
//! |---|---|---|---|---|---|
//! | Den Hartog (harmonic force, undamped main mass) | 1/(1+μ) | √(3μ/(8(1+μ)³)) | ω_s | 0.99010 / 0.06033 / 0.06033 | 0.98039 / 0.08407 / 0.08407 |
//! | Warburton, base acceleration (white noise) | √(1−μ/2)/(1+μ) | √(μ(1−μ/4)/(4(1+μ)(1−μ/2))) | ω_d | 0.98762 / 0.04981 / 0.04920 | 0.97548 / 0.07019 / 0.06847 |
//! | Warburton, force on the main mass (wind, white noise) | √(1+μ/2)/(1+μ) | √(μ(1+3μ/4)/(4(1+μ)(1+μ/2))) | ω_d | 0.99257 / 0.04981 / 0.04944 | 0.98528 / 0.07019 / 0.06915 |
//!
//! Sources: Den Hartog, *Mechanical Vibrations* (1956), with c_c = 2·m_d·ω_s (Brock's
//! equal-peak damping); Warburton, "Optimum absorber parameters for various combinations
//! of response and excitation parameters", EESD 10 (1982), displacement-minimising
//! optima for white-noise excitation (the same closed forms as the H2 optima of Asami,
//! Nishihara & Baz, J. Vib. Acoust. 124 (2002), whose ζ is c_d/(2·m_d·ω_d)).
//!
//! **ζ normalisation (plan §4: "ζ is normalised to ω_s").** The published ζ values (the
//! EXOTIC §2 anchors) do not share one basis. Checked numerically in Wave 1 on the
//! undamped 2-DOF model (tests below; H2 by the Lyapunov equation, H∞ by a frequency
//! sweep):
//!
//! * the Warburton f is the exact H2 optimum, and the Warburton ζ is the exact H2
//!   optimum **only** as ζ_d = c_d/(2·m_d·ω_d); as c_d/(2·m_d·ω_s) the optimum is
//!   ζ_s = f·ζ_d (1.2 % lower at μ 0.01, 2.5 % at μ 0.02);
//! * the Den Hartog ζ is normalised to ω_s: the numeric minimax optimum at f = 1/(1+μ)
//!   is 0.06035 at μ 0.01 (formula 0.06033; ζ_d/f would be 0.06093).
//!
//! [`TmdTuning`] from [`den_hartog`], [`warburton_base`] and [`warburton_force`] keeps
//! the published ζ (anchors, rel 1e-12); [`TuningForm::omega_s`] re-expresses it on ω_s
//! (ζ_s = f·ζ_d), and every dimensional output here ([`damper_design_for`],
//! [`dvf_damping_n_s_per_m`]) uses ζ_s: c_d = 2·ζ_s·m_d·ω_s.
//!
//! The added damping of a stroke-limited damper is bounded by ζ_add ≤ (μ/2)·(S/X)
//! (EXOTIC §X6; 3.5 % at μ 1 %, S/X 7; 0.7 % at S/X 1.4).

use fabrail_params::derive;

/// Optimal absorber tuning: frequency ratio f = ω_d/ω_s and damping ratio ζ.
///
/// `zeta` is the value of the published closed form, in that form's own basis
/// ([`TuningForm::zeta_basis`]): ω_s for Den Hartog, ω_d for both Warburton forms. Use
/// [`TuningForm::omega_s`] (or [`TmdTuning::zeta_s`]) for the ω_s-normalised value.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TmdTuning {
    /// Frequency ratio f = ω_d/ω_s.
    pub f_ratio: f64,
    /// Damping ratio ζ as published (see [`TuningForm::zeta_basis`]).
    pub zeta: f64,
}

impl TmdTuning {
    /// ζ re-expressed on the main-structure frequency ω_s, given the basis `zeta` is
    /// normalised to: c_d = 2·ζ_d·m_d·ω_d = 2·(f·ζ_d)·m_d·ω_s, so ζ_s = f·ζ_d.
    pub fn zeta_s(self, basis: ZetaBasis) -> f64 {
        match basis {
            ZetaBasis::OmegaS => self.zeta,
            ZetaBasis::OmegaD => self.f_ratio * self.zeta,
        }
    }
}

/// The frequency a damping ratio is normalised to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ZetaBasis {
    /// ζ = c_d/(2·m_d·ω_s), main-structure frequency (plan §4 convention).
    OmegaS = 0,
    /// ζ = c_d/(2·m_d·ω_d), absorber frequency.
    OmegaD = 1,
}

/// A published absorber optimum.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum TuningForm {
    /// Den Hartog equal-peak optimum (harmonic force on an undamped main mass).
    DenHartog = 0,
    /// Warburton H2 optimum for base-acceleration (seismic) white noise.
    WarburtonBase = 1,
    /// Warburton H2 optimum for force (wind) white noise on the main mass.
    WarburtonForce = 2,
}

impl TuningForm {
    /// Every form.
    pub const ALL: [TuningForm; 3] = [
        TuningForm::DenHartog,
        TuningForm::WarburtonBase,
        TuningForm::WarburtonForce,
    ];

    /// Snake-case name.
    pub const fn slug(self) -> &'static str {
        match self {
            TuningForm::DenHartog => "den_hartog",
            TuningForm::WarburtonBase => "warburton_base",
            TuningForm::WarburtonForce => "warburton_force",
        }
    }

    /// The basis the published ζ is normalised to (verified numerically, module doc).
    pub const fn zeta_basis(self) -> ZetaBasis {
        match self {
            TuningForm::DenHartog => ZetaBasis::OmegaS,
            TuningForm::WarburtonBase | TuningForm::WarburtonForce => ZetaBasis::OmegaD,
        }
    }

    /// The closed form as published (EXOTIC §2 anchors).
    pub fn published(self, mu: f64) -> TmdTuning {
        match self {
            TuningForm::DenHartog => den_hartog(mu),
            TuningForm::WarburtonBase => warburton_base(mu),
            TuningForm::WarburtonForce => warburton_force(mu),
        }
    }

    /// The optimum with ζ normalised to ω_s (plan §4): ζ_s = f·ζ_d for the Warburton
    /// forms, unchanged for Den Hartog.
    pub fn omega_s(self, mu: f64) -> TmdTuning {
        let t = self.published(mu);
        TmdTuning {
            f_ratio: t.f_ratio,
            zeta: t.zeta_s(self.zeta_basis()),
        }
    }
}

/// Den Hartog optimum (harmonic force, undamped main mass), mass ratio μ = m_d/m_s;
/// ζ normalised to ω_s.
pub fn den_hartog(mu: f64) -> TmdTuning {
    let (f_ratio, zeta) = derive::den_hartog(mu);
    TmdTuning { f_ratio, zeta }
}

/// Warburton optimum for base-acceleration (seismic) white-noise excitation; ζ as
/// published, normalised to ω_d (use [`TuningForm::omega_s`] for ζ_s).
pub fn warburton_base(mu: f64) -> TmdTuning {
    let (f_ratio, zeta) = derive::warburton(mu);
    TmdTuning { f_ratio, zeta }
}

/// Warburton optimum for force (wind) white-noise excitation on the main mass; ζ as
/// published, normalised to ω_d (use [`TuningForm::omega_s`] for ζ_s).
pub fn warburton_force(mu: f64) -> TmdTuning {
    let (f_ratio, zeta) = derive::warburton_force(mu);
    TmdTuning { f_ratio, zeta }
}

/// Den Hartog equal-peak amplification √(1 + 2/μ) of the undamped main mass
/// (FAB-EXO-602: 14.177 at μ 0.01, 10.050 at μ 0.02).
pub fn den_hartog_peak(mu: f64) -> f64 {
    derive::den_hartog_peak(mu)
}

/// Upper bound on the damping a stroke-limited damper adds, ζ_add ≤ (μ/2)·(S/X), with
/// S the damper stroke and X the structure amplitude (EXOTIC §X6; FAB-EXO-603).
pub fn zeta_add_bound(mu: f64, stroke_ratio: f64) -> f64 {
    if mu <= 0.0 || stroke_ratio <= 0.0 {
        return 0.0;
    }
    0.5 * mu * stroke_ratio
}

/// Dimensional damper design.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DamperDesign {
    /// Damper mass m_d = μ·m_s, kg.
    pub m_d_kg: f64,
    /// Damper frequency ω_d = f·ω_s, rad/s.
    pub omega_d_rad_s: f64,
    /// Damper stiffness k_d = m_d·ω_d², N/m (passive guideway plus LSM trim).
    pub k_d_n_per_m: f64,
    /// Damper damping c_d = 2·ζ_s·m_d·ω_s, N·s/m (ζ normalised to ω_s).
    pub c_d_n_s_per_m: f64,
}

/// Designs a damper of mass ratio `mu` for a structure of modal mass `m_s_kg` and
/// frequency `omega_s_rad_s` with tuning `t`, whose `zeta` must be normalised to ω_s
/// ([`den_hartog`] or [`TuningForm::omega_s`]); prefer [`damper_design_for`].
pub fn damper_design(m_s_kg: f64, omega_s_rad_s: f64, mu: f64, t: TmdTuning) -> DamperDesign {
    let m_d = mu * m_s_kg;
    let omega_d = t.f_ratio * omega_s_rad_s;
    DamperDesign {
        m_d_kg: m_d,
        omega_d_rad_s: omega_d,
        k_d_n_per_m: m_d * omega_d * omega_d,
        c_d_n_s_per_m: 2.0 * t.zeta * m_d * omega_s_rad_s,
    }
}

/// Designs a damper with the optimum of `form`, ζ normalised to ω_s (plan §4).
pub fn damper_design_for(
    m_s_kg: f64,
    omega_s_rad_s: f64,
    mu: f64,
    form: TuningForm,
) -> DamperDesign {
    damper_design(m_s_kg, omega_s_rad_s, mu, form.omega_s(mu))
}

/// DVF fallback gain c_dvf = c_d,opt − c_eddy, N·s/m (exotic design §3.1: the LSM
/// supplies the optimum damping the passive eddy damping lacks; c_d,opt = 2·ζ_s·m_d·ω_s,
/// ζ normalised to ω_s). Never negative: an over-damped passive stage gets no DVF.
pub fn dvf_damping_n_s_per_m(d: &DamperDesign, c_eddy_n_s_per_m: f64) -> f64 {
    let c = d.c_d_n_s_per_m - c_eddy_n_s_per_m;
    if c.is_finite() && c > 0.0 { c } else { 0.0 }
}

#[cfg(test)]
pub(crate) mod testkit {
    //! Test-only numerics shared by the exo tests (std math allowed in tests).

    /// Solves the dense system `a·x = b` in place by Gaussian elimination with partial
    /// pivoting; returns false if singular.
    pub fn solve<const N: usize>(a: &mut [[f64; N]; N], b: &mut [f64; N]) -> bool {
        for col in 0..N {
            let piv = (col..N)
                .max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))
                .unwrap_or(col);
            if a[piv][col].abs() < 1e-300 {
                return false;
            }
            a.swap(col, piv);
            b.swap(col, piv);
            let pivot_row = a[col];
            let pivot_b = b[col];
            for (row, bi) in a.iter_mut().zip(b.iter_mut()).skip(col + 1) {
                let m = row[col] / pivot_row[col];
                for (c, p) in row.iter_mut().zip(pivot_row.iter()).skip(col) {
                    *c -= m * p;
                }
                *bi -= m * pivot_b;
            }
        }
        for col in (0..N).rev() {
            let mut s = b[col];
            for k in col + 1..N {
                s -= a[col][k] * b[k];
            }
            b[col] = s / a[col][col];
        }
        true
    }

    /// Solves the Lyapunov equation A·P + P·Aᵀ + Q = 0 (A Hurwitz) for P.
    pub fn lyapunov4(a: &[[f64; 4]; 4], q: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
        // vec(P) column-major, p = i + 4j: (I⊗A + A⊗I)·vec(P) = −vec(Q).
        let mut m = [[0.0; 16]; 16];
        let mut rhs = [0.0; 16];
        for i in 0..4 {
            for j in 0..4 {
                let row = i + 4 * j;
                rhs[row] = -q[i][j];
                for k in 0..4 {
                    m[row][k + 4 * j] += a[i][k];
                    m[row][i + 4 * k] += a[j][k];
                }
            }
        }
        assert!(solve(&mut m, &mut rhs), "Lyapunov system singular");
        let mut p = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                p[i][j] = rhs[i + 4 * j];
            }
        }
        p
    }

    /// Minimises a unimodal `f` on [lo, hi] by golden-section search (tolerance
    /// `tol` on the abscissa).
    pub fn golden(mut lo: f64, mut hi: f64, tol: f64, f: impl Fn(f64) -> f64) -> f64 {
        let g = (5.0_f64.sqrt() - 1.0) / 2.0;
        let mut c = hi - g * (hi - lo);
        let mut d = lo + g * (hi - lo);
        let (mut fc, mut fd) = (f(c), f(d));
        while hi - lo > tol {
            if fc < fd {
                hi = d;
                d = c;
                fd = fc;
                c = hi - g * (hi - lo);
                fc = f(c);
            } else {
                lo = c;
                c = d;
                fc = fd;
                d = lo + g * (hi - lo);
                fd = f(d);
            }
        }
        0.5 * (lo + hi)
    }
}

#[cfg(test)]
mod tests {
    use super::testkit::{golden, lyapunov4};
    use super::*;

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    /// Closed forms evaluated independently with std math, for the rel 1e-12 check.
    fn ref_dh(mu: f64) -> (f64, f64) {
        (
            1.0 / (1.0 + mu),
            (3.0 * mu / (8.0 * (1.0 + mu).powi(3))).sqrt(),
        )
    }
    fn ref_wb(mu: f64) -> (f64, f64) {
        (
            (1.0 - mu / 2.0).sqrt() / (1.0 + mu),
            (mu * (1.0 - mu / 4.0) / (4.0 * (1.0 + mu) * (1.0 - mu / 2.0))).sqrt(),
        )
    }
    fn ref_wf(mu: f64) -> (f64, f64) {
        (
            (1.0 + mu / 2.0).sqrt() / (1.0 + mu),
            (mu * (1.0 + 3.0 * mu / 4.0) / (4.0 * (1.0 + mu) * (1.0 + mu / 2.0))).sqrt(),
        )
    }

    /// Main-mass displacement variance of the undamped 2-DOF model (m_s = k_s = ω_s = 1)
    /// under unit white noise, from the Lyapunov equation (EXOTIC §X6). `base`: base
    /// acceleration (relative displacement), else force on the main mass. `zeta_s` is
    /// normalised to ω_s: c_d = 2·ζ_s·m_d·ω_s.
    fn h2_var(mu: f64, f: f64, zeta_s: f64, base: bool) -> f64 {
        let (md, kd, cd) = (mu, mu * f * f, 2.0 * zeta_s * mu);
        // State [x_s, x_d, v_s, v_d] (relative to the ground for base excitation).
        let a = [
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [-(1.0 + kd), kd, -cd, cd],
            [kd / md, -kd / md, cd / md, -cd / md],
        ];
        let b = if base {
            [0.0, 0.0, -1.0, -1.0]
        } else {
            [0.0, 0.0, 1.0, 0.0]
        };
        let mut q = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                q[i][j] = b[i] * b[j];
            }
        }
        lyapunov4(&a, &q)[0][0]
    }

    /// Peak |X_s/X_st| of the undamped main mass under harmonic force (Den Hartog),
    /// over g = ω/ω_s in [0.8, 1.2] (Δg 1e-5).
    fn dh_peak(mu: f64, f: f64, zeta_s: f64) -> f64 {
        let mut peak: f64 = 0.0;
        for k in 0..=40_000 {
            let g = 0.8 + 1e-5 * k as f64;
            let (nr, ni) = (f * f - g * g, 2.0 * zeta_s * g);
            let dr = (1.0 - g * g) * (f * f - g * g) - mu * f * f * g * g;
            let di = 2.0 * zeta_s * g * (1.0 - g * g - mu * g * g);
            peak = peak.max(((nr * nr + ni * ni) / (dr * dr + di * di)).sqrt());
        }
        peak
    }

    // req: FAB-EXO-601
    #[test]
    fn den_hartog_warburton_closed_forms() {
        // 1. Numeric verification first (Wave-1 brief: check the forms against the
        //    2-DOF optimum before asserting anchors). Warburton: the published (f, ζ)
        //    is the H2 optimum with ζ on ω_d; on ω_s the optimum is f·ζ_d.
        for mu in [0.01, 0.02] {
            for (form, base) in [
                (TuningForm::WarburtonBase, true),
                (TuningForm::WarburtonForce, false),
            ] {
                let t = form.published(mu);
                let zs = t.f_ratio * t.zeta;
                let z_opt = golden(0.5 * zs, 1.5 * zs, 1e-10, |z| {
                    h2_var(mu, t.f_ratio, z, base)
                });
                let f_opt = golden(0.9 * t.f_ratio, 1.1 * t.f_ratio, 1e-10, |f| {
                    h2_var(mu, f, zs, base)
                });
                assert!(
                    rel(z_opt, zs) < 1e-5,
                    "{} mu {mu}: H2-optimal zeta_s {z_opt} vs f*zeta_d {zs} (rel 1e-5, Warburton 1982 / Lyapunov)",
                    form.slug()
                );
                assert!(
                    rel(z_opt, t.zeta) > 5e-3,
                    "{} mu {mu}: published zeta must NOT be the omega_s optimum",
                    form.slug()
                );
                assert!(
                    rel(f_opt, t.f_ratio) < 1e-6,
                    "{} mu {mu}: H2-optimal f {f_opt} vs {} (rel 1e-6)",
                    form.slug(),
                    t.f_ratio
                );
                // Physics perspective A17: ±5 % perturbations each increase σ².
                let v0 = h2_var(mu, t.f_ratio, zs, base);
                for (df, dz) in [(1.05, 1.0), (0.95, 1.0), (1.0, 1.05), (1.0, 0.95)] {
                    assert!(
                        h2_var(mu, t.f_ratio * df, zs * dz, base) > v0,
                        "{} mu {mu}: not a local minimum at ({df}, {dz})",
                        form.slug()
                    );
                }
                assert_eq!(form.zeta_basis(), ZetaBasis::OmegaD);
                assert!(rel(form.omega_s(mu).zeta, zs) < 1e-15);
            }
            // Den Hartog: the minimax ζ at f = 1/(1+μ) is the published value on ω_s
            // (Brock's formula is within 0.1 %; the ω_d reading is ~1 % off).
            let t = den_hartog(mu);
            let z_opt = golden(0.5 * t.zeta, 1.5 * t.zeta, 1e-7, |z| {
                dh_peak(mu, t.f_ratio, z)
            });
            assert!(
                rel(z_opt, t.zeta) < 1e-3,
                "DH mu {mu}: minimax zeta_s {z_opt} vs {} (0.1 %, Den Hartog 1956)",
                t.zeta
            );
            assert!(rel(z_opt, t.zeta / t.f_ratio) > 5e-3);
            let peak = dh_peak(mu, t.f_ratio, t.zeta);
            assert!(
                rel(peak, den_hartog_peak(mu)) < 0.02,
                "DH mu {mu}: sweep peak {peak} vs sqrt(1+2/mu) (±2 %, FAB-EXO-602)"
            );
            assert_eq!(TuningForm::DenHartog.omega_s(mu), t);
        }

        // 2. Anchors (EXOTIC §2, 5 significant digits; tolerance ±5e-6 absolute).
        let cases = [
            (den_hartog(0.01), 0.99010, 0.06033),
            (den_hartog(0.02), 0.98039, 0.08407),
            (warburton_base(0.01), 0.98762, 0.04981),
            (warburton_base(0.02), 0.97548, 0.07019),
            (warburton_force(0.01), 0.99257, 0.04981),
            (warburton_force(0.02), 0.98528, 0.07019),
        ];
        for (t, f, z) in cases {
            assert!(
                (t.f_ratio - f).abs() < 5e-6,
                "f {} vs anchor {f} (±5e-6, EXOTIC §2)",
                t.f_ratio
            );
            assert!(
                (t.zeta - z).abs() < 5e-6,
                "zeta {} vs anchor {z} (±5e-6, EXOTIC §2)",
                t.zeta
            );
        }
        // ω_s-normalised Warburton values (module table).
        for (form, mu, zs) in [
            (TuningForm::WarburtonBase, 0.01, 0.04920),
            (TuningForm::WarburtonBase, 0.02, 0.06847),
            (TuningForm::WarburtonForce, 0.01, 0.04944),
            (TuningForm::WarburtonForce, 0.02, 0.06915),
        ] {
            let z = form.omega_s(mu).zeta;
            assert!((z - zs).abs() < 5e-6, "{}: zeta_s {z} vs {zs}", form.slug());
        }

        // 3. Exact closed forms, rel 1e-12 (EXOTIC §2 tolerance class E), over a μ sweep.
        for k in 1..=100 {
            let mu = 0.001 * k as f64;
            for (got, want) in [
                (den_hartog(mu), ref_dh(mu)),
                (warburton_base(mu), ref_wb(mu)),
                (warburton_force(mu), ref_wf(mu)),
            ] {
                assert!(rel(got.f_ratio, want.0) < 1e-12, "mu {mu}: f (rel 1e-12)");
                assert!(rel(got.zeta, want.1) < 1e-12, "mu {mu}: zeta (rel 1e-12)");
            }
        }
        // Equal-peak amplification (FAB-EXO-602 closed form).
        assert!((den_hartog_peak(0.01) - 14.177).abs() < 5e-4);
        assert!((den_hartog_peak(0.02) - 10.050).abs() < 5e-4);
        // Stroke bound (EXOTIC §2): 3.5 % at S/X 7, 0.7 % at 1.4, μ 1 %.
        assert!((zeta_add_bound(0.01, 7.0) - 0.035).abs() < 1e-15);
        assert!((zeta_add_bound(0.01, 1.4) - 0.007).abs() < 1e-15);
        assert_eq!(zeta_add_bound(0.0, 7.0), 0.0);

        // 4. ζ normalised to ω_s in every dimensional output: c_d = 2ζ_s·m_d·ω_s, which
        //    equals the published 2ζ_d·m_d·ω_d for the Warburton forms.
        let w_s = 2.0 * core::f64::consts::PI / 7.0;
        let d = damper_design(1.0e8, w_s, 0.01, den_hartog(0.01));
        assert!(rel(d.m_d_kg, 1.0e6) < 1e-15);
        assert!(rel(d.c_d_n_s_per_m, 2.0 * 0.0603 * 1.0e6 * w_s) < 1e-3);
        for form in TuningForm::ALL {
            let t = form.published(0.01);
            let d = damper_design_for(1.0e8, w_s, 0.01, form);
            assert!(rel(d.omega_d_rad_s, t.f_ratio * w_s) < 1e-15);
            assert!(rel(d.k_d_n_per_m, 1.0e6 * (t.f_ratio * w_s).powi(2)) < 1e-12);
            let c_pub = match form.zeta_basis() {
                ZetaBasis::OmegaS => 2.0 * t.zeta * d.m_d_kg * w_s,
                ZetaBasis::OmegaD => 2.0 * t.zeta * d.m_d_kg * d.omega_d_rad_s,
            };
            assert!(rel(d.c_d_n_s_per_m, c_pub) < 1e-12, "{}", form.slug());
            // DVF supplies what the eddy damping lacks, never negative.
            let c_eddy = 0.4 * d.c_d_n_s_per_m;
            assert!(rel(dvf_damping_n_s_per_m(&d, c_eddy), 0.6 * d.c_d_n_s_per_m) < 1e-12);
            assert_eq!(dvf_damping_n_s_per_m(&d, 2.0 * d.c_d_n_s_per_m), 0.0);
        }
    }
}
