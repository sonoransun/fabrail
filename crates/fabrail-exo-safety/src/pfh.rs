//! PFH calculator for 2oo3 in both forms (conservative and Ed.2) with a CTMC cross-check (FAB-EXO-305). Owner: X1 (API frozen by W0a).
//!
//! EXOTIC §X13 (IEC 61508-6 simplified equations; an analysis aid, not a certification
//! artefact; nothing here establishes a PFH for hardware). Host-side f64 algebra only
//! (no transcendental functions), usable from the no_std crate.
//!
//! With λ_DD = DC·λ_D, λ_DU = (1 − DC)·λ_D and the channel-equivalent down time
//!
//! t_CE = (λ_DU/λ_D)·(T1/2 + MRT) + (λ_DD/λ_D)·MTTR,
//!
//! * **Conservative form** (exotic design §2.4; detected failures stay dangerous until
//!   repaired, detected common cause is dangerous):
//!   PFH = 6·[(1 − β_D)·λ_DD + (1 − β)·λ_DU]²·t_CE + β_D·λ_DD + β·λ_DU.
//! * **IEC 61508-6 Ed. 2 high-demand form** (the second failure must be undetected; a
//!   detected failure, including a detected common cause, drives the safe state):
//!   PFH = 6·[(1 − β_D)·λ_DD + (1 − β)·λ_DU]·(1 − β)·λ_DU·t_CE + β·λ_DU
//!   (exotic critique: edition-dependent, about 50× apart; \[UNCERTAIN: recalled form\]).
//!
//! Reference case (λ_D 1e-6/h, DC 99 %, β 2 %, β_D 1 %, T1 8760 h, MTTR = MRT = 8 h):
//! t_CE = 51.8 h; conservative 1.04e-8/h; Ed. 2 2.03e-10/h. Common cause dominates, so
//! the conservative form reaches the SIL 4 band only with β_D ≤ about 0.5 %.
//!
//! **CTMC cross-check** (FAB-EXO-117, ±20 %): a multi-phase continuous-time Markov
//! chain in the style of IEC 61508-6 Annex B.5.2, integrated with RK4 over two proof-test
//! intervals. States: all channels good; one channel failed dangerous-detected (repair
//! rate 1/MTTR); one failed dangerous-undetected (no repair until the proof test); one
//! found at the proof test (repair rate 1/MRT); tripped to the safe state (Ed. 2 only,
//! renewed at 1/MTTR); dangerous system failure (absorbing). The proof test is
//! deterministic at T1 (the undetected state moves to repair), unlike the T1/2
//! averaging of t_CE. PFH is the failure flux of the second interval divided by the time
//! spent operating. It shares the rate model with the closed forms, so it checks the
//! algebra and the t_CE approximation, not the rates.

use fabrail_types::fmath;

/// Inputs of the 2oo3 PFH calculation (per-hour rates, hours).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PfhInputs {
    /// Dangerous failure rate per channel λ_D, 1/h.
    pub lambda_d_per_h: f64,
    /// Diagnostic coverage DC, 0..1.
    pub dc: f64,
    /// Common-cause factor for undetected failures β, 0..1.
    pub beta: f64,
    /// Common-cause factor for detected failures β_D, 0..1.
    pub beta_d: f64,
    /// Proof-test interval T1, h.
    pub t1_h: f64,
    /// Mean time to restoration after a detected failure MTTR, h.
    pub mttr_h: f64,
    /// Mean repair time after a proof test MRT, h.
    pub mrt_h: f64,
}

impl PfhInputs {
    /// The EXOTIC §2 reference case.
    pub const REFERENCE: PfhInputs = PfhInputs {
        lambda_d_per_h: 1.0e-6,
        dc: 0.99,
        beta: 0.02,
        beta_d: 0.01,
        t1_h: 8760.0,
        mttr_h: 8.0,
        mrt_h: 8.0,
    };

    /// λ_DD = DC·λ_D, 1/h.
    pub fn lambda_dd(&self) -> f64 {
        self.dc * self.lambda_d_per_h
    }

    /// λ_DU = (1 − DC)·λ_D, 1/h.
    pub fn lambda_du(&self) -> f64 {
        (1.0 - self.dc) * self.lambda_d_per_h
    }

    /// Inputs inside the calculator's domain: finite, λ_D > 0, fractions in [0, 1],
    /// positive T1, MTTR and MRT.
    pub fn is_valid(&self) -> bool {
        let frac = |x: f64| x.is_finite() && (0.0..=1.0).contains(&x);
        let pos = |x: f64| x.is_finite() && x > 0.0;
        pos(self.lambda_d_per_h)
            && frac(self.dc)
            && frac(self.beta)
            && frac(self.beta_d)
            && pos(self.t1_h)
            && pos(self.mttr_h)
            && pos(self.mrt_h)
    }
}

/// Channel-equivalent down time t_CE, h.
pub fn t_ce_h(p: &PfhInputs) -> f64 {
    if p.lambda_d_per_h <= 0.0 {
        return 0.0;
    }
    let l_dd = p.dc * p.lambda_d_per_h;
    let l_du = (1.0 - p.dc) * p.lambda_d_per_h;
    (l_du / p.lambda_d_per_h) * (0.5 * p.t1_h + p.mrt_h) + (l_dd / p.lambda_d_per_h) * p.mttr_h
}

/// Conservative 2oo3 PFH, 1/h (reference case 1.04e-8/h).
pub fn pfh_2oo3_conservative(p: &PfhInputs) -> f64 {
    let l_dd = p.dc * p.lambda_d_per_h;
    let l_du = (1.0 - p.dc) * p.lambda_d_per_h;
    let indep = (1.0 - p.beta_d) * l_dd + (1.0 - p.beta) * l_du;
    6.0 * indep * indep * t_ce_h(p) + p.beta_d * l_dd + p.beta * l_du
}

/// IEC 61508-6 Ed. 2 2oo3 PFH, 1/h (reference case 2.03e-10/h ≈ 2.0e-10/h).
pub fn pfh_2oo3_ed2(p: &PfhInputs) -> f64 {
    let l_dd = p.dc * p.lambda_d_per_h;
    let l_du = (1.0 - p.dc) * p.lambda_d_per_h;
    let indep = (1.0 - p.beta_d) * l_dd + (1.0 - p.beta) * l_du;
    6.0 * indep * (1.0 - p.beta) * l_du * t_ce_h(p) + p.beta * l_du
}

/// Continuous-time Markov-chain 2oo3 PFH under the conservative assumptions, 1/h
/// (cross-checks [`pfh_2oo3_conservative`] within ±20 %). NaN outside
/// [`PfhInputs::is_valid`] or when the repair times are too short against T1 to
/// integrate ([`CTMC_MAX_STEPS`]).
pub fn pfh_2oo3_ctmc(p: &PfhInputs) -> f64 {
    ctmc(p, Form::Conservative)
}

/// Continuous-time Markov-chain 2oo3 PFH under the Ed. 2 assumptions, 1/h
/// (cross-checks [`pfh_2oo3_ed2`] within ±20 %).
pub fn pfh_2oo3_ctmc_ed2(p: &PfhInputs) -> f64 {
    ctmc(p, Form::Ed2)
}

/// RK4 steps per proof-test interval above which the CTMC declines (NaN): the step is
/// at most a quarter of the shortest repair time and T1/256.
pub const CTMC_MAX_STEPS: u64 = 2_000_000;

/// Which failure assumptions the chain encodes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Form {
    Conservative,
    Ed2,
}

/// Chain state: probabilities plus the flux and operating-time integrals.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Chain {
    /// All three channels good.
    p0: f64,
    /// One channel failed dangerous-detected, under repair.
    pd: f64,
    /// One channel failed dangerous-undetected, awaiting the proof test.
    pu: f64,
    /// One channel found at the proof test, under repair.
    pr: f64,
    /// Tripped to the safe state (Ed. 2 only).
    ps: f64,
    /// Dangerous system failure (absorbing).
    pf: f64,
    /// ∫ failure flux dt.
    flux: f64,
    /// ∫ P(operating) dt, h.
    op: f64,
}

impl Chain {
    fn axpy(self, k: f64, d: Chain) -> Chain {
        Chain {
            p0: self.p0 + k * d.p0,
            pd: self.pd + k * d.pd,
            pu: self.pu + k * d.pu,
            pr: self.pr + k * d.pr,
            ps: self.ps + k * d.ps,
            pf: self.pf + k * d.pf,
            flux: self.flux + k * d.flux,
            op: self.op + k * d.op,
        }
    }
}

/// Transition rates, 1/h.
#[derive(Clone, Copy, Debug)]
struct Rates {
    /// Good → one detected failure: 3·(1 − β_D)·λ_DD.
    a_d: f64,
    /// Good → one undetected failure: 3·(1 − β)·λ_DU.
    a_u: f64,
    /// Common cause to the dangerous failure, from any operating state.
    ccf_f: f64,
    /// Common cause to the safe state (Ed. 2: the detected share).
    ccf_s: f64,
    /// One failed → second independent failure, dangerous.
    s1_f: f64,
    /// One failed → second independent failure, safe (Ed. 2: a detected one).
    s1_s: f64,
    /// Repair after detection, 1/MTTR (also the safe-state renewal).
    mu_d: f64,
    /// Repair after the proof test, 1/MRT.
    mu_r: f64,
}

impl Rates {
    fn new(p: &PfhInputs, form: Form) -> Rates {
        let (l_dd, l_du) = (p.lambda_dd(), p.lambda_du());
        let (i_dd, i_du) = ((1.0 - p.beta_d) * l_dd, (1.0 - p.beta) * l_du);
        let (c_dd, c_du) = (p.beta_d * l_dd, p.beta * l_du);
        let (ccf_f, ccf_s, s1_f, s1_s) = match form {
            Form::Conservative => (c_dd + c_du, 0.0, 2.0 * (i_dd + i_du), 0.0),
            Form::Ed2 => (c_du, c_dd, 2.0 * i_du, 2.0 * i_dd),
        };
        Rates {
            a_d: 3.0 * i_dd,
            a_u: 3.0 * i_du,
            ccf_f,
            ccf_s,
            s1_f,
            s1_s,
            mu_d: 1.0 / p.mttr_h,
            mu_r: 1.0 / p.mrt_h,
        }
    }

    /// Kolmogorov forward equations dπ/dt = π·Q (probability-conserving).
    fn deriv(&self, c: &Chain) -> Chain {
        let one_failed = c.pd + c.pu + c.pr;
        let out1 = self.s1_f + self.s1_s + self.ccf_f + self.ccf_s;
        let flux = c.p0 * self.ccf_f + one_failed * (self.s1_f + self.ccf_f);
        let to_safe = c.p0 * self.ccf_s + one_failed * (self.s1_s + self.ccf_s);
        Chain {
            p0: -c.p0 * (self.a_d + self.a_u + self.ccf_f + self.ccf_s)
                + c.pd * self.mu_d
                + c.pr * self.mu_r
                + c.ps * self.mu_d,
            pd: c.p0 * self.a_d - c.pd * (self.mu_d + out1),
            pu: c.p0 * self.a_u - c.pu * out1,
            pr: -c.pr * (self.mu_r + out1),
            ps: to_safe - c.ps * self.mu_d,
            pf: flux,
            flux,
            op: c.p0 + one_failed,
        }
    }
}

/// RK4 over one proof-test interval of `n` steps of `dt` hours.
fn integrate(r: &Rates, mut c: Chain, n: u64, dt: f64) -> Chain {
    for _ in 0..n {
        let k1 = r.deriv(&c);
        let k2 = r.deriv(&c.axpy(0.5 * dt, k1));
        let k3 = r.deriv(&c.axpy(0.5 * dt, k2));
        let k4 = r.deriv(&c.axpy(dt, k3));
        c = c
            .axpy(dt / 6.0, k1)
            .axpy(dt / 3.0, k2)
            .axpy(dt / 3.0, k3)
            .axpy(dt / 6.0, k4);
    }
    c
}

fn ctmc(p: &PfhInputs, form: Form) -> f64 {
    if !p.is_valid() {
        return f64::NAN;
    }
    let dt_target = fmath::fmin(0.25 * fmath::fmin(p.mttr_h, p.mrt_h), p.t1_h / 256.0);
    let n_f = fmath::ceil(p.t1_h / dt_target);
    if !(n_f >= 1.0 && n_f <= CTMC_MAX_STEPS as f64) {
        return f64::NAN;
    }
    let n = n_f as u64;
    let dt = p.t1_h / n as f64;
    let r = Rates::new(p, form);
    // Interval 1 from as-good-as-new, then the proof test, then interval 2 (periodic
    // steady state: the only memory left is the absorbed probability).
    let start = Chain {
        p0: 1.0,
        ..Chain::default()
    };
    let mut c = integrate(&r, start, n, dt);
    c.pr += c.pu;
    c.pu = 0.0;
    c.flux = 0.0;
    c.op = 0.0;
    let c = integrate(&r, c, n, dt);
    if c.op > 0.0 { c.flux / c.op } else { f64::NAN }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rel(a: f64, b: f64) -> f64 {
        (a / b - 1.0).abs()
    }

    #[test]
    fn conservative_reference_case() {
        let p = PfhInputs::REFERENCE;
        assert!((t_ce_h(&p) - 51.8).abs() < 0.05, "t_CE 51.8 h");
        let pfh = pfh_2oo3_conservative(&p);
        assert!(
            (pfh / 1.04e-8 - 1.0).abs() < 0.02,
            "PFH 1.04e-8/h ±2 %: {pfh:e}"
        );
    }

    // req: FAB-EXO-305
    #[test]
    fn two_forms_anchor() {
        let p = PfhInputs::REFERENCE;
        // t_CE = 0.01·(4380 + 8) + 0.99·8 = 51.8 h (exotic design §2.4).
        assert!((t_ce_h(&p) - 51.8).abs() < 1e-9, "t_CE {} h", t_ce_h(&p));
        let cons = pfh_2oo3_conservative(&p);
        let ed2 = pfh_2oo3_ed2(&p);
        assert!(
            rel(cons, 1.04e-8) < 0.02,
            "conservative {cons:e}/h vs 1.04e-8 ±2 % (EXOTIC §2)"
        );
        assert!(
            rel(ed2, 2.0e-10) < 0.02,
            "Ed. 2 {ed2:e}/h vs 2.0e-10 ±2 % (EXOTIC §2)"
        );
        // The two editions differ by about 50× on the reference case.
        assert!(
            (45.0..=55.0).contains(&(cons / ed2)),
            "ratio {}",
            cons / ed2
        );
        // Closed-form pieces: CCF terms β_D·λ_DD + β·λ_DU = 1.01e-8 (conservative) and
        // β·λ_DU = 2.0e-10 (Ed. 2) dominate.
        assert!(rel(cons - 1.01e-8, 6.0 * 9.899e-7f64.powi(2) * 51.8) < 1e-3);
        assert!(rel(ed2 - 2.0e-10, 6.0 * 9.899e-7 * 9.8e-9 * 51.8) < 1e-3);
        // Common cause dominates: the conservative form enters the SIL 4 band
        // (below 1e-8/h) only with β_D ≤ about 0.5 % (exotic design §2.4).
        let with_bd = |beta_d| pfh_2oo3_conservative(&PfhInputs { beta_d, ..p });
        assert!(with_bd(0.005) < 1.0e-8 && with_bd(0.002) < 1.0e-8);
        assert!(with_bd(0.01) > 1.0e-8);
        // Both forms grow monotonically with λ_D, β and T1.
        for q in [
            PfhInputs {
                lambda_d_per_h: 2e-6,
                ..p
            },
            PfhInputs { beta: 0.05, ..p },
            PfhInputs {
                t1_h: 17_520.0,
                ..p
            },
        ] {
            assert!(
                pfh_2oo3_conservative(&q) > cons && pfh_2oo3_ed2(&q) > ed2,
                "{q:?}"
            );
        }
    }

    // req: FAB-EXO-117
    #[test]
    fn ctmc_cross_check() {
        let p = PfhInputs::REFERENCE;
        let (c, e) = (pfh_2oo3_ctmc(&p), pfh_2oo3_ctmc_ed2(&p));
        assert!(
            rel(c, pfh_2oo3_conservative(&p)) < 0.20,
            "CTMC {c:e} vs conservative {:e} (±20 %, FAB-EXO-117)",
            pfh_2oo3_conservative(&p)
        );
        assert!(
            rel(e, pfh_2oo3_ed2(&p)) < 0.20,
            "CTMC {e:e} vs Ed. 2 {:e} (±20 %, FAB-EXO-117)",
            pfh_2oo3_ed2(&p)
        );
        // Sweep inside the simplified equations' domain (λ_DU·T1 ≪ 1).
        let mut n = 0;
        for lambda_d_per_h in [1e-7, 1e-6, 5e-6] {
            for dc in [0.6, 0.9, 0.99] {
                for beta in [0.0, 0.02, 0.1] {
                    for t1_h in [4380.0, 8760.0, 17_520.0] {
                        for (mttr_h, mrt_h) in [(8.0, 8.0), (24.0, 72.0)] {
                            let q = PfhInputs {
                                lambda_d_per_h,
                                dc,
                                beta,
                                beta_d: beta / 2.0,
                                t1_h,
                                mttr_h,
                                mrt_h,
                            };
                            let (cc, ce) = (pfh_2oo3_ctmc(&q), pfh_2oo3_ctmc_ed2(&q));
                            let (fc, fe) = (pfh_2oo3_conservative(&q), pfh_2oo3_ed2(&q));
                            assert!(
                                rel(cc, fc) < 0.20,
                                "{q:?}: CTMC {cc:e} vs conservative {fc:e}"
                            );
                            assert!(rel(ce, fe) < 0.20, "{q:?}: CTMC {ce:e} vs Ed. 2 {fe:e}");
                            n += 1;
                        }
                    }
                }
            }
        }
        assert_eq!(n, 162);
        // Outside the domain the CTMC declines rather than guessing.
        for q in [
            PfhInputs {
                lambda_d_per_h: 0.0,
                ..p
            },
            PfhInputs { dc: 1.5, ..p },
            PfhInputs { mttr_h: 0.0, ..p },
            PfhInputs {
                t1_h: f64::INFINITY,
                ..p
            },
            PfhInputs { mrt_h: 1e-6, ..p },
        ] {
            assert!(
                pfh_2oo3_ctmc(&q).is_nan() && pfh_2oo3_ctmc_ed2(&q).is_nan(),
                "{q:?}"
            );
        }
    }
}
