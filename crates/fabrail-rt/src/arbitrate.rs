//! Safety arbitration: LSM force <= min(core limit, verdict cap), STO/ASC on demand, demand latch (SAF-016). Owner: A5.
//!
//! Rules (core design §B.5 "Arbitration", in order; PHYSICS §18):
//! 1. LSM force ≤ min(core limit, `verdict.force_cap_n`): |i_q| ≤ min(F_core, cap)/K_f,
//!    with K_f the monitor's diverse copy (`safety.k_f_copy_n_per_a`, sealed equal to
//!    the design value) because the cap is the monitor's;
//! 2. a `StoLsm*` demand sets `Sto`/`Asc` on the same job (currents zeroed);
//! 3. a demand ≥ SafeStop forces FAULT_REACTION at the next supervisor release (the
//!    latched demand travels to the supervisor in `FastOut.verdict`; the latch window of
//!    two supervisor periods ensures a supervisor release reads it);
//! 4. a demand level can only rise within a latch window; the core can never lower it
//!    (the core's outputs never enter the demand; SAF-016).
//!
//! STO/ASC act on PROPULSION only; the coil command passes through unchanged
//! (levitation's safe state is SS1 Safe Landing, driven by the supervisor), except that
//! a non-finite coil value never reaches the HAL (0, the zero-power bias; the monitor
//! reports NonFinite, SAF-017). The arbiter never turns a core STO/ASC into RUN.

use fabrail_core::msg::FastOutputs;
use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{CoilCmd, LsmCmd, LsmMode, SafetyDemand, SafetyVerdict, Tick, fmath};

/// Demand latch: the level can only rise within `window_ticks` of the last rise.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DemandLatch {
    /// Latched level.
    pub level: SafetyDemand,
    /// Tick until which the level cannot fall.
    pub until: Tick,
    /// Latch window, base ticks.
    pub window_ticks: u32,
}

impl DemandLatch {
    /// A latch with the given window (base ticks).
    pub const fn new(window_ticks: u32) -> DemandLatch {
        DemandLatch {
            level: SafetyDemand::None,
            until: Tick(0),
            window_ticks,
        }
    }

    /// Applies demand `d` at tick `t`; returns the effective (latched) demand.
    pub fn update(&mut self, t: Tick, d: SafetyDemand) -> SafetyDemand {
        if d > self.level || (d == self.level && d != SafetyDemand::None) {
            if d > self.level {
                self.level = d;
            }
            self.until = t.plus(self.window_ticks as u64);
        } else if t >= self.until {
            self.level = d;
        }
        self.level
    }
}

/// The arbiter between core commands and the monitor verdict.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Arbiter {
    /// Demand latch.
    pub latch: DemandLatch,
    /// Thrust constant K_f (the monitor's diverse copy), N/A.
    pub k_f_n_per_a: f32,
    /// Core force limit (LSM peak thrust), N.
    pub core_force_max_n: f32,
}

impl Arbiter {
    /// Arbiter from the LSM design; the latch window is two supervisor periods, so a
    /// raised demand is always seen by the next supervisor release (rule 3).
    pub fn new(p: &SystemParams) -> Result<Arbiter, ParamError> {
        let window = (p.control.rates.sup_decim as u32).saturating_mul(2);
        Ok(Arbiter {
            latch: DemandLatch::new(window),
            k_f_n_per_a: p.safety.k_f_copy_n_per_a as f32,
            core_force_max_n: p.design.lsm.f_peak_n as f32,
        })
    }

    /// Arbitrates one fast release: returns the commands to apply and the latched
    /// demand.
    pub fn step(
        &mut self,
        t: Tick,
        core: &FastOutputs,
        v: &SafetyVerdict,
    ) -> (FastOutputs, SafetyDemand) {
        let demand = self.latch.update(t, v.demand);
        let mut out = *core;
        if let CoilCmd::CurrentRef(r) | CoilCmd::Voltage(r) = &mut out.coil {
            for x in r.iter_mut() {
                if !x.is_finite() {
                    *x = 0.0;
                }
            }
        }
        match demand {
            SafetyDemand::StoLsm => out.lsm = sto(LsmMode::Sto),
            SafetyDemand::StoLsmAsc => out.lsm = sto(LsmMode::Asc),
            _ => {
                if out.lsm.mode == LsmMode::Run {
                    // A non-finite verdict cap is the most restrictive (0 N); libm fminf
                    // would otherwise return the core limit for a NaN.
                    let vcap = if v.force_cap_n.is_finite() {
                        v.force_cap_n
                    } else {
                        0.0
                    };
                    let cap = fmath::fminf(self.core_force_max_n, vcap);
                    out.lsm.iq_ref_a = limit_iq(out.lsm.iq_ref_a, cap, self.k_f_n_per_a);
                    if !out.lsm.id_ref_a.is_finite() {
                        out.lsm.id_ref_a = 0.0;
                    }
                } else {
                    // The core's own STO/ASC passes through with zero currents.
                    out.lsm = sto(out.lsm.mode);
                }
            }
        }
        (out, demand)
    }
}

/// An LSM command in a safe mode with zero currents.
#[inline]
const fn sto(mode: LsmMode) -> LsmCmd {
    LsmCmd {
        iq_ref_a: 0.0,
        id_ref_a: 0.0,
        mode,
    }
}

/// Limits |i_q| to cap/K_f (non-finite or non-positive inputs give 0 A).
#[inline]
pub fn limit_iq(iq_a: f32, cap_n: f32, k_f_n_per_a: f32) -> f32 {
    if !(iq_a.is_finite() && cap_n.is_finite() && k_f_n_per_a.is_finite())
        || cap_n <= 0.0
        || k_f_n_per_a <= 0.0
    {
        return 0.0;
    }
    let i_max = cap_n / k_f_n_per_a;
    fmath::clampf(iq_a, -i_max, i_max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_types::MAX_MAGNETS;
    use proptest::prelude::*;

    fn arb() -> Arbiter {
        Arbiter {
            latch: DemandLatch::new(40),
            k_f_n_per_a: 10.0,
            core_force_max_n: 150.0,
        }
    }

    #[test]
    fn force_capped_by_min_of_core_and_verdict() {
        let mut a = arb();
        let core = FastOutputs {
            lsm: LsmCmd {
                iq_ref_a: 50.0,
                id_ref_a: 0.0,
                mode: LsmMode::Run,
            },
            ..FastOutputs::default()
        };
        let v = SafetyVerdict {
            force_cap_n: 67.0,
            ..SafetyVerdict::default()
        };
        let (o, _) = a.step(Tick(0), &core, &v);
        assert_eq!(o.lsm.iq_ref_a, 6.7);
        assert_eq!(o.lsm.mode, LsmMode::Run);
        let v0 = SafetyVerdict::default();
        let (o, _) = a.step(Tick(1), &core, &v0);
        assert_eq!(
            o.lsm.iq_ref_a, 0.0,
            "default verdict (0 N cap) allows no force"
        );
        let big = SafetyVerdict {
            force_cap_n: 1.0e6,
            ..SafetyVerdict::default()
        };
        let (o, _) = a.step(Tick(2), &core, &big);
        assert_eq!(o.lsm.iq_ref_a, 15.0, "core limit 150 N binds");
    }

    #[test]
    fn demand_only_rises_within_window() {
        let mut a = arb();
        let core = FastOutputs::default();
        let sto = SafetyVerdict {
            demand: SafetyDemand::StoLsmAsc,
            force_cap_n: 100.0,
            ..SafetyVerdict::default()
        };
        let (o, d) = a.step(Tick(100), &core, &sto);
        assert_eq!(o.lsm.mode, LsmMode::Asc);
        assert_eq!(d, SafetyDemand::StoLsmAsc);
        let calm = SafetyVerdict {
            force_cap_n: 100.0,
            ..SafetyVerdict::default()
        };
        let (o, d) = a.step(Tick(120), &core, &calm);
        assert_eq!(d, SafetyDemand::StoLsmAsc, "cannot fall inside the window");
        assert_eq!(o.lsm.mode, LsmMode::Asc);
        let (_, d) = a.step(Tick(140), &core, &calm);
        assert_eq!(d, SafetyDemand::None, "falls after the window");
    }

    #[test]
    fn nonfinite_is_zeroed() {
        assert_eq!(limit_iq(f32::NAN, 100.0, 10.0), 0.0);
        assert_eq!(limit_iq(3.0, 100.0, 0.0), 0.0);
        assert_eq!(limit_iq(-30.0, 100.0, 10.0), -10.0);
        let mut a = arb();
        let core = FastOutputs {
            coil: CoilCmd::Voltage([f32::NAN; MAX_MAGNETS]),
            lsm: LsmCmd {
                iq_ref_a: f32::INFINITY,
                id_ref_a: f32::NAN,
                mode: LsmMode::Run,
            },
        };
        let v = SafetyVerdict {
            force_cap_n: 100.0,
            ..SafetyVerdict::default()
        };
        let (o, _) = a.step(Tick(0), &core, &v);
        assert_eq!(
            o.coil,
            CoilCmd::Voltage([0.0; MAX_MAGNETS]),
            "(T) no NaN to the coils"
        );
        assert_eq!(
            (o.lsm.iq_ref_a, o.lsm.id_ref_a),
            (0.0, 0.0),
            "(T) no NaN to the LSM"
        );
    }

    fn any_f32() -> impl Strategy<Value = f32> {
        prop_oneof![
            -1.0e4f32..1.0e4,
            Just(f32::NAN),
            Just(f32::INFINITY),
            Just(f32::NEG_INFINITY),
            any::<u32>().prop_map(f32::from_bits),
        ]
    }

    fn release() -> impl Strategy<Value = (u64, FastOutputs, SafetyVerdict)> {
        (
            1u64..60,
            any_f32(),
            any_f32(),
            0u8..3,
            proptest::array::uniform8(any_f32()),
            0u8..6,
            prop_oneof![0.0f32..500.0, Just(f32::NAN), Just(f32::INFINITY)],
        )
            .prop_map(|(dt, iq, id, mode, coil, d, cap)| {
                let core = FastOutputs {
                    coil: CoilCmd::CurrentRef(coil),
                    lsm: LsmCmd {
                        iq_ref_a: iq,
                        id_ref_a: id,
                        mode: LsmMode::from_u8(mode).unwrap_or(LsmMode::Sto),
                    },
                };
                let v = SafetyVerdict {
                    demand: SafetyDemand::from_u8(d).unwrap_or(SafetyDemand::None),
                    force_cap_n: cap,
                    ..SafetyVerdict::default()
                };
                (dt, core, v)
            })
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            cases: 512,
            failure_persistence: None,
            ..ProptestConfig::default()
        })]

        // req: FAB-SAF-016
        #[test]
        fn arbitration_monotone(seq in proptest::collection::vec(release(), 1..80)) {
            let mut a = Arbiter {
                latch: DemandLatch::new(40),
                k_f_n_per_a: 30.0,
                core_force_max_n: 150.0,
            };
            let mut t = 0u64;
            let mut prev = SafetyDemand::None;
            let mut raised_at = 0u64;
            for (dt, core, v) in &seq {
                t += dt;
                let (o, d) = a.step(Tick(t), core, v);
                // (T) never below the monitor's demand of this release, SAF-016.
                prop_assert!(d >= v.demand, "{d:?} < {:?}", v.demand);
                // (T) never falls inside the latch window after a rise or a repeat.
                if d < prev {
                    prop_assert!(t >= raised_at + 40, "fell at {t} inside the window from {raised_at}");
                }
                if d > prev || (d == prev && d != SafetyDemand::None && v.demand == d) {
                    raised_at = t;
                }
                prev = d;
                // (T) STO / ASC on the same job, currents zero, whatever the core says.
                match d {
                    SafetyDemand::StoLsm => prop_assert_eq!(o.lsm, sto(LsmMode::Sto)),
                    SafetyDemand::StoLsmAsc => prop_assert_eq!(o.lsm, sto(LsmMode::Asc)),
                    _ => {
                        // The arbiter never turns a core STO/ASC into RUN.
                        prop_assert_eq!(o.lsm.mode, core.lsm.mode);
                        // (T) |F| = K_f·|i_q| ≤ min(core limit, cap), finite.
                        prop_assert!(o.lsm.iq_ref_a.is_finite() && o.lsm.id_ref_a.is_finite());
                        let cap = if v.force_cap_n.is_finite() { v.force_cap_n.min(150.0) } else { 0.0 };
                        prop_assert!(
                            o.lsm.iq_ref_a.abs() * 30.0 <= cap * (1.0 + 1.0e-6) + 1.0e-6,
                            "|iq| {} exceeds cap {}", o.lsm.iq_ref_a, cap
                        );
                        // Same-sign, never amplified.
                        if core.lsm.iq_ref_a.is_finite() && o.lsm.iq_ref_a != 0.0 {
                            prop_assert!(o.lsm.iq_ref_a.signum() == core.lsm.iq_ref_a.signum());
                            prop_assert!(o.lsm.iq_ref_a.abs() <= core.lsm.iq_ref_a.abs());
                        }
                    }
                }
                // Coils: passed through except non-finite values (0).
                let (CoilCmd::CurrentRef(ci), CoilCmd::CurrentRef(co)) = (core.coil, o.coil) else {
                    return Err(TestCaseError::fail("coil command kind changed"));
                };
                for (i, o) in ci.iter().zip(co.iter()) {
                    if i.is_finite() {
                        prop_assert_eq!(i.to_bits(), o.to_bits());
                    } else {
                        prop_assert_eq!(*o, 0.0);
                    }
                }
            }
        }
    }
}
