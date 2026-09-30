//! Hold-up supply check: enough supercap energy for a Safe Landing (SAF-009). Owner: A5.
//!
//! [`SupplyCheck`] on integer codes, at the supervisor rate (`Monitor::step_slow`):
//! * mains lost → MainsLost (SS1 Safe Landing on the supercap, then STO; SAF-009);
//! * hold-up voltage code below `v_hold_min_code` → HoldupLow (Degrade: refuse lift).
//!
//! The usable hold-up energy above the converter cut-out V_min is
//! E = η·½·C·(V² − V_min²) (PHYSICS §14), monotone in V, so the SAF-009 requirement
//! E ≥ 3·E_need is exactly a voltage floor V_req = √(V_min² + 2·3·E_need/(η·C)): the
//! monitor enforces it as one integer comparison on the raw code, with the threshold
//! code rounded UP in `Monitor::new` (never below V_req). `SafetyLimits::v_hold_min_v`
//! is that floor ("minimum hold-up voltage for a Safe Landing").
#![deny(clippy::float_arithmetic)]

use fabrail_types::{FaultCode, FaultSet, PowerRaw};

/// Hold-up supply supervision.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SupplyCheck {
    /// Minimum hold-up voltage code for a Safe Landing.
    pub v_hold_min_code: u16,
}

impl SupplyCheck {
    /// One supervisor release.
    pub fn step(&self, p: &PowerRaw) -> FaultSet {
        let mut f = FaultSet::EMPTY;
        if !p.mains_ok {
            f.set(FaultCode::MainsLost);
        }
        if p.v_hold_code < self.v_hold_min_code {
            f.set(FaultCode::HoldupLow);
        }
        f
    }
}

#[cfg(test)]
#[allow(clippy::float_arithmetic)]
mod tests {
    use super::*;
    use crate::monitor::Monitor;
    use fabrail_params::{derive, presets};
    use fabrail_types::{LevState, SafetyDemand, SafetyInRaw, Tick, fmath};

    // req: FAB-SAF-009
    #[test]
    fn holdup_energy() {
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let pw = p.design.power;
            let h = derive::holdup_energy(&p);
            assert!(
                h.avail_j >= 3.0 * h.need_j,
                "(A) preset hold-up ≥ 3× landing need, SAF-009: avail {} J need {} J",
                h.avail_j,
                h.need_j
            );
            // The SAF-009 voltage floor: η·½·C·(V_req² − V_min²) = 3·E_need.
            let v_req = fmath::sqrt(
                pw.supercap_v_min_v * pw.supercap_v_min_v
                    + 2.0 * 3.0 * h.need_j / (pw.converter_eff * pw.supercap_c_f),
            );
            assert!(
                v_req < pw.supercap_v_max_v,
                "a charged supercap clears the floor"
            );
            let mut limits = p.safety;
            limits.v_hold_min_v = v_req;
            let m = Monitor::new(&limits, &p.design.sensors);
            let per = p.design.sensors.v_hold_v_per_code;
            let code = m.supply.v_hold_min_code;
            // The integer threshold never admits less than 3× the need: the energy at the
            // threshold code is ≥ 3·E_need, and one code below it is < 3·E_need.
            let e_at = |c: u16| {
                let v = c as f64 * per;
                pw.converter_eff
                    * 0.5
                    * pw.supercap_c_f
                    * (v * v - pw.supercap_v_min_v * pw.supercap_v_min_v)
            };
            assert!(
                e_at(code) >= 3.0 * h.need_j,
                "(E) threshold rounds up, SAF-009"
            );
            assert!(
                e_at(code - 1) < 3.0 * h.need_j,
                "(E) one code below is short"
            );
            let ok = PowerRaw {
                v_hold_code: code,
                mains_ok: true,
                ..PowerRaw::default()
            };
            assert!(m.supply.step(&ok).is_empty(), "at the floor: lift allowed");
            let low = PowerRaw {
                v_hold_code: code - 1,
                ..ok
            };
            assert!(
                m.supply.step(&low).contains(FaultCode::HoldupLow),
                "(T) refuse lift"
            );
            assert_eq!(
                FaultCode::HoldupLow.demand(),
                SafetyDemand::None,
                "HoldupLow degrades (refuse lift), no stop demand"
            );
            // Mains loss demands SS1 Safe Landing through the slow verdict.
            let mut m = m;
            let lost = PowerRaw {
                mains_ok: false,
                ..ok
            };
            let v = m.step_slow(
                Tick(20),
                &lost,
                &SafetyInRaw::default(),
                LevState::LevitatedIdle,
            );
            assert!(v.faults.contains(FaultCode::MainsLost));
            assert_eq!(
                v.demand,
                SafetyDemand::SafeLand,
                "(T) SS1 Safe Landing, SAF-009"
            );
        }
    }
}
