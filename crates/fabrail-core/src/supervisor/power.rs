//! Supply monitoring, hold-up energy check and SS1 Safe Landing on supply loss (PHYSICS §14, §18; SAF-009). Owner: A4.
//!
//! Frozen API (W0): [`SupplyMonitor::new`], [`SupplyMonitor::step`] →
//! [`SupplyStatus`]. Bus and hold-up voltages from their codes; usable hold-up energy
//! η·½C(V² − V_min²) (supercap 10 F at 12 V, ESR 10 mΩ, η 0.9); `holdup_ok` when it is
//! ≥ `holdup_margin` (3) × the landing need (seat-first for DRAWER; plan §4b); mains
//! loss raises MainsLost (SS1 Safe Landing on the supercap). The open DRAWER hover is
//! limited by the supercap to about 5 min (plan §2b item 4).
//!
//! Faults raised here (the supervisor latches them through `faults::FaultManager`):
//!
//! | Code | Condition | Severity |
//! |---|---|---|
//! | `MainsLost` | `mains_ok` false for [`MAINS_DEBOUNCE`] consecutive releases | SafeLand (SS1) |
//! | `BusUnder` | mains present and V_bus < [`BUS_UNDER_FRAC`]·V_bus,nom | SafeLand |
//! | `BusOver` | V_bus > [`BUS_OVER_FRAC`]·V_chopper (regeneration beyond the chopper) | SafeStop |
//! | `HoldupLow` | E_hold < margin·E_need (refuse lift; a live condition, not latched) | Degrade |
//! | `OverTemp` | coil > `t_coil_max_c` or magnet > `t_mag_max_c` | SafeLand |
//!
//! The landing need E_need is computed once in `new()` (f64, `params::derive`) and cast
//! to f32; per release only the f32 energy of the measured hold-up voltage is computed.

use fabrail_params::{ParamError, RestPolicy, SystemParams, derive};
use fabrail_types::{FaultCode, FaultSet, PowerRaw, fmath};

/// Consecutive supervisor releases without mains before `MainsLost` is raised (2 ms at
/// 1 kHz): rejects a single corrupted sample of the mains-detect line. Design choice
/// (PHYSICS §14); well inside the SS1 budget because the supercap carries the load
/// from the first instant of the loss.
pub const MAINS_DEBOUNCE: u8 = 2;

/// Bus undervoltage threshold as a fraction of the nominal bus (0.8·V_bus). Design
/// choice: below it the coil slew k_i·V/L falls under the PWR-005 requirement margin
/// (PHYSICS §14).
pub const BUS_UNDER_FRAC: f32 = 0.8;

/// Bus overvoltage threshold as a fraction of the brake-chopper threshold (1.05): the
/// chopper holds regeneration at `v_chopper_v` (PRP-010 ≤ 16 % rise); 5 % above it means
/// the chopper failed. Design choice (PHYSICS §14).
pub const BUS_OVER_FRAC: f32 = 1.05;

/// Supply status for one supervisor release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SupplyStatus {
    /// DC bus voltage, V.
    pub v_bus_v: f32,
    /// Hold-up voltage, V.
    pub v_hold_v: f32,
    /// Usable hold-up energy, J.
    pub e_hold_j: f32,
    /// Mains present.
    pub mains_ok: bool,
    /// Hold-up energy ≥ margin × landing need.
    pub holdup_ok: bool,
    /// Faults (BusUnder, BusOver, MainsLost, HoldupLow, OverTemp).
    pub faults: FaultSet,
    /// Energy one Safe Landing needs (seat-first for SEAT classes), J.
    pub e_need_j: f32,
    /// Coil hot-spot temperature, °C.
    pub temp_coil_c: f32,
    /// Magnet temperature, °C.
    pub temp_mag_c: f32,
}

/// Supply monitor.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SupplyMonitor {
    /// Bus voltage scale, V/code.
    pub v_bus_v_per_code: f32,
    /// Hold-up voltage scale, V/code.
    pub v_hold_v_per_code: f32,
    /// Bus undervoltage threshold, V.
    pub v_bus_under_v: f32,
    /// Bus overvoltage threshold, V.
    pub v_bus_over_v: f32,
    /// Supercap capacitance C, F.
    pub c_f: f32,
    /// Supercap minimum usable voltage V_min, V.
    pub v_min_v: f32,
    /// Hold-up converter efficiency η.
    pub eta: f32,
    /// Energy one Safe Landing needs (seat-first for SEAT classes), J.
    pub e_need_j: f32,
    /// Hold-up margin (3; SAF-009).
    pub margin: f32,
    /// Temperature scale, °C/code.
    pub temp_c_per_code: f32,
    /// Temperature offset, °C.
    pub temp_offset_c: f32,
    /// Coil hot-spot limit, °C.
    pub t_coil_max_c: f32,
    /// Magnet limit, °C.
    pub t_mag_max_c: f32,
    /// Consecutive releases without mains.
    pub mains_lost_count: u8,
}

/// Usable supercap energy η·½·C·(V² − V_min²), clamped at 0 below V_min (PHYSICS §14;
/// SAF-009: 10 F from 12 V to 6 V at η 0.9 gives 486 J), J.
pub fn usable_energy_j(c_f: f32, v_v: f32, v_min_v: f32, eta: f32) -> f32 {
    if v_v.is_nan() || v_v <= v_min_v || c_f <= 0.0 {
        return 0.0;
    }
    eta * 0.5 * c_f * (v_v * v_v - v_min_v * v_min_v)
}

/// Energy of the DRAWER seat-first move from the standoff (plan §4b "SS1 from standoff
/// is seat-first … included in the 3× hold-up check"): kinetic energy at the unattended
/// close speed ½·m_rated·v_u², drag over the standoff ((b_dist + c_rail)·v_u + F_hyst)·s,
/// and the electronics over the log-damped terminus t_arrive(v_u) plus the cruise
/// s/v_u (PHYSICS §9, §14, §16.1), J. Zero for classes without a seat.
pub fn seat_first_energy_j(p: &SystemParams) -> f64 {
    let d = &p.design;
    if d.rest_policy != RestPolicy::Seat {
        return 0.0;
    }
    let v_u = derive::unattended_close_speed(
        p.safety.v_close_max_mps,
        p.safety.unattended.f_pinch_n,
        d.carriage.m_rated_kg,
        p.safety.unattended.k_series_n_per_m,
    );
    if v_u <= 0.0 {
        return 0.0;
    }
    let s = p.control.motion.inertial.standoff_m;
    let t = &p.control.motion.terminus;
    let e_k = 0.5 * d.carriage.m_rated_kg * v_u * v_u;
    let e_drag = derive::drag_force(p, v_u) * s;
    let t_move = derive::terminus_t_arrive(v_u, t.j_mps3, t.tau_s, t.d_eps_m) + s / v_u;
    e_k + e_drag + d.power.p_electronics_w * t_move
}

impl SupplyMonitor {
    /// Monitor from `design.sensors`, `design.power` and the supervisor tuning.
    ///
    /// E_need = `derive::holdup_energy(p).need_j` (controlled transfer to the rest gap,
    /// seat-first for SEAT) plus [`seat_first_energy_j`] for SEAT classes.
    pub fn new(p: &SystemParams) -> Result<SupplyMonitor, ParamError> {
        let pw = &p.design.power;
        let sc = &p.design.sensors;
        let th = &p.design.thermal;
        let need = derive::holdup_energy(p).need_j + seat_first_energy_j(p);
        Ok(SupplyMonitor {
            v_bus_v_per_code: sc.v_bus_v_per_code as f32,
            v_hold_v_per_code: sc.v_hold_v_per_code as f32,
            v_bus_under_v: BUS_UNDER_FRAC * pw.v_bus_v as f32,
            v_bus_over_v: BUS_OVER_FRAC * pw.v_chopper_v as f32,
            c_f: pw.supercap_c_f as f32,
            v_min_v: pw.supercap_v_min_v as f32,
            eta: pw.converter_eff as f32,
            e_need_j: need as f32,
            margin: p.control.supervisor.holdup_margin as f32,
            temp_c_per_code: sc.temp_c_per_code as f32,
            temp_offset_c: sc.temp_offset_c as f32,
            t_coil_max_c: th.t_coil_max_c as f32,
            t_mag_max_c: th.t_mag_max_c as f32,
            mains_lost_count: 0,
        })
    }

    /// One supervisor release.
    pub fn step(&mut self, pr: &PowerRaw) -> SupplyStatus {
        let v_bus_v = pr.v_bus_code as f32 * self.v_bus_v_per_code;
        let v_hold_v = pr.v_hold_code as f32 * self.v_hold_v_per_code;
        let e_hold_j = usable_energy_j(self.c_f, v_hold_v, self.v_min_v, self.eta);
        // A non-finite or non-positive need can never be met (refuse lift).
        let holdup_ok = self.e_need_j.is_finite()
            && self.e_need_j > 0.0
            && e_hold_j >= self.margin * self.e_need_j;

        let mut faults = FaultSet::EMPTY;
        if pr.mains_ok {
            self.mains_lost_count = 0;
            if v_bus_v < self.v_bus_under_v {
                faults.set(FaultCode::BusUnder);
            }
        } else {
            self.mains_lost_count = self.mains_lost_count.saturating_add(1);
            if self.mains_lost_count >= MAINS_DEBOUNCE {
                faults.set(FaultCode::MainsLost);
            }
        }
        if v_bus_v > self.v_bus_over_v {
            faults.set(FaultCode::BusOver);
        }
        if !holdup_ok {
            faults.set(FaultCode::HoldupLow);
        }
        let [c_coil, c_mag] = pr.temp_code;
        let temp_coil_c = c_coil as f32 * self.temp_c_per_code + self.temp_offset_c;
        let temp_mag_c = c_mag as f32 * self.temp_c_per_code + self.temp_offset_c;
        if temp_coil_c > self.t_coil_max_c || temp_mag_c > self.t_mag_max_c {
            faults.set(FaultCode::OverTemp);
        }
        SupplyStatus {
            v_bus_v,
            v_hold_v,
            e_hold_j,
            mains_ok: self.mains_lost_count < MAINS_DEBOUNCE,
            holdup_ok,
            faults,
            e_need_j: self.e_need_j,
            temp_coil_c,
            temp_mag_c,
        }
    }

    /// The raw codes that read as `v_bus_v` / `v_hold_v` (tests and SimBoard helpers).
    pub fn codes_for(&self, v_bus_v: f32, v_hold_v: f32) -> (u16, u16) {
        let code = |v: f32, s: f32| {
            if s > 0.0 {
                fmath::roundf(v / s) as u16
            } else {
                0
            }
        };
        (
            code(v_bus_v, self.v_bus_v_per_code),
            code(v_hold_v, self.v_hold_v_per_code),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    fn nominal(m: &SupplyMonitor, p: &SystemParams) -> PowerRaw {
        let (vb, vh) = m.codes_for(
            p.design.power.v_bus_v as f32,
            p.design.power.supercap_v_max_v as f32,
        );
        // 20 °C on both sensors: (20 − offset)/scale codes.
        let t = ((20.0 - p.design.sensors.temp_offset_c) / p.design.sensors.temp_c_per_code) as u16;
        PowerRaw {
            v_bus_code: vb,
            v_hold_code: vh,
            i_bus_code: 0,
            mains_ok: true,
            temp_code: [t, t],
        }
    }

    /// η·½C(V² − V_min²): 10 F, 12 V → 6 V, η 0.9 = 486 J (PHYSICS §14). Tolerance (E)
    /// f32 rounding, 1e-3 J.
    #[test]
    fn usable_energy_closed_form() {
        let e = usable_energy_j(10.0, 12.0, 6.0, 0.9);
        assert!(
            (e - 486.0).abs() < 1e-3,
            "(E) 486 J ±1e-3 (PHYSICS §14), got {e}"
        );
        assert_eq!(usable_energy_j(10.0, 5.0, 6.0, 0.9), 0.0, "below V_min");
        assert_eq!(
            usable_energy_j(10.0, f32::NAN, 6.0, 0.9),
            0.0,
            "NaN reads empty"
        );
    }

    /// Every preset meets the SAF-009 3× hold-up margin at the charged supercap and
    /// raises no supply fault at nominal inputs.
    #[test]
    fn presets_meet_holdup_margin() {
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let built = SupplyMonitor::new(&p);
            assert!(built.is_ok());
            let Ok(mut m) = built else { return };
            let st = m.step(&nominal(&m, &p));
            assert!(
                st.holdup_ok,
                "{:?}: E_hold {} J ≥ 3 × need {} J (SAF-009)",
                p.design.class, st.e_hold_j, st.e_need_j
            );
            assert!(
                st.faults.is_empty(),
                "{:?}: {:?}",
                p.design.class,
                st.faults
            );
            assert!(st.mains_ok);
        }
    }

    /// Seat-first adds a positive x-move energy only for SEAT classes (plan §4b).
    #[test]
    fn seat_first_energy_only_for_seat() {
        assert!(seat_first_energy_j(&presets::drawer()) > 0.0);
        assert_eq!(seat_first_energy_j(&presets::door()), 0.0);
        assert_eq!(seat_first_energy_j(&presets::heavy()), 0.0);
    }

    /// Mains loss is debounced by MAINS_DEBOUNCE releases; bus under/over, a flat
    /// supercap and over-temperature each raise their code.
    #[test]
    fn supply_faults() {
        let p = presets::door();
        let Ok(mut m) = SupplyMonitor::new(&p) else {
            return;
        };
        let good = nominal(&m, &p);
        let lost = PowerRaw {
            mains_ok: false,
            ..good
        };
        let s1 = m.step(&lost);
        assert!(!s1.faults.contains(FaultCode::MainsLost), "debounced");
        let s2 = m.step(&lost);
        assert!(s2.faults.contains(FaultCode::MainsLost));
        assert!(!s2.mains_ok);
        assert!(
            !s2.faults.contains(FaultCode::BusUnder),
            "no BusUnder without mains"
        );
        let s3 = m.step(&good);
        assert!(s3.faults.is_empty() && s3.mains_ok);

        let (under, _) = m.codes_for(0.7 * p.design.power.v_bus_v as f32, 0.0);
        let s = m.step(&PowerRaw {
            v_bus_code: under,
            ..good
        });
        assert!(s.faults.contains(FaultCode::BusUnder));
        let (over, _) = m.codes_for(1.1 * p.design.power.v_chopper_v as f32, 0.0);
        let s = m.step(&PowerRaw {
            v_bus_code: over,
            ..good
        });
        assert!(s.faults.contains(FaultCode::BusOver));
        let s = m.step(&PowerRaw {
            v_hold_code: 0,
            ..good
        });
        assert!(s.faults.contains(FaultCode::HoldupLow) && !s.holdup_ok);
        let hot =
            ((130.0 - p.design.sensors.temp_offset_c) / p.design.sensors.temp_c_per_code) as u16;
        let s = m.step(&PowerRaw {
            temp_code: [hot, good.temp_code[1]],
            ..good
        });
        assert!(
            s.faults.contains(FaultCode::OverTemp),
            "coil 130 °C > 120 °C"
        );
    }
}
