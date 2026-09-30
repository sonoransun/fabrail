//! Bus with power limit and brake chopper, supercap hold-up (ESR, efficiency), regeneration (PHYSICS §14). Owner: A6b.
//!
//! Supercap 10 F at 12 V (ESR 10 mΩ, η 0.9) by default (core design §C.1). On mains
//! loss the hold-up converter feeds the bus for the SS1 Safe Landing.
//!
//! Quasi-static bus model (PHYSICS §14; physics perspective §1.13). `p_load_w` is the
//! whole bus load: coil and LSM electrical input plus the standby electronics (the
//! caller adds `p_electronics_w`); negative means net regeneration.
//!
//! * Mains present: the supply regulates the bus at V_bus up to P_max; beyond it the bus
//!   sags as a constant-resistance load would, V = V_bus·√(P_max/P). The supply cannot
//!   sink power, so net regeneration first recharges the supercap (up to the charger
//!   rating) and the rest is burned in the brake chopper, which clamps the bus at
//!   V_chopper (PRP-010: the chopper sits above the ≤ 16 % regen rise). While the
//!   supercap is below its charged voltage the charger draws up to
//!   [`CHARGER_FRAC`]·P_max from the headroom.
//! * Mains lost: the hold-up converter (efficiency η) regulates the bus at V_bus from
//!   the supercap while V_hold > V_min. The cell current solves P/η = I·(V − I·ESR),
//!   I = (V − √(V² − 4·ESR·P/η))/(2·ESR); above the maximum-power point V²/(4·ESR) the
//!   bus sags. Regeneration charges the cell through the converter (η) until V_max,
//!   then the chopper burns it. At V_hold ≤ V_min the converter drops out: the bus
//!   collapses to 0 V and the drives are unpowered.
//!
//! Hold-up accounting: [`usable_energy_j`] is ½·C·(V² − V_min²), the store; the bus can
//! draw η times that less the ESR loss (the SAF-009 margin uses
//! `derive::holdup_energy`, which applies η).

use fabrail_params::PowerDesign;
use fabrail_types::fmath;

/// Supercap charger rating as a fraction of the bus power limit (design choice: a
/// recharge never takes more than a tenth of the drive budget, so it cannot starve the
/// drives; PHYSICS §14), dimensionless.
pub const CHARGER_FRAC: f64 = 0.1;

/// Bus operating point and hold-up flow for one step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PowerFlow {
    /// Bus voltage, V.
    pub v_bus_v: f64,
    /// Bus current, A (negative = regeneration).
    pub i_bus_a: f64,
    /// Power delivered to the loads, W.
    pub p_bus_w: f64,
    /// Supercap voltage rate, V/s.
    pub dv_hold_dt_v_per_s: f64,
    /// Power burned in the brake chopper, W.
    pub p_chopper_w: f64,
    /// The hold-up converter is feeding the bus.
    pub from_hold: bool,
}

/// Energy stored in a capacitor, ½·C·V², J.
pub fn supercap_energy_j(c_f: f64, v_v: f64) -> f64 {
    0.5 * c_f * v_v * v_v
}

/// Hold-up energy remaining above the converter drop-out, ½·C·(V² − V_min²) (≥ 0), J
/// (the `PowerTruth::e_hold_j` definition; PHYSICS §14).
pub fn usable_energy_j(d: &PowerDesign, v_hold_v: f64) -> f64 {
    (supercap_energy_j(d.supercap_c_f, v_hold_v)
        - supercap_energy_j(d.supercap_c_f, d.supercap_v_min_v))
    .max(0.0)
}

/// Bus current for power `p_w` at voltage `v_v`, A (0 on a dead or unset bus).
fn bus_current(p_w: f64, v_v: f64) -> f64 {
    if v_v > 0.0 { p_w / v_v } else { 0.0 }
}

/// Converter efficiency clamped to (0, 1] (1 when unset).
fn eta(d: &PowerDesign) -> f64 {
    if d.converter_eff.is_finite() && d.converter_eff > 0.0 {
        d.converter_eff.min(1.0)
    } else {
        1.0
    }
}

/// dV/dt of the cell for an internal (EMF-side) power `p_cell_w` (> 0 discharging),
/// V/s: −P/(C·V), with V floored at 1 % of V_max to keep the rate finite at an empty
/// cell (0 for an unset cell).
fn cell_rate(d: &PowerDesign, v_hold_v: f64, p_cell_w: f64) -> f64 {
    if d.supercap_c_f <= 0.0 {
        return 0.0;
    }
    let v = v_hold_v.max(0.01 * d.supercap_v_max_v);
    if v.is_nan() || v <= 0.0 {
        return 0.0;
    }
    -p_cell_w / (d.supercap_c_f * v)
}

/// Bus and hold-up flow for load power `p_load_w` (negative = regeneration) with the
/// supercap at `v_hold_v` (PHYSICS §14). `p_load_w` is the whole bus load, standby
/// electronics included.
pub fn power_flow(d: &PowerDesign, v_hold_v: f64, p_load_w: f64, mains_ok: bool) -> PowerFlow {
    let p = if p_load_w.is_finite() { p_load_w } else { 0.0 };
    let v_hold = if v_hold_v.is_finite() { v_hold_v } else { 0.0 };
    let eta = eta(d);
    let full = v_hold >= d.supercap_v_max_v;
    let charger_max = CHARGER_FRAC * d.p_bus_max_w.max(0.0);
    if mains_ok {
        if p >= 0.0 {
            let p_max = d.p_bus_max_w;
            let (p_bus, v_bus) = if p_max > 0.0 && p > p_max {
                (p_max, d.v_bus_v * fmath::sqrt(p_max / p))
            } else {
                (p, d.v_bus_v)
            };
            let headroom = if p_max > 0.0 {
                (p_max - p_bus).max(0.0)
            } else {
                charger_max
            };
            let p_charge = if full { 0.0 } else { charger_max.min(headroom) };
            PowerFlow {
                v_bus_v: v_bus,
                i_bus_a: bus_current(p_bus + p_charge, v_bus),
                p_bus_w: p_bus,
                dv_hold_dt_v_per_s: cell_rate(d, v_hold, -eta * p_charge),
                p_chopper_w: 0.0,
                from_hold: false,
            }
        } else {
            // Net regeneration: recharge first, burn the rest at the chopper threshold.
            let regen = -p;
            let p_charge = if full { 0.0 } else { charger_max.min(regen) };
            let p_chop = regen - p_charge;
            let v_bus = if p_chop > 0.0 {
                d.v_chopper_v
            } else {
                d.v_bus_v
            };
            PowerFlow {
                v_bus_v: v_bus,
                i_bus_a: bus_current(p, v_bus),
                p_bus_w: p,
                dv_hold_dt_v_per_s: cell_rate(d, v_hold, -eta * p_charge),
                p_chopper_w: p_chop,
                from_hold: false,
            }
        }
    } else if v_hold <= d.supercap_v_min_v || d.supercap_c_f <= 0.0 {
        // Converter dropped out: the bus collapses and nothing flows.
        PowerFlow::default()
    } else if p >= 0.0 {
        let esr = d.supercap_esr_ohm.max(0.0);
        let p_out = p / eta; // converter input power drawn from the cell terminals
        let (i, delivered) = if esr > 0.0 {
            let disc = v_hold * v_hold - 4.0 * esr * p_out;
            if disc >= 0.0 {
                ((v_hold - fmath::sqrt(disc)) / (2.0 * esr), p)
            } else {
                // Beyond the maximum-power point: I = V/(2·ESR) delivers η·V²/(4·ESR).
                let i = v_hold / (2.0 * esr);
                (i, eta * v_hold * v_hold / (4.0 * esr))
            }
        } else {
            (p_out / v_hold, p)
        };
        let v_bus = if delivered < p && p > 0.0 {
            d.v_bus_v * fmath::sqrt(delivered / p)
        } else {
            d.v_bus_v
        };
        PowerFlow {
            v_bus_v: v_bus,
            i_bus_a: bus_current(delivered, v_bus),
            p_bus_w: delivered,
            // The EMF side supplies V·I (terminal power plus the I²·ESR loss).
            dv_hold_dt_v_per_s: cell_rate(d, v_hold, v_hold * i),
            p_chopper_w: 0.0,
            from_hold: true,
        }
    } else {
        // Regeneration during hold-up: charge through the converter, chopper when full.
        let regen = -p;
        let (p_charge, p_chop) = if full { (0.0, regen) } else { (regen, 0.0) };
        let v_bus = if p_chop > 0.0 {
            d.v_chopper_v
        } else {
            d.v_bus_v
        };
        PowerFlow {
            v_bus_v: v_bus,
            i_bus_a: bus_current(p, v_bus),
            p_bus_w: p,
            dv_hold_dt_v_per_s: cell_rate(d, v_hold, -eta * p_charge),
            p_chopper_w: p_chop,
            from_hold: true,
        }
    }
}

/// Time the hold-up can carry a constant bus load `p_load_w` from `v_hold_v` down to
/// the converter drop-out, ignoring the ESR loss: η·½·C·(V² − V_min²)/P, s (PHYSICS §14;
/// SAF-009 "DRAWER open hover ≈ 5 min at 1.5 W").
pub fn holdup_time_s(d: &PowerDesign, v_hold_v: f64, p_load_w: f64) -> f64 {
    if p_load_w.is_nan() || p_load_w <= 0.0 {
        return f64::INFINITY;
    }
    eta(d) * usable_energy_j(d, v_hold_v) / p_load_w
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integrator::rk4_step;
    use fabrail_params::presets;

    fn drawer() -> PowerDesign {
        presets::drawer().design.power
    }

    #[test]
    fn supercap_energy_closed_form() {
        // 10 F at 12 V: ½·10·144 = 720 J; usable above 6 V: 540 J.
        assert_eq!(supercap_energy_j(10.0, 12.0), 720.0);
        assert_eq!(usable_energy_j(&drawer(), 12.0), 540.0);
        assert_eq!(usable_energy_j(&drawer(), 5.0), 0.0);
    }

    #[test]
    fn mains_bus_limit_and_chopper() {
        // 24 V, P_max 120 W (fixed here: the fixture predates the W1.5 preset bus
        // sizing), chopper 27.6 V.
        let d = PowerDesign {
            p_bus_max_w: 120.0,
            ..drawer()
        };
        let f = power_flow(&d, 12.0, 48.0, true);
        assert_eq!((f.v_bus_v, f.p_bus_w, f.from_hold), (24.0, 48.0, false));
        assert!((f.i_bus_a - 2.0).abs() < 1e-12);
        assert_eq!(f.dv_hold_dt_v_per_s, 0.0, "full cell: no charge");
        // Over the limit the bus sags as V_bus·√(P_max/P).
        let f = power_flow(&d, 12.0, 480.0, true);
        assert_eq!(f.p_bus_w, 120.0);
        assert!((f.v_bus_v - 12.0).abs() < 1e-12);
        // Net regeneration with a full cell: the chopper burns it at 27.6 V (PRP-010).
        let f = power_flow(&d, 12.0, -30.0, true);
        assert_eq!((f.v_bus_v, f.p_chopper_w), (27.6, 30.0));
        assert!(f.i_bus_a < 0.0);
        // A partly drained cell recharges from the headroom at ≤ 0.1·P_max.
        let f = power_flow(&d, 8.0, 10.0, true);
        assert!(
            (f.i_bus_a - 22.0 / 24.0).abs() < 1e-12,
            "10 W load + 12 W charger"
        );
        assert!((f.dv_hold_dt_v_per_s - 0.9 * 12.0 / (10.0 * 8.0)).abs() < 1e-12);
        let f = power_flow(&d, 8.0, -5.0, true);
        assert_eq!(f.p_chopper_w, 0.0, "regen goes into the cell first");
        assert!(f.dv_hold_dt_v_per_s > 0.0);
    }

    #[test]
    fn holdup_discharge_energy_accounting() {
        // DRAWER open hover on the supercap at 1.5 W (1.0 W drive + 0.5 W electronics):
        // integrating the flow from 12 V to drop-out lasts η·540 J/1.5 W = 324 s within
        // 0.1 % (ESR loss ≈ 1e-5 relative), matching SAF-009's "≈ 5 min, ≈ 490 J usable".
        let d = drawer();
        let p_load = 1.0 + d.p_electronics_w;
        let want = holdup_time_s(&d, 12.0, p_load);
        assert!((want - 324.0).abs() < 1e-9, "closed form {want} s");
        let h = 0.01;
        let mut v = 12.0;
        let mut t = 0.0;
        let mut delivered = 0.0;
        while power_flow(&d, v, p_load, false).from_hold && t < 400.0 {
            delivered += power_flow(&d, v, p_load, false).p_bus_w * h;
            v = rk4_step(&v, t, h, |_, _, vv| {
                power_flow(&d, *vv, p_load, false).dv_hold_dt_v_per_s
            });
            t += h;
        }
        assert!(
            (t - want).abs() < 1e-3 * want,
            "hold-up {t} s vs {want} s (0.1 %)"
        );
        assert!(
            (delivered - 0.9 * 540.0).abs() < 2e-3 * 486.0,
            "delivered {delivered} J"
        );
        // Dropped out: bus collapses.
        let f = power_flow(&d, 5.9, p_load, false);
        assert_eq!(f, PowerFlow::default());
    }

    #[test]
    fn esr_limits_the_holdup_power() {
        // Max terminal power V²/(4·ESR) at 6.5 V and 10 mΩ = 1056 W; η·that reaches the bus.
        let d = drawer();
        let f = power_flow(&d, 6.5, 2000.0, false);
        assert!(f.from_hold);
        assert!((f.p_bus_w - 0.9 * 6.5 * 6.5 / 0.04).abs() < 1e-9);
        assert!(
            f.v_bus_v < d.v_bus_v,
            "bus sags beyond the maximum-power point"
        );
        // Below it the delivered power equals the demand and the cell supplies P/η + I²R.
        let f = power_flow(&d, 12.0, 100.0, false);
        assert_eq!(f.p_bus_w, 100.0);
        let i = (12.0 - (144.0 - 4.0 * 0.01 * 100.0 / 0.9f64).sqrt()) / 0.02;
        assert!((f.dv_hold_dt_v_per_s + 12.0 * i / (10.0 * 12.0)).abs() < 1e-12);
        // Regeneration during hold-up charges the cell, or the chopper when full.
        assert!(power_flow(&d, 10.0, -20.0, false).dv_hold_dt_v_per_s > 0.0);
        let f = power_flow(&d, 12.0, -20.0, false);
        assert_eq!((f.p_chopper_w, f.v_bus_v), (20.0, 27.6));
    }
}
