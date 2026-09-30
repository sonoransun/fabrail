//! Integer kinetic energy and momentum at rated mass (u128 / saturating), KE and momentum trips (SAF-012). Owner: A5.
//!
//! Integer units (core design §B.5; plan §2b item 17; PHYSICS §18):
//! * mass in grams (u32; 5 t = 5e6 g),
//! * velocity in µm/s (i64),
//! * twice the kinetic energy in femtojoules: 2·KE\[fJ\] = m_g·v_µm/s², because
//!   (m·1e3)·(v·1e6)² = m·v²·1e15; the 1.69 J cap is 3.38e15. u128 holds 5 t at 10 m/s
//!   (5e6·1e14 = 5e20 > u64::MAX) without overflow (`ke_no_overflow_10mps`),
//! * momentum in g·µm/s (u128; 22.3 kg·m/s = 2.23e10).
//!
//! The checks use RATED mass, never m̂ (the governor uses max(m_rated, 1.05·m̂); F19).
//! They apply to unsensed (low-energy) operation only: with PL d presence sensing the
//! sensed speed caps replace the energy basis (SAF-003), and in ISOLATE the explicit
//! `IsolateLimits` replace them (SAF-020). A limit exceeded for more than the SLS
//! persistence (the same 10 ms as SAF-008) trips EnergyLimit / MomentumLimit, with the
//! drive-evidence exemption of `velocity::SlsTimer` (hand-driven motion does not trip).
#![deny(clippy::float_arithmetic)]

use fabrail_types::{FaultCode, FaultSet, Tick};

use crate::velocity::{DriveEvidence, SlsTimer};

/// 2·KE in femtojoules from mass (g) and velocity (µm/s): m_g·v². Exact below
/// u128::MAX (any physical input), saturating beyond (never panics or wraps).
#[inline]
pub const fn ke2_fj(m_g: u32, v_umps: i64) -> u128 {
    let v = v_umps.unsigned_abs() as u128;
    (m_g as u128).saturating_mul(v.saturating_mul(v))
}

/// Momentum magnitude in g·µm/s from mass (g) and velocity (µm/s). Exact.
#[inline]
pub const fn momentum_g_umps(m_g: u32, v_umps: i64) -> u128 {
    (m_g as u128) * (v_umps.unsigned_abs() as u128)
}

/// Energy and momentum caps at rated mass.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EnergyCheck {
    /// Rated mass, g.
    pub m_g: u32,
    /// 2·E_max, fJ.
    pub ke2_max_fj: u128,
    /// p_max, g·µm/s (0 = not applicable).
    pub p_max_g_umps: u128,
    /// Persistence before tripping, base ticks (10 ms, as SAF-008).
    pub persist_ticks: u32,
    /// Speed increase that counts as speeding up over one interval, µm/s.
    pub accel_tol_umps: u64,
    /// Energy persistence state.
    pub ke_timer: SlsTimer,
    /// Momentum persistence state.
    pub p_timer: SlsTimer,
}

impl EnergyCheck {
    /// Check with rated mass (g), 2·E_max (fJ) and p_max (g·µm/s, 0 = n/a).
    pub const fn new(m_g: u32, ke2_max_fj: u128, p_max_g_umps: u128) -> EnergyCheck {
        EnergyCheck {
            m_g,
            ke2_max_fj,
            p_max_g_umps,
            persist_ticks: 0,
            accel_tol_umps: 0,
            ke_timer: SlsTimer {
                since: None,
                v_start_umps: 0,
                brake_accel_since: None,
            },
            p_timer: SlsTimer {
                since: None,
                v_start_umps: 0,
                brake_accel_since: None,
            },
        }
    }

    /// Instantaneous comparison at `v_umps`: (energy above the cap, momentum above
    /// the cap). Strictly greater: exactly at the cap is allowed.
    #[inline]
    pub fn exceeds(&self, v_umps: i64) -> (bool, bool) {
        let ke = ke2_fj(self.m_g, v_umps) > self.ke2_max_fj;
        let p = self.p_max_g_umps > 0 && momentum_g_umps(self.m_g, v_umps) > self.p_max_g_umps;
        (ke, p)
    }

    /// One release at tick `t` and velocity `v_umps`: EnergyLimit / MomentumLimit.
    pub fn step(&mut self, t: Tick, v_umps: i64, ev: DriveEvidence) -> FaultSet {
        let (ke, p) = self.exceeds(v_umps);
        let v = v_umps.unsigned_abs();
        let mut f = FaultSet::EMPTY;
        if self
            .ke_timer
            .step(t, ke, v, self.persist_ticks, self.accel_tol_umps, ev)
        {
            f.set(FaultCode::EnergyLimit);
        }
        if self
            .p_timer
            .step(t, p, v, self.persist_ticks, self.accel_tol_umps, ev)
        {
            f.set(FaultCode::MomentumLimit);
        }
        f
    }

    /// Clears the persistence state (mode change).
    pub fn reset(&mut self) {
        self.ke_timer.reset();
        self.p_timer.reset();
    }
}

#[cfg(test)]
#[allow(clippy::float_arithmetic)]
mod tests {
    use super::*;
    use fabrail_types::fmath;

    const DRIVE: DriveEvidence = DriveEvidence {
        energised: true,
        cmd_braking: false,
        cmd_driving: true,
    };

    #[test]
    fn unit_contract() {
        // 1.69 J cap: 2·KE = 3.38e15 fJ, e.g. 48 kg at 0.265 347... m/s.
        assert_eq!(
            ke2_fj(1_000, 1_000_000),
            1_000_000_000_000_000,
            "1 kg at 1 m/s = 2·0.5 J"
        );
        assert_eq!(
            momentum_g_umps(100_000, 223_000),
            22_300_000_000,
            "22.3 kg·m/s"
        );
        assert_eq!(
            ke2_fj(u32::MAX, i64::MIN),
            u128::MAX,
            "saturates, never wraps"
        );
    }

    // req: FAB-SAF-012
    #[test]
    fn ke_no_overflow_10mps() {
        // 5 t at 10 m/s: 2·KE = 5e6 g · (1e7 µm/s)² = 5e20 fJ (KE = 250 kJ), beyond
        // u64::MAX (1.8e19) and exact in u128; momentum 5e13 g·µm/s (5e4 kg·m/s).
        let m_g = 5_000_000u32;
        let v = 10_000_000i64;
        let ke2 = ke2_fj(m_g, v);
        assert_eq!(
            ke2, 500_000_000_000_000_000_000,
            "(E) 2·KE of 5 t at 10 m/s, PLAN §2b.17"
        );
        assert!(ke2 > u64::MAX as u128, "(E) the case u64 would overflow");
        assert_eq!(ke2_fj(m_g, -v), ke2, "(E) even in v");
        assert_eq!(
            momentum_g_umps(m_g, v),
            50_000_000_000_000,
            "(E) p = 5e4 kg·m/s"
        );
        // KE in joules: ½·5000·100 = 250 kJ.
        let ke_j = ke2 as f64 / 2.0e15;
        assert_eq!(ke_j, 250_000.0, "(E) 250 kJ, PHYSICS §18");
        // The trip decision at 10 m/s is correct (no wrap to a small value).
        let mut e = EnergyCheck::new(m_g, 3_380_000_000_000_000, 22_300_000_000);
        assert_eq!(
            e.exceeds(v),
            (true, true),
            "(E) 10 m/s at 5 t is far above both caps"
        );
        assert!(e.step(Tick(0), v, DRIVE).is_empty(), "persistence starts");
        let f = e.step(Tick(1), v, DRIVE);
        assert!(
            f.contains(FaultCode::EnergyLimit) && f.contains(FaultCode::MomentumLimit),
            "zero persistence trips on the next check"
        );
        // Saturation at the u32 mass / i64 speed extremes is monotone, never wraps.
        assert!(ke2_fj(u32::MAX, i64::MAX) >= ke2_fj(u32::MAX, i64::MAX / 2));
    }

    // req: FAB-SAF-012
    #[test]
    fn integer_matches_float() {
        // Unit conversion: the integer 2·KE and momentum equal the f64 forms
        // m·v²·1e15 and m·v·1e9 to f64 rounding (≤ 1 fJ below 2^53, else 1 ulp).
        for m_g in [1u32, 8_000, 48_000, 100_000, 500_000, 5_000_000, u32::MAX] {
            for v in [
                0i64, 1, -7, 25_000, 152_000, 183_848, 265_330, 1_000_000, 10_000_000,
            ] {
                let m = m_g as f64 / 1.0e3;
                let vf = v as f64 / 1.0e6;
                let ke2_f = m * vf * vf * 1.0e15;
                let ke2_i = ke2_fj(m_g, v) as f64;
                let tol = fmath::fmax(1.0, 4.0 * f64::EPSILON * ke2_f);
                assert!(
                    fmath::abs(ke2_i - ke2_f) <= tol,
                    "(E) 2·KE m_g={m_g} v={v}: int {ke2_i} vs f64 {ke2_f}; one quantum, PLAN §2b.17"
                );
                let p_f = m * fmath::abs(vf) * 1.0e9;
                let p_i = momentum_g_umps(m_g, v) as f64;
                let tol = fmath::fmax(1.0, 4.0 * f64::EPSILON * p_f);
                assert!(fmath::abs(p_i - p_f) <= tol, "(E) momentum m_g={m_g} v={v}");
            }
        }
        // Decision agreement at the presets' caps: the integer check trips iff
        // ½·m·v² > E_max (or m·v > p_max) in f64, except within one velocity quantum
        // (1 µm/s) of the threshold.
        for (m_kg, e_j, p) in [(48.0, 1.69, 22.3), (100.0, 1.69, 22.3), (500.0, 1.69, 22.3)] {
            let m_g = fmath::round(m_kg * 1.0e3) as u32;
            let ke2_max = fmath::round(2.0 * e_j * 1.0e15) as u128;
            let p_max = fmath::round(p * 1.0e9) as u128;
            let e = EnergyCheck::new(m_g, ke2_max, p_max);
            let v_ke = fmath::sqrt(2.0 * e_j / m_kg) * 1.0e6;
            let v_p = p / m_kg * 1.0e6;
            for v_star in [v_ke, v_p] {
                let base = fmath::round(v_star) as i64;
                for dv in -50..=50i64 {
                    let v = base + dv;
                    let vf = v as f64 / 1.0e6;
                    let float_ke = 0.5 * m_kg * vf * vf > e_j;
                    let float_p = m_kg * vf > p;
                    let (int_ke, int_p) = e.exceeds(v);
                    if fmath::abs(v as f64 - v_ke) > 1.0 {
                        assert_eq!(
                            int_ke, float_ke,
                            "(E) KE decision m={m_kg} v={v} µm/s, one quantum"
                        );
                    }
                    if fmath::abs(v as f64 - v_p) > 1.0 {
                        assert_eq!(int_p, float_p, "(E) momentum decision m={m_kg} v={v} µm/s");
                    }
                }
            }
        }
    }

    // req: FAB-SAF-012
    #[test]
    fn ke_trip() {
        // DOOR at rated mass 100 kg: 1.69 J ⇔ v* = √(3.38/100) = 0.183848 m/s (SAF-002).
        let m_g = 100_000u32;
        let mut e = EnergyCheck::new(m_g, 3_380_000_000_000_000, 22_300_000_000);
        e.persist_ticks = 200;
        e.accel_tol_umps = 200;
        let v_cap = 183_847i64; // floor of v*·1e6 (v* = 183 847.76 µm/s)
        assert_eq!(e.exceeds(v_cap), (false, false), "(E) just below the cap");
        assert_eq!(
            e.exceeds(v_cap + 1),
            (true, false),
            "(E) 1 µm/s above the KE cap"
        );
        // Held above the cap: trips after > 10 ms, not before.
        let v = 190_000i64;
        for k in 0..=100u64 {
            assert!(e.step(Tick(2 * k), v, DRIVE).is_empty(), "within 10 ms");
        }
        let f = e.step(Tick(202), v, DRIVE);
        assert!(
            f.contains(FaultCode::EnergyLimit),
            "(S) KE > 1.69 J for > 10 ms trips"
        );
        assert!(!f.contains(FaultCode::MomentumLimit), "19 kg·m/s < 22.3");
        assert_eq!(
            FaultCode::EnergyLimit.demand(),
            fabrail_types::SafetyDemand::SafeStop,
            "EnergyLimit is a controlled stop"
        );
        // Momentum-bound class (WALL 500 kg, v* = 22.3/500 = 0.0446 m/s; F3).
        let mut w = EnergyCheck::new(500_000, 3_380_000_000_000_000, 22_300_000_000);
        assert_eq!(
            w.exceeds(44_600),
            (false, false),
            "(E) exactly at 22.3 kg·m/s"
        );
        assert_eq!(
            w.exceeds(44_601),
            (false, true),
            "(E) momentum binds first for WALL"
        );
        w.step(Tick(0), 50_000, DRIVE);
        assert!(
            w.step(Tick(1), 50_000, DRIVE)
                .contains(FaultCode::MomentumLimit)
        );
        // A hand-thrown carriage with the LSM idle does not trip.
        let mut h = EnergyCheck::new(m_g, 3_380_000_000_000_000, 22_300_000_000);
        h.persist_ticks = 200;
        for k in 0..1000u64 {
            assert!(
                h.step(Tick(2 * k), 400_000, DriveEvidence::default())
                    .is_empty()
            );
        }
        // D4: a hand-thrown WALL above the momentum cap, met by governor damping (the
        // LSM energised and braking while it slows), never trips; the drive holding it
        // above the cap (F·v > 0) does.
        let brake = DriveEvidence {
            energised: true,
            cmd_braking: true,
            cmd_driving: false,
        };
        let mut t = EnergyCheck::new(500_000, 3_380_000_000_000_000, 22_300_000_000);
        t.persist_ticks = 200;
        t.accel_tol_umps = 200;
        for k in 0..4_000u64 {
            let v = 60_000 - 5 * k as i64; // 60 → 40 mm/s under the brake
            assert!(
                t.step(Tick(2 * k), v, brake).is_empty(),
                "(S) governor damping of a thrown WALL does not trip (D4)"
            );
        }
        let mut d = EnergyCheck::new(500_000, 3_380_000_000_000_000, 22_300_000_000);
        d.persist_ticks = 200;
        let mut tripped = false;
        for k in 0..200u64 {
            tripped |= d
                .step(Tick(2 * k), 46_000, DRIVE)
                .contains(FaultCode::MomentumLimit);
        }
        assert!(
            tripped,
            "(S) the drive holding 23 kg·m/s for > 10 ms trips (D4)"
        );
        // p_max = 0 means "not applicable" (HEAVY hold-to-run).
        let n = EnergyCheck::new(5_000_000, 3_380_000_000_000_000, 0);
        assert!(!n.exceeds(1_000_000).1, "no momentum cap when p_max = 0");
    }
}
