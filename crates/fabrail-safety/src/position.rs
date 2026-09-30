//! Safely-limited position, safe direction and standstill monitoring (SLP/SDI/SSM). Owner: A5.
//!
//! * [`PositionCheck`] on encoder counts (only while the encoder is referenced; an
//!   unreferenced carriage runs hold-to-run in the core): SLP (x outside
//!   [x_min, x_max] → PositionLimit, a controlled stop) and SDI (beyond a limit, only
//!   motion back toward the permitted range is allowed: moving further out faster than
//!   standstill → UnintendedMotion, STO). Wave 2 lead decision D3: both escalate only
//!   while the DRIVE injects energy outward (`DriveEvidence`: energised and commanding
//!   force along an outward velocity, F·v > 0). Floor-driven displacement past a limit
//!   with the LSM in STO, idle or braking (the physical jamb/seat and end stops take it)
//!   is returned as an excursion and counted by [`PositionReport`], never a new trip.
//! * [`StoCheck`] (SSM, the monitor's STO feedback): with STO commanded, the LSM phase
//!   currents must decay below i_lsm_max/10 within the current-decay grace; torque
//!   producing current while STO is commanded → UnintendedMotion (STO + SS1). The check
//!   is suspended while the monitor itself demands ASC (short-circuit braking currents
//!   are intended).
//!
//! Hand motion of an LSM in STO is legitimate (MANUAL, push-to-open), so standstill is
//! supervised through the current, never through the velocity (IEC 61800-5-2 SSM/STO
//! intent; PHYSICS §18). Integer only.
#![deny(clippy::float_arithmetic)]

use fabrail_types::{FaultCode, FaultSet, Tick};

use crate::velocity::DriveEvidence;

/// Position and direction supervision.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PositionCheck {
    /// x_min in encoder counts.
    pub min_counts: i64,
    /// x_max in encoder counts.
    pub max_counts: i64,
    /// Standstill speed threshold, µm/s.
    pub standstill_umps: i64,
}

impl PositionCheck {
    /// One release from counts, velocity (µm/s) and the drive evidence; call only with a
    /// referenced encoder. Returns the faults and whether the carriage is outside the
    /// range (an excursion, tripping or reported).
    ///
    /// Outside the range the check trips only while the drive does positive work along
    /// an outward velocity (D3): PositionLimit (controlled stop) and, moving further out
    /// faster than standstill, UnintendedMotion (STO). A drive that brakes, is idle or in
    /// STO, or pushes inward, never trips here.
    pub fn step(&self, counts: i64, v_umps: i64, ev: DriveEvidence) -> (FaultSet, bool) {
        let mut f = FaultSet::EMPTY;
        let still = self.standstill_umps.max(0);
        let above = counts > self.max_counts;
        let below = counts < self.min_counts;
        let outward = (above && v_umps > still) || (below && v_umps < -still);
        if outward && ev.energised && ev.cmd_driving {
            f.set(FaultCode::PositionLimit);
            f.set(FaultCode::UnintendedMotion);
        }
        (f, above || below)
    }
}

/// Reported position excursions without drive work (D3): how many excursions began,
/// the deepest excursion past the limit (counts) and the tick of the latest one. Pure
/// reporting state for telemetry and the simulator; it never demands anything.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PositionReport {
    /// An excursion is in progress.
    pub active: bool,
    /// Excursions begun since boot.
    pub count: u32,
    /// Deepest excursion beyond x_min/x_max, counts.
    pub max_depth_counts: i64,
    /// Tick at which the latest excursion began.
    pub last_start: Tick,
}

impl PositionReport {
    /// Updates the record from this release's excursion flag and position.
    pub fn update(&mut self, t: Tick, excursion: bool, counts: i64, pc: &PositionCheck) {
        if excursion {
            if !self.active {
                self.count = self.count.saturating_add(1);
                self.last_start = t;
            }
            let depth = if counts > pc.max_counts {
                counts.saturating_sub(pc.max_counts)
            } else {
                pc.min_counts.saturating_sub(counts)
            };
            if depth > self.max_depth_counts {
                self.max_depth_counts = depth;
            }
        }
        self.active = excursion;
    }
}

/// STO feedback: no torque-producing LSM current while STO is commanded.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StoCheck {
    /// Residual current threshold as Σ i_abc², code².
    pub i_sq_max: u64,
    /// Current-decay grace after STO is commanded, base ticks.
    pub grace_ticks: u32,
    /// Tick since which STO has been commanded continuously.
    pub sto_since: Option<Tick>,
}

impl StoCheck {
    /// One release at `t`: `sto` = STO commanded (and no ASC demanded), `i_sq` =
    /// measured Σ i_abc².
    pub fn step(&mut self, t: Tick, sto: bool, i_sq: u64) -> FaultSet {
        let mut f = FaultSet::EMPTY;
        if !sto {
            self.sto_since = None;
            return f;
        }
        let since = *self.sto_since.get_or_insert(t);
        if t.since(since) > self.grace_ticks as u64 && i_sq > self.i_sq_max {
            f.set(FaultCode::UnintendedMotion);
        }
        f
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slp_limits() {
        // DOOR: x_min = −10 mm, x_max = 1.01 m at 1 µm/count.
        let p = PositionCheck {
            min_counts: -10_000,
            max_counts: 1_010_000,
            standstill_umps: 2_000,
        };
        let drive = DriveEvidence {
            energised: true,
            cmd_braking: false,
            cmd_driving: true,
        };
        let sto = DriveEvidence::default();
        let brake = DriveEvidence {
            energised: true,
            cmd_braking: true,
            cmd_driving: false,
        };
        assert!(p.step(0, 0, drive).0.is_empty(), "closed stop");
        let (f, exc) = p.step(1_010_000, 150_000, drive);
        assert!(f.is_empty() && !exc, "exactly at x_max");
        let (f, exc) = p.step(1_010_001, 0, drive);
        assert!(
            f.is_empty() && exc,
            "past x_max at standstill: no drive work, reported (D3)"
        );
        let (f, _) = p.step(1_020_000, 5_000, drive);
        assert!(
            f.contains(FaultCode::UnintendedMotion) && f.contains(FaultCode::PositionLimit),
            "SDI: the drive pushing further out beyond x_max trips (D3)"
        );
        assert!(
            p.step(1_020_000, -5_000, drive).0.is_empty(),
            "SDI: moving back toward the range is permitted"
        );
        let (f, _) = p.step(-10_001, -3_000, drive);
        assert!(f.contains(FaultCode::PositionLimit) && f.contains(FaultCode::UnintendedMotion));
        // D3: floor-driven slide past x_min with the LSM in STO, or braked by it, is
        // reported (excursion) and never trips.
        for ev in [sto, brake] {
            let (f, exc) = p.step(-25_000, -40_000, ev);
            assert!(
                f.is_empty() && exc,
                "floor-driven excursion is reported, not a trip (D3)"
            );
        }
        let mut rep = PositionReport::default();
        rep.update(Tick(10), true, -25_000, &p);
        rep.update(Tick(12), true, -30_000, &p);
        rep.update(Tick(14), false, 0, &p);
        rep.update(Tick(40), true, 1_011_000, &p);
        assert_eq!(
            (rep.count, rep.max_depth_counts, rep.last_start),
            (2, 20_000, Tick(40))
        );

        // STO feedback: current decays within the grace; residual current afterwards
        // is torque without a command.
        let mut s = StoCheck {
            i_sq_max: 30_000,
            grace_ticks: 20,
            sto_since: None,
        };
        assert!(s.step(Tick(0), true, 3_000_000).is_empty(), "decaying");
        assert!(
            s.step(Tick(20), true, 3_000_000).is_empty(),
            "still inside the grace"
        );
        assert!(
            s.step(Tick(22), true, 3_000_000)
                .contains(FaultCode::UnintendedMotion)
        );
        assert!(s.step(Tick(24), true, 100).is_empty(), "decayed");
        assert!(
            s.step(Tick(26), false, 3_000_000).is_empty(),
            "Run: currents intended"
        );
    }
}
