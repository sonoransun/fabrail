//! Fault aggregation and reactions: FAULT_REACTION is SS1 Safe Landing with STO on the LSM only (PHYSICS §18). Owner: A4.
//!
//! Frozen API (W0): [`FaultManager::new`], [`FaultManager::step`] → [`FaultReaction`],
//! [`FaultManager::reset`]. Faults from the fast group, the seismic unit, the monitor
//! verdict and the supervisor are latched; the reaction demand is the maximum of the
//! latched default severities and the monitor demand, which the core can never lower
//! (SAF-016). A demand ≥ SafeStop forces FAULT_REACTION at the next supervisor release.
//! `PayloadMismatch` is `Degrade`: levitated, hold-to-run only (plan §2b item 10).
//!
//! Latch rules (PHYSICS §18; SAF-013):
//!
//! * a code of severity ≥ `Degrade` latches until [`FaultManager::reset`]
//!   (`UserCommand::Reset` after inspection), except the live conditions in
//!   [`LIVE_CODES`] (`NotReferenced` clears once homed, `HoldupLow` once the supercap
//!   has charged), which are reported while raised;
//! * `Info`/`Warn` codes are events: reported in the release they are raised (and by the
//!   event ring), never latched, never a reaction;
//! * the reaction demand is max(latched severity demand, monitor demand), so neither
//!   the core nor a reset in the same release can lower a monitor demand.
//!
//! Capability consequences of the reported set: `PayloadMismatch` and `NotReferenced`
//! force hold-to-run (F18; plan §2b item 10); `HoldupLow`, `ParamCrc` and
//! `SelftestHash` refuse lift (SAF-009, SAF-015); `ObstructionDegraded` forbids automatic
//! (unattended) motion (plan §4b).

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{FaultCode, FaultSet, SafetyDemand, SafetyVerdict, Severity};

/// Degrade-class conditions that are reported while raised instead of latched: they
/// describe the present state of the machine (encoder not yet homed, supercap still
/// charging) and clear without an inspection.
pub const LIVE_CODES: [FaultCode; 2] = [FaultCode::NotReferenced, FaultCode::HoldupLow];

/// True when `c` latches until Reset.
pub fn latches(c: FaultCode) -> bool {
    c.severity() >= Severity::Degrade && !LIVE_CODES.contains(&c)
}

/// Result of fault aggregation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FaultReaction {
    /// Latched faults (plus the live conditions and events raised this release).
    pub latched: FaultSet,
    /// Reaction demand.
    pub demand: SafetyDemand,
    /// Enter FAULT_REACTION (demand ≥ SafeStop).
    pub fault_reaction: bool,
    /// Faults that became visible this release (event log).
    pub raised: FaultSet,
    /// Motion is hold-to-run only (`PayloadMismatch`, `NotReferenced`).
    pub hold_to_run: bool,
    /// Lift refused (`HoldupLow`, `ParamCrc`, `SelftestHash`, or a fault reaction).
    pub refuse_lift: bool,
    /// Automatic (unattended) motion forbidden (`ObstructionDegraded`).
    pub no_auto_motion: bool,
}

/// Fault latch.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FaultManager {
    /// Latched faults.
    pub latched: FaultSet,
    /// Everything raised in the last release (edge detection for events).
    pub live: FaultSet,
}

impl FaultManager {
    /// Fault latch (no parameters: the latch rules are fixed by the registry).
    pub fn new(p: &SystemParams) -> Result<FaultManager, ParamError> {
        let _ = p;
        Ok(FaultManager::default())
    }

    /// Latches this release's faults and computes the reaction.
    pub fn step(
        &mut self,
        fast: FaultSet,
        seis: FaultSet,
        own: FaultSet,
        verdict: &SafetyVerdict,
    ) -> FaultReaction {
        let now = fast.union(seis).union(own).union(verdict.faults);
        let before = self.latched.union(self.live);
        for c in now.iter() {
            if latches(c) {
                self.latched.set(c);
            }
        }
        self.live = now;
        let report = self.latched.union(now);
        let mut raised = FaultSet::EMPTY;
        for c in report.iter() {
            if !before.contains(c) {
                raised.set(c);
            }
        }
        let demand = core::cmp::max(report.demand(), verdict.demand);
        let fault_reaction = demand >= SafetyDemand::SafeStop;
        FaultReaction {
            latched: report,
            demand,
            fault_reaction,
            raised,
            hold_to_run: report.contains(FaultCode::PayloadMismatch)
                || report.contains(FaultCode::NotReferenced),
            refuse_lift: fault_reaction
                || report.contains(FaultCode::HoldupLow)
                || report.contains(FaultCode::ParamCrc)
                || report.contains(FaultCode::SelftestHash),
            no_auto_motion: report.contains(FaultCode::ObstructionDegraded),
        }
    }

    /// Clears the latch (`UserCommand::Reset` after manual inspection). Sources that
    /// are still present are raised again at the next [`FaultManager::step`].
    pub fn reset(&mut self) {
        self.latched = FaultSet::EMPTY;
        self.live = FaultSet::EMPTY;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn one(c: FaultCode) -> FaultSet {
        let mut f = FaultSet::EMPTY;
        f.set(c);
        f
    }

    #[test]
    fn latch_and_demand() {
        let mut fm = FaultManager::default();
        let r = fm.step(
            one(FaultCode::PayloadMismatch),
            FaultSet::EMPTY,
            FaultSet::EMPTY,
            &SafetyVerdict::default(),
        );
        assert_eq!(r.demand, SafetyDemand::None, "PayloadMismatch is Degrade");
        assert!(!r.fault_reaction);
        assert!(
            r.hold_to_run,
            "Degrade → hold-to-run only (plan §2b item 10)"
        );
        let r = fm.step(
            one(FaultCode::MainsLost),
            FaultSet::EMPTY,
            FaultSet::EMPTY,
            &SafetyVerdict::default(),
        );
        assert_eq!(r.demand, SafetyDemand::SafeLand);
        assert!(r.fault_reaction && r.refuse_lift);
        assert!(r.raised.contains(FaultCode::MainsLost));
        assert!(!r.raised.contains(FaultCode::PayloadMismatch), "edge only");
        // Latched even when the source clears.
        let r = fm.step(
            FaultSet::EMPTY,
            FaultSet::EMPTY,
            FaultSet::EMPTY,
            &SafetyVerdict::default(),
        );
        assert!(r.latched.contains(FaultCode::MainsLost));
        assert!(r.latched.contains(FaultCode::PayloadMismatch));
        assert!(r.fault_reaction);
        fm.reset();
        assert!(fm.latched.is_empty());
    }

    /// Live conditions and Warn events are reported while raised and never latch.
    #[test]
    fn live_codes_and_warnings_do_not_latch() {
        let mut fm = FaultManager::default();
        let mut f = one(FaultCode::NotReferenced);
        f.set(FaultCode::HoldupLow);
        f.set(FaultCode::NotHoldToRun);
        let r = fm.step(
            f,
            FaultSet::EMPTY,
            FaultSet::EMPTY,
            &SafetyVerdict::default(),
        );
        assert!(r.hold_to_run && r.refuse_lift && !r.fault_reaction);
        assert!(
            r.latched.contains(FaultCode::NotHoldToRun),
            "reported this release"
        );
        let r = fm.step(
            FaultSet::EMPTY,
            FaultSet::EMPTY,
            FaultSet::EMPTY,
            &SafetyVerdict::default(),
        );
        assert!(r.latched.is_empty(), "{:?}", r.latched);
        assert!(!r.hold_to_run && !r.refuse_lift);
        assert!(!latches(FaultCode::Obstruction) && latches(FaultCode::ObstructionDegraded));
    }

    /// The monitor demand is never lowered (SAF-016), even with no latched fault.
    #[test]
    fn monitor_demand_passes_through() {
        let mut fm = FaultManager::default();
        let v = SafetyVerdict {
            demand: SafetyDemand::StoLsmAsc,
            ..SafetyVerdict::default()
        };
        let r = fm.step(FaultSet::EMPTY, FaultSet::EMPTY, FaultSet::EMPTY, &v);
        assert_eq!(r.demand, SafetyDemand::StoLsmAsc);
        assert!(r.fault_reaction);
    }
}
