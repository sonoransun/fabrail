//! Executive: new, on_base_tick (SIL), split (RTIC), watchdog feed rule, deadline faults; fault-injection hook (sim only). Owner: A5.
//!
//! API (W0; core design §B.5; PHYSICS §24):
//! * [`Executive::new`] validates the parameters (`SystemParams::validate`) and builds
//!   the four groups and the mailboxes;
//! * [`Executive::on_base_tick`] runs one 20 kHz base tick against a `Board` (SIL and
//!   the golden selftest); [`Executive::on_base_tick_with_policy`] is the same with an
//!   `ActionPolicy` for the supervisor (the exotic/choreographer seam, simulation only);
//!   [`Executive::on_base_tick_in_order`] runs the due groups in a given order (the LET
//!   permutation test; any order gives identical outputs);
//! * [`Executive::split`] hands the groups and mailboxes to the RTIC firmware tasks;
//! * [`Executive::snapshot`] exposes the latest published outputs (recorder, `Session`);
//! * [`Executive::report_overrun`] lets the firmware report a DWT-measured job overrun
//!   (a deadline miss, SAF-013); in SIL a skipped release is detected by the groups;
//! * `inject` / `clear_injection` exist only with the `fault-injection` feature
//!   (enabled only by fabrail-sim); the golden selftest hash is identical with and
//!   without the feature.
//!
//! **Order within a base tick (LET).** The base task ALWAYS runs first: it latches the
//! `FastRaw` that a fast release at the same tick consumes. The groups due at that tick
//! (fast when t % fast_decim == 0, supervisor when t % sup_decim == 0, seismic when
//! t % seismic_decim == 0) then run in any order: each reads mailbox values active at
//! ≤ t and publishes values that activate at t + P, so none can observe another's
//! same-tick output. The permutation test covers all 3! = 6 orders of fast, supervisor
//! and seismic (plan §2b item 9). This executive uses fast → supervisor → seismic.
//!
//! **Watchdog.** Fed by the supervisor group only when every group's sequence number
//! advanced within its deadline and no overrun was flagged ([`WatchdogRule`]).

use fabrail_core::seismic::action_policy::{ActionPolicy, CorePolicy};
use fabrail_hal::{Board, Clock};
use fabrail_params::{ParamError, Rates, SystemParams};
use fabrail_types::{FastRaw, Tick};

#[cfg(feature = "fault-injection")]
use crate::groups::CoreFault;
use crate::groups::{BaseTask, FastGroup, SeismicGroup, SupervisorGroup};
use crate::mailbox::{BaseOut, FastOut, Mailboxes, SeisOut, SupOut};

/// Watchdog feed rule: feed only when the fast and seismic sequence numbers (as seen by
/// the supervisor through the mailboxes) advanced within their deadlines and there was
/// no overrun. Deadline = 2 own periods + 1 supervisor period (LET visibility delay).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WatchdogRule {
    /// Fast deadline, base ticks.
    pub fast_limit_ticks: u64,
    /// Seismic deadline, base ticks.
    pub seis_limit_ticks: u64,
    /// Last fast sequence number seen.
    pub last_fast_seq: u32,
    /// Tick it last changed.
    pub last_fast_change: Tick,
    /// Last seismic sequence number seen.
    pub last_seis_seq: u32,
    /// Tick it last changed.
    pub last_seis_change: Tick,
}

impl WatchdogRule {
    /// Rule for the given rates.
    pub fn new(r: &Rates) -> WatchdogRule {
        let sup = r.sup_decim as u64;
        WatchdogRule {
            fast_limit_ticks: 2 * r.fast_decim as u64 + sup,
            seis_limit_ticks: 2 * r.seismic_decim as u64 + sup,
            ..WatchdogRule::default()
        }
    }

    /// One supervisor release at `t`; true when the watchdog may be fed.
    pub fn check(&mut self, t: Tick, fast_seq: u32, seis_seq: u32, overrun: bool) -> bool {
        if fast_seq != self.last_fast_seq {
            self.last_fast_seq = fast_seq;
            self.last_fast_change = t;
        }
        if seis_seq != self.last_seis_seq {
            self.last_seis_seq = seis_seq;
            self.last_seis_change = t;
        }
        !overrun
            && t.since(self.last_fast_change) <= self.fast_limit_ticks
            && t.since(self.last_seis_change) <= self.seis_limit_ticks
    }
}

/// A rate group that runs after the base task within a tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GroupId {
    /// Fast group (10 / 5 kHz).
    Fast,
    /// Supervisor group (1 kHz).
    Supervisor,
    /// Seismic group (100 Hz).
    Seismic,
}

impl GroupId {
    /// The executive's order: fast → supervisor → seismic.
    pub const DEFAULT_ORDER: [GroupId; 3] = [GroupId::Fast, GroupId::Supervisor, GroupId::Seismic];

    /// All 3! = 6 orders (the LET permutation test, plan §2b item 9).
    pub const ALL_ORDERS: [[GroupId; 3]; 6] = [
        [GroupId::Fast, GroupId::Supervisor, GroupId::Seismic],
        [GroupId::Fast, GroupId::Seismic, GroupId::Supervisor],
        [GroupId::Supervisor, GroupId::Fast, GroupId::Seismic],
        [GroupId::Supervisor, GroupId::Seismic, GroupId::Fast],
        [GroupId::Seismic, GroupId::Fast, GroupId::Supervisor],
        [GroupId::Seismic, GroupId::Supervisor, GroupId::Fast],
    ];

    #[inline]
    const fn bit(self) -> u8 {
        match self {
            GroupId::Fast => 1,
            GroupId::Supervisor => 2,
            GroupId::Seismic => 4,
        }
    }
}

/// The latest published outputs of every group (not LET-activated; recorder only).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ExecSnapshot {
    /// Last base tick executed.
    pub tick: Tick,
    /// Base output.
    pub base: BaseOut,
    /// Fast output (arbitrated commands, status, verdict).
    pub fast: FastOut,
    /// Supervisor output (directive, status).
    pub sup: SupOut,
    /// Seismic output (report).
    pub seis: SeisOut,
}

/// The executive: the exact group code the MCU runs, driven tick by tick.
#[derive(Clone, Debug)]
pub struct Executive {
    /// Base task.
    pub base: BaseTask,
    /// Fast group.
    pub fast: FastGroup,
    /// Supervisor group.
    pub sup: SupervisorGroup,
    /// Seismic group.
    pub seis: SeismicGroup,
    /// LET mailboxes.
    pub mb: Mailboxes,
    /// Latest latched raw sample.
    pub last_raw: FastRaw,
    /// Rates.
    pub rates: Rates,
    /// Last base tick executed.
    pub tick: Tick,
}

impl Executive {
    /// Validates `p` and builds every group.
    pub fn new(p: &SystemParams) -> Result<Executive, ParamError> {
        p.validate()?;
        let rates = p.control.rates;
        Ok(Executive {
            base: BaseTask::new(p)?,
            fast: FastGroup::new(p)?,
            sup: SupervisorGroup::new(p)?,
            seis: SeismicGroup::new(p)?,
            mb: Mailboxes::new(&rates)?,
            last_raw: FastRaw::default(),
            rates,
            tick: Tick(0),
        })
    }

    /// One base tick with the core seismic policy.
    pub fn on_base_tick<B: Board>(&mut self, b: &mut B) {
        self.on_base_tick_with_policy(b, &mut CorePolicy);
    }

    /// One base tick; the supervisor consults `policy` for the seismic action.
    pub fn on_base_tick_with_policy<B: Board>(&mut self, b: &mut B, policy: &mut dyn ActionPolicy) {
        self.on_base_tick_in_order(b, policy, GroupId::DEFAULT_ORDER);
    }

    /// One base tick running the due groups in `order` after the base task. Each due
    /// group runs exactly once per tick whatever `order` holds (a group missing from it
    /// runs afterwards in the default order); by LET every order gives identical
    /// outputs.
    pub fn on_base_tick_in_order<B: Board>(
        &mut self,
        b: &mut B,
        policy: &mut dyn ActionPolicy,
        order: [GroupId; 3],
    ) {
        let t = <B as Clock>::now(b);
        self.tick = t;
        // 1. Base first (latches the FastRaw this tick's fast release consumes).
        let bi = self.mb.read_base(t);
        let (bo, raw) = self.base.run(t, &bi, b);
        self.mb.publish_base(t, &bo);
        if let Some(r) = raw {
            self.last_raw = r;
        }
        // 2. Due groups; any order gives the same result (LET).
        let mut ran = 0u8;
        for g in order.into_iter().chain(GroupId::DEFAULT_ORDER) {
            if ran & g.bit() != 0 {
                continue;
            }
            ran |= g.bit();
            match g {
                GroupId::Fast => self.run_fast(t),
                GroupId::Supervisor => self.run_sup(t, b, policy),
                GroupId::Seismic => self.run_seis(t, b),
            }
        }
    }

    fn run_fast(&mut self, t: Tick) {
        if t.is_release(self.rates.fast_decim as u32) {
            let fi = self.mb.read_fast(t);
            let fo = self.fast.run(t, &self.last_raw, &fi);
            self.mb.publish_fast(t, &fo);
        }
    }

    fn run_sup<B: Board>(&mut self, t: Tick, b: &mut B, policy: &mut dyn ActionPolicy) {
        if t.is_release(self.rates.sup_decim as u32) {
            let si = self.mb.read_sup(t);
            let so = self.sup.run_with_policy(t, &si, b, policy);
            self.mb.publish_sup(t, &so);
        }
    }

    fn run_seis<B: Board>(&mut self, t: Tick, b: &mut B) {
        if t.is_release(self.rates.seismic_decim as u32) {
            let si = self.mb.read_seis(t);
            let so = self.seis.run(t, &si, b);
            self.mb.publish_seis(t, &so);
        }
    }

    /// Reports a job overrun measured by the firmware (DWT): the group raises its
    /// overrun fault at its next release (SAF-013).
    pub fn report_overrun(&mut self, g: GroupId) {
        match g {
            GroupId::Fast => self.fast.report_overrun(),
            GroupId::Supervisor => self.sup.report_overrun(),
            GroupId::Seismic => self.seis.report_overrun(),
        }
    }

    /// Hands the groups and mailboxes to the firmware's RTIC tasks.
    pub fn split(
        self,
    ) -> (
        BaseTask,
        FastGroup,
        SupervisorGroup,
        SeismicGroup,
        Mailboxes,
    ) {
        (self.base, self.fast, self.sup, self.seis, self.mb)
    }

    /// Latest published outputs.
    pub fn snapshot(&self) -> ExecSnapshot {
        let (base, fast, sup, seis) = self.mb.latest();
        ExecSnapshot {
            tick: self.tick,
            base,
            fast,
            sup,
            seis,
        }
    }

    /// Injects a core fault into the fast group's outputs before the monitor sees them
    /// (simulation only).
    #[cfg(feature = "fault-injection")]
    pub fn inject(&mut self, f: CoreFault) {
        self.fast.inject = Some(f);
    }

    /// Removes an injected core fault.
    #[cfg(feature = "fault-injection")]
    pub fn clear_injection(&mut self) {
        self.fast.inject = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::selftest::{LcgBoard, selftest_params};
    use fabrail_types::{FaultCode, SafetyDemand};

    #[test]
    fn watchdog_feed_rule() {
        let r = Rates {
            base_hz: 20_000,
            fast_decim: 2,
            sup_decim: 20,
            seismic_decim: 200,
            coil_mode_voltage: true,
        };
        let mut w = WatchdogRule::new(&r);
        assert!(w.check(Tick(20), 10, 0, false));
        assert!(!w.check(Tick(40), 20, 0, true), "overrun blocks the feed");
        // Fast stalls: seq unchanged beyond 2·2 + 20 = 24 ticks.
        assert!(w.check(Tick(60), 20, 0, false));
        assert!(!w.check(Tick(80), 20, 0, false));
        // Seismic stalls: unchanged beyond 2·200 + 20 = 420 ticks.
        let mut w = WatchdogRule::new(&r);
        let mut seq = 0;
        for k in 1..=21u64 {
            seq += 1;
            assert!(w.check(Tick(20 * k), seq, 0, false), "t = {}", 20 * k);
        }
        assert!(
            !w.check(Tick(440), seq + 1, 0, false),
            "(T) seismic stalled > 420 ticks"
        );
    }

    // req: FAB-SAF-013
    #[test]
    fn deadline_miss_faults() {
        let p = selftest_params();
        let mut ex = Executive::new(&p).unwrap_or_else(|e| panic!("selftest params: {e}"));
        let mut b = LcgBoard::new();
        let fast = p.control.rates.fast_decim as u64;
        let sup = p.control.rates.sup_decim as u64;
        // 50 ms of nominal execution (ticks 0..=1000, ending on a supervisor
        // release): no execution faults, watchdog fed.
        for _ in 0..=1_000 {
            ex.on_base_tick(&mut b);
            b.advance();
        }
        let snap = ex.snapshot();
        assert!(!snap.fast.status.faults.contains(FaultCode::FastOverrun));
        assert!(!snap.fast.verdict.faults.contains(FaultCode::FastOverrun));
        let fed = b.feeds();
        assert!(
            fed > 0,
            "the watchdog is fed while every group keeps its deadline"
        );

        // The fast job misses its releases: the clock jumps by 3 fast periods + 1 tick
        // (a stalled or overrunning job on firmware), without skipping a supervisor
        // release.
        b.skip(3 * fast + 1);
        for _ in 0..(2 * sup) {
            ex.on_base_tick(&mut b);
            b.advance();
        }
        // The fast group raised FastOverrun (SS1) and it stays visible for the
        // demand-latch window, so the next supervisor release reads it.
        let si = ex.mb.read_sup(ex.tick);
        assert!(
            si.verdict.faults.contains(FaultCode::FastOverrun),
            "(T) deadline miss → FastOverrun in the verdict the supervisor reads, SAF-013"
        );
        assert!(
            si.verdict.demand >= SafetyDemand::SafeStop,
            "(T) demand ≥ SafeStop forces FAULT_REACTION (arbitration rule 3), SAF-013: {:?}",
            si.verdict.demand
        );
        assert!(si.fast.faults.contains(FaultCode::FastOverrun));
        assert_eq!(ex.fast.deadline.misses, 1);
        assert_eq!(
            ex.sup.deadline.misses, 0,
            "no supervisor release was skipped"
        );
        // No watchdog feed while the overrun is visible (the 5 ms IWDG bites only if
        // misses persist); feeding resumes once it has cleared.
        assert_eq!(
            b.feeds(),
            fed,
            "(T) no watchdog feed after an overrun, SAF-013"
        );
        for _ in 0..(3 * sup) {
            ex.on_base_tick(&mut b);
            b.advance();
        }
        assert!(b.feeds() > fed, "feeding resumes after a single overrun");

        // A firmware-reported supervisor overrun: the core supervisor's own verdict
        // carries SupervisorOverrun at its next release.
        ex.report_overrun(GroupId::Supervisor);
        for _ in 0..sup {
            ex.on_base_tick(&mut b);
            b.advance();
        }
        let so = ex.snapshot().sup;
        assert!(
            so.status.faults.contains(FaultCode::SupervisorOverrun),
            "(T) supervisor deadline miss reported to the supervisor, SAF-013"
        );
        assert_eq!(ex.sup.deadline.misses, 1);

        // A fast release more than 3 supervisor periods late: the monitor's heartbeat
        // deadline also fires (HeartbeatLost → STO, latched until Reset).
        let mut fg = crate::groups::FastGroup::new(&p).unwrap_or_else(|e| panic!("{e}"));
        let raw = ex.last_raw;
        let fi = ex.mb.read_fast(ex.tick);
        let _ = fg.run(Tick(1_000_000), &raw, &fi);
        let late = fg.run(Tick(1_000_000 + 4 * sup), &raw, &fi);
        assert!(late.verdict.faults.contains(FaultCode::FastOverrun));
        assert!(
            late.verdict.faults.contains(FaultCode::HeartbeatLost),
            "(T) heartbeat deadline > 3 periods, SAF-013"
        );
        assert!(
            late.verdict.demand >= SafetyDemand::StoLsm,
            "(T) STO on the LSM"
        );
    }
}
