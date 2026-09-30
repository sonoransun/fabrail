//! Tick-stamped LET mailboxes between the rate groups (FastIn/FastOut, SupIn/SupOut, SeisIn/SeisOut, BaseIn/BaseOut). Owner: A5.
//!
//! Frozen API (W0; core design §A.5, §B.5).
//!
//! **LET rule.** A job released at tick r reads mailbox values whose activation tick is
//! ≤ r and publishes values that activate at r + P, P being its own period in base
//! ticks. [`LetSlot`] holds one active and one pending value; a read at t returns the
//! pending value once its activation tick is ≤ t. Because nothing published at r is
//! visible before r + P, the groups due at the same tick may run in any order and
//! produce identical outputs (the 3! = 6 orders of fast, supervisor and seismic are
//! tested in `tests/let_order.rs`). The base task always runs first in a tick: it
//! latches the `FastRaw` the fast group consumes at that same tick (plan §2b item 9).
//!
//! **Message map.** Each group publishes one `*Out`; the `read_*` accessors assemble a
//! group's `*In` from the other groups' active outputs:
//!
//! | Reader     | Reads (activation ≤ t)                                              |
//! |------------|---------------------------------------------------------------------|
//! | base       | `FastOut.outputs` (arbitrated coil/LSM commands)                    |
//! | fast       | `SupOut.directive`, raw `SupOut.{power, safety_in}`, `SupOut.status.lev` |
//! | supervisor | `FastOut.{status, verdict}`, `SeisOut.report`, `BaseOut`            |
//! | seismic    | `SupOut.status.veto`, `SupOut.inspect_resets`                       |
//!
//! `Default` values are the safe "nothing published yet" messages (levitation off,
//! LSM STO, zero limits).

use fabrail_core::msg::{Directive, FastOutputs, FastStatus, SeismicReport, SupStatus};
use fabrail_params::{ParamError, Rates};
use fabrail_types::{
    CoilCmd, DriverStatus, LevState, LsmCmd, MAX_MAGNETS, PowerRaw, SafetyInRaw, SafetyVerdict,
    Tick,
};

/// One LET mailbox slot: an active value and a pending value with its activation tick.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LetSlot<T: Copy> {
    active: T,
    pending: T,
    pending_at: Tick,
    has_pending: bool,
}

impl<T: Copy> LetSlot<T> {
    /// A slot whose active value is `init`.
    pub const fn new(init: T) -> LetSlot<T> {
        LetSlot {
            active: init,
            pending: init,
            pending_at: Tick(0),
            has_pending: false,
        }
    }

    /// The value active at tick `t` (activation ≤ t).
    #[inline]
    pub fn read(&self, t: Tick) -> T {
        if self.has_pending && self.pending_at <= t {
            self.pending
        } else {
            self.active
        }
    }

    /// Publishes `v` from a job released at `release` with period `period_ticks`: it
    /// activates at `release + period_ticks`. A previous pending value whose activation
    /// is ≤ `release` becomes the active value first.
    #[inline]
    pub fn publish(&mut self, release: Tick, period_ticks: u64, v: T) {
        if self.has_pending && self.pending_at <= release {
            self.active = self.pending;
        }
        self.pending = v;
        self.pending_at = release.plus(period_ticks);
        self.has_pending = true;
    }

    /// The most recently published value, activated or not (recorder/snapshot only;
    /// never read by a group).
    #[inline]
    pub fn latest(&self) -> T {
        if self.has_pending {
            self.pending
        } else {
            self.active
        }
    }
}

/// Base-task input: the arbitrated actuator commands.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BaseIn {
    /// Coil command (current references, or voltages in lag-free test setups).
    pub coil: CoilCmd,
    /// LSM command.
    pub lsm: LsmCmd,
}

/// Base-task output (published every base tick).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BaseOut {
    /// Base tick.
    pub tick: Tick,
    /// Bridge voltages applied (voltage mode) or current references passed (lag mode).
    pub coil_applied: [f32; MAX_MAGNETS],
    /// Coil driver status.
    pub coil_status: DriverStatus,
    /// LSM driver status.
    pub lsm_status: DriverStatus,
}

/// Fast-group input (besides the latched `FastRaw`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FastIn {
    /// Active supervisor directive.
    pub directive: Directive,
    /// Sequence number of the supervisor publication (a change triggers the monitor's
    /// `step_slow` inside the fast group).
    pub sup_seq: u32,
    /// Raw power sample forwarded unchanged for the monitor.
    pub power: PowerRaw,
    /// Raw safety inputs forwarded unchanged for the monitor.
    pub safety_in: SafetyInRaw,
    /// LevState (monitor context).
    pub lev: LevState,
}

/// Fast-group output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FastOut {
    /// Arbitrated actuator commands (what the base task applies).
    pub outputs: FastOutputs,
    /// Fast status (the core heartbeat is `status.seq`).
    pub status: FastStatus,
    /// Combined monitor verdict with the arbitration's latched demand.
    pub verdict: SafetyVerdict,
}

/// Supervisor-group input.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SupIn {
    /// Latest active fast status.
    pub fast: FastStatus,
    /// Latest active combined verdict.
    pub verdict: SafetyVerdict,
    /// Latest active seismic report.
    pub seis: SeismicReport,
    /// Latest active base output (driver status).
    pub base: BaseOut,
}

/// Supervisor-group output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SupOut {
    /// Directive for the fast group.
    pub directive: Directive,
    /// Supervisor status.
    pub status: SupStatus,
    /// Raw power sample (forwarded to the monitor).
    pub power: PowerRaw,
    /// Raw safety inputs (forwarded to the monitor).
    pub safety_in: SafetyInRaw,
    /// Count of INSPECT resets requested so far (a change triggers
    /// `SeismicUnit::reset_inspect`; a counter survives the 10:1 rate ratio).
    pub inspect_resets: u32,
}

/// Seismic-group input.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SeisIn {
    /// Actuator veto (self-actuation).
    pub veto: bool,
    /// INSPECT reset counter from the supervisor.
    pub inspect_resets: u32,
}

/// Seismic-group output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SeisOut {
    /// Seismic report.
    pub report: SeismicReport,
}

/// All LET mailboxes of one executive.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mailboxes {
    base_out: LetSlot<BaseOut>,
    fast_out: LetSlot<FastOut>,
    sup_out: LetSlot<SupOut>,
    seis_out: LetSlot<SeisOut>,
    /// Fast period, base ticks.
    pub fast_period: u64,
    /// Supervisor period, base ticks.
    pub sup_period: u64,
    /// Seismic period, base ticks.
    pub seis_period: u64,
}

impl Mailboxes {
    /// Mailboxes for the given rates (every decimation must be non-zero).
    pub fn new(r: &Rates) -> Result<Mailboxes, ParamError> {
        if r.fast_decim == 0 {
            return Err(ParamError::new("control.rates.fast_decim", "must be > 0"));
        }
        if r.sup_decim == 0 {
            return Err(ParamError::new("control.rates.sup_decim", "must be > 0"));
        }
        if r.seismic_decim == 0 {
            return Err(ParamError::new(
                "control.rates.seismic_decim",
                "must be > 0",
            ));
        }
        Ok(Mailboxes {
            fast_period: r.fast_decim as u64,
            sup_period: r.sup_decim as u64,
            seis_period: r.seismic_decim as u64,
            ..Mailboxes::default()
        })
    }

    /// Base-task input active at `t`.
    pub fn read_base(&self, t: Tick) -> BaseIn {
        let o = self.fast_out.read(t).outputs;
        BaseIn {
            coil: o.coil,
            lsm: o.lsm,
        }
    }

    /// Publishes the base output (period 1 tick).
    pub fn publish_base(&mut self, release: Tick, o: &BaseOut) {
        self.base_out.publish(release, 1, *o);
    }

    /// Fast-group input active at `t`.
    pub fn read_fast(&self, t: Tick) -> FastIn {
        let s = self.sup_out.read(t);
        FastIn {
            directive: s.directive,
            sup_seq: s.status.seq,
            power: s.power,
            safety_in: s.safety_in,
            lev: s.status.lev,
        }
    }

    /// Publishes a fast output released at `release`.
    pub fn publish_fast(&mut self, release: Tick, o: &FastOut) {
        self.fast_out.publish(release, self.fast_period, *o);
    }

    /// Supervisor-group input active at `t`.
    pub fn read_sup(&self, t: Tick) -> SupIn {
        let f = self.fast_out.read(t);
        SupIn {
            fast: f.status,
            verdict: f.verdict,
            seis: self.seis_out.read(t).report,
            base: self.base_out.read(t),
        }
    }

    /// Publishes a supervisor output released at `release`.
    pub fn publish_sup(&mut self, release: Tick, o: &SupOut) {
        self.sup_out.publish(release, self.sup_period, *o);
    }

    /// Seismic-group input active at `t`.
    pub fn read_seis(&self, t: Tick) -> SeisIn {
        let s = self.sup_out.read(t);
        SeisIn {
            veto: s.status.veto,
            inspect_resets: s.inspect_resets,
        }
    }

    /// Publishes a seismic output released at `release`.
    pub fn publish_seis(&mut self, release: Tick, o: &SeisOut) {
        self.seis_out.publish(release, self.seis_period, *o);
    }

    /// Most recently published outputs (recorder / `Executive::snapshot` only).
    pub fn latest(&self) -> (BaseOut, FastOut, SupOut, SeisOut) {
        (
            self.base_out.latest(),
            self.fast_out.latest(),
            self.sup_out.latest(),
            self.seis_out.latest(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn let_slot_activates_after_one_period() {
        let mut s = LetSlot::new(0u32);
        s.publish(Tick(10), 20, 7);
        assert_eq!(s.read(Tick(10)), 0, "not visible at the release tick");
        assert_eq!(s.read(Tick(29)), 0);
        assert_eq!(s.read(Tick(30)), 7, "active at release + P");
        assert_eq!(s.latest(), 7);
        s.publish(Tick(30), 20, 8);
        assert_eq!(
            s.read(Tick(30)),
            7,
            "previous value promoted, new one pending"
        );
        assert_eq!(s.read(Tick(50)), 8);
    }

    #[test]
    fn mailbox_let_visibility() {
        let r = Rates {
            base_hz: 20_000,
            fast_decim: 2,
            sup_decim: 20,
            seismic_decim: 200,
            coil_mode_voltage: true,
        };
        let Ok(mut mb) = Mailboxes::new(&r) else {
            panic!("valid rates");
        };
        // Base output at t = 40 activates at 41 (period 1): the supervisor released at
        // 40 still reads the base output of tick 39.
        mb.publish_base(
            Tick(39),
            &BaseOut {
                tick: Tick(39),
                ..BaseOut::default()
            },
        );
        mb.publish_base(
            Tick(40),
            &BaseOut {
                tick: Tick(40),
                ..BaseOut::default()
            },
        );
        assert_eq!(
            mb.read_sup(Tick(40)).base.tick,
            Tick(39),
            "(E) LET r + 1, PHYSICS §24"
        );
        assert_eq!(mb.read_sup(Tick(41)).base.tick, Tick(40));
        // A fast output released at 40 activates at 42: the base at 40 and 41 still
        // applies the previous commands, the base at 42 the new ones.
        let mut fo = FastOut::default();
        fo.outputs.lsm.iq_ref_a = 1.5;
        fo.status.seq = 7;
        mb.publish_fast(Tick(40), &fo);
        assert_eq!(
            mb.read_base(Tick(40)).lsm.iq_ref_a,
            0.0,
            "(E) not before r + P"
        );
        assert_eq!(mb.read_base(Tick(41)).lsm.iq_ref_a, 0.0);
        assert_eq!(
            mb.read_base(Tick(42)).lsm.iq_ref_a,
            1.5,
            "(E) active at r + P"
        );
        assert_eq!(mb.read_sup(Tick(42)).fast.seq, 7);
        // A supervisor output released at 40 activates at 60 for fast and seismic.
        let mut so = SupOut::default();
        so.status.seq = 3;
        so.inspect_resets = 1;
        mb.publish_sup(Tick(40), &so);
        assert_eq!(mb.read_fast(Tick(58)).sup_seq, 0);
        assert_eq!(mb.read_fast(Tick(60)).sup_seq, 3, "(E) r + P_sup");
        assert_eq!(mb.read_seis(Tick(60)).inspect_resets, 1);
        assert_eq!(mb.latest().2.status.seq, 3, "latest() ignores activation");
    }

    #[test]
    fn zero_decimation_rejected() {
        let r = Rates::default();
        assert!(Mailboxes::new(&r).is_err());
    }
}
