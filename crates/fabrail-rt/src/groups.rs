//! Rate groups: BaseTask (HAL, current loops, latches FastRaw), FastGroup (pure), SupervisorGroup, SeismicGroup. Owner: A5.
//!
//! API (W0; core design §A.5, §B.5, §F task map; PHYSICS §24):
//!
//! | Group | Rate | Signature | HAL |
//! |---|---|---|---|
//! | [`BaseTask`] | 20 kHz | `run<H: BaseHal>(t, &BaseIn, h) -> (BaseOut, Option<FastRaw>)` | coils, LSM, gap, encoder, IMU, rail accel |
//! | [`FastGroup`] | 10 / 5 kHz | `run(t, &FastRaw, &FastIn) -> FastOut` (pure) | none |
//! | [`SupervisorGroup`] | 1 kHz | `run<H: SupervisorHal>(t, &SupIn, h) -> SupOut` | power, safety inputs, aux, command link, watchdog, telemetry |
//! | [`SeismicGroup`] | 100 Hz | `run<H: SeismicHal>(t, &SeisIn, h) -> SeisOut` | structure accelerometer, alert link |
//!
//! * Base: samples coil currents every tick; in voltage coil mode runs the coil current
//!   PI (`core::current::CurrentLoops`) on the fast group's current references (the only
//!   mode in which SIL ≡ firmware is claimed, plan §2b item 16), in lag mode passes the
//!   references through; passes the arbitrated LSM d/q command to the inverter, whose
//!   d/q current PI (`core::foc::DqPi` on firmware) realises it (in SIL the LSM is the
//!   current-controlled FOC abstraction of PHYSICS §7); commands take effect at k + 1
//!   (ZOH); at fast releases latches the complete `FastRaw`. A non-finite bridge voltage
//!   never reaches the HAL (0 V, counted in `nonfinite_events`). The PI limits use the
//!   nominal `design.power.v_bus_v`: `BaseHal` carries no 20 kHz bus sample.
//! * Fast: frontend (`Frontend::from_params`: LSM pole pitch for i_d/i_q, the fitted
//!   gap channels; stuck-gap check armed while levitated) → `FastLoop::step` → (fault
//!   injection, sim only) → monitor
//!   (`step_slow` when a new supervisor publication is active, then `step_fast`) →
//!   arbitration. A pure function of (FastRaw, FastIn) and its own state.
//! * Supervisor: samples its HAL inputs, runs `Supervisor::step_with_policy`, commands
//!   aux, emits telemetry frames/events and feeds the watchdog under
//!   `exec::WatchdogRule`. Raw power/safety samples are forwarded unchanged to the
//!   monitor through the mailbox.
//! * Seismic: drains the ADXL355 FIFO (≤ `ACCEL_DRAIN_MAX`), polls the alert link and
//!   runs `SeismicUnit::step`; applies an INSPECT reset when the supervisor's counter
//!   changes.
//!
//! **Deadline supervision** ([`ReleaseCheck`], SAF-013). Each group checks that its
//! release arrives no later than one period after the previous one; in SIL a skipped
//! release (a clock that jumps) is a deadline miss, and firmware reports DWT-measured
//! overruns through `report_overrun`. A fast miss raises FastOverrun in the fast status
//! and the verdict (SS1 demand, sticky for the demand-latch window so the supervisor's
//! next release sees it); a supervisor miss raises SupervisorOverrun in the verdict the
//! core supervisor reads (so FAULT_REACTION follows at once) and blocks the watchdog
//! feed; a seismic miss raises SeismicOverrun (Warn) in the report.

use fabrail_core::current::CurrentLoops;
use fabrail_core::fast::FastLoop;
use fabrail_core::frontend::Frontend;
use fabrail_core::msg::SupInputs;
use fabrail_core::seismic::SeismicUnit;
use fabrail_core::seismic::action_policy::{ActionPolicy, CorePolicy};
use fabrail_core::supervisor::Supervisor;
use fabrail_core::telemetry::assemble_frame;
use fabrail_hal::{
    Accelerometer, AlertLink, Aux, BaseHal, CarriageImu, CoilDrivers, CommandLink, Encoder,
    GapSensors, LsmDriver, PowerMonitor, RailAccel, SafetyInputs, SeismicHal, SupervisorHal,
    Telemetry, Watchdog,
};
use fabrail_params::{ParamError, SystemParams};
use fabrail_safety::Monitor;
use fabrail_types::{
    ACCEL_DRAIN_MAX, AccelRaw, CoilCmd, FastRaw, FaultCode, FaultSet, MAX_MAGNETS, MotionMode,
    RailClass, Tick,
};

use crate::arbitrate::Arbiter;
use crate::exec::WatchdogRule;
use crate::mailbox::{BaseIn, BaseOut, FastIn, FastOut, SeisIn, SeisOut, SupIn, SupOut};

/// Deadline supervision of one rate group: a release more than one period after the
/// previous one (missed releases), or an overrun reported by the firmware, is a miss.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReleaseCheck {
    /// Group period, base ticks.
    pub period_ticks: u64,
    /// Previous release.
    pub last: Option<Tick>,
    /// An overrun was reported since the last release (firmware DWT).
    pub overrun_pending: bool,
    /// Deadline misses so far.
    pub misses: u32,
}

impl ReleaseCheck {
    /// Supervision for a group of the given period (base ticks).
    pub const fn new(period_ticks: u64) -> ReleaseCheck {
        ReleaseCheck {
            period_ticks,
            last: None,
            overrun_pending: false,
            misses: 0,
        }
    }

    /// Records a release at `t`; true when a deadline was missed since the previous
    /// release.
    pub fn release(&mut self, t: Tick) -> bool {
        let late = self.last.is_some_and(|l| t.since(l) > self.period_ticks);
        self.last = Some(t);
        let missed = late || self.overrun_pending;
        self.overrun_pending = false;
        if missed {
            self.misses = self.misses.saturating_add(1);
        }
        missed
    }

    /// Reports an overrun of the current job (firmware: DWT cycle count beyond r + P).
    pub fn report_overrun(&mut self) {
        self.overrun_pending = true;
    }
}

/// 20 kHz base task.
#[derive(Clone, Debug)]
pub struct BaseTask {
    /// Coil current loops (voltage mode).
    pub loops: CurrentLoops,
    /// Voltage coil mode.
    pub voltage_mode: bool,
    /// Fast decimation (FastRaw latch).
    pub fast_decim: u16,
    /// Coil current scale, A/code (controller calibration).
    pub coil_a_per_code: f32,
    /// Nominal bus voltage for the PI limits, V.
    pub v_bus_v: f32,
    /// Non-finite bridge voltages replaced by 0 V so far.
    pub nonfinite_events: u32,
}

impl BaseTask {
    /// Base task from the parameters.
    pub fn new(p: &SystemParams) -> Result<BaseTask, ParamError> {
        Ok(BaseTask {
            loops: CurrentLoops::new(p)?,
            voltage_mode: p.control.rates.coil_mode_voltage,
            fast_decim: p.control.rates.fast_decim,
            coil_a_per_code: p.design.sensors.coil_a_per_code as f32,
            v_bus_v: p.design.power.v_bus_v as f32,
            nonfinite_events: 0,
        })
    }

    /// One base tick at `t`: sample, command, and latch `FastRaw` at fast releases.
    pub fn run<H: BaseHal>(
        &mut self,
        t: Tick,
        inp: &BaseIn,
        h: &mut H,
    ) -> (BaseOut, Option<FastRaw>) {
        let cur = <H as CoilDrivers>::sample(h);
        let mut cmd = match (self.voltage_mode, inp.coil) {
            (true, CoilCmd::CurrentRef(refs)) => {
                let mut meas = [0.0f32; MAX_MAGNETS];
                for (m, c) in meas.iter_mut().zip(cur.code.iter()) {
                    *m = *c as f32 * self.coil_a_per_code;
                }
                CoilCmd::Voltage(self.loops.step(&refs, &meas, self.v_bus_v))
            }
            (_, CoilCmd::Off) => {
                self.loops.reset();
                CoilCmd::Off
            }
            (_, c) => c,
        };
        if let CoilCmd::CurrentRef(v) | CoilCmd::Voltage(v) = &mut cmd {
            for x in v.iter_mut() {
                if !x.is_finite() {
                    *x = 0.0;
                    self.nonfinite_events = self.nonfinite_events.saturating_add(1);
                }
            }
        }
        <H as CoilDrivers>::command(h, &cmd);
        <H as LsmDriver>::command(h, &inp.lsm);
        let raw = if t.is_release(self.fast_decim as u32) {
            Some(FastRaw {
                tick: t,
                gap: <H as GapSensors>::sample(h),
                cur,
                enc: <H as Encoder>::sample(h),
                lsm: <H as LsmDriver>::sample(h),
                imu: <H as CarriageImu>::sample(h),
                rail: <H as RailAccel>::sample(h),
            })
        } else {
            None
        };
        let out = BaseOut {
            tick: t,
            coil_applied: cmd.values(),
            coil_status: <H as CoilDrivers>::status(h),
            lsm_status: <H as LsmDriver>::status(h),
        };
        (out, raw)
    }
}

/// Core fault injected into the fast group's outputs BEFORE the monitor sees them
/// (simulation only; scenario events `core_fault: iq_gain_x3 | gap_sign | nan`).
#[cfg(feature = "fault-injection")]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CoreFault {
    /// LSM quadrature current reference multiplied by 3.
    IqGainX3,
    /// Coil current references sign-inverted (a gap-sign error's effect).
    GapSign,
    /// NaN in every output.
    Nan,
}

/// Fast group (10 / 5 kHz): pure function of (FastRaw, FastIn) and its own state.
#[derive(Clone, Debug)]
pub struct FastGroup {
    /// Frontend (raw → measurements).
    pub frontend: Frontend,
    /// Core fast loop.
    pub fast: FastLoop,
    /// Independent safety monitor (both fast and slow checks run here).
    pub monitor: Monitor,
    /// Safety arbitration.
    pub arbiter: Arbiter,
    /// Supervisor publication last passed to `Monitor::step_slow`.
    pub last_sup_seq: Option<u32>,
    /// Deadline supervision of the fast releases.
    pub deadline: ReleaseCheck,
    /// Execution faults kept visible until `exec_faults_until` (FastOverrun).
    pub exec_faults: FaultSet,
    /// End of the execution-fault visibility window (2 supervisor periods).
    pub exec_faults_until: Tick,
    /// Visibility window of an execution fault, base ticks.
    pub exec_window_ticks: u64,
    /// Active injected core fault (simulation only).
    #[cfg(feature = "fault-injection")]
    pub inject: Option<CoreFault>,
}

impl FastGroup {
    /// Fast group from the parameters.
    pub fn new(p: &SystemParams) -> Result<FastGroup, ParamError> {
        let r = &p.control.rates;
        Ok(FastGroup {
            frontend: Frontend::from_params(p),
            fast: FastLoop::new(p)?,
            monitor: Monitor::new(&p.safety, &p.design.sensors),
            arbiter: Arbiter::new(p)?,
            last_sup_seq: None,
            deadline: ReleaseCheck::new(r.fast_decim as u64),
            exec_faults: FaultSet::EMPTY,
            exec_faults_until: Tick(0),
            exec_window_ticks: 2 * r.sup_decim as u64,
            #[cfg(feature = "fault-injection")]
            inject: None,
        })
    }

    /// One fast release at `t`.
    pub fn run(&mut self, t: Tick, raw: &FastRaw, inp: &FastIn) -> FastOut {
        if self.deadline.release(t) {
            self.exec_faults.set(FaultCode::FastOverrun);
            self.exec_faults_until = t.plus(self.exec_window_ticks);
        } else if t > self.exec_faults_until {
            self.exec_faults = FaultSet::EMPTY;
        }
        let (m, fe_faults) = self.frontend.step(raw);
        let (mut out, mut status) = self.fast.step(&m, &inp.directive);
        // Stuck-gap detection (GapStuck) only while levitated: the unseat/lift ramps and
        // the rest states change the current at a legitimately frozen gap (A1 contract).
        self.frontend
            .set_stuck_check(status.lev == fabrail_core::msg::LevPhase::Levitated);
        status.faults.merge(fe_faults);
        status.faults.merge(self.exec_faults);
        #[cfg(feature = "fault-injection")]
        self.apply_injection(&mut out);
        // Slow checks on each new supervisor publication. Before the first one is active
        // (sup_seq 0: the supervisor numbers its releases from 1) the forwarded raw
        // samples are the mailbox defaults, not measurements, and are not checked.
        let published = inp.sup_seq != 0 || self.last_sup_seq.is_some();
        if published && self.last_sup_seq != Some(inp.sup_seq) {
            self.monitor
                .step_slow(t, &inp.power, &inp.safety_in, inp.lev);
            self.last_sup_seq = Some(inp.sup_seq);
        }
        self.monitor
            .set_isolate(inp.directive.mode == MotionMode::Isolate);
        let mut verdict = self
            .monitor
            .step_fast(t, raw, &out.coil, &out.lsm, status.seq);
        verdict.faults.merge(self.exec_faults);
        verdict.demand = verdict.demand.max(self.exec_faults.demand());
        let (arb, demand) = self.arbiter.step(t, &out, &verdict);
        out = arb;
        verdict.demand = demand;
        FastOut {
            outputs: out,
            status,
            verdict,
        }
    }

    /// Reports an overrun of the fast job (firmware DWT); raised at the next release.
    pub fn report_overrun(&mut self) {
        self.deadline.report_overrun();
    }

    #[cfg(feature = "fault-injection")]
    fn apply_injection(&self, out: &mut fabrail_core::msg::FastOutputs) {
        match self.inject {
            None => {}
            Some(CoreFault::IqGainX3) => out.lsm.iq_ref_a *= 3.0,
            Some(CoreFault::GapSign) => {
                if let CoilCmd::CurrentRef(r) | CoilCmd::Voltage(r) = &mut out.coil {
                    for x in r.iter_mut() {
                        *x = -*x;
                    }
                }
            }
            Some(CoreFault::Nan) => {
                out.lsm.iq_ref_a = f32::NAN;
                out.lsm.id_ref_a = f32::NAN;
                if let CoilCmd::CurrentRef(r) | CoilCmd::Voltage(r) = &mut out.coil {
                    *r = [f32::NAN; MAX_MAGNETS];
                }
            }
        }
    }
}

/// Supervisor group (1 kHz).
#[derive(Clone, Debug)]
pub struct SupervisorGroup {
    /// Core supervisor.
    pub sup: Supervisor,
    /// Rail class (telemetry).
    pub class: RailClass,
    /// Telemetry frame decimation (supervisor releases per frame, ≥ 1).
    pub telemetry_decim: u16,
    /// Supervisor releases so far.
    pub runs: u32,
    /// Telemetry frames emitted.
    pub frame_seq: u32,
    /// INSPECT resets requested so far.
    pub inspect_resets: u32,
    /// Watchdog feed rule.
    pub wd: WatchdogRule,
    /// Deadline supervision of the supervisor releases.
    pub deadline: ReleaseCheck,
}

impl SupervisorGroup {
    /// Supervisor group from the parameters.
    pub fn new(p: &SystemParams) -> Result<SupervisorGroup, ParamError> {
        Ok(SupervisorGroup {
            sup: Supervisor::new(p)?,
            class: p.design.class,
            telemetry_decim: p.control.supervisor.telemetry_decim.max(1),
            runs: 0,
            frame_seq: 0,
            inspect_resets: 0,
            wd: WatchdogRule::new(&p.control.rates),
            deadline: ReleaseCheck::new(p.control.rates.sup_decim as u64),
        })
    }

    /// One supervisor release with the core policy.
    pub fn run<H: SupervisorHal>(&mut self, t: Tick, inp: &SupIn, h: &mut H) -> SupOut {
        self.run_with_policy(t, inp, h, &mut CorePolicy)
    }

    /// One supervisor release consulting `policy` for the seismic action (simulation
    /// seam for the exotic tier and the house choreographer; plan §4).
    pub fn run_with_policy<H: SupervisorHal>(
        &mut self,
        t: Tick,
        inp: &SupIn,
        h: &mut H,
        policy: &mut dyn ActionPolicy,
    ) -> SupOut {
        let missed = self.deadline.release(t);
        let power = <H as PowerMonitor>::sample(h);
        let safety_in = <H as SafetyInputs>::sample(h);
        let cmd = <H as CommandLink>::poll(h);
        let mut verdict = inp.verdict;
        if missed {
            // Own deadline miss: the core supervisor sees it in this release's verdict
            // and enters FAULT_REACTION (SAF-013; arbitration rule 3).
            verdict.faults.set(FaultCode::SupervisorOverrun);
            verdict.demand = verdict.demand.max(FaultCode::SupervisorOverrun.demand());
        }
        let si = SupInputs {
            tick: t,
            power,
            safety_in,
            cmd,
            verdict,
        };
        let (directive, mut status) = self.sup.step_with_policy(&inp.fast, &si, &inp.seis, policy);
        if missed {
            status.faults.set(FaultCode::SupervisorOverrun);
        }
        <H as Aux>::command(h, &status.aux);
        if status.inspect_reset {
            self.inspect_resets = self.inspect_resets.wrapping_add(1);
        }
        if self.runs.is_multiple_of(self.telemetry_decim as u32) {
            self.frame_seq = self.frame_seq.wrapping_add(1);
            let f = assemble_frame(
                self.frame_seq,
                self.class,
                &inp.fast,
                &directive,
                &status,
                &inp.seis,
                &verdict,
            );
            <H as Telemetry>::frame(h, &f);
        }
        while let Some(e) = self.sup.events.pop() {
            <H as Telemetry>::event(h, &e);
        }
        let overrun = missed
            || inp.fast.faults.contains(FaultCode::FastOverrun)
            || inp.fast.faults.contains(FaultCode::SupervisorOverrun)
            || inp.verdict.faults.contains(FaultCode::FastOverrun);
        if self.wd.check(t, inp.fast.seq, inp.seis.seq, overrun) {
            <H as Watchdog>::feed(h);
        }
        self.runs = self.runs.wrapping_add(1);
        SupOut {
            directive,
            status,
            power,
            safety_in,
            inspect_resets: self.inspect_resets,
        }
    }

    /// Reports an overrun of the supervisor job (firmware DWT); raised at the next
    /// release.
    pub fn report_overrun(&mut self) {
        self.deadline.report_overrun();
    }
}

/// Seismic group (100 Hz).
#[derive(Clone, Debug)]
pub struct SeismicGroup {
    /// Core seismic unit.
    pub unit: SeismicUnit,
    /// FIFO drain buffer.
    pub buf: [AccelRaw; ACCEL_DRAIN_MAX],
    /// INSPECT reset counter last applied.
    pub last_resets: u32,
    /// Deadline supervision of the seismic releases.
    pub deadline: ReleaseCheck,
}

impl SeismicGroup {
    /// Seismic group from the parameters.
    pub fn new(p: &SystemParams) -> Result<SeismicGroup, ParamError> {
        Ok(SeismicGroup {
            unit: SeismicUnit::new(p)?,
            buf: [AccelRaw::default(); ACCEL_DRAIN_MAX],
            last_resets: 0,
            deadline: ReleaseCheck::new(p.control.rates.seismic_decim as u64),
        })
    }

    /// One seismic release at `t`.
    pub fn run<H: SeismicHal>(&mut self, t: Tick, inp: &SeisIn, h: &mut H) -> SeisOut {
        let missed = self.deadline.release(t);
        if inp.inspect_resets != self.last_resets {
            self.last_resets = inp.inspect_resets;
            self.unit.reset_inspect();
        }
        let n = <H as Accelerometer>::drain(h, &mut self.buf).min(ACCEL_DRAIN_MAX);
        let alert = <H as AlertLink>::poll(h);
        let samples = self.buf.get(..n).unwrap_or(&[]);
        let mut report = self.unit.step(t, samples, alert.as_ref(), inp.veto);
        if missed {
            report.faults.set(FaultCode::SeismicOverrun);
        }
        SeisOut { report }
    }

    /// Reports an overrun of the seismic job (firmware DWT); raised at the next release.
    pub fn report_overrun(&mut self) {
        self.deadline.report_overrun();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn release_check_detects_skips_and_reported_overruns() {
        let mut r = ReleaseCheck::new(2);
        assert!(!r.release(Tick(0)), "first release");
        assert!(!r.release(Tick(2)), "on time");
        assert!(r.release(Tick(6)), "(T) release at 4 was skipped, SAF-013");
        assert!(!r.release(Tick(8)));
        r.report_overrun();
        assert!(
            r.release(Tick(10)),
            "(T) firmware-reported overrun, SAF-013"
        );
        assert!(!r.release(Tick(12)));
        assert_eq!(r.misses, 2);
    }
}
