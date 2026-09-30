//! Trace record and ReplayBoard: per-tick HAL inputs recorded from a run and replayed through a fresh executive, bit-identically (FW-006). Owner: A7.
//!
//! [`record`] runs a scenario exactly like `run` while `SimBoard` logs every value a HAL
//! call returns ([`HalInput`], in call order, including the `&self` status calls) and
//! every command the executive issues ([`HalOutput`]). [`replay`] builds a fresh
//! `fabrail_rt::Executive` from the recorded parameters and drives it tick by tick
//! against a [`ReplayBoard`] that answers each HAL call from the recording; the outputs
//! must be bit-identical (SIL ≡ firmware, core design §A.4 item 7). Controller reboots
//! and core-fault injections recorded in the run are re-applied at their ticks
//! ([`ControlAction`]).
//!
//! Replay equality is claimed in voltage coil mode only (plan §2b item 16): that is the
//! mode in which the firmware runs the same base-task current loop.

use core::cell::Cell;

use fabrail_hal::{
    Accelerometer, AlertLink, Aux, CarriageImu, Clock, CoilDrivers, CommandLink, Encoder,
    GapSensors, LsmDriver, PowerMonitor, RailAccel, SafetyInputs, Telemetry, Watchdog,
};
use fabrail_params::SystemParams;
use fabrail_rt::Executive;
use fabrail_rt::selftest::Fnv1a;
use fabrail_types::{
    ACCEL_DRAIN_MAX, AccelRaw, AuxCmd, CoilCmd, CurrentRaw, DriverStatus, EncoderRaw, Event,
    FastRaw, GapRaw, ImuRaw, LsmCmd, LsmRaw, NetworkAlert, PowerRaw, RailAccelRaw, ResetCause,
    SafetyInRaw, TelemetryFrame, Tick, UserCommand,
};

use crate::recorder::RunData;
use crate::scenario::{CoreFaultKind, Scenario, SimError};

/// One value returned by a HAL call.
#[derive(Clone, Debug, PartialEq)]
pub enum HalInput {
    /// `CoilDrivers::sample`.
    Current(CurrentRaw),
    /// `CoilDrivers::status`.
    CoilStatus(DriverStatus),
    /// `GapSensors::sample`.
    Gap(GapRaw),
    /// `Encoder::sample`.
    Encoder(EncoderRaw),
    /// `LsmDriver::sample`.
    Lsm(LsmRaw),
    /// `LsmDriver::status`.
    LsmStatus(DriverStatus),
    /// `CarriageImu::sample`.
    Imu(ImuRaw),
    /// `RailAccel::sample`.
    Rail(RailAccelRaw),
    /// `PowerMonitor::sample`.
    Power(PowerRaw),
    /// `SafetyInputs::sample`.
    Safety(SafetyInRaw),
    /// `CommandLink::poll`.
    Command(Option<UserCommand>),
    /// `Accelerometer::drain` (the samples written).
    Accel(Vec<AccelRaw>),
    /// `AlertLink::poll`.
    Alert(Option<NetworkAlert>),
    /// `Watchdog::reset_cause`.
    ResetCause(ResetCause),
}

impl HalInput {
    /// Short name of the HAL call (diagnostics).
    pub fn name(&self) -> &'static str {
        match self {
            HalInput::Current(_) => "coil.sample",
            HalInput::CoilStatus(_) => "coil.status",
            HalInput::Gap(_) => "gap.sample",
            HalInput::Encoder(_) => "encoder.sample",
            HalInput::Lsm(_) => "lsm.sample",
            HalInput::LsmStatus(_) => "lsm.status",
            HalInput::Imu(_) => "imu.sample",
            HalInput::Rail(_) => "rail.sample",
            HalInput::Power(_) => "power.sample",
            HalInput::Safety(_) => "safety.sample",
            HalInput::Command(_) => "command.poll",
            HalInput::Accel(_) => "accel.drain",
            HalInput::Alert(_) => "alert.poll",
            HalInput::ResetCause(_) => "watchdog.reset_cause",
        }
    }
}

/// One command the executive issued through the HAL.
#[derive(Clone, Debug, PartialEq)]
pub enum HalOutput {
    /// `CoilDrivers::command`.
    Coil(CoilCmd),
    /// `LsmDriver::command`.
    Lsm(LsmCmd),
    /// `Aux::command`.
    Aux(AuxCmd),
    /// `Telemetry::frame` (the frame's wire bytes).
    Frame(Box<[u8; TelemetryFrame::SIZE]>),
    /// `Telemetry::event`.
    Event(Event),
    /// `Watchdog::feed`.
    Feed,
}

impl HalOutput {
    /// Appends a canonical byte encoding (floats by bit pattern) to `out`: two outputs
    /// are bit-identical iff their encodings are equal.
    pub fn encode(&self, out: &mut Vec<u8>) {
        match self {
            HalOutput::Coil(c) => {
                out.push(1);
                out.push(match c {
                    CoilCmd::Off => 0,
                    CoilCmd::CurrentRef(_) => 1,
                    CoilCmd::Voltage(_) => 2,
                });
                for v in c.values() {
                    out.extend_from_slice(&v.to_bits().to_le_bytes());
                }
            }
            HalOutput::Lsm(l) => {
                out.push(2);
                out.extend_from_slice(&l.iq_ref_a.to_bits().to_le_bytes());
                out.extend_from_slice(&l.id_ref_a.to_bits().to_le_bytes());
                out.push(l.mode.as_u8());
            }
            HalOutput::Aux(a) => {
                out.push(3);
                out.push(a.parking_lock as u8);
                out.push(a.seismic_latch as u8);
            }
            HalOutput::Frame(b) => {
                out.push(4);
                out.extend_from_slice(&b[..]);
            }
            HalOutput::Event(e) => {
                out.push(5);
                out.extend_from_slice(&e.tick.0.to_le_bytes());
                out.push(e.kind.as_u8());
                out.extend_from_slice(&e.a.to_bits().to_le_bytes());
                out.extend_from_slice(&e.b.to_bits().to_le_bytes());
            }
            HalOutput::Feed => out.push(6),
        }
    }

    /// True when `self` and `o` are bit-identical.
    pub fn same_bits(&self, o: &HalOutput) -> bool {
        let (mut a, mut b) = (Vec::new(), Vec::new());
        self.encode(&mut a);
        o.encode(&mut b);
        a == b
    }
}

/// FNV-1a 64 digest of an output sequence (the same hash as the golden selftest).
pub fn outputs_digest(outputs: &[HalOutput]) -> u64 {
    let mut h = Fnv1a::new();
    let mut buf = Vec::with_capacity(TelemetryFrame::SIZE + 1);
    for o in outputs {
        buf.clear();
        o.encode(&mut buf);
        h.write(&buf);
    }
    h.finish()
}

/// Executive-level actions of a run, re-applied at their tick before the base tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlAction {
    /// The controller rebooted (a fresh executive).
    Reboot,
    /// A core-output fault was injected.
    Inject(CoreFaultKind),
    /// The injected core fault was cleared.
    ClearInjection,
    /// The supervisor was warm-started into this support state (`LevState` wire value;
    /// a scenario that starts levitated, `Supervisor::warm_start`).
    WarmStart(u8),
}

/// Recorded HAL traffic of one run.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TraceRecord {
    /// The parameter set the executive was built from.
    pub params: SystemParams,
    /// Base ticks executed.
    pub ticks: u64,
    /// Every HAL return value, in call order.
    pub inputs: Vec<HalInput>,
    /// Every HAL command, in call order.
    pub outputs: Vec<HalOutput>,
    /// Reboots and core-fault injections by tick.
    pub control: Vec<(Tick, ControlAction)>,
    /// Every `FastRaw` latched for a fast release (the fast group's complete input
    /// besides its mailbox), in order.
    pub fast: Vec<FastRaw>,
}

impl TraceRecord {
    /// An empty trace.
    pub fn new() -> TraceRecord {
        TraceRecord::default()
    }

    /// Appends one fast-release sample.
    pub fn push_fast(&mut self, r: FastRaw) {
        self.fast.push(r);
    }

    /// Number of recorded fast releases.
    pub fn len(&self) -> usize {
        self.fast.len()
    }

    /// True when nothing was recorded.
    pub fn is_empty(&self) -> bool {
        self.fast.is_empty() && self.inputs.is_empty() && self.outputs.is_empty()
    }

    /// Digest of the recorded outputs.
    pub fn outputs_digest(&self) -> u64 {
        outputs_digest(&self.outputs)
    }
}

/// Maps a scenario core fault onto the rt injection.
pub(crate) fn core_fault(k: CoreFaultKind) -> fabrail_rt::CoreFault {
    match k {
        CoreFaultKind::IqGainX3 => fabrail_rt::CoreFault::IqGainX3,
        CoreFaultKind::GapSign => fabrail_rt::CoreFault::GapSign,
        CoreFaultKind::Nan => fabrail_rt::CoreFault::Nan,
    }
}

/// A board that answers every HAL call, including the `&self` status calls, from a
/// recording (a `Cell` cursor) and logs the outputs.
#[derive(Debug)]
pub struct ReplayBoard<'a> {
    inputs: &'a [HalInput],
    next: Cell<usize>,
    diverged: Cell<Option<usize>>,
    exhausted: Cell<bool>,
    /// Current base tick (set by the replay driver).
    pub tick: Tick,
    /// Outputs issued so far.
    pub outputs: Vec<HalOutput>,
}

impl<'a> ReplayBoard<'a> {
    /// A board over `inputs`.
    pub fn new(inputs: &'a [HalInput]) -> ReplayBoard<'a> {
        ReplayBoard {
            inputs,
            next: Cell::new(0),
            diverged: Cell::new(None),
            exhausted: Cell::new(false),
            tick: Tick::ZERO,
            outputs: Vec::new(),
        }
    }

    /// Inputs consumed so far.
    pub fn consumed(&self) -> usize {
        self.next.get()
    }

    /// Index of the first input whose kind did not match the call.
    pub fn diverged_at(&self) -> Option<usize> {
        self.diverged.get()
    }

    /// A call found the recording exhausted.
    pub fn exhausted(&self) -> bool {
        self.exhausted.get()
    }

    /// The next input if it is of the kind `pick` accepts; records divergence or
    /// exhaustion otherwise (and then answers with the type's default).
    fn take<T>(&self, pick: impl Fn(&HalInput) -> Option<T>) -> Option<T> {
        let i = self.next.get();
        let Some(v) = self.inputs.get(i) else {
            self.exhausted.set(true);
            return None;
        };
        match pick(v) {
            Some(t) => {
                self.next.set(i + 1);
                Some(t)
            }
            None => {
                if self.diverged.get().is_none() {
                    self.diverged.set(Some(i));
                }
                None
            }
        }
    }
}

impl Clock for ReplayBoard<'_> {
    fn now(&self) -> Tick {
        self.tick
    }
}

impl CoilDrivers for ReplayBoard<'_> {
    fn sample(&mut self) -> CurrentRaw {
        self.take(|i| match i {
            HalInput::Current(c) => Some(*c),
            _ => None,
        })
        .unwrap_or_default()
    }
    fn command(&mut self, c: &CoilCmd) {
        self.outputs.push(HalOutput::Coil(*c));
    }
    fn status(&self) -> DriverStatus {
        self.take(|i| match i {
            HalInput::CoilStatus(s) => Some(*s),
            _ => None,
        })
        .unwrap_or_default()
    }
}

impl GapSensors for ReplayBoard<'_> {
    fn sample(&mut self) -> GapRaw {
        self.take(|i| match i {
            HalInput::Gap(g) => Some(*g),
            _ => None,
        })
        .unwrap_or_default()
    }
}

impl Encoder for ReplayBoard<'_> {
    fn sample(&mut self) -> EncoderRaw {
        self.take(|i| match i {
            HalInput::Encoder(e) => Some(*e),
            _ => None,
        })
        .unwrap_or_default()
    }
}

impl LsmDriver for ReplayBoard<'_> {
    fn sample(&mut self) -> LsmRaw {
        self.take(|i| match i {
            HalInput::Lsm(l) => Some(*l),
            _ => None,
        })
        .unwrap_or_default()
    }
    fn command(&mut self, c: &LsmCmd) {
        self.outputs.push(HalOutput::Lsm(*c));
    }
    fn status(&self) -> DriverStatus {
        self.take(|i| match i {
            HalInput::LsmStatus(s) => Some(*s),
            _ => None,
        })
        .unwrap_or_default()
    }
}

impl CarriageImu for ReplayBoard<'_> {
    fn sample(&mut self) -> ImuRaw {
        self.take(|i| match i {
            HalInput::Imu(v) => Some(*v),
            _ => None,
        })
        .unwrap_or_default()
    }
}

impl RailAccel for ReplayBoard<'_> {
    fn sample(&mut self) -> RailAccelRaw {
        self.take(|i| match i {
            HalInput::Rail(v) => Some(*v),
            _ => None,
        })
        .unwrap_or_default()
    }
}

impl Accelerometer for ReplayBoard<'_> {
    fn drain(&mut self, out: &mut [AccelRaw; ACCEL_DRAIN_MAX]) -> usize {
        let n = match self.inputs.get(self.next.get()) {
            Some(HalInput::Accel(v)) => {
                for (o, s) in out.iter_mut().zip(v.iter()) {
                    *o = *s;
                }
                v.len().min(ACCEL_DRAIN_MAX)
            }
            _ => 0,
        };
        // Consume (or flag) through the common path.
        let _ = self.take(|i| matches!(i, HalInput::Accel(_)).then_some(()));
        n
    }
}

impl PowerMonitor for ReplayBoard<'_> {
    fn sample(&mut self) -> PowerRaw {
        self.take(|i| match i {
            HalInput::Power(v) => Some(*v),
            _ => None,
        })
        .unwrap_or_default()
    }
}

impl SafetyInputs for ReplayBoard<'_> {
    fn sample(&mut self) -> SafetyInRaw {
        self.take(|i| match i {
            HalInput::Safety(v) => Some(*v),
            _ => None,
        })
        .unwrap_or_default()
    }
}

impl Aux for ReplayBoard<'_> {
    fn command(&mut self, c: &AuxCmd) {
        self.outputs.push(HalOutput::Aux(*c));
    }
}

impl CommandLink for ReplayBoard<'_> {
    fn poll(&mut self) -> Option<UserCommand> {
        self.take(|i| match i {
            HalInput::Command(c) => Some(*c),
            _ => None,
        })
        .flatten()
    }
}

impl AlertLink for ReplayBoard<'_> {
    fn poll(&mut self) -> Option<NetworkAlert> {
        self.take(|i| match i {
            HalInput::Alert(a) => Some(*a),
            _ => None,
        })
        .flatten()
    }
}

impl Watchdog for ReplayBoard<'_> {
    fn feed(&mut self) {
        self.outputs.push(HalOutput::Feed);
    }
    fn reset_cause(&self) -> ResetCause {
        self.take(|i| match i {
            HalInput::ResetCause(c) => Some(*c),
            _ => None,
        })
        .unwrap_or_default()
    }
}

impl Telemetry for ReplayBoard<'_> {
    fn frame(&mut self, f: &TelemetryFrame) {
        self.outputs.push(HalOutput::Frame(Box::new(f.to_bytes())));
    }
    fn event(&mut self, e: &Event) {
        self.outputs.push(HalOutput::Event(*e));
    }
}

/// Result of a replay.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReplayOutcome {
    /// Outputs the replayed executive issued.
    pub outputs: Vec<HalOutput>,
    /// First input index whose kind did not match the executive's call.
    pub diverged_at: Option<usize>,
    /// The executive asked for more inputs than were recorded.
    pub exhausted: bool,
    /// Inputs consumed.
    pub consumed: usize,
}

impl ReplayOutcome {
    /// Index of the first output that differs from `expected` (by bits or by count).
    pub fn first_mismatch(&self, expected: &[HalOutput]) -> Option<usize> {
        let n = self.outputs.len().min(expected.len());
        (0..n)
            .find(|&i| !self.outputs[i].same_bits(&expected[i]))
            .or((self.outputs.len() != expected.len()).then_some(n))
    }

    /// Digest of the replayed outputs.
    pub fn outputs_digest(&self) -> u64 {
        outputs_digest(&self.outputs)
    }
}

/// Runs scenario `s` with parameter set `p` (exactly as [`crate::run`]) and records its
/// HAL traffic.
pub fn record(p: &SystemParams, s: &Scenario) -> Result<(RunData, TraceRecord), SimError> {
    crate::scheduler::run_traced(p, s)
}

/// Replays `trace` through a fresh executive built from `trace.params`.
pub fn replay(trace: &TraceRecord) -> Result<ReplayOutcome, SimError> {
    let mut exec = Executive::new(&trace.params)?;
    let mut board = ReplayBoard::new(&trace.inputs);
    let mut control = trace.control.iter().peekable();
    for k in 0..trace.ticks {
        let t = Tick(k);
        while let Some((_, a)) = control.next_if(|(tc, _)| *tc <= t) {
            match a {
                ControlAction::Reboot => exec = Executive::new(&trace.params)?,
                ControlAction::Inject(w) => exec.inject(core_fault(*w)),
                ControlAction::ClearInjection => exec.clear_injection(),
                ControlAction::WarmStart(l) => {
                    if let Some(lev) = fabrail_types::LevState::from_u8(*l) {
                        exec.sup.sup.warm_start(lev);
                    }
                }
            }
        }
        board.tick = t;
        exec.on_base_tick(&mut board);
    }
    Ok(ReplayOutcome {
        consumed: board.consumed(),
        diverged_at: board.diverged_at(),
        exhausted: board.exhausted(),
        outputs: board.outputs,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn replay_board_flags_divergence_and_exhaustion() {
        let inputs = vec![
            HalInput::Current(CurrentRaw { code: [3; 8] }),
            HalInput::CoilStatus(DriverStatus {
                enabled: true,
                ..DriverStatus::default()
            }),
            HalInput::Gap(GapRaw::default()),
        ];
        let mut b = ReplayBoard::new(&inputs);
        assert_eq!(CoilDrivers::sample(&mut b).code[0], 3);
        assert!(CoilDrivers::status(&b).enabled, "&self calls replay too");
        // Asking for an encoder sample where a gap sample was recorded diverges.
        assert_eq!(Encoder::sample(&mut b), EncoderRaw::default());
        assert_eq!(b.diverged_at(), Some(2));
        assert!(!b.exhausted());
        let _ = GapSensors::sample(&mut b);
        let _ = GapSensors::sample(&mut b);
        assert!(b.exhausted());
        assert_eq!(b.consumed(), 3);
    }

    #[test]
    fn output_encoding_is_bitwise() {
        let a = HalOutput::Lsm(LsmCmd {
            iq_ref_a: 0.0,
            ..LsmCmd::default()
        });
        let b = HalOutput::Lsm(LsmCmd {
            iq_ref_a: -0.0,
            ..LsmCmd::default()
        });
        assert_eq!(a, b, "PartialEq says equal");
        assert!(!a.same_bits(&b), "but the bits differ");
        let nan = HalOutput::Coil(CoilCmd::Voltage([f32::NAN; 8]));
        assert!(nan.same_bits(&nan.clone()), "NaN payloads compare by bits");
        let o = ReplayOutcome {
            outputs: vec![a.clone(), HalOutput::Feed],
            ..ReplayOutcome::default()
        };
        assert_eq!(o.first_mismatch(&[a.clone(), HalOutput::Feed]), None);
        assert_eq!(o.first_mismatch(&[b, HalOutput::Feed]), Some(0));
        assert_eq!(o.first_mismatch(&[a]), Some(1));
        assert_ne!(outputs_digest(&o.outputs), outputs_digest(&[]));
    }
}
