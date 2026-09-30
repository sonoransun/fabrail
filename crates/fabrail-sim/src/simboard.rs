//! SimBoard: every fabrail-hal trait implemented over plant truth with sensor models and command latching. Owner: A7.
//!
//! Conversions happen only here (core design §A.4 item 1): f64 truth becomes integer raw
//! codes with noise, quantisation and bandwidth (`sensors`, PHYSICS §13); f32 commands
//! become f64 plant inputs. The coil mode comes from the scenario (single source, plan
//! §2b item 16); a `CoilCmd` variant that disagrees with it is a harness error
//! ([`SimBoard::take_error`]).
//!
//! Timing within base tick k (PHYSICS §24):
//!
//! 1. [`SimBoard::begin_tick`] snapshots what the sensors measure at t_k into a delay
//!    line of `latency_ticks + 1` entries (1-tick transport latency by default), runs
//!    the gap anti-alias filters and produces the ADXL355 FIFO samples due by t_k;
//! 2. the executive's HAL calls read the delayed snapshot (noise is drawn per call) and
//!    its commands are stored;
//! 3. [`SimBoard::actuation`] latches the stored commands (sanitised, clamped to the
//!    bridge, faults applied) as the zero-order-hold input of the plant step.
//!
//! Travel1d/Structure step every 20 ticks: between plant steps the encoder position is
//! extrapolated kinematically from the last truth (x + v·τ + ½·a·τ²), so the fast group
//! sees a smooth position rather than a 1 ms staircase.
//!
//! Commands and alerts reach the links after the CAN latency `link_latency_s`
//! (3 ms, core design §C.5). The encoder is referenced from boot (the simulated
//! carriage is homed). Every HAL value and command can be logged for bit-identical
//! replay (`replay`).

use core::cell::RefCell;
use std::collections::VecDeque;

use fabrail_hal::{
    Accelerometer, AlertLink, Aux, CarriageImu, Clock, CoilDrivers, CommandLink, Encoder,
    GapSensors, LsmDriver, PowerMonitor, RailAccel, SafetyInputs, Telemetry, Watchdog,
};
use fabrail_params::{SensorCal, SystemParams};
use fabrail_plant::{CoilMode, PlantInputs, Truth};
use fabrail_types::{
    ACCEL_DRAIN_MAX, AccelRaw, AuxCmd, BASE_TICK_HZ, CoilCmd, CurrentRaw, DriverStatus, EncoderRaw,
    Event, GapRaw, ImuRaw, LsmCmd, LsmMode, LsmRaw, MAX_MAGNETS, NetworkAlert, PowerRaw,
    RailAccelRaw, ResetCause, SafetyInRaw, TelemetryFrame, Tick, UserCommand,
};

use crate::faults::FaultInjector;
use crate::replay::{HalInput, HalOutput, TraceRecord};
use crate::scenario::ControlLossDir;
use crate::sensors::{Adxl355Fifo, SensorModels, adc12_code, encoder_counts, i16_code};

/// What the sensors measure at one base tick, before transport latency and noise.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sensed {
    /// Time of the snapshot, s.
    pub t_s: f64,
    /// Anti-aliased gaps, m.
    pub gaps_m: [f64; MAX_MAGNETS],
    /// Coil currents, A.
    pub coil_i_a: [f64; MAX_MAGNETS],
    /// Travel position (extrapolated between 1 ms plant steps), m.
    pub x_m: f64,
    /// LSM phase currents, A.
    pub i_abc_a: [f64; 3],
    /// Carriage IMU specific force (inertial acceleration + tilt leakage), m/s².
    pub imu_mps2: [f64; 3],
    /// Rail floor acceleration, m/s².
    pub rail_mps2: [f64; 3],
    /// Hand force on the handle, N.
    pub f_hand_n: f64,
    /// Bus voltage, V.
    pub v_bus_v: f64,
    /// Hold-up voltage, V.
    pub v_hold_v: f64,
    /// Bus current, A.
    pub i_bus_a: f64,
    /// Mains present.
    pub mains_ok: bool,
    /// Temperatures [coil hot-spot, magnet], °C.
    pub temps_c: [f64; 2],
    /// Parking lock engaged.
    pub locked: bool,
}

/// Discrete board inputs driven by scenario events (safety-related inputs).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BoardEnv {
    /// Presence-sensing zone occupied.
    pub presence_active: bool,
    /// Presence-sensing device healthy.
    pub presence_ok: bool,
    /// Hold-to-run line actuated.
    pub hold_to_run: bool,
    /// Building fire alarm.
    pub fire_alarm: bool,
    /// Emergency stop pressed.
    pub estop: bool,
}

/// The latched actuator state for one plant step (f64, sanitised).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Actuation {
    /// Per-magnet bridge voltage (voltage mode) or current reference (lag mode).
    pub coil: [f64; MAX_MAGNETS],
    /// Coil bridges enabled.
    pub coil_enabled: bool,
    /// LSM i_q reference, A.
    pub iq_ref_a: f64,
    /// LSM i_d reference, A.
    pub id_ref_a: f64,
    /// LSM mode.
    pub lsm_mode: LsmMode,
    /// Parking lock commanded.
    pub parking_lock: bool,
    /// Seismic latch commanded.
    pub seismic_latch: bool,
}

/// The simulated board for one device.
#[derive(Clone, Debug)]
pub struct SimBoard {
    /// Current base tick.
    pub tick: Tick,
    /// Truth snapshot the sensors sample (the last plant step's end state).
    pub truth: Truth,
    /// Latched plant inputs of the last plant step (set by the harness; introspection).
    pub inputs: PlantInputs,
    /// Noise streams and filter states.
    pub sensors: SensorModels,
    /// Active sensor/driver faults.
    pub faults: FaultInjector,
    /// Scenario-driven discrete inputs.
    pub env: BoardEnv,
    /// Reset cause the watchdog reports (set on a simulated reboot).
    pub reset_cause: ResetCause,
    cal: SensorCal,
    n_magnets: usize,
    coil_mode: CoilMode,
    v_bus_nominal_v: f64,
    i_pulse_a: f64,
    t_coil_max_c: f64,
    v_uvlo_v: f64,
    latency: usize,
    delay: VecDeque<Sensed>,
    coil_cmd: CoilCmd,
    lsm_cmd: LsmCmd,
    aux_cmd: AuxCmd,
    cmd_q: VecDeque<(u64, UserCommand)>,
    alert_q: VecDeque<(u64, NetworkAlert)>,
    fifo: Adxl355Fifo,
    accel_period_ticks: f64,
    accel_n: u64,
    feeds: u64,
    frames: u64,
    last_frame: Option<TelemetryFrame>,
    events: Vec<Event>,
    nonfinite_cmds: u64,
    error: Option<String>,
    tracing: bool,
    trace: RefCell<TraceRecord>,
}

impl SimBoard {
    /// A board for device `device` of a run seeded with `noise_seed`, with the default
    /// (all-zero) calibration: every code reads 0 until [`SimBoard::with_params`] is
    /// used. Kept for callers that only need the HAL plumbing.
    pub fn new(noise_seed: u64, device: u16) -> SimBoard {
        SimBoard::with_params(&SystemParams::default(), CoilMode::Lag, noise_seed, device)
    }

    /// A board for parameter set `p` in coil mode `coil` (the scenario's, single
    /// source) for device `device` of a run seeded with `noise_seed`.
    pub fn with_params(p: &SystemParams, coil: CoilMode, noise_seed: u64, device: u16) -> SimBoard {
        let cal = p.design.sensors;
        let odr = cal.accel_odr_hz;
        let accel_period_ticks = if odr.is_finite() && odr > 0.0 {
            BASE_TICK_HZ as f64 / odr
        } else {
            f64::INFINITY
        };
        SimBoard {
            tick: Tick::ZERO,
            truth: Truth::default(),
            inputs: PlantInputs::default(),
            sensors: SensorModels::new(noise_seed, device),
            faults: FaultInjector::new(noise_seed, device),
            env: BoardEnv {
                presence_ok: p.design.presence == fabrail_types::PresenceSensing::PlD,
                ..BoardEnv::default()
            },
            reset_cause: ResetCause::PowerOn,
            cal,
            n_magnets: (p.design.suspension.n_magnets as usize).min(MAX_MAGNETS),
            coil_mode: coil,
            v_bus_nominal_v: p.design.power.v_bus_v,
            i_pulse_a: p.design.suspension.i_pulse_a,
            t_coil_max_c: p.design.thermal.t_coil_max_c,
            v_uvlo_v: p.design.power.supercap_v_min_v,
            latency: cal.latency_ticks as usize,
            delay: VecDeque::with_capacity(cal.latency_ticks as usize + 2),
            coil_cmd: CoilCmd::Off,
            lsm_cmd: LsmCmd::default(),
            aux_cmd: AuxCmd::default(),
            cmd_q: VecDeque::new(),
            alert_q: VecDeque::new(),
            fifo: Adxl355Fifo::new(),
            accel_period_ticks,
            accel_n: 0,
            feeds: 0,
            frames: 0,
            last_frame: None,
            events: Vec::new(),
            nonfinite_cmds: 0,
            error: None,
            tracing: false,
            trace: RefCell::new(TraceRecord::default()),
        }
    }

    /// Sets the initial parking-lock state (the lock stays as it was until the
    /// supervisor's first aux command).
    pub fn set_initial_aux(&mut self, aux: AuxCmd) {
        self.aux_cmd = aux;
    }

    /// Starts logging every HAL value and command for replay.
    pub fn enable_trace(&mut self) {
        self.tracing = true;
    }

    /// Takes the logged HAL traffic (inputs and outputs only).
    pub fn take_trace(&mut self) -> TraceRecord {
        core::mem::take(self.trace.get_mut())
    }

    #[inline]
    fn trace_in(&self, v: HalInput) {
        if self.tracing {
            self.trace.borrow_mut().inputs.push(v);
        }
    }

    #[inline]
    fn trace_out(&self, v: HalOutput) {
        if self.tracing {
            self.trace.borrow_mut().outputs.push(v);
        }
    }

    /// Snapshots the sensed state at base tick `k` from `truth` (see the module docs).
    /// `floor_mps2` is the rail floor acceleration at t_k and `structure_mps2` what the
    /// structure ADXL355 feels (floor + site noise).
    pub fn begin_tick(
        &mut self,
        k: Tick,
        truth: &Truth,
        floor_mps2: [f64; 3],
        structure_mps2: [f64; 3],
    ) {
        self.tick = k;
        self.truth = *truth;
        let t = k.seconds_f64();
        // Kinematic extrapolation only when the truth is at least half a base tick old
        // (the 1 ms tiers between plant steps); the 50 µs tiers' truth is at t_k up to
        // rounding and is used as is.
        let tau = t - truth.t_s;
        let x_m = if tau > 0.5 * fabrail_types::BASE_TICK_S {
            truth.x_m + truth.v_mps * tau + 0.5 * truth.a_mps2 * tau * tau
        } else {
            truth.x_m
        };
        let gaps_m = self.sensors.filter_gaps(&truth.gaps_m);
        // Tilt leakage: the IMU measures R^T·(0, 0, g) − (0, 0, g) on top of the
        // gravity-free inertial acceleration (third row of the body→world rotation of
        // the unit quaternion [w, x, y, z]; plan §4b "g·θ_tilt, 1 mg per mrad";
        // PHYSICS §13). IMU axes are aligned with the carriage frame.
        let [_, _, _, qw, qx, qy, qz] = truth.pose;
        let g = fabrail_params::derive::G0_MPS2;
        let leak = [
            g * 2.0 * (qx * qz - qw * qy),
            g * 2.0 * (qy * qz + qw * qx),
            -g * 2.0 * (qx * qx + qy * qy),
        ];
        let a = truth.acc_abs_mps2;
        let s = Sensed {
            t_s: t,
            gaps_m,
            coil_i_a: truth.coil_i_a,
            x_m,
            i_abc_a: truth.i_abc_a,
            imu_mps2: [a[0] + leak[0], a[1] + leak[1], a[2] + leak[2]],
            rail_mps2: floor_mps2,
            f_hand_n: truth.f_hand_n,
            v_bus_v: truth.power.v_bus_v,
            v_hold_v: truth.power.v_hold_v,
            i_bus_a: truth.power.i_bus_a,
            mains_ok: truth.power.mains_ok,
            temps_c: truth.temps_c,
            locked: truth.state.locked,
        };
        self.delay.push_back(s);
        while self.delay.len() > self.latency + 1 {
            self.delay.pop_front();
        }
        // ADXL355 samples at the ODR: sample n is due at tick n·(base/ODR).
        while (self.accel_n as f64) * self.accel_period_ticks <= k.0 as f64 {
            let codes = self.sensors.adxl355_codes(&self.cal, &structure_mps2);
            let codes = self.faults.accel(codes);
            if !self.faults.accel_lost {
                self.fifo.push(codes);
            }
            self.accel_n += 1;
        }
    }

    /// The (latency-delayed) sensed state the HAL samples this tick.
    pub fn sensed(&self) -> Sensed {
        self.delay.front().copied().unwrap_or_default()
    }

    /// Queues a user command that reaches the command link at tick `due`.
    pub fn enqueue_command(&mut self, due: Tick, c: UserCommand) {
        self.cmd_q.push_back((due.0, c));
    }

    /// Queues an alert (or cancellation) that reaches the alert link at tick `due`.
    pub fn enqueue_alert(&mut self, due: Tick, a: NetworkAlert) {
        self.alert_q.push_back((due.0, a));
    }

    /// Controller events emitted since the last call.
    pub fn take_events(&mut self) -> Vec<Event> {
        core::mem::take(&mut self.events)
    }

    /// True when controller events are pending.
    pub fn has_events(&self) -> bool {
        !self.events.is_empty()
    }

    /// A harness error raised by a HAL call (e.g. a coil command in the wrong mode).
    pub fn take_error(&mut self) -> Option<String> {
        self.error.take()
    }

    /// Watchdog feeds so far.
    pub fn feeds(&self) -> u64 {
        self.feeds
    }

    /// Telemetry frames emitted so far.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// The last telemetry frame.
    pub fn last_frame(&self) -> Option<&TelemetryFrame> {
        self.last_frame.as_ref()
    }

    /// Non-finite command values replaced by 0 so far (a core output the monitor did
    /// not catch; hardware PWM conversion saturates NaN to a zero duty).
    pub fn nonfinite_commands(&self) -> u64 {
        self.nonfinite_cmds
    }

    /// Samples queued in the ADXL355 FIFO.
    pub fn accel_queued(&self) -> usize {
        self.fifo.len()
    }

    /// The commands last issued by the executive.
    pub fn commands(&self) -> (CoilCmd, LsmCmd, AuxCmd) {
        (self.coil_cmd, self.lsm_cmd, self.aux_cmd)
    }

    fn bus_v(&self) -> f64 {
        let v = self.truth.power.v_bus_v;
        if v > 0.0 { v } else { self.v_bus_nominal_v }
    }

    fn sanitize(&mut self, v: f32) -> f64 {
        if v.is_finite() {
            v as f64
        } else {
            self.nonfinite_cmds += 1;
            0.0
        }
    }

    /// Latches the issued commands as the plant-step input (see the module docs): coil
    /// values sanitised (NaN → 0) and, in voltage mode, clamped to ±V_bus (an H-bridge
    /// cannot exceed its DC link); faulted bridges give zero drive; a `control_loss`
    /// holds every bridge at full output (±V_bus in voltage mode, ±i_pulse in lag
    /// mode); a faulted LSM inverter is in STO.
    pub fn actuation(&mut self) -> Actuation {
        let voltage = self.coil_mode == CoilMode::Voltage;
        let v_bus = self.bus_v();
        let (raw, mut enabled) = match self.coil_cmd {
            CoilCmd::Off => ([0.0f32; MAX_MAGNETS], false),
            CoilCmd::CurrentRef(r) | CoilCmd::Voltage(r) => (r, true),
        };
        let mut coil = [0.0f64; MAX_MAGNETS];
        for ch in 0..MAX_MAGNETS {
            let mut v = self.sanitize(raw[ch]);
            if voltage {
                v = v.clamp(-v_bus, v_bus);
            }
            if self.faults.coil_off(ch) {
                v = 0.0;
            }
            coil[ch] = v;
        }
        if let Some(dir) = self.faults.control_loss {
            let full = if voltage { v_bus } else { self.i_pulse_a };
            let v = match dir {
                ControlLossDir::Up => full,
                ControlLossDir::Down => -full,
            };
            for c in coil.iter_mut().take(self.n_magnets) {
                *c = v;
            }
            enabled = true;
        }
        let lsm = self.lsm_cmd;
        let iq = self.sanitize(lsm.iq_ref_a);
        let id = self.sanitize(lsm.id_ref_a);
        Actuation {
            coil,
            coil_enabled: enabled,
            iq_ref_a: iq,
            id_ref_a: id,
            lsm_mode: if self.faults.lsm_driver {
                LsmMode::Sto
            } else {
                lsm.mode
            },
            parking_lock: self.aux_cmd.parking_lock,
            seismic_latch: self.aux_cmd.seismic_latch,
        }
    }

    fn driver_common(&self) -> (bool, bool) {
        let s = self.sensed();
        (s.temps_c[0] > self.t_coil_max_c, s.v_bus_v <= self.v_uvlo_v)
    }
}

impl Clock for SimBoard {
    fn now(&self) -> Tick {
        self.tick
    }
}

impl CoilDrivers for SimBoard {
    fn sample(&mut self) -> CurrentRaw {
        let s = self.sensed();
        let mut code = [0i16; MAX_MAGNETS];
        for (ch, c) in code.iter_mut().enumerate().take(self.n_magnets) {
            *c = self.sensors.coil_code(&self.cal, ch, s.coil_i_a[ch]);
        }
        let v = CurrentRaw { code };
        self.trace_in(HalInput::Current(v));
        v
    }

    fn command(&mut self, c: &CoilCmd) {
        let bad = match (self.coil_mode, c) {
            (CoilMode::Voltage, CoilCmd::CurrentRef(_)) => Some("current_ref in voltage mode"),
            (CoilMode::Lag, CoilCmd::Voltage(_)) => Some("voltage in lag mode"),
            _ => None,
        };
        if let Some(why) = bad
            && self.error.is_none()
        {
            self.error = Some(format!(
                "SimBoard: coil command {why} at tick {} (the scenario is the single source of \
                 the coil mode, plan §2b item 16)",
                self.tick.0
            ));
        }
        self.coil_cmd = *c;
        self.trace_out(HalOutput::Coil(*c));
    }

    fn status(&self) -> DriverStatus {
        let (overtemp, undervoltage) = self.driver_common();
        let s = DriverStatus {
            enabled: !matches!(self.coil_cmd, CoilCmd::Off),
            fault: self.faults.coil_driver_mask != 0,
            overcurrent: false,
            overtemp,
            undervoltage,
        };
        self.trace_in(HalInput::CoilStatus(s));
        s
    }
}

impl GapSensors for SimBoard {
    fn sample(&mut self) -> GapRaw {
        let s = self.sensed();
        let mut raw = GapRaw::default();
        for ch in 0..self.n_magnets {
            let (a, b) = self
                .sensors
                .gap_codes(&self.cal, ch, s.gaps_m[ch], s.temps_c[1]);
            let (a, ok) = self.faults.gap(ch, a);
            raw.code[ch] = a;
            raw.code_b[ch] = b;
            if ok {
                raw.valid |= 1u8 << ch;
            }
        }
        self.trace_in(HalInput::Gap(raw));
        raw
    }
}

impl Encoder for SimBoard {
    fn sample(&mut self) -> EncoderRaw {
        let s = self.sensed();
        let (counts, err) = self
            .faults
            .encoder(encoder_counts(s.x_m, self.cal.nm_per_count));
        let v = EncoderRaw {
            counts,
            referenced: true,
            err,
        };
        self.trace_in(HalInput::Encoder(v));
        v
    }
}

impl LsmDriver for SimBoard {
    fn sample(&mut self) -> LsmRaw {
        let s = self.sensed();
        let v = LsmRaw {
            i_abc_code: self.sensors.lsm_codes(&self.cal, &s.i_abc_a),
            fault: self.faults.lsm_driver,
        };
        self.trace_in(HalInput::Lsm(v));
        v
    }

    fn command(&mut self, c: &LsmCmd) {
        self.lsm_cmd = *c;
        self.trace_out(HalOutput::Lsm(*c));
    }

    fn status(&self) -> DriverStatus {
        let (_, undervoltage) = self.driver_common();
        let s = DriverStatus {
            enabled: self.lsm_cmd.mode != LsmMode::Sto && !self.faults.lsm_driver,
            fault: self.faults.lsm_driver,
            overcurrent: false,
            overtemp: false,
            undervoltage,
        };
        self.trace_in(HalInput::LsmStatus(s));
        s
    }
}

impl CarriageImu for SimBoard {
    fn sample(&mut self) -> ImuRaw {
        let present = self.cal.imu_present && !self.faults.imu_lost;
        let v = if present {
            let s = self.sensed();
            ImuRaw {
                xyz: self.sensors.imu_codes(&self.cal, &s.imu_mps2),
                present,
            }
        } else {
            ImuRaw::default()
        };
        self.trace_in(HalInput::Imu(v));
        v
    }
}

impl RailAccel for SimBoard {
    fn sample(&mut self) -> RailAccelRaw {
        let present = self.cal.rail_accel_present && !self.faults.rail_accel_lost;
        let v = if present {
            let s = self.sensed();
            RailAccelRaw {
                xyz: self.sensors.rail_codes(&self.cal, &s.rail_mps2),
                present,
            }
        } else {
            RailAccelRaw::default()
        };
        self.trace_in(HalInput::Rail(v));
        v
    }
}

impl Accelerometer for SimBoard {
    fn drain(&mut self, out: &mut [AccelRaw; ACCEL_DRAIN_MAX]) -> usize {
        let n = if self.faults.accel_lost {
            0
        } else {
            self.fifo.drain(out)
        };
        if self.tracing {
            self.trace_in(HalInput::Accel(out[..n].to_vec()));
        }
        n
    }
}

impl PowerMonitor for SimBoard {
    fn sample(&mut self) -> PowerRaw {
        let s = self.sensed();
        let c = &self.cal;
        let t = |x: f64| adc12_code(x - c.temp_offset_c, c.temp_c_per_code);
        let v = PowerRaw {
            v_bus_code: adc12_code(s.v_bus_v, c.v_bus_v_per_code),
            v_hold_code: adc12_code(s.v_hold_v, c.v_hold_v_per_code),
            i_bus_code: i16_code(s.i_bus_a, c.i_bus_a_per_code),
            mains_ok: s.mains_ok,
            temp_code: [t(s.temps_c[0]), t(s.temps_c[1])],
        };
        self.trace_in(HalInput::Power(v));
        v
    }
}

impl SafetyInputs for SimBoard {
    fn sample(&mut self) -> SafetyInRaw {
        let s = self.sensed();
        let e = self.env;
        let v = SafetyInRaw {
            presence_ok: e.presence_ok,
            presence_active: e.presence_active,
            handle_force_code: self.sensors.handle_code(&self.cal, s.f_hand_n),
            has_handle_cell: self.cal.handle_n_per_code > 0.0,
            fire_alarm: e.fire_alarm,
            estop: e.estop,
            lock_engaged: s.locked,
            hold_to_run: e.hold_to_run,
        };
        self.trace_in(HalInput::Safety(v));
        v
    }
}

impl Aux for SimBoard {
    fn command(&mut self, c: &AuxCmd) {
        self.aux_cmd = *c;
        self.trace_out(HalOutput::Aux(*c));
    }
}

impl CommandLink for SimBoard {
    fn poll(&mut self) -> Option<UserCommand> {
        let v = match self.cmd_q.front() {
            Some(&(due, c)) if due <= self.tick.0 => {
                self.cmd_q.pop_front();
                Some(c)
            }
            _ => None,
        };
        self.trace_in(HalInput::Command(v));
        v
    }
}

impl AlertLink for SimBoard {
    fn poll(&mut self) -> Option<NetworkAlert> {
        let v = match self.alert_q.front() {
            Some(&(due, a)) if due <= self.tick.0 => {
                self.alert_q.pop_front();
                Some(a)
            }
            _ => None,
        };
        self.trace_in(HalInput::Alert(v));
        v
    }
}

impl Watchdog for SimBoard {
    fn feed(&mut self) {
        self.feeds += 1;
        self.trace_out(HalOutput::Feed);
    }

    fn reset_cause(&self) -> ResetCause {
        self.trace_in(HalInput::ResetCause(self.reset_cause));
        self.reset_cause
    }
}

impl Telemetry for SimBoard {
    fn frame(&mut self, f: &TelemetryFrame) {
        self.frames += 1;
        self.last_frame = Some(*f);
        if self.tracing {
            self.trace_out(HalOutput::Frame(Box::new(f.to_bytes())));
        }
    }

    fn event(&mut self, e: &Event) {
        self.events.push(*e);
        self.trace_out(HalOutput::Event(*e));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_plant::InitLev;
    use fabrail_plant::params::{Environment, Mismatch, PlantParams};

    fn board(seed: u64) -> (SimBoard, Truth) {
        let p = fabrail_params::presets::door();
        let pp = PlantParams::from_design(&p.design, &Mismatch::default(), &Environment::default());
        let init = fabrail_plant::InitialState {
            lev: InitLev::Levitated,
            x_m: 0.1,
            ..Default::default()
        };
        let t = Truth::initial(&pp, &init);
        (SimBoard::with_params(&p, CoilMode::Voltage, seed, 0), t)
    }

    #[test]
    fn sensors_quantise_truth_with_one_tick_latency() {
        let (mut b, mut t) = board(3);
        b.begin_tick(Tick(0), &t, [0.0; 3], [0.0; 3]);
        let e0 = Encoder::sample(&mut b);
        assert_eq!(e0.counts, 100_000, "0.1 m at 1 µm/count");
        t.x_m = 0.2;
        t.t_s = 5.0e-5;
        b.begin_tick(Tick(1), &t, [0.0; 3], [0.0; 3]);
        assert_eq!(Encoder::sample(&mut b).counts, 100_000, "1-tick latency");
        b.begin_tick(Tick(2), &t, [0.0; 3], [0.0; 3]);
        assert_eq!(Encoder::sample(&mut b).counts, 200_000);
        let g = GapSensors::sample(&mut b);
        let n = b.n_magnets;
        assert_eq!(g.valid, ((1u16 << n) - 1) as u8);
        let lsb = b.cal.gap_poly_m[1];
        let want = t.gaps_m[0] / lsb;
        assert!(
            (g.code[0] as f64 - want).abs() < 20.0 * b.cal.gap_noise_m / lsb + 1.0,
            "(R) gap code within 20 σ of truth, PHYSICS §13"
        );
    }

    #[test]
    fn same_seed_same_samples_different_seed_differs() {
        let run = |seed| {
            let (mut b, t) = board(seed);
            let mut v = Vec::new();
            for k in 0..50u64 {
                b.begin_tick(Tick(k), &t, [0.1, 0.0, 0.0], [0.1, 0.0, 0.0]);
                v.push(GapSensors::sample(&mut b));
            }
            v
        };
        assert_eq!(run(5), run(5));
        assert_ne!(run(5), run(6));
    }

    #[test]
    fn accel_fifo_runs_at_the_odr_and_links_are_delayed() {
        let (mut b, t) = board(1);
        let mut out = [AccelRaw::default(); ACCEL_DRAIN_MAX];
        let mut total = 0;
        b.enqueue_command(Tick(60), UserCommand::Open);
        let mut got = None;
        for k in 0..=400u64 {
            b.begin_tick(Tick(k), &t, [0.0; 3], [0.0, 0.0, 0.5]);
            if k % 200 == 0 {
                total += b.drain(&mut out);
            }
            if got.is_none()
                && let Some(c) = CommandLink::poll(&mut b)
            {
                got = Some((k, c));
            }
        }
        // 500 Hz over 0…20 ms inclusive: samples at ticks 0, 40, …, 400 → 11.
        assert_eq!(total, 11, "ADXL355 at the 500 Hz ODR");
        assert_eq!(got, Some((60, UserCommand::Open)));
        let z = out[0].xyz[2] as f64 * b.cal.accel_mps2_per_lsb;
        assert!(
            (z - 0.5).abs() < 0.02,
            "(R) 0.5 m/s² through noise and quantisation"
        );
    }

    #[test]
    fn actuation_sanitises_and_applies_faults() {
        let (mut b, t) = board(1);
        b.begin_tick(Tick(0), &t, [0.0; 3], [0.0; 3]);
        let mut v = [0.0f32; MAX_MAGNETS];
        v[0] = f32::NAN;
        v[1] = 1.0e6;
        v[2] = -3.0;
        CoilDrivers::command(&mut b, &CoilCmd::Voltage(v));
        let a = b.actuation();
        assert!(a.coil_enabled);
        assert_eq!(a.coil[0], 0.0);
        assert_eq!(a.coil[1], b.bus_v(), "clamped to the DC link");
        assert_eq!(a.coil[2], -3.0);
        assert_eq!(b.nonfinite_commands(), 1);
        assert!(b.take_error().is_none());
        b.faults
            .activate(crate::scenario::SensorFaultKind::CoilDriver, 2, 0);
        assert_eq!(b.actuation().coil[2], 0.0);
        b.faults.control_loss = Some(ControlLossDir::Down);
        let a = b.actuation();
        assert_eq!(a.coil[3], -b.bus_v());
        CoilDrivers::command(&mut b, &CoilCmd::CurrentRef(v));
        assert!(
            b.take_error().is_some(),
            "current_ref in voltage mode is a harness error"
        );
    }
}
