//! `fabrail-hal` trait implementations over the BSP drivers (core design §F "HAL
//! implementation and BSP"). Safe code only: register access lives in `bsp/`.
//!
//! Each board is owned by exactly one RTIC task, so the `fabrail-rt` groups call it
//! without locks:
//!
//! | Board | Task | Traits (`fabrail_hal`) |
//! |---|---|---|
//! | [`BaseBoard`] | `base` (ADC1_2, 20 kHz) | `CoilDrivers`, `LsmDriver`, `GapSensors`, `Encoder`, `CarriageImu`, `RailAccel` = `BaseHal` |
//! | [`SupBoard`] | `supervisor` (1 kHz) | `PowerMonitor`, `SafetyInputs`, `Aux`, `CommandLink`, `Watchdog`, `Telemetry` = `SupervisorHal` |
//! | [`SeisBoard`] | `seismic` (100 Hz) | `Accelerometer`, `AlertLink` = `SeismicHal` |
//!
//! The boards convert only units the HAL contract fixes (volts → PWM duty, codes → amps
//! for the inverter's d/q current PI, which is `fabrail_core::foc::DqPi`, the core's own
//! code): scaling constants come from `fabrail_params`, and every decision stays in the
//! shared crates.

use fabrail_core::foc::DqPi;
use fabrail_hal::{
    Accelerometer, AlertLink, Aux, CarriageImu, CoilDrivers, CommandLink, Encoder, GapSensors,
    LsmDriver, PowerMonitor, RailAccel, SafetyInputs, Telemetry, Watchdog,
};
use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{
    ACCEL_DRAIN_MAX, AccelRaw, AuxCmd, CoilCmd, CurrentRaw, DriverStatus, EncoderRaw, Event,
    GapRaw, ImuRaw, LsmCmd, LsmMode, LsmRaw, MAX_MAGNETS, NetworkAlert, PosNm, PowerRaw,
    RailAccelRaw, ResetCause, SafetyInRaw, TelemetryFrame, UserCommand,
};

use crate::bsp::accel_spi::InertialSpi;
use crate::bsp::adc::{self, Adc, Currents};
use crate::bsp::encoder::Encoder as Tim2Encoder;
use crate::bsp::hrtim::Hrtim;
use crate::bsp::pins;
use crate::bsp::tim1::{self, Tim1};
use crate::bsp::watchdog::Watchdogs;

/// Fixed-capacity FIFO of `Copy` values (no allocation, no panics). Full queues drop
/// the newest value and count it.
#[derive(Clone, Copy, Debug)]
pub struct Ring<T: Copy, const N: usize> {
    buf: [Option<T>; N],
    head: usize,
    len: usize,
    /// Values dropped because the queue was full.
    pub dropped: u32,
}

impl<T: Copy, const N: usize> Ring<T, N> {
    /// An empty queue.
    pub const fn new() -> Self {
        Ring {
            buf: [None; N],
            head: 0,
            len: 0,
            dropped: 0,
        }
    }

    /// Appends `v`; false (and counted) when full.
    pub fn push(&mut self, v: T) -> bool {
        if self.len >= N || N == 0 {
            self.dropped = self.dropped.wrapping_add(1);
            return false;
        }
        let at = (self.head + self.len) % N;
        if let Some(slot) = self.buf.get_mut(at) {
            *slot = Some(v);
            self.len += 1;
            true
        } else {
            false
        }
    }

    /// Removes the oldest value.
    pub fn pop(&mut self) -> Option<T> {
        if self.len == 0 || N == 0 {
            return None;
        }
        let v = self.buf.get_mut(self.head).and_then(Option::take);
        self.head = (self.head + 1) % N;
        self.len -= 1;
        v
    }

    /// Queued values.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Nothing queued.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<T: Copy, const N: usize> Default for Ring<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Telemetry handed from the supervisor group to the telemetry task: the latest frame
/// (the CAN link carries 100 Hz of the 1 kHz frames; the recorder in SIL keeps all) and
/// every event.
#[derive(Clone, Copy, Debug, Default)]
pub struct TeleQueue {
    /// Most recent supervisor frame not yet sent.
    pub latest: Option<TelemetryFrame>,
    /// Frames produced since boot.
    pub frames: u32,
    /// Events not yet sent.
    pub events: Ring<Event, 64>,
}

/// Constants the base board needs for register work, cast once from the parameters.
#[derive(Clone, Copy, Debug)]
pub struct BaseCal {
    /// Coils fitted.
    pub n_magnets: u8,
    /// Nominal bus voltage for volts → duty (the base task's PI uses the same value).
    pub v_bus_v: f32,
    /// LSM phase current scale, A/code.
    pub lsm_a_per_code: f32,
    /// Encoder resolution, nm/count.
    pub nm_per_count: i64,
    /// Redundant gap probes fitted.
    pub gap_redundant: bool,
}

impl BaseCal {
    /// From the parameter set (f64 → f32 casts happen here, as in every `new()`).
    pub fn new(p: &SystemParams) -> BaseCal {
        let s = &p.design.sensors;
        BaseCal {
            n_magnets: p.design.suspension.n_magnets.min(MAX_MAGNETS as u8),
            v_bus_v: p.design.power.v_bus_v as f32,
            lsm_a_per_code: s.lsm_a_per_code as f32,
            nm_per_count: i64::from(s.nm_per_count),
            gap_redundant: s.gap_redundant,
        }
    }
}

/// Everything the 20 kHz base task touches.
pub struct BaseBoard {
    tim1: Tim1,
    hrtim: Hrtim,
    adc: Adc,
    enc: Tim2Encoder,
    inertial: InertialSpi,
    dq: DqPi,
    cal: BaseCal,
    cur: Currents,
    /// `CoilCmd::CurrentRef` commands refused (lag coil mode is simulation-only; boot
    /// already rejects it, so this stays 0).
    pub lag_mode_refusals: u32,
}

impl BaseBoard {
    /// Assembles the base board; the LSM d/q PI is designed from the parameters.
    pub fn new(
        tim1: Tim1,
        hrtim: Hrtim,
        adc: Adc,
        enc: Tim2Encoder,
        inertial: InertialSpi,
        p: &SystemParams,
    ) -> Result<BaseBoard, ParamError> {
        Ok(BaseBoard {
            tim1,
            hrtim,
            adc,
            enc,
            inertial,
            dq: DqPi::new(p)?,
            cal: BaseCal::new(p),
            cur: Currents::default(),
            lag_mode_refusals: 0,
        })
    }

    /// Start of a base tick: acknowledge the ADC interrupt, latch this PWM period's
    /// currents and extend the encoder count. Every HAL sample of the tick reads these
    /// latched values.
    #[inline]
    pub fn begin_tick(&mut self) {
        self.adc.ack_injected();
        self.cur = self.adc.currents();
        self.enc.update();
    }

    /// Arms the PWM-synchronous injected conversions (starts the base interrupt).
    pub fn arm_adc(&mut self) {
        self.adc.arm_injected();
    }

    /// Fast-release poll of the SPI inertial sensors.
    #[inline]
    pub fn poll_inertial(&mut self) {
        self.inertial.poll();
    }

    fn n(&self) -> usize {
        usize::from(self.cal.n_magnets)
    }
}

/// Locked anti-phase bridge: V = V_bus·(2d − 1) ⇒ d = 1/2 + v/(2·V_bus).
#[inline]
fn bridge_duty(v: f32, v_bus: f32) -> f32 {
    if v_bus > 0.0 {
        0.5 + 0.5 * v / v_bus
    } else {
        0.5
    }
}

impl CoilDrivers for BaseBoard {
    fn sample(&mut self) -> CurrentRaw {
        let mut code = [0i16; MAX_MAGNETS];
        for (dst, src) in code.iter_mut().zip(self.cur.coil).take(self.n()) {
            *dst = src;
        }
        CurrentRaw { code }
    }

    fn command(&mut self, c: &CoilCmd) {
        match c {
            CoilCmd::Off => self.hrtim.disable(),
            CoilCmd::Voltage(v) => {
                for (i, volts) in v.iter().enumerate().take(self.n()) {
                    self.hrtim
                        .set_duty(i, bridge_duty(*volts, self.cal.v_bus_v));
                }
                // A latched hardware fault keeps the bridges off; the status reports it.
                let _ = self.hrtim.enable();
            }
            CoilCmd::CurrentRef(_) => {
                self.lag_mode_refusals = self.lag_mode_refusals.wrapping_add(1);
                self.hrtim.disable();
            }
        }
    }

    fn status(&self) -> DriverStatus {
        let hw = self.hrtim.fault_latched();
        DriverStatus {
            enabled: self.hrtim.enabled(),
            fault: hw || !pins::COIL_NFAULT.is_high(),
            overcurrent: hw,
            overtemp: false,
            undervoltage: false,
        }
    }
}

impl LsmDriver for BaseBoard {
    fn sample(&mut self) -> LsmRaw {
        LsmRaw {
            i_abc_code: self.cur.phase,
            fault: self.tim1.break_latched() || !pins::LSM_NFAULT.is_high(),
        }
    }

    fn command(&mut self, c: &LsmCmd) {
        match c.mode {
            LsmMode::Sto => {
                self.tim1.sto();
                self.dq.reset();
            }
            LsmMode::Asc => {
                self.dq.reset();
                let _ = self.tim1.asc();
            }
            LsmMode::Run => {
                let k = self.cal.lsm_a_per_code;
                let i_abc = self.cur.phase.map(|code| f32::from(code) * k);
                let x = PosNm(self.enc.counts().saturating_mul(self.cal.nm_per_count));
                let v_bus = self.cal.v_bus_v;
                let v = self.dq.step(c.iq_ref_a, c.id_ref_a, i_abc, x, v_bus);
                // Min–max zero-sequence injection (the SVPWM-equivalent carrier offset):
                // the phase voltages are centred in the bus, so the linear range is
                // V_bus/√3, matching DqPi's limit.
                let hi = v[0].max(v[1]).max(v[2]);
                let lo = v[0].min(v[1]).min(v[2]);
                let off = -0.5 * (hi + lo);
                let ccr = v.map(|vk| {
                    let d = if v_bus > 0.0 {
                        0.5 + (vk + off) / v_bus
                    } else {
                        0.5
                    };
                    let d = if d.is_finite() {
                        d.clamp(0.0, 1.0)
                    } else {
                        0.5
                    };
                    (d * tim1::ARR as f32) as u32
                });
                self.tim1.set_compare(ccr);
                let _ = self.tim1.run();
            }
        }
    }

    fn status(&self) -> DriverStatus {
        let hw = self.tim1.break_latched();
        DriverStatus {
            enabled: self.tim1.outputs_enabled(),
            fault: hw || !pins::LSM_NFAULT.is_high(),
            overcurrent: hw,
            overtemp: false,
            undervoltage: false,
        }
    }
}

impl GapSensors for BaseBoard {
    fn sample(&mut self) -> GapRaw {
        let (a, b) = adc::gap_codes();
        let n = self.n();
        let mut raw = GapRaw::default();
        for (i, (ca, cb)) in a.iter().zip(b.iter()).enumerate().take(n) {
            if let Some(d) = raw.code.get_mut(i) {
                *d = *ca;
            }
            if self.cal.gap_redundant
                && let Some(d) = raw.code_b.get_mut(i)
            {
                *d = *cb;
            }
        }
        raw.valid = ((1u16 << n) - 1) as u8;
        raw
    }
}

impl Encoder for BaseBoard {
    fn sample(&mut self) -> EncoderRaw {
        EncoderRaw {
            counts: self.enc.counts(),
            referenced: self.enc.referenced(),
            err: false,
        }
    }
}

impl CarriageImu for BaseBoard {
    fn sample(&mut self) -> ImuRaw {
        self.inertial.imu()
    }
}

impl RailAccel for BaseBoard {
    fn sample(&mut self) -> RailAccelRaw {
        self.inertial.rail()
    }
}

/// Supervisor-side hardware owned by the supervisor task.
pub struct SupHw {
    /// Watchdogs (started after the boot selftest).
    pub wd: Option<Watchdogs>,
    /// Reset cause captured at boot.
    pub reset: ResetCause,
    /// Handle load cell fitted (`design.sensors.handle_n_per_code > 0`).
    pub handle_cell: bool,
}

/// The supervisor board for one release: hardware plus the command queue it drains and
/// the telemetry queue it fills (both locked by the task for the job; their ceilings
/// equal the supervisor's own priority, so holding them costs nothing).
pub struct SupBoard<'a> {
    /// Hardware.
    pub hw: &'a mut SupHw,
    /// Commands from the link.
    pub cmds: &'a mut Ring<UserCommand, 8>,
    /// Telemetry for the link.
    pub tele: &'a mut TeleQueue,
}

impl PowerMonitor for SupBoard<'_> {
    fn sample(&mut self) -> PowerRaw {
        let p = adc::power_codes();
        PowerRaw {
            v_bus_code: p.v_bus,
            v_hold_code: p.v_hold,
            i_bus_code: p.i_bus,
            mains_ok: pins::MAINS_OK.is_high(),
            temp_code: p.temp,
        }
    }
}

impl SafetyInputs for SupBoard<'_> {
    fn sample(&mut self) -> SafetyInRaw {
        // The load-cell amplifier is biased at mid-scale (12-bit, 2048 = 0 N).
        let handle = i32::from(adc::power_codes().handle) - 2048;
        SafetyInRaw {
            presence_ok: pins::PRESENCE_OK.is_high(),
            presence_active: pins::PRESENCE_ACTIVE.is_high(),
            handle_force_code: handle as i16,
            has_handle_cell: self.hw.handle_cell,
            fire_alarm: !pins::FIRE_ALARM_N.is_high(),
            estop: !pins::ESTOP_LOOP.is_high(),
            lock_engaged: pins::LOCK_ENGAGED.is_high(),
            hold_to_run: pins::HOLD_TO_RUN.is_high(),
        }
    }
}

impl Aux for SupBoard<'_> {
    fn command(&mut self, c: &AuxCmd) {
        pins::PARKING_LOCK.set(c.parking_lock);
        pins::SEISMIC_LATCH.set(c.seismic_latch);
    }
}

impl CommandLink for SupBoard<'_> {
    fn poll(&mut self) -> Option<UserCommand> {
        self.cmds.pop()
    }
}

impl Watchdog for SupBoard<'_> {
    fn feed(&mut self) {
        if let Some(wd) = self.hw.wd.as_mut() {
            wd.feed();
        }
    }

    fn reset_cause(&self) -> ResetCause {
        self.hw.reset
    }
}

impl Telemetry for SupBoard<'_> {
    fn frame(&mut self, f: &TelemetryFrame) {
        self.tele.latest = Some(*f);
        self.tele.frames = self.tele.frames.wrapping_add(1);
    }

    fn event(&mut self, e: &Event) {
        let _ = self.tele.events.push(*e);
    }
}

/// The seismic board for one release: the accelerometer samples staged from the
/// DRDY/DMA queue and at most one alert from the link.
pub struct SeisBoard<'a> {
    /// Samples, oldest first (≤ `ACCEL_DRAIN_MAX`).
    pub samples: &'a [AccelRaw],
    /// Pending alert.
    pub alert: Option<NetworkAlert>,
}

impl Accelerometer for SeisBoard<'_> {
    fn drain(&mut self, out: &mut [AccelRaw; ACCEL_DRAIN_MAX]) -> usize {
        let mut n = 0;
        for (dst, src) in out.iter_mut().zip(self.samples.iter()) {
            *dst = *src;
            n += 1;
        }
        self.samples = self.samples.get(n..).unwrap_or(&[]);
        n
    }
}

impl AlertLink for SeisBoard<'_> {
    fn poll(&mut self) -> Option<NetworkAlert> {
        self.alert.take()
    }
}
