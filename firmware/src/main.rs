//! fabrail firmware: the `fabrail-rt` executive as an RTIC 2 application on the
//! STM32G474RE (core design §F; plan §2b items 14–15; FAB-FW-001…013).
//!
//! **Status: builds and links for thumbv7em-none-eabihf; never flashed.** Cycle counts,
//! PWM-synchronous sampling, comparator latency and every register value are unverified
//! on hardware (FAB-FW-012). See `README.md`.
//!
//! **SIL ≡ firmware.** The tasks below run the same `fabrail-rt` group code as the
//! simulator (`BaseTask`, `FastGroup`, `SupervisorGroup`, `SeismicGroup`, LET
//! `Mailboxes`); only the HAL boards differ (`hal_impl.rs` here, `SimBoard` in SIL).
//! The claim holds in voltage coil mode only, which is the only mode this image accepts.
//!
//! # Task map
//!
//! NVIC with 4 priority bits (RTIC priorities 1..=16). `FMAC`, `CORDIC` and `I2C4_EV`
//! are otherwise unused vectors that the base task pends (their peripherals stay off),
//! which releases the lower-rate groups drift-free on base ticks.
//!
//! | Task | Binds | Rate | Priority | Work |
//! |---|---|---|---|---|
//! | `base` | ADC1_2 (ADC1 injected end of sequence, triggered by TIM1_TRGO at the PWM peak) | 20 kHz | 8 | `BaseTask::run` (coil current PI, LSM d/q PI in the inverter driver); snapshots the LET inputs of every group released at this tick; pends fast / supervisor / seismic |
//! | `fast` | FMAC (pended) | 10 / 5 kHz | 6 | `FastGroup::run` (frontend, levitation, motion, haptics, monitor, arbitration) |
//! | `accel` | EXTI9_5 (ADXL355 DRDY, EXTI6) | 500 Hz | 5 | starts the SPI1 DMA read |
//! | `accel_dma` | DMA1_CH1 (SPI1 RX complete) | 500 Hz | 5 | decodes the sample into the seismic queue |
//! | `supervisor` | CORDIC (pended) | 1 kHz | 4 | `SupervisorGroup::run`, watchdog feed |
//! | `seismic` | I2C4_EV (pended) | 100 Hz | 3 | `SeismicGroup::run` |
//! | `can_rx` | FDCAN1_IT0 | event | 3 | commands and network alerts into queues |
//! | `telemetry` | async software task (dispatcher LPTIM1) | 100 Hz | 1 | frames, events and link status over FDCAN1; defmt/RTT with `dev` |
//! | `idle` | — | — | 0 | WFI |
//!
//! Software-task dispatchers: `UCPD1`, `LPTIM1` (unused peripherals); RTIC assigns
//! `LPTIM1` to the priority-1 telemetry task and keeps `UCPD1` spare.
//!
//! # LET on RTIC
//!
//! A group released at base tick r must read the mailbox values active at r. The base
//! task (highest priority, running exactly at r) therefore takes the input snapshot of
//! every group it releases (`read_fast/sup/seis(r)`) after its own publication, and the
//! group task, whenever it runs, uses that snapshot and publishes with release r
//! (activation r + P). This is the order `Executive::on_base_tick` uses (base first; by
//! LET the other groups' order is immaterial, FAB-FW-004). A job still unfinished at its
//! next release is an overrun: the base task flags it, the group reports it with
//! `report_overrun` and the rt deadline supervision raises FastOverrun /
//! SupervisorOverrun / SeismicOverrun (SAF-013) and withholds the watchdog feed.
//!
//! # Boot sequence
//!
//! 1. Gate enables preset low; crash record and reset flags read (a panic/HardFault
//!    record in `.uninit`, or an IWDG/WWDG reset, holds the outputs off, below).
//! 2. 170 MHz clocks, pins, DWT cycle counter, SysTick monotonic.
//! 3. SELFTEST (before any watchdog, plan §2b item 15): FPSCR FZ = DN = 0 and
//!    `fabrail_rt::selftest_hash(SELFTEST_TICKS) == GOLDEN_SELFTEST_HASH`.
//! 4. Parameters from the flash A/B pages (factory preset of the build class when both
//!    pages are erased): decode, safety seal (`crc::verify_seal`), `validate`, build
//!    class, voltage coil mode, 20 kHz base rate.
//! 5. Peripherals, always (so the power stages are held in their safe state): hardware
//!    overcurrent trips, TIM1 in STO, HRTIM outputs disabled, ADCs, encoder, SPI
//!    sensors, FDCAN.
//! 6. If every check passed: `Executive::new` → `split()` into the tasks, arm the ADC
//!    triggers (the base interrupt starts at the next PWM peak), start IWDG + WWDG.
//!    Otherwise **boot hold**: no control task runs, outputs stay off, the link status
//!    frame reports why. A crash or watchdog hold is cleared by a `Reset` command over
//!    the link only while every gap reads at rest (≥ g_land, or ≥ g_seat for a seated
//!    DRAWER, minus 3σ of the gap-sensor noise; core design §F), which clears the crash
//!    record and resets. Until Wave 3 freezes `GOLDEN_SELFTEST_HASH` (placeholder 0),
//!    every build boots into the SELFTEST hold, as intended.

#![no_std]
#![no_main]
#![deny(unsafe_code)]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

#[cfg(not(any(
    feature = "class-drawer",
    feature = "class-door",
    feature = "class-wall",
    feature = "class-heavy"
)))]
compile_error!("enable one of the class-drawer / class-door / class-wall / class-heavy features");

#[cfg(any(
    all(feature = "class-drawer", feature = "class-door"),
    all(feature = "class-drawer", feature = "class-wall"),
    all(feature = "class-drawer", feature = "class-heavy"),
    all(feature = "class-door", feature = "class-wall"),
    all(feature = "class-door", feature = "class-heavy"),
    all(feature = "class-wall", feature = "class-heavy"),
))]
compile_error!("enable exactly one class feature (use --no-default-features for non-door classes)");

mod bsp;
mod hal_impl;

#[cfg(feature = "dev")]
use defmt_rtt as _;
#[cfg(feature = "dev")]
use panic_probe as _;

use fabrail_core::frontend;
use fabrail_hal::ParamStore;
use fabrail_params::{RestPolicy, SensorCal, SystemParams, presets};
use fabrail_rt::{FastIn, SeisIn, SupIn};
use fabrail_types::{
    BASE_TICK_HZ, FastRaw, FaultCode, GapRaw, MAX_MAGNETS, RailClass, ResetCause, Tick,
};

use bsp::crash::CrashRecord;
use bsp::flash_params::{self, FlashError, FlashParams};

/// The rail class this image is built for (selects the factory preset and is checked
/// against the stored parameters).
#[cfg(feature = "class-drawer")]
pub const CLASS: RailClass = RailClass::Drawer;
/// The rail class this image is built for.
#[cfg(feature = "class-door")]
pub const CLASS: RailClass = RailClass::Door;
/// The rail class this image is built for.
#[cfg(feature = "class-wall")]
pub const CLASS: RailClass = RailClass::Wall;
/// The rail class this image is built for.
#[cfg(feature = "class-heavy")]
pub const CLASS: RailClass = RailClass::Heavy;

/// The factory preset of the build class. Only this class's preset (plus the DOOR
/// preset the selftest uses) is linked: `presets::by_class` would pull all four.
fn factory_preset() -> SystemParams {
    #[cfg(feature = "class-drawer")]
    let p = presets::drawer();
    #[cfg(feature = "class-door")]
    let p = presets::door();
    #[cfg(feature = "class-wall")]
    let p = presets::wall();
    #[cfg(feature = "class-heavy")]
    let p = presets::heavy();
    p
}

/// Why the firmware holds its outputs off instead of running the control tasks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BootHold {
    /// Selftest hash differs from `GOLDEN_SELFTEST_HASH`, or FPSCR FZ/DN set.
    Selftest = 1,
    /// Parameter image corrupt (CRC, decode or safety seal).
    ParamCrc = 2,
    /// Parameters out of range or not for this build (class, coil mode, base rate).
    ParamRange = 3,
    /// A converter or trip DAC did not come up.
    Hardware = 4,
    /// The previous run ended in an IWDG/WWDG reset.
    Watchdog = 5,
    /// The previous run ended in a panic, HardFault or unexpected interrupt.
    Crash = 6,
}

impl BootHold {
    /// The fault code reported for this hold (a crash is reported by its record).
    pub const fn fault_code(self) -> Option<FaultCode> {
        match self {
            BootHold::Selftest => Some(FaultCode::SelftestHash),
            BootHold::ParamCrc => Some(FaultCode::ParamCrc),
            BootHold::ParamRange => Some(FaultCode::ParamRange),
            BootHold::Hardware => Some(FaultCode::CurrentSense),
            BootHold::Watchdog => Some(FaultCode::WatchdogReset),
            BootHold::Crash => None,
        }
    }

    /// A Reset command over the link may clear this hold (the others recur until the
    /// firmware or parameters are replaced or the hardware is repaired).
    pub const fn operator_clearable(self) -> bool {
        matches!(self, BootHold::Watchdog | BootHold::Crash)
    }
}

/// What the boot sequence found (reported in the link status frame).
#[derive(Clone, Copy, Debug)]
pub struct BootStatus {
    /// First hold reason found, if any.
    pub hold: Option<BootHold>,
    /// Reset cause (RCC CSR, or `Crash` when a crash record was present).
    pub reset: ResetCause,
    /// Crash record left by the previous run.
    pub crash: Option<CrashRecord>,
    /// Computed selftest hash.
    pub selftest_hash: u64,
    /// FPSCR FZ = DN = 0.
    pub fpscr_ok: bool,
    /// Converters and trip DACs came up.
    pub hw_ok: bool,
}

impl BootStatus {
    fn hold(&mut self, h: BootHold) {
        if self.hold.is_none() {
            self.hold = Some(h);
        }
    }
}

/// A job the base task released: release tick, LET input snapshot, and the
/// release/completion counters that detect overruns.
#[derive(Clone, Copy, Debug, Default)]
pub struct Job<T: Copy + Default> {
    /// Release tick.
    pub t: Tick,
    /// Input snapshot taken by the base task at `t`.
    pub input: T,
    /// Jobs released.
    pub released: u32,
    /// Last job completed.
    pub done: u32,
    /// A release found the previous job unfinished.
    pub overrun: bool,
}

impl<T: Copy + Default> Job<T> {
    fn release(&mut self, t: Tick, input: T) {
        if self.released != self.done {
            self.overrun = true;
        }
        self.t = t;
        self.input = input;
        self.released = self.released.wrapping_add(1);
    }

    /// (release tick, input, job number, overrun flag), clearing the flag.
    fn take(&mut self) -> (Tick, T, u32, bool) {
        let o = core::mem::replace(&mut self.overrun, false);
        (self.t, self.input, self.released, o)
    }
}

/// Fast-group job input: the `FastRaw` latched by the base task and the LET inputs.
#[derive(Clone, Copy, Debug, Default)]
pub struct FastJob {
    /// Raw samples latched at the release.
    pub raw: FastRaw,
    /// Mailbox inputs active at the release.
    pub inp: FastIn,
}

/// Per-task execution statistics (DWT cycles; reported and reset by the telemetry task).
#[derive(Clone, Copy, Debug, Default)]
pub struct CycleStats {
    /// Maximum cycles per job: base, fast, supervisor, seismic.
    pub max: [u32; 4],
    /// Overruns: fast, supervisor, seismic.
    pub overruns: [u32; 3],
}

impl CycleStats {
    fn note(&mut self, k: usize, cycles: u32) {
        if let Some(m) = self.max.get_mut(k) {
            *m = (*m).max(cycles);
        }
    }

    fn overrun(&mut self, k: usize) {
        if let Some(o) = self.overruns.get_mut(k) {
            *o = o.wrapping_add(1);
        }
    }
}

/// Everything the crash/watchdog-hold Reset needs to decide "at rest".
#[derive(Clone, Copy, Debug)]
pub struct RestCheck {
    /// Sensor calibration (gap linearisation).
    pub cal: SensorCal,
    /// Gap at rest minus 3σ of the gap-sensor noise, m.
    pub threshold_m: f32,
    /// Magnets fitted.
    pub n: u8,
}

impl RestCheck {
    fn new(p: &SystemParams) -> RestCheck {
        let s = &p.design.suspension;
        let rest = if p.design.rest_policy == RestPolicy::Seat {
            s.g_seat_m
        } else {
            s.g_land_m
        };
        RestCheck {
            cal: p.design.sensors,
            threshold_m: (rest - 3.0 * p.design.sensors.gap_noise_m) as f32,
            n: s.n_magnets.min(MAX_MAGNETS as u8),
        }
    }

    /// Every fitted gap valid and at rest (landed on the rollers or seated), converted
    /// by the core frontend from the live gap-probe codes.
    fn at_rest(&self) -> bool {
        let (a, b) = bsp::adc::gap_codes();
        let n = usize::from(self.n);
        let raw = FastRaw {
            gap: GapRaw {
                code: a,
                code_b: if self.cal.gap_redundant {
                    b
                } else {
                    [0; MAX_MAGNETS]
                },
                valid: ((1u16 << n) - 1) as u8,
            },
            ..FastRaw::default()
        };
        let (m, _) = frontend::convert(&raw, &self.cal);
        n > 0
            && (0..n).all(|i| {
                m.gap_ok & (1 << i) != 0 && m.gap_m.get(i).is_some_and(|g| *g >= self.threshold_m)
            })
    }
}

/// Loads, checks and returns the parameter set (boot step 4).
fn load_params(store: &mut FlashParams) -> Result<SystemParams, BootHold> {
    let mut buf = [0u8; flash_params::PAYLOAD_MAX];
    let p = match store.read(&mut buf) {
        Ok(n) => postcard::from_bytes::<SystemParams>(buf.get(..n).unwrap_or(&[]))
            .map_err(|_| BootHold::ParamCrc)?,
        Err(FlashError::Erased) => factory_preset(),
        Err(_) => return Err(BootHold::ParamCrc),
    };
    fabrail_params::crc::verify_seal(&p).map_err(|_| BootHold::ParamCrc)?;
    p.validate().map_err(|_| BootHold::ParamRange)?;
    // Firmware constraints on top of `validate`: this build's class, the only coil mode
    // SIL ≡ firmware is claimed for, the 20 kHz PWM/ADC timing of the BSP, and an ODR
    // the ADXL355 offers.
    let r = &p.control.rates;
    if p.design.class != CLASS
        || !r.coil_mode_voltage
        || r.base_hz != BASE_TICK_HZ
        || bsp::accel_spi::adxl355_odr_code(p.design.sensors.accel_odr_hz).is_none()
    {
        return Err(BootHold::ParamRange);
    }
    Ok(p)
}

/// Link status words (the 0x320 frame; `bsp::can::encode_status`).
fn status_words(b: &BootStatus, s: &CycleStats, frames: u32, running: bool) -> [u32; 16] {
    let hold = b.hold.map_or(0, |h| h as u32);
    let crash = b.crash.as_ref();
    [
        u32::from(running)
            | (hold << 8)
            | (u32::from(b.fpscr_ok) << 16)
            | (u32::from(b.hw_ok) << 17),
        b.hold
            .and_then(BootHold::fault_code)
            .map_or(0, |c| u32::from(c as u16)),
        u32::from(b.reset.as_u8()),
        crash.map_or(0, |c| c.kind),
        crash.map_or(0, |c| c.pc),
        crash.map_or(0, |c| c.line),
        b.selftest_hash as u32,
        (b.selftest_hash >> 32) as u32,
        s.max[0],
        s.max[1],
        s.max[2],
        s.max[3],
        s.overruns[0],
        s.overruns[1],
        s.overruns[2],
        frames,
    ]
}

#[rtic::app(device = stm32g4xx_hal::stm32, peripherals = true, dispatchers = [UCPD1, LPTIM1])]
mod app {
    use super::*;

    use fabrail_params::Rates;
    use fabrail_rt::{
        BaseTask, Executive, FastGroup, GOLDEN_SELFTEST_HASH, Mailboxes, SELFTEST_TICKS,
        SeismicGroup, SupervisorGroup, selftest_hash,
    };
    use fabrail_types::{ACCEL_DRAIN_MAX, AccelRaw, Event, EventKind, NetworkAlert, UserCommand};
    use rtic_monotonics::Monotonic;
    use rtic_monotonics::fugit::ExtU32;
    use stm32g4xx_hal::stm32::Interrupt;

    use crate::bsp::accel_spi::{Adxl355, InertialSpi};
    use crate::bsp::adc::Adc;
    use crate::bsp::can::{self, CanRx, CanTx, Inbound};
    use crate::bsp::clocks::{self, Mono};
    use crate::bsp::encoder::Encoder;
    use crate::bsp::hrtim::Hrtim;
    use crate::bsp::tim1::Tim1;
    use crate::bsp::watchdog::Watchdogs;
    use crate::bsp::{SYSCLK_HZ, crash, pins, protect};
    use crate::hal_impl::{BaseBoard, Ring, SeisBoard, SupBoard, SupHw, TeleQueue};

    /// The base task's state (absent in a boot hold).
    pub struct BaseCtx {
        task: BaseTask,
        board: BaseBoard,
        rates: Rates,
    }

    #[shared]
    struct Shared {
        mb: Mailboxes,
        fast_job: Job<FastJob>,
        sup_job: Job<SupIn>,
        seis_job: Job<SeisIn>,
        now: Tick,
        accel_q: Ring<AccelRaw, 32>,
        cmd_q: Ring<UserCommand, 8>,
        alert_q: Ring<NetworkAlert, 4>,
        tele_q: TeleQueue,
        stats: CycleStats,
        #[lock_free]
        adxl: Adxl355,
    }

    #[local]
    struct Local {
        base: Option<BaseCtx>,
        fast: Option<FastGroup>,
        sup: Option<SupervisorGroup>,
        sup_hw: SupHw,
        seis: Option<SeismicGroup>,
        can_rx: CanRx,
        rest: Option<RestCheck>,
        hold_rx: Option<BootHold>,
        can_tx: CanTx,
        boot: BootStatus,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let mut core = cx.core;

        // 1. Power stages held off; what ended the previous run.
        pins::force_gates_off();
        let crash_rec = crash::take();

        // 2. Clocks, pins, counters.
        let rcc = clocks::init(dp.RCC, dp.PWR);
        let flags = clocks::take_reset_cause(&rcc);
        let reset = if crash_rec.is_some() {
            ResetCause::Crash
        } else {
            flags
        };
        pins::configure(&rcc);
        clocks::enable_cycle_counter(&mut core.DCB, &mut core.DWT);
        clocks::systick_lowest_priority(&mut core.SCB);
        Mono::start(core.SYST, SYSCLK_HZ);

        // 3. SELFTEST (before any watchdog).
        let fpscr = cortex_m::register::fpscr::read();
        let fpscr_ok = !fpscr.fz() && !fpscr.dn();
        let hash = selftest_hash(SELFTEST_TICKS);
        let mut boot = BootStatus {
            hold: None,
            reset,
            crash: crash_rec,
            selftest_hash: hash,
            fpscr_ok,
            hw_ok: true,
        };
        if crash_rec.is_some() {
            boot.hold(BootHold::Crash);
        } else if matches!(
            reset,
            ResetCause::IndependentWatchdog | ResetCause::WindowWatchdog
        ) {
            boot.hold(BootHold::Watchdog);
        }
        if !fpscr_ok || hash != GOLDEN_SELFTEST_HASH {
            boot.hold(BootHold::Selftest);
        }

        // 4. Parameters.
        let mut store = FlashParams::new(dp.FLASH);
        let params = match load_params(&mut store) {
            Ok(p) => Some(p),
            Err(h) => {
                boot.hold(h);
                None
            }
        };
        let fallback = factory_preset();
        let hw_p = params.as_ref().unwrap_or(&fallback);

        // 5. Peripherals (always: they hold the power stages safe).
        boot.hw_ok &= protect::init(&dp.COMP, &dp.DAC3, &dp.DAC4, &rcc);
        let tim1 = Tim1::init(dp.TIM1, &rcc);
        let hrtim = Hrtim::init(
            dp.HRTIM_COMMON,
            dp.HRTIM_MASTER,
            (dp.HRTIM_TIMA, dp.HRTIM_TIMB, dp.HRTIM_TIMC, dp.HRTIM_TIMD),
            hw_p.design.suspension.n_magnets,
            &rcc,
        );
        let (mut adc, adc_ok) = Adc::init(
            (dp.ADC1, dp.ADC2, dp.ADC3, dp.ADC4, dp.ADC5),
            (dp.ADC12_COMMON, dp.ADC345_COMMON),
            &dp.DMA1,
            &dp.DMAMUX,
            hw_p.design.suspension.n_magnets,
            hw_p.design.sensors.gap_redundant,
            &rcc,
        );
        boot.hw_ok &= adc_ok;
        if !boot.hw_ok {
            boot.hold(BootHold::Hardware);
        }
        let enc = Encoder::init(dp.TIM2, &rcc);
        let inertial = InertialSpi::init(
            dp.SPI3,
            dp.DMA2,
            &dp.DMAMUX,
            hw_p.design.sensors.imu_present,
            hw_p.design.sensors.rail_accel_present,
            &rcc,
        );
        let adxl = Adxl355::init(
            dp.SPI1,
            dp.DMA1,
            dp.EXTI,
            &dp.SYSCFG,
            &dp.DMAMUX,
            hw_p.design.sensors.accel_odr_hz,
            &rcc,
        );
        let (can_rx, can_tx) = can::init(dp.FDCAN1, &rcc);

        // 6. Run, or hold.
        if boot.hw_ok {
            // Gap probes and power sense convert in both cases (the hold's Reset
            // interlock and telemetry read them).
            adc.start_regular();
        }
        let mut shared_mb = Mailboxes::default();
        let (mut base, mut fast, mut sup, mut seis) = (None, None, None, None);
        let mut wd_timeout_s = None;
        let mut sup_hw = SupHw {
            wd: None,
            reset,
            handle_cell: hw_p.design.sensors.handle_n_per_code > 0.0,
        };
        if let (None, Some(p)) = (boot.hold, params.as_ref()) {
            match Executive::new(p) {
                Ok(ex) => match BaseBoard::new(tim1, hrtim, adc, enc, inertial, p) {
                    Ok(board) => {
                        let rates = ex.rates;
                        let (b, f, s, z, mb) = ex.split();
                        base = Some(BaseCtx {
                            task: b,
                            board,
                            rates,
                        });
                        fast = Some(f);
                        sup = Some(s);
                        seis = Some(z);
                        shared_mb = mb;
                        wd_timeout_s = Some(p.control.supervisor.watchdog_s);
                    }
                    // The drivers are dropped with their outputs in STO / disabled.
                    Err(_) => boot.hold(BootHold::ParamRange),
                },
                Err(_) => boot.hold(BootHold::ParamRange),
            }
        }
        if let (Some(b), Some(timeout_s)) = (base.as_mut(), wd_timeout_s) {
            // Watchdogs start only after the selftest and only when control runs.
            sup_hw.wd = Some(Watchdogs::start(dp.IWDG, dp.WWDG, timeout_s, &rcc));
            b.board.arm_adc();
        }

        let mut tele_q = TeleQueue::default();
        let _ = tele_q.events.push(Event {
            tick: Tick(0),
            kind: EventKind::Selftest,
            a: if boot.hold.is_none() { 1.0 } else { 0.0 },
            b: 0.0,
        });
        if let Some(code) = boot.hold.and_then(BootHold::fault_code) {
            let _ = tele_q.events.push(Event {
                tick: Tick(0),
                kind: EventKind::FaultRaised,
                a: f32::from(code as u16),
                b: 0.0,
            });
        }
        #[cfg(feature = "dev")]
        defmt::info!(
            "fabrail-fw boot: class {=u8} hold {=u8} selftest {=u64:#x} golden {=u64:#x}",
            CLASS.as_u8(),
            boot.hold.map_or(0, |h| h as u8),
            hash,
            GOLDEN_SELFTEST_HASH
        );
        telemetry::spawn().ok();

        (
            Shared {
                mb: shared_mb,
                fast_job: Job::default(),
                sup_job: Job::default(),
                seis_job: Job::default(),
                now: Tick(0),
                accel_q: Ring::new(),
                cmd_q: Ring::new(),
                alert_q: Ring::new(),
                tele_q,
                stats: CycleStats::default(),
                adxl,
            },
            Local {
                base,
                fast,
                sup,
                sup_hw,
                seis,
                can_rx,
                rest: params.as_ref().map(RestCheck::new),
                hold_rx: boot.hold,
                can_tx,
                boot,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    /// 20 kHz base task.
    #[task(binds = ADC1_2, priority = 8, local = [base, tick: u64 = 0],
           shared = [mb, fast_job, sup_job, seis_job, now, stats])]
    fn base(cx: base::Context) {
        let c0 = clocks::cycles();
        let Some(ctx) = cx.local.base.as_mut() else {
            return;
        };
        ctx.board.begin_tick();
        let t = Tick(*cx.local.tick);
        *cx.local.tick = cx.local.tick.wrapping_add(1);
        crash::note_tick(t.0);
        let r = ctx.rates;
        let sup_rel = t.is_release(u32::from(r.sup_decim));
        let seis_rel = t.is_release(u32::from(r.seismic_decim));
        if t.is_release(u32::from(r.fast_decim)) {
            ctx.board.poll_inertial();
        }
        let mut shared = cx.shared;
        shared.now.lock(|n| *n = t);
        let mut pend = [false; 3];
        (
            &mut shared.mb,
            &mut shared.fast_job,
            &mut shared.sup_job,
            &mut shared.seis_job,
        )
            .lock(|mb, fj, sj, zj| {
                let bi = mb.read_base(t);
                let (bo, raw) = ctx.task.run(t, &bi, &mut ctx.board);
                mb.publish_base(t, &bo);
                if let Some(raw) = raw {
                    fj.release(
                        t,
                        FastJob {
                            raw,
                            inp: mb.read_fast(t),
                        },
                    );
                    pend[0] = true;
                }
                if sup_rel {
                    sj.release(t, mb.read_sup(t));
                    pend[1] = true;
                }
                if seis_rel {
                    zj.release(t, mb.read_seis(t));
                    pend[2] = true;
                }
            });
        for (due, irq) in pend
            .iter()
            .zip([Interrupt::FMAC, Interrupt::CORDIC, Interrupt::I2C4_EV])
        {
            if *due {
                rtic::pend(irq);
            }
        }
        let dt = clocks::cycles().wrapping_sub(c0);
        shared.stats.lock(|s| s.note(0, dt));
    }

    /// Fast group (10 / 5 kHz).
    #[task(binds = FMAC, priority = 6, local = [fast], shared = [mb, fast_job, stats])]
    fn fast(mut cx: fast::Context) {
        let c0 = clocks::cycles();
        let Some(group) = cx.local.fast.as_mut() else {
            return;
        };
        let (t, job, seq, overrun) = cx.shared.fast_job.lock(|j| j.take());
        if overrun {
            group.report_overrun();
            cx.shared.stats.lock(|s| s.overrun(0));
        }
        let out = group.run(t, &job.raw, &job.inp);
        cx.shared.mb.lock(|mb| mb.publish_fast(t, &out));
        cx.shared.fast_job.lock(|j| j.done = seq);
        let dt = clocks::cycles().wrapping_sub(c0);
        cx.shared.stats.lock(|s| s.note(1, dt));
    }

    /// ADXL355 DRDY: start the SPI DMA read.
    #[task(binds = EXTI9_5, priority = 5, shared = [adxl])]
    fn accel(cx: accel::Context) {
        cx.shared.adxl.on_drdy();
    }

    /// ADXL355 DMA complete: queue the sample for the seismic group.
    #[task(binds = DMA1_CH1, priority = 5, shared = [adxl, accel_q])]
    fn accel_dma(mut cx: accel_dma::Context) {
        if let Some(s) = cx.shared.adxl.on_dma_complete() {
            cx.shared.accel_q.lock(|q| {
                let _ = q.push(s);
            });
        }
    }

    /// Supervisor group (1 kHz).
    #[task(binds = CORDIC, priority = 4, local = [sup, sup_hw],
           shared = [mb, sup_job, cmd_q, tele_q, stats])]
    fn supervisor(mut cx: supervisor::Context) {
        let c0 = clocks::cycles();
        let Some(group) = cx.local.sup.as_mut() else {
            return;
        };
        let (t, inp, seq, overrun) = cx.shared.sup_job.lock(|j| j.take());
        if overrun {
            group.report_overrun();
            cx.shared.stats.lock(|s| s.overrun(1));
        }
        let hw = &mut *cx.local.sup_hw;
        let out = (&mut cx.shared.cmd_q, &mut cx.shared.tele_q).lock(|cmds, tele| {
            let mut board = SupBoard { hw, cmds, tele };
            group.run(t, &inp, &mut board)
        });
        cx.shared.mb.lock(|mb| mb.publish_sup(t, &out));
        cx.shared.sup_job.lock(|j| j.done = seq);
        let dt = clocks::cycles().wrapping_sub(c0);
        cx.shared.stats.lock(|s| s.note(2, dt));
    }

    /// Seismic group (100 Hz).
    #[task(binds = I2C4_EV, priority = 3, local = [seis],
           shared = [mb, seis_job, accel_q, alert_q, stats])]
    fn seismic(mut cx: seismic::Context) {
        let c0 = clocks::cycles();
        let Some(group) = cx.local.seis.as_mut() else {
            return;
        };
        let (t, inp, seq, overrun) = cx.shared.seis_job.lock(|j| j.take());
        if overrun {
            group.report_overrun();
            cx.shared.stats.lock(|s| s.overrun(2));
        }
        let mut stage = [AccelRaw::default(); ACCEL_DRAIN_MAX];
        let n = cx.shared.accel_q.lock(|q| {
            let mut n = 0;
            for slot in stage.iter_mut() {
                match q.pop() {
                    Some(s) => {
                        *slot = s;
                        n += 1;
                    }
                    None => break,
                }
            }
            n
        });
        let alert = cx.shared.alert_q.lock(|q| q.pop());
        let mut board = SeisBoard {
            samples: stage.get(..n).unwrap_or(&[]),
            alert,
        };
        let out = group.run(t, &inp, &mut board);
        cx.shared.mb.lock(|mb| mb.publish_seis(t, &out));
        cx.shared.seis_job.lock(|j| j.done = seq);
        let dt = clocks::cycles().wrapping_sub(c0);
        cx.shared.stats.lock(|s| s.note(3, dt));
    }

    /// FDCAN1 RX FIFO 0: commands and alerts into the queues; the Reset path of a boot
    /// hold.
    #[task(binds = FDCAN1_IT0, priority = 3, local = [can_rx, rest, hold_rx],
           shared = [now, cmd_q, alert_q])]
    fn can_rx(mut cx: can_rx::Context) {
        let mut buf = [0u8; 64];
        // RX FIFO 0 holds three elements (RM0440 §44.3.3): a bounded drain.
        for _ in 0..4 {
            let Some((id, len)) = cx.local.can_rx.receive(&mut buf) else {
                break;
            };
            let now = cx.shared.now.lock(|n| *n);
            match can::decode(id, buf.get(..len).unwrap_or(&[]), now) {
                Some(Inbound::Command(c)) => match *cx.local.hold_rx {
                    None => {
                        cx.shared.cmd_q.lock(|q| {
                            let _ = q.push(c);
                        });
                    }
                    Some(h) => {
                        let rest_ok = cx.local.rest.as_ref().is_some_and(RestCheck::at_rest);
                        if c == UserCommand::Reset && h.operator_clearable() && rest_ok {
                            crash::clear();
                            cortex_m::peripheral::SCB::sys_reset();
                        }
                    }
                },
                Some(Inbound::Alert(a)) if cx.local.hold_rx.is_none() => {
                    cx.shared.alert_q.lock(|q| {
                        let _ = q.push(a);
                    });
                }
                Some(Inbound::Alert(_)) | None => {}
            }
        }
    }

    /// Telemetry (100 Hz): latest frame, events, and every 100 ms the link status.
    #[task(priority = 1, local = [can_tx, boot, cycle: u32 = 0, tx_drops: u32 = 0],
           shared = [tele_q, stats])]
    async fn telemetry(mut cx: telemetry::Context) {
        loop {
            let (frame, frames, events, n_ev) = cx.shared.tele_q.lock(|q| {
                let mut ev = [Event::default(); 3];
                let mut n = 0;
                for slot in ev.iter_mut() {
                    match q.events.pop() {
                        Some(e) => {
                            *slot = e;
                            n += 1;
                        }
                        None => break,
                    }
                }
                (q.latest.take(), q.frames, ev, n)
            });
            let tx = &mut *cx.local.can_tx;
            let mut sent = 0u32;
            let mut tried = 0u32;
            if let Some(f) = frame {
                let (p0, p1, p2) = can::encode_telemetry(&f);
                tried += 3;
                sent += u32::from(can::send(tx, can::ID_TELEMETRY, &p0));
                sent += u32::from(can::send(tx, can::ID_TELEMETRY + 1, &p1));
                sent += u32::from(can::send(tx, can::ID_TELEMETRY + 2, &p2));
                #[cfg(feature = "dev")]
                defmt::info!(
                    "t={=u64} lev={=u8} seis={=u8} x_nm={=i64} gap0={=f32} demand={=u8}",
                    f.tick.0,
                    f.lev.as_u8(),
                    f.seis.as_u8(),
                    f.x.0,
                    f.gaps_m[0],
                    f.demand.as_u8()
                );
            }
            Mono::delay(2.millis()).await;
            for e in events.iter().take(n_ev) {
                tried += 1;
                sent += u32::from(can::send(tx, can::ID_EVENT, &can::encode_event(e)));
                #[cfg(feature = "dev")]
                defmt::info!(
                    "event t={=u64} kind={=u8} a={=f32} b={=f32}",
                    e.tick.0,
                    e.kind.as_u8(),
                    e.a,
                    e.b
                );
            }
            *cx.local.tx_drops = cx.local.tx_drops.wrapping_add(tried - sent);
            if cx.local.cycle.is_multiple_of(10) {
                Mono::delay(2.millis()).await;
                let stats = cx.shared.stats.lock(core::mem::take);
                let running = cx.local.boot.hold.is_none();
                let words = status_words(cx.local.boot, &stats, frames, running);
                let _ = can::send(tx, can::ID_STATUS, &can::encode_status(words));
            }
            *cx.local.cycle = cx.local.cycle.wrapping_add(1);
            Mono::delay(6.millis()).await;
        }
    }
}
