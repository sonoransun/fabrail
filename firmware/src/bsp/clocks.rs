//! Clocks, reset cause and core debug counters (RM0440 §6 PWR, §7 RCC; ARMv7-M ARM
//! §C1.8 DWT).
//!
//! **System clock.** 24 MHz HSE (NUCLEO-G474RE crystal X3, UM2505 §6.7.1) → PLL with
//! M = 6 (4 MHz input, inside 2.66–8 MHz), N = 85 (VCO 340 MHz, inside 96–344 MHz),
//! R = 2 → SYSCLK = 170 MHz (RM0440 §7.2.4 and DS12288 Table 43). f > 150 MHz needs
//! Range-1 boost mode (PWR_CR5.R1MODE = 0) and 4 flash wait states (RM0440 §5.3.3,
//! Table 29); stm32g4xx-hal's `Rcc::freeze` applies both. AHB, APB1 and APB2 are not
//! divided, so TIM1/TIM2/HRTIM count at 170 MHz. FDCAN runs from the HSE (24 MHz).

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::{DCB, DWT, SCB};
use fabrail_types::ResetCause;
use stm32g4xx_hal::pwr::{PwrExt, VoltageScale};
use stm32g4xx_hal::rcc::{
    Config, FdCanClockSource, PllConfig, PllMDiv, PllNMul, PllRDiv, PllSrc, Prescaler, Rcc, RccExt,
};
use stm32g4xx_hal::stm32::{PWR, RCC};
use stm32g4xx_hal::time::RateExtU32;

// RTIC monotonic for the async telemetry task: SysTick at 1 kHz (rtic-monotonics
// `cortex-m-systick`; its handler runs at the lowest urgency, see
// [`systick_lowest_priority`]).
rtic_monotonics::systick_monotonic!(Mono, 1_000);

/// HSE crystal frequency, MHz (NUCLEO-G474RE X3).
pub const HSE_MHZ: u32 = 24;

/// Configures 170 MHz and returns the frozen HAL clock tree.
pub fn init(rcc: RCC, pwr: PWR) -> Rcc {
    let pwr = pwr
        .constrain()
        .vos(VoltageScale::Range1 { enable_boost: true })
        .freeze();
    let cfg = Config::pll()
        .pll_cfg(PllConfig {
            mux: PllSrc::HSE(HSE_MHZ.MHz()),
            m: PllMDiv::DIV_6,
            n: PllNMul::MUL_85,
            r: Some(PllRDiv::DIV_2),
            q: None,
            p: None,
        })
        .ahb_psc(Prescaler::NotDivided)
        .apb1_psc(Prescaler::NotDivided)
        .apb2_psc(Prescaler::NotDivided)
        .boost(true)
        .fdcan_src(FdCanClockSource::HSE);
    rcc.constrain().freeze(cfg, pwr)
}

/// RCC_CSR reset flags (RM0440 §7.4.29).
const LPWRRSTF: u32 = 1 << 31;
const WWDGRSTF: u32 = 1 << 30;
const IWDGRSTF: u32 = 1 << 29;
const SFTRSTF: u32 = 1 << 28;
const BORRSTF: u32 = 1 << 27;
const PINRSTF: u32 = 1 << 26;

/// Reads the reset flags, then clears them (RMVF) so the next reset reports afresh.
/// A software reset after a crash record is reported as `Crash` by the caller
/// ([`super::crash`]); here the flags alone decide. Every reset asserts NRST, so
/// PINRSTF is the fallback; a power-on also sets BORRSTF (RM0440 §7.1.2).
pub fn take_reset_cause(rcc: &Rcc) -> ResetCause {
    let f = rcc.csr().read().bits();
    rcc.csr().modify(|_, w| w.rmvf().set_bit());
    if f & IWDGRSTF != 0 {
        ResetCause::IndependentWatchdog
    } else if f & WWDGRSTF != 0 {
        ResetCause::WindowWatchdog
    } else if f & SFTRSTF != 0 {
        ResetCause::Software
    } else if f & LPWRRSTF != 0 {
        ResetCause::LowPower
    } else if f & BORRSTF != 0 {
        ResetCause::PowerOn
    } else if f & PINRSTF != 0 {
        ResetCause::Pin
    } else {
        ResetCause::Unknown
    }
}

/// Starts the DWT cycle counter used for per-task execution-time telemetry and the
/// overrun checks (core design §F: "DWT CYCCNT min/max per task goes into telemetry").
pub fn enable_cycle_counter(dcb: &mut DCB, dwt: &mut DWT) {
    dcb.enable_trace();
    dwt.enable_cycle_counter();
}

/// Current DWT cycle count (wraps every 2³²/170 MHz ≈ 25 s; differences are taken with
/// wrapping arithmetic).
#[inline(always)]
pub fn cycles() -> u32 {
    DWT::cycle_count()
}

/// Gives SysTick (the RTIC monotonic tick for the telemetry task) the lowest urgency,
/// so it never delays the 20 kHz base task. With 4 priority bits (RM0440 §12.1) the
/// hardware value 0xF0 is logical priority 1.
pub fn systick_lowest_priority(scb: &mut SCB) {
    // SAFETY: changing an exception priority is unsafe because it can break
    // priority-based critical sections. SysTick only runs the rtic-monotonics timer
    // queue, which protects its state with critical sections, not BASEPRI ceilings, and
    // no RTIC resource is shared with it; this runs in init with interrupts disabled.
    unsafe { scb.set_priority(SystemHandler::SysTick, 0xF0) }
}
