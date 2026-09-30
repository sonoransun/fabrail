//! Independent and window watchdogs (RM0440 §41 IWDG, §42 WWDG).
//!
//! Both start only after the boot selftest (plan §2b item 15: the ≈ 65 ms
//! `selftest_hash` run happens before the IWDG exists) and are fed only by the
//! supervisor group, and only when `fabrail_rt::WatchdogRule` says every group's
//! sequence number advanced without an overrun.
//!
//! * **IWDG** (LSI 32 kHz, prescaler /4 → 8 kHz): reload = `supervisor.watchdog_s` ×
//!   8 kHz (5 ms → 40 counts; core design §F "IWDG about 5 ms"). It keeps running from
//!   the LSI even if the main clock fails.
//! * **WWDG** (PCLK1 170 MHz, WDGTB = 3): one count = 4096·2³/170 MHz = 192.8 µs; the
//!   counter reloads to 0x7F and resets at 0x3F, i.e. after 64 counts = 12.3 ms
//!   (tWWDG formula RM0440 §42.3.4). The window is open (W = 0x7F): the feed cadence is
//!   fixed by the base task's supervisor releases, not by a software loop, and the very
//!   first feed follows the start by one supervisor period at most, so an early-refresh
//!   window would only add a start-up reset hazard. The WWDG's value is a second,
//!   independent clock domain (PCLK) next to the IWDG's LSI.

use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::stm32::{IWDG, WWDG};

/// LSI frequency, Hz (DS12288 Table 55, typical).
const LSI_HZ: f64 = 32_000.0;
/// IWDG prescaler divider (PR = 0 → /4).
const IWDG_DIV: f64 = 4.0;
// IWDG/WWDG registers are 16-bit in the PAC.
const KR_START: u16 = 0xCCCC;
const KR_ACCESS: u16 = 0x5555;
const KR_REFRESH: u16 = 0xAAAA;
/// WWDG: WDGTB = 3 (bits 13:11), window 0x7F (open); counter 0x7F with WDGA (bit 7).
const WWDG_CFR: u16 = (0b011 << 11) | 0x7F;
const WWDG_CR_RELOAD: u16 = (1 << 7) | 0x7F;

/// IWDG reload counts for a timeout (clamped to the 12-bit reload register).
pub fn iwdg_reload(timeout_s: f64) -> u16 {
    let counts = fabrail_types::fmath::round(timeout_s * LSI_HZ / IWDG_DIV);
    if counts.is_finite() {
        (counts.clamp(1.0, 4095.0)) as u16
    } else {
        40
    }
}

/// Both watchdogs, once started.
pub struct Watchdogs {
    iwdg: IWDG,
    wwdg: WWDG,
}

impl Watchdogs {
    /// Starts IWDG (reload from `timeout_s`) and WWDG. Irreversible until reset.
    pub fn start(iwdg: IWDG, wwdg: WWDG, timeout_s: f64, rcc: &Rcc) -> Watchdogs {
        let reload = iwdg_reload(timeout_s);
        // SAFETY: the IWDG key sequence and 12-bit reload of the owned IWDG (RM0440
        // §41.3.2); PR/RLR updates complete when SR clears (bounded wait).
        unsafe {
            iwdg.kr().write(|w| w.bits(KR_START));
            iwdg.kr().write(|w| w.bits(KR_ACCESS));
            iwdg.pr().write(|w| w.bits(0));
            iwdg.rlr().write(|w| w.bits(reload));
        }
        for _ in 0..100_000u32 {
            if iwdg.sr().read().bits() == 0 {
                break;
            }
        }
        // SAFETY: refresh key.
        unsafe { iwdg.kr().write(|w| w.bits(KR_REFRESH)) };
        rcc.apb1enr1().modify(|_, w| w.wwdgen().set_bit());
        // SAFETY: WWDG configuration and activation of the owned WWDG (RM0440 §42.5).
        unsafe {
            wwdg.cfr().write(|w| w.bits(WWDG_CFR));
            wwdg.cr().write(|w| w.bits(WWDG_CR_RELOAD));
        }
        Watchdogs { iwdg, wwdg }
    }

    /// Feeds both watchdogs.
    #[inline]
    pub fn feed(&mut self) {
        // SAFETY: refresh key / counter reload of the owned watchdogs.
        unsafe {
            self.iwdg.kr().write(|w| w.bits(KR_REFRESH));
            self.wwdg.cr().write(|w| w.bits(WWDG_CR_RELOAD));
        }
    }
}
