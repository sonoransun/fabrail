//! TIM2: 32-bit quadrature encoder extended to i64 (RM0440 §29.3.24 "Encoder interface
//! mode"; core design §F, PHYSICS §13 and §24).
//!
//! Encoder mode 3 (SMS = 011) counts every edge of TI1 and TI2 (×4 decoding); input
//! filter f_DTS/1, N = 8 (≈ 47 ns at 170 MHz) against line noise. The 32-bit counter is
//! extended to i64 by accumulating the wrapping difference every base tick (the 20 kHz
//! call rate bounds |Δ| far below 2³¹ counts per 50 µs). The reference mark (PC3,
//! active low) sets the zero: counts are reported relative to the first mark seen after
//! boot, and `referenced` becomes true (homing; `EncoderRaw` contract). A plain
//! quadrature interface has no hardware signal-loss flag, so `err` stays false here and
//! the frontend's plausibility check (implausible jump) covers loss (PHYSICS §13).

use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::stm32::TIM2;

use super::pins;

const SMS_ENCODER3: u32 = 0b011;
/// CCMR1 input: CC1S = 01 (IC1 ← TI1), IC1F = 0011; CC2S = 01 (IC2 ← TI2), IC2F = 0011.
const CCMR1_TI12: u32 = 0b01 | (0b0011 << 4) | (0b01 << 8) | (0b0011 << 12);
const CEN: u32 = 1;

/// The encoder counter and its 64-bit extension.
pub struct Encoder {
    tim: TIM2,
    last: u32,
    acc: i64,
    zero: i64,
    referenced: bool,
}

impl Encoder {
    /// Configures TIM2 in encoder mode and starts counting.
    pub fn init(tim: TIM2, rcc: &Rcc) -> Encoder {
        rcc.apb1enr1().modify(|_, w| w.tim2en().set_bit());
        // SAFETY: TIM2 is owned here and configured once in init; field encodings per
        // RM0440 §29.4 (SMCR.SMS, CCMR1 input mode, ARR full 32-bit range).
        unsafe {
            tim.cr1().write(|w| w.bits(0));
            tim.smcr().write(|w| w.bits(SMS_ENCODER3));
            tim.ccmr1_input().write(|w| w.bits(CCMR1_TI12));
            tim.ccer().write(|w| w.bits(0));
            tim.arr().write(|w| w.bits(u32::MAX));
            tim.cnt().write(|w| w.bits(0));
            tim.cr1().write(|w| w.bits(CEN));
        }
        Encoder {
            tim,
            last: 0,
            acc: 0,
            zero: 0,
            referenced: false,
        }
    }

    /// Updates the 64-bit count (call once per base tick) and the reference latch.
    #[inline]
    pub fn update(&mut self) {
        let now = self.tim.cnt().read().bits();
        let delta = now.wrapping_sub(self.last) as i32;
        self.last = now;
        self.acc = self.acc.wrapping_add(i64::from(delta));
        if !self.referenced && !pins::ENC_REF.is_high() {
            self.zero = self.acc;
            self.referenced = true;
        }
    }

    /// Counts relative to the reference mark (relative to power-up before homing).
    #[inline]
    pub fn counts(&self) -> i64 {
        self.acc.wrapping_sub(self.zero)
    }

    /// The reference mark has been seen since boot.
    #[inline]
    pub fn referenced(&self) -> bool {
        self.referenced
    }
}
