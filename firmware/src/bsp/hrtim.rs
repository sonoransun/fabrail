//! HRTIM timing units A–D: up to eight coil H-bridges (RM0440 §27 "High-resolution
//! timer").
//!
//! **Bridge drive.** Each HRTIM output drives one full bridge in *locked anti-phase*:
//! the gate driver makes leg B the complement of leg A (with its own dead time), so
//! V_coil = V_bus·(2d − 1) and one output per coil suffices. Coil i ↔ output i:
//! TA1, TA2, TB1, TB2, TC1, TC2, TD1, TD2 (HRTIM_OENR bits 0..7), which is how eight
//! coils (HEAVY, `MAX_MAGNETS`) fit on timers A–D. The volts → duty conversion
//! `d = 1/2 + v/(2·V_bus)` is done by the caller (`hal_impl`).
//!
//! **Timing.** f_HRTIM = 170 MHz with CKPSC = 0b101 (f_HRCK = f_HRTIM, 5.88 ns steps,
//! no DLL needed; RM0440 Table 213). Every timing unit is reset and started by the sync
//! input (HRTIM_MCR.SYNCIN = 0b10: internal TIM1_TRGO), i.e. once per 50 µs PWM period
//! of the LSM timer; PER = 8600 > 8500 counts is only the fallback roll-over. Output x1
//! is set on the resync/period event and reset on CMP1 (x2 on CMP2); CMP registers are
//! preloaded and transferred on the reset event (TxRSTU), which gives the k + 1 ZOH.
//!
//! **Faults** (hardware, no software in the path): TA ← FLT2 (COMP4), TB ← FLT6 (COMP5),
//! TC ← FLT3 (COMP6), TD ← FLT1 (PA12, external comparator); each comparator watches a
//! bridge pair (diode-OR of the two coil current amplifiers). A fault forces the
//! outputs inactive (OUTxR.FAULTy = 0b10) and latches in HRTIM_ISR until the core
//! commands `CoilCmd::Off`, which clears it; the fault configuration is locked.

use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::stm32::{
    HRTIM_COMMON, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD,
};

use super::{pins, reg};

/// Counts per PWM period (170 MHz / 20 kHz).
pub const PERIOD_COUNTS: u32 = 8500;
/// Fallback roll-over period (the TIM1 sync resets the counter first).
const PER: u32 = 8600;
/// Compare limits (RM0440 §27.3.4: CMP ≥ 3 at CKPSC ≥ 5; margin for the driver's
/// minimum pulse).
const CMP_MIN: u32 = 16;
const CMP_MAX: u32 = PERIOD_COUNTS - 16;

/// Timing unit register offsets (RM0440 §27.5.19–27.5.49, PAC hrtim_tima layout).
const T_CR: usize = 0x00;
const T_PER: usize = 0x14;
const T_CMP1: usize = 0x1C;
const T_CMP2: usize = 0x24;
const T_SET1: usize = 0x3C;
const T_RST1: usize = 0x40;
const T_SET2: usize = 0x44;
const T_RST2: usize = 0x48;
const T_OUT: usize = 0x64;
const T_FLT: usize = 0x68;

// TIMxCR.
const CKPSC_1: u32 = 0b101;
const CONT: u32 = 1 << 3;
const SYNCRST: u32 = 1 << 10;
const SYNCSTRT: u32 = 1 << 11;
const TRSTU: u32 = 1 << 18;
const PREEN: u32 = 1 << 27;
// SETx/RSTx event bits.
const EV_RESYNC: u32 = 1 << 1;
const EV_PER: u32 = 1 << 2;
const EV_CMP1: u32 = 1 << 3;
const EV_CMP2: u32 = 1 << 4;
// OUTxR: fault state inactive for both outputs.
const OUT_FAULT_INACTIVE: u32 = (0b10 << 4) | (0b10 << 20);
// FLTxR: lock.
const FLTLCK: u32 = 1 << 31;
// MCR.
const MCR_SYNCIN_TIM1: u32 = 0b10 << 8;
const MCR_TABCD_EN: u32 = (1 << 17) | (1 << 18) | (1 << 19) | (1 << 20);
// HRTIM_ISR / HRTIM_ICR fault flags FLT1, FLT2, FLT3, FLT6 (RM0440 §27.5.52).
const FLT_FLAGS: u32 = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 6);

/// Per timing unit: FLTxR enable bit (FLT1EN = bit 0 … FLT6EN = bit 5).
const UNIT_FAULT: [u32; 4] = [1 << 1, 1 << 5, 1 << 2, 1 << 0];

/// The coil bridge timer.
pub struct Hrtim {
    common: HRTIM_COMMON,
    _master: HRTIM_MASTER,
    _units: (HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD),
    /// Coils fitted (outputs 0..n).
    n: u8,
    enabled: bool,
}

impl Hrtim {
    /// Configures timing units A–D, the fault inputs and the TIM1 sync; outputs stay
    /// disabled. `n_magnets` (≤ 8) outputs are used.
    pub fn init(
        common: HRTIM_COMMON,
        master: HRTIM_MASTER,
        units: (HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD),
        n_magnets: u8,
        rcc: &Rcc,
    ) -> Hrtim {
        rcc.apb2enr().modify(|_, w| w.hrtim1en().set_bit());
        let bases = [
            HRTIM_TIMA::ptr() as usize,
            HRTIM_TIMB::ptr() as usize,
            HRTIM_TIMC::ptr() as usize,
            HRTIM_TIMD::ptr() as usize,
        ];
        // SAFETY: the HRTIM master, common block and timing units A–D are owned here
        // (moved in) and configured once in init with interrupts disabled. Offsets are
        // the RM0440 §27.5 timing-unit layout (identical for A–F, as the PAC register
        // blocks show); values are valid field encodings named above.
        unsafe {
            master.cr().write(|w| w.bits(MCR_SYNCIN_TIM1));
            for (base, flt) in bases.iter().zip(UNIT_FAULT) {
                reg::write(
                    base + T_CR,
                    CKPSC_1 | CONT | SYNCRST | SYNCSTRT | TRSTU | PREEN,
                );
                reg::write(base + T_PER, PER);
                reg::write(base + T_CMP1, PERIOD_COUNTS / 2);
                reg::write(base + T_CMP2, PERIOD_COUNTS / 2);
                reg::write(base + T_SET1, EV_RESYNC | EV_PER);
                reg::write(base + T_RST1, EV_CMP1);
                reg::write(base + T_SET2, EV_RESYNC | EV_PER);
                reg::write(base + T_RST2, EV_CMP2);
                reg::write(base + T_OUT, OUT_FAULT_INACTIVE);
                reg::write(base + T_FLT, flt | FLTLCK);
            }
            // Fault inputs (RM0440 §27.5.54–55): FLTxE | FLTxP (active high) | FLTxSRC
            // (1 = internal comparator) | FLTxLCK, one byte per input.
            let e_p = 0b11u32;
            let src_int = 1u32 << 2;
            let lck = 1u32 << 7;
            common.fltinr1().write(|w| {
                w.bits(
                    (e_p | lck) // FLT1: PA12, external comparator
                        | ((e_p | src_int | lck) << 8) // FLT2: COMP4
                        | ((e_p | src_int | lck) << 16), // FLT3: COMP6
                )
            });
            common
                .fltinr2()
                .write(|w| w.bits((e_p | src_int | lck) << 8)); // FLT6: COMP5
            common.odisr().write(|w| w.bits(0xFF));
            common.icr().write(|w| w.bits(FLT_FLAGS));
            master.cr().modify(|r, w| w.bits(r.bits() | MCR_TABCD_EN));
        }
        pins::COIL_GATE_EN.set(false);
        Hrtim {
            common,
            _master: master,
            _units: units,
            n: n_magnets.min(8),
            enabled: false,
        }
    }

    /// Sets the duty of coil `i` (0..n) as a fraction of the period, clamped to the
    /// minimum pulse; applied from the next PWM period.
    #[inline]
    pub fn set_duty(&mut self, i: usize, duty: f32) {
        if i >= self.n as usize {
            return;
        }
        let d = if duty.is_finite() {
            duty.clamp(0.0, 1.0)
        } else {
            0.5
        };
        let cmp = ((d * PERIOD_COUNTS as f32) as u32).clamp(CMP_MIN, CMP_MAX);
        let base = match i / 2 {
            0 => HRTIM_TIMA::ptr() as usize,
            1 => HRTIM_TIMB::ptr() as usize,
            2 => HRTIM_TIMC::ptr() as usize,
            _ => HRTIM_TIMD::ptr() as usize,
        };
        let off = if i.is_multiple_of(2) { T_CMP1 } else { T_CMP2 };
        // SAFETY: preloaded compare register of an owned timing unit; value in range.
        unsafe { reg::write(base + off, cmp) }
    }

    /// A bridge fault (hardware overcurrent) has latched since the last disable.
    #[inline]
    pub fn fault_latched(&self) -> bool {
        self.common.isr().read().bits() & FLT_FLAGS != 0
    }

    /// Outputs driving.
    #[inline]
    pub fn enabled(&self) -> bool {
        self.enabled && !self.fault_latched()
    }

    /// Enables outputs 0..n and the gate drivers; false (outputs stay off) while a
    /// fault is latched.
    pub fn enable(&mut self) -> bool {
        if self.fault_latched() {
            return false;
        }
        if !self.enabled {
            let mask = (1u32 << self.n) - 1;
            pins::COIL_GATE_EN.set(true);
            // SAFETY: OENR is a write-1-to-enable register of the owned HRTIM.
            unsafe { self.common.oenr().write(|w| w.bits(mask)) };
            self.enabled = true;
        }
        true
    }

    /// Disables every output and the gate drivers, and clears latched faults so a later
    /// enable can re-arm.
    pub fn disable(&mut self) {
        pins::COIL_GATE_EN.set(false);
        // SAFETY: ODISR/ICR are write-1 registers of the owned HRTIM.
        unsafe {
            self.common.odisr().write(|w| w.bits(0xFF));
            self.common.icr().write(|w| w.bits(FLT_FLAGS));
        }
        self.enabled = false;
    }
}
