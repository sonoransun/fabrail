//! ADC1–ADC5 (RM0440 §21) at PAC level: stm32g4xx-hal 0.1.0 has no injected-group
//! support (core design §F), and all five converters share one clock and trigger plan.
//!
//! | Converter | Group | Trigger | Channels (ADC input numbers) | Use |
//! |---|---|---|---|---|
//! | ADC1 + ADC2 | injected, dual simultaneous | TIM1_TRGO (PWM peak) | ADC1: IN2, IN4; ADC2: IN1, IN12 | LSM phase A/B/C currents, DC-bus current |
//! | ADC3 | injected | TIM1_TRGO | IN1–IN4 | coil currents 0–3 |
//! | ADC4 | injected | TIM1_TRGO | IN1–IN4 | coil currents 4–7 |
//! | ADC5 | regular, continuous, DMA1 CH3 | — | IN1–IN8 | primary gap probes (16× oversampled → 16-bit) |
//! | ADC3 | regular, continuous, DMA1 CH4 | — | IN5–IN12 | redundant gap probes (16-bit) |
//! | ADC4 | regular, continuous, DMA1 CH5 | — | IN5–IN9 | V_bus, V_hold, T_coil, T_magnet, handle force (12-bit averaged) |
//!
//! Channel numbers are fixed by the board design (DS12288 Table 13 pins; to verify at
//! bring-up). Clock: synchronous HCLK/4 = 42.5 MHz (CKMODE = 11), so conversions keep a
//! fixed phase to the TIM1 trigger. Current channels use the hardware offset (OFRx =
//! 2048, VREF+/2 bias of the bipolar sense amplifiers), so the injected data registers
//! hold the signed 12-bit codes the HAL contract expects (`CurrentRaw`, `LsmRaw`).
//!
//! **Base-tick timing.** The base task is the ADC1_2 interrupt (ADC1 end of injected
//! sequence). ADC1/ADC2 sample for 47.5 cycles: 2 × (47.5 + 12.5)/42.5 MHz = 2.8 µs,
//! longer than the four ADC3/ADC4 conversions at 12.5 cycles (4 × 25/42.5 MHz =
//! 2.4 µs), so every current of a PWM period is converted when the interrupt fires; the
//! ISR also checks the ADC3/ADC4 end-of-sequence flags ([`Adc::ack_injected`]).

use core::ptr::{addr_of, addr_of_mut};

use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::stm32::{
    ADC1, ADC2, ADC3, ADC4, ADC5, ADC12_COMMON, ADC345_COMMON, DMA1, DMAMUX, adc1,
};

/// ADC1 injected channels: phase A, phase B.
const ADC1_J: [u32; 2] = [2, 4];
/// ADC2 injected channels: phase C, DC-bus current.
const ADC2_J: [u32; 2] = [1, 12];
/// ADC3 injected channels: coil currents 0–3.
const ADC3_J: [u32; 4] = [1, 2, 3, 4];
/// ADC4 injected channels: coil currents 4–7.
const ADC4_J: [u32; 4] = [1, 2, 3, 4];
/// ADC5 regular channels: primary gap probes 0–7.
const ADC5_R: [u32; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
/// ADC3 regular channels: redundant gap probes 0–7.
const ADC3_R: [u32; 8] = [5, 6, 7, 8, 9, 10, 11, 12];
/// ADC4 regular channels: V_bus, V_hold, T_coil, T_magnet, handle force.
const ADC4_R: [u32; 5] = [5, 6, 7, 8, 9];

/// Sampling-time codes (RM0440 §21.7.6): 001 = 6.5, 010 = 12.5, 100 = 47.5 cycles.
const SMP_6_5: u32 = 0b001;
const SMP_12_5: u32 = 0b010;
const SMP_47_5: u32 = 0b100;

// CR.
const ADEN: u32 = 1 << 0;
const ADSTART: u32 = 1 << 2;
const JADSTART: u32 = 1 << 3;
const ADVREGEN: u32 = 1 << 28;
const ADCAL: u32 = 1 << 31;
// ISR / IER.
const ADRDY: u32 = 1 << 0;
const OVR: u32 = 1 << 4;
const JEOC: u32 = 1 << 5;
const JEOS: u32 = 1 << 6;
// CFGR.
const DMAEN: u32 = 1 << 0;
const DMACFG_CIRC: u32 = 1 << 1;
const OVRMOD: u32 = 1 << 12;
const CONT: u32 = 1 << 13;
const JQDIS: u32 = 1 << 31;
// CFGR2: regular oversampling, ratio 16 (OVSR = 011), shift 0 (16-bit sum) or 4 (12-bit
// average).
const ROVSE: u32 = 1 << 0;
const OVSR_16: u32 = 0b011 << 2;
const OVSS_4: u32 = 0b0100 << 5;
// JSQR: JEXTEN rising (01), JEXTSEL 0 = TIM1_TRGO (ADC12 and ADC345 injected tables).
const JEXTEN_RISING: u32 = 0b01 << 7;
// OFRx: OFFSET_EN, OFFSET = 2048.
const OFFSET_EN: u32 = 1 << 31;
const OFFSET_MID: u32 = 2048;
// Common CCR: CKMODE = 11 (HCLK/4); DUAL = 00101 (injected simultaneous only).
const CKMODE_HCLK4: u32 = 0b11 << 16;
const DUAL_INJ_SIMULT: u32 = 0b00101;

// DMA channel CR (RM0440 §12.6.3): 16-bit peripheral and memory, MINC, circular.
const DMA_EN: u32 = 1 << 0;
const DMA_CIRC: u32 = 1 << 5;
const DMA_MINC: u32 = 1 << 7;
const DMA_16BIT: u32 = (0b01 << 8) | (0b01 << 10);
// DMAMUX request IDs (RM0440 Table 91).
const REQ_ADC3: u32 = 37;
const REQ_ADC4: u32 = 38;
const REQ_ADC5: u32 = 39;

/// DMA targets (SRAM1: `.bss`; DMA never targets the stack or CCM).
static mut GAP_A: [u16; 8] = [0; 8];
static mut GAP_B: [u16; 8] = [0; 8];
static mut POWER: [u16; 5] = [0; 5];

/// One PWM period's current samples (signed codes, mid-scale removed in hardware).
#[derive(Clone, Copy, Debug, Default)]
pub struct Currents {
    /// LSM phase currents a, b, c.
    pub phase: [i16; 3],
    /// Coil currents 0..7.
    pub coil: [i16; 8],
}

/// Supervisor-rate power samples (12-bit codes).
#[derive(Clone, Copy, Debug, Default)]
pub struct PowerCodes {
    /// DC bus voltage.
    pub v_bus: u16,
    /// Hold-up (supercap) voltage.
    pub v_hold: u16,
    /// Coil hot-spot and magnet temperatures.
    pub temp: [u16; 2],
    /// Handle load cell (unsigned, mid-scale = 0 N).
    pub handle: u16,
    /// DC-bus current (signed, mid-scale removed).
    pub i_bus: i16,
}

/// The five converters and their DMA channels.
pub struct Adc {
    adc1: ADC1,
    adc2: ADC2,
    adc3: ADC3,
    adc4: ADC4,
    _adc5: ADC5,
    /// ADC3 runs the redundant gap scan.
    adc3_regular: bool,
    /// Injected sequences whose end-of-sequence flag was missing at the base interrupt.
    pub late_sequences: u32,
}

fn smpr_bits(ch: u32, smp: u32) -> (bool, u32) {
    if ch < 10 {
        (false, smp << (3 * ch))
    } else {
        (true, smp << (3 * (ch - 10)))
    }
}

/// Powers up, calibrates (single-ended) and enables one converter; false on timeout.
fn power_up(adc: &adc1::RegisterBlock) -> bool {
    // SAFETY: the converter is owned by the caller and still disabled; the sequence
    // DEEPPWD = 0 → ADVREGEN = 1 → t_ADCVREG_STUP (20 µs, DS12288 Table 66) → ADCAL is
    // RM0440 §21.4.6–21.4.8.
    unsafe {
        adc.cr().write(|w| w.bits(0));
        adc.cr().write(|w| w.bits(ADVREGEN));
    }
    cortex_m::asm::delay(4_000); // ≥ 20 µs at 170 MHz
    // SAFETY: as above.
    unsafe { adc.cr().write(|w| w.bits(ADVREGEN | ADCAL)) };
    let mut ok = false;
    for _ in 0..200_000u32 {
        if adc.cr().read().bits() & ADCAL == 0 {
            ok = true;
            break;
        }
    }
    cortex_m::asm::delay(100); // ADEN ≥ 4 ADC clocks after ADCAL clears
    // SAFETY: ISR.ADRDY is rc_w1; CR.ADEN starts the converter (RM0440 §21.4.9).
    unsafe {
        adc.isr().write(|w| w.bits(ADRDY));
        adc.cr().write(|w| w.bits(ADVREGEN | ADEN));
    }
    let mut ready = false;
    for _ in 0..200_000u32 {
        if adc.isr().read().bits() & ADRDY != 0 {
            ready = true;
            break;
        }
    }
    ok && ready
}

/// Injected sequence with sample time `smp`, hardware mid-scale offsets on every
/// channel (≤ 4), and the TIM1_TRGO trigger when `trigger` (the dual-mode slave takes
/// the master's trigger).
fn config_injected(adc: &adc1::RegisterBlock, chans: &[u32], smp: u32, trigger: bool) {
    let n = chans.len().clamp(1, 4) as u32;
    let mut jsqr = (n - 1) | if trigger { JEXTEN_RISING } else { 0 };
    for (k, ch) in chans.iter().take(4).enumerate() {
        let shift = [9u32, 15, 21, 27][k];
        jsqr |= (ch & 0x1F) << shift;
    }
    // SAFETY: owned, enabled converter with ADSTART = JADSTART = 0 (writable state per
    // RM0440 §21.7); encodings per §21.7.4 (SMPR), §21.7.11 (JSQR), §21.7.12 (OFRy).
    unsafe {
        for ch in chans.iter().take(4) {
            let (hi, bits) = smpr_bits(*ch, smp);
            let mask = smpr_bits(*ch, 0b111).1;
            if hi {
                adc.smpr2().modify(|r, w| w.bits((r.bits() & !mask) | bits));
            } else {
                adc.smpr1().modify(|r, w| w.bits((r.bits() & !mask) | bits));
            }
        }
        adc.jsqr().write(|w| w.bits(jsqr));
        for (k, ch) in chans.iter().take(4).enumerate() {
            let ofr = OFFSET_EN | ((ch & 0x1F) << 26) | OFFSET_MID;
            match k {
                0 => adc.ofr1().write(|w| w.bits(ofr)),
                1 => adc.ofr2().write(|w| w.bits(ofr)),
                2 => adc.ofr3().write(|w| w.bits(ofr)),
                _ => adc.ofr4().write(|w| w.bits(ofr)),
            };
        }
    }
}

/// A circular DMA destination for one regular scan.
struct DmaTarget<'a> {
    /// DMA channel.
    ch: &'a stm32g4xx_hal::stm32::dma1::CH,
    /// Its DMAMUX channel control register.
    mux: &'a stm32g4xx_hal::stm32::dmamux::CCR,
    /// DMAMUX request ID of the converter.
    req: u32,
    /// Static buffer of at least as many u16 as channels.
    buf: *mut u16,
}

/// Continuous regular scan into a circular DMA buffer, 16× oversampled.
fn config_regular(
    adc: &adc1::RegisterBlock,
    chans: &[u32],
    smp: u32,
    average: bool,
    t: DmaTarget<'_>,
) {
    let DmaTarget {
        ch: dma,
        mux: dmamux_ccr,
        req,
        buf,
    } = t;
    let n = chans.len().clamp(1, 8) as u32;
    let mut sqr1 = n - 1;
    let mut sqr2 = 0u32;
    for (k, ch) in chans.iter().take(8).enumerate() {
        let ch = ch & 0x1F;
        match k {
            0..=3 => sqr1 |= ch << (6 + 6 * k as u32),
            _ => sqr2 |= ch << (6 * (k as u32 - 4)),
        }
    }
    let cfgr2 = ROVSE | OVSR_16 | if average { OVSS_4 } else { 0 };
    // SAFETY: owned converter (ADSTART = 0) and owned DMA channel (disabled); the DMA
    // source is the converter's DR and the target is a static buffer of ≥ n u16 that
    // only this DMA channel writes (encodings RM0440 §12.6, §13.6, §21.7).
    unsafe {
        for ch in chans.iter().take(8) {
            let (hi, bits) = smpr_bits(*ch, smp);
            let mask = smpr_bits(*ch, 0b111).1;
            if hi {
                adc.smpr2().modify(|r, w| w.bits((r.bits() & !mask) | bits));
            } else {
                adc.smpr1().modify(|r, w| w.bits((r.bits() & !mask) | bits));
            }
        }
        adc.sqr1().write(|w| w.bits(sqr1));
        adc.sqr2().write(|w| w.bits(sqr2));
        adc.cfgr2().write(|w| w.bits(cfgr2));
        adc.cfgr()
            .write(|w| w.bits(JQDIS | CONT | OVRMOD | DMACFG_CIRC | DMAEN));
        dma.cr().write(|w| w.bits(0));
        dmamux_ccr.write(|w| w.bits(req));
        dma.par().write(|w| w.bits(adc.dr().as_ptr() as u32));
        dma.mar().write(|w| w.bits(buf as u32));
        dma.ndtr().write(|w| w.bits(n));
        dma.cr()
            .write(|w| w.bits(DMA_16BIT | DMA_MINC | DMA_CIRC | DMA_EN));
    }
}

impl Adc {
    /// Configures all converters; returns the driver and whether every converter came
    /// up (false = boot fault). `n_gaps` primary probes (≤ 8), redundant probes when
    /// `redundant`.
    pub fn init(
        adcs: (ADC1, ADC2, ADC3, ADC4, ADC5),
        common: (ADC12_COMMON, ADC345_COMMON),
        dma1: &DMA1,
        dmamux: &DMAMUX,
        n_gaps: u8,
        redundant: bool,
        rcc: &Rcc,
    ) -> (Adc, bool) {
        let (adc1, adc2, adc3, adc4, adc5) = adcs;
        let (c12, c345) = common;
        rcc.ahb2enr().modify(|_, w| {
            w.adc12en().set_bit();
            w.adc345en().set_bit()
        });
        rcc.ahb1enr().modify(|_, w| {
            w.dma1en().set_bit();
            w.dmamux1en().set_bit()
        });
        // SAFETY: common registers written while every converter is disabled (RM0440
        // §21.7.21), valid encodings named above.
        unsafe {
            c12.ccr().write(|w| w.bits(CKMODE_HCLK4 | DUAL_INJ_SIMULT));
            c345.ccr().write(|w| w.bits(CKMODE_HCLK4));
        }
        let mut ok = true;
        for adc in [&*adc1, &*adc2, &*adc3, &*adc4, &*adc5] {
            ok &= power_up(adc);
        }
        let n = n_gaps.clamp(1, 8) as usize;
        config_injected(&adc1, &ADC1_J, SMP_47_5, true);
        config_injected(&adc2, &ADC2_J, SMP_47_5, false);
        config_injected(&adc3, &ADC3_J, SMP_12_5, true);
        config_injected(&adc4, &ADC4_J, SMP_12_5, true);
        config_regular(
            &adc5,
            ADC5_R.get(..n).unwrap_or(&ADC5_R),
            SMP_6_5,
            false,
            DmaTarget {
                ch: dma1.ch3(),
                mux: dmamux.ccr(2),
                req: REQ_ADC5,
                buf: addr_of_mut!(GAP_A).cast::<u16>(),
            },
        );
        if redundant {
            config_regular(
                &adc3,
                ADC3_R.get(..n).unwrap_or(&ADC3_R),
                SMP_6_5,
                false,
                DmaTarget {
                    ch: dma1.ch4(),
                    mux: dmamux.ccr(3),
                    req: REQ_ADC3,
                    buf: addr_of_mut!(GAP_B).cast::<u16>(),
                },
            );
        }
        config_regular(
            &adc4,
            &ADC4_R,
            SMP_47_5,
            true,
            DmaTarget {
                ch: dma1.ch5(),
                mux: dmamux.ccr(4),
                req: REQ_ADC4,
                buf: addr_of_mut!(POWER).cast::<u16>(),
            },
        );
        // SAFETY: interrupt enable of the owned ADC1 (end of injected sequence only).
        unsafe { adc1.ier().write(|w| w.bits(JEOS)) };
        (
            Adc {
                adc1,
                adc2,
                adc3,
                adc4,
                _adc5: adc5,
                adc3_regular: redundant,
                late_sequences: 0,
            },
            ok,
        )
    }

    /// Starts the continuous regular scans (gap probes, power sense).
    pub fn start_regular(&mut self) {
        let adc5 = stm32g4xx_hal::stm32::ADC5::ptr();
        // SAFETY: ADSTART on owned, enabled converters (ADC5 is owned by this driver
        // through `_adc5`; its pointer only reaches the same registers).
        unsafe {
            for adc in [&*self.adc4, &*adc5] {
                adc.cr().modify(|r, w| w.bits(r.bits() | ADSTART));
            }
            if self.adc3_regular {
                self.adc3.cr().modify(|r, w| w.bits(r.bits() | ADSTART));
            }
        }
    }

    /// Arms the injected triggers: the next TIM1_TRGO starts the 20 kHz base interrupt.
    pub fn arm_injected(&mut self) {
        // SAFETY: JADSTART on owned, enabled converters; ADC1 is the dual-mode master,
        // so its JADSTART arms ADC1 and ADC2 (RM0440 §21.4.31).
        unsafe {
            for adc in [&*self.adc1, &*self.adc3, &*self.adc4] {
                adc.cr().modify(|r, w| w.bits(r.bits() | JADSTART));
            }
        }
    }

    /// Acknowledges the base interrupt (ADC1 end of injected sequence) and the ADC3/4
    /// sequences. Counts a late ADC3/ADC4 sequence in [`Adc::late_sequences`].
    #[inline]
    pub fn ack_injected(&mut self) {
        let late = [&*self.adc3, &*self.adc4]
            .iter()
            .any(|a| a.isr().read().bits() & JEOS == 0);
        if late {
            self.late_sequences = self.late_sequences.wrapping_add(1);
        }
        // SAFETY: ISR flags are rc_w1; clearing JEOS/JEOC/OVR of owned converters.
        unsafe {
            self.adc1.isr().write(|w| w.bits(JEOS | JEOC | OVR));
            self.adc3.isr().write(|w| w.bits(JEOS | JEOC));
            self.adc4.isr().write(|w| w.bits(JEOS | JEOC));
        }
    }

    /// The injected results of this PWM period.
    #[inline]
    pub fn currents(&self) -> Currents {
        let j = |a: &adc1::RegisterBlock, k: usize| -> i16 {
            let v = match k {
                0 => a.jdr1().read().bits(),
                1 => a.jdr2().read().bits(),
                2 => a.jdr3().read().bits(),
                _ => a.jdr4().read().bits(),
            };
            v as u16 as i16
        };
        let mut c = Currents {
            phase: [j(&self.adc1, 0), j(&self.adc1, 1), j(&self.adc2, 0)],
            coil: [0; 8],
        };
        for k in 0..4 {
            if let Some(x) = c.coil.get_mut(k) {
                *x = j(&self.adc3, k);
            }
            if let Some(x) = c.coil.get_mut(k + 4) {
                *x = j(&self.adc4, k);
            }
        }
        c
    }
}

/// Latest gap codes (primary, redundant) from the DMA buffers.
#[inline]
pub fn gap_codes() -> ([u16; 8], [u16; 8]) {
    // SAFETY: GAP_A/GAP_B are written only by their DMA channels; aligned u16 reads are
    // single bus accesses, so each element is a complete conversion result.
    unsafe {
        (
            core::ptr::read_volatile(addr_of!(GAP_A)),
            core::ptr::read_volatile(addr_of!(GAP_B)),
        )
    }
}

/// Latest power codes (supervisor group). The DC-bus current is ADC2's second injected
/// result, read without side effects from the register (the base task owns ADC2).
#[inline]
pub fn power_codes() -> PowerCodes {
    // SAFETY: POWER is written only by DMA1 CH5 (element-atomic u16 reads); JDR2 of
    // ADC2 is a read-only data register without read side effects.
    let (p, i_bus) = unsafe {
        (
            core::ptr::read_volatile(addr_of!(POWER)),
            (*ADC2::ptr()).jdr2().read().bits() as u16 as i16,
        )
    };
    PowerCodes {
        v_bus: p[0],
        v_hold: p[1],
        temp: [p[2], p[3]],
        handle: p[4],
        i_bus,
    }
}
