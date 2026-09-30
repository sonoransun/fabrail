//! Hardware overcurrent trips: COMP1–7 against internal DAC3/DAC4 thresholds (RM0440
//! §24 COMP, §22 DAC). The comparator outputs reach the power stages without software:
//! COMP1–3 → TIM1 BKIN2 (LSM phases), COMP7 → TIM1 BKIN (DC bus), COMP4/COMP6/COMP5 →
//! HRTIM FLT2/FLT3/FLT6 (coil bridges 0–1, 4–5, 2–3), latency one comparator delay plus
//! the break/fault input path (DS12288 §5.3.24: tens of ns; the < 1 µs budget of
//! FAB-FW-012 is to be measured on hardware).
//!
//! **Threshold.** Every sensed current is scaled by its front end so the ADC full scale
//! equals the sense range (`SensorCal::*_a_per_code`, 12-bit signed about mid-scale
//! VREF+/2). All trips therefore share one normalised DAC code, [`TRIP_CODE`], just
//! inside the positive full scale: beyond it the software monitor can no longer see the
//! current, so the hardware takes over there. The comparators trip on the positive
//! excursion only (one comparator per signal); a negative LSM phase excursion appears as
//! a positive excursion on another phase (i_a + i_b + i_c = 0). This is a board-level
//! design statement to be verified at bring-up, not a measured property.
//!
//! Each comparator is locked after configuration (CSR.LOCK), so no later software can
//! disable a trip until reset.

use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::stm32::{COMP, DAC3, DAC4};

/// Common trip threshold, 12-bit DAC code (2048 mid-scale + 1984 = 96.9 % of the
/// positive sense range).
pub const TRIP_CODE: u32 = 4032;

/// COMPx_CSR fields (RM0440 §24.6.1).
const EN: u32 = 1 << 0;
const INMSEL_DAC: u32 = 0b100 << 4; // DAC3/DAC4 channel (comparator-specific, see below)
const INPSEL_1: u32 = 1 << 8;
const HYST_20MV: u32 = 0b010 << 16;
const LOCK: u32 = 1 << 31;

/// DAC_MCR / DAC_CR fields (RM0440 §22.7).
const MODE_INTERNAL: u32 = 0b011; // normal mode, buffer off, on-chip peripherals only
const HFSEL_160: u32 = 0b10 << 14; // AHB clock > 160 MHz
const DAC_EN1: u32 = 1 << 0;
const DAC_EN2: u32 = 1 << 16;
const DAC1RDY: u32 = 1 << 11;
const DAC2RDY: u32 = 1 << 27;

/// Inverting-input mapping with INMSEL = 0b100 (RM0440 Table 196; the same map as the
/// stm32g4xx-hal 0.1.0 comparator module): COMP1/COMP3 ← DAC3_CH1, COMP2/COMP4 ←
/// DAC3_CH2, COMP5/COMP7 ← DAC4_CH1, COMP6 ← DAC4_CH2. Non-inverting input selection
/// (INPSEL) per the pin map in [`super::pins`].
const COMP_CSR: [u32; 7] = [
    EN | INMSEL_DAC | HYST_20MV, // COMP1: PA1 (phase A) vs DAC3_CH1
    EN | INMSEL_DAC | INPSEL_1 | HYST_20MV, // COMP2: PA3 (phase B) vs DAC3_CH2
    EN | INMSEL_DAC | HYST_20MV, // COMP3: PA0 (phase C) vs DAC3_CH1
    EN | INMSEL_DAC | INPSEL_1 | HYST_20MV, // COMP4: PE7 (coils 0–1) vs DAC3_CH2
    EN | INMSEL_DAC | INPSEL_1 | HYST_20MV, // COMP5: PD12 (coils 2–3) vs DAC4_CH1
    EN | INMSEL_DAC | HYST_20MV, // COMP6: PB11 (coils 4–5) vs DAC4_CH2
    EN | INMSEL_DAC | INPSEL_1 | HYST_20MV, // COMP7: PD14 (DC bus) vs DAC4_CH1
];

/// Configures DAC3/DAC4 at [`TRIP_CODE`] and enables and locks COMP1–7. Returns false
/// if a DAC channel did not report ready (the caller treats it as a boot fault).
pub fn init(comp: &COMP, dac3: &DAC3, dac4: &DAC4, rcc: &Rcc) -> bool {
    // COMP is clocked by SYSCFG (RM0440 §24.3.2); DAC3/DAC4 on AHB2.
    rcc.apb2enr().modify(|_, w| w.syscfgen().set_bit());
    rcc.ahb2enr().modify(|_, w| {
        w.dac3en().set_bit();
        w.dac4en().set_bit()
    });
    let mut ok = true;
    for dac in [&**dac3, &**dac4] {
        // SAFETY: DAC3/DAC4 are owned by the caller and configured once in init with
        // interrupts disabled; the field values are valid encodings per RM0440 §22.7.
        unsafe {
            dac.mcr()
                .write(|w| w.bits(HFSEL_160 | MODE_INTERNAL | (MODE_INTERNAL << 16)));
            dac.dhr12r1().write(|w| w.bits(TRIP_CODE));
            dac.dhr12r2().write(|w| w.bits(TRIP_CODE));
            dac.cr().write(|w| w.bits(DAC_EN1 | DAC_EN2));
        }
        // tWAKEUP ≤ 8 µs (DS12288 Table 70); bounded poll of the ready flags.
        let mut ready = false;
        for _ in 0..20_000u32 {
            let sr = dac.sr().read().bits();
            if sr & DAC1RDY != 0 && sr & DAC2RDY != 0 {
                ready = true;
                break;
            }
        }
        ok &= ready;
    }
    for (i, csr) in COMP_CSR.iter().enumerate() {
        // SAFETY: COMP is owned by the caller; each CSR is written once with a valid
        // configuration, then locked (a second write with LOCK set is the documented
        // locking procedure, RM0440 §24.3.9).
        unsafe {
            comp.ccsr(i).write(|w| w.bits(*csr));
            comp.ccsr(i).write(|w| w.bits(*csr | LOCK));
        }
    }
    ok
}
