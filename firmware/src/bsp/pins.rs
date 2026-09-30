//! Pin map and GPIO register work (RM0440 §9 "GPIO"; DS12288 Table 13 alternate
//! functions, cross-checked against the stm32g4xx-hal 0.1.0 pin tables).
//!
//! Digital functions of the LQFP64 STM32G474RE. Analog inputs are selected by ADC
//! channel number in [`super::adc`] (GPIOs reset to analog mode, RM0440 §9.4.1, so
//! analog pins need no configuration here).
//!
//! | Pin | Function | Mode |
//! |---|---|---|
//! | PA15, PB3 | TIM2_CH1/CH2 encoder A/B | AF1 |
//! | PC3 | encoder reference mark (active low) | input, pull-up |
//! | PA7, PB0, PB1 | TIM1_CH1N/CH2N/CH3N (LSM low sides) | AF6 |
//! | PC0, PC1, PC2 | TIM1_CH1/CH2/CH3 (LSM high sides) | AF2 |
//! | PA8–PA11, PB12–PB15 | HRTIM CHA1..CHD2 = coil bridges 0..7 | AF13 |
//! | PA12 | HRTIM_FLT1 (external comparator, coil bridges 6–7) | AF13 |
//! | PA1, PA3, PA0 | COMP1/COMP2/COMP3 non-inverting inputs (phase A/B/C current) | analog |
//! | PB11 | COMP6 non-inverting input (coil bridges 4–5 current) | analog |
//! | PE7, PD12, PD14 | COMP4/COMP5/COMP7 inputs (coil 0–1, coil 2–3, DC bus) † | analog |
//! | PA5, PA6, PB5 | SPI1 SCK/MISO/MOSI (ADXL355) | AF5 |
//! | PB4 | ADXL355 chip select (idle high) | output |
//! | PC6 | ADXL355 DRDY (EXTI6, rising) | input |
//! | PC10, PC11, PC12 | SPI3 SCK/MISO/MOSI (carriage IMU, rail accelerometer) | AF6 |
//! | PB6, PB7 | IMU / rail accelerometer chip selects (idle high) | output |
//! | PB8, PB9 | FDCAN1 RX/TX | AF9 |
//! | PB2 | coil bridge gate enable (hardware pull-down: reset = disabled) | output, low |
//! | PB10 | LSM inverter gate enable (hardware pull-down) | output, low |
//! | PC4 | coil gate-driver nFAULT | input, pull-up |
//! | PD2 | LSM gate-driver nFAULT | input, pull-up |
//! | PC7, PC8 | presence device OSSD healthy / person in zone | input, pull-down |
//! | PC9 | E-stop loop (high = released; broken wire reads pressed) | input, pull-down |
//! | PC13 | building fire alarm (active low; broken wire reads alarm) | input, pull-down |
//! | PC14, PC15 | parking-lock engaged switch / hold-to-run button | input, pull-down |
//! | PA2 | mains present (high = present; open circuit reads mains lost) | input, pull-down |
//! | PA4, PC5 | parking-lock / seismic-latch solenoid drivers | output, low |
//!
//! † Ports D/E beyond PD2 exist only on the LQFP100/128 packages (STM32G474VE/QE: same
//! die, memory map, PAC and firmware image as the RE). The full analog front end
//! (≈ 30 ADC inputs: 8 coil currents, 3 phase currents, 8 + 8 gap probes, power sense)
//! needs such a package; the LQFP64 NUCLEO-G474RE bring-up kit carries the digital
//! functions above and a DOOR-sized analog subset. Correction to core design §F
//! ("FLT ← COMP4–7"): COMP7 has no HRTIM fault path on the G474 (the fault-input map is
//! FLT1←COMP2, FLT2←COMP4, FLT3←COMP6, FLT4←COMP1, FLT5←COMP3, FLT6←COMP5), so COMP7
//! trips the LSM on TIM1 BKIN (DC-bus overcurrent) and bridges 6–7 use FLT1 from an
//! external comparator on PA12.
//!
//! PC13–PC15 are low-drive backup-domain pins (DS12288 §5.3.14), used as inputs only.

use super::reg;

/// GPIO port (index into the 0x400-spaced port blocks at 0x4800_0000, RM0440 §2.2.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Port {
    /// GPIOA.
    A = 0,
    /// GPIOB.
    B = 1,
    /// GPIOC.
    C = 2,
    /// GPIOD.
    D = 3,
}

/// One GPIO pin. Fields are private: every `Pin` is one of the constants below (pin
/// numbers 0..=15), which keeps the register accessors sound.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pin {
    port: Port,
    n: u8,
}

const fn pin(port: Port, n: u8) -> Pin {
    Pin { port, n }
}

/// GPIO port register offsets (RM0440 §9.4).
const MODER: usize = 0x00;
const OTYPER: usize = 0x04;
const OSPEEDR: usize = 0x08;
const PUPDR: usize = 0x0C;
const IDR: usize = 0x10;
const BSRR: usize = 0x18;
const AFRL: usize = 0x20;
const AFRH: usize = 0x24;
/// GPIOA base; ports are 0x400 apart (RM0440 Table 4).
const GPIO_BASE: usize = 0x4800_0000;

impl Pin {
    #[inline(always)]
    const fn base(self) -> usize {
        GPIO_BASE + 0x400 * self.port as usize
    }

    /// Input data register level.
    #[inline(always)]
    pub fn is_high(self) -> bool {
        // SAFETY: IDR of an existing GPIO port (RM0440 §9.4.5), read-only access.
        unsafe { reg::read(self.base() + IDR) & (1 << self.n) != 0 }
    }

    /// Drives the output high or low through BSRR (atomic set/reset, RM0440 §9.4.7), so
    /// no read-modify-write race exists between contexts.
    #[inline(always)]
    pub fn set(self, high: bool) {
        let bit = if high {
            1u32 << self.n
        } else {
            1u32 << (self.n + 16)
        };
        // SAFETY: BSRR of an existing GPIO port; write-only set/reset register whose
        // only effect is this pin's output latch.
        unsafe { reg::write(self.base() + BSRR, bit) }
    }
}

// ---- actuator enables and fault lines ---------------------------------------------

/// Coil H-bridge gate enable (active high; hardware pull-down, so reset = disabled).
pub const COIL_GATE_EN: Pin = pin(Port::B, 2);
/// LSM inverter gate enable (active high; hardware pull-down).
pub const LSM_GATE_EN: Pin = pin(Port::B, 10);
/// Coil gate-driver fault (active low, open drain).
pub const COIL_NFAULT: Pin = pin(Port::C, 4);
/// LSM gate-driver fault (active low, open drain).
pub const LSM_NFAULT: Pin = pin(Port::D, 2);

// ---- sensors -----------------------------------------------------------------------

/// ADXL355 chip select (SPI1).
pub const ADXL_CS: Pin = pin(Port::B, 4);
/// ADXL355 DRDY (EXTI line 6).
pub const ADXL_DRDY: Pin = pin(Port::C, 6);
/// Carriage IMU chip select (SPI3).
pub const IMU_CS: Pin = pin(Port::B, 6);
/// Rail accelerometer chip select (SPI3).
pub const RAIL_CS: Pin = pin(Port::B, 7);
/// Encoder reference mark (active low).
pub const ENC_REF: Pin = pin(Port::C, 3);

// ---- safety inputs and aux outputs ------------------------------------------------

/// Presence-sensing device OSSD pair healthy.
pub const PRESENCE_OK: Pin = pin(Port::C, 7);
/// Presence-sensing device: person in the zone.
pub const PRESENCE_ACTIVE: Pin = pin(Port::C, 8);
/// E-stop loop (high = released).
pub const ESTOP_LOOP: Pin = pin(Port::C, 9);
/// Building fire alarm (high = no alarm).
pub const FIRE_ALARM_N: Pin = pin(Port::C, 13);
/// Parking lock engaged feedback switch.
pub const LOCK_ENGAGED: Pin = pin(Port::C, 14);
/// Hold-to-run control actuated.
pub const HOLD_TO_RUN: Pin = pin(Port::C, 15);
/// Mains present (high).
pub const MAINS_OK: Pin = pin(Port::A, 2);
/// Parking-lock solenoid driver.
pub const PARKING_LOCK: Pin = pin(Port::A, 4);
/// Seismic x-latch solenoid driver.
pub const SEISMIC_LATCH: Pin = pin(Port::C, 5);

#[derive(Clone, Copy)]
enum Pull {
    None = 0b00,
    Up = 0b01,
    Down = 0b10,
}

#[derive(Clone, Copy)]
enum Mode {
    Input(Pull),
    /// Push-pull output, initial level low.
    OutputLow,
    /// Push-pull output, initial level high (chip selects).
    OutputHigh,
    /// Alternate function, push-pull, very high speed.
    Af(u8),
}

/// The complete digital pin configuration, applied in order by [`configure`]. Gate
/// enables come first so the power stages are held disabled before anything else.
const PINS: &[(Pin, Mode)] = &[
    (COIL_GATE_EN, Mode::OutputLow),
    (LSM_GATE_EN, Mode::OutputLow),
    (PARKING_LOCK, Mode::OutputLow),
    (SEISMIC_LATCH, Mode::OutputLow),
    (COIL_NFAULT, Mode::Input(Pull::Up)),
    (LSM_NFAULT, Mode::Input(Pull::Up)),
    // TIM2 encoder.
    (pin(Port::A, 15), Mode::Af(1)),
    (pin(Port::B, 3), Mode::Af(1)),
    (ENC_REF, Mode::Input(Pull::Up)),
    // TIM1 LSM inverter.
    (pin(Port::C, 0), Mode::Af(2)),
    (pin(Port::C, 1), Mode::Af(2)),
    (pin(Port::C, 2), Mode::Af(2)),
    (pin(Port::A, 7), Mode::Af(6)),
    (pin(Port::B, 0), Mode::Af(6)),
    (pin(Port::B, 1), Mode::Af(6)),
    // HRTIM coil bridges 0..7.
    (pin(Port::A, 8), Mode::Af(13)),
    (pin(Port::A, 9), Mode::Af(13)),
    (pin(Port::A, 10), Mode::Af(13)),
    (pin(Port::A, 11), Mode::Af(13)),
    (pin(Port::B, 12), Mode::Af(13)),
    (pin(Port::B, 13), Mode::Af(13)),
    (pin(Port::B, 14), Mode::Af(13)),
    (pin(Port::B, 15), Mode::Af(13)),
    // HRTIM_FLT1 (external comparator for bridges 6–7).
    (pin(Port::A, 12), Mode::Af(13)),
    // SPI1 ADXL355.
    (ADXL_CS, Mode::OutputHigh),
    (pin(Port::A, 5), Mode::Af(5)),
    (pin(Port::A, 6), Mode::Af(5)),
    (pin(Port::B, 5), Mode::Af(5)),
    (ADXL_DRDY, Mode::Input(Pull::None)),
    // SPI3 carriage IMU and rail accelerometer.
    (IMU_CS, Mode::OutputHigh),
    (RAIL_CS, Mode::OutputHigh),
    (pin(Port::C, 10), Mode::Af(6)),
    (pin(Port::C, 11), Mode::Af(6)),
    (pin(Port::C, 12), Mode::Af(6)),
    // FDCAN1.
    (pin(Port::B, 8), Mode::Af(9)),
    (pin(Port::B, 9), Mode::Af(9)),
    // Safety inputs: pull-downs so an open circuit reads the unsafe level.
    (PRESENCE_OK, Mode::Input(Pull::Down)),
    (PRESENCE_ACTIVE, Mode::Input(Pull::Down)),
    (ESTOP_LOOP, Mode::Input(Pull::Down)),
    (FIRE_ALARM_N, Mode::Input(Pull::Down)),
    (LOCK_ENGAGED, Mode::Input(Pull::Down)),
    (HOLD_TO_RUN, Mode::Input(Pull::Down)),
    (MAINS_OK, Mode::Input(Pull::Down)),
];

/// Holds both power stages disabled: gate enables low. Callable from any context,
/// including the fault handlers (BSRR writes are atomic and need no ownership), and
/// before [`configure`] (the pins are then still inputs with the hardware pull-downs
/// holding the drivers off, and the output latch is preset low).
pub fn force_gates_off() {
    COIL_GATE_EN.set(false);
    LSM_GATE_EN.set(false);
}

/// PWR_CR3.UCPD1_DBDIS (RM0440 §6.4.3): disables the USB Type-C dead-battery
/// pull-downs that the G4 applies to PB4 and PB6 (UCPD1 CC2/CC1) after reset; both pins
/// are chip-select outputs here.
const PWR_CR3_UCPD1_DBDIS: u32 = 1 << 14;

/// Enables the GPIO clocks and applies [`PINS`].
pub fn configure(rcc: &stm32g4xx_hal::rcc::Rcc) {
    rcc.apb1enr1().modify(|_, w| w.pwren().set_bit());
    // SAFETY: a single bit set in PWR_CR3 during init (interrupts disabled); the HAL's
    // PWR configuration touches CR1/CR5 only, so no other owner writes CR3.
    unsafe {
        (*stm32g4xx_hal::stm32::PWR::ptr())
            .cr3()
            .modify(|r, w| w.bits(r.bits() | PWR_CR3_UCPD1_DBDIS));
    }
    rcc.ahb2enr().modify(|_, w| {
        w.gpioaen().set_bit();
        w.gpioben().set_bit();
        w.gpiocen().set_bit();
        w.gpioden().set_bit()
    });
    for &(p, mode) in PINS {
        apply(p, mode);
    }
}

fn apply(p: Pin, mode: Mode) {
    let base = p.base();
    let n = p.n as usize;
    let two = 0b11u32 << (2 * n);
    let (moder, pull, af) = match mode {
        Mode::Input(pull) => (0b00u32, pull as u32, None),
        Mode::OutputLow | Mode::OutputHigh => (0b01, Pull::None as u32, None),
        Mode::Af(af) => (0b10, Pull::None as u32, Some(af as u32)),
    };
    match mode {
        Mode::OutputLow => p.set(false),
        Mode::OutputHigh => p.set(true),
        _ => {}
    }
    // SAFETY: GPIO registers of an existing port, configured during init before any
    // task runs (interrupts disabled; no concurrent access). Field positions per
    // RM0440 §9.4.1–9.4.10.
    unsafe {
        // Push-pull, very high speed (0b11) for AF and outputs.
        reg::modify(base + OTYPER, 1 << n, 0);
        reg::modify(base + OSPEEDR, two, 0b11 << (2 * n));
        reg::modify(base + PUPDR, two, pull << (2 * n));
        if let Some(af) = af {
            let (afr, shift) = if n < 8 {
                (AFRL, 4 * n)
            } else {
                (AFRH, 4 * (n - 8))
            };
            reg::modify(base + afr, 0xF << shift, (af & 0xF) << shift);
        }
        reg::modify(base + MODER, two, moder << (2 * n));
    }
}
