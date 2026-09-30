//! Board support package: register-level drivers for the STM32G474 (RM0440 Rev 8).
//!
//! **This directory is the only place in the firmware with `unsafe` code** (plan §2b
//! item 15, FAB-FW-002); every `unsafe` block carries a `// SAFETY:` comment. The BSP
//! does register work only: scaling constants come from `fabrail_params`, and every
//! control or safety decision stays in the shared crates (core, safety, rt). The HAL
//! trait implementations that the `fabrail-rt` groups call live in `hal_impl.rs` and
//! are safe code over these drivers.
//!
//! | Module | Peripheral | Function (core design §F) |
//! |---|---|---|
//! | [`clocks`] | RCC, PWR, FLASH, DWT, SysTick | 170 MHz from the 24 MHz HSE, boost mode, cycle counter |
//! | [`tim1`] | TIM1 | LSM 3-phase inverter, centre-aligned 20 kHz; STO = MOE clear, ASC = low sides on |
//! | [`hrtim`] | HRTIM TA–TD | 8 coil H-bridges (locked anti-phase), hardware fault inputs |
//! | [`protect`] | COMP1–7 + DAC3/DAC4 | hardware overcurrent trips into TIM1 break and HRTIM faults |
//! | [`adc`] | ADC1–5 + DMA1 | PWM-synchronous phase and coil currents, gap probes, power sense |
//! | [`encoder`] | TIM2 | 32-bit quadrature encoder extended to i64, reference mark |
//! | [`accel_spi`] | SPI1 + DMA1, SPI3 + DMA2, EXTI | ADXL355 FIFO (structure), carriage IMU, rail accelerometer |
//! | [`can`] | FDCAN1 | command/alert link and telemetry (frame layouts documented there) |
//! | [`flash_params`] | FLASH | parameter A/B pages (postcard + CRC32) |
//! | [`watchdog`] | IWDG, WWDG | ≈ 5 ms independent watchdog, windowed watchdog |
//! | [`crash`] | SCB, `.uninit` | panic/HardFault handler: outputs off, crash record, reset |
//!
//! **Pin map** ([`pins`]): digital functions on the LQFP64 STM32G474RE; the analog
//! channels are assigned by ADC channel number and their pins follow DS12288 Table 13
//! (the board design fixes them; verify at bring-up; the full analog front end needs
//! the LQFP100/128 package of the same die). Alternate-function numbers are
//! cross-checked against the stm32g4xx-hal 0.1.0 pin tables (HRTIM AF13, TIM1 AF2/AF6,
//! TIM2 AF1, SPI1 AF5, SPI3 AF6, FDCAN1 AF9).
//!
//! Nothing here has run on hardware (FAB-FW-012): register values are derived from
//! RM0440 and the device data sheets and must be verified at bring-up.
//!
//! **Where the numbers come from.** Register offsets and bit positions were checked
//! against the stm32g4 0.16 PAC (generated from ST's SVD), and the interrupt names
//! against its `Interrupt` enum. The DMAMUX request IDs, the comparator input maps, the
//! HRTIM fault-input map and the alternate-function numbers were checked against the
//! stm32g4xx-hal 0.1.0 tables. RM0440 / DS12288 section and table numbers quoted in these
//! files identify the topic (Rev 8 numbering) and were not re-read in this environment.
//! Datasheet timing values are typical figures and need re-checking before bring-up.

#![allow(unsafe_code)] // the only module tree allowed to contain `unsafe` (see above)

pub mod accel_spi;
pub mod adc;
pub mod can;
pub mod clocks;
pub mod crash;
pub mod encoder;
pub mod flash_params;
pub mod hrtim;
pub mod pins;
pub mod protect;
pub mod tim1;
pub mod watchdog;

mod reg;

/// Core clock after [`clocks::init`], Hz (170 MHz; RM0440 §7.2, boost mode).
pub const SYSCLK_HZ: u32 = 170_000_000;
