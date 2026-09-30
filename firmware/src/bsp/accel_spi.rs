//! SPI accelerometers (RM0440 §39 SPI, §12–13 DMA/DMAMUX, §15 EXTI).
//!
//! * [`Adxl355`] on SPI1 + DMA1 CH1/CH2: the **structure-mounted** earthquake-early-
//!   warning accelerometer (SEI-001; never on the carriage, core design F16). At each
//!   DRDY rising edge (EXTI6, 500 Hz ODR) the `accel` task starts one DMA read of the
//!   nine data registers XDATA3..ZDATA1; the DMA completion task (`accel_dma`) decodes
//!   them with `fabrail_core::frontend::adxl355_decode` and queues an `AccelRaw` for the
//!   100 Hz seismic group (5 samples per release: the "FIFO drain" of core design §A.5).
//!   Register map and settings: Analog Devices ADXL354/ADXL355 data sheet Rev. D
//!   (DEVID_AD 0x00 = 0xAD, PARTID 0x02 = 0xED, FILTER 0x28 ODR_LPF from
//!   `design.sensors.accel_odr_hz` via [`adxl355_odr_code`] (0011 → 500 Hz),
//!   RANGE 0x2C = 01 → ±2 g with INT_POL high, POWER_CTL 0x2D = 0 → measurement,
//!   RESET 0x2F ← 0x52; SPI mode 0, address byte = reg << 1 | R).
//! * [`InertialSpi`] on SPI3 + DMA2 CH1/CH2: the carriage IMU (skyhook/fusion only) and
//!   the rail floor accelerometer (inertial-smoothing feed-forward, PHYSICS §16.2), both
//!   LSM6DSO-class parts (ST DS12814: WHO_AM_I 0x0F = 0x6C, CTRL1_XL 0x10 = 0xAC →
//!   6.66 kHz, ±8 g; CTRL3_C 0x12 = 0x44 → BDU + auto-increment; OUTX_L_A 0x28; SPI
//!   mode 3, read bit 0x80). The base task polls it at fast releases: it latches the
//!   transfer started at the previous release, then starts the next one, alternating
//!   the two devices, so no interrupt is needed and the latency is one fast period.
//!
//! Absent or silent devices report `present = false`; the frontend raises the
//! corresponding sensor fault.

use core::ptr::{addr_of, addr_of_mut};

use fabrail_types::{AccelRaw, ImuRaw, RailAccelRaw};
use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::stm32::{DMA1, DMA2, DMAMUX, EXTI, SPI1, SPI3, SYSCFG, dma1, spi1};

use super::pins::{self, Pin};

// SPI CR1/CR2/SR (RM0440 §39.6; 16-bit registers).
const CPHA: u16 = 1 << 0;
const CPOL: u16 = 1 << 1;
const MSTR: u16 = 1 << 2;
const BR_DIV32: u16 = 0b100 << 3; // 170 MHz / 32 = 5.3 MHz (both parts ≤ 10 MHz)
const SPE: u16 = 1 << 6;
const SSI: u16 = 1 << 8;
const SSM: u16 = 1 << 9;
const RXDMAEN: u16 = 1 << 0;
const TXDMAEN: u16 = 1 << 1;
const DS_8BIT: u16 = 0b0111 << 8;
const FRXTH: u16 = 1 << 12;
const RXNE: u16 = 1 << 0;
const TXE: u16 = 1 << 1;
const BSY: u16 = 1 << 7;

// DMA channel CR (8-bit transfers).
const DMA_EN: u32 = 1 << 0;
const DMA_TCIE: u32 = 1 << 1;
const DMA_DIR_M2P: u32 = 1 << 4;
const DMA_MINC: u32 = 1 << 7;
/// DMA_ISR/IFCR bits for channel 1 (TCIF1, CGIF1) and channel 2.
const TCIF1: u32 = 1 << 1;
const CGIF1: u32 = 1 << 0;
const CGIF2: u32 = 1 << 4;
// DMAMUX request IDs (RM0440 Table 91).
const REQ_SPI1_RX: u32 = 10;
const REQ_SPI1_TX: u32 = 11;
const REQ_SPI3_RX: u32 = 14;
const REQ_SPI3_TX: u32 = 15;

/// ADXL355 registers.
const ADXL_DEVID_AD: u8 = 0x00;
const ADXL_PARTID: u8 = 0x02;
const ADXL_XDATA3: u8 = 0x08;
const ADXL_FILTER: u8 = 0x28;
const ADXL_RANGE: u8 = 0x2C;
const ADXL_POWER_CTL: u8 = 0x2D;
const ADXL_RESET: u8 = 0x2F;
/// Address byte + nine data bytes.
const ADXL_XFER: usize = 10;

/// LSM6DSO registers.
const LSM_WHO_AM_I: u8 = 0x0F;
const LSM_CTRL1_XL: u8 = 0x10;
const LSM_CTRL3_C: u8 = 0x12;
const LSM_OUTX_L_A: u8 = 0x28;
/// Address byte + six data bytes.
const LSM_XFER: usize = 7;

static mut ADXL_TX: [u8; ADXL_XFER] = [0; ADXL_XFER];
static mut ADXL_RX: [u8; ADXL_XFER] = [0; ADXL_XFER];
static mut LSM_TX: [u8; LSM_XFER] = [0; LSM_XFER];
static mut LSM_RX: [u8; LSM_XFER] = [0; LSM_XFER];

/// ADXL355 FILTER.ODR_LPF code for an output data rate (data sheet Rev. D, FILTER register:
/// 4000 Hz · 2⁻ᵏ for k = 0..=10). None for a rate the device does not offer; the boot
/// sequence rejects such a parameter set.
pub fn adxl355_odr_code(odr_hz: f64) -> Option<u8> {
    (0u8..=10).find(|k| {
        let rate = 4000.0 / f64::from(1u32 << k);
        let rel = (odr_hz - rate) / rate;
        rel > -1e-3 && rel < 1e-3
    })
}

/// Blocking single-byte full-duplex transfer (init only); None on timeout.
fn xfer(spi: &spi1::RegisterBlock, b: u8) -> Option<u8> {
    let mut ok = false;
    for _ in 0..10_000u32 {
        if spi.sr().read().bits() & TXE != 0 {
            ok = true;
            break;
        }
    }
    if !ok {
        return None;
    }
    // SAFETY: 8-bit write to DR of an owned, enabled SPI (8-bit frames).
    unsafe { spi.dr8().write(|w| w.bits(b)) };
    for _ in 0..10_000u32 {
        if spi.sr().read().bits() & RXNE != 0 {
            return Some(spi.dr8().read().bits());
        }
    }
    None
}

/// Blocking register write (init only).
fn write_reg(spi: &spi1::RegisterBlock, cs: Pin, addr: u8, val: u8) -> bool {
    cs.set(false);
    let ok = xfer(spi, addr).is_some() && xfer(spi, val).is_some();
    cs.set(true);
    ok
}

/// Blocking register read (init only).
fn read_reg(spi: &spi1::RegisterBlock, cs: Pin, addr: u8) -> Option<u8> {
    cs.set(false);
    let r = xfer(spi, addr).and_then(|_| xfer(spi, 0));
    cs.set(true);
    r
}

fn spi_init(spi: &spi1::RegisterBlock, mode3: bool) {
    let mode = if mode3 { CPOL | CPHA } else { 0 };
    // SAFETY: owned SPI configured once in init: master, software NSS, 8-bit frames,
    // RXNE at 8 bits (RM0440 §39.6.1–2).
    unsafe {
        spi.cr1()
            .write(|w| w.bits(mode | MSTR | BR_DIV32 | SSI | SSM));
        spi.cr2().write(|w| w.bits(DS_8BIT | FRXTH));
        spi.cr1()
            .write(|w| w.bits(mode | MSTR | BR_DIV32 | SSI | SSM | SPE));
    }
}

/// Programs an RX/TX DMA channel pair for one SPI transfer of `n` bytes and starts it
/// (RM0440 §39.4.9: RXDMAEN, channels, then TXDMAEN).
///
/// # Safety
/// `rx`/`tx` must be static buffers of ≥ `n` bytes used only by these channels, and the
/// channels and SPI must be owned by the caller with no transfer in progress.
unsafe fn dma_start(
    spi: &spi1::RegisterBlock,
    rx_ch: &dma1::CH,
    tx_ch: &dma1::CH,
    rx: *mut u8,
    tx: *const u8,
    n: usize,
    tcie: bool,
) {
    let dr = spi.dr8().as_ptr() as u32;
    // SAFETY: forwarded caller guarantees; encodings per RM0440 §12.6.3–12.6.6.
    unsafe {
        rx_ch.cr().write(|w| w.bits(0));
        tx_ch.cr().write(|w| w.bits(0));
        rx_ch.par().write(|w| w.bits(dr));
        rx_ch.mar().write(|w| w.bits(rx as u32));
        rx_ch.ndtr().write(|w| w.bits(n as u32));
        tx_ch.par().write(|w| w.bits(dr));
        tx_ch.mar().write(|w| w.bits(tx as u32));
        tx_ch.ndtr().write(|w| w.bits(n as u32));
        spi.cr2().modify(|r, w| w.bits(r.bits() | RXDMAEN));
        rx_ch
            .cr()
            .write(|w| w.bits(DMA_MINC | DMA_EN | if tcie { DMA_TCIE } else { 0 }));
        tx_ch
            .cr()
            .write(|w| w.bits(DMA_MINC | DMA_DIR_M2P | DMA_EN));
        spi.cr2().modify(|r, w| w.bits(r.bits() | TXDMAEN));
    }
}

/// Ends a DMA transfer: channels off, SPI DMA requests off.
fn dma_stop(spi: &spi1::RegisterBlock, rx_ch: &dma1::CH, tx_ch: &dma1::CH) {
    // SAFETY: disabling owned DMA channels and the SPI DMA request bits.
    unsafe {
        rx_ch.cr().write(|w| w.bits(0));
        tx_ch.cr().write(|w| w.bits(0));
        spi.cr2()
            .modify(|r, w| w.bits(r.bits() & !(RXDMAEN | TXDMAEN)));
    }
}

/// ADXL355 driver (structure accelerometer).
pub struct Adxl355 {
    spi: SPI1,
    dma: DMA1,
    exti: EXTI,
    present: bool,
    busy: bool,
    seq: u32,
    /// A DRDY arrived while a transfer was still running (sample lost).
    missed: bool,
}

impl Adxl355 {
    /// Configures SPI1, DMA1 CH1/CH2, EXTI6 (DRDY) and the sensor. `present` is false
    /// when the device IDs do not match (the seismic group then sees no samples).
    pub fn init(
        spi: SPI1,
        dma: DMA1,
        exti: EXTI,
        syscfg: &SYSCFG,
        dmamux: &DMAMUX,
        odr_hz: f64,
        rcc: &Rcc,
    ) -> Adxl355 {
        rcc.apb2enr().modify(|_, w| {
            w.spi1en().set_bit();
            w.syscfgen().set_bit()
        });
        spi_init(&spi, false);
        let cs = pins::ADXL_CS;
        let mut ok = write_reg(&spi, cs, ADXL_RESET << 1, 0x52);
        cortex_m::asm::delay(170_000); // ≥ 1 ms after a soft reset (5 ms turn-on budget)
        let id_ad = read_reg(&spi, cs, (ADXL_DEVID_AD << 1) | 1);
        let part = read_reg(&spi, cs, (ADXL_PARTID << 1) | 1);
        ok &= id_ad == Some(0xAD) && part == Some(0xED);
        let odr = adxl355_odr_code(odr_hz);
        ok &= odr.is_some() && write_reg(&spi, cs, ADXL_FILTER << 1, odr.unwrap_or(0x03));
        ok &= write_reg(&spi, cs, ADXL_RANGE << 1, 0x41);
        ok &= write_reg(&spi, cs, ADXL_POWER_CTL << 1, 0x00);
        // SAFETY: DMAMUX channel 0/1 = DMA1 CH1/CH2 (request routing, init only); the
        // TX buffer is static and written only here; EXTI line 6 ← port C (SYSCFG
        // EXTICR2 field EXTI6 = 0x2), rising edge, unmasked (RM0440 §10.2.4, §15.5).
        unsafe {
            dmamux.ccr(0).write(|w| w.bits(REQ_SPI1_RX));
            dmamux.ccr(1).write(|w| w.bits(REQ_SPI1_TX));
            let tx = &mut *addr_of_mut!(ADXL_TX);
            tx[0] = (ADXL_XDATA3 << 1) | 1;
            syscfg
                .exticr2()
                .modify(|r, w| w.bits((r.bits() & !(0xF << 8)) | (0x2 << 8)));
            exti.rtsr1().modify(|r, w| w.bits(r.bits() | (1 << 6)));
            exti.pr1().write(|w| w.bits(1 << 6));
            if ok {
                exti.imr1().modify(|r, w| w.bits(r.bits() | (1 << 6)));
            }
        }
        Adxl355 {
            spi,
            dma,
            exti,
            present: ok,
            busy: false,
            seq: 0,
            missed: false,
        }
    }

    /// Device found and configured.
    pub fn present(&self) -> bool {
        self.present
    }

    /// DRDY interrupt: acknowledge EXTI6 and start the DMA read.
    pub fn on_drdy(&mut self) {
        // SAFETY: EXTI_PR1 is rc_w1; only line 6 is cleared.
        unsafe { self.exti.pr1().write(|w| w.bits(1 << 6)) };
        if !self.present {
            return;
        }
        if self.busy {
            self.missed = true;
            return;
        }
        self.busy = true;
        pins::ADXL_CS.set(false);
        // SAFETY: static transfer buffers used only by DMA1 CH1/CH2, owned here; no
        // transfer is in progress (busy was false).
        unsafe {
            dma_start(
                &self.spi,
                self.dma.ch1(),
                self.dma.ch2(),
                addr_of_mut!(ADXL_RX).cast::<u8>(),
                addr_of!(ADXL_TX).cast::<u8>(),
                ADXL_XFER,
                true,
            );
        }
    }

    /// DMA1 CH1 transfer-complete interrupt: ends the transfer and decodes the sample.
    pub fn on_dma_complete(&mut self) -> Option<AccelRaw> {
        let done = self.dma.isr().read().bits() & TCIF1 != 0;
        // SAFETY: IFCR is write-1-to-clear; clears only channel 1 and 2 flags.
        unsafe { self.dma.ifcr().write(|w| w.bits(CGIF1 | CGIF2)) };
        if !done || !self.busy {
            return None;
        }
        for _ in 0..1_000u32 {
            if self.spi.sr().read().bits() & BSY == 0 {
                break;
            }
        }
        pins::ADXL_CS.set(true);
        dma_stop(&self.spi, self.dma.ch1(), self.dma.ch2());
        self.busy = false;
        // SAFETY: the transfer has completed and the channel is disabled, so no DMA
        // writes ADXL_RX concurrently.
        let rx = unsafe { core::ptr::read_volatile(addr_of!(ADXL_RX)) };
        let mut data = [0u8; 9];
        if let Some(src) = rx.get(1..ADXL_XFER) {
            data.copy_from_slice(src);
        }
        let overrun = core::mem::replace(&mut self.missed, false);
        self.seq = self.seq.wrapping_add(1);
        Some(AccelRaw {
            xyz: fabrail_core::frontend::adxl355_decode(&data),
            seq: self.seq,
            overrun,
        })
    }
}

/// Which LSM6DSO-class device a transfer addresses.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Dev {
    Imu,
    Rail,
}

/// Carriage IMU and rail accelerometer on SPI3 + DMA2 CH1/CH2 (polled).
pub struct InertialSpi {
    spi: SPI3,
    dma: DMA2,
    imu_ok: bool,
    rail_ok: bool,
    pending: Option<Dev>,
    last: Dev,
    imu: [i32; 3],
    rail: [i32; 3],
}

fn lsm_init(spi: &spi1::RegisterBlock, cs: Pin) -> bool {
    let who = read_reg(spi, cs, LSM_WHO_AM_I | 0x80);
    who == Some(0x6C)
        && write_reg(spi, cs, LSM_CTRL3_C, 0x44)
        && write_reg(spi, cs, LSM_CTRL1_XL, 0xAC)
}

impl InertialSpi {
    /// Configures SPI3 and DMA2 CH1/CH2 and probes the devices the parameters declare
    /// fitted (`imu_fitted`, `rail_fitted`).
    pub fn init(
        spi: SPI3,
        dma: DMA2,
        dmamux: &DMAMUX,
        imu_fitted: bool,
        rail_fitted: bool,
        rcc: &Rcc,
    ) -> InertialSpi {
        rcc.apb1enr1().modify(|_, w| w.spi3en().set_bit());
        rcc.ahb1enr().modify(|_, w| w.dma2en().set_bit());
        spi_init(&spi, true);
        let imu_ok = imu_fitted && lsm_init(&spi, pins::IMU_CS);
        let rail_ok = rail_fitted && lsm_init(&spi, pins::RAIL_CS);
        // SAFETY: DMAMUX channels 8/9 = DMA2 CH1/CH2 (init only); TX buffer static and
        // written only here.
        unsafe {
            dmamux.ccr(8).write(|w| w.bits(REQ_SPI3_RX));
            dmamux.ccr(9).write(|w| w.bits(REQ_SPI3_TX));
            let tx = &mut *addr_of_mut!(LSM_TX);
            tx[0] = LSM_OUTX_L_A | 0x80;
        }
        InertialSpi {
            spi,
            dma,
            imu_ok,
            rail_ok,
            pending: None,
            last: Dev::Rail,
            imu: [0; 3],
            rail: [0; 3],
        }
    }

    /// Fast-release poll: latch the finished transfer, start the next one.
    pub fn poll(&mut self) {
        if let Some(dev) = self.pending {
            if self.dma.isr().read().bits() & TCIF1 == 0 {
                return; // still running (should not happen at 5–10 kHz); keep the old sample
            }
            // SAFETY: IFCR write-1-to-clear of channels 1/2 of the owned DMA2.
            unsafe { self.dma.ifcr().write(|w| w.bits(CGIF1 | CGIF2)) };
            match dev {
                Dev::Imu => pins::IMU_CS.set(true),
                Dev::Rail => pins::RAIL_CS.set(true),
            }
            dma_stop(&self.spi, self.dma.ch1(), self.dma.ch2());
            // SAFETY: transfer complete and channel disabled: no concurrent DMA write.
            let rx = unsafe { core::ptr::read_volatile(addr_of!(LSM_RX)) };
            let axis = |k: usize| -> i32 {
                let lo = rx.get(1 + 2 * k).copied().unwrap_or(0);
                let hi = rx.get(2 + 2 * k).copied().unwrap_or(0);
                i32::from(i16::from_le_bytes([lo, hi]))
            };
            let xyz = [axis(0), axis(1), axis(2)];
            match dev {
                Dev::Imu => self.imu = xyz,
                Dev::Rail => self.rail = xyz,
            }
            self.pending = None;
        }
        let next = match (self.imu_ok, self.rail_ok) {
            (true, true) if self.last == Dev::Rail => Some(Dev::Imu),
            (true, true) | (false, true) => Some(Dev::Rail),
            (true, false) => Some(Dev::Imu),
            (false, false) => None,
        };
        if let Some(dev) = next {
            match dev {
                Dev::Imu => pins::IMU_CS.set(false),
                Dev::Rail => pins::RAIL_CS.set(false),
            }
            // SAFETY: static buffers used only by DMA2 CH1/CH2, owned here; the previous
            // transfer has been stopped above.
            unsafe {
                dma_start(
                    &self.spi,
                    self.dma.ch1(),
                    self.dma.ch2(),
                    addr_of_mut!(LSM_RX).cast::<u8>(),
                    addr_of!(LSM_TX).cast::<u8>(),
                    LSM_XFER,
                    false,
                );
            }
            self.pending = Some(dev);
            self.last = dev;
        }
    }

    /// Latest carriage IMU sample.
    pub fn imu(&self) -> ImuRaw {
        ImuRaw {
            xyz: self.imu,
            present: self.imu_ok,
        }
    }

    /// Latest rail accelerometer sample.
    pub fn rail(&self) -> RailAccelRaw {
        RailAccelRaw {
            xyz: self.rail,
            present: self.rail_ok,
        }
    }
}
