//! Production fault handling (core design §F "Panic and HardFault handler"; FAB-FW-009).
//!
//! On a panic (production image), a HardFault or an unexpected interrupt:
//! 1. every power output is switched off: both gate enables low, TIM1 MOE cleared (LSM
//!    Safe Torque Off), every HRTIM output disabled (coil bridges off). With the
//!    bridges off the hybrid-PM carriage settles onto its mechanical stops / rollers
//!    (SAF-010; STO/ASC act on propulsion only, and a crash is the one case where the
//!    energised SS1 landing is no longer available);
//! 2. a [`CrashRecord`] (magic, kind, PC, LR, xPSR, tick, source line, CRC32) is
//!    written to the `.uninit` section, which survives a warm reset;
//! 3. `SCB::sys_reset()`.
//!
//! `panic-halt` is rejected: it would halt with the timers still driving the last duty
//! cycle. `panic-probe` is used only with the `dev` feature (it also ends in a
//! breakpoint/reset, and the HardFault path below still switches the outputs off).
//!
//! At boot `main` reads the record ([`take`]): a valid record keeps the firmware in its
//! boot-fault hold (outputs off, control not started) until an explicit Reset command
//! arrives over the link while the gaps read landed/seated, then [`clear`] and reset.
//! A power cycle loses the record (SRAM is not retained), which is the documented
//! limit of this mechanism.

use core::mem::MaybeUninit;
use core::panic::PanicInfo;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::SCB;
use stm32g4xx_hal::stm32;

use super::pins;

/// Record magic ("FBCR").
const MAGIC: u32 = 0x4642_4352;

/// What ended the previous run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum CrashKind {
    /// Rust panic (production handler).
    Panic = 1,
    /// HardFault exception.
    HardFault = 2,
    /// An interrupt without a handler fired (DefaultHandler).
    UnexpectedIrq = 3,
}

impl CrashKind {
    const fn from_u32(v: u32) -> Option<CrashKind> {
        match v {
            1 => Some(CrashKind::Panic),
            2 => Some(CrashKind::HardFault),
            3 => Some(CrashKind::UnexpectedIrq),
            _ => None,
        }
    }
}

/// Crash record in `.uninit` (40 bytes; `repr(C)`, little-endian words).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(C)]
pub struct CrashRecord {
    /// [`MAGIC`].
    pub magic: u32,
    /// [`CrashKind`] as u32.
    pub kind: u32,
    /// Faulting PC (HardFault) or 0.
    pub pc: u32,
    /// LR at the fault (HardFault) or 0.
    pub lr: u32,
    /// xPSR at the fault (HardFault; the low 9 bits are the exception number) or the IRQ
    /// number (UnexpectedIrq) or 0.
    pub xpsr: u32,
    /// Base tick at the fault, low word.
    pub tick_lo: u32,
    /// Base tick at the fault, high word.
    pub tick_hi: u32,
    /// Panic source line or 0.
    pub line: u32,
    /// CRC32 of the panic source file name, or 0.
    pub file_crc: u32,
    /// CRC32 of the nine words above (fabrail_params::crc32).
    pub crc: u32,
}

impl CrashRecord {
    fn words_crc(&self) -> u32 {
        let w = [
            self.magic,
            self.kind,
            self.pc,
            self.lr,
            self.xpsr,
            self.tick_lo,
            self.tick_hi,
            self.line,
            self.file_crc,
        ];
        let mut b = [0u8; 36];
        for (dst, v) in b.as_chunks_mut::<4>().0.iter_mut().zip(w) {
            dst.copy_from_slice(&v.to_le_bytes());
        }
        fabrail_params::crc32(&b)
    }

    /// The crash kind, if the record is valid.
    pub fn kind(&self) -> Option<CrashKind> {
        CrashKind::from_u32(self.kind)
    }

    /// Base tick at the crash.
    pub fn tick(&self) -> u64 {
        (u64::from(self.tick_hi) << 32) | u64::from(self.tick_lo)
    }
}

#[unsafe(link_section = ".uninit.fabrail_crash")]
static mut CRASH: MaybeUninit<CrashRecord> = MaybeUninit::uninit();

/// Latest base tick, low and high words (written by the base task every tick; read
/// only by the fault handlers, so a torn pair at a 2³² boundary is harmless).
static TICK_LO: AtomicU32 = AtomicU32::new(0);
static TICK_HI: AtomicU32 = AtomicU32::new(0);

/// Records the current base tick for a later crash record (two relaxed stores).
#[inline(always)]
pub fn note_tick(t: u64) {
    TICK_LO.store(t as u32, Ordering::Relaxed);
    TICK_HI.store((t >> 32) as u32, Ordering::Relaxed);
}

/// The record left by the previous run, if valid (magic, kind and CRC check out).
pub fn take() -> Option<CrashRecord> {
    // SAFETY: CRASH lives in .uninit (never initialised by the runtime); reading its
    // bytes as u32 words is defined for any bit pattern (CrashRecord is plain u32s).
    // Called once in init, before any handler that writes it can run.
    let r = unsafe { core::ptr::read_volatile(addr_of_mut!(CRASH).cast::<CrashRecord>()) };
    (r.magic == MAGIC && r.kind().is_some() && r.crc == r.words_crc()).then_some(r)
}

/// Invalidates the record (operator Reset accepted).
pub fn clear() {
    // SAFETY: a single aligned word write into our own .uninit record; the fault
    // handlers that also write it end in a reset and never return to this code.
    unsafe {
        core::ptr::write_volatile(addr_of_mut!(CRASH).cast::<u32>(), 0);
    }
}

fn record(kind: CrashKind, pc: u32, lr: u32, xpsr: u32, line: u32, file_crc: u32) {
    let mut r = CrashRecord {
        magic: MAGIC,
        kind: kind as u32,
        pc,
        lr,
        xpsr,
        tick_lo: TICK_LO.load(Ordering::Relaxed),
        tick_hi: TICK_HI.load(Ordering::Relaxed),
        line,
        file_crc,
        crc: 0,
    };
    r.crc = r.words_crc();
    // SAFETY: interrupts are disabled by the caller and the handler never returns, so
    // this is the only access to CRASH; the pointer is valid and aligned.
    unsafe {
        core::ptr::write_volatile(addr_of_mut!(CRASH).cast::<CrashRecord>(), r);
    }
}

/// Switches every power output off without owning the peripherals: gate enables low
/// (BSRR), TIM1 MOE cleared (LSM STO, RM0440 §28.6.20 BDTR.MOE), all HRTIM outputs
/// disabled (HRTIM_ODISR, RM0440 §27.5.51). Idempotent; safe to call from any context.
pub fn outputs_off() {
    pins::force_gates_off();
    // SAFETY: fault/shutdown path. Clearing MOE and writing the write-only ODISR
    // register only disable outputs; a racing task write can at worst re-disable. The
    // pointers come from the PAC and are valid for the device.
    unsafe {
        (*stm32::TIM1::ptr())
            .bdtr()
            .modify(|_, w| w.moe().clear_bit());
        (*stm32::HRTIM_COMMON::ptr())
            .odisr()
            .write(|w| w.bits(0x0FFF));
    }
}

/// Outputs off, crash record, reset.
fn die(kind: CrashKind, pc: u32, lr: u32, xpsr: u32, line: u32, file_crc: u32) -> ! {
    cortex_m::interrupt::disable();
    outputs_off();
    record(kind, pc, lr, xpsr, line, file_crc);
    SCB::sys_reset()
}

/// Production panic handler (the `dev` feature uses panic-probe instead).
#[cfg(not(feature = "dev"))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let (line, file_crc) = info.location().map_or((0, 0), |l| {
        (l.line(), fabrail_params::crc32(l.file().as_bytes()))
    });
    die(CrashKind::Panic, 0, 0, 0, line, file_crc)
}

/// Keeps `PanicInfo` referenced in dev builds (panic-probe owns the handler there).
#[cfg(feature = "dev")]
#[allow(dead_code)]
fn _panic_info_type(_: &PanicInfo) {}

/// HardFault: outputs off, crash record with the stacked PC/LR/xPSR, reset.
///
/// # Safety
/// Called only by the hardware exception entry (cortex-m-rt trampoline) with a valid
/// stacked exception frame; never called from Rust code.
#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    die(CrashKind::HardFault, ef.pc(), ef.lr(), ef.xpsr(), 0, 0)
}

/// Any interrupt without a handler: outputs off, crash record, reset.
///
/// # Safety
/// Called only by the hardware for an unhandled interrupt; never called from Rust code.
#[cortex_m_rt::exception]
unsafe fn DefaultHandler(irqn: i16) {
    die(CrashKind::UnexpectedIrq, 0, 0, irqn as u32, 0, 0)
}
