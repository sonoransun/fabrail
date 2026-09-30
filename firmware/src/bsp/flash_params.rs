//! Parameter A/B flash pages (core design §F: "Flash A/B pages (postcard + CRC32)";
//! RM0440 §5 embedded flash).
//!
//! Two slots at the top of flash (reserved in `memory.x`, where a link-time assertion
//! keeps the program image below them): slot A = 0x0807_E000, slot B = 0x0807_F000,
//! 4 KB each (two 2 KB pages in the default dual-bank mode, one 4 KB page in single-bank
//! mode; OPTR.DBANK decides, RM0440 §5.3.1). Slot layout, little-endian words:
//!
//! | Offset | Field |
//! |---|---|
//! | 0 | magic "FBPR" (0x5250_4246) |
//! | 4 | sequence number (the higher valid one is active) |
//! | 8 | payload length, bytes (≤ 4080) |
//! | 12 | CRC32 of the payload (`fabrail_params::crc32`) |
//! | 16.. | postcard image of `SystemParams` |
//!
//! A write erases the inactive slot, programs the payload, then the (length, CRC)
//! double-word, then the (magic, sequence) double-word, so an interrupted write leaves
//! the old slot active. Erasing stalls flash reads for tens of milliseconds, so writes
//! happen only with the executive stopped (boot/maintenance), never under control.
//! The safety block inside the image carries its own seal (`fabrail_params::crc`),
//! which the boot sequence verifies separately (SAF-015).

use fabrail_hal::ParamStore;
use stm32g4xx_hal::stm32::FLASH;

/// Slot base addresses.
pub const SLOT_A: usize = 0x0807_E000;
/// Slot B base address.
pub const SLOT_B: usize = 0x0807_F000;
/// Slot size, bytes.
pub const SLOT_LEN: usize = 4096;
/// Header size, bytes.
const HEADER: usize = 16;
/// Largest payload.
pub const PAYLOAD_MAX: usize = SLOT_LEN - HEADER;
const MAGIC: u32 = 0x5250_4246;
const ERASED: u32 = 0xFFFF_FFFF;

// FLASH_CR / FLASH_SR / FLASH_OPTR (RM0440 §5.7).
const CR_PG: u32 = 1 << 0;
const CR_PER: u32 = 1 << 1;
const CR_BKER: u32 = 1 << 11;
const CR_STRT: u32 = 1 << 16;
const CR_LOCK: u32 = 1 << 31;
const SR_BSY: u32 = 1 << 16;
const SR_ERRORS: u32 = 0xC3FA; // OPERR, PROGERR, WRPERR, PGAERR, SIZERR, PGSERR, MISSERR, FASTERR, RDERR, OPTVERR
const SR_EOP: u32 = 1 << 0;
const OPTR_DBANK: u32 = 1 << 22;
/// FLASH_KEYR unlock sequence (RM0440 §5.3.5).
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

/// Parameter store errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlashError {
    /// Both slots erased: no stored image (factory preset applies).
    Erased,
    /// A slot holds data but no slot passes its CRC (refuse to lift: ParamCrc).
    Corrupt,
    /// Image larger than a slot, or larger than the caller's buffer.
    TooLarge,
    /// Erase/program reported an error or timed out.
    Program,
}

#[derive(Clone, Copy)]
struct Header {
    magic: u32,
    seq: u32,
    len: u32,
    crc: u32,
}

fn word(addr: usize) -> u32 {
    // SAFETY: addr lies in the two reserved slots at the top of main flash (4-byte
    // aligned); flash is memory-mapped and always readable.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

fn header(slot: usize) -> Header {
    Header {
        magic: word(slot),
        seq: word(slot + 4),
        len: word(slot + 8),
        crc: word(slot + 12),
    }
}

/// The payload bytes of a slot (≤ PAYLOAD_MAX).
fn payload(slot: usize, len: usize) -> &'static [u8] {
    let len = len.min(PAYLOAD_MAX);
    // SAFETY: the range lies inside the reserved, memory-mapped slot and flash contents
    // are never modified while a borrow exists (writes happen only in `write`, which
    // takes `&mut self` and runs with the executive stopped).
    unsafe { core::slice::from_raw_parts((slot + HEADER) as *const u8, len) }
}

fn valid(slot: usize) -> Option<Header> {
    let h = header(slot);
    let len = h.len as usize;
    (h.magic == MAGIC && len <= PAYLOAD_MAX && fabrail_params::crc32(payload(slot, len)) == h.crc)
        .then_some(h)
}

/// The parameter store over the two flash slots.
pub struct FlashParams {
    flash: FLASH,
}

impl FlashParams {
    /// Wraps the flash controller.
    pub fn new(flash: FLASH) -> FlashParams {
        FlashParams { flash }
    }

    /// The active slot (highest valid sequence number).
    fn active(&self) -> Result<(usize, Header), FlashError> {
        match (valid(SLOT_A), valid(SLOT_B)) {
            (Some(a), Some(b)) => Ok(if b.seq > a.seq {
                (SLOT_B, b)
            } else {
                (SLOT_A, a)
            }),
            (Some(a), None) => Ok((SLOT_A, a)),
            (None, Some(b)) => Ok((SLOT_B, b)),
            (None, None) => {
                if header(SLOT_A).magic == ERASED && header(SLOT_B).magic == ERASED {
                    Err(FlashError::Erased)
                } else {
                    Err(FlashError::Corrupt)
                }
            }
        }
    }

    fn wait(&self) -> bool {
        for _ in 0..2_000_000u32 {
            if self.flash.sr().read().bits() & SR_BSY == 0 {
                return self.flash.sr().read().bits() & SR_ERRORS == 0;
            }
        }
        false
    }

    fn erase_slot(&mut self, slot: usize) -> bool {
        let dual = self.flash.optr().read().bits() & OPTR_DBANK != 0;
        let (page_size, bank_base) = if dual {
            (2048usize, 0x0804_0000usize)
        } else {
            (4096, 0x0800_0000)
        };
        let first = (slot - bank_base) / page_size;
        let pages = SLOT_LEN / page_size;
        let mut ok = true;
        for p in first..first + pages {
            let bker = if dual { CR_BKER } else { 0 };
            // SAFETY: page erase of a reserved parameter page (never program code;
            // memory.x asserts the image ends below SLOT_A); FLASH owned via &mut self
            // and unlocked by the caller (RM0440 §5.3.7).
            unsafe {
                self.flash.sr().write(|w| w.bits(SR_ERRORS | SR_EOP));
                self.flash
                    .cr()
                    .write(|w| w.bits(CR_PER | bker | (((p as u32) & 0x7F) << 3)));
                self.flash
                    .cr()
                    .write(|w| w.bits(CR_PER | bker | (((p as u32) & 0x7F) << 3) | CR_STRT));
            }
            ok &= self.wait();
        }
        // SAFETY: clears PER in the owned FLASH_CR.
        unsafe { self.flash.cr().write(|w| w.bits(0)) };
        ok
    }

    /// Programs one double-word at an 8-aligned address inside a slot.
    fn program(&mut self, addr: usize, lo: u32, hi: u32) -> bool {
        // SAFETY: double-word programming (RM0440 §5.3.8) of an erased location in a
        // reserved parameter slot: PG set, two consecutive aligned word writes.
        unsafe {
            self.flash.cr().write(|w| w.bits(CR_PG));
            core::ptr::write_volatile(addr as *mut u32, lo);
            core::ptr::write_volatile((addr + 4) as *mut u32, hi);
        }
        let ok = self.wait();
        // SAFETY: clears PG in the owned FLASH_CR.
        unsafe { self.flash.cr().write(|w| w.bits(0)) };
        ok
    }

    fn unlock(&mut self) {
        // SAFETY: the documented key sequence on the owned FLASH_KEYR.
        unsafe {
            self.flash.keyr().write(|w| w.bits(KEY1));
            self.flash.keyr().write(|w| w.bits(KEY2));
        }
    }

    fn lock(&mut self) {
        // SAFETY: setting LOCK in the owned FLASH_CR.
        unsafe { self.flash.cr().write(|w| w.bits(CR_LOCK)) };
    }
}

impl ParamStore for FlashParams {
    type Error = FlashError;

    fn read(&mut self, buf: &mut [u8]) -> Result<usize, FlashError> {
        let (slot, h) = self.active()?;
        let len = h.len as usize;
        let dst = buf.get_mut(..len).ok_or(FlashError::TooLarge)?;
        dst.copy_from_slice(payload(slot, len));
        Ok(len)
    }

    fn write(&mut self, data: &[u8]) -> Result<(), FlashError> {
        if data.len() > PAYLOAD_MAX {
            return Err(FlashError::TooLarge);
        }
        let (target, seq) = match self.active() {
            Ok((SLOT_A, h)) => (SLOT_B, h.seq.wrapping_add(1)),
            Ok((_, h)) => (SLOT_A, h.seq.wrapping_add(1)),
            Err(_) => (SLOT_A, 1),
        };
        self.unlock();
        let mut ok = self.erase_slot(target);
        for (k, chunk) in data.chunks(8).enumerate() {
            let mut dw = [0xFFu8; 8];
            dw.get_mut(..chunk.len())
                .map(|d| d.copy_from_slice(chunk))
                .unwrap_or(());
            let lo = u32::from_le_bytes([dw[0], dw[1], dw[2], dw[3]]);
            let hi = u32::from_le_bytes([dw[4], dw[5], dw[6], dw[7]]);
            ok &= self.program(target + HEADER + 8 * k, lo, hi);
        }
        ok &= self.program(target + 8, data.len() as u32, fabrail_params::crc32(data));
        ok &= self.program(target, MAGIC, seq);
        self.lock();
        if ok && valid(target).is_some() {
            Ok(())
        } else {
            Err(FlashError::Program)
        }
    }
}
