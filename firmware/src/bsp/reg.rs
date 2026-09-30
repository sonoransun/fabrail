//! Volatile 32-bit register access by address.
//!
//! Used where several peripherals share one register layout under different PAC types
//! (the GPIO ports, the HRTIM timing units A–D) or where a register is touched from a
//! fault handler without owning the peripheral. Everything else uses the PAC's typed
//! accessors.

/// Reads a 32-bit register.
///
/// # Safety
/// `addr` must be the address of a readable, 4-byte-aligned memory-mapped register of
/// this device (RM0440 memory map).
#[inline(always)]
pub(crate) unsafe fn read(addr: usize) -> u32 {
    // SAFETY: the caller guarantees a valid, aligned MMIO register address.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Writes a 32-bit register.
///
/// # Safety
/// `addr` must be the address of a writable, 4-byte-aligned memory-mapped register of
/// this device, and the value must be valid for it; the caller owns the peripheral (or
/// the register is a write-only set/reset register such as GPIO BSRR).
#[inline(always)]
pub(crate) unsafe fn write(addr: usize, value: u32) {
    // SAFETY: the caller guarantees a valid, aligned MMIO register address and value.
    unsafe { core::ptr::write_volatile(addr as *mut u32, value) }
}

/// Read-modify-write: clears `clear`, then sets `set`.
///
/// # Safety
/// As [`read`] and [`write`]; additionally no higher-priority context may modify the
/// same register concurrently (callers run in init, or own the peripheral in one task).
#[inline(always)]
pub(crate) unsafe fn modify(addr: usize, clear: u32, set: u32) {
    // SAFETY: forwarded caller guarantees.
    unsafe {
        let v = read(addr);
        write(addr, (v & !clear) | set);
    }
}
