# fabrail firmware (STM32G474RE, RTIC 2)

**Status: builds and links; never flashed.** The image compiles for
`thumbv7em-none-eabihf`, links with `rust-lld` and meets its size budget of 320 KiB flash
and 64 KiB static RAM (`scripts/check_firmware.sh`, FAB-FW-010; "Size" below). Nothing here has run on hardware: cycle
counts, PWM-synchronous sampling, comparator trip latency, CAN timing and every
register value are unverified (FAB-FW-012). Treat the BSP as a reviewed first draft for
bring-up, not as working firmware.

This is a separate Cargo workspace. It runs the same `fabrail-rt` group code as the
simulator (SIL ≡ firmware, claimed in voltage coil mode only); the only
firmware-specific code is the RTIC wiring (`src/main.rs`), the HAL trait implementations
(`src/hal_impl.rs`, safe code) and the register-level board support (`src/bsp/`, the
only place with `unsafe`, each block with a `// SAFETY:` comment).

## Build

```bash
# Homebrew rustc comes first on PATH and has no thumbv7em sysroot (CLAUDE.md):
cd firmware
RUSTC=$(rustup which rustc --toolchain stable) rustup run stable cargo build --release
RUSTC=$(rustup which rustc --toolchain stable) rustup run stable cargo build --release \
    --no-default-features --features class-wall        # drawer | door (default) | wall | heavy
RUSTC=$(rustup which rustc --toolchain stable) rustup run stable cargo build --release \
    --features dev                                      # defmt over RTT + panic-probe

scripts/check_firmware.sh     # from the repo root: policy, lint, all four classes, size
```

Flashing would be `cargo run --release` with probe-rs (`.cargo/config.toml` runner);
it has not been done.

| Feature | Effect |
|---|---|
| `class-door` (default), `class-drawer`, `class-wall`, `class-heavy` | Exactly one: the factory preset used when the parameter pages are erased, and the class the stored parameters must match |
| `dev` | defmt/RTT logging and panic-probe. Production images use the handler in `bsp/crash.rs` |

## Task map

NVIC with 4 priority bits. `FMAC`, `CORDIC` and `I2C4_EV` are unused vectors that the
base task pends, which releases the lower-rate groups drift-free on base ticks. Interrupt
names are checked against the stm32g4 0.16 PAC `Interrupt` enum.

| Task | Binds | Rate | Priority | Budget (cycles) | C/T | Work |
|---|---|---|---|---|---|---|
| `base` | ADC1_2 | 20 kHz | 8 | 2 500 | 0.294 | `BaseTask::run`; LET input snapshots; pends the groups |
| `fast` | FMAC | 10 / 5 kHz | 6 | 4 000 / 8 000 | 0.235 | `FastGroup::run` |
| `accel` | EXTI9_5 | 500 Hz | 5 | 500 | 0.001 | ADXL355 DRDY → SPI1 DMA read |
| `accel_dma` | DMA1_CH1 | 500 Hz | 5 | 500 | 0.001 | decode → seismic queue |
| `supervisor` | CORDIC | 1 kHz | 4 | 17 000 | 0.100 | `SupervisorGroup::run`, watchdog feed |
| `seismic` | I2C4_EV | 100 Hz | 3 | 17 000 | 0.010 | `SeismicGroup::run` |
| `can_rx` | FDCAN1_IT0 | event | 3 | 3 000 | (0.018 at 1 kHz) | commands and alerts into queues |
| `telemetry` | software (dispatcher LPTIM1; UCPD1 spare) | 100 Hz | 1 | remainder | — | frames, events, link status |
| `idle` | — | — | 0 | — | — | WFI |

The budgets are the core design §F design values at 170 MHz, not measurements:
ΣC/T = 0.64 over the periodic tasks (0.66 with CAN frames at 1 kHz), below the 0.70
design limit (FAB-FW-008); the periods are harmonic, so the rate-monotonic bound is 1.0.

The base task snapshots every released group's LET inputs at the release tick, so a
lower-priority job that starts late still reads exactly what `Executive::on_base_tick`
would have read. A job unfinished at its next release is reported through
`report_overrun` (FastOverrun / SupervisorOverrun / SeismicOverrun, SAF-013), which also
withholds the watchdog feed. DWT cycle maxima per task are sent in the link status frame.
The largest mailbox copies are the supervisor output (≈ 390 B) and the fast and
supervisor inputs (≈ 340 B), made under locks at the base priority.

## Boot sequence

1. Gate enables preset low; the crash record in `.uninit` and the RCC reset flags are read.
2. 170 MHz from the 24 MHz HSE (boost mode), pins, DWT cycle counter, SysTick monotonic.
3. SELFTEST before any watchdog runs: FPSCR FZ = DN = 0 and
   `fabrail_rt::selftest_hash(2000) == GOLDEN_SELFTEST_HASH` (plan §2b item 15).
4. Parameters from the flash A/B pages (postcard + CRC32), or the factory preset of the
   build class if both pages are erased; safety seal, `validate`, class, voltage coil
   mode and 20 kHz base rate are checked.
5. Peripherals come up in every case, so the power stages are held safe: hardware trips,
   TIM1 in STO, HRTIM outputs disabled, ADCs, encoder, SPI sensors and FDCAN.
6. If everything passed, the executive is split into the tasks, the ADC triggers are
   armed (the base interrupt starts at the next PWM peak) and IWDG (≈ 5 ms) and WWDG
   start. Otherwise the firmware stays in a **boot hold**: no control task runs, the
   outputs stay off and the link status frame (CAN 0x320) reports the reason.

A crash or watchdog hold clears only on a `Reset` command while every gap reads at
rest: ≥ g_land, or ≥ g_seat for a seated DRAWER, minus 3σ of the gap-sensor noise. The
Reset clears the crash record and resets the MCU. A power cycle loses the record, since
SRAM is not retained.

**`GOLDEN_SELFTEST_HASH` is the placeholder 0 until Wave 3 (C1) freezes it.** Until
then every image boots into the SELFTEST hold by design: the firmware refuses to run
control code that has not been shown to match the simulator bit for bit.

## Peripherals

| Function | Peripheral | Notes |
|---|---|---|
| LSM inverter | TIM1 CH1–3 + CH1N–3N, centre-aligned 20 kHz, 500 ns dead time | STO = MOE clear + gate enable low; ASC = low sides on; BKIN2 ← COMP1–3, BKIN ← COMP7 |
| LSM current loop | `fabrail_core::foc::DqPi` in the inverter driver (`hal_impl.rs`) | min–max zero-sequence injection, nominal V_bus |
| Coil bridges (≤ 8) | HRTIM TA–TD outputs 1/2, locked anti-phase, synchronised to TIM1_TRGO | faults FLT2 ← COMP4, FLT6 ← COMP5, FLT3 ← COMP6, FLT1 ← PA12 |
| Currents | ADC1+ADC2 dual injected (phases, bus), ADC3/ADC4 injected (coils) | hardware mid-scale offset, signed codes |
| Gap probes, power sense | ADC5 / ADC3 / ADC4 regular scans, circular DMA | 16× oversampling (16-bit gaps, 12-bit averaged power) |
| Encoder | TIM2 32-bit ×4 quadrature, extended to i64 | zero at the first reference mark |
| Structure accelerometer | ADXL355 on SPI1 + DMA1, DRDY on EXTI6 | 500 Hz ODR, ±2 g |
| Carriage IMU, rail accelerometer | LSM6DSO-class on SPI3 + DMA2 | polled at fast releases |
| Link | FDCAN1, 1 Mbit/s nominal, FD frames without BRS | "fabrail link v0", documented in `bsp/can.rs` |
| Parameters | flash pages 0x0807_E000 / 0x0807_F000 | `memory.x` asserts that the image ends below them |
| Watchdogs | IWDG (LSI, `supervisor.watchdog_s`), WWDG (PCLK, 12.3 ms, open window) | fed only under `fabrail_rt::WatchdogRule` |

The digital pin map (`bsp/pins.rs`) fits the LQFP64 STM32G474RE. The full analog front
end needs about 30 ADC inputs and the comparator inputs on ports D/E, so it needs the
LQFP100/128 package of the same die (same memory map, PAC and image). The NUCLEO-G474RE
bring-up kit carries only a DOOR-sized subset.

Two corrections to core design §F:

- COMP7 has no HRTIM fault path on the G474. It trips the LSM on BKIN (DC bus), and
  bridges 6–7 use FLT1 from an external comparator.
- Coil PWM is locked anti-phase, one HRTIM output per bridge. Eight coils therefore fit
  on timers A–D.

## Size

Measured with `llvm-size` on 2026-09-27, during Wave 2, while the shared crates were
still changing. Release profile (W2 lead decision D9): opt-level `"z"` for the whole
image, fat LTO, one codegen unit, `panic = "abort"`, no defmt (defmt, RTT and
panic-probe exist only with the `dev` feature).

| Class | Flash (text + data) | Static RAM (data + bss) |
|---|---|---|
| door | 279,872 B | 22,352 B |
| drawer | 282,176 B | 22,352 B |
| wall | 282,000 B | 22,352 B |
| heavy | 282,096 B | 22,352 B |

Door image at other settings: opt-level `"s"` (the Wave 1 profile) 320,068 B on the same
tree an hour earlier, when `"z"` gave 279,528 B; opt-level 3 ≈ 400 KB (Wave 1). Earlier
the same day, a split of levels (`"s"` for the ISR crates core, rt, safety, types and libm;
`"z"` elsewhere) saved only 3.8 KB over all-`"s"`, because the init-time code lives in the
same crates as the ISR paths; `-C target-cpu=cortex-m4` and lower LLVM inline thresholds
made the `"z"` image 0.2–2.2 KB larger.

**Budget: flash ≤ 320 KiB, RAM ≤ 64 KiB** (`scripts/check_firmware.sh`, FAB-FW-010).
The design-stage budget was 256 KiB: one 256 KiB bank of the STM32G474RE's 512 KiB
dual-bank flash, which would leave the other bank free for an A/B firmware image. That is
not reached, and the budget was changed instead of claimed:

- The rest of the image is init-time code of the shared crates, which runs once at boot:
  - the f64 soft-float gain design in `Executive::new` / `FastLoop::new` (the M4F has
    single-precision hardware only);
  - the 368-field `meta` range table that `validate` walks, with its path, unit and
    description strings (the table alone is ≈ 15 KB; with the strings it is most of the
    ≈ 49 KB of `.rodata`);
  - the postcard decoder of `SystemParams` (≈ 20 KB at `"z"`).
- Getting under 256 KiB needs changes outside this crate: a params feature that drops the
  `meta` strings from firmware builds, or a parameter-page format without the serde
  decoder (an override list against the factory preset). Both are open items for the
  params and firmware owners, not done here.
- 320 KiB is 62.5 % of the device and leaves ≈ 15 % headroom over today's image for the
  rest of the project (the shared crates grew ≈ 12 KB during Wave 2 alone). The hard
  limit is 504 KiB: the top 8 KiB hold the parameter pages, and `memory.x` asserts at
  link time that the image ends below them. A/B firmware images do not fit and are
  roadmap.

The ISR paths are built at `"z"` too. The cycle budgets in the task map are core design
§F estimates for optimised code, not measurements, and `"z"` code is usually somewhat
slower than `"s"` or 3. If DWT measurements on hardware show a group over its budget, the
`speed` profile (opt 3, not size-checked) is the fallback and this budget is re-derived.

## Not verified here (needs hardware)

- Flashing and bring-up.
- Cycle budgets. Design utilisation is ≤ 0.70; the DWT maxima are reported for when a
  board exists.
- ADC sampling at the PWM peak.
- COMP → break/fault latency < 1 µs.
- The CAN bus.
- ADXL355 and IMU communication.
- Flash programming.
- Watchdog timing.
- The CiA 402 mapping. It is roadmap, and so are the 5 Mbit/s data phase and the
  diverse STM32G0 monitor channel.
