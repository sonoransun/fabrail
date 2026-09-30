//! selftest_hash (LCG raw inputs, FNV-1a over output bits) and GOLDEN_SELFTEST_HASH (frozen by C1). Owner: A5 (constant frozen by C1).
//!
//! API (W0; core design §A.4 item 7, §F; PHYSICS §24): [`selftest_hash`]`(n_ticks)`
//! drives an [`Executive`] built from [`selftest_params`] (the DOOR preset in voltage
//! coil mode — SIL ≡ firmware is claimed in voltage mode only, plan §2b item 16) against
//! the deterministic [`LcgBoard`], and FNV-1a-hashes the `to_bits()` of every output the
//! executive issues through the HAL (coil and LSM commands, aux commands, telemetry
//! frames and events, watchdog feeds). The firmware boots into SELFTEST, recomputes
//! `selftest_hash(SELFTEST_TICKS)` (≈ 65 ms, before the IWDG starts; plan §2b item 15),
//! asserts FPSCR FZ = DN = 0 and compares with [`GOLDEN_SELFTEST_HASH`]; a mismatch
//! raises `SelftestHash`. The browser wasm build shows the same hash (docs site badge).
//!
//! The board's raw inputs sit near a DOOR hover (gaps at g* = 5 mm, small current and
//! encoder noise, 11.5 V hold-up, mains present) so the run exercises the nominal
//! control and monitor paths, not only the fault reactions. Each HAL group draws from
//! its own LCG stream (base, supervisor, seismic), so the inputs a group sees do not
//! depend on the order in which the groups run within a tick (the LET permutation test
//! relies on this).
//!
//! The hash must be identical with and without the `fault-injection` feature (no
//! injection happens here). The DOOR preset is used whatever class the firmware is
//! built for, so there is exactly one golden value.
//!
//! Constants: FNV-1a 64-bit offset basis 0xcbf29ce484222325 and prime 0x100000001b3
//! (Fowler–Noll–Vo, IETF draft-eastlake-fnv); LCG multiplier 6364136223846793005 and
//! increment 1442695040888963407 (Knuth, MMIX).

use fabrail_hal::{
    Accelerometer, AlertLink, Aux, CarriageImu, Clock, CoilDrivers, CommandLink, Encoder,
    GapSensors, LsmDriver, PowerMonitor, RailAccel, SafetyInputs, Telemetry, Watchdog,
};
use fabrail_params::{SystemParams, presets};
use fabrail_types::{
    ACCEL_DRAIN_MAX, AccelRaw, AuxCmd, CoilCmd, CurrentRaw, DriverStatus, EncoderRaw, Event,
    GapRaw, ImuRaw, LsmCmd, LsmRaw, NetworkAlert, PowerRaw, RailAccelRaw, ResetCause, SafetyInRaw,
    TelemetryFrame, Tick, UserCommand,
};

use crate::exec::Executive;

/// Placeholder until Wave 3 (C1) freezes the value printed by `tests/golden.rs`.
pub const GOLDEN_SELFTEST_HASH: u64 = 0;

/// Ticks the boot selftest runs (0.1 s of base ticks).
pub const SELFTEST_TICKS: u32 = 2000;

/// Returned when the selftest executive cannot be built (parameters rejected). It is
/// never a valid golden value.
pub const SELFTEST_ERROR_HASH: u64 = u64::MAX;

/// FNV-1a 64-bit hasher.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fnv1a(pub u64);

impl Default for Fnv1a {
    fn default() -> Fnv1a {
        Fnv1a::new()
    }
}

impl Fnv1a {
    /// FNV-1a 64-bit offset basis.
    pub const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    /// FNV-1a 64-bit prime.
    pub const PRIME: u64 = 0x0000_0100_0000_01b3;

    /// A fresh hasher.
    pub const fn new() -> Fnv1a {
        Fnv1a(Fnv1a::OFFSET)
    }

    /// Hashes bytes.
    pub fn write(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.0 ^= *b as u64;
            self.0 = self.0.wrapping_mul(Fnv1a::PRIME);
        }
    }

    /// Hashes a u32 (little-endian).
    pub fn write_u32(&mut self, v: u32) {
        self.write(&v.to_le_bytes());
    }

    /// Hashes a u64 (little-endian).
    pub fn write_u64(&mut self, v: u64) {
        self.write(&v.to_le_bytes());
    }

    /// Hashes an f32 by its bit pattern.
    pub fn write_f32(&mut self, v: f32) {
        self.write_u32(v.to_bits());
    }

    /// The hash value.
    pub const fn finish(&self) -> u64 {
        self.0
    }
}

/// The selftest parameter set: the DOOR preset in voltage coil mode.
pub fn selftest_params() -> SystemParams {
    let mut p = presets::door();
    p.control.rates.coil_mode_voltage = true;
    p
}

/// Runs the executive for `n_ticks` base ticks on the LCG board and returns the FNV-1a
/// hash of every HAL output ([`SELFTEST_ERROR_HASH`] if the executive cannot be built).
pub fn selftest_hash(n_ticks: u32) -> u64 {
    selftest_hash_with(&selftest_params(), n_ticks)
}

/// [`selftest_hash`] for an arbitrary parameter set (tests and diagnostics).
pub fn selftest_hash_with(p: &SystemParams, n_ticks: u32) -> u64 {
    let Ok(mut exec) = Executive::new(p) else {
        return SELFTEST_ERROR_HASH;
    };
    let mut board = LcgBoard::new();
    for _ in 0..n_ticks {
        exec.on_base_tick(&mut board);
        board.advance();
    }
    board.hash()
}

/// One 64-bit LCG stream (Knuth MMIX constants).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Lcg(u64);

impl Lcg {
    const A: u64 = 6_364_136_223_846_793_005;
    const C: u64 = 1_442_695_040_888_963_407;

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(Lcg::A).wrapping_add(Lcg::C);
        self.0
    }

    /// 16 pseudo-random bits from the top of the state.
    fn next_u16(&mut self) -> u16 {
        (self.next() >> 48) as u16
    }

    fn next_i16(&mut self) -> i16 {
        self.next_u16() as i16
    }
}

/// Deterministic selftest board: LCG raw inputs (one stream per HAL group), every
/// output hashed with FNV-1a. Also used by the LET permutation and golden tests.
#[derive(Clone, Debug)]
pub struct LcgBoard {
    tick: Tick,
    base: Lcg,
    sup: Lcg,
    seis: Lcg,
    counts: i64,
    feeds: u32,
    h: Fnv1a,
}

impl Default for LcgBoard {
    fn default() -> LcgBoard {
        LcgBoard::new()
    }
}

impl LcgBoard {
    /// DOOR hover near g* = 5 mm on the 16-bit / 20 mm gap sensor (code 16 384).
    const GAP_CODE: u16 = 16_384;
    /// 48 V bus at 60 V / 4096 codes.
    const V_BUS_CODE: u16 = 3_277;
    /// 11.5 V hold-up at 15 V / 4096 codes.
    const V_HOLD_CODE: u16 = 3_140;
    /// 1 g on the ADXL355 (3.9 µg/LSB).
    const ACCEL_1G: i32 = 256_000;
    /// 1 g on the ±8 g, 16-bit IMU and rail accelerometer.
    const IMU_1G: i32 = 4_096;

    /// A board at tick 0 with fixed seeds.
    pub fn new() -> LcgBoard {
        LcgBoard {
            tick: Tick(0),
            base: Lcg(0x0123_4567_89ab_cdef),
            sup: Lcg(0x0fed_cba9_8765_4321),
            seis: Lcg(0x5a5a_a5a5_3c3c_c3c3),
            counts: 0,
            feeds: 0,
            h: Fnv1a::new(),
        }
    }

    /// Advances the clock by one base tick.
    pub fn advance(&mut self) {
        self.tick = self.tick.plus(1);
    }

    /// Advances the clock by `n` base ticks without running the executive (a deadline
    /// miss in tests).
    pub fn skip(&mut self, n: u64) {
        self.tick = self.tick.plus(n);
    }

    /// The board's clock.
    pub fn now_tick(&self) -> Tick {
        self.tick
    }

    /// Watchdog feeds so far.
    pub fn feeds(&self) -> u32 {
        self.feeds
    }

    /// FNV-1a hash of every output so far.
    pub fn hash(&self) -> u64 {
        self.h.finish()
    }
}

impl Clock for LcgBoard {
    fn now(&self) -> Tick {
        self.tick
    }
}

impl CoilDrivers for LcgBoard {
    fn sample(&mut self) -> CurrentRaw {
        let mut c = CurrentRaw::default();
        for x in c.code.iter_mut().take(4) {
            *x = self.base.next_i16() >> 12;
        }
        c
    }
    fn command(&mut self, c: &CoilCmd) {
        let tag: u32 = match c {
            CoilCmd::Off => 0,
            CoilCmd::CurrentRef(_) => 1,
            CoilCmd::Voltage(_) => 2,
        };
        self.h.write_u32(tag);
        for v in c.values() {
            self.h.write_f32(v);
        }
    }
    fn status(&self) -> DriverStatus {
        DriverStatus {
            enabled: true,
            ..DriverStatus::default()
        }
    }
}

impl GapSensors for LcgBoard {
    fn sample(&mut self) -> GapRaw {
        let mut g = GapRaw {
            valid: 0x0F,
            ..GapRaw::default()
        };
        for (a, b) in g.code.iter_mut().zip(g.code_b.iter_mut()).take(4) {
            let v = LcgBoard::GAP_CODE.wrapping_add_signed(self.base.next_i16() >> 10);
            *a = v;
            *b = v;
        }
        g
    }
}

impl Encoder for LcgBoard {
    fn sample(&mut self) -> EncoderRaw {
        self.counts = self
            .counts
            .wrapping_add((self.base.next_u16() % 5) as i64 - 2);
        EncoderRaw {
            counts: self.counts,
            referenced: true,
            err: false,
        }
    }
}

impl LsmDriver for LcgBoard {
    fn sample(&mut self) -> LsmRaw {
        let a = self.base.next_i16() >> 12;
        let b = self.base.next_i16() >> 12;
        LsmRaw {
            i_abc_code: [a, b, a.wrapping_neg().wrapping_sub(b)],
            fault: false,
        }
    }
    fn command(&mut self, c: &LsmCmd) {
        self.h.write_f32(c.iq_ref_a);
        self.h.write_f32(c.id_ref_a);
        self.h.write_u32(c.mode.as_u8() as u32);
    }
    fn status(&self) -> DriverStatus {
        DriverStatus {
            enabled: true,
            ..DriverStatus::default()
        }
    }
}

impl CarriageImu for LcgBoard {
    fn sample(&mut self) -> ImuRaw {
        ImuRaw {
            xyz: [
                (self.base.next_i16() >> 8) as i32,
                (self.base.next_i16() >> 8) as i32,
                LcgBoard::IMU_1G + (self.base.next_i16() >> 8) as i32,
            ],
            present: true,
        }
    }
}

impl RailAccel for LcgBoard {
    fn sample(&mut self) -> RailAccelRaw {
        RailAccelRaw {
            xyz: [
                (self.base.next_i16() >> 8) as i32,
                (self.base.next_i16() >> 8) as i32,
                LcgBoard::IMU_1G + (self.base.next_i16() >> 8) as i32,
            ],
            present: true,
        }
    }
}

impl Accelerometer for LcgBoard {
    fn drain(&mut self, out: &mut [AccelRaw; ACCEL_DRAIN_MAX]) -> usize {
        // 500 Hz ODR drained at 100 Hz: 5 samples per release.
        let base_seq = (self.tick.0 / 40) as u32;
        let n = 5usize.min(ACCEL_DRAIN_MAX);
        for (k, s) in out.iter_mut().take(n).enumerate() {
            *s = AccelRaw {
                xyz: [
                    (self.seis.next_i16() >> 6) as i32,
                    (self.seis.next_i16() >> 6) as i32,
                    LcgBoard::ACCEL_1G + (self.seis.next_i16() >> 6) as i32,
                ],
                seq: base_seq.wrapping_add(k as u32),
                overrun: false,
            };
        }
        n
    }
}

impl AlertLink for LcgBoard {
    fn poll(&mut self) -> Option<NetworkAlert> {
        None
    }
}

impl PowerMonitor for LcgBoard {
    fn sample(&mut self) -> PowerRaw {
        PowerRaw {
            v_bus_code: LcgBoard::V_BUS_CODE + (self.sup.next_u16() >> 12),
            v_hold_code: LcgBoard::V_HOLD_CODE + (self.sup.next_u16() >> 12),
            i_bus_code: self.sup.next_i16() >> 8,
            mains_ok: true,
            temp_code: [700, 650],
        }
    }
}

impl SafetyInputs for LcgBoard {
    fn sample(&mut self) -> SafetyInRaw {
        SafetyInRaw {
            presence_ok: true,
            handle_force_code: self.sup.next_i16() >> 8,
            ..SafetyInRaw::default()
        }
    }
}

impl Aux for LcgBoard {
    fn command(&mut self, c: &AuxCmd) {
        self.h
            .write_u32(c.parking_lock as u32 | (c.seismic_latch as u32) << 1);
    }
}

impl CommandLink for LcgBoard {
    fn poll(&mut self) -> Option<UserCommand> {
        // A fixed command script exercising lift, open, close and stop.
        match self.tick.0 {
            100 => Some(UserCommand::Lift),
            600 => Some(UserCommand::Open),
            1_400 => Some(UserCommand::Close),
            1_900 => Some(UserCommand::Stop),
            _ => None,
        }
    }
}

impl Watchdog for LcgBoard {
    fn feed(&mut self) {
        self.feeds = self.feeds.wrapping_add(1);
        self.h.write_u64(self.tick.0);
    }
    fn reset_cause(&self) -> ResetCause {
        ResetCause::PowerOn
    }
}

impl Telemetry for LcgBoard {
    fn frame(&mut self, f: &TelemetryFrame) {
        self.h.write(&f.to_bytes());
    }
    fn event(&mut self, e: &Event) {
        self.h.write_u64(e.tick.0);
        self.h.write_u32(e.kind.as_u8() as u32);
        self.h.write_f32(e.a);
        self.h.write_f32(e.b);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fnv1a_reference_vectors() {
        // FNV-1a 64 test vectors (draft-eastlake-fnv): "" and "a".
        assert_eq!(Fnv1a::new().finish(), 0xcbf2_9ce4_8422_2325);
        let mut h = Fnv1a::new();
        h.write(b"a");
        assert_eq!(h.finish(), 0xaf63_dc4c_8601_ec8c);
    }

    #[test]
    fn selftest_is_deterministic() {
        assert_eq!(selftest_hash(64), selftest_hash(64));
        assert_ne!(selftest_hash(64), SELFTEST_ERROR_HASH);
    }

    /// Drives the full executive (all four groups, LET mailboxes, monitor and
    /// arbitration) for 4 000 ticks with explicit rates, independent of the preset
    /// values: deterministic and never the error sentinel.
    #[test]
    fn executive_runs_with_explicit_rates() {
        let mut p = selftest_params();
        p.control.rates = fabrail_params::Rates {
            base_hz: 20_000,
            fast_decim: 2,
            sup_decim: 20,
            seismic_decim: 200,
            coil_mode_voltage: true,
        };
        let a = selftest_hash_with(&p, 4_000);
        assert_ne!(a, SELFTEST_ERROR_HASH);
        assert_eq!(a, selftest_hash_with(&p, 4_000));
        // A second valid rate plan (fast = base). fast_decim = 4 is rejected by
        // validate(): the DOOR detent stiffness exceeds the HAP-009 passivity bound
        // at T = 200 µs, so the executive would (correctly) refuse to build.
        p.control.rates.fast_decim = 1;
        assert_ne!(selftest_hash_with(&p, 4_000), SELFTEST_ERROR_HASH);
    }

    /// The selftest board sits inside the monitor's envelope: no monitor fault in the
    /// verdict over the whole selftest run (the hash covers the nominal paths).
    #[test]
    fn selftest_board_is_nominal_for_the_monitor() {
        let p = selftest_params();
        let Ok(mut ex) = Executive::new(&p) else {
            panic!("selftest params rejected");
        };
        let mut b = LcgBoard::new();
        for _ in 0..SELFTEST_TICKS {
            ex.on_base_tick(&mut b);
            b.advance();
            let v = ex.snapshot().fast.verdict;
            assert!(v.faults.is_empty(), "t={}: {:?}", ex.tick.0, v.faults);
        }
        assert!(b.feeds() > 0, "the watchdog is fed");
    }
}
