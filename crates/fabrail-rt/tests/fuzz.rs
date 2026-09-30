//! Fuzzed raw inputs through the whole executive: no panic and no NaN reaches an
//! actuator (FAB-SAF-017; firmware panic policy; core design §E.2).
//!
//! A board feeds random HAL inputs (SplitMix64 streams, Steele et al. 2014) to the
//! executive of every class preset (voltage coil mode) together with a random command
//! and alert script. Two regimes: `Bounded` draws codes around a plausible operating
//! point (gaps near g*, currents and encoder steps within the converters' ranges), and
//! `Wild` draws every integer field over its full range, flags at random, and alert
//! payloads from arbitrary f32 bit patterns. Every coil and LSM command handed to the
//! HAL must be finite, and the verdict's force cap finite and non-negative.

use fabrail_core::seismic::action_policy::CorePolicy;
use fabrail_hal::{
    Accelerometer, AlertLink, Aux, CarriageImu, Clock, CoilDrivers, CommandLink, Encoder,
    GapSensors, LsmDriver, PowerMonitor, RailAccel, SafetyInputs, Telemetry, Watchdog,
};
use fabrail_params::presets;
use fabrail_rt::{Executive, GroupId};
use fabrail_types::{
    ACCEL_DRAIN_MAX, AccelRaw, AuxCmd, CoilCmd, CurrentRaw, DriverStatus, EncoderRaw, Event,
    GapRaw, ImuRaw, LsmCmd, LsmRaw, NetworkAlert, PosNm, PowerRaw, RailAccelRaw, ResetCause,
    SafetyInRaw, TelemetryFrame, Tick, UserCommand,
};
use proptest::prelude::*;

/// SplitMix64 (Steele, Lea & Flood 2014).
#[derive(Clone, Copy, Debug)]
struct SplitMix(u64);

impl SplitMix {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn below(&mut self, n: u64) -> u64 {
        if n == 0 { 0 } else { self.next() % n }
    }
    fn i16(&mut self) -> i16 {
        self.next() as i16
    }
    fn chance(&mut self, one_in: u64) -> bool {
        self.below(one_in) == 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Regime {
    Bounded,
    Wild,
}

/// Random-input board that checks every actuator command it receives.
struct FuzzBoard {
    tick: Tick,
    rng: SplitMix,
    regime: Regime,
    gap_code: u16,
    counts: i64,
    bad_commands: u32,
    commands: u32,
}

impl FuzzBoard {
    fn new(seed: u64, regime: Regime, gap_code: u16) -> FuzzBoard {
        FuzzBoard {
            tick: Tick(0),
            rng: SplitMix(seed),
            regime,
            gap_code,
            counts: 50_000,
            bad_commands: 0,
            commands: 0,
        }
    }
    fn wild(&self) -> bool {
        self.regime == Regime::Wild
    }
    fn any_f32(&mut self) -> f32 {
        f32::from_bits(self.rng.next() as u32)
    }
}

impl Clock for FuzzBoard {
    fn now(&self) -> Tick {
        self.tick
    }
}

impl CoilDrivers for FuzzBoard {
    fn sample(&mut self) -> CurrentRaw {
        let mut c = CurrentRaw::default();
        for x in c.code.iter_mut() {
            *x = if self.wild() {
                self.rng.i16()
            } else {
                self.rng.i16() >> 6
            };
        }
        c
    }
    fn command(&mut self, c: &CoilCmd) {
        self.commands += 1;
        if c.values().iter().any(|v| !v.is_finite()) {
            self.bad_commands += 1;
        }
    }
    fn status(&self) -> DriverStatus {
        DriverStatus {
            enabled: true,
            ..DriverStatus::default()
        }
    }
}

impl GapSensors for FuzzBoard {
    fn sample(&mut self) -> GapRaw {
        let mut g = GapRaw::default();
        if self.wild() {
            for (a, b) in g.code.iter_mut().zip(g.code_b.iter_mut()) {
                *a = self.rng.next() as u16;
                *b = self.rng.next() as u16;
            }
            g.valid = self.rng.next() as u8;
        } else {
            for (a, b) in g.code.iter_mut().zip(g.code_b.iter_mut()) {
                let v = self.gap_code.wrapping_add_signed(self.rng.i16() >> 4);
                *a = v;
                *b = v.wrapping_add_signed(self.rng.i16() >> 12);
            }
            g.valid = 0xFF;
        }
        g
    }
}

impl Encoder for FuzzBoard {
    fn sample(&mut self) -> EncoderRaw {
        if self.wild() {
            self.counts = self.rng.next() as i64;
            EncoderRaw {
                counts: self.counts,
                referenced: self.rng.chance(2),
                err: self.rng.chance(8),
            }
        } else {
            self.counts = self.counts.wrapping_add((self.rng.below(41) as i64) - 20);
            EncoderRaw {
                counts: self.counts,
                referenced: true,
                err: false,
            }
        }
    }
}

impl LsmDriver for FuzzBoard {
    fn sample(&mut self) -> LsmRaw {
        if self.wild() {
            LsmRaw {
                i_abc_code: [self.rng.i16(), self.rng.i16(), self.rng.i16()],
                fault: self.rng.chance(16),
            }
        } else {
            let a = self.rng.i16() >> 5;
            let b = self.rng.i16() >> 5;
            LsmRaw {
                i_abc_code: [a, b, a.wrapping_neg().wrapping_sub(b)],
                fault: false,
            }
        }
    }
    fn command(&mut self, c: &LsmCmd) {
        self.commands += 1;
        if !(c.iq_ref_a.is_finite() && c.id_ref_a.is_finite()) {
            self.bad_commands += 1;
        }
    }
    fn status(&self) -> DriverStatus {
        DriverStatus {
            enabled: true,
            ..DriverStatus::default()
        }
    }
}

impl CarriageImu for FuzzBoard {
    fn sample(&mut self) -> ImuRaw {
        let s = if self.wild() { 0 } else { 6 };
        ImuRaw {
            xyz: [
                (self.rng.next() as i32) >> (16 + s),
                (self.rng.next() as i32) >> (16 + s),
                4_096 + ((self.rng.next() as i32) >> (16 + s)),
            ],
            present: !self.wild() || self.rng.chance(2),
        }
    }
}

impl RailAccel for FuzzBoard {
    fn sample(&mut self) -> RailAccelRaw {
        let s = if self.wild() { 0 } else { 6 };
        RailAccelRaw {
            xyz: [
                (self.rng.next() as i32) >> (16 + s),
                (self.rng.next() as i32) >> (16 + s),
                4_096 + ((self.rng.next() as i32) >> (16 + s)),
            ],
            present: !self.wild() || self.rng.chance(2),
        }
    }
}

impl Accelerometer for FuzzBoard {
    fn drain(&mut self, out: &mut [AccelRaw; ACCEL_DRAIN_MAX]) -> usize {
        let n = if self.wild() {
            self.rng.below(ACCEL_DRAIN_MAX as u64 + 1) as usize
        } else {
            5
        };
        for (k, s) in out.iter_mut().take(n).enumerate() {
            let xyz = if self.wild() {
                [
                    (self.rng.next() as i32) >> 12,
                    (self.rng.next() as i32) >> 12,
                    (self.rng.next() as i32) >> 12,
                ]
            } else {
                [
                    self.rng.i16() as i32 >> 4,
                    self.rng.i16() as i32 >> 4,
                    256_000 + (self.rng.i16() as i32 >> 4),
                ]
            };
            *s = AccelRaw {
                xyz,
                seq: (self.tick.0 / 40) as u32 + k as u32,
                overrun: self.wild() && self.rng.chance(4),
            };
        }
        n
    }
}

impl AlertLink for FuzzBoard {
    fn poll(&mut self) -> Option<NetworkAlert> {
        if !self.rng.chance(150) {
            return None;
        }
        Some(if self.wild() {
            NetworkAlert {
                id: self.rng.next() as u32,
                received: self.tick,
                origin_offset_s: self.any_f32(),
                magnitude: self.any_f32(),
                r_epi_km: self.any_f32(),
                depth_km: self.any_f32(),
                mmi: self.any_f32(),
                cancel: self.rng.chance(3),
            }
        } else {
            NetworkAlert {
                id: 1 + self.rng.below(3) as u32,
                received: self.tick,
                origin_offset_s: -(self.rng.below(20) as f32),
                magnitude: 4.0 + self.rng.below(50) as f32 / 10.0,
                r_epi_km: 5.0 + self.rng.below(200) as f32,
                depth_km: 5.0 + self.rng.below(30) as f32,
                mmi: 3.0 + self.rng.below(60) as f32 / 10.0,
                cancel: self.rng.chance(4),
            }
        })
    }
}

impl PowerMonitor for FuzzBoard {
    fn sample(&mut self) -> PowerRaw {
        if self.wild() {
            PowerRaw {
                v_bus_code: self.rng.next() as u16,
                v_hold_code: self.rng.next() as u16,
                i_bus_code: self.rng.i16(),
                mains_ok: self.rng.chance(2),
                temp_code: [self.rng.next() as u16, self.rng.next() as u16],
            }
        } else {
            PowerRaw {
                v_bus_code: 3_000 + (self.rng.next() as u16 >> 8),
                v_hold_code: 3_000 + (self.rng.next() as u16 >> 8),
                i_bus_code: self.rng.i16() >> 6,
                mains_ok: !self.rng.chance(500),
                temp_code: [700, 650],
            }
        }
    }
}

impl SafetyInputs for FuzzBoard {
    fn sample(&mut self) -> SafetyInRaw {
        let w = self.wild();
        SafetyInRaw {
            presence_ok: !w || self.rng.chance(2),
            presence_active: w && self.rng.chance(2),
            handle_force_code: if w {
                self.rng.i16()
            } else {
                self.rng.i16() >> 8
            },
            has_handle_cell: w && self.rng.chance(2),
            fire_alarm: w && self.rng.chance(8),
            estop: w && self.rng.chance(16),
            lock_engaged: w && self.rng.chance(2),
            hold_to_run: self.rng.chance(2),
        }
    }
}

impl Aux for FuzzBoard {
    fn command(&mut self, _c: &AuxCmd) {}
}

impl CommandLink for FuzzBoard {
    fn poll(&mut self) -> Option<UserCommand> {
        if !self.rng.chance(120) {
            return None;
        }
        Some(match self.rng.below(10) {
            0 => UserCommand::Open,
            1 => UserCommand::Close,
            2 => UserCommand::GoTo(PosNm(self.rng.next() as i64 >> 20)),
            3 => UserCommand::Stop,
            4 => UserCommand::Lift,
            5 => UserCommand::Land,
            6 => UserCommand::Reset,
            7 => UserCommand::Manual,
            8 => UserCommand::HoldToRun(true),
            _ => UserCommand::HoldToRun(false),
        })
    }
}

impl Watchdog for FuzzBoard {
    fn feed(&mut self) {}
    fn reset_cause(&self) -> ResetCause {
        ResetCause::PowerOn
    }
}

impl Telemetry for FuzzBoard {
    fn frame(&mut self, f: &TelemetryFrame) {
        let _ = f.to_bytes();
    }
    fn event(&mut self, _e: &Event) {}
}

/// Gap code of g* on the preset's sensor polynomial (linear term).
fn g_star_code(p: &fabrail_params::SystemParams) -> u16 {
    let poly = p.design.sensors.gap_poly_m;
    if poly[1] > 0.0 {
        fabrail_types::fmath::round((p.design.suspension.g_star_m - poly[0]) / poly[1]) as u16
    } else {
        16_384
    }
}

fn run_case(seed: u64, class: usize, regime: Regime, ticks: u32) -> (u32, u32) {
    let make = [
        presets::drawer,
        presets::door,
        presets::wall,
        presets::heavy,
    ];
    let mut p = make[class % 4]();
    p.control.rates.coil_mode_voltage = true;
    let mut ex = Executive::new(&p).unwrap_or_else(|e| panic!("preset rejected: {e}"));
    let mut b = FuzzBoard::new(seed, regime, g_star_code(&p));
    let order = GroupId::ALL_ORDERS[(seed % 6) as usize];
    for _ in 0..ticks {
        ex.on_base_tick_in_order(&mut b, &mut CorePolicy, order);
        let v = ex.snapshot().fast.verdict;
        assert!(
            v.force_cap_n.is_finite() && v.force_cap_n >= 0.0,
            "(T) finite cap at t={}: {}",
            b.tick.0,
            v.force_cap_n
        );
        // Occasional clock jumps (deadline misses) in the wild regime.
        if regime == Regime::Wild && b.rng.chance(997) {
            b.tick = b.tick.plus(1 + b.rng.below(100));
        } else {
            b.tick = b.tick.plus(1);
        }
    }
    (b.bad_commands, b.commands)
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 24,
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    // req: FAB-SAF-017
    #[test]
    fn no_panic_no_nan(seed in any::<u64>(), class in 0usize..4, wild in any::<bool>()) {
        let regime = if wild { Regime::Wild } else { Regime::Bounded };
        let (bad, n) = run_case(seed, class, regime, 1_200);
        prop_assert!(n > 0, "the executive commanded the actuators");
        prop_assert_eq!(bad, 0, "(T) no NaN/inf actuator command ({:?}, class {}), FAB-SAF-017", regime, class);
    }
}

/// Every class and both regimes at least once, longer (deterministic seeds).
#[test]
fn every_class_both_regimes() {
    for class in 0..4 {
        for regime in [Regime::Bounded, Regime::Wild] {
            let (bad, n) = run_case(0x5eed_0000 + class as u64, class, regime, 4_000);
            assert!(n > 0);
            assert_eq!(
                bad, 0,
                "(T) no NaN actuator output: class {class} {regime:?}"
            );
        }
    }
}
