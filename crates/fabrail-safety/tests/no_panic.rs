//! Fuzzed raw inputs never panic the monitor and never produce a NaN verdict
//! (FAB-SAF-017; core design §E.2; firmware panic policy, TECHNICAL_DESIGN).
//!
//! Every raw field takes its full integer range, core commands take arbitrary f32 bit
//! patterns (NaN, ±inf, subnormals), ticks may jump or repeat, and the limits come from
//! the four presets or from arbitrary f64 bit patterns (NaN, ±inf, negative). The
//! verdict must stay finite and non-negative, and a non-finite command must always be
//! reported as NonFinite.
#![allow(clippy::float_arithmetic)]

use fabrail_params::{IsolateLimits, SafetyLimits, SensorCal, presets};
use fabrail_safety::Monitor;
use fabrail_types::{
    CoilCmd, CurrentRaw, EncoderRaw, FastRaw, FaultCode, GapRaw, ImuRaw, LevState, LsmCmd, LsmMode,
    LsmRaw, MAX_MAGNETS, PowerRaw, PresenceSensing, RailAccelRaw, SafetyInRaw, Tick,
};
use proptest::prelude::*;

fn fast_raw() -> impl Strategy<Value = FastRaw> {
    (
        any::<u64>(),
        (
            any::<[u16; MAX_MAGNETS]>(),
            any::<[u16; MAX_MAGNETS]>(),
            any::<u8>(),
        ),
        any::<[i16; MAX_MAGNETS]>(),
        (any::<i64>(), any::<bool>(), any::<bool>()),
        (any::<[i16; 3]>(), any::<bool>()),
        (any::<[i32; 3]>(), any::<bool>()),
    )
        .prop_map(
            |(tick, (code, code_b, valid), cur, enc, lsm, imu)| FastRaw {
                tick: Tick(tick),
                gap: GapRaw {
                    code,
                    code_b,
                    valid,
                },
                cur: CurrentRaw { code: cur },
                enc: EncoderRaw {
                    counts: enc.0,
                    referenced: enc.1,
                    err: enc.2,
                },
                lsm: LsmRaw {
                    i_abc_code: lsm.0,
                    fault: lsm.1,
                },
                imu: ImuRaw {
                    xyz: imu.0,
                    present: imu.1,
                },
                rail: RailAccelRaw {
                    xyz: imu.0,
                    present: imu.1,
                },
            },
        )
}

fn any_f32() -> impl Strategy<Value = f32> {
    prop_oneof![
        any::<u32>().prop_map(f32::from_bits),
        Just(f32::NAN),
        Just(f32::INFINITY),
        Just(f32::NEG_INFINITY),
        -200.0f32..200.0,
    ]
}

fn commands() -> impl Strategy<Value = (CoilCmd, LsmCmd)> {
    (
        0u8..3,
        proptest::array::uniform8(any_f32()),
        any_f32(),
        any_f32(),
        0u8..3,
    )
        .prop_map(|(kind, v, iq, id, mode)| {
            let coil = match kind {
                0 => CoilCmd::Off,
                1 => CoilCmd::CurrentRef(v),
                _ => CoilCmd::Voltage(v),
            };
            let mode = LsmMode::from_u8(mode).unwrap_or(LsmMode::Sto);
            (
                coil,
                LsmCmd {
                    iq_ref_a: iq,
                    id_ref_a: id,
                    mode,
                },
            )
        })
}

fn any_f64() -> impl Strategy<Value = f64> {
    prop_oneof![
        any::<u64>().prop_map(f64::from_bits),
        Just(f64::NAN),
        Just(f64::INFINITY),
        -1.0e3f64..1.0e3,
    ]
}

/// Limits: one of the four presets, or arbitrary bit patterns in every field.
fn limits() -> impl Strategy<Value = (SafetyLimits, SensorCal)> {
    let presets = (0usize..4).prop_map(|i| {
        let p = [
            presets::drawer,
            presets::door,
            presets::wall,
            presets::heavy,
        ][i]();
        (p.safety, p.design.sensors)
    });
    let wild = (
        proptest::collection::vec(any_f64(), 34),
        any::<i32>(),
        any::<u32>(),
        any::<bool>(),
        proptest::collection::vec(any_f64(), 4),
    )
        .prop_map(|(f, nm, hb, pld, c)| {
            let g = |i: usize| f.get(i).copied().unwrap_or(0.0);
            let l = SafetyLimits {
                mass_rated_kg: g(0),
                e_max_j: g(1),
                p_max_kgmps: g(2),
                v_open_max_mps: g(3),
                v_close_max_mps: g(4),
                v_open_sensed_mps: g(5),
                v_close_sensed_mps: g(6),
                f_drive_max_n: g(7),
                t_drive_max_s: g(8),
                f_static_n: g(9),
                f_static_sensed_n: g(10),
                f_static_final_n: g(11),
                t_static_final_s: g(12),
                f_close_last50_n: g(13),
                last_zone_m: g(14),
                i_coil_cont_a: g(15),
                i_coil_pulse_a: g(16),
                t_coil_pulse_s: g(17),
                i_lsm_max_a: g(18),
                gap_min_m: g(19),
                gap_max_m: g(20),
                x_min_m: g(21),
                x_max_m: g(22),
                overspeed_ratio: g(23),
                overspeed_t_s: g(24),
                vel_window_s: g(25),
                v_runaway_mps: g(26),
                v_hold_min_v: g(27),
                k_f_copy_n_per_a: g(28),
                adc_a_per_code_copy: g(29),
                lsm_a_per_code_copy: g(30),
                nm_per_count_copy: nm,
                presence_copy: if pld {
                    PresenceSensing::PlD
                } else {
                    PresenceSensing::None
                },
                heartbeat_max_ticks: hb,
                isolate: IsolateLimits {
                    stroke_m: g(31),
                    bumper_ke_j: g(32),
                    f_cont_n: g(33),
                },
                ..SafetyLimits::default()
            };
            let cal = SensorCal {
                gap_poly_m: [
                    c.first().copied().unwrap_or(0.0),
                    c.get(1).copied().unwrap_or(0.0),
                    c.get(2).copied().unwrap_or(0.0),
                ],
                v_hold_v_per_code: c.get(3).copied().unwrap_or(0.0),
                ..SensorCal::default()
            };
            (l, cal)
        });
    prop_oneof![presets, wild]
}

/// One step of the fuzzed schedule.
#[derive(Clone, Debug)]
struct Step {
    dt: u64,
    raw: FastRaw,
    coil: CoilCmd,
    lsm: LsmCmd,
    hb: u32,
    slow: Option<(PowerRaw, SafetyInRaw, u8)>,
    isolate: bool,
}

fn step() -> impl Strategy<Value = Step> {
    (
        prop_oneof![Just(0u64), 1u64..5, any::<u64>()],
        fast_raw(),
        commands(),
        any::<u32>(),
        proptest::option::of((
            (
                any::<u16>(),
                any::<u16>(),
                any::<i16>(),
                any::<bool>(),
                any::<[u16; 2]>(),
            ),
            (any::<[bool; 8]>(), any::<i16>()),
            0u8..16,
        )),
        any::<bool>(),
    )
        .prop_map(|(dt, raw, (coil, lsm), hb, slow, isolate)| Step {
            dt,
            raw,
            coil,
            lsm,
            hb,
            slow: slow.map(|((vb, vh, ib, mains, temp), (b, hf), lev)| {
                (
                    PowerRaw {
                        v_bus_code: vb,
                        v_hold_code: vh,
                        i_bus_code: ib,
                        mains_ok: mains,
                        temp_code: temp,
                    },
                    SafetyInRaw {
                        presence_ok: b[0],
                        presence_active: b[1],
                        handle_force_code: hf,
                        has_handle_cell: b[2],
                        fire_alarm: b[3],
                        estop: b[4],
                        lock_engaged: b[5],
                        hold_to_run: b[6],
                    },
                    lev,
                )
            }),
            isolate,
        })
}

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 256,
        failure_persistence: None,
        ..ProptestConfig::default()
    })]

    // req: FAB-SAF-017
    #[test]
    fn fuzz_raw_inputs(
        (l, cal) in limits(),
        steps in proptest::collection::vec(step(), 1..64),
    ) {
        let mut m = Monitor::new(&l, &cal);
        let mut t = 0u64;
        for s in &steps {
            t = t.wrapping_add(s.dt);
            if let Some((p, si, lev)) = &s.slow {
                let lev = LevState::from_u8(*lev).unwrap_or(LevState::Fault);
                let v = m.step_slow(Tick(t), p, si, lev);
                prop_assert!(v.force_cap_n.is_finite() && v.force_cap_n >= 0.0);
            }
            m.set_isolate(s.isolate);
            let v = m.step_fast(Tick(t), &s.raw, &s.coil, &s.lsm, s.hb);
            prop_assert!(
                v.force_cap_n.is_finite() && v.force_cap_n >= 0.0,
                "(T) finite, non-negative cap: {}", v.force_cap_n
            );
            let finite = s.coil.values().iter().all(|x| x.is_finite())
                && s.lsm.iq_ref_a.is_finite()
                && s.lsm.id_ref_a.is_finite();
            if !finite {
                prop_assert!(v.faults.contains(FaultCode::NonFinite), "(T) NaN/inf command → NonFinite");
            }
            // The demand is always at least what the faults imply.
            prop_assert!(v.demand >= v.faults.demand());
        }
    }
}
