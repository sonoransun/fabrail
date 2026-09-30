//! fabrail-web unit tests: the frozen ABI against `engine.js`, the engine against the
//! native `fabrail_sim::run` trace (bit-exact on the host; the wasm32 side is
//! `tests/test_site.py`), input semantics and the handle layer.

use std::collections::BTreeMap;

use fabrail_plant::Tier;
use fabrail_sim::{ChannelData, RunData, Scenario, ScenarioSpec, encode_f64_le, run};

use super::*;

/// Base ticks compared against the native run (FAB-SW-014: the first 2,000 ticks).
const TRACE_TICKS: usize = 2_000;

fn preset_id(class: fabrail_types::RailClass) -> u32 {
    PRESETS
        .iter()
        .position(|c| *c == class)
        .map(|i| i as u32)
        .expect("class has a preset id")
}

fn tier_id_of(tier: Tier) -> u32 {
    TIERS
        .iter()
        .position(|t| *t == tier)
        .map(|i| i as u32)
        .expect("tier has an id")
}

#[test]
fn state_layout_matches_engine_js() {
    let js = include_str!("../../../docs/assets/js/engine.js");
    let start = js
        .find("var STATE_FIELDS = [")
        .expect("engine.js declares STATE_FIELDS");
    let body = &js[start..];
    let end = body.find("];").expect("STATE_FIELDS closes");
    let names: Vec<&str> = body[..end].split('"').skip(1).step_by(2).collect();
    assert_eq!(
        names,
        STATE_FIELDS.to_vec(),
        "state buffer = engine.js STATE_FIELDS"
    );
    for name in [
        "fabrail_abi_version",
        "fabrail_create",
        "fabrail_set_inputs",
        "fabrail_step",
        "fabrail_state_ptr",
        "fabrail_state_len",
        "fabrail_selftest_hash",
        "fabrail_anchor",
    ] {
        assert!(js.contains(name), "engine.js expects {name}");
    }
    assert!(js.contains(r#"var PRESETS = ["drawer", "door", "wall", "heavy"];"#));
    // engine.js TIERS are the tier ids of `TIERS` (ABI v2).
    let tiers: Vec<String> = TIERS.iter().map(|t| format!("\"{}\"", t.name())).collect();
    assert!(
        js.contains(&format!("var TIERS = [{}];", tiers.join(", "))),
        "engine.js TIERS = {tiers:?}"
    );
    // engine.js COMMANDS are the codes of `cmd` (9 = manual is a v2 addition).
    for (name, code) in [
        ("manual", cmd::MANUAL),
        ("none", cmd::NONE),
        ("open", cmd::OPEN),
        ("close", cmd::CLOSE),
        ("stop", cmd::STOP),
        ("lift", cmd::LIFT),
        ("land", cmd::LAND),
        ("reset", cmd::RESET),
        ("hold_to_run_on", cmd::HOLD_TO_RUN_ON),
        ("hold_to_run_off", cmd::HOLD_TO_RUN_OFF),
    ] {
        assert!(
            js.contains(&format!("{name}: {code}")),
            "engine.js COMMANDS.{name} = {code}"
        );
    }
}

#[test]
fn engines_and_anchors() {
    assert_eq!(fabrail_abi_version(), ABI_VERSION);
    assert_eq!(fabrail_create(1), 0);
    assert_eq!(fabrail_tier(), DEFAULT_TIER);
    assert_eq!(fabrail_create(9), 1);
    assert_eq!(fabrail_create_tier(1, 7), 3);
    assert_eq!(
        fabrail_tier(),
        DEFAULT_TIER,
        "a failed create keeps the engine"
    );
    assert_eq!(fabrail_status(), 0);
    fabrail_set_inputs(5.0, 0.1, 0.0, 0.0, cmd::OPEN);
    assert_eq!(fabrail_step(200), 200);
    let s = with_default(|e| *e.state()).unwrap_or([f64::NAN; STATE_LEN]);
    assert_eq!(s[idx::TICK], 200.0);
    assert_eq!(s[idx::T_S], 0.01, "t = tick / 20 kHz");
    assert_eq!(s[idx::F_HAND_N], 5.0, "pure force source: applied = input");
    assert_eq!(s[idx::FLOOR_AX_MPS2], 0.1, "floor held over the step");
    assert!(s.iter().all(|v| v.is_finite()), "{s:?}");
    let h = fr_create(0);
    assert!(h >= 1);
    assert!(!fr_state_ptr(h).is_null());
    assert_eq!(fr_status(h), 0);
    assert_eq!(fr_step(h, 20), 20);
    assert_eq!(fr_go_to(h, 0.1), 1);
    assert_eq!(fr_go_to(h, f64::NAN), 0);
    assert_eq!(fr_destroy(h), 1);
    assert_eq!(fr_destroy(h), 0);
    assert!(fr_state_ptr(h).is_null());
    assert_eq!(fr_status(h), STATUS_NO_ENGINE);
    assert_eq!(fr_tier(h), TIER_NONE);
    assert_eq!(fr_set_inputs(h, 1.0, 0.0, 0.0, 0.0, 0), 0);
    assert_eq!(fr_step(0, 20), 0, "handle 0 is never valid");
    assert_eq!(fr_create_tier(4, 0), 0, "unknown preset");
    assert_eq!(fr_create_tier(0, 3), 0, "Structure is not offered");
    // DOOR open-loop pole anchor through the derived table (47.57 rad/s).
    let k = (0..ANCHOR_STRIDE)
        .find(|k| anchor_name(ANCHOR_STRIDE + k) == Some("pole_rad_s"))
        .expect("derived table has pole_rad_s");
    assert!((fabrail_anchor(ANCHOR_STRIDE + k) - 47.57).abs() < 0.05);
    assert!(fabrail_anchor(4 * ANCHOR_STRIDE).is_nan());
    assert_eq!(fr_anchor_name(4 * ANCHOR_STRIDE), 0);
    assert_eq!(fr_anchor_name(ANCHOR_STRIDE + k), "pole_rad_s".len() as u32);
    assert!((fr_den_hartog_f(0.02) - 0.98039).abs() < 5e-6);
    assert_eq!(
        fr_terminus_tb(0.184, 2.0, 0.25),
        derive::terminus_blend(0.184, 2.0, 0.25).t_b_s
    );
}

#[test]
fn selftest_hash_is_the_rt_golden_run() {
    let h = fabrail_rt::selftest::selftest_hash(fabrail_rt::selftest::SELFTEST_TICKS);
    assert_ne!(h, fabrail_rt::selftest::SELFTEST_ERROR_HASH, "selftest ran");
    assert_eq!(fabrail_selftest_hash(), h);
    assert_eq!(
        (u64::from(fr_selftest_hash_hi()) << 32) | u64::from(fr_selftest_hash_lo()),
        h
    );
    if fabrail_rt::GOLDEN_SELFTEST_HASH != 0 {
        assert_eq!(
            h,
            fabrail_rt::GOLDEN_SELFTEST_HASH,
            "frozen golden hash (C1)"
        );
    }
}

#[test]
fn commands_map_to_user_commands() {
    use fabrail_types::UserCommand as U;
    assert_eq!(user_command(cmd::NONE), None);
    assert_eq!(user_command(cmd::OPEN), Some(U::Open));
    assert_eq!(user_command(cmd::CLOSE), Some(U::Close));
    assert_eq!(user_command(cmd::STOP), Some(U::Stop));
    assert_eq!(user_command(cmd::LIFT), Some(U::Lift));
    assert_eq!(user_command(cmd::LAND), Some(U::Land));
    assert_eq!(user_command(cmd::RESET), Some(U::Reset));
    assert_eq!(user_command(cmd::HOLD_TO_RUN_ON), Some(U::HoldToRun(true)));
    assert_eq!(
        user_command(cmd::HOLD_TO_RUN_OFF),
        Some(U::HoldToRun(false))
    );
    assert_eq!(user_command(cmd::MANUAL), Some(U::Manual));
    assert_eq!(user_command(cmd::MAX + 1), None);
}

#[test]
fn scenario_is_generated_per_preset_and_tier() {
    for (p, class) in PRESETS.iter().enumerate() {
        let params = fabrail_params::presets::by_class(*class);
        for tier in TIERS {
            let json = scenario_json(&params, tier).expect("scenario builds");
            let spec = ScenarioSpec::from_json(&json).expect("round trip");
            assert_eq!(spec.to_json(), json, "canonical form");
            assert_eq!(spec.tier(), tier);
            assert_eq!(spec.noise_seed, NOISE_SEED);
            assert!(spec.series.is_empty() && spec.events.is_empty());
            assert_eq!(
                spec.fidelity.coil.is_voltage(),
                params.control.rates.coil_mode_voltage,
                "coil mode single source (plan §2b item 16)"
            );
            let n = fr_scenario_json(p as u32, tier_id_of(tier));
            assert_eq!(n as usize, json.len());
        }
    }
    assert_eq!(fr_scenario_json(4, 0), 0);
    assert_eq!(fr_scenario_json(0, 3), 0);
}

#[test]
fn every_preset_and_tier_steps_finitely() {
    for p in 0..PRESETS.len() as u32 {
        for t in 0..TIERS.len() as u32 {
            let mut e = Engine::with_tier(p, t).unwrap_or_else(|err| panic!("({p}, {t}): {err}"));
            let s0 = *e.state();
            assert_eq!(s0[idx::TICK], 0.0);
            assert!(s0.iter().all(|v| v.is_finite()), "({p}, {t}) rest {s0:?}");
            assert_eq!(e.step(400), 400);
            assert_eq!(
                e.status(),
                EngineStatus::Running,
                "({p}, {t}) {:?}",
                e.error()
            );
            let s = e.state();
            assert_eq!(s[idx::TICK], 400.0);
            assert!(s.iter().all(|v| v.is_finite()), "({p}, {t}) {s:?}");
            assert!(s[idx::GAP_M] > 0.0, "({p}, {t}) gap {}", s[idx::GAP_M]);
        }
    }
}

/// Engine states after 0, 1, …, `n` single-tick steps with the inputs set once.
fn engine_trace(
    preset: u32,
    tier: u32,
    floor: [f64; 3],
    command: u32,
    n: usize,
) -> (Engine, Vec<[f64; STATE_LEN]>) {
    let mut e = Engine::with_tier(preset, tier).expect("engine");
    e.set_inputs(0.0, floor[0], floor[1], floor[2], command);
    let mut v = vec![*e.state()];
    for _ in 0..n {
        assert_eq!(e.step(1), 1, "{:?}", e.error());
        v.push(*e.state());
    }
    (e, v)
}

/// The native run of the engine's own scenario plus the same inputs as scenario data: a
/// constant floor series and a command event at t = 0.
fn native_trace(e: &Engine, floor: [f64; 3], command: &str, ticks: usize, rate_hz: f64) -> RunData {
    native_trace_event(
        e,
        floor,
        &format!(r#"{{"kind": "command", "t_s": 0, "cmd": "{command}"}}"#),
        ticks,
        rate_hz,
    )
}

/// [`native_trace`] with an arbitrary event (JSON) at t = 0.
fn native_trace_event(
    e: &Engine,
    floor: [f64; 3],
    event: &str,
    ticks: usize,
    rate_hz: f64,
) -> RunData {
    let mut spec = ScenarioSpec::from_json(&e.scenario_json()).expect("engine scenario");
    let coil = if spec.fidelity.coil.is_voltage() {
        "voltage"
    } else {
        "lag"
    };
    let extra = ScenarioSpec::from_json(&format!(
        r#"{{"schema": "fabrail.scenario/1", "name": "x", "duration_s": 1, "noise_seed": 0,
            "fidelity": {{"model": "{}", "coil": "{coil}"}},
            "series": {{"floor": {{"axis": "time", "fs_hz": 5, "t0_s": 0, "channels": ["ax", "ay", "az"]}}}},
            "events": [{event}]}}"#,
        e.tier().name()
    ))
    .expect("helper spec");
    spec.duration_s = ticks as f64 / f64::from(fabrail_types::BASE_TICK_HZ);
    spec.record.rate_hz = rate_hz;
    spec.series = extra.series;
    spec.events = extra.events;
    // A constant floor over [0, 0.4] s: linear interpolation of equal samples is exact.
    let bytes = encode_f64_le(&floor.map(|a| vec![a; 3]));
    let series = BTreeMap::from([("floor".to_string(), bytes)]);
    let s = Scenario::from_spec(spec, &series).expect("native scenario");
    run(e.params(), &s).expect("native run")
}

fn f64_channel<'a>(d: &'a RunData, name: &str) -> &'a [f64] {
    match &d.channel(name).expect(name).data {
        ChannelData::F64(v) => v,
        ChannelData::U8(_) => panic!("{name} is u8"),
    }
}

fn u8_channel<'a>(d: &'a RunData, name: &str) -> &'a [u8] {
    match &d.channel(name).expect(name).data {
        ChannelData::U8(v) => v,
        ChannelData::F64(_) => panic!("{name} is f64"),
    }
}

/// Bit-exact comparison of an engine trace with a native run recorded every `decim`
/// ticks. Recorded sample j carries the truth at tick j·decim (= the engine after
/// j·decim steps) and the executive outputs of tick j·decim (= the engine after
/// j·decim + 1 steps).
fn assert_trace_matches(states: &[[f64; STATE_LEN]], d: &RunData, decim: usize, n_mag: usize) {
    let truth = [
        ("x_m", idx::X_M),
        ("v_mps", idx::V_MPS),
        ("a_mps2", idx::A_MPS2),
        ("iq_a", idx::IQ_A),
        ("f_lsm_n", idx::F_LSM_N),
        ("f_hand_n", idx::F_HAND_N),
        ("floor_ax_mps2", idx::FLOOR_AX_MPS2),
    ];
    let ctrl_f = [
        ("f_hand_hat_n", idx::F_HAND_HAT_N),
        ("m_hat_kg", idx::M_HAT_KG),
        ("e_kin_gov_j", idx::E_KIN_GOV_J),
    ];
    let ctrl_u = [
        ("lev_state", idx::LEV_STATE),
        ("motion_mode", idx::MOTION_MODE),
        ("seismic_state", idx::SEISMIC_STATE),
        ("level", idx::SEISMIC_LEVEL),
        ("demand", idx::SAFETY_DEMAND),
    ];
    let gaps: Vec<&[f64]> = (0..n_mag)
        .map(|i| f64_channel(d, &format!("gap_{i}_m")))
        .collect();
    let coils: Vec<&[f64]> = (0..n_mag)
        .map(|i| f64_channel(d, &format!("coil_{i}_a")))
        .collect();
    let mean = |ch: &[&[f64]], j: usize| ch.iter().fold(0.0, |s, c| s + c[j]) / n_mag as f64;
    let n_samples = d.n_samples();
    assert_eq!((n_samples - 1) * decim, states.len() - 1, "sample grid");
    let mut compared = 0usize;
    for j in 0..n_samples {
        let k = j * decim;
        let st = &states[k];
        for (name, i) in truth {
            let native = f64_channel(d, name)[j];
            assert_eq!(
                st[i].to_bits(),
                native.to_bits(),
                "{name} at tick {k}: engine {} vs native {native} (bit-exact, FAB-SW-014)",
                st[i]
            );
        }
        assert_eq!(
            st[idx::GAP_M].to_bits(),
            mean(&gaps, j).to_bits(),
            "mean gap at tick {k}"
        );
        assert_eq!(
            st[idx::COIL_I_A].to_bits(),
            mean(&coils, j).to_bits(),
            "mean coil current at tick {k}"
        );
        if let Some(sc) = states.get(k + 1) {
            for (name, i) in ctrl_f {
                let native = f64_channel(d, name)[j];
                assert_eq!(
                    sc[i].to_bits(),
                    native.to_bits(),
                    "{name} of tick {k}: engine {} vs native {native} (bit-exact)",
                    sc[i]
                );
            }
            for (name, i) in ctrl_u {
                let native = f64::from(u8_channel(d, name)[j]);
                assert_eq!(sc[i], native, "{name} of tick {k}");
            }
        }
        compared += 1;
    }
    assert_eq!(compared, n_samples);
}

// FAB-SW-014 on the host: the browser engine is the native closed loop (the wasm32
// build is compared with the native module by tests/test_site.py).
#[test]
fn engine_matches_native_run_carriage6_door() {
    let floor = [0.3, -0.05, 0.02];
    let door = preset_id(fabrail_types::RailClass::Door);
    let (e, states) = engine_trace(
        door,
        tier_id_of(Tier::Carriage6),
        floor,
        cmd::OPEN,
        TRACE_TICKS,
    );
    let d = native_trace(&e, floor, "open", TRACE_TICKS, 20_000.0);
    let n_mag = e.params().design.suspension.n_magnets as usize;
    assert_trace_matches(&states, &d, 1, n_mag);
}

#[test]
fn engine_matches_native_run_heave1_drawer_and_travel1d_wall() {
    let floor = [0.0, 0.0, 0.5];
    let drawer = preset_id(fabrail_types::RailClass::Drawer);
    let (e, states) = engine_trace(
        drawer,
        tier_id_of(Tier::Heave1),
        floor,
        cmd::LIFT,
        TRACE_TICKS,
    );
    let d = native_trace(&e, floor, "lift", TRACE_TICKS, 20_000.0);
    assert_trace_matches(
        &states,
        &d,
        1,
        e.params().design.suspension.n_magnets as usize,
    );
    // Travel1d steps the plant every 20 ticks and records at most at 1 kHz.
    let floor = [0.4, 0.0, 0.0];
    let wall = preset_id(fabrail_types::RailClass::Wall);
    let (e, states) = engine_trace(
        wall,
        tier_id_of(Tier::Travel1d),
        floor,
        cmd::LIFT,
        TRACE_TICKS,
    );
    let d = native_trace(&e, floor, "lift", TRACE_TICKS, 1_000.0);
    assert_trace_matches(
        &states,
        &d,
        20,
        e.params().design.suspension.n_magnets as usize,
    );
}

#[test]
fn chunked_steps_equal_single_ticks() {
    let door = preset_id(fabrail_types::RailClass::Door);
    let (_, one) = engine_trace(door, DEFAULT_TIER, [0.2, 0.0, 0.0], cmd::OPEN, 1_400);
    let mut e = Engine::with_tier(door, DEFAULT_TIER).expect("engine");
    e.set_inputs(0.0, 0.2, 0.0, 0.0, cmd::OPEN);
    // 7-tick steps, then one step longer than the internal chunk.
    for k in 1..=100usize {
        assert_eq!(e.step(7), 7);
        assert_eq!(e.state(), &one[7 * k], "after {} ticks", 7 * k);
    }
    assert_eq!(e.step(700), 700);
    assert_eq!(e.state(), &one[1_400]);
}

#[test]
fn inputs_are_held_and_commands_consumed_once() {
    let door = preset_id(fabrail_types::RailClass::Door);
    let mut e = Engine::with_tier(door, tier_id_of(Tier::Travel1d)).expect("engine");
    e.set_inputs(f64::NAN, f64::INFINITY, 0.0, 0.0, cmd::MAX + 1);
    assert_eq!(
        e.inputs(),
        Inputs::default(),
        "non-finite and unknown inputs read as 0/none"
    );
    // A later `none` leaves the pending open in place; the next step consumes it.
    e.set_inputs(0.0, 0.0, 0.0, 0.0, cmd::OPEN);
    e.set_inputs(20.0, 0.0, 0.0, 0.0, cmd::NONE);
    assert_eq!(e.inputs().command, cmd::NONE);
    assert_eq!(e.pending_command(), Some(fabrail_types::UserCommand::Open));
    assert_eq!(e.step(40), 40);
    assert_eq!(e.pending_command(), None, "consumed by one step");
    let s = *e.state();
    assert_eq!(
        s[idx::F_HAND_N],
        20.0,
        "Travel1d: the hand acts from the first plant step"
    );
    assert!(
        s[idx::V_MPS] > 0.0,
        "a 20 N push moves the levitated door toward +x"
    );
    // Releasing the hand removes the force at the next plant step.
    e.set_inputs(0.0, 0.0, 0.0, 0.0, cmd::NONE);
    assert_eq!(e.step(20), 20);
    assert_eq!(e.state()[idx::F_HAND_N], 0.0);
    // A newer command replaces a pending one; a zero-tick step still consumes it.
    e.set_inputs(0.0, 0.0, 0.0, 0.0, cmd::OPEN);
    e.set_inputs(0.0, 0.0, 0.0, 0.0, cmd::STOP);
    assert_eq!(e.pending_command(), Some(fabrail_types::UserCommand::Stop));
    assert_eq!(e.step(0), 0);
    assert_eq!(e.pending_command(), None);
}

#[test]
fn go_to_is_the_native_go_to_command() {
    let door = preset_id(fabrail_types::RailClass::Door);
    let mut e = Engine::with_tier(door, tier_id_of(Tier::Travel1d)).expect("engine");
    assert!(!e.go_to(f64::NAN), "non-finite target refused");
    let x = e.params().design.rail.x_closed_m + 0.25 * e.params().design.rail.travel_m;
    assert!(e.go_to(x));
    let mut states = vec![*e.state()];
    for _ in 0..TRACE_TICKS {
        assert_eq!(e.step(1), 1);
        states.push(*e.state());
    }
    let d = native_trace_event(
        &e,
        [0.0; 3],
        &format!(r#"{{"kind": "command", "t_s": 0, "cmd": "go_to", "x_m": {x:?}}}"#),
        TRACE_TICKS,
        1_000.0,
    );
    assert_trace_matches(
        &states,
        &d,
        20,
        e.params().design.suspension.n_magnets as usize,
    );
}

#[test]
fn step_is_capped_per_call() {
    let wall = preset_id(fabrail_types::RailClass::Wall);
    let mut e = Engine::with_tier(wall, tier_id_of(Tier::Travel1d)).expect("engine");
    assert_eq!(e.step(u32::MAX), MAX_STEP_TICKS);
    assert_eq!(e.tick().0, u64::from(MAX_STEP_TICKS));
    assert_eq!(e.status(), EngineStatus::Running);
}

#[test]
fn engine_ends_at_the_scenario_end() {
    let wall = preset_id(fabrail_types::RailClass::Wall);
    let mut e = Engine::with_tier(wall, tier_id_of(Tier::Travel1d))
        .expect("engine")
        .with_end_tick(1_000);
    assert_eq!(e.step(600), 600);
    assert_eq!(e.status(), EngineStatus::Running);
    assert_eq!(e.step(600), 400, "never past the scenario end");
    assert_eq!(e.status(), EngineStatus::Ended);
    assert_eq!(e.step(10), 0);
    assert_eq!(e.state()[idx::TICK], 1_000.0);
}

/// Native throughput of each (preset, tier), to choose the default tier. Run with
/// `cargo test --release -p fabrail-web -- --ignored --nocapture throughput`.
#[test]
#[ignore = "timing probe, not a check"]
fn throughput() {
    for p in 0..PRESETS.len() as u32 {
        for t in 0..TIERS.len() as u32 {
            let mut e = Engine::with_tier(p, t).expect("engine");
            e.set_inputs(0.0, 0.0, 0.0, 0.0, cmd::OPEN);
            let t0 = std::time::Instant::now();
            e.step(20_000);
            let dt = t0.elapsed().as_secs_f64();
            println!(
                "{:>6} {:>9}: {:6.2}x real time ({:.2} us/tick)",
                fabrail_params::presets::class_name(PRESETS[p as usize]),
                TIERS[t as usize].name(),
                1.0 / dt,
                dt / 20_000.0 * 1e6
            );
        }
    }
}
