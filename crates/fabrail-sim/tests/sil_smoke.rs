//! Closed-loop SIL smoke tests (core design §E.2 sim row; Wave 1.5 done-criterion): the
//! real executive against the real plant through `SimBoard`, in voltage coil mode (the
//! SIL ≡ firmware configuration, plan §2b item 16).
//!
//! W1.5 closed the loop end to end and removed the Wave-1 `#[ignore]`s (the rt fast
//! group now builds its frontend with `Frontend::from_params`, the supervisor
//! warm-starts a levitated initial state, and the motion loop controls the CoM with the
//! levitation's thrust-moment feed-forward). Tolerances marked (P) are provisional
//! (core design §E.1) and are tightened in Wave 2.

mod common;

use fabrail_params::presets;
use fabrail_sim::{RunData, run};

fn u8_channel<'a>(d: &'a RunData, name: &str) -> &'a [u8] {
    match &d.channel(name).expect("channel").data {
        fabrail_sim::ChannelData::U8(v) => v,
        fabrail_sim::ChannelData::F64(_) => panic!("{name} is not a u8 channel"),
    }
}

fn f64_channel<'a>(d: &'a RunData, name: &str) -> &'a [f64] {
    match &d.channel(name).expect("channel").data {
        fabrail_sim::ChannelData::F64(v) => v,
        fabrail_sim::ChannelData::U8(_) => panic!("{name} is not an f64 channel"),
    }
}

fn kpi(d: &RunData, k: &str) -> f64 {
    *d.kpis
        .get(k)
        .unwrap_or_else(|| panic!("KPI {k} missing: {:?}", d.kpis.keys()))
}

const CHANNELS: &str = r#"["x_m", "z_m", "gap_0_m", "coil_0_a", "demand", "lev_state",
    "contact_state"]"#;

#[test]

fn door_hover_1s() {
    let p = presets::door();
    let s = common::scenario(
        &format!(
            r#"{{"schema": "fabrail.scenario/1", "name": "door_hover_1s", "duration_s": 1,
                "noise_seed": 42, "fidelity": {{"model": "carriage6", "coil": "voltage"}},
                "initial": {{"lev": "levitated"}},
                "record": {{"rate_hz": 1000, "channels": {CHANNELS}}}}}"#
        ),
        &[],
    );
    let d = run(&p, &s).expect("run");
    let contact = u8_channel(&d, "contact_state");
    assert!(
        contact.iter().all(|c| c & 1 == 1),
        "levitated throughout (no roller or stop contact)"
    );
    assert!(
        u8_channel(&d, "demand").iter().all(|&x| x == 0),
        "no safety demand while hovering"
    );
    assert!(
        kpi(&d, "gap_dev_max_m") <= 0.2e-3,
        "(P) gap within 0.2 mm of the zero-power gap (PHYSICS §3, §15)"
    );
    assert!(
        kpi(&d, "coil_i_ss_a") <= 0.5,
        "(P) zero-power hover: mean |i| ≤ 0.5 A in the last 0.25 s (PHYSICS §15)"
    );
    let x = f64_channel(&d, "x_m");
    let drift = x.iter().fold(0.0f64, |m, v| m.max((v - x[0]).abs()));
    assert!(drift <= 1.0e-3, "(P) travel drift ≤ 1 mm at hold: {drift}");
}

#[test]

fn drawer_unseat() {
    let p = presets::drawer();
    let s = common::scenario(
        &format!(
            r#"{{"schema": "fabrail.scenario/1", "name": "drawer_unseat", "duration_s": 1.5,
                "noise_seed": 42, "fidelity": {{"model": "carriage6", "coil": "voltage"}},
                "initial": {{"lev": "seated"}},
                "record": {{"rate_hz": 1000, "channels": {CHANNELS}}},
                "events": [{{"kind": "command", "t_s": 0.1, "cmd": "lift"}}]}}"#
        ),
        &[],
    );
    let d = run(&p, &s).expect("run");
    let last = *u8_channel(&d, "contact_state").last().expect("samples");
    assert_eq!(
        last & 0b11,
        0b01,
        "levitated and out of the captive seat at the end"
    );
    let t = kpi(&d, "unseat_time_s");
    assert!(
        t <= 1.0,
        "(P) unseat within 1 s of the command (plan §2b item 4: pulse budget t_pulse = 0.2 s for the current peak): {t}"
    );
    assert!(
        u8_channel(&d, "demand").iter().all(|&x| x < 3),
        "no SafeLand/STO demand during the unseat"
    );
}

#[test]

fn wall_land() {
    let p = presets::wall();
    let s = common::scenario(
        &format!(
            r#"{{"schema": "fabrail.scenario/1", "name": "wall_land", "duration_s": 3,
                "noise_seed": 42, "fidelity": {{"model": "carriage6", "coil": "voltage"}},
                "initial": {{"lev": "levitated"}},
                "record": {{"rate_hz": 1000, "channels": {CHANNELS}}},
                "events": [{{"kind": "command", "t_s": 0.1, "cmd": "land"}}]}}"#
        ),
        &[],
    );
    let d = run(&p, &s).expect("run");
    let last = *u8_channel(&d, "contact_state").last().expect("samples");
    assert_eq!(
        last & 0b101,
        0b100,
        "on the rollers, not levitated, at the end"
    );
    let v = kpi(&d, "touchdown_v_mps");
    assert!(
        v <= 10.0e-3,
        "(P) controlled landing: touchdown ≤ 10 mm/s (class land rate 3 mm/s, core design §C.4): {v}"
    );
    assert!(kpi(&d, "land_time_s") <= 2.5, "(P) landed within 2.5 s");
}
