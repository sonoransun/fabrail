//! Shared helpers for the fabrail-sim integration tests (no tests here).

#![allow(dead_code)]

use std::collections::BTreeMap;

use fabrail_params::{SystemParams, presets};
use fabrail_sim::{Scenario, encode_f64_le};

/// A scenario from JSON with optional series (name → channels).
pub fn scenario(json: &str, series: &[(&str, Vec<Vec<f64>>)]) -> Scenario {
    let bytes: BTreeMap<String, Vec<u8>> = series
        .iter()
        .map(|(k, ch)| (k.to_string(), encode_f64_le(ch)))
        .collect();
    Scenario::from_json(json, &bytes).expect("valid scenario")
}

/// A 1 Hz / 0.3 m/s² floor sine on x and a small 7 Hz vertical at 200 Hz for `dur_s`.
pub fn floor_series(dur_s: f64) -> Vec<Vec<f64>> {
    let fs = 200.0;
    let n = (dur_s * fs) as usize + 2;
    let tau = std::f64::consts::TAU;
    let ax = (0..n).map(|k| 0.3 * (tau * k as f64 / fs).sin()).collect();
    let ay = vec![0.0; n];
    let az = (0..n)
        .map(|k| 0.05 * (tau * 7.0 * k as f64 / fs).sin())
        .collect();
    vec![ax, ay, az]
}

/// DOOR on Carriage6 in voltage mode with a command, a hand, an alert and a floor
/// series (the SIL ≡ firmware configuration, plan §2b item 16).
pub fn door_case(seed: u64, dur_s: f64) -> (SystemParams, Scenario) {
    let json = format!(
        r#"{{"schema": "fabrail.scenario/1", "name": "door_det", "duration_s": {dur_s},
            "noise_seed": {seed},
            "fidelity": {{"model": "carriage6", "coil": "voltage"}},
            "record": {{"rate_hz": 2000}},
            "series": {{"floor": {{"axis": "time", "fs_hz": 200, "channels": ["ax", "ay", "az"]}}}},
            "events": [
              {{"kind": "command", "t_s": 0.01, "cmd": "open"}},
              {{"kind": "hand", "t_s": 0.02, "t_pts_s": [0, 0.02, 0.05], "f_pts_n": [0, 8, 0]}},
              {{"kind": "network_alert", "t_s": 0.03, "magnitude": 6.5, "r_epi_km": 40,
                "depth_km": 10, "origin_t_s": -4}},
              {{"kind": "fault", "t_s": 0.04, "what": "gap_stuck", "ch": 1}}
            ]}}"#
    );
    (
        presets::door(),
        scenario(&json, &[("floor", floor_series(dur_s))]),
    )
}

/// WALL on Travel1d with a network alert and a floor series.
pub fn wall_case(seed: u64, dur_s: f64) -> (SystemParams, Scenario) {
    let json = format!(
        r#"{{"schema": "fabrail.scenario/1", "name": "wall_det", "duration_s": {dur_s},
            "noise_seed": {seed},
            "fidelity": {{"model": "travel1d", "coil": "voltage"}},
            "record": {{"rate_hz": 1000}},
            "series": {{"floor": {{"axis": "time", "fs_hz": 200, "channels": ["ax", "ay", "az"]}},
                        "site": {{"axis": "time", "fs_hz": 200, "channels": ["az"]}}}},
            "events": [
              {{"kind": "network_alert", "t_s": 0.05, "magnitude": 7, "r_epi_km": 20,
                "depth_km": 10, "origin_t_s": -2}},
              {{"kind": "command", "t_s": 0.1, "cmd": "close"}}
            ]}}"#
    );
    let n = (dur_s * 200.0) as usize + 2;
    let site = vec![(0..n).map(|k| 1.0e-3 * ((k % 7) as f64 - 3.0)).collect()];
    (
        presets::wall(),
        scenario(&json, &[("floor", floor_series(dur_s)), ("site", site)]),
    )
}
