//! Physical end stops in the closed loop (W2 lead decision D3; PHYSICS §8, §18).
//!
//! Contract: every class at its rest state at the closed position, on Travel1d in
//! voltage coil mode with the full executive, under 12 s of 0.25 g, 1 Hz floor motion
//! along x from t = 0 (the villa smoke's floor level): the carriage's truth position never
//! passes the monitor's SLP limit `safety.x_min_m` (x_closed − 10 mm, presets), because
//! the closed-end jamb (DOOR, WALL, HEAVY) or the DRAWER carcass stops it; every state
//! stays finite; and a contact with a stop is reported as a `plant:end_stop` event (impact
//! speed, position), never as a fault of its own. The controller's reaction (freeze,
//! land, lock, faults) belongs to the core and safety units and is not asserted here.

mod common;

use fabrail_params::presets;
use fabrail_plant::contact::end_stops;
use fabrail_plant::params::{Environment, Mismatch, PlantParams};
use fabrail_sim::{ChannelData, run};

#[test]
fn floor_shaking_stays_inside_the_slp_band_at_the_closed_end() {
    let dur = 12.0;
    let fs = 200.0;
    let n = (dur * fs) as usize + 2;
    let ax: Vec<f64> = (0..n)
        .map(|k| 0.25 * 9.80665 * (std::f64::consts::TAU * k as f64 / fs).sin())
        .collect();
    for (p, lev) in [
        (presets::drawer(), "seated"),
        (presets::door(), "levitated"),
        (presets::wall(), "landed"),
        (presets::heavy(), "landed"),
    ] {
        let class = p.design.class;
        let x0 = p.design.rail.x_closed_m;
        let json = format!(
            r#"{{"schema": "fabrail.scenario/1", "name": "shake_closed", "duration_s": {dur},
                "noise_seed": 3, "fidelity": {{"model": "travel1d", "coil": "voltage"}},
                "initial": {{"lev": "{lev}", "x_m": {x0}}},
                "record": {{"rate_hz": 1000, "channels": ["x_m", "v_mps", "f_end_stop_n"]}},
                "series": {{"floor": {{"axis": "time", "fs_hz": 200, "channels": ["ax"]}}}}}}"#
        );
        let s = common::scenario(&json, &[("floor", vec![ax.clone()])]);
        let d = run(&p, &s).expect("run");
        let ch = |name: &str| -> Vec<f64> {
            match d.channels.iter().find(|c| c.name == name).map(|c| &c.data) {
                Some(ChannelData::F64(v)) => v.clone(),
                _ => panic!("{class:?}: channel {name} recorded"),
            }
        };
        let (x, v, f) = (ch("x_m"), ch("v_mps"), ch("f_end_stop_n"));
        assert!(
            x.iter().chain(&v).chain(&f).all(|q| q.is_finite()),
            "{class:?}: finite truth"
        );
        let lo = x.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(
            lo >= p.safety.x_min_m,
            "{class:?}: min x {:.2} mm inside the SLP band x_min {:.2} mm (presets; D3)",
            1e3 * lo,
            1e3 * p.safety.x_min_m
        );
        let pp = PlantParams::from_design(&p.design, &Mismatch::default(), &Environment::default());
        let st = end_stops(&pp.design);
        let stop_events = d
            .events
            .iter()
            .filter(|e| e.kind == "plant:end_stop")
            .count();
        let touched = f.iter().any(|&q| q > 0.0);
        assert!(
            !touched || stop_events > 0,
            "{class:?}: end-stop contact (min x {:.2} mm, face {:.2} mm) is reported as \
             plant:end_stop ({stop_events} events)",
            1e3 * lo,
            1e3 * st.closed_m
        );
    }
}
