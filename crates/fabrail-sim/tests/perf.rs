//! Real-time factors of the SIL harness (core design §E.6; reported by
//! `sim/tests/perf`, asserted by the slow Python `performance_budget` test).
//!
//! `#[ignore]`: timing depends on the host and on the build profile; run with
//! `cargo test -p fabrail-sim --release --test perf -- --ignored --nocapture`.
//!
//! W2 lead decision D8: the full-rate executive runs at the 20 kHz base tick in every
//! tier (SIL fidelity: the same `fabrail-rt` groups at the same rates as the firmware),
//! so the Travel1d factor is bounded by the controller's cost rather than the 1 ms plant
//! step. Budgets: Carriage6 DOOR lag ≥ 10×, voltage ≥ 3×, HEAVY ≥ 5×, Travel1d ≥ 100×
//! (was 1000×, unreachable with a full-rate executive: ≈ 0.4 µs per base tick), and the
//! isolation Monte Carlo (N = 200 WALL runs × 60 s on Travel1d) ≤ 60 s wall on 8 workers.

mod common;

use std::time::Instant;

use fabrail_params::presets;
use fabrail_sim::run;

fn factor(
    name: &str,
    p: &fabrail_params::SystemParams,
    model: &str,
    coil: &str,
    dur_s: f64,
) -> f64 {
    let s = common::scenario(
        &format!(
            r#"{{"schema": "fabrail.scenario/1", "name": "perf", "duration_s": {dur_s},
                "noise_seed": 1, "fidelity": {{"model": "{model}", "coil": "{coil}"}},
                "record": {{"rate_hz": 1000}}}}"#
        ),
        &[],
    );
    let t0 = Instant::now();
    let d = run(p, &s).expect("run");
    let wall = t0.elapsed().as_secs_f64();
    let rtf = dur_s / wall;
    println!(
        "{name:<28} {dur_s:>6.2} s simulated in {wall:>8.4} s: {rtf:>9.1}x real time ({} samples)",
        d.n_samples()
    );
    rtf
}

#[test]
#[ignore = "host- and profile-dependent timing; run on demand with --ignored --release"]
fn realtime_factors() {
    let mut door_lag = presets::door();
    door_lag.control.rates.coil_mode_voltage = false;
    let cases = [
        (
            "carriage6 DOOR lag",
            door_lag,
            "carriage6",
            "lag",
            1.0,
            10.0,
        ),
        (
            "carriage6 DOOR voltage",
            presets::door(),
            "carriage6",
            "voltage",
            1.0,
            3.0,
        ),
        (
            "carriage6 HEAVY voltage",
            presets::heavy(),
            "carriage6",
            "voltage",
            1.0,
            5.0,
        ),
        (
            "travel1d WALL voltage",
            presets::wall(),
            "travel1d",
            "voltage",
            10.0,
            100.0,
        ),
    ];
    let mut misses = Vec::new();
    for (name, p, model, coil, dur, budget) in cases {
        let rtf = factor(name, &p, model, coil, dur);
        if rtf < budget {
            misses.push(format!("{name}: {rtf:.1}x < {budget}x"));
        }
    }
    assert!(
        misses.is_empty(),
        "(S) real-time budgets of core design §E.6 missed: {misses:?}"
    );
}

/// One isolation Monte Carlo run: WALL on Travel1d (voltage coil mode), isolation
/// enabled (research tier), levitated at mid-travel, 60 s of band-limited random floor
/// motion (a sum of 40 random-phase sinusoids over 0.3–8 Hz scaled to ≈ 0.5 g peak; the
/// timing does not depend on the spectrum's details), as the Python `isolation_kt`
/// scenario and SEI-015's N = 200 study use (core design §E.6).
fn isolation_run(seed: u64, dur_s: f64) -> (fabrail_params::SystemParams, fabrail_sim::Scenario) {
    let mut p = presets::wall();
    p.control.seismic.isolation.enable = true;
    let x_mid = p.design.rail.x_closed_m + 0.5 * p.design.rail.travel_m;
    let fs = 200.0;
    let n = (dur_s * fs) as usize + 2;
    let mut rng = fabrail_sim::rng::Xoshiro256pp::from_seed(seed);
    let comps: Vec<(f64, f64)> = (0..40)
        .map(|_| {
            let f = 0.3 + 7.7 * rng.next_f64();
            (f, std::f64::consts::TAU * rng.next_f64())
        })
        .collect();
    let mut ax: Vec<f64> = (0..n)
        .map(|k| {
            let t = k as f64 / fs;
            comps
                .iter()
                .map(|&(f, ph)| (std::f64::consts::TAU * f * t + ph).sin())
                .sum::<f64>()
        })
        .collect();
    let peak = ax.iter().fold(0.0f64, |m, v| m.max(v.abs())).max(1e-9);
    for v in &mut ax {
        *v *= 0.5 * 9.80665 / peak;
    }
    let json = format!(
        r#"{{"schema": "fabrail.scenario/1", "name": "isolation_mc", "duration_s": {dur_s},
            "noise_seed": {seed},
            "fidelity": {{"model": "travel1d", "coil": "voltage"}},
            "initial": {{"lev": "levitated", "x_m": {x_mid}}},
            "record": {{"rate_hz": 100}},
            "series": {{"floor": {{"axis": "time", "fs_hz": 200, "channels": ["ax"]}}}}}}"#
    );
    (p, common::scenario(&json, &[("floor", vec![ax])]))
}

#[test]
#[ignore = "host- and profile-dependent timing; run on demand with --ignored --release"]
fn isolation_monte_carlo_wall_time() {
    // D8 budget: N = 200 × 60 s WALL isolation runs on Travel1d ≤ 60 s wall on 8 workers
    // (the slow SEI-015 study). Also checks that every run completes.
    let n: u64 = std::env::var("FABRAIL_PERF_MC_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(200);
    let jobs: Vec<_> = (0..n).map(|i| isolation_run(42 + i, 60.0)).collect();
    let t0 = Instant::now();
    let out = fabrail_sim::run_batch(&jobs, Some(8));
    let wall = t0.elapsed().as_secs_f64();
    let ok = out.iter().filter(|r| r.is_ok()).count();
    println!(
        "isolation MC: {n} × 60 s WALL Travel1d on 8 workers in {wall:.2} s wall \
         ({:.1}x real time aggregate, {ok}/{n} completed)",
        n as f64 * 60.0 / wall
    );
    assert_eq!(ok as u64, n, "every isolation run completes");
    let budget = 60.0 * n as f64 / 200.0;
    assert!(
        wall <= budget,
        "(S) isolation MC {wall:.1} s > {budget:.1} s budget (D8, core design §E.6)"
    );
}
