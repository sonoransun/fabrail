//! Base-tick simulation loop and run(params, scenario) -> RunData. Owner: A7 (signatures frozen by W0-B3).
//!
//! Per base tick k (core design §B.6; PHYSICS §24):
//! 1. apply scenario events due at k (`events`);
//! 2. SimBoard snapshots truth at t_k;
//! 3. `Executive::on_base_tick` (LET: base first, then any order of fast, supervisor
//!    and seismic, plan §2b item 9);
//! 4. latch commands (zero-order hold over the plant step);
//! 5. RK4 `plant.step` with the floor acceleration at t, t + h/2 and t + h (the floor
//!    series linearly interpolated); Travel1d/Structure step every 20 ticks;
//! 6. metrics at the base rate;
//! 7. recorder at `record_decimation()`.
//!
//! The loop itself lives in the private `engine` module, shared with
//! `replay::record` and `Session`. [`run`] performs every up-front check (parameter
//! validation, scenario/parameter compatibility including the coil-mode single source,
//! the plant stability report) before it starts.
//!
//! Output: channels sampled at `record.rate_hz` from t = 0 to t = duration inclusive
//! (sample k at t = k/rate_hz), controller, scenario and plant events in time order,
//! the KPIs of `metrics` and the provenance (`RunMeta.golden` =
//! `fabrail_rt::GOLDEN_SELFTEST_HASH`, `build_profile` from `debug_assertions`).

use std::collections::BTreeMap;

use fabrail_core::seismic::action_policy::CorePolicy;
use fabrail_params::SystemParams;
use fabrail_plant::stability::{self, StabilityReport};

use crate::engine::{Engine, EngineOpts, FloorInput};
use crate::recorder::{RunData, RunMeta};
use crate::replay::TraceRecord;
use crate::scenario::{Scenario, SimError, check_compat};

/// The up-front checks shared by [`run`], [`stability_report`] and `Session::new`.
fn preflight(p: &SystemParams, s: &Scenario) -> Result<StabilityReport, SimError> {
    p.validate()?;
    s.spec.validate()?;
    check_compat(p, &s.spec)?;
    let pp = s.plant_params(p)?;
    Ok(stability::report_tier(
        &pp,
        s.spec.tier(),
        s.spec.tier().nominal_h_s(),
    ))
}

/// Per-element |λ·h| for this parameter set and scenario (Python `stability_report`).
pub fn stability_report(p: &SystemParams, s: &Scenario) -> Result<BTreeMap<String, f64>, SimError> {
    Ok(preflight(p, s)?.to_map())
}

/// The scenario's floor and site series as a floor input.
struct ScenarioFloor<'a>(&'a Scenario);

impl FloorInput for ScenarioFloor<'_> {
    fn floor(&self, t_s: f64) -> [f64; 3] {
        self.0.floor_mps2(t_s)
    }
    fn site(&self, t_s: f64) -> [f64; 3] {
        self.0.site_mps2(t_s)
    }
}

/// Wall-clock stopwatch (absent on wasm32, where `Instant::now` is unavailable).
struct Stopwatch {
    #[cfg(not(target_arch = "wasm32"))]
    t0: std::time::Instant,
}

impl Stopwatch {
    fn start() -> Stopwatch {
        Stopwatch {
            #[cfg(not(target_arch = "wasm32"))]
            t0: std::time::Instant::now(),
        }
    }
    fn elapsed_s(&self) -> f64 {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.t0.elapsed().as_secs_f64()
        }
        #[cfg(target_arch = "wasm32")]
        {
            0.0
        }
    }
}

fn run_inner(
    p: &SystemParams,
    s: &Scenario,
    trace: bool,
) -> Result<(RunData, Option<TraceRecord>), SimError> {
    let rep = preflight(p, s)?;
    if !rep.ok() {
        let (element, lambda_h) = rep
            .worst_oscillatory()
            .map_or((String::new(), f64::NAN), |e| {
                (e.element.clone(), e.lambda_h)
            });
        return Err(SimError::Unstable { element, lambda_h });
    }
    let watch = Stopwatch::start();
    let pp = s.plant_params(p)?;
    let mut eng = Engine::new(
        p,
        &s.spec,
        &pp,
        EngineOpts {
            record: true,
            metrics: true,
            trace,
            device: 0,
        },
    )?;
    let floor = ScenarioFloor(s);
    let n = s.spec.duration_ticks();
    for _ in 0..n {
        eng.step(&floor, &mut CorePolicy)?;
    }
    eng.record_final();
    let mut events = core::mem::take(&mut eng.log);
    events.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    let meta = RunMeta {
        version: env!("CARGO_PKG_VERSION").to_string(),
        scenario: s.spec.name.clone(),
        noise_seed: s.spec.noise_seed,
        fidelity: s.spec.tier().name().to_string(),
        coil: if s.spec.fidelity.coil.is_voltage() {
            "voltage".into()
        } else {
            "lag".into()
        },
        params: serde_json::to_value(p).unwrap_or(serde_json::Value::Null),
        spec: serde_json::to_value(&s.spec).unwrap_or(serde_json::Value::Null),
        golden: fabrail_rt::GOLDEN_SELFTEST_HASH,
        build_profile: if cfg!(debug_assertions) {
            "debug".into()
        } else {
            "release".into()
        },
        wall_s: 0.0,
    };
    let kpis = eng.kpis();
    let channels = eng.take_channels();
    let trace = eng.take_trace();
    let data = RunData {
        rate_hz: s.spec.record.rate_hz,
        channels,
        events,
        kpis,
        meta: RunMeta {
            wall_s: watch.elapsed_s(),
            ..meta
        },
    };
    Ok((data, trace))
}

/// Runs one scenario (core design §B.6). Refuses a plant step that is too large for an
/// oscillatory element (`SimError::Unstable`, PHYSICS §24).
pub fn run(p: &SystemParams, s: &Scenario) -> Result<RunData, SimError> {
    run_inner(p, s, false).map(|(d, _)| d)
}

/// [`run`] with the HAL traffic recorded for `replay` (crate-internal entry point of
/// `replay::record`).
pub(crate) fn run_traced(
    p: &SystemParams,
    s: &Scenario,
) -> Result<(RunData, TraceRecord), SimError> {
    let (d, t) = run_inner(p, s, true)?;
    Ok((d, t.unwrap_or_default()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario(json: &str) -> Scenario {
        Scenario::from_json(json, &BTreeMap::new()).expect("valid scenario")
    }

    #[test]
    fn run_produces_uniform_channels_events_and_meta() {
        let p = fabrail_params::presets::door();
        let s = scenario(
            r#"{"schema": "fabrail.scenario/1", "name": "t", "duration_s": 0.05, "noise_seed": 3,
                "fidelity": {"model": "carriage6", "coil": "voltage"},
                "record": {"rate_hz": 1000},
                "events": [{"kind": "command", "t_s": 0.01, "cmd": "open"}]}"#,
        );
        let d = run(&p, &s).expect("run");
        assert_eq!(d.rate_hz, 1000.0);
        assert_eq!(d.n_samples(), 51, "0 … 50 ms inclusive at 1 kHz");
        assert!(d.channels.iter().all(|c| c.data.len() == 51));
        assert!(d.channel("x_m").is_some() && d.channel("lev_state").is_some());
        assert!(d.events.iter().any(|e| e.kind == "scenario:command"));
        assert!(
            d.events.windows(2).all(|w| w[0].t_s <= w[1].t_s),
            "time order"
        );
        assert_eq!(d.meta.coil, "voltage");
        assert_eq!(d.meta.fidelity, "carriage6");
        assert_eq!(d.meta.golden, fabrail_rt::GOLDEN_SELFTEST_HASH);
        for k in [
            "ke_max_gov_j",
            "level_max",
            "stroke_max_m",
            "bumper_impacts",
        ] {
            assert!(d.kpis.contains_key(k), "always-present KPI {k}");
        }
        assert!(d.kpis.values().all(|v| v.is_finite()));
    }

    #[test]
    fn run_rejects_bad_setups() {
        let p = fabrail_params::presets::door();
        let lag = scenario(
            r#"{"schema": "fabrail.scenario/1", "name": "t", "duration_s": 0.01, "noise_seed": 3,
                "fidelity": {"model": "carriage6", "coil": "lag"}}"#,
        );
        assert!(matches!(
            run(&p, &lag),
            Err(SimError::CoilModeMismatch { .. })
        ));
        let unknown = scenario(
            r#"{"schema": "fabrail.scenario/1", "name": "t", "duration_s": 0.01, "noise_seed": 3,
                "fidelity": {"model": "carriage6", "coil": "voltage"},
                "record": {"channels": ["x_m", "no_such_channel"]}}"#,
        );
        assert_eq!(
            run(&p, &unknown).expect_err("unknown channel"),
            SimError::UnknownChannel("no_such_channel".into())
        );
    }

    #[test]
    fn travel1d_records_on_the_plant_step_grid() {
        let p = fabrail_params::presets::wall();
        let s = scenario(
            r#"{"schema": "fabrail.scenario/1", "name": "t", "duration_s": 0.1, "noise_seed": 3,
                "fidelity": {"model": "travel1d", "coil": "voltage"},
                "record": {"rate_hz": 100, "channels": ["x_m", "floor_ax_mps2", "hand_active"]}}"#,
        );
        let d = run(&p, &s).expect("run");
        assert_eq!(d.n_samples(), 11);
        assert_eq!(d.channel_names(), ["x_m", "floor_ax_mps2", "hand_active"]);
        assert_eq!(d.channel("hand_active").expect("u8").data.dtype(), "u1");
    }
}
