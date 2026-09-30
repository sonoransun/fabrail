//! Flat exo_scene_* bindings (scene validation and runs). Owner: E2 (surface frozen by W0a/W0-C1 in python/fabrail/_native.pyi).
//!
//! * `EXO_SCENE_SCHEMA`: `"fabrail.scene/1"`.
//! * `exo_scene_validate(spec_json)` → canonical JSON (defaults filled in), or
//!   `ValueError("path: reason")`.
//! * `exo_scene_run(spec_json, series=None, workers=None, quick=False,
//!   device_events_json=None)` → a dict
//!   {"meta": str (JSON), "rate_hz": float, "devices": list\[dict\], "building":
//!   dict[str, tuple[bytes, str, str]], "events": list[tuple[float, str | None, str,
//!   float, float]], "kpis": dict[str, float], "plan": str (JSON)}; device dicts carry
//!   id, class, policy, not_secured, kpis and channels {name: (bytes, dtype, unit)}.
//!   Channel bytes are little-endian (`np.frombuffer(b, dtype)`); events are (t_s,
//!   device id or None, kind, a, b) in time order; `plan` holds the house plan, the
//!   Gantt timeline and the compensation-stack waterfall; `meta` excludes nothing but
//!   `SceneMeta.hashed_json` drops `wall_s` for byte-identical comparisons. The run
//!   releases the GIL; results do not depend on `workers`. `ValueError("path: reason")`
//!   for a rejected scene or series.
//! * `exo_contents_cone(...)` → the vector friction cone of one sample.

use std::collections::BTreeMap;

use pyo3::exceptions::{PyNotImplementedError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict};

use crate::scene::contents::friction_cone;
use crate::scene::{
    SCENE_SCHEMA, SceneData, SceneError, SceneExtras, SceneRunOptions, SceneSpec, run_scene_with,
};

fn scene_err(e: SceneError) -> PyErr {
    match e {
        SceneError::NotImplemented(_) => PyNotImplementedError::new_err(e.to_string()),
        _ => PyValueError::new_err(e.to_string()),
    }
}

/// Validates a scene document; returns its canonical JSON.
#[pyfunction]
fn exo_scene_validate(spec_json: &str) -> PyResult<String> {
    SceneSpec::from_json(spec_json)
        .map(|s| s.to_json())
        .map_err(scene_err)
}

fn channels_dict<'py>(
    py: Python<'py>,
    chans: &[fabrail_sim::recorder::Channel],
) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    for c in chans {
        let b = PyBytes::new(py, &c.data.to_le_bytes());
        d.set_item(&c.name, (b, c.data.dtype(), &c.unit))?;
    }
    Ok(d)
}

fn scene_to_py<'py>(py: Python<'py>, s: &SceneData) -> PyResult<Bound<'py, PyDict>> {
    let out = PyDict::new(py);
    out.set_item("meta", serde_json::to_string(&s.meta).unwrap_or_default())?;
    out.set_item("rate_hz", s.rate_hz)?;
    let devices = s
        .devices
        .iter()
        .map(|d| {
            let dd = PyDict::new(py);
            dd.set_item("id", &d.id)?;
            dd.set_item("class", &d.class)?;
            dd.set_item("policy", &d.policy)?;
            dd.set_item("not_secured", d.not_secured.clone())?;
            dd.set_item("kpis", d.kpis.clone())?;
            dd.set_item("channels", channels_dict(py, &d.channels)?)?;
            Ok(dd)
        })
        .collect::<PyResult<Vec<_>>>()?;
    out.set_item("devices", devices)?;
    out.set_item("building", channels_dict(py, &s.building)?)?;
    let events: Vec<(f64, Option<String>, String, f64, f64)> = s
        .events
        .iter()
        .map(|e| {
            (
                e.event.t_s,
                e.device.clone(),
                e.event.kind.clone(),
                e.event.a,
                e.event.b,
            )
        })
        .collect();
    out.set_item("events", events)?;
    out.set_item("kpis", s.kpis.clone())?;
    out.set_item("plan", s.plan.to_string())?;
    Ok(out)
}

/// Runs a building scene (see the module docs for the result layout).
/// `device_events_json` (additive, Wave 2b E5): a JSON object {device id: [scenario
/// events]} appended to those devices' `fabrail.scenario/1` events (test-harness hands
/// and obstacles; not part of the scene document).
#[pyfunction]
#[pyo3(signature = (spec_json, series = None, workers = None, quick = false, device_events_json = None))]
fn exo_scene_run<'py>(
    py: Python<'py>,
    spec_json: &str,
    series: Option<BTreeMap<String, Vec<u8>>>,
    workers: Option<usize>,
    quick: bool,
    device_events_json: Option<&str>,
) -> PyResult<Bound<'py, PyDict>> {
    let spec = SceneSpec::from_json(spec_json).map_err(scene_err)?;
    let series = series.unwrap_or_default();
    let opts = SceneRunOptions { workers, quick };
    let extras = match device_events_json {
        None => SceneExtras::default(),
        Some(js) => {
            let device_events: BTreeMap<String, Vec<serde_json::Value>> =
                serde_json::from_str(js).map_err(|e| {
                    PyValueError::new_err(format!(
                        "device_events_json: expected {{device id: [events]}}: {e}"
                    ))
                })?;
            SceneExtras { device_events }
        }
    };
    let data = py
        .detach(move || run_scene_with(&spec, &series, &opts, &extras))
        .map_err(scene_err)?;
    scene_to_py(py, &data)
}

/// One friction-cone sample (plan §4b): {"a_h_mps2": (x, y), "g_eff_mps2", "utilisation",
/// "slips", "rocks"}.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
#[pyo3(signature = (a_drawer_x_mps2, a_floor_x_mps2, a_floor_y_mps2, a_floor_z_mps2, heading_deg, mu_s, b_over_h = None))]
fn exo_contents_cone<'py>(
    py: Python<'py>,
    a_drawer_x_mps2: f64,
    a_floor_x_mps2: f64,
    a_floor_y_mps2: f64,
    a_floor_z_mps2: f64,
    heading_deg: f64,
    mu_s: f64,
    b_over_h: Option<f64>,
) -> PyResult<Bound<'py, PyDict>> {
    let c = friction_cone(
        a_drawer_x_mps2,
        [a_floor_x_mps2, a_floor_y_mps2],
        a_floor_z_mps2,
        heading_deg.to_radians(),
        mu_s,
        b_over_h,
    );
    let d = PyDict::new(py);
    d.set_item("a_h_mps2", (c.a_h_mps2[0], c.a_h_mps2[1]))?;
    d.set_item("g_eff_mps2", c.g_eff_mps2)?;
    d.set_item("utilisation", c.utilisation)?;
    d.set_item("slips", c.slips)?;
    d.set_item("rocks", c.rocks)?;
    Ok(d)
}

/// Adds this module's constants and functions to `fabrail._native`.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("EXO_SCENE_SCHEMA", SCENE_SCHEMA)?;
    m.add_function(wrap_pyfunction!(exo_scene_validate, m)?)?;
    m.add_function(wrap_pyfunction!(exo_scene_run, m)?)?;
    m.add_function(wrap_pyfunction!(exo_contents_cone, m)?)?;
    Ok(())
}
