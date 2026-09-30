//! Scenario class: from_json(spec, series bytes), to_json; SCENARIO_SCHEMA. Owner: A8 (surface frozen by W0a/W0-C1 in python/fabrail/_native.pyi).
//!
//! Series cross as `dict[str, bytes]` of channel-major little-endian f64
//! (`fabrail_sim::scenario::encode_f64_le`). Errors map as: document, schema, series,
//! parameter and compatibility errors → `ValueError("path: reason")`; Wave-0 stubs →
//! `NotImplementedError`; runtime failures → `RuntimeError`.

use std::collections::BTreeMap;

use fabrail_sim::{SCENARIO_SCHEMA, Scenario, SimError};
use pyo3::exceptions::{PyNotImplementedError, PyRuntimeError, PyValueError};
use pyo3::prelude::*;

/// Maps a simulation error onto a Python exception.
pub fn sim_err(e: SimError) -> PyErr {
    match e {
        SimError::NotImplemented(_) => PyNotImplementedError::new_err(e.to_string()),
        SimError::Runtime(_) | SimError::Plant(_) => PyRuntimeError::new_err(e.to_string()),
        _ => PyValueError::new_err(e.to_string()),
    }
}

/// A validated scenario (`fabrail.scenario/1`) with its decoded series.
#[pyclass(name = "Scenario", module = "fabrail._native", from_py_object)]
#[derive(Clone, Debug)]
pub struct PyScenario {
    /// The scenario.
    pub inner: Scenario,
}

#[pymethods]
impl PyScenario {
    /// Parses `spec` and decodes `series` (name → channel-major LE f64 bytes).
    #[staticmethod]
    #[pyo3(signature = (spec, series = None))]
    fn from_json(spec: &str, series: Option<BTreeMap<String, Vec<u8>>>) -> PyResult<PyScenario> {
        let series = series.unwrap_or_default();
        Scenario::from_json(spec, &series)
            .map(|inner| PyScenario { inner })
            .map_err(sim_err)
    }

    /// The canonical document JSON (series bytes are not included).
    fn to_json(&self) -> String {
        self.inner.to_json()
    }

    /// The series as bytes (round trip of `from_json`'s input).
    fn series_bytes(&self) -> BTreeMap<String, Vec<u8>> {
        self.inner.series_bytes()
    }

    /// Scenario name.
    #[getter]
    fn name(&self) -> String {
        self.inner.spec.name.clone()
    }

    /// Duration, s.
    #[getter]
    fn duration_s(&self) -> f64 {
        self.inner.spec.duration_s
    }

    /// Rust noise seed.
    #[getter]
    fn noise_seed(&self) -> u64 {
        self.inner.spec.noise_seed
    }

    fn __repr__(&self) -> String {
        format!("Scenario(name='{}')", self.inner.spec.name)
    }
}

/// Adds this module's classes/constants to `fabrail._native`.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("SCENARIO_SCHEMA", SCENARIO_SCHEMA)?;
    m.add_class::<PyScenario>()?;
    Ok(())
}
