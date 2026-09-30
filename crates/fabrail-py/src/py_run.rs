//! run, run_batch (py.detach around every run) and the RunData class. Owner: A8 (surface frozen by W0a/W0-C1 in python/fabrail/_native.pyi).
//!
//! `run` and `run_batch` release the GIL (`py.detach`) for the whole simulation;
//! `run_batch` returns results in job order for any worker count (fabrail_sim::run_batch)
//! and raises the first failing job's error prefixed with `jobs[i]: `. Until A7 fills
//! the simulation loop both raise `NotImplementedError` after the up-front checks.

use std::collections::BTreeMap;

use fabrail_sim::RunData;
use pyo3::exceptions::PyKeyError;
use pyo3::prelude::*;
use pyo3::types::PyBytes;

use crate::py_params::PyParams;
use crate::py_scenario::{PyScenario, sim_err};

/// A finished run: uniform-rate channels as little-endian bytes, events, KPIs, meta.
#[pyclass(
    name = "RunData",
    module = "fabrail._native",
    frozen,
    skip_from_py_object
)]
#[derive(Clone, Debug)]
pub struct PyRunData {
    /// The run.
    pub inner: RunData,
}

#[pymethods]
impl PyRunData {
    /// Channel sample rate, Hz.
    #[getter]
    fn rate_hz(&self) -> f64 {
        self.inner.rate_hz
    }

    /// Channel names in recording order.
    fn channel_names(&self) -> Vec<String> {
        self.inner
            .channel_names()
            .into_iter()
            .map(str::to_string)
            .collect()
    }

    /// Channel `name` as (buffer, dtype `"<f8"` | `"u1"`, unit); `KeyError` if unknown.
    fn channel<'py>(
        &self,
        py: Python<'py>,
        name: &str,
    ) -> PyResult<(Bound<'py, PyBytes>, &'static str, String)> {
        let c = self
            .inner
            .channel(name)
            .ok_or_else(|| PyKeyError::new_err(format!("channel: unknown channel \"{name}\"")))?;
        Ok((
            PyBytes::new(py, &c.data.to_le_bytes()),
            c.data.dtype(),
            c.unit.clone(),
        ))
    }

    /// Samples per channel.
    fn n_samples(&self) -> usize {
        self.inner.n_samples()
    }

    /// Events as (t_s, kind, a, b).
    fn events(&self) -> Vec<(f64, String, f64, f64)> {
        self.inner
            .events
            .iter()
            .map(|e| (e.t_s, e.kind.clone(), e.a, e.b))
            .collect()
    }

    /// KPIs by name.
    fn kpis(&self) -> BTreeMap<String, f64> {
        self.inner.kpis.clone()
    }

    /// Provenance JSON (`RunMeta::to_json`).
    fn meta_json(&self) -> String {
        self.inner.meta.to_json()
    }

    fn __repr__(&self) -> String {
        format!(
            "RunData(scenario='{}', channels={}, samples={})",
            self.inner.meta.scenario,
            self.inner.channels.len(),
            self.inner.n_samples()
        )
    }
}

/// Runs one scenario (GIL released).
#[pyfunction]
fn run(py: Python<'_>, params: PyParams, scenario: PyScenario) -> PyResult<PyRunData> {
    let (p, s) = (params.inner, scenario.inner);
    py.detach(move || fabrail_sim::run(&p, &s))
        .map(|inner| PyRunData { inner })
        .map_err(sim_err)
}

/// Runs every job in parallel; results in job order (GIL released).
#[pyfunction]
#[pyo3(signature = (jobs, workers = None))]
fn run_batch(
    py: Python<'_>,
    jobs: Vec<(PyParams, PyScenario)>,
    workers: Option<usize>,
) -> PyResult<Vec<PyRunData>> {
    let jobs: Vec<_> = jobs.into_iter().map(|(p, s)| (p.inner, s.inner)).collect();
    let results = py.detach(move || fabrail_sim::run_batch(&jobs, workers));
    results
        .into_iter()
        .enumerate()
        .map(|(i, r)| {
            r.map(|inner| PyRunData { inner }).map_err(|e| {
                let err = sim_err(e);
                let msg = err.value(py).to_string();
                PyErr::from_type(err.get_type(py), format!("jobs[{i}]: {msg}"))
            })
        })
        .collect()
}

/// Adds this module's classes/functions to `fabrail._native`.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRunData>()?;
    m.add_function(wrap_pyfunction!(run, m)?)?;
    m.add_function(wrap_pyfunction!(run_batch, m)?)?;
    Ok(())
}
