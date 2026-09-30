//! DetectorReplay class. Owner: A8 (surface frozen by W0a/W0-C1 in python/fabrail/_native.pyi).
//!
//! Streams structure acceleration (m/s², little-endian f64 bytes per axis) through the
//! firmware seismic front end (`fabrail_sim::DetectorReplay`), chunk by chunk (the 24 h
//! noise study feeds 10-minute chunks). Report keys are frozen: samples, duration_s,
//! fs_hz, raw_triggers, freeze_count, protect_count, level_max.

use std::collections::BTreeMap;

use fabrail_sim::DetectorReplay;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::py_params::PyParams;

/// Decodes little-endian f64 samples.
pub fn decode_f64(name: &str, b: &[u8]) -> PyResult<Vec<f64>> {
    if !b.len().is_multiple_of(8) {
        return Err(PyValueError::new_err(format!(
            "{name}: {} bytes is not a whole number of f64 samples",
            b.len()
        )));
    }
    Ok(b.chunks_exact(8)
        .map(|c| f64::from_le_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]))
        .collect())
}

/// Streaming seismic detector replay.
#[pyclass(
    name = "DetectorReplay",
    module = "fabrail._native",
    skip_from_py_object
)]
pub struct PyDetectorReplay {
    inner: DetectorReplay,
}

#[pymethods]
impl PyDetectorReplay {
    /// A detector for `params` fed at `fs_hz` (the ADXL355 ODR, 500 Hz by default).
    /// `ValueError("fs_hz: …")` unless the rate is finite and positive.
    #[new]
    #[pyo3(signature = (params, fs_hz = 500.0))]
    fn new(params: PyParams, fs_hz: f64) -> PyResult<PyDetectorReplay> {
        if !(fs_hz.is_finite() && fs_hz > 0.0) {
            return Err(PyValueError::new_err(format!(
                "fs_hz: must be a finite positive sample rate (got {fs_hz})"
            )));
        }
        Ok(PyDetectorReplay {
            inner: DetectorReplay::new(&params.inner, fs_hz),
        })
    }

    /// Feeds one chunk (x, y, z as LE f64 bytes, m/s²); returns the events raised in it
    /// as (t_s, kind, a, b). The three axes must hold the same number of samples.
    fn feed(
        &mut self,
        py: Python<'_>,
        ax: Vec<u8>,
        ay: Vec<u8>,
        az: Vec<u8>,
    ) -> PyResult<Vec<(f64, String, f64, f64)>> {
        let (x, y, z) = (
            decode_f64("ax", &ax)?,
            decode_f64("ay", &ay)?,
            decode_f64("az", &az)?,
        );
        if x.len() != y.len() || x.len() != z.len() {
            return Err(PyValueError::new_err(format!(
                "feed: ax, ay and az differ in length ({}, {}, {} samples)",
                x.len(),
                y.len(),
                z.len()
            )));
        }
        let det = &mut self.inner;
        let ev = py.detach(move || det.feed(&x, &y, &z));
        Ok(ev.into_iter().map(|e| (e.t_s, e.kind, e.a, e.b)).collect())
    }

    /// Summary so far (frozen keys; see the module docs).
    fn report(&self) -> BTreeMap<String, f64> {
        self.inner.report()
    }
}

/// Adds this module's classes/functions to `fabrail._native`.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyDetectorReplay>()?;
    Ok(())
}
