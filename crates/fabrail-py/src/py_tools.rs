//! Tools and constants: stability_report, stray_field, selftest_hash, BASE_TICK_HZ, MAX_MAGNETS, GOLDEN_SELFTEST_HASH. Owner: A8 (surface frozen by W0a/W0-C1 in python/fabrail/_native.pyi).

use std::collections::BTreeMap;

use fabrail_rt::selftest::{
    GOLDEN_SELFTEST_HASH, SELFTEST_TICKS, selftest_hash as rt_selftest_hash,
};
use fabrail_types::{BASE_TICK_HZ, MAX_MAGNETS};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyBytes;

use crate::py_detect::decode_f64;
use crate::py_params::PyParams;
use crate::py_scenario::{PyScenario, sim_err};

/// Per-element |λ·h| of the plant for this parameter set and scenario (GIL released).
#[pyfunction]
fn stability_report(
    py: Python<'_>,
    params: PyParams,
    scenario: PyScenario,
) -> PyResult<BTreeMap<String, f64>> {
    let (p, s) = (params.inner, scenario.inner);
    py.detach(move || fabrail_sim::stability_report(&p, &s))
        .map_err(sim_err)
}

/// Stray field at points (n×3 LE f64 bytes, m) → n×3 LE f64 bytes, T (GIL released).
#[pyfunction]
fn stray_field<'py>(
    py: Python<'py>,
    params: PyParams,
    points_xyz: Vec<u8>,
) -> PyResult<Bound<'py, PyBytes>> {
    let flat = decode_f64("points_xyz", &points_xyz)?;
    if !flat.len().is_multiple_of(3) {
        return Err(PyValueError::new_err(format!(
            "points_xyz: {} values is not a whole number of (x, y, z) points",
            flat.len()
        )));
    }
    if let Some(bad) = flat.iter().find(|v| !v.is_finite()) {
        return Err(PyValueError::new_err(format!(
            "points_xyz: every coordinate must be finite (got {bad})"
        )));
    }
    let pts: Vec<[f64; 3]> = flat.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
    let design = params.inner.design;
    let out: Vec<u8> = py.detach(move || {
        fabrail_plant::stray::field(&design, &pts)
            .iter()
            .flat_map(|v| v.iter().flat_map(|x| x.to_le_bytes()))
            .collect()
    });
    Ok(PyBytes::new(py, &out))
}

/// The `fabrail_rt` selftest hash over `n_ticks` base ticks (GIL released).
#[pyfunction]
#[pyo3(signature = (n_ticks = SELFTEST_TICKS))]
fn selftest_hash(py: Python<'_>, n_ticks: u32) -> u64 {
    py.detach(move || rt_selftest_hash(n_ticks))
}

/// Adds this module's constants and functions to `fabrail._native`.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("BASE_TICK_HZ", BASE_TICK_HZ)?;
    m.add("MAX_MAGNETS", MAX_MAGNETS)?;
    m.add("GOLDEN_SELFTEST_HASH", GOLDEN_SELFTEST_HASH)?;
    m.add_function(wrap_pyfunction!(stability_report, m)?)?;
    m.add_function(wrap_pyfunction!(stray_field, m)?)?;
    m.add_function(wrap_pyfunction!(selftest_hash, m)?)?;
    Ok(())
}
