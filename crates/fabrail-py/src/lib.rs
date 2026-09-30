//! fabrail-py: the `fabrail._native` extension module (core design §B.7).
//!
//! Arrays cross as little-endian `bytes` (no rust-numpy); the Python wrappers call
//! `np.frombuffer`. The surface is drift-guarded against `python/fabrail/_native.pyi`.
//! Each binding module exposes `register(m)`; the flat `exo_*` / `exo_scene_*`
//! functions come from `fabrail_exo_sim::py::register` (no PyO3 submodule).
//! No `forbid(unsafe_code)` here: the PyO3 macros expand to `unsafe` FFI glue.

use pyo3::prelude::*;

pub mod py_analytic;
pub mod py_detect;
pub mod py_params;
pub mod py_run;
pub mod py_scenario;
pub mod py_tools;

/// Cargo profile the extension was built with; `tests/conftest.py` refuses a debug
/// build (plan §2b item 13).
pub const BUILD_PROFILE: &str = if cfg!(debug_assertions) {
    "debug"
} else {
    "release"
};

#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add("BUILD_PROFILE", BUILD_PROFILE)?;
    py_params::register(m)?;
    py_scenario::register(m)?;
    py_run::register(m)?;
    py_detect::register(m)?;
    py_analytic::register(m)?;
    py_tools::register(m)?;
    fabrail_exo_sim::py::register(m)?;
    Ok(())
}
