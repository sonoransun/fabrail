//! PyO3 glue (feature `py`): `register(m)` adds the flat `exo_*` and `exo_scene_*`
//! functions to `fabrail._native` (no submodule). Owner: X6 (hook frozen by W0a).

use pyo3::prelude::*;

pub mod exo;
pub mod scene;

/// Adds every exotic function to `fabrail._native`.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    exo::register(m)?;
    scene::register(m)?;
    Ok(())
}
