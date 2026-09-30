//! Params class: preset, from_json, describe, to_json, get/set (serde_json pointer), with_overrides, validate, derived; RailClass. Owner: A8 (surface frozen by W0a/W0-C1 in python/fabrail/_native.pyi).
//!
//! `Params` wraps `fabrail_params::SystemParams` (the single source of presets).
//! Paths are the dotted `fabrail_params::meta` paths (`design.suspension.g_star_m`,
//! array elements by index: `design.suspension.stations.0.x_m`) or the equivalent JSON
//! pointer (`/design/suspension/g_star_m`). `set` and `with_overrides` keep the JSON type
//! of the leaf (bool, integer, float, enum string), re-seal the safety block (diverse
//! calibration copies and CRC follow the design values; `crc::seal`) unless the path is
//! itself a diverse copy or the CRC, and re-validate; a rejected edit raises
//! `ValueError("path: reason")` and leaves the object unchanged.

use std::collections::BTreeMap;

use fabrail_params::derive;
use fabrail_params::{SystemParams, crc, meta, presets};
use fabrail_types::RailClass as CoreRailClass;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyFloat, PyInt, PyString};
use serde_json::Value;

/// Product class of a rail (mirrors `fabrail_types::RailClass`).
#[pyclass(
    name = "RailClass",
    module = "fabrail._native",
    eq,
    eq_int,
    frozen,
    hash,
    from_py_object,
    rename_all = "SCREAMING_SNAKE_CASE"
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PyRailClass {
    /// Cabinet drawer.
    Drawer,
    /// Sliding door.
    Door,
    /// Partition wall.
    Wall,
    /// Infrastructure panel.
    Heavy,
}

impl PyRailClass {
    /// From the core enum.
    pub fn from_core(c: CoreRailClass) -> PyRailClass {
        match c {
            CoreRailClass::Drawer => PyRailClass::Drawer,
            CoreRailClass::Door => PyRailClass::Door,
            CoreRailClass::Wall => PyRailClass::Wall,
            CoreRailClass::Heavy => PyRailClass::Heavy,
        }
    }

    /// To the core enum.
    pub fn to_core(self) -> CoreRailClass {
        match self {
            PyRailClass::Drawer => CoreRailClass::Drawer,
            PyRailClass::Door => CoreRailClass::Door,
            PyRailClass::Wall => CoreRailClass::Wall,
            PyRailClass::Heavy => CoreRailClass::Heavy,
        }
    }
}

#[pymethods]
impl PyRailClass {
    /// Lower-case class name (`"drawer"`, …), as used by `Params.preset`.
    #[getter]
    fn key(&self) -> &'static str {
        presets::class_name(self.to_core())
    }
}

/// A complete, validated parameter set.
#[pyclass(name = "Params", module = "fabrail._native", eq, from_py_object)]
#[derive(Clone, Debug, PartialEq)]
pub struct PyParams {
    /// The parameter set.
    pub inner: SystemParams,
}

/// `path: reason` as a Python ValueError.
fn value_err(path: &str, reason: impl std::fmt::Display) -> PyErr {
    PyValueError::new_err(format!("{path}: {reason}"))
}

/// Dotted path or JSON pointer → JSON pointer.
fn pointer(path: &str) -> String {
    if path.starts_with('/') {
        path.to_string()
    } else {
        format!("/{}", path.replace('.', "/"))
    }
}

/// JSON pointer → dotted path (for messages).
fn dotted(path: &str) -> String {
    path.trim_start_matches('/').replace('/', ".")
}

/// Paths `set` does not re-seal (editing them is a deliberate seal mismatch).
const UNSEALED_PATHS: [&str; 6] = [
    "safety.k_f_copy_n_per_a",
    "safety.adc_a_per_code_copy",
    "safety.lsm_a_per_code_copy",
    "safety.nm_per_count_copy",
    "safety.presence_copy",
    "safety.crc",
];

fn json_to_py(py: Python<'_>, v: &Value, path: &str) -> PyResult<Py<PyAny>> {
    Ok(match v {
        Value::Bool(b) => PyBool::new(py, *b).to_owned().into_any().unbind(),
        Value::Number(n) => {
            if let Some(u) = n.as_u64() {
                u.into_pyobject(py)?.into_any().unbind()
            } else if let Some(i) = n.as_i64() {
                i.into_pyobject(py)?.into_any().unbind()
            } else {
                PyFloat::new(py, n.as_f64().unwrap_or(f64::NAN))
                    .into_any()
                    .unbind()
            }
        }
        Value::String(s) => PyString::new(py, s).into_any().unbind(),
        _ => return Err(value_err(path, "not a leaf parameter (an object or array)")),
    })
}

/// Converts a Python scalar to the JSON type of the existing leaf.
fn py_to_json(old: &Value, value: &Bound<'_, PyAny>, path: &str) -> PyResult<Value> {
    match old {
        Value::Bool(_) => {
            if value.is_instance_of::<PyBool>() {
                Ok(Value::Bool(value.extract::<bool>()?))
            } else {
                Err(value_err(path, "expects a bool"))
            }
        }
        Value::String(_) => value
            .extract::<String>()
            .map(Value::String)
            .map_err(|_| value_err(path, "expects a string (enum name)")),
        Value::Number(n) => {
            if value.is_instance_of::<PyBool>() {
                return Err(value_err(path, "expects a number, got a bool"));
            }
            let x: f64 = value
                .extract()
                .map_err(|_| value_err(path, "expects a number"))?;
            if !x.is_finite() {
                return Err(value_err(path, "must be finite"));
            }
            if n.is_u64() || n.is_i64() {
                let is_int = value.is_instance_of::<PyInt>() || x.fract() == 0.0;
                if !is_int {
                    return Err(value_err(path, "expects an integer"));
                }
                if value.is_instance_of::<PyInt>() {
                    let i: i128 = value
                        .extract()
                        .map_err(|_| value_err(path, "integer out of range"))?;
                    return if i >= 0 {
                        u64::try_from(i)
                            .map(|u| Value::Number(u.into()))
                            .map_err(|_| value_err(path, "integer out of range"))
                    } else {
                        i64::try_from(i)
                            .map(|v| Value::Number(v.into()))
                            .map_err(|_| value_err(path, "integer out of range"))
                    };
                }
                return if x >= 0.0 {
                    Ok(Value::Number((x as u64).into()))
                } else {
                    Ok(Value::Number((x as i64).into()))
                };
            }
            serde_json::Number::from_f64(x)
                .map(Value::Number)
                .ok_or_else(|| value_err(path, "must be finite"))
        }
        _ => Err(value_err(path, "not a leaf parameter (an object or array)")),
    }
}

impl PyParams {
    /// Applies one edit to a JSON document of the parameters.
    fn apply(doc: &mut Value, path: &str, value: &Bound<'_, PyAny>) -> PyResult<()> {
        let ptr = pointer(path);
        let name = dotted(&ptr);
        let slot = doc
            .pointer_mut(&ptr)
            .ok_or_else(|| value_err(&name, "unknown parameter"))?;
        let new = py_to_json(slot, value, &name)?;
        *slot = new;
        Ok(())
    }

    /// Decodes an edited document, seals (unless told not to) and validates.
    fn finish(doc: Value, reseal: bool) -> PyResult<SystemParams> {
        let mut p: SystemParams = serde_json::from_value(doc)
            .map_err(|e| PyValueError::new_err(format!("params: {e}")))?;
        if reseal {
            crc::seal(&mut p);
        }
        p.validate()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(p)
    }

    fn to_doc(&self) -> PyResult<Value> {
        serde_json::to_value(self.inner).map_err(|e| PyValueError::new_err(format!("params: {e}")))
    }
}

#[pymethods]
impl PyParams {
    /// The preset of class `cls` (`"drawer" | "door" | "wall" | "heavy"`).
    #[staticmethod]
    fn preset(cls: &str) -> PyResult<PyParams> {
        let c = presets::class_from_name(cls).ok_or_else(|| {
            value_err(
                "class",
                format!("unknown class \"{cls}\" (drawer, door, wall, heavy)"),
            )
        })?;
        Ok(PyParams {
            inner: presets::by_class(c),
        })
    }

    /// Parses and validates a JSON parameter document, then checks its seal.
    ///
    /// The document must be sealed (`crc::verify_seal`: the diverse copies equal the
    /// design values and `safety.crc` matches; SAF-015), exactly as the firmware checks
    /// its parameter store at boot. Every document written by `to_json` is sealed; edit
    /// through `set`/`with_overrides`, which reseal, rather than by hand.
    #[staticmethod]
    fn from_json(s: &str) -> PyResult<PyParams> {
        let p: SystemParams =
            serde_json::from_str(s).map_err(|e| PyValueError::new_err(format!("params: {e}")))?;
        p.validate()
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        crc::verify_seal(&p).map_err(|e| {
            PyValueError::new_err(format!(
                "{e}; edit parameters with Params.set or Params.with_overrides, which reseal"
            ))
        })?;
        Ok(PyParams { inner: p })
    }

    /// Metadata of every parameter leaf: path, unit, min, max, source, default_by_class.
    #[staticmethod]
    fn describe(py: Python<'_>) -> PyResult<Vec<Bound<'_, PyDict>>> {
        let by_class: Vec<(&str, SystemParams)> = presets::CLASSES
            .iter()
            .map(|c| (presets::class_name(*c), presets::by_class(*c)))
            .collect();
        meta::FIELDS
            .iter()
            .enumerate()
            .map(|(i, f)| {
                let d = PyDict::new(py);
                d.set_item("path", f.path)?;
                d.set_item("unit", f.unit)?;
                d.set_item("min", f.min)?;
                d.set_item("max", f.max)?;
                d.set_item("source", f.source)?;
                let defaults: BTreeMap<&str, f64> = by_class
                    .iter()
                    .map(|(n, p)| (*n, meta::value_at(p, i).unwrap_or(f64::NAN)))
                    .collect();
                d.set_item("default_by_class", defaults)?;
                Ok(d)
            })
            .collect()
    }

    /// JSON document (compact, or indented by `indent` spaces).
    #[pyo3(signature = (indent = None))]
    fn to_json(&self, indent: Option<usize>) -> PyResult<String> {
        let err = |e: serde_json::Error| PyValueError::new_err(format!("params: {e}"));
        match indent {
            None => serde_json::to_string(&self.inner).map_err(err),
            Some(n) => {
                // serde_json pretty-prints with 2-space steps; JSON strings never contain a
                // raw newline, so re-indenting line by line is exact.
                let pretty = serde_json::to_string_pretty(&self.inner).map_err(err)?;
                Ok(pretty
                    .lines()
                    .map(|l| {
                        let body = l.trim_start_matches(' ');
                        let depth = (l.len() - body.len()) / 2;
                        format!("{}{body}", " ".repeat(depth * n))
                    })
                    .collect::<Vec<_>>()
                    .join("\n"))
            }
        }
    }

    /// The value of leaf `path` (float, int, bool or enum string).
    fn get(&self, py: Python<'_>, path: &str) -> PyResult<Py<PyAny>> {
        let doc = self.to_doc()?;
        let ptr = pointer(path);
        let name = dotted(&ptr);
        let v = doc
            .pointer(&ptr)
            .ok_or_else(|| value_err(&name, "unknown parameter"))?;
        json_to_py(py, v, &name)
    }

    /// Sets leaf `path` to `value`, re-seals and re-validates (see the module docs).
    fn set(&mut self, path: &str, value: &Bound<'_, PyAny>) -> PyResult<()> {
        let mut doc = self.to_doc()?;
        PyParams::apply(&mut doc, path, value)?;
        let reseal = !UNSEALED_PATHS.contains(&dotted(&pointer(path)).as_str());
        self.inner = PyParams::finish(doc, reseal)?;
        Ok(())
    }

    /// A copy with every `{path: value}` override applied, then sealed and validated.
    fn with_overrides(&self, overrides: &Bound<'_, PyDict>) -> PyResult<PyParams> {
        let mut doc = self.to_doc()?;
        let mut reseal = true;
        for (k, v) in overrides.iter() {
            let path: String = k
                .extract()
                .map_err(|_| PyValueError::new_err("overrides: keys must be str paths"))?;
            PyParams::apply(&mut doc, &path, &v)?;
            if UNSEALED_PATHS.contains(&dotted(&pointer(&path)).as_str()) {
                reseal = false;
            }
        }
        Ok(PyParams {
            inner: PyParams::finish(doc, reseal)?,
        })
    }

    /// Validates; raises `ValueError("path: reason")`.
    fn validate(&self) -> PyResult<()> {
        self.inner
            .validate()
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Named derived quantities (`fabrail_params::derive::derived`), SI with unit
    /// suffixes (e.g. `pole_rad_s`).
    fn derived(&self) -> BTreeMap<String, f64> {
        derive::derived(&self.inner)
            .iter()
            .map(|(k, v)| (k.to_string(), *v))
            .collect()
    }

    /// The rail class.
    #[getter]
    fn rail_class(&self) -> PyRailClass {
        PyRailClass::from_core(self.inner.design.class)
    }

    fn __repr__(&self) -> String {
        format!(
            "Params(class='{}')",
            presets::class_name(self.inner.design.class)
        )
    }
}

/// Adds this module's classes/functions to `fabrail._native`.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRailClass>()?;
    m.add_class::<PyParams>()?;
    Ok(())
}
