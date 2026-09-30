//! fabrail-params: the single source of parameters for sim, Python, docs and firmware
//! (core design §B.2; plan §2b item 8).
//!
//! * [`design`]: the nominal hardware ([`DesignParams`]).
//! * [`control`]: tuning ratios and bandwidths ([`ControlParams`]); gains are derived.
//! * [`safety`]: the monitor's limits and diverse calibration copies ([`SafetyLimits`]).
//! * [`presets`]: `drawer()`, `door()`, `wall()`, `heavy()`, `by_class()`, and the
//!   pedestrian caps that exotic code imports (never duplicates).
//! * [`derive`](mod@derive): f64 closed forms, each citing its PHYSICS §.
//! * [`validate`](mod@validate): ranges and cross-field rules, `path: reason` errors.
//! * [`meta`]: per-field unit/range/source metadata (drives PARAMETERS.md).
//! * [`crc`]: CRC32 and the safety-block seal.
//!
//! Every parameter field is f64 SI with a unit suffix; controllers cast to f32 in
//! `new()` and the monitor derives integer thresholds in `new()`.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod control;
pub mod crc;
pub mod derive;
pub mod design;
pub mod meta;
pub mod presets;
pub mod safety;
pub mod validate;

pub use control::*;
pub use crc::{crc32, seal};
pub use design::*;
pub use safety::*;
pub use validate::validate;

pub use fabrail_types::{EgressRole, PresenceSensing, RailClass, RestPolicy, SeismicAction};

/// A complete parameter set: design (what was built), control (how it is tuned) and
/// safety (what the monitor enforces).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct SystemParams {
    /// Nominal hardware.
    pub design: DesignParams,
    /// Tuning.
    pub control: ControlParams,
    /// Monitor limits.
    pub safety: SafetyLimits,
}

impl SystemParams {
    /// Validates this parameter set (see [`validate()`]).
    #[inline]
    pub fn validate(&self) -> Result<(), ParamError> {
        validate::validate(self)
    }
}

/// A rejected parameter: the dotted field path (as in [`meta::FIELDS`]) and a reason.
/// Python surfaces it as `ValueError("path: reason")`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ParamError {
    /// Dotted field path, e.g. `"design.suspension.g_up_m"`.
    pub path: &'static str,
    /// Human-readable reason.
    pub reason: &'static str,
}

impl ParamError {
    /// Constructs an error.
    #[inline]
    pub const fn new(path: &'static str, reason: &'static str) -> ParamError {
        ParamError { path, reason }
    }
}

impl core::fmt::Display for ParamError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}: {}", self.path, self.reason)
    }
}

impl core::error::Error for ParamError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn param_error_displays_path_colon_reason() {
        let e = ParamError::new("design.suspension.g_up_m", "must be below g_star_m");
        assert_eq!(
            e.to_string(),
            "design.suspension.g_up_m: must be below g_star_m"
        );
    }

    #[test]
    fn default_params_are_constructible() {
        let p = SystemParams::default();
        assert_eq!(p.design.class, RailClass::Door);
        assert!(
            !p.control.motion.inertial.enable,
            "inertial smoothing is off by default"
        );
        assert!(!p.control.seismic.isolation.enable);
        assert!(!p.control.seismic.amd.enable);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_json_roundtrip_and_pointer_paths() {
        let mut p = presets::drawer();
        p.design.suspension.g_star_m = 2.5e-3;
        p.design.suspension.stations[3].has_pm = true;
        p.control.motion.terminus.tau_s = 0.15;
        p.safety.nm_per_count_copy = 1000;
        let s = serde_json::to_string(&p).expect("serialise");
        let back: SystemParams = serde_json::from_str(&s).expect("deserialise");
        assert_eq!(back, p);
        let v = serde_json::to_value(p).expect("to value");
        // Dotted meta paths map to JSON pointers with '/' separators.
        assert_eq!(
            v.pointer("/design/class").and_then(|x| x.as_str()),
            Some("drawer")
        );
        assert_eq!(
            v.pointer("/design/rest_policy").and_then(|x| x.as_str()),
            Some("seat")
        );
        assert_eq!(
            v.pointer("/design/suspension/g_star_m")
                .and_then(|x| x.as_f64()),
            Some(2.5e-3)
        );
        assert_eq!(
            v.pointer("/design/suspension/stations/3/has_pm")
                .and_then(|x| x.as_bool()),
            Some(true)
        );
        assert_eq!(
            v.pointer("/control/motion/terminus/tau_s")
                .and_then(|x| x.as_f64()),
            Some(0.15)
        );
        // Unknown fields are rejected (typos in --set / from_json surface as errors).
        let bad = s.replacen("\"g_star_m\"", "\"g_star\"", 1);
        assert!(serde_json::from_str::<SystemParams>(&bad).is_err());
    }
}
