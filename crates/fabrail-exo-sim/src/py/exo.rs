//! Flat exo_* bindings (exo_version, class data, admission, HMD/HAVEN runs, EEW and tsunami timing, scenarios). Owner: X6 (surface frozen by W0a/W0-C1 in python/fabrail/_native.pyi).
//!
//! Every function is flat on `fabrail._native` (no submodule) and prefixed `exo_`.
//! Closed forms delegate to the Rust owners (fabrail-exo, fabrail-exo-safety, this
//! crate). `exo_simulate` runs the XS/XC scenarios (`crate::scenarios`) and returns
//! `fabrail.exo_run/1` JSON; `exo_scenario_info` describes a scenario (classes, inputs
//! with defaults and ranges, criteria). Wave 2 (X6) added, additively: the
//! `inputs_json`/`channels` keywords of `exo_simulate`, `exo_scenario_info`,
//! `exo_admission_table`, `exo_hts_area_table`, `exo_hts_force`, `exo_tuning`,
//! `exo_eew_warning`, `exo_das_velocity`, `exo_tsunami_arrival`, `exo_tidegate_plan` and
//! the `ctmc_ed2` key of `exo_pfh_2oo3`. Errors: `ValueError("path: reason")` for bad
//! input, `NotImplementedError` for stubs.

use std::collections::BTreeMap;

use fabrail_exo::closure::{self, ClosureDecision, ClosureInputs};
use fabrail_exo::{energy, tuning};
use fabrail_exo_safety::pfh::{self, PfhInputs};
use fabrail_exo_safety::presets::{self, ExoPreset};
use fabrail_exo_safety::zone::iso13855_separation_mm;
use fabrail_exo_safety::{ExoClass, Ke2Uj, MassKg, SafetyRegime, VelMmps, ZoneCfg};
use pyo3::exceptions::{PyNotImplementedError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;

use crate::scenarios::{ExoScenarioId, ExoSimError, ScenarioOptions, run_scenario_with};
use crate::suspension::{HtsBulk, SuspensionModel};
use crate::{eew_timing, platform, storage, suspension, tsunami};
use fabrail_exo::tuning::TuningForm;
use fabrail_types::fmath;

/// Maps an exotic simulation error onto a Python exception.
pub(crate) fn to_py_err(e: ExoSimError) -> PyErr {
    match e {
        ExoSimError::NotImplemented(_) => PyNotImplementedError::new_err(e.to_string()),
        _ => PyValueError::new_err(e.to_string()),
    }
}

/// fabrail-exo-sim version string.
#[pyfunction]
fn exo_version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

/// Class slugs in `ExoClass` order.
#[pyfunction]
fn exo_classes() -> Vec<&'static str> {
    ExoClass::ALL.iter().map(|c| c.slug()).collect()
}

fn regime_dict<'py>(py: Python<'py>, p: &ExoPreset) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    d.set_item("kind", p.regime.label())?;
    match p.regime {
        SafetyRegime::Pedestrian => {}
        SafetyRegime::GuardedZone(z) => {
            d.set_item("presence_pl_e", z.presence_pl_e)?;
            d.set_item("pps", z.pps)?;
            d.set_item("two_person_override", z.two_person_override)?;
            d.set_item("teleoperated", z.teleoperated)?;
            d.set_item("k_mmps", z.k_mmps)?;
            d.set_item("t1_ms", z.t1_ms)?;
            d.set_item("c_mm", z.c_mm)?;
            d.set_item("a_brake_mmps2", z.a_brake_mmps2)?;
            d.set_item("jog_v_mmps", z.jog_v_mmps)?;
            d.set_item("edge_k_n_per_mm", z.edge_k_n_per_mm)?;
            d.set_item(
                "iso13855_s_mm",
                iso13855_separation_mm(&z, p.kinematics.v_max_mmps),
            )?;
        }
        SafetyRegime::OccupiedPlatform(o) => {
            d.set_item("a_peak_max_mmps2", o.a_peak_max_mmps2)?;
            d.set_item("jerk_max_mmps3", o.jerk_max_mmps3)?;
            d.set_item("landing_v_max_mmps", o.landing_v_max_mmps)?;
            d.set_item("moat_presence", o.moat_presence)?;
            d.set_item("moat_covers", o.moat_covers)?;
        }
    }
    Ok(d)
}

/// The EXOTIC §1 class table from code (drift-guarded against the doc), one dict per
/// class: class, display, design_mass_kg, mass_min_kg, mass_max_kg, suspension, pole,
/// regime (dict), safe_state, seismic_action, seismic_role, rest_policy, travel_mm,
/// v_max_mmps, a_max_mmps2, j_max_mmps3, period_ms, badge, badge_symbol, trl_lo,
/// trl_hi, row (the verbatim table cells).
#[pyfunction]
fn exo_class_table(py: Python<'_>) -> PyResult<Vec<Bound<'_, PyDict>>> {
    presets::PRESETS
        .iter()
        .map(|p| {
            let d = PyDict::new(py);
            d.set_item("class", p.class.slug())?;
            d.set_item("display", p.class.display())?;
            d.set_item("design_mass_kg", p.design_mass.0)?;
            d.set_item("mass_min_kg", p.mass_min.0)?;
            d.set_item("mass_max_kg", p.mass_max.0)?;
            d.set_item("suspension", p.suspension.name())?;
            d.set_item("pole", p.pole.name())?;
            d.set_item("regime", regime_dict(py, p)?)?;
            d.set_item("safe_state", p.safe_state.name())?;
            d.set_item("seismic_action", p.safe_state.seismic_action().name())?;
            d.set_item("seismic_role", p.seismic_role.name())?;
            d.set_item("rest_policy", p.rest_policy.name())?;
            d.set_item("travel_mm", p.kinematics.travel_mm)?;
            d.set_item("v_max_mmps", p.kinematics.v_max_mmps)?;
            d.set_item("a_max_mmps2", p.kinematics.a_max_mmps2)?;
            d.set_item("j_max_mmps3", p.kinematics.j_max_mmps3)?;
            d.set_item("period_ms", p.kinematics.period_ms)?;
            d.set_item("badge", p.badge.name())?;
            d.set_item("badge_symbol", p.badge.symbol())?;
            d.set_item("trl_lo", p.trl.lo)?;
            d.set_item("trl_hi", p.trl.hi)?;
            let row = PyDict::new(py);
            row.set_item("class", p.row.class)?;
            row.set_item("design_mass", p.row.design_mass)?;
            row.set_item("suspension", p.row.suspension)?;
            row.set_item("regime", p.row.regime)?;
            row.set_item("maturity", p.row.maturity)?;
            row.set_item("effort", p.row.effort)?;
            d.set_item("row", row)?;
            Ok(d)
        })
        .collect()
}

/// Den Hartog optimum (f = ω_d/ω_s, ζ normalised to ω_s).
#[pyfunction]
fn exo_den_hartog(mu: f64) -> (f64, f64) {
    let t = tuning::den_hartog(mu);
    (t.f_ratio, t.zeta)
}

/// Warburton optimum for base-acceleration (seismic) white noise.
#[pyfunction]
fn exo_warburton_base(mu: f64) -> (f64, f64) {
    let t = tuning::warburton_base(mu);
    (t.f_ratio, t.zeta)
}

/// Warburton optimum for force (wind) white noise.
#[pyfunction]
fn exo_warburton_force(mu: f64) -> (f64, f64) {
    let t = tuning::warburton_force(mu);
    (t.f_ratio, t.zeta)
}

/// Stroke bound on added damping ζ_add ≤ (μ/2)·(S/X).
#[pyfunction]
fn exo_zeta_add_bound(mu: f64, stroke_ratio: f64) -> f64 {
    tuning::zeta_add_bound(mu, stroke_ratio)
}

/// Magnetic pressure B²/(2μ0), Pa.
#[pyfunction]
fn exo_magnetic_pressure(b_t: f64) -> f64 {
    suspension::magnetic_pressure_pa(b_t)
}

/// Pole-area admission A = k·m·g/(η_fill·min(B_pole, B_rail, B_gap,max)²/2μ0), m².
#[pyfunction]
#[pyo3(signature = (m_kg, eta_fill, b_pole_t, b_rail_t, b_gap_max_t, k = suspension::ADMISSION_K))]
fn exo_admission_area(
    m_kg: f64,
    eta_fill: f64,
    b_pole_t: f64,
    b_rail_t: f64,
    b_gap_max_t: f64,
    k: f64,
) -> f64 {
    suspension::admission_area_m2(m_kg, k, eta_fill, b_pole_t, b_rail_t, b_gap_max_t)
}

/// Curved-guideway period T = 2π·√(R/g), s.
#[pyfunction]
fn exo_guideway_period(r_m: f64) -> f64 {
    platform::guideway_period_s(r_m)
}

/// Curved-guideway radius R = g·T²/(4π²), m.
#[pyfunction]
fn exo_guideway_radius(period_s: f64) -> f64 {
    platform::guideway_radius_m(period_s)
}

/// Pole sagitta L²/(8R), m.
#[pyfunction]
fn exo_sagitta(l_m: f64, r_m: f64) -> f64 {
    platform::sagitta_m(l_m, r_m)
}

/// ISO 13855 separation S = K·(t1 + v/a_brake) + C, m (computed in the integer
/// exo-safety units: mm, ms).
#[pyfunction]
#[pyo3(signature = (v_mps, a_brake_mps2, k_mps = 1.6, t1_s = 0.1, c_m = 0.85))]
fn exo_iso13855_distance(
    v_mps: f64,
    a_brake_mps2: f64,
    k_mps: f64,
    t1_s: f64,
    c_m: f64,
) -> PyResult<f64> {
    let vals = [v_mps, a_brake_mps2, k_mps, t1_s, c_m];
    if vals
        .iter()
        .any(|x| !(x.is_finite() && *x >= 0.0 && *x < 1.0e6))
    {
        return Err(PyValueError::new_err(
            "iso13855: inputs must be finite, non-negative and < 1e6",
        ));
    }
    let cfg = ZoneCfg {
        k_mmps: (k_mps * 1000.0).round() as u32,
        t1_ms: (t1_s * 1000.0).round() as u32,
        c_mm: (c_m * 1000.0).round() as u32,
        a_brake_mmps2: (a_brake_mps2 * 1000.0).round() as u32,
        ..ZoneCfg::default()
    };
    let s = iso13855_separation_mm(&cfg, (v_mps * 1000.0).round() as u32);
    Ok(if s == u64::MAX {
        f64::INFINITY
    } else {
        s as f64 / 1000.0
    })
}

/// 2·KE in integer µJ for a mass in kg and a velocity in mm/s (None on overflow).
#[pyfunction]
fn exo_ke2_uj(m_kg: u32, v_mmps: i32) -> Option<u64> {
    Ke2Uj::of(MassKg(m_kg), VelMmps(v_mmps)).map(|k| k.0)
}

/// 2oo3 PFH in both forms plus the CTMC cross-checks, 1/h: keys conservative, ed2,
/// ctmc (conservative-form chain), ctmc_ed2 (Ed. 2 chain), t_ce_h.
#[pyfunction]
fn exo_pfh_2oo3(
    lambda_d_per_h: f64,
    dc: f64,
    beta: f64,
    beta_d: f64,
    t1_h: f64,
    mttr_h: f64,
    mrt_h: f64,
) -> BTreeMap<String, f64> {
    let p = PfhInputs {
        lambda_d_per_h,
        dc,
        beta,
        beta_d,
        t1_h,
        mttr_h,
        mrt_h,
    };
    [
        ("conservative", pfh::pfh_2oo3_conservative(&p)),
        ("ed2", pfh::pfh_2oo3_ed2(&p)),
        ("ctmc", pfh::pfh_2oo3_ctmc(&p)),
        ("ctmc_ed2", pfh::pfh_2oo3_ctmc_ed2(&p)),
        ("t_ce_h", pfh::t_ce_h(&p)),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v))
    .collect()
}

/// Flywheel usable fraction 1 − (ω_min/ω_max)².
#[pyfunction]
fn exo_flywheel_usable_fraction(omega_min_over_max: f64) -> f64 {
    storage::flywheel_usable_fraction(omega_min_over_max)
}

/// Ride-through window T_ride = max(300 s, shaking) + landing, s.
#[pyfunction]
fn exo_t_ride(t_shaking_s: f64, t_land_s: f64) -> f64 {
    energy::t_ride_s(t_shaking_s, t_land_s)
}

/// Ride-through energy P_hold·T_ride, J.
#[pyfunction]
fn exo_ride_through_energy(p_hold_w: f64, t_ride_s: f64) -> f64 {
    energy::ride_through_energy_j(p_hold_w, t_ride_s)
}

/// Lead-time gain per km of sensor advance 1/v, s/km.
#[pyfunction]
fn exo_lead_time_gain(v_kmps: f64) -> f64 {
    eew_timing::lead_time_gain_s_per_km(v_kmps)
}

/// Blind-zone radius √((V_s·t_alert)² − h²), km.
#[pyfunction]
fn exo_blind_zone_km(v_s_kmps: f64, t_alert_s: f64, depth_km: f64) -> f64 {
    eew_timing::blind_zone_km(v_s_kmps, t_alert_s, depth_km)
}

/// Long-wave speed √(g·h), m/s.
#[pyfunction]
fn exo_tsunami_speed(depth_m: f64) -> f64 {
    tsunami::wave_speed_mps(depth_m)
}

/// Green's-law amplitude ratio (h0/h1)^(1/4).
#[pyfunction]
fn exo_greens_law(h0_m: f64, h1_m: f64) -> f64 {
    tsunami::greens_law_factor(h0_m, h1_m)
}

/// Closure-race decision: {"close": bool, "t_close_s": float | None, "reason": str | None}.
#[pyfunction]
#[pyo3(signature = (d_m, v0_mps, a_mps2, j_mps3, v_max_mps, t_seal_s, margin_s, t_a_s, t_detect_s, zone_clear, two_person_override = false))]
#[allow(clippy::too_many_arguments)]
fn exo_closure_decision<'py>(
    py: Python<'py>,
    d_m: f32,
    v0_mps: f32,
    a_mps2: f32,
    j_mps3: f32,
    v_max_mps: f32,
    t_seal_s: f32,
    margin_s: f32,
    t_a_s: f32,
    t_detect_s: f32,
    zone_clear: bool,
    two_person_override: bool,
) -> PyResult<Bound<'py, PyDict>> {
    let inp = ClosureInputs {
        d_m,
        v0_mps,
        a_mps2,
        j_mps3,
        v_max_mps,
        t_seal_s,
        margin_s,
        t_a_s,
        t_detect_s,
        zone_clear,
        two_person_override,
    };
    let d = PyDict::new(py);
    match closure::plan(&inp) {
        ClosureDecision::Close { t_close_s } => {
            d.set_item("close", true)?;
            d.set_item("t_close_s", t_close_s)?;
            d.set_item("reason", py.None())?;
        }
        ClosureDecision::Fallback(r) => {
            d.set_item("close", false)?;
            d.set_item("t_close_s", py.None())?;
            d.set_item("reason", format!("{r:?}"))?;
        }
    }
    Ok(d)
}

/// The exotic scenario catalogue: (name, pass criteria).
#[pyfunction]
fn exo_scenarios() -> Vec<(&'static str, &'static str)> {
    ExoScenarioId::ALL
        .iter()
        .map(|s| (s.name(), s.criteria()))
        .collect()
}

/// Runs scenario `scenario` for class `class_name`; returns the result JSON
/// (`fabrail.exo_run/1`: scenario, class, model, reduced_model, passed, seed,
/// criteria_text, requirements, criteria [{name, passed, value, limit, detail}], kpis,
/// inputs, labels, notes, rate_hz, and `channels` {name: {unit, data}} when `channels`).
/// `inputs_json` is a JSON object of input overrides (see `exo_scenario_info`). Errors:
/// `ValueError("class: …")`, `ValueError("scenario: …")`, `ValueError("inputs.<name>: …")`.
/// Releases the GIL.
#[pyfunction]
#[pyo3(signature = (class_name, scenario, seed = 42, inputs_json = None, channels = false))]
fn exo_simulate(
    py: Python<'_>,
    class_name: &str,
    scenario: &str,
    seed: u64,
    inputs_json: Option<&str>,
    channels: bool,
) -> PyResult<String> {
    if ExoClass::from_slug(class_name).is_none() {
        return Err(PyValueError::new_err(format!(
            "class: unknown exotic class \"{class_name}\""
        )));
    }
    let Some(id) = ExoScenarioId::from_name(scenario) else {
        return Err(PyValueError::new_err(format!(
            "scenario: unknown exotic scenario \"{scenario}\""
        )));
    };
    let inputs = parse_inputs(inputs_json)?;
    let class = class_name.to_string();
    let opts = ScenarioOptions {
        seed,
        inputs,
        record: channels,
    };
    let r = py
        .detach(move || run_scenario_with(id, &class, &opts))
        .map_err(to_py_err)?;
    Ok(r.to_json(channels).to_string())
}

/// Parses `inputs_json` (a JSON object of numbers) into input overrides.
fn parse_inputs(inputs_json: Option<&str>) -> PyResult<BTreeMap<String, f64>> {
    let Some(txt) = inputs_json else {
        return Ok(BTreeMap::new());
    };
    let v: serde_json::Value = serde_json::from_str(txt)
        .map_err(|e| PyValueError::new_err(format!("inputs_json: {e}")))?;
    let Some(obj) = v.as_object() else {
        return Err(PyValueError::new_err(
            "inputs_json: expected a JSON object of numbers",
        ));
    };
    obj.iter()
        .map(|(k, x)| {
            x.as_f64()
                .map(|f| (k.clone(), f))
                .ok_or_else(|| PyValueError::new_err(format!("inputs.{k}: not a number")))
        })
        .collect()
}

/// Describes scenario `scenario` as JSON: name, input (catalogue "Input" text), criteria,
/// requirements, classes (slugs) and inputs [{name, default, min, max, unit, doc}].
#[pyfunction]
fn exo_scenario_info(scenario: &str) -> PyResult<String> {
    let Some(id) = ExoScenarioId::from_name(scenario) else {
        return Err(PyValueError::new_err(format!(
            "scenario: unknown exotic scenario \"{scenario}\""
        )));
    };
    let inputs: Vec<serde_json::Value> = id
        .input_specs()
        .iter()
        .map(|s| {
            serde_json::json!({
                "name": s.name, "default": s.default, "min": s.min, "max": s.max,
                "unit": s.unit, "doc": s.doc,
            })
        })
        .collect();
    let classes: Vec<&str> = id.classes().iter().map(|c| c.slug()).collect();
    Ok(serde_json::json!({
        "name": id.name(),
        "input": id.input_label(),
        "criteria": id.criteria(),
        "requirements": id.requirements(),
        "classes": classes,
        "inputs": inputs,
    })
    .to_string())
}

/// The per-class pole-area admission table (FAB-EXO-103, the EXOTIC `EXO_ADMISSION`
/// block): one dict per class with class, suspension, m_kg, b_eff_t, p_eff_pa, area_m2.
#[pyfunction]
#[pyo3(signature = (eta_fill = suspension::ETA_FILL_DEFAULT, b_rail_t = suspension::B_RAIL_STEEL_T, b_gap_max_t = suspension::B_GAP_MAX_DEFAULT_T))]
fn exo_admission_table(
    py: Python<'_>,
    eta_fill: f64,
    b_rail_t: f64,
    b_gap_max_t: f64,
) -> PyResult<Vec<Bound<'_, PyDict>>> {
    suspension::admission_table(eta_fill, b_rail_t, b_gap_max_t)
        .iter()
        .map(|r| {
            let d = PyDict::new(py);
            d.set_item("class", r.class.slug())?;
            d.set_item("suspension", r.suspension.name())?;
            d.set_item("m_kg", r.m_kg)?;
            d.set_item("b_eff_t", r.b_eff_t)?;
            d.set_item("p_eff_pa", r.p_eff_pa)?;
            d.set_item("area_m2", r.area_m2)?;
            Ok(d)
        })
        .collect()
}

/// The HTS area table (FAB-EXO-121, the EXOTIC `EXO_HTS_AREA` block): one dict per class
/// with class, m_kg, area_m2, cryostats, heat_w, ln2_kg_per_day, ems_area_m2.
#[pyfunction]
fn exo_hts_area_table(py: Python<'_>) -> PyResult<Vec<Bound<'_, PyDict>>> {
    suspension::hts_area_table()
        .iter()
        .map(|r| {
            let d = PyDict::new(py);
            d.set_item("class", r.class.slug())?;
            d.set_item("m_kg", r.m_kg)?;
            d.set_item("area_m2", r.area_m2)?;
            d.set_item("cryostats", r.cryostats)?;
            d.set_item("heat_w", r.heat_w)?;
            d.set_item("ln2_kg_per_day", r.ln2_kg_per_day)?;
            d.set_item("ems_area_m2", r.ems_area_m2)?;
            Ok(d)
        })
        .collect()
}

/// HTS lift of the ATZ-class demonstrator (4 cryostats) on its first-loading branch at
/// height `z_m` (field cooled at 35 mm and loaded down to `z_m`): {"z_m", "fz_n",
/// "pressure_pa", "kz_n_per_m", "ky_n_per_m", "first_loading_n"} (EXOTIC §X3; research
/// tier). `ValueError` outside (0, 35 mm].
#[pyfunction]
fn exo_hts_force(py: Python<'_>, z_m: f64) -> PyResult<Bound<'_, PyDict>> {
    let b = HtsBulk::atz_demonstrator();
    if !(z_m.is_finite() && z_m > 0.0 && z_m <= b.z_fc_m) {
        return Err(PyValueError::new_err(format!(
            "z_m: {z_m} outside (0, {}] m",
            b.z_fc_m
        )));
    }
    let mut h = b.field_cool(0.0);
    let n = fmath::ceil((b.z_fc_m - z_m) / 1.0e-5).max(1.0) as usize;
    for i in 1..=n {
        let z = b.z_fc_m - (b.z_fc_m - z_m) * i as f64 / n as f64;
        b.step_hysteresis(
            &suspension::GapState {
                z_m: z,
                ..suspension::GapState::default()
            },
            &mut h,
            0.0,
        );
    }
    let s = suspension::GapState {
        z_m,
        ..suspension::GapState::default()
    };
    let f = b.force(&s, &h);
    let d = PyDict::new(py);
    d.set_item("z_m", z_m)?;
    d.set_item("fz_n", f.fz_n)?;
    d.set_item("pressure_pa", b.pressure_pa(&s, &h))?;
    d.set_item("kz_n_per_m", f.kz_n_per_m)?;
    d.set_item("ky_n_per_m", f.ky_n_per_m)?;
    d.set_item("first_loading_n", b.first_loading_force_n(z_m))?;
    Ok(d)
}

/// Absorber tuning of `form` ("den_hartog", "warburton_base", "warburton_force") at mass
/// ratio `mu`: {"form", "f_ratio", "zeta_published", "zeta_basis", "zeta_s"} and, with
/// `m_s_kg` and `omega_s_rad_s`, the damper design {"m_d_kg", "omega_d_rad_s",
/// "k_d_n_per_m", "c_d_n_s_per_m"} with ζ normalised to ω_s (plan §4).
#[pyfunction]
#[pyo3(signature = (mu, form = "den_hartog", m_s_kg = None, omega_s_rad_s = None))]
fn exo_tuning<'py>(
    py: Python<'py>,
    mu: f64,
    form: &str,
    m_s_kg: Option<f64>,
    omega_s_rad_s: Option<f64>,
) -> PyResult<Bound<'py, PyDict>> {
    let Some(f) = TuningForm::ALL.iter().copied().find(|t| t.slug() == form) else {
        return Err(PyValueError::new_err(format!(
            "form: unknown tuning form \"{form}\" (den_hartog, warburton_base, warburton_force)"
        )));
    };
    if !(mu.is_finite() && mu > 0.0 && mu < 1.0) {
        return Err(PyValueError::new_err("mu: must be in (0, 1)"));
    }
    let t = f.published(mu);
    let d = PyDict::new(py);
    d.set_item("form", f.slug())?;
    d.set_item("f_ratio", t.f_ratio)?;
    d.set_item("zeta_published", t.zeta)?;
    d.set_item(
        "zeta_basis",
        match f.zeta_basis() {
            fabrail_exo::tuning::ZetaBasis::OmegaS => "omega_s",
            fabrail_exo::tuning::ZetaBasis::OmegaD => "omega_d",
        },
    )?;
    d.set_item("zeta_s", f.omega_s(mu).zeta)?;
    if let (Some(m), Some(w)) = (m_s_kg, omega_s_rad_s) {
        if !(m.is_finite() && m > 0.0 && w.is_finite() && w > 0.0) {
            return Err(PyValueError::new_err(
                "m_s_kg, omega_s_rad_s: must be finite and positive",
            ));
        }
        let dd = fabrail_exo::tuning::damper_design_for(m, w, mu, f);
        d.set_item("m_d_kg", dd.m_d_kg)?;
        d.set_item("omega_d_rad_s", dd.omega_d_rad_s)?;
        d.set_item("k_d_n_per_m", dd.k_d_n_per_m)?;
        d.set_item("c_d_n_s_per_m", dd.c_d_n_s_per_m)?;
    }
    Ok(d)
}

/// EEW warning time with its inputs printed (EXOTIC §X10): epicentral distances of the
/// site and of the reporting (k-th) sensor, depth, wave speeds, latency total (default
/// split) and actuation time. Returns every input plus r_site_hyp_km, r_sensor_hyp_km,
/// t_alert_s, t_s_site_s, warning_s, blind_zone_km, gain_p_s_per_km, gain_s_s_per_km.
#[pyfunction]
#[pyo3(signature = (r_site_km, r_sensor_km, depth_km = 10.0, v_p_kmps = eew_timing::V_P_KMPS, v_s_kmps = eew_timing::V_S_KMPS, latency_s = eew_timing::DEFAULT_LATENCY_S, t_act_s = 0.0))]
#[allow(clippy::too_many_arguments)]
fn exo_eew_warning(
    r_site_km: f64,
    r_sensor_km: f64,
    depth_km: f64,
    v_p_kmps: f64,
    v_s_kmps: f64,
    latency_s: f64,
    t_act_s: f64,
) -> PyResult<BTreeMap<String, f64>> {
    let vals = [r_site_km, r_sensor_km, depth_km, latency_s, t_act_s];
    if vals.iter().any(|x| !(x.is_finite() && *x >= 0.0))
        || !(v_p_kmps.is_finite() && v_p_kmps > 0.0 && v_s_kmps.is_finite() && v_s_kmps > 0.0)
    {
        return Err(PyValueError::new_err(
            "eew: distances, depth and times must be finite and ≥ 0; speeds > 0",
        ));
    }
    let lat = eew_timing::EewLatency::scaled_to(latency_s);
    let r_site = eew_timing::hypocentral_km(r_site_km, depth_km);
    let r_sensor = eew_timing::hypocentral_km(r_sensor_km, depth_km);
    let t_alert = eew_timing::t_alert_s(r_sensor, v_p_kmps, &lat);
    Ok([
        ("r_site_km", r_site_km),
        ("r_sensor_km", r_sensor_km),
        ("depth_km", depth_km),
        ("v_p_kmps", v_p_kmps),
        ("v_s_kmps", v_s_kmps),
        ("latency_s", lat.total_s()),
        ("t_act_s", t_act_s),
        ("r_site_hyp_km", r_site),
        ("r_sensor_hyp_km", r_sensor),
        ("t_alert_s", t_alert),
        ("t_s_site_s", r_site / v_s_kmps),
        (
            "warning_s",
            eew_timing::warning_s(r_site, v_s_kmps, t_alert, t_act_s),
        ),
        (
            "blind_zone_km",
            eew_timing::blind_zone_km(v_s_kmps, t_alert, depth_km),
        ),
        (
            "gain_p_s_per_km",
            eew_timing::lead_time_gain_s_per_km(v_p_kmps),
        ),
        (
            "gain_s_s_per_km",
            eew_timing::lead_time_gain_s_per_km(v_s_kmps),
        ),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v))
    .collect())
}

/// DAS particle velocity from axial strain, u̇ = −V_app·ε, m/s (exotic design §2.8).
#[pyfunction]
fn exo_das_velocity(strain: f64, v_app_mps: f64) -> f64 {
    eew_timing::das_particle_velocity_mps(v_app_mps, strain)
}

/// Tsunami arrival ∫dx/√(g·h) over `profile` [(distance m, depth m), …] (None: the
/// reference source-to-coast path), refined to `step_m` panels, s (EXOTIC §X12).
#[pyfunction]
#[pyo3(signature = (profile = None, step_m = 100.0))]
fn exo_tsunami_arrival(profile: Option<Vec<(f64, f64)>>, step_m: f64) -> PyResult<f64> {
    let prof = profile.unwrap_or_else(tsunami::reference_profile);
    if prof.len() < 2
        || prof
            .iter()
            .any(|(x, h)| !(x.is_finite() && h.is_finite() && *h > 0.0))
    {
        return Err(PyValueError::new_err(
            "profile: at least two (distance, depth > 0) points",
        ));
    }
    if !(step_m.is_finite() && step_m > 0.0) {
        return Err(PyValueError::new_err("step_m: must be > 0"));
    }
    Ok(tsunami::arrival_time_s(&tsunami::refine_profile(
        &prof, step_m,
    )))
}

/// TIDEGATE closure plan (EXOTIC §X12): the gap check after rail distortion picks maglev
/// or wheels; the close starts after the shaking and only on the external command.
/// Returns {"mode", "gap_error_m", "gap_tolerance_m", "gap_pass", "t_start_s",
/// "t_move_s", "t_closed_s", "budget_s", "within_budget"}.
#[pyfunction]
#[pyo3(signature = (t_arrival_s, t_shaking_s, t_latency_s, command = true, subsidence_m = 0.0, differential_frac = 0.05, tilt_rad = 0.0))]
#[allow(clippy::too_many_arguments)]
fn exo_tidegate_plan<'py>(
    py: Python<'py>,
    t_arrival_s: f64,
    t_shaking_s: f64,
    t_latency_s: f64,
    command: bool,
    subsidence_m: f64,
    differential_frac: f64,
    tilt_rad: f64,
) -> PyResult<Bound<'py, PyDict>> {
    let vals = [
        t_arrival_s,
        t_shaking_s,
        t_latency_s,
        subsidence_m,
        differential_frac,
    ];
    if vals.iter().any(|x| !(x.is_finite() && *x >= 0.0)) || !tilt_rad.is_finite() {
        return Err(PyValueError::new_err(
            "tidegate: times, subsidence and differential must be finite and ≥ 0",
        ));
    }
    let mut inp = tsunami::TidegateInputs::tidegate(t_arrival_s, t_shaking_s, t_latency_s);
    inp.distortion = tsunami::RailDistortion {
        subsidence_m,
        differential_frac,
        step_frac: 0.5,
        tilt_rad,
    };
    if !command {
        inp.t_command_s = None;
    }
    let p = tsunami::plan_tidegate_close(&inp);
    let d = PyDict::new(py);
    d.set_item("mode", p.mode.name())?;
    d.set_item("gap_error_m", p.gap.max_gap_error_m)?;
    d.set_item("gap_tolerance_m", p.gap.tolerance_m)?;
    d.set_item("gap_pass", p.gap.pass)?;
    d.set_item("t_start_s", p.t_start_s)?;
    d.set_item("t_move_s", p.t_move_s)?;
    d.set_item("t_closed_s", p.t_closed_s)?;
    d.set_item("budget_s", p.budget_s)?;
    d.set_item("within_budget", p.within_budget)?;
    Ok(d)
}

/// Adds this module's functions to `fabrail._native`.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(exo_version, m)?)?;
    m.add_function(wrap_pyfunction!(exo_classes, m)?)?;
    m.add_function(wrap_pyfunction!(exo_class_table, m)?)?;
    m.add_function(wrap_pyfunction!(exo_den_hartog, m)?)?;
    m.add_function(wrap_pyfunction!(exo_warburton_base, m)?)?;
    m.add_function(wrap_pyfunction!(exo_warburton_force, m)?)?;
    m.add_function(wrap_pyfunction!(exo_zeta_add_bound, m)?)?;
    m.add_function(wrap_pyfunction!(exo_magnetic_pressure, m)?)?;
    m.add_function(wrap_pyfunction!(exo_admission_area, m)?)?;
    m.add_function(wrap_pyfunction!(exo_guideway_period, m)?)?;
    m.add_function(wrap_pyfunction!(exo_guideway_radius, m)?)?;
    m.add_function(wrap_pyfunction!(exo_sagitta, m)?)?;
    m.add_function(wrap_pyfunction!(exo_iso13855_distance, m)?)?;
    m.add_function(wrap_pyfunction!(exo_ke2_uj, m)?)?;
    m.add_function(wrap_pyfunction!(exo_pfh_2oo3, m)?)?;
    m.add_function(wrap_pyfunction!(exo_flywheel_usable_fraction, m)?)?;
    m.add_function(wrap_pyfunction!(exo_t_ride, m)?)?;
    m.add_function(wrap_pyfunction!(exo_ride_through_energy, m)?)?;
    m.add_function(wrap_pyfunction!(exo_lead_time_gain, m)?)?;
    m.add_function(wrap_pyfunction!(exo_blind_zone_km, m)?)?;
    m.add_function(wrap_pyfunction!(exo_tsunami_speed, m)?)?;
    m.add_function(wrap_pyfunction!(exo_greens_law, m)?)?;
    m.add_function(wrap_pyfunction!(exo_closure_decision, m)?)?;
    m.add_function(wrap_pyfunction!(exo_scenarios, m)?)?;
    m.add_function(wrap_pyfunction!(exo_simulate, m)?)?;
    m.add_function(wrap_pyfunction!(exo_scenario_info, m)?)?;
    m.add_function(wrap_pyfunction!(exo_admission_table, m)?)?;
    m.add_function(wrap_pyfunction!(exo_hts_area_table, m)?)?;
    m.add_function(wrap_pyfunction!(exo_hts_force, m)?)?;
    m.add_function(wrap_pyfunction!(exo_tuning, m)?)?;
    m.add_function(wrap_pyfunction!(exo_eew_warning, m)?)?;
    m.add_function(wrap_pyfunction!(exo_das_velocity, m)?)?;
    m.add_function(wrap_pyfunction!(exo_tsunami_arrival, m)?)?;
    m.add_function(wrap_pyfunction!(exo_tidegate_plan, m)?)?;
    Ok(())
}
