//! Analytic functions wrapping fabrail_params::derive (f64 closed forms). Owner: A8 (surface frozen by W0a/W0-C1 in python/fabrail/_native.pyi).
//!
//! Every function is a thin wrapper of the single-source closed form in
//! `fabrail_params::derive` (each citing its PHYSICS §). Signatures follow the frozen
//! Rust functions (W0-A report): `bias_current`/`stop_ratio` take the parameter set,
//! `zp_gains` takes a mode, `passivity_k_max` takes the loop delay (default one period),
//! `pd_wu_zhao` returns metres. Default arguments come from `fabrail_params::presets`
//! (pedestrian caps), never restated literals.

use std::collections::BTreeMap;

use fabrail_params::derive::{self, FailDir, LevMode};
use fabrail_params::presets::{PEDESTRIAN_E_MAX_J, PEDESTRIAN_P_MAX_KGMPS};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::py_params::PyParams;

/// Open-loop heave pole √(4g₀/(2g* + h′)), rad/s.
#[pyfunction]
fn open_loop_pole(g_star_m: f64, h_prime_m: f64) -> f64 {
    derive::open_loop_pole(g_star_m, h_prime_m)
}

/// Zero-power mass at gap g, kg.
#[pyfunction]
fn zero_power_mass(gap_m: f64, g_star_m: f64, h_prime_m: f64, m_star_kg: f64) -> f64 {
    derive::zero_power_mass(gap_m, g_star_m, h_prime_m, m_star_kg)
}

/// Zero-power gap at mass m, m.
#[pyfunction]
fn zero_power_gap(m_kg: f64, g_star_m: f64, h_prime_m: f64, m_star_kg: f64) -> f64 {
    derive::zero_power_gap(m_kg, g_star_m, h_prime_m, m_star_kg)
}

/// Bias current holding mass m at gap g, A.
#[pyfunction]
fn bias_current(params: PyParams, gap_m: f64, m_kg: f64) -> f64 {
    derive::bias_current(&params.inner.design, gap_m, m_kg)
}

/// Stop ratio F_pm(g)/(m·g₀).
#[pyfunction]
fn stop_ratio(params: PyParams, gap_m: f64, m_kg: f64) -> f64 {
    derive::stop_ratio(&params.inner.design, gap_m, m_kg)
}

fn lev_mode(mode: &str) -> PyResult<LevMode> {
    match mode {
        "heave" => Ok(LevMode::Heave),
        "pitch" => Ok(LevMode::Pitch),
        "roll" => Ok(LevMode::Roll),
        other => Err(PyValueError::new_err(format!(
            "mode: unknown mode \"{other}\" (heave, pitch, roll)"
        ))),
    }
}

/// Zero-power gains per magnet of one mode: kp_a_per_m, kd_a_s_per_m, kappa_per_s
/// (the discrete design the controller uses, or the continuous reference).
#[pyfunction]
#[pyo3(signature = (params, mode = "heave", discrete = true))]
fn zp_gains(params: PyParams, mode: &str, discrete: bool) -> PyResult<BTreeMap<String, f64>> {
    let m = lev_mode(mode)?;
    let g = if discrete {
        derive::zp_gains_discrete(&params.inner, m)
    } else {
        derive::zp_gains(&params.inner, m)
    };
    Ok([
        ("kp_a_per_m", g.kp_a_per_m),
        ("kd_a_s_per_m", g.kd_a_s_per_m),
        ("kappa_per_s", g.kappa_per_s),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v))
    .collect())
}

/// Force-noise gain per element, N/m.
#[pyfunction]
fn noise_force_gain(params: PyParams) -> f64 {
    derive::noise_force_gain(&params.inner)
}

/// Low-energy speed cap √(2E/m), m/s (E defaults to the pedestrian cap).
#[pyfunction]
#[pyo3(signature = (m_kg, e_j = PEDESTRIAN_E_MAX_J))]
fn v_low_energy(m_kg: f64, e_j: f64) -> f64 {
    derive::v_low_energy(m_kg, e_j)
}

/// Momentum speed cap p/m, m/s (p defaults to the pedestrian cap).
#[pyfunction]
#[pyo3(signature = (m_kg, p = PEDESTRIAN_P_MAX_KGMPS))]
fn momentum_speed_cap(m_kg: f64, p: f64) -> f64 {
    derive::momentum_cap(m_kg, p)
}

/// Jerk-limited stop distance, m.
#[pyfunction]
fn stop_distance(v: f64, a: f64, j: f64) -> f64 {
    derive::stop_distance(v, a, j)
}

/// Clamp force √(2E·k), N.
#[pyfunction]
fn clamp_force(e_j: f64, k_n_per_m: f64) -> f64 {
    derive::clamp_force(e_j, k_n_per_m)
}

/// Springs in series, N/m.
#[pyfunction]
fn series_stiffness(k1: f64, k2: f64) -> f64 {
    derive::series_k(k1, k2)
}

/// ISO/TS 15066 transient-contact speed, m/s.
#[pyfunction]
fn iso15066_speed(f_n: f64, m_h_kg: f64, m_kg: f64, k: f64) -> f64 {
    derive::iso15066_speed(f_n, m_h_kg, m_kg, k)
}

/// Passive admittance stiffness bound 2b/(T + 2T_d), N/m (T_d defaults to T).
#[pyfunction]
#[pyo3(signature = (b, t_s, t_d_s = None))]
fn passivity_k_max(b: f64, t_s: f64, t_d_s: Option<f64>) -> f64 {
    derive::passivity_k_max(b, t_s, t_d_s.unwrap_or(t_s))
}

/// Control-loss impact speed, m/s (`direction` = "up" | "down").
#[pyfunction]
fn impact_speed(params: PyParams, direction: &str) -> PyResult<f64> {
    let dir = match direction {
        "up" => FailDir::Up,
        "down" => FailDir::Down,
        other => {
            return Err(PyValueError::new_err(format!(
                "direction: unknown direction \"{other}\" (up, down)"
            )));
        }
    };
    Ok(derive::impact_speed(&params.inner, dir))
}

/// Thermal gap shift for a temperature rise d_t (K), m.
#[pyfunction]
fn thermal_gap_shift(params: PyParams, d_t: f64) -> f64 {
    derive::thermal_gap_shift(&params.inner, d_t)
}

/// Manual force on the landing rollers, N (PM-relieved or the no-PM bound).
#[pyfunction]
#[pyo3(signature = (params, relieved = true))]
fn manual_force(params: PyParams, relieved: bool) -> f64 {
    derive::manual_force(&params.inner, relieved)
}

/// Cavity piston force (orifice C_d included), N.
#[pyfunction]
fn piston_force(a_p: f64, v: f64, a_leak: f64) -> f64 {
    derive::piston_force(a_p, v, a_leak)
}

/// Den Hartog optimum (f, ζ).
#[pyfunction]
fn den_hartog(mu: f64) -> (f64, f64) {
    derive::den_hartog(mu)
}

/// Den Hartog equal-peak amplification √(1 + 2/μ).
#[pyfunction]
fn den_hartog_peak(mu: f64) -> f64 {
    derive::den_hartog_peak(mu)
}

/// Warburton optimum, base-acceleration form (f, ζ).
#[pyfunction]
fn warburton(mu: f64) -> (f64, f64) {
    derive::warburton(mu)
}

/// Warburton optimum, force (wind) form (f, ζ).
#[pyfunction]
fn warburton_force(mu: f64) -> (f64, f64) {
    derive::warburton_force(mu)
}

/// Passive transmissibility.
#[pyfunction]
fn transmissibility(r: f64, zeta: f64) -> f64 {
    derive::transmissibility(r, zeta)
}

/// Skyhook transmissibility.
#[pyfunction]
fn skyhook_transmissibility(r: f64, zeta: f64) -> f64 {
    derive::t_sky(r, zeta)
}

/// Minimum isolation period 2π·√(m·s/F_cont), s.
#[pyfunction]
fn t_min_isolation(m: f64, stroke: f64, f: f64) -> f64 {
    derive::t_min_iso(m, stroke, f)
}

/// Wu–Zhao peak displacement, m.
#[pyfunction]
fn pd_wu_zhao(magnitude: f64, r_km: f64) -> f64 {
    derive::pd_wu_zhao(magnitude, r_km)
}

/// S−P time, s.
#[pyfunction]
#[pyo3(signature = (r_hyp_km, vp = 6.0, vs = 3.5))]
fn sp_time(r_hyp_km: f64, vp: f64, vs: f64) -> f64 {
    derive::sp_time(r_hyp_km, vp, vs)
}

/// Warning (lead) time r/v_s − t_alert, s.
#[pyfunction]
fn lead_time(r_hyp_km: f64, v_s_kmps: f64, t_alert_s: f64) -> f64 {
    derive::lead_time(r_hyp_km, v_s_kmps, t_alert_s)
}

/// Network/on-site crossover distance v_p·(t_network − t_onsite), km.
#[pyfunction]
#[pyo3(signature = (v_p_kmps = 6.0, t_onsite_s = derive::ONSITE_DECISION_S, t_network_s = derive::NETWORK_ALERT_S))]
fn lead_crossover_km(v_p_kmps: f64, t_onsite_s: f64, t_network_s: f64) -> f64 {
    derive::lead_crossover_km(v_p_kmps, t_onsite_s, t_network_s)
}

/// Terminus blend (t_b_s, d_b_m, d_0_m).
#[pyfunction]
fn terminus_blend(v: f64, j: f64, tau: f64) -> BTreeMap<String, f64> {
    let b = derive::terminus_blend(v, j, tau);
    [("t_b_s", b.t_b_s), ("d_b_m", b.d_b_m), ("d_0_m", b.d_0_m)]
        .into_iter()
        .map(|(k, v)| (k.to_string(), v))
        .collect()
}

/// Terminus arrival time, s (d_ε defaults to 0.2 mm, plan §4b).
#[pyfunction]
#[pyo3(signature = (v, j, tau, d_eps_m = 2.0e-4))]
fn terminus_t_arrive(v: f64, j: f64, tau: f64, d_eps_m: f64) -> f64 {
    derive::terminus_t_arrive(v, j, tau, d_eps_m)
}

/// Minimum terminus time constant v/a − a/(2j), s.
#[pyfunction]
fn tau_min(v_app: f64, a_max: f64, j: f64) -> f64 {
    derive::tau_min(v_app, a_max, j)
}

/// Unattended close speed min(v_cap, F/√(m_gov·k_series)), m/s.
#[pyfunction]
fn unattended_close_speed(v_cap: f64, f_pinch: f64, m_gov: f64, k_series: f64) -> f64 {
    derive::unattended_close_speed(v_cap, f_pinch, m_gov, k_series)
}

/// Bumper energy rating ½·m·v², J.
#[pyfunction]
fn bumper_rating(m_kg: f64, v_mps: f64) -> f64 {
    derive::bumper_rating(m_kg, v_mps)
}

/// Adds this module's functions to `fabrail._native`.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(open_loop_pole, m)?)?;
    m.add_function(wrap_pyfunction!(zero_power_mass, m)?)?;
    m.add_function(wrap_pyfunction!(zero_power_gap, m)?)?;
    m.add_function(wrap_pyfunction!(bias_current, m)?)?;
    m.add_function(wrap_pyfunction!(stop_ratio, m)?)?;
    m.add_function(wrap_pyfunction!(zp_gains, m)?)?;
    m.add_function(wrap_pyfunction!(noise_force_gain, m)?)?;
    m.add_function(wrap_pyfunction!(v_low_energy, m)?)?;
    m.add_function(wrap_pyfunction!(momentum_speed_cap, m)?)?;
    m.add_function(wrap_pyfunction!(stop_distance, m)?)?;
    m.add_function(wrap_pyfunction!(clamp_force, m)?)?;
    m.add_function(wrap_pyfunction!(series_stiffness, m)?)?;
    m.add_function(wrap_pyfunction!(iso15066_speed, m)?)?;
    m.add_function(wrap_pyfunction!(passivity_k_max, m)?)?;
    m.add_function(wrap_pyfunction!(impact_speed, m)?)?;
    m.add_function(wrap_pyfunction!(thermal_gap_shift, m)?)?;
    m.add_function(wrap_pyfunction!(manual_force, m)?)?;
    m.add_function(wrap_pyfunction!(piston_force, m)?)?;
    m.add_function(wrap_pyfunction!(den_hartog, m)?)?;
    m.add_function(wrap_pyfunction!(den_hartog_peak, m)?)?;
    m.add_function(wrap_pyfunction!(warburton, m)?)?;
    m.add_function(wrap_pyfunction!(warburton_force, m)?)?;
    m.add_function(wrap_pyfunction!(transmissibility, m)?)?;
    m.add_function(wrap_pyfunction!(skyhook_transmissibility, m)?)?;
    m.add_function(wrap_pyfunction!(t_min_isolation, m)?)?;
    m.add_function(wrap_pyfunction!(pd_wu_zhao, m)?)?;
    m.add_function(wrap_pyfunction!(sp_time, m)?)?;
    m.add_function(wrap_pyfunction!(lead_time, m)?)?;
    m.add_function(wrap_pyfunction!(lead_crossover_km, m)?)?;
    m.add_function(wrap_pyfunction!(terminus_blend, m)?)?;
    m.add_function(wrap_pyfunction!(terminus_t_arrive, m)?)?;
    m.add_function(wrap_pyfunction!(tau_min, m)?)?;
    m.add_function(wrap_pyfunction!(unattended_close_speed, m)?)?;
    m.add_function(wrap_pyfunction!(bumper_rating, m)?)?;
    Ok(())
}
