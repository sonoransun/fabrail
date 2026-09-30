"""Closed forms from ``fabrail_params::derive`` (f64), documented with units and PHYSICS § (core §B.7).

Each function forwards to the same-named function of ``fabrail._native``; the Rust
code in ``crates/fabrail-params/src/derive.rs`` is the single implementation, and the
anchors of SPECIFICATION §2 and core design §E.5 are asserted against it
(``tests/test_physics_anchors.py``). Nothing here re-implements a formula or restates
a parameter value: optional arguments left as None use the native default (the
pedestrian caps live in ``fabrail_params::presets``, the V_p/V_s and alert latencies in
``fabrail_params::derive``).

Units are SI throughout (m, s, kg, N, A, rad/s); names carry unit suffixes where the
native signature does. PHYSICS § numbers refer to docs/PHYSICS.md.

Owner: W0-C2 (thin wrappers over the core §B.7 names), then A8 (Wave 1: aligned with
``_native.pyi`` and completed to the whole native analytic surface).
"""

from __future__ import annotations

from types import ModuleType
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ._native import Params

__all__ = [
    "bias_current",
    "bumper_rating",
    "clamp_force",
    "den_hartog",
    "den_hartog_peak",
    "impact_speed",
    "iso15066_speed",
    "lead_crossover_km",
    "lead_time",
    "manual_force",
    "momentum_speed_cap",
    "noise_force_gain",
    "open_loop_pole",
    "passivity_k_max",
    "pd_wu_zhao",
    "piston_force",
    "series_stiffness",
    "skyhook_transmissibility",
    "sp_time",
    "stop_distance",
    "stop_ratio",
    "t_min_isolation",
    "tau_min",
    "terminus_blend",
    "terminus_t_arrive",
    "thermal_gap_shift",
    "transmissibility",
    "unattended_close_speed",
    "v_low_energy",
    "warburton",
    "warburton_force",
    "zero_power_gap",
    "zero_power_mass",
    "zp_gains",
]


def _n() -> ModuleType:
    from . import require_native

    return require_native()


def _opt(**kwargs: object) -> dict[str, object]:
    """Keyword arguments that are not None (None = use the native default)."""
    return {k: v for k, v in kwargs.items() if v is not None}


# ---- levitation (PHYSICS §3, §12, §15) --------------------------------------------


def open_loop_pole(g_star_m: float, h_prime_m: float) -> float:
    """Open-loop (Earnshaw) heave pole p = √(4·g₀/(2g* + h′)), rad/s.

    ``g_star_m`` is the zero-power gap and ``h_prime_m`` = h_m/μ_rec the magnet's
    reluctance length, both m. PHYSICS §3; FAB-LEV-003 (69.49 / 47.57 / 32.50 rad/s).
    """
    return float(_n().open_loop_pole(g_star_m, h_prime_m))


def zero_power_mass(gap_m: float, g_star_m: float, h_prime_m: float, m_star_kg: float) -> float:
    """Mass held at zero coil current at ``gap_m``: m(g) = m*·((2g* + h′)/(2g + h′))², kg.

    PHYSICS §3; FAB-LEV-005 (DRAWER 1.5 → 45.76, 5.0 → 9.96, 6.2 → 7.12 kg).
    """
    return float(_n().zero_power_mass(gap_m, g_star_m, h_prime_m, m_star_kg))


def zero_power_gap(m_kg: float, g_star_m: float, h_prime_m: float, m_star_kg: float) -> float:
    """Zero-current gap for mass ``m_kg``: g = ((2g* + h′)·√(m*/m) − h′)/2, m.

    The inverse of :func:`zero_power_mass`. PHYSICS §3; FAB-LEV-004 (DOOR +98.1 N →
    4.5967 mm).
    """
    return float(_n().zero_power_gap(m_kg, g_star_m, h_prime_m, m_star_kg))


def bias_current(params: Params, gap_m: float, m_kg: float) -> float:
    """Coil current that holds ``m_kg`` at ``gap_m`` with every lift element equal, A.

    i = ((2g + h′)·√(m·g₀/(n·μ₀·A_p)) − M_pm)/N. PHYSICS §3, §15; F17 (DRAWER seat
    3.5 mm: +11.20 A at 48 kg, −4.99 A at 8 kg).
    """
    return float(_n().bias_current(params, gap_m, m_kg))


def stop_ratio(params: Params, gap_m: float, m_kg: float) -> float:
    """Magnet-only force ratio n·F_pm(g)/(m·g₀) (dimensionless).

    At g_up with m* it is the upper-stop ratio (≤ 1.30 for DOOR/WALL/HEAVY); at g_land
    with m_op_min it is the stay-landed ratio (≤ 0.90). PHYSICS §3; FAB-LEV-006, F18.
    """
    return float(_n().stop_ratio(params, gap_m, m_kg))


def zp_gains(params: Params, mode: str = "heave", discrete: bool = True) -> dict[str, float]:
    """Zero-power gains per magnet of one mode ("heave" | "pitch" | "roll").

    Keys ``kp_a_per_m`` (A/m), ``kd_a_s_per_m`` (A·s/m), ``kappa_per_s`` (1/s).
    ``discrete`` selects the discrete design on the ZOH plant with the LET delay and
    the current and eddy lags, which the controller uses; False gives the continuous
    pole-placement reference. Passive or inactive modes return zeros. PHYSICS §15;
    plan §2b item 1; FAB-LEV-007.
    """
    return {str(k): float(v) for k, v in _n().zp_gains(params, mode, discrete).items()}


def noise_force_gain(params: Params) -> float:
    """Gap-sensor noise to force gain per element K = m_eff·(a₁ + p²), N/m.

    a₁ = (1 + 2ζα)·ω² is the s¹ coefficient of the heave target polynomial with
    ω = omega_ratio·p. Equals k_i·k_p of the continuous heave gains. PHYSICS §13, §15;
    FAB-HAP-011 (DRAWER ≈ 0.31 N/µm at ω = 2.5·p).
    """
    return float(_n().noise_force_gain(params))


def thermal_gap_shift(params: Params, d_t_k: float) -> float:
    """Zero-power gap shift for a magnet temperature change ``d_t_k`` (K), m.

    Δg = (2g* + h′)·α_Br·ΔT/2 through the remanence tempco. PHYSICS §12; FAB-LEV-014
    (+40 K: −0.195 / −0.416 / −0.891 mm).
    """
    return float(_n().thermal_gap_shift(params, d_t_k))


def impact_speed(params: Params, direction: str) -> float:
    """Stop impact speed after a control loss at (g*, m*), ``direction`` "up" or "down", m/s.

    Energy balance of the PM force alone (coil current 0, no lag, no dissipation).
    PHYSICS §15; FAB-LEV-019 (DOOR 50.6 / 55.6 mm/s).
    """
    return float(_n().impact_speed(params, direction))


# ---- motion, clamps and haptics (PHYSICS §16, §16.1, §17, §18) ------------------------


def v_low_energy(m_kg: float, e_j: float | None = None) -> float:
    """Low-energy speed cap √(2E/m), m/s.

    ``e_j`` None uses the pedestrian 1.69 J cap (``fabrail_params::presets``).
    PHYSICS §16; FAB-SAF-001/002.
    """
    return float(_n().v_low_energy(m_kg, **_opt(e_j=e_j)))


def momentum_speed_cap(m_kg: float, p: float | None = None) -> float:
    """Momentum speed cap p/m, m/s.

    ``p`` (kg·m/s) None uses the pedestrian 22.3 kg·m/s cap (``fabrail_params::presets``).
    PHYSICS §16; FAB-SAF-002 (WALL 0.0446 m/s).
    """
    return float(_n().momentum_speed_cap(m_kg, **_opt(p=p)))


def stop_distance(v: float, a: float, j: float) -> float:
    """Jerk-limited stop distance from speed ``v`` (m/s) at ``a`` (m/s²) and ``j`` (m/s³), m.

    v²/(2a) + v·a/(2j) when v ≥ a²/j, else v·√(v/j). PHYSICS §16; FAB-PRP-005
    (33.43 / 36.31 / 8.86 / 4.91 mm at the caps).
    """
    return float(_n().stop_distance(v, a, j))


def clamp_force(e_j: float, k_n_per_m: float) -> float:
    """Quasi-static clamp force √(2E·k) of kinetic energy ``e_j`` (J) into a body spring, N.

    Equivalently v·√(m·k). PHYSICS §18; FAB-SAF-005 (1.69 J: chest 290.7 N, hand
    503.5 N, through a 10 N/mm edge 173 N).
    """
    return float(_n().clamp_force(e_j, k_n_per_m))


def series_stiffness(k1: float, k2: float) -> float:
    """Springs in series k₁·k₂/(k₁ + k₂), N/m. PHYSICS §18."""
    return float(_n().series_stiffness(k1, k2))


def iso15066_speed(f_n: float, m_h_kg: float, m_kg: float, k: float) -> float:
    """Transient-contact speed v = F/√(μ·k) with the reduced mass μ = m_h·m/(m_h + m), m/s.

    ``f_n`` is the permissible force (N), ``m_h_kg`` the body-region effective mass,
    ``m_kg`` the moving mass, ``k`` the body-region stiffness (N/m). ISO/TS 15066:2016
    Annex A form; PHYSICS §18, §22 (500 kg against the chest at 140 / 280 N:
    0.145 / 0.291 m/s).
    """
    return float(_n().iso15066_speed(f_n, m_h_kg, m_kg, k))


def passivity_k_max(b: float, t_s: float, t_d_s: float | None = None) -> float:
    """Passive virtual-stiffness bound 2b/(T + 2·T_d), N/m.

    ``b`` is the physical damping (N·s/m), ``t_s`` the sample period and ``t_d_s`` the
    loop delay (None = one period), s. Colgate–Schenkel; PHYSICS §17; plan §2b
    item 19; FAB-HAP-009.
    """
    return float(_n().passivity_k_max(b, t_s, t_d_s))


def manual_force(params: Params, relieved: bool = True) -> float:
    """Manual push force on the landing rollers at the rated mass, N.

    ``relieved``: μ·(W − n·F_pm(g_land)) with the magnets relieving the rollers;
    otherwise the no-PM bound μ·W. PHYSICS §8, §18; F8; FAB-SAF-010.
    """
    return float(_n().manual_force(params, relieved))


def terminus_blend(v: float, j: float, tau: float) -> dict[str, float]:
    """Constant-jerk blend from cruise speed ``v`` (m/s) into the log-damped terminus.

    Keys ``t_b_s`` = √(τ² + 2v/j) − τ, ``d_b_m`` = j·t_b·τ² (distance-to-go at
    capture) and ``d_0_m`` = d_b + v·t_b − j·t_b³/6 (distance-to-go at blend start).
    PHYSICS §16.1; plan §4b (DRAWER τ 0.15 s: 49.8 / 32.1 mm).
    """
    return {str(k): float(val) for k, val in _n().terminus_blend(v, j, tau).items()}


def terminus_t_arrive(v: float, j: float, tau: float, d_eps_m: float | None = None) -> float:
    """Terminus arrival time t_b + τ·ln(d_b/d_ε) + T_q with T_q = τ, s.

    ``d_eps_m`` is the quintic hand-over distance (None = 0.2 mm, the native default).
    PHYSICS §16.1; FAB-HAP-016 (DRAWER 0.98 s before the +0.2 s seat step).
    """
    return float(_n().terminus_t_arrive(v, j, tau, **_opt(d_eps_m=d_eps_m)))


def tau_min(v_app: float, a_max: float, j: float) -> float:
    """Smallest admissible terminus time constant v_app/a_max − a_max/(2j), s.

    Enforced by ``Params.validate``; at run time τ_eff = max(τ, τ_min(v_measured)).
    PHYSICS §16.1; plan §4b.
    """
    return float(_n().tau_min(v_app, a_max, j))


def unattended_close_speed(v_cap: float, f_pinch: float, m_gov: float, k_series: float) -> float:
    """Unattended automatic-close speed min(v_cap, F_pinch/√(m_gov·k_series)), m/s.

    Plan §4b: 0.23 m/s at 48 kg with 150 N through hand 75 N/mm ⊕ edge 10 N/mm.
    PHYSICS §18.
    """
    return float(_n().unattended_close_speed(v_cap, f_pinch, m_gov, k_series))


def bumper_rating(m_kg: float, v_mps: float) -> float:
    """Bumper energy rating ½·m·v², J. PHYSICS §8; FAB-SAF-014 (≥ 24 / 113 / 563 / 5625 J)."""
    return float(_n().bumper_rating(m_kg, v_mps))


# ---- environment (PHYSICS §11) ---------------------------------------------------------


def piston_force(a_p: float, v: float, a_leak: float) -> float:
    """Cavity piston force through a leak orifice, N.

    ΔP = ½ρ(v·A_p/(C_d·A_leak))² with C_d = 0.6, F = ΔP·A_p, for front area ``a_p``
    (m²), speed ``v`` (m/s) and leak area ``a_leak`` (m²); a sealed cavity
    (``a_leak`` ≤ 0) returns 0. PHYSICS §11; FAB-ENV-003 (DRAWER 0.06 m², 0.5 m/s,
    40 cm² → 5.6 N).
    """
    return float(_n().piston_force(a_p, v, a_leak))


# ---- seismic response, dampers and isolation (PHYSICS §21) --------------------------------


def den_hartog(mu: float) -> tuple[float, float]:
    """Den Hartog optimum (frequency ratio f, damping ratio ζ) for mass ratio ``mu``.

    f = 1/(1 + μ), ζ = √(3μ/(8(1 + μ)³)). PHYSICS §21; μ = 0.02: 0.98039 / 0.08407.
    """
    f, z = _n().den_hartog(mu)
    return float(f), float(z)


def den_hartog_peak(mu: float) -> float:
    """Den Hartog equal-peak amplification √(1 + 2/μ). PHYSICS §21 (μ = 0.02: 10.05)."""
    return float(_n().den_hartog_peak(mu))


def warburton(mu: float) -> tuple[float, float]:
    """Warburton optimum (f, ζ), base-acceleration (seismic) form. PHYSICS §21 (μ = 0.02: 0.97548 / 0.07019)."""
    f, z = _n().warburton(mu)
    return float(f), float(z)


def warburton_force(mu: float) -> tuple[float, float]:
    """Warburton optimum (f, ζ), force (wind) form. PHYSICS §21; plan §4 anchors (μ = 0.01: f 0.99257)."""
    f, z = _n().warburton_force(mu)
    return float(f), float(z)


def transmissibility(r: float, zeta: float) -> float:
    """Passive base-excitation transmissibility √((1 + (2ζr)²)/((1 − r²)² + (2ζr)²)).

    ``r`` = f/f_n. PHYSICS §21; T(3, 0.2) = 0.1931.
    """
    return float(_n().transmissibility(r, zeta))


def skyhook_transmissibility(r: float, zeta: float) -> float:
    """Skyhook transmissibility 1/√((1 − r²)² + (2ζr)²). PHYSICS §16.2, §21."""
    return float(_n().skyhook_transmissibility(r, zeta))


def t_min_isolation(m: float, stroke: float, f: float) -> float:
    """Minimum isolation period 2π·√(m·s/F_cont) for mass ``m`` (kg), stroke (m), force ``f`` (N), s.

    PHYSICS §21; plan §2b item 5; FAB-SAF-020 (WALL 6.3 s with F_cont).
    """
    return float(_n().t_min_isolation(m, stroke, f))


# ---- detection and early warning (PHYSICS §20) ----------------------------------------


def pd_wu_zhao(magnitude: float, r_km: float) -> float:
    """Peak P-wave displacement, m (the regression is in cm).

    log₁₀ Pd[cm] = −3.463 + 0.729·M − 1.374·log₁₀ R[km] (Wu and Zhao 2006).
    PHYSICS §20; FAB-SEI-012 (M7 at 20 km: 0.7118 cm).
    """
    return float(_n().pd_wu_zhao(magnitude, r_km))


def sp_time(r_hyp_km: float, vp: float | None = None, vs: float | None = None) -> float:
    """S−P time R·(1/V_s − 1/V_p), s (``vp``/``vs`` in km/s; None uses the native 6.0 / 3.5).

    PHYSICS §20; FAB-SEI-012 (0.1190 s/km).
    """
    return float(_n().sp_time(r_hyp_km, **_opt(vp=vp, vs=vs)))


def lead_time(r_hyp_km: float, v_s_kmps: float, t_alert_s: float) -> float:
    """Warning time before the S arrival R/V_s − t_alert, s (both times from the origin).

    PHYSICS §20. Negative inside the blind zone.
    """
    return float(_n().lead_time(r_hyp_km, v_s_kmps, t_alert_s))


def lead_crossover_km(
    v_p_kmps: float | None = None, t_onsite_s: float | None = None, t_network_s: float | None = None
) -> float:
    """Distance where on-site and network alerts arrive together: V_p·(t_network − t_onsite), km.

    None uses the native defaults (6.0 km/s, 1.2 s on-site decision, 8 s network
    latency). PHYSICS §20; FAB-SEI-012 (40.8 km).
    """
    return float(_n().lead_crossover_km(**_opt(v_p_kmps=v_p_kmps, t_onsite_s=t_onsite_s, t_network_s=t_network_s)))
