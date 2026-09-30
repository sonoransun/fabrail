"""EEW timing, sensor advance and fusion demos (EXOTIC §X10; FAB-EXO-801..805, FAB-EXO-810).

fabrail is not an earthquake early-warning provider: live feeds need ShakeAlert LtO or
JMA licensing and are roadmap, every alert here is synthetic or injected, and every
timing output prints its inputs (FAB-EXO-803).

Timing (``fabrail_exo_sim::eew_timing``, through ``_native.exo_eew_warning``)::

    t_alert = R_k/V_p + τ_win + t_pkt + t_tel + t_proc + t_diss
    W       = R_site/V_s − t_alert − t_act
    ∂W/∂x   = 1/V_p = 0.15385 s/km (V_p 6.5)   or   1/V_s = 0.27027 s/km (PLUM, V_s 3.7)
    r_b     = √((V_s·t_alert)² − h²)

The fusion itself (maximum predicted intensity with an S-ETA distribution, capped
log-odds over the local detectors, cost–loss decision) is ``fabrail_exo::fusion`` and is
exercised through the XS08 (cancel, concurrent-quake false alarm) and XS09 (offshore
M8) scenarios. :func:`intensity_exceedance`, :func:`capped_log_odds` and
:func:`act_by_cost_loss` restate the three decision steps of EXOTIC §X10 as algebra for
demos and plots; they are illustrations of the rule, not the implementation.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

from ._util import native

__all__ = [
    "OFFSHORE_CASE",
    "Warning",
    "act_by_cost_loss",
    "blind_zone_km",
    "capped_log_odds",
    "das_velocity",
    "fusion_demo",
    "gains",
    "intensity_exceedance",
    "lead_time_gain",
    "offshore_worked_case",
    "sensor_advance",
    "warning",
]


def lead_time_gain(v_kmps: float) -> float:
    """∂W/∂x = 1/v, s per km of sensor advance (``eew_timing``; FAB-EXO-803)."""
    return float(native().exo_lead_time_gain(float(v_kmps)))


def gains(v_p_kmps: float = 6.5, v_s_kmps: float = 3.7) -> dict[str, float]:
    """The two lead-time gains with their inputs: P-based 0.15385 s/km, PLUM-type 0.27027 s/km."""
    return {"gain_p_s_per_km": lead_time_gain(v_p_kmps), "gain_plum_s_per_km": lead_time_gain(v_s_kmps), "v_p_kmps": float(v_p_kmps), "v_s_kmps": float(v_s_kmps)}


def blind_zone_km(t_alert_s: float, depth_km: float, v_s_kmps: float = 3.7) -> float:
    """Blind-zone epicentral radius r_b = √((V_s·t_alert)² − h²), km (0 when V_s·t_alert ≤ h)."""
    return float(native().exo_blind_zone_km(float(v_s_kmps), float(t_alert_s), float(depth_km)))


def das_velocity(strain: float, v_app_mps: float) -> float:
    """DAS particle velocity from strain, u̇ = −V_app·ε, m/s (EXOTIC §X10)."""
    return float(native().exo_das_velocity(float(strain), float(v_app_mps)))


@dataclass(frozen=True)
class Warning:
    """One warning-time case with every input (``eew_timing``)."""

    label: str
    r_site_km: float
    r_sensor_km: float
    depth_km: float
    t_alert_s: float
    t_s_site_s: float
    warning_s: float
    blind_zone_km: float
    inputs: dict[str, float] = field(default_factory=dict)


def warning(
    r_site_km: float,
    r_sensor_km: float,
    depth_km: float = 10.0,
    *,
    v_p_kmps: float = 6.5,
    v_s_kmps: float = 3.7,
    latency_s: float = 3.3,
    t_act_s: float = 0.0,
    label: str = "",
) -> Warning:
    """Warning time at a site from a sensor ``r_sensor_km`` from the epicentre (EXOTIC §X10).

    ``latency_s`` is τ_win + t_pkt + t_tel + t_proc + t_diss (default 1 + 1 + 0.5 + 0.5 +
    0.3 = 3.3 s [UNCERTAIN: typical-order values]). A negative warning means the S wave
    arrives before the alert (the site is in the blind zone).
    """
    r = native().exo_eew_warning(float(r_site_km), float(r_sensor_km), float(depth_km), float(v_p_kmps), float(v_s_kmps), float(latency_s), float(t_act_s))
    inputs = {k: float(r[k]) for k in ("r_site_km", "r_sensor_km", "depth_km", "v_p_kmps", "v_s_kmps", "latency_s", "t_act_s")}
    return Warning(
        label=label,
        r_site_km=float(r["r_site_km"]),
        r_sensor_km=float(r["r_sensor_km"]),
        depth_km=float(r["depth_km"]),
        t_alert_s=float(r["t_alert_s"]),
        t_s_site_s=float(r["t_s_site_s"]),
        warning_s=float(r["warning_s"]),
        blind_zone_km=float(r["blind_zone_km"]),
        inputs=inputs,
    )


def sensor_advance(r_site_km: float = 60.0, depth_km: float = 10.0, advances_km: tuple[float, ...] = (0.0, 10.0, 20.0, 30.0, 40.0, 50.0), latency_s: float = 3.3) -> list[Warning]:
    """Warning at a fixed site as the sensor moves toward the epicentre (EXOTIC §X10).

    The sensor starts at the site (advance 0) and moves by ``advances_km``; each
    kilometre buys 1/V_p = 0.154 s until the fixed latency terms take over.
    """
    out = []
    for dx in advances_km:
        r_sensor = max(0.0, r_site_km - dx)
        out.append(warning(r_site_km, r_sensor, depth_km, latency_s=latency_s, label=f"advance {dx:g} km"))
    return out


#: The EXOTIC §X10 worked case: epicentre 100 km offshore at 20 km depth, city 20 km
#: inland (S at ≈ 32.9 s), fourth-nearest station, 3.3 s latency.
OFFSHORE_CASE: dict[str, float] = {"r_site_km": 120.0, "depth_km": 20.0, "latency_s": 3.3, "r_land_km": 110.0, "r_snet_km": 34.0, "r_das_km": 8.0}


def offshore_worked_case(case: dict[str, float] | None = None) -> list[Warning]:
    """Coastal land network, S-net-like seafloor stations and DAS over the source (EXOTIC §X10 table).

    With the default inputs: alert 20.5 / 9.4 / 6.6 s, warning at the city 12.4 / 23.5 / 26.3 s.
    """
    c = {**OFFSHORE_CASE, **(case or {})}
    return [
        warning(c["r_site_km"], c[key], c["depth_km"], latency_s=c["latency_s"], label=label)
        for label, key in (("coastal land network", "r_land_km"), ("S-net-like seafloor", "r_snet_km"), ("DAS over the source", "r_das_km"))
    ]


# ---- fusion decision steps as algebra (illustrations of EXOTIC §X10; the Rust fusion decides) ----


def _phi(x: float) -> float:
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


def intensity_exceedance(i_max: float, i_crit: float = 6.0, sigma_i: float = 0.7) -> float:
    """Primary estimate: P(I ≥ I_crit) = Φ((I_max − I_crit)/σ_I) (EXOTIC §X10 step 1) [UNCERTAIN: I_crit, σ_I]."""
    if sigma_i <= 0:
        raise ValueError("sigma_i must be > 0")
    return _phi((i_max - i_crit) / sigma_i)


def capped_log_odds(p0: float, likelihood_ratios: dict[str, float], weights: dict[str, float] | None = None, cap: float = 3.0) -> float:
    """Secondary: logit P = logit P0 + clamp(Σ w_i·ln LR_i, ±cap) (EXOTIC §X10 step 2) [UNCERTAIN: w, cap].

    The cap stops correlated local evidence from compounding.
    """
    if not 0.0 < p0 < 1.0:
        raise ValueError("p0 must lie in (0, 1)")
    w = weights or {}
    s = sum(w.get(k, 1.0) * math.log(v) for k, v in likelihood_ratios.items())
    s = max(-cap, min(cap, s))
    z = math.log(p0 / (1.0 - p0)) + s
    return 1.0 / (1.0 + math.exp(-z))


def act_by_cost_loss(p: float, c_over_l: float) -> bool:
    """Decision: act when P > C/L (EXOTIC §X10 step 3)."""
    return p > c_over_l


def fusion_demo(seed: int = 42) -> dict[str, object]:
    """Run the fusion scenarios (XS08 cancel/false alarm for every class; XS09 offshore M8) and summarise.

    Returns ``{xs08: {class: {passed, kpis}}, xs09: {class: {passed, kpis, inputs}}}``;
    every alert in these runs is synthetic.
    """
    n = native()
    out: dict[str, object] = {"note": "Every alert is synthetic/injected; fabrail is not an EEW provider."}
    for scen in ("XS08", "XS09"):
        info = json.loads(n.exo_scenario_info(scen))
        per: dict[str, object] = {}
        for cls in info["classes"]:
            r = json.loads(n.exo_simulate(cls, scen, seed))
            per[cls] = {"passed": bool(r["passed"]), "kpis": r["kpis"], "inputs": r["inputs"], "criteria": r["criteria"]}
        out[scen.lower()] = per
    return out
