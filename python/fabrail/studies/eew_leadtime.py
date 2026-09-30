"""Study ``eew_leadtime`` (tier 1): usable warning vs distance for on-site, network and fused alerts.

Question: how much warning does each alert path give before the S-wave arrives, where
do the on-site and network paths cross over, and how wide is the blind zone?

Model (PHYSICS §20; SEISMIC.md §2.1; FAB-SEI-012), all times from the origin and R the
**hypocentral** distance in km:

- S arrival t_S = R/V_s; P arrival t_P = R/V_p.
- On site: the structure accelerometer picks P and needs the pick plus the short Pd/τc
  window, ``t_onsite`` = 1.2 s in total, before it decides: lead_onsite = t_S − (t_P + t_onsite)
  = R·(1/V_s − 1/V_p) − t_onsite.
- Network: a regional alert a fixed ``t_network`` = 8 s after origin (the first
  ShakeAlert alert for Ridgecrest 2019 came 6.9 s after origin; SEISMIC.md §2.1):
  lead_network = R/V_s − t_network.
- Fused: the seismic FSM acts on whichever arrives first, so lead_fused = max(on-site,
  network). The crossover is R_c = V_p·(t_network − t_onsite) = 40.8 km at the defaults
  (``analytic.lead_crossover_km``).
- Bands: V_p 5.5–6.5 km/s and V_s 3.2–3.7 km/s (physics perspective §2.11 sensitivity;
  core design §C.4). Each lead is monotonic in each speed, so the band is the envelope of
  the four corners.

Every alert is synthetic or injected: fabrail is not an EEW provider, and it claims no
useful on-site warning below about 20 km (plan §7). The closed forms are cross-checked
against the native ``lead_time``/``sp_time``/``lead_crossover_km`` at the SEISMIC.md table
distances (``native_check``).

Owner: B3 (Wave 2).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from .. import pwave
from .runner import StudyResult, StudySpec

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..config import RunConfig

__all__ = [
    "NETWORK_ALERT_S",
    "ONSITE_DECISION_S",
    "R_TABLE_KM",
    "STUDY",
    "V_P_BAND_KMPS",
    "V_S_BAND_KMPS",
    "compute",
    "leads",
    "run",
]

#: On-site decision time after the P pick, s: the pick plus the 1 s Pd/τc window and
#: release latency (the native ``lead_crossover_km`` default; SEISMIC.md §2.1).
ONSITE_DECISION_S = 1.2
#: Network alert latency after origin, s (the native default; SEISMIC.md §2.1).
NETWORK_ALERT_S = 8.0
#: Speed sensitivity bands, km/s (physics perspective §2.11: "V_p 6.0 and V_s 3.5 km/s
#: (sensitivity 5.5–6.5 and 3.2–3.7)").
V_P_BAND_KMPS = (5.5, 6.5)
V_S_BAND_KMPS = (3.2, 3.7)
#: The SEISMIC.md §2.1 table distances, km (40.8 is the crossover).
R_TABLE_KM = (10.0, 15.0, 20.0, 25.0, 30.0, 40.8, 60.0, 100.0, 150.0, 200.0)
#: Hypocentral distance grid, km.
_R_GRID_KM = np.round(np.arange(1.0, 250.0 + 1e-9, 1.0), 6)
#: Focal depths for the epicentral panel, km.
_DEPTHS_KM = (5.0, 10.0, 20.0)
#: Epicentral distance grid, km.
_EPI_GRID_KM = np.round(np.arange(0.0, 120.0 + 1e-9, 1.0), 6)


def leads(
    r_hyp_km: np.ndarray | float,
    v_p_kmps: float = pwave.V_P_KMPS,
    v_s_kmps: float = pwave.V_S_KMPS,
    t_onsite_s: float = ONSITE_DECISION_S,
    t_network_s: float = NETWORK_ALERT_S,
) -> dict[str, np.ndarray]:
    """Lead times before the S arrival, s, for on-site, network and fused alerts.

    ``r_hyp_km`` is the hypocentral distance (km). Negative leads mean the alert comes
    after the S-wave (blind zone). PHYSICS §20; SEISMIC.md §2.1.
    """
    r = np.asarray(r_hyp_km, dtype=np.float64)
    if np.any(r < 0.0) or not (v_p_kmps > v_s_kmps > 0.0):
        raise ValueError("leads: need r >= 0 and v_p > v_s > 0")
    onsite = r / v_s_kmps - (r / v_p_kmps + t_onsite_s)
    network = r / v_s_kmps - t_network_s
    return {"onsite_s": onsite, "network_s": network, "fused_s": np.maximum(onsite, network)}


def _band(r: np.ndarray, t_onsite_s: float, t_network_s: float) -> dict[str, np.ndarray]:
    """Envelope of each lead over the V_p/V_s sensitivity box (monotonic in each speed)."""
    corners = [leads(r, vp, vs, t_onsite_s, t_network_s) for vp in V_P_BAND_KMPS for vs in V_S_BAND_KMPS]
    out: dict[str, np.ndarray] = {}
    for key in ("onsite_s", "network_s", "fused_s"):
        stack = np.vstack([c[key] for c in corners])
        out[key + "_lo"] = stack.min(axis=0)
        out[key + "_hi"] = stack.max(axis=0)
    return out


def _native_check() -> dict[str, Any]:
    """Max |closed form − native| over the table distances (``_native`` analytic functions)."""
    from .. import analytic

    worst_lead = 0.0
    worst_sp = 0.0
    for r in R_TABLE_KM:
        lv = leads(r)
        on_native = analytic.lead_time(r, pwave.V_S_KMPS, r / pwave.V_P_KMPS + ONSITE_DECISION_S)
        net_native = analytic.lead_time(r, pwave.V_S_KMPS, NETWORK_ALERT_S)
        worst_lead = max(worst_lead, abs(float(lv["onsite_s"]) - on_native), abs(float(lv["network_s"]) - net_native))
        worst_sp = max(worst_sp, abs(analytic.sp_time(r) - r * (1.0 / pwave.V_S_KMPS - 1.0 / pwave.V_P_KMPS)))
    crossover_native = analytic.lead_crossover_km()
    return {
        "max_abs_lead_err_s": worst_lead,
        "max_abs_sp_err_s": worst_sp,
        "crossover_native_km": crossover_native,
        "sp_per_km_native_s": analytic.sp_time(1.0),
    }


def compute(
    t_onsite_s: float = ONSITE_DECISION_S, t_network_s: float = NETWORK_ALERT_S, native_check: bool = True
) -> dict[str, Any]:
    """Every number of the study (the JSON record), without plotting."""
    r = _R_GRID_KM
    nominal = leads(r, t_onsite_s=t_onsite_s, t_network_s=t_network_s)
    band = _band(r, t_onsite_s, t_network_s)
    sp_per_km = 1.0 / pwave.V_S_KMPS - 1.0 / pwave.V_P_KMPS
    crossover = pwave.V_P_KMPS * (t_network_s - t_onsite_s)
    crossover_band = [V_P_BAND_KMPS[0] * (t_network_s - t_onsite_s), V_P_BAND_KMPS[1] * (t_network_s - t_onsite_s)]
    # Blind zone: the on-site lead is zero at R0 = t_onsite/(1/V_s − 1/V_p); the best lead
    # reaches t_onsite only at 2·R0 ("no useful on-site warning below about 20 km").
    r_zero = t_onsite_s / sp_per_km
    r_useful = 2.0 * t_onsite_s / sp_per_km
    lead_at_crossover = float(leads(crossover, t_onsite_s=t_onsite_s, t_network_s=t_network_s)["fused_s"])
    table = []
    for rt in R_TABLE_KM:
        lv = leads(rt, t_onsite_s=t_onsite_s, t_network_s=t_network_s)
        table.append(
            {
                "r_hyp_km": rt,
                "s_minus_p_s": rt * sp_per_km,
                "onsite_s": float(lv["onsite_s"]),
                "network_s": float(lv["network_s"]),
                "best_s": max(0.0, float(lv["fused_s"])),
            }
        )
    epi: dict[str, Any] = {"r_epi_km": _EPI_GRID_KM}
    for depth in _DEPTHS_KM:
        rh = np.hypot(_EPI_GRID_KM, depth)
        epi[f"fused_depth_{depth:g}_km_s"] = leads(rh, t_onsite_s=t_onsite_s, t_network_s=t_network_s)["fused_s"]
    record: dict[str, Any] = {
        "inputs": {
            "v_p_kmps": pwave.V_P_KMPS,
            "v_s_kmps": pwave.V_S_KMPS,
            "t_onsite_s": t_onsite_s,
            "t_network_s": t_network_s,
            "v_p_band_kmps": list(V_P_BAND_KMPS),
            "v_s_band_kmps": list(V_S_BAND_KMPS),
            "distance": "hypocentral R (km); lead measured to the S arrival",
            "alerts": "synthetic/injected; fabrail is not an EEW provider",
        },
        "r_hyp_km": r,
        "lead_onsite_s": nominal["onsite_s"],
        "lead_network_s": nominal["network_s"],
        "lead_fused_s": nominal["fused_s"],
        "band": band,
        "sp_per_km_s": sp_per_km,
        "crossover_km": crossover,
        "crossover_band_km": crossover_band,
        "lead_at_crossover_s": lead_at_crossover,
        "blind_zone": {
            "onsite_zero_lead_km": r_zero,
            "best_lead_reaches_t_onsite_km": r_useful,
            "note": "no useful on-site warning below about 20 km (plan §7): the best lead there is at most "
            f"{t_onsite_s:.1f} s, enough to FREEZE (<= 100 ms, FAB-SEI-009) but not to land",
        },
        "table": table,
        "epicentral": epi,
    }
    if native_check:
        record["native_check"] = _native_check()
    return record


def _figure(rec: dict[str, Any]) -> Any:
    from matplotlib.figure import Figure

    from .. import plotstyle as ps

    ps.apply()
    c = ps.COLORS
    fig = Figure(figsize=(11.5, 4.6))
    ax1, ax2 = fig.subplots(1, 2, gridspec_kw={"width_ratios": [1.55, 1.0]})
    r = rec["r_hyp_km"]
    band = rec["band"]
    ax1.fill_between(r, band["onsite_s_lo"], band["onsite_s_hi"], color=c["s1"], alpha=0.12, linewidth=0)
    ax1.fill_between(r, band["network_s_lo"], band["network_s_hi"], color=c["s2"], alpha=0.12, linewidth=0)
    ax1.plot(r, rec["lead_onsite_s"], color=c["s1"], lw=1.6, label="on-site (P pick + 1.2 s)")
    ax1.plot(r, rec["lead_network_s"], color=c["s2"], lw=1.6, label="network (origin + 8 s)")
    ax1.plot(r, rec["lead_fused_s"], color=c["ink"], lw=2.2, ls=(0, (5, 2)), label="fused (earliest)")
    ps.zero_line(ax1)
    ax1.scatter([rec["crossover_km"]], [rec["lead_at_crossover_s"]], color=c["star"], zorder=5, s=36, label="crossover")
    ax1.axvspan(0.0, rec["blind_zone"]["best_lead_reaches_t_onsite_km"], color=c["ctrl"], alpha=0.10, linewidth=0)
    ax1.annotate(
        f"crossover {rec['crossover_km']:.1f} km\n(band {rec['crossover_band_km'][0]:.1f}-{rec['crossover_band_km'][1]:.1f} km)",
        xy=(rec["crossover_km"], rec["lead_at_crossover_s"]),
        xytext=(62, -2.0),
        fontsize=8.5,
        color=c["ink2"],
        arrowprops={"arrowstyle": "-", "color": c["muted"], "lw": 0.8},
    )
    ax1.annotate("blind zone:\nbest lead <= 1.2 s", xy=(1.5, 8.0), fontsize=8.5, color=c["ink2"])
    ax1.set_xlim(0.0, 150.0)
    ax1.set_ylim(-8.0, 36.0)
    ax1.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(ax1, "hypocentral distance R (km)", "lead before S arrival (s)", "Warning time by alert path")
    ax1.annotate(
        "bands: V_p 5.5-6.5, V_s 3.2-3.7 km/s",
        xy=(0.99, 0.02),
        xycoords="axes fraction",
        ha="right",
        fontsize=8,
        color=c["muted"],
    )
    epi = rec["epicentral"]
    for i, depth in enumerate(_DEPTHS_KM):
        ax2.plot(epi["r_epi_km"], epi[f"fused_depth_{depth:g}_km_s"], color=ps.SERIES[i], lw=1.6, label=f"depth {depth:g} km")
    ps.zero_line(ax2)
    ax2.set_xlim(0.0, 120.0)
    ax2.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(ax2, "epicentral distance (km)", "fused lead (s)", "Depth moves the blind zone")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    ps.brand(fig, "eew_leadtime · closed forms (PHYSICS §20) · synthetic/injected alerts, not an EEW provider")
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the study (closed forms only; ``cfg`` is not used: the inputs are the documented defaults)."""
    del cfg
    rec = compute()
    return StudyResult(fig=_figure(rec), record=rec)


STUDY = StudySpec(
    name="eew_leadtime",
    tier=1,
    question="How much warning before the S-wave do on-site, network and fused alerts give vs distance, "
    "and where is the 41 km crossover?",
    run=run,
)
