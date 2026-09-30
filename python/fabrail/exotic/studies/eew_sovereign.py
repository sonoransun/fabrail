"""Study ``eew_sovereign`` (tier 1): what does moving sensors toward the fault buy, and what does fusion do?

Model (EXOTIC §X10; FAB-EXO-802..805):

- Warning W = R_site/V_s − t_alert − t_act with t_alert = R_k/V_p + 3.3 s latency; each km
  of sensor advance buys 1/V_p = 0.154 s (P-based) or 1/V_s = 0.270 s (PLUM-type) until
  the fixed latency takes over (``fabrail_exo_sim::eew_timing``).
- The offshore worked case (epicentre 100 km offshore, 20 km deep, city 20 km inland):
  coastal land network, S-net-like seafloor stations, DAS over the source; analytic and the
  XS09 wavefield-timing simulation (±1 s, FAB-EXO-804).
- XS08: a cancelled alert and a concurrent-quake false alarm return every class to NORMAL.

fabrail is not an EEW provider: every alert here is synthetic/injected, and the warning
near the source is about zero whatever the budget.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from ...studies.runner import StudyResult, StudySpec, finish, new_figure, quick
from ._common import NON_CLAIMS, arr, check, meta

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...config import RunConfig

__all__ = ["STUDY", "compute", "run"]

QUICK_AWARE = True

#: Sensor-advance geometry: site at 60 km epicentral, 10 km deep (EXOTIC §X10 illustration).
R_SITE_KM = 60.0
DEPTH_KM = 10.0


def compute(seed: int = 42, reduced: bool = False) -> dict[str, Any]:
    """Every number of the study (the record), without plotting."""
    from .. import eew
    from ..scenarios import info, simulate

    adv = np.round(np.arange(0.0, 60.0 + 1e-9, 10.0 if reduced else 2.0), 6)
    ws = eew.sensor_advance(R_SITE_KM, DEPTH_KM, tuple(float(a) for a in adv))
    warn = np.array([w.warning_s for w in ws])
    g = eew.gains()
    line = warn[0] + g["gain_p_s_per_km"] * adv
    off = eew.offshore_worked_case()
    xs09 = simulate("tidegate", "XS09", seed)
    k = xs09.kpis
    sim = [k["land_warning_sim_s"], k["snet_warning_sim_s"], k["das_warning_sim_s"]]
    xs08 = {c: simulate(c, "XS08", seed).passed for c in info("XS08").classes}
    err = max(abs(a.warning_s - b) for a, b in zip(off, sim))
    return {
        "inputs": {"r_site_km": R_SITE_KM, "depth_km": DEPTH_KM, "latency_s": 3.3, "v_p_kmps": 6.5, "v_s_kmps": 3.7, "offshore": eew.OFFSHORE_CASE, "seed": seed},
        "gains": g,
        "advance": {"advance_km": arr(adv), "warning_s": arr(warn), "warning_gain_line_s": arr(line), "t_alert_s": arr([w.t_alert_s for w in ws])},
        "offshore": {
            "offshore_label": [w.label for w in off],
            "offshore_warning_s": arr([w.warning_s for w in off]),
            "offshore_t_alert_s": arr([w.t_alert_s for w in off]),
            "offshore_warning_sim_s": arr(sim),
            "t_s_city_s": off[0].t_s_site_s,
        },
        "xs08_returns_normal": xs08,
        "checks": {
            "gain_p_s_per_km": check(abs(g["gain_p_s_per_km"] - 0.15385) <= 5e-6, g["gain_p_s_per_km"], 0.15385, "FAB-EXO-803"),
            "xs09_within_1s": check(err <= 1.0, err, 1.0, "FAB-EXO-804 (±1 s)"),
            "xs08_all_normal": check(all(xs08.values()), sum(xs08.values()), len(xs08), "FAB-EXO-805"),
        },
        "caption": (
            "Timing model with a 3.3 s latency budget [UNCERTAIN]; the offshore case compares the analytic warning with the "
            "XS09 wavefield-timing simulation. fabrail is not an EEW provider; every alert is synthetic; near the source the "
            "warning is about zero. " + NON_CLAIMS
        ),
    }


def _figure(rec: dict[str, Any], cfg: RunConfig) -> Any:
    from ... import plotstyle as ps
    from ..plots import font_safe

    fig, axes = new_figure(1, 2, size=(12.0, 4.8))
    ax1, ax2 = axes[0]
    a = rec["advance"]
    ax1.plot(a["advance_km"], a["warning_s"], color=ps.COLORS["s1"], marker="o", lw=1.6, label="warning at the site")
    ax1.plot(a["advance_km"], a["warning_gain_line_s"], color=ps.COLORS["muted"], lw=1.0, ls="--", label="+0.154 s per km")
    ax1.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(ax1, "sensor advance toward the epicentre (km)", "warning before S (s)", f"Site {R_SITE_KM:g} km, {DEPTH_KM:g} km deep")
    o = rec["offshore"]
    y = np.arange(len(o["offshore_label"]))
    ax2.barh(y - 0.18, o["offshore_warning_s"], height=0.36, color=ps.COLORS["s1"], label="analytic")
    ax2.barh(y + 0.18, o["offshore_warning_sim_s"], height=0.36, color=ps.COLORS["s2"], label="XS09 simulation")
    ax2.set_yticks(y, o["offshore_label"])
    ax2.invert_yaxis()
    ax2.legend(loc="lower right", **ps.LEGEND_KW)
    ps.style_ax(ax2, "warning at the city (s)", None, "Offshore M8 (synthetic)")
    finish(fig, title="Sovereign EEW timing", question=STUDY.question, caption=rec["caption"], meta=meta(cfg, STUDY.name))
    font_safe(fig)
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the study (timing closed forms plus XS08/XS09)."""
    rec = compute(cfg.seed, reduced=quick())
    rec["quick"] = quick()
    return StudyResult(fig=_figure(rec, cfg), record=rec)


STUDY = StudySpec(
    name="eew_sovereign",
    tier=1,
    question="How much warning does moving sensors offshore buy, and do cancelled or false alerts return to NORMAL?",
    run=run,
)
