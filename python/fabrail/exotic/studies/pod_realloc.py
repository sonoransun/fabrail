"""Study ``pod_realloc`` (tier 1): how much capacity headroom does static pod reallocation need?

Model: the XC-ARN-1 scenario (``fabrail_exo::alloc`` through ``fabrail_exo_sim::scenarios``):
an 8,600 t ARENA tray on 500 zero-power hybrid pods loses one pod (the most loaded by
default); a rigid-body heave/pitch/roll wrench is reallocated to the remaining pods by
weighted least squares with active-set clamping, and the tray lands (LAND) when a
quadrant's residual capacity falls below 1.3× its weight share. The sweep varies the pod
capacity ratio (capacity/nominal share) and the centre-of-gravity offset.

Static allocation only: the dynamic gap error after a pod loss needs the dynamic ARENA
simulation, which is roadmap 🗺️; the result runs on a reduced model and is labelled so.
Research tier 🧪.

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


def compute(seed: int = 42, reduced: bool = False) -> dict[str, Any]:
    """Every number of the study (the record), without plotting."""
    from ..scenarios import simulate

    ratios = np.round(np.linspace(1.0, 2.0, 6 if reduced else 21), 6)
    cogs = np.round(np.linspace(0.0, 20.0, 5 if reduced else 21), 6)
    by_ratio = {"capacity_ratio": arr(ratios), "max_utilisation": [], "land": [], "ok": []}
    for r in ratios:
        run = simulate("arena", "XC-ARN-1", seed, {"capacity_ratio": float(r)})
        by_ratio["max_utilisation"].append(float(run.kpis["realloc_max_utilisation"]))
        by_ratio["land"].append(float(run.kpis["realloc_land"]))
        by_ratio["ok"].append(float(run.kpis["realloc_ok"]))
    by_cog = {"cog_x_m": arr(cogs), "max_utilisation": [], "land": []}
    for c in cogs:
        run = simulate("arena", "XC-ARN-1", seed, {"cog_x_m": float(c)})
        by_cog["max_utilisation"].append(float(run.kpis["realloc_max_utilisation"]))
        by_cog["land"].append(float(run.kpis["realloc_land"]))
    base = simulate("arena", "XC-ARN-1", seed)
    lands = [r for r, land in zip(by_ratio["capacity_ratio"], by_ratio["land"]) if land >= 0.5]
    return {
        "inputs": {"scenario": "XC-ARN-1", "class": "arena", "pods": 500, "defaults": base.inputs, "seed": seed},
        "by_capacity_ratio": by_ratio,
        "by_cog_offset": by_cog,
        "baseline": {"kpis": base.kpis, "labels": base.labels, "passed": base.passed, "reduced_model": base.reduced_model},
        "land_below_capacity_ratio": max(lands) if lands else None,
        "checks": {"baseline_reallocates_or_lands": check(base.passed, base.labels.get("outcome", ""), "reallocated or LAND", "EXOTIC §X17 XC-ARN-1")},
        "caption": (
            "XC-ARN-1 reduced model: static weighted-least-squares reallocation of 500 pods after one pod is lost; LAND when a "
            "quadrant's residual capacity falls below 1.3× its share. The dynamic gap error is roadmap. " + NON_CLAIMS
        ),
    }


def _figure(rec: dict[str, Any], cfg: RunConfig) -> Any:
    from ... import plotstyle as ps
    from ..plots import font_safe

    fig, axes = new_figure(1, 2, size=(12.0, 4.8))
    ax1, ax2 = axes[0]
    br = rec["by_capacity_ratio"]
    ax1.plot(br["capacity_ratio"], br["max_utilisation"], color=ps.COLORS["s1"], marker="o", lw=1.6, label="max pod utilisation after loss")
    ax1.plot(br["capacity_ratio"], br["land"], color=ps.COLORS["s2"], marker="s", lw=1.0, ls="--", label="LAND demanded (1 = yes)")
    ax1.legend(loc="upper right", **ps.LEGEND_KW)
    ps.style_ax(ax1, "pod capacity / nominal share", "utilisation · LAND flag", "Headroom decides reallocate or land")
    bc = rec["by_cog_offset"]
    ax2.plot(bc["cog_x_m"], bc["max_utilisation"], color=ps.COLORS["s3"], marker="o", lw=1.6, label="max pod utilisation")
    ax2.plot(bc["cog_x_m"], bc["land"], color=ps.COLORS["s2"], marker="s", lw=1.0, ls="--", label="LAND demanded")
    ax2.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(ax2, "centre-of-gravity offset x (m)", "utilisation · LAND flag", "Load offset at capacity ratio 1.6")
    finish(fig, title="ARENA static pod reallocation (reduced model)", question=STUDY.question, caption=rec["caption"], meta=meta(cfg, STUDY.name))
    font_safe(fig)
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the study (XC-ARN-1 sweeps; ``quick()`` uses coarse grids)."""
    rec = compute(cfg.seed, reduced=quick())
    rec["quick"] = quick()
    return StudyResult(fig=_figure(rec, cfg), record=rec)


STUDY = StudySpec(
    name="pod_realloc",
    tier=1,
    question="When one of 500 ARENA pods fails, how much capacity headroom lets static reallocation carry the tray instead of landing?",
    run=run,
)
