"""Study ``sentinel_wind`` (tier 1): what does the SENTINEL hybrid mass damper do in wind, against ζ_s?

Model: the XS12 scenario of ``fabrail_exo_sim::scenarios`` (EXOTIC §X6–§X8; FAB-EXO-607):
a 2-DOF host (first mode, 100 kt modal mass, T 7 s) with a 1,000 t HMD (μ 1 %, Warburton
force tuning, ζ on ω_s) under Davenport wind synthesised as a Shinozuka sum and scaled to
the requested uncontrolled peak acceleration; bare, passive and LQR-active runs, RK4.

The host RMS reduction is always reported with the host's own damping ζ_s, because the
passive optimum falls from 44.7 % at ζ_s 1 % to 30.0 % at ζ_s 2 % (H2, white-noise force);
the ≥ 40 % target applies to the active mode only. Activation assumes a low-coercivity
rail and an ironless LSM (an iron-core LSM's cogging alone would be 0.12–0.37 mg).
Research tier 🧪; mass dampers are mainly wind devices.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from ...studies.runner import StudyResult, StudySpec, finish, new_figure, quick
from ._common import NON_CLAIMS, arr, check, decimate, meta

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...config import RunConfig

__all__ = ["STUDY", "ZETA_S", "compute", "run"]

QUICK_AWARE = True

#: Host damping ratios swept (EXOTIC §X6: the reduction is reported against ζ_s).
ZETA_S: tuple[float, ...] = (0.005, 0.01, 0.02)
#: Uncontrolled peak host acceleration, mg (wind comfort 3–7 mg; XS12 default).
A_TARGET_MG = 5.0
#: Time-series window shown, s.
WINDOW_S = 600.0


def compute(seed: int = 42, reduced: bool = False) -> dict[str, Any]:
    """Every number of the study (the record), without plotting."""
    from ..scenarios import simulate

    zetas = (0.01, 0.02) if reduced else ZETA_S
    duration = 600.0 if reduced else 3600.0
    rows = []
    series: dict[str, Any] = {}
    for z in zetas:
        r = simulate("sentinel", "XS12", seed, {"zeta_s": z, "a_target_mg": A_TARGET_MG, "duration_s": duration}, channels=(z == 0.01))
        k = r.kpis
        rows.append(
            {
                "zeta_s": z,
                "rms_reduction_passive_pct": k["rms_reduction_passive_pct"],
                "rms_reduction_active_pct": k["rms_reduction_active_pct"],
                "h2_reduction_passive_pct": k["h2_reduction_passive_pct"],
                "h2_reduction_active_pct": k["h2_reduction_active_pct"],
                "activation_mg": k["activation_mg"],
                "stroke_peak_active_m": k["stroke_peak_active_m"],
                "lsm_force_peak_n": k["lsm_force_peak_n"],
                "passed": r.passed,
            }
        )
        if z == 0.01:
            t = r.t_s
            m = t <= t[0] + WINDOW_S
            td, xb, xp, xa = decimate(t[m], r.channels["x_host_bare_m"][m], r.channels["x_host_passive_m"][m], r.channels["x_host_active_m"][m])
            series = {
                "t_s": arr(td, 6),
                "x_host_bare_mm": arr(np.asarray(xb) * 1e3, 6),
                "x_host_passive_mm": arr(np.asarray(xp) * 1e3, 6),
                "x_host_active_mm": arr(np.asarray(xa) * 1e3, 6),
                "labels": r.labels,
                "model": r.model,
                "inputs": r.inputs,
            }
    at1 = next(row for row in rows if row["zeta_s"] == 0.01)
    return {
        "inputs": {"a_target_mg": A_TARGET_MG, "duration_s": duration, "zeta_s": list(zetas), "mu": 0.01, "scenario": "XS12", "seed": seed},
        "sweep": {key: arr([row[key] for row in rows]) for key in rows[0] if key != "passed"},
        "passed": [row["passed"] for row in rows],
        "series_zeta_0p01": series,
        "checks": {
            "activation_le_0p3_mg": check(at1["activation_mg"] <= 0.3, at1["activation_mg"], 0.3, "FAB-EXO-607; EXOTIC §X8"),
            "active_reduction_ge_40pct": check(at1["rms_reduction_active_pct"] >= 40.0, at1["rms_reduction_active_pct"], 40.0, "FAB-EXO-607 (active mode only)"),
            "h2_passive_at_zeta_0p01_pct": check(abs(at1["h2_reduction_passive_pct"] - 44.7) <= 0.05, at1["h2_reduction_passive_pct"], 44.7, "EXOTIC §2 (reported)"),
        },
        "caption": (
            "XS12 reduced model (2-DOF host plus 1,000 t HMD, Davenport wind). Reductions are reported against ζ_s; "
            "the 40 % target is for active mode only. Activation assumes a low-coercivity rail and an ironless LSM. "
            "Research tier. " + NON_CLAIMS
        ),
    }


def _figure(rec: dict[str, Any], cfg: RunConfig) -> Any:
    from ... import plotstyle as ps
    from ..plots import font_safe

    fig, axes = new_figure(1, 2, size=(12.0, 5.2))
    ax1, ax2 = axes[0]
    sw = rec["sweep"]
    z = np.asarray(sw["zeta_s"]) * 100.0
    ax1.plot(z, sw["rms_reduction_passive_pct"], color=ps.COLORS["s1"], marker="o", lw=1.6, label="passive (simulated)")
    ax1.plot(z, sw["rms_reduction_active_pct"], color=ps.COLORS["s2"], marker="s", lw=1.6, label="LQR active (simulated)")
    ax1.plot(z, sw["h2_reduction_passive_pct"], color=ps.COLORS["ctrl"], marker="^", lw=1.0, ls="--", label="passive H2 optimum (white noise)")
    ax1.axhline(40.0, color=ps.COLORS["star"], lw=1.0, ls=":", label="40 % target (active only)")
    ax1.legend(loc="lower left", **ps.LEGEND_KW)
    ps.style_ax(ax1, "host damping ζ_s (%)", "host displacement RMS reduction (%)", "Reduction depends on ζ_s")
    s = rec["series_zeta_0p01"]
    ax2.plot(s["t_s"], s["x_host_bare_mm"], color=ps.COLORS["ctrl"], lw=0.8, label="bare structure")
    ax2.plot(s["t_s"], s["x_host_passive_mm"], color=ps.COLORS["s1"], lw=0.9, label="passive HMD")
    ax2.plot(s["t_s"], s["x_host_active_mm"], color=ps.COLORS["s2"], lw=0.9, label="active HMD")
    ax2.legend(loc="upper right", **ps.LEGEND_KW)
    ps.style_ax(ax2, "time (s)", "host roof displacement (mm)", "ζ_s = 1 %, 5 mg uncontrolled peak")
    finish(fig, title="SENTINEL in wind", question=STUDY.question, caption=rec["caption"], meta=meta(cfg, STUDY.name))
    font_safe(fig)
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the study (XS12 sweep; ``quick()`` uses 2 ζ_s values and 600 s records)."""
    rec = compute(cfg.seed, reduced=quick())
    rec["quick"] = quick()
    return StudyResult(fig=_figure(rec, cfg), record=rec)


STUDY = StudySpec(
    name="sentinel_wind",
    tier=1,
    question="How much does a 1,000 t hybrid mass damper reduce tower sway in wind, and how does that depend on ζ_s?",
    run=run,
)
