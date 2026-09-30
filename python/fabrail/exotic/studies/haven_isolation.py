"""Study ``haven_isolation`` (tier 1): what does a levitated isolation platform at T = 6 s buy, and cost?

Model (EXOTIC §X9; FAB-EXO-110, FAB-EXO-505, FAB-EXO-701..703):

- Transmissibility with r = f/f_iso (closed forms in ``fabrail_params::derive``, PHYSICS §21):
  passive T = √[(1 + (2ζr)²)/((1 − r²)² + (2ζr)²)], skyhook T = 1/√[(1 − r²)² + (2ζr)²].
- Demand: ASCE 7-16 Ch. 17 displacement S_D = S_D1·g·T/(4π²·B) and the platform peak
  a_peak = ω²·S_D·√(1 + 4ζ²), at DBE (S_D1 0.6) and MCE (S_M1 0.9), an analysis output
  against the 0.1 g occupant target, never a trip.
- Simulated: the XS01 near-fault pulse (Kobe-type, Mw 6.9, 0.9 m/s) and the XS02 fling
  (5 m in 5 s) on the HAVEN-R 3-DOF planar reduced model (``fabrail_exo_sim::platform``).

Research tier 🧪. There are no ASCE 7 Ch. 17 prototype tests and no code path; maglev
vertical isolation is not proposed (the vertical load goes to passive air springs); the
moat edge is a shear hazard, so the moat has passive covers plus exclusion.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from ...studies.runner import StudyResult, StudySpec, finish, new_figure, quick
from ._common import NON_CLAIMS, arr, check, decimate, meta

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...config import RunConfig

__all__ = ["STUDY", "compute", "run"]

QUICK_AWARE = True

#: Isolation damping ratio of the closed forms (EXOTIC §X9 tables).
ZETA = 0.2
#: Occupant target, g (HAVEN OccupiedPlatform; an analysis output at DBE).
A_TARGET_G = 0.1


def compute(seed: int = 42, reduced: bool = False) -> dict[str, Any]:
    """Every number of the study (the record), without plotting."""
    from ..classes import haven_demand, isolation_transmissibility
    from ..scenarios import simulate

    f_hz = np.round(np.geomspace(0.05, 5.0, 60 if reduced else 200), 9)
    t_iso = 6.0
    passive, sky = isolation_transmissibility(f_hz, t_iso, ZETA)
    periods = np.round(np.arange(2.0, 8.0 + 1e-9, 0.5 if reduced else 0.25), 6)
    dbe = [haven_demand(0.6, float(T), ZETA).a_peak_g for T in periods]
    mce = [haven_demand(0.9, float(T), ZETA).a_peak_g for T in periods]
    anchors = []
    for T in (4.0, 6.0):
        for f in (0.5, 1.0, 2.0):
            p, s = isolation_transmissibility(np.array([f]), T, ZETA)
            anchors.append({"t_iso_s": T, "f_hz": f, "passive": float(p[0]), "skyhook": float(s[0])})
    xs01 = simulate("haven_r", "XS01", seed, channels=True)
    xs02 = simulate("haven_r", "XS02", seed)
    t, ag, aa, x = decimate(xs01.t_s, xs01.channels["a_ground_mps2"], xs01.channels["a_abs_mps2"], xs01.channels["x_m"], max_points=800)
    d6 = haven_demand(0.6, 6.0, ZETA)
    return {
        "inputs": {"zeta": ZETA, "t_iso_s": t_iso, "s_d1_dbe": 0.6, "s_m1_mce": 0.9, "b_factor": d6.b_factor, "seed": seed},
        "transmissibility": {"f_hz": arr(f_hz), "passive": arr(passive), "skyhook": arr(sky)},
        "anchors": anchors,
        "demand": {"period_s": arr(periods), "a_peak_dbe_g": arr(dbe), "a_peak_mce_g": arr(mce), "target_g": A_TARGET_G},
        "haven_r_t6": {"s_d_dbe_m": d6.s_d_m, "a_peak_dbe_g": d6.a_peak_g, "s_d_mce_m": haven_demand(0.9, 6.0, ZETA).s_d_m, "a_peak_mce_g": haven_demand(0.9, 6.0, ZETA).a_peak_g},
        "xs01": {
            "t_s": arr(t, 6),
            "a_ground_g": arr(np.asarray(ag) / 9.80665, 6),
            "a_platform_g": arr(np.asarray(aa) / 9.80665, 6),
            "x_m": arr(x, 6),
            "kpis": xs01.kpis,
            "passed": xs01.passed,
            "inputs": xs01.inputs,
        },
        "xs02": {"kpis": xs02.kpis, "labels": xs02.labels, "passed": xs02.passed, "inputs": xs02.inputs},
        "checks": {
            "a_peak_dbe_t6_le_0p1g": check(d6.a_peak_g <= A_TARGET_G, d6.a_peak_g, A_TARGET_G, "FAB-EXO-505 (analysis output at DBE)"),
            "xs01_no_buffer_contact": check(xs01.kpis.get("buffer_contact") == 0.0, xs01.kpis.get("buffer_contact"), 0.0, "EXOTIC §X17 XS01"),
            "xs02_moat_exceedance_reported": check(True, xs02.labels.get("exceedance", ""), "reported", "EXOTIC §X17 XS02"),
        },
        "caption": (
            "Closed-form transmissibility at T = 6 s, ζ 0.2 (PHYSICS §21), ASCE 7-16 Ch. 17 demand (analysis output), "
            "and the HAVEN-R 3-DOF reduced model under a Kobe-type pulse. No ASCE 7 prototype tests; vertical support stays "
            "passive; the moat is covered and excluded. Research tier. " + NON_CLAIMS
        ),
    }


def _figure(rec: dict[str, Any], cfg: RunConfig) -> Any:
    from ... import plotstyle as ps
    from ..plots import font_safe

    fig, axes = new_figure(1, 3, size=(13.5, 5.0))
    ax1, ax2, ax3 = axes[0]
    tr = rec["transmissibility"]
    ax1.loglog(tr["f_hz"], tr["passive"], color=ps.COLORS["s1"], lw=1.6, label="passive")
    ax1.loglog(tr["f_hz"], tr["skyhook"], color=ps.COLORS["s2"], lw=1.6, label="skyhook")
    ax1.legend(loc="lower left", **ps.LEGEND_KW)
    ps.style_ax(ax1, "frequency (Hz)", "transmissibility", "T_iso 6 s, ζ 0.2")
    d = rec["demand"]
    ax2.plot(d["period_s"], d["a_peak_dbe_g"], color=ps.COLORS["s3"], lw=1.6, label="DBE (S_D1 0.6)")
    ax2.plot(d["period_s"], d["a_peak_mce_g"], color=ps.COLORS["s4"], lw=1.6, label="MCE (S_M1 0.9)")
    ax2.axhline(d["target_g"], color=ps.COLORS["star"], lw=1.0, ls=":", label="0.1 g occupant target")
    ax2.legend(loc="upper right", **ps.LEGEND_KW)
    ps.style_ax(ax2, "isolation period T (s)", "platform peak a_peak (g)", "Why T = 6 s")
    s = rec["xs01"]
    ax3.plot(s["t_s"], s["a_ground_g"], color=ps.COLORS["ctrl"], lw=0.8, label="ground")
    ax3.plot(s["t_s"], s["a_platform_g"], color=ps.COLORS["s1"], lw=1.2, label="platform (absolute)")
    ax3.legend(loc="upper right", **ps.LEGEND_KW)
    ps.style_ax(ax3, "time (s)", "acceleration (g)", "XS01 Kobe-type pulse, HAVEN-R")
    finish(fig, title="HAVEN-R isolation platform", question=STUDY.question, caption=rec["caption"], meta=meta(cfg, STUDY.name))
    font_safe(fig)
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the study (closed forms plus XS01/XS02; ``quick()`` uses coarser grids)."""
    rec = compute(cfg.seed, reduced=quick())
    rec["quick"] = quick()
    return StudyResult(fig=_figure(rec, cfg), record=rec)


STUDY = StudySpec(
    name="haven_isolation",
    tier=1,
    question="What does a levitated isolation platform at T = 6 s transmit, what acceleration does it see, and what does a pulse do?",
    run=run,
)
