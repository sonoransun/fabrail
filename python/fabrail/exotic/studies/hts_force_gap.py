"""Study ``hts_force_gap`` (tier 1): how much does HTS flux pinning carry, and at what area and logistics?

Model (EXOTIC §X3; FAB-EXO-113, FAB-EXO-202, FAB-EXO-204, FAB-EXO-205):

- The frozen-image HTS model of ``fabrail_exo_sim::suspension::HtsBulk`` fitted to the
  ATZ demonstrator (four cryostats, 4 × 492 cm², field-cooled at 35 mm): 300 kgf at 20 mm
  and 550 kgf at 15 mm (fit, ±1 %); out of sample 53 kPa at 10 mm (±15 %) and the
  cryostat-500 datum of 5,000 N over 912 cm² at 5–8 mm (±30 %, evaluated near 8 mm).
- The area and logistics table (53 kPa at 10 mm; EMS comparison at 0.3 MPa): HTS needs
  about 5–8× the area of EMS.
- XC-HTS-1: LN₂ depletion on the HAVEN-R reduced model (land at 10 % LN₂, no free fall;
  the post-depletion fade takes tens of minutes, not hours).

All parameters are fitted to one four-cryostat demonstrator; extrapolation to hundreds
of square metres is unvalidated. Cryogenic logistics (LN₂ supply, oxygen depletion,
vacuum, field-cooling interlocks) apply. HTS pins the lateral axis, so it is usable
only in 1-D stages. Research tier 🧪 for the application; the model is 🔶.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from ...studies.runner import StudyResult, StudySpec, finish, new_figure, quick
from ._common import NON_CLAIMS, arr, check, decimate, meta

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...config import RunConfig

__all__ = ["ATZ_AREA_M2", "STUDY", "compute", "run"]

QUICK_AWARE = True

#: Active area of the four ATZ cryostat-250 units, m² (``suspension::ATZ_CRYO250_AREA_M2`` × 4).
ATZ_AREA_M2 = 4 * 492.0e-4
#: kgf in N (standard gravity).
KGF_N = 9.80665


def compute(seed: int = 42, reduced: bool = False) -> dict[str, Any]:
    """Every number of the study (the record), without plotting."""
    from .._util import native
    from ..classes import hts_area_table
    from ..scenarios import simulate

    n = native()
    z_mm = np.round(np.arange(5.0, 30.0 + 1e-9, 1.0 if reduced else 0.25), 6)
    fz_kgf, p_kpa = [], []
    for z in z_mm:
        h = n.exo_hts_force(float(z) * 1e-3)
        fz_kgf.append(h["fz_n"] / KGF_N)
        p_kpa.append(h["pressure_pa"] / 1e3)
    f20 = n.exo_hts_force(0.020)["fz_n"] / KGF_N
    f15 = n.exo_hts_force(0.015)["fz_n"] / KGF_N
    p10 = n.exo_hts_force(0.010)["pressure_pa"] / 1e3
    p8 = n.exo_hts_force(0.008)["pressure_pa"] / 1e3
    rows = hts_area_table()
    ln2 = simulate("haven_r", "XC-HTS-1", seed, channels=True)
    t_h, frac, lift = decimate(ln2.t_s / 3600.0, ln2.channels["ln2_frac"], ln2.channels["lift_over_load"], max_points=600)
    return {
        "inputs": {"atz_area_m2": ATZ_AREA_M2, "kgf_n": KGF_N, "z_fc_mm": 35.0, "seed": seed},
        "force_gap": {"z_mm": arr(z_mm), "fz_kgf": arr(fz_kgf, 6), "pressure_kpa": arr(p_kpa, 6)},
        "atz_points": {"z_mm": [20.0, 15.0], "fz_kgf": [300.0, 550.0], "model_kgf": [f20, f15]},
        "out_of_sample": {"z_mm": [10.0, 8.0], "measured_kpa": [53.0, 5000.0 / 912e-4 / 1e3], "model_kpa": [p10, p8], "tolerance": [0.15, 0.30]},
        "area_table": {
            "class": [r.name for r in rows],
            "hts_area_m2": arr([r.area_m2 for r in rows]),
            "ems_area_m2": arr([r.ems_area_m2 for r in rows]),
            "area_ratio": arr([r.area_m2 / r.ems_area_m2 for r in rows]),
            "cryostats": [r.cryostats for r in rows],
            "heat_w": arr([r.heat_w for r in rows]),
            "ln2_kg_per_day": arr([r.ln2_kg_per_day for r in rows]),
        },
        "xc_hts_1": {"t_h": arr(t_h, 6), "ln2_frac": arr(frac, 6), "lift_over_load": arr(lift, 6), "kpis": ln2.kpis, "passed": ln2.passed},
        "checks": {
            "fit_20mm_kgf": check(abs(f20 / 300.0 - 1.0) <= 0.01, f20, 300.0, "FAB-EXO-202 (±1 %)"),
            "fit_15mm_kgf": check(abs(f15 / 550.0 - 1.0) <= 0.01, f15, 550.0, "FAB-EXO-202 (±1 %)"),
            "oos_10mm_kpa": check(abs(p10 / 53.0 - 1.0) <= 0.15, p10, 53.0, "FAB-EXO-113 (±15 %)"),
            "oos_8mm_kpa": check(abs(p8 / (5000.0 / 912e-4 / 1e3) - 1.0) <= 0.30, p8, 5000.0 / 912e-4 / 1e3, "FAB-EXO-113 (±30 %, near 8 mm)"),
            "lands_at_10pct": check(abs(ln2.kpis["ln2_frac_at_land"] - 0.10) <= 0.005, ln2.kpis["ln2_frac_at_land"], 0.10, "FAB-EXO-205"),
        },
        "caption": (
            "Frozen-image HTS model fitted to one four-cryostat ATZ demonstrator; area table at 53 kPa (10 mm); "
            "XC-HTS-1 LN2 depletion on the HAVEN-R reduced model. Extrapolation to hundreds of square metres is unvalidated; "
            "cryogenic logistics apply; HTS pins the lateral axis (1-D stages only). " + NON_CLAIMS
        ),
    }


def _figure(rec: dict[str, Any], cfg: RunConfig) -> Any:
    from ... import plotstyle as ps
    from ..plots import font_safe

    fig, axes = new_figure(1, 3, size=(13.5, 5.0))
    ax1, ax2, ax3 = axes[0]
    fg = rec["force_gap"]
    ax1.plot(fg["z_mm"], fg["fz_kgf"], color=ps.COLORS["s1"], lw=1.6, label="model (4 cryostats)")
    pts = rec["atz_points"]
    ax1.scatter(pts["z_mm"], pts["fz_kgf"], color=ps.COLORS["star"], zorder=5, s=30, label="ATZ fit points")
    ax1.legend(loc="upper right", **ps.LEGEND_KW)
    ps.style_ax(ax1, "gap z (mm)", "lift (kgf)", "Force against gap")
    at = rec["area_table"]
    x = np.arange(len(at["class"]))
    ax2.bar(x - 0.18, at["hts_area_m2"], width=0.36, color=ps.COLORS["s2"], label="HTS at 53 kPa")
    ax2.bar(x + 0.18, at["ems_area_m2"], width=0.36, color=ps.COLORS["s1"], label="EMS at 0.3 MPa")
    ax2.set_xticks(x, [c.upper().replace("_", "-") for c in at["class"]])
    ax2.set_yscale("log")
    ax2.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(ax2, None, "area (m²)", "HTS needs 5–8× the EMS area")
    s = rec["xc_hts_1"]
    ax3.plot(s["t_h"], s["ln2_frac"], color=ps.COLORS["s3"], lw=1.6, label="LN2 fraction")
    ax3.plot(s["t_h"], s["lift_over_load"], color=ps.COLORS["s4"], lw=1.6, label="lift / load")
    ax3.axhline(0.10, color=ps.COLORS["star"], lw=1.0, ls=":", label="land at 10 %")
    ax3.legend(loc="upper right", **ps.LEGEND_KW)
    ps.style_ax(ax3, "time (h)", "fraction", "XC-HTS-1 LN2 depletion")
    finish(fig, title="HTS flux pinning: force, area, cryogen", question=STUDY.question, caption=rec["caption"], meta=meta(cfg, STUDY.name))
    font_safe(fig)
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the study (model sweep, area table, XC-HTS-1; ``quick()`` uses a coarser gap grid)."""
    rec = compute(cfg.seed, reduced=quick())
    rec["quick"] = quick()
    return StudyResult(fig=_figure(rec, cfg), record=rec)


STUDY = StudySpec(
    name="hts_force_gap",
    tier=1,
    question="How much does HTS flux pinning lift against gap, how much area and cryogen does a sovereign class need?",
    run=run,
)
