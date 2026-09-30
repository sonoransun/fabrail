"""Study ``ride_through`` (tier 1): how much stored energy does riding through a long earthquake take?

Model (EXOTIC §X4, §X5; FAB-EXO-120, FAB-EXO-122, FAB-EXO-401, FAB-EXO-403):

- T_ride = max(300 s, t_shaking) + t_land (``fabrail_exo::energy``); E = P_hold·T_ride with
  the hybrid-EMS hover loss P_W·(a_rms/2g)² (P_W = 1.7 kW/t), pure EMS P_W, HTS kW-scale.
- 10 kt anchors: hybrid 0.2 g RMS, 51 MJ (102 MJ with the 2× margin), within one
  EMALS-class disk (121 MJ per cycle, ≈ 357 MJ stored); pure EMS 5.1 GJ, flagged
  insufficient against one JET-class flywheel (≈ 2.8 GJ usable).
- Absorption: charge headroom + dump ≥ 1.2·KE_max; ROTATOR fails against an empty 2 kWh
  buffer with no dump (1.05×) and is reported, not hidden.
- XS06 (grid loss at S) on the HAVEN-R reduced model.

Budget buys storage, not reliability growth (EMALS: 614 against a 4,166-cycle requirement).

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

#: ROTATOR regeneration buffer, J (2 kWh, the exotic design §1.3 lower bound; empty, no dump).
ROTATOR_BUFFER_J = 2.0 * 3.6e6


def compute(seed: int = 42, reduced: bool = False) -> dict[str, Any]:
    """Every number of the study (the record), without plotting."""
    from .. import energy
    from ..presets import preset
    from ..scenarios import simulate

    table = energy.ride_through_table(1.0e7, 300.0)
    labels = [f"hybrid {r.a_rms_g:g} g" if r.suspension == "hybrid" else r.suspension for r in table]
    cls = energy.class_ride_through()
    shaking = np.round(np.arange(0.0, 900.0 + 1e-9, 60.0 if reduced else 15.0), 6)
    t_ride = [energy.t_ride_s(float(s), 30.0) for s in shaking]
    rot = preset("rotator")
    absorb = energy.absorption_check(rot.design_mass_kg, rot.v_max_mps, ROTATOR_BUFFER_J)
    xs06 = simulate("haven_r", "XS06", seed)
    hybrid02 = table[1]
    pure = table[3]
    return {
        "inputs": {"m_kg": 1.0e7, "t_shaking_s": 300.0, "margin": energy.DISCHARGE_MARGIN, "t_land_s_for_curve": 30.0, "seed": seed},
        "cases": {
            "case": labels,
            "p_hold_w": arr([r.p_hold_w for r in table]),
            "e_ride_j": arr([r.e_ride_j for r in table]),
            "e_with_margin_j": arr([r.e_with_margin_j for r in table]),
            "verdict": [r.verdict for r in table],
            "emals_delivered_j": [energy.EMALS_DELIVERED_J],
            "jet_usable_j": [energy.jet_usable_j()],
        },
        "storage": {"emals_stored_j": energy.emals_stored_j(), "jet_stored_j": energy.JET_STORED_J, "flywheel_usable_at_half": energy.flywheel_usable_fraction(0.5)},
        "per_class": {"class": [c["class"] for c in cls], "e_with_margin_j": arr([c["e_with_margin_j"] for c in cls]), "suspension": [c["suspension"] for c in cls]},
        "t_ride_curve": {"t_shaking_s": arr(shaking), "t_ride_s": arr(t_ride)},
        "rotator_absorption": absorb,
        "xs06": {"kpis": xs06.kpis, "passed": xs06.passed, "inputs": xs06.inputs},
        "checks": {
            "hybrid_10kt_51mj": check(abs(hybrid02.e_ride_j - 51e6) <= 1.0, hybrid02.e_ride_j, 51e6, "FAB-EXO-403 (exact)"),
            "pure_10kt_5p1gj_insufficient": check(abs(pure.e_ride_j - 5.1e9) <= 1.0 and "insufficient" in pure.verdict, pure.e_ride_j, 5.1e9, "FAB-EXO-403"),
            "rotator_absorption_fails_reported": check(not absorb["pass"], absorb["ratio"], 1.2, "FAB-EXO-122 (reported, not hidden)"),
        },
        "caption": (
            "T_ride = max(300 s, shaking) + landing; hybrid-EMS hold power P_W·(a_rms/2g)²; storage benchmarks EMALS-class "
            "(121 MJ per cycle) and JET-class (3.75 GJ stored). ROTATOR fails the absorption check against a 2 kWh buffer "
            "and is reported. Budget does not buy reliability growth. " + NON_CLAIMS
        ),
    }


def _figure(rec: dict[str, Any], cfg: RunConfig) -> Any:
    from ... import plotstyle as ps
    from ..plots import font_safe
    from ..plots import class_color

    fig, axes = new_figure(1, 2, size=(12.5, 5.0))
    ax1, ax2 = axes[0]
    c = rec["cases"]
    x = np.arange(len(c["case"]))
    ax1.bar(x, c["e_with_margin_j"], color=ps.COLORS["s1"], width=0.6, label="E_ride × 2 margin (10 kt, 300 s)")
    ax1.axhline(c["emals_delivered_j"][0], color=ps.COLORS["s3"], lw=1.2, ls="--", label="EMALS-class disk, per cycle")
    ax1.axhline(c["jet_usable_j"][0], color=ps.COLORS["s2"], lw=1.2, ls="--", label="JET-class flywheel, usable")
    ax1.set_xticks(x, c["case"])
    ax1.set_yscale("log")
    ax1.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(ax1, None, "energy (J)", "10 kt ride-through")
    pc = rec["per_class"]
    x2 = np.arange(len(pc["class"]))
    ax2.bar(x2, pc["e_with_margin_j"], color=[class_color(n) for n in pc["class"]], width=0.6)
    ax2.set_xticks(x2, [n.upper().replace("_", "-") for n in pc["class"]], rotation=30)
    ax2.set_yscale("log")
    ps.style_ax(ax2, None, "E_ride × 2 (J)", "Per class at 0.2 g RMS, 300 s")
    finish(fig, title="Ride-through energy", question=STUDY.question, caption=rec["caption"], meta=meta(cfg, STUDY.name))
    font_safe(fig)
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the study (closed forms plus XS06)."""
    rec = compute(cfg.seed, reduced=quick())
    rec["quick"] = quick()
    return StudyResult(fig=_figure(rec, cfg), record=rec)


STUDY = StudySpec(
    name="ride_through",
    tier=1,
    question="How much stored energy does riding through 300 s of shaking take per class, and which stores cover it?",
    run=run,
)
