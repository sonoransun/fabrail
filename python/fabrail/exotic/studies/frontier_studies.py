"""🔭 Frontier studies: the models of :mod:`fabrail.exotic.frontier` as bannered study outputs.

Every output of these studies carries the frontier banner, ``"tier": "frontier"``, the
missing-orders figure and the plot watermark (FAB-EXO-004, FAB-FRN-008): the record is
the model's stamped :class:`~fabrail.exotic.frontier._record.FrontierRecord` (plus the
figure's plotted numbers), and the figure is watermarked. The physics stays in the
frontier package (FAB-EXO-002); this module only presents it. Frontier items are
theoretical what-if studies and never products.

- ``frontier_pegs``: detection time of an on-site gravity gradiometer against the P wave
  across sensitivities (``pegs.sweep``), with the 5-orders sensitivity gap.
- ``frontier_<item>`` for the other headline items (RTSC, metabarrier, structure-scale
  cloak, SELENE): the headline record and its missing-orders meter.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from ...studies.runner import StudyResult, StudySpec, finish, new_figure
from ._common import meta

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...config import RunConfig

__all__ = ["STUDIES", "frontier_record"]

QUICK_AWARE = True

#: Headline items shown as cards: (frontier item key, title).
_CARDS: tuple[tuple[str, str], ...] = (
    ("rtsc", "Ambient-pressure RTSC suspension"),
    ("metabarrier", "Building-scale active metabarrier"),
    ("cloak_structure", "Structure-scale magnetic concentrator"),
    ("selene", "SELENE lunar hatch"),
)


def frontier_record(rec: Any) -> dict[str, Any]:
    """A frontier record as JSON-ready data (keeps tier, banner, missing_orders and note)."""
    from ..frontier._record import jsonable

    return dict(jsonable(dict(rec)))


def _stamp(fig: Any, rec: dict[str, Any], cfg: RunConfig, name: str, title: str, question: str) -> None:
    from ..frontier import watermark_text
    from ..plots import font_safe, frontier_watermark

    # The caption repeats the banner without the 🔭 glyph, which the plot fonts lack.
    caption = f"{watermark_text(rec)}. {rec.get('note', '')} Frontier items are theoretical what-if studies, never products; numpy only."
    finish(fig, title=f"FRONTIER · {title}", question=question, caption=caption, meta=meta(cfg, name, "frontier numpy model"))
    frontier_watermark(fig, rec)
    font_safe(fig)


def _pegs(cfg: RunConfig) -> StudyResult:
    from ... import plotstyle as ps
    from ..frontier import pegs

    sweep = pegs.sweep()
    gap = pegs.sensitivity_gap()
    rec = frontier_record(sweep)
    asd = np.asarray(sweep["inputs"]["asd_per_rthz"], dtype=float)
    t_det = np.asarray(sweep["t_det_s"], dtype=float)
    t_p = np.asarray(sweep["t_p_site_s"], dtype=float)
    cases = [f"Mw {m:g} at {r:g} km" for m, r in sweep["inputs"]["cases_mw_r_km"]]
    fig, axes = new_figure(1, 1, size=(10.0, 5.0))
    ax = axes[0, 0]
    for j, label in enumerate(cases):
        color = ps.SERIES[j % len(ps.SERIES)]
        ax.loglog(asd, t_det[:, j], color=color, marker="o", lw=1.4, label=f"{label}: detection")
        ax.axhline(t_p[j], color=color, lw=0.8, ls=":")
    ax.axvline(gap["asd_best_per_rthz"], color=ps.COLORS["star"], lw=1.0, ls="--", label="best demonstrated ASD")
    ax.legend(loc="upper left", fontsize=8, frameon=False)
    ps.style_ax(ax, "gradiometer noise ASD (1/s²/√Hz)", "time after origin (s)", "Detection vs P arrival (dotted)")
    rec["plotted"] = {"asd_per_rthz": [float(x) for x in asd], "t_det_s": t_det.tolist(), "t_p_site_s": [float(x) for x in t_p], "asd_best_per_rthz": float(gap["asd_best_per_rthz"])}
    rec["sensitivity_gap"] = frontier_record(gap)
    _stamp(fig, rec, cfg, "frontier_pegs", "PEGS on-site gravity gradiometer", _PEGS_Q)
    return StudyResult(fig=fig, record=rec)


def _card(key: str, title: str) -> Any:
    def run(cfg: RunConfig) -> StudyResult:
        from ... import plotstyle as ps
        from ..frontier import meta as fmeta

        rec = frontier_record(fmeta.headline(key))
        mo = float(rec["missing_orders"])
        fig, axes = new_figure(1, 1, size=(9.0, 3.6))
        ax = axes[0, 0]
        ax.barh([0], [mo], color=ps.COLORS["frontier"], height=0.5)
        ax.set_xlim(0.0, max(6.0, mo + 1.0))
        ax.set_yticks([0], ["missing orders of magnitude"])
        ax.text(mo, 0, f"  {mo:g}", va="center", color=ps.COLORS["ink"])
        ps.style_ax(ax, "orders of magnitude between need and best demonstration", None, rec.get("note", ""))
        rec["plotted"] = {"missing_orders": mo}
        _stamp(fig, rec, cfg, f"frontier_{key}", title, f"How far is this frontier item ({title}) from its nearest real anchor?")
        return StudyResult(fig=fig, record=rec)

    return run


_PEGS_Q = "Could an on-site gravity gradiometer beat the P wave, and how far is the sensor from that?"

#: Every frontier study (tier 1: closed forms, under a second each).
STUDIES: tuple[StudySpec, ...] = (StudySpec(name="frontier_pegs", tier=1, question=_PEGS_Q, run=_pegs),) + tuple(
    StudySpec(name=f"frontier_{key}", tier=1, question=f"How far is this frontier item ({title}) from its nearest real anchor?", run=_card(key, title))
    for key, title in _CARDS
)
