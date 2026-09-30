"""Figures of the exotic tier in the house style (:mod:`fabrail.plotstyle`).

- :func:`plot_run`: every recorded channel of an exotic scenario run against time, with
  the verdict of each pass criterion, the reduced-model label and the run stamp.
- :func:`plot_admission`, :func:`plot_pfh`, :func:`plot_ride_through` and
  :func:`plot_lead_time`: the algebra of the tier (EXOTIC §X2, §X13, §X4, §X10).
- :func:`frontier_watermark`: the 🔭 watermark of a frontier output (FAB-EXO-004).

Figures are built with :class:`matplotlib.figure.Figure` (no pyplot, no GUI backend).
Every plotted number is also returned by the data functions the studies record
(FAB-SW-009 rule).

Owner: X7 (Wave 2).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

__all__ = [
    "class_color",
    "font_safe",
    "frontier_watermark",
    "new_figure",
    "plot_admission",
    "plot_lead_time",
    "plot_pfh",
    "plot_ride_through",
    "plot_run",
]

_EXO_ORDER = ("sentinel", "haven_r", "haven_f", "shield", "bastion", "arena", "rotator", "tidegate", "selene")


def new_figure(nrows: int = 1, ncols: int = 1, size: tuple[float, float] = (11.0, 4.6), **kw: Any) -> tuple[Any, np.ndarray]:
    """A house-styled Figure and its 2-D axes grid (no pyplot)."""
    from matplotlib.figure import Figure

    from .. import plotstyle

    plotstyle.apply()
    fig = Figure(figsize=size)
    axes = fig.subplots(nrows, ncols, squeeze=False, **kw)
    return fig, axes


def font_safe(fig: Any) -> Any:
    """Set every text of ``fig`` through :func:`fabrail.studies.runner.tex_safe` (glyphs the house serif lacks).

    STIX Two Text has no ≥ ≤ → ≈ √ ∫ ∞ ≠ ∑; they are drawn as mathtext instead of a box.
    Returns ``fig``.
    """
    from matplotlib.text import Text

    from ..studies.runner import tex_safe

    for t in fig.findobj(Text):
        s = t.get_text()
        if s:
            fixed = tex_safe(s)
            if fixed != s:
                t.set_text(fixed)
    return fig


def class_color(name: str) -> str:
    """A fixed colour per exotic class (the five categorical series, cycled in EXOTIC §1 order)."""
    from .. import plotstyle

    try:
        i = _EXO_ORDER.index(name)
    except ValueError:
        return plotstyle.COLORS["s5"]
    return plotstyle.SERIES[i % len(plotstyle.SERIES)]


def frontier_watermark(fig: Any, record: Mapping[str, Any]) -> str:
    """Stamp ``fig`` with the frontier watermark of a stamped frontier record (FAB-EXO-004); returns the text."""
    from .. import plotstyle
    from .frontier import watermark_text

    text = watermark_text(record)
    plotstyle.watermark(fig, text)
    return text


def plot_run(run: Any, max_panels: int = 6) -> Any:
    """Every channel of an :class:`~fabrail.exotic.scenarios.ExoRun` against time (channels must be recorded).

    The title carries the scenario, class and verdict; a reduced-model run says so; the
    caption lists each criterion's value and limit. Returns the figure.
    """
    from .. import plotstyle as ps

    names = [n for n in run.channels if n != "t_s"]
    if run.t_s.size == 0 or not names:
        fig, axes = new_figure(1, 1, size=(10.0, 3.2))
        ax = axes[0, 0]
        crit = [c for c in run.criteria if c.value is not None]
        ax.barh([c.name for c in crit], [c.value for c in crit], color=[ps.COLORS["s3"] if c.passed else ps.COLORS["s2"] for c in crit])
        lim = [c.limit for c in crit]
        ax.scatter([x if x is not None else np.nan for x in lim], [c.name for c in crit], color=ps.COLORS["star"], zorder=5, s=24, label="limit")
        ps.style_ax(ax, "value (criterion units)", None, f"{run.scenario} · {run.rail_class}: criteria (no time series recorded)")
        ax.legend(loc="lower right", **ps.LEGEND_KW)
    else:
        names = names[:max_panels]
        fig, axes = new_figure(len(names), 1, size=(10.5, 1.6 + 1.55 * len(names)), sharex=True)
        t = run.t_s
        for i, name in enumerate(names):
            ax = axes[i, 0]
            ax.plot(t, run.channels[name], color=ps.SERIES[i % len(ps.SERIES)], lw=1.2)
            unit = run.units.get(name, "")
            ps.style_ax(ax, None, f"{name}\n({unit})" if unit else name)
        ps.style_ax(axes[-1, 0], "time (s)", axes[-1, 0].get_ylabel())
    verdict = "PASS" if run.passed else "FAIL"
    label = " · reduced model" if run.reduced_model else ""
    fig.suptitle(f"{run.scenario} · {run.rail_class.upper()} · {verdict}{label}", x=0.01, ha="left", fontsize=ps.FONT["suptitle"], color=ps.COLORS["ink"])
    fig.tight_layout(rect=(0, 0.04, 1, 0.97))
    ps.run_stamp(fig, {"scenario": run.scenario, "class": run.rail_class, "seed": run.seed})
    return font_safe(fig)


def plot_admission(adm: Mapping[str, Sequence[Any]]) -> Any:
    """Pole area per class from the single admission function, with the HTS alternative (EXOTIC §X2, §X3).

    ``adm`` keys: ``class``, ``area_m2`` and optionally ``hts_class``, ``hts_area_m2``.
    """
    from .. import plotstyle as ps

    fig, axes = new_figure(1, 1, size=(10.0, 4.4))
    ax = axes[0, 0]
    names = list(adm["class"])
    y = np.arange(len(names))
    ax.barh(y, adm["area_m2"], color=[class_color(n) for n in names], height=0.6, label="EMS admission area (k 1.5, η 0.45, B 1.2 T)")
    if adm.get("hts_class"):
        idx = [names.index(n) for n in adm["hts_class"]]
        ax.scatter(adm["hts_area_m2"], idx, marker="D", color=ps.COLORS["star"], zorder=5, s=28, label="HTS bulk area at 10 mm (53 kPa)")
    ax.set_yticks(y, [n.upper().replace("_", "-") for n in names])
    ax.set_xscale("log")
    ax.invert_yaxis()
    ax.legend(loc="lower right", **ps.LEGEND_KW)
    ps.style_ax(ax, "pole area (m²)", None, "Pole area from one admission function")
    return font_safe(fig)


def plot_pfh(sweep: Mapping[str, Sequence[float]]) -> Any:
    """2oo3 PFH against β_D, both forms, with the numeric SIL band edges (EXOTIC §X13)."""
    from .. import plotstyle as ps

    fig, axes = new_figure(1, 1, size=(8.5, 4.4))
    ax = axes[0, 0]
    bd = np.asarray(sweep["beta_d"], dtype=float) * 100.0
    ax.plot(bd, sweep["conservative_per_h"], color=ps.COLORS["s1"], marker="o", lw=1.6, label="conservative form")
    ax.plot(bd, sweep["ed2_per_h"], color=ps.COLORS["s2"], marker="s", lw=1.6, label="IEC 61508-6 Ed. 2 form")
    for edge in sweep.get("band_edges_per_h", ()):
        ax.axhline(edge, color=ps.COLORS["muted"], lw=0.8, ls=":")
    ax.set_yscale("log")
    ax.legend(loc="best", **ps.LEGEND_KW)
    ps.style_ax(ax, "β_D (%)", "PFH (1/h)", "Common cause sets the floor")
    return font_safe(fig)


def plot_ride_through(rt: Mapping[str, Sequence[Any]]) -> Any:
    """Ride-through energy (with the 2× margin) per case against the storage benchmarks (EXOTIC §X4)."""
    from .. import plotstyle as ps

    fig, axes = new_figure(1, 1, size=(10.0, 4.4))
    ax = axes[0, 0]
    x = np.arange(len(rt["case"]))
    ax.bar(x, rt["e_with_margin_j"], color=ps.COLORS["s1"], width=0.6, label="E_ride × 2 margin")
    for key, label, color in (("emals_delivered_j", "EMALS-class disk, per cycle", "s3"), ("jet_usable_j", "JET-class flywheel, usable", "s2")):
        if key in rt:
            ax.axhline(float(np.asarray(rt[key]).ravel()[0]), color=ps.COLORS[color], lw=1.2, ls="--", label=label)
    ax.set_xticks(x, list(rt["case"]), rotation=0)
    ax.set_yscale("log")
    ax.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(ax, None, "energy (J)", "Ride-through energy against storage")
    return font_safe(fig)


def plot_lead_time(lt: Mapping[str, Sequence[float]]) -> Any:
    """Warning time against sensor advance, and the offshore worked case (EXOTIC §X10)."""
    from .. import plotstyle as ps

    fig, axes = new_figure(1, 2, size=(11.0, 4.4))
    ax1, ax2 = axes[0]
    ax1.plot(lt["advance_km"], lt["warning_s"], color=ps.COLORS["s1"], marker="o", lw=1.6, label="warning at the site")
    ax1.plot(lt["advance_km"], lt["warning_gain_line_s"], color=ps.COLORS["muted"], lw=1.0, ls="--", label="0.154 s/km")
    ps.zero_line(ax1)
    ax1.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(ax1, "sensor advance toward the epicentre (km)", "warning before S (s)", "Moving sensors toward the fault")
    y = np.arange(len(lt["offshore_label"]))
    ax2.barh(y, lt["offshore_warning_s"], color=[ps.SERIES[i] for i in range(len(y))], height=0.55)
    ax2.set_yticks(y, list(lt["offshore_label"]))
    ax2.invert_yaxis()
    ps.style_ax(ax2, "warning at the city (s)", None, "Offshore M8, 3.3 s latency")
    return font_safe(fig)
