"""House plotting style: one design system for every matplotlib figure.

The palette is the house palette shared with the sibling projects (vacuumprobe
``plotstyle.py``) and mirrored by the site: every ``--ps-<name>`` token in the
``PLOTSTYLE`` block of docs/assets/css/site.css equals ``COLORS["<name>"]``
(tests/test_site.py::test_css_tokens_match_plotstyle, FAB-SW-013). The rail classes
keep fixed colours (``CLASS_COLORS``), matching the site's ``--class-*`` tokens.

Call :func:`apply` once before building figures, :func:`style_ax` per axes, and
:func:`run_stamp` per figure. matplotlib is imported only inside the functions, so
reading the tokens is cheap. :func:`css_palette` reads the site's ``PLOTSTYLE`` block
and :func:`palette_drift` lists every token that differs from :data:`COLORS`, so the
site and docs drift tests share one parser.

Owner: W0-C2 (complete), then A8 (Wave 1).
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any

__all__ = [
    "CLASS_COLORS",
    "CLASS_LABELS",
    "CLASS_MARKERS",
    "COLORS",
    "FONT",
    "GRID_KW",
    "LEGEND_KW",
    "SERIES",
    "WORDMARK",
    "apply",
    "brand",
    "css_palette",
    "palette_drift",
    "ref_line",
    "run_stamp",
    "save",
    "style_ax",
    "watermark",
    "zero_line",
]

# ── Design tokens (light theme; PNGs render on paper and README surfaces) ──────
COLORS: dict[str, str] = {
    # ink and chrome
    "ink": "#14161c",  # primary text
    "ink2": "#4d5462",  # secondary text, axis labels
    "muted": "#848c9e",  # tick labels, de-emphasised annotations
    "grid": "#e6e5de",  # hairline gridlines
    "axis": "#c2c1b8",  # spines, baselines
    "surface": "#fcfcfa",  # axes background
    "page": "#f7f7f4",  # figure background
    # categorical series (validated order: assign by entity, never cycle)
    "s1": "#2a78d6",  # blue
    "s2": "#eb6834",  # orange
    "s3": "#1baf7a",  # aqua
    "s4": "#eda100",  # yellow
    "s5": "#e87ba4",  # magenta
    # reserved roles
    "star": "#d03b3b",  # reference/anchor data points only (closed forms, standard limits)
    "accent": "#1c63c7",  # emphasis rules and arrows, never a data series
    "good": "#0ca30c",  # validation-pass accents
    "ctrl": "#7d8598",  # control/baseline channels (open markers), e.g. controller off
    "band": "#2a78d6",  # uncertainty bands (use alpha ≈ 0.12)
    "frontier": "#6b4fd8",  # 🔭 frontier banners and watermarks (site --frontier, light theme)
}

SERIES: list[str] = [COLORS["s1"], COLORS["s2"], COLORS["s3"], COLORS["s4"], COLORS["s5"]]

#: Fixed class identities across every figure (site --class-* tokens).
CLASS_COLORS: dict[str, str] = {
    "drawer": COLORS["s1"],
    "door": COLORS["s2"],
    "wall": COLORS["s3"],
    "heavy": COLORS["s4"],
    "exotic": COLORS["s5"],
}
CLASS_MARKERS: dict[str, str] = {"drawer": "o", "door": "s", "wall": "^", "heavy": "D", "exotic": "P"}
CLASS_LABELS: dict[str, str] = {"drawer": "DRAWER", "door": "DOOR", "wall": "WALL", "heavy": "HEAVY", "exotic": "EXOTIC"}

WORDMARK = "F A B R A I L"

_SERIF = ["STIX Two Text", "STIXGeneral", "DejaVu Serif"]
_MONO = ["DejaVu Sans Mono", "Menlo", "monospace"]

FONT: dict[str, float] = {"label": 11.5, "tick": 10, "legend": 10, "title": 13, "suptitle": 15}
LEGEND_KW: dict[str, Any] = {
    "fontsize": FONT["legend"],
    "frameon": False,
    "labelcolor": COLORS["ink2"],
    "handlelength": 1.6,
    "handletextpad": 0.6,
    "borderaxespad": 0.4,
}
GRID_KW: dict[str, Any] = {"color": COLORS["grid"], "linestyle": "-", "linewidth": 0.8, "alpha": 1.0}


def apply() -> None:
    """Install the house rcParams (idempotent; call before building figures)."""
    import matplotlib as mpl
    from cycler import cycler

    mpl.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": _SERIF,
            "font.monospace": _MONO,
            "mathtext.fontset": "stix",
            "text.color": COLORS["ink"],
            "figure.facecolor": COLORS["page"],
            "figure.dpi": 110,
            "savefig.dpi": 200,
            "savefig.facecolor": COLORS["page"],
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.18,
            "axes.facecolor": COLORS["surface"],
            "axes.edgecolor": COLORS["axis"],
            "axes.linewidth": 1.0,
            "axes.labelcolor": COLORS["ink2"],
            "axes.labelsize": FONT["label"],
            "axes.titlesize": FONT["title"],
            "axes.titleweight": "semibold",
            "axes.titlecolor": COLORS["ink"],
            "axes.titlepad": 10,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.grid": False,
            "axes.axisbelow": True,
            "axes.prop_cycle": cycler(color=SERIES),
            "xtick.color": COLORS["axis"],
            "ytick.color": COLORS["axis"],
            "xtick.labelcolor": COLORS["muted"],
            "ytick.labelcolor": COLORS["muted"],
            "xtick.labelsize": FONT["tick"],
            "ytick.labelsize": FONT["tick"],
            "xtick.major.size": 3.5,
            "ytick.major.size": 3.5,
            "legend.frameon": False,
            "legend.fontsize": FONT["legend"],
            "figure.titlesize": FONT["suptitle"],
            "figure.titleweight": "semibold",
            "lines.linewidth": 2.0,
            "lines.markersize": 6.5,
            "errorbar.capsize": 2.5,
            "hatch.linewidth": 0.7,
        }
    )


def style_ax(ax: Any, xlabel: str | None = None, ylabel: str | None = None, title: str | None = None, grid: bool = True) -> None:
    """Per-axes finishing: labels, left-aligned title, y hairline grid."""
    if xlabel:
        ax.set_xlabel(xlabel, fontsize=FONT["label"], color=COLORS["ink2"])
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=FONT["label"], color=COLORS["ink2"])
    if title:
        ax.set_title(title, fontsize=FONT["title"], color=COLORS["ink"], fontweight="semibold", loc="left")
    ax.tick_params(labelsize=FONT["tick"])
    if grid:
        ax.grid(axis="y", **GRID_KW)
        ax.set_axisbelow(True)


def zero_line(ax: Any, y: float = 0.0) -> None:
    """The reference baseline, drawn as a solid axis-tone rule."""
    ax.axhline(y, color=COLORS["axis"], linewidth=1.0, zorder=1)


def ref_line(ax: Any, y: float, label: str | None = None, color: str | None = None, x: float = 0.985, va: str = "bottom") -> None:
    """A dashed reference level (a limit or closed form) with a small right-aligned label."""
    c = color or COLORS["muted"]
    ax.axhline(y, color=c, linestyle=(0, (4, 4)), linewidth=1.2, zorder=2)
    if label:
        ax.annotate(
            label,
            xy=(x, y),
            xycoords=("axes fraction", "data"),
            ha="right",
            va=va,
            fontsize=8.5,
            color=c,
            xytext=(0, 2 if va == "bottom" else -2),
            textcoords="offset points",
        )


def brand(fig: Any, note: str | None = None) -> None:
    """Wordmark (bottom right) and an optional note (bottom left)."""
    fig.text(0.99, 0.005, WORDMARK, ha="right", va="bottom", fontsize=6.5, family="monospace", color=COLORS["muted"])
    if note:
        fig.text(0.01, 0.005, note, ha="left", va="bottom", fontsize=6.5, family="monospace", color=COLORS["muted"])


def _stamp_text(meta: Mapping[str, Any] | None) -> str:
    from . import __version__

    meta = dict(meta or {})
    parts = [f"fabrail {meta.get('version', __version__)}"]
    scenario = meta.get("scenario")
    if isinstance(scenario, Mapping):
        scenario = scenario.get("name")
    rail_class = meta.get("rail_class") or meta.get("class")
    if rail_class and scenario:
        parts.append(f"{rail_class}/{scenario}")
    elif scenario or rail_class:
        parts.append(str(scenario or rail_class))
    seed = meta.get("seed", meta.get("noise_seed"))
    if seed is not None:
        parts.append(f"seed {seed}")
    fidelity, coil = meta.get("fidelity"), meta.get("coil")
    if fidelity or coil:
        parts.append("/".join(str(x) for x in (fidelity, coil) if x))
    golden = meta.get("golden")
    if isinstance(golden, int):
        parts.append(f"golden {golden:#018x}")
    elif golden:
        parts.append(f"golden {golden}")
    profile = meta.get("build_profile")
    if profile and profile != "release":
        parts.append(f"{profile} build")
    return " · ".join(parts)


def run_stamp(fig: Any, meta: Mapping[str, Any] | None = None) -> str:
    """Stamp ``fig`` with the wordmark and a run line (version, class/scenario, seed, tier, golden hash).

    ``meta`` is a run's metadata (``RunResult.meta``: version, scenario, noise_seed or
    seed, fidelity, coil, golden, build_profile) or any mapping with some of those keys.
    Returns the stamp text.
    """
    text = _stamp_text(meta)
    brand(fig, text)
    return text


def watermark(fig: Any, text: str, color: str | None = None, alpha: float = 0.16) -> None:
    """A large diagonal watermark across ``fig`` (frontier outputs use the 🔭 banner text)."""
    fig.text(
        0.5,
        0.5,
        text,
        ha="center",
        va="center",
        rotation=24,
        fontsize=28,
        color=color or COLORS["frontier"],
        alpha=alpha,
        family="monospace",
        zorder=0,
    )


def save(fig: Any, path: str | Path | None) -> Path | None:
    """Save ``fig`` to ``path`` (parents created); no-op when ``path`` is None."""
    if not path:
        return None
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    return out


# ── Site drift (docs/assets/css/site.css PLOTSTYLE block) ─────────────────────

_PLOTSTYLE_BLOCK = re.compile(r"/\*\s*PLOTSTYLE:BEGIN.*?PLOTSTYLE:END\s*\*/", re.S)
_PS_TOKEN = re.compile(r"--ps-([a-z0-9]+)\s*:\s*([^;]+);")


def css_palette(css_text: str) -> dict[str, str]:
    """The ``--ps-<name>: <value>`` tokens of the ``PLOTSTYLE:BEGIN … PLOTSTYLE:END`` block.

    Values are lower-cased and stripped. Raises ``ValueError`` when the block is absent.
    """
    block = _PLOTSTYLE_BLOCK.search(css_text)
    if block is None:
        raise ValueError("site.css: no /* PLOTSTYLE:BEGIN … PLOTSTYLE:END */ block")
    return {name: value.strip().lower() for name, value in _PS_TOKEN.findall(block.group(0))}


def palette_drift(css_text: str) -> list[str]:
    """Every ``--ps-*`` token that is missing from :data:`COLORS` or differs from it (empty = in sync)."""
    drift = []
    for name, value in sorted(css_palette(css_text).items()):
        want = COLORS.get(name)
        if want is None:
            drift.append(f"--ps-{name}: {value} has no plotstyle.COLORS entry")
        elif want.lower() != value:
            drift.append(f"--ps-{name}: {value} != plotstyle.COLORS[{name!r}] {want}")
    return drift
