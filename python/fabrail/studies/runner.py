"""Study infrastructure (core §B.8, §C.6): specs, discovery, the runner and shared helpers.

A study is a module in :mod:`fabrail.studies` with a module-level ``STUDY``::

    # fabrail/studies/pole_map.py
    from .runner import StudyResult, StudySpec

    def run(cfg):                       # cfg: fabrail.config.RunConfig
        fig = ...                       # matplotlib Figure (or None)
        return StudyResult(fig=fig, record={"pole_rad_s": {...}})

    STUDY = StudySpec(name="pole_map", tier=1,
                      question="Where are the open- and closed-loop poles per class?", run=run)

:func:`discover` finds every such module with :mod:`pkgutil`; :func:`run_study` runs
one and writes ``<output>/studies/<name>.png`` and ``<output>/studies/<name>.json``
(schema ``fabrail.study/1``). Tier 1 runs in under 30 s each; tier 2 is longer.

Reduced settings. :func:`run_study` and :func:`execute` take ``quick=True``; inside
the call :func:`quick` returns True and a study shortens its grids and runs (the
smoke tests in ``tests/test_studies.py`` use it; the record carries ``"quick": true``).
A study module that honours :func:`quick` sets the module constant ``QUICK_AWARE =
True``; the smoke test runs those in the non-slow tier and every other tier-1 study in
the ``slow`` tier.

Record contract (FAB-SW-009): every number a study plots is in its record, in the
plotted unit (keys carry the unit suffix, e.g. ``gap_mm``). :func:`missing_numbers`
lists plotted values the record lacks (``tests/test_studies.py::json_records_numbers``).

Shared helpers for the study modules (all additive to the Wave-0 surface):
:func:`class_cfg` (a per-class copy of the run config in voltage coil mode, the only
mode SIL ≡ firmware is claimed for), :func:`run_jobs` (a parallel ``_native.run_batch``
decoded into :class:`fabrail.results.RunResult`), :func:`new_figure` /
:func:`finish` (house style, title, question, an honest caption and a run stamp),
:func:`fault_name` and :func:`floats`.

Owner: W0-C2 (infrastructure), then B2 (Wave 2); studies by B2/B3.
"""

from __future__ import annotations

import contextlib
import dataclasses
import importlib
import math
import pkgutil
import textwrap
from collections.abc import Callable, Iterator, Mapping, Sequence
from contextvars import ContextVar
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..config import RunConfig
    from ..results import RunResult

__all__ = [
    "CLASSES",
    "StudyResult",
    "StudySpec",
    "class_cfg",
    "discover",
    "execute",
    "fault_name",
    "finish",
    "floats",
    "missing_numbers",
    "new_figure",
    "quick",
    "quick_mode",
    "run_jobs",
    "run_study",
    "stamp_meta",
    "tex_safe",
    "write_result",
]

_NOT_STUDIES = frozenset({"runner"})

#: The four core rail classes in their fixed order (colours follow the class).
CLASSES: tuple[str, ...] = ("drawer", "door", "wall", "heavy")

_QUICK: ContextVar[bool] = ContextVar("fabrail_study_quick", default=False)


@dataclass(frozen=True)
class StudyResult:
    """What a study returns: a figure (or None) and a JSON-ready record of every plotted number."""

    fig: Any = None
    record: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class StudySpec:
    """One study: its name, tier (1 or 2), the question it answers and ``run(cfg) -> StudyResult``."""

    name: str
    tier: int
    question: str
    run: Callable[[RunConfig], StudyResult]

    def __post_init__(self) -> None:
        if self.tier not in (1, 2):
            raise ValueError(f"study {self.name!r}: tier must be 1 or 2 (got {self.tier!r})")
        if not callable(self.run):
            raise ValueError(f"study {self.name!r}: run must be callable")


def discover() -> dict[str, StudySpec]:
    """Every study module's ``STUDY``, keyed by name, ordered by (tier, name)."""
    from .. import studies as pkg

    found: dict[str, StudySpec] = {}
    for info in pkgutil.iter_modules(pkg.__path__):
        if info.name.startswith("_") or info.name in _NOT_STUDIES:
            continue
        module = importlib.import_module(f"{pkg.__name__}.{info.name}")
        spec = getattr(module, "STUDY", None)
        if not isinstance(spec, StudySpec):
            continue
        if spec.name in found:
            raise ValueError(f"duplicate study name {spec.name!r} ({info.name})")
        found[spec.name] = spec
    return dict(sorted(found.items(), key=lambda kv: (kv[1].tier, kv[0])))


def quick_aware(name: str) -> bool:
    """True when study ``name``'s module declares ``QUICK_AWARE = True`` (it honours :func:`quick`)."""
    spec = discover()[name]
    module = importlib.import_module(spec.run.__module__)
    return bool(getattr(module, "QUICK_AWARE", False))


# ---- reduced settings --------------------------------------------------------------------


def quick() -> bool:
    """True inside a ``quick=True`` study run (reduced grids and durations; smoke tests)."""
    return _QUICK.get()


@contextlib.contextmanager
def quick_mode(on: bool = True) -> Iterator[None]:
    """Context in which :func:`quick` returns ``on``."""
    token = _QUICK.set(bool(on))
    try:
        yield
    finally:
        _QUICK.reset(token)


# ---- running and writing -----------------------------------------------------------------


def _spec_of(name_or_spec: str | StudySpec) -> StudySpec:
    if isinstance(name_or_spec, StudySpec):
        return name_or_spec
    studies = discover()
    if name_or_spec not in studies:
        known = ", ".join(studies) or "none registered yet"
        raise KeyError(f"unknown study {name_or_spec!r} (available: {known})")
    return studies[name_or_spec]


def execute(name_or_spec: str | StudySpec, cfg: RunConfig, *, quick: bool = False) -> StudyResult:
    """Run one study and return its :class:`StudyResult` without writing anything.

    ``quick=True`` runs it with reduced settings (:func:`quick`). The figure stays open;
    the caller closes it (or passes the result to :func:`write_result`, which does).
    """
    spec = _spec_of(name_or_spec)
    with quick_mode(quick):
        result = spec.run(cfg)
    if not isinstance(result, StudyResult):
        raise TypeError(f"study {spec.name!r}: run() must return a StudyResult (got {type(result).__name__})")
    return result


def write_result(spec: StudySpec, result: StudyResult, cfg: RunConfig) -> list[Path]:
    """Write ``result`` under ``cfg.output / "studies"`` and close its figure.

    Returns the written paths: ``<name>.png`` (only when there is a figure and
    ``cfg.plots`` is set) and ``<name>.json`` (``fabrail.study/1``).
    """
    from ..export import study_record, write_json

    out_dir = Path(cfg.output) / "studies"
    out_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    if result.fig is not None and cfg.plots:
        png = out_dir / f"{spec.name}.png"
        result.fig.savefig(png)
        written.append(png)
    if result.fig is not None:
        try:
            import matplotlib.pyplot as plt

            plt.close(result.fig)
        except ImportError:  # pragma: no cover - matplotlib is a hard dependency
            pass
    path = out_dir / f"{spec.name}.json"
    write_json(path, study_record(spec.name, spec.tier, spec.question, result.record, cfg))
    written.append(path)
    return written


def run_study(name: str, cfg: RunConfig, *, quick: bool = False) -> list[Path]:
    """Run study ``name`` and write its outputs under ``cfg.output / "studies"``.

    Returns the written paths (the PNG only when the study made a figure and
    ``cfg.plots`` is set). ``quick=True`` uses reduced settings (:func:`quick`).
    """
    spec = _spec_of(name)
    return write_result(spec, execute(spec, cfg, quick=quick), cfg)


# ---- helpers for study modules -----------------------------------------------------------


def class_cfg(cfg: RunConfig, rail_class: str, **changes: Any) -> RunConfig:
    """A copy of ``cfg`` for ``rail_class`` in voltage coil mode (plan §2b item 16).

    The seed, workers, ``--set`` overrides and ``--plant`` keys are kept (they apply to
    every class a study runs, as a sensitivity knob); ``changes`` replace fields
    (``duration_s``, ``record_rate_hz``, ``overrides`` …). The scenario duration and the
    recording rate are the study's design, so a CLI ``--duration`` or ``--record-rate``
    does not leak into them. Voltage mode is the only coil mode for which SIL ≡
    firmware is claimed, so every closed-loop study run uses it.
    """
    changes.setdefault("coil", "voltage")
    changes.setdefault("fidelity", "carriage6")
    changes.setdefault("duration_s", None)
    changes.setdefault("record_rate_hz", 1000.0)
    return dataclasses.replace(cfg, rail_class=rail_class, **changes)


def run_jobs(jobs: Sequence[tuple[Any, Mapping[str, Any], Mapping[str, bytes]]], cfg: RunConfig) -> list[RunResult]:
    """Run ``(params, spec_dict, series_bytes)`` jobs with ``_native.run_batch`` (``cfg.workers``).

    Results come back in job order and do not depend on the worker count (the native
    batch is deterministic per job).
    """
    from .. import require_native
    from ..results import RunResult
    from ..scenarios import to_json

    n = require_native()
    batch = [(p, n.Scenario.from_json(to_json(d), dict(s))) for p, d, s in jobs]
    if not batch:
        return []
    return [RunResult.from_native(r) for r in n.run_batch(batch, cfg.workers)]


def floats(values: Any, digits: int | None = None) -> list[float]:
    """``values`` as a flat list of Python floats (non-finite become None in JSON)."""
    arr = np.asarray(values, dtype=np.float64).ravel()
    if digits is not None:
        arr = np.round(arr, digits)
    return [float(v) for v in arr]


def missing_numbers(fig: Any, record: Any, rtol: float = 1e-9) -> list[str]:
    """Plotted values of ``fig`` that ``record`` lacks (empty = every plotted number is recorded)."""
    from ..export import missing_plotted

    return missing_plotted(fig, record, rtol=rtol, atol=0.0)


#: Wire value → name of the ``fabrail_types::FaultCode`` registry (crates/fabrail-types/src/fault.rs),
#: used only to name the codes a study reports; the Rust registry is the source.
_FAULT_NAMES: dict[int, str] = {
    0x0101: "GapRange",
    0x0102: "GapStuck",
    0x0103: "GapDisagree",
    0x0110: "EncoderLost",
    0x0111: "EncoderJump",
    0x0120: "AccelLost",
    0x0121: "AccelStuck",
    0x0122: "ImuLost",
    0x0123: "RailAccelLost",
    0x0130: "CurrentSense",
    0x0201: "CoilOvercurrent",
    0x0202: "CoilOpen",
    0x0203: "CoilDriver",
    0x0210: "LsmOvercurrent",
    0x0211: "LsmPhaseSum",
    0x0212: "LsmDriver",
    0x0220: "LockFailed",
    0x0221: "LatchFailed",
    0x0301: "BusUnder",
    0x0302: "BusOver",
    0x0303: "MainsLost",
    0x0304: "HoldupLow",
    0x0310: "OverTemp",
    0x0401: "FastOverrun",
    0x0402: "SupervisorOverrun",
    0x0403: "WatchdogReset",
    0x0404: "ParamCrc",
    0x0405: "ParamRange",
    0x0406: "SelftestHash",
    0x0407: "NonFinite",
    0x0408: "HeartbeatLost",
    0x0409: "SeismicOverrun",
    0x0501: "GapEnvelope",
    0x0502: "LiftTimeout",
    0x0503: "LandTimeout",
    0x0504: "PayloadMismatch",
    0x0505: "SaturationPersist",
    0x0506: "SeatFailed",
    0x0507: "NotReferenced",
    0x0601: "EnergyLimit",
    0x0602: "MomentumLimit",
    0x0603: "ForceLimit",
    0x0604: "Overspeed",
    0x0605: "PositionLimit",
    0x0606: "UnintendedMotion",
    0x0607: "Obstruction",
    0x0608: "PassivityViolation",
    0x0609: "NotHoldToRun",
    0x060A: "ObstructionDegraded",
    0x060B: "IsolateLimit",
    0x0701: "VerticalSaturation",
    0x0702: "CenterUnreachable",
}


def fault_name(code: float | int) -> str:
    """``"0x0505 SaturationPersist"`` for a ``FaultRaised`` event payload."""
    c = int(code)
    return f"{c:#06x} {_FAULT_NAMES.get(c, 'unknown')}"


def faults_of(result: RunResult, t0_s: float = -math.inf, t1_s: float = math.inf) -> list[str]:
    """Distinct raised fault names of a run in [t0, t1], in first-raised order."""
    seen: list[str] = []
    for ev in result.events:
        if ev.kind == "FaultRaised" and t0_s <= ev.t_s <= t1_s:
            name = fault_name(ev.a)
            if name not in seen:
                seen.append(name)
    return seen


# ---- figures -----------------------------------------------------------------------------

_HEADER_IN = 0.78  # title + question band, inches

#: Glyphs the house serif (STIX Two Text) lacks, rendered through mathtext instead.
_MATH_GLYPHS: dict[str, str] = {
    "≥": r"$\geq$",
    "≤": r"$\leq$",
    "→": r"$\rightarrow$",
    "≈": r"$\approx$",
    "√": "sqrt",
    "∫": r"$\int$",
    "∞": r"$\infty$",
    "≠": r"$\neq$",
    "∑": r"$\sum$",
}


def tex_safe(text: str) -> str:
    """``text`` with the glyphs the house font lacks (≥ ≤ → ≈ ∫ ∞ ≠ ∑) set as mathtext, and √ as "sqrt".

    Figure text only; records keep the plain Unicode. Text that already contains ``$``
    is returned unchanged.
    """
    if "$" in text:
        return text
    for ch, tex in _MATH_GLYPHS.items():
        text = text.replace(ch, tex)
    return text


_STAMP_IN = 0.24  # run-stamp band, inches
_CAPTION_PT = 8.6


def new_figure(nrows: int = 1, ncols: int = 1, *, size: tuple[float, float] = (12.0, 5.0), **subplot_kw: Any) -> tuple[Any, np.ndarray]:
    """A house-styled :class:`matplotlib.figure.Figure` and its axes grid (always 2-D).

    The figure is not registered with pyplot, so no GUI backend is involved and many
    studies can run in one process.
    """
    from matplotlib.figure import Figure

    from .. import plotstyle

    plotstyle.apply()
    fig = Figure(figsize=size)
    axes = fig.subplots(nrows, ncols, squeeze=False, **subplot_kw)
    return fig, axes


def stamp_meta(cfg: RunConfig, study: str, tiers: str | None = None) -> dict[str, Any]:
    """Run-stamp metadata of a study figure: version, study name, seed, tiers, coil, build."""
    from .. import __version__

    meta: dict[str, Any] = {"version": __version__, "scenario": f"study {study}", "seed": cfg.seed}
    if tiers:
        meta["fidelity"] = tiers
        meta["coil"] = "voltage"
    try:
        from .. import require_native

        n = require_native()
        meta["build_profile"] = getattr(n, "BUILD_PROFILE", None)
        golden = int(getattr(n, "GOLDEN_SELFTEST_HASH", 0))
        if golden:  # 0 is the Wave-0 placeholder until C1 freezes the hash
            meta["golden"] = golden
    except ImportError:  # pragma: no cover - studies need the extension anyway
        pass
    return meta


def finish(
    fig: Any,
    *,
    title: str,
    question: str,
    caption: str,
    meta: Mapping[str, Any],
    hspace: float = 0.42,
    wspace: float = 0.28,
    left_in: float = 0.8,
    right_in: float = 0.25,
) -> str:
    """Lay out ``fig``: title and question on top, the wrapped caption and run stamp below.

    Returns the run-stamp text. The caption states what is computed, what is simulated
    (tier, coil mode) and what is not claimed; the study also puts it in its record.
    """
    from .. import plotstyle

    w_in, h_in = fig.get_size_inches()
    width_chars = max(60, int(w_in * 72 / (_CAPTION_PT * 0.47)))
    paragraphs = [textwrap.fill(p.strip(), width=width_chars) for p in caption.split("\n") if p.strip()]
    text = "\n".join(paragraphs)
    n_lines = text.count("\n") + 1 if text else 0
    caption_in = n_lines * _CAPTION_PT * 1.32 / 72 + 0.12
    fig.text(
        0.012,
        1.0 - 0.10 / h_in,
        tex_safe(title),
        ha="left",
        va="top",
        fontsize=plotstyle.FONT["suptitle"],
        fontweight="semibold",
        color=plotstyle.COLORS["ink"],
    )
    fig.text(
        0.012, 1.0 - 0.44 / h_in, tex_safe(question), ha="left", va="top", fontsize=10.5, color=plotstyle.COLORS["ink2"], style="italic"
    )
    if text:
        fig.text(
            0.012,
            (_STAMP_IN + 0.04) / h_in,
            tex_safe(text),
            ha="left",
            va="bottom",
            fontsize=_CAPTION_PT,
            color=plotstyle.COLORS["ink2"],
            linespacing=1.32,
        )
    bottom_in = _STAMP_IN + caption_in + 0.62  # room for x tick labels and axis label
    fig.subplots_adjust(
        left=left_in / w_in,
        right=1.0 - right_in / w_in,
        top=1.0 - (_HEADER_IN + 0.18) / h_in,
        bottom=bottom_in / h_in,
        hspace=hspace,
        wspace=wspace,
    )
    return plotstyle.run_stamp(fig, meta)


def class_style(rail_class: str) -> dict[str, Any]:
    """Colour, marker and label of a rail class (plotstyle's fixed class identities)."""
    from .. import plotstyle

    return {
        "color": plotstyle.CLASS_COLORS[rail_class],
        "marker": plotstyle.CLASS_MARKERS[rail_class],
        "label": plotstyle.CLASS_LABELS[rail_class],
    }


def check(ok: bool, value: Any, limit: Any, source: str, **extra: Any) -> dict[str, Any]:
    """A uniform check entry of a study record: pass flag, value, limit and source."""
    out: dict[str, Any] = {"pass": bool(ok), "value": value, "limit": limit, "source": source}
    out.update(extra)
    return out
