"""Gallery examples 9–14: the Sovereign/Exotic tier (plan §4b gallery table; EXOTIC.md).

Each example runs the exotic reduced models of ``fabrail-exo-sim`` through the flat
``fabrail._native.exo_*`` bindings (the scenario catalogue XS/XC of EXOTIC §X17, plus
the closed forms the tier is anchored on), reduces them to numbers, checks those
numbers against the requirement they demonstrate and writes three files into
``out_dir``:

- ``figure.png``: a four-panel figure (house style, :mod:`fabrail.plotstyle`);
- ``record.json``: a ``fabrail.example/1`` record (the E3 format: canonical JSON,
  ``sha256`` over the payload without ``wall_s``/``timing``) with every number the story
  and the figure quote, each check with its tolerance and source, and every run with its
  model string, inputs, criteria, KPIs, labels and notes;
- ``story.txt``: the story line with the run's own numbers and the required caveats.

Labels (FAB-EXA-002). Every record carries ``labels``: ``"reduced model"`` whenever a run
is a reduced model (the Rust ``reduced_model`` flag) or the class is on the roadmap for
dynamic simulation (ARENA, TIDEGATE; plan §4 "Reduced models"), and
``"synthetic/injected, not a live EEW feed"`` whenever an alert or a P pick appears. The
same labels head the figure and the story.

Badges (FAB-EXA-002; EXOTIC §0.1). The requirements of an example include the class row
of EXOTIC §1 with the badge the code gives it (``exo_class_table()``: every class here is
🧪 Research), so the badge cap is the weakest status among the FAB-EXO requirements and
the class row. The plan table's 🧪 is at or below that cap for all six.

Nothing here is tuned to pass: the scenario defaults are the catalogue's, the sweeps
and overrides are stated, and a failing check is reported in the record, the story and
the figure, never hidden. Seeds: every exotic run uses the master seed itself, so the
sweeps are paired comparisons on one realisation (the same wind record, the same
pulse) and the defaults equal ``python -m fabrail exo simulate --seed 42``.

| # | Example | Badge | Class and model |
|---|---|---|---|
| 9 | ``supertall_wind_day`` | 🧪 | SENTINEL, 2-DOF first mode + HMD (XS12) |
| 10 | ``museum_haven_quake`` | 🧪 | HAVEN-R, 3-DOF platform (XS01, XS02, XS07) + closed forms |
| 11 | ``proton_shield_pwave`` | 🧪 | SHIELD, reduced kinematic timeline (XC-SHD-1) |
| 12 | ``vault_close_race`` | 🧪 | BASTION, closure race (XS10) + intrusion stop (XC-SAF-1) |
| 13 | ``stadium_tray_pod_failure`` | 🧪 | ARENA, static WLS allocation (XC-ARN-1), reduced model |
| 14 | ``tidegate_offshore_m8`` | 🧪 | TIDEGATE, EEW timing (XS09) + tsunami close (XS11), reduced model |

``run(out_dir, *, quick=False, seed=42, workers=None)`` is the frozen calling
convention (:mod:`fabrail.gallery.registry`). Quick runs are the test-suite default;
full runs add denser sweeps. The 🔭 examples 15–16 live in
:mod:`fabrail.gallery.frontier_examples`, which reuses this module's record machinery.

Owner: E4 (Wave 2b).
"""

from __future__ import annotations

import json
import math
import re
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .core_examples import RECORD_SCHEMA, story_is_clean
from .registry import REGISTRY, ExampleSpec, register

__all__ = [
    "ALERT_LABEL",
    "CLASS_STATUS",
    "EXAMPLES",
    "META",
    "REDUCED_LABEL",
    "REQUIREMENT_STATUS",
    "register_all",
    "requirement_status_drift",
    "run_example",
    "script_main",
]

#: Label of every alert, feed or P pick shown (plan §4b; FAB-EXA-002).
ALERT_LABEL = "synthetic/injected, not a live EEW feed"
#: Label of a reduced model or a class whose dynamic simulation is roadmap (plan §4; FAB-EXA-002).
REDUCED_LABEL = "reduced model"
#: Exotic classes whose dynamic simulation is roadmap (plan §4 "Scope": ARENA, ROTATOR,
#: TIDEGATE, SELENE and full HAVEN-F); their examples are always labelled reduced.
ROADMAP_CLASSES = frozenset({"arena", "rotator", "tidegate", "selene", "haven_f"})

#: Standard gravity, m/s² (CGPM 1901; PHYSICS §1).
_G0 = 9.80665

#: EXOTIC.md / FRONTIER.md St. column of every requirement an example here or in
#: frontier_examples demonstrates (SPECIFICATION §0.5: V verified in simulation, S
#: simplified, R research, M roadmap; "V(I)" counts as V). Drift-checked by
#: :func:`requirement_status_drift`.
REQUIREMENT_STATUS: dict[str, str] = {
    "FAB-EXO-004": "V",
    "FAB-EXO-103": "S",
    "FAB-EXO-110": "V",
    "FAB-EXO-115": "S",
    "FAB-EXO-119": "V",
    "FAB-EXO-502": "V",
    "FAB-EXO-503": "V",
    "FAB-EXO-504": "V",
    "FAB-EXO-505": "R",
    "FAB-EXO-507": "V",
    "FAB-EXO-601": "V",
    "FAB-EXO-603": "R",
    "FAB-EXO-604": "R",
    "FAB-EXO-607": "R",
    "FAB-EXO-609": "S",
    "FAB-EXO-701": "V",
    "FAB-EXO-702": "V",
    "FAB-EXO-703": "R",
    "FAB-EXO-704": "R",
    "FAB-EXO-802": "S",
    "FAB-EXO-803": "V",
    "FAB-EXO-804": "S",
    "FAB-EXO-806": "V",
    "FAB-EXO-807": "V",
    "FAB-EXO-808": "V",
    "FAB-EXO-809": "S",
    "FAB-EXO-810": "V",
    "FAB-FRN-003": "R",
    "FAB-FRN-006": "R",
    "FAB-FRN-008": "V",
}

#: Status of an exotic class row from its code badge (``exo_class_table()['badge']``;
#: EXOTIC §1, §0.1). ``F`` marks the 🔭 frontier override.
CLASS_STATUS: dict[str, str] = {"Verified": "V", "Simplified": "S", "Research": "R", "Roadmap": "M", "Frontier": "F"}

_STATUS_BADGE = {"V": "✅", "S": "🔶", "R": "🧪", "M": "🗺️", "F": "🔭"}
_STATUS_ORDER = ("M", "R", "S", "V")  # weakest first
_STATUS_WORD = {
    "V": "V (verified in simulation)",
    "S": "S (simplified model)",
    "R": "R (research)",
    "M": "M (roadmap)",
    "F": "frontier (a what-if, not a product claim)",
}

#: Math glyphs the house serif lacks (B2 note): drawn through mathtext in figures.
_MATHTEXT = {
    "≤": r"$\leq$",
    "≥": r"$\geq$",
    "≈": r"$\approx$",
    "→": r"$\rightarrow$",
    "∞": r"$\infty$",
    "≠": r"$\neq$",
    "∝": r"$\propto$",
    "√": "sqrt",
}


# =====================================================================================
# Runs
# =====================================================================================


def _native() -> Any:
    from .. import require_native

    return require_native()


def _floats(values: Sequence[Any]) -> np.ndarray:
    return np.array([math.nan if v is None else float(v) for v in values], dtype=np.float64)


@dataclass
class _Run:
    """One exotic run (``exo_simulate``), a scene run or a numpy reference computation."""

    label: str
    kind: str
    cls: str
    scenario: str
    data: dict[str, Any]
    wall_s: float
    overrides: dict[str, Any] = field(default_factory=dict)
    ch: dict[str, np.ndarray] = field(default_factory=dict)

    @property
    def kpis(self) -> dict[str, Any]:
        return self.data.get("kpis", {})

    def kpi(self, name: str) -> float:
        v = self.kpis.get(name)
        return math.nan if v is None else float(v)

    def criterion(self, name: str) -> dict[str, Any]:
        for c in self.data.get("criteria", []):
            if c.get("name") == name:
                return c
        return {"name": name, "passed": False, "value": None, "limit": None, "detail": "criterion not reported"}

    @property
    def reduced(self) -> bool:
        return bool(self.data.get("reduced_model", False))


def _simulate(
    label: str,
    cls: str,
    scenario: str,
    seed: int,
    inputs: Mapping[str, float] | None = None,
    channels: bool = False,
) -> _Run:
    """``exo_simulate(cls, scenario, seed, inputs, channels)`` parsed (fabrail.exo_run/1)."""
    n = _native()
    ov = {k: float(v) for k, v in (inputs or {}).items()}
    t0 = time.perf_counter()
    raw = n.exo_simulate(cls, scenario, int(seed), json.dumps(ov, sort_keys=True) if ov else None, bool(channels))
    wall = time.perf_counter() - t0
    data = json.loads(raw)
    ch = {k: _floats(v.get("data", [])) for k, v in (data.pop("channels", None) or {}).items()}
    return _Run(label, "exo_simulate", cls, scenario, data, wall, ov, ch)


def _run_entry(r: _Run) -> dict[str, Any]:
    """The record's description of one run (channels are plotted, not stored)."""
    d = r.data
    return {
        "label": r.label,
        "kind": r.kind,
        "class": r.cls,
        "scenario": r.scenario,
        "model": d.get("model"),
        "reduced_model": r.reduced,
        "passed": d.get("passed"),
        "seed": d.get("seed"),
        "overrides": dict(r.overrides),
        "inputs": d.get("inputs", {}),
        "criteria": d.get("criteria", []),
        "kpis": d.get("kpis", {}),
        "labels": d.get("labels", {}),
        "notes": d.get("notes", []),
        "wall_s": round(r.wall_s, 4),
    }


def _sweep_entry(label: str, runs: Sequence[_Run], varied: Mapping[str, Any]) -> dict[str, Any]:
    """A compact record entry for a sweep of many runs of one scenario (values in ``results``)."""
    first = runs[0]
    return {
        "label": label,
        "kind": "exo_simulate sweep",
        "class": first.cls,
        "scenario": first.scenario,
        "model": first.data.get("model"),
        "reduced_model": any(r.reduced for r in runs),
        "n_runs": len(runs),
        "varied": dict(varied),
        "all_passed": all(bool(r.data.get("passed")) for r in runs),
        "seed": first.data.get("seed"),
        "wall_s": round(sum(r.wall_s for r in runs), 4),
    }


def _class_row(cls: str) -> dict[str, Any]:
    """The EXOTIC §1 class row from code (``exo_class_table``)."""
    for row in _native().exo_class_table():
        if row.get("class") == cls:
            return dict(row)
    raise KeyError(f"unknown exotic class {cls!r}")


# =====================================================================================
# Checks, badges, records, stories
# =====================================================================================


def _f(x: Any) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


def _rnd(x: Any, nd: int = 7) -> float | None:
    """Round for the record (significant digits); None for non-finite."""
    v = _f(x)
    if not math.isfinite(v):
        return None
    if v == 0.0:
        return 0.0
    return float(f"{v:.{nd}g}")


def _check(
    what: str,
    value: Any,
    limit: Any,
    op: str,
    unit: str,
    source: str,
    req: str | None = None,
    note: str | None = None,
) -> dict[str, Any]:
    """One acceptance check (the E3 format). ``op`` is <=, <, >=, >, ==, abs<= (|value| ≤
    limit), in (limit [lo, hi]) or true (value must be truthy). Non-finite values fail."""
    if op == "true":
        ok = bool(value)
    elif op == "in":
        lo, hi = limit
        ok = math.isfinite(_f(value)) and lo <= _f(value) <= hi
    else:
        fv, fl = _f(value), _f(limit)
        ok = math.isfinite(fv) and math.isfinite(fl) and {
            "<=": fv <= fl,
            "<": fv < fl,
            ">=": fv >= fl,
            ">": fv > fl,
            "==": fv == fl,
            "abs<=": abs(fv) <= fl,
        }[op]
    out: dict[str, Any] = {
        "what": what,
        "value": value if isinstance(value, (bool, str)) or value is None else _rnd(value),
        "op": op,
        "limit": [_rnd(limit[0]), _rnd(limit[1])] if op == "in" else (limit if isinstance(limit, (bool, str)) or limit is None else _rnd(limit)),
        "unit": unit,
        "pass": bool(ok),
        "source": source,
    }
    if req:
        out["requirement"] = req
    if note:
        out["note"] = note
    return out


def _requirements(meta: _Meta) -> list[dict[str, Any]]:
    """The example's requirements with their statuses: FAB IDs from the spec tables, the
    class rows from the code badge (EXOTIC §1)."""
    out: list[dict[str, Any]] = []
    for cls in meta.classes:
        row = _class_row(cls)
        st = CLASS_STATUS.get(str(row.get("badge")), "R")
        out.append(
            {
                "id": f"class:{cls}",
                "status": st,
                "shows": f"{row.get('display', cls.upper())} class row, code badge {row.get('badge')} "
                f"(TRL {row.get('trl_lo')}–{row.get('trl_hi')}; EXOTIC §1)",
            }
        )
    for rid, what in meta.requirements:
        out.append({"id": rid, "status": REQUIREMENT_STATUS[rid], "shows": what})
    return out


def _badge_rule(reqs: Sequence[Mapping[str, Any]], badge: str) -> dict[str, Any]:
    core = [r["status"] for r in reqs if r["status"] in _STATUS_ORDER]
    weakest = min(core, key=_STATUS_ORDER.index) if core else "V"
    frontier = any(r["status"] == "F" for r in reqs) or badge == "🔭"
    rule: dict[str, Any] = {
        "weakest_status": weakest,
        "badge_cap": _STATUS_BADGE[weakest],
        "rule": "no example's badge exceeds the weakest requirement it demonstrates (FAB-EXA-002); "
        "🔭 replaces the badge when the technology's maturity is frontier (EXOTIC §0.1)",
    }
    if frontier:
        rule["frontier_override"] = True
    elif badge in _STATUS_BADGE.values() and badge != _STATUS_BADGE[weakest]:
        rule["below_cap"] = True
    return rule


def _guard(text: str, where: str) -> str:
    for line in str(text).splitlines():
        if line.lstrip().startswith(">"):
            continue
        if not story_is_clean(line):
            raise ValueError(f"{where}: banned vocabulary in {line[:100]!r} (SPECIFICATION_EXAMPLES §0.3; FAB-EXA-003)")
    return text


def _guard_strings(obj: Any, where: str) -> None:
    """Every string of a record passes the vocabulary guard (the examples/out JSON is scanned too)."""
    if isinstance(obj, str):
        _guard(obj, where)
    elif isinstance(obj, Mapping):
        for k, v in obj.items():
            _guard_strings(v, f"{where}.{k}")
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            _guard_strings(v, f"{where}[{i}]")


def _fmt(x: Any, spec: str = ".3g", unit: str = "") -> str:
    v = _f(x)
    if not math.isfinite(v):
        return "n/a"
    s = format(v, spec)
    return f"{s} {unit}".rstrip() if unit else s


def _failed_sentence(checks: Sequence[Mapping[str, Any]]) -> str:
    bad = [c for c in checks if not c["pass"]]
    if not bad:
        return f"All {len(checks)} checks pass."

    def one(c: Mapping[str, Any]) -> str:
        v, lim, op = c["value"], c["limit"], c["op"]
        if v is None:
            got = "not reached"
        elif isinstance(v, bool):
            got = "yes" if v else "no"
        else:
            got = f"{v:g} {c['unit']}".strip() if isinstance(v, (int, float)) else str(v)
        want = {"true": "required", "abs<=": f"|·| ≤ {lim}", "in": f"in {lim}"}.get(op, f"{op} {lim} {c['unit']}".strip())
        return f"{c['what']}: {got} ({want}; {c['source']})"

    return f"{len(checks) - len(bad)} of {len(checks)} checks pass. Not met in this build: " + "; ".join(one(c) for c in bad) + "."


@dataclass(frozen=True)
class _Meta:
    """Static facts of one example."""

    number: int
    name: str
    title: str
    badge: str
    maturity: str
    story: str
    classes: tuple[str, ...]
    requirements: tuple[tuple[str, str], ...]  # (FAB id, what the example shows)
    caveats: tuple[str, ...]
    labels: tuple[str, ...] = ()
    badge_note: str = ""


def _labels(meta: _Meta, runs: Sequence[_Run]) -> list[str]:
    labels = list(meta.labels)
    if (any(r.reduced for r in runs) or any(c in ROADMAP_CLASSES for c in meta.classes)) and REDUCED_LABEL not in labels:
        labels.insert(0, REDUCED_LABEL)
    return labels


def _finish(
    meta: _Meta,
    out_dir: Path,
    *,
    quick: bool,
    seed: int,
    runs: Sequence[_Run],
    run_entries: Sequence[Mapping[str, Any]] | None = None,
    results: Mapping[str, Any],
    checks: Sequence[Mapping[str, Any]],
    story: str,
    figure: Callable[[Any], None],
    wall_s: float,
    frontier: Mapping[str, Any] | None = None,
    extra: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Write figure.png, record.json and story.txt; return the record.

    ``frontier`` (🔭 examples) carries the stamp keys of FAB-EXO-004 (``tier``,
    ``banner``, ``missing_orders``, ``note``) and the frontier records; the banner then
    heads the story and a watermark crosses the figure.
    """
    from .. import __version__, export, plotstyle

    n = _native()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    story = _guard(" ".join(story.split()), f"{meta.name} story")
    reqs = _requirements(meta)
    labels = _labels(meta, runs)
    status = "F" if meta.badge == "🔭" else {v: k for k, v in _STATUS_BADGE.items()}[meta.badge]
    entries = list(run_entries) if run_entries is not None else [_run_entry(r) for r in runs]
    models = sorted({str(e.get("model")) for e in entries if e.get("model")})
    record: dict[str, Any] = {
        "schema": RECORD_SCHEMA,
        "number": meta.number,
        "name": meta.name,
        "title": meta.title,
        "badge": meta.badge,
        "status": status,
        "maturity": meta.maturity,
        "tier": "frontier" if meta.badge == "🔭" else "sovereign",
        "label": "simulated"
        + (f" ({', '.join(labels)})" if labels else "")
        + "; checked in simulation, not measured on hardware",
        "labels": labels,
        "reduced_model": REDUCED_LABEL in labels,
        "alert_label": ALERT_LABEL if ALERT_LABEL in labels else None,
        "classes": list(meta.classes),
        "models": models,
        "quick": bool(quick),
        "seed": int(seed),
        "command": f"python -m fabrail example run {meta.name}" + (" --quick" if quick else ""),
        "script": f"examples/{meta.name}.py",
        "fabrail_version": __version__,
        "exo_version": n.exo_version(),
        "build_profile": getattr(n, "BUILD_PROFILE", None),
        "requirements": reqs,
        "badge_rule": _badge_rule(reqs, meta.badge),
        "caveats": list(meta.caveats),
        "results": dict(results),
        "checks": list(checks),
        "checks_passed": sum(1 for c in checks if c["pass"]),
        "checks_total": len(checks),
        "all_checks_pass": all(c["pass"] for c in checks),
        "runs": entries,
        "story": story,
        "outputs": {"figure": "figure.png", "record": "record.json", "story": "story.txt"},
        "timing": {"wall_s": round(wall_s, 3)},
    }
    if meta.badge_note:
        record["badge_rule"]["note"] = meta.badge_note
    if frontier:
        record.update(dict(frontier))
    if extra:
        record.update(dict(extra))
    record = export.to_jsonable(record)
    _guard_strings(record, f"{meta.name} record")

    from matplotlib.figure import Figure

    plotstyle.apply()
    fig = Figure(figsize=(12.0, 8.4))
    figure(fig)
    fig.suptitle(_mt(f"{meta.number}. {meta.name}: {meta.title}"), x=0.01, y=0.995, ha="left", fontsize=plotstyle.FONT["suptitle"] - 1)
    state = "all checks pass" if record["all_checks_pass"] else f"{record['checks_passed']}/{record['checks_total']} checks pass"
    tags = "  ·  ".join([f"badge {_STATUS_WORD[status]}", state, *labels, "simulated, not measured on hardware"])
    fig.text(0.01, 0.952, _mt(tags), ha="left", va="top", fontsize=9.5, color=plotstyle.COLORS["ink2"])
    if frontier:
        plotstyle.watermark(fig, str(frontier.get("watermark", "")))
    stamp = {"scenario": meta.name, "rail_class": "/".join(c.upper() for c in meta.classes) or None, "seed": seed, "build_profile": record["build_profile"]}
    plotstyle.run_stamp(fig, stamp)
    fig.tight_layout(rect=(0.0, 0.02, 1.0, 0.93))
    plotstyle.save(fig, out_dir / "figure.png")

    record["sha256"] = export.payload_sha256(record)
    export.write_json(out_dir / "record.json", record)
    head = f"{frontier['banner']}\n\n" if frontier else ""
    label_line = f"Labels: {'; '.join(labels)}.\n\n" if labels else ""
    caveat_text = "\n".join(f"- {c}" for c in meta.caveats)
    (out_dir / "story.txt").write_text(
        f"{head}{meta.number}. {meta.name} ({meta.badge}; maturity: {meta.maturity})\n\n{label_line}{story}\n\nCaveats:\n{caveat_text}\n",
        encoding="utf-8",
    )
    return record


def _options(options: Mapping[str, Any]) -> tuple[int, int | None]:
    """(seed, workers) from the run options; other non-None options are rejected."""
    opts = dict(options)
    seed = opts.pop("seed", 42)
    workers = opts.pop("workers", None)
    for k in ("variant", "building", "output", "out_dir"):
        opts.pop(k, None)
    unknown = sorted(k for k, v in opts.items() if v is not None)
    if unknown:
        raise ValueError(f"unknown option(s) {', '.join(unknown)} (accepted: seed, workers)")
    seed = 42 if seed is None else int(seed)
    if seed < 0:
        raise ValueError(f"seed: must be >= 0 (got {seed})")
    return seed, (None if workers is None else int(workers))


# ---- plotting helpers ------------------------------------------------------------------------


def _mt(text: str) -> str:
    """Figure text with the glyphs the house serif lacks drawn through mathtext."""
    out = str(text)
    for k, v in _MATHTEXT.items():
        out = out.replace(k, v)
    return out


def _style(ax: Any, xlabel: str | None, ylabel: str | None, title: str) -> None:
    from .. import plotstyle

    plotstyle.style_ax(ax, _mt(xlabel) if xlabel else None, _mt(ylabel) if ylabel else None, _mt(title))
    ax.title.set_fontsize(11.5)


def _vmark(ax: Any, t: float, label: str, color: str | None = None, y: float = 0.97) -> None:
    from .. import plotstyle

    if not math.isfinite(_f(t)):
        return
    c = color or plotstyle.COLORS["muted"]
    ax.axvline(t, color=c, linestyle=(0, (3, 3)), linewidth=1.0, zorder=1)
    ax.annotate(_mt(label), xy=(t, y), xycoords=("data", "axes fraction"), rotation=90, ha="right", va="top", fontsize=7.5, color=plotstyle.COLORS["ink2"], xytext=(-2, 0), textcoords="offset points")


def _hline(ax: Any, y: float, label: str, color: str | None = None, va: str = "bottom", x: float = 0.985) -> None:
    from .. import plotstyle

    plotstyle.ref_line(ax, y, _mt(label), color=color, x=x, va=va)


def _legend(ax: Any, **kw: Any) -> None:
    from .. import plotstyle

    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, [_mt(s) for s in labels], **{**plotstyle.LEGEND_KW, "fontsize": 8.5, **kw})


def _note(ax: Any, text: str, x: float = 0.02, y: float = 0.96, ha: str = "left", va: str = "top", size: float = 8.0) -> None:
    from .. import plotstyle

    ax.text(x, y, _mt(text), transform=ax.transAxes, ha=ha, va=va, fontsize=size, color=plotstyle.COLORS["ink2"], linespacing=1.35)


def _colors() -> dict[str, str]:
    from .. import plotstyle

    return dict(plotstyle.COLORS)


# =====================================================================================
# 9. supertall_wind_day (SENTINEL, XS12)
# =====================================================================================

_SUPERTALL = _Meta(
    number=9,
    name="supertall_wind_day",
    title="a 1,000 t hybrid mass damper on a windy day",
    badge="🧪",
    maturity="TRL 3–4 (1,000 t maglev planar bearing); eddy damping TRL 9",
    story=(
        "SENTINEL, a 1,000 t hybrid mass damper with an ironless LSM trim, rides a residential supertall "
        "through a windy day. On a low-coercivity rail it activates far below the 0.3 mg target, and the drop "
        "in the tower's sway is reported against the tower's own damping ζs, passive and active. Mass dampers "
        "are mainly wind devices."
    ),
    classes=("sentinel",),
    requirements=(
        ("FAB-EXO-607", "activation ≤ 0.3 mg; RMS reduction reported against ζs; the ≥ 40 % target for the active mode only; ironless trim LSM"),
        ("FAB-EXO-604", "stroke within s_max = 1.5 m and LSM force within F_max under the saturation stack"),
        ("FAB-EXO-601", "Warburton force-form tuning at μ 1 % (f 0.99257, ζ re-expressed on ω_s)"),
        ("FAB-EXO-609", "Coulomb activation threshold a = μ·g and the rail condition it needs"),
        ("FAB-EXO-110", "kinematic budget at T 7 s, ±1.5 m: 1.346 m/s, 1.209 m/s², 1.085 m/s³"),
        ("FAB-EXO-603", "the stroke bound ζ_add ≤ (μ/2)(S/X): 3.5 % in wind, 0.7 % at design-level shaking"),
    ),
    caveats=(
        "Mass dampers are mainly wind devices: at design-level shaking the stroke bound caps the added damping at "
        "about 0.7 % (ζ_add ≤ (μ/2)(S/X) with S/X = 1.4), so SENTINEL is not a seismic safety feature.",
        "The activation figure needs a low-coercivity or homopolar rail (μ_eq about 1e-4, [UNCERTAIN] magnitude); on a "
        "solid-steel or laminated rail (μ_eq about 1e-3) the damper activates near 1 mg and fails the one-tenth rule. "
        "An iron-core LSM's cogging alone would break the 0.3 mg budget, hence the ironless trim.",
        "The RMS reduction depends on the tower's own damping ζs and is always printed with it; the ≥ 40 % target applies "
        "to the active mode only (the passive H2 optimum gives 44.7 % at μ 1 %, ζs 1 % but 30.0 % at ζs 2 %).",
        "Model: the tower's first mode (100 kt modal mass, T 7 s) plus the damper as a 2-DOF system; Davenport wind as a "
        "quasi-steady force with unit aerodynamic admittance [UNCERTAIN]. Simulated, not measured; TRL 3–4.",
    ),
)

#: Tower damping ratios of the paired wind runs (the reduction is reported against ζs; FAB-EXO-607).
_ZS_QUICK = (0.01, 0.02)
_ZS_FULL = (0.005, 0.01, 0.02, 0.03, 0.05)
#: Uncontrolled peak host accelerations of the full-run sweep, mg (wind comfort 3–7 mg; XS12 input range).
_A_TARGET_FULL = (3.0, 5.0, 7.0)
#: Coulomb activation μ·g of the EXOTIC §X8 bearing table, mg ([UNCERTAIN] magnitudes, swept there).
_BEARINGS_MG = (
    ("PTFE slider (μ 0.05)", 50.0),
    ("rolling guide (μ 0.003)", 3.0),
    ("hybrid EMS, solid-steel rail (μ_eq ~1e-3)", 1.0),
    ("hybrid EMS, laminated rail (μ_eq ~1e-3)", 1.0),
    ("hybrid EMS, low-coercivity rail (μ_eq ~1e-4)", 0.1),
    ("HTS over a PM guideway (μ ≤ 1e-5)", 0.01),
)
#: Mass ratio of SENTINEL (EXOTIC §X7: μ ≈ 1 % of the 100 kt modal mass).
_SENTINEL_MU = 0.01


def _frf(f_in: np.ndarray, x_out: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """|H(f)| = |S_xf|/S_ff and the coherence from averaged Hann periodograms (50 % overlap)."""
    n = len(f_in)
    seg = max(32, int(2 ** math.floor(math.log2(max(32, n // 4)))))
    step = seg // 2
    win = np.hanning(seg)
    sff = np.zeros(seg // 2 + 1)
    sxx = np.zeros(seg // 2 + 1)
    sxf = np.zeros(seg // 2 + 1, dtype=complex)
    for i in range(0, n - seg + 1, step):
        a = np.fft.rfft((f_in[i : i + seg] - np.mean(f_in[i : i + seg])) * win)
        b = np.fft.rfft((x_out[i : i + seg] - np.mean(x_out[i : i + seg])) * win)
        sff += np.abs(a) ** 2
        sxx += np.abs(b) ** 2
        sxf += b * np.conj(a)
    with np.errstate(divide="ignore", invalid="ignore"):
        h = np.abs(sxf) / sff
        coh = np.abs(sxf) ** 2 / (sff * sxx)
    return np.fft.rfftfreq(seg, 1.0 / fs), np.nan_to_num(h), np.nan_to_num(coh)


def _supertall(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    n = _native()
    seed, _ = _options(options)
    t0 = time.perf_counter()
    meta = _SUPERTALL
    zs = _ZS_QUICK if quick else _ZS_FULL
    runs = [_simulate(f"wind_zs{z:g}", "sentinel", "XS12", seed, {"zeta_s": z}, channels=(z == 0.01)) for z in zs]
    base = next(r for r in runs if r.overrides["zeta_s"] == 0.01)
    grid: list[_Run] = []
    if not quick:
        grid = [
            _simulate(f"wind_a{a:g}_zs{z:g}", "sentinel", "XS12", seed, {"zeta_s": z, "a_target_mg": a})
            for a in _A_TARGET_FULL
            for z in (0.01, 0.02)
            if a != 5.0
        ]
    row = _class_row("sentinel")
    period = row["period_ms"] / 1000.0
    s_max = row["travel_mm"] / 1000.0
    w = 2.0 * math.pi / period
    kin = {
        "v_mps": row["v_max_mmps"] / 1000.0,
        "a_mps2": row["a_max_mmps2"] / 1000.0,
        "j_mps3": row["j_max_mmps3"] / 1000.0,
        "v_closed_form_mps": w * s_max,
        "a_closed_form_mps2": w * w * s_max,
        "j_closed_form_mps3": w**3 * s_max,
    }
    m_d = float(row["design_mass_kg"])
    tuning = n.exo_tuning(_SENTINEL_MU, "warburton_force", m_d / _SENTINEL_MU, w)
    bound_wind = n.exo_zeta_add_bound(_SENTINEL_MU, 7.0)
    bound_quake = n.exo_zeta_add_bound(_SENTINEL_MU, 1.4)

    per_zs = [
        {
            "zeta_s": r.overrides["zeta_s"],
            "rms_reduction_passive_pct": r.kpi("rms_reduction_passive_pct"),
            "rms_reduction_active_pct": r.kpi("rms_reduction_active_pct"),
            "h2_reduction_passive_pct": r.kpi("h2_reduction_passive_pct"),
            "h2_reduction_active_pct": r.kpi("h2_reduction_active_pct"),
            "activation_mg": r.kpi("activation_mg"),
            "stroke_peak_active_m": r.kpi("stroke_peak_active_m"),
            "lsm_force_peak_n": r.kpi("lsm_force_peak_n"),
            "host_peak_accel_uncontrolled_mg": r.kpi("host_peak_accel_uncontrolled_mg"),
            "host_peak_accel_passive_mg": r.kpi("host_peak_accel_passive_mg"),
            "host_peak_accel_active_mg": r.kpi("host_peak_accel_active_mg"),
        }
        for r in runs
    ]
    results: dict[str, Any] = {
        "per_zeta_s": per_zs,
        "grid": [
            {
                "a_target_mg": r.overrides["a_target_mg"],
                "zeta_s": r.overrides["zeta_s"],
                "rms_reduction_passive_pct": r.kpi("rms_reduction_passive_pct"),
                "rms_reduction_active_pct": r.kpi("rms_reduction_active_pct"),
                "stroke_peak_active_m": r.kpi("stroke_peak_active_m"),
                "activation_mg": r.kpi("activation_mg"),
            }
            for r in grid
        ],
        "iron_core_cogging_mg": [base.kpi("iron_core_cogging_mg_lo"), base.kpi("iron_core_cogging_mg_hi")],
        "coulomb_stick_0p9_m": base.kpi("coulomb_stick_0p9_m"),
        "coulomb_slip_1p1_m": base.kpi("coulomb_slip_1p1_m"),
        "tuning_warburton_force": dict(tuning),
        "kinematics": kin,
        "zeta_add_bound": {"wind_s_over_x_7": bound_wind, "design_quake_s_over_x_1p4": bound_quake},
        "bearing_activation_mg": [{"bearing": b, "a_act_mg": a} for b, a in _BEARINGS_MG],
        "rail_condition": base.data.get("labels", {}).get("rail_condition"),
        "lsm": base.data.get("labels", {}).get("lsm"),
    }

    act = base.kpi("activation_mg")
    stroke = base.criterion("stroke_and_force_within_limits")
    checks = [
        _check("bearing activation (low-coercivity rail, ironless LSM)", act, 0.3, "<=", "mg", "FAB-EXO-607 ≤ 0.3 mg (EXOTIC §X8)", "FAB-EXO-607"),
        _check("active host RMS reduction at ζs 1 %", base.kpi("rms_reduction_active_pct"), 40.0, ">=", "%", "FAB-EXO-607 ≥ 40 % (active mode only)", "FAB-EXO-607"),
        _check(
            "passive H2 reduction at μ 1 %, ζs 1 % vs the 44.7 % anchor",
            base.kpi("h2_reduction_passive_pct") - 44.7,
            0.1,
            "abs<=",
            "points",
            "EXOTIC §2 wind anchor 44.7 % (±0.1 point, the XS12 unit test)",
            "FAB-EXO-607",
        ),
        _check("peak damper stroke (active) within s_max", _f(stroke.get("value")), s_max, "<=", "m", "FAB-EXO-604 stroke ≤ s_max = 1.5 m", "FAB-EXO-604"),
        _check("stroke and LSM force within limits (XS12 criterion)", bool(stroke.get("passed")), True, "true", "", "XS12 stroke_and_force_within_limits", "FAB-EXO-604"),
        _check("Warburton force-form f at μ 1 %", float(tuning["f_ratio"]) - 0.99257, 5e-6, "abs<=", "", "FAB-EXO-601 (E) f 0.99257 (printed digits)", "FAB-EXO-601"),
        _check(
            "Coulomb activation: stuck at 0.9·μg, slips at 1.1·μg",
            base.kpi("coulomb_stick_0p9_m") == 0.0 and base.kpi("coulomb_slip_1p1_m") > 0.0,
            True,
            "true",
            "",
            "FAB-EXO-609 no slip at 0.9·μg, slip at 1.1·μg",
            "FAB-EXO-609",
        ),
        _check(
            "iron-core cogging upper estimate vs the 0.3 mg budget (why the trim is ironless)",
            base.kpi("iron_core_cogging_mg_hi"),
            0.3,
            ">",
            "mg",
            "EXOTIC §X7: cogging at 1–3 % of 121 kN on 1,000 t is 0.12–0.37 mg",
            "FAB-EXO-607",
        ),
        _check("kinematic budget v = ω·S", kin["v_mps"] - kin["v_closed_form_mps"], 0.001, "abs<=", "m/s", "FAB-EXO-110 1.346 m/s (class row, mm/s units)", "FAB-EXO-110"),
        _check("stroke bound at design-level shaking", bound_quake, 0.007 + 1e-12, "<=", "", "FAB-EXO-603 ζ_add ≤ (μ/2)(S/X) = 0.7 % at S/X 1.4", "FAB-EXO-603"),
    ]

    zs2 = next((p for p in per_zs if p["zeta_s"] == 0.02), None)
    story = (
        f"SENTINEL, a {m_d / 1e3:,.0f} t hybrid mass damper (μ {100 * _SENTINEL_MU:g} % of a 100 kt first mode, T {period:g} s, "
        f"±{s_max:g} m), rides a residential supertall through an hour of Davenport wind that would sway the bare tower to "
        f"{_fmt(base.kpi('host_peak_accel_uncontrolled_mg'), '.1f')} mg peak. On a low-coercivity rail with an ironless trim LSM "
        f"it activates at {_fmt(act, '.2g')} mg, below the 0.3 mg target (an iron-core LSM's cogging alone would be "
        f"{_fmt(base.kpi('iron_core_cogging_mg_lo'), '.2f')}–{_fmt(base.kpi('iron_core_cogging_mg_hi'), '.2f')} mg). "
        f"At ζs = 1 % the tower's displacement RMS falls by {_fmt(base.kpi('rms_reduction_passive_pct'), '.1f')} % with the damper "
        f"passive (H2 closed form {_fmt(base.kpi('h2_reduction_passive_pct'), '.1f')} %) and by "
        f"{_fmt(base.kpi('rms_reduction_active_pct'), '.1f')} % in HMD_ACTIVE, using {_fmt(base.kpi('stroke_peak_active_m'), '.2f')} m "
        f"of stroke and {_fmt(base.kpi('lsm_force_peak_n') / 1e3, '.0f')} kN of LSM force"
        + (
            f"; at ζs = 2 % the same wind gives {_fmt(zs2['rms_reduction_passive_pct'], '.1f')} % passive and "
            f"{_fmt(zs2['rms_reduction_active_pct'], '.1f')} % active."
            if zs2
            else "."
        )
        + f" Seismically the same damper adds at most about {100 * bound_quake:.1f} % damping at design level: it is a wind device. "
        + _failed_sentence(checks)
    )

    def figure(fig: Any) -> None:
        c = _colors()
        col = {"bare": c["ctrl"], "passive": c["s2"], "active": c["s1"]}
        t = base.ch.get("t_s", np.zeros(0))
        ax1, ax2, ax3, ax4 = (fig.add_subplot(2, 2, i) for i in (1, 2, 3, 4))
        # (a) 10 minutes of sway
        if t.size:
            w0, w1 = 1200.0, 1800.0
            m = (t >= w0) & (t <= w1)
            for key, lab in (("bare", "bare tower"), ("passive", "damper passive"), ("active", "HMD_ACTIVE (LQR)")):
                x = base.ch[f"x_host_{key}_m"]
                ax1.plot((t[m] - w0) / 60.0, 1e3 * x[m], color=col[key], lw=1.3 if key != "bare" else 1.0, label=f"{lab}: RMS {1e3 * float(np.sqrt(np.mean(x[t > 60] ** 2))):.1f} mm")
            lim = 1e3 * float(np.max(np.abs(base.ch["x_host_bare_m"][m])))
            ax1.set_ylim(-1.15 * lim, 1.75 * lim)
        _style(ax1, "time in the record window (min)", "tower displacement (mm)", "Tower sway over 10 minutes of wind (ζs 1 %)")
        _legend(ax1, loc="upper left", ncol=1, fontsize=8)
        # (b) frequency response estimated from the wind record
        if t.size:
            fs = 1.0 / float(t[1] - t[0])
            keep = t > 60.0
            k_host = (m_d / _SENTINEL_MU) * w * w
            for key in ("bare", "passive", "active"):
                f, h, coh = _frf(base.ch["wind_force_n"][keep], base.ch[f"x_host_{key}_m"][keep], fs)
                sel = (f > 0.06) & (f < 0.32) & (coh > 0.5)
                ax2.plot(f[sel], k_host * h[sel], color=col[key], lw=1.5 if key != "bare" else 1.1, marker="o", ms=2.5, label=key)
            _vmark(ax2, 1.0 / period, f"f_s = 1/{period:g} Hz")
        _style(ax2, "frequency (Hz)", "|X/F|·M·ω_s² (dynamic amplification)", "Tower response measured from the wind record")
        _legend(ax2, loc="upper right")
        # (c) reduction vs ζs
        xs = np.arange(len(per_zs))
        wbar = 0.26
        ax3.bar(xs - wbar, [p["rms_reduction_passive_pct"] for p in per_zs], wbar, color=col["passive"], label="passive, simulated")
        ax3.bar(xs, [p["h2_reduction_passive_pct"] for p in per_zs], wbar, color=col["passive"], alpha=0.45, label="passive, H2 closed form")
        ax3.bar(xs + wbar, [p["rms_reduction_active_pct"] for p in per_zs], wbar, color=col["active"], label="HMD_ACTIVE, simulated")
        for i, p in enumerate(per_zs):
            ax3.annotate(f"{p['rms_reduction_active_pct']:.0f} %", xy=(i + wbar, p["rms_reduction_active_pct"]), ha="center", va="bottom", fontsize=7.5, color=c["ink2"])
        ax3.set_xticks(xs, [f"ζs {100 * p['zeta_s']:g} %" for p in per_zs])
        _hline(ax3, 40.0, "≥ 40 % target (active mode only)", color=c["star"], x=0.62)
        ax3.set_ylim(0, max(80.0, 1.15 * max(p["rms_reduction_active_pct"] for p in per_zs)))
        _style(ax3, None, "host displacement RMS reduction (%)", "Reduction reported against the tower's own damping ζs")
        _legend(ax3, loc="upper right", fontsize=8)
        # (d) activation
        names = [b for b, _ in _BEARINGS_MG]
        vals = [a for _, a in _BEARINGS_MG]
        ys = np.arange(len(names))[::-1]
        ax4.barh(ys, vals, color=[c["ctrl"] if v > 0.3 else c["s3"] for v in vals], height=0.6)
        ax4.set_xscale("log")
        ax4.set_yticks(ys, [_mt(s) for s in names], fontsize=8)
        lo, hi = results["iron_core_cogging_mg"]
        ax4.axvspan(lo, hi, color=c["s2"], alpha=0.18, label="iron-core LSM cogging alone")
        ax4.axvline(0.3, color=c["star"], lw=1.4, ls=(0, (4, 3)), label="0.3 mg target (1/10 of 3 mg)")
        ax4.plot([act], [ys[4]], marker="D", color=c["s1"], ms=7, ls="none", label=f"simulated activation {act:.2g} mg")
        ax4.axvline(7e-3, color=c["muted"], lw=1.0, ls=(0, (1, 2)), label="ADXL355 floor ~7 µg rms")
        _style(ax4, "activation threshold μ·g (mg, log)", None, "Activation: the rail condition decides (EXOTIC §X8)")
        _legend(ax4, loc="lower right", fontsize=7.5)

    return _finish(meta, out_dir, quick=quick, seed=seed, runs=runs + grid, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t0)


# =====================================================================================
# 10. museum_haven_quake (HAVEN-R, XS01/XS02/XS07)
# =====================================================================================

_MUSEUM = _Meta(
    number=10,
    name="museum_haven_quake",
    title="a 50 t isolation platform at T = 6 s through a near-fault pulse",
    badge="🧪",
    maturity="TRL 2–3 (no ASCE 7 Ch. 17 prototype tests; about 10⁴× scale-up from lithography stages)",
    story=(
        "A HAVEN-R platform carries a museum collection on a curved guideway of radius 8.95 m (T = 6 s). A Kobe-type "
        "near-fault pulse moves it inside the buffer onset of its 1.2 m moat at under 0.1 g, skyhook damping trades "
        "stroke for acceleration against a passive damper, and the moat edge has passive covers and exclusion."
    ),
    classes=("haven_r",),
    requirements=(
        ("FAB-EXO-701", "curved-guideway period T = 2π·√(R/g): R 8.95 m at T 6 s"),
        ("FAB-EXO-702", "transmissibility anchors, passive / skyhook, at T 4 s, 1 Hz, ζ 0.2: 0.1251 / 0.0663"),
        ("FAB-EXO-703", "the pulse stays within the moat; the fling keeps support; bumper energy quoted in MJ"),
        ("FAB-EXO-704", "control loss ends in an FPS landing at ≤ 0.1 m/s through the PM-bias ramp"),
        ("FAB-EXO-505", "a_peak ≤ 0.1 g at DBE as an analysis output; moat covers plus exclusion"),
    ),
    caveats=(
        "Research tier: maglev isolation has no code path (no ASCE 7 Ch. 17 prototype tests), needs 10–500× the bearing "
        "area of LRB or FPS and depends entirely on power; a 10–20 mm gap makes settlement and fault offset critical.",
        "The moat edge is a shear hazard that presence sensing cannot stop while the earthquake drives the platform: it "
        "has passive covers plus exclusion, and PL e presence sensing gates only controlled moves (recentring, "
        "pre-positioning, landing).",
        "The 0.1 g occupant limit is an analysis output at DBE for the stated site (S_D1 0.6); MCE and near-fault values "
        "are reported, not bounded. The Chi-Chi-like fling exceeds the moat by construction; its bumper energy is quoted in MJ.",
        "Vertical isolation stays passive (air springs); maglev is horizontal trim only. The passive-against-skyhook time "
        "history is a linear 1-DOF numpy reference on the same ground record, not the 3-DOF platform model.",
    ),
)

#: ASCE 7-16 Ch. 17 inputs of the EXOTIC §X9 table: S_D1 at DBE and S_M1 at MCE (g), B at ζ 0.2.
_ASCE_SD1_DBE = 0.6
_ASCE_SM1_MCE = 0.9
_ASCE_B_ZETA02 = 1.5
_ISO_ZETA = 0.2
#: Skyhook ratio of the reference HAVEN-R isolation controller (fabrail-exo-sim haven_iso_cfg: ζ_sky 0.3).
_HAVEN_ZETA_SKY = 0.3
#: HAVEN-R design mass, kg, from the class row at run time; the plinth KE uses it.


def _a_peak_g(sd1_g: float, period_s: float, b: float, zeta: float) -> tuple[float, float]:
    """(S_D m, a_peak g): S_D = S_D1·g·T/(4π²·B), a_peak = ω²·S_D·√(1 + 4ζ²) (EXOTIC §X9; ASCE 7-16 Ch. 17)."""
    sd = sd1_g * _G0 * period_s / (4.0 * math.pi**2 * b)
    w = 2.0 * math.pi / period_s
    return sd, w * w * sd * math.sqrt(1.0 + 4.0 * zeta * zeta) / _G0


def _sdof_isolator(t_s: np.ndarray, a_g: np.ndarray, period_s: float, zeta: float, skyhook: bool, dt: float = 0.005) -> dict[str, float]:
    """Linear 1-DOF isolator on the ground record ``a_g(t)`` (RK4, linear interpolation):
    passive = relative damper c·ẋ_rel, skyhook = absolute damper c·(ẋ_rel + v_g); c = 2ζ·m·ω.
    Returns the peak relative displacement (m) and peak absolute acceleration (g)."""
    tt = np.arange(0.0, float(t_s[-1]), dt)
    a = np.interp(tt, t_s, a_g)
    vg = np.concatenate([[0.0], np.cumsum(0.5 * (a[1:] + a[:-1]) * dt)])
    w = 2.0 * math.pi / period_s
    c = 2.0 * zeta * w

    def f(s: np.ndarray, ai: float, vgi: float) -> np.ndarray:
        damp = c * (s[1] + vgi) if skyhook else c * s[1]
        return np.array([s[1], -w * w * s[0] - damp - ai])

    s = np.zeros(2)
    x_pk = a_pk = 0.0
    for i in range(len(tt) - 1):
        a0, a1, g0, g1 = a[i], a[i + 1], vg[i], vg[i + 1]
        am, gm = 0.5 * (a0 + a1), 0.5 * (g0 + g1)
        k1 = f(s, a0, g0)
        k2 = f(s + 0.5 * dt * k1, am, gm)
        k3 = f(s + 0.5 * dt * k2, am, gm)
        k4 = f(s + dt * k3, a1, g1)
        s = s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        x_pk = max(x_pk, abs(s[0]))
        a_abs = -w * w * s[0] - (c * (s[1] + g1) if skyhook else c * s[1])
        a_pk = max(a_pk, abs(a_abs))
    return {"peak_x_rel_m": x_pk, "peak_a_abs_g": a_pk / _G0}


def _museum(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    n = _native()
    seed, _ = _options(options)
    t0 = time.perf_counter()
    meta = _MUSEUM
    row = _class_row("haven_r")
    m_kg = float(row["design_mass_kg"])
    t_iso = row["period_ms"] / 1000.0
    pulse = _simulate("near_fault_pulse", "haven_r", "XS01", seed, channels=True)
    fling = _simulate("fling_5m", "haven_r", "XS02", seed, channels=True)
    loss = _simulate("control_loss", "haven_r", "XS07", seed, channels=True)
    runs = [pulse, fling, loss]
    if not quick:
        runs += [
            _simulate("pulse_with_cross_component", "haven_r", "XS01", seed, {"y_frac": 0.7}),
            _simulate("pulse_mw7p6_1p2mps", "haven_r", "XS01", seed, {"mw": 7.6, "v_pulse_mps": 1.2}),
            _simulate("grid_loss_at_s", "haven_r", "XS06", seed),
        ]

    r_6 = n.exo_guideway_radius(t_iso)
    trans = {
        f"{p}_{f:g}hz_T{T:g}s": (n.transmissibility if p == "passive" else n.skyhook_transmissibility)(f * T, _ISO_ZETA)
        for T in (4.0, 6.0)
        for f in (0.5, 1.0, 2.0)
        for p in ("passive", "skyhook")
    }
    sd_dbe6, ap_dbe6 = _a_peak_g(_ASCE_SD1_DBE, 6.0, _ASCE_B_ZETA02, _ISO_ZETA)
    sd_mce6, ap_mce6 = _a_peak_g(_ASCE_SM1_MCE, 6.0, _ASCE_B_ZETA02, _ISO_ZETA)
    sd_dbe4, ap_dbe4 = _a_peak_g(_ASCE_SD1_DBE, 4.0, _ASCE_B_ZETA02, _ISO_ZETA)

    # Numpy 1-DOF reference on the XS01 ground record (passive vs skyhook).
    t = pulse.ch["t_s"]
    ag = pulse.ch["a_ground_mps2"]
    ref = {
        f"{'skyhook' if sky else 'passive'}_zeta{z:g}": _sdof_isolator(t, ag, t_iso, z, sky)
        for z in ((0.2, _HAVEN_ZETA_SKY) if not quick else (_HAVEN_ZETA_SKY,))
        for sky in (False, True)
    }
    x = pulse.ch["x_m"]
    y = pulse.ch.get("y_m", np.zeros_like(x))
    v_rel = np.hypot(np.gradient(x, t), np.gradient(y, t))
    v_pk = float(np.max(v_rel))
    ke_kj = 0.5 * m_kg * v_pk**2 / 1e3

    results: dict[str, Any] = {
        "guideway_radius_m_at_T6": r_6,
        "transmissibility_zeta0p2": trans,
        "asce_a_peak": {
            "T6_dbe": {"s_d_m": sd_dbe6, "a_peak_g": ap_dbe6},
            "T6_mce": {"s_d_m": sd_mce6, "a_peak_g": ap_mce6},
            "T4_dbe": {"s_d_m": sd_dbe4, "a_peak_g": ap_dbe4},
            "inputs": {"s_d1_dbe_g": _ASCE_SD1_DBE, "s_m1_mce_g": _ASCE_SM1_MCE, "b": _ASCE_B_ZETA02, "zeta": _ISO_ZETA},
        },
        "pulse_3dof": {
            "pga_mps2": pulse.kpi("pga_mps2"),
            "pgv_mps": pulse.kpi("pgv_mps"),
            "max_radial_m": pulse.kpi("max_radial_m"),
            "buffer_onset_m": pulse.kpi("buffer_onset_m"),
            "moat_m": pulse.kpi("moat_m"),
            "peak_abs_accel_g": pulse.kpi("peak_abs_accel_g"),
            "copper_dt_k": pulse.kpi("copper_dt_k"),
            "peak_relative_speed_mps": v_pk,
            "kinetic_energy_at_peak_kj": ke_kj,
        },
        "reference_1dof": ref,
        "fling": {k: fling.kpi(k) for k in ("max_radial_m", "peak_decel_g", "bumper_energy_mj", "touchdown_mps", "moat_exceeded")},
        "control_loss": {k: loss.kpi(k) for k in ("max_radial_m", "touchdown_mps", "buffer_energy_peak_j", "buffer_rating_j")},
    }
    sky = ref[f"skyhook_zeta{_HAVEN_ZETA_SKY:g}"]
    pas = ref[f"passive_zeta{_HAVEN_ZETA_SKY:g}"]
    support = fling.criterion("support_never_lost")
    checks = [
        _check("guideway radius at T 6 s vs 8.95 m", (r_6 - 8.95) / 8.95, 0.005, "abs<=", "rel", "FAB-EXO-701 T = 2π·√(R/g) ±0.5 %", "FAB-EXO-701"),
        _check("passive transmissibility, T 4 s, 1 Hz, ζ 0.2", (trans["passive_1hz_T4s"] - 0.1251) / 0.1251, 0.02, "abs<=", "rel", "FAB-EXO-702 0.1251 ±2 %", "FAB-EXO-702"),
        _check("skyhook transmissibility, T 4 s, 1 Hz, ζ 0.2", (trans["skyhook_1hz_T4s"] - 0.0663) / 0.0663, 0.02, "abs<=", "rel", "FAB-EXO-702 0.0663 ±2 %", "FAB-EXO-702"),
        _check("near-fault pulse: max radial excursion vs the buffer onset", pulse.kpi("max_radial_m"), pulse.kpi("buffer_onset_m"), "<", "m", "FAB-EXO-703 within the moat, no buffer contact (XS01)", "FAB-EXO-703"),
        _check("near-fault pulse: no buffer contact", pulse.kpi("buffer_contact") == 0.0, True, "true", "", "FAB-EXO-703 (XS01)", "FAB-EXO-703"),
        _check("fling of 5 m: support never lost", _f(support.get("value")), _f(support.get("limit")), "<=", "m", "FAB-EXO-703 guideway extent (XS02)", "FAB-EXO-703"),
        _check("fling of 5 m: bumper energy quoted", fling.kpi("bumper_energy_mj"), 0.0, ">=", "MJ", "FAB-EXO-703 quoted in MJ (XS02)", "FAB-EXO-703"),
        _check("control loss: FPS touchdown speed", loss.kpi("touchdown_mps"), 0.1, "<=", "m/s", "FAB-EXO-704 ≤ 0.1 m/s (XS07)", "FAB-EXO-704"),
        _check("a_peak at DBE, T 6 s (analysis output)", ap_dbe6, 0.1, "<=", "g", "FAB-EXO-505 ≤ 0.1 g at DBE, S_D1 0.6 (EXOTIC §X9)", "FAB-EXO-505"),
    ]

    story = (
        f"A {m_kg / 1e3:g} t HAVEN-R platform carries a museum collection on a curved guideway of radius {r_6:.2f} m "
        f"(T = {t_iso:g} s). A Kobe-type near-fault pulse (Mw {pulse.data['inputs'].get('mw', 6.9):g}, "
        f"{pulse.kpi('pgv_mps'):g} m/s, PGA {pulse.kpi('pga_mps2') / _G0:.2f} g) moves the 3-DOF platform "
        f"{pulse.kpi('max_radial_m'):.2f} m, inside the {pulse.kpi('buffer_onset_m'):.1f} m buffer onset of its "
        f"{pulse.kpi('moat_m'):.1f} m moat, with a peak absolute acceleration of {pulse.kpi('peak_abs_accel_g'):.3f} g. "
        f"On the same record a linear 1-DOF reference with ζ {_HAVEN_ZETA_SKY:g} shows the trade: skyhook "
        f"{sky['peak_a_abs_g']:.3f} g at {sky['peak_x_rel_m']:.2f} m of stroke, a passive damper "
        f"{pas['peak_a_abs_g']:.3f} g at {pas['peak_x_rel_m']:.2f} m. At 1 Hz the closed-form transmissibility is "
        f"{trans['passive_1hz_T6s']:.4f} passive and {trans['skyhook_1hz_T6s']:.4f} skyhook (T 6 s, ζ 0.2). "
        f"At its peak relative speed of {v_pk:.2f} m/s the platform carries {ke_kj:.0f} kJ relative to the fixed moat edge, "
        f"a shear hazard that presence sensing cannot stop while the earthquake drives the platform, so the moat has "
        f"passive covers and exclusion. A 5 m fling exceeds "
        f"the moat as expected (bumper {1e3 * fling.kpi('bumper_energy_mj'):.1f} kJ = {fling.kpi('bumper_energy_mj'):.4f} MJ, "
        f"support kept), and a controller fault ends on the FPS catcher at {1e3 * loss.kpi('touchdown_mps'):.2f} mm/s. "
        f"At DBE the ASCE analysis gives a_peak {ap_dbe6:.3f} g at T 6 s against {ap_dbe4:.3f} g at T 4 s. "
        + _failed_sentence(checks)
    )

    def figure(fig: Any) -> None:
        c = _colors()
        ax1, ax2, ax3, ax4 = (fig.add_subplot(2, 2, i) for i in (1, 2, 3, 4))
        # (a) pulse through the 3-DOF platform
        ax1.plot(t, x, color=c["s1"], lw=1.8, label="platform x, 3-DOF skyhook (m)")
        onset, moat = pulse.kpi("buffer_onset_m"), pulse.kpi("moat_m")
        for s in (1, -1):
            ax1.axhline(s * onset, color=c["s4"], lw=1.0, ls=(0, (4, 3)))
            ax1.axhline(s * moat, color=c["star"], lw=1.0, ls=(0, (2, 2)))
        ax1.annotate("buffer onset", xy=(0.99, onset), xycoords=("axes fraction", "data"), ha="right", va="bottom", fontsize=7.5, color=c["ink2"])
        ax1.annotate("moat wall", xy=(0.99, moat), xycoords=("axes fraction", "data"), ha="right", va="bottom", fontsize=7.5, color=c["ink2"])
        axg = ax1.twinx()
        axg.plot(t, ag / _G0, color=c["ctrl"], lw=0.9, alpha=0.8, label="ground acceleration (g)")
        axg.set_ylabel("ground acceleration (g)", fontsize=9, color=c["muted"])
        axg.tick_params(labelsize=8)
        axg.spines["right"].set_visible(True)
        ax1.set_ylim(-1.35 * moat, 1.35 * moat)
        ax1.set_xlim(0, min(float(t[-1]), 40.0))
        _style(ax1, "time (s)", "platform displacement (m)", "Near-fault pulse through the 3-DOF platform")
        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = axg.get_legend_handles_labels()
        ax1.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8, frameon=False)
        # (b) transmissibility
        f = np.logspace(-2, math.log10(3.0), 400)
        for T, ls in ((6.0, "-"), (4.0, (0, (4, 2)))):
            ax2.loglog(f, [n.transmissibility(fi * T, _ISO_ZETA) for fi in f], color=c["s2"], ls=ls, lw=1.6, label=f"passive, T {T:g} s")
            ax2.loglog(f, [n.skyhook_transmissibility(fi * T, _ISO_ZETA) for fi in f], color=c["s1"], ls=ls, lw=1.6, label=f"skyhook, T {T:g} s")
        ax2.plot([1.0, 1.0], [0.1251, 0.0663], ls="none", marker="o", color=c["star"], ms=6, label="FAB-EXO-702 anchors (T 4 s, 1 Hz)")
        ax2.axhline(1.0, color=c["axis"], lw=1.0)
        _style(ax2, "ground-motion frequency (Hz)", "transmissibility |a_abs / a_g|", "Passive against skyhook (ζ 0.2): closed forms")
        _legend(ax2, loc="lower left", fontsize=7.5)
        # (c) stroke vs acceleration trade
        pts = [(k, v) for k, v in ref.items()]
        for k, v in pts:
            skyk = k.startswith("skyhook")
            ax3.plot(v["peak_x_rel_m"], v["peak_a_abs_g"], marker="o" if skyk else "s", ms=8, ls="none", color=c["s1"] if skyk else c["s2"], label=f"1-DOF {k.replace('_zeta', ', ζ ')}")
        ax3.plot(pulse.kpi("max_radial_m"), pulse.kpi("peak_abs_accel_g"), marker="D", ms=9, ls="none", color=c["s3"], label="3-DOF platform (skyhook ζ 0.3 + copper damper)")
        ax3.axvline(onset, color=c["s4"], lw=1.0, ls=(0, (4, 3)))
        ax3.annotate("moat buffer onset", xy=(onset, 0.02), xycoords=("data", "axes fraction"), rotation=90, ha="right", va="bottom", fontsize=7.5, color=c["ink2"])
        _hline(ax3, 0.1, "0.1 g occupant target (DBE analysis)", color=c["star"], va="bottom", x=0.55)
        ax3.set_xlim(0.4, max(1.0, onset * 1.1))
        ax3.set_ylim(0.0, 0.13)
        _style(ax3, "peak relative displacement (m)", "peak absolute acceleration (g)", "Same pulse: skyhook trades stroke for acceleration")
        _legend(ax3, loc="lower left", fontsize=7.5)
        # (d) beyond the design basis
        tf, xf = fling.ch["t_s"], fling.ch["x_m"]
        ax4.plot(tf, xf, color=c["s5"], lw=1.6, label=f"5 m fling: bumper {fling.kpi('bumper_energy_mj'):.4f} MJ, support kept")
        tl, xl = loss.ch["t_s"], np.hypot(loss.ch["x_m"], loss.ch.get("y_m", np.zeros_like(loss.ch["x_m"])))
        ax4.plot(tl, xl, color=c["s3"], lw=1.4, label=f"controller fault at 30 s: FPS touchdown {1e3 * loss.kpi('touchdown_mps'):.2f} mm/s")
        _vmark(ax4, 30.0, "controller fault", color=c["s3"], y=0.35)
        ax4.axhline(moat, color=c["star"], lw=1.0, ls=(0, (2, 2)))
        ax4.axhline(_f(support.get("limit")), color=c["ink2"], lw=1.0, ls=(0, (1, 2)))
        ax4.annotate("moat wall", xy=(0.99, moat), xycoords=("axes fraction", "data"), ha="right", va="top", fontsize=7.5, color=c["ink2"])
        ax4.annotate("guideway extent (support)", xy=(0.99, _f(support.get("limit"))), xycoords=("axes fraction", "data"), ha="right", va="bottom", fontsize=7.5, color=c["ink2"])
        _style(ax4, "time (s)", "x (fling), radial (fault) (m)", "Beyond the design basis, and control loss")
        _legend(ax4, loc="lower right", fontsize=7.5)

    return _finish(meta, out_dir, quick=quick, seed=seed, runs=runs, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t0)


# =====================================================================================
# 11. proton_shield_pwave (SHIELD, XC-SHD-1)
# =====================================================================================

_SHIELD = _Meta(
    number=11,
    name="proton_shield_pwave",
    title="a 50 t shielding door takes a P-wave in transit: beam off, stop, land and lock",
    badge="🧪",
    maturity="TRL 3 (proton-therapy vault doors in service on rollers; the passive maglev mover is a concept)",
    story=(
        "A 50 t SHIELD door slides through a proton-therapy vault inside a guarded zone with a personnel protection "
        "system. A P pick drops the beam permit on the detection cycle; the door stops, lands on a PM-bias ramp and "
        "locks within the 2 s budget. The timeline is a reduced kinematic model."
    ),
    classes=("shield",),
    requirements=(
        ("FAB-EXO-502", "SS1 stop within the analytic d_stop; ISO 13855 separation S = 2.61 m"),
        ("FAB-EXO-503", "attended jog ≤ 10 mm/s and ≤ the imported pedestrian energy cap (8.2 mm/s at 50 t); momentum stated"),
        ("FAB-EXO-504", "bumper rating ½·m_rated·v_max² = 6.25 kJ at 50 t and 0.5 m/s"),
        ("FAB-EXO-115", "the landing is an energised PM-bias ramp with a mechanical backstop, not a clamp-up"),
    ),
    caveats=(
        "Reduced model: a kinematic timeline at 1 kHz (reaction, jerk-limited SS1 stop, PM-bias-ramp landing, lock), not a "
        "closed-loop simulation; the 2 s result depends on the SS1 jerk (5 m/s³, [UNCERTAIN]), a 0.5 s landing ramp and a "
        "0.2 s lock, all swept in the figure. XC-SHD-1 has no FAB requirement of its own.",
        "Radiation ageing of magnets and sensors is unquantified, and SmCo activates to Co-60; the passive mover keeps every "
        "electronic part out of the beam line.",
        "At speed this is a guarded machine behind PL e presence sensing and a PPS, not a door anyone walks through; attended "
        "jog runs at the imported pedestrian energy cap and its momentum (about 411 kg·m/s) is stated, not capped.",
        "The P pick comes from a synthetic event (synthetic/injected, not a live EEW feed); the seismic action is not a rated "
        "safety function and nothing here is a compliance claim.",
    ),
    labels=(ALERT_LABEL,),
)

_V_QUICK = (0.1, 0.2, 0.3, 0.4, 0.5)
_J_QUICK = (2.0, 5.0, 10.0)
_V_FULL = tuple(round(v, 3) for v in np.linspace(0.05, 0.5, 10))
_J_FULL = (1.0, 2.0, 3.0, 5.0, 8.0, 10.0, 20.0)


def _shield(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    n = _native()
    seed, _ = _options(options)
    t0 = time.perf_counter()
    meta = _SHIELD
    row = _class_row("shield")
    reg = row["regime"]
    m_kg = float(row["design_mass_kg"])
    v_max = row["v_max_mmps"] / 1000.0
    a_brake = reg["a_brake_mmps2"] / 1000.0
    base = _simulate("p_wave_in_transit", "shield", "XC-SHD-1", seed, channels=True)
    vs, js = (_V_QUICK, _J_QUICK) if quick else (_V_FULL, _J_FULL)
    sweep = [_simulate(f"v{v:g}_j{j:g}", "shield", "XC-SHD-1", seed, {"v_mps": v, "j_ss1_mps3": j}) for j in js for v in vs]
    react = [] if quick else [_simulate(f"react{tr:g}", "shield", "XC-SHD-1", seed, {"t_react_s": tr}) for tr in (0.05, 0.2, 0.3, 0.5)]
    inp = base.data["inputs"]
    t_react, j_ss1 = inp["t_react_s"], inp["j_ss1_mps3"]
    d_stop_analytic = v_max * t_react + n.stop_distance(v_max, a_brake, j_ss1)
    s_iso = n.exo_iso13855_distance(v_max, a_brake)
    v_jog = min(reg["jog_v_mmps"] / 1000.0, n.v_low_energy(m_kg))
    p_jog = m_kg * v_jog
    bumper = n.bumper_rating(m_kg, v_max)
    t_total = base.kpi("t_land_and_lock_s")
    grid = {f"j{j:g}": [sw.kpi("t_land_and_lock_s") for sw in sweep if sw.overrides["j_ss1_mps3"] == j] for j in js}
    results: dict[str, Any] = {
        "timeline": {
            "t_react_s": t_react,
            "t_stop_s": base.kpi("t_stop_s"),
            "t_land_s": inp["t_land_s"],
            "t_lock_s": inp["t_lock_s"],
            "t_land_and_lock_s": t_total,
            "stop_distance_m": base.kpi("stop_distance_m"),
            "d_stop_analytic_m": d_stop_analytic,
            "landing_peak_speed_mps": base.kpi("landing_peak_speed_mps"),
            "beam_permit_drop_latency_s": base.kpi("beam_permit_drop_latency_s"),
        },
        "sensitivity": {"v_mps": list(vs), "t_land_and_lock_s_by_jerk": grid},
        "reaction_sweep": [{"t_react_s": r.overrides["t_react_s"], "t_land_and_lock_s": r.kpi("t_land_and_lock_s")} for r in react],
        "guarded_zone": {
            "iso13855_s_m": s_iso,
            "iso13855_s_class_row_m": reg["iso13855_s_mm"] / 1000.0,
            "a_brake_mps2": a_brake,
            "v_max_mps": v_max,
            "jog_speed_mps": v_jog,
            "jog_momentum_kgmps": p_jog,
            "bumper_rating_j": bumper,
            "pps": reg.get("pps"),
            "presence_pl_e": reg.get("presence_pl_e"),
        },
    }
    over = [(sw.overrides["v_mps"], sw.overrides["j_ss1_mps3"], sw.kpi("t_land_and_lock_s")) for sw in sweep if sw.kpi("t_land_and_lock_s") > 2.0]
    results["sensitivity"]["cases_over_2s"] = [{"v_mps": v, "j_mps3": j, "t_s": t} for v, j, t in over]
    checks = [
        _check("P pick to landed and locked (v 0.5 m/s, j 5 m/s³)", t_total, 2.0, "<=", "s", "XC-SHD-1 ≤ 2 s (EXOTIC §X17; reduced model)"),
        _check("beam permit withdrawn after the pick", base.kpi("beam_permit_drop_latency_s"), 0.001, "<=", "s", "XC-SHD-1: on the detection cycle (1 ms)"),
        _check("SS1 stop distance vs the analytic d_stop", base.kpi("stop_distance_m"), d_stop_analytic, "<=", "m", "FAB-EXO-502 d_stop = v·t_react + v²/(2a) + v·a/(2j)", "FAB-EXO-502"),
        _check("ISO 13855 separation S", s_iso, 2.61, "==", "m", "FAB-EXO-502 S = 2.61 m (exact)", "FAB-EXO-502"),
        _check("attended jog speed at 50 t", 1e3 * v_jog, [8.17, 8.27], "in", "mm/s", "FAB-EXO-503 8.2 mm/s at the imported energy cap", "FAB-EXO-503"),
        _check("jog momentum (stated, not capped)", p_jog, [405.0, 417.0], "in", "kg·m/s", "FAB-EXO-503 about 411 kg·m/s", "FAB-EXO-503"),
        _check("bumper rating at rated mass", bumper, 6250.0, "==", "J", "FAB-EXO-504 ½·m_rated·v_max² = 6.25 kJ", "FAB-EXO-504"),
        _check("PM-bias-ramp landing peak speed", base.kpi("landing_peak_speed_mps"), 0.1, "<=", "m/s", "FAB-EXO-115 energised landing (the 0.1 m/s landing bound of EXOTIC §X9)", "FAB-EXO-115"),
    ]
    slow = max(over, key=lambda o: (o[1], -o[0])) if over else None
    story = (
        f"A {m_kg / 1e3:g} t SHIELD door slides at {v_max:g} m/s through a proton-therapy vault, inside a guarded zone with "
        f"PL e presence sensing and a personnel protection system. A synthetic P pick drops the beam permit on the "
        f"detection cycle ({1e3 * base.kpi('beam_permit_drop_latency_s'):.0f} ms); after a {t_react:g} s reaction the SS1 ramp "
        f"(a {a_brake:g} m/s², j {j_ss1:g} m/s³) stops the door in {base.kpi('t_stop_s'):.2f} s over "
        f"{base.kpi('stop_distance_m'):.3f} m (analytic d_stop {d_stop_analytic:.3f} m), the PM-bias ramp lands it at "
        f"{1e3 * base.kpi('landing_peak_speed_mps'):.0f} mm/s peak and the lock engages {t_total:.2f} s after the pick, "
        f"inside the 2 s budget with {2.0 - t_total:.2f} s to spare. The margin is thin: "
        + (
            f"a softer SS1 jerk of {slow[1]:g} m/s³ at {slow[0]:g} m/s already takes {slow[2]:.2f} s. "
            if slow
            else "every swept speed and jerk stays within 2 s. "
        )
        + f"The zone numbers: ISO 13855 separation {s_iso:.2f} m, attended jog {1e3 * v_jog:.1f} mm/s with "
        f"{p_jog:.0f} kg·m/s of momentum stated, bumper {bumper / 1e3:.2f} kJ. "
        + _failed_sentence(checks)
    )

    def figure(fig: Any) -> None:
        c = _colors()
        ax1, ax2, ax3, ax4 = (fig.add_subplot(2, 2, i) for i in (1, 2, 3, 4))
        t = base.ch["t_s"]
        t_p = float(t[np.argmax(base.ch["beam_permit"] < 0.5)]) if np.any(base.ch["beam_permit"] < 0.5) else math.nan
        t_stop = t_p + t_react + base.kpi("t_stop_s")
        t_land = t_stop + inp["t_land_s"]
        t_lock = t_land + inp["t_lock_s"]
        ax1.fill_between(t, 0, 1e3 * v_max * 1.1, where=base.ch["beam_permit"] > 0.5, color=c["s2"], alpha=0.10, step="post", label="beam permit on")
        ax1.plot(t, 1e3 * base.ch["v_mps"], color=c["s1"], lw=2.0, label="door speed (mm/s)")
        ax1b = ax1.twinx()
        ax1b.plot(t, base.ch["x_m"], color=c["s3"], lw=1.5, label="travel x (m)")
        ax1b.set_ylabel("travel x (m)", fontsize=9, color=c["muted"])
        ax1b.tick_params(labelsize=8)
        ax1b.spines["right"].set_visible(True)
        for tt, lab in ((t_p, "P pick: beam off"), (t_stop, "stopped"), (t_land, "landed"), (t_lock, "locked")):
            _vmark(ax1, tt, lab)
        _style(ax1, "time (s)", "door speed (mm/s)", "The timeline after the P pick (v 0.5 m/s)")
        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax1b.get_legend_handles_labels()
        ax1.legend(h1 + h2, l1 + l2, loc="center left", fontsize=8, frameon=False)
        # (b) landing
        ax2.plot(t, 1e3 * base.ch["gap_m"], color=c["s4"], lw=2.0, label="suspension gap (mm)")
        ax2.axvspan(t_p, t_p + 2.0, color=c["good"], alpha=0.08, label="2 s budget from the pick")
        for tt, lab in ((t_p, "P pick"), (t_stop, "stopped"), (t_land, "landed"), (t_lock, "locked")):
            _vmark(ax2, tt, lab)
        ax2.set_ylim(-0.5, 1e3 * float(np.max(base.ch["gap_m"])) * 1.3)
        _style(ax2, "time (s)", "gap (mm)", "PM-bias-ramp landing, then the lock")
        _legend(ax2, loc="lower left")
        # (c) sensitivity
        palette = [c["s1"], c["s2"], c["s3"], c["s4"], c["s5"], c["ctrl"], c["ink2"]]
        for k, j in enumerate(js):
            ax3.plot(vs, grid[f"j{j:g}"], marker="o", ms=4, lw=1.6, color=palette[k % len(palette)], label=f"SS1 jerk {j:g} m/s³")
        _hline(ax3, 2.0, "2 s budget (XC-SHD-1)", color=c["star"], x=0.6, va="top")
        _style(ax3, "transit speed at the pick (m/s)", "pick to locked (s)", "The 2 s budget against speed and SS1 jerk")
        _legend(ax3, loc="lower right", fontsize=7.5)
        # (d) decomposition
        cases = [("v 0.5, j 5 (default)", base)] + [
            (f"v {sw.overrides['v_mps']:g}, j {sw.overrides['j_ss1_mps3']:g}", sw)
            for sw in sweep
            if (sw.overrides["v_mps"], sw.overrides["j_ss1_mps3"]) in ((0.5, js[0]), (vs[len(vs) // 2], 5.0), (vs[0], js[-1]))
        ]
        ys = np.arange(len(cases))[::-1]
        for yi, (lab, r) in zip(ys, cases, strict=True):
            parts = [(r.data["inputs"]["t_react_s"], "reaction", c["ctrl"]), (r.kpi("t_stop_s"), "SS1 stop", c["s1"]), (r.data["inputs"]["t_land_s"], "landing ramp", c["s4"]), (r.data["inputs"]["t_lock_s"], "lock", c["s3"])]
            left = 0.0
            for w_, name, col in parts:
                ax4.barh(yi, w_, left=left, color=col, height=0.55, label=name if yi == ys[0] else None)
                left += w_
            ax4.annotate(f"{left:.2f} s", xy=(left, yi), xytext=(4, 0), textcoords="offset points", va="center", fontsize=8, color=c["ink2"])
        ax4.axvline(2.0, color=c["star"], lw=1.4, ls=(0, (4, 3)))
        ax4.set_yticks(ys, [lab for lab, _ in cases], fontsize=8)
        ax4.set_xlim(0, 3.2)
        _style(ax4, "time from the P pick (s)", None, "Where the 2 s goes")
        _legend(ax4, loc="lower right", fontsize=7.5)

    return _finish(meta, out_dir, quick=quick, seed=seed, runs=[base, *sweep, *react], run_entries=[_run_entry(base), _sweep_entry("speed_jerk_sweep", sweep, {"v_mps": list(vs), "j_ss1_mps3": list(js)}), *[_run_entry(r) for r in react]], results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t0)


# =====================================================================================
# 12. vault_close_race (BASTION, XS10 + XC-SAF-1)
# =====================================================================================

_VAULT = _Meta(
    number=12,
    name="vault_close_race",
    title="a 50 t blast plug closes 3 m only if the front arrives late enough",
    badge="🧪",
    maturity="TRL 2–3 (blast doors in service are hydraulic swing doors; the maglev plug is a concept)",
    story=(
        "BASTION, a 50 t sliding plug door, gets a warning with the front's arrival time t_a as an input. The jerk-aware "
        "planner closes 3 m in 5.25 s and seals only when that fits before t_a; by the weak-shock bound t_a ≤ R/c0 a close "
        "needs a source at least about 2 km away. Earlier fronts get the fallback, and presence in the zone wins over the "
        "close unless a two-person override is active."
    ),
    classes=("bastion",),
    requirements=(
        ("FAB-EXO-807", "CLOSE iff t_min + t_seal + margin ≤ t_a − t_detect; 3 m in 5.25 s; a close needs R ≥ 2 km"),
        ("FAB-EXO-806", "Friedlander impulse and the reflection factor P_r/P_s from their closed forms"),
        ("FAB-EXO-808", "no yield-to-distance table or scaled-distance law in code, outputs or docs"),
        ("FAB-EXO-507", "presence sensing wins over closure unless a two-person override is active"),
        ("FAB-EXO-502", "intrusion mid-move stops within the analytic d_stop; ISO 13855 separation 5.01 m"),
    ),
    caveats=(
        "t_a is an input, never computed from a charge or a distance: fabrail publishes no yield-to-distance table and no "
        "scaled-distance law (FAB-EXO-808). The only distance shown is the bound t_a ≤ R/c0 (a shock is never slower than sound).",
        "Blast lead time is usable only for km-scale events; nearby fronts arrive in 0.1–0.5 s, where only normally closed doors "
        "and passive blast valves act. The operational value of a 5.25 s close is less exposure while the door must be open.",
        "The Friedlander and reflection numbers are generic closed forms of the inputs P_s, t_d and b; structural response is "
        "outside this repository (UFC 3-340-02 governs it), and fabrail is not a protective-design tool.",
        "At full speed BASTION is a guarded machine (PL e presence, ISO 13855 separation 5.01 m), not a door anyone walks "
        "through; the intrusion stop is a kinematic reduced model.",
    ),
)

_TA_QUICK = (4.0, 5.0, 6.0, 6.3, 6.4, 7.0, 8.0, 10.0)
_TA_FULL = tuple(round(x, 2) for x in np.arange(3.0, 12.01, 0.25))
#: Generic overpressures of the reflection anchors, kPa (EXOTIC §2: 2.008 / 2.741 / 7.60 at 1 / 100 / 10,000 kPa).
_PS_ANCHORS = ((1.0, 2.008), (100.0, 2.741), (10000.0, 7.60))
#: Atmospheric pressure of the Rankine–Hugoniot form, Pa (EXOTIC §X11: P0 = 101.325 kPa).
_P0_PA = 101325.0


def _friedlander_fraction(b: float) -> float:
    """i/(P_s·t_d) = 1/b − (1 − e^(−b))/b² (EXOTIC §X11)."""
    return 1.0 / b - (1.0 - math.exp(-b)) / (b * b)


def _reflection(p_s_pa: float) -> float:
    """P_r/P_s = 2·(7·P0 + 4·P_s)/(7·P0 + P_s) (Rankine–Hugoniot, γ 1.4; EXOTIC §X11)."""
    return 2.0 * (7.0 * _P0_PA + 4.0 * p_s_pa) / (7.0 * _P0_PA + p_s_pa)


def _vault(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    n = _native()
    seed, _ = _options(options)
    t0 = time.perf_counter()
    meta = _VAULT
    row = _class_row("bastion")
    base = _simulate("close_race_ta8", "bastion", "XS10", seed, channels=True)
    tas = _TA_QUICK if quick else _TA_FULL
    sweep = [_simulate(f"ta{ta:g}", "bastion", "XS10", seed, {"t_a_s": ta}) for ta in tas]
    no_det = _simulate("detect0_ta6", "bastion", "XS10", seed, {"t_detect_s": 0.0, "t_a_s": 6.0})
    bound = _simulate("range_2km_bound", "bastion", "XS10", seed, {"range_m": 2000.0})
    refl = [_simulate(f"ps{ps:g}kpa", "bastion", "XS10", seed, {"p_s_kpa": ps}) for ps, _ in _PS_ANCHORS]
    intr = _simulate("intrusion_mid_move", "bastion", "XC-SAF-1", seed, channels=True)
    inp = base.data["inputs"]
    d, a, j, vmax = inp["d_m"], inp["a_mps2"], inp["j_mps3"], inp["v_max_mps"]
    dec = {
        label: n.exo_closure_decision(d, 0.0, a, j, vmax, inp["t_seal_s"], inp["margin_s"], inp["t_a_s"], inp["t_detect_s"], clear, override)
        for label, clear, override in (("zone_clear", True, False), ("presence_in_zone", False, False), ("presence_with_two_person_override", False, True))
    }
    t_needed = base.kpi("t_needed_s")
    c0 = base.kpi("min_standoff_m") / t_needed
    decisions = [
        {
            "t_a_s": sw.overrides["t_a_s"],
            "decision": sw.data.get("labels", {}).get("decision"),
            "t_needed_s": sw.kpi("t_needed_s"),
            "t_closed_and_sealed_s": sw.kpi("t_closed_and_sealed_s"),
            "min_range_for_t_a_m": c0 * sw.overrides["t_a_s"],
        }
        for sw in sweep
    ]
    iff = all((dd["decision"] == "close") == (dd["t_needed_s"] <= dd["t_a_s"]) for dd in decisions)
    s_iso = n.exo_iso13855_distance(row["v_max_mmps"] / 1000.0, row["regime"]["a_brake_mmps2"] / 1000.0)
    keys = set(base.data.get("inputs", {}))
    forbidden = sorted(k for k in keys if any(w in k.lower() for w in ("yield", "charge", "tnt", "scaled")))
    results: dict[str, Any] = {
        "close": {
            "t_min_s": base.kpi("t_min_s"),
            "t_needed_s": t_needed,
            "t_closed_and_sealed_s": base.kpi("t_closed_and_sealed_s"),
            "peak_speed_mps": float(np.max(base.ch["v_mps"])) if base.ch else math.nan,
            "min_standoff_m": base.kpi("min_standoff_m"),
            "min_standoff_detect0_m": no_det.kpi("min_standoff_m"),
            "c0_mps_implied": c0,
            "arrival_bound_s_at_2km": bound.kpi("arrival_bound_s"),
        },
        "decisions_vs_t_a": decisions,
        "zone_decisions": dec,
        "blast_closed_forms": {
            "p_s_kpa": inp["p_s_kpa"],
            "t_d_ms": inp["t_d_ms"],
            "b": inp["b"],
            "friedlander_impulse_pa_s": base.kpi("friedlander_impulse_pa_s"),
            "friedlander_fraction_by_b": {f"{b:g}": _friedlander_fraction(b) for b in (0.5, 1.0, 2.0, 5.0)},
            "reflection_factor": {f"{ps:g}_kpa": r.kpi("reflection_factor") for (ps, _), r in zip(_PS_ANCHORS, refl, strict=True)} | {f"{inp['p_s_kpa']:g}_kpa": base.kpi("reflection_factor")},
        },
        "intrusion": {k: intr.kpi(k) for k in ("t_intrusion_s", "v_at_intrusion_mps", "d_stop_sim_m", "d_stop_analytic_m", "t_stop_s", "iso13855_s_m")},
        "input_keys": sorted(keys),
    }
    fr_imp = inp["p_s_kpa"] * 1e3 * inp["t_d_ms"] * 1e-3 * math.exp(-1.0)
    checks = [
        _check("closed and sealed at t_a 8 s vs t_a − margin", base.kpi("t_closed_and_sealed_s"), inp["t_a_s"] - inp["margin_s"], "<=", "s", "FAB-EXO-807 closed before t_a (XS10)", "FAB-EXO-807"),
        _check("jerk-aware 3 m close time t_min", base.kpi("t_min_s"), [5.24, 5.25], "in", "s", "FAB-EXO-807 3 m in 5.246 s at a 0.48, j 2", "FAB-EXO-807"),
        _check("decision is CLOSE exactly when t_needed ≤ t_a (swept t_a)", iff, True, "true", "", "FAB-EXO-807 CLOSE iff t_min + t_seal + margin ≤ t_a − t_detect", "FAB-EXO-807"),
        _check("minimum source range for a close (t_detect 0)", no_det.kpi("min_standoff_m") / 1e3, [1.95, 2.05], "in", "km", "FAB-EXO-807 a close needs R ≥ 2 km (exact anchor, rounded)", "FAB-EXO-807"),
        _check("Friedlander impulse at b = 1 vs P_s·t_d·e⁻¹", (base.kpi("friedlander_impulse_pa_s") - fr_imp) / fr_imp, 1e-9, "abs<=", "rel", "FAB-EXO-806 (rel 1e-9)", "FAB-EXO-806"),
        *[
            _check(f"reflection factor at {ps:g} kPa", round(r.kpi("reflection_factor"), 3 if ps < 1e4 else 2), anchor, "==", "", f"FAB-EXO-806 P_r/P_s = {anchor:g} (printed digits)", "FAB-EXO-806")
            for (ps, anchor), r in zip(_PS_ANCHORS, refl, strict=True)
        ],
        _check("no yield, charge or scaled-distance input exists", not forbidden, True, "true", "", "FAB-EXO-808 t_a, P_s, t_d and b are the only blast inputs", "FAB-EXO-808"),
        _check("presence in the zone: no close", not dec["presence_in_zone"]["close"], True, "true", "", "FAB-EXO-507 presence wins without the override", "FAB-EXO-507"),
        _check("presence with a two-person override: close", bool(dec["presence_with_two_person_override"]["close"]), True, "true", "", "FAB-EXO-507 the override replaces the presence channels only", "FAB-EXO-507"),
        _check("intrusion mid-move: stop distance vs analytic d_stop", intr.kpi("d_stop_sim_m"), intr.kpi("d_stop_analytic_m"), "<=", "m", "FAB-EXO-502 (XC-SAF-1, kinematic reduced model)", "FAB-EXO-502"),
        _check("ISO 13855 separation S", s_iso, 5.01, "==", "m", "FAB-EXO-502 S = 5.01 m (exact)", "FAB-EXO-502"),
    ]
    first_close = min((dd["t_a_s"] for dd in decisions if dd["decision"] == "close"), default=math.nan)
    last_fb = max((dd["t_a_s"] for dd in decisions if dd["decision"] != "close"), default=math.nan)
    story = (
        f"BASTION, a {row['design_mass_kg'] / 1e3:g} t sliding plug door, gets a warning whose front arrival time t_a is an input. "
        f"The jerk-aware planner (a {a:g} m/s², j {j:g} m/s³) closes {d:g} m in {base.kpi('t_min_s'):.3f} s at up to "
        f"{results['close']['peak_speed_mps']:.3f} m/s; with {inp['t_detect_s']:g} s of detection, {inp['t_seal_s']:g} s of seal and "
        f"{inp['margin_s']:g} s of margin it needs t_a ≥ {t_needed:.2f} s, and at t_a = {inp['t_a_s']:g} s it is sealed at "
        f"{base.kpi('t_closed_and_sealed_s'):.2f} s. Across the swept arrivals the planner falls back (land in place) up to "
        f"t_a = {last_fb:g} s and closes from {first_close:g} s. Because a shock is never slower than sound (t_a ≤ R/c0, "
        f"c0 {c0:.0f} m/s), a close needs a source at least {results['close']['min_standoff_m'] / 1e3:.2f} km away, or "
        f"{no_det.kpi('min_standoff_m') / 1e3:.2f} km with instant detection. With someone in the zone the plan does not start "
        f"({dec['presence_in_zone']['reason']}) unless a two-person override is active. An intrusion mid-move at "
        f"{intr.kpi('v_at_intrusion_mps'):.2f} m/s stops the plug in {intr.kpi('d_stop_sim_m'):.2f} m (analytic "
        f"{intr.kpi('d_stop_analytic_m'):.2f} m). For the generic front of the inputs (P_s {inp['p_s_kpa']:g} kPa, t_d "
        f"{inp['t_d_ms']:g} ms, b {inp['b']:g}) the impulse is {base.kpi('friedlander_impulse_pa_s'):.0f} Pa·s and the normal "
        f"reflection factor {base.kpi('reflection_factor'):.3f}. "
        + _failed_sentence(checks)
    )

    def figure(fig: Any) -> None:
        c = _colors()
        ax1, ax2, ax3, ax4 = (fig.add_subplot(2, 2, i) for i in (1, 2, 3, 4))
        t = base.ch["t_s"]
        ax1.plot(t, base.ch["x_m"], color=c["s1"], lw=2.0, label="plug travel x (m)")
        ax1b = ax1.twinx()
        ax1b.plot(t, base.ch["v_mps"], color=c["s2"], lw=1.4, label="speed (m/s)")
        ax1b.set_ylabel("speed (m/s)", fontsize=9, color=c["muted"])
        ax1b.tick_params(labelsize=8)
        ax1b.spines["right"].set_visible(True)
        t_seal = base.kpi("t_closed_and_sealed_s")
        for tt, lab in ((inp["t_detect_s"], "detection done"), (t_seal - inp["t_seal_s"], "closed"), (t_seal, "sealed"), (inp["t_a_s"] - inp["margin_s"], "t_a − margin"), (inp["t_a_s"], "front arrives (t_a input)")):
            _vmark(ax1, tt, lab)
        ax1.set_xlim(0, inp["t_a_s"] + 0.5)
        _style(ax1, "time after detection start (s)", "travel x (m)", f"The close at t_a = {inp['t_a_s']:g} s: 3 m in {base.kpi('t_min_s'):.2f} s")
        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax1b.get_legend_handles_labels()
        ax1.legend(h1 + h2, l1 + l2, loc="center right", fontsize=8, frameon=False)
        # (b) decision vs t_a
        ta = np.array([dd["t_a_s"] for dd in decisions])
        slack = ta - np.array([dd["t_needed_s"] for dd in decisions])
        closes = np.array([dd["decision"] == "close" for dd in decisions])
        wbar = 0.18
        ax2.bar(ta[closes], slack[closes], width=wbar, color=c["good"], label="CLOSE (sealed before t_a − margin)")
        ax2.bar(ta[~closes], slack[~closes], width=wbar, color=c["star"], alpha=0.8, label="fallback: land in place")
        ax2.axhline(0.0, color=c["axis"], lw=1.0)
        _vmark(ax2, t_needed, f"t_needed {t_needed:.2f} s")
        top = ax2.secondary_xaxis("top", functions=(lambda x: c0 * x / 1e3, lambda r: 1e3 * r / c0))
        top.set_xlabel(_mt("minimum source range for this t_a, R ≥ c0·t_a (km)"), fontsize=9, color=c["ink2"])
        top.tick_params(labelsize=8)
        _style(ax2, "front arrival time t_a after detection start (s, an input)", "t_a − t_needed (s)", "")
        ax2.set_title("Close or fall back: t_a decides", fontsize=11.5, loc="left", pad=28)
        _legend(ax2, loc="lower right", fontsize=8)
        # (c) Friedlander
        tau = np.linspace(0.0, 1.0, 300)
        for k, b in enumerate((0.5, 1.0, 2.0, 5.0)):
            ax3.plot(tau, (1 - tau) * np.exp(-b * tau), color=[c["s1"], c["s2"], c["s3"], c["s4"]][k], lw=1.7, label=f"b {b:g}: i/(P_s·t_d) = {_friedlander_fraction(b):.4f}")
        _style(ax3, "t / t_d", "P(t) / P_s", "Friedlander positive phase (generic inputs)")
        _legend(ax3, loc="upper right", fontsize=8)
        # (d) reflection
        ps = np.logspace(0, 4, 200)
        ax4.semilogx(ps, [_reflection(p * 1e3) for p in ps], color=c["s1"], lw=1.8, label="P_r/P_s = 2(7P0 + 4P_s)/(7P0 + P_s)")
        pts = [(p, r.kpi("reflection_factor")) for (p, _), r in zip(_PS_ANCHORS, refl, strict=True)] + [(inp["p_s_kpa"], base.kpi("reflection_factor"))]
        ax4.plot([p for p, _ in pts], [v for _, v in pts], ls="none", marker="o", color=c["star"], ms=6, label="XS10 runs (anchors 2.008 / 2.741 / 7.60)")
        _hline(ax4, 8.0, "strong-shock limit 8", color=c["muted"], x=0.98, va="top")
        ax4.set_ylim(1.5, 8.6)
        _style(ax4, "side-on overpressure P_s (kPa, generic input)", "normal reflection factor P_r/P_s", "Normal reflection (Rankine–Hugoniot, γ 1.4)")
        _legend(ax4, loc="center left", fontsize=8)

    run_entries = [_run_entry(base), _sweep_entry("t_a_sweep", sweep, {"t_a_s": list(tas)}), _run_entry(no_det), _run_entry(bound), *[_run_entry(r) for r in refl], _run_entry(intr)]
    return _finish(meta, out_dir, quick=quick, seed=seed, runs=[base, *sweep, no_det, bound, *refl, intr], run_entries=run_entries, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t0)


# =====================================================================================
# 13. stadium_tray_pod_failure (ARENA, XC-ARN-1)
# =====================================================================================

_STADIUM = _Meta(
    number=13,
    name="stadium_tray_pod_failure",
    title="an 8,600 t field tray loses 1 of its 500 pods: static reallocation, or LAND",
    badge="🧪",
    maturity="TRL 3; dynamic simulation 🗺️ roadmap",
    story=(
        "An 8,600 t stadium field tray rides on 500 zero-power hybrid EMS pods. When one fails, weighted least squares "
        "reallocates the weight and both moments to the other 499; the tray lands if any quadrant's healthy capacity falls "
        "below 1.3× its weight share. Only the static allocation is built; the dynamic gap error is roadmap."
    ),
    classes=("arena",),
    requirements=(
        ("FAB-EXO-103", "the pod pole area from the single admission function (ARENA row of the admission table)"),
        ("FAB-EXO-502", "intrusion mid-move stops within the analytic d_stop; ISO 13855 separation 10.61 m"),
        ("FAB-EXO-504", "bumper rating ½·m_rated·v_max² = 387 kJ at 0.3 m/s"),
    ),
    caveats=(
        "Reduced model: static weighted-least-squares allocation only. The dynamic gap error of a 500-pod tray during and "
        "after a pod loss is roadmap (🗺️), so nothing here says the tray stays within its gap during the transient.",
        "Floor flatness over 150 m against a 15–25 mm gap is the open engineering problem; the energy saving over wheels is "
        "small at 0.04–0.07 m/s.",
        "At 8,600 t this is a guarded machine (ISO 13855 separation 10.61 m, PL e presence sensing); the pedestrian caps do "
        "not apply, and the momentum at 0.3 m/s (2.58e6 kg·m/s) is stated, not capped.",
        "The pod capacity is 1.6× the weight share [UNCERTAIN design pick] and the 1.3× quadrant LAND rule is a design rule "
        "(fabrail-exo alloc), not a structural verification.",
    ),
)

#: Quadrant LAND rule of the allocator (fabrail-exo alloc::QUADRANT_CAPACITY_MIN; exotic design §3.3).
_QUADRANT_MIN = 1.3
_CAP_QUICK = (1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 2.0)
_CAP_FULL = tuple(round(x, 3) for x in np.arange(1.0, 2.501, 0.05))
_COG_QUICK = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 8.0)
_COG_FULL = tuple(round(x, 2) for x in np.arange(0.0, 10.01, 0.5))


def _stadium(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    n = _native()
    seed, _ = _options(options)
    t0 = time.perf_counter()
    meta = _STADIUM
    row = _class_row("arena")
    m_kg = float(row["design_mass_kg"])
    v_max = row["v_max_mmps"] / 1000.0
    base = _simulate("lose_most_loaded_pod", "arena", "XC-ARN-1", seed)
    model = str(base.data.get("model", ""))
    mg = re.search(r"(\d+)\s*×\s*(\d+)\s*grid,\s*([\d.]+)\s*×\s*([\d.]+)\s*m pitch", model)
    nx, ny, px, py = (int(mg.group(1)), int(mg.group(2)), float(mg.group(3)), float(mg.group(4))) if mg else (25, 20, 4.0, 4.3)
    n_pods = nx * ny
    lost = [_simulate(f"lose_pod_{i}", "arena", "XC-ARN-1", seed, {"lost_pod": float(i)}) for i in range(n_pods)]
    caps = [_simulate(f"capacity_{cr:g}", "arena", "XC-ARN-1", seed, {"capacity_ratio": cr}) for cr in (_CAP_QUICK if quick else _CAP_FULL)]
    cogs = [_simulate(f"cog_{cx:g}", "arena", "XC-ARN-1", seed, {"cog_x_m": cx, "cog_y_m": cx / 2.0}) for cx in (_COG_QUICK if quick else _COG_FULL)]
    intr = _simulate("intrusion_mid_move", "arena", "XC-SAF-1", seed)
    util = np.array([r.kpi("realloc_max_utilisation") for r in lost])
    lands = [int(r.kpi("lost_pod_index")) for r in lost if r.kpi("realloc_land") > 0]
    worst = int(np.argmax(util))
    adm = next((r for r in n.exo_admission_table() if r.get("class") == "arena"), {})
    s_iso = n.exo_iso13855_distance(v_max, row["regime"]["a_brake_mmps2"] / 1000.0)
    bumper = n.bumper_rating(m_kg, v_max)
    cap_rows = [{"capacity_ratio": r.overrides["capacity_ratio"], "nominal_max_utilisation": r.kpi("nominal_max_utilisation"), "realloc_max_utilisation": r.kpi("realloc_max_utilisation"), "outcome": r.data.get("labels", {}).get("outcome")} for r in caps]
    cog_rows = [{"cog_x_m": r.overrides["cog_x_m"], "cog_y_m": r.overrides["cog_y_m"], "nominal_max_utilisation": r.kpi("nominal_max_utilisation"), "realloc_max_utilisation": r.kpi("realloc_max_utilisation"), "outcome": r.data.get("labels", {}).get("outcome")} for r in cogs]
    land_caps = [r["capacity_ratio"] for r in cap_rows if r["outcome"] == "LAND"]
    ok_caps = [r["capacity_ratio"] for r in cap_rows if r["outcome"] != "LAND"]
    results: dict[str, Any] = {
        "layout": {"nx": nx, "ny": ny, "pitch_x_m": px, "pitch_y_m": py, "n_pods": n_pods, "pod_capacity_n": base.data["inputs"].get("pod_capacity_n")},
        "default": {k: base.kpi(k) for k in ("lost_pod_index", "nominal_max_utilisation", "realloc_max_utilisation", "realloc_residual_fz_n", "healthy_pods", "realloc_land")} | {"outcome": base.data.get("labels", {}).get("outcome")},
        "single_pod_losses": {"n": n_pods, "max_utilisation_min": float(util.min()), "max_utilisation_max": float(util.max()), "worst_pod": worst, "land_count": len(lands)},
        "capacity_sweep": cap_rows,
        "cog_sweep": cog_rows,
        "land_rule_quadrant_min": _QUADRANT_MIN,
        "admission": adm,
        "intrusion": {k: intr.kpi(k) for k in ("t_intrusion_s", "v_at_intrusion_mps", "d_stop_sim_m", "d_stop_analytic_m", "t_stop_s", "iso13855_s_m")},
        "guarded_zone": {"iso13855_s_m": s_iso, "bumper_rating_j": bumper, "momentum_at_v_max_kgmps": m_kg * v_max},
    }
    checks = [
        _check("most-loaded pod lost: reallocated max utilisation", base.kpi("realloc_max_utilisation"), 1.0, "<=", "", "XC-ARN-1 static reallocation meets the wrench (EXOTIC §X17)"),
        _check("most-loaded pod lost: outcome", base.data.get("labels", {}).get("outcome") == "reallocated", True, "true", "", "XC-ARN-1 at capacity ratio 1.6"),
        _check(f"all {n_pods} single-pod losses reallocated without LAND", len(lands), 0, "==", "pods", "XC-ARN-1 swept over every pod"),
        _check("LAND declared when a quadrant falls below 1.3× its share (capacity ratio 1.2)", any(r["outcome"] == "LAND" and r["capacity_ratio"] == 1.2 for r in cap_rows), True, "true", "", "fabrail-exo alloc LAND rule (XC-ARN-1 'else LAND')"),
        _check("intrusion mid-move: stop distance vs analytic d_stop", intr.kpi("d_stop_sim_m"), intr.kpi("d_stop_analytic_m"), "<=", "m", "FAB-EXO-502 (XC-SAF-1, kinematic reduced model)", "FAB-EXO-502"),
        _check("ISO 13855 separation S", s_iso, 10.61, "==", "m", "FAB-EXO-502 S = 10.61 m (exact)", "FAB-EXO-502"),
        _check("bumper rating at rated mass and 0.3 m/s", bumper, 387000.0, "==", "J", "FAB-EXO-504 387 kJ", "FAB-EXO-504"),
        _check("pod pole area reported by the admission function", _f(adm.get("area_m2")), 0.0, ">", "m²", "FAB-EXO-103 single admission function (reported)", "FAB-EXO-103"),
    ]
    story = (
        f"An {m_kg / 1e3:,.0f} t stadium field tray rides on {n_pods} zero-power hybrid EMS pods ({nx} × {ny}, pitch "
        f"{px:g} × {py:g} m, each rated {results['layout']['pod_capacity_n'] / 1e3:.0f} kN, 1.6× its weight share) with its "
        f"centre of gravity 2 m and 1 m off centre. When the most-loaded pod (#{int(base.kpi('lost_pod_index'))}) fails, weighted "
        f"least squares reallocates the weight and both moments to the other {int(base.kpi('healthy_pods'))}: the highest pod "
        f"utilisation rises from {base.kpi('nominal_max_utilisation'):.3f} to {base.kpi('realloc_max_utilisation'):.3f} and the tray "
        f"stays up. Over every single-pod loss the worst case is {util.max():.3f} (pod #{worst}), with no LAND. The rule that "
        f"lands the tray (a quadrant's healthy capacity below {_QUADRANT_MIN:g}× its weight share) trips at capacity ratios up to "
        f"{max(land_caps) if land_caps else math.nan:g} and clears from {min(ok_caps) if ok_caps else math.nan:g}. An intrusion "
        f"into the {s_iso:.2f} m separation zone mid-move stops the tray in {intr.kpi('d_stop_sim_m'):.2f} m (analytic "
        f"{intr.kpi('d_stop_analytic_m'):.2f} m). Only the static allocation is built; the dynamic gap error of the tray is roadmap. "
        + _failed_sentence(checks)
    )

    def figure(fig: Any) -> None:
        c = _colors()
        ax1, ax2, ax3, ax4 = (fig.add_subplot(2, 2, i) for i in (1, 2, 3, 4))
        grid = util.reshape(ny, nx)
        ext = (-(nx / 2) * px, (nx / 2) * px, -(ny / 2) * py, (ny / 2) * py)
        im = ax1.imshow(grid, origin="lower", extent=ext, cmap="Blues", aspect="auto", vmin=float(util.min()), vmax=float(util.max()))
        cb = fig.colorbar(im, ax=ax1, fraction=0.046, pad=0.02)
        cb.set_label("max pod utilisation after losing this pod", fontsize=8.5)
        cb.ax.tick_params(labelsize=8)
        ix, iy = worst % nx, worst // nx
        ax1.plot((ix - (nx - 1) / 2) * px, (iy - (ny - 1) / 2) * py, marker="s", ms=9, mfc="none", mec=c["star"], mew=1.8, ls="none", label=f"worst loss, pod #{worst}")
        ax1.plot(base.data["inputs"].get("cog_x_m", 2.0), base.data["inputs"].get("cog_y_m", 1.0), marker="+", ms=12, mew=2, color=c["s2"], ls="none", label="centre of gravity")
        _style(ax1, "x along the tray (m)", "y across the tray (m)", "Every single-pod loss, mapped at the lost pod")
        _legend(ax1, loc="upper left", fontsize=7.5)
        ax1.grid(False)
        # (b) distribution
        srt = np.sort(util)
        ax2.plot(np.arange(1, n_pods + 1), srt, color=c["s1"], lw=1.8, label=f"after losing one pod ({n_pods} cases)")
        ax2.axhline(base.kpi("nominal_max_utilisation"), color=c["ctrl"], lw=1.2, ls=(0, (4, 3)), label=f"all pods healthy {base.kpi('nominal_max_utilisation'):.3f}")
        _style(ax2, "cases, sorted", "max pod utilisation", f"Reallocation adds at most {100.0 * (float(util.max()) - base.kpi('nominal_max_utilisation')):.1f} % of pod capacity")
        _legend(ax2, loc="upper left", fontsize=8)
        # (c) capacity sweep
        cr = np.array([r["capacity_ratio"] for r in cap_rows])
        ax3.plot(cr, [r["nominal_max_utilisation"] for r in cap_rows], color=c["ctrl"], lw=1.4, marker="o", ms=3.5, label="all pods healthy")
        ax3.plot(cr, [r["realloc_max_utilisation"] for r in cap_rows], color=c["s1"], lw=1.8, marker="o", ms=4, label="most-loaded pod lost")
        for r in cap_rows:
            if r["outcome"] == "LAND":
                ax3.axvspan(r["capacity_ratio"] - 0.025, r["capacity_ratio"] + 0.025, color=c["star"], alpha=0.12, lw=0)
        ax3.plot([], [], color=c["star"], alpha=0.3, lw=8, label=f"LAND declared (a quadrant below {_QUADRANT_MIN:g}× its share)")
        _style(ax3, "total pod capacity / weight", "max pod utilisation", "How much spare capacity keeps the tray up")
        _legend(ax3, loc="upper right", fontsize=8)
        # (d) CoG sweep
        cx = np.array([r["cog_x_m"] for r in cog_rows])
        ax4.plot(cx, [r["realloc_max_utilisation"] for r in cog_rows], color=c["s2"], lw=1.8, marker="o", ms=4, label="most-loaded pod lost (capacity 1.6)")
        for r in cog_rows:
            if r["outcome"] == "LAND":
                ax4.axvspan(r["cog_x_m"] - 0.25, r["cog_x_m"] + 0.25, color=c["star"], alpha=0.12, lw=0)
        ax4.plot([], [], color=c["star"], alpha=0.3, lw=8, label="LAND declared")
        _style(ax4, "centre-of-gravity offset x (m), with y = x/2", "max pod utilisation", "An off-centre load uses the margin up")
        _legend(ax4, loc="lower right", fontsize=8)

    run_entries = [_run_entry(base), _sweep_entry("single_pod_loss_sweep", lost, {"lost_pod": f"0..{n_pods - 1}"}), _sweep_entry("capacity_sweep", caps, {"capacity_ratio": [r.overrides["capacity_ratio"] for r in caps]}), _sweep_entry("cog_sweep", cogs, {"cog_x_m": [r.overrides["cog_x_m"] for r in cogs], "cog_y_m": "cog_x_m / 2"}), _run_entry(intr)]
    return _finish(meta, out_dir, quick=quick, seed=seed, runs=[base, *lost, *caps, *cogs, intr], run_entries=run_entries, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t0)


# =====================================================================================
# 14. tidegate_offshore_m8 (TIDEGATE, XS09 + XS11)
# =====================================================================================

_TIDEGATE = _Meta(
    number=14,
    name="tidegate_offshore_m8",
    title="an offshore M8: fused feeds time the warning, and the gate closes only after the shaking, on command",
    badge="🧪",
    maturity="TRL 2 (maglev gate); triggering TRL 7–9",
    story=(
        "An M8 ruptures 100 km offshore. Land, seafloor-cable and fibre-optic feeds are timed against the analytic chain, "
        "and every km of sensor advance buys 0.154 s. The 150 t TIDEGATE does not move on the alert: it closes only after "
        "the shaking ends and only on an external decision-system command, on its wheels if the rail gap check fails."
    ),
    classes=("tidegate",),
    requirements=(
        ("FAB-EXO-804", "offshore M8 warning (land, S-net-like, DAS) within ±1 s of the analytic value"),
        ("FAB-EXO-803", "lead-time gain 0.15385 s/km (V_p 6.5) and 0.27027 s/km (V_s 3.7), every output printing its inputs"),
        ("FAB-EXO-809", "tsunami timing ∫dx/√(g·h) and Green's law; closes only after the shaking and only on command; wheels on a failed gap check"),
    ),
    caveats=(
        "Timing only: no inundation, run-up or hydrodynamic load. Debris and salt in an attractive-EMS gap are unsolved, and "
        "buoyant or hydraulic gates beat maglev physically; the transferable product is the fused trigger stack.",
        "fabrail is not an EEW provider: every feed is synthetic/injected, not a live EEW feed, and every timing output prints its "
        "inputs (latency budget 3.3 s, [UNCERTAIN] typical-order values).",
        "The decision path has a SIL 3 target (SIL 4 is unclaimable) and nothing here is a compliance claim. The gate never "
        "closes on the alert alone: the shaking ends first, then the external command.",
        "Reduced model: the TIDEGATE close is a kinematic plan after a rail gap check; its dynamic simulation is roadmap (🗺️).",
    ),
    labels=(ALERT_LABEL,),
)

_SUB_QUICK = (0.0, 0.3, 0.6, 0.9, 1.2)
_SUB_FULL = tuple(round(x, 2) for x in np.arange(0.0, 1.201, 0.1))


def _tidegate(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    n = _native()
    seed, _ = _options(options)
    t0 = time.perf_counter()
    meta = _TIDEGATE
    row = _class_row("tidegate")
    warn = _simulate("offshore_m8_warning", "tidegate", "XS09", seed)
    close = _simulate("close_on_command", "tidegate", "XS11", seed, channels=True)
    no_cmd = _simulate("no_command", "tidegate", "XS11", seed, {"command": 0.0})
    subs = [_simulate(f"subsidence_{s:g}m", "tidegate", "XS11", seed, {"subsidence_m": s}) for s in (_SUB_QUICK if quick else _SUB_FULL)]
    extra: list[_Run] = []
    if not quick:
        extra = [_simulate(f"snet_{k:g}km", "tidegate", "XS09", seed, {"snet_km": k}) for k in (15.0, 60.0, 90.0)]
        extra += [_simulate(f"path_scale_{k:g}", "tidegate", "XS11", seed, {"path_scale": k}) for k in (0.5, 2.0)]
    wi = warn.data["inputs"]
    r_site = wi["epi_offshore_km"] + wi["city_inland_km"]
    depth = wi["depth_km"]
    feeds = {}
    for key, km in (("land", wi["land_km"]), ("snet", wi["snet_km"]), ("das", wi["das_offset_km"])):
        feeds[key] = {
            "sensor_km": km,
            "t_alert_sim_s": warn.kpi(f"{key}_t_alert_sim_s"),
            "t_alert_analytic_s": warn.kpi(f"{key}_t_alert_analytic_s"),
            "warning_sim_s": warn.kpi(f"{key}_warning_sim_s"),
            "warning_analytic_s": warn.kpi(f"{key}_warning_analytic_s"),
            "blind_zone_km": warn.kpi(f"{key}_blind_zone_km"),
            "picks": warn.kpi(f"{key}_picks"),
        }
    xs = np.linspace(0.0, 150.0, 151)
    w_curve = [n.exo_eew_warning(r_site, float(x), depth, wi["v_p_kmps"], wi["v_s_kmps"], wi["latency_s"], wi["t_act_s"])["warning_s"] for x in xs]
    ck = close.kpis
    c_4km = n.exo_tsunami_speed(4000.0)
    green = n.exo_greens_law(4000.0, 10.0)
    sub_rows = [{"subsidence_m": r.overrides["subsidence_m"], "gap_error_mm": r.kpi("gap_error_mm"), "gap_tolerance_mm": r.kpi("gap_tolerance_mm"), "mode": r.data.get("labels", {}).get("close_mode"), "t_closed_s": r.kpi("t_closed_s")} for r in subs]
    follows = all((r["mode"] == "maglev") == (r["gap_error_mm"] <= r["gap_tolerance_mm"]) for r in sub_rows)
    results: dict[str, Any] = {
        "warning": {"inputs": wi, "t_s_city_s": warn.kpi("t_s_city_s"), "r_site_epicentral_km": r_site, "feeds": feeds, "gain_p_s_per_km": warn.kpi("gain_p_s_per_km"), "gain_plum_s_per_km": warn.kpi("gain_plum_s_per_km")},
        "warning_curve": {"sensor_km": xs.tolist(), "warning_s": w_curve},
        "close": {k: ck.get(k) for k in ("t_start_s", "t_move_s", "t_closed_s", "t_arrival_s", "budget_s", "gap_error_mm", "gap_tolerance_mm", "greens_law_4000_to_10")} | {"mode": close.data.get("labels", {}).get("close_mode"), "inputs": close.data["inputs"]},
        "no_command": {"mode": no_cmd.data.get("labels", {}).get("close_mode"), "t_start_s": no_cmd.kpi("t_start_s")},
        "subsidence_sweep": sub_rows,
        "tsunami": {"c_at_4000m_mps": c_4km, "greens_law_4000_to_10": green, "t_arrival_s": close.kpi("t_arrival_s")},
        "class": {"design_mass_kg": row["design_mass_kg"], "v_max_mps": row["v_max_mmps"] / 1000.0, "safe_state": row["safe_state"], "two_person_override": row["regime"].get("two_person_override")},
        "extra": [{"label": r.label, "kpis": r.kpis} for r in extra],
    }
    t_shake = close.data["inputs"]["t_shaking_s"]
    checks = [
        *[
            _check(f"{name} warning, simulated − analytic", _f(warn.criterion(f"{key}_warning_within_1s").get("value")), 1.0, "abs<=", "s", "FAB-EXO-804 ±1 s (XS09)", "FAB-EXO-804")
            for key, name in (("land", "coastal land network"), ("snet", "S-net-like seafloor"), ("das", "DAS fibre"))
        ],
        _check("lead-time gain per km, P-based", warn.kpi("gain_p_s_per_km") - 1.0 / wi["v_p_kmps"], 1e-9 * 0.15385, "abs<=", "s/km", "FAB-EXO-803 0.15385 s/km (rel 1e-9)", "FAB-EXO-803"),
        _check("lead-time gain per km, intensity-based (PLUM)", warn.kpi("gain_plum_s_per_km") - 1.0 / wi["v_s_kmps"], 1e-9 * 0.27027, "abs<=", "s/km", "FAB-EXO-803 0.27027 s/km (rel 1e-9)", "FAB-EXO-803"),
        _check("tsunami speed at 4 km depth vs √(g·h)", (c_4km - math.sqrt(_G0 * 4000.0)) / math.sqrt(_G0 * 4000.0), 1e-9, "abs<=", "rel", "FAB-EXO-809 198 m/s (1e-9)", "FAB-EXO-809"),
        _check("Green's law, 4,000 m to 10 m", (green - 400.0**0.25) / 400.0**0.25, 1e-9, "abs<=", "rel", "FAB-EXO-809 ×4.47 (1e-9)", "FAB-EXO-809"),
        _check("close starts after the shaking ends", close.kpi("t_start_s"), t_shake, ">=", "s", "FAB-EXO-809 closes only after the shaking", "FAB-EXO-809"),
        _check("no external command: no close", no_cmd.data.get("labels", {}).get("close_mode") == "no_command" and no_cmd.kpis.get("t_start_s") is None, True, "true", "", "FAB-EXO-809 only on the decision-system command", "FAB-EXO-809"),
        _check("closed with margin before the tsunami arrives", bool(close.criterion("closes_within_budget_or_wheels").get("passed")), True, "true", "", "FAB-EXO-809 closes within the budget, or wheels (XS11)", "FAB-EXO-809"),
        _check("maglev only when the gap check passes, else wheels (subsidence sweep)", follows, True, "true", "", "FAB-EXO-809 wheel fallback on rail distortion", "FAB-EXO-809"),
    ]
    sub_maglev = max((r["subsidence_m"] for r in sub_rows if r["mode"] == "maglev"), default=math.nan)
    story = (
        f"An M{wi['magnitude']:g} ruptures {wi['epi_offshore_km']:g} km offshore at {depth:g} km depth; the city sits "
        f"{wi['city_inland_km']:g} km inland and its S wave arrives {warn.kpi('t_s_city_s'):.1f} s after origin. With the "
        f"{int(wi['k_sensor'])}th station reporting and {wi['latency_s']:g} s of latency, a coastal land network "
        f"({wi['land_km']:g} km) alerts at {feeds['land']['t_alert_sim_s']:.1f} s, an S-net-like seafloor network "
        f"({wi['snet_km']:g} km) at {feeds['snet']['t_alert_sim_s']:.1f} s and a DAS fibre over the source "
        f"({wi['das_offset_km']:g} km) at {feeds['das']['t_alert_sim_s']:.1f} s, giving the city "
        f"{feeds['land']['warning_sim_s']:.1f}, {feeds['snet']['warning_sim_s']:.1f} and {feeds['das']['warning_sim_s']:.1f} s of "
        f"warning (analytic {feeds['land']['warning_analytic_s']:.1f}, {feeds['snet']['warning_analytic_s']:.1f} and "
        f"{feeds['das']['warning_analytic_s']:.1f} s): each km of sensor advance buys {warn.kpi('gain_p_s_per_km'):.5f} s. "
        f"The {row['design_mass_kg'] / 1e3:g} t TIDEGATE does not move on any of these alerts. The shaking lasts {t_shake:g} s; "
        f"after 0.6 m of subsidence the rail gap error is {close.kpi('gap_error_mm'):.1f} mm against a {close.kpi('gap_tolerance_mm'):.2f} mm "
        f"tolerance, so on the external command at {close.kpi('t_start_s'):.0f} s the gate closes on its "
        f"{close.data.get('labels', {}).get('close_mode')} in {close.kpi('t_move_s'):.1f} s, closed at {close.kpi('t_closed_s'):.1f} s, "
        f"{(close.kpi('t_arrival_s') - close.kpi('t_closed_s')) / 60.0:.0f} min before the tsunami arrives at "
        f"{close.kpi('t_arrival_s') / 60.0:.1f} min (∫dx/√(g·h) over the reference path). Maglev closes are kept up to "
        f"{sub_maglev:g} m of subsidence. Without the command the gate does not close. "
        + _failed_sentence(checks)
    )

    def figure(fig: Any) -> None:
        c = _colors()
        ax1, ax2, ax3, ax4 = (fig.add_subplot(2, 2, i) for i in (1, 2, 3, 4))
        names = [("land", "coastal land\nnetwork"), ("snet", "S-net-like\nseafloor"), ("das", "DAS fibre\nover the source")]
        xs_ = np.arange(3)
        ax1.bar(xs_ - 0.18, [feeds[k]["warning_sim_s"] for k, _ in names], 0.36, color=c["s1"], label="simulated (picks, packets, latency)")
        ax1.bar(xs_ + 0.18, [feeds[k]["warning_analytic_s"] for k, _ in names], 0.36, color=c["s1"], alpha=0.4, label="analytic chain")
        for i, (k, _) in enumerate(names):
            ax1.annotate(f"alert {feeds[k]['t_alert_sim_s']:.1f} s", xy=(i - 0.18, feeds[k]["warning_sim_s"]), ha="center", va="bottom", fontsize=7.5, color=c["ink2"])
        ax1.set_xticks(xs_, [lab for _, lab in names], fontsize=8.5)
        _style(ax1, None, "warning before the S wave at the city (s)", "Warning at the city by sensing (synthetic feeds)")
        _legend(ax1, loc="upper left", fontsize=8)
        # (b) sensor advance
        ax2.plot(xs, w_curve, color=c["s1"], lw=1.8, label="analytic W(sensor distance)")
        for k, lab in names:
            ax2.plot(feeds[k]["sensor_km"], feeds[k]["warning_sim_s"], marker="o", ms=6, ls="none", color=c["s2"])
            ax2.annotate(lab.replace("\n", " "), xy=(feeds[k]["sensor_km"], feeds[k]["warning_sim_s"]), xytext=(5, 4), textcoords="offset points", fontsize=7.5, color=c["ink2"])
        ax2.plot([], [], marker="o", ls="none", color=c["s2"], label="simulated feeds")
        _note(ax2, f"slope far from the source: −{warn.kpi('gain_p_s_per_km'):.4f} s/km (1/V_p)\nsite {r_site:g} km epicentral, depth {depth:g} km, latency {wi['latency_s']:g} s", x=0.98, y=0.96, ha="right")
        ax2.axhline(0.0, color=c["axis"], lw=1.0)
        _style(ax2, "epicentral distance of the reporting station (km)", "warning at the city (s)", "Every km of sensor advance buys 0.154 s")
        _legend(ax2, loc="lower left", fontsize=8)
        # (c) post-event timeline in minutes
        tt = close.ch.get("t_s", np.zeros(0))
        gx = close.ch.get("gate_x_m", np.zeros(0))
        bars = [
            ("shaking", 0.0, t_shake, c["s2"]),
            ("decision latency", t_shake, close.kpi("t_start_s"), c["ctrl"]),
            (f"close on {close.data.get('labels', {}).get('close_mode')}", close.kpi("t_start_s"), close.kpi("t_closed_s"), c["s3"]),
            ("margin to arrival", close.kpi("t_closed_s"), close.kpi("t_arrival_s"), c["good"]),
        ]
        for k, (lab, a0, a1, col) in enumerate(bars):
            ax3.barh(len(bars) - k, (a1 - a0) / 60.0, left=a0 / 60.0, color=col, alpha=0.85 if k < 3 else 0.25, height=0.6)
            ax3.annotate(lab, xy=(a1 / 60.0, len(bars) - k), xytext=(4, 0), textcoords="offset points", va="center", fontsize=8, color=c["ink2"])
        ax3.axvline(close.kpi("t_arrival_s") / 60.0, color=c["star"], lw=1.4, ls=(0, (4, 3)))
        ax3.annotate(f"tsunami arrives {close.kpi('t_arrival_s') / 60.0:.1f} min", xy=(close.kpi("t_arrival_s") / 60.0, 0.4), xycoords=("data", "data"), ha="right", va="center", fontsize=8, color=c["star"], xytext=(-4, 0), textcoords="offset points")
        ax3.set_yticks([])
        ax3.set_xlim(0, close.kpi("t_arrival_s") / 60.0 * 1.12)
        ax3.set_ylim(0, len(bars) + 0.8)
        if tt.size and gx.size:
            ins = ax3.inset_axes((0.52, 0.62, 0.3, 0.3))
            ins.plot(tt / 60.0, gx, color=c["s3"], lw=1.4)
            ins.set_title("gate travel (m)", fontsize=7)
            ins.tick_params(labelsize=6.5)
        _style(ax3, "time after origin (min)", None, "Close after the shaking, on the external command")
        # (d) gap check
        sx = [r["subsidence_m"] for r in sub_rows]
        ax4.plot(sx, [r["gap_error_mm"] for r in sub_rows], color=c["s1"], lw=1.6, zorder=2)
        for r in sub_rows:
            ax4.plot(r["subsidence_m"], r["gap_error_mm"], marker="o" if r["mode"] == "maglev" else "s", ms=7, ls="none", color=c["s3"] if r["mode"] == "maglev" else c["s2"], zorder=3)
        ax4.plot([], [], marker="o", ls="none", color=c["s3"], label="maglev close")
        ax4.plot([], [], marker="s", ls="none", color=c["s2"], label="wheel close (gap check failed)")
        _hline(ax4, sub_rows[0]["gap_tolerance_mm"], f"gap tolerance {sub_rows[0]['gap_tolerance_mm']:.2f} mm", color=c["star"], x=0.98)
        _style(ax4, "coastal subsidence (m; 5 % differential, 2 mrad tilt)", "rail gap error (mm)", "The rail after the earthquake decides maglev or wheels")
        _legend(ax4, loc="upper left", fontsize=8)

    runs = [warn, close, no_cmd, *subs, *extra]
    run_entries = [_run_entry(warn), _run_entry(close), _run_entry(no_cmd), _sweep_entry("subsidence_sweep", subs, {"subsidence_m": [r.overrides["subsidence_m"] for r in subs]}), *[_run_entry(r) for r in extra]]
    return _finish(meta, out_dir, quick=quick, seed=seed, runs=runs, run_entries=run_entries, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t0)


# =====================================================================================
# Registry
# =====================================================================================


def _spec_of(meta: _Meta, run: Callable[..., Mapping[str, object]]) -> ExampleSpec:
    return ExampleSpec(name=meta.name, badge=meta.badge, maturity=meta.maturity, story=_guard(meta.story, meta.name), run=run)


#: The Sovereign/Exotic gallery examples 9–14, in gallery order (plan §4b).
EXAMPLES: list[ExampleSpec] = [
    _spec_of(_SUPERTALL, _supertall),
    _spec_of(_MUSEUM, _museum),
    _spec_of(_SHIELD, _shield),
    _spec_of(_VAULT, _vault),
    _spec_of(_STADIUM, _stadium),
    _spec_of(_TIDEGATE, _tidegate),
]

#: Static facts per example (number, title, classes, requirements, caveats, labels) for docs and tests.
META: dict[str, _Meta] = {m.name: m for m in (_SUPERTALL, _MUSEUM, _SHIELD, _VAULT, _STADIUM, _TIDEGATE)}


def register_all() -> list[ExampleSpec]:
    """Register :data:`EXAMPLES` in :data:`fabrail.gallery.registry.REGISTRY` (idempotent).

    Called on import; calling it again, or after the registry already holds these very
    specs, is a no-op. A different spec already registered under one of these names is an
    error (the registry rejects duplicates).
    """
    for spec in EXAMPLES:
        if REGISTRY.get(spec.name) is spec:
            continue
        register(spec)
    return list(EXAMPLES)


def run_example(name: str, out_dir: str | Path | None = None, *, quick: bool = False, **options: Any) -> Mapping[str, object]:
    """Run exotic example ``name`` into ``out_dir`` (default ``examples/out/<name>``); returns the record."""
    from .registry import OUT_DIR

    spec = next((s for s in EXAMPLES if s.name == name), None)
    if spec is None:
        raise KeyError(f"{name!r} is not a Sovereign/Exotic example (these are: {', '.join(s.name for s in EXAMPLES)})")
    target = Path(out_dir) if out_dir is not None else OUT_DIR / name
    return spec.run(target, quick=quick, **options)


register_all()


def script_main(
    name: str,
    argv: Sequence[str] | None = None,
    *,
    doc: str | None = None,
    meta: Mapping[str, Any] | None = None,
    runner: Callable[..., Mapping[str, object]] | None = None,
) -> int:
    """Entry point of the thin ``examples/<name>.py`` scripts.

    ``[--quick] [--seed N] [--output DIR] [--workers N] [--strict]``: runs the example into
    ``DIR/<name>/`` (default ``examples/out/<name>/``), prints the story and the checks and
    returns 0 (or 1 with ``--strict`` when a check fails). ``meta``/``runner`` let
    :mod:`fabrail.gallery.frontier_examples` reuse this entry point.
    """
    import argparse

    from .registry import OUT_DIR

    m = (meta or META)[name]
    run = runner or run_example
    ap = argparse.ArgumentParser(prog=f"{name}.py", description=doc or f"{m.number}. {name}: {m.title}")
    ap.add_argument("--quick", action="store_true", help="reduced run (the test-suite default)")
    ap.add_argument("-s", "--seed", type=int, default=42, help="master seed (default: 42)")
    ap.add_argument("-o", "--output", type=Path, default=None, help=f"output root (default: {OUT_DIR})")
    ap.add_argument("--workers", type=int, default=None, help="worker threads (results do not depend on it)")
    ap.add_argument("--strict", action="store_true", help="exit 1 when any check fails")
    args = ap.parse_args(list(argv) if argv is not None else None)
    root = args.output if args.output is not None else OUT_DIR
    out = Path(root) / name
    rec = run(name, out, quick=args.quick, seed=args.seed, workers=args.workers)
    if rec.get("banner"):
        print(rec["banner"])
    print(f"{m.number}. {name} ({m.badge}) -> {out}/figure.png, record.json, story.txt")
    labels = rec.get("labels") or []
    if labels:
        print("labels: " + "; ".join(str(x) for x in labels))  # type: ignore[union-attr]
    print(rec["story"])
    for c in rec["checks"]:  # type: ignore[union-attr]
        bound = " ".join(str(x) for x in (c["op"], c["limit"], c["unit"]) if x not in ("", None))
        print(f"  [{'PASS' if c['pass'] else 'FAIL'}] {c['what']}: {c['value']} ({bound}; {c['source']})")
    ok = bool(rec["all_checks_pass"])
    return 0 if ok or not args.strict else 1


def requirement_status_drift(spec_markdown: str) -> list[str]:
    """Mismatches between :data:`REQUIREMENT_STATUS` and the St. column of the given
    EXOTIC.md + FRONTIER.md text (empty list = in sync). For the drift tests."""
    found: dict[str, str] = {}
    for line in spec_markdown.splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) >= 2 and cells[0] in REQUIREMENT_STATUS:
            found[cells[0]] = cells[-1]
    out = []
    for rid, st in REQUIREMENT_STATUS.items():
        got = found.get(rid)
        if got is None:
            out.append(f"{rid}: not found in the specification")
        elif not got.startswith(st):
            out.append(f"{rid}: gallery says {st}, specification says {got}")
    return out
