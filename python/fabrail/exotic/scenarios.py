"""The XS/XC scenario catalogue as Python objects (EXOTIC §X17; ``fabrail_exo_sim::scenarios``).

Every scenario runs a reduced model of its class in Rust (``_native.exo_simulate``,
schema ``fabrail.exo_run/1``), labelled as such, prints its inputs and reports each pass
criterion with its value and limit. This module wraps it:

- :func:`catalog` / :func:`info`: names, pass criteria (EXOTIC §X17 text), the classes a
  scenario runs on and its inputs (defaults, ranges, units);
- :func:`simulate`: one run as an :class:`ExoRun` (criteria, KPIs, inputs, labels,
  notes, and the recorded channels as numpy arrays when requested);
- :func:`run_catalog`: every (scenario, class) pair, threaded (the native call releases
  the GIL), results in catalogue order whatever the worker count;
- :func:`exo_run_entry`: an ``EXO_RUN.runs`` entry of ``docs/assets/data/exo_run.js``
  (schema ``fabrail.docs.exo_run/1``) for C2a.

A roadmap class shown on a reduced model carries ``reduced_model = True``; alerts in
these runs are synthetic.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterable, Mapping
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Any

import numpy as np

from ._util import native

__all__ = [
    "Criterion",
    "ExoRun",
    "InputSpec",
    "ScenarioInfo",
    "catalog",
    "default_class",
    "exo_run_entry",
    "info",
    "normalise_scenario",
    "run_catalog",
    "simulate",
]

EXO_RUN_SCHEMA = "fabrail.exo_run/1"


@dataclass(frozen=True)
class InputSpec:
    """One scenario input: default, range, unit and meaning (``fabrail_exo_sim::scenarios::InputSpec``)."""

    name: str
    default: float
    min: float
    max: float
    unit: str
    doc: str


@dataclass(frozen=True)
class ScenarioInfo:
    """Catalogue entry of one scenario."""

    name: str
    input: str
    criteria: str
    classes: tuple[str, ...]
    requirements: tuple[str, ...]
    inputs: tuple[InputSpec, ...]

    def input_names(self) -> tuple[str, ...]:
        """Names of the scenario's numeric inputs."""
        return tuple(i.name for i in self.inputs)


def normalise_scenario(name: str) -> str:
    """``xs12`` → ``XS12``; ``xc-red-1`` / ``xc_red_1`` → ``XC-RED-1``."""
    return str(name).strip().upper().replace("_", "-")


@lru_cache(maxsize=None)
def _info(name: str) -> ScenarioInfo:
    raw = json.loads(native().exo_scenario_info(name))
    return ScenarioInfo(
        name=str(raw["name"]),
        input=str(raw.get("input", "")),
        criteria=str(raw.get("criteria", "")),
        classes=tuple(str(c) for c in raw.get("classes", ())),
        requirements=tuple(str(r) for r in raw.get("requirements", ())),
        inputs=tuple(
            InputSpec(str(i["name"]), float(i["default"]), float(i["min"]), float(i["max"]), str(i.get("unit", "")), str(i.get("doc", ""))) for i in raw.get("inputs", ())
        ),
    )


def info(name: str) -> ScenarioInfo:
    """Catalogue entry of scenario ``name`` (case-insensitive).

    Raises:
        KeyError: unknown scenario (the message lists the catalogue).
    """
    key = normalise_scenario(name)
    names = [n for n, _ in native().exo_scenarios()]
    if key not in names:
        raise KeyError(f"unknown exotic scenario {name!r} (catalogue: {', '.join(names)})")
    return _info(key)


def catalog() -> list[ScenarioInfo]:
    """Every XS/XC scenario, in EXOTIC §X17 order."""
    return [_info(n) for n, _ in native().exo_scenarios()]


def default_class(name: str) -> str:
    """The first class a scenario runs on (e.g. XS12 → sentinel, XS10 → bastion)."""
    return info(name).classes[0]


@dataclass(frozen=True)
class Criterion:
    """One pass criterion of a run: its value, limit and detail (EXOTIC §X17 check)."""

    name: str
    passed: bool
    value: float | None
    limit: float | None
    detail: str


@dataclass(frozen=True)
class ExoRun:
    """One exotic scenario run (``fabrail.exo_run/1``), with channels as numpy arrays."""

    scenario: str
    rail_class: str
    seed: int
    model: str
    reduced_model: bool
    passed: bool
    criteria_text: str
    requirements: tuple[str, ...]
    criteria: tuple[Criterion, ...]
    kpis: dict[str, float | None]
    inputs: dict[str, Any]
    labels: dict[str, str]
    notes: tuple[str, ...]
    rate_hz: float
    channels: dict[str, np.ndarray] = field(default_factory=dict)
    units: dict[str, str] = field(default_factory=dict)
    raw: dict[str, Any] = field(default_factory=dict, repr=False)

    @property
    def t_s(self) -> np.ndarray:
        """The time channel, s (empty when channels were not requested)."""
        return self.channels.get("t_s", np.zeros(0))

    def failed_criteria(self) -> list[Criterion]:
        """The criteria that did not pass."""
        return [c for c in self.criteria if not c.passed]

    def summary(self) -> str:
        """Plain-text summary: scenario, class, model, verdict, every criterion, inputs and notes."""
        lines = [
            f"{self.scenario} · {self.rail_class} · seed {self.seed} · {'PASS' if self.passed else 'FAIL'}"
            + (" · reduced model" if self.reduced_model else ""),
            f"model: {self.model}",
            f"criteria (EXOTIC §X17): {self.criteria_text}",
        ]
        for c in self.criteria:
            val = "—" if c.value is None else f"{c.value:.6g}"
            lim = "—" if c.limit is None else f"{c.limit:.6g}"
            lines.append(f"  [{'pass' if c.passed else 'FAIL'}] {c.name}: {val} (limit {lim}) — {c.detail}")
        lines.append("inputs: " + ", ".join(f"{k}={_fmt(v)}" for k, v in sorted(self.inputs.items())))
        for k, v in sorted(self.labels.items()):
            lines.append(f"{k}: {v}")
        lines.extend(f"note: {n}" for n in self.notes)
        return "\n".join(lines)

    def to_record(self, include_channels: bool = False) -> dict[str, Any]:
        """JSON-ready ``fabrail.exo_run/1`` document (the native result, channels optional)."""
        doc = {k: v for k, v in self.raw.items() if k != "channels"}
        if include_channels and "channels" in self.raw:
            doc["channels"] = self.raw["channels"]
        return doc


def _fmt(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


def _num(v: Any) -> float | None:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _from_raw(raw: dict[str, Any]) -> ExoRun:
    chans: dict[str, np.ndarray] = {}
    units: dict[str, str] = {}
    for name, ch in dict(raw.get("channels") or {}).items():
        chans[name] = np.asarray([np.nan if x is None else x for x in ch.get("data", [])], dtype=float)
        units[name] = str(ch.get("unit", ""))
    return ExoRun(
        scenario=str(raw["scenario"]),
        rail_class=str(raw["class"]),
        seed=int(raw.get("seed", 0)),
        model=str(raw.get("model", "")),
        reduced_model=bool(raw.get("reduced_model", False)),
        passed=bool(raw.get("passed", False)),
        criteria_text=str(raw.get("criteria_text", "")),
        requirements=tuple(str(r) for r in raw.get("requirements", ())),
        criteria=tuple(
            Criterion(str(c["name"]), bool(c["passed"]), _num(c.get("value")), _num(c.get("limit")), str(c.get("detail", ""))) for c in raw.get("criteria", ())
        ),
        kpis={str(k): _num(v) for k, v in dict(raw.get("kpis", {})).items()},
        inputs=dict(raw.get("inputs", {})),
        labels={str(k): str(v) for k, v in dict(raw.get("labels", {})).items()},
        notes=tuple(str(n) for n in raw.get("notes", ())),
        rate_hz=float(raw.get("rate_hz", 0.0) or 0.0),
        channels=chans,
        units=units,
        raw=raw,
    )


def simulate(
    rail_class: str | None,
    scenario: str,
    seed: int = 42,
    inputs: Mapping[str, float] | None = None,
    channels: bool = False,
) -> ExoRun:
    """Run one exotic scenario (Rust reduced model; ``_native.exo_simulate``).

    Args:
        rail_class: exotic class (``sentinel``, ``haven_r`` …); None uses the scenario's
            first class.
        scenario: ``XS01`` … ``XS12`` or ``XC-…`` (case-insensitive).
        seed: master seed (the result is a pure function of the inputs and the seed).
        inputs: overrides of the scenario inputs (see :func:`info`); out-of-range values
            raise ``ValueError`` from the native side.
        channels: also return the recorded channels.

    Raises:
        KeyError: unknown scenario or a class the scenario does not run on.
    """
    si = info(scenario)
    cls = si.classes[0] if rail_class is None else str(rail_class).strip().lower().replace("-", "_")
    if cls not in si.classes:
        raise KeyError(f"scenario {si.name} does not run on class {cls!r} (classes: {', '.join(si.classes)})")
    if inputs:
        unknown = set(inputs) - set(si.input_names())
        if unknown:
            raise KeyError(f"{si.name}: unknown inputs {sorted(unknown)} (known: {', '.join(si.input_names())})")
    payload = json.dumps({k: float(v) for k, v in inputs.items()}) if inputs else None
    raw = json.loads(native().exo_simulate(cls, si.name, int(seed), payload, bool(channels)))
    return _from_raw(raw)


def pairs(scenarios: Iterable[str] | None = None, *, all_classes: bool = True) -> list[tuple[str, str]]:
    """(scenario, class) pairs of the catalogue, in catalogue order (``all_classes=False``: first class only)."""
    out: list[tuple[str, str]] = []
    wanted = None if scenarios is None else [normalise_scenario(s) for s in scenarios]
    for si in catalog():
        if wanted is not None and si.name not in wanted:
            continue
        for cls in si.classes if all_classes else si.classes[:1]:
            out.append((si.name, cls))
    return out


def run_catalog(
    scenarios: Iterable[str] | None = None, *, all_classes: bool = True, seed: int = 42, workers: int | None = None, channels: bool = False
) -> list[ExoRun]:
    """Run every (scenario, class) pair; results in catalogue order, identical for any worker count."""
    jobs = pairs(scenarios, all_classes=all_classes)
    if not jobs:
        return []
    n = max(1, min(len(jobs), workers or 4))
    if n == 1:
        return [simulate(c, s, seed, channels=channels) for s, c in jobs]
    with ThreadPoolExecutor(max_workers=n) as pool:
        return list(pool.map(lambda job: simulate(job[1], job[0], seed, channels=channels), jobs))


def exo_run_entry(run: ExoRun, max_points: int = 2000) -> dict[str, Any]:
    """An ``EXO_RUN.runs`` entry (``fabrail.docs.exo_run/1``) for ``docs/assets/data/exo_run.js``.

    Badge and maturity come from the class preset; channels are decimated to at most
    ``max_points`` samples (the time channel moves to ``t_s``); non-finite KPIs are
    omitted (the schema wants numbers).
    """
    from .presets import preset

    p = preset(run.rail_class)
    badge_code = {"Verified": "V", "Simplified": "S", "Research": "R", "Roadmap": "M", "Frontier": "frontier"}[p.badge]
    t = run.t_s
    step = max(1, int(math.ceil(t.size / max_points))) if t.size else 1
    chans = {
        name: {"unit": run.units.get(name, ""), "data": [None if not math.isfinite(x) else float(x) for x in arr[::step]]}
        for name, arr in run.channels.items()
        if name != "t_s"
    }
    return {
        "class": run.rail_class,
        "scenario": run.scenario,
        "badge": badge_code,
        "maturity": p.maturity,
        "reduced_model": run.reduced_model,
        "rate_hz": run.rate_hz / step if run.rate_hz else 0.0,
        "t_s": [float(x) for x in t[::step]],
        "channels": chans,
        "kpis": {k: v for k, v in run.kpis.items() if v is not None},
        "inputs": {k: (float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else str(v)) for k, v in run.inputs.items()},
    }
