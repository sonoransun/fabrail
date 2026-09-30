"""Gallery examples 1–7: the core-tier examples (plan §4b gallery table).

Each example composes ``fabrail.scenario/1`` runs of the real closed loop (the
``fabrail-rt`` executive over the f64 plant, voltage coil mode, so SIL ≡ firmware
holds for the Carriage6 runs), reduces them to numbers, checks those numbers against
the requirement they demonstrate and writes three files into ``out_dir``:

- ``figure.png``: a four-panel figure (house style, :mod:`fabrail.plotstyle`);
- ``record.json``: a ``fabrail.example/1`` record (canonical JSON, ``sha256`` over the
  payload without ``wall_s``/``timing``), with every number the story and the figure
  quote, the checks with their tolerance and source, the runs and their events;
- ``story.txt``: the story line with the run's own numbers and the required caveat.

Nothing here is tuned to pass: every example runs the class presets (plus the stated,
validated overrides of a derived device), every check keeps its requirement's
tolerance, and a failing check is reported in the record, the story and the figure,
never hidden. The badge is the core status of the weakest requirement an example
demonstrates (FAB-EXA-002); the technology maturity is a separate field.

| # | Example | Badge | Class and tier |
|---|---|---|---|
| 1 | ``kitchen_push_open`` | ✅ | DRAWER, Carriage6 |
| 2 | ``pantry_cabinet_slider`` | ✅ | 8 kg slider derived from DRAWER, Carriage6 |
| 3 | ``office_door_obstruction`` | 🔶 | DOOR, Carriage6 (FAB-SAF-004 is S) |
| 4 | ``ballroom_partition`` | ✅ | WALL, Carriage6 (+ Travel1d full traverse in full runs) |
| 5 | ``archive_heavy_shelving`` | ✅ | HEAVY, Carriage6 (+ Travel1d in full runs) |
| 6 | ``drawer_power_loss`` | ✅ | DRAWER, Carriage6 |
| 7 | ``wall_pwave_m7`` | 🔶 | WALL, Travel1d (FAB-SEI-010 is S; plan table says ✅) |

``run(out_dir, *, quick=False, seed=42, workers=None)`` is the frozen calling
convention (:mod:`fabrail.gallery.registry`). Quick runs are the test-suite default;
full runs add masses, gestures, seeds or tiers.

Owner: E3 (Wave 2a).
"""

from __future__ import annotations

import math
import re
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .registry import REGISTRY, ExampleSpec, register

__all__ = [
    "BANNED_WORDS",
    "EXAMPLES",
    "META",
    "RECORD_SCHEMA",
    "REQUIREMENT_STATUS",
    "register_all",
    "requirement_status_drift",
    "run_example",
    "script_main",
    "story_is_clean",
]

#: Schema id of an example record.
RECORD_SCHEMA = "fabrail.example/1"

#: Standard gravity, m/s² (CGPM 1901; PHYSICS §1).
_G0 = 9.80665

#: The banned-vocabulary list of SPECIFICATION_EXAMPLES §0.3, kept as the quoted prompt
#: block it is there (the guard ignores quoted lines; FAB-EXA-003).
_BANNED_QUOTED = "> perfect, perfectly, absurdly, guarantee, earthquake-proof, no damage, protects"
#: Banned vocabulary outside quoted prompt blocks (SPECIFICATION_EXAMPLES §0.3; FAB-EXA-003).
BANNED_WORDS: tuple[str, ...] = tuple(w.strip() for w in _BANNED_QUOTED.lstrip("> ").split(","))
_BANNED_RE = re.compile(r"\b(" + "|".join(re.escape(w) + (r"\w*" if w.isalpha() else "") for w in BANNED_WORDS) + r")\b", re.IGNORECASE)

#: SPECIFICATION.md status of every requirement an example here demonstrates (§0.5:
#: V verified in simulation, S simplified, R research, M roadmap). Mirrors the St.
#: column; ``tests`` owners drift-check it against the spec.
REQUIREMENT_STATUS: dict[str, str] = {
    "FAB-HAP-001": "V",
    "FAB-HAP-008": "V",
    "FAB-HAP-012": "V",
    "FAB-HAP-013": "V",
    "FAB-HAP-016": "V",
    "FAB-LEV-017": "V",
    "FAB-LEV-019": "V",
    "FAB-SAF-001": "V",
    "FAB-SAF-002": "V",
    "FAB-SAF-003": "V",
    "FAB-SAF-004": "S",
    "FAB-SAF-005": "V",
    "FAB-SAF-006": "V",
    "FAB-SAF-009": "V",
    "FAB-SAF-010": "V",
    "FAB-SEI-009": "V",
    "FAB-SEI-010": "S",
}

#: Badge of each core status (registry.BADGES inverted).
_STATUS_BADGE = {"V": "✅", "S": "🔶", "R": "🧪", "M": "🗺️"}
_STATUS_ORDER = ("M", "R", "S", "V")  # weakest first

#: Monitor verdicts that stop motion (a "monitor trip"; FaultCode 0x06xx with Severity ≥ SafeStop).
_MONITOR_TRIPS = frozenset(
    {"EnergyLimit", "MomentumLimit", "ForceLimit", "Overspeed", "PositionLimit", "UnintendedMotion", "IsolateLimit"}
)

#: Event kinds kept in a record's event summary (plant contact edges only on first occurrence).
_SUMMARY_KINDS = frozenset(
    {
        "Command",
        "Rejected",
        "LevState",
        "FaultRaised",
        "FaultCleared",
        "Demand",
        "Intent",
        "Unseated",
        "Lifted",
        "Landed",
        "Seated",
        "TerminusCapture",
        "Arrived",
        "Obstruction",
        "Reversal",
        "Freeze",
        "Protect",
        "SeismicState",
        "Trigger",
    }
)
_MAX_SUMMARY_EVENTS = 80


# =====================================================================================
# Runs
# =====================================================================================


@dataclass
class _Job:
    """One scenario run to do: label, class, parameters, spec and series."""

    label: str
    rail_class: str
    params: Any
    spec: dict[str, Any]
    series: dict[str, bytes] = field(default_factory=dict)
    overrides: dict[str, Any] = field(default_factory=dict)


@dataclass
class _Done:
    """A finished run (the job plus its decoded result)."""

    job: _Job
    result: Any  # fabrail.results.RunResult
    wall_s: float

    @property
    def r(self) -> Any:
        return self.result


def _native() -> Any:
    from .. import require_native

    return require_native()


def _params(rail_class: str, seed: int, overrides: Mapping[str, Any] | None = None) -> Any:
    """Class preset + ``overrides`` in voltage coil mode (the SIL ≡ firmware mode)."""
    from ..config import RunConfig

    cfg = RunConfig(rail_class=rail_class, seed=seed, coil="voltage", overrides=dict(overrides or {}))
    return cfg.params()


def _channels(params: Any) -> list[str]:
    """Default recorder channels plus the analysis extras (reference, energy ledger, Pd/τc)."""
    from ..analysis import analysis_channel_names

    return analysis_channel_names(int(params.get("design.suspension.n_magnets")))


def _noise_seeds(seed: int, n: int) -> list[int]:
    """Per-run Rust noise seeds: ``SeedSequence(seed).spawn(n)[i]`` (fabrail.ensemble)."""
    from ..ensemble import member_seeds

    return [m.noise_seed for m in member_seeds(seed, n)]


def _spec(
    params: Any,
    rail_class: str,
    name: str,
    duration_s: float,
    *,
    initial: Mapping[str, Any],
    events: Sequence[Mapping[str, Any]],
    noise_seed: int,
    fidelity: str = "carriage6",
    rate_hz: float = 1000.0,
    windows: Mapping[str, Sequence[float]] | None = None,
    plant: Mapping[str, Any] | None = None,
    doc: str = "",
) -> dict[str, Any]:
    """A validated ``fabrail.scenario/1`` dict (schema + the Rust cross-field rules)."""
    from .. import scenarios

    wins = {k: [float(a), min(float(b), float(duration_s))] for k, (a, b) in (windows or {}).items() if a < min(b, duration_s)}
    spec = {
        "schema": scenarios.SCENARIO_SCHEMA,
        "name": f"{rail_class}_{name}",
        "doc": (doc or f"gallery example run {name}")[:4096],
        "duration_s": float(duration_s),
        "noise_seed": int(noise_seed),
        "fidelity": {"model": fidelity, "coil": "voltage", "eddy_lag": True, "kinematic_heave": True},
        "record": {"rate_hz": float(rate_hz), "channels": _channels(params)},
        "initial": dict(initial),
        "plant": dict(plant or {}),
        "series": {},
        "events": sorted((dict(e) for e in events), key=lambda e: float(e["t_s"])),
        "metrics": {"windows": wins},
    }
    scenarios.validate_spec(spec)
    scenarios.cross_check(spec, {}, params)
    return spec


def _run_jobs(jobs: Sequence[_Job], workers: int | None) -> list[_Done]:
    """Run ``jobs`` through ``_native.run_batch`` (index order; worker-count invariant)."""
    from .. import scenarios
    from ..results import RunResult

    n = _native()
    batch = [(j.params, n.Scenario.from_json(scenarios.to_json(j.spec), j.series)) for j in jobs]
    t0 = time.perf_counter()
    data = n.run_batch(batch, workers)
    wall = time.perf_counter() - t0
    out = []
    for j, d in zip(jobs, data, strict=True):
        out.append(_Done(j, RunResult.from_native(d), wall / max(1, len(jobs))))
    return out


# =====================================================================================
# Reductions shared by the examples
# =====================================================================================


def _f(x: Any) -> float:
    try:
        v = float(x)
    except (TypeError, ValueError):
        return math.nan
    return v


def _rnd(x: Any, nd: int = 6) -> float | None:
    """Round for the record (significant digits); None for non-finite."""
    v = _f(x)
    if not math.isfinite(v):
        return None
    if v == 0.0:
        return 0.0
    return float(f"{v:.{nd}g}")


def _events(r: Any, kind: str, after: float = -math.inf, before: float = math.inf) -> list[Any]:
    return [e for e in r.events if e.kind == kind and after <= e.t_s < before]


def _first(r: Any, kind: str, after: float = -math.inf, before: float = math.inf) -> Any:
    ev = _events(r, kind, after, before)
    return min(ev, key=lambda e: e.t_s) if ev else None


def _t(ev: Any) -> float:
    return float(ev.t_s) if ev is not None else math.nan


def _fault_events(r: Any) -> list[tuple[float, str, str]]:
    from ..analysis import fault_name, fault_severity

    return [(float(e.t_s), fault_name(e.a), fault_severity(e.a)) for e in r.events if e.kind == "FaultRaised"]


def _monitor_trips(r: Any) -> list[dict[str, Any]]:
    return [{"t_s": _rnd(t), "fault": name} for t, name, _ in _fault_events(r) if name in _MONITOR_TRIPS]


def _event_summary(r: Any) -> list[dict[str, Any]]:
    """Controller and first-occurrence plant events in time order (capped)."""
    from ..analysis import fault_name, state_name

    names = {"LevState": "lev_state", "Mode": "motion_mode", "Demand": "demand", "SeismicState": "seismic_state"}
    seen_plant: set[str] = set()
    out: list[dict[str, Any]] = []
    for e in sorted(r.events, key=lambda e: e.t_s):
        k = e.kind
        if k.startswith("plant:"):
            if k in seen_plant or k == "plant:end_stop":
                continue
            seen_plant.add(k)
            out.append({"t_s": _rnd(e.t_s), "kind": k, "a": _rnd(e.a), "b": _rnd(e.b)})
        elif k.startswith("scenario:"):
            out.append({"t_s": _rnd(e.t_s), "kind": k})
        elif k in _SUMMARY_KINDS:
            item: dict[str, Any] = {"t_s": _rnd(e.t_s), "kind": k}
            if k in ("FaultRaised", "FaultCleared"):
                item["what"] = fault_name(e.a)
            elif k in names:
                item["what"] = state_name(names[k], e.a)
            else:
                item["a"], item["b"] = _rnd(e.a), _rnd(e.b)
            out.append(item)
        if len(out) >= _MAX_SUMMARY_EVENTS:
            break
    return out


def _run_entry(d: _Done) -> dict[str, Any]:
    """The record's description of one run."""
    j, r = d.job, d.result
    return {
        "label": j.label,
        "rail_class": j.rail_class,
        "scenario": j.spec["name"],
        "fidelity": j.spec["fidelity"]["model"],
        "coil": j.spec["fidelity"]["coil"],
        "duration_s": j.spec["duration_s"],
        "noise_seed": j.spec["noise_seed"],
        "initial": j.spec["initial"],
        "plant": j.spec["plant"],
        "overrides": {k: v for k, v in j.overrides.items()},
        "scenario_events": j.spec["events"],
        "faults": sorted({name for _, name, _ in _fault_events(r)}),
        "monitor_trips": _monitor_trips(r),
        "events": _event_summary(r),
        "wall_s": round(d.wall_s, 3),
    }


def _kpi(r: Any, name: str, window: str | None = None) -> float:
    key = f"{window}.{name}" if window else name
    return float(r.kpis.get(key, math.nan))


def _terminus_traces(r: Any) -> list[dict[str, Any]]:
    """Distance-to-go of every terminus capture (plant truth x, recorded rate) with a semilog fit.

    For each ``TerminusCapture`` (a = d_b, b = τ_eff): the target is the reference at the
    following ``Arrived`` (else the last sample), d = sign·(target − x); the fit is the
    least-squares line of ln d over d_ε < d ≤ d_b (PHYSICS §16.1; the recorded-sample
    counterpart of the base-rate Rust KPI ``terminus_r2_min``).
    """
    from ..analysis import TERMINUS_D_EPS_M

    t = r.t_s
    x = np.asarray(r["x_m"], dtype=np.float64)
    xr = np.asarray(r["x_ref_m"], dtype=np.float64) if "x_ref_m" in r else x
    out = []
    for cap in sorted(_events(r, "TerminusCapture"), key=lambda e: e.t_s):
        t_c, d_b, tau = float(cap.t_s), float(cap.a), float(cap.b)
        arr = _first(r, "Arrived", after=t_c)
        t_end = float(arr.t_s) if arr is not None else float(t[-1])
        i0 = int(np.searchsorted(t, t_c - 1e-12))
        i1 = min(int(np.searchsorted(t, t_end + 0.05)), len(t) - 1)
        i_arr = min(int(np.searchsorted(t, t_end - 1e-12)), len(t) - 1)
        target = float(xr[i_arr])
        sign = 1.0 if target >= float(x[i0]) else -1.0
        d = sign * (target - x[i0 : i1 + 1])
        ts = t[i0 : i1 + 1] - t_c
        fit = {"tau_fit_s": math.nan, "r2": math.nan, "t_fit_s": [], "d_fit_m": []}
        below = np.flatnonzero(d <= d_b * (1.0 + 1e-9))
        if below.size:
            k0 = int(below[0])
            after = np.flatnonzero(d[k0:] <= TERMINUS_D_EPS_M)
            k1 = k0 + int(after[0]) if after.size else d.size
            win = np.arange(k0, k1)
            win = win[d[win] > TERMINUS_D_EPS_M]
            if win.size >= 5:
                tt, ln_d = ts[win], np.log(d[win])
                slope, icpt = np.polyfit(tt, ln_d, 1)
                res = ln_d - (slope * tt + icpt)
                ss = float(np.sum((ln_d - ln_d.mean()) ** 2))
                fit = {
                    "tau_fit_s": -1.0 / slope if slope < 0 else math.nan,
                    "r2": 1.0 - float(np.sum(res**2)) / ss if ss > 0 else math.nan,
                    "t_fit_s": [float(tt[0]), float(tt[-1])],
                    "d_fit_m": [float(math.exp(slope * tt[0] + icpt)), float(math.exp(slope * tt[-1] + icpt))],
                }
        out.append(
            {
                "t_capture_s": t_c,
                "t_arrived_s": _t(arr),
                "d_b_m": d_b,
                "tau_eff_s": tau,
                "target_m": target,
                "direction": "open" if sign > 0 else "close",
                "t_rel_s": ts,
                "d_m": d,
                **fit,
            }
        )
    return out


def _apparent_mass(r: Any, t0: float, t1: float, mass_ratio: float) -> dict[str, float]:
    """Least-squares M of F_hand = M·a + B·v over hand-on ``Admittance`` samples in [t0, t1)
    (the FAB-HAP-001 definition of fabrail-sim metrics, restricted to one gesture), and
    M_v = median(m̂)/mass_ratio over the same samples (the supervisor's virtual mass)."""
    t = r.t_s
    f, a, v = (np.asarray(r[c], dtype=np.float64) for c in ("f_hand_n", "a_mps2", "v_mps"))
    mode = np.asarray(r["motion_mode"])
    mask = (t >= t0) & (t < t1) & (mode == 3) & (np.abs(f) > 1e-9) & np.isfinite(f) & np.isfinite(a) & np.isfinite(v)
    out = {"m_app_kg": math.nan, "b_app_n_s_per_m": math.nan, "m_v_kg": math.nan, "m_hat_kg": math.nan, "n_samples": int(mask.sum())}
    if mask.sum() < 20:
        return out
    design = np.column_stack([a[mask], v[mask]])
    if np.linalg.matrix_rank(design) < 2:
        return out
    coef, *_ = np.linalg.lstsq(design, f[mask], rcond=None)
    m_hat = float(np.median(np.asarray(r["m_hat_kg"], dtype=np.float64)[mask]))
    out.update(m_app_kg=float(coef[0]), b_app_n_s_per_m=float(coef[1]), m_hat_kg=m_hat, m_v_kg=m_hat / mass_ratio)
    return out


# =====================================================================================
# Checks, badges, records, stories
# =====================================================================================


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
    """One acceptance check. ``op`` is one of <=, <, >=, >, ==, abs<= (|value| ≤ limit), in
    (limit is [lo, hi]), true (value must be truthy). Non-finite values fail."""
    v = value
    if op == "true":
        ok = bool(v)
    elif op == "in":
        lo, hi = limit
        ok = math.isfinite(_f(v)) and lo <= _f(v) <= hi
    else:
        fv, fl = _f(v), _f(limit)
        if not (math.isfinite(fv) and math.isfinite(fl)):
            ok = False
        else:
            ok = {
                "<=": fv <= fl,
                "<": fv < fl,
                ">=": fv >= fl,
                ">": fv > fl,
                "==": fv == fl,
                "abs<=": abs(fv) <= fl,
            }[op]
    out: dict[str, Any] = {
        "what": what,
        "value": v if isinstance(v, (bool, str)) or v is None else _rnd(v, 7),
        "op": op,
        "limit": [_rnd(limit[0]), _rnd(limit[1])] if op == "in" else (limit if isinstance(limit, (bool, str)) or limit is None else _rnd(limit, 7)),
        "unit": unit,
        "pass": bool(ok),
        "source": source,
    }
    if req:
        out["requirement"] = req
    if note:
        out["note"] = note
    return out


def _badge_rule(reqs: Sequence[Mapping[str, str]]) -> dict[str, Any]:
    statuses = [REQUIREMENT_STATUS[r["id"]] for r in reqs]
    weakest = min(statuses, key=_STATUS_ORDER.index) if statuses else "V"
    return {
        "weakest_status": weakest,
        "badge_cap": _STATUS_BADGE[weakest],
        "rule": "no example's badge exceeds the weakest requirement it demonstrates (FAB-EXA-002)",
    }


def story_is_clean(text: str) -> bool:
    """True when ``text`` uses none of :data:`BANNED_WORDS` (FAB-EXA-003 guard)."""
    return _BANNED_RE.search(text) is None


def _guard(text: str, where: str) -> str:
    m = _BANNED_RE.search(text)
    if m:
        raise ValueError(f"{where}: banned word {m.group(0)!r} (SPECIFICATION_EXAMPLES §0.3; FAB-EXA-003)")
    return text


def _fmt(x: Any, spec: str = ".3g", unit: str = "") -> str:
    v = _f(x)
    if not math.isfinite(v):
        return "n/a"
    s = format(v, spec)
    return f"{s} {unit}".rstrip() if unit else s


def _pct(x: Any) -> str:
    v = _f(x)
    return "n/a" if not math.isfinite(v) else f"{100.0 * v:+.1f} %"


def _failed_sentence(checks: Sequence[Mapping[str, Any]]) -> str:
    bad = [c for c in checks if not c["pass"]]
    if not bad:
        return f"All {len(checks)} checks pass."

    def one(c: Mapping[str, Any]) -> str:
        v, lim, op = c["value"], c["limit"], c["op"]
        if v is None:
            got = "not reached"
        elif isinstance(v, bool):
            got = "no" if not v else "yes"
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
    requirements: tuple[tuple[str, str], ...]  # (id, what the example shows)
    caveats: tuple[str, ...]


def _finish(
    meta: _Meta,
    out_dir: Path,
    *,
    quick: bool,
    seed: int,
    runs: Sequence[_Done],
    results: Mapping[str, Any],
    checks: Sequence[Mapping[str, Any]],
    story: str,
    figure: Callable[[Any], None],
    wall_s: float,
    extra: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Write figure.png, record.json and story.txt; return the record."""
    from .. import __version__, export, plotstyle

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    story = _guard(" ".join(story.split()), f"{meta.name} story")
    reqs = [{"id": rid, "status": REQUIREMENT_STATUS[rid], "shows": what} for rid, what in meta.requirements]
    first = runs[0].result.meta if runs else {}
    record: dict[str, Any] = {
        "schema": RECORD_SCHEMA,
        "number": meta.number,
        "name": meta.name,
        "title": meta.title,
        "badge": meta.badge,
        "status": {v: k for k, v in _STATUS_BADGE.items()}[meta.badge],
        "maturity": meta.maturity,
        "label": "simulated; checked in simulation, not measured on hardware",
        "quick": bool(quick),
        "seed": int(seed),
        "command": f"python -m fabrail example run {meta.name}" + (" --quick" if quick else ""),
        "script": f"examples/{meta.name}.py",
        "fabrail_version": __version__,
        "build_profile": first.get("build_profile"),
        "golden": first.get("golden"),
        "requirements": reqs,
        "badge_rule": _badge_rule(reqs),
        "caveats": list(meta.caveats),
        "results": results,
        "checks": list(checks),
        "checks_passed": sum(1 for c in checks if c["pass"]),
        "checks_total": len(checks),
        "all_checks_pass": all(c["pass"] for c in checks),
        "runs": [_run_entry(d) for d in runs],
        "story": story,
        "outputs": {"figure": "figure.png", "record": "record.json", "story": "story.txt"},
        "timing": {"wall_s": round(wall_s, 3)},
    }
    if extra:
        record.update(dict(extra))
    for c in meta.caveats:
        _guard(c, f"{meta.name} caveat")

    from matplotlib.figure import Figure

    plotstyle.apply()
    fig = Figure(figsize=(12.0, 8.4))
    figure(fig)
    fig.suptitle(f"{meta.number}. {meta.name}: {meta.title}", x=0.01, y=0.995, ha="left", fontsize=plotstyle.FONT["suptitle"])
    status = "all checks pass" if record["all_checks_pass"] else f"{record['checks_passed']}/{record['checks_total']} checks pass"
    word = {"V": "V (verified in simulation)", "S": "S (simplified model)", "R": "R (research)", "M": "M (roadmap)"}[record["status"]]
    fig.text(0.01, 0.952, f"badge {word}  ·  {status}  ·  simulated in voltage coil mode, not measured on hardware", ha="left", va="top", fontsize=9.5, color=plotstyle.COLORS["ink2"])
    stamp = {**{k: first.get(k) for k in ("version", "golden", "build_profile", "fidelity", "coil")}, "scenario": meta.name, "seed": seed}
    plotstyle.run_stamp(fig, stamp)
    fig.tight_layout(rect=(0.0, 0.02, 1.0, 0.93))
    plotstyle.save(fig, out_dir / "figure.png")

    record["sha256"] = export.payload_sha256(record)
    export.write_json(out_dir / "record.json", record)
    caveat_text = "\n".join(f"- {c}" for c in meta.caveats)
    (out_dir / "story.txt").write_text(
        f"{meta.number}. {meta.name} ({meta.badge}; maturity: {meta.maturity})\n\n{story}\n\nCaveats:\n{caveat_text}\n",
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


def _style(ax: Any, xlabel: str | None, ylabel: str | None, title: str) -> None:
    from .. import plotstyle

    plotstyle.style_ax(ax, xlabel, ylabel, title)
    ax.title.set_fontsize(11.5)


def _vmark(ax: Any, t: float, label: str, color: str | None = None, y: float = 0.97) -> None:
    from .. import plotstyle

    if not math.isfinite(_f(t)):
        return
    c = color or plotstyle.COLORS["muted"]
    ax.axvline(t, color=c, linestyle=(0, (3, 3)), linewidth=1.0, zorder=1)
    ax.annotate(label, xy=(t, y), xycoords=("data", "axes fraction"), rotation=90, ha="right", va="top", fontsize=7.5, color=plotstyle.COLORS["ink2"], xytext=(-2, 0), textcoords="offset points")


def _legend(ax: Any, **kw: Any) -> None:
    from .. import plotstyle

    handles, _ = ax.get_legend_handles_labels()
    if handles:
        ax.legend(**{**plotstyle.LEGEND_KW, "fontsize": 8.5, **kw})


def _semilog_panel(ax: Any, traces: Sequence[Mapping[str, Any]], colors: Sequence[str], labels: Sequence[str], tau_s: float) -> None:
    """Distance-to-go on a log axis: a straight line of slope −1/τ is the log-damped signature."""
    from .. import plotstyle

    for tr, c, lab in zip(traces, colors, labels, strict=False):
        d, ts = np.asarray(tr["d_m"]), np.asarray(tr["t_rel_s"])
        pos = d > 1e-6
        r2 = tr.get("r2", math.nan)
        ax.semilogy(ts[pos], d[pos] * 1e3, color=c, lw=1.8, label=f"{lab} (R² {_fmt(r2, '.5f')}, τ fit {_fmt(tr.get('tau_fit_s'), '.3f')} s)")
        if tr.get("t_fit_s"):
            ax.semilogy(tr["t_fit_s"], np.asarray(tr["d_fit_m"]) * 1e3, color=plotstyle.COLORS["star"], lw=1.0, ls=(0, (4, 3)))
    ax.axhline(0.2, color=plotstyle.COLORS["muted"], lw=1.0, ls=(0, (2, 3)))
    ax.annotate("d_ε = 0.2 mm (quintic finish)", xy=(0.99, 0.2), xycoords=("axes fraction", "data"), ha="right", va="bottom", fontsize=7.5, color=plotstyle.COLORS["muted"])
    ax.plot([], [], color=plotstyle.COLORS["star"], lw=1.0, ls=(0, (4, 3)), label=f"semilog least-squares fit (design τ = {tau_s:g} s)")
    _style(ax, "time since capture (s)", "distance to go (mm, log)", "Log-damped terminus: ln d is a straight line")
    _legend(ax, loc="lower left")


#: Class colours and the fixed categorical order (fabrail.plotstyle; assigned by entity).
CLASS_DRAWER = "#2a78d6"
SERIES_ORDER = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4")


# =====================================================================================
# 1. kitchen_push_open (DRAWER)
# =====================================================================================

_KITCHEN = _Meta(
    number=1,
    name="kitchen_push_open",
    title="a kitchen drawer that opens from a push and closes on a log-damped curve",
    badge="✅",
    maturity="concept; hybrid PM-biased EMS and long-stator LSM are established components, no maglev drawer product exists",
    story=(
        "A kitchen DRAWER rests in its captive seat. A push unseats it and it glides open on an S-curve that "
        "ends in a log-damped terminus; a guiding hand feels an apparent mass M_v = m̂/1.5 within ±10 % across "
        "8–48 kg; a tap closes it through the same terminus into the seat. Haptic lightness does not reduce "
        "impact energy."
    ),
    requirements=(
        ("FAB-HAP-012", "push-to-open from the seat: detect, unseat and first-motion latencies"),
        ("FAB-HAP-001", "apparent mass within ±10 % of M_v across 8–48 kg"),
        ("FAB-HAP-013", "semilog R² ≥ 0.999 of the open and close termini"),
        ("FAB-HAP-016", "arrival ≤ 1 mm/s, overshoot ≤ 0.02 mm, t_arrive within ±2 % of the closed form"),
        ("FAB-LEV-017", "unseat and slot exit across the rated mass range"),
    ),
    caveats=(
        "Haptic lightness does not reduce impact energy: the drawer's kinetic energy is ½·m·v² of its physical "
        "mass, whatever mass the hand feels (core non-claim 6).",
        "Contactless travel is limited to 0.30 m (≤ 50 % extension); beyond that a drawer runs on rollers.",
        "Simulated with the class preset in voltage coil mode; not measured on hardware.",
    ),
)

#: Guiding gesture for the apparent-mass fit: 3 N, below the DRAWER intent threshold
#: (4.5 N), so the drawer stays in admittance for the whole gesture (the fit excludes
#: intent-triggered motion by construction).
_GUIDE_N = 3.0
#: A firmer push used when the 2·F_th tap does not unseat the drawer (reported as such).
_FIRM_PUSH_N = 20.0


def _kitchen(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    from .. import _native as nat
    from .. import human

    seed, workers = _options(options)
    t_start = time.perf_counter()
    p = _params("drawer", seed)
    f_th = float(p.get("control.intent.f_th_n"))
    ratio = float(p.get("control.haptic.mass_ratio"))
    m_star = float(p.get("design.carriage.m_star_kg"))
    x0 = float(p.get("design.rail.x_closed_m"))
    travel = float(p.get("design.rail.travel_m"))
    masses = [8.0, 26.0, 48.0] if quick else [8.0, 16.0, 26.0, 37.0, 48.0]
    seeds = _noise_seeds(seed, 3 + 2 * len(masses))

    # Stage 1: the push-to-open tap at 2·F_th, and one open + guide run per mass.
    tap_t = 0.5
    tap = human.hand_profile("push_release", -2.0 * f_th, tap_t, 0.08, hold_s=0.1)
    jobs = [
        _Job(
            "tap_2fth",
            "drawer",
            p,
            _spec(p, "drawer", "kitchen_tap", 3.0, initial={"lev": "seated", "x_m": x0}, events=[tap], noise_seed=seeds[0], windows={"push": [tap_t, 3.0]}),
        )
    ]
    t_guide = 3.6
    for i, m in enumerate(masses):
        guide = human.hand_profile("pull", -_GUIDE_N, t_guide, 0.1, hold_s=0.2)
        jobs.append(
            _Job(
                f"mass_{m:g}kg",
                "drawer",
                p,
                _spec(
                    p,
                    "drawer",
                    f"kitchen_m{m:g}",
                    5.2,
                    initial={"lev": "seated", "x_m": x0, "payload_kg": m - m_star},
                    events=[{"kind": "command", "t_s": 0.5, "cmd": "open"}, guide],
                    noise_seed=seeds[1 + i],
                    windows={"open": [0.5, 3.5], "guide": [t_guide, 5.2]},
                ),
            )
        )
    stage1 = _run_jobs(jobs, workers)
    tap_run = stage1[0].result
    tapped = _first(tap_run, "Unseated") is not None

    # Stage 2: the full flow at m* (push-to-open, guide, push-and-go close into the seat),
    # plus a levitated guide run for any mass whose slot exit failed.
    f_push = 2.0 * f_th if tapped else _FIRM_PUSH_N
    t_g2, t_close = 4.0, 5.2
    flow_events = [
        human.hand_profile("push_release", -f_push, tap_t, 0.08, hold_s=0.1),
        human.hand_profile("pull", -_GUIDE_N, t_g2, 0.1, hold_s=0.2),
        human.hand_profile("push_release", -2.0 * f_th, t_close, 0.08, hold_s=0.1),
    ]
    jobs2 = [
        _Job(
            "flow",
            "drawer",
            p,
            _spec(
                p,
                "drawer",
                "kitchen_flow",
                9.0,
                initial={"lev": "seated", "x_m": x0},
                events=flow_events,
                noise_seed=seeds[1 + len(masses)],
                windows={"open": [tap_t, 3.9], "guide": [t_g2, 5.1], "close": [t_close, 9.0]},
            ),
        )
    ]
    failed = []
    for d in stage1[1:]:
        m = float(d.job.label.split("_")[1][:-2])
        if _first(d.result, "Arrived", after=0.5, before=3.5) is None:
            failed.append(m)
    for k, m in enumerate(failed):
        jobs2.append(
            _Job(
                f"mass_{m:g}kg_levitated",
                "drawer",
                p,
                _spec(
                    p,
                    "drawer",
                    f"kitchen_m{m:g}_lev",
                    2.6,
                    initial={"lev": "levitated", "x_m": x0 + travel, "payload_kg": m - m_star},
                    events=[human.hand_profile("pull", -_GUIDE_N, 1.0, 0.1, hold_s=0.2)],
                    noise_seed=seeds[2 + len(masses) + k],
                    windows={"guide": [1.0, 2.6]},
                ),
            )
        )
    stage2 = _run_jobs(jobs2, workers)
    flow = stage2[0].result
    lev_runs = {float(d.job.label.split("_")[1][:-2]): d.result for d in stage2[1:]}

    # --- push-to-open latencies (HAP-012), from hand contact ---
    def _pto(r: Any, t0: float) -> dict[str, Any]:
        intent = _first(r, "Intent", after=t0)
        uns = _first(r, "Unseated", after=t0)
        t = r.t_s
        x = np.asarray(r["x_m"])
        moved = np.flatnonzero((t >= t0) & (x >= x0 + 1e-3))
        confirm = _kpi(r, "intent_confirm_s", "push") if "push.intent_confirm_s" in r.kpis else _t(intent) - t0
        return {
            "push_n": None,
            "detect_s": _rnd(confirm),
            "intent_event_s": _rnd(_t(intent) - t0),
            "unseat_s": _rnd(_t(uns) - t0),
            "first_motion_s": _rnd(float(t[moved[0]]) - t0 if moved.size else math.nan),
            "unseated": uns is not None,
            "m_hat_at_unseat_kg": _rnd(uns.a) if uns is not None else None,
        }

    tap_res = _pto(tap_run, tap_t)
    tap_res["push_n"] = _rnd(2.0 * f_th)
    flow_pto = _pto(flow, tap_t)
    flow_pto["push_n"] = _rnd(f_push)
    x_arr = np.asarray(tap_run["x_m"])
    tap_res["seat_compression_max_mm"] = _rnd(1e3 * max(0.0, x0 - float(x_arr.min())))

    # --- per-mass apparent mass and cruise energy ---
    per_mass = []
    for d in stage1[1:]:
        m = float(d.job.label.split("_")[1][:-2])
        r = d.result
        uns = _first(r, "Unseated", after=0.5)
        arrived = _first(r, "Arrived", after=0.5, before=3.5)
        t = r.t_s
        v = np.asarray(r["v_mps"])
        w_open = (t >= 0.5) & (t < 3.5)
        # Cruise speed of the open move (only meaningful when the move completed).
        v_cruise = float(np.max(np.abs(v[w_open]))) if (w_open.any() and arrived is not None) else math.nan
        src = "open from the seat, guided at the open end"
        rr, g0, g1 = r, t_guide, 5.2
        if arrived is None and m in lev_runs:
            rr, g0, g1 = lev_runs[m], 1.0, 2.6
            src = "slot exit failed; guided from a levitated start at the open end (m̂ boots at m_rated)"
        am = _apparent_mass(rr, g0, g1, ratio)
        err = am["m_app_kg"] / am["m_v_kg"] - 1.0 if math.isfinite(am["m_v_kg"]) else math.nan
        m_v = am["m_v_kg"] if math.isfinite(am["m_v_kg"]) else m / ratio
        per_mass.append(
            {
                "mass_kg": m,
                "slot_exit_ok": arrived is not None,
                "faults_open": sorted({n for t_, n, _ in _fault_events(r) if t_ < 3.5}),
                "m_hat_at_unseat_kg": _rnd(uns.a) if uns is not None else None,
                "unseat_time_s": _rnd(_kpi(r, "unseat_time_s", "open")),
                "m_app_kg": _rnd(am["m_app_kg"]),
                "m_v_kg": _rnd(am["m_v_kg"]),
                "m_hat_kg": _rnd(am["m_hat_kg"]),
                "apparent_mass_err": _rnd(err),
                "apparent_mass_err_kpi": _rnd(_kpi(rr, "apparent_mass_err", "guide")),
                "fit_samples": am["n_samples"],
                "guide_source": src,
                "v_cruise_mps": _rnd(v_cruise),
                "ke_physical_j": _rnd(0.5 * m * v_cruise**2),
                "ke_rendered_j": _rnd(0.5 * m_v * v_cruise**2),
                "m_gov_kg": _rnd(max(float(p.get("design.carriage.m_rated_kg")), 1.05 * _f(uns.a) if uns is not None else 0.0)),
            }
        )

    # --- termini of the flow run ---
    traces = _terminus_traces(flow)
    v_open = float(p.derived()["v_open_cap_mps"])
    j_t = float(p.get("control.motion.terminus.j_mps3"))
    tau = float(p.get("control.motion.terminus.tau_s"))
    blend = nat.terminus_blend(v_open, j_t, tau)
    t_cf = nat.terminus_t_arrive(v_open, j_t, tau)
    term = {
        "count": int(_kpi(flow, "terminus_count")) if math.isfinite(_kpi(flow, "terminus_count")) else len(traces),
        "r2_min": _rnd(_kpi(flow, "terminus_r2_min")),
        "tau_fit_s": _rnd(_kpi(flow, "terminus_tau_fit_s")),
        "v_arrive_max_mps": _rnd(_kpi(flow, "terminus_v_arrive_max_mps")),
        "overshoot_max_m": _rnd(_kpi(flow, "terminus_overshoot_max_m")),
        "t_arrive_err_max": _rnd(_kpi(flow, "terminus_t_arrive_err_max")),
        "tau_design_s": tau,
        "closed_form": {"v_mps": _rnd(v_open), "j_mps3": j_t, "t_b_s": _rnd(blend["t_b_s"]), "d_b_m": _rnd(blend["d_b_m"]), "d_0_m": _rnd(blend["d_0_m"]), "t_arrive_s": _rnd(t_cf)},
        "per_terminus": [
            {k: (_rnd(v) if isinstance(v, float) else v) for k, v in tr.items() if k not in ("t_rel_s", "d_m", "t_fit_s", "d_fit_m")}
            for tr in traces
        ],
    }
    seated_ev = _first(flow, "Seated", after=t_close)
    flow_res = {
        "push_to_open": flow_pto,
        "open_arrived_s": _rnd(_t(_first(flow, "Arrived", after=tap_t, before=t_g2))),
        "close_intent_s": _rnd(_t(_first(flow, "Intent", after=t_close))),
        "seated_s": _rnd(_t(seated_ev)),
        "x_final_m": _rnd(float(np.asarray(flow["x_m"])[-1])),
        "guide": {k: _rnd(v) if isinstance(v, float) else v for k, v in _apparent_mass(flow, t_g2, 5.1, ratio).items()},
        "terminus": term,
        "monitor_trips": _monitor_trips(flow),
        "faults": sorted({n for _, n, _ in _fault_events(flow)}),
    }

    # --- checks ---
    checks = [
        _check("push-to-open detect (2·F_th tap, from hand contact)", tap_res["detect_s"], 0.060, "<=", "s", "FAB-HAP-012 ≤ 60 ms", "FAB-HAP-012"),
        _check("push-to-open unseat (2·F_th tap, from hand contact)", tap_res["unseat_s"], 0.100, "<=", "s", "FAB-HAP-012 ≤ 100 ms", "FAB-HAP-012"),
        _check("push-to-open first motion (x ≥ x_closed + 1 mm, from hand contact)", tap_res["first_motion_s"], 0.160, "<=", "s", "FAB-HAP-012 ≤ 160 ms", "FAB-HAP-012"),
    ]
    for pm in per_mass:
        checks.append(
            _check(f"apparent mass error at {pm['mass_kg']:g} kg (M_app/M_v − 1)", pm["apparent_mass_err"], 0.10, "abs<=", "", "FAB-HAP-001 ±10 %", "FAB-HAP-001")
        )
    heavy = max(per_mass, key=lambda q: q["mass_kg"])
    cruised = [q for q in per_mass if q["ke_physical_j"] is not None]
    ke_top = max(cruised, key=lambda q: q["mass_kg"]) if cruised else heavy
    tt, xx = flow.t_s, np.asarray(flow["v_mps"])
    v_flow = float(np.max(np.abs(xx[(tt >= tap_t) & (tt < t_g2)])))
    checks.append(
        _check(f"unseat and slot exit at {heavy['mass_kg']:g} kg (open terminus reached)", heavy["slot_exit_ok"], True, "true", "", "FAB-LEV-017 (rated 48 kg)", "FAB-LEV-017")
    )
    checks += [
        _check("terminus semilog R² (min over open and close, base-rate truth)", term["r2_min"], 0.999, ">=", "", "FAB-HAP-013 R² ≥ 0.999", "FAB-HAP-013"),
        _check("terminus arrival speed (max)", term["v_arrive_max_mps"], 1e-3, "<=", "m/s", "FAB-HAP-016 ≤ 1 mm/s", "FAB-HAP-016"),
        _check("terminus overshoot (max)", term["overshoot_max_m"], 2e-5, "<=", "m", "FAB-HAP-016 ≤ 0.02 mm", "FAB-HAP-016"),
        _check("terminus t_arrive error vs closed form (max)", term["t_arrive_err_max"], 0.02, "<=", "", "FAB-HAP-016 ±2 %", "FAB-HAP-016"),
        _check("drawer back in its seat after the push-and-go close", seated_ev is not None, True, "true", "", "plan §4b kitchen flow"),
        _check("monitor trips in the flow run", len(flow_res["monitor_trips"]), 0, "==", "count", "a monitor trip in an example is a failure (FAB-EXA-007 rule)"),
    ]
    results = {
        "rail_class": "drawer",
        "m_star_kg": m_star,
        "mass_ratio": ratio,
        "f_th_n": f_th,
        "guide_force_n": _GUIDE_N,
        "tap": tap_res,
        "flow": flow_res,
        "per_mass": per_mass,
        "e_cap_j": float(p.get("safety.e_max_j")),
        "v_cruise_flow_mps": _rnd(v_flow),
        "v_open_cap_mps": _rnd(v_open),
    }

    # --- story ---
    em = [abs(q["apparent_mass_err"]) for q in per_mass if q["apparent_mass_err"] is not None]
    tap_txt = (
        f"A {2 * f_th:g} N tap (2·F_th) unseats it: intent after {_fmt(tap_res['detect_s'] * 1e3 if tap_res['detect_s'] is not None else None, '.0f')} ms, "
        f"out of the seat after {_fmt((tap_res['unseat_s'] or math.nan) * 1e3, '.0f')} ms, first motion after "
        f"{_fmt((tap_res['first_motion_s'] or math.nan) * 1e3, '.0f')} ms (FAB-HAP-012 asks 60 / 100 / 160 ms)."
        if tapped
        else f"A {2 * f_th:g} N tap (2·F_th) compresses the seat push-stop by only {_fmt(tap_res['seat_compression_max_mm'], '.3f')} mm and does not unseat it in this build; "
        f"a firmer {f_push:g} N push does (intent {_fmt((flow_pto['intent_event_s'] or math.nan) * 1e3, '.0f')} ms and unseat {_fmt((flow_pto['unseat_s'] or math.nan) * 1e3, '.0f')} ms after contact)."
    )
    story = (
        f"A {m_star:g} kg kitchen DRAWER rests in its captive seat. {tap_txt} It glides open on an S-curve at "
        f"{_fmt(v_flow, '.4f')} m/s (class cap {_fmt(v_open, '.4f')} m/s; the governor plans with m_gov = max(m_rated, 1.05·m̂)) and enters the log-damped terminus: distance-to-go falls as d_b·e^(−t/τ) with semilog "
        f"R² ≥ {_fmt(term['r2_min'], '.5f')} and τ fit {_fmt(term['tau_fit_s'], '.3f')} s (design {tau:g} s), arriving at "
        f"≤ {_fmt((term['v_arrive_max_mps'] or math.nan) * 1e3, '.2f')} mm/s with t_arrive within {_fmt((term['t_arrive_err_max'] or math.nan) * 100, '.2f')} % of the closed form "
        f"{_fmt(t_cf, '.3f')} s. A {_GUIDE_N:g} N guiding hand feels M_app = {_fmt(flow_res['guide']['m_app_kg'], '.1f')} kg against "
        f"M_v = m̂/{ratio:g} = {_fmt(flow_res['guide']['m_v_kg'], '.1f')} kg; across {min(masses):g}–{max(masses):g} kg the apparent-mass error "
        f"stays within {_fmt(max(em) * 100 if em else math.nan, '.1f')} % (limit 10 %). A {2 * f_th:g} N tap closes it through the same terminus into the seat"
        f"{' (seated at ' + _fmt(flow_res['seated_s'], '.2f') + ' s)' if seated_ev is not None else ' (it did not re-seat in this run)'}. "
        f"Haptic lightness does not reduce impact energy: at {ke_top['mass_kg']:g} kg the drawer cruises with "
        f"{_fmt(ke_top['ke_physical_j'], '.2f')} J of kinetic energy (½·m·v²) while the hand feels only the {_fmt(ke_top['m_v_kg'], '.1f')} kg of the rendered mass. "
        f"{'Slot exit at ' + format(heavy['mass_kg'], 'g') + ' kg failed (' + ', '.join(heavy['faults_open']) + '); its apparent mass was measured from a levitated start. ' if not heavy['slot_exit_ok'] else ''}"
        f"{_failed_sentence(checks)}"
    )

    def figure(fig: Any) -> None:
        from .. import plotstyle

        C = plotstyle.COLORS
        ax = fig.subplots(2, 2)
        a0, a1, a2, a3 = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]
        t = flow.t_s
        a0.plot(t, np.asarray(flow["x_m"]) * 1e3, color=C["s1"], label="drawer x (plant truth)")
        if "x_ref_m" in flow:
            a0.plot(t, np.asarray(flow["x_ref_m"]) * 1e3, color=C["ctrl"], lw=1.0, ls=(0, (4, 3)), label="reference")
        _vmark(a0, tap_t, f"push {f_push:g} N")
        _vmark(a0, _t(_first(flow, "Unseated")), "unseated")
        _vmark(a0, t_g2, f"guide {_GUIDE_N:g} N")
        _vmark(a0, t_close, f"tap {2 * f_th:g} N")
        _vmark(a0, _t(seated_ev), "seated")
        for tr in traces:
            a0.axvspan(tr["t_capture_s"], tr["t_arrived_s"] if math.isfinite(tr["t_arrived_s"]) else tr["t_capture_s"], color=C["band"], alpha=0.10)
        _style(a0, "time (s)", "travel x (mm)", f"Push, glide, guide, close ({m_star:g} kg)")
        _legend(a0, loc="center right")
        _semilog_panel(a1, traces, [C["s1"], C["s2"], C["s3"]], [f"{tr['direction']} terminus" for tr in traces], tau)
        ms = np.array([q["mass_kg"] for q in per_mass])
        mv = np.array([_f(q["m_v_kg"]) for q in per_mass])
        mapp = np.array([_f(q["m_app_kg"]) for q in per_mass])
        mv_line = np.where(np.isfinite(mv), mv, ms / ratio)
        a2.fill_between(ms, 0.9 * mv_line, 1.1 * mv_line, color=C["band"], alpha=0.12, label="M_v ± 10 % (FAB-HAP-001)")
        a2.plot(ms, mv_line, color=C["ctrl"], lw=1.2, label=f"M_v = m̂/{ratio:g} (rendered)")
        a2.plot(ms, ms, color=C["muted"], lw=1.0, ls=(0, (2, 3)), label="physical mass m")
        a2.plot(ms, mapp, "o", color=C["s1"], ms=7, label=f"apparent mass (fit of a {_GUIDE_N:g} N guide)")
        for q in per_mass:
            if q["apparent_mass_err"] is not None:
                a2.annotate(_pct(q["apparent_mass_err"]), (q["mass_kg"], _f(q["m_app_kg"])), textcoords="offset points", xytext=(6, -12), fontsize=8, color=C["ink2"])
        _style(a2, "drawer mass (kg)", "mass felt by the hand (kg)", "Apparent mass across the rated range")
        _legend(a2, loc="upper left")
        w = 0.38
        idx = np.arange(len(per_mass))
        a3.bar(idx - w / 2, np.nan_to_num([_f(q["ke_physical_j"]) for q in per_mass]), w, color=C["s2"], label="½·m·v² (physical: what an impact sees)")
        a3.bar(idx + w / 2, np.nan_to_num([_f(q["ke_rendered_j"]) for q in per_mass]), w, color=C["s1"], label="½·M_v·v² (rendered: what the hand feels)")
        for i_, q in enumerate(per_mass):
            if q["ke_physical_j"] is None:
                a3.annotate("slot exit failed:\nno cruise", (i_, 0.05), ha="center", va="bottom", fontsize=8, color=C["ink2"])
        plotstyle.ref_line(a3, results["e_cap_j"], f"E cap {results['e_cap_j']:g} J", color=C["star"])
        a3.set_xticks(idx, [f"{q['mass_kg']:g} kg" for q in per_mass])
        a3.set_ylim(0.0, 1.45 * results["e_cap_j"])
        _style(a3, None, "kinetic energy at cruise (J)", "Lightness is rendered; impact energy is physical")
        _legend(a3, loc="upper left")

    return _finish(_KITCHEN, out_dir, quick=quick, seed=seed, runs=[*stage1, *stage2], results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t_start)


# =====================================================================================
# 2. pantry_cabinet_slider (8 kg slider derived from DRAWER)
# =====================================================================================

_PANTRY = _Meta(
    number=2,
    name="pantry_cabinet_slider",
    title="an 8 kg sliding cabinet door derived from DRAWER, with detents and a latch",
    badge="✅",
    maturity="concept; derived from the DRAWER class (no maglev cabinet slider product exists)",
    story=(
        "An 8 kg sliding cabinet door is a DRAWER-derived device (egress role None, seismic latch fitted): "
        "it unseats and glides open, renders a haptic detent to a guiding hand, and a tap closes it at the "
        "low-energy close speed through a log-damped terminus (τ = 0.2 s) into its seat, where the latch holds it."
    ),
    requirements=(
        ("FAB-HAP-008", "detent peak force within ±10 % of the 3 N / 2 mm profile"),
        ("FAB-HAP-013", "semilog R² ≥ 0.999 of the close terminus"),
        ("FAB-HAP-016", "arrival ≤ 1 mm/s, overshoot ≤ 0.02 mm, t_arrive within ±2 % of the closed form"),
    ),
    caveats=(
        "The slider is derived from DRAWER (m* = m_rated = 8 kg, pole length rebalanced to hold 8 kg at g*, "
        "close cap 0.152 m/s, terminus τ 0.2 s and j 5 m/s³); these overrides are validated by fabrail-params, "
        "not a separate product class. It is not on an egress path (EgressRole::None).",
        "Intent confirms by force × time only (dx_th = 0) so a guiding hand can pass the detents without "
        "starting an automatic move; the close is then started by a firm 20 N tap (a 2·F_th tap separates "
        "from the light slider before the force-time threshold).",
        "The latch rating assumes a design floor acceleration of 1.0 g; the latch is not a rated safety function.",
        "Simulated; not measured on hardware.",
    ),
)

#: Assumed design floor acceleration for the latch rating, m/s² (example assumption, stated).
_LATCH_A_DESIGN = 1.0 * _G0
_LATCH_RETENTION_N = 100.0


def _pantry_overrides(base: Any) -> dict[str, Any]:
    """The 8 kg DRAWER-derived slider (validated by ``Params.with_overrides``)."""
    m = 8.0
    scale = m / float(base.get("design.carriage.m_star_kg"))
    ov: dict[str, Any] = {
        "design.carriage.m_star_kg": m,
        "design.carriage.m_rated_kg": m,
        "design.carriage.m_op_min_kg": m,
        "safety.mass_rated_kg": m,
        # PM force at g* scales with pole area: rebalance to hold 8 kg at the same g*.
        "design.suspension.pole_l_m": float(base.get("design.suspension.pole_l_m")) * scale,
        "control.motion.terminus.tau_s": 0.2,
        "control.motion.terminus.j_mps3": 5.0,
        "safety.v_close_max_mps": 0.152,
        "safety.v_close_sensed_mps": 0.152,
        "control.intent.dx_th_m": 0.0,
        "control.haptic.n_detents": 2,
        "control.haptic.detent_x_m.0": 0.10,
        "control.haptic.detent_x_m.1": 0.20,
        "design.landing.seismic_latch.enabled": True,
        "design.landing.seismic_latch.x_retention_n": _LATCH_RETENTION_N,
    }
    for i in range(3):
        ov[f"design.carriage.j_kgm2.{i}"] = float(base.get(f"design.carriage.j_kgm2.{i}")) * scale
    return ov


def _pantry(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    from .. import _native as nat
    from .. import human

    seed, workers = _options(options)
    t_start = time.perf_counter()
    base = _params("drawer", seed)
    ov = _pantry_overrides(base)
    p = _params("drawer", seed, ov)
    ratio = float(p.get("control.haptic.mass_ratio"))
    b_v = float(p.get("control.haptic.b_v_n_s_per_m"))
    x0 = float(p.get("design.rail.x_closed_m"))
    detent_x = float(p.get("control.haptic.detent_x_m.1"))
    detent_f = float(p.get("control.haptic.detent_f_n"))
    detent_w = float(p.get("control.haptic.detent_width_m"))
    gestures = [4.0] if quick else [3.0, 3.5, 4.0]
    seeds = _noise_seeds(seed, 1 + len(gestures))

    t_open, t_close = 0.5, 3.8
    # With dx_th = 0 intent confirms by force × time (F_th for t_th): a 2·F_th tap on the
    # light slider separates before t_th, so the close is started by a firm tap.
    flow_events = [{"kind": "command", "t_s": t_open, "cmd": "open"}, human.hand_profile("push_release", -_FIRM_PUSH_N, t_close, 0.08, hold_s=0.1)]
    jobs = [
        _Job(
            "flow",
            "drawer",
            p,
            _spec(p, "drawer", "pantry_flow", 8.5, initial={"lev": "seated", "x_m": x0}, events=flow_events, noise_seed=seeds[0], windows={"open": [t_open, t_close], "close": [t_close, 8.5]}),
            overrides=ov,
        )
    ]
    arm = {"m_kg": 1.5, "b_n_s_per_m": 10.0, "k_n_per_m": 200.0}  # the soft end of the arm range (physics perspective §1.11)
    for i, g in enumerate(gestures):
        guide = human.hand_profile("pull", -g, 0.8, 0.15, hold_s=1.0, impedance=arm)
        jobs.append(
            _Job(
                f"detent_{g:g}N",
                "drawer",
                p,
                _spec(p, "drawer", f"pantry_detent_{g:g}", 3.3, initial={"lev": "levitated", "x_m": detent_x + 0.007}, events=[guide], noise_seed=seeds[1 + i], rate_hz=2000.0, windows={"guide": [0.8, 3.3]}),
                overrides=ov,
            )
        )
    done = _run_jobs(jobs, workers)
    flow = done[0].result

    # --- termini ---
    traces = _terminus_traces(flow)
    close_tr = [tr for tr in traces if tr["direction"] == "close"]
    v_close = float(p.get("safety.v_close_max_mps"))
    j_t = float(p.get("control.motion.terminus.j_mps3"))
    tau = float(p.get("control.motion.terminus.tau_s"))
    cf = nat.terminus_blend(v_close, j_t, tau)
    t_cf = nat.terminus_t_arrive(v_close, j_t, tau)
    cap = _first(flow, "TerminusCapture", after=t_close)
    seated = _first(flow, "Seated", after=t_close)
    latched = _first(flow, "plant:latched", after=t_close)
    term = {
        "r2": _rnd(_kpi(flow, "terminus_r2_min", "close")),
        "tau_fit_s": _rnd(_kpi(flow, "terminus_tau_fit_s", "close")),
        "v_arrive_mps": _rnd(_kpi(flow, "terminus_v_arrive_max_mps", "close")),
        "overshoot_m": _rnd(_kpi(flow, "terminus_overshoot_max_m", "close")),
        "t_arrive_s": _rnd(_kpi(flow, "terminus_t_arrive_s", "close")),
        "t_arrive_err": _rnd(_kpi(flow, "terminus_t_arrive_err_max", "close")),
        "d_b_capture_m": _rnd(cap.a) if cap is not None else None,
        "closed_form": {"v_mps": v_close, "j_mps3": j_t, "tau_s": tau, "d_0_m": _rnd(cf["d_0_m"]), "d_b_m": _rnd(cf["d_b_m"]), "t_b_s": _rnd(cf["t_b_s"]), "t_arrive_s": _rnd(t_cf)},
        "plan_table": {"d_0_m": 0.0400, "d_b_m": 0.0235, "t_arrive_s": 1.27},
        "recorded_fits": [{"direction": tr["direction"], "r2": _rnd(tr["r2"]), "tau_fit_s": _rnd(tr["tau_fit_s"])} for tr in traces],
    }
    tt, vv = flow.t_s, np.asarray(flow["v_mps"])
    traj = (tt >= t_close) & (np.asarray(flow["motion_mode"]) == 2)  # MotionMode::Trajectory (the governed cruise)
    v_close_meas = float(np.max(np.abs(vv[traj]))) if traj.any() else math.nan
    v_hand_max = float(np.max(np.abs(vv[(tt >= t_close) & (tt < t_close + 0.4)])))

    # --- detents ---
    det = []
    for d in done[1:]:
        r = d.result
        x = np.asarray(r["x_m"])
        f, a, v = (np.asarray(r[c]) for c in ("f_hand_n", "a_mps2", "v_mps"))
        m_hat = float(np.median(np.asarray(r["m_hat_kg"])))
        res = f - (m_hat / ratio) * a - b_v * v
        inside = np.abs(x - detent_x) < detent_w
        peak = float(np.max(np.abs(res[inside]))) if inside.any() else math.nan
        det.append(
            {
                "gesture_n": float(d.job.label.split("_")[1][:-1]),
                "detent_peak_err_kpi": _rnd(_kpi(r, "detent_peak_err")),
                "residual_peak_n": _rnd(peak),
                "samples_inside": int(inside.sum()),
                "x_min_m": _rnd(float(x.min())),
                "intent_events": len(_events(r, "Intent")),
                "monitor_trips": _monitor_trips(r),
            }
        )

    m_gov = max(float(p.get("design.carriage.m_rated_kg")), 1.05 * float(np.median(np.asarray(flow["m_hat_kg"])[(tt > 1.0) & (tt < 3.5)])))
    latch = {
        "retention_n": _LATCH_RETENTION_N,
        "a_design_mps2": _LATCH_A_DESIGN,
        "m_gov_kg": _rnd(m_gov),
        "demand_n": _rnd(m_gov * _LATCH_A_DESIGN),
        "pfa_x_capacity_mps2": _rnd(_LATCH_RETENTION_N / m_gov),
        "latched_event_s": _rnd(_t(latched)),
    }
    checks = [
        _check("close terminus semilog R² (base-rate truth)", term["r2"], 0.999, ">=", "", "FAB-HAP-013 R² ≥ 0.999", "FAB-HAP-013"),
        _check("close terminus arrival speed", term["v_arrive_mps"], 1e-3, "<=", "m/s", "FAB-HAP-016 ≤ 1 mm/s", "FAB-HAP-016"),
        _check("close terminus overshoot", term["overshoot_m"], 2e-5, "<=", "m", "FAB-HAP-016 ≤ 0.02 mm", "FAB-HAP-016"),
        _check("close terminus t_arrive error vs closed form", term["t_arrive_err"], 0.02, "<=", "", "FAB-HAP-016 ±2 %", "FAB-HAP-016"),
    ]
    for q in det:
        checks.append(_check(f"detent peak error, {q['gesture_n']:g} N guide through a 200 N/m arm", q["detent_peak_err_kpi"], 0.10, "<=", "", "FAB-HAP-008 ±10 %", "FAB-HAP-008"))
    checks += [
        _check("slider back in its seat after the tap", seated is not None, True, "true", "", "plan §4b pantry flow"),
        _check("latch retention ≥ m_gov·a_design (1.0 g)", _LATCH_RETENTION_N - m_gov * _LATCH_A_DESIGN, 0.0, ">=", "N", "plan §4b: x_retention_n ≥ m_rated·a_design"),
        _check("monitor trips (all runs)", sum(len(_monitor_trips(d.result)) for d in done), 0, "==", "count", "a monitor trip in an example is a failure (FAB-EXA-007 rule)"),
    ]
    results = {
        "device": "8 kg cabinet slider derived from DRAWER",
        "egress_role": "None",
        "overrides": ov,
        "terminus": term,
        "v_close_measured_mps": _rnd(v_close_meas),
        "v_during_tap_max_mps": _rnd(v_hand_max),
        "open_arrived_s": _rnd(_t(_first(flow, "Arrived", after=t_open, before=t_close))),
        "close_intent_s": _rnd(_t(_first(flow, "Intent", after=t_close))),
        "seated_s": _rnd(_t(seated)),
        "detents": det,
        "detent_profile": {"x_m": detent_x, "f_n": detent_f, "width_m": detent_w},
        "latch": latch,
    }
    dq = det[-1]
    story = (
        f"The pantry's 8 kg sliding cabinet door is a DRAWER-derived device (egress role None, seismic latch fitted). "
        f"On an open command it unseats and glides 0.30 m open. A {dq['gesture_n']:g} N guiding hand drags it across its "
        f"{detent_f:g} N detent at {detent_x * 1e3:.0f} mm: the rendered detent peaks at {_fmt(dq['residual_peak_n'], '.2f')} N "
        f"(error {_fmt((dq['detent_peak_err_kpi'] or math.nan) * 100, '.1f')} % against ±10 %). A firm {_FIRM_PUSH_N:g} N tap starts the close, which cruises at "
        f"{_fmt(v_close_meas, '.3f')} m/s (low-energy close cap {v_close:g} m/s; the tap itself flicks it to {_fmt(v_hand_max, '.3f')} m/s) and the log-damped terminus captures it at d_b = "
        f"{_fmt((term['d_b_capture_m'] or math.nan) * 1e3, '.1f')} mm (closed form {cf['d_b_m'] * 1e3:.1f} mm, d_0 {cf['d_0_m'] * 1e3:.1f} mm): semilog R² "
        f"{_fmt(term['r2'], '.5f')}, τ fit {_fmt(term['tau_fit_s'], '.3f')} s, arrival {_fmt((term['v_arrive_mps'] or math.nan) * 1e3, '.2f')} mm/s, "
        f"t_arrive {_fmt(term['t_arrive_s'], '.3f')} s against the closed form {t_cf:.3f} s. It re-seats"
        f"{' at ' + _fmt(_t(seated), '.2f') + ' s' if seated is not None else ' (not in this run)'}; the latch's {_LATCH_RETENTION_N:g} N retention "
        f"holds a floor acceleration up to {_fmt(latch['pfa_x_capacity_mps2'], '.1f')} m/s² at m_gov {_fmt(m_gov, '.1f')} kg. {_failed_sentence(checks)}"
    )

    def figure(fig: Any) -> None:
        from .. import plotstyle

        C = plotstyle.COLORS
        ax = fig.subplots(2, 2)
        a0, a1, a2, a3 = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]
        t = flow.t_s
        a0.plot(t, np.asarray(flow["x_m"]) * 1e3, color=CLASS_DRAWER, label="slider x (plant truth)")
        if "x_ref_m" in flow:
            a0.plot(t, np.asarray(flow["x_ref_m"]) * 1e3, color=C["ctrl"], lw=1.0, ls=(0, (4, 3)), label="reference")
        _vmark(a0, t_open, "open command")
        _vmark(a0, t_close, f"tap {_FIRM_PUSH_N:g} N")
        _vmark(a0, _t(seated), "seated")
        for tr in traces:
            a0.axvspan(tr["t_capture_s"], tr["t_arrived_s"] if math.isfinite(tr["t_arrived_s"]) else tr["t_capture_s"], color=C["band"], alpha=0.10)
        _style(a0, "time (s)", "travel x (mm)", "Open, then a tap closes it into the seat")
        _legend(a0, loc="center right")
        _semilog_panel(a1, close_tr or traces, [C["s1"]], ["close terminus"], tau)
        a1.annotate(
            f"closed form (v {v_close:g} m/s, j {j_t:g} m/s³, τ {tau:g} s):\nd_0 {cf['d_0_m'] * 1e3:.1f} mm, d_b {cf['d_b_m'] * 1e3:.1f} mm, t_arrive {t_cf:.2f} s",
            xy=(0.98, 0.97), xycoords="axes fraction", ha="right", va="top", fontsize=8.5, color=C["ink2"],
        )
        for k, d in enumerate(done[1:]):
            r = d.result
            x = np.asarray(r["x_m"])
            f, a, v = (np.asarray(r[c]) for c in ("f_hand_n", "a_mps2", "v_mps"))
            m_hat = float(np.median(np.asarray(r["m_hat_kg"])))
            res = f - (m_hat / ratio) * a - b_v * v
            near = np.abs(x - detent_x) < 3.0 * detent_w
            a2.plot((x[near] - detent_x) * 1e3, res[near], ".", ms=2.5, color=SERIES_ORDER[k % len(SERIES_ORDER)], label=f"{det[k]['gesture_n']:g} N guide")
        for sgn in (1.0, -1.0):
            a2.axhspan(sgn * 0.9 * detent_f, sgn * 1.1 * detent_f, color=C["band"], alpha=0.12)
        plotstyle.ref_line(a2, detent_f, f"± {detent_f:g} N profile peak ± 10 %", color=C["star"])
        plotstyle.ref_line(a2, -detent_f, None, color=C["star"])
        a2.axvspan(-detent_w * 1e3, detent_w * 1e3, color=C["grid"], alpha=0.5)
        _style(a2, "position relative to the detent (mm)", "F_hand − M_v·a − B_v·v (N)", "Rendered detent felt by a guiding hand")
        _legend(a2, loc="lower left")
        pfa = np.linspace(0.0, 1.6 * _G0, 50)
        a3.plot(pfa / _G0, m_gov * pfa, color=C["s1"], label=f"demand m_gov·PFA_x (m_gov {m_gov:.1f} kg)")
        plotstyle.ref_line(a3, _LATCH_RETENTION_N, f"latch retention {_LATCH_RETENTION_N:g} N", color=C["star"])
        a3.axvline(_LATCH_A_DESIGN / _G0, color=C["muted"], ls=(0, (3, 3)), lw=1.0)
        a3.annotate("a_design 1.0 g (assumed)", xy=(_LATCH_A_DESIGN / _G0, 0.05), xycoords=("data", "axes fraction"), rotation=90, ha="right", va="bottom", fontsize=8, color=C["ink2"])
        _style(a3, "floor acceleration along x (g)", "force on the seated slider (N)", "Seismic latch: retention against demand")
        _legend(a3, loc="upper left")

    return _finish(_PANTRY, out_dir, quick=quick, seed=seed, runs=done, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t_start)


# =====================================================================================
# 3. office_door_obstruction (DOOR)
# =====================================================================================

_OFFICE = _Meta(
    number=3,
    name="office_door_obstruction",
    title="a low-energy office door closing onto a chest",
    badge="🔶",
    maturity="automatic sliding doors are TRL 9 with conventional drives; the maglev door is a concept",
    story=(
        "A 100 kg office DOOR closes in low-energy mode onto a clamped chest (ISO/TS 15066 body model behind "
        "a 10 N/mm edge seal). The obstruction observer stops it and it reverses 50 mm; the contact-force "
        "profile is checked in simulation against numeric limits from EN 16005 (2011 draft)."
    ),
    requirements=(
        ("FAB-SAF-004", "contact force–time profile against the EN 16005 (2011 draft) numeric limits"),
        ("FAB-SAF-005", "clamp force against the energy form F = √(2·E·k)"),
        ("FAB-SAF-006", "stop and reverse 50 mm, reversal ≤ 20 ms after detection"),
    ),
    caveats=(
        "Checked in simulation against numeric limits from EN 16005 (2011 draft); no compliance or certification is claimed.",
        "The chest is the ISO/TS 15066 body model (25 N/mm, 40 kg) behind the door's 10 N/mm edge seal; real bodies vary.",
        "FAB-SAF-004 is status S (simplified), so this example's badge is 🔶.",
    ),
)


def _office(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    from .. import _native as nat
    from .. import human

    seed, workers = _options(options)
    t_start = time.perf_counter()
    p = _params("door", seed)
    x0 = float(p.get("design.rail.x_closed_m"))
    m = float(p.get("design.carriage.m_star_kg"))
    k_edge = float(p.get("design.damping.k_edge_n_per_m"))
    lim = {k: float(p.get(f"safety.{k}")) for k in ("f_drive_max_n", "t_drive_max_s", "f_static_n", "f_static_final_n", "t_static_final_s", "v_close_max_mps", "e_max_j")}
    reverse_m = float(p.get("control.haptic.obstruction.reverse_m"))
    pos_tol = float(p.get("control.motion.pos_tol_travel_m"))
    cases = [("chest_clamped", "chest", 0.30, "clamped")]
    if not quick:
        cases += [("chest_free", "chest", 0.30, "free"), ("hand_edge_clamped", "hand", 0.02, "clamped")]
    body_k = {"chest": 25e3, "hand": 75e3}  # ISO/TS 15066 contact stiffness (scenario-1 schema obstacle.body)
    seeds = _noise_seeds(seed, len(cases))
    jobs = []
    for i, (label, body, x_obs, config) in enumerate(cases):
        ev = [human.obstacle(0.0, body, x0 + x_obs, config=config), {"kind": "command", "t_s": 0.5, "cmd": "close"}]
        jobs.append(
            _Job(label, "door", p, _spec(p, "door", f"office_{label}", 9.5, initial={"lev": "levitated", "x_m": x0 + 0.6}, events=ev, noise_seed=seeds[i], windows={"contact": [0.5, 9.5]}))
        )
    done = _run_jobs(jobs, workers)

    per_case = []
    for (label, body, x_obs, config), d in zip(cases, done, strict=True):
        r = d.result
        t = r.t_s
        f = np.asarray(r["f_obstacle_n"])
        contact = _first(r, "plant:obstacle_contact")
        t_c = _t(contact)
        v_c = float(contact.a) if contact is not None else math.nan
        det = _first(r, "Obstruction", after=t_c - 1e-9) if contact is not None else None
        rev = _first(r, "Reversal", after=_t(det) - 1e-9) if det is not None else None
        x = np.asarray(r["x_m"])
        after = t >= _t(det) if det is not None else np.zeros_like(t, dtype=bool)
        # Reverse travel: from the stop point (the closest approach after detection) to the
        # furthest reopened position.
        if after.any():
            xa = x[after]
            k_max = int(np.argmax(xa))
            rev_dist = float(xa[k_max] - np.min(xa[: k_max + 1]))
        else:
            rev_dist = math.nan
        e_c = 0.5 * m * v_c**2
        k_s = nat.series_stiffness(body_k[body], k_edge)
        f_cf = nat.clamp_force(e_c, k_s) if math.isfinite(e_c) else math.nan
        f_cap = nat.clamp_force(lim["e_max_j"], k_s)
        above = f > lim["f_static_n"]
        t_above = float(np.sum(above)) / r.rate_hz
        late = t >= t_c + lim["t_drive_max_s"]
        f_late = float(np.max(f[late])) if late.any() else math.nan
        i5 = int(np.searchsorted(t, t_c + lim["t_static_final_s"]))
        f_5 = float(f[min(i5, len(f) - 1)]) if math.isfinite(t_c) else math.nan
        per_case.append(
            {
                "case": label,
                "body": body,
                "config": config,
                "x_obstacle_m": x0 + x_obs,
                "t_contact_s": _rnd(t_c),
                "v_contact_mps": _rnd(v_c),
                "ke_contact_j": _rnd(e_c),
                "detect_after_contact_s": _rnd(_t(det) - t_c),
                "reverse_after_detect_s": _rnd(_t(rev) - _t(det)),
                "reverse_latency_kpi_s": _rnd(_kpi(r, "reverse_latency_s")),
                "reverse_distance_m": _rnd(rev_dist),
                "f_peak_n": _rnd(float(np.max(f))),
                "k_series_n_per_m": _rnd(k_s),
                "f_closed_form_at_contact_ke_n": _rnd(f_cf),
                "f_closed_form_at_e_cap_n": _rnd(f_cap),
                "t_above_f_static_s": _rnd(t_above),
                "f_max_after_t_drive_n": _rnd(f_late),
                "f_at_t_static_final_n": _rnd(f_5),
                "monitor_trips": _monitor_trips(r),
            }
        )
    c0 = per_case[0]
    src = "EN 16005 (2011 draft) numeric limits as carried by the DOOR safety block"
    checks = []
    for q in per_case:
        tag = f" ({q['case'].replace('_', ' ')})"
        checks += [
            _check("peak contact force" + tag, q["f_peak_n"], lim["f_drive_max_n"], "<=", "N", f"{src}: f_drive_max", "FAB-SAF-004"),
            _check("time above the 67 N static limit" + tag, q["t_above_f_static_s"], lim["t_drive_max_s"], "<=", "s", f"{src}: t_drive_max", "FAB-SAF-004"),
            _check("max force after t_drive_max" + tag, q["f_max_after_t_drive_n"], lim["f_static_n"], "<=", "N", f"{src}: f_static (low-energy)", "FAB-SAF-004"),
            _check("force 5 s after contact" + tag, q["f_at_t_static_final_n"], lim["f_static_final_n"], "<", "N", f"{src}: < 80 N within 5 s", "FAB-SAF-004"),
            _check("peak force ≤ closed-form clamp force √(2·E·k) at the contact energy" + tag, _f(q["f_peak_n"]) - _f(q["f_closed_form_at_contact_ke_n"]), 0.0, "<=", "N", "FAB-SAF-005 energy form (bound)", "FAB-SAF-005"),
            _check("reversal after detection" + tag, q["reverse_after_detect_s"], 0.020, "<=", "s", "FAB-SAF-006 DOOR ≤ 20 ms", "FAB-SAF-006"),
            _check("reverse travel from the stop point" + tag, _f(q["reverse_distance_m"]) - reverse_m, pos_tol, "abs<=", "m", f"FAB-SAF-006 reverse {reverse_m * 1e3:g} mm, within the class travel tolerance {pos_tol * 1e3:g} mm", "FAB-SAF-006"),
        ]
    checks.append(_check("monitor trips (all runs)", sum(len(q["monitor_trips"]) for q in per_case), 0, "==", "count", "a monitor trip in an example is a failure (FAB-EXA-007 rule)"))
    results = {"rail_class": "door", "m_kg": m, "k_edge_n_per_m": k_edge, "limits": lim, "reverse_m": reverse_m, "cases": per_case}
    story = (
        f"A {m:g} kg office DOOR closes in low-energy mode at ≤ {lim['v_close_max_mps']:g} m/s onto a clamped chest "
        f"(ISO/TS 15066 body, 25 N/mm, behind the {k_edge / 1e3:g} N/mm edge seal) {c0['x_obstacle_m'] * 1e3:.0f} mm from the jamb. It touches at "
        f"{_fmt(c0['v_contact_mps'], '.3f')} m/s ({_fmt(c0['ke_contact_j'], '.2f')} J), the obstruction observer detects the contact "
        f"{_fmt((c0['detect_after_contact_s'] or math.nan) * 1e3, '.0f')} ms later and the door reverses "
        f"{_fmt((c0['reverse_after_detect_s'] or math.nan) * 1e3, '.0f')} ms after detection by {_fmt((c0['reverse_distance_m'] or math.nan) * 1e3, '.1f')} mm. "
        f"The contact force peaks at {_fmt(c0['f_peak_n'], '.1f')} N, below the energy-form bound √(2·E·k) = {_fmt(c0['f_closed_form_at_contact_ke_n'], '.1f')} N at that energy "
        f"({_fmt(c0['f_closed_form_at_e_cap_n'], '.0f')} N at the {lim['e_max_j']:g} J cap), and is {_fmt(c0['f_at_t_static_final_n'], '.1f')} N five seconds after contact. "
        f"Checked in simulation against numeric limits from EN 16005 (2011 draft): ≤ {lim['f_drive_max_n']:g} N, above {lim['f_static_n']:g} N for at most "
        f"{lim['t_drive_max_s']:g} s, < {lim['f_static_final_n']:g} N within {lim['t_static_final_s']:g} s. {_failed_sentence(checks)}"
    )

    def figure(fig: Any) -> None:
        from .. import plotstyle

        C = plotstyle.COLORS
        ax = fig.subplots(2, 2)
        a0, a1, a2, a3 = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]
        r0 = done[0].result
        a0.plot(r0.t_s, np.asarray(r0["x_m"]) * 1e3, color=C["s2"], label="door x (plant truth)")
        if "x_ref_m" in r0:
            a0.plot(r0.t_s, np.asarray(r0["x_ref_m"]) * 1e3, color=C["ctrl"], lw=1.0, ls=(0, (4, 3)), label="reference")
        plotstyle.ref_line(a0, c0["x_obstacle_m"] * 1e3, "chest surface", color=C["star"])
        _vmark(a0, c0["t_contact_s"], "contact")
        _vmark(a0, _f(c0["t_contact_s"]) + _f(c0["detect_after_contact_s"]), "detected", y=0.55)
        _style(a0, "time (s)", "travel x (mm)", "Close, contact, stop and reverse 50 mm")
        _legend(a0, loc="upper right")
        for k, (q, d) in enumerate(zip(per_case, done, strict=True)):
            r = d.result
            tc = _f(q["t_contact_s"])
            if not math.isfinite(tc):
                continue
            sel = (r.t_s >= tc - 0.1) & (r.t_s <= tc + 6.0)
            a1.plot(r.t_s[sel] - tc, np.asarray(r["f_obstacle_n"])[sel], color=SERIES_ORDER[(k + 1) % 5], label=q["case"].replace("_", " "))
        plotstyle.ref_line(a1, lim["f_drive_max_n"], f"{lim['f_drive_max_n']:g} N", color=C["star"])
        plotstyle.ref_line(a1, lim["f_static_final_n"], f"{lim['f_static_final_n']:g} N by {lim['t_static_final_s']:g} s", color=C["muted"])
        plotstyle.ref_line(a1, lim["f_static_n"], f"{lim['f_static_n']:g} N after {lim['t_drive_max_s']:g} s", color=C["muted"], va="top")
        a1.axvline(lim["t_drive_max_s"], color=C["muted"], lw=1.0, ls=(0, (2, 3)))
        a1.axvline(lim["t_static_final_s"], color=C["muted"], lw=1.0, ls=(0, (2, 3)))
        _style(a1, "time since contact (s)", "contact force (N)", "Force–time profile, EN 16005 (2011 draft) limits")
        a1.set_ylim(-5.0, 1.25 * lim["f_drive_max_n"])
        _legend(a1, loc="upper center", ncol=len(per_case))
        a2.plot(r0.t_s, np.asarray(r0["v_mps"]), color=C["s2"], label="door v (plant truth)")
        plotstyle.ref_line(a2, -lim["v_close_max_mps"], f"close cap {lim['v_close_max_mps']:g} m/s", color=C["star"], va="top")
        plotstyle.zero_line(a2)
        _vmark(a2, c0["t_contact_s"], "contact")
        _style(a2, "time (s)", "velocity (m/s)", "Low-energy close speed and the reversal")
        _legend(a2, loc="upper right")
        labels = [q["case"].replace("_", "\n") for q in per_case]
        idx = np.arange(len(per_case))
        w = 0.26
        a3.bar(idx - w, [_f(q["f_peak_n"]) for q in per_case], w, color=C["s2"], label="simulated peak")
        a3.bar(idx, [_f(q["f_closed_form_at_contact_ke_n"]) for q in per_case], w, color=C["s1"], label=r"$\sqrt{2Ek}$ at the contact energy")
        a3.bar(idx + w, [_f(q["f_closed_form_at_e_cap_n"]) for q in per_case], w, color=C["ctrl"], label=rf"$\sqrt{{2Ek}}$ at the {lim['e_max_j']:g} J cap")
        a3.set_xticks(idx, labels)
        _style(a3, None, "force (N)", "Peak force against the energy form (FAB-SAF-005)")
        a3.set_ylim(0.0, 1.3 * max(_f(q["f_closed_form_at_e_cap_n"]) for q in per_case))
        _legend(a3, loc="upper left")

    return _finish(_OFFICE, out_dir, quick=quick, seed=seed, runs=done, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t_start)


# =====================================================================================
# 4. ballroom_partition (WALL)
# =====================================================================================

_BALLROOM = _Meta(
    number=4,
    name="ballroom_partition",
    title="a 500 kg ballroom partition that moves like 100 kg and seals to ±1 mm",
    badge="✅",
    maturity="operable partitions are TRL 9 (mechanical); the maglev partition is a concept",
    story=(
        "A 500 kg WALL partition is rendered to a guiding hand at M_v = m̂/5; unsensed it moves at the "
        "pedestrian energy and momentum caps, sensed (PL d presence sensing) its class caps rise; it closes "
        "through a log-damped terminus and lands sealed to ±1 mm. Rendered lightness does not reduce impact energy."
    ),
    requirements=(
        ("FAB-HAP-001", "apparent mass within ±10 % of M_v = m̂/5"),
        ("FAB-HAP-013", "semilog R² ≥ 0.999 of the closing terminus"),
        ("FAB-HAP-016", "arrival ≤ 1 mm/s, overshoot ≤ 0.02 mm, t_arrive within ±2 %"),
        ("FAB-SAF-002", "unsensed speed cap 0.0446 m/s"),
        ("FAB-SAF-003", "speeds above the unsensed cap only with PL d presence sensing"),
    ),
    caveats=(
        "Rendered lightness (M_v = m/5) does not reduce impact energy: a moving partition carries ½·m·v² of its "
        "500 kg (core non-claim 6).",
        "The seal is a positional result of the simulated terminus and landing; seal compression and acoustics are not modelled.",
        "Presence sensing is modelled as a healthy PL d input with the zone clear; the sensor itself is not simulated.",
        "Simulated; not measured on hardware.",
    ),
)


def _move_time(d: float, v: float, a: float) -> float:
    """Trapezoidal move time d/v + v/a (jerk ignored; a planning estimate, not a simulation)."""
    if v <= 0.0 or a <= 0.0:
        return math.nan
    return d / v + v / a if d >= v * v / a else 2.0 * math.sqrt(d / a)


def _ballroom(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    from .. import human

    seed, workers = _options(options)
    t_start = time.perf_counter()
    p = _params("wall", seed)
    ps = _params("wall", seed, {"design.presence": "pl_d"})
    x0 = float(p.get("design.rail.x_closed_m"))
    travel = float(p.get("design.rail.travel_m"))
    m = float(p.get("design.carriage.m_star_kg"))
    m_rated = float(p.get("design.carriage.m_rated_kg"))
    ratio = float(p.get("control.haptic.mass_ratio"))
    f_th = float(p.get("control.intent.f_th_n"))
    a_max = float(p.get("control.motion.a_max_mps2"))
    a_max_s = float(p.get("control.motion.a_max_sensed_mps2"))
    caps = {k: float(p.get(f"safety.{k}")) for k in ("v_open_max_mps", "v_open_sensed_mps", "e_max_j", "p_max_kgmps")}
    t_arr_cf = float(p.derived()["terminus_t_arrive_s"])
    d_move = 0.5 if quick else 1.5
    seeds = _noise_seeds(seed, 5)

    f_guide = 1.2 * f_th
    t_guide, t_close, x_start = 1.0, 3.0, x0 + 0.5
    dur_close = t_close + (x_start - x0) / 0.04 + 0.5 + t_arr_cf + 2.5
    jobs = [
        _Job(
            "guide_and_close",
            "wall",
            p,
            _spec(
                p,
                "wall",
                "ballroom_close",
                dur_close,
                initial={"lev": "levitated", "x_m": x_start},
                events=[human.hand_profile("pull", -f_guide, t_guide, 0.15, hold_s=0.4), {"kind": "command", "t_s": t_close, "cmd": "close"}],
                noise_seed=seeds[0],
                windows={"guide": [t_guide, t_close], "close": [t_close, dur_close]},
            ),
        )
    ]
    dur_move = 0.5 + d_move / 0.04 + 1.0 + t_arr_cf + 1.5
    for label, pp, extra, sd in (("move_unsensed", p, [], seeds[1]), ("move_sensed", ps, [{"kind": "presence", "t_s": 0.1, "active": False, "ok": True}], seeds[2])):
        ev = [*extra, {"kind": "command", "t_s": 0.5, "cmd": "go_to", "x_m": x0 + d_move}]
        jobs.append(
            _Job(label, "wall", pp, _spec(pp, "wall", f"ballroom_{label}", dur_move, initial={"lev": "levitated", "x_m": x0}, events=ev, noise_seed=sd, windows={"move": [0.5, dur_move]}), overrides={"design.presence": "pl_d"} if pp is ps else {})
        )
    if not quick:
        dur_trav = 0.5 + travel / 0.04 + 2.0 + t_arr_cf + 2.0
        for label, pp, extra, sd in (("traverse_unsensed", p, [], seeds[3]), ("traverse_sensed", ps, [{"kind": "presence", "t_s": 0.1, "active": False, "ok": True}], seeds[4])):
            ev = [*extra, {"kind": "command", "t_s": 0.5, "cmd": "open"}]
            jobs.append(
                _Job(label, "wall", pp, _spec(pp, "wall", f"ballroom_{label}", dur_trav, initial={"lev": "levitated", "x_m": x0}, events=ev, noise_seed=sd, fidelity="travel1d", rate_hz=100.0, windows={"move": [0.5, dur_trav]}), overrides={"design.presence": "pl_d"} if pp is ps else {})
            )
    done = _run_jobs(jobs, workers)
    rc = done[0].result

    am = _apparent_mass(rc, t_guide, t_close, ratio)
    am_err = am["m_app_kg"] / am["m_v_kg"] - 1.0 if math.isfinite(am["m_v_kg"]) else math.nan
    traces = _terminus_traces(rc)
    arrived = _first(rc, "Arrived", after=t_close)
    landed = _first(rc, "Landed", after=t_close)
    x_final = float(np.asarray(rc["x_m"])[-1])
    close = {
        "r2": _rnd(_kpi(rc, "terminus_r2_min", "close")),
        "tau_fit_s": _rnd(_kpi(rc, "terminus_tau_fit_s", "close")),
        "v_arrive_mps": _rnd(_kpi(rc, "terminus_v_arrive_max_mps", "close")),
        "overshoot_m": _rnd(_kpi(rc, "terminus_overshoot_max_m", "close")),
        "t_arrive_err": _rnd(_kpi(rc, "terminus_t_arrive_err_max", "close")),
        "t_arrive_cf_s": _rnd(t_arr_cf),
        "arrived_s": _rnd(_t(arrived)),
        "landed_s": _rnd(_t(landed)),
        "x_final_m": _rnd(x_final),
        "seal_error_m": _rnd(x_final - x0),
    }

    def _move(d: _Done, dist: float) -> dict[str, Any]:
        r = d.result
        t, x, v = r.t_s, np.asarray(r["x_m"]), np.asarray(r["v_mps"])
        arr = _first(r, "Arrived", after=0.5)
        reach = np.flatnonzero(np.abs(x - (x0 + dist)) <= 1e-3)
        return {
            "label": d.job.label,
            "fidelity": d.job.spec["fidelity"]["model"],
            "distance_m": dist,
            "v_max_mps": _rnd(float(np.max(np.abs(v)))),
            "v_ref_max_mps": _rnd(float(np.max(np.abs(np.asarray(r["v_ref_mps"]))))) if "v_ref_mps" in r else None,
            "t_arrived_s": _rnd(_t(arr) - 0.5),
            "t_within_1mm_s": _rnd(float(t[reach[0]]) - 0.5 if reach.size else math.nan),
            "ke_max_j": _rnd(0.5 * m * float(np.max(np.abs(v))) ** 2),
            "momentum_max_kgmps": _rnd(_kpi(r, "momentum_max")),
            "monitor_trips": _monitor_trips(r),
        }

    moves = [_move(done[1], d_move), _move(done[2], d_move)]
    if not quick:
        moves += [_move(done[3], travel), _move(done[4], travel)]
    m_gov = max(m_rated, 1.05 * m)
    v_gov = min(math.sqrt(2.0 * 0.90 * caps["e_max_j"] / m_gov), 0.95 * caps["p_max_kgmps"] / m_gov)
    plan = {
        "m_gov_kg": m_gov,
        "v_governor_mps": _rnd(v_gov),
        "full_travel_m": travel,
        "t_full_unsensed_cap_s": _rnd(_move_time(travel, caps["v_open_max_mps"], a_max)),
        "t_full_sensed_cap_s": _rnd(_move_time(travel, caps["v_open_sensed_mps"], a_max_s)),
        "t_full_at_governor_s": _rnd(_move_time(travel, min(v_gov, caps["v_open_max_mps"]), a_max)),
        "note": "closed-form trapezoid estimates (d/v + v/a), not simulations",
    }
    t_u, t_s = _f(moves[0]["t_arrived_s"]), _f(moves[1]["t_arrived_s"])
    checks = [
        _check(f"apparent mass error, {f_guide:g} N guide (M_app/M_v − 1)", am_err, 0.10, "abs<=", "", "FAB-HAP-001 ±10 %", "FAB-HAP-001"),
        _check("closing terminus semilog R²", close["r2"], 0.999, ">=", "", "FAB-HAP-013 R² ≥ 0.999", "FAB-HAP-013"),
        _check("closing terminus arrival speed", close["v_arrive_mps"], 1e-3, "<=", "m/s", "FAB-HAP-016 ≤ 1 mm/s", "FAB-HAP-016"),
        _check("closing terminus overshoot", close["overshoot_m"], 2e-5, "<=", "m", "FAB-HAP-016 ≤ 0.02 mm", "FAB-HAP-016"),
        _check("closing terminus t_arrive error", close["t_arrive_err"], 0.02, "<=", "", "FAB-HAP-016 ±2 %", "FAB-HAP-016"),
        _check("seal: final position error after landing", close["seal_error_m"], 1e-3, "abs<=", "m", "plan §4b ballroom: seal to ±1 mm"),
        _check("unsensed commanded (reference) peak speed", moves[0]["v_ref_max_mps"], caps["v_open_max_mps"], "<=", "m/s", "FAB-SAF-002 WALL 0.0446 m/s", "FAB-SAF-002"),
        _check("unsensed peak speed (plant truth)", moves[0]["v_max_mps"], caps["v_open_max_mps"], "<=", "m/s", "FAB-SAF-002 WALL 0.0446 m/s (truth; the monitor trips at 1.1·v_lim)", "FAB-SAF-002"),
        _check(
            "sensed move faster than unsensed (sensed caps usable with PL d)",
            t_u - t_s,
            0.0,
            ">",
            "s",
            "FAB-SAF-003; monitor energy.rs: with PL d sensing the sensed caps replace the energy basis",
            "FAB-SAF-003",
        ),
        _check("monitor trips (all runs)", sum(len(_monitor_trips(d.result)) for d in done), 0, "==", "count", "a monitor trip in an example is a failure (FAB-EXA-007 rule)"),
    ]
    results = {
        "rail_class": "wall",
        "m_kg": m,
        "mass_ratio": ratio,
        "guide_force_n": f_guide,
        "apparent_mass": {**{k: _rnd(v) if isinstance(v, float) else v for k, v in am.items()}, "error": _rnd(am_err), "error_kpi": _rnd(_kpi(rc, "apparent_mass_err", "guide")), "breakaway_n": _rnd(_kpi(rc, "breakaway_n", "guide"))},
        "close": close,
        "moves": moves,
        "caps": caps,
        "planning": plan,
        "ke_at_unsensed_cap_j": _rnd(0.5 * m * caps["v_open_max_mps"] ** 2),
    }
    same = math.isfinite(t_u) and math.isfinite(t_s) and abs(t_u - t_s) < 0.05 * t_u
    sensed_txt = (
        f"With PL d presence sensing the same move takes {_fmt(t_s, '.1f')} s at {_fmt(moves[1]['v_max_mps'], '.4f')} m/s: no faster in this build, "
        f"because the governor keeps the pedestrian momentum cap ({caps['p_max_kgmps']:g} kg·m/s → {_fmt(v_gov, '.4f')} m/s at m_gov {m_gov:g} kg) "
        f"in sensed operation, so the sensed class cap {caps['v_open_sensed_mps']:g} m/s is never reached"
        if same
        else f"With PL d presence sensing it takes {_fmt(t_s, '.1f')} s at up to {_fmt(moves[1]['v_max_mps'], '.4f')} m/s (sensed cap {caps['v_open_sensed_mps']:g} m/s)"
    )
    story = (
        f"A {m:g} kg ballroom WALL partition floats on its rail. A {f_guide:g} N guiding pull feels an apparent mass of "
        f"{_fmt(am['m_app_kg'], '.1f')} kg against M_v = m̂/{ratio:g} = {_fmt(am['m_v_kg'], '.1f')} kg ({_pct(am_err)}, limit ±10 %; breakaway "
        f"{_fmt(results['apparent_mass']['breakaway_n'], '.1f')} N). Unsensed, a {d_move:g} m move takes {_fmt(t_u, '.1f')} s at up to "
        f"{_fmt(moves[0]['v_max_mps'], '.4f')} m/s (cap {caps['v_open_max_mps']:g} m/s). {sensed_txt}. For the full {travel:g} m the closed-form "
        f"estimates are {_fmt(plan['t_full_unsensed_cap_s'], '.0f')} s at the unsensed cap and {_fmt(plan['t_full_sensed_cap_s'], '.0f')} s at the sensed cap"
        + (f"; simulated on Travel1d the full traverse takes {_fmt(moves[2]['t_arrived_s'], '.1f')} s unsensed and {_fmt(moves[3]['t_arrived_s'], '.1f')} s sensed. " if len(moves) > 3 else ". ")
        + (
            f"The unsensed run's truth momentum peaks at {_fmt(moves[0]['momentum_max_kgmps'], '.2f')} kg·m/s, above the {caps['p_max_kgmps']:g} kg·m/s cap for a moment at the end of the ramp (no monitor trip). "
            if _f(moves[0]["momentum_max_kgmps"]) > caps["p_max_kgmps"]
            else ""
        )
        + f"Closing, the log-damped terminus (R² {_fmt(close['r2'], '.5f')}, τ fit {_fmt(close['tau_fit_s'], '.3f')} s) arrives at "
        f"{_fmt((close['v_arrive_mps'] or math.nan) * 1e3, '.2f')} mm/s and the partition lands {_fmt((close['seal_error_m'] or math.nan) * 1e3, '.3f')} mm from "
        f"its closed position (seal band ±1 mm). Rendered lightness does not reduce impact energy: at the unsensed cap it still carries "
        f"{_fmt(results['ke_at_unsensed_cap_j'], '.2f')} J. {_failed_sentence(checks)}"
    )

    def figure(fig: Any) -> None:
        from .. import plotstyle

        C = plotstyle.COLORS
        ax = fig.subplots(2, 2)
        a0, a1, a2, a3 = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]
        t = rc.t_s
        fh = np.asarray(rc["f_hand_n"])
        on = np.flatnonzero((t >= t_guide) & (t < t_close) & (np.abs(fh) > 1e-9))
        sel = (t >= (t[on[0]] - 0.05 if on.size else t_guide)) & (t <= (t[on[-1]] + 0.05 if on.size else t_close))
        f, a, v = (np.asarray(rc[c])[sel] for c in ("f_hand_n", "a_mps2", "v_mps"))
        a0.plot(t[sel], f, color=C["s3"], lw=2.0, label="hand force F (truth)")
        if math.isfinite(am["m_app_kg"]):
            a0.plot(t[sel], am["m_app_kg"] * a + am["b_app_n_s_per_m"] * v, color=C["ink2"], lw=0.9, label=f"fit M·a + B·v, M = {am['m_app_kg']:.1f} kg")
            a0.plot(t[sel], am["m_v_kg"] * a + am["b_app_n_s_per_m"] * v, color=C["star"], lw=0.9, ls=(0, (2, 2)), label=f"M_v·a + B·v, M_v = {am['m_v_kg']:.1f} kg")
        _style(a0, "time (s)", "force (N)", f"Guiding pull: rendered mass (m̂/{ratio:g})")
        _legend(a0, loc="lower right")
        for k, q in enumerate(moves[:2]):
            r = done[1 + k].result
            a1.plot(r.t_s - 0.5, np.asarray(r["x_m"]) * 1e3, color=(C["s3"], C["s1"])[k], label=f"{q['label'].split('_')[1]}: arrived {_fmt(q['t_arrived_s'], '.1f')} s")
        _style(a1, "time since command (s)", "travel x (mm)", f"A {d_move:g} m move, unsensed and sensed")
        _legend(a1, loc="lower right")
        close_tr = [tr for tr in traces if tr["direction"] == "close"]
        _semilog_panel(a2, close_tr or traces, [C["s3"]], ["closing terminus"], float(p.get("control.motion.terminus.tau_s")))
        a2.annotate(f"landed {_fmt((close['seal_error_m'] or math.nan) * 1e3, '.3f')} mm from closed (band ±1 mm)", xy=(0.98, 0.97), xycoords="axes fraction", ha="right", va="top", fontsize=8.5, color=C["ink2"])
        names = ["unsensed cap\n(closed form)", "sensed cap\n(closed form)", "governor cap\n(closed form)"]
        vals = [plan["t_full_unsensed_cap_s"], plan["t_full_sensed_cap_s"], plan["t_full_at_governor_s"]]
        cols = [C["ctrl"], C["ctrl"], C["ctrl"]]
        if not quick:
            names += ["unsensed\n(Travel1d)", "sensed\n(Travel1d)"]
            vals += [moves[2]["t_arrived_s"], moves[3]["t_arrived_s"]]
            cols += [C["s3"], C["s1"]]
        a3.bar(np.arange(len(vals)), np.nan_to_num([_f(x) for x in vals]), 0.6, color=cols)
        for i_, v_ in enumerate(vals):
            a3.annotate(_fmt(v_, ".0f", "s"), (i_, _f(v_) if math.isfinite(_f(v_)) else 0.0), ha="center", va="bottom", fontsize=8.5, color=C["ink2"], xytext=(0, 2), textcoords="offset points")
        a3.set_xticks(np.arange(len(vals)), names)
        _style(a3, None, f"time for the full {travel:g} m (s)", "Full-travel timing: sensed against unsensed")

    return _finish(_BALLROOM, out_dir, quick=quick, seed=seed, runs=done, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t_start)


# =====================================================================================
# 5. archive_heavy_shelving (HEAVY)
# =====================================================================================

_ARCHIVE = _Meta(
    number=5,
    name="archive_heavy_shelving",
    title="5 t archive shelving: hold-to-run, and a governor that ignores a low mass estimate",
    badge="✅",
    maturity="mobile archive shelving is TRL 9 (mechanical); the maglev heavy carriage is a concept",
    story=(
        "A 5 t HEAVY archive carriage moves only while hold-to-run is held (unsensed), at ≤ 0.025 m/s. "
        "A second run reads the mass estimate low (magnets stronger than the nominal model): the momentum and "
        "energy governor uses m_gov = max(m_rated, 1.05·m̂), so the commanded speed does not change."
    ),
    requirements=(
        ("FAB-SAF-001", "m_gov = max(m_rated, 1.05·m̂): a low m̂ leaves v_lim unchanged"),
        ("FAB-SAF-002", "HEAVY unsensed cap 0.025 m/s hold-to-run"),
        ("FAB-SAF-003", "HEAVY without PL d sensing moves only hold-to-run"),
    ),
    caveats=(
        "HEAVY momentum (≈ 125 kg·m/s at 0.025 m/s) is stated, not capped: hold-to-run with a person at the "
        "control is the protective measure (core non-claim 7), not the energy limit.",
        "The low mass estimate is produced by stronger truth magnets (B_r × 1.1, m̂ ≈ 0.8·m); B_r × √2 (m̂ ≈ 0.5·m) "
        "pulls a landed 5 t carriage onto its upper stop at start-up, so the full 0.5·m case is shown only by the "
        "governor unit tests.",
        "Runs on Carriage6 in this build (HEAVY acceptance otherwise uses Heave1/Travel1d; full runs add a Travel1d run).",
        "Simulated; not measured on hardware.",
    ),
)

_SABOTAGE_BR = 1.1


def _archive(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    seed, workers = _options(options)
    t_start = time.perf_counter()
    p = _params("heavy", seed)
    x0 = float(p.get("design.rail.x_closed_m"))
    m = float(p.get("design.carriage.m_star_kg"))
    m_rated = float(p.get("design.carriage.m_rated_kg"))
    v_cap = float(p.get("safety.v_open_max_mps"))
    e_cap = float(p.get("safety.e_max_j"))
    a_max = float(p.get("control.motion.a_max_mps2"))
    seeds = _noise_seeds(seed, 2)
    tgt = x0 + 0.2
    t_rej, t_on1, t_go1, t_off, t_on2, t_go2 = 2.0, 2.5, 2.7, 5.5, 6.5, 6.7
    events = [
        {"kind": "command", "t_s": 0.5, "cmd": "lift"},
        {"kind": "command", "t_s": t_rej, "cmd": "go_to", "x_m": tgt},
        {"kind": "hold_to_run", "t_s": t_on1, "on": True},
        {"kind": "command", "t_s": t_go1, "cmd": "go_to", "x_m": tgt},
        {"kind": "hold_to_run", "t_s": t_off, "on": False},
        {"kind": "hold_to_run", "t_s": t_on2, "on": True},
        {"kind": "command", "t_s": t_go2, "cmd": "go_to", "x_m": tgt},
    ]
    dur = 19.5
    wins = {"move": [t_go1, dur]}
    jobs = [
        _Job("nominal", "heavy", p, _spec(p, "heavy", "archive_nominal", dur, initial={"lev": "landed", "x_m": x0}, events=events, noise_seed=seeds[0], windows=wins)),
        _Job("m_hat_low", "heavy", p, _spec(p, "heavy", "archive_m_hat_low", dur, initial={"lev": "landed", "x_m": x0}, events=events, noise_seed=seeds[0], windows=wins, plant={"br_scale": _SABOTAGE_BR})),
    ]
    if not quick:
        jobs.append(_Job("nominal_travel1d", "heavy", p, _spec(p, "heavy", "archive_travel1d", dur, initial={"lev": "landed", "x_m": x0}, events=events, noise_seed=seeds[1], fidelity="travel1d", rate_hz=200.0, windows=wins)))
    done = _run_jobs(jobs, workers)

    def _red(d: _Done) -> dict[str, Any]:
        r = d.result
        t = r.t_s
        v = np.asarray(r["v_mps"])
        lev = np.asarray(r["lev_state"])
        lv = np.isin(lev, (5, 6, 7))
        m_hat = float(np.median(np.asarray(r["m_hat_kg"])[lv])) if lv.any() else math.nan
        vr = np.asarray(r["v_ref_mps"]) if "v_ref_mps" in r else np.full_like(v, np.nan)
        rej = _first(r, "Rejected", after=t_rej - 1e-9, before=t_on1)
        after = t >= t_off
        still = np.flatnonzero(after & (np.abs(v) < 1e-3))
        x = np.asarray(r["x_m"])
        seg1 = (t >= t_go1) & (t < t_off)
        lag = (vr - v)[seg1]
        dem = np.asarray(r["demand"]) if "demand" in r else np.zeros_like(t)
        return {
            "label": d.job.label,
            "fidelity": d.job.spec["fidelity"]["model"],
            "v_ref_max_first_segment_mps": _rnd(float(np.nanmax(np.abs(vr[seg1])))),
            "v_lag_max_first_segment_mps": _rnd(float(np.nanmax(lag))) if lag.size else None,
            "v_lag_rms_first_segment_mps": _rnd(float(np.sqrt(np.nanmean(lag**2)))) if lag.size else None,
            "limit_force_time_s": _rnd(float(np.sum(dem == 1)) / r.rate_hz),
            "m_hat_kg": _rnd(m_hat),
            "m_gov_kg": _rnd(max(m_rated, 1.05 * m_hat)) if math.isfinite(m_hat) else None,
            "rejected_without_hold_to_run": rej is not None and int(rej.b) == 2,
            "not_hold_to_run_fault": any(n_ == "NotHoldToRun" for _, n_, _ in _fault_events(r)),
            "v_ref_max_mps": _rnd(float(np.nanmax(np.abs(vr)))),
            "v_max_mps": _rnd(float(np.max(np.abs(v)))),
            "ke_max_j": _rnd(0.5 * m * float(np.max(np.abs(v))) ** 2),
            "ke_gov_max_j": _rnd(_kpi(r, "ke_max_gov_j")),
            "momentum_max_kgmps": _rnd(_kpi(r, "momentum_max")),
            "stop_after_release_s": _rnd(float(t[still[0]]) - t_off if still.size else math.nan),
            "x_at_release_m": _rnd(float(np.interp(t_off, t, x))),
            "x_stop_m": _rnd(float(x[still[0]]) if still.size else math.nan),
            "arrived_s": _rnd(_t(_first(r, "Arrived", after=t_go2))),
            "landed_s": _rnd(_t(_first(r, "Landed", after=t_go2))),
            "locked_s": _rnd(_t(_first(r, "plant:locked", after=t_go2))),
            "x_final_m": _rnd(float(x[-1])),
            "faults": sorted({n_ for _, n_, _ in _fault_events(r)}),
            "monitor_trips": _monitor_trips(r),
        }

    red = [_red(d) for d in done]
    nom, sab = red[0], red[1]
    # Invariance of the governor limit: compare the commanded speed of the first hold-to-run
    # segment (identical start state and target). The resumed segment is re-planned from
    # where each run stopped, which differs with the tracking of each plant.
    dv = abs(_f(sab["v_ref_max_first_segment_mps"]) - _f(nom["v_ref_max_first_segment_mps"])) / _f(nom["v_ref_max_first_segment_mps"])
    t_stop_bound = v_cap / a_max + 0.1
    checks = [
        _check("go_to without hold-to-run rejected (NotHoldToRun)", nom["rejected_without_hold_to_run"] and nom["not_hold_to_run_fault"], True, "true", "", "FAB-SAF-003 HEAVY unsensed is hold-to-run", "FAB-SAF-003"),
        _check("stopped after hold-to-run release (|v| < 1 mm/s)", nom["stop_after_release_s"], t_stop_bound, "<=", "s", f"hold-to-run: stop within v_cap/a_max + 0.1 s = {t_stop_bound:.2f} s", "FAB-SAF-003"),
        _check("commanded (reference) peak speed", nom["v_ref_max_mps"], v_cap, "<=", "m/s", "FAB-SAF-002 HEAVY 0.025 m/s", "FAB-SAF-002"),
        _check("peak speed (plant truth)", nom["v_max_mps"], v_cap, "<=", "m/s", "FAB-SAF-002 HEAVY 0.025 m/s (truth; the monitor trips at 1.1·v_lim)", "FAB-SAF-002"),
        _check("mass estimate reads low in the sabotaged run (m̂ < m_rated/1.05)", _f(sab["m_hat_kg"]) - m_rated / 1.05, 0.0, "<", "kg", "precondition of the FAB-SAF-001 case"),
        _check("commanded cruise speed of the first segment, low m̂ vs nominal (relative difference)", dv, 1e-6, "<=", "", "FAB-SAF-001: a low m̂ changes nothing (m_gov = m_rated)", "FAB-SAF-001"),
        _check("kinetic energy ½·m·v² (truth), low-m̂ run", sab["ke_max_j"], e_cap, "<=", "J", "FAB-SAF-001 energy cap at the true mass", "FAB-SAF-001"),
        _check("landed and locked at the end (nominal)", nom["locked_s"] is not None, True, "true", "", "HEAVY rest LAND + lock"),
        _check("monitor trips (Carriage6 runs)", len(nom["monitor_trips"]) + len(sab["monitor_trips"]), 0, "==", "count", "a monitor trip in an example is a failure (FAB-EXA-007 rule)"),
    ]
    if not quick:
        checks.append(_check("monitor trips (Travel1d run)", len(red[2]["monitor_trips"]), 0, "==", "count", "a monitor trip in an example is a failure (FAB-EXA-007 rule)"))
    results = {
        "rail_class": "heavy",
        "m_kg": m,
        "m_rated_kg": m_rated,
        "v_cap_mps": v_cap,
        "e_cap_j": e_cap,
        "sabotage": {"plant.br_scale": _SABOTAGE_BR, "m_hat_expected_ratio": _rnd(1.0 / _SABOTAGE_BR**2)},
        "runs": red,
        "v_ref_relative_difference": _rnd(dv),
        "momentum_note": "HEAVY momentum is stated, not capped (safety.p_max_kgmps = 0 for HEAVY)",
    }
    trav = red[2] if not quick else None
    trav_txt = (
        f" A Travel1d run of the same flow {'trips ' + ', '.join(q['fault'] for q in trav['monitor_trips']) if trav['monitor_trips'] else 'runs without a monitor trip'}."
        if trav
        else ""
    )
    story = (
        f"A {m / 1000:g} t HEAVY archive carriage rests landed and locked. After lifting, a go_to without hold-to-run is "
        f"{'rejected (NotHoldToRun)' if nom['rejected_without_hold_to_run'] else 'not rejected'}; with hold-to-run held it moves at "
        f"{_fmt(nom['v_ref_max_mps'], '.4f')} m/s commanded ({_fmt(nom['v_max_mps'], '.4f')} m/s truth peak; cap {v_cap:g} m/s), and releasing the "
        f"switch stops it within {_fmt((nom['stop_after_release_s'] or math.nan) * 1e3, '.0f')} ms. The carriage lags its reference by up to "
        f"{_fmt((nom['v_lag_max_first_segment_mps'] or math.nan) * 1e3, '.1f')} mm/s (monitor force limit active for {_fmt(nom['limit_force_time_s'], '.2f')} s). It resumes on the next press, arrives through the "
        f"log-damped terminus and lands{' and locks' if nom['locked_s'] is not None else ''}. In a paired run the truth magnets are {_SABOTAGE_BR:g}× stronger, "
        f"so the estimator reads m̂ = {_fmt(sab['m_hat_kg'], '.0f')} kg for the {m:g} kg carriage; m_gov = max(m_rated, 1.05·m̂) = "
        f"{_fmt(sab['m_gov_kg'], '.0f')} kg and the commanded cruise speed differs by {_fmt(dv, '.1e')} (relative), so the kinetic energy stays at "
        f"{_fmt(sab['ke_max_j'], '.2f')} J against the {e_cap:g} J cap. Its momentum, about {_fmt(nom['momentum_max_kgmps'], '.0f')} kg·m/s, is stated, "
        f"not capped: hold-to-run is the protective measure.{trav_txt} {_failed_sentence(checks)}"
    )

    def figure(fig: Any) -> None:
        from .. import plotstyle

        C = plotstyle.COLORS
        ax = fig.subplots(2, 2)
        a0, a1, a2, a3 = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]
        cols = (C["s4"], C["s1"], C["ctrl"])
        disp = {"nominal": "nominal", "m_hat_low": f"m̂ read low (B_r × {_SABOTAGE_BR:g})", "nominal_travel1d": "nominal (Travel1d)"}
        for k, d in enumerate(done[:2]):
            r = d.result
            a0.plot(r.t_s, np.asarray(r["v_mps"]) * 1e3, color=cols[k], lw=1.6, label=f"{disp[d.job.label]} (truth)")
            if "v_ref_mps" in r:
                a0.plot(r.t_s, np.asarray(r["v_ref_mps"]) * 1e3, color=cols[k], lw=1.0, ls=(0, (4, 3)), label=f"{disp[d.job.label]} (reference)")
        plotstyle.ref_line(a0, v_cap * 1e3, f"cap {v_cap * 1e3:g} mm/s", color=C["star"])
        for t0_, t1_ in ((t_on1, t_off), (t_on2, dur)):
            a0.axvspan(t0_, t1_, color=C["band"], alpha=0.08)
        _vmark(a0, t_rej, "go_to rejected (no hold-to-run)")
        _vmark(a0, t_off, "hold-to-run released")
        _style(a0, "time (s)", "speed (mm/s)", "Hold-to-run (shaded: switch held)")
        _legend(a0, loc="lower right")
        for k, d in enumerate(done):
            r = d.result
            a1.plot(r.t_s, np.asarray(r["x_m"]) * 1e3, color=cols[k], lw=1.6, label=disp[d.job.label])
        _style(a1, "time (s)", "travel x (mm)", "Move, stop on release, resume, arrive and land")
        _legend(a1, loc="lower right")
        for k, d in enumerate(done[:2]):
            r = d.result
            lv = np.isin(np.asarray(r["lev_state"]), (5, 6, 7))
            a2.plot(r.t_s[lv], np.asarray(r["m_hat_kg"])[lv], ".", ms=1.5, color=cols[k], label=disp[d.job.label])
        plotstyle.ref_line(a2, m_rated, f"m_rated {m_rated:g} kg", color=C["star"])
        plotstyle.ref_line(a2, m_rated / 1.05, "m_rated/1.05: below it m_gov = m_rated", color=C["muted"], va="top")
        _style(a2, "time (s)", "mass estimate m̂ (kg)", "Mass estimate: nominal and read low")
        _legend(a2, loc="lower right", markerscale=6)
        for k, d in enumerate(done[:2]):
            r = d.result
            a3.plot(r.t_s, 0.5 * m * np.asarray(r["v_mps"]) ** 2, color=cols[k], lw=1.6, label=f"½·m·v² {disp[d.job.label]}")
        plotstyle.ref_line(a3, e_cap, f"E cap {e_cap:g} J", color=C["star"])
        _style(a3, "time (s)", "kinetic energy (J)", "Energy at the true mass stays under the cap")
        a3.set_ylim(0.0, 1.3 * e_cap)
        _legend(a3, loc="lower right")

    return _finish(_ARCHIVE, out_dir, quick=quick, seed=seed, runs=done, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t_start)


# =====================================================================================
# 6. drawer_power_loss (DRAWER)
# =====================================================================================

_POWER = _Meta(
    number=6,
    name="drawer_power_loss",
    title="an open drawer loses mains: SS1 safe landing, then re-lift or a manual re-seat",
    badge="✅",
    maturity="concept; supercapacitor hold-up for a controlled stop is established practice",
    story=(
        "An open DRAWER loses mains power. SS1 Safe Landing lowers it onto its travel rollers from the "
        "supercapacitor; when power returns a light drawer re-lifts, above 20 kg it cannot re-lift from the "
        "rollers and needs a manual push back into its seat."
    ),
    requirements=(
        ("FAB-SAF-009", "SS1 Safe Landing touchdown ≤ 5 mm/s on supply loss; hold-up ≥ 3× the landing need"),
        ("FAB-LEV-019", "fail direction: rests on rollers; DRAWER re-lift only up to 20 kg"),
        ("FAB-SAF-010", "manual force after landing reported against 22.2 N (the requirement itself covers DOOR/WALL)"),
    ),
    caveats=(
        "Manual re-seat required above 20 kg: a drawer that failed down onto its rollers cannot re-lift from "
        "6.5 mm at 24 V (F17; core non-claim 12).",
        "The manual push is simulated only for its first millimetres (the arm model is anchored); re-seating over "
        "the slot ramp is not simulated.",
        "Simulated; not measured on hardware.",
    ),
)


def _power(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    from .. import human

    seed, workers = _options(options)
    t_start = time.perf_counter()
    p = _params("drawer", seed)
    x0 = float(p.get("design.rail.x_closed_m"))
    travel = float(p.get("design.rail.travel_m"))
    m_star = float(p.get("design.carriage.m_star_kg"))
    der = p.derived()
    relift_max = 20.0  # FAB-LEV-019 / F17: fail-down re-lift works only up to 20 kg
    masses = [16.0, 26.0] if quick else [12.0, 16.0, 20.0, 26.0, 48.0]
    seeds = _noise_seeds(seed, len(masses))
    # The manual push happens during the outage, after the landing (LSM de-energised);
    # after power returns and a reset, a lift is tried once the supercap has recharged, and
    # a close command then re-seats any drawer that re-lifted.
    t_loss, t_push, t_restore, t_reset, t_lift, t_close, dur = 3.5, 4.8, 5.5, 6.0, 15.0, 16.5, 20.0
    push_n = 5.0
    jobs = []
    for i, m in enumerate(masses):
        ev = [
            {"kind": "power_loss", "t_s": t_loss, "dur_s": t_restore - t_loss},
            {"kind": "command", "t_s": t_reset, "cmd": "reset"},
            {"kind": "command", "t_s": t_lift, "cmd": "lift"},
            human.hand_profile("push", -push_n, t_push, 0.1, hold_s=0.4),
            {"kind": "command", "t_s": t_close, "cmd": "close"},
        ]
        if m <= 40.0:
            init = {"lev": "seated", "x_m": x0, "payload_kg": m - m_star}
            ev.append({"kind": "command", "t_s": 0.5, "cmd": "open"})
        else:  # slot exit at the rated 48 kg is kitchen_push_open's open item (FAB-LEV-017): start open
            init = {"lev": "levitated", "x_m": x0 + travel, "payload_kg": m - m_star}
        jobs.append(
            _Job(
                f"mass_{m:g}kg",
                "drawer",
                p,
                _spec(p, "drawer", f"power_loss_m{m:g}", dur, initial=init, events=ev, noise_seed=seeds[i], windows={"landing": [t_loss, t_push], "manual": [t_push, t_restore]}),
            )
        )
    done = _run_jobs(jobs, workers)

    per = []
    for m, d in zip(masses, done, strict=True):
        r = d.result
        landed = _events(r, "plant:landed", after=t_loss)
        first = min(landed, key=lambda e: e.t_s) if landed else None
        lifted = _first(r, "Lifted", after=t_lift)
        faults_land = sorted({n_ for t_, n_, _ in _fault_events(r) if t_loss <= t_ < t_reset})
        faults_lift = sorted({n_ for t_, n_, _ in _fault_events(r) if t_ >= t_lift})
        t = r.t_s
        x = np.asarray(r["x_m"])
        xm = x[(t >= t_push) & (t < t_restore)]
        per.append(
            {
                "mass_kg": m,
                "start": "seated, opened by command" if m <= 40.0 else "levitated at the open end",
                "open_reached": _first(r, "Arrived", after=0.5, before=t_loss) is not None or m > 40.0,
                "t_first_touchdown_after_loss_s": _rnd(_t(first) - t_loss),
                "touchdown_v_first_mps": _rnd(first.a) if first is not None else None,
                "touchdown_v_kpi_mps": _rnd(_kpi(r, "touchdown_v_mps", "landing")),
                "touchdown_contacts": len(landed),
                "faults_during_landing": faults_land,
                "e_hold_min_j": _rnd(_kpi(r, "e_hold_min_j", "landing")),
                "relift_ok": lifted is not None,
                "relift_time_s": _rnd(_t(lifted) - t_lift) if lifted is not None else None,
                "faults_on_relift": faults_lift,
                "manual_force_n": _rnd(_kpi(r, "manual_force_n", "manual")),
                "manual_push_displacement_m": _rnd(float(np.max(xm) - np.min(xm)) if xm.size else math.nan),
                "landed_before_push": first is not None and _t(first) < t_push,
                "reseated_after_close": _first(r, "Seated", after=t_close) is not None,
                "x_final_m": _rnd(float(x[-1])),
                "monitor_trips": _monitor_trips(r),
            }
        )
    checks = [
        _check(
            "hold-up energy ≥ 3× the landing need (preset, derived)",
            der["holdup_avail_j"] / der["holdup_need_j"],
            3.0,
            ">=",
            "×",
            "FAB-SAF-009 hold-up ≥ 3× landing need",
            "FAB-SAF-009",
        )
    ]
    for q in per:
        checks.append(_check(f"SS1 touchdown speed at {q['mass_kg']:g} kg (first roller contact, heave)", q["touchdown_v_first_mps"], 5e-3, "<=", "m/s", "FAB-SAF-009 ≤ 5 mm/s", "FAB-SAF-009"))
    for q in per:
        want = q["mass_kg"] <= relift_max
        checks.append(
            _check(
                f"re-lift from the rollers at {q['mass_kg']:g} kg {'succeeds' if want else 'is not possible'}",
                q["relift_ok"] == want,
                True,
                "true",
                "",
                f"FAB-LEV-019 / F17: re-lift only up to {relift_max:g} kg",
                "FAB-LEV-019",
            )
        )
    for q in per:
        checks.append(
            _check(
                f"a {push_n:g} N push moves the landed drawer at {q['mass_kg']:g} kg (≥ 5 mm in 0.5 s)",
                q["manual_push_displacement_m"],
                5e-3,
                ">=",
                "m",
                "manual operability after landing; 22.2 N reference (ADA 404.2.9; FAB-SAF-010 covers DOOR/WALL)",
                "FAB-SAF-010",
            )
        )
    for q in per:
        if q["relift_ok"]:
            checks.append(_check(f"re-lifted drawer at {q['mass_kg']:g} kg closes into its seat", q["reseated_after_close"], True, "true", "", "plan §4b drawer_power_loss"))
    checks.append(_check("monitor trips (all runs)", sum(len(q["monitor_trips"]) for q in per), 0, "==", "count", "a monitor trip in an example is a failure (FAB-EXA-007 rule)"))
    results = {
        "rail_class": "drawer",
        "relift_limit_kg": relift_max,
        "holdup_avail_j": _rnd(der["holdup_avail_j"]),
        "holdup_need_j": _rnd(der["holdup_need_j"]),
        "manual_force_relieved_n_closed_form": _rnd(der["manual_force_relieved_n"]),
        "manual_force_bound_n_closed_form": _rnd(der["manual_force_bound_n"]),
        "timeline_s": {"power_loss": t_loss, "manual_push": t_push, "power_restored": t_restore, "reset": t_reset, "lift": t_lift, "close": t_close},
        "manual_push_n": push_n,
        "per_mass": per,
    }
    light = [q for q in per if q["relift_ok"]]
    heavy = [q for q in per if not q["relift_ok"]]
    parts = []
    for q in per:
        parts.append(
            f"{q['mass_kg']:g} kg: first touchdown {_fmt((q['touchdown_v_first_mps'] or math.nan) * 1e3, '.1f')} mm/s"
            + (f" ({', '.join(q['faults_during_landing'])})" if set(q["faults_during_landing"]) - {"MainsLost"} else "")
        )
    story = (
        f"An open DRAWER loses mains power at {t_loss:g} s. SS1 Safe Landing lowers it from the supercapacitor onto its travel rollers "
        f"(hold-up {_fmt(der['holdup_avail_j'], '.0f')} J against a landing need of {_fmt(der['holdup_need_j'], '.0f')} J): {'; '.join(parts)} "
        f"(limit 5 mm/s). Power returns at {t_restore:g} s and the fault is reset; after the supercapacitor recharges a lift is commanded at "
        f"{t_lift:g} s. "
        + (f"{', '.join(format(q['mass_kg'], 'g') for q in light)} kg re-lift{'s' if len(light) == 1 else ''} in {', '.join(_fmt((q['relift_time_s'] or math.nan) * 1e3, '.0f') for q in light)} ms. " if light else "")
        + (
            f"{', '.join(format(q['mass_kg'], 'g') for q in heavy)} kg cannot re-lift from the rollers ({', '.join(sorted({f for q in heavy for f in q['faults_on_relift']})) or 'refused'}): "
            f"manual re-seat required. "
            if heavy
            else ""
        )
        + f"During the outage a {push_n:g} N push (22.2 N reference; closed-form rolling force {_fmt(der['manual_force_relieved_n'], '.2f')} N at m*) moves the landed drawers by "
        + ", ".join(f"{_fmt((q['manual_push_displacement_m'] or math.nan) * 1e3, '.1f')} mm ({q['mass_kg']:g} kg)" for q in per)
        + ". "
        + _failed_sentence(checks)
    )

    def figure(fig: Any) -> None:
        from .. import plotstyle

        C = plotstyle.COLORS
        ax = fig.subplots(2, 2)
        a0, a1, a2, a3 = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]
        for k, (q, d) in enumerate(zip(per, done, strict=True)):
            r = d.result
            sel = (r.t_s >= t_loss - 0.2) & (r.t_s <= t_loss + 1.2)
            gaps = [np.asarray(r[f"gap_{i}_m"]) for i in range(int(p.get("design.suspension.n_magnets")))]
            a0.plot(r.t_s[sel] - t_loss, np.mean(gaps, axis=0)[sel] * 1e3, color=SERIES_ORDER[k % 5], label=f"{q['mass_kg']:g} kg")
        plotstyle.ref_line(a0, float(p.get("design.suspension.g_land_m")) * 1e3, "rollers (g_land)", color=C["star"])
        plotstyle.ref_line(a0, float(p.get("design.suspension.g_star_m")) * 1e3, "g* (zero power)", color=C["muted"], va="top")
        _style(a0, "time since mains loss (s)", "mean gap (mm)", "SS1 Safe Landing onto the rollers")
        _legend(a0, loc="center right")
        idx = np.arange(len(per))
        a1.bar(idx, [_f(q["touchdown_v_first_mps"]) * 1e3 for q in per], 0.6, color=[SERIES_ORDER[k % 5] for k in range(len(per))])
        plotstyle.ref_line(a1, 5.0, "5 mm/s (FAB-SAF-009)", color=C["star"])
        a1.set_xticks(idx, [f"{q['mass_kg']:g} kg" for q in per])
        a1.set_yscale("log")
        a1.set_ylim(0.5, 2.0 * max(10.0, max(_f(q["touchdown_v_first_mps"]) * 1e3 for q in per)))
        for i_, q in enumerate(per):
            a1.annotate(_fmt(_f(q["touchdown_v_first_mps"]) * 1e3, ".1f", "mm/s"), (i_, _f(q["touchdown_v_first_mps"]) * 1e3), ha="center", va="bottom", fontsize=8.5, color=C["ink2"], xytext=(0, 2), textcoords="offset points")
        _style(a1, None, "first touchdown speed (mm/s, log)", "Touchdown speed by drawer mass")
        a2.bar(idx, [1.0] * len(per), 0.6, color=[C["good"] if q["relift_ok"] else C["star"] for q in per], alpha=0.85)
        for i_, q in enumerate(per):
            a2.annotate("re-lifted\n(" + _fmt((q["relift_time_s"] or math.nan) * 1e3, ".0f", "ms") + ")" if q["relift_ok"] else "cannot re-lift:\nmanual re-seat", (i_, 0.5), ha="center", va="center", fontsize=9, color=C["surface"])
        if len(per) > 1:
            a2.axvline(float(np.interp(relift_max, [q["mass_kg"] for q in per], idx)), color=C["ink2"], ls=(0, (3, 3)), lw=1.2)
        a2.set_xticks(idx, [f"{q['mass_kg']:g} kg" for q in per])
        a2.set_yticks([])
        a2.set_ylim(0.0, 1.15)
        _style(a2, None, None, f"Re-lift from the rollers after power returns (limit {relift_max:g} kg, dashed)")
        a2.grid(False)
        for k, (q, d) in enumerate(zip(per, done, strict=True)):
            r = d.result
            sel = (r.t_s >= t_push - 0.1) & (r.t_s < t_restore)
            a3.plot(r.t_s[sel] - t_push, (np.asarray(r["x_m"])[sel] - float(np.interp(t_push, r.t_s, np.asarray(r["x_m"])))) * 1e3, color=SERIES_ORDER[k % 5], label=f"{q['mass_kg']:g} kg")
        _style(a3, "time since the push (s)", "displacement (mm)", f"A {push_n:g} N manual push on the landed drawer")
        _legend(a3, loc="lower right")

    return _finish(_POWER, out_dir, quick=quick, seed=seed, runs=done, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t_start)


# =====================================================================================
# 7. wall_pwave_m7 (WALL)
# =====================================================================================

_PWAVE = _Meta(
    number=7,
    name="wall_pwave_m7",
    title="a moving partition freezes on the P-wave and lands and locks before the S-wave",
    badge="🔶",
    maturity="on-site P-wave triggering of automated actions is in service in other industries; this device response is a concept",
    story=(
        "A WALL partition is moving when a synthetic M7 earthquake 20 km away arrives. The structure "
        "accelerometer picks the P-wave, the partition freezes, PROTECT is declared about 1.2 s later and the "
        "partition lands and locks before the S-wave: land-first, with the timeline against t_S. The seismic "
        "action is not a rated safety function (SF-08)."
    ),
    requirements=(
        ("FAB-SEI-009", "a local pick reaches FREEZE within ≤ 100 ms"),
        ("FAB-SEI-010", "land-first class action: M7 R20 gives PROTECT at t_P + 1.2 ± 0.2 s and freeze before t_S"),
    ),
    caveats=(
        "The seismic action is not a rated safety function (SF-08): it reduces motion and exposure, it is not "
        "certified protection.",
        "The ground motion is synthetic (Boore 2003 stochastic, P scaled to Wu–Zhao Pd at hypocentral R). No "
        "network alert is used here, only the on-site P pick; any alert in the gallery is synthetic/injected, "
        "not a live EEW feed, and fabrail is not an EEW provider.",
        "FAB-SEI-010 is status S, so this example's badge is 🔶 (the plan table lists ✅; FAB-EXA-002 caps it).",
        "Travel1d emulates heave kinematically for the landing; simulated, not measured on hardware.",
    ),
)


def _pwave(out_dir: Path, *, quick: bool = False, **options: Any) -> dict[str, Any]:
    from .. import scenarios
    from ..analysis import analyze_seismic
    from ..config import RunConfig
    from ..ensemble import member_seeds

    seed, workers = _options(options)
    t_start = time.perf_counter()
    cfg = RunConfig(rail_class="wall", seed=seed, coil="voltage", record_rate_hz=200.0)
    p = cfg.params()
    n_seeds = 1 if quick else 5
    members = member_seeds(seed, n_seeds)
    jobs = []
    tiers = [("travel1d", m) for m in members]
    if not quick:
        tiers.append(("carriage6", members[0]))
    for tier, mem in tiers:
        spec, series = scenarios.spec("pwave_m7_r20", p, cfg, fidelity=tier, member=mem)
        spec["record"]["channels"] = _channels(p)
        if tier == "carriage6":
            spec["record"]["rate_hz"] = 1000.0
        scenarios.validate_spec(spec)
        scenarios.cross_check(spec, series, p)
        jobs.append(_Job(f"{tier}_member{mem.index}", "wall", p, spec, series))
    done = _run_jobs(jobs, workers)

    per = []
    for d in done:
        r = d.result
        sa = analyze_seismic(r)
        t = r.t_s
        v = np.asarray(r["v_mps"])
        x = np.asarray(r["x_m"])
        frz = _first(r, "Freeze")
        stopped = np.flatnonzero((t >= _t(frz)) & (np.abs(v) < 1e-3)) if frz is not None else np.zeros(0, dtype=int)
        locked = sa.lock_t_s
        after_lock = t >= locked if math.isfinite(locked) else np.zeros_like(t, dtype=bool)
        info = sa.info
        per.append(
            {
                "run": d.job.label,
                "fidelity": d.job.spec["fidelity"]["model"],
                "noise_seed": d.job.spec["noise_seed"],
                "magnitude": info.get("magnitude"),
                "r_epi_km": info.get("r_epi_km"),
                "depth_km": info.get("depth_km"),
                "r_hyp_km": _rnd(info.get("r_hyp_km")),
                "pd_target_m": _rnd(info.get("pd_target_m")),
                "pga_mps2": _rnd(sa.pga_mps2),
                "t_p_s": _rnd(sa.t_p_s),
                "t_s_s": _rnd(sa.t_s_arrival_s),
                "trigger_s": _rnd(sa.trigger_t_s),
                "freeze_s": _rnd(sa.freeze_t_s),
                "freeze_after_trigger_s": _rnd(sa.freeze_t_s - sa.trigger_t_s),
                "stopped_after_trigger_s": _rnd(float(t[stopped[0]]) - sa.trigger_t_s if stopped.size else math.nan),
                "protect_s": _rnd(sa.protect_t_s),
                "protect_after_p_s": _rnd(sa.protect_t_s - sa.t_p_s),
                "level_max": int(sa.level_max),
                "pd_m_at_protect": _rnd(sa.pd_m),
                "tau_c_s_at_protect": _rnd(sa.tau_c_s),
                "landed_s": _rnd(sa.land_t_s),
                "locked_s": _rnd(locked),
                "lock_margin_to_s_s": _rnd(sa.t_s_arrival_s - locked),
                "x_drift_after_lock_m": _rnd(float(np.max(np.abs(x[after_lock] - x[after_lock][0]))) if after_lock.any() else math.nan),
                "timeline": [(round(tt, 3), lab) for tt, lab in sa.timeline[:40]],
                "faults": sorted({n_ for _, n_, _ in _fault_events(r)}),
                "monitor_trips": _monitor_trips(r),
            }
        )
    checks = []
    for q in per:
        tag = f" ({q['run'].replace('_', ' ')})"
        checks += [
            _check("FREEZE after the local pick" + tag, q["freeze_after_trigger_s"], 0.100, "<=", "s", "FAB-SEI-009 ≤ 100 ms", "FAB-SEI-009"),
            _check("PROTECT after t_P" + tag, q["protect_after_p_s"], [1.0, 1.4], "in", "s", "FAB-SEI-010 M7 R20: t_P + 1.2 ± 0.2 s", "FAB-SEI-010"),
            _check("detector level" + tag, q["level_max"], 3, "==", "", "FAB-SEI-010 M7 R20: L3", "FAB-SEI-010"),
            _check("landed and locked before t_S (margin)" + tag, q["lock_margin_to_s_s"], 0.0, ">", "s", "FAB-SEI-010 land-first", "FAB-SEI-010"),
            _check("monitor trips" + tag, len(q["monitor_trips"]), 0, "==", "count", "a monitor trip in an example is a failure (FAB-EXA-007 rule)"),
        ]
    q0 = per[0]
    results = {
        "rail_class": "wall",
        "scenario": "pwave_m7_r20",
        "ground_motion": "synthetic (Boore 2003 stochastic; P scaled to Wu–Zhao at hypocentral R)",
        "alert": "none: on-site P pick only (no network alert; any alert would be synthetic/injected, not a live EEW feed)",
        "runs": per,
    }
    if len(per) > 1:
        tr = [q for q in per if q["fidelity"] == "travel1d"]
        results["seed_summary"] = {
            "n": len(tr),
            "protect_after_p_s": [q["protect_after_p_s"] for q in tr],
            "lock_margin_to_s_s": [q["lock_margin_to_s_s"] for q in tr],
        }
    story = (
        f"A {float(p.get('design.carriage.m_star_kg')):g} kg WALL partition is moving when a synthetic M{q0['magnitude']:g} earthquake "
        f"(R_epi {q0['r_epi_km']:g} km, {q0['depth_km']:g} km deep, R_hyp {_fmt(q0['r_hyp_km'], '.1f')} km) arrives: P at {_fmt(q0['t_p_s'], '.2f')} s, "
        f"S at {_fmt(q0['t_s_s'], '.2f')} s in the record. The structure accelerometer picks the P-wave at {_fmt(q0['trigger_s'], '.3f')} s and the "
        f"partition freezes {_fmt((q0['freeze_after_trigger_s'] or math.nan) * 1e3, '.0f')} ms later (standstill after "
        f"{_fmt((q0['stopped_after_trigger_s'] or math.nan) * 1e3, '.0f')} ms). PROTECT follows at t_P + {_fmt(q0['protect_after_p_s'], '.2f')} s "
        f"(level L{q0['level_max']}, Pd {_fmt((q0['pd_m_at_protect'] or math.nan) * 100, '.2f')} cm); the partition lands at {_fmt(q0['landed_s'], '.2f')} s and "
        f"locks at {_fmt(q0['locked_s'], '.2f')} s, {_fmt(q0['lock_margin_to_s_s'], '.2f')} s before the S-wave, and stays within "
        f"{_fmt((q0['x_drift_after_lock_m'] or math.nan) * 1e3, '.2f')} mm while the floor shakes at up to {_fmt(q0['pga_mps2'], '.2f')} m/s². "
        + (
            f"Over {results['seed_summary']['n']} synthesis seeds PROTECT falls at t_P + "
            f"{_fmt(min(results['seed_summary']['protect_after_p_s']), '.2f')}–{_fmt(max(results['seed_summary']['protect_after_p_s']), '.2f')} s and the lock "
            f"precedes t_S by {_fmt(min(results['seed_summary']['lock_margin_to_s_s']), '.2f')}–{_fmt(max(results['seed_summary']['lock_margin_to_s_s']), '.2f')} s. "
            if "seed_summary" in results
            else ""
        )
        + "The seismic action is not a rated safety function (SF-08). "
        + _failed_sentence(checks)
    )

    def figure(fig: Any) -> None:
        from .. import plotstyle
        from ..analysis import LEV_STATE_NAMES, SEISMIC_STATE_NAMES

        C = plotstyle.COLORS
        ax = fig.subplots(2, 2)
        a0, a1, a2, a3 = ax[0, 0], ax[0, 1], ax[1, 0], ax[1, 1]
        r = done[0].result
        t = r.t_s
        tp, ts = _f(q0["t_p_s"]), _f(q0["t_s_s"])
        win = (t >= tp - 3.0) & (t <= ts + 12.0)
        ah = np.hypot(np.asarray(r["floor_ax_mps2"]), np.asarray(r["floor_ay_mps2"]))
        a0.plot(t[win], ah[win], color=C["s3"], lw=1.0, label="|a_xy| floor")
        a0.plot(t[win], np.asarray(r["floor_az_mps2"])[win], color=C["s1"], lw=0.8, label="a_z floor")
        _vmark(a0, tp, "t_P", C["star"])
        _vmark(a0, ts, "t_S", C["star"])
        _style(a0, "time (s)", "floor acceleration (m/s²)", f"Synthetic M{q0['magnitude']:g} at R_epi {q0['r_epi_km']:g} km (record)")
        _legend(a0, loc="lower left")
        # Gantt of the seismic and levitation states.
        rows = [("seismic", np.asarray(r["seismic_state"]), SEISMIC_STATE_NAMES), ("carriage", np.asarray(r["lev_state"]), LEV_STATE_NAMES)]
        palette = {"Normal": C["grid"], "Triggered": C["s4"], "Protect": C["s2"], "Shaking": C["s5"], "Quiet": C["s3"], "Inspect": C["ctrl"],
                   "LevitatedIdle": C["grid"], "Moving": C["s1"], "Seismic": C["s4"], "Landing": C["s2"], "Landed": C["s3"], "Stopping": C["s5"]}
        seen: set[str] = set()
        for yi, (_lab, codes, names) in enumerate(rows):
            tw, cw = t[win], codes[win]
            edges = np.flatnonzero(np.diff(cw.astype(int))) + 1
            starts = np.concatenate(([0], edges))
            ends = np.concatenate((edges, [len(cw) - 1]))
            for s0, s1 in zip(starts, ends, strict=True):
                nm = names[int(cw[s0])] if int(cw[s0]) < len(names) else "?"
                col = palette.get(nm, C["muted"])
                a1.barh(yi, tw[s1] - tw[s0], left=tw[s0], height=0.5, color=col, label=nm if nm not in seen else None)
                seen.add(nm)
        lk = _f(q0["locked_s"])
        if math.isfinite(lk):
            a1.plot([lk], [1], marker="D", color=C["ink"], ms=7, ls="none", label="locked")
        _vmark(a1, tp, "t_P", C["star"])
        _vmark(a1, ts, "t_S", C["star"])
        a1.set_yticks([0, 1], ["seismic state", "carriage state"])
        _style(a1, "time (s)", None, "Land-first timeline against t_S")
        a1.grid(False)
        _legend(a1, loc="upper center", ncol=5, bbox_to_anchor=(0.5, -0.18))
        a2.plot(t[win], np.asarray(r["x_m"])[win] * 1e3, color=C["s3"], label="partition x (truth)")
        _vmark(a2, _f(q0["freeze_s"]), "freeze")
        _vmark(a2, _f(q0["locked_s"]), "locked")
        _style(a2, "time (s)", "travel x (mm)", "Travel: freeze on the pick, hold through the shaking")
        _legend(a2, loc="lower right")
        tr = [q for q in per if q["fidelity"] == "travel1d"]
        idx = np.arange(len(tr))
        w = 0.38
        a3.bar(idx - w / 2, [_f(q["protect_after_p_s"]) for q in tr], w, color=C["s2"], label="PROTECT − t_P (s)")
        a3.bar(idx + w / 2, [_f(q["lock_margin_to_s_s"]) for q in tr], w, color=C["s3"], label="t_S − locked (s)")
        a3.axhspan(1.0, 1.4, color=C["band"], alpha=0.12)
        plotstyle.ref_line(a3, 1.2, "1.2 ± 0.2 s (FAB-SEI-010)", color=C["star"])
        a3.set_xticks(idx, [f"seed {q['run'].split('member')[1]}" for q in tr])
        a3.set_ylim(0.0, 2.0)
        _style(a3, None, "time (s)", "PROTECT timing and the lock margin by synthesis seed")
        _legend(a3, loc="upper left", ncol=2)

    return _finish(_PWAVE, out_dir, quick=quick, seed=seed, runs=done, results=results, checks=checks, story=story, figure=figure, wall_s=time.perf_counter() - t_start)


# =====================================================================================
# Registry
# =====================================================================================


def _spec_of(meta: _Meta, run: Callable[..., Mapping[str, object]]) -> ExampleSpec:
    return ExampleSpec(name=meta.name, badge=meta.badge, maturity=meta.maturity, story=_guard(meta.story, meta.name), run=run)


#: The core-tier gallery examples 1–7, in gallery order (plan §4b).
EXAMPLES: list[ExampleSpec] = [
    _spec_of(_KITCHEN, _kitchen),
    _spec_of(_PANTRY, _pantry),
    _spec_of(_OFFICE, _office),
    _spec_of(_BALLROOM, _ballroom),
    _spec_of(_ARCHIVE, _archive),
    _spec_of(_POWER, _power),
    _spec_of(_PWAVE, _pwave),
]

#: Static facts per example (number, title, requirements, caveats) for docs and tests.
META: dict[str, _Meta] = {m.name: m for m in (_KITCHEN, _PANTRY, _OFFICE, _BALLROOM, _ARCHIVE, _POWER, _PWAVE)}


def register_all() -> list[ExampleSpec]:
    """Register :data:`EXAMPLES` in :data:`fabrail.gallery.registry.REGISTRY` (idempotent).

    Called on import; calling it again, or after the registry already holds these very
    specs, is a no-op. A different spec already registered under one of these names is
    an error (the registry rejects duplicates).
    """
    for spec in EXAMPLES:
        if REGISTRY.get(spec.name) is spec:
            continue
        register(spec)
    return list(EXAMPLES)


def run_example(name: str, out_dir: str | Path | None = None, *, quick: bool = False, **options: Any) -> Mapping[str, object]:
    """Run core example ``name`` into ``out_dir`` (default ``examples/out/<name>``); returns the record."""
    from .registry import OUT_DIR

    spec = next((s for s in EXAMPLES if s.name == name), None)
    if spec is None:
        raise KeyError(f"{name!r} is not a core example (core examples: {', '.join(s.name for s in EXAMPLES)})")
    target = Path(out_dir) if out_dir is not None else OUT_DIR / name
    return spec.run(target, quick=quick, **options)


register_all()


def script_main(name: str, argv: Sequence[str] | None = None, *, doc: str | None = None) -> int:
    """Entry point of the thin ``examples/<name>.py`` scripts.

    ``[--quick] [--seed N] [--output DIR] [--strict]``: runs the example into
    ``DIR/<name>/`` (default ``examples/out/<name>/`` next to the script), prints the story
    and the checks, and returns 0 (or 1 with ``--strict`` when a check fails).
    """
    import argparse

    from .registry import OUT_DIR

    meta = META[name]
    ap = argparse.ArgumentParser(prog=f"{name}.py", description=doc or f"{meta.number}. {name}: {meta.title}")
    ap.add_argument("--quick", action="store_true", help="reduced run (the test-suite default)")
    ap.add_argument("-s", "--seed", type=int, default=42, help="master seed (default: 42)")
    ap.add_argument("-o", "--output", type=Path, default=None, help=f"output root (default: {OUT_DIR})")
    ap.add_argument("--workers", type=int, default=None, help="worker threads (results do not depend on it)")
    ap.add_argument("--strict", action="store_true", help="exit 1 when any check fails")
    args = ap.parse_args(list(argv) if argv is not None else None)
    root = args.output if args.output is not None else OUT_DIR
    out = Path(root) / name
    rec = run_example(name, out, quick=args.quick, seed=args.seed, workers=args.workers)
    print(f"{meta.number}. {name} ({meta.badge}) -> {out}/figure.png, record.json, story.txt")
    print(rec["story"])
    for c in rec["checks"]:  # type: ignore[union-attr]
        bound = " ".join(str(x) for x in (c["op"], c["limit"], c["unit"]) if x not in ("", None))
        print(f"  [{'PASS' if c['pass'] else 'FAIL'}] {c['what']}: {c['value']} ({bound}; {c['source']})")
    ok = bool(rec["all_checks_pass"])
    return 0 if ok or not args.strict else 1


def requirement_status_drift(spec_markdown: str) -> list[str]:
    """Mismatches between :data:`REQUIREMENT_STATUS` and the St. column of the given
    SPECIFICATION.md text (empty list = in sync). For the drift tests."""
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
