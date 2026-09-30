"""The fabrail command line (core design §B.9; FAB-SW-001).

Usage::

    python -m fabrail presets [--json PATH]
    python -m fabrail params -c door [--describe | --markdown]
    python -m fabrail scenarios [-c drawer]
    python -m fabrail simulate -c door --scenario push_open [--coil voltage] [--json output/door.json]
    python -m fabrail compare --scenario terminus_soft_close [--classes drawer,door]
    python -m fabrail ensemble -c wall --scenario isolation_kt -n 20 --set control.seismic.isolation.enable=true
    python -m fabrail seismic -c wall --ground pwave --magnitude 7 --distance-km 20
    python -m fabrail detect --input record.csv --fs 500 --columns z,x,y
    python -m fabrail study all --tier 1
    python -m fabrail anchors [--quick] [--write-tests] [--check]
    python -m fabrail trace [--json]
    python -m fabrail selftest
    python -m fabrail data fetch elcentro --url URL
    python -m fabrail exo simulate --class sentinel --scenario XS12
    python -m fabrail example run villa_quake --quick

Every subcommand sets a ``handler(args) -> int`` default; :func:`main` dispatches to
it and returns the exit status: 0 success, 1 a failed check or rejected input
(``ValueError("path: reason")``, a missing native build, a missing file), 2 a
subcommand whose body belongs to a later wave. The native core is imported only
inside handlers, so ``--help`` and ``trace`` work without ``fabrail._native``.

Outputs. ``simulate`` and ``seismic`` write figures (``<output>/<stem>_<kind>.png``)
and always a JSON record (``--json PATH``, default ``<output>/<stem>.json``): the
``fabrail.run/1`` document of :func:`fabrail.export.run_record` plus three additive
keys: ``config`` (seed, class, scenario, tier, coil, overrides), ``analysis`` (every
:mod:`fabrail.analysis` object the figures draw, so the record holds every plotted
number, FAB-SW-009) and ``figures`` (file names); ``sha256`` is re-sealed over them
and ``wall_s`` stays in the unhashed ``timing`` block. ``compare``, ``ensemble``,
``detect``, ``presets``, ``anchors`` and ``selftest`` write their own canonical JSON
documents (``fabrail.compare/1``, ``fabrail.ensemble/1`` plus ``analysis``,
``fabrail.detect/1``, ``fabrail.presets/1``, ``fabrail.anchors/1``,
``fabrail.selftest/1``), each sealed the same way.

Recording. By default ``simulate``/``seismic``/``compare`` record the recorder's
default channels plus :data:`fabrail.analysis.ANALYSIS_EXTRA_CHANNELS` (reference,
energy ledger, detector estimates); ``--channels default`` keeps the scenario's own
set. Recording never changes the dynamics.

Honesty. Every alert here is synthetic/injected, never a live EEW feed; isolation and
AMD (``seismic --mode isolate|damp``) are research tier; nothing printed is a
compliance or certification statement.

Hooks: ``exo`` comes from :func:`fabrail.exotic.cli.register` and ``example`` from
:func:`fabrail.gallery.cli.register`; each receives the subparsers object and adds its
own parser with a ``handler`` default. If a hook is missing or fails to register, a
placeholder parser keeps the rest of the CLI working.

Owner: W0-C2 (skeleton), then B1 (Wave 2).
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import os
import sys
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

from . import __version__
from ._pending import NotYetImplementedError
from .config import COILS, FIDELITIES
from .presets import CLASSES

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .config import RunConfig
    from .results import RunResult

Handler = Callable[[argparse.Namespace], int]

__all__ = ["main", "make_parser"]

#: Default ensemble recording rate, Hz, when ``--record-rate`` is not given (KPIs are
#: online base-rate accumulators, so they do not depend on it; channels only cost memory).
ENSEMBLE_RECORD_RATE_HZ = 50.0

#: KPIs the run summary prints when present (whole-run values; windowed copies are in the JSON).
_SUMMARY_KPIS: tuple[tuple[str, str, float], ...] = (
    ("breakaway_n", "N", 1.0),
    ("apparent_mass_err", "", 1.0),
    ("intent_onset_s", "ms", 1e3),
    ("intent_confirm_s", "ms", 1e3),
    ("hover_gap_mean_m", "mm", 1e3),
    ("hover_gap_rms_m", "µm", 1e6),
    ("hover_power_w", "W", 1.0),
    ("coil_i_ss_a", "A", 1.0),
    ("lift_time_s", "s", 1.0),
    ("unseat_time_s", "s", 1.0),
    ("land_time_s", "s", 1.0),
    ("touchdown_v_mps", "mm/s", 1e3),
    ("ke_max_gov_j", "J", 1.0),
    ("momentum_max", "kg·m/s", 1.0),
    ("contact_force_max_n", "N", 1.0),
    ("reverse_latency_s", "ms", 1e3),
    ("manual_force_n", "N", 1.0),
    ("soft_close_v_final", "mm/s", 1e3),
    ("overshoot_m", "mm", 1e3),
    ("terminus_r2_min", "", 1.0),
    ("terminus_tau_fit_s", "s", 1.0),
    ("terminus_t_arrive_s", "s", 1.0),
    ("terminus_v_arrive_max_mps", "mm/s", 1e3),
    ("trigger_t_s", "s", 1.0),
    ("freeze_t_s", "s", 1.0),
    ("level_max", "", 1.0),
    ("protect_margin_to_s_s", "s", 1.0),
    ("stroke_max_m", "mm", 1e3),
    ("bumper_impacts", "", 1.0),
    ("bumper_energy_j", "J", 1.0),
    ("p_bus_max_w", "W", 1.0),
)


# --------------------------------------------------------------------------- argument types


def _positive_int(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected a positive integer, got {text!r}") from None
    if value < 1:
        raise argparse.ArgumentTypeError(f"expected a positive integer, got {text!r}")
    return value


def _nonneg_int(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected a non-negative integer, got {text!r}") from None
    if value < 0:
        raise argparse.ArgumentTypeError(f"expected a non-negative integer, got {text!r}")
    return value


def _positive_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected a positive number, got {text!r}") from None
    if not value > 0.0 or value == float("inf"):
        raise argparse.ArgumentTypeError(f"expected a positive finite number, got {text!r}")
    return value


def _finite_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected a number, got {text!r}") from None
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"expected a finite number, got {text!r}")
    return value


# --------------------------------------------------------------------------- shared helpers


def _warn(message: str) -> None:
    print(f"fabrail: {message}", file=sys.stderr)


def _native() -> Any:
    from . import require_native

    return require_native()


def _fmt(value: Any, digits: int = 4) -> str:
    if isinstance(value, bool) or value is None:
        return "—" if value is None else str(value).lower()
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return "—" if not math.isfinite(value) else f"{value:.{digits}g}"
    return str(value)


def _seal(doc: dict[str, Any], timing: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Canonical JSON form of ``doc`` with ``sha256`` over everything but ``timing``/``wall_s``."""
    from . import export

    body = export.to_jsonable({k: v for k, v in doc.items() if k not in ("sha256", "timing")})
    body["sha256"] = export.payload_sha256(body)
    if timing:
        body["timing"] = export.to_jsonable(dict(timing))
    return body


def _write_doc(path: Path, doc: Mapping[str, Any], *, compact: bool = False) -> Path:
    """Canonical JSON to ``path``; ``compact`` (no indentation) for the large analysis records."""
    from . import export

    return export.write_json(path, doc, indent=None if compact else 2)


def _options(items: Sequence[str] | None) -> dict[str, object]:
    from .config import parse_set

    return parse_set(items or [])


def _requires_note(name: str, params: Any) -> None:
    """Warn when a research-tier scenario runs without the parameters it is meant for (never applied silently)."""
    from .scenarios import CATALOG

    sdef = CATALOG.get(name)
    if sdef is None:
        return
    missing = [f"{p}={json.dumps(v)}" for p, v in sdef.requires.items() if params.get(p) != v]
    if missing:
        _warn(f"scenario {name!r} is research tier and meant for --set {' --set '.join(missing)}; running with the preset values")


def _set_channels(spec: dict[str, Any], params: Any, mode: str) -> None:
    from .analysis import analysis_channel_names

    if mode == "analysis" and spec["record"]["channels"] == "default":
        spec["record"]["channels"] = analysis_channel_names(int(params.get("design.suspension.n_magnets")))


def _native_scenario(spec: dict[str, Any], series: Mapping[str, bytes], params: Any) -> Any:
    from . import scenarios

    scenarios.validate_spec(spec)
    scenarios.cross_check(spec, series, params)
    return _native().Scenario.from_json(scenarios.to_json(spec), dict(series))


def _run(params: Any, scenario: Any) -> RunResult:
    from .results import RunResult

    return RunResult.from_native(_native().run(params, scenario))


def _config_block(cfg: RunConfig, command: str, spec: Mapping[str, Any], **extra: Any) -> dict[str, Any]:
    return {
        "command": command,
        "seed": cfg.seed,
        "rail_class": cfg.rail_class,
        "fidelity": spec["fidelity"]["model"],
        "coil": spec["fidelity"]["coil"],
        "duration_s": spec["duration_s"],
        "record_rate_hz": spec["record"]["rate_hz"],
        "overrides": dict(cfg.overrides),
        "plant": dict(cfg.plant),
        **extra,
    }


def _run_doc(
    result: RunResult, analyses: Mapping[str, Any], config: Mapping[str, Any], figures: Sequence[str], include_channels: bool
) -> dict[str, Any]:
    """``fabrail.run/1`` + ``config`` + ``analysis`` + ``figures`` (additive keys), re-sealed."""
    from . import export

    doc: dict[str, Any] = dict(export.run_record(result, include_channels=include_channels))
    timing = doc.pop("timing", {})
    doc["config"] = dict(config)
    doc["analysis"] = {k: export.to_jsonable(v) for k, v in analyses.items()}
    doc["figures"] = list(figures)
    return _seal(doc, timing)


_PLOTTERS = {
    "levitation": "plot_levitation",
    "haptics": "plot_haptics",
    "safety": "plot_safety",
    "seismic": "plot_seismic",
}


def _plot(analyses: Mapping[str, Any], kinds: Sequence[str], out_dir: Path, stem: str) -> list[str]:
    from . import visualization

    names = []
    for kind in kinds:
        a = analyses.get(kind)
        if a is None or getattr(a, "t_s", None) is None or a.t_s.size == 0:
            continue
        name = f"{stem}_{kind}.png"
        getattr(visualization, _PLOTTERS[kind])(a, out_dir / name)
        names.append(name)
    return names


def _seismic_relevant(result: RunResult, analyses: Mapping[str, Any]) -> bool:
    sei = analyses.get("seismic")
    if sei is None:
        return False
    floor = getattr(sei, "floor_acc_mps2", None)
    moved = floor is not None and floor.size > 0 and bool((abs(floor) > 0).any())
    return moved or sei.level_max > 0 or math.isfinite(sei.t_p_s) or math.isfinite(sei.alert_t_s)


def _print_run_summary(result: RunResult, analyses: Mapping[str, Any]) -> None:
    from .analysis import fault_name, fault_severity

    meta = result.meta
    wall = float(meta.get("wall_s") or 0.0)
    raw_spec = meta.get("spec")
    spec: Mapping[str, Any] = raw_spec if isinstance(raw_spec, Mapping) else {}
    simulated = result.duration_s if result.n_samples else float(spec.get("duration_s", 0.0))  # kpis_only records no samples
    rtf = f" ({simulated / wall:.3g}× real time)" if wall > 0 else ""
    print(f"{meta.get('scenario')}  {meta.get('fidelity')}/{meta.get('coil')}  {simulated:.3f} s simulated in {wall:.2f} s{rtf}")
    faults = [f"{fault_name(e.a)} ({fault_severity(e.a)}) at {e.t_s:.3f} s" for e in result.events if e.kind == "FaultRaised"]
    print(
        "  faults: " + ("; ".join(faults[:8]) + (f"; … {len(faults) - 8} more" if len(faults) > 8 else "") if faults else "none")
    )
    safety = analyses.get("safety")
    trips = getattr(safety, "trips", []) if safety is not None else []
    print("  monitor demands: " + ("; ".join(f"{n} at {t:.3f} s" for t, n in trips) if trips else "none"))
    for name, unit, scale in _SUMMARY_KPIS:
        if name in result.kpis:
            v = result.kpis[name] * scale
            print(f"  {name:28s} {_fmt(v, 5):>12s} {unit}")


# --------------------------------------------------------------------------- handlers


def _pending(wave: str, unit: str) -> Handler:
    """A handler for a subcommand whose body lands in ``wave`` (unit ``unit``)."""

    def handler(args: argparse.Namespace) -> int:
        print(f"fabrail {args.command}: not yet implemented ({wave}, unit {unit})", file=sys.stderr)
        return 2

    return handler


#: Rows of the ``presets`` table: (label, source, key, scale, unit).
_PRESET_ROWS: tuple[tuple[str, str, str, float, str], ...] = (
    ("design mass m*", "param", "design.carriage.m_star_kg", 1.0, "kg"),
    ("rated mass", "param", "design.carriage.m_rated_kg", 1.0, "kg"),
    ("rest policy", "param", "design.rest_policy", 1.0, ""),
    ("travel", "param", "design.rail.travel_m", 1.0, "m"),
    ("magnets", "param", "design.suspension.n_magnets", 1.0, ""),
    ("zero-power gap g*", "param", "design.suspension.g_star_m", 1e3, "mm"),
    ("open-loop pole p", "derived", "pole_rad_s", 1.0, "rad/s"),
    ("speed cap open", "derived", "v_open_cap_mps", 1.0, "m/s"),
    ("speed cap close", "derived", "v_close_cap_mps", 1.0, "m/s"),
    ("stop distance", "derived", "d_stop_m", 1e3, "mm"),
    ("terminus arrival (closed form)", "derived", "terminus_t_arrive_s", 1.0, "s"),
    ("clamp force, hand edge", "derived", "clamp_hand_edge_n", 1.0, "N"),
    ("lift current at rated mass", "derived", "lift_current_rated_a", 1.0, "A"),
    ("manual force after landing", "derived", "manual_force_relieved_n", 1.0, "N"),
    ("isolation T_min (research)", "derived", "t_min_iso_s", 1.0, "s"),
    ("hold-up energy available", "derived", "holdup_avail_j", 1.0, "J"),
    ("hold-up energy needed", "derived", "holdup_need_j", 1.0, "J"),
)


def _cmd_presets(args: argparse.Namespace) -> int:
    native = _native()
    classes = [args.rail_class] if args.rail_class else list(CLASSES)
    params = {c: native.Params.preset(c) for c in classes}
    derived = {c: {str(k): float(v) for k, v in p.derived().items()} for c, p in params.items()}
    table: dict[str, dict[str, object]] = {c: {} for c in classes}
    for label, source, key, scale, unit in _PRESET_ROWS:
        for c in classes:
            raw: object = params[c].get(key) if source == "param" else derived[c].get(key)
            if isinstance(raw, int | float) and not isinstance(raw, bool) and scale != 1.0:
                raw = float(raw) * scale
            table[c][f"{label} ({unit})" if unit else label] = raw
    width = max(len(k) for k in next(iter(table.values())))
    print(f"{'':{width}s}  " + "  ".join(f"{c.upper():>12s}" for c in classes))
    for row in next(iter(table.values())):
        print(f"{row:{width}s}  " + "  ".join(f"{_fmt(table[c][row], 5):>12s}" for c in classes))
    print("values from fabrail-params (presets and derive); PARAMETERS.md lists every field")
    if args.json_path:
        body = {c: {"table": table[c], "derived": derived[c]} for c in classes}
        doc = {"schema": "fabrail.presets/1", "fabrail_version": __version__, "classes": body}
        print(f"wrote {_write_doc(args.json_path, _seal(doc))}")
    return 0


def _cmd_params(args: argparse.Namespace) -> int:
    from .config import RunConfig, parameters_markdown

    if args.markdown:
        sys.stdout.write(parameters_markdown())
        return 0
    native = _native()
    if args.describe:
        rows = native.Params.describe()
        print(json.dumps(rows, indent=2, ensure_ascii=False, default=str))
        return 0
    cfg = RunConfig.from_args(args)
    text = cfg.params().to_json(indent=2)
    if args.json_path:
        Path(args.json_path).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json_path).write_text(text + "\n", encoding="utf-8")
        print(f"wrote {args.json_path}")
    else:
        print(text)
    return 0


def _cmd_scenarios(args: argparse.Namespace) -> int:
    from . import scenarios

    if not scenarios.CATALOG:
        print("fabrail scenarios: the scenario catalog is not yet implemented (Wave 1, unit A9)", file=sys.stderr)
        return 2
    rail_class = getattr(args, "rail_class", None)
    rows = []
    for name, sdef in scenarios.CATALOG.items():
        if rail_class and rail_class not in sdef.classes:
            continue
        research = " [research tier]" if sdef.requires else ""
        print(f"  {name:26s} {'/'.join(sdef.classes):24s} {sdef.tiers[0]:10s} {sdef.doc}{research}")
        rows.append(
            {
                "name": name,
                "classes": list(sdef.classes),
                "tiers": list(sdef.tiers),
                "options": list(sdef.options),
                "slow_classes": list(sdef.slow_classes),
                "requires": dict(sdef.requires),
                "doc": sdef.doc,
            }
        )
    if args.json_path:
        doc = {"schema": "fabrail.scenarios/1", "fabrail_version": __version__, "scenarios": rows}
        print(f"wrote {_write_doc(args.json_path, _seal(doc))}")
    return 0


def _simulate_one(
    args: argparse.Namespace,
    cfg: RunConfig,
    name: str,
    *,
    command: str,
    stem: str,
    fidelity: str | None = None,
    options: Mapping[str, Any] | None = None,
    mutate: Callable[[dict[str, Any]], None] | None = None,
    plot_kinds: Sequence[str] | None = None,
    extra_config: Mapping[str, Any] | None = None,
) -> tuple[RunResult, dict[str, Any], dict[str, Any], Path]:
    """Build, run, analyse, plot and record one scenario; returns (result, analyses, doc, json path)."""
    from . import scenarios
    from .analysis import analyze_run

    params = cfg.params()
    _requires_note(name, params)
    spec, series = scenarios.spec(name, params, cfg, fidelity=fidelity, options=options)
    if mutate is not None:
        mutate(spec)
    _set_channels(spec, params, args.channels)
    result = _run(params, _native_scenario(spec, series, params))
    analyses = analyze_run(result, max_points=args.max_points, meta={"seed": cfg.seed})
    kinds = list(plot_kinds) if plot_kinds is not None else ["levitation", "haptics", "safety"]
    if plot_kinds is None and _seismic_relevant(result, analyses):
        kinds.append("seismic")
    out_dir = Path(cfg.output)
    figures = _plot(analyses, kinds, out_dir, stem) if cfg.plots and result.n_samples > 0 and result.channels else []
    plain_options = {k: v for k, v in dict(options or {}).items() if k != "record"}  # a Record is described in extra_config
    config = _config_block(cfg, command, spec, scenario=name, options=plain_options, **dict(extra_config or {}))
    doc = _run_doc(result, analyses, config, figures, args.json_channels)
    path = Path(args.json_path) if args.json_path else out_dir / f"{stem}.json"
    _write_doc(path, doc, compact=True)
    _print_run_summary(result, analyses)
    for f in figures:
        print(f"  wrote {out_dir / f}")
    print(f"  wrote {path}")
    return result, analyses, doc, path


def _cmd_simulate(args: argparse.Namespace) -> int:
    from .config import RunConfig

    cfg = RunConfig.from_args(args)
    name = args.scenario
    _simulate_one(args, cfg, name, command="simulate", stem=f"{cfg.rail_class}_{name}", options=_options(args.option))
    return 0


def _cmd_compare(args: argparse.Namespace) -> int:
    from . import export, scenarios, visualization
    from .analysis import analyze_haptics, analyze_safety
    from .config import RunConfig

    base = RunConfig.from_args(args)
    name = args.scenario
    sdef = scenarios.CATALOG.get(name)
    if sdef is None:
        raise ValueError(f"unknown scenario {name!r} (see `python -m fabrail scenarios`)")
    wanted = [args.rail_class] if args.rail_class else [c.strip().lower() for c in args.classes.split(",") if c.strip()]
    for c in wanted:
        if c not in CLASSES:
            raise ValueError(f"--classes: unknown class {c!r} (expected {', '.join(CLASSES)})")
    classes = [c for c in wanted if c in sdef.classes]
    for c in wanted:
        if c not in sdef.classes:
            _warn(f"scenario {name!r} does not apply to {c} (classes: {', '.join(sdef.classes)}); skipped")
    if not classes:
        raise ValueError(f"scenario {name!r} applies to none of {', '.join(wanted)}")
    options = _options(args.option)
    jobs: list[tuple[Any, Any]] = []
    specs: dict[str, Any] = {}
    for c in classes:
        cfg = base.with_class(c)
        params = cfg.params()
        _requires_note(name, params)
        spec, series = scenarios.spec(name, params, cfg, options=options)
        _set_channels(spec, params, args.channels)
        jobs.append((params, _native_scenario(spec, series, params)))
        specs[c] = spec
    from .results import RunResult

    runs = {c: RunResult.from_native(r) for c, r in zip(classes, _native().run_batch(jobs, base.workers), strict=True)}
    haptics = {c: analyze_haptics(r, max_points=args.max_points, meta={"seed": base.seed}) for c, r in runs.items()}
    safety = {c: analyze_safety(r, max_points=args.max_points, meta={"seed": base.seed}) for c, r in runs.items()}
    out_dir = Path(base.output)
    figures = []
    if base.plots:
        fig_name = f"compare_{name}.png"
        visualization.plot_class_comparison(haptics, out_dir / fig_name)
        figures.append(fig_name)
    print(f"{'class':8s} {'tier':10s} {'KE max (J)':>11s} {'p max (kg·m/s)':>15s} {'overshoot (mm)':>15s} {'faults':s}")
    records: dict[str, Any] = {}
    timing: dict[str, Any] = {}
    for c, r in runs.items():
        faults = list(safety[c].faults)
        print(
            f"{c:8s} {specs[c]['fidelity']['model']:10s} {_fmt(r.kpis.get('ke_max_gov_j')):>11s} "
            f"{_fmt(r.kpis.get('momentum_max')):>15s} {_fmt(r.kpis.get('overshoot_m', math.nan) * 1e3):>15s} "
            f"{', '.join(faults) or 'none'}"
        )
        rec: dict[str, Any] = dict(export.run_record(r, include_channels=args.json_channels))
        timing[c] = rec.pop("timing", {})
        rec.pop("sha256", None)
        records[c] = rec
    doc = {
        "schema": "fabrail.compare/1",
        "fabrail_version": __version__,
        "scenario": name,
        "classes": classes,
        "config": {"seed": base.seed, "coil": base.coil, "overrides": dict(base.overrides), "options": options},
        "runs": records,
        "analysis": {"haptics": haptics, "safety": safety},
        "figures": figures,
    }
    path = Path(args.json_path) if args.json_path else out_dir / f"compare_{name}.json"
    _write_doc(path, _seal(doc, {"wall_s": timing}), compact=True)
    for f in figures:
        print(f"wrote {out_dir / f}")
    print(f"wrote {path}")
    return 0


def _cmd_ensemble(args: argparse.Namespace) -> int:
    from . import export, visualization
    from .analysis import analyze_ensemble
    from .config import RunConfig
    from .ensemble import run_ensemble

    defaults: dict[str, object] = {}
    if args.record_rate_hz is None:
        defaults["record_rate_hz"] = ENSEMBLE_RECORD_RATE_HZ
    cfg = RunConfig.from_args(args, **defaults)
    params = cfg.params()
    _requires_note(args.scenario, params)
    thresholds = {k: float(v) for k, v in _options(args.threshold).items()}  # type: ignore[arg-type]
    result = run_ensemble(params, args.scenario, cfg, args.n_members, options=_options(args.option) or None)
    ea = analyze_ensemble(result, thresholds)
    names = list(thresholds) + [k for k, *_ in _SUMMARY_KPIS if k in ea.kpis and k not in thresholds and ea.n_reporting.get(k, 0)]
    print(f"{cfg.rail_class}/{args.scenario}: {ea.n} members, seed {ea.seed}, tier {result.meta.get('fidelity')}")
    print(f"  {'KPI':28s} {'n':>4s} {'P50':>12s} {'P95':>12s}  exceedance")
    for k in names:
        exc = ""
        if k in thresholds:
            upper = _fmt(ea.wilson_upper.get(k))
            exc = f"{ea.exceedances.get(k, 0)}/{ea.n_reporting.get(k, 0)} > {thresholds[k]:g} (Wilson 95 % upper {upper})"
        print(f"  {k:28s} {ea.n_reporting.get(k, 0):4d} {_fmt(ea.p50.get(k), 5):>12s} {_fmt(ea.p95.get(k), 5):>12s}  {exc}")
    stem = f"{cfg.rail_class}_{args.scenario}_ensemble"
    out_dir = Path(cfg.output)
    figures = []
    if cfg.plots and ea.kpis:
        visualization.plot_ensemble(ea, out_dir / f"{stem}.png")
        figures.append(f"{stem}.png")
    doc: dict[str, Any] = dict(export.ensemble_record(result))
    timing = doc.pop("timing", {})
    doc["analysis"] = {"ensemble": export.to_jsonable(ea)}
    doc["figures"] = figures
    path = Path(args.json_path) if args.json_path else out_dir / f"{stem}.json"
    _write_doc(path, _seal(doc, timing), compact=True)
    for f in figures:
        print(f"wrote {out_dir / f}")
    print(f"wrote {path}")
    return 0


_SEISMIC_MODE_OVERRIDES: dict[str, dict[str, object]] = {
    "land": {},
    "isolate": {"control.seismic.isolation.enable": True, "control.seismic.policy.action": "isolate"},
    "damp": {"control.seismic.amd.enable": True, "control.seismic.policy.action": "damp"},
}


def _record_for(args: argparse.Namespace, cfg: RunConfig) -> Any:
    """The ground-motion record for ``--ground kt|cp|file``."""
    import numpy as np

    from . import ground, records
    from .ensemble import split_seed

    if args.ground == "file":
        if args.record_path is None:
            raise ValueError("--ground file needs --record FILE (CSV or PEER AT2)")
        path = Path(args.record_path)
        if path.suffix.lower() == ".at2":
            return records.load_at2(path)
        return records.load_csv(path, fs_hz=args.record_fs_hz, columns=args.record_columns, units=args.record_units)
    pga = 0.5 if args.pga_g is None else float(args.pga_g)
    rng = np.random.default_rng(split_seed(cfg.seed)[0])
    fs, dur = 200.0, float(args.record_duration_s)
    a = ground.synthesize(rng, dur, fs, soil=args.soil, pga_g=pga, envelope=None, model=args.ground)
    model = "Kanai-Tajimi" if args.ground == "kt" else "Clough-Penzien"
    return records.Record(
        f"{args.ground}_{args.soil}",
        fs,
        a,
        "synthetic",
        meta={"model": model, "soil": args.soil, "pga_g": pga, "duration_s": dur},
    )


def _cmd_seismic(args: argparse.Namespace) -> int:
    from . import scenarios
    from .config import RunConfig

    base = RunConfig.from_args(args)
    overrides = {**_SEISMIC_MODE_OVERRIDES[args.mode], **base.overrides}  # an explicit --set wins
    cfg = RunConfig.from_args(args, overrides=overrides)
    if args.mode != "land":
        _warn(f"--mode {args.mode} is research tier (isolation and AMD are not product features)")
    if args.mode == "damp":
        params = cfg.params()
        gains = [params.get("control.seismic.amd.c_dvf_n_s_per_m")] + [
            params.get(f"control.seismic.amd.k_lqr.{i}") for i in range(4)
        ]
        if not any(float(g) for g in gains):
            _warn("AMD gains are zero in every preset: set control.seismic.amd.* with --set to damp anything")
    cls = cfg.rail_class
    extra: dict[str, Any] = {"mode": args.mode, "ground": args.ground}
    if args.ground == "pwave":
        if args.pga_g is not None:
            _warn("--pga is ignored with --ground pwave (the P+S synthesis sets the amplitude from M and R)")
        options: dict[str, Any] = {}
        for key, value in (("magnitude", args.magnitude), ("r_epi_km", args.distance_km), ("depth_km", args.depth_km)):
            if value is not None:
                options[key] = float(value)
        options.setdefault("magnitude", 7.0)
        options.setdefault("r_epi_km", 20.0)
        options.setdefault("depth_km", 10.0)
        if args.network_alert_delay_s is not None:
            name = "network_alert"
            options["latency_s"] = float(args.network_alert_delay_s)
        else:
            if args.cancel_at_s is not None:
                raise ValueError("--cancel-at needs --network-alert-delay (there is no alert to cancel)")
            name = "pwave_m7_r20"
        mutate = None
        if args.cancel_at_s is not None:
            if not args.cancel_at_s > args.network_alert_delay_s:
                raise ValueError("--cancel-at must be later than --network-alert-delay (both are seconds after the origin)")

            def mutate(spec: dict[str, Any]) -> None:
                origin = float(scenarios.info(spec)["origin_t_s"])
                spec["events"].append({"kind": "alert_cancel", "t_s": origin + float(args.cancel_at_s), "id": 1})
                spec["events"].sort(key=lambda e: float(e["t_s"]))

        m, r = options["magnitude"], options["r_epi_km"]
        stem = f"{cls}_seismic_pwave_m{m:g}_r{r:g}_{args.mode}"
        extra.update(options)
    else:
        for flag, value in (
            ("--magnitude", args.magnitude),
            ("--distance-km", args.distance_km),
            ("--depth-km", args.depth_km),
            ("--network-alert-delay", args.network_alert_delay_s),
            ("--cancel-at", args.cancel_at_s),
        ):
            if value is not None:
                raise ValueError(f"{flag} needs --ground pwave (a {args.ground} record has no source or origin time)")
        rec = _record_for(args, cfg)
        if getattr(rec.a_mps2, "ndim", 1) > 1:
            _warn("the record's x (travel) component drives the floor; y and z are not applied in this scenario")
        name = "elcentro_free"
        options = {"record": rec}

        def mutate(spec: dict[str, Any]) -> None:
            # A controlled device under the record: drop the SEI-013 'manual' release so the
            # land-first path (or the research-tier mode) acts; the name and doc say so.
            spec["events"] = [e for e in spec["events"] if not (e.get("kind") == "command" and e.get("cmd") == "manual")]
            spec["name"] = f"{cls}_seismic_{args.ground}"
            head, sep, tail = str(spec.get("doc", "")).partition(" | info: ")
            spec["doc"] = (
                f"seismic CLI: controlled {cls} under a {args.ground} record ({rec.name}), mode {args.mode}"
                + (sep + tail if sep else "")
            )[:4096]

        stem = f"{cls}_seismic_{args.ground}_{args.mode}"
        extra.update({"record": rec.name, "record_source": rec.source, "record_sha256": rec.sha256, "record_fs_hz": rec.fs_hz})
    result, analyses, _doc, _path = _simulate_one(
        args,
        cfg,
        name,
        command="seismic",
        stem=stem,
        options=options,
        mutate=mutate,
        plot_kinds=["seismic", "safety"],
        extra_config=extra,
    )
    sei = analyses["seismic"]
    print("  timeline (alerts are synthetic/injected, never a live EEW feed):")
    marks = [(sei.t_p_s, "t_P (synthesis)"), (sei.t_s_arrival_s, "t_S (synthesis)")] + list(sei.timeline)
    for t, label in sorted((m for m in marks if math.isfinite(m[0])), key=lambda m: m[0]):
        print(f"    {t:9.3f} s  {label}")
    if math.isfinite(sei.land_t_s) and math.isfinite(sei.t_s_arrival_s):
        print(f"  touchdown at t_S {sei.land_t_s - sei.t_s_arrival_s:+.3f} s (negative: before the synthetic S arrival)")
    return 0


def _cmd_detect(args: argparse.Namespace) -> int:
    import numpy as np

    from . import records, visualization
    from .analysis import analyze_detection
    from .config import RunConfig

    cfg = RunConfig.from_args(args)
    path = Path(args.input_path)
    if not path.is_file():
        raise FileNotFoundError(f"--input: no such file {path}")
    if path.suffix.lower() == ".at2":
        rec = records.load_at2(path)
    else:
        rec = records.load_csv(path, fs_hz=args.fs_hz, columns=args.columns, units=args.units)
    a = np.asarray(rec.a_mps2, dtype=np.float64)
    if a.ndim == 1:
        axis = str(rec.meta.get("axis", "x"))
        full = np.zeros((3, a.shape[0]))
        full["xyz".index(axis)] = a
        a = full
    params = cfg.params()
    replay = _native().DetectorReplay(params, float(rec.fs_hz))
    chunk = max(1, int(round(600.0 * rec.fs_hz)))  # 10-minute chunks (plan §2b item 13)
    events: list[tuple[float, str, float, float]] = []
    for i in range(0, a.shape[1], chunk):
        part = np.ascontiguousarray(a[:, i : i + chunk], dtype="<f8")
        events.extend(replay.feed(part[0].tobytes(), part[1].tobytes(), part[2].tobytes()))
    report = {str(k): float(v) for k, v in replay.report().items()}
    span = a.shape[1] / rec.fs_hz
    print(f"{path.name}: {a.shape[1]} samples at {rec.fs_hz:g} Hz ({span:.1f} s), detector of the {cfg.rail_class} preset")
    for t, kind, x, y in events:
        print(f"  {t:10.3f} s  {kind:14s} a={x:.4g} b={y:.4g}")
    print("  report: " + ", ".join(f"{k} {_fmt(v)}" for k, v in sorted(report.items())))
    sei = analyze_detection(
        0.0,
        rec.fs_hz,
        a,
        events,
        report,
        rail_class=cfg.rail_class,
        name=path.stem,
        max_points=args.max_points,
        meta={"seed": cfg.seed, "version": __version__},
    )
    out_dir = Path(cfg.output)
    figures = []
    if cfg.plots and a.shape[1] > 0:
        name = f"detect_{path.stem}.png"
        visualization.plot_seismic(sei, out_dir / name)
        figures.append(name)
        print(f"wrote {out_dir / name}")
    doc = {
        "schema": "fabrail.detect/1",
        "fabrail_version": __version__,
        "input": {
            "file": path.name,
            "sha256": rec.sha256,
            "fs_hz": rec.fs_hz,
            "n_samples": int(a.shape[1]),
            "columns": args.columns,
            "units": args.units,
            "format": rec.meta.get("format"),
        },
        "rail_class": cfg.rail_class,
        "events": [{"t_s": t, "kind": k, "a": x, "b": y} for t, k, x, y in events],
        "report": report,
        "analysis": {"seismic": sei},
        "figures": figures,
    }
    json_path = Path(args.json_path) if args.json_path else out_dir / f"detect_{path.stem}.json"
    print(f"wrote {_write_doc(json_path, _seal(doc), compact=True)}")
    return 0


def _cmd_study(args: argparse.Namespace) -> int:
    from .config import RunConfig
    from .studies import discover, run_study

    studies = discover()
    if args.list_studies:
        if not studies:
            print("no studies registered yet (Wave 2, units B2/B3)")
        for name, spec in studies.items():
            print(f"  {name:22s} tier {spec.tier}  {spec.question}")
        return 0
    name = args.name or "all"
    if name == "all":
        names = [n for n, s in studies.items() if args.tier is None or s.tier == args.tier]
    elif name in studies:
        names = [name]
    else:
        known = ", ".join(studies) or "none registered yet"
        print(f"fabrail study: unknown study {name!r} (available: {known})", file=sys.stderr)
        return 1
    cfg = RunConfig.from_args(args)
    for n in names:
        paths = run_study(n, cfg)
        print(f"  {n}: " + ", ".join(str(p) for p in paths))
    print(f"completed {len(names)} studies")
    return 0


def _anchors_dest(args: argparse.Namespace) -> Path:
    from .trace import find_root

    return Path(args.dest) if args.dest else find_root() / "tests" / "data" / "anchors_seed42.json"


def _cmd_anchors(args: argparse.Namespace) -> int:
    from . import export
    from .analysis import compute_anchors

    values, tols, missing = compute_anchors(quick=args.quick, workers=args.workers, progress=lambda m: print(f"  {m}"))
    doc = export.anchors_record(values, tols)
    for name in sorted(values):
        v, t = values[name], tols[name]
        print(f"  {name:52s} {_fmt(v, 10) if not isinstance(v, list) else str(v):>22s} {t.get('unit', '')}")
    if missing:
        _warn("not measured in these runs (phase absent; nothing invented): " + ", ".join(missing))
    status = 0
    if args.check:
        ref_path = Path(args.check) if args.check != "-" else _anchors_dest(args)
        failures = export.check_anchors(export.load_json(ref_path), values)
        for f in failures:
            print(f"  FAIL {f}")
        print(f"checked {len(values)} anchors against {ref_path}: {len(failures)} outside tolerance")
        status = 1 if failures else 0
    if args.json_path:
        print(f"wrote {export.write_json(args.json_path, doc)}")
    if args.write_tests:
        print(f"wrote {export.write_json(_anchors_dest(args), doc)}")
    return status


def _cmd_trace(args: argparse.Namespace) -> int:
    from . import trace

    return trace.cli(args)


_SIL_SMOKE: tuple[tuple[str, str, float | None], ...] = (
    ("door", "hover", 1.0),
    ("drawer", "unseat", None),
    ("wall", "land", None),
)


def _cmd_selftest(args: argparse.Namespace) -> int:
    from . import export
    from .analysis import fault_name, fault_severity
    from .config import RunConfig

    native = _native()
    h = int(native.selftest_hash(args.ticks))
    golden = int(native.GOLDEN_SELFTEST_HASH)
    ok = True
    if golden == 0:
        verdict = "not frozen yet (GOLDEN_SELFTEST_HASH is the placeholder 0; C1 freezes it)"
    elif args.ticks != 2000:
        verdict = f"not compared ({args.ticks} ticks; the golden hash is defined for 2000)"
    elif h == golden:
        verdict = "matches GOLDEN_SELFTEST_HASH"
    else:
        verdict, ok = f"MISMATCH against GOLDEN_SELFTEST_HASH {golden:#018x}", False
    print(f"selftest hash ({args.ticks} ticks): {h:#018x}  {verdict}")
    cases = _SIL_SMOKE[:1] if args.quick else _SIL_SMOKE
    smoke = []
    for cls, name, dur in cases:
        cfg = RunConfig(rail_class=cls, seed=args.seed, coil="voltage", duration_s=dur, fidelity="carriage6")
        params = cfg.params()
        from . import scenarios

        spec, series = scenarios.spec(name, params, cfg, fidelity="carriage6")
        scen = _native_scenario(spec, series, params)
        r1 = _run(params, scen)
        bad = [
            f"{fault_name(e.a)} ({fault_severity(e.a)})"
            for e in r1.events
            if e.kind == "FaultRaised" and fault_severity(e.a) in ("SafeStop", "SafeLand", "Sto")
        ]
        demand_max = int(r1["demand"].max()) if "demand" in r1 and r1.n_samples else 0
        entry: dict[str, Any] = {
            "case": f"{cls}/{name}",
            "fidelity": "carriage6",
            "coil": "voltage",
            "duration_s": r1.duration_s,
            "faults": bad,
            "demand_max": demand_max,
            "run_sha256": export.run_record(r1)["sha256"],
        }
        if cls == "door":
            r2 = _run(params, _native_scenario(spec, series, params))
            entry["repeat_identical"] = export.run_record(r2)["sha256"] == entry["run_sha256"]
        passed = bool(not bad and demand_max == 0 and entry.get("repeat_identical", True))
        entry["pass"] = passed
        ok &= passed
        smoke.append(entry)
        rep = "" if "repeat_identical" not in entry else f", repeat {'bit-identical' if entry['repeat_identical'] else 'DIFFERS'}"
        print(
            f"  SIL smoke {cls}/{name} (Carriage6, voltage, {r1.duration_s:.3f} s): "
            f"{'PASS' if passed else 'FAIL'}; faults ≥ SafeStop: {', '.join(bad) or 'none'}; monitor demand max {demand_max}{rep}"
        )
    print("SIL ≡ firmware is claimed in voltage coil mode only; this is simulation, not hardware verification")
    if args.json_path:
        doc = {
            "schema": "fabrail.selftest/1",
            "fabrail_version": __version__,
            "ticks": args.ticks,
            "hash": h,
            "golden": golden,
            "verdict": verdict,
            "build_profile": getattr(native, "BUILD_PROFILE", "?"),
            "smoke": smoke,
            "ok": ok,
        }
        print(f"wrote {_write_doc(args.json_path, _seal(doc))}")
    return 0 if ok else 1


def _cmd_data(args: argparse.Namespace) -> int:
    from . import records

    if args.action != "fetch" or args.dataset != "elcentro":
        raise ValueError(f"unsupported: data {args.action} {args.dataset}")
    if not (args.url or os.environ.get(records.ELCENTRO_URL_ENV)):
        raise ValueError(
            "no source URL: records are fetched, never vendored. Obtain the 1940 El Centro record file from "
            f"{records.ELCENTRO_SOURCE_PAGE} (CESMD data policy: attribution required) and pass --url URL "
            f"or set {records.ELCENTRO_URL_ENV} (file:// works for a local copy)"
        )
    path = records.fetch_elcentro(args.dest, url=args.url, force=args.force)
    pins = (Path(args.dest) / records.PIN_FILE).read_text(encoding="utf-8").strip().splitlines()
    pin = next((line.split()[0] for line in pins if line.strip().endswith(path.name)), "?")
    print(f"El Centro record: {path} (SHA-256 {pin}, pinned on first use in {Path(args.dest) / records.PIN_FILE})")
    print(f"provenance: {Path(args.dest) / 'elcentro.json'}; cite the provider per its data policy (never vendored)")
    return 0


# --------------------------------------------------------------------------- parser


def _add_common(p: argparse.ArgumentParser) -> None:
    """The common flags of core §B.9."""
    g = p.add_argument_group("common options")
    g.add_argument(
        "-c",
        "--class",
        dest="rail_class",
        choices=CLASSES,
        default=None,
        help="rail class (default: door; `compare` and `presets` default to all)",
    )
    g.add_argument("-s", "--seed", type=_nonneg_int, default=42, help="master seed (default: 42)")
    g.add_argument(
        "--set",
        action="append",
        default=None,
        metavar="PATH=VAL",
        help="parameter override by dotted path, e.g. control.motion.a_max_mps2=0.5 (repeatable)",
    )
    g.add_argument(
        "--plant",
        action="append",
        default=None,
        metavar="KEY=VAL",
        help="plant truth mismatch or environment key, e.g. mass_scale=1.1 (repeatable)",
    )
    g.add_argument("-o", "--output", type=Path, default=Path("output"), help="output directory (default: ./output)")
    g.add_argument("--no-plots", action="store_true", help="skip figures")
    g.add_argument("--json", dest="json_path", type=Path, default=None, metavar="PATH", help="write the result as JSON to PATH")
    g.add_argument(
        "-w",
        "--workers",
        type=_positive_int,
        default=None,
        help="worker threads for batches (default: all cores; results do not depend on it)",
    )
    g.add_argument(
        "--duration",
        dest="duration_s",
        type=_positive_float,
        default=None,
        metavar="S",
        help="scenario duration, s (default: the scenario's)",
    )
    g.add_argument("--fidelity", choices=FIDELITIES, default=None, help="plant tier (default: the scenario's)")
    g.add_argument(
        "--coil", choices=COILS, default=None, help="coil mode (default: lag; SIL ≡ firmware is claimed in voltage mode only)"
    )
    g.add_argument(
        "--record-rate",
        dest="record_rate_hz",
        type=_positive_float,
        default=None,
        metavar="HZ",
        help="recording rate, Hz (default: 1000; ensemble: 50)",
    )


def _add_run_output(p: argparse.ArgumentParser) -> None:
    """Flags shared by the subcommands that run and record scenarios."""
    p.add_argument(
        "--option",
        action="append",
        default=None,
        metavar="KEY=VAL",
        help="scenario builder option, e.g. mass_kg=48 for unseat (repeatable; see `scenarios --json`)",
    )
    p.add_argument(
        "--channels",
        choices=("analysis", "default"),
        default="analysis",
        help="record the default channels plus the analysis extras (default) or the scenario's own set",
    )
    p.add_argument("--json-channels", action="store_true", help="also store every recorded channel sample in the JSON")
    p.add_argument(
        "--max-points",
        type=_nonneg_int,
        default=4000,
        metavar="N",
        help="plotted samples per series (0 = all; peaks are always kept; default 4000)",
    )


def _register_hook(sub: argparse._SubParsersAction, module: str, name: str, help_text: str, wave: str, unit: str) -> None:
    """Let ``module.register(sub)`` add the ``name`` subcommand; fall back to a placeholder."""
    problem: str | None = None
    try:
        register = importlib.import_module(module).register
    except (ImportError, AttributeError) as exc:
        register, problem = None, f"{module}.register unavailable ({exc})"
    if register is not None:
        try:
            register(sub)
            return
        except Exception as exc:  # noqa: BLE001 - a broken hook must not break the whole CLI
            problem = f"{module}.register failed: {exc!r}"
    parser = sub.choices.get(name) if hasattr(sub, "choices") else None
    if parser is None:
        parser = sub.add_parser(name, help=help_text, description=help_text)

    def handler(args: argparse.Namespace) -> int:
        print(f"fabrail {name}: not yet implemented ({wave}, unit {unit}); {problem}", file=sys.stderr)
        return 2

    parser.set_defaults(handler=handler)


def make_parser() -> argparse.ArgumentParser:
    """Build the full argument parser (every subcommand, both hooks)."""
    parser = argparse.ArgumentParser(
        prog="fabrail",
        description=(
            "Magnetically suspended, magnetically propelled rails: simulator, control stack, "
            "haptics, seismic early warning and traceability tools."
        ),
    )
    parser.add_argument("--version", action="version", version=f"fabrail {__version__}")
    sub = parser.add_subparsers(dest="command", metavar="COMMAND")

    common = argparse.ArgumentParser(add_help=False)
    _add_common(common)

    def add(name: str, help_text: str, handler: Handler, *, with_common: bool = True) -> argparse.ArgumentParser:
        p = sub.add_parser(name, help=help_text, description=help_text, parents=[common] if with_common else [])
        p.set_defaults(handler=handler)
        return p

    add("presets", "class table with derived quantities (all classes unless -c)", _cmd_presets)

    p = add("params", "print the resolved parameter set, --describe the fields, or --markdown the reference", _cmd_params)
    p.add_argument(
        "--describe", action="store_true", help="field metadata (path, unit, min, max, source, per-class default) as JSON"
    )
    p.add_argument("--markdown", action="store_true", help="write the docs/PARAMETERS.md content to stdout")

    add("scenarios", "scenario catalog with one-line docs (classes, recommended tier)", _cmd_scenarios)

    p = add("simulate", "run one scenario and write figures and a fabrail.run/1 JSON record", _cmd_simulate)
    p.add_argument("--scenario", required=True, metavar="NAME", help="scenario name (see `fabrail scenarios`)")
    _add_run_output(p)

    p = add("compare", "run the same scenario across classes", _cmd_compare)
    p.add_argument("--scenario", required=True, metavar="NAME", help="scenario name")
    p.add_argument(
        "--classes",
        default=",".join(CLASSES),
        metavar="LIST",
        help=f"comma-separated classes (default: {','.join(CLASSES)}; -c picks one)",
    )
    _add_run_output(p)

    p = add("ensemble", "Monte Carlo ensemble of one scenario (P50/P95, exceedances, Wilson 95 %% upper bound)", _cmd_ensemble)
    p.add_argument("--scenario", required=True, metavar="NAME", help="scenario name")
    p.add_argument(
        "-n", "--members", dest="n_members", type=_positive_int, default=20, metavar="N", help="ensemble size (default: 20)"
    )
    p.add_argument(
        "--threshold",
        action="append",
        default=None,
        metavar="KPI=VAL",
        help="count members whose KPI exceeds VAL (repeatable), e.g. bumper_energy_j=281.5",
    )
    p.add_argument("--option", action="append", default=None, metavar="KEY=VAL", help="scenario builder option (repeatable)")

    p = add("seismic", "seismic run: land-first, isolation or AMD against synthetic or recorded ground motion", _cmd_seismic)
    p.add_argument(
        "--mode",
        choices=("land", "isolate", "damp"),
        default="land",
        help="land (default), isolate (research tier, WALL/HEAVY) or damp (research tier)",
    )
    p.add_argument(
        "--ground",
        choices=("kt", "cp", "pwave", "file"),
        default="pwave",
        help="Kanai-Tajimi, Clough-Penzien, P+S synthesis (default) or a record file",
    )
    p.add_argument(
        "--pga",
        dest="pga_g",
        type=_positive_float,
        default=None,
        metavar="G",
        help="peak ground acceleration, g (kt/cp; default 0.5)",
    )
    p.add_argument(
        "--soil",
        choices=("soft", "medium", "firm"),
        default="medium",
        help="site class of the kt/cp filter (fabrail.ground.SOILS; default: medium)",
    )
    p.add_argument(
        "--record-duration",
        dest="record_duration_s",
        type=_positive_float,
        default=40.0,
        metavar="S",
        help="kt/cp record length, s (default: 40)",
    )
    p.add_argument("--magnitude", type=_positive_float, default=None, metavar="M", help="moment magnitude (pwave; default 7)")
    p.add_argument(
        "--distance-km",
        dest="distance_km",
        type=_positive_float,
        default=None,
        metavar="R",
        help="epicentral distance, km (pwave; default 20)",
    )
    p.add_argument(
        "--depth-km", dest="depth_km", type=_positive_float, default=None, metavar="D", help="focal depth, km (pwave; default 10)"
    )
    p.add_argument(
        "--network-alert-delay",
        dest="network_alert_delay_s",
        type=_finite_float,
        default=None,
        metavar="S",
        help="inject a synthetic network alert S seconds after the origin (never a live EEW feed)",
    )
    p.add_argument(
        "--cancel-at",
        dest="cancel_at_s",
        type=_finite_float,
        default=None,
        metavar="S",
        help="cancel the synthetic alert S seconds after the origin",
    )
    p.add_argument(
        "--record",
        dest="record_path",
        type=Path,
        default=None,
        metavar="FILE",
        help="ground-motion record (CSV or PEER AT2) for --ground file",
    )
    p.add_argument(
        "--record-fs",
        dest="record_fs_hz",
        type=_positive_float,
        default=None,
        metavar="HZ",
        help="record sample rate, Hz (CSV without a time column)",
    )
    p.add_argument("--record-columns", default="x", metavar="LIST", help="CSV data columns as axes (default: x)")
    p.add_argument("--record-units", default="g", metavar="UNIT", help="CSV units: g, m/s2 or gal (default: g)")
    _add_run_output(p)

    p = add("detect", "replay an accelerometer record through the seismic detector", _cmd_detect)
    p.add_argument(
        "--input",
        dest="input_path",
        type=Path,
        required=True,
        metavar="FILE",
        help="three-axis acceleration record (CSV, optional time column; or PEER AT2)",
    )
    p.add_argument(
        "--fs",
        dest="fs_hz",
        type=_positive_float,
        default=None,
        metavar="HZ",
        help="sample rate, Hz (required without a time column; the ADXL355 rate is 500)",
    )
    p.add_argument("--columns", default="x,y,z", metavar="LIST", help="file data-column order as axes (default: x,y,z)")
    p.add_argument("--units", default="g", metavar="UNIT", help="units of the data columns: g, m/s2 or gal (default: g)")
    p.add_argument("--max-points", type=_nonneg_int, default=4000, metavar="N", help="plotted samples per series (default 4000)")

    p = add("study", "exploratory studies (output/studies/<name>.{png,json})", _cmd_study)
    p.add_argument("name", nargs="?", default=None, help="study name, or 'all' (default)")
    p.add_argument("--list", dest="list_studies", action="store_true", help="list studies and exit")
    p.add_argument("--tier", type=int, choices=(1, 2), default=None, help="with 'all': only this tier")

    p = add("anchors", "compute the seed-42 acceptance anchors (core §E.5; fabrail.anchors/1)", _cmd_anchors)
    p.add_argument("--write-tests", action="store_true", help="write tests/data/anchors_seed42.json (or --dest)")
    p.add_argument("--dest", type=Path, default=None, metavar="PATH", help="anchors file for --write-tests/--check")
    p.add_argument("--quick", action="store_true", help="only the few-second cases (DOOR push_open, DRAWER unseat/payload, hash)")
    p.add_argument(
        "--check",
        nargs="?",
        const="-",
        default=None,
        metavar="PATH",
        help="compare with a recorded anchors file (default: --dest or tests/data/anchors_seed42.json)",
    )

    p = add("trace", "requirement ⇄ test traceability guard (FAB-SW-007)", _cmd_trace, with_common=False)
    from .trace import add_arguments as _trace_arguments

    _trace_arguments(p)

    p = add("selftest", "golden selftest hash and a SIL smoke run (Carriage6, voltage coil mode)", _cmd_selftest)
    p.add_argument("--ticks", type=_positive_int, default=2000, metavar="N", help="selftest length in base ticks (default: 2000)")
    p.add_argument("--quick", action="store_true", help="smoke-run only the DOOR hover")

    p = add("data", "fetch external data (never vendored)", _cmd_data, with_common=False)
    p.add_argument("action", choices=("fetch",), help="what to do")
    p.add_argument("dataset", choices=("elcentro",), help="which dataset")
    p.add_argument("--dest", type=Path, default=Path("data"), help="destination directory (default: ./data)")
    p.add_argument(
        "--url", default=None, metavar="URL", help="record file URL (or FABRAIL_ELCENTRO_URL; file:// works for a local copy)"
    )
    p.add_argument("--force", action="store_true", help="download again even when a pinned copy exists")

    _register_hook(
        sub, "fabrail.exotic.cli", "exo", "Sovereign/Exotic tier: classes, scenarios, simulations and studies", "Wave 2", "X7"
    )
    _register_hook(sub, "fabrail.gallery.cli", "example", "example gallery: list and run the named examples", "Wave 2b", "E5")
    return parser


def main(argv: list[str] | None = None) -> int:
    """Parse ``argv`` (default: ``sys.argv[1:]``), run the handler and return its exit status."""
    os.environ.setdefault("MPLBACKEND", "Agg")
    parser = make_parser()
    args = parser.parse_args(argv)
    handler: Handler | None = getattr(args, "handler", None)
    if args.command is None or handler is None:
        parser.print_help()
        return 0
    try:
        return int(handler(args) or 0)
    except NotYetImplementedError as exc:
        print(f"fabrail {args.command}: {exc}", file=sys.stderr)
        return 2
    except (ImportError, ValueError, KeyError, OSError, RuntimeError) as exc:
        if os.environ.get("FABRAIL_DEBUG") == "1":
            raise
        text = exc.args[0] if isinstance(exc, KeyError) and exc.args else exc
        print(f"fabrail {args.command}: {text}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
