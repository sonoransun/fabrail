"""JSON export (core §B.8; FAB-SW-009): every plotted number is recorded and round-trips.

Schemas (the ``schema`` key of each document):

- ``fabrail.run/1``: one run. Keys ``schema``, ``fabrail_version``, ``scenario``,
  ``rate_hz``, ``n_samples``, ``meta`` (the Rust ``RunMeta`` without ``wall_s``),
  ``kpis``, ``events`` (``[{t_s, kind, a, b}]``), ``summary`` (per-channel min, max,
  mean, first, last), optionally ``channels`` (``names`` in recording order, ``units``,
  ``dtypes`` ``"f8"``/``"u1"``, ``t_s`` and ``data``), ``sha256`` and ``timing``
  (``wall_s``).
- ``fabrail.ensemble/1``: an ensemble. Keys ``schema``, ``fabrail_version``, ``seed``,
  ``scenario``, ``n_members``, ``noise_seeds``, ``meta``, ``params`` and ``spec`` (of
  member 0), ``kpis`` (``{name: [value or null per member]}``), ``summary``
  (``EnsembleResult.kpi_summary``), ``members`` (per member: index, noise_seed, meta
  without params/spec plus their hashes, events), ``sha256`` and ``timing``.
- ``fabrail.anchors/1``: acceptance anchors (tests/data/anchors_seed42.json). Keys
  ``schema``, ``fabrail_version``, ``anchors`` (``{name: {value, rel, abs, source,
  unit}}``) and ``sha256``.
- ``fabrail.study/1``: one study record (output/studies/<name>.json).

Canonical JSON: sorted keys, UTF-8, no NaN or infinity (non-finite floats become
``null``), numpy scalars and arrays converted to Python numbers and lists. Python
writes floats with ``repr``, so every float64 round-trips bit for bit.

Determinism: ``sha256`` is the SHA-256 of the compact canonical JSON of the document
without ``sha256``, ``timing`` and every ``wall_s`` key (:func:`stable_payload`). Two
runs with the same seed therefore hash identically, and ``wall_s`` never enters a
hashed document (plan §2b item 16; FAB-EXA "byte-identical JSON, wall_s excluded").

Plotted numbers (FAB-SW-009): :func:`plotted_arrays` extracts the data of a matplotlib
figure and :func:`missing_plotted` lists every plotted value that the JSON record does
not contain, so a figure and its record can be checked against each other.

Owner: W0-C2 (stub; the canonical-JSON helpers and ``study_record`` are complete),
then A8 (Wave 1).
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
from collections.abc import Iterable, Mapping, Sequence
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .config import RunConfig
    from .results import EnsembleResult, RunResult

__all__ = [
    "ANCHORS_SCHEMA",
    "ENSEMBLE_SCHEMA",
    "RUN_SCHEMA",
    "STUDY_SCHEMA",
    "VOLATILE_KEYS",
    "anchors_record",
    "check_anchors",
    "dumps",
    "ensemble_from_record",
    "ensemble_record",
    "load_json",
    "missing_plotted",
    "payload_sha256",
    "plotted_arrays",
    "recorded_numbers",
    "run_from_record",
    "run_record",
    "save_run_json",
    "stable_payload",
    "study_record",
    "to_jsonable",
    "write_json",
]

RUN_SCHEMA = "fabrail.run/1"
ENSEMBLE_SCHEMA = "fabrail.ensemble/1"
ANCHORS_SCHEMA = "fabrail.anchors/1"
STUDY_SCHEMA = "fabrail.study/1"

#: Keys never hashed: wall-clock timing anywhere in a document (plan §2b item 16).
VOLATILE_KEYS: frozenset[str] = frozenset({"wall_s"})

#: Top-level keys outside the hashed payload.
_UNHASHED_TOP: frozenset[str] = frozenset({"sha256", "timing"})

#: JSON dtype tags of channel data ("f8" float64, "u1" uint8 state codes).
_DTYPE_TAGS: Mapping[str, np.dtype] = {"f8": np.dtype(np.float64), "u1": np.dtype(np.uint8)}


# ---- canonical JSON ----------------------------------------------------------------------


def to_jsonable(obj: Any) -> Any:
    """Convert ``obj`` to plain JSON types (dicts with str keys, lists, str, int, float, bool, None).

    Handles dataclasses, mappings, sequences, sets (sorted), numpy scalars and arrays,
    ``Path`` and ``Enum``. Non-finite floats become None.
    """
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, int):
        return int(obj)
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, Path):
        return obj.as_posix()
    if isinstance(obj, Enum):
        return to_jsonable(obj.value)
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: to_jsonable(getattr(obj, f.name)) for f in dataclasses.fields(obj)}
    if isinstance(obj, Mapping):
        return {str(k): to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (set, frozenset)):
        return [to_jsonable(v) for v in sorted(obj, key=repr)]
    if isinstance(obj, (list, tuple)):
        return [to_jsonable(v) for v in obj]
    tolist = getattr(obj, "tolist", None)  # numpy arrays and scalars
    if callable(tolist):
        return to_jsonable(tolist())
    item = getattr(obj, "item", None)
    if callable(item):
        return to_jsonable(item())
    raise TypeError(f"cannot convert {type(obj).__name__} to JSON")


def dumps(obj: Any, indent: int | None = 2) -> str:
    """Canonical JSON text of ``obj`` (sorted keys, no NaN, UTF-8)."""
    return json.dumps(to_jsonable(obj), indent=indent, sort_keys=True, ensure_ascii=False, allow_nan=False)


def write_json(path: str | Path, obj: Any, indent: int | None = 2) -> Path:
    """Write canonical JSON to ``path`` (parents created) and return the path."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(dumps(obj, indent=indent) + "\n", encoding="utf-8")
    return out


def load_json(path: str | Path) -> Any:
    """Read a JSON document."""
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _strip_volatile(obj: Any) -> Any:
    if isinstance(obj, Mapping):
        return {k: _strip_volatile(v) for k, v in obj.items() if k not in VOLATILE_KEYS}
    if isinstance(obj, list):
        return [_strip_volatile(v) for v in obj]
    return obj


def stable_payload(doc: Mapping[str, Any]) -> dict[str, Any]:
    """``doc`` as plain JSON without ``sha256``, ``timing`` and every ``wall_s`` key."""
    plain = to_jsonable({k: v for k, v in doc.items() if k not in _UNHASHED_TOP})
    return _strip_volatile(plain)


def payload_sha256(doc: Mapping[str, Any]) -> str:
    """SHA-256 (hex) of the compact canonical JSON of :func:`stable_payload`."""
    text = dumps(stable_payload(doc), indent=None)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _seal(doc: dict[str, Any]) -> dict[str, Any]:
    doc["sha256"] = payload_sha256(doc)
    return doc


def _version() -> str:
    from . import __version__

    return __version__


def _check_schema(doc: Mapping[str, Any], schema: str) -> None:
    got = doc.get("schema") if isinstance(doc, Mapping) else None
    if got != schema:
        raise ValueError(f"schema: expected {schema!r} (got {got!r})")


# ---- fabrail.study/1 ---------------------------------------------------------------------


def study_record(
    name: str, tier: int, question: str, record: Mapping[str, object], cfg: RunConfig | None = None
) -> dict[str, object]:
    """A ``fabrail.study/1`` document (Wave-0 shape plus ``sha256``; keys are only ever added)."""
    doc: dict[str, object] = {
        "schema": STUDY_SCHEMA,
        "fabrail_version": _version(),
        "name": name,
        "tier": tier,
        "question": question,
        "record": to_jsonable(record),
    }
    if cfg is not None:
        doc["config"] = {
            "seed": cfg.seed,
            "rail_class": cfg.rail_class,
            "fidelity": cfg.fidelity,
            "coil": cfg.coil,
            "overrides": to_jsonable(cfg.overrides),
        }
    doc["sha256"] = payload_sha256(doc)
    return doc


# ---- fabrail.run/1 -----------------------------------------------------------------------


def _event_list(events: Iterable[Any]) -> list[dict[str, object]]:
    return [{"t_s": float(e.t_s), "kind": str(e.kind), "a": float(e.a), "b": float(e.b)} for e in events]


def _dtype_tag(values: np.ndarray) -> str:
    return "u1" if np.asarray(values).dtype == np.uint8 else "f8"


def run_record(result: RunResult, include_channels: bool = False) -> dict[str, object]:
    """A ``fabrail.run/1`` document for one run.

    ``wall_s`` moves from ``meta`` into the unhashed ``timing`` block; ``sha256`` covers
    everything else (:func:`payload_sha256`). With ``include_channels`` the full
    channel data and time base are recorded, so every plotted sample can be found in
    the document (FAB-SW-009); without it, the per-channel ``summary`` is.
    """
    meta = dict(result.meta)
    timing = {k: meta.pop(k) for k in sorted(VOLATILE_KEYS) if k in meta}
    doc: dict[str, Any] = {
        "schema": RUN_SCHEMA,
        "fabrail_version": _version(),
        "scenario": meta.get("scenario"),
        "rate_hz": float(result.rate_hz),
        "n_samples": int(result.n_samples),
        "meta": to_jsonable(meta),
        "kpis": {str(k): float(v) for k, v in result.kpis.items()},
        "events": _event_list(result.events),
        "summary": to_jsonable(result.channel_summary()),
    }
    if include_channels:
        names = list(result.channels)
        doc["channels"] = {
            "names": names,
            "units": {n: str(result.units.get(n, "")) for n in names},
            "dtypes": {n: _dtype_tag(result.channels[n]) for n in names},
            "t_s": to_jsonable(np.asarray(result.t_s, dtype=np.float64)),
            "data": {n: to_jsonable(np.asarray(result.channels[n])) for n in names},
        }
    doc = to_jsonable(doc)
    _seal(doc)
    doc["timing"] = to_jsonable(timing)
    return doc


def save_run_json(result: RunResult, path: str | Path, include_channels: bool = False) -> Path:
    """Write :func:`run_record` to ``path`` (canonical JSON) and return the path."""
    return write_json(path, run_record(result, include_channels=include_channels))


def _float_array(values: Sequence[Any] | None) -> np.ndarray:
    return np.array([math.nan if v is None else float(v) for v in (values or [])], dtype=np.float64)


def _num(value: Any) -> float:
    """A recorded number; ``null`` (a non-finite value on export) comes back as NaN."""
    return math.nan if value is None else float(value)


def _events_from(items: Iterable[Mapping[str, Any]]) -> list[Any]:
    from .results import RunEvent

    return [RunEvent(_num(e["t_s"]), str(e["kind"]), _num(e["a"]), _num(e["b"])) for e in items]


def run_from_record(doc: Mapping[str, Any]) -> RunResult:
    """Rebuild a :class:`~fabrail.results.RunResult` from a ``fabrail.run/1`` document.

    Channels come back only when the record included them (``null`` samples become
    NaN; ``"u1"`` channels come back as uint8). ``timing.wall_s`` returns to ``meta``.
    ``run_record(run_from_record(doc), include_channels=…)`` reproduces ``doc``.
    """
    from .results import RunResult

    _check_schema(doc, RUN_SCHEMA)
    meta = dict(doc.get("meta") or {})
    meta.update(dict(doc.get("timing") or {}))
    rate = float(doc["rate_hz"])
    n = int(doc["n_samples"])
    channels: dict[str, np.ndarray] = {}
    units: dict[str, str] = {}
    block = doc.get("channels")
    if block:
        for name in block["names"]:
            tag = block["dtypes"][name]
            if tag not in _DTYPE_TAGS:
                raise ValueError(f"channels.dtypes.{name}: unknown dtype tag {tag!r}")
            raw = block["data"][name]
            channels[name] = np.array(raw, dtype=np.uint8) if tag == "u1" else _float_array(raw)
            units[name] = str(block["units"].get(name, ""))
        t = _float_array(block.get("t_s"))
    else:
        t = np.arange(n, dtype=np.float64) / rate if rate > 0 else np.zeros(n)
    return RunResult(
        rate_hz=rate,
        t_s=t,
        channels=channels,
        units=units,
        events=_events_from(doc.get("events", [])),
        kpis={str(k): _num(v) for k, v in dict(doc.get("kpis") or {}).items()},
        meta=meta,
    )


# ---- fabrail.ensemble/1 ------------------------------------------------------------------

#: Member meta keys stored once at the top of an ensemble record (with per-member hashes).
_SHARED_META = ("params", "spec")


def _json_sha256(obj: Any) -> str:
    return hashlib.sha256(dumps(obj, indent=None).encode("utf-8")).hexdigest()


def ensemble_record(result: EnsembleResult, *, include_events: bool = True) -> dict[str, object]:
    """A ``fabrail.ensemble/1`` document.

    KPIs are stored per member (``null`` where a member lacks one) and summarised
    (``EnsembleResult.kpi_summary``). The parameter set and scenario spec of member 0
    are stored once; each member records the SHA-256 of its own, so a sweep over
    parameters stays traceable. Member ``wall_s`` values go to the unhashed ``timing``.
    """
    names = result.all_kpi_names()
    members: list[dict[str, Any]] = []
    wall: list[Any] = []
    for i, m in enumerate(result.members):
        meta = dict(m.meta)
        wall.append(meta.pop("wall_s", None))
        entry: dict[str, Any] = {
            "index": i,
            "noise_seed": int(result.noise_seeds[i]) if i < len(result.noise_seeds) else None,
            "meta": {k: v for k, v in meta.items() if k not in _SHARED_META},
            "shared_sha256": {k: _json_sha256(meta.get(k)) for k in _SHARED_META if k in meta},
        }
        if include_events:
            entry["events"] = _event_list(m.events)
        members.append(entry)
    first = result.members[0].meta if result.members else {}
    ens_meta = dict(result.meta)
    ens_timing = {k: ens_meta.pop(k) for k in sorted(VOLATILE_KEYS) if k in ens_meta}
    doc: dict[str, Any] = {
        "schema": ENSEMBLE_SCHEMA,
        "fabrail_version": _version(),
        "seed": int(result.seed),
        "scenario": result.scenario,
        "n_members": len(result.members),
        "noise_seeds": [int(s) for s in result.noise_seeds],
        "meta": ens_meta,
        "params": first.get("params"),
        "spec": first.get("spec"),
        "kpis": {n: [m.kpis.get(n) for m in result.members] for n in names},
        "summary": result.kpi_summary(names),
        "members": members,
    }
    doc = to_jsonable(doc)
    _seal(doc)
    doc["timing"] = to_jsonable({**ens_timing, "member_wall_s": wall})
    return doc


def ensemble_from_record(doc: Mapping[str, Any]) -> EnsembleResult:
    """Rebuild an :class:`~fabrail.results.EnsembleResult` (KPIs, events, meta) from a
    ``fabrail.ensemble/1`` document. Members carry no channels. A member whose recorded
    params/spec hash matches member 0's gets that document back in its ``meta``."""
    from .results import EnsembleResult, RunResult

    _check_schema(doc, ENSEMBLE_SCHEMA)
    timing = dict(doc.get("timing") or {})
    wall = list(timing.pop("member_wall_s", []) or [])
    shared = {k: doc.get(k) for k in _SHARED_META}
    shared_hash = {k: _json_sha256(v) for k, v in shared.items()}
    kpis = dict(doc.get("kpis") or {})
    members: list[RunResult] = []
    for i, entry in enumerate(doc.get("members", [])):
        meta = dict(entry.get("meta") or {})
        for k, h in dict(entry.get("shared_sha256") or {}).items():
            if h == shared_hash.get(k):
                meta[k] = shared[k]
        if i < len(wall) and wall[i] is not None:
            meta["wall_s"] = wall[i]
        member_kpis = {name: float(vals[i]) for name, vals in kpis.items() if i < len(vals) and vals[i] is not None}
        events = _events_from(entry.get("events", []))
        members.append(RunResult(rate_hz=0.0, t_s=np.zeros(0), events=events, kpis=member_kpis, meta=meta))
    ens_meta = dict(doc.get("meta") or {})
    ens_meta.update(timing)
    return EnsembleResult(
        seed=int(doc["seed"]),
        noise_seeds=[int(s) for s in doc.get("noise_seeds", [])],
        members=members,
        scenario=str(doc.get("scenario", "")),
        meta=ens_meta,
    )


# ---- fabrail.anchors/1 -------------------------------------------------------------------

_TOL_KEYS = ("rel", "abs", "source", "unit")


def anchors_record(values: Mapping[str, object], tolerances: Mapping[str, object]) -> dict[str, object]:
    """A ``fabrail.anchors/1`` document (values plus per-anchor tolerance and source).

    ``values``: anchor name → number (or a list of numbers). ``tolerances``: the same
    names → a relative tolerance (a number) or a mapping with ``rel`` and/or ``abs``
    and optional ``source`` and ``unit``. Every anchor needs a tolerance and a source
    is strongly expected ("re-record after touching RNG, plant or control; never
    invent numbers", core §E.5). Raises ``ValueError("<name>: …")``.
    """
    extra = sorted(set(tolerances) - set(values))
    if extra:
        raise ValueError(f"{extra[0]}: tolerance given for an anchor with no value")
    anchors: dict[str, dict[str, object]] = {}
    for name in sorted(values):
        if name not in tolerances:
            raise ValueError(f"{name}: every anchor needs a tolerance")
        tol = tolerances[name]
        spec: dict[str, object]
        if isinstance(tol, Mapping):
            unknown = sorted(set(tol) - set(_TOL_KEYS))
            if unknown:
                raise ValueError(f"{name}: unknown tolerance key {unknown[0]!r} (rel, abs, source, unit)")
            spec = {k: tol[k] for k in _TOL_KEYS if k in tol}
        elif isinstance(tol, (int, float)) and not isinstance(tol, bool):
            spec = {"rel": float(tol)}
        else:
            raise ValueError(f"{name}: tolerance must be a number (rel) or a mapping")
        for key in ("rel", "abs"):
            if key in spec:
                t = float(spec[key])  # type: ignore[arg-type]
                if not (math.isfinite(t) and t >= 0.0):
                    raise ValueError(f"{name}: {key} tolerance must be finite and >= 0")
                spec[key] = t
        if "rel" not in spec and "abs" not in spec:
            raise ValueError(f"{name}: tolerance needs rel and/or abs")
        value = to_jsonable(values[name])
        flat = value if isinstance(value, list) else [value]
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in flat):
            raise ValueError(f"{name}: anchor values must be finite numbers")
        anchors[name] = {"value": value, **spec}
    doc: dict[str, Any] = {"schema": ANCHORS_SCHEMA, "fabrail_version": _version(), "anchors": anchors}
    return _seal(to_jsonable(doc))


def check_anchors(doc: Mapping[str, Any], measured: Mapping[str, object]) -> list[str]:
    """Compare ``measured`` against a ``fabrail.anchors/1`` document.

    An anchor passes when abs(m − v) ≤ abs + rel·abs(v) element-wise (a missing tolerance
    counts as 0). Returns one message per failure (missing, shape mismatch or out of
    tolerance), each naming the anchor, the tolerance and the source; empty means pass.
    """
    _check_schema(doc, ANCHORS_SCHEMA)
    failures: list[str] = []
    for name, spec in sorted(dict(doc.get("anchors") or {}).items()):
        if name not in measured:
            failures.append(f"{name}: not measured")
            continue
        want = np.atleast_1d(np.asarray(spec["value"], dtype=np.float64))
        got = np.atleast_1d(np.asarray(to_jsonable(measured[name]), dtype=np.float64))
        if want.shape != got.shape:
            failures.append(f"{name}: shape {got.shape} != recorded {want.shape}")
            continue
        rel = float(spec.get("rel", 0.0))
        tol_abs = float(spec.get("abs", 0.0))
        bound = tol_abs + rel * np.abs(want)
        bad = ~(np.abs(got - want) <= bound)
        if bad.any():
            i = int(np.argmax(bad))
            failures.append(
                f"{name}: {got[i]!r} vs recorded {want[i]!r} (rel {rel:g}, abs {tol_abs:g}; source: {spec.get('source', '?')})"
            )
    return failures


# ---- plotted numbers (FAB-SW-009) -----------------------------------------------------------


def _numeric(values: Any) -> np.ndarray | None:
    try:
        arr = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    return arr.ravel()


def plotted_arrays(fig: Any) -> list[tuple[str, np.ndarray]]:
    """The data-coordinate numbers drawn in a matplotlib figure, as (label, values).

    Covered per axes: lines (x and y; for ``axhline``/``axvline`` only the data
    coordinate), scatter and other offset collections, line collections
    (``vlines``/``hlines``/error bars), filled polygons (``fill_between``), bar
    heights (widths for horizontal bars) and images. Tick labels, titles and text
    annotations are not data. Values that are not finite are dropped.
    """
    from matplotlib.collections import Collection, LineCollection, PolyCollection
    from matplotlib.container import BarContainer

    out: list[tuple[str, np.ndarray]] = []

    def add(label: str, values: Any) -> None:
        arr = _numeric(values)
        if arr is None:
            return
        arr = arr[np.isfinite(arr)]
        if arr.size:
            out.append((label, arr))

    for ai, ax in enumerate(fig.axes):
        x_only = ax.get_xaxis_transform()  # x in data, y in axes coordinates (axvline)
        y_only = ax.get_yaxis_transform()  # x in axes coordinates, y in data (axhline)
        for li, line in enumerate(ax.get_lines()):
            label = f"axes[{ai}].lines[{li}]({line.get_label()})"
            tr = line.get_transform()
            if tr is not y_only:
                add(label + ".x", line.get_xdata())
            if tr is not x_only:
                add(label + ".y", line.get_ydata())
        for ci, cont in enumerate(ax.containers):
            if isinstance(cont, BarContainer):
                vertical = getattr(cont, "orientation", "vertical") != "horizontal"
                sizes = [p.get_height() if vertical else p.get_width() for p in cont.patches]
                add(f"axes[{ai}].bars[{ci}]", sizes)
        for ci, coll in enumerate(ax.collections):
            if not isinstance(coll, Collection):
                continue
            label = f"axes[{ai}].collections[{ci}]({coll.get_label()})"
            if isinstance(coll, LineCollection):
                segs = coll.get_segments()
                if segs:
                    add(label, np.concatenate([np.asarray(s, dtype=np.float64).ravel() for s in segs]))
                continue
            if isinstance(coll, PolyCollection):
                verts = [p.vertices for p in coll.get_paths()]
                if verts:
                    add(label, np.concatenate([np.asarray(v, dtype=np.float64).ravel() for v in verts]))
                continue
            offsets = coll.get_offsets()
            if offsets is not None and len(offsets):
                add(label, offsets)
        for ii, img in enumerate(ax.get_images()):
            add(f"axes[{ai}].images[{ii}]", np.ma.filled(np.ma.asarray(img.get_array(), dtype=np.float64), np.nan))
    return out


def recorded_numbers(record: Any) -> np.ndarray:
    """Every finite number in a JSON-able record (bools excluded), sorted and unique."""
    found: list[float] = []

    def walk(obj: Any) -> None:
        if isinstance(obj, bool) or obj is None or isinstance(obj, str):
            return
        if isinstance(obj, (int, float)):
            if math.isfinite(obj):
                found.append(float(obj))
            return
        if isinstance(obj, Mapping):
            for v in obj.values():
                walk(v)
            return
        if isinstance(obj, (list, tuple)):
            for v in obj:
                walk(v)

    walk(to_jsonable(record))
    return np.unique(np.asarray(found, dtype=np.float64))


def missing_plotted(
    fig: Any,
    record: Any,
    *,
    scales: Sequence[float] = (1.0,),
    rtol: float = 1e-12,
    atol: float = 0.0,
    limit: int = 20,
) -> list[str]:
    """Plotted values of ``fig`` that ``record`` does not contain (empty = all recorded).

    A plotted value x is found when some recorded number r and some ``scales`` entry s
    (the plotting unit factor, e.g. 1e3 for m plotted as mm) satisfy
    abs(x/s − r) ≤ atol + rtol·abs(x/s). Returns up to ``limit`` messages
    ``"<label>: <value>"``.
    """
    numbers = recorded_numbers(record)
    missing: list[str] = []
    for label, values in plotted_arrays(fig):
        found = np.zeros(values.shape, dtype=bool)
        for s in scales:
            if s == 0:
                continue
            target = values / s
            if numbers.size == 0:
                continue
            idx = np.searchsorted(numbers, target)
            lo = numbers[np.clip(idx - 1, 0, numbers.size - 1)]
            hi = numbers[np.clip(idx, 0, numbers.size - 1)]
            tol = atol + rtol * np.abs(target)
            found |= (np.abs(target - lo) <= tol) | (np.abs(target - hi) <= tol)
        for v in values[~found]:
            if len(missing) >= limit:
                return missing
            missing.append(f"{label}: {float(v)!r}")
    return missing
