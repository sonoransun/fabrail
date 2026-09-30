"""Run configuration, ``--set`` parsing and the generated parameter reference (core §B.8).

- :class:`RunConfig` is what the CLI, the scenario builders, ensembles and studies
  share. Parameter values are never stored here: they come from the
  ``fabrail-params`` presets plus dotted-path ``overrides``; plant (truth) mismatch
  goes in ``plant``.
- :func:`parse_set` turns repeated ``--set PATH=VAL`` / ``--plant KEY=VAL`` flags
  into a dict. Values are parsed as JSON scalars; with ``like=`` (a ``Params``) each
  value is also coerced to the type of the parameter leaf it sets
  (:func:`coerce_value`), so ``--set control.levitation.roll_active=1`` and
  ``--set control.rates.fast_decim=2.0`` do what they say.
- :func:`plant_overrides` checks and coerces ``--plant`` keys against the ``plant``
  block of ``schemas/scenario-1.schema.json`` (the single source of those keys).
- :func:`parameters_markdown` renders docs/PARAMETERS.md from ``Params.describe()``.
  Its signature is frozen (plan §2b item 16); ``python -m fabrail params --markdown``
  prints it and ``tests/test_docs_drift.py::test_parameters_md_current`` compares.

Owner: W0-C2 (stub), then A8 (Wave 1).
"""

from __future__ import annotations

import argparse
import functools
import json
import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .presets import CLASSES, class_name

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ._native import Params

__all__ = [
    "COILS",
    "COIL_MODE_PATH",
    "FIDELITIES",
    "MAX_DURATION_S",
    "RunConfig",
    "coerce_overrides",
    "coerce_value",
    "parameters_markdown",
    "parse_set",
    "parse_value",
    "plant_keys",
    "plant_overrides",
]

#: Plant tiers (scenario ``fidelity.model``; core §C.1).
FIDELITIES: tuple[str, ...] = ("carriage6", "heave1", "travel1d", "structure")

#: Coil modes (scenario ``fidelity.coil``). SIL ≡ firmware is claimed in voltage mode only.
COILS: tuple[str, ...] = ("lag", "voltage")

#: Upper bound on a scenario's duration, s (``fabrail_sim::scenario::MAX_DURATION_S``).
MAX_DURATION_S = 86_400.0

#: The parameter that must agree with the scenario coil mode (plan §2b item 16).
COIL_MODE_PATH = "control.rates.coil_mode_voltage"

#: The scenario schema whose ``plant`` block defines the ``--plant`` keys.
_SCENARIO_SCHEMA_PATH = Path(__file__).resolve().parent / "schemas" / "scenario-1.schema.json"

_TRUE = frozenset({"true", "yes", "on", "1"})
_FALSE = frozenset({"false", "no", "off", "0"})


@dataclass
class RunConfig:
    """One run's configuration (core §B.8; CLI common flags of §B.9).

    Attributes:
        rail_class: "drawer", "door", "wall" or "heavy".
        seed: master seed; ``numpy.random.SeedSequence(seed)`` splits it into the
            synthesis generator and the Rust ``noise_seed`` (core §A.4 rule 5).
        duration_s: scenario duration override, s (None = the scenario's default).
        fidelity: plant tier (:data:`FIDELITIES`).
        coil: coil mode (:data:`COILS`). The scenario is the single source of the coil
            mode (plan §2b item 16): :meth:`param_overrides` sets
            ``control.rates.coil_mode_voltage`` to match, so a preset never disagrees.
        record_rate_hz: recording rate, Hz (20 kHz / rate must be a whole number of
            plant steps; the Rust scenario validator checks it).
        workers: worker threads for batches and ensembles (None = all cores).
            Results do not depend on it.
        output: output directory for figures and JSON.
        plots: write figures.
        overrides: dotted-path parameter overrides (``--set``).
        plant: plant truth mismatch and environment keys (``--plant``); checked against
            the scenario schema's ``plant`` block (:func:`plant_overrides`).
    """

    rail_class: str = "door"
    seed: int = 42
    duration_s: float | None = None
    fidelity: str = "carriage6"
    coil: str = "lag"
    record_rate_hz: float = 1000.0
    workers: int | None = None
    output: Path = Path("output")
    plots: bool = True
    overrides: dict[str, object] = field(default_factory=dict)
    plant: dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.rail_class = class_name(self.rail_class)
        if isinstance(self.seed, bool) or not isinstance(self.seed, int) or self.seed < 0:
            raise ValueError(f"seed: must be a non-negative integer (got {self.seed!r})")
        if self.duration_s is not None:
            d = float(self.duration_s)
            if not (math.isfinite(d) and 0.0 < d <= MAX_DURATION_S):
                raise ValueError(f"duration_s: must be in (0, {MAX_DURATION_S:g}] s (got {self.duration_s!r})")
            self.duration_s = d
        if self.fidelity not in FIDELITIES:
            raise ValueError(f"fidelity: expected one of {', '.join(FIDELITIES)} (got {self.fidelity!r})")
        if self.coil not in COILS:
            raise ValueError(f"coil: expected one of {', '.join(COILS)} (got {self.coil!r})")
        rate = float(self.record_rate_hz)
        if not (math.isfinite(rate) and rate > 0.0):
            raise ValueError(f"record_rate_hz: must be a positive number (got {self.record_rate_hz!r})")
        self.record_rate_hz = rate
        if self.workers is not None:
            if isinstance(self.workers, bool) or int(self.workers) != self.workers or int(self.workers) < 1:
                raise ValueError(f"workers: must be an integer >= 1 or None (got {self.workers!r})")
            self.workers = int(self.workers)
        self.output = Path(self.output)
        self.overrides = {str(k): v for k, v in dict(self.overrides).items()}
        for path in self.overrides:
            if not path or any(ch.isspace() for ch in path):
                raise ValueError(f"overrides: invalid parameter path {path!r}")
        self.plant = plant_overrides(self.plant)

    @classmethod
    def from_args(cls, args: argparse.Namespace, **defaults: object) -> RunConfig:
        """Build from the CLI common flags (``fabrail.__main__``).

        Flags left unset (None) keep the dataclass default or ``defaults``.
        """
        kwargs: dict[str, object] = dict(defaults)
        mapping = {
            "rail_class": "rail_class",
            "seed": "seed",
            "duration_s": "duration_s",
            "fidelity": "fidelity",
            "coil": "coil",
            "record_rate_hz": "record_rate_hz",
            "workers": "workers",
            "output": "output",
        }
        for attr, key in mapping.items():
            value = getattr(args, key, None)
            if value is not None:
                kwargs[attr] = value
        if getattr(args, "no_plots", False):
            kwargs["plots"] = False
        kwargs["overrides"] = {**dict(kwargs.get("overrides", {})), **parse_set(getattr(args, "set", None) or [])}
        kwargs["plant"] = {**dict(kwargs.get("plant", {})), **parse_set(getattr(args, "plant", None) or [])}
        return cls(**kwargs)  # type: ignore[arg-type]

    def with_class(self, rail_class: str) -> RunConfig:
        """A copy for another class (``compare``)."""
        return replace(self, rail_class=rail_class)

    def param_overrides(self) -> dict[str, object]:
        """``overrides`` plus the coil-mode flag that must match the scenario."""
        out = dict(self.overrides)
        out[COIL_MODE_PATH] = self.coil == "voltage"
        return out

    def params(self) -> Params:
        """The class preset with :meth:`param_overrides` applied (needs ``_native``).

        Each override is first coerced to the type of its leaf (:func:`coerce_value`);
        ``Params.with_overrides`` then reseals the safety block and validates, raising
        ``ValueError("path: reason")``.
        """
        from . import require_native

        base = require_native().Params.preset(self.rail_class)
        return base.with_overrides(coerce_overrides(base, self.param_overrides()))


# ---- --set / --plant parsing ----------------------------------------------------------


def parse_value(text: str) -> object:
    """Parse one ``--set`` value: JSON scalars first, then Python-style booleans, else the string.

    ``"0.5"`` → 0.5, ``"2"`` → 2, ``"true"``/``"True"`` → True, ``"null"`` → None,
    ``"drawer"`` → "drawer", ``'"2"'`` → "2" (quoted JSON string).
    """
    s = text.strip()
    lowered = s.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    if lowered in ("none", "null"):
        return None
    try:
        value = json.loads(s)
    except ValueError:
        return s
    if isinstance(value, (int, float, str, bool)) or value is None:
        return value
    return s


def parse_set(items: Iterable[str], *, like: Params | None = None) -> dict[str, object]:
    """Parse repeated ``PATH=VAL`` strings into ``{path: value}`` (last one wins).

    The value is split at the first ``=``, so values may contain ``=``. Raises
    ``ValueError("--set expects PATH=VAL …")`` on a missing ``=`` or an empty path.
    With ``like`` (a ``Params``), every value is coerced to the type of its leaf
    (:func:`coerce_overrides`), and an unknown path raises ``ValueError("path: …")``.
    """
    out: dict[str, object] = {}
    for item in items:
        text = str(item)
        path, sep, value = text.partition("=")
        path = path.strip()
        if not sep or not path:
            raise ValueError(f"--set expects PATH=VAL (got {text!r})")
        if any(ch.isspace() for ch in path):
            raise ValueError(f"--set path must not contain whitespace (got {path!r})")
        out[path] = parse_value(value)
    if like is not None:
        out = coerce_overrides(like, out)
    return out


def coerce_value(template: object, value: object, path: str = "value") -> object:
    """Coerce ``value`` to the JSON type of ``template`` (a leaf's current value).

    - bool leaf: bools; the integers 0/1; strings true/false, yes/no, on/off, 1/0.
    - integer leaf: integers; integral floats (2.0 → 2); integral numeric strings.
    - float leaf: any real number (not bool) or numeric string; must be finite.
    - enum (string) leaf: strings, normalised to lower case (serde snake_case names).

    Raises ``ValueError("path: expects …")`` otherwise; the native ``Params`` does the
    range and cross-field checks afterwards.
    """
    if isinstance(template, bool):
        if isinstance(value, bool):
            return value
        if isinstance(value, int) and value in (0, 1):
            return bool(value)
        if isinstance(value, float) and value in (0.0, 1.0):
            return bool(value)
        if isinstance(value, str) and value.strip().lower() in _TRUE | _FALSE:
            return value.strip().lower() in _TRUE
        raise ValueError(f"{path}: expects a bool (got {value!r})")
    if isinstance(template, int):
        number = _as_number(value, path, "an integer")
        if isinstance(number, float):
            if not (math.isfinite(number) and number.is_integer()):
                raise ValueError(f"{path}: expects an integer (got {value!r})")
            return int(number)
        return number
    if isinstance(template, float):
        number = float(_as_number(value, path, "a number"))
        if not math.isfinite(number):
            raise ValueError(f"{path}: must be finite (got {value!r})")
        return number
    if isinstance(template, str):
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{path}: expects an enum name (got {value!r})")
        return value.strip().lower()
    raise ValueError(f"{path}: not a scalar parameter")


def _as_number(value: object, path: str, what: str) -> int | float:
    if isinstance(value, bool) or value is None:
        raise ValueError(f"{path}: expects {what} (got {value!r})")
    if isinstance(value, (int, float)):
        return value
    if isinstance(value, str):
        try:
            parsed = json.loads(value.strip())
        except ValueError:
            parsed = None
        if isinstance(parsed, (int, float)) and not isinstance(parsed, bool):
            return parsed
    raise ValueError(f"{path}: expects {what} (got {value!r})")


def coerce_overrides(like: Params, overrides: Mapping[str, object]) -> dict[str, object]:
    """Coerce every ``{path: value}`` to the type of that leaf in ``like``.

    Paths are dotted ``Params.describe()`` paths or JSON pointers; an unknown path
    raises the native ``ValueError("path: unknown parameter")``.
    """
    return {str(path): coerce_value(like.get(str(path)), value, str(path)) for path, value in overrides.items()}


@functools.lru_cache(maxsize=1)
def _plant_schema() -> dict[str, Any]:
    doc = json.loads(_SCENARIO_SCHEMA_PATH.read_text(encoding="utf-8"))
    return dict(doc["$defs"]["plant"]["properties"])


def plant_keys() -> tuple[str, ...]:
    """The scalar ``--plant`` keys of the scenario schema's ``plant`` block, sorted.

    ``structure`` (an object) is set by the scenario builders, not from the CLI.
    """
    return tuple(sorted(k for k in _plant_schema() if k != "structure"))


def plant_overrides(plant: Mapping[str, object]) -> dict[str, object]:
    """Check and coerce ``--plant`` values against the scenario schema ``plant`` block.

    Numbers become floats (``null``/None keeps "the design value" where the schema
    allows it); enum keys (``rail_profile``) must be one of the schema's values. Bounds
    are left to the Rust scenario validator. Raises ``ValueError("plant.<key>: …")``.
    """
    props = _plant_schema()
    out: dict[str, object] = {}
    for raw_key, value in plant.items():
        key = str(raw_key)
        if key == "structure":
            out[key] = value
            continue
        spec = props.get(key)
        if spec is None:
            raise ValueError(f"plant.{key}: unknown plant key (expected one of {', '.join(plant_keys())})")
        if "enum" in spec:
            if value not in spec["enum"]:
                raise ValueError(f"plant.{key}: expected one of {spec['enum']!r} (got {value!r})")
            out[key] = value
            continue
        types = spec.get("type", "number")
        types = [types] if isinstance(types, str) else list(types)
        if value is None:
            if "null" not in types:
                raise ValueError(f"plant.{key}: must be a number, not null")
            out[key] = None
            continue
        number = float(_as_number(value, f"plant.{key}", "a number"))
        if not math.isfinite(number):
            raise ValueError(f"plant.{key}: must be finite (got {value!r})")
        out[key] = number
    return out


# ---- docs/PARAMETERS.md ------------------------------------------------------------------


def _fmt(value: object) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if math.isinf(value):
            return "∞" if value > 0 else "−∞"
        if math.isnan(value):
            return "—"
        return f"{value:.6g}"
    return str(value).replace("|", "\\|")


def _class_default(entry: Mapping[str, object], cls: str, index: int) -> object:
    defaults = entry.get("default_by_class")
    if isinstance(defaults, Mapping):
        return defaults.get(cls, defaults.get(cls.upper()))
    if isinstance(defaults, Sequence) and not isinstance(defaults, str) and index < len(defaults):
        return defaults[index]
    return None


def parameters_markdown(describe: Sequence[Mapping[str, object]] | None = None) -> str:
    """Render docs/PARAMETERS.md (frozen signature; plan §2b item 16).

    Args:
        describe: rows as returned by ``fabrail._native.Params.describe()`` (keys
            ``path``, ``unit``, ``min``, ``max``, ``source``, ``default_by_class``,
            the last as ``{class: value}`` or a list in :data:`CLASSES` order).
            None reads them from the native module; the class columns then show each
            preset's typed value (enum names, true/false, integers) and a closing
            section lists ``Params.derived()`` per class. Given rows, the output is a
            pure function of them (numbers only, no derived section).

    Returns:
        The whole Markdown document. It is deterministic (no timestamps), so the
        drift test can compare it byte for byte.
    """
    typed: dict[str, Params] | None = None
    derived: dict[str, dict[str, float]] | None = None
    if describe is None:
        from . import require_native

        native = require_native()
        describe = native.Params.describe()
        typed = {c: native.Params.preset(c) for c in CLASSES}
        derived = {c: dict(p.derived()) for c, p in typed.items()}
    rows = [dict(r) for r in describe]

    lines = [
        "# fabrail parameters",
        "",
        "<!-- GENERATED by `python -m fabrail params --markdown` from fabrail-params "
        "(Params.describe()); never edit by hand. -->",
        "",
        "Every parameter lives in the Rust `fabrail-params` crate, the single source for the "
        "simulator, Python, the docs and the firmware. Physical fields are f64 SI with a unit "
        "suffix. Paths are the dotted serde paths used by `--set PATH=VAL`, `Params.get/set` "
        "(also as a JSON pointer with `/`) and `Params.with_overrides`. Min and max are the ranges "
        "`Params.validate()` enforces (enums and flags as their wire value); the cross-field rules "
        "of `fabrail_params::validate` apply on top. Editing any safety-relevant field reseals the "
        "safety block (`crc::seal`), which is why `safety.crc` differs per class.",
        "",
        f"{len(rows)} parameters.",
    ]
    header = "| Parameter | Unit | Min | Max | " + " | ".join(c.upper() for c in CLASSES) + " | Source |"
    rule = "|---|---|---|---|" + "---|" * len(CLASSES) + "---|"

    current_top: str | None = None
    current_block: str | None = None
    for row in rows:
        path = str(row.get("path", ""))
        parts = path.split(".")
        top = parts[0]
        block = ".".join(parts[:2]) if len(parts) > 2 else top
        if top != current_top:
            lines += ["", f"## {top}"]
            current_top = top
            current_block = None
        if block != current_block:
            lines += ["", f"### `{block}`", "", header, rule]
            current_block = block
        if typed is not None:
            values = [typed[c].get(path) for c in CLASSES]
        else:
            values = [_class_default(row, c, i) for i, c in enumerate(CLASSES)]
        cells = [
            f"`{path}`",
            _fmt(row.get("unit")) if row.get("unit") else "",
            _fmt(row.get("min")),
            _fmt(row.get("max")),
            *(_fmt(v) for v in values),
            _fmt(row.get("source", "")),
        ]
        lines.append("| " + " | ".join(cells) + " |")

    if derived is not None:
        names = sorted(set().union(*(d.keys() for d in derived.values())))
        lines += [
            "",
            "## derived",
            "",
            "Closed forms of `fabrail_params::derive::derived` for each preset (`Params.derived()`), "
            "SI with unit-suffixed names. These are the numbers SPECIFICATION §2 quotes.",
            "",
            "| Quantity | " + " | ".join(c.upper() for c in CLASSES) + " |",
            "|---|" + "---|" * len(CLASSES),
        ]
        for name in names:
            cells = [f"`{name}`", *(_fmt(derived[c].get(name)) for c in CLASSES)]
            lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"
