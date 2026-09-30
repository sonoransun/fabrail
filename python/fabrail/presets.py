"""Class presets: thin factories over the Rust ``fabrail-params`` presets (core §B.8).

``fabrail-params`` (no_std) is the single source of every preset for the simulator,
Python, the docs and the firmware; nothing here restates a parameter value. Each
factory returns a native ``fabrail._native.Params``::

    from fabrail import presets
    door = presets.sliding_door()
    heavy_door = presets.sliding_door(**{"design.carriage.m_star_kg": 120.0})
    heavy_door = presets.sliding_door(design__carriage__m_star_kg=120.0)  # same

Override keys are the dotted parameter paths of ``Params.describe()`` (also used
by ``--set PATH=VAL`` and docs/PARAMETERS.md). In keyword form ``__`` stands for
``.``. Overrides go through ``Params.with_overrides``, which re-validates and
raises ``ValueError("path: reason")``.

:func:`all_presets` and :func:`derived_table` feed the ``presets`` CLI table and the
generated docs/PARAMETERS.md (``Params.derived()``, the ``fabrail_params::derive``
table).

Owner: W0-C2 (stub), then A8 (Wave 1).
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ._native import Params

__all__ = [
    "CLASSES",
    "FACTORIES",
    "all_presets",
    "by_class",
    "cabinet_drawer",
    "class_name",
    "derived_table",
    "infrastructure_panel",
    "partition_wall",
    "sliding_door",
]

#: Rail classes in their canonical order (``fabrail_params::presets::CLASSES``).
CLASSES: tuple[str, ...] = ("drawer", "door", "wall", "heavy")

#: Factory function name per class.
FACTORIES: Mapping[str, str] = {
    "drawer": "cabinet_drawer",
    "door": "sliding_door",
    "wall": "partition_wall",
    "heavy": "infrastructure_panel",
}


def class_name(rail_class: str) -> str:
    """Normalise and check a class name (case-insensitive); raises ``ValueError``."""
    name = str(rail_class).strip().lower()
    if name not in CLASSES:
        raise ValueError(f"unknown rail class {rail_class!r}; expected one of {', '.join(CLASSES)}")
    return name


def _normalise_overrides(overrides: Mapping[str, object]) -> dict[str, object]:
    return {str(key).replace("__", "."): value for key, value in overrides.items()}


def by_class(rail_class: str, **overrides: object) -> Params:
    """The preset for ``rail_class`` ("drawer", "door", "wall" or "heavy") with overrides."""
    from . import require_native

    params = require_native().Params.preset(class_name(rail_class))
    if overrides:
        params = params.with_overrides(_normalise_overrides(overrides))
    return params


def cabinet_drawer(**overrides: object) -> Params:
    """DRAWER: 8–48 kg captive drawer, rests on its SEAT (core §C.2)."""
    return by_class("drawer", **overrides)


def sliding_door(**overrides: object) -> Params:
    """DOOR: 100 kg top-hung sliding door, rests hovering (core §C.2)."""
    return by_class("door", **overrides)


def partition_wall(**overrides: object) -> Params:
    """WALL: 500 kg top-hung partition wall, rests landed (core §C.2)."""
    return by_class("wall", **overrides)


def infrastructure_panel(**overrides: object) -> Params:
    """HEAVY: 5 t movable infrastructure panel, rests landed and locked (core §C.2)."""
    return by_class("heavy", **overrides)


def all_presets(**overrides: object) -> dict[str, Params]:
    """Every class preset in :data:`CLASSES` order, each with the same ``overrides``."""
    return {c: by_class(c, **overrides) for c in CLASSES}


def derived_table(classes: Iterable[str] = CLASSES) -> dict[str, dict[str, float]]:
    """``{class: Params.derived()}`` for ``classes``: the named derived quantities of each
    preset (SI, unit-suffixed names such as ``pole_rad_s``), in class order."""
    return {name: dict(by_class(name).derived()) for name in (class_name(c) for c in classes)}
