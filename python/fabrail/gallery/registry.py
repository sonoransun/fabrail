"""The example registry (plan §4b; FAB-EXA-001).

An example module registers itself with :func:`register`::

    from fabrail.gallery.registry import ExampleSpec, register

    def _run(out_dir, *, quick=False, **options):
        ...                       # writes a figure and a JSON record into out_dir
        return record             # JSON-ready dict

    register(ExampleSpec(
        name="kitchen_push_open", badge="✅", maturity="…",
        story="…", run=_run,
    ))

``run(out_dir: pathlib.Path, *, quick: bool = False, **options) -> dict`` is the
frozen calling convention; ``options`` carries example-specific flags such as the
villa ``variant`` and ``building``. Story lines are subject to the vocabulary guard
(FAB-EXA-003).

Owner: W0-C2 (stub), then E5 (Wave 2b).
"""

from __future__ import annotations

import re
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path

__all__ = ["BADGES", "EXAMPLE_NAMES", "OUT_DIR", "REGISTRY", "ExampleSpec", "examples", "get", "register"]

#: The 16 gallery examples, in gallery order (plan §4b table). Exactly these names.
EXAMPLE_NAMES: tuple[str, ...] = (
    "kitchen_push_open",
    "pantry_cabinet_slider",
    "office_door_obstruction",
    "ballroom_partition",
    "archive_heavy_shelving",
    "drawer_power_loss",
    "wall_pwave_m7",
    "villa_quake",
    "supertall_wind_day",
    "museum_haven_quake",
    "proton_shield_pwave",
    "vault_close_race",
    "stadium_tray_pod_failure",
    "tidegate_offshore_m8",
    "pegs_long_period_securing",
    "lunar_hatch_selene",
)

#: Allowed badges: the core status (V, S, R, M) with 🔭 for frontier maturity (SPECIFICATION §0.5).
BADGES: Mapping[str, str] = {"✅": "V", "🔶": "S", "🧪": "R", "🗺️": "M", "🔭": "frontier"}

#: Where examples write their outputs (``examples/out/<name>/``), relative to the repo root.
OUT_DIR = Path("examples") / "out"

_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


@dataclass(frozen=True)
class ExampleSpec:
    """One gallery example.

    Attributes:
        name: exact example name (one of :data:`EXAMPLE_NAMES`).
        badge: one of :data:`BADGES`; never above the weakest requirement shown.
        maturity: technology maturity (TRL or a short phrase), separate from the badge.
        story: the one-paragraph story line shown in the gallery.
        run: ``run(out_dir, *, quick=False, **options) -> dict`` (JSON record).
    """

    name: str
    badge: str
    maturity: str
    story: str
    run: Callable[..., Mapping[str, object]]

    def __post_init__(self) -> None:
        if not _NAME_RE.match(self.name):
            raise ValueError(f"example name {self.name!r} must be snake_case")
        if self.badge not in BADGES:
            raise ValueError(f"{self.name}: badge {self.badge!r} is not one of {', '.join(BADGES)}")
        if not callable(self.run):
            raise ValueError(f"{self.name}: run must be callable")


#: Registered examples by name (filled by the example modules).
REGISTRY: dict[str, ExampleSpec] = {}


def register(spec: ExampleSpec) -> ExampleSpec:
    """Register ``spec``; its name must be one of :data:`EXAMPLE_NAMES` and not yet registered."""
    if spec.name not in EXAMPLE_NAMES:
        raise ValueError(f"{spec.name!r} is not a gallery example (plan §4b lists exactly {len(EXAMPLE_NAMES)})")
    if spec.name in REGISTRY:
        raise ValueError(f"example {spec.name!r} is already registered")
    REGISTRY[spec.name] = spec
    return spec


def get(name: str) -> ExampleSpec:
    """The registered example ``name``; ``KeyError`` names the registered ones."""
    try:
        return REGISTRY[name]
    except KeyError:
        known = ", ".join(n for n in EXAMPLE_NAMES if n in REGISTRY) or "none registered yet"
        raise KeyError(f"unknown example {name!r} (registered: {known})") from None


def examples() -> list[ExampleSpec]:
    """Registered examples in gallery order."""
    return [REGISTRY[n] for n in EXAMPLE_NAMES if n in REGISTRY]
