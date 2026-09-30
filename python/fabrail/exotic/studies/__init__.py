"""Exotic-tier studies, run through the core study runner (:mod:`fabrail.studies.runner`).

Each module defines a module-level ``STUDY`` (:class:`fabrail.studies.runner.StudySpec`)
and ``QUICK_AWARE = True`` (it honours :func:`fabrail.studies.runner.quick`), exactly
like a core study; :func:`discover` finds them and ``python -m fabrail exo study NAME``
runs them, writing ``<output>/studies/<name>.{png,json}`` (``fabrail.study/1``) through
:func:`fabrail.studies.runner.write_result`. Every plotted number is in the record
(FAB-SW-009 rule).

Sovereign studies (tier 1): ``sentinel_wind``, ``haven_isolation``, ``hts_force_gap``,
``ride_through``, ``pod_realloc``, ``eew_sovereign``. The ``frontier_*`` studies present
the 🔭 models of :mod:`fabrail.exotic.frontier` with the banner, ``"tier": "frontier"``
and the watermark (FAB-EXO-004); their physics stays in the frontier package.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...config import RunConfig
    from ...studies.runner import StudySpec

__all__ = ["STUDY_MODULES", "discover", "run_study"]

#: The sovereign study modules (each has a ``STUDY``).
STUDY_MODULES: tuple[str, ...] = (
    "sentinel_wind",
    "haven_isolation",
    "hts_force_gap",
    "ride_through",
    "pod_realloc",
    "eew_sovereign",
)


def discover() -> dict[str, StudySpec]:
    """Every exotic and frontier study, keyed by name, ordered by (tier, name)."""
    from . import frontier_studies

    found: dict[str, StudySpec] = {}
    for name in STUDY_MODULES:
        spec = importlib.import_module(f"{__name__}.{name}").STUDY
        found[spec.name] = spec
    for spec in frontier_studies.STUDIES:
        if spec.name in found:
            raise ValueError(f"duplicate study name {spec.name!r}")
        found[spec.name] = spec
    return dict(sorted(found.items(), key=lambda kv: (kv[1].tier, kv[0])))


def run_study(name: str, cfg: RunConfig, *, quick: bool = False) -> list[Path]:
    """Run exotic study ``name`` and write ``<cfg.output>/studies/<name>.{png,json}``."""
    from ...studies.runner import execute, write_result

    studies = discover()
    if name not in studies:
        raise KeyError(f"unknown exotic study {name!r} (available: {', '.join(studies)})")
    spec = studies[name]
    return write_result(spec, execute(spec, cfg, quick=quick), cfg)
