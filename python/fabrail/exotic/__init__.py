"""Sovereign/Exotic tier: Python layer (docs/EXOTIC.md; plan §4).

Classes beyond the four pedestrian rail classes (SENTINEL, HAVEN-R, HAVEN-F, SHIELD,
BASTION, ARENA, ROTATOR, TIDEGATE, SELENE). The Rust models live in
``fabrail-exo-safety`` (integer safety logic), ``fabrail-exo`` (f32 controllers) and
``fabrail-exo-sim`` (f64 truth), exposed as flat ``exo_*`` functions in
``fabrail._native``; this package gives them a Python face:

- :mod:`~fabrail.exotic.presets`: the class presets as typed SI views (EXOTIC §1, §S1);
  the pedestrian caps are imported from ``fabrail-params``, never restated (FAB-EXO-114);
- :mod:`~fabrail.exotic.classes`: the single admission function and its generated
  tables, HTS area, kinematic budgets, the isolation demand, ISO 13855 distances, the
  attended jog, bumper ratings and the stray-field shroud flag (EXOTIC §X2, §X3, §X7,
  §X9, §X14);
- :mod:`~fabrail.exotic.scenarios`: the XS/XC catalogue over ``exo_simulate`` (EXOTIC §X17);
- :mod:`~fabrail.exotic.pfh`: 2oo3 PFH in both forms with a CTMC cross-check (EXOTIC §X13);
- :mod:`~fabrail.exotic.eew`: lead-time gain, sensor advance, the offshore case and
  fusion demos (EXOTIC §X10);
- :mod:`~fabrail.exotic.energy`: the ride-through calculator and storage benchmarks
  (EXOTIC §X4, §X5);
- :mod:`~fabrail.exotic.pressure`: air-blast closed forms, the closure race and tsunami
  timing (EXOTIC §X11, §X12), with no yield-to-distance law anywhere (FAB-EXO-808);
- :mod:`~fabrail.exotic.plots` and :mod:`~fabrail.exotic.studies`: figures and the
  exotic studies; :mod:`~fabrail.exotic.cli`: ``python -m fabrail exo …``.

Every exotic class declares a safety regime (GuardedZone or OccupiedPlatform); nothing
is certified; every alert is synthetic or injected. The 🔭 frontier what-if studies live
only in :mod:`fabrail.exotic.frontier` (numpy only; FAB-EXO-002).

This package imports nothing at load time; submodules load on first attribute access.

Owner: W0-C2 (stub), then X7 (Wave 2); ``frontier/`` is X8.
"""

from __future__ import annotations

import importlib
from types import ModuleType

__all__ = ["classes", "cli", "eew", "energy", "frontier", "pfh", "plots", "presets", "pressure", "scenarios", "studies"]


def __getattr__(name: str) -> ModuleType:
    if name in __all__:
        module = importlib.import_module(f"{__name__}.{name}")
        globals()[name] = module
        return module
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
