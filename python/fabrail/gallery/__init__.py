"""Example gallery: the 16 named, runnable examples (plan §4b; docs/SPECIFICATION_EXAMPLES.md).

``python -m fabrail example list | run NAME [--quick]``. Each example is a thin script
with an exact filename in ``examples/`` that writes to ``examples/out/<name>/``. The
registry (:mod:`fabrail.gallery.registry`) holds one :class:`ExampleSpec` per example;
the badge is the core status (✅ V, 🔶 S, 🧪 R, 🗺️ M) with 🔭 overriding when maturity is
frontier, and no badge exceeds the weakest requirement the example demonstrates
(FAB-EXA-002). Every alert shown is labelled "synthetic/injected, not a live EEW feed".

Owner: W0-C2 (stub), then E5 (Wave 2b); E3/E4 add the example modules.
"""

from __future__ import annotations

from .registry import BADGES, EXAMPLE_NAMES, REGISTRY, ExampleSpec, examples, get, register

__all__ = ["BADGES", "EXAMPLE_NAMES", "REGISTRY", "ExampleSpec", "examples", "get", "register"]
