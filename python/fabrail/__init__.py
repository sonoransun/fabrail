"""fabrail: magnetically suspended, magnetically propelled rails.

Hybrid PM-biased EMS suspension with zero-power control and a long-stator LSM for
cabinet drawers, sliding doors, partition walls and heavy movable infrastructure,
with programmatic haptics, energy and momentum governance, and seismic early
warning (the README's "pressure wave" is the P-wave).

The physics, the control stack and the safety monitor are Rust (``crates/``); this
package owns scenarios, analysis, plots and the CLI (``python -m fabrail``). The
compiled core is the extension submodule ``fabrail._native`` (built with
``maturin develop --release``). It is imported lazily, so ``import fabrail``,
``fabrail.trace`` (requirement traceability), ``fabrail.plotstyle`` and the CLI's
``--help`` all work without it. Native names (``Params``, ``Scenario``, ``run``, …)
are forwarded on first access::

    import fabrail
    p = fabrail.Params.preset("door")      # imports fabrail._native here
    fabrail.native_available()             # False until the extension is built

Frames: x is travel along the rail (+x opens), y is cross-track ("lateral" in
code), z is up. "Zero friction" is a direction (μ_eq ≈ 1e-3 to 1e-2), never a
claim. docs/SPECIFICATION.md is authoritative.
"""

from __future__ import annotations

import importlib
from types import ModuleType
from typing import Any

from ._pending import NotYetImplementedError

__version__ = "0.1.0"

#: Pure-Python submodules, imported on first attribute access (``fabrail.trace``).
_SUBMODULES: frozenset[str] = frozenset(
    {
        "analysis",
        "analytic",
        "config",
        "ensemble",
        "exotic",
        "export",
        "gallery",
        "ground",
        "human",
        "plotstyle",
        "presets",
        "pwave",
        "rail",
        "records",
        "results",
        "scenarios",
        "studies",
        "trace",
        "visualization",
    }
)

#: Names forwarded from ``fabrail._native`` on first access (core design §B.7).
_NATIVE_EXPORTS: frozenset[str] = frozenset(
    {
        "BASE_TICK_HZ",
        "BUILD_PROFILE",
        "DetectorReplay",
        "GOLDEN_SELFTEST_HASH",
        "MAX_MAGNETS",
        "Params",
        "RailClass",
        "RunData",
        "SCENARIO_SCHEMA",
        "Scenario",
        "run",
        "run_batch",
        "selftest_hash",
        "stability_report",
        "stray_field",
    }
)

_NATIVE_HINT = (
    "fabrail._native (the compiled Rust core) is not available; build it with "
    "`maturin develop --release` in a virtualenv (see CLAUDE.md)"
)


def require_native() -> ModuleType:
    """Return the compiled ``fabrail._native`` module, importing it on first use.

    Raises:
        ImportError: the extension is not built (the message says how to build it).
    """
    try:
        return importlib.import_module(f"{__name__}._native")
    except ImportError as exc:
        raise ImportError(f"{_NATIVE_HINT} ({exc})") from exc


def native_available() -> bool:
    """True when ``fabrail._native`` imports."""
    try:
        require_native()
    except ImportError:
        return False
    return True


def __getattr__(name: str) -> Any:
    """Lazy access to ``_native``, the native re-exports and the submodules."""
    if name == "_native":
        return require_native()
    if name in _NATIVE_EXPORTS:
        value = getattr(require_native(), name)
        globals()[name] = value
        return value
    if name in _SUBMODULES:
        return importlib.import_module(f"{__name__}.{name}")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(globals()) | _SUBMODULES | _NATIVE_EXPORTS | {"_native"})


__all__ = [
    "NotYetImplementedError",
    "__version__",
    "native_available",
    "require_native",
    *sorted(_SUBMODULES),
]
