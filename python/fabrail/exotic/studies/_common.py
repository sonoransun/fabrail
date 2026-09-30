"""Shared pieces of the exotic studies (private).

Owner: X7 (Wave 2).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...config import RunConfig

#: Caption sentence every sovereign study carries (EXOTIC §S5 non-claims, abridged).
NON_CLAIMS = (
    "Simulation and closed forms only: no hardware, no FEM, nothing certified; research-tier classes are "
    "not products, and every alert is synthetic/injected."
)


def meta(cfg: RunConfig, study: str, tiers: str = "exo reduced models") -> dict[str, Any]:
    """Run-stamp metadata of an exotic study figure."""
    from ... import __version__

    m: dict[str, Any] = {"version": __version__, "scenario": f"exo study {study}", "seed": cfg.seed, "fidelity": tiers}
    try:
        from .._util import native

        m["build_profile"] = getattr(native(), "BUILD_PROFILE", None)
    except ImportError:  # pragma: no cover - studies need the extension anyway
        pass
    return m


def arr(values: Any, digits: int | None = None) -> list[float]:
    """``values`` as a flat list of floats (non-finite stay NaN; export converts them to null)."""
    a = np.asarray(values, dtype=float).ravel()
    if digits is not None:
        a = np.round(a, digits)
    return [float(x) for x in a]


def decimate(t: np.ndarray, *series: np.ndarray, max_points: int = 1500) -> tuple[np.ndarray, ...]:
    """Every ``step``-th sample so at most ``max_points`` remain (the same step for every series)."""
    step = max(1, int(np.ceil(t.size / max_points)))
    return (t[::step],) + tuple(s[::step] for s in series)


def check(ok: bool, value: Any, limit: Any, source: str, **extra: Any) -> dict[str, Any]:
    """A uniform record check (pass flag, value, limit, source)."""
    out: dict[str, Any] = {"pass": bool(ok), "value": value, "limit": limit, "source": source}
    out.update(extra)
    return out
