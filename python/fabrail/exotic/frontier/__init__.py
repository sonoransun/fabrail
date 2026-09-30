"""🔭 Frontier tier: theoretical what-if studies, never product claims (docs/FRONTIER.md).

Frontier code lives only here and uses numpy only (FAB-EXO-002). Every output
(stdout, JSON, plot watermark) carries the 🔭 banner, ``"tier": "frontier"`` and the
orders of magnitude still missing (FAB-EXO-004, FAB-FRN-008; FRONTIER §F1). This
module provides that banner; the models are submodules, loaded on first access:

- ``pegs``: on-site gravity gradiometer against the P wave (§F4, FAB-FRN-003);
- ``rtsc``: ambient-pressure RTSC suspension sweep (§F2, FAB-FRN-001);
- ``trapped_field``: trapped-field bulk pressure caps, 🧪 (§F3, FAB-FRN-002);
- ``metabarrier``: active metabarrier Γ, gap edge and ρλ³ mass law (§F5, FAB-FRN-004);
- ``qzs``: quasi-zero-stiffness series singularity and tilt error (§F6, FAB-FRN-005);
- ``lunar``: SELENE gravity, radiator and dust scaling (§F7, FAB-FRN-006);
- ``cloak``: magnetic cloak μ and the concentrator gain cap (§F8, FAB-FRN-007);
- ``diamagnetic``: diamagnetic pressure and the FLOAT check (§F9, FAB-FRN-009);
- ``quantum``, ``das_rail``, ``safe_rl``: the documented-only items as closed-form
  calculators (NV gap noise and gradiometer presets, DAS moveout, a CBF safety filter
  around a learned haptic policy; §F10);
- ``meta``: the FRONTIER §1 matrix and the frontier.html cards, generated from the
  models (FAB-FRN-010).

Every public function of a model submodule returns a
:class:`~fabrail.exotic.frontier._record.FrontierRecord` (a stamped ``dict``;
``str(record)`` is the stdout form, ``record.to_json()`` the JSON form and
``record.watermark`` the plot-watermark text).

    >>> b = frontier_banner(5, "PEGS on-site gradiometer: needs 1e-15/√Hz, best demonstrated 1e-10/√Hz")
    >>> b.meta["tier"]
    'frontier'
    >>> record = b.stamp({"lead_time_gain_s": 0.0})
    >>> watermark_text(b)
    'FRONTIER what-if, not a product claim · missing 5 orders of magnitude'

Owner: W0-C2 (banner helper, complete), then X8 (Wave 1: models, records, meta).
"""

from __future__ import annotations

import importlib
import math
from collections.abc import Mapping
from types import ModuleType
from typing import NamedTuple, cast

__all__ = [
    "BANNER_HEAD",
    "Banner",
    "MODEL_MODULES",
    "TIER",
    "WATERMARK_HEAD",
    "format_missing_orders",
    "frontier_banner",
    "watermark_text",
]

#: The model submodules; every public function of each returns a stamped record
#: (FAB-EXO-004, FAB-FRN-008).
MODEL_MODULES: tuple[str, ...] = (
    "pegs",
    "rtsc",
    "trapped_field",
    "metabarrier",
    "qzs",
    "lunar",
    "cloak",
    "diamagnetic",
    "quantum",
    "das_rail",
    "safe_rl",
)

#: Head of the plot-watermark text: the banner's wording without the 🔭 glyph, which
#: matplotlib's bundled DejaVu fonts lack (Wave-0 note).
WATERMARK_HEAD = "FRONTIER what-if, not a product claim"

#: Value of the ``"tier"`` key in every frontier JSON record.
TIER = "frontier"

#: First sentence of every frontier banner.
BANNER_HEAD = "🔭 FRONTIER: a theoretical what-if study, not a product claim."


class Banner(NamedTuple):
    """A frontier banner: the text to print and the keys to merge into JSON records."""

    text: str
    meta: dict[str, object]

    def stamp(self, record: Mapping[str, object]) -> dict[str, object]:
        """A copy of ``record`` carrying ``tier``, ``banner``, ``missing_orders`` and ``note``."""
        return {**dict(record), **self.meta}


def format_missing_orders(missing_orders: float) -> str:
    """``5`` → ``"5 orders of magnitude"``; ``1`` → ``"1 order of magnitude"``; ``2.5`` → ``"2.5 orders of magnitude"``."""
    value = _check(missing_orders)
    unit = "order" if value == 1.0 else "orders"
    return f"{value:g} {unit} of magnitude"


def frontier_banner(missing_orders: float, note: str = "") -> Banner:
    """The standard frontier banner.

    Args:
        missing_orders: orders of magnitude between the requirement and the best
            demonstrated capability (for example 5 for a 1e-15/√Hz need against a
            1e-10/√Hz demonstration). Finite and >= 0.
        note: one line naming the item and the gap.

    Returns:
        ``Banner(text, meta)`` where ``meta`` is ``{"tier": "frontier", "banner": text,
        "missing_orders": missing_orders, "note": note}``.
    """
    value = _check(missing_orders)
    note = " ".join(str(note).split())
    text = f"{BANNER_HEAD} Missing: {format_missing_orders(value)}."
    if note:
        text = f"{text} {note}"
    meta: dict[str, object] = {"tier": TIER, "banner": text, "missing_orders": value, "note": note}
    return Banner(text, meta)


def watermark_text(source: Banner | Mapping[str, object] | float) -> str:
    """Plot-watermark text for a frontier output (FAB-EXO-004).

    Args:
        source: a :class:`Banner`, a stamped record (any mapping with
            ``"missing_orders"``) or the missing-orders figure itself.

    Returns:
        ``"FRONTIER what-if, not a product claim · missing 5 orders of magnitude"``
        (no emoji, so every matplotlib font renders it); pass it to
        ``fabrail.plotstyle.watermark(fig, text)``.
    """
    if isinstance(source, Banner):
        value = source.meta["missing_orders"]
    elif isinstance(source, Mapping):
        if "missing_orders" not in source:
            raise ValueError("watermark_text needs a stamped frontier record (no 'missing_orders' key)")
        value = source["missing_orders"]
    else:
        value = source
    return f"{WATERMARK_HEAD} · missing {format_missing_orders(cast(float, value))}"


def __getattr__(name: str) -> ModuleType:
    """Load a model submodule (or ``meta``) on first attribute access."""
    if name in MODEL_MODULES or name == "meta":
        return importlib.import_module(f"{__name__}.{name}")
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def _check(missing_orders: float) -> float:
    try:
        value = float(missing_orders)
    except (TypeError, ValueError):
        raise ValueError(f"missing_orders must be a number (got {missing_orders!r})") from None
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"missing_orders must be finite and >= 0 (got {missing_orders!r})")
    return value
