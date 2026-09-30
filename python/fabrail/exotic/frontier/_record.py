"""Frontier output records (FAB-EXO-004, FAB-FRN-008; FRONTIER §F1).

Every public function of a frontier model module returns a :class:`FrontierRecord`:
a plain ``dict`` stamped by :func:`fabrail.exotic.frontier.frontier_banner` with
``tier``, ``banner``, ``missing_orders`` and ``note``, plus the item's FRONTIER §1
metadata (``item``, ``badge``, ``maturity``, ``anchor_tier``, ``requirement``), the
``model`` that produced it and the ``inputs`` it was computed from. Outputs sit at the
top level under their unit-suffixed names.

The three output forms of FAB-EXO-004:

- stdout: ``str(record)`` (or :meth:`FrontierRecord.emit`) starts with the 🔭 banner,
  which holds the missing-orders figure, and repeats the figure on its last line;
- JSON: :meth:`FrontierRecord.to_json` (canonical: sorted keys, NaN/inf → null) carries
  ``"tier": "frontier"``, the banner and ``missing_orders``;
- plot watermark: :attr:`FrontierRecord.watermark`, the banner without the 🔭 glyph
  (matplotlib's DejaVu fonts lack it; Wave-0 note), for ``fabrail.plotstyle.watermark``.

numpy only (FAB-EXO-002).

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import json
import math
import sys
from collections.abc import Mapping
from typing import IO, Any

import numpy as np

from . import format_missing_orders, frontier_banner, watermark_text
from ._items import ITEMS_BY_KEY

__all__ = ["RESERVED_KEYS", "FrontierRecord", "jsonable", "make_record", "orders"]

#: Keys a model output may not use (the stamp and the metadata own them).
RESERVED_KEYS = frozenset(
    {
        "tier",
        "banner",
        "missing_orders",
        "note",
        "item",
        "item_key",
        "badge",
        "maturity",
        "anchor_tier",
        "requirement",
        "model",
        "inputs",
    }
)


class FrontierRecord(dict):
    """A stamped frontier output. A ``dict``; ``str()`` renders the stdout form."""

    @property
    def outputs(self) -> dict[str, Any]:
        """The model outputs (every key that is neither metadata nor stamp)."""
        return {k: v for k, v in self.items() if k not in RESERVED_KEYS}

    @property
    def watermark(self) -> str:
        """The plot-watermark text: banner wording without the 🔭 glyph, with the missing orders."""
        return watermark_text(self)

    def render(self) -> str:
        """The stdout form: the 🔭 banner first, then item, inputs, outputs and the missing orders."""
        lines = [str(self.get("banner", ""))]
        lines.append(
            f"{self.get('item', '')} [{self.get('badge', '')} · {self.get('maturity', '')}; "
            f"anchor tier {self.get('anchor_tier', '—')}; {self.get('requirement', '—')}]"
        )
        lines.append(f"model: {self.get('model', '')}")
        inputs = self.get("inputs") or {}
        if inputs:
            lines.append("inputs: " + ", ".join(f"{k}={_fmt(v)}" for k, v in inputs.items()))
        for key, value in self.outputs.items():
            lines.append(f"  {key} = {_fmt(value)}")
        lines.append(
            f"missing: {format_missing_orders(float(self.get('missing_orders', 0.0)))} (tier {self.get('tier', '')})"
        )
        return "\n".join(lines)

    def __str__(self) -> str:
        return self.render()

    def emit(self, file: IO[str] | None = None) -> FrontierRecord:
        """Print :meth:`render` to ``file`` (default stdout) and return ``self``."""
        print(self.render(), file=file if file is not None else sys.stdout)
        return self

    def to_json(self, indent: int | None = 2) -> str:
        """Canonical JSON (sorted keys, UTF-8, non-finite floats as null)."""
        return json.dumps(jsonable(self), indent=indent, sort_keys=True, ensure_ascii=False, allow_nan=False)


def orders(ratio: float) -> float:
    """Missing orders of magnitude for a required/demonstrated ``ratio``: log10, floored at 0."""
    r = float(ratio)
    if not math.isfinite(r) or r <= 1.0:
        return 0.0
    return math.log10(r)


def make_record(
    key: str,
    model: str,
    missing_orders: float,
    note: str,
    inputs: Mapping[str, Any],
    outputs: Mapping[str, Any],
) -> FrontierRecord:
    """Build a stamped record for item ``key`` (see :data:`ITEMS_BY_KEY`).

    ``missing_orders`` is rounded to 0.1 for the banner (the figure is an estimate); the
    unrounded value stays available as the ``missing_orders_exact`` output.

    Raises:
        KeyError: unknown item key.
        ValueError: an output uses a reserved key, or ``missing_orders`` is negative or
            not finite.
    """
    item = ITEMS_BY_KEY[key]
    clash = RESERVED_KEYS.intersection(outputs)
    if clash:
        raise ValueError(f"frontier output uses reserved keys {sorted(clash)}")
    exact = float(missing_orders)
    banner = frontier_banner(round(exact, 1), note)
    rec = FrontierRecord()
    rec["item"] = item.item
    rec["item_key"] = item.key
    rec["badge"] = item.badge
    rec["maturity"] = item.maturity
    rec["anchor_tier"] = item.anchor_tier
    rec["requirement"] = item.requirement
    rec["model"] = model
    rec["inputs"] = dict(inputs)
    rec.update(outputs)
    rec["missing_orders_exact"] = exact
    rec.update(banner.meta)
    return rec


def jsonable(obj: Any) -> Any:
    """Plain JSON types: numpy arrays and scalars converted, non-finite floats → None."""
    if obj is None or isinstance(obj, (bool, str)):
        return obj
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    if isinstance(obj, int):
        return int(obj)
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, Mapping):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return jsonable(obj.tolist())
    if isinstance(obj, np.generic):
        return jsonable(obj.item())
    raise TypeError(f"cannot convert {type(obj).__name__} to JSON")


def _fmt(value: Any) -> str:
    """Compact text for one value in :meth:`FrontierRecord.render`."""
    if isinstance(value, (bool, np.bool_)):
        return str(bool(value))
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        return f"{float(value):.6g}"
    if isinstance(value, str):
        return value
    if isinstance(value, np.ndarray):
        if value.size <= 8:
            if value.dtype.kind in "fc":
                return np.array2string(value, precision=4, separator=", ")
            return np.array2string(value, separator=", ")
        if value.dtype.kind in "fiu":
            finite = value[np.isfinite(value)] if value.dtype.kind == "f" else value
            if finite.size:
                return f"array{value.shape} [{float(finite.min()):.4g} … {float(finite.max()):.4g}]"
        return f"array{value.shape}"
    if isinstance(value, Mapping):
        return "{" + ", ".join(f"{k}: {_fmt(v)}" for k, v in value.items()) + "}"
    if isinstance(value, (list, tuple)):
        if len(value) > 8:
            return f"[{len(value)} items]"
        return "[" + ", ".join(_fmt(v) for v in value) + "]"
    return repr(value)
