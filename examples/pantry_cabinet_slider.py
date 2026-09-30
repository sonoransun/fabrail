#!/usr/bin/env python3
"""Gallery example 2: ``pantry_cabinet_slider`` (✅).

An 8 kg sliding cabinet door derived from DRAWER (EgressRole None, latch fitted): detents and a semilog terminus plot.

Usage::

    python examples/pantry_cabinet_slider.py [--quick] [--seed 42] [--output examples/out] [--strict]
    python -m fabrail example run pantry_cabinet_slider [--quick]

Writes ``examples/out/pantry_cabinet_slider/{figure.png, record.json, story.txt}``. This script is a
thin wrapper: the example lives in :mod:`fabrail.gallery.core_examples` (plan §4b).
Simulated, not measured on hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

NAME = "pantry_cabinet_slider"


def main(argv: list[str] | None = None) -> int:
    try:
        from fabrail.gallery.core_examples import script_main
    except ImportError:  # running from a source checkout without an installed package
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
        from fabrail.gallery.core_examples import script_main
    return script_main(NAME, argv, doc=__doc__)


if __name__ == "__main__":
    raise SystemExit(main())
