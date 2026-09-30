#!/usr/bin/env python3
"""Gallery example 4: ``ballroom_partition`` (✅).

A 500 kg WALL rendered at M_v = m/5: sensed against unsensed timing, seal to ±1 mm (rendered lightness does not reduce impact energy).

Usage::

    python examples/ballroom_partition.py [--quick] [--seed 42] [--output examples/out] [--strict]
    python -m fabrail example run ballroom_partition [--quick]

Writes ``examples/out/ballroom_partition/{figure.png, record.json, story.txt}``. This script is a
thin wrapper: the example lives in :mod:`fabrail.gallery.core_examples` (plan §4b).
Simulated, not measured on hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

NAME = "ballroom_partition"


def main(argv: list[str] | None = None) -> int:
    try:
        from fabrail.gallery.core_examples import script_main
    except ImportError:  # running from a source checkout without an installed package
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
        from fabrail.gallery.core_examples import script_main
    return script_main(NAME, argv, doc=__doc__)


if __name__ == "__main__":
    raise SystemExit(main())
