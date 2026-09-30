#!/usr/bin/env python3
"""Gallery example 3: ``office_door_obstruction`` (🔶).

A low-energy DOOR closing onto a chest, checked in simulation against numeric limits from EN 16005 (2011 draft).

Usage::

    python examples/office_door_obstruction.py [--quick] [--seed 42] [--output examples/out] [--strict]
    python -m fabrail example run office_door_obstruction [--quick]

Writes ``examples/out/office_door_obstruction/{figure.png, record.json, story.txt}``. This script is a
thin wrapper: the example lives in :mod:`fabrail.gallery.core_examples` (plan §4b).
Simulated, not measured on hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

NAME = "office_door_obstruction"


def main(argv: list[str] | None = None) -> int:
    try:
        from fabrail.gallery.core_examples import script_main
    except ImportError:  # running from a source checkout without an installed package
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
        from fabrail.gallery.core_examples import script_main
    return script_main(NAME, argv, doc=__doc__)


if __name__ == "__main__":
    raise SystemExit(main())
