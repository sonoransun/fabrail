#!/usr/bin/env python3
"""Gallery example 1: ``kitchen_push_open`` (✅).

A kitchen DRAWER: push-to-open from the seat, log-damped open and close termini, apparent mass within ±10 % of M_v across 8–48 kg (haptic lightness does not reduce impact energy).

Usage::

    python examples/kitchen_push_open.py [--quick] [--seed 42] [--output examples/out] [--strict]
    python -m fabrail example run kitchen_push_open [--quick]

Writes ``examples/out/kitchen_push_open/{figure.png, record.json, story.txt}``. This script is a
thin wrapper: the example lives in :mod:`fabrail.gallery.core_examples` (plan §4b).
Simulated, not measured on hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

NAME = "kitchen_push_open"


def main(argv: list[str] | None = None) -> int:
    try:
        from fabrail.gallery.core_examples import script_main
    except ImportError:  # running from a source checkout without an installed package
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
        from fabrail.gallery.core_examples import script_main
    return script_main(NAME, argv, doc=__doc__)


if __name__ == "__main__":
    raise SystemExit(main())
