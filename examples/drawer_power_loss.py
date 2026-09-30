#!/usr/bin/env python3
"""Gallery example 6: ``drawer_power_loss`` (✅).

SS1 Safe Landing of an open DRAWER on mains loss; manual force; manual re-seat required above 20 kg.

Usage::

    python examples/drawer_power_loss.py [--quick] [--seed 42] [--output examples/out] [--strict]
    python -m fabrail example run drawer_power_loss [--quick]

Writes ``examples/out/drawer_power_loss/{figure.png, record.json, story.txt}``. This script is a
thin wrapper: the example lives in :mod:`fabrail.gallery.core_examples` (plan §4b).
Simulated, not measured on hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

NAME = "drawer_power_loss"


def main(argv: list[str] | None = None) -> int:
    try:
        from fabrail.gallery.core_examples import script_main
    except ImportError:  # running from a source checkout without an installed package
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
        from fabrail.gallery.core_examples import script_main
    return script_main(NAME, argv, doc=__doc__)


if __name__ == "__main__":
    raise SystemExit(main())
