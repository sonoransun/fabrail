#!/usr/bin/env python3
"""Gallery example 7: ``wall_pwave_m7`` (🔶).

Land-first timeline of a moving WALL against t_S under a synthetic M7 at 20 km (the seismic action is not a rated safety function, SF-08).

Usage::

    python examples/wall_pwave_m7.py [--quick] [--seed 42] [--output examples/out] [--strict]
    python -m fabrail example run wall_pwave_m7 [--quick]

Writes ``examples/out/wall_pwave_m7/{figure.png, record.json, story.txt}``. This script is a
thin wrapper: the example lives in :mod:`fabrail.gallery.core_examples` (plan §4b).
Simulated, not measured on hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

NAME = "wall_pwave_m7"


def main(argv: list[str] | None = None) -> int:
    try:
        from fabrail.gallery.core_examples import script_main
    except ImportError:  # running from a source checkout without an installed package
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
        from fabrail.gallery.core_examples import script_main
    return script_main(NAME, argv, doc=__doc__)


if __name__ == "__main__":
    raise SystemExit(main())
