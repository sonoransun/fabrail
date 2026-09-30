#!/usr/bin/env python3
"""Gallery example 5: ``archive_heavy_shelving`` (✅).

5 t HEAVY archive shelving: hold-to-run and the momentum/energy governor with a sabotaged mass estimate.

Usage::

    python examples/archive_heavy_shelving.py [--quick] [--seed 42] [--output examples/out] [--strict]
    python -m fabrail example run archive_heavy_shelving [--quick]

Writes ``examples/out/archive_heavy_shelving/{figure.png, record.json, story.txt}``. This script is a
thin wrapper: the example lives in :mod:`fabrail.gallery.core_examples` (plan §4b).
Simulated, not measured on hardware.
"""

from __future__ import annotations

import sys
from pathlib import Path

NAME = "archive_heavy_shelving"


def main(argv: list[str] | None = None) -> int:
    try:
        from fabrail.gallery.core_examples import script_main
    except ImportError:  # running from a source checkout without an installed package
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
        from fabrail.gallery.core_examples import script_main
    return script_main(NAME, argv, doc=__doc__)


if __name__ == "__main__":
    raise SystemExit(main())
