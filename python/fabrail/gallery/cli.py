"""``python -m fabrail example list | run NAME [--quick]``: the gallery CLI hook (plan §4b).

:func:`register` receives the top-level subparsers object from
:func:`fabrail.__main__.make_parser` and adds the ``example`` parser; every parser it
adds sets a ``handler(args) -> int`` default.

Wave-0 stub: the flags below are the frozen entry point; the handlers print
``not yet implemented (Wave 2b, unit E5)`` and return 2.

Owner: W0-C2 (stub), then E5 (Wave 2b).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .registry import EXAMPLE_NAMES, OUT_DIR

__all__ = ["BUILDINGS", "VARIANTS", "register"]

#: Villa scene variants (scene-1 schema ``variant``).
VARIANTS: tuple[str, ...] = ("warned", "near_field", "spoofed_alert", "quiet", "custom")

#: Villa building configurations (scene-1 schema ``building.config``).
BUILDINGS: tuple[str, ...] = ("fixed_base", "fps_isolated", "haven_f", "tower_sentinel")


def _pending(args: argparse.Namespace) -> int:
    what = getattr(args, "example_command", None)
    name = f"fabrail example {what}" if what else "fabrail example"
    print(f"{name}: not yet implemented (Wave 2b, unit E5)", file=sys.stderr)
    return 2


def register(subparsers: argparse._SubParsersAction) -> argparse.ArgumentParser:
    """Add the ``example`` subcommand to ``subparsers`` and return its parser."""
    parser = subparsers.add_parser(
        "example",
        help="example gallery: list and run the named examples",
        description="Run the example gallery; outputs go to examples/out/<name>/.",
    )
    parser.set_defaults(handler=_pending)
    sub = parser.add_subparsers(dest="example_command", metavar="EXAMPLE_COMMAND")

    p = sub.add_parser("list", help="list the examples with their badges")
    p.set_defaults(handler=_pending)

    p = sub.add_parser("run", help="run one example")
    p.add_argument("name", choices=EXAMPLE_NAMES, metavar="NAME", help="example name (see `example list`)")
    p.add_argument("--quick", action="store_true", help="reduced run (the test-suite default)")
    p.add_argument("--variant", choices=VARIANTS, default=None, help="villa_quake scene variant")
    p.add_argument("--building", choices=BUILDINGS, default=None, help="villa_quake building configuration")
    p.add_argument("-s", "--seed", type=int, default=42, help="master seed (default: 42)")
    p.add_argument("-o", "--output", type=Path, default=OUT_DIR, help=f"output root (default: {OUT_DIR})")
    p.set_defaults(handler=_pending)
    return parser
