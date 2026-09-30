"""Static metadata of the frontier items: the non-numeric cells of the FRONTIER §1 matrix.

The matrix is generated from this table plus the model outputs
(:mod:`fabrail.exotic.frontier.meta`, FAB-FRN-010). The badge is the core status
(✅ V, 🔶 S, 🧪 R, 🗺️ M), with 🔭 replacing it whenever the maturity is frontier; the
maturity is a separate column and never merges into the badge (EXOTIC §0.1,
FAB-EXO-001). A 🔭 row always names its anchor tier.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import NamedTuple

from ._const import FLOAT_PAYLOAD_KG_M2

__all__ = ["ITEMS", "ITEMS_BY_KEY", "FrontierItem", "matrix_items"]


class FrontierItem(NamedTuple):
    """One frontier item: the static cells of its FRONTIER §1 row."""

    #: Short key, used by records (``record["item_key"]``) and cards.
    key: str
    #: The "Item" cell.
    item: str
    #: The "Badge" cell.
    badge: str
    #: The "Maturity" cell.
    maturity: str
    #: The "Anchor tier" cell ("—" when the item is not frontier).
    anchor_tier: str
    #: The "Requirement" cell ("—" when none).
    requirement: str
    #: The model submodule of :mod:`fabrail.exotic.frontier`.
    module: str
    #: Whether the item is a row of the FRONTIER §1 matrix.
    in_matrix: bool = True


#: Every item, in FRONTIER §1 matrix order, then the items that are not matrix rows.
ITEMS: tuple[FrontierItem, ...] = (
    FrontierItem(
        "pegs",
        "PEGS on-site gravity gradiometer",
        "🔭",
        "frontier",
        "🧪 (PEGS network magnitude, Mw ≥ 8.3)",
        "FAB-FRN-003",
        "pegs",
    ),
    FrontierItem(
        "rtsc",
        "Ambient-pressure RTSC suspension",
        "🔭",
        "frontier",
        "🧪 (HTS flux pinning)",
        "FAB-FRN-001",
        "rtsc",
    ),
    FrontierItem(
        "metabarrier",
        "Building-scale active metabarrier",
        "🔭",
        "frontier",
        "🧪",
        "FAB-FRN-004",
        "metabarrier",
    ),
    FrontierItem(
        "cloak_structure",
        "Structure-scale magnetic cloak or concentrator",
        "🔭",
        "frontier",
        "🧪 (cm-scale lab cloaks)",
        "FAB-FRN-007",
        "cloak",
    ),
    FrontierItem(
        "selene",
        "SELENE lunar hatch and cargo rail",
        "🔭",
        "frontier",
        "🧪 (FLOAT concept)",
        "FAB-FRN-006",
        "lunar",
    ),
    FrontierItem(
        "trapped_field",
        "Trapped-field bulk magnets",
        "🧪",
        "research (relabelled from 🔭)",
        "—",
        "FAB-FRN-002",
        "trapped_field",
    ),
    FrontierItem(
        "diamagnetic",
        "Diamagnetic levitation",
        "✅ physics",
        f"physics deployed; FLOAT research (cm-scale, ≤ {FLOAT_PAYLOAD_KG_M2:.0f} kg/m²)",
        "—",
        "FAB-FRN-009",
        "diamagnetic",
    ),
    FrontierItem(
        "qzs",
        "Quasi-zero-stiffness (QZS) isolation",
        "🧪",
        "deployed at instrument scale; research at building scale",
        "—",
        "FAB-FRN-005",
        "qzs",
    ),
    FrontierItem(
        "cloak_lab",
        "Lab-scale magnetic cloak",
        "🧪",
        "research",
        "—",
        "FAB-FRN-007",
        "cloak",
    ),
    FrontierItem(
        "quantum",
        "NV-centre or atom-interferometer sensing",
        "—",
        "documented only",
        "—",
        "—",
        "quantum",
    ),
    FrontierItem(
        "das_rail",
        "DAS along the rail",
        "🧪",
        "documented only",
        "—",
        "—",
        "das_rail",
    ),
    FrontierItem(
        "safe_rl",
        "Safe-RL haptics with a CBF filter",
        "🧪",
        "documented only",
        "—",
        "—",
        "safe_rl",
    ),
    # Not a matrix row: the network PEGS magnitude is the anchor tier of the "pegs" row
    # (EXOTIC FAB-EXO-810: only a network magnitude advisory enters fusion).
    FrontierItem(
        "pegs_network",
        "PEGS network magnitude advisory",
        "🧪",
        "research (PEGSNet offline; PEGSGraph real time in Peru)",
        "—",
        "FAB-EXO-810",
        "pegs",
        in_matrix=False,
    ),
)

#: The items keyed by :attr:`FrontierItem.key`.
ITEMS_BY_KEY: Mapping[str, FrontierItem] = {it.key: it for it in ITEMS}


def matrix_items() -> tuple[FrontierItem, ...]:
    """The items that are FRONTIER §1 matrix rows, in row order."""
    return tuple(it for it in ITEMS if it.in_matrix)
