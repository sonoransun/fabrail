"""The FRONTIER §1 matrix and the frontier.html cards, generated from the models (FAB-FRN-010).

Every numeric claim in the "Anchor and missing orders of magnitude" column is formatted
from a model output, and every row's missing-orders figure is the one its headline
model prints, so the matrix cannot drift from the code without the
``py:exotic/frontier::matrix_matches_code`` guard failing. To regenerate the table,
print :func:`render_matrix` and paste it under FRONTIER §1 (docs owner); never
hand-edit the numbers.

:func:`frontier_cards` gives the ``EXO_RUN.frontier`` entries of
docs/assets/data/exo_run.js (schema ``fabrail.docs.exo_run/1``: item, badge, maturity,
anchor_tier, missing_orders, note, requirement), each also carrying ``tier`` and
``banner``.

The documented-only rows (NV/atom sensing, DAS, safe-RL) have small closed-form
calculators in this package; their maturity cell stays "documented only" until the
docs owner changes the matrix and this table together.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

from collections.abc import Callable

from . import cloak, das_rail, diamagnetic, lunar, metabarrier, pegs, quantum, qzs, rtsc, safe_rl, trapped_field
from . import frontier_banner
from ._items import ITEMS_BY_KEY, FrontierItem, matrix_items
from ._record import FrontierRecord

__all__ = [
    "MATRIX_HEADER",
    "frontier_cards",
    "headline",
    "matrix_rows",
    "missing_orders",
    "render_matrix",
]

#: The FRONTIER §1 header cells.
MATRIX_HEADER: tuple[str, ...] = (
    "Item",
    "Badge",
    "Maturity",
    "Anchor tier",
    "Anchor and missing orders of magnitude",
    "Requirement",
)

#: The headline model of each item (defaults reproduce the FRONTIER figures). safe_rl is
#: absent: its models read the native caps, and its figure is the constant
#: ``safe_rl.MISSING_ORDERS``.
_HEADLINES: dict[str, Callable[[], FrontierRecord]] = {
    "pegs": pegs.sensitivity_gap,
    "rtsc": rtsc.required_tc,
    "metabarrier": metabarrier.mass_law,
    "cloak_structure": lambda: cloak.concentrator_gain(r2_m=cloak.STRUCTURE_SCALE_M),
    "selene": lunar.selene_scaling,
    "trapped_field": trapped_field.pressure_cap,
    "diamagnetic": diamagnetic.pressure,
    "qzs": lambda: qzs.series_stiffness(scale="building"),
    "cloak_lab": lambda: cloak.cloak_permeability(r2_m=cloak.LAB_SCALE_M),
    "quantum": quantum.nv_gap_noise,
    "das_rail": das_rail.beamform_moveout,
    "pegs_network": pegs.magnitude_advisory,
}


def headline(key: str) -> FrontierRecord | None:
    """The headline record of item ``key`` (None for safe_rl, which needs the native caps)."""
    if key not in ITEMS_BY_KEY:
        raise KeyError(f"unknown frontier item {key!r}")
    fn = _HEADLINES.get(key)
    return fn() if fn is not None else None


def missing_orders(key: str) -> float:
    """The banner's missing-orders figure of item ``key`` (rounded to 0.1)."""
    rec = headline(key)
    if rec is None:
        return round(float(safe_rl.MISSING_ORDERS), 1)
    return float(rec["missing_orders"])


def _anchor_text(key: str) -> str:
    """The "Anchor and missing orders of magnitude" cell, numbers formatted from the models."""
    if key == "pegs":
        gap = pegs.sensitivity_gap()
        det = pegs.detection()
        return (
            f"Needs {gap['asd_needed_per_rthz']:.0e}/√Hz; best demonstrated {gap['asd_best_per_rthz']:.0e}/√Hz, "
            f"{gap['missing_orders']:g} orders short. Against a dense P network the lead-time gain is ≈ "
            f"{det['gain_vs_network_s']:.0f} s; the real value is an unsaturated magnitude"
        )
    if key == "rtsc":
        tc = rtsc.required_tc()
        bound = rtsc.pressure_bound()
        return (
            f"Needs Tc ≈ {tc['tc_required_k']:.0f} K and more pinning energy; the pressure ceiling stays "
            f"rail-limited ({bound['p_bound_pa'] / 1e6:.2f} MPa at {bound['inputs']['h_m'] * 1e3:.0f} mm)"
        )
    if key == "metabarrier":
        law = metabarrier.mass_law()
        f = law["inputs"]["f_hz"]
        v = law["inputs"]["v_s_mps"]
        m = law["mass_mt"]
        return (
            f"ρλ³ at {f:g} Hz: {m[0]:.1f} Mt of resonating mass at v_s = {v[0]:.0f} m/s "
            f"({m[1]:.1f} Mt at {v[1]:.0f} m/s)"
        )
    if key == "cloak_structure":
        return "Lab demonstrations only; gain capped at B_sat/B0"
    if key == "selene":
        s = lunar.selene_scaling()
        return (
            f"Pole ×{s['pole_ratio']:.3f}; radiator limit {s['q_rad_w_m2']:.3f} W/m² at "
            f"{s['inputs']['t_psr_k']:.0f} K; nanophase-Fe dust collects on PM guideways"
        )
    if key == "trapped_field":
        t = trapped_field.pressure_cap()
        return f"Pressure capped by rail saturation (≤ {t['p_rail_pa'] / 1e6:.1f} MPa against steel) whatever B_trapped"
    if key == "diamagnetic":
        d = diamagnetic.pressure()
        return f"{d['p_pa']:.0f} Pa at {d['inputs']['b_t']:g} T as an upper bound"
    if key == "qzs":
        return "Stiffness singularity at k_m = −k1; tilt error g·θ/ω²"
    if key == "quantum":
        return "Noise-ASD presets are roadmap"
    return "—"


def _row(item: FrontierItem) -> tuple[str, str, str, str, str, str]:
    return (item.item, item.badge, item.maturity, item.anchor_tier, _anchor_text(item.key), item.requirement)


def matrix_rows() -> list[tuple[str, str, str, str, str, str]]:
    """The FRONTIER §1 rows (six cells each), in matrix order."""
    return [_row(it) for it in matrix_items()]


def render_matrix() -> str:
    """The FRONTIER §1 matrix as a Markdown table (header, separator, rows)."""
    lines = ["| " + " | ".join(MATRIX_HEADER) + " |", "|" + "---|" * len(MATRIX_HEADER)]
    lines += ["| " + " | ".join(row) + " |" for row in matrix_rows()]
    return "\n".join(lines)


def frontier_cards() -> list[dict[str, object]]:
    """The ``EXO_RUN.frontier`` cards for frontier.html, one per matrix row, each bannered.

    Keys: item, badge, maturity, anchor_tier, missing_orders, note (the anchor cell, or
    the item name when the cell is "—"), requirement, plus tier and banner.
    """
    cards: list[dict[str, object]] = []
    for it in matrix_items():
        mo = missing_orders(it.key)
        text = _anchor_text(it.key)
        note = text if text != "—" else it.item
        banner = frontier_banner(mo, f"{it.item}: {note}" if note != it.item else it.item)
        cards.append(
            {
                "item": it.item,
                "badge": it.badge,
                "maturity": it.maturity,
                "anchor_tier": it.anchor_tier,
                "missing_orders": mo,
                "note": note,
                "requirement": it.requirement,
                "tier": banner.meta["tier"],
                "banner": banner.text,
            }
        )
    return cards
