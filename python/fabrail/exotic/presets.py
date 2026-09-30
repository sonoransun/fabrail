"""Exotic class presets as Python data (EXOTIC §1, §S1; FAB-EXO-101, FAB-EXO-102).

The presets are ``fabrail_exo_safety::presets`` (plain data, integer units), read through
``_native.exo_class_table()``. This module only converts them into typed, SI-unit
Python views; it never restates a class value. The pedestrian caps come from
``fabrail_params::presets`` through :func:`fabrail.exotic._util.pedestrian_caps`
(FAB-EXO-114).

    >>> from fabrail.exotic import presets
    >>> p = presets.preset("sentinel")          # doctest: +SKIP
    >>> p.design_mass_kg, p.regime.kind, p.badge_symbol   # doctest: +SKIP
    (1000000.0, 'GZ', '🧪')

Badge and maturity are separate fields (EXOTIC §0.1): ``badge`` is the core status
(V/S/R/M) or ``Frontier`` when the technology's maturity is frontier; ``trl`` is the
technology readiness range.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Any

from ._util import native, pedestrian_caps

__all__ = [
    "BADGE_SYMBOLS",
    "CLASS_NAMES",
    "ExoPreset",
    "Regime",
    "all_presets",
    "badge_rank",
    "class_table_markdown",
    "pedestrian_regime_speeds",
    "preset",
]

#: Badge name → symbol (EXOTIC §0.1; SPECIFICATION §0.5). Order is strongest first.
BADGE_SYMBOLS: dict[str, str] = {
    "Verified": "✅",
    "Simplified": "🔶",
    "Research": "🧪",
    "Roadmap": "🗺️",
    "Frontier": "🔭",
}

#: The exotic classes in the order of EXOTIC §1 (``_native.exo_classes()``).
CLASS_NAMES: tuple[str, ...] = (
    "sentinel",
    "haven_r",
    "haven_f",
    "shield",
    "bastion",
    "arena",
    "rotator",
    "tidegate",
    "selene",
)

_U64_MAX = 2**64 - 1


def badge_rank(symbol: str) -> int:
    """Strength of a badge symbol, strongest first: ✅ 0, 🔶 1, 🧪 2, 🗺️ 3, 🔭 4 (EXOTIC §0.1)."""
    order = list(BADGE_SYMBOLS.values())
    for i, s in enumerate(order):
        if symbol.startswith(s.rstrip("️")):
            return i
    raise ValueError(f"not a badge symbol: {symbol!r}")


@dataclass(frozen=True)
class Regime:
    """A class's safety regime (EXOTIC §S1): GuardedZone (``GZ``) or OccupiedPlatform (``OP``).

    GuardedZone fields (SI): ISO 13855 inputs ``k_mps`` (approach speed), ``t1_s``,
    ``c_m``, ``a_brake_mps2``, the stated separation ``iso13855_s_m`` and the attended
    jog limits. OccupiedPlatform fields: occupant limits ``a_peak_max_mps2``,
    ``jerk_max_mps3``, ``landing_v_max_mps`` and the moat flags. Missing fields are None.
    """

    kind: str
    presence_pl_e: bool = False
    pps: bool = False
    two_person_override: bool = False
    teleoperated: bool = False
    k_mps: float | None = None
    t1_s: float | None = None
    c_m: float | None = None
    a_brake_mps2: float | None = None
    jog_v_mps: float | None = None
    edge_k_n_per_m: float | None = None
    iso13855_s_m: float | None = None
    a_peak_max_mps2: float | None = None
    jerk_max_mps3: float | None = None
    landing_v_max_mps: float | None = None
    moat_presence: bool = False
    moat_covers: bool = False

    @property
    def name(self) -> str:
        """``GuardedZone`` or ``OccupiedPlatform``."""
        return {"GZ": "GuardedZone", "OP": "OccupiedPlatform"}.get(self.kind, self.kind)

    @classmethod
    def from_native(cls, raw: dict[str, Any]) -> Regime:
        def mm(key: str, scale: float = 1e-3) -> float | None:
            v = raw.get(key)
            if v is None or (isinstance(v, int) and v >= _U64_MAX):
                return None  # u64::MAX marks "not applicable" (teleoperated SELENE)
            return float(v) * scale

        return cls(
            kind=str(raw.get("kind")),
            presence_pl_e=bool(raw.get("presence_pl_e", False)),
            pps=bool(raw.get("pps", False)),
            two_person_override=bool(raw.get("two_person_override", False)),
            teleoperated=bool(raw.get("teleoperated", False)),
            k_mps=mm("k_mmps"),
            t1_s=mm("t1_ms"),
            c_m=mm("c_mm"),
            a_brake_mps2=mm("a_brake_mmps2"),
            jog_v_mps=mm("jog_v_mmps"),
            edge_k_n_per_m=mm("edge_k_n_per_mm", 1e3),
            iso13855_s_m=mm("iso13855_s_mm"),
            a_peak_max_mps2=mm("a_peak_max_mmps2"),
            jerk_max_mps3=mm("jerk_max_mmps3"),
            landing_v_max_mps=mm("landing_v_max_mmps"),
            moat_presence=bool(raw.get("moat_presence", False)),
            moat_covers=bool(raw.get("moat_covers", False)),
        )


@dataclass(frozen=True)
class ExoPreset:
    """One exotic class preset in SI units (``fabrail_exo_safety::presets``; EXOTIC §1, §S1).

    ``travel_m``, ``v_max_mps``, ``a_max_mps2``, ``j_max_mps3`` and ``period_s`` are the
    class kinematics (0 where the table is silent); ``row`` holds the verbatim EXOTIC §1
    cells (class, design_mass, suspension, regime, maturity, effort).
    """

    name: str
    display: str
    design_mass_kg: float
    mass_min_kg: float
    mass_max_kg: float
    suspension: str
    pole: str
    regime: Regime
    safe_state: str
    seismic_action: str
    seismic_role: str
    rest_policy: str
    travel_m: float
    v_max_mps: float
    a_max_mps2: float
    j_max_mps3: float
    period_s: float
    badge: str
    trl: tuple[int, int]
    row: dict[str, str] = field(default_factory=dict)

    @property
    def badge_symbol(self) -> str:
        """The single badge of the class (🔭 when the maturity is frontier)."""
        return BADGE_SYMBOLS[self.badge]

    @property
    def maturity(self) -> str:
        """The EXOTIC §1 maturity cell (TRL, or 🔭 for frontier), separate from the badge."""
        return self.row.get("maturity", f"TRL {self.trl[0]}–{self.trl[1]}")

    @property
    def is_frontier(self) -> bool:
        """True for a frontier-maturity class (SELENE): numpy frontier module only."""
        return self.badge == "Frontier"

    @property
    def kinetic_energy_j(self) -> float:
        """½·m_design·v_max², J (the bumper rating uses the rated mass; FAB-EXO-504)."""
        return 0.5 * self.design_mass_kg * self.v_max_mps**2

    @property
    def momentum_kgmps(self) -> float:
        """m_design·v_max, kg·m/s (stated, never capped, for sovereign classes; EXOTIC §X14)."""
        return self.design_mass_kg * self.v_max_mps

    @classmethod
    def from_native(cls, raw: dict[str, Any]) -> ExoPreset:
        period_ms = float(raw.get("period_ms", 0))
        return cls(
            name=str(raw["class"]),
            display=str(raw["display"]),
            design_mass_kg=float(raw["design_mass_kg"]),
            mass_min_kg=float(raw["mass_min_kg"]),
            mass_max_kg=float(raw["mass_max_kg"]),
            suspension=str(raw["suspension"]),
            pole=str(raw["pole"]),
            regime=Regime.from_native(dict(raw["regime"])),
            safe_state=str(raw["safe_state"]),
            seismic_action=str(raw["seismic_action"]),
            seismic_role=str(raw["seismic_role"]),
            rest_policy=str(raw["rest_policy"]),
            travel_m=float(raw["travel_mm"]) * 1e-3,
            v_max_mps=float(raw["v_max_mmps"]) * 1e-3,
            a_max_mps2=float(raw["a_max_mmps2"]) * 1e-3,
            j_max_mps3=float(raw["j_max_mmps3"]) * 1e-3,
            period_s=period_ms * 1e-3,
            badge=str(raw["badge"]),
            trl=(int(raw["trl_lo"]), int(raw["trl_hi"])),
            row={str(k): str(v) for k, v in dict(raw.get("row", {})).items()},
        )


@lru_cache(maxsize=1)
def _table() -> tuple[ExoPreset, ...]:
    return tuple(ExoPreset.from_native(dict(r)) for r in native().exo_class_table())


def all_presets() -> list[ExoPreset]:
    """Every exotic class preset, in EXOTIC §1 order."""
    return list(_table())


def preset(name: str) -> ExoPreset:
    """The preset of class ``name`` (``sentinel``, ``haven_r``, …; case-insensitive, ``-`` = ``_``).

    Raises:
        KeyError: unknown class (the message lists the known ones).
    """
    key = str(name).strip().lower().replace("-", "_")
    for p in _table():
        if p.name == key or p.display.lower().replace("-", "_") == key:
            return p
    raise KeyError(f"unknown exotic class {name!r} (known: {', '.join(p.name for p in _table())})")


_TABLE_HEADER = (
    "| Class | Design mass | Suspension and key parameters | Regime / safe state | Maturity | This effort |\n"
    "|---|---|---|---|---|---|"
)


def class_table_markdown() -> str:
    """The EXOTIC §1 class table rendered from the preset rows (FAB-EXO-101)."""
    lines = [_TABLE_HEADER]
    for p in _table():
        r = p.row
        cells = (r["class"], r["design_mass"], r["suspension"], r["regime"], r["maturity"], r["effort"])
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def pedestrian_regime_speeds(masses_kg: tuple[float, ...] = (5e3, 5e4, 5e5)) -> list[dict[str, float]]:
    """Speeds the pedestrian caps would allow at sovereign masses (EXOTIC §X1 "Safe by separation").

    Returns rows ``{m_kg, v_energy_mps, v_momentum_mps}``: v ≤ √(2·E/m) and v ≤ p/m with
    the imported caps. At 500 t these are 2.6 mm/s and 0.045 mm/s, which is why every
    exotic class declares GuardedZone or OccupiedPlatform instead.
    """
    caps = pedestrian_caps()
    out = []
    for m in masses_kg:
        if not (math.isfinite(m) and m > 0):
            raise ValueError(f"mass must be finite and > 0 (got {m!r})")
        out.append({"m_kg": float(m), "v_energy_mps": caps.energy_speed_mps(m), "v_momentum_mps": caps.momentum_speed_mps(m)})
    return out
