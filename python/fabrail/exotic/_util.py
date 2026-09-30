"""Shared helpers of the exotic Python layer (private; EXOTIC.md §X1).

- :func:`native` loads ``fabrail._native`` lazily, so ``import fabrail.exotic`` and the
  ``fabrail exo --help`` path work without the compiled extension.
- :func:`pedestrian_caps` reads the pedestrian caps from the native defaults, which are
  the ``fabrail_params::presets`` constants (FAB-EXO-114: imported, never restated).
- Physical constants carry their source and the Rust constant they mirror.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import ModuleType

__all__ = [
    "C0_AIR_MPS",
    "G0_MPS2",
    "GAMMA_AIR",
    "MU0_H_PER_M",
    "P0_PA",
    "PedestrianCaps",
    "fmt_sig",
    "fmt_thousands",
    "native",
    "pedestrian_caps",
]

#: Standard gravity, m/s² (CGPM 1901; ``fabrail_params::derive::G0_MPS2``).
G0_MPS2 = 9.806_65
#: Vacuum permeability, H/m (CODATA 2018; ``fabrail_params::derive::MU0_H_PER_M``).
MU0_H_PER_M = 1.256_637_062_12e-6
#: Standard atmosphere, Pa (EXOTIC §X11, Rankine–Hugoniot reflection).
P0_PA = 101_325.0
#: Ratio of specific heats of air (ideal gas; EXOTIC §X11).
GAMMA_AIR = 1.4
#: Speed of sound in dry air at 20 °C, m/s (``fabrail_exo::closure::C0_AIR_MPS``; ISO 9613-1).
C0_AIR_MPS = 343.0

_BUILD_HINT = "build it with `maturin develop --release` (see CLAUDE.md)"


def native() -> ModuleType:
    """The compiled ``fabrail._native`` module.

    Raises:
        ImportError: the extension is not built (the message says how to build it).
    """
    try:
        from fabrail import _native
    except ImportError as exc:  # pragma: no cover - depends on the build
        raise ImportError(f"fabrail.exotic needs the compiled extension fabrail._native; {_BUILD_HINT}") from exc
    return _native


@dataclass(frozen=True)
class PedestrianCaps:
    """The core pedestrian caps as imported from ``fabrail_params::presets`` (PHYSICS §16).

    ``e_max_j`` and ``p_max_kgmps`` are the defaults of the native ``v_low_energy`` and
    ``momentum_speed_cap`` (``PEDESTRIAN_E_MAX_J``, ``PEDESTRIAN_P_MAX_KGMPS``); the static
    force limits are read from the DOOR preset's safety block, which the presets fill
    from ``PEDESTRIAN_F_STATIC_N`` and ``PEDESTRIAN_F_STATIC_SENSED_N``. No literal is
    restated here (FAB-EXO-114).
    """

    e_max_j: float
    p_max_kgmps: float
    f_static_n: float
    f_static_sensed_n: float
    source: str

    def energy_speed_mps(self, m_kg: float) -> float:
        """Speed allowed by the energy cap, √(2·E/m), m/s (EXOTIC §X1 table)."""
        return math.sqrt(2.0 * self.e_max_j / float(m_kg))

    def momentum_speed_mps(self, m_kg: float) -> float:
        """Speed allowed by the momentum cap, p/m, m/s (EXOTIC §X1 table)."""
        return self.p_max_kgmps / float(m_kg)


def pedestrian_caps() -> PedestrianCaps:
    """Read the pedestrian caps from the native build (single source, FAB-EXO-114)."""
    n = native()
    e = 0.5 * n.v_low_energy(1.0) ** 2  # default e_j = PEDESTRIAN_E_MAX_J, so v² = 2E at 1 kg
    p = n.momentum_speed_cap(1.0)  # default p = PEDESTRIAN_P_MAX_KGMPS
    door = n.Params.preset("door")
    return PedestrianCaps(
        e_max_j=float(e),
        p_max_kgmps=float(p),
        f_static_n=float(door.get("safety.f_static_n")),
        f_static_sensed_n=float(door.get("safety.f_static_sensed_n")),
        source="fabrail._native defaults of v_low_energy/momentum_speed_cap and Params.preset('door').safety "
        "(fabrail_params::presets PEDESTRIAN_*)",
    )


def fmt_thousands(value: float) -> str:
    """``1000.0`` → ``"1,000"`` (integers with thousands separators, as in EXOTIC tables)."""
    return f"{round(value):,}"


def fmt_sig(value: float, sig: int = 3) -> str:
    """``value`` to ``sig`` significant figures, fixed-point: 57.05 → "57.1", 285.26 → "285", 53000 → "53000".

    Values of more than ``sig`` integer digits are rounded to an integer (callers switch
    units first, e.g. kg/day → t/day, as the EXOTIC generated tables do).
    """
    v = float(value)
    if v == 0.0 or not math.isfinite(v):
        return f"{v:g}"
    exponent = math.floor(math.log10(abs(v)))
    decimals = max(0, sig - 1 - exponent)
    rounded = round(v, decimals)
    # Rounding can carry into a new digit (9.996 → 10.0); recompute the decimals.
    if rounded != 0.0 and math.floor(math.log10(abs(rounded))) != exponent:
        decimals = max(0, sig - 1 - math.floor(math.log10(abs(rounded))))
        rounded = round(v, decimals)
    return f"{rounded:.{decimals}f}"
