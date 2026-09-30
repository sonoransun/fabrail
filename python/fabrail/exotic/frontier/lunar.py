"""🔭 SELENE: a lunar hatch and cargo rail in a permanently shadowed region (PSR).

FAB-FRN-006; FRONTIER §F7 (anchor tier 🧪: NASA FLOAT concept). EXOTIC §1 lists SELENE
(100 t hatch, 5–10 m travel, passive HTS in a PSR, teleoperated, LandAndLock) as
numpy-frontier only.

**Scaling** (frontier research §3g; PHYSICS §3 for the open-loop pole)::

    pole ratio   p_body/p_earth = √(g_body/g0) = √(1.62/9.81) = 0.406
                 (EMS pole p = √(2g/gap): Earth 44.3 rad/s, Moon 18.0 rad/s at 10 mm)
    lift ratio   g0/g_body = 6.05 (the same magnet lifts 6× the mass)
    FPS period   T = 2π√(R/g): ×√(g0/g_body) = 2.46 on the Moon
    radiator     q = ε·σ·T⁴ = 0.145 W/m² at 40 K (ε = 1), 0.022 W/m² at 25 K

PSRs sit at about 25–40 K (Diviner, Paige et al. 2010 [REF-MEM]), which suits REBCO, but
every watt dissipated there needs 7–45 m² of ideal radiator: only passive flux-pinned
HTS over permanent magnets, driven only while moving, fits the budget. NdFeB undergoes
a spin reorientation below about 135 K, so PSR magnets are PrFeB or SmCo. Shallow
moonquakes (up to mB 5.5) ring for about an hour, so holding matters more than warning
(LandAndLock).

**Dust caveat.** Lunar regolith carries nanophase metallic iron (npFe⁰), which is
ferromagnetic: dust collects on permanent-magnet guideways. fabrail does not model it.

Missing orders: log10(5 m hatch travel / 5 cm FLOAT prototypes) = 2 (FLOAT is a concept
with cm-scale prototypes; exotic critique XCRIT §6 [UNCERTAIN: prototype size]).

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from ._const import G0_MPS2, G_MARS_MPS2, G_MOON_MPS2, SIGMA_SB_W_PER_M2_K4
from ._record import FrontierRecord, make_record, orders

__all__ = [
    "DUST_CAVEAT",
    "FLOAT_PROTOTYPE_SCALE_M",
    "GAP_M",
    "HATCH_TRAVEL_M",
    "NDFEB_SPIN_REORIENTATION_K",
    "T_PSR_K",
    "gravity_sweep",
    "radiator_limit",
    "selene_scaling",
]

#: PSR temperature, K (Diviner: 25–40 K, Paige et al. 2010 [REF-MEM]; frontier research
#: §3g; FRONTIER FAB-FRN-006: 0.145 W/m² at 40 K; FRONTIER §F7).
T_PSR_K = 40.0
#: Pole example gap, m (frontier research §3g: p = √(2g/gap) at 10 mm; FRONTIER §F7; PHYSICS §3).
GAP_M = 0.010
#: NdFeB spin-reorientation temperature, K (frontier research §3g; exotic design §1
#: SELENE row: PrFeB/SmCo below ~135 K; FRONTIER §F7; PHYSICS §12).
NDFEB_SPIN_REORIENTATION_K = 135.0
#: SELENE hatch travel, m (EXOTIC §1 class table: 5–10 m; lower end; FRONTIER §F7).
HATCH_TRAVEL_M = 5.0
#: FLOAT prototype scale, m ("cm-scale prototypes", exotic critique XCRIT §6; FRONTIER §F7) [UNCERTAIN].
FLOAT_PROTOTYPE_SCALE_M = 0.05
#: The nanophase-Fe dust caveat, printed by every SELENE output (FAB-FRN-006).
DUST_CAVEAT = (
    "Dust caveat: lunar regolith carries nanophase metallic iron (npFe⁰), which is ferromagnetic; "
    "nanophase-Fe dust collects on permanent-magnet (PM) guideways and fabrail does not model it."
)
_MOONQUAKE_NOTE = "Shallow moonquakes reach mB 5.5 with codas of about 1 h: hold (LandAndLock) rather than race."


def _q_rad(t_k: np.ndarray | float, emissivity: float) -> np.ndarray:
    """Radiated flux ε·σ·T⁴, W/m² (Stefan–Boltzmann; FRONTIER §F7)."""
    t = np.asarray(t_k, dtype=float)
    return emissivity * SIGMA_SB_W_PER_M2_K4 * t**4


def _missing() -> float:
    return orders(HATCH_TRAVEL_M / FLOAT_PROTOTYPE_SCALE_M)


def _note(ratio: float, q: float, t_k: float) -> str:
    return (
        f"SELENE lunar hatch and cargo rail: pole ×{ratio:.3f}; radiator limit {q:.3f} W/m² at {t_k:.0f} K; "
        f"FLOAT is a concept with cm-scale prototypes. {DUST_CAVEAT}"
    )


def selene_scaling(
    g_body_mps2: float = G_MOON_MPS2,
    t_psr_k: float = T_PSR_K,
    emissivity: float = 1.0,
    gap_m: float = GAP_M,
) -> FrontierRecord:
    """Gravity, radiator and magnet scaling for the SELENE hatch (FRONTIER §F7).

    Returns:
        A record with ``pole_ratio`` (√(g/g0) = 0.406 on the Moon), ``pole_earth_rad_s``,
        ``pole_body_rad_s``, ``lift_ratio``, ``fps_period_ratio``, ``q_rad_w_m2``
        (0.145 W/m² at 40 K), ``radiator_m2_per_w``, ``ndfeb_usable`` and the
        ``dust_caveat`` text.
    """
    if g_body_mps2 <= 0.0 or t_psr_k <= 0.0 or not 0.0 < emissivity <= 1.0 or gap_m <= 0.0:
        raise ValueError("g_body_mps2, t_psr_k, gap_m > 0 and emissivity in (0, 1] required")
    ratio = math.sqrt(g_body_mps2 / G0_MPS2)
    q = float(_q_rad(t_psr_k, emissivity))
    return make_record(
        "selene",
        "lunar.selene_scaling",
        _missing(),
        _note(ratio, q, t_psr_k),
        {"g_body_mps2": g_body_mps2, "t_psr_k": t_psr_k, "emissivity": emissivity, "gap_m": gap_m},
        {
            "pole_ratio": ratio,
            "pole_earth_rad_s": math.sqrt(2.0 * G0_MPS2 / gap_m),
            "pole_body_rad_s": math.sqrt(2.0 * g_body_mps2 / gap_m),
            "lift_ratio": G0_MPS2 / g_body_mps2,
            "fps_period_ratio": math.sqrt(G0_MPS2 / g_body_mps2),
            "q_rad_w_m2": q,
            "radiator_m2_per_w": 1.0 / q,
            "ndfeb_usable": bool(t_psr_k > NDFEB_SPIN_REORIENTATION_K),
            "magnet_note": (
                f"NdFeB spin reorientation below about {NDFEB_SPIN_REORIENTATION_K:.0f} K: use PrFeB or SmCo"
            ),
            "moonquake_note": _MOONQUAKE_NOTE,
            "dust_caveat": DUST_CAVEAT,
        },
    )


def radiator_limit(
    t_k: Sequence[float] | np.ndarray | None = None,
    emissivity: float = 1.0,
) -> FrontierRecord:
    """Radiator-limited heat rejection ε·σ·T⁴ over temperature (default 25–400 K; FRONTIER §F7).

    Returns:
        A record with ``q_rad_w_m2`` and ``radiator_m2_per_w`` per temperature.
    """
    t = np.array([25.0, 40.0, 77.0, 135.0, 260.0, 400.0]) if t_k is None else np.asarray(t_k, float).reshape(-1)
    if np.any(t <= 0.0) or not 0.0 < emissivity <= 1.0:
        raise ValueError("t_k must be > 0 and emissivity in (0, 1]")
    q = _q_rad(t, emissivity)
    return make_record(
        "selene",
        "lunar.radiator_limit",
        _missing(),
        _note(math.sqrt(G_MOON_MPS2 / G0_MPS2), float(_q_rad(T_PSR_K, emissivity)), T_PSR_K),
        {"t_k": t, "emissivity": emissivity},
        {"q_rad_w_m2": q, "radiator_m2_per_w": 1.0 / q, "dust_caveat": DUST_CAVEAT},
    )


def gravity_sweep(
    g_mps2: Sequence[float] | np.ndarray = (G0_MPS2, G_MARS_MPS2, G_MOON_MPS2),
    gap_m: float = GAP_M,
) -> FrontierRecord:
    """Pole, lift and FPS-period scaling for Earth, Mars and the Moon (FRONTIER §F7).

    Returns:
        A record with ``pole_rad_s`` (√(2g/gap): 44.3 / 27.2 / 18.0 rad/s at 10 mm),
        ``pole_ratio``, ``lift_ratio`` and ``fps_period_ratio`` per gravity.
    """
    g = np.asarray(g_mps2, dtype=float).reshape(-1)
    if np.any(g <= 0.0) or gap_m <= 0.0:
        raise ValueError("g_mps2 and gap_m must be > 0")
    return make_record(
        "selene",
        "lunar.gravity_sweep",
        _missing(),
        _note(math.sqrt(G_MOON_MPS2 / G0_MPS2), float(_q_rad(T_PSR_K, 1.0)), T_PSR_K),
        {"g_mps2": g, "gap_m": gap_m},
        {
            "pole_rad_s": np.sqrt(2.0 * g / gap_m),
            "pole_ratio": np.sqrt(g / G0_MPS2),
            "lift_ratio": G0_MPS2 / g,
            "fps_period_ratio": np.sqrt(G0_MPS2 / g),
            "dust_caveat": DUST_CAVEAT,
        },
    )
