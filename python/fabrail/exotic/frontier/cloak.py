"""🧪 lab / 🔭 structure: magnetic cloaks and concentrators, capped by iron saturation.

FAB-FRN-007; FRONTIER §F8. Lab cloaks exist at centimetre scale (Gömöry et al. 2012
Science 335:1466: a superconducting inner shell with a ferromagnetic outer shell, 77 K,
DC uniform field); a structure-scale cloak or concentrator around a rail is 🔭.

**Model** (frontier research §3e; PHYSICS §2, §23)::

    cloak:         μ_FM = (R2² + R1²)/(R2² − R1²)     for an ideal SC-FM bilayer cylinder
                   R2/R1 = 1.2 → μ = 5.5;  R2/R1 = 2 → μ = 1.67
    concentrator:  B_inside = (R2/R1)·B0              (μ_ρ → ∞, μ_φ → 0; Navau et al. 2012 [REF-MEM])
                   gain ≤ B_sat/B0                    (the shell saturates)

A concentrator cannot beat iron: the force per area stays ≤ B_sat²/2μ0 (FeCo 2.3 T →
2.1 MPa, against the 0.4–0.9 MPa of an EMS gap), a gain that pole tapering largely
gives already. Superconductor AC losses rule these structures out for LSM travelling
fields. Their real use is a rail that is magnetically invisible next to MRI, electron
microscopes or quantum instruments; the Halbach one-sided decay already meets the
0.5 mT target within about 1.2 λ without a cloak.

Scale label: 🧪 when R2 ≤ 1 cm (the lab scale), 🔭 above; the missing-orders figure
is log10(R2 / 1 cm), floored at 0 [UNCERTAIN: "cm-scale" lab cylinders].

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from ._const import magnetic_pressure_pa
from ._record import FrontierRecord, make_record, orders

__all__ = [
    "B_SAT_T",
    "LAB_SCALE_M",
    "RADIUS_RATIOS",
    "STRUCTURE_SCALE_M",
    "cloak_permeability",
    "concentrator_gain",
]

#: Saturation of the concentrator shell, T (FeCo; EXOTIC §X2 FAB-EXO-207; frontier
#: research §3e quotes 2.35 T → 2.2 MPa; PHYSICS §2).
B_SAT_T = 2.3
#: Largest demonstrated cloak radius, m ("cm-scale lab cloaks", Gömöry 2012; FRONTIER §1)
#: [UNCERTAIN] (FRONTIER §F8).
LAB_SCALE_M = 0.01
#: Default structure-scale outer radius, m (a rail-side cloak; FRONTIER §F8).
STRUCTURE_SCALE_M = 1.0
#: Radius ratios of the frontier research §3e worked examples (FRONTIER §F8).
RADIUS_RATIOS: tuple[float, ...] = (1.2, 2.0)


def _scale(r2_m: float) -> tuple[str, float, str]:
    if not math.isfinite(r2_m) or r2_m <= 0.0:
        raise ValueError(f"r2_m must be finite and > 0 (got {r2_m!r})")
    missing = orders(r2_m / LAB_SCALE_M)
    if missing == 0.0:
        return "cloak_lab", 0.0, "lab"
    return "cloak_structure", missing, "structure"


def _note(scale: str, r2_m: float) -> str:
    if scale == "lab":
        return (
            f"Lab-scale magnetic cloak (R2 = {r2_m * 100:.2g} cm): research tier; cm-scale lab "
            f"demonstrations exist; gain capped at B_sat/B0."
        )
    return (
        f"Structure-scale magnetic cloak or concentrator (R2 = {r2_m:g} m): lab demonstrations only "
        f"(about {LAB_SCALE_M * 100:.0f} cm); gain capped at B_sat/B0."
    )


def cloak_permeability(
    r2_over_r1: Sequence[float] | np.ndarray | float = RADIUS_RATIOS,
    r2_m: float = STRUCTURE_SCALE_M,
) -> FrontierRecord:
    """Ferromagnetic-shell permeability of an ideal bilayer cloak (FRONTIER §F8).

    Args:
        r2_over_r1: outer/inner radius ratio(s), > 1.
        r2_m: outer radius, m (sets the 🧪/🔭 label and the missing orders).

    Returns:
        A record with ``mu_r`` per ratio, ``scale`` ("lab" or "structure").
    """
    ratio = np.asarray(r2_over_r1, dtype=float).reshape(-1)
    if np.any(~np.isfinite(ratio)) or np.any(ratio <= 1.0):
        raise ValueError("r2_over_r1 must be finite and > 1")
    key, missing, scale = _scale(r2_m)
    q = ratio * ratio
    mu = (q + 1.0) / (q - 1.0)
    return make_record(
        key,
        "cloak.cloak_permeability",
        missing,
        _note(scale, r2_m),
        {"r2_over_r1": ratio, "r2_m": r2_m},
        {"mu_r": mu, "scale": scale},
    )


def concentrator_gain(
    r2_over_r1: float = 3.0,
    b0_t: float = 1.0,
    b_sat_t: float = B_SAT_T,
    r2_m: float = STRUCTURE_SCALE_M,
) -> FrontierRecord:
    """Concentrator field gain, capped by shell saturation (FRONTIER §F8).

    Args:
        r2_over_r1: outer/inner radius ratio, > 1 (the geometric gain).
        b0_t: applied field, T.
        b_sat_t: shell saturation, T.
        r2_m: outer radius, m.

    Returns:
        A record with ``gain_geometric`` (R2/R1), ``gain_cap`` (B_sat/B0), ``gain``
        (their minimum), ``b_inside_t``, ``pressure_gain`` (gain²), ``p_max_pa``
        (B_sat²/2μ0) and ``saturation_limited``.
    """
    if not (math.isfinite(r2_over_r1) and r2_over_r1 > 1.0) or b0_t <= 0.0 or b_sat_t <= 0.0:
        raise ValueError("r2_over_r1 > 1, b0_t > 0 and b_sat_t > 0 required")
    key, missing, scale = _scale(r2_m)
    cap = b_sat_t / b0_t
    gain = min(r2_over_r1, cap)
    return make_record(
        key,
        "cloak.concentrator_gain",
        missing,
        _note(scale, r2_m),
        {"r2_over_r1": r2_over_r1, "b0_t": b0_t, "b_sat_t": b_sat_t, "r2_m": r2_m},
        {
            "gain_geometric": r2_over_r1,
            "gain_cap": cap,
            "gain": gain,
            "b_inside_t": gain * b0_t,
            "pressure_gain": gain * gain,
            "p_max_pa": float(magnetic_pressure_pa(b_sat_t)),
            "saturation_limited": bool(r2_over_r1 > cap),
            "scale": scale,
        },
    )
