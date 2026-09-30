"""✅ physics / 🧪 FLOAT: diamagnetic levitation pressure and the NASA FLOAT consistency check.

FAB-FRN-009; FRONTIER §F9. The physics is deployed (diamagnetic levitation of graphite
is textbook); FLOAT (NASA/JPL "Flexible Levitation on A Track", NIAC) is research with
cm-scale prototypes and a claimed payload of up to 33 kg/m² (exotic critique XCRIT §6).

**Model** (frontier research §3g; PHYSICS §2 for the magnetic pressure)::

    p = abs(χ)·B²/(2μ0)        179 Pa at 1 T for pyrolytic graphite, χ⊥ ≈ −4.5e-4

This is an upper bound (the thick-body limit: the whole B² acts on a diamagnet that
fills the field region). The gross areal mass it lifts is p/g: 110 kg/m² under lunar
gravity, which is consistent with FLOAT's 33 kg/m² of payload, but only 18 kg/m² on
Earth. At the same field an EMS rail pulls B²/2μ0, so diamagnetic levitation is
abs(χ) = 4.5e-4 of it: the missing-orders figure is log10(1/abs(χ)) ≈ 3.3.

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math

import numpy as np

from ._const import FLOAT_PAYLOAD_KG_M2, G0_MPS2, G_MOON_MPS2, magnetic_pressure_pa
from ._record import FrontierRecord, make_record, orders

__all__ = ["B_REFERENCE_T", "CHI_PYROLYTIC_GRAPHITE", "FLOAT_PAYLOAD_KG_M2", "pressure"]

#: Pyrolytic-graphite volume susceptibility perpendicular to the basal plane, SI
#: (frontier research §3g [REF-MEM]; exotic critique XCRIT §6 "χ = −4.5e-4, OK";
#: FRONTIER §F9).
CHI_PYROLYTIC_GRAPHITE = -4.5e-4
#: Reference field, T (FRONTIER FAB-FRN-009: 179 Pa at 1 T; FRONTIER §F9; PHYSICS §2).
B_REFERENCE_T = 1.0


def pressure(
    b_t: float | np.ndarray = B_REFERENCE_T,
    chi: float = CHI_PYROLYTIC_GRAPHITE,
    g_body_mps2: float = G_MOON_MPS2,
    payload_kg_m2: float = FLOAT_PAYLOAD_KG_M2,
) -> FrontierRecord:
    """Diamagnetic pressure abs(χ)·B²/2μ0 and the FLOAT consistency check (FRONTIER §F9).

    Args:
        b_t: field(s), T.
        chi: volume susceptibility (negative for a diamagnet).
        g_body_mps2: gravity of the body the payload sits on (default: the Moon).
        payload_kg_m2: claimed payload to check (default: FLOAT's 33 kg/m²).

    Returns:
        A record with ``p_pa`` (the upper bound), ``gross_kg_m2`` (p/g on the body),
        ``gross_earth_kg_m2``, ``float_consistent`` (payload ≤ gross), ``ratio_to_ems``
        (abs(χ)) and ``upper_bound`` (always True).
    """
    b = np.asarray(b_t, dtype=float)
    if np.any(b < 0.0) or not math.isfinite(chi) or g_body_mps2 <= 0.0 or payload_kg_m2 < 0.0:
        raise ValueError("b_t >= 0, finite chi, g_body_mps2 > 0 and payload_kg_m2 >= 0 required")
    p = abs(chi) * magnetic_pressure_pa(b)
    p_out = float(p) if p.ndim == 0 else p
    gross = p / g_body_mps2
    gross_earth = p / G0_MPS2
    consistent = np.all(gross >= payload_kg_m2)
    p_ref = abs(chi) * magnetic_pressure_pa(B_REFERENCE_T)
    return make_record(
        "diamagnetic",
        "diamagnetic.pressure",
        orders(1.0 / abs(chi)) if chi != 0.0 else 0.0,
        (
            f"Diamagnetic levitation: {p_ref:.0f} Pa at {B_REFERENCE_T:g} T as an upper bound (abs(χ) = "
            f"{abs(chi):.2g}); FLOAT's {payload_kg_m2:.0f} kg/m² fits lunar gravity only; "
            f"{math.log10(1.0 / abs(chi)) if chi else 0.0:.1f} orders below an EMS rail at the same field."
        ),
        {"b_t": b_t, "chi": chi, "g_body_mps2": g_body_mps2, "payload_kg_m2": payload_kg_m2},
        {
            "p_pa": p_out,
            "gross_kg_m2": float(gross) if gross.ndim == 0 else gross,
            "gross_earth_kg_m2": float(gross_earth) if gross_earth.ndim == 0 else gross_earth,
            "float_consistent": bool(consistent),
            "float_consistent_on_earth": bool(np.all(gross_earth >= payload_kg_m2)),
            "ratio_to_ems": abs(chi),
            "upper_bound": True,
        },
    )
