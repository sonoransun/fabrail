"""🧪 Trapped-field bulk magnets: magnetic pressure capped by the rail and by the bulk's strength.

FAB-FRN-002; FRONTIER §F3. Research tier (relabelled from 🔭): bulk superconductors
trapping several tesla are demonstrated (17.6 T in a reinforced bulk pair at 26 K,
Durrell et al. 2014 [REF-MEM]); what they cannot do is push a rail harder than the
rail's own saturation allows.

**Model** (exotic design §2.3 TrappedFieldBulk; EXOTIC §X2; PHYSICS §2)::

    p_raw  = η·B²/(2μ0)                                  η: fraction of B² reaching the gap
    B_mech = √(2μ0·σ_mech)                               the bulk cracks when B²/2μ0 > σ_mech
    p      = min(η·min(B, B_mech)²/(2μ0),  B_rail²/(2μ0))

B_mech bounds the field a bulk of tensile strength σ_mech (30–100 MPa [UNCERTAIN])
survives trapping, since the magnetisation hoop stress scales as B²/2μ0. B_rail is the
saturation of the steel it pulls on. The default is the envelope of practical
soft-magnetic steels, FeCo at 2.3 T, which gives 2.10 MPa (FAB-EXO-207); the exotic-tier
rails sit below it (structural steel 1.6 T → 1.02 MPa, FeCo-lined 2.15 T → 1.84 MPa;
fabrail-exo-sim suspension). Whatever B_trapped is, the pressure against a steel rail
stays at or below 2.1 MPa.

Missing orders: 0; the cap is physics, not a missing capability.

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from ._const import MU0_H_PER_M, magnetic_pressure_pa
from ._record import FrontierRecord, make_record

__all__ = [
    "B_RAIL_FECO_LINED_T",
    "B_RAIL_SAT_T",
    "B_RAIL_STRUCTURAL_T",
    "B_TRAPPED_RECORD_T",
    "B_TRAPPED_SWEEP_T",
    "SIGMA_MECH_PA",
    "pressure_cap",
]

#: Saturation envelope of practical soft-magnetic steel (FeCo), T (EXOTIC §X2,
#: FAB-EXO-207: P(2.3 T) = 2.10 MPa; fabrail-exo-sim suspension B_SAT_FECO_T; PHYSICS §2).
B_RAIL_SAT_T = 2.3
#: The exotic-tier rails, T: structural steel 1.6 T and FeCo-lined 2.15 T (exotic
#: critique; fabrail-exo-sim suspension B_RAIL_STEEL_T, B_RAIL_FECO_T; EXOTIC §X2).
B_RAIL_STRUCTURAL_T = 1.6
B_RAIL_FECO_LINED_T = 2.15
#: Bulk tensile limit range, Pa (exotic design §2.3: σ_mech ≈ 30–100 MPa [UNCERTAIN]; FRONTIER §F3).
SIGMA_MECH_PA: tuple[float, float] = (30e6, 100e6)
#: Trapped-field sweep, T (exotic design §2.3: B 3–7 T; FRONTIER §F3).
B_TRAPPED_SWEEP_T: tuple[float, float] = (3.0, 7.0)
#: Record trapped field, T (Durrell et al. 2014, 26 K, shrink-fit reinforced [REF-MEM]; FRONTIER §F3).
B_TRAPPED_RECORD_T = 17.6


def pressure_cap(
    b_t: Sequence[float] | np.ndarray | None = None,
    eta: Sequence[float] | np.ndarray | None = None,
    b_rail_t: float = B_RAIL_SAT_T,
    sigma_mech_pa: float = SIGMA_MECH_PA[0],
) -> FrontierRecord:
    """Trapped-field pressure over B × η, with the rail and σ_mech caps (FRONTIER §F3).

    Args:
        b_t: trapped fields, T (default 3 … 7 T in 0.5 T steps).
        eta: gap fractions of B² (default 0.1, 0.3, 0.6, 1.0).
        b_rail_t: rail saturation, T.
        sigma_mech_pa: bulk tensile limit, Pa.

    Returns:
        A record with ``p_raw_pa`` and ``p_pa`` shaped (n_eta, n_b), ``p_rail_pa``,
        ``b_mech_t``, ``binding`` ("rail", "sigma_mech" or "none" per cell) and
        ``rail_capped`` (every p ≤ the rail cap).
    """
    b = np.arange(3.0, 7.01, 0.5) if b_t is None else np.asarray(b_t, dtype=float).reshape(-1)
    e = np.array([0.1, 0.3, 0.6, 1.0]) if eta is None else np.asarray(eta, dtype=float).reshape(-1)
    if np.any(b <= 0.0) or np.any(e <= 0.0) or np.any(e > 1.0) or b_rail_t <= 0.0 or sigma_mech_pa <= 0.0:
        raise ValueError("b_t, b_rail_t and sigma_mech_pa must be > 0 and eta in (0, 1]")
    b_mech = math.sqrt(2.0 * MU0_H_PER_M * sigma_mech_pa)
    p_rail = float(magnetic_pressure_pa(b_rail_t))
    p_raw = e[:, None] * magnetic_pressure_pa(b)[None, :]
    p_mech = e[:, None] * magnetic_pressure_pa(np.minimum(b, b_mech))[None, :]
    p = np.minimum(p_mech, p_rail)
    binding = np.where(p_mech >= p_rail, "rail", np.where(b[None, :] > b_mech, "sigma_mech", "none"))
    return make_record(
        "trapped_field",
        "trapped_field.pressure_cap",
        0.0,
        (
            f"Trapped-field bulk magnets: research tier (relabelled from 🔭); pressure capped by rail "
            f"saturation (≤ {p_rail / 1e6:.1f} MPa against steel at {b_rail_t:g} T) and by σ_mech "
            f"({sigma_mech_pa / 1e6:.0f} MPa → B ≤ {b_mech:.1f} T) whatever B_trapped."
        ),
        {"b_t": b, "eta": e, "b_rail_t": b_rail_t, "sigma_mech_pa": sigma_mech_pa},
        {
            "p_raw_pa": p_raw,
            "p_pa": p,
            "p_rail_pa": p_rail,
            "p_structural_rail_pa": float(magnetic_pressure_pa(B_RAIL_STRUCTURAL_T)),
            "p_feco_lined_rail_pa": float(magnetic_pressure_pa(B_RAIL_FECO_LINED_T)),
            "b_mech_t": b_mech,
            "binding": binding,
            "p_max_pa": float(p.max()),
            "rail_capped": bool(np.all(p <= p_rail)),
        },
    )
