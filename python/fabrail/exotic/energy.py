"""Ride-through and storage calculator (EXOTIC §X4, §X5; FAB-EXO-120, FAB-EXO-122, FAB-EXO-401..405).

    T_ride = max(300 s, t_shaking of the class scenario) + t_land      (``fabrail_exo::energy::t_ride_s``)
    E_ride = P_hold · T_ride                                         (``ride_through_energy_j``)
    hybrid EMS hover loss P_hold = P_W·(a_rms/2g)²,  P_W = 1.7 kW/t × m
    pure EMS P_hold = P_W (Transrapid, levitation and guidance)
    HTS: sensors and propulsion only (kW-scale); the LN₂ reservoir is the buffer

Discharge admission needs E_usable ≥ 2·(E_stop + E_land + P_hold·T_ride); absorption
needs charge headroom + dump or eddy-brake rating ≥ 1.2·KE_max, and a failing class is
reported, not hidden (ROTATOR today). Storage benchmarks: an EMALS-class disk delivers
121 MJ per launch (6400 → 5205 rpm, so ≈ 357 MJ stored); a JET-class flywheel stores
3.75 GJ (≈ 2.8 GJ usable at ω_min = ω_max/2). Budget does not buy reliability growth
(EMALS: 614 against a 4,166-cycle requirement).

Every result carries its inputs.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ._util import G0_MPS2, native

__all__ = [
    "ABSORB_MARGIN",
    "DISCHARGE_MARGIN",
    "EMALS_DELIVERED_J",
    "EMALS_RPM_AFTER_LAUNCH",
    "EMALS_RPM_MAX",
    "JET_STORED_J",
    "PURE_EMS_HOVER_W_PER_KG",
    "RideThrough",
    "absorption_check",
    "class_ride_through",
    "emals_stored_j",
    "flywheel_usable_fraction",
    "hover_power_w",
    "jet_usable_j",
    "ride_through",
    "ride_through_table",
    "t_ride_s",
]

#: Pure-EMS hover power, W/kg (1.7 kW/t, Transrapid; ``suspension::PURE_EMS_HOVER_W_PER_KG``).
PURE_EMS_HOVER_W_PER_KG = 1.7
#: Discharge margin of the admission (``fabrail_exo::energy::DISCHARGE_MARGIN``).
DISCHARGE_MARGIN = 2.0
#: Absorption margin on KE_max (``fabrail_exo::energy::ABSORB_MARGIN``).
ABSORB_MARGIN = 1.2
#: EMALS-class disk: energy delivered per launch, J (XCRIT P1-11).
EMALS_DELIVERED_J = 121.0e6
#: EMALS disk speeds, rpm (``storage::EMALS_RPM_MAX`` / ``EMALS_RPM_AFTER_LAUNCH``).
EMALS_RPM_MAX = 6400.0
EMALS_RPM_AFTER_LAUNCH = 5205.0
#: JET flywheel stored energy, J (``storage::JET_STORED_J``).
JET_STORED_J = 3.75e9


def t_ride_s(t_shaking_s: float, t_land_s: float = 0.0) -> float:
    """T_ride = max(300 s, t_shaking) + t_land, s (Rust ``energy::t_ride_s``; FAB-EXO-120)."""
    return float(native().exo_t_ride(float(t_shaking_s), float(t_land_s)))


def flywheel_usable_fraction(omega_min_over_max: float) -> float:
    """1 − (ω_min/ω_max)²: 75 % at ω_min = ω_max/2 (Rust ``storage``; FAB-EXO-401)."""
    return float(native().exo_flywheel_usable_fraction(float(omega_min_over_max)))


def emals_stored_j() -> float:
    """Stored energy of an EMALS-class disk from 121 MJ delivered between 6400 and 5205 rpm, J (≈ 357 MJ)."""
    return EMALS_DELIVERED_J / flywheel_usable_fraction(EMALS_RPM_AFTER_LAUNCH / EMALS_RPM_MAX)


def jet_usable_j(omega_min_over_max: float = 0.5) -> float:
    """Usable energy of one JET-class flywheel between ω_max and ω_min, J (≈ 2.8 GJ at ω_max/2)."""
    return JET_STORED_J * flywheel_usable_fraction(omega_min_over_max)


def hover_power_w(m_kg: float, suspension: str = "hybrid", a_rms_g: float = 0.2, hts_kw: float = 5.0) -> float:
    """Hold power during a ride-through, W (EXOTIC §X4; FAB-EXO-206, FAB-EXO-405).

    Args:
        m_kg: suspended mass.
        suspension: ``hybrid`` (P_W·(a_rms/2g)², zero-power: the coils carry only m·a),
            ``pure`` (P_W = 1.7 kW/t, Transrapid) or ``hts`` (sensors and propulsion only,
            ``hts_kw`` [UNCERTAIN: kW-scale]).
        a_rms_g: RMS vertical acceleration at the suspension, g (not the ground PGA).
    """
    if not (math.isfinite(m_kg) and m_kg > 0):
        raise ValueError(f"mass must be finite and > 0 (got {m_kg!r})")
    p_w = PURE_EMS_HOVER_W_PER_KG * m_kg
    if suspension == "hybrid":
        return p_w * (a_rms_g / 2.0) ** 2
    if suspension == "pure":
        return p_w
    if suspension == "hts":
        return hts_kw * 1e3
    raise ValueError(f"suspension must be 'hybrid', 'pure' or 'hts' (got {suspension!r})")


@dataclass(frozen=True)
class RideThrough:
    """Ride-through energy of one case, with the verdict against the storage benchmarks."""

    m_kg: float
    suspension: str
    a_rms_g: float
    p_hold_w: float
    t_ride_s: float
    e_ride_j: float
    e_with_margin_j: float
    verdict: str
    inputs: dict[str, float | str] = field(default_factory=dict)


def ride_through(
    m_kg: float,
    suspension: str = "hybrid",
    *,
    a_rms_g: float = 0.2,
    t_shaking_s: float = 300.0,
    t_land_s: float = 0.0,
    margin: float = DISCHARGE_MARGIN,
    hts_kw: float = 5.0,
) -> RideThrough:
    """Ride-through energy E = P_hold·T_ride and its verdict (EXOTIC §X4; FAB-EXO-403).

    10 kt hybrid EMS at 0.2 g RMS for 300 s: 170 kW, 51 MJ (102 MJ with the 2× margin),
    within one EMALS-class disk's 121 MJ per cycle; pure EMS: 17 MW, 5.1 GJ, flagged
    insufficient against one JET-class flywheel (≈ 2.8 GJ usable).
    """
    p = hover_power_w(m_kg, suspension, a_rms_g, hts_kw)
    t = t_ride_s(t_shaking_s, t_land_s)
    e = float(native().exo_ride_through_energy(p, t))
    em = margin * e
    if em <= EMALS_DELIVERED_J:
        verdict = "within one EMALS-class disk's 121 MJ per cycle"
    elif em <= jet_usable_j():
        verdict = "needs more than one EMALS-class disk; within one JET-class flywheel's usable energy"
    else:
        verdict = "flagged insufficient against one JET-class flywheel (usable energy below the margin)"
    inputs: dict[str, float | str] = {
        "m_kg": float(m_kg),
        "suspension": suspension,
        "a_rms_g": float(a_rms_g),
        "t_shaking_s": float(t_shaking_s),
        "t_land_s": float(t_land_s),
        "margin": float(margin),
        "g_mps2": G0_MPS2,
        "p_w_per_kg_pure": PURE_EMS_HOVER_W_PER_KG,
    }
    if suspension == "hts":
        inputs["hts_kw"] = float(hts_kw)
    return RideThrough(float(m_kg), suspension, float(a_rms_g), p, t, e, em, verdict, inputs)


def ride_through_table(m_kg: float = 1.0e7, t_shaking_s: float = 300.0) -> list[RideThrough]:
    """The EXOTIC §X4 table: hybrid EMS at 0.1/0.2/0.5 g, pure EMS and HTS for ``m_kg`` over T_ride."""
    rows = [ride_through(m_kg, "hybrid", a_rms_g=a, t_shaking_s=t_shaking_s) for a in (0.1, 0.2, 0.5)]
    rows.append(ride_through(m_kg, "pure", t_shaking_s=t_shaking_s))
    rows.append(ride_through(m_kg, "hts", t_shaking_s=t_shaking_s))
    return rows


def class_ride_through(a_rms_g: float = 0.2, t_shaking_s: float = 300.0) -> list[dict[str, object]]:
    """Hybrid-EMS ride-through energy per exotic class at its design mass (SELENE: HTS)."""
    from .presets import all_presets

    out: list[dict[str, object]] = []
    for p in all_presets():
        kind = "hts" if p.suspension == "HtsBulk" else "hybrid"
        r = ride_through(p.design_mass_kg, kind, a_rms_g=a_rms_g, t_shaking_s=t_shaking_s)
        out.append({"class": p.name, "suspension": kind, "p_hold_w": r.p_hold_w, "t_ride_s": r.t_ride_s, "e_ride_j": r.e_ride_j, "e_with_margin_j": r.e_with_margin_j, "verdict": r.verdict})
    return out


def absorption_check(m_rated_kg: float, v_max_mps: float, headroom_j: float, dump_j: float = 0.0) -> dict[str, float | bool]:
    """Absorption admission: charge headroom + dump rating ≥ 1.2·KE_max (FAB-EXO-122).

    KE_max = ½·m_rated·v_max² (an upper bound for a ring, I ≤ m·r²). ROTATOR (6,100 t at a
    1.5 m/s rim, 6.86 MJ) against an empty 2 kWh buffer with no dump: 1.05×, fails, and is
    reported.
    """
    ke = 0.5 * m_rated_kg * v_max_mps**2
    cap = headroom_j + dump_j
    return {
        "ke_max_j": ke,
        "capacity_j": cap,
        "ratio": cap / ke if ke > 0 else math.inf,
        "required_ratio": ABSORB_MARGIN,
        "pass": cap >= ABSORB_MARGIN * ke,
        "shortfall_j": max(0.0, ABSORB_MARGIN * ke - cap),
        "m_rated_kg": float(m_rated_kg),
        "v_max_mps": float(v_max_mps),
        "headroom_j": float(headroom_j),
        "dump_j": float(dump_j),
    }
