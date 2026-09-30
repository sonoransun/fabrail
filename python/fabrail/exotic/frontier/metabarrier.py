"""🔭 Building-scale active maglev metabarrier: the resonating mass it would take.

FAB-FRN-004; FRONTIER §F5 (anchor tier 🧪). Seismic metamaterials are field-tested only
at ≥ 30–50 Hz (Brûlé et al. 2014 PRL, boreholes at 50 Hz; Colombi et al. 2016 Sci Rep,
forests at 30–150 Hz); a scaled test (Palermo et al. 2016 Sci Rep) gave up to 50 %
reduction below 10 Hz; there is no field test below 10 Hz.

**Lumped-resonator Rayleigh metasurface** (frontier research §3b, an adaptation of
Colquitt et al. 2017 eq. 3.28–3.29, not a published equation [UNCERTAIN]; PHYSICS §19)::

    L²·ρ·v_s·(x² − 1) = √(1 − r²)·m·ω_r·x,   x = ω/ω_r,  r = v_s/v_p
    Γ   = √(1 − r²)·m·ω_r/(L²·ρ·v_s)
    x_u = (Γ + √(Γ² + 4))/2 ≈ 1 + Γ/2        (upper gap edge; the gap is [f_r, x_u·f_r])

**Mass law.** For a fixed relative gap (Γ ≈ 0.2) the areal mass m/L² ∝ ρ·v_s/f, and a
barrier about one shear wavelength λ = v_s/f wide around a site has an area ∝ λ², so the
resonating mass scales as ρλ³ = ρ·v_s³/f³ (exotic critique XCRIT §1): 1.8 Mt at 1 Hz
and v_s = 100 m/s, 14.4 Mt at 200 m/s. Every output states v_s. Active LSM units can
retune the resonance in seconds but cannot create momentum capacity: the mass is the
limit, and a pure-LSM spring at 1 Hz would need about 79 kN per 5 t unit
(:func:`unit_budget`), hence a hybrid spring plus LSM trim.

Missing orders: 3·log10(30 Hz / f), the resonating-mass ratio between the lowest field-
tested frequency and f under the f⁻³ law (4.4 at 1 Hz).

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from ._record import FrontierRecord, make_record, orders

__all__ = [
    "F_FIELD_TEST_MIN_HZ",
    "F_REFERENCE_HZ",
    "GAMMA_TARGET",
    "RHO_SOFT_KG_M3",
    "UNIT_MASS_KG",
    "VP_OVER_VS",
    "V_S_REFERENCE_MPS",
    "gap_edge",
    "mass_law",
    "sweep",
    "unit_budget",
]

#: Soft-soil density, kg/m³ (frontier research §3b table, "soft (150, 1800)"; with
#: v_s = 100 m/s it gives XCRIT's 1.8 Mt; FRONTIER §F5).
RHO_SOFT_KG_M3 = 1800.0
#: Reference shear speeds, m/s (exotic critique XCRIT §1: 1.8 Mt at 100 m/s, 14.4 Mt at
#: 200 m/s; FRONTIER FAB-FRN-004; FRONTIER §F5).
V_S_REFERENCE_MPS: tuple[float, float] = (100.0, 200.0)
#: Reference resonance, Hz (FRONTIER §1: "ρλ³ at 1 Hz"; FRONTIER §F5).
F_REFERENCE_HZ = 1.0
#: v_p/v_s of the soil: 2 (Poisson ratio 1/3), which reproduces the frontier research §3b
#: areal masses (5.7 t/m² Mexico City clay at 0.5 Hz, 9.9 t/m² soft soil at 1 Hz) [calc] (FRONTIER §F5; PHYSICS §19).
VP_OVER_VS = 2.0
#: Γ for about a 10 % gap (frontier research §3b; FRONTIER §F5).
GAMMA_TARGET = 0.2
#: Lowest field-tested metabarrier frequency, Hz (Colombi 2016 forests 30–150 Hz; Brûlé
#: 2014 at 50 Hz; frontier research §3b; FRONTIER §F5).
F_FIELD_TEST_MIN_HZ = 30.0
#: Resonator unit mass, kg (frontier research §3b: 5 t units, HEAVY-class carriages; FRONTIER §F5).
UNIT_MASS_KG = 5000.0


def _missing(f_hz: float) -> float:
    """3·log10(f_test/f): the f⁻³ resonating-mass ratio beyond the lowest field test."""
    return 3.0 * orders(F_FIELD_TEST_MIN_HZ / f_hz)


def _note(f_hz: float, rho: float, v_s: Sequence[float]) -> str:
    masses = ", ".join(f"{rho * (v / f_hz) ** 3 / 1e9:.3g} Mt at v_s = {v:g} m/s" for v in v_s)
    return (
        f"Building-scale active metabarrier: ρλ³ at {f_hz:g} Hz, resonating mass {masses} "
        f"(ρ = {rho:g} kg/m³); field tests exist only at ≥ {F_FIELD_TEST_MIN_HZ:.0f} Hz."
    )


def _check_positive(**values: object) -> None:
    for name, value in values.items():
        v = np.asarray(value, dtype=float)
        if v.size == 0 or not np.all(np.isfinite(v)) or np.any(v <= 0.0):
            raise ValueError(f"{name} must be finite and > 0 (got {value!r})")


def gap_edge(
    f_r_hz: float = F_REFERENCE_HZ,
    m_kg: float = UNIT_MASS_KG,
    spacing_m: float = 1.0,
    rho_kg_m3: float = RHO_SOFT_KG_M3,
    v_s_mps: float = V_S_REFERENCE_MPS[0],
    vp_over_vs: float = VP_OVER_VS,
) -> FrontierRecord:
    """Γ and the upper gap edge of a lumped-resonator metasurface (FRONTIER §F5).

    Args:
        f_r_hz: resonator frequency, Hz.
        m_kg: resonator mass, kg.
        spacing_m: resonator spacing L, m.
        rho_kg_m3, v_s_mps: soil density and shear speed.
        vp_over_vs: v_p/v_s (> 1).

    Returns:
        A record with ``gamma``, ``x_upper``, ``f_upper_hz``, ``gap_relative_width``,
        ``areal_mass_kg_m2`` (m/L²) and ``areal_mass_for_target_kg_m2`` (Γ = 0.2).
    """
    _check_positive(f_r_hz=f_r_hz, m_kg=m_kg, spacing_m=spacing_m, rho_kg_m3=rho_kg_m3, v_s_mps=v_s_mps)
    if not vp_over_vs > 1.0:
        raise ValueError(f"vp_over_vs must be > 1 (got {vp_over_vs!r})")
    r = 1.0 / vp_over_vs
    w_r = 2.0 * math.pi * f_r_hz
    root = math.sqrt(1.0 - r * r)
    gamma = root * m_kg * w_r / (spacing_m**2 * rho_kg_m3 * v_s_mps)
    x_u = 0.5 * (gamma + math.sqrt(gamma * gamma + 4.0))
    target = GAMMA_TARGET * rho_kg_m3 * v_s_mps / (root * w_r)
    return make_record(
        "metabarrier",
        "metabarrier.gap_edge",
        _missing(f_r_hz),
        _note(f_r_hz, rho_kg_m3, (v_s_mps,)),
        {
            "f_r_hz": f_r_hz,
            "m_kg": m_kg,
            "spacing_m": spacing_m,
            "rho_kg_m3": rho_kg_m3,
            "v_s_mps": v_s_mps,
            "vp_over_vs": vp_over_vs,
        },
        {
            "gamma": gamma,
            "x_upper": x_u,
            "f_upper_hz": x_u * f_r_hz,
            "gap_relative_width": x_u - 1.0,
            "areal_mass_kg_m2": m_kg / spacing_m**2,
            "areal_mass_for_target_kg_m2": target,
            "gamma_target": GAMMA_TARGET,
        },
    )


def mass_law(
    f_hz: float = F_REFERENCE_HZ,
    v_s_mps: Sequence[float] | float = V_S_REFERENCE_MPS,
    rho_kg_m3: float = RHO_SOFT_KG_M3,
    unit_mass_kg: float = UNIT_MASS_KG,
) -> FrontierRecord:
    """Resonating mass M = ρ·λ³ = ρ·v_s³/f³ of a building-scale barrier (FRONTIER §F5).

    Args:
        f_hz: barrier frequency, Hz.
        v_s_mps: soil shear speed(s), m/s (always stated in the output).
        rho_kg_m3: soil density, kg/m³.
        unit_mass_kg: resonator unit mass, kg (for the unit count).

    Returns:
        A record with ``lambda_m``, ``mass_kg``, ``mass_mt`` and ``units`` per v_s.
    """
    v = np.asarray(v_s_mps, dtype=float).reshape(-1)
    _check_positive(f_hz=f_hz, v_s_mps=v, rho_kg_m3=rho_kg_m3, unit_mass_kg=unit_mass_kg)
    lam = v / f_hz
    mass = rho_kg_m3 * lam**3
    return make_record(
        "metabarrier",
        "metabarrier.mass_law",
        _missing(f_hz),
        _note(f_hz, rho_kg_m3, tuple(float(x) for x in v)),
        {"f_hz": f_hz, "v_s_mps": v, "rho_kg_m3": rho_kg_m3, "unit_mass_kg": unit_mass_kg},
        {
            "lambda_m": lam,
            "mass_kg": mass,
            "mass_mt": mass / 1e9,
            "units": mass / unit_mass_kg,
        },
    )


def sweep(
    f_hz: Sequence[float] | np.ndarray | None = None,
    v_s_mps: Sequence[float] | np.ndarray | None = None,
    rho_kg_m3: float = RHO_SOFT_KG_M3,
) -> FrontierRecord:
    """The mass law over f × v_s (default f 0.5–10 Hz, v_s 60–400 m/s; FRONTIER §F5).

    Returns:
        A record with ``mass_kg`` shaped (n_f, n_vs) and ``missing_orders_by_f``.
    """
    f = np.array([0.5, 1.0, 2.0, 5.0, 10.0]) if f_hz is None else np.asarray(f_hz, dtype=float).reshape(-1)
    v = (
        np.array([60.0, 100.0, 150.0, 200.0, 400.0])
        if v_s_mps is None
        else np.asarray(v_s_mps, dtype=float).reshape(-1)
    )
    _check_positive(f_hz=f, v_s_mps=v, rho_kg_m3=rho_kg_m3)
    mass = rho_kg_m3 * (v[None, :] / f[:, None]) ** 3
    f_ref = float(f.min())
    return make_record(
        "metabarrier",
        "metabarrier.sweep",
        _missing(f_ref),
        _note(f_ref, rho_kg_m3, (float(v.min()), float(v.max()))),
        {"f_hz": f, "v_s_mps": v, "rho_kg_m3": rho_kg_m3},
        {
            "mass_kg": mass,
            "missing_orders_by_f": np.array([_missing(float(x)) for x in f]),
        },
    )


def unit_budget(
    pgv_mps: float = 0.5,
    f_hz: float = F_REFERENCE_HZ,
    zeta: float = 0.1,
    m_kg: float = UNIT_MASS_KG,
) -> FrontierRecord:
    """Stroke and spring force of one resonator unit (frontier research §3b; FRONTIER §F5).

    u_g = PGV/ω (harmonic ground displacement), stroke ≈ u_g/(2ζ) at resonance, and a
    pure-LSM spring force m·ω²·stroke (79 kN per 5 t unit at 1 Hz, PGV 0.5 m/s, ζ 0.1).

    Returns:
        A record with ``ground_disp_m``, ``stroke_m`` and ``spring_force_n``.
    """
    _check_positive(pgv_mps=pgv_mps, f_hz=f_hz, zeta=zeta, m_kg=m_kg)
    w = 2.0 * math.pi * f_hz
    u_g = pgv_mps / w
    stroke = u_g / (2.0 * zeta)
    return make_record(
        "metabarrier",
        "metabarrier.unit_budget",
        _missing(f_hz),
        _note(f_hz, RHO_SOFT_KG_M3, V_S_REFERENCE_MPS),
        {"pgv_mps": pgv_mps, "f_hz": f_hz, "zeta": zeta, "m_kg": m_kg},
        {
            "ground_disp_m": u_g,
            "stroke_m": stroke,
            "spring_force_n": m_kg * w * w * stroke,
        },
    )
