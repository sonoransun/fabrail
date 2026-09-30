"""Exotic class algebra: admission, HTS area, kinematics, isolation demand, ISO 13855, stray field.

Everything here is a closed form over the class presets (:mod:`fabrail.exotic.presets`)
or a thin wrapper over the Rust single implementation in ``fabrail-exo-sim`` /
``fabrail-exo-safety`` (called through ``fabrail._native``). Each function names its
EXOTIC section and requirement.

- **Admission** (EXOTIC §X2; FAB-EXO-103): the one admission function
  A = k·m·g / (η_fill·min(B_pole, B_rail, B_gap,max)²/(2·μ0)) from
  ``suspension::admission_table``; :func:`render_admission_table` gives the generated
  EXOTIC §X2 block, and :func:`regenerate_exotic_md` rewrites both generated blocks
  (FAB-EXO-121).
- **HTS area** (EXOTIC §X3): ``suspension::hts_area_table`` at 53 kPa at 10 mm.
- **Kinematics** (EXOTIC §X7, §X9; FAB-EXO-110): v = ω·S, a = ω²·S, j = ω³·S; the
  ASCE 7-16 Ch. 17 displacement S_D = S_D1·g·T/(4π²·B) and the platform peak
  a_peak = ω²·S_D·√(1 + 4ζ²), an analysis output, never a trip.
- **ISO 13855** (EXOTIC §X14; FAB-EXO-502): S = K·(t1 + v/a_brake) + C.
- **Stray field** (FAB-EXO-104; PHYSICS §23 method): a bare-magnet estimate at the
  accessible surface with a shroud flag. Iron return paths are not modelled, so it is
  an upper-bound-style estimate, not a measured or FEM value.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from ._util import G0_MPS2, MU0_H_PER_M, fmt_sig, fmt_thousands, native, pedestrian_caps
from .presets import ExoPreset, all_presets, preset

__all__ = [
    "ADMISSION_K",
    "ADMISSION_MARKERS",
    "ASCE_B_TABLE",
    "AdmissionRow",
    "B_GAP_MAX_DEFAULT_T",
    "B_RAIL_STEEL_T",
    "ETA_FILL_DEFAULT",
    "HTS_AREA_MARKERS",
    "HavenDemand",
    "HtsAreaRow",
    "Kinematics",
    "StrayFieldRow",
    "admission_closed_form",
    "admission_table",
    "asce_b_factor",
    "attended_jog",
    "bumper_ratings",
    "doc_block",
    "guideway_geometry",
    "haven_demand",
    "hts_area_table",
    "iso13855_distances",
    "isolation_transmissibility",
    "kinematics",
    "pole_saturation_t",
    "prism_on_axis_t",
    "dipole_on_axis_t",
    "regenerate_exotic_md",
    "render_admission_table",
    "render_hts_area_table",
    "stray_field_report",
]

# ---- admission (EXOTIC §X2) ----------------------------------------------------------------

#: Force headroom over the static weight (``suspension::ADMISSION_K``; ET §2.4).
ADMISSION_K = 1.5
#: Default active-pole fill fraction η_fill (``suspension::ETA_FILL_DEFAULT``) [UNCERTAIN].
ETA_FILL_DEFAULT = 0.45
#: Structural-steel rail saturation, T (``suspension::B_RAIL_STEEL_T``; XCRIT §1).
B_RAIL_STEEL_T = 1.6
#: Default PM-biased gap flux ceiling, T (``suspension::B_GAP_MAX_DEFAULT_T``) [UNCERTAIN].
B_GAP_MAX_DEFAULT_T = 1.2

#: Pole saturation by pole material, T (``suspension::B_SAT_FE_T`` / ``B_SAT_FECO_T``).
_B_SAT_POLE_T: dict[str, float] = {"Fe": 1.5, "FeCo": 2.3}

#: The EXOTIC.md generated-block markers (FAB-EXO-121).
ADMISSION_MARKERS = ("<!-- EXO_ADMISSION:BEGIN", "<!-- EXO_ADMISSION:END -->")
HTS_AREA_MARKERS = ("<!-- EXO_HTS_AREA:BEGIN", "<!-- EXO_HTS_AREA:END -->")

_SUSPENSION_TEXT = {"HybridEms": "hybrid EMS", "PureEms": "pure EMS", "HtsBulk": "HTS bulk", "HybridHtsPlusEms": "HTS + EMS trim", "AirPad": "air pad"}


def pole_saturation_t(pole: str) -> float | None:
    """Pole saturation flux density for a pole material (``Fe`` 1.5 T, ``FeCo`` 2.3 T), None without iron poles."""
    return _B_SAT_POLE_T.get(pole)


@dataclass(frozen=True)
class AdmissionRow:
    """One row of the admission table (``suspension::AdmissionRow``)."""

    name: str
    display: str
    suspension: str
    pole: str
    m_kg: float
    b_eff_t: float
    p_eff_pa: float
    area_m2: float


def admission_closed_form(
    m_kg: float,
    b_pole_t: float,
    *,
    eta_fill: float = ETA_FILL_DEFAULT,
    b_rail_t: float = B_RAIL_STEEL_T,
    b_gap_max_t: float = B_GAP_MAX_DEFAULT_T,
    k: float = ADMISSION_K,
) -> float:
    """A = k·m·g / (η_fill·min(B_pole, B_rail, B_gap,max)²/(2·μ0)), m² (EXOTIC §X2; restated for the cross-check)."""
    b = min(b_pole_t, b_rail_t, b_gap_max_t)
    return k * m_kg * G0_MPS2 / (eta_fill * b * b / (2.0 * MU0_H_PER_M))


def admission_table(
    eta_fill: float = ETA_FILL_DEFAULT, b_rail_t: float = B_RAIL_STEEL_T, b_gap_max_t: float = B_GAP_MAX_DEFAULT_T
) -> list[AdmissionRow]:
    """The per-class admission table from ``suspension::admission_table`` (FAB-EXO-103).

    HTS rows use the ATZ pressure of 53 kPa at 10 mm under Earth gravity without k
    (EXOTIC §X2); SELENE's lunar numbers are in FRONTIER §F7.
    """
    rows = native().exo_admission_table(eta_fill, b_rail_t, b_gap_max_t)
    out = []
    for r in rows:
        p = preset(str(r["class"]))
        out.append(
            AdmissionRow(
                name=p.name,
                display=p.display,
                suspension=str(r["suspension"]),
                pole=p.pole,
                m_kg=float(r["m_kg"]),
                b_eff_t=float(r["b_eff_t"]),
                p_eff_pa=float(r["p_eff_pa"]),
                area_m2=float(r["area_m2"]),
            )
        )
    return out


def _mass_t(m_kg: float) -> str:
    return f"{fmt_thousands(m_kg / 1000.0)} t"


def _pressure(p_pa: float) -> str:
    return f"{fmt_sig(p_pa / 1e6)} MPa" if p_pa >= 1e6 else f"{fmt_sig(p_pa / 1e3)} kPa"


def _area(a_m2: float) -> str:
    return f"{fmt_sig(a_m2)} m²"


def render_admission_table(rows: list[AdmissionRow] | None = None) -> str:
    """The generated EXOTIC §X2 admission table (Markdown, no markers; FAB-EXO-121)."""
    rows = admission_table() if rows is None else rows
    lines = [
        "| Class | Suspension | Pole | Design mass | Governing B | Effective pressure | Area |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        pole = "—" if r.pole in ("None", "") else r.pole
        b = "—" if r.b_eff_t <= 0.0 else f"{r.b_eff_t:.2f} T"
        cells = (r.display, _SUSPENSION_TEXT.get(r.suspension, r.suspension), pole, _mass_t(r.m_kg), b, _pressure(r.p_eff_pa), _area(r.area_m2))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


# ---- HTS area (EXOTIC §X3) -----------------------------------------------------------------


@dataclass(frozen=True)
class HtsAreaRow:
    """One row of the HTS area and logistics table (``suspension::hts_area_table``)."""

    name: str
    display: str
    m_kg: float
    area_m2: float
    cryostats: int
    heat_w: float
    ln2_kg_per_day: float
    ems_area_m2: float


def hts_area_table() -> list[HtsAreaRow]:
    """HTS bulk area at 10 mm, cryostat count, 77 K heat and LN₂ boil-off per class (EXOTIC §X3)."""
    out = []
    for r in native().exo_hts_area_table():
        p = preset(str(r["class"]))
        out.append(
            HtsAreaRow(
                name=p.name,
                display=p.display,
                m_kg=float(r["m_kg"]),
                area_m2=float(r["area_m2"]),
                cryostats=int(round(float(r["cryostats"]))),
                heat_w=float(r["heat_w"]),
                ln2_kg_per_day=float(r["ln2_kg_per_day"]),
                ems_area_m2=float(r["ems_area_m2"]),
            )
        )
    return out


def _power(w: float) -> str:
    if w >= 1e6:
        return f"{fmt_sig(w / 1e6)} MW"
    if w >= 1e3:
        return f"{fmt_sig(w / 1e3)} kW"
    return f"{fmt_sig(w)} W"


def _boil_off(kg_per_day: float) -> str:
    return f"{fmt_sig(kg_per_day / 1e3)} t/day" if kg_per_day >= 1e3 else f"{fmt_sig(kg_per_day)} kg/day"


def render_hts_area_table(rows: list[HtsAreaRow] | None = None) -> str:
    """The generated EXOTIC §X3 HTS area table (Markdown, no markers; FAB-EXO-121)."""
    rows = hts_area_table() if rows is None else rows
    lines = [
        "| Class | Mass | Bulk area at 10 mm | Cryostats | Heat at 77 K | LN₂ boil-off | EMS area at 0.3 MPa |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        cells = (
            r.display,
            _mass_t(r.m_kg),
            _area(r.area_m2),
            fmt_thousands(r.cryostats),
            _power(r.heat_w),
            _boil_off(r.ln2_kg_per_day),
            _area(r.ems_area_m2),
        )
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def doc_block(text: str, markers: tuple[str, str]) -> str | None:
    """The content between a generated block's markers in ``text`` (None when the markers are absent)."""
    begin, end = markers
    i = text.find(begin)
    if i < 0:
        return None
    j = text.find("\n", i)
    k = text.find(end, j)
    if j < 0 or k < 0:
        return None
    return text[j + 1 : k].strip("\n")


def _replace_block(text: str, markers: tuple[str, str], body: str) -> str:
    begin, end = markers
    i = text.find(begin)
    if i < 0:
        raise ValueError(f"marker {begin!r} not found")
    j = text.find("\n", i)
    k = text.find(end, j)
    if k < 0:
        raise ValueError(f"marker {end!r} not found after {begin!r}")
    return text[: j + 1] + body.rstrip("\n") + "\n" + text[k:]


def regenerate_exotic_md(path: str | Path | None = None, *, write: bool = False) -> str:
    """EXOTIC.md with its generated admission and HTS-area blocks rewritten from code (FAB-EXO-121).

    Args:
        path: the EXOTIC.md to read (default: ``docs/EXOTIC.md`` of the repository).
        write: also write the result back (docs owner / C2a; never hand-edit the blocks).

    Returns:
        The regenerated text.
    """
    if path is None:
        from ..trace import find_root

        path = find_root() / "docs" / "EXOTIC.md"
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    new = _replace_block(text, ADMISSION_MARKERS, render_admission_table())
    new = _replace_block(new, HTS_AREA_MARKERS, render_hts_area_table())
    if write and new != text:
        p.write_text(new, encoding="utf-8")
    return new


# ---- kinematics and isolation demand (EXOTIC §X7, §X9; FAB-EXO-110) ------------------------


@dataclass(frozen=True)
class Kinematics:
    """Kinematic budget of a periodic stroke S at period T: v = ω·S, a = ω²·S, j = ω³·S (EXOTIC §X7)."""

    period_s: float
    stroke_m: float
    omega_rad_s: float
    v_mps: float
    a_mps2: float
    j_mps3: float
    guideway_radius_m: float


def kinematics(period_s: float, stroke_m: float) -> Kinematics:
    """Closed-form kinematic budget at period ``period_s`` and amplitude ``stroke_m`` (FAB-EXO-110).

    SENTINEL at T 7 s, ±1.5 m: 1.346 m/s, 1.209 m/s², 1.085 m/s³; the curved guideway
    radius R = g/ω² = 12.2 m supplies the restoring force passively.
    """
    if not (period_s > 0 and stroke_m > 0):
        raise ValueError(f"period and stroke must be > 0 (got {period_s!r}, {stroke_m!r})")
    w = 2.0 * math.pi / period_s
    return Kinematics(
        period_s=float(period_s),
        stroke_m=float(stroke_m),
        omega_rad_s=w,
        v_mps=w * stroke_m,
        a_mps2=w * w * stroke_m,
        j_mps3=w**3 * stroke_m,
        guideway_radius_m=G0_MPS2 / (w * w),
    )


#: ASCE 7-16 Table 17.5-1 damping coefficient B against effective damping ζ (linear
#: interpolation between rows; EXOTIC §X9 quotes 1.35 at 15 %, 1.5 at 20 %, 1.7 at 30 %).
ASCE_B_TABLE: tuple[tuple[float, float], ...] = ((0.02, 0.8), (0.05, 1.0), (0.10, 1.2), (0.20, 1.5), (0.30, 1.7), (0.40, 1.9), (0.50, 2.0))


def asce_b_factor(zeta: float) -> float:
    """ASCE 7-16 damping coefficient B at effective damping ``zeta`` (interpolated; clamped to the table)."""
    z = float(zeta)
    pts = ASCE_B_TABLE
    if z <= pts[0][0]:
        return pts[0][1]
    for (z0, b0), (z1, b1) in zip(pts, pts[1:]):
        if z <= z1:
            return b0 + (b1 - b0) * (z - z0) / (z1 - z0)
    return pts[-1][1]


@dataclass(frozen=True)
class HavenDemand:
    """Isolation-platform demand at one site level (EXOTIC §X9; FAB-EXO-110, FAB-EXO-505).

    ``a_peak_mps2`` is an analysis output against the occupant target, not a trip.
    """

    s_d1: float
    period_s: float
    zeta: float
    b_factor: float
    s_d_m: float
    a_peak_mps2: float
    a_peak_g: float
    guideway_radius_m: float
    tilt_deg: float


def haven_demand(s_d1: float, period_s: float = 6.0, zeta: float = 0.2, b_factor: float | None = None) -> HavenDemand:
    """S_D = S_D1·g·T/(4π²·B) and a_peak = ω²·S_D·√(1 + 4ζ²) (ASCE 7-16 Ch. 17 form; EXOTIC §X9).

    Args:
        s_d1: the site's 1-s spectral acceleration coefficient (DBE S_D1 or MCE S_M1), g.
        period_s: isolation period T (HAVEN-R: 6 s).
        zeta: effective damping ratio.
        b_factor: damping coefficient B (default :func:`asce_b_factor`).
    """
    b = asce_b_factor(zeta) if b_factor is None else float(b_factor)
    w = 2.0 * math.pi / period_s
    s_d = s_d1 * G0_MPS2 * period_s / (4.0 * math.pi**2 * b)
    a_peak = w * w * s_d * math.sqrt(1.0 + 4.0 * zeta * zeta)
    r = G0_MPS2 / (w * w)
    return HavenDemand(
        s_d1=float(s_d1),
        period_s=float(period_s),
        zeta=float(zeta),
        b_factor=b,
        s_d_m=s_d,
        a_peak_mps2=a_peak,
        a_peak_g=a_peak / G0_MPS2,
        guideway_radius_m=r,
        tilt_deg=math.degrees(math.asin(min(1.0, s_d / r))),
    )


def isolation_transmissibility(f_hz: Any, t_iso_s: float, zeta: float) -> tuple[Any, Any]:
    """Passive and skyhook transmissibility at frequencies ``f_hz`` for isolation period ``t_iso_s`` (EXOTIC §X9).

    r = f·T_iso; passive √[(1 + (2ζr)²)/((1 − r²)² + (2ζr)²)], skyhook 1/√[(1 − r²)² + (2ζr)²],
    evaluated by the native closed forms (``fabrail_params::derive``; PHYSICS §21).
    """
    import numpy as np

    n = native()
    r = np.asarray(f_hz, dtype=float) * float(t_iso_s)
    passive = np.array([n.transmissibility(float(x), float(zeta)) for x in r.ravel()]).reshape(r.shape)
    sky = np.array([n.skyhook_transmissibility(float(x), float(zeta)) for x in r.ravel()]).reshape(r.shape)
    return passive, sky


def guideway_geometry(pole_length_m: float, radius_m: float, gap_m: float, stroke_m: float) -> dict[str, float | bool]:
    """Curved-guideway admission: sagitta L²/(8R) ≤ 10 % of the gap, tilt asin(S/R) ≤ 11° (EXOTIC §X9; FAB-EXO-111)."""
    n = native()
    sag = float(n.exo_sagitta(pole_length_m, radius_m))
    tilt = math.degrees(math.asin(min(1.0, stroke_m / radius_m)))
    return {
        "sagitta_m": sag,
        "sagitta_frac_of_gap": sag / gap_m,
        "sagitta_ok": sag <= 0.1 * gap_m,
        "tilt_deg": tilt,
        "tilt_ok": tilt <= 11.0,
        "period_s": float(n.exo_guideway_period(radius_m)),
    }


# ---- guarded zones (EXOTIC §X14; FAB-EXO-502..504) ---------------------------------------


def iso13855_distances() -> list[dict[str, float | str]]:
    """ISO 13855 separation S = K·(t1 + v/a_brake) + C for every GuardedZone class (FAB-EXO-502).

    Each row carries its inputs (K, t1, C, v_max, a_brake), the native value
    (``exo_iso13855_distance``) and the class preset's stated integer value. SHIELD,
    BASTION and ARENA are the §2 anchors (2.61 / 5.01 / 10.61 m). Checked against numeric
    limits from ISO 13855 (2010) only; no compliance claim.
    """
    n = native()
    out: list[dict[str, float | str]] = []
    for p in all_presets():
        r = p.regime
        if r.kind != "GZ" or r.a_brake_mps2 is None or r.a_brake_mps2 <= 0 or p.v_max_mps <= 0:
            continue
        s = float(n.exo_iso13855_distance(p.v_max_mps, r.a_brake_mps2, r.k_mps, r.t1_s, r.c_m))
        out.append(
            {
                "class": p.name,
                "display": p.display,
                "k_mps": float(r.k_mps or 0.0),
                "t1_s": float(r.t1_s or 0.0),
                "c_m": float(r.c_m or 0.0),
                "v_max_mps": p.v_max_mps,
                "a_brake_mps2": r.a_brake_mps2,
                "t2_s": p.v_max_mps / r.a_brake_mps2,
                "s_m": s,
                "s_preset_m": float(r.iso13855_s_m or float("nan")),
            }
        )
    return out


def bumper_ratings() -> list[dict[str, float | str]]:
    """Bumper rating ≥ ½·m_rated·v_max², always with the rated mass (never m̂; FAB-EXO-504)."""
    out: list[dict[str, float | str]] = []
    n = native()
    for p in all_presets():
        if p.v_max_mps <= 0:
            continue
        out.append(
            {
                "class": p.name,
                "m_rated_kg": p.design_mass_kg,
                "v_max_mps": p.v_max_mps,
                "rating_j": float(n.bumper_rating(p.design_mass_kg, p.v_max_mps)),
                "momentum_kgmps": p.momentum_kgmps,
            }
        )
    return out


def attended_jog(p: ExoPreset | str, jog_cap_mps: float | None = None) -> dict[str, float]:
    """Attended-jog limits at rated mass (EXOTIC §X14; FAB-EXO-503).

    v_jog = min(jog cap, √(2·E_ped/m)) with the imported pedestrian energy cap; momentum
    √(2·E_ped·m) is stated, not capped; the peak edge force of the compliant edge
    (stiffness k) with the drive capped at the imported sensed static limit F is
    F + √(F² + 2·k·KE).
    """
    p = preset(p) if isinstance(p, str) else p
    caps = pedestrian_caps()
    cap = p.regime.jog_v_mps if jog_cap_mps is None else jog_cap_mps
    if cap is None:
        raise ValueError(f"{p.name}: no attended-jog cap (not a GuardedZone class)")
    m = p.design_mass_kg
    v = min(cap, caps.energy_speed_mps(m))
    ke = 0.5 * m * v * v
    k = p.regime.edge_k_n_per_m or 0.0
    f = caps.f_static_sensed_n
    return {
        "m_rated_kg": m,
        "v_jog_mps": v,
        "ke_j": ke,
        "momentum_kgmps": math.sqrt(2.0 * caps.e_max_j * m),
        "edge_k_n_per_m": k,
        "f_drive_cap_n": f,
        "f_edge_peak_n": f + math.sqrt(f * f + 2.0 * k * ke),
        "m_full_speed_max_kg": 2.0 * caps.e_max_j / (cap * cap),
    }


# ---- stray field (FAB-EXO-104; PHYSICS §23 method) ------------------------------------------

#: Accessible-surface distance from the nearest magnet face, m (the ENV-001 10 cm target
#: distance, measured from the magnet face as the conservative reading).
ACCESSIBLE_DISTANCE_M = 0.10
#: Remanence of the sovereign-class magnets, T (GBD NdFeB at 20 °C) [UNCERTAIN: grade].
B_R_SOVEREIGN_T = 1.3
#: Pole-module face used for the nearest-module estimate, m (the 0.5 m pole of EXOTIC
#: §X9's sagitta example; one module dominates the field at the accessible surface).
MODULE_FACE_M = 0.5
#: Operating gap per class, m: the midpoint of the exotic design §1.2 gap range
#: [UNCERTAIN: design ranges, not presets].
CLASS_GAP_M: dict[str, float] = {
    "sentinel": 0.0125,
    "haven_r": 0.0125,
    "haven_f": 0.0175,
    "shield": 0.015,
    "bastion": 0.0125,
    "arena": 0.020,
    "rotator": 0.0175,
    "tidegate": 0.0225,
    "selene": 0.010,
}
#: HTS guideway (SELENE): Halbach thickness, m [UNCERTAIN] and wavenumber, 1/m (the ATZ
#: fit λ ≈ 158 mm; EXOTIC §X3).
HALBACH_THICKNESS_M = 0.020
HALBACH_K_PER_M = 2.0 * math.pi / 0.1576


def prism_on_axis_t(z_m: float, half_a_m: float, half_b_m: float, thickness_m: float, b_r_t: float) -> float:
    """On-axis field of a uniformly magnetised rectangular block at ``z_m`` from its face, T.

    Surface-charge closed form (Furlani 2001, §2.7; the "exact block" of PHYSICS §23):
    B = (B_r/π)·[atan(ab/(z·√(a²+b²+z²))) − atan(ab/((z+L)·√(a²+b²+(z+L)²)))], with a, b
    the half-sides of the face and L the thickness.
    """
    a, b, L, z = half_a_m, half_b_m, thickness_m, z_m

    def term(zz: float) -> float:
        return math.atan(a * b / (zz * math.sqrt(a * a + b * b + zz * zz)))

    return b_r_t / math.pi * (term(z) - term(z + L))


def dipole_on_axis_t(r_m: float, moment_a_m2: float) -> float:
    """On-axis point-dipole field (μ0/4π)·2m/r³, T (the PHYSICS §23 far-field form)."""
    return MU0_H_PER_M / (4.0 * math.pi) * 2.0 * moment_a_m2 / r_m**3


@dataclass(frozen=True)
class StrayFieldRow:
    """Stray-field estimate of one class at the accessible surface (FAB-EXO-104)."""

    name: str
    model: str
    gap_m: float
    magnet_thickness_m: float
    b_accessible_t: float
    target_t: float
    accessible_m: float
    distance_to_target_m: float
    shroud_required: bool
    inputs: dict[str, float]


def _stray_target_t() -> float:
    """The 0.5 mT target, read from the core preset (``design.suspension.stray_target_t``; FAB-ENV-001)."""
    return float(native().Params.preset("door").get("design.suspension.stray_target_t"))


def _distance_where(f, target: float, lo: float, hi: float) -> float:
    """Distance in [lo, hi] where the decreasing field ``f`` falls to ``target`` (bisection; hi if never)."""
    if f(hi) > target:
        return math.inf
    if f(lo) <= target:
        return lo
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if f(mid) > target:
            lo = mid
        else:
            hi = mid
    return hi


def stray_field_report(accessible_m: float = ACCESSIBLE_DISTANCE_M) -> list[StrayFieldRow]:
    """Stray field at the accessible surface for every class, with the shroud flag (FAB-EXO-104).

    Hybrid EMS classes: the nearest pole module as a bare NdFeB block of face
    ``MODULE_FACE_M`` square, sized by the maximum-energy-product rule
    h_m = 4·B_gap²·g/B_r² (B_gap = the admission's governing flux, g = the class gap),
    field on axis by the exact block form. SELENE (HTS over a PM guideway): a one-sided
    Halbach guideway, B(z) = B_r·(1 − e^(−k·t))·(2√2/π)·e^(−k·z). Both are bare magnets
    (no iron return path), so the values are upper-bound-style estimates, not measured.
    A class whose field exceeds the target at ``accessible_m`` gets ``shroud_required``.
    """
    target = _stray_target_t()
    adm = {r.name: r for r in admission_table()}
    rows: list[StrayFieldRow] = []
    for p in all_presets():
        g = CLASS_GAP_M[p.name]
        if p.suspension == "HtsBulk":
            b0 = B_R_SOVEREIGN_T * (1.0 - math.exp(-HALBACH_K_PER_M * HALBACH_THICKNESS_M)) * 2.0 * math.sqrt(2.0) / math.pi

            def field(z: float, b0: float = b0) -> float:
                return b0 * math.exp(-HALBACH_K_PER_M * z)

            model = "one-sided Halbach PM guideway, B0·e^(−k·z) (bare magnets)"
            h_m = HALBACH_THICKNESS_M
            inputs = {"b_r_t": B_R_SOVEREIGN_T, "halbach_thickness_m": h_m, "k_per_m": HALBACH_K_PER_M, "b0_t": b0}
        else:
            b_gap = adm[p.name].b_eff_t
            h_m = 4.0 * b_gap * b_gap * g / B_R_SOVEREIGN_T**2
            half = 0.5 * MODULE_FACE_M

            def field(z: float, h_m: float = h_m, half: float = half) -> float:
                return prism_on_axis_t(z, half, half, h_m, B_R_SOVEREIGN_T)

            model = "nearest pole module as a bare NdFeB block, exact on-axis form (Furlani 2001)"
            inputs = {"b_r_t": B_R_SOVEREIGN_T, "b_gap_t": b_gap, "module_face_m": MODULE_FACE_M, "magnet_thickness_m": h_m}
        b_acc = field(accessible_m)
        rows.append(
            StrayFieldRow(
                name=p.name,
                model=model,
                gap_m=g,
                magnet_thickness_m=h_m,
                b_accessible_t=b_acc,
                target_t=target,
                accessible_m=accessible_m,
                distance_to_target_m=_distance_where(field, target, 1e-4, 1e3),
                shroud_required=b_acc > target,
                inputs=inputs,
            )
        )
    return rows


def pole_areas_by_class() -> dict[str, float]:
    """Admission pole area per class, m² (convenience for plots and studies)."""
    return {r.name: r.area_m2 for r in admission_table()}

