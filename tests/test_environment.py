"""Contract: environment requirements FAB-ENV-001 (stray field) and FAB-ENV-004 (demagnetisation proxy).

Both are analyses (method A, status S) on the frozen parameter presets, evaluated through
the native surface:

- ENV-001: the stray field is the point-dipole sum of the permanent-magnet blocks
  (``fabrail_plant::stray::field`` via ``_native.stray_field``; PHYSICS §23). The test
  re-derives the same dipole sum in numpy (tolerance (E), rel 1e-9) so the budget below
  is known to be the documented model, then evaluates it 10 cm from every magnet centre
  on the accessible side of the pole plane (z ≤ 0: below and beside the carriage; above
  the pole plane is the steel rail). The rule asserted for every class: the worst field
  at 10 cm meets ``design.suspension.stray_target_t`` (0.5 mT) **or** the preset declares
  ``design.suspension.shroud = true`` ("shroud required"); WALL and HEAVY must be the
  flagged classes (core design §C.2: 16 mT / 80 mT unshielded). The model is a bare-magnet
  upper-bound estimate (the rail and pole iron that close most of the flux are not
  modelled), which is why the status is S; it is not a measured stray-field value.
- ENV-004: the demagnetisation proxy is the magnet's internal field H_m at the operating
  point with the coil driving its full pulse current against the magnet (−N·i_pulse), hot
  (80 °C, the SH-grade limit ``design.thermal.t_mag_max_c``), over the whole physical gap
  range [g_up, g_land], in the series magnetic circuit the plant uses (PHYSICS §2):
  B = μ0·(M_eff(T) − N·i)/(2g + h′), H_m = (B − B_r(T))/(μ0·μ_rec). The margin is to the
  knee of the SH-grade intrinsic curve at 80 °C. The material data below are typical
  datasheet values and are marked [UNCERTAIN]; the proxy is reported per class
  (``record_property``) and asserted to stay above the knee.

Owner: B5 (Wave 2). Pure analysis, well under 1 s.
"""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

CLASSES = ("drawer", "door", "wall", "heavy")

#: Vacuum permeability, H/m (CODATA 2018; the value ``fabrail_params::derive::MU0_H_PER_M`` uses).
MU0 = 1.25663706212e-6

#: ENV-001 evaluation distance from a magnet centre, m (SPECIFICATION FAB-ENV-001 "at 10 cm").
R_EVAL_M = 0.10

#: Classes the specification expects to carry the "shroud required" flag (FAB-ENV-001;
#: core design §C.2 stray-field row: WALL at risk, 16 mT; HEAVY shrouds required, 80 mT).
SHROUD_CLASSES = ("wall", "heavy")

#: SH-grade sintered NdFeB, intrinsic coercivity at 20 °C, A/m: ≥ 20 kOe = 1592 kA/m (the
#: grade-class minimum of the "SH" suffix in manufacturer grade tables, e.g. N35SH–N45SH)
#: [UNCERTAIN: supplier data, not a measured lot].
HCJ_20C_A_PER_M = 1592.0e3
#: Temperature coefficient of H_cj, 1/K: −0.60 %/K, the conservative end of the
#: −0.55…−0.60 %/K typical datasheet range for SH grades (20–120 °C) [UNCERTAIN].
BETA_HCJ_PER_K = -0.0060
#: Knee field as a fraction of H_cj (squareness H_k/H_cj ≥ 0.90, typical sintered NdFeB
#: datasheet specification) [UNCERTAIN].
KNEE_FRACTION = 0.90


def _directions() -> np.ndarray:
    """The 26 lattice directions (i, j, k) ∈ {−1, 0, 1}³ \\ 0 with k ≤ 0 (accessible half), unit length."""
    dirs = [d for d in itertools.product((-1, 0, 1), repeat=3) if d != (0, 0, 0) and d[2] <= 0]
    v = np.array(dirs, dtype=np.float64)
    return v / np.linalg.norm(v, axis=1)[:, None]


def _magnets(p) -> list[tuple[np.ndarray, np.ndarray]]:
    """(centre, moment) of every PM block, PHYSICS §23 as documented in plant/src/stray.rs:
    block centre h_m/2 behind the pole face along the element normal, moment B_r·w·l·h_m/μ0."""
    s = "design.suspension."
    n = int(p.get(s + "n_magnets"))
    h_m, b_r = p.get(s + "h_m_m"), p.get(s + "b_r_t")
    area = p.get(s + "pole_w_m") * p.get(s + "pole_l_m")
    out = []
    for k in range(n):
        st = f"{s}stations.{k}."
        sign = p.get(st + "normal_sign")
        if not p.get(st + "has_pm") or sign == 0:
            continue
        d = 1.0 if sign > 0 else -1.0
        centre = np.array([p.get(st + "x_m"), p.get(st + "y_m"), p.get(st + "z_m") - d * 0.5 * h_m])
        out.append((centre, np.array([0.0, 0.0, d * b_r * area * h_m / MU0])))
    return out


def _dipole_sum(p, pts: np.ndarray) -> np.ndarray:
    """B = Σ (μ0/4π)·(3(m·r̂)r̂ − m)/r³ over the PM blocks, T (PHYSICS §23)."""
    b = np.zeros_like(pts)
    for c, m in _magnets(p):
        r = pts - c
        r2 = np.einsum("ij,ij->i", r, r)
        mr = (r @ m) / r2
        b += MU0 / (4.0 * math.pi) * (3.0 * mr[:, None] * r - m[None, :]) / (r2 * np.sqrt(r2))[:, None]
    return b


def _native_field(native, p, pts: np.ndarray) -> np.ndarray:
    raw = native.stray_field(p, np.ascontiguousarray(pts, dtype="<f8").tobytes())
    return np.frombuffer(raw, dtype="<f8").reshape(-1, 3)


def _worst_at(native, p, r_m: float) -> tuple[float, np.ndarray]:
    """Largest |B| (T) at distance r_m from any magnet centre on the accessible side, and where."""
    dirs = _directions()
    pts = np.vstack([c + r_m * dirs for c, _ in _magnets(p)])
    mag = np.linalg.norm(_native_field(native, p, pts), axis=1)
    i = int(np.argmax(mag))
    return float(mag[i]), pts[i]


def _distance_for(native, p, target_t: float) -> float:
    """Smallest distance (m, bisection to 0.1 mm) beyond which the worst accessible field is ≤ target."""
    lo, hi = 0.01, 5.0
    while hi - lo > 1e-4:
        mid = 0.5 * (lo + hi)
        if _worst_at(native, p, mid)[0] > target_t:
            lo = mid
        else:
            hi = mid
    return hi


@pytest.mark.req("FAB-ENV-001")
def test_stray_field_budget(native, record_property):
    """Contract: the dipole-sum stray field 10 cm from every magnet meets ≤ 0.5 mT, or the class is flagged
    "shroud required"; WALL/HEAVY are the flagged classes (FAB-ENV-001, target, status S)."""
    rows = {}
    for cls in CLASSES:
        p = native.Params.preset(cls)
        target = p.get("design.suspension.stray_target_t")
        assert target == pytest.approx(0.5e-3, rel=1e-12), f"(E) stray target 0.5 mT (FAB-ENV-001), got {target} T"

        # The native field is the documented dipole sum (tolerance (E), rel 1e-9; PHYSICS §23).
        probe = np.array([[0.1, 0.2, -0.3], [0.0, 0.0, -0.1], [1.0, -0.5, -0.2], [0.0, 0.0, -2.0]])
        b_nat, b_ref = _native_field(native, p, probe), _dipole_sum(p, probe)
        err = np.max(np.linalg.norm(b_nat - b_ref, axis=1) / np.linalg.norm(b_ref, axis=1))
        assert err <= 1e-9, f"{cls}: native stray_field differs from the numpy dipole sum by rel {err:.2e} (> 1e-9, PHYSICS §23)"

        b10, where = _worst_at(native, p, R_EVAL_M)
        d_target = _distance_for(native, p, target)
        shroud = bool(p.get("design.suspension.shroud"))
        rows[cls] = (b10, d_target, shroud)
        record_property(f"{cls}_B_10cm_mT", round(b10 * 1e3, 4))
        record_property(f"{cls}_distance_for_0p5mT_cm", round(d_target * 100.0, 2))
        record_property(f"{cls}_shroud_flag", shroud)

    for cls in SHROUD_CLASSES:
        b10, _, shroud = rows[cls]
        assert shroud, f"{cls}: preset must flag 'shroud required' (FAB-ENV-001; core design §C.2), shroud={shroud}"
        assert b10 > 0.5e-3, f"{cls}: the flag is warranted only above the 0.5 mT target, got {b10 * 1e3:.3f} mT at 10 cm"

    violations = {
        cls: f"{b10 * 1e3:.3f} mT at 10 cm (meets 0.5 mT only beyond {d * 100:.1f} cm), shroud={sh}"
        for cls, (b10, d, sh) in rows.items()
        if b10 > 0.5e-3 and not sh
    }
    assert not violations, (
        "FAB-ENV-001 (target ≤ 0.5 mT at 10 cm, dipole sum, PHYSICS §23): unflagged classes above the target: "
        f"{violations}; either the class meets the target at 10 cm or its preset declares shroud = true"
    )


@pytest.mark.req("FAB-ENV-004")
def test_demag_margin_proxy(native, record_property):
    """Contract: with the coil driving −N·i_pulse against the magnet at 80 °C over the physical gap range,
    the SH-grade magnet's operating point stays above the intrinsic knee in every class; the proxy margin is
    reported (FAB-ENV-004, analysis, status S)."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        s = "design.suspension."
        t_hot = p.get("design.thermal.t_mag_max_c")
        assert t_hot == pytest.approx(80.0, abs=1e-12), f"{cls}: SH-grade proxy temperature 80 °C (FAB-ENV-004), got {t_hot}"
        b_r_hot = p.get(s + "b_r_t") * (1.0 + p.get(s + "tempco_br_per_k") * (t_hot - 20.0))
        mu_rec, h_m = p.get(s + "mu_rec"), p.get(s + "h_m_m")
        k_sigma = 1.0 / (1.0 + p.get(s + "leak"))
        m_eff = k_sigma * b_r_hot * h_m / (MU0 * mu_rec)  # magnet MMF, A-turns (PHYSICS §2)
        h_prime = h_m / mu_rec
        ni = p.get(s + "turns") * p.get(s + "i_pulse_a")
        gaps = np.linspace(p.get(s + "g_up_m"), p.get(s + "g_land_m"), 401)
        b = MU0 * (m_eff - ni) / (2.0 * gaps + h_prime)
        h_m_field = (b - b_r_hot) / (MU0 * mu_rec)
        worst = float(h_m_field.min())
        h_knee = KNEE_FRACTION * HCJ_20C_A_PER_M * (1.0 + BETA_HCJ_PER_K * (t_hot - 20.0))
        margin = 1.0 - (-worst) / h_knee
        record_property(f"{cls}_Hm_worst_kA_per_m", round(worst / 1e3, 1))
        record_property(f"{cls}_knee_kA_per_m", round(h_knee / 1e3, 1))
        record_property(f"{cls}_demag_margin", round(margin, 4))
        assert math.isfinite(margin), f"{cls}: demagnetisation proxy must be a finite number"
        assert margin > 0.0, (
            f"{cls}: operating point H_m = {worst / 1e3:.0f} kA/m at −N·i_pulse = −{ni:.0f} A-turns, 80 °C, "
            f"is beyond the SH-grade knee −{h_knee / 1e3:.0f} kA/m (margin {margin:.3f} ≤ 0; FAB-ENV-004 proxy, "
            "PHYSICS §2 series circuit, [UNCERTAIN] material data)"
        )
