"""🔭 Ambient-pressure room-temperature superconductor (RTSC) suspension: what if Tc were high?

FAB-FRN-001; FRONTIER §F2 (anchor tier 🧪: bulk HTS flux pinning, EXOTIC §X3).

No room-temperature superconductor exists (as of Sept 2026: LK-99 refuted in 2023, the
Dias papers retracted; the highest ambient-pressure Tc is Hg-1223 at about 133–138 K).
This model asks what a hypothetical one would need and what it would buy, using the
frozen-image HTS model with cryogenics removed (exotic design §2.3 RtscIdeal; frontier
research §3d; PHYSICS §2 for the magnetic pressure).

**The ceiling is the rail, not the superconductor.** Over a PM guideway of surface-field
amplitude B0 and period λ = 2π/k, ideal flux exclusion (the frozen image at full
strength) gives the pressure bound

    P̄(h) = (B0²/μ0)·e^(−2kh)        1.15 MPa at contact, 0.51 MPa at h = 10 mm (λ 154 mm)

(exotic critique XCRIT §1: the 0.64 MPa of the design needs λ = 214 mm; the HTS model's
λ is 154 mm). A bulk superconductor reaches a fraction η of that image; today's bulk
YBCO reaches η ≈ 0.15 (the ATZ seed F0 = 32.9 kN over 4 × 492 cm² against P̄(0)).

**Temperature.** Jc(T) = Jc0·(1 − T/Tc)^n with n ≈ 1.5 [UNCERTAIN]. REBCO works at
77 K because t = T/Tc = 77/92 = 0.84 leaves enough Jc. Matching that reduced temperature
at 300 K needs Tc = 300·92/77 ≈ 358 K, and keeping the flux-creep rate S ∝ kT/U0 needs
the pinning energy U0 scaled by 300/77 ≈ 3.9. In the sweep the pinning-limited pressure
is p = min(η·j, 1)·P̄ with j = Jc(300 K, Tc)/Jc(77 K, 92 K): more pinning than REBCO's
never lifts p above P̄, so the pressure ceiling stays rail-limited whatever Tc is.

The missing-orders figure is log10(358 K / 138 K) ≈ 0.4: Tc must rise 2.6× beyond any
known ambient-pressure material, and at 300 K no known material superconducts at all.

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from ._const import MU0_H_PER_M
from ._record import FrontierRecord, make_record, orders

__all__ = [
    "B0_T",
    "ETA_SWEEP",
    "ETA_TODAY",
    "GAP_M",
    "JC_EXPONENT",
    "K_PER_M",
    "TC_BEST_AMBIENT_K",
    "TC_REBCO_K",
    "TC_SWEEP_K",
    "T_AMBIENT_K",
    "T_LN2_K",
    "pressure_bound",
    "required_tc",
    "sweep",
]

#: PM guideway surface-field amplitude, T: B0²/μ0 = 1.15 MPa, the ideal frozen-image
#: bound at contact (exotic design §2.3 RtscIdeal; FRONTIER §F2; PHYSICS §2).
B0_T = 1.2
#: Guideway wavenumber k = 2π/λ, 1/m (λ = 154 mm; fabrail-exo-sim
#: HtsBulk::atz_demonstrator; exotic design §2.3; EXOTIC §X3).
K_PER_M = 40.85
#: Reference levitation gap, m (FRONTIER §1: "0.51 MPa at 10 mm"; FRONTIER §F2).
GAP_M = 0.010
#: REBCO/YBCO critical temperature and LN₂ operating point, K (fabrail-exo-sim HtsBulk
#: tc_k 92 K; exotic design §2.3; FRONTIER §F2; EXOTIC §X3).
TC_REBCO_K = 92.0
T_LN2_K = 77.0
#: Ambient operating temperature with cryogenics removed, K (FRONTIER FAB-FRN-001; FRONTIER §F2).
T_AMBIENT_K = 300.0
#: Jc(T) = Jc0·(1 − T/Tc)^n exponent (frontier research §3d, n ≈ 1.5 [UNCERTAIN]; FRONTIER §F2).
JC_EXPONENT = 1.5
#: Highest ambient-pressure Tc, K: Hg-1223, 133–138 K (frontier research §3d; 151 K after
#: pressure quench is [UNCERTAIN] and not used; FRONTIER §F2).
TC_BEST_AMBIENT_K = 138.0
#: η of today's bulk HTS: F0/(A·B0²/μ0) with the ATZ seed F0 = 32.9 kN over 4 × 492 cm²
#: (fabrail-exo-sim HtsBulk::atz_demonstrator; exotic critique XCRIT §1) ≈ 0.146 (FRONTIER §F2; EXOTIC §X3).
ETA_TODAY = 32.9e3 / (4.0 * 492e-4 * B0_T * B0_T / MU0_H_PER_M)
#: Sweep ranges (FRONTIER FAB-FRN-001): η 0.15–1, Tc 90–600 K (FRONTIER §F2).
ETA_SWEEP: tuple[float, float] = (0.15, 1.0)
TC_SWEEP_K: tuple[float, float] = (90.0, 600.0)


def _bound_pa(h_m: np.ndarray | float, b0_t: float, k_per_m: float) -> np.ndarray:
    """Ideal frozen-image pressure P̄ = (B0²/μ0)·e^(−2kh), Pa (FRONTIER §F2; EXOTIC §X3)."""
    return (b0_t * b0_t / MU0_H_PER_M) * np.exp(-2.0 * k_per_m * np.asarray(h_m, dtype=float))


def _jc_factor(t_k: float, tc_k: np.ndarray | float, n: float) -> np.ndarray:
    """(1 − T/Tc)^n, 0 at or above Tc (frontier research §3d; FRONTIER §F2)."""
    tc = np.asarray(tc_k, dtype=float)
    base = np.clip(1.0 - t_k / tc, 0.0, None)
    return base**n


def _required_tc_k(t_op_k: float) -> float:
    """Tc giving REBCO's reduced temperature 77/92 at T_op: T_op·92/77, K (XCRIT §1)."""
    return t_op_k * TC_REBCO_K / T_LN2_K


def _note(tc_needed_k: float, ceiling_pa: float, h_m: float) -> str:
    return (
        f"Ambient-pressure RTSC suspension: no such material exists; it needs Tc ≈ {tc_needed_k:.0f} K "
        f"(best ambient-pressure Tc ≈ {TC_BEST_AMBIENT_K:.0f} K) and about {T_AMBIENT_K / T_LN2_K:.1f}× REBCO's "
        f"pinning energy, and the pressure ceiling stays rail-limited ({ceiling_pa / 1e6:.2f} MPa at "
        f"{h_m * 1e3:.0f} mm)."
    )


def _missing() -> float:
    return orders(_required_tc_k(T_AMBIENT_K) / TC_BEST_AMBIENT_K)


def pressure_bound(h_m: float = GAP_M, b0_t: float = B0_T, k_per_m: float = K_PER_M) -> FrontierRecord:
    """The rail-limited pressure ceiling (B0²/μ0)·e^(−2kh) (FRONTIER §F2).

    Returns:
        A record with ``p_bound_pa`` at ``h_m``, ``p_contact_pa``, ``lambda_m`` and
        ``p_today_pa`` = η_today·P̄ (the zero-field-cooled limit of today's bulk HTS).
    """
    if not (math.isfinite(h_m) and h_m >= 0.0) or b0_t <= 0.0 or k_per_m <= 0.0:
        raise ValueError("h_m must be >= 0 and b0_t, k_per_m > 0")
    p = float(_bound_pa(h_m, b0_t, k_per_m))
    return make_record(
        "rtsc",
        "rtsc.pressure_bound",
        _missing(),
        _note(_required_tc_k(T_AMBIENT_K), p, h_m),
        {"h_m": h_m, "b0_t": b0_t, "k_per_m": k_per_m},
        {
            "lambda_m": 2.0 * math.pi / k_per_m,
            "p_contact_pa": float(_bound_pa(0.0, b0_t, k_per_m)),
            "p_bound_pa": p,
            "eta_today": ETA_TODAY,
            "p_today_pa": ETA_TODAY * p,
        },
    )


def required_tc(t_op_k: float = T_AMBIENT_K) -> FrontierRecord:
    """What Tc and pinning an RTSC needs to match REBCO-at-77 K at ``t_op_k`` (FRONTIER §F2).

    Returns:
        A record with ``tc_required_k`` (358.4 K at 300 K), ``reduced_temperature``
        (77/92), ``pinning_energy_ratio`` (T_op/77 K, from S ∝ kT/U0),
        ``tc_best_ambient_k`` and ``tc_factor_short``.
    """
    if not math.isfinite(t_op_k) or t_op_k <= 0.0:
        raise ValueError(f"t_op_k must be finite and > 0 (got {t_op_k!r})")
    tc = _required_tc_k(t_op_k)
    ceiling = float(_bound_pa(GAP_M, B0_T, K_PER_M))
    return make_record(
        "rtsc",
        "rtsc.required_tc",
        orders(tc / TC_BEST_AMBIENT_K),
        _note(tc, ceiling, GAP_M),
        {"t_op_k": t_op_k, "tc_rebco_k": TC_REBCO_K, "t_rebco_op_k": T_LN2_K},
        {
            "tc_required_k": tc,
            "reduced_temperature": T_LN2_K / TC_REBCO_K,
            "pinning_energy_ratio": t_op_k / T_LN2_K,
            "tc_best_ambient_k": TC_BEST_AMBIENT_K,
            "tc_factor_short": tc / TC_BEST_AMBIENT_K,
            "jc_factor_rebco_77k": float(_jc_factor(T_LN2_K, TC_REBCO_K, JC_EXPONENT)),
        },
    )


def sweep(
    eta: Sequence[float] | np.ndarray | None = None,
    tc_k: Sequence[float] | np.ndarray | None = None,
    t_op_k: float = T_AMBIENT_K,
    h_m: float = GAP_M,
    n: float = JC_EXPONENT,
) -> FrontierRecord:
    """The RTSC what-if sweep: pressure over η × Tc at ambient temperature, no cryogenics.

    p(η, Tc) = min(η·j(Tc), 1)·P̄(h), j = (1 − T_op/Tc)^n / (1 − 77/92)^n (FRONTIER §F2).

    Args:
        eta: image fractions (default 0.15 … 1 in 0.05 steps).
        tc_k: critical temperatures, K (default 90 … 600 K in 10 K steps).
        t_op_k: operating temperature, K (300 K: no cryogenics).
        h_m: gap, m.
        n: Jc exponent.

    Returns:
        A record with ``p_pa`` shaped (n_eta, n_tc), ``ceiling_pa`` (P̄ at ``h_m``),
        ``jc_ratio`` per Tc, ``levitates`` (p > 0), ``tc_required_k``, ``p_max_pa`` and
        ``rail_limited`` (every p ≤ the ceiling).
    """
    eta_a = np.round(np.arange(15, 101, 5) / 100.0, 12) if eta is None else np.asarray(eta, dtype=float).reshape(-1)
    tc_a = np.arange(90.0, 601.0, 10.0) if tc_k is None else np.asarray(tc_k, dtype=float).reshape(-1)
    if np.any(eta_a <= 0.0) or np.any(eta_a > 1.0) or np.any(tc_a <= 0.0):
        raise ValueError("eta must lie in (0, 1] and tc_k must be > 0")
    ceiling = float(_bound_pa(h_m, B0_T, K_PER_M))
    j = _jc_factor(t_op_k, tc_a, n) / _jc_factor(T_LN2_K, TC_REBCO_K, n)
    p = np.minimum(eta_a[:, None] * j[None, :], 1.0) * ceiling
    tc_req = _required_tc_k(t_op_k)
    return make_record(
        "rtsc",
        "rtsc.sweep",
        _missing(),
        _note(tc_req, ceiling, h_m),
        {"eta": eta_a, "tc_k": tc_a, "t_op_k": t_op_k, "h_m": h_m, "n": n, "b0_t": B0_T, "k_per_m": K_PER_M},
        {
            "p_pa": p,
            "jc_ratio": j,
            "levitates": p > 0.0,
            "ceiling_pa": ceiling,
            "p_max_pa": float(p.max()),
            "rail_limited": bool(np.all(p <= ceiling)),
            "tc_required_k": tc_req,
            "tc_min_levitating_k": float(tc_a[np.any(p > 0.0, axis=0)].min()) if np.any(p > 0.0) else math.inf,
        },
    )
