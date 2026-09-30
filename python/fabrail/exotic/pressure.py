"""The sovereign tier's other pressure waves: air blast and tsunami timing (EXOTIC §X11, §X12).

**Air blast** (FAB-EXO-806, FAB-EXO-807, FAB-EXO-808). Generic closed forms whose
inputs are the peak overpressure P_s, the positive-phase duration t_d and the arrival
time t_a. There is no scaled-distance law and no yield-to-distance table here, in any
output or anywhere else in fabrail (FAB-EXO-808); t_a is always an input, and the only
distance quantity reported is the bound t_a ≤ R/c0 (the front travels at c0 or faster).
fabrail is not a protective-design or weapons-effects tool; UFC 3-340-02 governs
structural response, which is not modelled.

- Friedlander waveform P(t) = P_s·(1 − t/t_d)·e^(−b·t/t_d) and its positive-phase
  impulse i = P_s·t_d·[1/b − (1 − e^(−b))/b²] (b = 1 gives e⁻¹).
- Normal reflection (Rankine–Hugoniot, ideal gas, γ = 1.4, P0 = 101.325 kPa):
  P_r = 2·P_s·(7·P0 + 4·P_s)/(7·P0 + P_s); P_r/P_s → 2 for weak and → 8 for strong shocks.
- Shock-front speed U_s = c0·√(1 + (γ + 1)/(2γ)·P_s/P0) ≥ c0.
- The closure race of ``fabrail_exo::closure`` through ``_native.exo_closure_decision``.

**Tsunami timing** (FAB-EXO-809): c = √(g·h), t = ∫dx/√(g·h(x)), Green's law
A1/A0 = (h0/h1)^(1/4), budget = t_arrival − t_shaking − t_latency, all from
``fabrail_exo_sim::tsunami``. Timing only; no inundation, run-up or loads.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ._util import C0_AIR_MPS, G0_MPS2, GAMMA_AIR, P0_PA, native

__all__ = [
    "BASTION_CLOSE",
    "ClosureDecision",
    "arrival_bound_s",
    "closure_decision",
    "friedlander",
    "friedlander_impulse",
    "friedlander_impulse_numeric",
    "min_range_for_close_m",
    "reflection_factor",
    "reflected_pressure",
    "shock_speed",
    "tidegate_plan",
    "tsunami_arrival_s",
    "tsunami_timing",
]


def friedlander(t_s: np.ndarray | float, p_s_pa: float, t_d_s: float, b: float) -> np.ndarray:
    """Friedlander overpressure P(t) = P_s·(1 − t/t_d)·e^(−b·t/t_d) for 0 ≤ t ≤ t_d, else 0, Pa (EXOTIC §X11)."""
    _check_blast(p_s_pa, t_d_s, b)
    t = np.asarray(t_s, dtype=float)
    x = t / t_d_s
    return np.where((x >= 0.0) & (x <= 1.0), p_s_pa * (1.0 - x) * np.exp(-b * x), 0.0)


def friedlander_impulse(p_s_pa: float, t_d_s: float, b: float) -> float:
    """Positive-phase impulse i = P_s·t_d·[1/b − (1 − e^(−b))/b²], Pa·s (closed form; FAB-EXO-806).

    ``b → 0`` tends to the triangular pulse P_s·t_d/2; the series branch keeps it exact
    near zero (1/b − (1 − e^(−b))/b² = 1/2 − b/6 + b²/24 − …).
    """
    _check_blast(p_s_pa, t_d_s, b)
    if abs(b) < 1e-4:
        shape = 0.5 - b / 6.0 + b * b / 24.0 - b**3 / 120.0
    else:
        shape = 1.0 / b - (1.0 - math.exp(-b)) / (b * b)
    return p_s_pa * t_d_s * shape


def friedlander_impulse_numeric(p_s_pa: float, t_d_s: float, b: float, n: int = 20_001) -> float:
    """Positive-phase impulse by composite Simpson quadrature of :func:`friedlander` (cross-check)."""
    if n < 3 or n % 2 == 0:
        raise ValueError("n must be odd and >= 3")
    t = np.linspace(0.0, t_d_s, n)
    y = friedlander(t, p_s_pa, t_d_s, b)
    h = t_d_s / (n - 1)
    return float(h / 3.0 * (y[0] + y[-1] + 4.0 * y[1:-1:2].sum() + 2.0 * y[2:-1:2].sum()))


def reflected_pressure(p_s_pa: float | np.ndarray, p0_pa: float = P0_PA) -> np.ndarray | float:
    """Normal reflected overpressure P_r = 2·P_s·(7·P0 + 4·P_s)/(7·P0 + P_s), Pa (γ = 1.4; EXOTIC §X11)."""
    ps = np.asarray(p_s_pa, dtype=float)
    if np.any(ps < 0) or not np.all(np.isfinite(ps)):
        raise ValueError("P_s must be finite and >= 0")
    pr = 2.0 * ps * (7.0 * p0_pa + 4.0 * ps) / (7.0 * p0_pa + ps)
    return float(pr) if np.ndim(pr) == 0 else pr


def reflection_factor(p_s_pa: float | np.ndarray, p0_pa: float = P0_PA) -> np.ndarray | float:
    """P_r/P_s = 2·(7·P0 + 4·P_s)/(7·P0 + P_s): 2.008 at 1 kPa, 2.741 at 100 kPa, 7.60 at 10 MPa, → 8."""
    ps = np.asarray(p_s_pa, dtype=float)
    if np.any(ps <= 0) or not np.all(np.isfinite(ps)):
        raise ValueError("P_s must be finite and > 0")
    f = 2.0 * (7.0 * p0_pa + 4.0 * ps) / (7.0 * p0_pa + ps)
    return float(f) if np.ndim(f) == 0 else f


def shock_speed(p_s_pa: float, c0_mps: float = C0_AIR_MPS, p0_pa: float = P0_PA, gamma: float = GAMMA_AIR) -> float:
    """Shock-front speed U_s = c0·√(1 + (γ + 1)/(2γ)·P_s/P0) ≥ c0, m/s (1.04·c0 at 10 kPa, 1.36·c0 at 100 kPa)."""
    if p_s_pa < 0:
        raise ValueError("P_s must be >= 0")
    return c0_mps * math.sqrt(1.0 + (gamma + 1.0) / (2.0 * gamma) * p_s_pa / p0_pa)


def arrival_bound_s(r_m: float, c0_mps: float = C0_AIR_MPS) -> float:
    """Upper bound of the blast arrival time at range R: t_a ≤ R/c0, s (reported next to every decision).

    This bound is the only range-related blast quantity fabrail computes; t_a itself is
    always an input (FAB-EXO-808).
    """
    if r_m < 0:
        raise ValueError("R must be >= 0")
    return r_m / c0_mps


def min_range_for_close_m(t_needed_s: float, c0_mps: float = C0_AIR_MPS) -> float:
    """Range a close needing ``t_needed_s`` of lead requires, R ≥ c0·t (necessary, not sufficient), m."""
    return c0_mps * t_needed_s


@dataclass(frozen=True)
class ClosureDecision:
    """A closure-race decision (``fabrail_exo::closure``; FAB-EXO-807) with every input."""

    close: bool
    t_close_s: float | None
    reason: str | None
    t_available_s: float
    arrival_bound_range_m: float
    inputs: dict[str, float | bool]


#: The BASTION close of EXOTIC §X11: 3 m from rest at a 0.48 m/s², j 2 m/s³, v_max 1.2 m/s,
#: 0.3 s seal and 0.3 s margin (5.25 s + 0.6 s → t_a ≥ 5.85 s → R ≥ c0·5.85 s ≈ 2 km).
BASTION_CLOSE: dict[str, float] = {"d_m": 3.0, "v0_mps": 0.0, "a_mps2": 0.48, "j_mps3": 2.0, "v_max_mps": 1.2, "t_seal_s": 0.3, "margin_s": 0.3}


def closure_decision(
    t_a_s: float,
    *,
    t_detect_s: float = 0.0,
    zone_clear: bool = True,
    two_person_override: bool = False,
    **plan: float,
) -> ClosureDecision:
    """CLOSE iff t_min + t_seal + margin ≤ t_a − t_detect and the zone is clear (EXOTIC §X11; FAB-EXO-807).

    ``plan`` overrides :data:`BASTION_CLOSE` keys (d_m, v0_mps, a_mps2, j_mps3, v_max_mps,
    t_seal_s, margin_s). The decision is the Rust planner's; ``arrival_bound_range_m`` is
    c0·t_a, the smallest range compatible with the given t_a (the front is never slower
    than c0). No yield or range is ever converted into t_a.
    """
    unknown = set(plan) - set(BASTION_CLOSE)
    if unknown:
        raise ValueError(f"unknown closure inputs {sorted(unknown)} (known: {sorted(BASTION_CLOSE)})")
    p = {**BASTION_CLOSE, **{k: float(v) for k, v in plan.items()}}
    r = native().exo_closure_decision(
        p["d_m"], p["v0_mps"], p["a_mps2"], p["j_mps3"], p["v_max_mps"], p["t_seal_s"], p["margin_s"], float(t_a_s), float(t_detect_s), bool(zone_clear), bool(two_person_override)
    )
    t_close = r.get("t_close_s")
    inputs: dict[str, float | bool] = {**p, "t_a_s": float(t_a_s), "t_detect_s": float(t_detect_s), "zone_clear": bool(zone_clear), "two_person_override": bool(two_person_override), "c0_mps": C0_AIR_MPS}
    return ClosureDecision(
        close=bool(r["close"]),
        t_close_s=None if t_close is None else float(t_close),
        reason=None if r.get("reason") is None else str(r["reason"]),
        t_available_s=float(t_a_s) - float(t_detect_s),
        arrival_bound_range_m=min_range_for_close_m(float(t_a_s)),
        inputs=inputs,
    )


def _check_blast(p_s_pa: float, t_d_s: float, b: float) -> None:
    if not (math.isfinite(p_s_pa) and p_s_pa >= 0):
        raise ValueError(f"P_s must be finite and >= 0 (got {p_s_pa!r})")
    if not (math.isfinite(t_d_s) and t_d_s > 0):
        raise ValueError(f"t_d must be finite and > 0 (got {t_d_s!r})")
    if not (math.isfinite(b) and b >= 0):
        raise ValueError(f"b must be finite and >= 0 (got {b!r})")


# ---- tsunami timing (EXOTIC §X12; FAB-EXO-809) ------------------------------------------------


def tsunami_arrival_s(profile: list[tuple[float, float]] | None = None, step_m: float = 100.0) -> float:
    """t = ∫dx/√(g·h(x)) over a (distance m, depth m) profile, s (``tsunami::arrival``; default reference profile)."""
    return float(native().exo_tsunami_arrival(profile, step_m))


def tsunami_timing(depths_m: tuple[float, ...] = (4000.0, 1000.0, 50.0, 10.0), h0_m: float = 4000.0) -> list[dict[str, float]]:
    """Wave speed c = √(g·h) and Green's-law amplification (h0/h)^(1/4) at each depth (EXOTIC §X12).

    Each row carries the native value and the closed form restated here (g = 9.80665).
    """
    n = native()
    rows = []
    for h in depths_m:
        rows.append(
            {
                "depth_m": float(h),
                "c_mps": float(n.exo_tsunami_speed(h)),
                "c_closed_form_mps": math.sqrt(G0_MPS2 * h),
                "greens_factor": float(n.exo_greens_law(h0_m, h)),
                "greens_closed_form": (h0_m / h) ** 0.25,
                "h0_m": float(h0_m),
            }
        )
    return rows


def tidegate_plan(
    t_arrival_s: float,
    t_shaking_s: float,
    t_latency_s: float,
    *,
    command: bool = True,
    subsidence_m: float = 0.0,
    differential_frac: float = 0.05,
    tilt_rad: float = 0.0,
) -> dict[str, object]:
    """The TIDEGATE close plan (``tsunami::plan_tidegate_close``): maglev after the gap check, else wheels.

    TIDEGATE closes only after the shaking ends and only on the external decision
    system's command (``command``); budget = t_arrival − t_shaking − t_latency.
    """
    plan = dict(
        native().exo_tidegate_plan(float(t_arrival_s), float(t_shaking_s), float(t_latency_s), bool(command), float(subsidence_m), float(differential_frac), float(tilt_rad))
    )
    plan["inputs"] = {
        "t_arrival_s": float(t_arrival_s),
        "t_shaking_s": float(t_shaking_s),
        "t_latency_s": float(t_latency_s),
        "command": bool(command),
        "subsidence_m": float(subsidence_m),
        "differential_frac": float(differential_frac),
        "tilt_rad": float(tilt_rad),
    }
    return plan
