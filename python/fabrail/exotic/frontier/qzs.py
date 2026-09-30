"""🧪 Quasi-zero-stiffness (QZS) isolation: Mizuno's series singularity and the tilt error.

FAB-FRN-005; FRONTIER §F6. Deployed at instrument scale (Mizuno lab tables; LIGO HAM-ISI
gives up to 40 dB from about 0.1 to 8 Hz, Matichard et al. 2015 CQG), research at
building scale.

**Series (Mizuno)** (Mizuno, Takasaki, Kishita & Hirakawa 2007, Control Eng. Practice
15:187 [REF-MEM]; frontier research §3c; PHYSICS §21). Zero-power control leaves a
negative steady-state stiffness k_m = −k_s (the PM force gradient). In series with a
spring k1::

    k = k1·k_m/(k1 + k_m)   → ∞  at k_m = −k1

so the payload does not sag under direct loads while k1 still isolates ground motion.
The same stiffness is the core's ``fabrail._native.series_stiffness`` (PHYSICS §3).

**Parallel (QZS)**: f_n = f_p·√ε, ε = (k_p − k_n)/k_p. A WALL-class 500 kg on a 20 mm
static-deflection spring has f_p = 3.5 Hz; reaching 0.3 Hz needs ε = 0.0073, a 0.7 %
stiffness match that k_n drift with gap and load makes an active-trim problem.

**Tilt** (LIGO tilt-horizontal coupling; frontier research §3c): an inertial sensor
reads g·θ as acceleration, so a horizontal estimate carries x_err = g·θ/ω²
(25 mm at 0.1 Hz and θ = 1e-3 rad).

**Scale.** At building scale the stroke must cover the peak ground displacement
(0.5–2 m) while a zero-power EMS stage has a millimetre gap (frontier research §3c):
the missing-orders figure is log10(0.5 m / 1 mm) ≈ 2.7 [UNCERTAIN: stroke of the
demonstrated stages]; at instrument scale it is 0.

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from ._const import G0_MPS2
from ._record import FrontierRecord, make_record, orders

__all__ = [
    "K1_N_PER_M",
    "PGD_BUILDING_M",
    "SCALES",
    "STATIC_DEFLECTION_M",
    "STROKE_DEMONSTRATED_M",
    "parallel_qzs",
    "series_stiffness",
    "tilt_error",
]

#: Illustrative series spring, N/m (FRONTIER §F6; the singularity is scale-free).
K1_N_PER_M = 1.0e5
#: WALL-class example: 20 mm static deflection, target 0.3 Hz (frontier research §3c; FRONTIER §F6).
STATIC_DEFLECTION_M = 0.020
F_TARGET_HZ = 0.3
#: Building-scale stroke need: peak ground displacement, lower end of 0.5–2 m (frontier
#: research §3c; FRONTIER §F6; PHYSICS §21).
PGD_BUILDING_M = 0.5
#: Stroke of a demonstrated zero-power EMS stage: a millimetre gap (frontier research §3c:
#: "z cannot be done with a millimetre EMS gap"; FRONTIER §F6) [UNCERTAIN].
STROKE_DEMONSTRATED_M = 1.0e-3
#: Scales and their labels.
SCALES: tuple[str, str] = ("instrument", "building")


def _missing(scale: str) -> float:
    if scale not in SCALES:
        raise ValueError(f"scale must be one of {SCALES} (got {scale!r})")
    return orders(PGD_BUILDING_M / STROKE_DEMONSTRATED_M) if scale == "building" else 0.0


def _note(scale: str) -> str:
    if scale == "building":
        return (
            f"QZS isolation at building scale: research tier; the stroke must cover the peak ground "
            f"displacement (≥ {PGD_BUILDING_M:g} m) against a millimetre EMS gap; stiffness singularity "
            f"at k_m = −k1; tilt error g·θ/ω²."
        )
    return "QZS isolation at instrument scale: deployed (LIGO-class tables); stiffness singularity at k_m = −k1."


def series_stiffness(
    k1_n_per_m: float = K1_N_PER_M,
    km_over_k1: Sequence[float] | np.ndarray | None = None,
    load_n: float = 1000.0,
    scale: str = "building",
) -> FrontierRecord:
    """Series stiffness k1·k_m/(k1 + k_m) near the singularity k_m = −k1 (FRONTIER §F6).

    Args:
        k1_n_per_m: positive series spring, N/m.
        km_over_k1: k_m/k1 values (default −2 … −0.5 in 0.01 steps, exactly −1 included).
        load_n: direct load on the payload, N (for the sag δ = F/k).
        scale: "instrument" or "building".

    Returns:
        A record with ``k_series_n_per_m`` (±inf at the singularity), ``sag_m`` (0 there)
        and ``singular_km_n_per_m`` (= −k1).
    """
    if not (math.isfinite(k1_n_per_m) and k1_n_per_m > 0.0):
        raise ValueError(f"k1_n_per_m must be finite and > 0 (got {k1_n_per_m!r})")
    ratio = (np.arange(-100, 51) / 100.0 - 1.0) if km_over_k1 is None else np.asarray(km_over_k1, float).reshape(-1)
    k_m = ratio * k1_n_per_m
    den = k1_n_per_m + k_m
    with np.errstate(divide="ignore", invalid="ignore"):
        k = np.where(den == 0.0, np.inf, k1_n_per_m * k_m / np.where(den == 0.0, 1.0, den))
        sag = np.where(np.isinf(k), 0.0, load_n / k)
    return make_record(
        "qzs",
        "qzs.series_stiffness",
        _missing(scale),
        _note(scale),
        {"k1_n_per_m": k1_n_per_m, "km_over_k1": ratio, "load_n": load_n, "scale": scale},
        {
            "k_m_n_per_m": k_m,
            "k_series_n_per_m": k,
            "sag_m": sag,
            "singular_km_n_per_m": -k1_n_per_m,
        },
    )


def parallel_qzs(
    static_deflection_m: float = STATIC_DEFLECTION_M,
    f_target_hz: float = F_TARGET_HZ,
    scale: str = "building",
) -> FrontierRecord:
    """Parallel QZS: the stiffness match ε for a target natural frequency (FRONTIER §F6).

    Returns:
        A record with ``f_p_hz`` = √(g/δ)/2π, ``epsilon`` = (f_n/f_p)² and
        ``match_percent`` = 100·ε.
    """
    if static_deflection_m <= 0.0 or f_target_hz <= 0.0:
        raise ValueError("static_deflection_m and f_target_hz must be > 0")
    f_p = math.sqrt(G0_MPS2 / static_deflection_m) / (2.0 * math.pi)
    eps = (f_target_hz / f_p) ** 2
    return make_record(
        "qzs",
        "qzs.parallel_qzs",
        _missing(scale),
        _note(scale),
        {"static_deflection_m": static_deflection_m, "f_target_hz": f_target_hz, "scale": scale},
        {"f_p_hz": f_p, "epsilon": eps, "match_percent": 100.0 * eps},
    )


def tilt_error(
    theta_rad: Sequence[float] | np.ndarray | float = 1.0e-3,
    f_hz: float = 0.1,
    scale: str = "building",
) -> FrontierRecord:
    """Tilt-to-horizontal error x_err = g·θ/ω² (FRONTIER §F6; θ sweep 1e-9 … 1e-3 rad).

    Returns:
        A record with ``x_err_m`` per θ.
    """
    th = np.asarray(theta_rad, dtype=float)
    if f_hz <= 0.0 or not np.all(np.isfinite(th)):
        raise ValueError("f_hz must be > 0 and theta_rad finite")
    w = 2.0 * math.pi * f_hz
    x = G0_MPS2 * th / (w * w)
    return make_record(
        "qzs",
        "qzs.tilt_error",
        _missing(scale),
        _note(scale),
        {"theta_rad": theta_rad, "f_hz": f_hz, "scale": scale},
        {"x_err_m": float(x) if x.ndim == 0 else x},
    )
