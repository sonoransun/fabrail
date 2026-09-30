"""🔭 PEGS on-site gravity gradiometry: could a gravity-gradient detector beat the P wave?

FAB-FRN-003; FRONTIER §F4 (anchor tier 🧪: the PEGS network magnitude, Mw ≥ 8.3).

A rupture redistributes mass, and the gravity change travels at the speed of light, so a
prompt elasto-gravity signal (PEGS) reaches a site before the P wave. On an
accelerometer the gravity change and the ground acceleration it induces cancel in a
full space; gradiometers measure the spatial derivative, which does not cancel
(Juhel et al. 2019 GJI; Vallée et al. 2017 Science). That is the case for an on-site
gradiometer, and it needs a sensitivity nobody has built.

**Reduced-order what-if model** (frontier research §3a; the r⁵ form after Harms 2015
[REF-MEM]; the coefficient is a one-point fit [UNCERTAIN]; PHYSICS §19, §20 for the
P-wave timing)::

    M0(Mw)   = 10^(1.5·Mw + 9.1) N·m                       (Hanks & Kanamori 1979)
    Ṁ0(t)    triangle of half-duration T_h = 2.26e-6·M0^(1/3) s
             (the GCMT 1.05e-8·M0^(1/3) with M0 in dyne·cm; Ekström et al. 2012 [REF-MEM])
    I₂(t)    = ∫₀ᵗ ∫₀^τ M0(s) ds dτ                         (second time integral of M0(t))
    h(R, t)  ≈ κ_h·G·I₂(t)/R⁵,  κ_h = 4.6                   (R: source–sensor distance)
    t_det    = first t with h/(ASD·√B) ≥ 5,  B = 0.1 Hz
    gain over on-site P       = R/v_P − t_det               (v_P 6.5 km/s)
    gain over a dense network = t_net − t_det,  t_net = R_net/v_P + latency

κ_h = 4.6 is fitted so that Mw 7 at R = 140 km is detected at t = 10 s with
1e-15/√Hz (Shimoda et al. 2021 GJI: "Mw 7 within 10 s out to about 140 km"); the fit
gives 4.599. With the source-distance form, the research sweep table (frontier
research §3a, depth 10 km) is reproduced to its printed 0.1 s.

**The honest reading** (exotic critique XCRIT §1): the required sensitivity is
1e-15/√Hz and the best demonstrated is 1e-10/√Hz (TOBA Phase-II, 3–8 Hz), 5 orders
short. Against a dense P network (nearest station at hypocentral ≈ 22 km, alert
latency 3.3 s) the lead-time gain at the reference case is ≈ 0 s; the real value of
PEGS is a magnitude that does not saturate, which network PEGS already gives for
Mw ≥ 8.3 (:func:`magnitude_advisory`). PEGS is never a sole trigger (FAB-EXO-810).
After the P wave arrives the prompt-signal model no longer applies, so a detection
later than R/v_P is flagged (``before_p`` False) and has no lead-time value.

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from functools import lru_cache

import numpy as np

from ._const import G_NEWTON_M3_PER_KG_S2
from ._record import FrontierRecord, make_record, orders

__all__ = [
    "ASD_BEST_DEMONSTRATED",
    "ASD_NEEDED",
    "ASD_SWEEP",
    "BANDWIDTH_HZ",
    "DENSE_NETWORK_R_KM",
    "KAPPA_H",
    "NETWORK_LATENCY_S",
    "PEGSNET_MW_MIN",
    "REFERENCE_MW",
    "REFERENCE_R_KM",
    "REFERENCE_T_S",
    "RESEARCH_CASES",
    "SNR_DETECT",
    "V_P_KMPS",
    "detection",
    "magnitude_advisory",
    "sensitivity_gap",
    "sweep",
]

#: Gradient coefficient κ_h in h ≈ κ_h·G·I₂/R⁵ (one-point fit to Shimoda 2021; frontier
#: research §3a [CALC, UNCERTAIN]; FRONTIER §F4).
KAPPA_H = 4.6
#: Detection threshold h/(ASD·√B), and the band B, Hz (Shimoda 2021 via frontier
#: research §3a: SNR 5, B = 0.1 Hz; FRONTIER §F4).
SNR_DETECT = 5.0
BANDWIDTH_HZ = 0.1
#: P speed for site and network arrivals, km/s (frontier research §3a sweep; exotic
#: design §2.8 lead-time gain 0.15385 s/km; PHYSICS §20).
V_P_KMPS = 6.5
#: Required gradient ASD, s⁻²/√Hz (TOBA Phase-III target; Shimoda 2021; FRONTIER §1, §F4).
ASD_NEEDED = 1e-15
#: Best demonstrated gradient ASD, s⁻²/√Hz (TOBA Phase-II, about 1e-10 at 3–8 Hz;
#: TOBA review arXiv 2412.01323; frontier research §3a; FRONTIER §1: 5 orders short; §F4).
ASD_BEST_DEMONSTRATED = 1e-10
#: The research sweep, s⁻²/√Hz: 100× the Phase-III target down to the final TOBA
#: target (10 m bars at 4 K) (frontier research §3a; FRONTIER §F4).
ASD_SWEEP: tuple[float, ...] = (1e-13, 1e-15, 1e-17, 1e-19)
#: Calibration case: Mw 7 detected at 10 s at a source distance of 140 km (Shimoda 2021; FRONTIER §F4).
REFERENCE_MW = 7.0
REFERENCE_R_KM = 140.0
REFERENCE_T_S = 10.0
#: Research sweep cases (Mw, epicentral km) at depth 10 km (frontier research §3a; FRONTIER §F4).
RESEARCH_CASES: tuple[tuple[float, float], ...] = ((7.0, 50.0), (7.0, 100.0), (7.0, 200.0), (8.0, 200.0), (9.0, 500.0))
RESEARCH_DEPTH_KM = 10.0
#: Dense onshore network: nearest station at hypocentral ≈ 22 km (exotic critique
#: XCRIT §1, PEGS row; FRONTIER §F4).
DENSE_NETWORK_R_KM = 22.0
#: Network alert latency after the first station's P (window + packet + telemetry +
#: processing + dissemination), s (exotic design §2.8, the fabrail-exo-sim
#: eew_timing::DEFAULT_LATENCY_S [UNCERTAIN]; EXOTIC §X10; PHYSICS §20). XCRIT's
#: ≈ 10.7 s alert adds a 4 s window on top; the gain is ≈ 0 s either way.
NETWORK_LATENCY_S = 3.3
#: Search horizon for the detection time, s.
HORIZON_S = 600.0
#: GCMT half-duration scaling in SI, s per (N·m)^(1/3) (Ekström et al. 2012 [REF-MEM]; FRONTIER §F4; PHYSICS §19).
_HALF_DURATION_COEFF = 2.26e-6
#: Network PEGS (anchor tier): PEGSNet tracks magnitude reliably from Mw 8.3, from about
#: 40 s after origin (Tohoku alert at 53 s) (Licciardi et al. 2022 Nature); PEGSGraph
#: (Peru, real time) targets Mw ≥ 8.2 and stabilises at about 300 s (Arias et al. 2025
#: GJI). Frontier research §3a (FRONTIER §F4; EXOTIC §X10).
PEGSNET_MW_MIN = 8.3
PEGSNET_T_FIRST_S = 40.0
PEGSNET_TOHOKU_ALERT_S = 53.0
PEGSGRAPH_MW_MIN = 8.2
PEGSGRAPH_T_STABLE_S = 300.0
#: Far-field long-period securing mode for Mw ≥ 8.5 (frontier research §3a action ladder; FRONTIER §F4).
LONG_PERIOD_MW = 8.5


def _moment_n_m(mw: np.ndarray | float) -> np.ndarray:
    """Seismic moment M0 = 10^(1.5·Mw + 9.1), N·m (Hanks & Kanamori 1979)."""
    return np.power(10.0, 1.5 * np.asarray(mw, dtype=float) + 9.1)


def _half_duration_s(m0_n_m: np.ndarray) -> np.ndarray:
    """Triangle half-duration T_h = 2.26e-6·M0^(1/3), s (GCMT scaling [REF-MEM])."""
    return _HALF_DURATION_COEFF * np.cbrt(m0_n_m)


def _i2(t_s: np.ndarray | float, mw: np.ndarray | float) -> np.ndarray:
    """I₂(t), the second time integral of M0(t), N·m·s², for a triangular moment rate.

    Ṁ0 = (M0/T_h²)·[r(t) − 2r(t − T_h) + r(t − 2T_h)] with the ramp r(x) = max(x, 0), so
    integrating three times replaces r by q(x) = max(x, 0)⁴/24 (exact; FRONTIER §F4).
    """
    t = np.asarray(t_s, dtype=float)
    m0 = _moment_n_m(mw)
    th = _half_duration_s(m0)

    def q(x: np.ndarray) -> np.ndarray:
        xp = np.maximum(x, 0.0)
        return xp**4 / 24.0

    return m0 / th**2 * (q(t) - 2.0 * q(t - th) + q(t - 2.0 * th))


def _gradient(t_s: np.ndarray | float, mw: np.ndarray | float, r_m: np.ndarray | float, kappa: float) -> np.ndarray:
    """Gravity gradient h ≈ κ_h·G·I₂(t)/R⁵, s⁻² (FRONTIER §F4)."""
    return kappa * G_NEWTON_M3_PER_KG_S2 * _i2(t_s, mw) / np.asarray(r_m, dtype=float) ** 5


def _t_detect(
    asd: np.ndarray | float,
    mw: np.ndarray | float,
    r_m: np.ndarray | float,
    kappa: float,
    snr: float,
    bandwidth_hz: float,
    horizon_s: float,
) -> np.ndarray:
    """First t ≤ horizon with h/(ASD·√B) ≥ snr, s (inf when never). Vectorised bisection.

    I₂ is non-decreasing (M0(t) ≥ 0), so bisection on I₂(t) = I₂* is exact to float
    resolution after 64 halvings of the horizon.
    """
    a, m, r = np.broadcast_arrays(np.asarray(asd, float), np.asarray(mw, float), np.asarray(r_m, float))
    need = snr * a * math.sqrt(bandwidth_hz) * r**5 / (kappa * G_NEWTON_M3_PER_KG_S2)
    lo = np.zeros(a.shape)
    hi = np.full(a.shape, float(horizon_s))
    reach = _i2(hi, m) >= need
    for _ in range(64):
        mid = 0.5 * (lo + hi)
        ok = _i2(mid, m) >= need
        hi = np.where(ok, mid, hi)
        lo = np.where(ok, lo, mid)
    return np.where(reach, hi, np.inf)


def _source_km(r_km: np.ndarray | float, depth_km: np.ndarray | float) -> np.ndarray:
    return np.hypot(np.asarray(r_km, float), np.asarray(depth_km, float))


def _positive(**values: object) -> None:
    for name, value in values.items():
        v = np.asarray(value, dtype=float)
        if not np.all(np.isfinite(v)) or np.any(v <= 0.0):
            raise ValueError(f"{name} must be finite and > 0 (got {value!r})")


@lru_cache(maxsize=1)
def _reference_gap() -> tuple[float, float, float]:
    """(ASD needed at the reference case, orders short of the best demonstrated, κ fit)."""
    r_m = REFERENCE_R_KM * 1e3
    h = float(_gradient(REFERENCE_T_S, REFERENCE_MW, r_m, KAPPA_H))
    asd_needed = h / (SNR_DETECT * math.sqrt(BANDWIDTH_HZ))
    kappa_fit = (
        SNR_DETECT
        * ASD_NEEDED
        * math.sqrt(BANDWIDTH_HZ)
        * r_m**5
        / (G_NEWTON_M3_PER_KG_S2 * float(_i2(REFERENCE_T_S, REFERENCE_MW)))
    )
    return asd_needed, orders(ASD_BEST_DEMONSTRATED / asd_needed), kappa_fit


@lru_cache(maxsize=1)
def _reference_network_gain() -> float:
    t_det = float(
        _t_detect(ASD_NEEDED, REFERENCE_MW, REFERENCE_R_KM * 1e3, KAPPA_H, SNR_DETECT, BANDWIDTH_HZ, HORIZON_S)
    )
    t_net = DENSE_NETWORK_R_KM / V_P_KMPS + NETWORK_LATENCY_S
    return max(0.0, t_net - t_det)


def _note() -> str:
    _, short, _ = _reference_gap()
    return (
        f"PEGS on-site gradiometer: needs {ASD_NEEDED:.0e}/√Hz, best demonstrated "
        f"{ASD_BEST_DEMONSTRATED:.0e}/√Hz ({round(short, 1):g} orders short); against a dense P network "
        f"the lead-time gain is ≈ {_reference_network_gain():.0f} s (Mw {REFERENCE_MW:g} at "
        f"{REFERENCE_R_KM:.0f} km); the real value is an unsaturated magnitude."
    )


def detection(
    asd_per_rthz: float = ASD_NEEDED,
    mw: float = REFERENCE_MW,
    r_km: float = REFERENCE_R_KM,
    depth_km: float = 0.0,
    *,
    r_network_km: float = DENSE_NETWORK_R_KM,
    latency_s: float = NETWORK_LATENCY_S,
    v_p_kmps: float = V_P_KMPS,
    kappa_h: float = KAPPA_H,
    snr: float = SNR_DETECT,
    bandwidth_hz: float = BANDWIDTH_HZ,
    horizon_s: float = HORIZON_S,
) -> FrontierRecord:
    """Detection time of an on-site gradiometer and its lead-time gain (FRONTIER §F4).

    Args:
        asd_per_rthz: detector gradient noise ASD, s⁻²/√Hz.
        mw: moment magnitude.
        r_km: epicentral distance, km. With ``depth_km = 0`` (default) it is the source
            distance R itself, the calibration geometry of Shimoda 2021.
        depth_km: source depth, km; R = √(r² + depth²).
        r_network_km: hypocentral distance of the nearest network station, km.
        latency_s: network alert latency after that station's P, s.
        v_p_kmps, kappa_h, snr, bandwidth_hz, horizon_s: model constants (module defaults).

    Returns:
        A record with ``t_det_s`` (inf when never within the horizon), ``t_p_site_s``,
        ``t_network_alert_s``, the clipped gains ``gain_vs_onsite_p_s`` and
        ``gain_vs_network_s`` (≥ 0), the signed ``advantage_*`` values, ``detected`` and
        ``before_p``. Missing orders: the reference sensitivity gap (5).
    """
    _positive(
        asd_per_rthz=asd_per_rthz,
        r_km=r_km,
        v_p_kmps=v_p_kmps,
        kappa_h=kappa_h,
        snr=snr,
        bandwidth_hz=bandwidth_hz,
        horizon_s=horizon_s,
        r_network_km=r_network_km,
    )
    if not math.isfinite(depth_km) or depth_km < 0.0 or not math.isfinite(latency_s) or latency_s < 0.0:
        raise ValueError("depth_km and latency_s must be finite and >= 0")
    r_src_km = float(_source_km(r_km, depth_km))
    t_det = float(_t_detect(asd_per_rthz, mw, r_src_km * 1e3, kappa_h, snr, bandwidth_hz, horizon_s))
    t_p = r_src_km / v_p_kmps
    t_net = r_network_km / v_p_kmps + latency_s
    adv_p = t_p - t_det
    adv_net = t_net - t_det
    _, short, _ = _reference_gap()
    return make_record(
        "pegs",
        "pegs.detection",
        short,
        _note(),
        {
            "asd_per_rthz": asd_per_rthz,
            "mw": mw,
            "r_km": r_km,
            "depth_km": depth_km,
            "r_network_km": r_network_km,
            "latency_s": latency_s,
            "v_p_kmps": v_p_kmps,
            "kappa_h": kappa_h,
            "snr": snr,
            "bandwidth_hz": bandwidth_hz,
        },
        {
            "r_source_km": r_src_km,
            "t_det_s": t_det,
            "t_p_site_s": t_p,
            "t_network_alert_s": t_net,
            "detected": math.isfinite(t_det),
            "before_p": bool(t_det < t_p),
            "advantage_vs_onsite_p_s": adv_p,
            "advantage_vs_network_s": adv_net,
            "gain_vs_onsite_p_s": max(0.0, adv_p) if math.isfinite(adv_p) else 0.0,
            "gain_vs_network_s": max(0.0, adv_net) if math.isfinite(adv_net) else 0.0,
        },
    )


def sweep(
    asd_per_rthz: Sequence[float] = ASD_SWEEP,
    cases: Sequence[tuple[float, float]] = RESEARCH_CASES,
    depth_km: float = RESEARCH_DEPTH_KM,
    *,
    r_network_km: float = DENSE_NETWORK_R_KM,
    latency_s: float = NETWORK_LATENCY_S,
    v_p_kmps: float = V_P_KMPS,
) -> FrontierRecord:
    """Detection time and lead-time gains over an ASD × (Mw, distance) grid (FRONTIER §F4).

    The defaults reproduce the frontier research §3a table: ASD 1e-13 … 1e-19 against
    Mw 7 at 50/100/200 km, Mw 8 at 200 km and Mw 9 at 500 km, depth 10 km.

    Returns:
        A record with ``t_det_s``, ``advantage_vs_onsite_p_s``, ``advantage_vs_network_s``,
        ``gain_vs_network_s`` and ``before_p``, each shaped (n_asd, n_cases), plus the
        per-case ``t_p_site_s`` and ``r_source_km``.
    """
    asd = np.asarray(asd_per_rthz, dtype=float).reshape(-1)
    mw = np.asarray([c[0] for c in cases], dtype=float)
    r_epi = np.asarray([c[1] for c in cases], dtype=float)
    _positive(asd_per_rthz=asd, r_km=r_epi, v_p_kmps=v_p_kmps, r_network_km=r_network_km)
    r_src = _source_km(r_epi, depth_km)
    t_det = _t_detect(asd[:, None], mw[None, :], r_src[None, :] * 1e3, KAPPA_H, SNR_DETECT, BANDWIDTH_HZ, HORIZON_S)
    t_p = r_src / v_p_kmps
    t_net = r_network_km / v_p_kmps + latency_s
    adv_p = t_p[None, :] - t_det
    adv_net = t_net - t_det
    _, short, _ = _reference_gap()
    return make_record(
        "pegs",
        "pegs.sweep",
        short,
        _note(),
        {
            "asd_per_rthz": asd,
            "cases_mw_r_km": [tuple(map(float, c)) for c in cases],
            "depth_km": depth_km,
            "r_network_km": r_network_km,
            "latency_s": latency_s,
            "v_p_kmps": v_p_kmps,
        },
        {
            "r_source_km": r_src,
            "t_p_site_s": t_p,
            "t_network_alert_s": t_net,
            "t_det_s": t_det,
            "advantage_vs_onsite_p_s": adv_p,
            "advantage_vs_network_s": adv_net,
            "gain_vs_network_s": np.where(np.isfinite(adv_net), np.maximum(adv_net, 0.0), 0.0),
            "before_p": t_det < t_p[None, :],
        },
    )


def sensitivity_gap(
    mw: float = REFERENCE_MW,
    r_km: float = REFERENCE_R_KM,
    t_target_s: float = REFERENCE_T_S,
    asd_best_per_rthz: float = ASD_BEST_DEMONSTRATED,
) -> FrontierRecord:
    """The sensitivity an on-site gradiometer needs, against the best demonstrated (FRONTIER §F4).

    Args:
        mw, r_km, t_target_s: detect this magnitude at this source distance within this
            time (default: Shimoda's Mw 7, 140 km, 10 s).
        asd_best_per_rthz: best demonstrated gradient ASD (TOBA Phase-II, 1e-10/√Hz).

    Returns:
        A record with ``asd_needed_per_rthz``, ``asd_best_per_rthz``, ``orders_short``
        (the missing orders), the calibration ``kappa_fit`` (4.599 against the 4.6 used) and
        ``t_det_best_s``, the detection time at the best demonstrated sensitivity (inf:
        never within the horizon).
    """
    _positive(r_km=r_km, t_target_s=t_target_s, asd_best_per_rthz=asd_best_per_rthz)
    r_m = r_km * 1e3
    h = float(_gradient(t_target_s, mw, r_m, KAPPA_H))
    asd_needed = h / (SNR_DETECT * math.sqrt(BANDWIDTH_HZ))
    short = orders(asd_best_per_rthz / asd_needed)
    _, _, kappa_fit = _reference_gap()
    t_best = float(_t_detect(asd_best_per_rthz, mw, r_m, KAPPA_H, SNR_DETECT, BANDWIDTH_HZ, HORIZON_S))
    return make_record(
        "pegs",
        "pegs.sensitivity_gap",
        short,
        _note(),
        {
            "mw": mw,
            "r_km": r_km,
            "t_target_s": t_target_s,
            "kappa_h": KAPPA_H,
            "snr": SNR_DETECT,
            "bandwidth_hz": BANDWIDTH_HZ,
        },
        {
            "gradient_at_target_per_s2": h,
            "asd_needed_per_rthz": asd_needed,
            "asd_best_per_rthz": asd_best_per_rthz,
            "orders_short": short,
            "kappa_fit": kappa_fit,
            "t_det_best_s": t_best,
            "best_detects_before_p": bool(t_best < r_km / V_P_KMPS),
        },
    )


def magnitude_advisory(mw: float = 9.0) -> FrontierRecord:
    """Network PEGS magnitude advisory: the research-tier anchor (🧪; FAB-EXO-810).

    PEGSNet tracks the magnitude of Mw ≥ 8.3 events without saturation from about 40 s
    after origin; P-wave magnitudes saturate for great earthquakes. The advisory can
    select the far-field long-period securing mode (Mw ≥ 8.5), but it is never a sole
    trigger: motion needs a corroborating network alert or a local P detection.

    Returns:
        A record with ``advisory_available``, ``t_first_estimate_s`` (inf when not
        available), ``t_stable_s``, ``long_period_mode`` and ``sole_trigger`` (always
        False). Missing orders: 0 (network PEGS is demonstrated offline and runs in real
        time in Peru; the on-site gradiometer is the 🔭 item).
    """
    if not math.isfinite(mw):
        raise ValueError(f"mw must be finite (got {mw!r})")
    available = mw >= PEGSNET_MW_MIN
    return make_record(
        "pegs_network",
        "pegs.magnitude_advisory",
        0.0,
        (
            f"PEGS network magnitude: research tier, reliable for Mw ≥ {PEGSNET_MW_MIN:g} from about "
            f"{PEGSNET_T_FIRST_S:.0f} s; an advisory for magnitude, never a sole trigger (FAB-EXO-810). "
            f"The on-site gradiometer stays 🔭."
        ),
        {"mw": mw},
        {
            "advisory_available": available,
            "t_first_estimate_s": PEGSNET_T_FIRST_S if available else math.inf,
            "t_tohoku_alert_s": PEGSNET_TOHOKU_ALERT_S,
            "t_stable_s": PEGSGRAPH_T_STABLE_S if mw >= PEGSGRAPH_MW_MIN else math.inf,
            "long_period_mode": bool(available and mw >= LONG_PERIOD_MW),
            "sole_trigger": False,
            "role": "magnitude advisory into fusion; motion needs a corroborating network alert or local P",
        },
    )
