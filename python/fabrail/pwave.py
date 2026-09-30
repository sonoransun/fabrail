"""P+S synthesis for early-warning scenarios (core §B.8; PHYSICS §19, §20).

``boore_ps`` builds a three-axis ground record: a Brune-spectrum stochastic P phase
scaled so its Pd follows Wu–Zhao at the hypocentral distance (plan §2b item 6), then
the S phase at t_S. ``site_noise`` adds building noise (MEMS sensor noise, footsteps,
door slams, HVAC, the rail's own actuation) for the false-trigger suites, and
``site_noise_chunks`` streams it in 10-minute chunks for the 24 h detector replay
(plan §2b item 13). numpy/scipy only; every draw comes from the ``rng`` argument.

**Stochastic method** (Boore 2003, "Simulation of ground motion using the stochastic
method", PAGEOPH 160; PHYSICS §19), per phase and component: Gaussian white noise under
a Saragoni–Hart window of length 2·T_d (T_d = 1/f_c + 0.05·R), transformed, normalised
to unit mean-square amplitude, multiplied by the target Fourier amplitude spectrum

    A(f) = C·M0·(2πf)²/(1 + (f/f_c)²) · (1/R) · exp(−πfR/(Q(f)·v)) · exp(−πκf) · S(f),

transformed back, cut at the phase arrival (the zero-phase spectrum's acausal precursor
is dropped) and tapered on over 0.1 s. C = R_Θφ·V·F/(4πρβ³R0) (Boore 2003 eq. 3),
M0 = 10^(1.5M + 16.05) dyne·cm (Hanks & Kanamori 1979), Q(f) = 180·f^0.45
(Raoof, Herrmann & Malagnini 1999), S(f) the Kanai–Tajimi site filter for the S phase
(physics perspective §3). The P phase uses V_p, a corner 1.5·f_c and a 20° incidence
(H/Z = tan 20°); its absolute level is irrelevant because it is rescaled so its 3 s Pd
(the detector's own chain, :func:`pd_tauc`) equals Wu–Zhao at the hypocentral R.
The S phase keeps Boore's absolute level; its PGA is an output (``meta``), not a
target. Travel times use V_p 6.0 and V_s 3.5 km/s (core §C.4; SEI-012).

Owner: W0-C2 (stub; ``hypocentral_km`` complete), then A9 (Wave 1).
"""

from __future__ import annotations

import heapq
import math
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field

import numpy as np
from scipy import optimize, signal

from .ground import G0_MPS2, kanai_tajimi_psd, trapezoid_integral

__all__ = [
    "ADXL355_NOISE_MPS2_PER_RTHZ",
    "PSEvent",
    "V_P_KMPS",
    "V_S_KMPS",
    "boore_ps",
    "brune_corner_hz",
    "hypocentral_km",
    "pd_tauc",
    "seismic_moment_dyne_cm",
    "site_noise",
    "site_noise_chunks",
    "tau_c_wu_kanamori_s",
    "wu_zhao_pd_m",
]

#: P and S speeds for travel times and S-ETA, km/s (core design §C.4 fusion; the
#: ``control.seismic.fusion.v_p_kmps``/``v_s_kmps`` defaults; SEI-012: S−P 0.1190 s/km).
V_P_KMPS = 6.0
V_S_KMPS = 3.5

#: Source-region density, g/cm³, and shear speed, km/s (Boore 2003 Table 2, WNA crust).
_RHO_GCC = 2.8
_BETA_KMPS = 3.5
#: Average S radiation pattern, horizontal partition 1/√2 and free-surface factor 2
#: (Boore & Boatwright 1984; Boore 2003 eq. 3).
_R_THETA_PHI_S = 0.55
_PARTITION = 1.0 / math.sqrt(2.0)
_FREE_SURFACE = 2.0
#: Average P radiation coefficient (Boore & Boatwright 1984). The P level is rescaled
#: to Wu–Zhao, so only the spectral shape of the P phase matters.
_R_P = 0.52
#: Q(f) = 180·f^0.45 (Raoof, Herrmann & Malagnini 1999; Boore 2003 Table 2).
_Q0 = 180.0
_Q_ETA = 0.45
#: Path duration coefficient, s/km (Herrmann 1985; Boore 2003 §3).
_PATH_DURATION_S_PER_KM = 0.05
#: Saragoni–Hart window: ε = 0.2, η = 0.05, window length f_tb·T_d with f_tb = 2
#: (Boore 2003 eqs. 24–26).
_SH_EPS = 0.2
_SH_ETA = 0.05
_SH_FTB = 2.0
#: P corner-frequency ratio f_cP/f_cS (Hanks 1981; physics perspective §3).
_P_CORNER_RATIO = 1.5
#: P incidence from vertical, degrees (physics perspective §3: H/Z = tan i).
_P_INCIDENCE_DEG = 20.0
#: S vertical-to-horizontal amplitude ratio (physics perspective §3).
_S_ZH = 0.3
#: Stress drop, bar (core design §C.5: Brune Δσ 30 bar).
_STRESS_DROP_BAR = 30.0
#: Onset taper of each phase, s (removes the cut of the acausal precursor).
_ONSET_TAPER_S = 0.1
#: Default P arrival in the record, s: the STA/LTA needs its 3000-sample LTA (30 s at
#: 100 Hz; ``control.seismic.detector.lta_n``) before it may trigger, plus 5 s.
_T_P_DEFAULT_S = 35.0
#: Pd window and high-pass (core design §C.4: 3 s window, 0.075 Hz 2-pole high-pass).
_PD_WINDOW_S = 3.0
_PD_HP_HZ = 0.075

#: ADXL355 noise density, m/s²/√Hz: 22.5 µg/√Hz (Analog Devices ADXL355 data sheet
#: Rev. B; the ``design.sensors.accel_noise_mps2_per_rthz`` preset value; PHYSICS §13).
ADXL355_NOISE_MPS2_PER_RTHZ = 22.5e-6 * G0_MPS2

_MG = 1e-3 * G0_MPS2


@dataclass(frozen=True)
class PSEvent:
    """A synthetic P+S record.

    Attributes:
        t_p_s: P arrival, s from record start.
        t_s_s: S arrival, s from record start.
        a_xyz_mps2: acceleration, shape (3, n), axes x (travel), y, z (up), m/s².
        fs_hz: sample rate, Hz.
        magnitude, r_epi_km, depth_km: source parameters used.
        meta: synthesis details (stress drop, κ, Pd target, seeds).
    """

    t_p_s: float
    t_s_s: float
    a_xyz_mps2: np.ndarray
    fs_hz: float
    magnitude: float = math.nan
    r_epi_km: float = math.nan
    depth_km: float = math.nan
    meta: dict[str, object] = field(default_factory=dict)


def hypocentral_km(r_epi_km: float, depth_km: float) -> float:
    """Hypocentral distance √(R_epi² + h²), km."""
    if r_epi_km < 0.0 or depth_km < 0.0:
        raise ValueError("hypocentral_km: distances must be >= 0")
    return math.hypot(r_epi_km, depth_km)


def seismic_moment_dyne_cm(magnitude: float) -> float:
    """M0 = 10^(1.5·M + 16.05) dyne·cm (Hanks & Kanamori 1979, JGR 84)."""
    return 10.0 ** (1.5 * float(magnitude) + 16.05)


def brune_corner_hz(magnitude: float, *, stress_drop_bar: float = _STRESS_DROP_BAR, beta_kmps: float = 3.5) -> float:
    """Brune corner frequency f_c = 4.906e6·β·(Δσ/M0)^(1/3) (Boore 2003 eq. 4; M0 from Hanks–Kanamori).

    β in km/s, Δσ in bar, M0 in dyne·cm. At Δσ = 30 bar: 0.75 / 0.24 / 0.075 Hz for
    M5 / M6 / M7 (physics perspective §3 [calc]).
    """
    if not (stress_drop_bar > 0.0 and beta_kmps > 0.0):
        raise ValueError("brune_corner_hz: stress_drop_bar and beta_kmps must be > 0")
    return 4.906e6 * float(beta_kmps) * (float(stress_drop_bar) / seismic_moment_dyne_cm(magnitude)) ** (1.0 / 3.0)


def wu_zhao_pd_m(magnitude: float, r_hyp_km: float) -> float:
    """Wu–Zhao Pd, m: log10 Pd[cm] = −3.463 + 0.729·M − 1.374·log10 R (Wu & Zhao 2006, GRL 33 L16312).

    R is the hypocentral distance in km (plan §2b item 6). M7 at 20 km gives 0.7118 cm
    (core design §E.5; SEI-012). The native ``pd_wu_zhao`` holds the same regression.
    """
    if r_hyp_km <= 0.0:
        raise ValueError("wu_zhao_pd_m: r_hyp_km must be > 0")
    return 0.01 * 10.0 ** (-3.463 + 0.729 * float(magnitude) - 1.374 * math.log10(float(r_hyp_km)))


def tau_c_wu_kanamori_s(magnitude: float) -> float:
    """Empirical τc, s: M = 3.373·log10 τc + 5.787 (Wu & Kanamori 2005, BSSA 95; reported, not a target)."""
    return 10.0 ** ((float(magnitude) - 5.787) / 3.373)


def pd_tauc(
    a_z_mps2: np.ndarray,
    fs_hz: float,
    t_start_s: float,
    window_s: float = _PD_WINDOW_S,
    f_hp_hz: float = _PD_HP_HZ,
) -> tuple[float, float]:
    """(Pd in m, τc in s) of a vertical record over ``window_s`` from ``t_start_s`` (PHYSICS §20).

    The detector's chain in f64 (``fabrail_core::seismic::pdtc``): every high-pass a
    2-pole Butterworth at ``f_hp_hz`` (bilinear, corner prewarped), a_b = HP(a − a_0),
    v = HP(∫a_b), u = HP(∫v) with trapezoidal ∫, u̇ = HP(v); Pd = max|u| and
    τc = 2π·√(Σu²/Σu̇²) over the window (Wu & Kanamori 2005; Wu & Zhao 2006).
    """
    a = np.asarray(a_z_mps2, dtype=np.float64)
    sos = signal.butter(2, f_hp_hz, btype="highpass", fs=fs_hz, output="sos")
    a_b = signal.sosfilt(sos, a - a[0])
    v = signal.sosfilt(sos, trapezoid_integral(a_b, fs_hz))
    u = signal.sosfilt(sos, trapezoid_integral(v, fs_hz))
    ud = signal.sosfilt(sos, v)
    i0 = int(math.ceil(float(t_start_s) * fs_hz - 1e-9))
    i1 = min(a.shape[0], i0 + int(round(window_s * fs_hz)))
    if not 0 <= i0 < i1:
        raise ValueError(f"pd_tauc: window [{t_start_s}, +{window_s}] s is outside the record")
    uw, udw = u[i0:i1], ud[i0:i1]
    den = float(np.sum(udw * udw))
    tau_c = 2.0 * math.pi * math.sqrt(float(np.sum(uw * uw)) / den) if den > 0.0 else math.nan
    return float(np.max(np.abs(uw))), tau_c


def _saragoni_hart(t_s: np.ndarray, t_eta_s: float) -> np.ndarray:
    """Saragoni & Hart (1974) window, peak 1 at ε·t_η and η at t_η (Boore 2003 eqs. 24–26)."""
    b = -_SH_EPS * math.log(_SH_ETA) / (1.0 + _SH_EPS * (math.log(_SH_EPS) - 1.0))
    c = b / _SH_EPS
    a = (math.e / _SH_EPS) ** b
    x = np.clip(np.asarray(t_s, dtype=np.float64) / t_eta_s, 0.0, None)
    return a * x**b * np.exp(-c * x)


def _next_pow2(n: int) -> int:
    return 1 << max(1, int(n - 1).bit_length())


def _noise_spectrum(rng: np.random.Generator, fs_hz: float, t_d_s: float, pad_s: float) -> tuple[np.ndarray, int, int, int]:
    """Saragoni–Hart-windowed white noise, padded and transformed, normalised to unit mean-square amplitude.

    Returns (spectrum, n_fft, npad, nw); the window starts at sample ``npad`` of the
    padded buffer (Boore 2003 time-domain method, steps 1–3).
    """
    t_eta = _SH_FTB * t_d_s
    nw = max(8, int(math.ceil(t_eta * fs_hz)))
    npad = int(math.ceil(pad_s * fs_hz))
    buf = np.zeros(npad + nw + npad)
    buf[npad : npad + nw] = rng.standard_normal(nw) * _saragoni_hart(np.arange(nw) / fs_hz, t_eta)
    n_fft = _next_pow2(buf.shape[0])
    spec = np.fft.rfft(buf, n_fft)
    spec /= math.sqrt(float(np.mean(np.abs(spec) ** 2)))
    return spec, n_fft, npad, nw


def _shaped(noise: tuple[np.ndarray, int, int, int], fs_hz: float, fas: Callable[[np.ndarray], np.ndarray]) -> np.ndarray:
    """The phase with Fourier amplitude ``fas`` starting at its arrival (index 0), m/s².

    |X(f)|·Δt ≈ A(f) on average, so the spectrum is multiplied by A(f)·fs. The
    zero-phase product has an acausal precursor before the window start; it is cut and
    the onset tapered over :data:`_ONSET_TAPER_S`.
    """
    spec, n_fft, npad, nw = noise
    f = np.fft.rfftfreq(n_fft, 1.0 / fs_hz)
    x = np.fft.irfft(spec * fas(f) * fs_hz, n_fft)
    out = x[npad : npad + nw + npad].copy()
    nt = min(out.shape[0], max(1, int(round(_ONSET_TAPER_S * fs_hz))))
    out[:nt] *= 0.5 * (1.0 - np.cos(np.pi * np.arange(nt) / nt))
    return out


def _calibrate_p_corner(
    noise: tuple[np.ndarray, int, int, int],
    fs_hz: float,
    fas_of_corner: Callable[[float], Callable[[np.ndarray], np.ndarray]],
    tau_target_s: float,
) -> tuple[float, float]:
    """The P corner (Hz) whose phase, from this noise realisation, has τc = ``tau_target_s``.

    τc falls monotonically with the corner in practice; Brent's method on log f over
    [0.01, 30] Hz, with the nearer bracket end when the target is out of reach (τc of a
    3 s window saturates for great events). Returns (corner, achieved τc).
    """
    n_win = int(round(_PD_WINDOW_S * fs_hz))

    def tau(log_f: float) -> float:
        x = _shaped(noise, fs_hz, fas_of_corner(10.0**log_f))[:n_win]
        return pd_tauc(x, fs_hz, 0.0)[1]

    lo, hi = math.log10(0.01), math.log10(30.0)
    g_lo, g_hi = math.log(tau(lo) / tau_target_s), math.log(tau(hi) / tau_target_s)
    if g_lo * g_hi > 0.0:
        best = lo if abs(g_lo) < abs(g_hi) else hi
        return 10.0**best, tau(best)
    root = optimize.brentq(lambda x: math.log(tau(x) / tau_target_s), lo, hi, xtol=1e-6)
    return 10.0**root, tau(root)


def _fas(
    magnitude: float,
    r_km: float,
    kappa_s: float,
    corner_hz: float,
    radiation: float,
    speed_kmps: float,
    site_soil: str | None,
) -> Callable[[np.ndarray], np.ndarray]:
    """Target Fourier amplitude spectrum A(f) of one component, m/s (Boore 2003 eqs. 2–5)."""
    m0 = seismic_moment_dyne_cm(magnitude)
    # C in cm·s per dyne·cm at R0 = 1 km; the 1e-20 converts units (Boore 2003 eq. 3).
    c = radiation * _PARTITION * _FREE_SURFACE / (4.0 * math.pi * _RHO_GCC * _BETA_KMPS**3) * 1e-20

    def fas(f: np.ndarray) -> np.ndarray:
        f = np.asarray(f, dtype=np.float64)
        src = c * m0 / (1.0 + (f / corner_hz) ** 2)  # displacement source spectrum, cm·s
        q = _Q0 * np.power(np.maximum(f, 1e-12), _Q_ETA)
        path = np.exp(-np.pi * f * r_km / (q * speed_kmps)) / r_km
        site = np.exp(-np.pi * kappa_s * f)
        if site_soil is not None:
            site = site * np.sqrt(kanai_tajimi_psd(f, site_soil, 1.0))
        return 0.01 * (2.0 * np.pi * f) ** 2 * src * path * site  # cm → m

    return fas


def boore_ps(
    rng: np.random.Generator,
    magnitude: float,
    r_epi_km: float,
    depth_km: float,
    fs_hz: float = 500.0,
    duration_s: float = 60.0,
    kappa_s: float = 0.04,
    *,
    t_p_s: float = _T_P_DEFAULT_S,
    stress_drop_bar: float = _STRESS_DROP_BAR,
    soil: str | None = "medium",
    p_corner: str = "wu_kanamori",
) -> PSEvent:
    """Stochastic P+S record at a site (Boore 2003 method; P phase scaled to Wu–Zhao Pd).

    Args:
        rng: synthesis generator (draw order: back-azimuth, P noise, S_x, S_y, S_z).
        magnitude, r_epi_km, depth_km: source; R = √(R_epi² + h²) ≥ 1 km.
        fs_hz, duration_s: record sampling; the P arrival is at ``t_p_s`` (default
            35 s: the LTA warm-up of 30 s plus 5 s) and S at t_p + R/V_s − R/V_p.
        kappa_s: site κ (core design §C.5: 0.04).
        stress_drop_bar: Brune Δσ of the S phase (core design §C.5: 30 bar).
        soil: Kanai–Tajimi site filter of the S phase (``None`` = none).
        p_corner: ``"wu_kanamori"`` (default) solves the P corner, for this noise
            realisation, so the P phase's 3 s τc equals the Wu–Kanamori (2005) τc(M);
            ``"brune"`` uses 1.5·f_c (physics perspective §3). Why the default differs:
            at Δσ = 30 bar the M5 P corner is 1.13 Hz, whose 3 s τc (≈ 1.2 s median over
            seeds) is twice the observed 0.58 s and would put every M5 at Level 2,
            contradicting the τc–M scaling the level matrix is calibrated on.

    The P phase is then scaled so its 3 s Pd (:func:`pd_tauc`, the detector's chain,
    P alone) equals :func:`wu_zhao_pd_m` at the hypocentral R (plan §2b item 6). The
    S phase keeps the absolute Boore level; its PGA is reported, not targeted. ``meta``
    carries origin_t_s (= t_p − R/V_p), r_hyp_km, both corners, pd_target_m, pd_raw_m,
    the P scale, the P phase's τc and the Wu–Kanamori τc, the horizontal and vertical
    PGA, and the back-azimuth.
    """
    m, fs, dur = float(magnitude), float(fs_hz), float(duration_s)
    r = hypocentral_km(r_epi_km, depth_km)
    if r < 1.0:
        raise ValueError(f"boore_ps: hypocentral distance must be >= 1 km (got {r} km)")
    if not (0.0 <= t_p_s < dur):
        raise ValueError(f"boore_ps: t_p_s must lie in [0, duration_s) (got {t_p_s} for {dur} s)")
    if p_corner not in ("wu_kanamori", "brune"):
        raise ValueError(f"boore_ps: p_corner must be 'wu_kanamori' or 'brune' (got {p_corner!r})")
    n = int(round(dur * fs))
    t_s = t_p_s + r / V_S_KMPS - r / V_P_KMPS
    fc = brune_corner_hz(m, stress_drop_bar=stress_drop_bar)
    tau_wk = tau_c_wu_kanamori_s(m)
    fc_p0 = _P_CORNER_RATIO * fc if p_corner == "brune" else 1.0 / tau_wk
    t_d_s = 1.0 / fc + _PATH_DURATION_S_PER_KM * r
    t_d_p = 1.0 / fc_p0 + _PATH_DURATION_S_PER_KM * r
    pad_s = min(60.0, 3.0 / min(fc, fc_p0))

    backazimuth = float(rng.uniform(0.0, 2.0 * math.pi))
    p_noise = _noise_spectrum(rng, fs, t_d_p, pad_s)

    def p_fas(corner_hz: float) -> Callable[[np.ndarray], np.ndarray]:
        return _fas(m, r, kappa_s, corner_hz, _R_P, V_P_KMPS, None)

    if p_corner == "brune":
        fc_p = fc_p0
    else:
        fc_p, _ = _calibrate_p_corner(p_noise, fs, p_fas, tau_wk)
    p = _shaped(p_noise, fs, p_fas(fc_p))
    s_fas = _fas(m, r, kappa_s, fc, _R_THETA_PHI_S, _BETA_KMPS, soil)
    s_x = _shaped(_noise_spectrum(rng, fs, t_d_s, pad_s), fs, s_fas)
    s_y = _shaped(_noise_spectrum(rng, fs, t_d_s, pad_s), fs, s_fas)
    s_z = _shaped(_noise_spectrum(rng, fs, t_d_s, pad_s), fs, s_fas)

    def place(x: np.ndarray, t0: float) -> np.ndarray:
        out = np.zeros(n)
        i0 = int(math.ceil(t0 * fs - 1e-9))
        if i0 < n:
            k = min(n - i0, x.shape[0])
            out[i0 : i0 + k] = x[:k]
        return out

    p_z = place(p, t_p_s)
    pd_target = wu_zhao_pd_m(m, r)
    pd_raw, tau_c = pd_tauc(p_z, fs, t_p_s)
    scale = pd_target / pd_raw if pd_raw > 0.0 else 0.0
    p_z *= scale
    h = math.tan(math.radians(_P_INCIDENCE_DEG))
    a = np.vstack(
        [
            h * math.cos(backazimuth) * p_z + place(s_x, t_s),
            h * math.sin(backazimuth) * p_z + place(s_y, t_s),
            p_z + _S_ZH * place(s_z, t_s),
        ]
    )
    meta: dict[str, object] = {
        "method": "Boore 2003 stochastic; P scaled to Wu-Zhao Pd at hypocentral R",
        "origin_t_s": t_p_s - r / V_P_KMPS,
        "r_hyp_km": r,
        "v_p_kmps": V_P_KMPS,
        "v_s_kmps": V_S_KMPS,
        "stress_drop_bar": float(stress_drop_bar),
        "kappa_s": float(kappa_s),
        "soil": soil,
        "p_corner": p_corner,
        "fc_hz": fc,
        "fc_p_hz": fc_p,
        "t_d_s": t_d_s,
        "pd_target_m": pd_target,
        "pd_raw_m": pd_raw,
        "p_scale": scale,
        "tau_c_s": tau_c,
        "tau_c_wu_kanamori_s": tau_wk,
        "pga_h_mps2": float(np.max(np.hypot(a[0], a[1]))),
        "pga_z_mps2": float(np.max(np.abs(a[2]))),
        "backazimuth_deg": math.degrees(backazimuth),
    }
    return PSEvent(t_p_s, t_s, a, fs, m, float(r_epi_km), float(depth_km), meta)


# ---------------------------------------------------------------------------------
# Site noise (PHYSICS §19, §22)
# ---------------------------------------------------------------------------------

#: Default site-noise rates and amplitudes. All [UNCERTAIN] engineering assumptions for
#: a residential/office floor; the ranges follow the physics perspective §3 (footsteps
#: 1.5–2.5 Hz trains of 10–30 Hz damped pulses at 1–10 mg; slams 5–50 mg, < 0.3 s;
#: HVAC tones; self-actuation through a 15 Hz, ζ 5 % structural mode).
_FOOTSTEP_EPISODE_RATE_PER_S = 1.0 / 120.0
_SLAM_RATE_PER_S = 1.0 / 600.0
_ACTUATION_RATE_PER_S = 1.0 / 60.0
_FLOOR_ZETA = 0.03
_STRUCT_MODE_HZ = 15.0
_STRUCT_ZETA = 0.05
_ACT_FORCE_N = 50.0
_STRUCT_MASS_KG = 2000.0
_ATOM_MAX_S = 2.0


class _Atom:
    """One decaying transient at absolute time t0: a·gain·e^(−σ·τ)·shape(ω_d·τ), τ = t − t0 ≥ 0."""

    __slots__ = ("amp", "gains", "kind", "sigma", "t0", "wd")

    def __init__(self, t0: float, amp: float, sigma: float, wd: float, gains: np.ndarray, kind: str) -> None:
        self.t0, self.amp, self.sigma, self.wd, self.gains, self.kind = t0, amp, sigma, wd, gains, kind

    def __lt__(self, other: _Atom) -> bool:
        return self.t0 < other.t0

    def render(self, buf: np.ndarray, n0: int, fs: float) -> None:
        i0 = int(math.ceil(self.t0 * fs - 1e-9))
        length = int(math.ceil(min(_ATOM_MAX_S, math.log(1e3) / self.sigma) * fs))
        lo, hi = max(i0, n0), min(i0 + length, n0 + buf.shape[1])
        if hi <= lo:
            return
        tau = np.arange(lo, hi) / fs - self.t0
        env = self.amp * np.exp(-self.sigma * tau)
        if self.kind == "sine":
            wave = env * np.sin(self.wd * tau)
        else:  # acceleration of an SDOF under a force step: e^(−στ)(cos ω_dτ − (σ/ω_d) sin ω_dτ)
            wave = env * (np.cos(self.wd * tau) - self.sigma / self.wd * np.sin(self.wd * tau))
        buf[:, lo - n0 : hi - n0] += self.gains[:, None] * wave[None, :]


class _Source:
    """A Poisson-arrival source with its own generator (chunk-invariant draw order)."""

    def __init__(self, rng: np.random.Generator, rate_per_s: float) -> None:
        self.rng = rng
        self.rate = rate_per_s
        self.next_t = float(rng.exponential(1.0 / rate_per_s)) if rate_per_s > 0.0 else math.inf

    def emit(self, until_s: float, pending: list[_Atom]) -> None:
        while self.next_t < until_s:
            for atom in self.make(self.next_t):
                heapq.heappush(pending, atom)
            self.next_t += float(self.rng.exponential(1.0 / self.rate))

    def make(self, t0: float) -> list[_Atom]:  # pragma: no cover - overridden
        raise NotImplementedError


class _Footsteps(_Source):
    def make(self, t0: float) -> list[_Atom]:
        g = self.rng
        dur = g.uniform(5.0, 30.0)
        f_step = g.uniform(1.5, 2.5)
        f_floor = g.uniform(10.0, 30.0)
        amp = g.uniform(1.0, 10.0) * _MG
        gains = np.array([g.uniform(-0.3, 0.3), g.uniform(-0.3, 0.3), 1.0])
        n_steps = max(1, int(dur * f_step))
        jitter = g.uniform(-0.05, 0.05, size=n_steps) / f_step
        scale = g.uniform(0.8, 1.2, size=n_steps)
        w = 2.0 * math.pi * f_floor
        sigma = _FLOOR_ZETA * w
        wd = w * math.sqrt(1.0 - _FLOOR_ZETA**2)
        return [_Atom(t0 + k / f_step + jitter[k], amp * scale[k], sigma, wd, gains, "sine") for k in range(n_steps)]


class _Slams(_Source):
    def make(self, t0: float) -> list[_Atom]:
        g = self.rng
        amp = g.uniform(5.0, 50.0) * _MG
        f = g.uniform(8.0, 20.0)
        t_decay = g.uniform(0.1, 0.3)  # to 1 % within < 0.3 s
        az = g.uniform(0.0, 2.0 * math.pi)
        vz = g.uniform(0.2, 0.6)
        gains = np.array([math.cos(az), math.sin(az), vz])
        return [_Atom(t0, amp, math.log(100.0) / t_decay, 2.0 * math.pi * f, gains, "sine")]


class _Actuation(_Source):
    def __init__(self, rng: np.random.Generator, rate_per_s: float, force_n: float, mass_kg: float) -> None:
        super().__init__(rng, rate_per_s)
        self.force_n, self.mass_kg = force_n, mass_kg

    def make(self, t0: float) -> list[_Atom]:
        g = self.rng
        f_a = g.uniform(0.5, 1.0) * self.force_n
        t_a = g.uniform(0.3, 1.0)
        gains = np.array([1.0, g.uniform(-0.1, 0.1), g.uniform(-0.2, 0.2)])
        w = 2.0 * math.pi * _STRUCT_MODE_HZ
        sigma = _STRUCT_ZETA * w
        wd = w * math.sqrt(1.0 - _STRUCT_ZETA**2)
        a0 = f_a / self.mass_kg
        # Accelerate (+F), decelerate (−F) and stop: three force steps into the mode.
        return [
            _Atom(t0, a0, sigma, wd, gains, "step"),
            _Atom(t0 + t_a, -2.0 * a0, sigma, wd, gains, "step"),
            _Atom(t0 + 2.0 * t_a, a0, sigma, wd, gains, "step"),
        ]


def site_noise_chunks(
    rng: np.random.Generator,
    duration_s: float,
    fs_hz: float,
    footsteps: bool = True,
    slams: bool = True,
    hvac: bool = True,
    self_actuation: bool = True,
    *,
    chunk_s: float = 600.0,
    mems: bool = True,
    noise_density_mps2_per_rthz: float = ADXL355_NOISE_MPS2_PER_RTHZ,
    footstep_rate_per_s: float = _FOOTSTEP_EPISODE_RATE_PER_S,
    slam_rate_per_s: float = _SLAM_RATE_PER_S,
    actuation_rate_per_s: float = _ACTUATION_RATE_PER_S,
    actuation_force_n: float = _ACT_FORCE_N,
    structure_mass_kg: float = _STRUCT_MASS_KG,
) -> Iterator[np.ndarray]:
    """Building noise in chunks of ``chunk_s`` (10 minutes by default), each shape (3, m), m/s².

    Every component draws from its own child generator (seeded from ``rng`` in a fixed
    order), transients are rendered at absolute times and carried across chunk
    boundaries, so the concatenated chunks do not depend on ``chunk_s`` (up to
    floating-point summation order). Components (PHYSICS §19, §22; physics perspective
    §3; rates and amplitudes are [UNCERTAIN] defaults):

    - MEMS white noise at ``noise_density_mps2_per_rthz`` (ADXL355 22.5 µg/√Hz) on each
      axis, σ = density·√(fs/2). Leave it off (``mems=False``) for a scenario ``site``
      series: the simulator's sensor model already adds the ADXL355 noise.
    - Footsteps: Poisson walking episodes (1 per 120 s, 5–30 s long) of 1.5–2.5 Hz
      step trains; each step a 10–30 Hz damped floor pulse (ζ 3 %), 1–10 mg, mostly
      vertical.
    - Door slams: Poisson (1 per 600 s), 5–50 mg, 8–20 Hz, decaying to 1 % in 0.1–0.3 s.
    - HVAC: a fan at 12–25 Hz and its 2nd harmonic, 0.05–0.5 mg per axis.
    - Self-actuation: Poisson moves (1 per 60 s) of the rail's own carriage; the LSM
      reaction (±25–50 N for 0.3–1 s each way) excites a 15 Hz, ζ 5 % structural mode
      of 2000 kg effective mass, mostly along x.
    """
    fs, dur = float(fs_hz), float(duration_s)
    n_total = int(round(dur * fs))
    if n_total < 1 or not (chunk_s > 0.0):
        raise ValueError("site_noise_chunks: need duration_s·fs_hz >= 1 and chunk_s > 0")
    seeds = rng.integers(0, 2**63, size=7)
    child = [np.random.default_rng(int(s)) for s in seeds]
    mems_rng = child[0:3]
    sources: list[_Source] = []
    if footsteps:
        sources.append(_Footsteps(child[3], footstep_rate_per_s))
    if slams:
        sources.append(_Slams(child[4], slam_rate_per_s))
    if self_actuation:
        sources.append(_Actuation(child[6], actuation_rate_per_s, actuation_force_n, structure_mass_kg))
    tones: list[tuple[float, np.ndarray, np.ndarray]] = []
    if hvac:
        g = child[5]
        f_rot = g.uniform(12.0, 25.0)
        for harmonic in (1.0, 2.0):
            tones.append((harmonic * f_rot, g.uniform(0.05, 0.5, size=3) * _MG, g.uniform(0.0, 2.0 * math.pi, size=3)))
    sigma = noise_density_mps2_per_rthz * math.sqrt(0.5 * fs)
    tail = int(math.ceil(_ATOM_MAX_S * fs)) + 1
    m_chunk = max(1, int(round(chunk_s * fs)))
    pending: list[_Atom] = []
    carry = np.zeros((3, 0))
    n0 = 0
    while n0 < n_total:
        m = min(m_chunk, n_total - n0)
        n1 = n0 + m
        buf = np.zeros((3, m + tail))
        buf[:, : carry.shape[1]] += carry
        t_end = n1 / fs
        for src in sources:
            src.emit(t_end, pending)
        while pending and pending[0].t0 * fs < n1 - 1e-9:
            heapq.heappop(pending).render(buf, n0, fs)
        out = buf[:, :m]
        carry = buf[:, m:].copy()
        if tones:
            t = np.arange(n0, n1) / fs
            for f, amp, phase in tones:
                out += amp[:, None] * np.sin(2.0 * math.pi * f * t[None, :] + phase[:, None])
        if mems:
            for axis in range(3):
                out[axis] += sigma * mems_rng[axis].standard_normal(m)
        yield out.copy()
        n0 = n1


def site_noise(
    rng: np.random.Generator,
    duration_s: float,
    fs_hz: float,
    footsteps: bool = True,
    slams: bool = True,
    hvac: bool = True,
    self_actuation: bool = True,
    *,
    mems: bool = True,
    **kwargs: float,
) -> np.ndarray:
    """Building noise, shape (3, n), m/s² (PHYSICS §22): :func:`site_noise_chunks` in one piece."""
    chunks = list(
        site_noise_chunks(
            rng, duration_s, fs_hz, footsteps, slams, hvac, self_actuation, chunk_s=float(duration_s), mems=mems, **kwargs
        )
    )
    return np.concatenate(chunks, axis=1)
