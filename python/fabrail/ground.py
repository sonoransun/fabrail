"""Ground-motion synthesis and spectra (core §B.8; PHYSICS §19).

Stochastic records from Kanai–Tajimi / Clough–Penzien spectra under a
Jennings–Housner–Tsai envelope, baseline correction, response spectra, spectrum
matching to the ASCE 7 design spectrum, and the floor amplification used by the
structure tier. numpy/scipy only; every random draw comes from the ``rng`` argument,
so a record is a pure function of (seed, arguments).

The chain used by :func:`synthesize` (PHYSICS §19):

1. :func:`stationary`: spectral representation (Shinozuka & Deodatis 1991) of a
   zero-mean Gaussian process with the one-sided PSD G(f) of :func:`kanai_tajimi_psd`
   or :func:`clough_penzien_psd`: a(t) = Σ_k √(2·G(f_k)·Δf)·cos(2πf_k·t + φ_k), φ_k
   uniform on [0, 2π), evaluated with one inverse FFT; the period 1/Δf is at least the
   record length (core design: 2π/Δω ≥ duration);
2. the JHT amplitude envelope (:func:`jht_envelope`);
3. zero-phase 0.05 Hz baseline correction with zero pads (:func:`baseline_correct`);
4. numeric PGA scaling: the record is multiplied so max|a| equals ``pga_g``·g exactly
   (core design §C.5 "PGA is scaled numerically").

Owner: W0-C2 (stub; ``asce7_sa`` and ``floor_amplification`` are complete closed
forms), then A9 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np
from scipy import linalg, signal

__all__ = [
    "G0_MPS2",
    "JHT",
    "SHEAR_BEAM_GAMMA",
    "SOILS",
    "SOIL_FILTERS",
    "SoilFilter",
    "asce7_sa",
    "baseline_correct",
    "clough_penzien_psd",
    "floor_amplification",
    "floor_response",
    "jht_envelope",
    "kanai_tajimi_psd",
    "nigam_jennings",
    "resolve_jht",
    "response_spectrum",
    "spectrum_match",
    "stationary",
    "synthesize",
]

#: Standard gravity, m/s² (3rd CGPM 1901; ISO 80000-3). Every "g" in this package.
G0_MPS2 = 9.80665


@dataclass(frozen=True)
class SoilFilter:
    """Kanai–Tajimi ground filter (ω_g, ζ_g) and Clough–Penzien high-pass (ω_f, ζ_f)."""

    omega_g_rad_s: float
    zeta_g: float
    omega_f_rad_s: float
    zeta_f: float


#: Site classes for the filter parameters. Source: firm (15 rad/s, 0.6), medium
#: (10 rad/s, 0.4, the default), soft (5 rad/s, 0.2), with the Clough–Penzien high-pass
#: at ω_f = 0.1·ω_g and ζ_f = ζ_g, as tabulated by Der Kiureghian & Neuenhofer (1992,
#: EESD 21) and adopted by the core design §C.5 and the physics perspective §3
#: (PHYSICS §19). ω_f matters: the isolation Monte Carlo outcome is sensitive to it.
SOIL_FILTERS: dict[str, SoilFilter] = {
    "soft": SoilFilter(5.0, 0.2, 0.5, 0.2),
    "medium": SoilFilter(10.0, 0.4, 1.0, 0.4),
    "firm": SoilFilter(15.0, 0.6, 1.5, 0.6),
}

#: Site class names (keys of :data:`SOIL_FILTERS`).
SOILS: tuple[str, ...] = tuple(SOIL_FILTERS)

#: JHT default for a 40 s record: rise 2 s, strong phase to 12 s, decay 0.2 /s. Source:
#: core design §C.5 and the physics perspective §3 ("t1 2 s, t2 12 s, c 0.2 (M6.5–7);
#: duration 40 s"), after Jennings, Housner & Tsai (1968) (PHYSICS §19).
_JHT_REF = (2.0, 12.0, 0.2)
_JHT_REF_DURATION_S = 40.0

#: Modal participation factor of the first mode of a uniform shear beam with
#: φ(z) = sin(πz/2h) normalised to 1 at the roof: Γ = 4/π (Chopra, Dynamics of
#: Structures, uniform shear-building modes; PHYSICS §19).
SHEAR_BEAM_GAMMA = 4.0 / math.pi


def _soil(soil: str | SoilFilter) -> SoilFilter:
    if isinstance(soil, SoilFilter):
        return soil
    try:
        return SOIL_FILTERS[soil]
    except KeyError:
        raise ValueError(f"soil: expected one of {', '.join(SOILS)} (got {soil!r})") from None


def _next_pow2(n: int) -> int:
    return 1 << max(1, int(n - 1).bit_length())


@dataclass(frozen=True)
class JHT:
    """Jennings–Housner–Tsai (1968) amplitude envelope: (t/t_rise)² rise, flat strong phase, exp decay.

    ``None`` fields are chosen from the record duration D by :func:`resolve_jht`: the
    40 s reference (2 s, 12 s, 0.2 /s; core design §C.5) is scaled in time,
    t_rise = 2·D/40, t_strong_end = 12·D/40, decay = 0.2·40/D, so the envelope shape in
    t/D (and its end value e^(−5.6) ≈ 0.4 %) is the reference's for every duration.
    """

    t_rise_s: float | None = None
    t_strong_end_s: float | None = None
    decay_per_s: float | None = None


def resolve_jht(envelope: JHT | None = None, duration_s: float | None = None) -> JHT:
    """``envelope`` with every ``None`` field filled from ``duration_s`` (default 40 s).

    Raises ``ValueError`` unless 0 < t_rise ≤ t_strong_end and decay > 0.
    """
    d = _JHT_REF_DURATION_S if duration_s is None else float(duration_s)
    if not (math.isfinite(d) and d > 0.0):
        raise ValueError(f"resolve_jht: duration_s must be > 0 (got {duration_s!r})")
    env = envelope or JHT()
    scale = d / _JHT_REF_DURATION_S
    t1 = _JHT_REF[0] * scale if env.t_rise_s is None else float(env.t_rise_s)
    t2 = _JHT_REF[1] * scale if env.t_strong_end_s is None else float(env.t_strong_end_s)
    c = _JHT_REF[2] / scale if env.decay_per_s is None else float(env.decay_per_s)
    if not (0.0 < t1 <= t2 and c > 0.0 and math.isfinite(t2) and math.isfinite(c)):
        raise ValueError(f"resolve_jht: need 0 < t_rise <= t_strong_end and decay > 0 (got {t1}, {t2}, {c})")
    return JHT(t1, t2, c)


def kanai_tajimi_psd(f_hz: np.ndarray, soil: str | SoilFilter = "medium", s0: float = 1.0) -> np.ndarray:
    """Kanai–Tajimi one-sided PSD at ``f_hz`` (PHYSICS §19).

    G(f) = s0·(ω_g⁴ + 4ζ_g²ω_g²ω²)/((ω_g² − ω²)² + 4ζ_g²ω_g²ω²) with ω = 2πf (Kanai
    1957; Tajimi 1960). ``s0`` is the one-sided bedrock white-noise PSD in
    (m/s²)²/Hz; G(0) = s0 and G(f_g) = s0·(1 + 4ζ_g²)/(4ζ_g²).
    """
    p = _soil(soil)
    w = 2.0 * np.pi * np.asarray(f_hz, dtype=np.float64)
    wg2 = p.omega_g_rad_s**2
    c = 4.0 * p.zeta_g**2 * wg2 * w**2
    return float(s0) * (wg2**2 + c) / ((wg2 - w**2) ** 2 + c)


def clough_penzien_psd(f_hz: np.ndarray, soil: str | SoilFilter = "medium", s0: float = 1.0) -> np.ndarray:
    """Clough–Penzien PSD (Kanai–Tajimi with the low-frequency high-pass filter; PHYSICS §19).

    G_CP(f) = G_KT(f)·ω⁴/((ω_f² − ω²)² + 4ζ_f²ω_f²ω²) (Clough & Penzien, Dynamics of
    Structures): the second filter removes the KT PSD's non-zero value at f = 0, which
    would otherwise give unbounded ground displacement.
    """
    p = _soil(soil)
    w = 2.0 * np.pi * np.asarray(f_hz, dtype=np.float64)
    wf2 = p.omega_f_rad_s**2
    hp = w**4 / ((wf2 - w**2) ** 2 + 4.0 * p.zeta_f**2 * wf2 * w**2)
    return kanai_tajimi_psd(f_hz, p, s0) * hp


def _psd(model: str) -> Callable[..., np.ndarray]:
    if model == "kt":
        return kanai_tajimi_psd
    if model == "cp":
        return clough_penzien_psd
    raise ValueError(f"model: expected 'kt' or 'cp' (got {model!r})")


def stationary(
    rng: np.random.Generator,
    duration_s: float,
    fs_hz: float,
    soil: str | SoilFilter = "medium",
    model: str = "cp",
    s0: float = 1.0,
) -> np.ndarray:
    """A stationary Gaussian sample with the KT (``"kt"``) or CP (``"cp"``) PSD, m/s² (PHYSICS §19).

    Spectral representation (Shinozuka & Deodatis 1991, Appl. Mech. Rev. 44(4)):
    a(t) = Σ_k √(2·G(f_k)·Δf)·cos(2πf_k·t + φ_k) on the grid f_k = k·Δf, Δf = fs/N_fft
    with N_fft the next power of two ≥ the sample count (so the period 1/Δf ≥ the
    duration), φ_k uniform on [0, 2π) from ``rng``; the DC and Nyquist terms are zero.
    The variance is Σ G(f_k)·Δf, and the periodogram of a full period equals G at the
    grid frequencies exactly.
    """
    n = int(round(float(duration_s) * float(fs_hz)))
    if n < 2 or not math.isfinite(fs_hz) or fs_hz <= 0.0:
        raise ValueError(f"stationary: need duration_s·fs_hz >= 2 samples (got {duration_s!r} s at {fs_hz!r} Hz)")
    psd = _psd(model)
    n_fft = _next_pow2(n)
    df = float(fs_hz) / n_fft
    f = np.arange(n_fft // 2 + 1, dtype=np.float64) * df
    amp = np.sqrt(2.0 * psd(f, soil, s0) * df)
    amp[0] = 0.0
    amp[-1] = 0.0
    phi = rng.uniform(0.0, 2.0 * np.pi, size=f.shape[0])
    spec = (n_fft / 2.0) * amp * np.exp(1j * phi)
    return np.fft.irfft(spec, n=n_fft)[:n]


def jht_envelope(t_s: np.ndarray, envelope: JHT | None = None, duration_s: float | None = None) -> np.ndarray:
    """The JHT envelope at ``t_s`` (peak 1; PHYSICS §19).

    e(t) = (t/t1)² for 0 ≤ t < t1, 1 for t1 ≤ t ≤ t2, exp(−c·(t − t2)) after t2, and 0
    for t < 0 (Jennings, Housner & Tsai 1968). ``None`` fields follow
    :func:`resolve_jht` (the 40 s reference scaled to ``duration_s``).
    """
    env = resolve_jht(envelope, duration_s)
    t1, t2, c = float(env.t_rise_s), float(env.t_strong_end_s), float(env.decay_per_s)  # type: ignore[arg-type]
    t = np.asarray(t_s, dtype=np.float64)
    out = np.where(t < t1, (np.clip(t, 0.0, None) / t1) ** 2, 1.0)
    return np.where(t > t2, np.exp(-c * (t - t2)), out)


def trapezoid_integral(x: np.ndarray, fs_hz: float) -> np.ndarray:
    """Cumulative trapezoidal integral from rest (first value 0) at sample rate ``fs_hz``."""
    x = np.asarray(x, dtype=np.float64)
    out = np.zeros_like(x)
    if x.shape[0] > 1:
        out[1:] = np.cumsum(0.5 * (x[1:] + x[:-1])) / float(fs_hz)
    return out


def _end_functionals(n: int, dt: float) -> np.ndarray:
    """Rows c_v, c_d with c_v·a = v(T) and c_d·a = d(T) under :func:`trapezoid_integral` twice."""
    w = np.ones(n)
    w[0] = w[-1] = 0.5
    c_v = dt * w
    # d(T) = dt·Σ_k w_k·v_k and ∂v_k/∂a_j = dt·(1 if k > j, ½ if k == j ≥ 1, ½ if j == 0 < k).
    tail = np.concatenate([np.cumsum(w[::-1])[::-1][1:], [0.0]])  # Σ_{k>j} w_k
    c_d = dt * dt * (0.5 * w + tail)
    c_d[0] = dt * dt * 0.5 * float(np.sum(w[1:]))
    return np.vstack([c_v, c_d])


def baseline_correct(a_mps2: np.ndarray, fs_hz: float, f_hp_hz: float = 0.05) -> np.ndarray:
    """High-pass baseline correction so velocity and displacement do not drift (PHYSICS §19).

    1. Zero-phase (forward-backward) 4th-order Butterworth high-pass at ``f_hp_hz``
       (0.05 Hz; core design §C.5), applied to the record padded with zeros of length
       1.5·n/f_c on both sides (Boore 2005, "On pads and filters", BSSA 95(2), for
       acausal filtering) and cut back to the input length.
    2. Cutting the pads off discards the filter's pre- and post-event response, which
       leaves a small end velocity (a few % of PGV) and hence a displacement drift. A
       minimum-norm acceleration adjustment δ = W²Cᵀ(CW²Cᵀ)⁻¹(−C·a), weighted by the
       record's 1 s moving RMS W (so the quiet start stays at rest), makes the
       trapezoidal end velocity and end displacement exactly zero (C holds the two
       linear end functionals; a least-squares baseline in the sense of the classic
       polynomial-fit corrections, e.g. Converse & Brady 1992, USGS OFR 92-296A).

    Offline input generation only — the controller never sees an acausal filter.
    """
    a = np.asarray(a_mps2, dtype=np.float64)
    if a.ndim != 1:
        raise ValueError("baseline_correct: a_mps2 must be one-dimensional")
    if not (math.isfinite(f_hp_hz) and 0.0 < f_hp_hz < 0.5 * fs_hz):
        raise ValueError(f"baseline_correct: need 0 < f_hp_hz < fs_hz/2 (got {f_hp_hz!r} at {fs_hz!r} Hz)")
    n = a.shape[0]
    order = 4
    pad = int(math.ceil(1.5 * order / f_hp_hz * fs_hz))
    sos = signal.butter(order, f_hp_hz, btype="highpass", fs=fs_hz, output="sos")
    padded = np.concatenate([np.zeros(pad), a, np.zeros(pad)])
    y = signal.sosfiltfilt(sos, padded, padtype=None)[pad : pad + n]
    if n < 4 or not np.any(y):
        return y
    win = max(1, int(round(float(fs_hz))))
    rms = np.sqrt(np.convolve(y * y, np.ones(win) / win, mode="same"))
    w2 = rms * rms + 1e-12 * float(np.max(rms * rms))
    c = _end_functionals(n, 1.0 / float(fs_hz))
    lam = np.linalg.solve((c * w2) @ c.T, -(c @ y))
    return y + w2 * (c.T @ lam)


def synthesize(
    rng: np.random.Generator,
    duration_s: float,
    fs_hz: float,
    soil: str = "medium",
    pga_g: float = 0.5,
    envelope: JHT | None = None,
    model: str = "cp",
) -> np.ndarray:
    """A baseline-corrected horizontal acceleration record, m/s², scaled to ``pga_g`` (PHYSICS §19).

    :func:`stationary` → :func:`jht_envelope` → :func:`baseline_correct` → numeric
    scaling so max|a| = ``pga_g``·g exactly (the scaling is linear, so the baseline
    correction is kept). One horizontal component, shape (n,).
    """
    if not (math.isfinite(pga_g) and pga_g > 0.0):
        raise ValueError(f"synthesize: pga_g must be > 0 (got {pga_g!r})")
    a = stationary(rng, duration_s, fs_hz, soil, model)
    t = np.arange(a.shape[0], dtype=np.float64) / float(fs_hz)
    a = baseline_correct(a * jht_envelope(t, envelope, duration_s), fs_hz)
    return a * (pga_g * G0_MPS2 / float(np.max(np.abs(a))))


def asce7_sa(period_s: np.ndarray | float, sds_g: float = 1.0, sd1_g: float = 0.6, tl_s: float = 8.0) -> np.ndarray | float:
    """ASCE 7-16 §11.4.6 design response spectrum S_a(T), in g.

    With T0 = 0.2·S_D1/S_DS and T_S = S_D1/S_DS:
    S_a = S_DS·(0.4 + 0.6·T/T0) for T < T0; S_DS for T0 ≤ T ≤ T_S; S_D1/T for
    T_S < T ≤ T_L; S_D1·T_L/T² for T > T_L. Scalar in, scalar out.
    """
    if not (sds_g > 0.0 and sd1_g > 0.0 and tl_s > 0.0):
        raise ValueError("asce7_sa: sds_g, sd1_g and tl_s must be positive")
    t = np.asarray(period_s, dtype=np.float64)
    if np.any(t < 0.0):
        raise ValueError("asce7_sa: period_s must be >= 0")
    t0 = 0.2 * sd1_g / sds_g
    ts = sd1_g / sds_g
    with np.errstate(divide="ignore", invalid="ignore"):
        sa = np.where(
            t < t0,
            sds_g * (0.4 + 0.6 * t / t0),
            np.where(
                t <= ts, sds_g, np.where(t <= tl_s, sd1_g / np.where(t > 0, t, 1.0), sd1_g * tl_s / np.where(t > 0, t, 1.0) ** 2)
            ),
        )
    return float(sa) if np.ndim(period_s) == 0 else sa


def nigam_jennings(omega_rad_s: float, zeta: float, dt_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Nigam–Jennings (1969) exact recursion matrices for ü + 2ζωu̇ + ω²u = −a_g(t).

    For ground acceleration linear within each step,
    [u, u̇]_{k+1} = A·[u, u̇]_k + B·[a_g,k, a_g,k+1] exactly (Nigam & Jennings 1969,
    BSSA 59(2) 909–922, eqs. for the piecewise-linear excitation; 0 ≤ ζ < 1).
    Returns (A, B), both 2×2.
    """
    w, z, h = float(omega_rad_s), float(zeta), float(dt_s)
    if not (w > 0.0 and 0.0 <= z < 1.0 and h > 0.0):
        raise ValueError(f"nigam_jennings: need omega > 0, 0 <= zeta < 1, dt > 0 (got {w}, {z}, {h})")
    sq = math.sqrt(1.0 - z * z)
    wd = w * sq
    e = math.exp(-z * w * h)
    s = math.sin(wd * h)
    c = math.cos(wd * h)
    a = np.array(
        [
            [e * (z / sq * s + c), e * s / wd],
            [-w / sq * e * s, e * (c - z / sq * s)],
        ]
    )
    k1 = (2.0 * z * z - 1.0) / (w * w * h)
    k2 = 2.0 * z / (w**3 * h)
    b11 = e * ((k1 + z / w) * s / wd + (k2 + 1.0 / w**2) * c) - k2
    b12 = -e * (k1 * s / wd + k2 * c) - 1.0 / w**2 + k2
    b21 = e * ((k1 + z / w) * (c - z / sq * s) - (k2 + 1.0 / w**2) * (wd * s + z * w * c)) + 1.0 / (w * w * h)
    b22 = -e * (k1 * (c - z / sq * s) - k2 * (wd * s + z * w * c)) - 1.0 / (w * w * h)
    return a, np.array([[b11, b12], [b21, b22]])


def _nj_filters(omega: float, zeta: float, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """IIR coefficients (b_u, b_v, a) of the Nigam–Jennings recursion from a_g to u and u̇.

    With x_{k+1} = A·x_k + b1·a_k + b2·a_{k+1} and x at rest, U(z)/A_g(z) =
    e1ᵀ(zI − A)⁻¹(b1 + b2·z) (and e2ᵀ… for u̇); the denominator is z² − tr(A)·z + det(A).
    """
    a, b = nigam_jennings(omega, zeta, dt)
    b1, b2 = b[:, 0], b[:, 1]
    den = np.array([1.0, -(a[0, 0] + a[1, 1]), a[0, 0] * a[1, 1] - a[0, 1] * a[1, 0]])
    num_u = np.array([b2[0], b1[0] - a[1, 1] * b2[0] + a[0, 1] * b2[1], -a[1, 1] * b1[0] + a[0, 1] * b1[1]])
    num_v = np.array([b2[1], a[1, 0] * b2[0] + b1[1] - a[0, 0] * b2[1], a[1, 0] * b1[0] - a[0, 0] * b1[1]])
    return num_u, num_v, den


def _sdof_response(a_mps2: np.ndarray, fs_hz: float, omega: float, zeta: float) -> tuple[np.ndarray, np.ndarray]:
    """Relative displacement and velocity of an SDOF at rest before the record (Nigam–Jennings).

    A zero sample is prepended so the system starts at rest with the excitation rising
    linearly from 0 to a_0 over one step (exact for the piecewise-linear record).
    """
    num_u, num_v, den = _nj_filters(omega, zeta, 1.0 / fs_hz)
    x = np.concatenate([[0.0], np.asarray(a_mps2, dtype=np.float64)])
    return signal.lfilter(num_u, den, x)[1:], signal.lfilter(num_v, den, x)[1:]


def response_spectrum(a_mps2: np.ndarray, fs_hz: float, periods_s: np.ndarray, zeta: float = 0.05) -> np.ndarray:
    """Pseudo-acceleration response spectrum of a record (exact piecewise-linear SDOF recursion).

    PSA(T) = ω²·max|u| with u from the Nigam–Jennings (1969) recursion
    (:func:`nigam_jennings`), in the record's units (m/s²); T = 0 gives the PGA.
    """
    a = np.asarray(a_mps2, dtype=np.float64)
    periods = np.atleast_1d(np.asarray(periods_s, dtype=np.float64))
    if np.any(periods < 0.0) or not np.all(np.isfinite(periods)):
        raise ValueError("response_spectrum: periods_s must be finite and >= 0")
    out = np.empty(periods.shape[0])
    pga = float(np.max(np.abs(a)))
    for i, t in enumerate(periods):
        if t == 0.0:
            out[i] = pga
            continue
        w = 2.0 * math.pi / t
        u, _ = _sdof_response(a, fs_hz, w, zeta)
        out[i] = w * w * float(np.max(np.abs(u)))
    return out if np.ndim(periods_s) else out[0]


def spectrum_match(
    rng: np.random.Generator,
    a_mps2: np.ndarray,
    fs_hz: float,
    target_sa_g: np.ndarray,
    periods_s: np.ndarray,
    zeta: float = 0.05,
    iterations: int = 8,
) -> np.ndarray:
    """Adjust a record so its response spectrum matches ``target_sa_g`` at ``periods_s`` (PHYSICS §19).

    Iterative Fourier-amplitude scaling (physics perspective §3; the SIMQKE approach of
    Gasparini & Vanmarcke 1976): each iteration multiplies the record's Fourier
    amplitudes by the ratio target/PSA, interpolated log-log in frequency f = 1/T and
    held constant beyond the period range, clipped to [0.2, 5] per iteration, then
    re-applies :func:`baseline_correct`. Deterministic: ``rng`` is accepted so every
    synthesis stage shares the generator argument, and is not drawn from.
    """
    del rng  # the Fourier scaling is deterministic
    a = np.asarray(a_mps2, dtype=np.float64).copy()
    periods = np.asarray(periods_s, dtype=np.float64)
    target = np.asarray(target_sa_g, dtype=np.float64) * G0_MPS2
    if periods.shape != target.shape or periods.ndim != 1 or periods.shape[0] < 2:
        raise ValueError("spectrum_match: periods_s and target_sa_g need equal 1-D shapes (>= 2 points)")
    if np.any(periods <= 0.0) or np.any(target <= 0.0):
        raise ValueError("spectrum_match: periods and targets must be > 0")
    order = np.argsort(1.0 / periods)
    log_f = np.log(1.0 / periods[order])
    n = a.shape[0]
    n_fft = _next_pow2(2 * n)
    f = np.fft.rfftfreq(n_fft, 1.0 / fs_hz)
    log_fq = np.log(np.maximum(f, f[1] if f.shape[0] > 1 else 1.0))
    for _ in range(int(iterations)):
        sa = response_spectrum(a, fs_hz, periods, zeta)
        ratio = np.clip(target / np.maximum(sa, 1e-30), 0.2, 5.0)
        scale = np.exp(np.interp(log_fq, log_f, np.log(ratio[order])))
        a = np.fft.irfft(np.fft.rfft(a, n_fft) * scale, n_fft)[:n]
        a = baseline_correct(a, fs_hz)
    return a


def floor_amplification(z_over_h: float = 0.5) -> float:
    """Floor acceleration amplification 1 + 2·z/h (ASCE 7-16 eq. 13.3-1; ``fabrail_plant::structure``).

    ``z_over_h`` is the storey height over the roof height, in [0, 1].
    """
    x = float(z_over_h)
    if not 0.0 <= x <= 1.0:
        raise ValueError(f"floor_amplification: z_over_h must be in [0, 1] (got {z_over_h!r})")
    return 1.0 + 2.0 * x


def floor_response(
    a_ground_mps2: np.ndarray,
    fs_hz: float,
    period_s: float,
    zeta: float = 0.05,
    z_over_h: float = 0.5,
    gamma: float = SHEAR_BEAM_GAMMA,
) -> np.ndarray:
    """Absolute floor acceleration of a fixed-base building in its first mode, m/s² (PHYSICS §19).

    The physics perspective's ``BuildingFilter`` option (first-mode SDOF, period T1,
    5 %, mode shape φ(z)): with D the relative response of an SDOF (T1, ζ) to the
    ground record (Nigam–Jennings, exact for piecewise-linear input), the floor at
    height z sees a_f = a_g + Γ·φ(z)·D̈ with φ(z) = sin(π·z/2h) and Γ = 4/π for a uniform
    shear building (Chopra). One-way coupling (the building is unaffected by the rail).
    Components pass independently: ``a_ground_mps2`` of shape (n,) or (k, n).
    """
    if not 0.0 <= z_over_h <= 1.0:
        raise ValueError(f"floor_response: z_over_h must be in [0, 1] (got {z_over_h!r})")
    if not (period_s > 0.0 and 0.0 <= zeta < 1.0):
        raise ValueError(f"floor_response: need period_s > 0 and 0 <= zeta < 1 (got {period_s!r}, {zeta!r})")
    a = np.asarray(a_ground_mps2, dtype=np.float64)
    rows: Sequence[np.ndarray] = [a] if a.ndim == 1 else list(a)
    w = 2.0 * math.pi / float(period_s)
    gp = float(gamma) * math.sin(0.5 * math.pi * float(z_over_h))
    out = []
    for row in rows:
        u, v = _sdof_response(row, fs_hz, w, zeta)
        d_rel_acc = -row - 2.0 * zeta * w * v - w * w * u
        out.append(row + gp * d_rel_acc)
    return out[0] if a.ndim == 1 else np.vstack(out)


def _nj_expm(omega: float, zeta: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Reference (A, B) from the matrix exponential of the first-order-hold system (Van Loan 1978).

    Used by the tests to check :func:`nigam_jennings` (the two are the same exact
    solution for piecewise-linear input).
    """
    f = np.array([[0.0, 1.0], [-omega * omega, -2.0 * zeta * omega]])
    g = np.array([0.0, -1.0])
    m = np.zeros((4, 4))
    m[:2, :2] = f
    m[:2, 2] = g
    m[2, 3] = 1.0
    e = linalg.expm(m * dt)
    phi, e1, e2 = e[:2, :2], e[:2, 2], e[:2, 3]
    return phi, np.column_stack([e1 - e2 / dt, e2 / dt])
