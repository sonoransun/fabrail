"""Rail irregularity profiles (core §B.8; PHYSICS §10; FAB-LEV-010).

A space series (``axis: "space"``, channels ``z`` and ``y``) for the scenario's
``series.rail``: span deflection at L/``deflection_ratio``, joint steps and random
roughness. The defaults are the core design's irregularity case (L/1150, 0.3 mm
joints, 20 µm rms; CRIT G3). numpy only; every draw comes from ``rng``.

The vertical offset z(x) (positive up; a sagging rail drops) is the sum of (physics
perspective §1.8; PHYSICS §10):

- sag half-sines between supports every ``span_m``: −δ·sin(π·s/L) with δ = L/ratio
  and s the position within the span (support offset drawn from ``rng``);
- joint steps every ``joint_spacing_m``: segment offsets form a bounded walk whose
  steps have magnitudes U(⅓, 1)·``joint_step_m`` (0.1–0.3 mm for 0.3 mm) and turn
  back toward zero once the offset passes ±½·``joint_step_m`` (so it stays within
  ±1.5·``joint_step_m``), each step smoothed over 1 mm with a C¹ smoothstep;
- band-limited roughness with a flat wavenumber spectrum over wavelengths 5–500 mm,
  random phases, scaled numerically to exactly ``roughness_rms_m``.

The lateral offset y(x) carries the lateral misalignment steps (``lateral_step_m``,
the ``design.rail.lateral_step_m`` preset value) at the joints, smoothed the same way.
The simulator interpolates the 1 mm grid with C¹ Hermite splines
(``fabrail_plant::rail``).

Owner: W0-C2 (stub), then A9 (Wave 1).
"""

from __future__ import annotations

import math

import numpy as np

__all__ = ["irregularity_profile"]

#: Roughness wavelength band, m (physics perspective §1.8: 20 µm rms over λ 5–500 mm).
_ROUGH_LAMBDA_M = (0.005, 0.5)
#: Joint-step smoothing length, m (physics perspective §1.8: "smoothed over 1 mm").
_JOINT_SMOOTH_M = 1e-3


def _smoothstep(u: np.ndarray) -> np.ndarray:
    """C¹ ramp 0 → 1 over u ∈ [0, 1] (3u² − 2u³)."""
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def _steps(rng: np.random.Generator, x: np.ndarray, spacing_m: float, step_m: float) -> np.ndarray:
    """Joint offsets: a bounded walk (each step toward zero once past ±step/2), smoothed over 1 mm."""
    out = np.zeros_like(x)
    if spacing_m <= 0.0 or step_m <= 0.0:
        return out
    x0 = float(rng.uniform(0.0, spacing_m))
    joints = np.arange(x[0] + x0, x[-1], spacing_m)
    offset = 0.0
    for xj in joints:
        mag = float(rng.uniform(1.0 / 3.0, 1.0)) * step_m
        sign = float(rng.choice((-1.0, 1.0))) if abs(offset) < 0.5 * step_m else -math.copysign(1.0, offset)
        offset_new = offset + sign * mag
        out += (offset_new - offset) * _smoothstep((x - xj) / _JOINT_SMOOTH_M)
        offset = offset_new
    return out


def irregularity_profile(
    rng: np.random.Generator,
    length_m: float,
    span_m: float,
    deflection_ratio: float = 1150.0,
    joint_step_m: float = 3e-4,
    roughness_rms_m: float = 2e-5,
    dx_m: float = 1e-3,
    *,
    joint_spacing_m: float | None = None,
    lateral_step_m: float = 1e-4,
) -> tuple[np.ndarray, np.ndarray]:
    """Vertical and lateral rail offsets ``(z_m, y_m)`` sampled every ``dx_m`` over ``length_m``.

    Sample k is at x = k·``dx_m`` from the profile start (the caller places it with the
    series ``x0_m``). ``joint_spacing_m`` defaults to ``span_m`` (a joint at every
    support); 0 means a continuous rail (no joints, as the DRAWER preset). Draw order:
    sag support offset, vertical joints, lateral joints, roughness phases.
    """
    if not (length_m > 0.0 and span_m > 0.0 and dx_m > 0.0 and deflection_ratio > 0.0):
        raise ValueError("irregularity_profile: length_m, span_m, dx_m and deflection_ratio must be > 0")
    if joint_step_m < 0.0 or roughness_rms_m < 0.0 or lateral_step_m < 0.0:
        raise ValueError("irregularity_profile: step and roughness amplitudes must be >= 0")
    n = int(math.floor(length_m / dx_m + 1e-9)) + 1
    x = np.arange(n, dtype=np.float64) * dx_m
    spacing = span_m if joint_spacing_m is None else float(joint_spacing_m)

    # Sag: half-sine per span, δ = L/ratio (PHYSICS §10).
    delta = span_m / deflection_ratio
    s0 = float(rng.uniform(0.0, span_m))
    z = -delta * np.abs(np.sin(np.pi * (x + s0) / span_m))
    z += _steps(rng, x, spacing, joint_step_m)
    y = _steps(rng, x, spacing, lateral_step_m)

    # Roughness: flat spectrum in wavenumber over λ 5–500 mm, exact rms after scaling.
    if roughness_rms_m > 0.0 and n > 2:
        n_fft = 1 << int(n - 1).bit_length()
        k = np.fft.rfftfreq(n_fft, dx_m)  # cycles/m
        band = (k >= 1.0 / _ROUGH_LAMBDA_M[1]) & (k <= 1.0 / _ROUGH_LAMBDA_M[0])
        phi = rng.uniform(0.0, 2.0 * np.pi, size=k.shape[0])
        rough = np.fft.irfft(np.where(band, np.exp(1j * phi), 0.0), n_fft)[:n]
        std = float(np.std(rough))
        if std > 0.0:
            z += (rough - float(np.mean(rough))) * (roughness_rms_m / std)
    return z, y
