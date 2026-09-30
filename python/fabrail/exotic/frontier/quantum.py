"""NV-centre gap sensing and gradiometer noise presets (documented-only item; FRONTIER §F10).

**NV gap sensing** (frontier research §3f; PHYSICS §13). A field sensor in the gap reads
the gap through the field slope, so its noise maps to gap noise as

    δg = g0·δB/B,    δB_rms = ASD·√(bandwidth)

For the DRAWER case (g0 = 2 mm, B = 0.4 T, 2 kHz) the 0.3 µm hum requirement allows
δB ≤ 60 µT rms, about 1.3 µT/√Hz; an NV magnetometer at 10 nT/√Hz gives about 2 nm of
gap noise, so quantum sensing is over-specified for gap control (rail roughness and
PWM ripple dominate) and the missing-orders figure is 0. Caveats [UNCERTAIN]: at 0.4 T
the ODMR line moves from 2.87 GHz to about 14 GHz, and off-axis fields cost contrast, so
NV belongs in fringe-field spots.

**Gradiometer presets** feed :mod:`fabrail.exotic.frontier.pegs`: the demonstrated TOBA
Phase-I/II values, the Phase-III target and the final TOBA target (frontier research
§3a, §3f). Their role is PEGS gradiometry, not P-wave triggering; the atom-interferometer
gradiometer preset sits at the PEGS need and is a target, not a demonstration. Every
gradiometer preset reports the PEGS gap: 5 orders between the need and the best
demonstrated. Wiring presets into the core gap loop is roadmap (FRONTIER §1).

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import NamedTuple

from . import pegs
from ._record import FrontierRecord, make_record, orders

__all__ = ["GRADIOMETER_PRESETS", "NV_PRESETS", "SensorPreset", "gradiometer_preset", "nv_gap_noise"]


class SensorPreset(NamedTuple):
    """One noise preset: ASD, unit, status ("demonstrated" or "target") and source."""

    asd: float
    unit: str
    status: str
    source: str


#: NV magnetometer presets, T/√Hz (frontier research §3f; FRONTIER §F10; PHYSICS §13).
NV_PRESETS: Mapping[str, SensorPreset] = {
    "nv_lab_wolf2015": SensorPreset(0.9e-12, "T/√Hz", "demonstrated", "Wolf et al. 2015 PRX (1e11 NV centres)"),
    "nv_practical": SensorPreset(10e-9, "T/√Hz", "demonstrated", "frontier research §3f: 1–10 nT/√Hz [UNCERTAIN]"),
}

#: Gravity-gradient presets for PEGS, s⁻²/√Hz (frontier research §3a, §3f; TOBA review
#: arXiv 2412.01323; Shimoda et al. 2021; FRONTIER §F4, §F10).
GRADIOMETER_PRESETS: Mapping[str, SensorPreset] = {
    "toba_phase1": SensorPreset(1e-8, "1/(s²·√Hz)", "demonstrated", "TOBA Phase-I 2010, 0.1 Hz"),
    "toba_phase2": SensorPreset(pegs.ASD_BEST_DEMONSTRATED, "1/(s²·√Hz)", "demonstrated", "TOBA Phase-II, 3–8 Hz"),
    "toba_phase3_target": SensorPreset(pegs.ASD_NEEDED, "1/(s²·√Hz)", "target", "TOBA Phase-III target; Shimoda 2021"),
    "toba_final_target": SensorPreset(1e-19, "1/(s²·√Hz)", "target", "final TOBA, 10 m bars at 4 K"),
    "atom_gradiometer_target": SensorPreset(
        pegs.ASD_NEEDED, "1/(s²·√Hz)", "target", "atom-interferometer gradiometer at the PEGS need [UNCERTAIN]"
    ),
}

#: DRAWER example (frontier research §3f): nominal gap, pole field, loop bandwidth and the
#: gap-noise requirement (the core 0.3 µm hum target; PHYSICS §13).
G0_DRAWER_M = 2.0e-3
B_DRAWER_T = 0.4
BANDWIDTH_HZ = 2000.0
GAP_NOISE_SPEC_M = 0.3e-6


def nv_gap_noise(
    asd_t_per_rthz: float = NV_PRESETS["nv_practical"].asd,
    g0_m: float = G0_DRAWER_M,
    b_t: float = B_DRAWER_T,
    bandwidth_hz: float = BANDWIDTH_HZ,
    spec_m: float = GAP_NOISE_SPEC_M,
) -> FrontierRecord:
    """Gap noise of a field sensor, δg = g0·δB/B (FRONTIER §F10; PHYSICS §13).

    Returns:
        A record with ``db_rms_t``, ``gap_noise_m`` (≈ 2.2 nm for 10 nT/√Hz),
        ``asd_required_t_per_rthz`` (≈ 1.3 µT/√Hz for 0.3 µm) and ``margin_orders``.
    """
    for name, v in (
        ("asd_t_per_rthz", asd_t_per_rthz),
        ("g0_m", g0_m),
        ("b_t", b_t),
        ("bandwidth_hz", bandwidth_hz),
        ("spec_m", spec_m),
    ):
        if not (math.isfinite(v) and v > 0.0):
            raise ValueError(f"{name} must be finite and > 0 (got {v!r})")
    db = asd_t_per_rthz * math.sqrt(bandwidth_hz)
    dg = g0_m * db / b_t
    asd_req = spec_m * b_t / g0_m / math.sqrt(bandwidth_hz)
    return make_record(
        "quantum",
        "quantum.nv_gap_noise",
        orders(asd_t_per_rthz / asd_req),
        (
            "NV-centre gap sensing: documented only; over-specified for gap control "
            f"({dg * 1e9:.2g} nm against the {spec_m * 1e6:g} µm requirement); noise-ASD presets are roadmap."
        ),
        {"asd_t_per_rthz": asd_t_per_rthz, "g0_m": g0_m, "b_t": b_t, "bandwidth_hz": bandwidth_hz, "spec_m": spec_m},
        {
            "db_rms_t": db,
            "gap_noise_m": dg,
            "db_allowed_rms_t": spec_m * b_t / g0_m,
            "asd_required_t_per_rthz": asd_req,
            "margin_orders": math.log10(asd_req / asd_t_per_rthz),
        },
    )


def gradiometer_preset(name: str = "atom_gradiometer_target") -> FrontierRecord:
    """A gravity-gradient noise preset and its PEGS detection time for Mw 7 at 140 km (FRONTIER §F10).

    Returns:
        A record with the preset (``asd_per_rthz``, ``status``, ``source``) and
        ``t_det_s``/``before_p`` for the PEGS reference case. Missing orders: the PEGS gap
        (need against the best demonstrated), 5.
    """
    if name not in GRADIOMETER_PRESETS:
        raise ValueError(f"unknown gradiometer preset {name!r}; choose from {sorted(GRADIOMETER_PRESETS)}")
    preset = GRADIOMETER_PRESETS[name]
    det = pegs.detection(preset.asd)
    short = orders(pegs.ASD_BEST_DEMONSTRATED / pegs.ASD_NEEDED)
    return make_record(
        "quantum",
        "quantum.gradiometer_preset",
        short,
        (
            f"Gradiometer noise preset {name} ({preset.status}): the PEGS use needs {pegs.ASD_NEEDED:.0e}/√Hz, "
            f"best demonstrated {pegs.ASD_BEST_DEMONSTRATED:.0e}/√Hz; noise-ASD presets are roadmap."
        ),
        {"name": name},
        {
            "asd_per_rthz": preset.asd,
            "unit": preset.unit,
            "status": preset.status,
            "source": preset.source,
            "t_det_s": det["t_det_s"],
            "before_p": det["before_p"],
            "presets": sorted(GRADIOMETER_PRESETS),
        },
    )
