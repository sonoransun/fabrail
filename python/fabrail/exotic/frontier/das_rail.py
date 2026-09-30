"""🧪 DAS along the rail: the rail's fibre as a dense P-wave array (documented-only item).

FRONTIER §F10. Distributed acoustic sensing (DAS) turns a telecom fibre into strain
channels about 10 m long over 40–125 km (frontier research §3f; Marra 2018, Zhan 2021
[REF-MEM]); earthquake-detection pilots exist, rail integration is research. A fibre laid
along a long rail is a dense array: beamforming across an aperture L resolves the
apparent slowness from the moveout

    Δt = L/v_app                    (17 ms over 100 m at v_app = 6 km/s)

which, sampled at 1 kHz, gives back-azimuth and slowness within about a second (frontier
research §3f; PHYSICS §20). A DAS feed enters fusion as one feed kind
(``AlertFeedKind::Das``, EXOTIC §X10), never as a sole trigger for motion.

Missing orders: 0 (DAS is deployed for sensing; the rail array is research).

numpy only (FAB-EXO-002). Every public function returns a stamped
:class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from ._record import FrontierRecord, make_record

__all__ = ["APERTURE_M", "CHANNEL_M", "FS_HZ", "V_APP_MPS", "beamform_moveout"]

#: Beamforming aperture, m (frontier research §3f: "17 ms over 100 m"; FRONTIER §F10).
APERTURE_M = 100.0
#: Apparent P speed, m/s (core fusion default v_p 6.0 km/s; pwave.V_P_KMPS; PHYSICS §20).
V_APP_MPS = 6000.0
#: Sampling rate, Hz (frontier research §3f: "at 1 kHz"; FRONTIER §F10).
FS_HZ = 1000.0
#: DAS channel (gauge) length, m (frontier research §3f: about 10 m; FRONTIER §F10).
CHANNEL_M = 10.0


def beamform_moveout(
    aperture_m: float = APERTURE_M,
    v_app_mps: Sequence[float] | np.ndarray | float = V_APP_MPS,
    fs_hz: float = FS_HZ,
    channel_m: float = CHANNEL_M,
) -> FrontierRecord:
    """Moveout Δt = L/v_app across the rail array and whether it is resolvable (FRONTIER §F10).

    Returns:
        A record with ``moveout_s``, ``moveout_samples`` (Δt·fs), ``resolvable``
        (≥ 1 sample), ``slowness_s_per_km`` and ``n_channels`` (L/channel).
    """
    v = np.asarray(v_app_mps, dtype=float)
    if aperture_m <= 0.0 or fs_hz <= 0.0 or channel_m <= 0.0 or np.any(v <= 0.0):
        raise ValueError("aperture_m, v_app_mps, fs_hz and channel_m must be > 0")
    dt = aperture_m / v
    samples = dt * fs_hz

    def scalar(x: np.ndarray) -> float | np.ndarray:
        return float(x) if x.ndim == 0 else x

    return make_record(
        "das_rail",
        "das_rail.beamform_moveout",
        0.0,
        (
            "DAS along the rail: research tier (DAS itself is deployed for sensing); "
            "one fusion feed, never a sole trigger."
        ),
        {"aperture_m": aperture_m, "v_app_mps": v_app_mps, "fs_hz": fs_hz, "channel_m": channel_m},
        {
            "moveout_s": scalar(dt),
            "moveout_samples": scalar(samples),
            "resolvable": bool(np.all(samples >= 1.0)),
            "slowness_s_per_km": scalar(1000.0 / v),
            "n_channels": int(aperture_m // channel_m),
        },
    )
