"""Gallery example 8: ``villa_quake``, "a house that closes its own drawers" (plan §4b).

The flagship scene: a hillside villa with a kitchen of six DRAWERs and two cabinet
sliders, a pantry sliding DOOR on the egress path, a salon WALL off it and a HAVEN-R art
plinth, all on one ``fabrail.scene/1`` building scene
(:func:`fabrail._native.exo_scene_run`): one synthetic ground record through a building
model (fixed-base shear building 🔶, conventional FPS isolation at T = 3 s 🔶, HAVEN-F
reduced platform 🧪, SENTINEL supertall tower 🧪), every core device on the real closed
loop (``fabrail_sim::Session``: the ``fabrail-rt`` executive over the f64 plant, voltage
coil mode), the house-level EEW fusion and the house choreographer
(``fabrail-exo`` ``choreo``; not a rated safety function), and the contents of every
drawer as sliding blocks in the vector friction cone.

**Label: simulated replay.** Every alert is "synthetic/injected, not a live EEW feed".
The badge is 🧪 overall (the choreographer and inertial smoothing are research tier);
every panel carries its own badge (FAB-EXA-002).

Wording (SPECIFICATION_EXAMPLES §0): *smooth* is a C² reference with arrival ≤ 1 mm/s
and overshoot ≤ 0.02 mm; *calculated* is a deterministic plan computed at alert time
with every deadline met or the miss reported with a reason; *log-damped* is the
terminus capture d(t) = d_b·e^(−t/τ), a straight line on the semilog inset. The only
permitted contents wording is "no Coulomb slip event in the sliding-block model".

**Ground motion** (stated in every run). The waveform, the P/S timing and the P-phase
Pd are the Boore (2003) stochastic synthesis of :func:`fabrail.pwave.boore_ps` (P
scaled so Pd follows Wu–Zhao at the hypocentral distance, the detector's calibration).
Its absolute S level is low for this magnitude and distance (no crustal site
amplification in the synthesis; W1/B3 note), so the S phase is scaled to a stated
scenario PGA: the ``VILLA_EVENTS`` value, labelled with its ground-motion percentile
([UNCERTAIN] scenario choice; see ``ground_note``). The building model's topographic
factor then applies (hillside site).

**Scene time.** The earthquake origin is at scene time ``T_ORIGIN_S`` = 25 s so the
on-site STA/LTA detector has its 30 s LTA window filled before the P wave; every time the
record and the VILLA_RUN block quote is **seconds after the origin** (t_P ≈ 10.2 s,
t_S ≈ 17.5 s for the warned event).

Owner: E5 (Wave 2b).
"""

from __future__ import annotations

import json
import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

__all__ = [
    "BUILDINGS",
    "BUS_BUDGET_W",
    "EXAMPLES",
    "META",
    "SCENE_PROVENANCE",
    "T_ORIGIN_S",
    "VARIANTS",
    "VILLA_EVENTS",
    "VillaRun",
    "build_scene",
    "run_scene",
    "villa_example",
]

#: Standard gravity, m/s² (CGPM 1901; PHYSICS §1).
G0 = 9.80665

#: The mandatory alert label (``SCENE_PROVENANCE`` of fabrail-exo-sim; FAB-EXA-002).
SCENE_PROVENANCE = "synthetic/injected, not a live EEW feed"

#: The record label (plan §4b).
LABEL = "simulated replay"

#: Scene time of the earthquake origin, s: the on-site detector's LTA window is 30 s
#: (``control.seismic.detector.lta_n`` = 3000 at 100 Hz), so the P wave (origin + 10.2 s)
#: arrives after the warm-up (PHYSICS §19).
T_ORIGIN_S = 25.0

#: Villa scene variants run by the example (scene-1 ``variant``).
VARIANTS: tuple[str, ...] = ("warned", "near_field")

#: Building configurations and their panel badges (plan §4b).
BUILDINGS: dict[str, str] = {
    "fixed_base": "🔶",
    "fps_isolated": "🔶",
    "haven_f": "🧪",
    "tower_sentinel": "🧪",
}

#: The house DC motion-bus budget, W (Wave 2 lead decision D10). A 48 V DC bus behind a
#: 1 kW supply (≈ 21 A), the size of one dedicated kitchen-appliance circuit, feeds every
#: device's own bus converter; the choreographer schedules its motions under it and the
#: scene sums the devices' measured bus draw (truth) in registry order at every sync
#: step (FAB-EXA-005). [UNCERTAIN] design choice.
BUS_BUDGET_W = 1000.0

#: Scenario events (plan §4b; SPECIFICATION_EXAMPLES §2). ``pga_h_g`` is the stated
#: scenario PGA of the S phase at the site before the topographic factor, labelled with
#: its ground-motion percentile ([UNCERTAIN]: an 84th-percentile-type level for M7.2 at
#: 60 km and a median-type level at 15 km, stiff-soil hillside; the synthesis supplies
#: the waveform and the P phase). Near-field: R_epi 15 km at 10 km depth gives
#: R_hyp 18.0 km and S−P = 2.15 s (V_P 6.0, V_S 3.5 km/s; the plan's 2.29 s is the 12 km
#: depth value).
VILLA_EVENTS: dict[str, dict[str, float]] = {
    "warned": {
        "magnitude": 7.2,
        "r_epi_km": 60.0,
        "depth_km": 12.0,
        "pga_h_g": 0.13,
        "percentile": 84.0,
        "alert_latency_s": 7.5,
        "duration_s": 150.0,
    },
    "near_field": {
        "magnitude": 7.2,
        "r_epi_km": 15.0,
        "depth_km": 10.0,
        "pga_h_g": 0.30,
        "percentile": 50.0,
        "alert_latency_s": math.nan,  # blind zone: no network alert before the S wave
        "duration_s": 60.0,
    },
}

#: Owner actions after the event, s after the origin. The core runs SHAKING → QUIET →
#: INSPECT (QUIET dwell ``control.seismic.fusion.quiet_s`` = 30 s); for the warned seed-42
#: record INSPECT is reached ≈ 101 s after the origin, so the manual INSPECT reset comes at
#: +106 s (the scene refuses a reset outside INSPECT and reports it), with a local restore
#: before it and a remote-only restore after it (both rejected, FAB-EXA-015).
OPERATOR_T_S: dict[str, float] = {
    "restore_early": 104.0,
    "inspect_reset": 106.0,
    "restore_remote": 107.0,
    "restore": 108.0,
}

#: Synthesis stress drop of the Boore record, bar (core design §C.5 default).
STRESS_DROP_BAR = 30.0

#: Ground record sample rate, Hz.
GROUND_FS_HZ = 500.0

#: Topographic amplification of the hillside site (stated in every run; [UNCERTAIN]:
#: typical ridge/slope factors 1.2–1.4, EN 1998-5 Annex A).
TOPO_FACTOR = 1.25

#: S-arrival standard deviation of the house plan's deadline, s (plan §4b: deadlines
#: 13.2–14.8 s for σ between 2 and 1 s; 1.5 s gives 14.0 s).
S_ETA_SIGMA_S = 1.5

#: Villa floors: lower (gallery, plinth), main (kitchen, pantry, salon), upper.
_FLOORS = [
    {"level": 0, "name": "lower (gallery)", "z_m": 0.0},
    {"level": 1, "name": "main (kitchen, pantry, salon)", "z_m": 3.4},
    {"level": 2, "name": "upper", "z_m": 6.8},
]
_MAIN_LEVEL = 1

#: SENTINEL penthouse tower: 60 storeys of 4 m, T1 7 s (plan §4b).
_TOWER_FLOORS = [{"level": k, "name": f"L{k}", "z_m": 4.0 * k} for k in range(0, 61, 6)]


def _building(config: str) -> dict[str, Any]:
    """Scene-1 building block for ``config`` (plan §4b; parameters stated)."""
    common = {"zeta": 0.05, "topo_factor": TOPO_FACTOR}
    if config == "fixed_base":
        return {"config": "fixed_base", "period_s": 0.25, "floors": _FLOORS, **common}
    if config == "fps_isolated":
        return {
            "config": "fps_isolated",
            "period_s": 0.25,
            "iso_period_s": 3.0,
            "mu": 0.05,
            "iso_stroke_m": 0.3,
            "floors": _FLOORS,
            **common,
        }
    if config == "haven_f":
        return {
            "config": "haven_f",
            "period_s": 0.25,
            "iso_period_s": 6.0,
            "iso_zeta": 0.3,
            "stroke_m": 0.9,
            "control": "passive",
            "floors": _FLOORS,
            **common,
        }
    if config == "tower_sentinel":
        return {
            "config": "tower_sentinel",
            "period_s": 7.0,
            "zeta": 0.02,
            "zeta_s": 0.01,
            "hmd_mass_ratio": 0.01,
            "hmd_mode": "seismic_passive",
            "topo_factor": 1.0,
            "floors": _TOWER_FLOORS,
        }
    raise ValueError(f"unknown building {config!r} (one of {', '.join(BUILDINGS)})")


def _device_level(config: str) -> int:
    return _TOWER_FLOORS[-1]["level"] if config == "tower_sentinel" else _MAIN_LEVEL


def _plinth_level(config: str) -> int:
    return _TOWER_FLOORS[-1]["level"] if config == "tower_sentinel" else 0


# ---- Devices ---------------------------------------------------------------------------------


def _slider_overrides() -> dict[str, Any]:
    """The 8 kg cabinet slider derived from DRAWER (gallery example 2's derivation,
    ``core_examples._pantry_overrides``; latch fitted, EgressRole none)."""
    from .. import require_native
    from .core_examples import _pantry_overrides

    return _pantry_overrides(require_native().Params.preset("drawer"))


#: Drawer latch design floor acceleration and retention (a fitted seismic latch:
#: x_retention ≥ m_rated·a_design with a_design 1 g, 48 kg → 471 N; stated design choice).
_LATCH_RETENTION_N = 500.0


def _drawer(
    i: int,
    x_open_m: float,
    contents: list[dict[str, Any]],
    *,
    level: int,
    hero: bool,
    fidelity: str,
    latch: bool = False,
    smoothing: bool = False,
    heading_deg: float = 90.0,
    commands: list[dict[str, Any]] | None = None,
    position: tuple[float, float] = (0.0, 0.0),
) -> dict[str, Any]:
    ov: dict[str, Any] = {}
    if latch:
        ov["design.landing.seismic_latch.enabled"] = True
        ov["design.landing.seismic_latch.x_retention_n"] = _LATCH_RETENTION_N
    if smoothing:
        ov["control.motion.inertial.enable"] = True
    d: dict[str, Any] = {
        "id": f"drawer_{i}",
        "label": f"kitchen drawer {i}",
        "class": "drawer",
        "level": level,
        "room": "kitchen",
        "position_m": list(position),
        "heading_deg": heading_deg,
        "egress": "none",
        "fidelity": fidelity,
        "coil": "voltage",
        "hero": hero and fidelity == "carriage6",
        "contents": contents,
        "initial_state": {"lev": "levitated" if x_open_m > 0.0 else "seated", "x_m": x_open_m},
        "overrides": ov,
        "commands": list(commands or []),
    }
    return d


@dataclass(frozen=True)
class SceneOptions:
    """What distinguishes one villa scene from another (all defaults: the main run)."""

    variant: str = "warned"
    building: str = "fixed_base"
    #: "fused" (network alert + local P) or "onsite_only" (local P only; FAB-EXA-004).
    alert: str = "fused"
    #: Inertial smoothing / SmoothRideThrough admitted (research tier; FAB-EXA-010).
    smoothing: bool = False
    #: Carriage6 for the devices that move at the alert (the hero drawers and slider);
    #: False puts every device on Travel1d.
    carriage6: bool = True
    #: Scene duration override, s (None: the event's own).
    duration_s: float | None = None
    #: Operator actions after the event (INSPECT reset, restore attempts).
    operator: bool = True
    #: Mains lost at the S arrival (ride-through power-loss case, FAB-EXA-013).
    mains_loss_at_s: bool = False
    #: Only these device ids (None: the full villa).
    devices: tuple[str, ...] | None = None
    #: Extra scenario events per device id (test harness: hands, obstacles).
    device_events: tuple[tuple[str, tuple[Mapping[str, Any], ...]], ...] = ()
    #: Record rate, Hz.
    record_rate_hz: float = 100.0
    #: Master seed.
    seed: int = 42


def event_times(variant: str) -> dict[str, float]:
    """P/S arrival times after the origin, s (V_P 6.0, V_S 3.5 km/s; PHYSICS §19)."""
    from .. import pwave

    ev = VILLA_EVENTS[variant]
    r = math.hypot(ev["r_epi_km"], ev["depth_km"])
    return {
        "r_hyp_km": r,
        "t_p_s": r / pwave.V_P_KMPS,
        "t_s_s": r / pwave.V_S_KMPS,
        "s_minus_p_s": r / pwave.V_S_KMPS - r / pwave.V_P_KMPS,
    }


def ground_record(variant: str, seed: int, duration_s: float) -> tuple[np.ndarray, dict[str, Any]]:
    """The villa ground record (3, n) at :data:`GROUND_FS_HZ` in scene time, and its info.

    Boore (2003) synthesis (:func:`fabrail.pwave.boore_ps`, P phase at Wu–Zhao Pd) seeded
    from ``SeedSequence(seed)``; the S phase (from 0.5 s before t_S) scaled to the
    scenario PGA ``VILLA_EVENTS[variant]['pga_h_g']`` (module docs). The building model
    applies the topographic factor.
    """
    from .. import pwave

    ev = VILLA_EVENTS[variant]
    tt = event_times(variant)
    rng = np.random.default_rng(np.random.SeedSequence(int(seed)))
    rec = pwave.boore_ps(
        rng,
        ev["magnitude"],
        ev["r_epi_km"],
        ev["depth_km"],
        fs_hz=GROUND_FS_HZ,
        duration_s=float(duration_s),
        t_p_s=T_ORIGIN_S + tt["t_p_s"],
        stress_drop_bar=STRESS_DROP_BAR,
    )
    a = np.array(rec.a_xyz_mps2, dtype=float)
    i_s = max(0, int((rec.t_s_s - 0.5) * GROUND_FS_HZ))
    pga_synth = float(np.max(np.hypot(a[0, i_s:], a[1, i_s:])))
    target = ev["pga_h_g"] * G0
    scale = target / pga_synth if pga_synth > 0 else 1.0
    a[:, i_s:] *= scale
    info = {
        "magnitude": ev["magnitude"],
        "r_epi_km": ev["r_epi_km"],
        "depth_km": ev["depth_km"],
        "r_hyp_km": tt["r_hyp_km"],
        "origin_scene_s": T_ORIGIN_S,
        "t_p_s": rec.t_p_s - T_ORIGIN_S,
        "t_s_s": rec.t_s_s - T_ORIGIN_S,
        "s_minus_p_s": rec.t_s_s - rec.t_p_s,
        "percentile": ev["percentile"],
        "pga_h_synth_g": pga_synth / G0,
        "pga_h_g": target / G0,
        "s_phase_scale": scale,
        "pga_z_g": float(np.max(np.abs(a[2]))) / G0,
        "pd_target_m": rec.meta.get("pd_target_m"),
        "stress_drop_bar": STRESS_DROP_BAR,
        "topo_factor": TOPO_FACTOR,
        "ground_note": (
            "Boore (2003) stochastic synthesis (fabrail.pwave.boore_ps; P phase scaled to Wu-Zhao Pd); "
            f"S phase scaled from {pga_synth / G0:.3f} g to the scenario PGA_h {target / G0:.2f} g "
            f"({ev['percentile']:.0f}th-percentile-type level, [UNCERTAIN] scenario choice); "
            f"topographic factor {TOPO_FACTOR} applied by the building model"
        ),
    }
    return a, info


def build_scene(opts: SceneOptions = SceneOptions()) -> tuple[dict[str, Any], dict[str, bytes], dict[str, Any]]:
    """The ``fabrail.scene/1`` document, its series bytes and the ground info for ``opts``."""
    if opts.variant not in VILLA_EVENTS:
        raise ValueError(f"variant {opts.variant!r}: the villa runs {', '.join(VILLA_EVENTS)}")
    if opts.building not in BUILDINGS:
        raise ValueError(f"building {opts.building!r}: one of {', '.join(BUILDINGS)}")
    if opts.alert not in ("fused", "onsite_only"):
        raise ValueError(f"alert {opts.alert!r}: 'fused' or 'onsite_only'")
    ev = VILLA_EVENTS[opts.variant]
    dur = float(opts.duration_s if opts.duration_s is not None else ev["duration_s"])
    a, info = ground_record(opts.variant, opts.seed, dur)
    t0 = T_ORIGIN_S
    lvl = _device_level(opts.building)
    moving = "carriage6" if opts.carriage6 else "travel1d"
    near = opts.variant == "near_field"
    t_p = t0 + info["t_p_s"]

    # Kitchen: two open drawers and an open slider (the owner is unloading), the rest
    # seated; near-field: drawer 1 gliding closed, drawer 2 gliding open at the P wave.
    d1_cmd = [{"t_s": round(t_p - 0.6, 3), "cmd": "close", "source": "local"}] if near else []
    d2_cmd = [{"t_s": round(t_p - 0.3, 3), "cmd": "open", "source": "local"}] if near else []
    devices: list[dict[str, Any]] = [
        _drawer(1, 0.30, [{"kind": "glassware", "count": 8}, {"kind": "stemware", "count": 4, "name": "wine glasses"}],
                level=lvl, hero=True, fidelity=moving, smoothing=opts.smoothing, commands=d1_cmd, position=(1.0, 4.0)),
        _drawer(2, 0.08 if near else 0.15, [{"kind": "porcelain", "count": 6, "name": "plates"}],
                level=lvl, hero=True, fidelity=moving, smoothing=opts.smoothing, commands=d2_cmd, position=(1.6, 4.0)),
        _drawer(3, 0.0, [{"kind": "glassware", "count": 8, "name": "tumblers"}], level=lvl, hero=False, fidelity="travel1d",
                latch=True, position=(2.2, 4.0)),
        _drawer(4, 0.0, [{"kind": "porcelain", "count": 8, "name": "bowls"}], level=lvl, hero=False, fidelity="travel1d",
                position=(1.0, 4.6)),
        _drawer(5, 0.0, [{"kind": "lined", "count": 10, "name": "cutlery tray"}], level=lvl, hero=False, fidelity="travel1d",
                position=(1.6, 4.6)),
        _drawer(6, 0.0, [{"kind": "stemware", "count": 6, "name": "coupes"}], level=lvl, hero=False, fidelity="travel1d",
                heading_deg=0.0, position=(2.2, 4.6)),
    ]
    slider = _slider_overrides()
    devices += [
        {
            "id": "slider_1",
            "label": "cabinet slider (open)",
            "class": "drawer",
            "preset": "cabinet_slider",
            "level": lvl,
            "room": "kitchen",
            "position_m": [3.0, 4.2],
            "heading_deg": 0.0,
            "egress": "none",
            "fidelity": moving,
            "coil": "voltage",
            "hero": opts.carriage6,
            "overrides": slider,
            "contents": [{"kind": "porcelain", "count": 4, "name": "cups"}],
            "initial_state": {"lev": "levitated", "x_m": 0.2},
        },
        {
            "id": "slider_2",
            "label": "cabinet slider (closed)",
            "class": "drawer",
            "preset": "cabinet_slider",
            "level": lvl,
            "room": "kitchen",
            "position_m": [3.6, 4.2],
            "heading_deg": 0.0,
            "egress": "none",
            "fidelity": "travel1d",
            "coil": "voltage",
            "overrides": slider,
            "contents": [{"kind": "glassware", "count": 6, "name": "jars"}],
            "initial_state": {"lev": "seated", "x_m": 0.0},
        },
        {
            "id": "pantry_door",
            "label": "pantry sliding door (egress path)",
            "class": "door",
            "level": lvl,
            "room": "pantry",
            "position_m": [4.5, 2.0],
            "heading_deg": 0.0,
            "egress": "non_rated",
            "fidelity": moving,
            "coil": "voltage",
            "policy": "stop_and_land",
            "initial_state": {"lev": "levitated", "x_m": 0.6},
        },
        {
            # Repositioned by the owner just before the event (a WALL rests landed; the
            # core locks a WALL on the Seismic→Landed transition, W2b note).
            "id": "salon_wall",
            "label": "salon partition wall",
            "class": "wall",
            "level": lvl,
            "room": "salon",
            "position_m": [7.0, 3.0],
            "heading_deg": 90.0,
            "egress": "none",
            "fidelity": "travel1d",
            "coil": "voltage",
            "policy": "land_and_lock",
            "contents": [{"kind": "lined", "count": 2, "name": "vases on the ledge"}],
            "initial_state": {"lev": "levitated", "x_m": 2.0},
            "commands": [{"t_s": round(t0 + 1.0, 3), "cmd": "go_to", "x_m": 1.5, "source": "local"}],
        },
        {
            "id": "plinth",
            "label": "HAVEN-R art plinth",
            "class": "haven_r",
            "level": _plinth_level(opts.building),
            "room": "gallery",
            "position_m": [9.0, 6.0],
            "heading_deg": 0.0,
            "egress": "none",
            "fidelity": "platform3dof",
            "contents": [{"kind": "custom", "name": "sculpture", "mu_s": 0.5, "b_over_h": 0.3, "mass_kg": 80.0}],
        },
    ]
    if opts.devices is not None:
        keep = set(opts.devices)
        unknown = keep - {d["id"] for d in devices}
        if unknown:
            raise ValueError(f"devices: unknown id(s) {sorted(unknown)}")
        devices = [d for d in devices if d["id"] in keep]

    feeds: list[dict[str, Any]] = []
    if opts.alert == "fused" and math.isfinite(ev["alert_latency_s"]):
        feeds.append(
            {
                "kind": "network",
                "t_s": round(t0 + ev["alert_latency_s"], 6),
                "id": 1,
                "magnitude": ev["magnitude"],
                "r_epi_km": ev["r_epi_km"],
                "depth_km": ev["depth_km"],
                "origin_t_s": t0,
                "mmi": 7.0,
            }
        )
    operator: list[dict[str, Any]] = []
    if opts.operator and dur >= OPERATOR_T_S["restore"] + 15.0:
        operator = [
            # Before the INSPECT reset: rejected (FAB-EXA-015).
            {"t_s": t0 + OPERATOR_T_S["restore_early"], "action": "restore", "source": "local"},
            {"t_s": t0 + OPERATOR_T_S["inspect_reset"], "action": "inspect_reset", "source": "local"},
            # Remote-only: rejected.
            {"t_s": t0 + OPERATOR_T_S["restore_remote"], "action": "restore", "source": "remote"},
            {"t_s": t0 + OPERATOR_T_S["restore"], "action": "restore", "source": "local"},
        ]
    power: dict[str, Any] = {"p_bus_max_w": BUS_BUDGET_W, "ups": False}
    if opts.mains_loss_at_s:
        power["mains_loss_t_s"] = round(t0 + info["t_s_s"], 3)
    scene = {
        "schema": "fabrail.scene/1",
        "name": f"villa_quake.{opts.variant}.{opts.building}",
        "doc": f"{LABEL}: {info['ground_note']}",
        "seed": int(opts.seed),
        "duration_s": dur,
        "sync_hz": 100,
        "variant": opts.variant,
        "building": _building(opts.building),
        "ground": {
            "source": "boore_ps",
            "series": "series:ground",
            "magnitude": ev["magnitude"],
            "r_epi_km": ev["r_epi_km"],
            "depth_km": ev["depth_km"],
            "origin_t_s": t0,
            "t_p_s": t0 + info["t_p_s"],
            "t_s_s": t0 + info["t_s_s"],
            "percentile": ev["percentile"],
            "kappa_s": 0.04,
        },
        "series": {"ground": {"axis": "time", "fs_hz": GROUND_FS_HZ, "t0_s": 0.0, "channels": ["ax", "ay", "az"]}},
        "devices": devices,
        "alert": {
            "provenance": SCENE_PROVENANCE,
            "local_p": True,
            "feeds": feeds,
            "s_eta_sigma_s": S_ETA_SIGMA_S,
        },
        "power": power,
        "choreo": {"enable": True, "smoothing": bool(opts.smoothing), "deadline_quantile": 0.05, "deadline_margin_s": 1.0},
        "operator": operator,
        "contents_mu_sweep_frac": 0.3,
        "record": {"rate_hz": float(opts.record_rate_hz), "devices": "all", "channels": "default"},
    }
    series = {"ground": np.ascontiguousarray(a, dtype="<f8").tobytes()}
    return scene, series, info


# ---- Running ---------------------------------------------------------------------------------


@dataclass
class VillaRun:
    """One finished villa scene."""

    opts: SceneOptions
    scene: dict[str, Any]
    ground: dict[str, Any]
    meta: dict[str, Any]
    rate_hz: float
    devices: list[dict[str, Any]]
    building: dict[str, np.ndarray]
    events: list[tuple[float, str | None, str, float, float]]
    kpis: dict[str, float]
    plan: dict[str, Any]
    wall_s: float
    _chan: dict[str, dict[str, np.ndarray]] = field(default_factory=dict)

    @property
    def t(self) -> np.ndarray:
        """Record time, s (scene time)."""
        return self.building["t_s"]

    def channel(self, device: str, name: str) -> np.ndarray:
        return self._chan[device][name]

    def device(self, device_id: str) -> dict[str, Any]:
        return next(d for d in self.devices if d["id"] == device_id)

    def dev_events(self, device: str | None, prefix: str = "") -> list[tuple[float, str, float, float]]:
        return [(e[0], e[2], e[3], e[4]) for e in self.events if e[1] == device and e[2].startswith(prefix)]


def run_scene(opts: SceneOptions = SceneOptions(), *, workers: int | None = None) -> VillaRun:
    """Builds and runs one villa scene (``fabrail._native.exo_scene_run``)."""
    from .. import require_native

    nat = require_native()
    scene, series, info = build_scene(opts)
    ev_json = json.dumps({k: [dict(e) for e in v] for k, v in opts.device_events}) if opts.device_events else None
    t0 = time.perf_counter()
    raw = nat.exo_scene_run(json.dumps(scene), series, workers, False, ev_json)
    wall = time.perf_counter() - t0
    chans: dict[str, dict[str, np.ndarray]] = {}
    devices = []
    for d in raw["devices"]:
        chans[d["id"]] = {n: np.frombuffer(b, dtype=dt) for n, (b, dt, _u) in d["channels"].items()}
        devices.append({k: d[k] for k in ("id", "class", "policy", "not_secured", "kpis")})
    building = {n: np.frombuffer(b, dtype=dt) for n, (b, dt, _u) in raw["building"].items()}
    return VillaRun(
        opts=opts,
        scene=scene,
        ground=info,
        meta=json.loads(raw["meta"]),
        rate_hz=float(raw["rate_hz"]),
        devices=devices,
        building=building,
        events=[tuple(e) for e in raw["events"]],
        kpis=dict(raw["kpis"]),
        plan=json.loads(raw["plan"]),
        wall_s=wall,
        _chan=chans,
    )
