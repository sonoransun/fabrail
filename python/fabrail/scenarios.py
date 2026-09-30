"""Scenario catalog: named scenarios built as ``fabrail.scenario/1`` specs (core §B.6, §B.8).

The scenario JSON is defined by ``fabrail_sim::scenario::ScenarioSpec`` and mirrored
by ``schemas/scenario-1.schema.json`` (W0-B3). Python owns synthesis (ground motion,
P+S, rail profiles, hand profiles; core §A.1): each :class:`ScenarioDef` builds the
spec dict plus any ``series`` bytes (channel-major little-endian f64), and
:func:`build` wraps them in a native ``Scenario``.

Seeds (core §A.4 rule 5): ``SeedSequence(cfg.seed).spawn(2)``; child 0 is the numpy
synthesis generator passed to ``ScenarioDef.build``, child 1 gives the Rust
``noise_seed`` (:func:`fabrail.ensemble.split_seed`). Ensemble members pass their own
:class:`fabrail.ensemble.MemberSeed` (``member=``).

Coil mode (plan §2b item 16): the spec's ``fidelity.coil`` is the single source;
builders set it from ``cfg.coil`` and :meth:`fabrail.config.RunConfig.params` sets the
matching ``control.rates.coil_mode_voltage``.

Fidelity (plant tier): every entry lists the tiers it supports, the first being the
recommended one (``fabrail scenarios`` prints it). :func:`resolve_fidelity` uses an
explicit ``fidelity=`` argument, else ``cfg.fidelity`` when it differs from the
``RunConfig`` default ``"carriage6"``, else the recommended tier — so the seismic
scenarios default to Travel1d and the thermal ramp to Heave1 (plan §2b item 13), and
``fidelity="carriage6"`` forces Carriage6 where supported. An unsupported tier is a
``ValueError``.

Class: the rail class is ``params.rail_class`` (the physics source); an entry that does
not apply to it is a ``ValueError``. Class-dependent values (travel, speed caps,
thresholds, terminus timing) are read from ``params``, never restated here.

Validation: :func:`spec` checks every spec against the shipped JSON schema
(:func:`validate_spec`, a draft 2020-12 subset validator — no jsonschema dependency)
and against the cross-field rules the Rust loader enforces (:func:`cross_check`), with
the same ``"path: reason"`` messages.

Research tier: entries whose ``requires`` is non-empty (isolation, inertial smoothing,
AMD) are meaningful only with those research parameters set by the caller
(``--set``); nothing here enables a research feature.

Hands (Wave 2, W1.5 P2): an intent-triggered automatic move follows a confirmed gesture
(push-and-go), so no catalog hand is an anchored grip (:func:`fabrail.human.anchored_grip`
is checked by :func:`catalog_errors`). ``push_open`` uses a push-and-release (option
``hand="hold"``: a walk-along grip), ``throw_soft_close`` a walk-along throw,
``detent_pass`` a position-led palm below the intent rate (:func:`fabrail.human.guide`)
and ``terminus_hand_replan`` a brief walk-along grip above the controller's hand threshold
F_th + 0.25·|F_cmd| and below the obstruction residual. ``governor_sabotage`` uses the core
fault ``m_hat_half`` once the schema accepts it, else the truth-magnet fallback (stated in
its info). ``closed_end_stop_under_shaking`` exercises decision D3's physical end stops.

Catalog check: :func:`catalog_errors` builds every entry for every class (optionally every
tier and coil) and returns each schema, cross-field, anchored-grip or native-loader
failure; ``[]`` means clean.

Owner: W0-C2 (stub), then A9 (Wave 1), S1 (Wave 2 scenario-builder fixes).
"""

from __future__ import annotations

import json
import math
import re
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from functools import cached_property, lru_cache
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from . import ground, human, pwave, rail, records
from .ensemble import MemberSeed, split_seed

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ._native import Params, Scenario
    from .config import RunConfig

__all__ = [
    "BASE_TICK_HZ",
    "CATALOG",
    "NAMES",
    "PLAN_NAMES",
    "PLANT_KEYS",
    "SCENARIO_SCHEMA",
    "SCENE_SCHEMA_PATH",
    "SCHEMA_PATH",
    "ScenarioDef",
    "SpecParts",
    "build",
    "build_spec",
    "catalog_errors",
    "cross_check",
    "info",
    "resolve_fidelity",
    "schema_errors",
    "spec",
    "to_json",
    "validate_spec",
]

#: Scenario JSON schema id (= ``fabrail_sim::SCENARIO_SCHEMA`` = ``_native.SCENARIO_SCHEMA``).
SCENARIO_SCHEMA = "fabrail.scenario/1"

#: The shipped JSON schemas (FAB-SW-015).
SCHEMA_PATH = Path(__file__).with_name("schemas") / "scenario-1.schema.json"
SCENE_SCHEMA_PATH = Path(__file__).with_name("schemas") / "scene-1.schema.json"

#: Catalog names frozen by core §B.8 (the minimum set; A9 may add more).
NAMES: tuple[str, ...] = (
    "hover",
    "lift",
    "unseat",
    "land",
    "payload_step",
    "push_open",
    "throw_soft_close",
    "detent_pass",
    "obstruction_chest",
    "obstruction_hand_edge",
    "lean",
    "piston_pulse",
    "smoke_dp",
    "power_loss",
    "control_loss_up",
    "control_loss_down",
    "rail_traverse",
    "lateral_push",
    "thermal_ramp",
    "governor_sabotage",
    "core_fault_iq",
    "payload_mismatch",
    "pwave_m5_r10",
    "pwave_m7_r20",
    "pwave_m78_r100",
    "network_alert",
    "alert_cancel",
    "elcentro_free",
    "isolation_kt",
    "vertical_pulse",
    "power_loss_at_s",
    "egress_eew",
    "harmonic_base",
    "noise_1h",
)

#: Additions from plan §4b (terminus, inertial smoothing, choreography), HAP-006 and the
#: Wave 2 lead decision D3 (physical end stops under floor shaking).
PLAN_NAMES: tuple[str, ...] = (
    "terminus_soft_close",
    "soft_open",
    "terminus_hand_replan",
    "choreographed_close",
    "inertial_smooth_fixedbase",
    "footsteps",
    "closed_end_stop_under_shaking",
)

#: Spec parts: the ``fabrail.scenario/1`` dict and its ``series`` bytes by name.
SpecParts = tuple[dict[str, object], dict[str, bytes]]

#: Base tick rate, Hz (``fabrail_types::BASE_TICK_HZ`` = ``_native.BASE_TICK_HZ``; PHYSICS §24).
BASE_TICK_HZ = 20_000

#: Base ticks per plant step: RK4 at 50 µs for Carriage6/Heave1, 1 ms for Travel1d and
#: Structure (core §C.1; PHYSICS §24). ``record.rate_hz`` must divide into whole steps.
_TIER_STEP_TICKS: dict[str, int] = {"carriage6": 1, "heave1": 1, "travel1d": 20, "structure": 20}

#: The ``RunConfig.fidelity`` default; it defers to the entry's recommended tier.
_DEFAULT_FIDELITY = "carriage6"

#: Keys accepted in ``cfg.plant`` (the scenario ``plant`` block's truth mismatch).
PLANT_KEYS: tuple[str, ...] = ("mass_scale", "br_scale", "leak_sigma", "c_rail_n_s_per_m", "f_hyst_n", "leak_area_m2")

#: Marker that separates the entry doc from its synthesis summary in ``spec["doc"]``.
_INFO_MARK = " | info: "


@dataclass(frozen=True)
class ScenarioDef:
    """One catalog entry.

    Attributes:
        name: catalog name.
        classes: rail classes it applies to.
        doc: one-line description (``fabrail scenarios``).
        build: ``build(params, cfg, rng, *, noise_seed=None, fidelity=None, **options)
            -> (spec_dict, series_bytes)``; ``rng`` is the numpy synthesis generator
            (child 0 of the seed split). The frozen three-argument call works.
        tiers: supported plant tiers, recommended first.
        options: keyword options the builder accepts (``spec(..., options=)``).
        slow_classes: classes whose run belongs to the ``slow`` test tier.
        requires: research-tier parameter settings the scenario is meant for; never
            applied automatically.
    """

    name: str
    classes: tuple[str, ...]
    doc: str
    build: Callable[..., SpecParts]
    tiers: tuple[str, ...] = ("carriage6",)
    options: tuple[str, ...] = ()
    slow_classes: tuple[str, ...] = ()
    requires: Mapping[str, Any] = field(default_factory=dict)


#: Catalog by name (every entry builds a schema-valid spec).
CATALOG: dict[str, ScenarioDef] = {}


# ---------------------------------------------------------------------------------
# Build context and helpers
# ---------------------------------------------------------------------------------


@dataclass
class _Ctx:
    """What a builder sees: parameters, config, generator and the resolved tier."""

    sdef: ScenarioDef
    params: Any
    cfg: RunConfig
    rng: np.random.Generator
    noise_seed: int
    tier: str
    options: dict[str, Any]
    cls: str

    def f(self, path: str) -> float:
        return float(self.params.get(path))

    def s(self, path: str) -> str:
        return str(self.params.get(path))

    @cached_property
    def derived(self) -> dict[str, float]:
        return {str(k): float(v) for k, v in dict(self.params.derived()).items()}

    def opt(self, key: str, default: Any) -> Any:
        return self.options.get(key, default)

    # Class geometry and limits (all from params).
    @property
    def x0(self) -> float:
        return self.f("design.rail.x_closed_m")

    @property
    def travel(self) -> float:
        return self.f("design.rail.travel_m")

    @property
    def m_star(self) -> float:
        return self.f("design.carriage.m_star_kg")

    @property
    def m_rated(self) -> float:
        return self.f("design.carriage.m_rated_kg")

    @property
    def v_open(self) -> float:
        return self.derived["v_open_cap_mps"]

    @property
    def v_close(self) -> float:
        return self.derived["v_close_cap_mps"]

    @property
    def a_max(self) -> float:
        return self.f("control.motion.a_max_mps2")

    @property
    def t_terminus(self) -> float:
        return self.derived["terminus_t_arrive_s"]

    @property
    def f_intent(self) -> float:
        return self.f("control.intent.f_th_n")


def _ceil_half(x: float) -> float:
    return math.ceil(2.0 * x - 1e-9) / 2.0


def _move_s(dist_m: float, v_mps: float, a_mps2: float) -> float:
    """Trapezoidal move time (jerk ignored), s."""
    d = abs(dist_m)
    if v_mps <= 0.0 or a_mps2 <= 0.0:
        return 0.0
    return d / v_mps + v_mps / a_mps2 if d >= v_mps * v_mps / a_mps2 else 2.0 * math.sqrt(d / a_mps2)


def _cmd(t_s: float, cmd: str, x_m: float | None = None) -> dict[str, Any]:
    ev: dict[str, Any] = {"kind": "command", "t_s": float(t_s), "cmd": cmd}
    if x_m is not None:
        ev["x_m"] = float(x_m)
    return ev


def _hold_to_run(ctx: _Ctx, t_s: float) -> list[dict[str, Any]]:
    """HEAVY moves only hold-to-run when unsensed (core design F5; SAF-003)."""
    if ctx.cls == "heavy" and ctx.s("design.presence") != "pl_d":
        return [{"kind": "hold_to_run", "t_s": float(t_s), "on": True}]
    return []


def _open_to(ctx: _Ctx, t_s: float, dist_m: float) -> dict[str, Any]:
    """A full ``open`` when the distance is the whole travel, else ``go_to``."""
    if dist_m >= ctx.travel - 1e-12:
        return _cmd(t_s, "open")
    return _cmd(t_s, "go_to", ctx.x0 + dist_m)


def _record_rate(ctx: _Ctx, cap_hz: float | None = None) -> float:
    rate = float(ctx.cfg.record_rate_hz)
    if cap_hz is not None:
        rate = min(rate, cap_hz)
    step = _TIER_STEP_TICKS[ctx.tier]
    d = BASE_TICK_HZ / rate
    if not (d >= 1.0 and abs(d - round(d)) <= 1e-9 * d and int(round(d)) % step == 0):
        raise ValueError(
            f"record.rate_hz: {BASE_TICK_HZ} / rate_hz must be a whole multiple of the {ctx.tier} plant step "
            f"({step} base ticks) (got {rate} Hz)"
        )
    return rate


def _time_series(a: np.ndarray, fs_hz: float, channels: Sequence[str], t0_s: float = 0.0) -> tuple[dict[str, Any], np.ndarray]:
    arr = np.atleast_2d(np.asarray(a, dtype=np.float64))
    if arr.shape[0] != len(channels):
        raise ValueError(f"series: {arr.shape[0]} rows for channels {list(channels)}")
    return {"axis": "time", "fs_hz": float(fs_hz), "t0_s": float(t0_s), "channels": list(channels)}, arr


def _encode(arr: np.ndarray) -> bytes:
    """Channel-major little-endian f64 (``fabrail_sim::scenario::encode_f64_le``)."""
    return np.ascontiguousarray(arr, dtype="<f8").tobytes()


def _spec(
    ctx: _Ctx,
    duration_s: float,
    *,
    initial: Mapping[str, Any] | None = None,
    events: Sequence[Mapping[str, Any]] = (),
    series: Mapping[str, tuple[dict[str, Any], np.ndarray]] | None = None,
    plant: Mapping[str, Any] | None = None,
    windows: Mapping[str, Sequence[float]] | None = None,
    rate_cap_hz: float | None = None,
    channels: object = "default",
    info: Mapping[str, Any] | None = None,
) -> SpecParts:
    """Assemble one spec; ``cfg.duration_s`` overrides the duration (later events drop, windows clip)."""
    dur = float(ctx.cfg.duration_s) if ctx.cfg.duration_s is not None else float(duration_s)
    evs = sorted((dict(e) for e in events if float(e["t_s"]) <= dur + 1e-12), key=lambda e: float(e["t_s"]))
    wins: dict[str, list[float]] = {}
    for k, (t0, t1) in (windows or {}).items():
        t1c = min(float(t1), dur)
        if float(t0) < t1c:
            wins[k] = [float(t0), t1c]
    plant_block: dict[str, Any] = dict(plant or {})
    for k, v in ctx.cfg.plant.items():
        if k not in PLANT_KEYS:
            raise ValueError(f"plant.{k}: unknown key (known: {', '.join(PLANT_KEYS)})")
        plant_block[k] = v
    doc = ctx.sdef.doc
    if info:
        doc = f"{doc}{_INFO_MARK}{json.dumps(dict(info), sort_keys=True, separators=(',', ':'))}"
    spec_d: dict[str, Any] = {
        "schema": SCENARIO_SCHEMA,
        "name": f"{ctx.cls}_{ctx.sdef.name}",
        "doc": doc[:4096],
        "duration_s": dur,
        "noise_seed": int(ctx.noise_seed),
        "fidelity": {"model": ctx.tier, "coil": ctx.cfg.coil, "eddy_lag": True, "kinematic_heave": True},
        "record": {"rate_hz": _record_rate(ctx, rate_cap_hz), "channels": channels},
        "initial": dict(initial or {}),
        "plant": plant_block,
        "series": {},
        "events": evs,
        "metrics": {"windows": wins},
    }
    blobs: dict[str, bytes] = {}
    for name, (meta, arr) in (series or {}).items():
        spec_d["series"][name] = meta
        blobs[name] = _encode(arr)
    return spec_d, blobs


def _levitated(ctx: _Ctx, x_m: float, **extra: object) -> dict[str, Any]:
    return {"lev": "levitated", "x_m": float(x_m), **extra}


def _park_x(ctx: _Ctx) -> float:
    """Where a hovering/levitated carriage sits when the class does not matter: DOOR closed, others mid-travel."""
    return ctx.x0 if ctx.cls == "door" else ctx.x0 + 0.5 * ctx.travel


# ---------------------------------------------------------------------------------
# Builders
# ---------------------------------------------------------------------------------


def _b_hover(ctx: _Ctx) -> SpecParts:
    dur = 10.0
    return _spec(ctx, dur, initial=_levitated(ctx, _park_x(ctx)), windows={"hover": [2.0, dur]})


_LIFT_S = {"door": 3.0, "wall": 5.0, "heavy": 8.0}  # covers the LEV-012 budgets 0.3 / 1 / 2 s with margin


def _b_lift(ctx: _Ctx) -> SpecParts:
    dur = _LIFT_S[ctx.cls]
    return _spec(ctx, dur, initial={"lev": "landed", "x_m": ctx.x0}, events=[_cmd(0.5, "lift")], windows={"lift": [0.5, dur]})


def _b_unseat(ctx: _Ctx) -> SpecParts:
    payload = float(ctx.opt("payload_kg", 0.0))
    if "mass_kg" in ctx.options:
        payload = float(ctx.options["mass_kg"]) - ctx.m_star
    dur = 1.5
    return _spec(
        ctx,
        dur,
        initial={"lev": "seated", "x_m": ctx.x0, "payload_kg": payload},
        events=[_cmd(0.2, "lift")],
        windows={"unseat": [0.2, dur]},
        info={"truth_mass_kg": ctx.m_star + payload},
    )


_LAND_S = {"drawer": 3.0, "door": 3.0, "wall": 5.0, "heavy": 8.0}  # land speeds 5/5/3/2 mm/s over ≈ 2.5 mm, core §C.4


def _b_land(ctx: _Ctx) -> SpecParts:
    dur = _LAND_S[ctx.cls]
    x = ctx.x0 + 0.5 * ctx.travel if ctx.cls == "drawer" else ctx.x0
    return _spec(ctx, dur, initial=_levitated(ctx, x), events=[_cmd(0.5, "land")], windows={"land": [0.5, dur]})


def _b_payload_step(ctx: _Ctx) -> SpecParts:
    delta = float(ctx.opt("delta_kg", 0.1 * ctx.m_star))  # +10 % of m*: DOOR +98.1 N (LEV-004)
    dur = 5.0
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, _park_x(ctx)),
        events=[{"kind": "payload", "t_s": 1.0, "delta_kg": delta}],
        windows={"settle": [3.0, dur]},
        info={"delta_kg": delta, "delta_n": delta * ground.G0_MPS2},
    )


def _hand_rng(ctx: _Ctx) -> np.random.Generator | None:
    return ctx.rng if bool(ctx.opt("jitter", False)) else None


#: ``push_open`` hand styles: push-and-release (push-and-go) or a walk-along hold.
_PUSH_OPEN_HANDS = ("push_release", "hold")


def _b_push_open(ctx: _Ctx) -> SpecParts:
    # Wave 2 (W1.5 P2): an intent-triggered automatic open follows the gesture
    # (push-and-go: the supervisor starts it once the hand flag clears), so the hand must
    # not stay anchored to the carriage. "push_release" is a push-only contact that ends
    # after the confirmation window; "hold" is a walk-along grip that guides the carriage
    # for 1.5 s and lets go. Neither can drag the carriage back.
    t0 = 0.5
    style = str(ctx.opt("hand", "push_release"))
    if style not in _PUSH_OPEN_HANDS:
        raise ValueError(f"options.hand: must be one of {', '.join(_PUSH_OPEN_HANDS)} (got {style!r})")
    events: list[dict[str, Any]] = []
    confirm = ctx.f("control.intent.confirm_max_s")
    if ctx.cls == "drawer":
        if style != "push_release":
            raise ValueError("options.hand: a seated DRAWER opens by push-to-open only (hand = 'push_release')")
        # Push-to-open from the seat: press inward (−x) against the seat's x compliance.
        hand = human.hand_profile("push_release", -2.0 * ctx.f_intent, t0, 0.08, _hand_rng(ctx), hold_s=0.1)
        initial: dict[str, Any] = {"lev": "seated", "x_m": ctx.x0}
        dur = 4.0
    else:
        if style == "push_release":
            # A push on the leaf (+x) held through the class confirmation window (F12), released.
            hand = human.hand_profile("push_release", 2.0 * ctx.f_intent, t0, 0.1, _hand_rng(ctx), hold_s=max(0.1, confirm))
        else:
            hand = human.hand_profile("hold", 2.0 * ctx.f_intent, t0, 0.1, _hand_rng(ctx), hold_s=1.5)
        initial = _levitated(ctx, ctx.x0)
        # DOOR: the whole automatic open onto its terminus; WALL/HEAVY end mid-move.
        t_rel = human.release_s(hand)
        dur_door = _ceil_half(t_rel + _move_s(ctx.travel, ctx.v_open, ctx.a_max) + ctx.t_terminus + 1.5)
        dur = {"door": dur_door, "wall": 10.0, "heavy": 12.0}[ctx.cls]
        events += _hold_to_run(ctx, 0.4)
    events.append(hand)
    t_pts = cast("list[float]", hand["t_pts_s"])
    return _spec(
        ctx,
        dur,
        initial=initial,
        events=events,
        windows={"push": [t0 + t_pts[1], t0 + t_pts[-1]], "motion": [t0, dur]},
        info={"hand": style, "t_release_s": human.release_s(hand), "f_th_n": ctx.f_intent, "confirm_max_s": confirm},
    )


_THROW_DIST_M = {"drawer": 0.24, "door": 0.5, "wall": 0.3}


def _b_throw_soft_close(ctx: _Ctx) -> SpecParts:
    # The throw is a walk-along grip (human.WALK_ALONG_KINDS): it pushes the carriage
    # closed at 3·F_th for rise + hold and lets go at speed; an intent-triggered close that
    # starts after the release finds no hand attached (W1.5 P2).
    dist = min(_THROW_DIST_M[ctx.cls], ctx.travel)
    t0, rise, hold = 1.0, 0.05, 0.25
    hand = human.hand_profile("throw", -3.0 * ctx.f_intent, t0, rise, _hand_rng(ctx), hold_s=hold)
    t_rel = human.release_s(hand)
    dur = _ceil_half(t_rel + dist / ctx.v_close + 3.0 * ctx.t_terminus + 2.0)
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0 + dist),
        events=[hand],
        windows={"close": [t_rel, dur]},
        info={"t_release_s": t_rel, "f_throw_n": float(cast("list[float]", hand["f_pts_n"])[-1])},
    )


#: ``detent_pass`` hand styles: a position-led slow drag (default) or a walk-along hold.
_DETENT_HANDS = ("guide", "hold")

#: Fraction of the intent displacement rate dx_th / confirm_max_s at which the guided drag
#: moves (design choice: the drag covers < 0.4·dx_th inside one confirmation window, so a
#: slow detent drag is not an opening gesture; PHYSICS §17).
_DETENT_V_FRAC = 0.4

#: Guided-drag approach and run-out beyond the detent, m (design choice).
_DETENT_APPROACH_M = 0.010


def _b_detent_pass(ctx: _Ctx) -> SpecParts:
    # Wave 2 (W1.5 P2): the Wave-1 1.5·F_th anchored pull confirmed intent, the automatic
    # move then dragged the gripping arm and raised Obstruction. A detent is a
    # position-dependent force, so the default drag is position-led (human.guide): a palm
    # whose reference moves at v_h below the intent displacement rate; the arm lag grows
    # by F_detent/k over the bump, then snaps over. A push-only palm cannot drag the
    # carriage if an automatic move starts anyway. DOOR/WALL are pushed open (+x) from
    # before the detent; the DRAWER is pushed closed (−x) from beyond it (a palm cannot
    # pull a drawer open). "hold" is a walk-along grip at 1.5·F_th for 2 s (fast crossing).
    xd = ctx.f("control.haptic.detent_x_m.0")
    width = ctx.f("control.haptic.detent_width_m")
    f_det = ctx.f("control.haptic.detent_f_n")
    style = str(ctx.opt("hand", "guide"))
    if style not in _DETENT_HANDS:
        raise ValueError(f"options.hand: must be one of {', '.join(_DETENT_HANDS)} (got {style!r})")
    t0 = 1.0
    info: dict[str, Any] = {"detent_x_m": xd, "detent_f_n": f_det, "detent_width_m": width, "hand": style}
    if style == "guide":
        sign = -1.0 if ctx.cls == "drawer" else 1.0
        v_h = _DETENT_V_FRAC * ctx.f("control.intent.dx_th_m") / ctx.f("control.intent.confirm_max_s")
        k_arm = human.NOMINAL_IMPEDANCE["k_n_per_m"]
        lag = f_det / k_arm
        dist = _DETENT_APPROACH_M + lag + 2.0 * width + _DETENT_APPROACH_M
        x_start = xd - sign * (_DETENT_APPROACH_M + width)
        hand = human.guide(t0, sign * dist, v_h, _hand_rng(ctx), rise_s=0.2, dwell_s=0.3)
        info.update({"v_hand_mps": v_h, "arm_lag_at_detent_m": lag, "dist_m": sign * dist})
    else:
        x_start = max(ctx.x0, xd - 0.05)
        hand = human.hand_profile("hold", 1.5 * ctx.f_intent, t0, 0.1, _hand_rng(ctx), hold_s=2.0)
    t_rel = human.release_s(hand)
    info["t_release_s"] = t_rel
    dur = _ceil_half(t_rel + 1.5)
    return _spec(ctx, dur, initial=_levitated(ctx, x_start), events=[hand], windows={"detent": [t0, t_rel]}, info=info)


_OBSTRUCT_START_M = {"drawer": 0.2, "door": 0.6, "wall": 0.4, "heavy": 0.3}


def _obstruction(ctx: _Ctx, body: str, x_obs_offset_m: float | None) -> SpecParts:
    dist = min(_OBSTRUCT_START_M[ctx.cls], ctx.travel)
    x_start = ctx.x0 + dist
    x_obs = ctx.x0 + (0.5 * dist if x_obs_offset_m is None else x_obs_offset_m)
    config = str(ctx.opt("config", "clamped"))
    events = [human.obstacle(0.0, body, x_obs, config=config), *_hold_to_run(ctx, 0.4), _cmd(0.5, "close")]
    dur = _ceil_half(0.5 + (x_start - x_obs) / ctx.v_close + ctx.t_terminus + 2.0)
    return _spec(
        ctx, dur, initial=_levitated(ctx, x_start), events=events, windows={"contact": [0.5, dur]}, info={"x_obstacle_m": x_obs}
    )


def _b_obstruction_chest(ctx: _Ctx) -> SpecParts:
    return _obstruction(ctx, "chest", None)


def _b_obstruction_hand_edge(ctx: _Ctx) -> SpecParts:
    return _obstruction(ctx, "hand", 0.02)  # fingers in the last 20 mm of the closing edge


def _b_lean(ctx: _Ctx) -> SpecParts:
    # HAP-006: 15 N rising over ≥ 1 s, held 5 s. DRAWER leans inward (the push-to-open
    # direction, the harder false-intent case); the others in the opening direction.
    f = float(ctx.opt("f_n", 15.0)) * (-1.0 if ctx.cls == "drawer" else 1.0)
    ev = human.lean(1.0, f, dur_s=7.0, rise_s=1.0)
    initial: dict[str, Any] = {"lev": "seated", "x_m": ctx.x0} if ctx.cls == "drawer" else _levitated(ctx, ctx.x0)
    return _spec(ctx, 10.0, initial=initial, events=[ev], windows={"lean": [1.0, 8.0]})


def _b_piston_pulse(ctx: _Ctx) -> SpecParts:
    # HAP-006 / ENV-003: a neighbour's closure pushes this closed drawer outward with
    # 5.6 N for 50 ms. The DRAWER preset has no smoke leaf area (design.air.leaf_area_m2
    # = 0), so the pressure pulse is applied as an external front force (the `lean`
    # event kind with a 10 ms ramp), not as a human lean.
    f = float(ctx.opt("f_n", 5.6))
    ev = {"kind": "lean", "t_s": 1.0, "f_n": f, "dur_s": 0.05, "rise_s": 0.01}
    return _spec(ctx, 3.0, initial={"lev": "seated", "x_m": ctx.x0}, events=[ev], windows={"pulse": [1.0, 2.0]})


def _b_smoke_dp(ctx: _Ctx) -> SpecParts:
    dp = float(ctx.opt("dp_pa", 25.0))  # 12.5–25 Pa smoke-control ΔP (physics perspective §1.9): 50 N on a 2 m² DOOR leaf
    ev = human.smoke_dp(1.0, dp, ramp_s=2.0, dur_s=8.0)
    return _spec(ctx, 12.0, initial=_levitated(ctx, ctx.x0), events=[ev], windows={"smoke": [1.0, 11.0]})


_POWER_S = {"drawer": 4.0, "door": 4.0, "wall": 6.0, "heavy": 8.0}


def _b_power_loss(ctx: _Ctx) -> SpecParts:
    dur = _POWER_S[ctx.cls]
    events: list[dict[str, Any]] = []
    t_loss = 1.0
    if bool(ctx.opt("moving", False)):
        events += [*_hold_to_run(ctx, 0.4), _open_to(ctx, 0.5, min(ctx.travel, 0.5))]
        t_loss = 1.5
        dur += 0.5
    x = ctx.x0 + 0.5 * ctx.travel if ctx.cls == "drawer" else ctx.x0
    events.append({"kind": "power_loss", "t_s": t_loss})
    return _spec(ctx, dur, initial=_levitated(ctx, x), events=events, windows={"landing": [t_loss, dur]})


def _control_loss(ctx: _Ctx, direction: str) -> SpecParts:
    x = ctx.x0 + 0.5 * ctx.travel if ctx.cls == "drawer" else ctx.x0
    ev = {"kind": "control_loss", "t_s": 0.5, "dir": direction}
    return _spec(ctx, 2.0, initial=_levitated(ctx, x), events=[ev], windows={"fail": [0.5, 2.0]})


def _b_control_loss_up(ctx: _Ctx) -> SpecParts:
    return _control_loss(ctx, "up")


def _b_control_loss_down(ctx: _Ctx) -> SpecParts:
    return _control_loss(ctx, "down")


_RAIL_MARGIN_M = 2.0  # profile beyond both travel ends: covers every magnet station offset


def _b_rail_traverse(ctx: _Ctx) -> SpecParts:
    z, y = rail.irregularity_profile(
        ctx.rng,
        ctx.travel + 2.0 * _RAIL_MARGIN_M,
        ctx.f("design.rail.span_m"),
        deflection_ratio=ctx.f("design.rail.deflection_ratio"),
        joint_step_m=ctx.f("design.rail.joint_step_m"),
        roughness_rms_m=ctx.f("design.rail.roughness_rms_m"),
        dx_m=1e-3,
        joint_spacing_m=ctx.f("design.rail.joint_spacing_m"),
        lateral_step_m=ctx.f("design.rail.lateral_step_m"),
    )
    series = {
        "rail": ({"axis": "space", "dx_m": 1e-3, "x0_m": ctx.x0 - _RAIL_MARGIN_M, "channels": ["z", "y"]}, np.vstack([z, y]))
    }
    dur = _ceil_half(0.5 + _move_s(ctx.travel, ctx.v_open, ctx.a_max) + ctx.t_terminus + 2.0)
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0),
        events=[*_hold_to_run(ctx, 0.4), _cmd(0.5, "open")],
        series=series,
        plant={"rail_profile": "series:rail"},
        windows={"traverse": [0.5, dur]},
    )


def _b_lateral_push(ctx: _Ctx) -> SpecParts:
    # LEV-011: a 20 N lateral push on a top-hung leaf. fabrail.scenario/1 has no y-force
    # event, so the push is emulated by a floor a_y step: the body feels the pseudo-force
    # −m·a_y at its CoM (not at handle height), m = m* here.
    f = float(ctx.opt("f_n", 20.0))
    fs = 200.0
    dist = min(ctx.travel, 0.3 if ctx.cls == "wall" else 1.0)
    dur = _ceil_half(2.0 + _move_s(dist, ctx.v_open, ctx.a_max) + ctx.t_terminus + 2.0)
    t = np.arange(int(round(dur * fs)) + 1) / fs
    ramp = 0.5 * (1.0 - np.cos(np.pi * np.clip((t - 1.0) / 0.2, 0.0, 1.0)))
    ay = -(f / ctx.m_star) * ramp
    series = {"floor": _time_series(ay, fs, ["ay"])}
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0),
        events=[_open_to(ctx, 2.0, dist)],
        series=series,
        windows={"push": [1.5, 2.0], "drag": [2.5, dur - 1.0]},
        info={"f_lateral_n": f, "a_y_floor_mps2": -f / ctx.m_star},
    )


def _b_thermal_ramp(ctx: _Ctx) -> SpecParts:
    dt_k = float(ctx.opt("dt_k", 40.0))
    ramp_s = float(ctx.opt("ramp_s", 20.0))  # compressed quasi-static ramp (plan §2b item 13)
    dur = 1.0 + ramp_s + 5.0
    ev = {"kind": "temp_ramp", "t_s": 1.0, "dt_k": dt_k, "dur_s": ramp_s}
    return _spec(ctx, dur, initial=_levitated(ctx, _park_x(ctx)), events=[ev], windows={"hot": [2.0 + ramp_s, dur]})


_MOVE_DIST_M = {"drawer": 0.3, "door": 1.0, "wall": 1.0, "heavy": 0.5}


#: The clean SAF-001 input: a core fault that halves the m̂ the core uses (lead decision
#: W2, W1.5 P2). It needs ``core_fault.what = "m_hat_half"`` in the shipped schema and the
#: Rust loader (fabrail-sim ``CoreFaultKind``, fabrail-rt ``CoreFault``).
_M_HAT_HALF = "m_hat_half"

#: Fallback truth-magnet sabotage (m̂ ≈ m/br² in the zero-power band): 1.1 gives m̂ ≈ 0.83·m
#: without coil saturation in every class (W2 probe: √2 saturated the DOOR coils,
#: SaturationPersist at 0.22 s, and raised Obstruction on DRAWER).
_BR_SABOTAGE = 1.1

_SABOTAGE_KINDS = ("auto", _M_HAT_HALF, "br_scale")


def _core_fault_kinds() -> tuple[str, ...]:
    """``core_fault.what`` values the shipped scenario schema accepts."""
    root = _load_schema(str(SCHEMA_PATH))
    what = root.get("$defs", {}).get("ev_core_fault", {}).get("properties", {}).get("what", {})
    return tuple(str(x) for x in what.get("enum", ()))


def _b_governor_sabotage(ctx: _Ctx) -> SpecParts:
    # SAF-001: m_gov = max(m_rated, 1.05·m̂), so sabotaging m̂ to 0.5·m must leave the
    # governed speed at or below the rated-mass cap. The clean input is the core fault
    # "m_hat_half" at t = 0.2 s (before the move); until the schema and the Rust loader
    # accept it, "auto" falls back to truth magnets stronger than the nominal model
    # (plant.br_scale, m̂ ≈ m/br²; the achieved m̂ is in the m_hat_kg channel), which
    # reaches only m̂ ≈ 0.83·m at the default 1.1 (stated in info).
    sabotage = str(ctx.opt("sabotage", "auto"))
    if sabotage not in _SABOTAGE_KINDS:
        raise ValueError(f"options.sabotage: must be one of {', '.join(_SABOTAGE_KINDS)} (got {sabotage!r})")
    supported = _M_HAT_HALF in _core_fault_kinds()
    if sabotage == _M_HAT_HALF and not supported:
        raise ValueError(
            'options.sabotage: core_fault what "m_hat_half" is not in the shipped scenario schema yet '
            "(needs the schema enum plus fabrail-sim CoreFaultKind / fabrail-rt CoreFault); use 'br_scale'"
        )
    if "br_scale" in ctx.options and sabotage == _M_HAT_HALF:
        raise ValueError("options.br_scale: only with sabotage 'br_scale' (or 'auto' without m_hat_half)")
    use_fault = sabotage == _M_HAT_HALF or (sabotage == "auto" and supported and "br_scale" not in ctx.options)
    dist = min(_MOVE_DIST_M[ctx.cls], ctx.travel)
    dur = _ceil_half(0.5 + _move_s(dist, ctx.v_open, ctx.a_max) + ctx.t_terminus + 2.0)
    events: list[dict[str, Any]] = [*_hold_to_run(ctx, 0.4), _open_to(ctx, 0.5, dist)]
    plant: dict[str, Any] = {}
    info: dict[str, Any] = {"m_rated_kg": ctx.m_rated, "v_open_cap_mps": ctx.v_open, "m_star_kg": ctx.m_star}
    if use_fault:
        events.append({"kind": "core_fault", "t_s": 0.2, "what": _M_HAT_HALF})
        info.update({"sabotage": _M_HAT_HALF, "m_hat_expected_ratio": 0.5})
    else:
        br = float(ctx.opt("br_scale", _BR_SABOTAGE))
        plant["br_scale"] = br
        info.update(
            {
                "sabotage": "br_scale",
                "br_scale": br,
                "m_hat_expected_ratio": 1.0 / (br * br),
                "note": "fallback: core_fault m_hat_half is not in the shipped schema; m-hat reads low through truth magnets",
            }
        )
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0),
        events=events,
        plant=plant,
        windows={"move": [0.5, dur]},
        info=info,
    )


def _b_core_fault_iq(ctx: _Ctx) -> SpecParts:
    dist = min(_MOVE_DIST_M[ctx.cls], ctx.travel)
    dur = {"drawer": 3.0, "door": 3.0, "wall": 5.0, "heavy": 6.0}[ctx.cls]
    events = [*_hold_to_run(ctx, 0.4), _open_to(ctx, 0.5, dist), {"kind": "core_fault", "t_s": 1.0, "what": "iq_gain_x3"}]
    return _spec(ctx, dur, initial=_levitated(ctx, ctx.x0), events=events, windows={"fault": [1.0, dur]})


def _b_payload_mismatch(ctx: _Ctx) -> SpecParts:
    # SAF-018: m̂ > 1.05·m_rated raises PayloadMismatch (Degrade: hold-to-run only). The
    # later automatic open (no hold-to-run) must not move the carriage.
    delta = float(ctx.opt("delta_kg", 1.10 * ctx.m_rated - ctx.m_star))
    x = ctx.x0 + 0.05 if ctx.cls == "drawer" else _park_x(ctx)
    dist = min(_MOVE_DIST_M[ctx.cls], ctx.x0 + ctx.travel - x)
    events = [{"kind": "payload", "t_s": 1.0, "delta_kg": delta}, _cmd(3.0, "go_to", x + 0.5 * dist)]
    return _spec(
        ctx,
        6.0,
        initial=_levitated(ctx, x),
        events=events,
        windows={"mismatch": [1.0, 6.0]},
        info={"truth_mass_kg": ctx.m_star + delta},
    )


# --- seismic -----------------------------------------------------------------------

_T_P_S = 35.0  # P arrival in the record: the 30 s STA/LTA warm-up plus 5 s (pwave.boore_ps default)
_PS_FS_HZ = 500.0  # the ADXL355 output data rate (design.sensors.accel_odr_hz)


def _ps_duration_s(
    magnitude: float, r_epi_km: float, depth_km: float, t_p_s: float, stress_drop_bar: float, tail_s: float
) -> float:
    """Record length covering P, the S window (2·T_d, Boore 2003) and ``tail_s``."""
    r = pwave.hypocentral_km(r_epi_km, depth_km)
    t_s = t_p_s + r / pwave.V_S_KMPS - r / pwave.V_P_KMPS
    t_d = 1.0 / pwave.brune_corner_hz(magnitude, stress_drop_bar=stress_drop_bar) + 0.05 * r
    return _ceil_half(t_s + 2.0 * t_d + tail_s)


def _ps(
    ctx: _Ctx, magnitude: float, r_epi_km: float, depth_km: float, tail_s: float = 5.0
) -> tuple[pwave.PSEvent, dict[str, Any]]:
    """Synthesise the P+S floor record (the device floor = ground × floor_amplification(z/h))."""
    m = float(ctx.opt("magnitude", magnitude))
    r_epi = float(ctx.opt("r_epi_km", r_epi_km))
    depth = float(ctx.opt("depth_km", depth_km))
    t_p = float(ctx.opt("t_p_s", _T_P_S))
    dsig = float(ctx.opt("stress_drop_bar", 30.0))
    dur = _ps_duration_s(m, r_epi, depth, t_p, dsig, tail_s)
    ev = pwave.boore_ps(
        ctx.rng,
        m,
        r_epi,
        depth,
        fs_hz=_PS_FS_HZ,
        duration_s=dur,
        t_p_s=t_p,
        stress_drop_bar=dsig,
        p_corner=str(ctx.opt("p_corner", "wu_kanamori")),
    )
    z_over_h = float(ctx.opt("z_over_h", 0.0))
    amp = ground.floor_amplification(z_over_h)
    info: dict[str, Any] = {
        "magnitude": m,
        "r_epi_km": r_epi,
        "depth_km": depth,
        "r_hyp_km": ev.meta["r_hyp_km"],
        "t_p_s": ev.t_p_s,
        "t_s_s": ev.t_s_s,
        "origin_t_s": ev.meta["origin_t_s"],
        "pd_target_m": ev.meta["pd_target_m"],
        "tau_c_s": ev.meta["tau_c_s"],
        "pga_h_mps2": cast(float, ev.meta["pga_h_mps2"]) * amp,
        "z_over_h": z_over_h,
        "floor_amplification": amp,
        "ground_motion": "synthetic (Boore 2003 stochastic; P scaled to Wu-Zhao at hypocentral R)",
    }
    return ev, info


def _ps_series(ctx: _Ctx, ev: pwave.PSEvent, amp: float) -> dict[str, tuple[dict[str, Any], np.ndarray]]:
    series = {"floor": _time_series(ev.a_xyz_mps2 * amp, ev.fs_hz, ["ax", "ay", "az"])}
    if bool(ctx.opt("site_noise", False)):
        dur = ev.a_xyz_mps2.shape[1] / ev.fs_hz
        series["site"] = _time_series(pwave.site_noise(ctx.rng, dur, ev.fs_hz, mems=False), ev.fs_hz, ["ax", "ay", "az"])
    return series


_PS_CASES = {"pwave_m5_r10": (5.0, 10.0, 10.0), "pwave_m7_r20": (7.0, 20.0, 10.0), "pwave_m78_r100": (7.8, 100.0, 10.0)}


def _b_pwave(ctx: _Ctx) -> SpecParts:
    ev, info = _ps(ctx, *_PS_CASES[ctx.sdef.name])
    dur = ev.a_xyz_mps2.shape[1] / ev.fs_hz
    t_p, t_s = ev.t_p_s, ev.t_s_s
    # Moving at the P arrival (so FREEZE is observable, SEI-009): open 0.5 s before t_P.
    events = [*_hold_to_run(ctx, t_p - 0.6), _open_to(ctx, t_p - 0.5, min(ctx.travel, _MOVE_DIST_M[ctx.cls]))]
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0),
        events=events,
        series=_ps_series(ctx, ev, float(info["floor_amplification"])),
        windows={"p": [t_p, min(t_p + 3.0, dur)], "shaking": [t_s, dur]},
        info=info,
    )


def _alert(info: Mapping[str, Any], latency_s: float, alert_id: int = 1) -> dict[str, Any]:
    """A synthetic/injected network alert ``latency_s`` after the origin (not a live EEW feed)."""
    origin = float(info["origin_t_s"])
    return {
        "kind": "network_alert",
        "t_s": max(0.0, origin + latency_s),
        "magnitude": float(info["magnitude"]),
        "r_epi_km": float(info["r_epi_km"]),
        "depth_km": float(info["depth_km"]),
        "origin_t_s": origin,
        "id": alert_id,
    }


def _b_network_alert(ctx: _Ctx) -> SpecParts:
    ev, info = _ps(ctx, 7.0, 60.0, 10.0, tail_s=10.0)
    alert = _alert(info, float(ctx.opt("latency_s", 5.0)))
    t_a, dur = float(alert["t_s"]), ev.a_xyz_mps2.shape[1] / ev.fs_hz
    events = [*_hold_to_run(ctx, t_a - 1.1), _open_to(ctx, t_a - 1.0, min(ctx.travel, _MOVE_DIST_M[ctx.cls])), alert]
    info = {**info, "t_alert_s": t_a, "alert": "synthetic/injected, not a live EEW feed"}
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0),
        events=events,
        series=_ps_series(ctx, ev, float(info["floor_amplification"])),
        windows={"lead": [t_a, ev.t_p_s], "shaking": [ev.t_s_s, dur]},
        info=info,
    )


def _b_alert_cancel(ctx: _Ctx) -> SpecParts:
    alert = {
        "kind": "network_alert",
        "t_s": 2.0,
        "magnitude": 6.5,
        "r_epi_km": 40.0,
        "depth_km": 10.0,
        "origin_t_s": -1.0,
        "id": 1,
    }
    cancel = {"kind": "alert_cancel", "t_s": 5.0, "id": 1}
    return _spec(
        ctx,
        15.0,
        initial=_levitated(ctx, _park_x(ctx)),
        events=[alert, cancel],
        windows={"alert": [2.0, 5.0], "after": [5.0, 15.0]},
        info={"alert": "synthetic/injected, not a live EEW feed", "false_alert": True},
    )


def _b_elcentro_free(ctx: _Ctx) -> SpecParts:
    rec = ctx.opt("record", None)
    if rec is None:
        rec = records.elcentro_like(ctx.rng)
    if not isinstance(rec, records.Record):
        raise ValueError("options.record: expected a fabrail.records.Record")
    a = np.asarray(rec.a_mps2, dtype=np.float64)
    ax = a if a.ndim == 1 else a[0]
    t0 = 1.0
    dur = _ceil_half(t0 + ax.shape[0] / rec.fs_hz + 10.0)
    v = ground.trapezoid_integral(ax, rec.fs_hz)
    pgd = float(np.max(np.abs(ground.trapezoid_integral(v, rec.fs_hz))))
    # A free (levitated, unpropelled) mass: `manual` releases the propulsion (SEI-013).
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0 + 0.5 * ctx.travel),
        events=[_cmd(0.5, "manual")],
        series={"floor": _time_series(ax, rec.fs_hz, ["ax"], t0_s=t0)},
        windows={"shaking": [t0, t0 + ax.shape[0] / rec.fs_hz], "after": [t0, dur]},
        info={"record": rec.name, "source": rec.source, "pga_mps2": float(np.max(np.abs(ax))), "pgd_m": pgd},
    )


def _b_isolation_kt(ctx: _Ctx) -> SpecParts:
    pga = float(ctx.opt("pga_g", 0.5))
    soil = str(ctx.opt("soil", "medium"))
    model = str(ctx.opt("model", "cp"))
    fs, rec_s, t0 = 200.0, 40.0, 1.0
    a = ground.synthesize(ctx.rng, rec_s, fs, soil=soil, pga_g=pga, envelope=None, model=model)
    pgd = float(np.max(np.abs(ground.trapezoid_integral(ground.trapezoid_integral(a, fs), fs))))
    dur = t0 + rec_s + 5.0
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0 + 0.5 * ctx.travel),
        series={"floor": _time_series(a, fs, ["ax"], t0_s=t0)},
        windows={"shaking": [t0, t0 + rec_s]},
        info={"pga_g": pga, "soil": soil, "model": model, "pgd_m": pgd},
    )


def _b_vertical_pulse(ctx: _Ctx) -> SpecParts:
    # SEI-017: a downward 0.8 g half-sine floor pulse (0.2 s default) at t = 1 s.
    a_g = float(ctx.opt("a_g", 0.8))
    pulse_s = float(ctx.opt("pulse_s", 0.2))
    fs, t_on = 1000.0, 1.0
    t = np.arange(int(round(3.0 * fs)) + 1) / fs
    az = np.where((t >= t_on) & (t <= t_on + pulse_s), -a_g * ground.G0_MPS2 * np.sin(np.pi * (t - t_on) / pulse_s), 0.0)
    return _spec(
        ctx,
        5.0,
        initial=_levitated(ctx, _park_x(ctx)),
        series={"floor": _time_series(az, fs, ["az"])},
        windows={"pulse": [t_on, 5.0]},
        info={"a_peak_mps2": -a_g * ground.G0_MPS2, "pulse_s": pulse_s},
    )


def _b_power_loss_at_s(ctx: _Ctx) -> SpecParts:
    ev, info = _ps(ctx, 7.0, 20.0, 10.0)
    dur = _ceil_half(max(ev.a_xyz_mps2.shape[1] / ev.fs_hz, ev.t_s_s + 20.0))
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, _park_x(ctx)),
        events=[{"kind": "power_loss", "t_s": ev.t_s_s}],
        series=_ps_series(ctx, ev, float(info["floor_amplification"])),
        windows={"shaking": [ev.t_s_s, dur]},
        info=info,
    )


def _b_egress_eew(ctx: _Ctx) -> SpecParts:
    # SEI-011: the egress door never locks; a fire-rated door (--set design.egress=fire_rated)
    # never opens; a non-rated door may open only if stopped by t_S − 1 s.
    ev, info = _ps(ctx, 7.0, 60.0, 10.0, tail_s=10.0)
    alert = _alert(info, float(ctx.opt("latency_s", 5.0)))
    dur = _ceil_half(max(ev.a_xyz_mps2.shape[1] / ev.fs_hz, ev.t_s_s + 20.0))
    t_a = float(alert["t_s"])
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0),
        events=[alert],
        series=_ps_series(ctx, ev, float(info["floor_amplification"])),
        windows={"lead": [t_a, ev.t_s_s], "after": [ev.t_s_s, dur]},
        info={**info, "t_alert_s": t_a, "egress": ctx.s("design.egress"), "alert": "synthetic/injected, not a live EEW feed"},
    )


#: SEI-021 host building: one storey of 1e7 kg at T1 = 1.13 s, ζ 2 % (Kyobashi-like
#: host, physics perspective §2.13; a 5 t panel is μ = 0.05 %).
_HOST_MASS_KG = 1.0e7
_HOST_T1_S = 1.13
_HOST_ZETA = 0.02


def _b_harmonic_base(ctx: _Ctx) -> SpecParts:
    plant: dict[str, Any] = {}
    if ctx.tier == "structure":
        k = _HOST_MASS_KG * (2.0 * math.pi / _HOST_T1_S) ** 2
        plant["structure"] = {
            "storey_mass_kg": [_HOST_MASS_KG],
            "storey_k_n_per_m": [k],
            "rayleigh_zeta": [_HOST_ZETA, _HOST_ZETA],
            "device_storey": 1,
        }
        freq = float(ctx.opt("freq_hz", 1.0 / _HOST_T1_S))
    else:
        f_iso = 1.0 / ctx.f("control.seismic.isolation.t_iso_s")
        freq = float(ctx.opt("freq_hz", float(ctx.opt("r", 3.0)) * f_iso))  # SEI-014: T(3, 0.2)
    amp = float(ctx.opt("amp_mps2", 0.2))
    cycles = float(ctx.opt("cycles", 30.0))
    fs, t0 = 200.0, 1.0
    rec_s = cycles / freq
    t = np.arange(int(round(rec_s * fs)) + 1) / fs
    ramp = 0.5 * (1.0 - np.cos(np.pi * np.clip(t * freq / 2.0, 0.0, 1.0)))  # 2-cycle cosine ramp-in
    ax = amp * ramp * np.sin(2.0 * np.pi * freq * t)
    dur = _ceil_half(t0 + rec_s + 2.0)
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0 + 0.5 * ctx.travel),
        series={"floor": _time_series(ax, fs, ["ax"], t0_s=t0)},
        plant=plant,
        windows={"steady": [t0 + 0.5 * rec_s, t0 + rec_s]},
        info={"freq_hz": freq, "amp_mps2": amp, "cycles": cycles},
    )


def _b_noise_1h(ctx: _Ctx) -> SpecParts:
    fs, dur = 250.0, 3600.0
    noise = pwave.site_noise(ctx.rng, dur, fs, mems=False)  # the simulator adds the ADXL355 noise itself
    initial: dict[str, Any] = {"lev": "seated", "x_m": ctx.x0} if ctx.cls == "drawer" else _levitated(ctx, ctx.x0)
    return _spec(
        ctx,
        dur,
        initial=initial,
        series={"site": _time_series(noise, fs, ["ax", "ay", "az"])},
        windows={"quiet": [0.0, dur]},
        rate_cap_hz=10.0,
        channels="kpis_only",
        info={"site": "footsteps, slams, HVAC, self-actuation (pwave.site_noise, no MEMS term)"},
    )


# --- plan §4b additions ------------------------------------------------------------

_TERMINUS_DIST_M = {"drawer": 0.3, "door": 0.5, "wall": 0.5, "heavy": 0.3}


def _b_terminus_soft_close(ctx: _Ctx) -> SpecParts:
    dist = min(_TERMINUS_DIST_M[ctx.cls], ctx.travel)
    seat = ctx.f("control.levitation.seat_time_s") if ctx.cls == "drawer" else 0.0
    dur = _ceil_half(0.5 + _move_s(dist, ctx.v_close, ctx.a_max) + ctx.t_terminus + seat + 1.5)
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0 + dist),
        events=[*_hold_to_run(ctx, 0.4), _cmd(0.5, "close")],
        windows={"terminus": [0.5, dur]},
        info={"t_arrive_closed_form_s": ctx.t_terminus, "d0_m": ctx.derived["terminus_d0_m"]},
    )


def _b_soft_open(ctx: _Ctx) -> SpecParts:
    dist = min(_TERMINUS_DIST_M[ctx.cls] if ctx.cls in ("wall", "heavy") else ctx.travel, ctx.travel)
    initial: dict[str, Any] = {"lev": "seated", "x_m": ctx.x0} if ctx.cls == "drawer" else _levitated(ctx, ctx.x0)
    unseat = 0.3 if ctx.cls == "drawer" else 0.0
    dur = _ceil_half(0.5 + unseat + _move_s(dist, ctx.v_open, ctx.a_max) + ctx.t_terminus + 1.5)
    return _spec(
        ctx,
        dur,
        initial=initial,
        events=[*_hold_to_run(ctx, 0.4), _open_to(ctx, 0.5, dist)],
        windows={"terminus": [0.5, dur]},
        info={"t_arrive_closed_form_s": ctx.t_terminus, "target_x_m": ctx.x0 + dist},
    )


#: Hand-detection allowance per newton of commanded thrust: the controller flags a hand
#: when |F̂| > F_th + 0.25·|F_cmd| (fabrail-core fast.rs ``HAND_TH_THRUST_FRAC``, W1.5; a
#: core design constant, not a parameter, restated here only to place the scenario hand).
_HAND_TH_THRUST_FRAC = 0.25

#: Scenario design choices for ``terminus_hand_replan``: the hold-back force is
#: ``_REPLAN_TH0_FRAC``·th0 (a hand that holds a capture back harder than the obstruction
#: residual th0 latches an obstruction on release, plan §4b "obstruction always
#: overrides"; F̂ overshot the truth hand by up to ≈ 30 % in the W2 probes, hence 0.7),
#: placed where it exceeds the estimated hand threshold by
#: ``_REPLAN_TH_MARGIN``; the capture time is estimated at ``_APPROACH_V_FRAC`` of the
#: class close cap (slower than any planned speed seen in W1.5/W2, so the error puts the
#: hand later in the exponential capture, where the threshold is lower).
_REPLAN_TH0_FRAC = 0.7
_REPLAN_TH_MARGIN = 1.15
_APPROACH_V_FRAC = 0.85
_REPLAN_DIST_M = {"drawer": 0.3, "door": 0.5, "wall": 0.25}
#: Hold of the hand pulse at its peak, s: a brief grip (50 ms rise, hold, 50 ms ramp down) long
#: enough for the hand onset (≤ 10 / 20 ms) without pulling the admittance-rendered carriage far back.
_REPLAN_HOLD_S = 0.05


def _approach_s(dist_m: float, v_mps: float, a_mps2: float, j_mps3: float) -> float:
    """Time for a jerk-limited launch from rest to cover ``dist_m`` at cruise ``v_mps``, s (approximate)."""
    t_acc = v_mps / a_mps2 + a_mps2 / j_mps3
    d_acc = 0.5 * v_mps * t_acc
    if dist_m >= d_acc:
        return t_acc + (dist_m - d_acc) / v_mps
    return 2.0 * math.sqrt(max(dist_m, 0.0) / a_mps2)


def _b_terminus_hand_replan(ctx: _Ctx) -> SpecParts:
    # HAP-016: a hand during capture switches the carriage to class admittance; its
    # release re-plans the terminus from the measured state. Wave 2 (W1.5 P2): the Wave-1
    # hand (0.5·F_th) stayed below the controller's hand threshold F_th + 0.25·|F_cmd|, so
    # no hand was ever detected. The hand is now a brief walk-along grip (rise 50 ms, hold
    # 50 ms, release) that holds the close back (+x) with F = 0.7·th0 (below the
    # obstruction residual, so its release re-plans instead of latching an obstruction; a
    # longer constant pull would drag the light admittance-rendered carriage far open),
    # placed at the capture time Δ after the blend
    # where 0.75·F ≥ 1.15·(F_th + 0.25·m*·a_ref(Δ)), a_ref(Δ) = (d_b/τ²)·e^(−Δ/τ) (the
    # controller counters the hand, so |F_cmd| ≈ m*·a_ref + F). Options t_hand_s, f_hand_n
    # (signed, +x holds a close back) and hold_s override; info reports the estimates.
    nat = _native()
    dist = min(_REPLAN_DIST_M[ctx.cls], ctx.travel)
    tau = ctx.f("control.motion.terminus.tau_s")
    j_term = ctx.f("control.motion.terminus.j_mps3")
    th0 = ctx.f("control.haptic.obstruction.residual_n")
    v_app = _APPROACH_V_FRAC * ctx.v_close
    blend = dict(nat.terminus_blend(v_app, j_term, tau))
    t_b, d_b, d0 = float(blend["t_b_s"]), float(blend["d_b_m"]), float(blend["d_0_m"])
    t_capture = 0.5 + _approach_s(dist - d0, v_app, ctx.a_max, ctx.f("control.motion.j_max_mps3"))
    f_hand = float(ctx.opt("f_hand_n", _REPLAN_TH0_FRAC * th0))
    a_peak = d_b / (tau * tau)
    a_ok = (0.75 * abs(f_hand) / _REPLAN_TH_MARGIN - ctx.f_intent) / (_HAND_TH_THRUST_FRAC * ctx.m_star)
    window_ok = a_ok > 0.0
    delta = tau * math.log(a_peak / a_ok) if window_ok and a_ok < a_peak else 0.0
    t_hand = float(ctx.opt("t_hand_s", t_capture + t_b + delta))
    hold = float(ctx.opt("hold_s", _REPLAN_HOLD_S))
    hand = human.hand_profile("hold", f_hand, t_hand, 0.05, _hand_rng(ctx), hold_s=hold)
    t_rel = human.release_s(hand)
    a_at = a_peak * math.exp(-max(0.0, t_hand - t_capture - t_b) / tau)
    th_est = (ctx.f_intent + _HAND_TH_THRUST_FRAC * ctx.m_star * a_at) / (1.0 - _HAND_TH_THRUST_FRAC)
    seat = ctx.f("control.levitation.seat_time_s") if ctx.cls == "drawer" else 0.0
    dur = _ceil_half(t_rel + 2.0 * ctx.t_terminus + seat + 1.5)
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0 + dist),
        events=[_cmd(0.5, "close"), hand],
        windows={"capture": [t_capture, t_hand], "hand": [t_hand, t_rel], "replan": [t_rel, dur]},
        info={
            "t_capture_est_s": t_capture,
            "t_hand_s": t_hand,
            "t_release_s": t_rel,
            "f_hand_n": f_hand,
            "hand_threshold_est_n": th_est,
            "obstruction_th0_n": th0,
            "hand_window_ok": bool(
                window_ok and abs(f_hand) >= _REPLAN_TH_MARGIN * th_est * (1.0 - 1e-9) and abs(f_hand) < th0
            ),
            "v_approach_est_mps": v_app,
            "d_at_hand_est_m": d_b * math.exp(-max(0.0, t_hand - t_capture - t_b) / tau),
        },
    )


#: The villa's warned event (plan §4b): M7.2 at R_epi 60 km, 12 km deep (t_P 10.2 s,
#: t_S 17.5 s after the origin), fused alert ≈ 7.5 s after the origin.
_VILLA_EVENT = (7.2, 60.0, 12.0)
_VILLA_ALERT_S = 7.5


def _b_choreographed_close(ctx: _Ctx) -> SpecParts:
    ev, info = _ps(ctx, *_VILLA_EVENT, tail_s=10.0)
    alert = _alert(info, float(ctx.opt("latency_s", _VILLA_ALERT_S)))
    dur = _ceil_half(max(ev.a_xyz_mps2.shape[1] / ev.fs_hz, ev.t_s_s + 30.0))
    t_a = float(alert["t_s"])
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0 + ctx.travel),
        events=[alert],
        series=_ps_series(ctx, ev, float(info["floor_amplification"])),
        windows={"lead": [t_a, ev.t_s_s], "shaking": [ev.t_s_s, dur]},
        info={
            **info,
            "t_alert_s": t_a,
            "alert": "synthetic/injected, not a live EEW feed",
            "policy": "core CorePolicy under run(); the house choreographer's close needs its ActionPolicy (Session.step_to_with_policy)",
        },
    )


def _b_inertial_smooth_fixedbase(ctx: _Ctx) -> SpecParts:
    ev, info = _ps(ctx, *_VILLA_EVENT, tail_s=10.0)
    t1 = float(ctx.opt("building_t1_s", 0.25))  # a low-rise fixed-base building, 3–5 Hz response (plan §4b)
    zh = float(ctx.opt("building_z_over_h", 0.5))
    horiz = ground.floor_response(ev.a_xyz_mps2[:2], ev.fs_hz, t1, zeta=0.05, z_over_h=zh)
    floor = np.vstack([horiz, ev.a_xyz_mps2[2:3]])
    alert = _alert(info, float(ctx.opt("latency_s", _VILLA_ALERT_S)))
    dur = _ceil_half(max(ev.a_xyz_mps2.shape[1] / ev.fs_hz, ev.t_s_s + 30.0))
    x_open = float(ctx.opt("x_open_m", 0.2))
    t_a = float(alert["t_s"])
    return _spec(
        ctx,
        dur,
        initial=_levitated(ctx, ctx.x0 + min(x_open, ctx.travel)),
        events=[alert],
        series={"floor": _time_series(floor, ev.fs_hz, ["ax", "ay", "az"])},
        windows={"smoothing": [ev.t_s_s, min(ev.t_s_s + 20.0, dur)]},
        info={
            **info,
            "t_alert_s": t_a,
            "building": {"config": "fixed_base", "t1_s": t1, "zeta": 0.05, "z_over_h": zh},
            "pga_floor_h_mps2": float(np.max(np.hypot(horiz[0], horiz[1]))),
            "alert": "synthetic/injected, not a live EEW feed",
        },
    )


def _windowed_sine_floor(pga_mps2: float, freq_hz: float, cycles: float, fs_hz: float) -> tuple[np.ndarray, float]:
    """Floor acceleration of the displacement d(t) = D·sin²(πt/T)·sin(ωt), T = cycles/f, scaled to peak ``pga_mps2``.

    d, ḋ are zero at both ends, so a free mass returns to its start relative to the rail
    unless a stop intervenes; the first half-cycle moves the floor +x, i.e. a free carriage
    −x relative to the rail (into the closed end). Returns (a, D).
    """
    t_end = cycles / freq_hz
    t = np.arange(int(round(t_end * fs_hz)) + 1) / fs_hz
    w_, big = 2.0 * math.pi * freq_hz, 2.0 * math.pi / t_end
    win, dwin, ddwin = 0.5 - 0.5 * np.cos(big * t), 0.5 * big * np.sin(big * t), 0.5 * big * big * np.cos(big * t)
    unit = ddwin * np.sin(w_ * t) + 2.0 * dwin * w_ * np.cos(w_ * t) - win * w_ * w_ * np.sin(w_ * t)
    amp = pga_mps2 / float(np.max(np.abs(unit)))
    return amp * unit, amp


def _b_closed_end_stop_under_shaking(ctx: _Ctx) -> SpecParts:
    # Wave 2 decision D3: physical closed-end (jamb / DRAWER carcass push-stop / seat) and
    # open-end stops in the plant; floor-driven displacement with the LSM in STO inside
    # them is reported, never a new monitor trip. A carriage at its closed stop (DRAWER
    # seated, the others levitated; option lev = "rest" | "levitated" — a DRAWER levitated
    # inside its slot saturates its coils, SaturationPersist in the W2 probe) is released
    # to a free mass by `manual` (LSM STO; option free = False keeps the drive on) and the
    # floor is shaken along x by a windowed sine whose first half-cycle drives the
    # carriage into the closed end (default 0.35 g at 1.5 Hz, 6 cycles: floor stroke
    # ≈ ±39 mm, relative impact speed ≈ 0.36 m/s). Synthetic, deterministic input.
    pga_g = float(ctx.opt("pga_g", 0.35))
    freq = float(ctx.opt("freq_hz", 1.5))
    cycles = float(ctx.opt("cycles", 6.0))
    lev = str(ctx.opt("lev", "rest" if ctx.cls == "drawer" else "levitated"))
    free = bool(ctx.opt("free", True))
    if not (pga_g > 0.0 and freq > 0.0 and cycles >= 1.0):
        raise ValueError(f"options: need pga_g > 0, freq_hz > 0 and cycles >= 1 (got {pga_g}, {freq}, {cycles})")
    if lev not in ("levitated", "rest"):
        raise ValueError(f"options.lev: must be 'levitated' or 'rest' (got {lev!r})")
    fs, t0 = 1000.0, 1.0
    ax, amp = _windowed_sine_floor(pga_g * ground.G0_MPS2, freq, cycles, fs)
    t_end = t0 + cycles / freq
    dur = _ceil_half(t_end + 3.0)
    initial: dict[str, Any] = _levitated(ctx, ctx.x0) if lev == "levitated" else {"lev": None, "x_m": ctx.x0}
    events = [_cmd(0.5, "manual")] if free else []
    return _spec(
        ctx,
        dur,
        initial=initial,
        events=events,
        series={"floor": _time_series(ax, fs, ["ax"], t0_s=t0)},
        windows={"shaking": [t0, t_end], "after": [t_end, dur]},
        info={
            "pga_mps2": pga_g * ground.G0_MPS2,
            "freq_hz": freq,
            "cycles": cycles,
            "floor_stroke_m": amp,
            "v_rel_peak_est_mps": amp * 2.0 * math.pi * freq,
            "free": free,
            "lev": lev,
            "x_closed_m": ctx.x0,
            "x_min_slp_m": ctx.f("safety.x_min_m"),
            "ground_motion": "synthetic windowed sine (deterministic), not a recorded or site-specific motion",
        },
    )


def _b_footsteps(ctx: _Ctx) -> SpecParts:
    fs, dur = 250.0, 30.0
    rate = float(ctx.opt("episode_rate_per_s", 0.2))
    floor = pwave.site_noise(
        ctx.rng, dur, fs, footsteps=True, slams=False, hvac=False, self_actuation=False, mems=False, footstep_rate_per_s=rate
    )
    initial: dict[str, Any] = {"lev": "seated", "x_m": ctx.x0} if ctx.cls == "drawer" else _levitated(ctx, ctx.x0)
    return _spec(
        ctx, dur, initial=initial, series={"floor": _time_series(floor, fs, ["ax", "ay", "az"])}, windows={"walk": [0.0, dur]}
    )


# ---------------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------------

_ALL = ("drawer", "door", "wall", "heavy")
_SEISMIC_TIERS = ("travel1d", "carriage6", "heave1")
_PS_OPTIONS = ("magnitude", "r_epi_km", "depth_km", "t_p_s", "stress_drop_bar", "p_corner", "z_over_h", "site_noise")


def _make_build(sdef_ref: list[ScenarioDef], fn: Callable[[_Ctx], SpecParts]) -> Callable[..., SpecParts]:
    def build(
        params: Params,
        cfg: RunConfig,
        rng: np.random.Generator,
        *,
        noise_seed: int | None = None,
        fidelity: str | None = None,
        **options: Any,
    ) -> SpecParts:
        sdef = sdef_ref[0]
        cls = _class_of(params)
        if cls not in sdef.classes:
            raise ValueError(f"scenario {sdef.name!r} does not apply to class {cls!r} (classes: {', '.join(sdef.classes)})")
        unknown = sorted(set(options) - set(sdef.options))
        if unknown:
            accepted = ", ".join(sdef.options) or "none"
            raise ValueError(f"scenario {sdef.name!r}: unknown option(s) {', '.join(unknown)} (accepted: {accepted})")
        seed = split_seed(cfg.seed)[1] if noise_seed is None else int(noise_seed)
        ctx = _Ctx(sdef, params, cfg, rng, seed, _resolve_tier(sdef, cfg, fidelity), dict(options), cls)
        return fn(ctx)

    return build


def _register(
    name: str,
    classes: tuple[str, ...],
    doc: str,
    fn: Callable[[_Ctx], SpecParts],
    tiers: tuple[str, ...] = ("carriage6",),
    options: tuple[str, ...] = (),
    slow_classes: tuple[str, ...] = (),
    requires: Mapping[str, Any] | None = None,
) -> None:
    ref: list[ScenarioDef] = []
    sdef = ScenarioDef(name, classes, doc, _make_build(ref, fn), tiers, options, slow_classes, dict(requires or {}))
    ref.append(sdef)
    CATALOG[name] = sdef


_register(
    "hover",
    _ALL,
    "levitated at rest for 10 s: gap, force noise, hover power (PWR-002, LEV-003)",
    _b_hover,
    ("carriage6", "heave1"),
)
_register(
    "lift",
    ("door", "wall", "heavy"),
    "lift from the rollers: LEV-012 latency, no upper-stop contact",
    _b_lift,
    ("carriage6", "heave1"),
)
_register(
    "unseat",
    ("drawer",),
    "DRAWER unseat from the 3.5 mm captive seat (LEV-017); option payload_kg or mass_kg",
    _b_unseat,
    options=("payload_kg", "mass_kg"),
)
_register(
    "land", _ALL, "commanded landing from levitation: touchdown speed and time", _b_land, ("carriage6", "heave1", "travel1d")
)
_register(
    "payload_step",
    _ALL,
    "+10 % m* payload step at 1 s: zero-power settling, i_ss (LEV-004)",
    _b_payload_step,
    ("carriage6", "heave1"),
    ("delta_kg",),
)
_register(
    "push_open",
    _ALL,
    "push-and-release opens the carriage (DRAWER: push-to-open from the seat), then the intent-triggered "
    "open runs: apparent mass, intent latency (HAP-001/005/012); option hand = hold (walk-along grip)",
    _b_push_open,
    ("carriage6", "travel1d"),
    ("jitter", "hand"),
)
_register(
    "throw_soft_close",
    ("drawer", "door", "wall"),
    "walk-along hand throws the carriage closed and lets go at speed: soft-close capture (HAP-007)",
    _b_throw_soft_close,
    ("carriage6", "travel1d"),
    ("jitter",),
)
_register(
    "detent_pass",
    ("drawer", "door", "wall"),
    "a palm drags the carriage slowly across its first detent (position-led, below the intent rate) (HAP-008)",
    _b_detent_pass,
    ("carriage6", "travel1d"),
    ("jitter", "hand"),
)
_register(
    "obstruction_chest",
    ("door", "wall", "heavy"),
    "closing onto a clamped ISO/TS 15066 chest: stop and reverse/release (SAF-005/006)",
    _b_obstruction_chest,
    ("carriage6", "travel1d"),
    ("config",),
)
_register(
    "obstruction_hand_edge",
    _ALL,
    "closing onto a hand clamped 20 mm from the closed stop (SAF-005/006)",
    _b_obstruction_hand_edge,
    ("carriage6", "travel1d"),
    ("config",),
)
_register(
    "lean", _ALL, "15 N lean rising over 1 s, held 5 s: no false intent (HAP-006)", _b_lean, ("carriage6", "travel1d"), ("f_n",)
)
_register(
    "piston_pulse",
    ("drawer",),
    "5.6 N outward cabinet-pressure pulse for 50 ms on a seated drawer: no false intent (HAP-006, ENV-003)",
    _b_piston_pulse,
    ("carriage6", "travel1d"),
    ("f_n",),
)
_register(
    "smoke_dp",
    ("door", "wall"),
    "25 Pa smoke-control ΔP ramped over 2 s (50 N on the DOOR leaf): no false intent (HAP-006)",
    _b_smoke_dp,
    ("carriage6", "travel1d"),
    ("dp_pa",),
)
_register(
    "power_loss",
    _ALL,
    "mains loss while levitated: SS1 Safe Landing on the supercap (SAF-009/010)",
    _b_power_loss,
    ("carriage6", "heave1", "travel1d"),
    ("moving",),
)
_register(
    "control_loss_up",
    _ALL,
    "coil drivers stick at full attraction: fail-direction impact (LEV-019)",
    _b_control_loss_up,
    ("carriage6", "heave1"),
)
_register(
    "control_loss_down",
    _ALL,
    "coil drivers stick at full release: fail-direction impact (LEV-019)",
    _b_control_loss_down,
    ("carriage6", "heave1"),
)
_register(
    "rail_traverse",
    _ALL,
    "full-travel open at v_max over an L/1150 rail with joints and 20 µm roughness (LEV-010)",
    _b_rail_traverse,
    slow_classes=("wall", "heavy"),
)
_register(
    "lateral_push",
    ("door", "wall"),
    "20 N lateral push on a top-hung leaf (as a floor a_y step), then travel: guide-fin drag (LEV-011)",
    _b_lateral_push,
    options=("f_n",),
)
_register(
    "thermal_ramp",
    _ALL,
    "+40 K compressed quasi-static ambient ramp: gap shift, |i| (LEV-014)",
    _b_thermal_ramp,
    ("heave1", "carriage6"),
    ("dt_k", "ramp_s"),
)
_register(
    "governor_sabotage",
    _ALL,
    "move with m̂ sabotaged (core fault m_hat_half when the schema has it, else truth B_r × 1.1): "
    "governed speed ≤ the rated-mass cap (SAF-001)",
    _b_governor_sabotage,
    ("carriage6",),
    ("sabotage", "br_scale"),
)
_register(
    "core_fault_iq",
    _ALL,
    "core commands 3× thrust mid-move: the integer monitor trips STO/ASC (SAF-012)",
    _b_core_fault_iq,
    ("carriage6", "travel1d"),
)
_register(
    "payload_mismatch",
    _ALL,
    "payload to 1.10·m_rated: PayloadMismatch, hold-to-run only (SAF-018)",
    _b_payload_mismatch,
    ("carriage6", "heave1"),
    ("delta_kg",),
)
_register(
    "pwave_m5_r10",
    _ALL,
    "M5 at R_epi 10 km, 10 km deep (synthetic P+S): level ≤ 1, freeze only (SEI-010)",
    _b_pwave,
    _SEISMIC_TIERS,
    _PS_OPTIONS,
)
_register(
    "pwave_m7_r20",
    _ALL,
    "M7 at R_epi 20 km, 10 km deep (synthetic P+S): L3, PROTECT at t_P + 1.2 s (SEI-010)",
    _b_pwave,
    _SEISMIC_TIERS,
    _PS_OPTIONS,
)
_register(
    "pwave_m78_r100",
    _ALL,
    "M7.8 at R_epi 100 km, 10 km deep (synthetic P+S): ≥ L2, land ≥ 8 s before t_S (SEI-010, slow)",
    _b_pwave,
    _SEISMIC_TIERS,
    _PS_OPTIONS,
    slow_classes=_ALL,
)
_register(
    "network_alert",
    _ALL,
    "synthetic network alert (M7, R_epi 60 km) 5 s after origin, before local P: S-ETA, actions before t_P (SEI-007)",
    _b_network_alert,
    _SEISMIC_TIERS[:2],
    (*_PS_OPTIONS, "latency_s"),
)
_register(
    "alert_cancel",
    _ALL,
    "synthetic network alert cancelled after 3 s, no shaking: back to NORMAL (SEI-007/008)",
    _b_alert_cancel,
    _SEISMIC_TIERS[:2],
)
_register(
    "elcentro_free",
    _ALL,
    "free (manual, unpropelled) mass under the El Centro-like surrogate: drift ≈ PGD (SEI-013); DRAWER/DOOR hit their stops",
    _b_elcentro_free,
    _SEISMIC_TIERS[:2],
    ("record",),
)
_register(
    "isolation_kt",
    ("wall", "heavy"),
    "KT/CP ground motion at 0.5 g, 40 s: isolation stroke and bumper energy (SEI-015; research tier)",
    _b_isolation_kt,
    _SEISMIC_TIERS[:2],
    ("pga_g", "soil", "model"),
    requires={"control.seismic.isolation.enable": True},
)
_register(
    "vertical_pulse",
    _ALL,
    "downward 0.8 g, 0.2 s half-sine floor pulse: VerticalSaturation and landing (SEI-017)",
    _b_vertical_pulse,
    ("heave1", "carriage6"),
    ("a_g", "pulse_s"),
)
_register(
    "power_loss_at_s",
    _ALL,
    "mains loss at the S arrival of synthetic M7 R_epi 20 km: land, captive, manual force (SEI-019)",
    _b_power_loss_at_s,
    _SEISMIC_TIERS[:2],
    _PS_OPTIONS,
)
_register(
    "egress_eew",
    ("door",),
    "egress DOOR under a synthetic network alert (M7, R_epi 60 km): never locks; stop by t_S − 1 s (SEI-011)",
    _b_egress_eew,
    _SEISMIC_TIERS[:2],
    (*_PS_OPTIONS, "latency_s"),
)
_register(
    "harmonic_base",
    ("wall", "heavy"),
    "harmonic floor motion (3·f_iso; on Structure the 1e7 kg host at T1 1.13 s): transmissibility (SEI-014), AMD (SEI-021)",
    _b_harmonic_base,
    ("travel1d", "carriage6", "structure"),
    ("freq_hz", "r", "amp_mps2", "cycles"),
    requires={"control.seismic.isolation.enable": True},
)
_register(
    "noise_1h",
    _ALL,
    "1 h of site noise on the structure accelerometer: 0 PROTECT, raw triggers counted (SEI-006)",
    _b_noise_1h,
    ("travel1d", "carriage6"),
)
_register(
    "terminus_soft_close",
    _ALL,
    "close onto the log-damped terminus: semilog fit, arrival, overshoot (HAP-013…016)",
    _b_terminus_soft_close,
    ("carriage6", "travel1d"),
)
_register(
    "soft_open",
    _ALL,
    "open onto the log-damped open terminus (DRAWER from the seat; WALL/HEAVY go_to)",
    _b_soft_open,
    ("carriage6", "travel1d"),
)
_register(
    "terminus_hand_replan",
    ("drawer", "door", "wall"),
    "a brief walk-along grip holds the terminus capture back (0.7·th0, above the hand threshold) and lets go: "
    "admittance, re-plan (HAP-016)",
    _b_terminus_hand_replan,
    ("carriage6", "travel1d"),
    ("jitter", "t_hand_s", "f_hand_n", "hold_s"),
)
_register(
    "choreographed_close",
    ("drawer",),
    "open DRAWER under the villa's synthetic warned event (M7.2, R_epi 60 km, alert at +7.5 s): "
    "the choreographer's close inputs (plan §4b)",
    _b_choreographed_close,
    ("carriage6", "travel1d"),
    (*_PS_OPTIONS, "latency_s"),
)
_register(
    "inertial_smooth_fixedbase",
    ("drawer",),
    "open DRAWER in a fixed-base building (T1 0.25 s) under the villa event: SmoothRideThrough (HAP-017…019; research tier)",
    _b_inertial_smooth_fixedbase,
    ("carriage6",),
    (*_PS_OPTIONS, "latency_s", "building_t1_s", "building_z_over_h", "x_open_m"),
    requires={"control.motion.inertial.enable": True},
)
_register(
    "closed_end_stop_under_shaking",
    _ALL,
    "free carriage at its closed stop (DRAWER seated) under 0.35 g x shaking (synthetic windowed sine): physical stops, "
    "floor-driven motion reported, no new monitor trip (Wave 2 D3; SAF-012/014)",
    _b_closed_end_stop_under_shaking,
    ("travel1d", "carriage6"),
    ("pga_g", "freq_hz", "cycles", "lev", "free"),
)
_register(
    "footsteps",
    _ALL,
    "30 s of footsteps shaking the floor: no false intent (HAP-006)",
    _b_footsteps,
    ("carriage6", "travel1d"),
    ("episode_rate_per_s",),
)

if set(CATALOG) != set(NAMES) | set(PLAN_NAMES):  # pragma: no cover - import-time guard
    raise RuntimeError("scenarios: CATALOG drifted from NAMES + PLAN_NAMES")


# ---------------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------------


def _class_of(params: Any) -> str:
    rc = getattr(params, "rail_class", None)
    key = getattr(rc, "key", None)
    if not isinstance(key, str):
        raise ValueError("params: expected a fabrail._native.Params (rail_class.key)")
    return key


def _get(name: str) -> ScenarioDef:
    try:
        return CATALOG[name]
    except KeyError:
        raise ValueError(f"unknown scenario {name!r} (see `python -m fabrail scenarios`)") from None


def _resolve_tier(sdef: ScenarioDef, cfg: RunConfig, fidelity: str | None) -> str:
    if fidelity is not None:
        if fidelity not in sdef.tiers:
            raise ValueError(f"fidelity: scenario {sdef.name!r} supports {', '.join(sdef.tiers)} (got {fidelity!r})")
        return fidelity
    if cfg.fidelity != _DEFAULT_FIDELITY:
        if cfg.fidelity not in sdef.tiers:
            raise ValueError(f"fidelity: scenario {sdef.name!r} supports {', '.join(sdef.tiers)} (got {cfg.fidelity!r})")
        return cfg.fidelity
    return sdef.tiers[0]


def resolve_fidelity(name: str, cfg: RunConfig, fidelity: str | None = None) -> str:
    """The plant tier scenario ``name`` runs on for ``cfg`` (see the module docstring)."""
    return _resolve_tier(_get(name), cfg, fidelity)


def _seeds(cfg: RunConfig, member: MemberSeed | None) -> tuple[np.random.Generator, int]:
    if member is not None:
        return member.rng(), int(member.noise_seed)
    synth, noise_seed = split_seed(cfg.seed)
    return np.random.default_rng(synth), noise_seed


def spec(
    name: str,
    params: Params,
    cfg: RunConfig,
    *,
    fidelity: str | None = None,
    member: MemberSeed | None = None,
    options: Mapping[str, Any] | None = None,
) -> SpecParts:
    """The spec dict and series bytes for scenario ``name`` (seeded from ``cfg.seed``).

    ``member`` (an ensemble :class:`fabrail.ensemble.MemberSeed`) replaces the seed
    split of ``cfg.seed``; ``fidelity`` forces a tier (:func:`resolve_fidelity`);
    ``options`` go to the builder (``CATALOG[name].options``). The result passes
    :func:`validate_spec` and :func:`cross_check` (``ValueError("path: reason")``).
    """
    sdef = _get(name)
    rng, noise_seed = _seeds(cfg, member)
    d, series = sdef.build(params, cfg, rng, noise_seed=noise_seed, fidelity=fidelity, **dict(options or {}))
    validate_spec(d)
    cross_check(d, series, params)
    return d, series


def build_spec(
    name: str,
    params: Params,
    cfg: RunConfig,
    *,
    fidelity: str | None = None,
    member: MemberSeed | None = None,
    options: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """The ``fabrail.scenario/1`` dict of ``name``, validated against ``schemas/scenario-1.schema.json``."""
    return spec(name, params, cfg, fidelity=fidelity, member=member, options=options)[0]


def build(
    name: str,
    params: Params,
    cfg: RunConfig,
    *,
    fidelity: str | None = None,
    member: MemberSeed | None = None,
    options: Mapping[str, Any] | None = None,
) -> Scenario:
    """The native ``Scenario`` for ``name`` (``Scenario.from_json(json, series)``)."""
    d, series = spec(name, params, cfg, fidelity=fidelity, member=member, options=options)
    return _native().Scenario.from_json(to_json(d), series)


def _native() -> Any:
    from . import require_native

    return require_native()


def catalog_errors(
    names: Sequence[str] | None = None,
    *,
    classes: Sequence[str] | None = None,
    tiers: str = "recommended",
    coils: Sequence[str] = ("voltage",),
    native: bool = True,
    seed: int = 42,
) -> list[str]:
    """Build CATALOG entries for every class (and tier, coil) they apply to; return every failure.

    For each (name, class, tier, coil): :func:`spec` must build (it enforces the schema
    and :func:`cross_check`); :func:`schema_errors` must be empty on the canonical JSON
    round-trip; the name, tier and coil must be the requested ones; no ``hand`` event may
    be an anchored grip (:func:`fabrail.human.anchored_grip`: it would drag a carriage that
    an intent-triggered move drives away, the W1.5 finding); and with ``native`` the Rust
    loader (``Scenario.from_json``) must accept the spec and return the same series
    names. ``tiers`` is ``"recommended"`` (each entry's first tier) or ``"all"``.
    Research-tier ``requires`` are not applied (the specs do not depend on them). Returns
    ``"name/class/tier/coil: reason"`` strings; ``[]`` means the catalog is clean
    (FAB-SW-015 check for every entry).
    """
    from .config import RunConfig

    if tiers not in ("recommended", "all"):
        raise ValueError(f"catalog_errors: tiers must be 'recommended' or 'all' (got {tiers!r})")
    nat = _native() if native else None
    out: list[str] = []
    for name in names if names is not None else sorted(CATALOG):
        sdef = _get(name)
        for cls in sdef.classes:
            if classes is not None and cls not in classes:
                continue
            for tier in sdef.tiers if tiers == "all" else sdef.tiers[:1]:
                for coil in coils:
                    tag = f"{name}/{cls}/{tier}/{coil}"
                    try:
                        cfg = RunConfig(rail_class=cls, coil=coil, seed=seed)
                        d_raw, series = spec(name, cfg.params(), cfg, fidelity=tier)
                        d = cast("dict[str, Any]", d_raw)
                        text = to_json(d)
                        errs = schema_errors(json.loads(text))
                        if errs:
                            out.append(f"{tag}: schema: {'; '.join(errs[:4])}")
                        if d["name"] != f"{cls}_{name}" or d["fidelity"]["model"] != tier or d["fidelity"]["coil"] != coil:
                            out.append(f"{tag}: name/tier/coil mismatch ({d['name']}, {d['fidelity']})")
                        for i, ev in enumerate(d["events"]):
                            if human.anchored_grip(ev):
                                out.append(f"{tag}: events[{i}]: anchored grip hand (drags an automatically moved carriage)")
                        if nat is not None:
                            scen = nat.Scenario.from_json(text, series)
                            if set(scen.series_bytes()) != set(series):
                                out.append(f"{tag}: native loader series {sorted(scen.series_bytes())} != {sorted(series)}")
                    except Exception as exc:  # noqa: BLE001 - every failure is reported, not raised
                        out.append(f"{tag}: {type(exc).__name__}: {exc}")
    return out


def to_json(spec_d: Mapping[str, Any]) -> str:
    """Canonical JSON (sorted keys, compact, no NaN) of a spec dict."""
    return json.dumps(spec_d, sort_keys=True, separators=(",", ":"), allow_nan=False)


def info(spec_d: Mapping[str, Any]) -> dict[str, Any]:
    """The synthesis summary a builder stored in ``spec["doc"]`` (t_P, t_S, R_hyp, Pd target, PGA, …); {} if none."""
    doc = str(spec_d.get("doc") or "")
    _, sep, tail = doc.partition(_INFO_MARK)
    if not sep:
        return {}
    try:
        out = json.loads(tail)
    except ValueError:
        return {}
    return out if isinstance(out, dict) else {}


# ---------------------------------------------------------------------------------
# Validation: JSON schema (draft 2020-12 subset) and the Rust cross-field rules
# ---------------------------------------------------------------------------------


@lru_cache(maxsize=4)
def _load_schema(path: str) -> dict[str, Any]:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _json_type(x: object) -> str:
    if x is None:
        return "null"
    if isinstance(x, bool):
        return "boolean"
    if isinstance(x, int):
        return "integer"
    if isinstance(x, float):
        return "number"
    if isinstance(x, str):
        return "string"
    if isinstance(x, (list, tuple)):
        return "array"
    if isinstance(x, dict):
        return "object"
    return type(x).__name__


def _type_ok(x: Any, t: str) -> bool:
    jt = _json_type(x)
    if t == "number":
        return jt in ("integer", "number")
    if t == "integer":
        return jt == "integer" or (jt == "number" and math.isfinite(x) and float(x).is_integer())
    return jt == t


def _json_eq(a: Any, b: Any) -> bool:
    ta, tb = _json_type(a), _json_type(b)
    if {ta, tb} <= {"integer", "number"}:
        return float(a) == float(b)
    if ta != tb:
        return False
    if ta == "array":
        return len(a) == len(b) and all(_json_eq(x, y) for x, y in zip(a, b, strict=True))
    if ta == "object":
        return a.keys() == b.keys() and all(_json_eq(a[k], b[k]) for k in a)
    return a == b


def _join(path: str, key: str | int) -> str:
    if isinstance(key, int):
        return f"{path}[{key}]"
    return f"{path}.{key}" if path else key


def _resolve(root: Mapping[str, Any], ref: str) -> Any:
    if not ref.startswith("#/"):
        raise ValueError(f"schema: only local $ref are supported (got {ref!r})")
    node: Any = root
    for part in ref[2:].split("/"):
        node = node[part.replace("~1", "/").replace("~0", "~")]
    return node


def _errors(x: Any, s: Any, root: Mapping[str, Any], path: str) -> list[str]:
    """Validation errors of ``x`` against schema node ``s`` as ``"path: reason"``."""
    if s is True:
        return []
    if s is False:
        return [f"{path or '(root)'}: not allowed"]
    p = path or "(root)"
    out: list[str] = []
    if "$ref" in s:
        out += _errors(x, _resolve(root, s["$ref"]), root, path)
    t = s.get("type")
    if t is not None:
        types = t if isinstance(t, list) else [t]
        if not any(_type_ok(x, tt) for tt in types):
            return [*out, f"{p}: expected {' or '.join(types)}, got {_json_type(x)}"]
    if "const" in s and not _json_eq(x, s["const"]):
        out.append(f"{p}: must be {json.dumps(s['const'])}")
    if "enum" in s and not any(_json_eq(x, e) for e in s["enum"]):
        out.append(f"{p}: must be one of {json.dumps(s['enum'])}")
    jt = _json_type(x)
    if jt in ("integer", "number"):
        v = float(x)
        if not math.isfinite(v):
            out.append(f"{p}: must be finite")
        if "minimum" in s and v < s["minimum"]:
            out.append(f"{p}: must be >= {s['minimum']}")
        if "maximum" in s and v > s["maximum"]:
            out.append(f"{p}: must be <= {s['maximum']}")
        if "exclusiveMinimum" in s and v <= s["exclusiveMinimum"]:
            out.append(f"{p}: must be > {s['exclusiveMinimum']}")
        if "exclusiveMaximum" in s and v >= s["exclusiveMaximum"]:
            out.append(f"{p}: must be < {s['exclusiveMaximum']}")
    elif jt == "string":
        if "pattern" in s and re.search(s["pattern"], x) is None:
            out.append(f"{p}: must match {s['pattern']}")
        if "minLength" in s and len(x) < s["minLength"]:
            out.append(f"{p}: shorter than {s['minLength']}")
        if "maxLength" in s and len(x) > s["maxLength"]:
            out.append(f"{p}: longer than {s['maxLength']}")
    elif jt == "array":
        arr = list(x)
        if "minItems" in s and len(arr) < s["minItems"]:
            out.append(f"{p}: needs at least {s['minItems']} items")
        if "maxItems" in s and len(arr) > s["maxItems"]:
            out.append(f"{p}: allows at most {s['maxItems']} items")
        if s.get("uniqueItems"):
            seen = [json.dumps(i, sort_keys=True) for i in arr]
            if len(set(seen)) != len(seen):
                out.append(f"{p}: items must be unique")
        if "items" in s:
            for i, item in enumerate(arr):
                out += _errors(item, s["items"], root, _join(path, i))
    elif jt == "object":
        obj: Mapping[str, Any] = x
        for k in s.get("required", ()):
            if k not in obj:
                out.append(f"{_join(path, k)}: required")
        props = s.get("properties", {})
        for k, v in obj.items():
            if "propertyNames" in s:
                out += [
                    e.replace(f"{_join(path, k)}:", f"{_join(path, k)} (name):")
                    for e in _errors(k, s["propertyNames"], root, _join(path, k))
                ]
            if k in props:
                out += _errors(v, props[k], root, _join(path, k))
            elif "additionalProperties" in s:
                ap = s["additionalProperties"]
                if ap is False:
                    out.append(f"{_join(path, k)}: unknown field")
                elif ap is not True:
                    out += _errors(v, ap, root, _join(path, k))
    for sub in s.get("allOf", ()):
        out += _errors(x, sub, root, path)
    if "anyOf" in s:
        branches = [_errors(x, sub, root, path) for sub in s["anyOf"]]
        if all(branches):
            out.append(f"{p}: matches none of the anyOf branches ({min(branches, key=len)[0]})")
    if "oneOf" in s:
        subs = s["oneOf"]
        branches = [_errors(x, sub, root, path) for sub in subs]
        n_ok = sum(1 for b in branches if not b)
        if n_ok > 1:
            out.append(f"{p}: matches more than one oneOf branch")
        elif n_ok == 0:
            kind = x.get("kind") if isinstance(x, dict) else None
            chosen: list[str] | None = None
            for sub, errs in zip(subs, branches, strict=True):
                target = _resolve(root, sub["$ref"]) if isinstance(sub, dict) and "$ref" in sub else sub
                if (
                    kind is not None
                    and isinstance(target, dict)
                    and target.get("properties", {}).get("kind", {}).get("const") == kind
                ):
                    chosen = errs
            if chosen is None and kind is not None:
                out.append(f"{_join(path, 'kind')}: unknown event kind {kind!r}")
            else:
                fewest: list[str] = min(branches, key=len)
                out += chosen if chosen is not None else fewest
    if "not" in s and not _errors(x, s["not"], root, path):
        out.append(f"{p}: must not match the 'not' schema")
    if "if" in s:
        if not _errors(x, s["if"], root, path):
            if "then" in s:
                out += _errors(x, s["then"], root, path)
        elif "else" in s:
            out += _errors(x, s["else"], root, path)
    return out


def schema_errors(instance: object, schema_path: str | Path = SCHEMA_PATH) -> list[str]:
    """Every ``"path: reason"`` error of ``instance`` against a shipped JSON schema (draft 2020-12 subset).

    Supports the keywords the fabrail schemas use: ``$ref`` (local), ``type``,
    ``const``, ``enum``, numeric and string bounds, ``pattern``, ``items``,
    ``min/maxItems``, ``uniqueItems``, ``required``, ``properties``,
    ``additionalProperties``, ``propertyNames``, ``allOf``/``anyOf``/``oneOf``,
    ``not`` and ``if``/``then``/``else``. Use ``SCENE_SCHEMA_PATH`` for scenes.
    """
    root = _load_schema(str(schema_path))
    return _errors(instance, root, root, "")


def validate_spec(spec_d: Mapping[str, Any]) -> None:
    """Raise ``ValueError("path: reason; …")`` unless ``spec_d`` is valid ``fabrail.scenario/1`` JSON."""
    errs = schema_errors(spec_d, SCHEMA_PATH)
    if errs:
        raise ValueError("; ".join(errs[:8]) + (f"; … ({len(errs)} errors)" if len(errs) > 8 else ""))


def cross_check(spec_d: Mapping[str, Any], series: Mapping[str, bytes] | None = None, params: Any = None) -> None:
    """The cross-field rules the schema cannot express (mirrors ``ScenarioSpec::validate`` and ``check_compat``).

    Raises ``ValueError("path: reason")`` on the first violation: event times within the
    run; hand ``t_pts_s`` strictly increasing and as long as ``f_pts_n``; lean
    2·rise ≤ dur; smoke 2·ramp ≤ dur; alert origin ≤ receipt; metrics windows within the
    run; ``record.rate_hz`` a whole number of plant steps; ``plant.structure`` iff the
    Structure tier; ``rail_profile`` iff ``series.rail``; series bytes present, finite and
    a whole number of channel-major f64 rows. With ``params``: the coil mode equals
    ``control.rates.coil_mode_voltage``, a seated start needs a captive seat, a locked
    start a parking lock, the start within the travel, and the truth mass
    (mass_scale·m* + payload) > 0 through every payload event.
    """
    dur = float(spec_d["duration_s"])
    tier = str(spec_d["fidelity"]["model"])
    rate = float(spec_d.get("record", {}).get("rate_hz", 1000.0))
    d = BASE_TICK_HZ / rate
    step = _TIER_STEP_TICKS[tier]
    if not (d >= 1.0 and abs(d - round(d)) <= 1e-9 * d and int(round(d)) % step == 0):
        raise ValueError(
            f"record.rate_hz: {BASE_TICK_HZ} / rate_hz must be a whole multiple of the {tier} plant step ({step} base ticks)"
        )
    events = list(spec_d.get("events", []))
    for i, e in enumerate(events):
        t = float(e["t_s"])
        if not (0.0 <= t <= dur):
            raise ValueError(f"events[{i}].t_s: must be in [0, duration_s = {dur}] (got {t})")
        kind = e["kind"]
        if kind == "hand":
            tp, fp = e["t_pts_s"], e["f_pts_n"]
            if not tp or len(tp) != len(fp):
                raise ValueError(f"events[{i}].t_pts_s: t_pts_s and f_pts_n need equal, non-zero lengths")
            if any(b <= a for a, b in zip(tp, tp[1:], strict=False)) or tp[0] < 0.0:
                raise ValueError(f"events[{i}].t_pts_s: must be finite, >= 0 and strictly increasing")
        elif kind == "lean" and float(e.get("rise_s", 1.0)) > 0.5 * float(e["dur_s"]):
            raise ValueError(f"events[{i}].rise_s: must be at most dur_s / 2")
        elif kind == "smoke_dp" and e.get("dur_s") is not None and 2.0 * float(e.get("ramp_s", 2.0)) > float(e["dur_s"]):
            raise ValueError(f"events[{i}].ramp_s: must be at most dur_s / 2")
        elif kind == "network_alert" and float(e["origin_t_s"]) > t:
            raise ValueError(f"events[{i}].origin_t_s: the origin must not be after the alert's receipt (t_s)")
    for k, (t0, t1) in spec_d.get("metrics", {}).get("windows", {}).items():
        if not (0.0 <= t0 < t1 <= dur):
            raise ValueError(f"metrics.windows.{k}: must be [t0_s, t1_s] with 0 <= t0 < t1 <= duration_s")
    plant = spec_d.get("plant", {})
    if (plant.get("structure") is not None) != (tier == "structure"):
        raise ValueError('plant.structure: required with fidelity.model "structure" and only valid there')
    decl = spec_d.get("series", {})
    if (plant.get("rail_profile") is not None) != ("rail" in decl):
        raise ValueError('plant.rail_profile: must be "series:rail" exactly when series.rail is declared')
    blobs = dict(series or {})
    for name, meta in decl.items():
        if name not in blobs:
            raise ValueError(f"series.{name}: declared but no bytes were given")
        n_ch = len(meta["channels"])
        raw = blobs[name]
        if len(raw) == 0 or len(raw) % (8 * n_ch):
            raise ValueError(f"series.{name}: {len(raw)} bytes is not a non-empty multiple of 8 x {n_ch} channels")
        if not np.all(np.isfinite(np.frombuffer(raw, dtype="<f8"))):
            raise ValueError(f"series.{name}: values must be finite")
    for name in blobs:
        if name not in decl:
            raise ValueError(f"series.{name}: bytes were given but the scenario does not declare this series")
    if params is None:
        return
    coil_v = str(spec_d["fidelity"]["coil"]) == "voltage"
    if coil_v != bool(params.get("control.rates.coil_mode_voltage")):
        raise ValueError(
            "fidelity.coil: the scenario coil mode must equal control.rates.coil_mode_voltage "
            "(plan §2b item 16; use RunConfig.params())"
        )
    init = spec_d.get("initial", {})
    rest = str(params.get("design.rest_policy"))
    lev = init.get("lev") or {"seat": "seated", "hover": "levitated"}.get(rest, "landed")
    if lev == "seated" and str(params.get("design.landing.kind")) != "seat":
        raise ValueError('initial.lev: "seated" needs a captive seat (design.landing.kind = "seat")')
    locked = init.get("locked")
    if locked is None:
        locked = rest == "land_lock" and lev == "landed"
    if locked and not bool(params.get("design.landing.parking_lock")):
        raise ValueError("initial.locked: needs a parking lock (design.landing.parking_lock)")
    x0 = float(params.get("design.rail.x_closed_m"))
    x1 = x0 + float(params.get("design.rail.travel_m"))
    x = init.get("x_m")
    x = x0 if x is None else float(x)
    if not (x0 - 1e-9 <= x <= x1 + 1e-9):
        raise ValueError(f"initial.x_m: must lie within the travel [{x0}, {x1}] m (got {x})")
    m = float(plant.get("mass_scale", 1.0)) * float(params.get("design.carriage.m_star_kg")) + float(init.get("payload_kg", 0.0))
    if not m > 0.0:
        raise ValueError(f"initial.payload_kg: the truth mass plant.mass_scale * m_star + payload_kg must be > 0 (got {m} kg)")
    for i, e in sorted(enumerate(events), key=lambda ie: float(ie[1]["t_s"])):
        if e["kind"] == "payload":
            m += float(e["delta_kg"])
            if not m > 0.0:
                raise ValueError(f"events[{i}].delta_kg: the truth mass would drop to {m} kg")
