"""Human and disturbance inputs as scenario events (core §B.8; PHYSICS §11, §22).

Each function returns one ``fabrail.scenario/1`` event dict (see
``schemas/scenario-1.schema.json``): ``hand`` (piecewise-linear driving force through
an arm impedance), ``lean``, ``obstacle`` and ``smoke_dp``. Disturbance profiles
respect plan §2b item 11: a lean rises over at least 1 s, a smoke pressure ramp over
at least 2 s (both rejected otherwise). Forces follow the rail frame: +x opens.

Hand profile kinds (physics perspective §1.11): a force ramp 0 → F_pk over 50–150 ms
through the arm impedance (m 1–2 kg, b 5–20 N·s/m, k 200–1000 N/m):

- ``push``: push-only contact, ramp up, hold, ramp down;
- ``push_release``: push-and-release (push-to-open, push-and-go), push-only contact,
  ramp up, short hold, release;
- ``pull``: grip contact (can pull) with an **anchored** arm (a person standing still),
  ramp up, hold, ramp down;
- ``hold``: grip contact with a **walk-along** arm (the person moves with the carriage),
  ramp up, hold, ramp down, release;
- ``throw``: walk-along grip, ramp up, hold, then let go at speed (the hand releases
  after the last point with the force still applied);
- ``lean``: a ``lean`` event (no motion intent, rise ≥ 1 s);
- ``slam``: a short push-only impulse (triangular, rise = ``rise_s``);
- ``pinch``: a finger clamped at the closing edge — an ``obstacle`` event with the
  ISO/TS 15066 hand region (the force emerges from the edge closing on it).

:func:`guide` adds a position-led hand: its reference moves a set distance at a set speed
and the force emerges through the arm impedance (a slow drag across a detent).

Arm models (PHYSICS §22; the plant computes F_h = F_drive − b·ẋ − k·x_rel − m·ẍ with
x_rel measured from the contact point):

- **anchored** (``push``, ``push_release``, ``pull``, ``slam``): the arm's spring and
  damper act about the contact point. A *grip* that stays attached while the controller
  drives the carriage away (an intent-triggered automatic move, push-and-go) is dragged
  back by k·x_rel. That is correct physics, and the obstruction detector flags it (the
  W1.5 finding), so scenarios where an automatic move can follow a gesture use
  ``push_release`` (push-only contact separates) or ``hold``/``throw``.
- **walk-along** (``hold``, ``throw``): the arm impedance acts about a reference that
  follows the carriage, so the spring and damper carry no force (b = k = 0 are written
  to the event; the arm mass m still loads the carriage). The contact force is
  F_drive − m·ẍ, bounded by the profile, and it never drags a carriage that the
  controller moves. An explicit ``impedance=`` overrides this.
- **position-led** (:func:`guide`): an anchored arm driven by F_drive = k·s(t) + b·ṡ(t),
  which is exactly an arm whose anchor moves along s(t) (the plant's hand law is
  linear). A push-only contact (the default) separates if the carriage runs ahead.

Body-region data (effective mass, contact stiffness) are the ISO/TS 15066 values held
once in Rust (``fabrail_plant::interaction::ObstacleBody``, ``fabrail_params::presets``);
:data:`ISO15066_BODIES` is filled from there by :func:`iso15066_bodies`, never restated.

Owner: W0-C2 (stub), then A9 (Wave 1), S1 (Wave 2: hold, guide, walk-along arms).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import cast

import numpy as np

from ._pending import pending

__all__ = [
    "ANCHORED_GRIP_KINDS",
    "ARM_IMPEDANCE_RANGES",
    "BODY_NAMES",
    "BodyRegion",
    "HAND_KINDS",
    "INTENT_RISE_RANGE_S",
    "ISO15066_BODIES",
    "LEAN_RISE_MIN_S",
    "NOMINAL_IMPEDANCE",
    "SMOKE_RAMP_MIN_S",
    "WALK_ALONG_KINDS",
    "anchored_grip",
    "guide",
    "hand_profile",
    "iso15066_bodies",
    "lean",
    "obstacle",
    "release_s",
    "smoke_dp",
]

#: Hand-profile kinds (physics perspective §1.11: Push, PushRelease, Pull, Throw, Lean,
#: Slam, Pinch; ``hold`` added in Wave 2 for the walk-along grip).
HAND_KINDS: tuple[str, ...] = ("push", "push_release", "pull", "hold", "throw", "lean", "slam", "pinch")

#: Kinds whose arm walks along with the carriage (b = k = 0 unless ``impedance=`` is given).
WALK_ALONG_KINDS: tuple[str, ...] = ("hold", "throw")

#: Kinds with a grip contact and an anchored arm (drag the carriage if it is driven away).
ANCHORED_GRIP_KINDS: tuple[str, ...] = ("pull",)

#: Obstacle body regions accepted by ``fabrail.scenario/1`` (the schema's ``body`` enum).
BODY_NAMES: tuple[str, ...] = ("chest", "hand", "face", "leg")

#: Arm impedance ranges (physics perspective §1.11; core design §C.5 "Arm impedance").
ARM_IMPEDANCE_RANGES: dict[str, tuple[float, float]] = {
    "m_kg": (1.0, 2.0),
    "b_n_s_per_m": (5.0, 20.0),
    "k_n_per_m": (200.0, 1000.0),
}

#: The nominal arm impedance: the scenario schema's ``impedance`` default (mid-range).
NOMINAL_IMPEDANCE: dict[str, float] = {"m_kg": 1.5, "b_n_s_per_m": 10.0, "k_n_per_m": 500.0}

#: Intent force-ramp duration range, s (physics perspective §1.11: 0 → F_pk over 50–150 ms).
INTENT_RISE_RANGE_S: tuple[float, float] = (0.05, 0.15)

#: Minimum lean rise and smoke ramp, s (plan §2b item 11: lean rise ≥ 1 s, smoke ramp ≥ 2 s).
LEAN_RISE_MIN_S = 1.0
SMOKE_RAMP_MIN_S = 2.0

#: Default hold of each kind at F_pk, s (engineering choices: long enough for the intent
#: confirmation window of ≤ 170 ms, core design F12; push-to-open is a tap).
_HOLD_DEFAULT_S: dict[str, float] = {
    "push": 0.5,
    "pull": 0.5,
    "hold": 1.0,
    "push_release": 0.1,
    "throw": 0.25,
    "lean": 5.0,
}

#: Contact type by kind (schema ``hand.contact``).
_CONTACT: dict[str, str] = {
    "push": "push",
    "push_release": "push",
    "slam": "push",
    "pull": "grip",
    "hold": "grip",
    "throw": "grip",
}

#: Schema bound on a hand driving force, N (``hand.f_pts_n`` items).
_F_MAX_N = 2000.0


@dataclass(frozen=True)
class BodyRegion:
    """One ISO/TS 15066 body region: contact stiffness and effective mass."""

    name: str
    k_n_per_m: float
    m_eff_kg: float


#: Body regions by scenario name (``obstacle.body``); filled by :func:`iso15066_bodies`
#: from the native single source.
ISO15066_BODIES: dict[str, BodyRegion] = {}


def iso15066_bodies() -> dict[str, BodyRegion]:
    """:data:`ISO15066_BODIES`, filled on first use from the native single source.

    The values live once in Rust (``fabrail_plant::interaction::ObstacleBody``). They
    are read through ``_native.iso15066_bodies()`` when the extension provides it; the
    frozen Wave-0 surface does not, so until the integrator adds that accessor this
    raises :class:`fabrail.NotYetImplementedError` rather than restating the numbers.
    """
    if not ISO15066_BODIES:
        from . import require_native

        accessor = getattr(require_native(), "iso15066_bodies", None)
        if accessor is None:
            pending("human.iso15066_bodies (needs _native.iso15066_bodies)", "Wave 1.5", "A8")
        for name, row in dict(accessor()).items():
            ISO15066_BODIES[str(name)] = BodyRegion(str(name), float(row["k_n_per_m"]), float(row["m_eff_kg"]))
    return ISO15066_BODIES


def _finite(name: str, value: float) -> float:
    v = float(value)
    if not math.isfinite(v):
        raise ValueError(f"{name}: must be finite (got {value!r})")
    return v


def _impedance(rng: np.random.Generator | None, impedance: dict[str, float] | None) -> dict[str, float]:
    if impedance is not None:
        out = {k: _finite(f"impedance.{k}", impedance[k]) for k in ("m_kg", "b_n_s_per_m", "k_n_per_m")}
    elif rng is not None:
        out = {k: float(rng.uniform(lo, hi)) for k, (lo, hi) in ARM_IMPEDANCE_RANGES.items()}
    else:
        out = dict(NOMINAL_IMPEDANCE)
    return out


def hand_profile(
    kind: str,
    f_pk_n: float,
    t0_s: float,
    rise_s: float,
    rng: np.random.Generator | None = None,
    *,
    hold_s: float | None = None,
    impedance: dict[str, float] | None = None,
    x_m: float | None = None,
    config: str = "clamped",
) -> dict[str, object]:
    """A ``hand`` event: ``kind`` profile peaking at ``f_pk_n`` from ``t0_s`` (optional jitter from ``rng``).

    ``f_pk_n`` is signed (+x opens; a DRAWER push-to-open presses inward, so it is
    negative). ``rise_s`` is the 0 → F_pk ramp; ``hold_s`` the time at F_pk (kind
    default when ``None``). With ``rng``: F_pk × U(0.9, 1.1), rise × U(0.8, 1.2) (kept
    within 50–150 ms for the intent kinds) and the arm impedance drawn from
    :data:`ARM_IMPEDANCE_RANGES`, in that draw order; without it the nominal impedance.
    The walk-along kinds (:data:`WALK_ALONG_KINDS`) then write b = k = 0 (the draw
    count is unchanged, so a seeded stream stays aligned across kinds) unless
    ``impedance`` is given. ``lean`` returns :func:`lean` (rise ≥ 1 s) and ``pinch``
    returns :func:`obstacle` with the hand region at ``x_m`` (required there).

    The contact ends after the last profile point (the schema's release): ``push``,
    ``push_release``, ``pull`` and ``hold`` ramp the drive back to 0 first, ``throw``
    lets go with the force still applied.
    """
    if kind not in HAND_KINDS:
        raise ValueError(f"hand_profile: kind must be one of {', '.join(HAND_KINDS)} (got {kind!r})")
    t0 = _finite("t0_s", t0_s)
    if t0 < 0.0:
        raise ValueError(f"hand_profile: t0_s must be >= 0 (got {t0_s!r})")
    if kind == "pinch":
        if x_m is None:
            raise ValueError("hand_profile: kind 'pinch' needs x_m (the closing-edge position of the finger)")
        return obstacle(t0, "hand", x_m, config=config)
    f = _finite("f_pk_n", f_pk_n)
    rise = _finite("rise_s", rise_s)
    hold = _HOLD_DEFAULT_S.get(kind, 0.0) if hold_s is None else _finite("hold_s", hold_s)
    if rise <= 0.0 or hold < 0.0:
        raise ValueError(f"hand_profile: need rise_s > 0 and hold_s >= 0 (got {rise_s!r}, {hold_s!r})")
    if kind == "lean":
        return lean(t0, f, dur_s=2.0 * rise + hold, rise_s=rise)
    if rng is not None:
        f *= float(rng.uniform(0.9, 1.1))
        rise *= float(rng.uniform(0.8, 1.2))
        if kind in ("push", "push_release", "pull", "hold", "throw"):
            rise = min(max(rise, INTENT_RISE_RANGE_S[0]), INTENT_RISE_RANGE_S[1])
    imp = _impedance(rng, impedance)
    if kind in WALK_ALONG_KINDS and impedance is None:
        imp["b_n_s_per_m"] = 0.0
        imp["k_n_per_m"] = 0.0
    if kind == "slam":
        t_pts, f_pts = [0.0, rise, 2.0 * rise], [0.0, f, 0.0]
    elif kind == "throw":
        t_pts, f_pts = [0.0, rise, rise + hold], [0.0, f, f]
    elif kind == "push_release":
        t_pts, f_pts = [0.0, rise, rise + hold, rise + hold + 0.5 * rise], [0.0, f, f, 0.0]
    else:  # push, pull, hold
        t_pts, f_pts = [0.0, rise, rise + hold, 2.0 * rise + hold], [0.0, f, f, 0.0]
    if hold == 0.0 and kind != "slam":  # drop the repeated time point
        t_pts, f_pts = [t_pts[0], t_pts[1], *t_pts[3:]], [f_pts[0], f_pts[1], *f_pts[3:]]
    if abs(f) > _F_MAX_N:
        raise ValueError(f"hand_profile: |f_pk_n| must be <= {_F_MAX_N:g} N (schema bound; got {f})")
    return {
        "kind": "hand",
        "t_s": t0,
        "t_pts_s": t_pts,
        "f_pts_n": f_pts,
        "impedance": imp,
        "contact": _CONTACT[kind],
    }


def guide(
    t0_s: float,
    dist_m: float,
    v_mps: float,
    rng: np.random.Generator | None = None,
    *,
    rise_s: float = 0.2,
    dwell_s: float = 0.2,
    contact: str = "push",
    impedance: dict[str, float] | None = None,
    n_ramp: int = 8,
) -> dict[str, object]:
    """A position-led ``hand`` event: the hand's reference moves ``dist_m`` (signed, +x opens) at ``v_mps``.

    The reference s(t), measured from the contact point, accelerates at constant
    acceleration over ``rise_s``, cruises at ``v_mps``, decelerates over ``rise_s`` (a
    triangular profile with a lower peak when ``|dist_m| < v_mps·rise_s``), dwells
    ``dwell_s`` at ``dist_m`` and releases. The drive is F_drive = k·s + b·ṡ, so the
    contact force F_h = k·(s − x_rel) + b·(ṡ − ẋ) − m·ẍ is the arm impedance about the
    moving reference (the plant's hand law, PHYSICS §22, is linear in x_rel). The
    reference's own arm-mass term m·s̈ (≤ m·v/rise_s, 0.4 N at 1.5 kg, 50 mm/s, 0.2 s) is
    omitted. Ramps are sampled at ``n_ramp`` intervals (piecewise linear between them).

    ``contact="push"`` (default) is a palm: it carries force only along the motion and
    separates if the carriage runs ahead, so an automatic move that starts meanwhile is
    never dragged. ``"grip"`` holds the carriage to the moving reference both ways.
    With ``rng``: v × U(0.9, 1.1), rise × U(0.8, 1.2), then the arm impedance from
    :data:`ARM_IMPEDANCE_RANGES` (the :func:`hand_profile` draw order).
    """
    t0, dist, v = _finite("t0_s", t0_s), _finite("dist_m", dist_m), _finite("v_mps", v_mps)
    rise, dwell = _finite("rise_s", rise_s), _finite("dwell_s", dwell_s)
    if t0 < 0.0 or dist == 0.0 or v <= 0.0 or rise <= 0.0 or dwell < 0.0 or int(n_ramp) < 1:
        raise ValueError(
            "guide: need t0_s >= 0, dist_m != 0, v_mps > 0, rise_s > 0, dwell_s >= 0 and n_ramp >= 1 "
            f"(got {t0_s!r}, {dist_m!r}, {v_mps!r}, {rise_s!r}, {dwell_s!r}, {n_ramp!r})"
        )
    if contact not in ("push", "grip"):
        raise ValueError(f"guide: contact must be 'push' or 'grip' (got {contact!r})")
    if rng is not None:
        v *= float(rng.uniform(0.9, 1.1))
        rise *= float(rng.uniform(0.8, 1.2))
    imp = _impedance(rng, impedance)
    k, b = imp["k_n_per_m"], imp["b_n_s_per_m"]
    sign, d = (1.0 if dist > 0.0 else -1.0), abs(dist)
    v = min(v, d / rise)  # triangular profile when the distance is short
    acc = v / rise
    t_cruise = max(0.0, (d - v * rise) / v)
    n = int(n_ramp)
    pts: list[tuple[float, float, float]] = []  # (t, s, ṡ)
    for i in range(n + 1):  # acceleration
        t = rise * i / n
        pts.append((t, 0.5 * acc * t * t, acc * t))
    t1, s1 = rise + t_cruise, 0.5 * v * rise + v * t_cruise
    for i in range(0 if t_cruise > 0.0 else 1, n + 1):  # deceleration (joins the cruise line)
        tau = rise * i / n
        pts.append((t1 + tau, s1 + v * tau - 0.5 * acc * tau * tau, v - acc * tau))
    if dwell > 0.0:
        pts.append((t1 + rise + dwell, pts[-1][1], 0.0))
    t_pts = [p[0] for p in pts]
    f_pts = [sign * (k * p[1] + b * p[2]) + 0.0 for p in pts]  # + 0.0: no -0.0 in the JSON
    if max(abs(x) for x in f_pts) > _F_MAX_N:
        raise ValueError(f"guide: the drive k·s + b·ṡ exceeds the {_F_MAX_N:g} N schema bound (k = {k} N/m, dist = {dist} m)")
    return {"kind": "hand", "t_s": t0, "t_pts_s": t_pts, "f_pts_n": f_pts, "impedance": imp, "contact": contact}


def release_s(ev: dict[str, object]) -> float:
    """Absolute time at which a ``hand`` event's contact ends (its last profile point), s."""
    if ev.get("kind") != "hand":
        raise ValueError(f"release_s: expected a hand event (got kind {ev.get('kind')!r})")
    t_pts = cast("list[float]", ev["t_pts_s"])
    return float(cast(float, ev["t_s"])) + float(t_pts[-1])


def anchored_grip(ev: dict[str, object]) -> bool:
    """True for a ``hand`` event with a grip contact whose arm spring or damper acts about a fixed anchor.

    Such a hand drags the carriage back if the controller moves it while the hand is
    attached (the W1.5 finding). :func:`guide` grips are anchored about a moving
    reference and are reported too; the scenario catalog uses neither.
    """
    if ev.get("kind") != "hand" or ev.get("contact", "grip") != "grip":
        return False
    imp = ev.get("impedance") or NOMINAL_IMPEDANCE
    assert isinstance(imp, dict)
    return float(imp.get("k_n_per_m", 0.0)) > 0.0 or float(imp.get("b_n_s_per_m", 0.0)) > 0.0


def lean(t0_s: float, f_n: float, dur_s: float, rise_s: float = 1.0) -> dict[str, object]:
    """A ``lean`` event (rise ≥ 1 s, plan §2b item 11): trapezoid to ``f_n`` and back by ``t0_s + dur_s``."""
    t0, f, dur, rise = _finite("t0_s", t0_s), _finite("f_n", f_n), _finite("dur_s", dur_s), _finite("rise_s", rise_s)
    if rise < LEAN_RISE_MIN_S:
        raise ValueError(f"lean: rise_s must be >= {LEAN_RISE_MIN_S} s (plan §2b item 11; got {rise_s!r})")
    if not (t0 >= 0.0 and dur >= 2.0 * rise):
        raise ValueError(f"lean: need t0_s >= 0 and dur_s >= 2·rise_s (got {t0_s!r}, {dur_s!r}, {rise_s!r})")
    if abs(f) > 2000.0:
        raise ValueError(f"lean: |f_n| must be <= 2000 N (got {f_n!r})")
    return {"kind": "lean", "t_s": t0, "f_n": f, "dur_s": dur, "rise_s": rise}


def obstacle(
    t0_s: float,
    body: str,
    x_m: float,
    config: str = "clamped",
    k_edge_n_per_m: float | None = None,
    dur_s: float | None = None,
) -> dict[str, object]:
    """An ``obstacle`` event: an ISO/TS 15066 body region at ``x_m``, clamped or free."""
    if body not in BODY_NAMES:
        raise ValueError(f"obstacle: body must be one of {', '.join(BODY_NAMES)} (got {body!r})")
    if config not in ("clamped", "free"):
        raise ValueError(f"obstacle: config must be 'clamped' or 'free' (got {config!r})")
    t0 = _finite("t0_s", t0_s)
    if t0 < 0.0:
        raise ValueError(f"obstacle: t0_s must be >= 0 (got {t0_s!r})")
    ev: dict[str, object] = {"kind": "obstacle", "t_s": t0, "body": body, "x_m": _finite("x_m", x_m), "config": config}
    if k_edge_n_per_m is not None:
        k = _finite("k_edge_n_per_m", k_edge_n_per_m)
        if k <= 0.0:
            raise ValueError(f"obstacle: k_edge_n_per_m must be > 0 (got {k_edge_n_per_m!r})")
        ev["k_edge_n_per_m"] = k
    if dur_s is not None:
        d = _finite("dur_s", dur_s)
        if d <= 0.0:
            raise ValueError(f"obstacle: dur_s must be > 0 (got {dur_s!r})")
        ev["dur_s"] = d
    return ev


def smoke_dp(t0_s: float, dp_pa: float, ramp_s: float = 2.0, dur_s: float | None = None) -> dict[str, object]:
    """A ``smoke_dp`` event (ramp ≥ 2 s, plan §2b item 11): ΔP ramps to ``dp_pa`` (+ pushes toward +x)."""
    t0, dp, ramp = _finite("t0_s", t0_s), _finite("dp_pa", dp_pa), _finite("ramp_s", ramp_s)
    if ramp < SMOKE_RAMP_MIN_S:
        raise ValueError(f"smoke_dp: ramp_s must be >= {SMOKE_RAMP_MIN_S} s (plan §2b item 11; got {ramp_s!r})")
    if t0 < 0.0 or abs(dp) > 1000.0:
        raise ValueError(f"smoke_dp: need t0_s >= 0 and |dp_pa| <= 1000 Pa (got {t0_s!r}, {dp_pa!r})")
    ev: dict[str, object] = {"kind": "smoke_dp", "t_s": t0, "dp_pa": dp, "ramp_s": ramp}
    if dur_s is not None:
        d = _finite("dur_s", dur_s)
        if d < 2.0 * ramp:
            raise ValueError(f"smoke_dp: dur_s must be >= 2·ramp_s (got {dur_s!r}, {ramp_s!r})")
        ev["dur_s"] = d
    return ev
