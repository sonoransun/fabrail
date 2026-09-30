"""🧪 Safe-RL haptics: a control-barrier-function (CBF) filter around a learned "feel" policy.

Documented-only item; FRONTIER §F10. Reinforcement learning or Bayesian optimisation
could tune the haptic feel of a door, but only inside a safety filter that enforces the
pedestrian caps no matter what the learned policy commands (frontier research §3h;
Ames et al. 2019 CBF; Berkenkamp et al. 2016 SafeOpt [REF-MEM]). The learned part is
never safety-rated; in fabrail the safety monitor stays the independent integer monitor
(PHYSICS §18), and this module is a what-if study of the filter alone.

**Filter** (discrete-time CBF, Agrawal & Sreenath 2017 [REF-MEM]; PHYSICS §16, §17, §18).
The carriage is a 1-D mass: v⁺ = a·v + (dt/m)·u with a = 1 − b·dt/m (viscous drag b).
Barriers h_E = E_max − ½·m·v² and h_p = p_max − m·|v| must satisfy h⁺ ≥ (1 − γ)·h, which
bounds |v⁺| ≤ v_b with

    v_b = min( √(v² + 2γ·h_E/m),  |v| + γ·h_p/m )

and the actuator stays within the static force cap |u| ≤ F (unsensed, or sensed when
presence sensing is fitted). The QP  min (u − u_RL)²  subject to these linear bounds is
scalar, so its exact solution is the projection u = clip(u_RL, lo, hi). With h ≥ 0 the
set always contains u = 0, so the filter is always feasible and, on the model it
filters, never lets energy, momentum or force exceed their caps.

**Caps.** The pedestrian caps (energy, momentum, the unsensed and sensed static force)
are read from ``fabrail._native.Params.preset(cls)``, i.e. from
``fabrail_params::presets``; they are never restated here (FAB-EXO-114). Functions
raise RuntimeError when the native module is not built.

Missing orders: 0 (research tier; nothing is missing in orders of magnitude).

numpy only (FAB-EXO-002) plus the native parameter read. Every public function returns
a stamped :class:`~fabrail.exotic.frontier._record.FrontierRecord`.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import NamedTuple

import numpy as np

from ._record import FrontierRecord, make_record

__all__ = ["GAMMA", "MISSING_ORDERS", "POLICIES", "PedestrianCaps", "cbf_filter", "simulate"]

#: Discrete CBF decay γ per step (h⁺ ≥ (1 − γ)·h), design choice (frontier research §3h; FRONTIER §F10).
GAMMA = 0.05
#: Missing orders of magnitude for this item (research tier).
MISSING_ORDERS = 0.0
#: Adversarial learned policies exercised by :func:`simulate`.
POLICIES: tuple[str, ...] = ("push", "bang_bang", "random", "resonant")
#: Default moving mass, kg (frontier research §3h: "for a 100 kg door"; FRONTIER §F10).
M_DOOR_KG = 100.0
#: Viscous drag of the carriage, N·s/m (design choice; small against the caps; FRONTIER §F10; PHYSICS §9).
B_DRAG_N_S_PER_M = 2.0
#: Relative margin inside every cap, so float rounding never lands on the wrong side.
_MARGIN = 1e-9

_NOTE = (
    "Safe-RL haptics with a CBF filter: research tier, documented only; the filter enforces the "
    "pedestrian energy, momentum and force caps imported from fabrail_params::presets; the learned "
    "policy is never safety-rated."
)


class PedestrianCaps(NamedTuple):
    """The pedestrian caps of one rail class, read from ``fabrail._native`` (FAB-EXO-114)."""

    e_max_j: float
    p_max_kgmps: float
    f_static_n: float
    f_static_sensed_n: float
    f_drive_max_n: float
    rail_class: str
    source: str


def _pedestrian_caps(rail_class: str = "door") -> PedestrianCaps:
    """Read the caps from the native preset of ``rail_class`` (fabrail_params::presets).

    Raises:
        RuntimeError: ``fabrail._native`` is not built.
    """
    try:
        from fabrail import _native
    except ImportError as exc:  # pragma: no cover - depends on the build
        raise RuntimeError(
            "safe_rl reads the pedestrian caps from fabrail._native (fabrail_params::presets, FAB-EXO-114); "
            "build it with `maturin develop --release`"
        ) from exc
    params = _native.Params.preset(rail_class)
    return PedestrianCaps(
        e_max_j=float(params.get("safety.e_max_j")),
        p_max_kgmps=float(params.get("safety.p_max_kgmps")),
        f_static_n=float(params.get("safety.f_static_n")),
        f_static_sensed_n=float(params.get("safety.f_static_sensed_n")),
        f_drive_max_n=float(params.get("safety.f_drive_max_n")),
        rail_class=rail_class,
        source=f"fabrail._native.Params.preset({rail_class!r}) (fabrail_params::presets)",
    )


def _bounds(
    v: np.ndarray, m: float, dt: float, b: float, caps: PedestrianCaps, f_cap: float, gamma: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """(lo, hi) of the filtered force and the barrier-only (lo_bar, hi_bar), N."""
    a = 1.0 - b * dt / m
    e_max = caps.e_max_j * (1.0 - _MARGIN)
    p_max = caps.p_max_kgmps * (1.0 - _MARGIN)
    speed = np.abs(v)
    h_e = e_max - 0.5 * m * v * v
    h_p = p_max - m * speed
    vb_e = np.sqrt(np.maximum(v * v + 2.0 * gamma * h_e / m, 0.0))
    vb_p = np.maximum(speed + gamma * h_p / m, 0.0)
    vb = np.minimum(vb_e, vb_p)
    lo_bar = (-vb - a * v) * m / dt
    hi_bar = (vb - a * v) * m / dt
    return np.maximum(lo_bar, -f_cap), np.minimum(hi_bar, f_cap), lo_bar, hi_bar


def _filter(
    u_rl: np.ndarray, v: np.ndarray, m: float, dt: float, b: float, caps: PedestrianCaps, f_cap: float, gamma: float
) -> tuple[np.ndarray, np.ndarray]:
    """The projected force and the active constraint per element.

    Labels: "none" (unmodified), "barrier" (an energy or momentum barrier bound was the
    tighter one), "force" (the static force cap was) or "brake" (h < 0 on entry: the
    set is empty, so brake toward v⁺ = 0 within the force cap).
    """
    lo, hi, lo_bar, hi_bar = _bounds(v, m, dt, b, caps, f_cap, gamma)
    a = 1.0 - b * dt / m
    feasible = lo <= hi
    brake = np.clip(-a * v * m / dt, -f_cap, f_cap)
    u = np.where(feasible, np.clip(u_rl, lo, hi), brake)
    upper = np.where(hi_bar < f_cap, "barrier", "force")
    lower = np.where(lo_bar > -f_cap, "barrier", "force")
    label = np.where(u_rl > hi, upper, np.where(u_rl < lo, lower, "none"))
    return u, np.where(feasible, label, "brake")


def _check(m_kg: float, dt_s: float, gamma: float, b: float) -> None:
    if not (math.isfinite(m_kg) and m_kg > 0.0) or not (math.isfinite(dt_s) and dt_s > 0.0):
        raise ValueError("m_kg and dt_s must be finite and > 0")
    if not 0.0 < gamma <= 1.0:
        raise ValueError(f"gamma must lie in (0, 1] (got {gamma!r})")
    if b < 0.0 or b * dt_s / m_kg >= 1.0:
        raise ValueError("drag must satisfy 0 <= b·dt/m < 1")


def cbf_filter(
    u_rl_n: Sequence[float] | np.ndarray | float = 400.0,
    v_mps: Sequence[float] | np.ndarray | None = None,
    m_kg: float = M_DOOR_KG,
    dt_s: float = 1e-3,
    sensed: bool = False,
    gamma: float = GAMMA,
    b_n_s_per_m: float = B_DRAG_N_S_PER_M,
    rail_class: str = "door",
) -> FrontierRecord:
    """Filter learned force commands at given speeds (the exact scalar CBF-QP; FRONTIER §F10).

    Args:
        u_rl_n: the learned policy's force command(s), N.
        v_mps: carriage speed(s), m/s (default −0.3 … 0.3 m/s in 0.1 steps).
        m_kg, dt_s, gamma, b_n_s_per_m: plant and filter constants.
        sensed: presence sensing fitted (uses the sensed static force cap).
        rail_class: preset the caps are read from.

    Returns:
        A record with ``u_safe_n``, ``u_lo_n``, ``u_hi_n``, ``modified`` and the ``caps``.
    """
    _check(m_kg, dt_s, gamma, b_n_s_per_m)
    caps = _pedestrian_caps(rail_class)
    f_cap = caps.f_static_sensed_n if sensed else caps.f_static_n
    v = np.round(np.arange(-3, 4) / 10.0, 12) if v_mps is None else np.asarray(v_mps, dtype=float)
    u_rl, v = np.broadcast_arrays(np.asarray(u_rl_n, dtype=float), v)
    u, label = _filter(u_rl, v, m_kg, dt_s, b_n_s_per_m, caps, f_cap, gamma)
    lo, hi, _, _ = _bounds(v, m_kg, dt_s, b_n_s_per_m, caps, f_cap, gamma)
    return make_record(
        "safe_rl",
        "safe_rl.cbf_filter",
        MISSING_ORDERS,
        _NOTE,
        {
            "u_rl_n": u_rl,
            "v_mps": v,
            "m_kg": m_kg,
            "dt_s": dt_s,
            "sensed": sensed,
            "gamma": gamma,
            "b_n_s_per_m": b_n_s_per_m,
            "rail_class": rail_class,
        },
        {
            "u_safe_n": u,
            "u_lo_n": lo,
            "u_hi_n": hi,
            "modified": u != u_rl,
            "constraint": label,
            "force_cap_n": f_cap,
            "caps": caps._asdict(),
        },
    )


def _policy(name: str, t: float, f: float, rng: np.random.Generator) -> float:
    if name == "push":
        return f
    if name == "bang_bang":
        return f if int(t / 0.5) % 2 == 0 else -f
    if name == "random":
        return float(rng.uniform(-f, f))
    return f * math.sin(2.0 * math.pi * 2.0 * t)  # "resonant": 2 Hz forcing


def simulate(
    m_kg: float = M_DOOR_KG,
    duration_s: float = 4.0,
    dt_s: float = 1e-3,
    sensed: bool = False,
    gamma: float = GAMMA,
    b_n_s_per_m: float = B_DRAG_N_S_PER_M,
    seed: int = 42,
    rail_class: str = "door",
    policies: Sequence[str] = POLICIES,
) -> FrontierRecord:
    """Adversarial learned policies with and without the filter (FRONTIER §F10).

    Each policy commands up to the class's dynamic drive-force limit; the filtered run
    must show zero violations of the energy, momentum and force caps, while the
    unfiltered run shows what the policy alone would reach.

    Returns:
        A record with per-policy ``ke_max_j``, ``momentum_max_kgmps``, ``force_max_n``,
        ``v_max_mps``, the violation counts ``energy_violations``,
        ``momentum_violations``, ``force_violations`` and ``total_violations``, the
        unfiltered ``ke_max_unfiltered_j``, the speed caps ``v_cap_energy_mps``
        (√(2E/m)) and ``v_cap_momentum_mps`` (p/m), and the ``v_mps`` traces.
    """
    _check(m_kg, dt_s, gamma, b_n_s_per_m)
    unknown = [p for p in policies if p not in POLICIES]
    if unknown or not policies:
        raise ValueError(f"policies must be a non-empty subset of {POLICIES} (got {list(policies)!r})")
    if not (math.isfinite(duration_s) and duration_s > 0.0):
        raise ValueError("duration_s must be finite and > 0")
    caps = _pedestrian_caps(rail_class)
    f_cap = caps.f_static_sensed_n if sensed else caps.f_static_n
    f_cmd = caps.f_drive_max_n
    n_steps = int(round(duration_s / dt_s))
    n_pol = len(policies)
    rng = np.random.default_rng(seed)
    a = 1.0 - b_n_s_per_m * dt_s / m_kg
    v = np.zeros(n_pol)
    v_raw = np.zeros(n_pol)
    trace = np.zeros((n_pol, n_steps + 1))
    ke_max = np.zeros(n_pol)
    p_max = np.zeros(n_pol)
    f_max = np.zeros(n_pol)
    ke_raw_max = np.zeros(n_pol)
    viol_e = np.zeros(n_pol, dtype=np.int64)
    viol_p = np.zeros(n_pol, dtype=np.int64)
    viol_f = np.zeros(n_pol, dtype=np.int64)
    for k in range(n_steps):
        t = k * dt_s
        u_rl = np.array([_policy(p, t, f_cmd, rng) for p in policies])
        u, _ = _filter(u_rl, v, m_kg, dt_s, b_n_s_per_m, caps, f_cap, gamma)
        v = a * v + (dt_s / m_kg) * u
        v_raw = a * v_raw + (dt_s / m_kg) * u_rl
        trace[:, k + 1] = v
        ke = 0.5 * m_kg * v * v
        mom = m_kg * np.abs(v)
        ke_max = np.maximum(ke_max, ke)
        p_max = np.maximum(p_max, mom)
        f_max = np.maximum(f_max, np.abs(u))
        ke_raw_max = np.maximum(ke_raw_max, 0.5 * m_kg * v_raw * v_raw)
        viol_e += ke > caps.e_max_j
        viol_p += mom > caps.p_max_kgmps
        viol_f += np.abs(u) > f_cap
    total = int(viol_e.sum() + viol_p.sum() + viol_f.sum())
    return make_record(
        "safe_rl",
        "safe_rl.simulate",
        MISSING_ORDERS,
        _NOTE,
        {
            "m_kg": m_kg,
            "duration_s": duration_s,
            "dt_s": dt_s,
            "sensed": sensed,
            "gamma": gamma,
            "b_n_s_per_m": b_n_s_per_m,
            "seed": seed,
            "rail_class": rail_class,
            "policies": list(policies),
        },
        {
            "caps": caps._asdict(),
            "force_cap_n": f_cap,
            "policy_force_n": f_cmd,
            "v_cap_energy_mps": math.sqrt(2.0 * caps.e_max_j / m_kg),
            "v_cap_momentum_mps": caps.p_max_kgmps / m_kg,
            "ke_max_j": ke_max,
            "momentum_max_kgmps": p_max,
            "force_max_n": f_max,
            "v_max_mps": np.max(np.abs(trace), axis=1),
            "energy_violations": viol_e,
            "momentum_violations": viol_p,
            "force_violations": viol_f,
            "total_violations": total,
            "ke_max_unfiltered_j": ke_raw_max,
            "t_s": np.arange(n_steps + 1) * dt_s,
            "v_mps": trace,
        },
    )
