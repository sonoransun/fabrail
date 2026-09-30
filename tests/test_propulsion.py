"""Propulsion in closed loop (FAB-PRP-006/007/010/011).

The real cascade (position P, velocity PI, feed-forward m̂a + b̂v + ripple table,
PHYSICS §16) drives the LSM plant through the full executive. DRAWER, DOOR and WALL run
on Carriage6 in voltage coil mode; HEAVY on Travel1d (plan §2b S3, lead decision D1:
HEAVY acceptance runs on Heave1/Travel1d while its Carriage6 roll design is open).

Measured on plant truth (``x_m``) against the class tolerances in ``control.motion``
(pos_tol_closed_m / pos_tol_travel_m, checked (E) against SPECIFICATION §2). Runs are
shared through the module-scoped ``runs`` cache. Owner: B4 (Wave 2).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Any

import numpy as np
import pytest

from fabrail import scenarios
from fabrail.config import RunConfig
from fabrail.results import RunResult

CLASSES = ("drawer", "door", "wall", "heavy")
#: SPEC FAB-PRP-006 position accuracy closed / travel, m.
POS_TOL = {"drawer": (0.3e-3, 1e-3), "door": (0.5e-3, 2e-3), "wall": (1e-3, 5e-3), "heavy": (1e-3, 5e-3)}
#: SPEC FAB-PRP-007 velocity bandwidth ω_v/2π, Hz.
OMEGA_V_HZ = {"drawer": 8.0, "door": 6.0, "wall": 3.0, "heavy": 2.0}
#: ``motion_mode`` wire value of Trajectory (fabrail-types MotionMode).
MODE_TRAJECTORY = 2
#: Fault codes of severity ≥ SafeStop (fabrail-types fault.rs).
_STOPPING = {
    0x0101,
    0x0102,
    0x0110,
    0x0111,
    0x0130,
    0x0201,
    0x0202,
    0x0203,
    0x0204,
    0x0210,
    0x0211,
    0x0212,
    0x0301,
    0x0302,
    0x0303,
    0x0310,
    0x0401,
    0x0402,
    0x0403,
    0x0404,
    0x0405,
    0x0406,
    0x0407,
    0x0408,
    0x0501,
    0x0502,
    0x0503,
    0x0505,
    0x0506,
    0x0601,
    0x0602,
    0x0603,
    0x0604,
    0x0605,
    0x0606,
    0x060B,
    0x0701,
}
#: Linear thermal expansion of a steel rail, 1/K (EN 1993-1-1 §3.2.6: 12e-6), used only
#: to check that a thermal-expansion effect is present at all.
ALPHA_STEEL_PER_K = 12e-6


def _tier(cls: str) -> str:
    return "travel1d" if cls == "heavy" else "carriage6"


class _Runs:
    """Module-level run cache: key → (RunResult, params, spec dict)."""

    def __init__(self, native: Any) -> None:
        self._n = native
        self._cache: dict[str, tuple[RunResult, Any, dict[str, Any]]] = {}

    def get(self, key: str, factory: Callable[[], tuple[Any, dict[str, Any], dict[str, bytes]]]):
        if key not in self._cache:
            p, d, series = factory()
            d["events"] = sorted(d["events"], key=lambda e: float(e["t_s"]))
            scenarios.validate_spec(d)
            scenarios.cross_check(d, series, p)
            sc = self._n.Scenario.from_json(scenarios.to_json(d), series)
            self._cache[key] = (RunResult.from_native(self._n.run(p, sc)), p, d)
        return self._cache[key]


@pytest.fixture(scope="module")
def runs(native: Any) -> _Runs:
    return _Runs(native)


def _catalog(
    name: str,
    cls: str,
    *,
    fidelity: str | None = None,
    overrides: dict[str, object] | None = None,
    edit: Callable[[dict[str, Any], dict[str, bytes], Any], None] | None = None,
):
    def factory():
        cfg = RunConfig(rail_class=cls, coil="voltage", overrides=dict(overrides or {}))
        p = cfg.params()
        d, series = scenarios.spec(name, p, cfg, fidelity=fidelity)
        if edit is not None:
            edit(d, series, p)
        return p, d, series

    return factory


def _stopping(r: RunResult) -> list[str]:
    return [f"{e.t_s:.3f}s:0x{int(e.a):04X}" for e in r.events if e.kind == "FaultRaised" and int(e.a) in _STOPPING]


def _hold_to_run(d: dict[str, Any]) -> list[dict[str, Any]]:
    return [e for e in d["events"] if e["kind"] == "hold_to_run"]


# ---------------------------------------------------------------------------------
# FAB-PRP-006 position accuracy
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-PRP-006")
@pytest.mark.parametrize("cls", CLASSES)
def test_position_accuracy(runs, cls):
    """PRP-006: settled position error closed / travel ±0.3/±1, ±0.5/±2, ±1/±5, ±1/±5 mm.

    Contract: the class tolerances equal SPECIFICATION §2 ((E) parameters); after a
    close onto the closed stop (``terminus_soft_close``; DRAWER seats) the truth position
    over the last 0.2 s is within the closed tolerance of ``x_closed``; after an open to
    a travel position (``soft_open``: DRAWER/DOOR to the open end, WALL/HEAVY ``go_to``)
    it is within the travel tolerance of the target. No stopping fault in either run.
    """
    tol_c, tol_t = POS_TOL[cls]
    r, p, d = runs.get(f"close:{cls}", _catalog("terminus_soft_close", cls, fidelity=_tier(cls)))
    assert abs(p.get("control.motion.pos_tol_closed_m") - tol_c) <= 1e-12, "(E) closed tolerance, SPEC §2"
    assert abs(p.get("control.motion.pos_tol_travel_m") - tol_t) <= 1e-12, "(E) travel tolerance, SPEC §2"
    assert not _stopping(r), f"{cls} close interrupted: {_stopping(r)}"
    tail = r.t_s >= r.t_s[-1] - 0.2
    err_c = float(r["x_m"][tail].mean() - p.get("design.rail.x_closed_m"))
    assert abs(err_c) <= tol_c, f"(S) ±{tol_c * 1e3} mm closed, SPEC FAB-PRP-006: {cls} error {err_c * 1e3:+.3f} mm"

    r2, p2, d2 = runs.get(f"open:{cls}", _catalog("soft_open", cls, fidelity=_tier(cls)))
    assert not _stopping(r2), f"{cls} open interrupted: {_stopping(r2)}"
    target = float(scenarios.info(d2)["target_x_m"])
    tail2 = r2.t_s >= r2.t_s[-1] - 0.2
    err_t = float(r2["x_m"][tail2].mean() - target)
    assert abs(err_t) <= tol_t, f"(S) ±{tol_t * 1e3} mm travel, SPEC FAB-PRP-006: {cls} error {err_t * 1e3:+.3f} mm"


# ---------------------------------------------------------------------------------
# FAB-PRP-007 velocity loop bandwidth
# ---------------------------------------------------------------------------------

#: (start offset from x_closed m, lean time s, fit window s) per class: the lean step
#: lands in the constant-speed cruise of an ``open`` and the fit window stays in it.
_VBW = {"drawer": (0.03, 0.80, 0.55), "door": (0.0, 2.0, 1.0), "wall": (0.0, 4.0, 1.0), "heavy": (0.0, 6.0, 1.5)}


def _vbw_edit(cls: str, lean: bool = True):
    x_off, t_lean, win = _VBW[cls]

    def edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
        f = 0.6 * p.get("control.intent.f_th_n")  # below the hand threshold: the cascade, not admittance
        d["initial"] = {"lev": "levitated", "x_m": p.get("design.rail.x_closed_m") + x_off}
        d["events"] = _hold_to_run(d) + [{"kind": "command", "t_s": 0.5, "cmd": "open"}]
        if lean:
            d["events"].append({"kind": "lean", "t_s": t_lean, "f_n": f, "dur_s": win + 0.3, "rise_s": 0.0005})
        d["duration_s"] = t_lean + win + 0.5
        d["record"]["rate_hz"] = 5000.0 if cls != "heavy" else 1000.0
        d["record"]["channels"] = ["x_m", "x_ref_m", "x_meas_m", "motion_mode", "m_hat_kg"]
        d["metrics"]["windows"] = {}
        d["doc"] = f"{d['doc']} [B4: lean step {f:.2f} N at {t_lean} s]" if lean else d["doc"]

    return edit


def _impulse(t: np.ndarray, w: float, kx: float, ki: float) -> np.ndarray:
    from scipy import signal

    den = [1.0, w, (kx + ki) * w * w, kx * ki * w**3]
    _, y = signal.impulse(signal.lti([1.0], den), T=t)
    return y


@pytest.mark.req("FAB-PRP-007")
@pytest.mark.parametrize("cls", CLASSES)
def test_velocity_loop_bandwidth(runs, cls):
    """PRP-007: the velocity cascade bandwidth ω_v = 2π×{8, 6, 3, 2} Hz within ±20 %.

    Contract: a lean force step F = 0.6·F_th (below the hand threshold) during the
    cruise of an ``open`` displaces the carriage, relative to an otherwise identical run
    without the step (same noise seed, so the acceleration-end transient cancels), by
    e(s) = (F/m)/(s³ + ω s² + (k_x + k_i)ω² s + k_x k_i ω³) (K_v = m̂ω_v, K_x = k_x ω_v,
    K_vi = k_i K_v ω_v; PHYSICS §16). A least-squares fit of that impulse shape (free ω
    and gain) to the truth error over the window recovers ω_v within ±20 % with R² ≥ 0.95
    and a gain within ±20 % of F/m. Before the step the feed-forward cascade tracks the
    reference within the class travel tolerance (encoder error, SPEC FAB-PRP-006).
    """
    from scipy import optimize

    r, p, d = runs.get(f"vbw:{cls}", _catalog("soft_open", cls, fidelity=_tier(cls), edit=_vbw_edit(cls)))
    rb, _, _ = runs.get(f"vbw0:{cls}", _catalog("soft_open", cls, fidelity=_tier(cls), edit=_vbw_edit(cls, lean=False)))
    assert not _stopping(r), f"{cls} cruise interrupted: {_stopping(r)}"
    lean = next(e for e in d["events"] if e["kind"] == "lean")
    t_l, f = float(lean["t_s"]), float(lean["f_n"])
    win = _VBW[cls][2]
    t = r.t_s
    k0 = int(np.searchsorted(t, t_l))
    traj = r["motion_mode"] == MODE_TRAJECTORY
    k1 = k0
    while k1 < t.size and traj[k1] and t[k1] < t_l + win:
        k1 += 1
    assert traj[k0] and t[k1 - 1] - t_l >= 0.5 * win, f"{cls}: the lean does not fall in a Trajectory cruise"
    pre = (t >= t_l - 0.2) & (t < t_l)
    track = float(np.abs(r["x_meas_m"][pre] - r["x_ref_m"][pre]).max())
    tol_t = POS_TOL[cls][1]
    assert track <= tol_t, f"(S) ≤ {tol_t * 1e3} mm, feed-forward tracking before the step: {track * 1e3:.4f} mm"
    e = r["x_m"] - rb["x_m"]
    tt, ee = t[k0:k1] - t_l, e[k0:k1] - e[k0 - 1]
    w0 = 2.0 * math.pi * p.get("control.motion.omega_v_hz")
    assert abs(w0 - 2.0 * math.pi * OMEGA_V_HZ[cls]) <= 1e-9, "(E) ω_v preset, SPEC FAB-PRP-007"
    kx, ki = p.get("control.motion.kx_ratio"), p.get("control.motion.ki_ratio")

    def cost(lw: float) -> float:
        y = _impulse(tt, w0 * math.exp(lw), kx, ki)
        a = float(np.dot(y, ee) / np.dot(y, y))
        return float(np.sum((ee - a * y) ** 2))

    res = optimize.minimize_scalar(cost, bounds=(-1.0, 1.0), method="bounded")
    w = w0 * math.exp(res.x)
    y = _impulse(tt, w, kx, ki)
    gain = float(np.dot(y, ee) / np.dot(y, y))
    r2 = 1.0 - float(np.sum((ee - gain * y) ** 2) / np.sum((ee - ee.mean()) ** 2))
    m = float(r["m_hat_kg"][k0])
    assert r2 >= 0.95, f"(P) fit R² ≥ 0.95: {r2:.4f} (ω {w:.2f} rad/s)"
    assert abs(w / w0 - 1.0) <= 0.20, (
        f"(S) ±20 %, SPEC FAB-PRP-007: {cls} fitted ω_v {w:.2f} rad/s vs {w0:.2f} rad/s ({100 * (w / w0 - 1):+.1f} %)"
    )
    assert abs(gain / (f / m) - 1.0) <= 0.20, f"(P) ±20 %: fitted gain {gain:.4f} vs F/m̂ {f / m:.4f} m/s²"


# ---------------------------------------------------------------------------------
# FAB-PRP-010 regeneration
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-PRP-010")
@pytest.mark.parametrize("cls", CLASSES)
def test_regen_bus_rise(runs, cls):
    """PRP-010: a low-energy stop raises the bus by ≤ 16 %; the brake chopper takes the rest.

    Contract: the brake-chopper threshold lies above the nominal bus and at most 16 %
    above it; through a close (``terminus_soft_close``) and an open (``soft_open``) at
    the unsensed caps, the simulated bus voltage never exceeds 1.16·V_bus.
    """
    p0 = RunConfig(rail_class=cls, coil="voltage").params()
    v_bus, v_ch = p0.get("design.power.v_bus_v"), p0.get("design.power.v_chopper_v")
    assert v_bus < v_ch <= 1.16 * v_bus, f"(A) chopper threshold {v_ch} V not in (V_bus, 1.16·V_bus] = ({v_bus}, {1.16 * v_bus}]"
    for name, key in (("terminus_soft_close", "close"), ("soft_open", "open")):
        r, p, _ = runs.get(f"{key}:{cls}", _catalog(name, cls, fidelity=_tier(cls)))
        rise = float(r["v_bus_v"].max() / v_bus - 1.0)
        assert rise <= 0.16, f"(S) ≤ 16 %, SPEC FAB-PRP-010: {cls} {name} bus rise {100 * rise:.2f} %"


# ---------------------------------------------------------------------------------
# FAB-PRP-011 rail thermal expansion
# ---------------------------------------------------------------------------------

_THERMAL_COMPRESS = {"design.thermal.c_th_mag_j_per_k": 0.5, "design.thermal.c_th_coil_j_per_k": 0.5}


def _expansion_edit(cls: str):
    def edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
        x0, travel = p.get("design.rail.x_closed_m"), p.get("design.rail.travel_m")
        x_far = x0 + min(travel, 3.0)
        d["initial"] = {"lev": "levitated", "x_m": x_far}
        v = p.derived()["v_close_cap_mps"]
        t_close = 30.0
        t_move = (x_far - x0) / v + p.derived()["terminus_t_arrive_s"] + 10.0
        d["events"] = [
            {"kind": "temp_ramp", "t_s": 1.0, "dt_k": 20.0, "dur_s": 20.0},
            *([{"kind": "hold_to_run", "t_s": t_close - 0.1, "on": True}] if cls == "heavy" else []),
            {"kind": "command", "t_s": t_close, "cmd": "close"},
        ]
        d["duration_s"] = math.ceil(t_close + t_move)
        d["record"]["rate_hz"] = 100.0
        d["record"]["channels"] = ["x_m", "x_meas_m", "t_mag_c", "lev_state"]
        d["metrics"]["windows"] = {}

    return edit


@pytest.mark.req("FAB-PRP-011")
@pytest.mark.parametrize("cls", ["wall", pytest.param("heavy", marks=pytest.mark.slow)])
def test_thermal_expansion_homing(runs, cls):
    """PRP-011: rail thermal expansion (sized 15 m × 20 K) is absorbed by homing.

    Contract (Travel1d): parked 3 m from the closed stop, a +20 K ramp (thermal nodes
    compressed) must change the encoder-vs-truth reading there by at least half of
    α_steel·x·ΔT (the precondition that the plant models the scale expansion at all;
    otherwise the check is vacuous and fails as "not modelled"); a close then homes at
    the closed stop, ending within the class closed tolerance of ``x_closed``.
    15 m × 20 K × 12e-6 = 3.6 mm sits inside the ±5 mm travel tolerance (SPEC §2).
    """
    r, p, d = runs.get(
        f"expansion:{cls}",
        _catalog("terminus_soft_close", cls, fidelity="travel1d", overrides=_THERMAL_COMPRESS, edit=_expansion_edit(cls)),
    )
    t = r.t_s
    x0 = p.get("design.rail.x_closed_m")
    before = t < 1.0
    hot = (t >= 25.0) & (t < 29.0)
    off = r["x_meas_m"] - r["x_m"]
    d_off = float(off[hot].mean() - off[before].mean())
    x_far = float(r["x_m"][before].mean()) - x0
    need = 0.5 * ALPHA_STEEL_PER_K * x_far * 20.0
    assert abs(d_off) >= need, (
        f"(S) SPEC FAB-PRP-011: rail thermal expansion is not modelled: encoder − truth changed by {d_off * 1e6:.2f} µm "
        f"over +20 K at {x_far:.2f} m (expected ≈ {2 * need * 1e6:.0f} µm); homing cannot be verified"
    )
    assert not _stopping(r), f"{cls} close interrupted: {_stopping(r)}"
    tail = t >= t[-1] - 0.5
    err = float(r["x_m"][tail].mean() - x0)
    tol = POS_TOL[cls][0]
    assert abs(err) <= tol, f"(S) ±{tol * 1e3} mm, SPEC FAB-PRP-011: closed error after homing {err * 1e3:+.3f} mm"
