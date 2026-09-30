"""Safety functions in closed loop (FAB-PRP-004, FAB-SAF-001/004/005/006/008/009/010/011/012/018).

The real core (governor, directional clamp, obstruction, supervisor) and the independent
integer monitor (fabrail-safety, via the arbitration in fabrail-rt) run against the f64
plant through ``_native.run``. Core faults are injected with the scenario ``core_fault``
event (fabrail-rt ``fault-injection``, always on in fabrail-sim); sensor and driver
faults with ``fault``.

Tiers: DRAWER, DOOR and WALL on Carriage6 in voltage coil mode unless the requirement is
an x-axis quantity whose closed form assumes a 1-D body (clamp force, stall force), which
run on Travel1d; HEAVY on Travel1d / Heave1 (lead decision D1).

Honesty: every standard is "checked in simulation against numeric limits from
[edition]" (EN 16005 2011 draft, ISO/TS 15066, ADA 404.2.9), never a compliance claim.
Owner: B4 (Wave 2).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from typing import Any

import numpy as np
import pytest

from fabrail import human, scenarios
from fabrail.config import RunConfig
from fabrail.results import RunResult

CLASSES = ("drawer", "door", "wall", "heavy")
#: Standard gravity, m/s² (fabrail_params::derive::G0_MPS2).
G0 = 9.80665
#: FaultCode wire values (fabrail-types fault.rs).
F = {
    "CoilOvercurrent": 0x0201,
    "LsmDriver": 0x0212,
    "MainsLost": 0x0303,
    "HoldupLow": 0x0304,
    "NonFinite": 0x0407,
    "GapEnvelope": 0x0501,
    "PayloadMismatch": 0x0504,
    "SaturationPersist": 0x0505,
    "EnergyLimit": 0x0601,
    "MomentumLimit": 0x0602,
    "ForceLimit": 0x0603,
    "Overspeed": 0x0604,
    "Obstruction": 0x0607,
    "NotHoldToRun": 0x0609,
}
_NAMES = {v: k for k, v in F.items()}
#: Fault codes of severity ≥ SafeStop (fault.rs).
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
#: ``demand`` wire values (fabrail-types SafetyDemand).
D_SAFESTOP, D_SAFELAND, D_STO, D_STO_ASC = 2, 3, 4, 5
#: ``lev_state`` wire values (LevState).
LEV_LANDED, LEV_LEVITATED_IDLE, LEV_MOVING, LEV_FAULT = 3, 5, 6, 11
#: ``motion_mode`` wire values (MotionMode).
MODE_OFF, MODE_REVERSE = 0, 5
#: ``contact_state`` bits: seated, landed.
CS_SEATED, CS_LANDED = 2, 4
ADA_N = 22.2


def _tier(cls: str, *, axis: bool = False) -> str:
    if cls == "heavy":
        return "travel1d"
    return "travel1d" if axis else "carriage6"


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
    options: dict[str, Any] | None = None,
    overrides: dict[str, object] | None = None,
    edit: Callable[[dict[str, Any], dict[str, bytes], Any], None] | None = None,
):
    def factory():
        cfg = RunConfig(rail_class=cls, coil="voltage", overrides=dict(overrides or {}))
        p = cfg.params()
        d, series = scenarios.spec(name, p, cfg, fidelity=fidelity, options=options)
        if edit is not None:
            edit(d, series, p)
        return p, d, series

    return factory


def _faults(r: RunResult) -> list[tuple[float, int]]:
    return [(e.t_s, int(e.a)) for e in r.events if e.kind == "FaultRaised"]


def _named(r: RunResult) -> list[str]:
    return [f"{t:.3f}s:{_NAMES.get(c, hex(c))}" for t, c in _faults(r)]


def _stopping(r: RunResult) -> list[str]:
    return [f"{t:.3f}s:{_NAMES.get(c, hex(c))}" for t, c in _faults(r) if c in _STOPPING]


def _first(t: np.ndarray, mask: np.ndarray) -> float | None:
    k = int(np.argmax(mask))
    return float(t[k]) if mask.size and mask[k] else None


def _events(r: RunResult, kind: str) -> list[Any]:
    return [e for e in r.events if e.kind == kind]


def _fast_period(p: Any) -> float:
    return p.get("control.rates.fast_decim") / p.get("control.rates.base_hz")


def _hold_to_run(d: dict[str, Any]) -> list[dict[str, Any]]:
    return [e for e in d["events"] if e["kind"] == "hold_to_run"]


# ---------------------------------------------------------------------------------
# FAB-PRP-004 directional clamp
# ---------------------------------------------------------------------------------

#: Obstruction detection disabled (residual at its parameter maximum): the directional
#: clamp is the backstop under test, so the obstruction reversal must not pre-empt it.
_NO_OBSTRUCTION = {"control.haptic.obstruction.residual_n": 1.0e6}
#: Clamp tolerance on the measured LSM force (current-loop ripple), relative.
CLAMP_TOL = 0.02


def _stall_edit(offset_m: float):
    def edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
        x0 = p.get("design.rail.x_closed_m")
        for e in d["events"]:
            if e["kind"] == "obstacle":
                e["x_m"] = x0 + offset_m
                e["body"] = "hand"
                e["k_edge_n_per_m"] = 1.0e9  # stiff: a stall, not a springy bounce
        d["initial"]["x_m"] = x0 + offset_m + 0.1
        d["duration_s"] = 12.0
        d["record"]["rate_hz"] = 1000.0
        d["record"]["channels"] = ["x_m", "v_mps", "f_obstacle_n", "f_lsm_n", "force_cap_n", "demand", "motion_mode"]
        d["metrics"]["windows"] = {}

    return edit


_STALL_CASES = [(c, off) for c in CLASSES for off in (0.02, 0.2)]


@pytest.mark.req("FAB-PRP-004")
@pytest.mark.parametrize(("cls", "offset_m"), _STALL_CASES)
def test_stall_force_clamp(runs, cls, offset_m):
    """PRP-004: stall force ≤ 67 N unsensed (150 N in the last 50 mm), < 80 N within 5 s, drive ≤ F_drive_max.

    Contract (Carriage6; HEAVY Travel1d; obstruction detection disabled so the clamp is
    the only limit):
    the carriage closes onto a stiff clamped hand 20 mm (last 50 mm) or 200 mm from the
    stop. |F_lsm| ≤ F_drive_max always; wherever |v| < 2 mm/s has persisted > 0.2 s the
    force is ≤ 67 N (≤ 150 N inside the last 50 mm); from 5 s after the first contact it
    stays < 80 N (+2 % measurement tolerance). Braking is never clamped: a ``stop`` at
    cruise brakes at a force above 67 N when m_rated·a_max exceeds it (DOOR/WALL/HEAVY).
    "Checked in simulation against numeric limits from EN 16005 (2011 draft)".
    """
    name = "obstruction_hand_edge"
    r, p, d = runs.get(
        f"stall:{cls}:{offset_m}",
        _catalog(name, cls, fidelity=_tier(cls), overrides=_NO_OBSTRUCTION, edit=_stall_edit(offset_m)),
    )
    s = p.get
    t, v, f = r.t_s, r["v_mps"], np.abs(r["f_lsm_n"])
    f_drive, f_static, f_sensed, f_final = (
        s("safety.f_drive_max_n"),
        s("safety.f_static_n"),
        s("safety.f_close_last50_n"),
        s("safety.f_static_final_n"),
    )
    assert f.max() <= f_drive * (1 + CLAMP_TOL), f"(S) ≤ F_drive_max {f_drive} N, SPEC FAB-PRP-004: {f.max():.1f} N"
    contact = _first(t, r["f_obstacle_n"] > 0.0)
    assert contact is not None, f"{cls}: never reached the obstacle (faults {_named(r)})"
    still = np.abs(v) < s("control.supervisor.stall_v_mps")
    run_len = np.zeros(t.size)
    for k in range(1, t.size):
        run_len[k] = run_len[k - 1] + (t[k] - t[k - 1]) if still[k] else 0.0
    stall = run_len > s("control.supervisor.stall_t_s")
    in_last = (r["x_m"] - s("design.rail.x_closed_m")) < s("safety.last_zone_m")
    cap = np.where(in_last, f_sensed, f_static)
    over = stall & (f > cap * (1 + CLAMP_TOL))
    assert not over.any(), (
        f"(S) stall ≤ {f_static:.0f}/{f_sensed:.0f} N, SPEC FAB-PRP-004: {cls} {f[over].max():.1f} N at {t[over][0]:.2f} s"
    )
    late = t >= contact + s("safety.t_static_final_s")
    assert late.any(), "run too short for the 5 s final-force check"
    assert f[late].max() < f_final * (1 + CLAMP_TOL), (
        f"(S) < {f_final:.0f} N within 5 s, SPEC FAB-PRP-004: {cls} {f[late].max():.1f} N at "
        f"{t[late][int(np.argmax(f[late]))]:.2f} s (contact {contact:.2f} s)"
    )

    m_a = p.get("design.carriage.m_rated_kg") * p.get("control.motion.a_max_mps2")
    if offset_m == 0.2 and m_a > f_static * 1.05:

        def stop_edit(dd, series, pp):
            dd["events"] = _hold_to_run(dd) + [
                {"kind": "command", "t_s": 0.5, "cmd": "open"},
                {"kind": "command", "t_s": 0.5 + 0.6 * dd["duration_s"] / 2, "cmd": "stop"},
            ]
            dd["record"]["channels"] = ["x_m", "v_mps", "f_lsm_n"]
            dd["metrics"]["windows"] = {}

        rb, _, _ = runs.get(f"brake:{cls}", _catalog("soft_open", cls, fidelity=_tier(cls, axis=True), edit=stop_edit))
        brake = rb["f_lsm_n"] * np.sign(rb["v_mps"]) < 0.0
        fb = float(np.abs(rb["f_lsm_n"][brake]).max()) if brake.any() else 0.0
        assert fb > f_static, f"(S) braking is never clamped: peak braking force {fb:.1f} N ≤ {f_static} N"


# ---------------------------------------------------------------------------------
# FAB-SAF-001 governor sabotage
# ---------------------------------------------------------------------------------

#: Truth B_r scale per class that makes m̂ read low (m̂ ≈ m/br²) while the zero-power
#: gap stays inside the float band (DOOR ≤ 5.8 mm → br ≤ 1.092, WALL ≤ 11.5 mm → br ≤
#: 1.081; DRAWER's band is wide). The catalog's √2 (m̂ ≈ 0.5·m) drives DOOR/WALL out of
#: the band into coil saturation (SaturationPersist), a plant-mismatch question.
BR_SABOTAGE = {"drawer": 1.2, "door": 1.09, "wall": 1.08}


@pytest.mark.req("FAB-SAF-001")
@pytest.mark.parametrize("cls", ["drawer", "door", "wall"])
def test_governor_sabotage(runs, cls):
    """SAF-001: m_gov = max(m_rated, 1.05·m̂): sabotaging m̂ low never raises speed past the m_rated envelope.

    Contract (Carriage6, catalog ``governor_sabotage``): with the truth magnets stronger
    than the model (B_r ×1.2 / 1.09 / 1.08: m̂ reads ≈ 0.69 / 0.84 / 0.86·m) both runs
    finish without a stopping fault; the governor energy equals ½·m_gov·v̂² with
    m_gov = max(m_rated, 1.05·m̂) (±2 %); the sabotaged peak speed stays within the
    m_rated envelope min(class cap, √(2E/m_rated), p_max/m_rated). Where 1.05·m ≤ m_rated
    (DRAWER at m*) m_gov is m_rated in both runs and the sabotage changes nothing: peak
    speed and peak governor energy within 1 % of the unsabotaged move. Where m ≈ m_rated
    (DOOR, WALL) the low m̂ can only floor m_gov at m_rated, so the envelope is the check.
    """
    runs_ = {}
    br_s = BR_SABOTAGE[cls]
    for br in (1.0, br_s):
        runs_[br] = runs.get(f"gov:{cls}:{br}", _catalog("governor_sabotage", cls, options={"br_scale": br}))
    (r0, p, _), (r1, _, _) = runs_[1.0], runs_[br_s]
    obs = [round(t, 3) for t, c in _faults(r1) if c == F["Obstruction"]]
    for r in (r0, r1):
        assert not _stopping(r), f"{cls}: stopping faults {_stopping(r)}"
    m_rated, m_star = p.get("design.carriage.m_rated_kg"), p.get("design.carriage.m_star_kg")
    moving = np.abs(r1["v_mps"]) > 1e-3
    ratio = float(np.median(r1["m_hat_kg"][moving]) / m_star)
    for r in (r0, r1):
        sel = r["e_kin_gov_j"] > 0.05 * r["e_kin_gov_j"].max()
        m_gov = np.maximum(m_rated, 1.05 * r["m_hat_kg"][sel])
        e_ref = 0.5 * m_gov * r["v_hat_mps"][sel] ** 2
        worst = float(np.abs(r["e_kin_gov_j"][sel] / e_ref - 1.0).max())
        assert worst <= 0.02, f"(P) ±2 %, SPEC FAB-SAF-001: governor energy vs ½·m_gov·v̂², worst {100 * worst:.2f} %"
    der = p.derived()
    env = min(
        max(der["v_open_cap_mps"], der["v_close_cap_mps"]),
        math.sqrt(2.0 * p.get("safety.e_max_j") / m_rated),
        p.get("safety.p_max_kgmps") / m_rated,
    )
    v0, v1 = float(np.abs(r0["v_mps"]).max()), float(np.abs(r1["v_mps"]).max())
    assert v1 <= env * 1.005, f"(S) SPEC FAB-SAF-001: sabotaged peak {v1:.5f} m/s above the m_rated envelope {env:.5f} m/s"
    if 1.05 * m_star <= m_rated:
        for key, a, b in (("peak speed", v0, v1), ("peak governor energy", r0.kpis["ke_max_gov_j"], r1.kpis["ke_max_gov_j"])):
            assert abs(b / a - 1.0) <= 0.01, (
                f"(S) sabotage changes nothing, SPEC FAB-SAF-001: {key} {a:.5f} → {b:.5f} (m̂/m {ratio:.3f}; "
                f"obstruction flagged at {obs})"
            )


# ---------------------------------------------------------------------------------
# FAB-SAF-004 force-time profile / FAB-SAF-006 obstruction
# ---------------------------------------------------------------------------------

_OBS_CASES = [
    ("drawer", "obstruction_hand_edge"),
    ("door", "obstruction_hand_edge"),
    ("wall", "obstruction_hand_edge"),
    ("heavy", "obstruction_hand_edge"),
    ("door", "obstruction_chest"),
    ("wall", "obstruction_chest"),
    ("heavy", "obstruction_chest"),
]


def _obstruction_run(runs: _Runs, cls: str, name: str):
    def edit(d, series, p):
        d["duration_s"] = d["duration_s"] + 6.0
        d["record"]["rate_hz"] = 1000.0 if cls == "heavy" else 5000.0
        d["metrics"]["windows"] = {}

    return runs.get(f"{name}:{cls}", _catalog(name, cls, fidelity=_tier(cls), edit=edit))


@pytest.mark.req("FAB-SAF-004")
@pytest.mark.parametrize(("cls", "name"), _OBS_CASES)
def test_en16005_force_profile(runs, cls, name):
    """SAF-004: contact force ≤ 400 N, > 150 N for ≤ 0.75 s, < 80 N within 5 s, ≤ 67 N static unsensed.

    Contract: closing onto a clamped ISO/TS 15066 body (chest; hand 20 mm from the stop):
    the truth contact force never exceeds 400 N, exceeds 150 N for at most 0.75 s in
    total, stays ≤ 67 N from 0.75 s after the first contact (unsensed presets) and < 80 N
    from 5 s after it. "Checked in simulation against numeric limits from EN 16005 (2011
    draft)."
    """
    r, p, _ = _obstruction_run(runs, cls, name)
    t, fo = r.t_s, r["f_obstacle_n"]
    contact = _first(t, fo > 0.0)
    assert contact is not None, f"{cls} {name}: never reached the obstacle (faults {_named(r)})"
    dt = 1.0 / r.rate_hz
    assert fo.max() <= 400.0, f"(S) ≤ 400 N, SPEC FAB-SAF-004: peak {fo.max():.1f} N"
    above = float(np.sum(fo > 150.0) * dt)
    assert above <= 0.75, f"(S) > 150 N for ≤ 0.75 s, SPEC FAB-SAF-004: {above:.3f} s"
    static = t >= contact + 0.75
    assert fo[static].max() <= p.get("safety.f_static_n"), f"(S) ≤ 67 N static, SPEC FAB-SAF-004: {fo[static].max():.1f} N"
    late = t >= contact + 5.0
    assert late.any() and fo[late].max() < 80.0, f"(S) < 80 N within 5 s, SPEC FAB-SAF-004: {fo[late].max():.1f} N"


@pytest.mark.req("FAB-SAF-006")
@pytest.mark.parametrize(("cls", "name"), _OBS_CASES)
def test_obstruction_stop_reverse(runs, cls, name):
    """SAF-006: an obstruction stops the carriage and reverses 50 mm (DOOR/WALL) or releases to 0 N (DRAWER/HEAVY).

    Contract: an Obstruction event follows the first contact; DOOR/WALL enter Reverse
    within 20 ms of the detection and back off 50 mm ±10 mm (P) from the detection
    position with no stopping fault; DRAWER/HEAVY drop the LSM force to |F| < 1 N within
    20 ms and hold it released for 0.5 s.
    """
    r, p, _ = _obstruction_run(runs, cls, name)
    t = r.t_s
    obs = [e.t_s for e in _events(r, "Obstruction")]
    contact = _first(t, r["f_obstacle_n"] > 0.0)
    assert obs, f"(S) SPEC FAB-SAF-006: no obstruction detected (contact at {contact}, faults {_named(r)})"
    t_det = obs[0]
    k_det = int(np.searchsorted(t, t_det))
    if cls in ("door", "wall"):
        t_rev = _first(t, (t >= t_det) & (r["motion_mode"] == MODE_REVERSE))
        assert t_rev is not None and t_rev - t_det <= 0.020, f"(S) reverse ≤ 20 ms, SPEC FAB-SAF-006: {t_rev} vs {t_det:.3f}"
        back = float(r["x_m"][k_det:].max() - r["x_m"][k_det])
        rev = p.get("control.haptic.obstruction.reverse_m")
        assert abs(back - rev) <= 0.010, f"(P) reverse {rev * 1e3:.0f} ±10 mm, SPEC FAB-SAF-006: backed off {back * 1e3:.1f} mm"
        assert not _stopping(r), f"stopping faults during the reversal: {_stopping(r)}"
    else:
        k20, k500 = int(np.searchsorted(t, t_det + 0.020)), int(np.searchsorted(t, t_det + 0.5))
        f_after = np.abs(r["f_lsm_n"][k20:k500])
        assert f_after.max() < 1.0, f"(S) release to 0 N within 20 ms, SPEC FAB-SAF-006: |F_lsm| up to {f_after.max():.2f} N"


# ---------------------------------------------------------------------------------
# FAB-SAF-005 clamp force
# ---------------------------------------------------------------------------------

#: (class, body, E at contact J, edge seal) → SPEC anchor N: DOOR chest 290.7 N and hand
#: 503.5 N at 1.69 J, hand through the 10 N/mm edge seal 173 N, WALL chest 157.7 N at
#: its 0.5 J cap; DRAWER hand at 1.69 J.
_CLAMP_CASES = [
    ("door", "chest", False, 290.7),
    ("door", "hand", False, 503.5),
    ("door", "hand", True, 173.0),
    ("drawer", "hand", True, 173.0),
    ("wall", "chest", False, 157.7),
]
#: ISO/TS 15066 body stiffness, N/m (chest 25 N/mm, hand 75 N/mm; ``obstacle.body``).
K_BODY = {"chest": 25.0e3, "hand": 75.0e3}


def _clamp_edit(body: str, seal: bool, e_j: float):
    def edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
        x_obs = p.get("design.rail.x_closed_m") + 0.2
        ob: dict[str, Any] = {"kind": "obstacle", "t_s": 0.0, "body": body, "x_m": x_obs, "config": "clamped"}
        if not seal:
            ob["k_edge_n_per_m"] = 1.0e9
        v0 = math.sqrt(2.0 * e_j / p.get("design.carriage.m_star_kg"))
        d["initial"] = {"lev": "levitated", "x_m": x_obs + 0.005, "v_mps": -v0}
        # LSM gate-driver fault at t = 0: STO (no ASC below v_runaway), so the carriage
        # coasts into the body with no drive and no admittance.
        d["events"] = [ob, {"kind": "fault", "t_s": 0.0, "what": "lsm_driver", "ch": 0}]
        d["duration_s"] = 0.4
        d["record"]["rate_hz"] = 1000.0
        d["record"]["channels"] = ["x_m", "v_mps", "f_obstacle_n", "f_lsm_n"]
        d["metrics"]["windows"] = {}

    return edit


@pytest.mark.req("FAB-SAF-005")
@pytest.mark.parametrize(("cls", "body", "seal", "anchor"), _CLAMP_CASES)
def test_clamp_force_sim(native, runs, cls, body, seal, anchor):
    """SAF-005: clamp force F = √(2Ek) within ±0.5 % in simulation; every unsensed cap ≤ 400 N via the seal.

    Contract: the closed form ``clamp_force(E, k)`` reproduces the SPEC anchor (0.05 %)
    at the class KE cap (DOOR/DRAWER 1.69 J, WALL ½·m·v_cap²) with k the body stiffness,
    in series with the 10 N/mm edge seal where fitted; on Travel1d (the closed form's
    1-D body) a carriage coasting with the LSM in STO into the clamped body peaks at
    √(m·k)·v_contact within ±0.5 %; for every class the unsensed KE cap through the edge
    seal and a hand gives ≤ 400 N. "Checked in simulation against numeric limits from
    ISO/TS 15066."
    """
    p = RunConfig(rail_class=cls, coil="voltage").params()
    k_edge = p.get("design.damping.k_edge_n_per_m")
    k = native.series_stiffness(K_BODY[body], k_edge) if seal else K_BODY[body]
    e_cap = (
        p.derived()["ke_at_cap_j"]
        if cls != "wall"
        else 0.5 * p.get("design.carriage.m_rated_kg") * p.get("safety.v_close_max_mps") ** 2
    )
    f_cf = native.clamp_force(e_cap, k)
    quoted = 0.5 if anchor == 173.0 else 0.05  # half a unit of the quoted last digit
    assert abs(f_cf - anchor) <= quoted, f"(E) anchor {anchor} N as quoted, SPEC FAB-SAF-005: closed form {f_cf:.2f} N"
    for c in CLASSES:
        pc = RunConfig(rail_class=c, coil="voltage").params()
        f_seal = native.clamp_force(pc.derived()["ke_at_cap_j"], native.series_stiffness(K_BODY["hand"], k_edge))
        assert f_seal <= 400.0, f"(A) unsensed cap through the edge seal ≤ 400 N: {c} {f_seal:.1f} N"

    r, pp, _ = runs.get(
        f"clamp:{cls}:{body}:{seal}",
        _catalog("obstruction_hand_edge", cls, fidelity="travel1d", edit=_clamp_edit(body, seal, e_cap)),
    )
    fo = r["f_obstacle_n"]
    k0 = int(np.argmax(fo > 0.0))
    assert fo[k0] > 0.0, "the carriage never reached the body"
    v_c = float(abs(r["v_mps"][k0 - 1]))
    m = pp.get("design.carriage.m_star_kg")
    f_ref = v_c * math.sqrt(m * k)
    assert np.abs(r["f_lsm_n"][k0:]).max() < 1.0, "the LSM was not off during the contact"
    assert abs(fo.max() / f_ref - 1.0) <= 0.005, (
        f"(S) ±0.5 %, SPEC FAB-SAF-005: {cls} {body}{' + seal' if seal else ''} peak {fo.max():.2f} N vs √(m·k)·v "
        f"{f_ref:.2f} N ({100 * (fo.max() / f_ref - 1):+.2f} %; KE at contact {0.5 * m * v_c * v_c:.3f} J)"
    )


# ---------------------------------------------------------------------------------
# FAB-SAF-008 overspeed / FAB-SAF-011 ASC
# ---------------------------------------------------------------------------------


def _push_edit(f_n: float, inject: bool):
    def edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
        x0 = p.get("design.rail.x_closed_m")
        d["initial"] = {"lev": "levitated", "x_m": x0 + 0.02}
        d["events"] = [human.hand_profile("push", f_n, 1.5, 0.1, None, hold_s=2.0)]
        if inject:
            d["events"].append({"kind": "core_fault", "t_s": 1.5, "what": "iq_gain_x3"})
        d["duration_s"] = 4.5
        d["record"]["rate_hz"] = 5000.0
        d["record"]["channels"] = ["x_m", "v_mps", "f_lsm_n", "f_hand_n", "demand", "motion_mode", "force_cap_n"]
        d["metrics"]["windows"] = {}

    return edit


#: Hard sustained pushes that drive the carriage past its cap (N).
_PUSH_N = {"drawer": 30.0, "door": 80.0, "wall": 150.0}


def _overspeed_runs(runs: _Runs):
    out = []
    for cls in ("drawer", "door", "wall"):
        for inject in (False, True):
            out.append(
                (
                    cls,
                    inject,
                    *runs.get(f"push:{cls}:{inject}", _catalog("push_open", cls, edit=_push_edit(_PUSH_N[cls], inject))),
                )
            )
    return out


def _drive_episodes(r: RunResult, p: Any) -> list[tuple[float, float]]:
    """Intervals where the windowed speed exceeds 1.1·v_lim while the drive does positive work.

    Windowed speed over ``safety.vel_window_s`` from truth x (the monitor windows its
    integer encoder counts the same way); drive energised in the direction of motion:
    F·v > 0 and |F| > F_drive_max/20 (the monitor's LSM-active threshold). Only
    episodes lasting longer than ``safety.overspeed_t_s`` are returned.
    """
    t, x = r.t_s, r["x_m"]
    n = max(1, int(round(p.get("safety.vel_window_s") * r.rate_hz)))
    vw = np.zeros_like(x)
    vw[n:] = (x[n:] - x[:-n]) / (n / r.rate_hz)
    v_lim = max(p.get("safety.v_open_max_mps"), p.get("safety.v_close_max_mps"))
    f = r["f_lsm_n"]
    active = (np.abs(vw) > 1.1 * v_lim) & (f * vw > 0.0) & (np.abs(f) > p.get("safety.f_drive_max_n") / 20.0)
    eps = []
    k = 0
    while k < t.size:
        if active[k]:
            j = k
            while j + 1 < t.size and active[j + 1]:
                j += 1
            if t[j] - t[k] > p.get("safety.overspeed_t_s"):
                eps.append((float(t[k]), float(t[j])))
            k = j + 1
        else:
            k += 1
    return eps


@pytest.mark.req("FAB-SAF-008")
def test_overspeed_trips(runs):
    """SAF-008: v > 1.1·v_lim for > 10 ms with the drive energised → STO + ASC.

    Contract: over hard hand pushes past the cap (DRAWER 30 N, DOOR 80 N, WALL 150 N),
    with and without an injected 3× LSM current gain, every episode in which the
    windowed speed exceeds 1.1·v_lim while the drive does positive work (F·v > 0) for
    more than 10 ms is followed by demand STO+ASC within overspeed_t + vel_window + one
    fast period of the episode start; at least one such episode occurs (non-vacuous).
    A carriage driven past the cap by the hand alone (drive idle or braking) is excused
    (core design F7, lead decision D4): no Overspeed fault in a run with no drive episode.
    """
    total = 0
    for cls, inject, r, p, _ in _overspeed_runs(runs):
        eps = _drive_episodes(r, p)
        total += len(eps)
        lat = p.get("safety.overspeed_t_s") + p.get("safety.vel_window_s") + _fast_period(p)
        for t0, _ in eps:
            t_asc = _first(r.t_s, (r.t_s >= t0) & (r["demand"] == D_STO_ASC))
            assert t_asc is not None and t_asc - t0 <= lat, (
                f"(S) STO+ASC within {lat * 1e3:.1f} ms, SPEC FAB-SAF-008: {cls} inject={inject} drive overspeed from "
                f"{t0:.3f} s, STO+ASC at {t_asc} (faults {_named(r)})"
            )
        if not eps:
            os_faults = [t for t, c in _faults(r) if c == F["Overspeed"]]
            assert not os_faults, f"{cls} inject={inject}: Overspeed trip at {os_faults} with no drive-driven overspeed (F7/D4)"
    assert total >= 1, "no drive-driven overspeed episode occurred: the check is vacuous"


@pytest.mark.req("FAB-SAF-011")
def test_asc_policy(runs):
    """SAF-011: ASC only above v_runaway or in a seismic landing; released after stop on egress classes.

    Contract: (a) a mains loss at standstill lands without ASC (demand never STO+ASC) in
    every class; (b) an STO-class fault below v_runaway (LSM gate-driver fault at cruise)
    demands STO without ASC; (c) the egress DOOR's seismic landing (synthetic network
    alert, M7 at 60 km) ends with ASC released: a hand push after the landing moves the
    door with no LSM drag (|F_lsm| < 0.5 N while it moves) and ≤ 22.2 N.
    """
    for cls in CLASSES:
        fid = "heave1" if cls == "heavy" else "carriage6"
        r, _, _ = runs.get(f"power_loss:{cls}", _catalog("power_loss", cls, fidelity=fid))
        assert not (r["demand"] == D_STO_ASC).any(), f"(S) SPEC FAB-SAF-011: ASC on a standstill mains loss ({cls})"

    def cruise_fault(d, series, p):
        d["events"] = _hold_to_run(d) + [
            {"kind": "command", "t_s": 0.5, "cmd": "open"},
            {"kind": "fault", "t_s": 2.0, "what": "lsm_driver", "ch": 0},
        ]
        d["metrics"]["windows"] = {}

    r, p, _ = runs.get("asc:door:cruise", _catalog("soft_open", "door", edit=cruise_fault))
    k = int(np.searchsorted(r.t_s, 2.0))
    assert abs(r["v_mps"][k]) < p.get("safety.v_runaway_mps"), "precondition: cruise below v_runaway"
    dem = r["demand"][k:]
    assert (dem == D_STO).any() and not (dem == D_STO_ASC).any(), (
        f"(S) SPEC FAB-SAF-011: STO-class fault below v_runaway demanded {sorted(set(dem.tolist()))} (expected STO, no ASC)"
    )

    def push_after(d, series, p):
        t_s = float(scenarios.info(d)["t_s_s"])
        d["events"].append(human.hand_profile("push", 15.0, t_s + 5.0, 1.0, None, hold_s=2.0))
        d["duration_s"] = math.ceil(t_s + 10.0)
        d["record"]["channels"] = ["x_m", "v_mps", "f_lsm_n", "f_hand_n", "demand", "lev_state"]
        d["metrics"]["windows"] = {}

    r, p, d = runs.get("asc:egress", _catalog("egress_eew", "door", fidelity="travel1d", options={"t_p_s": 8.0}, edit=push_after))
    landed = [e.t_s for e in _events(r, "Landed")]
    assert landed, f"egress door never landed (faults {_named(r)})"
    moving = (r.t_s > landed[0]) & (np.abs(r["v_mps"]) > 5e-3)
    assert moving.any(), "the hand push did not move the landed door"
    drag = float(np.abs(r["f_lsm_n"][moving]).max())
    assert drag < 0.5, f"(S) ASC released after stop, SPEC FAB-SAF-011: LSM drag {drag:.2f} N on the landed egress door"
    push = float(np.abs(r["f_hand_n"][r.t_s > landed[0]]).max())
    assert push <= ADA_N, f"(S) ≤ {ADA_N} N: manual push {push:.1f} N"


# ---------------------------------------------------------------------------------
# FAB-SAF-009 safe landing / FAB-SAF-010 manual force
# ---------------------------------------------------------------------------------

_LOSS_CASES = [("drawer", False), ("door", False), ("wall", False), ("heavy", False), ("door", True)]


def _power_loss(runs: _Runs, cls: str, moving: bool):
    fid = "heave1" if (cls == "heavy" and not moving) else ("travel1d" if cls == "heavy" else "carriage6")

    def edit(d, series, p):
        d["record"]["channels"] = [
            "x_m",
            "v_mps",
            "z_m",
            "f_lsm_n",
            "motion_mode",
            "demand",
            "lev_state",
            "e_hold_j",
            "contact_state",
            "f_rollers_n",
        ]
        d["metrics"]["windows"] = {}

    opts = {"moving": True} if moving else None
    return runs.get(f"loss:{cls}:{moving}", _catalog("power_loss", cls, fidelity=fid, options=opts, edit=edit))


@pytest.mark.req("FAB-SAF-009")
@pytest.mark.parametrize(("cls", "moving"), _LOSS_CASES)
def test_safe_landing_on_supply_loss(runs, cls, moving):
    """SAF-009: on supply loss SS1 lands at ≤ 5 mm/s, then STO; hold-up ≥ 3× the landing need.

    Contract: after the ``power_loss`` event the carriage touches down (rollers or
    seat) at a heave speed ≤ 5 mm/s (``touchdown_v_mps``), the LSM is then off (motion
    mode Off, |F_lsm| < 0.5 N over the last 0.5 s), and the supercap energy spent from
    the loss to 0.5 s after touchdown is at most a third of the usable energy at the
    loss (E_hold − ½·C·V_min²). DOOR also while moving; HEAVY on Heave1 (moving: Travel1d).
    """
    r, p, d = _power_loss(runs, cls, moving)
    t_loss = next(float(e["t_s"]) for e in d["events"] if e["kind"] == "power_loss")
    t = r.t_s
    cs = r["contact_state"].astype(np.int64)
    t_td = _first(t, (t >= t_loss) & ((cs & (CS_LANDED | CS_SEATED)) != 0))
    assert t_td is not None, f"{cls}: never touched down after the loss (faults {_named(r)})"
    v_td = r.kpis.get("touchdown_v_mps")
    assert v_td is not None and v_td <= 5e-3, f"(S) ≤ 5 mm/s, SPEC FAB-SAF-009: touchdown {v_td}"
    tail = t >= t[-1] - 0.5
    assert np.all(r["motion_mode"][tail] == MODE_OFF) and np.abs(r["f_lsm_n"][tail]).max() < 0.5, "LSM not off after landing"
    e = r["e_hold_j"]
    e_min = 0.5 * p.get("design.power.supercap_c_f") * p.get("design.power.supercap_v_min_v") ** 2
    k_loss = int(np.searchsorted(t, t_loss))
    k_end = min(int(np.searchsorted(t, t_td + 0.5)), t.size - 1)
    usable = float(e[k_loss] - e_min)
    spent = float(e[k_loss] - e[k_end])
    assert 3.0 * spent <= usable, (
        f"(S) hold-up ≥ 3× the landing need, SPEC FAB-SAF-009: spent {spent:.1f} J of {usable:.1f} J usable "
        f"(margin {usable / max(spent, 1e-9):.2f}×; faults {_named(r)})"
    )


@pytest.mark.req("FAB-SAF-010")
@pytest.mark.parametrize("cls", ["door", "wall"])
def test_manual_force_after_landing(native, runs, cls):
    """SAF-010: manual force after landing ≤ 22.2 N on DOOR; relieved value and no-PM bound reported.

    Contract: the PM-relieved roller force ``manual_force(relieved=True)`` is at most the
    no-PM bound μ·m·g (roller μ 0.003, (E) against 0.003·m*·g₀); WALL is flagged when the
    bound exceeds 22.2 N (it does not: 14.7 N). In simulation, after a mains-loss landing a
    hand ramping to 30 N over 3 s breaks the carriage loose (``breakaway_n``) at ≤ 22.2 N
    on DOOR (reported with the relieved value for WALL). "Checked in simulation against
    numeric limits from ADA 404.2.9 (22.2 N)."
    """
    p = RunConfig(rail_class=cls, coil="voltage").params()
    relieved, bound = native.manual_force(p, True), native.manual_force(p, False)
    mu = p.get("design.landing.rollers_mu")
    assert mu == 0.003, f"(E) roller μ 0.003, PLAN §2b.19: {mu}"
    assert abs(bound / (mu * p.get("design.carriage.m_star_kg") * G0) - 1.0) <= 1e-12, "(E) no-PM bound μ·m·g"
    assert relieved <= bound, f"relieved {relieved:.3f} N above the no-PM bound {bound:.3f} N"
    if cls == "wall":
        assert bound <= ADA_N, f"WALL flagged: no-PM bound {bound:.2f} N > {ADA_N} N (SPEC FAB-SAF-010)"

    def edit(d, series, p_):
        d["events"].append(human.hand_profile("push", 30.0, 2.5, 3.0, None, hold_s=0.5))
        d["duration_s"] = 7.0
        d["metrics"]["windows"] = {}

    r, _, _ = runs.get(f"manual:{cls}", _catalog("power_loss", cls, edit=edit))
    f = r.kpis.get("breakaway_n")
    assert f is not None, f"{cls}: the landed carriage never moved under a 30 N push"
    if cls == "door":
        assert f <= ADA_N, (
            f"(S) ≤ {ADA_N} N, SPEC FAB-SAF-010: breakaway {f:.2f} N (relieved {relieved:.2f} N, bound {bound:.2f} N)"
        )


# ---------------------------------------------------------------------------------
# FAB-SAF-012 monitor vs injected core faults
# ---------------------------------------------------------------------------------


def _inject_edit(what: str, t_inj: float, rate: float):
    def edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
        for e in d["events"]:
            if e["kind"] == "core_fault":
                e["what"], e["t_s"] = what, t_inj
        d["record"]["rate_hz"] = rate
        chans = ["x_m", "v_mps", "f_lsm_n", "force_cap_n", "demand", "lev_state"]
        d["record"]["channels"] = chans + [f"gap_{i}_m" for i in range(int(p.get("design.suspension.n_magnets")))]
        d["metrics"]["windows"] = {}

    return edit


def _cf_tier(cls: str, what: str) -> str:
    if cls != "heavy":
        return "carriage6"
    return "heave1" if what == "gap_sign" else "travel1d"


@pytest.mark.req("FAB-SAF-012")
@pytest.mark.parametrize("cls", CLASSES)
def test_monitor_catches_core_fault(runs, cls):
    """SAF-012: the integer monitor trips on injected core faults; force/current checks within 1 fast period.

    Contract: (a) NaN core outputs: demand ≥ SafeLand within one fast period of the
    injection; (b) coil references sign-inverted (``gap_sign``): demand ≥ SafeLand no
    later than one fast period plus one sensor sample after the first truth gap leaves
    the monitor's envelope [safety.gap_min_m, safety.gap_max_m]; (c) the catalog
    ``core_fault_iq`` (3× LSM current): at every sample the measured LSM force stays
    within 1.1× the monitor's cap in force, or ForceLimit/STO follows within one fast
    period. HEAVY: (a)/(c) on Travel1d, (b) on Heave1.
    """
    rate = 20_000.0
    # (a) NaN
    fid = _cf_tier(cls, "nan")
    r, p, _ = runs.get(
        f"cf:nan:{cls}",
        _catalog("core_fault_iq", cls, fidelity=fid, edit=_inject_edit("nan", 1.0, rate if fid != "travel1d" else 1000.0)),
    )
    tp = _fast_period(p) + 1.0 / r.rate_hz
    t_trip = _first(r.t_s, (r.t_s >= 1.0) & (r["demand"] >= D_SAFELAND))
    assert t_trip is not None and t_trip - 1.0 <= tp, f"(T/S) NaN: trip at {t_trip}, limit {1.0 + tp:.5f} s, SPEC FAB-SAF-012"
    # (b) gap sign (hovering; HEAVY on Heave1)
    fid = _cf_tier(cls, "gap_sign")

    def gap_edit(d, series, p_):
        d["events"] = [{"kind": "core_fault", "t_s": 1.0, "what": "gap_sign"}]
        d["duration_s"] = 2.0
        _inject_edit("gap_sign", 1.0, rate)(d, series, p_)

    r, p, _ = runs.get(f"cf:gap:{cls}", _catalog("hover", cls, fidelity=fid, edit=gap_edit))
    nm = int(p.get("design.suspension.n_magnets"))
    gaps = np.array([r[f"gap_{i}_m"] for i in range(nm)])
    out = (gaps < p.get("safety.gap_min_m")) | (gaps > p.get("safety.gap_max_m"))
    t_out = _first(r.t_s, (r.t_s >= 1.0) & out.any(axis=0))
    t_trip = _first(r.t_s, (r.t_s >= 1.0) & (r["demand"] >= D_SAFELAND))
    assert t_trip is not None, f"(S) gap_sign never tripped the monitor (faults {_named(r)})"
    if t_out is not None:
        lim = t_out + _fast_period(p) + 2.0 / r.rate_hz
        assert t_trip <= lim, f"(S) SPEC FAB-SAF-012: gap left the envelope at {t_out:.5f} s, trip at {t_trip:.5f} s"
    # (c) 3× LSM current: force within 1.1× the cap in force, or a trip within one fast period
    fid = _cf_tier(cls, "iq_gain_x3")
    r, p, _ = runs.get(
        f"cf:iq:{cls}",
        _catalog("core_fault_iq", cls, fidelity=fid, edit=_inject_edit("iq_gain_x3", 1.0, rate if fid != "travel1d" else 1000.0)),
    )
    over = np.abs(r["f_lsm_n"]) > 1.1 * r["force_cap_n"] + 1e-6
    if over.any():
        t0 = float(r.t_s[int(np.argmax(over))])
        t_trip = _first(r.t_s, (r.t_s >= t0) & (r["demand"] >= D_SAFESTOP))
        assert t_trip is not None and t_trip - t0 <= _fast_period(p) + 2.0 / r.rate_hz, (
            f"(T/S) force above 1.1× cap at {t0:.5f} s, monitor trip at {t_trip}, SPEC FAB-SAF-012"
        )


# ---------------------------------------------------------------------------------
# FAB-SAF-018 payload mismatch
# ---------------------------------------------------------------------------------


def _mismatch_edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    go = next(e for e in d["events"] if e["kind"] == "command")
    x_t = float(go["x_m"])
    d["events"] += [
        {"kind": "hold_to_run", "t_s": 4.0, "on": True},
        {"kind": "command", "t_s": 4.1, "cmd": "go_to", "x_m": x_t},
    ]
    d["duration_s"] = 9.0
    d["metrics"]["windows"] = {}


@pytest.mark.req("FAB-SAF-018")
@pytest.mark.parametrize("cls", CLASSES)
def test_payload_mismatch(runs, cls):
    """SAF-018: m̂ > 1.05·m_rated raises PayloadMismatch (Degrade): levitation continues, motion hold-to-run only.

    Contract: catalog ``payload_mismatch`` (payload to 1.10·m_rated at 1 s): PayloadMismatch
    is raised within 1 s (m̂ convergence, FAB-LEV-013); the carriage stays levitated with
    no stopping fault; the automatic ``go_to`` at 3 s is rejected and the carriage does not
    move (< 1 mm); with hold-to-run actuated at 4 s the same ``go_to`` moves it. HEAVY on
    Heave1 has no x axis, so only the levitation part applies there (D1).
    """
    fid = "heave1" if cls == "heavy" else "carriage6"
    r, p, d = runs.get(f"mismatch:{cls}", _catalog("payload_mismatch", cls, fidelity=fid, edit=_mismatch_edit))
    t_pay = next(float(e["t_s"]) for e in d["events"] if e["kind"] == "payload")
    raised = [t for t, c in _faults(r) if c == F["PayloadMismatch"]]
    assert raised and raised[0] - t_pay <= 1.0, f"(S) SPEC FAB-SAF-018: PayloadMismatch at {raised} (faults {_named(r)})"
    assert not _stopping(r), f"(S) levitation continues (Degrade), SPEC FAB-SAF-018: stopping faults {_stopping(r)}"
    lev = np.isin(r["lev_state"][r.t_s >= t_pay], (LEV_LEVITATED_IDLE, LEV_MOVING))
    assert lev.all(), f"left levitation after the mismatch (states {sorted(set(r['lev_state'].tolist()))})"
    if cls == "heavy":
        return
    t = r.t_s
    x3 = r["x_m"][(t >= 3.0) & (t < 4.0)]
    assert _events(r, "Rejected"), "the automatic go_to was not rejected"
    assert np.abs(x3 - x3[0]).max() < 1e-3, (
        f"(S) hold-to-run only: moved {np.abs(x3 - x3[0]).max() * 1e3:.2f} mm on the automatic go_to"
    )
    x_end = float(r["x_m"][-1])
    assert abs(x_end - x3[0]) > 5e-3, f"hold-to-run go_to did not move the carriage (Δx {1e3 * (x_end - x3[0]):.2f} mm)"
