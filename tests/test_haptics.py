"""Haptics and motion feel in closed loop (FAB-HAP-001/003/005/006/007/008/009/012/013/016/018/019).

The real admittance, momentum observer, intent detector, passivity controller,
terminus planner and supervisor run against the f64 plant (``_native.run``). DRAWER,
DOOR and WALL run on Carriage6 in voltage coil mode; HEAVY runs on Travel1d where the
quantity is an x-axis haptic quantity (lead decision D1), and on Carriage6 for the
terminus criteria, which are asserted on Carriage6 devices only (SPEC FAB-HAP-013).

Latencies are measured from the instant the truth hand force crosses the class intent
threshold F_th (SPECIFICATION §2 "Intent" row; ``control.intent.f_th_n``): admittance
onset is the controller's hand flag (``obstruction`` channel bit 2, FastStatus.hand),
confirmation the first confirmed intent (``intent`` channel Open/Close).

Terminus criteria (plan §4b, PHYSICS §16.1) are checked on plant truth on a quiet floor
(no floor series), independently of the online KPIs, which are also asserted.

Research tier: inertial smoothing (FAB-HAP-018/019) is disabled in every preset by
design (``control.motion.inertial.enable = false``); the tests enable it explicitly.
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
#: ``motion_mode`` wire values (fabrail-types MotionMode).
MODE_TRAJECTORY, MODE_ADMITTANCE, MODE_TERMINUS, MODE_SOFT_OPEN, MODE_INERTIAL = 2, 3, 10, 11, 12
AUTO_MODES = (MODE_TRAJECTORY, MODE_TERMINUS, MODE_SOFT_OPEN, 4)  # 4 = SoftClose
#: ``intent`` wire values (fabrail-core msg.rs IntentFlag).
INTENT_OPEN, INTENT_CLOSE = 3, 4
#: ``lev_state`` wire values (fabrail-types LevState).
LEV_SEATED = 2
#: Controller hand flag in the ``obstruction`` channel (recorder: bit2 = FastStatus.hand).
OBS_DETECTED, OBS_HAND = 1, 4
#: FaultCode Obstruction (fabrail-types fault.rs).
FAULT_OBSTRUCTION = 0x0607
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
#: SPEC FAB-HAP-005 onset / confirmation limits, s.
ONSET_S = {"drawer": 0.010, "door": 0.010, "wall": 0.020, "heavy": 0.020}
CONFIRM_S = {"drawer": 0.120, "door": 0.120, "wall": 0.170, "heavy": 0.170}
#: SPEC FAB-HAP-003 breakaway limits, N (reported against ADA 404.2.9's 22.2 N).
BREAKAWAY_N = {"drawer": 1.0, "door": 5.0, "wall": 20.0}
ADA_N = 22.2
#: SPEC FAB-HAP-007 soft-close final speed, m/s.
SOFT_CLOSE_V = {"drawer": 0.020, "door": 0.050, "wall": 0.020, "heavy": 0.010}
#: SPEC FAB-HAP-016 terminus arrival bounds.
TERM_OVERSHOOT_M, TERM_V_MPS, TERM_T_REL = 0.02e-3, 1.0e-3, 0.02
#: Research-tier reason for FAB-HAP-018/019 (plan §4b; SPEC §2 "Inertial smoothing").
INERTIAL_REASON = (
    "research tier: control.motion.inertial.enable = false in every preset by design (plan §4b 🧪); "
    "SmoothRideThrough is admitted only by the house choreographer's ActionPolicy (fabrail-exo choreo), "
    "which _native.run does not consult, so the drawer never enters InertialSmooth here"
)


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


def _stopping(r: RunResult) -> list[str]:
    return [f"{t:.3f}s:0x{c:04X}" for t, c in _faults(r) if c in _STOPPING]


def _events(r: RunResult, kind: str) -> list[Any]:
    return [e for e in r.events if e.kind == kind]


def _first(t: np.ndarray, mask: np.ndarray) -> float | None:
    k = int(np.argmax(mask))
    return float(t[k]) if mask.size and mask[k] else None


def _hand_events(d: dict[str, Any]) -> list[dict[str, Any]]:
    return [e for e in d["events"] if e["kind"] == "hand"]


def _hold_to_run(d: dict[str, Any]) -> list[dict[str, Any]]:
    return [e for e in d["events"] if e["kind"] == "hold_to_run"]


#: Hand start, s: after the warm-start m̂ convergence (≤ 1 s, FAB-LEV-013), so the
#: admittance renders M_v for the converged mass estimate.
T_HAND_S = 1.5


def _levitated_pull(
    cls: str,
    *,
    x_frac: float = 0.3,
    f_ratio: float = 2.0,
    rise_s: float = 0.1,
    hold_s: float = 1.0,
    mass_kg: float | None = None,
    t0: float = T_HAND_S,
    rate_hz: float | None = None,
    dur_s: float | None = None,
):
    """Edit: levitated at x_closed + x_frac·travel (DOOR/WALL/HEAVY at the closed end),
    a grip pull of f_ratio·F_th toward +x (rise, hold, ramp down), optional payload."""

    def edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
        x0, travel = p.get("design.rail.x_closed_m"), p.get("design.rail.travel_m")
        x = x0 + (x_frac * travel if cls == "drawer" else 0.0)
        init: dict[str, Any] = {"lev": "levitated", "x_m": x}
        if mass_kg is not None:
            init["payload_kg"] = mass_kg - p.get("design.carriage.m_star_kg")
        d["initial"] = init
        hand = human.hand_profile("pull", f_ratio * p.get("control.intent.f_th_n"), t0, rise_s, None, hold_s=hold_s)
        d["events"] = _hold_to_run(d) + [hand]
        if dur_s is not None:
            d["duration_s"] = dur_s
        if rate_hz is not None:
            d["record"]["rate_hz"] = rate_hz
        d["metrics"]["windows"] = {}

    return edit


def _push_open_run(runs: _Runs, cls: str, mass_kg: float | None = None, rate_hz: float | None = None):
    """Hand pull on a levitated carriage (DRAWER at 30 % travel; others at the closed end)."""
    rate = rate_hz if rate_hz is not None else (20_000.0 if cls != "heavy" else 1000.0)
    key = f"pull:{cls}:{mass_kg}:{rate}"
    dur = {"drawer": 4.0, "door": 5.0, "wall": 6.0, "heavy": 6.0}[cls]
    return runs.get(
        key, _catalog("push_open", cls, fidelity=_tier(cls), edit=_levitated_pull(cls, mass_kg=mass_kg, rate_hz=rate, dur_s=dur))
    )


# ---------------------------------------------------------------------------------
# FAB-HAP-001 apparent mass / FAB-HAP-003 breakaway
# ---------------------------------------------------------------------------------

_AM_CASES = [("drawer", 8.0), ("drawer", 26.0), ("drawer", 48.0), ("door", None), ("wall", None), ("heavy", None)]


@pytest.mark.req("FAB-HAP-001")
@pytest.mark.parametrize(("cls", "mass_kg"), _AM_CASES)
def test_apparent_mass(runs, cls, mass_kg):
    """HAP-001: the rendered apparent mass is within ±10 % of M_v across the class mass range.

    Contract: a grip pull of 2·F_th (0.1 s rise, 1 s hold, from 1.5 s so m̂ has
    converged) on a levitated carriage in Admittance; ``apparent_mass_err`` = |M − M_v|/M_v from the least-squares fit
    F_hand = M·a + B·v over the hand contact (fabrail-sim metrics) ≤ 0.10. DRAWER at 8 /
    26 / 48 kg (the SPEC's range); HEAVY on Travel1d (D1).
    """
    r, _, _ = _push_open_run(runs, cls, mass_kg, rate_hz=1000.0)
    err = r.kpis.get("apparent_mass_err")
    assert err is not None, f"{cls}: no admittance hand contact was measured (faults {_faults(r)})"
    assert err <= 0.10, f"(S) ±10 %, SPEC FAB-HAP-001: {cls} {mass_kg or ''} apparent-mass error {100 * err:.1f} %"


@pytest.mark.req("FAB-HAP-003")
@pytest.mark.parametrize("cls", ["drawer", "door", "wall"])
def test_breakaway_force(runs, cls):
    """HAP-003: breakaway ≤ 1 / 5 / 20 N (DRAWER / DOOR / WALL), reported against 22.2 N.

    Contract: ``breakaway_n`` (peak hand force before the carriage first exceeds 1 mm/s,
    fabrail-sim metrics) under the 2·F_th grip pull (0.1 s rise) on a levitated carriage
    is at most the class limit; the ADA 404.2.9 22.2 N reference is stated in the message.
    """
    r, _, _ = _push_open_run(runs, cls, rate_hz=1000.0)
    f = r.kpis.get("breakaway_n")
    assert f is not None, f"{cls}: the carriage never moved under the hand"
    assert f <= BREAKAWAY_N[cls], (
        f"(S) ≤ {BREAKAWAY_N[cls]} N, SPEC FAB-HAP-003: {cls} breakaway {f:.2f} N (ADA 404.2.9 reference {ADA_N} N)"
    )


# ---------------------------------------------------------------------------------
# FAB-HAP-005 intent latency
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-HAP-005")
@pytest.mark.parametrize("cls", CLASSES)
def test_intent_latency(runs, cls):
    """HAP-005: admittance onset ≤ 10 / 10 / 20 / 20 ms, intent confirmation ≤ 120 / 170 ms.

    Contract: from the truth hand force crossing F_th (§2 thresholds) under a 2·F_th
    grip pull (0.1 s rise), the controller registers the hand (onset: the first pending
    or confirmed intent flag or FastStatus.hand, whichever comes first; an idle levitated
    carriage already renders admittance, so onset is the hand's detection) within the
    onset limit, and a confirmed intent (Open) follows within the confirmation limit.
    Recorded at 20 kHz on Carriage6 (1 kHz on Travel1d HEAVY).
    """
    r, p, d = _push_open_run(runs, cls)
    t = r.t_s
    f_th = p.get("control.intent.f_th_n")
    t_hand = float(_hand_events(d)[0]["t_s"])
    after = t >= t_hand
    t_cross = _first(t, after & (np.abs(r["f_hand_n"]) >= f_th))
    assert t_cross is not None, f"{cls}: the hand force never reached F_th {f_th} N"
    detected = ((r["obstruction"].astype(np.int64) & OBS_HAND) != 0) | (r["intent"] > 0)
    t_on = _first(t, after & detected)
    t_conf = _first(t, after & np.isin(r["intent"], (INTENT_OPEN, INTENT_CLOSE)))
    assert t_on is not None, f"{cls}: the controller never registered the hand"
    onset = max(0.0, t_on - t_cross)
    assert onset <= ONSET_S[cls], (
        f"(S) ≤ {ONSET_S[cls] * 1e3:.0f} ms, SPEC FAB-HAP-005: {cls} admittance onset {onset * 1e3:.1f} ms"
    )
    assert t_conf is not None, f"{cls}: intent never confirmed under a 2·F_th pull"
    conf = t_conf - t_cross
    assert conf <= CONFIRM_S[cls], (
        f"(S) ≤ {CONFIRM_S[cls] * 1e3:.0f} ms, SPEC FAB-HAP-005: {cls} intent confirmation {conf * 1e3:.1f} ms"
    )


# ---------------------------------------------------------------------------------
# FAB-HAP-006 false intent
# ---------------------------------------------------------------------------------

#: SPEC FAB-HAP-006 smoke ΔP force on the leaf, N.
SMOKE_N = 50.0


def _smoke_edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    area = p.get("design.air.leaf_area_m2")
    for e in d["events"]:
        if e["kind"] == "smoke_dp":
            e["dp_pa"] = SMOKE_N / area
    d["doc"] = f"{d['doc']} [B4: ΔP set for {SMOKE_N} N on the leaf]"


def _footsteps_edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    # 15 s at 0.4 episodes/s (≈ 6 walking episodes, as the catalog's 30 s at 0.2/s).
    d["duration_s"] = 15.0
    d["metrics"]["windows"] = {}


_FALSE_CASES = [
    ("drawer", "piston_pulse"),
    ("drawer", "lean"),
    ("drawer", "footsteps"),
    ("door", "lean"),
    ("door", "smoke_dp"),
    ("door", "footsteps"),
    ("wall", "lean"),
    ("wall", "smoke_dp"),
    ("wall", "footsteps"),
    ("heavy", "lean"),
    ("heavy", "footsteps"),
]


@pytest.mark.req("FAB-HAP-006")
@pytest.mark.parametrize(("cls", "name"), _FALSE_CASES)
def test_false_intent_suite(runs, cls, name):
    """HAP-006: no false intent from the cabinet piston, a lean, a smoke ΔP or footsteps.

    Contract: under the catalog disturbance (DRAWER piston 5.6 N / 50 ms outward; 15 N
    lean rising over 1 s, held 5 s; smoke ΔP ramped over 2 s to 50 N on the leaf;
    15 s of footsteps at 0.4 episodes/s) no intent is confirmed (no Intent event, the intent channel never
    Open/Close) and no automatic motion starts (no Trajectory/Terminus/SoftOpen/SoftClose
    mode); a seated DRAWER never unseats. HEAVY on Travel1d (D1).
    """
    edit = _smoke_edit if name == "smoke_dp" else (_footsteps_edit if name == "footsteps" else None)
    opts = {"episode_rate_per_s": 0.4} if name == "footsteps" else None
    r, _, _ = runs.get(f"false:{cls}:{name}", _catalog(name, cls, fidelity=_tier(cls), options=opts, edit=edit))
    intents = _events(r, "Intent")
    conf = np.isin(r["intent"], (INTENT_OPEN, INTENT_CLOSE))
    auto = np.isin(r["motion_mode"], AUTO_MODES)
    dx = float(np.abs(r["x_m"] - r["x_m"][0]).max())
    msg = f"{cls} {name}: intent events {[(round(e.t_s, 3), e.b) for e in intents][:3]}, max |Δx| {dx * 1e3:.2f} mm"
    assert not intents and not conf.any(), f"(S) SPEC FAB-HAP-006: false intent — {msg}"
    assert not auto.any(), f"(S) SPEC FAB-HAP-006: automatic motion started at {r.t_s[int(np.argmax(auto))]:.3f} s — {msg}"
    if cls == "drawer":
        assert not _events(r, "Unseated"), f"seated DRAWER unseated under {name}"


# ---------------------------------------------------------------------------------
# FAB-HAP-007 soft close
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-HAP-007")
@pytest.mark.parametrize("cls", CLASSES)
def test_soft_close(runs, cls):
    """HAP-007: soft-close ends at ≤ 20 / 50 / 20 / 10 mm/s, overshoot ≤ 1 mm, no bumper contact.

    Contract: DRAWER/DOOR/WALL are thrown closed by hand (catalog ``throw_soft_close``:
    grip, 3·F_th, released at speed); HEAVY (hold-to-run, never thrown) closes onto its
    terminus on Travel1d. The speed on the last approach within 0.5 mm of the closed
    stop (``soft_close_v_final``) is within the class bound, the carriage travels at
    most 1 mm past the closed stop on the approach (to 0.3 s after first reaching it)
    and past any motion segment's end (``overshoot_m``), no bumper impact, no stopping
    fault.
    """
    name = "terminus_soft_close" if cls == "heavy" else "throw_soft_close"
    r, p, _ = runs.get(f"soft_close:{cls}", _catalog(name, cls, fidelity=_tier(cls)))
    assert not _stopping(r), f"{cls} {name} interrupted: {_stopping(r)}"
    v = r.kpis.get("soft_close_v_final")
    obs = [round(t, 3) for t, c in _faults(r) if c == FAULT_OBSTRUCTION]
    assert v is not None, f"{cls}: never reached the closed stop (x end {r['x_m'][-1] * 1e3:.1f} mm; obstruction at {obs})"
    assert v <= SOFT_CLOSE_V[cls], (
        f"(S) ≤ {SOFT_CLOSE_V[cls] * 1e3:.0f} mm/s, SPEC FAB-HAP-007: {cls} final speed {v * 1e3:.2f} mm/s"
    )
    x0 = p.get("design.rail.x_closed_m")
    k_near = int(np.argmax(r["x_m"] - x0 <= 0.5e-3))
    k_end = int(np.searchsorted(r.t_s, r.t_s[k_near] + 0.3))
    past = float(x0 - r["x_m"][:k_end].min())
    assert past <= 1e-3 and r.kpis.get("overshoot_m", 0.0) <= 1e-3, (
        f"(S) overshoot ≤ 1 mm, SPEC FAB-HAP-007: {cls} {past * 1e3:.3f} mm past the closed stop on the approach, "
        f"segment overshoot {r.kpis.get('overshoot_m', 0.0) * 1e3:.3f} mm"
    )
    assert r.kpis["bumper_impacts"] == 0.0, f"{cls}: bumper contact ({r.kpis['bumper_impacts']:.0f} impacts)"


# ---------------------------------------------------------------------------------
# FAB-HAP-008 detents
# ---------------------------------------------------------------------------------


#: Intent confirmation pushed to its parameter maxima for the detent drag, so no
#: intent-triggered automatic move takes the carriage out of Admittance (the catalog
#: ``detent_pass`` crosses the detent in Trajectory mode, measuring the move, not the
#: rendering).
_NO_INTENT = {"control.intent.t_th_s": 5.0, "control.intent.dx_th_m": 0.1}


def _detent_edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    # Start 8 mm before the detent zone and pull at 1.5·F_th: a steady supra-threshold
    # drag reaches the admittance speed limiter in every class (F_th/B_v > v_cap), so
    # the detent is crossed while the carriage is still accelerating, below the cap.
    xd, w = p.get("control.haptic.detent_x_m.0"), p.get("control.haptic.detent_width_m")
    d["initial"] = {"lev": "levitated", "x_m": xd - w - 8e-3}
    hand = human.hand_profile("pull", 1.5 * p.get("control.intent.f_th_n"), 0.5, 0.1, None, hold_s=0.6)
    d["events"] = [hand]
    d["duration_s"] = 2.5
    d["metrics"]["windows"] = {}


@pytest.mark.req("FAB-HAP-008")
@pytest.mark.parametrize("cls", ["drawer", "door", "wall"])
def test_detent_peak(runs, cls):
    """HAP-008: detent peak force within ±10 % of the profile (3 N / 2 mm, 8 N / 3 mm, 20 N / 5 mm).

    Contract: a 1.5·F_th grip drag across the first detent from 8 mm before its zone,
    intent confirmation suppressed so the carriage stays in Admittance and crosses below
    the speed cap; ``detent_peak_err`` = |peak(F_hand − M_v·a − B_v·v) − F_detent|/F_detent
    inside the detent zone (fabrail-sim metrics) ≤ 0.10. The profile parameters equal
    SPEC FAB-HAP-008 ((E)).
    """
    r, p, _ = runs.get(f"detent:{cls}", _catalog("detent_pass", cls, overrides=_NO_INTENT, edit=_detent_edit))
    f_ref, w_ref = {"drawer": (3.0, 2e-3), "door": (8.0, 3e-3), "wall": (20.0, 5e-3)}[cls]
    assert p.get("control.haptic.detent_f_n") == f_ref, "(E) detent force, SPEC FAB-HAP-008"
    assert abs(p.get("control.haptic.detent_width_m") - w_ref) <= 1e-12, "(E) detent width, SPEC FAB-HAP-008"
    xd = p.get("control.haptic.detent_x_m.0")
    assert r["x_m"].max() >= xd + w_ref, f"{cls}: the drag never crossed the detent at {xd} m (x max {r['x_m'].max():.4f})"
    assert not _events(r, "Intent"), "the drag confirmed intent; the detent measurement is not isolated"
    inside = np.abs(r["x_m"] - xd) < w_ref
    assert np.all(r["motion_mode"][inside] == MODE_ADMITTANCE), "the detent was not crossed in Admittance"
    assert np.abs(r["v_mps"][inside]).max() < p.derived()["v_open_cap_mps"], "the detent was crossed at the speed cap"
    err = r.kpis.get("detent_peak_err")
    assert err is not None, f"{cls}: no detent-zone hand samples"
    assert err <= 0.10, f"(S) ±10 %, SPEC FAB-HAP-008: {cls} detent peak error {100 * err:.1f} % of {f_ref} N"


# ---------------------------------------------------------------------------------
# FAB-HAP-009 passivity
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-HAP-009")
@pytest.mark.parametrize("cls", ["drawer", "door", "wall"])
def test_passivity_observer(native, runs, cls):
    """HAP-009: K ≤ K_max = 2b/(T + 2T_d); with the passivity controller on, E_obs ≥ −1 mJ.

    Contract: ``passivity_k_max(b_dist, T_fast, T_d = T_fast)`` equals 2b/(T + 2T_d)
    ((E) rel 1e-12); the preset k_virtual_frac < 1 and validate rejects
    k_virtual_frac = 1 (K = K_max is not admitted), so no loadable parameter set renders
    3·K_max — the violation at 3·K_max is reproduced by the unit test
    ``rs:core/haptics/passivity::pc_bounds_energy``. In closed loop with the passivity
    controller on, the minimum passivity-observer energy over a hand pull, a detent drag
    and a throw is ≥ −1 mJ.
    """
    cfg = RunConfig(rail_class=cls, coil="voltage")
    p = cfg.params()
    b = p.get("design.damping.b_dist_n_s_per_m")
    t_fast = p.get("control.rates.fast_decim") / p.get("control.rates.base_hz")
    k_max = native.passivity_k_max(b, t_fast, t_fast)
    assert abs(k_max / (2.0 * b / (3.0 * t_fast)) - 1.0) <= 1e-12, f"(E) 2b/(T + 2T_d): {k_max}"
    assert p.get("control.haptic.k_virtual_frac") < 1.0, "preset virtual stiffness at or above K_max"
    with pytest.raises(ValueError, match="k_virtual_frac"):
        p.with_overrides({"control.haptic.k_virtual_frac": 1.0})
    worst = []
    for key, factory in (
        (f"pull:{cls}:None:1000.0", None),
        (f"detent:{cls}", _catalog("detent_pass", cls, overrides=_NO_INTENT, edit=_detent_edit)),
        (f"soft_close:{cls}", _catalog("throw_soft_close", cls)),
    ):
        if factory is None:
            r, _, _ = _push_open_run(runs, cls, rate_hz=1000.0)
        else:
            r, _, _ = runs.get(key, factory)
        worst.append((key, r.kpis["passivity_min_j"]))
    bad = [(k, e) for k, e in worst if e < -1e-3]
    assert not bad, f"(S) E_obs ≥ −1 mJ, SPEC FAB-HAP-009: {[(k, round(e * 1e3, 3)) for k, e in bad]} mJ"


# ---------------------------------------------------------------------------------
# FAB-HAP-012 DRAWER push-to-open
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-HAP-012")
def test_drawer_push_to_open(runs):
    """HAP-012: DRAWER push-to-open from the seat: detect ≤ 60 ms, unseat ≤ 100 ms, first motion ≤ 160 ms.

    Contract: catalog ``push_open`` (seated DRAWER, push-release of 2·F_th inward using
    the seat's x push-stop compliance); from the truth push crossing F_th: an opening
    intent is confirmed within 60 ms, the drawer is out of the seat (Unseated) within
    100 ms and moves outward (x > x_closed + 1 mm) within 160 ms.
    """

    def edit(d, series, p):
        d["record"]["rate_hz"] = 20_000.0

    r, p, d = runs.get("push_to_open", _catalog("push_open", "drawer", edit=edit))
    t = r.t_s
    f_th = p.get("control.intent.f_th_n")
    t_hand = float(_hand_events(d)[0]["t_s"])
    t_cross = _first(t, (t >= t_hand) & (np.abs(r["f_hand_n"]) >= f_th))
    assert t_cross is not None, "the push never reached F_th"
    t_det = _first(t, (t >= t_hand) & (r["intent"] == INTENT_OPEN))
    unseat = [e.t_s for e in _events(r, "Unseated")]
    t_move = _first(t, r["x_m"] > p.get("design.rail.x_closed_m") + 1e-3)
    dx = float(r["x_m"].max() - r["x_m"].min())
    assert t_det is not None and t_det - t_cross <= 0.060, (
        f"(S) detect ≤ 60 ms, SPEC FAB-HAP-012: {None if t_det is None else round((t_det - t_cross) * 1e3, 1)} ms "
        f"(seat compliance travel {dx * 1e3:.3f} mm, unseat {unseat}, faults {_faults(r)})"
    )
    assert unseat and unseat[0] - t_cross <= 0.100, f"(S) unseat ≤ 100 ms, SPEC FAB-HAP-012: {unseat}"
    assert t_move is not None and t_move - t_cross <= 0.160, f"(S) first motion ≤ 160 ms, SPEC FAB-HAP-012: {t_move}"


# ---------------------------------------------------------------------------------
# FAB-HAP-013 / 016 termini (Carriage6, quiet floor)
# ---------------------------------------------------------------------------------


def _open_near_end(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    # WALL/HEAVY: the catalog soft_open is a go_to (plain Trajectory); an open onto the
    # open-end terminus from 0.5 m before it exercises the SoftOpen terminus.
    x0, travel = p.get("design.rail.x_closed_m"), p.get("design.rail.travel_m")
    d["initial"] = {"lev": "levitated", "x_m": x0 + travel - 0.5}
    d["events"] = _hold_to_run(d) + [{"kind": "command", "t_s": 0.5, "cmd": "open"}]
    der = p.derived()
    v, a = der["v_open_cap_mps"], p.get("control.motion.a_max_mps2")
    d["duration_s"] = math.ceil(0.5 + 0.5 / v + v / a + der["terminus_t_arrive_s"] + 2.5)
    d["metrics"]["windows"] = {}
    d["doc"] = f"{d['doc']} [B4: open from 0.5 m before the open end]"


def _terminus_run(runs: _Runs, cls: str, direction: str):
    if direction == "close":
        return runs.get(f"term_close:{cls}", _catalog("terminus_soft_close", cls, fidelity="carriage6"))
    edit = _open_near_end if cls in ("wall", "heavy") else None
    return runs.get(f"term_open:{cls}", _catalog("soft_open", cls, fidelity="carriage6", edit=edit))


def _terminus_target(p: Any, d: dict[str, Any], direction: str) -> float:
    x0 = p.get("design.rail.x_closed_m")
    return x0 if direction == "close" else x0 + p.get("design.rail.travel_m")


_TERM_CASES = [(c, dr) for c in CLASSES for dr in ("close", "open")]


def _capture(r: RunResult, p: Any, d: dict[str, Any], direction: str):
    """(capture start, d_ε crossing index window, distance-to-go) from the TerminusCapture event."""
    cap = _events(r, "TerminusCapture")
    assert cap, f"no TerminusCapture event (faults {_faults(r)})"
    target = _terminus_target(p, d, direction)
    dist = np.abs(r["x_m"] - target)
    d_eps = p.get("control.motion.terminus.d_eps_m")
    k0 = int(np.searchsorted(r.t_s, cap[-1].t_s))
    below = np.flatnonzero(dist[k0:] <= d_eps)
    assert below.size, f"distance-to-go never fell below d_ε {d_eps * 1e3} mm (end {dist[-1] * 1e3:.3f} mm)"
    return k0, k0 + int(below[0]), dist, target, cap[-1]


@pytest.mark.req("FAB-HAP-013")
@pytest.mark.parametrize(("cls", "direction"), _TERM_CASES)
def test_terminus_semilog_fit(runs, cls, direction):
    """HAP-013: during capture ln(d) is linear in t (R² ≥ 0.999) on a quiet floor.

    Contract (Carriage6, no floor series, so |a_floor,x| < 0.01 g): the truth
    distance-to-go from the TerminusCapture event to its first crossing of d_ε, fitted
    as ln d = α − t/τ, has R² ≥ 0.999 and τ_fit within ±10 % (P) of the event's τ_eff;
    the online KPI ``terminus_r2_min`` is also ≥ 0.999. HEAVY Carriage6 is the D1 gap.
    """
    r, p, d = _terminus_run(runs, cls, direction)
    assert not _stopping(r), f"{cls} {direction}: stopping faults {_stopping(r)} (D1 for HEAVY Carriage6)"
    k0, k1, dist, _, cap = _capture(r, p, d, direction)
    tt, yy = r.t_s[k0:k1], np.log(dist[k0:k1])
    slope, icpt = np.polyfit(tt, yy, 1)
    r2 = 1.0 - float(np.sum((yy - (slope * tt + icpt)) ** 2) / np.sum((yy - yy.mean()) ** 2))
    tau_fit, tau = -1.0 / slope, float(cap.b)
    assert r2 >= 0.999, f"(S) R² ≥ 0.999, SPEC FAB-HAP-013: {cls} {direction} semilog R² {r2:.5f} over {tt[-1] - tt[0]:.3f} s"
    assert abs(tau_fit / tau - 1.0) <= 0.10, f"(P) ±10 %: {cls} {direction} τ_fit {tau_fit:.4f} s vs τ_eff {tau:.4f} s"
    kpi = r.kpis.get("terminus_r2_min")
    assert kpi is not None and kpi >= 0.999, f"(S) online terminus_r2_min {kpi}"


@pytest.mark.req("FAB-HAP-016")
@pytest.mark.parametrize(("cls", "direction"), _TERM_CASES)
def test_terminus_arrival(native, runs, cls, direction):
    """HAP-016: overshoot ≤ 0.02 mm, arrival speed ≤ 1 mm/s on truth, t_arrive ±2 %.

    Contract (Carriage6, quiet floor): from the capture to arrival (Arrived + 50 ms)
    x − x_term never crosses zero by more than 0.02 mm; the truth speed where the
    distance-to-go first reaches 0.02 mm is ≤ 1 mm/s; the online KPIs agree
    (``terminus_overshoot_max_m``, ``terminus_v_arrive_max_mps``) and the arrival time
    error against the closed form of the run's own segment, measured at the
    reference's own 0.02 mm crossing (``terminus_t_arrive_err_max``, W1.5 convention for
    the quintic tail; the planner approaches below the cap, so the SPEC's cap-speed
    table values are not the run's closed form), is ≤ 2 %. DRAWER closes seat within
    seat_time_s (+0.2 s) of arrival.
    """
    r, p, d = _terminus_run(runs, cls, direction)
    assert not _stopping(r), f"{cls} {direction}: stopping faults {_stopping(r)} (D1 for HEAVY Carriage6)"
    k0, _, dist, target, _ = _capture(r, p, d, direction)
    arrived = [e.t_s for e in _events(r, "Arrived") if e.t_s >= r.t_s[k0]]
    assert arrived, f"{cls} {direction}: no Arrived event after the capture"
    k_end = int(np.searchsorted(r.t_s, arrived[0] + 0.05))
    sign = -1.0 if direction == "close" else 1.0
    past = float(np.max((r["x_m"][k0:k_end] - target) * sign))
    assert past <= TERM_OVERSHOOT_M, f"(S) ≤ 0.02 mm, SPEC FAB-HAP-016: {cls} {direction} overshoot {past * 1e3:.4f} mm"
    k_arr = k0 + int(np.argmax(dist[k0:] <= 0.02e-3))
    assert dist[k_arr] <= 0.02e-3, f"{cls} {direction}: never within 0.02 mm of the target"
    v_arr = float(abs(r["v_mps"][k_arr]))
    assert v_arr <= TERM_V_MPS, f"(S) ≤ 1 mm/s on truth, SPEC FAB-HAP-016: {cls} {direction} arrival {v_arr * 1e3:.3f} mm/s"
    k = r.kpis
    assert k["terminus_overshoot_max_m"] <= TERM_OVERSHOOT_M and k["terminus_v_arrive_max_mps"] <= TERM_V_MPS, (
        f"online KPIs: overshoot {k['terminus_overshoot_max_m']}, v {k['terminus_v_arrive_max_mps']}"
    )
    assert k["terminus_t_arrive_err_max"] <= TERM_T_REL, (
        f"(S) ±2 %, SPEC FAB-HAP-016: t_arrive error {100 * k['terminus_t_arrive_err_max']:.2f} %"
    )
    if cls == "drawer" and direction == "close":
        arrived, seated = _events(r, "Arrived"), _events(r, "Seated")
        seated_after = [e.t_s for e in seated if arrived and e.t_s >= arrived[-1].t_s]
        assert seated_after, "DRAWER did not seat after the close terminus"
        dt = seated_after[0] - arrived[-1].t_s
        assert dt <= 1.02 * p.get("control.levitation.seat_time_s"), f"(S) +0.2 s seat step: {dt * 1e3:.1f} ms"


def _replan_edit(f_ratio_obstruction: float | None):
    """Edit the catalog ``terminus_hand_replan``: a grip that opposes the close during capture.

    ``f_ratio_obstruction`` None: 0.9·th0 (a hand above F_th, below the obstruction
    residual th0); otherwise that multiple of th0 (held back hard).
    """

    def edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
        th0 = p.get("control.haptic.obstruction.residual_n")
        f = 0.9 * th0 if f_ratio_obstruction is None else f_ratio_obstruction * th0
        t_hand = float(scenarios.info(d)["t_hand_s"])
        hand = human.hand_profile("pull", f, t_hand, 0.03, None, hold_s=0.3)
        d["events"] = [e for e in d["events"] if e["kind"] != "hand"] + [hand]
        d["record"]["rate_hz"] = 20_000.0
        d["record"]["channels"] = ["x_m", "v_mps", "f_hand_n", "motion_mode", "obstruction", "intent", "lev_state"]

    return edit


@pytest.mark.req("FAB-HAP-016")
@pytest.mark.parametrize("cls", ["drawer", "door", "wall"])
def test_terminus_hand_replan(runs, cls):
    """HAP-016: a hand during capture → class admittance; release re-plans; obstruction overrides.

    Contract (Carriage6, catalog ``terminus_hand_replan`` with the hand raised to 0.9·th0,
    above F_th and below the obstruction residual; the catalog's 0.5·F_th is below the
    hand threshold): within the class onset limit
    of the hand force crossing F_th the carriage is in Admittance; after the release a new
    terminus capture starts and the carriage arrives closed (within the class closed
    tolerance, |v| < 1 mm/s at the end) with no obstruction. A second run holds the
    capture back at 1.5× the obstruction residual th0: on release an Obstruction is
    latched and the carriage does not re-close (it ends > 5 mm from the stop).
    """
    r, p, d = runs.get(f"replan:{cls}", _catalog("terminus_hand_replan", cls, edit=_replan_edit(None)))
    t = r.t_s
    f_th = p.get("control.intent.f_th_n")
    hand = _hand_events(d)[0]
    t_hand = float(hand["t_s"])
    t_rel = t_hand + float(hand["t_pts_s"][-1])
    t_cross = _first(t, (t >= t_hand) & (np.abs(r["f_hand_n"]) >= f_th))
    assert t_cross is not None, "hand never reached F_th"
    t_adm = _first(t, (t >= t_hand) & (r["motion_mode"] == MODE_ADMITTANCE))
    assert t_adm is not None and t_adm - t_cross <= ONSET_S[cls], (
        f"(S) ≤ {ONSET_S[cls] * 1e3:.0f} ms, SPEC FAB-HAP-016: admittance at {t_adm} vs hand at {t_cross:.4f} s"
    )
    captures = [e.t_s for e in _events(r, "TerminusCapture") if e.t_s > t_rel]
    obs = [round(tt, 3) for tt, c in _faults(r) if c == FAULT_OBSTRUCTION]
    assert captures, f"no re-planned terminus capture after the release at {t_rel:.3f} s (obstruction {obs})"
    assert not obs, f"a hand below th0 must not latch an obstruction: {obs}"
    x0 = p.get("design.rail.x_closed_m")
    tail = t >= t[-1] - 0.2
    err = float(r["x_m"][tail].mean() - x0)
    assert abs(err) <= p.get("control.motion.pos_tol_closed_m"), f"(S) re-planned close ends {err * 1e3:+.3f} mm from the stop"
    assert np.abs(r["v_mps"][tail]).max() < 1e-3, "still moving at the end"

    r2, p2, d2 = runs.get(f"replan_obs:{cls}", _catalog("terminus_hand_replan", cls, edit=_replan_edit(1.5)))
    obs2 = [tt for tt, c in _faults(r2) if c == FAULT_OBSTRUCTION]
    assert obs2, (
        f"(S) SPEC FAB-HAP-016: holding at 1.5·th0 then releasing latched no obstruction (x end {r2['x_m'][-1] * 1e3:.2f} mm)"
    )
    assert r2["x_m"][-1] - x0 > 5e-3, f"(S) obstruction overrides the target: re-closed to {(r2['x_m'][-1] - x0) * 1e3:.2f} mm"


# ---------------------------------------------------------------------------------
# FAB-HAP-018 / 019 inertial smoothing (research tier)
# ---------------------------------------------------------------------------------

_INERTIAL_ON = {"control.motion.inertial.enable": True}
#: Villa event with the P arrival at 8 s instead of the catalog's 35 s (the 30 s STA/LTA
#: warm-up is not needed: the synthetic network alert drives the action), for runtime.
_VILLA_SHORT = {"t_p_s": 8.0}


def _inertial_window(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    """Cut the villa record to the S arrival + 12 s (alert, P, S onset and the exit)."""
    d["duration_s"] = math.ceil(float(scenarios.info(d)["t_s_s"]) + 12.0)
    d["metrics"]["windows"] = {}


def _t_sky(r_: float, zeta: float) -> float:
    return 1.0 / math.sqrt((1.0 - r_ * r_) ** 2 + (2.0 * zeta * r_) ** 2)


@pytest.mark.req("FAB-HAP-018")
def test_inertial_residual_transmissibility(runs):
    """HAP-018 (R): with smoothing on, the floor-to-drawer x transmissibility follows T_sky ±20 %.

    Contract: an open DRAWER (inertial.enable = true, research tier) under the catalog
    ``inertial_smooth_fixedbase`` record (fixed-base building, T1 0.25 s); while in
    InertialSmooth the cross-spectral |H(f)| = |S_fa/S_ff| from floor x acceleration to
    the drawer's absolute x acceleration lies within ±20 % of
    T_sky(f/f_c, ζ) = 1/√((1 − r²)² + (2ζr)²) at 2 Hz and 5 Hz. If the drawer never
    enters InertialSmooth under ``_native.run`` the check is an expected failure with the
    research-tier reason (see module docstring).
    """
    r, p, _ = runs.get(
        "inertial:fixedbase",
        _catalog("inertial_smooth_fixedbase", "drawer", options=_VILLA_SHORT, overrides=_INERTIAL_ON, edit=_inertial_window),
    )
    mode = r["motion_mode"] == MODE_INERTIAL
    if mode.sum() < int(5.0 * r.rate_hz):
        pytest.xfail(f"{INERTIAL_REASON} (InertialSmooth for {mode.sum() / r.rate_hz:.2f} s)")
    from scipy import signal

    fs = r.rate_hz
    af = r["floor_ax_mps2"][mode]
    aa = r["a_mps2"][mode] + af
    nper = int(4 * fs)
    f, pff = signal.csd(af, af, fs=fs, nperseg=nper)
    _, pfa = signal.csd(af, aa, fs=fs, nperseg=nper)
    h = np.abs(pfa / pff)
    fc, zeta = p.get("control.motion.inertial.f_c_hz"), p.get("control.motion.inertial.zeta")
    for f0 in (2.0, 5.0):
        k = int(np.argmin(np.abs(f - f0)))
        ref = _t_sky(f0 / fc, zeta)
        assert abs(h[k] / ref - 1.0) <= 0.20, f"(R) ±20 %, SPEC FAB-HAP-018: |H({f0} Hz)| {h[k]:.4f} vs T_sky {ref:.4f}"


@pytest.mark.req("FAB-HAP-019")
def test_inertial_exit_to_seat(runs):
    """HAP-019 (R): seated drawers never unseat on an alert; every smoothing exit is a log-damped close to the seat.

    Contract: with inertial.enable = true, a seated DRAWER under the villa's synthetic
    warned event (M7.2, R_epi 60 km, alert 7.5 s after the origin) never unseats (no Unseated event,
    lev_state Seated throughout). An open DRAWER that enters InertialSmooth and then loses
    mains exits through a Terminus close that ends Seated, with no monitor demand. If the
    open drawer never enters InertialSmooth under ``_native.run`` the second part is an
    expected failure with the research-tier reason.
    """

    def seated(d, series, p):
        _inertial_window(d, series, p)
        d["initial"] = {"lev": "seated", "x_m": p.get("design.rail.x_closed_m")}

    r, _, _ = runs.get(
        "inertial:seated",
        _catalog("inertial_smooth_fixedbase", "drawer", options=_VILLA_SHORT, overrides=_INERTIAL_ON, edit=seated),
    )
    assert not _events(r, "Unseated"), (
        f"(T) SPEC FAB-HAP-019: a seated drawer unseated on an alert at {[e.t_s for e in _events(r, 'Unseated')]}"
    )
    k_seat = int(np.argmax(r["lev_state"] == LEV_SEATED))
    assert r["lev_state"][k_seat] == LEV_SEATED and np.all(r["lev_state"][k_seat:] == LEV_SEATED), (
        f"seated drawer left Seated (states after boot {sorted(set(r['lev_state'][k_seat:].tolist()))})"
    )

    def exit_on_mains_loss(d, series, p):
        _inertial_window(d, series, p)
        t_s = float(scenarios.info(d)["t_s_s"])
        d["events"].append({"kind": "power_loss", "t_s": t_s + 5.0})

    r2, _, _ = runs.get(
        "inertial:mains",
        _catalog("inertial_smooth_fixedbase", "drawer", options=_VILLA_SHORT, overrides=_INERTIAL_ON, edit=exit_on_mains_loss),
    )
    mode = r2["motion_mode"] == MODE_INERTIAL
    if not mode.any():
        pytest.xfail(INERTIAL_REASON)
    k_exit = int(np.flatnonzero(mode)[-1]) + 1
    after = r2["motion_mode"][k_exit:]
    assert (after == MODE_TERMINUS).any(), "the smoothing exit was not a Terminus close"
    assert _events(r2, "Seated") and r2["lev_state"][-1] == LEV_SEATED, "the exit did not end seated"
    assert int(r2["demand"].max()) == 0, f"monitor demand {int(r2['demand'].max())} during the ride-through exit"
