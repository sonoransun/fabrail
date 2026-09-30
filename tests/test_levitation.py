"""Levitation acceptance in closed loop (FAB-LEV-003/004/009/010/011/012/013/014/016/017/018).

Every test runs the real control stack (``fabrail._native.run``: fabrail-rt groups,
fabrail-core controllers, the integer fabrail-safety monitor) against the f64 plant.
Scenarios come from :mod:`fabrail.scenarios` (``scenarios.spec``) and are edited here
only where the catalog entry does not isolate the quantity under test; every edited
spec is re-validated (``validate_spec`` + ``cross_check``) before it runs.

Tiers (plan §2b S3, lead decision D1): DRAWER, DOOR and WALL run on Carriage6 in
voltage coil mode (the SIL-claimed mode). HEAVY Carriage6 diverges in roll (W1.5,
PHYSICS §6); where the quantity is a heave quantity HEAVY runs on Heave1 (D1
acceptance tier), where it needs the 6-DOF body (warp, rail traverse) it runs on
Carriage6 and a failure there is the D1 gap, reported, never hidden.

Measured quantities come from plant truth (channels ``gap_i_m``, ``coil_i_a``, ``z_m``,
…) unless the requirement names a controller output (``m_hat_kg``). Every assertion
message states the tolerance class and source (SPECIFICATION §0.8).

Runs are shared through the module-scoped ``runs`` cache. Owner: B4 (Wave 2).
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from typing import Any

import numpy as np
import pytest

from fabrail import scenarios
from fabrail.config import RunConfig
from fabrail.results import RunResult

#: FaultCode wire values used here (fabrail-types/src/fault.rs registry).
FAULT = {
    "CoilI2t": 0x0204,
    "BusUnder": 0x0301,
    "GapEnvelope": 0x0501,
    "LiftTimeout": 0x0502,
    "LandTimeout": 0x0503,
    "PayloadMismatch": 0x0504,
    "SaturationPersist": 0x0505,
    "SeatFailed": 0x0506,
}
#: Fault codes of severity ≥ SafeStop that end nominal operation (fault.rs severities).
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
#: ``lev_state`` wire values (fabrail-types/src/state.rs LevState).
LEV_SEATED, LEV_LEVITATED_IDLE, LEV_MOVING = 2, 5, 6
#: ``contact_state`` bits (fabrail-sim recorder): levitated, seated, landed, upper stop.
CS_LEVITATED, CS_SEATED, CS_LANDED, CS_UPPER = 1, 2, 4, 8
#: ``sat_flags`` bits (fabrail-core msg.rs FastStatus).
SAT_COIL, SAT_EXTENSION = 1 << 0, 1 << 5

CLASSES = ("drawer", "door", "wall", "heavy")


# ---------------------------------------------------------------------------------
# Helpers and the shared run cache
# ---------------------------------------------------------------------------------


def _params(cls: str, overrides: dict[str, object] | None = None, coil: str = "voltage") -> tuple[RunConfig, Any]:
    cfg = RunConfig(rail_class=cls, coil=coil, overrides=dict(overrides or {}))
    return cfg, cfg.params()


def _spec(
    name: str,
    cls: str,
    *,
    fidelity: str | None = None,
    options: dict[str, Any] | None = None,
    overrides: dict[str, object] | None = None,
    edit: Callable[[dict[str, Any], dict[str, bytes], Any], None] | None = None,
) -> tuple[Any, dict[str, Any], dict[str, bytes]]:
    """Catalog spec ``name`` for ``cls`` (voltage coil mode), optionally edited, re-validated."""
    cfg, p = _params(cls, overrides)
    d, series = scenarios.spec(name, p, cfg, fidelity=fidelity, options=options)
    if edit is not None:
        edit(d, series, p)
        d["events"] = sorted(d["events"], key=lambda e: float(e["t_s"]))
    scenarios.validate_spec(d)
    scenarios.cross_check(d, series, p)
    return p, d, series


class _Runs:
    """Module-level run cache: key → (RunResult, params, spec dict)."""

    def __init__(self, native: Any) -> None:
        self._n = native
        self._cache: dict[str, tuple[RunResult, Any, dict[str, Any]]] = {}

    def get(self, key: str, factory: Callable[[], tuple[Any, dict[str, Any], dict[str, bytes]]]):
        if key not in self._cache:
            p, d, series = factory()
            sc = self._n.Scenario.from_json(scenarios.to_json(d), series)
            self._cache[key] = (RunResult.from_native(self._n.run(p, sc)), p, d)
        return self._cache[key]


@pytest.fixture(scope="module")
def runs(native: Any) -> _Runs:
    return _Runs(native)


def _faults(r: RunResult) -> list[tuple[float, int]]:
    return [(e.t_s, int(e.a)) for e in r.events if e.kind == "FaultRaised"]


def _fault_names(r: RunResult) -> list[str]:
    inv = {v: k for k, v in FAULT.items()}
    return [f"{t:.3f}s:{inv.get(c, hex(c))}" for t, c in _faults(r)]


def _stopping_faults(r: RunResult) -> list[str]:
    return [f"{t:.3f}s:0x{c:04X}" for t, c in _faults(r) if c in _STOPPING]


def _n_mag(p: Any) -> int:
    return int(p.get("design.suspension.n_magnets"))


def _coils(r: RunResult, p: Any) -> np.ndarray:
    return np.array([r[f"coil_{i}_a"] for i in range(_n_mag(p))])


def _gaps(r: RunResult, p: Any) -> np.ndarray:
    return np.array([r[f"gap_{i}_m"] for i in range(_n_mag(p))])


def _event_time(d: dict[str, Any], kind: str) -> float:
    return float(next(e["t_s"] for e in d["events"] if e["kind"] == kind))


def _zp_gap(native: Any, p: Any, m_kg: float) -> float:
    return native.zero_power_gap(
        m_kg,
        p.get("design.suspension.g_star_m"),
        p.derived()["h_prime_m"],
        p.get("design.carriage.m_star_kg"),
    )


def _truth_mass(p: Any, d: dict[str, Any]) -> float:
    m = float(d["plant"].get("mass_scale", 1.0)) * p.get("design.carriage.m_star_kg")
    m += float(d["initial"].get("payload_kg", 0.0))
    return m + sum(float(e["delta_kg"]) for e in d["events"] if e["kind"] == "payload")


def _tier(cls: str) -> str:
    """Heave-quantity tier: Carriage6, except HEAVY on Heave1 (D1)."""
    return "heave1" if cls == "heavy" else "carriage6"


# ---------------------------------------------------------------------------------
# FAB-LEV-003 open-loop pole
# ---------------------------------------------------------------------------------

_POLE_QUOTED = {"drawer": 69.49, "door": 47.57, "wall": 32.50, "heavy": 32.50}


def _pole_edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    # Heave1, lag coil (current → 0 with τ_cl), no eddy lag, all coil drivers dead at
    # 0.5 s: the carriage leaves the zero-power equilibrium on the open-loop pole.
    d["duration_s"] = 0.9
    d["record"]["rate_hz"] = 20_000.0
    d["record"]["channels"] = ["z_m"] + [f"gap_{i}_m" for i in range(_n_mag(p))] + [f"coil_{i}_a" for i in range(_n_mag(p))]
    d["fidelity"]["eddy_lag"] = False
    d["events"] = [{"kind": "fault", "t_s": 0.5, "what": "coil_driver", "ch": i} for i in range(_n_mag(p))]
    d["metrics"]["windows"] = {}


@pytest.mark.req("FAB-LEV-003")
@pytest.mark.parametrize("cls", CLASSES)
def test_open_loop_pole_fit(native, runs, cls):
    """LEV-003: the simulated open-loop pole equals the closed form within ±0.5 %.

    Contract: p = √(k_s/m*) at g* is 69.49 / 47.57 / 32.50 / 32.50 rad/s ((E) against
    ``open_loop_pole``); with every coil driver dead (current → 0) the Heave1 plant leaves
    the zero-power equilibrium and a least-squares fit of its own heave dynamics
    g̈ = p²·δg + c₂·δg² + b·ī + c over |δg| < 100 µm recovers p within ±0.5 %
    (the δg² term absorbs the co-energy curvature, ī the decaying coil current).
    """

    def factory():
        cfg, p = _params(cls, coil="lag")
        d, series = scenarios.spec("hover", p, cfg, fidelity="heave1")
        _pole_edit(d, series, p)
        scenarios.validate_spec(d)
        scenarios.cross_check(d, series, p)
        return p, d, series

    r, p, _ = runs.get(f"pole:{cls}", factory)
    pole = p.derived()["pole_rad_s"]
    closed = native.open_loop_pole(p.get("design.suspension.g_star_m"), p.derived()["h_prime_m"])
    assert abs(pole / closed - 1.0) <= 1e-12, f"(E) rel 1e-12, PHYSICS §3: derived {pole} vs open_loop_pole {closed}"
    assert abs(pole - _POLE_QUOTED[cls]) <= 0.005, (
        f"(E) quoted to 0.01 rad/s, SPEC FAB-LEV-003: {pole:.4f} vs {_POLE_QUOTED[cls]}"
    )

    t, h = r.t_s, 1.0 / r.rate_hz
    g = _gaps(r, p).mean(axis=0)
    i = _coils(r, p).mean(axis=0)
    k0 = int(np.searchsorted(t, 0.5))
    dg = g - g[k0 - 1]
    acc = np.gradient(np.gradient(g, h), h)
    ok = (t > 0.5 + 3 * h) & (np.abs(dg) < 1e-4)
    idx = np.flatnonzero(ok)
    assert idx.size > 0, "no open-loop excursion after the coil-driver faults"
    end = idx[0]
    while end + 1 < ok.size and ok[end + 1]:
        end += 1
    sel = slice(idx[0], end)
    n = end - idx[0]
    assert n >= 500 and np.abs(dg[sel]).max() > 5e-5, (
        f"open-loop window too short to fit: {n} samples, max |δg| {np.abs(dg[sel]).max():.2e} m"
    )
    x = np.column_stack([dg[sel], dg[sel] ** 2, i[sel], np.ones(n)])
    coef, *_ = np.linalg.lstsq(x, acc[sel], rcond=None)
    assert coef[0] > 0.0, f"fitted stiffness term {coef[0]} is not destabilising"
    p_fit = math.sqrt(coef[0])
    assert abs(p_fit / pole - 1.0) <= 0.005, (
        f"(S) ±0.5 %, SPEC FAB-LEV-003: fitted p {p_fit:.3f} rad/s vs closed form {pole:.3f} rad/s "
        f"({100 * (p_fit / pole - 1):+.3f} %)"
    )


# ---------------------------------------------------------------------------------
# FAB-LEV-004 zero power / FAB-LEV-013 payload estimate
# ---------------------------------------------------------------------------------


def _payload_step(runs: _Runs, cls: str):
    return runs.get(f"payload_step:{cls}", lambda: _spec("payload_step", cls, fidelity=_tier(cls)))


@pytest.mark.req("FAB-LEV-004")
@pytest.mark.parametrize("cls", CLASSES)
def test_zero_power_load_step(native, runs, cls):
    """LEV-004: after a +10 % m* step, |i_ss| < 1 mA within 2 s and the gap follows m(g).

    Contract: per coil, the mean truth current over [t_step + 2 s, end] is below 1 mA in
    magnitude (the steady-state current; the sensor-noise ripple averages out); the mean
    lift-element gap equals ``zero_power_gap(m)`` within ±2 µm (DOOR +98.1 N → 4.5967 mm);
    PayloadMismatch is raised exactly when m > payload_mismatch_ratio·m_rated, and
    levitation continues (Severity::Degrade: no stopping fault, carriage levitated).
    """
    r, p, d = _payload_step(runs, cls)
    t_step = _event_time(d, "payload")
    m = _truth_mass(p, d)
    w = r.t_s >= t_step + 2.0
    assert w.sum() > 100, "run too short for the 2 s settling window"
    i_ss = _coils(r, p)[:, w].mean(axis=1)
    assert np.all(np.abs(i_ss) < 1e-3), (
        f"(S) |i_ss| < 1 mA within 2 s, SPEC FAB-LEV-004: per-coil mean {np.round(i_ss * 1e3, 3)} mA"
    )
    g_zp = _zp_gap(native, p, m)
    g = float(_gaps(r, p)[:, w].mean())
    assert abs(g - g_zp) <= 2e-6, f"(S) ±2 µm, SPEC FAB-LEV-004: gap {g * 1e3:.4f} mm vs m(g) {g_zp * 1e3:.4f} mm at {m:.2f} kg"
    if cls == "door":
        assert abs(g_zp - 4.5967e-3) <= 5e-8, f"(E) anchor 4.5967 mm, SPEC FAB-LEV-004: {g_zp * 1e3:.5f} mm"
    mismatch = m > p.get("control.supervisor.payload_mismatch_ratio") * p.get("design.carriage.m_rated_kg")
    raised = any(c == FAULT["PayloadMismatch"] for _, c in _faults(r))
    assert raised == mismatch, (
        f"PayloadMismatch raised={raised}, expected {mismatch} (m {m:.2f} kg vs 1.05·m_rated), SPEC FAB-LEV-004/SAF-018"
    )
    assert not _stopping_faults(r), f"levitation must continue (Degrade only): {_stopping_faults(r)}"
    assert int(r["lev_state"][-1]) == LEV_LEVITATED_IDLE, f"lev_state at end {int(r['lev_state'][-1])}, expected LevitatedIdle"


@pytest.mark.req("FAB-LEV-013")
@pytest.mark.parametrize("cls", CLASSES)
def test_payload_estimate_converges(runs, cls):
    """LEV-013: m̂ converges within ±5 % in ≤ 1 s; the governor uses m_gov, not m̂.

    Contract: m̂ starts from m_rated at the warm start; from 0.9 s to the step (< 1 s)
    it is within ±5 % of m*, and from 1 s after the +10 % step to the end within ±5 %
    of the new truth mass. The
    non-safety tag: on a DOOR/DRAWER move (soft_open) the governor energy channel equals
    ½·m_gov·v̂² with m_gov = max(m_rated, 1.05·m̂) (±2 %, samples above 0.1 J).
    """
    r, p, d = _payload_step(runs, cls)
    t_step = _event_time(d, "payload")
    m0 = p.get("design.carriage.m_star_kg")
    m1 = _truth_mass(p, d)
    mh = r["m_hat_kg"]
    pre = (r.t_s >= 0.9) & (r.t_s < t_step)
    post = r.t_s >= t_step + 1.0
    err0 = np.abs(mh[pre] / m0 - 1.0).max()
    err1 = np.abs(mh[post] / m1 - 1.0).max()
    assert err0 <= 0.05, f"(S) ±5 %, SPEC FAB-LEV-013: hover m̂ error {100 * err0:.2f} % of {m0} kg"
    assert err1 <= 0.05, f"(S) ±5 % within 1 s, SPEC FAB-LEV-013: m̂ error {100 * err1:.2f} % of {m1:.2f} kg"
    if cls in ("drawer", "door"):
        r2, p2, _ = runs.get(f"soft_open:{cls}", lambda: _spec("soft_open", cls))
        m_gov = np.maximum(p2.get("design.carriage.m_rated_kg"), 1.05 * r2["m_hat_kg"])
        e_ref = 0.5 * m_gov * r2["v_hat_mps"] ** 2
        e = r2["e_kin_gov_j"]
        sel = e > 0.1
        assert sel.sum() > 10, "no moving samples above 0.1 J"
        rel = np.abs(e[sel] / e_ref[sel] - 1.0).max()
        assert rel <= 0.02, f"(P) ±2 %, SPEC FAB-SAF-001 m_gov rule: governor energy vs ½·m_gov·v̂², worst {100 * rel:.2f} %"


# ---------------------------------------------------------------------------------
# FAB-LEV-009 warp loop
# ---------------------------------------------------------------------------------


def _null_projector(p: Any) -> np.ndarray:
    """Orthogonal projector onto the null space of the controlled modal map (modal.rs)."""
    doc = json.loads(p.to_json())
    s = doc["design"]["suspension"]
    st = s["stations"][: s["n_magnets"]]
    cg = doc["design"]["carriage"]["cg_m"]
    rows = [np.ones(len(st)), np.array([q["x_m"] - cg[0] for q in st])]
    if doc["control"]["levitation"]["roll_active"]:
        rows.append(np.array([q["y_m"] - cg[1] for q in st]))
    t = np.array(rows)
    return np.eye(len(st)) - t.T @ np.linalg.pinv(t @ t.T) @ t


_WARP_CASES = [("drawer", "soft_open"), ("door", "lift"), ("wall", "lift"), ("heavy", "lift")]


@pytest.mark.req("FAB-LEV-009")
@pytest.mark.parametrize(("cls", "name"), _WARP_CASES)
def test_warp_loop_equalises(runs, cls, name):
    """LEV-009: the warp loop equalises coil currents to ≤ 5 % within 1 s.

    Contract: through a lift (DRAWER: unseat and drive-out), the null-space part of the
    measured coil currents (the orthogonal projection onto null(T) of the controlled
    heave/pitch(/roll) map, modal.rs) is, from 1 s after the peak total coil current to
    the end, at most 5 % of that peak current norm. All four classes on Carriage6 (the
    warp needs the 6-DOF body); HEAVY Carriage6 is the D1 gap.

    Note: these symmetric carriages excite null-space currents only at the
    current-sense noise level in this plant (no per-magnet mismatch input exists in
    fabrail.scenario/1), so the bound also shows the allocation keeps the lift in the
    controlled-mode range.
    """
    r, p, _ = runs.get(f"{name}:{cls}:c6", lambda: _spec(name, cls, fidelity="carriage6"))
    stop = _stopping_faults(r)
    assert not stop, f"{cls} {name} on Carriage6 ended in stopping faults {stop} (D1 for HEAVY)"
    cur = _coils(r, p)
    tot = np.linalg.norm(cur, axis=0)
    null = np.linalg.norm(_null_projector(p) @ cur, axis=0)
    k = int(np.argmax(tot))
    k1 = int(np.searchsorted(r.t_s, r.t_s[k] + 1.0))
    assert k1 < r.n_samples - 10, "run ends within 1 s of the peak current"
    ratio = null[k1:].max() / tot[k]
    assert ratio <= 0.05, (
        f"(S) ≤ 5 % within 1 s, SPEC FAB-LEV-009: null-space current {null[k1:].max():.4f} A vs peak {tot[k]:.3f} A "
        f"({100 * ratio:.2f} %)"
    )


# ---------------------------------------------------------------------------------
# FAB-LEV-010 rail irregularity
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-LEV-010")
@pytest.mark.parametrize(
    "cls",
    ["drawer", "door", pytest.param("wall", marks=pytest.mark.slow), pytest.param("heavy", marks=pytest.mark.slow)],
)
def test_rail_irregularity_tracking(runs, cls):
    """LEV-010: an open at v_max over an L/1150 rail with joints and 20 µm roughness.

    Contract: the carriage completes the travel (final x within the class travel
    tolerance of the open end) with no stopping fault; no upper-stop or roller contact
    during the traverse; lift-element gap error |g_i − g_zp(m)| ≤ 0.3·(g* − g_up).
    Carriage6 (catalog tier); WALL/HEAVY traverses are slow-tier (PLAN §2b.13).
    """
    r, p, d = runs.get(f"rail_traverse:{cls}", lambda: _spec("rail_traverse", cls))
    x_open = p.get("design.rail.x_closed_m") + p.get("design.rail.travel_m")
    tol = p.get("control.motion.pos_tol_travel_m")
    assert not _stopping_faults(r), f"traverse interrupted by {_stopping_faults(r)} (SPEC FAB-LEV-010)"
    assert abs(r["x_m"][-1] - x_open) <= tol, f"traverse ended at {r['x_m'][-1]:.4f} m, open end {x_open:.4f} m ±{tol}"
    k = r.kpis
    assert k.get("traverse.upper_force_max_n", 0.0) == 0.0, f"upper-stop contact {k.get('traverse.upper_force_max_n')} N"
    assert k.get("traverse.roller_force_max_n", 0.0) == 0.0, f"roller contact {k.get('traverse.roller_force_max_n')} N"
    bound = 0.3 * (p.get("design.suspension.g_star_m") - p.get("design.suspension.g_up_m"))
    dev = k["traverse.gap_dev_max_m"]
    assert dev <= bound, f"(S) ≤ 0.3·(g* − g_up) = {bound * 1e3:.3f} mm, SPEC FAB-LEV-010: gap error {dev * 1e3:.4f} mm"


# ---------------------------------------------------------------------------------
# FAB-LEV-011 top-hung sway
# ---------------------------------------------------------------------------------


def _sway_impulse_edit(f_n: float, pulse_s: float) -> Callable[[dict[str, Any], dict[str, bytes], Any], None]:
    def edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
        fs, dur = 1000.0, 9.0
        t = np.arange(int(round(dur * fs)) + 1) / fs
        ay = np.where((t >= 1.0) & (t < 1.0 + pulse_s), -f_n / p.get("design.carriage.m_star_kg"), 0.0)
        d["series"]["floor"] = {"axis": "time", "fs_hz": fs, "t0_s": 0.0, "channels": ["ay"]}
        series["floor"] = np.ascontiguousarray(ay, dtype="<f8").tobytes()
        d["events"] = []
        d["duration_s"] = dur
        d["record"]["channels"] = ["roll_rad", "y_m", "f_guide_n", "f_upper_n", "f_rollers_n"]
        d["metrics"]["windows"] = {}

    return edit


def _damped_fit(t: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    """(ζ, ω_n, R²) of the best damped-sinusoid fit A·e^{−ζω t}·cos(ω_d t + φ) + c."""
    from scipy import optimize

    def model(tt, a, z, wn, ph, c):
        return a * np.exp(-z * wn * tt) * np.cos(wn * math.sqrt(max(1.0 - z * z, 1e-9)) * tt + ph) + c

    best = (math.nan, math.nan, -math.inf)
    for wn0 in (1.0, 2.0, 3.0, 5.0):
        try:
            popt, _ = optimize.curve_fit(model, t, y, p0=[np.abs(y).max(), 0.1, wn0, 0.0, 0.0], maxfev=20000)
        except RuntimeError:
            continue
        res = y - model(t, *popt)
        r2 = 1.0 - res.var() / y.var()
        if r2 > best[2]:
            best = (float(popt[1]), float(popt[2]), float(r2))
    return best


@pytest.mark.req("FAB-LEV-011")
@pytest.mark.parametrize("cls", ["door", "wall"])
def test_top_hung_sway_push(runs, cls):
    """LEV-011: a 20 N lateral push engages the guide fin; x-drag = μN ±5 %; sway ζ ≥ 0.05.

    Contract: (a) catalog ``lateral_push`` (20 N as a floor a_y step, the schema has no
    y-force event, then travel): while levitated (before the class rest landing) the fin
    carries load, there is no magnet–rail contact (upper stop and rollers unloaded), and while the fin is loaded (> 5 N) and the leaf
    travels (|v| > 3 mm/s) the tangential contact drag equals pad_mu·F_fin within ±5 %
    (PHYSICS §6, tanh regularisation ≥ 0.995 there). (b) a 20 N·50 ms (DOOR 10 N) lateral
    impulse that stays inside the fin clearance: the pendulum sway (roll about the rail,
    PHYSICS §6) decays with fitted ζ ≥ 0.05 (R² ≥ 0.9).
    """

    def push():
        return _spec(
            "lateral_push",
            cls,
            edit=lambda d, s, p: d["record"].__setitem__(
                "channels",
                [
                    "x_m",
                    "v_mps",
                    "y_m",
                    "roll_rad",
                    "f_guide_n",
                    "f_pads_n",
                    "f_tangential_n",
                    "f_upper_n",
                    "f_rollers_n",
                    "lev_state",
                ],
            ),
        )

    r, p, _ = runs.get(f"lateral_push:{cls}", push)
    lev = np.isin(r["lev_state"], (LEV_LEVITATED_IDLE, LEV_MOVING))
    fin, ft, v = r["f_guide_n"], r["f_tangential_n"], r["v_mps"]
    assert fin[lev].max() > 0.0, f"(S) SPEC FAB-LEV-011: the guide fin never engaged (max {fin[lev].max():.2f} N)"
    up, rol = r["f_upper_n"][lev].max(), r["f_rollers_n"][lev].max()
    assert up == 0.0 and rol == 0.0, f"magnet–rail contact while levitated: upper {up:.1f} N, rollers {rol:.1f} N"
    mu = p.get("design.guidance.pad_mu")
    sel = lev & (fin > 5.0) & (np.abs(v) > 3e-3)
    assert sel.sum() > 0, "no fin-loaded travel samples to measure the x-drag"
    rel = np.abs(np.abs(ft[sel]) / (mu * fin[sel]) - 1.0)
    assert rel.max() <= 0.05, f"(S) ±5 %, SPEC FAB-LEV-011: x-drag/μN worst {100 * rel.max():.2f} % ({sel.sum()} samples)"

    f_imp = 10.0 if cls == "door" else 20.0
    r2, _, _ = runs.get(f"sway_impulse:{cls}", lambda: _spec("lateral_push", cls, edit=_sway_impulse_edit(f_imp, 0.05)))
    w = r2.t_s >= 1.1
    zeta, wn, r2fit = _damped_fit(r2.t_s[w] - 1.1, r2["roll_rad"][w])
    assert r2fit >= 0.9, f"sway fit R² {r2fit:.3f} < 0.9 (fin {r2['f_guide_n'].max():.1f} N)"
    assert zeta >= 0.05, f"(S) ζ ≥ 0.05, SPEC FAB-LEV-011 / PHYSICS §6: fitted sway ζ {zeta:.4f} (ω_n {wn:.3f} rad/s)"


# ---------------------------------------------------------------------------------
# FAB-LEV-012 lift latency
# ---------------------------------------------------------------------------------

_LIFT_LIMIT_S = {"door": 0.3, "wall": 1.0, "heavy": 2.0}


@pytest.mark.req("FAB-LEV-012")
@pytest.mark.parametrize(
    ("cls", "mass_kg"), [("drawer", 8.0), ("drawer", 26.0), ("drawer", 48.0), ("door", None), ("wall", None), ("heavy", None)]
)
def test_lift_latency(runs, cls, mass_kg):
    """LEV-012: DRAWER unseat ≤ 100 ms, DOOR lift ≤ 300 ms, WALL ≤ 1 s, HEAVY ≤ 2 s.

    Contract: the ``unseat_time_s`` / ``lift_time_s`` KPI (directive → truth out of the
    seat / levitated) is within the class limit and no upper-stop contact occurs
    (``upper_force_max_n`` = 0). DRAWER at 8 / 26 / 48 kg; HEAVY on Heave1 (D1).
    """
    if cls == "drawer":
        r, _, _ = runs.get(f"unseat:{mass_kg}", lambda: _spec("unseat", "drawer", options={"mass_kg": mass_kg}))
        kpi, limit = r.kpis.get("unseat_time_s"), 0.1
    else:
        r, _, _ = runs.get(f"lift:{cls}", lambda: _spec("lift", cls, fidelity=_tier(cls)))
        kpi, limit = r.kpis.get("lift_time_s"), _LIFT_LIMIT_S[cls]
    assert kpi is not None, f"{cls}: lift/unseat never completed (faults {_fault_names(r)})"
    assert kpi <= limit, f"(S) ≤ {limit} s, SPEC FAB-LEV-012: {cls} {kpi * 1e3:.1f} ms"
    assert r.kpis["upper_force_max_n"] == 0.0, f"upper-stop contact {r.kpis['upper_force_max_n']:.1f} N during lift"


# ---------------------------------------------------------------------------------
# FAB-LEV-014 thermal drift
# ---------------------------------------------------------------------------------


#: Compressed thermal capacities for the quasi-static ramp (PLAN §2b.13): the preset
#: magnet node has τ = r_th·c_th ≈ 900 s, so a 20 s ramp would warm it by < 1 K. With
#: c_th = 0.5 J/K (τ ≈ 1.5 s) the magnet follows the ambient; the static shift does not
#: depend on c_th (PHYSICS §12).
_THERMAL_COMPRESS = {"design.thermal.c_th_mag_j_per_k": 0.5, "design.thermal.c_th_coil_j_per_k": 0.5}


def _thermal_edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    d["duration_s"] = 31.0
    d["metrics"]["windows"] = {"hot": [29.0, 31.0]}


@pytest.mark.req("FAB-LEV-014")
def test_thermal_drift_absorbed(native, runs):
    """LEV-014: DOOR on Heave1, +40 K compressed quasi-static ramp: gap shift ±3 %, |i| < 5 mA.

    Contract: with the thermal nodes compressed (c_th 0.5 J/K) the +40 K ambient ramp
    over 20 s plus a 10 s hold brings the magnet to +40 ±1 K; the mean gap over the last
    2 s minus the mean before the ramp equals ``thermal_gap_shift(params, 40)``
    (−0.416 mm) within ±3 %; the steady coil current (per-coil mean over the hot window)
    stays below 5 mA: zero power absorbs the B_r drift by moving the gap, not by current.
    "Absorbed" also means no fault: the drift must not be read as a payload
    (PayloadMismatch is for m̂ > 1.05·m_rated from real payload, SAF-018).
    """
    r, p, d = runs.get(
        "thermal_ramp:door",
        lambda: _spec("thermal_ramp", "door", fidelity="heave1", overrides=_THERMAL_COMPRESS, edit=_thermal_edit),
    )
    t0 = _event_time(d, "temp_ramp")
    hot = d["metrics"]["windows"]["hot"]
    g = _gaps(r, p).mean(axis=0)
    before = (r.t_s >= 0.3) & (r.t_s < t0)
    after = (r.t_s >= hot[0]) & (r.t_s < hot[1])
    dt_mag = float(r["t_mag_c"][after].mean() - r["t_mag_c"][before].mean())
    assert abs(dt_mag - 40.0) <= 1.0, f"precondition: magnet warmed by {dt_mag:.2f} K, expected +40 ±1 K"
    shift = float(g[after].mean() - g[before].mean())
    ref = native.thermal_gap_shift(p, 40.0)
    assert abs(ref - (-0.416e-3)) <= 0.5e-6, f"(E) anchor −0.416 mm, SPEC FAB-LEV-014: {ref * 1e3:.4f} mm"
    assert abs(shift / ref - 1.0) <= 0.03, (
        f"(S) ±3 %, SPEC FAB-LEV-014: dynamic shift {shift * 1e3:.4f} mm vs {ref * 1e3:.4f} mm (magnet +{dt_mag:.2f} K)"
    )
    i_hot = _coils(r, p)[:, after].mean(axis=1)
    assert np.all(np.abs(i_hot) < 5e-3), f"(S) |i| < 5 mA, SPEC FAB-LEV-014: {np.round(i_hot * 1e3, 3)} mA"
    assert not _faults(r), f"thermal drift raised faults {_fault_names(r)} (m̂ is not temperature-compensated)"


# ---------------------------------------------------------------------------------
# FAB-LEV-016 DRAWER extension envelope
# ---------------------------------------------------------------------------------

_ENV_MASSES = (8.0, 26.0, 48.0)
_ENV_EXT_M = (0.15, 0.30, 0.33, 0.36, 0.42)
_ENV_TRAVEL_M = 0.45


def _envelope_point(native: Any, m_kg: float, ext_m: float) -> dict[str, Any]:
    over = {"design.rail.travel_m": _ENV_TRAVEL_M, "safety.x_max_m": _ENV_TRAVEL_M + 0.01}
    cfg, p = _params("drawer", over)
    d, series = scenarios.spec("hover", p, cfg)
    d["initial"] = {
        "lev": "levitated",
        "x_m": p.get("design.rail.x_closed_m") + ext_m,
        "payload_kg": m_kg - p.get("design.carriage.m_star_kg"),
    }
    d["duration_s"] = 1.5
    d["metrics"]["windows"] = {}
    scenarios.validate_spec(d)
    scenarios.cross_check(d, series, p)
    try:
        r = RunResult.from_native(native.run(p, native.Scenario.from_json(scenarios.to_json(d), series)))
    except RuntimeError as exc:
        return {"m": m_kg, "s": ext_m, "error": str(exc)}
    sat = int(np.bitwise_or.reduce(r["sat_flags"].astype(np.int64)))
    codes = {c for _, c in _faults(r)}
    flagged = bool(sat & (SAT_COIL | SAT_EXTENSION)) or bool(
        codes & {FAULT["SaturationPersist"], FAULT["LiftTimeout"], FAULT["GapEnvelope"]}
    )
    contactless = (
        int(r["lev_state"][-1]) == LEV_LEVITATED_IDLE
        and r.kpis["upper_force_max_n"] == 0.0
        and r.kpis["roller_force_max_n"] == 0.0
        and not (codes & _STOPPING)
    )
    return {"m": m_kg, "s": ext_m, "contactless": contactless, "flagged": flagged, "sat": sat, "faults": sorted(codes)}


@pytest.mark.req("FAB-LEV-016")
def test_drawer_extension_envelope(native, runs):
    """LEV-016: DRAWER contactless travel ≤ 50 % extension; the envelope is reported and flagged.

    Contract: the preset travel is 0.30 m = 50 % of the 0.6 m body. A simulated envelope
    (Carriage6 hover at 8 / 26 / 48 kg and extensions 0.15–0.42 m, the travel parameter
    widened to 0.45 m so validate admits the start) must (i) run with finite plant truth
    at every point, (ii) flag every point that does not hold contactless levitation
    (SAT_COIL/SAT_EXTENSION or SaturationPersist/LiftTimeout/GapEnvelope), never a silent
    loss, and (iii) be monotone (feasible at (m, s) ⇒ feasible at lighter m and shorter s).
    No specific feasible point is asserted (SPEC FAB-LEV-016); the table is in the message.
    """
    _, p = _params("drawer")
    travel, length = p.get("design.rail.travel_m"), p.get("design.carriage.length_m")
    assert abs(travel - 0.30) <= 1e-12, f"(E) preset travel 0.30 m, SPEC FAB-LEV-016: {travel}"
    assert travel <= 0.5 * length + 1e-12, f"(E) ≤ 50 % extension: travel {travel} m, body {length} m"

    table = [_envelope_point(native, m, s) for m in _ENV_MASSES for s in _ENV_EXT_M]
    summary = "; ".join(
        f"{q['m']:.0f}kg@{q['s']:.2f}m:"
        + ("ERR" if "error" in q else ("ok" if q["contactless"] else "x") + ("F" if q["flagged"] else ""))
        for q in table
    )
    errors = [q for q in table if "error" in q]
    assert not errors, (
        f"non-finite plant truth in the envelope: {[(q['m'], q['s'], q['error'][:80]) for q in errors]} | {summary}"
    )
    silent = [(q["m"], q["s"]) for q in table if not q["contactless"] and not q["flagged"]]
    assert not silent, f"infeasible points without a saturation flag: {silent} | {summary}"
    ok = {(q["m"], q["s"]) for q in table if q["contactless"]}
    for m, s in ok:
        for m2 in _ENV_MASSES:
            for s2 in _ENV_EXT_M:
                if m2 <= m and s2 <= s:
                    assert (m2, s2) in ok, f"envelope not monotone: ({m}, {s}) feasible but ({m2}, {s2}) not | {summary}"


# ---------------------------------------------------------------------------------
# FAB-LEV-017 DRAWER seat
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-LEV-017")
def test_drawer_seat_unseat(runs):
    """LEV-017: unseating within i_pulse for 8–48 kg; rest height independent of load ±0.05 mm.

    Contract: closed-form lift-off currents −4.98 A (8 kg) and +11.23 A (48 kg) ((E) to
    the quoted 0.05 A); seat geometry (z clearance ≈ 0.15 mm, x push-stop compliance
    ≥ 2 mm); in simulation each mass unseats (Unseated event) with peak |i| ≤ i_pulse and
    the lift-off current sign of the closed form; the seated truth height before the
    unseat differs by at most 0.05 mm between 8 and 48 kg.
    """
    _, p = _params("drawer")
    der = p.derived()
    assert abs(der["lift_current_min_a"] - (-4.98)) <= 0.05, f"(E) −4.98 A, SPEC FAB-LEV-017: {der['lift_current_min_a']:.4f} A"
    assert abs(der["lift_current_rated_a"] - 11.23) <= 0.05, (
        f"(E) +11.23 A, SPEC FAB-LEV-017: {der['lift_current_rated_a']:.4f} A"
    )
    assert abs(p.get("design.landing.seat_z_clearance_m") - 0.15e-3) <= 0.02e-3, "seat z clearance ≈ 0.15 mm"
    assert p.get("design.landing.seat_x_compliance_m") >= 2e-3, "seat x push-stop compliance ≥ 2 mm"
    i_pulse = p.get("design.suspension.i_pulse_a")
    z_rest = {}
    for m in (8.0, 26.0, 48.0):
        r, pp, d = runs.get(f"unseat:{m}", lambda m=m: _spec("unseat", "drawer", options={"mass_kg": m}))
        ev = [e for e in r.events if e.kind == "Unseated"]
        assert ev, f"{m} kg: no Unseated event (faults {_fault_names(r)})"
        t_lift = _event_time(d, "command")
        k0, k1 = int(np.searchsorted(r.t_s, t_lift)), int(np.searchsorted(r.t_s, ev[0].t_s))
        cur = _coils(r, pp)[:, k0 : k1 + 1]
        peak = float(np.abs(cur).max())
        assert peak <= i_pulse, f"(S) ≤ i_pulse {i_pulse} A, SPEC FAB-LEV-017: {m} kg peak {peak:.2f} A"
        i_off = float(cur[:, -1].mean())
        cf = der["lift_current_min_a"] if m == 8.0 else der["lift_current_rated_a"] if m == 48.0 else None
        if cf is not None:
            assert math.copysign(1.0, i_off) == math.copysign(1.0, cf), (
                f"{m} kg lift-off current {i_off:.2f} A, closed form {cf:.2f} A"
            )
        z_rest[m] = float(r["z_m"][max(k0 - 20, 0) : k0].mean())
    dz = abs(z_rest[8.0] - z_rest[48.0])
    assert dz <= 0.05e-3, (
        f"(S) ±0.05 mm, SPEC FAB-LEV-017: seated height 8 kg {z_rest[8.0] * 1e3:.4f} mm vs 48 kg {z_rest[48.0] * 1e3:.4f} mm "
        f"(Δ {dz * 1e3:.3f} mm)"
    )


def _pulse_edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    d["initial"] = {
        "lev": "seated",
        "x_m": p.get("design.rail.x_closed_m"),
        "payload_kg": 48.0 - p.get("design.carriage.m_star_kg"),
    }
    d["events"] = [{"kind": "command", "t_s": 0.2, "cmd": "open"}]
    d["duration_s"] = 1.5
    d["record"]["rate_hz"] = 20_000.0
    d["record"]["channels"] = ["x_m", "z_m", "lev_state", "seat_status", "sat_flags"] + [f"coil_{i}_a" for i in range(_n_mag(p))]
    d["metrics"]["windows"] = {}


@pytest.mark.req("FAB-LEV-017")
def test_drawer_pulse_budget(runs):
    """LEV-017: unseat plus slot exit at 48 kg within t_pulse = 200 ms under the I²t clamp.

    Contract: from the first coil current above i_cont after the open command to the
    first truth x ≥ x_closed + slot_len (slot exit) takes ≤ t_pulse; per element
    ∫(i² − i_cont²)⁺dt over that interval ≤ (i_pulse² − i_cont²)·t_pulse; no CoilI2t,
    SaturationPersist or SeatFailed fault; the drawer is levitated at the end.
    """
    r, p, _ = runs.get("pulse:48", lambda: _spec("unseat", "drawer", options={"mass_kg": 48.0}, edit=_pulse_edit))
    s = p.get
    i_c, i_p, t_p = s("design.suspension.i_cont_a"), s("design.suspension.i_pulse_a"), s("design.suspension.t_pulse_s")
    x_exit = s("design.rail.x_closed_m") + s("design.landing.slot_len_m")
    cur = _coils(r, p)
    t = r.t_s
    after = t >= 0.2
    k_start = int(np.argmax(after & (np.abs(cur).max(axis=0) > i_c)))
    exited = np.flatnonzero(r["x_m"] >= x_exit)
    bad = [n for n in _fault_names(r) if any(k in n for k in ("CoilI2t", "SaturationPersist", "SeatFailed"))]
    assert exited.size, f"48 kg drawer never left the slot (x max {r['x_m'].max() * 1e3:.2f} mm; faults {_fault_names(r)})"
    k_exit = int(exited[0])
    dt = t[k_exit] - t[k_start]
    budget = (i_p**2 - i_c**2) * t_p
    used = (np.clip(cur[:, k_start : k_exit + 1] ** 2 - i_c**2, 0.0, None).sum(axis=1)) / r.rate_hz
    assert not bad, (
        f"I²t clamp exceeded: {bad} (slot exit after {dt * 1e3:.1f} ms, I²t used {np.round(used, 2)} of {budget:.2f} A²s)"
    )
    assert dt <= t_p, (
        f"(S) ≤ t_pulse {t_p} s, SPEC FAB-LEV-017: unseat + slot exit took {dt * 1e3:.1f} ms "
        f"(I²t used {np.round(used, 2)} of {budget:.2f} A²s)"
    )
    assert np.all(used <= budget), f"(S) I²t ≤ {budget:.2f} A²s, SPEC FAB-LEV-017: {np.round(used, 2)} A²s"
    assert int(r["lev_state"][-1]) in (LEV_LEVITATED_IDLE, LEV_MOVING), f"lev_state at end {int(r['lev_state'][-1])}"


# ---------------------------------------------------------------------------------
# FAB-LEV-018 modified zero-power band
# ---------------------------------------------------------------------------------

_BAND_CASES = [("drawer", 48.0), ("drawer", 7.0), ("door", 120.0), ("wall", 580.0), ("heavy", 5800.0)]


@pytest.mark.req("FAB-LEV-018")
@pytest.mark.parametrize(("cls", "mass_kg"), _BAND_CASES)
def test_modified_zero_power_band(native, runs, cls, mass_kg):
    """LEV-018: outside the float band the gap is clamped to the band edge with the closed-form bias.

    Contract: for a truth mass whose zero-power gap lies outside ``zp_band_m`` (DRAWER
    48 kg below 1.5 mm, 7 kg above 6.0 mm; DOOR/WALL/HEAVY above rated), the hovering
    gap (mean of lift elements over [2, 4] s) sits at the band edge within ±5 µm and the
    mean coil current equals ``bias_current(params, edge, m)`` within ±2 % (DRAWER 48 kg
    +0.39 A, (E) to the quoted 0.005 A). HEAVY on Heave1 (D1).
    """

    def edit(d, series, p):
        d["initial"] = {
            "lev": "levitated",
            "x_m": d["initial"]["x_m"],
            "payload_kg": mass_kg - p.get("design.carriage.m_star_kg"),
        }
        d["duration_s"] = 4.0
        d["metrics"]["windows"] = {}

    r, p, d = runs.get(f"band:{cls}:{mass_kg}", lambda: _spec("hover", cls, fidelity=_tier(cls), edit=edit))
    lo, hi = p.get("design.suspension.zp_band_m.0"), p.get("design.suspension.zp_band_m.1")
    g_zp = _zp_gap(native, p, mass_kg)
    assert not (lo <= g_zp <= hi), f"precondition: zero-power gap {g_zp * 1e3:.3f} mm is inside the band"
    edge = lo if g_zp < lo else hi
    w = r.t_s >= 2.0
    gap = float(_gaps(r, p)[:, w].mean())
    i_mean = float(_coils(r, p)[:, w].mean())
    bias = native.bias_current(p, edge, mass_kg)
    if cls == "drawer" and mass_kg == 48.0:
        assert abs(bias - 0.39) <= 0.005, f"(E) +0.39 A at 48 kg, SPEC FAB-LEV-018: {bias:.4f} A"
    assert abs(gap - edge) <= 5e-6, f"(S) band edge ±5 µm, SPEC FAB-LEV-018: gap {gap * 1e3:.4f} mm vs edge {edge * 1e3:.3f} mm"
    assert abs(i_mean / bias - 1.0) <= 0.02, f"(S) ±2 %, SPEC FAB-LEV-018: bias {i_mean:.4f} A vs closed form {bias:.4f} A"
    assert not _stopping_faults(r), f"stopping faults {_stopping_faults(r)}"
