"""Power, hold-up and energy reporting (FAB-SAF-009 hold-up, FAB-PWR-002/003/004/006).

Closed-loop runs (``_native.run``) measure the plant's power path: ``p_bus_w`` (coil,
LSM and declared electronics), the zero-power copper loss (``hover_power_w`` KPI: coil +
LSM copper while levitated idle) and the supercap energy ``e_hold_j``. The analytic side
comes from ``Params.derived()`` (``derive::holdup_energy``: avail = η·½·C·(V_max² −
V_min²); need = transfer copper + electronics over stop, transfer and seat).

Tiers: DRAWER, DOOR, WALL on Carriage6 in voltage coil mode; HEAVY on Heave1 (hover,
landing) or Travel1d (moves), lead decision D1. Owner: B4 (Wave 2).
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
#: ``contact_state`` bits: seated, landed.
CS_SEATED, CS_LANDED = 2, 4
#: ``lev_state`` wire values (LevState).
LEV_LANDED, LEV_LEVITATED_IDLE = 3, 5


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
    edit: Callable[[dict[str, Any], dict[str, bytes], Any], None] | None = None,
):
    def factory():
        cfg = RunConfig(rail_class=cls, coil="voltage")
        p = cfg.params()
        d, series = scenarios.spec(name, p, cfg, fidelity=fidelity, options=options)
        if edit is not None:
            edit(d, series, p)
        return p, d, series

    return factory


def _params(cls: str) -> Any:
    return RunConfig(rail_class=cls, coil="voltage").params()


def _first(t: np.ndarray, mask: np.ndarray) -> float | None:
    k = int(np.argmax(mask))
    return float(t[k]) if mask.size and mask[k] else None


def _faults(r: RunResult) -> list[str]:
    return [f"{e.t_s:.3f}s:0x{int(e.a):04X}" for e in r.events if e.kind == "FaultRaised"]


_LOSS_CHANNELS = ["x_m", "v_mps", "z_m", "p_bus_w", "e_hold_j", "contact_state", "lev_state", "motion_mode"]


def _loss_spent(r: RunResult, p: Any, d: dict[str, Any]) -> tuple[float, float, float]:
    """(spent, usable at loss, touchdown time): supercap energy from the loss to 0.5 s after touchdown."""
    t = r.t_s
    t_loss = next(float(e["t_s"]) for e in d["events"] if e["kind"] == "power_loss")
    cs = r["contact_state"].astype(np.int64)
    t_td = _first(t, (t >= t_loss) & ((cs & (CS_SEATED | CS_LANDED)) != 0))
    assert t_td is not None, f"never touched down after the loss (faults {_faults(r)})"
    e = r["e_hold_j"]
    e_min = 0.5 * p.get("design.power.supercap_c_f") * p.get("design.power.supercap_v_min_v") ** 2
    k0 = int(np.searchsorted(t, t_loss))
    k1 = min(int(np.searchsorted(t, t_td + 0.5)), t.size - 1)
    return float(e[k0] - e[k1]), float(e[k0] - e_min), t_td


# ---------------------------------------------------------------------------------
# FAB-SAF-009 hold-up margin
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-SAF-009")
@pytest.mark.parametrize("cls", CLASSES)
def test_holdup_margin(runs, cls):
    """SAF-009: hold-up energy ≥ 3× the landing need; DRAWER open hover ≈ 5 min (≈ 490 J usable at 1.5 W).

    Contract: (A) the usable supercap energy equals η·½·C·(V_max² − V_min²) ((E) rel
    1e-12) and is ≥ 3× ``derive::holdup_energy`` need in every class; for DRAWER it is
    ≈ 490 J (±2 %) and lasts ≈ 5 min at the 1.5 W hover budget (4–6 min, P). (S) a mains
    loss while levitated (catalog ``power_loss``) spends at most a third of the usable
    energy at the loss up to 0.5 s after touchdown; the simulated spend is reported
    against the analytic need.
    """
    p = _params(cls)
    der = p.derived()
    s = p.get
    avail = (
        s("design.power.converter_eff")
        * 0.5
        * s("design.power.supercap_c_f")
        * (s("design.power.supercap_v_max_v") ** 2 - s("design.power.supercap_v_min_v") ** 2)
    )
    assert abs(der["holdup_avail_j"] / avail - 1.0) <= 1e-12, f"(E) avail {der['holdup_avail_j']} vs {avail}"
    assert der["holdup_avail_j"] >= s("control.supervisor.holdup_margin") * der["holdup_need_j"], (
        f"(A) avail {der['holdup_avail_j']:.1f} J < 3 × need {der['holdup_need_j']:.1f} J"
    )
    if cls == "drawer":
        assert abs(der["holdup_avail_j"] / 490.0 - 1.0) <= 0.02, (
            f"(A) ≈ 490 J usable, SPEC FAB-SAF-009: {der['holdup_avail_j']:.1f} J"
        )
        t_hover = der["holdup_avail_j"] / s("design.power.p_hover_budget_w")
        assert 240.0 <= t_hover <= 360.0, f"(P) open hover ≈ 5 min at 1.5 W: {t_hover / 60:.2f} min"

    fid = "heave1" if cls == "heavy" else "carriage6"

    def edit(d, series, p_):
        d["record"]["channels"] = _LOSS_CHANNELS
        d["metrics"]["windows"] = {}

    r, pp, d = runs.get(f"loss:{cls}", _catalog("power_loss", cls, fidelity=fid, edit=edit))
    spent, usable, _ = _loss_spent(r, pp, d)
    assert 3.0 * spent <= usable, (
        f"(S) hold-up ≥ 3× the landing need, SPEC FAB-SAF-009: {cls} spent {spent:.1f} J of {usable:.1f} J usable "
        f"(analytic need {der['holdup_need_j']:.1f} J; faults {_faults(r)})"
    )


# ---------------------------------------------------------------------------------
# FAB-PWR-002 hover / sleep power, FAB-PWR-003 zero-power copper loss
# ---------------------------------------------------------------------------------


def _hover(runs: _Runs, cls: str):
    fid = "heave1" if cls == "heavy" else "carriage6"

    def edit(d, series, p):
        d["metrics"]["windows"] = {}

    return runs.get(f"hover:{cls}", _catalog("hover", cls, fidelity=fid, edit=edit))


def _rest(runs: _Runs, cls: str):
    """The class at rest in its low-power state: DRAWER seated, DOOR landed (``land``)."""

    def edit(d, series, p):
        if cls == "drawer":
            d["initial"] = {"lev": "seated", "x_m": p.get("design.rail.x_closed_m")}
            d["events"] = []
        d["duration_s"] = 6.0
        d["metrics"]["windows"] = {}

    name = "piston_pulse" if cls == "drawer" else "land"
    return runs.get(f"rest:{cls}", _catalog(name, cls, edit=edit))


@pytest.mark.req("FAB-PWR-002")
@pytest.mark.parametrize("cls", ["drawer", "door"])
def test_hover_power_budget(runs, cls):
    """PWR-002: hover (coil + declared electronics) ≤ 1.5 W open DRAWER, ≤ 3 W DOOR; sleep ≤ 0.3 / 0.5 W.

    Contract: the budgets equal SPECIFICATION §2 ((E) parameters); levitated idle
    (catalog ``hover``, DRAWER open at mid-travel) the zero-power copper power
    (``hover_power_w``) plus the declared electronics power, and the mean simulated bus
    power over [2 s, end], are within the hover budget; at rest in the low-power state
    (DRAWER seated, DOOR landed) the mean bus power over the last 2 s is within the
    sleep budget.
    """
    hover_b, sleep_b = {"drawer": (1.5, 0.3), "door": (3.0, 0.5)}[cls]
    r, p, _ = _hover(runs, cls)
    assert p.get("design.power.p_hover_budget_w") == hover_b and p.get("design.power.p_sleep_budget_w") == sleep_b, (
        "(E) budgets, SPEC §2"
    )
    p_cu = r.kpis["hover_power_w"]
    p_el = p.get("design.power.p_electronics_w")
    assert p_cu + p_el <= hover_b, f"(S) ≤ {hover_b} W, SPEC FAB-PWR-002: copper {p_cu:.4f} W + electronics {p_el} W"
    p_bus = float(r["p_bus_w"][r.t_s >= 2.0].mean())
    assert p_bus <= hover_b, f"(S) ≤ {hover_b} W, SPEC FAB-PWR-002: simulated bus power {p_bus:.3f} W"
    r2, _, _ = _rest(runs, cls)
    p_rest = float(r2["p_bus_w"][r2.t_s >= r2.t_s[-1] - 2.0].mean())
    state = int(r2["lev_state"][-1])
    assert p_rest <= sleep_b, (
        f"(S) sleep ≤ {sleep_b} W, SPEC FAB-PWR-002: {cls} at rest (lev_state {state}) draws {p_rest:.3f} W "
        f"(declared electronics {p_el} W)"
    )


@pytest.mark.req("FAB-PWR-003")
@pytest.mark.parametrize("cls", CLASSES)
def test_zero_power_copper_loss(runs, cls):
    """PWR-003: zero-power copper loss inside the float band ≤ 5 % of the hover budget.

    Contract: levitated idle at m* (inside the band), the coil + LSM copper power
    (``hover_power_w`` KPI, plant truth ledger) is at most 5 % of
    ``design.power.p_hover_budget_w``; the mean lift gap is inside the band. HEAVY on
    Heave1 (D1).
    """
    r, p, _ = _hover(runs, cls)
    lo, hi = p.get("design.suspension.zp_band_m.0"), p.get("design.suspension.zp_band_m.1")
    g = r.kpis["hover_gap_mean_m"]
    assert lo <= g <= hi, f"precondition: hover gap {g * 1e3:.3f} mm outside the band"
    budget = p.get("design.power.p_hover_budget_w")
    p_cu = r.kpis["hover_power_w"]
    assert p_cu <= 0.05 * budget, f"(S) ≤ 5 % of {budget} W, SPEC FAB-PWR-003: copper {p_cu:.4f} W ({100 * p_cu / budget:.2f} %)"


# ---------------------------------------------------------------------------------
# FAB-PWR-004 dock energy
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-PWR-004")
@pytest.mark.parametrize("cls", ["wall", "heavy"])
def test_dock_energy_margin(runs, cls):
    """PWR-004: WALL/HEAVY dock energy ≥ 3× (move + landing), analytically plus a short simulation.

    Contract: (A) the usable supercap energy is ≥ 3× the ``derive::holdup_energy`` need,
    which covers stopping a move in progress (electronics over the braking time; braking
    regenerates) plus the landing transfer; (S) mains lost while moving (catalog
    ``power_loss`` with ``moving``): the supercap energy spent from the loss to 0.5 s
    after touchdown is at most a third of the usable energy at the loss. WALL on
    Carriage6, HEAVY on Travel1d (D1).
    """
    p = _params(cls)
    der = p.derived()
    assert der["holdup_avail_j"] >= 3.0 * der["holdup_need_j"], (
        f"(A) dock energy {der['holdup_avail_j']:.1f} J < 3 × (move + landing) {der['holdup_need_j']:.1f} J"
    )
    fid = "travel1d" if cls == "heavy" else "carriage6"

    def edit(d, series, p_):
        d["record"]["channels"] = _LOSS_CHANNELS
        d["metrics"]["windows"] = {}

    r, pp, d = runs.get(f"loss_moving:{cls}", _catalog("power_loss", cls, fidelity=fid, options={"moving": True}, edit=edit))
    t_loss = next(float(e["t_s"]) for e in d["events"] if e["kind"] == "power_loss")
    assert np.abs(r["v_mps"][int(np.searchsorted(r.t_s, t_loss))]) > 1e-3, "precondition: moving at the loss"
    spent, usable, _ = _loss_spent(r, pp, d)
    assert 3.0 * spent <= usable, (
        f"(S) ≥ 3× (move + landing), SPEC FAB-PWR-004: {cls} spent {spent:.1f} J of {usable:.1f} J usable "
        f"(analytic need {der['holdup_need_j']:.1f} J)"
    )


# ---------------------------------------------------------------------------------
# FAB-PWR-006 cycle energy report
# ---------------------------------------------------------------------------------


def _cycle_edit(d: dict[str, Any], series: dict[str, bytes], p: Any) -> None:
    x0, travel = p.get("design.rail.x_closed_m"), p.get("design.rail.travel_m")
    x1 = x0 + min(travel, 0.5)
    d["initial"] = {"lev": "levitated", "x_m": x0}
    d["events"] = [e for e in d["events"] if e["kind"] == "hold_to_run"] + [
        {"kind": "command", "t_s": 0.5, "cmd": "go_to", "x_m": x1},
        {"kind": "command", "t_s": 0.5 + 0.5 * d["duration_s"], "cmd": "close"},
    ]
    d["duration_s"] = 2.0 * d["duration_s"]
    d["record"]["channels"] = ["x_m", "v_mps", "p_bus_w", "e_kin_j", "e_drag_j", "e_hold_j", "e_kin_gov_j", "demand", "f_lsm_n"]
    d["metrics"]["windows"] = {}


@pytest.mark.req("FAB-PWR-006")
@pytest.mark.parametrize("cls", ["drawer", "door", "wall"])
def test_cycle_energy_report(runs, cls):
    """PWR-006: each open/close cycle reports its energy (kinetic, copper, standby).

    Contract: for a levitated open-and-close cycle, the run's energy report
    (``fabrail.analysis.analyze_safety(result).energy_ledger_j``, the Python analysis
    boundary) has finite, non-negative kinetic, copper and standby entries; the kinetic
    entry matches ½·m·v_max² of the truth (±1 %) and the standby entry the declared
    electronics power over the run (±5 %). The ledger keys are matched by substring so
    naming stays with the analysis owner.
    """
    from fabrail import analysis

    r, p, _ = runs.get(f"cycle:{cls}", _catalog("terminus_soft_close", cls, edit=_cycle_edit))
    assert r["x_m"].max() - r["x_m"][0] > 0.1, "the cycle did not open"
    ledger = analysis.analyze_safety(r).energy_ledger_j
    missing = [k for k in ("kinetic", "copper", "standby") if not any(k in name for name in ledger)]
    assert not missing, f"(S) SPEC FAB-PWR-006: the cycle energy report lacks {missing} (ledger keys {sorted(ledger)})"
    for name, val in ledger.items():
        assert math.isfinite(val) and val >= 0.0, f"ledger {name} = {val}"
    kin = (
        next(v for k, v in ledger.items() if "kinetic" in k and "max" in k)
        if any("kinetic" in k and "max" in k for k in ledger)
        else next(v for k, v in ledger.items() if "kinetic" in k)
    )
    m = p.get("design.carriage.m_star_kg")
    ref = 0.5 * m * float(np.abs(r["v_mps"]).max()) ** 2
    assert abs(kin / ref - 1.0) <= 0.01, f"(S) kinetic {kin:.4f} J vs ½·m·v_max² {ref:.4f} J"
    standby = next(v for k, v in ledger.items() if "standby" in k)
    ref_sb = p.get("design.power.p_electronics_w") * r.duration_s
    assert abs(standby / ref_sb - 1.0) <= 0.05, f"(S) standby {standby:.2f} J vs {ref_sb:.2f} J"
