"""Fail direction after a control loss (FAB-LEV-019; SAFETY.md H-03/H-04).

A control loss here is the physics of ``derive::impact_speed``: every coil driver dead
(current → 0 through the current-loop lag), so only the permanent magnets act and the
carriage leaves the unstable zero-power equilibrium at g*. The zero-power equilibrium is
exactly balanced, so a ±0.1 % payload change applied at the same instant selects the
fail direction (up: toward the upper stop; down: onto the travel rollers). The closed
form is evaluated in Python for that truth mass (PHYSICS §15; checked (E) against
``_native.impact_speed`` at m*).

Tier: Heave1 in lag coil mode without eddy lag (the closed form's "no lag, no
dissipation" heave problem), recorded at 20 kHz. DRAWER runs at mid-travel (rollers
below, elastomer upper rollers above); the re-lift test runs on Carriage6 in voltage
mode. Owner: B4 (Wave 2).
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

CLASSES = ("drawer", "door", "wall", "heavy")
#: μ₀, H/m (CODATA 2018; fabrail_params::derive::MU0_H_PER_M).
MU0 = 1.256_637_062_12e-6
#: A separation counts as a bounce when the carriage rises more than this off the stop
#: (above the contact-compliance and gap-sensor-noise scale), m.
BOUNCE_MIN_M = 10e-6
#: Standard gravity, m/s² (fabrail_params::derive::G0_MPS2).
G0 = 9.80665
#: ``contact_state`` bits (fabrail-sim recorder): levitated, seated, landed, upper stop.
CS_SEATED, CS_LANDED, CS_UPPER = 2, 4, 8
#: ``lev_state`` wire value of LevitatedIdle (fabrail-types LevState).
LEV_LEVITATED_IDLE = 5
#: Lift/land failure codes (fabrail-types fault.rs).
LIFT_FAULTS = {0x0502: "LiftTimeout", 0x0503: "LandTimeout", 0x0506: "SeatFailed"}
#: Relative payload nudge selecting the fail direction.
NUDGE = 1.0e-3


class _Runs:
    """Module-level run cache: key → (RunResult, params, spec dict)."""

    def __init__(self, native: Any) -> None:
        self._n = native
        self._cache: dict[str, tuple[RunResult, Any, dict[str, Any]]] = {}

    def get(self, key: str, factory: Callable[[], tuple[Any, dict[str, Any], dict[str, bytes]]]):
        if key not in self._cache:
            p, d, series = factory()
            scenarios.validate_spec(d)
            scenarios.cross_check(d, series, p)
            sc = self._n.Scenario.from_json(scenarios.to_json(d), series)
            self._cache[key] = (RunResult.from_native(self._n.run(p, sc)), p, d)
        return self._cache[key]


@pytest.fixture(scope="module")
def runs(native: Any) -> _Runs:
    return _Runs(native)


def _n_lift(p: Any) -> int:
    doc = json.loads(p.to_json())["design"]["suspension"]
    return sum(1 for s in doc["stations"][: doc["n_magnets"]] if s["has_pm"] and s["normal_sign"] > 0)


def _impact_speed_cf(p: Any, direction: str, m_kg: float, g0_m: float) -> float:
    """PM-only impact speed from rest at g0 (PHYSICS §15, derive::impact_speed_at), m/s."""
    der = p.derived()
    hp, ap, mpm = der["h_prime_m"], der["pole_area_m2"], der["m_pm_at"]
    k = 0.5 * _n_lift(p) * MU0 * ap * mpm * mpm

    def work(ga: float, gb: float) -> float:
        return k * (1.0 / (2.0 * ga + hp) - 1.0 / (2.0 * gb + hp))

    w = m_kg * G0
    g_up, g_land = p.get("design.suspension.g_up_m"), p.get("design.suspension.g_land_m")
    e = work(g_up, g0_m) - w * (g0_m - g_up) if direction == "up" else w * (g_land - g0_m) - work(g0_m, g_land)
    return math.sqrt(2.0 * e / m_kg) if e > 0.0 else 0.0


def _control_loss(cls: str, direction: str) -> Callable[[], tuple[Any, dict[str, Any], dict[str, bytes]]]:
    def factory():
        cfg = RunConfig(rail_class=cls, coil="lag")
        p = cfg.params()
        d, series = scenarios.spec("control_loss_" + direction, p, cfg, fidelity="heave1")
        nm = int(p.get("design.suspension.n_magnets"))
        m_star = p.get("design.carriage.m_star_kg")
        t_f = 0.5
        nudge = (1.0 if direction == "down" else -1.0) * NUDGE * m_star
        d["events"] = [{"kind": "payload", "t_s": t_f, "delta_kg": nudge}] + [
            {"kind": "fault", "t_s": t_f, "what": "coil_driver", "ch": i} for i in range(nm)
        ]
        d["duration_s"] = 2.0
        d["fidelity"]["eddy_lag"] = False
        d["record"]["rate_hz"] = 20_000.0
        d["record"]["channels"] = ["z_m", "contact_state", "f_upper_n", "f_rollers_n", "gap_0_m"]
        d["metrics"]["windows"] = {}
        return p, d, series

    return factory


def _contact_bit(direction: str) -> int:
    return CS_UPPER if direction == "up" else (CS_LANDED | CS_SEATED)


def _impacts(r: RunResult, direction: str, t_f: float) -> tuple[np.ndarray, np.ndarray]:
    """(contact mask in the fail direction, onset indices after t_f)."""
    c = (r["contact_state"].astype(np.int64) & _contact_bit(direction)) != 0
    onsets = np.flatnonzero(c[1:] & ~c[:-1]) + 1
    return c, onsets[r.t_s[onsets] >= t_f]


@pytest.mark.req("FAB-LEV-019")
@pytest.mark.parametrize("direction", ["up", "down"])
@pytest.mark.parametrize("cls", CLASSES)
def test_impact_speeds(native, runs, cls, direction):
    """LEV-019: the control-loss impact speed equals ``derive::impact_speed`` ±5 % with ≤ 2 bounces.

    Contract: the Python closed form reproduces ``_native.impact_speed`` at m* ((E) rel
    1e-9); on Heave1 with every coil driver dead the heave speed at the first contact in
    the fail direction equals the closed form for the truth mass and start gap within
    ±5 %; at most 2 bounces (separations rising > 10 µm off the stop) follow within 1 s.
    """
    r, p, d = runs.get(f"loss:{cls}:{direction}", _control_loss(cls, direction))
    ref_star = native.impact_speed(p, direction)
    mine = _impact_speed_cf(p, direction, p.get("design.carriage.m_star_kg"), p.get("design.suspension.g_star_m"))
    assert abs(mine / ref_star - 1.0) <= 1e-9, f"(E) rel 1e-9, PHYSICS §15: python {mine} vs native {ref_star}"

    t_f = float(d["events"][0]["t_s"])
    k_f = int(np.searchsorted(r.t_s, t_f))
    g0 = p.get("design.suspension.g_star_m") - float(r["z_m"][k_f - 1])
    m = p.get("design.carriage.m_star_kg") * (1.0 + (NUDGE if direction == "down" else -NUDGE))
    v_cf = _impact_speed_cf(p, direction, m, g0)
    _, onsets = _impacts(r, direction, t_f)
    assert onsets.size, f"{cls}: no {direction} contact after the control loss"
    k = int(onsets[0])
    vz = np.gradient(r["z_m"], 1.0 / r.rate_hz)
    v_imp = float(abs(vz[k - 1]))
    assert abs(v_imp / v_cf - 1.0) <= 0.05, (
        f"(S) ±5 %, SPEC FAB-LEV-019: {cls} {direction} impact {v_imp * 1e3:.2f} mm/s vs closed form {v_cf * 1e3:.2f} mm/s"
    )
    c, _ = _impacts(r, direction, t_f)
    z = r["z_m"]
    heights = []
    for a in np.flatnonzero(~c[1:] & c[:-1]) + 1:
        if r.t_s[a] > r.t_s[k] + 1.0:
            break
        nxt = onsets[onsets > a]
        b = int(nxt[0]) if nxt.size else z.size
        heights.append(float(np.abs(z[a:b] - z[a]).max()))
    bounces = [h for h in heights if h > BOUNCE_MIN_M]
    assert len(bounces) <= 2, (
        f"(S) ≤ 2 bounces, SPEC FAB-LEV-019: {cls} {direction} {len(bounces)} bounces within 1 s, heights "
        f"{[round(h * 1e6, 1) for h in bounces]} µm"
    )


@pytest.mark.req("FAB-LEV-019")
@pytest.mark.parametrize("direction", ["up", "down"])
@pytest.mark.parametrize("cls", CLASSES)
def test_rests_on_rollers(runs, cls, direction):
    """LEV-019 / H-03, H-04: either fail direction ends at rest on a designed support.

    Contract: over the last 0.3 s of the run (≥ 1 s after the loss) the carriage is in
    continuous contact with the support in its fail direction (down: travel rollers or
    seat; up: the upper stop, a designed sliding surface or the DRAWER's elastomer
    upper rollers) and at rest (|ż| < 1 mm/s); the support load stays within its
    rating (upper stop ≤ stop ratio·m*·g, DRAWER upper rollers ≤ their rating).
    """
    r, p, _ = runs.get(f"loss:{cls}:{direction}", _control_loss(cls, direction))
    c, _ = _impacts(r, direction, 0.0)
    tail = r.t_s >= r.t_s[-1] - 0.3
    assert c[tail].all(), f"{cls} {direction}: not resting on its {direction} support at the end (contact {c[tail].mean():.2f})"
    vz = np.gradient(r["z_m"], 1.0 / r.rate_hz)
    assert np.abs(vz[tail]).max() < 1e-3, f"{cls} {direction}: still moving, |ż| {np.abs(vz[tail]).max() * 1e3:.2f} mm/s"
    if direction == "up":
        f_up = float(r["f_upper_n"][tail].mean())
        m_star = p.get("design.carriage.m_star_kg")
        limit = p.get("design.landing.upper_roller_rating_n") if cls == "drawer" else 1.30 * m_star * G0
        assert f_up <= limit, f"(A) SPEC FAB-LEV-006: upper support load {f_up:.1f} N above {limit:.1f} N"


def _relift(mass_kg: float) -> Callable[[], tuple[Any, dict[str, Any], dict[str, bytes]]]:
    def factory():
        cfg = RunConfig(rail_class="drawer", coil="voltage")
        p = cfg.params()
        d, series = scenarios.spec("hover", p, cfg)
        x = p.get("design.rail.x_closed_m") + 0.5 * p.get("design.rail.travel_m")
        d["initial"] = {"lev": "landed", "x_m": x, "payload_kg": mass_kg - p.get("design.carriage.m_star_kg")}
        d["events"] = [{"kind": "command", "t_s": 0.5, "cmd": "lift"}]
        d["duration_s"] = 3.0
        d["metrics"]["windows"] = {}
        return p, d, series

    return factory


@pytest.mark.req("FAB-LEV-019")
@pytest.mark.parametrize("mass_kg", [18.0, 20.0, 26.0])
def test_drawer_relift_limit(native, runs, mass_kg):
    """LEV-019: a DRAWER that failed down onto its rollers re-lifts only up to 20 kg (i_pulse 12 A).

    Contract: closed form: the lift current from the 6.5 mm rollers is ≤ i_pulse at
    20 kg and above it from 21 kg (the anchor 27.45 A at 48 kg, (E) to 0.05 A). In
    simulation (Carriage6, voltage mode, landed mid-travel, ``lift``): ≤ 20 kg ends
    levitated (LevitatedIdle, no roller contact); above 20 kg it stays on the rollers,
    raises a lift/land fault and de-energises the coils (|i| < 0.1 A at the end), leaving
    a manual push to re-seat.
    """
    p = RunConfig(rail_class="drawer", coil="voltage").params()
    g_land, i_pulse = p.get("design.suspension.g_land_m"), p.get("design.suspension.i_pulse_a")
    assert native.bias_current(p, g_land, 20.0) <= i_pulse, "(A) 20 kg must be re-liftable within i_pulse"
    assert native.bias_current(p, g_land, 21.0) > i_pulse, "(A) 21 kg must exceed i_pulse"
    assert abs(native.bias_current(p, g_land, 48.0) - 27.45) <= 0.05, "(E) anchor 27.45 A at 48 kg from 6.5 mm"

    r, pp, _ = runs.get(f"relift:{mass_kg}", _relift(mass_kg))
    faults = {int(e.a) for e in r.events if e.kind == "FaultRaised"}
    tail = r.t_s >= r.t_s[-1] - 0.3
    if mass_kg <= 20.0:
        need = native.bias_current(pp, g_land, mass_kg)
        assert int(r["lev_state"][-1]) == LEV_LEVITATED_IDLE and r["f_rollers_n"][tail].max() == 0.0, (
            f"(S) SPEC FAB-LEV-019: {mass_kg} kg did not re-lift from the rollers (needs {need:.2f} A ≤ i_pulse "
            f"{i_pulse} A; peak coil {max(np.abs(r[f'coil_{i}_a']).max() for i in range(4)):.2f} A; faults "
            f"{[LIFT_FAULTS.get(c, hex(c)) for c in sorted(faults)]})"
        )
    else:
        assert r["f_rollers_n"][tail].min() > 0.0, f"{mass_kg} kg left the rollers"
        assert faults & set(LIFT_FAULTS), f"{mass_kg} kg: no lift/land fault flags the failed re-lift ({sorted(faults)})"
        i_end = max(np.abs(r[f"coil_{i}_a"][tail]).max() for i in range(4))
        assert i_end < 0.1, f"{mass_kg} kg: coils still energised at the end ({i_end:.2f} A)"
