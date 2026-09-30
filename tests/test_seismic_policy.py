"""Contract: seismic fusion and land-first class actions, FAB-SEI-007, -009, -010, -011 and -019.

Closed-loop runs through ``_native.run``/``run_batch`` on the catalog scenarios
(``fabrail.scenarios``; synthetic P+S records scaled so Pd follows Wu–Zhao at hypocentral
R, PHYSICS §19). Every network alert is synthetic/injected, not a live EEW feed. Tier:
Travel1d for all four classes (the seismic recommendation, plan §2b item 13); DRAWER and
DOOR are also run on Carriage6 (voltage coil mode) where the class action is asserted,
because they are the hero devices closed end-to-end in W1.5.

Plan core §4.3 scenarios covered here: 2 (M5 R10), 3 (M7 R20), 4 (M7.8 R100, slow),
5 and 6 (network alert, cancel), 9 (power loss at S arrival) and 12 (egress door). The
PROTECT rule is plan §2b item 6: Level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal; Level 1 means
freeze only. The M5 criterion is the §2b formulation: no LAND or LOCK in ≥ 95 % of seeds.

Test-harness choices (documented, not scenario changes):

- Where FREEZE must be observed, the builder's move command (0.5 s before t_P) is moved
  earlier by a class lead so the carriage is at cruise at the pick (the 5 t HEAVY breaks
  away at the 67 N static clamp and needs seconds), and HEAVY's 30 s idle hover timeout
  (``control.supervisor.hover_timeout_s``) is raised so it is still levitated after the
  30 s STA/LTA warm-up.
- ``t_p_s`` = 31 s shortens the pre-event record (the LTA needs 30 s) where noted.
- Lead decision D3 (Wave 2): the carriage's physical closed- and open-end stops bound
  floor-driven displacement, and the monitor escalates (SLP/SDI/UnintendedMotion) only
  while the drive injects energy; floor-driven motion with the LSM in STO inside the stops
  is reported, not a new trip. SEI-019 asserts this.

Owner: B5 (Wave 2).
"""

from __future__ import annotations

import functools
import math

import numpy as np
import pytest

from fabrail import config, ensemble, scenarios
from fabrail.analysis import wilson_upper
from fabrail.results import RunResult

CLASSES = ("drawer", "door", "wall", "heavy")

# Wire values of the frozen fabrail_types enums (crates/fabrail-types/src/state.rs, fault.rs; W0).
SEIS = {"normal": 0, "triggered": 1, "protect": 2, "shaking": 3, "quiet": 4, "inspect": 5}
LEV_FAULT_REACTION, LEV_FAULT = 10, 11
MODE_FREEZE, MODE_HOLD = 6, 1
ACT = {"none": 0, "freeze": 1, "center": 2, "land": 3, "lock": 4, "open_and_stop": 7}
DEMAND_SAFE_STOP = 2
#: Monitor faults that floor-driven motion must not raise once the drive is in STO (D3; fault.rs codes).
FLOOR_DRIVEN_TRIPS = {
    0x0601: "EnergyLimit",
    0x0602: "MomentumLimit",
    0x0604: "Overspeed",
    0x0605: "PositionLimit",
    0x0606: "UnintendedMotion",
}
#: contact_state bits (fabrail-sim recorder): levitated, seated, landed, upper_stop, locked, latched.
C_SEATED, C_LANDED, C_LOCKED = 1 << 1, 1 << 2, 1 << 4

#: Class landing action of the core land-first table (core design §C.2; SEISMIC §2.5; policy.rs).
LANDING_ACTION = {"drawer": ACT["land"], "door": ACT["land"], "wall": ACT["lock"], "heavy": ACT["lock"]}
#: Lead of the move command before t_P so the carriage cruises at the pick, s (see the module docstring).
MOVE_LEAD_S = {"drawer": 0.5, "door": 0.8, "wall": 1.0, "heavy": 5.0}
HEAVY_HOVER = {"control.supervisor.hover_timeout_s": 300.0}

#: IBC 2021 §1010.3.3 horizontal sliding egress doors: ≤ 30 lbf (133 N) to set in motion, ≤ 15 lbf (67 N) to
#: operate. The static manual-force bound (rollers without PM relief, derive::manual_force) is checked against the
#: stricter 67 N. SAFETY.md SF-07 declares ≤ 22.2 N (5 lbf) manual force for the DOOR.
IBC_OPERATE_N, DOOR_MANUAL_N = 67.0, 22.2
#: Captive envelope: the bumper faces sit at the outer edges of the 150 mm end zones beyond the travel
#: (SAF-014; plant contact::bumper_penetration); 10 mm allows bumper compression.
CAPTIVE_SLACK_M = 0.010
#: A seismic opening moves the door a large part of its travel; floor-driven sliding of the landed door on its
#: rollers under the S wave stays within a few centimetres. Displacement beyond this counts as "opened", m.
OPENING_M = 0.05

BASE_CHANNELS = [
    "x_m",
    "v_mps",
    "seismic_state",
    "lev_state",
    "motion_mode",
    "seismic_action",
    "demand",
    "level",
    "contact_state",
    "f_lsm_n",
]


# ---------------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------------


def _job(
    native,
    cls,
    name,
    *,
    fidelity=None,
    options=None,
    overrides=None,
    duration_s=None,
    member=None,
    mutate=None,
    channels=None,
    rate_hz=None,
):
    """(params, spec dict, native Scenario) for one catalog scenario, voltage coil mode, seed 42."""
    over = dict(overrides or {})
    if cls == "heavy":
        over = {**HEAVY_HOVER, **over}
    cfg = config.RunConfig(rail_class=cls, seed=42, coil="voltage", overrides=over, duration_s=duration_s)
    p = cfg.params()
    d, series = scenarios.spec(name, p, cfg, fidelity=fidelity, member=member, options=options)
    d["record"]["channels"] = list(channels or BASE_CHANNELS)
    if rate_hz is not None:
        d["record"]["rate_hz"] = float(rate_hz)
    if mutate is not None:
        mutate(d)
    return p, d, native.Scenario.from_json(scenarios.to_json(d), series)


def _run_all(native, jobs):
    return [RunResult.from_native(r) for r in native.run_batch([(p, sc) for p, _, sc in jobs], None)]


def _lead_moves(cls):
    """Move the builder's hold-to-run/move commands to t_P − MOVE_LEAD_S[cls] (cruise at the pick)."""

    def mutate(d):
        t_p = float(scenarios.info(d)["t_p_s"])
        t_cmd = t_p - MOVE_LEAD_S[cls]
        for e in d["events"]:
            if e["kind"] == "hold_to_run":
                e["t_s"] = t_cmd - 0.1
            elif e["kind"] == "command":
                e["t_s"] = t_cmd
        d["events"].sort(key=lambda e: e["t_s"])

    return mutate


def _first(r, kind, pred=lambda e: True):
    return next((e for e in r.events if e.kind == kind and pred(e)), None)


def _faults(r, t0=-math.inf, t1=math.inf):
    return [(round(e.t_s, 3), int(e.a)) for e in r.events if e.kind == "FaultRaised" and t0 <= e.t_s < t1]


def _landed_at(r, t0=-math.inf):
    """First time ≥ t0 the carriage rests on its rollers or seat (plant contact state)."""
    idx = np.nonzero((r.t_s >= t0) & ((r["contact_state"] & (C_LANDED | C_SEATED)) != 0))[0]
    return float(r.t_s[idx[0]]) if idx.size else math.nan


def _state_time(r, state):
    idx = np.nonzero(r["seismic_state"] >= state)[0]
    return float(r.t_s[idx[0]]) if idx.size else math.nan


# ---------------------------------------------------------------------------------
# SEI-007 (scenarios 5 and 6)
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-SEI-007")
def test_network_alert_eta_and_cancel(native, record_property):
    """Contract: a synthetic network alert before the local P gives an S-ETA within ±1 s of the true S arrival, the
    class action starts before t_P, and a cancel returns the seismic state to NORMAL (FAB-SEI-007; scenarios 5, 6)."""
    jobs = [_job(native, c, "network_alert", rate_hz=100.0) for c in CLASSES]
    jobs += [_job(native, c, "alert_cancel", rate_hz=100.0) for c in CLASSES]
    runs = _run_all(native, jobs)
    for (_p, d, _), r, cls in zip(jobs[:4], runs[:4], CLASSES, strict=True):
        info = scenarios.info(d)
        t_alert, t_p, t_s = float(info["t_alert_s"]), float(info["t_p_s"]), float(info["t_s_s"])
        prot = _first(r, "Protect")
        assert prot is not None, f"{cls}: no PROTECT on the network alert at {t_alert:.2f} s (SEI-007)"
        eta = prot.t_s + prot.b
        record_property(f"{cls}_eta_error_s", round(eta - t_s, 3))
        assert abs(eta - t_s) <= 1.0, f"{cls}: S-ETA {eta:.2f} s vs true t_S {t_s:.2f} s, required ±1 s (SEI-007; SEISMIC §2.4)"
        assert t_alert <= prot.t_s <= t_alert + 0.1, f"{cls}: PROTECT at {prot.t_s:.3f} s, alert at {t_alert:.3f} s (≤ 100 ms)"
        acted = min(e.t_s for e in r.events if e.kind in ("Freeze", "Protect"))
        assert acted < t_p, f"{cls}: class action started at {acted:.2f} s, not before t_P {t_p:.2f} s (SEI-007)"
    for (_p, d, _), r, cls in zip(jobs[4:], runs[4:], CLASSES, strict=True):
        alert = next(e for e in d["events"] if e["kind"] == "network_alert")
        cancel = next(e for e in d["events"] if e["kind"] == "alert_cancel")
        s = r["seismic_state"]
        during = (r.t_s >= alert["t_s"] + 0.1) & (r.t_s < cancel["t_s"])
        after = r.t_s >= cancel["t_s"] + 0.1
        assert np.all(s[during] >= SEIS["protect"]), f"{cls}: the alert must hold PROTECT until the cancel (SEI-007)"
        stuck = sorted(set(int(v) for v in s[after]) - {SEIS["normal"]})
        assert not stuck, (
            f"{cls}: seismic state after the cancel must return to and stay NORMAL, saw states {stuck} (SEI-007/008)"
        )


# ---------------------------------------------------------------------------------
# M7 R20 runs shared by SEI-009 and SEI-010 (scenario 3)
# ---------------------------------------------------------------------------------

#: (class, tier) of the shared M7 R20 runs: all classes on Travel1d, the hero devices also on Carriage6.
M7_CASES = (
    ("drawer", "travel1d"),
    ("door", "travel1d"),
    ("wall", "travel1d"),
    ("heavy", "travel1d"),
    ("drawer", "carriage6"),
    ("door", "carriage6"),
)
M7_TP_S = 31.0


@functools.lru_cache(maxsize=1)
def _m7_runs(native):
    jobs = []
    for cls, tier in M7_CASES:
        dur = M7_TP_S + 2.7 + 6.0  # past t_S (S−P 2.66 s at R_hyp 22.4 km) and the landing/lock
        jobs.append(
            _job(native, cls, "pwave_m7_r20", fidelity=tier, options={"t_p_s": M7_TP_S}, duration_s=dur, mutate=_lead_moves(cls))
        )
    return list(zip(M7_CASES, jobs, _run_all(native, jobs), strict=True))


@pytest.mark.req("FAB-SEI-009")
def test_freeze_latency(native, record_property):
    """Contract: a local P pick reaches FREEZE — the freeze command, then deceleration at a_brk — within ≤ 100 ms:
    the carriage is commanded to freeze ≤ 100 ms after the pick and stops no later than 100 ms + 1.1·v0/a_brk after
    it (FAB-SEI-009; DRAWER/DOOR on Carriage6, WALL/HEAVY on Travel1d)."""
    wanted = {("drawer", "carriage6"), ("door", "carriage6"), ("wall", "travel1d"), ("heavy", "travel1d")}
    bad = []
    for (cls, tier), (p, d, _), r in _m7_runs(native):
        if (cls, tier) not in wanted:
            continue
        t_pick = r.kpis.get("trigger_t_s")
        t_p = float(scenarios.info(d)["t_p_s"])
        if t_pick is None or not (t_p <= t_pick <= t_p + 0.5):
            bad.append(f"{cls}/{tier}: no local pick within 0.5 s of t_P {t_p:.2f} s (trigger_t_s={t_pick})")
            continue
        v0 = abs(r.at("v_mps", t_pick))
        v_move = float(np.max(np.abs(r["v_mps"][(r.t_s > t_p - MOVE_LEAD_S[cls]) & (r.t_s < t_pick)])))
        if v0 < 0.5 * v_move or v0 < 1e-3:
            bad.append(
                f"{cls}/{tier}: not cruising at the pick (|v| {v0 * 1e3:.2f} mm/s, move peak {v_move * 1e3:.2f} mm/s); "
                f"faults raised (t, code) {_faults(r)}"
            )
            continue
        cmd = next(
            (e for e in r.events if e.kind == "Mode" and e.t_s >= t_pick - 1e-9 and int(e.a) in (MODE_FREEZE, MODE_HOLD)), None
        )
        a_brk = p.get("control.motion.a_brk_ratio") * p.get("control.motion.a_max_mps2")
        after = (r.t_s >= t_pick) & (np.abs(r["v_mps"]) <= max(1e-3, 0.02 * v0))
        t_stop = float(r.t_s[np.nonzero(after)[0][0]]) if after.any() else math.inf
        budget = 0.100 + 1.1 * v0 / a_brk
        record_property(f"{cls}_{tier}_freeze_cmd_ms", None if cmd is None else round((cmd.t_s - t_pick) * 1e3, 1))
        record_property(f"{cls}_{tier}_stop_after_pick_s", round(t_stop - t_pick, 3))
        if cmd is None or cmd.t_s - t_pick > 0.100:
            lat = None if cmd is None else round((cmd.t_s - t_pick) * 1e3, 1)
            bad.append(f"{cls}/{tier}: freeze command {lat} ms after the pick (≤ 100 ms)")
        if t_stop - t_pick > budget:
            bad.append(
                f"{cls}/{tier}: stopped {t_stop - t_pick:.3f} s after the pick from {v0:.4f} m/s; ≤ {budget:.3f} s "
                f"(100 ms + 1.1·v0/a_brk, a_brk {a_brk} m/s²)"
            )
    assert not bad, f"SEI-009 (pick → FREEZE ≤ 100 ms, deceleration at a_brk): {bad}"


@pytest.mark.req("FAB-SEI-010")
def test_class_actions(native, record_property):
    """Contract: on an M7 R20 event each class takes its land-first action — FREEZE, then DRAWER/DOOR land (DOOR
    never locks), WALL/HEAVY land and lock — and ends at rest in contact without a fault reaction or a monitor
    stop (FAB-SEI-010; core design §C.2, SEISMIC §2.5)."""
    bad = []
    for (cls, tier), (_p, d, _), r in _m7_runs(native):
        tag = f"{cls}/{tier}"
        acts = set(int(a) for a in r["seismic_action"])
        allowed = {ACT["none"], ACT["freeze"], ACT["center"], LANDING_ACTION[cls]}
        if not acts <= allowed:
            bad.append(f"{tag}: actions {sorted(acts)} outside the land-first set {sorted(allowed)}")
        if ACT["freeze"] not in acts or LANDING_ACTION[cls] not in acts:
            bad.append(f"{tag}: expected FREEZE then action {LANDING_ACTION[cls]}, saw {sorted(acts)}")
        final = int(r["contact_state"][-1])
        if not final & (C_LANDED | C_SEATED):
            bad.append(f"{tag}: not at rest in contact at the end (contact_state {final:#x})")
        locked = bool(final & C_LOCKED)
        if cls in ("wall", "heavy") and not locked:
            bad.append(f"{tag}: WALL/HEAVY must end landed and locked (contact_state {final:#x})")
        if cls == "door" and (locked or _first(r, "plant:locked") is not None):
            bad.append(f"{tag}: an egress DOOR must never lock (SEI-011)")
        fr = np.nonzero(np.isin(r["lev_state"], (LEV_FAULT_REACTION, LEV_FAULT)))[0]
        if fr.size:
            bad.append(f"{tag}: fault reaction at {r.t_s[fr[0]]:.3f} s; faults raised {_faults(r)}")
        if int(r["demand"].max()) >= DEMAND_SAFE_STOP:
            bad.append(
                f"{tag}: monitor demand {int(r['demand'].max())} (≥ SafeStop) during the class action; faults {_faults(r)}"
            )
        record_property(f"{tag}_landed_s_before_tS", round(float(scenarios.info(d)["t_s_s"]) - _landed_at(r, M7_TP_S), 3))
    assert not bad, f"SEI-010 land-first class actions: {bad}"


@pytest.mark.req("FAB-SEI-010")
def test_scenario_m7_r20(native, record_property):
    """Contract: M7 at R_epi 20 km (10 km deep) reaches L3, PROTECT at t_P + 1.2 ± 0.2 s, and the freeze completes
    before t_S; landing before t_S is reported, not required (FAB-SEI-010, scenario 3; Travel1d, all classes)."""
    bad = []
    for (cls, tier), (_p, d, _), r in _m7_runs(native):
        if tier != "travel1d":
            continue
        info = scenarios.info(d)
        t_p, t_s = float(info["t_p_s"]), float(info["t_s_s"])
        lv = r["level"]
        l3 = np.nonzero(lv >= 3)[0]
        t_l3 = float(r.t_s[l3[0]]) if l3.size else math.nan
        t_prot = _state_time(r, SEIS["protect"])
        frz = np.nonzero(r["motion_mode"] == MODE_FREEZE)[0]
        t_frz_end = float(r.t_s[frz[-1]]) if frz.size else math.nan
        record_property(f"{cls}_protect_minus_tP_s", round(t_prot - t_p, 3))
        record_property(f"{cls}_L3_minus_tP_s", round(t_l3 - t_p, 3))
        record_property(f"{cls}_landed_minus_tS_s", round(_landed_at(r, t_p) - t_s, 3))
        if not t_l3 < t_s:
            bad.append(f"{cls}: L3 at {t_l3:.2f} s, not before t_S {t_s:.2f} s (max level {int(lv.max())})")
        if not abs(t_prot - (t_p + 1.2)) <= 0.2:
            bad.append(f"{cls}: PROTECT at t_P + {t_prot - t_p:.3f} s, required 1.2 ± 0.2 s")
        if frz.size and not t_frz_end < t_s:
            bad.append(f"{cls}: freeze still running at {t_frz_end:.2f} s ≥ t_S {t_s:.2f} s")
        if abs(r.at("v_mps", t_s)) > 1e-3 and not _landed_at(r, t_p) < t_s:
            bad.append(f"{cls}: still moving at t_S ({r.at('v_mps', t_s):.4f} m/s)")
    assert not bad, f"SEI-010 scenario M7 R20: {bad}"


# ---------------------------------------------------------------------------------
# SEI-010 scenario 2 (M5 R10)
# ---------------------------------------------------------------------------------

M5_SEEDS = 40
M5_CLASSES = ("wall", "door")
M5_TP_S = 35.0  # the catalog P arrival (scenarios._T_P_S)


@pytest.mark.req("FAB-SEI-010")
def test_scenario_m5_r10(native, record_property):
    """Contract: an M5 at R_epi 10 km (10 km deep) stays at ≤ L1 and only freezes: no LAND or LOCK in ≥ 95 % of
    seeds (FAB-SEI-010 as formulated by plan §2b item 6; scenario 2; WALL and DOOR, 40 seeds each)."""
    members = ensemble.member_seeds(42, M5_SEEDS)
    jobs, tags = [], []
    for cls in M5_CLASSES:
        for m in members:
            jobs.append(
                _job(
                    native,
                    cls,
                    "pwave_m5_r10",
                    member=m,
                    duration_s=M5_TP_S + 4.5,  # catalog t_P; past S (S−P 1.68 s) and the 3 s Pd/τc window
                    rate_hz=100.0,
                    channels=["level", "seismic_action", "seismic_state"],
                )
            )
            tags.append((cls, m.index))
    runs = _run_all(native, jobs)
    msgs = []
    for cls in M5_CLASSES:
        rows = [(i, r) for (c, i), r in zip(tags, runs, strict=True) if c == cls]
        landlock = [i for i, r in rows if set(int(a) for a in r["seismic_action"]) & {ACT["land"], ACT["lock"]}]
        levels = np.bincount([int(r["level"].max()) for _, r in rows], minlength=4)
        protect = sum(1 for _, r in rows if (r["seismic_state"] >= SEIS["protect"]).any())
        frac_ok = 1.0 - len(landlock) / len(rows)
        record_property(f"{cls}_no_land_fraction", round(frac_ok, 4))
        record_property(f"{cls}_level_histogram_L0_L3", levels.tolist())
        record_property(f"{cls}_wilson95_upper_land", round(wilson_upper(len(landlock), len(rows)), 4))
        if frac_ok < 0.95:
            msgs.append(
                f"{cls}: {len(landlock)}/{len(rows)} seeds LAND/LOCK (no-land fraction {frac_ok:.1%} < 95 %), "
                f"{protect} reached PROTECT, level histogram L0..L3 {levels.tolist()}"
            )
    assert not msgs, f"SEI-010 M5 R10 (≤ L1, freeze only, ≥ 95 % of seeds; plan §2b item 6): {msgs}"


# ---------------------------------------------------------------------------------
# SEI-010 scenario 4 (M7.8 R100, slow)
# ---------------------------------------------------------------------------------


@pytest.mark.slow
@pytest.mark.req("FAB-SEI-010")
def test_scenario_m78_r100(native, record_property):
    """Contract: M7.8 at R_epi 100 km reaches ≥ L2 and every class is landed ≥ 8 s before t_S (FAB-SEI-010,
    scenario 4, slow tier; Travel1d, all classes)."""
    jobs = [_job(native, c, "pwave_m78_r100", rate_hz=100.0) for c in CLASSES]
    bad = []
    for (_p, d, _), r, cls in zip(jobs, _run_all(native, jobs), CLASSES, strict=True):
        info = scenarios.info(d)
        t_p, t_s = float(info["t_p_s"]), float(info["t_s_s"])
        lv = int(r["level"].max())
        t_land = _landed_at(r, t_p)
        record_property(f"{cls}_level_max", lv)
        record_property(f"{cls}_landed_before_tS_s", round(t_s - t_land, 3))
        if lv < 2:
            bad.append(f"{cls}: level {lv} < L2")
        if not t_s - t_land >= 8.0:
            bad.append(f"{cls}: landed {t_s - t_land:.2f} s before t_S (required ≥ 8 s); faults {_faults(r)}")
        record_property(f"{cls}_faults_raised", _faults(r))  # reported; fault-free class actions are SEI-010 class_actions
    assert not bad, f"SEI-010 M7.8 R100: {bad}"


# ---------------------------------------------------------------------------------
# SEI-011 (scenario 12)
# ---------------------------------------------------------------------------------


def _drop_moves(d):
    d["events"] = [e for e in d["events"] if e["kind"] not in ("command", "hold_to_run")]


@pytest.mark.req("FAB-SEI-011")
def test_egress_door_eew(native, record_property):
    """Contract: an egress DOOR never locks, a fire-rated door never opens (validate refuses the configuration),
    a non-rated door opens only on a purely local decision and only if it is at rest ≥ 1 s before t_S, and after
    the event the door stays manually operable: no ASC drag, not locked, manual force ≤ 22.2 N (FAB-SEI-011)."""
    # Configuration guards (params::validate): a fire-rated door can never be tuned to open, an egress door never to lock.
    with pytest.raises(ValueError, match="(?i)fire"):
        config.RunConfig(
            rail_class="door", overrides={"design.egress": "fire_rated", "control.seismic.policy.action": "open_and_stop"}
        ).params()
    for egress in ("non_rated", "fire_rated"):
        for tuning in ({"control.seismic.policy.action": "lock"}, {"control.seismic.policy.lock_after_land": True}):
            with pytest.raises(ValueError, match="(?i)lock"):
                config.RunConfig(rail_class="door", overrides={"design.egress": egress, **tuning}).params()

    cases = {
        # network alert before P, OpenAndStop configured: a non-local source forbids opening.
        "network_open_and_stop": ("egress_eew", {"control.seismic.policy.action": "open_and_stop"}, None, None),
        # local pick with a long lead (R_epi 120 km): the non-rated door may open and stop.
        "local_open_and_stop": (
            "pwave_m7_r20",
            {"control.seismic.policy.action": "open_and_stop"},
            {"r_epi_km": 120.0},
            _drop_moves,
        ),
        # local pick with a short lead (R_epi 60 km, S−P 7.2 s; the local S-ETA is biased early): if it opens at
        # all, it must be at rest ≥ 1 s before t_S.
        "local_short_lead": ("pwave_m7_r20", {"control.seismic.policy.action": "open_and_stop"}, {"r_epi_km": 60.0}, _drop_moves),
        # fire-rated door under a local pick with a long lead: never opens.
        "fire_rated_local": ("pwave_m7_r20", {"design.egress": "fire_rated"}, {"r_epi_km": 120.0}, _drop_moves),
    }
    jobs = [_job(native, "door", sc, overrides=ov, options=opt, mutate=mut, rate_hz=200.0) for sc, ov, opt, mut in cases.values()]
    runs = _run_all(native, jobs)
    bad = []
    for name, (p, d, _), r in zip(cases, jobs, runs, strict=True):
        info = scenarios.info(d)
        x0, travel = p.get("design.rail.x_closed_m"), p.get("design.rail.travel_m")
        t_s = float(info["t_s_s"])
        acts = set(int(a) for a in r["seismic_action"])
        opened = float(np.max(r["x_m"])) - x0
        record_property(f"{name}_max_open_m", round(opened, 4))
        if ACT["lock"] in acts or _first(r, "plant:locked") is not None or (r["contact_state"] & C_LOCKED).any():
            bad.append(f"{name}: the egress door locked")
        did_open = opened > OPENING_M
        # Non-local source, or fire-rated: OpenAndStop is never decided and the door never opens.
        if name.startswith(("network", "fire")) and (ACT["open_and_stop"] in acts or did_open):
            bad.append(f"{name}: OpenAndStop decided={ACT['open_and_stop'] in acts}, opened by {opened:.3f} m")
        if name == "local_open_and_stop" and not (ACT["open_and_stop"] in acts and opened >= 0.5 * travel):
            bad.append(f"{name}: a feasible local OpenAndStop should open the non-rated door (opened {opened:.3f} m)")
        if did_open:
            late = (r.t_s >= t_s - 1.0) & (r.t_s < t_s) & (np.abs(r["v_mps"]) > 5e-3)
            if late.any():
                bad.append(f"{name}: opened, and still moving at {r.t_s[late][0]:.2f} s, within 1 s of t_S {t_s:.2f} s")
        t_land = _landed_at(r, float(info["t_p_s"]) - 10.0)
        if not math.isfinite(t_land):
            bad.append(f"{name}: the door never came to rest on its rollers")
            continue
        drag = float(np.max(np.abs(r["f_lsm_n"][r.t_s >= t_land + 0.1]))) if (r.t_s >= t_land + 0.1).any() else 0.0
        record_property(f"{name}_lsm_force_after_landing_n", round(drag, 3))
        if drag > 0.5:
            bad.append(f"{name}: LSM force {drag:.2f} N after landing — ASC drag not released (manual operability)")
    manual = native.Params.preset("door").derived()["manual_force_bound_n"]
    record_property("door_manual_force_bound_n", round(manual, 3))
    if manual > DOOR_MANUAL_N:
        bad.append(f"manual force bound {manual:.2f} N > {DOOR_MANUAL_N} N (SAFETY SF-07)")
    assert not bad, f"SEI-011 egress rules (IBC 1010.3; NFPA 80): {bad}"


# ---------------------------------------------------------------------------------
# SEI-019 (scenario 9)
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-SEI-019")
def test_power_loss_at_s(native, record_property):
    """Contract: with mains lost at the S arrival — after the PROTECT landing (catalog M7 R20) and while still
    levitated (near field, R_hyp 9.4 km) — every class lands, stays captive between its physical stops without a new
    floor-driven monitor trip (D3), and keeps the manual force within IBC 1010.3.3 where it is not locked (FAB-SEI-019)."""
    variants = {"catalog": None, "near_field": {"r_epi_km": 5.0, "depth_km": 8.0}}
    jobs, tags = [], []
    for vname, opt in variants.items():
        for cls in CLASSES:
            jobs.append(_job(native, cls, "power_loss_at_s", options=opt, rate_hz=200.0))
            tags.append((vname, cls))
    runs = _run_all(native, jobs)
    bad = []
    for (vname, cls), (p, d, _), r in zip(tags, jobs, runs, strict=True):
        tag = f"{vname}/{cls}"
        t_loss = float(next(e["t_s"] for e in d["events"] if e["kind"] == "power_loss"))
        final = int(r["contact_state"][-1])
        t_rest = _landed_at(r, t_loss - 30.0)
        record_property(f"{tag}_rest_minus_loss_s", round(t_rest - t_loss, 3))
        if not final & (C_LANDED | C_SEATED):
            bad.append(f"{tag}: not at rest in contact at the end (contact_state {final:#x})")
        elif not t_rest <= t_loss + 3.0:
            bad.append(f"{tag}: at rest only {t_rest - t_loss:.2f} s after the power loss (≤ 3 s, (P))")
        x = r["x_m"]
        x0, travel, ez = p.get("design.rail.x_closed_m"), p.get("design.rail.travel_m"), p.get("design.damping.ez_len_m")
        x_lo, x_hi = x0 - ez - CAPTIVE_SLACK_M, x0 + travel + ez + CAPTIVE_SLACK_M
        record_property(f"{tag}_x_range_m", [round(float(x.min()), 4), round(float(x.max()), 4)])
        if x.min() < x_lo or x.max() > x_hi:
            bad.append(
                f"{tag}: x ∈ [{x.min():.4f}, {x.max():.4f}] m leaves the stop envelope [{x_lo:.3f}, {x_hi:.3f}] m (captive)"
            )
        trips = [(t, FLOOR_DRIVEN_TRIPS[c]) for t, c in _faults(r, t_rest) if c in FLOOR_DRIVEN_TRIPS]
        if trips:
            bad.append(f"{tag}: floor-driven monitor trips after coming to rest {trips} (D3: report, do not trip)")
        if r.kpis.get("bumper_energy_j", 0.0) > p.get("design.damping.bumper_e_rated_j"):
            bad.append(f"{tag}: bumper impact {r.kpis['bumper_energy_j']:.1f} J above its rating")
        if not final & C_LOCKED:
            manual = p.derived()["manual_force_bound_n"]
            record_property(f"{tag}_manual_force_bound_n", round(manual, 2))
            if manual > IBC_OPERATE_N:
                bad.append(f"{tag}: manual force bound {manual:.1f} N > {IBC_OPERATE_N} N (IBC 1010.3.3 operate)")
    assert not bad, f"SEI-019 power loss at S arrival: {bad}"
