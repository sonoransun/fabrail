"""Study ``seismic_actions`` (tier 1): the land-first class action timeline per class and distance.

Question: after the P-wave (or a network alert), when does each class reach its
land-first rest state, relative to t_P and t_S, at each distance, with on-site detection
alone and with a synthetic network alert fused in?

Runs (the real controller, monitor and plant through ``_native.run_batch``; Travel1d,
the seismic tier, plan §1): the catalog scenario ``pwave_m7_r20`` with ``r_epi_km``
overridden, i.e. a synthetic Boore (2003) M7 P+S record at 10 km depth with the P phase
scaled to Wu–Zhao at hypocentral R (PHYSICS §19), the carriage starting a move 0.5 s
before t_P so FREEZE is observable (SEI-009), for every class and
R_epi ∈ {10, 20, 40, 60, 100} km:

- ``onsite``: the structure accelerometer only (STA/LTA → Pd/τc → level → PROTECT);
- ``fused``: the same record plus a synthetic network alert 8 s after origin
  (``eew_leadtime.NETWORK_ALERT_S``; "synthetic/injected, not a live EEW feed").

Study configuration (recorded): ``control.supervisor.hover_timeout_s`` = 600 s so the
non-hover classes are still levitated and moving when the P-wave arrives (with the
preset 30 s they would land on the idle timeout before t_P = 35 s and the action would
be trivially complete); the catalog's 35 s LTA warm-up before t_P is kept. All times
are reported **from the origin** (t_P = R/V_p, t_S = R/V_s).

Per run: first alert (freeze), PROTECT and its level, the local S-ETA estimate and its
error, stop, landing, locking, "secured" (locked where the class locks, else landed or
seated) and its margin to t_S, the monitor/levitation faults raised, and the plant's
touchdown. The seismic action is declared not a rated safety function (SF-08); a
missed t_S or a monitor trip is reported, never hidden. Single seed (``cfg.seed``): the
record is identical across classes at each distance.

Owner: B3 (Wave 2).
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from ..presets import CLASSES
from .eew_leadtime import NETWORK_ALERT_S
from .runner import StudyResult, StudySpec

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..config import RunConfig

__all__ = ["DEPTH_KM", "MAGNITUDE", "MODES", "R_EPI_KM", "STUDY", "STUDY_OVERRIDES", "compute", "extract", "run"]

MAGNITUDE = 7.0
DEPTH_KM = 10.0
R_EPI_KM = (10.0, 20.0, 40.0, 60.0, 100.0)
MODES = ("onsite", "fused")
#: Study configuration: keep the carriages levitated (and moving) until the P-wave.
STUDY_OVERRIDES: dict[str, object] = {"control.supervisor.hover_timeout_s": 600.0}
_RECORD_RATE_HZ = 100.0
_V_STOP_MPS = 1e-3
#: P arrival in the catalog P+S records (``scenarios._T_P_S``: the 30 s LTA warm-up + 5 s).
_T_P_CATALOG_S = 35.0
#: Run length after t_S, s.
_AFTER_S_S = 12.0
#: Recorded channels (the timeline needs only these; keeps 40 runs small in memory).
_CHANNELS = ("x_m", "v_mps", "lev_state", "seismic_state")

#: Class land-first actions (SEISMIC.md §2.5; SEI-010, SEI-011).
_ACTION_TEXT = {
    "drawer": "FREEZE, then stay seated or land",
    "door": "FREEZE, stop, land on rollers, release ASC, never lock",
    "wall": "FREEZE, centre only if reachable, land + lock",
    "heavy": "FREEZE, centre only if reachable, land + lock",
}

_LEV = {0: "boot", 1: "selftest", 2: "seated", 3: "landed", 4: "lifting", 5: "levitated_idle", 6: "moving",
        7: "stopping", 8: "landing", 9: "seating", 10: "fault_reaction", 11: "fault", 12: "seismic", 13: "escape",
        14: "manual"}


def _fault_name(code: int) -> str:
    names = {
        0x0501: "GapEnvelope", 0x0502: "LiftTimeout", 0x0503: "LandTimeout", 0x0504: "PayloadMismatch",
        0x0505: "SaturationPersist", 0x0506: "SeatFailed", 0x0601: "EnergyLimit", 0x0602: "MomentumLimit",
        0x0603: "ForceLimit", 0x0604: "Overspeed", 0x0605: "PositionLimit", 0x0606: "UnintendedMotion",
        0x0607: "Obstruction", 0x0608: "PassivityViolation", 0x0609: "NotHoldToRun", 0x060A: "ObstructionDegraded",
        0x060B: "IsolateLimit", 0x0701: "VerticalSaturation", 0x0702: "CenterUnreachable", 0x0301: "BusUnder",
        0x0303: "MainsLost", 0x0204: "CoilI2t", 0x0210: "LsmOvercurrent",
    }
    return names.get(code, f"0x{code:04X}")


@dataclass(frozen=True)
class _Job:
    rail_class: str
    r_epi_km: float
    mode: str


def _info(spec_d: dict[str, Any]) -> dict[str, Any]:
    from ..scenarios import _INFO_MARK

    return json.loads(str(spec_d["doc"]).split(_INFO_MARK, 1)[1])


def _build(job: _Job, cfg: RunConfig) -> tuple[Any, Any, dict[str, Any]]:
    from .. import pwave, require_native, scenarios
    from ..config import RunConfig as _RC

    n = require_native()
    overrides = {**STUDY_OVERRIDES, **cfg.overrides}
    # Stop each run _AFTER_S_S after t_S: every outcome of interest is decided by then.
    r_hyp = pwave.hypocentral_km(job.r_epi_km, DEPTH_KM)
    dur = math.ceil(2.0 * (_T_P_CATALOG_S + r_hyp / pwave.V_S_KMPS - r_hyp / pwave.V_P_KMPS + _AFTER_S_S)) / 2.0
    rc = _RC(rail_class=job.rail_class, seed=cfg.seed, duration_s=dur, fidelity="travel1d", coil=cfg.coil,
             record_rate_hz=_RECORD_RATE_HZ, overrides=overrides, plant=dict(cfg.plant))
    params = rc.params()
    spec_d, series = scenarios.spec("pwave_m7_r20", params, rc, options={"magnitude": MAGNITUDE,
                                                                        "r_epi_km": job.r_epi_km, "depth_km": DEPTH_KM})
    d = cast(dict[str, Any], spec_d)
    d["record"]["channels"] = list(_CHANNELS)
    info = _info(d)
    if job.mode == "fused":
        origin = float(info["origin_t_s"])
        t_alert = max(0.0, origin + NETWORK_ALERT_S)
        alert: dict[str, Any] = {
            "kind": "network_alert",
            "t_s": t_alert,
            "magnitude": MAGNITUDE,
            "r_epi_km": job.r_epi_km,
            "depth_km": DEPTH_KM,
            "origin_t_s": origin,
            "id": 1,
        }
        d["events"] = sorted([*d["events"], alert], key=lambda e: float(e["t_s"]))
        d["name"] = f"{d['name']}_fused"
        info = {**info, "t_alert_s": t_alert, "alert": "synthetic/injected, not a live EEW feed"}
    scenarios.validate_spec(d)
    scenarios.cross_check(d, series, params)
    return params, n.Scenario.from_json(scenarios.to_json(d), series), info


def extract(run: Any, info: dict[str, Any], rail_class: str) -> dict[str, Any]:
    """The action timeline of one finished run, times from the origin (s)."""
    origin = float(info["origin_t_s"])
    t_p = float(info["t_p_s"]) - origin
    t_s = float(info["t_s_s"]) - origin
    ev = [(float(t) - origin, str(k), float(a), float(b)) for t, k, a, b in run.events()]
    after_move = float(info["t_p_s"]) - 0.5 - origin  # the scenario's move starts 0.5 s before t_P

    def first(kind: str, pred: Any = None, t_min: float = -math.inf) -> tuple[float, float, float] | None:
        for t, k, a, b in ev:
            if k == kind and t >= t_min and (pred is None or pred(a, b)):
                return t, a, b
        return None

    freeze = first("SeismicState", lambda a, _b: int(a) == 1)
    protect = first("SeismicState", lambda a, _b: int(a) == 2)
    prot_ev = first("Protect")
    t_alert = min((x[0] for x in (freeze, protect) if x is not None), default=math.nan)
    # Outcomes count from the scenario's move (0.5 s before t_P) or the first alert,
    # whichever is earlier (a fused network alert can precede the move at long range).
    t_ref = min(after_move, t_alert) if math.isfinite(t_alert) else after_move
    landed = first("plant:landed", t_min=t_ref)
    locked = first("plant:locked", t_min=t_ref)
    seated = first("LevState", lambda a, _b: int(a) == 2, t_min=t_ref)
    faults: list[dict[str, Any]] = [
        {"t_s": t, "code": int(a), "name": _fault_name(int(a))}
        for t, k, a, _b in ev
        if k == "FaultRaised" and t >= t_ref
    ]
    demands: list[dict[str, Any]] = [{"t_s": t, "demand": int(a)} for t, k, a, _b in ev if k == "Demand" and t >= t_ref]
    lev_final = next((int(a) for t, k, a, _b in reversed(ev) if k == "LevState"), -1)
    # Stop: first sample after the first alert with |v| below 1 mm/s.
    t_stop = math.nan
    if math.isfinite(t_alert):
        v = np.frombuffer(run.channel("v_mps")[0], dtype="<f8")
        t = np.arange(v.shape[0]) / float(run.rate_hz) - origin
        idx = np.nonzero((t >= t_alert) & (np.abs(v) < _V_STOP_MPS))[0]
        if idx.size:
            t_stop = float(t[idx[0]])
    x = np.frombuffer(run.channel("x_m")[0], dtype="<f8")
    locks = rail_class in ("wall", "heavy")
    candidates = []
    if locks and locked is not None:
        candidates.append(locked[0])
    if not locks:
        candidates += [e[0] for e in (landed, seated) if e is not None]
    t_secured = min(candidates) if candidates else math.nan
    s_eta_est = (prot_ev[0] + prot_ev[2]) if (prot_ev is not None and prot_ev[2] > 0.0) else math.nan
    kpis = run.kpis()
    return {
        "t_p_s": t_p,
        "t_s_s": t_s,
        "t_network_alert_s": float(info["t_alert_s"]) - origin if "t_alert_s" in info else None,
        "t_first_alert_s": t_alert,
        "t_move_start_s": after_move,
        "first_fault_before_alert": bool(faults and (not math.isfinite(t_alert) or faults[0]["t_s"] < t_alert)),
        "t_protect_s": protect[0] if protect else math.nan,
        "level": int(prot_ev[1]) if prot_ev else int(kpis.get("level_max", 0.0)),
        "s_eta_estimate_s": s_eta_est,
        "s_eta_error_s": s_eta_est - t_s if math.isfinite(s_eta_est) else math.nan,
        "t_stopped_s": t_stop,
        "t_landed_s": landed[0] if landed else math.nan,
        "touchdown_v_mps": landed[1] if landed else math.nan,
        "t_locked_s": locked[0] if locked else math.nan,
        "t_seated_s": seated[0] if seated else math.nan,
        "t_secured_s": t_secured,
        "secured_before_t_s": bool(math.isfinite(t_secured) and t_secured <= t_s),
        "margin_to_t_s_s": t_s - t_secured if math.isfinite(t_secured) else math.nan,
        "x_final_m": float(x[-1]),
        "stroke_max_m": float(kpis.get("stroke_max_m", math.nan)),
        "faults": faults,
        "t_first_fault_s": faults[0]["t_s"] if faults else math.nan,
        "p_picked_before_t_s": bool(math.isfinite(t_alert) and t_alert <= t_s),
        "monitor_trips": [f for f in faults if 0x0600 <= f["code"] <= 0x06FF],
        "max_demand": max((d["demand"] for d in demands), default=0),
        "lev_final": _LEV.get(lev_final, str(lev_final)),
        "pga_h_mps2": float(info["pga_h_mps2"]),
        "pd_target_m": float(info["pd_target_m"]),
    }


def compute(
    cfg: RunConfig,
    classes: tuple[str, ...] = CLASSES,
    r_epi_km: tuple[float, ...] = R_EPI_KM,
    modes: tuple[str, ...] = MODES,
) -> dict[str, Any]:
    """Run every (class, R, mode) case and return the record."""
    from .. import pwave, require_native

    jobs = [_Job(c, r, m) for c in classes for r in r_epi_km for m in modes]
    t0 = time.perf_counter()
    built = [_build(j, cfg) for j in jobs]
    t1 = time.perf_counter()
    runs = require_native().run_batch([(p, s) for p, s, _ in built], cfg.workers)
    t2 = time.perf_counter()
    rows = []
    per_mode: dict[str, int] = {}
    for i, (job, (_p, _s, info), run) in enumerate(zip(jobs, built, runs, strict=True)):
        row = {"row": i, "plot_y": per_mode.get(job.mode, 0), "rail_class": job.rail_class, "r_epi_km": job.r_epi_km,
               "mode": job.mode, "r_hyp_km": pwave.hypocentral_km(job.r_epi_km, DEPTH_KM),
               **extract(run, info, job.rail_class)}
        per_mode[job.mode] = row["plot_y"] + 1
        row["detect_s"] = row["t_protect_s"] - row["t_first_alert_s"]
        row["act_s"] = row["t_secured_s"] - row["t_protect_s"]
        # Secured by a fault reaction (SS1) with no seismic PROTECT: shown hatched.
        row["fault_secure_s"] = (
            row["t_secured_s"] - row["t_first_fault_s"]
            if not math.isfinite(row["t_protect_s"]) and math.isfinite(row["t_secured_s"])
            and math.isfinite(row["t_first_fault_s"])
            else math.nan
        )
        rows.append(row)
    summary: dict[str, Any] = {}
    for mode in modes:
        sel = [r for r in rows if r["mode"] == mode]
        summary[mode] = {
            "secured_before_t_s": sum(r["secured_before_t_s"] for r in sel),
            "n": len(sel),
            "with_monitor_trip": sum(bool(r["monitor_trips"]) for r in sel),
            "with_any_fault": sum(bool(r["faults"]) for r in sel),
            "alert_before_t_s": sum(bool(r["p_picked_before_t_s"]) for r in sel),
            "s_eta_error_s": sorted({round(r["s_eta_error_s"], 6) for r in sel if math.isfinite(r["s_eta_error_s"])}),
        }
    sim_s = sum(float(sc.duration_s) for _p, sc, _i in built)
    return {
        "inputs": {
            "magnitude": MAGNITUDE,
            "depth_km": DEPTH_KM,
            "r_epi_km": list(r_epi_km),
            "modes": list(modes),
            "classes": list(classes),
            "scenario": "pwave_m7_r20 with r_epi_km overridden (synthetic Boore 2003 P+S, P scaled to Wu-Zhao)",
            "fidelity": "travel1d",
            "coil": cfg.coil,
            "seed": cfg.seed,
            "study_overrides": dict(STUDY_OVERRIDES),
            "run_ends_after_t_s_s": _AFTER_S_S,
            "network_alert_after_origin_s": NETWORK_ALERT_S,
            "alerts": "synthetic/injected, not a live EEW feed",
            "time_origin": "earthquake origin (t_P = R/V_p, t_S = R/V_s)",
            "secured": "locked for WALL/HEAVY, else landed or seated",
            "sf08": "the seismic action is declared not a rated safety function (SAFETY.md SF-08)",
        },
        "actions": {c: _ACTION_TEXT[c] for c in classes},
        "rows": rows,
        "summary": summary,
        "timing": {"simulated_s": sim_s,
                   "wall_s": {"synthesis_s": t1 - t0, "run_batch_s": t2 - t1,
                              "workers": cfg.workers or os.cpu_count()}},
    }


def _figure(rec: dict[str, Any]) -> Any:
    from matplotlib.figure import Figure

    from .. import plotstyle as ps

    ps.apply()
    c = ps.COLORS
    modes = rec["inputs"]["modes"]
    fig = Figure(figsize=(13.0, 7.2))
    axes = fig.subplots(1, len(modes), sharey=True, squeeze=False)[0]
    for ax, mode in zip(axes, modes, strict=True):
        rows = [r for r in rec["rows"] if r["mode"] == mode]
        for r in rows:
            k = r["plot_y"]
            col = ps.CLASS_COLORS[r["rail_class"]]
            if math.isfinite(r["t_first_alert_s"]) and math.isfinite(r["t_protect_s"]):
                ax.barh(k, r["detect_s"], left=r["t_first_alert_s"], height=0.55,
                        color=c["ctrl"], alpha=0.45)
            if math.isfinite(r["t_protect_s"]) and math.isfinite(r["t_secured_s"]):
                ax.barh(k, r["act_s"], left=r["t_protect_s"], height=0.55, color=col)
            if math.isfinite(r["fault_secure_s"]):
                ax.barh(k, r["fault_secure_s"], left=r["t_first_fault_s"], height=0.55, color="none", edgecolor=col,
                        hatch="////", linewidth=0.8)
        ys = [r["plot_y"] for r in rows]
        ax.scatter([r["t_p_s"] for r in rows], ys, marker="|", s=120, color=c["ink"], label="t_P", zorder=4)
        ax.scatter([r["t_s_s"] for r in rows], ys, marker="D", s=26, color=c["star"], label="t_S", zorder=4)
        net = [r for r in rows if r["t_network_alert_s"] is not None]
        if net:
            ax.scatter([r["t_network_alert_s"] for r in net], [r["plot_y"] for r in net], marker=">", s=30,
                       color=c["accent"], label="network alert", zorder=4)
        trips = [(f["t_s"], r["plot_y"]) for r in rows for f in r["monitor_trips"]]
        if trips:
            ax.scatter([t for t, _ in trips], [y for _, y in trips], marker="x", s=40, color=c["star"],
                       label="monitor trip", zorder=5)
        ax.set_yticks(ys, [f"{r['rail_class'].upper()} {r['r_epi_km']:g} km" for r in rows], fontsize=8.5)
        ax.invert_yaxis()
        ax.set_xlim(0.0, 36.0)
        ps.style_ax(ax, "time from origin (s)", None, "on-site detection only" if mode == "onsite"
                    else "on-site + synthetic network alert (origin + 8 s)")
        ax.grid(axis="x", **ps.GRID_KW)
        ax.legend(loc="lower right", **ps.LEGEND_KW)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    ps.brand(fig, "seismic_actions · M7, 10 km deep, Travel1d · grey: alert to PROTECT, colour: PROTECT to secured "
                  "· hatched: secured by a fault reaction (SS1), not the seismic action · not a rated safety function (SF-08)")
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the 4 classes × 5 distances × 2 alert modes and plot the Gantt."""
    rec = compute(cfg)
    fig = _figure(rec)
    return StudyResult(fig=fig, record=rec)


STUDY = StudySpec(
    name="seismic_actions",
    tier=1,
    question="When does each class reach its land-first rest state relative to t_P and t_S, per distance, "
    "on site alone and with a network alert?",
    run=run,
)
