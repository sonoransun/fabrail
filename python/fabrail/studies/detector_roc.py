"""Study ``detector_roc`` (tier 2): false alarms and misses of the on-site P-wave detector.

Question: over synthetic earthquakes and a day of synthetic building noise, how often
does the detector (the firmware seismic unit through ``_native.DetectorReplay``) PROTECT
when it should, how often does it PROTECT or FREEZE when it should not, how well do its
levels and Pd follow the calibration, and how good is its local S-ETA?

**Events.** Boore (2003) stochastic P+S records (``pwave.boore_ps``; P phase scaled so
Pd follows Wu–Zhao at hypocentral R and its 3 s τc follows Wu–Kanamori τc(M)) for
M ∈ {4.0 … 7.5} × R_epi ∈ {10, 20, 40, 80, 120} km at 10 km depth, several seeds per
cell, each with building noise and ADXL355 sensor noise added
(``pwave.site_noise(..., mems=True)``: the replay takes its input as what the sensor
measured). Truth labels are the PROTECT rule evaluated on the synthesis truth
(SEI-005: Level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal, with the expected level from
τc_WK(M) ≥ 0.6 s and Pd_WZ(M, R) ≥ 0.5 cm and the record's horizontal PGA). A hit is a
PROTECT at or after t_P and no later than t_S.

**Noise.** 24 h of ``pwave.site_noise_chunks`` (footsteps, slams, HVAC, self-actuation,
MEMS) streamed in 10-minute chunks. The seismic FSM latches INSPECT after a PROTECT and
waits for a manual reset (SEI-008), which would hide every later false alarm; the replay
therefore emulates the manual reset: after an INSPECT entry the detector is rebuilt and
warmed up on the preceding 60 s (events during the warm-up are discarded). Rates are per
24 h. A per-source attribution runs each noise source alone at the default threshold.

**Sweep.** The STA/LTA trigger ratio ``control.seismic.detector.trig_on`` ∈
{2.5 … 8.0} (preset 4.0) is the ROC knob; everything else is the preset.

Pd: the detector does not export its Pd, so the study reports the f64 reference chain
(``pwave.pd_tauc``, which mirrors ``fabrail_core::seismic::pdtc``) over the 3 s window
from the native pick, against Wu–Zhao (SEI-004 tolerance ±20 %).

Every record here is synthetic; nothing is validated on real records (SEISMIC.md §2.2).

Owner: B3 (Wave 2).
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from .. import pwave
from .runner import StudyResult, StudySpec

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..config import RunConfig

__all__ = [
    "DEPTH_KM",
    "MAGNITUDES",
    "NOISE_SOURCES",
    "R_EPI_KM",
    "STUDY",
    "TRIG_ON",
    "compute",
    "expected_level",
    "noise_rates",
    "run",
    "should_protect",
]

MAGNITUDES = (4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5)
R_EPI_KM = (10.0, 20.0, 40.0, 80.0, 120.0)
DEPTH_KM = 10.0
SEEDS_PER_CELL = 4
TRIG_ON = (2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0)
NOISE_HOURS = 24.0
NOISE_SOURCES = ("footsteps", "slams", "hvac", "self_actuation", "mems")
_FS_HZ = 500.0
_CHUNK_S = 600.0
_WARMUP_S = 60.0
_TRIG_PATH = "control.seismic.detector.trig_on"
_TAIL_S = 5.0


def expected_level(magnitude: float, r_hyp_km: float, tauc_l3_s: float = 0.6, pd_l3_m: float = 0.005) -> int:
    """Calibration-matrix level from the synthesis truth (SEI-005): τc_WK(M) and Pd_WZ(M, R)."""
    tc = pwave.tau_c_wu_kanamori_s(magnitude) >= tauc_l3_s
    pd = pwave.wu_zhao_pd_m(magnitude, r_hyp_km) >= pd_l3_m
    return 3 if (tc and pd) else 2 if tc else 1 if pd else 0


def should_protect(level: int, pd_m: float, pga_mps2: float, pd_alert_m: float = 0.0035,
                   pga_alert_mps2: float = 0.8, protect_level: int = 2) -> bool:
    """The PROTECT rule on the truth: Level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal (plan §2b item 6)."""
    return level >= protect_level or pd_m >= pd_alert_m or pga_mps2 >= pga_alert_mps2


def _bytes(a: np.ndarray) -> tuple[bytes, bytes, bytes]:
    return tuple(np.ascontiguousarray(a[i], dtype="<f8").tobytes() for i in range(3))  # type: ignore[return-value]


@dataclass
class _Stream:
    """One streamed detector with the manual-reset emulation (see the module docstring).

    After an INSPECT entry a fresh detector is warmed up on the 60 s of history that end
    1 s before the trigger that opened the episode (so the warm-up does not replay the
    episode itself), then continues with the next chunk; its event times are mapped as
    if the warm-up segment immediately preceded that chunk. Events before 60 s of the
    stream and during a warm-up are not counted.
    """

    params: Any
    detector: Any
    hist: deque[tuple[float, np.ndarray]]
    t0_s: float = 0.0
    warm_end_s: float = _WARMUP_S
    last_trigger_s: float = math.nan
    raw_triggers: int = 0
    freezes: int = 0
    protect_count: int = 0
    resets: int = 0
    level_max: int = 0
    protect_times_s: list[float] | None = None

    @classmethod
    def new(cls, params: Any) -> _Stream:
        from .. import require_native

        return cls(params, require_native().DetectorReplay(params, _FS_HZ), deque(), protect_times_s=[])

    def _segment(self, t_end_s: float) -> np.ndarray:
        """History samples in [t_end − 60 s, t_end) (possibly shorter near the start)."""
        parts = []
        for t_start, ch in self.hist:
            i0 = max(0, int(round((t_end_s - _WARMUP_S - t_start) * _FS_HZ)))
            i1 = min(ch.shape[1], int(round((t_end_s - t_start) * _FS_HZ)))
            if i1 > i0:
                parts.append(ch[:, i0:i1])
        return np.concatenate(parts, axis=1) if parts else np.zeros((3, 0))

    def feed(self, chunk: np.ndarray, t_chunk_s: float) -> None:
        from .. import require_native

        events = self.detector.feed(*_bytes(chunk))
        inspect = False
        for t, kind, a, _b in events:
            t_abs = self.t0_s + float(t)
            if t_abs < self.warm_end_s:
                continue
            if kind == "Trigger":
                self.raw_triggers += 1
                self.last_trigger_s = t_abs
            elif kind == "Freeze":
                self.freezes += 1
            elif kind == "Protect":
                self.protect_count += 1
                self.level_max = max(self.level_max, int(a))
                assert self.protect_times_s is not None
                self.protect_times_s.append(t_abs)
            elif kind == "SeismicState" and int(a) == 5:
                inspect = True
        self.hist.append((t_chunk_s, chunk))
        while len(self.hist) > 2:
            self.hist.popleft()
        if inspect:
            self.resets += 1
            t_cut = (self.last_trigger_s - 1.0) if math.isfinite(self.last_trigger_s) else t_chunk_s
            warm = self._segment(t_cut)
            if warm.shape[1] < int(0.6 * _WARMUP_S * _FS_HZ):
                warm = self._segment(t_chunk_s + chunk.shape[1] / _FS_HZ)
            t_end = t_chunk_s + chunk.shape[1] / _FS_HZ
            self.detector = require_native().DetectorReplay(self.params, _FS_HZ)
            self.t0_s = t_end - warm.shape[1] / _FS_HZ
            self.warm_end_s = t_end
            self.detector.feed(*_bytes(warm))


def noise_rates(params_list: list[Any], rng: np.random.Generator, hours: float = NOISE_HOURS,
                **noise_kw: Any) -> list[dict[str, Any]]:
    """Stream ``hours`` of site noise through one detector per parameter set; per-day rates."""
    streams = [_Stream.new(p) for p in params_list]
    dur = hours * 3600.0
    t = 0.0
    for chunk in pwave.site_noise_chunks(rng, dur, _FS_HZ, chunk_s=_CHUNK_S, **noise_kw):
        for s in streams:
            s.feed(chunk, t)
        t += chunk.shape[1] / _FS_HZ
    per_day = 24.0 / hours
    return [
        {
            "raw_triggers": s.raw_triggers,
            "freezes": s.freezes,
            "protect_count": s.protect_count,
            "manual_resets_emulated": s.resets,
            "level_max": s.level_max,
            "raw_triggers_per_day": s.raw_triggers * per_day,
            "freezes_per_day": s.freezes * per_day,
            "protect_per_day": s.protect_count * per_day,
            "first_protect_s": (s.protect_times_s or [math.nan])[0],
        }
        for s in streams
    ]


def _event_case(params_list: list[Any], rng: np.random.Generator, m: float, r_epi: float) -> dict[str, Any]:
    r_hyp = pwave.hypocentral_km(r_epi, DEPTH_KM)
    t_p = 35.0
    t_s = t_p + r_hyp / pwave.V_S_KMPS - r_hyp / pwave.V_P_KMPS
    t_d = 1.0 / pwave.brune_corner_hz(m) + 0.05 * r_hyp
    dur = math.ceil(2.0 * (t_s + 2.0 * t_d + _TAIL_S)) / 2.0
    ev = pwave.boore_ps(rng, m, r_epi, DEPTH_KM, fs_hz=_FS_HZ, duration_s=dur, t_p_s=t_p)
    a = ev.a_xyz_mps2 + pwave.site_noise(rng, dur, _FS_HZ, mems=True)
    lvl = expected_level(m, r_hyp)
    meta = cast(dict[str, float], ev.meta)
    pga = float(meta["pga_h_mps2"])
    pd_target = float(meta["pd_target_m"])
    truth = should_protect(lvl, pd_target, pga)
    per_thr = []
    from .. import require_native

    for p in params_list:
        det = require_native().DetectorReplay(p, _FS_HZ)
        events = det.feed(*_bytes(a))
        trig = [float(t) for t, k, _a, _b in events if k == "Trigger" and float(t) >= t_p - 0.5]
        early = [float(t) for t, k, _a, _b in events if k == "Trigger" and float(t) < t_p - 0.5]
        pre = [float(t) for t, k, _a, _b in events if k == "Protect" and float(t) < t_p - 0.5]
        prot = [(float(t), int(la), float(b)) for t, k, la, b in events if k == "Protect" and float(t) >= t_p - 0.5]
        hit = bool(prot and prot[0][0] <= t_s)
        per_thr.append(
            {
                "pick_s": trig[0] - t_p if trig else None,
                "triggers_before_p": len(early),
                # A noise PROTECT before the P-wave latches the FSM (QUIET/INSPECT): the
                # event is then not picked, although the device is already protected.
                "pre_event_protect": bool(pre),
                "protect": bool(prot),
                "protect_before_t_s": hit,
                "protected_by_t_s_any_cause": bool(pre) or hit,
                "protect_minus_t_s_s": prot[0][0] - t_s if prot else None,
                "level": prot[0][1] if prot else int(det.report()["level_max"]),
                "s_eta_error_s": (prot[0][0] + prot[0][2] - t_s) if (prot and prot[0][2] > 0.0) else None,
            }
        )
    # Pd of the f64 reference chain over 3 s from the default-threshold pick.
    default = per_thr[TRIG_ON.index(4.0)] if 4.0 in TRIG_ON else per_thr[0]
    t_pick = t_p + (default["pick_s"] if default["pick_s"] is not None else 0.0)
    pd_meas, tau_meas = pwave.pd_tauc(a[2], _FS_HZ, t_pick)
    return {
        "magnitude": m,
        "r_epi_km": r_epi,
        "r_hyp_km": r_hyp,
        "t_s_minus_t_p_s": t_s - t_p,
        "expected_level": lvl,
        "should_protect": truth,
        "pd_wu_zhao_m": pd_target,
        "pd_reference_chain_m": pd_meas,
        "pd_ratio": pd_meas / pd_target if pd_target > 0.0 else math.nan,
        "tau_c_wu_kanamori_s": float(meta["tau_c_wu_kanamori_s"]),
        "tau_c_reference_chain_s": tau_meas,
        "pga_h_mps2": pga,
        "by_trig_on": per_thr,
    }


def compute(
    cfg: RunConfig,
    magnitudes: tuple[float, ...] = MAGNITUDES,
    r_epi_km: tuple[float, ...] = R_EPI_KM,
    seeds_per_cell: int = SEEDS_PER_CELL,
    noise_hours: float = NOISE_HOURS,
    attribution: bool = True,
) -> dict[str, Any]:
    """Events × thresholds, the streamed noise day, and the per-source attribution."""
    base = cfg.params()
    params_list = [base.with_overrides({_TRIG_PATH: float(v)}) for v in TRIG_ON]
    root = np.random.SeedSequence(int(cfg.seed))
    ss_events, ss_noise, ss_attr = root.spawn(3)
    cells = [(m, r) for m in magnitudes for r in r_epi_km for _ in range(seeds_per_cell)]
    t0 = time.perf_counter()
    events = [
        _event_case(params_list, np.random.default_rng(child), m, r)
        for (m, r), child in zip(cells, ss_events.spawn(len(cells)), strict=True)
    ]
    t1 = time.perf_counter()
    noise = noise_rates(params_list, np.random.default_rng(ss_noise), noise_hours)
    t2 = time.perf_counter()
    attr: dict[str, Any] = {}
    if attribution:
        i_def = TRIG_ON.index(4.0)
        for src, child in zip(NOISE_SOURCES, ss_attr.spawn(len(NOISE_SOURCES)), strict=True):
            flags = {k: (k == src) for k in NOISE_SOURCES}
            attr[src] = noise_rates([params_list[i_def]], np.random.default_rng(child), noise_hours, **flags)[0]
    t3 = time.perf_counter()
    roc = []
    pos = [e for e in events if e["should_protect"]]
    neg = [e for e in events if not e["should_protect"]]
    for i, thr in enumerate(TRIG_ON):
        clean_pos = [e for e in pos if not e["by_trig_on"][i]["pre_event_protect"]]
        clean_neg = [e for e in neg if not e["by_trig_on"][i]["pre_event_protect"]]
        tp = sum(e["by_trig_on"][i]["protect_before_t_s"] for e in clean_pos)
        late = sum(e["by_trig_on"][i]["protect"] and not e["by_trig_on"][i]["protect_before_t_s"] for e in clean_pos)
        fp = sum(e["by_trig_on"][i]["protect"] for e in clean_neg)
        roc.append(
            {
                "trig_on": thr,
                "tpr": tp / len(clean_pos) if clean_pos else math.nan,
                "hits": tp,
                "late_protect_count": late,
                "misses": len(clean_pos) - tp - late,
                "positives": len(pos),
                "positives_clean": len(clean_pos),
                "pre_event_noise_protect_count": sum(e["by_trig_on"][i]["pre_event_protect"] for e in events),
                "protected_by_t_s_any_cause": sum(e["by_trig_on"][i]["protected_by_t_s_any_cause"] for e in pos),
                "false_protect_on_events": fp,
                "negatives": len(neg),
                "negatives_clean": len(clean_neg),
                "fpr_events": fp / len(clean_neg) if clean_neg else math.nan,
                "picks_on_negatives": sum(e["by_trig_on"][i]["pick_s"] is not None for e in clean_neg),
                **{f"noise_{k}": v for k, v in noise[i].items()},
            }
        )
    i_def = TRIG_ON.index(4.0)
    conf = np.zeros((4, 4), dtype=np.int64)
    for e in events:
        conf[e["expected_level"], min(3, max(0, e["by_trig_on"][i_def]["level"]))] += 1
    m5r10 = [e for e in events if e["magnitude"] == 5.0 and e["r_epi_km"] == 10.0]
    return {
        "inputs": {
            "magnitudes": list(magnitudes),
            "r_epi_km": list(r_epi_km),
            "depth_km": DEPTH_KM,
            "seeds_per_cell": seeds_per_cell,
            "trig_on": list(TRIG_ON),
            "trig_on_preset": float(base.get(_TRIG_PATH)),
            "noise_hours": noise_hours,
            "fs_hz": _FS_HZ,
            "chunk_s": _CHUNK_S,
            "warmup_s": _WARMUP_S,
            "rail_class": cfg.rail_class,
            "seed": cfg.seed,
            "truth": "PROTECT rule on the synthesis truth: expected level (tau_c_WK(M) >= 0.6 s, Pd_WZ >= 0.5 cm) "
            ">= 2 OR Pd_WZ >= 0.35 cm OR PGA_h >= 80 gal",
            "hit": "PROTECT at or after t_P - 0.5 s and no later than t_S; events with a noise PROTECT before "
            "t_P - 0.5 s (FSM already latched) are excluded from tpr/fpr and counted separately",
            "reset_emulation": "after INSPECT the detector is rebuilt and warmed up on the last 60 s (manual reset)",
            "pd_ratio_reference": 1.0,
            "pd_tolerance_band": [0.8, 1.2],
            "records": "synthetic (Boore 2003 stochastic + pwave.site_noise); not validated on real records",
        },
        "events": events,
        "roc": roc,
        "noise_attribution_default": attr,
        "level_confusion_default": {"rows_expected_cols_detected": conf.tolist()},
        "m5_r10": {
            "n": len(m5r10),
            "protect_count": sum(e["by_trig_on"][i_def]["protect"] for e in m5r10),
            "levels": [e["by_trig_on"][i_def]["level"] for e in m5r10],
        },
        "timing": {"n_events": len(events),
                   "wall_s": {"events_s": t1 - t0, "noise_s": t2 - t1, "attribution_s": t3 - t2}},
    }


def _outcomes_by_magnitude(rec: dict[str, Any], i_def: int) -> dict[str, Any]:
    """Per-magnitude counts at the preset threshold: hit, late, miss, pre-event noise PROTECT, correct reject, false."""
    out: dict[str, Any] = {"magnitude": list(rec["inputs"]["magnitudes"])}
    for key in ("hit", "late", "miss", "pre_event", "correct_reject", "false_protect"):
        out[key] = []
    for m in rec["inputs"]["magnitudes"]:
        sel = [e for e in rec["events"] if e["magnitude"] == m]
        cnt = dict.fromkeys(("hit", "late", "miss", "pre_event", "correct_reject", "false_protect"), 0)
        for e in sel:
            b = e["by_trig_on"][i_def]
            if b["pre_event_protect"]:
                cnt["pre_event"] += 1
            elif e["should_protect"]:
                cnt["hit" if b["protect_before_t_s"] else "late" if b["protect"] else "miss"] += 1
            else:
                cnt["false_protect" if b["protect"] else "correct_reject"] += 1
        for key, v in cnt.items():
            out[key].append(v)
    return out


def _figure(rec: dict[str, Any]) -> Any:
    from matplotlib.figure import Figure

    from .. import plotstyle as ps

    ps.apply()
    c = ps.COLORS
    fig = Figure(figsize=(15.0, 9.0))
    (a1, a2, a3), (a4, a5, a6) = fig.subplots(2, 3)
    roc = rec["roc"]
    preset = rec["inputs"]["trig_on_preset"]
    i_def = rec["inputs"]["trig_on"].index(preset)
    # (a) ROC: noise false PROTECT per day vs hit rate on clean events.
    a1.plot([r["noise_protect_per_day"] for r in roc], [r["tpr"] for r in roc], color=c["s1"], lw=1.4, marker="o",
            ms=5)
    a1.scatter([roc[i_def]["noise_protect_per_day"]], [roc[i_def]["tpr"]], color=c["star"], s=60, zorder=5,
               label=f"preset trig_on {preset:g}")
    for r in roc:
        a1.annotate(f"{r['trig_on']:g}", xy=(r["noise_protect_per_day"], r["tpr"]), xytext=(4, 4),
                    textcoords="offset points", fontsize=8, color=c["ink2"])
    a1.set_ylim(0.0, 1.05)
    a1.legend(loc="lower right", **ps.LEGEND_KW)
    ps.style_ax(a1, "false PROTECT per day in site noise (SEI-006 target: 0)", "P(PROTECT by t_S | should)",
                "ROC over the STA/LTA trigger ratio")
    # (b) Noise attribution at the preset threshold.
    attr = rec["noise_attribution_default"]
    srcs = list(attr)
    xs = np.arange(len(srcs))
    rec["noise_attribution_plot"] = {"sources": srcs,
                                     "freeze_per_day": [attr[k]["freezes_per_day"] for k in srcs],
                                     "protect_per_day": [attr[k]["protect_per_day"] for k in srcs]}
    a2.bar(xs - 0.2, rec["noise_attribution_plot"]["freeze_per_day"], width=0.4, color=c["ctrl"], label="FREEZE / day")
    a2.bar(xs + 0.2, rec["noise_attribution_plot"]["protect_per_day"], width=0.4, color=c["star"],
           label="PROTECT / day")
    a2.set_xticks(xs, [k.replace("_", "\n") for k in srcs], fontsize=8.5)
    a2.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(a2, None, "per day (each source alone)", f"False alarms by noise source ({rec['inputs']['noise_hours']:g} h)")
    # (c) Outcomes by magnitude at the preset threshold.
    om = _outcomes_by_magnitude(rec, i_def)
    rec["outcomes_by_magnitude_preset"] = om
    bottom = np.zeros(len(om["magnitude"]))
    colors = {"hit": c["good"], "late": c["s4"], "miss": c["star"], "pre_event": c["s5"],
              "correct_reject": c["s1"], "false_protect": c["s2"]}
    labels = {"hit": "hit", "late": "late (after t_S)", "miss": "miss", "pre_event": "noise PROTECT before P",
              "correct_reject": "correct no-PROTECT", "false_protect": "false PROTECT"}
    for key in ("hit", "late", "miss", "pre_event", "correct_reject", "false_protect"):
        vals = np.asarray(om[key], dtype=np.float64)
        a3.bar(om["magnitude"], vals, bottom=bottom, width=0.35, color=colors[key], label=labels[key])
        bottom += vals
    a3.legend(loc="upper center", fontsize=7.5, frameon=False, ncol=2)
    a3.set_ylim(0.0, float(bottom.max()) * 1.6 if bottom.size else 1.0)
    ps.style_ax(a3, "magnitude", "events", "Outcome by magnitude (preset)")
    # (d) Level confusion.
    conf = np.asarray(rec["level_confusion_default"]["rows_expected_cols_detected"], dtype=np.float64)
    a4.imshow(conf, cmap="Blues", origin="upper")
    for i in range(4):
        for j in range(4):
            a4.annotate(f"{int(conf[i, j])}", xy=(j, i), ha="center", va="center", fontsize=10,
                        color=c["ink"] if conf[i, j] < conf.max() * 0.6 else c["surface"])
    a4.set_xticks(range(4), ["L0", "L1", "L2", "L3"])
    a4.set_yticks(range(4), ["L0", "L1", "L2", "L3"])
    ps.style_ax(a4, "detected level (at PROTECT, else max)", "expected level (tau_c_WK, Pd_WZ)",
                "Level confusion (preset)", grid=False)
    # (e) Pd vs Wu-Zhao.
    ev = rec["events"]
    a5.scatter([e["magnitude"] for e in ev], [e["pd_ratio"] for e in ev], s=14, color=c["s1"], alpha=0.7)
    ps.ref_line(a5, rec["inputs"]["pd_ratio_reference"], "Wu-Zhao", color=c["star"])
    for band in rec["inputs"]["pd_tolerance_band"]:
        a5.axhline(band, color=c["muted"], lw=0.8, ls=":")
    a5.set_yscale("log")
    ps.style_ax(a5, "magnitude", "Pd (reference chain from pick) / Pd_WZ", "Pd vs Wu-Zhao (dotted: +-20 %)")
    # (f) Local S-ETA error.
    pts = [(e["r_hyp_km"], e["by_trig_on"][i_def]["s_eta_error_s"], e["magnitude"]) for e in ev
           if e["by_trig_on"][i_def]["s_eta_error_s"] is not None]
    if pts:
        sc = a6.scatter([p[0] for p in pts], [p[1] for p in pts], c=[p[2] for p in pts], cmap="viridis", s=16)
        fig.colorbar(sc, ax=a6, label="magnitude")
    ps.zero_line(a6)
    a6.set_xscale("log")
    a6.set_yscale("symlog", linthresh=5.0)
    ps.style_ax(a6, "hypocentral distance (km)", "local S-ETA - true t_S (s)", "Local S-ETA error (> 0: late)")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    ps.brand(fig, "detector_roc · firmware seismic unit via DetectorReplay · synthetic events and site noise, "
                  "not real records")
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the event grid, the 24 h noise day and the attribution (tier 2, run once)."""
    rec = compute(cfg)
    return StudyResult(fig=_figure(rec), record=rec)


STUDY = StudySpec(
    name="detector_roc",
    tier=2,
    question="How often does the on-site detector PROTECT when it should, and FREEZE or PROTECT when it should "
    "not, over synthetic events and 24 h of building noise (trigger-ratio sweep)?",
    run=run,
)
