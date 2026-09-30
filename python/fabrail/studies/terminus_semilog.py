"""terminus_semilog: the log-damped soft-open and soft-close termini as straight semilog capture lines.

Claims backed: HAP-013 (during capture the distance-to-go decays as d_b·e^(−t/τ), so ln d is
a straight line of slope −1/τ with R² ≥ 0.999), HAP-016 (arrival ≤ 1 mm/s on truth,
overshoot ≤ 0.02 mm, t_arrive within ±2 % of the closed form) and the README's "log
dampened soft open/close", read through the plan's measurable definition (plan §4b).

Method. Closed loop on Carriage6 (voltage coil mode, quiet floor; HAP-013…016 are asserted
on Carriage6 only): the catalog ``terminus_soft_close`` and ``soft_open`` moves per class.
The distance-to-go d = |x_term − x| on plant truth is normalised by its value at the
capture event and plotted against (t − t_capture)/τ: every capture is then the same
straight line e^(−u) on a semilog axis, ending where the quintic takes over at d_ε. The
criteria are the simulator's terminus KPIs (semilog R² over the capture, arrival speed on
truth, overshoot, arrival time against the closed form measured to the reference's own
20 µm crossing), each divided by its limit; a Python least-squares fit on the recorded
1 kHz trace cross-checks τ and R².
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .. import plotstyle
from .runner import (
    CLASSES,
    StudyResult,
    StudySpec,
    check,
    class_cfg,
    class_style,
    faults_of,
    finish,
    floats,
    new_figure,
    quick,
    run_jobs,
    stamp_meta,
    tex_safe,
)

QUICK_AWARE = True

QUESTION = "Do the closed-loop soft-open and soft-close termini of each class plot as straight lines on a semilog chart, and do they arrive softly?"

LIMITS = {"one_minus_r2": 1e-3, "v_arrive_mps": 1e-3, "t_arrive_err": 0.02, "overshoot_m": 2e-5}  # FAB-HAP-013/016
_CRIT_LABELS = {
    "one_minus_r2": "1 − R²\n(≤ 1e-3)",
    "v_arrive_mps": "arrival speed\n(≤ 1 mm/s)",
    "t_arrive_err": "|t_arrive error|\n(≤ 2 %)",
    "overshoot_m": "overshoot\n(≤ 0.02 mm)",
}
_FLOOR = 1e-4  # plotted floor of a zero criterion on the log axis
_MOVES = (("close", "terminus_soft_close"), ("open", "soft_open"))


def _semilog_fit(t: np.ndarray, d: np.ndarray) -> tuple[float, float]:
    """(τ, R²) of the least-squares line ln d = a − t/τ."""
    y = np.log(d)
    b, a = np.polyfit(t, y, 1)
    resid = y - (a + b * t)
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - float(np.sum(resid**2)) / ss_tot if ss_tot > 0 else float("nan")
    return (-1.0 / b if b < 0 else float("inf")), r2


def run(cfg: Any) -> StudyResult:
    from .. import scenarios

    q = quick()
    cases = [("drawer", "close"), ("door", "close")] if q else [(c, mv) for c in CLASSES for mv, _ in _MOVES]
    jobs, infos = [], []
    for cls, move in cases:
        c = class_cfg(cfg, cls)
        p = c.params()
        name = dict(_MOVES)[move]
        d, s = scenarios.spec(name, p, c, fidelity="carriage6")
        d["record"] = {"rate_hz": 1000.0, "channels": ["x_m", "v_mps"]}
        jobs.append((p, d, s))
        infos.append(scenarios.info(d))
    results = run_jobs(jobs, cfg)

    fig, axes = new_figure(1, 2, size=(15.0, 6.8), gridspec_kw={"width_ratios": [1.25, 1.0]})
    ax_s, ax_c = axes[0]
    rec: dict[str, Any] = {}
    checks: dict[str, Any] = {}
    u_ideal = np.linspace(0.0, 6.0, 61)
    ax_s.plot(u_ideal, np.exp(-u_ideal), color=plotstyle.COLORS["ink"], ls=(0, (4, 3)), lw=1.3, label="ideal d/d_b = e^(−t/τ)")
    d_eps_ratio = []
    for (cls, move), r, info, (p, _spec, _) in zip(cases, results, infos, jobs, strict=True):
        st = class_style(cls)
        key = f"{cls}_{move}"
        cap = next((e for e in r.events if e.kind == "TerminusCapture"), None)
        faults = faults_of(r)
        if cap is None:
            rec[key] = {
                "captured": False,
                "faults": faults,
                "x_end_mm": float(r["x_m"][-1] * 1e3),
                "reason": "no terminus capture in this run" + (f" (faults: {', '.join(faults)})" if faults else ""),
            }
            continue
        x0 = float(p.get("design.rail.x_closed_m"))
        x_term = x0 if move == "close" else float(info.get("target_x_m", x0 + float(p.get("design.rail.travel_m"))))
        tau = float(cap.b)
        t_c = float(cap.t_s)
        d_eps = float(p.get("control.motion.terminus.d_eps_m"))
        dist = np.abs(x_term - r["x_m"])
        d_b = float(cap.a)  # the capture event carries the planned d_b; the blend runs from d_0 down to it
        reached = (r.t_s >= t_c) & (dist <= d_b)
        if not np.any(reached):
            rec[key] = {"captured": False, "faults": faults, "x_end_mm": float(r["x_m"][-1] * 1e3), "reason": "the blend never reached d_b"}
            continue
        t_x = float(r.t_s[np.argmax(reached)])  # start of the exponential phase (t_c + t_b)
        after = r.t_s >= t_x
        d_c = d_b
        # Plot from the exponential start to 5 % of d_ε (the quintic tail), at most 8τ.
        keep = after & (r.t_s <= t_x + 8.0 * tau)
        t_rel = (r.t_s[keep] - t_x) / tau
        ratio = dist[keep] / d_c
        stop = np.argmax(ratio < 0.05 * d_eps / d_c) if np.any(ratio < 0.05 * d_eps / d_c) else len(ratio)
        t_rel, ratio = t_rel[:stop], ratio[:stop]
        ratio = np.maximum(ratio, 1e-12)
        ax_s.plot(
            t_rel,
            ratio,
            color=st["color"],
            lw=1.9 if move == "close" else 1.4,
            ls="-" if move == "close" else (0, (6, 2)),
            label=f"{st['label']} {move}",
        )
        d_eps_ratio.append(d_eps / d_c)
        # Python cross-check fit over the exponential phase [t_c, t_ε).
        below = after & (dist < d_eps)
        t_eps = float(r.t_s[np.argmax(below)]) if np.any(below) else t_x + 8.0 * tau
        fit_w = after & (r.t_s < t_eps)
        tau_fit, r2_fit = _semilog_fit(r.t_s[fit_w] - t_x, dist[fit_w]) if np.sum(fit_w) > 5 else (float("nan"), float("nan"))
        k = r.kpis
        crit = {
            "one_minus_r2": None if k.get("terminus_r2_min") is None else 1.0 - k["terminus_r2_min"],
            "v_arrive_mps": k.get("terminus_v_arrive_max_mps"),
            "t_arrive_err": None if k.get("terminus_t_arrive_err_max") is None else abs(k["terminus_t_arrive_err_max"]),
            "overshoot_m": k.get("terminus_overshoot_max_m"),
        }
        rec[key] = {
            "captured": True,
            "tau_s": tau,
            "t_capture_event_s": t_c,
            "t_exponential_start_s": t_x,
            "t_eps_s": t_eps if np.any(below) else None,
            "reached_d_eps": bool(np.any(below)),
            "d_b_mm": d_b * 1e3,
            "x_term_mm": x_term * 1e3,
            "plot_t_over_tau": floats(t_rel),
            "plot_d_over_d_capture": floats(ratio),
            "kpi_r2_min": k.get("terminus_r2_min"),
            "kpi_tau_fit_s": k.get("terminus_tau_fit_s"),
            "kpi_t_arrive_s": k.get("terminus_t_arrive_s"),
            "kpi_t_arrive_closed_form_s": k.get("terminus_t_arrive_cf_s"),
            "python_fit_tau_s": tau_fit,
            "python_fit_r2": r2_fit,
            "criteria": crit,
            "criteria_over_limit": {c_: (None if v is None else v / LIMITS[c_]) for c_, v in crit.items()},
            "faults": faults,
        }
        for c_, v in crit.items():
            checks[f"{key}.{c_}"] = check(v is not None and v <= LIMITS[c_], v, LIMITS[c_], "FAB-HAP-013/016 (Carriage6, quiet floor)")
    if d_eps_ratio:
        ref_eps = float(np.median(d_eps_ratio))
        plotstyle.ref_line(ax_s, ref_eps, "d_ε = 0.2 mm: the quintic takes over", va="top")
    else:
        ref_eps = None
    ax_s.set_yscale("log")
    ax_s.set_ylim(1e-3, 1.5)
    ax_s.set_xlim(0.0, 6.5)
    plotstyle.style_ax(
        ax_s, "time since the exponential capture began / τ", "distance-to-go / d_b", "(a) Semilog capture lines (plant truth)"
    )
    ax_s.legend(loc="lower left", ncols=2, **plotstyle.LEGEND_KW)

    # (b) criteria over their limits, as a dot plot on a log axis.
    crit_keys = list(LIMITS)
    captured = [k for k, v in rec.items() if v.get("captured")]
    dots: dict[str, Any] = {}
    for j, key in enumerate(captured):
        cls, move = key.split("_")
        st = class_style(cls)
        off = (j - (len(captured) - 1) / 2.0) * 0.07
        xs_, ys_ = [], []
        for i, c_ in enumerate(crit_keys):
            v = rec[key]["criteria_over_limit"][c_]
            if v is None:
                continue
            xs_.append(i + off)
            ys_.append(max(v, _FLOOR))
        ax_c.plot(
            xs_,
            ys_,
            ls="none",
            marker=st["marker"],
            ms=8,
            color=st["color"],
            mfc=st["color"] if move == "close" else "none",
            mew=1.6,
            mec=plotstyle.COLORS["surface"] if move == "close" else st["color"],
        )
        dots[key] = {"x": xs_, "ratio_plotted": ys_}
    plotstyle.ref_line(ax_c, 1.0, "limit", color=plotstyle.COLORS["star"])
    ax_c.set_yscale("log")
    ax_c.set_ylim(_FLOOR * 0.6, 3.0)
    ax_c.set_xticks(range(len(crit_keys)), [tex_safe(_CRIT_LABELS[c_]) for c_ in crit_keys])
    ax_c.set_xlim(-0.6, len(crit_keys) - 0.4)
    plotstyle.style_ax(ax_c, None, "value / limit (below 1 passes)", "(b) HAP-013…016 criteria")
    from matplotlib.lines import Line2D

    handles = [
        Line2D([], [], ls="none", marker=plotstyle.CLASS_MARKERS[c], ms=8, color=plotstyle.CLASS_COLORS[c], label=plotstyle.CLASS_LABELS[c])
        for c in CLASSES
    ]
    handles += [
        Line2D([], [], ls="none", marker="o", ms=8, color=plotstyle.COLORS["ink2"], label="close (filled)"),
        Line2D([], [], ls="none", marker="o", ms=8, mfc="none", mew=1.6, color=plotstyle.COLORS["ink2"], label="open (hollow)"),
    ]
    ax_c.legend(handles=handles, loc="center right", **plotstyle.LEGEND_KW)

    missing = [f"{k.replace('_', ' ')}: {v['reason']}" for k, v in rec.items() if not v.get("captured")]
    worst_r2 = min((v["kpi_r2_min"] for v in rec.values() if v.get("captured") and v["kpi_r2_min"] is not None), default=None)
    fails = sorted({k.rsplit(".", 1)[0] for k, v in checks.items() if not v["pass"]})
    caption = (
        "(a) Distance-to-go on plant truth, normalised by the blend end d_b, against time since the exponential phase began (the "
        "distance first reaches d_b) in units of "
        "τ (DRAWER 0.15, DOOR 0.25, WALL 0.6, HEAVY 1.0 s): each log-damped terminus is the same straight line e^(−t/τ) until "
        "the quintic takes the last d_ε = 0.2 mm to rest. Carriage6, voltage coil mode, quiet floor; catalog terminus_soft_close "
        "(solid) and soft_open (dashed).\n"
        "(b) The simulator's terminus KPIs over their FAB-HAP-013/016 limits (a zero is drawn at 1e-4). "
        + (f"Worst semilog R² {worst_r2:.5f}. " if worst_r2 is not None else "")
        + (f"Criteria not met: {', '.join(fails)}. " if fails else "Every captured terminus meets all four criteria. ")
        + "".join(
            f"{k.replace('_', ' ').upper()} stalls at {v['plot_d_over_d_capture'][-1] * v['d_b_mm']:.1f} mm from its target without reaching "
            f"d_ε (a semilog fit to its whole capture gives R² {v['python_fit_r2']:.2f}"
            + (f"; the simulator KPI {v['kpi_r2_min']:.5f} covers only the moving part" if v["kpi_r2_min"] is not None else "")
            + "). "
            for k, v in rec.items()
            if v.get("captured") and not v["reached_d_eps"]
        )
        + (f"Not shown: {'; '.join(missing)}. " if missing else "")
        + "The straight line is the plan's definition of 'log-damped'; smoothness is a property of this simulation, not of a "
        "built drawer."
    )
    title = "terminus_semilog: log-damped capture lines"
    stamp = finish(fig, title=title, question=QUESTION, caption=caption, meta=stamp_meta(cfg, "terminus_semilog", "carriage6"), wspace=0.2)
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "limits": LIMITS,
        "plot_floor": _FLOOR,
        "limit_ratio": 1.0,
        "ideal": {"t_over_tau": floats(u_ideal), "d_ratio": floats(np.exp(-u_ideal))},
        "d_eps_ratio_line": ref_eps,
        "moves": rec,
        "dots": dots,
        "checks": checks,
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="terminus_semilog", tier=1, question=QUESTION, run=run)
