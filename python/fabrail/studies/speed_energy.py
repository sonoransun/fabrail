"""speed_energy: which pedestrian limit sets each class's speed cap, and how close governed moves come to it.

Claims backed: SAF-002 (unsensed caps DRAWER 0.2654 m/s, DOOR open 0.1838 / close 0.152, WALL
0.0446, HEAVY 0.025 hold-to-run), SAF-001 (the governor keeps v within the energy and
momentum caps with m_gov = max(m_rated, 1.05·m̂)) and PRP-005 (the stopping distance at
the cap). The Wave-2 lead decision D4 sets a planning margin (≤ 0.90 of the KE cap, i.e.
≈ 0.949 of the energy-limited speed; ≤ 0.95 of the momentum cap); this study measures the
current build against it.

Method. Closed forms from the native ``v_low_energy`` and ``momentum_speed_cap`` (the
pedestrian caps 1.69 J and 22.3 kg·m/s live in ``fabrail_params::presets`` and are not
restated here) and ``stop_distance``. Closed loop, voltage coil mode: the catalog
``soft_open`` and ``terminus_soft_close`` moves (Carriage6 for DRAWER, DOOR and WALL;
Travel1d for HEAVY, whose Carriage6 roll/sway loop is an open item), with the peak truth
speed in the move direction and the run's governor-energy and momentum KPIs.
"""

from __future__ import annotations

import math
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

QUESTION = "Which pedestrian limit sets each class's unsensed speed cap, and how close do governed moves come to it?"

D4_KE_FRAC = 0.90  # lead decision D4 (Wave 2): governor plans at ≤ 0.90 of the KE cap
D4_P_FRAC = 0.95  # lead decision D4: ≤ 0.95 of the momentum cap
_MOVES = (("open", "soft_open"), ("close", "terminus_soft_close"))


def _tier(cls: str) -> str:
    return "travel1d" if cls == "heavy" else "carriage6"


def _limit_kind(n: Any, p: Any, v_cap: float) -> str:
    m = float(p.get("design.carriage.m_rated_kg"))
    e = float(p.get("safety.e_max_j"))
    pm = float(p.get("safety.p_max_kgmps"))
    if abs(v_cap - float(n.v_low_energy(m, e))) <= 1e-9:
        return "energy"
    if pm > 0.0 and abs(v_cap - float(n.momentum_speed_cap(m, pm))) <= 1e-9:
        return "momentum"
    return "class rule"


def run(cfg: Any) -> StudyResult:
    from .. import require_native, scenarios

    n = require_native()
    q = quick()
    jobs, keys = [], []
    for cls in CLASSES:
        c = class_cfg(cfg, cls, record_rate_hz=500.0)
        p = c.params()
        for direction, name in _MOVES:
            d, s = scenarios.spec(name, p, c, fidelity=_tier(cls))
            d["record"] = {"rate_hz": 500.0, "channels": ["x_m", "v_mps"]}
            jobs.append((p, d, s))
            keys.append((cls, direction))
    results = run_jobs(jobs, cfg)

    fig, axes = new_figure(1, 3, size=(16.0, 6.6), gridspec_kw={"width_ratios": [1.15, 1.0, 1.0]})
    ax_c, ax_b, ax_d = axes[0]
    p0 = class_cfg(cfg, "drawer").params()
    e_cap = float(p0.get("safety.e_max_j"))
    p_cap = float(p0.get("safety.p_max_kgmps"))
    masses = np.geomspace(1.0, 2.0e4, 60 if q else 240)
    v_e = [float(n.v_low_energy(float(m), e_cap)) for m in masses]
    v_p = [float(n.momentum_speed_cap(float(m), p_cap)) for m in masses]
    v_min = [min(a, b) for a, b in zip(v_e, v_p, strict=True)]
    m_x = p_cap * p_cap / (2.0 * e_cap)
    ax_c.plot(masses, v_e, color=plotstyle.COLORS["ctrl"], lw=1.1, ls=(0, (3, 2)), label=tex_safe(f"energy √(2E/m), E = {e_cap:g} J"))
    ax_c.plot(masses, v_p, color=plotstyle.COLORS["ctrl"], lw=1.1, ls=(0, (1, 2)), label=f"momentum p/m, p = {p_cap:g} kg·m/s")
    ax_c.plot(masses, v_min, color=plotstyle.COLORS["ink"], lw=1.8, label="min of the two")
    ax_c.axvline(m_x, color=plotstyle.COLORS["muted"], lw=1.0, ls=(0, (4, 4)))
    ax_c.annotate(
        f"crossover {m_x:.0f} kg",
        xy=(m_x, 0.98),
        xycoords=("data", "axes fraction"),
        xytext=(4, 0),
        textcoords="offset points",
        va="top",
        fontsize=8.5,
        color=plotstyle.COLORS["muted"],
    )
    rec: dict[str, Any] = {}
    checks: dict[str, Any] = {}
    bar_vals, bar_x, bar_cols, bar_alpha, target_x, target_y = [], [], [], [], [], []
    width = 0.36
    for i, cls in enumerate(CLASSES):
        p = class_cfg(cfg, cls).params()
        st = class_style(cls)
        m_rated = float(p.get("design.carriage.m_rated_kg"))
        a_max = float(p.get("control.motion.a_max_mps2"))
        j_max = float(p.get("control.motion.j_max_mps3"))
        caps = {"open": float(p.get("safety.v_open_max_mps")), "close": float(p.get("safety.v_close_max_mps"))}
        crec: dict[str, Any] = {"m_rated_kg": m_rated, "caps_mps": caps, "moves": {}}
        for k, direction in enumerate(("open", "close")):
            cap = caps[direction]
            kind = _limit_kind(n, p, cap)
            ax_c.plot(
                [m_rated],
                [cap],
                ls="none",
                marker=st["marker"],
                ms=9,
                color=st["color"],
                mfc=st["color"] if direction == "open" else "none",
                mew=1.8,
                mec=plotstyle.COLORS["surface"] if direction == "open" else st["color"],
            )
            r = next(rr for (kc, dd), rr in zip(keys, results, strict=True) if kc == cls and dd == direction)
            sign = 1.0 if direction == "open" else -1.0
            v_peak = float(np.max(sign * r["v_mps"]))
            ke_gov = r.kpis.get("ke_max_gov_j")
            mom = r.kpis.get("momentum_max")
            target = math.sqrt(D4_KE_FRAC) if kind == "energy" else (D4_P_FRAC if kind == "momentum" else None)
            ratio = v_peak / cap
            crec["moves"][direction] = {
                "limit": kind,
                "v_cap_mps": cap,
                "v_peak_mps": v_peak,
                "v_peak_over_cap": ratio,
                "d4_target_ratio": target,
                "ke_max_gov_j": ke_gov,
                "ke_gov_over_cap": None if ke_gov is None else ke_gov / e_cap,
                "momentum_max_kgmps": mom,
                "momentum_over_cap": None
                if (mom is None or p_cap <= 0 or float(p.get("safety.p_max_kgmps")) <= 0)
                else mom / float(p.get("safety.p_max_kgmps")),
                "faults": faults_of(r),
                "tier": _tier(cls),
                "d_stop_at_cap_mm": float(n.stop_distance(cap, a_max, j_max)) * 1e3,
            }
            x = i + (k - 0.5) * (width + 0.03)
            bar_vals.append(ratio)
            bar_x.append(x)
            bar_cols.append(st["color"])
            bar_alpha.append(1.0 if direction == "open" else 0.5)
            if target is not None:
                target_x.append([x - width / 2, x + width / 2])
                target_y.append(target)
            checks[f"{cls}.{direction}.within_cap"] = check(ratio <= 1.0, ratio, 1.0, "FAB-SAF-001/002 (peak truth speed / class cap)")
            if target is not None:
                checks[f"{cls}.{direction}.d4_margin"] = check(
                    ratio <= target + 1e-9, ratio, target, "lead decision D4 planning margin (reported)"
                )
        # (c) stopping distance versus speed.
        v_top = max(caps.values())
        vs = np.linspace(0.0, 1.25 * v_top, 30 if q else 120)
        dst = [float(n.stop_distance(float(v), a_max, j_max)) * 1e3 for v in vs]
        ax_d.plot(vs * 1e3, dst, color=st["color"], lw=1.8, label=st["label"])
        d_cap = float(p.derived()["d_stop_m"]) * 1e3
        v_cap_open = caps["open"] * 1e3
        ax_d.plot(
            [v_cap_open], [d_cap], ls="none", marker=st["marker"], ms=8.5, color=st["color"], mec=plotstyle.COLORS["surface"], mew=1.2
        )
        crec.update(
            {
                "stop_curve_v_mm_s": floats(vs * 1e3),
                "stop_curve_d_mm": dst,
                "d_stop_cap_mm": d_cap,
                "v_cap_open_mm_s": v_cap_open,
                "a_max_mps2": a_max,
                "j_max_mps3": j_max,
            }
        )
        rec[cls] = crec
    for x, v, c_, a in zip(bar_x, bar_vals, bar_cols, bar_alpha, strict=True):
        ax_b.bar([x], [v], width, color=c_, alpha=a)
    for xs_, y in zip(target_x, target_y, strict=True):
        ax_b.plot(xs_, [y, y], color=plotstyle.COLORS["star"], lw=2.2)
    plotstyle.ref_line(ax_b, 1.0, "class cap", color=plotstyle.COLORS["ink2"])
    ax_b.set_xticks(range(len(CLASSES)), [plotstyle.CLASS_LABELS[c] for c in CLASSES])
    ax_b.set_ylim(0.0, 1.3)
    plotstyle.style_ax(ax_b, None, "peak speed / class cap", "(b) Governed moves: open (solid), close (light)")
    from matplotlib.lines import Line2D

    ax_b.legend(
        handles=[
            Line2D(
                [],
                [],
                color=plotstyle.COLORS["star"],
                lw=2.2,
                label=tex_safe("D4 planning target (√(0.90) of the energy cap speed, 0.95 momentum)"),
            )
        ],
        loc="upper left",
        **plotstyle.LEGEND_KW,
    )
    ax_c.set_xscale("log")
    ax_c.set_yscale("log")
    plotstyle.style_ax(ax_c, "rated mass (kg)", "unsensed speed cap (m/s)", "(a) Energy or momentum sets the cap")
    ax_c.legend(loc="lower left", **plotstyle.LEGEND_KW)
    plotstyle.style_ax(ax_d, "speed (mm/s)", "stopping distance at a_max, j_max (mm)", "(c) Stopping distance; markers at the open cap")
    ax_d.legend(loc="upper left", **plotstyle.LEGEND_KW)

    lim = {c: rec[c]["moves"]["open"]["limit"] for c in CLASSES}
    over = [
        f"{plotstyle.CLASS_LABELS[c]} {d} {rec[c]['moves'][d]['v_peak_over_cap']:.3f}"
        for c in CLASSES
        for d in ("open", "close")
        if rec[c]["moves"][d]["d4_target_ratio"] is not None
        and rec[c]["moves"][d]["v_peak_over_cap"] > rec[c]["moves"][d]["d4_target_ratio"]
    ]
    faulty = [
        f"{plotstyle.CLASS_LABELS[c]} {d} ({', '.join(rec[c]['moves'][d]['faults'])})"
        for c in CLASSES
        for d in ("open", "close")
        if rec[c]["moves"][d]["faults"]
    ]
    caption = (
        f"(a) The pedestrian caps from fabrail_params::presets: kinetic energy {e_cap:g} J and momentum {p_cap:g} kg·m/s; below "
        f"{m_x:.0f} kg energy binds, above it momentum. Markers at m_rated: open cap (filled), close cap (open). Limits: "
        + ", ".join(f"{plotstyle.CLASS_LABELS[c]} {lim[c]}" for c in CLASSES)
        + " (DOOR close 0.152 m/s and HEAVY 0.025 m/s hold-to-run are class rules; HEAVY's 125 kg·m/s is stated, not capped).\n"
        "(b) Peak truth speed of the catalog soft_open and terminus_soft_close moves over the class cap (voltage coil mode; "
        "Carriage6, HEAVY on Travel1d). Red ticks: the D4 planning target. "
        + (f"Above it in this build: {'; '.join(over)}. " if over else "Every move is inside the D4 target. ")
        + (f"Moves with raised faults: {'; '.join(faulty)}. " if faulty else "")
        + "(c) Stopping distance of the jerk-limited stop at a_max and j_max (native stop_distance; PRP-005 anchors at the caps). "
        "Limits are checked in simulation against numeric values from A156.19/A156.38 and A156.10 as cited in SPECIFICATION; "
        "no standards compliance is claimed."
    )
    title = "speed_energy: what sets each speed cap"
    stamp = finish(
        fig, title=title, question=QUESTION, caption=caption, meta=stamp_meta(cfg, "speed_energy", "carriage6+travel1d"), wspace=0.28
    )
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "e_cap_j": e_cap,
        "p_cap_kgmps": p_cap,
        "crossover_mass_kg": m_x,
        "cap_curves": {"m_kg": floats(masses), "v_energy_mps": v_e, "v_momentum_mps": v_p, "v_cap_mps": v_min},
        "bars": {"x": bar_x, "v_peak_over_cap": bar_vals, "target_x": target_x, "target_ratio": target_y, "cap_ratio": 1.0},
        "d4": {"ke_frac": D4_KE_FRAC, "p_frac": D4_P_FRAC, "speed_frac_energy": math.sqrt(D4_KE_FRAC)},
        "classes": rec,
        "checks": checks,
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="speed_energy", tier=1, question=QUESTION, run=run)
