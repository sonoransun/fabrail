"""zero_power_band: where a carriage floats at zero coil current, and what the band clamp does outside it.

Claim backed: LEV-004 (after a load change the coil current settles to |i| < 1 mA and the
gap follows m(g)), LEV-005 (m(g) = m*·((2g* + h′)/(2g + h′))²) and LEV-018 (outside the
modified zero-power band the clamp holds the band edge with the closed-form bias
current ±2 %).

Method. Closed forms from the native ``zero_power_gap`` and ``bias_current``. Closed loop
on Heave1 (voltage coil mode): a levitated hover with the truth mass set by the initial
payload, 4 s; the gap and the coil current are averaged over the last second. Heave1
carries the heave dynamics every class shares (HEAVY Carriage6 roll/sway is an open
Wave-2 item, so all four classes use the same tier here).
"""

from __future__ import annotations

from typing import Any, cast

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
)

QUICK_AWARE = True

QUESTION = (
    "At what gap does each class float with zero coil current across its payload range, and what holds it outside the zero-power band?"
)

_I_SS_MAX_A = 1e-3  # FAB-LEV-004
_GAP_TOL_M = 2e-6  # FAB-LEV-004 (±2 µm)
_BIAS_REL = 0.02  # FAB-LEV-018 (±2 %)


def _expected(n: Any, p: Any, m: float) -> tuple[float, float, float, bool]:
    """(unclamped zero-power gap, held gap, held bias current, in band) for mass m."""
    d = p.derived()
    g_star = float(p.get("design.suspension.g_star_m"))
    hp = float(d["h_prime_m"])
    m_star = float(p.get("design.carriage.m_star_kg"))
    lo, hi = float(p.get("design.suspension.zp_band_m.0")), float(p.get("design.suspension.zp_band_m.1"))
    g_zp = float(n.zero_power_gap(m, g_star, hp, m_star))
    if lo <= g_zp <= hi:
        return g_zp, g_zp, 0.0, True
    g = min(max(g_zp, lo), hi)
    return g_zp, g, float(n.bias_current(p, g, m)), False


def run(cfg: Any) -> StudyResult:
    from .. import require_native, scenarios

    n = require_native()
    q = quick()
    dur = 3.0 if q else 4.0
    fig, axes = new_figure(2, 4, size=(16.0, 8.4), sharex="col")
    rec: dict[str, Any] = {}
    checks: dict[str, Any] = {}
    jobs, keys = [], []
    for cls in CLASSES:
        p = class_cfg(cfg, cls).params()
        d = p.derived()
        m_star = float(p.get("design.carriage.m_star_kg"))
        g_star = float(p.get("design.suspension.g_star_m"))
        hp = float(d["h_prime_m"])
        lo, hi = float(p.get("design.suspension.zp_band_m.0")), float(p.get("design.suspension.zp_band_m.1"))
        m_band = (float(n.zero_power_mass(hi, g_star, hp, m_star)), float(n.zero_power_mass(lo, g_star, hp, m_star)))
        m_op = (float(p.get("design.carriage.m_op_min_kg")), float(p.get("design.carriage.m_rated_kg")))
        m_plot = (0.9 * min(m_band[0], m_op[0]), 1.1 * max(m_band[1], m_op[1]))
        masses = np.linspace(*m_plot, 60 if q else 200)
        exp = [_expected(n, p, float(m)) for m in masses]
        m_sim = (min(m_band[0], m_op[0]), m_plot[1])  # the light end stays at the band/operating minimum (see the caption)
        sim_m = [m_sim[0], m_star, m_sim[1]] if q else sorted({*np.linspace(m_sim[0], m_sim[1], 7).tolist(), m_star, *m_op})
        for m in sim_m:
            c = class_cfg(cfg, cls, duration_s=dur)
            pp = c.params()
            spec_d, series = scenarios.spec("hover", pp, c, fidelity="heave1")
            spec_d["name"] = f"{cls}_zero_power_{m:.3f}kg"
            cast("dict[str, Any]", spec_d["initial"])["payload_kg"] = float(m - m_star)
            spec_d["record"] = {"rate_hz": 1000.0, "channels": ["gap_0_m", "coil_0_a"]}
            spec_d["metrics"] = {"windows": {}}
            jobs.append((pp, spec_d, series))
            keys.append((cls, float(m)))
        rec[cls] = {
            "m_star_kg": m_star,
            "g_star_mm": g_star * 1e3,
            "h_prime_mm": hp * 1e3,
            "band_mm": [lo * 1e3, hi * 1e3],
            "band_mass_kg": list(m_band),
            "m_op_kg": list(m_op),
            "sim_mass_range_kg": list(m_sim),
            "curve_m_kg": floats(masses),
            "curve_gap_zero_power_mm": [e[0] * 1e3 for e in exp],
            "curve_gap_held_mm": [e[1] * 1e3 for e in exp],
            "curve_bias_current_a": [e[2] for e in exp],
            "zero_current_a": 0.0,
        }
    results = run_jobs(jobs, cfg)
    for j, cls in enumerate(CLASSES):
        st = class_style(cls)
        r_cls = rec[cls]
        p = class_cfg(cfg, cls).params()
        ax_g, ax_i = axes[0, j], axes[1, j]
        pts: dict[str, list[Any]] = {
            "m_kg": [],
            "gap_mm": [],
            "current_a": [],
            "expected_gap_mm": [],
            "expected_current_a": [],
            "in_band": [],
            "gap_err_um": [],
            "faults": [],
        }
        for (kc, m), r in zip(keys, results, strict=True):
            if kc != cls:
                continue
            w = r.t_s >= r.t_s[-1] - 1.0
            gap = float(np.mean(r["gap_0_m"][w]))
            cur = float(np.mean(r["coil_0_a"][w]))
            _, g_exp, i_exp, inside = _expected(n, p, m)
            pts["m_kg"].append(m)
            pts["gap_mm"].append(gap * 1e3)
            pts["current_a"].append(cur)
            pts["expected_gap_mm"].append(g_exp * 1e3)
            pts["expected_current_a"].append(i_exp)
            pts["in_band"].append(inside)
            pts["gap_err_um"].append((gap - g_exp) * 1e6)
            pts["faults"].append(faults_of(r))
            tag = f"{cls}.{m:.3f}kg"
            checks[f"{tag}.gap"] = check(
                abs(gap - g_exp) <= _GAP_TOL_M, (gap - g_exp) * 1e6, _GAP_TOL_M * 1e6, "FAB-LEV-004/018 (gap vs m(g) or the band edge), µm"
            )
            if inside:
                checks[f"{tag}.i_ss"] = check(abs(cur) < _I_SS_MAX_A, cur * 1e3, _I_SS_MAX_A * 1e3, "FAB-LEV-004 (|i_ss| < 1 mA), mA")
            else:
                rel = abs(cur - i_exp) / abs(i_exp) if i_exp else float("inf")
                checks[f"{tag}.bias"] = check(rel <= _BIAS_REL, rel, _BIAS_REL, "FAB-LEV-018 (bias current vs closed form, relative)")
        r_cls["sim"] = pts
        m = np.array(r_cls["curve_m_kg"])
        ax_g.plot(m, r_cls["curve_gap_zero_power_mm"], color=plotstyle.COLORS["ctrl"], lw=1.1, ls=(0, (3, 2)))
        ax_g.plot(m, r_cls["curve_gap_held_mm"], color=st["color"], lw=2.0)
        ax_g.plot(
            pts["m_kg"], pts["gap_mm"], ls="none", marker=st["marker"], ms=7.5, color=st["color"], mec=plotstyle.COLORS["surface"], mew=1.2
        )
        for edge in r_cls["band_mm"]:
            ax_g.axhline(edge, color=plotstyle.COLORS["muted"], ls=(0, (4, 4)), lw=1.0)
        ax_i.plot(m, r_cls["curve_bias_current_a"], color=st["color"], lw=2.0)
        ax_i.plot(
            pts["m_kg"],
            pts["current_a"],
            ls="none",
            marker=st["marker"],
            ms=7.5,
            color=st["color"],
            mec=plotstyle.COLORS["surface"],
            mew=1.2,
        )
        plotstyle.zero_line(ax_i)
        for ax in (ax_g, ax_i):
            ax.axvspan(r_cls["m_op_kg"][0], r_cls["m_op_kg"][1], color=plotstyle.COLORS["band"], alpha=0.08, lw=0)
        plotstyle.style_ax(ax_g, None, "float gap (mm)" if j == 0 else None, st["label"])
        plotstyle.style_ax(ax_i, "carriage mass (kg)", "mean coil current (A)" if j == 0 else None)
        ax_g.annotate(
            "band",
            xy=(0.98, r_cls["band_mm"][1]),
            xycoords=("axes fraction", "data"),
            ha="right",
            va="bottom",
            fontsize=8,
            color=plotstyle.COLORS["muted"],
        )
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    handles = [
        Line2D([], [], color=plotstyle.COLORS["ink2"], lw=2.0, label="held gap / current (closed form)"),
        Line2D([], [], color=plotstyle.COLORS["ctrl"], lw=1.1, ls=(0, (3, 2)), label="unclamped m(g)"),
        Line2D([], [], color=plotstyle.COLORS["ink2"], ls="none", marker="o", ms=7, label="Heave1 closed loop"),
        Patch(facecolor=plotstyle.COLORS["band"], alpha=0.15, label="operating mass range"),
    ]
    axes[0, 0].legend(handles=handles, loc="center right", **plotstyle.LEGEND_KW)

    in_band = [(c, i) for c in CLASSES for i, ok in enumerate(rec[c]["sim"]["in_band"]) if ok]
    out_band = [(c, i) for c in CLASSES for i, ok in enumerate(rec[c]["sim"]["in_band"]) if not ok]
    worst_gap = max(abs(rec[c]["sim"]["gap_err_um"][i]) for c in CLASSES for i in range(len(rec[c]["sim"]["m_kg"])))
    worst_i = max((abs(rec[c]["sim"]["current_a"][i]) for c, i in in_band), default=0.0)
    worst_bias = max(
        (
            abs(rec[c]["sim"]["current_a"][i] - rec[c]["sim"]["expected_current_a"][i]) / abs(rec[c]["sim"]["expected_current_a"][i])
            for c, i in out_band
            if rec[c]["sim"]["expected_current_a"][i]
        ),
        default=0.0,
    )
    mismatch = [
        f"{plotstyle.CLASS_LABELS[c]} ≥ {min(rec[c]['sim']['m_kg'][i] for i, f in enumerate(rec[c]['sim']['faults']) if any('PayloadMismatch' in x for x in f)):.0f} kg"
        for c in CLASSES
        if any(any("PayloadMismatch" in x for x in f) for f in rec[c]["sim"]["faults"])
    ]
    drawer = rec["drawer"]
    caption = (
        "Top: the gap a carriage of mass m floats at. Inside the zero-power band (dashed) it is m(g) at zero coil current "
        "(LEV-005); outside it the band clamp holds the band edge (LEV-018). Bottom: the steady coil current — zero inside the "
        "band, the closed-form bias current (native bias_current at the band edge) outside. Markers: Heave1 closed loop, voltage "
        f"coil mode, {dur:.0f} s hover, last-second means. Worst gap error {worst_gap:.2f} µm (LEV-004: ±2 µm); worst in-band "
        f"current {worst_i * 1e3:.3f} mA (< 1 mA); worst bias-current error {worst_bias * 100:.2f} % (LEV-018: ±2 %). The DRAWER "
        f"band {drawer['band_mm'][0]:.1f}–{drawer['band_mm'][1]:.1f} mm covers {drawer['band_mass_kg'][0]:.1f}–"
        f"{drawer['band_mass_kg'][1]:.1f} kg, so its rated {drawer['m_op_kg'][1]:.0f} kg floats at the 1.5 mm edge with bias "
        "current (core design F1). Masses above 1.05·m_rated raise PayloadMismatch (Degrade, hold-to-run)"
        + (f"; the simulated points that raised it start at {', '.join(mismatch)}." if mismatch else ".")
        + " Simulated masses start at min(band, m_op,min): a carriage released at g* with a much lighter truth mass is a "
        "different, transient test. Heave only; a simulated steady state with sensor noise but no gap-probe bias or tempco (they default to 0 in the sim), "
        "not a measured hover current."
    )
    title = "zero_power_band: float gap and holding current versus mass"
    stamp = finish(
        fig, title=title, question=QUESTION, caption=caption, meta=stamp_meta(cfg, "zero_power_band", "heave1"), hspace=0.18, wspace=0.26
    )
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "duration_s": dur,
        "classes": rec,
        "checks": checks,
        "tolerances": {"gap_um": _GAP_TOL_M * 1e6, "i_ss_ma": _I_SS_MAX_A * 1e3, "bias_rel": _BIAS_REL},
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="zero_power_band", tier=1, question=QUESTION, run=run)
