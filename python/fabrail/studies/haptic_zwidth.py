"""haptic_zwidth: the range of virtual stiffness each class can render passively, and the passivity observer.

Claim backed: HAP-009 (virtual stiffness K ≤ K_max = 2b/(T + 2T_d); with the passivity
controller on, the observed energy stays ≥ −1 mJ; with it off, the violation is
reproducible) and the NARRATIVE point that the deliberate damping b_dist is what makes
firm virtual detents and end stops possible: with b → 0, K_max → 0.

Method.

- Z-width (Colgate–Schenkel, PHYSICS §17): the sampled virtual wall is passive while
  K ≤ 2b/(T + 2T_d), with b the physical damping, T the fast-group period and T_d = T the
  one-period delay (native ``passivity_k_max``). Plotted per sample period, with each
  class's rendered virtual stiffness k_virtual_frac·2b/T and its detent slope 2π·F/w at
  b = b_dist (``params.validate`` rejects presets above the bound).
- Closed loop (Carriage6, voltage coil mode): the catalog ``detent_pass`` (a hand drags
  the carriage across its first detent) with the passivity controller as preset and
  with it disabled (``control.haptic.passivity_alpha_ratio`` = 0), reporting the minimum
  passivity-observer energy. HEAVY has no detent and is analysed in closed form only.
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
)

QUICK_AWARE = True

QUESTION = "What range of virtual stiffness can each class render passively (Z-width), and does the passivity observer stay above −1 mJ?"

E_OBS_MIN_J = -1e-3  # FAB-HAP-009
_SIM_CLASSES = ("drawer", "door", "wall")


def run(cfg: Any) -> StudyResult:
    from .. import require_native, scenarios

    n = require_native()
    q = quick()
    sim_classes = ("drawer",) if q else _SIM_CLASSES
    jobs, keys = [], []
    for cls in sim_classes:
        for pc_on in (True, False):
            over = dict(cfg.overrides)
            if not pc_on:
                over["control.haptic.passivity_alpha_ratio"] = 0.0
            c = class_cfg(cfg, cls, overrides=over)
            p = c.params()
            d, s = scenarios.spec("detent_pass", p, c, fidelity="carriage6")
            d["record"] = {"rate_hz": 1000.0, "channels": ["x_m", "v_mps", "e_passivity_j"]}
            jobs.append((p, d, s))
            keys.append((cls, pc_on))
    results = run_jobs(jobs, cfg)

    fig, axes = new_figure(1, 2, size=(15.0, 6.6), gridspec_kw={"width_ratios": [1.2, 1.0]})
    ax_z, ax_e = axes[0]
    b_grid = np.geomspace(0.5, 5000.0, 30 if q else 120)
    rec: dict[str, Any] = {}
    lines: dict[str, Any] = {}
    for cls in CLASSES:
        p = class_cfg(cfg, cls).params()
        st = class_style(cls)
        t = float(p.get("control.rates.fast_decim")) / float(p.get("control.rates.base_hz"))
        key = f"T_{t * 1e6:.0f}us"
        if key not in lines:
            k_line = [float(n.passivity_k_max(float(b), t, t)) for b in b_grid]
            lines[key] = {"t_s": t, "b_n_s_per_m": floats(b_grid), "k_max_n_per_m": k_line}
            ax_z.plot(
                b_grid,
                k_line,
                color=plotstyle.COLORS["ink"],
                lw=1.6 if len(lines) == 1 else 1.1,
                ls="-" if len(lines) == 1 else (0, (5, 3)),
                label=f"K_max = 2b/(T + 2T_d), T = T_d = {t * 1e6:.0f} µs",
            )
        b = float(p.get("design.damping.b_dist_n_s_per_m"))
        k_max = float(n.passivity_k_max(b, t, t))
        k_v = float(p.get("control.haptic.k_virtual_frac")) * 2.0 * b / t
        n_det = int(p.get("control.haptic.n_detents"))
        slope = (
            2.0 * math.pi * float(p.get("control.haptic.detent_f_n")) / float(p.get("control.haptic.detent_width_m")) if n_det > 0 else None
        )
        ax_z.plot([b], [k_v], ls="none", marker=st["marker"], ms=9, color=st["color"], mec=plotstyle.COLORS["surface"], mew=1.3)
        if slope is not None:
            ax_z.plot([b], [slope], ls="none", marker=st["marker"], ms=9, mfc="none", mew=1.8, color=st["color"])
        rec[cls] = {
            "fast_period_us": t * 1e6,
            "b_dist_n_s_per_m": b,
            "k_max_n_per_m": k_max,
            "k_max_derived_n_per_m": float(p.derived()["k_max_passive_n_per_m"]),
            "k_virtual_n_per_m": k_v,
            "k_virtual_over_k_max": k_v / k_max,
            "detent_slope_n_per_m": slope,
            "detent_over_k_max": None if slope is None else slope / k_max,
        }
    ax_z.set_xscale("log")
    ax_z.set_yscale("log")
    ax_z.annotate(
        "passive region (below the line)", xy=(30.0, 30.0 * 2.0 / 3e-4 / 30.0), fontsize=9, color=plotstyle.COLORS["ink2"], rotation=0
    )
    from matplotlib.lines import Line2D

    handles = ax_z.get_legend_handles_labels()[0]
    handles += [
        Line2D([], [], ls="none", marker="o", ms=8, color=plotstyle.COLORS["ink2"], label="rendered virtual stiffness at b_dist"),
        Line2D(
            [], [], ls="none", marker="o", ms=8, mfc="none", mew=1.8, color=plotstyle.COLORS["ink2"], label="detent slope 2π·F/w at b_dist"
        ),
    ]
    handles += [
        Line2D([], [], ls="none", marker=plotstyle.CLASS_MARKERS[c], ms=8, color=plotstyle.CLASS_COLORS[c], label=plotstyle.CLASS_LABELS[c])
        for c in CLASSES
    ]
    ax_z.legend(handles=handles, loc="upper left", **plotstyle.LEGEND_KW)
    plotstyle.style_ax(ax_z, "physical damping b (N·s/m)", "virtual stiffness K (N/m)", "(a) Z-width: the passive stiffness bound")

    sims: dict[str, Any] = {}
    xs, vals, cols, hatches = [], [], [], []
    width = 0.36
    for i, cls in enumerate(sim_classes):
        st = class_style(cls)
        for pc_on in (True, False):
            r = next(rr for (kc, pc), rr in zip(keys, results, strict=True) if kc == cls and pc == pc_on)
            e_min = r.kpis.get("passivity_min_j")
            e_mj = None if e_min is None else e_min * 1e3
            obstruction_warnings = sum(1 for e in r.events if e.kind == "Obstruction")
            sims[f"{cls}_pc_{'on' if pc_on else 'off'}"] = {
                "e_obs_min_j": e_min,
                "e_obs_min_mj": e_mj,
                "faults": faults_of(r),
                "obstruction_events": obstruction_warnings,
                "x_range_mm": [float(np.min(r["x_m"]) * 1e3), float(np.max(r["x_m"]) * 1e3)],
            }
            x = i + (-0.5 if pc_on else 0.5) * (width + 0.03)
            xs.append(x)
            vals.append(0.0 if e_mj is None else e_mj)
            cols.append(st["color"])
            hatches.append(None if pc_on else "///")
    for x, v, c_, h in zip(xs, vals, cols, hatches, strict=True):
        ax_e.bar([x], [v], width, color=c_, alpha=1.0 if h is None else 0.45, hatch=h, edgecolor=plotstyle.COLORS["surface"])
        ax_e.annotate(
            f"{v:.3g} mJ",
            xy=(x, min(v, 0.0)),
            xytext=(0, -3),
            textcoords="offset points",
            ha="center",
            va="top",
            fontsize=8,
            color=plotstyle.COLORS["ink2"],
        )
    plotstyle.ref_line(ax_e, E_OBS_MIN_J * 1e3, "HAP-009: −1 mJ", color=plotstyle.COLORS["star"], va="top")
    plotstyle.zero_line(ax_e)
    ax_e.set_xticks(range(len(sim_classes)), [plotstyle.CLASS_LABELS[c] for c in sim_classes])
    from matplotlib.patches import Patch

    ax_e.legend(
        handles=[
            Patch(facecolor=plotstyle.COLORS["ink2"], label="passivity controller on (preset)"),
            Patch(facecolor=plotstyle.COLORS["ink2"], alpha=0.45, hatch="///", label="controller off (α_max = 0)"),
        ],
        loc="lower right",
        **plotstyle.LEGEND_KW,
    )
    plotstyle.style_ax(ax_e, None, "minimum passivity-observer energy (mJ)", "(b) Carriage6 detent pass")
    checks = {}
    for cls in CLASSES:
        r_ = rec[cls]
        checks[f"{cls}.k_virtual_le_k_max"] = check(
            r_["k_virtual_n_per_m"] <= r_["k_max_n_per_m"] * (1 + 1e-12), r_["k_virtual_over_k_max"], 1.0, "FAB-HAP-009 (K ≤ 2b/(T + 2T_d))"
        )
        if r_["detent_over_k_max"] is not None:
            checks[f"{cls}.detent_le_k_max"] = check(
                r_["detent_over_k_max"] <= 1.0, r_["detent_over_k_max"], 1.0, "FAB-HAP-009 (detent slope)"
            )
    for cls in sim_classes:
        e = sims[f"{cls}_pc_on"]["e_obs_min_j"]
        checks[f"{cls}.pc_on_e_obs"] = check(
            e is not None and e >= E_OBS_MIN_J, e, E_OBS_MIN_J, "FAB-HAP-009 (E_obs ≥ −1 mJ with the controller on), J"
        )
    off = [
        f"{plotstyle.CLASS_LABELS[c]} {sims[f'{c}_pc_off']['e_obs_min_mj']:.2f} mJ"
        for c in sim_classes
        if (sims[f"{c}_pc_off"]["e_obs_min_j"] or 0.0) < E_OBS_MIN_J
    ]
    warn = [
        f"{plotstyle.CLASS_LABELS[c]} ({sims[f'{c}_pc_on']['obstruction_events']})"
        for c in sim_classes
        if sims[f"{c}_pc_on"]["obstruction_events"]
    ]
    caption = (
        "(a) A sampled virtual wall is passive while K ≤ 2b/(T + 2T_d) (Colgate–Schenkel; native passivity_k_max with the "
        "one-period delay T_d = T). The physical damping b sets the bound: with b → 0 no stiffness is passive, which is why "
        "b_dist is kept on purpose. Filled markers: each class's rendered virtual stiffness at its b_dist ("
        + ", ".join(f"{plotstyle.CLASS_LABELS[c]} {rec[c]['k_virtual_over_k_max']:.2f}·K_max" for c in CLASSES)
        + "); open markers: the detent slope 2π·F/w (HEAVY has no detent).\n"
        "(b) Minimum passivity-observer energy over the catalog detent_pass (Carriage6, voltage coil mode) with the passivity "
        "controller as preset and disabled. "
        + (
            f"With it off the observer drops below −1 mJ ({', '.join(off)}); with it on every class stays above. "
            if off
            else "Neither setting crosses −1 mJ in this scenario. "
        )
        + (
            f"Obstruction warnings during the drag (the scenario's gripping hand profile, a known catalog issue): {', '.join(warn)}. "
            if warn
            else ""
        )
        + "The observer energy is a controller quantity; no haptic hardware was measured."
    )
    title = "haptic_zwidth: the passive stiffness range"
    stamp = finish(fig, title=title, question=QUESTION, caption=caption, meta=stamp_meta(cfg, "haptic_zwidth", "carriage6"), wspace=0.24)
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "k_max_lines": lines,
        "classes": rec,
        "sims": sims,
        "bars": {"x": xs, "e_obs_min_mj": vals},
        "e_obs_limit_mj": E_OBS_MIN_J * 1e3,
        "zero_mj": 0.0,
        "checks": checks,
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="haptic_zwidth", tier=1, question=QUESTION, run=run)
