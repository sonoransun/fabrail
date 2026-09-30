"""drag_ledger: the levitated drag ledger and μ_eq, over the uncertain rail-drag range.

Claim backed: HAP-010 (μ_eq at 0.1 m/s is reported, lies within [1e-3, 1e-2] at defaults
and is never 0) and the NARRATIVE reading of the README's friction wording as a direction
(μ_eq ≈ 1e-3 to 1e-2), not a number. The rail-drag values c_rail and F_hyst are
[UNCERTAIN] (core §C.5); this study sweeps them by a factor of 4 either way (a study
choice, not a measured range).

Method.

- Closed-form ledger at v = 0.1 m/s and m* (PHYSICS §9): deliberate damping b_dist·v, rail
  eddy drag c_rail·v, rail hysteresis F_hyst and, on the DRAWER, the cavity piston drag
  through its leak orifice (native ``piston_force``). Aerodynamic drag of the moving face
  is in the plant but not in this closed form (it is ≤ 1e-4 of the weight here).
  μ = F/(m·g₀), so a lighter carriage on the same rail shows a larger μ.
- Simulation: a levitated coast from 0.1 m/s at mid-travel with the LSM inverter latched
  off (``fault: lsm_driver``) so no thrust acts, 0.2 s, voltage coil mode, on Travel1d at
  the nominal and swept rail drag (``plant.c_rail_n_s_per_m``, ``plant.f_hyst_n``) and on
  Carriage6 at the nominal; μ_eq is the plant's ledger KPI (drag energy / (m·g₀·distance)).
"""

from __future__ import annotations

import json
from typing import Any

import numpy as np

from .. import plotstyle
from ..ground import G0_MPS2
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

QUESTION = "What equivalent friction coefficient μ_eq does each class's levitated drag ledger give, and how far does it move over the uncertain rail drag?"

V_REF_MPS = 0.1  # FAB-HAP-010 reference speed
SWEEP = (0.25, 1.0, 4.0)  # study choice: rail-drag factor applied to c_rail and F_hyst together
BAND = (1e-3, 1e-2)  # FAB-HAP-010 band
_COMPONENTS = ("b_dist", "c_rail", "f_hyst", "piston")
_COMPONENT_LABELS = {
    "b_dist": "deliberate damping b_dist·v",
    "c_rail": "rail eddy c_rail·v",
    "f_hyst": "rail hysteresis F_hyst",
    "piston": "cavity piston (DRAWER)",
}
_ALPHA = {"b_dist": 1.0, "c_rail": 0.68, "f_hyst": 0.42, "piston": 0.22}


def ledger(p: Any, v_mps: float = V_REF_MPS, factor: float = 1.0) -> dict[str, float]:
    """Closed-form drag components at speed v with the rail terms scaled by ``factor``, N."""
    from .. import require_native

    n = require_native()
    return {
        "b_dist": float(p.get("design.damping.b_dist_n_s_per_m")) * v_mps,
        "c_rail": factor * float(p.get("design.damping.c_rail_n_s_per_m")) * v_mps,
        "f_hyst": factor * float(p.get("design.damping.f_hyst_n")),
        "piston": float(n.piston_force(float(p.get("design.air.cavity_area_m2")), v_mps, float(p.get("design.air.leak_area_m2")))),
    }


def _coast_job(cfg: Any, cls: str, tier: str, factor: float) -> tuple[Any, dict[str, Any], dict[str, bytes]]:
    from .. import require_native
    from ..ensemble import split_seed

    n = require_native()
    p = class_cfg(cfg, cls).params()
    x_mid = float(p.get("design.rail.x_closed_m")) + 0.5 * float(p.get("design.rail.travel_m"))
    plant: dict[str, Any] = {}
    if factor != 1.0:
        plant = {
            "c_rail_n_s_per_m": factor * float(p.get("design.damping.c_rail_n_s_per_m")),
            "f_hyst_n": factor * float(p.get("design.damping.f_hyst_n")),
        }
    spec = {
        "schema": n.SCENARIO_SCHEMA,
        "name": f"{cls}_coast_{tier}_x{factor:g}",
        "doc": "levitated coast from 0.1 m/s with the LSM inverter latched off: drag ledger and mu_eq (FAB-HAP-010)",
        "duration_s": 0.2,
        "noise_seed": split_seed(cfg.seed)[1],
        "fidelity": {"model": tier, "coil": "voltage"},
        "initial": {"lev": "levitated", "x_m": x_mid, "v_mps": V_REF_MPS},
        "plant": plant,
        "record": {"rate_hz": 1000.0, "channels": ["x_m", "v_mps"]},
        "events": [{"kind": "fault", "t_s": 0.0, "what": "lsm_driver", "ch": 0}],
    }
    json.dumps(spec)
    return p, spec, {}


def run(cfg: Any) -> StudyResult:
    q = quick()
    jobs, keys = [], []
    for cls in CLASSES:
        for factor in SWEEP:
            jobs.append(_coast_job(cfg, cls, "travel1d", factor))
            keys.append((cls, "travel1d", factor))
        jobs.append(_coast_job(cfg, cls, "carriage6", 1.0))
        keys.append((cls, "carriage6", 1.0))
    results = run_jobs(jobs, cfg)
    rec: dict[str, Any] = {}
    checks: dict[str, Any] = {}
    fig, axes = new_figure(1, 2, size=(15.0, 6.6), gridspec_kw={"width_ratios": [1.0, 1.05]})
    ax_l, ax_m = axes[0]
    ys = np.arange(len(CLASSES), dtype=float)[::-1]
    for y, cls in zip(ys, CLASSES, strict=True):
        p = class_cfg(cfg, cls).params()
        st = class_style(cls)
        m_star = float(p.get("design.carriage.m_star_kg"))
        comp = ledger(p)
        mu_milli = {k: v / (m_star * G0_MPS2) * 1e3 for k, v in comp.items()}
        left = 0.0
        seg_left, seg_width = [], []
        for k in _COMPONENTS:
            w = mu_milli[k]
            if w <= 0.0:
                continue
            ax_l.barh(y, w, 0.56, left=left, color=st["color"], alpha=_ALPHA[k], edgecolor=plotstyle.COLORS["surface"], lw=1.5)
            seg_left.append(left)
            seg_width.append(w)
            left += w
        sims: dict[str, dict[str, Any]] = {}
        for (kc, tier, factor), r in zip(keys, results, strict=True):
            if kc != cls:
                continue
            mu = r.kpis.get("mu_eq")
            sims[f"{tier}_x{factor:g}"] = {
                "mu_eq": mu,
                "mu_eq_milli": None if mu is None else mu * 1e3,
                "v_end_mps": float(r["v_mps"][-1]),
                "faults": faults_of(r),
            }
        sim_nom = [sims["travel1d_x1"]["mu_eq_milli"], sims["carriage6_x1"]["mu_eq_milli"]]
        ax_l.plot(sim_nom[0], y + 0.36, ls="none", marker="D", ms=7, color=plotstyle.COLORS["ink"])
        ax_l.plot(sim_nom[1], y + 0.36, ls="none", marker="o", ms=7, mfc="none", mew=1.6, color=plotstyle.COLORS["ink"])
        # (b) μ versus mass for the swept rail drag.
        m_op = (float(p.get("design.carriage.m_op_min_kg")), float(p.get("design.carriage.m_rated_kg")))
        masses = np.geomspace(0.5 * m_op[0], 2.0 * m_op[1], 24 if q else 80)
        curves: dict[str, Any] = {}
        for factor in SWEEP:
            f_tot = sum(ledger(p, factor=factor).values())
            mu_curve = f_tot / (masses * G0_MPS2)
            curves[f"x{factor:g}"] = floats(mu_curve)
            ax_m.plot(
                masses,
                mu_curve,
                color=st["color"],
                lw=2.0 if factor == 1.0 else 1.0,
                ls="-" if factor == 1.0 else (0, (3, 2)),
                label=st["label"] if factor == 1.0 else None,
            )
        sim_sweep = [sims[f"travel1d_x{factor:g}"]["mu_eq"] for factor in SWEEP]
        ax_m.plot(
            [m_star] * len(SWEEP),
            sim_sweep,
            ls="none",
            marker=st["marker"],
            ms=7.5,
            color=st["color"],
            mec=plotstyle.COLORS["surface"],
            mew=1.2,
        )
        total = sum(comp.values())
        rec[cls] = {
            "m_star_kg": m_star,
            "ledger_n": comp,
            "ledger_total_n": total,
            "mu_ledger": total / (m_star * G0_MPS2),
            "mu_components_milli": mu_milli,
            "bar_y": y,
            "bar_left_milli": seg_left,
            "bar_width_milli": seg_width,
            "sim_marker_y": y + 0.36,
            "sim": sims,
            "mass_curve_kg": floats(masses),
            "mu_vs_mass": curves,
            "sim_sweep_mass_kg": [m_star] * len(SWEEP),
            "sim_sweep_mu_eq": sim_sweep,
            "m_op_kg": list(m_op),
        }
        for tier in ("travel1d", "carriage6"):
            mu_sim = sims[f"{tier}_x1"]["mu_eq"]
            checks[f"{cls}.{tier}.hap010_band"] = check(
                mu_sim is not None and BAND[0] <= mu_sim <= BAND[1], mu_sim, list(BAND), "FAB-HAP-010 (defaults, 0.1 m/s coast)"
            )
        rel = (sims["travel1d_x1"]["mu_eq"] - rec[cls]["mu_ledger"]) / rec[cls]["mu_ledger"]
        rec[cls]["sim_vs_ledger_rel"] = rel
        checks[f"{cls}.sim_vs_closed_form"] = check(
            abs(rel) <= 0.10, rel, 0.10, "coast μ_eq vs the closed-form ledger (±10 %; aero and deceleration not in the closed form)"
        )

    rollers = float(class_cfg(cfg, "door").params().get("design.landing.rollers_mu"))
    refs = {
        "band_low_milli": BAND[0] * 1e3,
        "band_high_milli": BAND[1] * 1e3,
        "rollers_mu_milli": rollers * 1e3,
        "band_low": BAND[0],
        "band_high": BAND[1],
        "rollers_mu": rollers,
    }
    for xv in (refs["band_low_milli"], refs["band_high_milli"]):
        ax_l.axvline(xv, color=plotstyle.COLORS["muted"], ls=(0, (4, 4)), lw=1.1)
    ax_l.axvline(refs["rollers_mu_milli"], color=plotstyle.COLORS["star"], ls=(0, (4, 4)), lw=1.1)
    ax_l.annotate(
        "rollers μ 0.003",
        xy=(refs["rollers_mu_milli"], 1.0),
        xycoords=("data", "axes fraction"),
        xytext=(3, -2),
        textcoords="offset points",
        va="top",
        fontsize=8.5,
        color=plotstyle.COLORS["star"],
    )
    ax_l.annotate(
        "HAP-010 band",
        xy=(refs["band_high_milli"], 1.0),
        xycoords=("data", "axes fraction"),
        xytext=(-3, -2),
        textcoords="offset points",
        va="top",
        ha="right",
        fontsize=8.5,
        color=plotstyle.COLORS["muted"],
    )
    ax_l.set_yticks(ys, [plotstyle.CLASS_LABELS[c] for c in CLASSES])
    ax_l.set_xlim(0.0, refs["band_high_milli"] * 1.08)
    plotstyle.style_ax(ax_l, "μ = F/(m*·g₀)  (×10⁻³)", None, "(a) Ledger at 0.1 m/s and m*", grid=False)
    ax_l.grid(axis="x", **plotstyle.GRID_KW)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    handles: list[Any] = [Patch(facecolor=plotstyle.COLORS["ink2"], alpha=_ALPHA[k], label=_COMPONENT_LABELS[k]) for k in _COMPONENTS]
    handles += [
        Line2D([], [], ls="none", marker="D", ms=7, color=plotstyle.COLORS["ink"], label="coast μ_eq, Travel1d"),
        Line2D([], [], ls="none", marker="o", ms=7, mfc="none", mew=1.6, color=plotstyle.COLORS["ink"], label="coast μ_eq, Carriage6"),
    ]
    ax_l.legend(handles=handles, loc="lower right", **plotstyle.LEGEND_KW)
    ax_m.set_xscale("log")
    ax_m.set_yscale("log")
    plotstyle.ref_line(ax_m, BAND[1], "μ_eq = 1e-2", va="top")
    plotstyle.ref_line(ax_m, BAND[0], "μ_eq = 1e-3")
    plotstyle.ref_line(ax_m, rollers, "rollers μ 0.003", color=plotstyle.COLORS["star"])
    plotstyle.style_ax(ax_m, "carriage mass (kg)", "μ_eq at 0.1 m/s", "(b) Versus mass; rail drag ×0.25 / ×1 / ×4")
    ax_m.legend(loc="lower left", **plotstyle.LEGEND_KW)

    lo_out = [
        f"{plotstyle.CLASS_LABELS[c]} {rec[c]['sim']['travel1d_x0.25']['mu_eq']:.1e}"
        for c in CLASSES
        if rec[c]["sim"]["travel1d_x0.25"]["mu_eq"] < BAND[0]
    ]
    hi_out = [
        f"{plotstyle.CLASS_LABELS[c]} {rec[c]['sim']['travel1d_x4']['mu_eq']:.1e}"
        for c in CLASSES
        if rec[c]["sim"]["travel1d_x4"]["mu_eq"] > BAND[1]
    ]
    above_rollers = [plotstyle.CLASS_LABELS[c] for c in CLASSES if rec[c]["sim"]["travel1d_x1"]["mu_eq"] > rollers]
    nominal = ", ".join(f"{plotstyle.CLASS_LABELS[c]} {rec[c]['sim']['travel1d_x1']['mu_eq']:.2e}" for c in CLASSES)
    caption = (
        "(a) Closed-form drag at 0.1 m/s and m*, stacked by component and expressed as μ = F/(m*·g₀). The DRAWER's cavity piston "
        "drag (air pushed through its 40 cm² leak) is a term the NARRATIVE ledger table leaves out; with it the DRAWER ledger is "
        f"{rec['drawer']['mu_ledger']:.2e}. Markers: μ_eq of a 0.2 s levitated coast from 0.1 m/s with the LSM inverter latched off, "
        f"voltage coil mode (the plant's drag-energy ledger, aero drag included): {nominal} on Travel1d.\n"
        "(b) The rail sets the drag, not the load, so μ_eq falls as 1/m; thin dashed lines scale c_rail and F_hyst by 0.25 and 4 "
        "(their values are [UNCERTAIN]; the factor is a study choice). Markers: Travel1d coasts at m* for the three factors. "
        "At defaults every class sits inside [1e-3, 1e-2] and none reaches 0"
        + (f"; at ×0.25 rail drag {', '.join(lo_out)} fall below 1e-3" if lo_out else "")
        + (f"; at ×4 {', '.join(hi_out)} exceed 1e-2" if hi_out else "")
        + (
            f". Above the modelled roller μ {rollers:g} at 0.1 m/s: {', '.join(above_rollers)} (levitation removes rolling contact and "
            "stick-slip, not drag)"
            if above_rollers
            else ""
        )
        + ". 'Zero friction' is a direction: no drag has been measured on a real rail."
    )
    title = "drag_ledger: μ_eq is small, never zero"
    stamp = finish(
        fig,
        title=title,
        question=QUESTION,
        caption=caption,
        meta=stamp_meta(cfg, "drag_ledger", "travel1d+carriage6"),
        wspace=0.22,
        left_in=1.0,
    )
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "v_ref_mps": V_REF_MPS,
        "sweep_factors": list(SWEEP),
        "reference_lines": refs,
        "classes": rec,
        "checks": checks,
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="drag_ledger", tier=1, question=QUESTION, run=run)
