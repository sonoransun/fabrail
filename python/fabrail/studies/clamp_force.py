"""clamp_force: how hard a closing carriage presses on a trapped body, in closed form and in simulation.

Claims backed: SAF-005 (clamp force F = √(2Ek): 290.7 N chest and 503.5 N hand at 1.69 J,
173 N through a 10 N/mm edge seal; every unsensed cap ≤ 400 N through the edge seal),
SAF-004 (force–time profile ≤ 400 N for ≤ 0.75 s, ≤ 150 N, < 80 N within 5 s) and SAF-006
(an obstruction stops the carriage and reverses or releases).

Method.

- Closed form (PHYSICS §18): all kinetic energy into the body spring, F = √(2Ek) =
  v·√(m·k), with the closing-edge seal in series. The body stiffnesses are the plant's
  ISO/TS 15066 table, read from the obstacle events of the simulated runs (the Rust
  ``ObstacleBody`` is the single source; ``human.iso15066_bodies`` is used when the
  extension provides it). The edge seal is ``design.damping.k_edge_n_per_m``.
- Simulation (voltage coil mode; Carriage6 for DRAWER, DOOR and WALL, Travel1d for HEAVY):
  the catalog ``obstruction_hand_edge`` (fingers 20 mm from the closed stop) and
  ``obstruction_chest`` moves at the class caps; the contact force is recorded at 2 kHz
  (1 kHz on Travel1d, its plant step) from the first contact. The closed-form bound at the *measured* contact speed shows how
  much of the force comes from the drive pushing on until the obstruction is detected.
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

QUESTION = "How hard can a closing carriage press on a trapped hand or chest, what does the edge seal buy, and what do simulated obstructions reach?"

_BODY_LS = {"hand": "-", "chest": (0, (5, 3))}
_WINDOW_S = (-0.05, 1.5)


def _tier(cls: str) -> str:
    return "travel1d" if cls == "heavy" else "carriage6"


def _series_k(k1: float, k2: float) -> float:
    return k1 * k2 / (k1 + k2)


def run(cfg: Any) -> StudyResult:
    from .. import human, require_native, scenarios

    n = require_native()
    q = quick()
    cases = [(c, "hand") for c in CLASSES] + [(c, "chest") for c in ("door", "wall", "heavy")]
    if q:
        cases = [("drawer", "hand"), ("wall", "chest")]
    jobs = []
    for cls, body in cases:
        c = class_cfg(cfg, cls)
        p = c.params()
        d, s = scenarios.spec(f"obstruction_{'hand_edge' if body == 'hand' else 'chest'}", p, c, fidelity=_tier(cls))
        d["record"] = {"rate_hz": 1000.0 if _tier(cls) == "travel1d" else 2000.0, "channels": ["x_m", "v_mps", "f_obstacle_n", "f_lsm_n"]}
        jobs.append((p, d, s))
    results = run_jobs(jobs, cfg)

    # Body stiffnesses: the native table if exposed, else the obstacle events of the runs.
    k_body: dict[str, float] = {}
    try:
        k_body = {name: b.k_n_per_m for name, b in human.iso15066_bodies().items()}
        k_source = "_native.iso15066_bodies"
    except Exception:  # noqa: BLE001 - NotYetImplementedError until the accessor exists
        k_source = "scenario:obstacle events (fabrail_plant ObstacleBody)"
    for (_cls, body), r in zip(cases, results, strict=True):
        ev = next((e for e in r.events if e.kind == "scenario:obstacle"), None)
        if ev is not None:
            k_body.setdefault(body, float(ev.b))
    p_door = class_cfg(cfg, "door").params()
    k_edge = float(p_door.get("design.damping.k_edge_n_per_m"))
    limits = {
        "dynamic_n": float(max(class_cfg(cfg, c).params().get("safety.f_drive_max_n") for c in CLASSES)),
        "last50_n": float(p_door.get("safety.f_close_last50_n")),
        "static_n": float(p_door.get("safety.f_static_n")),
        "final_n": float(p_door.get("safety.f_static_final_n")),
        "final_s": float(p_door.get("safety.t_static_final_s")),
    }

    fig, axes = new_figure(1, 2, size=(15.0, 6.6), gridspec_kw={"width_ratios": [1.0, 1.25]})
    ax_c, ax_t = axes[0]
    e_grid = np.linspace(0.0, 2.0, 41 if q else 201)
    curves: dict[str, Any] = {}
    body_colors = {"hand": plotstyle.COLORS["ink"], "chest": plotstyle.COLORS["ink2"]}
    for body in ("hand", "chest"):
        if body not in k_body:
            continue
        k = k_body[body]
        for sealed in (False, True):
            k_eff = _series_k(k, k_edge) if sealed else k
            f = [float(n.clamp_force(float(e), k_eff)) for e in e_grid]
            key = f"{body}{'_edge' if sealed else ''}"
            curves[key] = {"k_n_per_m": k_eff, "force_n": f}
            ax_c.plot(
                e_grid,
                f,
                color=body_colors[body],
                ls=_BODY_LS[body],
                lw=2.0 if sealed else 1.1,
                alpha=1.0 if sealed else 0.6,
                label=f"{body} ({k / 1e3:.0f} N/mm)" + (f" + {k_edge / 1e3:.0f} N/mm edge seal" if sealed else ""),
            )
    cls_pts: dict[str, Any] = {}
    for cls in CLASSES:
        p = class_cfg(cfg, cls).params()
        st = class_style(cls)
        m = float(p.get("design.carriage.m_rated_kg"))
        v = max(float(p.get("safety.v_open_max_mps")), float(p.get("safety.v_close_max_mps")))
        e = 0.5 * m * v * v
        f_edge = float(n.clamp_force(e, _series_k(k_body.get("hand", math.nan), k_edge))) if "hand" in k_body else None
        cls_pts[cls] = {"ke_at_cap_j": e, "hand_edge_force_n": f_edge, "v_cap_mps": v, "m_rated_kg": m}
        if f_edge is not None:
            ax_c.plot([e], [f_edge], ls="none", marker=st["marker"], ms=9, color=st["color"], mec=plotstyle.COLORS["surface"], mew=1.3)
    for key, y, lab, col in (
        ("dynamic_n", limits["dynamic_n"], "400 N dynamic (≤ 0.75 s)", plotstyle.COLORS["star"]),
        ("last50_n", limits["last50_n"], "150 N", plotstyle.COLORS["muted"]),
        ("static_n", limits["static_n"], "67 N static, unsensed", plotstyle.COLORS["muted"]),
    ):
        plotstyle.ref_line(ax_c, y, tex_safe(lab), color=col, x=0.3, va="top" if key == "static_n" else "bottom")
    plotstyle.style_ax(ax_c, "kinetic energy at contact (J)", "clamp force F = sqrt(2Ek) (N)", "(a) Energy form of the clamp force")
    ax_c.legend(loc="upper left", **plotstyle.LEGEND_KW)

    sims: dict[str, Any] = {}
    checks: dict[str, Any] = {}
    for (cls, body), r in zip(cases, results, strict=True):
        st = class_style(cls)
        p = class_cfg(cfg, cls).params()
        m_truth = float(p.get("design.carriage.m_star_kg"))
        contact = next((e for e in r.events if e.kind == "plant:obstacle_contact"), None)
        detect = next((e for e in r.events if e.kind == "Obstruction"), None)
        key = f"{cls}_{body}"
        if contact is None:
            sims[key] = {
                "contact": False,
                "faults": faults_of(r),
                "x_end_mm": float(r["x_m"][-1] * 1e3),
                "reason": "no contact: the move stopped before reaching the obstacle",
            }
            continue
        t_c = contact.t_s
        w = (r.t_s >= t_c + _WINDOW_S[0]) & (r.t_s <= t_c + _WINDOW_S[1])
        t_rel = r.t_s[w] - t_c
        f_win = r["f_obstacle_n"][w]
        dec = max(1, int(round(r.rate_hz / 500.0)))  # plot at 500 Hz; the peak is taken at the recording rate
        ax_t.plot(t_rel[::dec], f_win[::dec], color=st["color"], ls=_BODY_LS[body], lw=1.8, label=f"{st['label']}, {body}")
        k_b = k_body[body]
        v_c = float(contact.a)
        f_cf = v_c * math.sqrt(m_truth * _series_k(k_b, k_edge))
        full = r["f_obstacle_n"]
        after = r.t_s >= t_c
        peak = float(np.max(full[after]))
        dt = 1.0 / r.rate_hz
        t_end = r.t_s[-1]
        f_final = float(full[-1])
        sims[key] = {
            "contact": True,
            "t_contact_s": t_c,
            "v_contact_mps": v_c,
            "k_body_n_per_m": k_b,
            "k_series_n_per_m": _series_k(k_b, k_edge),
            "m_truth_kg": m_truth,
            "closed_form_at_contact_n": f_cf,
            "peak_n": peak,
            "peak_over_closed_form": peak / f_cf if f_cf > 0 else None,
            "t_peak_after_contact_s": float(r.t_s[after][int(np.argmax(full[after]))] - t_c),
            "t_detect_after_contact_s": None if detect is None else float(detect.t_s - t_c),
            "time_above_150_s": float(np.sum(full[after] > limits["last50_n"]) * dt),
            "time_above_67_s": float(np.sum(full[after] > limits["static_n"]) * dt),
            "force_at_end_n": f_final,
            "t_end_after_contact_s": float(t_end - t_c),
            "plot_t_s": floats(t_rel[::dec]),
            "plot_force_n": floats(f_win[::dec]),
            "faults": faults_of(r),
        }
        checks[f"{key}.peak_le_400"] = check(peak <= limits["dynamic_n"], peak, limits["dynamic_n"], "FAB-SAF-004 (≤ 400 N), N")
        checks[f"{key}.le_150_after_0p75s"] = check(
            float(np.max(full[r.t_s >= t_c + 0.75], initial=0.0)) <= limits["last50_n"],
            float(np.max(full[r.t_s >= t_c + 0.75], initial=0.0)),
            limits["last50_n"],
            "FAB-SAF-004 (≤ 150 N after the 0.75 s dynamic window), N",
        )
        if t_end - t_c >= limits["final_s"]:
            checks[f"{key}.lt_80_within_5s"] = check(
                f_final < limits["final_n"], f_final, limits["final_n"], "FAB-SAF-004 (< 80 N within 5 s), N"
            )
    for _key, y, lab, col in (
        ("dynamic_n", limits["dynamic_n"], "400 N", plotstyle.COLORS["star"]),
        ("last50_n", limits["last50_n"], "150 N", plotstyle.COLORS["muted"]),
        ("static_n", limits["static_n"], "67 N", plotstyle.COLORS["muted"]),
    ):
        plotstyle.ref_line(ax_t, y, lab, color=col)
    ax_t.axvline(0.75, color=plotstyle.COLORS["axis"], lw=1.0)
    ax_t.annotate(
        "0.75 s",
        xy=(0.75, 0.02),
        xycoords=("data", "axes fraction"),
        xytext=(3, 0),
        textcoords="offset points",
        fontsize=8.5,
        color=plotstyle.COLORS["muted"],
    )
    plotstyle.zero_line(ax_t)
    plotstyle.style_ax(
        ax_t, "time since first contact (s)", "contact force on the body (N)", "(b) Simulated obstructions at the class caps"
    )
    ax_t.legend(loc="center right", **plotstyle.LEGEND_KW)

    hand_edge = curves.get("hand_edge", {})
    at_cap = ", ".join(
        f"{plotstyle.CLASS_LABELS[c]} {cls_pts[c]['hand_edge_force_n']:.0f} N"
        for c in CLASSES
        if cls_pts[c]["hand_edge_force_n"] is not None
    )
    contacts = [(k, v) for k, v in sims.items() if v["contact"]]
    worst = max(contacts, key=lambda kv: kv[1]["peak_over_closed_form"] or 0.0) if contacts else None
    nocontact = [f"{k.replace('_', ' ')} ({', '.join(v['faults']) or 'no fault'})" for k, v in sims.items() if not v["contact"]]
    over150 = [
        f"{k.replace('_', ' ')} {v['peak_n']:.0f} N for {v['time_above_150_s']:.2f} s" for k, v in contacts if v["time_above_150_s"] > 0.0
    ]
    caption = (
        "(a) F = sqrt(2Ek) with the body stiffness in series with the "
        f"{k_edge / 1e3:.0f} N/mm closing-edge seal (bold) and without it (thin); stiffnesses from {k_source}. Markers: each class's "
        f"kinetic energy at its speed cap with m_rated, through hand + seal: {at_cap}"
        + (f" (hand + seal stiffness {hand_edge['k_n_per_m'] / 1e3:.2f} N/mm)" if hand_edge else "")
        + ". This bound assumes the drive stops at contact.\n"
        "(b) Contact force after first contact in the catalog obstruction scenarios (voltage coil mode; Carriage6, HEAVY on "
        "Travel1d), truth mass m*. "
        + (
            f"The largest excess over the energy bound at the measured contact speed is {worst[0].replace('_', ' ')}: "
            f"{worst[1]['peak_n']:.0f} N against {worst[1]['closed_form_at_contact_n']:.0f} N: the drive keeps a force on the body "
            f"(obstruction detected {worst[1]['t_detect_after_contact_s']:.3f} s after contact). "
            if worst and worst[1]["t_detect_after_contact_s"] is not None
            else ""
        )
        + (f"Above 150 N: {'; '.join(over150)}. " if over150 else "No run exceeds 150 N. ")
        + "".join(
            f"{k.replace('_', ' ').upper()}: the contact force peaks {v['t_peak_after_contact_s']:.2f} s after contact, after the "
            f"obstruction was detected ({v['t_detect_after_contact_s']:.3f} s): the reaction does not keep the carriage off the body "
            "(reported as an implementation defect for Wave 2.5). "
            for k, v in contacts
            if v["t_detect_after_contact_s"] is not None and v["t_peak_after_contact_s"] > v["t_detect_after_contact_s"] + 0.2
        )
        + (f"No contact (the move stopped first): {'; '.join(nocontact)}. " if nocontact else "")
        + "Body models are linear springs (ISO/TS 15066 values); forces are checked in simulation against numeric limits from "
        "EN 16005 (2011 draft), not measured on a test rig."
    )
    title = "clamp_force: trapped-body forces"
    stamp = finish(
        fig, title=title, question=QUESTION, caption=caption, meta=stamp_meta(cfg, "clamp_force", "carriage6+travel1d"), wspace=0.2
    )
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "k_body_n_per_m": k_body,
        "k_source": k_source,
        "k_edge_n_per_m": k_edge,
        "limits_n": limits,
        "reference_t_s": 0.75,
        "zero_force_n": 0.0,
        "energy_grid_j": floats(e_grid),
        "curves": curves,
        "class_points": cls_pts,
        "sims": sims,
        "checks": checks,
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="clamp_force", tier=1, question=QUESTION, run=run)
