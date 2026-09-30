"""fail_direction: which way a carriage goes when levitation control is lost, and how hard it lands.

Claim backed: LEV-006 (upper-stop force ratio ≤ 1.30 on DOOR/WALL/HEAVY, stay-landed
ratio ≤ 0.90) and LEV-019 (impact speed on control loss equals ``derive::impact_speed``;
the carriage ends on a stop, never hanging free).

Method.

- Closed form (PHYSICS §3, §15): the PM-only force of the n lift elements at coil current
  0, F(g) = μ₀A_p·M_pm²/(2g + h′)² per element, and the energy balance
  ½mv² = ∫n·F dg − m·g₀·Δg to the upper stop or the rollers. The mirror here uses no
  restated constant: μ₀A_p follows from the derived F* at g*, and it is checked against
  ``_native.impact_speed`` at (m*, g*).
- Simulation (Heave1, voltage coil mode, 20 kHz recording): (1) *bridges off*: every coil
  H-bridge latches off at t_f with a ±1 % m* payload step that picks the direction (the
  zero-power point is an unstable equilibrium, so the sign of the mass error decides);
  (2) *drivers stuck*: the catalog ``control_loss_up/down`` scenarios, whose bridges stick
  at full output — a harder failure than i = 0.

The mirrors :func:`pm_force`, :func:`pm_work`, :func:`impact_speed_at`, :func:`n_lift` and
:func:`stations` are reused by ``zero_power_band``, ``pole_map`` and ``drawer_envelope``.
"""

from __future__ import annotations

import math
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

QUESTION = "If levitation control is lost, which way does each carriage go, how fast does it hit, and does it stay on a stop?"

# ---- magnet closed forms (mirrors of fabrail_params::derive, PHYSICS §2–§3) ----------------


def stations(p: Any) -> list[dict[str, Any]]:
    """The first ``n_magnets`` suspension stations of ``p`` (x, y, z, normal_sign, has_pm)."""
    n = int(p.get("design.suspension.n_magnets"))
    out = []
    for i in range(n):

        def g(key: str, i: int = i) -> Any:
            return p.get(f"design.suspension.stations.{i}.{key}")

        out.append(
            {
                "x": float(g("x_m")),
                "y": float(g("y_m")),
                "z": float(g("z_m")),
                "normal_sign": float(g("normal_sign")),
                "has_pm": bool(g("has_pm")),
            }
        )
    return out


def n_lift(p: Any) -> int:
    """Lift elements carrying the weight: stations with a PM and an upward normal (derive::n_lift)."""
    count = sum(1 for s in stations(p) if s["normal_sign"] > 0 and s["has_pm"])
    return count if count > 0 else max(int(p.get("design.suspension.n_magnets")), 1)


def magnet(p: Any) -> dict[str, float]:
    """Element constants: g*, h′, M_pm, N, A_p and μ₀·A_p (from the derived F* at g*)."""
    d = p.derived()
    g_star = float(p.get("design.suspension.g_star_m"))
    hp = float(d["h_prime_m"])
    m_pm = float(d["m_pm_at"])
    d_star = 2.0 * g_star + hp
    mu0_a = float(d["f_star_n"]) * d_star * d_star / (m_pm * m_pm)
    return {
        "g_star_m": g_star,
        "h_prime_m": hp,
        "m_pm_at": m_pm,
        "turns": float(p.get("design.suspension.turns")),
        "a_p_m2": float(d["pole_area_m2"]),
        "mu0_a": mu0_a,
    }


def pm_force(p: Any, gap_m: float | np.ndarray, i_a: float = 0.0) -> Any:
    """Vertical force of one element at y = 0: μ₀A_p·(M_pm + N·i)²/(2g + h′)², N (derive::pm_force)."""
    k = magnet(p)
    mmf = k["m_pm_at"] + k["turns"] * i_a
    return k["mu0_a"] * mmf * mmf / (2.0 * np.asarray(gap_m, dtype=float) + k["h_prime_m"]) ** 2


def pm_work(p: Any, g_a: float, g_b: float) -> float:
    """PM-only work of the n lift elements from g_a to g_b ≥ g_a, J (derive::pm_work)."""
    k = magnet(p)
    hp = k["h_prime_m"]
    return 0.5 * n_lift(p) * k["mu0_a"] * k["m_pm_at"] ** 2 * (1.0 / (2.0 * g_a + hp) - 1.0 / (2.0 * g_b + hp))


def impact_speed_at(p: Any, direction: str, m_kg: float, g_start_m: float) -> float:
    """Impact speed of a control loss at i = 0 from rest at g_start (derive::impact_speed_at), m/s."""
    g_up = float(p.get("design.suspension.g_up_m"))
    g_land = float(p.get("design.suspension.g_land_m"))
    w = m_kg * G0_MPS2
    if direction == "up":
        e = pm_work(p, g_up, g_start_m) - w * (g_start_m - g_up)
    else:
        e = w * (g_land - g_start_m) - pm_work(p, g_start_m, g_land)
    return math.sqrt(2.0 * e / m_kg) if e > 0.0 and m_kg > 0.0 else 0.0


def force_ratio(p: Any, gap_m: Any, m_kg: float) -> Any:
    """PM-only lift over weight n·F(g, 0)/(m·g₀) (derive::stop_ratio): > 1 pulls up, < 1 lets it fall."""
    return n_lift(p) * pm_force(p, gap_m) / (m_kg * G0_MPS2)


# ---- simulations ---------------------------------------------------------------------------

_T_FAULT_S = 0.3
_REC_HZ = 20000.0
_CHANNELS = ["gap_0_m", "coil_0_a"]


def _bridges_off_job(cfg: Any, cls: str, sign: int, dur_s: float) -> tuple[Any, dict[str, Any], dict[str, bytes]]:
    from .. import scenarios

    c = class_cfg(cfg, cls, duration_s=dur_s, record_rate_hz=_REC_HZ)
    p = c.params()
    d, s = scenarios.spec("hover", p, c, fidelity="heave1")
    m_star = float(p.get("design.carriage.m_star_kg"))
    n_mag = int(p.get("design.suspension.n_magnets"))
    d["name"] = f"{cls}_bridges_off_{'down' if sign > 0 else 'up'}"
    d["record"] = {"rate_hz": _REC_HZ, "channels": list(_CHANNELS)}
    d["events"] = [{"kind": "payload", "t_s": _T_FAULT_S, "delta_kg": sign * 0.01 * m_star}] + [
        {"kind": "fault", "t_s": _T_FAULT_S, "what": "coil_driver", "ch": i} for i in range(n_mag)
    ]
    d["metrics"] = {"windows": {}}
    return p, d, s


def _stuck_job(cfg: Any, cls: str, direction: str) -> tuple[Any, dict[str, Any], dict[str, bytes]]:
    from .. import scenarios

    c = class_cfg(cfg, cls, record_rate_hz=_REC_HZ, duration_s=1.2)
    p = c.params()
    d, s = scenarios.spec(f"control_loss_{direction}", p, c, fidelity="heave1")
    d["record"] = {"rate_hz": _REC_HZ, "channels": list(_CHANNELS)}
    return p, d, s


def _t_control_loss(d: dict[str, Any]) -> float:
    for e in d["events"]:
        if e["kind"] in ("control_loss", "fault"):
            return float(e["t_s"])
    return _T_FAULT_S


def _impact(r: Any, t_fault: float) -> dict[str, Any]:
    """Direction, impact speed (from the gap rate just before the first stop contact) and bounces."""
    first_up = next((e for e in r.events if e.kind == "plant:upper_stop" and e.t_s >= t_fault), None)
    first_down = next((e for e in r.events if e.kind == "plant:landed" and e.t_s >= t_fault), None)
    if first_up is None and first_down is None:
        return {"direction": "none", "v_impact_mps": 0.0, "t_impact_s": None, "contacts": 0, "rests_on_stop": False}
    up = first_down is None or (first_up is not None and first_up.t_s < first_down.t_s)
    hit = first_up if up else first_down
    assert hit is not None
    t_hit = hit.t_s
    gap = r["gap_0_m"]
    rate = np.gradient(gap, r.t_s)
    win = (r.t_s >= t_fault) & (r.t_s <= t_hit + 0.5 / _REC_HZ)
    v = float(np.max(np.abs(rate[win]))) if np.any(win) else 0.0
    kind = "plant:upper_stop" if up else "plant:landed"
    contacts = sum(1 for e in r.events if e.kind == kind and e.t_s >= t_fault)
    stop_gap = gap[-1]
    return {
        "direction": "up" if up else "down",
        "v_impact_mps": v,
        "v_event_mps": float(first_down.a) if (not up and first_down is not None) else None,
        "t_impact_s": float(t_hit),
        "contacts": int(contacts),
        "final_gap_mm": float(stop_gap * 1e3),
    }


def run(cfg: Any) -> StudyResult:
    from .. import require_native

    n = require_native()
    q = quick()
    dur = 0.6 if q else 0.9
    classes = CLASSES
    jobs = []
    keys = []
    for cls in classes:
        for sign in (-1, +1):
            jobs.append(_bridges_off_job(cfg, cls, sign, dur))
            keys.append((cls, "bridges_off", "up" if sign < 0 else "down"))
        for direction in ("up", "down"):
            jobs.append(_stuck_job(cfg, cls, direction))
            keys.append((cls, "drivers_stuck", direction))
    results = run_jobs(jobs, cfg)

    rec_classes: dict[str, Any] = {}
    checks: dict[str, Any] = {}
    fig, axes = new_figure(1, 3, size=(14.0, 6.2))
    ax_r, ax_up, ax_dn = axes[0]

    for cls in classes:
        p = class_cfg(cfg, cls).params()
        m_star = float(p.get("design.carriage.m_star_kg"))
        m_min = float(p.get("design.carriage.m_op_min_kg"))
        g_star = float(p.get("design.suspension.g_star_m"))
        g_up = float(p.get("design.suspension.g_up_m"))
        g_land = float(p.get("design.suspension.g_land_m"))
        gaps = np.linspace(g_up, g_land, 25 if q else 121)
        ratio = force_ratio(p, gaps, m_star)
        ratio_up = float(force_ratio(p, g_up, m_star))
        ratio_land = float(force_ratio(p, g_land, m_min))
        up_native = float(n.impact_speed(p, "up"))
        dn_native = float(n.impact_speed(p, "down"))
        up_mirror = impact_speed_at(p, "up", m_star, g_star)
        dn_mirror = impact_speed_at(p, "down", m_star, g_star)
        mirror_err = max(abs(up_mirror - up_native) / up_native, abs(dn_mirror - dn_native) / dn_native)
        runs: dict[str, Any] = {}
        for (kc, kind, direction), r, (_, d, _) in zip(keys, results, jobs, strict=True):
            if kc != cls:
                continue
            t_f = _t_control_loss(d)
            imp = _impact(r, t_f)
            m_truth = m_star * (1.0 + (0.01 if direction == "down" else -0.01)) if kind == "bridges_off" else m_star
            cf = impact_speed_at(p, direction, m_truth, g_star)
            imp.update(
                {
                    "m_truth_kg": m_truth,
                    "closed_form_mps": cf,
                    "faults": faults_of(r, t_f),
                    "ratio_to_closed_form": imp["v_impact_mps"] / cf if cf > 0 else None,
                }
            )
            runs[f"{kind}_{direction}"] = imp
        rule_up = None if cls == "drawer" else 1.30  # FAB-LEV-006 upper-stop rule
        st = class_style(cls)
        x = gaps / g_star
        ax_r.plot(x, ratio, color=st["color"], lw=1.8, label=st["label"])
        ax_r.plot(
            [g_up / g_star], [ratio_up], marker=st["marker"], color=st["color"], ls="none", ms=8, mec=plotstyle.COLORS["surface"], mew=1.5
        )
        ax_r.plot([g_land / g_star], [ratio_land], marker=st["marker"], color=st["color"], ls="none", ms=8, mfc="none", mew=1.8)
        rec_classes[cls] = {
            "g_star_mm": g_star * 1e3,
            "g_up_mm": g_up * 1e3,
            "g_land_mm": g_land * 1e3,
            "m_star_kg": m_star,
            "m_op_min_kg": m_min,
            "gap_over_gstar": floats(x),
            "force_ratio_at_m_star": floats(ratio),
            "stop_ratio_up_at_m_star": ratio_up,
            "stay_landed_ratio_at_m_op_min": ratio_land,
            "gap_over_gstar_up": g_up / g_star,
            "gap_over_gstar_land": g_land / g_star,
            "impact_closed_form_mps": {"up": up_native, "down": dn_native},
            "impact_mirror_rel_err": mirror_err,
            "runs": runs,
        }
        checks[f"{cls}.stay_landed"] = check(ratio_land <= 0.90, ratio_land, 0.90, "FAB-LEV-006 (F18)")
        if rule_up is not None:
            checks[f"{cls}.upper_stop_ratio"] = check(ratio_up <= rule_up, ratio_up, rule_up, "FAB-LEV-006")
        else:
            rating = float(p.get("design.landing.upper_roller_rating_n"))
            f_up = n_lift(p) * float(pm_force(p, g_up))
            checks[f"{cls}.upper_roller_rating"] = check(
                rating >= 550.0 and f_up <= rating * n_lift(p),
                rating,
                550.0,
                "FAB-LEV-006 (DRAWER elastomer upper rollers, F2)",
                upper_stop_force_n=f_up,
            )
        bo = runs["bridges_off_up"], runs["bridges_off_down"]
        checks[f"{cls}.bridges_off_direction"] = check(
            bo[0]["direction"] == "up" and bo[1]["direction"] == "down", None, None, "lighter than m(g*) rises, heavier falls (PHYSICS §3)"
        )
        checks[f"{cls}.impact_within_5pct"] = check(
            all(r["ratio_to_closed_form"] is not None and abs(r["ratio_to_closed_form"] - 1.0) <= 0.05 for r in bo),
            max(abs((r["ratio_to_closed_form"] or 0.0) - 1.0) for r in bo),
            0.05,
            "FAB-LEV-019 (±5 % of derive::impact_speed; bridges-off Heave1 run vs the i = 0 closed form)",
        )

    # Reference lines of panel (a): equilibrium 1, upper-stop rule 1.30, stay-landed rule 0.90.
    refs = {"equilibrium": 1.0, "upper_stop_rule": 1.30, "stay_landed_rule": 0.90}
    plotstyle.ref_line(ax_r, refs["upper_stop_rule"], "upper-stop rule 1.30 (DOOR/WALL/HEAVY)")
    plotstyle.ref_line(ax_r, refs["equilibrium"], "n·F = m·g: zero-power point", color=plotstyle.COLORS["ink2"])
    plotstyle.ref_line(ax_r, refs["stay_landed_rule"], "stay-landed rule 0.90 (m_op,min)", va="top", x=0.8)
    ax_r.set_ylim(0.0, 2.55)
    plotstyle.style_ax(ax_r, "gap / g*", "PM lift / weight  n·F(g, 0)/(m·g₀)", "(a) Fail direction at i = 0")
    ax_r.legend(loc="upper center", ncols=2, **plotstyle.LEGEND_KW)

    width = 0.36
    xs = np.arange(len(classes), dtype=float)
    bars: dict[str, Any] = {}
    for ax, direction, label in ((ax_up, "up", "(b) Up: snap to the upper stop"), (ax_dn, "down", "(c) Down: drop to the rollers/seat")):
        off = [rec_classes[c]["runs"][f"bridges_off_{direction}"]["v_impact_mps"] * 1e3 for c in classes]
        stuck = [rec_classes[c]["runs"][f"drivers_stuck_{direction}"]["v_impact_mps"] * 1e3 for c in classes]
        cf_bars = [rec_classes[c]["runs"][f"bridges_off_{direction}"]["closed_form_mps"] * 1e3 for c in classes]
        colors = [plotstyle.CLASS_COLORS[c] for c in classes]
        x_off, x_stuck = xs - width / 2 - 0.01, xs + width / 2 + 0.01
        ax.bar(x_off, off, width, color=colors, label="sim: bridges off (freewheel)")
        ax.bar(
            x_stuck,
            stuck,
            width,
            color=colors,
            alpha=0.45,
            hatch="///",
            edgecolor=plotstyle.COLORS["surface"],
            label="sim: drivers stuck at full output",
        )
        ax.plot(
            x_off,
            cf_bars,
            ls="none",
            marker="_",
            ms=26,
            mew=2.6,
            color=plotstyle.COLORS["star"],
            label="closed form, i = 0 (derive::impact_speed_at)",
        )
        ax.set_xticks(xs, [plotstyle.CLASS_LABELS[c] for c in classes])
        plotstyle.style_ax(ax, None, "impact speed (mm/s)", label)
        ax.set_ylim(0.0, 1.45 * max(max(off), max(stuck), max(cf_bars)))
        ax.legend(loc="upper left", markerscale=0.55, **plotstyle.LEGEND_KW)
        bars[direction] = {
            "bridges_off_mm_s": off,
            "drivers_stuck_mm_s": stuck,
            "closed_form_mm_s": cf_bars,
            "bar_x": floats(x_off),
            "bar_x_stuck": floats(x_stuck),
        }

    ratios = [rec_classes[c]["runs"][f"bridges_off_{d}"]["ratio_to_closed_form"] or 0.0 for c in classes for d in ("up", "down")]
    stuck_ratio = [rec_classes[c]["runs"][f"drivers_stuck_{d}"]["ratio_to_closed_form"] or 0.0 for c in classes for d in ("up", "down")]
    mirror = max(rec_classes[c]["impact_mirror_rel_err"] for c in classes)
    caption = (
        "(a) PM-only lift over weight at coil current 0 for m* (lines), at the upper stop with m* (filled markers) and at the landing "
        "gap with m_op,min (open markers). Above 1 the carriage snaps up, below 1 it falls; WALL and HEAVY share g*, h′ and the "
        "design flux, so their curves coincide. The DRAWER's 2.16 at its 1.2 mm stop is carried by elastomer upper rollers "
        "(FAB-LEV-006), not by the 1.30 rule.\n"
        f"(b, c) Closed form (red ticks): energy balance at i = 0, no coil lag and no dissipation; the Python mirror agrees with "
        f"_native.impact_speed at (m*, g*) (relative difference {mirror:.1e}). Simulated on Heave1, voltage coil mode, 20 kHz: 'bridges off' latches "
        f"every coil bridge off at {_T_FAULT_S} s with a ±1 % m* payload step choosing the direction; it lands at "
        f"{min(ratios):.2f}–{max(ratios):.2f}× the closed form: a latched-off bridge freewheels, so the moving magnet induces a coil current "
        f"(motional EMF, τ = L/R) that brakes the fall, and the eddy lag adds to it; the closed form assumes an open coil. "
        f"'Drivers stuck' (catalog control_loss_up/down, bridges at full output) is a harder failure: {min(stuck_ratio):.2f}–"
        f"{max(stuck_ratio):.2f}× the i = 0 closed form. Rigid stops use a Hunt–Crossley contact model; no hardware impact test "
        "is claimed."
    )
    title = "fail_direction: control loss sends each carriage to a stop"
    stamp = finish(fig, title=title, question=QUESTION, caption=caption, meta=stamp_meta(cfg, "fail_direction", "heave1"), wspace=0.3)
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "t_fault_s": _T_FAULT_S,
        "record_rate_hz": _REC_HZ,
        "reference_lines": refs,
        "classes": rec_classes,
        "bars": bars,
        "checks": checks,
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="fail_direction", tier=1, question=QUESTION, run=run)
