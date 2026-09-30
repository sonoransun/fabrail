"""drawer_envelope: for which DRAWER payloads contactless extension, unseat, re-lift and slot exit work.

Claims backed: LEV-016 (contactless travel ≤ 50 % extension; the study reports the maximum
contactless extension versus mass and sets the saturation flag, asserting no specific
feasible point), LEV-017 (unseat within i_pulse for 8–48 kg; unseat plus slot exit at
48 kg under the I²t clamp) and LEV-019 (after a fail-down onto the rollers the drawer is
re-liftable only up to about 20 kg).

Method.

- (a) Extension envelope: a Python mirror of ``derive::extension_capacity`` (the frozen
  ``_native`` surface has no binding for it; the Rust closed form is verified by
  ``rs:params/derive::extension_capacity_closed_form``). The weight at the CoM is shared
  statically by the front and rear lift groups; an element engaged over a fraction e of
  its pole length supplies e·F_max, with F_max the band-edge force at i_cont capped at
  the saturation force B_sat²·A_p/μ₀.
- (b) Lift currents: native ``bias_current`` from the 3.5 mm seat (unseat) and from the
  6.5 mm rollers (re-lift after a fail-down), against i_pulse and V_bus/R.
- (c, d) Closed loop, Carriage6, voltage coil mode: the catalog ``soft_open`` (unseat,
  slot exit, open to the preset travel on the log-damped terminus) with the payload
  swept. Success = arrival at the travel end without a raised fault that lands the
  drawer; the peak per-coil I²t accumulator (floored at 0, the core clamp's rule) is compared with the budget
  (i_pulse² − i_cont²)·t_pulse, and the steady holding current at full extension with
  i_cont.
"""

from __future__ import annotations

import math
from typing import Any, cast

import numpy as np

from .. import plotstyle
from ..ground import G0_MPS2
from .fail_direction import magnet, pm_force, stations
from .runner import (
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

QUESTION = "For which DRAWER payloads are contactless extension, unseat, re-lift from the rollers and slot exit feasible?"

_LANDING_FAULTS = ("SaturationPersist", "BusUnder", "GapEnvelope", "LiftTimeout", "SeatFailed", "EnergyLimit", "Overspeed")


def element_force_max(p: Any) -> float:
    """Maximum continuous lift of one fully engaged element: F(band min gap, i_cont), capped at B_sat²·A_p/μ₀ (derive::element_force_max), N."""
    k = magnet(p)
    lo = float(p.get("design.suspension.zp_band_m.0"))
    g = lo if lo > 0.0 else k["g_star_m"]
    f = float(pm_force(p, g, float(p.get("design.suspension.i_cont_a"))))
    b_sat = float(p.get("design.suspension.b_sat_t"))
    mu0 = k["mu0_a"] / k["a_p_m2"]
    return min(f, b_sat * b_sat * k["a_p_m2"] / mu0) if b_sat > 0.0 else f


def extension_capacity(p: Any, m_kg: float) -> tuple[float, bool]:
    """(maximum contactless extension / body length, saturated flag) at mass m (derive::extension_capacity)."""
    length = float(p.get("design.carriage.length_m"))
    pole_l = float(p.get("design.suspension.pole_l_m"))
    if length <= 0.0 or pole_l <= 0.0 or m_kg <= 0.0:
        return 0.0, True
    lift = [s for s in stations(p) if s["normal_sign"] > 0]
    if not lift:
        return 0.0, True
    x_front = max(s["x"] for s in lift)
    x_rear = min(s["x"] for s in lift)
    n_front = sum(1 for s in lift if s["x"] == x_front)
    n_rear = sum(1 for s in lift if s["x"] == x_rear)
    w = m_kg * G0_MPS2
    x_c = float(p.get("design.carriage.cg_m.0"))
    if x_front - x_rear > 0.0:
        span = x_front - x_rear
        f_front, f_rear = w * (x_c - x_rear) / (span * n_front), w * (x_front - x_c) / (span * n_rear)
    else:
        f_front = f_rear = w / n_front
    f_max = element_force_max(p)
    if f_front < 0.0 or f_rear < 0.0 or f_front > f_max or f_rear > f_max or f_max <= 0.0:
        return 0.0, True

    def reach(x_k: float, f_k: float) -> float:
        return length / 2.0 - x_k + pole_l / 2.0 - (f_k / f_max) * pole_l

    s_max = min(max(min(reach(x_front, f_front), reach(x_rear, f_rear)), 0.0), length)
    return s_max / length, float(p.get("design.rail.travel_m")) > s_max


def _i2t_peak(i_a: np.ndarray, i_cont_a: float, dt_s: float) -> float:
    """Peak of the I²t accumulator E⁺ = max(0, E + (i² − i_cont²)·dt), the core clamp's rule (A²s)."""
    e = peak = 0.0
    for v in (i_a * i_a - i_cont_a * i_cont_a) * dt_s:
        e = max(0.0, e + float(v))
        peak = max(peak, e)
    return peak


def _mass_at_current(n: Any, p: Any, gap_m: float, i_a: float, m_hi: float = 500.0) -> float:
    """The mass whose bias current at ``gap_m`` equals ``i_a`` (bisection; bias current rises with mass), kg."""
    lo, hi = 1e-3, m_hi
    if float(n.bias_current(p, gap_m, hi)) < i_a:
        return math.inf
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if float(n.bias_current(p, gap_m, mid)) < i_a:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def run(cfg: Any) -> StudyResult:
    from .. import require_native, scenarios

    n = require_native()
    q = quick()
    c0 = class_cfg(cfg, "drawer")
    p = c0.params()
    st = class_style("drawer")
    m_star = float(p.get("design.carriage.m_star_kg"))
    m_op = (float(p.get("design.carriage.m_op_min_kg")), float(p.get("design.carriage.m_rated_kg")))
    travel = float(p.get("design.rail.travel_m"))
    length = float(p.get("design.carriage.length_m"))
    i_cont = float(p.get("design.suspension.i_cont_a"))
    i_pulse = float(p.get("design.suspension.i_pulse_a"))
    t_pulse = float(p.get("design.suspension.t_pulse_s"))
    v_over_r = float(p.get("design.power.v_bus_v")) / float(p.get("design.suspension.r_coil_ohm"))
    g_seat = float(p.get("design.suspension.g_seat_m"))
    g_land = float(p.get("design.suspension.g_land_m"))

    # (a) extension envelope.
    masses = np.linspace(1.0, 2.0 * m_op[1], 40 if q else 200)
    env = [extension_capacity(p, float(m)) for m in masses]
    frac = [e[0] for e in env]
    sat = [bool(e[1]) for e in env]
    first_sat = next((float(m) for m, s in zip(masses, sat, strict=True) if s), None)
    rated_frac, rated_sat = extension_capacity(p, m_op[1])
    # (b) lift currents.
    i_seat = [float(n.bias_current(p, g_seat, float(m))) for m in masses]
    i_land = [float(n.bias_current(p, g_land, float(m))) for m in masses]
    m_relift_pulse = _mass_at_current(n, p, g_land, i_pulse)
    m_relift_bus = _mass_at_current(n, p, g_land, v_over_r)
    m_unseat_pulse = _mass_at_current(n, p, g_seat, i_pulse)
    # (c, d) closed-loop slot exit and open.
    sim_m = [m_star, 42.0, m_op[1]] if q else sorted({8.0, 16.0, m_star, 32.0, 36.0, 40.0, 42.0, 44.0, 46.0, m_op[1]})
    jobs = []
    rec_hz = 2000.0
    for m in sim_m:
        spec_d, series = scenarios.spec("soft_open", p, c0, fidelity="carriage6")
        spec_d["name"] = f"drawer_slot_exit_{m:.1f}kg"
        cast("dict[str, Any]", spec_d["initial"])["payload_kg"] = float(m - m_star)
        spec_d["record"] = {
            "rate_hz": rec_hz,
            "channels": ["x_m", "coil_0_a", "coil_1_a", "coil_2_a", "coil_3_a", "gap_0_m", "gap_1_m", "gap_2_m", "gap_3_m"],
        }
        jobs.append((p, spec_d, series))
    results = run_jobs(jobs, cfg)
    budget = (i_pulse**2 - i_cont**2) * t_pulse
    sims: dict[str, list[Any]] = {
        "m_kg": [],
        "arrived": [],
        "x_end_mm": [],
        "i_peak_a": [],
        "i2t_a2s": [],
        "i2t_frac": [],
        "i_hold_open_a": [],
        "gap_open_mm": [],
        "gap_spread_open_um": [],
        "gap_expected_mm": [],
        "faults": [],
    }
    for m, r in zip(sim_m, results, strict=True):
        coils = np.vstack([r[f"coil_{i}_a"] for i in range(4)])
        dt = 1.0 / r.rate_hz
        i2t = max(_i2t_peak(row, i_cont, dt) for row in coils)
        faults = faults_of(r)
        arrived = (
            any(e.kind == "Arrived" for e in r.events)
            and r["x_m"][-1] >= travel - 1e-3
            and not any(any(k in f for k in _LANDING_FAULTS) for f in faults)
        )
        last = r.t_s >= r.t_s[-1] - 0.5
        sims["m_kg"].append(float(m))
        sims["arrived"].append(bool(arrived))
        sims["x_end_mm"].append(float(r["x_m"][-1] * 1e3))
        sims["i_peak_a"].append(float(np.max(np.abs(coils))))
        sims["i2t_a2s"].append(i2t)
        sims["i2t_frac"].append(i2t / budget)
        sims["i_hold_open_a"].append(float(np.max(np.abs(np.mean(coils[:, last], axis=1)))) if arrived else None)
        gaps = np.array([float(np.mean(r[f"gap_{i}_m"][last])) for i in range(4)])
        sims["gap_open_mm"].append(float(np.mean(gaps) * 1e3) if arrived else None)
        sims["gap_spread_open_um"].append(float((gaps.max() - gaps.min()) * 1e6) if arrived else None)
        lo_b, hi_b = float(p.get("design.suspension.zp_band_m.0")), float(p.get("design.suspension.zp_band_m.1"))
        g_zp = float(n.zero_power_gap(float(m), float(p.get("design.suspension.g_star_m")), float(p.derived()["h_prime_m"]), m_star))
        sims["gap_expected_mm"].append(min(max(g_zp, lo_b), hi_b) * 1e3)
        sims["faults"].append(faults)
    ok_masses = [m for m, a in zip(sims["m_kg"], sims["arrived"], strict=True) if a]
    fail_masses = [m for m, a in zip(sims["m_kg"], sims["arrived"], strict=True) if not a]
    limit_ok = max((m for m in ok_masses if all(mm > m for mm in fail_masses)), default=None)
    first_fail = min(fail_masses) if fail_masses else None

    fig, axes = new_figure(2, 2, size=(14.0, 10.2))
    (ax_e, ax_l), (ax_s, ax_h) = axes
    # (a)
    ax_e.plot(masses, frac, color=st["color"], lw=2.0, label="max contactless extension / body length")
    refs = {"preset_travel_frac": travel / length, "fifty_pct_rule": 0.5}
    plotstyle.ref_line(
        ax_e,
        refs["preset_travel_frac"],
        va="top",
        x=0.62,
        label=f"preset travel {travel * 1e3:.0f} mm = {refs['preset_travel_frac']:.2f} of the body (LEV-016)",
    )
    if first_sat is not None:
        ax_e.axvspan(first_sat, float(masses[-1]), color=plotstyle.COLORS["ctrl"], alpha=0.12, lw=0)
        ax_e.annotate(
            "saturation flag set",
            xy=(first_sat, 0.03),
            xytext=(4, 0),
            textcoords="offset points",
            fontsize=8.5,
            color=plotstyle.COLORS["ink2"],
        )
    ax_e.axvspan(*m_op, color=plotstyle.COLORS["band"], alpha=0.07, lw=0)
    plotstyle.style_ax(ax_e, "payload-inclusive mass (kg)", "extension / body length", "(a) Contactless extension envelope")
    ax_e.set_ylim(0.0, max(frac) * 1.25 if max(frac) > 0 else 1.0)
    # (b)
    ax_l.plot(masses, i_seat, color=st["color"], lw=2.0, label=f"unseat from the {g_seat * 1e3:.1f} mm seat")
    ax_l.plot(masses, i_land, color=st["color"], lw=2.0, ls=(0, (5, 3)), label=f"re-lift from the {g_land * 1e3:.1f} mm rollers")
    plotstyle.ref_line(ax_l, i_pulse, f"i_pulse {i_pulse:.0f} A ({t_pulse * 1e3:.0f} ms)", color=plotstyle.COLORS["star"])
    plotstyle.ref_line(ax_l, v_over_r, f"V_bus/R {v_over_r:.0f} A", color=plotstyle.COLORS["muted"])
    plotstyle.ref_line(ax_l, i_cont, f"i_cont {i_cont:.0f} A", color=plotstyle.COLORS["muted"], va="top")
    marks = {"relift_limit_kg": m_relift_pulse, "relift_bus_limit_kg": m_relift_bus, "unseat_limit_kg": m_unseat_pulse}
    ax_l.plot([m_relift_pulse, m_unseat_pulse], [i_pulse, i_pulse], ls="none", marker="o", ms=8, color=plotstyle.COLORS["star"])
    ax_l.annotate(
        f"re-lift limit {m_relift_pulse:.1f} kg",
        xy=(m_relift_pulse, i_pulse),
        xytext=(6, 8),
        textcoords="offset points",
        fontsize=8.5,
        color=plotstyle.COLORS["ink2"],
    )
    ax_l.annotate(
        f"unseat limit {m_unseat_pulse:.1f} kg",
        xy=(m_unseat_pulse, i_pulse),
        xytext=(6, -14),
        textcoords="offset points",
        fontsize=8.5,
        color=plotstyle.COLORS["ink2"],
    )
    ax_l.axvspan(*m_op, color=plotstyle.COLORS["band"], alpha=0.07, lw=0)
    ax_l.set_ylim(min(i_seat) - 2.0, 2.2 * i_pulse)
    plotstyle.style_ax(ax_l, "payload-inclusive mass (kg)", "bias current per coil (A)", "(b) Lift current from rest")
    ax_l.legend(loc="upper left", **plotstyle.LEGEND_KW)
    # (c)
    colors = [st["color"] if a else plotstyle.COLORS["ctrl"] for a in sims["arrived"]]
    xpos = np.arange(len(sim_m), dtype=float)
    ax_s.bar(xpos, sims["i2t_frac"], 0.7, color=colors)
    plotstyle.ref_line(ax_s, 1.0, "I²t budget (i_pulse² − i_cont²)·t_pulse", color=plotstyle.COLORS["star"], x=0.45)
    ax_s.set_ylim(0.0, 1.25)
    ax_s.set_xticks(xpos, [f"{m:.0f}" for m in sim_m])
    for x, a, h in zip(xpos, sims["arrived"], sims["i2t_frac"], strict=True):
        ax_s.annotate(
            "open" if a else "failed",
            xy=(x, h),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=8,
            color=plotstyle.COLORS["ink2"],
        )
    plotstyle.style_ax(ax_s, "payload-inclusive mass (kg)", "peak-coil I²t spend / budget", "(c) Carriage6: unseat, slot exit and open")
    # (d)
    hold_m = [m for m, g in zip(sim_m, sims["gap_open_mm"], strict=True) if g is not None]
    hold_g = [g for g in sims["gap_open_mm"] if g is not None]
    m_curve = np.linspace(min(sim_m), max(sim_m), 60)
    lo_b, hi_b = float(p.get("design.suspension.zp_band_m.0")), float(p.get("design.suspension.zp_band_m.1"))
    g_curve = [
        min(
            max(
                float(n.zero_power_gap(float(m), float(p.get("design.suspension.g_star_m")), float(p.derived()["h_prime_m"]), m_star)), lo_b
            ),
            hi_b,
        )
        * 1e3
        for m in m_curve
    ]
    ax_h.plot(m_curve, g_curve, color=st["color"], lw=1.6, label="zero-power gap, band-clamped (closed form)")
    ax_h.plot(
        hold_m,
        hold_g,
        ls="none",
        marker=st["marker"],
        ms=8,
        color=st["color"],
        mec=plotstyle.COLORS["surface"],
        mew=1.2,
        label="Carriage6 at full extension (mean of 4 elements)",
    )
    plotstyle.ref_line(ax_h, lo_b * 1e3, f"band edge {lo_b * 1e3:.1f} mm", color=plotstyle.COLORS["muted"], va="top")
    plotstyle.style_ax(ax_h, "payload-inclusive mass (kg)", "float gap at the open position (mm)", "(d) Float gap at full extension")
    ax_h.legend(loc="upper right", **plotstyle.LEGEND_KW)

    checks = {
        "lev016_travel_le_50pct": check(
            travel / length <= 0.5 + 1e-12, travel / length, 0.5, "FAB-LEV-016 (preset travel ≤ 50 % extension)"
        ),
        "lev016_rated_not_saturated": check(
            not rated_sat, rated_frac, travel / length, "FAB-LEV-016 (saturation flag at m_rated; reported, not asserted)"
        ),
        "lev017_unseat_within_i_pulse": check(
            max(abs(float(n.bias_current(p, g_seat, m))) for m in m_op) <= i_pulse,
            float(n.bias_current(p, g_seat, m_op[1])),
            i_pulse,
            "FAB-LEV-017 (unseat 8–48 kg within i_pulse), A",
        ),
        "lev019_relift_limit": check(
            m_relift_pulse >= 20.0, m_relift_pulse, 20.0, "FAB-LEV-019 (re-lift from the rollers up to 20 kg within i_pulse), kg"
        ),
        "lev017_slot_exit_rated": check(
            m_op[1] in ok_masses, m_op[1], limit_ok, "FAB-LEV-017 (unseat plus slot exit at 48 kg; closed loop, this build)"
        ),
    }
    fail_txt = (
        f"fails from {first_fail:.0f} kg ({'; '.join(', '.join(f) for m, f in zip(sims['m_kg'], sims['faults'], strict=True) if m == first_fail)})"
        if first_fail is not None
        else "succeeds at every simulated mass"
    )
    caption = (
        "(a) Maximum contactless extension versus mass from the Python mirror of derive::extension_capacity (no binding exists "
        "in the frozen _native surface); "
        + (
            f"the saturation flag is set from {first_sat:.1f} kg"
            if first_sat is not None
            else "the saturation flag is never set in the swept range"
        )
        + f" and the rated {m_op[1]:.0f} kg reaches {rated_frac:.3f} of the body length (preset travel {travel / length:.2f}). "
        "The model shares the weight statically between the front and rear lift groups and does not include the drawer's "
        "pitch compliance; beyond 50 % extension the drawer runs on rollers (roadmap). Shaded blue: the 8–48 kg range.\n"
        f"(b) Closed-form bias current (native bias_current). Unseating from the seat stays within i_pulse up to "
        f"{m_unseat_pulse:.1f} kg; re-lift after a fail-down onto the {g_land * 1e3:.1f} mm rollers needs i_pulse at "
        f"{m_relift_pulse:.1f} kg and V_bus/R at {m_relift_bus:.1f} kg, so heavier drawers need a manual push to re-seat (LEV-019).\n"
        f"(c, d) Carriage6, voltage coil mode, catalog soft_open from the seat: in this build unseat, slot exit and open "
        + (f"succeed up to {limit_ok:.0f} kg and " if limit_ok is not None else "")
        + f"{fail_txt}. Grey bars failed. The I²t spend is the peak of the busiest coil's accumulator E⁺ = max(0, E + (i² − "
        "i_cont²)·dt) from the recorded current at 2 kHz; the core clamp folds its reference back at 0.95 of the budget, and a "
        "spend above 1 means the measured current overran the budget. At the open position the four element gaps agree to "
        f"{max(v for v in sims['gap_spread_open_um'] if v is not None):.1f} µm at zero steady current: at the preset travel every element stays "
        "fully engaged, so (a)'s envelope is not approached. These are simulated results of the current controller, not a "
        "hardware rating."
    )
    title = "drawer_envelope: DRAWER payload limits"
    stamp = finish(fig, title=title, question=QUESTION, caption=caption, meta=stamp_meta(cfg, "drawer_envelope", "carriage6"), hspace=0.36)
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "m_star_kg": m_star,
        "m_op_kg": list(m_op),
        "travel_mm": travel * 1e3,
        "body_length_mm": length * 1e3,
        "element_force_max_n": element_force_max(p),
        "envelope": {
            "m_kg": floats(masses),
            "max_extension_frac": frac,
            "saturated": sat,
            "first_saturated_kg": first_sat,
            "rated_extension_frac": rated_frac,
            "rated_saturated": bool(rated_sat),
        },
        "reference_lines": refs,
        "currents": {
            "i_cont_a": i_cont,
            "i_pulse_a": i_pulse,
            "t_pulse_s": t_pulse,
            "v_bus_over_r_a": v_over_r,
            "unseat_bias_a": i_seat,
            "relift_bias_a": i_land,
            "g_seat_mm": g_seat * 1e3,
            "g_land_mm": g_land * 1e3,
        },
        "mass_limits": marks,
        "i2t_budget_a2s": budget,
        "i2t_budget_frac": 1.0,
        "slot_exit": {**sims, "bar_x": floats(xpos), "succeeds_up_to_kg": limit_ok, "first_failure_kg": first_fail},
        "open_gap_curve": {"m_kg": floats(m_curve), "gap_mm": g_curve, "band_edge_mm": lo_b * 1e3},
        "checks": checks,
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="drawer_envelope", tier=1, question=QUESTION, run=run)
