"""gain_margin: how much eddy-current lag the zero-power levitation loops tolerate.

Claim backed: LEV-020 (stable for eddy lag τ_e ≤ 2 ms in every class at preset gains;
"the gain_margin study reports the critical τ_e per class") and the TECHNICAL_DESIGN /
SPECIFICATION non-claim that the design assumes a laminated or SMC rail.

Method.

- Design model (PHYSICS §15; the six-state discrete loop of ``pole_map``): the decay rate
  −ln ρ/T of the least-stable closed-loop pole as the *plant* eddy lag τ_e grows while
  the gains stay at the preset discrete design (nominal τ_e). The critical τ_e is found
  by bisection per class and mode; heave is cross-checked against the native derived
  ``critical_tau_e_heave_s``. A class's critical τ_e is the minimum over its active modes.
- Closed loop (voltage coil mode; Carriage6 for DRAWER, DOOR and WALL, Heave1 for HEAVY,
  whose Carriage6 roll/sway loop is an open Wave-2 item): 3–4 s levitated hover with the
  plant's ``design.suspension.tau_eddy_s`` swept. The sensor-noise-driven RMS of the
  limiting mode's channel rises as its damping falls. ``params.validate`` rejects a plant
  τ_e at or beyond the design-model critical value (LEV-020), so the sweep ends at
  99.5 % of it.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .. import plotstyle
from .pole_map import MODES, critical_tau_e, decay_rate_per_s, gains, mode_active, mode_model
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

QUESTION = "How much eddy-current lag can each class's levitation loop tolerate before it goes unstable (critical τ_e per class and mode)?"

_MODE_LS = {"heave": "-", "pitch": (0, (5, 3)), "roll": (0, (1.2, 2.2))}
_MODE_HATCH = {"heave": None, "pitch": "///", "roll": "..."}
_CHANNEL = {"heave": "z_m", "pitch": "pitch_rad", "roll": "roll_rad"}
_LEV020_S = 2.0e-3  # FAB-LEV-020 bound (SPECIFICATION §LEV)


def _sim_tier(cls: str) -> str:
    return "heave1" if cls == "heavy" else "carriage6"


def run(cfg: Any) -> StudyResult:
    from .. import scenarios

    q = quick()
    tau_grid = np.linspace(0.0, 6.0e-3, 13 if q else 61)
    tol = 1e-5 if q else 1e-6
    fig, axes = new_figure(1, 3, size=(16.0, 6.4), gridspec_kw={"width_ratios": [1.25, 1.0, 1.0]})
    ax_d, ax_c, ax_n = axes[0]
    rec: dict[str, Any] = {}
    checks: dict[str, Any] = {}
    jobs, keys = [], []
    for cls in CLASSES:
        p = class_cfg(cfg, cls).params()
        st = class_style(cls)
        tau_nom = float(p.get("control.levitation.tau_e_nominal_s"))
        crec: dict[str, Any] = {
            "tau_e_nominal_ms": tau_nom * 1e3,
            "tau_e_plant_ms": float(p.get("design.suspension.tau_eddy_s")) * 1e3,
            "modes": {},
        }
        crit_all: dict[str, float] = {}
        for mode in MODES:
            if not mode_active(p, mode):
                crec["modes"][mode] = {"active": False}
                continue
            k = gains(p, mode)
            mm = mode_model(p, mode)
            decay = [decay_rate_per_s(p, mode, k, float(te), mm) for te in tau_grid]
            crit = critical_tau_e(p, mode, tol_s=tol)
            crit_all[mode] = crit
            ax_d.plot(tau_grid * 1e3, decay, color=st["color"], ls=_MODE_LS[mode], lw=1.7)
            ax_d.plot(
                [crit * 1e3], [0.0], ls="none", marker=st["marker"], ms=8, color=st["color"], mec=plotstyle.COLORS["surface"], mew=1.2
            )
            crec["modes"][mode] = {
                "active": True,
                "tau_e_ms": floats(tau_grid * 1e3),
                "decay_rate_per_s": decay,
                "critical_tau_e_ms": crit * 1e3,
                "critical_marker_y": 0.0,
            }
        native_heave = float(p.derived()["critical_tau_e_heave_s"])
        crec["critical_tau_e_heave_native_ms"] = native_heave * 1e3
        crec["heave_mirror_abs_err_us"] = abs(crit_all["heave"] - native_heave) * 1e6
        limiting = min(crit_all, key=lambda m: crit_all[m])
        tau_lim = crit_all[limiting]
        crec["limiting_mode"] = limiting
        crec["critical_tau_e_ms"] = tau_lim * 1e3
        checks[f"{cls}.lev020_2ms"] = check(
            tau_lim > _LEV020_S, tau_lim * 1e3, _LEV020_S * 1e3, "FAB-LEV-020 (critical τ_e > 2 ms, design model), ms"
        )
        checks[f"{cls}.heave_mirror_vs_native"] = check(
            crec["heave_mirror_abs_err_us"] <= 2.0 * tol * 1e6,
            crec["heave_mirror_abs_err_us"],
            2.0 * tol * 1e6,
            "derive::zp_critical_tau_e (bisection tolerance), µs",
        )
        # Closed-loop sweep of the plant eddy lag.
        tier = _sim_tier(cls)
        meas_mode = limiting if tier == "carriage6" else "heave"
        base = [0.5e-3, 1.0e-3, 2.0e-3] if q else [0.25e-3, 0.5e-3, 1.0e-3, 1.5e-3, 2.0e-3]
        fracs = [0.97] if q else [0.9, 0.95, 0.99, 0.995]
        sweep = sorted({round(v, 9) for v in base + [f * tau_lim for f in fracs] if v < tau_lim})
        dur = 2.0 if q else 4.0
        for te in sweep:
            c = class_cfg(cfg, cls, duration_s=dur, overrides={**cfg.overrides, "design.suspension.tau_eddy_s": float(te)})
            pp = c.params()
            d, s = scenarios.spec("hover", pp, c, fidelity=tier)
            d["record"] = {"rate_hz": 2000.0, "channels": [_CHANNEL[meas_mode]]}
            d["metrics"] = {"windows": {}}
            jobs.append((pp, d, s))
            keys.append((cls, te))
        crec["sim"] = {
            "tier": tier,
            "mode": meas_mode,
            "channel": _CHANNEL[meas_mode],
            "duration_s": dur,
            "critical_tau_e_measured_mode_ms": crit_all[meas_mode] * 1e3,
        }
        rec[cls] = crec

    results = run_jobs(jobs, cfg)
    for cls in CLASSES:
        st = class_style(cls)
        sim = rec[cls]["sim"]
        rows = [(te, r) for (kc, te), r in zip(keys, results, strict=True) if kc == cls]
        taus, rms, faults, rms_early, rms_late, f_peak = [], [], [], [], [], []
        for te, r in rows:
            y = r[sim["channel"]]
            w = r.t_s >= 1.0
            rms.append(float(np.std(y[w])))
            rms_early.append(float(np.std(y[(r.t_s >= 1.0) & (r.t_s < 2.0)])))
            late = r.t_s >= r.t_s[-1] - 1.0
            rms_late.append(float(np.std(y[late])))
            yy = y[w] - np.mean(y[w])
            spec = np.abs(np.fft.rfft(yy))
            freqs = np.fft.rfftfreq(yy.size, 1.0 / r.rate_hz)
            f_peak.append(float(freqs[1 + int(np.argmax(spec[1:]))]) if yy.size > 2 else 0.0)
            taus.append(te)
            faults.append(faults_of(r))
        i_ref = int(np.argmin([abs(t - 1.0e-3) for t in taus]))
        norm = [v / rms[i_ref] for v in rms]
        x = [t / (sim["critical_tau_e_measured_mode_ms"] * 1e-3) for t in taus]
        ax_n.plot(
            x,
            norm,
            color=st["color"],
            lw=1.6,
            marker=st["marker"],
            ms=7,
            mec=plotstyle.COLORS["surface"],
            mew=1.0,
            label=f"{st['label']} ({sim['mode']}, {sim['tier']})",
        )
        sustained = [late > 5.0 * early for early, late in zip(rms_early, rms_late, strict=True)]
        onset = next((t for t, flag in zip(taus, sustained, strict=True) if flag), None)
        sim.update(
            {
                "tau_e_ms": floats(np.array(taus) * 1e3),
                "rms": rms,
                "rms_over_rms_at_1ms": norm,
                "tau_over_critical": x,
                "rms_1_to_2s": rms_early,
                "rms_last_1s": rms_late,
                "peak_frequency_hz": f_peak,
                "sustained_oscillation": sustained,
                "sustained_onset_tau_e_ms": None if onset is None else onset * 1e3,
                "faults": faults,
                "reference_tau_e_ms": taus[i_ref] * 1e3,
            }
        )
        checks[f"{cls}.sim_no_sustained_oscillation_to_2ms"] = check(
            onset is None or onset > _LEV020_S,
            None if onset is None else onset * 1e3,
            _LEV020_S * 1e3,
            "FAB-LEV-020 in closed loop: no growing/sustained oscillation up to 2 ms (last-second RMS < 5× the 1–2 s RMS), ms",
        )
        checks[f"{cls}.sim_no_faults"] = check(
            not any(faults), None, None, "closed-loop hover without raised faults up to 99.5 % of the critical τ_e"
        )

    # (a) decay-rate curves with the LEV-020 bound and the nominal design lag.
    plotstyle.zero_line(ax_d)
    lines = {"lev020_bound_ms": _LEV020_S * 1e3, "nominal_design_ms": rec["drawer"]["tau_e_nominal_ms"]}
    ax_d.axvline(lines["lev020_bound_ms"], color=plotstyle.COLORS["star"], ls=(0, (4, 4)), lw=1.2)
    ax_d.axvline(lines["nominal_design_ms"], color=plotstyle.COLORS["muted"], ls=(0, (4, 4)), lw=1.1)
    ax_d.annotate(
        "LEV-020 bound 2 ms",
        xy=(lines["lev020_bound_ms"], 0.97),
        xycoords=("data", "axes fraction"),
        xytext=(4, 0),
        textcoords="offset points",
        fontsize=8.5,
        color=plotstyle.COLORS["star"],
        va="top",
    )
    ax_d.annotate(
        "design point",
        xy=(lines["nominal_design_ms"], 0.97),
        xycoords=("data", "axes fraction"),
        xytext=(-4, 0),
        textcoords="offset points",
        fontsize=8.5,
        color=plotstyle.COLORS["muted"],
        va="top",
        ha="right",
    )
    plotstyle.style_ax(ax_d, "plant eddy lag τ_e (ms)", "decay rate of the slowest pole (1/s)", "(a) Design model: damping vs eddy lag")
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    handles = [Line2D([], [], color=plotstyle.CLASS_COLORS[c], lw=2, label=plotstyle.CLASS_LABELS[c]) for c in CLASSES]
    handles += [Line2D([], [], color=plotstyle.COLORS["ink2"], ls=_MODE_LS[m], lw=1.7, label=m) for m in MODES]
    ax_d.legend(handles=handles, loc="lower left", ncols=2, **plotstyle.LEGEND_KW)

    # (b) critical τ_e per class and mode.
    xs = np.arange(len(CLASSES), dtype=float)
    width = 0.26
    bars: dict[str, Any] = {}
    for j, mode in enumerate(MODES):
        vals, xpos, cols = [], [], []
        for i, cls in enumerate(CLASSES):
            m = rec[cls]["modes"][mode]
            if not m.get("active"):
                continue
            vals.append(m["critical_tau_e_ms"])
            xpos.append(xs[i] + (j - 1) * (width + 0.02))
            cols.append(plotstyle.CLASS_COLORS[cls])
        ax_c.bar(xpos, vals, width, color=cols, hatch=_MODE_HATCH[mode], edgecolor=plotstyle.COLORS["surface"], lw=0.0)
        bars[mode] = {"critical_tau_e_ms": vals, "bar_x": floats(xpos)}
    for i, cls in enumerate(CLASSES):
        v = rec[cls]["critical_tau_e_ms"]
        j = MODES.index(rec[cls]["limiting_mode"])
        ax_c.annotate(
            f"{v:.2f}",
            xy=(xs[i] + (j - 1) * (width + 0.02), v),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=8.5,
            color=plotstyle.COLORS["ink"],
            fontweight="semibold",
        )
    plotstyle.ref_line(ax_c, lines["lev020_bound_ms"], "LEV-020: 2 ms", color=plotstyle.COLORS["star"], x=0.36)
    ax_c.set_xticks(xs, [plotstyle.CLASS_LABELS[c] for c in CLASSES])
    plotstyle.style_ax(ax_c, None, "critical τ_e (ms)", "(b) Critical eddy lag per mode")
    ax_c.legend(
        handles=[
            Patch(facecolor=plotstyle.COLORS["ctrl"], hatch=_MODE_HATCH[m], edgecolor=plotstyle.COLORS["surface"], label=m) for m in MODES
        ],
        loc="upper left",
        **plotstyle.LEGEND_KW,
    )

    # (c) closed-loop noise response as τ_e approaches its critical value.
    ref_one = 1.0
    plotstyle.ref_line(ax_n, ref_one, "at the 1 ms design lag", color=plotstyle.COLORS["muted"])
    ax_n.set_yscale("log")
    plotstyle.style_ax(
        ax_n, "plant τ_e / critical τ_e of the measured mode", "hover RMS / RMS at τ_e = 1 ms", "(c) Closed loop: noise response"
    )
    ax_n.legend(loc="upper left", **plotstyle.LEGEND_KW)

    crit_txt = ", ".join(f"{plotstyle.CLASS_LABELS[c]} {rec[c]['critical_tau_e_ms']:.2f} ms ({rec[c]['limiting_mode']})" for c in CLASSES)
    heave_txt = " / ".join(f"{rec[c]['modes']['heave']['critical_tau_e_ms']:.2f}" for c in CLASSES)
    grow = {c: max(rec[c]["sim"]["rms_over_rms_at_1ms"]) for c in CLASSES}
    caption = (
        "(a) Decay rate of the least-stable pole of the six-state discrete design model (pole_map) as the plant eddy lag grows "
        "while the gains stay at their nominal-lag design; markers: the critical τ_e where it crosses zero. (b) Critical τ_e "
        f"per class and mode; per class (minimum over modes): {crit_txt}. Heave alone gives {heave_txt} ms (equal to the native "
        "derived critical_tau_e_heave_s), so the pitch or roll loop sets the margin. Every class stays stable to 2 ms "
        "(LEV-020) in this model; the implemented loops add a state observer and are bisected separately in "
        "crates/fabrail-core/tests/levitation_linear.rs. A solid-steel rail can exceed these lags: the design assumes a "
        "laminated or SMC rail.\n"
        "(c) Closed loop, voltage coil mode: levitated hover with the plant τ_e swept up to 99.5 % of the design critical value "
        "(params.validate rejects more). The sensor-noise-driven RMS of the limiting mode's channel rises by up to "
        + ", ".join(f"{grow[c]:.1f}× ({plotstyle.CLASS_LABELS[c]})" for c in CLASSES)
        + " as its damping falls. "
        + "".join(
            f"{plotstyle.CLASS_LABELS[c]}: a sustained {rec[c]['sim']['peak_frequency_hz'][rec[c]['sim']['sustained_oscillation'].index(True)]:.0f} Hz "
            f"oscillation (RMS {max(rec[c]['sim']['rms_last_1s']) * 1e3:.2f} m{'rad' if rec[c]['sim']['channel'] != 'z_m' else 'm'}) sets in from "
            f"τ_e = {rec[c]['sim']['sustained_onset_tau_e_ms']:.2f} ms, below the design-model critical value, a limit cycle the linear model "
            "does not predict. "
            for c in CLASSES
            if rec[c]["sim"]["sustained_onset_tau_e_ms"] is not None
        )
        + "HEAVY runs on Heave1 (heave only), because HEAVY Carriage6 roll/sway is an open item; "
        "its roll margin is shown by the design model only."
    )
    title = "gain_margin: critical eddy lag per class and mode"
    stamp = finish(
        fig, title=title, question=QUESTION, caption=caption, meta=stamp_meta(cfg, "gain_margin", "carriage6+heave1"), wspace=0.3
    )
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "reference_lines": lines,
        "reference_ratio": ref_one,
        "zero_decay_per_s": 0.0,
        "classes": rec,
        "bars": bars,
        "checks": checks,
        "bisection_tolerance_us": tol * 1e6,
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="gain_margin", tier=1, question=QUESTION, run=run)
