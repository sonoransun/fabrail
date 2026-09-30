"""Figures: ``plot_*`` functions that consume only :mod:`fabrail.analysis` dataclasses (core §B.8).

Every figure uses :mod:`fabrail.plotstyle` (``apply``, ``style_ax``, ``ref_line``,
``run_stamp``), and every plotted number is a field of the analysis it draws, so the
JSON record that carries the analysis carries every plotted number (FAB-SW-009;
checked with :func:`fabrail.export.missing_plotted` and :data:`PLOT_SCALES`). Each
function returns a matplotlib ``Figure`` (built without pyplot, so no global figure
state or GUI backend is involved) and saves it when ``path`` is given.

Design rules (house plotstyle; one axis per panel, never two y-scales):

- identity colours in the fixed categorical order (floor x/y/z = s1/s2/s3; classes by
  ``CLASS_COLORS``); per-element traces in the ``ctrl`` tone with the element mean in s1;
- limits and closed forms as dashed reference lines with a small label; event times
  (t_P, t_S, PROTECT, trips) as thin vertical rules;
- state timelines as post-steps with the state names as tick labels;
- units scaled for reading (mm, mm/s) with the factor in :data:`PLOT_SCALES`.

Owner: W0-C2 (stub), then B1 (Wave 2).
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from . import plotstyle as ps

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .analysis import EnsembleAnalysis, HapticAnalysis, LevitationAnalysis, SafetyAnalysis, SeismicAnalysis

__all__ = [
    "PLOT_SCALES",
    "plot_class_comparison",
    "plot_ensemble",
    "plot_haptics",
    "plot_levitation",
    "plot_safety",
    "plot_seismic",
]

#: Unit factors the figures apply to recorded SI values (1 = SI, 1e3 = mm or mm/s);
#: pass them to :func:`fabrail.export.missing_plotted` as ``scales``.
PLOT_SCALES: tuple[float, ...] = (1.0, 1e3)

_MM = 1e3
_PANEL_H = 2.15
_WIDTH = 8.6


# ---- figure scaffolding --------------------------------------------------------------------------


def _figure(n_rows: int, *, width: float = _WIDTH, row_h: float = _PANEL_H) -> Any:
    from matplotlib.figure import Figure

    ps.apply()
    fig = Figure(figsize=(width, row_h * n_rows + 0.9), layout="constrained")
    engine: Any = fig.get_layout_engine()  # ConstrainedLayoutEngine (layout="constrained")
    engine.set(rect=(0.0, 0.03, 1.0, 0.97), h_pad=0.06, hspace=0.04)
    return fig


def _title(fig: Any, analysis: Any, what: str) -> None:
    meta = getattr(analysis, "meta", {}) or {}
    cls = str(getattr(analysis, "rail_class", "") or meta.get("rail_class", "")).upper()
    scen = str(getattr(analysis, "scenario", "") or meta.get("scenario", ""))
    head = " · ".join(p for p in (cls, scen) if p)
    fig.suptitle(f"{what} — {head}" if head else what, x=0.01, ha="left", color=ps.COLORS["ink"])


def _finish(fig: Any, meta: Mapping[str, Any] | None, path: str | Path | None, note: str | None = None) -> Any:
    ps.run_stamp(fig, meta)
    if note:
        fig.text(0.01, 0.022, note, ha="left", va="bottom", fontsize=6.5, family="monospace", color=ps.COLORS["muted"])
    ps.save(fig, path)
    return fig


def _legend(ax: Any, **kw: Any) -> None:
    handles, labels = ax.get_legend_handles_labels()
    if len(handles) >= 2:
        ax.legend(**{**ps.LEGEND_KW, "loc": "best", **kw})


def _has(a: np.ndarray) -> bool:
    return isinstance(a, np.ndarray) and a.size > 0 and bool(np.any(np.isfinite(np.asarray(a, dtype=np.float64))))


def _v(x: float, spec: str = ".3g") -> str:
    """A number for a title; "n/a" when it was not measured (NaN)."""
    return format(float(x), spec) if _finite(x) else "n/a"


def _finite(x: float) -> bool:
    return isinstance(x, int | float) and math.isfinite(float(x))


def _vline(ax: Any, t: float, label: str | None, color: str, *, style: Any = "-", text_y: float = 0.97) -> None:
    if not _finite(t):
        return
    ax.axvline(t, color=color, linestyle=style, linewidth=1.0, zorder=2)
    if label:
        ax.annotate(
            label,
            xy=(t, text_y),
            xycoords=("data", "axes fraction"),
            xytext=(3, 0),
            textcoords="offset points",
            ha="left",
            va="top",
            fontsize=8,
            color=ps.COLORS["ink2"],
        )


def _state_axis(ax: Any, codes: np.ndarray, names: Sequence[str]) -> None:
    present = sorted({int(c) for c in np.asarray(codes, dtype=np.int64)})
    if not present:
        return
    ax.set_yticks(present)
    ax.set_yticklabels([names[c] if 0 <= c < len(names) else f"?{c}" for c in present], fontsize=8)
    ax.set_ylim(min(present) - 0.6, max(present) + 0.6)


def _step(ax: Any, t: np.ndarray, codes: np.ndarray, color: str, label: str) -> None:
    ax.step(t, np.asarray(codes, dtype=np.float64), where="post", color=color, linewidth=1.6, label=label)


def _x_scale(x: np.ndarray) -> tuple[float, str]:
    """mm below 2 m of travel, else m."""
    span = float(np.nanmax(np.abs(x))) if _has(x) else 0.0
    return (_MM, "mm") if span < 2.0 else (1.0, "m")


# ---- levitation ------------------------------------------------------------------------------------


def plot_levitation(analysis: LevitationAnalysis, path: str | Path | None = None) -> Any:
    """Gap and coil current against time, gap PSD, zero-power convergence."""
    a = analysis
    has_mhat = _has(a.m_hat_kg_t)
    has_psd = _has(a.gap_psd_m2_per_hz)
    n_rows = 2 + int(has_mhat) + int(has_psd)
    fig = _figure(n_rows)
    gs = fig.add_gridspec(n_rows, 1)
    ax_gap = fig.add_subplot(gs[0])
    axes = [ax_gap]
    for r in range(1, 2 + int(has_mhat)):
        axes.append(fig.add_subplot(gs[r], sharex=ax_gap))
    t = a.t_s
    ctrl = ps.COLORS["ctrl"]

    gap = np.atleast_2d(a.gap_m) if a.gap_m.size else np.zeros((0, t.size))
    for i, g in enumerate(gap):
        ax_gap.plot(t, g * _MM, color=ctrl, linewidth=0.9, alpha=0.8, label="per element" if i == 0 else None)
    if _has(a.gap_mean_t_m):
        ax_gap.plot(t, a.gap_mean_t_m * _MM, color=ps.COLORS["s1"], linewidth=1.8, label="element mean")
    if len(a.gap_limits_m) == 2:
        lo, hi = a.gap_limits_m
        ps.ref_line(ax_gap, lo * _MM, "monitor gap_min", va="bottom")
        ps.ref_line(ax_gap, hi * _MM, "monitor gap_max", va="top")
    if _finite(a.g_star_m):
        ps.ref_line(ax_gap, a.g_star_m * _MM, "g* (design)", color=ps.COLORS["star"], x=0.6)
    ps.style_ax(
        ax_gap, ylabel="gap (mm)", title=f"gap: mean {_v(a.gap_mean_m * _MM, '.4f')} mm, rms {_v(a.gap_rms_m * 1e6, '.3g')} µm"
    )
    _legend(ax_gap, loc="center right")

    ax_i = axes[1]
    coil = np.atleast_2d(a.coil_i_a) if a.coil_i_a.size else np.zeros((0, t.size))
    for i, c in enumerate(coil):
        ax_i.plot(t, c, color=ctrl, linewidth=0.9, alpha=0.85, label="per element" if i == 0 else None)
    ps.zero_line(ax_i)
    ps.style_ax(ax_i, ylabel="coil current (A)", title=f"zero-power: i_ss {_v(a.coil_i_ss_a)} A, hover {_v(a.hover_power_w)} W")
    if has_mhat:
        ax_m = axes[2]
        ax_m.plot(t, a.m_hat_kg_t, color=ps.COLORS["s1"], linewidth=1.6, label="m̂")
        if _finite(a.m_star_kg):
            ps.ref_line(ax_m, a.m_star_kg, "m* (design)", color=ps.COLORS["star"])
        ps.style_ax(ax_m, ylabel="m̂ (kg)", title=f"payload estimate, last levitated {_v(a.m_hat_kg, '.4g')} kg")
    axes[-1].set_xlabel("time (s)", color=ps.COLORS["ink2"])
    for ax in axes[:-1]:
        ax.tick_params(labelbottom=False)
    if has_psd:
        ax_p = fig.add_subplot(gs[n_rows - 1])
        ax_p.loglog(a.psd_f_hz, a.gap_psd_m2_per_hz, color=ps.COLORS["s1"], linewidth=1.4)
        win = f" over {a.psd_window_s[0]:.3g}–{a.psd_window_s[1]:.3g} s" if len(a.psd_window_s) == 2 else ""
        ps.style_ax(ax_p, xlabel="frequency (Hz)", ylabel="PSD (m²/Hz)", title=f"Welch PSD of the mean gap{win}")
    _title(fig, a, "Levitation")
    return _finish(fig, a.meta, path)


# ---- haptics ---------------------------------------------------------------------------------------


def plot_haptics(analysis: HapticAnalysis, path: str | Path | None = None) -> Any:
    """Position, speed and hand force; apparent mass; the semilog terminus inset."""
    a = analysis
    has_term = _has(a.terminus_d_m)
    n_rows = 3 + int(has_term)
    fig = _figure(n_rows)
    gs = fig.add_gridspec(n_rows, 1)
    ax_x = fig.add_subplot(gs[0])
    ax_v = fig.add_subplot(gs[1], sharex=ax_x)
    ax_f = fig.add_subplot(gs[2], sharex=ax_x)
    t = a.t_s
    xs, xu = _x_scale(a.x_m)

    ax_x.plot(t, a.x_m * xs, color=ps.COLORS["s1"], linewidth=1.8, label="x (truth)")
    if _has(a.x_ref_m):
        ax_x.plot(t, a.x_ref_m * xs, color=ps.COLORS["s2"], linewidth=1.1, linestyle=(0, (4, 3)), label="x_ref")
    ps.style_ax(ax_x, ylabel=f"position ({xu})", title="travel (+x opens)")
    _legend(ax_x)

    ax_v.plot(t, a.v_mps * _MM, color=ps.COLORS["s1"], linewidth=1.6, label="v (truth)")
    if len(a.v_cap_band_mps) == 2:
        ps.ref_line(ax_v, a.v_cap_band_mps[1] * _MM, "speed cap (open)", va="bottom")
        ps.ref_line(ax_v, a.v_cap_band_mps[0] * _MM, "speed cap (close)", va="top")
    ps.zero_line(ax_v)
    ps.style_ax(ax_v, ylabel="speed (mm/s)")

    ax_f.plot(t, a.f_hand_n, color=ps.COLORS["s1"], linewidth=1.5, label="F_hand (truth)")
    if _has(a.f_hand_hat_n):
        ax_f.plot(t, a.f_hand_hat_n, color=ps.COLORS["s2"], linewidth=1.1, label="F̂_hand (observer)")
    ps.zero_line(ax_f)
    bits = []
    if _finite(a.apparent_mass_kg) and _finite(a.m_virtual_kg):
        bits.append(f"apparent mass {a.apparent_mass_kg:.3g} kg vs M_v {a.m_virtual_kg:.3g} kg")
    if _finite(a.breakaway_n):
        bits.append(f"breakaway {a.breakaway_n:.3g} N")
    if _finite(a.intent_confirm_s):
        bits.append(f"intent confirm {a.intent_confirm_s * 1e3:.0f} ms")
    ps.style_ax(ax_f, xlabel="time (s)", ylabel="hand force (N)", title="; ".join(bits) or "hand force")
    _legend(ax_f)
    for ax in (ax_x, ax_v):
        ax.tick_params(labelbottom=False)

    if has_term:
        ax_t = fig.add_subplot(gs[3])
        ax_t.semilogy(a.terminus_t_s, a.terminus_d_m * _MM, color=ps.COLORS["s1"], linewidth=1.6, label="distance-to-go (truth)")
        if _has(a.terminus_fit_d_m):
            ax_t.semilogy(
                a.terminus_fit_t_s,
                a.terminus_fit_d_m * _MM,
                color=ps.COLORS["s2"],
                linewidth=1.2,
                linestyle=(0, (5, 3)),
                label="semilog fit",
            )
        from .analysis import TERMINUS_D_EPS_M

        ps.ref_line(ax_t, TERMINUS_D_EPS_M * _MM, "d_ε 0.2 mm", va="top")
        if _finite(a.terminus_d_b_m):
            ps.ref_line(ax_t, a.terminus_d_b_m * _MM, "d_b", va="bottom")
        head = "log-damped terminus"
        if _finite(a.terminus_tau_fit_s):
            head += f": τ_fit {a.terminus_tau_fit_s:.4g} s, R² {a.terminus_semilog_r2:.6f} ({a.terminus_fit_source})"
        if _finite(a.arrival_v_mps):
            head += f", arrival {a.arrival_v_mps * _MM:.3g} mm/s"
        ps.style_ax(ax_t, xlabel="time since capture (s)", ylabel="d (mm, log)", title=head)
        _legend(ax_t, loc="lower left")
    _title(fig, a, "Haptics and motion")
    return _finish(fig, a.meta, path)


# ---- safety ----------------------------------------------------------------------------------------


def plot_safety(analysis: SafetyAnalysis, path: str | Path | None = None) -> Any:
    """Governed kinetic energy against the cap, contact force, monitor demand timeline."""
    from .analysis import DEMAND_NAMES

    a = analysis
    has_ledger = _has(a.e_bus_j) or _has(a.e_kin_j) or _has(a.e_drag_j)
    n_rows = 3 + int(has_ledger)
    fig = _figure(n_rows)
    gs = fig.add_gridspec(n_rows, 1)
    ax_e = fig.add_subplot(gs[0])
    axes = [ax_e] + [fig.add_subplot(gs[r], sharex=ax_e) for r in range(1, n_rows)]
    t = a.t_s

    ax_e.plot(t, a.e_kin_gov_j, color=ps.COLORS["s1"], linewidth=1.6, label="½·m_gov·v² (governor)")
    if _finite(a.ke_cap_j):
        ps.ref_line(ax_e, a.ke_cap_j, f"KE cap {a.ke_cap_j:g} J", color=ps.COLORS["star"])
    head = f"kinetic energy: max {_v(a.ke_max_gov_j)} J"
    if _finite(a.momentum_max_kgmps):
        cap = f" (cap {a.momentum_cap_kgmps:g})" if _finite(a.momentum_cap_kgmps) else " (no momentum cap: hold-to-run class)"
        head += f"; momentum max {a.momentum_max_kgmps:.3g} kg·m/s{cap}"
    ps.style_ax(ax_e, ylabel="energy (J)", title=head)

    ax_c = axes[1]
    ax_c.plot(t, a.contact_force_n, color=ps.COLORS["s1"], linewidth=1.5, label="obstacle contact force (truth)")
    if _has(a.force_cap_n):
        ax_c.plot(t, a.force_cap_n, color=ps.COLORS["ctrl"], linewidth=1.1, linestyle=(0, (4, 3)), label="monitor force cap")
    ps.zero_line(ax_c)
    ps.style_ax(ax_c, ylabel="force (N)", title=f"contact force max {_v(a.contact_force_max_n)} N")
    _legend(ax_c)

    ax_d = axes[2]
    if a.demand.size:
        _step(ax_d, t, a.demand, ps.COLORS["s1"], "monitor demand")
        _state_axis(ax_d, np.concatenate([a.demand.astype(np.int64), [0]]), DEMAND_NAMES)
    for t_trip, name in a.trips:
        _vline(ax_d, t_trip, name, ps.COLORS["accent"])
    faults = ", ".join(a.faults) if a.faults else "none"
    ps.style_ax(ax_d, ylabel="demand", title=f"monitor demand; faults: {faults}", grid=False)

    if has_ledger:
        ax_l = axes[3]
        for arr, label, color in (
            (a.e_bus_j, "bus energy drawn (time integral of p_bus)", ps.COLORS["s1"]),
            (a.e_kin_j, "kinetic energy (truth)", ps.COLORS["s2"]),
            (a.e_drag_j, "drag dissipation (cumulative)", ps.COLORS["s3"]),
        ):
            if _has(arr):
                y = np.where(arr > 0, arr, np.nan)
                ax_l.plot(t, y, color=color, linewidth=1.4, label=label)
        ax_l.set_yscale("log")
        peaks = [float(np.nanmax(arr)) for arr in (a.e_bus_j, a.e_kin_j, a.e_drag_j) if _has(arr)]
        top = max(peaks) if peaks else 0.0
        if top > 0.0:
            ax_l.set_ylim(bottom=top * 1e-5, top=top * 3.0)  # view only: the recorded ledger keeps every sample
        ledger = ", ".join(f"{k} {v:.3g}" for k, v in a.energy_ledger_j.items())
        ps.style_ax(ax_l, ylabel="energy (J, log)", title="energy ledger" + (f": {ledger}" if ledger else ""))
        _legend(ax_l, loc="lower right")
    axes[-1].set_xlabel("time (s)", color=ps.COLORS["ink2"])
    for ax in axes[:-1]:
        ax.tick_params(labelbottom=False)
    _title(fig, a, "Safety")
    return _finish(fig, a.meta, path)


# ---- seismic ---------------------------------------------------------------------------------------


def plot_seismic(analysis: SeismicAnalysis, path: str | Path | None = None) -> Any:
    """Floor acceleration, level and state timeline against t_P and t_S."""
    from .analysis import LEV_STATE_NAMES, LEVEL_NAMES, SEISMIC_STATE_NAMES

    a = analysis
    has_lev = a.lev_state.size > 0
    n_rows = 3 + int(has_lev)
    fig = _figure(n_rows)
    gs = fig.add_gridspec(n_rows, 1)
    ax_a = fig.add_subplot(gs[0])
    axes = [ax_a] + [fig.add_subplot(gs[r], sharex=ax_a) for r in range(1, n_rows)]
    t = a.t_s
    acc = np.atleast_2d(a.floor_acc_mps2) if a.floor_acc_mps2.size else np.zeros((0, t.size))
    for i, (row, label) in enumerate(zip(acc, ("x (travel)", "y (cross-track)", "z (up)"), strict=False)):
        ax_a.plot(t, row, color=ps.SERIES[i], linewidth=0.9, label=f"floor a_{label}")
    ps.zero_line(ax_a)
    head = "floor acceleration"
    if _finite(a.pga_mps2):
        head += f": horizontal peak {a.pga_mps2:.3g} m/s²"
    ps.style_ax(ax_a, ylabel="a (m/s²)", title=head)
    _legend(ax_a, loc="lower left", ncols=3)

    ax_l = axes[1]
    if a.level.size:
        _step(ax_l, t, a.level, ps.COLORS["s1"], "detector level")
        _state_axis(ax_l, np.arange(len(LEVEL_NAMES)), LEVEL_NAMES)
    ps.style_ax(ax_l, ylabel="level", title=f"detector level (max L{a.level_max})", grid=False)
    ax_s = axes[2]
    if a.seismic_state.size:
        _step(ax_s, t, a.seismic_state, ps.COLORS["s1"], "seismic state")
        _state_axis(ax_s, np.concatenate([a.seismic_state.astype(np.int64), [0]]), SEISMIC_STATE_NAMES)
    ps.style_ax(ax_s, ylabel="state", title="seismic state", grid=False)
    if has_lev:
        ax_v = axes[3]
        _step(ax_v, t, a.lev_state, ps.COLORS["s1"], "supervisor state")
        _state_axis(ax_v, a.lev_state, LEV_STATE_NAMES)
        ps.style_ax(ax_v, ylabel="state", title="supervisor (levitation) state", grid=False)

    dashed = (0, (3, 2))
    marks: list[tuple[float, str | None, str, Any]] = [
        (a.t_p_s, "t_P, t_S (synthesis)", ps.COLORS["star"], dashed),
        (a.t_s_arrival_s, None, ps.COLORS["star"], dashed),
        (a.alert_t_s, "network alert (synthetic)", ps.COLORS["s4"], "-"),
        (a.protect_t_s, "PROTECT", ps.COLORS["s5"], "-"),
        (a.land_t_s, "landed", ps.COLORS["ink2"], "-"),
    ]
    for ax in axes:
        for tm, mark_label, color, style in marks:
            if _finite(tm):
                ax.axvline(tm, color=color, linestyle=style, linewidth=1.0, zorder=2, label=mark_label if ax is ax_l else None)
    if a.level.size:
        _legend(ax_l, loc="center left", fontsize=8)
    events = [
        f"{label.split(' (')[0]} {tm:.2f} s"
        for tm, label in (
            (a.t_p_s, "t_P"),
            (a.trigger_t_s, "trigger"),
            (a.protect_t_s, "PROTECT"),
            (a.land_t_s, "landed"),
            (a.t_s_arrival_s, "t_S"),
        )
        if _finite(tm)
    ]
    if events:
        ax_s.set_title("seismic state: " + " · ".join(events), loc="left", fontsize=ps.FONT["title"], color=ps.COLORS["ink"])
    if t.size and _finite(a.t_p_s):
        ax_a.set_xlim(max(float(t[0]), a.t_p_s - 5.0), float(t[-1]))  # view only; every sample is recorded
    axes[-1].set_xlabel("time (s)", color=ps.COLORS["ink2"])
    for ax in axes[:-1]:
        ax.tick_params(labelbottom=False)
    note = None
    if _finite(a.protect_margin_to_s_s):
        note = f"PROTECT {a.protect_margin_to_s_s:.3g} s before the predicted S arrival (detector S-ETA)"
    _title(fig, a, "Seismic")
    return _finish(fig, a.meta, path, note)


# ---- ensemble --------------------------------------------------------------------------------------

#: KPIs an ensemble figure shows by default, in order, when no thresholds are given.
_HEADLINE_KPIS = (
    "stroke_max_m",
    "bumper_energy_j",
    "ke_max_gov_j",
    "momentum_max",
    "contact_force_max_n",
    "level_max",
    "trigger_t_s",
    "hover_gap_rms_m",
    "force_noise_rms_n",
    "coil_i_ss_a",
)


def plot_ensemble(analysis: EnsembleAnalysis, path: str | Path | None = None, *, names: Sequence[str] | None = None) -> Any:
    """KPI distributions with P50/P95 and the Wilson upper bound."""
    a = analysis
    if names is None:
        chosen = [k for k in a.thresholds if k in a.kpis]
        chosen += [k for k in _HEADLINE_KPIS if k in a.kpis and k not in chosen and a.n_reporting.get(k, 0) > 0]
        names = chosen[:6] or [k for k in a.kpis if "." not in k and a.n_reporting.get(k, 0) > 0][:6]
    names = list(names)
    n_cols = 2 if len(names) > 1 else 1
    n_rows = max(1, math.ceil(len(names) / n_cols))
    fig = _figure(n_rows, row_h=2.4)
    axes = fig.subplots(n_rows, n_cols, squeeze=False).ravel()
    idx = a.member_index if a.member_index.size else np.arange(a.n, dtype=np.float64)
    for ax, name in zip(axes, names, strict=False):
        y = np.asarray(a.kpis.get(name, np.zeros(0)), dtype=np.float64)
        ax.plot(idx[: y.size], y, linestyle="none", marker="o", markersize=4.5, color=ps.COLORS["s1"], label="members")
        if _finite(a.p50.get(name, math.nan)):
            ps.ref_line(ax, a.p50[name], "P50", va="bottom", x=0.2)
        if _finite(a.p95.get(name, math.nan)):
            ps.ref_line(ax, a.p95[name], "P95", va="bottom")
        sub = f"n = {a.n_reporting.get(name, 0)}"
        if name in a.thresholds:
            ps.ref_line(ax, a.thresholds[name], "threshold", color=ps.COLORS["star"], va="top")
            k = a.exceedances.get(name, 0)
            sub += f"; {k} above; Wilson 95 % upper {a.wilson_upper.get(name, math.nan):.3g}"
        ps.style_ax(ax, xlabel="member", ylabel=name, title=sub)
        from matplotlib.ticker import MaxNLocator

        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    for ax in axes[len(names) :]:
        ax.set_visible(False)
    fig.suptitle(f"Ensemble — {a.scenario} (n = {a.n}, seed {a.seed})", x=0.01, ha="left", color=ps.COLORS["ink"])
    meta = {**a.meta, "seed": a.seed}
    return _finish(fig, meta, path)


# ---- class comparison ------------------------------------------------------------------------------


def plot_class_comparison(analyses: Mapping[str, HapticAnalysis], path: str | Path | None = None) -> Any:
    """The same scenario across classes (``fabrail compare``), coloured by ``CLASS_COLORS``."""
    fig = _figure(2)
    ax_x, ax_v = fig.subplots(2, 1, sharex=True)
    scenario = ""
    first_meta: Mapping[str, Any] = {}
    for cls, a in analyses.items():
        color = ps.CLASS_COLORS.get(cls, ps.COLORS["ctrl"])
        label = ps.CLASS_LABELS.get(cls, cls.upper())
        ax_x.plot(a.t_s, a.x_m, color=color, linewidth=1.6, label=label)
        ax_v.plot(a.t_s, a.v_mps, color=color, linewidth=1.4, label=label)
        scenario = scenario or a.scenario
        first_meta = first_meta or a.meta
    ps.style_ax(ax_x, ylabel="position (m)", title="travel (+x opens)")
    ps.zero_line(ax_v)
    ps.style_ax(ax_v, xlabel="time (s)", ylabel="speed (m/s)", title="speed (each class under its own caps)")
    ax_x.tick_params(labelbottom=False)
    _legend(ax_x, loc="best")
    fig.suptitle(f"Class comparison — {scenario}", x=0.01, ha="left", color=ps.COLORS["ink"])
    meta = {k: v for k, v in dict(first_meta).items() if k not in ("rail_class", "spec_name")}
    meta["scenario"] = f"{'/'.join(analyses)}: {scenario}"
    return _finish(fig, meta, path)
