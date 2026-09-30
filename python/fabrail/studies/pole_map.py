"""pole_map: open- and closed-loop levitation poles of every class and mode.

Claim backed: LEV-003 (open-loop pole p = 69.49 / 47.57 / 32.50 / 32.50 rad/s) and
LEV-007 (the discrete design places the dominant poles at ω = 2.5·p with ζ ≥ 0.5, and the
loop stays stable across the zero-power float band).

Method (PHYSICS §15). The zero-power loop of each rigid-body mode is the six-state
discrete model of ``fabrail_params::derive``: the ZOH plant (δg, δġ, coil current with
the current-loop lag τ_cl, eddy-lagged force current), the Mizuno integral and the
one-period LET command delay. The model is mirrored here with scipy (``expm``), the
gains are the native discrete gains (``_native.zp_gains(p, mode, True)``), and the
closed-loop z-plane poles are mapped to s = ln(z)/T. The mirror is cross-checked
against the native derived quantities (open-loop poles, heave critical eddy lag).

Across the zero-power band (heave): a mass m floats at g(m) with zero current; the
modal pole is p(g)² = 4g₀/(2g + h′) and the current gain per unit modal mass
b = 2N·g₀/(M_pm + N·i_b) (i_b = 0 inside the band; outside it the band clamp holds the
edge with the bias current i_b of ``_native.bias_current``). The preset gains are kept.

:func:`mode_model`, :func:`closed_loop_poles_z`, :func:`spectral_radius` and
:func:`critical_tau_e` are reused by ``gain_margin``.
"""

from __future__ import annotations

import cmath
import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from .. import plotstyle
from ..ground import G0_MPS2
from .fail_direction import n_lift, stations
from .runner import CLASSES, StudyResult, StudySpec, check, class_cfg, class_style, finish, floats, new_figure, quick, stamp_meta

QUICK_AWARE = True

QUESTION = "Where are the open- and closed-loop levitation poles of each class and mode, and does the discrete design hold across the zero-power band?"

MODES: tuple[str, ...] = ("heave", "pitch", "roll")
_MODE_MARKERS = {"heave": "o", "pitch": "s", "roll": "^"}


# ---- design model (mirror of derive::{mode_model, zoh_plant, closed_loop}, PHYSICS §15) ----


@dataclass(frozen=True)
class ModeModel:
    """Per-element modal model: m_eff·ü = k_eff·u − k_i·i_mode."""

    m_eff_kg: float
    k_eff_n_per_m: float
    k_i_n_per_a: float

    @property
    def pole_rad_s(self) -> float:
        """Open-loop pole √(k_eff/m_eff); 0 for a passively stable mode."""
        if self.k_eff_n_per_m <= 0.0 or self.m_eff_kg <= 0.0:
            return 0.0
        return math.sqrt(self.k_eff_n_per_m / self.m_eff_kg)


def mode_model(p: Any, mode: str) -> ModeModel:
    """Modal model of ``mode`` at (g*, i = 0) with m* (derive::mode_model)."""
    d = p.derived()
    ks, ki = float(d["k_s_n_per_m"]), float(d["k_i_n_per_a"])
    m_star = float(p.get("design.carriage.m_star_kg"))
    if mode == "heave":
        return ModeModel(m_star / n_lift(p), ks, ki)
    lift = [s for s in stations(p) if s["normal_sign"] > 0]
    cg = [float(p.get(f"design.carriage.cg_m.{i}")) for i in range(3)]
    arms = [(s["x"] - cg[0]) if mode == "pitch" else (s["y"] - cg[1]) for s in lift]
    sum_r2 = sum(r * r for r in arms)
    if sum_r2 <= 0.0 or not lift:
        return ModeModel(0.0, 0.0, 0.0)
    h_cg = cg[2] - sum(s["z"] for s in lift) / len(lift)
    j = float(p.get("design.carriage.j_kgm2.1" if mode == "pitch" else "design.carriage.j_kgm2.0"))
    return ModeModel(j / sum_r2, (ks * sum_r2 + m_star * G0_MPS2 * h_cg) / sum_r2, ki)


def mode_active(p: Any, mode: str) -> bool:
    """Heave and pitch always; roll only with ``roll_active``; never a passively stable mode."""
    wanted = bool(p.get("control.levitation.roll_active")) if mode == "roll" else True
    return wanted and mode_model(p, mode).pole_rad_s > 0.0


def fast_period_s(p: Any) -> float:
    """Fast-group period T = fast_decim/base_hz, s."""
    return float(p.get("control.rates.fast_decim")) / float(p.get("control.rates.base_hz"))


def tau_cl_s(p: Any) -> float:
    """Current-loop time constant 1/(2π·current_bw_hz), s."""
    return 1.0 / (2.0 * math.pi * float(p.get("control.levitation.current_bw_hz")))


def omega_rad_s(p: Any, mode: str) -> float:
    """Design natural frequency ω = omega_ratio·max(p_mode, p_heave), rad/s."""
    return float(p.get("control.levitation.omega_ratio")) * max(mode_model(p, mode).pole_rad_s, mode_model(p, "heave").pole_rad_s)


def target_poles_s(p: Any, mode: str) -> np.ndarray:
    """Continuous targets (s² + 2ζωs + ω²)(s + αω): the pair, then αω."""
    w = omega_rad_s(p, mode)
    z = float(p.get("control.levitation.zeta"))
    a = float(p.get("control.levitation.alpha"))
    if z < 1.0:
        wd = w * math.sqrt(1.0 - z * z)
        pair = [complex(-z * w, wd), complex(-z * w, -wd)]
    else:
        r = w * math.sqrt(z * z - 1.0)
        pair = [complex(-z * w + r, 0.0), complex(-z * w - r, 0.0)]
    return np.array([*pair, complex(-a * w, 0.0)])


def gains(p: Any, mode: str) -> tuple[float, float, float]:
    """The native discrete zero-power gains (k_p, k_d, κ) of ``mode``."""
    from .. import require_native

    g = require_native().zp_gains(p, mode, True)
    return float(g["kp_a_per_m"]), float(g["kd_a_s_per_m"]), float(g["kappa_per_s"])


def _zoh(mm: ModeModel, t: float, tau_cl: float, tau_e: float) -> tuple[np.ndarray, np.ndarray]:
    from scipy.linalg import expm

    p2 = mm.k_eff_n_per_m / mm.m_eff_kg
    b = mm.k_i_n_per_a / mm.m_eff_kg
    a = np.zeros((5, 5))
    a[0, 1] = 1.0
    a[1, 0] = p2
    a[2, 2] = -1.0 / tau_cl
    a[2, 4] = 1.0 / tau_cl
    if tau_e > 0.0:
        a[1, 3] = -b
        a[3, 2] = 1.0 / tau_e
        a[3, 3] = -1.0 / tau_e
    else:
        a[1, 2] = -b
        a[3, 3] = -1.0 / tau_cl
    e = expm(a * t)
    return e[:4, :4], e[:4, 4]


def closed_loop_poles_z(p: Any, mode: str, k: tuple[float, float, float], tau_e_s: float, mm: ModeModel | None = None) -> np.ndarray:
    """The six closed-loop z-plane poles with gains ``k`` and plant eddy lag ``tau_e_s`` (derive::zp_closed_loop_poles)."""
    mm = mm or mode_model(p, mode)
    t = fast_period_s(p)
    phi, gam = _zoh(mm, t, tau_cl_s(p), max(float(tau_e_s), 0.0))
    m = np.zeros((6, 6))
    m[:4, :4] = phi
    m[:4, 5] = gam
    m[4, 2] = t
    m[4, 4] = 1.0
    m[5, 0], m[5, 1], m[5, 4] = k
    return np.linalg.eigvals(m)


def spectral_radius(p: Any, mode: str, k: tuple[float, float, float], tau_e_s: float, mm: ModeModel | None = None) -> float:
    """max |z| of :func:`closed_loop_poles_z` (stable iff < 1)."""
    return float(np.max(np.abs(closed_loop_poles_z(p, mode, k, tau_e_s, mm))))


def decay_rate_per_s(p: Any, mode: str, k: tuple[float, float, float], tau_e_s: float, mm: ModeModel | None = None) -> float:
    """Decay rate of the least-stable closed-loop pole −ln(ρ)/T, 1/s (negative = unstable)."""
    return -math.log(spectral_radius(p, mode, k, tau_e_s, mm)) / fast_period_s(p)


def critical_tau_e(p: Any, mode: str, hi_s: float = 0.05, tol_s: float = 1e-6) -> float:
    """Smallest plant eddy lag (bisection to ``tol_s`` over [0, hi_s]) at which the preset loop is unstable (derive::zp_critical_tau_e)."""
    if not mode_active(p, mode):
        return math.inf
    k = gains(p, mode)
    mm = mode_model(p, mode)
    lo, hi = 0.0, hi_s
    if spectral_radius(p, mode, k, lo, mm) >= 1.0:
        return 0.0
    if spectral_radius(p, mode, k, hi, mm) < 1.0:
        return math.inf
    while hi - lo > tol_s:
        mid = 0.5 * (lo + hi)
        if spectral_radius(p, mode, k, mid, mm) >= 1.0:
            hi = mid
        else:
            lo = mid
    return lo


def s_of_z(z: np.ndarray, t: float) -> np.ndarray:
    """s = ln(z)/T for the finite poles (z = 0, the pure delay, maps to −∞ and is dropped)."""
    zs = [complex(v) for v in z if abs(v) > 1e-12]
    return np.array([cmath.log(v) / t for v in zs])


def _match(poles: np.ndarray, targets: np.ndarray) -> np.ndarray:
    """The closed-loop poles nearest each target (greedy, each pole used once), in target order."""
    left = list(poles)
    out = []
    for tg in targets:
        j = int(np.argmin([abs(v - tg) for v in left]))
        out.append(left.pop(j))
    return np.array(out)


def heave_model_at(p: Any, m_kg: float) -> tuple[ModeModel, float, float]:
    """Heave model of mass m at its zero-power gap (band-clamped, with the clamp's bias current).

    Returns (model, gap, bias current per element).
    """
    from .. import require_native

    n = require_native()
    d = p.derived()
    g_star = float(p.get("design.suspension.g_star_m"))
    hp = float(d["h_prime_m"])
    m_star = float(p.get("design.carriage.m_star_kg"))
    lo, hi = float(p.get("design.suspension.zp_band_m.0")), float(p.get("design.suspension.zp_band_m.1"))
    g = min(max(n.zero_power_gap(m_kg, g_star, hp, m_star), lo), hi)
    i_b = float(n.bias_current(p, g, m_kg)) if not (lo < n.zero_power_gap(m_kg, g_star, hp, m_star) < hi) else 0.0
    mmf = float(d["m_pm_at"]) + float(p.get("design.suspension.turns")) * i_b
    m_eff = m_kg / n_lift(p)
    p2 = 4.0 * G0_MPS2 / (2.0 * g + hp)
    b = 2.0 * float(p.get("design.suspension.turns")) * G0_MPS2 / mmf
    return ModeModel(m_eff, m_eff * p2, m_eff * b), g, i_b


# ---- study ---------------------------------------------------------------------------------


def run(cfg: Any) -> StudyResult:
    from .. import require_native

    n = require_native()
    q = quick()
    n_band = 9 if q else 41
    fig, axes = new_figure(1, 2, size=(15.0, 6.8), gridspec_kw={"width_ratios": [1.2, 1.0]})
    ax_s, ax_b = axes[0]
    rec: dict[str, Any] = {}
    checks: dict[str, Any] = {}
    r_max = 0.0
    for cls in CLASSES:
        p = class_cfg(cfg, cls).params()
        d = p.derived()
        st = class_style(cls)
        t = fast_period_s(p)
        tau_nom = float(p.get("control.levitation.tau_e_nominal_s"))
        crec: dict[str, Any] = {"fast_period_us": t * 1e6, "tau_cl_ms": tau_cl_s(p) * 1e3, "tau_e_nominal_ms": tau_nom * 1e3, "modes": {}}
        native_pole = {"heave": d["pole_rad_s"], "pitch": d["pole_pitch_rad_s"], "roll": d["pole_roll_rad_s"]}
        for mode in MODES:
            mm = mode_model(p, mode)
            mrec: dict[str, Any] = {
                "open_loop_pole_rad_s": mm.pole_rad_s,
                "open_loop_pole_native_rad_s": float(native_pole[mode]),
                "m_eff_kg": mm.m_eff_kg,
                "k_eff_n_per_m": mm.k_eff_n_per_m,
            }
            if not mode_active(p, mode):
                mrec["active"] = False
                mrec["reason"] = (
                    "gravity-stabilised (CoM below the elements): no unstable pole, no zero-power loop (FAB-LEV-001)"
                    if mm.pole_rad_s == 0.0
                    else "roll_active = false"
                )
                crec["modes"][mode] = mrec
                continue
            k = gains(p, mode)
            cont = n.zp_gains(p, mode, False)
            z = closed_loop_poles_z(p, mode, k, tau_nom, mm)
            s_all = s_of_z(z, t)
            targets = target_poles_s(p, mode)
            dom = _match(s_all, targets)
            err = max(abs(sd - tg) / abs(tg) for sd, tg in zip(dom, targets, strict=True))
            zeta_dom = float(np.min(-dom.real / np.abs(dom)))
            zeta_all = float(np.min(-s_all.real / np.abs(s_all)))
            pole = mm.pole_rad_s
            mrec.update(
                {
                    "active": True,
                    "omega_rad_s": omega_rad_s(p, mode),
                    "gains_discrete": {"kp_a_per_m": k[0], "kd_a_s_per_m": k[1], "kappa_per_s": k[2]},
                    "gains_continuous": {key: float(v) for key, v in cont.items()},
                    "poles_z_re": floats(z.real),
                    "poles_z_im": floats(z.imag),
                    "poles_s_re": floats(s_all.real),
                    "poles_s_im": floats(s_all.imag),
                    "design_s_re": floats(dom.real),
                    "design_s_im": floats(dom.imag),
                    "target_s_re": floats(targets.real),
                    "target_s_im": floats(targets.imag),
                    "target_rel_err_max": float(err),
                    "zeta_design_min": zeta_dom,
                    "zeta_all_min": zeta_all,
                    "spectral_radius": float(np.max(np.abs(z))),
                    "open_loop_s": [pole, -pole],
                    "open_loop_s_im": [0.0, 0.0],
                }
            )
            crec["modes"][mode] = mrec
            checks[f"{cls}.{mode}.targets_within_2pct"] = check(
                err <= 0.02, float(err), 0.02, "FAB-LEV-007 (design poles vs the ω = 2.5·p targets)"
            )
            checks[f"{cls}.{mode}.zeta_ge_0p5"] = check(zeta_dom >= 0.5, zeta_dom, 0.5, "FAB-LEV-007 (closed-loop ζ of the design poles)")
            checks[f"{cls}.{mode}.open_loop_pole_mirror"] = check(
                abs(pole - native_pole[mode]) <= 1e-9 * max(pole, 1.0), pole, float(native_pole[mode]), "derive::mode_pole (mirror)"
            )
            ax_s.plot([pole, -pole], [0.0, 0.0], ls="none", marker="x", ms=8, mew=1.8, color=st["color"])
            ax_s.plot(
                dom.real,
                dom.imag,
                ls="none",
                marker=_MODE_MARKERS[mode],
                ms=8.5,
                color=st["color"],
                mec=plotstyle.COLORS["surface"],
                mew=1.2,
            )
            r_max = max(r_max, float(np.max(np.abs(dom))), pole)
        # Zero-power band sweep (heave, preset gains, nominal τ_e).
        m_star = float(p.get("design.carriage.m_star_kg"))
        g_star = float(p.get("design.suspension.g_star_m"))
        hp = float(d["h_prime_m"])
        lo, hi = float(p.get("design.suspension.zp_band_m.0")), float(p.get("design.suspension.zp_band_m.1"))
        m_band = (n.zero_power_mass(hi, g_star, hp, m_star), n.zero_power_mass(lo, g_star, hp, m_star))
        m_op = (float(p.get("design.carriage.m_op_min_kg")), float(p.get("design.carriage.m_rated_kg")))
        m_lo, m_hi = min(m_band[0], m_op[0]), max(m_band[1], m_op[1])
        masses = np.linspace(m_lo, m_hi, n_band)
        k_h = gains(p, "heave")
        decay, gaps, ib = [], [], []
        for m in masses:
            mm_b, g_b, i_b = heave_model_at(p, float(m))
            decay.append(decay_rate_per_s(p, "heave", k_h, tau_nom, mm_b))
            gaps.append(g_b * 1e3)
            ib.append(i_b)
        x = masses / m_star
        ax_b.plot(x, decay, color=st["color"], lw=1.8, label=st["label"])
        op_x = [m_op[0] / m_star, m_op[1] / m_star]
        op_y = [decay_rate_per_s(p, "heave", k_h, tau_nom, heave_model_at(p, m)[0]) for m in m_op]
        ax_b.plot(op_x, op_y, ls="none", marker=st["marker"], ms=8, color=st["color"], mec=plotstyle.COLORS["surface"], mew=1.2)
        crec["band"] = {
            "m_kg": floats(masses),
            "m_over_m_star": floats(x),
            "gap_mm": gaps,
            "bias_current_a": ib,
            "decay_rate_per_s": decay,
            "band_mass_kg": list(m_band),
            "op_m_over_m_star": op_x,
            "op_decay_rate_per_s": op_y,
        }
        checks[f"{cls}.heave_stable_across_band"] = check(
            min(decay) > 0.0, float(min(decay)), 0.0, "FAB-LEV-007 (stable across the zero-power float band; heave)"
        )
        rec[cls] = crec

    # ζ = 0.5 rays and the stability boundary.
    ray = 1.05 * r_max
    zeta_ref = {"x": [0.0, -0.5 * ray], "y_up": [0.0, ray * math.sqrt(0.75)], "y_down": [0.0, -ray * math.sqrt(0.75)]}
    ax_s.plot(zeta_ref["x"], zeta_ref["y_up"], color=plotstyle.COLORS["muted"], ls=(0, (4, 4)), lw=1.1)
    ax_s.plot(zeta_ref["x"], zeta_ref["y_down"], color=plotstyle.COLORS["muted"], ls=(0, (4, 4)), lw=1.1)
    ax_s.axvline(0.0, color=plotstyle.COLORS["axis"], lw=1.0)
    plotstyle.zero_line(ax_s)
    ax_s.annotate("ζ = 0.5", xy=(-0.47 * ray, 0.83 * ray), fontsize=9, color=plotstyle.COLORS["muted"])
    ax_s.set_xlim(-1.12 * ray, 0.45 * ray)
    ax_s.set_ylim(-1.0 * ray, 1.0 * ray)
    plotstyle.style_ax(ax_s, "Re s (rad/s)", "Im s (rad/s)", "(a) s-plane at the nominal eddy lag")
    from matplotlib.lines import Line2D

    handles = [Line2D([], [], color=plotstyle.CLASS_COLORS[c], lw=6, label=plotstyle.CLASS_LABELS[c]) for c in CLASSES]
    handles += [Line2D([], [], color=plotstyle.COLORS["ink2"], ls="none", marker="x", ms=8, label="open loop ±p")]
    handles += [
        Line2D([], [], color=plotstyle.COLORS["ink2"], ls="none", marker=_MODE_MARKERS[m], ms=8, label=f"closed loop, {m}") for m in MODES
    ]
    ax_s.legend(handles=handles, loc="upper left", bbox_to_anchor=(1.0, 1.0), **plotstyle.LEGEND_KW)
    plotstyle.zero_line(ax_b)
    plotstyle.style_ax(ax_b, "carriage mass / m*", "decay rate of the slowest pole  −ln ρ / T (1/s)", "(b) Heave across the float band")
    ax_b.legend(loc="lower right", **plotstyle.LEGEND_KW)

    worst_zeta = min(m["zeta_design_min"] for c in rec.values() for m in c["modes"].values() if m.get("active"))
    worst_err = max(m["target_rel_err_max"] for c in rec.values() for m in c["modes"].values() if m.get("active"))
    worst_zeta_all = min(m["zeta_all_min"] for c in rec.values() for m in c["modes"].values() if m.get("active"))
    passive = [f"{c.upper()} {m}" for c, cr in rec.items() for m, mr in cr["modes"].items() if not mr.get("active")]
    min_decay = min(min(c["band"]["decay_rate_per_s"]) for c in rec.values())
    caption = (
        "(a) Open-loop poles ±p (×) and the three design closed-loop poles (markers by mode, colour by class) of the six-state "
        "discrete design model (ZOH plant, current-loop lag τ_cl, nominal eddy lag, Mizuno integral, one-period LET delay) at "
        f"the preset discrete gains; s = ln(z)/T. Worst design-pole ζ = {worst_zeta:.3f} (LEV-007 needs ≥ 0.5); they sit "
        f"within {worst_err:.1e} (relative) of the continuous targets at ω = 2.5·max(p_mode, p_heave). The current-loop, eddy and "
        f"delay poles (the DRAWER roll eddy pole sits near −270 rad/s) are in the record; worst ζ over all six poles "
        f"{worst_zeta_all:.3f}. Passive modes (no loop): {', '.join(passive)}. WALL and HEAVY share the heave design "
        f"(same g*, h′, flux and ω), so their heave poles and band curves coincide; "
        f"ω = 2.5·max(p_mode, p_heave) also puts every pitch pair with p_pitch < p_heave on its heave pair.\n"
        "(b) Heave decay rate of the least-stable pole as the carriage mass moves across the zero-power band (inside the band "
        "the gap follows m(g) at zero current; outside it the band clamp holds the edge with the bias current). Markers: "
        f"m_op,min and m_rated. Slowest decay {min_decay:.1f} 1/s (> 0: stable). Design model only: the implemented loops add "
        "a state observer (crates/fabrail-core/tests/levitation_linear.rs checks them); pitch and roll across the band are "
        "not swept here."
    )
    title = "pole_map: levitation poles per class and mode"
    stamp = finish(fig, title=title, question=QUESTION, caption=caption, meta=stamp_meta(cfg, "pole_map"), wspace=0.62)
    record = {
        "quick": q,
        "title": title,
        "caption": caption,
        "run_stamp": stamp,
        "classes": rec,
        "zeta_ref_line": zeta_ref,
        "stability_boundary_per_s": 0.0,
        "checks": checks,
    }
    return StudyResult(fig=fig, record=record)


STUDY = StudySpec(name="pole_map", tier=1, question=QUESTION, run=run)
