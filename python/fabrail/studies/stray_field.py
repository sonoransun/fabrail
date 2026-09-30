"""Study ``stray_field`` (tier 1): accessible static field vs the 0.5 mT target (FAB-ENV-001).

Question: how strong is the permanent-magnet field at the distances a person can reach,
per class, against the ≤ 0.5 mT at 10 cm target, and what housing standoff or shroud
does each class need?

Model (PHYSICS §23; ``fabrail_plant::stray``, called through ``_native.stray_field``):
every PM block is a point dipole of moment B_r·V/μ₀ at its block centre, and the field is
the dipole sum over all blocks of the class preset. The rail and pole iron, which close
most of the flux, are not modelled, so these are **upper-bound-style bare-magnet
estimates**, and coil fields are omitted (zero-power control holds i ≈ 0). Nothing here
is measured; ENV-001 is status S.

Distances are measured from the centre of the first PM block (station 0):

- ``below``: along −z (the magnetisation axis, where a dipole's field is largest);
- ``beside``: along +y from the outermost block, in the magnet plane.

The 10 cm target applies from an *accessible surface*, so the study reports the distance
at which the field falls to 0.5 mT and the housing standoff that implies
(standoff = d(0.5 mT) − 10 cm), and the shielding factor a shroud needs
(B(10 cm)/0.5 mT) where no standoff is practical.

**Leakage sensitivity.** With a flux-leakage coefficient σ the PM must carry (1 + σ)
times the gap flux, so a magnet resized for the same gap flux has (1 + σ) times the
volume and moment; the dipole field scales by (1 + σ). σ = 0 is the preset ("ideal");
σ = 0.25 is the value the core design uses for sensitivity studies (core design §C.5,
``design.suspension.leak``) [UNCERTAIN as a stray-field proxy]. The native model itself
does not read σ.

Owner: B3 (Wave 2).
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any

import numpy as np

from ..presets import CLASSES
from .runner import StudyResult, StudySpec

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..config import RunConfig

__all__ = ["LEAK_SIGMAS", "STUDY", "TARGET_T", "TARGET_DISTANCE_M", "class_field", "compute", "run"]

#: ENV-001 target: ≤ 0.5 mT at 10 cm from an accessible surface (ICNIRP 2009 precaution
#: level for implant wearers; SPECIFICATION §5 [R17]). The per-class copy is
#: ``design.suspension.stray_target_t``.
TARGET_T = 0.5e-3
TARGET_DISTANCE_M = 0.10
#: Leakage coefficients for the sensitivity curves (core design §C.5: 0 ideal, 0.25 studies).
LEAK_SIGMAS = (0.0, 0.25)
#: μ₀, H/m (CODATA 2018), for the single-magnet reference value only.
_MU0 = 1.25663706212e-6
#: Distance grid from the block centre, m.
_D_GRID_M = np.round(np.geomspace(0.02, 1.0, 120), 9)


def _blocks(p: Any) -> list[dict[str, float]]:
    """PM block centres and moments of preset ``p`` (the plant's ``stray::magnets`` rule)."""
    s = p.get
    n = int(s("design.suspension.n_magnets"))
    h = float(s("design.suspension.h_m_m"))
    vol = h * float(s("design.suspension.pole_w_m")) * float(s("design.suspension.pole_l_m"))
    br = float(s("design.suspension.b_r_t"))
    out = []
    for i in range(n):
        pre = f"design.suspension.stations.{i}."
        if not bool(s(pre + "has_pm")) or float(s(pre + "normal_sign")) == 0.0:
            continue
        sign = 1.0 if float(s(pre + "normal_sign")) > 0.0 else -1.0
        out.append(
            {
                "x_m": float(s(pre + "x_m")),
                "y_m": float(s(pre + "y_m")),
                "z_m": float(s(pre + "z_m")) - sign * 0.5 * h,
                "moment_a_m2": br * vol / _MU0,
            }
        )
    return out


def _field_mag_t(p: Any, pts: np.ndarray) -> np.ndarray:
    from .. import require_native

    raw = require_native().stray_field(p, np.ascontiguousarray(pts, dtype="<f8").tobytes())
    return np.linalg.norm(np.frombuffer(raw, dtype="<f8").reshape(-1, 3), axis=1)


def _distance_for(d: np.ndarray, b: np.ndarray, target: float) -> float:
    """Smallest grid-interpolated distance beyond which |B| stays ≤ ``target`` (log-log), m."""
    above = np.nonzero(b > target)[0]
    if above.size == 0:
        return float(d[0])
    i = int(above[-1])
    if i + 1 >= d.shape[0]:
        return math.inf
    x0, x1 = math.log(d[i]), math.log(d[i + 1])
    y0, y1 = math.log(b[i]), math.log(b[i + 1])
    return math.exp(x0 + (math.log(target) - y0) * (x1 - x0) / (y1 - y0))


def class_field(rail_class: str) -> dict[str, Any]:
    """Field vs distance below and beside station 0 of ``rail_class`` (preset), and the derived numbers."""
    from .. import require_native

    p = require_native().Params.preset(rail_class)
    blocks = _blocks(p)
    b0 = blocks[0]
    y_out = max(abs(b["y_m"]) for b in blocks)
    y_side = math.copysign(y_out, b0["y_m"] if b0["y_m"] != 0.0 else 1.0)
    below = np.column_stack([np.full_like(_D_GRID_M, b0["x_m"]), np.full_like(_D_GRID_M, b0["y_m"]), b0["z_m"] - _D_GRID_M])
    beside = np.column_stack(
        [np.full_like(_D_GRID_M, b0["x_m"]), y_side + math.copysign(1.0, y_side) * _D_GRID_M, np.full_like(_D_GRID_M, b0["z_m"])]
    )
    out: dict[str, Any] = {
        "n_blocks": len(blocks),
        "moment_a_m2": b0["moment_a_m2"],
        "shroud_flag_preset": bool(p.get("design.suspension.shroud")),
        "target_t_preset": float(p.get("design.suspension.stray_target_t")),
        # Single block on its axis at 10 cm: (μ₀/4π)·2m/r³ (the ENV-001 reference values).
        "single_block_on_axis_10cm_mt": 1e3 * _MU0 / (4.0 * math.pi) * 2.0 * b0["moment_a_m2"] / TARGET_DISTANCE_M**3,
    }
    for name, pts in (("below", below), ("beside", beside)):
        b = _field_mag_t(p, pts)
        u = _unit(pts)
        at10 = float(_field_mag_t(p, pts[:1] + u * (TARGET_DISTANCE_M - float(_D_GRID_M[0])))[0])
        per_sigma: dict[str, Any] = {}
        for sigma in LEAK_SIGMAS:
            scale = 1.0 + sigma
            d_target = _distance_for(_D_GRID_M, b * scale, TARGET_T)
            per_sigma[f"sigma_{sigma:g}"] = {
                "b_mt": 1e3 * b * scale,
                "b_at_10cm_mt": 1e3 * at10 * scale,
                "d_at_target_m": d_target,
                "standoff_needed_m": max(0.0, d_target - TARGET_DISTANCE_M),
                "shield_factor_at_10cm": at10 * scale / TARGET_T,
                "meets_target_at_10cm": bool(at10 * scale <= TARGET_T),
            }
        out[name] = per_sigma
    worst = max(out["below"]["sigma_0"]["b_at_10cm_mt"], out["beside"]["sigma_0"]["b_at_10cm_mt"])
    out["worst_at_10cm_mt"] = worst
    out["shroud_required"] = bool(worst > 1e3 * TARGET_T * 10.0)
    return out


def _unit(pts: np.ndarray) -> np.ndarray:
    """Unit direction of the sampling ray (first two grid points)."""
    d = pts[1] - pts[0]
    return (d / np.linalg.norm(d))[None, :]


def compute() -> dict[str, Any]:
    """Every number of the study (the JSON record)."""
    classes = {c: class_field(c) for c in CLASSES}
    return {
        "inputs": {
            "target_t": TARGET_T,
            "target_mt": TARGET_T * 1e3,
            "target_distance_m": TARGET_DISTANCE_M,
            "leak_sigmas": list(LEAK_SIGMAS),
            "model": "point-dipole sum of bare PM blocks (fabrail_plant::stray::field); rail/pole iron and coil "
            "fields not modelled; upper-bound-style estimate, not a measurement",
            "leakage": "moment x (1 + sigma): the PM resized to carry the same gap flux under leakage sigma "
            "[UNCERTAIN as a stray-field proxy]",
            "geometry": "distances from the centre of PM block 0; 'below' along -z, 'beside' along +y from the "
            "outermost block",
            "shroud_rule": "shroud required when the unshielded field at 10 cm exceeds 10x the target "
            "(no practical housing standoff)",
        },
        "distance_m": _D_GRID_M,
        "classes": classes,
    }


def _figure(rec: dict[str, Any]) -> Any:
    from matplotlib.figure import Figure

    from .. import plotstyle as ps

    ps.apply()
    c = ps.COLORS
    fig = Figure(figsize=(12.5, 4.5))
    axes = fig.subplots(1, 3, gridspec_kw={"width_ratios": [1.2, 1.2, 0.9]})
    d = rec["distance_m"]
    for ax, where, title in ((axes[0], "below", "Below the magnet (axis)"), (axes[1], "beside", "Beside the carriage")):
        for cls in CLASSES:
            col = ps.CLASS_COLORS[cls]
            data = rec["classes"][cls][where]
            ax.plot(d, data["sigma_0"]["b_mt"], color=col, lw=1.7, label=ps.CLASS_LABELS[cls])
            ax.plot(d, data["sigma_0.25"]["b_mt"], color=col, lw=1.0, ls=(0, (3, 2)))
        ps.ref_line(ax, rec["inputs"]["target_mt"], "0.5 mT target", color=c["star"])
        ax.axvline(rec["inputs"]["target_distance_m"], color=c["muted"], lw=0.9, ls=":")
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlim(0.02, 1.0)
        ps.style_ax(ax, "distance from magnet centre (m)", "|B| (mT)", title)
    axes[0].legend(loc="lower left", **ps.LEGEND_KW)
    axes[1].annotate("dashed: leakage sigma 0.25", xy=(0.98, 0.95), xycoords="axes fraction", ha="right", fontsize=8.5,
                     color=c["muted"])
    ax3 = axes[2]
    factors = [rec["classes"][cls]["below"]["sigma_0"]["shield_factor_at_10cm"] for cls in CLASSES]
    ax3.bar(range(len(CLASSES)), factors, color=[ps.CLASS_COLORS[k] for k in CLASSES], width=0.6)
    ax3.set_yscale("log")
    ax3.set_xticks(range(len(CLASSES)), [ps.CLASS_LABELS[k] for k in CLASSES])
    ps.ref_line(ax3, 1.0, "meets target", color=c["good"])
    for i, cls in enumerate(CLASSES):
        info = rec["classes"][cls]
        tag = "shroud" if info["shroud_required"] else f"standoff {info['below']['sigma_0']['standoff_needed_m'] * 100:.0f} cm"
        ax3.annotate(tag, xy=(i, factors[i]), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=8,
                     color=c["ink2"])
    ps.style_ax(ax3, None, "B(10 cm) / 0.5 mT", "Shielding needed at 10 cm")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    ps.brand(fig, "stray_field · bare-magnet dipole sum (PHYSICS §23) · upper-bound-style estimate, not measured")
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the study (class presets; ``cfg`` overrides are not applied: the presets are the subject)."""
    del cfg
    rec = compute()
    return StudyResult(fig=_figure(rec), record=rec)


STUDY = StudySpec(
    name="stray_field",
    tier=1,
    question="What static field reaches an accessible surface per class vs the 0.5 mT at 10 cm target, "
    "and what standoff or shroud does each need?",
    run=run,
)
