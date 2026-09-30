"""Study ``amd_reality`` (tier 2, research tier): does a maglev panel used as a mass damper help the building?

Question (SEISMIC.md §3; FAB-SEI-016/021 context): how much does a mass damper reduce a
building's seismic response as a function of the mass ratio μ = m_damper/M_building,
and what stroke, force and power does it need, for (i) an ideal passive TMD of that
mass, (ii) the rail panel as an LSM-driven damper emulating the same TMD within the
LSM's force and stroke limits, and (iii) the core's direct velocity feedback (DVF) law
within the same limits? Mass ratios below 0.5 % are shaded "product-irrelevant".

**Model (SENTINEL-style 2-DOF, numpy).** Host: one storey of M = 1e7 kg at T1 = 1.13 s,
ζ_s = 2 % (Kyobashi-like; the SEI-021 host of ``scenarios.harmonic_base``). Damper: a
mass m_d = μ·M moving relative to the storey (x_r) under the damper force F (reaction
−F on the host); base acceleration a_g(t):

    M·ẍ_s = −c_s·ẋ_s − k_s·x_s − M·a_g − F,   m_d·(ẍ_s + ẍ_r) = −m_d·a_g + F.

- ``tmd_ideal``: a real spring and dashpot, F = −k_d·x_r − c_d·ẋ_r, Warburton
  base-excitation optimum (f, ζ) = ``analytic.warburton(μ)`` (0.97548 / 0.07019 at
  μ = 0.02, the SEI-016 anchor); no force or stroke limit (the stroke it needs is
  reported). This is what the mass ratio alone can buy.
- ``lsm_tmd``: a levitated panel has no spring, so the LSM must supply the whole tuned
  force: the same law saturated at F_max = a_lsm·m_d, with the stroke limited to
  ±s_max by end stops (stiff Hunt–Crossley-like springs, 10 Hz on the damper mass, ζ 0.3).
- ``lsm_dvf``: the core AMD law (``fabrail_core::seismic::amd``): F on the damper
  = +c_dvf·ẋ_s (the host feels −c_dvf·ẋ_s), c_dvf sized to add 5 % host damping if it
  never saturated, saturated at F_max, same end stops.
- ``free_panel``: the LSM off: a levitated panel free between the same end stops. It
  separates what the limited variants get from the LSM from what they get from the
  panel striking its stops (an impact load on the rail, not a damping feature).

LSM limits from the HEAVY preset (the 5 t panel): a_lsm = ``control.seismic.amd.f_max_n``
/ m_rated = 0.1 m/s² and s_max = ``safety.isolate.stroke_m`` = 0.2 m (the SAF-020 envelope;
there is no separate Damp envelope). A 5 t panel in a 1e7 kg building is μ = 0.05 %;
Kyobashi Seiwa (1989) used about 1 %.

**Excitation.** N_REC Kanai–Tajimi × Clough–Penzien records (medium soil) at PGA 0.3 g,
40 s at 200 Hz (``ground.synthesize``), member seeds from ``SeedSequence(seed)``,
integrated at 1 ms (symplectic Euler, record linearly interpolated, controller force
updated every step). Reported per μ and variant: the median over records of the RMS
and peak host-drift reduction against the same host without a damper, the damper
stroke (p95 of the per-record peak), the actuator force and mechanical power
(peak |F·ẋ_r| and mean |F·ẋ_r|), and the stroke-limited added-damping bound
ζ_add ≤ (μ/2)·(S/X) (``_native.exo_zeta_add_bound``; EXOTIC §2) with S = s_max and X the
median uncontrolled peak drift.

Research tier: mass damping by a rail panel is not a building-protection feature, and
mass dampers are mainly wind devices (plan §7; SEISMIC.md §3).

Owner: B3 (Wave 2).
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from .. import ground
from .runner import StudyResult, StudySpec

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..config import RunConfig

__all__ = ["HOST_MASS_KG", "HOST_T1_S", "HOST_ZETA", "MU_GRID", "PRODUCT_IRRELEVANT_MU", "STUDY", "VARIANTS",
           "compute", "run", "simulate"]

HOST_MASS_KG = 1.0e7
HOST_T1_S = 1.13
HOST_ZETA = 0.02
#: Mass ratios: log grid plus the HEAVY panel (5 t / 1e7 kg) and Kyobashi Seiwa (~1 %).
MU_HEAVY = 5.0e-4
MU_KYOBASHI = 1.0e-2
MU_GRID = tuple(sorted({*np.round(np.geomspace(1e-4, 5e-2, 13), 8).tolist(), MU_HEAVY, MU_KYOBASHI}))
#: Below this mass ratio the effect is shaded "product-irrelevant" (core design §C.6).
PRODUCT_IRRELEVANT_MU = 5.0e-3
VARIANTS = ("tmd_ideal", "lsm_tmd", "lsm_dvf", "free_panel")
N_REC = 20
PGA_G = 0.3
_REC_S = 40.0
_FS_HZ = 200.0
_DT_S = 1e-3
_DVF_ZETA_ADD = 0.05
_STOP_HZ = 10.0
_STOP_ZETA = 0.3


def _lsm_limits() -> tuple[float, float, float]:
    """(a_lsm m/s², s_max m, panel mass kg) from the HEAVY preset."""
    from .. import require_native

    p = require_native().Params.preset("heavy")
    m = float(p.get("design.carriage.m_rated_kg"))
    return float(p.get("control.seismic.amd.f_max_n")) / m, float(p.get("safety.isolate.stroke_m")), m


def simulate(records: np.ndarray, mus: np.ndarray, variant: str, a_lsm: float, s_max: float) -> dict[str, np.ndarray]:
    """Host + damper response for every (μ, record) pair; ``mus`` of shape (k,), records (r, n) at 200 Hz.

    ``variant`` "none" gives the host alone. Returns arrays of shape (k, r): peak and RMS host
    drift, peak damper stroke, peak |F|, peak and mean |F·ẋ_r|.
    """
    from .. import analytic

    a_g = np.asarray(records, dtype=np.float64)
    k_mu, n_rec = len(mus), a_g.shape[0]
    mu = np.repeat(np.asarray(mus, dtype=np.float64), n_rec)
    ag_rows = np.tile(a_g, (k_mu, 1))
    w_s = 2.0 * math.pi / HOST_T1_S
    M = HOST_MASS_KG
    k_s, c_s = M * w_s * w_s, 2.0 * HOST_ZETA * M * w_s
    m_d = mu * M
    tun = np.array([analytic.warburton(float(u)) for u in mus])
    f_opt = np.repeat(tun[:, 0], n_rec)
    z_opt = np.repeat(tun[:, 1], n_rec)
    k_d = m_d * (f_opt * w_s) ** 2
    c_d = 2.0 * z_opt * m_d * f_opt * w_s
    c_dvf = 2.0 * _DVF_ZETA_ADD * M * w_s
    f_max = a_lsm * m_d
    k_stop = m_d * (2.0 * math.pi * _STOP_HZ) ** 2
    c_stop = 2.0 * _STOP_ZETA * m_d * 2.0 * math.pi * _STOP_HZ
    up = int(round(1.0 / (_DT_S * _FS_HZ)))
    n_s = a_g.shape[1]
    n_steps = (n_s - 1) * up
    rows = mu.shape[0]
    xs, vs, xr, vr = (np.zeros(rows) for _ in range(4))
    pk_x, sum_x2, pk_r, pk_f, pk_p, sum_p = (np.zeros(rows) for _ in range(6))
    pk_lsm, pk_stop = np.zeros(rows), np.zeros(rows)
    for i in range(n_steps):
        j, fr = divmod(i, up)
        ag = ag_rows[:, j] + (ag_rows[:, j + 1] - ag_rows[:, j]) * (fr / up)
        stop = np.zeros(rows)
        if variant == "none":
            f_act = np.zeros(rows)
            p_act = f_act
        elif variant == "tmd_ideal":
            f_act = -k_d * xr - c_d * vr
            p_act = c_d * vr * vr  # dissipated in the dashpot (the spring is reactive)
        else:
            if variant == "lsm_tmd":
                f_act = np.clip(-k_d * xr - c_d * vr, -f_max, f_max)
            elif variant == "lsm_dvf":
                f_act = np.clip(c_dvf * vs, -f_max, f_max)
            else:
                f_act = np.zeros(rows)
            p_act = np.abs(f_act * vr)
            over = np.abs(xr) - s_max
            stop = np.where(over > 0.0, -np.sign(xr) * np.maximum(k_stop * over + c_stop * np.sign(xr) * vr, 0.0), 0.0)
        f = f_act + stop
        a_s = (-c_s * vs - k_s * xs - f) / M - ag
        a_r = np.where(m_d > 0.0, f / np.maximum(m_d, 1e-30), 0.0) - ag - a_s
        vs = vs + a_s * _DT_S
        xs = xs + vs * _DT_S
        vr = vr + a_r * _DT_S
        xr = xr + vr * _DT_S
        pk_x = np.maximum(pk_x, np.abs(xs))
        sum_x2 += xs * xs
        pk_r = np.maximum(pk_r, np.abs(xr))
        pk_f = np.maximum(pk_f, np.abs(f))
        pk_lsm = np.maximum(pk_lsm, np.abs(f_act))
        pk_stop = np.maximum(pk_stop, np.abs(stop))
        pk_p = np.maximum(pk_p, p_act)
        sum_p += p_act
    shape = (k_mu, n_rec)
    return {
        "peak_drift_m": pk_x.reshape(shape),
        "rms_drift_m": np.sqrt(sum_x2 / n_steps).reshape(shape),
        "peak_stroke_m": pk_r.reshape(shape),
        "peak_force_n": pk_f.reshape(shape),
        "peak_actuator_force_n": pk_lsm.reshape(shape),
        "peak_stop_force_n": pk_stop.reshape(shape),
        "peak_power_w": pk_p.reshape(shape),
        "mean_power_w": (sum_p / n_steps).reshape(shape),
    }


def compute(cfg: RunConfig, n_rec: int = N_REC, mus: tuple[float, ...] = MU_GRID) -> dict[str, Any]:
    """Every number of the study (the JSON record)."""
    from .. import analytic, require_native
    from ..ensemble import member_seeds

    a_lsm, s_max, m_panel = _lsm_limits()
    t0 = time.perf_counter()
    recs = np.vstack([ground.synthesize(ms.rng(), _REC_S, _FS_HZ, soil="medium", pga_g=PGA_G, model="cp")
                      for ms in member_seeds(cfg.seed, n_rec)])
    mu_arr = np.asarray(mus, dtype=np.float64)
    base = simulate(recs, np.array([0.0]), "none", a_lsm, s_max)
    out: dict[str, Any] = {}
    for variant in VARIANTS:
        r = simulate(recs, mu_arr, variant, a_lsm, s_max)
        red_rms = 1.0 - r["rms_drift_m"] / base["rms_drift_m"]
        red_pk = 1.0 - r["peak_drift_m"] / base["peak_drift_m"]
        out[variant] = {
            "rms_reduction_p50": np.median(red_rms, axis=1),
            "peak_reduction_p50": np.median(red_pk, axis=1),
            "rms_reduction_min": np.min(red_rms, axis=1),
            "rms_reduction_max": np.max(red_rms, axis=1),
            "stroke_p95_m": np.percentile(r["peak_stroke_m"], 95, axis=1),
            "force_peak_n": np.max(r["peak_actuator_force_n"], axis=1),
            "stop_force_peak_n": np.max(r["peak_stop_force_n"], axis=1),
            "power_peak_w": np.max(r["peak_power_w"], axis=1),
            "power_mean_w": np.median(r["mean_power_w"], axis=1),
            "saturated": variant != "tmd_ideal",
        }
    t1 = time.perf_counter()
    x_unc = float(np.median(base["peak_drift_m"]))
    native = require_native()
    zeta_bound = np.array([native.exo_zeta_add_bound(float(u), s_max / x_unc) for u in mus])
    i_heavy = mus.index(MU_HEAVY) if MU_HEAVY in mus else None
    i_02 = int(np.argmin(np.abs(mu_arr - 0.02)))
    heavy: dict[str, Any] | None = None
    if i_heavy is not None:
        heavy = {v: {k: float(np.asarray(val)[i_heavy]) for k, val in out[v].items() if k != "saturated"}
                 for v in VARIANTS}
        heavy["zeta_add_bound"] = float(zeta_bound[i_heavy])
    wf, wz = analytic.warburton(0.02)
    return {
        "inputs": {
            "host_mass_kg": HOST_MASS_KG,
            "host_t1_s": HOST_T1_S,
            "host_zeta": HOST_ZETA,
            "records": {"n": n_rec, "model": "KT/CP medium soil", "pga_g": PGA_G, "record_s": _REC_S,
                        "seed": cfg.seed},
            "a_lsm_mps2": a_lsm,
            "s_max_m": s_max,
            "panel_mass_kg": m_panel,
            "mu_heavy": MU_HEAVY,
            "mu_kyobashi_seiwa": MU_KYOBASHI,
            "product_irrelevant_below_mu": PRODUCT_IRRELEVANT_MU,
            "dvf_zeta_add_unsaturated": _DVF_ZETA_ADD,
            "dt_s": _DT_S,
            "tuning": "Warburton base-excitation optimum (analytic.warburton)",
            "force": "actuator force: the spring + dashpot of the ideal TMD, the LSM force (<= F_max) of the "
            "limited variants; end-stop impact forces are reported separately",
            "power": "ideal TMD: dashpot dissipation c_d*v_r^2; limited variants: |F_lsm * v_r|",
            "reduction_reference_pct": 0.0,
            "end_stops": f"penalty springs at +-s_max, {_STOP_HZ:g} Hz on the damper mass, zeta {_STOP_ZETA:g} "
            "[UNCERTAIN study assumption; stop forces scale with it]",
            "tier": "research; mass dampers are mainly wind devices; not a building-protection feature",
        },
        "mu": mu_arr,
        "mu_pct": mu_arr * 100.0,
        "variants": out,
        "uncontrolled": {"peak_drift_p50_m": x_unc, "rms_drift_p50_m": float(np.median(base["rms_drift_m"]))},
        "zeta_add_bound": zeta_bound,
        "zeta_add_bound_pct": zeta_bound * 100.0,
        "host_zeta_pct": HOST_ZETA * 100.0,
        "heavy_panel": heavy,
        "anchors": {"warburton_mu_0p02": [wf, wz], "mu_nearest_0p02": float(mu_arr[i_02])},
        "timing": {"wall_s": {"compute_s": t1 - t0}},
    }


def _figure(rec: dict[str, Any]) -> Any:
    from matplotlib.figure import Figure

    from .. import plotstyle as ps

    ps.apply()
    c = ps.COLORS
    fig = Figure(figsize=(13.0, 8.6))
    (a1, a2), (a3, a4) = fig.subplots(2, 2)
    mu = rec["mu_pct"]
    labels = {"tmd_ideal": "ideal passive TMD (same mass, no stroke limit)",
              "lsm_tmd": "LSM-emulated TMD (force/stroke limited)", "lsm_dvf": "LSM DVF (force/stroke limited)",
              "free_panel": "LSM off, panel free between stops"}
    colors = {"tmd_ideal": c["s1"], "lsm_tmd": c["s2"], "lsm_dvf": c["s3"], "free_panel": c["ctrl"]}
    v = rec["variants"]
    rec["variants_pct"] = {k: {"rms_reduction_p50_pct": np.asarray(v[k]["rms_reduction_p50"]) * 100.0,
                               "rms_reduction_min_pct": np.asarray(v[k]["rms_reduction_min"]) * 100.0,
                               "rms_reduction_max_pct": np.asarray(v[k]["rms_reduction_max"]) * 100.0,
                               "force_peak_kn": np.asarray(v[k]["force_peak_n"]) * 1e-3,
                               "power_mean_kw": np.asarray(v[k]["power_mean_w"]) * 1e-3,
                               "stop_force_peak_kn": np.asarray(v[k]["stop_force_peak_n"]) * 1e-3}
                           for k in VARIANTS}
    vp = rec["variants_pct"]
    x_irr = rec["inputs"]["product_irrelevant_below_mu"] * 100.0
    for ax in (a1, a2, a3, a4):
        ax.axvspan(mu[0], x_irr, color=c["ctrl"], alpha=0.12, linewidth=0)
        ax.axvline(rec["inputs"]["mu_heavy"] * 100.0, color=c["star"], lw=0.9, ls=":")
        ax.axvline(rec["inputs"]["mu_kyobashi_seiwa"] * 100.0, color=c["muted"], lw=0.9, ls=":")
        ax.set_xscale("log")
    # Record spread (min-max over records) for the ideal TMD only, to keep the panel legible.
    a1.fill_between(mu, vp["tmd_ideal"]["rms_reduction_min_pct"], vp["tmd_ideal"]["rms_reduction_max_pct"],
                    color=colors["tmd_ideal"], alpha=0.10, linewidth=0)
    for k in VARIANTS:
        a1.plot(mu, vp[k]["rms_reduction_p50_pct"], color=colors[k], lw=1.7, marker="o", ms=3, label=labels[k])
    ps.zero_line(a1, rec["inputs"]["reduction_reference_pct"])
    a1.annotate("product-irrelevant\n(mu < 0.5 %)", xy=(0.012, 0.93), xycoords="axes fraction", fontsize=8.5,
                color=c["ink2"], va="top")
    a1.annotate("5 t panel\nmu 0.05 %", xy=(rec["inputs"]["mu_heavy"] * 100.0, 0.22), xycoords=("data", "axes fraction"),
                fontsize=8, color=c["star"], ha="right", xytext=(-3, 0), textcoords="offset points")
    a1.annotate("Kyobashi\nSeiwa ~1 %", xy=(rec["inputs"]["mu_kyobashi_seiwa"] * 100.0, 0.22),
                xycoords=("data", "axes fraction"), fontsize=8, color=c["ink2"], ha="right", xytext=(-3, 0),
                textcoords="offset points")
    a1.legend(loc="upper left", bbox_to_anchor=(0.0, 0.84), **ps.LEGEND_KW)
    ps.style_ax(a1, "mass ratio mu (%)", "RMS drift reduction (%)", "What the mass ratio buys (KT/CP 0.3 g)")
    for k in VARIANTS:
        a2.plot(mu, v[k]["stroke_p95_m"], color=colors[k], lw=1.6, marker="o", ms=3)
    ps.ref_line(a2, rec["inputs"]["s_max_m"], "rail stroke limit", color=c["star"])
    a2.set_yscale("log")
    ps.style_ax(a2, "mass ratio mu (%)", "damper stroke, p95 of peaks (m)", "Stroke needed")
    for k in VARIANTS:
        if k == "free_panel":
            continue
        a3.plot(mu, vp[k]["force_peak_kn"], color=colors[k], lw=1.6, marker="o", ms=3)
        a3.plot(mu, vp[k]["power_mean_kw"], color=colors[k], lw=1.0, ls=(0, (3, 2)))
    a3.plot(mu, vp["free_panel"]["stop_force_peak_kn"], color=c["star"], lw=1.2, ls=":", label="end-stop impact force")
    a3.set_yscale("log")
    a3.legend(loc="upper left", **ps.LEGEND_KW)
    ps.style_ax(a3, "mass ratio mu (%)", "peak force (kN, solid) / mean power (kW, dashed)",
                "Actuator force and power")
    a4.plot(mu, rec["zeta_add_bound_pct"], color=c["ink"], lw=1.7, label="bound (mu/2)(S/X), S = 0.2 m")
    ps.ref_line(a4, rec["host_zeta_pct"], "host damping 2 %", color=c["s1"])
    a4.set_yscale("log")
    a4.legend(loc="lower right", **ps.LEGEND_KW)
    ps.style_ax(a4, "mass ratio mu (%)", "added damping (%)", "Stroke-limited added damping")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    ps.brand(fig, "amd_reality · 2-DOF host 1e7 kg, T1 1.13 s, 2 % · research tier; mass dampers are mainly wind "
                  "devices, not a building-protection feature")
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the μ sweep for the three variants over the record ensemble (tier 2)."""
    rec = compute(cfg)
    return StudyResult(fig=_figure(rec), record=rec)


STUDY = StudySpec(
    name="amd_reality",
    tier=2,
    question="How much does a maglev panel used as a mass damper reduce building response vs mass ratio, within "
    "the LSM's force and stroke limits (mu < 0.5 % shaded product-irrelevant)?",
    run=run,
)
