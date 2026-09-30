"""Study ``isolation_montecarlo`` (tier 2, research tier): WALL contactless isolation under a KT/CP ensemble.

Question (FAB-SEI-015, FAB-SAF-020; SEISMIC.md §4.3): with N = 200 Kanai–Tajimi ×
Clough–Penzien records at PGA 0.5 g, how often does a levitated WALL in research-tier
isolation overload its end bumpers when its free stroke is ±0.2 m followed by the
150 mm eddy end zones, how often does it enter the end zones, and how sensitive is that
to the Clough–Penzien high-pass corner ω_f?

**Native Monte Carlo (the gate).** The real controller, integer monitor and Travel1d
plant (``_native.run_batch``; Travel1d is the seismic tier, plan §1), catalog scenario
``isolation_kt`` (KT/CP medium soil, ω_g 10 rad/s, ζ_g 0.4, ω_f = 0.1·ω_g, JHT
envelope, 40 s record at 200 Hz, 0.05 Hz baseline correction, PGA scaled to exactly
0.5 g; ``ground.synthesize``), 60 s runs, member seeds
``SeedSequence(seed).spawn(N)[i]`` (``fabrail.ensemble.member_seeds``). Research-tier
settings (never enabled by a preset): ``control.seismic.isolation.enable``,
``control.seismic.policy.action = isolate``, ``center_if_reachable = false``. Geometry
(physics perspective §4.3 scenario 8, "±0.2 m free stroke, 150 mm end zones, rated
bumper"): ``design.rail.travel_m`` = 0.4 m with the carriage starting at its centre, so
the nominal travel ends are the ±0.2 m stroke, the progressive end zones
(``design.damping.ez_len_m`` 0.15 m, ``ez_cmax`` 1.5 kN·s/m) lie beyond them and the
bumper faces beyond those (``safety.x_max_m`` and the WALL detent moved inside the
shortened travel so ``validate`` accepts it). ISOLATE runs under ``IsolateLimits``
(SAF-020: stroke 0.2 m, bumper KE 281.5 J, F_cont 100 N).

Per member: peak bumper impact energy ½·m·v² at contact onset (plant KPI
``bumper_energy_j``) against the rating ``design.damping.bumper_e_rated_j`` (563 J;
"overload" = above the rating) and against the IsolateLimits bumper KE; end-zone entry
(x outside the nominal travel) and its depth; peak relative stroke; the time actually
spent in ``MotionMode::Isolate``; the fraction of that time the LSM force sat at F_cont;
the first fault the monitor or supervisor raised. The gate is the Wilson-95 upper bound
of P(overload) < 5 % (k ≤ 3 of 200; ``analysis.wilson_upper``); P(end-zone entry) is
reported with its Wilson interval, never gated. **If the isolator never engages (a
monitor trip before PROTECT, for instance) the gate passes for the wrong reason: the
record says so (``isolate_engaged_members``) and the result must not be read as
evidence for isolation.**

**ω_f sensitivity.** The same member phases (``stationary`` draws the same φ_k from
the member generator) re-synthesised at ω_f/ω_g ∈ {0.05, 0.1, 0.2, 0.4}: native on the
first ``N_SENS`` members, reference model on all N.

**Reference model (design intent, not a gate).** A numpy 1-DOF along x of the same
records and plant constants, integrated at 1 ms (symplectic Euler, record linearly
interpolated): the core isolator law F = −k·x − c_sky·v̂_abs with the leaky-integrated
absolute velocity (0.1 Hz), clamped at F_max = min(F_cont, IsolateLimits.f_cont), the
stroke governor of ``fabrail_core::seismic::isolate`` (full braking once
m·v²/(2r)·a_brk_ratio ≥ F_max, push-back at the envelope), distributed and rail drag
(b_dist + c_rail, F_hyst), the progressive end zones and the Hunt–Crossley bumpers.
Two variants: ``isolator`` (the law never trips: no monitor) and ``free`` (LSM off, a
free levitated mass; the land-first justification, SEI-013). It says what the law would
do if nothing tripped; it is not the SIL result.

**T_min saturation.** T_min = 2π·√(m·s/F_cont) = 6.28 s and T_iso = 1.1·T_min = 6.91 s
for WALL (``analytic.t_min_isolation``); the force authority F_cont/m = 0.02 g against
0.5 g floor shaking, so the law saturates for most of the strong motion and WALL
isolation largely saturates (plan §2b item 5; SEISMIC.md §4.2). Research tier: nothing
here is a product claim.

Owner: B3 (Wave 2).
"""

from __future__ import annotations

import math
import os
import time
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from .. import ground
from ..analysis import wilson_upper
from .runner import StudyResult, StudySpec

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..config import RunConfig

__all__ = [
    "GEOMETRY_OVERRIDES",
    "N",
    "N_SENS",
    "OMEGA_F_RATIOS",
    "RESEARCH_OVERRIDES",
    "STUDY",
    "compute",
    "reference_model",
    "run",
    "wilson_lower",
]

N = 200
N_SENS = 50
PGA_G = 0.5
SOIL = "medium"
MODEL = "cp"
DURATION_S = 60.0
OMEGA_F_RATIOS = (0.05, 0.1, 0.2, 0.4)
_REC_S = 40.0
_REC_FS_HZ = 200.0
_T0_S = 1.0
_RECORD_RATE_HZ = 100.0
_CHANNELS = ("x_m", "v_mps", "f_lsm_n", "motion_mode")
_MODE_ISOLATE = 8
_REF_DT_S = 1e-3
#: SEI-015 gate: Wilson-95 upper bound of P(bumper overload) below 5 %.
_GATE_P = 0.05

#: Research-tier settings the scenario requires (``ScenarioDef.requires``) plus the
#: isolate action; never set by a preset.
RESEARCH_OVERRIDES: dict[str, object] = {
    "control.seismic.isolation.enable": True,
    "control.seismic.policy.action": "isolate",
    "control.seismic.policy.center_if_reachable": False,
}
#: ±0.2 m free stroke about the start, then the end zones (physics perspective §4.3).
GEOMETRY_OVERRIDES: dict[str, object] = {
    "design.rail.travel_m": 0.4,
    "safety.x_max_m": 0.41,
    "control.haptic.detent_x_m.0": 0.2,
}


def wilson_lower(k: int, n: int, z: float = 1.959963984540054) -> float:
    """Lower Wilson score bound (the companion of ``analysis.wilson_upper``; Wilson 1927)."""
    if n <= 0 or not 0 <= k <= n:
        raise ValueError(f"wilson_lower: need n > 0 and 0 <= k <= n (got k={k}, n={n})")
    p = k / n
    z2 = z * z
    centre = p + z2 / (2 * n)
    spread = z * math.sqrt(p * (1.0 - p) / n + z2 / (4.0 * n * n))
    return max(0.0, (centre - spread) / (1.0 + z2 / n))


def _cfg(cfg: RunConfig) -> RunConfig:
    from ..config import RunConfig as _RC

    return _RC(
        rail_class="wall",
        seed=cfg.seed,
        duration_s=DURATION_S,
        fidelity="travel1d",
        coil=cfg.coil,
        record_rate_hz=_RECORD_RATE_HZ,
        workers=cfg.workers,
        overrides={**RESEARCH_OVERRIDES, **GEOMETRY_OVERRIDES, **cfg.overrides},
        plant=dict(cfg.plant),
    )


def _soil(ratio: float) -> ground.SoilFilter:
    base = ground.SOIL_FILTERS[SOIL]
    return ground.SoilFilter(base.omega_g_rad_s, base.zeta_g, ratio * base.omega_g_rad_s, base.zeta_f)


def _record(member: Any, ratio: float) -> np.ndarray:
    """The member's record at ω_f = ratio·ω_g (same phases for every ratio)."""
    return ground.synthesize(member.rng(), _REC_S, _REC_FS_HZ, soil=_soil(ratio), pga_g=PGA_G, model=MODEL)  # type: ignore[arg-type]


def _jobs(params: Any, rc: RunConfig, members: list[Any], records: list[np.ndarray]) -> list[tuple[Any, Any]]:
    from .. import require_native, scenarios

    n = require_native()
    jobs = []
    for m, a in zip(members, records, strict=True):
        spec_d, series = scenarios.spec("isolation_kt", params, rc, member=m,
                                        options={"pga_g": PGA_G, "soil": SOIL, "model": MODEL})
        d = cast(dict[str, Any], spec_d)
        d["record"]["channels"] = list(_CHANNELS)
        series = {**series, "floor": np.ascontiguousarray(a, dtype="<f8").tobytes()}
        scenarios.validate_spec(d)
        scenarios.cross_check(d, series, params)
        jobs.append((params, n.Scenario.from_json(scenarios.to_json(d), series)))
    return jobs


def _member_result(run: Any, rec: np.ndarray, x_lo: float, x_hi: float, x_c: float, f_max: float) -> dict[str, Any]:
    k = run.kpis()
    ch = {name: np.frombuffer(run.channel(name)[0], dtype="<f8" if name != "motion_mode" else "u1") for name in
          _CHANNELS}
    x = ch["x_m"]
    mode = ch["motion_mode"]
    f = ch["f_lsm_n"]
    iso = mode == _MODE_ISOLATE
    ez_depth = float(max(0.0, x_lo - float(np.min(x)), float(np.max(x)) - x_hi))
    faults = [(float(t), int(a)) for t, kind, a, _b in run.events() if kind == "FaultRaised"]
    protect = next((float(t) for t, kind, a, _b in run.events() if kind == "SeismicState" and int(a) == 2), math.nan)
    disp = ground.trapezoid_integral(ground.trapezoid_integral(rec, _REC_FS_HZ), _REC_FS_HZ)
    return {
        "bumper_energy_j": float(k.get("bumper_energy_j", 0.0)),
        "bumper_impacts": int(k.get("bumper_impacts", 0.0)),
        "stroke_max_m": float(np.max(np.abs(x - x_c))),
        "end_zone_depth_m": ez_depth,
        "end_zone_entry": bool(ez_depth > 0.0),
        "isolate_s": float(np.count_nonzero(iso)) / float(run.rate_hz),
        "f_sat_fraction_in_isolate": float(np.mean(np.abs(f[iso]) >= 0.99 * f_max)) if np.any(iso) else math.nan,
        "t_protect_s": protect,
        "first_fault_code": faults[0][1] if faults else 0,
        "first_fault_t_s": faults[0][0] if faults else math.nan,
        "pgd_m": float(np.max(np.abs(disp))),
    }


def _smoothstep(u: np.ndarray) -> np.ndarray:
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def reference_model(records: np.ndarray, p: Any, lsm_on: np.ndarray | bool = True) -> dict[str, np.ndarray]:
    """Monitor-free 1-DOF of the isolator law (rows with ``lsm_on``) or a free mass (the others), vectorised.

    ``records``: (rows, n) floor acceleration at 200 Hz, m/s², starting at rest. Returns per
    row the peak bumper impact energy, peak stroke about the start, end-zone depth and the
    fraction of time the law saturated. Constants from ``p`` (the study's WALL parameters).
    """
    g = p.get
    m = float(g("design.carriage.m_rated_kg"))
    travel = float(g("design.rail.travel_m"))
    ez_len = float(g("design.damping.ez_len_m"))
    ez_c = float(g("design.damping.ez_cmax_n_s_per_m"))
    k_b = float(g("design.damping.bumper_k_n_per_m"))
    hc = float(g("design.damping.bumper_hc_s_per_m"))
    b_lin = float(g("design.damping.b_dist_n_s_per_m")) + float(g("design.damping.c_rail_n_s_per_m"))
    f_hyst = float(g("design.damping.f_hyst_n"))
    t_iso = float(g("control.seismic.isolation.t_iso_s"))
    zeta = float(g("control.seismic.isolation.zeta"))
    hp = float(g("control.seismic.isolation.hp_hz"))
    stroke = min(float(g("control.seismic.isolation.stroke_m")), float(g("safety.isolate.stroke_m")))
    f_max = min(float(g("design.lsm.f_cont_n")), float(g("safety.isolate.f_cont_n")))
    brk = max(float(g("control.motion.a_brk_ratio")), 1.0)
    w = 2.0 * math.pi / t_iso
    k_iso, c_sky = m * w * w, 2.0 * zeta * m * w
    a_g = np.asarray(records, dtype=np.float64)
    n_mem, n_rec = a_g.shape
    on = np.broadcast_to(np.asarray(lsm_on, dtype=bool), (n_mem,))
    up = int(round(1.0 / (_REF_DT_S * _REC_FS_HZ)))
    n_steps = (n_rec - 1) * up + int(round((DURATION_S - _REC_S) / _REF_DT_S))
    x = np.zeros(n_mem)  # relative to the stroke centre
    v = np.zeros(n_mem)
    v_abs = np.zeros(n_mem)
    half = 0.5 * travel
    in_contact = np.zeros(n_mem, dtype=bool)
    e_bump = np.zeros(n_mem)
    x_max = np.zeros(n_mem)
    ez_max = np.zeros(n_mem)
    sat = np.zeros(n_mem)
    a_hp = math.exp(-2.0 * math.pi * hp * _REF_DT_S)
    for i in range(n_steps):
        j, frac = divmod(i, up)
        ag = a_g[:, j] + (a_g[:, j + 1] - a_g[:, j]) * (frac / up) if j + 1 < n_rec else np.zeros(n_mem)
        law = -k_iso * x - c_sky * v_abs
        f = np.clip(law, -f_max, f_max)
        sat_now = np.abs(law) > f_max
        ax_ = np.abs(x)
        outward = x * v > 0.0
        r = np.maximum(stroke - ax_, 1e-9)
        gov = outward & (ax_ < stroke) & (m * v * v / (2.0 * r) * brk >= f_max)
        f = np.where(gov, -np.sign(v) * f_max, f)
        beyond = (ax_ >= stroke) & (v * x >= 0.0)
        f = np.where(beyond, -np.sign(x) * f_max, f)
        f = np.where(on, f, 0.0)
        sat += on & (sat_now | gov | beyond)
        depth = np.maximum(np.abs(x) - half, 0.0)
        f_ez = -ez_c * _smoothstep(depth / ez_len) * v
        pen = np.maximum(np.abs(x) - half - ez_len, 0.0)
        sgn = np.sign(x)
        pen_rate = sgn * v
        f_b = -sgn * np.maximum(k_b * pen * (1.0 + hc * pen_rate), 0.0) * (pen > 0.0)
        f_drag = -b_lin * v - f_hyst * np.tanh(v / 1e-3)
        a_rel = (f + f_ez + f_b + f_drag) / m - ag
        v = v + a_rel * _REF_DT_S
        x = x + v * _REF_DT_S
        a_abs = a_rel + ag
        v_abs = a_hp * v_abs + a_abs * _REF_DT_S
        contact = pen > 0.0
        onset = contact & ~in_contact
        e_bump = np.where(onset, np.maximum(e_bump, 0.5 * m * v * v), e_bump)
        in_contact = contact
        x_max = np.maximum(x_max, np.abs(x))
        ez_max = np.maximum(ez_max, depth)
    return {
        "bumper_energy_j": e_bump,
        "stroke_max_m": x_max,
        "end_zone_depth_m": ez_max,
        "saturated_fraction": sat / n_steps,
    }


def _stats(values: np.ndarray, flags: np.ndarray, n: int) -> dict[str, Any]:
    k = int(np.count_nonzero(flags))
    return {
        "k": k,
        "n": n,
        "p": k / n,
        "wilson_upper": wilson_upper(k, n),
        "wilson_lower": wilson_lower(k, n),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def compute(cfg: RunConfig, n: int = N, n_sens: int = N_SENS,
            ratios: tuple[float, ...] = OMEGA_F_RATIOS, reference: bool = True) -> dict[str, Any]:
    """The Monte Carlo, the ω_f sensitivity and the reference model (the JSON record)."""
    from .. import analytic, require_native
    from ..ensemble import member_seeds

    rc = _cfg(cfg)
    params = rc.params()
    e_rated = float(params.get("design.damping.bumper_e_rated_j"))
    e_iso = float(params.get("safety.isolate.bumper_ke_j"))
    f_max = min(float(params.get("design.lsm.f_cont_n")), float(params.get("safety.isolate.f_cont_n")))
    m = float(params.get("design.carriage.m_rated_kg"))
    stroke = float(params.get("safety.isolate.stroke_m"))
    x_lo = float(params.get("design.rail.x_closed_m"))
    x_hi = x_lo + float(params.get("design.rail.travel_m"))
    x_c = 0.5 * (x_lo + x_hi)
    members = member_seeds(cfg.seed, n)
    workers = cfg.workers or os.cpu_count() or 1
    native = require_native()

    t0 = time.perf_counter()
    ratios = tuple(ratios) if n_sens > 0 else ()
    recs = {r: [_record(mb, r) for mb in members[:n_sens]] for r in ratios if r != 0.1}
    recs[0.1] = [_record(mb, 0.1) for mb in members]  # the main ensemble (catalog omega_f = 0.1 omega_g)
    t1 = time.perf_counter()
    main_jobs = _jobs(params, rc, members, recs[0.1])
    t2 = time.perf_counter()
    main_runs = native.run_batch(main_jobs, cfg.workers)
    t3 = time.perf_counter()
    main = [_member_result(r, a, x_lo, x_hi, x_c, f_max) for r, a in zip(main_runs, recs[0.1], strict=True)]
    del main_runs
    sens: dict[str, Any] = {}
    for r in ratios:
        sub = main[:n_sens] if r == 0.1 else [
            _member_result(run, a, x_lo, x_hi, x_c, f_max)
            for run, a in zip(native.run_batch(_jobs(params, rc, members[:n_sens], recs[r]), cfg.workers), recs[r],
                              strict=True)
        ]
        eb = np.array([s["bumper_energy_j"] for s in sub])
        ez = np.array([s["end_zone_entry"] for s in sub])
        sens[f"{r:g}"] = {
            "omega_f_ratio": r,
            "omega_f_rad_s": r * ground.SOIL_FILTERS[SOIL].omega_g_rad_s,
            "n": len(sub),
            "overload": _stats(eb, eb > e_rated, len(sub)),
            "end_zone_entry": _stats(np.array([s["end_zone_depth_m"] for s in sub]), ez, len(sub)),
            "pgd_p50_m": float(np.median([s["pgd_m"] for s in sub])),
            "stroke_p50_m": float(np.median([s["stroke_max_m"] for s in sub])),
        }
    t4 = time.perf_counter()
    ref: dict[str, Any] = {}
    if reference:
        rows, keys = [], []
        for r in sorted({0.1, *ratios}):
            a_all = np.vstack(recs[0.1]) if r == 0.1 else np.vstack([_record(mb, r) for mb in members])
            rows.append(a_all)
            keys.append(r)
        stack = np.vstack(rows)
        both = np.vstack([stack, stack])
        on = np.concatenate([np.ones(stack.shape[0], dtype=bool), np.zeros(stack.shape[0], dtype=bool)])
        out = reference_model(both, params, on)
        for vi, variant in enumerate(("isolator", "free")):
            for ri, r in enumerate(keys):
                sl = slice(vi * stack.shape[0] + ri * n, vi * stack.shape[0] + (ri + 1) * n)
                o = {name: arr[sl] for name, arr in out.items()}
                ref[f"{variant}_{r:g}"] = {
                    "omega_f_ratio": r,
                    "variant": variant,
                    "overload": _stats(o["bumper_energy_j"], o["bumper_energy_j"] > e_rated, n),
                    "end_zone_entry": _stats(o["end_zone_depth_m"], o["end_zone_depth_m"] > 0.0, n),
                    "stroke_p50_m": float(np.median(o["stroke_max_m"])),
                    "stroke_sorted_m": np.sort(o["stroke_max_m"]),
                    "saturated_fraction_p50": float(np.median(o["saturated_fraction"])),
                    "bumper_energy_sorted_j": np.sort(o["bumper_energy_j"]),
                }
    t5 = time.perf_counter()
    eb = np.array([s["bumper_energy_j"] for s in main])
    ezd = np.array([s["end_zone_depth_m"] for s in main])
    iso_s = np.array([s["isolate_s"] for s in main])
    codes = [s["first_fault_code"] for s in main]
    code_counts = {f"0x{c:04X}": codes.count(c) for c in sorted(set(codes)) if c}
    gate = _stats(eb, eb > e_rated, n)
    t_min = analytic.t_min_isolation(m, stroke, f_max)
    engaged = int(np.count_nonzero(iso_s > 0.0))
    return {
        "inputs": {
            "n": n,
            "n_sens": n_sens,
            "pga_g": PGA_G,
            "soil": SOIL,
            "model": MODEL,
            "omega_f_ratios": list(ratios),
            "api": "compute(cfg, n=200, n_sens=50, ratios=OMEGA_F_RATIOS, reference=True); n_sens=0 skips the "
            "sensitivity; record['native']['overload']['wilson_upper'] is the SEI-015 gate value and "
            "record['native']['isolate_engaged_members'] says whether it means anything",
            "record_s": _REC_S,
            "duration_s": DURATION_S,
            "seed": cfg.seed,
            "seed_rule": "SeedSequence(seed).spawn(N)[i] (fabrail.ensemble.member_seeds)",
            "research_overrides": dict(RESEARCH_OVERRIDES),
            "geometry_overrides": dict(GEOMETRY_OVERRIDES),
            "fidelity": "travel1d",
            "bumper_e_rated_j": e_rated,
            "isolate_bumper_ke_j": e_iso,
            "isolate_stroke_m": stroke,
            "bumper_face_m": 0.5 * (x_hi - x_lo) + float(params.get("design.damping.ez_len_m")),
            "f_max_n": f_max,
            "mass_kg": m,
            "overload": "peak bumper impact energy 1/2 m v^2 at contact onset > bumper_e_rated_j",
            "gate": "Wilson-95 upper bound of P(overload) < 5 % (k <= 3 of 200) (FAB-SEI-015)",
            "gate_p": 0.05,
            "tier": "research (isolation is not a product feature)",
        },
        "native": {
            "overload": gate,
            "gate_pass": bool(gate["wilson_upper"] < _GATE_P),
            "above_isolate_bumper_ke": int(np.count_nonzero(eb > e_iso)),
            "end_zone_entry": _stats(ezd, ezd > 0.0, n),
            "isolate_engaged_members": engaged,
            "isolate_s_p50": float(np.median(iso_s)),
            "isolate_s_max": float(np.max(iso_s)),
            "first_fault_counts": code_counts,
            "t_first_fault_p50_s": float(np.nanmedian([s["first_fault_t_s"] for s in main])) if code_counts
            else math.nan,
            "t_protect_p50_s": float(np.nanmedian([s["t_protect_s"] for s in main])),
            "gate_meaningful": bool(engaged == n),
            "bumper_energy_sorted_j": np.sort(eb),
            "stroke_sorted_m": np.sort([s["stroke_max_m"] for s in main]),
            "members": main,
        },
        "sensitivity_native": sens,
        "reference": ref,
        "saturation": {
            "t_min_s": t_min,
            "t_iso_s": float(params.get("control.seismic.isolation.t_iso_s")),
            "force_authority_g": f_max / (m * ground.G0_MPS2),
            "floor_pga_g": PGA_G,
            "note": "the LSM can push with F_cont/m = 0.02 g against 0.5 g floor shaking; WALL isolation largely "
            "saturates (plan §2b item 5)",
        },
        # Wall-clock numbers live under "wall_s", which export excludes from the sha256.
        "timing": {
            "main_simulated_s": n * DURATION_S,
            "budget": "D8: isolation MC N=200 x 60 s <= 60 s on 8 workers (wall_s.main_batch_s)",
            "wall_s": {
                "synthesis_s": t1 - t0,
                "build_s": t2 - t1,
                "main_batch_s": t3 - t2,
                "main_real_time_factor": n * DURATION_S / (t3 - t2) if t3 > t2 else math.nan,
                "sensitivity_s": t4 - t3,
                "reference_s": t5 - t4,
                "workers": workers,
            },
        },
    }


def _figure(rec: dict[str, Any]) -> Any:
    from matplotlib.figure import Figure

    from .. import plotstyle as ps

    ps.apply()
    c = ps.COLORS
    fig = Figure(figsize=(13.0, 4.8))
    a1, a2, a3 = fig.subplots(1, 3)
    nat = rec["native"]
    n = rec["inputs"]["n"]
    q = (np.arange(1, n + 1) - 0.5) / n
    rec["native"]["cdf_q"] = q
    a1.plot(nat["bumper_energy_sorted_j"], q, color=c["ink"], lw=2.0, label="native (controller + monitor)")
    ref = rec["reference"]
    if "isolator_0.1" in ref:
        a1.plot(ref["isolator_0.1"]["bumper_energy_sorted_j"], q, color=c["s1"], lw=1.4, label="reference: isolator law")
        a1.plot(ref["free_0.1"]["bumper_energy_sorted_j"], q, color=c["s2"], lw=1.4, label="reference: free mass")
    a1.axvline(rec["inputs"]["bumper_e_rated_j"], color=c["star"], ls=(0, (4, 3)), lw=1.2)
    a1.axvline(rec["inputs"]["isolate_bumper_ke_j"], color=c["muted"], ls=":", lw=1.0)
    a1.set_ylim(0.0, 1.0)
    a1.legend(loc="lower right", **ps.LEGEND_KW)
    if not nat["gate_meaningful"]:
        a1.annotate(
            f"isolator engaged in {nat['isolate_engaged_members']}/{n} native runs:\nthe native curve is mostly SS1 "
            "+ rollers, not isolation",
            xy=(0.98, 0.42), xycoords="axes fraction", ha="right", fontsize=8.5, color=c["star"],
        )
    ps.style_ax(a1, "peak bumper impact energy (J)", "CDF over records", "Bumper energy (rating dashed)")
    a2.plot(nat["stroke_sorted_m"], q, color=c["ink"], lw=2.0, label="native")
    if "isolator_0.1" in ref:
        a2.plot(ref["isolator_0.1"]["stroke_sorted_m"], q, color=c["s1"], lw=1.4, label="reference: isolator law")
        a2.plot(ref["free_0.1"]["stroke_sorted_m"], q, color=c["s2"], lw=1.4, label="reference: free mass")
    a2.axvline(rec["inputs"]["isolate_stroke_m"], color=c["muted"], ls=":", lw=1.0)
    a2.axvline(rec["inputs"]["bumper_face_m"], color=c["star"], ls=(0, (4, 3)), lw=1.0)
    a2.set_ylim(0.0, 1.0)
    ps.style_ax(a2, "peak stroke about the centre (m)", None, "Stroke (end zone 0.2 m, bumper face dashed)")
    sens = rec["sensitivity_native"]
    ratios = [sens[k]["omega_f_ratio"] for k in sens]
    a3.plot(ratios, [sens[k]["end_zone_entry"]["p"] for k in sens], color=c["ink"], marker="o", lw=1.6,
            label=f"native P(end-zone entry), N={rec['inputs']['n_sens']}")
    a3.plot(ratios, [sens[k]["overload"]["wilson_upper"] for k in sens], color=c["star"], marker="s", lw=1.2,
            label=f"native Wilson-95 P(overload), N={rec['inputs']['n_sens']}")
    iso_keys = [f"isolator_{r:g}" for r in ratios if f"isolator_{r:g}" in ref]
    if not ratios:
        ratios = [0.1]
    if iso_keys:
        a3.plot([ref[k]["omega_f_ratio"] for k in iso_keys], [ref[k]["end_zone_entry"]["p"] for k in iso_keys],
                color=c["s1"], marker="o", lw=1.2, ls=(0, (3, 2)), label="reference law P(end-zone entry)")
        a3.plot([ref[k]["omega_f_ratio"] for k in iso_keys], [ref[k]["overload"]["wilson_upper"] for k in iso_keys],
                color=c["s1"], marker="s", lw=1.0, ls=":", label="reference law Wilson-95 P(overload)")
    ps.ref_line(a3, rec["inputs"]["gate_p"], "5 % gate", color=c["star"], x=0.6)
    a3.set_xscale("log")
    a3.set_xticks(ratios, [f"{r:g}" for r in ratios])
    a3.minorticks_off()
    a3.set_ylim(0.0, 1.05)
    a3.legend(loc="center left", fontsize=8, frameon=False)
    ps.style_ax(a3, "omega_f / omega_g (Clough-Penzien)", "probability", "Sensitivity to omega_f")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    ps.brand(fig, f"isolation_montecarlo · WALL, N={n}, KT/CP 0.5 g, +-0.2 m + 150 mm end zones · research tier, "
                  "not a product feature")
    return fig


def run(cfg: RunConfig) -> StudyResult:
    """Run the N = 200 native Monte Carlo, the ω_f sensitivity and the reference model (tier 2)."""
    rec = compute(cfg)
    return StudyResult(fig=_figure(rec), record=rec)


STUDY = StudySpec(
    name="isolation_montecarlo",
    tier=2,
    question="Under N = 200 KT/CP records at 0.5 g, how often does an isolating WALL (+-0.2 m stroke, 150 mm end "
    "zones) overload its bumpers, and how sensitive is that to omega_f?",
    run=run,
)
