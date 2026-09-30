"""Contract: free-mass drift, isolation and vertical shaking, FAB-SEI-013, -014, -015 and -017.

Closed-loop runs through ``_native.run``/``run_batch`` on catalog scenarios
(``fabrail.scenarios``). Isolation is research tier (status R; ``isolation.enable = false``
in every preset): where it is exercised the test enables it, selects the ``isolate``
policy action, turns F11 centring off and injects a synthetic network alert (not a live
EEW feed) before the base motion so the unit is in PROTECT → ISOLATE when shaking starts
(the 30 s STA/LTA warm-up is longer than these records' lead-in).

- SEI-013 (scenario 7): the ``elcentro_free`` surrogate (and the fetched 1940 record,
  ``requires_data``) drives a released ("manual", unpropelled) WALL and HEAVY; the peak
  drift relative to the floor must be the record's baseline-corrected PGD ±10 %. DRAWER
  and DOOR travel (0.3 / 1.0 m) is shorter than the surrogate's PGD (≈ 0.7 m), so for them
  the claim is the end-stop behaviour covered by FAB-SEI-019. The free mass must stay
  unforced (no LSM force, no floor-driven monitor trip; lead decision D3).
- SEI-014: the A-part is the passive transmissibility closed form (``_native
  .transmissibility``): T(3, 0.2) = 0.1931 and T(√2) = 1 for any ζ. The S-part runs the
  WALL isolator under harmonic base motion in its linear range (rail hysteresis and
  viscous rail drag off in the plant truth, amplitude well inside F_cont and the stroke)
  and compares the measured |a_abs/a_floor| at r = 3 and r = √2 with the closed form of
  the law the core implements, Karnopp skyhook F = −k·x_rel − c_sky·v̂_abs with v̂_abs the
  leaky-integrated carriage IMU (``fabrail_core::seismic::isolate``; the ideal form is
  ``_native.skyhook_transmissibility``), ±2 % (tolerance class I; physics perspective A15).
- SEI-015 (slow; scenario 8): Monte Carlo, N = 200 Clough–Penzien medium-soil records at
  0.5 g on a WALL with a ±0.2 m free stroke (travel 0.4 m about the centre), 150 mm end
  zones and rated bumpers beyond it, under ``IsolateLimits``; gate: Wilson-95 upper bound
  of P(bumper energy > rating) < 5 % (k ≤ 3). P(end-zone entry), the bumper-energy
  quantiles and the ω_f sensitivity (Kanai–Tajimi, i.e. ω_f → 0, and CP with ω_f doubled,
  N = 50 each) are reported.
- SEI-017 (scenario 10): a downward 0.8 g, 0.2 s half-sine floor pulse on a levitated WALL
  (Heave1 and Carriage6) saturates the lift's low side (SAT_COIL), raises
  VerticalSaturation and lands the carriage; any upper-stop contact is at or below the
  rated snap-up speed ``impact_speed(params, "up")``.

Owner: B5 (Wave 2).
"""

from __future__ import annotations

import dataclasses
import math
import time
from pathlib import Path

import numpy as np
import pytest

from fabrail import config, ensemble, ground, records, scenarios
from fabrail.analysis import wilson_upper
from fabrail.results import RunResult

# Wire values (crates/fabrail-types/src/state.rs, fault.rs; fabrail-core msg.rs SAT_* bits; W0-frozen).
LEV_MANUAL, LEV_FAULT_REACTION, LEV_FAULT = 14, 10, 11
MODE_ISOLATE = 8
FAULT_VERTICAL_SATURATION = 0x0701
SAT_COIL = 1 << 0
FLOOR_DRIVEN_TRIPS = {
    0x0601: "EnergyLimit",
    0x0602: "MomentumLimit",
    0x0604: "Overspeed",
    0x0605: "PositionLimit",
    0x0606: "UnintendedMotion",
}
C_LANDED = 1 << 2

#: Research-tier isolation configuration (module docstring).
ISOLATE = {
    "control.seismic.isolation.enable": True,
    "control.seismic.policy.action": "isolate",
    "control.seismic.policy.center_if_reachable": False,
    "control.seismic.fusion.alert_timeout_s": 600.0,
}


def _alert_at(t_s: float, r_epi_km: float = 60.0):
    """Synthetic/injected network alert (not a live EEW feed) that puts the unit in PROTECT at t_s."""

    def add(d):
        d["events"] = sorted(
            [
                *d["events"],
                {
                    "kind": "network_alert",
                    "t_s": t_s,
                    "magnitude": 7.0,
                    "r_epi_km": r_epi_km,
                    "depth_km": 10.0,
                    "origin_t_s": t_s - 5.0,
                    "id": 1,
                },
            ],
            key=lambda e: e["t_s"],
        )

    return add


def _job(
    native,
    cls,
    name,
    *,
    fidelity=None,
    options=None,
    overrides=None,
    plant=None,
    member=None,
    mutate=None,
    channels=None,
    rate_hz=None,
    seed=42,
):
    cfg = config.RunConfig(rail_class=cls, seed=seed, coil="voltage", overrides=dict(overrides or {}), plant=dict(plant or {}))
    p = cfg.params()
    d, series = scenarios.spec(name, p, cfg, fidelity=fidelity, member=member, options=options)
    if channels is not None:
        d["record"]["channels"] = list(channels)
    if rate_hz is not None:
        d["record"]["rate_hz"] = float(rate_hz)
    if mutate is not None:
        mutate(d)
    return p, d, series


def _run_all(native, jobs):
    batch = [(p, native.Scenario.from_json(scenarios.to_json(d), s)) for p, d, s in jobs]
    return [RunResult.from_native(r) for r in native.run_batch(batch, None)]


def _faults(r, t0=-math.inf):
    return [(round(e.t_s, 3), int(e.a)) for e in r.events if e.kind == "FaultRaised" and e.t_s >= t0]


# ---------------------------------------------------------------------------------
# SEI-013
# ---------------------------------------------------------------------------------

_DATA = Path(__file__).resolve().parents[1] / "data"


@pytest.mark.req("FAB-SEI-013")
@pytest.mark.parametrize("record_name", ["elcentro_like", pytest.param("elcentro_1940", marks=pytest.mark.requires_data)])
def test_free_mass_drifts_pgd(native, record_property, record_name):
    """Contract: a released (unpropelled) levitated-class carriage under the El Centro record drifts relative to
    the floor by the record's baseline-corrected PGD ±10 % — the land-first justification (FAB-SEI-013,
    scenario 7; WALL and HEAVY, whose free travel exceeds the PGD)."""
    options = None
    if record_name == "elcentro_1940":
        if not (_DATA / "elcentro.json").exists():
            pytest.skip("requires_data: the El Centro record has not been fetched into data/")
        rec = records.load_elcentro(_DATA)
        a = np.asarray(rec.a_mps2, dtype=np.float64)
        rec = dataclasses.replace(rec, a_mps2=ground.baseline_correct(a if a.ndim == 1 else a[0], rec.fs_hz))
        options = {"record": rec}
    channels = ["x_m", "v_mps", "f_lsm_n", "lev_state", "demand", "floor_ax_mps2"]
    jobs = [_job(native, cls, "elcentro_free", options=options, channels=channels, rate_hz=200.0) for cls in ("wall", "heavy")]
    bad = []
    for (p, d, _), r, cls in zip(jobs, _run_all(native, jobs), ("wall", "heavy"), strict=True):
        info = scenarios.info(d)
        pgd = float(info["pgd_m"])
        x0 = float(d["initial"]["x_m"])
        drift = float(np.max(np.abs(r["x_m"] - x0)))
        record_property(f"{cls}_{record_name}_drift_over_pgd", round(drift / pgd, 4))
        travel, x_closed = p.get("design.rail.travel_m"), p.get("design.rail.x_closed_m")
        assert x_closed + pgd * 1.2 < x0 < x_closed + travel - pgd * 1.2, f"{cls}: free travel must exceed the PGD {pgd:.2f} m"
        if abs(drift / pgd - 1.0) > 0.10:
            bad.append(f"{cls}: drift {drift:.3f} m vs PGD {pgd:.3f} m (ratio {drift / pgd:.3f}; faults {_faults(r)})")
        manual = np.nonzero(r["lev_state"] == LEV_MANUAL)[0]
        if not manual.size:
            bad.append(f"{cls}: never reached the Manual (released, unpropelled) state; faults {_faults(r)}")
            continue
        t_rel = float(r.t_s[manual[0]])
        # Unforced: no dissipative LSM force after the release. The equivalent viscous coefficient
        # b_eq = −⟨F·v⟩/⟨v²⟩ ignores conservative, position-periodic cogging (HEAVY's iron-core LSM) and would be
        # the ASC drag b_e = (2/3)·K_f²/R (lead decision D5; PHYSICS §7) if the monitor had shorted the winding.
        free = r.t_s >= t_rel + 0.05
        f_lsm, v = r["f_lsm_n"][free], r["v_mps"][free]
        b_eq = -float(np.dot(f_lsm, v)) / max(float(np.dot(v, v)), 1e-12)
        b_asc = (2.0 / 3.0) * p.get("design.lsm.k_f_n_per_a") ** 2 / p.get("design.lsm.r_phase_ohm")
        record_property(f"{cls}_{record_name}_lsm_b_eq_n_s_per_m", round(b_eq, 3))
        if abs(b_eq) > 0.02 * b_asc:
            bad.append(
                f"{cls}: LSM drag b_eq {b_eq:.1f} N·s/m after the release (ASC would be {b_asc:.0f}) — the mass is not free"
            )
        trips = [(t, FLOOR_DRIVEN_TRIPS[c]) for t, c in _faults(r, t_rel) if c in FLOOR_DRIVEN_TRIPS]
        if trips:
            bad.append(f"{cls}: floor-driven monitor trips after the release {trips} (D3)")
    assert not bad, f"SEI-013 free-mass drift ≈ PGD ±10 % ({record_name}): {bad}"


# ---------------------------------------------------------------------------------
# SEI-014
# ---------------------------------------------------------------------------------


def _fit(t: np.ndarray, y: np.ndarray, f_hz: float) -> complex:
    """Complex amplitude of the f_hz component (least squares with an offset) over the window."""
    w = 2.0 * math.pi * f_hz
    c, *_ = np.linalg.lstsq(np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)]), y, rcond=None)
    return complex(c[0], c[1])


#: Isolation damping ratio of the anchor (SPECIFICATION FAB-SEI-014: T(3, 0.2)).
ZETA = 0.2
#: Base amplitudes, m/s²: the relative stroke stays ≤ ≈ 0.1 m and the force ≤ ≈ 20 % of F_cont (linear range).
T_CASES = ((3.0, 0.2), (math.sqrt(2.0), 0.03))
T_TOL = 0.02


def _skyhook_law_t(p, r_ratio: float) -> float:
    """|a_abs/a_floor| of the core isolation law as documented (fabrail_core::seismic::isolate; PHYSICS §21):
    F = −k·x_rel − c_sky·v̂_abs, k = m·ω_i², c_sky = 2ζ·m·ω_i (m rated, ω_i = 2π/T_iso), with v̂_abs the carriage IMU
    through the leaky integrator 1/(s + ω_h), ω_h = 2π·``isolation.hp_hz``:
    T(jω) = k / (k − m·ω² − c_sky·ω²/(jω + ω_h)). ω_h → 0 gives the ideal skyhook 1/√((1 − r²)² + (2ζr)²)."""
    m = p.get("design.carriage.m_rated_kg")
    w_i = 2.0 * math.pi / p.get("control.seismic.isolation.t_iso_s")
    zeta = p.get("control.seismic.isolation.zeta")
    w_h = 2.0 * math.pi * p.get("control.seismic.isolation.hp_hz")
    k, c = m * w_i * w_i, 2.0 * zeta * m * w_i
    w = r_ratio * w_i
    return abs(k / (k - m * w * w - c * w * w / (1j * w + w_h)))


@pytest.mark.req("FAB-SEI-014")
def test_transmissibility(native, record_property):
    """Contract: T(3, 0.2) = 0.1931 and T(√2) = 1 for any ζ (passive closed form, A); the simulated WALL isolator
    in its linear range reproduces the transmissibility of its skyhook law at r = 3 and r = √2 within ±2 % (S)
    (FAB-SEI-014, research tier)."""
    t3 = native.transmissibility(3.0, ZETA)
    assert abs(t3 - 0.1931) <= 5e-5, f"T(3, 0.2) = {t3:.5f}, anchor 0.1931 (half a unit of the last digit; SEI-014, core §E.5)"
    t3_cf = math.sqrt((1.0 + (2.0 * ZETA * 3.0) ** 2) / ((1.0 - 9.0) ** 2 + (2.0 * ZETA * 3.0) ** 2))
    assert abs(t3 - t3_cf) <= 1e-12 * t3_cf, f"(E) transmissibility(3, 0.2) {t3!r} vs closed form {t3_cf!r}"
    for z in (0.0, 0.05, 0.2, 0.5, 1.0, 3.0):
        tr = native.transmissibility(math.sqrt(2.0), z)
        assert abs(tr - 1.0) <= 1e-12, f"(E) T(√2, ζ={z}) = {tr!r}, must be 1 for any ζ (SEI-014)"

    jobs = []
    for r_ratio, amp in T_CASES:
        jobs.append(
            _job(
                native,
                "wall",
                "harmonic_base",
                overrides={**ISOLATE, "control.seismic.isolation.zeta": ZETA},
                plant={"c_rail_n_s_per_m": 0.0, "f_hyst_n": 0.0},
                options={"r": r_ratio, "amp_mps2": amp, "cycles": 40.0},
                channels=["x_m", "acc_abs_x_mps2", "floor_ax_mps2", "motion_mode", "f_lsm_n"],
                rate_hz=200.0,
                mutate=_alert_at(0.5),
            )
        )
    bad = []
    for (p, d, _), r, (r_ratio, _amp) in zip(jobs, _run_all(native, jobs), T_CASES, strict=True):
        f_hz = float(scenarios.info(d)["freq_hz"])
        rec_s = 40.0 / f_hz
        t0 = 1.0 + 0.5 * rec_s
        win = (r.t_s >= t0) & (r.t_s < t0 + math.floor(0.5 * rec_s * f_hz) / f_hz)
        iso = float(np.mean(r["motion_mode"][win] == MODE_ISOLATE))
        t_sim = abs(_fit(r.t_s[win], r["acc_abs_x_mps2"][win], f_hz) / _fit(r.t_s[win], r["floor_ax_mps2"][win], f_hz))
        t_law = _skyhook_law_t(p, r_ratio)
        f_peak = float(np.max(np.abs(r["f_lsm_n"][win])))
        stroke = float(np.ptp(r["x_m"][win])) / 2.0
        record_property(f"r{r_ratio:.3f}_T_sim", round(t_sim, 4))
        record_property(f"r{r_ratio:.3f}_T_law_with_leaky_imu", round(t_law, 4))
        record_property(f"r{r_ratio:.3f}_T_ideal_skyhook", round(native.skyhook_transmissibility(r_ratio, ZETA), 4))
        record_property(f"r{r_ratio:.3f}_T_passive", round(native.transmissibility(r_ratio, ZETA), 4))
        if iso < 0.99:
            bad.append(f"r={r_ratio:.3f}: isolation mode held for only {iso:.0%} of the steady window; faults {_faults(r)}")
            continue
        if f_peak > 0.5 * p.get("design.lsm.f_cont_n") or stroke > 0.5 * p.get("control.seismic.isolation.stroke_m"):
            bad.append(f"r={r_ratio:.3f}: not in the linear range (|F| {f_peak:.1f} N, stroke {stroke:.3f} m)")
        if abs(t_sim / t_law - 1.0) > T_TOL:
            bad.append(
                f"r={r_ratio:.3f}: simulated T {t_sim:.4f} vs the implemented skyhook law {t_law:.4f} (ratio "
                f"{t_sim / t_law:.3f}, ±{T_TOL:.0%}); ideal skyhook {native.skyhook_transmissibility(r_ratio, ZETA):.4f}, "
                f"passive closed form {native.transmissibility(r_ratio, ZETA):.4f}"
            )
    assert not bad, f"SEI-014 transmissibility, simulated WALL isolator (ζ = {ZETA}, linear range): {bad}"


# ---------------------------------------------------------------------------------
# SEI-015 (slow)
# ---------------------------------------------------------------------------------

MC_N = 200
MC_SENS_N = 50
#: Monte Carlo geometry: a ±0.2 m free stroke about the centre, the end zones and bumpers beyond it
#: (SPECIFICATION FAB-SEI-015; physics perspective scenario 8). The monitor's SLP follows the travel.
MC_GEOMETRY = {"design.rail.travel_m": 0.4, "control.haptic.n_detents": 0, "safety.x_max_m": 0.41}


def _mc_jobs(native, n: int, model: str, omega_f: float | None = None):
    cfg_seed = 42
    jobs = []
    for m in ensemble.member_seeds(cfg_seed, n):
        p, d, s = _job(
            native,
            "wall",
            "isolation_kt",
            overrides={**ISOLATE, **MC_GEOMETRY},
            options={"pga_g": 0.5, "soil": "medium", "model": model},
            member=m,
            channels=["x_m"],
            rate_hz=100.0,
            mutate=_alert_at(0.5, r_epi_km=30.0),
        )
        if omega_f is not None:
            # Same synthesis as the builder (ground.synthesize), with the Clough–Penzien corner ω_f changed.
            soil = dataclasses.replace(ground.SOIL_FILTERS["medium"], omega_f_rad_s=omega_f)
            meta = d["series"]["floor"]
            a = ground.synthesize(m.rng(), 40.0, float(meta["fs_hz"]), soil=soil, pga_g=0.5, envelope=None, model="cp")
            s = {**s, "floor": np.ascontiguousarray(a, dtype="<f8").tobytes()}
        jobs.append((p, d, s))
    return jobs


def _mc_stats(jobs, runs):
    rated = jobs[0][0].get("design.damping.bumper_e_rated_j")
    x0, travel = jobs[0][0].get("design.rail.x_closed_m"), jobs[0][0].get("design.rail.travel_m")
    e = np.array([r.kpis.get("bumper_energy_j", 0.0) for r in runs])
    entry = np.array([bool((r["x_m"] < x0).any() or (r["x_m"] > x0 + travel).any()) for r in runs])
    k = int(np.count_nonzero(e > rated))
    return {
        "k_overload": k,
        "n": len(runs),
        "wilson95": wilson_upper(k, len(runs)),
        "p_end_zone": float(entry.mean()),
        "e_p50_j": float(np.quantile(e, 0.5)),
        "e_p95_j": float(np.quantile(e, 0.95)),
        "rated_j": rated,
    }


@pytest.mark.slow
@pytest.mark.req("FAB-SEI-015")
def test_monte_carlo_bumper_overload(native, record_property):
    """Contract: over N = 200 KT/CP records at 0.5 g, the isolated WALL (±0.2 m, end zones, under IsolateLimits)
    keeps the Wilson-95 upper bound of P(bumper overload) below 5 % (k ≤ 3); P(end-zone entry) and the ω_f
    sensitivity are reported (FAB-SEI-015, research tier, slow)."""
    t0 = time.perf_counter()
    jobs = _mc_jobs(native, MC_N, "cp")
    runs = _run_all(native, jobs)
    wall = time.perf_counter() - t0
    base = _mc_stats(jobs, runs)
    for key, val in base.items():
        record_property(f"cp_omega_f_1_{key}", val)
    record_property("mc_wall_s", round(wall, 1))
    for label, model, omega_f in (("kt_omega_f_0", "kt", None), ("cp_omega_f_2", "cp", 2.0)):
        sj = _mc_jobs(native, MC_SENS_N, model, omega_f)
        for key, val in _mc_stats(sj, _run_all(native, sj)).items():
            record_property(f"{label}_{key}", val)
    assert base["wilson95"] < 0.05, (
        f"Wilson-95 upper bound of P(bumper energy > {base['rated_j']} J) = {base['wilson95']:.4f} "
        f"(k = {base['k_overload']}/{base['n']}); required < 5 %, k ≤ 3 (SEI-015; P(end-zone entry) {base['p_end_zone']:.2f})"
    )


# ---------------------------------------------------------------------------------
# SEI-017
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-SEI-017")
@pytest.mark.parametrize("tier", ["heave1", "carriage6"])
def test_vertical_pulse_forces_landing(native, record_property, tier):
    """Contract: a downward 0.8 g floor pulse on a levitated WALL saturates the low side of the lift (SAT_COIL),
    raises VerticalSaturation and lands the carriage, with any upper-stop contact at or below the rated snap-up
    speed (FAB-SEI-017, scenario 10; Heave1 and Carriage6)."""
    channels = ["z_m", "gap_0_m", "gap_1_m", "gap_2_m", "gap_3_m", "f_upper_n", "sat_flags", "lev_state", "contact_state"]
    (p, d, s) = _job(native, "wall", "vertical_pulse", fidelity=tier, channels=channels, rate_hz=2000.0)
    r = _run_all(native, [(p, d, s)])[0]
    info = scenarios.info(d)
    t_on = 1.0  # builder: half-sine from t = 1 s
    t_off = t_on + float(info["pulse_s"])
    bad = []
    pulse = (r.t_s >= t_on) & (r.t_s <= t_off + 0.1)
    if not (r["sat_flags"][pulse] & SAT_COIL).any():
        bad.append("no coil saturation (SAT_COIL) during the pulse")
    raised = {code for _, code in _faults(r)}
    record_property("faults_raised", sorted(hex(c) for c in raised))
    if FAULT_VERTICAL_SATURATION not in raised:
        bad.append(f"VerticalSaturation (0x0701) not raised; faults raised {sorted(hex(c) for c in raised)}")
    landed = [e.t_s for e in r.events if e.kind == "plant:landed" and e.t_s >= t_on]
    if not landed or landed[0] > t_off + 2.0 or not (int(r["contact_state"][-1]) & C_LANDED):
        bad.append(
            f"no landing within 2 s of the pulse (plant:landed at {landed[:1]}, final contact {int(r['contact_state'][-1]):#x})"
        )
    v_rated = native.impact_speed(p, "up")
    contacts = np.nonzero((r["f_upper_n"] > 0.0) & (np.concatenate([[0.0], r["f_upper_n"][:-1]]) <= 0.0))[0]
    # Closing speed of the magnet gaps (carriage relative to the moving rail) over the 10 ms before each contact.
    closing = np.max(np.vstack([-np.gradient(r[f"gap_{k}_m"], r.t_s) for k in range(4)]), axis=0)
    speeds = [float(np.max(closing[max(0, i - 20) : i + 1])) for i in contacts]
    record_property("upper_contacts", len(contacts))
    record_property("upper_impact_speed_max_mps", round(max(speeds), 4) if speeds else 0.0)
    record_property("rated_up_speed_mps", round(v_rated, 4))
    if speeds and max(speeds) > v_rated:
        bad.append(f"upper-stop impact at {max(speeds):.4f} m/s > rated snap-up speed {v_rated:.4f} m/s (impact_speed 'up')")
    assert not bad, f"SEI-017 downward 0.8 g pulse on WALL ({tier}): {bad}"
