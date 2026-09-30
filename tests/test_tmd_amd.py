"""Contract: tuned and active mass damping, FAB-SEI-016 (analysis) and FAB-SEI-021 (closed loop).

Research tier (status R): mass damping by a rail panel is not a building-protection
feature; the mass ratio μ = m_damper/M_building decides the effect and is reported by
every test here (``record_property`` and the assertion messages).

- SEI-016 (method A): the native closed forms (``fabrail_params::derive`` via
  ``_native.den_hartog``, ``den_hartog_peak``, ``warburton``) equal Den Hartog's and
  Warburton's formulas (tolerance (E), rel 1e-12) and the quoted anchors (half a unit of
  the last quoted digit). They are then checked against physics computed independently in
  numpy/scipy: the 2-DOF frequency response of an undamped primary with the Den Hartog
  absorber (fixed points P and Q invariant in ζ at rel 1e-9, equal heights √(1 + 2/μ),
  peak within 2 %); the white-noise base-excitation variance of the primary, from the
  Lyapunov equation, which has the Warburton optimum as a strict local minimum under ±5 %
  perturbations; and the scalar LQR closed form K = (a + √(a² + b²q/r))/b against the host
  Riccati solver the design uses for the injected AMD/HMD gains (scipy CARE, rel 1e-9).
- SEI-021 (method S): a closed-loop active mass damper (direct velocity feedback) on the
  2-DOF Structure model (one host storey plus the HEAVY panel; ``harmonic_base`` on the
  ``structure`` tier) at the host's first-mode frequency. The mass ratio is swept by the
  host mass at fixed period (μ = 0.05 %, 0.5 %, 2 %). The reduction of the host's steady
  harmonic amplitude, AMD on versus the same run with a zero DVF gain, must follow the
  linear closed form r = (c/Mω)/(2ζ + c/Mω) within ±25 % (P) where that form predicts a
  measurable effect, be negligible (≤ 2 %) at μ = 0.05 % (a 5 t panel in a 10⁷ kg
  building), and honour the LSM force limit ``amd.f_max_n`` and the stroke envelope
  min(``isolation.stroke_m``, ``safety.isolate.stroke_m``). The AMD mode is research tier
  (``amd.enable = false`` in every preset); the test enables it, selects the ``damp``
  policy action and injects a synthetic network alert (not a live EEW feed) so the
  seismic state is PROTECT before the base motion starts.

Owner: B5 (Wave 2).
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy import linalg, optimize

from fabrail import config, scenarios
from fabrail.results import RunResult

#: The reference mass ratio of the anchors (SPECIFICATION FAB-SEI-016; core design §E.5).
MU = 0.02


def _rel(a: float, b: float) -> float:
    return abs(a - b) / max(abs(a), abs(b), 1e-300)


# ---------------------------------------------------------------------------------
# SEI-016: Den Hartog frequency response
# ---------------------------------------------------------------------------------


def _frf(mu: float, f: float, zeta: float, r: np.ndarray) -> np.ndarray:
    """|X1/X_st| of an undamped primary (m1 = k1 = 1) with an absorber (μ, f, ζ) under a harmonic force.

    Solved as the 2-DOF complex linear system at each forcing ratio r = ω/ω1 (no closed form
    is used, so the Den Hartog formulas are checked against independent algebra).
    """
    k2, c2 = mu * f * f, 2.0 * mu * zeta * f
    out = np.empty(r.shape)
    for i, w in enumerate(r):
        k_dyn = np.array(
            [[1.0 + k2 - w * w + 1j * w * c2, -k2 - 1j * w * c2], [-k2 - 1j * w * c2, k2 - mu * w * w + 1j * w * c2]]
        )
        out[i] = abs(np.linalg.solve(k_dyn, np.array([1.0, 0.0]))[0])
    return out


def _frf_locked(mu: float, r: float) -> float:
    """ζ → ∞: the absorber is locked to the primary (mass 1 + μ, stiffness 1)."""
    return 1.0 / abs(1.0 - (1.0 + mu) * r * r)


@pytest.mark.req("FAB-SEI-016")
def test_den_hartog_frf(native, record_property):
    """Contract: Den Hartog's optimum (f 0.98039, ζ 0.08407 at μ = 0.02) is reproduced by the native closed form
    and by an independent 2-DOF FRF: fixed points invariant in ζ, equal heights √(1 + 2/μ) = 10.05, peak ±2 %."""
    record_property("mu", MU)
    f, zeta = native.den_hartog(MU)
    f_cf, z_cf = 1.0 / (1.0 + MU), math.sqrt(3.0 * MU / (8.0 * (1.0 + MU) ** 3))
    assert _rel(f, f_cf) <= 1e-12, f"(E) rel 1e-12: den_hartog f {f} vs 1/(1+μ) {f_cf} (Den Hartog; PHYSICS §21), μ={MU}"
    assert _rel(zeta, z_cf) <= 1e-12, f"(E) rel 1e-12: den_hartog ζ {zeta} vs √(3μ/8(1+μ)³) {z_cf}, μ={MU}"
    assert abs(f - 0.98039) <= 5e-6, f"f = {f:.6f}, anchor 0.98039 ±5e-6 (SPECIFICATION FAB-SEI-016), μ={MU}"
    assert abs(zeta - 0.08407) <= 5e-6, f"ζ = {zeta:.6f}, anchor 0.08407 ±5e-6 (SPECIFICATION FAB-SEI-016), μ={MU}"

    h_eq = math.sqrt(1.0 + 2.0 / MU)
    peak_native = native.den_hartog_peak(MU)
    assert _rel(peak_native, h_eq) <= 1e-12, f"(E) den_hartog_peak {peak_native} vs √(1+2/μ) {h_eq}, μ={MU}"
    assert abs(peak_native - 10.05) <= 5e-3, f"equal-peak height {peak_native:.4f}, anchor 10.05 ±0.005 (core §E.5), μ={MU}"

    # Fixed points P and Q: where the undamped and the locked responses cross (|H| independent of ζ).
    def gap(r: float) -> float:
        return float(_frf(MU, f, 0.0, np.array([r]))[0]) - _frf_locked(MU, r)

    r_nat = (0.85, 1.0 / math.sqrt(1.0 + MU), 1.15)  # locked-system resonance separates P and Q
    r_p = optimize.brentq(gap, r_nat[0], r_nat[1] - 1e-6, xtol=1e-15)
    r_q = optimize.brentq(gap, r_nat[1] + 1e-6, r_nat[2], xtol=1e-15)
    for r_fp in (r_p, r_q):
        h_fp = _frf_locked(MU, r_fp)
        assert _rel(h_fp, h_eq) <= 1e-9, f"fixed point r={r_fp:.6f}: height {h_fp:.8f} vs √(1+2/μ) {h_eq:.8f} (rel 1e-9), μ={MU}"
        for z in (0.01, 0.05, zeta, 0.2, 1.0):
            h = float(_frf(MU, f, z, np.array([r_fp]))[0])
            assert _rel(h, h_fp) <= 1e-9, f"fixed point r={r_fp:.6f} not invariant in ζ={z}: {h} vs {h_fp} (rel 1e-9), μ={MU}"

    r = np.linspace(0.85, 1.15, 30001)
    peak = float(_frf(MU, f, zeta, r).max())
    record_property("frf_peak", round(peak, 4))
    assert abs(peak / h_eq - 1.0) <= 0.02, f"FRF peak {peak:.3f} vs √(1+2/μ) {h_eq:.3f}: 2 % peak tolerance (physics A16), μ={MU}"


# ---------------------------------------------------------------------------------
# SEI-016: Warburton optimum (white-noise base acceleration)
# ---------------------------------------------------------------------------------


def _variance_base(mu: float, f: float, zeta: float) -> float:
    """σ² of the primary's displacement relative to the base under unit white-noise base acceleration.

    Undamped primary (m1 = k1 = 1), absorber (μ, f, ζ); state [x1, x2, ẋ1, ẋ2] relative to the base;
    A·P + P·Aᵀ + B·Bᵀ = 0 (Lyapunov), σ² = P[0, 0].
    """
    k2, c2 = mu * f * f, 2.0 * mu * zeta * f
    a = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [-(1.0 + k2), k2, -c2, c2],
            [k2 / mu, -k2 / mu, c2 / mu, -c2 / mu],
        ]
    )
    b = np.array([[0.0], [0.0], [-1.0], [-1.0]])
    p = linalg.solve_continuous_lyapunov(a, -b @ b.T)
    return float(p[0, 0])


@pytest.mark.req("FAB-SEI-016")
def test_warburton_minimum(native, record_property):
    """Contract: Warburton's base-excitation optimum (0.97548 / 0.07019 at μ = 0.02) is the native closed form and a
    strict local minimum of the primary's white-noise response variance (±5 % perturbations each increase σ²)."""
    record_property("mu", MU)
    f, zeta = native.warburton(MU)
    f_cf = math.sqrt(1.0 - MU / 2.0) / (1.0 + MU)
    z_cf = math.sqrt(MU * (1.0 - MU / 4.0) / (4.0 * (1.0 + MU) * (1.0 - MU / 2.0)))
    assert _rel(f, f_cf) <= 1e-12, f"(E) rel 1e-12: warburton f {f} vs √(1−μ/2)/(1+μ) {f_cf} (Warburton 1982), μ={MU}"
    assert _rel(zeta, z_cf) <= 1e-12, f"(E) rel 1e-12: warburton ζ {zeta} vs Warburton's base-excitation form {z_cf}"
    assert abs(f - 0.97548) <= 5e-6, f"f = {f:.6f}, anchor 0.97548 ±5e-6 (SPECIFICATION FAB-SEI-016), μ={MU}"
    assert abs(zeta - 0.07019) <= 5e-6, f"ζ = {zeta:.6f}, anchor 0.07019 ±5e-6 (SPECIFICATION FAB-SEI-016), μ={MU}"

    v0 = _variance_base(MU, f, zeta)
    record_property("sigma2_opt", v0)
    for label, (ff, zz) in {
        "f −5 %": (0.95 * f, zeta),
        "f +5 %": (1.05 * f, zeta),
        "ζ −5 %": (f, 0.95 * zeta),
        "ζ +5 %": (f, 1.05 * zeta),
    }.items():
        v = _variance_base(MU, ff, zz)
        assert v > v0, f"Warburton optimum is not a strict local minimum: σ²({label}) = {v:.9g} ≤ σ²(opt) = {v0:.9g}, μ={MU}"


# ---------------------------------------------------------------------------------
# SEI-016: scalar LQR
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-SEI-016")
def test_lqr_scalar(record_property):
    """Contract: the scalar LQR gain K = (a + √(a² + b²q/r))/b equals the host Riccati solver (scipy CARE, the
    offline solver for the injected AMD/HMD gains) to rel 1e-9, and the closed loop pole is −√(a² + b²q/r)."""
    cases = [
        (0.0, 1.0, 1.0, 1.0),
        (0.5, 2.0, 3.0, 0.1),  # open-loop unstable
        (-0.3, 0.01, 1.0e4, 1.0e-6),  # stable, weak actuator (a mass damper's force channel)
        (1.0e-3, 1.0 / 5000.0, 1.0e6, 1.0e-4),  # 5 t reaction mass: b = 1/m
        (-2.0, 5.0, 0.25, 4.0),
    ]
    record_property("cases", len(cases))
    for a, b, q, r in cases:
        p = linalg.solve_continuous_are(np.array([[a]]), np.array([[b]]), np.array([[q]]), np.array([[r]]))
        k_care = float(b * p[0, 0] / r)
        root = math.sqrt(a * a + b * b * q / r)
        k_cf = (a + root) / b
        p_cf = r * (a + root) / (b * b)
        assert _rel(k_care, k_cf) <= 1e-9, (
            f"(a,b,q,r)=({a},{b},{q},{r}): CARE K {k_care!r} vs closed form {k_cf!r} (rel 1e-9, physics A24)"
        )
        assert _rel(float(p[0, 0]), p_cf) <= 1e-9, f"(a,b,q,r)=({a},{b},{q},{r}): CARE P {p[0, 0]!r} vs r(a+√…)/b² {p_cf!r}"
        pole = a - b * k_cf
        assert _rel(pole, -root) <= 1e-12 and pole < 0.0, f"closed-loop pole {pole} must equal −√(a²+b²q/r) = {-root} (stable)"


# ---------------------------------------------------------------------------------
# SEI-021: closed-loop AMD on the 2-DOF Structure model
# ---------------------------------------------------------------------------------

#: Host masses of the μ sweep, kg (HEAVY panel 5 t: μ = 0.05 %, 0.5 %, 2 %). The period is
#: kept at the builder's T1 = 1.13 s (``scenarios._HOST_T1_S``, Kyobashi-like host).
HOST_MASSES_KG = (1.0e7, 1.0e6, 2.5e5)
#: DVF gain, N·s/m: large enough to matter at μ = 2 %, small enough that the resonant host
#: velocity keeps |F| below the HEAVY force limit (500 N) at the base amplitude below.
C_DVF = 2.0e4
#: Harmonic base acceleration amplitude, m/s² (linear, unsaturated range).
A_BASE = 0.005
#: Forcing cycles (the builder ramps in over 2 cycles; the second half is the steady window).
CYCLES = 30.0
#: "Negligible" at μ = 0.05 %: ≤ 2 % response reduction, the scale of the stroke-limited bound
#: ζ_add ≤ (μ/2)(S/X) = 0.035 % against ζ = 2 % at S/X = 1.4 (SEISMIC.md §3; EXOTIC §2).
NEGLIGIBLE = 0.02
#: Provisional (P) relative tolerance of the simulated reduction against the linear closed form.
REDUCTION_TOL = 0.25


def _fit_amplitude(t: np.ndarray, y: np.ndarray, f_hz: float) -> float:
    """Least-squares amplitude of the f_hz component (plus offset) over whole cycles."""
    w = 2.0 * math.pi * f_hz
    basis = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    c, *_ = np.linalg.lstsq(basis, y, rcond=None)
    return math.hypot(c[0], c[1])


def _amd_job(native, host_kg: float, c_dvf: float):
    overrides = {
        "control.seismic.amd.enable": True,
        "control.seismic.amd.c_dvf_n_s_per_m": c_dvf,
        "control.seismic.policy.action": "damp",
        "control.seismic.policy.center_if_reachable": False,
        "control.seismic.fusion.alert_timeout_s": 600.0,
    }
    cfg = config.RunConfig(rail_class="heavy", seed=42, coil="voltage", overrides=overrides)
    p = cfg.params()
    d, series = scenarios.spec("harmonic_base", p, cfg, fidelity="structure", options={"cycles": CYCLES, "amp_mps2": A_BASE})
    st = d["plant"]["structure"]
    k_per_kg = st["storey_k_n_per_m"][0] / st["storey_mass_kg"][0]
    st["storey_mass_kg"] = [host_kg]
    st["storey_k_n_per_m"] = [k_per_kg * host_kg]
    # Synthetic/injected alert (not a live EEW feed) so the unit is in PROTECT → Damp before the base motion.
    d["events"] = sorted(
        [
            *d["events"],
            {
                "kind": "network_alert",
                "t_s": 0.5,
                "magnitude": 7.0,
                "r_epi_km": 60.0,
                "depth_km": 10.0,
                "origin_t_s": -4.5,
                "id": 1,
            },
        ],
        key=lambda e: e["t_s"],
    )
    d["record"]["channels"] = ["x_m", "host_x_m", "f_lsm_n", "motion_mode", "seismic_action"]
    d["record"]["rate_hz"] = 200.0
    info = scenarios.info(d)
    return p, d, native.Scenario.from_json(scenarios.to_json(d), series), float(info["freq_hz"])


@pytest.mark.req("FAB-SEI-021")
def test_closed_loop_structure_2dof(native, record_property):
    """Contract: a closed-loop DVF active mass damper on the 2-DOF Structure model reduces the host's resonant
    response as the linear closed form predicts for its mass ratio, negligibly at μ = 0.05 %, within the LSM
    force and stroke limits (FAB-SEI-021, research tier)."""
    jobs, meta = [], []
    for host in HOST_MASSES_KG:
        for c in (0.0, C_DVF):
            p, d, sc, f_hz = _amd_job(native, host, c)
            jobs.append((p, sc))
            meta.append((host, c, p, f_hz))
    results = [RunResult.from_native(r) for r in native.run_batch(jobs, None)]

    m_panel = native.Params.preset("heavy").get("design.carriage.m_star_kg")
    amp: dict[tuple[float, float], float] = {}
    for (host, c, p, f_hz), r in zip(meta, results, strict=True):
        rec_s = CYCLES / f_hz
        t0 = 1.0 + 0.5 * rec_s  # series t0 = 1 s (builder), steady second half
        t1 = t0 + math.floor(0.5 * rec_s * f_hz) / f_hz
        win = (r.t_s >= t0) & (r.t_s < t1)
        amp[(host, c)] = _fit_amplitude(r.t_s[win], r["host_x_m"][win], f_hz)
        if c > 0.0:
            damp_share = float(np.mean(r["motion_mode"][win] == 9))  # MotionMode::Damp wire value (fabrail-types state.rs)
            f_lim = p.get("control.seismic.amd.f_max_n")
            s_lim = min(p.get("control.seismic.isolation.stroke_m"), p.get("safety.isolate.stroke_m"))
            x_c = float(np.median(r["x_m"][r.t_s < 0.5]))
            f_max = float(np.max(np.abs(r["f_lsm_n"][win])))
            stroke = float(np.max(np.abs(r["x_m"][win] - x_c)))
            assert damp_share >= 0.99, (
                f"μ={m_panel / host:.4%}: the carriage must be in MotionMode::Damp in the steady window (got {damp_share:.2f})"
            )
            assert f_max <= f_lim * (1.0 + 1e-6), (
                f"μ={m_panel / host:.4%}: |F_LSM| {f_max:.1f} N exceeds amd.f_max_n {f_lim} N (SEI-021 force limit)"
            )
            assert stroke <= s_lim, (
                f"μ={m_panel / host:.4%}: stroke {stroke:.4f} m exceeds the envelope {s_lim} m (SEI-021; SAF-020)"
            )

    zeta = 0.02  # the builder's host damping (scenarios._HOST_ZETA)
    rows = []
    for host in HOST_MASSES_KG:
        mu = m_panel / host
        w1 = 2.0 * math.pi * meta[0][3]
        add = C_DVF / (host * w1)
        predicted = add / (2.0 * zeta + add)
        measured = 1.0 - amp[(host, C_DVF)] / amp[(host, 0.0)]
        rows.append((mu, predicted, measured))
        record_property(f"mu_{mu:.4f}_reduction_measured", round(measured, 5))
        record_property(f"mu_{mu:.4f}_reduction_predicted", round(predicted, 5))

    mu0, pred0, meas0 = rows[0]
    assert pred0 <= NEGLIGIBLE, f"closed form at μ={mu0:.4%} predicts {pred0:.3%}: must itself be negligible (≤ {NEGLIGIBLE:.0%})"
    assert abs(meas0) <= NEGLIGIBLE, (
        f"μ={mu0:.4%} (5 t panel, 10⁷ kg host): reduction {meas0:.3%} must be negligible (≤ {NEGLIGIBLE:.0%}; SEI-021)"
    )
    for mu, pred, meas in rows[1:]:
        assert abs(meas - pred) <= REDUCTION_TOL * pred, (
            f"μ={mu:.3%}: host response reduction {meas:.3%} vs linear DVF closed form (c/Mω)/(2ζ + c/Mω) = {pred:.3%} "
            f"(±{REDUCTION_TOL:.0%} relative, (P); c = {C_DVF:g} N·s/m, ζ = {zeta}, a_base = {A_BASE} m/s²; SEI-021)"
        )
    measured = [m for _, _, m in rows]
    assert measured == sorted(measured), (
        f"the reduction must grow with μ (SEI-021): {[(f'{mu:.4%}', f'{m:.3%}') for mu, _, m in rows]}"
    )
