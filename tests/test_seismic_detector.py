"""Contract: the structure-mounted P-wave detector, FAB-SEI-002, -003, -004 and -006.

The detector is the firmware's own seismic unit (``fabrail_core::seismic``), reached two
ways: ``_native.DetectorReplay`` (ADXL355 quantisation, the 32-sample FIFO, the 100 Hz
seismic releases; events stamped with the release time) and, where the actuator veto or
Pd must be observed, full ``_native.run`` scenarios on the Travel1d tier.

The f64 numpy reference below is written from PHYSICS §20 / SEISMIC §2.2 and the
parameter preset, not from the Rust code:

- anti-alias: the second-order CIC ÷5, i.e. the triangular FIR [1 2 3 4 5 4 3 2 1]/25 on
  the integer ADXL355 codes, evaluated at every 5th sample and primed with the first
  sample; output m is due at ODR sample 5m + 4 and is processed by release m + 1, so its
  event time is (m + 1)·10 ms;
- band-pass: 4-pole Butterworth 0.5–20 Hz at 100 Hz, prewarped Tustin
  (``scipy.signal.butter`` with ``fs``), started from the DC steady state of its first
  input (``sosfilt_zi``);
- STA/LTA on the band-passed vertical channel: e = x², gains 1/min(k, N), STA 50 and LTA
  3000 samples, LTA frozen while triggered, on at ratio ≥ 4 once k ≥ 3000, off below 1.5.

Inputs are built from integer codes times the code scale, so the native quantiser
reproduces them exactly. The "≤ 1e-4 relative" of SEI-002 is checked where it is
observable, in the trigger decisions: the input is tuned (bisection on a burst amplitude)
so that the reference ratio peaks at θ·(1 + δ) with δ ∈ [+0.3, +1]·1e-4 (the native unit
must trigger on that very sample) or at θ·(1 − δ) (it must not trigger at all); a native
ratio off by more than 1e-4 relative would flip one of the two decisions.

Scenarios (plan core §4.3): 1 (24 h site noise, slow) and 11 (PGA bypass under the
actuator veto) live here; the policy scenarios are in test_seismic_policy.py.

Owner: B5 (Wave 2).
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest
from scipy import signal

from fabrail import config, ensemble, pwave, scenarios
from fabrail.results import RunResult

#: Wire values of fabrail_types enums (crates/fabrail-types/src/state.rs, frozen in W0).
SEIS_NORMAL, SEIS_PROTECT = 0, 2
LEV_LANDING = 8

FS_ODR = 500.0  # ADXL355 output data rate, Hz (design.sensors.accel_odr_hz)
#: Allowance for the simulated ADXL355 noise moving the 80 gal crossing earlier than the noise-free reference, s.
NOISE_LEAD_S = 0.03
REF_CLASS = "wall"  # the detector does not depend on the class; WALL is the land-first reference


# ---------------------------------------------------------------------------------
# f64 reference front end (PHYSICS §20)
# ---------------------------------------------------------------------------------


def _cic2(codes: np.ndarray, decim: int) -> np.ndarray:
    """Second-order CIC ÷decim on integer codes (3, N): weighted sums (int64), one per decim samples."""
    taps = 2 * decim - 1
    w = np.array([min(k + 1, taps - k) for k in range(taps)], dtype=np.int64)
    c = np.asarray(codes, dtype=np.int64)
    ext = np.concatenate([np.repeat(c[:, :1], taps - 1, axis=1), c], axis=1)
    idx = np.arange(decim - 1, c.shape[1], decim)
    out = np.zeros((c.shape[0], idx.size), dtype=np.int64)
    for k in range(taps):
        out += w[k] * ext[:, idx + taps - 1 - k]
    return out


def _bandpass_sos(det: dict, fs: float) -> np.ndarray:
    return signal.butter(det["bp_order"] // 2, [det["bp_lo_hz"], det["bp_hi_hz"]], btype="bandpass", fs=fs, output="sos")


def _front_end(codes: np.ndarray, lsb: float, det: dict) -> np.ndarray:
    """Band-passed (3, M) acceleration, m/s², at the decimated rate."""
    d = det["aa_decim"]
    raw = _cic2(codes, d).astype(np.float64) * (lsb / (d * d))
    sos = _bandpass_sos(det, FS_ODR / d)
    zi = signal.sosfilt_zi(sos)
    return np.vstack([signal.sosfilt(sos, raw[ax], zi=zi * raw[ax, 0])[0] for ax in range(3)])


def _stalta(x: np.ndarray, det: dict, floor: float, *, freeze: bool = True) -> tuple[np.ndarray, list[int]]:
    """Exact recursive STA/LTA (PHYSICS §20): ratio per sample and the onset sample indices."""
    n_sta, n_lta, on, off = det["sta_n"], det["lta_n"], det["trig_on"], det["trig_off"]
    sta = lta = 0.0
    trig = False
    ratio = np.empty(x.size)
    onsets: list[int] = []
    for k in range(1, x.size + 1):
        e = float(x[k - 1]) ** 2
        sta += (e - sta) / min(k, n_sta)
        if not (trig and freeze):
            lta += (e - lta) / min(k, n_lta)
        r = sta / max(lta, floor)
        ratio[k - 1] = r
        if trig:
            if r < off:
                trig = False
        elif k >= n_lta and r >= on:
            trig = True
            onsets.append(k - 1)
    return ratio, onsets


def _detector(p) -> dict:
    keys = ("aa_decim", "bp_order", "bp_lo_hz", "bp_hi_hz", "sta_n", "lta_n", "trig_on", "trig_off")
    return {k: p.get(f"control.seismic.detector.{k}") for k in keys}


def _replay(native, p, codes: np.ndarray, lsb: float) -> tuple[list[tuple[float, str, float, float]], dict]:
    a = np.asarray(codes, dtype=np.float64) * lsb
    det = native.DetectorReplay(p, FS_ODR)
    ev = det.feed(*(np.ascontiguousarray(a[ax], dtype="<f8").tobytes() for ax in range(3)))
    return ev, det.report()


def _onset_times(events) -> list[float]:
    return [round(t, 6) for t, kind, _, _ in events if kind == "Trigger"]


def _ref_times(onsets: list[int]) -> list[float]:
    return [round((m + 1) * 0.01, 6) for m in onsets]  # decimated sample m → release m + 1 (10 ms)


# ---------------------------------------------------------------------------------
# SEI-002
# ---------------------------------------------------------------------------------


@pytest.mark.req("FAB-SEI-002")
def test_stalta_matches_reference(native, record_property):
    """Contract: the native front end (÷5 CIC, 0.5–20 Hz 4-pole prewarped band-pass, STA/LTA 50/3000 with the LTA
    frozen while triggered) makes the same trigger decisions as the f64 numpy reference, including decisions whose
    reference ratio sits within 1e-4 relative of the threshold (FAB-SEI-002)."""
    p = native.Params.preset(REF_CLASS)
    det = _detector(p)
    assert det == {
        "aa_decim": 5,
        "bp_order": 4,
        "bp_lo_hz": 0.5,
        "bp_hi_hz": 20.0,
        "sta_n": 50,
        "lta_n": 3000,
        "trig_on": 4.0,
        "trig_off": 1.5,
    }, f"detector tuning drifted from core design §C.4 / SEI-002: {det}"
    lsb = p.get("design.sensors.accel_mps2_per_lsb")
    floor = lsb * lsb  # SeismicUnit sets the ratio floor to 1 LSB²
    rng = np.random.default_rng(20260926)
    n = int(60.0 * FS_ODR)
    t = np.arange(n) / FS_ODR
    g = rng.standard_normal((3, n))

    # (a) Bursts, some arriving while a trigger holds (the LTA freeze decides their onsets).
    amp = np.full(n, 300.0)
    for t0, dur, k in (
        (32.0, 1.5, 4.0),
        (33.9, 0.8, 6.0),
        (38.0, 2.0, 2.6),
        (44.0, 1.2, 5.0),
        (45.8, 0.5, 8.0),
        (52.0, 3.0, 3.0),
    ):
        amp[(t >= t0) & (t < t0 + dur)] *= k
    codes = np.zeros((3, n), dtype=np.int64)
    codes[2] = np.round(amp * g[2]).astype(np.int64)
    codes[:2] = np.round(90.0 * g[:2]).astype(np.int64)
    bp = _front_end(codes, lsb, det)
    _, on_ref = _stalta(bp[2], det, floor)
    _, on_nofreeze = _stalta(bp[2], det, floor, freeze=False)
    ev, rep = _replay(native, p, codes, lsb)
    native_times, ref_times = _onset_times(ev), _ref_times(on_ref)
    record_property("burst_onsets", ref_times)
    assert len(ref_times) >= 3, f"test signal must trigger several times (reference onsets {ref_times})"
    assert _ref_times(on_nofreeze) != ref_times, (
        "test signal must be sensitive to the LTA freeze (reference with/without freeze agree)"
    )
    assert native_times == ref_times, (
        f"trigger onsets: native {native_times} vs f64 reference {ref_times} (exact, 10 ms releases; SEI-002, PHYSICS §20)"
    )
    assert int(rep["raw_triggers"]) == len(ref_times), f"raw_triggers {rep['raw_triggers']} vs reference {len(ref_times)}"

    # (b) Threshold-sharp decisions: the reference ratio peaks at θ(1 ± δ), δ ∈ [0.3e-4, 1e-4].
    eps, theta = 1e-4, det["trig_on"]
    n2 = int(50.0 * FS_ODR)
    t2 = np.arange(n2) / FS_ODR
    g2 = rng.standard_normal((3, n2))
    burst = (t2 >= 40.0) & (t2 < 42.0)
    m0 = int(40.0 * 100) - 1

    def codes_for(s: float) -> np.ndarray:
        c = np.zeros((3, n2), dtype=np.int64)
        c[2] = np.round(np.where(burst, s * 300.0, 300.0) * g2[2]).astype(np.int64)
        c[:2] = np.round(90.0 * g2[:2]).astype(np.int64)
        return c

    def peak_ratio(s: float) -> float:
        # Without the freeze the ratio equals the frozen one up to and including the onset sample.
        ratio, _ = _stalta(_front_end(codes_for(s), lsb, det)[2], det, floor, freeze=False)
        return float(ratio[m0 : m0 + 600].max())

    lo, hi = 1.0, 4.0
    assert peak_ratio(lo) < theta < peak_ratio(hi), "burst amplitude bracket must straddle the trigger threshold"
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if peak_ratio(mid) >= theta else (mid, hi)

    def search(s0: float, step: float, lo_rel: float, hi_rel: float) -> tuple[float, float]:
        s = s0
        for _ in range(400):
            v = peak_ratio(s) / theta - 1.0
            if lo_rel <= v <= hi_rel:
                return s, v
            s *= 1.0 + step
        raise AssertionError(f"could not tune the burst to a reference ratio in θ·(1 + [{lo_rel}, {hi_rel}])")

    s_above, d_above = search(hi, +2e-6, 0.3 * eps, eps)
    s_below, d_below = search(lo, -2e-6, -eps, -0.3 * eps)
    record_property("sharp_above_rel", d_above)
    record_property("sharp_below_rel", d_below)

    c_above = codes_for(s_above)
    _, on_above = _stalta(_front_end(c_above, lsb, det)[2], det, floor)
    nat_above = _onset_times(_replay(native, p, c_above, lsb)[0])
    assert on_above and nat_above == _ref_times(on_above), (
        f"reference ratio θ·(1 + {d_above:.2e}) at sample {on_above}: native onsets {nat_above} vs reference "
        f"{_ref_times(on_above)} — the native ratio is not within 1e-4 relative of the f64 reference (SEI-002)"
    )
    c_below = codes_for(s_below)
    _, on_below = _stalta(_front_end(c_below, lsb, det)[2], det, floor)
    nat_below = _onset_times(_replay(native, p, c_below, lsb)[0])
    assert not on_below and not nat_below, (
        f"reference peak ratio θ·(1 − {-d_below:.2e}) never triggers; native onsets {nat_below} — the native ratio is "
        "not within 1e-4 relative of the f64 reference (SEI-002)"
    )


# ---------------------------------------------------------------------------------
# SEI-003
# ---------------------------------------------------------------------------------


def _exact_trigger_sample(k: float, det: dict) -> int:
    """First sample (1-based) at which the exact discrete recursion from the steady state, LTA term included,
    reaches the trigger ratio after a variance step 1 → K (plan §2b item 7)."""
    sta = lta = 1.0
    for i in range(1, 100_000):
        sta += (k - sta) / det["sta_n"]
        lta += (k - lta) / det["lta_n"]
        if sta / lta >= det["trig_on"]:
            return i
    raise AssertionError("no trigger")


@pytest.mark.req("FAB-SEI-003")
def test_trigger_time_step(native, record_property):
    """Contract: after a variance step 1 → K the native trigger fires at the sample the exact discrete STA/LTA
    recursion (LTA term included) gives, ±1 sample, for K = 16 and 9; the continuous estimate
    τ_sta·ln((K − 1)/(K − θ)) = 0.1116 s at K = 16 is reported (FAB-SEI-003)."""
    p = native.Params.preset(REF_CLASS)
    det = _detector(p)
    lsb = p.get("design.sensors.accel_mps2_per_lsb")
    fs_dec = FS_ODR / det["aa_decim"]
    tau_sta = det["sta_n"] / fs_dec
    # A 25 Hz tone is sampled every quarter period at 100 Hz: with the band-passed phase at π/4 (mod π/2) every
    # decimated sample has the same energy, so the STA/LTA sees an exact variance step (no noise realisation).
    f_tone = fs_dec / 4.0
    _, h = signal.sosfreqz(_bandpass_sos(det, fs_dec), worN=[f_tone], fs=fs_dec)
    phase = math.pi / 4.0 - float(np.angle(h[0]))
    t_warm, t_after = 120.0, 3.0
    n = int((t_warm + t_after) * FS_ODR)
    t = np.arange(n) / FS_ODR
    m_step = int(round(t_warm * fs_dec))  # first decimated sample whose CIC window holds post-step samples

    cont = tau_sta * math.log((16.0 - 1.0) / (16.0 - det["trig_on"]))
    record_property("continuous_estimate_K16_s", round(cont, 5))
    assert abs(cont - 0.1116) <= 5e-5, f"continuous estimate {cont:.5f} s vs 0.1116 s (SEI-003, Allen 1978)"

    for k in (16.0, 9.0):
        a = np.where(t < t_warm, 2000.0, 2000.0 * math.sqrt(k)) * np.sin(2.0 * math.pi * f_tone * t + phase)
        codes = np.zeros((3, n), dtype=np.int64)
        codes[2] = np.round(a).astype(np.int64)
        exact = _exact_trigger_sample(k, det)
        _, on_ref = _stalta(_front_end(codes, lsb, det)[2], det, lsb * lsb)
        nat = _onset_times(_replay(native, p, codes, lsb)[0])
        assert len(nat) == 1 and len(on_ref) == 1, f"K={k}: exactly one trigger expected (native {nat}, reference {on_ref})"
        i_native = int(round(nat[0] * fs_dec)) - 1 - m_step + 1
        record_property(f"K{int(k)}_exact_sample", exact)
        record_property(f"K{int(k)}_native_sample", i_native)
        record_property(f"K{int(k)}_native_s", i_native / fs_dec)
        assert abs(i_native - exact) <= 1, (
            f"K={k}: native trigger at step sample {i_native} ({i_native / fs_dec:.2f} s) vs exact discrete recursion "
            f"{exact} ({exact / fs_dec:.2f} s), ±1 sample (SEI-003, plan §2b item 7)"
        )
        assert _ref_times(on_ref) == nat, f"K={k}: native onset {nat} vs f64 reference front end {_ref_times(on_ref)} (exact)"


# ---------------------------------------------------------------------------------
# SEI-004
# ---------------------------------------------------------------------------------

#: (M, R_hyp km): synthetic M6–7 records far enough that S−P (0.119 s/km) exceeds the 3 s Pd window plus the
#: pick delay, so the detector window holds P only (SEI-004; the Wu–Zhao anchor M6.5 at 30 km is 0.1762 cm).
PD_CASES = ((6.0, 30.0), (6.5, 30.0), (7.0, 40.0))
PD_SEEDS = 3
PD_TOL = 0.20


@pytest.mark.req("FAB-SEI-004")
def test_pd_recovery(native, record_property):
    """Contract: on synthetic M6–7 records whose P phase follows Wu–Zhao at hypocentral R, the detector's 3 s Pd
    recovers the Wu–Zhao value within ±20 % in every run (FAB-SEI-004, status S)."""
    depth = 10.0
    cfg = config.RunConfig(rail_class=REF_CLASS, seed=42, coil="voltage", duration_s=40.0)
    p = cfg.params()
    jobs, meta = [], []
    for mag, r_hyp in PD_CASES:
        r_epi = math.sqrt(r_hyp * r_hyp - depth * depth)
        assert r_hyp * (1.0 / pwave.V_S_KMPS - 1.0 / pwave.V_P_KMPS) > 3.3, f"M{mag} R{r_hyp}: S−P must exceed the 3 s window"
        for member in ensemble.member_seeds(cfg.seed, PD_SEEDS):
            d, series = scenarios.spec(
                "pwave_m7_r20", p, cfg, member=member, options={"magnitude": mag, "r_epi_km": r_epi, "depth_km": depth}
            )
            d["record"]["channels"] = ["pd_m", "tau_c_s"]
            d["record"]["rate_hz"] = 100.0
            jobs.append((p, native.Scenario.from_json(scenarios.to_json(d), series)))
            meta.append((mag, r_hyp, member.index, scenarios.info(d)))
    results = [RunResult.from_native(r) for r in native.run_batch(jobs, None)]
    bad = []
    for (mag, r_hyp, idx, info), r in zip(meta, results, strict=True):
        target = native.pd_wu_zhao(mag, r_hyp)
        assert target == pytest.approx(info["pd_target_m"], rel=1e-9), "scenario Pd target must be Wu–Zhao at R_hyp"
        pd_m = float(r["pd_m"][-1])
        ratio = pd_m / target
        record_property(f"M{mag}_R{r_hyp:g}_seed{idx}_pd_ratio", round(ratio, 4))
        if abs(ratio - 1.0) > PD_TOL:
            bad.append(f"M{mag} R{r_hyp:g} member {idx}: Pd {pd_m * 100:.4f} cm vs {target * 100:.4f} cm (ratio {ratio:.3f})")
    assert not bad, f"Pd recovery outside ±{PD_TOL:.0%} of Wu–Zhao at hypocentral R (SEI-004; PHYSICS §19–20): {bad}"


# ---------------------------------------------------------------------------------
# SEI-006
# ---------------------------------------------------------------------------------


@pytest.mark.slow
@pytest.mark.req("FAB-SEI-006")
def test_noise_24h_no_protect(native, record_property):
    """Contract: 24 h of synthetic site noise (MEMS noise, footsteps, slams, HVAC, self-actuation) streamed through
    DetectorReplay in 10-minute chunks produces 0 PROTECT; the raw-trigger count is reported (FAB-SEI-006,
    scenario 1, slow tier)."""
    p = native.Params.preset(REF_CLASS)
    det = native.DetectorReplay(p, FS_ODR)
    rng = np.random.default_rng(np.random.SeedSequence(42))
    first_protect = None
    t0 = time.perf_counter()
    for chunk in pwave.site_noise_chunks(rng, 86_400.0, FS_ODR, chunk_s=600.0):
        ev = det.feed(*(np.ascontiguousarray(chunk[ax], dtype="<f8").tobytes() for ax in range(3)))
        if first_protect is None:
            first_protect = next(((t, a) for t, kind, a, _ in ev if kind == "Protect"), None)
    wall = time.perf_counter() - t0
    rep = det.report()
    record_property("raw_triggers", int(rep["raw_triggers"]))
    record_property("freeze_count", int(rep["freeze_count"]))
    record_property("protect_count", int(rep["protect_count"]))
    record_property("level_max", int(rep["level_max"]))
    record_property("wall_s", round(wall, 1))
    assert rep["duration_s"] == pytest.approx(86_400.0), f"streamed {rep['duration_s']} s, expected 24 h"
    assert rep["protect_count"] == 0, (
        f"{int(rep['protect_count'])} PROTECT in 24 h of site noise (first at {first_protect}, level_max "
        f"{int(rep['level_max'])}, raw triggers {int(rep['raw_triggers'])}); required 0 (SEI-006, SEISMIC §2.3). "
        "A PROTECT latches INSPECT, which needs a manual reset, so later false PROTECT events are masked: this is a lower bound"
    )


def _bypass_job(native, burst_mps2: float, veto: bool = True):
    """WALL M7 R20 (seed 42), levitated and parked; a land command at t_P holds the actuator veto over the P pick
    (``veto``); a 3 Hz horizontal burst of ``burst_mps2`` starts 0.66 s after t_P, while the landing (and the veto)
    is still on."""
    cfg = config.RunConfig(rail_class=REF_CLASS, seed=42, coil="voltage")
    p = cfg.params()
    d, series = scenarios.spec("pwave_m7_r20", p, cfg)
    info = scenarios.info(d)
    t_p = float(info["t_p_s"])
    land = [{"kind": "command", "t_s": t_p, "cmd": "land"}] if veto else []
    d["events"] = sorted(
        [e for e in d["events"] if e["kind"] not in ("command", "hold_to_run")] + land,
        key=lambda e: e["t_s"],
    )
    meta = d["series"]["floor"]
    floor = np.frombuffer(series["floor"], dtype="<f8").reshape(len(meta["channels"]), -1).copy()
    tt = float(meta.get("t0_s", 0.0)) + np.arange(floor.shape[1]) / float(meta["fs_hz"])
    t_b = t_p + 0.66
    on = (tt >= t_b) & (tt < t_b + 2.0)
    floor[0] += np.where(on, burst_mps2 * np.sin(2.0 * math.pi * 3.0 * (tt - t_b)), 0.0)
    series = {**series, "floor": np.ascontiguousarray(floor, dtype="<f8").tobytes()}
    d["record"]["channels"] = ["seismic_state", "lev_state", "level"]
    d["record"]["rate_hz"] = 1000.0
    return p, d, series, floor, float(meta["fs_hz"]), t_p, t_b


@pytest.mark.req("FAB-SEI-006")
def test_pga_bypass_under_veto(native, record_property):
    """Contract: while the rail's own actuation holds the actuator veto (here a landing sequence), a P pick is
    suppressed, but horizontal shaking that reaches 80 gal bypasses the veto and reaches PROTECT within 100 ms
    (FAB-SEI-006, scenario 11)."""
    jobs, metas = [], []
    for amp in (1.2, 0.5):  # above / below the 0.8 m/s² (80 gal) bypass
        p, d, series, floor, fs, t_p, t_b = _bypass_job(native, amp)
        jobs.append((p, native.Scenario.from_json(scenarios.to_json(d), series)))
        metas.append((amp, p, floor, fs, t_p, t_b))
    # Control without the veto: the same record freezes at the P pick, so the suppression above is the veto's.
    p0, d0, s0, *_ = _bypass_job(native, 0.5, veto=False)
    jobs.append((p0, native.Scenario.from_json(scenarios.to_json(d0), s0)))
    runs = [RunResult.from_native(r) for r in native.run_batch(jobs, None)]
    free = runs.pop()
    t_p0 = float(scenarios.info(d0)["t_p_s"])
    t_free = free.t_s[np.nonzero(free["seismic_state"] != SEIS_NORMAL)[0]]
    record_property("no_veto_first_state_change_minus_tP_s", round(float(t_free[0]) - t_p0, 3) if t_free.size else None)
    assert t_free.size and t_p0 <= t_free[0] <= t_p0 + 0.5, (
        f"without the veto the P pick must freeze within 0.5 s of t_P (first state change {t_free[:1]}); control for SEI-006"
    )
    for (amp, p, floor, fs, t_p, t_b), r in zip(metas, runs, strict=True):
        assert fs == FS_ODR, "pwave records are synthesised at the ADXL355 ODR"
        seis, lev = r["seismic_state"], r["lev_state"]
        det = _detector(p)
        # Horizontal band-passed PGA of the input, reference front end (the detector's PGA definition, PHYSICS §20).
        lsb = p.get("design.sensors.accel_mps2_per_lsb")
        bp = _front_end(np.round(floor / lsb).astype(np.int64), lsb, det)
        a_h = np.hypot(bp[0], bp[1])
        pga_veto = p.get("control.seismic.detector.pga_veto_mps2")
        assert pga_veto == pytest.approx(0.8), "80 gal bypass (core design §C.4)"
        idx = np.nonzero(a_h >= pga_veto)[0]
        pick_win = (r.t_s >= t_p + 0.05) & (r.t_s < t_b)
        assert np.all(lev[pick_win] == LEV_LANDING), f"burst {amp} m/s²: the landing (actuator veto) must span the P pick"
        if amp > pga_veto:
            assert idx.size, "the burst must reach 80 gal after the band-pass"
            t_cross = (idx[0] + 1) * 0.01  # release that sees the first band-passed sample ≥ 80 gal
            record_property("t_cross_minus_tP_s", round(t_cross - t_p, 3))
            # The simulated ADXL355 adds its noise, so the native crossing may lead the noise-free reference by a
            # release or two: the "vetoed" window ends NOISE_LEAD_S before the reference crossing.
            early = (r.t_s >= t_p) & (r.t_s < t_cross - NOISE_LEAD_S)
            assert np.all(seis[early] == SEIS_NORMAL), (
                f"the P pick must be vetoed: seismic state left NORMAL at "
                f"{r.t_s[early][seis[early] != SEIS_NORMAL][0]:.3f} s, before the 80 gal crossing at {t_cross:.3f} s"
            )
            assert r.at("lev_state", t_cross) == LEV_LANDING, "the veto (landing) must still hold when 80 gal is reached"
            reached = np.nonzero((r.t_s >= t_cross - NOISE_LEAD_S) & (seis >= SEIS_PROTECT))[0]
            assert reached.size, f"no PROTECT after the 80 gal crossing at {t_cross:.2f} s (SEI-006 PGA bypass)"
            latency = max(0.0, float(r.t_s[reached[0]] - t_cross))
            record_property("bypass_latency_s", round(latency, 3))
            assert latency <= 0.100, (
                f"PROTECT {latency * 1e3:.0f} ms after 80 gal under the veto; required ≤ 100 ms (SEI-006, SEI-009 budget)"
            )
        else:
            assert not idx.size, f"control burst {amp} m/s² must stay below 80 gal after the band-pass"
            window = (r.t_s >= t_p) & (r.t_s < t_b + 2.0)
            assert np.all(seis[window] == SEIS_NORMAL), (
                f"burst {amp} m/s² (< 80 gal) under the veto: the vetoed pick must not freeze or PROTECT (SEI-006 veto)"
            )
