"""Contract: synthetic ground motion, P+S records, site noise, record loaders and the scenario catalog.

Covers FAB-SEI-018 (Kanai–Tajimi / Clough–Penzien PSDs within ±15 %, exact PGA after
scaling, the ASCE 7-16 spectrum and seed determinism) and FAB-SEI-020 (floor
amplification 1 + 2z/h), plus the core §E.3 catalog tests of this file
(``boore_corner_frequency``, ``jht_envelope``, ``elcentro_surrogate``,
``record_loaders``; the real record is ``requires_data``) and the A9 done-criterion:
every ``CATALOG`` entry builds a schema-valid ``fabrail.scenario/1`` spec. Pure
numpy/scipy except the catalog tests, which need ``fabrail._native`` for ``Params``
and skip without it. Tolerance classes follow core design §E.1: (E) exact algebra,
(S) statistical, (P) provisional.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np
import pytest
from scipy import signal

from fabrail import ground, human, pwave, rail, records, scenarios
from fabrail.config import RunConfig
from fabrail.ensemble import member_seeds, split_seed

G = ground.G0_MPS2


def _native_or_skip():
    try:
        from fabrail import _native
    except ImportError:
        pytest.skip("fabrail._native is not built (maturin develop --release)")
    return _native


# ---------------------------------------------------------------------------------
# FAB-SEI-018 / FAB-SEI-020
# ---------------------------------------------------------------------------------


# req: FAB-SEI-018
@pytest.mark.req("FAB-SEI-018")
def test_kt_cp_psd():
    """KT/CP closed forms, the synthesised PSD against them (±15 %), and exact PGA scaling."""
    p = ground.SOIL_FILTERS["medium"]
    fg = p.omega_g_rad_s / (2.0 * math.pi)
    # Closed forms (E): G_KT(0) = s0, G_KT(f_g) = s0·(1 + 4ζ²)/(4ζ²), G_CP(0) = 0.
    assert ground.kanai_tajimi_psd(np.array([0.0]), "medium", 2.5)[0] == pytest.approx(2.5, rel=1e-12), (
        "(E) rel 1e-12, KT at f = 0, PHYSICS §19"
    )
    peak = (1.0 + 4.0 * p.zeta_g**2) / (4.0 * p.zeta_g**2)
    assert ground.kanai_tajimi_psd(np.array([fg]))[0] == pytest.approx(peak, rel=1e-12), "(E) rel 1e-12, KT at f_g, PHYSICS §19"
    assert ground.clough_penzien_psd(np.array([0.0]))[0] == 0.0, "(E) CP high-pass removes the DC PSD, PHYSICS §19"
    f = np.array([0.1, 0.5, 2.0, 10.0])
    w = 2.0 * np.pi * f
    wf2 = p.omega_f_rad_s**2
    hp = w**4 / ((wf2 - w**2) ** 2 + 4.0 * p.zeta_f**2 * wf2 * w**2)
    np.testing.assert_allclose(
        ground.clough_penzien_psd(f),
        ground.kanai_tajimi_psd(f) * hp,
        rtol=1e-12,
        err_msg="(E) rel 1e-12, CP = KT × high-pass, PHYSICS §19",
    )

    # Synthesis (S): Welch PSD of 600 s stationary samples, band-averaged, within ±15 %
    # of the target over 0.5–15 Hz (FAB-SEI-018; CRIT G11).
    rng = np.random.default_rng(20260926)
    bands = [(0.5, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 5.0), (5.0, 8.0), (8.0, 12.0), (12.0, 15.0)]
    for model, psd in (("kt", ground.kanai_tajimi_psd), ("cp", ground.clough_penzien_psd)):
        x = ground.stationary(rng, 600.0, 100.0, "medium", model)
        fw, pw_ = signal.welch(x, fs=100.0, nperseg=1024)
        target = psd(fw, "medium")
        for lo, hi in bands:
            sel = (fw >= lo) & (fw < hi)
            ratio = float(pw_[sel].mean() / target[sel].mean())
            assert abs(ratio - 1.0) <= 0.15, f"(S) ±15 % PSD match, {model} {lo}-{hi} Hz: ratio {ratio:.3f} (FAB-SEI-018)"

    # Exact PGA after numeric scaling (E) and the baseline correction keeps it at rest.
    for model in ("kt", "cp"):
        a = ground.synthesize(np.random.default_rng(7), 40.0, 200.0, "medium", 0.5, model=model)
        assert float(np.max(np.abs(a))) == pytest.approx(0.5 * G, rel=1e-12), (
            f"(E) rel 1e-12, PGA scaled numerically ({model}), core §C.5"
        )
        v = ground.trapezoid_integral(a, 200.0)
        d = ground.trapezoid_integral(v, 200.0)
        assert abs(v[-1]) <= 1e-9 * float(np.max(np.abs(v))), (
            f"(E) end velocity zero after baseline correction ({model}), PHYSICS §19"
        )
        assert abs(d[-1]) <= 1e-9 * float(np.max(np.abs(d))), (
            f"(E) end displacement zero after baseline correction ({model}), PHYSICS §19"
        )


# req: FAB-SEI-018
@pytest.mark.req("FAB-SEI-018")
def test_asce_spectrum():
    """ASCE 7-16 §11.4.6 values and continuity (1e-12); spectrum-matched records follow it (0.9–1.3)."""
    # Anchors (E): S_DS 1.0, S_D1 0.6, T_L 8 s → T0 0.12, T_S 0.6, Sa(0) 0.4, Sa(1) 0.6,
    # Sa(10) 0.048 (physics perspective §3 [calc]; core §E.5 A22).
    for t, sa in ((0.0, 0.4), (0.12, 1.0), (0.3, 1.0), (0.6, 1.0), (1.0, 0.6), (8.0, 0.075), (10.0, 0.048)):
        assert ground.asce7_sa(t) == pytest.approx(sa, rel=1e-12, abs=1e-15), f"(E) rel 1e-12, ASCE 7-16 Sa({t}) = {sa}"
    for t_edge in (0.12, 0.6, 8.0):
        left, right = ground.asce7_sa(t_edge * (1 - 1e-13)), ground.asce7_sa(t_edge * (1 + 1e-13))
        assert abs(left - right) <= 1e-12, f"(E) 1e-12, ASCE spectrum continuous at T = {t_edge} s"
    assert isinstance(ground.asce7_sa(1.0), float), "scalar in, scalar out"

    # Spectrum matching (S): mean of 20 matched records within 0.9–1.3 × target over
    # 0.2–2 s (physics perspective §4.1 A22).
    periods = np.geomspace(0.05, 4.0, 30)
    target = ground.asce7_sa(periods)
    spectra = []
    for child in np.random.SeedSequence(42).spawn(20):
        rng = np.random.default_rng(child)
        a = ground.synthesize(rng, 20.0, 100.0, "medium", 0.4)
        a = ground.spectrum_match(rng, a, 100.0, target, periods, iterations=8)
        spectra.append(ground.response_spectrum(a, 100.0, periods) / G)
    ratio = np.mean(spectra, axis=0) / target
    sel = (periods >= 0.2) & (periods <= 2.0)
    assert np.all((ratio[sel] >= 0.9) & (ratio[sel] <= 1.3)), (
        f"(S) 0.9–1.3 × ASCE target over 0.2–2 s, got {np.round(ratio[sel], 3)} (A22)"
    )


# req: FAB-SEI-018
@pytest.mark.req("FAB-SEI-018")
def test_deterministic():
    """Same seed → bit-identical synthesis; different seed → different records (core §A.4 rule 5)."""

    def draw(seed: int) -> list[np.ndarray]:
        rng = np.random.default_rng(seed)
        z, y = rail.irregularity_profile(rng, 3.0, 1.2)
        ev = pwave.boore_ps(rng, 6.0, 20.0, 10.0, duration_s=45.0)
        return [
            ground.synthesize(rng, 20.0, 100.0),
            ground.synthesize(rng, 20.0, 100.0, model="kt"),
            ev.a_xyz_mps2,
            pwave.site_noise(rng, 30.0, 250.0),
            records.elcentro_like(rng).a_mps2,
            z,
            y,
        ]

    first, again, other = draw(42), draw(42), draw(43)
    for a, b, c in zip(first, again, other, strict=True):
        assert np.array_equal(a, b), "(E) bit-identical output for the same seed (FAB-SEI-018)"
        assert not np.array_equal(a, c), "different seeds give different records"
    synth, noise_seed = split_seed(42)
    assert (synth.entropy, synth.spawn_key) == (42, (0,)), "synthesis stream = SeedSequence(seed).spawn(2)[0] (core §A.4)"
    assert noise_seed == int(np.random.SeedSequence(42).spawn(2)[1].generate_state(1, np.uint64)[0]), (
        "Rust noise_seed = child 1 (core §A.4)"
    )


# req: FAB-SEI-020
@pytest.mark.req("FAB-SEI-020")
def test_floor_amplification():
    """Floor amplification 1 + 2z/h (ASCE 7-16 eq. 13.3-1), and the first-mode floor filter limits."""
    for zh, amp in ((0.0, 1.0), (0.5, 2.0), (1.0, 3.0), (0.25, 1.5)):
        assert ground.floor_amplification(zh) == pytest.approx(amp, rel=1e-12), (
            f"(E) rel 1e-12, 1 + 2z/h at z/h = {zh} (ASCE 7-16 eq. 13.3-1)"
        )
    for bad in (-0.1, 1.1):
        with pytest.raises(ValueError):
            ground.floor_amplification(bad)
    # The BuildingFilter option: at the base (z/h = 0) the floor is the ground; at
    # resonance the roof amplifies (Γ·φ(h) = 4/π for a uniform shear building).
    fs, t1 = 200.0, 0.5
    t = np.arange(int(40 * fs)) / fs
    a_g = 0.1 * np.sin(2.0 * np.pi * t / t1) * np.clip(t / 5.0, 0.0, 1.0)
    assert np.array_equal(ground.floor_response(a_g, fs, t1, z_over_h=0.0), a_g), "(E) z/h = 0 is the ground itself"
    roof = ground.floor_response(a_g, fs, t1, zeta=0.05, z_over_h=1.0)
    gain = float(np.max(np.abs(roof[-int(5 * fs) :])) / 0.1)
    # At ω = ω_n the SDOF absolute acceleration is (1 − i/(2ζ))·a_g, so a_f/a_g = 1 − iΓ/(2ζ).
    expected = abs(1.0 - 1j * ground.SHEAR_BEAM_GAMMA / (2.0 * 0.05))
    assert gain == pytest.approx(expected, rel=0.03), (
        f"(I) 3 %, steady resonant roof gain {gain:.2f} vs {expected:.2f} (Chopra first mode)"
    )


# ---------------------------------------------------------------------------------
# Core §E.3 catalog tests of this file (no SPEC requirement row)
# ---------------------------------------------------------------------------------


def test_boore_corner_frequency():
    """Brune corner at Δσ 30 bar: 0.75 / 0.24 / 0.075 Hz for M5 / M6 / M7; Pd follows Wu–Zhao at hypocentral R."""
    for m, fc in ((5.0, 0.75), (6.0, 0.24), (7.0, 0.075)):
        got = pwave.brune_corner_hz(m, stress_drop_bar=30.0)
        assert got == pytest.approx(fc, rel=0.01), f"(P) 1 %, f_c(M{m}) = {got:.4f} Hz (physics perspective §3 [calc])"
    m0 = pwave.seismic_moment_dyne_cm(6.5)
    assert pwave.brune_corner_hz(6.5) == pytest.approx(4.906e6 * 3.5 * (30.0 / m0) ** (1.0 / 3.0), rel=1e-12), (
        "(E) Boore 2003 eq. 4"
    )
    assert m0 == pytest.approx(10.0 ** (1.5 * 6.5 + 16.05), rel=1e-12), "(E) Hanks & Kanamori 1979"
    # Wu–Zhao anchors (core §E.5): M7 R20 0.7118, M6.5 R30 0.1762, M5 R10 0.0643, M7.8 R100 0.2987 cm.
    for m, r, pd_cm in ((7.0, 20.0, 0.7118), (6.5, 30.0, 0.1762), (5.0, 10.0, 0.0643), (7.8, 100.0, 0.2987)):
        assert 100.0 * pwave.wu_zhao_pd_m(m, r) == pytest.approx(pd_cm, rel=1e-3), (
            f"(E) 1e-3 (anchor rounding), Wu–Zhao M{m} R{r}"
        )
    # Plan §2b item 6: the P phase's 3 s Pd (the detector chain, P alone) equals Wu–Zhao at
    # the HYPOCENTRAL distance; S arrives at R/V_s − R/V_p after P.
    ev = pwave.boore_ps(np.random.default_rng(5), 7.0, 60.0, 12.0, duration_s=80.0)
    r = math.hypot(60.0, 12.0)
    assert ev.meta["pd_target_m"] == pytest.approx(pwave.wu_zhao_pd_m(7.0, r), rel=1e-12), (
        "(E) Pd target at hypocentral R (plan §2b.6)"
    )
    p_only = ev.a_xyz_mps2[2].copy()
    p_only[int(math.ceil(ev.t_s_s * ev.fs_hz - 1e-9)) :] = 0.0  # S starts after the 3 s window here (S−P 7.3 s)
    pd, tau_c = pwave.pd_tauc(p_only, ev.fs_hz, ev.t_p_s)
    assert pd == pytest.approx(ev.meta["pd_target_m"], rel=1e-9), "(E) 1e-9, 3 s Pd of the P phase equals Wu–Zhao (plan §2b.6)"
    assert ev.t_s_s - ev.t_p_s == pytest.approx(r / 3.5 - r / 6.0, rel=1e-12), "(E) S−P = R/V_s − R/V_p (SEI-012)"
    assert ev.meta["origin_t_s"] == pytest.approx(ev.t_p_s - r / 6.0, rel=1e-12), "(E) origin = t_P − R/V_p"
    assert tau_c == pytest.approx(pwave.tau_c_wu_kanamori_s(7.0), rel=0.02), "(P) 2 %, P τc calibrated to Wu–Kanamori 2005"
    assert ev.a_xyz_mps2.shape == (3, int(80.0 * ev.fs_hz)), "three components at fs"
    assert np.all(ev.a_xyz_mps2[:, : int(ev.t_p_s * ev.fs_hz) - 1] == 0.0), (
        "(E) nothing before the P arrival (acausal precursor cut)"
    )


def test_jht_envelope():
    """JHT shape: (t/t1)² rise, flat strong phase, exp decay; defaults scale with duration from (2, 12, 0.2) at 40 s."""
    env = ground.resolve_jht(None, 40.0)
    assert (env.t_rise_s, env.t_strong_end_s, env.decay_per_s) == pytest.approx((2.0, 12.0, 0.2), rel=1e-12), (
        "(E) core §C.5 defaults"
    )
    half = ground.resolve_jht(None, 20.0)
    assert (half.t_rise_s, half.t_strong_end_s, half.decay_per_s) == pytest.approx((1.0, 6.0, 0.4), rel=1e-12), (
        "(E) time-scaled rule"
    )
    partial = ground.resolve_jht(ground.JHT(t_rise_s=3.0), 40.0)
    assert (partial.t_rise_s, partial.t_strong_end_s) == (3.0, 12.0), "given fields are kept"
    t = np.array([-1.0, 0.0, 1.0, 2.0, 7.0, 12.0, 17.0, 40.0])
    e = ground.jht_envelope(t, duration_s=40.0)
    expected = [0.0, 0.0, 0.25, 1.0, 1.0, 1.0, math.exp(-1.0), math.exp(-5.6)]
    np.testing.assert_allclose(e, expected, rtol=1e-12, atol=0.0, err_msg="(E) rel 1e-12, JHT 1968 envelope values")
    assert float(np.max(ground.jht_envelope(np.linspace(0, 40, 4001)))) == 1.0, "(E) peak 1"
    with pytest.raises(ValueError):
        ground.resolve_jht(ground.JHT(t_rise_s=5.0, t_strong_end_s=4.0))


def test_elcentro_surrogate():
    """The offline surrogate: medium soil, 0.34 g PGA exactly, 30 s, at rest at both ends, deterministic."""
    rec = records.elcentro_like(np.random.default_rng(1940))
    assert rec.source == "surrogate" and rec.sha256 == "", "labelled as a surrogate, no file hash"
    assert rec.a_mps2.ndim == 1 and rec.duration_s == pytest.approx(30.0, rel=1e-12), "(E) one component, 30 s"
    assert float(np.max(np.abs(rec.a_mps2))) == pytest.approx(0.34 * G, rel=1e-12), (
        "(E) rel 1e-12, PGA 0.34 g (physics perspective §3)"
    )
    assert rec.meta["soil"] == "medium" and rec.meta["baseline_hp_hz"] == 0.05, "medium soil, 0.05 Hz baseline"
    v = ground.trapezoid_integral(rec.a_mps2, rec.fs_hz)
    d = ground.trapezoid_integral(v, rec.fs_hz)
    assert abs(v[-1]) <= 1e-9 * rec.meta["pgv_mps"] and abs(d[-1]) <= 1e-9 * rec.meta["pgd_m"], (
        "(E) baseline-corrected: ends at rest"
    )
    assert float(np.max(np.abs(d))) == pytest.approx(rec.meta["pgd_m"], rel=1e-12), "PGD reported for SEI-013"
    assert np.array_equal(rec.a_mps2, records.elcentro_like(np.random.default_rng(1940)).a_mps2), "(E) deterministic per seed"


def test_record_loaders(tmp_path: Path):
    """CSV and AT2 loaders (units, time column, axes) and the SHA-256 trust-on-first-use fetch (file:// only)."""
    fs = 100.0
    a = np.sin(np.arange(200) / 10.0)
    # CSV with a header and a time column, in g, axes z,x.
    csv = tmp_path / "rec.csv"
    rows = ["t,az,ax"] + [f"{k / fs:.6f},{a[k]:.17g},{0.5 * a[k]:.17g}" for k in range(a.shape[0])]
    csv.write_text("\n".join(rows) + "\n")
    rec = records.load_csv(csv, columns="z,x", units="g")
    assert rec.fs_hz == pytest.approx(fs, rel=1e-6), "(E) fs from the time column"
    np.testing.assert_allclose(rec.a_mps2[2], a * G, rtol=1e-15, err_msg="(E) g → m/s² on z")
    np.testing.assert_allclose(rec.a_mps2[0], 0.5 * a * G, rtol=1e-15, err_msg="(E) second column on x")
    assert np.all(rec.a_mps2[1] == 0.0) and rec.sha256 == hashlib.sha256(csv.read_bytes()).hexdigest(), (
        "unlisted axis zero; file hash"
    )
    # Headerless single column with fs given, in gal.
    one = tmp_path / "one.txt"
    one.write_text("\n".join(f"{v:.17g}" for v in a))
    rec1 = records.load_csv(one, fs_hz=fs, columns="x", units="gal")
    assert rec1.a_mps2.shape == a.shape and np.allclose(rec1.a_mps2, 0.01 * a, rtol=1e-15), "(E) gal → m/s², shape (n,)"
    with pytest.raises(ValueError):
        records.load_csv(one, fs_hz=None, columns="x")  # needs a time column or fs
    # PEER AT2: 4 header lines, NPTS/DT, 5 values per line, units g.
    at2 = tmp_path / "elcentro_test.at2"
    body = [" ".join(f"{v:.7E}" for v in a[k : k + 5]) for k in range(0, a.shape[0], 5)]
    at2.write_text(
        "\n".join(
            [
                "PEER NGA STRONG MOTION DATABASE RECORD",
                "TEST EVENT, TEST STATION, 000",
                "ACCELERATION TIME SERIES IN UNITS OF G",
                f"NPTS=  {a.shape[0]}, DT=   .0100 SEC",
                *body,
            ]
        )
        + "\n"
    )
    rat = records.load_at2(at2)
    assert rat.fs_hz == pytest.approx(100.0, rel=1e-12) and rat.meta["npts"] == a.shape[0], "(E) NPTS/DT header"
    written = np.array([float(f"{v:.7E}") for v in a])
    np.testing.assert_allclose(rat.a_mps2, written * G, rtol=1e-15, err_msg="(E) AT2 values in g → m/s²")

    # Trust on first use, offline through a file:// URL.
    dest = tmp_path / "data"
    with pytest.raises(ValueError, match="never vendored"):
        records.fetch_elcentro(dest, url="")
    path = records.fetch_elcentro(dest, url=at2.as_uri())
    pins = (dest / records.PIN_FILE).read_text().split()
    assert pins == [hashlib.sha256(at2.read_bytes()).hexdigest(), path.name], "pinned on first use (sha256sum format)"
    loaded = records.load_elcentro(dest)
    assert loaded.sha256 == pins[0] and np.array_equal(loaded.a_mps2, rat.a_mps2), "load_elcentro verifies the pin"
    assert records.fetch_elcentro(dest, url=at2.as_uri()) == path, "re-fetch reuses the pinned file"
    at2.write_text(at2.read_text() + "  1.0\n")  # the source changed
    with pytest.raises(ValueError, match="does not match the pin"):
        records.fetch_elcentro(dest, url=at2.as_uri(), force=True)
    assert hashlib.sha256(path.read_bytes()).hexdigest() == pins[0], "a mismatching download never replaces the pinned file"
    assert json.loads((dest / "elcentro.json").read_text())["sha256"] == pins[0], "provenance recorded"


@pytest.mark.requires_data
def test_record_loaders_real_elcentro():
    """The fetched 1940 El Centro record (``python -m fabrail data fetch elcentro``): pinned, loadable, 0.3–0.4 g."""
    data = Path(__file__).resolve().parents[1] / "data"
    if not (data / "elcentro.json").exists():
        pytest.skip("requires_data: the El Centro record has not been fetched into data/")
    rec = records.load_elcentro(data)
    pga_g = float(np.max(np.abs(rec.a_mps2))) / G
    assert 0.2 <= pga_g <= 0.45, f"(P) the 1940 El Centro NS PGA is ≈ 0.32–0.36 g depending on digitisation (got {pga_g:.3f} g)"


# ---------------------------------------------------------------------------------
# Supporting checks of the synthesis modules
# ---------------------------------------------------------------------------------


def test_response_spectrum_exact():
    """Nigam–Jennings coefficients equal the first-order-hold matrix exponential; the recursion is exact."""
    for w, z, h in (
        (2 * np.pi / 0.1, 0.05, 0.01),
        (2 * np.pi / 2.0, 0.05, 0.005),
        (10.0, 0.7, 0.002),
        (2 * np.pi / 0.02, 0.0, 0.01),
    ):
        a, b = ground.nigam_jennings(w, z, h)
        a2, b2 = ground._nj_expm(w, z, h)
        np.testing.assert_allclose(a, a2, rtol=1e-9, atol=1e-12, err_msg="(E) N–J A = expm (Van Loan 1978)")
        np.testing.assert_allclose(b, b2, rtol=1e-9, atol=1e-12 * float(np.max(np.abs(b2))), err_msg="(E) N–J B = FOH expm")
    # A constant ground acceleration a0 (after the rest ramp) gives u → −a0/ω² (static
    # deflection), so PSA → a0 with ζ > 0 and the transient decayed... check the peak at 2a0 for ζ = 0.
    fs, t = 1000.0, 0.5
    a0 = np.ones(int(5 * fs))
    psa0 = ground.response_spectrum(a0, fs, np.array([t]), zeta=0.0)[0]
    assert psa0 == pytest.approx(2.0, rel=2e-3), "(I) 0.2 %, undamped step response peaks at 2× the static value"
    assert ground.response_spectrum(a0, fs, np.array([0.0]))[0] == 1.0, "(E) T = 0 gives the PGA"


def test_site_noise_chunks():
    """Site noise is chunk-invariant (the 24 h replay streams 10-minute chunks) and has the stated MEMS level."""
    whole = pwave.site_noise(np.random.default_rng(3), 900.0, 500.0)
    chunked = np.concatenate(list(pwave.site_noise_chunks(np.random.default_rng(3), 900.0, 500.0, chunk_s=137.1)), axis=1)
    np.testing.assert_allclose(chunked, whole, rtol=0.0, atol=1e-15, err_msg="(E) chunked == whole (plan §2b.13)")
    mems = pwave.site_noise(np.random.default_rng(4), 200.0, 500.0, False, False, False, False)
    sigma = pwave.ADXL355_NOISE_MPS2_PER_RTHZ * math.sqrt(250.0)
    assert np.std(mems, axis=1) == pytest.approx([sigma] * 3, rel=0.02), (
        "(S) 2 %, white noise at 22.5 µg/√Hz (ADXL355 data sheet)"
    )
    quiet = pwave.site_noise(np.random.default_rng(5), 3600.0, 250.0, mems=False)
    assert float(np.max(np.abs(quiet))) < 0.8, "building noise stays below the 80 gal PGA bypass (SEI-006 context)"


def test_rail_irregularity_profile():
    """Sag L/1150, joint steps 0.1–0.3 mm, exact 20 µm rms roughness (PHYSICS §10; LEV-010)."""
    z, y = rail.irregularity_profile(
        np.random.default_rng(9), 6.0, 1.5, roughness_rms_m=0.0, joint_step_m=0.0, lateral_step_m=0.0
    )
    assert z.shape == (6001,) and float(-z.min()) == pytest.approx(1.5 / 1150.0, rel=1e-5), "(E) sag δ = L/1150"
    assert np.all(y == 0.0) and np.all(z <= 0.0), "sag only drops the rail"
    rough, _ = rail.irregularity_profile(
        np.random.default_rng(9), 6.0, 1.5, deflection_ratio=1e12, joint_step_m=0.0, lateral_step_m=0.0
    )
    assert float(np.std(rough)) == pytest.approx(2e-5, rel=1e-3), "(E) 20 µm rms roughness (scaled numerically)"
    steps, lat = rail.irregularity_profile(
        np.random.default_rng(9), 12.0, 3.0, deflection_ratio=1e12, roughness_rms_m=0.0, joint_spacing_m=3.0
    )
    changes = np.abs(np.diff(steps))
    assert float(np.max(np.abs(steps))) <= 1.5 * 3e-4 + 1e-12, "bounded joint walk within ±1.5·joint_step (PHYSICS §10)"
    assert float(np.max(np.abs(lat))) <= 1.5 * 1e-4 + 1e-12 and np.any(changes > 0.0), "lateral steps bounded; joints present"


def test_human_events():
    """Hand, lean, obstacle and smoke events are schema-valid; lean rise ≥ 1 s and smoke ramp ≥ 2 s are enforced."""
    base = {
        "schema": "fabrail.scenario/1",
        "name": "x",
        "duration_s": 20.0,
        "noise_seed": 1,
        "fidelity": {"model": "carriage6", "coil": "lag"},
    }
    evs = [human.hand_profile(k, 20.0, 1.0, 0.1) for k in ("push", "push_release", "pull", "throw", "slam")]
    evs += [
        human.hand_profile("pull", 20.0, 1.0, 0.1, np.random.default_rng(1)),
        human.hand_profile("lean", 15.0, 1.0, 1.0),
        human.hand_profile("pinch", 0.0, 1.0, 0.1, x_m=0.02),
        human.lean(1.0, 15.0, 7.0),
        human.obstacle(0.0, "chest", 0.3, k_edge_n_per_m=1e4, dur_s=5.0),
        human.smoke_dp(1.0, 25.0, dur_s=8.0),
    ]
    scenarios.validate_spec({**base, "events": evs})
    scenarios.cross_check({**base, "events": evs})
    assert evs[0]["contact"] == "push" and evs[2]["contact"] == "grip", "push-only vs grip contact"
    assert evs[3]["f_pts_n"][-1] == 20.0, "a throw releases at speed (force still applied at the last point)"
    jit = evs[5]
    assert 0.05 <= jit["t_pts_s"][1] <= 0.15, "jittered rise stays within 50–150 ms (physics perspective §1.11)"
    assert all(lo <= jit["impedance"][k] <= hi for k, (lo, hi) in human.ARM_IMPEDANCE_RANGES.items()), "arm impedance in range"
    with pytest.raises(ValueError, match="plan §2b item 11"):
        human.lean(1.0, 15.0, 7.0, rise_s=0.5)
    with pytest.raises(ValueError, match="plan §2b item 11"):
        human.smoke_dp(1.0, 25.0, ramp_s=1.0)
    with pytest.raises(ValueError):
        human.hand_profile("pinch", 0.0, 1.0, 0.1)
    with pytest.raises(ValueError):
        human.obstacle(0.0, "elbow", 0.3)


def test_schema_validator_rejects():
    """The in-repo schema validator reports ``path: reason`` like the Rust loader."""
    ok = {
        "schema": "fabrail.scenario/1",
        "name": "x",
        "duration_s": 5.0,
        "noise_seed": 0,
        "fidelity": {"model": "heave1", "coil": "lag"},
    }
    scenarios.validate_spec(ok)
    cases = {
        "duration_s": {**ok, "duration_s": 0.0},
        "fidelity.model": {**ok, "fidelity": {"model": "rigid6", "coil": "lag"}},
        "bogus": {**ok, "bogus": 1},
        "events[0].kind": {**ok, "events": [{"kind": "teleport", "t_s": 1.0}]},
        "events[0].x_m": {**ok, "events": [{"kind": "command", "t_s": 1.0, "cmd": "go_to"}]},
        "events[0].rise_s": {**ok, "events": [{"kind": "lean", "t_s": 1.0, "f_n": 1.0, "dur_s": 1.0, "rise_s": -1.0}]},
        "noise_seed": {**ok, "noise_seed": True},
        "name": {**ok, "name": "bad name!"},
    }
    for path, bad in cases.items():
        with pytest.raises(ValueError, match=re.escape(path)):
            scenarios.validate_spec(bad)
    with pytest.raises(ValueError, match="events\\[0\\].t_s"):
        scenarios.cross_check({**ok, "events": [{"kind": "reset", "t_s": 6.0}]})
    with pytest.raises(ValueError, match="record.rate_hz"):
        scenarios.cross_check({**ok, "fidelity": {"model": "travel1d", "coil": "lag"}, "record": {"rate_hz": 2000.0}})


# ---------------------------------------------------------------------------------
# The scenario catalog (A9 done-criterion; needs _native for Params)
# ---------------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(scenarios.CATALOG))
def test_catalog_specs_schema_valid(name: str):
    """Every CATALOG entry, for every class it applies to, builds a schema-valid spec that the Rust loader accepts."""
    native = _native_or_skip()
    sdef = scenarios.CATALOG[name]
    assert sdef.classes and set(sdef.classes) <= {"drawer", "door", "wall", "heavy"} and sdef.doc, "classes and doc"
    assert sdef.tiers and set(sdef.tiers) <= {"carriage6", "heave1", "travel1d", "structure"}, "declared tiers"
    for cls in sdef.classes:
        cfg = RunConfig(rail_class=cls)
        params = cfg.params()
        d, series = scenarios.spec(name, params, cfg)
        assert scenarios.schema_errors(d) == [], f"{name}/{cls}: schema-valid (FAB-SW-015)"
        doc = json.loads(scenarios.to_json(d))
        assert doc["name"] == f"{cls}_{name}" and doc["fidelity"]["model"] == sdef.tiers[0], "name and recommended tier"
        scen = native.Scenario.from_json(scenarios.to_json(d), series)
        assert scen.name == d["name"] and scen.noise_seed == split_seed(42)[1], (
            "native Scenario accepts it; noise seed from the split"
        )
        assert set(scen.series_bytes()) == set(series), "series round-trip through the native loader"
    other = next((c for c in ("drawer", "door", "wall", "heavy") if c not in sdef.classes), None)
    if other is not None:
        with pytest.raises(ValueError, match="does not apply"):
            scenarios.spec(name, RunConfig(rail_class=other).params(), RunConfig(rail_class=other))


def test_catalog_contract():
    """The frozen core §B.8 names plus the plan §4b additions; tier resolution; determinism; coil single source."""
    assert set(scenarios.CATALOG) == set(scenarios.NAMES) | set(scenarios.PLAN_NAMES), "every frozen name is in CATALOG"
    assert {"terminus_soft_close", "soft_open", "choreographed_close", "inertial_smooth_fixedbase"} <= set(scenarios.CATALOG)
    for name in ("isolation_kt", "inertial_smooth_fixedbase", "harmonic_base"):
        assert scenarios.CATALOG[name].requires, f"{name} declares its research-tier parameter (never applied automatically)"
    assert scenarios.resolve_fidelity("pwave_m7_r20", RunConfig()) == "travel1d", "seismic default tier Travel1d"
    assert scenarios.resolve_fidelity("pwave_m7_r20", RunConfig(), "carriage6") == "carriage6", "explicit tier wins"
    assert scenarios.resolve_fidelity("thermal_ramp", RunConfig(fidelity="carriage6")) == "heave1", (
        "carriage6 is the default: defer"
    )
    with pytest.raises(ValueError, match="supports"):
        scenarios.resolve_fidelity("unseat", RunConfig(fidelity="travel1d"))
    _native_or_skip()
    cfg = RunConfig(rail_class="wall", coil="voltage", seed=7)
    params = cfg.params()
    a = scenarios.spec("pwave_m7_r20", params, cfg)
    b = scenarios.spec("pwave_m7_r20", params, cfg)
    assert scenarios.to_json(a[0]) == scenarios.to_json(b[0]) and a[1] == b[1], "(E) same seed → identical JSON and bytes"
    assert a[0]["fidelity"]["coil"] == "voltage", "the coil mode comes from cfg.coil (plan §2b.16)"
    info = scenarios.info(a[0])
    assert info["t_s_s"] - info["t_p_s"] == pytest.approx(info["r_hyp_km"] / 3.5 - info["r_hyp_km"] / 6.0, rel=1e-12), (
        "info carries t_P, t_S"
    )
    with pytest.raises(ValueError, match="coil"):
        scenarios.cross_check(a[0], a[1], RunConfig(rail_class="wall", coil="lag").params())
    m = member_seeds(7, 2)
    c = scenarios.spec("pwave_m7_r20", params, cfg, member=m[1])
    assert c[0]["noise_seed"] == m[1].noise_seed and c[1] != a[1], "ensemble members draw their own inputs"
    with pytest.raises(ValueError, match="unknown option"):
        scenarios.spec("hover", params, cfg, options={"nope": 1})
    short = scenarios.build_spec("pwave_m7_r20", params, RunConfig(rail_class="wall", coil="voltage", seed=7, duration_s=10.0))
    assert short["duration_s"] == 10.0 and all(e["t_s"] <= 10.0 for e in short["events"]), "duration override drops later events"


def test_run_ensemble_index_order(monkeypatch: pytest.MonkeyPatch):
    """run_ensemble builds member i from member_seeds(seed, n)[i] and keeps index order (worker-invariant)."""
    import fabrail
    from fabrail import ensemble

    _native_or_skip()
    captured: dict[str, object] = {}

    class FakeData:
        rate_hz = 10.0

        def __init__(self, seed: int) -> None:
            self.seed = seed

        def channel_names(self):
            return ["x_m"]

        def channel(self, name):
            return (np.array([float(self.seed % 1000)]).astype("<f8").tobytes(), "<f8", "m")

        def events(self):
            return []

        def kpis(self):
            return {"noise_seed_mod": float(self.seed % 1000)}

        def meta_json(self):
            return "{}"

    class FakeNative:
        Scenario = _native_or_skip().Scenario

        @staticmethod
        def run_batch(jobs, workers=None):
            captured["workers"] = workers
            return [FakeData(s.noise_seed) for _, s in jobs]

    cfg = RunConfig(rail_class="door", seed=11, workers=3, duration_s=1.0)
    params = cfg.params()
    monkeypatch.setattr(fabrail, "require_native", lambda: FakeNative)
    res = ensemble.run_ensemble(params, "hover", cfg, 4)
    seeds = [m.noise_seed for m in member_seeds(11, 4)]
    assert res.noise_seeds == seeds and captured["workers"] == 3, "member seeds in index order, workers passed through"
    assert list(res.kpi("noise_seed_mod")) == [float(s % 1000) for s in seeds], "results kept in index order"
    assert res.meta["fidelity"] == "carriage6" and len(res) == 4, "ensemble metadata"
