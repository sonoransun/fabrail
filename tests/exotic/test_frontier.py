"""Frontier tier (🔭): the FAB-FRN requirements of docs/FRONTIER.md §2, plus FAB-EXO-004.

Each test calls the numpy frontier models in :mod:`fabrail.exotic.frontier` and checks
them two ways where it can: against the closed form restated here from FRONTIER §F
(tolerance class (E), relative 1e-12), and against the rounded figure quoted by
FRONTIER §1/§2 (tolerance = half a unit of the quoted last digit, or the row's stated
tolerance). Research metrics are class (R). Every assertion message states its
tolerance and source (SPECIFICATION §0.8).

The two guards (FAB-EXO-004, FAB-FRN-008) call every public function of every model
submodule with its defaults and check the three output forms: stdout (banner first,
missing-orders figure printed), JSON (``"tier": "frontier"``) and the plot watermark.
safe_rl reads the pedestrian caps from ``fabrail._native``; its functions are skipped
from the guards when the native module is not built.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import importlib
import inspect
import json
import math
import re
from collections.abc import Callable, Iterator
from pathlib import Path

import numpy as np
import pytest

from fabrail.exotic import frontier
from fabrail.exotic.frontier import (
    BANNER_HEAD,
    MODEL_MODULES,
    TIER,
    cloak,
    diamagnetic,
    format_missing_orders,
    lunar,
    meta,
    metabarrier,
    pegs,
    qzs,
    rtsc,
    trapped_field,
)
from fabrail.exotic.frontier._record import FrontierRecord

REL_E = 1e-12
MU0 = 1.256_637_062_12e-6  # CODATA 2018 (fabrail_params::derive::MU0_H_PER_M)
G0 = 9.806_65  # CGPM 1901 (fabrail_params::derive::G0_MPS2)
SIGMA_SB = 5.670_374_419e-8  # CODATA 2018
ROOT = Path(__file__).resolve().parents[2]


def rel_err(a: float, b: float) -> float:
    return abs(a - b) / max(abs(a), abs(b), 1e-300)


def _native_built() -> bool:
    try:
        importlib.import_module("fabrail._native")
    except ImportError:
        return False
    return True


def public_functions() -> Iterator[tuple[str, Callable[[], FrontierRecord]]]:
    """Every public function of every model submodule (``__all__`` members defined there)."""
    for name in MODEL_MODULES:
        mod = importlib.import_module(f"fabrail.exotic.frontier.{name}")
        for attr in mod.__all__:
            obj = getattr(mod, attr)
            if inspect.isfunction(obj) and obj.__module__ == mod.__name__:
                yield f"{name}.{attr}", obj


def called_records() -> list[tuple[str, FrontierRecord]]:
    """Each public function called with its defaults (safe_rl skipped without the native module)."""
    out = []
    native = _native_built()
    for qual, fn in public_functions():
        if qual.startswith("safe_rl.") and not native:
            continue
        out.append((qual, fn()))
    return out


def test_public_function_inventory() -> None:
    names = {q for q, _ in public_functions()}
    assert len(names) >= 20, f"expected the frontier model surface (>= 20 functions), got {sorted(names)}"
    for module in MODEL_MODULES:
        assert any(q.startswith(module + ".") for q in names), f"model module {module} exports no public function"


@pytest.mark.req("FAB-FRN-001")
def test_rtsc_sweep() -> None:
    sw = rtsc.sweep()
    eta = sw["inputs"]["eta"]
    tc = sw["inputs"]["tc_k"]
    assert eta.min() == pytest.approx(0.15) and eta.max() == pytest.approx(1.0), "η sweep 0.15–1 (FRONTIER FAB-FRN-001)"
    assert tc.min() == 90.0 and tc.max() == 600.0, "Tc sweep 90–600 K (FRONTIER FAB-FRN-001)"
    assert sw["inputs"]["t_op_k"] == 300.0, "no cryogenics: operating at 300 K (FRONTIER FAB-FRN-001)"

    tc_req = sw["tc_required_k"]
    assert rel_err(tc_req, 300.0 * 92.0 / 77.0) <= REL_E, "(E) Tc = 300·92/77, FRONTIER §F2 / XCRIT §1"
    assert abs(tc_req - 358.0) <= 0.5, f"Tc ≈ 358 K ±0.5 (quoted, FRONTIER FAB-FRN-001): {tc_req}"

    ceiling = sw["ceiling_pa"]
    h, k, b0 = sw["inputs"]["h_m"], sw["inputs"]["k_per_m"], sw["inputs"]["b0_t"]
    assert rel_err(ceiling, b0 * b0 / MU0 * math.exp(-2.0 * k * h)) <= REL_E, "(E) P̄ = (B0²/μ0)·e^(−2kh), FRONTIER §F2"
    assert abs(ceiling / 1e6 - 0.51) <= 0.005, f"ceiling 0.51 MPa at 10 mm ±0.005 (quoted, FAB-FRN-001): {ceiling}"
    bound = rtsc.pressure_bound()
    assert abs(bound["lambda_m"] - 0.154) <= 0.0005, "λ = 154 mm ±0.5 mm (FAB-FRN-001)"
    assert abs(bound["p_contact_pa"] / 1e6 - 1.15) <= 0.005, "1.15 MPa at contact ±0.005 (exotic design §2.3)"

    p = sw["p_pa"]
    assert sw["rail_limited"] and np.all(p <= ceiling), (
        "(E) pressure never exceeds the rail-limited ceiling (FAB-FRN-001)"
    )
    assert sw["p_max_pa"] == ceiling, (
        "(E) the ceiling is reached (η = 1, Tc above 358 K) and not exceeded (FRONTIER §F2)"
    )
    assert np.all(p[:, tc <= 300.0] == 0.0), (
        "(E) no levitation at or below Tc = 300 K without cryogenics (FRONTIER §F2)"
    )
    above = tc >= tc_req
    assert np.all(p[-1, above] == ceiling), "(E) η = 1 at Tc ≥ 358 K sits on the ceiling (FRONTIER §F2)"
    assert np.all(p[-1, (tc > 300.0) & ~above] < ceiling), "(E) below 358 K the pinning is short of REBCO-at-77 K (§F2)"
    assert sw["missing_orders"] == round(math.log10(tc_req / rtsc.TC_BEST_AMBIENT_K), 1), (
        "missing orders log10(358/138)"
    )
    assert "358 K" in str(sw) and "rail-limited" in sw["banner"], (
        "stdout states Tc ≈ 358 K and the rail limit (FAB-FRN-001)"
    )


@pytest.mark.req("FAB-FRN-002")
def test_trapped_field_cap() -> None:
    tf = trapped_field.pressure_cap()
    b = tf["inputs"]["b_t"]
    eta = tf["inputs"]["eta"]
    assert b.min() == 3.0 and b.max() == 7.0, "B sweep 3–7 T (exotic design §2.3)"
    p_rail = tf["p_rail_pa"]
    assert rel_err(p_rail, 2.3**2 / (2.0 * MU0)) <= REL_E, "(E) rail cap B_sat²/2μ0 at 2.3 T (EXOTIC §X2)"
    assert abs(p_rail / 1e6 - 2.10) <= 0.005, f"rail cap 2.10 MPa ±0.005 (FAB-EXO-207, FAB-FRN-002): {p_rail}"
    raw = eta[:, None] * b[None, :] ** 2 / (2.0 * MU0)
    assert np.allclose(tf["p_raw_pa"], raw, rtol=REL_E, atol=0.0), "(E) p = η·B²/2μ0 (FRONTIER §F3)"
    assert np.all(tf["p_pa"] <= p_rail) and tf["rail_capped"], "(E) p ≤ 2.1 MPa against steel whatever B (FAB-FRN-002)"
    assert np.any(raw > p_rail), "the rail cap binds inside the 3–7 T sweep (FAB-FRN-002)"
    free = raw < p_rail
    assert np.allclose(tf["p_pa"][free], raw[free], rtol=REL_E), "(E) uncapped cells keep η·B²/2μ0 (FRONTIER §F3)"
    assert set(np.unique(tf["binding"][~free])) == {"rail"}, "capped cells name the rail (FRONTIER §F3)"

    record_b = trapped_field.pressure_cap(b_t=[trapped_field.B_TRAPPED_RECORD_T], eta=[1.0])
    assert record_b["p_max_pa"] <= p_rail, "(E) even the 17.6 T record bulk stays under the rail cap (FAB-FRN-002)"

    sigma = 1.0e6
    tm = trapped_field.pressure_cap(sigma_mech_pa=sigma)
    assert rel_err(tm["b_mech_t"], math.sqrt(2.0 * MU0 * sigma)) <= REL_E, "(E) B_mech = √(2μ0σ_mech) (FRONTIER §F3)"
    assert np.all(tm["p_pa"] <= sigma * (1.0 + REL_E)), "(E) p ≤ σ_mech when σ_mech binds (FAB-FRN-002)"
    assert set(np.unique(tm["binding"])) == {"sigma_mech"}, "σ_mech names the binding cap at 1 MPa (FAB-FRN-002)"


@pytest.mark.req("FAB-FRN-003")
def test_pegs_detection_and_gap() -> None:
    assert pegs.KAPPA_H == 4.6, "κ_h = 4.6 (FAB-FRN-003)"
    det = pegs.detection(1e-15, 7.0, 140.0)
    assert det["detected"] and det["before_p"], "Mw 7 at 140 km is detected before its P wave (FAB-FRN-003)"
    assert 9.9 <= det["t_det_s"] <= 10.0, (
        f"(R) detected within 10 s at 1e-15/√Hz (Shimoda 2021, FAB-FRN-003): {det['t_det_s']}"
    )
    assert det["gain_vs_onsite_p_s"] > 10.0, "(R) about 11.5 s over on-site P at 140 km (frontier research §3a)"
    assert det["gain_vs_network_s"] <= 0.5, (
        "(R) lead-time gain against a dense P network ≈ 0 s, ≤ 0.5 s (XCRIT §1, FAB-FRN-003): "
        f"{det['gain_vs_network_s']}"
    )

    gap = pegs.sensitivity_gap()
    assert abs(gap["asd_needed_per_rthz"] / 1e-15 - 1.0) <= 0.01, "(R) needs 1e-15/√Hz ±1 % (Shimoda 2021, FAB-FRN-003)"
    assert gap["asd_best_per_rthz"] == 1e-10, "best demonstrated 1e-10/√Hz (TOBA Phase-II, FAB-FRN-003)"
    assert gap["missing_orders"] == 5.0, f"5 orders short (FAB-FRN-003): {gap['missing_orders']}"
    assert abs(gap["kappa_fit"] - 4.6) <= 0.023, (
        f"(R) κ_h one-point fit 4.6 ±0.5 % (frontier research §3a): {gap['kappa_fit']}"
    )
    assert not gap["best_detects_before_p"], "(R) at 1e-10/√Hz Mw 7 at 140 km is never seen before P (FRONTIER §F4)"

    text = str(gap)
    assert "best demonstrated 1e-10/√Hz" in text and "5 orders" in text, "stdout states the gap (FAB-FRN-003)"
    assert "≈ 0 s" in det["banner"], "the banner states the ≈ 0 s network gain (FAB-FRN-003)"


@pytest.mark.req("FAB-FRN-004")
def test_metabarrier_mass_law() -> None:
    ml = metabarrier.mass_law(f_hz=1.0, v_s_mps=(100.0, 200.0))
    rho = ml["inputs"]["rho_kg_m3"]
    v = ml["inputs"]["v_s_mps"]
    assert np.allclose(ml["mass_kg"], rho * v**3 / 1.0**3, rtol=REL_E), "(E) M = ρ·v_s³/f³ (FRONTIER §F5)"
    assert abs(ml["mass_kg"][0] / 1.8e9 - 1.0) <= 0.05, (
        f"1.8 Mt at 1 Hz, v_s 100 m/s ±5 % (FAB-FRN-004): {ml['mass_kg'][0]}"
    )
    assert abs(ml["mass_kg"][1] / 14.4e9 - 1.0) <= 0.05, f"14.4 Mt at 200 m/s ±5 % (FAB-FRN-004): {ml['mass_kg'][1]}"
    sw = metabarrier.sweep(f_hz=[1.0, 2.0], v_s_mps=[100.0])
    assert rel_err(sw["mass_kg"][0, 0] / sw["mass_kg"][1, 0], 8.0) <= REL_E, "(E) mass ∝ f⁻³ (FRONTIER §F5)"
    for rec in (ml, sw, metabarrier.gap_edge(), metabarrier.unit_budget()):
        assert "v_s" in rec["banner"] and "m/s" in rec["banner"], "every metabarrier output states v_s (FAB-FRN-004)"
    assert "v_s = 100 m/s" in str(ml) and "v_s = 200 m/s" in str(ml), "stdout states v_s (FAB-FRN-004)"


@pytest.mark.req("FAB-FRN-005")
def test_qzs_stiffness_and_tilt() -> None:
    k1 = 1.0e5
    s = qzs.series_stiffness(k1_n_per_m=k1)
    ratio = s["inputs"]["km_over_k1"]
    k = s["k_series_n_per_m"]
    at = ratio == -1.0
    assert at.sum() == 1 and np.isinf(k[at]).all(), "(E) series stiffness → ∞ at k_m = −k1 (FAB-FRN-005)"
    assert s["sag_m"][at][0] == 0.0, "(E) no sag under a direct load at the singularity (FRONTIER §F6)"
    km = ratio[~at] * k1
    assert np.allclose(k[~at], k1 * km / (k1 + km), rtol=REL_E), "(E) k = k1·k_m/(k1 + k_m) (FRONTIER §F6)"
    left = np.abs(k[ratio < -1.0])  # ratio increasing toward −1
    right = np.abs(k[ratio > -1.0])  # ratio moving away from −1
    assert np.all(np.diff(left) > 0.0) and np.all(np.diff(right) < 0.0), "(E) |k| grows toward k_m = −k1 (FAB-FRN-005)"

    t = qzs.tilt_error(theta_rad=1e-3, f_hz=0.1)
    w = 2.0 * math.pi * 0.1
    assert rel_err(t["x_err_m"], G0 * 1e-3 / w**2) <= REL_E, "(E) x_err = g·θ/ω² (FRONTIER §F6)"
    assert abs(t["x_err_m"] * 1e3 - 25.0) <= 0.5, (
        f"≈ 25 mm at 0.1 Hz, 1e-3 rad ±0.5 mm (frontier research §3c): {t['x_err_m']}"
    )
    sweep = qzs.tilt_error(theta_rad=np.logspace(-9, -3, 7), f_hz=0.1)["x_err_m"]
    assert np.allclose(sweep, G0 * np.logspace(-9, -3, 7) / w**2, rtol=REL_E), "(E) linear in θ (FRONTIER §F6)"
    assert s["badge"] == "🧪" and "building" in s["banner"], (
        "QZS is 🧪 with the building-scale gap stated (FRONTIER §1)"
    )


@pytest.mark.req("FAB-FRN-006")
def test_selene_lunar_scaling(capsys: pytest.CaptureFixture[str]) -> None:
    s = lunar.selene_scaling()
    assert rel_err(s["pole_ratio"], math.sqrt(1.62 / G0)) <= REL_E, "(E) pole ×√(g_moon/g0) (FRONTIER §F7)"
    assert abs(s["pole_ratio"] - 0.406) <= 0.0005, f"pole ×0.406 ±0.0005 (FAB-FRN-006): {s['pole_ratio']}"
    assert abs(math.sqrt(1.62 / 9.81) - s["pole_ratio"]) <= 0.0005, (
        "√(1.62/9.81) and √(1.62/g0) agree to 0.406 (FAB-FRN-006)"
    )
    assert rel_err(s["q_rad_w_m2"], SIGMA_SB * 40.0**4) <= REL_E, "(E) q = σT⁴ at 40 K, ε = 1 (FRONTIER §F7)"
    assert abs(s["q_rad_w_m2"] - 0.145) <= 0.0005, f"0.145 W/m² at 40 K ±0.0005 (FAB-FRN-006): {s['q_rad_w_m2']}"
    s.emit()
    out = capsys.readouterr().out
    assert "nanophase" in out and "PM" in out and "guideways" in out, (
        "the nanophase-Fe dust caveat is printed (FAB-FRN-006)"
    )
    assert out.splitlines()[0] == s["banner"], "stdout starts with the 🔭 banner (FAB-FRN-006, FAB-EXO-004)"


@pytest.mark.req("FAB-FRN-007")
def test_cloak_permeability() -> None:
    c = cloak.cloak_permeability(r2_over_r1=(1.2, 2.0))
    q = np.array([1.2, 2.0]) ** 2
    assert np.allclose(c["mu_r"], (q + 1.0) / (q - 1.0), rtol=REL_E), "(E) μ = (R2² + R1²)/(R2² − R1²) (FRONTIER §F8)"
    assert abs(c["mu_r"][0] - 5.5) <= 0.05 and abs(c["mu_r"][1] - 1.67) <= 0.005, "μ 5.5 / 1.67 (frontier research §3e)"

    for r in (1.5, 2.0, 3.0, 10.0):
        for b0 in (0.25, 0.5, 1.0, 2.0):
            g = cloak.concentrator_gain(r2_over_r1=r, b0_t=b0)
            assert g["gain"] <= cloak.B_SAT_T / b0 * (1.0 + REL_E), "(E) concentrator gain ≤ B_sat/B0 (FAB-FRN-007)"
            assert g["gain"] == min(r, cloak.B_SAT_T / b0), "(E) gain = min(R2/R1, B_sat/B0) (FRONTIER §F8)"
            assert g["b_inside_t"] <= cloak.B_SAT_T * (1.0 + REL_E), "(E) the inside field never exceeds B_sat (§F8)"
    sat = cloak.concentrator_gain(r2_over_r1=3.0, b0_t=1.0)
    assert sat["saturation_limited"] and sat["gain"] == cloak.B_SAT_T, "(E) R2/R1 = 3 at 1 T saturates at 2.3 (§F8)"

    lab = cloak.cloak_permeability(r2_m=cloak.LAB_SCALE_M)
    big = cloak.cloak_permeability(r2_m=cloak.STRUCTURE_SCALE_M)
    assert lab["badge"] == "🧪" and lab["item"] == "Lab-scale magnetic cloak", "🧪 at lab scale (FAB-FRN-007)"
    assert big["badge"] == "🔭" and big["item"].startswith("Structure-scale"), "🔭 at structure scale (FAB-FRN-007)"
    assert lab["missing_orders"] == 0.0 and big["missing_orders"] == 2.0, "missing orders log10(1 m / 1 cm) (§F8)"


@pytest.mark.req("FAB-FRN-008")
def test_missing_orders_printed(capsys: pytest.CaptureFixture[str]) -> None:
    records = called_records()
    assert records, "no frontier output to check"
    for qual, rec in records:
        figure = format_missing_orders(rec["missing_orders"])
        rec.emit()
        out = capsys.readouterr().out
        assert f"Missing: {figure}." in out.splitlines()[0], f"{qual}: banner prints the missing orders (FAB-FRN-008)"
        assert out.rstrip().splitlines()[-1].startswith(f"missing: {figure}"), (
            f"{qual}: last line repeats it (FAB-FRN-008)"
        )
        assert figure in rec.watermark, f"{qual}: the watermark carries the missing orders (FAB-FRN-008)"
    for card in meta.frontier_cards():
        assert isinstance(card["missing_orders"], float), (
            f"{card['item']}: card missing_orders is a number (FAB-FRN-008)"
        )
        assert f"Missing: {format_missing_orders(card['missing_orders'])}." in card["banner"], (
            f"{card['item']}: card banner prints the missing orders (FAB-FRN-008)"
        )


@pytest.mark.req("FAB-FRN-009")
def test_diamagnetic_pressure() -> None:
    d = diamagnetic.pressure(b_t=1.0)
    chi = d["inputs"]["chi"]
    assert rel_err(d["p_pa"], abs(chi) * 1.0 / (2.0 * MU0)) <= REL_E, "(E) p = abs(χ)·B²/2μ0 (FRONTIER §F9)"
    assert abs(d["p_pa"] - 179.0) <= 0.5, f"≈ 179 Pa at 1 T ±0.5 (FAB-FRN-009): {d['p_pa']}"
    assert d["upper_bound"] and "upper bound" in d["banner"], "stated as an upper bound (FAB-FRN-009)"
    assert d["float_consistent"], "FLOAT's 33 kg/m² fits under lunar gravity (gross ≈ 110 kg/m²; FAB-FRN-009)"
    assert rel_err(d["gross_kg_m2"], d["p_pa"] / 1.62) <= REL_E, "(E) gross areal mass p/g_moon (FRONTIER §F9)"
    assert d["gross_kg_m2"] >= 33.0 > d["gross_earth_kg_m2"], "33 kg/m² fits the Moon, not Earth (FRONTIER §F9)"
    assert d["badge"] == "✅ physics" and "FLOAT research" in d["maturity"], "✅ physics, FLOAT research (FRONTIER §1)"


@pytest.mark.req("FAB-FRN-010")
def test_matrix_matches_code() -> None:
    text = (ROOT / "docs" / "FRONTIER.md").read_text(encoding="utf-8")
    section = text.split("## §1 Frontier matrix", 1)[1].split("\n---", 1)[0]
    table = [ln.strip() for ln in section.splitlines() if ln.strip().startswith("|")]
    assert len(table) >= 3, "FRONTIER §1 has a table"

    def cells(line: str) -> tuple[str, ...]:
        return tuple(c.strip() for c in line.strip().strip("|").split("|"))

    assert cells(table[0]) == meta.MATRIX_HEADER, "FRONTIER §1 header matches the code (FAB-FRN-010)"
    doc_rows = [cells(ln) for ln in table[2:]]
    code_rows = meta.matrix_rows()
    assert [r[0] for r in doc_rows] == [r[0] for r in code_rows], "same items in the same order (FAB-FRN-010)"
    for doc, code in zip(doc_rows, code_rows):
        assert doc == code, f"FRONTIER §1 row drifted from the code (FAB-FRN-010): doc {doc} vs code {code}"
    assert meta.render_matrix() == "\n".join(table), "render_matrix() regenerates FRONTIER §1 exactly (FAB-FRN-010)"

    ids = set(re.findall(r"\| (FAB-FRN-\d{3}) \|", text))
    for row in code_rows:
        if row[5] != "—":
            assert row[5] in ids, f"{row[0]}: requirement {row[5]} exists in FRONTIER §2 (FAB-FRN-010)"
        if row[1] == "🔭":
            assert row[3] != "—", f"{row[0]}: a 🔭 row names its anchor tier (FAB-EXO-001)"
    cards = meta.frontier_cards()
    assert [c["item"] for c in cards] == [r[0] for r in code_rows], "one site card per matrix row (FAB-FRN-010)"
    assert meta.missing_orders("pegs") == 5.0, "the PEGS row's 5 orders equal the model figure (FAB-FRN-010)"


@pytest.mark.req("FAB-EXO-004")
def test_every_output_bannered(capsys: pytest.CaptureFixture[str]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from fabrail import plotstyle

    records = called_records()
    for qual, rec in records:
        assert isinstance(rec, FrontierRecord) and isinstance(rec, dict), (
            f"{qual}: returns a stamped record (FAB-EXO-004)"
        )
        assert rec["tier"] == TIER == "frontier", f"{qual}: tier 'frontier' (FAB-EXO-004)"
        assert rec["banner"].startswith(BANNER_HEAD), f"{qual}: carries the 🔭 banner (FAB-EXO-004)"
        assert "🔭" in rec["banner"], f"{qual}: the banner shows 🔭 (FAB-EXO-004)"
        # stdout
        rec.emit()
        out = capsys.readouterr().out
        assert out.splitlines()[0] == rec["banner"], f"{qual}: stdout starts with the banner (FAB-EXO-004)"
        # JSON
        doc = json.loads(rec.to_json())
        assert doc["tier"] == "frontier" and doc["banner"] == rec["banner"], f"{qual}: JSON tier + banner (FAB-EXO-004)"
        assert doc["missing_orders"] == rec["missing_orders"], f"{qual}: JSON missing orders (FAB-EXO-004)"
        # plot watermark
        fig = plt.figure()
        try:
            plotstyle.watermark(fig, rec.watermark)
            texts = [t.get_text() for t in fig.texts]
        finally:
            plt.close(fig)
        assert any("FRONTIER" in t and format_missing_orders(rec["missing_orders"]) in t for t in texts), (
            f"{qual}: the plot watermark carries the banner wording and the missing orders (FAB-EXO-004)"
        )
    names = {q for q, _ in records}
    assert {"pegs.detection", "rtsc.sweep", "metabarrier.mass_law", "lunar.selene_scaling"} <= names, "guard coverage"
    if _native_built():
        assert {"safe_rl.simulate", "safe_rl.cbf_filter"} <= names, "safe_rl covered when the native module is built"
    assert frontier.watermark_text(frontier.frontier_banner(5)).startswith(frontier.WATERMARK_HEAD), "watermark helper"
