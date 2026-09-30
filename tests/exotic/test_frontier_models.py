"""Frontier tier (🔭): model checks beyond the FAB-FRN rows (no requirement markers).

These pin the frontier research §3 worked numbers the models reproduce (the PEGS sweep
table, the metabarrier areal masses, the QZS stiffness match, NV gap noise, the DAS
moveout), the safe-RL filter's zero-violation property, the record machinery, and two
self-checks on the frontier sources: numpy-only imports (the official guard is
``py:exotic/exo_guards::frontier_code_numpy_only``) and no restated pedestrian-cap
literals (FAB-EXO-114). Tolerances follow SPECIFICATION §0.8: (E) relative 1e-12;
quoted research values to half a unit of the printed digit plus the table's own
rounding slack, stated per assertion.

Owner: X8 (Wave 1).
"""

from __future__ import annotations

import ast
import json
import math
from pathlib import Path

import numpy as np
import pytest

from fabrail.exotic import frontier
from fabrail.exotic.frontier import (
    das_rail,
    lunar,
    meta,
    metabarrier,
    pegs,
    quantum,
    qzs,
    rtsc,
    safe_rl,
)
from fabrail.exotic.frontier._record import FrontierRecord, make_record

REL_E = 1e-12
FRONTIER_DIR = Path(frontier.__file__).resolve().parent

#: Frontier research §3a sweep table: rows ASD 1e-13, 1e-15, 1e-17, 1e-19; columns Mw 7 at
#: 50/100/200 km, Mw 8 at 200 km, Mw 9 at 500 km (depth 10 km); cells (t_det, gain) s.
RESEARCH_TABLE = {
    1e-13: ((8.9, -1.1), (23.7, -8.2), (98.9, -68.0), (37.3, -6.5), (87.0, -10.0)),
    1e-15: ((2.8, 5.0), (6.6, 8.9), (16.3, 14.5), (11.7, 19.1), (27.6, 49.0)),
    1e-17: ((0.9, 7.0), (2.1, 13.4), (4.9, 25.9), (3.7, 27.1), (8.7, 68.0)),
    1e-19: ((0.3, 7.6), (0.7, 14.8), (1.6, 29.3), (1.2, 29.6), (2.8, 74.0)),
}


def _slack(quoted: float) -> float:
    """Half a unit of the printed digit plus 0.1 s of the table's own rounding (research table)."""
    return 0.6 if float(quoted).is_integer() and abs(quoted) >= 10 else 0.15


def test_pegs_reproduces_research_table() -> None:
    sw = pegs.sweep()
    asd = list(sw["inputs"]["asd_per_rthz"])
    for a, row in RESEARCH_TABLE.items():
        i = asd.index(a)
        for j, (t_q, g_q) in enumerate(row):
            t = sw["t_det_s"][i, j]
            g = sw["advantage_vs_onsite_p_s"][i, j]
            assert abs(t - t_q) <= _slack(t_q), f"t_det {t:.2f} vs {t_q} (frontier research §3a, ASD {a}, case {j})"
            assert abs(g - g_q) <= _slack(g_q), f"gain {g:.2f} vs {g_q} (frontier research §3a, ASD {a}, case {j})"


def test_pegs_i2_closed_form_integrates_the_moment() -> None:
    mw = 7.0
    m0 = 10.0 ** (1.5 * mw + 9.1)
    th = 2.26e-6 * m0 ** (1.0 / 3.0)
    t = np.linspace(0.0, 4.0 * th, 40_001)
    i2 = pegs._i2(t, mw)
    # M0(t) = M0·t²/(2T_h²) on the rise, so I₂(T_h) = M0·T_h²/24; after the rupture M0(t) = M0,
    # so I₂'' = M0 (second difference of the closed form).
    assert rel_err(float(pegs._i2(th, mw)), m0 * th**2 / 24.0) <= REL_E, "(E) I₂(T_h) = M0·T_h²/24 (FRONTIER §F4)"
    dt = 0.5
    tail = pegs._i2(np.array([3.0, 3.0 + dt / th, 3.0 + 2 * dt / th]) * th, mw)
    assert rel_err((tail[2] - 2 * tail[1] + tail[0]) / dt**2, m0) <= 1e-9, "(E) I₂'' = M0 after the rupture (§F4)"
    assert np.all(np.diff(i2) >= 0.0), "(E) I₂ is non-decreasing (FRONTIER §F4)"


def test_pegs_calibration_and_network_inputs_printed() -> None:
    det = pegs.detection()
    for key in ("r_network_km", "latency_s", "v_p_kmps", "asd_per_rthz", "mw", "r_km"):
        assert key in det["inputs"], f"lead-time outputs print their inputs ({key}; EXOTIC §X10)"
    assert rel_err(det["t_network_alert_s"], 22.0 / 6.5 + 3.3) <= REL_E, "(E) t_net = R_net/v_P + latency (§F4)"
    far = pegs.detection(1e-15, 7.0, 140.0, depth_km=10.0)
    assert rel_err(far["r_source_km"], math.hypot(140.0, 10.0)) <= REL_E, "(E) R = √(r² + depth²) (§F4)"


def test_pegs_magnitude_advisory_never_sole_trigger() -> None:
    big = pegs.magnitude_advisory(9.0)
    small = pegs.magnitude_advisory(7.5)
    assert big["advisory_available"] and big["long_period_mode"], "Mw 9: advisory and long-period mode (§3a)"
    assert not small["advisory_available"] and math.isinf(small["t_first_estimate_s"]), "Mw 7.5 below PEGSNet's 8.3"
    assert big["sole_trigger"] is False and small["sole_trigger"] is False, "never a sole trigger (FAB-EXO-810)"
    assert big["badge"] == "🧪" and big["tier"] == "frontier", "network PEGS is 🧪, output still bannered"


def test_rtsc_today_eta_is_the_sweep_floor() -> None:
    assert abs(rtsc.ETA_TODAY - 0.15) <= 0.005, f"η_today ≈ 0.15 ±0.005 (ATZ seed / ideal image): {rtsc.ETA_TODAY}"
    req = rtsc.required_tc()
    assert abs(req["pinning_energy_ratio"] - 3.9) <= 0.05, "U0 ratio 300/77 ≈ 3.9 (S ∝ kT/U0; frontier research §3d)"


def test_metabarrier_research_areal_masses_and_unit_budget() -> None:
    soft = metabarrier.gap_edge(f_r_hz=1.0, rho_kg_m3=1800.0, v_s_mps=150.0)
    assert abs(soft["areal_mass_for_target_kg_m2"] / 1e3 - 9.9) <= 0.05, "9.9 t/m² soft soil at 1 Hz (research §3b)"
    clay = metabarrier.gap_edge(f_r_hz=0.5, rho_kg_m3=1300.0, v_s_mps=60.0)
    assert abs(clay["areal_mass_for_target_kg_m2"] / 1e3 - 5.7) <= 0.05, "5.7 t/m² Mexico City clay (research §3b)"
    g = soft["gamma"]
    assert rel_err(soft["x_upper"], (g + math.sqrt(g * g + 4.0)) / 2.0) <= REL_E, "(E) x_u = (Γ + √(Γ² + 4))/2"
    ub = metabarrier.unit_budget()
    assert abs(ub["stroke_m"] - 0.4) <= 0.005, "stroke ≈ 0.4 m at PGV 0.5 m/s, ζ 0.1 (research §3b)"
    assert abs(ub["spring_force_n"] / 1e3 - 79.0) <= 0.5, "≈ 79 kN per 5 t unit (research §3b)"


def test_qzs_parallel_match() -> None:
    p = qzs.parallel_qzs()
    assert abs(p["f_p_hz"] - 3.5) <= 0.05, "f_p ≈ 3.5 Hz on a 20 mm spring (research §3c)"
    assert rel_err(p["epsilon"], (0.3 / p["f_p_hz"]) ** 2) <= REL_E, "(E) ε = (f_n/f_p)² (FRONTIER §F6)"
    assert abs(p["epsilon"] - 0.0073) <= 0.0001, "ε ≈ 0.0073 ±0.0001 (research §3c rounds f_p to 3.5 Hz)"
    assert qzs.series_stiffness(scale="instrument")["missing_orders"] == 0.0, "instrument scale: nothing missing"
    assert qzs.series_stiffness(scale="building")["missing_orders"] == 2.7, "building scale: log10(0.5 m/1 mm)"
    with pytest.raises(ValueError):
        qzs.tilt_error(scale="city")


def test_lunar_gravity_sweep_research_poles() -> None:
    gs = lunar.gravity_sweep()
    assert np.allclose(np.round(gs["pole_rad_s"], 1), [44.3, 27.2, 18.0]), "44.3/27.2/18.0 rad/s at 10 mm (§3g)"
    rl = lunar.radiator_limit(t_k=[25.0])
    assert abs(rl["q_rad_w_m2"][0] - 0.022) <= 0.0005, "0.022 W/m² at 25 K (research §3g)"


def test_nv_gap_noise_and_presets() -> None:
    nv = quantum.nv_gap_noise()
    assert abs(nv["gap_noise_m"] * 1e9 - 2.2) <= 0.05, "≈ 2 nm gap noise at 10 nT/√Hz (research §3f)"
    assert abs(nv["asd_required_t_per_rthz"] * 1e6 - 1.3) <= 0.05, "≈ 1.3 µT/√Hz allowed for 0.3 µm (§3f)"
    assert nv["missing_orders"] == 0.0, "NV is over-specified for gap control"
    phase2 = quantum.gradiometer_preset("toba_phase2")
    assert phase2["status"] == "demonstrated" and not phase2["before_p"], "TOBA Phase-II never beats P at 140 km"
    with pytest.raises(ValueError):
        quantum.gradiometer_preset("unobtainium")


def test_das_moveout() -> None:
    d = das_rail.beamform_moveout()
    assert rel_err(d["moveout_s"], 100.0 / 6000.0) <= REL_E, "(E) Δt = L/v_app (FRONTIER §F10)"
    assert abs(d["moveout_s"] * 1e3 - 17.0) <= 0.5, "≈ 17 ms over 100 m (research §3f)"
    assert d["resolvable"] and d["n_channels"] == 10, "resolvable at 1 kHz; 10 channels of 10 m"


def test_safe_rl_filter_zero_violations(native) -> None:
    params = native.Params.preset("door")
    e_max = params.get("safety.e_max_j")
    p_cap = params.get("safety.p_max_kgmps")
    f_cap = params.get("safety.f_static_n")
    sim = safe_rl.simulate()
    assert sim["total_violations"] == 0, "(E) zero cap violations under adversarial policies (research §3h)"
    assert np.all(sim["ke_max_j"] <= e_max), "(E) KE ≤ the imported energy cap (FAB-EXO-114)"
    assert np.all(sim["momentum_max_kgmps"] <= p_cap), "(E) momentum ≤ the imported cap"
    assert np.all(sim["force_max_n"] <= f_cap), "(E) |F| ≤ the unsensed static force cap"
    assert np.max(sim["ke_max_unfiltered_j"]) > e_max, "the unfiltered policy exceeds the cap: the filter matters"
    v_cap = native.v_low_energy(100.0)
    assert rel_err(sim["v_cap_energy_mps"], v_cap) <= REL_E, "(E) √(2E/m) equals _native.v_low_energy (PHYSICS §16)"
    assert abs(np.max(sim["v_max_mps"]) - v_cap) <= 1e-6, "the push policy rides the energy cap (0.184 m/s at 100 kg)"
    sensed = safe_rl.simulate(sensed=True, duration_s=1.0)
    assert sensed["total_violations"] == 0
    assert np.all(sensed["force_max_n"] <= params.get("safety.f_static_sensed_n")), "sensed static force cap"


def test_safe_rl_filter_projection(native) -> None:
    rec = safe_rl.cbf_filter(u_rl_n=0.0, v_mps=[0.0, 0.1])
    assert np.all(rec["u_safe_n"] == 0.0) and not rec["modified"].any(), "(E) a safe command passes unchanged"
    brake = safe_rl.cbf_filter(u_rl_n=10.0, v_mps=[0.3])
    assert brake["constraint"][0] == "brake" and brake["u_safe_n"][0] < 0.0, "over the cap: brake (research §3h)"


def test_record_machinery() -> None:
    rec = make_record("pegs", "test", 4.96, "note", {"x": 1}, {"y_s": math.inf, "z": np.arange(3)})
    assert isinstance(rec, FrontierRecord) and rec["missing_orders"] == 5.0 and rec["missing_orders_exact"] == 4.96
    doc = json.loads(rec.to_json())
    assert doc["y_s"] is None and doc["z"] == [0, 1, 2] and doc["tier"] == "frontier", "canonical JSON, inf → null"
    with pytest.raises(ValueError):
        make_record("pegs", "test", 1.0, "", {}, {"tier": "product"})
    with pytest.raises(ValueError):
        make_record("pegs", "test", -1.0, "", {}, {})
    with pytest.raises(KeyError):
        make_record("unknown", "test", 1.0, "", {}, {})
    assert "🔭" not in rec.watermark and rec.watermark.startswith(frontier.WATERMARK_HEAD), "watermark has no glyph"
    with pytest.raises(ValueError):
        frontier.watermark_text({"tier": "frontier"})


def test_cards_match_exo_run_schema() -> None:
    keys = {"item", "badge", "maturity", "anchor_tier", "missing_orders", "note", "requirement"}
    for card in meta.frontier_cards():
        assert keys <= set(card), f"{card['item']}: exo_run.js frontier card keys (fabrail.docs.exo_run/1)"
        assert card["tier"] == "frontier" and card["banner"].startswith(frontier.BANNER_HEAD)
    assert meta.headline("safe_rl") is None and meta.missing_orders("safe_rl") == 0.0
    with pytest.raises(KeyError):
        meta.headline("nope")


def test_lazy_submodules() -> None:
    assert frontier.pegs is pegs
    with pytest.raises(AttributeError):
        _ = frontier.not_a_model


def test_frontier_sources_numpy_only() -> None:
    allowed = {"__future__", "math", "json", "sys", "importlib", "collections", "functools", "types", "typing", "numpy"}
    for path in sorted(FRONTIER_DIR.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                roots = {a.name.split(".")[0] for a in node.names}
                assert roots <= allowed, f"{path.name}: imports {roots - allowed} (numpy only, FAB-EXO-002)"
            elif isinstance(node, ast.ImportFrom) and node.level == 0:
                root = (node.module or "").split(".")[0]
                native = node.module == "fabrail" and [a.name for a in node.names] == ["_native"]
                assert root in allowed or (native and path.name == "safe_rl.py"), (
                    f"{path.name}: imports from {node.module} (numpy only; the native caps read is safe_rl's alone)"
                )


def test_frontier_sources_restate_no_pedestrian_caps() -> None:
    literals = ("1" + ".69", "22" + ".3")  # built so this file restates nothing either
    for path in sorted(FRONTIER_DIR.glob("*.py")):
        text = path.read_text(encoding="utf-8")
        for lit in literals:
            assert lit not in text, f"{path.name} restates a pedestrian cap literal {lit} (FAB-EXO-114)"


def test_outputs_avoid_the_vocabulary_lists() -> None:
    banned = (
        "certified",
        "guarantee",
        "protects",
        "sil 4 compliant",
        "prevents collapse",
        "perfect",
        "absurdly",
        "earthquake-proof",
        "no damage",
    )
    texts = [str(meta.headline(it.key)) for it in frontier._items.ITEMS if meta.headline(it.key) is not None]
    texts += [c["banner"] + c["note"] for c in meta.frontier_cards()]
    for path in sorted(FRONTIER_DIR.glob("*.py")):
        texts.append(path.read_text(encoding="utf-8"))
    for text in texts:
        low = text.lower()
        for word in banned:
            assert word not in low, f"frontier text uses {word!r} (EXOTIC §0.3, FAB-EXO-005)"


def rel_err(a: float, b: float) -> float:
    return abs(a - b) / max(abs(a), abs(b), 1e-300)
