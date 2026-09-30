"""Contract: the parameter presets and their validation rules, through the native surface.

``fabrail-params`` is the single source of every preset; ``Params.with_overrides``
reseals and re-validates, so each rule of ``fabrail_params::validate`` can be checked
from Python: the preset satisfies it, a value at the bound is accepted where the rule
is an inequality, and a violation raises ``ValueError("path: reason")`` naming the
field the SPECIFICATION row is about. Reference values quoted from SPECIFICATION §2
are restated here only as test expectations, each with its source.

Requirement rows (docs/SPECIFICATION.md §3): FAB-LEV-015, FAB-PRP-003, FAB-HAP-002,
FAB-SAF-003, FAB-SAF-014, FAB-SEI-001, FAB-PWR-001, FAB-PWR-005, FAB-SW-004. The
unmarked tests cover the Python platform (``Params.set``/``describe``/JSON, the
``--set`` coercion and the PARAMETERS.md generator; core design §E.3).
"""

from __future__ import annotations

import json
import math
import re

import pytest

from fabrail import config, presets

CLASSES = ("drawer", "door", "wall", "heavy")

#: Standard gravity, m/s² (fabrail_params::derive::G0_MPS2; CGPM 1901).
G0 = 9.80665
#: Vacuum permeability, H/m (fabrail_params::derive::MU0_H_PER_M; CODATA 2018).
MU0 = 1.25663706212e-6

#: "path: reason" (SPECIFICATION FAB-SW-004; ParamError Display).
PATH_REASON = re.compile(r"^(?P<path>[a-z][a-z0-9_]*(?:\.[a-z0-9_]+)+): (?P<reason>\S.*)$")


def _rejects(params, overrides: dict[str, object], path: str) -> str:
    """Assert ``params.with_overrides(overrides)`` raises ValueError naming ``path``; return the reason."""
    with pytest.raises(ValueError) as info:
        params.with_overrides(overrides)
    text = str(info.value)
    m = PATH_REASON.match(text)
    assert m is not None, f"error {text!r} is not 'path: reason' (FAB-SW-004)"
    assert m["path"] == path, f"{overrides} rejected at {m['path']!r}, expected {path!r}: {text}"
    return m["reason"]


def _lift_elements(p) -> int:
    """n_lift: active stations with a PM and an upward normal (fabrail_params::derive::n_lift)."""
    n = int(p.get("design.suspension.n_magnets"))
    count = sum(
        1
        for k in range(n)
        if p.get(f"design.suspension.stations.{k}.normal_sign") > 0 and p.get(f"design.suspension.stations.{k}.has_pm")
    )
    return count or n


# ---- FAB-LEV-015 ------------------------------------------------------------------------


@pytest.mark.req("FAB-LEV-015")
def test_rail_saturation_rule(native):
    """validate enforces flange thickness ≥ B_g·w/B_sat,rail (1.5 T); at the bound it passes, below it fails."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        d = p.derived()
        g_star = p.get("design.suspension.g_star_m")
        # B_g is the larger of the design gap flux and the PM flux at g* (PHYSICS §10).
        b_pm = MU0 * d["m_pm_at"] / (2.0 * g_star + d["h_prime_m"])
        b_g = max(p.get("design.suspension.b_g_t"), b_pm)
        b_sat = p.get("design.rail.b_sat_rail_t")
        assert b_sat == pytest.approx(1.5, abs=0.0), f"{cls}: rail B_sat is 1.5 T (SPECIFICATION FAB-LEV-015)"
        t_min = b_g * p.get("design.suspension.pole_w_m") / b_sat
        flange = p.get("design.rail.flange_t_m")
        assert flange >= t_min, f"{cls}: preset flange {flange} m below B_g·w/1.5 T = {t_min} m (exact; FAB-LEV-015)"
        p.with_overrides({"design.rail.flange_t_m": t_min * (1.0 + 1e-12)})
        _rejects(p, {"design.rail.flange_t_m": t_min * 0.99}, "design.rail.flange_t_m")


# ---- FAB-PRP-003 ------------------------------------------------------------------------

#: Thrust peak / continuous, N (SPECIFICATION §2 row "Thrust peak / continuous").
THRUST_TABLE_N = {"drawer": (90.0, 20.0), "door": (150.0, 50.0), "wall": (400.0, 100.0), "heavy": (2000.0, 500.0)}


@pytest.mark.req("FAB-PRP-003")
def test_thrust_sizing(native):
    """Peak and continuous LSM thrust ≥ the §2 values and ≥ m_rated·a_max + drag; validate rejects undersizing."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        f_peak, f_cont = p.get("design.lsm.f_peak_n"), p.get("design.lsm.f_cont_n")
        table_peak, table_cont = THRUST_TABLE_N[cls]
        assert f_peak >= table_peak and f_cont >= table_cont, (
            f"{cls}: thrust {f_peak}/{f_cont} N below the SPECIFICATION §2 values {table_peak}/{table_cont} N (exact)"
        )
        v_cap = max(p.get("safety.v_open_max_mps"), p.get("safety.v_close_max_mps"))
        # Levitated drag (b_dist + c_rail)·v + F_hyst (PHYSICS §9; derive::drag_force).
        drag = (p.get("design.damping.b_dist_n_s_per_m") + p.get("design.damping.c_rail_n_s_per_m")) * v_cap + p.get(
            "design.damping.f_hyst_n"
        )
        need = p.get("design.carriage.m_rated_kg") * p.get("control.motion.a_max_mps2") + drag
        assert f_peak >= need, f"{cls}: peak {f_peak} N < m_rated·a_max + drag = {need} N (exact; FAB-PRP-003)"
        assert f_cont >= drag, f"{cls}: continuous {f_cont} N < drag at the cap {drag} N (exact; FAB-PRP-003)"
        low_peak = 0.99 * need
        _rejects(p, {"design.lsm.f_peak_n": low_peak, "design.lsm.f_cont_n": min(f_cont, 0.5 * low_peak)}, "design.lsm.f_peak_n")
        _rejects(p, {"design.lsm.f_cont_n": 0.99 * drag}, "design.lsm.f_cont_n")


# ---- FAB-HAP-002 ------------------------------------------------------------------------


@pytest.mark.req("FAB-HAP-002")
def test_assist_ratio_bounds(native):
    """validate accepts m/M_v ≤ 2 (DRAWER), ≤ 5 sensorless, ≤ 20 with a handle load cell, and rejects anything above."""
    load_cell = {"design.sensors.handle_n_per_code": 0.01}
    for cls in CLASSES:
        p = native.Params.preset(cls)
        bound = 2.0 if cls == "drawer" else 5.0
        assert p.get("control.haptic.mass_ratio") <= bound, f"{cls}: preset m/M_v above {bound} (FAB-HAP-002)"
        assert p.get("design.sensors.handle_n_per_code") == 0.0, f"{cls}: presets are sensorless (no load cell)"
        p.with_overrides({"control.haptic.mass_ratio": bound})
        _rejects(p, {"control.haptic.mass_ratio": bound * 1.005}, "control.haptic.mass_ratio")
        p.with_overrides({**load_cell, "control.haptic.mass_ratio": 20.0})
        _rejects(p, {**load_cell, "control.haptic.mass_ratio": 20.5}, "control.haptic.mass_ratio")


# ---- FAB-SAF-003 ------------------------------------------------------------------------


@pytest.mark.req("FAB-SAF-003")
def test_presence_sensing_gate(native):
    """Speeds above the unsensed caps need PL d presence sensing; HEAVY automatic needs PL d, else hold-to-run."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        assert p.get("design.presence") == "none", f"{cls}: presets are unsensed (SPECIFICATION §2)"
        m = p.get("design.carriage.m_rated_kg")
        e = p.get("safety.e_max_j")
        pm = p.get("safety.p_max_kgmps")
        cap = native.v_low_energy(m, e)
        if pm > 0:
            cap = min(cap, native.momentum_speed_cap(m, pm))
        for key in ("safety.v_open_max_mps", "safety.v_close_max_mps"):
            v = p.get(key)
            assert v <= cap * (1 + 1e-12), f"{cls}: {key} {v} above the unsensed cap {cap} m/s (rel 1e-12; FAB-SAF-003)"
            sensed = p.get(key.replace("_max_mps", "_sensed_mps"))
            faster = min(1.05 * cap, sensed) if sensed > cap else 1.05 * cap
            reason = _rejects(p, {key: faster}, key)
            assert "presence" in reason, f"{cls}: {key} rejection must name the presence gate: {reason}"
            if sensed > cap:
                q = p.with_overrides({"design.presence": "pl_d", key: faster})
                assert q.get("safety.presence_copy") == "pl_d", "fitting PL d reseals the diverse copy (SAF-015)"
    heavy = native.Params.preset("heavy")
    assert heavy.get("design.operating_mode") == "hold_to_run", "HEAVY defaults to hold-to-run (SPECIFICATION §2; F5)"
    _rejects(heavy, {"design.operating_mode": "automatic"}, "design.operating_mode")
    auto = heavy.with_overrides(
        {"design.operating_mode": "automatic", "design.presence": "pl_d", "safety.p_max_kgmps": 22.3}
    )
    assert auto.get("safety.presence_copy") == "pl_d"


# ---- FAB-SAF-014 ------------------------------------------------------------------------

#: Bumper basis speed per class, m/s (fabrail_params::presets::bumper_basis_speed; F13:
#: DRAWER ½·48·1² = 24 J, the others 1.5 m/s).
BUMPER_BASIS_MPS = {"drawer": 1.0, "door": 1.5, "wall": 1.5, "heavy": 1.5}
#: Minimum bumper ratings, J (SPECIFICATION FAB-SAF-014).
BUMPER_MIN_J = {"drawer": 24.0, "door": 113.0, "wall": 563.0, "heavy": 5625.0}


@pytest.mark.req("FAB-SAF-014")
def test_bumper_energy_rating(native):
    """End zones are 150 mm, bumpers are rated ≥ ½·m_rated·v_basis² (24/113/563/5625 J) and validate rejects less;
    the terminus's governor overtravel stays inside the end zone, short of the bumper."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        assert p.get("design.damping.ez_len_m") == 0.15, f"{cls}: end zone must be 150 mm (exact; FAB-SAF-014)"
        need = native.bumper_rating(p.get("design.carriage.m_rated_kg"), BUMPER_BASIS_MPS[cls])
        rated = p.get("design.damping.bumper_e_rated_j")
        assert rated >= BUMPER_MIN_J[cls] and rated >= need * (1 - 1e-9), (
            f"{cls}: bumper {rated} J below ½·m·v² = {need} J / SPEC {BUMPER_MIN_J[cls]} J (rel 1e-9; FAB-SAF-014, F13)"
        )
        assert math.ceil(need) == BUMPER_MIN_J[cls], f"{cls}: SPEC rating is ⌈½·m·v²⌉ = {math.ceil(need)} J"
        _rejects(p, {"design.damping.bumper_e_rated_j": 0.95 * need}, "design.damping.bumper_e_rated_j")
        overtravel = p.get("control.motion.terminus.overtravel_m")
        assert p.get("control.motion.gov_margin_m") < overtravel < p.get("design.damping.ez_len_m"), (
            f"{cls}: governor overtravel {overtravel} m must exceed the margin and stay inside the 150 mm end zone "
            "(plan §4b: nominal termini never reach the bumpers)"
        )


# ---- FAB-SEI-001 ------------------------------------------------------------------------


@pytest.mark.req("FAB-SEI-001")
def test_accel_mount_rejects_carriage(native):
    """The EEW accelerometer is on the structure (or rail); validate rejects a carriage mount."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        assert p.get("design.sensors.accel_mount") == "structure", f"{cls}: EEW accelerometer on the structure (F16)"
        assert p.with_overrides({"design.sensors.accel_mount": "rail"}).get("design.sensors.accel_mount") == "rail"
        reason = _rejects(p, {"design.sensors.accel_mount": "carriage"}, "design.sensors.accel_mount")
        assert "SEI-001" in reason


# ---- FAB-PWR-001 ------------------------------------------------------------------------


@pytest.mark.req("FAB-PWR-001")
def test_rest_policy_defaults(native):
    """Rest policies default to SEAT / HOVER / LAND / LAND + lock, with the hardware each needs."""
    expected = {"drawer": "seat", "door": "hover", "wall": "land", "heavy": "land_lock"}
    for cls, policy in expected.items():
        p = native.Params.preset(cls)
        assert p.rail_class.key == cls
        assert p.get("design.rest_policy") == policy, f"{cls}: rest policy {p.get('design.rest_policy')!r} != {policy!r} (FAB-PWR-001)"
    drawer = native.Params.preset("drawer")
    assert drawer.get("design.landing.kind") == "seat", "DRAWER rests on its captive seat (F17)"
    g = {k: drawer.get(f"design.suspension.{k}") for k in ("g_star_m", "g_seat_m", "g_land_m")}
    assert g["g_star_m"] < g["g_seat_m"] < g["g_land_m"], f"DRAWER seat gap ordering {g} (F17)"
    assert native.Params.preset("heavy").get("design.landing.parking_lock") is True, "HEAVY LAND + lock needs a lock"
    _rejects(native.Params.preset("heavy"), {"design.landing.parking_lock": False}, "design.landing.parking_lock")


# ---- FAB-PWR-005 ------------------------------------------------------------------------

#: Bus voltages, V (SPECIFICATION §2 row "V_bus").
V_BUS_V = {"drawer": 24.0, "door": 48.0, "wall": 48.0, "heavy": 300.0}


@pytest.mark.req("FAB-PWR-005")
def test_bus_voltage_and_slew(native):
    """Buses are 24/48/48/300 V (SELV < 60 V except HEAVY) and the coil slew k_i·V/L meets m*·g₀/n·ω."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        v_bus = p.get("design.power.v_bus_v")
        assert v_bus == V_BUS_V[cls], f"{cls}: V_bus {v_bus} V != {V_BUS_V[cls]} V (exact; FAB-PWR-005)"
        assert v_bus < 60.0 or cls == "heavy", f"{cls}: only HEAVY may exceed 60 V SELV"
        d = p.derived()
        available = d["k_i_n_per_a"] * v_bus / d["coil_l_h"]
        required = p.get("design.carriage.m_star_kg") * G0 / _lift_elements(p) * d["omega_heave_rad_s"]
        assert available >= required, (
            f"{cls}: slew k_i·V/L = {available:.4g} N/s < m*·g₀/n·ω = {required:.4g} N/s (exact; PHYSICS §4; FAB-PWR-005)"
        )
        # A leakage inductance that pushes L past k_i·V/required breaks the slew rule.
        l_breaking = d["k_i_n_per_a"] * v_bus / required
        _rejects(p, {"design.suspension.l_sigma_h": 1.05 * l_breaking}, "design.power.v_bus_v")
        if cls != "heavy":
            _rejects(p, {"design.power.v_bus_v": 60.0}, "design.power.v_bus_v")


# ---- FAB-SW-004 -------------------------------------------------------------------------


@pytest.mark.req("FAB-SW-004")
def test_presets_validate(native):
    """Every preset validates and round-trips; unsafe combinations are rejected with "path: reason"."""
    paths = {row["path"] for row in native.Params.describe()}
    for cls in CLASSES:
        p = native.Params.preset(cls)
        p.validate()
        assert native.Params.from_json(p.to_json()) == p, f"{cls}: JSON round trip changed the preset"
        assert getattr(presets, presets.FACTORIES[cls])() == p, f"{cls}: presets factory differs from Params.preset"
    unsafe = [
        ("door", {"safety.e_max_j": 2.0}, "safety.e_max_j"),
        ("door", {"design.damping.k_edge_n_per_m": 20_000.0}, "design.damping.k_edge_n_per_m"),
        ("door", {"control.seismic.policy.lock_after_land": True}, "control.seismic.policy.lock_after_land"),
        ("door", {"design.egress": "fire_rated", "control.seismic.policy.action": "open_and_stop"}, "control.seismic.policy.action"),
        ("wall", {"safety.v_close_max_mps": 0.05}, "safety.v_close_max_mps"),
        ("drawer", {"design.power.pwm_hz": 16_000.0}, "design.power.pwm_hz"),
        ("drawer", {"safety.i_coil_pulse_a": 99.0}, "safety.i_coil_pulse_a"),
        ("heavy", {"design.operating_mode": "automatic"}, "design.operating_mode"),
    ]
    for cls, overrides, path in unsafe:
        _rejects(native.Params.preset(cls), overrides, path)
        assert path in paths, f"{path} is not a Params.describe() path"
    for cls, path, value in [("door", "design.power.v_bus_v", 70.0), ("drawer", "safety.e_max_j", 5.0)]:
        p = native.Params.preset(cls)
        before = p.to_json()
        with pytest.raises(ValueError, match=r"^[a-z_.0-9]+: "):
            p.set(path, value)
        assert p.to_json() == before, f"{cls}: a rejected Params.set must leave the object unchanged"


# ---- Python platform (core design §E.3; no requirement row) -------------------------------


def test_set_overrides_revalidate(native):
    """set/with_overrides keep the leaf type, reseal the safety block and re-validate."""
    p = native.Params.preset("door")
    crc0 = p.get("safety.crc")
    p.set("design.damping.b_dist_n_s_per_m", 6.0)
    assert p.get("design.damping.b_dist_n_s_per_m") == 6.0
    # K_f = √3·K_e must hold after every edit (PRP-002), so the pair goes in one call.
    with pytest.raises(ValueError, match=r"^design\.lsm\.k_e_v_s_per_m: "):
        p.set("design.lsm.k_f_n_per_a", 31.0)
    p = p.with_overrides({"design.lsm.k_f_n_per_a": 31.0, "design.lsm.k_e_v_s_per_m": 31.0 / math.sqrt(3.0)})
    assert p.get("safety.k_f_copy_n_per_a") == 31.0, "the diverse copy follows the design value (SAF-015)"
    assert p.get("safety.crc") != crc0, "the CRC is resealed after a safety-relevant edit"
    assert native.Params.from_json(p.to_json()) == p
    p.set("/control/rates/fast_decim", 2)  # JSON pointer form
    with pytest.raises(ValueError, match=r"^control\.rates\.fast_decim: expects an integer"):
        p.set("control.rates.fast_decim", 2.5)
    with pytest.raises(ValueError, match=r"^control\.levitation\.roll_active: expects a bool"):
        p.set("control.levitation.roll_active", 1)
    with pytest.raises(ValueError, match=r"^design\.nope: unknown parameter"):
        p.set("design.nope", 1.0)
    with pytest.raises(ValueError, match=r"^design\.carriage: not a leaf"):
        p.get("design.carriage")
    # Editing a diverse copy directly is a deliberate seal mismatch: rejected.
    with pytest.raises(ValueError, match=r"^safety\.k_f_copy_n_per_a: "):
        p.set("safety.k_f_copy_n_per_a", 1.0)
    q = p.with_overrides({"design.damping.b_dist_n_s_per_m": 7.0})
    assert q.get("design.damping.b_dist_n_s_per_m") == 7.0 and p.get("design.damping.b_dist_n_s_per_m") == 6.0


def test_from_json_verifies_seal(native):
    """A hand-edited JSON document whose safety block no longer matches its CRC is rejected (SAF-015)."""
    doc = json.loads(native.Params.preset("door").to_json())
    doc["safety"]["f_static_n"] = 60.0
    with pytest.raises(ValueError, match=r"^safety\.crc: .*reseal"):
        native.Params.from_json(json.dumps(doc))
    fixed = native.Params.preset("door").with_overrides({"safety.f_static_n": 60.0})
    assert native.Params.from_json(fixed.to_json()) == fixed


def test_describe_complete(native):
    """describe() lists every leaf once, with unit, range, source and a finite default per class inside its range."""
    rows = native.Params.describe()
    paths = [r["path"] for r in rows]
    assert len(paths) == len(set(paths)) and len(rows) > 300
    doc = json.loads(native.Params.preset("door").to_json())

    def leaves(obj, prefix=""):
        if isinstance(obj, dict):
            for k, v in obj.items():
                yield from leaves(v, f"{prefix}.{k}" if prefix else k)
        elif isinstance(obj, list):
            for i, v in enumerate(obj):
                yield from leaves(v, f"{prefix}.{i}")
        else:
            yield prefix

    assert sorted(leaves(doc)) == sorted(paths), "describe() and the JSON document disagree on the leaf set"
    for r in rows:
        assert set(r) >= {"path", "unit", "min", "max", "source", "default_by_class"}
        assert r["source"], f"{r['path']}: every parameter carries a source"
        assert r["min"] <= r["max"], f"{r['path']}: empty range"
        for cls in CLASSES:
            v = r["default_by_class"][cls]
            assert math.isfinite(v) and r["min"] <= v <= r["max"], f"{r['path']}: {cls} default {v} outside its range"


def test_json_roundtrip(native):
    """to_json/from_json is bit-exact for every preset, compact and indented."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        for indent in (None, 2, 4):
            text = p.to_json(indent=indent)
            q = native.Params.from_json(text)
            assert q == p and q.to_json(indent=indent) == text, f"{cls}: JSON round trip not exact (indent {indent})"
        derived = p.derived()
        assert all(math.isfinite(v) for v in derived.values()), f"{cls}: a derived quantity is not finite"


def test_parse_set_coercion(native):
    """--set values are parsed as JSON scalars and, with like=, coerced to each leaf's type."""
    raw = config.parse_set(["a.b=0.5", "a.c=2", "a.d=true", "a.e=door", "a.f=x=y", 'a.g="2"', "a.h=null"])
    assert raw == {"a.b": 0.5, "a.c": 2, "a.d": True, "a.e": "door", "a.f": "x=y", "a.g": "2", "a.h": None}
    for bad in (["noequals"], ["=1"], ["a b=1"]):
        with pytest.raises(ValueError):
            config.parse_set(bad)
    p = native.Params.preset("drawer")
    typed = config.parse_set(
        ["control.rates.fast_decim=2.0", "control.levitation.roll_active=on", "design.damping.b_dist_n_s_per_m=3", "design.class=DRAWER"],
        like=p,
    )
    assert typed == {
        "control.rates.fast_decim": 2,
        "control.levitation.roll_active": True,
        "design.damping.b_dist_n_s_per_m": 3.0,
        "design.class": "drawer",
    }
    assert isinstance(typed["control.rates.fast_decim"], int) and isinstance(typed["design.damping.b_dist_n_s_per_m"], float)
    for bad, msg in [
        ("control.rates.fast_decim=2.5", "expects an integer"),
        ("control.levitation.roll_active=2", "expects a bool"),
        ("design.damping.b_dist_n_s_per_m=fast", "expects a number"),
        ("design.damping.b_dist_n_s_per_m=NaN", "must be finite"),
    ]:
        with pytest.raises(ValueError, match=msg):
            config.parse_set([bad], like=p)
    with pytest.raises(ValueError, match="unknown parameter"):
        config.parse_set(["design.nope=1"], like=p)


def test_run_config_single_coil_source(native):
    """RunConfig sets control.rates.coil_mode_voltage from its coil mode and checks --plant keys against the schema."""
    for coil in config.COILS:
        cfg = config.RunConfig(rail_class="wall", coil=coil, overrides={"control.levitation.roll_active": 0})
        p = cfg.params()
        assert p.get(config.COIL_MODE_PATH) is (coil == "voltage")
        assert p.get("control.levitation.roll_active") is False
    cfg = config.RunConfig(plant={"mass_scale": "1.1", "c_rail_n_s_per_m": None, "rail_profile": None})
    assert cfg.plant == {"mass_scale": 1.1, "c_rail_n_s_per_m": None, "rail_profile": None}
    assert set(config.plant_keys()) >= {"mass_scale", "br_scale", "leak_sigma", "c_rail_n_s_per_m", "f_hyst_n", "leak_area_m2"}
    for bad in ({"bogus": 1.0}, {"mass_scale": None}, {"rail_profile": "series:floor"}, {"br_scale": "x"}):
        with pytest.raises(ValueError, match=r"^plant\."):
            config.RunConfig(plant=bad)
    for kwargs in ({"rail_class": "shelf"}, {"coil": "pwm"}, {"fidelity": "rigid"}, {"workers": 0}, {"duration_s": 0.0}, {"seed": -1}):
        with pytest.raises(ValueError):
            config.RunConfig(**kwargs)


def test_parameters_markdown_deterministic(native):
    """PARAMETERS.md is deterministic, lists every describe() path once and every derived quantity."""
    first, second = config.parameters_markdown(), config.parameters_markdown()
    assert first == second
    rows = native.Params.describe()
    for r in rows:
        assert first.count(f"| `{r['path']}` |") == 1, f"{r['path']} must appear exactly once"
    assert f"{len(rows)} parameters." in first
    for name in native.Params.preset("door").derived():
        assert f"| `{name}` |" in first, f"derived quantity {name} missing"
    assert "| `design.rest_policy` |  | 0 | 3 | seat | hover | land | land_lock |" in first
    pure = config.parameters_markdown(rows)
    assert pure == config.parameters_markdown(list(rows)) and "## derived" not in pure
