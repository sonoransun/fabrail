"""Contract: the analytic anchors, asserted through the native surface (``fabrail._native``).

Every closed form lives once, in ``fabrail_params::derive`` (Rust, f64). These tests
call it through PyO3 and check it two ways: against the closed form restated here from
docs/PHYSICS.md (tolerance class (E), relative 1e-12 unless a derivation step allows
1e-9), and against the rounded values quoted by SPECIFICATION §2/§3 and core design
§E.5 (tolerance = half a unit of the quoted last digit, or the row's stated tolerance).
Every assertion message states its tolerance and source.

FAB-HAP-010 (μ_eq) is a simulation requirement: it runs a short Travel1d coast through
``fabrail._native.run`` and skips only while the simulator is still a stub.

Requirement rows (docs/SPECIFICATION.md §3): FAB-LEV-001, FAB-LEV-005, FAB-LEV-006,
FAB-LEV-014, FAB-PRP-005, FAB-HAP-010, FAB-HAP-011, FAB-SAF-005, FAB-SEI-012 (two
tests), FAB-ENV-003. The unmarked tests pin the remaining core §E.5 anchors
(iso15066_speeds, pole, terminus, damper and isolation values).
"""

from __future__ import annotations

import json
import math

import pytest

CLASSES = ("drawer", "door", "wall", "heavy")

#: Standard gravity, m/s² (fabrail_params::derive::G0_MPS2; CGPM 1901).
G0 = 9.80665
#: Air density, kg/m³ (fabrail_params::derive::RHO_AIR_KG_PER_M3; ISA at 20 °C).
RHO_AIR = 1.204
#: Orifice discharge coefficient (fabrail_params::derive::ORIFICE_CD; research journal C15).
ORIFICE_CD = 0.6
#: ISO/TS 15066:2016 Annex A body regions: stiffness N/m and effective mass kg.
K_HAND, M_HAND = 75.0e3, 0.6
K_CHEST, M_CHEST = 25.0e3, 40.0
#: Closing-edge seal stiffness bound, N/m (SPECIFICATION §2 row "Closing-edge seal"; F6).
K_EDGE = 10.0e3
#: Pedestrian kinetic-energy cap, J (SPECIFICATION §2 "KE at cap"; ANSI A156.19).
E_CAP = 1.69

REL_E = 1e-12


def rel_err(a: float, b: float) -> float:
    return abs(a - b) / max(abs(a), abs(b), 1e-300)


def n_lift(p) -> int:
    """Lift elements: active stations with a PM and an upward normal (derive::n_lift)."""
    n = int(p.get("design.suspension.n_magnets"))
    count = sum(
        1
        for k in range(n)
        if p.get(f"design.suspension.stations.{k}.normal_sign") > 0 and p.get(f"design.suspension.stations.{k}.has_pm")
    )
    return count or n


def lift_stations(p) -> list[dict[str, float]]:
    n = int(p.get("design.suspension.n_magnets"))
    return [
        {k: p.get(f"design.suspension.stations.{i}.{k}") for k in ("x_m", "y_m", "z_m")}
        for i in range(n)
        if p.get(f"design.suspension.stations.{i}.normal_sign") > 0
    ]


# ---- FAB-LEV-001 ------------------------------------------------------------------------


@pytest.mark.req("FAB-LEV-001")
def test_earnshaw_and_roll_authority(native):
    """Heave and pitch are open-loop unstable in every class (Earnshaw); roll is unstable on DRAWER and
    HEAVY and gravity-stable on top-hung DOOR and WALL; validate rejects the wrong roll loop."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        d = p.derived()
        g_star, hp, m_star = p.get("design.suspension.g_star_m"), d["h_prime_m"], p.get("design.carriage.m_star_kg")
        pole = d["pole_rad_s"]
        assert pole > 0 and d["pole_pitch_rad_s"] > 0, f"{cls}: heave and pitch must be unstable (Earnshaw; PHYSICS §3)"
        assert rel_err(pole, native.open_loop_pole(g_star, hp)) <= REL_E
        # p = √(4g₀/(2g* + h′)) and p² = n·k_s/m* at the balanced pole (PHYSICS §3 eq for k_s).
        assert rel_err(pole, math.sqrt(4 * G0 / (2 * g_star + hp))) <= REL_E, f"{cls}: pole (E) rel 1e-12, PHYSICS §3"
        assert rel_err(pole**2, n_lift(p) * d["k_s_n_per_m"] / m_star) <= 1e-9, (
            f"{cls}: p² = n·k_s/m* (rel 1e-9, PHYSICS §3; the preset pole is balanced at g*)"
        )
        # Roll: magnetic negative stiffness k_s·Σ(y − y_c)² against gravity m*·g₀·h_cg
        # (h_cg < 0 below the pole plane stabilises; derive::mode_model; PHYSICS §6, §15).
        st = lift_stations(p)
        y_c = p.get("design.carriage.cg_m.1")
        sum_r2 = sum((s["y_m"] - y_c) ** 2 for s in st)
        h_cg = p.get("design.carriage.cg_m.2") - sum(s["z_m"] for s in st) / len(st)
        k_mag = d["k_s_n_per_m"] * sum_r2
        k_grav = m_star * G0 * h_cg
        j_x = p.get("design.carriage.j_kgm2.0")
        roll_active = p.get("control.levitation.roll_active")
        if cls in ("drawer", "heavy"):
            assert roll_active and d["pole_roll_rad_s"] > 0, f"{cls}: roll must be active (FAB-LEV-001)"
            assert rel_err(d["pole_roll_rad_s"] ** 2, (k_mag + k_grav) / j_x) <= 1e-9, (
                f"{cls}: roll pole² = (k_s·Σr² + m*g₀h_cg)/J_x (rel 1e-9; PHYSICS §15)"
            )
            _rejects_roll(p, False)
        else:
            assert not roll_active and d["pole_roll_rad_s"] == 0.0, f"{cls}: roll must be passive (top-hung; FAB-LEV-001)"
            assert k_grav + k_mag < 0, f"{cls}: gravity {-k_grav:.1f} must beat magnetic {k_mag:.1f} N·m/rad"
            _rejects_roll(p, True)
    door = native.Params.preset("door")
    st = lift_stations(door)
    k_mag = door.derived()["k_s_n_per_m"] * sum(s["y_m"] ** 2 for s in st)
    k_grav = -door.get("design.carriage.m_star_kg") * G0 * door.get("design.carriage.cg_m.2")
    assert abs(k_mag - 51.0) <= 0.5 and abs(k_grav - 1030.0) <= 0.5, (
        f"DOOR roll authority {k_mag:.1f} vs gravity {k_grav:.1f} N·m/rad; physics perspective 51 vs 1030 (±0.5)"
    )


def _rejects_roll(p, roll_active: bool) -> None:
    with pytest.raises(ValueError, match=r"^control\.levitation\.roll_active: "):
        p.with_overrides({"control.levitation.roll_active": roll_active})


# ---- FAB-LEV-005 ------------------------------------------------------------------------


@pytest.mark.req("FAB-LEV-005")
def test_zero_power_mass(native):
    """m(g) = m*·((2g* + h′)/(2g + h′))²: DRAWER 1.5 → 45.76, 5.0 → 9.96, 6.2 → 7.12 kg; the gap inverse and the
    E.5 float gaps agree."""
    p = native.Params.preset("drawer")
    g_star, hp, m_star = p.get("design.suspension.g_star_m"), p.derived()["h_prime_m"], p.get("design.carriage.m_star_kg")
    for gap_mm, quoted in ((1.5, 45.76), (5.0, 9.96), (6.2, 7.12)):
        g = gap_mm * 1e-3
        m = native.zero_power_mass(g, g_star, hp, m_star)
        exact = m_star * ((2 * g_star + hp) / (2 * g + hp)) ** 2
        assert rel_err(m, exact) <= REL_E, f"m({gap_mm} mm) (E) rel 1e-12, PHYSICS §3"
        assert abs(m - quoted) <= 0.005, f"m({gap_mm} mm) = {m:.4f} kg vs SPEC {quoted} kg (±0.005, FAB-LEV-005)"
        assert rel_err(native.zero_power_gap(m, g_star, hp, m_star), g) <= REL_E, "gap inverse (E) rel 1e-12"
    # Unclamped float gaps at 48 / 26 / 8 kg (core §E.5): 1.427 / 2.500 / 5.761 mm. E.5 used the rounded
    # 10×32 mm pole; the preset solves the pole length for exact balance at g* (W0-B1 note), which moves the
    # third decimal at 48 kg (1.4276 mm), hence ±0.001 mm.
    for m, quoted_mm in ((48.0, 1.427), (26.0, 2.500), (8.0, 5.761)):
        g_mm = native.zero_power_gap(m, g_star, hp, m_star) * 1e3
        assert abs(g_mm - quoted_mm) <= 1e-3, f"DRAWER float gap at {m} kg {g_mm:.4f} mm vs {quoted_mm} (±0.001, core §E.5)"
    # DOOR +98.1 N and WALL +490.5 N (+10 / +50 kg at 9.81 m/s²): 4.5967 / 9.1357 mm (core §E.5, FAB-LEV-004).
    for cls, extra_kg, quoted_mm in (("door", 10.0, 4.5967), ("wall", 50.0, 9.1357)):
        q = native.Params.preset(cls)
        m0 = q.get("design.carriage.m_star_kg")
        g_mm = native.zero_power_gap(m0 + extra_kg, q.get("design.suspension.g_star_m"), q.derived()["h_prime_m"], m0) * 1e3
        assert abs(g_mm - quoted_mm) <= 5e-5, f"{cls} +{extra_kg} kg gap {g_mm:.5f} mm vs {quoted_mm} (±0.00005, core §E.5)"


# ---- FAB-LEV-006 ------------------------------------------------------------------------

#: F/F* at the upper stop (SPECIFICATION §2 row "F/F* at stop / at landing").
UPPER_RATIO = {"drawer": 2.16, "door": 1.278, "wall": 1.26, "heavy": 1.26}
#: Stay-landed ratios F_pm(g_land)/(m_op_min·g) quoted by FAB-LEV-006.
STAY_LANDED = {"drawer": 0.824, "door": 0.850, "wall": 0.863, "heavy": 0.863}


@pytest.mark.req("FAB-LEV-006")
def test_stop_force_ratios(native):
    """Upper-stop F/(m*·g) ≤ 1.30 (DOOR/WALL/HEAVY), stay-landed ≤ 0.90, DRAWER upper rollers ≥ 550 N, and
    validate rejects violations."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        d = p.derived()
        s = {k: p.get(f"design.suspension.{k}") for k in ("g_star_m", "g_up_m", "g_land_m")}
        m_star, m_min = p.get("design.carriage.m_star_kg"), p.get("design.carriage.m_op_min_kg")
        up = native.stop_ratio(p, s["g_up_m"], m_star)
        landed = native.stop_ratio(p, s["g_land_m"], m_min)
        # With the pole balanced at g*, n·F_pm(g) = m(g)·g₀, so the ratio is m(g)/m (PHYSICS §3).
        for gap, m, ratio in ((s["g_up_m"], m_star, up), (s["g_land_m"], m_min, landed)):
            via_mass = native.zero_power_mass(gap, s["g_star_m"], d["h_prime_m"], m_star) / m
            assert rel_err(ratio, via_mass) <= 1e-9, f"{cls}: stop ratio vs m(g)/m (rel 1e-9, PHYSICS §3)"
        assert abs(up - UPPER_RATIO[cls]) <= 5e-3, f"{cls}: upper ratio {up:.4f} vs SPEC §2 {UPPER_RATIO[cls]} (±0.005)"
        if cls != "drawer":
            assert up <= 1.30, f"{cls}: upper-stop ratio {up:.4f} > 1.30 (exact; FAB-LEV-006)"
        assert landed <= 0.90, f"{cls}: stay-landed ratio {landed:.4f} > 0.90 (exact; F18)"
        # SPEC quotes 0.824/0.850 for the rounded 10×32 / 20×51 mm poles; the presets use the balanced pole
        # length (exact 0.825 / 0.849; W0 note), so the quote holds to ±0.002 (P).
        assert abs(landed - STAY_LANDED[cls]) <= 2e-3, f"{cls}: stay-landed {landed:.4f} vs SPEC {STAY_LANDED[cls]} (±0.002, P)"
        assert rel_err(d["stop_ratio_up"], up) <= REL_E and rel_err(d["stay_landed_ratio"], landed) <= REL_E
        rollers = p.get("design.landing.upper_roller_rating_n")
        assert rollers >= d["upper_stop_force_n"], f"{cls}: upper rollers {rollers} N below n·F_pm(g_up) {d['upper_stop_force_n']:.1f} N"
    drawer = native.Params.preset("drawer")
    assert drawer.get("design.landing.upper_roller_rating_n") >= 550.0, "DRAWER elastomer upper rollers ≥ 550 N (F2)"
    door = native.Params.preset("door")
    with pytest.raises(ValueError, match=r"^design\.suspension\.g_up_m: "):
        door.with_overrides({"design.suspension.g_up_m": 0.0035})
    with pytest.raises(ValueError, match=r"^design\.suspension\.g_land_m: "):
        door.with_overrides({"design.suspension.g_land_m": 0.0059})
    with pytest.raises(ValueError, match=r"^design\.landing\.upper_roller_rating_n: "):
        drawer.with_overrides({"design.landing.upper_roller_rating_n": 540.0})


# ---- FAB-LEV-014 ------------------------------------------------------------------------

THERMAL_40K_MM = {"drawer": -0.195, "door": -0.416, "wall": -0.891, "heavy": -0.891}


@pytest.mark.req("FAB-LEV-014")
def test_thermal_gap_shift(native):
    """+40 K static zero-power gap shift −0.195 / −0.416 / −0.891 mm: Δg = (2g* + h′)·α_Br·ΔT/2 (rel 1e-9)."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        d = p.derived()
        shift = native.thermal_gap_shift(p, 40.0)
        exact = (2 * p.get("design.suspension.g_star_m") + d["h_prime_m"]) * p.get("design.suspension.tempco_br_per_k") * 40.0 / 2
        assert rel_err(shift, exact) <= 1e-9, f"{cls}: Δg(40 K) closed form (rel 1e-9, PHYSICS §12; FAB-LEV-014)"
        assert rel_err(d["thermal_gap_40k_m"], shift) <= REL_E
        assert abs(shift * 1e3 - THERMAL_40K_MM[cls]) <= 5e-4, (
            f"{cls}: Δg(40 K) = {shift * 1e3:.4f} mm vs SPEC {THERMAL_40K_MM[cls]} mm (±0.0005)"
        )
        assert rel_err(native.thermal_gap_shift(p, -20.0), -0.5 * shift) <= 1e-9, "Δg is linear in ΔT (PHYSICS §12)"


# ---- FAB-PRP-005 ------------------------------------------------------------------------

D_STOP_MM = {"drawer": 33.43, "door": 36.31, "wall": 8.86, "heavy": 4.91}
#: The caps as SPECIFICATION §2 row "v_max unsensed" prints them (DOOR: the open cap), m/s; the d_stop quotes
#: were evaluated at these rounded values.
V_CAP_QUOTED = {"drawer": 0.2654, "door": 0.1838, "wall": 0.0446, "heavy": 0.025}


def _d_stop(v: float, a: float, j: float) -> float:
    """Jerk-limited stop distance (PHYSICS §16): v²/(2a) + v·a/(2j) if v ≥ a²/j, else v·√(v/j)."""
    return v * v / (2 * a) + v * a / (2 * j) if v >= a * a / j else v * math.sqrt(v / j)


@pytest.mark.req("FAB-PRP-005")
def test_d_stop_values(native):
    """d_stop at the caps equals the closed form (rel 1e-9): 33.43 / 36.31 / 8.86 / 4.91 mm; both branches."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        v = max(p.get("safety.v_open_max_mps"), p.get("safety.v_close_max_mps"))
        a, j = p.get("control.motion.a_max_mps2"), p.get("control.motion.j_max_mps3")
        d = native.stop_distance(v, a, j)
        assert rel_err(d, _d_stop(v, a, j)) <= 1e-9, f"{cls}: d_stop closed form (rel 1e-9, PHYSICS §16; FAB-PRP-005)"
        assert rel_err(p.derived()["d_stop_m"], d) <= REL_E
        d_quoted = native.stop_distance(V_CAP_QUOTED[cls], a, j)
        assert abs(d_quoted * 1e3 - D_STOP_MM[cls]) <= 5e-3, (
            f"{cls}: d_stop({V_CAP_QUOTED[cls]} m/s) {d_quoted * 1e3:.4f} mm vs SPEC {D_STOP_MM[cls]} mm (±0.005)"
        )
        assert abs(d - d_quoted) <= 2e-5, f"{cls}: exact vs printed cap moves d_stop by {abs(d - d_quoted) * 1e3:.4f} mm (≤ 0.02)"
        v_small = 0.5 * a * a / j  # the short (no constant-deceleration) branch
        assert rel_err(native.stop_distance(v_small, a, j), v_small * math.sqrt(v_small / j)) <= 1e-9
        # The two branches meet at v = a²/j (continuity of the closed form).
        v_edge = a * a / j
        assert rel_err(v_edge**2 / (2 * a) + v_edge * a / (2 * j), v_edge * math.sqrt(v_edge / j)) <= 1e-9


# ---- FAB-HAP-010 ------------------------------------------------------------------------


@pytest.mark.req("FAB-HAP-010")
def test_equivalent_friction_band(native, record_property):
    """μ_eq of a levitated 0.1 m/s coast (Travel1d, 50 ms, mid-travel) is reported and lies in [1e-3, 1e-2],
    never 0 (S: the drag ledger of the plant; PHYSICS §9)."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        x_mid = p.get("design.rail.x_closed_m") + 0.5 * p.get("design.rail.travel_m")
        spec = {
            "schema": native.SCENARIO_SCHEMA,
            "name": f"mu_eq_{cls}",
            "doc": "levitated coast at 0.1 m/s for the equivalent-friction report (FAB-HAP-010)",
            "duration_s": 0.05,
            "noise_seed": 42,
            "fidelity": {"model": "travel1d", "coil": "voltage" if p.get("control.rates.coil_mode_voltage") else "lag"},
            "initial": {"lev": "levitated", "x_m": x_mid, "v_mps": 0.1},
        }
        try:
            run = native.run(p, native.Scenario.from_json(json.dumps(spec)))
        except NotImplementedError as exc:
            pytest.skip(f"fabrail_sim::run is not implemented yet: {exc}")
        kpis = run.kpis()
        assert "mu_eq" in kpis, f"{cls}: the run did not report mu_eq (FAB-HAP-010)"
        mu = kpis["mu_eq"]
        record_property(f"mu_eq_{cls}", mu)
        # Analytic levitated drag at 0.1 m/s ((b_dist + c_rail)·v + F_hyst)/(m*·g₀), reported alongside (PHYSICS §9).
        m = p.get("design.carriage.m_star_kg")
        mu_rail = (
            (p.get("design.damping.b_dist_n_s_per_m") + p.get("design.damping.c_rail_n_s_per_m")) * 0.1 + p.get("design.damping.f_hyst_n")
        ) / (m * G0)
        record_property(f"mu_eq_rail_analytic_{cls}", mu_rail)
        assert mu > 0.0, f"{cls}: μ_eq must never be 0 ('zero friction' is a direction; FAB-HAP-010)"
        assert 1e-3 <= mu <= 1e-2, f"{cls}: μ_eq = {mu:.3e} outside [1e-3, 1e-2] (S, FAB-HAP-010; analytic rail part {mu_rail:.3e})"


# ---- FAB-HAP-011 ------------------------------------------------------------------------


@pytest.mark.req("FAB-HAP-011")
def test_sensor_noise_force_gain(native, record_property):
    """K = m_eff·(a₁ + p²), a₁ = (1 + 2ζα)ω², ω = 2.5·p (rel 1e-9); K = k_i·k_p; DRAWER ≈ 0.31 N/µm; force noise reported."""
    for cls in CLASSES:
        p = native.Params.preset(cls)
        d = p.derived()
        pole, omega = d["pole_rad_s"], d["omega_heave_rad_s"]
        ratio = p.get("control.levitation.omega_ratio")
        assert ratio == 2.5, f"{cls}: ω = 2.5·p (plan §2b item 1)"
        assert rel_err(omega, ratio * pole) <= REL_E, f"{cls}: ω = omega_ratio·p (E)"
        zeta, alpha = p.get("control.levitation.zeta"), p.get("control.levitation.alpha")
        m_eff = p.get("design.carriage.m_star_kg") / n_lift(p)
        k_expected = m_eff * ((1 + 2 * zeta * alpha) * omega**2 + pole**2)
        k = native.noise_force_gain(p)
        assert rel_err(k, k_expected) <= 1e-9, f"{cls}: K = m(a1 + p²) (rel 1e-9, PHYSICS §13/§15; FAB-HAP-011)"
        cont = native.zp_gains(p, "heave", False)
        assert rel_err(k, d["k_i_n_per_a"] * cont["kp_a_per_m"]) <= 1e-9, f"{cls}: K = k_i·k_p of the continuous design (rel 1e-9)"
        sigma = p.get("design.sensors.gap_noise_m")
        record_property(f"force_noise_n_{cls}", k * sigma)
        record_property(f"noise_force_gain_n_per_um_{cls}", k * 1e-6)
    drawer = native.noise_force_gain(native.Params.preset("drawer")) * 1e-6
    assert abs(drawer - 0.31) <= 0.31 * 0.02, f"DRAWER K = {drawer:.4f} N/µm vs ≈ 0.31 N/µm (±2 %, plan §2b item 1)"


# ---- FAB-SAF-005 ------------------------------------------------------------------------


@pytest.mark.req("FAB-SAF-005")
def test_clamp_force_energy_form(native):
    """F = √(2Ek) = v·√(mk): 1.69 J → chest 290.7 N, hand 503.5 N, 173 N through a 10 N/mm edge; WALL chest 157.7 N;
    every unsensed cap ≤ 400 N through the edge seal."""
    chest = native.clamp_force(E_CAP, K_CHEST)
    hand = native.clamp_force(E_CAP, K_HAND)
    k_series = native.series_stiffness(K_HAND, K_EDGE)
    assert rel_err(k_series, K_HAND * K_EDGE / (K_HAND + K_EDGE)) <= REL_E, "series stiffness (E)"
    edge = native.clamp_force(E_CAP, k_series)
    for got, quoted, what in ((chest, 290.7, "chest"), (hand, 503.5, "hand"), (edge, 173.0, "hand + edge")):
        exact = math.sqrt(2 * E_CAP * (K_CHEST if what == "chest" else K_HAND if what == "hand" else k_series))
        assert rel_err(got, exact) <= REL_E, f"{what}: √(2Ek) (E) rel 1e-12, PHYSICS §18"
        tol = 0.5 if what == "hand + edge" else 0.05
        assert abs(got - quoted) <= tol, f"{what}: {got:.2f} N vs SPEC {quoted} N (±{tol}; FAB-SAF-005)"
    for cls in CLASSES:
        p = native.Params.preset(cls)
        m = p.get("design.carriage.m_rated_kg")
        v = max(p.get("safety.v_open_max_mps"), p.get("safety.v_close_max_mps"))
        e = 0.5 * m * v * v
        f_edge = native.clamp_force(e, native.series_stiffness(K_HAND, p.get("design.damping.k_edge_n_per_m")))
        assert rel_err(f_edge, v * math.sqrt(m * native.series_stiffness(K_HAND, p.get("design.damping.k_edge_n_per_m")))) <= 1e-9, (
            f"{cls}: √(2Ek) = v·√(mk) (rel 1e-9)"
        )
        assert rel_err(f_edge, p.derived()["clamp_hand_edge_n"]) <= REL_E
        assert f_edge <= 400.0, f"{cls}: {f_edge:.1f} N through the edge seal exceeds 400 N (exact; FAB-SAF-005)"
    wall = native.Params.preset("wall")
    v_wall = max(wall.get("safety.v_open_max_mps"), wall.get("safety.v_close_max_mps"))
    f_wall = native.clamp_force(0.5 * wall.get("design.carriage.m_rated_kg") * v_wall**2, K_CHEST)
    assert abs(f_wall - 157.7) <= 0.05, f"WALL chest {f_wall:.2f} N vs SPEC 157.7 N (±0.05; FAB-SAF-005)"


# ---- FAB-SEI-012 ------------------------------------------------------------------------


@pytest.mark.req("FAB-SEI-012")
def test_lead_time_crossover(native):
    """S−P = R·(1/V_s − 1/V_p) = 0.1190 s/km at 6.0 / 3.5 km/s; the on-site/network crossover is 40.8 km."""
    sp = native.sp_time(1.0)
    assert rel_err(sp, 1 / 3.5 - 1 / 6.0) <= REL_E, "S−P (E) rel 1e-12, PHYSICS §20"
    assert abs(sp - 0.1190) <= 5e-5, f"S−P {sp:.6f} s/km vs SPEC 0.1190 (±0.00005; FAB-SEI-012)"
    assert rel_err(native.sp_time(1.0, 6.0, 3.5), sp) <= REL_E
    assert rel_err(native.sp_time(20.0), 20.0 * sp) <= REL_E, "S−P is linear in R"
    x = native.lead_crossover_km()
    assert rel_err(x, 6.0 * (8.0 - 1.2)) <= REL_E and abs(x - 40.8) <= 1e-9, f"crossover {x} km vs 40.8 km (E; FAB-SEI-012)"
    # At the crossover an on-site alert (P arrival + 1.2 s decision) and the 8 s network alert give the same lead;
    # nearer, on-site wins; farther, the network wins (PHYSICS §20).
    def leads(r_km: float) -> tuple[float, float]:
        return native.lead_time(r_km, 3.5, r_km / 6.0 + 1.2), native.lead_time(r_km, 3.5, 8.0)

    on, net = leads(x)
    assert abs(on - net) <= 1e-9, f"leads at the crossover {on} vs {net} s (E)"
    on, net = leads(20.0)
    assert on > net, "inside the crossover the on-site alert gives more lead"
    on, net = leads(80.0)
    assert net > on, "beyond the crossover the network alert gives more lead"
    assert rel_err(native.lead_time(70.0, 3.5, 8.0), 70.0 / 3.5 - 8.0) <= REL_E, "lead time R/V_s − t_alert (E)"


#: Wu–Zhao Pd anchors (magnitude, R km, cm): SPECIFICATION FAB-SEI-012 and core §E.5.
WU_ZHAO_CM = ((7.0, 20.0, 0.7118), (6.5, 30.0, 0.1762), (5.0, 10.0, 0.0643), (7.8, 100.0, 0.2987))


@pytest.mark.req("FAB-SEI-012")
def test_wu_zhao_pd(native):
    """log₁₀ Pd[cm] = −3.463 + 0.729·M − 1.374·log₁₀ R (Wu and Zhao 2006): M7 at 20 km = 0.7118 cm (rel 1e-4); native in metres."""
    for mag, r_km, quoted_cm in WU_ZHAO_CM:
        pd_cm = native.pd_wu_zhao(mag, r_km) * 100.0
        exact = 10 ** (-3.463 + 0.729 * mag - 1.374 * math.log10(r_km))
        assert rel_err(pd_cm, exact) <= 1e-12, f"Pd(M{mag}, {r_km} km) (E) rel 1e-12, PHYSICS §20"
        tol = 1e-4 if (mag, r_km) == (7.0, 20.0) else 5e-5 / quoted_cm
        assert rel_err(pd_cm, quoted_cm) <= tol, f"Pd(M{mag}, {r_km} km) = {pd_cm:.5f} cm vs {quoted_cm} cm (rel {tol:.1g})"


# ---- FAB-ENV-003 ------------------------------------------------------------------------


@pytest.mark.req("FAB-ENV-003")
def test_piston_pressure(native):
    """The DRAWER cabinet piston (0.06 m² front, 40 cm² leak) pushed at 0.5 m/s gives 5.6 N ±5 %."""
    p = native.Params.preset("drawer")
    a_p, a_leak = p.get("design.air.cavity_area_m2"), p.get("design.air.leak_area_m2")
    assert (a_p, a_leak) == (0.06, 40e-4), "DRAWER cavity 0.06 m², 40 cm² reveal (core §C.1; research C15)"
    v = 0.5  # m/s, the push speed of the C15 piston case
    f = native.piston_force(a_p, v, a_leak)
    exact = 0.5 * RHO_AIR * (v * a_p / (ORIFICE_CD * a_leak)) ** 2 * a_p
    assert rel_err(f, exact) <= REL_E, "ΔP = ½ρ(vA_p/(C_d·A_leak))², F = ΔP·A_p (E) rel 1e-12, PHYSICS §11"
    assert abs(f - 5.6) <= 0.05 * 5.6, f"piston force {f:.3f} N vs 5.6 N (±5 %, FAB-ENV-003)"
    assert rel_err(native.piston_force(a_p, 2 * v, a_leak), 4 * f) <= 1e-9, "orifice force is quadratic in v"
    assert native.piston_force(a_p, v, 0.0) == 0.0, "a sealed cavity is a gas spring, not an orifice (rs:plant/air)"


# ---- remaining core §E.5 anchors (no requirement row of their own) --------------------------


def test_iso15066_speeds(native):
    """ISO/TS 15066 v = F/√(μk), μ = m_h·m/(m_h + m): 500 kg against the chest at 140 / 280 N → 0.145 / 0.291 m/s (E)."""
    for force, quoted in ((140.0, 0.145), (280.0, 0.291)):
        v = native.iso15066_speed(force, M_CHEST, 500.0, K_CHEST)
        mu = M_CHEST * 500.0 / (M_CHEST + 500.0)
        assert rel_err(v, force / math.sqrt(mu * K_CHEST)) <= 1e-9, "F/√(μk) (rel 1e-9; physics perspective A13)"
        assert abs(v - quoted) <= 5e-4, f"{force} N: {v:.4f} m/s vs {quoted} (±0.0005, physics perspective A13)"
    assert native.iso15066_speed(140.0, M_HAND, 0.0, K_HAND) == 0.0, "no moving mass, no speed limit defined"


def test_open_loop_pole_and_caps(native):
    """p = 69.49 / 47.57 / 32.50 / 32.50 rad/s and the unsensed caps 0.2654 / 0.1838 · 0.152 / 0.0446 / 0.025 m/s."""
    poles = {"drawer": 69.49, "door": 47.57, "wall": 32.50, "heavy": 32.50}
    caps = {"drawer": (0.2654, 0.2654), "door": (0.1838, 0.152), "wall": (0.0446, 0.0446), "heavy": (0.025, 0.025)}
    for cls in CLASSES:
        p = native.Params.preset(cls)
        assert abs(p.derived()["pole_rad_s"] - poles[cls]) <= 5e-3, f"{cls}: pole vs SPEC (±0.005 rad/s; FAB-LEV-003)"
        v_open, v_close = p.get("safety.v_open_max_mps"), p.get("safety.v_close_max_mps")
        assert abs(v_open - caps[cls][0]) <= 5e-5 and abs(v_close - caps[cls][1]) <= 5e-5, f"{cls}: caps (±0.00005; FAB-SAF-002)"
    m = native.Params.preset("wall").get("design.carriage.m_rated_kg")
    assert rel_err(native.momentum_speed_cap(m), 22.3 / m) <= REL_E, "WALL momentum cap 22.3/m (F3)"
    assert rel_err(native.v_low_energy(48.0), math.sqrt(2 * E_CAP / 48.0)) <= REL_E, "√(2E/m) at the 1.69 J default"


#: Terminus table (SPECIFICATION §2 / plan §4b): (τ, v) → d_0 mm, d_b mm, t_arrive s.
TERMINUS = {
    ("drawer", "open"): (49.8, 32.1, 0.98),
    ("door", "open"): (57.6, 37.2, 1.68),
    ("door", "close"): (46.1, 31.6, 1.62),
    ("wall", "open"): (27.8, 25.7, 3.56),
    ("heavy", "open"): (25.3, 24.7, 5.84),
}


def test_terminus_table(native):
    """Terminus blend and arrival per class within ±2 % of the plan §4b table (the FAB-HAP-016 tolerance)."""
    for (cls, way), (d0_mm, db_mm, t_arr) in TERMINUS.items():
        p = native.Params.preset(cls)
        v = p.get(f"safety.v_{way}_max_mps")
        tau, j = p.get("control.motion.terminus.tau_s"), p.get("control.motion.terminus.j_mps3")
        b = native.terminus_blend(v, j, tau)
        t_b = math.sqrt(tau * tau + 2 * v / j) - tau
        assert rel_err(b["t_b_s"], t_b) <= REL_E and rel_err(b["d_b_m"], j * t_b * tau * tau) <= 1e-9, "blend (E), PHYSICS §16.1"
        t = native.terminus_t_arrive(v, j, tau, p.get("control.motion.terminus.d_eps_m"))
        for got, want, what in ((b["d_0_m"] * 1e3, d0_mm, "d_0"), (b["d_b_m"] * 1e3, db_mm, "d_b"), (t, t_arr, "t_arrive")):
            assert rel_err(got, want) <= 0.02, f"{cls} {way}: {what} {got:.4g} vs {want} (±2 %, plan §4b)"
        assert tau >= native.tau_min(v, p.get("control.motion.a_max_mps2"), j), f"{cls}: τ ≥ τ_min (PHYSICS §16.1)"


def test_damper_and_isolation_anchors(native):
    """Den Hartog μ 0.02: 0.98039 / 0.08407 / peak 10.05; Warburton 0.97548 / 0.07019; T(3, 0.2) = 0.1931, T(√2) = 1;
    WALL T_min with F_cont = 6.28 s; unattended close 0.23 m/s at 48 kg (PHYSICS §18, §21; plan §4b)."""
    f, z = native.den_hartog(0.02)
    assert abs(f - 0.98039) <= 5e-6 and abs(z - 0.08407) <= 5e-6, "Den Hartog (±5e-6; core §E.5)"
    assert rel_err(f, 1 / 1.02) <= REL_E and rel_err(z, math.sqrt(3 * 0.02 / (8 * 1.02**3))) <= REL_E, "Den Hartog (E)"
    assert abs(native.den_hartog_peak(0.02) - 10.05) <= 5e-3
    f, z = native.warburton(0.02)
    assert abs(f - 0.97548) <= 5e-6 and abs(z - 0.07019) <= 5e-6, "Warburton base (±5e-6; core §E.5)"
    t = native.transmissibility(3.0, 0.2)
    assert rel_err(t, math.sqrt((1 + (2 * 0.2 * 3) ** 2) / ((1 - 9) ** 2 + (2 * 0.2 * 3) ** 2))) <= REL_E
    assert abs(t - 0.1931) <= 5e-5, f"T(3, 0.2) = {t:.5f} vs 0.1931 (±0.00005)"
    for zeta in (0.05, 0.3, 0.7):
        assert abs(native.transmissibility(math.sqrt(2), zeta) - 1.0) <= 1e-12, "T(√2) = 1 for any ζ"
        assert rel_err(native.skyhook_transmissibility(1.0, zeta), 1 / (2 * zeta)) <= REL_E, "skyhook at r = 1: 1/(2ζ)"
    wall = native.Params.preset("wall")
    t_min = native.t_min_isolation(wall.get("design.carriage.m_rated_kg"), wall.get("control.seismic.isolation.stroke_m"), wall.get("design.lsm.f_cont_n"))
    assert abs(t_min - 6.28) <= 5e-3 and rel_err(t_min, wall.derived()["t_min_iso_s"]) <= REL_E, "WALL T_min 6.28 s (plan §2b item 5)"
    v = native.unattended_close_speed(0.2654, 150.0, 48.0, native.series_stiffness(K_HAND, K_EDGE))
    assert abs(v - 0.23) <= 5e-3, f"unattended close {v:.4f} m/s vs 0.23 (±0.005, plan §4b)"


def test_impact_speeds_and_manual_force(native):
    """Control-loss impact speeds DOOR 50.6 / 55.6, WALL 68.8 / 76.3 mm/s (1 %); no-PM manual force μ·m·g
    1.41 / 2.94 / 14.7 / 147 N (roller μ 0.003; PHYSICS §8, §15)."""
    impacts = {"door": (50.6, 55.6), "wall": (68.8, 76.3), "drawer": (109.6, 197.3)}
    for cls, (up, down) in impacts.items():
        p = native.Params.preset(cls)
        got_up, got_down = native.impact_speed(p, "up") * 1e3, native.impact_speed(p, "down") * 1e3
        assert rel_err(got_up, up) <= 0.01 and rel_err(got_down, down) <= 0.01, (
            f"{cls}: impact {got_up:.1f} / {got_down:.1f} mm/s vs {up} / {down} (1 %, physics perspective A7)"
        )
    with pytest.raises(ValueError, match=r"^direction: "):
        native.impact_speed(native.Params.preset("door"), "sideways")
    for cls, quoted in (("drawer", 1.41), ("door", 2.94), ("wall", 14.7), ("heavy", 147.0)):
        p = native.Params.preset(cls)
        bound = native.manual_force(p, False)
        assert rel_err(bound, 0.003 * p.get("design.carriage.m_rated_kg") * G0) <= REL_E, f"{cls}: μ·m·g (E)"
        assert rel_err(bound, quoted) <= 5e-3, f"{cls}: no-PM manual force {bound:.3f} N vs SPEC {quoted} N (±0.5 %)"
        assert 0.0 <= native.manual_force(p, True) <= bound, f"{cls}: PM relief lowers the manual force"
