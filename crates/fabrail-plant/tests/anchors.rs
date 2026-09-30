//! Plant analytic anchors (core design §E.2 plant row; physics perspective §4.1 A5–A8,
//! A25): mechanical energy conservation, the voltage-mode electromechanical power balance,
//! Heave1 ≡ Carriage6 constrained heave, control-loss impact speeds and the stop ratios.
//! Every assertion states its tolerance class and source (core design §E.1).

use fabrail_params::derive::{self, FailDir, G0_MPS2};
use fabrail_params::{SystemParams, presets};
use fabrail_plant::body::{Carriage6, EnergyAudit};
use fabrail_plant::heave1::Heave1;
use fabrail_plant::magnet::{ElementGeom, ElementModel, T_REF_C};
use fabrail_plant::params::{Environment, Mismatch, PlantParams};
use fabrail_plant::{CoilMode, InitLev, InitialState, ModelConfig, PlantInputs, PlantModel, Tier};
use fabrail_types::{LsmMode, MAX_MAGNETS};

const H: f64 = 5.0e-5;
const FLOOR0: [[f64; 3]; 3] = [[0.0; 3]; 3];

fn plant(sp: &SystemParams) -> PlantParams {
    PlantParams::from_design(&sp.design, &Mismatch::default(), &Environment::default())
}

/// A conservative variant of `sp`: saturation, distributed/end-zone/rail/air drag, sway
/// damping and the thermal network switched off (temperatures stay at the reference), so
/// only the magnets and gravity act on a levitated body (physics perspective A5).
fn conservative(sp: &SystemParams) -> PlantParams {
    let mut d = sp.design;
    d.suspension.b_sat_t = 0.0;
    d.damping.b_dist_n_s_per_m = 0.0;
    d.damping.ez_cmax_n_s_per_m = 0.0;
    d.guidance.c_sway_n_s_per_m = 0.0;
    d.air.leaf_area_m2 = 0.0;
    d.thermal.c_th_coil_j_per_k = 1.0e30;
    d.thermal.c_th_mag_j_per_k = 1.0e30;
    let mm = Mismatch {
        c_rail_n_s_per_m: Some(0.0),
        f_hyst_n: Some(0.0),
        ..Mismatch::default()
    };
    PlantParams::from_design(&d, &mm, &Environment::default())
}

fn mech_energy(a: &EnergyAudit) -> f64 {
    a.kinetic_trans_j + a.kinetic_rot_j + a.gravity_j - a.coenergy_j + a.cogging_j + a.lsm_normal_j
}

#[test]
fn energy_conservation() {
    // Physics perspective A5: Carriage6 with saturation, eddy lag, dissipation and control
    // off and i = const (0 A): E = KE + m·g₀·z − ΣW′ is conserved over 50 ms of free 6-DOF
    // motion (heave, pitch diverging; roll/sway/yaw oscillating) to 1e-8 relative
    // ((I) RK4 at 50 µs, PHYSICS §5, §24).
    for sp in [presets::door(), presets::wall(), presets::drawer()] {
        let p = conservative(&sp);
        let cfg = ModelConfig {
            tier: Tier::Carriage6,
            coil: CoilMode::Lag,
            eddy_lag: false,
            kinematic_heave: true,
        };
        let mut c = Carriage6::new(&p, &cfg);
        let x0 = 0.5 * sp.design.rail.travel_m;
        c.reset(&InitialState {
            x_m: x0,
            ..InitialState::default()
        });
        // The DRAWER's active roll mode is fast (small J, ±0.2 m lever arms): a smaller
        // push keeps it off the upper stop for 50 ms.
        let k = if matches!(sp.design.class, fabrail_types::RailClass::Drawer) {
            0.01
        } else {
            1.0
        };
        c.set_body_state(
            [x0, 0.2e-3 * k, 20.0e-6 * k],
            [2.0e-4 * k, -1.0e-4 * k, 3.0e-4 * k],
            [0.0, 1.0e-4 * k, -2.0e-4 * k],
            [1.0e-3 * k, -2.0e-3 * k, 5.0e-4 * k],
        );
        let u = PlantInputs {
            coil_enabled: true,
            lsm_mode: LsmMode::Sto,
            ..PlantInputs::default()
        };
        let a0 = c.energy_audit();
        let e0 = mech_energy(&a0);
        let mut ke_max: f64 = 0.0;
        let mut drift_max: f64 = 0.0;
        for k in 0..1000 {
            c.step(k as f64 * H, H, &u, &FLOOR0);
            let a = c.energy_audit();
            ke_max = ke_max.max(a.kinetic_trans_j + a.kinetic_rot_j);
            drift_max = drift_max.max((mech_energy(&a) - e0).abs());
        }
        let t = c.truth();
        assert!(
            t.state.levitated && t.f_contact.guide_fin_n == 0.0 && t.f_contact.pads_n == 0.0,
            "no contact during the conservation run"
        );
        assert!(ke_max > 1e-12, "the body must move (KE_max {ke_max} J)");
        let scale = a0.coenergy_j.abs();
        assert!(
            drift_max <= 1e-8 * scale,
            "{:?}: |ΔE| {drift_max:e} J ≤ 1e-8·ΣW′ ({scale} J) ((I) rel 1e-8, PHYSICS §5)",
            sp.design.class
        );
        assert!(
            drift_max <= 1e-5 * ke_max,
            "{:?}: |ΔE| {drift_max:e} J ≪ KE_max {ke_max:e} J ((I), PHYSICS §24)",
            sp.design.class
        );
    }
}

#[test]
fn power_balance_voltage_mode() {
    // Physics perspective A6: in voltage mode ∫v·i = ∫R·i² + ∫i·dψ with ψ = N·Φ + L_σ·i and
    // ∫(M + N·i)·dΦ = Δ(½Φ²/P) + ∫F·dq, i.e. E_in − E_cu = Δ(½Φ²/P − M·Φ + ½L_σ·i² + KE
    // + m·g₀·z) with only magnets and gravity acting. It fails on a motional-EMF sign
    // error. Tolerance 1e-6 of the energy throughput ((I), PHYSICS §4).
    for sp in [presets::door(), presets::heavy()] {
        let p = conservative(&sp);
        let cfg = ModelConfig {
            tier: Tier::Carriage6,
            coil: CoilMode::Voltage,
            eddy_lag: false,
            kinematic_heave: true,
        };
        let mut c = Carriage6::new(&p, &cfg);
        let x0 = 0.5 * sp.design.rail.travel_m;
        c.reset(&InitialState {
            x_m: x0,
            ..InitialState::default()
        });
        c.set_body_state(
            [x0, 0.1e-3, 10.0e-6],
            [1.0e-4, 1.0e-4, 0.0],
            [0.0; 3],
            [0.0; 3],
        );
        let h_of = |a: &EnergyAudit| {
            a.stored_j - a.m_phi_j
                + a.leakage_j
                + a.kinetic_trans_j
                + a.kinetic_rot_j
                + a.gravity_j
                + a.cogging_j
                + a.lsm_normal_j
        };
        let a0 = c.energy_audit();
        let l0 = c.truth().e_diss;
        let n = usize::from(sp.design.suspension.n_magnets);
        let v_amp = 0.12 * sp.design.power.v_bus_v;
        for k in 0..1000 {
            let t = k as f64 * H;
            let mut v = [0.0; MAX_MAGNETS];
            for (j, vj) in v.iter_mut().enumerate().take(n) {
                let ph = 0.7 * j as f64;
                *vj = v_amp * (2.0 * core::f64::consts::PI * 100.0 * t + ph).sin() + 0.02 * v_amp;
            }
            let u = PlantInputs {
                coil_enabled: true,
                coil_v_or_iref: v,
                lsm_mode: LsmMode::Sto,
                ..PlantInputs::default()
            };
            c.step(t, H, &u, &FLOOR0);
        }
        let a1 = c.energy_audit();
        let l1 = c.truth().e_diss;
        let e_in = l1.elec_in_j - l0.elec_in_j;
        let e_cu = l1.coil_copper_j - l0.coil_copper_j;
        let dh = h_of(&a1) - h_of(&a0);
        let throughput = e_in.abs()
            + e_cu
            + (a1.gravity_j - a0.gravity_j).abs()
            + (a1.stored_j - a0.stored_j).abs();
        assert!(
            e_cu > 1e-3 && throughput > 1e-2,
            "the run must exchange energy ({throughput} J)"
        );
        assert!(
            ((e_in - e_cu) - dh).abs() <= 1e-6 * throughput,
            "{:?}: E_in − E_cu = {} J vs ΔH = {dh} J (1e-6 of {throughput} J, (I), PHYSICS §4)",
            sp.design.class,
            e_in - e_cu
        );
        assert!(
            c.truth().state.levitated,
            "no contact during the balance run"
        );
    }
}

/// Test-side PD current law on the Heave1 gap (per element, DOOR): i* = k_p·δg + k_d·δġ
/// with k_p = 6 A/mm > k_s/k_i and k_d = 88 A·s/m (ζ ≈ 0.7).
fn pd_iref(g: f64, g_dot: f64, g_star: f64) -> f64 {
    6000.0 * (g - g_star) + 88.0 * g_dot
}

#[test]
fn tier_equivalence() {
    // Physics perspective A25: a symmetric, untilted Carriage6 heave equals Heave1 to
    // 1e-9 relative over 0.2 s (same element model, contacts, coil electrics, eddy lag)
    // under a +10 kg payload step at 50 ms with a stabilising current law, and under open-
    // loop voltages in voltage mode ((I) rel 1e-9, PHYSICS §5).
    let sp = presets::door();
    let p = plant(&sp);
    let g_star = sp.design.suspension.g_star_m;
    let init = InitialState {
        x_m: 0.5,
        ..InitialState::default()
    };
    for coil in [CoilMode::Lag, CoilMode::Voltage] {
        let cfg6 = ModelConfig {
            tier: Tier::Carriage6,
            coil,
            eddy_lag: true,
            kinematic_heave: true,
        };
        let cfg1 = ModelConfig {
            tier: Tier::Heave1,
            ..cfg6
        };
        let mut c6 = Carriage6::new(&p, &cfg6);
        let mut h1 = Heave1::new(&p, &cfg1);
        c6.reset(&init);
        h1.reset(&init);
        let steps = if matches!(coil, CoilMode::Lag) {
            4000
        } else {
            400
        };
        let mut g_prev = h1.truth().gaps_m[0];
        let mut worst: f64 = 0.0;
        for k in 0..steps {
            let t = k as f64 * H;
            let g1 = h1.truth().gaps_m[0];
            let g_dot = (g1 - g_prev) / H;
            g_prev = g1;
            let cmd = match coil {
                CoilMode::Lag => pd_iref(g1, g_dot, g_star),
                CoilMode::Voltage => 0.3 * (2.0 * core::f64::consts::PI * 50.0 * t).sin(),
            };
            let u = PlantInputs {
                coil_enabled: true,
                coil_v_or_iref: [cmd; MAX_MAGNETS],
                payload_kg: if t >= 0.05 { 10.0 } else { 0.0 },
                lsm_mode: LsmMode::Sto,
                ..PlantInputs::default()
            };
            c6.step(t, H, &u, &FLOOR0);
            h1.step(t, H, &u, &FLOOR0);
            let (a, b) = (c6.truth(), h1.truth());
            for j in 0..4 {
                worst = worst.max((a.gaps_m[j] - b.gaps_m[j]).abs() / g_star);
                worst =
                    worst.max((a.coil_i_a[j] - b.coil_i_a[j]).abs() / (1.0 + b.coil_i_a[j].abs()));
            }
            worst = worst.max((a.pose[2] - b.pose[2]).abs() / g_star);
        }
        let (a, b) = (c6.truth(), h1.truth());
        assert!(
            worst <= 1e-9,
            "{coil:?}: Carriage6 vs Heave1 worst rel {worst:e} (≤ 1e-9, physics perspective A25)"
        );
        assert!(
            a.omega_radps == [0.0; 3] && a.pose[1] == 0.0,
            "symmetric heave stays untilted"
        );
        if matches!(coil, CoilMode::Lag) {
            // The payload step moved the gap (P-law sag), so the comparison is dynamic.
            assert!(
                (b.gaps_m[0] - g_star).abs() > 1e-6,
                "the payload step must move the gap"
            );
            assert!(b.state.levitated);
        }
    }
}

/// Speed at which the Heave1 gap first crosses `g_cross` after a 1 nm push (up or down)
/// from the zero-power rest (coils off, lag off), interpolated linearly within the
/// crossing step.
fn crossing_speed(p: &PlantParams, init: &InitialState, up: bool, g_cross: f64) -> f64 {
    let cfg = ModelConfig {
        tier: Tier::Heave1,
        coil: CoilMode::Lag,
        eddy_lag: false,
        kinematic_heave: true,
    };
    let mut m = Heave1::new(p, &cfg);
    m.reset(init);
    let z0 = m.truth().pose[2];
    m.set_heave(z0 + if up { 1e-9 } else { -1e-9 }, 0.0);
    let u = PlantInputs {
        payload_kg: init.payload_kg,
        ..PlantInputs::default()
    };
    let mut g_prev = m.truth().gaps_m[0];
    let mut v_prev = m.heave_rate_mps();
    let mut a_prev = 0.0;
    for k in 0..200_000u32 {
        m.step(k as f64 * H, H, &u, &FLOOR0);
        let g = m.truth().gaps_m[0];
        let v = m.heave_rate_mps();
        let crossed = if up { g <= g_cross } else { g >= g_cross };
        if crossed {
            // The contact acts within the crossing step: extrapolate from the last
            // contact-free step at its acceleration, v² = v_prev² + 2·a·Δz (Δz = −Δg).
            let dz = -(g_cross - g_prev);
            return (v_prev * v_prev + 2.0 * a_prev * dz).max(0.0).sqrt();
        }
        a_prev = (v - v_prev) / H;
        g_prev = g;
        v_prev = v;
    }
    panic!("the stop must be reached");
}

#[test]
fn impact_speed_closed_form() {
    // Physics perspective A7: after a control loss (i = 0, no lag) from the zero-power
    // rest, v² = 2(ΔU_mag ∓ m·g₀·Δz)/m at the upper stop / landing rollers
    // (params::derive::impact_speed_at; DOOR 50.6 / 55.6 mm/s, WALL 68.8 / 76.3, DRAWER at
    // 26 kg 109.6 / 197.3, empty DRAWER snap-up 0.38 m/s), within 1 % ((I), PHYSICS §15).
    let cases: [(SystemParams, f64, [f64; 2]); 3] = [
        (presets::door(), 0.0, [50.6e-3, 55.6e-3]),
        (presets::wall(), 0.0, [68.8e-3, 76.3e-3]),
        (presets::drawer(), 0.0, [109.6e-3, 197.3e-3]),
    ];
    for (sp, payload, table) in cases {
        let p = plant(&sp);
        let s = &sp.design.suspension;
        let init = InitialState {
            x_m: 0.5 * sp.design.rail.travel_m,
            payload_kg: payload,
            ..InitialState::default()
        };
        let m = sp.design.carriage.m_star_kg + payload;
        for (dir, g_cross, want_tab) in [
            (FailDir::Up, s.g_up_m, table[0]),
            (FailDir::Down, s.g_land_m, table[1]),
        ] {
            let v = crossing_speed(&p, &init, matches!(dir, FailDir::Up), g_cross);
            let want = derive::impact_speed_at(&sp, dir, m, s.g_star_m);
            assert!(
                (v - want).abs() <= 0.01 * want,
                "{:?} {dir:?}: {v} m/s vs closed form {want} (1 %, physics perspective A7)",
                sp.design.class
            );
            assert!(
                (want - want_tab).abs() <= 0.005 * want_tab,
                "closed form {want} vs anchor {want_tab} (0.5 %, physics perspective A7)"
            );
        }
    }
    // Empty DRAWER (8 kg: payload −18 kg) from its 5.761 mm float snaps up at ≈ 0.38 m/s.
    let sp = presets::drawer();
    let p = plant(&sp);
    let s = &sp.design.suspension;
    let init = InitialState {
        lev: InitLev::Levitated,
        x_m: 0.15,
        payload_kg: -18.0,
        ..InitialState::default()
    };
    let hp = derive::h_prime(s.h_m_m, s.mu_rec);
    let g0 = derive::zero_power_gap(8.0, s.g_star_m, hp, 26.0);
    assert!(
        (g0 - 5.761e-3).abs() < 1e-6,
        "8 kg float gap {g0} (E.5 anchor 5.761 mm)"
    );
    let v = crossing_speed(&p, &init, true, s.g_up_m);
    let want = derive::impact_speed_at(&sp, FailDir::Up, 8.0, g0);
    assert!(
        (v - want).abs() <= 0.01 * want && (want - 0.38).abs() < 0.01,
        "empty drawer snap-up {v} vs {want} m/s (≈ 0.38, 1 %, physics perspective A7)"
    );
}

#[test]
fn stop_ratios() {
    // Physics perspective A8 / LEV-006: the plant element model at y = 0, e = 1, i = 0,
    // 20 °C gives n·F(g_up)/(m*·g₀) and n·F(g_land)/(m·g₀) equal to params::derive::
    // stop_ratio ((E) rel 1e-12), matching core design §C.2 (F/F* at stop / landing
    // 2.16 / 0.254, 1.278 / 0.764, 1.26 / 0.777) and the F18 stay-landed ratios at
    // m_op_min (0.825 / 0.849 / 0.863 / 0.863; W0 report) to their printed digits.
    let table = [
        (presets::drawer(), 2.16, 0.254, 0.825),
        (presets::door(), 1.278, 0.764, 0.849),
        (presets::wall(), 1.26, 0.777, 0.863),
        (presets::heavy(), 1.26, 0.777, 0.863),
    ];
    for (sp, up_tab, land_tab, stay_tab) in table {
        let d = &sp.design;
        let s = &d.suspension;
        let model = ElementModel::new(s, s.b_r_t, 0.0, true);
        let n = derive::n_lift(s);
        let force = |g: f64| {
            let ev = model.eval(
                &ElementGeom {
                    g_m: g,
                    y_m: 0.0,
                    e: 1.0,
                    de_dx_per_m: 0.0,
                },
                0.0,
                T_REF_C,
            );
            ev.loads(ev.phi_wb).f_attract_n
        };
        let m_star = d.carriage.m_star_kg;
        let m_min = d.carriage.m_op_min_kg;
        let r_up = n * force(s.g_up_m) / (m_star * G0_MPS2);
        let r_land = n * force(s.g_land_m) / (m_star * G0_MPS2);
        let r_stay = n * force(s.g_land_m) / (m_min * G0_MPS2);
        for (got, want) in [
            (r_up, derive::stop_ratio(d, s.g_up_m, m_star)),
            (r_land, derive::stop_ratio(d, s.g_land_m, m_star)),
            (r_stay, derive::stop_ratio(d, s.g_land_m, m_min)),
        ] {
            assert!(
                (got - want).abs() <= 1e-12 * want,
                "{:?}: plant ratio {got} vs derive {want} ((E) rel 1e-12, LEV-006)",
                d.class
            );
        }
        let digits = |x: f64, tab: f64, dec: f64| (x - tab).abs() <= 0.5 * 10f64.powf(-dec) + 1e-12;
        assert!(
            digits(r_up, up_tab, if up_tab == 1.278 { 3.0 } else { 2.0 }),
            "{:?} stop {r_up} vs {up_tab}",
            d.class
        );
        assert!(
            digits(r_land, land_tab, 3.0),
            "{:?} landing {r_land} vs {land_tab}",
            d.class
        );
        assert!(
            digits(r_stay, stay_tab, 3.0),
            "{:?} stay-landed {r_stay} vs {stay_tab} (F18)",
            d.class
        );
        // LEV-006 limits: upper stop ≤ 1.30 (DOOR/WALL/HEAVY), stay-landed ≤ 0.90.
        if !matches!(d.class, fabrail_types::RailClass::Drawer) {
            assert!(r_up <= 1.30, "upper-stop ratio {r_up} ≤ 1.30 (LEV-006)");
        }
        assert!(r_stay <= 0.90, "stay-landed {r_stay} ≤ 0.90 (F18)");
    }
}
