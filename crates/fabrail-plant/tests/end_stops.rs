//! Physical end stops at the class geometry (W2 lead decision D3; PHYSICS §8, §9).
//!
//! Contract: a free (levitated, LSM in STO) or landed carriage driven only by floor motion
//! stays between the closed-end jamb / DRAWER carcass stop and the open-end rail stop
//! behind the bumper (`contact::end_stops`), on every tier that has a travel axis, with the
//! same geometry on Carriage6 and Travel1d. The closed-end excursion stays inside the
//! 10 mm SLP band of the presets (`safety.x_min_m` = x_closed − 10 mm), so floor-driven
//! motion alone never reaches the monitor's position limit at the closed end. Tolerances
//! are stated per assertion with their source.

use core::f64::consts::PI;

use fabrail_params::derive::G0_MPS2;
use fabrail_params::{SystemParams, presets};
use fabrail_plant::contact::{self, end_stops};
use fabrail_plant::params::{Environment, Mismatch, PlantParams};
use fabrail_plant::{InitLev, InitialState, ModelConfig, PlantInputs, Tier, build};

fn plant(sp: &SystemParams) -> PlantParams {
    PlantParams::from_design(&sp.design, &Mismatch::default(), &Environment::default())
}

fn cfg(tier: Tier) -> ModelConfig {
    ModelConfig {
        tier,
        ..ModelConfig::default()
    }
}

/// Runs `dur_s` of floor acceleration `a(t)` along x and returns (min x, max x, final x,
/// max end-stop force, every state finite).
fn run(
    p: &PlantParams,
    tier: Tier,
    init: InitialState,
    u: &PlantInputs,
    dur_s: f64,
    a: impl Fn(f64) -> f64,
) -> (f64, f64, f64, f64, bool) {
    let mut m = build(&cfg(tier), p).expect("build");
    m.reset(&init);
    let h = tier.nominal_h_s();
    let n = (dur_s / h).round() as u64;
    let (mut lo, mut hi, mut f_max, mut finite) = (init.x_m, init.x_m, 0.0f64, true);
    for k in 0..n {
        let t = k as f64 * h;
        let floor = [
            [a(t), 0.0, 0.0],
            [a(t + 0.5 * h), 0.0, 0.0],
            [a(t + h), 0.0, 0.0],
        ];
        m.step(t, h, u, &floor);
        let tr = m.truth();
        finite &= tr.x_m.is_finite() && tr.v_mps.is_finite();
        lo = lo.min(tr.x_m);
        hi = hi.max(tr.x_m);
        f_max = f_max.max(tr.f_contact.end_stop_n);
    }
    (lo, hi, m.truth().x_m, f_max, finite)
}

#[test]
fn floor_driven_motion_stays_between_the_physical_stops() {
    // Every class free along x on Travel1d (levitated with ideal heave, or landed on its
    // rollers; LSM in STO, no controller), starting at the closed position, under 20 s of
    // harmonic floor motion:
    // * 0.5 g at 1 Hz (floor velocity ±0.78 m/s; twice the villa smoke's 0.25 g): the
    //   body stays inside the 10 mm SLP band at the closed end (min x ≥ safety.x_min_m =
    //   x_closed − 10 mm, presets), so floor motion alone never reaches the monitor's
    //   position limit there (D3);
    // * 0.5 g at 0.7 Hz (±1.1 m/s, repeated rebounds reach ≈ 3 m/s relative): the body
    //   still stays between the stops within the dynamic penetration 3.5 m/s·√(m/k) of
    //   each rigid face (PHYSICS §8), and every state stays finite (the W2 split bristle
    //   update; an RK4-embedded bristle diverged here on landed WALL/HEAVY).
    for sp in [
        presets::drawer(),
        presets::door(),
        presets::wall(),
        presets::heavy(),
    ] {
        let p = plant(&sp);
        let st = end_stops(&p.design);
        let x0 = p.design.rail.x_closed_m;
        let m = p.m_body_kg;
        let (k, _) = contact::end_stop_law(&p, m, Tier::Travel1d.nominal_h_s());
        let pen = 3.5 * (m / k).sqrt();
        let class = sp.design.class;
        for lev in [InitLev::Levitated, InitLev::Landed] {
            for (a_g, f_hz, slp) in [(0.5, 1.0, true), (0.5, 0.7, false)] {
                let a0 = a_g * G0_MPS2;
                let (lo, hi, _, f_stop, ok) = run(
                    &p,
                    Tier::Travel1d,
                    InitialState {
                        lev,
                        x_m: x0,
                        ..InitialState::default()
                    },
                    &PlantInputs::default(),
                    20.0,
                    |t| a0 * (2.0 * PI * f_hz * t).sin(),
                );
                let case = format!("{class:?} {lev:?} {a_g} g {f_hz} Hz");
                assert!(ok, "{case}: state stays finite");
                // An end stop engaged (a seated DRAWER meets its push-stop through the seat
                // law, reported as the seat force instead).
                assert!(
                    f_stop > 0.0 || lo < x0,
                    "{case}: a closed-end stop was engaged"
                );
                if slp {
                    assert!(
                        lo >= sp.safety.x_min_m,
                        "{case}: min x {:.2} mm inside the SLP band x_min {:.2} mm (presets, D3)",
                        1e3 * lo,
                        1e3 * sp.safety.x_min_m
                    );
                }
                assert!(
                    lo >= st.closed_m - pen,
                    "{case}: min x {:.2} mm ≥ closed stop {:.2} mm − {:.2} mm (D3, §8)",
                    1e3 * lo,
                    1e3 * st.closed_m,
                    1e3 * pen
                );
                assert!(
                    hi <= st.open_m + pen,
                    "{case}: max x {hi:.4} m ≤ open stop {:.4} m + {pen:.4} m (D3, §8)",
                    st.open_m
                );
            }
        }
    }
}

#[test]
fn carriage6_and_travel1d_share_the_stop_geometry() {
    // A landed WALL on its rollers (coils at 0 A, LSM in STO) pushed toward the closed end
    // by a steady 0.3 g floor acceleration comes to rest on the jamb at the same place on
    // Carriage6 (50 µs) and Travel1d (1 ms): within the static deflection m·a/k of the
    // closed face (±20 %, PHYSICS §8) and within 0.1 mm of each other. Thrown at the open
    // end at 2 m/s (KE 1 kJ, above the 563 J bumper rating), it compresses the bumper and
    // is held by the rail stop behind it on both tiers: max x ≤ open stop + v·√(m/k).
    let sp = presets::wall();
    let p = plant(&sp);
    let st = end_stops(&p.design);
    let m = p.m_body_kg;
    let a = 0.3 * G0_MPS2;
    let u = PlantInputs {
        coil_enabled: true,
        ..PlantInputs::default()
    };
    let landed = |x_m: f64, v_mps: f64| InitialState {
        lev: InitLev::Landed,
        x_m,
        v_mps,
        ..InitialState::default()
    };
    let mut rest = [0.0; 2];
    for (i, tier) in [Tier::Carriage6, Tier::Travel1d].into_iter().enumerate() {
        let (k, _) = contact::end_stop_law(&p, m, tier.nominal_h_s());
        let (lo, _, x_end, _, ok) = run(&p, tier, landed(0.01, 0.0), &u, 2.0, |_| a);
        assert!(ok, "{tier:?}: finite");
        let static_def = m * a / k;
        assert!(
            (st.closed_m - x_end - static_def).abs() <= 0.2 * static_def + 2e-5,
            "{tier:?}: rest {:.4} mm = closed face {:.4} mm − m·a/k {:.4} mm (±20 %, §8)",
            1e3 * x_end,
            1e3 * st.closed_m,
            1e3 * static_def
        );
        assert!(lo >= sp.safety.x_min_m, "{tier:?}: inside the SLP band");
        rest[i] = x_end;
        let v0 = 2.0;
        let (_, hi, _, f_stop, ok) = run(
            &p,
            tier,
            landed(st.open_bumper_m - 0.005, v0),
            &u,
            0.5,
            |_| 0.0,
        );
        assert!(
            ok && f_stop > 0.0,
            "{tier:?}: the rail stop behind the bumper engaged"
        );
        let pen = v0 * (m / k).sqrt();
        assert!(
            hi <= st.open_m + pen,
            "{tier:?}: max x {hi:.4} m ≤ open stop {:.4} m + v·√(m/k) {pen:.4} m (D3)",
            st.open_m
        );
    }
    assert!(
        (rest[0] - rest[1]).abs() <= 1.0e-4,
        "Carriage6 {:.4} mm vs Travel1d {:.4} mm rest on the jamb (±0.1 mm, one geometry)",
        1e3 * rest[0],
        1e3 * rest[1]
    );
}

#[test]
fn fast_sliding_landed_body_keeps_coulomb_friction() {
    // The split bristle update (hold z through RK4, advance it exactly afterwards;
    // PHYSICS §8, §24): a landed WALL thrown along its rollers at 2 m/s mid-travel on
    // Carriage6 and Travel1d (Travel1d's RK4-embedded bristle had λh ≈ 130 at this speed
    // and diverged in W2 shaking runs) keeps a finite state and a sliding friction of
    // μ_c·N (μ 0.003, plan §2b item 19) opposing the motion within 1 % (core design §E.2
    // sliding_force tolerance) once the presliding transient has passed.
    let sp = presets::wall();
    let p = plant(&sp);
    let mu = sp.design.landing.rollers_mu;
    let u = PlantInputs {
        coil_enabled: true,
        ..PlantInputs::default()
    };
    for tier in [Tier::Carriage6, Tier::Travel1d] {
        let mut m = build(&cfg(tier), &p).expect("build");
        m.reset(&InitialState {
            lev: InitLev::Landed,
            x_m: 2.0,
            v_mps: 2.0,
            ..InitialState::default()
        });
        let h = tier.nominal_h_s();
        let n = (0.3 / h).round() as u64;
        for k in 0..n {
            m.step(k as f64 * h, h, &u, &[[0.0; 3]; 3]);
            let tr = m.truth();
            assert!(
                tr.x_m.is_finite() && tr.v_mps.is_finite(),
                "{tier:?}: finite"
            );
            if k as f64 * h > 0.05 {
                let n_r = tr.f_contact.rollers_n;
                let f_t = tr.f_contact.tangential_x_n;
                assert!(
                    tr.v_mps > 0.0 && n_r > 0.0 && (f_t + mu * n_r).abs() <= 0.01 * mu * n_r,
                    "{tier:?} t {:.3}: F_t {f_t:.3} N vs −μ·N {:.3} N (1 %, §E.2)",
                    k as f64 * h,
                    -mu * n_r
                );
            }
        }
    }
}
