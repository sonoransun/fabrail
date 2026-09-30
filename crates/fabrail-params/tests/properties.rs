//! Property tests: validation never panics and always names a `meta::FIELDS` path; the
//! seal round-trips; the closed forms keep their identities over their input ranges; the
//! discrete zero-power design places its target poles for any tuning in range
//! (core design §E.1 tolerance classes; PHYSICS §3, §15, §16, §16.1).

use fabrail_params::derive::{self, LevMode};
use fabrail_params::{SystemParams, crc, meta, presets};
use fabrail_types::{RailClass, fmath};
use proptest::prelude::*;

fn class_strategy() -> impl Strategy<Value = RailClass> {
    prop_oneof![
        Just(RailClass::Drawer),
        Just(RailClass::Door),
        Just(RailClass::Wall),
        Just(RailClass::Heavy),
    ]
}

/// Scales one of a set of influential fields by `k`.
fn perturb(p: &mut SystemParams, which: u8, k: f64) {
    match which % 12 {
        0 => p.design.suspension.g_up_m *= k,
        1 => p.design.suspension.g_land_m *= k,
        2 => p.design.carriage.m_rated_kg *= k,
        3 => p.design.suspension.pole_l_m *= k,
        4 => p.control.levitation.omega_ratio *= k,
        5 => p.control.motion.terminus.tau_s *= k,
        6 => p.safety.v_open_max_mps *= k,
        7 => p.design.power.v_bus_v *= k,
        8 => p.control.haptic.mass_ratio *= k,
        9 => p.design.rail.flange_t_m *= k,
        10 => p.design.suspension.i_pulse_a *= k,
        _ => p.control.motion.a_max_mps2 *= k,
    }
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 256, ..ProptestConfig::default() })]

    #[test]
    fn validate_never_panics_and_names_a_field(c in class_strategy(), which in 0u8..12, k in 0.2f64..5.0) {
        let mut p = presets::by_class(c);
        perturb(&mut p, which, k);
        if let Err(e) = p.validate() {
            prop_assert!(meta::find(e.path).is_some(), "error path {} is a FIELDS path", e.path);
            prop_assert!(!e.reason.is_empty());
        }
        // Deterministic.
        prop_assert_eq!(p.validate(), p.validate());
    }

    #[test]
    fn seal_roundtrip(c in class_strategy(), which in 0u8..12, k in 0.5f64..2.0) {
        let mut p = presets::by_class(c);
        perturb(&mut p, which, k);
        crc::seal(&mut p);
        prop_assert_eq!(crc::verify_seal(&p), Ok(()));
        let crc0 = p.safety.crc;
        crc::seal(&mut p);
        prop_assert_eq!(p.safety.crc, crc0, "seal is idempotent");
    }

    #[test]
    fn zero_power_mass_and_gap_are_inverses(g in 0.5e-3f64..20e-3, gs in 1e-3f64..15e-3, hp in 1e-3f64..20e-3, ms in 1.0f64..1e4) {
        let m = derive::zero_power_mass(g, gs, hp, ms);
        let back = derive::zero_power_gap(m, gs, hp, ms);
        prop_assert!(fmath::abs(back - g) <= 1e-12 * (1.0 + g), "(E) PHYSICS §3");
    }

    #[test]
    fn stop_distance_is_monotone_and_continuous(v in 1e-4f64..2.0, a in 0.01f64..5.0, j in 0.1f64..50.0) {
        let d = derive::stop_distance(v, a, j);
        let d2 = derive::stop_distance(v * 1.001, a, j);
        prop_assert!(d > 0.0 && d2 > d, "monotone in v (PHYSICS §16)");
        let vb = a * a / j;
        let lo = derive::stop_distance(vb * (1.0 - 1e-12), a, j);
        let hi = derive::stop_distance(vb, a, j);
        prop_assert!(fmath::abs(lo - hi) <= 1e-9 * hi, "continuous at v = a²/j");
    }

    #[test]
    fn terminus_blend_is_c2(v in 0.01f64..1.0, j in 0.5f64..50.0, tau in 0.05f64..2.0) {
        let b = derive::terminus_blend(v, j, tau);
        let v_end = v - j * b.t_b_s * b.t_b_s / 2.0;
        prop_assert!(fmath::abs(v_end - b.d_b_m / tau) <= 1e-10 * (v + 1.0), "speed continuity (PHYSICS §16.1)");
        prop_assert!(fmath::abs(j * b.t_b_s - b.d_b_m / (tau * tau)) <= 1e-9 * (1.0 + j * b.t_b_s), "deceleration continuity");
        prop_assert!(b.d_0_m > b.d_b_m && b.d_b_m > 0.0);
    }

    #[test]
    fn discrete_design_places_poles(c in class_strategy(), ratio in 2.0f64..3.5, te in 0.0f64..2.0e-3) {
        let mut p = presets::by_class(c);
        p.control.levitation.omega_ratio = ratio;
        p.control.levitation.tau_e_nominal_s = te;
        let t = derive::fast_period_s(&p);
        for mode in LevMode::ALL {
            if !derive::mode_active(&p, mode) {
                continue;
            }
            let k = derive::zp_gains_discrete(&p, mode);
            let poles = derive::zp_closed_loop_poles(&p, mode, &k, te);
            for target in derive::zp_target_poles_z(&p, mode) {
                let miss = poles
                    .iter()
                    .map(|q| fmath::hypot(q.0 - target.0, q.1 - target.1))
                    .fold(f64::INFINITY, fmath::fmin);
                // 1e-5 of ω·T plus 1e-5 absolute in z: when a lag pole lands next to a
                // target pole the pair is a near-double root, whose reported position is
                // √ε-conditioned (~1e-7–1e-6 in z); a wrong placement misses by far more.
                let s_scale = derive::zp_omega(&p, mode) * t;
                prop_assert!(miss <= 1e-5 * s_scale + 1e-5, "{:?} {:?}: miss {} (plan §2b.1)", c, mode, miss);
            }
        }
    }
}
