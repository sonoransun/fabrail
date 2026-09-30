//! LSM thrust: FOC abstraction, Kf(x) segment overlap, 6th-order ripple, cogging, normal force, back-EMF, ASC drag (PHYSICS §7). Owner: A6a.
//!
//! dq convention (W2 lead decision D5): amplitude-invariant Park transform, so the
//! electrical power is 1.5·(v_d·i_d + v_q·i_q). With F = K_f·i_q the power identity
//! F·v = 1.5·e_q·i_q (PRP-002) fixes the back-EMF e_q = K_e·v with
//! K_e = (2/3)·K_f ([`KE_PER_KF`]), and the shorted-winding (ASC) drag is
//! b_e = K_f·K_e/R/(1 + (ω_e·L/R)²) = (2/3)·K_f²/R/(1 + (ω_e·L/R)²). The plant derives
//! K_e from K_f(x) and never reads `design.lsm.k_e_v_s_per_m` (the design field must agree
//! with it: `params::validate`, PRP-002). 6th-harmonic ripple (1 %); cogging and normal
//! force F_n for the iron-core HEAVY stator.
//!
//! As implemented (PHYSICS §7; physics perspective §1.5):
//! * The inverter's dq current loop is a first-order lag [`TAU_CURRENT_S`] (the tier
//!   applies it exactly within each step, like the coil lag mode); |i_dq| ≤ i_max.
//! * Thrust F = K_f(x)·i_q + F_cog(x), with K_f(x) = K_f·w_seg(x)·(1 + r₆·cos 6θ_e),
//!   θ_e = π·x/τ_p. The segment weight w_seg dips by the overlap fraction o/L_seg at each
//!   stator joint of a segmented stator ([`segment_weight`]).
//! * Back-EMF e_q = K_e(x)·v = (2/3)·K_f(x)·v, so the electromechanical identity
//!   F_em·v = 1.5·e_q·i_q holds exactly in every mode (PRP-002).
//! * Iron-core only: cogging F_cog = F_c·sin 6θ_e (conservative, potential
//!   [`cogging_energy_j`]) and the normal attraction F_n = F_n,rated·max(0, 1 + i_d/i_max)
//!   toward the stator, which is on the rail above the mover (it relieves the lift
//!   elements).
//! * STO: gates off; the currents decay to 0 through the lag. ASC: the phases are
//!   shorted; the steady shorted-winding solution of v_q = R·i_q + ω_e·L·i_d + e_q = 0,
//!   v_d = R·i_d − ω_e·L·i_q = 0 gives i_q = −(e_q/R)/(1 + (ω_e·L/R)²),
//!   i_d = (ω_e·L/R)·i_q and the drag F = K_f(x)·i_q = −b_e(ω_e)·v
//!   ([`asc_drag_n_s_per_m`]); the copper loss 1.5·R·(i_q² + i_d²) equals the braking
//!   power b_e·v² exactly (energy balance, no bus power).
use core::f64::consts::PI;

use fabrail_params::{LsmDesign, LsmKind};
use fabrail_types::{LsmMode, fmath};

/// Inverter dq current-loop time constant τ_q, s (physics perspective §1.5: "i_q, i_d
/// lag τ_q = 0.1 ms"; PHYSICS §7).
pub const TAU_CURRENT_S: f64 = 1.0e-4;

/// Back-EMF constant per thrust constant in the amplitude-invariant dq frame,
/// K_e/K_f = 2/3 (F·v = 1.5·e_q·i_q with F = K_f·i_q; W2 lead decision D5, PRP-002,
/// PHYSICS §7).
pub const KE_PER_KF: f64 = 2.0 / 3.0;

/// Back-EMF constant K_e = (2/3)·K_f for a thrust constant `k_f`, V·s/m (D5; PHYSICS §7).
pub fn k_e_from_kf(k_f: f64) -> f64 {
    KE_PER_KF * k_f
}

/// LSM loads and electrical quantities for one evaluation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LsmForces {
    /// Thrust along x (including ripple, cogging, ASC drag), N.
    pub fx_n: f64,
    /// Normal attraction (iron-core only), N.
    pub fn_n: f64,
    /// q-axis back-EMF, V.
    pub e_q_v: f64,
    /// Electrical input power, W.
    pub p_elec_w: f64,
    /// Actual q-axis current (ASC: the shorted-winding current), A.
    pub iq_a: f64,
    /// Actual d-axis current, A.
    pub id_a: f64,
    /// Winding copper loss (ASC: the braking power b_e·v²), W.
    pub p_copper_w: f64,
    /// Cogging part of `fx_n`, N.
    pub f_cog_n: f64,
}

/// Electrical angle θ_e = π·x/τ_p, rad (PHYSICS §7).
pub fn electrical_angle_rad(d: &LsmDesign, x_m: f64) -> f64 {
    if d.tau_p_m > 0.0 {
        PI * x_m / d.tau_p_m
    } else {
        0.0
    }
}

/// Segment hand-over weight w_seg(x) ∈ (0, 1] of a segmented long stator: 1 away from
/// the joints at x = k·L_seg, dipping by the overlap fraction o/L_seg at a joint along a
/// raised cosine of total width o (C¹). Design choice \[UNCERTAIN\]: the hand-over
/// geometry is not modelled in detail; the dip depth equals the overlap fraction
/// (10 % for WALL and HEAVY) (PHYSICS §7).
pub fn segment_weight(d: &LsmDesign, x_m: f64) -> f64 {
    let (l, o) = (d.segment_len_m, d.segment_overlap_m);
    if !(l > 0.0 && o > 0.0) {
        return 1.0;
    }
    let u = x_m / l;
    let dist = fmath::abs(u - fmath::round(u)) * l;
    let half = 0.5 * fmath::fmin(o, l);
    if dist >= half {
        return 1.0;
    }
    let depth = fmath::fmin(o / l, 1.0);
    1.0 - depth * 0.5 * (1.0 + fmath::cos(PI * dist / half))
}

/// Position-dependent thrust constant K_f(x) = K_f·w_seg(x)·(1 + r₆·cos 6θ_e), N/A
/// (PHYSICS §7).
pub fn kf_eff(d: &LsmDesign, x_m: f64) -> f64 {
    let th = electrical_angle_rad(d, x_m);
    d.k_f_n_per_a * segment_weight(d, x_m) * (1.0 + d.ripple_frac * fmath::cos(6.0 * th))
}

/// Cogging force F_cog = F_c·sin 6θ_e (iron-core stator only), N (PHYSICS §7).
pub fn cogging_n(d: &LsmDesign, x_m: f64) -> f64 {
    if matches!(d.kind, LsmKind::IronCoreSegmented) && d.cogging_n != 0.0 {
        d.cogging_n * fmath::sin(6.0 * electrical_angle_rad(d, x_m))
    } else {
        0.0
    }
}

/// Cogging potential U with F_cog = −dU/dx: U = F_c·τ_p/(6π)·cos 6θ_e, J (PHYSICS §7).
pub fn cogging_energy_j(d: &LsmDesign, x_m: f64) -> f64 {
    if matches!(d.kind, LsmKind::IronCoreSegmented) && d.cogging_n != 0.0 && d.tau_p_m > 0.0 {
        d.cogging_n * d.tau_p_m / (6.0 * PI) * fmath::cos(6.0 * electrical_angle_rad(d, x_m))
    } else {
        0.0
    }
}

/// Phase currents from dq (amplitude-invariant inverse Park at θ_e), A (PHYSICS §7).
pub fn dq_to_abc(d: &LsmDesign, x_m: f64, iq_a: f64, id_a: f64) -> [f64; 3] {
    let th = electrical_angle_rad(d, x_m);
    let ph = 2.0 * PI / 3.0;
    [
        id_a * fmath::cos(th) - iq_a * fmath::sin(th),
        id_a * fmath::cos(th - ph) - iq_a * fmath::sin(th - ph),
        id_a * fmath::cos(th + ph) - iq_a * fmath::sin(th + ph),
    ]
}

/// LSM loads at position `x_m`, velocity `v_mps`, actual dq currents and inverter `mode`
/// (PHYSICS §7). In ASC the currents are the shorted-winding solution and `iq_a`/`id_a`
/// are ignored.
pub fn thrust(
    d: &LsmDesign,
    x_m: f64,
    v_mps: f64,
    iq_a: f64,
    id_a: f64,
    mode: LsmMode,
) -> LsmForces {
    let kf = kf_eff(d, x_m);
    let f_cog = cogging_n(d, x_m);
    let e_q = k_e_from_kf(kf) * v_mps;
    let iron = matches!(d.kind, LsmKind::IronCoreSegmented);
    let normal = |id: f64| {
        if iron && d.f_normal_n > 0.0 {
            let r = if d.i_max_a > 0.0 { id / d.i_max_a } else { 0.0 };
            d.f_normal_n * fmath::fmax(0.0, 1.0 + r)
        } else {
            0.0
        }
    };
    match mode {
        LsmMode::Asc => {
            let w_e = PI * v_mps / fmath::fmax(d.tau_p_m, f64::MIN_POSITIVE);
            // K_e(x) = (2/3)·K_f(x): the same w_seg·ripple factor as the thrust (D5).
            let b = asc_drag_n_s_per_m(kf, k_e_from_kf(kf), d.r_phase_ohm, d.l_phase_h, w_e);
            let f_em = -b * v_mps;
            let iq = if kf != 0.0 { f_em / kf } else { 0.0 };
            let id = if d.r_phase_ohm > 0.0 {
                w_e * d.l_phase_h / d.r_phase_ohm * iq
            } else {
                0.0
            };
            LsmForces {
                fx_n: f_em + f_cog,
                fn_n: normal(id),
                e_q_v: e_q,
                p_elec_w: 0.0,
                iq_a: iq,
                id_a: id,
                // = 1.5·R·(i_q² + i_d²) exactly for K_e = (2/3)·K_f (D5).
                p_copper_w: b * v_mps * v_mps,
                f_cog_n: f_cog,
            }
        }
        LsmMode::Run | LsmMode::Sto => {
            let p_cu = 1.5 * d.r_phase_ohm * (iq_a * iq_a + id_a * id_a);
            LsmForces {
                fx_n: kf * iq_a + f_cog,
                fn_n: normal(id_a),
                e_q_v: e_q,
                p_elec_w: p_cu + 1.5 * e_q * iq_a,
                iq_a,
                id_a,
                p_copper_w: p_cu,
                f_cog_n: f_cog,
            }
        }
    }
}

/// Active-short-circuit drag coefficient b_e = K_f·K_e/R / (1 + (ω_e·L/R)²), N·s/m, at
/// electrical angular speed `omega_e_radps`: the steady shorted-winding solution in the
/// amplitude-invariant dq frame (W2 lead decision D5; PHYSICS §7). With the plant's
/// K_e = (2/3)·K_f ([`k_e_from_kf`]) this is (2/3)·K_f²/R at low speed; the Wave-0 form
/// 1.5·K_f·K_e/R with K_e = K_f/√3 (≈ 0.866·K_f²/R) contradicted F·v = 1.5·e_q·i_q and
/// was replaced (D5). 0 for a non-positive resistance.
pub fn asc_drag_n_s_per_m(k_f: f64, k_e: f64, r_ohm: f64, l_h: f64, omega_e_radps: f64) -> f64 {
    if r_ohm <= 0.0 || r_ohm.is_nan() {
        return 0.0;
    }
    let x = omega_e_radps * l_h / r_ohm;
    k_f * k_e / r_ohm / (1.0 + x * x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    #[test]
    fn asc_drag_low_speed_limit() {
        // ω_e → 0: b_e = K_f·K_e/R (D5).
        assert!((asc_drag_n_s_per_m(10.0, 5.0, 2.0, 1e-3, 0.0) - 25.0).abs() < 1e-12);
        // ω_e·L/R = 1 halves it.
        assert!((asc_drag_n_s_per_m(10.0, 5.0, 2.0, 1e-3, 2000.0) - 12.5).abs() < 1e-12);
        assert_eq!(asc_drag_n_s_per_m(10.0, 5.0, 0.0, 1e-3, 0.0), 0.0);
        // With the plant's K_e = (2/3)·K_f: (2/3)·K_f²/R.
        assert!((asc_drag_n_s_per_m(30.0, k_e_from_kf(30.0), 2.0, 0.0, 0.0) - 300.0).abs() < 1e-9);
    }

    // req: FAB-PRP-002
    #[test]
    fn kf_ke_and_power_balance() {
        // Contract (PRP-002 as amended by W2 lead decision D5; PHYSICS §7): in the
        // amplitude-invariant dq frame every preset has K_e = (2/3)·K_f, the plant's
        // back-EMF constant is K_e(x) = (2/3)·K_f(x) (e_q/v), and F_em·v = 1.5·e_q·i_q (F_em = F − F_cog) holds in Run, STO and ASC
        // across positions, speeds and currents (rel 1e-9). In ASC the drag equals the
        // closed form b_e = (2/3)·K_f²/R/(1 + (ω_e·L/R)²) and the copper loss
        // 1.5·R·(i_q² + i_d²) equals the braking power b_e·v² (rel 1e-9): no energy is
        // created or lost.
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let d = p.design.lsm;
            // The design field agrees with the plant's dq convention (presets, D5).
            assert!(
                (d.k_e_v_s_per_m - KE_PER_KF * d.k_f_n_per_a).abs() <= 1e-12 * d.k_f_n_per_a,
                "preset K_e {} = (2/3)·K_f {} (rel 1e-12, D5, PRP-002)",
                d.k_e_v_s_per_m,
                KE_PER_KF * d.k_f_n_per_a
            );
            for &x in &[0.0, 0.0123, 0.5, 0.99, 1.0, 1.05, 2.0, 3.3] {
                let kf = kf_eff(&d, x);
                for &v in &[-0.3, -0.01, 0.02, 0.18, 1.5] {
                    for &(iq, id) in &[(0.0, 0.0), (1.7, 0.0), (-3.0, 0.5), (5.0, -1.0)] {
                        for mode in [LsmMode::Run, LsmMode::Sto, LsmMode::Asc] {
                            let f = thrust(&d, x, v, iq, id, mode);
                            let ke = f.e_q_v / v;
                            assert!(
                                (ke - 2.0 / 3.0 * kf).abs() <= 1e-12 * kf.abs().max(1e-12),
                                "K_e(x) {ke} = (2/3)·K_f(x) {} (rel 1e-12, D5, PRP-002)",
                                2.0 / 3.0 * kf
                            );
                            let lhs = (f.fx_n - f.f_cog_n) * v;
                            let rhs = 1.5 * f.e_q_v * f.iq_a;
                            let scale = lhs.abs().max(1e-12);
                            assert!(
                                (lhs - rhs).abs() <= 1e-9 * scale,
                                "F·v {lhs} = 1.5·e_q·i_q {rhs} (rel 1e-9, PRP-002) at x {x} v {v} {mode:?}"
                            );
                            // Power into the machine = copper + converted power (Run/STO); ASC
                            // dissipates the braking power in the windings.
                            match mode {
                                LsmMode::Asc => {
                                    let cu =
                                        1.5 * d.r_phase_ohm * (f.iq_a * f.iq_a + f.id_a * f.id_a);
                                    assert!(
                                        (f.p_copper_w + lhs).abs()
                                            <= 1e-9 * scale.max(f.p_copper_w),
                                        "ASC braking power {} = copper loss {} (rel 1e-9, D5)",
                                        -lhs,
                                        f.p_copper_w
                                    );
                                    assert!(
                                        (cu - f.p_copper_w).abs() <= 1e-9 * cu.max(1e-12),
                                        "ASC 1.5·R·|i_dq|² {cu} = b_e·v² {} (rel 1e-9, D5)",
                                        f.p_copper_w
                                    );
                                    assert!(f.fx_n * v <= f.f_cog_n * v + 1e-12, "ASC brakes");
                                    assert_eq!(f.p_elec_w, 0.0, "ASC draws no bus power");
                                }
                                _ => assert!(
                                    (f.p_elec_w - f.p_copper_w - rhs).abs()
                                        <= 1e-9 * f.p_elec_w.abs().max(1e-12)
                                ),
                            }
                        }
                    }
                }
            }
            // The low-speed ASC coefficient equals (2/3)·K_f(x)²/R.
            let x = d.tau_p_m / 12.0 + 0.3;
            let v = 1e-6;
            let f = thrust(&d, x, v, 0.0, 0.0, LsmMode::Asc);
            let kf = kf_eff(&d, x);
            let b0 = 2.0 / 3.0 * kf * kf / d.r_phase_ohm;
            let b = -(f.fx_n - f.f_cog_n) / v;
            assert!(
                (b - b0).abs() <= 1e-6 * b0,
                "b_e {b} = (2/3)·K_f²/R {b0} at ω_e → 0 (rel 1e-6, D5)"
            );
        }
    }

    #[test]
    fn ripple_segments_and_cogging() {
        let wall = presets::wall().design.lsm;
        // Segment joint at x = 1 m: dip by o/L = 10 %; smooth away from it.
        assert!((segment_weight(&wall, 1.0) - 0.9).abs() < 1e-12);
        assert_eq!(segment_weight(&wall, 1.5), 1.0);
        assert!(segment_weight(&wall, 1.04) > 0.9 && segment_weight(&wall, 1.04) < 1.0);
        let door = presets::door().design.lsm;
        assert_eq!(segment_weight(&door, 0.5), 1.0, "continuous stator");
        // 1 % sixth-harmonic ripple.
        let k0 = kf_eff(&door, 0.0);
        assert!((k0 - door.k_f_n_per_a * 1.01).abs() < 1e-12);
        // Ironless: no cogging or normal force; iron-core: conservative cogging.
        assert_eq!(cogging_n(&door, 0.1), 0.0);
        let heavy = presets::heavy().design.lsm;
        for &x in &[0.01, 0.37, 1.23] {
            let dx = 1e-7;
            let fd =
                -(cogging_energy_j(&heavy, x + dx) - cogging_energy_j(&heavy, x - dx)) / (2.0 * dx);
            assert!((cogging_n(&heavy, x) - fd).abs() <= 1e-6 * heavy.cogging_n);
        }
        let f = thrust(&heavy, 0.3, 0.0, 0.0, 0.0, LsmMode::Sto);
        assert!(
            (f.fn_n - heavy.f_normal_n).abs() < 1e-9,
            "iron-core F_n at i_d = 0"
        );
        // Inverse Park: balanced phases.
        let abc = dq_to_abc(&door, 0.017, 2.0, 0.3);
        assert!((abc[0] + abc[1] + abc[2]).abs() < 1e-12);
    }
}
