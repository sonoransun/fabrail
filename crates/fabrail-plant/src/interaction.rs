//! Human interaction: hand impedance and ISO/TS 15066 obstacle bodies (PHYSICS §22). Owner: A6b.
//!
//! The hand is an arm impedance (m, b, k; core design §C.5: m 1–2 kg, b 5–20 N·s/m,
//! k 200–1000 N/m) driven by a force profile from the scenario; an obstacle is an ISO/TS
//! 15066 body region in series with the closing-edge seal k_edge (≤ 10 N/mm, F6).
//!
//! Laws (PHYSICS §22; physics perspective §1.11):
//!
//! * hand: the hand rides on the body front, so the force it applies is
//!   F_h = F_drive − b·ẋ − k·x_rel − m·ẍ (x_rel measured from the contact point, the
//!   arm's anchor); a grip can pull, a push-only contact carries force only in the
//!   direction of the driving force and separates when the body moves away (push-to-
//!   open);
//! * obstacle: the closing edge (moving toward −x, the closed stop) meets the body
//!   surface at `x_m`; the normal force is k_s·δ with δ = x_body − x_edge and
//!   k_s = k_edge·k_body/(k_edge + k_body) — pure springs, as ISO/TS 15066:2016 Annex A
//!   models the energy transfer. `Clamped` bodies are backed by the frame; `Free` bodies
//!   are a mass m_H pushed by the edge ([`FreeBody`]); the closed forms are
//!   F_max = v·√(m·k_s) (clamped, `derive::clamp_force`) and v·√(μ·k_s),
//!   μ = (1/m_H + 1/m)⁻¹ (transient);
//! * disturbance profiles: lean and smoke-control Δp are symmetric trapezoids (plan §2b
//!   item 11: lean rise ≥ 1 s, smoke ramp ≥ 2 s), hand drive forces piecewise linear and
//!   released after the last point (`fabrail.scenario/1` events `lean`, `smoke_dp`,
//!   `hand`).

use fabrail_types::fmath;

use crate::contact::penalty_normal_n;

/// ISO/TS 15066 body region used as an obstacle (core design §C.5).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum ObstacleBody {
    /// Chest: 25 N/mm, 40 kg.
    #[default]
    Chest,
    /// Hand / fingers: 75 N/mm, 0.6 kg.
    Hand,
    /// Face: 75 N/mm, 4.4 kg \[M\].
    Face,
    /// Lower leg: 60 N/mm, 75 kg \[M\] \[UNCERTAIN\].
    Leg,
}

impl ObstacleBody {
    /// Every body region.
    pub const ALL: &'static [ObstacleBody] = &[
        ObstacleBody::Chest,
        ObstacleBody::Hand,
        ObstacleBody::Face,
        ObstacleBody::Leg,
    ];

    /// Effective spring constant, N/m (ISO/TS 15066:2016 Annex A, as tabulated in core
    /// design §C.5).
    pub const fn k_n_per_m(self) -> f64 {
        match self {
            ObstacleBody::Chest => 25.0e3,
            ObstacleBody::Hand => 75.0e3,
            ObstacleBody::Face => 75.0e3,
            ObstacleBody::Leg => 60.0e3,
        }
    }

    /// Effective mass, kg (ISO/TS 15066:2016 Annex A effective masses; the leg value is
    /// \[UNCERTAIN\], core design §C.5 marks face and leg as roadmap \[M\]).
    pub const fn m_eff_kg(self) -> f64 {
        match self {
            ObstacleBody::Chest => 40.0,
            ObstacleBody::Hand => 0.6,
            ObstacleBody::Face => 4.4,
            ObstacleBody::Leg => 75.0,
        }
    }

    /// Quasi-static contact force limit F_qs, N (ISO/TS 15066:2016 Table A.2 as carried
    /// by the physics perspective §1.11: hand 140, chest 140, face 65 \[M\], lower leg
    /// 130 \[M\]). Reported against, never a certification claim.
    pub const fn f_qs_n(self) -> f64 {
        match self {
            ObstacleBody::Chest => 140.0,
            ObstacleBody::Hand => 140.0,
            ObstacleBody::Face => 65.0,
            ObstacleBody::Leg => 130.0,
        }
    }

    /// Lower-case name as used in `fabrail.scenario/1` obstacle events.
    pub const fn name(self) -> &'static str {
        match self {
            ObstacleBody::Chest => "chest",
            ObstacleBody::Hand => "hand",
            ObstacleBody::Face => "face",
            ObstacleBody::Leg => "leg",
        }
    }
}

/// A hand in contact with the body front at this step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HandInput {
    /// Muscle driving force along x from the scenario profile, N.
    pub f_drive_n: f64,
    /// Arm effective mass, kg.
    pub m_kg: f64,
    /// Arm damping, N·s/m.
    pub b_n_s_per_m: f64,
    /// Arm stiffness about the contact anchor, N/m.
    pub k_n_per_m: f64,
    /// Grip contact (can pull); `false` = push-only contact that separates when the
    /// body moves away (push-to-open).
    pub grip: bool,
}

impl HandInput {
    /// The default arm impedance m 1.5 kg, b 10 N·s/m, k 500 N/m (mid-range of core
    /// design §C.5; the `fabrail.scenario/1` hand default) with driving force `f_drive_n`.
    pub fn arm(f_drive_n: f64, grip: bool) -> HandInput {
        HandInput {
            f_drive_n,
            m_kg: 1.5,
            b_n_s_per_m: 10.0,
            k_n_per_m: 500.0,
            grip,
        }
    }
}

/// An obstacle body in the closing path.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Obstacle {
    /// Body region.
    pub body: ObstacleBody,
    /// Rail position of the body surface, m.
    pub x_m: f64,
    /// Clamped against a rigid backing (true) or free to be pushed (false).
    pub clamped: bool,
    /// Closing-edge seal stiffness in series with the body, N/m.
    pub k_edge_n_per_m: f64,
}

/// Series stiffness k₁·k₂/(k₁ + k₂), N/m (PHYSICS §22). Returns 0 when either input is
/// ≤ 0 or non-finite (no load path).
pub fn series_k(k1_n_per_m: f64, k2_n_per_m: f64) -> f64 {
    if !(k1_n_per_m.is_finite() && k2_n_per_m.is_finite()) || k1_n_per_m <= 0.0 || k2_n_per_m <= 0.0
    {
        return 0.0;
    }
    k1_n_per_m * k2_n_per_m / (k1_n_per_m + k2_n_per_m)
}

/// Force the hand applies to the body along x, N, given the displacement since contact
/// `x_rel_m`, the body velocity and acceleration (PHYSICS §22):
/// F = F_drive − b·ẋ − k·x_rel − m·ẍ. A push-only contact (`grip == false`) carries
/// force only in the direction of F_drive (0 when F_drive is 0: released). Non-finite
/// inputs give 0.
pub fn hand_force_n(h: &HandInput, x_rel_m: f64, v_mps: f64, a_mps2: f64) -> f64 {
    if !(h.f_drive_n.is_finite() && x_rel_m.is_finite() && v_mps.is_finite() && a_mps2.is_finite())
    {
        return 0.0;
    }
    let f = h.f_drive_n
        - h.b_n_s_per_m.max(0.0) * v_mps
        - h.k_n_per_m.max(0.0) * x_rel_m
        - h.m_kg.max(0.0) * a_mps2;
    if h.grip {
        f
    } else if h.f_drive_n > 0.0 {
        f.max(0.0)
    } else if h.f_drive_n < 0.0 {
        f.min(0.0)
    } else {
        0.0
    }
}

/// Effective series stiffness of the edge seal and the body, N/m: `series_k(k_edge,
/// k_body)`, or the body alone when no seal is fitted (k_edge ≤ 0) (F6; PHYSICS §22).
pub fn obstacle_k_n_per_m(o: &Obstacle) -> f64 {
    let kb = o.body.k_n_per_m();
    if o.k_edge_n_per_m.is_finite() && o.k_edge_n_per_m > 0.0 {
        series_k(kb, o.k_edge_n_per_m)
    } else {
        kb
    }
}

/// Contact force between the closing edge at `x_edge_m` and the obstacle, N (≥ 0,
/// through the series k_edge + body stiffness). The edge closes toward −x, so contact
/// means x_edge < `o.x_m`; the force acts on the moving body along +x (it resists the
/// closing). `v_mps` is the edge velocity relative to the body surface (the body's own
/// velocity is 0 when clamped); the springs are undamped, so it only gates NaN inputs.
pub fn obstacle_force_n(o: &Obstacle, x_edge_m: f64, v_mps: f64) -> f64 {
    if !(o.x_m.is_finite() && x_edge_m.is_finite() && v_mps.is_finite()) {
        return 0.0;
    }
    penalty_normal_n(obstacle_k_n_per_m(o), 0.0, o.x_m - x_edge_m, -v_mps)
}

/// ISO/TS 15066 closed-form peak force of an edge closing at `v_rel_mps` on the obstacle:
/// clamped v·√(m·k_s) (all of the device's kinetic energy into the springs), free
/// v·√(μ·k_s) with μ = (1/m_H + 1/m)⁻¹ (ISO/TS 15066:2016 Annex A, transient contact), N.
pub fn peak_force_n(o: &Obstacle, m_device_kg: f64, v_rel_mps: f64) -> f64 {
    let k = obstacle_k_n_per_m(o);
    let m = if o.clamped {
        m_device_kg
    } else {
        let mh = o.body.m_eff_kg();
        if m_device_kg > 0.0 {
            mh * m_device_kg / (mh + m_device_kg)
        } else {
            0.0
        }
    };
    if !(k > 0.0 && m > 0.0) {
        return 0.0;
    }
    v_rel_mps.abs() * fmath::sqrt(m * k)
}

/// A free (unclamped) obstacle body: its surface position and velocity along x, pushed
/// by the closing edge (ISO/TS 15066 transient contact; PHYSICS §22). The tier carries
/// one while a free obstacle is present and writes `x_m` back into the [`Obstacle`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FreeBody {
    /// Body surface position, m.
    pub x_m: f64,
    /// Body velocity, m/s.
    pub v_mps: f64,
}

impl FreeBody {
    /// A body at rest at the obstacle's surface position.
    pub fn at(o: &Obstacle) -> FreeBody {
        FreeBody {
            x_m: o.x_m,
            v_mps: 0.0,
        }
    }

    /// Advances the body over `h_s` under the edge contact force `f_contact_n` (the value
    /// from [`obstacle_force_n`], which pushes the body toward −x) held constant: exact
    /// constant-force kinematics x += v·h − ½·(F/m)·h², v −= (F/m)·h.
    pub fn advance(&mut self, o: &Obstacle, f_contact_n: f64, h_s: f64) {
        let m = o.body.m_eff_kg();
        let a = if m > 0.0 && f_contact_n.is_finite() {
            -f_contact_n.max(0.0) / m
        } else {
            0.0
        };
        self.x_m += self.v_mps * h_s + 0.5 * a * h_s * h_s;
        self.v_mps += a * h_s;
    }
}

/// Symmetric trapezoid in [0, 1] at `t_since_s` after onset: rises linearly over
/// `ramp_s`, holds, and falls over `ramp_s` to end at `end_s` when given (a triangle when
/// `end_s` < 2·ramp); 0 before onset and after the end (PHYSICS §22).
pub fn trapezoid(t_since_s: f64, ramp_s: f64, end_s: Option<f64>) -> f64 {
    if !t_since_s.is_finite() || t_since_s < 0.0 {
        return 0.0;
    }
    let up = if ramp_s > 0.0 {
        (t_since_s / ramp_s).min(1.0)
    } else {
        1.0
    };
    let down = match end_s {
        Some(e) if t_since_s >= e => 0.0,
        Some(e) if ramp_s > 0.0 => ((e - t_since_s) / ramp_s).min(1.0),
        _ => 1.0,
    };
    up.min(down).max(0.0)
}

/// Lean force along x at `t_since_s` after onset: `f_n`·trapezoid(rise_s, dur_s)
/// (plan §2b item 11: rise ≥ 1 s), N.
pub fn lean_force_n(f_n: f64, t_since_s: f64, rise_s: f64, dur_s: f64) -> f64 {
    f_n * trapezoid(t_since_s, rise_s, Some(dur_s))
}

/// Smoke-control / cabinet pressure difference at `t_since_s` after onset:
/// `dp_pa`·trapezoid(ramp_s, dur_s) (plan §2b item 11: ramp ≥ 2 s; held to the end of
/// the run when `dur_s` is `None`), Pa.
pub fn smoke_dp_pa(dp_pa: f64, t_since_s: f64, ramp_s: f64, dur_s: Option<f64>) -> f64 {
    dp_pa * trapezoid(t_since_s, ramp_s, dur_s)
}

/// Hand driving force at `t_since_s` after contact from the piecewise-linear profile
/// (`t_pts_s` strictly increasing, relative to contact), N: the first value until the
/// first point, linear between points, and `None` (released) after the last point, before
/// contact, or for a malformed profile.
pub fn hand_drive_n(t_pts_s: &[f64], f_pts_n: &[f64], t_since_s: f64) -> Option<f64> {
    let n = t_pts_s.len();
    if n == 0 || n != f_pts_n.len() || !t_since_s.is_finite() || t_since_s < 0.0 {
        return None;
    }
    if t_since_s > t_pts_s[n - 1] {
        return None;
    }
    if t_since_s <= t_pts_s[0] {
        return Some(f_pts_n[0]);
    }
    let k = t_pts_s.partition_point(|&t| t <= t_since_s).clamp(1, n - 1);
    let (t0, t1) = (t_pts_s[k - 1], t_pts_s[k]);
    let (f0, f1) = (f_pts_n[k - 1], f_pts_n[k]);
    if t1 <= t0 {
        return Some(f1);
    }
    Some(f0 + (f1 - f0) * (t_since_s - t0) / (t1 - t0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integrator::rk4_step;
    use fabrail_params::derive::clamp_force;

    #[test]
    fn series_stiffness_and_body_table() {
        // Chest 25 N/mm with a 10 N/mm seal → 7.142857 N/mm (F6 series edge).
        let k = series_k(ObstacleBody::Chest.k_n_per_m(), 10.0e3);
        assert!((k - 25.0e3 * 10.0e3 / 35.0e3).abs() < 1e-9);
        assert_eq!(series_k(0.0, 1.0), 0.0);
        assert_eq!(series_k(f64::NAN, 1.0), 0.0);
        for b in ObstacleBody::ALL {
            assert!(b.k_n_per_m() > 0.0 && b.m_eff_kg() > 0.0, "{}", b.name());
            assert!(b.f_qs_n() > 0.0);
        }
        let bare = Obstacle {
            body: ObstacleBody::Hand,
            k_edge_n_per_m: 0.0,
            ..Obstacle::default()
        };
        assert_eq!(obstacle_k_n_per_m(&bare), 75.0e3, "no seal: the body alone");
    }

    #[test]
    fn clamped_hand_through_the_edge_seal_matches_f6() {
        // DOOR (100 kg) closing at the 1.69 J cap (0.1838 m/s) onto a clamped hand
        // through the 10 N/mm seal: peak force v·√(m·k_s) = 172.7 N (core design F6
        // "173 N"; derive::clamp_force), simulated with RK4 at 50 µs within 1 %.
        let o = Obstacle {
            body: ObstacleBody::Hand,
            x_m: 0.0,
            clamped: true,
            k_edge_n_per_m: 10.0e3,
        };
        let m = 100.0_f64;
        let v0 = (2.0 * 1.69 / m).sqrt();
        let k_s = obstacle_k_n_per_m(&o);
        let want = clamp_force(1.69, k_s);
        assert!((want - 172.7).abs() < 0.1, "closed form {want} (F6)");
        assert!((peak_force_n(&o, m, v0) - want).abs() < 1e-9 * want);
        let h = 5.0e-5;
        let mut y = [1.0e-3, -v0]; // edge 1 mm from the hand, closing
        let mut f_max = 0.0f64;
        for k in 0..20_000u32 {
            y = rk4_step(&y, f64::from(k) * h, h, |_, _, s| {
                [s[1], obstacle_force_n(&o, s[0], s[1]) / m]
            });
            f_max = f_max.max(obstacle_force_n(&o, y[0], y[1]));
        }
        assert!(
            (f_max - want).abs() < 0.01 * want,
            "peak {f_max} vs {want} N (1 %, F6)"
        );
        assert!(
            y[1] > 0.0,
            "the undamped springs return the door (no energy lost)"
        );
    }

    #[test]
    fn free_hand_transient_matches_reduced_mass_form() {
        // Same door on a free hand (0.6 kg): peak v·√(μ·k_s), μ = 0.6·100/100.6
        // (ISO/TS 15066 Annex A transient form), split-integrated at 10 µs within 1 %.
        let mut o = Obstacle {
            body: ObstacleBody::Hand,
            x_m: 0.0,
            clamped: false,
            k_edge_n_per_m: 10.0e3,
        };
        let m = 100.0_f64;
        let v0 = (2.0 * 1.69 / m).sqrt();
        let want = peak_force_n(&o, m, v0);
        assert!((want - 13.33).abs() < 0.02, "transient closed form {want}");
        let h = 1.0e-5;
        let mut door = [1.0e-3, -v0];
        let mut hand = FreeBody::at(&o);
        let mut f_max = 0.0f64;
        for k in 0..4000u32 {
            o.x_m = hand.x_m;
            let vr = door[1] - hand.v_mps;
            let f = obstacle_force_n(&o, door[0], vr);
            door = rk4_step(&door, f64::from(k) * h, h, |_, _, s| {
                [s[1], obstacle_force_n(&o, s[0], s[1] - hand.v_mps) / m]
            });
            hand.advance(&o, f, h);
            f_max = f_max.max(f);
        }
        assert!(
            (f_max - want).abs() < 0.01 * want,
            "peak {f_max} vs {want} N (1 %)"
        );
        assert!(
            hand.v_mps < -v0,
            "the light hand is knocked away faster than the door"
        );
    }

    #[test]
    fn hand_grip_and_push_semantics() {
        let g = HandInput::arm(10.0, true);
        // F = 10 − 10·0.1 − 500·0.004 − 1.5·0.5 = 6.25 N.
        assert!((hand_force_n(&g, 0.004, 0.1, 0.5) - 6.25).abs() < 1e-12);
        assert!(hand_force_n(&g, 0.05, 0.0, 0.0) < 0.0, "a grip can pull");
        // Push-only: pressing in (−x) the body cannot be pulled out by the hand.
        let p = HandInput::arm(-5.0, false);
        assert!(hand_force_n(&p, 0.0, 0.0, 0.0) < 0.0);
        assert_eq!(
            hand_force_n(&p, -0.05, 0.0, 0.0),
            0.0,
            "separates when the body runs ahead"
        );
        assert_eq!(
            hand_force_n(&HandInput::arm(0.0, false), 0.01, -0.2, 0.0),
            0.0
        );
        assert_eq!(hand_force_n(&g, f64::NAN, 0.0, 0.0), 0.0);
    }

    #[test]
    fn disturbance_and_hand_profiles() {
        // Lean 15 N rising over 1 s, 5 s total (plan §2b item 11).
        assert_eq!(lean_force_n(15.0, -0.1, 1.0, 5.0), 0.0);
        assert!((lean_force_n(15.0, 0.5, 1.0, 5.0) - 7.5).abs() < 1e-12);
        assert_eq!(lean_force_n(15.0, 2.0, 1.0, 5.0), 15.0);
        assert!((lean_force_n(15.0, 4.5, 1.0, 5.0) - 7.5).abs() < 1e-12);
        assert_eq!(lean_force_n(15.0, 5.0, 1.0, 5.0), 0.0);
        // Triangle when the duration is shorter than two ramps.
        assert!((trapezoid(0.5, 1.0, Some(1.0)) - 0.5).abs() < 1e-12);
        // Smoke 25 Pa over a 2 s ramp, held to the end of the run.
        assert!((smoke_dp_pa(25.0, 1.0, 2.0, None) - 12.5).abs() < 1e-12);
        assert_eq!(smoke_dp_pa(25.0, 100.0, 2.0, None), 25.0);
        // Hand profile: 0 → 10 N over 0.1 s, hold to 0.5 s, then released.
        let t = [0.0, 0.1, 0.5];
        let f = [0.0, 10.0, 10.0];
        assert_eq!(hand_drive_n(&t, &f, -0.01), None);
        assert!((hand_drive_n(&t, &f, 0.05).unwrap_or(f64::NAN) - 5.0).abs() < 1e-12);
        assert_eq!(hand_drive_n(&t, &f, 0.3), Some(10.0));
        assert_eq!(hand_drive_n(&t, &f, 0.5), Some(10.0));
        assert_eq!(hand_drive_n(&t, &f, 0.51), None);
        assert_eq!(hand_drive_n(&[0.2], &[4.0], 0.1), Some(4.0));
        assert_eq!(hand_drive_n(&t, &f[..2], 0.1), None);
    }
}
