//! Haptic effects: detents and the legacy soft-close spring (PHYSICS §17; HAP-007, HAP-008). Owner: A2.
//!
//! Frozen API (W0): [`detent_force_n`] (DRAWER 3 N / 2 mm, DOOR 8 / 3, WALL 20 / 5) and
//! [`soft_close_force_n`] (ω_sc = 2π×{2, 1.5, 0.5, 0.5} Hz, capture 45 / 65 / 115 /
//! 150 mm, final speed ≤ 20 / 50 / 20 / 10 mm/s). The log-damped terminus
//! (`motion::terminus`) is the primary soft-close; this spring is the looser HAP-007
//! bound kept for `MotionMode::SoftClose`.
//!
//! All effects act on the admittance's virtual mass through F_eff (PHYSICS §17) and are
//! position-only (conservative) or dissipative, so they never add energy on their own:
//!
//! * **Detent** at x_k with peak F_d and half-width w (`detent_width_m`):
//!   F = −F_d·sin(π·(x − x_k)/w) for |x − x_k| ≤ w, else 0. It pulls toward x_k with the
//!   peak F_d at |x − x_k| = w/2 and is continuous at the edges (a potential well of depth
//!   2·F_d·w/π).
//! * **Soft-close** toward `target` inside the capture zone c: a critically damped
//!   spring on the virtual mass, F = −M_v·(ω²·e + 2ω·ė)·s(|e|), e = x − target, with a
//!   smoothstep taper s = 1 for |e| ≤ c/2 falling to 0 at |e| = c. The taper removes the
//!   force step at the capture edge (M_v·ω²·c would otherwise be ≈ 230 N on DRAWER).
//! * **Virtual wall** at `wall` (extra helper for W1.5 end-of-travel feel): a one-sided
//!   spring −K·penetration, K ≤ K_max (HAP-009).
//!
//! Positions are `PosNm` and only differences are cast to f32 (PHYSICS §24).

use fabrail_types::{PosNm, TravelDir, fmath};

use crate::msg::HapticProfile;

/// π, rad.
const PI: f32 = core::f32::consts::PI;

/// Sum of the detent forces at `x` for profile `prof`, N.
pub fn detent_force_n(x: PosNm, prof: &HapticProfile) -> f32 {
    let w = prof.detent_width_m;
    let f_d = prof.detent_f_n;
    if !(w.is_finite() && w > 0.0 && f_d.is_finite()) {
        return 0.0;
    }
    let n = usize::from(prof.n_detents.min(4));
    prof.detents
        .iter()
        .take(n)
        .map(|&xk| {
            let d = x.diff_m(xk);
            if fmath::absf(d) <= w {
                -f_d * fmath::sinf(PI * d / w)
            } else {
                0.0
            }
        })
        .sum()
}

/// Soft-close spring-damper force toward `target` at (x, v) inside the capture zone, N.
///
/// Zero outside the capture zone and for non-positive ω, M_v or capture.
pub fn soft_close_force_n(
    x: PosNm,
    v_mps: f32,
    target: PosNm,
    omega_rad_s: f32,
    m_v_kg: f32,
    capture_m: f32,
) -> f32 {
    let valid = |q: f32| q.is_finite() && q > 0.0;
    if !valid(omega_rad_s) || !valid(m_v_kg) || !valid(capture_m) || !v_mps.is_finite() {
        return 0.0;
    }
    let e = x.diff_m(target);
    let ae = fmath::absf(e);
    if ae >= capture_m {
        return 0.0;
    }
    // Smoothstep taper over the outer half of the capture zone.
    let half = 0.5 * capture_m;
    let s = if ae <= half {
        1.0
    } else {
        let u = (capture_m - ae) / half;
        u * u * (3.0 - 2.0 * u)
    };
    -m_v_kg * (omega_rad_s * omega_rad_s * e + 2.0 * omega_rad_s * v_mps) * s
}

/// One-sided virtual wall at `wall` facing `dir` (the side motion toward `dir` is
/// blocked): F = −K·p along `dir` for penetration p = dir·(x − wall) > 0, else 0, N.
pub fn virtual_wall_force_n(x: PosNm, wall: PosNm, dir: TravelDir, k_n_per_m: f32) -> f32 {
    let p = dir.sign() * x.diff_m(wall);
    if p > 0.0 && k_n_per_m.is_finite() && k_n_per_m > 0.0 {
        -dir.sign() * k_n_per_m * p
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn detent_profile() -> HapticProfile {
        HapticProfile {
            n_detents: 1,
            detents: [PosNm(150_000_000), PosNm(0), PosNm(0), PosNm(0)],
            detent_f_n: 3.0,
            detent_width_m: 2.0e-3,
            ..HapticProfile::default()
        }
    }

    /// Peak detent force equals the profile (DRAWER 3 N at w/2 = 1 mm), restoring,
    /// zero at the centre and outside ±w. (E) f32 1e-5, HAP-008, PHYSICS §17.
    #[test]
    fn detent_peak_and_shape() {
        let p = detent_profile();
        let c = PosNm(150_000_000);
        let f_peak = detent_force_n(c.offset_m(1.0e-3), &p);
        assert!((f_peak + 3.0).abs() < 1e-5, "peak {f_peak}");
        assert!((detent_force_n(c.offset_m(-1.0e-3), &p) - 3.0).abs() < 1e-5);
        assert_eq!(detent_force_n(c, &p), 0.0);
        assert_eq!(detent_force_n(c.offset_m(2.5e-3), &p), 0.0);
        // Scan: never above the peak.
        for k in -300..=300 {
            let f = detent_force_n(c.offset_m(k as f32 * 1e-5), &p);
            assert!(f.abs() <= 3.0 + 1e-5);
        }
    }

    #[test]
    fn soft_close_is_continuous_and_restoring() {
        let t = PosNm(0);
        let (w, m, cap) = (2.0 * PI * 2.0, 32.0, 0.045);
        assert_eq!(
            soft_close_force_n(PosNm(46_000_000), 0.0, t, w, m, cap),
            0.0
        );
        // Just inside the edge the force is small (taper), deep inside it is the spring.
        let edge = soft_close_force_n(PosNm(44_900_000), 0.0, t, w, m, cap);
        assert!(edge.abs() < 0.1, "edge {edge}");
        let deep = soft_close_force_n(PosNm(10_000_000), 0.0, t, w, m, cap);
        assert!((deep + m * w * w * 0.01).abs() < 1e-3);
        // Damping opposes motion toward the target.
        assert!(soft_close_force_n(PosNm(5_000_000), -0.1, t, w, m, cap) > deep.min(0.0));
    }

    #[test]
    fn wall_is_one_sided() {
        let w = PosNm(1_000_000);
        assert_eq!(virtual_wall_force_n(PosNm(0), w, TravelDir::Open, 1e4), 0.0);
        let f = virtual_wall_force_n(PosNm(2_000_000), w, TravelDir::Open, 1e4);
        assert!((f + 10.0).abs() < 1e-4);
    }
}
