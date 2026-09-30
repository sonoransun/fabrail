//! Stray field by dipole/cuboid superposition (PHYSICS §23; ENV-001). Owner: A6b.
//!
//! Target ≤ 0.5 mT at 10 cm from accessible surfaces (DRAWER/DOOR); WALL is at risk
//! (16 mT unshielded) and HEAVY needs shrouds (80 mT) (core design §C.2).
//!
//! Model (PHYSICS §23): every PM element is a uniformly magnetised block of the pole face
//! (l along x, w along y) and thickness h_m, magnetised along its normal (toward the
//! rail for a lift element), centred h_m/2 behind the pole face at its station. The
//! moment is m = B_r·V/μ₀ (V = w·l·h_m, B_r at 20 °C). [`field`] sums point dipoles,
//! B = (μ₀/4π)·(3(m·r̂)r̂ − m)/r³, the far-field form the ENV-001 target uses;
//! [`field_cuboid`] sums the exact field of the blocks (surface-charge method,
//! Furlani, Permanent Magnet and Electromechanical Devices, 2001, §2.7), for points near
//! the magnets. Both are bare magnets in free space: the rail and pole-piece iron, which
//! close most of the flux, are not modelled, so the result is an upper-bound style
//! estimate, and coil fields are omitted (zero-power control holds i ≈ 0).
//!
//! Reference values (single magnet, on axis, 10 cm from its centre, dipole): DRAWER
//! 0.22 mT, DOOR 1.6 mT, WALL 15.9 mT ("16 mT unshielded"), HEAVY 79.7 mT ("80 mT").

use fabrail_params::DesignParams;
use fabrail_params::derive::MU0_H_PER_M;
use fabrail_types::{MAX_MAGNETS, fmath};

/// μ₀/4π, T·m/A (from the CODATA 2018 μ₀ in `derive`; PHYSICS §23).
const MU0_OVER_4PI: f64 = MU0_H_PER_M / (4.0 * core::f64::consts::PI);

/// One permanent-magnet block of the suspension (carriage frame).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MagnetBlock {
    /// Block centre (x, y, z), m.
    pub centre_m: [f64; 3],
    /// Half extents (x, y, z) = (l/2, w/2, h_m/2), m.
    pub half_m: [f64; 3],
    /// Magnetisation direction along z: +1 toward the rail (lift), −1 (hold-down).
    pub dir: f64,
    /// Remanence, T.
    pub b_r_t: f64,
}

impl MagnetBlock {
    /// Volume w·l·h_m, m³.
    pub fn volume_m3(&self) -> f64 {
        8.0 * self.half_m[0] * self.half_m[1] * self.half_m[2]
    }

    /// Dipole moment magnitude B_r·V/μ₀, A·m² (PHYSICS §23).
    pub fn moment_a_m2(&self) -> f64 {
        self.b_r_t * self.volume_m3() / MU0_H_PER_M
    }
}

/// The PM blocks of design `p`: one per active station that carries a PM (pure-EMS
/// hold-down elements have none), in station order (PHYSICS §23).
pub fn magnets(p: &DesignParams) -> Vec<MagnetBlock> {
    let s = &p.suspension;
    let n = usize::from(s.n_magnets).min(MAX_MAGNETS);
    s.stations
        .iter()
        .take(n)
        .filter(|st| st.has_pm && st.normal_sign != 0.0)
        .map(|st| {
            let dir = if st.normal_sign > 0.0 { 1.0 } else { -1.0 };
            MagnetBlock {
                centre_m: [st.x_m, st.y_m, st.z_m - dir * 0.5 * s.h_m_m],
                half_m: [0.5 * s.pole_l_m, 0.5 * s.pole_w_m, 0.5 * s.h_m_m],
                dir,
                b_r_t: s.b_r_t,
            }
        })
        .collect()
}

/// Field of a point dipole of moment `m_a_m2` at offset `r_m` from it,
/// (μ₀/4π)·(3(m·r̂)r̂ − m)/|r|³, T; zero at the dipole itself (PHYSICS §23).
pub fn dipole_field(m_a_m2: [f64; 3], r_m: [f64; 3]) -> [f64; 3] {
    let r2 = r_m[0] * r_m[0] + r_m[1] * r_m[1] + r_m[2] * r_m[2];
    if !(r2 > 0.0 && r2.is_finite()) {
        return [0.0; 3];
    }
    let r = fmath::sqrt(r2);
    let r3 = r2 * r;
    let mr = (m_a_m2[0] * r_m[0] + m_a_m2[1] * r_m[1] + m_a_m2[2] * r_m[2]) / r2;
    [
        MU0_OVER_4PI * (3.0 * mr * r_m[0] - m_a_m2[0]) / r3,
        MU0_OVER_4PI * (3.0 * mr * r_m[1] - m_a_m2[1]) / r3,
        MU0_OVER_4PI * (3.0 * mr * r_m[2] - m_a_m2[2]) / r3,
    ]
}

/// ln(Y + R) with R = √(X² + Y² + Z²), stable for Y < 0 via ln((X² + Z²)/(R − Y)).
fn ln_y_plus_r(x: f64, y: f64, z: f64) -> f64 {
    let r = fmath::sqrt(x * x + y * y + z * z);
    if y >= 0.0 {
        fmath::ln(y + r)
    } else {
        let q = (x * x + z * z).max(f64::MIN_POSITIVE);
        fmath::ln(q / (r - y))
    }
}

/// Exact flux density of a uniformly magnetised block at `pt_m`, T (surface-charge
/// method: faces z = ±h/2 carry ±M; B = μ₀H outside the block; PHYSICS §23).
pub fn cuboid_field(b: &MagnetBlock, pt_m: [f64; 3]) -> [f64; 3] {
    let (a, bb, c) = (b.half_m[0], b.half_m[1], b.half_m[2]);
    let x = pt_m[0] - b.centre_m[0];
    let y = pt_m[1] - b.centre_m[1];
    let z = pt_m[2] - b.centre_m[2];
    let xs = [x - a, x + a];
    let ys = [y - bb, y + bb];
    let mut h = [0.0f64; 3];
    for (z0, sigma) in [(c, 1.0), (-c, -1.0)] {
        let zz = z - z0;
        for (i, &xx) in xs.iter().enumerate() {
            for (j, &yy) in ys.iter().enumerate() {
                // Corner sign of the double integral over [X1, X2] × [Y1, Y2].
                let s = if (i + j) % 2 == 0 { sigma } else { -sigma };
                let r = fmath::sqrt(xx * xx + yy * yy + zz * zz);
                // ∫∫X/R³ = −ln(Y + R); ∫∫Y/R³ = −ln(X + R); ∫∫Z/R³ = atan(XY/(ZR)).
                h[0] -= s * ln_y_plus_r(xx, yy, zz);
                h[1] -= s * ln_y_plus_r(yy, xx, zz);
                if zz != 0.0 && r > 0.0 {
                    h[2] += s * fmath::atan(xx * yy / (zz * r));
                }
            }
        }
    }
    // Corner order above is (X1,Y1)+, (X1,Y2)−, (X2,Y1)−, (X2,Y2)+, i.e. the definite
    // integral F(X2,Y2) − F(X2,Y1) − F(X1,Y2) + F(X1,Y1).
    let k = b.dir * b.b_r_t / (4.0 * core::f64::consts::PI);
    [k * h[0], k * h[1], k * h[2]]
}

/// Magnetic flux density B (x, y, z) in tesla at each point (x, y, z) in metres in the
/// carriage frame, one output per input point (PHYSICS §23): the superposition of the
/// PM blocks as point dipoles (ENV-001 "dipole sum").
pub fn field(p: &DesignParams, pts: &[[f64; 3]]) -> Vec<[f64; 3]> {
    let mags = magnets(p);
    pts.iter()
        .map(|pt| {
            let mut b = [0.0f64; 3];
            for m in &mags {
                let mv = [0.0, 0.0, m.dir * m.moment_a_m2()];
                let r = [
                    pt[0] - m.centre_m[0],
                    pt[1] - m.centre_m[1],
                    pt[2] - m.centre_m[2],
                ];
                let f = dipole_field(mv, r);
                b = [b[0] + f[0], b[1] + f[1], b[2] + f[2]];
            }
            b
        })
        .collect()
}

/// As [`field`] with the exact block field instead of point dipoles (near field;
/// PHYSICS §23). Points inside a block are not meaningful (B ≠ μ₀H there).
pub fn field_cuboid(p: &DesignParams, pts: &[[f64; 3]]) -> Vec<[f64; 3]> {
    let mags = magnets(p);
    pts.iter()
        .map(|pt| {
            let mut b = [0.0f64; 3];
            for m in &mags {
                let f = cuboid_field(m, *pt);
                b = [b[0] + f[0], b[1] + f[1], b[2] + f[2]];
            }
            b
        })
        .collect()
}

/// On-axis dipole field of the strongest single block at distance `r_m` from its centre,
/// (μ₀/4π)·2m/r³, T (the per-magnet "unshielded" figure of core design §C.2).
pub fn on_axis_peak_t(p: &DesignParams, r_m: f64) -> f64 {
    if !(r_m > 0.0 && r_m.is_finite()) {
        return 0.0;
    }
    let m = magnets(p)
        .iter()
        .map(MagnetBlock::moment_a_m2)
        .fold(0.0f64, f64::max);
    MU0_OVER_4PI * 2.0 * m / (r_m * r_m * r_m)
}

/// On-axis distance from the strongest block's centre at which its dipole field falls
/// to `b_t`: (μ₀·2m/(4π·B))^⅓, m (0 without magnets). For the ENV-001 budget: DOOR
/// reaches 0.5 mT at ≈ 14.8 cm from the magnet centre.
pub fn distance_for_field_m(p: &DesignParams, b_t: f64) -> f64 {
    if !(b_t > 0.0 && b_t.is_finite()) {
        return f64::INFINITY;
    }
    let m = magnets(p)
        .iter()
        .map(MagnetBlock::moment_a_m2)
        .fold(0.0f64, f64::max);
    fmath::cbrt(MU0_OVER_4PI * 2.0 * m / b_t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    /// The preset with only its first magnet active.
    fn single(mut d: DesignParams) -> DesignParams {
        d.suspension.n_magnets = 1;
        d
    }

    fn rel(a: f64, b: f64) -> f64 {
        (a - b).abs() / b.abs().max(f64::MIN_POSITIVE)
    }

    #[test]
    fn one_output_per_point() {
        let b = field(
            &DesignParams::default(),
            &[[0.1, 0.0, 0.0], [0.0, 0.1, 0.0]],
        );
        assert_eq!(b.len(), 2);
        assert_eq!(b[0], [0.0; 3], "no magnets, no field");
    }

    // req: FAB-ENV-001
    #[test]
    fn dipole_on_axis() {
        // Point dipole: on axis B = (μ₀/4π)·2m/r³, equatorial −(μ₀/4π)·m/r³, off-axis
        // components zero ((E) rel 1e-12, PHYSICS §23).
        let m = 80.0;
        let r = 0.1;
        let on = dipole_field([0.0, 0.0, m], [0.0, 0.0, r]);
        let want = MU0_OVER_4PI * 2.0 * m / (r * r * r);
        assert!(
            rel(on[2], want) < 1e-12,
            "on-axis {} vs {want} ((E) rel 1e-12, PHYSICS §23)",
            on[2]
        );
        assert_eq!((on[0], on[1]), (0.0, 0.0));
        let eq = dipole_field([0.0, 0.0, m], [r, 0.0, 0.0]);
        assert!(rel(eq[2], -0.5 * want) < 1e-12, "equatorial {}", eq[2]);
        assert_eq!(dipole_field([0.0, 0.0, m], [0.0; 3]), [0.0; 3]);
        // Preset magnets, single block, 10 cm on axis from its centre: WALL 15.9 mT and
        // HEAVY 79.7 mT (core design §C.2 "16 mT unshielded", "80 mT"; 1 %), DRAWER
        // 0.22 mT (≤ 0.5 mT target, ENV-001).
        for (sp, want_t) in [
            (presets::wall(), 16.0e-3),
            (presets::heavy(), 80.0e-3),
            (presets::drawer(), 0.2176e-3),
        ] {
            let d = single(sp.design);
            let mg = magnets(&d)[0];
            let pt = [mg.centre_m[0], mg.centre_m[1], mg.centre_m[2] - 0.1];
            let b = field(&d, &[pt])[0];
            assert!(
                rel(b[2], want_t) < 0.01,
                "{:?}: {} T vs {want_t} T (1 %, core design §C.2)",
                d.class,
                b[2]
            );
            assert!((b[2] - on_axis_peak_t(&d, 0.1)).abs() < 1e-15);
        }
        assert!(
            on_axis_peak_t(&presets::drawer().design, 0.1) <= 0.5e-3,
            "DRAWER meets 0.5 mT at 10 cm"
        );
        let r_door = distance_for_field_m(&presets::door().design, 0.5e-3);
        assert!(
            (r_door - 0.148).abs() < 0.002,
            "DOOR 0.5 mT at {r_door} m from the magnet"
        );
        // Superposition: the four-magnet DOOR field is the sum of its single blocks.
        let d = presets::door().design;
        let pt = [0.1, 0.05, -0.12];
        let total = field(&d, &[pt])[0];
        let mut sum = [0.0; 3];
        for mg in magnets(&d) {
            let f = dipole_field(
                [0.0, 0.0, mg.dir * mg.moment_a_m2()],
                [
                    pt[0] - mg.centre_m[0],
                    pt[1] - mg.centre_m[1],
                    pt[2] - mg.centre_m[2],
                ],
            );
            sum = [sum[0] + f[0], sum[1] + f[1], sum[2] + f[2]];
        }
        for k in 0..3 {
            assert!((total[k] - sum[k]).abs() < 1e-18);
        }
    }

    #[test]
    fn cuboid_matches_on_axis_closed_form_and_far_dipole() {
        let d = single(presets::wall().design);
        let mg = magnets(&d)[0];
        let (a, b, c) = (mg.half_m[0], mg.half_m[1], mg.half_m[2]);
        // On the axis above the top face: B_z = (B_r/π)·[atan(ab/(z₁√(a²+b²+z₁²)))
        // − atan(ab/(z₂√(a²+b²+z₂²)))], z₁ to the near face, z₂ = z₁ + 2c ((E) rel 1e-12).
        for dz in [0.002, 0.02, 0.1] {
            let z1 = dz;
            let z2 = dz + 2.0 * c;
            let f = |z: f64| fmath::atan(a * b / (z * (a * a + b * b + z * z).sqrt()));
            let want = mg.b_r_t / core::f64::consts::PI * (f(z1) - f(z2));
            let pt = [mg.centre_m[0], mg.centre_m[1], mg.centre_m[2] + c + dz];
            let got = cuboid_field(&mg, pt);
            assert!(rel(got[2], want) < 1e-12, "dz {dz}: {} vs {want}", got[2]);
            assert!(got[0].abs() < 1e-15 && got[1].abs() < 1e-15);
        }
        // Far field (4 m, arbitrary direction): the block equals its dipole within 1e-3
        // (octupole ≈ (size/r)²), every component — this fixes the ln-term signs.
        let pt = [
            mg.centre_m[0] + 2.4,
            mg.centre_m[1] - 1.8,
            mg.centre_m[2] + 2.6,
        ];
        let exact = cuboid_field(&mg, pt);
        let r = [
            pt[0] - mg.centre_m[0],
            pt[1] - mg.centre_m[1],
            pt[2] - mg.centre_m[2],
        ];
        let dip = dipole_field([0.0, 0.0, mg.moment_a_m2()], r);
        let norm = (dip[0] * dip[0] + dip[1] * dip[1] + dip[2] * dip[2]).sqrt();
        for k in 0..3 {
            assert!(
                (exact[k] - dip[k]).abs() < 1e-3 * norm,
                "component {k}: {exact:?} vs {dip:?}"
            );
        }
        assert_eq!(field_cuboid(&d, &[pt])[0], exact);
        // A hold-down (reversed) block flips the field.
        let mut rev = mg;
        rev.dir = -1.0;
        assert_eq!(cuboid_field(&rev, pt)[2], -exact[2]);
    }
}
