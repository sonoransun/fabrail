//! Rail geometry: sag, joints, roughness and lateral steps from a 1 mm profile with C1 Hermite interpolation; engagement e(x) (PHYSICS §10). Owner: A6b.
//!
//! The profile is synthesised by Python (`fabrail.rail.irregularity_profile`, L/1150
//! sag half-sines, 0.1–0.3 mm joint steps, 20 µm rms roughness) and arrives as the
//! scenario's space-axis series `rail` (channels z, y; channel-major LE f64 bytes).
//!
//! Interpolation (PHYSICS §10): cubic Hermite on the uniform grid with Catmull–Rom node
//! slopes (central differences; one-sided at the ends), so value and slope are
//! continuous (C¹) and quadratics are reproduced exactly inside the grid. Outside the
//! grid the edge value is held with zero slope.
//!
//! Engagement (PHYSICS §10; physics perspective §1.2): an element of pole length l
//! centred at rail coordinate x_c sees the fraction of its face that lies under rail
//! iron, with the rail end softened by the fringing field over ℓ_f = 2·g* (a linear
//! "iron presence" ramp across the end, integrated over the face, which is C¹ in x_c).
//! The DRAWER rail spans the cabinet depth, [x_closed − L/2, x_closed + L/2] for body
//! length L, so the extension moment and the rail-end pull-back emerge from e(x); the
//! fixed-rail classes (DOOR/WALL/HEAVY) run on a rail covering the whole travel plus
//! overtravel beyond the end zones ([`rail_extent`]). Joints are closed butt joints:
//! their vertical and lateral steps live in the profile, not in e(x); LSM stator segment
//! hand-over is the LSM model's K_f(x) (PHYSICS §7).

use fabrail_params::{CarriageDesign, DesignParams, RailDesign};
use fabrail_types::fmath;

use crate::model::PlantError;

/// Rail irregularity sampled on a uniform grid x0 + k·dx.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RailProfile {
    /// Grid origin, m.
    pub x0_m: f64,
    /// Grid spacing (1 mm nominal), m.
    pub dx_m: f64,
    /// Vertical deviation of the rail underside (up +), m.
    pub z_m: Vec<f64>,
    /// Lateral deviation (y +), m.
    pub y_m: Vec<f64>,
}

/// Rail deviation and slope at one x.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RailSample {
    /// Vertical deviation, m.
    pub z_m: f64,
    /// Vertical slope dz/dx.
    pub dz_dx: f64,
    /// Lateral deviation, m.
    pub y_m: f64,
    /// Lateral slope dy/dx.
    pub dy_dx: f64,
}

/// Catmull–Rom node slope (per grid step) of channel `v` at node `k`.
fn node_slope(v: &[f64], k: usize) -> f64 {
    let n = v.len();
    if n < 2 {
        return 0.0;
    }
    if k == 0 {
        v[1] - v[0]
    } else if k + 1 >= n {
        v[n - 1] - v[n - 2]
    } else {
        0.5 * (v[k + 1] - v[k - 1])
    }
}

/// Cubic Hermite value and d/dt on segment k at local t ∈ [0, 1] (per grid step).
fn hermite(v: &[f64], k: usize, t: f64) -> (f64, f64) {
    let (p0, p1) = (v[k], v[k + 1]);
    let (m0, m1) = (node_slope(v, k), node_slope(v, k + 1));
    let t2 = t * t;
    let t3 = t2 * t;
    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;
    let d00 = 6.0 * t2 - 6.0 * t;
    let d10 = 3.0 * t2 - 4.0 * t + 1.0;
    let d01 = -6.0 * t2 + 6.0 * t;
    let d11 = 3.0 * t2 - 2.0 * t;
    (
        h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1,
        d00 * p0 + d10 * m0 + d01 * p1 + d11 * m1,
    )
}

impl RailProfile {
    /// A validated profile: finite x0, finite dx > 0, equal-length z and y with at least
    /// 2 finite samples.
    pub fn new(
        x0_m: f64,
        dx_m: f64,
        z_m: Vec<f64>,
        y_m: Vec<f64>,
    ) -> Result<RailProfile, PlantError> {
        if !x0_m.is_finite() {
            return Err(PlantError::Invalid("rail.x0_m", "must be finite"));
        }
        if !(dx_m.is_finite() && dx_m > 0.0) {
            return Err(PlantError::Invalid("rail.dx_m", "must be finite and > 0"));
        }
        if z_m.len() < 2 || z_m.len() != y_m.len() {
            return Err(PlantError::Invalid(
                "rail",
                "z and y need equal lengths of at least 2 samples",
            ));
        }
        if !z_m.iter().chain(y_m.iter()).all(|v| v.is_finite()) {
            return Err(PlantError::Invalid("rail", "samples must be finite"));
        }
        Ok(RailProfile {
            x0_m,
            dx_m,
            z_m,
            y_m,
        })
    }

    /// Number of grid samples.
    pub fn len(&self) -> usize {
        self.z_m.len()
    }

    /// True when the profile has no samples.
    pub fn is_empty(&self) -> bool {
        self.z_m.is_empty()
    }

    /// Last grid coordinate, m.
    pub fn x_end_m(&self) -> f64 {
        self.x0_m + self.dx_m * (self.len().saturating_sub(1)) as f64
    }

    /// Deviation and slope at `x_m` by C¹ Hermite interpolation, edge-held outside the
    /// grid (PHYSICS §10). A profile with fewer than 2 samples (only constructible by
    /// struct literal) or a non-finite `x_m` samples as a flat rail.
    pub fn sample(&self, x_m: f64) -> RailSample {
        let n = self.z_m.len().min(self.y_m.len());
        if n < 2 || !x_m.is_finite() || self.dx_m.is_nan() || self.dx_m <= 0.0 {
            return RailSample::default();
        }
        let u = (x_m - self.x0_m) / self.dx_m;
        if u <= 0.0 {
            return RailSample {
                z_m: self.z_m[0],
                y_m: self.y_m[0],
                ..RailSample::default()
            };
        }
        let last = (n - 1) as f64;
        if u >= last {
            return RailSample {
                z_m: self.z_m[n - 1],
                y_m: self.y_m[n - 1],
                ..RailSample::default()
            };
        }
        // 0 ≤ floor(u) ≤ n − 2 here, so the cast is exact and in range.
        let k = (fmath::floor(u) as usize).min(n - 2);
        let t = u - k as f64;
        let (z, dz) = hermite(&self.z_m, k, t);
        let (y, dy) = hermite(&self.y_m, k, t);
        RailSample {
            z_m: z,
            dz_dx: dz / self.dx_m,
            y_m: y,
            dy_dx: dy / self.dx_m,
        }
    }
}

/// ∫_{-∞}^{s} of the unit iron-presence ramp that falls linearly from 1 to 0 across
/// [e − w/2, e + w/2] (a rail end at `e` softened over `w`), as a function of the
/// upper limit `s`, m. Piecewise quadratic, C¹.
fn presence_integral(s: f64, e: f64, w: f64) -> f64 {
    let a = e - 0.5 * w;
    let b = e + 0.5 * w;
    if s <= a {
        s - a
    } else if s >= b {
        0.5 * w
    } else {
        let d = s - a;
        d - 0.5 * d * d / w
    }
}

/// Fraction of the face [x_c − l/2, x_c + l/2] under a rail spanning [x_lo, x_hi] with
/// both ends softened over `w` (PHYSICS §10), in [0, 1].
pub fn overlap_fraction(x_c_m: f64, l_m: f64, x_lo_m: f64, x_hi_m: f64, w_m: f64) -> f64 {
    if !(x_c_m.is_finite() && l_m > 0.0 && x_hi_m > x_lo_m) {
        return 0.0;
    }
    let w = w_m.max(1e-9);
    let (s0, s1) = (x_c_m - 0.5 * l_m, x_c_m + 0.5 * l_m);
    // Exact shortcuts: fully under the rail clear of both ramps, or fully off it.
    if s0 >= x_lo_m + 0.5 * w && s1 <= x_hi_m - 0.5 * w {
        return 1.0;
    }
    if s1 <= x_lo_m - 0.5 * w || s0 >= x_hi_m + 0.5 * w {
        return 0.0;
    }
    // Iron presence = ramp down at the high end × ramp up at the low end; the two ramps
    // never overlap on a rail longer than w, so the integral splits.
    let hi = presence_integral(s1, x_hi_m, w) - presence_integral(s0, x_hi_m, w);
    // Low end: mirror (presence rises across x_lo): ∫ = length − ∫ of the falling ramp.
    let lo_missing =
        (presence_integral(-s0, -x_lo_m, w) - presence_integral(-s1, -x_lo_m, w)) - (s1 - s0);
    ((hi + lo_missing) / l_m).clamp(0.0, 1.0)
}

/// True when the rail is a cabinet-depth slide rail (the DRAWER type): the body does not
/// hang from the rail and travels no further than its own length, so it extends out of
/// a cabinet whose rail is only as deep as the body (PHYSICS §10; LEV-016). Every other
/// body (top-hung DOOR/WALL, HEAVY) runs on a fixed rail spanning its travel.
pub fn is_cabinet_rail(rail: &RailDesign, carriage: &CarriageDesign) -> bool {
    !carriage.top_hung && rail.travel_m <= carriage.length_m
}

/// Rail extent [x_lo, x_hi] in rail coordinates (PHYSICS §10): a cabinet rail spans
/// [x_closed − L/2, x_closed + L/2] (L = `carriage.length_m`, the cabinet depth); a fixed
/// rail spans [x_closed − L, x_closed + travel + L], i.e. the whole travel plus half a
/// body of overtravel beyond the end zones and bumpers at each end.
pub fn rail_extent(rail: &RailDesign, carriage: &CarriageDesign) -> [f64; 2] {
    let l = carriage.length_m.max(0.0);
    let x0 = rail.x_closed_m;
    if is_cabinet_rail(rail, carriage) {
        [x0 - 0.5 * l, x0 + 0.5 * l]
    } else {
        [x0 - l, x0 + rail.travel_m.max(0.0) + l]
    }
}

/// DRAWER engagement fraction e(x) ∈ [0, 1] of the element at carriage station
/// `station_x_m` when the carriage is at travel `x_m` (the extension moment emerges from
/// it; PHYSICS §10, LEV-016). The rail is [`rail_extent`], so the fixed-rail classes read
/// 1 over their whole travel. The pole length and gap are not in this signature, so the
/// face is taken as ℓ_f = 2·g* long and softened over the same ℓ_f, with g* the DRAWER
/// design gap 2.5 mm; [`engagement_for`] uses the design's pole length and g*.
pub fn engagement(rail: &RailDesign, carriage: &CarriageDesign, station_x_m: f64, x_m: f64) -> f64 {
    const L_F_DRAWER_M: f64 = 2.0 * 2.5e-3; // ℓ_f = 2·g*, DRAWER g* 2.5 mm (core design §C.2)
    let [lo, hi] = rail_extent(rail, carriage);
    overlap_fraction(station_x_m + x_m, L_F_DRAWER_M, lo, hi, L_F_DRAWER_M)
}

/// Rail extent [x_lo, x_hi] for design `d` ([`rail_extent`]), m.
pub fn rail_extent_m(d: &DesignParams) -> [f64; 2] {
    rail_extent(&d.rail, &d.carriage)
}

/// Engagement e ∈ [0, 1] of the element at carriage station `station_x_m` at travel
/// `x_m` for design `d`: the face of pole length l under the rail extent, the ends
/// softened over ℓ_f = 2·g* (PHYSICS §10). This is the tier entry point.
pub fn engagement_for(d: &DesignParams, station_x_m: f64, x_m: f64) -> f64 {
    let [lo, hi] = rail_extent_m(d);
    let s = &d.suspension;
    let l = if s.pole_l_m > 0.0 { s.pole_l_m } else { 1e-3 };
    let lf = 2.0 * s.g_star_m.max(0.0);
    overlap_fraction(station_x_m + x_m, l, lo, hi, lf)
}

/// ∂e/∂x by a central difference of [`engagement_for`] over ±1 µm, 1/m (the rail-end
/// x force F_x = −½(Φ/P)²·∂P/∂x uses it; PHYSICS §2).
pub fn engagement_slope_for(d: &DesignParams, station_x_m: f64, x_m: f64) -> f64 {
    let e = 1.0e-6;
    (engagement_for(d, station_x_m, x_m + e) - engagement_for(d, station_x_m, x_m - e)) / (2.0 * e)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    #[test]
    fn profile_validation() {
        let p = RailProfile::new(0.0, 1e-3, vec![0.0, 1e-5, 0.0], vec![0.0; 3]).expect("valid");
        assert_eq!(p.len(), 3);
        assert!((p.x_end_m() - 2e-3).abs() < 1e-15);
        assert!(RailProfile::new(0.0, 0.0, vec![0.0; 2], vec![0.0; 2]).is_err());
        assert!(RailProfile::new(0.0, 1e-3, vec![0.0; 2], vec![0.0; 3]).is_err());
        assert!(RailProfile::new(0.0, 1e-3, vec![0.0], vec![0.0]).is_err());
        assert!(RailProfile::new(f64::NAN, 1e-3, vec![0.0; 2], vec![0.0; 2]).is_err());
        assert!(RailProfile::new(0.0, 1e-3, vec![0.0, f64::NAN], vec![0.0; 2]).is_err());
    }

    #[test]
    fn hermite_reproduces_quadratics_and_is_c1() {
        // z = a + b·x + c·x² on a 1 mm grid: interior samples exact ((E) 1e-15 m,
        // PHYSICS §10), slope exact; y linear everywhere.
        let dx = 1.0e-3;
        let n = 50;
        let f = |x: f64| 2.0e-5 + 3.0e-3 * x - 0.4 * x * x;
        let xs: Vec<f64> = (0..n).map(|k| 0.1 + dx * k as f64).collect();
        let z: Vec<f64> = xs.iter().map(|&x| f(x)).collect();
        let y: Vec<f64> = xs.iter().map(|&x| 1.0e-4 * x).collect();
        let p = RailProfile::new(0.1, dx, z, y).expect("valid");
        for k in 0..400 {
            let x = 0.1 + dx * (1.0 + 46.0 * f64::from(k) / 400.0);
            let s = p.sample(x);
            assert!((s.z_m - f(x)).abs() < 1e-15, "z at {x}");
            assert!((s.dz_dx - (3.0e-3 - 0.8 * x)).abs() < 1e-11, "dz/dx at {x}");
            assert!((s.y_m - 1.0e-4 * x).abs() < 1e-17 && (s.dy_dx - 1.0e-4).abs() < 1e-12);
        }
        // C¹ across a node of an irregular profile: left and right limits agree.
        let q = RailProfile::new(0.0, dx, vec![0.0, 3e-6, -2e-6, 5e-6, 1e-6], vec![0.0; 5])
            .expect("valid");
        let e = 1e-12;
        let (l, r) = (q.sample(2.0 * dx - e), q.sample(2.0 * dx + e));
        assert!((l.z_m - r.z_m).abs() < 1e-13 && (l.dz_dx - r.dz_dx).abs() < 1e-6);
        // Edge-held outside.
        assert_eq!(q.sample(-1.0).z_m, 0.0);
        assert_eq!(
            q.sample(1.0),
            RailSample {
                z_m: 1e-6,
                ..RailSample::default()
            }
        );
        assert_eq!(q.sample(f64::NAN), RailSample::default());
    }

    #[test]
    fn overlap_is_c1_monotone_and_bounded() {
        // A 32 mm face sliding off a rail end softened over 5 mm.
        let mut last = 1.0;
        let mut last_slope = 0.0;
        for k in 0..=800 {
            let xc = 0.25 + 1.0e-4 * f64::from(k);
            let e = overlap_fraction(xc, 0.032, -0.3, 0.3, 0.005);
            assert!(
                (0.0..=1.0).contains(&e) && e <= last + 1e-15,
                "monotone at {xc}"
            );
            let slope = (overlap_fraction(xc + 1e-7, 0.032, -0.3, 0.3, 0.005)
                - overlap_fraction(xc - 1e-7, 0.032, -0.3, 0.3, 0.005))
                / 2e-7;
            assert!((slope - last_slope).abs() < 5.0, "slope jump at {xc}");
            last = e;
            last_slope = slope;
        }
        // Fully under: 1; half off the end (centre at the end): ½.
        assert_eq!(overlap_fraction(0.0, 0.032, -0.3, 0.3, 0.005), 1.0);
        assert!((overlap_fraction(0.3, 0.032, -0.3, 0.3, 0.005) - 0.5).abs() < 1e-12);
        assert!((overlap_fraction(-0.3, 0.032, -0.3, 0.3, 0.005) - 0.5).abs() < 1e-12);
        assert_eq!(overlap_fraction(0.5, 0.032, -0.3, 0.3, 0.005), 0.0);
    }

    #[test]
    fn drawer_engagement_over_the_preset_travel() {
        // At the preset 0.30 m (50 % extension, plan §2b item 3) every DRAWER element is
        // fully under the cabinet rail; the front stations start to leave it only past
        // the travel, in the end zone (LEV-016: no extension shortfall at 0.30 m).
        let d = presets::drawer().design;
        let n = usize::from(d.suspension.n_magnets);
        for st in &d.suspension.stations[..n] {
            for x in [0.0, 0.15, 0.30] {
                assert_eq!(
                    engagement_for(&d, st.x_m, x),
                    1.0,
                    "station {} at {x}",
                    st.x_m
                );
            }
        }
        let front = d.suspension.stations[0].x_m; // −0.02 m
        assert!(
            engagement_for(&d, front, 0.33) < 1.0,
            "front element leaves the rail"
        );
        assert!(
            engagement_slope_for(&d, front, 0.32) < 0.0,
            "pull-back gradient"
        );
        assert_eq!(engagement(&d.rail, &d.carriage, front, 0.1), 1.0);
        assert!(engagement(&d.rail, &d.carriage, front, 0.4) < 1e-12);
        // Fixed-rail classes: fully engaged over the whole travel and end zones, with the
        // frozen `engagement` too (it serves every class).
        for sp in [presets::door(), presets::wall(), presets::heavy()] {
            let dd = sp.design;
            assert!(!is_cabinet_rail(&dd.rail, &dd.carriage));
            let travel = dd.rail.travel_m;
            let nm = usize::from(dd.suspension.n_magnets);
            for st in &dd.suspension.stations[..nm] {
                for x in [-0.16, 0.0, 0.5 * travel, travel, travel + 0.16] {
                    assert_eq!(engagement_for(&dd, st.x_m, x), 1.0, "{:?} at {x}", dd.class);
                    assert_eq!(engagement(&dd.rail, &dd.carriage, st.x_m, x), 1.0);
                }
            }
        }
        assert!(is_cabinet_rail(&d.rail, &d.carriage));
    }
}
