//! Integer safety envelope: kinetic energy, momentum and stroke checks in integer units (FAB-EXO-903). Owner: X1 (API frozen by W0a).
//!
//! EXOTIC §X16, §X14, §S1. Every check uses the integer units of [`crate::units`]
//! (kg u32, mm/s i32, 2·KE in u64 µJ, momentum u64 kg·mm/s, force i64 N; no Q16.16, no
//! floats per cycle) and always uses the **rated** mass, never the estimate m̂ (core rule;
//! FAB-EXO-504): no function here takes m̂. Overflow fails safe (trips). The integer
//! envelope is the arithmetically diverse channel to the f32 controllers (exotic design
//! §4.2) and equals the f64 reference within 1 LSB (FAB-EXO-903).
//!
//! * [`check`] (the integer twin of the core energy/momentum governor, PHYSICS §16, and
//!   the core integer monitor, PHYSICS §18): 2·KE ≤ limit, optional momentum ≤ limit,
//!   speed ≤ limit, stroke inside [x_min, x_max]; [`force_within`]: |m_rated·a| ≤ F_max,
//!   exact in mN.
//! * [`bumper_rating_ke2`] / [`bumper_adequate`]: rating ≥ ½·m_rated·v_max² (as 2·KE);
//!   HAVEN-F 5 kt at 1.2 m/s needs 3.6 MJ.
//! * Attended jog (FAB-EXO-503): hold-to-run with a 3-position enabling device,
//!   v ≤ min(jog limit, ⌊√(2·E_ped/m_rated)⌋) ([`attended_jog_cap`], [`jog_envelope`])
//!   with the pedestrian energy cap imported from fabrail-params (never restated,
//!   FAB-EXO-114); momentum stays uncapped and is stated (SHIELD at 50 t:
//!   √(2·E_ped·m) = 411 kg·m/s). The drive force is capped at the imported sensed static
//!   limit ([`JOG_FORCE_CAP_N`]) and the closing edge is compliant (≤ 10 N/mm, imported);
//!   the peak contact force F_cap + √(F_cap² + 2·k·KE) ([`edge_peak_force_n`]) stays
//!   within the imported dynamic force limit.
//! * Occupied platform (HAVEN, EXOTIC §X9): controlled moves (recentring,
//!   pre-positioning, landing) stay within the occupant acceleration target, the jerk
//!   limit and the landing speed, and need the moat band clear ([`check_occupied_move`]).
//!   The seismic a_peak at DBE is an analysis output (FAB-EXO-505), not a trip here.

use fabrail_params::presets::{
    EDGE_K_MAX_N_PER_M, PEDESTRIAN_F_DYNAMIC_N, PEDESTRIAN_F_STATIC_SENSED_N,
};

use crate::types::{OccupantCfg, ZoneCfg};
use crate::units::{ForceN, Ke2Uj, MassKg, MomKgMmps, PEDESTRIAN_KE2, VelMmps, speed_cap_energy};

/// Attended-jog drive force cap, N: the sensed static clamp limit imported from
/// `fabrail_params::presets` (the jog runs with a sensing edge and an enabling device;
/// PHYSICS §16 directional clamp).
pub const JOG_FORCE_CAP_N: u32 = (PEDESTRIAN_F_STATIC_SENSED_N + 0.5) as u32;

/// Compliant closing-edge stiffness limit, N/mm (imported `EDGE_K_MAX_N_PER_M`).
pub const EDGE_K_LIMIT_N_PER_MM: u32 = (EDGE_K_MAX_N_PER_M / 1000.0 + 0.5) as u32;

/// Peak (dynamic) contact force limit, N (imported `PEDESTRIAN_F_DYNAMIC_N`).
pub const DYNAMIC_FORCE_LIMIT_N: u32 = (PEDESTRIAN_F_DYNAMIC_N + 0.5) as u32;

/// Envelope limits (integer units).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct EnvelopeLimits {
    /// Maximum 2·KE at rated mass, µJ.
    pub ke2_max: Ke2Uj,
    /// Maximum momentum, kg·mm/s (`None`: uncapped, stated in the class docs).
    pub mom_max: Option<MomKgMmps>,
    /// Maximum speed, mm/s.
    pub v_max_mmps: u32,
    /// Stroke lower bound, mm.
    pub x_min_mm: i64,
    /// Stroke upper bound, mm.
    pub x_max_mm: i64,
}

/// Which limit tripped.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum EnvelopeTrip {
    /// 2·KE above the limit (or overflowed).
    KineticEnergy = 0,
    /// Momentum above the limit.
    Momentum = 1,
    /// Speed above the limit.
    Speed = 2,
    /// Position outside the stroke.
    Stroke = 3,
}

/// Result of [`check`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct EnvelopeVerdict {
    /// First limit that tripped, in the order KE, momentum, speed, stroke.
    pub trip: Option<EnvelopeTrip>,
}

impl EnvelopeVerdict {
    /// True when nothing tripped.
    pub const fn ok(&self) -> bool {
        self.trip.is_none()
    }
}

/// Checks the state (rated mass `m_rated`, velocity `v`, position `x_mm`) against `l`.
/// Exactly at a limit is allowed; strictly above trips.
pub const fn check(l: &EnvelopeLimits, m_rated: MassKg, v: VelMmps, x_mm: i64) -> EnvelopeVerdict {
    let trip = match Ke2Uj::of(m_rated, v) {
        None => Some(EnvelopeTrip::KineticEnergy),
        Some(k) if k.0 > l.ke2_max.0 => Some(EnvelopeTrip::KineticEnergy),
        Some(_) => match l.mom_max {
            Some(p) if MomKgMmps::of(m_rated, v).0 > p.0 => Some(EnvelopeTrip::Momentum),
            _ => {
                if v.speed() > l.v_max_mmps {
                    Some(EnvelopeTrip::Speed)
                } else if x_mm < l.x_min_mm || x_mm > l.x_max_mm {
                    Some(EnvelopeTrip::Stroke)
                } else {
                    None
                }
            }
        },
    };
    EnvelopeVerdict { trip }
}

/// True when |F| = |m_rated·a| ≤ |f_max| (exact: compared in mN in i128; a in mm/s²).
pub const fn force_within(m_rated: MassKg, a_mmps2: i32, f_max: ForceN) -> bool {
    let f_mn = (m_rated.0 as i128 * a_mmps2 as i128).unsigned_abs();
    f_mn <= f_max.magnitude() as u128 * 1000
}

/// Required bumper rating as 2·KE: m_rated·v_max² (½·m·v² in joules after
/// `to_joules_f64`); `None` on overflow.
pub const fn bumper_rating_ke2(m_rated: MassKg, v_max: VelMmps) -> Option<Ke2Uj> {
    Ke2Uj::of(m_rated, v_max)
}

/// True when a bumper rated for `rating` (2·KE, µJ) absorbs ½·m_rated·v_max² (rated mass,
/// never m̂; an overflowing requirement is never adequate).
pub const fn bumper_adequate(rating: Ke2Uj, m_rated: MassKg, v_max: VelMmps) -> bool {
    match bumper_rating_ke2(m_rated, v_max) {
        Some(req) => rating.0 >= req.0,
        None => false,
    }
}

/// Attended-jog speed cap: min(`jog_v`, ⌊√(2·E_ped/m_rated)⌋) (EXOTIC §X14; SHIELD at
/// 50 t is energy-limited to 8 mm/s; the full 10 mm/s is available up to 33.8 t).
pub const fn attended_jog_cap(m_rated: MassKg, jog_v: VelMmps) -> VelMmps {
    let e = speed_cap_energy(m_rated, PEDESTRIAN_KE2);
    if e.0 < jog_v.0 { e } else { jog_v }
}

/// The attended-jog envelope of a guarded zone: 2·KE ≤ the imported pedestrian cap,
/// momentum uncapped (stated, not limited), speed ≤ [`attended_jog_cap`], stroke
/// [x_min, x_max].
pub const fn jog_envelope(
    m_rated: MassKg,
    cfg: &ZoneCfg,
    x_min_mm: i64,
    x_max_mm: i64,
) -> EnvelopeLimits {
    let jog = if cfg.jog_v_mmps > i32::MAX as u32 {
        i32::MAX
    } else {
        cfg.jog_v_mmps as i32
    };
    EnvelopeLimits {
        ke2_max: PEDESTRIAN_KE2,
        mom_max: None,
        v_max_mmps: attended_jog_cap(m_rated, VelMmps(jog)).speed(),
        x_min_mm,
        x_max_mm,
    }
}

/// Peak contact force through a compliant edge, N (rounded up): the edge of stiffness
/// k absorbs the kinetic energy plus the drive work while the drive pushes at F until
/// the edge trips it, ½·k·δ² = KE + F·δ, so F_peak = k·δ = F + √(F² + 2·k·KE). With
/// F = 0 this is the pinch force v·√(m·k) (PHYSICS §16). Units: k in N/mm and 2·KE in µJ
/// give 2·k·KE = k·ke2/1000 N².
pub const fn edge_peak_force_n(f_drive_n: u32, ke2: Ke2Uj, k_edge_n_per_mm: u32) -> u64 {
    let f = f_drive_n as u128;
    let s = f * f + (k_edge_n_per_mm as u128 * ke2.0 as u128).div_ceil(1000);
    let r = s.isqrt();
    let r = if r * r < s { r + 1 } else { r };
    let peak = f + r;
    if peak > u64::MAX as u128 {
        u64::MAX
    } else {
        peak as u64
    }
}

/// True when the attended jog of a class at rated mass keeps the peak edge force within
/// [`DYNAMIC_FORCE_LIMIT_N`] with the drive capped at [`JOG_FORCE_CAP_N`] and the edge
/// within [`EDGE_K_LIMIT_N_PER_MM`].
pub const fn jog_edge_ok(m_rated: MassKg, cfg: &ZoneCfg) -> bool {
    if cfg.edge_k_n_per_mm == 0 || cfg.edge_k_n_per_mm > EDGE_K_LIMIT_N_PER_MM {
        return false;
    }
    let v = jog_envelope(m_rated, cfg, 0, 0).v_max_mmps;
    let v = if v > i32::MAX as u32 {
        i32::MAX
    } else {
        v as i32
    };
    let ke2 = Ke2Uj::of_saturating(m_rated, VelMmps(v));
    edge_peak_force_n(JOG_FORCE_CAP_N, ke2, cfg.edge_k_n_per_mm) <= DYNAMIC_FORCE_LIMIT_N as u64
}

/// Which occupied-platform limit a controlled move breaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum PlatformTrip {
    /// The moat band is under presence monitoring and not clear.
    MoatOccupied = 0,
    /// Acceleration above the occupant target.
    Acceleration = 1,
    /// Jerk above the controlled-move limit.
    Jerk = 2,
    /// Landing speed above the limit.
    LandingSpeed = 3,
}

/// Checks a controlled move of an occupied platform (EXOTIC §X9, exotic design §2.6):
/// the moat band clear when monitored, |a| ≤ a_peak_max (0.1 g), |j| ≤ jerk_max
/// (1 m/s³) and, for a landing, the landing speed ≤ landing_v_max (0.1 m/s; an
/// uncontrolled 10–15 mm drop lands at 0.44–0.54 m/s). Seismic response is not a
/// controlled move and is reported, not tripped (FAB-EXO-505).
pub const fn check_occupied_move(
    cfg: &OccupantCfg,
    a_mmps2: i32,
    j_mmps3: i32,
    landing_v: Option<VelMmps>,
    moat_clear: bool,
) -> Option<PlatformTrip> {
    if cfg.moat_presence && !moat_clear {
        return Some(PlatformTrip::MoatOccupied);
    }
    if a_mmps2.unsigned_abs() > cfg.a_peak_max_mmps2 {
        return Some(PlatformTrip::Acceleration);
    }
    if j_mmps3.unsigned_abs() > cfg.jerk_max_mmps3 {
        return Some(PlatformTrip::Jerk);
    }
    match landing_v {
        Some(v) if v.speed() > cfg.landing_v_max_mmps => Some(PlatformTrip::LandingSpeed),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::presets::{self, PRESETS};
    use crate::types::{ExoClass, SafetyRegime};
    use crate::units::{MASS_RANGE_MAX_KG, MASS_RANGE_MIN_KG, V_RANGE_MMPS, isqrt_u64};
    use fabrail_params::presets::PEDESTRIAN_E_MAX_J;
    use proptest::prelude::*;

    fn vel(v_mmps: u32) -> VelMmps {
        VelMmps(v_mmps as i32)
    }

    // req: FAB-EXO-503
    #[test]
    fn attended_jog_limits() {
        assert_eq!(EDGE_K_LIMIT_N_PER_MM, 10, "imported edge limit, 10 N/mm");
        let mut n = 0;
        for p in PRESETS.iter() {
            let SafetyRegime::GuardedZone(z) = p.regime else {
                continue;
            };
            n += 1;
            let m = p.design_mass;
            let env = jog_envelope(m, &z, -1_000, 1_000);
            let v = env.v_max_mmps;
            // ≤ 10 mm/s and ≤ the imported energy cap at RATED mass.
            assert!(
                v <= 10,
                "{:?}: jog {v} mm/s > 10 mm/s (FAB-EXO-503)",
                p.class
            );
            let ke = Ke2Uj::of(m, vel(v)).unwrap();
            assert!(
                ke <= PEDESTRIAN_KE2,
                "{:?}: jog 2KE {} µJ above the imported cap {} µJ",
                p.class,
                ke.0,
                PEDESTRIAN_KE2.0
            );
            assert!(check(&env, m, vel(v), 0).ok());
            assert!(
                !check(&env, m, vel(v + 1), 0).ok(),
                "{:?}: v cap + 1 trips",
                p.class
            );
            assert!(!check(&env, m, vel(v), 1_001).ok(), "stroke still enforced");
            // Momentum is uncapped (stated, not limited).
            assert_eq!(env.mom_max, None);
            // Force cap and compliant edge: the drive stalls at the sensed static cap and
            // the peak edge force stays within the imported dynamic limit.
            assert!(
                z.edge_k_n_per_mm <= EDGE_K_LIMIT_N_PER_MM,
                "{:?}: edge",
                p.class
            );
            assert!(jog_edge_ok(m, &z), "{:?}", p.class);
            let peak = edge_peak_force_n(JOG_FORCE_CAP_N, ke, z.edge_k_n_per_mm);
            assert!(
                peak <= DYNAMIC_FORCE_LIMIT_N as u64 && peak >= JOG_FORCE_CAP_N as u64,
                "{:?}: peak edge force {peak} N (limit {DYNAMIC_FORCE_LIMIT_N} N)",
                p.class
            );
        }
        assert_eq!(n, 7, "seven guarded-zone classes");

        // SHIELD at 50 t is energy-limited: ⌊√(2·E/m)⌋ = 8 mm/s; momentum at the cap is
        // stated: 400 kg·m/s in integer units, √(2·E·m) = 411 kg·m/s at the exact
        // energy-limited speed (EXOTIC FAB-EXO-503, ±0.5 kg·m/s).
        let shield = presets::preset(ExoClass::Shield);
        let SafetyRegime::GuardedZone(z) = shield.regime else {
            panic!("SHIELD is a guarded zone");
        };
        let v = attended_jog_cap(shield.design_mass, vel(z.jog_v_mmps));
        assert_eq!(v, VelMmps(8));
        assert_eq!(MomKgMmps::of(shield.design_mass, v).to_kgmps_f64(), 400.0);
        let p_stated = (2.0 * PEDESTRIAN_E_MAX_J * shield.design_mass.to_kg_f64()).sqrt();
        assert!(
            (p_stated - 411.0).abs() < 0.5,
            "SHIELD p = {p_stated:.1} kg·m/s"
        );
        // The full 10 mm/s is available up to 33.8 t (exotic design §2.6), not above.
        assert_eq!(attended_jog_cap(MassKg(33_800), VelMmps(10)), VelMmps(10));
        assert_eq!(attended_jog_cap(MassKg(33_801), VelMmps(10)), VelMmps(9));
        // Static squeeze: the drive never exceeds the sensed cap; a compliant edge above
        // 10 N/mm fails the check.
        assert!(force_within(
            MassKg(50_000),
            3,
            ForceN(JOG_FORCE_CAP_N as i64)
        ));
        assert!(!force_within(
            MassKg(50_000),
            4,
            ForceN(JOG_FORCE_CAP_N as i64)
        ));
        let stiff = ZoneCfg {
            edge_k_n_per_mm: EDGE_K_LIMIT_N_PER_MM + 1,
            ..z
        };
        assert!(!jog_edge_ok(shield.design_mass, &stiff));
        // Pinch-force identity with no drive force: √(2·k·KE) = v·√(m·k)
        // (PHYSICS §16): 50 t at 8 mm/s into 10 N/mm → 0.008·√(5e4·1e4) = 178.9 N.
        assert_eq!(
            edge_peak_force_n(0, Ke2Uj::of(MassKg(50_000), VelMmps(8)).unwrap(), 10),
            179
        );
    }

    // req: FAB-EXO-504
    #[test]
    fn bumper_rating_rated_mass() {
        // Every class: rating = ½·m_rated·v_max², exact against f64.
        for p in PRESETS.iter() {
            let v = vel(p.kinematics.v_max_mmps);
            let req = bumper_rating_ke2(p.design_mass, v).unwrap();
            let e_ref = 0.5 * p.design_mass.to_kg_f64() * v.to_mps_f64() * v.to_mps_f64();
            assert!(
                (req.to_joules_f64() - e_ref).abs() <= 1e-12 * e_ref,
                "{:?}: {} J vs ½mv² {e_ref} J",
                p.class,
                req.to_joules_f64()
            );
            assert!(bumper_adequate(req, p.design_mass, v));
            assert!(!bumper_adequate(Ke2Uj(req.0 - 1), p.design_mass, v));
        }
        // Anchors: HAVEN-F 5 kt at 1.2 m/s = 3.6 MJ (FAB-EXO-504, exact); ARENA
        // 8,600 t at 0.3 m/s = 387 kJ; SENTINEL overstroke at 1.346 m/s = 0.906 MJ;
        // BASTION at its 100 t upper mass and 1.2 m/s = 72 kJ (exotic design §1.3).
        let j = |c: ExoClass, m: MassKg| {
            let p = presets::preset(c);
            bumper_rating_ke2(m, vel(p.kinematics.v_max_mmps))
                .unwrap()
                .to_joules_f64()
        };
        let hf = presets::preset(ExoClass::HavenF);
        assert_eq!(j(ExoClass::HavenF, hf.design_mass), 3.6e6);
        assert_eq!(j(ExoClass::Arena, presets::ARENA.design_mass), 387_000.0);
        assert!((j(ExoClass::Sentinel, presets::SENTINEL.design_mass) - 905_858.0).abs() < 1.0);
        assert_eq!(j(ExoClass::Bastion, presets::BASTION.mass_max), 72_000.0);

        // Rated mass, never m̂: a bumper sized from an under-reported m̂ (a sabotaged
        // estimator at 80 %) is inadequate for the rated mass; the API has no m̂ input.
        let m_rated = hf.design_mass;
        let m_hat = MassKg(m_rated.0 / 10 * 8);
        let v = vel(hf.kinematics.v_max_mmps);
        let sized_on_hat = bumper_rating_ke2(m_hat, v).unwrap();
        assert!(
            !bumper_adequate(sized_on_hat, m_rated, v),
            "m̂-sized bumper rejected"
        );
        // The envelope trips on the rated-mass energy even when m̂ would pass.
        let l = EnvelopeLimits {
            ke2_max: sized_on_hat,
            mom_max: None,
            v_max_mmps: u32::MAX,
            x_min_mm: i64::MIN,
            x_max_mm: i64::MAX,
        };
        assert!(check(&l, m_hat, v, 0).ok());
        assert_eq!(
            check(&l, m_rated, v, 0).trip,
            Some(EnvelopeTrip::KineticEnergy)
        );
        // Overflow fails safe.
        assert!(!bumper_adequate(
            Ke2Uj(u64::MAX),
            MassKg(u32::MAX),
            VelMmps(i32::MAX)
        ));
        assert_eq!(
            check(&l, MassKg(u32::MAX), VelMmps(i32::MIN), 0).trip,
            Some(EnvelopeTrip::KineticEnergy)
        );
    }

    #[test]
    fn occupied_move_limits() {
        let hr = presets::preset(ExoClass::HavenR);
        let SafetyRegime::OccupiedPlatform(o) = hr.regime else {
            panic!("HAVEN-R is an occupied platform");
        };
        assert_eq!(o.a_peak_max_mmps2, 981, "0.1 g in mm/s²");
        let a = o.a_peak_max_mmps2 as i32;
        let jm = o.jerk_max_mmps3 as i32;
        assert_eq!(
            check_occupied_move(&o, a, jm, Some(VelMmps(-100)), true),
            None
        );
        assert_eq!(
            check_occupied_move(&o, 0, 0, None, false),
            Some(PlatformTrip::MoatOccupied)
        );
        assert_eq!(
            check_occupied_move(&o, -(a + 1), 0, None, true),
            Some(PlatformTrip::Acceleration)
        );
        assert_eq!(
            check_occupied_move(&o, 0, jm + 1, None, true),
            Some(PlatformTrip::Jerk)
        );
        // An uncontrolled 10 mm drop lands at √(2·g·0.01) = 0.443 m/s: tripped.
        assert_eq!(
            check_occupied_move(&o, 0, 0, Some(VelMmps(443)), true),
            Some(PlatformTrip::LandingSpeed)
        );
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(4096))]

        // req: FAB-EXO-903
        #[test]
        fn matches_f64_reference(
            m in MASS_RANGE_MIN_KG..=MASS_RANGE_MAX_KG,
            v in -V_RANGE_MMPS..=V_RANGE_MMPS,
            x in -10_000_000i64..=10_000_000,
            fe in 0.0f64..2.0,
            fp in 0.0f64..2.0,
            dv in -3i64..=3,
            dx in -3i64..=3,
        ) {
            let mass = MassKg(m);
            let vel = VelMmps(v);
            let (m_f, v_mps) = (m as f64, v as f64 / 1000.0);
            // f64 reference quantities (EXOTIC §X16).
            let ke_ref = 0.5 * m_f * v_mps * v_mps;
            let p_ref = m_f * v_mps.abs();
            // 2·KE: 1 LSB = 1 µJ of 2·KE = 0.5 µJ of KE; the f64 reference itself carries
            // ≤ 4 ulp of rounding at 5e11 J.
            let ke = Ke2Uj::of(mass, vel).unwrap();
            let tol_e = 0.5e-6 + 4.0 * f64::EPSILON * ke_ref;
            prop_assert!((ke.to_joules_f64() - ke_ref).abs() <= tol_e,
                "KE {} vs f64 {} J (1 LSB)", ke.to_joules_f64(), ke_ref);
            let p = MomKgMmps::of(mass, vel);
            prop_assert!((p.to_kgmps_f64() - p_ref).abs() <= 1e-3 + 4.0 * f64::EPSILON * p_ref);

            // Limits given in f64 SI, converted once at configuration time.
            let e_lim = fe * ke_ref;
            let p_lim = fp * p_ref;
            let l = EnvelopeLimits {
                ke2_max: Ke2Uj::from_joules_f64(e_lim).unwrap(),
                mom_max: MomKgMmps::from_kgmps_f64(p_lim),
                v_max_mmps: (v.unsigned_abs() as i64 + dv).max(0) as u32,
                x_min_mm: x - 1_000 + dx,
                x_max_mm: x + dx,
            };
            // f64 verdict in the same order (KE, momentum, speed, stroke).
            let ref_trip = if ke_ref > e_lim {
                Some(EnvelopeTrip::KineticEnergy)
            } else if p_ref > p_lim {
                Some(EnvelopeTrip::Momentum)
            } else if v.unsigned_abs() > l.v_max_mmps {
                Some(EnvelopeTrip::Speed)
            } else if x < l.x_min_mm || x > l.x_max_mm {
                Some(EnvelopeTrip::Stroke)
            } else {
                None
            };
            // Agreement except within 1 LSB (+ f64 rounding) of the KE or momentum limit.
            let near_e = (2.0e6 * (ke_ref - e_lim)).abs() <= 1.0 + 8.0 * f64::EPSILON * 2.0e6 * ke_ref;
            let near_p = (1.0e3 * (p_ref - p_lim)).abs() <= 1.0 + 8.0 * f64::EPSILON * 1.0e3 * p_ref;
            let got = check(&l, mass, vel, x).trip;
            if !near_e && !near_p {
                prop_assert_eq!(got, ref_trip, "m {} v {} x {}", m, v, x);
            }

            // Speed caps against √(2E/m) and p/m (floor semantics, within 1 LSB).
            let v_e = speed_cap_energy(mass, PEDESTRIAN_KE2).0 as f64;
            let v_e_ref = (2.0 * PEDESTRIAN_E_MAX_J / m_f).sqrt() * 1000.0;
            prop_assert!(v_e <= v_e_ref + 1e-9 && v_e_ref - v_e < 1.0 + 1e-9);
            let jog = attended_jog_cap(mass, VelMmps(10)).0 as f64;
            prop_assert!(jog <= v_e_ref.min(10.0) + 1e-9 && v_e_ref.min(10.0) - jog < 1.0 + 1e-9);
            prop_assert_eq!(isqrt_u64(ke.0 / m as u64), v.unsigned_abs() as u64);
        }
    }
}
