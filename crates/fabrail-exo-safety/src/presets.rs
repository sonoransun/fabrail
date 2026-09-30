//! Exotic class presets (SENTINEL, HAVEN-R, HAVEN-F, SHIELD, BASTION, ARENA, ROTATOR, TIDEGATE, SELENE) importing the pedestrian caps from fabrail_params::presets. Owner: W0b (written in Wave 0 by W0-C1).
//!
//! Plain data reproducing the EXOTIC §1 class table (plan §4; FAB-EXO-101, 102). Each
//! preset carries its table row verbatim ([`ClassRow`]) so the documentation table is
//! drift-guarded against code, plus the typed data: design mass and range, suspension,
//! regime, safe-state policy (mapped onto the core `SeismicAction`), seismic role, core
//! rest policy, kinematic budget, badge and TRL.
//!
//! Sources: plan §4 class table (= EXOTIC §1); exotic design §1.2 (mass ranges,
//! travel/stroke, v/a/jerk) and §1.3 (ISO 13855 K, t1, C and t2 per class); EXOTIC §2
//! anchors (SENTINEL kinematics at T 7 s, ±1.5 m; HAVEN-F bumper at 1.2 m/s). The
//! pedestrian caps and the compliant-edge limit are imported from
//! `fabrail_params::presets`, never restated (FAB-EXO-114). SELENE is data only: its
//! physics lives in the numpy frontier module (FAB-EXO-002).

use fabrail_params::derive::G0_MPS2;
use fabrail_params::presets::EDGE_K_MAX_N_PER_M;
use fabrail_types::RestPolicy;

use crate::types::{
    Badge, ExoClass, OccupantCfg, PoleMaterial, SafeStatePolicy, SafetyRegime, SeismicRole,
    SuspensionKind, Trl, ZoneCfg,
};
use crate::units::MassKg;

/// The EXOTIC §1 table row of a class, verbatim (drift-guarded).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ClassRow {
    /// "Class" cell.
    pub class: &'static str,
    /// "Design mass" cell.
    pub design_mass: &'static str,
    /// "Suspension and key parameters" cell.
    pub suspension: &'static str,
    /// "Regime / safe state" cell.
    pub regime: &'static str,
    /// "Maturity" cell.
    pub maturity: &'static str,
    /// "This effort" cell.
    pub effort: &'static str,
}

/// Kinematic budget of a class (exotic design §1.2; integer units).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Kinematics {
    /// Travel (doors, trays, gates, rings: arc length) or ± stroke (dampers,
    /// platforms), mm.
    pub travel_mm: u32,
    /// Speed limit, mm/s.
    pub v_max_mmps: u32,
    /// Acceleration limit, mm/s² (0: not specified).
    pub a_max_mmps2: u32,
    /// Jerk limit, mm/s³ (0: not specified).
    pub j_max_mmps3: u32,
    /// Tuned / isolation period, ms (0: not a resonant class).
    pub period_ms: u32,
}

/// One sovereign class preset.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ExoPreset {
    /// Class.
    pub class: ExoClass,
    /// Design mass.
    pub design_mass: MassKg,
    /// Lower end of the product mass range.
    pub mass_min: MassKg,
    /// Upper end of the product mass range.
    pub mass_max: MassKg,
    /// Suspension kind.
    pub suspension: SuspensionKind,
    /// Pole material (FeCo is a hybrid-EMS parameter).
    pub pole: PoleMaterial,
    /// Safety regime (never `Pedestrian`).
    pub regime: SafetyRegime,
    /// Safe-state policy.
    pub safe_state: SafeStatePolicy,
    /// Seismic role.
    pub seismic_role: SeismicRole,
    /// Core rest policy when idle.
    pub rest_policy: RestPolicy,
    /// Kinematic budget.
    pub kinematics: Kinematics,
    /// Capability badge (🔭 when maturity is frontier).
    pub badge: Badge,
    /// Technology readiness (separate from the badge).
    pub trl: Trl,
    /// EXOTIC §1 row, verbatim.
    pub row: ClassRow,
}

/// Compliant closing-edge stiffness limit in N/mm, from the imported core constant.
const EDGE_K_N_PER_MM: u32 = (EDGE_K_MAX_N_PER_M / 1000.0 + 0.5) as u32;

/// 0.1 g in mm/s² (HAVEN occupant target), from the standard gravity in fabrail-params.
const A_0P1G_MMPS2: u32 = (0.1 * G0_MPS2 * 1000.0 + 0.5) as u32;

/// Kilograms per tonne.
const T: u32 = 1000;

/// A guarded zone with ISO 13855 K = 1.6 m/s, t1 = 0.1 s, C = 0.85 m (exotic design
/// §1.3) and attended jog ≤ 10 mm/s (exotic design §2.6).
const fn gz(
    presence_pl_e: bool,
    pps: bool,
    two_person: bool,
    teleop: bool,
    a_brake_mmps2: u32,
) -> SafetyRegime {
    SafetyRegime::GuardedZone(ZoneCfg {
        presence_pl_e,
        pps,
        two_person_override: two_person,
        teleoperated: teleop,
        k_mmps: 1600,
        t1_ms: 100,
        c_mm: 850,
        a_brake_mmps2,
        jog_v_mmps: 10,
        edge_k_n_per_mm: EDGE_K_N_PER_MM,
    })
}

/// HAVEN occupied platform: a_peak ≤ 0.1 g at DBE (analysis output), jerk ≤ 1 m/s³,
/// landing ≤ 0.1 m/s, moat covers plus exclusion under presence monitoring (exotic
/// design §2.6; FAB-EXO-505).
const OCCUPIED: SafetyRegime = SafetyRegime::OccupiedPlatform(OccupantCfg {
    a_peak_max_mmps2: A_0P1G_MMPS2,
    jerk_max_mmps3: 1000,
    landing_v_max_mmps: 100,
    moat_presence: true,
    moat_covers: true,
});

/// SENTINEL tower HMD.
pub const SENTINEL: ExoPreset = ExoPreset {
    class: ExoClass::Sentinel,
    design_mass: MassKg(1000 * T),
    mass_min: MassKg(200 * T),
    mass_max: MassKg(2000 * T),
    suspension: SuspensionKind::HybridEms,
    pole: PoleMaterial::FeCo,
    // Locked plant room, unoccupied in operation; PL d access plus LOTO; braking at the
    // class a_max.
    regime: gz(false, false, false, false, 1209),
    safe_state: SafeStatePolicy::RideThrough,
    seismic_role: SeismicRole::DamperRideThrough,
    rest_policy: RestPolicy::Hover,
    // T 7 s, ±1.5 m: ωA = 1.346 m/s, ω²A = 1.209 m/s², ω³A = 1.085 m/s³ (EXOTIC §2).
    kinematics: Kinematics {
        travel_mm: 1500,
        v_max_mmps: 1346,
        a_max_mmps2: 1209,
        j_max_mmps3: 1085,
        period_ms: 7000,
    },
    badge: Badge::Research,
    trl: Trl::new(3, 4),
    row: ClassRow {
        class: "SENTINEL tower HMD",
        design_mass: "1,000 t, ±1.5 m, T 7 s",
        suspension: "FeCo-pole EMS planar bowl as cross-stacked 1-D stages; F_max 24–121 kN (33–163 kW); ironless LSM trim; passive eddy damping",
        regime: "GZ / RideThrough",
        maturity: "TRL 3–4",
        effort: "Dynamic simulation: 2-DOF plus modal tower",
    },
};

/// HAVEN-R isolation platform.
pub const HAVEN_R: ExoPreset = ExoPreset {
    class: ExoClass::HavenR,
    design_mass: MassKg(50 * T),
    mass_min: MassKg(T),
    mass_max: MassKg(200 * T),
    suspension: SuspensionKind::HybridEms,
    pole: PoleMaterial::Fe,
    regime: OCCUPIED,
    safe_state: SafeStatePolicy::IsolateThenLandFps,
    seismic_role: SeismicRole::PrimaryIsolation,
    rest_policy: RestPolicy::Hover,
    // T 6 s (R 8.95 m), stroke = S_D at MCE ±0.9 m, seismic-driven ≤ 0.8 m/s, jerk
    // ≤ 1 m/s³ in controlled moves.
    kinematics: Kinematics {
        travel_mm: 900,
        v_max_mmps: 800,
        a_max_mmps2: A_0P1G_MMPS2,
        j_max_mmps3: 1000,
        period_ms: 6000,
    },
    badge: Badge::Research,
    trl: Trl::new(2, 3),
    row: ClassRow {
        class: "HAVEN-R isolation",
        design_mass: "50 t, T 6 s (R 8.95 m), stroke = S_D at MCE (±0.9 m)",
        suspension: "Hybrid EMS (HTS option in 1-D stages); passive vertical support; copper damper with σ(T)",
        regime: "OP (a_peak ≤ 0.1 g at DBE as an analysis output; moat covers plus exclusion) / IsolateThenLandFps",
        maturity: "TRL 2–3",
        effort: "Dynamic simulation: 3-DOF planar",
    },
};

/// HAVEN-F facility platform (extrapolated).
pub const HAVEN_F: ExoPreset = ExoPreset {
    class: ExoClass::HavenF,
    design_mass: MassKg(5000 * T),
    mass_min: MassKg(1000 * T),
    mass_max: MassKg(10_000 * T),
    suspension: SuspensionKind::HybridEms,
    pole: PoleMaterial::FeCo,
    regime: OCCUPIED,
    // "As HAVEN-R; always levitated".
    safe_state: SafeStatePolicy::IsolateThenLandFps,
    seismic_role: SeismicRole::PrimaryIsolation,
    rest_policy: RestPolicy::Hover,
    // ±0.75 m, ≤ 1.2 m/s (bumper 3.6 MJ at 1.2 m/s, FAB-EXO-504); period as HAVEN-R.
    kinematics: Kinematics {
        travel_mm: 750,
        v_max_mmps: 1200,
        a_max_mmps2: A_0P1G_MMPS2,
        j_max_mmps3: 1000,
        period_ms: 6000,
    },
    badge: Badge::Research,
    trl: Trl::new(2, 3),
    row: ClassRow {
        class: "HAVEN-F facility",
        design_mass: "5 kt",
        suspension: "As HAVEN-R; always levitated",
        regime: "OP",
        maturity: "R (extrapolated)",
        effort: "Data plus algebraic admission; villa building config 🧪 (reduced platform)",
    },
};

/// SHIELD radiation plug / vault door (passive mover).
pub const SHIELD: ExoPreset = ExoPreset {
    class: ExoClass::Shield,
    design_mass: MassKg(50 * T),
    mass_min: MassKg(5 * T),
    mass_max: MassKg(500 * T),
    suspension: SuspensionKind::HybridEms,
    pole: PoleMaterial::Fe,
    // GZ plus the facility PPS; t2 = 1 s at 0.5 m/s (a_brake 0.5 m/s²) → S = 2.61 m.
    regime: gz(true, true, false, false, 500),
    safe_state: SafeStatePolicy::LandAndLock,
    seismic_role: SeismicRole::LandSecure,
    rest_policy: RestPolicy::LandLock,
    // 2–8 m travel (8 m preset), 0.5 m/s, 0.1 m/s², 0.5 m/s³.
    kinematics: Kinematics {
        travel_mm: 8000,
        v_max_mmps: 500,
        a_max_mmps2: 100,
        j_max_mmps3: 500,
        period_ms: 0,
    },
    badge: Badge::Research,
    trl: Trl::new(3, 3),
    row: ClassRow {
        class: "SHIELD",
        design_mass: "50 t (5–500 t)",
        suspension: "Hybrid EMS, passive mover",
        regime: "GZ + PPS / LandAndLock ≤ 2 s",
        maturity: "TRL 3",
        effort: "Data, admission, closure/land timeline",
    },
};

/// BASTION blast / vault plug door (closure race).
pub const BASTION: ExoPreset = ExoPreset {
    class: ExoClass::Bastion,
    design_mass: MassKg(50 * T),
    mass_min: MassKg(25 * T),
    mass_max: MassKg(100 * T),
    suspension: SuspensionKind::HybridEms,
    pole: PoleMaterial::Fe,
    // GZ PL e with a two-person override; t2 = 2.5 s at 1.2 m/s → S = 5.01 m.
    regime: gz(true, false, true, false, 480),
    safe_state: SafeStatePolicy::CompleteClose,
    seismic_role: SeismicRole::ClosureRace,
    rest_policy: RestPolicy::LandLock,
    // 3 m in 5.25 s at 1.2 m/s, 0.48 m/s², 2 m/s³.
    kinematics: Kinematics {
        travel_mm: 3000,
        v_max_mmps: 1200,
        a_max_mmps2: 480,
        j_max_mmps3: 2000,
        period_ms: 0,
    },
    badge: Badge::Research,
    trl: Trl::new(2, 3),
    row: ClassRow {
        class: "BASTION",
        design_mass: "50 t; 3 m in 5.25 s (a 0.48, j 2) or 5.0 s at a 0.53",
        suspension: "Hybrid EMS, land-to-seal",
        regime: "GZ PL e / CompleteClose",
        maturity: "TRL 2–3",
        effort: "Closure-race planner (t_a is an input)",
    },
};

/// ARENA field trays, roof petals, stage wagons (pods).
pub const ARENA: ExoPreset = ExoPreset {
    class: ExoClass::Arena,
    design_mass: MassKg(8600 * T),
    mass_min: MassKg(500 * T),
    mass_max: MassKg(10_000 * T),
    suspension: SuspensionKind::HybridEms,
    pole: PoleMaterial::FeCo,
    // Entry interlocks PL e; t2 = 6 s at 0.3 m/s → S = 10.61 m.
    regime: gz(true, false, false, false, 50),
    safe_state: SafeStatePolicy::LandAndLock,
    seismic_role: SeismicRole::LandSecure,
    rest_policy: RestPolicy::LandLock,
    // 15–200 m travel (200 m preset), 0.3 m/s, 0.02 m/s², 0.01 m/s³.
    kinematics: Kinematics {
        travel_mm: 200_000,
        v_max_mmps: 300,
        a_max_mmps2: 20,
        j_max_mmps3: 10,
        period_ms: 0,
    },
    badge: Badge::Research,
    trl: Trl::new(3, 3),
    row: ClassRow {
        class: "ARENA",
        design_mass: "8,600 t in pods",
        suspension: "Zero-power hybrid, FeCo",
        regime: "GZ / LandAndLock",
        maturity: "TRL 3",
        effort: "Data plus static WLS pod reallocation (dynamic simulation 🗺️)",
    },
};

/// ROTATOR telescope dome / rotating floor ring.
pub const ROTATOR: ExoPreset = ExoPreset {
    class: ExoClass::Rotator,
    design_mass: MassKg(6100 * T),
    mass_min: MassKg(1000 * T),
    mass_max: MassKg(6100 * T),
    suspension: SuspensionKind::HybridEms,
    pole: PoleMaterial::Fe,
    // Ring PL d; braking at the class a_max.
    regime: gz(false, false, false, false, 150),
    safe_state: SafeStatePolicy::BrakeAndLand,
    seismic_role: SeismicRole::LandSecure,
    rest_policy: RestPolicy::Land,
    // Ring r ≈ 43 m (circumference 270.2 m), rim 1.5 m/s, 0.15 m/s², 0.1 m/s³.
    kinematics: Kinematics {
        travel_mm: 270_177,
        v_max_mmps: 1500,
        a_max_mmps2: 150,
        j_max_mmps3: 100,
        period_ms: 0,
    },
    badge: Badge::Research,
    trl: Trl::new(3, 3),
    row: ClassRow {
        class: "ROTATOR",
        design_mass: "6,100 t ring, 36 stator stations",
        suspension: "Hybrid ring plus hooks; the absorption check currently fails and is reported",
        regime: "GZ / BrakeAndLand",
        maturity: "TRL 3",
        effort: "Data plus admission",
    },
};

/// TIDEGATE flood / tsunami gate.
pub const TIDEGATE: ExoPreset = ExoPreset {
    class: ExoClass::Tidegate,
    design_mass: MassKg(150 * T),
    mass_min: MassKg(20 * T),
    mass_max: MassKg(300 * T),
    suspension: SuspensionKind::HybridEms,
    pole: PoleMaterial::Fe,
    // Unmanned closure; presence wins unless a two-person override (FAB-EXO-507).
    regime: gz(false, false, true, false, 50),
    safe_state: SafeStatePolicy::CloseAfterShaking,
    seismic_role: SeismicRole::PostShakingClosure,
    rest_policy: RestPolicy::Land,
    // 5–20 m (20 m preset), 0.5 m/s, 0.05 m/s², 0.1 m/s³.
    kinematics: Kinematics {
        travel_mm: 20_000,
        v_max_mmps: 500,
        a_max_mmps2: 50,
        j_max_mmps3: 100,
        period_ms: 0,
    },
    badge: Badge::Research,
    trl: Trl::new(2, 2),
    row: ClassRow {
        class: "TIDEGATE",
        design_mass: "150 t",
        suspension: "Hybrid plus wheels",
        regime: "GZ; decision path SIL 3 target (SIL 4 unclaimable) / CloseAfterShaking",
        maturity: "TRL 2 (triggering 7–9)",
        effort: "Fusion, tsunami timing, closure planner",
    },
};

/// SELENE lunar hatch (frontier tier; data only).
pub const SELENE: ExoPreset = ExoPreset {
    class: ExoClass::Selene,
    design_mass: MassKg(100 * T),
    mass_min: MassKg(T),
    mass_max: MassKg(100 * T),
    suspension: SuspensionKind::HtsBulk,
    pole: PoleMaterial::None,
    // Teleoperated; braking not specified (separation distance unbounded).
    regime: gz(false, false, false, true, 0),
    safe_state: SafeStatePolicy::LandAndLock,
    seismic_role: SeismicRole::LandSecure,
    rest_policy: RestPolicy::LandLock,
    // 5–10 m hatch (10 m preset), 0.5 m/s; a and j not specified.
    kinematics: Kinematics {
        travel_mm: 10_000,
        v_max_mmps: 500,
        a_max_mmps2: 0,
        j_max_mmps3: 0,
        period_ms: 0,
    },
    badge: Badge::Frontier,
    trl: Trl::new(1, 2),
    row: ClassRow {
        class: "SELENE",
        design_mass: "100 t hatch",
        suspension: "Passive HTS in a PSR",
        regime: "Teleoperated / LandAndLock",
        maturity: "🔭",
        effort: "numpy frontier only (FRONTIER.md)",
    },
};

/// Every preset, in [`ExoClass`] discriminant order.
pub const PRESETS: [ExoPreset; 9] = [
    SENTINEL, HAVEN_R, HAVEN_F, SHIELD, BASTION, ARENA, ROTATOR, TIDEGATE, SELENE,
];

/// The preset of class `c`.
pub const fn preset(c: ExoClass) -> ExoPreset {
    match c {
        ExoClass::Sentinel => SENTINEL,
        ExoClass::HavenR => HAVEN_R,
        ExoClass::HavenF => HAVEN_F,
        ExoClass::Shield => SHIELD,
        ExoClass::Bastion => BASTION,
        ExoClass::Arena => ARENA,
        ExoClass::Rotator => ROTATOR,
        ExoClass::Tidegate => TIDEGATE,
        ExoClass::Selene => SELENE,
    }
}

/// The preset for a class slug (`"sentinel"`, `"haven_r"`, …).
pub fn by_slug(slug: &str) -> Option<ExoPreset> {
    ExoClass::from_slug(slug).map(preset)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::envelope::{attended_jog_cap, bumper_rating_ke2};
    use crate::types::{AnyClass, ClassRegime};
    use crate::units::VelMmps;
    use crate::zone::iso13855_separation_mm;
    use fabrail_types::SeismicAction;

    /// The EXOTIC §1 table rows as cells.
    fn doc_rows() -> Vec<Vec<String>> {
        let doc = include_str!("../../../docs/EXOTIC.md");
        let start = doc.find("## §1 Classes").expect("EXOTIC.md has §1");
        let end = doc.find("## §2 Anchors").expect("EXOTIC.md has §2");
        doc[start..end]
            .lines()
            .filter(|l| l.starts_with('|') && !l.starts_with("| Class") && !l.starts_with("|---"))
            .map(|l| {
                l.trim()
                    .trim_matches('|')
                    .split('|')
                    .map(|c| c.trim().to_string())
                    .collect()
            })
            .collect()
    }

    /// Parses the leading design mass of a table cell ("1,000 t", "5 kt") in kg.
    fn mass_kg(cell: &str) -> u64 {
        let mut parts = cell.split_whitespace();
        let num: u64 = parts
            .next()
            .unwrap_or("0")
            .replace(',', "")
            .parse()
            .unwrap_or(0);
        let unit = parts.next().unwrap_or("").trim_end_matches([',', ';']);
        match unit {
            "t" => num * 1000,
            "kt" => num * 1_000_000,
            _ => 0,
        }
    }

    // req: FAB-EXO-101
    #[test]
    fn class_table_matches_plan() {
        let rows = doc_rows();
        assert_eq!(rows.len(), PRESETS.len(), "one EXOTIC §1 row per class");
        for (p, row) in PRESETS.iter().zip(rows.iter()) {
            let r = p.row;
            let cells = [
                r.class,
                r.design_mass,
                r.suspension,
                r.regime,
                r.maturity,
                r.effort,
            ];
            assert_eq!(row.len(), cells.len(), "{:?}: six cells", p.class);
            for (a, b) in cells.iter().zip(row.iter()) {
                assert_eq!(*a, b.as_str(), "{:?}: EXOTIC §1 cell drifted", p.class);
            }
            assert!(r.class.starts_with(p.class.display()), "{:?}", p.class);
            assert_eq!(
                p.design_mass.0 as u64,
                mass_kg(r.design_mass),
                "{:?}: design mass",
                p.class
            );
            assert!(p.mass_min <= p.design_mass && p.design_mass <= p.mass_max);
            // Regime and safe state as written in the table.
            if r.regime.starts_with("Teleoperated") {
                assert!(
                    matches!(p.regime, SafetyRegime::GuardedZone(z) if z.teleoperated),
                    "{:?}: teleoperated guarded zone",
                    p.class
                );
            } else {
                assert!(
                    r.regime.starts_with(p.regime.label()),
                    "{:?}: regime",
                    p.class
                );
            }
            if r.regime != "OP" {
                assert!(
                    r.regime.contains(p.safe_state.name()),
                    "{:?}: safe state",
                    p.class
                );
            }
            // Badge: 🔭 exactly when maturity is frontier; otherwise research tier.
            assert_eq!(
                p.badge == Badge::Frontier,
                r.maturity == "🔭",
                "{:?}",
                p.class
            );
            assert!(p.trl.lo >= 1 && p.trl.lo <= p.trl.hi && p.trl.hi <= 9);
            // FeCo appears in the table exactly where it is a parameter.
            assert_eq!(
                p.pole == PoleMaterial::FeCo,
                r.suspension.contains("FeCo") || p.class == ExoClass::HavenF,
                "{:?}: FeCo pole parameter",
                p.class
            );
        }
        // Discriminant order and lookup.
        for (i, p) in PRESETS.iter().enumerate() {
            assert_eq!(p.class.as_u8() as usize, i);
            assert_eq!(preset(p.class), *p);
            assert_eq!(by_slug(p.class.slug()), Some(*p));
        }
    }

    // req: FAB-EXO-102
    #[test]
    fn every_class_declares_regime_and_policy() {
        let mut actions = Vec::new();
        for p in PRESETS {
            assert!(
                ClassRegime::new(AnyClass::Exo(p.class), p.regime).is_ok(),
                "{:?}: admissible regime (never Pedestrian)",
                p.class
            );
            let a = p.safe_state.seismic_action();
            assert_ne!(a, SeismicAction::None, "{:?}: a protective action", p.class);
            actions.push(a);
        }
        for need in [
            SeismicAction::RideThrough,
            SeismicAction::CompleteClose,
            SeismicAction::CloseAfterShaking,
            SeismicAction::Lock,
            SeismicAction::Isolate,
            SeismicAction::Land,
        ] {
            assert!(actions.contains(&need), "{need:?} is used by some class");
        }
        // Land-first is not universal (plan §4).
        assert_eq!(SENTINEL.safe_state, SafeStatePolicy::RideThrough);
        assert_eq!(SENTINEL.rest_policy, RestPolicy::Hover);
        assert_eq!(BASTION.safe_state, SafeStatePolicy::CompleteClose);
        assert_eq!(TIDEGATE.safe_state, SafeStatePolicy::CloseAfterShaking);
        assert_eq!(TIDEGATE.seismic_role, SeismicRole::PostShakingClosure);
        assert_eq!(HAVEN_R.seismic_role, SeismicRole::PrimaryIsolation);
        assert!(matches!(HAVEN_R.regime, SafetyRegime::OccupiedPlatform(_)));
        assert!(matches!(SHIELD.regime, SafetyRegime::GuardedZone(z) if z.pps));
        assert!(matches!(BASTION.regime, SafetyRegime::GuardedZone(z) if z.two_person_override));
        assert!(matches!(SELENE.regime, SafetyRegime::GuardedZone(z) if z.teleoperated));
    }

    #[test]
    fn derived_limits_match_anchors() {
        // ISO 13855 S (EXOTIC §2): SHIELD 2.61 m, BASTION 5.01 m, ARENA 10.61 m.
        for (p, s_mm) in [(SHIELD, 2610u64), (BASTION, 5010), (ARENA, 10_610)] {
            let SafetyRegime::GuardedZone(z) = p.regime else {
                panic!("{:?} is a guarded zone", p.class)
            };
            assert_eq!(
                iso13855_separation_mm(&z, p.kinematics.v_max_mmps),
                s_mm,
                "{:?}",
                p.class
            );
            assert_eq!(z.edge_k_n_per_mm, 10, "≤ 10 N/mm compliant edge");
        }
        // HAVEN-F bumper 3.6 MJ at 1.2 m/s with rated mass (FAB-EXO-504).
        let b = bumper_rating_ke2(
            HAVEN_F.design_mass,
            VelMmps(HAVEN_F.kinematics.v_max_mmps as i32),
        );
        assert_eq!(b.map(|k| k.to_joules_f64()), Some(3.6e6));
        // SHIELD at 50 t: the attended jog is energy-limited below 10 mm/s.
        assert_eq!(
            attended_jog_cap(SHIELD.design_mass, VelMmps(10)),
            VelMmps(8)
        );
        assert_eq!(A_0P1G_MMPS2, 981);
    }
}
