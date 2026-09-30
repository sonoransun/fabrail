//! PlantModel trait (reset, step, truth), PlantInputs, Truth, InitialState, ExtForces,
//! ContactForces, EnergyLedger. Owner: A6a (signatures frozen by W0-B3).
//!
//! The plant is the f64 truth (core design §A.4 item 1): every model tier implements
//! [`PlantModel`] and is driven by the sim harness once per plant step. Controllers
//! never see [`Truth`]; `SimBoard` converts it to integer raw codes with noise and
//! quantisation (PHYSICS §13).
//!
//! Frames and signs (PHYSICS §1): x is travel along the rail (+x opens), y is
//! cross-track, z is up. Every force in this module is signed along +x unless its doc
//! says otherwise.
//!
//! Tiers (core design §C.1; PHYSICS §5):
//!
//! | Tier | Step | Implemented in |
//! |---|---|---|
//! | Carriage6 | 50 µs (the base tick) | [`crate::body::Carriage6`] |
//! | Heave1 | 50 µs | [`crate::heave1::Heave1`] |
//! | Travel1d | 1 ms | [`crate::travel1d::Travel1d`] |
//! | Structure | 1 ms | [`crate::structure::Structure`] |
//!
//! Coil mode has a single source, the scenario (plan §2b item 16): it is fixed at
//! construction in [`ModelConfig::coil`] and never travels per step.

use fabrail_types::{BASE_TICK_S, LsmMode, MAX_MAGNETS};

use crate::interaction::{HandInput, Obstacle};
use crate::params::PlantParams;

/// Plant model tier (core design §C.1).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Tier {
    /// Rigid body p, q, v, ω plus per-element coil states, LSM, bristles, cavity and
    /// thermal nodes. All closed-loop acceptance, haptics, obstruction, SIL.
    #[default]
    Carriage6,
    /// z, ż, i, Φ_eff (≡ Carriage6 constrained heave to 1e-9). Anchors, gain studies,
    /// vertical pulse, thermal.
    Heave1,
    /// x, ẋ, bristle z, lock, levitated/landed flag with kinematic heave emulation for
    /// LAND (plan §2b item 16). Seismic Monte Carlo, end zones, EEW timelines.
    Travel1d,
    /// n-storey shear building (Rayleigh 2–5 %) plus the maglev mass with LSM force and
    /// stroke. AMD/TMD research only (SEI-021).
    Structure,
}

impl Tier {
    /// Every tier, in declaration order.
    pub const ALL: &'static [Tier] = &[
        Tier::Carriage6,
        Tier::Heave1,
        Tier::Travel1d,
        Tier::Structure,
    ];

    /// Nominal plant step, s: the 50 µs base tick for Carriage6/Heave1, 1 ms for
    /// Travel1d/Structure (core design §A.5 "travel1d tier", §C.1).
    #[inline]
    pub const fn nominal_h_s(self) -> f64 {
        match self {
            Tier::Carriage6 | Tier::Heave1 => BASE_TICK_S,
            Tier::Travel1d | Tier::Structure => 1.0e-3,
        }
    }

    /// Base ticks (20 kHz) per plant step: 1 for Carriage6/Heave1, 20 for the 1 ms tiers.
    #[inline]
    pub const fn ticks_per_step(self) -> u64 {
        match self {
            Tier::Carriage6 | Tier::Heave1 => 1,
            Tier::Travel1d | Tier::Structure => 20,
        }
    }

    /// Lower-case name as used in `fabrail.scenario/1` (`fidelity.model`).
    #[inline]
    pub const fn name(self) -> &'static str {
        match self {
            Tier::Carriage6 => "carriage6",
            Tier::Heave1 => "heave1",
            Tier::Travel1d => "travel1d",
            Tier::Structure => "structure",
        }
    }
}

/// How `PlantInputs::coil_v_or_iref` is interpreted (PHYSICS §4).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum CoilMode {
    /// Current references (A) through the exact-discretised current-loop lag
    /// τ_cl (sim-only; the golden/replay/SIL runs never use it, plan §2b item 16).
    #[default]
    Lag,
    /// Bridge voltages (V) into the full flux-linkage ODE with motional EMF. SIL ≡
    /// firmware is claimed in this mode only (plan §2b item 16).
    Voltage,
}

/// Construction-time model options. Every field comes from the scenario's `fidelity`
/// block, the single source (plan §2b item 16).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ModelConfig {
    /// Model tier.
    pub tier: Tier,
    /// Coil mode.
    pub coil: CoilMode,
    /// Model the eddy lag on the force flux (τ_e, PHYSICS §2). With `false` the flux
    /// follows the coil current instantaneously.
    pub eddy_lag: bool,
    /// Travel1d only: emulate heave kinematically (gap ramps at the class land rate
    /// following [`PlantInputs::heave_hint`]) so LAND/SEAT sequences complete
    /// (plan §2b item 16). Ignored by the other tiers.
    pub kinematic_heave: bool,
}

impl Default for ModelConfig {
    fn default() -> Self {
        ModelConfig {
            tier: Tier::Carriage6,
            coil: CoilMode::Lag,
            eddy_lag: true,
            kinematic_heave: true,
        }
    }
}

/// Vertical rest condition at reset.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum InitLev {
    /// In the DRAWER captive seat at g_seat (F17).
    Seated,
    /// On the landing rollers at g_land.
    Landed,
    /// Levitated at the zero-power gap for the initial mass.
    #[default]
    Levitated,
}

/// Initial condition applied by [`PlantModel::reset`]. The sim harness resolves it from
/// the scenario's `initial` block (`fabrail_sim::InitialSpec::resolve`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InitialState {
    /// Vertical rest condition.
    pub lev: InitLev,
    /// Travel position (absolute rail coordinate), m.
    pub x_m: f64,
    /// Travel velocity relative to the rail, m/s.
    pub v_mps: f64,
    /// Payload added to the body at t = 0, kg (may be negative down to the empty
    /// body; body truth mass = `PlantParams::m_body_kg + payload_kg`).
    pub payload_kg: f64,
    /// Initial temperature of every thermal node and of the ambient, °C.
    pub temp_c: f64,
    /// Parking lock engaged at t = 0 (HEAVY LAND + lock rest).
    pub locked: bool,
}

impl Default for InitialState {
    fn default() -> Self {
        InitialState {
            lev: InitLev::Levitated,
            x_m: 0.0,
            v_mps: 0.0,
            payload_kg: 0.0,
            temp_c: 20.0,
            locked: false,
        }
    }
}

/// Travel1d kinematic heave emulation: the levitation directive in force, supplied by
/// the sim harness (plan §2b item 16). Carriage6 and Heave1 ignore it; they respond to
/// coil commands only.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum HeaveHint {
    /// Keep the current vertical state.
    #[default]
    Hold,
    /// Lift / unseat to the zero-power gap (class lift time, core design §C.4).
    Lift,
    /// Descend at the class land speed to the rollers (g_land).
    Land,
    /// DRAWER: descend into the captive seat (g_seat).
    Seat,
    /// Uncontrolled drop to the rollers (coils off, control loss down).
    Drop,
}

/// External (non-actuator) loads on the body at this step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ExtForces {
    /// Hand in contact (impedance plus driving force), or `None`.
    pub hand: Option<HandInput>,
    /// Lean force along x (a person leaning on the front), N.
    pub lean_n: f64,
    /// Pressure difference acting on the leaf/front, Pa; positive pushes toward +x
    /// (cabinet pressure pushes outward, plan §2b item 11). Force = Δp · area
    /// (`design.air.leaf_area_m2` for DOOR, `cavity_area_m2` for DRAWER).
    pub smoke_dp_pa: f64,
    /// Obstacle body in the closing path, or `None`.
    pub obstacle: Option<Obstacle>,
}

/// Everything that drives the plant for one step. Built by `SimBoard` from latched HAL
/// commands and the event executor's state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlantInputs {
    /// Per-magnet coil command: bridge voltage (V) in [`CoilMode::Voltage`], current
    /// reference (A) in [`CoilMode::Lag`] (mode fixed at construction).
    pub coil_v_or_iref: [f64; MAX_MAGNETS],
    /// Coil bridges enabled (`CoilCmd::Off` → false: currents decay through the
    /// freewheel path).
    pub coil_enabled: bool,
    /// LSM quadrature (thrust) current reference, A.
    pub iq_ref_a: f64,
    /// LSM direct-axis current reference, A.
    pub id_ref_a: f64,
    /// LSM inverter mode (STO/ASC act on propulsion only, C11).
    pub lsm_mode: LsmMode,
    /// Parking lock commanded engaged.
    pub parking_lock: bool,
    /// Seismic x-latch commanded engaged (`landing.seismic_latch`).
    pub seismic_latch: bool,
    /// Mains present (false → the bus runs from the hold-up supercap, PHYSICS §14).
    pub mains_ok: bool,
    /// Total payload currently on the body (initial + payload events), kg.
    pub payload_kg: f64,
    /// Ambient temperature (initial + temp_ramp events), °C.
    pub t_ambient_c: f64,
    /// Cavity leak area override (piston events), m²; `None` = the plant value.
    pub air_leak_area_m2: Option<f64>,
    /// Travel1d kinematic heave hint (ignored by other tiers).
    pub heave_hint: HeaveHint,
    /// External loads.
    pub f_ext: ExtForces,
}

impl Default for PlantInputs {
    fn default() -> Self {
        PlantInputs {
            coil_v_or_iref: [0.0; MAX_MAGNETS],
            coil_enabled: false,
            iq_ref_a: 0.0,
            id_ref_a: 0.0,
            lsm_mode: LsmMode::Sto,
            parking_lock: false,
            seismic_latch: false,
            mains_ok: true,
            payload_kg: 0.0,
            t_ambient_c: 20.0,
            air_leak_area_m2: None,
            heave_hint: HeaveHint::Hold,
            f_ext: ExtForces::default(),
        }
    }
}

/// Contact normal forces at the end of the step, N (magnitudes, ≥ 0; PHYSICS §8).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactForces {
    /// Upper stop / elastomer upper rollers (LEV-006: ≤ 1.3 W design).
    pub upper_n: f64,
    /// Landing / travel rollers.
    pub rollers_n: f64,
    /// DRAWER captive seat (z both ways, magnitude).
    pub seat_n: f64,
    /// Bottom guide fin (lateral).
    pub guide_fin_n: f64,
    /// UHMW side pads (lateral).
    pub pads_n: f64,
    /// End bumpers (Hunt–Crossley).
    pub bumper_n: f64,
    /// Physical end stops (magnitude): the closed-end jamb or DRAWER carcass push-stop
    /// while not seated, and the open-end rail stop behind the bumper (W2 lead decision
    /// D3; `contact::end_stops`), N. The seated DRAWER's push-stop is in `seat_n`'s x
    /// counterpart, the seat law.
    pub end_stop_n: f64,
    /// Parking lock (x retention).
    pub lock_n: f64,
    /// Seismic x-latch (x retention).
    pub latch_n: f64,
    /// Obstacle body through the closing-edge seal (series k_edge + body).
    pub obstacle_n: f64,
    /// Total tangential (friction) force on the body along x, signed.
    pub tangential_x_n: f64,
}

/// Cumulative energy ledger since reset, J (PHYSICS §9). Dissipation terms are ≥ 0.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EnergyLedger {
    /// Current kinetic energy of the body (rail frame), J (not cumulative).
    pub kinetic_j: f64,
    /// Distributed eddy damping b_dist.
    pub dist_j: f64,
    /// Progressive end zones.
    pub end_zone_j: f64,
    /// Rail eddy drag c_rail·ẋ (scaled by ΣΦ²/ΣΦ0²).
    pub rail_eddy_j: f64,
    /// Rail hysteresis drag F_hyst.
    pub rail_hyst_j: f64,
    /// Air drag and cavity throttling.
    pub air_j: f64,
    /// Bristle friction (rollers, pads, fin, seat).
    pub friction_j: f64,
    /// Penalty contact damping (stops, seat, lock, obstacle).
    pub contact_j: f64,
    /// Bumper impacts.
    pub bumper_j: f64,
    /// Coil copper loss.
    pub coil_copper_j: f64,
    /// LSM copper loss.
    pub lsm_copper_j: f64,
    /// Net electrical energy delivered by the bus to coils and LSM.
    pub elec_in_j: f64,
    /// Work done on the body by the hand.
    pub hand_work_j: f64,
    /// Work done on the body by floor motion (inertial load).
    pub floor_work_j: f64,
    /// Number of bumper impacts (contact onsets).
    pub bumper_impacts: u32,
    /// Largest single bumper impact energy, J.
    pub bumper_e_max_j: f64,
}

/// Bus and hold-up truth (PHYSICS §14).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PowerTruth {
    /// DC bus voltage, V.
    pub v_bus_v: f64,
    /// Hold-up supercap voltage, V.
    pub v_hold_v: f64,
    /// Bus current, A (negative = regeneration).
    pub i_bus_a: f64,
    /// Bus power drawn by coils + LSM + electronics, W.
    pub p_bus_w: f64,
    /// Usable hold-up energy remaining above `supercap_v_min_v`, J.
    pub e_hold_j: f64,
    /// Mains present.
    pub mains_ok: bool,
    /// Brake chopper conducting.
    pub chopper_on: bool,
}

/// Vertical/mechanical contact state flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct ContactState {
    /// No load path through rollers, seat or upper stop (magnetically suspended).
    pub levitated: bool,
    /// In the DRAWER captive seat.
    pub seated: bool,
    /// On the landing/travel rollers.
    pub landed: bool,
    /// Against the upper stop.
    pub upper_stop: bool,
    /// Parking lock engaged.
    pub locked: bool,
    /// Seismic x-latch engaged.
    pub latched: bool,
}

/// Plant truth after the last step (f64 SI). Travel quantities are relative to the rail
/// (floor frame); `acc_abs_mps2` is inertial.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Truth {
    /// Plant time at the end of the last step, s.
    pub t_s: f64,
    /// Travel position (absolute rail coordinate), m.
    pub x_m: f64,
    /// Travel velocity relative to the rail, m/s.
    pub v_mps: f64,
    /// Travel acceleration relative to the rail, m/s².
    pub a_mps2: f64,
    /// Pose [x, y, z, qw, qx, qy, qz]: position of the carriage reference point in the
    /// rail frame (z = heave relative to the zero-power position at g*, up +), m, and
    /// the unit attitude quaternion.
    pub pose: [f64; 7],
    /// Body angular velocity (roll, pitch, yaw), rad/s.
    pub omega_radps: [f64; 3],
    /// Absolute (inertial) acceleration of the carriage IMU point, excluding gravity,
    /// resolved in the carriage frame, m/s² (skyhook/fusion only, never EEW; F16).
    pub acc_abs_mps2: [f64; 3],
    /// Per-element air gap (unused elements 0), m.
    pub gaps_m: [f64; MAX_MAGNETS],
    /// Per-element coil current, A.
    pub coil_i_a: [f64; MAX_MAGNETS],
    /// LSM quadrature current, A.
    pub iq_a: f64,
    /// LSM direct-axis current, A.
    pub id_a: f64,
    /// LSM phase currents a, b, c, A.
    pub i_abc_a: [f64; 3],
    /// LSM thrust along x (including ripple and cogging), N.
    pub f_lsm_n: f64,
    /// Contact forces.
    pub f_contact: ContactForces,
    /// Force the hand applies to the body along x, N.
    pub f_hand_n: f64,
    /// Energy ledger.
    pub e_diss: EnergyLedger,
    /// Floor (rail) acceleration at the end of the step (x, y, z), m/s².
    pub floor_acc_mps2: [f64; 3],
    /// Temperatures [coil hot-spot, magnet], °C.
    pub temps_c: [f64; 2],
    /// DRAWER cavity overpressure, Pa.
    pub cavity_dp_pa: f64,
    /// Bus and hold-up.
    pub power: PowerTruth,
    /// Contact state flags.
    pub state: ContactState,
    /// Total body truth mass including payload, kg.
    pub mass_kg: f64,
    /// Payload currently on the body, kg.
    pub payload_kg: f64,
    /// Structure tier only: device-storey displacement relative to the ground, m.
    pub host_x_m: f64,
    /// Structure tier only: device-storey velocity relative to the ground, m/s.
    pub host_v_mps: f64,
    /// Structure tier only: device-storey absolute acceleration, m/s².
    pub host_a_mps2: f64,
}

impl Default for Truth {
    fn default() -> Self {
        Truth {
            t_s: 0.0,
            x_m: 0.0,
            v_mps: 0.0,
            a_mps2: 0.0,
            pose: [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            omega_radps: [0.0; 3],
            acc_abs_mps2: [0.0; 3],
            gaps_m: [0.0; MAX_MAGNETS],
            coil_i_a: [0.0; MAX_MAGNETS],
            iq_a: 0.0,
            id_a: 0.0,
            i_abc_a: [0.0; 3],
            f_lsm_n: 0.0,
            f_contact: ContactForces::default(),
            f_hand_n: 0.0,
            e_diss: EnergyLedger::default(),
            floor_acc_mps2: [0.0; 3],
            temps_c: [20.0, 20.0],
            cavity_dp_pa: 0.0,
            power: PowerTruth::default(),
            state: ContactState::default(),
            mass_kg: 0.0,
            payload_kg: 0.0,
            host_x_m: 0.0,
            host_v_mps: 0.0,
            host_a_mps2: 0.0,
        }
    }
}

impl Truth {
    /// The quasi-static truth at reset for `init`: position, mass, temperatures, bus
    /// and hold-up at their charged values, contact flags, and every active element's
    /// gap at the rest gap (g_seat / g_land / g*). Dynamic states (currents, forces,
    /// ledger) start at zero. Tiers call this from `reset` and then settle their own
    /// states.
    pub fn initial(p: &PlantParams, init: &InitialState) -> Truth {
        let s = &p.design.suspension;
        let gap = match init.lev {
            InitLev::Seated => s.g_seat_m,
            InitLev::Landed => s.g_land_m,
            InitLev::Levitated => s.g_star_m,
        };
        let n = (s.n_magnets as usize).min(MAX_MAGNETS);
        let mut gaps_m = [0.0; MAX_MAGNETS];
        for g in gaps_m.iter_mut().take(n) {
            *g = gap;
        }
        let pw = &p.design.power;
        let e_hold_j = crate::power::supercap_energy_j(pw.supercap_c_f, pw.supercap_v_max_v)
            - crate::power::supercap_energy_j(pw.supercap_c_f, pw.supercap_v_min_v);
        Truth {
            x_m: init.x_m,
            v_mps: init.v_mps,
            pose: [init.x_m, 0.0, s.g_star_m - gap, 1.0, 0.0, 0.0, 0.0],
            gaps_m,
            e_diss: EnergyLedger {
                kinetic_j: 0.5 * (p.m_body_kg + init.payload_kg) * init.v_mps * init.v_mps,
                ..EnergyLedger::default()
            },
            temps_c: [init.temp_c, init.temp_c],
            power: PowerTruth {
                v_bus_v: pw.v_bus_v,
                v_hold_v: pw.supercap_v_max_v,
                e_hold_j: e_hold_j.max(0.0),
                mains_ok: true,
                ..PowerTruth::default()
            },
            state: ContactState {
                levitated: matches!(init.lev, InitLev::Levitated),
                seated: matches!(init.lev, InitLev::Seated),
                landed: matches!(init.lev, InitLev::Landed),
                upper_stop: false,
                locked: init.locked,
                latched: false,
            },
            mass_kg: p.m_body_kg + init.payload_kg,
            payload_kg: init.payload_kg,
            ..Truth::default()
        }
    }
}

/// Errors building a plant model from its configuration.
#[derive(Clone, Debug, PartialEq)]
pub enum PlantError {
    /// The Structure tier needs `Environment::structure`.
    MissingStructure,
    /// A configuration value is out of range: (what, reason).
    Invalid(&'static str, &'static str),
}

impl core::fmt::Display for PlantError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PlantError::MissingStructure => {
                write!(
                    f,
                    "plant.structure: the structure tier needs a shear-building spec"
                )
            }
            PlantError::Invalid(what, reason) => write!(f, "{what}: {reason}"),
        }
    }
}

impl std::error::Error for PlantError {}

/// A plant model tier (core design §B.6).
///
/// Contract:
/// * `reset` puts the model at `init` in quasi-static equilibrium (no transient on the
///   first step).
/// * `step` advances from `t` to `t + h` with one RK4 step (PHYSICS §24) holding `u`
///   constant (zero-order hold) and taking the floor acceleration at t, t + h/2 and
///   t + h from `floor_mps2` (each [x, y, z] m/s²; the harness interpolates the floor
///   series linearly).
/// * `truth` is the state at the end of the last step. Deterministic and RNG-free.
pub trait PlantModel {
    /// This model's tier.
    fn tier(&self) -> Tier;
    /// Resets to the initial condition.
    fn reset(&mut self, init: &InitialState);
    /// Advances one step of `h` seconds starting at `t`.
    fn step(&mut self, t: f64, h: f64, u: &PlantInputs, floor_mps2: &[[f64; 3]; 3]);
    /// Truth after the last step.
    fn truth(&self) -> &Truth;
}

/// Builds the model tier `cfg.tier` over `p`. Fails only when the Structure tier has
/// no shear-building spec.
pub fn build(cfg: &ModelConfig, p: &PlantParams) -> Result<Box<dyn PlantModel + Send>, PlantError> {
    Ok(match cfg.tier {
        Tier::Carriage6 => Box::new(crate::body::Carriage6::new(p, cfg)),
        Tier::Heave1 => Box::new(crate::heave1::Heave1::new(p, cfg)),
        Tier::Travel1d => Box::new(crate::travel1d::Travel1d::new(p, cfg)),
        Tier::Structure => {
            let b = p
                .env
                .structure
                .as_ref()
                .ok_or(PlantError::MissingStructure)?;
            Box::new(crate::structure::Structure::new(p, b, cfg)?)
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::{Environment, Mismatch};
    use fabrail_params::DesignParams;

    fn plant() -> PlantParams {
        let mut d = DesignParams::default();
        d.carriage.m_star_kg = 100.0;
        d.suspension.n_magnets = 4;
        d.suspension.g_star_m = 5.0e-3;
        d.suspension.g_land_m = 6.25e-3;
        d.power.v_bus_v = 48.0;
        d.power.supercap_c_f = 10.0;
        d.power.supercap_v_max_v = 12.0;
        d.power.supercap_v_min_v = 6.0;
        PlantParams::from_design(&d, &Mismatch::default(), &Environment::default())
    }

    #[test]
    fn tier_steps_are_consistent_with_the_base_tick() {
        for &t in Tier::ALL {
            let h = t.ticks_per_step() as f64 * BASE_TICK_S;
            assert!((h - t.nominal_h_s()).abs() < 1e-15, "{}", t.name());
        }
    }

    #[test]
    fn initial_truth_sets_rest_gaps_mass_and_holdup() {
        let p = plant();
        let init = InitialState {
            lev: InitLev::Landed,
            x_m: 0.25,
            payload_kg: 10.0,
            ..InitialState::default()
        };
        let t = Truth::initial(&p, &init);
        assert_eq!(t.gaps_m[..4], [6.25e-3; 4]);
        assert_eq!(t.gaps_m[4..], [0.0; 4]);
        assert_eq!(t.mass_kg, 110.0);
        assert!(t.state.landed && !t.state.levitated);
        assert!(
            (t.pose[2] + 1.25e-3).abs() < 1e-15,
            "landed heave is below g*"
        );
        // ½·C·(Vmax² − Vmin²) = ½·10·(144 − 36) = 540 J (PHYSICS §14).
        assert!((t.power.e_hold_j - 540.0).abs() < 1e-9);
    }

    #[test]
    fn every_tier_builds_resets_and_steps_without_panicking() {
        let mut p = plant();
        p.env.structure = Some(crate::structure::ShearBuilding {
            storey_mass_kg: vec![2.0e5],
            storey_k_n_per_m: vec![8.0e7],
            rayleigh_zeta: [0.02, 0.02],
            device_storey: 1,
        });
        for &tier in Tier::ALL {
            let cfg = ModelConfig {
                tier,
                ..ModelConfig::default()
            };
            let mut m = build(&cfg, &p).expect("build");
            assert_eq!(m.tier(), tier);
            m.reset(&InitialState::default());
            let u = PlantInputs::default();
            m.step(0.0, tier.nominal_h_s(), &u, &[[0.0, 0.0, 0.1]; 3]);
            assert_eq!(m.truth().floor_acc_mps2, [0.0, 0.0, 0.1]);
            assert!(m.truth().t_s > 0.0);
        }
    }

    #[test]
    fn structure_tier_without_building_is_rejected() {
        let cfg = ModelConfig {
            tier: Tier::Structure,
            ..ModelConfig::default()
        };
        assert!(matches!(
            build(&cfg, &plant()),
            Err(PlantError::MissingStructure)
        ));
    }
}
