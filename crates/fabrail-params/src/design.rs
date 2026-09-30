//! Design parameters: the nominal hardware as built (core design §B.2, §C.2 as corrected
//! by plan §2b). Values live in `presets.rs`; each preset value cites its source there.
//!
//! Conventions:
//! * every physical field is f64 SI with a unit suffix (`_m`, `_kg`, `_a`, `_t` tesla,
//!   `_ohm`, `_h` henry, `_n_per_m`, …) (plan §2b item 8: f64 keeps the 1e-12 anchors;
//!   controllers cast to f32 in `new()`);
//! * counts and flags keep integer/bool types;
//! * frames per PHYSICS §1: x travel (+ open), y cross-track, z up; carriage-frame
//!   offsets are measured from the carriage reference point on the rail centreline at
//!   mid-carriage, rail underside.

use fabrail_types::{EgressRole, MAX_MAGNETS, PresenceSensing, RailClass, RestPolicy};

/// How the device may be operated without presence sensing (core design §C.2 row
/// "Operating mode"; SAF-003).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum OperatingMode {
    /// Automatic moves (DRAWER; WALL unsensed at the momentum cap).
    Automatic,
    /// Automatic, low-energy door (A156.19 / EN 16005 low energy; DOOR).
    #[default]
    LowEnergy,
    /// Hold-to-run only (HEAVY unsensed; any class after PayloadMismatch).
    HoldToRun,
}

/// LSM construction (PHYSICS §7).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum LsmKind {
    /// Ironless Halbach array (DRAWER): no cogging, no normal force.
    IronlessHalbach,
    /// Ironless (DOOR).
    #[default]
    Ironless,
    /// Ironless, segmented stator (WALL): K_f(x) segment overlap.
    IronlessSegmented,
    /// Iron-core, segmented stator (HEAVY): cogging and normal force F_n.
    IronCoreSegmented,
}

/// What the carriage rests on when not levitated (PHYSICS §8).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum LandingKind {
    /// Captive registration seat plus travel rollers (DRAWER, F17).
    Seat,
    /// Landing rollers (DOOR, WALL, HEAVY).
    #[default]
    Rollers,
    /// Skids (sovereign reduced models only).
    Skids,
}

/// Where the earthquake-early-warning accelerometer is mounted. Validation rejects a
/// carriage mount (SEI-001, core design F16).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum AccelMount {
    /// On the building structure (default).
    #[default]
    Structure,
    /// On the rail (fixed to the structure).
    Rail,
    /// On the carriage: invalid for EEW (moves with the controlled body).
    Carriage,
}

/// Top-level design block.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct DesignParams {
    /// Product class.
    pub class: RailClass,
    /// Rest state (PWR-001): SEAT / HOVER / LAND / LAND + lock.
    pub rest_policy: RestPolicy,
    /// Egress role of the opening (SEI-011).
    pub egress: EgressRole,
    /// Presence sensing fitted (SAF-003).
    pub presence: PresenceSensing,
    /// Unsensed operating mode (SAF-003; F5).
    pub operating_mode: OperatingMode,
    /// Moving body.
    pub carriage: CarriageDesign,
    /// Hybrid PM-biased EMS levitation elements (PHYSICS §2–§4).
    pub suspension: SuspensionDesign,
    /// Long-stator linear synchronous motor (PHYSICS §7).
    pub lsm: LsmDesign,
    /// Passive lateral guidance (PHYSICS §6).
    pub guidance: GuidanceDesign,
    /// Dissipation, end zones, bumpers, edge seal (PHYSICS §8–§9).
    pub damping: DampingDesign,
    /// Landing rollers, DRAWER seat, latches, parking lock (PHYSICS §8, §15).
    pub landing: LandingDesign,
    /// Rail geometry and material (PHYSICS §10).
    pub rail: RailDesign,
    /// Air cavity and pressure-disturbance geometry (PHYSICS §11).
    pub air: AirDesign,
    /// Thermal network (PHYSICS §12).
    pub thermal: ThermalDesign,
    /// Sensor calibration and noise specification (PHYSICS §13).
    pub sensors: SensorCal,
    /// Bus, hold-up storage and power budgets (PHYSICS §14).
    pub power: PowerDesign,
}

/// Moving-body mass properties (PHYSICS §5).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct CarriageDesign {
    /// PM balance mass m*: the load at which the magnets alone carry the weight at g*
    /// (zero-power equilibrium, PHYSICS §3), kg.
    pub m_star_kg: f64,
    /// Rated (maximum operating) mass, kg. The governor uses max(m_rated, 1.05·m̂).
    pub m_rated_kg: f64,
    /// Minimum operating mass (empty body), kg. Stay-landed rule F18 uses it.
    pub m_op_min_kg: f64,
    /// Principal moments of inertia about the CoM (roll x, pitch y, yaw z), kg·m².
    pub j_kgm2: [f64; 3],
    /// CoM offset from the carriage reference point (x, y, z), m. Top-hung leaves have
    /// z < 0 (DOOR: 1.05 m below the rail).
    pub cg_m: [f64; 3],
    /// Body hangs below the rail (DOOR, WALL): roll is gravity-stabilised and passive.
    pub top_hung: bool,
    /// Body length along x, m (DRAWER depth sets the extension fraction, LEV-016).
    pub length_m: f64,
}

/// One levitation element's mounting station in the carriage frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct MagnetStation {
    /// Station x, m.
    pub x_m: f64,
    /// Station y, m.
    pub y_m: f64,
    /// Station z (pole face), m.
    pub z_m: f64,
    /// +1 for a lift element (attracts up toward the rail), −1 for a hold-down element.
    pub normal_sign: f64,
    /// Permanent-magnet bias fitted (false = pure-EMS hold-down element, M = 0).
    pub has_pm: bool,
}

/// Hybrid PM-biased EMS elements (PHYSICS §2–§4, §15; core design §C.2).
///
/// Co-energy element: W′ = ½(M_pm + N·i)²·P(g, y), P = μ0·A_p / (2g/f(g,y) + h′),
/// h′ = h_m/μ_rec, M_pm = B_r·h_m/(μ0·μ_rec). Saturation Φ = Φ_sat·tanh(Φ_lin/Φ_sat).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct SuspensionDesign {
    /// Number of active elements (≤ MAX_MAGNETS); stations beyond it are unused.
    pub n_magnets: u8,
    /// Element stations.
    pub stations: [MagnetStation; MAX_MAGNETS],
    /// Magnet remanence B_r at 20 °C, T.
    pub b_r_t: f64,
    /// Magnet recoil relative permeability μ_rec.
    pub mu_rec: f64,
    /// Magnet thickness along the magnetisation h_m, m.
    pub h_m_m: f64,
    /// Pole face width w (across track), m.
    pub pole_w_m: f64,
    /// Pole face length l (along track), m. A_p = w·l.
    pub pole_l_m: f64,
    /// Design gap flux density at g* (sizing input, `derive::size_magnets`), T.
    pub b_g_t: f64,
    /// Coil turns N.
    pub turns: f64,
    /// Coil resistance at 20 °C, Ω.
    pub r_coil_ohm: f64,
    /// Coil leakage inductance L_σ, H.
    pub l_sigma_h: f64,
    /// Continuous coil current limit i_cont (I²t budget floor), A.
    pub i_cont_a: f64,
    /// Pulse coil current limit i_pulse (hardware clamp), A.
    pub i_pulse_a: f64,
    /// Maximum duration at i_pulse t_pulse (I²t budget (i_pulse² − i_cont²)·t_pulse), s.
    pub t_pulse_s: f64,
    /// Zero-power nominal gap g* at m*, m.
    pub g_star_m: f64,
    /// Upper stop gap g_up (elastomer upper rollers / stop), m.
    pub g_up_m: f64,
    /// Landing gap on the travel rollers g_land, m.
    pub g_land_m: f64,
    /// Captive seat gap g_seat (DRAWER 3.5 mm; 0 when no seat), m.
    pub g_seat_m: f64,
    /// Modified zero-power band [g_min, g_max]: outside it the outer PI biases current
    /// (F1), m.
    pub zp_band_m: [f64; 2],
    /// Iron/rail saturation flux density B_sat, T.
    pub b_sat_t: f64,
    /// Eddy-current lag on the force flux τ_e (solid rail ~0.5 ms), s.
    pub tau_eddy_s: f64,
    /// Remanence temperature coefficient, 1/K (NdFeB ≈ −0.0012).
    pub tempco_br_per_k: f64,
    /// Coil resistance temperature coefficient, 1/K (copper +0.00393).
    pub tempco_r_per_k: f64,
    /// Flux leakage fraction σ_leak (0 = ideal; 0.25 in sensitivity studies only).
    pub leak: f64,
    /// Stray-field target at 10 cm from accessible surfaces, T (0.5 mT; ENV-001).
    pub stray_target_t: f64,
    /// Magnetic shroud fitted (required for WALL/HEAVY, ENV-001).
    pub shroud: bool,
}

/// Long-stator LSM (PHYSICS §7). Amplitude-invariant dq: e_q = K_e·v with K_e = ⅔·K_f, so
/// F·v = 1.5·e_q·i_q (PRP-002); ASC drag b_e = K_f·K_e/R at low speed (D5).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct LsmDesign {
    /// Construction.
    pub kind: LsmKind,
    /// Pole pitch τ_p, m.
    pub tau_p_m: f64,
    /// Thrust constant K_f, N/A (i_q).
    pub k_f_n_per_a: f64,
    /// Back-EMF constant K_e, V·s/m.
    pub k_e_v_s_per_m: f64,
    /// Phase resistance, Ω.
    pub r_phase_ohm: f64,
    /// Phase synchronous inductance, H.
    pub l_phase_h: f64,
    /// Peak thrust (for ≤ `t_peak_s`), N.
    pub f_peak_n: f64,
    /// Continuous thrust, N.
    pub f_cont_n: f64,
    /// Peak-thrust duration, s (0.75 s, EN 16005 dynamic force window).
    pub t_peak_s: f64,
    /// Peak phase current, A.
    pub i_max_a: f64,
    /// 6th-harmonic thrust ripple fraction (≈ 0.01).
    pub ripple_frac: f64,
    /// Cogging force amplitude (iron-core only), N.
    pub cogging_n: f64,
    /// Normal attraction at rated current (iron-core only), N.
    pub f_normal_n: f64,
    /// Stator segment length (segmented stators; 0 = continuous), m.
    pub segment_len_m: f64,
    /// Segment overlap for the K_f(x) hand-over, m.
    pub segment_overlap_m: f64,
}

/// Passive lateral guidance (PHYSICS §6): y and yaw by reluctance, bottom guide fin on
/// top-hung classes, UHMW side pads.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct GuidanceDesign {
    /// Bottom guide fin fitted (DOOR/WALL/HEAVY).
    pub fin_present: bool,
    /// Fin free clearance each side (±2 mm), m.
    pub fin_clearance_m: f64,
    /// UHMW pad/fin friction coefficient μ (0.15).
    pub pad_mu: f64,
    /// Side-pad free clearance each side, m.
    pub pad_clearance_m: f64,
    /// Sway damping c_sway (eddy/guide), N·s/m (plan §2b item 16; LEV-011 ζ ≥ 0.05).
    pub c_sway_n_s_per_m: f64,
}

/// Dissipation ledger, end zones, bumpers and the closing-edge seal (PHYSICS §8–§9).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct DampingDesign {
    /// Distributed eddy damping b_dist (also sets K_virtual max = 2·b_dist/T), N·s/m.
    pub b_dist_n_s_per_m: f64,
    /// Progressive end-zone length at each end, m (150 mm; SAF-014).
    pub ez_len_m: f64,
    /// End-zone peak damping c_max, N·s/m.
    pub ez_cmax_n_s_per_m: f64,
    /// Bumper contact stiffness, N/m.
    pub bumper_k_n_per_m: f64,
    /// Bumper Hunt–Crossley damping factor, s/m.
    pub bumper_hc_s_per_m: f64,
    /// Bumper energy rating, J (≥ ½·m_rated·v², F13; SAF-014).
    pub bumper_e_rated_j: f64,
    /// Closing-edge seal stiffness k_edge (≤ 10 N/mm, F6), N/m.
    pub k_edge_n_per_m: f64,
    /// Nominal rail eddy drag c_rail (scaled by ΣΦ²/ΣΦ0²) [UNCERTAIN, swept], N·s/m.
    pub c_rail_n_s_per_m: f64,
    /// Nominal rail hysteresis drag F_hyst [UNCERTAIN, swept], N.
    pub f_hyst_n: f64,
}

/// Mechanical x-retention latch for seismic SeatAndLatch (plan §4b).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct SeismicLatch {
    /// Latch fitted.
    pub enabled: bool,
    /// x-retention force; SeatAndLatch requires ≥ m_rated·a_design, N.
    pub x_retention_n: f64,
}

/// Landing, seat and lock hardware (PHYSICS §8, §15; F15, F17; plan §2b item 4).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct LandingDesign {
    /// Rest hardware.
    pub kind: LandingKind,
    /// Travel/landing roller rolling-resistance coefficient μ (0.003 everywhere,
    /// plan §2b item 19).
    pub rollers_mu: f64,
    /// Upper roller / stop rating, N (DRAWER ≥ 550 N, F2).
    pub upper_roller_rating_n: f64,
    /// Seat vertical clearance (captive in z both ways, ≈ 0.15 mm), m.
    pub seat_z_clearance_m: f64,
    /// Seat x compliance: spring push-stop travel (≥ 2 mm so push-to-open works while
    /// seated), m.
    pub seat_x_compliance_m: f64,
    /// Seat push-stop spring stiffness, N/m.
    pub seat_k_x_n_per_m: f64,
    /// Seat slot length the LSM drives out of after unseating, m.
    pub slot_len_m: f64,
    /// Seat entry ramp length (fail-down re-seat by pushing closed), m.
    pub ramp_len_m: f64,
    /// Seat entry ramp rise, m.
    pub ramp_rise_m: f64,
    /// Seismic x-latch.
    pub seismic_latch: SeismicLatch,
    /// Parking lock fitted (HEAVY; WALL land + lock).
    pub parking_lock: bool,
}

/// Rail geometry and material (PHYSICS §10).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct RailDesign {
    /// Closed-stop position, m.
    pub x_closed_m: f64,
    /// Preset travel from the closed stop, m (DRAWER 0.30 = 50 % extension, plan §2b 3).
    pub travel_m: f64,
    /// Support span for sag, m.
    pub span_m: f64,
    /// Sag deflection ratio L/δ (1150).
    pub deflection_ratio: f64,
    /// Joint spacing, m.
    pub joint_spacing_m: f64,
    /// Joint vertical step (0.1–0.3 mm), m.
    pub joint_step_m: f64,
    /// Lateral joint step, m.
    pub lateral_step_m: f64,
    /// Surface roughness rms (20 µm), m.
    pub roughness_rms_m: f64,
    /// Rail flange thickness (≥ B_g·w/B_sat,rail; LEV-015), m.
    pub flange_t_m: f64,
    /// Rail steel saturation flux density (1.5 T), T.
    pub b_sat_rail_t: f64,
    /// Laminated or SMC rail (the discrete gain design assumes it; plan §2b item 1).
    pub laminated: bool,
}

/// Air cavity and pressure disturbances (PHYSICS §11).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct AirDesign {
    /// Piston area of the moving front into the cabinet cavity (DRAWER 0.06), m².
    pub cavity_area_m2: f64,
    /// Cabinet cavity volume (DRAWER 0.03), m³.
    pub cavity_volume_m3: f64,
    /// Cavity leak area (DRAWER 40 cm²), m².
    pub leak_area_m2: f64,
    /// Leaf area exposed to smoke-control ΔP (DOOR), m².
    pub leaf_area_m2: f64,
}

/// Lumped thermal network: one RC node per coil plus one magnet node (PHYSICS §12).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct ThermalDesign {
    /// Coil-to-ambient thermal resistance, K/W.
    pub r_th_coil_k_per_w: f64,
    /// Coil heat capacity, J/K.
    pub c_th_coil_j_per_k: f64,
    /// Magnet-to-ambient thermal resistance, K/W.
    pub r_th_mag_k_per_w: f64,
    /// Magnet heat capacity, J/K.
    pub c_th_mag_j_per_k: f64,
    /// Ambient temperature, °C.
    pub t_ambient_c: f64,
    /// Coil insulation limit, °C.
    pub t_coil_max_c: f64,
    /// Magnet demagnetisation-safe limit (SH grade 80 °C proxy, ENV-004), °C.
    pub t_mag_max_c: f64,
}

/// Sensor calibration and noise specification (PHYSICS §13; critique G10).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct SensorCal {
    /// Gap linearisation g = c0 + c1·code + c2·code², coefficients in m, m/code, m/code².
    pub gap_poly_m: [f64; 3],
    /// Gap sensor in-band noise rms, m.
    pub gap_noise_m: f64,
    /// Redundant gap probe fitted (GapRaw::code_b valid).
    pub gap_redundant: bool,
    /// Coil current scale, A/code.
    pub coil_a_per_code: f64,
    /// Coil current noise rms, A.
    pub coil_noise_a: f64,
    /// LSM phase current scale, A/code.
    pub lsm_a_per_code: f64,
    /// Encoder resolution, nm/count (1 µm/count → 1000).
    pub nm_per_count: i32,
    /// Structure accelerometer scale (ADXL355 ±2 g, 20-bit: 3.9 µg/LSB), m/s²/LSB.
    pub accel_mps2_per_lsb: f64,
    /// Structure accelerometer noise density (22.5 µg/√Hz), m/s²/√Hz.
    pub accel_noise_mps2_per_rthz: f64,
    /// Structure accelerometer output data rate (500 Hz), Hz.
    pub accel_odr_hz: f64,
    /// Structure accelerometer mount (carriage is rejected, SEI-001).
    pub accel_mount: AccelMount,
    /// Carriage IMU fitted.
    pub imu_present: bool,
    /// Carriage IMU scale, m/s²/LSB.
    pub imu_mps2_per_lsb: f64,
    /// Carriage IMU noise rms (≈ 1 mg), m/s².
    pub imu_noise_mps2: f64,
    /// Rail floor accelerometer fitted.
    pub rail_accel_present: bool,
    /// Rail floor accelerometer scale, m/s²/LSB.
    pub rail_accel_mps2_per_lsb: f64,
    /// Rail floor accelerometer noise rms, m/s².
    pub rail_accel_noise_mps2: f64,
    /// Bus voltage scale, V/code.
    pub v_bus_v_per_code: f64,
    /// Hold-up voltage scale, V/code.
    pub v_hold_v_per_code: f64,
    /// Bus current scale, A/code.
    pub i_bus_a_per_code: f64,
    /// Temperature scale, °C/code.
    pub temp_c_per_code: f64,
    /// Temperature offset, °C.
    pub temp_offset_c: f64,
    /// Handle load-cell scale, N/code.
    pub handle_n_per_code: f64,
    /// Handle load-cell noise rms (σ 0.05 N), N.
    pub handle_noise_n: f64,
    /// Sensor transport latency, base ticks (1).
    pub latency_ticks: u32,
    /// Command/alert link latency (CAN 3 ms), s.
    pub link_latency_s: f64,
}

/// Bus, hold-up storage and power budgets (PHYSICS §14).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct PowerDesign {
    /// DC bus voltage (24 / 48 / 48 / 300 V; SELV < 60 V except HEAVY, PWR-005), V.
    pub v_bus_v: f64,
    /// Bus power limit, W.
    pub p_bus_max_w: f64,
    /// Brake-chopper threshold voltage, V.
    pub v_chopper_v: f64,
    /// PWM frequency (≥ 20 kHz, ENV-002), Hz.
    pub pwm_hz: f64,
    /// Hold-up supercap capacitance (10 F), F.
    pub supercap_c_f: f64,
    /// Supercap charged voltage (12 V), V.
    pub supercap_v_max_v: f64,
    /// Supercap minimum usable voltage, V.
    pub supercap_v_min_v: f64,
    /// Supercap ESR (10 mΩ), Ω.
    pub supercap_esr_ohm: f64,
    /// Hold-up converter efficiency (0.9).
    pub converter_eff: f64,
    /// Standby electronics power (dominates energy use; non-claim 13), W.
    pub p_electronics_w: f64,
    /// Hover power budget (DRAWER open ≤ 1.5 W, DOOR ≤ 3 W; PWR-002), W.
    pub p_hover_budget_w: f64,
    /// Sleep/seated power budget (≤ 0.3 / 0.5 W), W.
    pub p_sleep_budget_w: f64,
    /// Building UPS backs the mains input (admission to SmoothRideThrough, plan §4b).
    pub ups: bool,
}
