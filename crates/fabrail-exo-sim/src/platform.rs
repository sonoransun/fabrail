//! HAVEN-R 3-DOF planar platform: curved guideway with sagitta, tilt admission, copper damper, FPS catcher, moat (FAB-EXO-701..707). Owner: X4 (API frozen by W0a).
//!
//! EXOTIC §X9 (f64 truth, research tier; PHYSICS §21 for the isolation closed forms).
//! Rigid platform (x, y, θz) on a curved guideway of radius R (k = W/R,
//! T = 2π·√(R/g); HAVEN-R: T 6 s, R 8.95 m), pod groups represented by their closed-loop
//! equivalents (the planar LSM force `u` is an input), a copper eddy damper whose
//! conductivity falls with temperature σ(T) = σ0/(1 + 0.00393·ΔT) (so c = c20·σ/σ0 and
//! the copper heats by ΔT = E/(m_cu·c_p)), an FPS catcher for control loss (landing
//! through a PM-bias ramp) and a moat with a progressive buffer. Yaw uses the same
//! guideway and copper distribution as translation (radius of gyration √(J/m)); the
//! moat acts on the radial excursion of the centre.
//!
//! Vertical support is passive (air spring or QZS): a platform without passive vertical
//! support is rejected above 1 t (maglev vertical isolation is not proposed; FAB-EXO-705).
//!
//! Curved-guideway admission ([`guideway_admission`], FAB-EXO-111): pole sagitta L²/8R
//! ≤ 10 % of the gap or curved pole faces; pod tilt asin(S/R) needs articulated pods
//! (≤ 11°); attractive 2-D travel uses cross-stacked 1-D stages, and HTS (which pins the
//! lateral axis) is usable only in 1-D stages whose free axis is the travel axis.
//!
//! Occupied-platform limits ([`occupied_platform_report`], FAB-EXO-505) are analysis
//! outputs: S_D = g·S_1·T/(4π²·B) (ASCE 7-16 §17.5.3.3, B from Table 17.5-1) and
//! a_peak = ω²·S_D·√(1 + 4ζ²); MCE and near-fault values are reported, not bounded.

use fabrail_exo_safety::SuspensionKind;
use fabrail_params::derive::G0_MPS2;
use fabrail_types::fmath;

/// Copper resistivity temperature coefficient, 1/K (0.00393; FAB-EXO-118).
pub const CU_ALPHA_PER_K: f64 = 0.00393;

/// Copper specific heat, J/(kg·K) (385 at 20 °C).
pub const CU_CP_J_PER_KG_K: f64 = 385.0;

/// Copper density, kg/m³ (8960).
pub const CU_DENSITY_KG_PER_M3: f64 = 8960.0;

/// Eddy-damper coefficient per copper area at 20 °C, N·s/m per m² (exotic design §1.2
/// HAVEN-F: 1.48e5) \[UNCERTAIN\].
pub const CU_C_PER_M2_DEFAULT: f64 = 1.48e5;

/// Mass above which passive vertical support is mandatory, kg (FAB-EXO-705).
pub const PASSIVE_VERTICAL_REQUIRED_ABOVE_KG: f64 = 1000.0;

/// Maximum pole sagitta as a fraction of the gap for flat pole faces (FAB-EXO-111).
pub const SAGITTA_GAP_FRAC_MAX: f64 = 0.10;

/// Maximum pod tilt of articulated pods, degrees (FAB-EXO-111: up to 11°).
pub const TILT_ARTICULATED_MAX_DEG: f64 = 11.0;

/// Buffer force at full compression as a multiple of the weight (progressive buffer;
/// design choice).
pub const BUFFER_G_AT_FULL: f64 = 1.0;

/// Buffer damping ratio (design choice).
pub const BUFFER_ZETA: f64 = 0.3;

/// Moat wall stiffness as a multiple of the buffer stiffness (hard stop).
pub const WALL_K_FACTOR: f64 = 100.0;

/// Speed that regularises FPS friction, m/s (PHYSICS §8 reference 1 mm/s).
pub const FPS_V_REG_MPS: f64 = 1.0e-3;

/// HAVEN occupant peak-acceleration target, m/s² (0.1 g at DBE; FAB-EXO-505).
pub const HAVEN_A_PEAK_MAX_MPS2: f64 = 0.1 * G0_MPS2;

/// HAVEN landing speed limit, m/s (0.1 m/s; FAB-EXO-505, FAB-EXO-704).
pub const HAVEN_LANDING_V_MAX_MPS: f64 = 0.1;

/// Pendulum period of a curved guideway T = 2π·√(R/g), s (FAB-EXO-701).
pub fn guideway_period_s(r_m: f64) -> f64 {
    if r_m <= 0.0 {
        return 0.0;
    }
    2.0 * core::f64::consts::PI * fmath::sqrt(r_m / G0_MPS2)
}

/// Guideway radius for period `t_s`: R = g·T²/(4π²), m (T 6 s → 8.94 m).
pub fn guideway_radius_m(t_s: f64) -> f64 {
    G0_MPS2 * t_s * t_s / (4.0 * core::f64::consts::PI * core::f64::consts::PI)
}

/// Pole sagitta over a pole of length `l_m` on radius `r_m`: L²/(8R), m (FAB-EXO-111:
/// 7.9 mm at L 0.5 m, R 3.98 m; must stay ≤ 10 % of the gap).
pub fn sagitta_m(l_m: f64, r_m: f64) -> f64 {
    if r_m <= 0.0 {
        return f64::INFINITY;
    }
    l_m * l_m / (8.0 * r_m)
}

/// Copper conductivity at a temperature rise `d_t_k`: σ0/(1 + 0.00393·ΔT), S/m.
pub fn copper_sigma(sigma0_s_per_m: f64, d_t_k: f64) -> f64 {
    sigma0_s_per_m / (1.0 + CU_ALPHA_PER_K * d_t_k)
}

/// Pod tilt at stroke `stroke_m` on radius `r_m`: asin(S/R), rad (FAB-EXO-111: 10.8° for
/// HAVEN at 0.75 m on 3.98 m; 7.1° for SENTINEL at 1.5 m on 12.2 m).
pub fn tilt_at_stroke_rad(stroke_m: f64, r_m: f64) -> f64 {
    if r_m <= 0.0 {
        return core::f64::consts::FRAC_PI_2;
    }
    fmath::asin((stroke_m.abs() / r_m).min(1.0))
}

/// ASCE 7-16 Table 17.5-1 damping coefficient B for effective damping ζ (linear
/// interpolation; 0.8 at ≤ 2 %, 1.0 at 5 %, 1.2 at 10 %, 1.5 at 20 %, 1.7 at 30 %, 1.9 at
/// 40 %, 2.0 at ≥ 50 %).
pub fn asce_damping_coefficient(zeta: f64) -> f64 {
    const TABLE: [(f64, f64); 7] = [
        (0.02, 0.8),
        (0.05, 1.0),
        (0.10, 1.2),
        (0.20, 1.5),
        (0.30, 1.7),
        (0.40, 1.9),
        (0.50, 2.0),
    ];
    if zeta <= TABLE[0].0 {
        return TABLE[0].1;
    }
    for w in TABLE.windows(2) {
        let (z0, b0) = w[0];
        let (z1, b1) = w[1];
        if zeta <= z1 {
            return b0 + (b1 - b0) * (zeta - z0) / (z1 - z0);
        }
    }
    TABLE[6].1
}

/// Isolator design displacement S_D = g·S_1·T/(4π²·B), m (ASCE 7-16 eq. 17.5-1; S_1 in
/// g: S_D1 at DBE, S_M1 at MCE).
pub fn design_displacement_m(s1_g: f64, t_s: f64, zeta: f64) -> f64 {
    G0_MPS2 * s1_g * t_s
        / (4.0 * core::f64::consts::PI * core::f64::consts::PI * asce_damping_coefficient(zeta))
}

/// Peak platform acceleration of a linear isolator a = ω²·S_D·√(1 + 4ζ²), m/s²
/// (FAB-EXO-110, 505).
pub fn isolated_peak_accel_mps2(t_s: f64, s_d_m: f64, zeta: f64) -> f64 {
    if t_s <= 0.0 {
        return f64::INFINITY;
    }
    let w = 2.0 * core::f64::consts::PI / t_s;
    w * w * s_d_m * fmath::sqrt(1.0 + 4.0 * zeta * zeta)
}

/// Occupied-platform analysis (FAB-EXO-505): values are reported; only the DBE
/// acceleration and the landing speed are held to the occupant targets.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OccupiedPlatformReport {
    /// DBE design displacement, m.
    pub dbe_s_d_m: f64,
    /// DBE peak platform acceleration, m/s².
    pub dbe_a_peak_mps2: f64,
    /// MCE displacement (reported), m.
    pub mce_s_m_m: f64,
    /// MCE peak acceleration (reported), m/s².
    pub mce_a_peak_mps2: f64,
    /// Near-fault peak displacement from a simulated pulse (reported), m.
    pub near_fault_disp_m: f64,
    /// Near-fault peak acceleration from a simulated pulse (reported), m/s².
    pub near_fault_a_peak_mps2: f64,
    /// Landing touchdown speed, m/s.
    pub landing_v_mps: f64,
    /// DBE acceleration ≤ 0.1 g.
    pub dbe_ok: bool,
    /// Landing ≤ 0.1 m/s.
    pub landing_ok: bool,
}

/// Builds the occupied-platform report for a site (S_D1, S_M1 in g), period `t_s`, damping
/// `zeta`, a near-fault result (displacement, acceleration) and a landing speed.
pub fn occupied_platform_report(
    sd1_g: f64,
    sm1_g: f64,
    t_s: f64,
    zeta: f64,
    near_fault: (f64, f64),
    landing_v_mps: f64,
) -> OccupiedPlatformReport {
    let s_d = design_displacement_m(sd1_g, t_s, zeta);
    let s_m = design_displacement_m(sm1_g, t_s, zeta);
    let a_d = isolated_peak_accel_mps2(t_s, s_d, zeta);
    OccupiedPlatformReport {
        dbe_s_d_m: s_d,
        dbe_a_peak_mps2: a_d,
        mce_s_m_m: s_m,
        mce_a_peak_mps2: isolated_peak_accel_mps2(t_s, s_m, zeta),
        near_fault_disp_m: near_fault.0,
        near_fault_a_peak_mps2: near_fault.1,
        landing_v_mps,
        dbe_ok: a_d <= HAVEN_A_PEAK_MAX_MPS2,
        landing_ok: landing_v_mps <= HAVEN_LANDING_V_MAX_MPS,
    }
}

// ---- Curved-guideway admission (FAB-EXO-111) ---------------------------------------------

/// Travel layout of a planar stage.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum GuidewayLayout {
    /// Two cross-stacked 1-D stages (the attractive-EMS layout).
    #[default]
    CrossStacked1D,
    /// One planar 2-D stage.
    Planar2D,
}

/// Inputs of [`guideway_admission`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GuidewayInputs {
    /// Pole length along the curve, m.
    pub pole_l_m: f64,
    /// Guideway radius, m.
    pub r_m: f64,
    /// Magnetic gap, m.
    pub gap_m: f64,
    /// Design stroke, m.
    pub stroke_m: f64,
    /// Pole faces machined to the guideway curvature.
    pub curved_faces: bool,
    /// Pods articulated in tilt.
    pub articulated: bool,
    /// Travel layout.
    pub layout: GuidewayLayout,
    /// Suspension kind of the pods.
    pub suspension: SuspensionKind,
}

/// A passed admission, with the figures it checked.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GuidewayAdmission {
    /// Pole sagitta, m.
    pub sagitta_m: f64,
    /// Sagitta as a fraction of the gap.
    pub sagitta_gap_frac: f64,
    /// Pod tilt at full stroke, degrees.
    pub tilt_deg: f64,
}

/// Why a curved-guideway configuration was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GuidewayReject {
    /// Non-positive length, radius, gap or stroke.
    InvalidField,
    /// Flat pole faces with a sagitta above 10 % of the gap.
    SagittaExceedsGap,
    /// Rigid pods whose tilt changes the gap across the pole by more than 10 %.
    TiltNeedsArticulation,
    /// Tilt above the 11° articulation range.
    TiltExceedsArticulation,
    /// Attractive EMS on a planar 2-D stage (use cross-stacked 1-D stages).
    PlanarAttractive,
    /// HTS outside a 1-D stage (it pins the lateral axis).
    HtsNotOneD,
}

/// Curved-guideway admission (FAB-EXO-111; exotic critique P1-10).
pub fn guideway_admission(i: &GuidewayInputs) -> Result<GuidewayAdmission, GuidewayReject> {
    let all_positive = i.pole_l_m > 0.0 && i.r_m > 0.0 && i.gap_m > 0.0 && i.stroke_m >= 0.0;
    if !all_positive {
        return Err(GuidewayReject::InvalidField);
    }
    let attractive = matches!(
        i.suspension,
        SuspensionKind::HybridEms | SuspensionKind::PureEms | SuspensionKind::HybridHtsPlusEms
    );
    let hts = matches!(
        i.suspension,
        SuspensionKind::HtsBulk | SuspensionKind::HybridHtsPlusEms
    );
    if i.layout == GuidewayLayout::Planar2D {
        if hts {
            return Err(GuidewayReject::HtsNotOneD);
        }
        if attractive {
            return Err(GuidewayReject::PlanarAttractive);
        }
    }
    let s = sagitta_m(i.pole_l_m, i.r_m);
    let frac = s / i.gap_m;
    if frac > SAGITTA_GAP_FRAC_MAX && !i.curved_faces {
        return Err(GuidewayReject::SagittaExceedsGap);
    }
    let tilt = tilt_at_stroke_rad(i.stroke_m, i.r_m);
    let tilt_deg = tilt * 180.0 / core::f64::consts::PI;
    if tilt_deg > TILT_ARTICULATED_MAX_DEG {
        return Err(GuidewayReject::TiltExceedsArticulation);
    }
    // A rigid pod tilted by θ opens the gap at one pole end by (L/2)·sin θ.
    if !i.articulated && 0.5 * i.pole_l_m * fmath::sin(tilt) > SAGITTA_GAP_FRAC_MAX * i.gap_m {
        return Err(GuidewayReject::TiltNeedsArticulation);
    }
    Ok(GuidewayAdmission {
        sagitta_m: s,
        sagitta_gap_frac: frac,
        tilt_deg,
    })
}

// ---- Friction pendulum (FPS) baseline and catcher ----------------------------------------

/// Friction pendulum system: restoring W/R plus Coulomb friction μ·W (the non-magnetic
/// isolation baseline, FAB-EXO-209, and the HAVEN control-loss catcher).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FrictionPendulum {
    /// Surface radius, m.
    pub r_m: f64,
    /// Friction coefficient.
    pub mu: f64,
}

impl FrictionPendulum {
    /// Pendulum period 2π·√(R/g), s.
    pub fn period_s(&self) -> f64 {
        guideway_period_s(self.r_m)
    }

    /// Activation (breakaway) ground acceleration μ·g, m/s².
    pub fn activation_mps2(&self) -> f64 {
        self.mu * G0_MPS2
    }

    /// Planar force on a weight `w_n` at offset `r` and velocity `v` (regularised
    /// friction), N.
    pub fn force_n(&self, w_n: f64, r: [f64; 2], v: [f64; 2]) -> [f64; 2] {
        let k = if self.r_m > 0.0 { w_n / self.r_m } else { 0.0 };
        let speed = fmath::hypot(v[0], v[1]);
        let f_fric = self.mu * w_n * fmath::tanh(speed / FPS_V_REG_MPS);
        let (ux, uy) = if speed > 0.0 {
            (v[0] / speed, v[1] / speed)
        } else {
            (0.0, 0.0)
        };
        [-k * r[0] - f_fric * ux, -k * r[1] - f_fric * uy]
    }
}

// ---- Platform ----------------------------------------------------------------------------

/// Passive vertical support type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum PassiveVertical {
    /// None (rejected above 1 t).
    #[default]
    None,
    /// Air springs.
    AirSpring,
    /// Quasi-zero-stiffness elements.
    Qzs,
}

/// Platform configuration.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlatformCfg {
    /// Platform mass, kg.
    pub m_kg: f64,
    /// Yaw inertia, kg·m².
    pub j_z_kgm2: f64,
    /// Guideway radius, m.
    pub r_guideway_m: f64,
    /// Design stroke S_D, m.
    pub stroke_m: f64,
    /// Moat clearance, m.
    pub moat_m: f64,
    /// Progressive buffer zone, m.
    pub buffer_zone_m: f64,
    /// Copper damper coefficient per area at 20 °C, N·s/m per m².
    pub c_cu_per_m2: f64,
    /// Copper area, m².
    pub cu_area_m2: f64,
    /// Copper mass, kg.
    pub cu_mass_kg: f64,
    /// Copper temperature limit rise, K.
    pub cu_dt_max_k: f64,
    /// FPS catcher radius, m.
    pub fps_r_m: f64,
    /// FPS friction coefficient.
    pub fps_mu: f64,
    /// Passive vertical support.
    pub vertical: PassiveVertical,
}

impl PlatformCfg {
    /// The HAVEN-R reference platform (EXOTIC §1): 50 t, 10 m × 10 m slab (J = m·200/12),
    /// T 6 s (R 8.945 m), design stroke S_D at MCE 0.9 m, moat 1.2 m with a 0.3 m
    /// progressive buffer from 0.9 m, copper sized for ζ 0.2 at 1.48e5 N·s/m per m² (20 mm
    /// plate), ΔT limit 100 K, FPS catcher on the guideway curvature with μ 0.05, air
    /// springs \[UNCERTAIN design picks\].
    pub fn haven_r() -> PlatformCfg {
        let m = 50.0e3;
        let t = 6.0;
        let w = 2.0 * core::f64::consts::PI / t;
        let zeta = 0.2;
        let c = 2.0 * zeta * m * w;
        let area = c / CU_C_PER_M2_DEFAULT;
        PlatformCfg {
            m_kg: m,
            j_z_kgm2: m * 200.0 / 12.0,
            r_guideway_m: guideway_radius_m(t),
            stroke_m: 0.9,
            moat_m: 1.2,
            buffer_zone_m: 0.3,
            c_cu_per_m2: CU_C_PER_M2_DEFAULT,
            cu_area_m2: area,
            cu_mass_kg: area * 0.020 * CU_DENSITY_KG_PER_M3,
            cu_dt_max_k: 100.0,
            fps_r_m: guideway_radius_m(t),
            fps_mu: 0.05,
            vertical: PassiveVertical::AirSpring,
        }
    }

    /// Passive copper damping ratio at 20 °C, c/(2·m·ω_iso).
    pub fn copper_zeta(&self) -> f64 {
        let t = guideway_period_s(self.r_guideway_m);
        if !(t > 0.0 && self.m_kg > 0.0) {
            return 0.0;
        }
        let w = 2.0 * core::f64::consts::PI / t;
        self.c_cu_per_m2 * self.cu_area_m2 / (2.0 * self.m_kg * w)
    }
}

/// Why a platform configuration was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PlatformError {
    /// No passive vertical support above 1 t.
    NoPassiveVertical,
    /// Non-positive mass, radius or stroke.
    InvalidField,
    /// Design stroke larger than the moat.
    StrokeExceedsMoat,
}

/// Platform state [x, y, θz, ẋ, ẏ, θ̇z] relative to the ground, plus copper ΔT.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlatformState {
    /// Pose and rates.
    pub q: [f64; 6],
    /// Copper temperature rise, K.
    pub cu_dt_k: f64,
    /// Landed on the FPS catcher.
    pub landed: bool,
}

/// PM-bias-ramp landing parameters (FAB-EXO-704).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LandingRamp {
    /// Levitation gap to close, m.
    pub gap_m: f64,
    /// Ramp duration, s.
    pub t_ramp_s: f64,
    /// Closed-loop pod bandwidth, rad/s.
    pub omega_cl_rad_s: f64,
    /// Closed-loop pod damping ratio.
    pub zeta_cl: f64,
}

impl Default for LandingRamp {
    /// 12 mm gap over 2 s (smooth-step, peak 9 mm/s), pods at 5 Hz, ζ 0.7 (closed-loop
    /// equivalents; design choice).
    fn default() -> LandingRamp {
        LandingRamp {
            gap_m: 0.012,
            t_ramp_s: 2.0,
            omega_cl_rad_s: 2.0 * core::f64::consts::PI * 5.0,
            zeta_cl: 0.7,
        }
    }
}

impl LandingRamp {
    /// Reference height above touchdown and its rate at time `t_s` into the ramp
    /// (smooth-step 3τ² − 2τ³).
    pub fn reference(&self, t_s: f64) -> (f64, f64) {
        if self.t_ramp_s <= 0.0 {
            return (0.0, 0.0);
        }
        let tau = (t_s / self.t_ramp_s).clamp(0.0, 1.0);
        let s = tau * tau * (3.0 - 2.0 * tau);
        let ds = if (0.0..1.0).contains(&tau) {
            6.0 * tau * (1.0 - tau) / self.t_ramp_s
        } else {
            0.0
        };
        (self.gap_m * (1.0 - s), -self.gap_m * ds)
    }

    /// Touchdown speed of an uncontrolled drop from the gap √(2·g·gap), m/s (0.44–0.54 m/s
    /// for 10–15 mm).
    pub fn free_drop_speed_mps(&self) -> f64 {
        fmath::sqrt(2.0 * G0_MPS2 * self.gap_m.max(0.0))
    }
}

/// Vertical landing state (private).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Vertical {
    /// Height above touchdown, m.
    h_m: f64,
    /// Rate, m/s.
    v_mps: f64,
    /// Time since the landing began, s.
    t_s: f64,
    /// Touchdown speed, m/s.
    touchdown_mps: Option<f64>,
}

/// Run diagnostics (private).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Diag {
    max_radial_m: f64,
    buffer_contact: bool,
    buffer_peak_stored_j: f64,
    cu_energy_j: f64,
    peak_abs_accel_mps2: f64,
    last_abs_accel: [f64; 2],
}

/// The HAVEN-R 3-DOF planar platform.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HavenR3Dof {
    cfg: PlatformCfg,
    /// State.
    pub state: PlatformState,
    ramp: LandingRamp,
    control_lost: bool,
    vert: Vertical,
    diag: Diag,
}

impl HavenR3Dof {
    /// Validates `cfg` (FAB-EXO-705 and the stroke/moat rule).
    pub fn new(cfg: &PlatformCfg) -> Result<HavenR3Dof, PlatformError> {
        if !(cfg.m_kg > 0.0 && cfg.r_guideway_m > 0.0 && cfg.stroke_m > 0.0) {
            return Err(PlatformError::InvalidField);
        }
        if cfg.m_kg > PASSIVE_VERTICAL_REQUIRED_ABOVE_KG && cfg.vertical == PassiveVertical::None {
            return Err(PlatformError::NoPassiveVertical);
        }
        if cfg.stroke_m > cfg.moat_m {
            return Err(PlatformError::StrokeExceedsMoat);
        }
        let ramp = LandingRamp::default();
        Ok(HavenR3Dof {
            cfg: *cfg,
            state: PlatformState::default(),
            ramp,
            control_lost: false,
            vert: Vertical {
                h_m: ramp.gap_m,
                ..Vertical::default()
            },
            diag: Diag::default(),
        })
    }

    /// The configuration.
    pub fn cfg(&self) -> &PlatformCfg {
        &self.cfg
    }

    /// Replaces the landing ramp (before control loss).
    pub fn set_landing_ramp(&mut self, ramp: LandingRamp) {
        self.ramp = ramp;
        if !self.control_lost {
            self.vert.h_m = ramp.gap_m;
        }
    }

    /// Guideway (or FPS) planar stiffness W/R, N/m.
    fn k_restore(&self) -> f64 {
        let r = if self.state.landed {
            self.cfg.fps_r_m
        } else {
            self.cfg.r_guideway_m
        };
        if r > 0.0 {
            self.cfg.m_kg * G0_MPS2 / r
        } else {
            0.0
        }
    }

    /// Copper damping coefficient at the present copper temperature,
    /// c = c20·σ(T)/σ0 = c20/(1 + 0.00393·ΔT), N·s/m (FAB-EXO-118).
    pub fn copper_c_n_s_per_m(&self) -> f64 {
        copper_sigma(
            self.cfg.c_cu_per_m2 * self.cfg.cu_area_m2,
            self.state.cu_dt_k,
        )
    }

    /// Radius of gyration squared J/m, m².
    fn r_gyr2(&self) -> f64 {
        if self.cfg.m_kg > 0.0 {
            self.cfg.j_z_kgm2 / self.cfg.m_kg
        } else {
            0.0
        }
    }

    /// Buffer and wall force (radial, on the platform) at offset `r` and velocity `v`, N.
    fn moat_force(&self, r: [f64; 2], v: [f64; 2]) -> [f64; 2] {
        let c = &self.cfg;
        let rad = fmath::hypot(r[0], r[1]);
        let onset = (c.moat_m - c.buffer_zone_m).max(0.0);
        if rad <= onset || rad <= 0.0 {
            return [0.0, 0.0];
        }
        let n = [r[0] / rad, r[1] / rad];
        let pen = rad - onset;
        let zone = c.buffer_zone_m.max(1.0e-3);
        // Progressive: linear plus cubic, W·BUFFER_G_AT_FULL at full compression.
        let k_lin = 0.5 * BUFFER_G_AT_FULL * c.m_kg * G0_MPS2 / zone;
        let mut f = k_lin * pen + k_lin * pen * pen * pen / (zone * zone);
        let v_n = v[0] * n[0] + v[1] * n[1];
        if v_n > 0.0 {
            f += 2.0 * BUFFER_ZETA * fmath::sqrt(k_lin * c.m_kg) * v_n;
        }
        if rad > c.moat_m {
            f += WALL_K_FACTOR * k_lin * (rad - c.moat_m);
        }
        [-f * n[0], -f * n[1]]
    }

    /// Stored buffer energy at radial offset `rad`, J.
    fn buffer_stored_j(&self, rad: f64) -> f64 {
        let c = &self.cfg;
        let onset = (c.moat_m - c.buffer_zone_m).max(0.0);
        let pen = (rad - onset).max(0.0);
        let zone = c.buffer_zone_m.max(1.0e-3);
        let k_lin = 0.5 * BUFFER_G_AT_FULL * c.m_kg * G0_MPS2 / zone;
        let wall = (rad - c.moat_m).max(0.0);
        0.5 * k_lin * pen * pen
            + 0.25 * k_lin * pen * pen * pen * pen / (zone * zone)
            + 0.5 * WALL_K_FACTOR * k_lin * wall * wall
    }

    /// State derivative (relative accelerations) and the absolute planar acceleration.
    fn derivative(&self, q: &[f64; 6], a_g: [f64; 2], u: [f64; 3]) -> ([f64; 6], [f64; 2]) {
        let c = &self.cfg;
        let m = c.m_kg;
        let jz = c.j_z_kgm2.max(f64::MIN_POSITIVE);
        let k = self.k_restore();
        let cc = self.copper_c_n_s_per_m();
        let rg2 = self.r_gyr2();
        let (r, v) = ([q[0], q[1]], [q[3], q[4]]);
        let u_eff = if self.control_lost { [0.0; 3] } else { u };
        let moat = self.moat_force(r, v);
        let mut f = [
            -k * r[0] - cc * v[0] + moat[0] + u_eff[0],
            -k * r[1] - cc * v[1] + moat[1] + u_eff[1],
        ];
        if self.state.landed {
            let fps = FrictionPendulum {
                r_m: c.fps_r_m,
                mu: c.fps_mu,
            };
            // The FPS supplies the restoring term itself; remove the guideway term.
            let ff = fps.force_n(m * G0_MPS2, [0.0, 0.0], v);
            f[0] += ff[0];
            f[1] += ff[1];
        }
        let moment = -k * rg2 * q[2] - cc * rg2 * q[5] + u_eff[2];
        let ax = f[0] / m - a_g[0];
        let ay = f[1] / m - a_g[1];
        (
            [q[3], q[4], q[5], ax, ay, moment / jz],
            [ax + a_g[0], ay + a_g[1]],
        )
    }

    /// Landed and slow (< 1 mm/s) with the non-friction planar force within μ·W: the FPS
    /// holds (Karnopp stick; the regularised friction alone would creep).
    fn fps_sticks(&self, q: &[f64; 6], a_g: [f64; 2]) -> bool {
        if !self.state.landed {
            return false;
        }
        let c = &self.cfg;
        let speed = fmath::hypot(q[3], q[4]);
        if speed >= FPS_V_REG_MPS {
            return false;
        }
        let k = self.k_restore();
        let cc = self.copper_c_n_s_per_m();
        let moat = self.moat_force([q[0], q[1]], [q[3], q[4]]);
        let fx = -k * q[0] - cc * q[3] + moat[0] - c.m_kg * a_g[0];
        let fy = -k * q[1] - cc * q[4] + moat[1] - c.m_kg * a_g[1];
        fmath::hypot(fx, fy) <= c.fps_mu * c.m_kg * G0_MPS2
    }

    /// Copper dissipation power at velocity state `q`, W.
    fn copper_power_w(&self, q: &[f64; 6]) -> f64 {
        let cc = self.copper_c_n_s_per_m();
        cc * (q[3] * q[3] + q[4] * q[4] + self.r_gyr2() * q[5] * q[5])
    }

    /// Advances by `dt_s` under ground acceleration (x, y) and LSM forces (x, y, moment).
    /// The LSM input is ignored after [`HavenR3Dof::lose_control`] (RK4 with inputs and
    /// copper temperature held over the step; PHYSICS §24).
    pub fn step(&mut self, a_ground_mps2: [f64; 2], u: [f64; 3], dt_s: f64) {
        if !(dt_s > 0.0 && self.cfg.m_kg > 0.0) {
            return;
        }
        let q = self.state.q;
        let add = |a: &[f64; 6], b: &[f64; 6], h: f64| -> [f64; 6] {
            let mut o = [0.0; 6];
            for i in 0..6 {
                o[i] = a[i] + h * b[i];
            }
            o
        };
        let p_cu0 = self.copper_power_w(&q);
        let qn = if self.fps_sticks(&q, a_ground_mps2) {
            // Karnopp stick on the FPS: the platform moves with the ground.
            [q[0], q[1], q[2], 0.0, 0.0, 0.0]
        } else {
            let (k1, _) = self.derivative(&q, a_ground_mps2, u);
            let (k2, _) = self.derivative(&add(&q, &k1, 0.5 * dt_s), a_ground_mps2, u);
            let (k3, _) = self.derivative(&add(&q, &k2, 0.5 * dt_s), a_ground_mps2, u);
            let (k4, _) = self.derivative(&add(&q, &k3, dt_s), a_ground_mps2, u);
            let mut qn = [0.0; 6];
            for i in 0..6 {
                qn[i] = q[i] + dt_s / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
            qn
        };
        self.state.q = qn;
        let rad = fmath::hypot(qn[0], qn[1]);
        let e_buf1 = self.buffer_stored_j(rad);
        // Copper heating: trapezoid of the dissipated power, ΔT = E/(m_cu·c_p).
        let p_cu1 = self.copper_power_w(&qn);
        let e_cu = 0.5 * (p_cu0 + p_cu1) * dt_s;
        self.diag.cu_energy_j += e_cu;
        if self.cfg.cu_mass_kg > 0.0 {
            self.state.cu_dt_k += e_cu / (self.cfg.cu_mass_kg * CU_CP_J_PER_KG_K);
        }
        // Diagnostics.
        let d = &mut self.diag;
        d.max_radial_m = d.max_radial_m.max(rad);
        let onset = (self.cfg.moat_m - self.cfg.buffer_zone_m).max(0.0);
        if rad > onset {
            d.buffer_contact = true;
        }
        d.buffer_peak_stored_j = d.buffer_peak_stored_j.max(e_buf1);
        let acc1 = if self.fps_sticks(&qn, a_ground_mps2) {
            a_ground_mps2
        } else {
            self.derivative(&qn, a_ground_mps2, u).1
        };
        self.diag.last_abs_accel = acc1;
        self.diag.peak_abs_accel_mps2 = self
            .diag
            .peak_abs_accel_mps2
            .max(fmath::hypot(acc1[0], acc1[1]));
        // Vertical PM-bias-ramp landing after control loss.
        if self.control_lost && !self.state.landed {
            self.step_landing(dt_s);
        }
    }

    /// Closed-loop pods tracking the landing ramp (FAB-EXO-704).
    fn step_landing(&mut self, dt_s: f64) {
        let r = self.ramp;
        let w = r.omega_cl_rad_s;
        let z = r.zeta_cl;
        // Sub-step so the pod loop is resolved (ω·h ≤ 0.2).
        let n = fmath::ceil(dt_s * w / 0.2).max(1.0) as u32;
        let h = dt_s / f64::from(n);
        for _ in 0..n {
            let (href, vref) = r.reference(self.vert.t_s);
            let a = w * w * (href - self.vert.h_m) + 2.0 * z * w * (vref - self.vert.v_mps);
            self.vert.v_mps += a * h;
            self.vert.h_m += self.vert.v_mps * h;
            self.vert.t_s += h;
            if self.vert.h_m <= 0.0 {
                self.vert.touchdown_mps = Some(fmath::abs(self.vert.v_mps));
                self.vert.h_m = 0.0;
                self.vert.v_mps = 0.0;
                self.state.landed = true;
                break;
            }
            if self.vert.t_s > r.t_ramp_s + 10.0 / w.max(1.0e-3) {
                // Settled on the reference (h ≈ 0) without crossing: touch down now.
                self.vert.touchdown_mps = Some(fmath::abs(self.vert.v_mps));
                self.vert.h_m = 0.0;
                self.state.landed = true;
                break;
            }
        }
    }

    /// Control loss: the LSM force stops and the pods land the platform on the FPS
    /// catcher through the PM-bias ramp (FAB-EXO-704).
    pub fn lose_control(&mut self) {
        if !self.control_lost {
            self.control_lost = true;
            self.vert = Vertical {
                h_m: self.ramp.gap_m,
                ..Vertical::default()
            };
        }
    }

    /// Control has been lost.
    pub fn control_lost(&self) -> bool {
        self.control_lost
    }

    /// Touchdown speed of the landing, once landed, m/s.
    pub fn touchdown_speed_mps(&self) -> Option<f64> {
        self.vert.touchdown_mps
    }

    /// Largest radial excursion so far, m.
    pub fn max_radial_m(&self) -> f64 {
        self.diag.max_radial_m
    }

    /// The buffer has been touched.
    pub fn buffer_contact(&self) -> bool {
        self.diag.buffer_contact
    }

    /// Largest energy stored in the buffer so far (the bumper energy to quote), J.
    pub fn buffer_peak_energy_j(&self) -> f64 {
        self.diag.buffer_peak_stored_j
    }

    /// Energy dissipated in the copper so far, J.
    pub fn copper_energy_j(&self) -> f64 {
        self.diag.cu_energy_j
    }

    /// The copper ΔT is within its limit.
    pub fn copper_within_limit(&self) -> bool {
        self.state.cu_dt_k <= self.cfg.cu_dt_max_k
    }

    /// Absolute planar platform acceleration after the last step, m/s².
    pub fn abs_accel_mps2(&self) -> [f64; 2] {
        self.diag.last_abs_accel
    }

    /// Largest absolute planar acceleration magnitude so far, m/s².
    pub fn peak_abs_accel_mps2(&self) -> f64 {
        self.diag.peak_abs_accel_mps2
    }

    /// Clears the peak diagnostics (e.g. after a settling window).
    pub fn reset_peaks(&mut self) {
        self.diag.peak_abs_accel_mps2 = 0.0;
        self.diag.max_radial_m = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ground::{NearFaultPulse, m9_long_duration_series};
    use fabrail_exo::isolation::{transmissibility_passive, transmissibility_skyhook};

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    fn deg(rad: f64) -> f64 {
        rad * 180.0 / core::f64::consts::PI
    }

    // req: FAB-EXO-111
    #[test]
    fn guideway_sagitta_tilt_admission() {
        // Sagitta L²/8R: 7.9 mm at L 0.5 m, R 3.98 m; 10.3 mm at 1 m, 12.2 m (exotic critique).
        assert!(
            rel(sagitta_m(0.5, 3.98), 7.9e-3) < 0.01,
            "7.9 mm (FAB-EXO-111)"
        );
        assert!(rel(sagitta_m(1.0, 12.2), 10.3e-3) < 0.01);
        // Pod tilt at full stroke: 10.8° (HAVEN, 0.75 m on 3.98 m), 7.1° (SENTINEL).
        assert!((deg(tilt_at_stroke_rad(0.75, 3.98)) - 10.8).abs() < 0.1);
        assert!((deg(tilt_at_stroke_rad(1.5, 12.2)) - 7.1).abs() < 0.1);
        let base = GuidewayInputs {
            pole_l_m: 0.5,
            r_m: 3.98,
            gap_m: 0.012,
            stroke_m: 0.75,
            curved_faces: false,
            articulated: true,
            layout: GuidewayLayout::CrossStacked1D,
            suspension: SuspensionKind::HybridEms,
        };
        // Flat faces: 7.9 mm ≫ 10 % of a 12 mm gap.
        assert_eq!(
            guideway_admission(&base),
            Err(GuidewayReject::SagittaExceedsGap)
        );
        let curved = GuidewayInputs {
            curved_faces: true,
            ..base
        };
        let ok = guideway_admission(&curved).expect("curved faces + articulated pods");
        assert!((ok.tilt_deg - 10.86).abs() < 0.01 && ok.sagitta_gap_frac > 0.5);
        assert_eq!(
            guideway_admission(&GuidewayInputs {
                articulated: false,
                ..curved
            }),
            Err(GuidewayReject::TiltNeedsArticulation)
        );
        assert_eq!(
            guideway_admission(&GuidewayInputs {
                stroke_m: 0.8,
                ..curved
            }),
            Err(GuidewayReject::TiltExceedsArticulation),
            "tilt above 11°"
        );
        // Attractive 2-D travel uses cross-stacked 1-D stages; HTS only in 1-D stages.
        let planar = GuidewayInputs {
            layout: GuidewayLayout::Planar2D,
            ..curved
        };
        assert_eq!(
            guideway_admission(&planar),
            Err(GuidewayReject::PlanarAttractive)
        );
        assert_eq!(
            guideway_admission(&GuidewayInputs {
                suspension: SuspensionKind::HtsBulk,
                ..planar
            }),
            Err(GuidewayReject::HtsNotOneD)
        );
        assert!(
            guideway_admission(&GuidewayInputs {
                suspension: SuspensionKind::HtsBulk,
                ..curved
            })
            .is_ok()
        );
        assert!(
            guideway_admission(&GuidewayInputs {
                suspension: SuspensionKind::AirPad,
                ..planar
            })
            .is_ok()
        );
        assert_eq!(
            guideway_admission(&GuidewayInputs { r_m: 0.0, ..base }),
            Err(GuidewayReject::InvalidField)
        );
        // HAVEN-R at T 6 s (R 8.945 m, S 0.9 m) still needs curved faces; tilt 5.8°.
        let haven = PlatformCfg::haven_r();
        let hr = GuidewayInputs {
            r_m: haven.r_guideway_m,
            stroke_m: haven.stroke_m,
            ..curved
        };
        let a = guideway_admission(&hr).expect("HAVEN-R admitted with curved faces");
        println!(
            "HAVEN-R guideway: sagitta {:.2} mm ({:.0} % of gap), tilt {:.2}°",
            a.sagitta_m * 1e3,
            a.sagitta_gap_frac * 100.0,
            a.tilt_deg
        );
        assert!(
            guideway_admission(&GuidewayInputs {
                curved_faces: false,
                ..hr
            })
            .is_err()
        );
    }

    // req: FAB-EXO-118
    #[test]
    fn copper_sigma_temperature() {
        // σ(T) = σ0/(1 + 0.00393·ΔT) (E).
        let s0 = 5.96e7;
        assert_eq!(copper_sigma(s0, 0.0), s0);
        assert!(rel(copper_sigma(s0, 100.0), s0 / 1.393) < 1e-12);
        // Coupled into the damping coefficient and into the dynamics.
        let cfg = PlatformCfg::haven_r();
        let mut p = HavenR3Dof::new(&cfg).expect("valid HAVEN-R");
        let c20 = p.copper_c_n_s_per_m();
        assert!(rel(c20, cfg.c_cu_per_m2 * cfg.cu_area_m2) < 1e-12);
        let q = [0.0, 0.0, 0.0, 0.3, 0.0, 0.0];
        let a20 = p.derivative(&q, [0.0; 2], [0.0; 3]).0[3];
        p.state.cu_dt_k = 100.0;
        let c100 = p.copper_c_n_s_per_m();
        let a100 = p.derivative(&q, [0.0; 2], [0.0; 3]).0[3];
        assert!(
            rel(c100 / c20, 1.0 / 1.393) < 1e-12,
            "c ∝ σ(T) (FAB-EXO-118)"
        );
        assert!(
            rel(a100 / a20, 1.0 / 1.393) < 1e-12,
            "the damper force follows σ(T)"
        );
    }

    /// Steady-state |a_abs|/|a_g| of the platform under harmonic x ground motion at `f_hz`;
    /// `skyhook` = Some(c) removes the copper and applies u = −c·v_abs.
    fn transmissibility_sim(f_hz: f64, skyhook: Option<f64>) -> f64 {
        let mut cfg = PlatformCfg::haven_r();
        if skyhook.is_some() {
            cfg.cu_area_m2 = 0.0;
        }
        let mut p = HavenR3Dof::new(&cfg).expect("valid");
        let w = 2.0 * core::f64::consts::PI * f_hz;
        let a = 0.5;
        let dt = 2.0e-3;
        let (mut t, mut peak) = (0.0, 0.0_f64);
        while t < 90.0 {
            let v_g = -a / w * (w * t).cos();
            let u = skyhook.map_or(0.0, |c| -c * (p.state.q[3] + v_g));
            p.step([a * (w * (t + 0.5 * dt)).sin(), 0.0], [u, 0.0, 0.0], dt);
            t += dt;
            if t > 80.0 {
                peak = peak.max(p.abs_accel_mps2()[0].abs());
            }
        }
        peak / a
    }

    #[test]
    fn isolation_transmissibility_t6() {
        // Passive (copper ζ 0.2) and skyhook (c_sky = 2ζmω, ζ 0.2) at T = 6 s against the
        // closed forms (PHYSICS §21; ±2 %).
        let cfg = PlatformCfg::haven_r();
        let zeta = cfg.copper_zeta();
        assert!(rel(zeta, 0.2) < 1e-12);
        let f_iso = 1.0 / guideway_period_s(cfg.r_guideway_m);
        let c_sky = 2.0 * zeta * cfg.m_kg * 2.0 * core::f64::consts::PI * f_iso;
        for f in [0.5, 1.0, 2.0] {
            let r = f / f_iso;
            let tp = transmissibility_sim(f, None);
            let ts = transmissibility_sim(f, Some(c_sky));
            let (ep, es) = (
                transmissibility_passive(r, zeta),
                transmissibility_skyhook(r, zeta),
            );
            println!(
                "T 6 s, {f} Hz (r {r:.2}): passive {tp:.4} (analytic {ep:.4}), skyhook {ts:.4} (analytic {es:.4})"
            );
            assert!(rel(tp, ep) <= 0.02, "passive transmissibility ±2 %");
            assert!(rel(ts, es) <= 0.02, "skyhook transmissibility ±2 %");
        }
    }

    // req: FAB-EXO-505
    #[test]
    fn occupied_platform_limits() {
        let cfg = PlatformCfg::haven_r();
        let t_iso = guideway_period_s(cfg.r_guideway_m);
        let zeta = cfg.copper_zeta();
        assert_eq!(asce_damping_coefficient(0.20), 1.5);
        assert_eq!(asce_damping_coefficient(0.05), 1.0);
        // The simulated platform reproduces a_peak = ω²·X·√(1 + 4ζ²) at resonance (±2 %).
        let mut p = HavenR3Dof::new(&cfg).expect("valid");
        let w = 2.0 * core::f64::consts::PI / t_iso;
        let ag = 0.05;
        let dt = 5.0e-3;
        let (mut t, mut x, mut a) = (0.0, 0.0_f64, 0.0_f64);
        while t < 120.0 {
            p.step([ag * (w * (t + 0.5 * dt)).sin(), 0.0], [0.0; 3], dt);
            t += dt;
            if t > 90.0 {
                x = x.max(p.state.q[0].abs());
                a = a.max(p.abs_accel_mps2()[0].abs());
            }
        }
        assert!(
            rel(a, isolated_peak_accel_mps2(t_iso, x, zeta)) <= 0.02,
            "a_peak = ω²X√(1 + 4ζ²) ±2 %"
        );
        // Near-fault pulse (Baker 2007, Mw 7, PGV 1 m/s): reported, not bounded.
        let mut nf = HavenR3Dof::new(&cfg).expect("valid");
        let pulse = NearFaultPulse::baker2007(7.0, 1.0, 5.0);
        let mut t = 0.0;
        while t < 40.0 {
            nf.step([pulse.accel_mps2(t + 0.5 * dt), 0.0], [0.0; 3], dt);
            t += dt;
        }
        // Landing through the PM-bias ramp.
        let mut land = HavenR3Dof::new(&cfg).expect("valid");
        land.lose_control();
        for _ in 0..2000 {
            land.step([0.0; 2], [0.0; 3], dt);
        }
        let v_land = land.touchdown_speed_mps().expect("landed");
        let rep = occupied_platform_report(
            0.6,
            0.9,
            t_iso,
            zeta,
            (nf.max_radial_m(), nf.peak_abs_accel_mps2()),
            v_land,
        );
        println!(
            "HAVEN-R T {t_iso:.3} s ζ {zeta:.2}: DBE S_D {:.3} m a {:.3} g; MCE S_M {:.3} m a {:.3} g (reported); near-fault {:.3} m, {:.3} g (reported{}); landing {:.4} m/s",
            rep.dbe_s_d_m,
            rep.dbe_a_peak_mps2 / G0_MPS2,
            rep.mce_s_m_m,
            rep.mce_a_peak_mps2 / G0_MPS2,
            rep.near_fault_disp_m,
            rep.near_fault_a_peak_mps2 / G0_MPS2,
            if nf.buffer_contact() {
                ", buffer contact"
            } else {
                ""
            },
            rep.landing_v_mps
        );
        assert!(
            rep.dbe_ok,
            "a_peak ≤ 0.1 g at DBE for S_D1 0.6 (FAB-EXO-505)"
        );
        assert!(rep.landing_ok, "landing ≤ 0.1 m/s (FAB-EXO-505)");
        // The MCE displacement is the design stroke (±0.9 m; ±1 %).
        assert!(
            rel(rep.mce_s_m_m, cfg.stroke_m) < 0.01,
            "S_M at MCE = design stroke 0.9 m"
        );
        assert!(
            rep.mce_a_peak_mps2.is_finite()
                && rep.near_fault_disp_m > 0.0
                && rep.near_fault_a_peak_mps2 > 0.0
        );
    }

    // req: FAB-EXO-701
    #[test]
    fn curved_guideway_period() {
        // T = 2π·√(R/g): HAVEN-R R 8.95 m → 6 s (±0.5 %).
        assert!(
            rel(guideway_period_s(8.95), 6.0) <= 0.005,
            "T(8.95 m) = 6 s ±0.5 % (FAB-EXO-701)"
        );
        assert!(
            rel(guideway_radius_m(6.0), 8.95) <= 0.005,
            "R(6 s) = 8.95 m ±0.5 %"
        );
        assert!(rel(guideway_period_s(guideway_radius_m(6.0)), 6.0) < 1e-12);
        // Simulated free oscillation of the undamped platform (zero crossings, 10 cycles).
        let mut cfg = PlatformCfg::haven_r();
        cfg.cu_area_m2 = 0.0;
        let mut p = HavenR3Dof::new(&cfg).expect("valid");
        p.state.q = [0.1, 0.0, 0.01, 0.0, 0.0, 0.0];
        let dt = 1.0e-3;
        let mut t = 0.0;
        let (mut prev_x, mut prev_th) = (p.state.q[0], p.state.q[2]);
        let (mut cx, mut cth) = (Vec::new(), Vec::new());
        while t < 61.0 {
            p.step([0.0; 2], [0.0; 3], dt);
            t += dt;
            let (x, th) = (p.state.q[0], p.state.q[2]);
            if prev_x > 0.0 && x <= 0.0 {
                cx.push(t - dt * x / (x - prev_x));
            }
            if prev_th > 0.0 && th <= 0.0 {
                cth.push(t - dt * th / (th - prev_th));
            }
            prev_x = x;
            prev_th = th;
        }
        let period = |c: &[f64]| (c[c.len() - 1] - c[0]) / (c.len() - 1) as f64;
        let (tx, tth) = (period(&cx), period(&cth));
        println!(
            "simulated period {tx:.4} s (translation), {tth:.4} s (yaw); closed form {:.4} s",
            guideway_period_s(cfg.r_guideway_m)
        );
        assert!(
            rel(tx, 6.0) <= 0.005,
            "simulated T 6 s ±0.5 % (FAB-EXO-701)"
        );
        assert!(rel(tx, guideway_period_s(cfg.r_guideway_m)) < 1e-4);
        assert!(
            rel(tth, tx) < 1e-4,
            "yaw shares the period (pods distributed like the mass)"
        );
    }

    // req: FAB-EXO-704
    #[test]
    fn control_loss_fps_landing() {
        let cfg = PlatformCfg::haven_r();
        let mut p = HavenR3Dof::new(&cfg).expect("valid");
        p.state.q = [0.3, 0.1, 0.0, 0.2, -0.1, 0.0];
        let dt = 5.0e-3;
        for _ in 0..400 {
            p.step([0.0; 2], [1.0e3, 0.0, 0.0], dt);
        }
        p.lose_control();
        assert!(p.control_lost());
        for _ in 0..12_000 {
            // The LSM command is ignored after control loss.
            p.step([0.0; 2], [5.0e4, 5.0e4, 0.0], dt);
        }
        let v = p.touchdown_speed_mps().expect("landed on the FPS");
        let free = LandingRamp::default().free_drop_speed_mps();
        println!(
            "control loss: touchdown {v:.4} m/s through the PM-bias ramp (free drop {free:.3} m/s)"
        );
        assert!(p.state.landed, "landed on the FPS catcher");
        assert!(
            v <= HAVEN_LANDING_V_MAX_MPS,
            "FPS landing ≤ 0.1 m/s (FAB-EXO-704): {v}"
        );
        assert!(
            free > HAVEN_LANDING_V_MAX_MPS,
            "an uncontrolled 12 mm drop would exceed it"
        );
        assert!(!p.buffer_contact(), "no buffer contact");
        // The FPS holds the platform: at rest, inside the friction band μ·R.
        let q = p.state.q;
        assert_eq!((q[3], q[4]), (0.0, 0.0), "stuck on the FPS");
        assert!(q[0].hypot(q[1]) <= cfg.fps_mu * cfg.fps_r_m + 1e-9);
    }

    // req: FAB-EXO-705
    #[test]
    fn rejects_no_passive_vertical() {
        let base = PlatformCfg {
            vertical: PassiveVertical::None,
            ..PlatformCfg::haven_r()
        };
        assert_eq!(
            HavenR3Dof::new(&base).err(),
            Some(PlatformError::NoPassiveVertical),
            "50 t without passive vertical support"
        );
        let light = PlatformCfg {
            m_kg: PASSIVE_VERTICAL_REQUIRED_ABOVE_KG,
            ..base
        };
        assert!(HavenR3Dof::new(&light).is_ok(), "≤ 1 t may omit it");
        let over = PlatformCfg {
            m_kg: PASSIVE_VERTICAL_REQUIRED_ABOVE_KG + 1.0,
            ..base
        };
        assert_eq!(
            HavenR3Dof::new(&over).err(),
            Some(PlatformError::NoPassiveVertical)
        );
        for v in [PassiveVertical::AirSpring, PassiveVertical::Qzs] {
            assert!(
                HavenR3Dof::new(&PlatformCfg {
                    vertical: v,
                    ..base
                })
                .is_ok()
            );
        }
        let f = PlatformCfg {
            m_kg: 5.0e6,
            ..base
        };
        assert_eq!(
            HavenR3Dof::new(&f).err(),
            Some(PlatformError::NoPassiveVertical),
            "HAVEN-F too"
        );
        assert_eq!(
            HavenR3Dof::new(&PlatformCfg { m_kg: 0.0, ..base }).err(),
            Some(PlatformError::InvalidField)
        );
        let wide = PlatformCfg {
            stroke_m: 1.5,
            ..PlatformCfg::haven_r()
        };
        assert_eq!(
            HavenR3Dof::new(&wide).err(),
            Some(PlatformError::StrokeExceedsMoat)
        );
    }

    // req: FAB-EXO-706
    #[test]
    fn copper_temperature_rise() {
        // Long-duration M9 (300 s, 0.3 g / 0.2 g): the copper ΔT is reported and stays
        // within its limit; ΔT·m_cu·c_p equals the dissipated energy.
        let cfg = PlatformCfg::haven_r();
        let mut p = HavenR3Dof::new(&cfg).expect("valid");
        let fs = 200.0;
        let ax = m9_long_duration_series(300.0, fs, 0.3 * G0_MPS2, 9);
        let ay = m9_long_duration_series(300.0, fs, 0.2 * G0_MPS2, 10);
        let c0 = p.copper_c_n_s_per_m();
        for (x, y) in ax.iter().zip(ay.iter()) {
            p.step([*x, *y], [0.0; 3], 1.0 / fs);
        }
        let dt_k = p.state.cu_dt_k;
        println!(
            "M9 300 s: copper ΔT {:.2} K (limit {:.0} K), E {:.3} MJ, max excursion {:.3} m (stroke {:.1} m, buffer {}), bumper peak {:.3} MJ",
            dt_k,
            cfg.cu_dt_max_k,
            p.copper_energy_j() / 1e6,
            p.max_radial_m(),
            cfg.stroke_m,
            p.buffer_contact(),
            p.buffer_peak_energy_j() / 1e6
        );
        assert!(dt_k > 0.0, "the copper heats");
        assert!(
            p.copper_within_limit() && dt_k <= cfg.cu_dt_max_k,
            "ΔT ≤ limit (FAB-EXO-706): {dt_k:.1} K"
        );
        assert!(
            rel(
                dt_k * cfg.cu_mass_kg * CU_CP_J_PER_KG_K,
                p.copper_energy_j()
            ) < 1e-9,
            "ΔT = E/(m·c_p)"
        );
        assert!(p.copper_c_n_s_per_m() < c0, "hot copper damps less (σ(T))");
    }
}
