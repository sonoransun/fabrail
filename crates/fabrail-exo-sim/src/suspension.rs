//! SuspensionModel trait and models: HybridEms adapter, PureEms, HTS (frozen image, play hysteresis, creep, Jc(T), LN2), AirPad (FAB-EXO-201..210). Owner: X4 (trait frozen by W0a).
//!
//! EXOTIC §X2, §X3, §X4 (f64 truth). [`SuspensionModel`] exposes force and stiffness in
//! z and y, a hysteresis state, the failure behaviour on supply loss, hover power, the
//! open-loop character and the gap window. Implementations:
//!
//! * [`HybridEmsAdapter`] delegates the magnetics to the core co-energy element
//!   (`fabrail_plant::magnet::element_force`, PHYSICS §2–§3) and never reimplements it;
//!   its hysteresis state is the eddy lag on the force flux (PHYSICS §2). FeCo is a
//!   pole-material parameter (B_sat 2.3 T), not a kind, and the rail saturates too: the
//!   element saturates at min(B_pole, B_rail) (FAB-EXO-207). On supply loss a zero-power
//!   hybrid EMS **diverges with an indeterminate direction** (at any gap below g*,
//!   F_PM > W; above it, F_PM < W): both stops are designed sliding surfaces and landing
//!   is an energised function with a monitored reserve (FAB-EXO-115).
//! * [`PureEms`]: F = K·(i/g)², 1.7 kW/t hover (Transrapid reference); drops at once.
//! * [`HtsBulk`]: frozen-image flux pinning with a rate-independent play operator,
//!   quasi-static log creep, the s_T(T) heuristic and an LN₂ reservoir with
//!   post-depletion warm-up; it fades in tens of minutes after depletion (research tier).
//! * [`HtsEmsTrim`]: HTS lift plus an EMS trim coil (`HybridHtsPlusEms`); the trim loop
//!   cancels creep drift (FAB-EXO-208).
//! * [`AirPad`]: non-magnetic comparison baseline (the FPS baseline is
//!   `crate::platform::FrictionPendulum`).
//!
//! No frontier or research-only suspension variant exists in Rust (FAB-EXO-210).
//!
//! The single pole-area admission function A = k·m·g/(η_fill·min(B_pole, B_rail,
//! B_gap,max)²/2μ0) is [`admission_area_m2`] (FAB-EXO-103); [`admission_table`] and
//! [`hts_area_table`] generate the per-class tables that EXOTIC.md reprints
//! (FAB-EXO-121 drift guard).
//!
//! Sign conventions: `ForceGap::kz_n_per_m` is ∂F_z/∂z literally. For attractive EMS
//! (magnet under the rail) the gap grows when the body moves down, so ∂F_z/∂z < 0 is
//! open-loop unstable; for HTS and the air pad z is a height, so ∂F_z/∂z < 0 is stable.
//! [`SuspensionModel::open_loop`] states the character explicitly.

use fabrail_exo_safety::presets::PRESETS;
use fabrail_exo_safety::{Badge, ExoClass, PoleMaterial, SuspensionKind};
use fabrail_params::SuspensionDesign;
use fabrail_params::derive::{self, G0_MPS2, MU0_H_PER_M};
use fabrail_plant::magnet;
use fabrail_types::fmath;

use crate::host::RailCondition;

/// Saturation flux density of electrical-steel poles, T (PHYSICS §2: B_sat 1.5 T).
pub const B_SAT_FE_T: f64 = 1.5;

/// Saturation flux density of FeCo poles, T (EXOTIC §X2: 2.3 T, P = 2.10 MPa).
pub const B_SAT_FECO_T: f64 = 2.3;

/// Saturation of a structural-steel rail, T (exotic critique: a 1.6 T rail caps the
/// pressure at 1.02 MPa whatever the pole material; EXOTIC §X2).
pub const B_RAIL_STEEL_T: f64 = 1.6;

/// Saturation of a FeCo-lined rail, T (exotic critique: 2.15 T → 1.84 MPa; EXOTIC §X2).
pub const B_RAIL_FECO_T: f64 = 2.15;

/// Admission safety factor k in A = k·m·g/P_eff (exotic design §2.3).
pub const ADMISSION_K: f64 = 1.5;

/// Default pole fill factor η_fill for the generated table, the middle of the design
/// range 0.3–0.6 (exotic design §2.3) \[UNCERTAIN\].
pub const ETA_FILL_DEFAULT: f64 = 0.45;

/// Default gap flux ceiling B_gap,max, T: the middle of 1.0–1.4 T for 10–15 mm PM-biased
/// gaps (FAB-EXO-103) \[UNCERTAIN\].
pub const B_GAP_MAX_DEFAULT_T: f64 = 1.2;

/// Pure-EMS hover power per kilogram, W/kg (Transrapid reference, 1.7 kW/t; EXOTIC §X4).
pub const PURE_EMS_HOVER_W_PER_KG: f64 = 1.7;

/// One kilogram-force, N (the ATZ data are quoted in kgf).
pub const KGF_N: f64 = G0_MPS2;

/// 20 °C in kelvin: the reference temperature of B_r and the coil resistance (PHYSICS
/// §12).
pub const T_REF_K: f64 = 293.15;

/// Speed that regularises the sign of a Coulomb-like drag, m/s (1 mm/s, the core
/// friction reference speed; PHYSICS §8).
pub const DRAG_V_REG_MPS: f64 = 1.0e-3;

// ---- HTS constants (EXOTIC §X3) ----------------------------------------------------------

/// LN₂ bath temperature used as the s_T reference, K (normal boiling point 77.3 K; the
/// Jc(T) heuristic is normalised at 77 K; exotic design §2.3).
pub const T_LN2_K: f64 = 77.0;

/// Liquid-nitrogen density at 77 K, kg/m³ (exotic design §2.3: 807).
pub const LN2_DENSITY_KG_PER_M3: f64 = 807.0;

/// LN₂ latent heat of vaporisation, J/kg (exotic design §2.3: 199 kJ/kg).
pub const LN2_H_FG_J_PER_KG: f64 = 199.0e3;

/// Usable fraction of the LN₂ fill before the bulks uncover, 1 (exotic design §2.3:
/// f_use 0.9).
pub const LN2_USABLE_FRAC: f64 = 0.9;

/// Land when the LN₂ reservoir falls to this fraction (FAB-EXO-205).
pub const LN2_LAND_FRAC: f64 = 0.10;

/// Critical LN₂ fraction (FAB-EXO-205).
pub const LN2_CRITICAL_FRAC: f64 = 0.04;

/// Cap of the s_T(T) heuristic (exotic design §2.3).
pub const HTS_ST_CAP: f64 = 2.0;

/// ATZ measured lift pressure at 10 mm, Pa (5.3 N/cm², Floegel-Delor et al., IEEE TAS
/// 29(5) 3601705, 2019; FAB-EXO-113 out-of-sample datum; used for the HTS area table).
pub const HTS_PRESSURE_10MM_PA: f64 = 53.0e3;

/// ATZ maximum lift pressure at a few mm, Pa (150 kPa; plan §4).
pub const HTS_PRESSURE_MAX_PA: f64 = 150.0e3;

/// ATZ cryostat-250 bulk area, m² (24 bulks of 64×32 mm; 492 cm², ATZ 2019).
pub const ATZ_CRYO250_AREA_M2: f64 = 492.0e-4;

/// ATZ cryostat-500 bulk area, m² (30 bulks of 78×39 mm; 912 cm², ATZ 2019).
pub const ATZ_CRYO500_AREA_M2: f64 = 912.0e-4;

/// ATZ cryostat-500 lift at 5–8 mm, N (5,000 N; FAB-EXO-113 second out-of-sample datum).
pub const ATZ_CRYO500_F_N: f64 = 5000.0;

/// ATZ cryostat-250 lift at 10 mm used for cryostat counts, N (2,500–3,000 N at 10 mm;
/// the exotic critique uses 2.6 kN).
pub const ATZ_CRYO250_F10_N: f64 = 2600.0;

/// ATZ cryostat-250 heat load at 77 K used for the area table, W (2–2.5 W; the exotic
/// critique uses 2.25 W).
pub const ATZ_CRYO250_HEAT_W: f64 = 2.25;

/// EMS comparison pressure of the HTS area table, Pa (0.3 MPa; EXOTIC §2).
pub const EMS_COMPARISON_PRESSURE_PA: f64 = 0.3e6;

/// Equivalent drag coefficient of HTS over a PM guideway (≤ 1e-5; EXOTIC §X8 table)
/// \[UNCERTAIN\].
pub const HTS_DRAG_MU: f64 = 1.0e-5;

/// Ambient temperature for the cryocooler Carnot bound, K.
pub const T_AMBIENT_K: f64 = 300.0;

/// Cryocooler efficiency as a fraction of Carnot at 77 K (10–20 % is typical; with it
/// the wall-plug power lands in the design's 50–250 W/t) \[UNCERTAIN\].
pub const CRYO_CARNOT_FRAC: f64 = 0.15;

/// Air-pad film stiffness factor κ in k_film = κ·W/h (orifice-compensated film, first
/// order) \[UNCERTAIN\].
pub const AIRPAD_FILM_KAPPA: f64 = 2.0;

/// Air-pad force ceiling as a multiple of the rated lift (supply pressure over film
/// pressure) \[UNCERTAIN\].
pub const AIRPAD_F_MAX_RATIO: f64 = 2.0;

/// Gap state of one suspension element (SI; z is the magnetic gap, y cross-track).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GapState {
    /// Gap, m.
    pub z_m: f64,
    /// Cross-track offset, m.
    pub y_m: f64,
    /// Gap rate, m/s.
    pub zdot_mps: f64,
    /// Cross-track rate, m/s.
    pub ydot_mps: f64,
    /// Travel speed, m/s (drag).
    pub xdot_mps: f64,
    /// Coil current, A.
    pub i_coil_a: f64,
    /// Temperature of the active material, K (0 = the model's reference temperature).
    pub temp_k: f64,
}

/// Force and stiffness of one element.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ForceGap {
    /// Lift force (toward closing the gap for attractive EMS), N.
    pub fz_n: f64,
    /// Cross-track force, N.
    pub fy_n: f64,
    /// ∂F_z/∂z, N/m (negative for EMS: open-loop unstable in heave).
    pub kz_n_per_m: f64,
    /// −∂F_y/∂y, N/m (positive: centring).
    pub ky_n_per_m: f64,
    /// Drag along travel, N.
    pub fx_drag_n: f64,
    /// Power dissipated, W.
    pub p_loss_w: f64,
}

/// Direction bias of a divergence.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Bias {
    /// Snaps up to the upper stop.
    Up,
    /// Drops to the lower stop.
    Down,
    /// Either way (zero-power hybrid EMS: depends on the gap relative to g*).
    #[default]
    Indeterminate,
}

/// What an element does when its supply is lost.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FailureBehaviour {
    /// Open-loop unstable: diverges at `pole_rad_s` in direction `bias`.
    Diverge {
        /// Open-loop pole, rad/s.
        pole_rad_s: f64,
        /// Direction.
        bias: Bias,
    },
    /// Lift decays with time constant `tau_s` (pure EMS, air pad).
    Immediate {
        /// Decay time constant, s.
        tau_s: f64,
    },
    /// Passive lift holds from a reservoir, then fades (HTS: LN₂).
    SlowFade {
        /// Reservoir hold time, s.
        reservoir_s: f64,
        /// Post-depletion fade time constant, s (tens of minutes, not hours).
        fade_tau_s: f64,
        /// Land when the reservoir falls to this fraction (0.10; 0.04 critical).
        land_trigger_frac: f64,
    },
    /// Unaffected.
    Unchanged,
}

/// Open-loop character of one axis.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum OpenLoop {
    /// Unstable with real pole `pole_rad_s`.
    Unstable {
        /// Pole, rad/s.
        pole_rad_s: f64,
    },
    /// Stable with natural frequency `wn_rad_s`.
    Stable {
        /// Natural frequency, rad/s.
        wn_rad_s: f64,
    },
    /// Neutral (no restoring force).
    #[default]
    Neutral,
}

/// Open-loop character per axis.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OpenLoopAxes {
    /// Heave (gap).
    pub z: OpenLoop,
    /// Cross-track.
    pub y: OpenLoop,
}

/// A suspension physics model (f64 truth).
pub trait SuspensionModel {
    /// Hysteresis / internal state (frozen-image references, eddy lag, reservoir).
    type Hyst: Copy + Default;
    /// Kind.
    const KIND: SuspensionKind;
    /// Model badge (the weakest of fidelity and technology maturity).
    const BADGE: Badge;
    /// Force and stiffness at state `s` with hysteresis `h`.
    fn force(&self, s: &GapState, h: &Self::Hyst) -> ForceGap;
    /// Advances the hysteresis state by `dt_s`.
    fn step_hysteresis(&self, s: &GapState, h: &mut Self::Hyst, dt_s: f64);
    /// Hover (holding) power at vertical acceleration RMS `a_vert_rms_mps2`, W.
    fn hover_power_w(&self, s: &GapState, a_vert_rms_mps2: f64) -> f64;
    /// Open-loop character per axis.
    fn open_loop(&self) -> OpenLoopAxes;
    /// Behaviour on supply loss.
    fn on_supply_loss(&self) -> FailureBehaviour;
    /// Usable gap window (lower stop, upper limit), m.
    fn gap_window_m(&self) -> (f64, f64);
}

// ---- Admission (EXOTIC §X2) --------------------------------------------------------------

/// Magnetic pressure B²/(2μ0), Pa (EXOTIC §X2; FeCo 2.3 T → 2.10 MPa).
pub fn magnetic_pressure_pa(b_t: f64) -> f64 {
    b_t * b_t / (2.0 * MU0_H_PER_M)
}

/// The single pole-area admission function (FAB-EXO-103):
/// A = k·m·g/(η_fill·min(B_pole, B_rail, B_gap,max)²/(2μ0)), m². The rail saturates too,
/// so the minimum of the pole, rail and gap flux densities governs (B_gap,max ≈
/// 1.0–1.4 T for 10–15 mm PM-biased gaps). Returns +∞ for a non-positive fill or flux.
pub fn admission_area_m2(
    m_kg: f64,
    k: f64,
    eta_fill: f64,
    b_pole_t: f64,
    b_rail_t: f64,
    b_gap_max_t: f64,
) -> f64 {
    let b = b_pole_t.min(b_rail_t).min(b_gap_max_t);
    if !(eta_fill > 0.0 && b > 0.0) {
        return f64::INFINITY;
    }
    k * m_kg * G0_MPS2 / (eta_fill * magnetic_pressure_pa(b))
}

/// Saturation flux density of a pole material, T (PHYSICS §2; EXOTIC §X2). `None` (no
/// iron poles) does not limit the flux.
pub fn pole_b_sat_t(pole: PoleMaterial) -> f64 {
    match pole {
        PoleMaterial::Fe => B_SAT_FE_T,
        PoleMaterial::FeCo => B_SAT_FECO_T,
        PoleMaterial::None => f64::INFINITY,
    }
}

/// One row of the generated pole-area admission table (FAB-EXO-103).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdmissionRow {
    /// Class.
    pub class: ExoClass,
    /// Suspension kind of the class preset.
    pub suspension: SuspensionKind,
    /// Design mass, kg.
    pub m_kg: f64,
    /// Governing flux density min(B_pole, B_rail, B_gap,max), T.
    pub b_eff_t: f64,
    /// Effective pressure η_fill·B²/2μ0 (EMS) or the HTS pressure at 10 mm, Pa.
    pub p_eff_pa: f64,
    /// Required pole (or bulk) area, m².
    pub area_m2: f64,
}

/// The per-class admission table: every EMS class through [`admission_area_m2`] (k = 1.5,
/// the preset pole material on a structural-steel rail at `b_rail_t`, fill `eta_fill`,
/// gap ceiling `b_gap_max_t`); HTS classes at the ATZ pressure m·g/p (EXOTIC §X2, §X3;
/// FAB-EXO-103). ARENA at η 0.45 and 1.2 T needs ≈ 491 m² (exotic critique).
pub fn admission_table(eta_fill: f64, b_rail_t: f64, b_gap_max_t: f64) -> [AdmissionRow; 9] {
    let mut rows = [AdmissionRow::default(); 9];
    for (row, p) in rows.iter_mut().zip(PRESETS.iter()) {
        let m = p.design_mass.to_kg_f64();
        let (b_eff, p_eff, area) = match p.suspension {
            SuspensionKind::HtsBulk => (
                0.0,
                HTS_PRESSURE_10MM_PA,
                hts_area_m2(m, HTS_PRESSURE_10MM_PA),
            ),
            _ => {
                let b = pole_b_sat_t(p.pole).min(b_rail_t).min(b_gap_max_t);
                (
                    b,
                    eta_fill * magnetic_pressure_pa(b),
                    admission_area_m2(
                        m,
                        ADMISSION_K,
                        eta_fill,
                        pole_b_sat_t(p.pole),
                        b_rail_t,
                        b_gap_max_t,
                    ),
                )
            }
        };
        *row = AdmissionRow {
            class: p.class,
            suspension: p.suspension,
            m_kg: m,
            b_eff_t: b_eff,
            p_eff_pa: p_eff,
            area_m2: area,
        };
    }
    rows
}

/// Bulk HTS area to carry `m_kg` at lift pressure `p_pa`: A = m·g/p, m² (EXOTIC §X3; the
/// area table uses the ATZ 53 kPa at 10 mm without a safety factor, as EXOTIC §2 does).
pub fn hts_area_m2(m_kg: f64, p_pa: f64) -> f64 {
    if p_pa <= 0.0 {
        return f64::INFINITY;
    }
    m_kg * G0_MPS2 / p_pa
}

/// One row of the generated HTS area table (EXOTIC §2 "HTS area needed at 10 mm").
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HtsAreaRow {
    /// Class.
    pub class: ExoClass,
    /// Mass, kg.
    pub m_kg: f64,
    /// Bulk area at 53 kPa, m².
    pub area_m2: f64,
    /// ATZ cryostat-250 units at 2.6 kN each.
    pub cryostats: f64,
    /// Heat load at 77 K at 2.25 W per cryostat, W.
    pub heat_w: f64,
    /// LN₂ boil-off, kg/day.
    pub ln2_kg_per_day: f64,
    /// EMS pole area at 0.3 MPa for comparison, m².
    pub ems_area_m2: f64,
}

/// The HTS area table for the classes with an HTS option: SENTINEL 1,000 t (185 m²),
/// HAVEN-R 50 t (9.3 m²), HAVEN-F 5 kt (925 m²) (EXOTIC §2, ±1 %; exotic critique).
/// HTS needs 5–8× the area of EMS.
pub fn hts_area_table() -> [HtsAreaRow; 3] {
    let classes = [ExoClass::Sentinel, ExoClass::HavenR, ExoClass::HavenF];
    let mut rows = [HtsAreaRow::default(); 3];
    for (row, class) in rows.iter_mut().zip(classes) {
        let m = PRESETS
            .iter()
            .find(|p| p.class == class)
            .map_or(0.0, |p| p.design_mass.to_kg_f64());
        let cryostats = fmath::ceil(m * G0_MPS2 / ATZ_CRYO250_F10_N);
        let heat_w = cryostats * ATZ_CRYO250_HEAT_W;
        *row = HtsAreaRow {
            class,
            m_kg: m,
            area_m2: hts_area_m2(m, HTS_PRESSURE_10MM_PA),
            cryostats,
            heat_w,
            ln2_kg_per_day: heat_w * 86_400.0 / LN2_H_FG_J_PER_KG,
            ems_area_m2: hts_area_m2(m, EMS_COMPARISON_PRESSURE_PA),
        };
    }
    rows
}

// ---- Hover power (EXOTIC §X4) ------------------------------------------------------------

/// Pure-EMS hover power P_W = 1.7 W/kg·m, W (Transrapid reference; EXOTIC §X4).
pub fn pure_ems_hover_power_w(m_kg: f64) -> f64 {
    PURE_EMS_HOVER_W_PER_KG * m_kg.max(0.0)
}

/// Zero-power hybrid EMS hover loss P = P_W·(a_rms/2g)², W (EXOTIC §X4; FAB-EXO-206:
/// 42 / 170 / 1,060 kW at 10 kt and 0.1 / 0.2 / 0.5 g RMS). The PM carries the weight;
/// the coil carries only the dynamic force m·a, which for F ∝ i² needs the pure-EMS
/// small-signal current δi = i_W·a/(2g), hence the copper loss P_W·(a/2g)².
pub fn hybrid_hover_power_w(m_kg: f64, a_vert_rms_mps2: f64) -> f64 {
    let r = a_vert_rms_mps2 / (2.0 * G0_MPS2);
    pure_ems_hover_power_w(m_kg) * r * r
}

// ---- Hybrid EMS adapter (PHYSICS §2–§3; EXOTIC §X2, §X4) ---------------------------------

/// Hybrid EMS adapter over the core reluctance model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HybridEmsAdapter {
    /// Element design (one element; `n_elements` identical ones share the load).
    pub design: SuspensionDesign,
    /// Number of lift elements.
    pub n_elements: f64,
    /// Pole material (FeCo raises the pole B_sat to 2.3 T; the rail, `design.b_sat_t`,
    /// still caps it).
    pub pole: PoleMaterial,
    /// Rail condition, which sets the hysteresis drag (EXOTIC §X8).
    pub rail: RailCondition,
}

/// Eddy-lag state of the hybrid adapter (PHYSICS §2: τ_e·Φ̇_eff = Φ − Φ_eff).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EddyLag {
    /// Lagged force flux Φ_eff as a fraction of the element's PM bias flux at g*
    /// ([`HybridEmsAdapter::flux_ref_wb`]). 0.0 (the default) means "not primed": the
    /// force uses the instantaneous flux until the first `step_hysteresis`.
    pub phi_frac: f64,
}

impl HybridEmsAdapter {
    /// An adapter of `n_elements` elements of `design` with `pole` poles on a solid-steel
    /// rail.
    pub fn new(design: SuspensionDesign, n_elements: f64, pole: PoleMaterial) -> HybridEmsAdapter {
        HybridEmsAdapter {
            design,
            n_elements,
            pole,
            rail: RailCondition::SolidSteel,
        }
    }

    /// Effective saturation flux density min(B_pole, B_rail), T (FAB-EXO-207 min rule;
    /// `design.b_sat_t` is the rail). +∞ when neither limits.
    pub fn b_sat_eff_t(&self) -> f64 {
        let rail = if self.design.b_sat_t > 0.0 {
            self.design.b_sat_t
        } else {
            f64::INFINITY
        };
        pole_b_sat_t(self.pole).min(rail)
    }

    /// The element design with the effective saturation (0 disables saturation in the
    /// plant element).
    fn element_design(&self) -> SuspensionDesign {
        let mut d = self.design;
        let b = self.b_sat_eff_t();
        d.b_sat_t = if b.is_finite() { b } else { 0.0 };
        d
    }

    /// Remanence at magnet temperature `temp_k` (tempco on B_r; PHYSICS §12), T.
    fn b_r_at(&self, temp_k: f64) -> f64 {
        let d = &self.design;
        if temp_k > 0.0 {
            d.b_r_t * (1.0 + d.tempco_br_per_k * (temp_k - T_REF_K))
        } else {
            d.b_r_t
        }
    }

    /// Coil resistance at `temp_k` (copper tempco; PHYSICS §12), Ω.
    fn r_coil_at(&self, temp_k: f64) -> f64 {
        let d = &self.design;
        if temp_k > 0.0 {
            d.r_coil_ohm * (1.0 + d.tempco_r_per_k * (temp_k - T_REF_K))
        } else {
            d.r_coil_ohm
        }
    }

    /// One element's loads at gap `g_m`, offset `y_m`, current `i_a` (PHYSICS §2).
    fn element(&self, g_m: f64, y_m: f64, i_a: f64, temp_k: f64) -> magnet::ElementForce {
        magnet::element_force(
            &self.element_design(),
            self.b_r_at(temp_k),
            g_m,
            y_m,
            i_a,
            1.0,
        )
    }

    /// PM bias flux of one element at g*, zero current, y = 0 and 20 °C, Wb: the scale
    /// of [`EddyLag::phi_frac`].
    pub fn flux_ref_wb(&self) -> f64 {
        self.element(self.design.g_star_m, 0.0, 0.0, 0.0).flux_wb
    }

    /// Mass the permanent magnets alone carry at the state's gap (zero power), kg.
    pub fn zero_power_mass_kg(&self, s: &GapState) -> f64 {
        let n = self.n_elements.max(0.0);
        n * self.element(s.z_m, s.y_m, 0.0, s.temp_k).f_attract_n / G0_MPS2
    }

    /// Lagged force flux of state `h`, or `phi` when the state is not primed, Wb.
    fn phi_eff(&self, h: &EddyLag, phi: f64) -> f64 {
        if h.phi_frac == 0.0 {
            phi
        } else {
            h.phi_frac * self.flux_ref_wb()
        }
    }
}

impl SuspensionModel for HybridEmsAdapter {
    type Hyst = EddyLag;
    const KIND: SuspensionKind = SuspensionKind::HybridEms;
    const BADGE: Badge = Badge::Research;

    fn force(&self, s: &GapState, h: &EddyLag) -> ForceGap {
        let n = self.n_elements.max(0.0);
        let ef = self.element(s.z_m, s.y_m, s.i_coil_a, s.temp_k);
        // Loads scale with the square of the force flux at fixed permeance
        // (F_q = ½·(Φ_f/P)²·∂P/∂q; PHYSICS §2), so the eddy lag enters as (Φ_eff/Φ)².
        let phi = ef.flux_wb;
        let lag = if fmath::abs(phi) > 1.0e-15 {
            let r = self.phi_eff(h, phi) / phi;
            r * r
        } else {
            1.0
        };
        // Static stiffness by central differences of the plant element (saturation and
        // fringing included; PHYSICS §3).
        let dz = (1.0e-4 * s.z_m).max(1.0e-8);
        let fp = self.element(s.z_m + dz, s.y_m, s.i_coil_a, s.temp_k);
        let fm = self.element(s.z_m - dz, s.y_m, s.i_coil_a, s.temp_k);
        let dy = 1.0e-6;
        let yp = self.element(s.z_m, s.y_m + dy, s.i_coil_a, s.temp_k);
        let ym = self.element(s.z_m, s.y_m - dy, s.i_coil_a, s.temp_k);
        let fz = n * ef.f_attract_n * lag;
        ForceGap {
            fz_n: fz,
            fy_n: n * ef.fy_n * lag,
            kz_n_per_m: n * (fp.f_attract_n - fm.f_attract_n) / (2.0 * dz),
            ky_n_per_m: -n * (yp.fy_n - ym.fy_n) / (2.0 * dy),
            // Hysteresis drag is velocity-independent: −μ_eq·|F_z|·sgn(ẋ) (EXOTIC §X8).
            fx_drag_n: -self.rail.mu_eq()
                * fmath::abs(fz)
                * fmath::tanh(s.xdot_mps / DRAG_V_REG_MPS),
            p_loss_w: n * self.r_coil_at(s.temp_k) * s.i_coil_a * s.i_coil_a,
        }
    }

    fn step_hysteresis(&self, s: &GapState, h: &mut EddyLag, dt_s: f64) {
        let phi_ref = self.flux_ref_wb();
        if fmath::abs(phi_ref) <= 1.0e-15 {
            return;
        }
        let phi = self.element(s.z_m, s.y_m, s.i_coil_a, s.temp_k).flux_wb;
        let mut phi_eff = self.phi_eff(h, phi);
        let tau = self.design.tau_eddy_s;
        // Exact first-order update over dt for a constant target (PHYSICS §2, §24).
        phi_eff = if tau > 0.0 && dt_s > 0.0 {
            phi_eff + (phi - phi_eff) * (1.0 - fmath::exp(-dt_s / tau))
        } else {
            phi
        };
        let frac = phi_eff / phi_ref;
        // 0.0 marks "not primed"; an exactly-zero lagged flux is stored as the smallest
        // normal, which is physically identical.
        h.phi_frac = if frac == 0.0 { f64::MIN_POSITIVE } else { frac };
    }

    fn hover_power_w(&self, s: &GapState, a_vert_rms_mps2: f64) -> f64 {
        let n = self.n_elements.max(0.0);
        let static_w = n * self.r_coil_at(s.temp_k) * s.i_coil_a * s.i_coil_a;
        static_w + hybrid_hover_power_w(self.zero_power_mass_kg(s), a_vert_rms_mps2)
    }

    fn open_loop(&self) -> OpenLoopAxes {
        let d = &self.design;
        OpenLoopAxes {
            z: OpenLoop::Unstable {
                pole_rad_s: derive::open_loop_pole(d.g_star_m, derive::h_prime(d.h_m_m, d.mu_rec)),
            },
            // Reluctance centring: F_y/F_z = −(g/w)·tanh(y/g), so k_y/m = g₀/w at small y
            // (PHYSICS §2, §6).
            y: if d.pole_w_m > 0.0 {
                OpenLoop::Stable {
                    wn_rad_s: fmath::sqrt(G0_MPS2 / d.pole_w_m),
                }
            } else {
                OpenLoop::Neutral
            },
        }
    }

    fn on_supply_loss(&self) -> FailureBehaviour {
        let d = &self.design;
        FailureBehaviour::Diverge {
            pole_rad_s: derive::open_loop_pole(d.g_star_m, derive::h_prime(d.h_m_m, d.mu_rec)),
            bias: Bias::Indeterminate,
        }
    }

    fn gap_window_m(&self) -> (f64, f64) {
        (self.design.g_up_m, self.design.g_land_m)
    }
}

// ---- Pure EMS (EXOTIC §X4) ---------------------------------------------------------------

/// Pure EMS element: F = K·(i/g)².
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PureEms {
    /// Force constant K, N·m²/A².
    pub k_n_m2_per_a2: f64,
    /// Supported mass, kg.
    pub m_kg: f64,
    /// Nominal gap, m.
    pub g0_m: f64,
    /// Coil time constant L/R, s.
    pub tau_coil_s: f64,
    /// Skid (lower stop) gap, m.
    pub g_skid_m: f64,
    /// Maximum gap, m.
    pub g_max_m: f64,
}

impl PureEms {
    /// A pure-EMS set that holds `m_kg` at gap `g0_m` with current `i0_a`:
    /// K = m·g·g0²/i0² (EXOTIC §X4).
    pub fn for_mass(
        m_kg: f64,
        g0_m: f64,
        i0_a: f64,
        tau_coil_s: f64,
        g_skid_m: f64,
        g_max_m: f64,
    ) -> PureEms {
        let k = if i0_a != 0.0 {
            m_kg * G0_MPS2 * g0_m * g0_m / (i0_a * i0_a)
        } else {
            0.0
        };
        PureEms {
            k_n_m2_per_a2: k,
            m_kg,
            g0_m,
            tau_coil_s,
            g_skid_m,
            g_max_m,
        }
    }

    /// Current that holds the weight at gap `g_m`: i = g·√(m·g₀/K), A.
    pub fn hover_current_a(&self, g_m: f64) -> f64 {
        if self.k_n_m2_per_a2 <= 0.0 {
            return 0.0;
        }
        g_m * fmath::sqrt(self.m_kg * G0_MPS2 / self.k_n_m2_per_a2)
    }
}

impl SuspensionModel for PureEms {
    type Hyst = ();
    const KIND: SuspensionKind = SuspensionKind::PureEms;
    const BADGE: Badge = Badge::Simplified;

    fn force(&self, s: &GapState, h: &()) -> ForceGap {
        let _ = h;
        let g = s.z_m.max(magnet::GAP_MIN_M);
        let i = s.i_coil_a;
        let k = self.k_n_m2_per_a2;
        let fz = k * i * i / (g * g);
        let i0 = self.hover_current_a(self.g0_m);
        // Copper loss scales with i²: P_W at the hover current (EXOTIC §X4).
        let p_loss = if i0 > 0.0 {
            pure_ems_hover_power_w(self.m_kg) * (i / i0) * (i / i0)
        } else {
            0.0
        };
        ForceGap {
            fz_n: fz,
            fy_n: 0.0,
            kz_n_per_m: -2.0 * fz / g,
            ky_n_per_m: 0.0,
            fx_drag_n: -RailCondition::Laminated.mu_eq()
                * fz
                * fmath::tanh(s.xdot_mps / DRAG_V_REG_MPS),
            p_loss_w: p_loss,
        }
    }

    fn step_hysteresis(&self, s: &GapState, h: &mut (), dt_s: f64) {
        let _ = (s, h, dt_s);
    }

    /// P_W·(g/g0)²: with F ∝ i² the copper loss ∝ i² ∝ F·g², so the mean power equals
    /// the weight's P_W whatever the vertical acceleration RMS (EXOTIC §X4).
    fn hover_power_w(&self, s: &GapState, a_vert_rms_mps2: f64) -> f64 {
        let _ = a_vert_rms_mps2;
        let r = if self.g0_m > 0.0 && s.z_m > 0.0 {
            s.z_m / self.g0_m
        } else {
            1.0
        };
        pure_ems_hover_power_w(self.m_kg) * r * r
    }

    fn open_loop(&self) -> OpenLoopAxes {
        OpenLoopAxes {
            // Fixed current: −∂F/∂g = 2F/g → p = √(2g₀/g0) (EXOTIC §X4).
            z: if self.g0_m > 0.0 {
                OpenLoop::Unstable {
                    pole_rad_s: fmath::sqrt(2.0 * G0_MPS2 / self.g0_m),
                }
            } else {
                OpenLoop::Neutral
            },
            y: OpenLoop::Neutral,
        }
    }

    fn on_supply_loss(&self) -> FailureBehaviour {
        FailureBehaviour::Immediate {
            tau_s: self.tau_coil_s,
        }
    }

    fn gap_window_m(&self) -> (f64, f64) {
        (self.g_skid_m, self.g_max_m)
    }
}

// ---- Bulk HTS (EXOTIC §X3) ---------------------------------------------------------------

/// Jc(T) heuristic s_T = 2j/(1 + j), j = [(1 − (T/Tc)²)/(1 − (77/Tc)²)]², capped at 2 and
/// 0 at or above Tc (exotic design §2.3) \[UNCERTAIN\].
pub fn hts_temperature_scale(temp_k: f64, tc_k: f64) -> f64 {
    if !(tc_k > T_LN2_K && temp_k < tc_k) {
        return 0.0;
    }
    let t = temp_k.max(0.0) / tc_k;
    let t77 = T_LN2_K / tc_k;
    let x = (1.0 - t * t) / (1.0 - t77 * t77);
    let j = x * x;
    (2.0 * j / (1.0 + j)).min(HTS_ST_CAP)
}

/// Quasi-static creep s_c(t) = 1 − S·log10(1 + t/τ0), floored at 0 (exotic design §2.3;
/// S 0.02–0.05/decade, τ0 1–10 s) \[UNCERTAIN\].
pub fn hts_creep_scale(t_s: f64, s_per_decade: f64, tau0_s: f64) -> f64 {
    if tau0_s <= 0.0 || t_s <= 0.0 {
        return 1.0;
    }
    (1.0 - s_per_decade * fmath::log10(1.0 + t_s / tau0_s)).max(0.0)
}

/// Bulk HTS flux pinning (frozen image): F_z = s_T·s_c·F0·[e^(−2kz) − e^(−k(z+z_ref))],
/// k_y = s_T·s_c·F0·k·e^(−k(z+z_ref)); a rate-independent play operator of half-width δ
/// moves (z_ref, y_ref) by η_h times its output; creep s_c(t) = 1 − S·log10(1 + t/τ0)
/// (quasi-static only); s_T = 2j/(1 + j), j = [(1 − (T/Tc)²)/(1 − (77/Tc)²)]² (heuristic,
/// capped at 2); an LN₂ reservoir t_hold = f_use·V·ρ·h_fg/Q̇ and, after depletion, a
/// bulk warm-up dT/dt = Q̇/C_bulk (EXOTIC §X3).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HtsBulk {
    /// Force scale F0 per cryostat set, N.
    pub f0_n: f64,
    /// Guideway wavenumber k = 2π/λ, 1/m.
    pub k_per_m: f64,
    /// Field-cooling gap z_fc, m.
    pub z_fc_m: f64,
    /// Play half-width δ, m.
    pub delta_m: f64,
    /// Play gain η_h (0: conservative loop).
    pub eta_h: f64,
    /// Creep rate S per decade.
    pub creep_per_decade: f64,
    /// Creep time constant τ0, s.
    pub creep_tau0_s: f64,
    /// Critical temperature Tc, K.
    pub tc_k: f64,
    /// LN₂ reservoir hold time at the static heat load, s (derived:
    /// [`HtsBulk::ln2_hold_s`] at zero activity).
    pub reservoir_s: f64,
    /// Post-depletion fade time constant, s (derived: [`HtsBulk::fade_time_s`]).
    pub fade_tau_s: f64,
    /// Working height (open-loop character, admission), m.
    pub z_work_m: f64,
    /// Landing skid height (lower stop), m.
    pub z_land_m: f64,
    /// Cryostats in the set.
    pub n_cryostats: f64,
    /// Total bulk area of the set, m².
    pub bulk_area_m2: f64,
    /// LN₂ fill per cryostat, m³.
    pub ln2_volume_m3: f64,
    /// Static heat load per cryostat at 77 K, W.
    pub q_static_w: f64,
    /// Heat load per cryostat under dynamic conditions, W (fitted to the ATZ dynamic
    /// hold).
    pub q_dynamic_w: f64,
    /// Bulk heat capacity per cryostat, J/K.
    pub c_bulk_j_per_k: f64,
    /// Speed at which the dynamic heat load is fully reached, m/s.
    pub v_dyn_ref_mps: f64,
}

impl HtsBulk {
    /// The 4-cryostat ATZ demonstrator (Floegel-Delor et al., IEEE TAS 29(5) 3601705,
    /// 2019; exotic design §2.3): z_fc 35 mm, δ 2 mm, η_h 0.2, S 0.03/decade, τ0 5 s,
    /// Tc 92 K (YBCO) \[UNCERTAIN picks within the design ranges\]; F0 and k are the
    /// first-loading fit to 300 kgf at 20 mm and 550 kgf at 15 mm
    /// ([`HtsBulk::fit_first_loading`]; the η_h = 0 fit is the design's 32.9 kN and
    /// 40.85 m⁻¹). Per cryostat: 492 cm² of bulk, 2.5 L LN₂, 2.5 W static (→ 40.1 h),
    /// 3.35 W dynamic (fitted to "> 30 h under dynamic conditions"), C_bulk
    /// 24 × 0.155 kg × 132 J/(kg·K) = 491 J/K (→ 26.9 min to s_T = e⁻¹).
    pub const fn atz_demonstrator() -> HtsBulk {
        let ln2_volume_m3 = 2.5e-3;
        let q_static_w = 2.5;
        let latent_j = LN2_USABLE_FRAC * ln2_volume_m3 * LN2_DENSITY_KG_PER_M3 * LN2_H_FG_J_PER_KG;
        HtsBulk {
            f0_n: 37_164.754_68,
            k_per_m: 39.870_377_104,
            z_fc_m: 0.035,
            delta_m: 0.002,
            eta_h: 0.2,
            creep_per_decade: 0.03,
            creep_tau0_s: 5.0,
            tc_k: 92.0,
            reservoir_s: latent_j / q_static_w,
            // (T_e − 77 K)·C_bulk/Q̇ with s_T(T_e) = e⁻¹, T_e = 85.2084 K.
            fade_tau_s: 1_612.266_84,
            z_work_m: 0.010,
            z_land_m: 0.003,
            n_cryostats: 4.0,
            bulk_area_m2: 4.0 * ATZ_CRYO250_AREA_M2,
            ln2_volume_m3,
            q_static_w,
            q_dynamic_w: latent_j / (30.0 * 3600.0),
            c_bulk_j_per_k: 24.0 * 0.155 * 132.0,
            v_dyn_ref_mps: 1.0,
        }
    }

    /// The state right after field cooling at z_fc with lateral offset `y_m`: frozen
    /// image at the FC pose, full reservoir, bulk at 77 K, creep clock at zero.
    pub fn field_cool(&self, y_m: f64) -> HtsState {
        HtsState {
            z_ref_m: self.z_fc_m,
            y_ref_m: y_m,
            t_since_fc_s: 0.0,
            ln2_frac: 1.0,
            z_play_m: self.z_fc_m,
            y_play_m: y_m,
            bulk_temp_k: T_LN2_K,
        }
    }

    /// s_T at `temp_k` for this material.
    pub fn s_t(&self, temp_k: f64) -> f64 {
        hts_temperature_scale(temp_k, self.tc_k)
    }

    /// s_c after `t_s` since field cooling.
    pub fn s_c(&self, t_s: f64) -> f64 {
        hts_creep_scale(t_s, self.creep_per_decade, self.creep_tau0_s)
    }

    /// Frozen-image reference on the first-loading branch from z_fc down to `z_m`:
    /// z_ref = z_fc + η_h·(min(z_fc, z + δ) − z_fc) (the play operator's closed form for a
    /// monotone approach).
    pub fn first_loading_z_ref(&self, z_m: f64) -> f64 {
        let p = self.z_fc_m.min(z_m + self.delta_m);
        self.z_fc_m + self.eta_h * (p - self.z_fc_m)
    }

    /// Unit-F0 frozen-image shape e^(−2kz) − e^(−k(z+z_ref)).
    fn shape(k: f64, z_m: f64, z_ref_m: f64) -> f64 {
        fmath::exp(-2.0 * k * z_m) - fmath::exp(-k * (z_m + z_ref_m))
    }

    /// Lift at `z_m` on the first-loading branch at s_T = s_c = 1, N.
    pub fn first_loading_force_n(&self, z_m: f64) -> f64 {
        self.f0_n * HtsBulk::shape(self.k_per_m, z_m, self.first_loading_z_ref(z_m))
    }

    /// Fits (F0, k) so the first-loading branch passes through `p1` and `p2` (gap m,
    /// force N), keeping z_fc, δ and η_h (EXOTIC §X3; FAB-EXO-202). The force ratio is
    /// monotone in k, so k comes from bisection on [1, 500] m⁻¹. `None` when the points
    /// cannot be bracketed.
    pub fn fit_first_loading(&self, p1: (f64, f64), p2: (f64, f64)) -> Option<HtsBulk> {
        let ((z1, f1), (z2, f2)) = (p1, p2);
        if !(f1 > 0.0 && f2 > 0.0 && z1 > 0.0 && z2 > 0.0 && z1 != z2) {
            return None;
        }
        let zr1 = self.first_loading_z_ref(z1);
        let zr2 = self.first_loading_z_ref(z2);
        let resid = |k: f64| -> f64 {
            let s1 = HtsBulk::shape(k, z1, zr1);
            let s2 = HtsBulk::shape(k, z2, zr2);
            if s1 <= 0.0 || s2 <= 0.0 {
                return f64::NAN;
            }
            fmath::ln(s2 / s1) - fmath::ln(f2 / f1)
        };
        let (mut lo, mut hi) = (1.0_f64, 500.0_f64);
        let (r_lo, r_hi) = (resid(lo), resid(hi));
        if !(r_lo.is_finite() && r_hi.is_finite()) || r_lo * r_hi > 0.0 {
            return None;
        }
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            let r = resid(mid);
            if !r.is_finite() {
                return None;
            }
            if (r > 0.0) == (r_hi > 0.0) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        let k = 0.5 * (lo + hi);
        let s1 = HtsBulk::shape(k, z1, zr1);
        let mut out = *self;
        out.k_per_m = k;
        out.f0_n = f1 / s1;
        Some(out)
    }

    /// Bulk temperature of state `h` (falls back to `s.temp_k`, then 77 K), K.
    fn bulk_temp_k(h: &HtsState, s: &GapState) -> f64 {
        if h.bulk_temp_k > 0.0 {
            h.bulk_temp_k
        } else if s.temp_k > 0.0 {
            s.temp_k
        } else {
            T_LN2_K
        }
    }

    /// Dynamic activity 0…1 from the element's speed (|v| / v_dyn_ref, capped).
    pub fn activity(&self, s: &GapState) -> f64 {
        let v = fmath::sqrt(
            s.xdot_mps * s.xdot_mps + s.ydot_mps * s.ydot_mps + s.zdot_mps * s.zdot_mps,
        );
        if self.v_dyn_ref_mps > 0.0 {
            (v / self.v_dyn_ref_mps).min(1.0)
        } else {
            0.0
        }
    }

    /// Heat load of the set at activity `a` (0 static … 1 dynamic), W.
    pub fn heat_load_w(&self, a: f64) -> f64 {
        let a = a.clamp(0.0, 1.0);
        self.n_cryostats * (self.q_static_w + a * (self.q_dynamic_w - self.q_static_w))
    }

    /// Usable LN₂ latent heat of the set, J (f_use·V·ρ·h_fg per cryostat).
    pub fn ln2_latent_j(&self) -> f64 {
        self.n_cryostats
            * LN2_USABLE_FRAC
            * self.ln2_volume_m3
            * LN2_DENSITY_KG_PER_M3
            * LN2_H_FG_J_PER_KG
    }

    /// LN₂ hold time at activity `a`, s (40 h static, 30 h dynamic for the ATZ set).
    pub fn ln2_hold_s(&self, a: f64) -> f64 {
        let q = self.heat_load_w(a);
        if q <= 0.0 {
            return f64::INFINITY;
        }
        self.ln2_latent_j() / q
    }

    /// Temperature at which s_T falls to e⁻¹, K.
    pub fn fade_temp_k(&self) -> f64 {
        let (mut lo, mut hi) = (T_LN2_K, self.tc_k);
        for _ in 0..100 {
            let mid = 0.5 * (lo + hi);
            if self.s_t(mid) > core::f64::consts::E.recip() {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }

    /// Post-depletion fade time: static-load warm-up from 77 K to s_T = e⁻¹,
    /// (T_e − 77)·C_bulk/Q̇, s (tens of minutes, not hours; FAB-EXO-205).
    pub fn fade_time_s(&self) -> f64 {
        if self.q_static_w <= 0.0 {
            return f64::INFINITY;
        }
        (self.fade_temp_k() - T_LN2_K) * self.c_bulk_j_per_k / self.q_static_w
    }

    /// Reservoir status of state `h` (FAB-EXO-205: land at 10 %, 4 % critical).
    pub fn ln2_status(&self, h: &HtsState) -> Ln2Status {
        if h.ln2_frac <= 0.0 {
            Ln2Status::Depleted
        } else if h.ln2_frac <= LN2_CRITICAL_FRAC {
            Ln2Status::Critical
        } else if h.ln2_frac <= LN2_LAND_FRAC {
            Ln2Status::Land
        } else {
            Ln2Status::Ok
        }
    }

    /// Lift pressure F_z/A_bulk, Pa.
    pub fn pressure_pa(&self, s: &GapState, h: &HtsState) -> f64 {
        if self.bulk_area_m2 <= 0.0 {
            return 0.0;
        }
        self.force(s, h).fz_n / self.bulk_area_m2
    }

    /// Wall-plug cryocooler power to remove heat `q_w` at 77 K: Q̇/(η_C·T_c/(T_a − T_c)),
    /// W (≈ 190 W/t for the ATZ set; design 50–250 W/t).
    pub fn cryo_wall_plug_w(q_w: f64) -> f64 {
        let cop = CRYO_CARNOT_FRAC * T_LN2_K / (T_AMBIENT_K - T_LN2_K);
        q_w.max(0.0) / cop
    }
}

/// LN₂ reservoir status (FAB-EXO-205).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Ln2Status {
    /// Above the landing trigger.
    #[default]
    Ok,
    /// At or below 10 %: land now.
    Land,
    /// At or below 4 %: critical.
    Critical,
    /// Empty: the bulks warm up and the lift fades.
    Depleted,
}

/// HTS hysteresis and reservoir state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HtsState {
    /// Frozen-image vertical reference, m.
    pub z_ref_m: f64,
    /// Frozen-image lateral reference, m.
    pub y_ref_m: f64,
    /// Time since field cooling, s (creep clock).
    pub t_since_fc_s: f64,
    /// LN₂ remaining fraction, 0..1.
    pub ln2_frac: f64,
    /// Play-operator output in z, m.
    pub z_play_m: f64,
    /// Play-operator output in y, m.
    pub y_play_m: f64,
    /// Bulk temperature, K (0: use `GapState::temp_k`, then 77 K).
    pub bulk_temp_k: f64,
}

/// Rate-independent play operator of half-width `delta`: returns the new output for
/// input `x` and previous output `p` (Krasnosel'skii–Pokrovskii play).
fn play(x: f64, p: f64, delta: f64) -> f64 {
    if x - p > delta {
        x - delta
    } else if p - x > delta {
        x + delta
    } else {
        p
    }
}

impl SuspensionModel for HtsBulk {
    type Hyst = HtsState;
    const KIND: SuspensionKind = SuspensionKind::HtsBulk;
    const BADGE: Badge = Badge::Research;

    fn force(&self, s: &GapState, h: &HtsState) -> ForceGap {
        let k = self.k_per_m;
        let a = self.s_t(HtsBulk::bulk_temp_k(h, s)) * self.s_c(h.t_since_fc_s) * self.f0_n;
        let z = s.z_m.max(0.0);
        let e1 = fmath::exp(-2.0 * k * z);
        let e2 = fmath::exp(-k * (z + h.z_ref_m));
        let fz = a * (e1 - e2);
        let ky = a * k * e2;
        ForceGap {
            fz_n: fz,
            fy_n: -ky * (s.y_m - h.y_ref_m),
            // ∂F_z/∂z at a frozen reference (the minor-loop stiffness): negative = stable
            // for a height coordinate.
            kz_n_per_m: -a * k * (2.0 * e1 - e2),
            ky_n_per_m: ky,
            fx_drag_n: -HTS_DRAG_MU * fmath::abs(fz) * fmath::tanh(s.xdot_mps / DRAG_V_REG_MPS),
            p_loss_w: self.heat_load_w(self.activity(s)),
        }
    }

    fn step_hysteresis(&self, s: &GapState, h: &mut HtsState, dt_s: f64) {
        // Play operator: the frozen image moves by η_h times the play output. This is the
        // rate-independent form of the design's "z_ref += η_h·(|z − z_ref| − δ)" (a
        // per-step update would make η_h a per-step rate and the loop dt-dependent).
        let zp = play(s.z_m, h.z_play_m, self.delta_m);
        h.z_ref_m += self.eta_h * (zp - h.z_play_m);
        h.z_play_m = zp;
        let yp = play(s.y_m, h.y_play_m, self.delta_m);
        h.y_ref_m += self.eta_h * (yp - h.y_play_m);
        h.y_play_m = yp;
        if dt_s <= 0.0 {
            return;
        }
        h.t_since_fc_s += dt_s;
        if h.bulk_temp_k <= 0.0 {
            h.bulk_temp_k = if s.temp_k > 0.0 { s.temp_k } else { T_LN2_K };
        }
        // LN₂ boil-off; once empty the heat warms the bulks (EXOTIC §X3).
        let q_j = self.heat_load_w(self.activity(s)) * dt_s;
        let latent = self.ln2_latent_j();
        let boil_j = if latent > 0.0 {
            (h.ln2_frac.max(0.0) * latent).min(q_j)
        } else {
            0.0
        };
        if latent > 0.0 {
            h.ln2_frac = (h.ln2_frac - boil_j / latent).max(0.0);
        }
        let excess_j = q_j - boil_j;
        let c = self.n_cryostats * self.c_bulk_j_per_k;
        if excess_j > 0.0 && c > 0.0 {
            h.bulk_temp_k += excess_j / c;
        }
    }

    /// Levitation itself needs no power; the cryocooler wall-plug power for the heat load
    /// at the element's activity is reported (EXOTIC §X3: 50–250 W/t). The vertical
    /// acceleration RMS enters through the element's velocities in `s`.
    fn hover_power_w(&self, s: &GapState, a_vert_rms_mps2: f64) -> f64 {
        let _ = a_vert_rms_mps2;
        HtsBulk::cryo_wall_plug_w(self.heat_load_w(self.activity(s)))
    }

    fn open_loop(&self) -> OpenLoopAxes {
        // Natural frequencies at the working height with the load that height carries on
        // the first-loading branch (ATZ 10 mm: ≈ 5 Hz vertical, ≈ 2.4 Hz lateral).
        let z = self.z_work_m;
        let zr = self.first_loading_z_ref(z);
        let k = self.k_per_m;
        let f = self.f0_n * HtsBulk::shape(k, z, zr);
        if !(f > 0.0 && f.is_finite()) {
            return OpenLoopAxes::default();
        }
        let e1 = fmath::exp(-2.0 * k * z);
        let e2 = fmath::exp(-k * (z + zr));
        let kz = self.f0_n * k * (2.0 * e1 - e2);
        let ky = self.f0_n * k * e2;
        OpenLoopAxes {
            z: OpenLoop::Stable {
                wn_rad_s: fmath::sqrt(kz.max(0.0) * G0_MPS2 / f),
            },
            y: OpenLoop::Stable {
                wn_rad_s: fmath::sqrt(ky.max(0.0) * G0_MPS2 / f),
            },
        }
    }

    fn on_supply_loss(&self) -> FailureBehaviour {
        FailureBehaviour::SlowFade {
            reservoir_s: self.reservoir_s,
            fade_tau_s: self.fade_tau_s,
            land_trigger_frac: LN2_LAND_FRAC,
        }
    }

    fn gap_window_m(&self) -> (f64, f64) {
        (self.z_land_m, self.z_fc_m)
    }
}

// ---- HTS plus EMS trim (EXOTIC §X3) ------------------------------------------------------

/// HTS lift plus an EMS trim coil, F_trim = k_i·i (`HybridHtsPlusEms`). An outer loop
/// drives the creep drift to zero and adds active damping (exotic design §2.3).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HtsEmsTrim {
    /// HTS lift.
    pub hts: HtsBulk,
    /// Trim force constant, N/A.
    pub k_i_n_per_a: f64,
    /// Trim coil resistance, Ω.
    pub r_coil_ohm: f64,
    /// Trim current limit, A.
    pub i_max_a: f64,
}

impl HtsEmsTrim {
    /// Trim current after the limit, A.
    pub fn trim_current_a(&self, i_a: f64) -> f64 {
        i_a.clamp(-self.i_max_a.abs(), self.i_max_a.abs())
    }
}

impl SuspensionModel for HtsEmsTrim {
    type Hyst = HtsState;
    const KIND: SuspensionKind = SuspensionKind::HybridHtsPlusEms;
    const BADGE: Badge = Badge::Research;

    fn force(&self, s: &GapState, h: &HtsState) -> ForceGap {
        let mut f = self.hts.force(s, h);
        let i = self.trim_current_a(s.i_coil_a);
        f.fz_n += self.k_i_n_per_a * i;
        f.p_loss_w += self.r_coil_ohm * i * i;
        f
    }

    fn step_hysteresis(&self, s: &GapState, h: &mut HtsState, dt_s: f64) {
        self.hts.step_hysteresis(s, h, dt_s);
    }

    fn hover_power_w(&self, s: &GapState, a_vert_rms_mps2: f64) -> f64 {
        let i = self.trim_current_a(s.i_coil_a);
        self.hts.hover_power_w(s, a_vert_rms_mps2) + self.r_coil_ohm * i * i
    }

    fn open_loop(&self) -> OpenLoopAxes {
        self.hts.open_loop()
    }

    /// The HTS lift is passive: the trim is lost and creep drift goes uncorrected, but
    /// the reservoir still holds.
    fn on_supply_loss(&self) -> FailureBehaviour {
        self.hts.on_supply_loss()
    }

    fn gap_window_m(&self) -> (f64, f64) {
        self.hts.gap_window_m()
    }
}

// ---- Air pad (comparison baseline) -------------------------------------------------------

/// Air-pad baseline: constant lift while the pump runs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AirPad {
    /// Lift capacity, N.
    pub lift_n: f64,
    /// Pump power, W.
    pub pump_w: f64,
    /// Time to lift, s (Citicorp damper: 3 min).
    pub lift_time_s: f64,
    /// Flying height, m.
    pub fly_height_m: f64,
}

impl SuspensionModel for AirPad {
    type Hyst = ();
    const KIND: SuspensionKind = SuspensionKind::AirPad;
    const BADGE: Badge = Badge::Simplified;

    /// Film force linear about the flying height, F = W·(1 − κ·(z − h)/h), clamped to
    /// [0, 2W] (first-order orifice-compensated film) \[UNCERTAIN\].
    fn force(&self, s: &GapState, h: &()) -> ForceGap {
        let _ = h;
        let hf = self.fly_height_m;
        if !(hf > 0.0 && hf.is_finite()) {
            return ForceGap::default();
        }
        let raw = 1.0 - AIRPAD_FILM_KAPPA * (s.z_m - hf) / hf;
        let r = raw.clamp(0.0, AIRPAD_F_MAX_RATIO);
        let linear = raw > 0.0 && raw < AIRPAD_F_MAX_RATIO;
        ForceGap {
            fz_n: self.lift_n * r,
            kz_n_per_m: if linear {
                -AIRPAD_FILM_KAPPA * self.lift_n / hf
            } else {
                0.0
            },
            p_loss_w: self.pump_w,
            ..ForceGap::default()
        }
    }

    fn step_hysteresis(&self, s: &GapState, h: &mut (), dt_s: f64) {
        let _ = (s, h, dt_s);
    }

    fn hover_power_w(&self, s: &GapState, a_vert_rms_mps2: f64) -> f64 {
        let _ = (s, a_vert_rms_mps2);
        self.pump_w
    }

    fn open_loop(&self) -> OpenLoopAxes {
        OpenLoopAxes {
            z: if self.fly_height_m > 0.0 {
                OpenLoop::Stable {
                    wn_rad_s: fmath::sqrt(AIRPAD_FILM_KAPPA * G0_MPS2 / self.fly_height_m),
                }
            } else {
                OpenLoop::Neutral
            },
            y: OpenLoop::Neutral,
        }
    }

    fn on_supply_loss(&self) -> FailureBehaviour {
        FailureBehaviour::Immediate { tau_s: 0.0 }
    }

    fn gap_window_m(&self) -> (f64, f64) {
        (0.0, self.fly_height_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::FrictionPendulum;
    use crate::storage::Supercap;
    use fabrail_exo::energy::EnergyStore;
    use fabrail_params::SystemParams;
    use fabrail_params::derive::FailDir;
    use fabrail_params::presets;

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    fn adapter(p: &SystemParams, pole: PoleMaterial) -> HybridEmsAdapter {
        HybridEmsAdapter::new(
            p.design.suspension,
            derive::n_lift(&p.design.suspension),
            pole,
        )
    }

    /// Loads an HTS element monotonically from its FC height to `z_target` in 10 µm steps
    /// (no elapsed time: play operator only); returns the state at `z_target`.
    fn load_to(h: &HtsBulk, z_target: f64) -> HtsState {
        let mut st = h.field_cool(0.0);
        let n = ((h.z_fc_m - z_target) / 1.0e-5).round() as usize;
        for i in 1..=n {
            let z = h.z_fc_m - (h.z_fc_m - z_target) * i as f64 / n as f64;
            let s = GapState {
                z_m: z,
                ..GapState::default()
            };
            h.step_hysteresis(&s, &mut st, 0.0);
        }
        st
    }

    fn fz_at(h: &HtsBulk, st: &HtsState, z: f64) -> f64 {
        h.force(
            &GapState {
                z_m: z,
                ..GapState::default()
            },
            st,
        )
        .fz_n
    }

    // req: FAB-EXO-103
    #[test]
    fn admission_single_function() {
        // Closed form (E, rel 1e-12): A = k·m·g/(η·B²/2μ0) at the governing flux.
        let m = 8.6e6;
        let a = admission_area_m2(m, ADMISSION_K, 0.45, 2.3, 1.6, 1.2);
        let expect = 1.5 * m * G0_MPS2 / (0.45 * 1.2 * 1.2 / (2.0 * MU0_H_PER_M));
        assert!(
            rel(a, expect) < 1e-12,
            "closed form (E) rel 1e-12: {a} vs {expect}"
        );
        // ARENA 8.6 kt at η 0.45 and 1.2 T with k = 1.5: 491 m² (exotic critique; ±1 %).
        assert!(
            rel(a, 491.0) < 0.01,
            "ARENA 491 m² ±1 % (exotic critique), got {a:.1}"
        );
        // One function, min rule: the order of the three flux limits is irrelevant.
        let perms = [
            (2.3, 1.6, 1.2),
            (2.3, 1.2, 1.6),
            (1.6, 2.3, 1.2),
            (1.6, 1.2, 2.3),
            (1.2, 2.3, 1.6),
            (1.2, 1.6, 2.3),
        ];
        for (bp, br, bg) in perms {
            assert_eq!(
                admission_area_m2(m, ADMISSION_K, 0.45, bp, br, bg),
                a,
                "min(B_pole, B_rail, B_gap) rule (FAB-EXO-103)"
            );
        }
        // Across the stated B_gap,max window 1.0–1.4 T the area scales as 1/B².
        let a10 = admission_area_m2(m, ADMISSION_K, 0.45, 2.3, 1.6, 1.0);
        let a14 = admission_area_m2(m, ADMISSION_K, 0.45, 2.3, 1.6, 1.4);
        assert!(rel(a10 / a14, 1.96) < 1e-12, "A ∝ 1/B² (E)");
        assert_eq!(
            admission_area_m2(1.0, 1.5, 0.0, 1.0, 1.0, 1.0),
            f64::INFINITY
        );
        // The generated per-class table is the same function row by row.
        let rows = admission_table(ETA_FILL_DEFAULT, B_RAIL_STEEL_T, B_GAP_MAX_DEFAULT_T);
        for (r, p) in rows.iter().zip(PRESETS.iter()) {
            assert_eq!(r.class, p.class);
            if p.suspension == SuspensionKind::HtsBulk {
                assert!(rel(r.area_m2, hts_area_m2(r.m_kg, HTS_PRESSURE_10MM_PA)) < 1e-12);
            } else {
                let direct = admission_area_m2(
                    r.m_kg,
                    ADMISSION_K,
                    ETA_FILL_DEFAULT,
                    pole_b_sat_t(p.pole),
                    B_RAIL_STEEL_T,
                    B_GAP_MAX_DEFAULT_T,
                );
                assert_eq!(
                    r.area_m2,
                    direct,
                    "{}: table row = single function",
                    p.class.slug()
                );
                assert_eq!(
                    r.b_eff_t, B_GAP_MAX_DEFAULT_T,
                    "the gap flux governs at 1.2 T"
                );
            }
            println!(
                "admission {:9} {:>10.0} kg  B {:.2} T  P_eff {:>8.0} Pa  A {:>9.2} m²",
                p.class.slug(),
                r.m_kg,
                r.b_eff_t,
                r.p_eff_pa,
                r.area_m2
            );
        }
        // SENTINEL 1 kt with k: inside the critique's 37–59 m² band.
        let sentinel = rows[0].area_m2;
        assert!(
            (37.0..=59.0).contains(&sentinel),
            "SENTINEL 37–59 m² (exotic critique), got {sentinel:.1}"
        );
    }

    // req: FAB-EXO-113
    #[test]
    fn hts_out_of_sample() {
        let h = HtsBulk::atz_demonstrator();
        // 5.3 N/cm² at 10 mm on the first-loading branch (ATZ 2019; ±15 %).
        let st = load_to(&h, 0.010);
        let p10 = fz_at(&h, &st, 0.010) / h.bulk_area_m2;
        println!(
            "HTS p(10 mm) = {:.2} kPa vs 53 kPa ({:+.1} %)",
            p10 / 1e3,
            (p10 / 53e3 - 1.0) * 100.0
        );
        assert!(
            rel(p10, HTS_PRESSURE_10MM_PA) <= 0.15,
            "53 kPa at 10 mm ±15 % (FAB-EXO-113, ATZ 2019): {p10:.0} Pa"
        );
        // Cryostat-500: 5,000 N over 912 cm² at 5–8 mm (±30 %), evaluated as the model
        // pressure × 912 cm² at the band midpoint 6.5 mm on the first-loading branch.
        let f_band = |z: f64| fz_at(&h, &load_to(&h, z), z) / h.bulk_area_m2 * ATZ_CRYO500_AREA_M2;
        let (f5, f65, f8) = (f_band(0.005), f_band(0.0065), f_band(0.008));
        println!(
            "cryostat-500 model: {f5:.0} N @5 mm, {f65:.0} N @6.5 mm, {f8:.0} N @8 mm (datum 5000 N)"
        );
        assert!(
            rel(f65, ATZ_CRYO500_F_N) <= 0.30,
            "5000 N at 5–8 mm ±30 % (FAB-EXO-113): {f65:.0} N at 6.5 mm"
        );
        assert!(
            f8 * 0.7 <= ATZ_CRYO500_F_N && ATZ_CRYO500_F_N <= f5 * 1.3,
            "5000 N inside the ±30 % model band over 5–8 mm"
        );
        // The maximum pressure stays within the ATZ ≈ 150 kPa at a few mm (±30 %).
        let p3 = fz_at(&h, &load_to(&h, 0.003), 0.003) / h.bulk_area_m2;
        assert!(
            p3 <= 1.3 * HTS_PRESSURE_MAX_PA,
            "≤ 150 kPa ±30 % at 3 mm (plan §4): {p3:.0} Pa"
        );
    }

    #[test]
    fn hts_area_table_anchors() {
        // EXOTIC §2 (generated, ±1 %): SENTINEL 185 m² (EMS at 0.3 MPa 33 m²), HAVEN-R 9.3 m²,
        // HAVEN-F 925 m².
        let rows = hts_area_table();
        let expect = [(185.0, 33.0), (9.3, 1.63), (925.0, 163.0)];
        for (r, (a, ems)) in rows.iter().zip(expect) {
            println!(
                "HTS {:9} A {:>8.2} m²  cryostats {:>6.0}  heat {:>7.0} W  LN2 {:>7.0} kg/day  EMS {:>6.1} m²",
                r.class.slug(),
                r.area_m2,
                r.cryostats,
                r.heat_w,
                r.ln2_kg_per_day,
                r.ems_area_m2
            );
            assert!(
                rel(r.area_m2, a) <= 0.01,
                "{}: HTS area {a} m² ±1 % (EXOTIC §2)",
                r.class.slug()
            );
            assert!(
                rel(r.ems_area_m2, ems) <= 0.02,
                "{}: EMS comparison area",
                r.class.slug()
            );
            // HTS needs 5–8× the EMS area (plan §4) at k = 1.5 and 0.3 MPa: 185/33 = 5.6.
            assert!(
                (5.0..=8.0).contains(&(r.area_m2 / r.ems_area_m2)),
                "HTS 5–8× EMS area"
            );
        }
    }

    // req: FAB-EXO-115
    #[test]
    fn no_power_safe_state_reachable() {
        for kind in SuspensionKind::ALL {
            match *kind {
                SuspensionKind::HybridEms => {
                    for p in [presets::door(), presets::wall(), presets::heavy()] {
                        hybrid_safe_state(&p);
                    }
                }
                SuspensionKind::PureEms => {
                    let m = PureEms::for_mass(1.0e4, 0.010, 100.0, 0.05, 0.004, 0.015);
                    assert!(matches!(
                        m.on_supply_loss(),
                        FailureBehaviour::Immediate { .. }
                    ));
                    let (lo, hi) = m.gap_window_m();
                    assert!(
                        lo > 0.0 && lo < m.g0_m && m.g0_m < hi,
                        "pure EMS drops onto designed skids"
                    );
                    let v = fmath::sqrt(2.0 * G0_MPS2 * (m.g0_m - lo));
                    println!("pure EMS skid drop {v:.3} m/s");
                }
                SuspensionKind::HtsBulk | SuspensionKind::HybridHtsPlusEms => {
                    let h = HtsBulk::atz_demonstrator();
                    let fb = if *kind == SuspensionKind::HtsBulk {
                        h.on_supply_loss()
                    } else {
                        HtsEmsTrim {
                            hts: h,
                            k_i_n_per_a: 200.0,
                            r_coil_ohm: 0.5,
                            i_max_a: 20.0,
                        }
                        .on_supply_loss()
                    };
                    match fb {
                        FailureBehaviour::SlowFade {
                            reservoir_s,
                            land_trigger_frac,
                            ..
                        } => {
                            assert_eq!(land_trigger_frac, LN2_LAND_FRAC);
                            // The window from the 10 % trigger to empty (≥ 3 h) dwarfs an
                            // energised landing (seconds to minutes).
                            assert!(
                                land_trigger_frac * reservoir_s >= 3.0 * 3600.0,
                                "HTS landing window ≥ 3 h"
                            );
                        }
                        other => panic!("HTS must fade slowly, got {other:?}"),
                    }
                    assert!(h.gap_window_m().0 > 0.0, "HTS lands on skids");
                }
                SuspensionKind::AirPad => {
                    let a = AirPad {
                        lift_n: 1.0e5,
                        pump_w: 2.0e3,
                        lift_time_s: 180.0,
                        fly_height_m: 1.0e-4,
                    };
                    assert_eq!(
                        a.on_supply_loss(),
                        FailureBehaviour::Immediate { tau_s: 0.0 }
                    );
                    assert_eq!(a.gap_window_m().0, 0.0, "air pad sets down on its pads");
                }
            }
        }
    }

    /// Hybrid EMS: Diverge{Indeterminate} onto two designed stops, and an energised
    /// landing from the supercap reserve with the lower stop as the mechanical backstop.
    fn hybrid_safe_state(p: &SystemParams) {
        let s = p.design.suspension;
        let d = &p.design;
        let ad = adapter(p, PoleMaterial::Fe);
        let m = d.carriage.m_star_kg;
        let w = m * G0_MPS2;
        let pole = derive::open_loop_pole(s.g_star_m, derive::h_prime(s.h_m_m, s.mu_rec));
        match ad.on_supply_loss() {
            FailureBehaviour::Diverge { pole_rad_s, bias } => {
                assert!(rel(pole_rad_s, pole) < 1e-12);
                assert_eq!(
                    bias,
                    Bias::Indeterminate,
                    "zero-power hybrid EMS: no clamp-up claim"
                );
            }
            other => panic!("hybrid EMS must diverge, got {other:?}"),
        }
        // Indeterminate direction: F_PM > W below g*, < W above it.
        let f = |g: f64| {
            ad.force(
                &GapState {
                    z_m: g,
                    ..GapState::default()
                },
                &EddyLag::default(),
            )
            .fz_n
        };
        assert!(
            f(s.g_star_m * 0.99) > w && f(s.g_star_m * 1.01) < w,
            "F_PM ≷ W across g*"
        );
        // Both stops are designed: upper ≤ 1.30 W (LEV-006), stay-landed ≤ 0.90.
        assert!(
            derive::stop_ratio(d, s.g_up_m, m) <= 1.30 + 1e-9,
            "upper stop ≤ 1.3 W (LEV-006)"
        );
        assert!(
            derive::stop_ratio(d, s.g_land_m, d.carriage.m_op_min_kg) <= 0.90 + 1e-9,
            "stay-landed ≤ 0.9"
        );
        // Open-loop divergence from either side of g* reaches the matching stop at the
        // derive::impact_speed_at energy speed (±2 %).
        for (dir, g0) in [
            (FailDir::Up, s.g_star_m * (1.0 - 1e-6)),
            (FailDir::Down, s.g_star_m * (1.0 + 1e-6)),
        ] {
            let (stop, v) = heave_open_loop(&ad, m, g0);
            let v_ref = derive::impact_speed_at(p, dir, m, s.g_star_m);
            let expect_stop = if dir == FailDir::Up {
                s.g_up_m
            } else {
                s.g_land_m
            };
            assert_eq!(stop, expect_stop, "divergence reaches the {dir:?} stop");
            assert!(
                rel(v, v_ref) < 0.02,
                "{dir:?} impact {v:.4} vs {v_ref:.4} m/s ±2 % (PHYSICS §15)"
            );
        }
        // Energised landing from the monitored supercap reserve.
        let pw = &d.power;
        let mut cap = Supercap {
            c_f: pw.supercap_c_f,
            v_v: pw.supercap_v_max_v,
            v_min_v: pw.supercap_v_min_v,
            v_max_v: pw.supercap_v_max_v,
            esr_ohm: pw.supercap_esr_ohm,
            p_max_w: pw.p_bus_max_w.max(1.0),
        };
        let hold = derive::holdup_energy(p);
        assert!(
            hold.avail_j >= hold.need_j,
            "reserve monitor: available ≥ needed (SAF-009)"
        );
        let usable0 = cap.usable_j();
        let (landed, v_td, e_used) = energised_landing(p, &ad, &mut cap);
        println!(
            "landing m* {:.0} kg: touchdown {:.4} m/s, copper {:.2} J of {:.0} J usable (need_j {:.1} J)",
            m, v_td, e_used, usable0, hold.need_j
        );
        assert!(landed, "energised landing reaches the rollers");
        assert!(v_td <= 0.02, "controlled touchdown ≤ 20 mm/s, got {v_td}");
        assert!(e_used < usable0, "the reserve covers the landing");
    }

    /// Open-loop heave at zero current from gap `g0` at rest; returns (stop gap, speed).
    fn heave_open_loop(ad: &HybridEmsAdapter, m: f64, g0: f64) -> (f64, f64) {
        let (lo, hi) = ad.gap_window_m();
        let acc = |g: f64| {
            G0_MPS2
                - ad.force(
                    &GapState {
                        z_m: g,
                        ..GapState::default()
                    },
                    &EddyLag::default(),
                )
                .fz_n
                    / m
        };
        let (mut g, mut v) = (g0, 0.0);
        let dt = 1.0e-5;
        for _ in 0..5_000_000 {
            // RK4 on (g, ġ).
            let k1 = (v, acc(g));
            let k2 = (v + 0.5 * dt * k1.1, acc(g + 0.5 * dt * k1.0));
            let k3 = (v + 0.5 * dt * k2.1, acc(g + 0.5 * dt * k2.0));
            let k4 = (v + dt * k3.1, acc(g + dt * k3.0));
            g += dt / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0);
            v += dt / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1);
            if g <= lo {
                return (lo, v.abs());
            }
            if g >= hi {
                return (hi, v.abs());
            }
        }
        (g, v.abs())
    }

    /// Ramp landing g* → g_land + 0.2 mm (smooth-step, 0.5 s) with feed-forward bias current
    /// and pole placement at 3p, powered from `cap`; current is cut if the reserve cannot
    /// deliver. Returns (landed, touchdown speed, copper energy).
    fn energised_landing(
        p: &SystemParams,
        ad: &HybridEmsAdapter,
        cap: &mut Supercap,
    ) -> (bool, f64, f64) {
        let s = p.design.suspension;
        let d = &p.design;
        let n = ad.n_elements;
        let m = d.carriage.m_star_kg;
        let pole = derive::open_loop_pole(s.g_star_m, derive::h_prime(s.h_m_m, s.mu_rec));
        let omega = 3.0 * pole;
        let t_ramp: f64 = 0.5;
        let g_end = s.g_land_m + 2.0e-4;
        let dt = 2.0e-5;
        let (mut g, mut v, mut t, mut e) = (s.g_star_m, 0.0_f64, 0.0_f64, 0.0_f64);
        while t < 2.0 * t_ramp {
            let tau = (t / t_ramp).min(1.0);
            let g_ref = s.g_star_m + (g_end - s.g_star_m) * tau * tau * (3.0 - 2.0 * tau);
            let v_ref = if tau < 1.0 {
                (g_end - s.g_star_m) * 6.0 * tau * (1.0 - tau) / t_ramp
            } else {
                0.0
            };
            let i_ff = derive::bias_current(d, g_ref, m);
            let ks = n * derive::k_s(&s, g_ref, i_ff);
            let ki = n * derive::k_i(&s, g_ref, i_ff);
            let mut i = i_ff
                + (m * omega * omega + ks) / ki * (g - g_ref)
                + 2.0 * m * omega / ki * (v - v_ref);
            let p_req = n * s.r_coil_ohm * i * i;
            let p_got = cap.step(p_req, dt);
            if p_got < 0.999 * p_req {
                i = 0.0; // reserve exhausted: the lower stop is the mechanical backstop
            }
            e += n * s.r_coil_ohm * i * i * dt;
            let acc = |gg: f64| {
                G0_MPS2
                    - ad.force(
                        &GapState {
                            z_m: gg,
                            i_coil_a: i,
                            ..GapState::default()
                        },
                        &EddyLag::default(),
                    )
                    .fz_n
                        / m
            };
            let k1 = (v, acc(g));
            let k2 = (v + 0.5 * dt * k1.1, acc(g + 0.5 * dt * k1.0));
            let k3 = (v + 0.5 * dt * k2.1, acc(g + 0.5 * dt * k2.0));
            let k4 = (v + dt * k3.1, acc(g + dt * k3.0));
            g += dt / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0);
            v += dt / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1);
            t += dt;
            if g >= s.g_land_m {
                return (true, v.abs(), e);
            }
        }
        (false, v.abs(), e)
    }

    /// Checks one model against the trait contract at state `s` (FAB-EXO-201).
    fn contract<M: SuspensionModel>(
        m: &M,
        s: GapState,
        mut h: M::Hyst,
        kind: SuspensionKind,
        badge: Badge,
    ) {
        assert_eq!(M::KIND, kind);
        assert_eq!(M::BADGE, badge);
        let f = m.force(&s, &h);
        for v in [
            f.fz_n,
            f.fy_n,
            f.kz_n_per_m,
            f.ky_n_per_m,
            f.fx_drag_n,
            f.p_loss_w,
        ] {
            assert!(v.is_finite(), "{kind:?}: finite force/stiffness");
        }
        // kz = ∂F_z/∂z and ky = −∂F_y/∂y (central differences, rel 1e-3).
        let dz = 1.0e-7;
        let fp = m
            .force(
                &GapState {
                    z_m: s.z_m + dz,
                    ..s
                },
                &h,
            )
            .fz_n;
        let fm = m
            .force(
                &GapState {
                    z_m: s.z_m - dz,
                    ..s
                },
                &h,
            )
            .fz_n;
        let kz_fd = (fp - fm) / (2.0 * dz);
        assert!(
            (kz_fd - f.kz_n_per_m).abs() <= 1e-3 * kz_fd.abs().max(1.0),
            "{kind:?}: kz {} vs FD {kz_fd}",
            f.kz_n_per_m
        );
        let dy = 1.0e-7;
        let yp = m
            .force(
                &GapState {
                    y_m: s.y_m + dy,
                    ..s
                },
                &h,
            )
            .fy_n;
        let ym = m
            .force(
                &GapState {
                    y_m: s.y_m - dy,
                    ..s
                },
                &h,
            )
            .fy_n;
        let ky_fd = -(yp - ym) / (2.0 * dy);
        assert!(
            (ky_fd - f.ky_n_per_m).abs() <= 1e-3 * ky_fd.abs().max(1.0),
            "{kind:?}: ky {} vs FD {ky_fd}",
            f.ky_n_per_m
        );
        assert!(m.hover_power_w(&s, 0.2 * G0_MPS2) >= 0.0);
        let (lo, hi) = m.gap_window_m();
        assert!(lo < hi, "{kind:?}: gap window");
        m.step_hysteresis(&s, &mut h, 1.0e-3);
        assert!(m.force(&s, &h).fz_n.is_finite());
        // Open-loop character agrees with the local stiffness per unit weight.
        let per_w = -f.kz_n_per_m * G0_MPS2 / f.fz_n;
        match (kind, m.open_loop().z) {
            (
                SuspensionKind::HybridEms | SuspensionKind::PureEms,
                OpenLoop::Unstable { pole_rad_s },
            ) => {
                assert!(
                    rel(pole_rad_s * pole_rad_s, per_w) < 0.01,
                    "{kind:?}: p² = −kz·g/F ±1 %"
                );
            }
            (_, OpenLoop::Stable { wn_rad_s }) => {
                assert!(
                    rel(wn_rad_s * wn_rad_s, per_w) < 0.01,
                    "{kind:?}: ω² = −kz·g/F ±1 %"
                );
            }
            (k, other) => panic!("{k:?}: unexpected open loop {other:?}"),
        }
    }

    // req: FAB-EXO-201
    #[test]
    fn trait_contract_all_models() {
        let door = presets::door();
        let g_star = door.design.suspension.g_star_m;
        let at = GapState {
            z_m: g_star,
            y_m: 1.0e-4,
            ..GapState::default()
        };
        contract(
            &adapter(&door, PoleMaterial::Fe),
            at,
            EddyLag::default(),
            SuspensionKind::HybridEms,
            Badge::Research,
        );
        // FeCo is a parameter of the same adapter, not a kind.
        contract(
            &adapter(&door, PoleMaterial::FeCo),
            at,
            EddyLag::default(),
            SuspensionKind::HybridEms,
            Badge::Research,
        );
        let pure = PureEms::for_mass(1.0e4, 0.010, 100.0, 0.05, 0.004, 0.015);
        contract(
            &pure,
            GapState {
                z_m: 0.010,
                i_coil_a: pure.hover_current_a(0.010),
                ..GapState::default()
            },
            (),
            SuspensionKind::PureEms,
            Badge::Simplified,
        );
        let hts = HtsBulk::atz_demonstrator();
        let loaded = load_to(&hts, hts.z_work_m);
        let hs = GapState {
            z_m: hts.z_work_m,
            y_m: 1.0e-4,
            ..GapState::default()
        };
        // The HTS open-loop ω uses the first-loading load; the minor-loop kz is at the
        // frozen reference of that loaded state.
        contract(&hts, hs, loaded, SuspensionKind::HtsBulk, Badge::Research);
        let trim = HtsEmsTrim {
            hts,
            k_i_n_per_a: 200.0,
            r_coil_ohm: 0.5,
            i_max_a: 20.0,
        };
        contract(
            &trim,
            hs,
            loaded,
            SuspensionKind::HybridHtsPlusEms,
            Badge::Research,
        );
        let pad = AirPad {
            lift_n: 1.0e5,
            pump_w: 2.0e3,
            lift_time_s: 180.0,
            fly_height_m: 1.0e-4,
        };
        contract(
            &pad,
            GapState {
                z_m: 1.0e-4,
                ..GapState::default()
            },
            (),
            SuspensionKind::AirPad,
            Badge::Simplified,
        );
        // Failure behaviours per kind.
        assert!(matches!(
            adapter(&door, PoleMaterial::Fe).on_supply_loss(),
            FailureBehaviour::Diverge {
                bias: Bias::Indeterminate,
                ..
            }
        ));
        assert!(matches!(
            pure.on_supply_loss(),
            FailureBehaviour::Immediate { .. }
        ));
        assert!(matches!(
            hts.on_supply_loss(),
            FailureBehaviour::SlowFade { .. }
        ));
        assert!(matches!(
            trim.on_supply_loss(),
            FailureBehaviour::SlowFade { .. }
        ));
        assert!(matches!(
            pad.on_supply_loss(),
            FailureBehaviour::Immediate { .. }
        ));
    }

    #[test]
    fn hybrid_eddy_lag_first_order() {
        // τ_e·Φ̇_eff = Φ − Φ_eff (PHYSICS §2): after one τ_e the force flux has covered
        // 1 − e⁻¹ of a current step (exact update, rel 1e-9).
        let door = presets::door();
        let ad = adapter(&door, PoleMaterial::Fe);
        let tau = door.design.suspension.tau_eddy_s;
        assert!(tau > 0.0);
        let s0 = GapState {
            z_m: door.design.suspension.g_star_m,
            ..GapState::default()
        };
        let mut h = EddyLag::default();
        ad.step_hysteresis(&s0, &mut h, 1.0e-4);
        let phi0 = h.phi_frac * ad.flux_ref_wb();
        let s1 = GapState {
            i_coil_a: 1.0,
            ..s0
        };
        let phi1 = ad.element(s1.z_m, 0.0, 1.0, 0.0).flux_wb;
        let n = 100;
        for _ in 0..n {
            ad.step_hysteresis(&s1, &mut h, tau / n as f64);
        }
        let phi = h.phi_frac * ad.flux_ref_wb();
        let frac = (phi - phi0) / (phi1 - phi0);
        assert!(
            rel(frac, 1.0 - (-1.0_f64).exp()) < 1e-9,
            "1 − e⁻¹ after τ_e: {frac}"
        );
        // The lagged force sits between the old and new instantaneous forces.
        let f_lag = ad.force(&s1, &h).fz_n;
        let f_new = ad.force(&s1, &EddyLag::default()).fz_n;
        let f_old = ad.force(&s0, &EddyLag::default()).fz_n;
        assert!(f_old < f_lag && f_lag < f_new);
    }

    // req: FAB-EXO-202
    #[test]
    fn hts_fit_atz() {
        let atz = HtsBulk::atz_demonstrator();
        let p20 = (0.020, 300.0 * KGF_N);
        let p15 = (0.015, 550.0 * KGF_N);
        let fit = atz.fit_first_loading(p20, p15).expect("ATZ points bracket");
        assert!(
            rel(fit.f0_n, atz.f0_n) < 1e-6,
            "demonstrator F0 = fit ({} vs {})",
            fit.f0_n,
            atz.f0_n
        );
        assert!(rel(fit.k_per_m, atz.k_per_m) < 1e-6, "demonstrator k = fit");
        // Driven along the first-loading branch by the play operator: ±1 % at both points.
        let f20 = fz_at(&atz, &load_to(&atz, 0.020), 0.020);
        let f15 = fz_at(&atz, &load_to(&atz, 0.015), 0.015);
        println!(
            "HTS fit: {:.1} kgf @20 mm, {:.1} kgf @15 mm; λ = {:.1} mm",
            f20 / KGF_N,
            f15 / KGF_N,
            2.0 * core::f64::consts::PI / atz.k_per_m * 1e3
        );
        assert!(
            rel(f20, p20.1) <= 0.01,
            "300 kgf at 20 mm ±1 % (FAB-EXO-202, ATZ 2019): {:.1} kgf",
            f20 / KGF_N
        );
        assert!(
            rel(f15, p15.1) <= 0.01,
            "550 kgf at 15 mm ±1 % (FAB-EXO-202, ATZ 2019): {:.1} kgf",
            f15 / KGF_N
        );
        // The conservative (η_h = 0) fit is the exotic design's seed: F0 32.9 kN, k 40.85 m⁻¹
        // (±0.1 %), kz(10 mm) 974 N/mm and ky(10 mm) 214 N/mm (±1 %) → 5.0 Hz / 2.3 Hz for 1 t.
        let cons = HtsBulk { eta_h: 0.0, ..atz }
            .fit_first_loading(p20, p15)
            .expect("bracket");
        assert!(
            rel(cons.f0_n, 32.9e3) < 1e-3 && rel(cons.k_per_m, 40.85) < 1e-3,
            "η_h = 0 fit = design seed"
        );
        let f = cons.force(
            &GapState {
                z_m: 0.010,
                ..GapState::default()
            },
            &cons.field_cool(0.0),
        );
        assert!(
            rel(-f.kz_n_per_m, 974.0e3) < 0.01,
            "kz(10 mm) 974 N/mm ±1 % (exotic design §6.1)"
        );
        assert!(
            rel(f.ky_n_per_m, 214.0e3) < 0.01,
            "ky(10 mm) 214 N/mm ±1 % (exotic design §6.1)"
        );
        let fz_hz = (-f.kz_n_per_m / 1000.0).sqrt() / (2.0 * core::f64::consts::PI);
        let fy_hz = (f.ky_n_per_m / 1000.0).sqrt() / (2.0 * core::f64::consts::PI);
        assert!(
            (fz_hz - 5.0).abs() < 0.05 && (fy_hz - 2.3).abs() < 0.05,
            "5.0 / 2.3 Hz for 1 t"
        );
    }

    /// Force along a 35 → 10 → 35 mm loop on a shared grid; returns (approach, return).
    fn hts_loop(h: &HtsBulk, dz: f64) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let n = ((h.z_fc_m - 0.010) / dz).round() as usize;
        let grid: Vec<f64> = (0..=n)
            .map(|i| 0.010 + (h.z_fc_m - 0.010) * i as f64 / n as f64)
            .collect();
        let mut st = h.field_cool(0.0);
        let mut approach = vec![0.0; n + 1];
        for i in (0..=n).rev() {
            let s = GapState {
                z_m: grid[i],
                ..GapState::default()
            };
            h.step_hysteresis(&s, &mut st, 0.0);
            approach[i] = h.force(&s, &st).fz_n;
        }
        let mut ret = vec![0.0; n + 1];
        for i in 0..=n {
            let s = GapState {
                z_m: grid[i],
                ..GapState::default()
            };
            h.step_hysteresis(&s, &mut st, 0.0);
            ret[i] = h.force(&s, &st).fz_n;
        }
        (grid, approach, ret)
    }

    /// Work done on the bulk over the loop, −∮F_z dz (approach then return), J.
    fn loop_work(grid: &[f64], approach: &[f64], ret: &[f64]) -> f64 {
        let mut w_in = 0.0;
        let mut w_out = 0.0;
        for i in (0..grid.len() - 1).rev() {
            w_in += 0.5 * (approach[i] + approach[i + 1]) * (grid[i + 1] - grid[i]);
        }
        for i in 0..grid.len() - 1 {
            w_out += 0.5 * (ret[i] + ret[i + 1]) * (grid[i + 1] - grid[i]);
        }
        w_in - w_out
    }

    // req: FAB-EXO-203
    #[test]
    fn hts_loop_conservative_and_dissipative() {
        let atz = HtsBulk::atz_demonstrator();
        // η_h = 0: conservative, |∮F dz| < 1e-9 J (FAB-EXO-203).
        let cons = HtsBulk { eta_h: 0.0, ..atz };
        let (g, a, r) = hts_loop(&cons, 1.0e-5);
        let w0 = loop_work(&g, &a, &r);
        assert!(w0.abs() < 1e-9, "η_h = 0 loop |∮F dz| = {w0:e} J < 1e-9 J");
        // η_h > 0: dissipative (∮ > 0 as work done on the bulk), return branch below approach.
        for eta in [0.1, 0.2, 0.3] {
            let h = HtsBulk { eta_h: eta, ..atz };
            let (g, a, r) = hts_loop(&h, 1.0e-5);
            let w = loop_work(&g, &a, &r);
            println!("HTS loop η_h {eta}: dissipated {w:.3} J per 35→10→35 mm cycle");
            assert!(w > 0.0, "η_h {eta}: dissipative loop");
            for i in 1..g.len() {
                assert!(
                    r[i] < a[i],
                    "η_h {eta}: return below approach at z = {:.2} mm",
                    g[i] * 1e3
                );
            }
            // Rate independence: halving the resolution changes the loop by < 1 %.
            let (g2, a2, r2) = hts_loop(&h, 2.0e-5);
            assert!(
                rel(loop_work(&g2, &a2, &r2), w) < 0.01,
                "play loop is rate independent"
            );
        }
    }

    // req: FAB-EXO-204
    #[test]
    fn hts_creep_and_ln2_hold() {
        let h = HtsBulk::atz_demonstrator();
        // Creep law (E): s_c = 1 − S·log10(1 + t/τ0).
        assert_eq!(h.s_c(0.0), 1.0);
        assert!(
            rel(h.s_c(9.0 * h.creep_tau0_s), 1.0 - h.creep_per_decade) < 1e-12,
            "one decade"
        );
        let day = 86_400.0;
        let expect = 1.0 - 0.03 * (1.0 + day / 5.0_f64).log10();
        assert!(rel(h.s_c(day), expect) < 1e-12);
        // Creep scales the force (quasi-static).
        let st = load_to(&h, 0.010);
        let aged = HtsState {
            t_since_fc_s: day,
            ..st
        };
        assert!(rel(fz_at(&h, &aged, 0.010), expect * fz_at(&h, &st, 0.010)) < 1e-12);
        // s_T law: 1 at 77 K, e.g. 0.386 at 85 K (j = 0.24), 0 at Tc, below the cap 2.
        assert_eq!(h.s_t(T_LN2_K), 1.0);
        assert!(
            (h.s_t(85.0) - 0.3856).abs() < 1e-3,
            "s_T(85 K) ≈ 0.386 (exotic design §2.3)"
        );
        assert_eq!(h.s_t(h.tc_k), 0.0);
        assert_eq!(h.s_t(100.0), 0.0);
        let mut prev = f64::INFINITY;
        for t in (0..92).map(f64::from) {
            let v = h.s_t(t);
            assert!(v <= prev && v < HTS_ST_CAP, "s_T monotone, below the cap");
            prev = v;
        }
        let warm = HtsState {
            bulk_temp_k: 85.0,
            ..st
        };
        assert!(rel(fz_at(&h, &warm, 0.010), h.s_t(85.0) * fz_at(&h, &st, 0.010)) < 1e-12);
        // LN₂: f_use·V·ρ·h_fg/Q̇ = 0.9 × 2.5 L × 807 × 199 kJ / 2.5 W = 40.1 h.
        assert!(
            rel(h.reservoir_s, h.ln2_hold_s(0.0)) < 1e-12,
            "reservoir_s is the derived hold"
        );
        for (activity, v, hours) in [(0.0, 0.0, 40.0), (1.0, 1.0, 30.0)] {
            let mut s = h.field_cool(0.0);
            let g = GapState {
                z_m: 0.010,
                xdot_mps: v,
                ..GapState::default()
            };
            let dt = 60.0;
            let mut t = 0.0;
            while s.ln2_frac > 0.0 && t < 100.0 * 3600.0 {
                h.step_hysteresis(&g, &mut s, dt);
                t += dt;
            }
            println!(
                "LN2 hold at activity {activity}: {:.2} h (heat {:.2} W)",
                t / 3600.0,
                h.heat_load_w(activity)
            );
            assert!(
                rel(t / 3600.0, hours) <= 0.10,
                "LN₂ hold {hours} h ±10 % (FAB-EXO-204, ATZ 2019): {:.2} h",
                t / 3600.0
            );
        }
        // The fitted dynamic load stays in the ATZ 2–4 W per cryostat range.
        assert!((2.0..=4.0).contains(&h.q_dynamic_w));
    }

    // req: FAB-EXO-205
    #[test]
    fn hts_ln2_landing_and_fade() {
        let h = HtsBulk::atz_demonstrator();
        let status = |f: f64| {
            h.ln2_status(&HtsState {
                ln2_frac: f,
                ..HtsState::default()
            })
        };
        assert_eq!(status(0.5), Ln2Status::Ok);
        assert_eq!(status(0.10), Ln2Status::Land);
        assert_eq!(status(0.04), Ln2Status::Critical);
        assert_eq!(status(0.0), Ln2Status::Depleted);
        match h.on_supply_loss() {
            FailureBehaviour::SlowFade {
                reservoir_s,
                fade_tau_s,
                land_trigger_frac,
            } => {
                assert_eq!(land_trigger_frac, 0.10, "land at 10 % LN₂ (FAB-EXO-205)");
                assert!(rel(reservoir_s, h.ln2_hold_s(0.0)) < 1e-12);
                assert!(
                    rel(fade_tau_s, h.fade_time_s()) < 1e-6,
                    "fade_tau_s = derived fade time"
                );
            }
            other => panic!("HTS fades slowly, got {other:?}"),
        }
        // Static depletion run: the land trigger precedes empty by ≈ 10 % of the hold.
        let mut st = load_to(&h, 0.010);
        st.bulk_temp_k = T_LN2_K;
        let g = GapState {
            z_m: 0.010,
            ..GapState::default()
        };
        let dt = 10.0;
        let (mut t, mut t_land, mut t_crit) = (0.0, None, None);
        while st.ln2_frac > 0.0 {
            h.step_hysteresis(&g, &mut st, dt);
            t += dt;
            match h.ln2_status(&st) {
                Ln2Status::Land if t_land.is_none() => t_land = Some(t),
                Ln2Status::Critical if t_crit.is_none() => t_crit = Some(t),
                _ => {}
            }
        }
        let (t_land, t_crit) = (t_land.expect("land trigger"), t_crit.expect("critical"));
        println!(
            "LN2: land at {:.2} h, critical at {:.2} h, empty at {:.2} h",
            t_land / 3600.0,
            t_crit / 3600.0,
            t / 3600.0
        );
        assert!(rel(t_land / t, 0.90) < 0.01 && rel(t_crit / t, 0.96) < 0.01);
        // Post-depletion fade: tens of minutes, not hours.
        let f_dep = fz_at(&h, &st, 0.010);
        let t_dep = t;
        let mut t_e = None;
        while t - t_dep < 4.0 * 3600.0 {
            h.step_hysteresis(&g, &mut st, 1.0);
            t += 1.0;
            if t_e.is_none() && fz_at(&h, &st, 0.010) <= f_dep * (-1.0_f64).exp() {
                t_e = Some(t - t_dep);
            }
        }
        let t_e = t_e.expect("lift fades after depletion");
        println!(
            "fade to e⁻¹ in {:.1} min (bulk at {:.1} K after 4 h)",
            t_e / 60.0,
            st.bulk_temp_k
        );
        assert!(
            (600.0..=3600.0).contains(&t_e),
            "fade in tens of minutes, not hours: {t_e:.0} s"
        );
        assert!(
            rel(t_e, h.fade_time_s()) < 0.01,
            "simulated fade = (T_e − 77 K)·C/Q̇ ±1 %"
        );
        assert_eq!(fz_at(&h, &st, 0.010), 0.0, "above Tc the lift is gone");
    }

    // req: FAB-EXO-206
    #[test]
    fn hover_loss_anchors() {
        let m = 10.0e6;
        // Pure EMS 1.7 kW/t (E), independent of the vertical acceleration RMS.
        assert!(
            rel(pure_ems_hover_power_w(m), 17.0e6) < 1e-12,
            "1.7 kW/t (EXOTIC §X4)"
        );
        let pure = PureEms::for_mass(m, 0.010, 1000.0, 0.1, 0.004, 0.015);
        let s = GapState {
            z_m: 0.010,
            i_coil_a: pure.hover_current_a(0.010),
            ..GapState::default()
        };
        assert!(
            rel(pure.force(&s, &()).fz_n, m * G0_MPS2) < 1e-12,
            "hover current holds the weight"
        );
        for a in [0.1, 0.2, 0.5] {
            assert!(rel(pure.hover_power_w(&s, a * G0_MPS2), 17.0e6) < 1e-12);
        }
        // Hybrid EMS at 10 kt: 42 / 170 / 1,060 kW at 0.1 / 0.2 / 0.5 g RMS (anchors quoted
        // to 2–3 significant figures: ±1.5 %).
        for (a_g, kw) in [(0.1, 42.0), (0.2, 170.0), (0.5, 1060.0)] {
            let p = hybrid_hover_power_w(m, a_g * G0_MPS2);
            println!("hybrid 10 kt at {a_g} g RMS: {:.1} kW", p / 1e3);
            assert!(rel(p / 1e3, kw) <= 0.015, "{kw} kW ±1.5 % (FAB-EXO-206)");
        }
        // The adapter, scaled to 10 kt of DOOR elements at zero power, reports the same.
        let door = presets::door();
        let sd = door.design.suspension;
        let f1 = magnet::element_force(&sd, sd.b_r_t, sd.g_star_m, 0.0, 0.0, 1.0).f_attract_n;
        let ad = HybridEmsAdapter::new(sd, m * G0_MPS2 / f1, PoleMaterial::Fe);
        let s0 = GapState {
            z_m: sd.g_star_m,
            ..GapState::default()
        };
        assert!(rel(ad.zero_power_mass_kg(&s0), m) < 1e-9);
        assert!(
            rel(
                ad.hover_power_w(&s0, 0.2 * G0_MPS2),
                hybrid_hover_power_w(m, 0.2 * G0_MPS2)
            ) < 1e-9
        );
        // A static current adds n·R·i².
        let s1 = GapState {
            i_coil_a: 0.5,
            ..s0
        };
        let extra = ad.hover_power_w(&s1, 0.0) - 0.0;
        assert!(rel(extra, ad.n_elements * sd.r_coil_ohm * 0.25) < 1e-9);
    }

    // req: FAB-EXO-207
    #[test]
    fn feco_pressure_and_rail_cap() {
        // P = B²/2μ0: FeCo 2.3 T → 2.10 MPa (EXOTIC §X2; ±0.5 % for the rounded anchor).
        assert!(
            rel(magnetic_pressure_pa(B_SAT_FECO_T), 2.10e6) < 5e-3,
            "FeCo 2.10 MPa"
        );
        // The rail saturates too: 1.6 / 2.0 / 2.15 T → 1.02 / 1.59 / 1.84 MPa (exotic critique).
        for (b, mpa) in [(1.6, 1.02), (2.0, 1.59), (2.15, 1.84)] {
            assert!(
                rel(magnetic_pressure_pa(b) / 1e6, mpa) < 5e-3,
                "rail {b} T → {mpa} MPa"
            );
        }
        // Admission: FeCo poles on a steel rail gain nothing when the rail governs.
        let m = 1.0e6;
        assert_eq!(
            admission_area_m2(m, ADMISSION_K, 0.5, B_SAT_FECO_T, B_RAIL_STEEL_T, 2.5),
            admission_area_m2(m, ADMISSION_K, 0.5, B_RAIL_STEEL_T, B_RAIL_STEEL_T, 2.5),
            "min rule: the rail caps FeCo"
        );
        assert!(
            admission_area_m2(m, ADMISSION_K, 0.5, B_SAT_FECO_T, B_RAIL_FECO_T, 2.5)
                < admission_area_m2(m, ADMISSION_K, 0.5, B_SAT_FE_T, B_RAIL_FECO_T, 2.5),
            "FeCo helps only with a FeCo rail"
        );
        // Element level: the adapter saturates at min(B_pole, B_rail).
        let door = presets::door();
        let steel = door.design.suspension;
        let mut feco_rail = steel;
        feco_rail.b_sat_t = B_RAIL_FECO_T;
        let n = 4.0;
        let fe = HybridEmsAdapter::new(steel, n, PoleMaterial::Fe);
        let feco_on_steel = HybridEmsAdapter::new(steel, n, PoleMaterial::FeCo);
        let feco_on_feco = HybridEmsAdapter::new(feco_rail, n, PoleMaterial::FeCo);
        let fe_on_feco = HybridEmsAdapter::new(feco_rail, n, PoleMaterial::Fe);
        assert_eq!(feco_on_steel.b_sat_eff_t(), steel.b_sat_t);
        assert_eq!(feco_on_feco.b_sat_eff_t(), B_RAIL_FECO_T);
        assert_eq!(fe_on_feco.b_sat_eff_t(), B_SAT_FE_T);
        // Deep in saturation (large current at the upper stop).
        let s = GapState {
            z_m: steel.g_up_m,
            i_coil_a: 20.0 * steel.i_pulse_a.max(1.0),
            ..GapState::default()
        };
        let phi_sat_fe = B_SAT_FE_T * steel.pole_w_m * steel.pole_l_m;
        let phi = fe.element(s.z_m, 0.0, s.i_coil_a, 0.0).flux_wb;
        assert!(phi > 0.9 * phi_sat_fe, "operating point is saturated");
        let f_fe = fe.force(&s, &EddyLag::default()).fz_n;
        assert_eq!(
            feco_on_steel.force(&s, &EddyLag::default()).fz_n,
            f_fe,
            "FeCo poles on a steel rail: no gain"
        );
        assert!(
            feco_on_feco.force(&s, &EddyLag::default()).fz_n > 1.3 * f_fe,
            "FeCo on a FeCo rail lifts more"
        );
    }

    // req: FAB-EXO-208
    #[test]
    fn hts_ems_trim_creep_drift() {
        let hts = HtsBulk::atz_demonstrator();
        let trim = HtsEmsTrim {
            hts,
            k_i_n_per_a: 200.0,
            r_coil_ohm: 0.5,
            i_max_a: 20.0,
        };
        let z_t = 0.010;
        let loaded = load_to(&hts, z_t);
        // The load is what the fresh, field-cooled set carries at 10 mm.
        let w = fz_at(&hts, &loaded, z_t);
        let run = |with_trim: bool| -> (f64, f64, f64) {
            let mut st = loaded;
            let (mut z, mut i, mut integ) = (z_t, 0.0_f64, 0.0_f64);
            let dt = 1.0;
            let kz0 = -hts
                .force(
                    &GapState {
                        z_m: z_t,
                        ..GapState::default()
                    },
                    &st,
                )
                .kz_n_per_m;
            let tau = 10.0;
            let ki_gain = kz0 / (trim.k_i_n_per_a * tau);
            let (mut max_dev, mut max_i) = (0.0_f64, 0.0_f64);
            let mut t = 0.0;
            while t < 86_400.0 {
                // Trim loop: integral on the height error (quasi-static creep test).
                if with_trim {
                    integ += (z_t - z) * dt;
                    i = trim.trim_current_a(ki_gain * integ);
                }
                let s_now = GapState {
                    z_m: z,
                    i_coil_a: i,
                    ..GapState::default()
                };
                trim.step_hysteresis(&s_now, &mut st, dt);
                t += dt;
                // Quasi-static equilibrium: F_HTS(z) + k_i·i = W (Newton on z).
                for _ in 0..20 {
                    let f = trim.force(
                        &GapState {
                            z_m: z,
                            i_coil_a: i,
                            ..GapState::default()
                        },
                        &st,
                    );
                    let r = f.fz_n - w;
                    z -= r / f.kz_n_per_m;
                    if r.abs() < 1e-9 * w {
                        break;
                    }
                }
                max_dev = max_dev.max((z - z_t).abs());
                max_i = max_i.max(i.abs());
            }
            (max_dev, (z - z_t).abs(), max_i)
        };
        let (free_max, free_end, _) = run(false);
        let (trim_max, trim_end, i_max) = run(true);
        println!(
            "24 h creep: free sag {:.3} mm, with EMS trim max {:.3} mm (end {:.4} mm, i ≤ {:.2} A)",
            free_end * 1e3,
            trim_max * 1e3,
            trim_end * 1e3,
            i_max
        );
        assert!(
            free_max > 0.5e-3,
            "creep alone drifts > 0.5 mm over 24 h (the trim is needed)"
        );
        assert!(
            trim_max <= 0.5e-3,
            "HTS + EMS trim holds creep drift ≤ 0.5 mm over 24 h (FAB-EXO-208): {:.3} mm",
            trim_max * 1e3
        );
        assert!(i_max < trim.i_max_a, "trim current within its limit");
    }

    // req: FAB-EXO-209
    #[test]
    fn baselines_available() {
        // Air pad (Citicorp-type lift in 3 min): constant lift, pump power, stable film.
        let pad = AirPad {
            lift_n: 4.0e6,
            pump_w: 30.0e3,
            lift_time_s: 180.0,
            fly_height_m: 1.0e-4,
        };
        let s = GapState {
            z_m: pad.fly_height_m,
            ..GapState::default()
        };
        assert_eq!(pad.force(&s, &()).fz_n, pad.lift_n);
        assert_eq!(pad.hover_power_w(&s, 0.3 * G0_MPS2), pad.pump_w);
        assert!(matches!(pad.open_loop().z, OpenLoop::Stable { .. }));
        assert_eq!(<AirPad as SuspensionModel>::KIND, SuspensionKind::AirPad);
        // FPS: T = 2π√(R/g), activation μ·g, friction opposes motion.
        let fps = FrictionPendulum {
            r_m: guideway_r(3.0),
            mu: 0.05,
        };
        assert!(rel(fps.period_s(), 3.0) < 1e-12, "FPS period (E)");
        assert!(rel(fps.activation_mps2(), 0.05 * G0_MPS2) < 1e-12);
        let f = fps.force_n(1.0e6, [0.1, 0.0], [0.2, 0.0]);
        assert!(
            f[0] < -1.0e6 / fps.r_m * 0.1,
            "restoring plus friction against +v"
        );
        // Hover-power comparison per tonne (reported): air pad vs hybrid vs pure EMS.
        let m = pad.lift_n / G0_MPS2;
        println!(
            "baseline per tonne: air pad {:.0} W, hybrid EMS at 0.1 g {:.0} W, pure EMS {:.0} W",
            pad.pump_w / (m / 1e3),
            hybrid_hover_power_w(m, 0.1 * G0_MPS2) / (m / 1e3),
            pure_ems_hover_power_w(m) / (m / 1e3)
        );
    }

    fn guideway_r(t: f64) -> f64 {
        crate::platform::guideway_radius_m(t)
    }
}
