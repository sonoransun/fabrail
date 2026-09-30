//! Exotic scenarios XS01..XS12 and XC-* with their pass criteria. Owner: X6 (API frozen by W0a).
//!
//! Exotic design §6.2 as corrected by EXOTIC §X17 (prefixed XS/XC to avoid collisions
//! with core scenario names). Each scenario runs a **reduced model** of its class,
//! labelled as such ([`ScenarioResult::model`]), prints every input it used
//! ([`ScenarioResult::inputs`]), reports KPIs and evaluates its pass criteria one by one
//! ([`ScenarioResult::criteria`]). Frontier items never run here (numpy only); SELENE
//! has no scenario.
//!
//! Everything here is simulation truth for the research/sovereign tier (🧪/🔶): nothing
//! claims hardware verification, certification or standards compliance, and dynamic
//! simulation of ARENA, ROTATOR, TIDEGATE and full HAVEN-F is roadmap (their scenarios are
//! static, kinematic or timing models, `reduced_model = true`).
//!
//! | Scenario | Classes | Model |
//! |---|---|---|
//! | XS01 near-fault pulse | SENTINEL, HAVEN-R | 2-DOF host + HMD (LQR → SEISMIC_PASSIVE) / 3-DOF platform + skyhook |
//! | XS02 fling | HAVEN-R | 3-DOF platform, progressive buffer and moat wall, FPS landing |
//! | XS03 M9 | HAVEN-R | 3-DOF platform, skyhook LSM on a supercap bus, energy ledgers |
//! | XS04 basin | SENTINEL | 2-DOF, SEISMIC_PASSIVE with the stroke governor |
//! | XS05 vertical pulse | HAVEN-R | vertical 1-DOF: air springs + saturating hybrid-EMS trim |
//! | XS06 grid loss | HAVEN-R | 3-DOF platform on islanded storage, PM-bias-ramp landing |
//! | XS07 controller/LSM fault | SENTINEL, HAVEN-R | mode ladder to LANDED_FPS / control loss to FPS |
//! | XS08 cancel / false alarm | all but SELENE | EEW fusion + alert gate |
//! | XS09 offshore M8 | all but SELENE | `eew_timing` wavefield timing simulation |
//! | XS10 generic blast | BASTION, TIDEGATE | closure-race planner + zone machine + S-curve |
//! | XS11 tsunami | TIDEGATE | `tsunami` timing, rail distortion, gap check |
//! | XS12 wind | SENTINEL | 2-DOF, Davenport wind, passive and LQR active |
//! | XC-RED-1/2 | all but SELENE | one hybrid-EMS pod heave loop behind the 2oo3 voter and ladder |
//! | XC-SAF-1 | ARENA, BASTION | zone machine + jerk-limited SS1 |
//! | XC-ARN-1 | ARENA | static WLS pod allocation (500 pods) |
//! | XC-SHD-1 | SHIELD | stop, PM-bias-ramp landing and lock timeline |
//! | XC-HTS-1 | HAVEN-R (HTS option) | quasi-static HTS module over the LN₂ depletion |
//! | XC-ROT-1 | ROTATOR | static per-station uplift, ovalisation and EMS admission |
//!
//! The HMD gains come from a discrete Riccati solution computed here (`sentinel_lqr`,
//! the same DARE scipy's `solve_discrete_are` solves) with an explicit weight on the
//! stroke; ζ is normalised to ω_s (`fabrail_exo::tuning::damper_design_for`).

use std::collections::BTreeMap;
use std::fmt;

use fabrail_exo::alloc::{self, Pod, Wrench};
use fabrail_exo::closure::{self, ClosureDecision, ClosureInputs};
use fabrail_exo::energy;
use fabrail_exo::fusion::{Feed, Fusion, FusionCfg, LocalDetection};
use fabrail_exo::hmd::{Hmd, HmdGains, HmdInputs, HmdLimits, HmdMode, SAT_STROKE};
use fabrail_exo::isolation::{IsolationCfg, IsolationInputs, Isolator};
use fabrail_exo::tuning::{DamperDesign, TuningForm, damper_design_for, dvf_damping_n_s_per_m};
use fabrail_exo_safety::alert_gate::{AlertAction, Confirmations, GateClass, gate};
use fabrail_exo_safety::ladder::{Ladder, LadderInputs, Redundancy};
use fabrail_exo_safety::presets;
use fabrail_exo_safety::vote::{Voter2oo3, VoterCfg};
use fabrail_exo_safety::zone::{ZoneInputs, ZoneMachine, ZoneState, ss1_stop_distance_mm};
use fabrail_exo_safety::{AlertFeedKind, ExoClass, SafeStatePolicy, SafetyRank, SafetyRegime};
use fabrail_params::derive::G0_MPS2;
use fabrail_sim::rng::{NormalStream, Xoshiro256pp};
use fabrail_types::{SeismicState, Tick, fmath};

use crate::eew_timing::{self, OffshoreCase, Sensing, SimSettings};
use crate::ground::{self, BasinResonance, FlingStep, NearFaultPulse};
use crate::host::{self, Damper, DavenportWind, Primary, RailCondition, TwoDof};
use crate::platform::{self, HavenR3Dof, PlatformCfg};
use crate::storage::{self, EnergyLedger, EnergyLedgerMj, Supercap};
use crate::suspension::{
    self, Bias, FailureBehaviour, GapState, HtsBulk, HtsState, HybridEmsAdapter, Ln2Status,
    SuspensionModel,
};
use crate::tsunami::{self, CloseMode, RailDistortion, TidegateInputs};

const TAU: f64 = core::f64::consts::TAU;

/// An exotic scenario.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum ExoScenarioId {
    /// Kobe-type near-fault pulse.
    Xs01,
    /// Chi-Chi fling, 5–9 m.
    Xs02,
    /// M9, 150–300 s.
    Xs03,
    /// Basin resonance > 10 min.
    Xs04,
    /// 1 g vertical pulse.
    Xs05,
    /// Grid loss at S arrival.
    Xs06,
    /// Controller or LSM fault.
    Xs07,
    /// Cancelled alert; concurrent-quake false alarm.
    Xs08,
    /// Offshore M8: land / S-net / DAS.
    Xs09,
    /// Generic blast (Ps, t_a as inputs).
    Xs10,
    /// Tsunami with 0.3–1.2 m subsidence.
    Xs11,
    /// Wind, 3–7 mg.
    Xs12,
    /// Single sensor faults (all types).
    XcRed1,
    /// β common-cause double fault.
    XcRed2,
    /// Intrusion mid-move (ARENA, BASTION).
    XcSaf1,
    /// 1 of 500 pods lost.
    XcArn1,
    /// P-wave during transit (SHIELD).
    XcShd1,
    /// LN₂ depletion (HTS).
    XcHts1,
    /// Wind uplift plus 10 mm ovalisation (ROTATOR).
    XcRot1,
}

impl ExoScenarioId {
    /// Every scenario, in catalogue order.
    pub const ALL: [ExoScenarioId; 19] = [
        ExoScenarioId::Xs01,
        ExoScenarioId::Xs02,
        ExoScenarioId::Xs03,
        ExoScenarioId::Xs04,
        ExoScenarioId::Xs05,
        ExoScenarioId::Xs06,
        ExoScenarioId::Xs07,
        ExoScenarioId::Xs08,
        ExoScenarioId::Xs09,
        ExoScenarioId::Xs10,
        ExoScenarioId::Xs11,
        ExoScenarioId::Xs12,
        ExoScenarioId::XcRed1,
        ExoScenarioId::XcRed2,
        ExoScenarioId::XcSaf1,
        ExoScenarioId::XcArn1,
        ExoScenarioId::XcShd1,
        ExoScenarioId::XcHts1,
        ExoScenarioId::XcRot1,
    ];

    /// Catalogue name (`"XS01"`, `"XC-RED-1"`, …).
    pub const fn name(self) -> &'static str {
        match self {
            ExoScenarioId::Xs01 => "XS01",
            ExoScenarioId::Xs02 => "XS02",
            ExoScenarioId::Xs03 => "XS03",
            ExoScenarioId::Xs04 => "XS04",
            ExoScenarioId::Xs05 => "XS05",
            ExoScenarioId::Xs06 => "XS06",
            ExoScenarioId::Xs07 => "XS07",
            ExoScenarioId::Xs08 => "XS08",
            ExoScenarioId::Xs09 => "XS09",
            ExoScenarioId::Xs10 => "XS10",
            ExoScenarioId::Xs11 => "XS11",
            ExoScenarioId::Xs12 => "XS12",
            ExoScenarioId::XcRed1 => "XC-RED-1",
            ExoScenarioId::XcRed2 => "XC-RED-2",
            ExoScenarioId::XcSaf1 => "XC-SAF-1",
            ExoScenarioId::XcArn1 => "XC-ARN-1",
            ExoScenarioId::XcShd1 => "XC-SHD-1",
            ExoScenarioId::XcHts1 => "XC-HTS-1",
            ExoScenarioId::XcRot1 => "XC-ROT-1",
        }
    }

    /// Pass criteria, verbatim from the EXOTIC §X17 scenario catalogue (the corrected
    /// exotic design §6.2: XS05 without a clamp-up claim, XS06 with T_ride, XC-ARN-1
    /// static reallocation only).
    pub const fn criteria(self) -> &'static str {
        match self {
            ExoScenarioId::Xs01 => {
                "HAVEN stroke within the moat with no buffer contact; SENTINEL host peak ≤ 1.02× uncontrolled"
            }
            ExoScenarioId::Xs02 => {
                "Exceedance expected; peak deceleration within design, bumper energy quoted in MJ; support never lost; stable landing"
            }
            ExoScenarioId::Xs03 => {
                "Energy balance within 1 %; copper ΔT within its limit; SOC > 0; no FPS fallback"
            }
            ExoScenarioId::Xs04 => {
                "The stroke governor engages before the end stops; ζ_add within ±20 % of the bound"
            }
            ExoScenarioId::Xs05 => {
                "EMS saturation detected; the passive vertical support carries the load; supply-loss direction indeterminate onto designed stops"
            }
            ExoScenarioId::Xs06 => {
                "Ride-through for T_ride on islanded storage; energised, controlled descent"
            }
            ExoScenarioId::Xs07 => {
                "Degrades to a friction-pendulum TMD or FPS isolation; impact within the buffer rating"
            }
            ExoScenarioId::Xs08 => "Returns to NORMAL; cost within the cost–loss bound",
            ExoScenarioId::Xs09 => "Warning within ±1 s of the analytic value",
            ExoScenarioId::Xs10 => "Closed before t_a, or marked infeasible",
            ExoScenarioId::Xs11 => "Closes within the budget, or falls back to wheels",
            ExoScenarioId::Xs12 => "Activation ≤ 0.3 mg; RMS reduction reported against ζ_s",
            ExoScenarioId::XcRed1 => "No loss of levitation",
            ExoScenarioId::XcRed2 => "Safe state within the ladder bound",
            ExoScenarioId::XcSaf1 => "Stop within the analytic d_stop",
            ExoScenarioId::XcArn1 => {
                "Static reallocation meets the wrench, else LAND (dynamic gap error is roadmap)"
            }
            ExoScenarioId::XcShd1 => {
                "Land and lock within 2 s; beam permit dropped (reduced model)"
            }
            ExoScenarioId::XcHts1 => "Lands at 10 % LN₂; no free fall",
            ExoScenarioId::XcRot1 => "Hooks engage; no contact at speed (data and admission only)",
        }
    }

    /// The "Input" column of the EXOTIC §X17 catalogue.
    pub const fn input_label(self) -> &'static str {
        match self {
            ExoScenarioId::Xs01 => "Kobe-type near-fault pulse",
            ExoScenarioId::Xs02 => "Chi-Chi-like fling of metres",
            ExoScenarioId::Xs03 => "M9, 150–300 s",
            ExoScenarioId::Xs04 => "Basin resonance, longer than 10 min",
            ExoScenarioId::Xs05 => "Vertical pulse up to 1 g",
            ExoScenarioId::Xs06 => "Grid loss at S arrival",
            ExoScenarioId::Xs07 => "Controller or LSM fault",
            ExoScenarioId::Xs08 => "Cancelled alert; concurrent-quake false alarm",
            ExoScenarioId::Xs09 => "Offshore M8: land, S-net-like, DAS",
            ExoScenarioId::Xs10 => "Generic blast with P_s, t_d and t_a as inputs",
            ExoScenarioId::Xs11 => "Tsunami with 0.3–1.2 m subsidence",
            ExoScenarioId::Xs12 => "Wind at 3–7 mg",
            ExoScenarioId::XcRed1 => "Single sensor faults of every kind",
            ExoScenarioId::XcRed2 => "β common-cause double fault",
            ExoScenarioId::XcSaf1 => "Intrusion mid-move (ARENA, BASTION)",
            ExoScenarioId::XcArn1 => "One of 500 ARENA pods lost",
            ExoScenarioId::XcShd1 => "P wave during SHIELD transit",
            ExoScenarioId::XcHts1 => "LN₂ depletion",
            ExoScenarioId::XcRot1 => "Wind uplift with 10 mm ring ovalisation",
        }
    }

    /// Requirements the scenario evidences (EXOTIC §X17).
    pub const fn requirements(self) -> &'static [&'static str] {
        match self {
            ExoScenarioId::Xs01 => &["FAB-EXO-606", "FAB-EXO-703"],
            ExoScenarioId::Xs02 => &["FAB-EXO-703"],
            ExoScenarioId::Xs03 => &["FAB-EXO-404", "FAB-EXO-706"],
            ExoScenarioId::Xs04 => &["FAB-EXO-603", "FAB-EXO-604"],
            ExoScenarioId::Xs05 => &["FAB-EXO-705", "FAB-EXO-115"],
            ExoScenarioId::Xs06 => &["FAB-EXO-120"],
            ExoScenarioId::Xs07 => &["FAB-EXO-605", "FAB-EXO-704"],
            ExoScenarioId::Xs08 => &["FAB-EXO-805"],
            ExoScenarioId::Xs09 => &["FAB-EXO-804"],
            ExoScenarioId::Xs10 => &["FAB-EXO-807"],
            ExoScenarioId::Xs11 => &["FAB-EXO-809"],
            ExoScenarioId::Xs12 => &["FAB-EXO-607"],
            ExoScenarioId::XcRed1 => &["FAB-EXO-301"],
            ExoScenarioId::XcRed2 => &["FAB-EXO-303"],
            ExoScenarioId::XcSaf1 => &["FAB-EXO-502"],
            ExoScenarioId::XcArn1 => &[],
            ExoScenarioId::XcShd1 => &[],
            ExoScenarioId::XcHts1 => &["FAB-EXO-205"],
            ExoScenarioId::XcRot1 => &[],
        }
    }

    /// Classes the scenario runs for.
    pub const fn classes(self) -> &'static [ExoClass] {
        const ALL_BUT_SELENE: &[ExoClass] = &[
            ExoClass::Sentinel,
            ExoClass::HavenR,
            ExoClass::HavenF,
            ExoClass::Shield,
            ExoClass::Bastion,
            ExoClass::Arena,
            ExoClass::Rotator,
            ExoClass::Tidegate,
        ];
        match self {
            ExoScenarioId::Xs01 | ExoScenarioId::Xs07 => &[ExoClass::Sentinel, ExoClass::HavenR],
            ExoScenarioId::Xs02
            | ExoScenarioId::Xs03
            | ExoScenarioId::Xs05
            | ExoScenarioId::Xs06
            | ExoScenarioId::XcHts1 => &[ExoClass::HavenR],
            ExoScenarioId::Xs04 | ExoScenarioId::Xs12 => &[ExoClass::Sentinel],
            ExoScenarioId::Xs08
            | ExoScenarioId::Xs09
            | ExoScenarioId::XcRed1
            | ExoScenarioId::XcRed2 => ALL_BUT_SELENE,
            ExoScenarioId::Xs10 => &[ExoClass::Bastion, ExoClass::Tidegate],
            ExoScenarioId::Xs11 => &[ExoClass::Tidegate],
            ExoScenarioId::XcSaf1 => &[ExoClass::Arena, ExoClass::Bastion],
            ExoScenarioId::XcArn1 => &[ExoClass::Arena],
            ExoScenarioId::XcShd1 => &[ExoClass::Shield],
            ExoScenarioId::XcRot1 => &[ExoClass::Rotator],
        }
    }

    /// The inputs the scenario accepts, with defaults and ranges.
    pub fn input_specs(self) -> &'static [InputSpec] {
        specs(self)
    }

    /// Parses a catalogue name (case-sensitive).
    pub fn from_name(s: &str) -> Option<ExoScenarioId> {
        ExoScenarioId::ALL.iter().copied().find(|x| x.name() == s)
    }
}

/// Errors of the exotic simulations and scenes.
#[derive(Clone, Debug, PartialEq)]
pub enum ExoSimError {
    /// Unknown class, scenario or name.
    Unknown(String),
    /// Invalid input (`path: reason`).
    Invalid {
        /// Dotted path.
        path: String,
        /// Reason.
        reason: String,
    },
    /// A Wave-0 stub was called (the Wave-1/2 owner is named).
    NotImplemented(&'static str),
}

impl fmt::Display for ExoSimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ExoSimError::Unknown(what) => write!(f, "unknown: {what}"),
            ExoSimError::Invalid { path, reason } => write!(f, "{path}: {reason}"),
            ExoSimError::NotImplemented(what) => write!(f, "not implemented yet: {what}"),
        }
    }
}

impl std::error::Error for ExoSimError {}

/// One scenario input: name, default, admissible range, unit and meaning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputSpec {
    /// Name (the key of [`ScenarioOptions::inputs`]).
    pub name: &'static str,
    /// Default value.
    pub default: f64,
    /// Smallest admissible value.
    pub min: f64,
    /// Largest admissible value.
    pub max: f64,
    /// Unit.
    pub unit: &'static str,
    /// Meaning and source.
    pub doc: &'static str,
}

const fn inp(
    name: &'static str,
    default: f64,
    min: f64,
    max: f64,
    unit: &'static str,
    doc: &'static str,
) -> InputSpec {
    InputSpec {
        name,
        default,
        min,
        max,
        unit,
        doc,
    }
}

/// One evaluated pass criterion.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Criterion {
    /// Short name.
    pub name: String,
    /// It held.
    pub passed: bool,
    /// Measured value.
    pub value: f64,
    /// Limit it was held to (NaN when the criterion is a flag).
    pub limit: f64,
    /// What was compared, with units.
    pub detail: String,
}

/// One recorded channel.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Channel {
    /// Name.
    pub name: String,
    /// Unit.
    pub unit: String,
    /// Samples at [`ScenarioResult::rate_hz`], aligned with the `t_s` channel.
    pub data: Vec<f64>,
}

/// Result of one scenario run.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ScenarioResult {
    /// Scenario name.
    pub name: String,
    /// Class slug.
    pub class: String,
    /// Every pass criterion held.
    pub passed: bool,
    /// KPIs by name (inputs included, per the print-your-inputs rule).
    pub kpis: BTreeMap<String, f64>,
    /// Reduced-model label (e.g. "2-DOF + 5-mode tower").
    pub model: String,
    /// The class is shown on a reduced (non-dynamic or roadmap-class) model.
    pub reduced_model: bool,
    /// Seed of the run.
    pub seed: u64,
    /// Every numeric input used, defaults included.
    pub inputs: BTreeMap<String, f64>,
    /// Categorical results and settings (modes, rail condition, decisions).
    pub labels: BTreeMap<String, String>,
    /// Criteria, evaluated one by one.
    pub criteria: Vec<Criterion>,
    /// Caveats and model notes.
    pub notes: Vec<String>,
    /// Channel sample rate, Hz (0 without channels).
    pub rate_hz: f64,
    /// Recorded channels (`t_s` first) when [`ScenarioOptions::record`] is set.
    pub channels: Vec<Channel>,
}

/// JSON schema of [`ScenarioResult::to_json`].
pub const EXO_RUN_SCHEMA: &str = "fabrail.exo_run/1";

impl ScenarioResult {
    /// The result as `fabrail.exo_run/1` JSON (non-finite numbers become null).
    /// Channels are included only with `with_channels`.
    pub fn to_json(&self, with_channels: bool) -> serde_json::Value {
        let criteria: Vec<serde_json::Value> = self
            .criteria
            .iter()
            .map(|c| {
                serde_json::json!({
                    "name": c.name,
                    "passed": c.passed,
                    "value": c.value,
                    "limit": c.limit,
                    "detail": c.detail,
                })
            })
            .collect();
        let id = ExoScenarioId::from_name(&self.name);
        let mut v = serde_json::json!({
            "schema": EXO_RUN_SCHEMA,
            "scenario": self.name,
            "class": self.class,
            "model": self.model,
            "reduced_model": self.reduced_model,
            "passed": self.passed,
            "seed": self.seed,
            "criteria_text": id.map_or("", |i| i.criteria()),
            "requirements": id.map_or(&[][..], |i| i.requirements()),
            "criteria": criteria,
            "kpis": self.kpis,
            "inputs": self.inputs,
            "labels": self.labels,
            "notes": self.notes,
            "rate_hz": self.rate_hz,
        });
        if with_channels {
            let mut ch = serde_json::Map::new();
            for c in &self.channels {
                ch.insert(
                    c.name.clone(),
                    serde_json::json!({"unit": c.unit, "data": c.data}),
                );
            }
            v["channels"] = serde_json::Value::Object(ch);
        }
        v
    }
}

/// Options of [`run_scenario_with`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ScenarioOptions {
    /// Seed (noise, wind phases, fault picks).
    pub seed: u64,
    /// Input overrides by name ([`ExoScenarioId::input_specs`]).
    pub inputs: BTreeMap<String, f64>,
    /// Record channels.
    pub record: bool,
}

/// Runs scenario `id` for class `class_slug` with seed `seed` (default inputs, no
/// channels).
pub fn run_scenario(
    id: ExoScenarioId,
    class_slug: &str,
    seed: u64,
) -> Result<ScenarioResult, ExoSimError> {
    run_scenario_with(
        id,
        class_slug,
        &ScenarioOptions {
            seed,
            ..ScenarioOptions::default()
        },
    )
}

/// Runs scenario `id` for class `class_slug` with `opts`. Errors: unknown class,
/// a class the scenario does not cover (`class`), an unknown input (`inputs.<name>`) or a
/// value outside its range.
pub fn run_scenario_with(
    id: ExoScenarioId,
    class_slug: &str,
    opts: &ScenarioOptions,
) -> Result<ScenarioResult, ExoSimError> {
    let Some(class) = ExoClass::from_slug(class_slug) else {
        return Err(ExoSimError::Unknown(format!(
            "exotic class \"{class_slug}\""
        )));
    };
    if !id.classes().contains(&class) {
        let list: Vec<&str> = id.classes().iter().map(|c| c.slug()).collect();
        return Err(ExoSimError::Invalid {
            path: "class".into(),
            reason: format!(
                "{} runs for {} only, not {class_slug}",
                id.name(),
                list.join(", ")
            ),
        });
    }
    let specs = id.input_specs();
    for (k, v) in &opts.inputs {
        let Some(s) = specs.iter().find(|s| s.name == k) else {
            let names: Vec<&str> = specs.iter().map(|s| s.name).collect();
            return Err(ExoSimError::Invalid {
                path: format!("inputs.{k}"),
                reason: format!(
                    "unknown input for {} (known: {})",
                    id.name(),
                    names.join(", ")
                ),
            });
        };
        if !(v.is_finite() && *v >= s.min && *v <= s.max) {
            return Err(ExoSimError::Invalid {
                path: format!("inputs.{k}"),
                reason: format!("{v} outside [{}, {}] {}", s.min, s.max, s.unit),
            });
        }
    }
    let mut c = Ctx {
        specs,
        given: &opts.inputs,
        record: opts.record,
        seed: opts.seed,
        out: ScenarioResult {
            name: id.name().into(),
            class: class.slug().into(),
            seed: opts.seed,
            ..ScenarioResult::default()
        },
    };
    match id {
        ExoScenarioId::Xs01 => xs01(&mut c, class),
        ExoScenarioId::Xs02 => xs02(&mut c),
        ExoScenarioId::Xs03 => xs03(&mut c),
        ExoScenarioId::Xs04 => xs04(&mut c),
        ExoScenarioId::Xs05 => xs05(&mut c),
        ExoScenarioId::Xs06 => xs06(&mut c),
        ExoScenarioId::Xs07 => xs07(&mut c, class),
        ExoScenarioId::Xs08 => xs08(&mut c, class),
        ExoScenarioId::Xs09 => xs09(&mut c, class),
        ExoScenarioId::Xs10 => xs10(&mut c, class),
        ExoScenarioId::Xs11 => xs11(&mut c),
        ExoScenarioId::Xs12 => xs12(&mut c),
        ExoScenarioId::XcRed1 => xc_red1(&mut c, class),
        ExoScenarioId::XcRed2 => xc_red2(&mut c, class),
        ExoScenarioId::XcSaf1 => xc_saf1(&mut c, class),
        ExoScenarioId::XcArn1 => xc_arn1(&mut c),
        ExoScenarioId::XcShd1 => xc_shd1(&mut c),
        ExoScenarioId::XcHts1 => xc_hts1(&mut c),
        ExoScenarioId::XcRot1 => xc_rot1(&mut c),
    }
    let mut out = c.out;
    out.passed = !out.criteria.is_empty() && out.criteria.iter().all(|k| k.passed);
    Ok(out)
}

// ---- Input specifications ----------------------------------------------------------------

fn specs(id: ExoScenarioId) -> &'static [InputSpec] {
    const XS01: &[InputSpec] = &[
        inp(
            "mw",
            6.9,
            5.5,
            8.0,
            "",
            "moment magnitude; pulse period ln Tp = −6.45 + 1.11·Mw (Baker 2007) [UNCERTAIN]",
        ),
        inp(
            "v_pulse_mps",
            0.9,
            0.1,
            2.5,
            "m/s",
            "peak pulse velocity (Kobe-type ≈ 0.9 m/s) [UNCERTAIN]",
        ),
        inp("t0_s", 10.0, 2.0, 60.0, "s", "pulse centre time"),
        inp("duration_s", 80.0, 20.0, 600.0, "s", "run length"),
        inp("zeta_s", 0.02, 0.0, 0.1, "", "SENTINEL host damping ζ_s"),
        inp(
            "y_frac",
            0.0,
            0.0,
            1.0,
            "",
            "HAVEN-R: cross component as a fraction of the pulse",
        ),
    ];
    const XS02: &[InputSpec] = &[
        inp(
            "d_m",
            5.0,
            1.0,
            9.0,
            "m",
            "fling displacement (Chi-Chi-like 5–9 m) [UNCERTAIN]",
        ),
        inp(
            "t_r_s",
            5.0,
            2.0,
            20.0,
            "s",
            "fling rise time (one sine cycle of acceleration) [UNCERTAIN]",
        ),
        inp("t0_s", 5.0, 1.0, 30.0, "s", "fling start"),
        inp(
            "duration_s",
            90.0,
            40.0,
            600.0,
            "s",
            "run length (landing 30 s before the end)",
        ),
        inp(
            "design_decel_g",
            1.0,
            0.1,
            10.0,
            "g",
            "design deceleration of the moat buffer (X4 buffer: 1 g at full compression)",
        ),
        inp(
            "support_margin_m",
            0.10,
            0.0,
            1.0,
            "m",
            "guideway extent beyond the moat wall (support lost beyond it)",
        ),
    ];
    const XS03: &[InputSpec] = &[
        inp(
            "duration_s",
            300.0,
            150.0,
            300.0,
            "s",
            "M9 record length (150–300 s)",
        ),
        inp(
            "pga_x_g",
            0.3,
            0.05,
            0.6,
            "g",
            "peak ground acceleration, x",
        ),
        inp("pga_y_g", 0.2, 0.0, 0.6, "g", "peak ground acceleration, y"),
        inp(
            "store_kwh",
            3.0,
            0.5,
            20.0,
            "kWh",
            "supercap usable energy (HAVEN-R 1–5 kWh)",
        ),
        inp("soc0", 0.8, 0.1, 1.0, "", "initial state of charge"),
        inp(
            "p_grid_w",
            2000.0,
            0.0,
            1.0e5,
            "W",
            "grid charging power into the store (0: islanded)",
        ),
        inp(
            "p_sensors_w",
            500.0,
            0.0,
            1.0e4,
            "W",
            "sensing and control power",
        ),
        inp(
            "eta_drive",
            0.9,
            0.5,
            1.0,
            "",
            "LSM drive efficiency (motoring and regeneration)",
        ),
        inp(
            "k_m_n_per_sqrt_w",
            500.0,
            50.0,
            5000.0,
            "N/√W",
            "LSM motor constant (copper loss (F/K_m)²) [UNCERTAIN]",
        ),
    ];
    const XS04: &[InputSpec] = &[
        inp(
            "duration_s",
            720.0,
            610.0,
            3600.0,
            "s",
            "basin record length (> 10 min)",
        ),
        inp("t_g_s", 6.0, 3.0, 10.0, "s", "basin period 2π/ω_g"),
        inp("zeta_g", 0.10, 0.05, 0.15, "", "basin damping ζ_g"),
        inp(
            "stroke_demand",
            1.0,
            0.5,
            2.0,
            "",
            "passive peak stroke (no stops) as a multiple of s_max; sets the RMS (1.0 = the design basis; the overdrive case 1.15× is reported alongside)",
        ),
        inp("zeta_s", 0.02, 0.0, 0.1, "", "host damping ζ_s"),
    ];
    const XS05: &[InputSpec] = &[
        inp(
            "a_peak_g",
            1.0,
            0.1,
            1.0,
            "g",
            "vertical pulse peak (≤ 1 g)",
        ),
        inp("mw", 6.9, 5.5, 8.0, "", "pulse magnitude (period)"),
        inp(
            "f_air_hz",
            2.0,
            0.5,
            5.0,
            "Hz",
            "air-spring vertical frequency [UNCERTAIN design pick]",
        ),
        inp("zeta_air", 0.15, 0.02, 0.5, "", "air-spring damping"),
        inp(
            "stroke_air_m",
            0.08,
            0.01,
            0.3,
            "m",
            "air-spring working stroke (± about the static height)",
        ),
        inp(
            "trim_frac",
            0.10,
            0.01,
            0.5,
            "",
            "hybrid-EMS trim authority as a fraction of the weight",
        ),
        inp("duration_s", 20.0, 5.0, 120.0, "s", "run length"),
    ];
    const XS06: &[InputSpec] = &[
        inp("t_shaking_s", 150.0, 30.0, 600.0, "s", "shaking duration"),
        inp(
            "pga_g",
            0.25,
            0.05,
            0.6,
            "g",
            "peak ground acceleration (x; 0.7× on y)",
        ),
        inp(
            "t_s_s",
            10.0,
            1.0,
            60.0,
            "s",
            "S arrival = grid loss, after the record start",
        ),
        inp(
            "store_kwh",
            3.0,
            0.2,
            20.0,
            "kWh",
            "supercap usable energy (HAVEN-R 1–5 kWh)",
        ),
        inp("soc0", 0.9, 0.1, 1.0, "", "initial state of charge"),
        inp(
            "p_sensors_w",
            500.0,
            0.0,
            1.0e4,
            "W",
            "sensing and control power",
        ),
        inp("eta_drive", 0.9, 0.5, 1.0, "", "LSM drive efficiency"),
        inp(
            "k_m_n_per_sqrt_w",
            500.0,
            50.0,
            5000.0,
            "N/√W",
            "LSM motor constant [UNCERTAIN]",
        ),
    ];
    const XS07: &[InputSpec] = &[
        inp(
            "pga_g",
            0.15,
            0.02,
            0.6,
            "g",
            "peak ground acceleration of the long-duration record",
        ),
        inp("duration_s", 120.0, 40.0, 600.0, "s", "record length"),
        inp(
            "t_fault_s",
            30.0,
            5.0,
            300.0,
            "s",
            "controller / LSM fault time",
        ),
        inp(
            "t_lev_loss_s",
            5.0,
            0.0,
            120.0,
            "s",
            "SENTINEL: levitation loss after the LSM fault",
        ),
        inp(
            "fps_mu",
            0.05,
            0.01,
            0.15,
            "",
            "friction-pendulum catcher coefficient",
        ),
        inp(
            "gap_m",
            0.012,
            0.005,
            0.03,
            "m",
            "SENTINEL levitation gap (touchdown drop)",
        ),
    ];
    const XS08: &[InputSpec] = &[
        inp(
            "magnitude",
            6.8,
            5.0,
            9.0,
            "",
            "magnitude of the cancelled alert",
        ),
        inp(
            "r_epi_km",
            80.0,
            5.0,
            400.0,
            "km",
            "epicentral distance of the cancelled alert",
        ),
        inp("t_cancel_s", 7.0, 0.5, 60.0, "s", "cancel after the alert"),
        inp(
            "fa_magnitude",
            7.0,
            5.0,
            9.0,
            "",
            "false alarm: reported magnitude (concurrent-quake mislocation)",
        ),
        inp(
            "fa_r_epi_km",
            40.0,
            5.0,
            400.0,
            "km",
            "false alarm: reported distance",
        ),
        inp("depth_km", 10.0, 0.0, 100.0, "km", "reported depth"),
    ];
    const XS09: &[InputSpec] = &[
        inp(
            "magnitude",
            8.0,
            6.5,
            9.2,
            "",
            "magnitude (sizes the synthetic P onset)",
        ),
        inp(
            "epi_offshore_km",
            100.0,
            10.0,
            400.0,
            "km",
            "epicentre distance offshore",
        ),
        inp("depth_km", 20.0, 0.0, 80.0, "km", "hypocentral depth"),
        inp(
            "city_inland_km",
            20.0,
            0.0,
            200.0,
            "km",
            "city distance inland",
        ),
        inp(
            "latency_s",
            3.3,
            0.5,
            20.0,
            "s",
            "latency chain total (default split 1/1/0.5/0.5/0.3 s)",
        ),
        inp("t_act_s", 0.0, 0.0, 10.0, "s", "actuation time at the site"),
        inp(
            "land_km",
            110.0,
            10.0,
            500.0,
            "km",
            "k-th land station (epicentral)",
        ),
        inp(
            "snet_km",
            34.0,
            5.0,
            300.0,
            "km",
            "k-th seafloor station (epicentral)",
        ),
        inp(
            "das_offset_km",
            8.0,
            0.0,
            100.0,
            "km",
            "DAS cable closest approach",
        ),
    ];
    const XS10: &[InputSpec] = &[
        inp(
            "p_s_kpa",
            50.0,
            1.0,
            1.0e4,
            "kPa",
            "peak side-on overpressure P_s (generic input; no yield or range law)",
        ),
        inp(
            "t_d_ms",
            30.0,
            1.0,
            1000.0,
            "ms",
            "positive-phase duration t_d",
        ),
        inp("b", 1.0, 0.1, 10.0, "", "Friedlander decay b"),
        inp(
            "t_a_s",
            8.0,
            0.1,
            120.0,
            "s",
            "arrival time t_a after detection start (an input)",
        ),
        inp(
            "t_detect_s",
            0.5,
            0.0,
            5.0,
            "s",
            "detection and decision latency",
        ),
        inp("t_seal_s", 0.3, 0.0, 5.0, "s", "land-to-seal time"),
        inp("margin_s", 0.3, 0.0, 5.0, "s", "safety margin"),
        inp(
            "range_m",
            0.0,
            0.0,
            1.0e5,
            "m",
            "range, only for the reported bound t_a ≤ R/c0 (0: not given)",
        ),
    ];
    const XS11: &[InputSpec] = &[
        inp(
            "subsidence_m",
            0.6,
            0.0,
            1.2,
            "m",
            "coastal subsidence (0.3–1.2 m)",
        ),
        inp(
            "differential_frac",
            0.05,
            0.0,
            0.5,
            "",
            "differential settlement across the span [UNCERTAIN]",
        ),
        inp(
            "tilt_mrad",
            2.0,
            0.0,
            50.0,
            "mrad",
            "longitudinal rail tilt",
        ),
        inp("t_shaking_s", 180.0, 30.0, 600.0, "s", "shaking duration"),
        inp(
            "t_latency_s",
            30.0,
            0.0,
            600.0,
            "s",
            "decision latency after the shaking",
        ),
        inp(
            "command",
            1.0,
            0.0,
            1.0,
            "",
            "external decision-system command sent (1) or not (0)",
        ),
        inp(
            "path_scale",
            1.0,
            0.1,
            3.0,
            "",
            "scale of the reference source-to-coast path length",
        ),
    ];
    const XS12: &[InputSpec] = &[
        inp(
            "a_target_mg",
            5.0,
            3.0,
            7.0,
            "mg",
            "uncontrolled peak host acceleration (wind comfort 3–7 mg)",
        ),
        inp(
            "zeta_s",
            0.01,
            0.002,
            0.05,
            "",
            "host damping ζ_s (the reduction is reported against it)",
        ),
        inp(
            "u10_mps",
            25.0,
            5.0,
            60.0,
            "m/s",
            "mean wind speed at 10 m (spectrum shape)",
        ),
        inp("duration_s", 3600.0, 600.0, 7200.0, "s", "record length"),
    ];
    const RED: &[InputSpec] = &[
        inp("gap_mm", 12.0, 5.0, 30.0, "mm", "nominal pod gap"),
        inp(
            "pole_rad_s",
            30.0,
            5.0,
            200.0,
            "rad/s",
            "open-loop heave pole of the pod [UNCERTAIN]",
        ),
        inp("noise_um", 5.0, 0.0, 200.0, "µm", "gap-sensor noise RMS"),
        inp(
            "e_th_um",
            300.0,
            20.0,
            5000.0,
            "µm",
            "voter discrepancy threshold e_th",
        ),
        inp(
            "t_persist_ms",
            20.0,
            1.0,
            500.0,
            "ms",
            "voter/ladder persistence",
        ),
    ];
    const SAF: &[InputSpec] = &[
        inp(
            "t_react_s",
            0.1,
            0.0,
            1.0,
            "s",
            "sensing + logic time t1 (ISO 13855 input)",
        ),
        inp(
            "intrusion_frac",
            0.5,
            0.05,
            0.95,
            "",
            "intrusion at this fraction of the move time",
        ),
    ];
    const ARN: &[InputSpec] = &[
        inp(
            "capacity_ratio",
            1.6,
            1.0,
            4.0,
            "",
            "total pod capacity / weight [UNCERTAIN]",
        ),
        inp(
            "cog_x_m",
            2.0,
            -20.0,
            20.0,
            "m",
            "centre-of-gravity offset along x",
        ),
        inp(
            "cog_y_m",
            1.0,
            -20.0,
            20.0,
            "m",
            "centre-of-gravity offset along y",
        ),
        inp(
            "lost_pod",
            -1.0,
            -1.0,
            499.0,
            "",
            "index of the lost pod (−1: the most loaded)",
        ),
    ];
    const SHD: &[InputSpec] = &[
        inp(
            "v_mps",
            0.5,
            0.05,
            0.5,
            "m/s",
            "transit speed (SHIELD v_max 0.5 m/s)",
        ),
        inp(
            "t_react_s",
            0.1,
            0.0,
            0.5,
            "s",
            "detection-to-brake reaction",
        ),
        inp("j_ss1_mps3", 5.0, 0.5, 50.0, "m/s³", "SS1 jerk [UNCERTAIN]"),
        inp("t_land_s", 0.5, 0.1, 5.0, "s", "PM-bias-ramp landing time"),
        inp("t_lock_s", 0.2, 0.0, 2.0, "s", "lock engagement time"),
    ];
    const HTS: &[InputSpec] = &[
        inp(
            "load_frac",
            0.9,
            0.3,
            1.0,
            "",
            "module load as a fraction of its first-loading force at the working gap",
        ),
        inp(
            "v_land_mmps",
            5.0,
            0.5,
            50.0,
            "mm/s",
            "controlled lowering speed onto the skids",
        ),
        inp(
            "dynamic",
            0.0,
            0.0,
            1.0,
            "",
            "0: static heat load; 1: dynamic (vibration) heat load",
        ),
    ];
    const ROT: &[InputSpec] = &[
        inp("u10_mps", 50.0, 10.0, 90.0, "m/s", "gust wind speed"),
        inp(
            "c_lift",
            0.5,
            0.0,
            3.0,
            "",
            "uplift coefficient of the dome [UNCERTAIN]",
        ),
        inp(
            "roof_area_m2",
            10000.0,
            100.0,
            50000.0,
            "m²",
            "dome area carried by the ring [UNCERTAIN]",
        ),
        inp(
            "uplift_factor",
            1.0,
            0.0,
            50.0,
            "",
            "multiplier on the uplift (what-if)",
        ),
        inp(
            "oval_mm",
            10.0,
            0.0,
            30.0,
            "mm",
            "in-plane ring ovalisation amplitude (cos 2θ)",
        ),
        inp(
            "gap_mm",
            17.5,
            15.0,
            20.0,
            "mm",
            "nominal ring gap (15–20 mm)",
        ),
        inp(
            "lat_clearance_mm",
            20.0,
            5.0,
            60.0,
            "mm",
            "lateral guide clearance [UNCERTAIN]",
        ),
        inp(
            "pole_width_mm",
            100.0,
            20.0,
            400.0,
            "mm",
            "pole width (overlap loss under radial offset) [UNCERTAIN]",
        ),
        inp(
            "capacity_ratio",
            1.5,
            1.0,
            4.0,
            "",
            "station EMS capacity / weight share at full overlap [UNCERTAIN]",
        ),
    ];
    match id {
        ExoScenarioId::Xs01 => XS01,
        ExoScenarioId::Xs02 => XS02,
        ExoScenarioId::Xs03 => XS03,
        ExoScenarioId::Xs04 => XS04,
        ExoScenarioId::Xs05 => XS05,
        ExoScenarioId::Xs06 => XS06,
        ExoScenarioId::Xs07 => XS07,
        ExoScenarioId::Xs08 => XS08,
        ExoScenarioId::Xs09 => XS09,
        ExoScenarioId::Xs10 => XS10,
        ExoScenarioId::Xs11 => XS11,
        ExoScenarioId::Xs12 => XS12,
        ExoScenarioId::XcRed1 | ExoScenarioId::XcRed2 => RED,
        ExoScenarioId::XcSaf1 => SAF,
        ExoScenarioId::XcArn1 => ARN,
        ExoScenarioId::XcShd1 => SHD,
        ExoScenarioId::XcHts1 => HTS,
        ExoScenarioId::XcRot1 => ROT,
    }
}

// ---- Run context -------------------------------------------------------------------------

/// Per-run context: inputs (validated), results, criteria and the recorder switch.
struct Ctx<'a> {
    specs: &'static [InputSpec],
    given: &'a BTreeMap<String, f64>,
    record: bool,
    seed: u64,
    out: ScenarioResult,
}

impl Ctx<'_> {
    /// The value of input `name` (override or default), recorded in the result.
    fn get(&mut self, name: &'static str) -> f64 {
        let spec = self.specs.iter().find(|s| s.name == name);
        debug_assert!(spec.is_some(), "input {name} is not in the scenario spec");
        let v = self
            .given
            .get(name)
            .copied()
            .or(spec.map(|s| s.default))
            .unwrap_or(f64::NAN);
        self.out.inputs.insert(name.into(), v);
        v
    }

    /// Records a derived input (not overridable).
    fn fixed(&mut self, name: &str, v: f64) {
        self.out.inputs.insert(name.into(), v);
    }

    fn kpi(&mut self, name: &str, v: f64) {
        self.out.kpis.insert(name.into(), v);
    }

    fn label(&mut self, name: &str, v: &str) {
        self.out.labels.insert(name.into(), v.into());
    }

    fn note(&mut self, s: &str) {
        self.out.notes.push(s.into());
    }

    fn model(&mut self, m: &str, reduced: bool) {
        self.out.model = m.into();
        self.out.reduced_model = reduced;
    }

    fn check(&mut self, name: &str, passed: bool, value: f64, limit: f64, detail: String) {
        self.out.criteria.push(Criterion {
            name: name.into(),
            passed,
            value,
            limit,
            detail,
        });
    }

    fn recorder(
        &self,
        dt_s: f64,
        rate_hz: f64,
        names: &[(&'static str, &'static str)],
    ) -> Recorder {
        Recorder::new(self.record, dt_s, rate_hz, names)
    }

    fn finish(&mut self, rec: Recorder) {
        if self.record {
            let (rate, ch) = rec.into_channels();
            self.out.rate_hz = rate;
            self.out.channels = ch;
        }
    }
}

/// Decimating channel recorder.
struct Recorder {
    on: bool,
    every: usize,
    rate_hz: f64,
    names: Vec<(&'static str, &'static str)>,
    t: Vec<f64>,
    data: Vec<Vec<f64>>,
}

impl Recorder {
    fn new(on: bool, dt_s: f64, rate_hz: f64, names: &[(&'static str, &'static str)]) -> Recorder {
        let every = if dt_s > 0.0 && rate_hz > 0.0 {
            fmath::round(1.0 / (dt_s * rate_hz)).max(1.0) as usize
        } else {
            1
        };
        Recorder {
            on,
            every,
            rate_hz: if dt_s > 0.0 {
                1.0 / (dt_s * every as f64)
            } else {
                rate_hz
            },
            names: names.to_vec(),
            t: Vec::new(),
            data: vec![Vec::new(); names.len()],
        }
    }

    fn push(&mut self, k: usize, t_s: f64, vals: &[f64]) {
        if !self.on || !k.is_multiple_of(self.every) {
            return;
        }
        self.t.push(t_s);
        for (d, v) in self.data.iter_mut().zip(vals) {
            d.push(*v);
        }
    }

    fn into_channels(self) -> (f64, Vec<Channel>) {
        let mut out = vec![Channel {
            name: "t_s".into(),
            unit: "s".into(),
            data: self.t,
        }];
        for ((n, u), d) in self.names.into_iter().zip(self.data) {
            out.push(Channel {
                name: n.into(),
                unit: u.into(),
                data: d,
            });
        }
        (self.rate_hz, out)
    }
}

// ---- Small dense linear algebra (LQR, H2) ------------------------------------------------

type M4 = [[f64; 4]; 4];

#[allow(clippy::needless_range_loop)]
fn mat_mul<const N: usize>(a: &[[f64; N]; N], b: &[[f64; N]; N]) -> [[f64; N]; N] {
    let mut c = [[0.0; N]; N];
    for i in 0..N {
        for k in 0..N {
            let aik = a[i][k];
            if aik == 0.0 {
                continue;
            }
            for j in 0..N {
                c[i][j] += aik * b[k][j];
            }
        }
    }
    c
}

#[allow(clippy::needless_range_loop)]
fn transpose<const N: usize>(a: &[[f64; N]; N]) -> [[f64; N]; N] {
    let mut t = [[0.0; N]; N];
    for i in 0..N {
        for j in 0..N {
            t[j][i] = a[i][j];
        }
    }
    t
}

/// Matrix exponential by scaling and squaring with a 16-term Taylor series.
#[allow(clippy::needless_range_loop)]
fn expm<const N: usize>(a: &[[f64; N]; N]) -> [[f64; N]; N] {
    let norm = a
        .iter()
        .map(|r| r.iter().map(|x| fmath::abs(*x)).sum::<f64>())
        .fold(0.0_f64, f64::max);
    let mut s = 0_u32;
    let mut scale = 1.0;
    while norm * scale > 0.5 && s < 60 {
        scale *= 0.5;
        s += 1;
    }
    let mut x = *a;
    for r in x.iter_mut() {
        for v in r.iter_mut() {
            *v *= scale;
        }
    }
    let mut result = [[0.0; N]; N];
    let mut term = [[0.0; N]; N];
    for i in 0..N {
        result[i][i] = 1.0;
        term[i][i] = 1.0;
    }
    for k in 1..=16 {
        term = mat_mul(&term, &x);
        let inv = 1.0 / f64::from(k);
        for r in term.iter_mut() {
            for v in r.iter_mut() {
                *v *= inv;
            }
        }
        for i in 0..N {
            for j in 0..N {
                result[i][j] += term[i][j];
            }
        }
    }
    for _ in 0..s {
        result = mat_mul(&result, &result);
    }
    result
}

/// Solves a·x = b (Gaussian elimination, partial pivoting); `None` when singular.
#[allow(clippy::needless_range_loop)]
fn solve<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    for col in 0..N {
        let mut piv = col;
        for r in col + 1..N {
            if fmath::abs(a[r][col]) > fmath::abs(a[piv][col]) {
                piv = r;
            }
        }
        if fmath::abs(a[piv][col]) < 1.0e-300 {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for r in col + 1..N {
            let m = a[r][col] / a[col][col];
            if m == 0.0 {
                continue;
            }
            for c in col..N {
                a[r][c] -= m * a[col][c];
            }
            b[r] -= m * b[col];
        }
    }
    let mut x = [0.0; N];
    for col in (0..N).rev() {
        let mut s = b[col];
        for k in col + 1..N {
            s -= a[col][k] * x[k];
        }
        x[col] = s / a[col][col];
    }
    Some(x)
}

/// Solves the continuous Lyapunov equation A·P + P·Aᵀ + Q = 0 (A Hurwitz).
#[allow(clippy::needless_range_loop)]
fn lyapunov4(a: &M4, q: &M4) -> Option<M4> {
    let mut m = [[0.0; 16]; 16];
    let mut rhs = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            let row = i + 4 * j;
            rhs[row] = -q[i][j];
            for k in 0..4 {
                m[row][k + 4 * j] += a[i][k];
                m[row][i + 4 * k] += a[j][k];
            }
        }
    }
    let x = solve(m, rhs)?;
    let mut p = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            p[i][j] = x[i + 4 * j];
        }
    }
    Some(p)
}

/// Zero-order-hold discretisation of (A, B) over `dt` through the augmented exponential.
#[allow(clippy::needless_range_loop)]
fn c2d(a: &M4, b: &[f64; 4], dt: f64) -> (M4, [f64; 4]) {
    let mut m = [[0.0; 5]; 5];
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] = a[i][j] * dt;
        }
        m[i][4] = b[i] * dt;
    }
    let e = expm(&m);
    let mut ad = [[0.0; 4]; 4];
    let mut bd = [0.0; 4];
    for i in 0..4 {
        for j in 0..4 {
            ad[i][j] = e[i][j];
        }
        bd[i] = e[i][4];
    }
    (ad, bd)
}

/// Discrete LQR gain K (u = −K·x) by Riccati iteration
/// P ← Q + AᵀPA − AᵀPB(R + BᵀPB)⁻¹BᵀPA (the equation scipy's `solve_discrete_are`
/// solves; converged to 1e-12 relative).
#[allow(clippy::needless_range_loop)]
fn dlqr(ad: &M4, bd: &[f64; 4], q: &M4, r: f64) -> [f64; 4] {
    let at = transpose(ad);
    let mut p = *q;
    let mut k = [0.0; 4];
    for _ in 0..200_000 {
        // PB and BᵀPB.
        let mut pb = [0.0; 4];
        for i in 0..4 {
            for j in 0..4 {
                pb[i] += p[i][j] * bd[j];
            }
        }
        let btpb: f64 = (0..4).map(|i| bd[i] * pb[i]).sum();
        // BᵀPA (row) = (PB)ᵀ·A.
        let mut btpa = [0.0; 4];
        for j in 0..4 {
            for i in 0..4 {
                btpa[j] += pb[i] * ad[i][j];
            }
        }
        let denom = r + btpb;
        for j in 0..4 {
            k[j] = btpa[j] / denom;
        }
        let pa = mat_mul(&p, ad);
        let atpa = mat_mul(&at, &pa);
        let mut pn = [[0.0; 4]; 4];
        let mut diff = 0.0_f64;
        let mut size = 0.0_f64;
        for i in 0..4 {
            for j in 0..4 {
                pn[i][j] = q[i][j] + atpa[i][j] - btpa[i] * btpa[j] / denom;
                diff = diff.max(fmath::abs(pn[i][j] - p[i][j]));
                size = size.max(fmath::abs(pn[i][j]));
            }
        }
        p = pn;
        if diff <= 1.0e-12 * size.max(f64::MIN_POSITIVE) {
            break;
        }
    }
    k
}

/// RMS of a series (0 when empty).
fn rms(x: &[f64]) -> f64 {
    if x.is_empty() {
        return 0.0;
    }
    fmath::sqrt(x.iter().map(|v| v * v).sum::<f64>() / x.len() as f64)
}

fn peak_abs(x: &[f64]) -> f64 {
    x.iter().fold(0.0_f64, |a, v| a.max(fmath::abs(*v)))
}

// ---- SENTINEL reduced model (2-DOF + HMD) ------------------------------------------------

/// SENTINEL reference: a 1,000 t damper (μ = 1 %) on the first mode of a 100 kt modal
/// mass at T = 7 s, ±1.5 m stroke, 121 kN / 163 kW trim LSM (exotic design §3.1; EXOTIC
/// §X7), on a low-coercivity rail (hysteresis drag μ_eq 1e-4) with an ironless LSM.
#[derive(Clone, Copy, Debug)]
struct Sentinel {
    m_s: f64,
    w_s: f64,
    zeta_s: f64,
    mu: f64,
    s_max: f64,
    f_max: f64,
    p_max: f64,
    form: TuningForm,
    rail: RailCondition,
}

/// Weights of the SENTINEL LQR on [x_s, ẋ_s, x_r, ẋ_r] (host displacement and velocity,
/// an explicit stroke weight) and the force weight, in SI per m², (m/s)², N².
const SENTINEL_Q_STROKE: f64 = 1.0e-3;
const SENTINEL_R_FORCE: f64 = 1.0e-13;
/// Controller sample time of the HMD in the scenarios, s.
const SENTINEL_DT: f64 = 0.02;
/// Floor acceleration that switches the HMD to SEISMIC_PASSIVE, m/s² (5 gal)
/// \[UNCERTAIN: a design pick well above wind floors of 3–7 mg\].
const SENTINEL_SEISMIC_MPS2: f64 = 0.05;

impl Sentinel {
    fn reference(zeta_s: f64, form: TuningForm) -> Sentinel {
        Sentinel {
            m_s: 1.0e8,
            w_s: TAU / 7.0,
            zeta_s,
            mu: 0.01,
            s_max: 1.5,
            f_max: 121.0e3,
            p_max: 163.0e3,
            form,
            rail: RailCondition::LowCoercivity,
        }
    }

    fn primary(&self) -> Primary {
        Primary {
            m_kg: self.m_s,
            omega_rad_s: self.w_s,
            zeta: self.zeta_s,
        }
    }

    fn design(&self) -> DamperDesign {
        damper_design_for(self.m_s, self.w_s, self.mu, self.form)
    }

    fn two_dof(&self, stops: bool) -> TwoDof {
        TwoDof::new(
            self.primary(),
            Damper::from_design(&self.design(), if stops { self.s_max } else { 0.0 }),
        )
    }

    /// Continuous (A, B_u, E_force) on [x_s, ẋ_s, x_r, ẋ_r] (u on the damper, reaction on
    /// the host; wind force on the host).
    fn state_space(&self, p: &Primary) -> (M4, [f64; 4], [f64; 4]) {
        let d = self.design();
        let (m, md) = (p.m_kg, d.m_d_kg);
        let (w, z) = (p.omega_rad_s, p.zeta);
        let (kd, cd) = (d.k_d_n_per_m, d.c_d_n_s_per_m);
        let a = [
            [0.0, 1.0, 0.0, 0.0],
            [-w * w, -2.0 * z * w, kd / m, cd / m],
            [0.0, 0.0, 0.0, 1.0],
            [w * w, 2.0 * z * w, -kd / md - kd / m, -cd / md - cd / m],
        ];
        let b = [0.0, -1.0 / m, 0.0, 1.0 / md + 1.0 / m];
        let e = [0.0, 1.0 / m, 0.0, -1.0 / m];
        (a, b, e)
    }

    fn hmd(&self, k_lqr: [f64; 4]) -> Hmd {
        let d = self.design();
        let c_dvf = dvf_damping_n_s_per_m(&d, 0.5 * d.c_d_n_s_per_m);
        Hmd::new(
            HmdGains {
                k_lqr: k_lqr.map(|g| g as f32),
                c_dvf_n_s_per_m: c_dvf as f32,
            },
            HmdLimits::from_actuator(self.s_max as f32, self.p_max as f32, self.f_max as f32),
        )
    }

    /// Hysteresis-drag force of the bearing on the damper, N (regularised Coulomb,
    /// μ_eq·m_d·g; EXOTIC §X8).
    fn drag_n(&self, v_r: f64) -> f64 {
        -self.rail.mu_eq()
            * self.design().m_d_kg
            * G0_MPS2
            * fmath::tanh(v_r / suspension::DRAG_V_REG_MPS)
    }
}

/// The SENTINEL LQR gains from the discrete Riccati equation at `dt` (Q = diag(ω², 1,
/// q_s·ω², q_s), R = 1e-13 N⁻², both scaled by dt).
fn sentinel_lqr(s: &Sentinel, dt: f64) -> [f64; 4] {
    let (a, b, _) = s.state_space(&s.primary());
    let (ad, bd) = c2d(&a, &b, dt);
    let w2 = s.w_s * s.w_s;
    let q = [
        [w2 * dt, 0.0, 0.0, 0.0],
        [0.0, dt, 0.0, 0.0],
        [0.0, 0.0, SENTINEL_Q_STROKE * w2 * dt, 0.0],
        [0.0, 0.0, 0.0, SENTINEL_Q_STROKE * dt],
    ];
    dlqr(&ad, &bd, &q, SENTINEL_R_FORCE * dt)
}

/// H2 host-displacement variance ratio (controlled/bare) under white-noise force, from
/// the Lyapunov equations; `k` = LQR gain row (zeros: passive).
fn h2_host_ratio(s: &Sentinel, k: [f64; 4]) -> Option<f64> {
    let p = s.primary();
    let (mut a, b, e) = s.state_space(&p);
    for i in 0..4 {
        for j in 0..4 {
            a[i][j] -= b[i] * k[j];
        }
    }
    let mut q = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            q[i][j] = e[i] * e[j];
        }
    }
    let pc = lyapunov4(&a, &q)?;
    // Bare SDOF with the same force: σ_x² = 1/(4ζω³m²) (unit two-sided white noise).
    let w3 = p.omega_rad_s * p.omega_rad_s * p.omega_rad_s;
    let var0 = 1.0 / (4.0 * p.zeta * w3 * p.m_kg * p.m_kg);
    Some(pc[0][0] / var0)
}

fn hmd_inputs(
    sys: &TwoDof,
    a_floor: f64,
    seismic: bool,
    power_ok: bool,
    lev_ok: bool,
) -> HmdInputs {
    HmdInputs {
        x_s_m: sys.state[0] as f32,
        v_s_mps: sys.state[1] as f32,
        x_r_m: sys.stroke_m() as f32,
        v_r_mps: (sys.state[3] - sys.state[1]) as f32,
        a_floor_mps2: a_floor as f32,
        sensors_ok: true,
        power_ok,
        levitation_ok: lev_ok,
        seismic,
        maintenance: false,
    }
}

/// Bare structure with the damper locked on (mass M + m_d, same stiffness and damping).
fn locked_primary(s: &Sentinel) -> Primary {
    let m = s.m_s + s.design().m_d_kg;
    let k = s.primary().k_n_per_m();
    let c = 2.0 * s.zeta_s * s.m_s * s.w_s;
    let w = fmath::sqrt(k / m);
    Primary {
        m_kg: m,
        omega_rad_s: w,
        zeta: c / (2.0 * m * w),
    }
}

/// SDOF (the locked-damper host) state [x, v] under ground acceleration, RK4.
fn sdof_step(p: &Primary, y: &mut [f64; 2], a_g: f64, f: f64, dt: f64) {
    let der = |y: [f64; 2]| -> [f64; 2] {
        [
            y[1],
            -2.0 * p.zeta * p.omega_rad_s * y[1] - p.omega_rad_s * p.omega_rad_s * y[0] - a_g
                + f / p.m_kg,
        ]
    };
    let k1 = der(*y);
    let k2 = der([y[0] + 0.5 * dt * k1[0], y[1] + 0.5 * dt * k1[1]]);
    let k3 = der([y[0] + 0.5 * dt * k2[0], y[1] + 0.5 * dt * k2[1]]);
    let k4 = der([y[0] + dt * k3[0], y[1] + dt * k3[1]]);
    y[0] += dt / 6.0 * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
    y[1] += dt / 6.0 * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
}

/// How the SENTINEL damper is driven in a run.
#[derive(Clone, Copy, Debug, PartialEq)]
enum HmdDrive {
    /// LSM off (passive eddy damping only): "uncontrolled".
    Off,
    /// The HMD with its ladder: LQR in wind, SEISMIC_PASSIVE above the floor threshold.
    Ladder,
    /// LQR in HMD_ACTIVE throughout (counterfactual).
    ActiveOnly,
    /// An alert is active from the start: SEISMIC_PASSIVE with the governor armed.
    Alerted,
}

/// One SENTINEL 2-DOF run: returns the host displacement series, stroke series, force
/// series, first SAT_STROKE time and the HMD.
struct SentinelRun {
    xs: Vec<f64>,
    xr: Vec<f64>,
    acc: Vec<f64>,
    force: Vec<f64>,
    power: Vec<f64>,
    t_governor: Option<f64>,
    t_stop: Option<f64>,
    e_removed_j: f64,
    int_vs2: f64,
    finite: bool,
    mode_final: HmdMode,
    seismic_at: Option<f64>,
}

#[allow(clippy::too_many_arguments)]
fn sentinel_run(
    s: &Sentinel,
    stops: bool,
    drive: HmdDrive,
    a_g: &dyn Fn(f64) -> f64,
    f_ext: &dyn Fn(usize) -> f64,
    n: usize,
    dt: f64,
    k_lqr: [f64; 4],
) -> SentinelRun {
    let mut sys = s.two_dof(stops);
    let mut hmd = s.hmd(k_lqr);
    let mut r = SentinelRun {
        xs: Vec::with_capacity(n),
        xr: Vec::with_capacity(n),
        acc: Vec::with_capacity(n),
        force: Vec::with_capacity(n),
        power: Vec::with_capacity(n),
        t_governor: None,
        t_stop: None,
        e_removed_j: 0.0,
        int_vs2: 0.0,
        finite: true,
        mode_final: HmdMode::HmdActive,
        seismic_at: None,
    };
    let mut seismic = false;
    for k in 0..n {
        let t = k as f64 * dt;
        let ag = a_g(t);
        let f = f_ext(k);
        let trigger = match drive {
            HmdDrive::Ladder => fmath::abs(ag) >= SENTINEL_SEISMIC_MPS2,
            HmdDrive::Alerted => true,
            HmdDrive::Off | HmdDrive::ActiveOnly => false,
        };
        if trigger && !seismic {
            seismic = true;
            r.seismic_at = Some(t);
        }
        let u_lsm = match drive {
            HmdDrive::Off => 0.0,
            HmdDrive::Ladder | HmdDrive::ActiveOnly | HmdDrive::Alerted => {
                let o = hmd.step(&hmd_inputs(&sys, ag, seismic, true, true), dt as f32);
                if o.sat & SAT_STROKE != 0 && r.t_governor.is_none() {
                    r.t_governor = Some(t);
                }
                r.mode_final = o.mode;
                f64::from(o.force_n)
            }
        };
        let u = u_lsm + s.drag_n(sys.state[3] - sys.state[1]);
        let v_s0 = sys.state[1];
        let f_d0 = sys.damper_force_on_structure_n(u);
        sys.step(ag, u, f, dt);
        let v_s1 = sys.state[1];
        let f_d1 = sys.damper_force_on_structure_n(u);
        r.e_removed_j -= 0.5 * (f_d0 * v_s0 + f_d1 * v_s1) * dt;
        r.int_vs2 += 0.5 * (v_s0 * v_s0 + v_s1 * v_s1) * dt;
        if stops && fmath::abs(sys.stroke_m()) > s.s_max && r.t_stop.is_none() {
            r.t_stop = Some(t + dt);
        }
        r.finite &= sys.state.iter().all(|v| v.is_finite());
        r.xs.push(sys.state[0]);
        r.xr.push(sys.stroke_m());
        r.acc.push(sys.structure_abs_accel_mps2(ag, u, f));
        r.force.push(u_lsm);
        r.power.push(u_lsm * (sys.state[3] - sys.state[1]));
    }
    r
}

// ---- HAVEN-R reduced model ---------------------------------------------------------------

/// HAVEN-R isolation controller configuration: skyhook ζ 0.3, recentring 0.01 Hz,
/// 0.1 Hz high-pass, 60 kN (the W1.5 smoke configuration).
fn haven_iso_cfg(cfg: &PlatformCfg) -> IsolationCfg {
    IsolationCfg {
        m_kg: cfg.m_kg as f32,
        f_iso_hz: (1.0 / platform::guideway_period_s(cfg.r_guideway_m)) as f32,
        zeta_sky: 0.3,
        f_rc_hz: 0.01,
        hp_hz: 0.1,
        f_max_n: 60.0e3,
        stroke_m: cfg.stroke_m as f32,
    }
}

/// HAVEN-R platform plus two isolation axes.
struct Haven {
    plat: HavenR3Dof,
    iso: [Isolator; 2],
}

impl Haven {
    fn new() -> Haven {
        Haven::with_cfg(&PlatformCfg::haven_r())
    }

    /// A platform of configuration `cfg` (a variant of the HAVEN-R reference).
    fn with_cfg(cfg: &PlatformCfg) -> Haven {
        let icfg = haven_iso_cfg(cfg);
        // The reference configurations validate (X4 tests); the default is never used.
        let plat = HavenR3Dof::new(cfg).unwrap_or_default();
        let iso = Isolator::new(&icfg).unwrap_or_default();
        Haven {
            plat,
            iso: [iso, iso],
        }
    }

    /// Isolation forces for this step.
    fn forces(&mut self, seis: SeismicState, low_soc: bool, dt: f64) -> [f64; 2] {
        let q = self.plat.state.q;
        let a = self.plat.abs_accel_mps2();
        let mut u = [0.0; 2];
        for (ax, (iso, uo)) in self.iso.iter_mut().zip(u.iter_mut()).enumerate() {
            *uo = f64::from(iso.step(
                &IsolationInputs {
                    a_abs_mps2: a[ax] as f32,
                    x_rel_m: q[ax] as f32,
                    v_rel_mps: q[3 + ax] as f32,
                    seis,
                    low_soc,
                    ..IsolationInputs::default()
                },
                dt as f32,
            ));
        }
        u
    }
}

/// Guideway-plus-kinetic mechanical energy of the platform relative to the ground, J.
fn haven_mech_energy(p: &HavenR3Dof) -> f64 {
    let c = p.cfg();
    let q = p.state.q;
    let k = if c.r_guideway_m > 0.0 {
        c.m_kg * G0_MPS2 / c.r_guideway_m
    } else {
        0.0
    };
    let rg2 = c.j_z_kgm2 / c.m_kg.max(f64::MIN_POSITIVE);
    0.5 * c.m_kg * (q[3] * q[3] + q[4] * q[4])
        + 0.5 * c.j_z_kgm2 * q[5] * q[5]
        + 0.5 * k * (q[0] * q[0] + q[1] * q[1])
        + 0.5 * k * rg2 * q[2] * q[2]
}

/// Electrical bus power of the LSM for mechanical power `p_mech` (u·v) and force `u`:
/// motoring p/η, regenerating η·p, plus copper (u/K_m)², W.
fn lsm_bus_w(p_mech: f64, u: f64, eta: f64, k_m: f64) -> f64 {
    let cu = if k_m > 0.0 {
        (u / k_m) * (u / k_m)
    } else {
        0.0
    };
    let p = if p_mech >= 0.0 {
        p_mech / eta.max(1e-3)
    } else {
        p_mech * eta
    };
    p + cu
}

/// A supercap bank with `usable_j` between 400 V and 800 V at state of charge `soc0`
/// (ESR 10 mΩ, 150 kW) \[UNCERTAIN design picks\].
fn haven_supercap(usable_j: f64, soc0: f64) -> Supercap {
    let (v_min, v_max) = (400.0, 800.0);
    let c = usable_j / (0.5 * (v_max * v_max - v_min * v_min));
    let v2 = v_min * v_min + soc0.clamp(0.0, 1.0) * (v_max * v_max - v_min * v_min);
    Supercap {
        c_f: c,
        v_v: fmath::sqrt(v2),
        v_min_v: v_min,
        v_max_v: v_max,
        esr_ohm: 0.01,
        p_max_w: 150.0e3,
    }
}

// ---- XS01 ---------------------------------------------------------------------------------

fn xs01(c: &mut Ctx, class: ExoClass) {
    let mw = c.get("mw");
    let v_p = c.get("v_pulse_mps");
    let t0 = c.get("t0_s");
    let dur = c.get("duration_s");
    let pulse = NearFaultPulse::baker2007(mw, v_p, t0);
    c.fixed("pulse_period_s", pulse.t_p_s);
    c.kpi("pga_mps2", pulse.peak_accel_mps2());
    c.kpi("pgv_mps", v_p);
    match class {
        ExoClass::Sentinel => {
            let zeta_s = c.get("zeta_s");
            c.model("2-DOF host (first mode, 100 kt modal mass, T 7 s) + 1,000 t HMD, RK4 at 20 ms; ladder LQR → SEISMIC_PASSIVE", false);
            let s = Sentinel::reference(zeta_s, TuningForm::WarburtonBase);
            let dt = SENTINEL_DT;
            let n = fmath::round(dur / dt) as usize;
            let k = sentinel_lqr(&s, dt);
            let ag = |t: f64| pulse.accel_mps2(t);
            let no_f = |_k: usize| 0.0;
            let passive = sentinel_run(&s, true, HmdDrive::Off, &ag, &no_f, n, dt, k);
            let ctrl = sentinel_run(&s, true, HmdDrive::Ladder, &ag, &no_f, n, dt, k);
            let active = sentinel_run(&s, true, HmdDrive::ActiveOnly, &ag, &no_f, n, dt, k);
            let lp = locked_primary(&s);
            let mut y = [0.0; 2];
            let mut x_locked = 0.0_f64;
            for i in 0..n {
                sdof_step(&lp, &mut y, ag(i as f64 * dt), 0.0, dt);
                x_locked = x_locked.max(fmath::abs(y[0]));
            }
            let (xp, xc, xa) = (
                peak_abs(&passive.xs),
                peak_abs(&ctrl.xs),
                peak_abs(&active.xs),
            );
            c.kpi("host_peak_uncontrolled_m", xp);
            c.kpi("host_peak_controlled_m", xc);
            c.kpi("host_peak_active_lqr_m", xa);
            c.kpi("host_peak_locked_damper_m", x_locked);
            c.kpi("ratio_controlled_to_uncontrolled", xc / xp);
            c.kpi("ratio_controlled_to_locked", xc / x_locked);
            c.kpi("ratio_active_to_uncontrolled", xa / xp);
            c.kpi("stroke_peak_m", peak_abs(&ctrl.xr));
            c.kpi("lsm_force_peak_n", peak_abs(&ctrl.force));
            c.kpi("seismic_passive_at_s", ctrl.seismic_at.unwrap_or(f64::NAN));
            c.label("hmd_mode_final", ctrl.mode_final.slug());
            c.label("lqr_gains", &format!("{k:?}"));
            c.note("\"Uncontrolled\" is the damper with its LSM off (passive eddy damping); the locked-damper host is reported alongside. A damper cannot build up before the first half-cycle of a pulse, so the criterion is only ≤ 1.02× (EXOTIC §X6).");
            c.check(
                "host_peak_le_1p02_uncontrolled",
                xc <= 1.02 * xp && ctrl.finite,
                xc / xp,
                1.02,
                format!("controlled host peak {xc:.4} m vs uncontrolled {xp:.4} m"),
            );
            // Over-design excursions end on the progressive eddy zone and the end buffers
            // (EXOTIC §X7): reported against the 0.91 MJ rating, not bounded here.
            let (k_stop, _) = Damper::from_design(&s.design(), s.s_max).stop_coefficients();
            let pen = (peak_abs(&ctrl.xr) - s.s_max).max(0.0);
            let v_max = s.w_s * s.s_max;
            c.kpi("buffer_energy_peak_j", 0.5 * k_stop * pen * pen);
            c.kpi("buffer_rating_j", 0.5 * s.design().m_d_kg * v_max * v_max);
            c.kpi("governor_first_s", ctrl.t_governor.unwrap_or(f64::NAN));
            c.note("A near-fault pulse can drive the 1,000 t damper past ±1.5 m onto its end buffers; the governor cannot arrest an over-design excursion (the trim LSM is about 10 % of the guideway restoring force), so the buffer energy is reported against its rating.");
            let mut rec = c.recorder(
                dt,
                10.0,
                &[
                    ("x_host_m", "m"),
                    ("x_host_uncontrolled_m", "m"),
                    ("stroke_m", "m"),
                    ("a_ground_mps2", "m/s²"),
                    ("lsm_force_n", "N"),
                ],
            );
            for i in 0..n {
                let t = i as f64 * dt;
                rec.push(
                    i,
                    t,
                    &[ctrl.xs[i], passive.xs[i], ctrl.xr[i], ag(t), ctrl.force[i]],
                );
            }
            c.finish(rec);
        }
        _ => {
            let y_frac = c.get("y_frac");
            c.model("HAVEN-R 3-DOF planar platform (50 t, T 6 s, moat 1.2 m, buffer from 0.9 m) + skyhook isolation, RK4 at 5 ms", false);
            let mut h = Haven::new();
            let dt = 0.005;
            let n = fmath::round(dur / dt) as usize;
            let mut seis = SeismicState::Normal;
            let mut rec = c.recorder(
                dt,
                20.0,
                &[
                    ("x_m", "m"),
                    ("y_m", "m"),
                    ("a_abs_mps2", "m/s²"),
                    ("a_ground_mps2", "m/s²"),
                ],
            );
            let mut finite = true;
            for i in 0..n {
                let t = i as f64 * dt;
                let a = pulse.accel_mps2(t);
                let ag = [a, y_frac * a];
                if fmath::abs(a) > 0.1 {
                    seis = SeismicState::Shaking;
                }
                let u = h.forces(seis, false, dt);
                h.plat.step(ag, [u[0], u[1], 0.0], dt);
                finite &= h.plat.state.q.iter().all(|v| v.is_finite());
                let q = h.plat.state.q;
                rec.push(
                    i,
                    t,
                    &[
                        q[0],
                        q[1],
                        fmath::hypot(h.plat.abs_accel_mps2()[0], h.plat.abs_accel_mps2()[1]),
                        a,
                    ],
                );
            }
            let cfg = *h.plat.cfg();
            let onset = cfg.moat_m - cfg.buffer_zone_m;
            c.kpi("max_radial_m", h.plat.max_radial_m());
            c.kpi("buffer_onset_m", onset);
            c.kpi("moat_m", cfg.moat_m);
            c.kpi("peak_abs_accel_g", h.plat.peak_abs_accel_mps2() / G0_MPS2);
            c.kpi("copper_dt_k", h.plat.state.cu_dt_k);
            c.kpi(
                "buffer_contact",
                f64::from(u8::from(h.plat.buffer_contact())),
            );
            c.check(
                "stroke_within_moat_no_buffer_contact",
                !h.plat.buffer_contact() && h.plat.max_radial_m() < onset && finite,
                h.plat.max_radial_m(),
                onset,
                "HAVEN-R max radial excursion vs the buffer onset (moat 1.2 m − 0.3 m buffer), m"
                    .into(),
            );
            c.finish(rec);
        }
    }
}

// ---- XS02 ---------------------------------------------------------------------------------

fn xs02(c: &mut Ctx) {
    let d = c.get("d_m");
    let t_r = c.get("t_r_s");
    let t0 = c.get("t0_s");
    let dur = c.get("duration_s");
    let design_g = c.get("design_decel_g");
    let margin = c.get("support_margin_m");
    c.model("HAVEN-R 3-DOF planar platform + skyhook; progressive buffer, moat wall; PM-bias-ramp landing onto the FPS catcher 30 s before the end", false);
    let fling = FlingStep {
        d_m: d,
        t_r_s: t_r,
        t0_s: t0,
    };
    c.kpi("fling_pga_mps2", fling.peak_accel_mps2());
    c.kpi("fling_pgv_mps", 2.0 * d / t_r);
    let mut h = Haven::new();
    let dt = 0.002;
    let n = fmath::round(dur / dt) as usize;
    let t_land = (dur - 30.0).max(t0 + t_r + 5.0);
    let mut seis = SeismicState::Normal;
    let mut rec = c.recorder(
        dt,
        20.0,
        &[("x_m", "m"), ("a_abs_mps2", "m/s²"), ("ground_disp_m", "m")],
    );
    let mut finite = true;
    let mut v_end = 0.0;
    for i in 0..n {
        let t = i as f64 * dt;
        let a = fling.accel_mps2(t);
        if fmath::abs(a) > 0.1 {
            seis = SeismicState::Shaking;
        }
        if t >= t_land && !h.plat.control_lost() {
            h.plat.lose_control();
        }
        let u = h.forces(seis, false, dt);
        h.plat.step([a, 0.0], [u[0], u[1], 0.0], dt);
        finite &= h.plat.state.q.iter().all(|v| v.is_finite());
        let q = h.plat.state.q;
        v_end = fmath::hypot(q[3], q[4]);
        rec.push(
            i,
            t,
            &[q[0], h.plat.abs_accel_mps2()[0], fling.displacement_m(t)],
        );
    }
    let cfg = *h.plat.cfg();
    let peak_g = h.plat.peak_abs_accel_mps2() / G0_MPS2;
    let e_mj = h.plat.buffer_peak_energy_j() / 1.0e6;
    let exceeded = h.plat.buffer_contact();
    let touchdown = h.plat.touchdown_speed_mps().unwrap_or(f64::NAN);
    c.kpi("max_radial_m", h.plat.max_radial_m());
    c.kpi("moat_exceeded", f64::from(u8::from(exceeded)));
    c.kpi("bumper_energy_mj", e_mj);
    c.kpi("peak_decel_g", peak_g);
    c.kpi("touchdown_mps", touchdown);
    c.kpi("final_speed_mps", v_end);
    c.label(
        "exceedance",
        if exceeded {
            "moat exceeded (expected for a fling of metres)"
        } else {
            "within the moat"
        },
    );
    c.note("Exceedance is expected: a fling of metres cannot fit a 1.2 m moat; the bumper energy says what the buffer must absorb.");
    c.check(
        "peak_deceleration_within_design",
        peak_g <= design_g,
        peak_g,
        design_g,
        "peak platform acceleration vs the buffer design deceleration, g".into(),
    );
    c.check(
        "bumper_energy_quoted_mj",
        e_mj.is_finite() && (!exceeded || e_mj > 0.0),
        e_mj,
        f64::NAN,
        "peak energy stored in the moat buffer, MJ".into(),
    );
    let extent = cfg.moat_m + margin;
    c.check(
        "support_never_lost",
        h.plat.max_radial_m() <= extent && finite,
        h.plat.max_radial_m(),
        extent,
        "max radial excursion vs the guideway extent (moat + margin), m".into(),
    );
    c.check(
        "stable_landing",
        touchdown <= platform::HAVEN_LANDING_V_MAX_MPS && v_end < 0.01,
        touchdown,
        platform::HAVEN_LANDING_V_MAX_MPS,
        format!("PM-bias-ramp touchdown speed (m/s); final platform speed {v_end:.4} m/s < 0.01"),
    );
    c.finish(rec);
}

// ---- XS03 ---------------------------------------------------------------------------------

/// Outcome of a HAVEN-R run on a supercap bus.
struct HavenBusRun {
    e_ground_j: f64,
    e_lsm_j: f64,
    e_cu_j: f64,
    d_mech_j: f64,
    soc_min: f64,
    soc_end: f64,
    ledger: EnergyLedger,
    ledger_mj: EnergyLedgerMj,
    fps_fallback: bool,
    finite: bool,
}

fn xs03(c: &mut Ctx) {
    let dur = c.get("duration_s");
    let pga_x = c.get("pga_x_g") * G0_MPS2;
    let pga_y = c.get("pga_y_g") * G0_MPS2;
    let store_j = c.get("store_kwh") * 3.6e6;
    let soc0 = c.get("soc0");
    let p_grid = c.get("p_grid_w");
    let p_sens = c.get("p_sensors_w");
    let eta = c.get("eta_drive");
    let k_m = c.get("k_m_n_per_sqrt_w");
    c.model("HAVEN-R 3-DOF planar platform + skyhook LSM on a supercap bus (grid charging), mechanical and storage energy ledgers, RK4 at 5 ms", false);
    let dt = 0.005;
    let fs = 1.0 / dt;
    let gx = ground::m9_long_duration_series(dur, fs, pga_x, c.seed);
    let gy = ground::m9_long_duration_series(dur, fs, pga_y, c.seed.wrapping_add(1));
    let mut h = Haven::new();
    let mut store = haven_supercap(store_j, soc0);
    let mut led = EnergyLedger::open(&store);
    let mut led_mj = EnergyLedgerMj {
        e_start_mj: storage::j_to_mj(led.e_start_j),
        e_end_mj: storage::j_to_mj(led.e_start_j),
        ..EnergyLedgerMj::default()
    };
    let n = gx.len().min(gy.len());
    let mut r = HavenBusRun {
        e_ground_j: 0.0,
        e_lsm_j: 0.0,
        e_cu_j: 0.0,
        d_mech_j: 0.0,
        soc_min: 1.0,
        soc_end: 0.0,
        ledger: led,
        ledger_mj: led_mj,
        fps_fallback: false,
        finite: true,
    };
    let e0 = haven_mech_energy(&h.plat);
    let m = h.plat.cfg().m_kg;
    let mut rec = c.recorder(
        dt,
        10.0,
        &[
            ("x_m", "m"),
            ("y_m", "m"),
            ("soc", ""),
            ("cu_dt_k", "K"),
            ("p_bus_w", "W"),
        ],
    );
    let mut acc_sq = 0.0;
    for i in 0..n {
        let t = i as f64 * dt;
        let ag = [gx[i], gy[i]];
        acc_sq += ag[0] * ag[0] + ag[1] * ag[1];
        let soc = fabrail_exo::energy::EnergyStore::soc(&store);
        let low = soc < 0.2;
        let u = h.forces(SeismicState::Shaking, low, dt);
        let q0 = h.plat.state.q;
        let cu0 = h.plat.copper_energy_j();
        h.plat.step(ag, [u[0], u[1], 0.0], dt);
        let q1 = h.plat.state.q;
        r.e_cu_j += h.plat.copper_energy_j() - cu0;
        for ax in 0..2 {
            let vbar = 0.5 * (q0[3 + ax] + q1[3 + ax]);
            r.e_ground_j += -m * ag[ax] * vbar * dt;
            r.e_lsm_j += u[ax] * vbar * dt;
        }
        // Bus: hold power (sensing + hybrid hover loss at the running RMS) + LSM, minus grid.
        let a_rms = fmath::sqrt(acc_sq / (i + 1) as f64);
        let p_hover = suspension::hybrid_hover_power_w(m, a_rms);
        let p_lsm: f64 = (0..2)
            .map(|ax| lsm_bus_w(u[ax] * q1[3 + ax], u[ax], eta, k_m))
            .sum();
        let p_bus = p_sens + p_hover + p_lsm - p_grid;
        let delivered = storage::exchange(&mut store, p_bus, dt, &mut led, &mut led_mj);
        if p_bus > 0.0 && delivered < 0.999 * p_bus && soc <= 1.0e-3 {
            r.fps_fallback = true;
            h.plat.lose_control();
        }
        let soc_now = fabrail_exo::energy::EnergyStore::soc(&store);
        r.soc_min = r.soc_min.min(soc_now);
        r.finite &= q1.iter().all(|v| v.is_finite());
        rec.push(i, t, &[q1[0], q1[1], soc_now, h.plat.state.cu_dt_k, p_bus]);
    }
    r.d_mech_j = haven_mech_energy(&h.plat) - e0;
    r.soc_end = fabrail_exo::energy::EnergyStore::soc(&store);
    r.ledger = led;
    r.ledger_mj = led_mj;
    r.fps_fallback |= h.plat.control_lost();
    let residual = r.d_mech_j - (r.e_ground_j + r.e_lsm_j - r.e_cu_j);
    let scale = r.e_cu_j + fmath::abs(r.e_lsm_j) + fmath::abs(r.d_mech_j);
    let rel_mech = fmath::abs(residual) / scale.max(1.0);
    let rel_store = r.ledger.balance_error();
    let rel_store_mj = r.ledger_mj.balance_error();
    let cfg = *h.plat.cfg();
    c.kpi("e_ground_in_mj", r.e_ground_j / 1e6);
    c.kpi("e_lsm_mj", r.e_lsm_j / 1e6);
    c.kpi("e_copper_mj", r.e_cu_j / 1e6);
    c.kpi("d_mech_mj", r.d_mech_j / 1e6);
    c.kpi("mech_balance_rel", rel_mech);
    c.kpi("store_balance_rel", rel_store);
    c.kpi("store_balance_rel_mj_ledger", rel_store_mj);
    c.kpi("soc_min", r.soc_min);
    c.kpi("soc_end", r.soc_end);
    c.kpi("copper_dt_k", h.plat.state.cu_dt_k);
    c.kpi("copper_dt_limit_k", cfg.cu_dt_max_k);
    c.kpi("max_radial_m", h.plat.max_radial_m());
    c.kpi(
        "buffer_contact",
        f64::from(u8::from(h.plat.buffer_contact())),
    );
    c.kpi("store_out_mj", r.ledger.e_out_j / 1e6);
    c.kpi("store_in_mj", r.ledger.e_in_j / 1e6);
    if h.plat.buffer_contact() {
        c.note("The platform touched the moat buffer: the buffer's contact damping is not in the mechanical ledger.");
    }
    c.check(
        "energy_balance_within_1pct",
        rel_mech <= 0.01 && rel_store <= 0.01 && rel_store_mj <= 0.01 && r.finite,
        rel_mech.max(rel_store).max(rel_store_mj),
        0.01,
        "relative residual of the platform ledger (ground + LSM − copper − ΔE) and of the storage ledgers (f64 and i64 mJ)".into(),
    );
    c.check(
        "copper_dt_within_limit",
        h.plat.copper_within_limit(),
        h.plat.state.cu_dt_k,
        cfg.cu_dt_max_k,
        "copper temperature rise, K".into(),
    );
    c.check(
        "soc_positive",
        r.soc_min > 0.0,
        r.soc_min,
        0.0,
        "minimum state of charge".into(),
    );
    c.check(
        "no_fps_fallback",
        !r.fps_fallback,
        f64::from(u8::from(r.fps_fallback)),
        0.0,
        "control never lost (no landing on the FPS catcher)".into(),
    );
    c.finish(rec);
}

// ---- XS04 ---------------------------------------------------------------------------------

fn xs04(c: &mut Ctx) {
    let dur = c.get("duration_s");
    let t_g = c.get("t_g_s");
    let zeta_g = c.get("zeta_g");
    let demand = c.get("stroke_demand");
    let zeta_s = c.get("zeta_s");
    c.model("2-DOF host (first mode, T 7 s) + 1,000 t HMD in SEISMIC_PASSIVE (stroke governor armed), basin record, RK4 at 20 ms", false);
    let s = Sentinel::reference(zeta_s, TuningForm::WarburtonBase);
    let dt = SENTINEL_DT;
    let fs = 1.0 / dt;
    let n = fmath::round(dur / dt) as usize;
    let unit = ground::basin_series(
        &BasinResonance {
            omega_g_rad_s: TAU / t_g,
            zeta_g,
            a_rms_mps2: 1.0,
            duration_s: dur,
        },
        fs,
        n,
        c.seed,
    );
    let k = sentinel_lqr(&s, dt);
    let no_f = |_k: usize| 0.0;
    // Linear probe (no stops, LSM off) at unit RMS, then scale to the stroke demand.
    let probe = {
        let a = |t: f64| unit[((t / dt) as usize).min(n.saturating_sub(1))];
        sentinel_run(&s, false, HmdDrive::Off, &a, &no_f, n, dt, k)
    };
    let s_probe = peak_abs(&probe.xr);
    let scale = if s_probe > 0.0 {
        demand * s.s_max / s_probe
    } else {
        0.0
    };
    let series: Vec<f64> = unit.iter().map(|v| v * scale).collect();
    c.kpi("ground_rms_mps2", scale);
    c.kpi("ground_peak_mps2", peak_abs(&series));
    let ag = |t: f64| series[((t / dt) as usize).min(n.saturating_sub(1))];
    // A long-period alert is active: SEISMIC_PASSIVE with the governor armed throughout.
    let run = sentinel_run(&s, true, HmdDrive::Alerted, &ag, &no_f, n, dt, k);
    // Overdrive what-if at 1.15× the design basis: end-stop contact and a lower ζ_add.
    let over_scale = 1.15 / demand.max(1e-9);
    let ag_over = |t: f64| over_scale * series[((t / dt) as usize).min(n.saturating_sub(1))];
    let over = sentinel_run(&s, true, HmdDrive::Alerted, &ag_over, &no_f, n, dt, k);
    let over_ratio = {
        let b = fabrail_exo::tuning::zeta_add_bound(s.mu, rms(&over.xr) / rms(&over.xs));
        let z = if over.int_vs2 > 0.0 {
            over.e_removed_j / (2.0 * s.m_s * s.w_s * over.int_vs2)
        } else {
            0.0
        };
        z / b
    };
    c.kpi("overdrive_1p15_zeta_add_over_bound", over_ratio);
    c.kpi(
        "overdrive_1p15_t_governor_s",
        over.t_governor.unwrap_or(f64::INFINITY),
    );
    c.kpi(
        "overdrive_1p15_t_end_stop_s",
        over.t_stop.unwrap_or(f64::INFINITY),
    );
    c.kpi("overdrive_1p15_stroke_peak_m", peak_abs(&over.xr));
    let x_rms = rms(&run.xs);
    let s_rms = rms(&run.xr);
    let bound = fabrail_exo::tuning::zeta_add_bound(s.mu, s_rms / x_rms);
    let zeta_add = if run.int_vs2 > 0.0 {
        run.e_removed_j / (2.0 * s.m_s * s.w_s * run.int_vs2)
    } else {
        0.0
    };
    let ratio = zeta_add / bound;
    let t_gov = run.t_governor.unwrap_or(f64::INFINITY);
    let t_stop = run.t_stop.unwrap_or(f64::INFINITY);
    c.kpi("stroke_peak_m", peak_abs(&run.xr));
    c.kpi("stroke_rms_m", s_rms);
    c.kpi("host_rms_m", x_rms);
    c.kpi("stroke_ratio_s_over_x", s_rms / x_rms);
    c.kpi("zeta_add", zeta_add);
    c.kpi("zeta_add_bound", bound);
    c.kpi("zeta_add_over_bound", ratio);
    c.kpi("t_governor_first_s", t_gov);
    c.kpi("t_end_stop_first_s", t_stop);
    c.kpi("passive_probe_peak_stroke_over_smax", demand);
    c.label("hmd_mode_final", run.mode_final.slug());
    c.note("The governor bounds what the controller does near the ends; it cannot arrest a damper driven past the design basis (the trim LSM is about 10 % of the guideway restoring force at full stroke), so end-stop contact after the governor is possible and reported.");
    c.note("ζ_add is the energy the damper assembly removes from the host over the record, divided by 2·M·ω_s·∫ẋ_s²; the bound uses RMS S/X. The same measure gives 0.98 of the bound under harmonic excitation at ω_s; under random narrow-band base excitation the passive damper reaches about 0.67–0.76 of it, so the two-sided ±20 % criterion can fail while the bound itself holds.");
    c.check(
        "governor_before_end_stops",
        run.t_governor.is_some() && t_gov < t_stop && run.finite,
        t_gov,
        t_stop,
        "first SAT_STROKE time vs first end-stop contact, s".into(),
    );
    c.check(
        "zeta_add_le_bound_plus_0p002",
        zeta_add <= bound + 0.002,
        zeta_add,
        bound + 0.002,
        "ζ_add ≤ (μ/2)(S/X) + 0.002 (FAB-EXO-603 stroke bound)".into(),
    );
    c.check(
        "zeta_add_within_20pct_of_bound",
        fmath::abs(ratio - 1.0) <= 0.2,
        ratio,
        1.0,
        format!("ζ_add {zeta_add:.5} vs (μ/2)(S/X) {bound:.5} (RMS S/X), ±20 %"),
    );
    let mut rec = c.recorder(
        dt,
        5.0,
        &[
            ("x_host_m", "m"),
            ("stroke_m", "m"),
            ("a_ground_mps2", "m/s²"),
            ("lsm_force_n", "N"),
        ],
    );
    for (i, a) in series.iter().enumerate().take(n) {
        rec.push(i, i as f64 * dt, &[run.xs[i], run.xr[i], *a, run.force[i]]);
    }
    c.finish(rec);
}

// ---- XS05 ---------------------------------------------------------------------------------

fn xs05(c: &mut Ctx) {
    let a_pk = c.get("a_peak_g");
    let mw = c.get("mw");
    let f_air = c.get("f_air_hz");
    let z_air = c.get("zeta_air");
    let stroke = c.get("stroke_air_m");
    let trim = c.get("trim_frac");
    let dur = c.get("duration_s");
    c.model("HAVEN-R vertical 1-DOF: 50 t on passive air springs + saturating hybrid-EMS trim (skyhook), RK4 at 1 ms", false);
    let m = PlatformCfg::haven_r().m_kg;
    let w = TAU * f_air;
    let k = m * w * w;
    let cc = 2.0 * z_air * m * w;
    let f_trim = trim * m * G0_MPS2;
    let half = ground::NearFaultPulse::baker2007(mw, 1.0, 0.0).half_duration_s();
    let pulse = ground::vertical_pulse(mw, a_pk, half + 1.0);
    c.kpi("static_deflection_m", G0_MPS2 / (w * w));
    let dt = 0.001;
    let n = fmath::round(dur / dt) as usize;
    let (mut z, mut v, mut v_abs) = (0.0_f64, 0.0_f64, 0.0_f64);
    let c_sky = 2.0 * 0.5 * m * w;
    let (mut sat_run, mut t_sat_first, mut t_detect) = (0_u32, None::<f64>, None::<f64>);
    let (mut z_max, mut f_air_min, mut f_air_max, mut u_max) =
        (0.0_f64, f64::INFINITY, 0.0_f64, 0.0_f64);
    let mut rec = c.recorder(
        dt,
        100.0,
        &[
            ("z_rel_m", "m"),
            ("f_air_n", "N"),
            ("u_trim_n", "N"),
            ("a_ground_mps2", "m/s²"),
        ],
    );
    for i in 0..n {
        let t = i as f64 * dt;
        let ag = pulse.accel_mps2(t);
        let u_cmd = -c_sky * v_abs;
        let saturated = fmath::abs(u_cmd) >= f_trim;
        let u = u_cmd.clamp(-f_trim, f_trim);
        if saturated {
            if t_sat_first.is_none() {
                t_sat_first = Some(t);
            }
            sat_run += 1;
            if sat_run >= 3 && t_detect.is_none() {
                t_detect = Some(t);
            }
        } else {
            sat_run = 0;
        }
        // m·z̈ = −k·z − c·ż + u − m·a_g (z relative to the ground, static preload W).
        let acc = |z: f64, v: f64| (-k * z - cc * v + u) / m - ag;
        let (k1v, k1z) = (acc(z, v), v);
        let (k2v, k2z) = (
            acc(z + 0.5 * dt * k1z, v + 0.5 * dt * k1v),
            v + 0.5 * dt * k1v,
        );
        let (k3v, k3z) = (
            acc(z + 0.5 * dt * k2z, v + 0.5 * dt * k2v),
            v + 0.5 * dt * k2v,
        );
        let (k4v, k4z) = (acc(z + dt * k3z, v + dt * k3v), v + dt * k3v);
        z += dt / 6.0 * (k1z + 2.0 * k2z + 2.0 * k3z + k4z);
        v += dt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
        // Absolute velocity for the skyhook: v_abs = ż + v_ground.
        v_abs = v + pulse.velocity_mps(t + dt);
        let f_air = m * G0_MPS2 - k * z - cc * v;
        z_max = z_max.max(fmath::abs(z));
        f_air_min = f_air_min.min(f_air);
        f_air_max = f_air_max.max(f_air);
        u_max = u_max.max(fmath::abs(u));
        rec.push(i, t, &[z, f_air, u, ag]);
    }
    // Supply-loss behaviour of the hybrid-EMS elements (zero-power: indeterminate).
    let door = fabrail_params::presets::door();
    let ems = HybridEmsAdapter::new(
        door.design.suspension,
        1.0,
        fabrail_exo_safety::PoleMaterial::Fe,
    );
    let indeterminate = matches!(
        ems.on_supply_loss(),
        FailureBehaviour::Diverge {
            bias: Bias::Indeterminate,
            ..
        }
    );
    let det_latency = match (t_sat_first, t_detect) {
        (Some(a), Some(b)) => b - a,
        _ => f64::NAN,
    };
    c.kpi("ems_trim_limit_n", f_trim);
    c.kpi("ems_trim_peak_n", u_max);
    c.kpi("t_saturation_first_s", t_sat_first.unwrap_or(f64::NAN));
    c.kpi("t_saturation_detected_s", t_detect.unwrap_or(f64::NAN));
    c.kpi("detection_latency_s", det_latency);
    c.kpi("air_deflection_peak_m", z_max);
    c.kpi("air_force_min_n", f_air_min);
    c.kpi("air_force_max_n", f_air_max);
    c.kpi("weight_n", m * G0_MPS2);
    c.label("supply_loss", if indeterminate { "Diverge{Indeterminate} onto designed sliding stops (upper ≤ 1.3 W, lower FPS/rollers); no clamp-up claim" } else { "not indeterminate" });
    c.note("Vertical isolation is passive (FAB-EXO-705): the air springs carry the load and the hybrid EMS is trim only.");
    c.check(
        "ems_saturation_detected",
        t_detect.is_some() && det_latency <= 0.003 + 1e-9,
        det_latency,
        0.003,
        "trim saturation flagged after 3 consecutive saturated samples (latency from the first saturated sample, s)".into(),
    );
    c.check(
        "passive_support_carries_load",
        z_max <= stroke && f_air_min > 0.0,
        z_max,
        stroke,
        format!("air-spring deflection within its stroke (m) and always in compression (min force {f_air_min:.0} N > 0)"),
    );
    c.check(
        "supply_loss_indeterminate_onto_stops",
        indeterminate,
        f64::from(u8::from(indeterminate)),
        1.0,
        "hybrid-EMS supply loss diverges in an indeterminate direction onto designed stops".into(),
    );
    c.finish(rec);
}

// ---- XS06 ---------------------------------------------------------------------------------

fn xs06(c: &mut Ctx) {
    let t_shake = c.get("t_shaking_s");
    let pga = c.get("pga_g") * G0_MPS2;
    let t_s = c.get("t_s_s");
    let store_j = c.get("store_kwh") * 3.6e6;
    let soc0 = c.get("soc0");
    let p_sens = c.get("p_sensors_w");
    let eta = c.get("eta_drive");
    let k_m = c.get("k_m_n_per_sqrt_w");
    c.model("HAVEN-R 3-DOF planar platform + skyhook LSM on islanded supercap storage after grid loss; PM-bias-ramp landing, RK4 at 5 ms", false);
    let ramp = platform::LandingRamp::default();
    let t_land_est = ramp.t_ramp_s + 10.0 / ramp.omega_cl_rad_s;
    let t_ride = energy::t_ride_s(t_shake, t_land_est);
    c.kpi("t_ride_s", t_ride);
    let t_land_cmd = t_s + energy::T_RIDE_MIN_S.max(t_shake);
    let dur = t_s + t_ride + 5.0;
    let dt = 0.005;
    let fs = 1.0 / dt;
    let gx = ground::m9_long_duration_series(t_s + t_shake, fs, pga, c.seed);
    let gy = ground::m9_long_duration_series(t_s + t_shake, fs, 0.7 * pga, c.seed.wrapping_add(1));
    let n = fmath::round(dur / dt) as usize;
    let mut h = Haven::new();
    let m = h.plat.cfg().m_kg;
    let mut store = haven_supercap(store_j, soc0);
    let mut led = EnergyLedger::open(&store);
    let mut led_mj = EnergyLedgerMj {
        e_start_mj: storage::j_to_mj(led.e_start_j),
        e_end_mj: storage::j_to_mj(led.e_start_j),
        ..EnergyLedgerMj::default()
    };
    let (mut soc_min, mut soc_touch, mut shortfall) = (1.0_f64, f64::NAN, false);
    let mut acc_sq = 0.0;
    let mut rec = c.recorder(dt, 5.0, &[("x_m", "m"), ("soc", ""), ("p_bus_w", "W")]);
    let mut e_island = 0.0;
    for i in 0..n {
        let t = i as f64 * dt;
        let ag = [
            gx.get(i).copied().unwrap_or(0.0),
            gy.get(i).copied().unwrap_or(0.0),
        ];
        acc_sq += ag[0] * ag[0] + ag[1] * ag[1];
        let shaking = t <= t_s + t_shake;
        let seis = if shaking {
            SeismicState::Shaking
        } else {
            SeismicState::Quiet
        };
        if t >= t_land_cmd && !h.plat.control_lost() {
            // Planned, energised landing at the end of the ride-through window.
            h.plat.lose_control();
        }
        let soc = fabrail_exo::energy::EnergyStore::soc(&store);
        let u = h.forces(seis, soc < 0.2, dt);
        h.plat.step(ag, [u[0], u[1], 0.0], dt);
        let q = h.plat.state.q;
        let a_rms = fmath::sqrt(acc_sq / (i + 1) as f64);
        let p_hover = if h.plat.state.landed {
            0.0
        } else {
            suspension::hybrid_hover_power_w(m, a_rms)
        };
        let p_lsm: f64 = if h.plat.control_lost() {
            0.0
        } else {
            (0..2)
                .map(|ax| lsm_bus_w(u[ax] * q[3 + ax], u[ax], eta, k_m))
                .sum()
        };
        // The landing ramp is energised: the pods draw hold power until touchdown.
        let p_bus = p_sens + p_hover + p_lsm;
        if t >= t_s {
            let p = storage::exchange(&mut store, p_bus, dt, &mut led, &mut led_mj);
            if p_bus > 0.0 && p < 0.999 * p_bus {
                shortfall = true;
            }
            e_island += p * dt;
        }
        let soc_now = fabrail_exo::energy::EnergyStore::soc(&store);
        soc_min = soc_min.min(soc_now);
        if h.plat.state.landed && soc_touch.is_nan() {
            soc_touch = soc_now;
        }
        rec.push(i, t, &[q[0], soc_now, p_bus]);
    }
    let touchdown = h.plat.touchdown_speed_mps().unwrap_or(f64::NAN);
    c.kpi("t_grid_loss_s", t_s);
    c.kpi("t_land_command_s", t_land_cmd);
    c.kpi("e_islanded_mj", e_island / 1e6);
    c.kpi("store_usable_mj", store_j * soc0 / 1e6);
    c.kpi("soc_min", soc_min);
    c.kpi("soc_at_touchdown", soc_touch);
    c.kpi("touchdown_mps", touchdown);
    c.kpi("max_radial_m", h.plat.max_radial_m());
    c.kpi("store_balance_rel", led.balance_error());
    c.check(
        "ride_through_t_ride_islanded",
        !shortfall && soc_min > 0.0 && h.plat.state.landed,
        soc_min,
        0.0,
        format!("islanded for T_ride = max(300 s, shaking) + landing = {t_ride:.1} s with no power shortfall; minimum SOC > 0"),
    );
    c.check(
        "energised_controlled_descent",
        touchdown <= platform::HAVEN_LANDING_V_MAX_MPS && soc_touch > 0.0,
        touchdown,
        platform::HAVEN_LANDING_V_MAX_MPS,
        "PM-bias-ramp touchdown speed on stored energy, m/s".into(),
    );
    c.finish(rec);
}

// ---- XS07 ---------------------------------------------------------------------------------

fn xs07(c: &mut Ctx, class: ExoClass) {
    let pga = c.get("pga_g") * G0_MPS2;
    let dur = c.get("duration_s");
    let t_fault = c.get("t_fault_s").min(dur - 1.0);
    let fps_mu = c.get("fps_mu");
    match class {
        ExoClass::Sentinel => {
            let t_lev = c.get("t_lev_loss_s");
            let gap = c.get("gap_m");
            c.model("2-DOF host + HMD mode ladder: LSM fault → PASSIVE_EDDY → levitation loss → LANDED_FPS (friction-pendulum TMD, Coulomb μ·m_d·g), RK4 at 20 ms", false);
            let s = Sentinel::reference(0.02, TuningForm::WarburtonBase);
            let dt = SENTINEL_DT;
            let fs = 1.0 / dt;
            let g = ground::m9_long_duration_series(dur, fs, pga, c.seed);
            let n = g.len();
            let k = sentinel_lqr(&s, dt);
            let mut sys = s.two_dof(true);
            let mut hmd = s.hmd(k);
            let d = s.design();
            let (mut modes, mut t_landed) = (Vec::<HmdMode>::new(), None::<f64>);
            let mut e_buf_max = 0.0_f64;
            let (k_stop, _) = sys.damper.stop_coefficients();
            let mut finite = true;
            let mut rec = c.recorder(
                dt,
                10.0,
                &[("x_host_m", "m"), ("stroke_m", "m"), ("mode", "")],
            );
            let mut seismic = false;
            for (i, &ag) in g.iter().enumerate().take(n) {
                let t = i as f64 * dt;
                let power_ok = t < t_fault;
                let lev_ok = t < t_fault + t_lev;
                // Latched: the ladder keeps its seismic demand once the ground crossed 5 gal.
                seismic |= fmath::abs(ag) >= SENTINEL_SEISMIC_MPS2;
                let o = hmd.step(&hmd_inputs(&sys, ag, seismic, power_ok, lev_ok), dt as f32);
                if modes.last() != Some(&o.mode) {
                    modes.push(o.mode);
                }
                let v_r = sys.state[3] - sys.state[1];
                let mut u = f64::from(o.force_n);
                if o.mode == HmdMode::LandedFps {
                    if t_landed.is_none() {
                        t_landed = Some(t);
                    }
                    // Friction-pendulum catcher of the guideway radius: the spring is the
                    // same (k = W/R), plus Coulomb friction μ·m_d·g.
                    u += -fps_mu
                        * d.m_d_kg
                        * G0_MPS2
                        * fmath::tanh(v_r / suspension::DRAG_V_REG_MPS);
                } else {
                    u += s.drag_n(v_r);
                }
                sys.step(ag, u, 0.0, dt);
                finite &= sys.state.iter().all(|v| v.is_finite());
                let pen = (fmath::abs(sys.stroke_m()) - s.s_max).max(0.0);
                e_buf_max = e_buf_max.max(0.5 * k_stop * pen * pen);
                rec.push(
                    i,
                    t,
                    &[sys.state[0], sys.stroke_m(), f64::from(o.mode as u8)],
                );
            }
            // Rating ≥ ½·m·v_max² at the kinematic budget v = ω·S (EXOTIC §X14: 0.91 MJ).
            let v_max = s.w_s * s.s_max;
            let rating = 0.5 * d.m_d_kg * v_max * v_max;
            let drop_v = fmath::sqrt(2.0 * G0_MPS2 * gap);
            let seq: Vec<&str> = modes.iter().map(|m| m.slug()).collect();
            c.label("mode_sequence", &seq.join(" → "));
            c.kpi("t_landed_fps_s", t_landed.unwrap_or(f64::NAN));
            c.kpi("buffer_energy_peak_j", e_buf_max);
            c.kpi("buffer_rating_j", rating);
            c.kpi("touchdown_drop_mps", drop_v);
            c.kpi("drop_energy_j", d.m_d_kg * G0_MPS2 * gap);
            c.note("The supply-loss direction of a zero-power hybrid EMS is indeterminate; the scenario takes the lower stop (the FPS catcher). The upper stop is a designed sliding surface at ≤ 1.3 W.");
            let landed = modes.last() == Some(&HmdMode::LandedFps);
            c.check(
                "degrades_to_friction_pendulum_tmd",
                landed && modes.contains(&HmdMode::PassiveEddy) && finite,
                f64::from(u8::from(landed)),
                1.0,
                "ladder reaches LANDED_FPS through PASSIVE_EDDY and stays latched".into(),
            );
            c.check(
                "impact_within_buffer_rating",
                e_buf_max <= rating,
                e_buf_max,
                rating,
                "peak end-stop buffer energy vs the rating ½·m_d·(ω·S)², J".into(),
            );
            c.finish(rec);
        }
        _ => {
            c.model("HAVEN-R 3-DOF planar platform: control loss mid-shaking → PM-bias-ramp landing → FPS isolation, RK4 at 5 ms", false);
            let dt = 0.005;
            let fs = 1.0 / dt;
            let gx = ground::m9_long_duration_series(dur, fs, pga, c.seed);
            let gy = ground::m9_long_duration_series(dur, fs, 0.7 * pga, c.seed.wrapping_add(1));
            let n = gx.len().min(gy.len());
            let mut pcfg = PlatformCfg::haven_r();
            pcfg.fps_mu = fps_mu;
            let mut h = Haven::with_cfg(&pcfg);
            let mut finite = true;
            let mut rec = c.recorder(dt, 10.0, &[("x_m", "m"), ("y_m", "m"), ("landed", "")]);
            for i in 0..n {
                let t = i as f64 * dt;
                if t >= t_fault && !h.plat.control_lost() {
                    h.plat.lose_control();
                }
                let u = h.forces(SeismicState::Shaking, false, dt);
                h.plat.step([gx[i], gy[i]], [u[0], u[1], 0.0], dt);
                finite &= h.plat.state.q.iter().all(|v| v.is_finite());
                let q = h.plat.state.q;
                rec.push(
                    i,
                    t,
                    &[q[0], q[1], f64::from(u8::from(h.plat.state.landed))],
                );
            }
            let cfg = *h.plat.cfg();
            let rating = 0.375 * cfg.m_kg * G0_MPS2 * cfg.buffer_zone_m;
            let touchdown = h.plat.touchdown_speed_mps().unwrap_or(f64::NAN);
            c.kpi("touchdown_mps", touchdown);
            c.kpi("buffer_energy_peak_j", h.plat.buffer_peak_energy_j());
            c.kpi("buffer_rating_j", rating);
            c.kpi("max_radial_m", h.plat.max_radial_m());
            c.check(
                "degrades_to_fps_isolation",
                h.plat.state.landed && touchdown <= platform::HAVEN_LANDING_V_MAX_MPS && finite,
                touchdown,
                platform::HAVEN_LANDING_V_MAX_MPS,
                "landed on the FPS catcher through the PM-bias ramp; touchdown speed, m/s".into(),
            );
            c.check(
                "impact_within_buffer_rating",
                h.plat.buffer_peak_energy_j() <= rating,
                h.plat.buffer_peak_energy_j(),
                rating,
                "peak moat-buffer energy vs its capacity (0.375·m·g·zone at 1 g full compression), J".into(),
            );
            c.finish(rec);
        }
    }
}

// ---- XS08 ---------------------------------------------------------------------------------

/// Trace of one fusion run.
struct FusionTrace {
    /// Every released alert had P(I ≥ I_crit) above the C/L threshold of its level.
    rational: bool,
    first_alert_s: Option<f64>,
    max_level: u8,
    p_at_first: f32,
    cancel_s: Option<f64>,
    normal_at_end: bool,
    injected_or_single_feed: bool,
    alert: Option<fabrail_exo_safety::FusedAlert>,
}

fn run_fusion(feeds: &[(f64, Feed)], dur_s: f64) -> FusionTrace {
    let mut fu = Fusion::new(FusionCfg::default());
    let dt = 0.01;
    let n = fmath::round(dur_s / dt) as usize;
    let cfg = FusionCfg::default();
    let mut tr = FusionTrace {
        rational: true,
        first_alert_s: None,
        max_level: 0,
        p_at_first: 0.0,
        cancel_s: None,
        normal_at_end: false,
        injected_or_single_feed: false,
        alert: None,
    };
    let mut next = 0;
    for i in 0..=n {
        let t = i as f64 * dt;
        let tick = Tick::from_seconds_f64(t);
        while next < feeds.len() && feeds[next].0 <= t + 1e-12 {
            let mut f = feeds[next].1;
            f.received = tick;
            fu.push(&f);
            next += 1;
        }
        if let Some(a) = fu.step(tick, &LocalDetection::default()) {
            if a.alert.cancel {
                if tr.cancel_s.is_none() {
                    tr.cancel_s = Some(t);
                }
            } else {
                if tr.first_alert_s.is_none() {
                    tr.first_alert_s = Some(t);
                    tr.p_at_first = a.p_damaging;
                    tr.alert = Some(a);
                }
                tr.max_level = tr.max_level.max(a.level as u8);
                let thr = if a.level as u8 >= 2 {
                    cfg.cost_loss
                } else {
                    cfg.cost_loss_freeze
                };
                tr.rational &= a.p_damaging >= thr - 1.0e-6;
                tr.injected_or_single_feed |= a.n_independent < 2;
            }
        }
    }
    tr.normal_at_end = fu.state() == SeismicState::Normal;
    tr
}

fn xs08(c: &mut Ctx, class: ExoClass) {
    let mag = c.get("magnitude");
    let r = c.get("r_epi_km");
    let t_cancel = c.get("t_cancel_s");
    let fa_m = c.get("fa_magnitude");
    let fa_r = c.get("fa_r_epi_km");
    let depth = c.get("depth_km");
    c.model("EEW fusion (max intensity, S-ETA, capped log-odds, cost–loss) at 100 Hz with synthetic network feeds; no local pick or shaking at the site; alert gate per class", true);
    let cfg = FusionCfg::default();
    let feed = |id: u32, m: f64, r: f64, cancel: bool| Feed {
        kind: AlertFeedKind::Network,
        id,
        received: Tick::ZERO,
        origin_offset_s: -5.0,
        magnitude: m as f32,
        r_epi_km: r as f32,
        depth_km: depth as f32,
        mmi: 0.0,
        cancel,
        injected: false,
    };
    // A: a network alert cancelled by the provider.
    let a = run_fusion(
        &[
            (2.0, feed(1, mag, r, false)),
            (2.0 + t_cancel, feed(1, mag, r, true)),
        ],
        60.0,
    );
    // B: a concurrent-quake false alarm: the network reports a nearby event the site never
    // feels (no local pick, no shaking) → withdrawn as a no-show after S-ETA + no_show_s.
    let b = run_fusion(&[(2.0, feed(2, fa_m, fa_r, false))], 90.0);
    let preset = presets::preset(class);
    let action = match preset.safe_state {
        SafeStatePolicy::RideThrough => AlertAction::SeismicPassive,
        SafeStatePolicy::LandAndLock => AlertAction::Lock,
        SafeStatePolicy::CompleteClose | SafeStatePolicy::CloseAfterShaking => AlertAction::Move {
            from: SafetyRank::FrozenOpen,
            to: SafetyRank::ClosedLevitated,
            closing: true,
        },
        _ => AlertAction::Land,
    };
    let conf = b
        .alert
        .map(|al| Confirmations::from_alert(&al, true, false, false))
        .unwrap_or(Confirmations {
            local_p: false,
            n_independent_feeds: 0,
            zone_clear_verified: true,
            obstruction_armed: false,
            within_low_energy_caps: false,
        });
    let verdict = gate(action, &conf, GateClass::Sovereign(class));
    let moved = matches!(action, AlertAction::Move { .. }) && verdict.is_allowed();
    let cost_ok = |tr: &FusionTrace| tr.rational;
    for (tag, tr) in [("cancel", &a), ("false_alarm", &b)] {
        c.kpi(
            &format!("{tag}_first_alert_s"),
            tr.first_alert_s.unwrap_or(f64::NAN),
        );
        c.kpi(
            &format!("{tag}_withdrawn_s"),
            tr.cancel_s.unwrap_or(f64::NAN),
        );
        c.kpi(&format!("{tag}_max_level"), f64::from(tr.max_level));
        c.kpi(&format!("{tag}_p_damaging"), f64::from(tr.p_at_first));
    }
    c.kpi("cost_loss_protect", f64::from(cfg.cost_loss));
    c.kpi("cost_loss_freeze", f64::from(cfg.cost_loss_freeze));
    c.label("class_action", &format!("{action:?}"));
    c.label("gate_verdict", &format!("{verdict:?}"));
    c.note("Every alert is synthetic: fabrail is not an EEW provider. A single network feed never confirms motion; the class keeps its land-first action.");
    c.check(
        "cancel_returns_normal",
        a.normal_at_end && a.cancel_s.is_some(),
        a.cancel_s.unwrap_or(f64::NAN),
        f64::NAN,
        "provider cancel withdraws the event (time of the withdrawal, s); fusion NORMAL at the end"
            .into(),
    );
    c.check(
        "false_alarm_returns_normal",
        b.normal_at_end,
        b.cancel_s.unwrap_or(f64::NAN),
        f64::NAN,
        "no-show withdrawal of the concurrent-quake false alarm (s); fusion NORMAL at the end"
            .into(),
    );
    c.check(
        "cost_within_cost_loss_bound",
        cost_ok(&a) && cost_ok(&b) && !moved,
        f64::from(a.p_at_first.min(b.p_at_first)),
        f64::from(cfg.cost_loss_freeze),
        "each action taken only with P(I ≥ I_crit) above its C/L threshold; no alert-triggered motion without confirmation".into(),
    );
}

// ---- XS09 ---------------------------------------------------------------------------------

fn xs09(c: &mut Ctx, class: ExoClass) {
    let case = OffshoreCase {
        magnitude: c.get("magnitude"),
        epi_offshore_km: c.get("epi_offshore_km"),
        depth_km: c.get("depth_km"),
        city_inland_km: c.get("city_inland_km"),
        v_p_kmps: eew_timing::V_P_KMPS,
        v_s_kmps: eew_timing::V_S_KMPS,
        latency: eew_timing::EewLatency::scaled_to(c.get("latency_s")),
        t_act_s: c.get("t_act_s"),
        k_sensor: 4,
        land_km: c.get("land_km"),
        snet_km: c.get("snet_km"),
        das_offset_km: c.get("das_offset_km"),
    };
    c.fixed("v_p_kmps", case.v_p_kmps);
    c.fixed("v_s_kmps", case.v_s_kmps);
    c.fixed("k_sensor", case.k_sensor as f64);
    c.model("EEW timing: P wavefield sampled at 100 Hz, STA/LTA picks, packetisation, telemetry, k-th pick, processing and dissemination (EXOTIC §X10; timing model simplified)", !matches!(class, ExoClass::Tidegate));
    c.kpi("t_s_city_s", case.t_s_city_s());
    c.kpi(
        "gain_p_s_per_km",
        eew_timing::lead_time_gain_s_per_km(case.v_p_kmps),
    );
    c.kpi(
        "gain_plum_s_per_km",
        eew_timing::lead_time_gain_s_per_km(case.v_s_kmps),
    );
    for s in Sensing::ALL {
        let sim = eew_timing::simulate_offshore(&case, s, c.seed, &SimSettings::default());
        let a = sim.analytic;
        let tag = s.name();
        c.kpi(&format!("{tag}_t_alert_analytic_s"), a.t_alert_s);
        c.kpi(&format!("{tag}_warning_analytic_s"), a.warning_s);
        c.kpi(&format!("{tag}_t_alert_sim_s"), sim.t_alert_s);
        c.kpi(&format!("{tag}_warning_sim_s"), sim.warning_s);
        c.kpi(&format!("{tag}_blind_zone_km"), a.blind_zone_km);
        c.kpi(&format!("{tag}_picks"), sim.n_picks as f64);
        let dw = sim.warning_s - a.warning_s;
        c.check(
            &format!("{tag}_warning_within_1s"),
            fmath::abs(dw) <= 1.0,
            dw,
            1.0,
            format!("simulated minus analytic warning ({tag}), s"),
        );
    }
    c.note("fabrail is not an EEW provider: every feed is synthetic and every output prints its inputs.");
}

// ---- XS10 ---------------------------------------------------------------------------------

fn xs10(c: &mut Ctx, class: ExoClass) {
    let p_s = c.get("p_s_kpa") * 1.0e3;
    let t_d = c.get("t_d_ms") * 1.0e-3;
    let b = c.get("b");
    let t_a = c.get("t_a_s");
    let t_det = c.get("t_detect_s");
    let t_seal = c.get("t_seal_s");
    let margin = c.get("margin_s");
    let range = c.get("range_m");
    let p = presets::preset(class);
    let kin = p.kinematics;
    let d = f64::from(kin.travel_mm) / 1000.0;
    let v = f64::from(kin.v_max_mmps) / 1000.0;
    let a = f64::from(kin.a_max_mmps2) / 1000.0;
    let j = f64::from(kin.j_max_mmps3) / 1000.0;
    c.fixed("d_m", d);
    c.fixed("v_max_mps", v);
    c.fixed("a_mps2", a);
    c.fixed("j_mps3", j);
    c.model("Closure race: zone machine (ClearVerified), jerk-aware planner, S-curve close sampled at 1 kHz, seal (EXOTIC §X11; no scaled-distance law)", matches!(class, ExoClass::Tidegate));
    // Generic blast closed forms (inputs only; no yield or range law).
    let impulse = p_s * t_d * (1.0 / b - (1.0 - fmath::exp(-b)) / (b * b));
    let p0 = 101_325.0;
    let pr_ps = 2.0 * (7.0 * p0 + 4.0 * p_s) / (7.0 * p0 + p_s);
    c.kpi("friedlander_impulse_pa_s", impulse);
    c.kpi("reflection_factor", pr_ps);
    if range > 0.0 {
        c.kpi(
            "arrival_bound_s",
            f64::from(closure::arrival_bound_s(range as f32)),
        );
    }
    // Zone: arm and verify.
    let zc = match p.regime {
        SafetyRegime::GuardedZone(z) => z,
        _ => fabrail_exo_safety::ZoneCfg::default(),
    };
    let mut zm = ZoneMachine::new(zc);
    let clear = ZoneInputs {
        presence_a_clear: true,
        presence_b_clear: true,
        interlocks_locked: true,
        ..ZoneInputs::default()
    };
    zm.step(Tick(0), &clear);
    zm.step(
        Tick(20),
        &ZoneInputs {
            reset_outside: true,
            ..clear
        },
    );
    let zone_ok = zm.state() == ZoneState::ClearVerified;
    let inp = ClosureInputs {
        d_m: d as f32,
        v0_mps: 0.0,
        a_mps2: a as f32,
        j_mps3: j as f32,
        v_max_mps: v as f32,
        t_seal_s: t_seal as f32,
        margin_s: margin as f32,
        t_a_s: t_a as f32,
        t_detect_s: t_det as f32,
        zone_clear: zone_ok,
        two_person_override: false,
    };
    let t_min = f64::from(closure::t_min_s(
        d as f32, 0.0, a as f32, v as f32, j as f32,
    ));
    c.kpi("t_min_s", t_min);
    c.kpi("t_needed_s", t_det + t_min + t_seal + margin);
    c.kpi(
        "min_standoff_m",
        f64::from(closure::min_standoff_m(
            (t_det + t_min + t_seal + margin) as f32,
        )),
    );
    let mut rec = c.recorder(0.001, 50.0, &[("x_m", "m"), ("v_mps", "m/s")]);
    match closure::plan(&inp) {
        ClosureDecision::Close { t_close_s } => {
            let sc = closure::s_curve(d as f32, 0.0, a as f32, v as f32, j as f32);
            let dt = 0.001;
            let n = fmath::ceil(f64::from(t_close_s) / dt) as usize + 1;
            let mut t_closed = f64::INFINITY;
            let mut x_last = 0.0;
            for i in 0..=n {
                let tau = i as f64 * dt;
                let (x, vv) = sc.map_or((0.0, 0.0), |s| {
                    let (x, v, _) = s.sample(tau as f32);
                    (f64::from(x), f64::from(v))
                });
                x_last = x;
                if t_closed.is_infinite() && x >= d - 1.0e-4 && vv <= 1.0e-3 {
                    t_closed = tau;
                }
                rec.push(i, t_det + tau, &[x, vv]);
            }
            let t_sealed = t_det + t_closed + t_seal;
            c.label("decision", "close");
            c.kpi("t_closed_and_sealed_s", t_sealed);
            c.kpi("final_position_m", x_last);
            c.check(
                "closed_before_t_a",
                t_sealed + margin <= t_a + 1e-3,
                t_sealed,
                t_a - margin,
                "detection + simulated close + seal vs t_a − margin, s".into(),
            );
        }
        ClosureDecision::Fallback(reason) => {
            c.label("decision", &format!("fallback: {reason:?} (land in place)"));
            let infeasible = t_det + t_min + t_seal + margin > t_a || !zone_ok;
            c.check(
                "marked_infeasible",
                infeasible,
                t_det + t_min + t_seal + margin,
                t_a,
                "needed time vs t_a: the planner did not start a race it cannot win, s".into(),
            );
        }
    }
    c.note("t_a is an input; fabrail publishes no yield-to-distance table or scaled-distance law (FAB-EXO-808).");
    c.finish(rec);
}

// ---- XS11 ---------------------------------------------------------------------------------

fn xs11(c: &mut Ctx) {
    let sub = c.get("subsidence_m");
    let diff = c.get("differential_frac");
    let tilt = c.get("tilt_mrad") * 1.0e-3;
    let t_shake = c.get("t_shaking_s");
    let t_lat = c.get("t_latency_s");
    let command = c.get("command") >= 0.5;
    let scale = c.get("path_scale");
    c.model("Tsunami timing ∫dx/√(g·h) over the reference path, rail distortion injection, gap check, maglev or wheel close (timing only; no inundation)", true);
    let prof: Vec<(f64, f64)> = tsunami::reference_profile()
        .into_iter()
        .map(|(x, h)| (x * scale, h))
        .collect();
    let t_arr = tsunami::arrival_time_s(&tsunami::refine_profile(&prof, 100.0));
    let mut inp = TidegateInputs::tidegate(t_arr, t_shake, t_lat);
    inp.distortion = RailDistortion {
        subsidence_m: sub,
        differential_frac: diff,
        step_frac: 0.5,
        tilt_rad: tilt,
    };
    if !command {
        inp.t_command_s = None;
    }
    let plan = tsunami::plan_tidegate_close(&inp);
    c.kpi("t_arrival_s", t_arr);
    c.kpi(
        "greens_law_4000_to_10",
        tsunami::greens_law_factor(4000.0, 10.0),
    );
    c.kpi("budget_s", plan.budget_s);
    c.kpi("gap_error_mm", plan.gap.max_gap_error_m * 1e3);
    c.kpi("gap_tolerance_mm", plan.gap.tolerance_m * 1e3);
    c.kpi("t_start_s", plan.t_start_s);
    c.kpi("t_move_s", plan.t_move_s);
    c.kpi("t_closed_s", plan.t_closed_s);
    c.label("close_mode", plan.mode.name());
    c.note("TIDEGATE closes only after the shaking and only on the external decision-system command (SIL 3 target; SIL 4 unclaimable). Debris and salt in an attractive-EMS gap are unsolved; buoyant or hydraulic gates beat maglev physically.");
    let only_after = plan.mode == CloseMode::NoCommand || (command && plan.t_start_s >= t_shake);
    c.check(
        "closes_only_after_shaking_on_command",
        only_after,
        plan.t_start_s,
        t_shake,
        "close start vs end of shaking, s (no command: no close)".into(),
    );
    c.check(
        "mode_follows_gap_check",
        plan.mode == CloseMode::NoCommand || (plan.mode == CloseMode::Maglev) == plan.gap.pass,
        plan.gap.max_gap_error_m * 1e3,
        plan.gap.tolerance_m * 1e3,
        "maglev only when the gap check passes, else wheels (gap error, mm)".into(),
    );
    c.check(
        "closes_within_budget_or_wheels",
        plan.mode == CloseMode::NoCommand || plan.within_budget,
        plan.t_closed_s + inp.margin_s,
        t_arr,
        "closed and sealed plus margin vs the arrival, s".into(),
    );
    // Gate position channel.
    let mut rec = c.recorder(0.1, 1.0, &[("gate_x_m", "m")]);
    if plan.t_start_s.is_finite() {
        let v = if plan.mode == CloseMode::Wheels {
            inp.v_mps * inp.wheel_speed_frac
        } else {
            inp.v_mps
        };
        let sc = closure::s_curve(
            inp.d_m as f32,
            0.0,
            inp.a_mps2 as f32,
            v as f32,
            inp.j_mps3 as f32,
        );
        let t_sw = if plan.mode == CloseMode::Wheels {
            inp.t_wheel_switch_s
        } else {
            0.0
        };
        let n = fmath::ceil((plan.t_closed_s + 10.0) / 0.1) as usize;
        for i in 0..n {
            let t = i as f64 * 0.1;
            let tau = t - plan.t_start_s - t_sw;
            let x = sc.map_or(0.0, |s| f64::from(s.sample(tau as f32).0));
            rec.push(i, t, &[x]);
        }
    }
    c.finish(rec);
}

// ---- XS12 ---------------------------------------------------------------------------------

fn xs12(c: &mut Ctx) {
    let a_target = c.get("a_target_mg") * 1.0e-3 * G0_MPS2;
    let zeta_s = c.get("zeta_s");
    let u10 = c.get("u10_mps");
    let dur = c.get("duration_s");
    c.model("2-DOF host (first mode, 100 kt modal mass, T 7 s) + 1,000 t HMD (Warburton force tuning, ζ on ω_s), Davenport wind (Shinozuka sum), RK4 at 20 ms; bare / passive / LQR active", false);
    let s = Sentinel::reference(zeta_s, TuningForm::WarburtonForce);
    let dt = SENTINEL_DT;
    let fs = 1.0 / dt;
    let n = fmath::round(dur / dt) as usize;
    let wind = DavenportWind {
        u10_mps: u10,
        kappa: 0.005,
        area_m2: 1.0,
        c_d: 1.3,
        rho_kg_per_m3: 1.225,
    };
    let unit = wind.series(fs, n, c.seed);
    // Bare structure (no damper) at unit area: scale so its peak acceleration = target.
    let p = s.primary();
    let bare = |f: &[f64]| -> (Vec<f64>, Vec<f64>) {
        let mut y = [0.0; 2];
        let mut xs = Vec::with_capacity(n);
        let mut acc = Vec::with_capacity(n);
        for fk in f.iter().take(n) {
            sdof_step(&p, &mut y, 0.0, *fk, dt);
            xs.push(y[0]);
            acc.push(
                -2.0 * p.zeta * p.omega_rad_s * y[1] - p.omega_rad_s * p.omega_rad_s * y[0]
                    + fk / p.m_kg,
            );
        }
        (xs, acc)
    };
    let (_, acc_unit) = bare(&unit);
    let skip = fmath::round(120.0 / dt) as usize;
    let a_unit = peak_abs(&acc_unit[skip.min(n)..]);
    let scale = if a_unit > 0.0 { a_target / a_unit } else { 0.0 };
    let force: Vec<f64> = unit.iter().map(|v| v * scale).collect();
    c.fixed("wind_area_equiv_m2", scale);
    let (xs0, acc0) = bare(&force);
    let k = sentinel_lqr(&s, dt);
    let ff = |i: usize| force[i];
    let no_ag = |_t: f64| 0.0;
    let passive = sentinel_run(&s, true, HmdDrive::Off, &no_ag, &ff, n, dt, [0.0; 4]);
    let active = sentinel_run(&s, true, HmdDrive::ActiveOnly, &no_ag, &ff, n, dt, k);
    let w = skip.min(n);
    let (x0, xp, xa) = (rms(&xs0[w..]), rms(&passive.xs[w..]), rms(&active.xs[w..]));
    let red_p = 1.0 - xp / x0;
    let red_a = 1.0 - xa / x0;
    let h2_p = h2_host_ratio(&s, [0.0; 4]).map_or(f64::NAN, |r| 1.0 - fmath::sqrt(r));
    let h2_a = h2_host_ratio(&s, k).map_or(f64::NAN, |r| 1.0 - fmath::sqrt(r));
    // Activation: hysteresis drag of the rail, plus cogging if the LSM had iron.
    let a_act = host::activation_threshold_mps2(s.rail.mu_eq());
    let a_act_mg = a_act / G0_MPS2 * 1e3;
    let cog_iron_mg = [0.01, 0.03].map(|f| f * s.f_max / s.design().m_d_kg / G0_MPS2 * 1e3);
    let ironless = true;
    let a_act_total_mg = a_act_mg + if ironless { 0.0 } else { cog_iron_mg[0] };
    // Coulomb stick/slip check at the activation threshold (X4 CoulombSdof).
    let w_d = s.design().omega_d_rad_s;
    let slip_at = |factor: f64| -> f64 {
        let md = s.design().m_d_kg;
        let mut sd = host::CoulombSdof::new(md, md * w_d * w_d, 0.0, s.rail.mu_eq());
        let mut x_max = 0.0_f64;
        for i in 0..20_000_u32 {
            let t = f64::from(i) * 5.0e-3;
            sd.step(factor * a_act * fmath::sin(s.w_s * t), 5.0e-3);
            x_max = x_max.max(fmath::abs(sd.x_m));
        }
        x_max
    };
    let (stick, slip) = (slip_at(0.9), slip_at(1.1));
    c.kpi(
        "host_peak_accel_uncontrolled_mg",
        peak_abs(&acc0[w..]) / G0_MPS2 * 1e3,
    );
    c.kpi(
        "host_rms_accel_uncontrolled_mg",
        rms(&acc0[w..]) / G0_MPS2 * 1e3,
    );
    c.kpi(
        "host_peak_accel_passive_mg",
        peak_abs(&passive.acc[w..]) / G0_MPS2 * 1e3,
    );
    c.kpi(
        "host_peak_accel_active_mg",
        peak_abs(&active.acc[w..]) / G0_MPS2 * 1e3,
    );
    c.kpi("rms_reduction_passive_pct", red_p * 100.0);
    c.kpi("rms_reduction_active_pct", red_a * 100.0);
    c.kpi("h2_reduction_passive_pct", h2_p * 100.0);
    c.kpi("h2_reduction_active_pct", h2_a * 100.0);
    c.kpi("zeta_s", zeta_s);
    c.kpi("stroke_peak_active_m", peak_abs(&active.xr));
    c.kpi("lsm_force_peak_n", peak_abs(&active.force));
    c.kpi("lsm_power_peak_w", peak_abs(&active.power));
    c.kpi("activation_mg", a_act_mg);
    c.kpi("iron_core_cogging_mg_lo", cog_iron_mg[0]);
    c.kpi("iron_core_cogging_mg_hi", cog_iron_mg[1]);
    c.kpi("coulomb_stick_0p9_m", stick);
    c.kpi("coulomb_slip_1p1_m", slip);
    c.label("rail_condition", s.rail.name());
    c.label("lsm", "ironless (no cogging, no normal force)");
    c.label("lqr_gains", &format!("{k:?}"));
    c.note("The RMS reduction is always printed with ζ_s: the passive optimum gives 44.7 % at μ 1 %, ζ_s 1 % but 30.0 % at ζ_s 2 % (H2, white-noise force). The ≥ 40 % target applies to the active mode only. Mass dampers are mainly wind devices.");
    c.check(
        "activation_le_0p3_mg",
        a_act_total_mg <= 0.3 && stick == 0.0 && slip > 0.0,
        a_act_total_mg,
        0.3,
        format!("bearing activation μ_eq·g ({}) with an ironless LSM, mg (stuck at 0.9·μg, slips at 1.1·μg)", s.rail.name()),
    );
    c.check(
        "active_rms_reduction_ge_40pct",
        red_a >= 0.40 && active.finite,
        red_a * 100.0,
        40.0,
        format!("host displacement RMS reduction in HMD_ACTIVE vs the bare structure at ζ_s = {zeta_s}, %"),
    );
    c.check(
        "stroke_and_force_within_limits",
        peak_abs(&active.xr) <= s.s_max && peak_abs(&active.force) <= s.f_max * (1.0 + 1e-6),
        peak_abs(&active.xr),
        s.s_max,
        "active peak stroke vs s_max (m); LSM force ≤ F_max".into(),
    );
    let mut rec = c.recorder(
        dt,
        2.0,
        &[
            ("x_host_bare_m", "m"),
            ("x_host_passive_m", "m"),
            ("x_host_active_m", "m"),
            ("wind_force_n", "N"),
        ],
    );
    for i in 0..n {
        rec.push(
            i,
            i as f64 * dt,
            &[xs0[i], passive.xs[i], active.xs[i], force[i]],
        );
    }
    c.finish(rec);
}

// ---- XC-RED-1 / XC-RED-2: one hybrid-EMS pod behind the 2oo3 voter ------------------------

/// Sensor fault kinds of XC-RED-1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SensorFault {
    Stuck,
    Offset,
    Drift,
    Dropout,
    Saturate,
    NoiseBurst,
    Inverted,
}

impl SensorFault {
    const ALL: [SensorFault; 7] = [
        SensorFault::Stuck,
        SensorFault::Offset,
        SensorFault::Drift,
        SensorFault::Dropout,
        SensorFault::Saturate,
        SensorFault::NoiseBurst,
        SensorFault::Inverted,
    ];

    const fn name(self) -> &'static str {
        match self {
            SensorFault::Stuck => "stuck",
            SensorFault::Offset => "offset",
            SensorFault::Drift => "drift",
            SensorFault::Dropout => "dropout",
            SensorFault::Saturate => "saturate",
            SensorFault::NoiseBurst => "noise_burst",
            SensorFault::Inverted => "inverted",
        }
    }
}

/// Pod loop settings.
#[derive(Clone, Copy, Debug)]
struct PodCfg {
    g0_um: f64,
    pole: f64,
    noise_um: f64,
    e_th_um: i32,
    persist_ticks: u32,
}

/// Outcome of one pod run.
#[derive(Clone, Copy, Debug)]
struct PodRun {
    max_err_um: f64,
    min_gap_um: f64,
    rung: Redundancy,
    t_rung_s: Option<f64>,
    t_safe_s: Option<f64>,
    touchdown_mps: Option<f64>,
    stop_contact: bool,
}

/// Upper sliding stop of the pod as a fraction of the nominal gap.
const POD_UPPER_STOP_FRAC: f64 = 0.25;
/// Landing skids: the gap opens by this multiple of the nominal gap.
const POD_SKID_GAP_FRAC: f64 = 1.0;

/// Sensor range of the gap probes, µm (0 … 2.5·g0; outside fails the self-test).
fn probe_range_um(g0_um: f64) -> (i32, i32) {
    (0, fmath::round(2.5 * g0_um) as i32)
}

/// One pod run at 5 kHz for 2 s: the gap loop regulates the voted gap; faults start at
/// 0.5 s on `faulty` channels; a 0.05 g load step at 1.0 s; after a safe-state demand the
/// pod lands (PM-bias ramp to the skids at 10 mm/s on the surviving channel).
fn pod_run(p: &PodCfg, fault: SensorFault, faulty: [bool; 3], seed: u64) -> PodRun {
    let dt = 2.0e-4;
    let ticks_per = 4_u64; // 20 kHz base ticks per 5 kHz step
    let n = 10_000;
    let wc = 2.5 * p.pole;
    let kp = wc * wc + p.pole * p.pole;
    let kd = 2.0 * 0.7 * wc;
    let mut rng = NormalStream::new(Xoshiro256pp::from_seed(seed));
    // The diverse L(g) channel is 5× noisier than the probes; its tolerance is 4·e_th
    // (EXOTIC §X13: a tie-breaker and common-cause veto only).
    let e_div = 4 * p.e_th_um;
    let div_noise = 5.0 * p.noise_um.max(5.0);
    let mut voter = Voter2oo3::with_diverse_tol(
        VoterCfg {
            e_th: p.e_th_um,
            t_persist_ticks: p.persist_ticks,
        },
        e_div,
    );
    let mut ladder = Ladder::new(p.persist_ticks);
    let (lo, hi) = probe_range_um(p.g0_um);
    let (mut z, mut v) = (0.0_f64, 0.0_f64); // gap deviation, µm and µm/s (+ = larger gap)
    let mut stuck = [0_i32; 3];
    let mut prev_e = 0.0_f64;
    let mut out = PodRun {
        max_err_um: 0.0,
        min_gap_um: p.g0_um,
        rung: Redundancy::TwoOoThree,
        t_rung_s: None,
        t_safe_s: None,
        touchdown_mps: None,
        stop_contact: false,
    };
    let mut landing_ref = 0.0_f64;
    let mut div_lp = p.g0_um;
    for k in 0..n {
        let t = k as f64 * dt;
        let tick = Tick(k as u64 * ticks_per);
        let gap = p.g0_um + z;
        let mut x = [0_i32; 3];
        let mut self_ok = [true; 3];
        for ch in 0..3 {
            let clean = gap + rng.next_scaled(p.noise_um);
            let active = faulty[ch] && t >= 0.5;
            let val = if !active {
                stuck[ch] = fmath::round(clean) as i32;
                clean
            } else {
                match fault {
                    SensorFault::Stuck => f64::from(stuck[ch]),
                    SensorFault::Offset => clean + 3000.0,
                    SensorFault::Drift => clean + 20_000.0 * (t - 0.5),
                    SensorFault::Dropout => f64::from(lo) - 1.0,
                    SensorFault::Saturate => f64::from(hi) + 1.0,
                    SensorFault::NoiseBurst => clean + rng.next_scaled(2000.0),
                    SensorFault::Inverted => 2.0 * p.g0_um - clean,
                }
            };
            let r = fmath::round(val) as i32;
            // Self-test: a reading outside the probe range fails at once.
            self_ok[ch] = r >= lo && r <= hi;
            x[ch] = r.clamp(lo, hi);
        }
        let diverse = fmath::round(gap + rng.next_scaled(div_noise)) as i32;
        let vo = voter.step_diverse(tick, x, Some(diverse));
        let healthy = [
            self_ok[0] && !vo.excluded[0],
            self_ok[1] && !vo.excluded[1],
            self_ok[2] && !vo.excluded[2],
        ];
        let rung = ladder.step(
            tick,
            &LadderInputs {
                healthy,
                diverse_ok: true,
            },
        );
        if rung != out.rung {
            out.rung = rung;
            if out.t_rung_s.is_none() {
                out.t_rung_s = Some(t);
            }
            if rung == Redundancy::SafeState && out.t_safe_s.is_none() {
                out.t_safe_s = Some(t);
            }
        }
        // Measurement: the voted value; in the safe state, the surviving healthy channel
        // (or the voted value) tracking the landing reference.
        // In the safe state the energised landing runs on the diverse L(g) estimate
        // (low-passed at 1 ms): every primary may be excluded by then.
        div_lp += (f64::from(diverse) - div_lp) * (dt / (dt + 1.0e-3));
        let meas = if rung == Redundancy::SafeState {
            div_lp
        } else {
            f64::from(vo.value)
        };
        let target = if rung == Redundancy::SafeState {
            // Landing: an attractive pod under its rail lands by opening the gap at
            // 10 mm/s onto the skids at 2·g0.
            // (The reference ends 5 % beyond the skids so the pod settles onto them.)
            landing_ref = (landing_ref + 10_000.0 * dt).min((POD_SKID_GAP_FRAC + 0.05) * p.g0_um);
            p.g0_um + landing_ref
        } else {
            p.g0_um
        };
        let e = meas - target;
        let de = (e - prev_e) / dt;
        prev_e = e;
        // Linearised heave: z̈ = p²·z − u + disturbance (µm/s²); the load step pulls the
        // pod toward a larger gap (heavier load, 0.05 g).
        let dist = if t >= 1.0 {
            0.05 * G0_MPS2 * 1.0e6
        } else {
            0.0
        };
        let u = kp * e + kd * de;
        let acc = p.pole * p.pole * z - u + dist;
        v += acc * dt;
        z += v * dt;
        // Designed stops: the upper sliding stop at 0.25·g0, the skids at 2·g0.
        let (z_up, z_down) = (
            (POD_UPPER_STOP_FRAC - 1.0) * p.g0_um,
            POD_SKID_GAP_FRAC * p.g0_um,
        );
        if z <= z_up || z >= z_down {
            if rung == Redundancy::SafeState && z >= z_down {
                if out.touchdown_mps.is_none() {
                    out.touchdown_mps = Some(fmath::abs(v) * 1.0e-6);
                }
            } else {
                out.stop_contact = true;
            }
            z = z.clamp(z_up, z_down);
            v = 0.0;
        }
        let gap_now = p.g0_um + z;
        if rung != Redundancy::SafeState {
            out.max_err_um = out.max_err_um.max(fmath::abs(z));
        }
        out.min_gap_um = out.min_gap_um.min(gap_now);
    }
    out
}

fn pod_cfg(c: &mut Ctx, class: ExoClass) -> PodCfg {
    let gap_mm = c.get("gap_mm");
    let pole = c.get("pole_rad_s");
    let noise = c.get("noise_um");
    let e_th = c.get("e_th_um");
    let persist_ms = c.get("t_persist_ms");
    c.label(
        "class_mass",
        &format!("{} kg", presets::preset(class).design_mass.0),
    );
    PodCfg {
        g0_um: gap_mm * 1000.0,
        pole,
        noise_um: noise,
        e_th_um: fmath::round(e_th) as i32,
        persist_ticks: fmath::round(persist_ms * 20.0) as u32,
    }
}

fn xc_red1(c: &mut Ctx, class: ExoClass) {
    let p = pod_cfg(c, class);
    c.model("One hybrid-EMS pod heave loop (linearised, 5 kHz, PD at 2.5× the open-loop pole) behind the 2oo3 mid-value voter and the degradation ladder; every single fault on every channel", true);
    let mut worst = 0.0_f64;
    let mut all_ok = true;
    let mut n_cases = 0;
    for f in SensorFault::ALL {
        for ch in 0..3 {
            let mut faulty = [false; 3];
            faulty[ch] = true;
            let r = pod_run(
                &p,
                f,
                faulty,
                c.seed.wrapping_add(ch as u64 * 31 + f as u64),
            );
            n_cases += 1;
            worst = worst.max(r.max_err_um);
            let ok = r.max_err_um < 0.5 * p.g0_um && r.t_safe_s.is_none() && !r.stop_contact;
            all_ok &= ok;
            c.kpi(
                &format!("{}_ch{ch}_max_gap_error_um", f.name()),
                r.max_err_um,
            );
            if ch == 0 {
                c.label(&format!("{}_rung", f.name()), &format!("{:?}", r.rung));
            }
        }
    }
    // Reference: no fault.
    let r0 = pod_run(&p, SensorFault::Stuck, [false; 3], c.seed);
    c.kpi("nominal_max_gap_error_um", r0.max_err_um);
    c.kpi("worst_max_gap_error_um", worst);
    c.kpi("cases", f64::from(n_cases));
    c.check(
        "no_loss_of_levitation",
        all_ok,
        worst,
        0.5 * p.g0_um,
        "worst pod gap error over every single fault on every channel (no stop contact, no safe-state demand), µm".into(),
    );
}

fn xc_red2(c: &mut Ctx, class: ExoClass) {
    let p = pod_cfg(c, class);
    c.model(
        "One hybrid-EMS pod heave loop behind the 2oo3 voter (with the diverse L(g) veto) and the ladder; β common-cause double fault (two channels on one cycle); energised landing on L(g)",
        true,
    );
    let bound_s = f64::from(p.persist_ticks) / 20_000.0 + 2.0e-4;
    let mut worst = 0.0_f64;
    let mut all_ok = true;
    // Detectable common-cause faults: out of range (self-test) or agreeing on a wrong
    // value that the diverse channel contradicts (veto).
    for f in [
        SensorFault::Dropout,
        SensorFault::Saturate,
        SensorFault::Offset,
    ] {
        let r = pod_run(&p, f, [true, true, false], c.seed.wrapping_add(f as u64));
        let lat = r.t_safe_s.map_or(f64::INFINITY, |t| t - 0.5);
        worst = worst.max(lat);
        let landed = r.touchdown_mps.is_some_and(|v| v <= 0.1);
        c.kpi(&format!("{}_safe_state_latency_s", f.name()), lat);
        c.kpi(
            &format!("{}_touchdown_mps", f.name()),
            r.touchdown_mps.unwrap_or(f64::NAN),
        );
        c.kpi(&format!("{}_min_gap_um", f.name()), r.min_gap_um);
        c.label(&format!("{}_rung", f.name()), &format!("{:?}", r.rung));
        all_ok &= lat <= bound_s && landed && !r.stop_contact;
    }
    // Two channels stuck together: invisible until the plant moves away from the frozen
    // reading; reported, not bounded by t_persist.
    let st = pod_run(
        &p,
        SensorFault::Stuck,
        [true, true, false],
        c.seed.wrapping_add(99),
    );
    c.kpi(
        "stuck_pair_safe_state_latency_s",
        st.t_safe_s.map_or(f64::INFINITY, |t| t - 0.5),
    );
    c.kpi("stuck_pair_max_gap_error_um", st.max_err_um);
    c.kpi("stuck_pair_min_gap_um", st.min_gap_um);
    c.kpi("ladder_bound_s", bound_s);
    c.kpi("worst_latency_s", worst);
    c.note("Two channels failing the same way on one cycle agree with each other; the ladder treats any double loss before the degraded rung is established as common cause and demands the safe state (STO on the LSM, brake, energised landing). An agreeing wrong pair is caught only by the diverse L(g) veto; a stuck pair only once the gap moves (its latency is reported, not bounded). This is the β share the PFH carries.");
    c.check(
        "safe_state_within_ladder_bound",
        all_ok,
        worst,
        bound_s,
        "safe-state demand after a detectable double fault (t_persist + one sample), s; landing touchdown ≤ 0.1 m/s".into(),
    );
    c.kpi(
        "stuck_pair_stop_contact",
        f64::from(u8::from(st.stop_contact)),
    );
    c.label(
        "stuck_pair",
        if st.stop_contact {
            "undetected: the agreeing stuck pair outvotes the healthy channel and L(g) is never consulted in 1oo2D; the pod reaches a designed stop (β common cause)"
        } else {
            "detected before a stop"
        },
    );
}

// ---- XC-SAF-1 -----------------------------------------------------------------------------

/// Jerk-limited stop distance from `v` with deceleration `a` and jerk `j` (ramp in and
/// out): v²/(2a) + v·a/(2j) when v ≥ a²/j, else v·√(v/j); and its duration.
fn ss1_stop(v: f64, a: f64, j: f64) -> (f64, f64) {
    if v <= 0.0 {
        return (0.0, 0.0);
    }
    if v >= a * a / j {
        (v * v / (2.0 * a) + v * a / (2.0 * j), v / a + a / j)
    } else {
        let tj = fmath::sqrt(v / j);
        (v * tj, 2.0 * tj)
    }
}

fn xc_saf1(c: &mut Ctx, class: ExoClass) {
    let t_react = c.get("t_react_s");
    let frac = c.get("intrusion_frac");
    let p = presets::preset(class);
    let k = p.kinematics;
    let d = f64::from(k.travel_mm) / 1000.0;
    let v_max = f64::from(k.v_max_mmps) / 1000.0;
    let a = f64::from(k.a_max_mmps2) / 1000.0;
    let j = f64::from(k.j_max_mmps3) / 1000.0;
    let zc = match p.regime {
        SafetyRegime::GuardedZone(z) => z,
        _ => fabrail_exo_safety::ZoneCfg::default(),
    };
    let a_brake = f64::from(zc.a_brake_mmps2) / 1000.0;
    c.fixed("travel_m", d);
    c.fixed("v_max_mps", v_max);
    c.fixed("a_brake_mps2", a_brake);
    c.fixed("j_mps3", j);
    c.model("Guarded zone machine at 1 kHz + class S-curve move + jerk-limited SS1 stop (kinematic reduced model)", true);
    let sc = closure::s_curve(d as f32, 0.0, a as f32, v_max as f32, j as f32);
    let Some(sc) = sc else {
        c.check(
            "move_planned",
            false,
            f64::NAN,
            f64::NAN,
            "the class move could not be planned".into(),
        );
        return;
    };
    let t_move = f64::from(sc.total_s());
    let t_i = frac * t_move;
    let mut zm = ZoneMachine::new(zc);
    let clear = ZoneInputs {
        presence_a_clear: true,
        presence_b_clear: true,
        interlocks_locked: true,
        ..ZoneInputs::default()
    };
    let dt = 0.001;
    let ticks = 20_u64;
    zm.step(Tick(0), &clear);
    zm.step(
        Tick(ticks),
        &ZoneInputs {
            reset_outside: true,
            ..clear
        },
    );
    let verified = zm.state() == ZoneState::ClearVerified;
    let (x_i, v_i, _) = sc.sample(t_i as f32);
    let (x_i, v_i) = (f64::from(x_i), f64::from(v_i));
    let (mut t_ss1, mut t_stop, mut x_stop) = (None::<f64>, None::<f64>, f64::NAN);
    let mut v_brake = 0.0;
    let mut x_brake = 0.0;
    let n = fmath::ceil((t_i + t_react + 60.0) / dt) as usize;
    let mut rec = c.recorder(
        dt,
        10.0,
        &[("x_m", "m"), ("v_mps", "m/s"), ("zone_state", "")],
    );
    for i in 2..n {
        let t = i as f64 * dt;
        let intruded = t >= t_i;
        if intruded && t_ss1.is_none() && t >= t_i + t_react {
            // SS1 begins after the reaction time; the drive followed its profile until now.
            t_ss1 = Some(t);
            let (xb, vb, _) = sc.sample((t_i + t_react) as f32);
            x_brake = f64::from(xb);
            v_brake = f64::from(vb);
        }
        let (x, v) = if t_ss1.is_none() {
            let (x, v, _) = sc.sample(t as f32);
            (f64::from(x), f64::from(v))
        } else {
            let tau = t - (t_i + t_react);
            let (d_stop, t_stop_dur) = ss1_stop(v_brake, a_brake, j.max(1e-9));
            if tau >= t_stop_dur {
                if t_stop.is_none() {
                    t_stop = Some(t);
                    x_stop = x_brake + d_stop;
                }
                (x_brake + d_stop, 0.0)
            } else {
                // Distance along the stop profile (sampled): scale by elapsed fraction of a
                // symmetric profile; use the closed-form stop distance at the end.
                let (dd, _) = ss1_stop(v_brake, a_brake, j.max(1e-9));
                let f = tau / t_stop_dur;
                (x_brake + dd * (f * (2.0 - f)).min(1.0), v_brake * (1.0 - f))
            }
        };
        let st = zm.step(
            Tick(i as u64 * ticks),
            &ZoneInputs {
                presence_a_clear: !intruded,
                motion_active: t_stop.is_none(),
                stopped: t_stop.is_some(),
                ..clear
            },
        );
        rec.push(i, t, &[x, v, f64::from(st as u8)]);
        if t_stop.is_some() && t > t_stop.unwrap_or(0.0) + 0.1 {
            break;
        }
    }
    let d_sim = x_stop - x_i;
    // The integer bound takes the speed rounded up to whole mm/s (conservative).
    let d_bound_mm = ss1_stop_distance_mm(
        fmath::ceil(v_i * 1000.0 - 1.0e-6) as u32,
        fmath::round(t_react * 1000.0) as u32,
        zc.a_brake_mmps2,
        zc.a_brake_mmps2,
        k.j_max_mmps3,
    );
    let d_bound = d_bound_mm as f64 / 1000.0;
    c.kpi("t_intrusion_s", t_i);
    c.kpi("v_at_intrusion_mps", v_i);
    c.kpi("d_stop_sim_m", d_sim);
    c.kpi("d_stop_analytic_m", d_bound);
    c.kpi("t_stop_s", t_stop.unwrap_or(f64::NAN));
    c.kpi(
        "iso13855_s_m",
        fabrail_exo_safety::zone::iso13855_separation_mm(&zc, k.v_max_mmps) as f64 / 1000.0,
    );
    c.label("zone_final", &format!("{:?}", zm.state()));
    c.check(
        "stop_within_analytic_d_stop",
        verified && d_sim <= d_bound + 1e-9 && zm.state() == ZoneState::Stopped,
        d_sim,
        d_bound,
        "stop distance from the intrusion vs d_stop = v·t_react + v²/(2a) + v·a/(2j), m".into(),
    );
    c.finish(rec);
}

// ---- XC-ARN-1 -----------------------------------------------------------------------------

fn xc_arn1(c: &mut Ctx) {
    let cap = c.get("capacity_ratio");
    let cx = c.get("cog_x_m");
    let cy = c.get("cog_y_m");
    let lost = c.get("lost_pod");
    c.model("ARENA static WLS allocation over 500 pods (25 × 20 grid, 4.0 × 4.3 m pitch); dynamic gap error is roadmap", true);
    const N: usize = 500;
    let m = f64::from(presets::ARENA.design_mass.0);
    let w = m * G0_MPS2;
    let f_max = cap * w / N as f64;
    let mut pods = [Pod::default(); N];
    for (i, pod) in pods.iter_mut().enumerate() {
        let (ix, iy) = (i % 25, i / 25);
        *pod = Pod {
            x_m: ((ix as f64 - 12.0) * 4.0) as f32,
            y_m: ((iy as f64 - 9.5) * 4.3) as f32,
            f_max_n: f_max as f32,
            weight: 1.0,
            healthy: true,
        };
    }
    let wrench = Wrench {
        fz_n: w as f32,
        mx_nm: (w * cy) as f32,
        my_nm: (-w * cx) as f32,
    };
    let mut out = [0.0_f32; N];
    let s0 = alloc::allocate(&wrench, &pods, &mut out);
    let max0 = out.iter().fold(0.0_f32, |a, f| a.max(*f)) / f_max as f32;
    let idx = if lost >= 0.0 {
        (lost as usize).min(N - 1)
    } else {
        out.iter()
            .enumerate()
            .fold(
                (0, f32::MIN),
                |acc, (i, f)| if *f > acc.1 { (i, *f) } else { acc },
            )
            .0
    };
    pods[idx].healthy = false;
    let s1 = alloc::allocate(&wrench, &pods, &mut out);
    let max1 = out.iter().fold(0.0_f32, |a, f| a.max(*f)) / f_max as f32;
    c.fixed("pod_capacity_n", f_max);
    c.kpi("lost_pod_index", idx as f64);
    c.kpi("nominal_ok", f64::from(u8::from(s0.ok)));
    c.kpi("nominal_max_utilisation", f64::from(max0));
    c.kpi("realloc_ok", f64::from(u8::from(s1.ok)));
    c.kpi("realloc_land", f64::from(u8::from(s1.land)));
    c.kpi("realloc_max_utilisation", f64::from(max1));
    c.kpi("realloc_residual_fz_n", f64::from(s1.residual.fz_n));
    c.kpi("healthy_pods", f64::from(s1.n_healthy));
    c.label(
        "outcome",
        if s1.ok && !s1.land {
            "reallocated"
        } else {
            "LAND"
        },
    );
    c.note("Only static reallocation is built; the dynamic gap error of a 500-pod tray is roadmap (🗺️).");
    c.check(
        "static_reallocation_or_land",
        s1.ok || s1.land,
        f64::from(max1),
        1.0,
        "the reallocated wrench is met within tolerance (max pod utilisation ≤ 1), else LAND is declared".into(),
    );
}

// ---- XC-SHD-1 -----------------------------------------------------------------------------

fn xc_shd1(c: &mut Ctx) {
    let v = c.get("v_mps");
    let t_react = c.get("t_react_s");
    let j = c.get("j_ss1_mps3");
    let t_land = c.get("t_land_s");
    let t_lock = c.get("t_lock_s");
    let p = presets::SHIELD;
    let a_brake = match p.regime {
        SafetyRegime::GuardedZone(z) => f64::from(z.a_brake_mmps2) / 1000.0,
        _ => 0.5,
    };
    c.fixed("a_brake_mps2", a_brake);
    c.model("SHIELD transit timeline: local P pick → beam-permit drop → SS1 stop → PM-bias-ramp landing → lock (reduced kinematic model at 1 kHz)", true);
    let t_p = 2.0;
    let dt = 0.001;
    let (d_stop, t_stop) = ss1_stop(v, a_brake, j);
    let t_brake = t_p + t_react;
    let t_stopped = t_brake + t_stop;
    let ramp = platform::LandingRamp {
        gap_m: 0.012,
        t_ramp_s: t_land,
        ..platform::LandingRamp::default()
    };
    let t_landed = t_stopped + t_land;
    let t_locked = t_landed + t_lock;
    let mut beam_drop = None::<f64>;
    let mut touchdown_v = 0.0_f64;
    let n = fmath::ceil((t_locked + 1.0) / dt) as usize;
    let mut rec = c.recorder(
        dt,
        100.0,
        &[
            ("x_m", "m"),
            ("v_mps", "m/s"),
            ("gap_m", "m"),
            ("beam_permit", ""),
        ],
    );
    let mut x = 0.0;
    for i in 0..n {
        let t = i as f64 * dt;
        let permit = t < t_p;
        if !permit && beam_drop.is_none() {
            beam_drop = Some(t);
        }
        let vel = if t < t_brake {
            v
        } else if t < t_stopped {
            let f = (t - t_brake) / t_stop;
            v * (1.0 - f)
        } else {
            0.0
        };
        x += vel * dt;
        let (gap, vz) = if t < t_stopped {
            (ramp.gap_m, 0.0)
        } else {
            ramp.reference(t - t_stopped)
        };
        if t >= t_stopped && t < t_landed {
            touchdown_v = touchdown_v.max(fmath::abs(vz));
        }
        rec.push(i, t, &[x, vel, gap, f64::from(u8::from(permit))]);
    }
    let total = t_locked - t_p;
    c.kpi("t_land_and_lock_s", total);
    c.kpi("stop_distance_m", d_stop);
    c.kpi("t_stop_s", t_stop);
    c.kpi("landing_peak_speed_mps", touchdown_v);
    c.kpi(
        "beam_permit_drop_latency_s",
        beam_drop.map_or(f64::NAN, |t| t - t_p),
    );
    c.check(
        "land_and_lock_within_2s",
        total <= 2.0,
        total,
        2.0,
        "P pick to landed and locked (reaction + SS1 stop + landing ramp + lock), s".into(),
    );
    c.check(
        "beam_permit_dropped",
        beam_drop.is_some_and(|t| t - t_p <= dt + 1e-12),
        beam_drop.map_or(f64::NAN, |t| t - t_p),
        dt,
        "beam permit withdrawn on the detection cycle, s".into(),
    );
    c.finish(rec);
}

// ---- XC-HTS-1 -----------------------------------------------------------------------------

/// Equilibrium height where the HTS lift equals `w_n` for state `h` (bisection on
/// [z_lo, z_hi]); `None` when even the lowest height cannot carry it.
fn hts_equilibrium(b: &HtsBulk, h: &HtsState, w_n: f64, z_lo: f64, z_hi: f64) -> Option<f64> {
    let f = |z: f64| {
        b.force(
            &GapState {
                z_m: z,
                ..GapState::default()
            },
            h,
        )
        .fz_n
            - w_n
    };
    if f(z_lo) < 0.0 {
        return None;
    }
    if f(z_hi) >= 0.0 {
        return Some(z_hi);
    }
    let (mut lo, mut hi) = (z_lo, z_hi);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if f(mid) >= 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(0.5 * (lo + hi))
}

fn xc_hts1(c: &mut Ctx) {
    let load_frac = c.get("load_frac");
    let v_land = c.get("v_land_mmps") * 1.0e-3;
    let dynamic = c.get("dynamic") >= 0.5;
    c.model(
        "One ATZ-class HTS module (4 cryostats, frozen image + play + creep + LN₂ reservoir) carrying its load share quasi-statically over the LN₂ depletion, landed by a controlled lowering; HAVEN-R HTS option (research tier)",
        true,
    );
    let b = HtsBulk::atz_demonstrator();
    let w = load_frac * b.first_loading_force_n(b.z_work_m);
    c.fixed("module_load_n", w);
    c.fixed("z_fc_mm", b.z_fc_m * 1e3);
    c.fixed("z_land_mm", b.z_land_m * 1e3);
    let fz = |h: &HtsState, z: f64| {
        b.force(
            &GapState {
                z_m: z,
                ..GapState::default()
            },
            h,
        )
        .fz_n
    };
    // Field cool at z_fc, then load: lower in 10 µm steps until the lift carries w.
    let mut h = b.field_cool(0.0);
    let mut z = b.z_fc_m;
    while z > b.z_land_m && fz(&h, z) < w {
        z -= 1.0e-5;
        b.step_hysteresis(
            &GapState {
                z_m: z,
                ..GapState::default()
            },
            &mut h,
            0.0,
        );
    }
    c.kpi("z_levitated_mm", z * 1e3);
    let v_dyn = if dynamic { b.v_dyn_ref_mps } else { 0.0 };
    let dt = 10.0;
    let hold_s = b.ln2_hold_s(if dynamic { 1.0 } else { 0.0 });
    let t_end = 1.2 * hold_s + 4.0 * 3600.0;
    let (mut t, mut t_land, mut ln2_at_land) = (0.0_f64, None::<f64>, f64::NAN);
    let (mut landed, mut lost_lift) = (false, false);
    let (mut min_ratio_path, mut max_hold_down) = (f64::INFINITY, 0.0_f64);
    let mut rec = c.recorder(
        dt,
        0.01,
        &[("z_mm", "mm"), ("ln2_frac", ""), ("lift_over_load", "")],
    );
    let mut k = 0usize;
    while t < t_end {
        b.step_hysteresis(
            &GapState {
                z_m: z,
                zdot_mps: v_dyn,
                ..GapState::default()
            },
            &mut h,
            dt,
        );
        t += dt;
        let status = b.ln2_status(&h);
        if !landed && t_land.is_none() && status != Ln2Status::Ok {
            t_land = Some(t);
            ln2_at_land = h.ln2_frac;
            // Controlled lowering onto the skids: the lift grows as the gap closes, so the
            // actuators only hold the module down; track F/W along the path.
            let dts = 0.1;
            while z > b.z_land_m {
                z = (z - v_land * dts).max(b.z_land_m);
                b.step_hysteresis(
                    &GapState {
                        z_m: z,
                        zdot_mps: -v_land,
                        ..GapState::default()
                    },
                    &mut h,
                    dts,
                );
                t += dts;
                let f = fz(&h, z);
                min_ratio_path = min_ratio_path.min(f / w);
                max_hold_down = max_hold_down.max((f - w) / w);
            }
            landed = true;
        } else if !landed {
            match hts_equilibrium(&b, &h, w, b.z_land_m, b.z_fc_m) {
                Some(ze) => z = ze,
                None => {
                    lost_lift = true;
                    z = b.z_land_m;
                }
            }
        }
        rec.push(k, t, &[z * 1e3, h.ln2_frac, fz(&h, z) / w]);
        k += 1;
    }
    let lift_end = fz(&h, b.z_land_m) / w;
    c.kpi("t_land_command_h", t_land.map_or(f64::NAN, |x| x / 3600.0));
    c.kpi("ln2_frac_at_land", ln2_at_land);
    c.kpi("lift_over_load_min_on_lowering", min_ratio_path);
    c.kpi("hold_down_force_over_load_max", max_hold_down);
    c.kpi("lift_over_load_at_end", lift_end);
    c.kpi("ln2_hold_h", hold_s / 3600.0);
    c.kpi("fade_time_min", b.fade_time_s() / 60.0);
    c.kpi("landing_speed_mps", v_land);
    c.note("After the landing the module rests on its skids; the lift then fades within tens of minutes of LN₂ depletion with no drop height left.");
    c.check(
        "lands_at_10pct_ln2",
        landed
            && ln2_at_land <= suspension::LN2_LAND_FRAC + 1e-9
            && ln2_at_land > suspension::LN2_CRITICAL_FRAC,
        ln2_at_land,
        suspension::LN2_LAND_FRAC,
        "LN₂ fraction when the landing starts (land at 10 %, before the 4 % critical level)".into(),
    );
    c.check(
        "no_free_fall",
        landed && !lost_lift && min_ratio_path >= 1.0 - 1.0e-6 && v_land <= 0.05,
        min_ratio_path,
        1.0,
        "lift over load along the controlled lowering (≥ 1: the module never loses support); lowering ≤ 50 mm/s; no loss of lift while levitated".into(),
    );
    c.finish(rec);
}

// ---- XC-ROT-1 -----------------------------------------------------------------------------

fn xc_rot1(c: &mut Ctx) {
    let u10 = c.get("u10_mps");
    let c_l = c.get("c_lift");
    let area = c.get("roof_area_m2");
    let factor = c.get("uplift_factor");
    let oval = c.get("oval_mm") * 1.0e-3;
    let g0 = c.get("gap_mm") * 1.0e-3;
    let lat_clear = c.get("lat_clearance_mm") * 1.0e-3;
    let pole_w = c.get("pole_width_mm") * 1.0e-3;
    let cap = c.get("capacity_ratio");
    c.model(
        "ROTATOR static per-station admission: 36 stations, wind uplift (1 + cos θ) windward, in-plane ovalisation cos 2θ against the lateral clearance and pole overlap, run-or-land policy (data and admission only)",
        true,
    );
    let m = f64::from(presets::ROTATOR.design_mass.0);
    let w = m * G0_MPS2;
    let n = 36_u32;
    let w_s = w / f64::from(n);
    let q = 0.5 * 1.225 * u10 * u10;
    let uplift = factor * c_l * q * area;
    let u_s = uplift / f64::from(n);
    let (mut hooks, mut over_cap, mut lat_hit) = (0_u32, 0_u32, 0_u32);
    let (mut max_util, mut max_off) = (0.0_f64, 0.0_f64);
    for i in 0..n {
        let th = TAU * f64::from(i) / f64::from(n);
        let up = u_s * (1.0 + fmath::cos(th));
        let off = fmath::abs(oval * fmath::cos(2.0 * th));
        max_off = max_off.max(off);
        if off >= lat_clear {
            lat_hit += 1;
        }
        let need = w_s - up;
        if need < 0.0 {
            // Attractive EMS cannot push: the anti-uplift hooks must carry this station.
            hooks += 1;
            continue;
        }
        let overlap = (1.0 - off / pole_w).max(0.0);
        let util = need / (cap * w_s * overlap).max(f64::MIN_POSITIVE);
        max_util = max_util.max(util);
        if util > 1.0 {
            over_cap += 1;
        }
    }
    let runnable = hooks == 0 && over_cap == 0 && lat_hit == 0;
    c.kpi("uplift_n", uplift);
    c.kpi("weight_n", w);
    c.kpi("uplift_over_weight", uplift / w);
    c.kpi("stations_needing_hooks", f64::from(hooks));
    c.kpi("stations_over_ems_capacity", f64::from(over_cap));
    c.kpi("stations_lateral_contact", f64::from(lat_hit));
    c.kpi("max_ems_utilisation", max_util);
    c.kpi("max_radial_offset_mm", max_off * 1e3);
    c.kpi("lateral_margin_mm", (lat_clear - max_off) * 1e3);
    c.kpi("gap_mm", g0 * 1e3);
    c.label(
        "decision",
        if runnable {
            "run at speed"
        } else {
            "brake, land on the pier and engage the anti-uplift hooks"
        },
    );
    c.note("Data and admission only: dynamic ring simulation is roadmap. The braking-energy absorption check of ROTATOR currently fails and is reported by fabrail-exo energy (FAB-EXO-122).");
    c.check(
        "hooks_engage_when_uplift_exceeds_weight",
        hooks == 0 || !runnable,
        f64::from(hooks),
        0.0,
        "stations whose uplift share exceeds their weight share: the ring is landed with the hooks engaged, never run".into(),
    );
    c.check(
        "no_contact_at_speed",
        !runnable || (max_util <= 1.0 && max_off < lat_clear),
        max_util,
        1.0,
        "when running: every station within its EMS capacity at its overlap (utilisation ≤ 1) and the ovalised ring inside the lateral clearance".into(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(id: ExoScenarioId, class: &str, inputs: &[(&str, f64)]) -> ScenarioResult {
        let opts = ScenarioOptions {
            seed: 42,
            inputs: inputs.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
            record: false,
        };
        let r = run_scenario_with(id, class, &opts).expect("scenario runs");
        println!("{} {}: passed {} — {}", r.name, r.class, r.passed, r.model);
        for c in &r.criteria {
            println!(
                "  [{}] {}: {} (limit {}) — {}",
                if c.passed { "ok" } else { "FAIL" },
                c.name,
                c.value,
                c.limit,
                c.detail
            );
        }
        r
    }

    fn crit<'a>(r: &'a ScenarioResult, name: &str) -> &'a Criterion {
        r.criteria
            .iter()
            .find(|c| c.name == name)
            .unwrap_or_else(|| panic!("criterion {name} in {}", r.name))
    }

    // req: FAB-EXO-606
    #[test]
    fn near_fault_pulse_peak() {
        // Kobe-type pulse (Mw 6.9, 0.9 m/s; EXOTIC §X15) and a resonant Mw 7.6 pulse (Tp
        // 7.3 s ≈ the 7 s tower): the controlled host peak stays ≤ 1.02× uncontrolled.
        for (mw, v) in [(6.9, 0.9), (7.6, 1.2)] {
            let r = run(
                ExoScenarioId::Xs01,
                "sentinel",
                &[("mw", mw), ("v_pulse_mps", v)],
            );
            let c = crit(&r, "host_peak_le_1p02_uncontrolled");
            println!(
                "  Mw {mw}: controlled/uncontrolled {:.4}, controlled/locked {:.4}, active LQR/uncontrolled {:.4}",
                r.kpis["ratio_controlled_to_uncontrolled"],
                r.kpis["ratio_controlled_to_locked"],
                r.kpis["ratio_active_to_uncontrolled"]
            );
            assert!(
                c.passed && c.value <= 1.02,
                "Mw {mw}: controlled host peak ≤ 1.02× uncontrolled (FAB-EXO-606; research metric R): {:.4}",
                c.value
            );
            assert!(
                r.inputs.contains_key("mw") && r.inputs.contains_key("pulse_period_s"),
                "inputs printed"
            );
        }
    }

    // req: FAB-EXO-607
    #[test]
    fn wind_rms_and_activation() {
        // H2 anchors (white-noise force, passive Warburton-force optimum, ζ on ω_s):
        // 44.7 % at μ 1 %, ζ_s 1 %; 30.0 % at ζ_s 2 % (EXOTIC §X6, ±0.1 point).
        for (zs, expect) in [(0.01, 44.7), (0.02, 30.0)] {
            let s = Sentinel::reference(zs, TuningForm::WarburtonForce);
            let red = (1.0 - h2_host_ratio(&s, [0.0; 4]).expect("Lyapunov").sqrt()) * 100.0;
            println!("ζ_s {zs}: passive H2 reduction {red:.2} %");
            assert!(
                (red - expect).abs() <= 0.1,
                "passive RMS reduction {red:.2} % vs {expect} % at ζ_s {zs} (±0.1 point; EXOTIC §X6)"
            );
        }
        let r = run(ExoScenarioId::Xs12, "sentinel", &[]);
        let act = crit(&r, "activation_le_0p3_mg");
        assert!(
            act.passed && act.value <= 0.3,
            "activation ≤ 0.3 mg (FAB-EXO-607): {} mg",
            act.value
        );
        assert_eq!(
            r.labels["lsm"], "ironless (no cogging, no normal force)",
            "SENTINEL trim LSM is ironless"
        );
        assert!(
            r.kpis["iron_core_cogging_mg_hi"] > 0.3,
            "an iron-core LSM's cogging alone would break 0.3 mg: {} mg",
            r.kpis["iron_core_cogging_mg_hi"]
        );
        // A solid-steel (or laminated) rail fails the target: 1 mg.
        assert!(
            host::activation_threshold_mps2(RailCondition::SolidSteel.mu_eq()) / G0_MPS2 * 1e3
                > 0.3
        );
        // Reported against ζ_s, with the ≥ 40 % target applied to the active mode only.
        assert!(r.kpis.contains_key("zeta_s") && r.kpis.contains_key("rms_reduction_passive_pct"));
        let a = crit(&r, "active_rms_reduction_ge_40pct");
        println!(
            "wind: passive {:.1} % (H2 {:.1} %), active {:.1} % (H2 {:.1} %) at ζ_s {}",
            r.kpis["rms_reduction_passive_pct"],
            r.kpis["h2_reduction_passive_pct"],
            r.kpis["rms_reduction_active_pct"],
            r.kpis["h2_reduction_active_pct"],
            r.kpis["zeta_s"]
        );
        assert!(
            a.passed,
            "active-mode RMS reduction ≥ 40 % (FAB-EXO-607 target): {:.1} %",
            a.value
        );
        let peak = r.kpis["host_peak_accel_uncontrolled_mg"];
        assert!(
            (3.0..=7.0).contains(&peak),
            "wind case at 3–7 mg: {peak:.2} mg"
        );
        // At ζ_s 2 % the passive stage falls below 40 %; the active target still holds.
        let r2 = run(ExoScenarioId::Xs12, "sentinel", &[("zeta_s", 0.02)]);
        assert!(r2.kpis["h2_reduction_passive_pct"] < 40.0);
        assert!(
            crit(&r2, "active_rms_reduction_ge_40pct").passed,
            "active ≥ 40 % at ζ_s 2 %"
        );
    }

    // req: FAB-EXO-608
    #[test]
    fn detuning_stability() {
        // ±10 % ω_s drift with the nominal LQR gains and passive tuning: the closed loop
        // stays stable (all closed-loop eigen-decay: the free response decays) and the
        // degradation of the H2 reduction is reported.
        let nominal = Sentinel::reference(0.01, TuningForm::WarburtonForce);
        let k = sentinel_lqr(&nominal, SENTINEL_DT);
        let base = h2_host_ratio(&nominal, k).expect("nominal H2");
        for scale in [0.9, 1.0, 1.1] {
            let mut s = nominal;
            s.w_s *= scale;
            // Same damper (designed for the nominal ω_s); the host drifted.
            let d = nominal.design();
            let p = s.primary();
            let mut sys = TwoDof::new(p, Damper::from_design(&d, 0.0));
            sys.state = [0.1, 0.0, 0.0, 0.0];
            let mut hmd = nominal.hmd(k);
            let e0 = sys.mechanical_energy_j();
            let dt = SENTINEL_DT;
            let mut e_late = 0.0_f64;
            let mut finite = true;
            for i in 0..(1200.0 / dt) as usize {
                let o = hmd.step(&hmd_inputs(&sys, 0.0, false, true, true), dt as f32);
                sys.step(0.0, f64::from(o.force_n), 0.0, dt);
                finite &= sys.state.iter().all(|v| v.is_finite());
                if i as f64 * dt > 1000.0 {
                    e_late = e_late.max(sys.mechanical_energy_j());
                }
            }
            // Stationary white-noise H2 of the detuned closed loop (Lyapunov solvable ⇔
            // Hurwitz for this system).
            let mut sd = nominal;
            sd.w_s *= scale;
            let ratio = {
                let pp = sd.primary();
                let (mut a, b, e) = {
                    let dd = nominal.design();
                    let (m, md) = (pp.m_kg, dd.m_d_kg);
                    let (w, z) = (pp.omega_rad_s, pp.zeta);
                    let (kd, cd) = (dd.k_d_n_per_m, dd.c_d_n_s_per_m);
                    (
                        [
                            [0.0, 1.0, 0.0, 0.0],
                            [-w * w, -2.0 * z * w, kd / m, cd / m],
                            [0.0, 0.0, 0.0, 1.0],
                            [w * w, 2.0 * z * w, -kd / md - kd / m, -cd / md - cd / m],
                        ],
                        [0.0, -1.0 / m, 0.0, 1.0 / md + 1.0 / m],
                        [0.0, 1.0 / m, 0.0, -1.0 / m],
                    )
                };
                for i in 0..4 {
                    for j in 0..4 {
                        a[i][j] -= b[i] * k[j];
                    }
                }
                let mut q = [[0.0; 4]; 4];
                for i in 0..4 {
                    for j in 0..4 {
                        q[i][j] = e[i] * e[j];
                    }
                }
                let pc = lyapunov4(&a, &q).expect("closed loop Lyapunov");
                assert!(
                    pc[0][0] > 0.0 && pc[1][1] > 0.0,
                    "positive variances (stable closed loop)"
                );
                let var0 = 1.0 / (4.0 * pp.zeta * pp.omega_rad_s.powi(3) * pp.m_kg * pp.m_kg);
                pc[0][0] / var0
            };
            let red = (1.0 - ratio.sqrt()) * 100.0;
            println!(
                "ω_s × {scale}: free response energy {:.3e} → {:.3e} J; active H2 reduction {red:.1} % (nominal {:.1} %)",
                e0,
                e_late,
                (1.0 - base.sqrt()) * 100.0
            );
            assert!(
                finite && e_late < 1e-6 * e0,
                "ω_s × {scale}: the free response decays (stable; FAB-EXO-608)"
            );
            assert!(
                red > 0.0,
                "ω_s × {scale}: still reduces the response (degradation reported)"
            );
        }
        // The seismic run under detuning stays bounded as well.
        let r = run(ExoScenarioId::Xs01, "sentinel", &[("zeta_s", 0.02)]);
        assert!(r.kpis["host_peak_controlled_m"].is_finite());
    }

    // req: FAB-EXO-703
    #[test]
    fn stroke_within_moat_and_fling() {
        // The design stroke S_D at MCE (±0.9 m for S_M1 = 0.9, T 6 s, ζ 0.2) fits the moat.
        let cfg = PlatformCfg::haven_r();
        let s_d = platform::design_displacement_m(0.9, 6.0, 0.2);
        println!(
            "S_D(MCE) {s_d:.3} m, stroke {} m, moat {} m",
            cfg.stroke_m, cfg.moat_m
        );
        assert!(
            s_d <= cfg.stroke_m + 0.01 && cfg.stroke_m <= cfg.moat_m,
            "S_D at MCE ±0.9 m within the moat (FAB-EXO-703)"
        );
        // XS01 near-fault pulse: stroke inside the moat, no buffer contact.
        let r1 = run(ExoScenarioId::Xs01, "haven_r", &[]);
        assert!(
            crit(&r1, "stroke_within_moat_no_buffer_contact").passed,
            "XS01 HAVEN-R stroke within the moat"
        );
        // XS02 fling: support never lost, bumper energy quoted in MJ, stable landing.
        let r2 = run(ExoScenarioId::Xs02, "haven_r", &[]);
        println!(
            "fling: max radial {:.3} m, bumper {:.4} MJ, peak decel {:.2} g, touchdown {:.3} m/s",
            r2.kpis["max_radial_m"],
            r2.kpis["bumper_energy_mj"],
            r2.kpis["peak_decel_g"],
            r2.kpis["touchdown_mps"]
        );
        assert!(
            crit(&r2, "support_never_lost").passed,
            "the fling loses no support (FAB-EXO-703)"
        );
        let e = crit(&r2, "bumper_energy_quoted_mj");
        assert!(
            e.passed && r2.kpis.contains_key("bumper_energy_mj"),
            "bumper energy quoted in MJ"
        );
        assert!(
            crit(&r2, "stable_landing").passed,
            "stable landing after the fling"
        );
    }

    #[test]
    fn every_scenario_runs_for_its_classes() {
        for id in ExoScenarioId::ALL {
            // Long scenarios run at their shortest admissible inputs here.
            let short: &[(&str, f64)] = match id {
                ExoScenarioId::Xs03 => &[("duration_s", 150.0)],
                ExoScenarioId::Xs04 => &[("duration_s", 610.0)],
                ExoScenarioId::Xs06 => &[("t_shaking_s", 30.0)],
                ExoScenarioId::Xs12 => &[("duration_s", 600.0)],
                _ => &[],
            };
            for class in id.classes() {
                let r = run(id, class.slug(), short);
                assert!(
                    !r.criteria.is_empty(),
                    "{} {}: criteria evaluated",
                    id.name(),
                    class.slug()
                );
                assert!(
                    !r.model.is_empty() && !r.inputs.is_empty(),
                    "model label and inputs printed"
                );
                for (k, v) in &r.kpis {
                    assert!(
                        !v.is_nan()
                            || k.contains("_s")
                            || k.contains("mps")
                            || k.contains("ln2")
                            || k.contains("lift"),
                        "{} {k} is NaN",
                        id.name()
                    );
                }
            }
        }
    }

    #[test]
    fn inputs_are_validated_and_runs_are_deterministic() {
        let bad = ScenarioOptions {
            seed: 1,
            inputs: [("nope".to_string(), 1.0)].into_iter().collect(),
            record: false,
        };
        match run_scenario_with(ExoScenarioId::Xs09, "tidegate", &bad) {
            Err(ExoSimError::Invalid { path, .. }) => assert_eq!(path, "inputs.nope"),
            other => panic!("unknown input rejected, got {other:?}"),
        }
        let out_of_range = ScenarioOptions {
            seed: 1,
            inputs: [("magnitude".to_string(), 12.0)].into_iter().collect(),
            record: false,
        };
        assert!(run_scenario_with(ExoScenarioId::Xs09, "tidegate", &out_of_range).is_err());
        assert!(matches!(
            run_scenario(ExoScenarioId::Xs12, "haven_r", 1),
            Err(ExoSimError::Invalid { .. })
        ));
        assert!(matches!(
            run_scenario(ExoScenarioId::Xs12, "selene", 1),
            Err(ExoSimError::Invalid { .. })
        ));
        assert!(matches!(
            run_scenario(ExoScenarioId::Xs12, "warp", 1),
            Err(ExoSimError::Unknown(_))
        ));
        let a = run_scenario(ExoScenarioId::Xs09, "tidegate", 7).expect("run");
        let b = run_scenario(ExoScenarioId::Xs09, "tidegate", 7).expect("run");
        assert_eq!(a, b, "deterministic per seed");
        let rec = run_scenario_with(
            ExoScenarioId::XcShd1,
            "shield",
            &ScenarioOptions {
                seed: 1,
                record: true,
                ..ScenarioOptions::default()
            },
        )
        .expect("run");
        assert!(rec.rate_hz > 0.0 && rec.channels[0].name == "t_s");
        let n = rec.channels[0].data.len();
        assert!(n > 10 && rec.channels.iter().all(|c| c.data.len() == n));
        let js = rec.to_json(true);
        assert_eq!(js["schema"], EXO_RUN_SCHEMA);
        assert!(js["channels"]["x_m"]["data"].is_array());
        // Every criterion string is the EXOTIC §X17 catalogue text; SELENE has none.
        for id in ExoScenarioId::ALL {
            assert!(!id.classes().contains(&ExoClass::Selene));
            assert!(!id.criteria().is_empty() && !id.input_label().is_empty());
        }
    }
}
