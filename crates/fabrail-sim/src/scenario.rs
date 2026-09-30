//! ScenarioSpec v1 (fabrail.scenario/1) serde schema, Series (axis time|space, dx_m; channel-major LE f64 bytes), Scenario, SimError. Owner: W0-B3 (frozen).
//!
//! A scenario is a JSON document (this module; mirrored by
//! `python/fabrail/schemas/scenario-1.schema.json`) plus named little-endian `f64`
//! byte buffers for its series (core design §B.6 as changed by plan §2b item 16):
//!
//! * the coil mode has a single source, `fidelity.coil`; [`check_compat`] rejects a
//!   parameter set whose `control.rates.coil_mode_voltage` disagrees. SIL ≡ firmware
//!   is claimed in voltage mode only; lag mode is sim-only;
//! * a series declares `axis: "time"` (with `fs_hz`, `t0_s`) or `axis: "space"` (with
//!   `dx_m`, `x0_m`), and its bytes are channel-major LE f64 (all samples of channel 0,
//!   then channel 1, …);
//! * there is no `accel_node` series: the structure ADXL355 signal is derived from
//!   `floor` + `site` + sensor noise;
//! * every physical key carries its unit suffix (`t_s`, `dp_pa`, `c_rail_n_s_per_m`, …).
//!
//! Series roles are fixed by name: `floor` (time; ax, ay, az in m/s², rail frame: x
//! travel, y cross-track, z up), `site` (time; ax, ay, az in m/s², added to the
//! structure accelerometer only: footsteps, slams, HVAC, self-actuation) and `rail`
//! (space; z, y in m, referenced by `plant.rail_profile = "series:rail"`). A series may
//! carry any non-empty subset of its role's channels; a missing channel reads as 0.
//!
//! Example (all optional blocks shown with their defaults where they have one):
//!
//! ```json
//! {"schema": "fabrail.scenario/1", "name": "door_push_open", "duration_s": 6,
//!  "noise_seed": 123,
//!  "fidelity": {"model": "carriage6", "coil": "voltage", "eddy_lag": true,
//!               "kinematic_heave": true},
//!  "record": {"rate_hz": 1000, "channels": "default"},
//!  "initial": {"lev": "levitated", "x_m": 0, "v_mps": 0, "payload_kg": 0,
//!              "temp_c": 20},
//!  "plant": {"mass_scale": 1, "br_scale": 1, "leak_sigma": null,
//!            "c_rail_n_s_per_m": null, "f_hyst_n": null, "leak_area_m2": null,
//!            "rail_profile": "series:rail", "structure": null},
//!  "series": {"floor": {"axis": "time", "fs_hz": 200, "t0_s": 0,
//!                       "channels": ["ax", "ay", "az"]},
//!             "rail": {"axis": "space", "dx_m": 0.001, "x0_m": 0,
//!                      "channels": ["z", "y"]}},
//!  "events": [{"kind": "command", "t_s": 0.5, "cmd": "open"},
//!             {"kind": "hand", "t_s": 1, "t_pts_s": [0, 0.1, 0.6],
//!              "f_pts_n": [0, 8, 0],
//!              "impedance": {"m_kg": 1.5, "b_n_s_per_m": 10, "k_n_per_m": 500}}],
//!  "metrics": {"windows": {"push": [0.9, 2.0]}}}
//! ```
//!
//! Parsing never panics; every rejection is a [`SimError`] whose `Display` reads
//! `path: reason` (e.g. `events[3].dur_s: must be > 0`), which Python surfaces as
//! `ValueError`.

use std::collections::BTreeMap;
use std::fmt;

use fabrail_params::{LandingKind, ParamError, SystemParams};
use fabrail_plant::interaction::ObstacleBody;
use fabrail_plant::params::{Environment, Mismatch, PlantParams};
use fabrail_plant::rail::RailProfile;
use fabrail_plant::structure::ShearBuilding;
use fabrail_plant::{CoilMode, InitLev, InitialState, ModelConfig, PlantError, Tier};
use fabrail_types::{
    BASE_TICK_HZ, MAX_MAGNETS, NetworkAlert, PosNm, ResetCause, RestPolicy, Tick, UserCommand,
};
use serde::de::{self, DeserializeOwned, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::Value;

/// The schema identifier every v1 scenario carries (`"schema"`).
pub const SCENARIO_SCHEMA: &str = "fabrail.scenario/1";

/// Longest accepted scenario, s (24 h; the 24 h detector-noise study streams through
/// `DetectorReplay` instead, plan §2b item 13).
pub const MAX_DURATION_S: f64 = 86_400.0;

/// Series role: floor (rail) acceleration at the device, time axis.
pub const SERIES_FLOOR: &str = "floor";
/// Series role: site noise added to the structure accelerometer only, time axis.
pub const SERIES_SITE: &str = "site";
/// Series role: rail irregularity profile, space axis.
pub const SERIES_RAIL: &str = "rail";
/// Channels of the `floor` and `site` roles (m/s²).
pub const ACCEL_CHANNELS: [&str; 3] = ["ax", "ay", "az"];
/// Channels of the `rail` role (m).
pub const RAIL_CHANNELS: [&str; 2] = ["z", "y"];
/// The only accepted `plant.rail_profile` reference.
pub const RAIL_PROFILE_REF: &str = "series:rail";

// ---------------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------------

/// Everything that can go wrong building or running a scenario.
#[derive(Clone, Debug, PartialEq)]
pub enum SimError {
    /// The text is not valid JSON (serde_json message with line/column).
    Json(String),
    /// The `schema` field names another schema (the value found).
    Schema(String),
    /// A field is missing, unknown, mistyped or out of range.
    Invalid {
        /// Dotted path, e.g. `events[3].dur_s`, `series.floor`, `scenario`.
        path: String,
        /// Human-readable reason.
        reason: String,
    },
    /// A series byte buffer does not match its spec.
    Series {
        /// Series name.
        name: String,
        /// Reason.
        reason: String,
    },
    /// The parameter set was rejected by `fabrail_params::validate`.
    Param(ParamError),
    /// `fidelity.coil` and `control.rates.coil_mode_voltage` disagree (plan §2b item 16).
    CoilModeMismatch {
        /// The scenario asks for voltage mode.
        scenario_voltage: bool,
        /// The parameters ask for voltage mode.
        params_voltage: bool,
    },
    /// The plant step is too large for an oscillatory element (PHYSICS §24).
    Unstable {
        /// Element name.
        element: String,
        /// Its |λ·h|.
        lambda_h: f64,
    },
    /// `record.channels` names a channel the recorder does not know.
    UnknownChannel(String),
    /// The plant rejected its configuration.
    Plant(String),
    /// A Wave-0 stub was called (the Wave-1 owner is named).
    NotImplemented(&'static str),
    /// A runtime failure (e.g. non-finite truth, time running backwards).
    Runtime(String),
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SimError::Json(m) => write!(f, "scenario: invalid JSON: {m}"),
            SimError::Schema(found) => {
                write!(
                    f,
                    "schema: expected \"{SCENARIO_SCHEMA}\", found \"{found}\""
                )
            }
            SimError::Invalid { path, reason } => write!(f, "{path}: {reason}"),
            SimError::Series { name, reason } => write!(f, "series.{name}: {reason}"),
            SimError::Param(e) => write!(f, "{e}"),
            SimError::CoilModeMismatch {
                scenario_voltage,
                params_voltage,
            } => write!(
                f,
                "fidelity.coil: the scenario says \"{}\" but control.rates.coil_mode_voltage is {} \
                 (the scenario is the single source of the coil mode, plan §2b item 16)",
                if *scenario_voltage { "voltage" } else { "lag" },
                params_voltage
            ),
            SimError::Unstable { element, lambda_h } => write!(
                f,
                "stability: element {element} has |lambda*h| = {lambda_h:.4} > 0.2 (PHYSICS §24)"
            ),
            SimError::UnknownChannel(n) => write!(f, "record.channels: unknown channel \"{n}\""),
            SimError::Plant(m) => write!(f, "plant: {m}"),
            SimError::NotImplemented(what) => write!(f, "not implemented yet: {what}"),
            SimError::Runtime(m) => write!(f, "runtime: {m}"),
        }
    }
}

impl std::error::Error for SimError {}

impl From<ParamError> for SimError {
    fn from(e: ParamError) -> Self {
        SimError::Param(e)
    }
}

impl From<PlantError> for SimError {
    fn from(e: PlantError) -> Self {
        match e {
            PlantError::Invalid(path, reason) => invalid(path, reason),
            other => SimError::Plant(other.to_string()),
        }
    }
}

fn invalid(path: impl Into<String>, reason: impl ToString) -> SimError {
    SimError::Invalid {
        path: path.into(),
        reason: reason.to_string(),
    }
}

// ---------------------------------------------------------------------------------
// Spec types
// ---------------------------------------------------------------------------------

fn default_true() -> bool {
    true
}
fn default_one() -> f64 {
    1.0
}
fn default_rate_hz() -> f64 {
    1000.0
}
fn default_temp_c() -> f64 {
    20.0
}
fn default_lean_rise_s() -> f64 {
    1.0
}
fn default_smoke_ramp_s() -> f64 {
    2.0
}
fn default_rayleigh() -> [f64; 2] {
    [0.02, 0.02]
}

/// A scenario document (`fabrail.scenario/1`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioSpec {
    /// Always [`SCENARIO_SCHEMA`].
    pub schema: String,
    /// Scenario name (`[A-Za-z0-9_.-]`, 1–64 characters).
    pub name: String,
    /// Free-text description.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    /// Run length, s (0 < duration ≤ 86 400).
    pub duration_s: f64,
    /// Seed of the Rust sensor-noise and fault-injection substreams (`rng`).
    pub noise_seed: u64,
    /// Model tier and coil mode (the single source of the coil mode).
    pub fidelity: Fidelity,
    /// Recording rate and channel selection.
    #[serde(default)]
    pub record: RecordSpec,
    /// Initial condition.
    #[serde(default)]
    pub initial: InitialSpec,
    /// Truth-vs-nominal perturbations and environment.
    #[serde(default)]
    pub plant: PlantSpec,
    /// Series declarations (bytes are passed separately).
    #[serde(default)]
    pub series: BTreeMap<String, SeriesSpec>,
    /// Timed events (applied in time order; ties keep document order).
    #[serde(default)]
    pub events: Vec<EventSpec>,
    /// KPI windows.
    #[serde(default)]
    pub metrics: MetricsSpec,
}

/// `fidelity` block.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fidelity {
    /// Plant tier.
    pub model: FidelityModel,
    /// Coil mode (single source; plan §2b item 16).
    pub coil: FidelityCoil,
    /// Model the eddy lag on the force flux.
    #[serde(default = "default_true")]
    pub eddy_lag: bool,
    /// Travel1d: emulate heave kinematically so LAND/SEAT complete (plan §2b item 16).
    #[serde(default = "default_true")]
    pub kinematic_heave: bool,
}

/// `fidelity.model`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FidelityModel {
    /// Full rigid body, 50 µs.
    Carriage6,
    /// Heave only, 50 µs.
    Heave1,
    /// Travel axis, 1 ms.
    Travel1d,
    /// Shear building + maglev mass, 1 ms.
    Structure,
}

impl FidelityModel {
    /// The plant tier.
    pub const fn tier(self) -> Tier {
        match self {
            FidelityModel::Carriage6 => Tier::Carriage6,
            FidelityModel::Heave1 => Tier::Heave1,
            FidelityModel::Travel1d => Tier::Travel1d,
            FidelityModel::Structure => Tier::Structure,
        }
    }
}

/// `fidelity.coil`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FidelityCoil {
    /// Current-loop lag (sim-only).
    Lag,
    /// Bridge voltages into the flux-linkage ODE (SIL ≡ firmware).
    Voltage,
}

impl FidelityCoil {
    /// The plant coil mode.
    pub const fn mode(self) -> CoilMode {
        match self {
            FidelityCoil::Lag => CoilMode::Lag,
            FidelityCoil::Voltage => CoilMode::Voltage,
        }
    }

    /// True for voltage mode.
    pub const fn is_voltage(self) -> bool {
        matches!(self, FidelityCoil::Voltage)
    }
}

/// `record` block.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordSpec {
    /// Recording rate, Hz; 20 000 / rate_hz must be a whole number of plant steps.
    #[serde(default = "default_rate_hz")]
    pub rate_hz: f64,
    /// `"default"`, `"kpis_only"` or a list of channel names.
    #[serde(default)]
    pub channels: ChannelSelect,
}

impl Default for RecordSpec {
    fn default() -> Self {
        RecordSpec {
            rate_hz: default_rate_hz(),
            channels: ChannelSelect::Default,
        }
    }
}

/// Channel selection: a preset string or an explicit list.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum ChannelSelect {
    /// The recorder's default channel set.
    #[default]
    Default,
    /// No channels; KPIs and events only.
    KpisOnly,
    /// Exactly these channels, in this order.
    Names(Vec<String>),
}

impl Serialize for ChannelSelect {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        match self {
            ChannelSelect::Default => s.serialize_str("default"),
            ChannelSelect::KpisOnly => s.serialize_str("kpis_only"),
            ChannelSelect::Names(v) => v.serialize(s),
        }
    }
}

impl<'de> Deserialize<'de> for ChannelSelect {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        struct V;
        impl<'de> Visitor<'de> for V {
            type Value = ChannelSelect;
            fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str("\"default\", \"kpis_only\" or a list of channel names")
            }
            fn visit_str<E: de::Error>(self, v: &str) -> Result<ChannelSelect, E> {
                match v {
                    "default" => Ok(ChannelSelect::Default),
                    "kpis_only" => Ok(ChannelSelect::KpisOnly),
                    other => Err(E::invalid_value(de::Unexpected::Str(other), &self)),
                }
            }
            fn visit_seq<A: SeqAccess<'de>>(self, mut seq: A) -> Result<ChannelSelect, A::Error> {
                let mut names = Vec::new();
                while let Some(n) = seq.next_element::<String>()? {
                    names.push(n);
                }
                Ok(ChannelSelect::Names(names))
            }
        }
        d.deserialize_any(V)
    }
}

/// `initial` block. `None` fields resolve from the parameter set
/// ([`InitialSpec::resolve`]).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InitialSpec {
    /// Vertical rest condition; `None` = the class rest state from `design.rest_policy`
    /// (seat → seated, hover → levitated, land / land_lock → landed).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lev: Option<InitLevSpec>,
    /// Travel position, m; `None` = the closed stop `design.rail.x_closed_m`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub x_m: Option<f64>,
    /// Travel velocity, m/s.
    #[serde(default)]
    pub v_mps: f64,
    /// Payload on the body at t = 0, kg (negative allowed down to the empty body:
    /// truth mass = plant.mass_scale · m* + payload_kg must stay > 0).
    #[serde(default)]
    pub payload_kg: f64,
    /// Initial temperature of every thermal node and the ambient, °C.
    #[serde(default = "default_temp_c")]
    pub temp_c: f64,
    /// Parking lock engaged at t = 0; `None` = engaged iff the rest policy is
    /// `land_lock` and the carriage starts landed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub locked: Option<bool>,
}

impl Default for InitialSpec {
    fn default() -> Self {
        InitialSpec {
            lev: None,
            x_m: None,
            v_mps: 0.0,
            payload_kg: 0.0,
            temp_c: default_temp_c(),
            locked: None,
        }
    }
}

/// `initial.lev`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InitLevSpec {
    /// DRAWER captive seat.
    Seated,
    /// On the rollers.
    Landed,
    /// Levitated.
    Levitated,
}

impl From<InitLevSpec> for InitLev {
    fn from(v: InitLevSpec) -> InitLev {
        match v {
            InitLevSpec::Seated => InitLev::Seated,
            InitLevSpec::Landed => InitLev::Landed,
            InitLevSpec::Levitated => InitLev::Levitated,
        }
    }
}

impl InitialSpec {
    /// The plant initial state for parameter set `p` (fills every `None`).
    pub fn resolve(&self, p: &SystemParams) -> InitialState {
        let rest = p.design.rest_policy;
        let lev = match self.lev {
            Some(l) => l.into(),
            None => match rest {
                RestPolicy::Seat => InitLev::Seated,
                RestPolicy::Hover => InitLev::Levitated,
                RestPolicy::Land | RestPolicy::LandLock => InitLev::Landed,
            },
        };
        let locked = self
            .locked
            .unwrap_or(matches!(rest, RestPolicy::LandLock) && lev == InitLev::Landed);
        InitialState {
            lev,
            x_m: self.x_m.unwrap_or(p.design.rail.x_closed_m),
            v_mps: self.v_mps,
            payload_kg: self.payload_kg,
            temp_c: self.temp_c,
            locked,
        }
    }
}

/// `plant` block: truth-vs-nominal perturbations and environment references. `None`
/// overrides mean "the design value".
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlantSpec {
    /// Body mass scale (truth m_body = mass_scale · m*), (0, 10].
    #[serde(default = "default_one")]
    pub mass_scale: f64,
    /// Remanence scale, (0, 10].
    #[serde(default = "default_one")]
    pub br_scale: f64,
    /// Flux leakage σ_leak override, [0, 1).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub leak_sigma: Option<f64>,
    /// Rail eddy drag override, N·s/m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub c_rail_n_s_per_m: Option<f64>,
    /// Rail hysteresis drag override, N.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub f_hyst_n: Option<f64>,
    /// DRAWER cavity leak area override, m².
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub leak_area_m2: Option<f64>,
    /// `"series:rail"` to use the rail series as the rail profile; `None` = ideal rail.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rail_profile: Option<String>,
    /// Shear building for `fidelity.model = "structure"` (required there, rejected
    /// elsewhere).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub structure: Option<StructureSpec>,
}

impl Default for PlantSpec {
    fn default() -> Self {
        PlantSpec {
            mass_scale: 1.0,
            br_scale: 1.0,
            leak_sigma: None,
            c_rail_n_s_per_m: None,
            f_hyst_n: None,
            leak_area_m2: None,
            rail_profile: None,
            structure: None,
        }
    }
}

impl PlantSpec {
    /// The plant mismatch.
    pub fn mismatch(&self) -> Mismatch {
        Mismatch {
            mass_scale: self.mass_scale,
            br_scale: self.br_scale,
            leak_sigma: self.leak_sigma,
            c_rail_n_s_per_m: self.c_rail_n_s_per_m,
            f_hyst_n: self.f_hyst_n,
            leak_area_m2: self.leak_area_m2,
        }
    }
}

/// `plant.structure`: an n-storey shear building (PHYSICS §21).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StructureSpec {
    /// Storey masses, bottom to top, kg.
    pub storey_mass_kg: Vec<f64>,
    /// Storey shear stiffnesses, bottom to top, N/m.
    pub storey_k_n_per_m: Vec<f64>,
    /// Rayleigh damping ratios of modes 1 and 2 (default 2 %, 2 %).
    #[serde(default = "default_rayleigh")]
    pub rayleigh_zeta: [f64; 2],
    /// Storey carrying the device, 1-based.
    pub device_storey: u32,
}

impl StructureSpec {
    /// The plant shear building.
    pub fn to_shear_building(&self) -> ShearBuilding {
        ShearBuilding {
            storey_mass_kg: self.storey_mass_kg.clone(),
            storey_k_n_per_m: self.storey_k_n_per_m.clone(),
            rayleigh_zeta: self.rayleigh_zeta,
            device_storey: self.device_storey,
        }
    }
}

/// Series coordinate axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    /// Uniform in time: t = t0_s + k / fs_hz.
    Time,
    /// Uniform in space: x = x0_m + k · dx_m.
    Space,
}

/// A series declaration. Time series set `fs_hz` (and optionally `t0_s`); space series
/// set `dx_m` (and optionally `x0_m`); the other pair must be absent.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SeriesSpec {
    /// Coordinate axis.
    pub axis: Axis,
    /// Sample rate (time axis), Hz.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fs_hz: Option<f64>,
    /// Time of sample 0 (time axis), s; default 0.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t0_s: Option<f64>,
    /// Grid spacing (space axis), m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dx_m: Option<f64>,
    /// Coordinate of sample 0 (space axis), m; default 0.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub x0_m: Option<f64>,
    /// Channel names, in byte order (channel-major).
    pub channels: Vec<String>,
}

impl SeriesSpec {
    /// (start, step) of the grid: (t0, 1/fs) or (x0, dx). Only meaningful after
    /// validation.
    pub fn grid(&self) -> (f64, f64) {
        match self.axis {
            Axis::Time => (self.t0_s.unwrap_or(0.0), 1.0 / self.fs_hz.unwrap_or(1.0)),
            Axis::Space => (self.x0_m.unwrap_or(0.0), self.dx_m.unwrap_or(1.0)),
        }
    }
}

/// `metrics` block.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MetricsSpec {
    /// Named KPI windows [t0_s, t1_s].
    #[serde(default)]
    pub windows: BTreeMap<String, [f64; 2]>,
}

// ---------------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------------

/// Arm impedance of a hand event (core design §C.5: m 1–2 kg, b 5–20 N·s/m,
/// k 200–1000 N/m).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Impedance {
    /// Arm effective mass, kg.
    pub m_kg: f64,
    /// Arm damping, N·s/m.
    pub b_n_s_per_m: f64,
    /// Arm stiffness, N/m.
    pub k_n_per_m: f64,
}

impl Default for Impedance {
    fn default() -> Self {
        Impedance {
            m_kg: 1.5,
            b_n_s_per_m: 10.0,
            k_n_per_m: 500.0,
        }
    }
}

/// `command.cmd`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandKind {
    /// Open (log-damped open terminus).
    Open,
    /// Close (log-damped close terminus, then seat/land).
    Close,
    /// Go to `x_m`.
    GoTo,
    /// Controlled stop.
    Stop,
    /// Lift / unseat.
    Lift,
    /// Land / seat.
    Land,
    /// Operator reset of a latched FAULT / INSPECT (`UserCommand::Reset`).
    Reset,
    /// Enter MANUAL.
    Manual,
}

impl CommandKind {
    /// The user command (`x_m` is used by `go_to` only).
    pub fn to_user_command(self, x_m: Option<f64>) -> UserCommand {
        match self {
            CommandKind::Open => UserCommand::Open,
            CommandKind::Close => UserCommand::Close,
            CommandKind::GoTo => UserCommand::GoTo(PosNm::from_m_f64(x_m.unwrap_or(0.0))),
            CommandKind::Stop => UserCommand::Stop,
            CommandKind::Lift => UserCommand::Lift,
            CommandKind::Land => UserCommand::Land,
            CommandKind::Reset => UserCommand::Reset,
            CommandKind::Manual => UserCommand::Manual,
        }
    }
}

/// `hand.contact`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HandContact {
    /// Gripping the handle (can pull).
    #[default]
    Grip,
    /// Push-only contact (separates when the body moves away; push-to-open).
    Push,
}

/// `obstacle.body` (ISO/TS 15066 body regions).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObstacleBodyKind {
    /// Chest.
    Chest,
    /// Hand.
    Hand,
    /// Face.
    Face,
    /// Leg.
    Leg,
}

impl From<ObstacleBodyKind> for ObstacleBody {
    fn from(v: ObstacleBodyKind) -> ObstacleBody {
        match v {
            ObstacleBodyKind::Chest => ObstacleBody::Chest,
            ObstacleBodyKind::Hand => ObstacleBody::Hand,
            ObstacleBodyKind::Face => ObstacleBody::Face,
            ObstacleBodyKind::Leg => ObstacleBody::Leg,
        }
    }
}

/// `obstacle.config`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObstacleConfig {
    /// Clamped against a rigid backing.
    Clamped,
    /// Free to be pushed.
    Free,
}

/// `control_loss.dir`: the coil drivers stick at full positive (up) or negative (down)
/// output (fail-direction tests).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlLossDir {
    /// Full attraction: the carriage snaps to the upper stop.
    Up,
    /// Full release: the carriage drops to the rollers.
    Down,
}

/// `fault.what`: sensor and driver faults applied by SimBoard.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SensorFaultKind {
    /// Coil H-bridge `ch` latches a driver fault (output off).
    CoilDriver,
    /// LSM inverter latches a driver fault (`ch` = phase 0–2, informational).
    LsmDriver,
    /// Gap sensor `ch` freezes at its current code.
    GapStuck,
    /// Gap sensor `ch` reports invalid conversions.
    GapLost,
    /// Encoder counts jump by `jump_m` (default 1 mm).
    EncoderJump,
    /// Encoder signal error.
    EncoderLost,
    /// Structure accelerometer axis `ch` freezes.
    AccelStuck,
    /// Structure accelerometer FIFO stops delivering.
    AccelLost,
    /// Carriage IMU stops responding.
    ImuLost,
    /// Rail floor accelerometer stops responding.
    RailAccelLost,
}

impl SensorFaultKind {
    /// Largest valid `ch` (inclusive) for this fault.
    pub const fn max_channel(self) -> u8 {
        match self {
            SensorFaultKind::CoilDriver | SensorFaultKind::GapStuck | SensorFaultKind::GapLost => {
                (MAX_MAGNETS - 1) as u8
            }
            SensorFaultKind::LsmDriver
            | SensorFaultKind::AccelStuck
            | SensorFaultKind::ImuLost
            | SensorFaultKind::RailAccelLost => 2,
            SensorFaultKind::EncoderJump
            | SensorFaultKind::EncoderLost
            | SensorFaultKind::AccelLost => 0,
        }
    }
}

/// Default encoder jump when `fault.jump_m` is absent, m (test default: 1000 counts at the
/// 1 µm/count encoder of core design §C.5).
pub const DEFAULT_ENCODER_JUMP_M: f64 = 1.0e-3;

/// `reset.cause`: why the controller reboots (plant state is preserved).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RebootCause {
    /// Software reset.
    #[default]
    Software,
    /// Independent watchdog.
    Watchdog,
    /// Brown-out.
    BrownOut,
    /// Reset pin.
    Pin,
    /// Panic/HardFault handler (crash record written; boot goes to FAULT).
    Crash,
}

impl From<RebootCause> for ResetCause {
    fn from(v: RebootCause) -> ResetCause {
        match v {
            RebootCause::Software => ResetCause::Software,
            RebootCause::Watchdog => ResetCause::IndependentWatchdog,
            RebootCause::BrownOut => ResetCause::BrownOut,
            RebootCause::Pin => ResetCause::Pin,
            RebootCause::Crash => ResetCause::Crash,
        }
    }
}

/// `core_fault.what`: perturbations of core outputs BEFORE the monitor sees them (rt
/// feature `fault-injection`; SAF "monitor catches core fault").
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CoreFaultKind {
    /// LSM i_q reference multiplied by 3.
    IqGainX3,
    /// Gap measurement sign inverted inside the core.
    GapSign,
    /// Core outputs become NaN.
    Nan,
}

/// One timed scenario event (`"kind"` selects the variant; every variant has `t_s`, the
/// absolute scenario time in seconds, 0 ≤ t_s ≤ duration_s).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EventSpec {
    /// A user command on the command link.
    Command {
        /// Event time, s.
        t_s: f64,
        /// Command.
        cmd: CommandKind,
        /// Target for `go_to` (required there, rejected otherwise), m.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        x_m: Option<f64>,
    },
    /// A hand grips/pushes the front: piecewise-linear driving force through the arm
    /// impedance; the hand releases after the last point.
    Hand {
        /// Contact time, s.
        t_s: f64,
        /// Profile times relative to `t_s` (≥ 0, strictly increasing), s.
        t_pts_s: Vec<f64>,
        /// Driving force at each profile time (+x opens), N.
        f_pts_n: Vec<f64>,
        /// Arm impedance.
        #[serde(default)]
        impedance: Impedance,
        /// Grip or push-only contact.
        #[serde(default)]
        contact: HandContact,
    },
    /// Payload added (or removed, negative) at `t_s`.
    Payload {
        /// Event time, s.
        t_s: f64,
        /// Mass change, kg.
        delta_kg: f64,
    },
    /// An ISO/TS 15066 body appears in the closing path.
    Obstacle {
        /// Event time, s.
        t_s: f64,
        /// Body region.
        body: ObstacleBodyKind,
        /// Rail position of the body surface, m.
        x_m: f64,
        /// Clamped or free.
        config: ObstacleConfig,
        /// Edge-seal stiffness override, N/m (default `design.damping.k_edge_n_per_m`).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        k_edge_n_per_m: Option<f64>,
        /// Presence duration, s (default: until the end).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        dur_s: Option<f64>,
    },
    /// A person leans on the front: symmetric trapezoid rising over `rise_s` to `f_n`
    /// and back to 0 at `t_s + dur_s` (plan §2b item 11: rise ≥ 1 s is the specified
    /// disturbance profile).
    Lean {
        /// Onset, s.
        t_s: f64,
        /// Plateau force along x, N.
        f_n: f64,
        /// Total duration, s (≥ 2·rise_s).
        dur_s: f64,
        /// Ramp time, s.
        #[serde(default = "default_lean_rise_s")]
        rise_s: f64,
    },
    /// Smoke-control / cabinet pressure difference on the leaf/front (positive pushes
    /// toward +x): ramps over `ramp_s`, holds, and ramps back over `ramp_s` ending at
    /// `t_s + dur_s` when `dur_s` is given (plan §2b item 11: ramp ≥ 2 s).
    SmokeDp {
        /// Onset, s.
        t_s: f64,
        /// Plateau pressure difference, Pa.
        dp_pa: f64,
        /// Ramp time, s.
        #[serde(default = "default_smoke_ramp_s")]
        ramp_s: f64,
        /// Total duration, s (default: until the end).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        dur_s: Option<f64>,
    },
    /// The DRAWER cavity leak area changes to `leak_area_m2` (piston pulse tests).
    Piston {
        /// Event time, s.
        t_s: f64,
        /// New leak area, m².
        leak_area_m2: f64,
    },
    /// The ambient temperature ramps linearly by `dt_k` over `dur_s`.
    TempRamp {
        /// Ramp start, s.
        t_s: f64,
        /// Temperature change, K.
        dt_k: f64,
        /// Ramp duration (0 = step), s.
        dur_s: f64,
    },
    /// Mains lost (the bus runs from the hold-up); restored after `dur_s` if given.
    PowerLoss {
        /// Event time, s.
        t_s: f64,
        /// Outage duration, s (default: until the end).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        dur_s: Option<f64>,
    },
    /// The coil drivers stick at full output (fail-direction tests).
    ControlLoss {
        /// Event time, s.
        t_s: f64,
        /// Up (full attraction) or down (full release).
        dir: ControlLossDir,
    },
    /// A sensor or driver fault (SimBoard).
    Fault {
        /// Event time, s.
        t_s: f64,
        /// Fault.
        what: SensorFaultKind,
        /// Channel (magnet index, axis or phase; see `SensorFaultKind::max_channel`).
        #[serde(default)]
        ch: u8,
        /// Encoder jump, m (`encoder_jump` only; default 1 mm).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        jump_m: Option<f64>,
    },
    /// A synthetic/injected network earthquake alert arrives on the alert link (not a
    /// live EEW feed).
    NetworkAlert {
        /// Receipt time, s.
        t_s: f64,
        /// Moment magnitude estimate.
        magnitude: f64,
        /// Epicentral distance, km.
        r_epi_km: f64,
        /// Hypocentral depth, km.
        depth_km: f64,
        /// Origin time (absolute scenario time, may be negative, ≤ t_s), s.
        origin_t_s: f64,
        /// Predicted site intensity (MMI).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        mmi: Option<f64>,
        /// Alert id (default: 1-based order among the scenario's network alerts).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        id: Option<u32>,
    },
    /// Cancels alert `id` (default: the most recent alert).
    AlertCancel {
        /// Event time, s.
        t_s: f64,
        /// Alert id.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        id: Option<u32>,
    },
    /// The controller reboots (BOOT → SELFTEST; the plant keeps its state). The
    /// operator's FAULT/INSPECT reset is `command` with `cmd: "reset"`.
    Reset {
        /// Event time, s.
        t_s: f64,
        /// Reset cause reported by the watchdog HAL.
        #[serde(default)]
        cause: RebootCause,
    },
    /// Core-output fault injection (rt `fault-injection`).
    CoreFault {
        /// Event time, s.
        t_s: f64,
        /// Perturbation.
        what: CoreFaultKind,
        /// Duration, s (default: until the end).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        dur_s: Option<f64>,
    },
    /// Presence-sensing input (`SafetyInRaw.presence_active` / `presence_ok`).
    Presence {
        /// Event time, s.
        t_s: f64,
        /// A person is in the protected zone.
        active: bool,
        /// Device healthy (OSSD pair consistent).
        #[serde(default = "default_true")]
        ok: bool,
    },
    /// Hold-to-run input line (`SafetyInRaw.hold_to_run`).
    HoldToRun {
        /// Event time, s.
        t_s: f64,
        /// Actuated.
        on: bool,
    },
    /// Building fire-alarm input (`SafetyInRaw.fire_alarm`).
    FireAlarm {
        /// Event time, s.
        t_s: f64,
        /// Alarm active.
        active: bool,
    },
    /// Emergency-stop input (`SafetyInRaw.estop`).
    Estop {
        /// Event time, s.
        t_s: f64,
        /// Pressed.
        active: bool,
    },
}

impl EventSpec {
    /// Every `kind` value, in declaration order (drift-checked against the JSON schema).
    pub const KINDS: &'static [&'static str] = &[
        "command",
        "hand",
        "payload",
        "obstacle",
        "lean",
        "smoke_dp",
        "piston",
        "temp_ramp",
        "power_loss",
        "control_loss",
        "fault",
        "network_alert",
        "alert_cancel",
        "reset",
        "core_fault",
        "presence",
        "hold_to_run",
        "fire_alarm",
        "estop",
    ];

    /// Event time, s.
    pub fn t_s(&self) -> f64 {
        match self {
            EventSpec::Command { t_s, .. }
            | EventSpec::Hand { t_s, .. }
            | EventSpec::Payload { t_s, .. }
            | EventSpec::Obstacle { t_s, .. }
            | EventSpec::Lean { t_s, .. }
            | EventSpec::SmokeDp { t_s, .. }
            | EventSpec::Piston { t_s, .. }
            | EventSpec::TempRamp { t_s, .. }
            | EventSpec::PowerLoss { t_s, .. }
            | EventSpec::ControlLoss { t_s, .. }
            | EventSpec::Fault { t_s, .. }
            | EventSpec::NetworkAlert { t_s, .. }
            | EventSpec::AlertCancel { t_s, .. }
            | EventSpec::Reset { t_s, .. }
            | EventSpec::CoreFault { t_s, .. }
            | EventSpec::Presence { t_s, .. }
            | EventSpec::HoldToRun { t_s, .. }
            | EventSpec::FireAlarm { t_s, .. }
            | EventSpec::Estop { t_s, .. } => *t_s,
        }
    }

    /// The `kind` string.
    pub fn kind(&self) -> &'static str {
        match self {
            EventSpec::Command { .. } => "command",
            EventSpec::Hand { .. } => "hand",
            EventSpec::Payload { .. } => "payload",
            EventSpec::Obstacle { .. } => "obstacle",
            EventSpec::Lean { .. } => "lean",
            EventSpec::SmokeDp { .. } => "smoke_dp",
            EventSpec::Piston { .. } => "piston",
            EventSpec::TempRamp { .. } => "temp_ramp",
            EventSpec::PowerLoss { .. } => "power_loss",
            EventSpec::ControlLoss { .. } => "control_loss",
            EventSpec::Fault { .. } => "fault",
            EventSpec::NetworkAlert { .. } => "network_alert",
            EventSpec::AlertCancel { .. } => "alert_cancel",
            EventSpec::Reset { .. } => "reset",
            EventSpec::CoreFault { .. } => "core_fault",
            EventSpec::Presence { .. } => "presence",
            EventSpec::HoldToRun { .. } => "hold_to_run",
            EventSpec::FireAlarm { .. } => "fire_alarm",
            EventSpec::Estop { .. } => "estop",
        }
    }

    /// The user command of a `command` event.
    pub fn user_command(&self) -> Option<UserCommand> {
        match self {
            EventSpec::Command { cmd, x_m, .. } => Some(cmd.to_user_command(*x_m)),
            _ => None,
        }
    }

    /// The alert of a `network_alert` event, received at its `t_s` tick. `default_id`
    /// is used when the event has no `id`; an absent `mmi` is sent as 0 (unknown); the
    /// origin is sent relative to receipt (`NetworkAlert::origin_offset_s` =
    /// origin_t_s − t_s ≤ 0), so an origin before the run start is kept.
    pub fn network_alert(&self, default_id: u32) -> Option<NetworkAlert> {
        match self {
            EventSpec::NetworkAlert {
                t_s,
                magnitude,
                r_epi_km,
                depth_km,
                origin_t_s,
                mmi,
                id,
            } => Some(NetworkAlert {
                id: id.unwrap_or(default_id),
                received: Tick::from_seconds_f64(*t_s),
                origin_offset_s: (*origin_t_s - *t_s) as f32,
                magnitude: *magnitude as f32,
                r_epi_km: *r_epi_km as f32,
                depth_km: *depth_km as f32,
                mmi: mmi.unwrap_or(0.0) as f32,
                cancel: false,
            }),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------------
// Parsing and validation
// ---------------------------------------------------------------------------------

fn check_finite(path: impl Fn() -> String, v: f64) -> Result<(), SimError> {
    if v.is_finite() {
        Ok(())
    } else {
        Err(invalid(path(), "must be finite"))
    }
}

fn check_range(path: impl Fn() -> String, v: f64, lo: f64, hi: f64) -> Result<(), SimError> {
    if v.is_finite() && v >= lo && v <= hi {
        Ok(())
    } else {
        Err(invalid(
            path(),
            format!("must be in [{lo}, {hi}] (got {v})"),
        ))
    }
}

fn check_positive(path: impl Fn() -> String, v: f64) -> Result<(), SimError> {
    if v.is_finite() && v > 0.0 {
        Ok(())
    } else {
        Err(invalid(path(), format!("must be finite and > 0 (got {v})")))
    }
}

fn is_ident(s: &str) -> bool {
    let mut chars = s.chars();
    matches!(chars.next(), Some(c) if c.is_ascii_lowercase())
        && chars.all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
        && s.len() <= 64
}

/// Deserialises one section for error localisation.
fn probe<T: DeserializeOwned>(v: &Value, path: String) -> Result<(), SimError> {
    serde_json::from_value::<T>(v.clone())
        .map(|_| ())
        .map_err(|e| invalid(path, e))
}

/// Turns a whole-document serde error into a path-qualified one by re-deserialising
/// each section and each event on its own.
fn locate(doc: &Value, top: serde_json::Error) -> SimError {
    let Some(obj) = doc.as_object() else {
        return invalid("scenario", "must be a JSON object");
    };
    type Probe = fn(&Value, String) -> Result<(), SimError>;
    let sections: [(&str, Probe); 5] = [
        ("fidelity", probe::<Fidelity>),
        ("record", probe::<RecordSpec>),
        ("initial", probe::<InitialSpec>),
        ("plant", probe::<PlantSpec>),
        ("metrics", probe::<MetricsSpec>),
    ];
    for (key, f) in sections {
        if let Some(v) = obj.get(key)
            && let Err(e) = f(v, key.to_string())
        {
            return e;
        }
    }
    if let Some(Value::Object(m)) = obj.get("series") {
        for (k, v) in m {
            if let Err(e) = probe::<SeriesSpec>(v, format!("series.{k}")) {
                return e;
            }
        }
    }
    if let Some(Value::Array(a)) = obj.get("events") {
        for (i, v) in a.iter().enumerate() {
            if let Err(e) = probe::<EventSpec>(v, format!("events[{i}]")) {
                return e;
            }
        }
    }
    invalid("scenario", top)
}

impl ScenarioSpec {
    /// Parses and validates a scenario document.
    pub fn from_json(s: &str) -> Result<ScenarioSpec, SimError> {
        let v: Value = serde_json::from_str(s).map_err(|e| SimError::Json(e.to_string()))?;
        ScenarioSpec::from_value(v)
    }

    /// Parses and validates a scenario from a JSON value.
    pub fn from_value(v: Value) -> Result<ScenarioSpec, SimError> {
        let Some(obj) = v.as_object() else {
            return Err(invalid("scenario", "must be a JSON object"));
        };
        match obj.get("schema") {
            Some(Value::String(s)) if s == SCENARIO_SCHEMA => {}
            Some(Value::String(s)) => return Err(SimError::Schema(s.clone())),
            Some(_) => {
                return Err(invalid(
                    "schema",
                    format!("must be the string \"{SCENARIO_SCHEMA}\""),
                ));
            }
            None => {
                return Err(invalid(
                    "schema",
                    format!("missing (expected \"{SCENARIO_SCHEMA}\")"),
                ));
            }
        }
        let spec: ScenarioSpec = match serde_json::from_value(v.clone()) {
            Ok(s) => s,
            Err(e) => return Err(locate(&v, e)),
        };
        spec.validate()?;
        Ok(spec)
    }

    /// Canonical compact JSON (absent optionals omitted; defaults written out).
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }

    /// Canonical pretty JSON.
    pub fn to_json_pretty(&self) -> String {
        serde_json::to_string_pretty(self).unwrap_or_default()
    }

    /// The plant tier.
    pub fn tier(&self) -> Tier {
        self.fidelity.model.tier()
    }

    /// The plant model configuration (the single source of the coil mode).
    pub fn model_config(&self) -> ModelConfig {
        ModelConfig {
            tier: self.tier(),
            coil: self.fidelity.coil.mode(),
            eddy_lag: self.fidelity.eddy_lag,
            kinematic_heave: self.fidelity.kinematic_heave,
        }
    }

    /// Run length in 20 kHz base ticks (rounded to the nearest tick).
    pub fn duration_ticks(&self) -> u64 {
        Tick::from_seconds_f64(self.duration_s).0
    }

    /// Base ticks between recorded samples (20 000 / rate_hz, validated to be a whole
    /// number of plant steps).
    pub fn record_decimation(&self) -> u64 {
        let d = BASE_TICK_HZ as f64 / self.record.rate_hz;
        if d.is_finite() && d >= 1.0 {
            d.round() as u64
        } else {
            1
        }
    }

    /// Events as (tick, index) in application order: by tick, ties in document order.
    pub fn schedule(&self) -> Vec<(Tick, usize)> {
        let mut v: Vec<(Tick, usize)> = self
            .events
            .iter()
            .enumerate()
            .map(|(i, e)| (Tick::from_seconds_f64(e.t_s()), i))
            .collect();
        v.sort_by_key(|&(t, i)| (t, i));
        v
    }

    /// Checks every range and cross-field rule that does not need the parameter set.
    pub fn validate(&self) -> Result<(), SimError> {
        if self.schema != SCENARIO_SCHEMA {
            return Err(SimError::Schema(self.schema.clone()));
        }
        let name_ok = !self.name.is_empty()
            && self.name.len() <= 64
            && self
                .name
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '.' | '-'));
        if !name_ok {
            return Err(invalid(
                "name",
                "must be 1-64 characters from [A-Za-z0-9_.-]",
            ));
        }
        if let Some(d) = &self.doc
            && d.len() > 4096
        {
            return Err(invalid("doc", "must be at most 4096 bytes"));
        }
        let dur = self.duration_s;
        if !(dur.is_finite() && dur > 0.0 && dur <= MAX_DURATION_S) {
            return Err(invalid(
                "duration_s",
                format!("must be in (0, {MAX_DURATION_S}] (got {dur})"),
            ));
        }
        self.validate_record()?;
        self.validate_initial()?;
        self.validate_plant()?;
        self.validate_series()?;
        for (i, e) in self.events.iter().enumerate() {
            validate_event(i, e, dur)?;
        }
        for (k, w) in &self.metrics.windows {
            let p = || format!("metrics.windows.{k}");
            if !is_ident(k) {
                return Err(invalid(p(), "window names must match [a-z][a-z0-9_]*"));
            }
            if !(w[0].is_finite() && w[1].is_finite() && 0.0 <= w[0] && w[0] < w[1] && w[1] <= dur)
            {
                return Err(invalid(
                    p(),
                    "must be [t0_s, t1_s] with 0 <= t0 < t1 <= duration_s",
                ));
            }
        }
        Ok(())
    }

    fn validate_record(&self) -> Result<(), SimError> {
        let r = self.record.rate_hz;
        check_positive(|| "record.rate_hz".into(), r)?;
        let d = BASE_TICK_HZ as f64 / r;
        let step = self.tier().ticks_per_step();
        let whole = d >= 1.0 && (d - d.round()).abs() <= 1e-9 * d;
        if !whole || !(d.round() as u64).is_multiple_of(step) {
            return Err(invalid(
                "record.rate_hz",
                format!(
                    "{BASE_TICK_HZ} / rate_hz must be a whole multiple of the {} plant step ({step} base ticks)",
                    self.tier().name()
                ),
            ));
        }
        if let ChannelSelect::Names(names) = &self.record.channels {
            if names.is_empty() {
                return Err(invalid(
                    "record.channels",
                    "an explicit list must not be empty",
                ));
            }
            let mut seen = std::collections::BTreeSet::new();
            for (i, n) in names.iter().enumerate() {
                if !is_ident(n) {
                    return Err(invalid(
                        format!("record.channels[{i}]"),
                        "must match [a-z][a-z0-9_]*",
                    ));
                }
                if !seen.insert(n.as_str()) {
                    return Err(invalid(
                        format!("record.channels[{i}]"),
                        format!("duplicate channel \"{n}\""),
                    ));
                }
            }
        }
        Ok(())
    }

    fn validate_initial(&self) -> Result<(), SimError> {
        let i = &self.initial;
        if let Some(x) = i.x_m {
            check_finite(|| "initial.x_m".into(), x)?;
        }
        check_range(|| "initial.v_mps".into(), i.v_mps, -10.0, 10.0)?;
        check_range(|| "initial.payload_kg".into(), i.payload_kg, -1.0e5, 1.0e5)?;
        check_range(|| "initial.temp_c".into(), i.temp_c, -40.0, 150.0)?;
        Ok(())
    }

    fn validate_plant(&self) -> Result<(), SimError> {
        let p = &self.plant;
        p.mismatch().validate()?;
        match &p.rail_profile {
            None => {
                if self.series.contains_key(SERIES_RAIL) {
                    return Err(invalid(
                        "plant.rail_profile",
                        "series.rail is declared, so rail_profile must be \"series:rail\"",
                    ));
                }
            }
            Some(r) if r == RAIL_PROFILE_REF => {
                if !self.series.contains_key(SERIES_RAIL) {
                    return Err(invalid(
                        "plant.rail_profile",
                        "references series.rail, which is not declared",
                    ));
                }
            }
            Some(r) => {
                return Err(invalid(
                    "plant.rail_profile",
                    format!("must be null or \"{RAIL_PROFILE_REF}\" (got \"{r}\")"),
                ));
            }
        }
        let wants_structure = self.fidelity.model == FidelityModel::Structure;
        match (&p.structure, wants_structure) {
            (Some(s), true) => s.to_shear_building().validate()?,
            (None, true) => {
                return Err(invalid(
                    "plant.structure",
                    "fidelity.model \"structure\" needs a shear-building spec",
                ));
            }
            (Some(_), false) => {
                return Err(invalid(
                    "plant.structure",
                    "only valid with fidelity.model \"structure\"",
                ));
            }
            (None, false) => {}
        }
        Ok(())
    }

    fn validate_series(&self) -> Result<(), SimError> {
        for (name, s) in &self.series {
            let p = |f: &str| format!("series.{name}{f}");
            let (axis, allowed): (Axis, &[&str]) = match name.as_str() {
                SERIES_FLOOR | SERIES_SITE => (Axis::Time, &ACCEL_CHANNELS[..]),
                SERIES_RAIL => (Axis::Space, &RAIL_CHANNELS[..]),
                other => {
                    return Err(invalid(
                        p(""),
                        format!("unknown series \"{other}\" (known: floor, site, rail)"),
                    ));
                }
            };
            if s.axis != axis {
                return Err(invalid(
                    p(".axis"),
                    format!(
                        "must be \"{}\"",
                        if axis == Axis::Time { "time" } else { "space" }
                    ),
                ));
            }
            match axis {
                Axis::Time => {
                    let Some(fs) = s.fs_hz else {
                        return Err(invalid(p(".fs_hz"), "required for a time series"));
                    };
                    check_positive(|| p(".fs_hz"), fs)?;
                    check_finite(|| p(".t0_s"), s.t0_s.unwrap_or(0.0))?;
                    if s.dx_m.is_some() || s.x0_m.is_some() {
                        return Err(invalid(p(""), "dx_m/x0_m are only valid on a space series"));
                    }
                }
                Axis::Space => {
                    let Some(dx) = s.dx_m else {
                        return Err(invalid(p(".dx_m"), "required for a space series"));
                    };
                    check_positive(|| p(".dx_m"), dx)?;
                    check_finite(|| p(".x0_m"), s.x0_m.unwrap_or(0.0))?;
                    if s.fs_hz.is_some() || s.t0_s.is_some() {
                        return Err(invalid(p(""), "fs_hz/t0_s are only valid on a time series"));
                    }
                }
            }
            if s.channels.is_empty() {
                return Err(invalid(p(".channels"), "must not be empty"));
            }
            for (i, c) in s.channels.iter().enumerate() {
                if !allowed.contains(&c.as_str()) {
                    return Err(invalid(
                        format!("series.{name}.channels[{i}]"),
                        format!("must be one of {allowed:?}"),
                    ));
                }
                if s.channels[..i].contains(c) {
                    return Err(invalid(
                        format!("series.{name}.channels[{i}]"),
                        format!("duplicate channel \"{c}\""),
                    ));
                }
            }
        }
        Ok(())
    }
}

fn validate_event(i: usize, e: &EventSpec, dur: f64) -> Result<(), SimError> {
    let p = |f: &str| format!("events[{i}].{f}");
    let t = e.t_s();
    if !(t.is_finite() && (0.0..=dur).contains(&t)) {
        return Err(invalid(
            p("t_s"),
            format!("must be in [0, duration_s = {dur}] (got {t})"),
        ));
    }
    let opt_dur = |d: &Option<f64>| -> Result<(), SimError> {
        match d {
            Some(v) => check_positive(|| p("dur_s"), *v),
            None => Ok(()),
        }
    };
    match e {
        EventSpec::Command { cmd, x_m, .. } => match (cmd, x_m) {
            (CommandKind::GoTo, Some(x)) => check_finite(|| p("x_m"), *x)?,
            (CommandKind::GoTo, None) => {
                return Err(invalid(p("x_m"), "required for cmd \"go_to\""));
            }
            (_, Some(_)) => return Err(invalid(p("x_m"), "only valid for cmd \"go_to\"")),
            (_, None) => {}
        },
        EventSpec::Hand {
            t_pts_s,
            f_pts_n,
            impedance,
            ..
        } => {
            if t_pts_s.is_empty() || t_pts_s.len() != f_pts_n.len() {
                return Err(invalid(
                    p("t_pts_s"),
                    "t_pts_s and f_pts_n need equal, non-zero lengths",
                ));
            }
            let mut prev = f64::NEG_INFINITY;
            for (k, &tp) in t_pts_s.iter().enumerate() {
                if !(tp.is_finite() && tp >= 0.0 && tp > prev) {
                    return Err(invalid(
                        format!("events[{i}].t_pts_s[{k}]"),
                        "must be finite, >= 0 and strictly increasing",
                    ));
                }
                prev = tp;
            }
            for (k, &f) in f_pts_n.iter().enumerate() {
                check_range(|| format!("events[{i}].f_pts_n[{k}]"), f, -2000.0, 2000.0)?;
            }
            check_range(|| p("impedance.m_kg"), impedance.m_kg, 1e-3, 20.0)?;
            check_range(
                || p("impedance.b_n_s_per_m"),
                impedance.b_n_s_per_m,
                0.0,
                1.0e3,
            )?;
            check_range(|| p("impedance.k_n_per_m"), impedance.k_n_per_m, 0.0, 1.0e5)?;
        }
        EventSpec::Payload { delta_kg, .. } => {
            check_range(|| p("delta_kg"), *delta_kg, -1.0e5, 1.0e5)?
        }
        EventSpec::Obstacle {
            x_m,
            k_edge_n_per_m,
            dur_s,
            ..
        } => {
            check_finite(|| p("x_m"), *x_m)?;
            if let Some(k) = k_edge_n_per_m {
                check_positive(|| p("k_edge_n_per_m"), *k)?;
            }
            opt_dur(dur_s)?;
        }
        EventSpec::Lean {
            f_n, dur_s, rise_s, ..
        } => {
            check_range(|| p("f_n"), *f_n, -2000.0, 2000.0)?;
            check_positive(|| p("dur_s"), *dur_s)?;
            check_range(|| p("rise_s"), *rise_s, 0.0, *dur_s / 2.0)?;
        }
        EventSpec::SmokeDp {
            dp_pa,
            ramp_s,
            dur_s,
            ..
        } => {
            check_range(|| p("dp_pa"), *dp_pa, -1000.0, 1000.0)?;
            check_range(|| p("ramp_s"), *ramp_s, 0.0, 3600.0)?;
            if let Some(d) = dur_s {
                check_positive(|| p("dur_s"), *d)?;
                if 2.0 * ramp_s > *d {
                    return Err(invalid(p("ramp_s"), "must be at most dur_s / 2"));
                }
            }
        }
        EventSpec::Piston { leak_area_m2, .. } => {
            check_range(|| p("leak_area_m2"), *leak_area_m2, 0.0, 1.0)?
        }
        EventSpec::TempRamp { dt_k, dur_s, .. } => {
            check_range(|| p("dt_k"), *dt_k, -150.0, 150.0)?;
            check_range(|| p("dur_s"), *dur_s, 0.0, MAX_DURATION_S)?;
        }
        EventSpec::PowerLoss { dur_s, .. } | EventSpec::CoreFault { dur_s, .. } => opt_dur(dur_s)?,
        EventSpec::Fault {
            what, ch, jump_m, ..
        } => {
            if *ch > what.max_channel() {
                return Err(invalid(
                    p("ch"),
                    format!("must be <= {} for this fault", what.max_channel()),
                ));
            }
            match (what, jump_m) {
                (SensorFaultKind::EncoderJump, Some(j)) => {
                    if !(j.is_finite() && *j != 0.0 && j.abs() <= 100.0) {
                        return Err(invalid(
                            p("jump_m"),
                            "must be finite, non-zero and |jump| <= 100 m",
                        ));
                    }
                }
                (_, Some(_)) => {
                    return Err(invalid(p("jump_m"), "only valid for what \"encoder_jump\""));
                }
                _ => {}
            }
        }
        EventSpec::NetworkAlert {
            t_s,
            magnitude,
            r_epi_km,
            depth_km,
            origin_t_s,
            mmi,
            ..
        } => {
            check_range(|| p("magnitude"), *magnitude, 0.0, 10.0)?;
            check_range(|| p("r_epi_km"), *r_epi_km, 0.0, 20_000.0)?;
            check_range(|| p("depth_km"), *depth_km, 0.0, 700.0)?;
            check_finite(|| p("origin_t_s"), *origin_t_s)?;
            if origin_t_s > t_s {
                return Err(invalid(
                    p("origin_t_s"),
                    "the origin must not be after the alert's receipt (t_s)",
                ));
            }
            if let Some(m) = mmi {
                check_range(|| p("mmi"), *m, 1.0, 12.0)?;
            }
        }
        EventSpec::ControlLoss { .. }
        | EventSpec::AlertCancel { .. }
        | EventSpec::Reset { .. }
        | EventSpec::Presence { .. }
        | EventSpec::HoldToRun { .. }
        | EventSpec::FireAlarm { .. }
        | EventSpec::Estop { .. } => {}
    }
    Ok(())
}

/// Cross-checks a scenario against a parameter set (does not run
/// `fabrail_params::validate`; `run` and `Session::new` call both):
///
/// * the coil mode (plan §2b item 16: the scenario is the single source, a mismatch is
///   rejected);
/// * `initial.lev = "seated"` needs a captive seat, `initial.locked = true` a parking
///   lock;
/// * the initial position lies within [x_closed, x_closed + travel];
/// * the truth mass stays > 0 through every payload event.
pub fn check_compat(p: &SystemParams, spec: &ScenarioSpec) -> Result<(), SimError> {
    let voltage = spec.fidelity.coil.is_voltage();
    if voltage != p.control.rates.coil_mode_voltage {
        return Err(SimError::CoilModeMismatch {
            scenario_voltage: voltage,
            params_voltage: p.control.rates.coil_mode_voltage,
        });
    }
    let init = spec.initial.resolve(p);
    if init.lev == InitLev::Seated && p.design.landing.kind != LandingKind::Seat {
        return Err(invalid(
            "initial.lev",
            "\"seated\" needs a captive seat (design.landing.kind = \"seat\")",
        ));
    }
    if init.locked && !p.design.landing.parking_lock {
        return Err(invalid(
            "initial.locked",
            "needs a parking lock (design.landing.parking_lock)",
        ));
    }
    let x0 = p.design.rail.x_closed_m;
    let x1 = x0 + p.design.rail.travel_m;
    let tol = 1e-9;
    if !(init.x_m >= x0 - tol && init.x_m <= x1 + tol) {
        return Err(invalid(
            "initial.x_m",
            format!(
                "must lie within the travel [{x0}, {x1}] m (got {})",
                init.x_m
            ),
        ));
    }
    let mut m = spec.plant.mass_scale * p.design.carriage.m_star_kg + init.payload_kg;
    if m.is_nan() || m <= 0.0 {
        return Err(invalid(
            "initial.payload_kg",
            format!(
                "the truth mass plant.mass_scale * m_star + payload_kg must be > 0 (got {m} kg)"
            ),
        ));
    }
    for (_, i) in spec.schedule() {
        if let Some(EventSpec::Payload { delta_kg, .. }) = spec.events.get(i) {
            m += delta_kg;
            if m.is_nan() || m <= 0.0 {
                return Err(invalid(
                    format!("events[{i}].delta_kg"),
                    format!("the truth mass would drop to {m} kg"),
                ));
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------------
// Series data
// ---------------------------------------------------------------------------------

/// Decodes channel-major little-endian f64 bytes into `n_channels` equal-length
/// channels (all samples of channel 0 first). The byte length must be a multiple of
/// 8·n_channels; every value must be finite.
pub fn decode_f64_le(bytes: &[u8], n_channels: usize) -> Result<Vec<Vec<f64>>, String> {
    if n_channels == 0 {
        return Err("needs at least one channel".into());
    }
    let row = 8 * n_channels;
    if !bytes.len().is_multiple_of(row) {
        return Err(format!(
            "{} bytes is not a multiple of 8 x {n_channels} channels (channel-major LE f64)",
            bytes.len()
        ));
    }
    let n = bytes.len() / row;
    let mut out = Vec::with_capacity(n_channels);
    for c in 0..n_channels {
        let chunk = bytes.get(c * n * 8..(c + 1) * n * 8).unwrap_or(&[]);
        let mut ch = Vec::with_capacity(n);
        for (k, b) in chunk.chunks_exact(8).enumerate() {
            let arr = <[u8; 8]>::try_from(b).map_err(|_| "internal: short chunk".to_string())?;
            let v = f64::from_le_bytes(arr);
            if !v.is_finite() {
                return Err(format!("channel {c} sample {k} is not finite"));
            }
            ch.push(v);
        }
        out.push(ch);
    }
    Ok(out)
}

/// Encodes channels as channel-major little-endian f64 bytes (the inverse of
/// [`decode_f64_le`] for equal-length channels).
pub fn encode_f64_le(channels: &[Vec<f64>]) -> Vec<u8> {
    let mut out = Vec::with_capacity(channels.iter().map(|c| 8 * c.len()).sum());
    for ch in channels {
        for v in ch {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    out
}

/// A decoded series on a uniform grid.
#[derive(Clone, Debug, PartialEq)]
pub struct Series {
    /// Coordinate axis.
    pub axis: Axis,
    /// Coordinate of sample 0 (s or m).
    pub start: f64,
    /// Grid step (1/fs_hz in s, or dx_m in m).
    pub step: f64,
    /// Channel names.
    pub channels: Vec<String>,
    /// Channel data, `data[channel][sample]`.
    pub data: Vec<Vec<f64>>,
}

impl Series {
    /// Decodes `bytes` for the declaration `spec` (needs ≥ 2 samples per channel).
    pub fn decode(spec: &SeriesSpec, bytes: &[u8]) -> Result<Series, String> {
        let data = decode_f64_le(bytes, spec.channels.len())?;
        let n = data.first().map_or(0, Vec::len);
        if n < 2 {
            return Err(format!("needs at least 2 samples per channel (got {n})"));
        }
        let (start, step) = spec.grid();
        Ok(Series {
            axis: spec.axis,
            start,
            step,
            channels: spec.channels.clone(),
            data,
        })
    }

    /// The bytes this series was decoded from.
    pub fn encode(&self) -> Vec<u8> {
        encode_f64_le(&self.data)
    }

    /// Samples per channel.
    pub fn n_samples(&self) -> usize {
        self.data.first().map_or(0, Vec::len)
    }

    /// Coordinate of the last sample.
    pub fn end(&self) -> f64 {
        self.start + self.step * self.n_samples().saturating_sub(1) as f64
    }

    /// Index of channel `name`.
    pub fn channel_index(&self, name: &str) -> Option<usize> {
        self.channels.iter().position(|c| c == name)
    }

    /// Samples of channel `name`.
    pub fn channel(&self, name: &str) -> Option<&[f64]> {
        self.channel_index(name)
            .and_then(|i| self.data.get(i))
            .map(Vec::as_slice)
    }

    /// Linear interpolation of channel `ch` at `coord`. Outside the grid a time series
    /// reads 0 (quiet) and a space series holds its edge value; an unknown channel or a
    /// non-finite coordinate reads 0.
    pub fn sample(&self, ch: usize, coord: f64) -> f64 {
        let Some(d) = self.data.get(ch) else {
            return 0.0;
        };
        let n = d.len();
        if n == 0 || !coord.is_finite() || self.step.is_nan() || self.step <= 0.0 {
            return 0.0;
        }
        let u = (coord - self.start) / self.step;
        let last = (n - 1) as f64;
        if u < 0.0 || u > last {
            return match self.axis {
                Axis::Time => 0.0,
                Axis::Space => {
                    if u < 0.0 {
                        d[0]
                    } else {
                        d[n - 1]
                    }
                }
            };
        }
        let k = (u.floor() as usize).min(n - 1);
        let frac = u - k as f64;
        let a = d[k];
        match d.get(k + 1) {
            Some(&b) if frac > 0.0 => a + (b - a) * frac,
            _ => a,
        }
    }

    /// Samples the given role channels (missing channels read 0).
    fn sample_named<const N: usize>(&self, names: [&str; N], coord: f64) -> [f64; N] {
        let mut out = [0.0; N];
        for (o, name) in out.iter_mut().zip(names) {
            if let Some(i) = self.channel_index(name) {
                *o = self.sample(i, coord);
            }
        }
        out
    }
}

/// A validated scenario with its decoded series.
#[derive(Clone, Debug, PartialEq)]
pub struct Scenario {
    /// The document.
    pub spec: ScenarioSpec,
    /// Decoded series by name.
    pub series: BTreeMap<String, Series>,
}

impl Scenario {
    /// Parses the document and decodes `series` (name → channel-major LE f64 bytes).
    /// Every declared series needs bytes and every byte buffer a declaration.
    pub fn from_json(
        spec_json: &str,
        series: &BTreeMap<String, Vec<u8>>,
    ) -> Result<Scenario, SimError> {
        Scenario::from_spec(ScenarioSpec::from_json(spec_json)?, series)
    }

    /// Validates `spec` and decodes `series`.
    pub fn from_spec(
        spec: ScenarioSpec,
        series: &BTreeMap<String, Vec<u8>>,
    ) -> Result<Scenario, SimError> {
        spec.validate()?;
        for name in series.keys() {
            if !spec.series.contains_key(name) {
                return Err(SimError::Series {
                    name: name.clone(),
                    reason: "bytes were given but the scenario does not declare this series".into(),
                });
            }
        }
        let mut decoded = BTreeMap::new();
        for (name, s) in &spec.series {
            let Some(bytes) = series.get(name) else {
                return Err(SimError::Series {
                    name: name.clone(),
                    reason: "declared but no bytes were given".into(),
                });
            };
            let d = Series::decode(s, bytes).map_err(|reason| SimError::Series {
                name: name.clone(),
                reason,
            })?;
            decoded.insert(name.clone(), d);
        }
        Ok(Scenario {
            spec,
            series: decoded,
        })
    }

    /// The canonical document JSON (series bytes are not included).
    pub fn to_json(&self) -> String {
        self.spec.to_json()
    }

    /// The series as byte buffers (round trip of `from_json`'s input).
    pub fn series_bytes(&self) -> BTreeMap<String, Vec<u8>> {
        self.series
            .iter()
            .map(|(k, s)| (k.clone(), s.encode()))
            .collect()
    }

    /// Floor acceleration (x, y, z) at `t_s`, m/s² (0 without a floor series).
    pub fn floor_mps2(&self, t_s: f64) -> [f64; 3] {
        self.series
            .get(SERIES_FLOOR)
            .map_or([0.0; 3], |s| s.sample_named(ACCEL_CHANNELS, t_s))
    }

    /// Site noise (x, y, z) at `t_s` added to the structure accelerometer, m/s².
    pub fn site_mps2(&self, t_s: f64) -> [f64; 3] {
        self.series
            .get(SERIES_SITE)
            .map_or([0.0; 3], |s| s.sample_named(ACCEL_CHANNELS, t_s))
    }

    /// The rail profile referenced by `plant.rail_profile`, if any.
    pub fn rail_profile(&self) -> Result<Option<RailProfile>, SimError> {
        if self.spec.plant.rail_profile.is_none() {
            return Ok(None);
        }
        let Some(s) = self.series.get(SERIES_RAIL) else {
            return Ok(None);
        };
        let n = s.n_samples();
        let z = s.channel("z").map_or_else(|| vec![0.0; n], <[f64]>::to_vec);
        let y = s.channel("y").map_or_else(|| vec![0.0; n], <[f64]>::to_vec);
        Ok(Some(RailProfile::new(s.start, s.step, z, y)?))
    }

    /// Plant truth parameters for parameter set `p` (mismatch, rail profile, building).
    pub fn plant_params(&self, p: &SystemParams) -> Result<PlantParams, SimError> {
        let env = Environment {
            rail: self.rail_profile()?,
            structure: self
                .spec
                .plant
                .structure
                .as_ref()
                .map(StructureSpec::to_shear_building),
        };
        // The run's control tuning sets the plant's closed-loop current lag and the
        // Travel1d kinematic lift/land (A6a `with_control`; W1.5).
        Ok(
            PlantParams::from_design(&p.design, &self.spec.plant.mismatch(), &env)
                .with_control(&p.control),
        )
    }
}

// ---------------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// A document exercising every block and every event kind.
    const FULL: &str = r#"{
      "schema": "fabrail.scenario/1", "name": "door_everything", "doc": "all kinds",
      "duration_s": 6, "noise_seed": 123,
      "fidelity": {"model": "carriage6", "coil": "voltage", "eddy_lag": true, "kinematic_heave": true},
      "record": {"rate_hz": 1000, "channels": ["x_m", "v_mps"]},
      "initial": {"lev": "levitated", "x_m": 0.1, "v_mps": 0, "payload_kg": 0, "temp_c": 20},
      "plant": {"mass_scale": 1.02, "br_scale": 0.98, "leak_sigma": null, "c_rail_n_s_per_m": null,
                "f_hyst_n": 1.0, "leak_area_m2": null, "rail_profile": "series:rail", "structure": null},
      "series": {"floor": {"axis": "time", "fs_hz": 200, "t0_s": 0, "channels": ["ax", "ay", "az"]},
                 "site": {"axis": "time", "fs_hz": 500, "channels": ["az"]},
                 "rail": {"axis": "space", "dx_m": 0.001, "x0_m": 0, "channels": ["z", "y"]}},
      "events": [
        {"kind": "command", "t_s": 0.5, "cmd": "open"},
        {"kind": "command", "t_s": 0.6, "cmd": "go_to", "x_m": 0.4},
        {"kind": "hand", "t_s": 1, "t_pts_s": [0, 0.1, 0.6], "f_pts_n": [0, 8, 0],
         "impedance": {"m_kg": 1.5, "b_n_s_per_m": 10, "k_n_per_m": 500}, "contact": "push"},
        {"kind": "payload", "t_s": 1.2, "delta_kg": 10},
        {"kind": "obstacle", "t_s": 1.3, "body": "chest", "x_m": 0.9, "config": "clamped", "k_edge_n_per_m": 10000},
        {"kind": "lean", "t_s": 1.4, "f_n": 15, "dur_s": 3, "rise_s": 1.0},
        {"kind": "smoke_dp", "t_s": 1.5, "dp_pa": 25, "ramp_s": 2, "dur_s": 4},
        {"kind": "piston", "t_s": 1.6, "leak_area_m2": 0.004},
        {"kind": "temp_ramp", "t_s": 1.7, "dt_k": 40, "dur_s": 600},
        {"kind": "power_loss", "t_s": 1.8},
        {"kind": "control_loss", "t_s": 1.9, "dir": "down"},
        {"kind": "fault", "t_s": 2.0, "what": "gap_stuck", "ch": 3},
        {"kind": "fault", "t_s": 2.0, "what": "encoder_jump", "jump_m": 0.002},
        {"kind": "network_alert", "t_s": 2.1, "magnitude": 7, "r_epi_km": 20, "depth_km": 10, "origin_t_s": -2},
        {"kind": "alert_cancel", "t_s": 2.2},
        {"kind": "reset", "t_s": 2.3, "cause": "watchdog"},
        {"kind": "core_fault", "t_s": 2.4, "what": "iq_gain_x3"},
        {"kind": "presence", "t_s": 2.5, "active": true},
        {"kind": "hold_to_run", "t_s": 2.6, "on": true},
        {"kind": "fire_alarm", "t_s": 2.7, "active": false},
        {"kind": "estop", "t_s": 0.0, "active": false}
      ],
      "metrics": {"windows": {"push": [0.9, 2.0]}}
    }"#;

    const MIN: &str = r#"{"schema": "fabrail.scenario/1", "name": "m", "duration_s": 1, "noise_seed": 1,
                          "fidelity": {"model": "carriage6", "coil": "lag"}}"#;

    fn bytes_for(spec: &ScenarioSpec) -> BTreeMap<String, Vec<u8>> {
        spec.series
            .iter()
            .map(|(k, s)| {
                let chans: Vec<Vec<f64>> = (0..s.channels.len())
                    .map(|c| (0..5).map(|k| (c * 10 + k) as f64 * 0.01).collect())
                    .collect();
                (k.clone(), encode_f64_le(&chans))
            })
            .collect()
    }

    fn err(s: &str) -> String {
        ScenarioSpec::from_json(s)
            .expect_err("must be rejected")
            .to_string()
    }

    fn with(base: &str, find: &str, replace: &str) -> String {
        assert!(base.contains(find), "fixture drift: {find}");
        base.replacen(find, replace, 1)
    }

    #[test]
    fn full_document_parses_and_round_trips() {
        let spec = ScenarioSpec::from_json(FULL).expect("valid");
        assert_eq!(spec.events.len(), 21);
        let kinds: std::collections::BTreeSet<_> =
            spec.events.iter().map(EventSpec::kind).collect();
        assert_eq!(
            kinds.len(),
            EventSpec::KINDS.len(),
            "every event kind is exercised"
        );
        let back = ScenarioSpec::from_json(&spec.to_json()).expect("canonical JSON reparses");
        assert_eq!(back, spec);
        let pretty = ScenarioSpec::from_json(&spec.to_json_pretty()).expect("pretty reparses");
        assert_eq!(pretty, spec);
        assert_eq!(spec.model_config().coil, CoilMode::Voltage);
        assert_eq!(spec.tier(), Tier::Carriage6);
        assert_eq!(spec.record_decimation(), 20);
        assert_eq!(spec.duration_ticks(), 120_000);
    }

    #[test]
    fn defaults_fill_optional_blocks() {
        let spec = ScenarioSpec::from_json(MIN).expect("valid");
        assert_eq!(spec.record, RecordSpec::default());
        assert_eq!(spec.initial, InitialSpec::default());
        assert_eq!(spec.plant, PlantSpec::default());
        assert!(spec.fidelity.eddy_lag && spec.fidelity.kinematic_heave);
        assert!(spec.events.is_empty() && spec.series.is_empty());
        // Canonical output writes defaults and omits absent optionals.
        let j = spec.to_json();
        assert!(
            j.contains("\"rate_hz\":1000.0") && j.contains("\"channels\":\"default\""),
            "{j}"
        );
        assert!(!j.contains("null"), "{j}");
    }

    #[test]
    fn event_defaults() {
        let spec = ScenarioSpec::from_json(FULL).expect("valid");
        match &spec.events[5] {
            EventSpec::Lean { rise_s, .. } => assert_eq!(*rise_s, 1.0),
            e => panic!("unexpected {e:?}"),
        }
        let s = with(
            MIN,
            "\"lag\"}",
            "\"lag\"}, \"events\": [{\"kind\": \"smoke_dp\", \"t_s\": 0, \"dp_pa\": 20}, {\"kind\": \"hand\", \"t_s\": 0, \"t_pts_s\": [0], \"f_pts_n\": [5]}, {\"kind\": \"presence\", \"t_s\": 0, \"active\": true}]",
        );
        let spec = ScenarioSpec::from_json(&s).expect("valid");
        assert!(
            matches!(spec.events[0], EventSpec::SmokeDp { ramp_s, dur_s: None, .. } if ramp_s == 2.0)
        );
        assert!(
            matches!(&spec.events[1], EventSpec::Hand { impedance, contact: HandContact::Grip, .. } if *impedance == Impedance::default())
        );
        assert!(matches!(
            spec.events[2],
            EventSpec::Presence { ok: true, .. }
        ));
    }

    #[test]
    fn schema_and_syntax_errors_are_clear() {
        assert!(matches!(
            ScenarioSpec::from_json("{"),
            Err(SimError::Json(_))
        ));
        assert!(matches!(
            ScenarioSpec::from_json(&with(MIN, "fabrail.scenario/1", "fabrail.scenario/2")),
            Err(SimError::Schema(s)) if s == "fabrail.scenario/2"
        ));
        assert!(
            err(&with(MIN, "\"schema\": \"fabrail.scenario/1\",", ""))
                .starts_with("schema: missing")
        );
        assert!(err("[1, 2]").starts_with("scenario: must be a JSON object"));
    }

    #[test]
    fn unknown_and_missing_fields_are_located() {
        let e = err(&with(
            MIN,
            "\"noise_seed\": 1",
            "\"noise_seed\": 1, \"bogus\": 2",
        ));
        assert!(e.starts_with("scenario:") && e.contains("bogus"), "{e}");
        let e = err(&with(MIN, "\"duration_s\": 1,", ""));
        assert!(e.contains("duration_s"), "{e}");
        let e = err(&with(MIN, "\"coil\": \"lag\"", "\"coil\": \"pwm\""));
        assert!(e.starts_with("fidelity:") && e.contains("pwm"), "{e}");
        let e = err(&with(
            FULL,
            "{\"kind\": \"piston\", \"t_s\": 1.6,",
            "{\"kind\": \"piston\",",
        ));
        assert!(e.starts_with("events[7]:") && e.contains("t_s"), "{e}");
        let e = err(&with(
            FULL,
            "\"kind\": \"piston\"",
            "\"kind\": \"piston_pulse\"",
        ));
        assert!(
            e.starts_with("events[7]:") && e.contains("unknown variant"),
            "{e}"
        );
        let e = err(&with(
            FULL,
            "\"leak_area_m2\": 0.004",
            "\"leak_area\": 0.004",
        ));
        assert!(
            e.starts_with("events[7]:") && e.contains("leak_area"),
            "{e}"
        );
        let e = err(&with(FULL, "\"axis\": \"space\"", "\"axis\": \"depth\""));
        assert!(e.starts_with("series.rail:"), "{e}");
        let e = err(&with(
            FULL,
            "\"channels\": [\"x_m\", \"v_mps\"]",
            "\"channels\": \"everything\"",
        ));
        assert!(e.starts_with("record:") && e.contains("kpis_only"), "{e}");
        let e = err(&with(FULL, "\"m_kg\": 1.5", "\"mass\": 1.5"));
        assert!(e.starts_with("events[2]:") && e.contains("mass"), "{e}");
    }

    #[test]
    fn range_and_cross_field_rules() {
        let cases: &[(&str, &str, &str)] = &[
            ("\"duration_s\": 6", "\"duration_s\": 0", "duration_s:"),
            ("\"duration_s\": 6", "\"duration_s\": 1e9", "duration_s:"),
            (
                "\"name\": \"door_everything\"",
                "\"name\": \"bad name\"",
                "name:",
            ),
            ("\"rate_hz\": 1000", "\"rate_hz\": 3000", "record.rate_hz:"),
            (
                "\"x_m\", \"v_mps\"",
                "\"x_m\", \"x_m\"",
                "record.channels[1]:",
            ),
            (
                "\"x_m\", \"v_mps\"",
                "\"X\", \"v_mps\"",
                "record.channels[0]:",
            ),
            ("\"temp_c\": 20", "\"temp_c\": 400", "initial.temp_c:"),
            (
                "\"mass_scale\": 1.02",
                "\"mass_scale\": 0",
                "plant.mass_scale:",
            ),
            (
                "\"leak_sigma\": null",
                "\"leak_sigma\": 1.5",
                "plant.leak_sigma:",
            ),
            (
                "\"rail_profile\": \"series:rail\"",
                "\"rail_profile\": \"series:floor\"",
                "plant.rail_profile:",
            ),
            (
                "\"rail_profile\": \"series:rail\"",
                "\"rail_profile\": null",
                "plant.rail_profile:",
            ),
            (
                "\"structure\": null",
                "\"structure\": {\"storey_mass_kg\": [1], \"storey_k_n_per_m\": [1], \"device_storey\": 1}",
                "plant.structure:",
            ),
            ("\"fs_hz\": 200", "\"fs_hz\": -200", "series.floor.fs_hz:"),
            (
                "\"fs_hz\": 200, \"t0_s\": 0",
                "\"t0_s\": 0",
                "series.floor.fs_hz:",
            ),
            (
                "\"dx_m\": 0.001",
                "\"dx_m\": 0.001, \"fs_hz\": 10",
                "series.rail:",
            ),
            (
                "\"axis\": \"space\"",
                "\"axis\": \"time\"",
                "series.rail.axis:",
            ),
            (
                "\"channels\": [\"z\", \"y\"]",
                "\"channels\": [\"z\", \"x\"]",
                "series.rail.channels[1]:",
            ),
            (
                "\"channels\": [\"az\"]",
                "\"channels\": []",
                "series.site.channels:",
            ),
            ("\"site\": {", "\"quake\": {", "series.quake:"),
            (
                "\"t_s\": 0.5, \"cmd\": \"open\"",
                "\"t_s\": 7, \"cmd\": \"open\"",
                "events[0].t_s:",
            ),
            (
                "\"cmd\": \"go_to\", \"x_m\": 0.4",
                "\"cmd\": \"go_to\"",
                "events[1].x_m:",
            ),
            (
                "\"cmd\": \"open\"",
                "\"cmd\": \"open\", \"x_m\": 1",
                "events[0].x_m:",
            ),
            (
                "\"t_pts_s\": [0, 0.1, 0.6]",
                "\"t_pts_s\": [0, 0.6, 0.1]",
                "events[2].t_pts_s[2]:",
            ),
            (
                "\"f_pts_n\": [0, 8, 0]",
                "\"f_pts_n\": [0, 8]",
                "events[2].t_pts_s:",
            ),
            ("\"m_kg\": 1.5", "\"m_kg\": 0", "events[2].impedance.m_kg:"),
            (
                "\"dur_s\": 3, \"rise_s\": 1.0",
                "\"dur_s\": 1.5, \"rise_s\": 1.0",
                "events[5].rise_s:",
            ),
            (
                "\"ramp_s\": 2, \"dur_s\": 4",
                "\"ramp_s\": 2, \"dur_s\": 3",
                "events[6].ramp_s:",
            ),
            (
                "\"k_edge_n_per_m\": 10000",
                "\"k_edge_n_per_m\": 0",
                "events[4].k_edge_n_per_m:",
            ),
            (
                "\"what\": \"gap_stuck\", \"ch\": 3",
                "\"what\": \"gap_stuck\", \"ch\": 8",
                "events[11].ch:",
            ),
            (
                "\"what\": \"gap_stuck\", \"ch\": 3",
                "\"what\": \"gap_stuck\", \"jump_m\": 1",
                "events[11].jump_m:",
            ),
            ("\"jump_m\": 0.002", "\"jump_m\": 0", "events[12].jump_m:"),
            (
                "\"magnitude\": 7",
                "\"magnitude\": 11",
                "events[13].magnitude:",
            ),
            (
                "\"origin_t_s\": -2",
                "\"origin_t_s\": 3",
                "events[13].origin_t_s:",
            ),
            (
                "\"windows\": {\"push\": [0.9, 2.0]}",
                "\"windows\": {\"push\": [2.0, 0.9]}",
                "metrics.windows.push:",
            ),
        ];
        for (find, replace, prefix) in cases {
            let e = err(&with(FULL, find, replace));
            assert!(e.starts_with(prefix), "{replace} → {e}");
        }
    }

    #[test]
    fn travel1d_record_rate_must_fit_the_1ms_step() {
        let s = with(MIN, "\"carriage6\"", "\"travel1d\"");
        assert!(ScenarioSpec::from_json(&s).is_ok(), "1000 Hz fits 1 ms");
        let s = with(&s, "\"lag\"}", "\"lag\"}, \"record\": {\"rate_hz\": 2000}");
        assert!(err(&s).starts_with("record.rate_hz:"));
    }

    #[test]
    fn structure_tier_needs_a_building() {
        let s = with(MIN, "\"carriage6\"", "\"structure\"");
        assert!(err(&s).starts_with("plant.structure:"));
        let s = with(
            &s,
            "\"lag\"}",
            "\"lag\"}, \"record\": {\"rate_hz\": 100}, \"plant\": {\"structure\": {\"storey_mass_kg\": [2e5, 2e5], \"storey_k_n_per_m\": [8e7, 8e7], \"device_storey\": 2}}",
        );
        let spec = ScenarioSpec::from_json(&s).expect("valid");
        assert_eq!(
            spec.plant.structure.as_ref().map(|b| b.rayleigh_zeta),
            Some([0.02, 0.02])
        );
        let bad = with(&s, "\"device_storey\": 2", "\"device_storey\": 3");
        assert!(err(&bad).starts_with("plant.structure.device_storey:"));
    }

    // req: FAB-SW-015
    #[test]
    fn series_bytes_channel_major_le() {
        let spec = ScenarioSpec::from_json(FULL).expect("valid");
        let bytes = bytes_for(&spec);
        let sc = Scenario::from_spec(spec.clone(), &bytes).expect("decodes");
        let floor = &sc.series[SERIES_FLOOR];
        assert_eq!(floor.n_samples(), 5);
        let ay = floor.channel("ay").expect("ay");
        for (k, v) in ay.iter().enumerate() {
            assert!(
                (v - (0.10 + 0.01 * k as f64)).abs() < 1e-15,
                "ay[{k}] = {v}"
            );
        }
        assert_eq!(sc.series_bytes(), bytes, "round trip");
        // Floor sampling: 200 Hz grid, linear in between, 0 after the end.
        let f = sc.floor_mps2(0.0075);
        assert!(
            (f[0] - 0.015).abs() < 1e-12 && (f[1] - 0.115).abs() < 1e-12,
            "{f:?}"
        );
        assert_eq!(sc.floor_mps2(1.0), [0.0; 3]);
        // The site series only carries az.
        let s = sc.site_mps2(0.002);
        assert_eq!((s[0], s[1]), (0.0, 0.0));
        assert!((s[2] - 0.01).abs() < 1e-12);
        // Rail: space axis, edge-held.
        let rail = sc.rail_profile().expect("ok").expect("present");
        assert_eq!(rail.len(), 5);
        assert!((sc.series[SERIES_RAIL].sample(0, 10.0) - 0.04).abs() < 1e-15);
        assert_eq!(sc.series[SERIES_RAIL].sample(0, -1.0), 0.0);
    }

    #[test]
    fn series_byte_errors() {
        let spec = ScenarioSpec::from_json(FULL).expect("valid");
        let good = bytes_for(&spec);
        let mut b = good.clone();
        b.remove("site");
        assert!(
            matches!(Scenario::from_spec(spec.clone(), &b), Err(SimError::Series { name, .. }) if name == "site")
        );
        let mut b = good.clone();
        b.insert("extra".into(), vec![0; 16]);
        assert!(
            matches!(Scenario::from_spec(spec.clone(), &b), Err(SimError::Series { name, .. }) if name == "extra")
        );
        let mut b = good.clone();
        b.insert("floor".into(), vec![0; 23]);
        let e = Scenario::from_spec(spec.clone(), &b)
            .expect_err("odd length")
            .to_string();
        assert!(
            e.starts_with("series.floor:") && e.contains("multiple"),
            "{e}"
        );
        let mut b = good.clone();
        b.insert("site".into(), encode_f64_le(&[vec![1.0]]));
        assert!(
            Scenario::from_spec(spec.clone(), &b).is_err(),
            "one sample is not enough"
        );
        let mut b = good;
        b.insert("site".into(), encode_f64_le(&[vec![1.0, f64::NAN]]));
        let e = Scenario::from_spec(spec, &b).expect_err("NaN").to_string();
        assert!(e.contains("not finite"), "{e}");
        assert!(decode_f64_le(&[], 0).is_err());
    }

    #[test]
    fn schedule_is_stable_by_time() {
        let spec = ScenarioSpec::from_json(FULL).expect("valid");
        let sched = spec.schedule();
        assert_eq!(sched.len(), spec.events.len());
        assert_eq!(sched[0].1, 20, "the t = 0 estop comes first");
        assert!(sched.windows(2).all(|w| w[0].0 <= w[1].0));
        // The two t = 2.0 faults keep document order.
        let pos = |i| sched.iter().position(|&(_, j)| j == i);
        assert!(pos(11) < pos(12));
    }

    #[test]
    fn event_conversions() {
        let spec = ScenarioSpec::from_json(FULL).expect("valid");
        assert_eq!(spec.events[0].user_command(), Some(UserCommand::Open));
        assert_eq!(
            spec.events[1].user_command(),
            Some(UserCommand::GoTo(PosNm(400_000_000)))
        );
        let a = spec.events[13].network_alert(1).expect("alert");
        assert_eq!((a.id, a.received), (1, Tick(42_000)));
        assert_eq!(a.origin_offset_s, (-2.0_f64 - 2.1) as f32);
        assert_eq!((a.magnitude, a.r_epi_km, a.depth_km), (7.0, 20.0, 10.0));
        assert!(spec.events[14].network_alert(1).is_none());
        assert_eq!(
            ResetCause::from(RebootCause::Watchdog),
            ResetCause::IndependentWatchdog
        );
        assert_eq!(ObstacleBody::from(ObstacleBodyKind::Leg), ObstacleBody::Leg);
    }

    fn door_params(voltage: bool) -> SystemParams {
        let mut p = fabrail_params::presets::door();
        p.control.rates.coil_mode_voltage = voltage;
        p.design.carriage.m_star_kg = 100.0;
        p.design.rail.x_closed_m = 0.0;
        p.design.rail.travel_m = 1.0;
        p.design.rest_policy = RestPolicy::Hover;
        p
    }

    // req: FAB-SW-015
    #[test]
    fn rejects_coil_mode_mismatch() {
        let spec = ScenarioSpec::from_json(FULL).expect("valid"); // voltage
        assert!(check_compat(&door_params(true), &spec).is_ok());
        let e = check_compat(&door_params(false), &spec).expect_err("mismatch");
        assert_eq!(
            e,
            SimError::CoilModeMismatch {
                scenario_voltage: true,
                params_voltage: false
            }
        );
        assert!(e.to_string().starts_with("fidelity.coil:"));
    }

    #[test]
    fn compat_checks_rest_travel_and_mass() {
        let p = door_params(false);
        let base = ScenarioSpec::from_json(MIN).expect("valid");
        assert!(check_compat(&p, &base).is_ok());
        let mut s = base.clone();
        s.initial.lev = Some(InitLevSpec::Seated);
        assert!(
            check_compat(&p, &s)
                .expect_err("no seat")
                .to_string()
                .starts_with("initial.lev:")
        );
        let mut s = base.clone();
        s.initial.locked = Some(true);
        assert!(
            check_compat(&p, &s)
                .expect_err("no lock")
                .to_string()
                .starts_with("initial.locked:")
        );
        let mut s = base.clone();
        s.initial.x_m = Some(1.5);
        assert!(
            check_compat(&p, &s)
                .expect_err("beyond travel")
                .to_string()
                .starts_with("initial.x_m:")
        );
        let mut s = base.clone();
        s.initial.payload_kg = -100.0;
        assert!(
            check_compat(&p, &s)
                .expect_err("zero mass")
                .to_string()
                .starts_with("initial.payload_kg:")
        );
        let mut s = base;
        s.events.push(EventSpec::Payload {
            t_s: 0.5,
            delta_kg: -120.0,
        });
        assert!(
            check_compat(&p, &s)
                .expect_err("negative mass")
                .to_string()
                .starts_with("events[0].delta_kg:")
        );
    }

    #[test]
    fn initial_resolves_from_rest_policy() {
        let mut p = door_params(false);
        let spec = InitialSpec::default();
        p.design.rest_policy = RestPolicy::Hover;
        assert_eq!(spec.resolve(&p).lev, InitLev::Levitated);
        p.design.rest_policy = RestPolicy::Seat;
        assert_eq!(spec.resolve(&p).lev, InitLev::Seated);
        p.design.rest_policy = RestPolicy::LandLock;
        let r = spec.resolve(&p);
        assert_eq!((r.lev, r.locked), (InitLev::Landed, true));
        p.design.rail.x_closed_m = 0.25;
        assert_eq!(spec.resolve(&p).x_m, 0.25);
    }

    #[test]
    fn plant_params_carry_mismatch_and_environment() {
        let spec = ScenarioSpec::from_json(FULL).expect("valid");
        let sc = Scenario::from_spec(spec.clone(), &bytes_for(&spec)).expect("decodes");
        let pp = sc.plant_params(&door_params(true)).expect("plant params");
        assert!((pp.m_body_kg - 102.0).abs() < 1e-9);
        assert_eq!(pp.f_hyst_n, 1.0);
        assert!(pp.env.rail.is_some() && pp.env.structure.is_none());
    }

    /// The JSON schema shipped for Python (`scenario-1.schema.json`) lists exactly the
    /// Rust event kinds and top-level keys (drift guard).
    #[test]
    fn json_schema_matches_rust_model() {
        let schema: Value = serde_json::from_str(include_str!(
            "../../../python/fabrail/schemas/scenario-1.schema.json"
        ))
        .expect("schema is JSON");
        assert_eq!(schema["properties"]["schema"]["const"], SCENARIO_SCHEMA);
        let top: std::collections::BTreeSet<&str> = schema["properties"]
            .as_object()
            .expect("properties")
            .keys()
            .map(String::as_str)
            .collect();
        let rust_top: std::collections::BTreeSet<&str> = [
            "schema",
            "name",
            "doc",
            "duration_s",
            "noise_seed",
            "fidelity",
            "record",
            "initial",
            "plant",
            "series",
            "events",
            "metrics",
        ]
        .into_iter()
        .collect();
        assert_eq!(top, rust_top);
        let kinds: std::collections::BTreeSet<String> = schema["$defs"]["event"]["oneOf"]
            .as_array()
            .expect("event oneOf")
            .iter()
            .filter_map(|r| r["$ref"].as_str())
            .filter_map(|r| {
                let key = r.rsplit('/').next()?;
                schema["$defs"][key]["properties"]["kind"]["const"]
                    .as_str()
                    .map(str::to_string)
            })
            .collect();
        let rust: std::collections::BTreeSet<String> =
            EventSpec::KINDS.iter().map(|s| s.to_string()).collect();
        assert_eq!(kinds, rust);
    }
}
