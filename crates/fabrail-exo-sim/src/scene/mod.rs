//! Multi-device building scene (plan §4b): building models (fixed-base shear building,
//! conventional isolation, HAVEN-F reduced platform, SENTINEL penthouse), SceneSpec
//! (`fabrail.scene/1`, egress declared per device), BuildingConfig, one-way coupling to
//! devices stepped on `fabrail_sim::Session` in 200-tick chunks at 100 Hz sync, in fixed
//! registry order. Owner: E2 (module list frozen by W0a).
//!
//! [`SceneSpec`] mirrors `python/fabrail/schemas/scene-1.schema.json` (W0-B3) field for
//! field; [`SceneSpec::validate`] enforces every rule the schema expresses plus the
//! loader rules the schema cannot (unique device ids and floor levels, device levels
//! naming a floor, operator/record device ids existing, `series.ground` iff
//! `ground.series`, egress devices never locking, including through a
//! `design.landing.parking_lock` override). The seismic-latch capacity rule
//! (`seat_and_latch` needs `x_retention_n ≥ m_rated·a_design`) needs the resolved
//! parameter set and is checked by [`run_scene`] (E2).
//!
//! Coupling is one-way (building → devices). Devices are stepped in registry order in
//! chunks of `BASE_TICK_HZ / sync_hz` ticks (200 at 100 Hz), rayon across devices behind
//! the `parallel` feature, bus power summed in index order. Every alert is labelled
//! "synthetic/injected, not a live EEW feed".
//!
//! [`run_scene`] validates the spec and the series bytes and then runs the scene loop
//! (private `runner`): the building model (private `building`: fixed-base shear
//! building, linear-equivalent FPS isolation, HAVEN-F reduced platform, SENTINEL tower),
//! one `fabrail_sim::Session` per core device with its noise seed from
//! `SeedSequence(seed).spawn(n)[i]` (private `seeds`), the house-level EEW fusion, the
//! choreographer (`fabrail_exo::choreo`) and the contents ([`contents`]). An explicit
//! `seat_and_latch` needs `design.landing.seismic_latch.enabled` (rejected otherwise);
//! the latch capacity x_retention_n ≥ m_rated·a_design is checked against the scene's
//! predicted PFA_x by the choreographer, which falls back to SeatAndHold, and the scene
//! reports `drawer_opened` whenever m·a_floor,x exceeds the retention.

mod building;
pub mod contents;
mod runner;
mod seeds;

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use fabrail_exo::choreo::policy::ChoreoPolicy;
use fabrail_exo::choreo::restore::CommandSource;
use fabrail_exo::hmd::HmdMode;
use fabrail_exo_safety::AlertFeedKind;
use fabrail_sim::recorder::{Channel, EventRec};
use fabrail_sim::scenario::{Axis, ChannelSelect, InitLevSpec, SimError};
use fabrail_types::{BASE_TICK_HZ, EgressRole};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use self::contents::{ContentItem, ContentKind};

/// Scene schema identifier.
pub const SCENE_SCHEMA: &str = "fabrail.scene/1";

/// The mandatory provenance label of every scene alert.
pub const SCENE_PROVENANCE: &str = "synthetic/injected, not a live EEW feed";

/// Reference of the ground-motion series in `ground.series`.
pub const GROUND_SERIES_REF: &str = "series:ground";

/// Name of the ground-motion series in `series`.
pub const SERIES_GROUND: &str = "ground";

/// Maximum devices per scene (= `fabrail_exo::choreo::registry::MAX_DEVICES`).
pub const MAX_SCENE_DEVICES: usize = fabrail_exo::choreo::registry::MAX_DEVICES;

/// Allowed device synchronisation rates, Hz.
pub const SYNC_HZ: [u32; 4] = [50, 100, 200, 1000];

// ---------------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------------

/// Everything that can go wrong loading or running a scene.
#[derive(Clone, Debug, PartialEq)]
pub enum SceneError {
    /// Not valid JSON, or a field of the wrong type/shape (serde message).
    Json(String),
    /// The `schema` field names another schema (the value found).
    Schema(String),
    /// A field violates a schema or loader rule.
    Invalid {
        /// Dotted path, e.g. `devices[3].policy`.
        path: String,
        /// Reason.
        reason: String,
    },
    /// A series byte buffer does not match its declaration.
    Series {
        /// Series name.
        name: String,
        /// Reason.
        reason: String,
    },
    /// A device's scenario or parameters were rejected by fabrail-sim.
    Sim(SimError),
    /// A Wave-0 stub was called (the owner is named).
    NotImplemented(&'static str),
}

impl fmt::Display for SceneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SceneError::Json(m) => write!(f, "scene: invalid JSON: {m}"),
            SceneError::Schema(found) => {
                write!(f, "schema: expected \"{SCENE_SCHEMA}\", found \"{found}\"")
            }
            SceneError::Invalid { path, reason } => write!(f, "{path}: {reason}"),
            SceneError::Series { name, reason } => write!(f, "series.{name}: {reason}"),
            SceneError::Sim(e) => write!(f, "{e}"),
            SceneError::NotImplemented(what) => write!(f, "not implemented yet: {what}"),
        }
    }
}

impl std::error::Error for SceneError {}

impl From<SimError> for SceneError {
    fn from(e: SimError) -> Self {
        SceneError::Sim(e)
    }
}

fn invalid(path: impl Into<String>, reason: impl Into<String>) -> SceneError {
    SceneError::Invalid {
        path: path.into(),
        reason: reason.into(),
    }
}

// ---------------------------------------------------------------------------------
// Spec types (mirror of scene-1.schema.json)
// ---------------------------------------------------------------------------------

/// Scene variant.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SceneVariant {
    /// Network alert confirmed by a local P pick before the S wave.
    #[default]
    Warned,
    /// Near-field: blind zone, on-site detection only.
    NearField,
    /// A spoofed (injected) alert with no earthquake.
    SpoofedAlert,
    /// No event.
    Quiet,
    /// Anything else.
    Custom,
}

/// One floor of the host building.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FloorSpec {
    /// Floor level (0 = ground), unique.
    pub level: u32,
    /// Floor label.
    pub name: String,
    /// Height above the ground (or the isolation plane), m.
    pub z_m: f64,
    /// Floor mass, kg (null: uniform distribution of the model mass).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mass_kg: Option<f64>,
    /// Storey shear stiffness below this floor, N/m (null: scaled to `period_s`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub k_n_per_m: Option<f64>,
}

fn d_zeta() -> f64 {
    0.05
}
fn d_one() -> f64 {
    1.0
}
fn d_iso_period_haven() -> f64 {
    6.0
}
fn d_iso_zeta() -> f64 {
    0.3
}
fn d_stroke_haven() -> f64 {
    0.9
}
fn d_zeta_s() -> f64 {
    0.01
}
fn d_hmd_mass_ratio() -> f64 {
    0.01
}

/// HAVEN-F platform control.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IsolationControl {
    /// Passive guideway plus copper damping.
    #[default]
    Passive,
    /// Skyhook (research tier).
    Skyhook,
}

/// The host building (internally tagged on `config`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "config", rename_all = "snake_case", deny_unknown_fields)]
pub enum BuildingConfig {
    /// Fixed-base shear building (🔶).
    FixedBase {
        /// Floors.
        floors: Vec<FloorSpec>,
        /// Fixed-base fundamental period, s.
        period_s: f64,
        /// Structural damping ratio.
        #[serde(default = "d_zeta")]
        zeta: f64,
        /// Building height for z/h, m (null: max z_m).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        height_m: Option<f64>,
        /// Topographic amplification factor.
        #[serde(default = "d_one")]
        topo_factor: f64,
    },
    /// Conventional FPS isolation, linear-equivalent (🔶).
    FpsIsolated {
        /// Floors.
        floors: Vec<FloorSpec>,
        /// Superstructure period, s.
        period_s: f64,
        /// Structural damping ratio.
        #[serde(default = "d_zeta")]
        zeta: f64,
        /// Building height, m.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        height_m: Option<f64>,
        /// Topographic amplification factor.
        #[serde(default = "d_one")]
        topo_factor: f64,
        /// Isolation period, s.
        iso_period_s: f64,
        /// FPS friction coefficient.
        mu: f64,
        /// Isolator stroke, m.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        iso_stroke_m: Option<f64>,
    },
    /// HAVEN-F reduced platform (🧪).
    HavenF {
        /// Floors.
        floors: Vec<FloorSpec>,
        /// Superstructure period, s.
        period_s: f64,
        /// Structural damping ratio.
        #[serde(default = "d_zeta")]
        zeta: f64,
        /// Building height, m.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        height_m: Option<f64>,
        /// Topographic amplification factor.
        #[serde(default = "d_one")]
        topo_factor: f64,
        /// Isolation period, s.
        #[serde(default = "d_iso_period_haven")]
        iso_period_s: f64,
        /// Isolation damping ratio.
        #[serde(default = "d_iso_zeta")]
        iso_zeta: f64,
        /// Stroke, m.
        #[serde(default = "d_stroke_haven")]
        stroke_m: f64,
        /// Control.
        #[serde(default)]
        control: IsolationControl,
    },
    /// Supertall tower with a SENTINEL damper (penthouse variant).
    TowerSentinel {
        /// Floors.
        floors: Vec<FloorSpec>,
        /// Tower fundamental period, s.
        period_s: f64,
        /// Structural damping ratio.
        #[serde(default = "d_zeta")]
        zeta: f64,
        /// Building height, m.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        height_m: Option<f64>,
        /// Topographic amplification factor.
        #[serde(default = "d_one")]
        topo_factor: f64,
        /// Structural damping ratio of the tower mode used by the damper.
        #[serde(default = "d_zeta_s")]
        zeta_s: f64,
        /// Damper mass ratio μ.
        #[serde(default = "d_hmd_mass_ratio")]
        hmd_mass_ratio: f64,
        /// Damper mode.
        #[serde(default)]
        hmd_mode: HmdMode,
    },
}

impl BuildingConfig {
    /// The `config` tag.
    pub fn config_name(&self) -> &'static str {
        match self {
            BuildingConfig::FixedBase { .. } => "fixed_base",
            BuildingConfig::FpsIsolated { .. } => "fps_isolated",
            BuildingConfig::HavenF { .. } => "haven_f",
            BuildingConfig::TowerSentinel { .. } => "tower_sentinel",
        }
    }

    /// The floors.
    pub fn floors(&self) -> &[FloorSpec] {
        match self {
            BuildingConfig::FixedBase { floors, .. }
            | BuildingConfig::FpsIsolated { floors, .. }
            | BuildingConfig::HavenF { floors, .. }
            | BuildingConfig::TowerSentinel { floors, .. } => floors,
        }
    }

    /// (period_s, zeta, height_m, topo_factor) shared by every configuration.
    pub fn common(&self) -> (f64, f64, Option<f64>, f64) {
        match *self {
            BuildingConfig::FixedBase {
                period_s,
                zeta,
                height_m,
                topo_factor,
                ..
            }
            | BuildingConfig::FpsIsolated {
                period_s,
                zeta,
                height_m,
                topo_factor,
                ..
            }
            | BuildingConfig::HavenF {
                period_s,
                zeta,
                height_m,
                topo_factor,
                ..
            }
            | BuildingConfig::TowerSentinel {
                period_s,
                zeta,
                height_m,
                topo_factor,
                ..
            } => (period_s, zeta, height_m, topo_factor),
        }
    }

    /// Building height for z/h: `height_m`, else the highest floor.
    pub fn height_m(&self) -> f64 {
        let (_, _, h, _) = self.common();
        h.unwrap_or_else(|| self.floors().iter().map(|f| f.z_m).fold(0.0, f64::max))
    }
}

/// Ground-motion source.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GroundSource {
    /// `fabrail.pwave.boore_ps` synthesis (Python, numpy SeedSequence).
    #[default]
    BoorePs,
    /// A recorded accelerogram.
    Record,
    /// No ground motion.
    None,
}

fn d_percentile() -> f64 {
    50.0
}

/// The ground-motion record.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundSpec {
    /// Source.
    pub source: GroundSource,
    /// `"series:ground"` (required unless `source` is `none`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub series: Option<String>,
    /// Magnitude (boore_ps).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub magnitude: Option<f64>,
    /// Epicentral distance, km (boore_ps).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub r_epi_km: Option<f64>,
    /// Depth, km (boore_ps).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub depth_km: Option<f64>,
    /// Origin time on the scene clock, s (boore_ps).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin_t_s: Option<f64>,
    /// P arrival at the site, s (reported).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t_p_s: Option<f64>,
    /// S arrival at the site, s (reported).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t_s_s: Option<f64>,
    /// Ground-motion percentile, stated in every run.
    #[serde(default = "d_percentile")]
    pub percentile: f64,
    /// Site κ, s.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kappa_s: Option<f64>,
    /// Record name (record source).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub record_name: Option<String>,
}

/// The ground-motion series declaration (time axis, channels ax/ay/az).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GroundSeriesSpec {
    /// Axis (must be `time`).
    pub axis: Axis,
    /// Sample rate, Hz.
    pub fs_hz: f64,
    /// Time of sample 0, s.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t0_s: Option<f64>,
    /// Channel names (subset of ax, ay, az), channel-major LE f64 bytes.
    pub channels: Vec<String>,
}

/// `series` block.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SceneSeries {
    /// Ground acceleration series.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ground: Option<GroundSeriesSpec>,
}

/// Device class (a core rail class, or the exotic HAVEN-R plinth).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceClassSpec {
    /// DRAWER.
    #[default]
    Drawer,
    /// DOOR.
    Door,
    /// WALL.
    Wall,
    /// HEAVY.
    Heavy,
    /// HAVEN-R art plinth (research tier).
    HavenR,
}

/// Device model fidelity.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceFidelity {
    /// Full 6-DOF carriage (hero devices; SIL ≡ firmware claimed only for these).
    Carriage6,
    /// Travel-only reduced model.
    #[default]
    Travel1d,
    /// HAVEN-R 3-DOF planar platform.
    Platform3dof,
}

/// Coil mode of a device (single source for its Session scenario).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CoilSpec {
    /// First-order current lag (sim only).
    Lag,
    /// Voltage mode (SIL ≡ firmware).
    #[default]
    Voltage,
}

/// Device command verb.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceCmd {
    /// Open.
    Open,
    /// Close.
    Close,
    /// Go to `x_m`.
    GoTo,
    /// Stop.
    Stop,
    /// Lift.
    Lift,
    /// Land.
    Land,
}

/// A timed user command to one device.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeviceCommand {
    /// Time, s.
    pub t_s: f64,
    /// Verb.
    pub cmd: DeviceCmd,
    /// Target (go_to), m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub x_m: Option<f64>,
    /// Command source.
    #[serde(default)]
    pub source: CommandSource,
}

/// A device's initial state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeviceInitialState {
    /// Vertical rest condition (null: class rest state).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lev: Option<InitLevSpec>,
    /// Travel position, m (null: closed).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub x_m: Option<f64>,
    /// Payload, kg.
    #[serde(default)]
    pub payload_kg: f64,
    /// Parking lock engaged (null: by rest policy).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub locked: Option<bool>,
}

/// One device of the registry (registry order = array order).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeviceSpec {
    /// Unique id `^[a-z][a-z0-9_]{0,31}$`.
    pub id: String,
    /// Display label.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
    /// Class.
    #[serde(rename = "class")]
    pub class: DeviceClassSpec,
    /// Named preset variant (e.g. `cabinet_slider`), resolved by Python.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preset: Option<String>,
    /// Parameter overrides by dotted path (`Params.with_overrides`).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub overrides: BTreeMap<String, Value>,
    /// Floor level (must name a floor).
    pub level: u32,
    /// Room label.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub room: Option<String>,
    /// Plan position (display), m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_m: Option<[f64; 2]>,
    /// Heading of the rail's +x in the building frame, degrees.
    #[serde(default)]
    pub heading_deg: f64,
    /// Egress role.
    #[serde(default)]
    pub egress: EgressRole,
    /// Model fidelity.
    pub fidelity: DeviceFidelity,
    /// Coil mode.
    #[serde(default)]
    pub coil: CoilSpec,
    /// Hero device (Carriage6 only).
    #[serde(default)]
    pub hero: bool,
    /// Choreographer policy.
    #[serde(default)]
    pub policy: ChoreoPolicy,
    /// Contents.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub contents: Vec<ContentItem>,
    /// Initial state.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub initial_state: Option<DeviceInitialState>,
    /// Timed commands.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub commands: Vec<DeviceCommand>,
    /// Rust noise seed (null: from the scene seed, `SeedSequence(seed).spawn(n)[i]`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub noise_seed: Option<u64>,
}

/// One synthetic alert feed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FeedSpec {
    /// Kind (never `local_p`: the local pick comes from the device detectors).
    pub kind: AlertFeedKind,
    /// Arrival time on the scene clock, s.
    pub t_s: f64,
    /// Feed id.
    pub id: u32,
    /// Magnitude.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub magnitude: Option<f64>,
    /// Epicentral distance, km.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub r_epi_km: Option<f64>,
    /// Depth, km.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub depth_km: Option<f64>,
    /// Origin time on the scene clock, s (may precede 0).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub origin_t_s: Option<f64>,
    /// Predicted intensity (MMI).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mmi: Option<f64>,
    /// Cancellation time, s.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cancel_t_s: Option<f64>,
    /// Spoofed (injected; never counts as independent).
    #[serde(default)]
    pub spoofed: bool,
}

fn d_true() -> bool {
    true
}
fn d_vp() -> f64 {
    6.0
}
fn d_vs() -> f64 {
    3.5
}
fn d_sigma() -> f64 {
    0.5
}

/// The synthetic alert inputs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AlertSpec {
    /// Must equal [`SCENE_PROVENANCE`].
    pub provenance: String,
    /// On-site P detection active.
    #[serde(default = "d_true")]
    pub local_p: bool,
    /// Feeds.
    #[serde(default)]
    pub feeds: Vec<FeedSpec>,
    /// P-wave speed, km/s.
    #[serde(default = "d_vp")]
    pub vp_kmps: f64,
    /// S-wave speed, km/s.
    #[serde(default = "d_vs")]
    pub vs_kmps: f64,
    /// S-ETA standard deviation, s.
    #[serde(default = "d_sigma")]
    pub s_eta_sigma_s: f64,
}

/// Building power.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PowerSpec {
    /// Bus power budget, W.
    pub p_bus_max_w: f64,
    /// A building UPS carries the bus through mains loss.
    #[serde(default)]
    pub ups: bool,
    /// Mains loss time, s.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mains_loss_t_s: Option<f64>,
    /// Mains restore time, s.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mains_restore_t_s: Option<f64>,
}

fn d_quantile() -> f64 {
    0.05
}

/// Choreographer settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChoreoSpec {
    /// Choreographer on (off: every device keeps its core class action).
    #[serde(default = "d_true")]
    pub enable: bool,
    /// Smooth ride-through allowed (research tier).
    #[serde(default)]
    pub smoothing: bool,
    /// Deadline quantile of the S arrival.
    #[serde(default = "d_quantile")]
    pub deadline_quantile: f64,
    /// Deadline margin, s.
    #[serde(default = "d_one")]
    pub deadline_margin_s: f64,
    /// Unattended automatic closing allowed.
    #[serde(default = "d_true")]
    pub unattended_close: bool,
}

impl Default for ChoreoSpec {
    fn default() -> Self {
        ChoreoSpec {
            enable: true,
            smoothing: false,
            deadline_quantile: d_quantile(),
            deadline_margin_s: 1.0,
            unattended_close: true,
        }
    }
}

/// Operator action verb.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OperatorKind {
    /// INSPECT manual reset.
    InspectReset,
    /// Restore (local only).
    Restore,
    /// Cancel the alert.
    CancelAlert,
}

/// A timed operator action.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OperatorAction {
    /// Time, s.
    pub t_s: f64,
    /// Verb.
    pub action: OperatorKind,
    /// Source.
    #[serde(default)]
    pub source: CommandSource,
    /// Devices (null: all).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub devices: Option<Vec<String>>,
}

/// Recorded devices: `"all"` or a list of ids.
#[derive(Clone, Debug, Default, PartialEq)]
pub enum RecordDevices {
    /// Every device.
    #[default]
    All,
    /// These ids.
    List(Vec<String>),
}

impl Serialize for RecordDevices {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        match self {
            RecordDevices::All => s.serialize_str("all"),
            RecordDevices::List(v) => v.serialize(s),
        }
    }
}

impl<'de> Deserialize<'de> for RecordDevices {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        match Value::deserialize(d)? {
            Value::String(s) if s == "all" => Ok(RecordDevices::All),
            v @ Value::Array(_) => serde_json::from_value::<Vec<String>>(v)
                .map(RecordDevices::List)
                .map_err(serde::de::Error::custom),
            other => Err(serde::de::Error::custom(format!(
                "expected \"all\" or a list of device ids, found {other}"
            ))),
        }
    }
}

fn d_rate() -> f64 {
    100.0
}

/// Recording settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SceneRecordSpec {
    /// Rate, Hz.
    #[serde(default = "d_rate")]
    pub rate_hz: f64,
    /// Devices.
    #[serde(default)]
    pub devices: RecordDevices,
    /// Channels.
    #[serde(default)]
    pub channels: ChannelSelect,
}

impl Default for SceneRecordSpec {
    fn default() -> Self {
        SceneRecordSpec {
            rate_hz: d_rate(),
            devices: RecordDevices::All,
            channels: ChannelSelect::Default,
        }
    }
}

fn d_sync() -> u32 {
    100
}
fn d_mu_sweep() -> f64 {
    0.3
}

/// A building scene document (`fabrail.scene/1`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SceneSpec {
    /// Must equal [`SCENE_SCHEMA`].
    pub schema: String,
    /// Scene name `^[A-Za-z0-9_.-]{1,64}$`.
    pub name: String,
    /// Description.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    /// Scene seed (numpy SeedSequence root).
    pub seed: u64,
    /// Duration, s (≤ 3600).
    pub duration_s: f64,
    /// Device sync rate, Hz.
    #[serde(default = "d_sync")]
    pub sync_hz: u32,
    /// Variant.
    pub variant: SceneVariant,
    /// Host building.
    pub building: BuildingConfig,
    /// Ground motion.
    pub ground: GroundSpec,
    /// Series declarations.
    #[serde(default)]
    pub series: SceneSeries,
    /// Devices in registry order.
    pub devices: Vec<DeviceSpec>,
    /// Alert inputs.
    pub alert: AlertSpec,
    /// Building power.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub power: Option<PowerSpec>,
    /// Choreographer settings.
    #[serde(default)]
    pub choreo: ChoreoSpec,
    /// Operator actions.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub operator: Vec<OperatorAction>,
    /// Contents μ sweep fraction (±).
    #[serde(default = "d_mu_sweep")]
    pub contents_mu_sweep_frac: f64,
    /// Recording.
    #[serde(default)]
    pub record: SceneRecordSpec,
}

fn id_ok(s: &str) -> bool {
    let b = s.as_bytes();
    !b.is_empty()
        && b.len() <= 32
        && b[0].is_ascii_lowercase()
        && b.iter()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || *c == b'_')
}

fn name_ok(s: &str) -> bool {
    !s.is_empty()
        && s.len() <= 64
        && s.bytes()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, b'_' | b'.' | b'-'))
}

fn channel_ok(s: &str) -> bool {
    let b = s.as_bytes();
    !b.is_empty()
        && b.len() <= 64
        && b[0].is_ascii_lowercase()
        && b.iter()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || *c == b'_')
}

fn override_path_ok(s: &str) -> bool {
    let mut parts = s.split('.');
    matches!(parts.next(), Some("design" | "control" | "safety")) && {
        let rest: Vec<&str> = parts.collect();
        !rest.is_empty()
            && rest.iter().all(|p| {
                !p.is_empty()
                    && p.bytes()
                        .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == b'_')
            })
    }
}

/// Checks `lo < x ≤ hi` style ranges (inclusive flags per side).
fn range(
    path: &str,
    x: f64,
    lo: f64,
    lo_incl: bool,
    hi: f64,
    hi_incl: bool,
) -> Result<(), SceneError> {
    let ok_lo = if lo_incl { x >= lo } else { x > lo };
    let ok_hi = if hi_incl { x <= hi } else { x < hi };
    if x.is_finite() && ok_lo && ok_hi {
        Ok(())
    } else {
        let l = if lo_incl { '[' } else { '(' };
        let h = if hi_incl { ']' } else { ')' };
        Err(invalid(path, format!("{x} is outside {l}{lo}, {hi}{h}")))
    }
}

fn opt_range(
    path: &str,
    x: Option<f64>,
    lo: f64,
    lo_incl: bool,
    hi: f64,
    hi_incl: bool,
) -> Result<(), SceneError> {
    match x {
        Some(v) => range(path, v, lo, lo_incl, hi, hi_incl),
        None => Ok(()),
    }
}

impl SceneSpec {
    /// Parses and validates a scene document.
    pub fn from_json(s: &str) -> Result<SceneSpec, SceneError> {
        let v: Value = serde_json::from_str(s).map_err(|e| SceneError::Json(e.to_string()))?;
        if let Some(found) = v.get("schema").and_then(Value::as_str)
            && found != SCENE_SCHEMA
        {
            return Err(SceneError::Schema(found.to_string()));
        }
        let spec: SceneSpec =
            serde_json::from_value(v).map_err(|e| SceneError::Json(e.to_string()))?;
        spec.validate()?;
        Ok(spec)
    }

    /// The canonical document JSON (defaults filled in).
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }

    /// Pretty-printed canonical JSON.
    pub fn to_json_pretty(&self) -> String {
        serde_json::to_string_pretty(self).unwrap_or_default()
    }

    /// Base ticks per device chunk (`BASE_TICK_HZ / sync_hz`; 200 at 100 Hz).
    pub fn chunk_ticks(&self) -> u64 {
        if self.sync_hz == 0 {
            return 0;
        }
        u64::from(BASE_TICK_HZ / self.sync_hz)
    }

    /// Checks every schema rule and loader rule (see the module docs).
    pub fn validate(&self) -> Result<(), SceneError> {
        if self.schema != SCENE_SCHEMA {
            return Err(SceneError::Schema(self.schema.clone()));
        }
        if !name_ok(&self.name) {
            return Err(invalid("name", "must match ^[A-Za-z0-9_.-]{1,64}$"));
        }
        if self.doc.as_ref().is_some_and(|d| d.chars().count() > 4096) {
            return Err(invalid("doc", "longer than 4096 characters"));
        }
        range("duration_s", self.duration_s, 0.0, false, 3600.0, true)?;
        if !SYNC_HZ.contains(&self.sync_hz) {
            return Err(invalid("sync_hz", "must be one of 50, 100, 200, 1000"));
        }
        let levels = self.validate_building()?;
        self.validate_ground()?;
        let ids = self.validate_devices(&levels)?;
        self.validate_alert()?;
        if let Some(p) = &self.power {
            range(
                "power.p_bus_max_w",
                p.p_bus_max_w,
                0.0,
                false,
                f64::MAX,
                true,
            )?;
            opt_range(
                "power.mains_loss_t_s",
                p.mains_loss_t_s,
                0.0,
                true,
                f64::MAX,
                true,
            )?;
            opt_range(
                "power.mains_restore_t_s",
                p.mains_restore_t_s,
                0.0,
                true,
                f64::MAX,
                true,
            )?;
        }
        range(
            "choreo.deadline_quantile",
            self.choreo.deadline_quantile,
            0.0,
            false,
            0.5,
            true,
        )?;
        range(
            "choreo.deadline_margin_s",
            self.choreo.deadline_margin_s,
            0.0,
            true,
            10.0,
            true,
        )?;
        for (i, op) in self.operator.iter().enumerate() {
            range(
                &format!("operator[{i}].t_s"),
                op.t_s,
                0.0,
                true,
                f64::MAX,
                true,
            )?;
            for (k, d) in op.devices.iter().flatten().enumerate() {
                if !ids.contains(d.as_str()) {
                    return Err(invalid(
                        format!("operator[{i}].devices[{k}]"),
                        format!("unknown device \"{d}\""),
                    ));
                }
            }
        }
        range(
            "contents_mu_sweep_frac",
            self.contents_mu_sweep_frac,
            0.0,
            true,
            1.0,
            false,
        )?;
        range(
            "record.rate_hz",
            self.record.rate_hz,
            0.0,
            false,
            1000.0,
            true,
        )?;
        if let RecordDevices::List(v) = &self.record.devices {
            for (k, d) in v.iter().enumerate() {
                if !ids.contains(d.as_str()) {
                    return Err(invalid(
                        format!("record.devices[{k}]"),
                        format!("unknown device \"{d}\""),
                    ));
                }
            }
        }
        if let ChannelSelect::Names(v) = &self.record.channels {
            if v.is_empty() {
                return Err(invalid(
                    "record.channels",
                    "an explicit list needs at least one name",
                ));
            }
            for (k, c) in v.iter().enumerate() {
                if !channel_ok(c) {
                    return Err(invalid(
                        format!("record.channels[{k}]"),
                        "must match ^[a-z][a-z0-9_]{0,63}$",
                    ));
                }
            }
        }
        Ok(())
    }

    fn validate_building(&self) -> Result<BTreeSet<u32>, SceneError> {
        let b = &self.building;
        let floors = b.floors();
        if floors.is_empty() || floors.len() > 60 {
            return Err(invalid("building.floors", "needs 1 to 60 floors"));
        }
        let mut levels = BTreeSet::new();
        for (i, f) in floors.iter().enumerate() {
            let p = format!("building.floors[{i}]");
            if f.level > 200 {
                return Err(invalid(format!("{p}.level"), "must be 0..=200"));
            }
            if !levels.insert(f.level) {
                return Err(invalid(format!("{p}.level"), "duplicate floor level"));
            }
            if f.name.is_empty() || f.name.chars().count() > 64 {
                return Err(invalid(format!("{p}.name"), "must be 1 to 64 characters"));
            }
            range(&format!("{p}.z_m"), f.z_m, 0.0, true, 1000.0, true)?;
            opt_range(
                &format!("{p}.mass_kg"),
                f.mass_kg,
                0.0,
                false,
                f64::MAX,
                true,
            )?;
            opt_range(
                &format!("{p}.k_n_per_m"),
                f.k_n_per_m,
                0.0,
                false,
                f64::MAX,
                true,
            )?;
        }
        let (period_s, zeta, height_m, topo) = b.common();
        range("building.period_s", period_s, 0.0, false, 20.0, true)?;
        range("building.zeta", zeta, 0.0, true, 0.3, true)?;
        opt_range("building.height_m", height_m, 0.0, false, f64::MAX, true)?;
        range("building.topo_factor", topo, 1.0, true, 3.0, true)?;
        match *b {
            BuildingConfig::FixedBase { .. } => {}
            BuildingConfig::FpsIsolated {
                iso_period_s,
                mu,
                iso_stroke_m,
                ..
            } => {
                range(
                    "building.iso_period_s",
                    iso_period_s,
                    0.0,
                    false,
                    10.0,
                    true,
                )?;
                range("building.mu", mu, 0.0, false, 0.3, true)?;
                opt_range(
                    "building.iso_stroke_m",
                    iso_stroke_m,
                    0.0,
                    false,
                    f64::MAX,
                    true,
                )?;
            }
            BuildingConfig::HavenF {
                iso_period_s,
                iso_zeta,
                stroke_m,
                ..
            } => {
                range(
                    "building.iso_period_s",
                    iso_period_s,
                    0.0,
                    false,
                    20.0,
                    true,
                )?;
                range("building.iso_zeta", iso_zeta, 0.0, true, 1.0, true)?;
                range("building.stroke_m", stroke_m, 0.0, false, 5.0, true)?;
            }
            BuildingConfig::TowerSentinel {
                zeta_s,
                hmd_mass_ratio,
                hmd_mode,
                ..
            } => {
                range("building.zeta_s", zeta_s, 0.0, true, 0.2, true)?;
                range(
                    "building.hmd_mass_ratio",
                    hmd_mass_ratio,
                    0.0,
                    false,
                    0.1,
                    true,
                )?;
                if hmd_mode == HmdMode::Locked {
                    return Err(invalid("building.hmd_mode", "locked is not a scene mode"));
                }
            }
        }
        Ok(levels)
    }

    fn validate_ground(&self) -> Result<(), SceneError> {
        let g = &self.ground;
        if let Some(s) = &g.series
            && s != GROUND_SERIES_REF
        {
            return Err(invalid(
                "ground.series",
                format!("must be \"{GROUND_SERIES_REF}\" or null"),
            ));
        }
        if matches!(g.source, GroundSource::BoorePs | GroundSource::Record) && g.series.is_none() {
            return Err(invalid(
                "ground.series",
                "required for boore_ps and record sources",
            ));
        }
        if g.source == GroundSource::BoorePs {
            for (k, v) in [
                ("magnitude", g.magnitude),
                ("r_epi_km", g.r_epi_km),
                ("depth_km", g.depth_km),
                ("origin_t_s", g.origin_t_s),
            ] {
                if v.is_none() {
                    return Err(invalid(format!("ground.{k}"), "required for boore_ps"));
                }
            }
        }
        opt_range("ground.magnitude", g.magnitude, 0.0, true, 10.0, true)?;
        opt_range("ground.r_epi_km", g.r_epi_km, 0.0, true, 20_000.0, true)?;
        opt_range("ground.depth_km", g.depth_km, 0.0, true, 700.0, true)?;
        for (k, v) in [
            ("origin_t_s", g.origin_t_s),
            ("t_p_s", g.t_p_s),
            ("t_s_s", g.t_s_s),
        ] {
            if v.is_some_and(|x| !x.is_finite()) {
                return Err(invalid(format!("ground.{k}"), "must be finite"));
            }
        }
        range("ground.percentile", g.percentile, 0.0, false, 100.0, false)?;
        opt_range("ground.kappa_s", g.kappa_s, 0.0, true, 0.2, true)?;
        if g.record_name
            .as_ref()
            .is_some_and(|n| n.chars().count() > 128)
        {
            return Err(invalid("ground.record_name", "longer than 128 characters"));
        }
        // Loader rule: series.ground iff ground.series.
        match (&self.series.ground, g.series.is_some()) {
            (Some(_), false) => {
                return Err(invalid(
                    "series.ground",
                    "declared but ground.series is null",
                ));
            }
            (None, true) => {
                return Err(invalid(
                    "series.ground",
                    "ground.series names it but it is not declared",
                ));
            }
            _ => {}
        }
        if let Some(s) = &self.series.ground {
            if s.axis != Axis::Time {
                return Err(invalid("series.ground.axis", "must be time"));
            }
            range("series.ground.fs_hz", s.fs_hz, 0.0, false, f64::MAX, true)?;
            if s.t0_s.is_some_and(|t| !t.is_finite()) {
                return Err(invalid("series.ground.t0_s", "must be finite"));
            }
            if s.channels.is_empty() || s.channels.len() > 3 {
                return Err(invalid(
                    "series.ground.channels",
                    "needs 1 to 3 of ax, ay, az",
                ));
            }
            for (k, c) in s.channels.iter().enumerate() {
                if !matches!(c.as_str(), "ax" | "ay" | "az") {
                    return Err(invalid(
                        format!("series.ground.channels[{k}]"),
                        "must be ax, ay or az",
                    ));
                }
            }
        }
        Ok(())
    }

    fn validate_devices(&self, levels: &BTreeSet<u32>) -> Result<BTreeSet<&str>, SceneError> {
        if self.devices.is_empty() || self.devices.len() > MAX_SCENE_DEVICES {
            return Err(invalid(
                "devices",
                format!("needs 1 to {MAX_SCENE_DEVICES} devices"),
            ));
        }
        let mut ids = BTreeSet::new();
        for (i, d) in self.devices.iter().enumerate() {
            let p = format!("devices[{i}]");
            if !id_ok(&d.id) {
                return Err(invalid(
                    format!("{p}.id"),
                    "must match ^[a-z][a-z0-9_]{0,31}$",
                ));
            }
            if !ids.insert(d.id.as_str()) {
                return Err(invalid(
                    format!("{p}.id"),
                    format!("duplicate device id \"{}\"", d.id),
                ));
            }
            if d.label.as_ref().is_some_and(|l| l.chars().count() > 64) {
                return Err(invalid(format!("{p}.label"), "longer than 64 characters"));
            }
            if d.preset.as_ref().is_some_and(|s| !id_ok(s)) {
                return Err(invalid(
                    format!("{p}.preset"),
                    "must match ^[a-z][a-z0-9_]{0,31}$",
                ));
            }
            for (k, v) in &d.overrides {
                if !override_path_ok(k) {
                    return Err(invalid(
                        format!("{p}.overrides.{k}"),
                        "path must match ^(design|control|safety)(\\.[a-z0-9_]+)+$",
                    ));
                }
                if !(v.is_number() || v.is_boolean() || v.is_string()) {
                    return Err(invalid(
                        format!("{p}.overrides.{k}"),
                        "value must be a number, boolean or string",
                    ));
                }
            }
            if d.level > 200 || !levels.contains(&d.level) {
                return Err(invalid(
                    format!("{p}.level"),
                    format!("level {} names no floor", d.level),
                ));
            }
            if d.room.as_ref().is_some_and(|r| r.chars().count() > 64) {
                return Err(invalid(format!("{p}.room"), "longer than 64 characters"));
            }
            if let Some([x, y]) = d.position_m
                && !(x.is_finite() && y.is_finite())
            {
                return Err(invalid(format!("{p}.position_m"), "must be finite"));
            }
            range(
                &format!("{p}.heading_deg"),
                d.heading_deg,
                -360.0,
                true,
                360.0,
                true,
            )?;
            let egress_path = d.egress != EgressRole::None;
            if egress_path
                && matches!(
                    d.policy,
                    ChoreoPolicy::LandAndLock | ChoreoPolicy::SeatAndLatch
                )
            {
                return Err(invalid(
                    format!("{p}.policy"),
                    "an egress device never locks or latches",
                ));
            }
            if egress_path
                && d.overrides
                    .get("design.landing.parking_lock")
                    .is_some_and(|v| v.as_bool() == Some(true))
            {
                return Err(invalid(
                    format!("{p}.overrides.design.landing.parking_lock"),
                    "an egress device never locks",
                ));
            }
            if d.class == DeviceClassSpec::HavenR && d.fidelity != DeviceFidelity::Platform3dof {
                return Err(invalid(
                    format!("{p}.fidelity"),
                    "haven_r must use platform3dof",
                ));
            }
            if d.class != DeviceClassSpec::HavenR && d.fidelity == DeviceFidelity::Platform3dof {
                return Err(invalid(
                    format!("{p}.fidelity"),
                    "platform3dof is for haven_r only",
                ));
            }
            if d.policy.drawer_only() && d.class != DeviceClassSpec::Drawer {
                return Err(invalid(
                    format!("{p}.policy"),
                    "smooth_ride_through, seat_and_hold and seat_and_latch are drawer-only",
                ));
            }
            if d.hero && d.fidelity != DeviceFidelity::Carriage6 {
                return Err(invalid(
                    format!("{p}.hero"),
                    "hero devices must be carriage6",
                ));
            }
            if d.contents.len() > 64 {
                return Err(invalid(format!("{p}.contents"), "at most 64 items"));
            }
            for (k, c) in d.contents.iter().enumerate() {
                let q = format!("{p}.contents[{k}]");
                if c.kind == ContentKind::Custom && c.mu_s.is_none() {
                    return Err(invalid(format!("{q}.mu_s"), "required for custom contents"));
                }
                if c.name.as_ref().is_some_and(|n| n.chars().count() > 64) {
                    return Err(invalid(format!("{q}.name"), "longer than 64 characters"));
                }
                opt_range(&format!("{q}.mu_s"), c.mu_s, 0.0, false, 2.0, true)?;
                opt_range(&format!("{q}.mu_k"), c.mu_k, 0.0, false, 2.0, true)?;
                opt_range(&format!("{q}.b_over_h"), c.b_over_h, 0.0, false, 5.0, true)?;
                opt_range(&format!("{q}.mass_kg"), c.mass_kg, 0.0, false, 100.0, true)?;
                if c.count == 0 || c.count > 1000 {
                    return Err(invalid(format!("{q}.count"), "must be 1..=1000"));
                }
            }
            if let Some(s) = &d.initial_state {
                if s.x_m.is_some_and(|x| !x.is_finite()) {
                    return Err(invalid(format!("{p}.initial_state.x_m"), "must be finite"));
                }
                range(
                    &format!("{p}.initial_state.payload_kg"),
                    s.payload_kg,
                    -1.0e5,
                    true,
                    1.0e5,
                    true,
                )?;
            }
            for (k, c) in d.commands.iter().enumerate() {
                let q = format!("{p}.commands[{k}]");
                range(&format!("{q}.t_s"), c.t_s, 0.0, true, f64::MAX, true)?;
                if c.cmd == DeviceCmd::GoTo && c.x_m.is_none() {
                    return Err(invalid(format!("{q}.x_m"), "required for go_to"));
                }
                if c.x_m.is_some_and(|x| !x.is_finite()) {
                    return Err(invalid(format!("{q}.x_m"), "must be finite"));
                }
            }
        }
        Ok(ids)
    }

    fn validate_alert(&self) -> Result<(), SceneError> {
        let a = &self.alert;
        if a.provenance != SCENE_PROVENANCE {
            return Err(invalid(
                "alert.provenance",
                format!("must be \"{SCENE_PROVENANCE}\""),
            ));
        }
        if a.feeds.len() > 32 {
            return Err(invalid("alert.feeds", "at most 32 feeds"));
        }
        for (i, f) in a.feeds.iter().enumerate() {
            let p = format!("alert.feeds[{i}]");
            if f.kind == AlertFeedKind::LocalP {
                return Err(invalid(
                    format!("{p}.kind"),
                    "local_p is not a feed (set alert.local_p)",
                ));
            }
            range(&format!("{p}.t_s"), f.t_s, 0.0, true, f64::MAX, true)?;
            opt_range(
                &format!("{p}.magnitude"),
                f.magnitude,
                0.0,
                true,
                10.0,
                true,
            )?;
            opt_range(
                &format!("{p}.r_epi_km"),
                f.r_epi_km,
                0.0,
                true,
                20_000.0,
                true,
            )?;
            opt_range(&format!("{p}.depth_km"), f.depth_km, 0.0, true, 700.0, true)?;
            if f.origin_t_s.is_some_and(|x| !x.is_finite()) {
                return Err(invalid(format!("{p}.origin_t_s"), "must be finite"));
            }
            opt_range(&format!("{p}.mmi"), f.mmi, 1.0, true, 12.0, true)?;
            opt_range(
                &format!("{p}.cancel_t_s"),
                f.cancel_t_s,
                0.0,
                true,
                f64::MAX,
                true,
            )?;
        }
        range("alert.vp_kmps", a.vp_kmps, 0.0, false, 10.0, true)?;
        range("alert.vs_kmps", a.vs_kmps, 0.0, false, 10.0, true)?;
        range(
            "alert.s_eta_sigma_s",
            a.s_eta_sigma_s,
            0.0,
            true,
            30.0,
            true,
        )?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------------

/// Options of a scene run.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct SceneRunOptions {
    /// Rayon workers across devices (`None`: default; results are worker-invariant).
    pub workers: Option<usize>,
    /// Quick mode: Travel1d for every device, 25 s window, one building config.
    pub quick: bool,
}

/// One device's recorded run.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DeviceRecord {
    /// Device id.
    pub id: String,
    /// Class name (`drawer`, …, `haven_r`).
    pub class: String,
    /// Resolved choreographer policy.
    pub policy: String,
    /// Channels at the scene record rate (all the same length).
    pub channels: Vec<Channel>,
    /// Device KPIs.
    pub kpis: BTreeMap<String, f64>,
    /// Why the device was not secured, if it was not.
    pub not_secured: Option<String>,
}

/// A scene event (device events carry the device id).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneEvent {
    /// Device id, or `None` for building/alert/choreographer events.
    pub device: Option<String>,
    /// The event.
    pub event: EventRec,
}

/// Scene provenance.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SceneMeta {
    /// fabrail version.
    pub version: String,
    /// Scene name.
    pub scene: String,
    /// Scene seed.
    pub seed: u64,
    /// Variant.
    pub variant: String,
    /// Building configuration.
    pub building: String,
    /// Alert provenance label (always [`SCENE_PROVENANCE`]).
    pub provenance: String,
    /// Canonical scene document.
    pub spec: Value,
    /// `fabrail_rt` GOLDEN_SELFTEST_HASH the run was built with.
    pub golden: u64,
    /// `"release"` or `"debug"`.
    pub build_profile: String,
    /// Wall-clock time, s (excluded from the hashed JSON).
    pub wall_s: f64,
}

impl SceneMeta {
    /// JSON without `wall_s` (the deterministic form; plan §2b item 16).
    pub fn hashed_json(&self) -> String {
        let mut v = serde_json::to_value(self).unwrap_or(Value::Null);
        if let Some(o) = v.as_object_mut() {
            o.remove("wall_s");
        }
        serde_json::to_string(&v).unwrap_or_default()
    }
}

/// A finished scene run.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SceneData {
    /// Record rate, Hz.
    pub rate_hz: f64,
    /// Building channels (floor accelerations per level, bus power, …).
    pub building: Vec<Channel>,
    /// Devices in registry order.
    pub devices: Vec<DeviceRecord>,
    /// Events in time order.
    pub events: Vec<SceneEvent>,
    /// Scene KPIs.
    pub kpis: BTreeMap<String, f64>,
    /// The house plan as JSON (per device: policy, target rank, start, deadline,
    /// status).
    pub plan: Value,
    /// Provenance.
    pub meta: SceneMeta,
}

/// Test-harness inputs of a scene run that the `fabrail.scene/1` document does not
/// carry (additive, Wave 2b E5): extra `fabrail.scenario/1` events per device id (a hand
/// or an ISO/TS 15066 obstacle in a drawer during a choreographed close, a hand at the
/// standoff during shaking). They are appended to the device's scenario events and
/// validated by the scenario loader; they never change the scene document itself.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SceneExtras {
    /// Extra scenario events per device id, in the device's scenario time (= scene time).
    pub device_events: BTreeMap<String, Vec<Value>>,
}

/// Runs a scene: `series` holds the declared series as channel-major LE f64 bytes.
/// Validates the spec and the series declarations, then runs the scene loop (see the
/// module docs). Results do not depend on `opts.workers`.
pub fn run_scene(
    spec: &SceneSpec,
    series: &BTreeMap<String, Vec<u8>>,
    opts: &SceneRunOptions,
) -> Result<SceneData, SceneError> {
    run_scene_with(spec, series, opts, &SceneExtras::default())
}

/// [`run_scene`] with test-harness inputs ([`SceneExtras`]); an unknown device id in
/// `extras` is rejected.
pub fn run_scene_with(
    spec: &SceneSpec,
    series: &BTreeMap<String, Vec<u8>>,
    opts: &SceneRunOptions,
    extras: &SceneExtras,
) -> Result<SceneData, SceneError> {
    spec.validate()?;
    for id in extras.device_events.keys() {
        if !spec.devices.iter().any(|d| &d.id == id) {
            return Err(invalid(
                format!("device_events.{id}"),
                "names no device of the scene",
            ));
        }
    }
    for name in series.keys() {
        if name != SERIES_GROUND || spec.series.ground.is_none() {
            return Err(SceneError::Series {
                name: name.clone(),
                reason: "bytes were given but the scene does not declare this series".into(),
            });
        }
    }
    if let Some(g) = &spec.series.ground {
        let Some(bytes) = series.get(SERIES_GROUND) else {
            return Err(SceneError::Series {
                name: SERIES_GROUND.into(),
                reason: "declared but no bytes were given".into(),
            });
        };
        let n_ch = g.channels.len();
        if n_ch == 0 || bytes.len() % (8 * n_ch) != 0 {
            return Err(SceneError::Series {
                name: SERIES_GROUND.into(),
                reason: format!(
                    "{} bytes is not a whole number of {n_ch}-channel f64 samples",
                    bytes.len()
                ),
            });
        }
    }
    runner::run(spec, series, opts, extras)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The villa example checked against scene-1.schema.json by W0-B3.
    const VILLA: &str = r#"{
 "schema": "fabrail.scene/1", "name": "villa_quake", "doc": "simulated replay", "seed": 42,
 "duration_s": 60, "sync_hz": 100, "variant": "warned",
 "building": {"config": "fps_isolated", "period_s": 0.35, "zeta": 0.05, "topo_factor": 1.2,
   "iso_period_s": 3.0, "mu": 0.05,
   "floors": [{"level": 0, "name": "ground", "z_m": 0.0}, {"level": 1, "name": "upper", "z_m": 3.4}]},
 "ground": {"source": "boore_ps", "series": "series:ground", "magnitude": 7.2, "r_epi_km": 60,
   "depth_km": 12, "origin_t_s": 0.0, "t_p_s": 10.2, "t_s_s": 17.5, "percentile": 50, "kappa_s": 0.04},
 "series": {"ground": {"axis": "time", "fs_hz": 500, "t0_s": 0, "channels": ["ax", "ay", "az"]}},
 "devices": [
  {"id": "drawer_1", "class": "drawer", "level": 0, "room": "kitchen", "heading_deg": 90,
   "fidelity": "carriage6", "hero": true,
   "contents": [{"kind": "glassware", "count": 12}, {"kind": "porcelain", "count": 6}],
   "initial_state": {"lev": "levitated", "x_m": 0.25},
   "commands": [{"t_s": 5.0, "cmd": "open", "source": "local"}]},
  {"id": "drawer_2", "class": "drawer", "level": 0, "fidelity": "travel1d", "policy": "seat_and_latch",
   "overrides": {"design.landing.seismic_latch.enabled": true, "design.landing.seismic_latch.x_retention_n": 200.0}},
  {"id": "slider_1", "class": "drawer", "preset": "cabinet_slider", "level": 0, "fidelity": "travel1d"},
  {"id": "pantry_door", "class": "door", "level": 0, "egress": "non_rated", "fidelity": "carriage6",
   "policy": "stop_and_land"},
  {"id": "salon_wall", "class": "wall", "level": 0, "fidelity": "travel1d", "policy": "land_and_lock"},
  {"id": "plinth", "class": "haven_r", "level": 0, "room": "gallery", "fidelity": "platform3dof",
   "contents": [{"kind": "custom", "name": "sculpture", "mu_s": 0.5, "b_over_h": 0.3, "mass_kg": 80}]}],
 "alert": {"provenance": "synthetic/injected, not a live EEW feed", "local_p": true,
   "feeds": [{"kind": "network", "t_s": 7.5, "id": 1, "magnitude": 7.2, "r_epi_km": 60, "depth_km": 12,
              "origin_t_s": 0.0, "mmi": 7}]},
 "power": {"p_bus_max_w": 1500, "ups": false, "mains_loss_t_s": null},
 "choreo": {"enable": true, "smoothing": false, "deadline_quantile": 0.05, "deadline_margin_s": 1.0},
 "operator": [{"t_s": 55, "action": "inspect_reset", "source": "local"},
              {"t_s": 58, "action": "restore", "source": "local", "devices": null}],
 "contents_mu_sweep_frac": 0.3,
 "record": {"rate_hz": 100, "devices": "all", "channels": "default"}
}"#;

    fn villa() -> Value {
        serde_json::from_str(VILLA).expect("villa example is JSON")
    }

    fn check(v: &Value) -> Result<SceneSpec, SceneError> {
        SceneSpec::from_json(&v.to_string())
    }

    #[test]
    fn villa_example_loads_and_round_trips() {
        let s = check(&villa()).expect("villa example validates");
        assert_eq!(s.devices.len(), 6);
        assert_eq!(s.chunk_ticks(), 200, "200-tick chunks at 100 Hz");
        assert_eq!(s.building.config_name(), "fps_isolated");
        assert!((s.building.height_m() - 3.4).abs() < 1e-12);
        let again = SceneSpec::from_json(&s.to_json()).expect("canonical JSON reloads");
        assert_eq!(again, s);
        // Series bytes are checked before anything runs.
        let bytes: BTreeMap<String, Vec<u8>> =
            [("ground".to_string(), vec![0u8; 24 * 10 + 8])].into();
        assert!(matches!(
            run_scene(&s, &bytes, &SceneRunOptions::default()),
            Err(SceneError::Series { .. })
        ));
        assert!(matches!(
            run_scene(&s, &BTreeMap::new(), &SceneRunOptions::default()),
            Err(SceneError::Series { .. })
        ));
    }

    #[test]
    fn scene_rules_reject_bad_documents() {
        type Edit = fn(&mut Value);
        let cases: [(&str, Edit); 17] = [
            ("egress lock", |v| {
                v["devices"][3]["policy"] = "land_and_lock".into()
            }),
            ("egress latch", |v| {
                v["devices"][3]["policy"] = "seat_and_latch".into()
            }),
            ("egress parking lock override", |v| {
                v["devices"][3]["overrides"] =
                    serde_json::json!({"design.landing.parking_lock": true})
            }),
            ("haven carriage6", |v| {
                v["devices"][5]["fidelity"] = "carriage6".into()
            }),
            ("drawer platform3dof", |v| {
                v["devices"][0]["fidelity"] = "platform3dof".into()
            }),
            ("latch on wall", |v| {
                v["devices"][4]["policy"] = "seat_and_latch".into()
            }),
            ("hero travel1d", |v| v["devices"][1]["hero"] = true.into()),
            ("bad id", |v| v["devices"][0]["id"] = "Drawer 1".into()),
            ("duplicate id", |v| {
                v["devices"][1]["id"] = "drawer_1".into()
            }),
            ("level names no floor", |v| {
                v["devices"][0]["level"] = 7.into()
            }),
            ("bad override path", |v| {
                v["devices"][1]["overrides"] = serde_json::json!({"landing.x": 1})
            }),
            ("custom without mu", |v| {
                v["devices"][5]["contents"][0]
                    .as_object_mut()
                    .map(|o| o.remove("mu_s"));
            }),
            ("provenance altered", |v| {
                v["alert"]["provenance"] = "live feed".into()
            }),
            ("feed local_p", |v| {
                v["alert"]["feeds"][0]["kind"] = "local_p".into()
            }),
            ("boore without series", |v| {
                v["ground"].as_object_mut().map(|o| o.remove("series"));
            }),
            ("fixed base with mu", |v| {
                v["building"] = serde_json::json!({"config": "fixed_base", "period_s": 0.3, "mu": 0.05,
                    "floors": [{"level": 0, "name": "g", "z_m": 0.0}]})
            }),
            ("unknown field", |v| v["surprise"] = 1.into()),
        ];
        for (name, edit) in cases {
            let mut v = villa();
            edit(&mut v);
            assert!(check(&v).is_err(), "{name}: must be rejected");
        }
        // Accepted variants.
        let mut q = villa();
        q["variant"] = "quiet".into();
        q["ground"] = serde_json::json!({"source": "none"});
        q["series"] = serde_json::json!({});
        assert!(check(&q).is_ok(), "quiet scene without ground motion");
        let mut s = villa();
        s["alert"]["feeds"][0]["spoofed"] = true.into();
        assert!(check(&s).is_ok());
        let mut t = villa();
        t["building"] = serde_json::json!({"config": "tower_sentinel", "period_s": 7.0,
            "floors": [{"level": 0, "name": "penthouse", "z_m": 380}], "hmd_mode": "seismic_passive"});
        assert!(check(&t).is_ok());
    }

    /// Enumerations in the Rust mirror equal the enums of scene-1.schema.json.
    #[test]
    fn scene_schema_enums_match() {
        let schema: Value = serde_json::from_str(include_str!(
            "../../../../python/fabrail/schemas/scene-1.schema.json"
        ))
        .expect("scene schema is JSON");
        let defs = &schema["$defs"];
        let names = |v: &Value| -> Vec<String> {
            v.as_array()
                .map(|a| {
                    a.iter()
                        .filter_map(|x| x.as_str().map(str::to_string))
                        .collect()
                })
                .unwrap_or_default()
        };
        let ser = |x: Value| x.as_str().unwrap_or_default().to_string();
        let dev = &defs["device"]["properties"];
        assert_eq!(
            names(&dev["policy"]["enum"]),
            ChoreoPolicy::ALL
                .iter()
                .map(|p| p.slug().to_string())
                .collect::<Vec<_>>()
        );
        let classes: Vec<String> = [
            DeviceClassSpec::Drawer,
            DeviceClassSpec::Door,
            DeviceClassSpec::Wall,
            DeviceClassSpec::Heavy,
            DeviceClassSpec::HavenR,
        ]
        .into_iter()
        .map(|c| ser(serde_json::to_value(c).unwrap_or_default()))
        .collect();
        assert_eq!(names(&dev["class"]["enum"]), classes);
        let fid: Vec<String> = [
            DeviceFidelity::Carriage6,
            DeviceFidelity::Travel1d,
            DeviceFidelity::Platform3dof,
        ]
        .into_iter()
        .map(|c| ser(serde_json::to_value(c).unwrap_or_default()))
        .collect();
        assert_eq!(names(&dev["fidelity"]["enum"]), fid);
        assert_eq!(
            names(&defs["contents_item"]["properties"]["kind"]["enum"]),
            ContentKind::ALL
                .iter()
                .map(|k| k.slug().to_string())
                .collect::<Vec<_>>()
        );
        let feeds: Vec<String> = AlertFeedKind::ALL
            .iter()
            .filter(|k| **k != AlertFeedKind::LocalP)
            .map(|k| ser(serde_json::to_value(k).unwrap_or_default()))
            .collect();
        assert_eq!(
            names(&defs["alert_feed"]["properties"]["kind"]["enum"]),
            feeds
        );
        let modes: Vec<String> = HmdMode::ALL
            .iter()
            .filter(|m| **m != HmdMode::Locked)
            .map(|m| m.slug().to_string())
            .collect();
        assert_eq!(
            names(&defs["building_tower_sentinel"]["properties"]["hmd_mode"]["enum"]),
            modes
        );
        let configs: Vec<String> = defs["building"]["oneOf"]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter_map(|r| r["$ref"].as_str())
                    .filter_map(|r| r.rsplit('/').next())
                    .map(|n| {
                        defs[n]["properties"]["config"]["const"]
                            .as_str()
                            .unwrap_or("")
                            .to_string()
                    })
                    .collect()
            })
            .unwrap_or_default();
        assert_eq!(
            configs,
            ["fixed_base", "fps_isolated", "haven_f", "tower_sentinel"]
        );
        assert_eq!(schema["properties"]["schema"]["const"], SCENE_SCHEMA);
        assert_eq!(
            defs["alert"]["properties"]["provenance"]["const"],
            SCENE_PROVENANCE
        );
        let sync: Vec<u64> = schema["properties"]["sync_hz"]["enum"]
            .as_array()
            .map(|a| a.iter().filter_map(Value::as_u64).collect())
            .unwrap_or_default();
        assert_eq!(
            sync,
            SYNC_HZ.iter().map(|x| u64::from(*x)).collect::<Vec<_>>()
        );
    }
}
