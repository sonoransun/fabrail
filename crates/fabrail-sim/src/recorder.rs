//! RunData, Channel, EventRec, RunMeta: uniform-rate structure-of-arrays channels plus events. Owner: A7 (types frozen by W0-B3).
//!
//! These types cross PyO3 unchanged: `RunData.channel(name)` returns the channel as
//! little-endian bytes (`"<f8"` or `"u1"`) plus its unit; events are
//! `(t_s, kind, a, b)` tuples; `meta_json()` is [`RunMeta::to_json`]. Sample k of every
//! channel is at t = k / rate_hz.
//!
//! [`default_channels`] freezes the default channel names (A7 may append, never
//! rename); `record.channels = "kpis_only"` records none. [`known_channels`] adds the
//! extra channels an explicit `record.channels` list may name.
//!
//! [`Recorder`] samples the selected channels at `record.rate_hz` (the scheduler calls
//! it at every `record_decimation()`-th base tick, before the plant step, so sample k
//! is the truth at t = k/rate_hz with the controller outputs of that tick). Truth
//! channels come from the plant, estimate channels (`x_meas_m`, `v_hat_mps`, …) from
//! the executive's latest published outputs, and the u8 channels are the wire values
//! of the state enums.

use std::collections::BTreeMap;

use fabrail_plant::Truth;
use fabrail_rt::ExecSnapshot;
use fabrail_rt::selftest::Fnv1a;
use fabrail_types::{Event, MAX_MAGNETS, fmath};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::scenario::{ChannelSelect, SimError};

/// Channel samples.
#[derive(Clone, Debug, PartialEq)]
pub enum ChannelData {
    /// f64 samples (numpy `"<f8"`).
    F64(Vec<f64>),
    /// u8 samples (numpy `"u1"`): enum states and flag bytes.
    U8(Vec<u8>),
}

impl ChannelData {
    /// Number of samples.
    pub fn len(&self) -> usize {
        match self {
            ChannelData::F64(v) => v.len(),
            ChannelData::U8(v) => v.len(),
        }
    }

    /// True when there are no samples.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// numpy dtype string: `"<f8"` or `"u1"`.
    pub fn dtype(&self) -> &'static str {
        match self {
            ChannelData::F64(_) => "<f8",
            ChannelData::U8(_) => "u1",
        }
    }

    /// Little-endian bytes (`np.frombuffer(b, dtype)` restores the samples).
    pub fn to_le_bytes(&self) -> Vec<u8> {
        match self {
            ChannelData::F64(v) => v.iter().flat_map(|x| x.to_le_bytes()).collect(),
            ChannelData::U8(v) => v.clone(),
        }
    }
}

/// One recorded channel.
#[derive(Clone, Debug, PartialEq)]
pub struct Channel {
    /// Channel name (unit-suffixed, e.g. `x_m`).
    pub name: String,
    /// Unit string (`m`, `m/s`, `A`, `""` for enum states).
    pub unit: String,
    /// Samples.
    pub data: ChannelData,
}

/// One discrete event in a run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EventRec {
    /// Time, s.
    pub t_s: f64,
    /// Kind: the core `EventKind` variant name (e.g. `"Landed"`) for controller events,
    /// `"scenario:<kind>"` for applied scenario events, `"plant:<what>"` for plant
    /// events (e.g. `"plant:bumper"`).
    pub kind: String,
    /// First payload value (meaning per kind).
    pub a: f64,
    /// Second payload value.
    pub b: f64,
}

impl EventRec {
    /// Converts a controller event (kind = `EventKind::name()`).
    pub fn from_core(e: &Event) -> EventRec {
        EventRec {
            t_s: e.tick.seconds_f64(),
            kind: e.kind.name().to_string(),
            a: e.a as f64,
            b: e.b as f64,
        }
    }
}

/// Run provenance.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RunMeta {
    /// fabrail version (`CARGO_PKG_VERSION`).
    pub version: String,
    /// Scenario name.
    pub scenario: String,
    /// Rust noise seed.
    pub noise_seed: u64,
    /// Plant tier name.
    pub fidelity: String,
    /// Coil mode (`"lag"` / `"voltage"`).
    pub coil: String,
    /// The parameter set as JSON.
    pub params: Value,
    /// The canonical scenario document.
    pub spec: Value,
    /// `fabrail_rt` GOLDEN_SELFTEST_HASH the run was built with.
    pub golden: u64,
    /// `"release"` or `"debug"`.
    pub build_profile: String,
    /// Wall-clock run time, s (excluded from [`RunMeta::hashed_json`]).
    pub wall_s: f64,
}

impl RunMeta {
    /// Full JSON (includes `wall_s`).
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }

    /// JSON without `wall_s`: the deterministic form hashed by determinism tests
    /// (plan §2b item 16).
    pub fn hashed_json(&self) -> String {
        let mut v = serde_json::to_value(self).unwrap_or(Value::Null);
        if let Some(o) = v.as_object_mut() {
            o.remove("wall_s");
        }
        serde_json::to_string(&v).unwrap_or_default()
    }
}

/// A finished run.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RunData {
    /// Channel sample rate, Hz.
    pub rate_hz: f64,
    /// Recorded channels (all the same length).
    pub channels: Vec<Channel>,
    /// Events in time order.
    pub events: Vec<EventRec>,
    /// KPIs by name (`metrics::KPI_NAMES`).
    pub kpis: BTreeMap<String, f64>,
    /// Provenance.
    pub meta: RunMeta,
}

impl RunData {
    /// Channel names in recording order.
    pub fn channel_names(&self) -> Vec<&str> {
        self.channels.iter().map(|c| c.name.as_str()).collect()
    }

    /// The channel called `name`.
    pub fn channel(&self, name: &str) -> Option<&Channel> {
        self.channels.iter().find(|c| c.name == name)
    }

    /// Samples per channel (0 without channels).
    pub fn n_samples(&self) -> usize {
        self.channels.first().map_or(0, |c| c.data.len())
    }

    /// FNV-1a 64 fingerprint of everything deterministic in the run: rate, every
    /// channel (name, unit, dtype, LE bytes), every event (time and payload by bit
    /// pattern), every KPI (by bit pattern) and [`RunMeta::hashed_json`] (which omits
    /// `wall_s`). Two runs are byte-identical iff their fingerprints agree (FAB-SW-002).
    pub fn fingerprint(&self) -> u64 {
        let mut h = Fnv1a::new();
        h.write_u64(self.rate_hz.to_bits());
        for c in &self.channels {
            h.write(c.name.as_bytes());
            h.write(&[0]);
            h.write(c.unit.as_bytes());
            h.write(&[0]);
            h.write(c.data.dtype().as_bytes());
            h.write(&c.data.to_le_bytes());
        }
        for e in &self.events {
            h.write_u64(e.t_s.to_bits());
            h.write(e.kind.as_bytes());
            h.write(&[0]);
            h.write_u64(e.a.to_bits());
            h.write_u64(e.b.to_bits());
        }
        for (k, v) in &self.kpis {
            h.write(k.as_bytes());
            h.write(&[0]);
            h.write_u64(v.to_bits());
        }
        h.write(self.meta.hashed_json().as_bytes());
        h.finish()
    }
}

/// Sample type of a channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ChannelKind {
    /// f64.
    F64,
    /// u8 (enum state / flags).
    U8,
}

/// A channel the recorder can produce.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct ChannelDef {
    /// Name.
    pub name: String,
    /// Unit.
    pub unit: &'static str,
    /// Sample type.
    pub kind: ChannelKind,
}

/// The default channel set for a carriage with `n_magnets` elements (frozen names;
/// truth first, then controller estimates, then u8 states).
pub fn default_channels(n_magnets: usize) -> Vec<ChannelDef> {
    let f = |name: &str, unit: &'static str| ChannelDef {
        name: name.to_string(),
        unit,
        kind: ChannelKind::F64,
    };
    let u = |name: &str| ChannelDef {
        name: name.to_string(),
        unit: "",
        kind: ChannelKind::U8,
    };
    let mut v = vec![
        f("x_m", "m"),
        f("v_mps", "m/s"),
        f("a_mps2", "m/s^2"),
        f("y_m", "m"),
        f("z_m", "m"),
        f("roll_rad", "rad"),
        f("pitch_rad", "rad"),
        f("yaw_rad", "rad"),
    ];
    let n = n_magnets.min(fabrail_types::MAX_MAGNETS);
    v.extend((0..n).map(|i| f(&format!("gap_{i}_m"), "m")));
    v.extend((0..n).map(|i| f(&format!("coil_{i}_a"), "A")));
    v.extend([
        f("iq_a", "A"),
        f("f_lsm_n", "N"),
        f("f_hand_n", "N"),
        f("f_obstacle_n", "N"),
        f("f_upper_n", "N"),
        f("f_rollers_n", "N"),
        f("floor_ax_mps2", "m/s^2"),
        f("floor_ay_mps2", "m/s^2"),
        f("floor_az_mps2", "m/s^2"),
        f("t_coil_c", "degC"),
        f("t_mag_c", "degC"),
        f("cavity_dp_pa", "Pa"),
        f("v_bus_v", "V"),
        f("v_hold_v", "V"),
        f("p_bus_w", "W"),
        f("x_meas_m", "m"),
        f("v_hat_mps", "m/s"),
        f("f_hand_hat_n", "N"),
        f("residual_n", "N"),
        f("m_hat_kg", "kg"),
        f("e_kin_gov_j", "J"),
        f("e_passivity_j", "J"),
        f("force_cap_n", "N"),
        u("lev_state"),
        u("seismic_state"),
        u("motion_mode"),
        u("demand"),
        u("level"),
        u("sat_flags"),
        u("intent"),
        u("obstruction"),
    ]);
    v
}

/// Extra channels (not recorded by default; name them in `record.channels`).
const EXTRA_CHANNELS: &[(&str, &str, ChannelKind)] = &[
    ("id_a", "A", ChannelKind::F64),
    ("i_a_a", "A", ChannelKind::F64),
    ("i_b_a", "A", ChannelKind::F64),
    ("i_c_a", "A", ChannelKind::F64),
    ("f_seat_n", "N", ChannelKind::F64),
    ("f_guide_n", "N", ChannelKind::F64),
    ("f_pads_n", "N", ChannelKind::F64),
    ("f_bumper_n", "N", ChannelKind::F64),
    ("f_end_stop_n", "N", ChannelKind::F64),
    ("f_lock_n", "N", ChannelKind::F64),
    ("f_latch_n", "N", ChannelKind::F64),
    ("f_tangential_n", "N", ChannelKind::F64),
    ("acc_abs_x_mps2", "m/s^2", ChannelKind::F64),
    ("acc_abs_y_mps2", "m/s^2", ChannelKind::F64),
    ("acc_abs_z_mps2", "m/s^2", ChannelKind::F64),
    ("omega_roll_radps", "rad/s", ChannelKind::F64),
    ("omega_pitch_radps", "rad/s", ChannelKind::F64),
    ("omega_yaw_radps", "rad/s", ChannelKind::F64),
    ("mass_kg", "kg", ChannelKind::F64),
    ("payload_kg", "kg", ChannelKind::F64),
    ("e_hold_j", "J", ChannelKind::F64),
    ("i_bus_a", "A", ChannelKind::F64),
    ("e_kin_j", "J", ChannelKind::F64),
    ("e_drag_j", "J", ChannelKind::F64),
    ("host_x_m", "m", ChannelKind::F64),
    ("host_v_mps", "m/s", ChannelKind::F64),
    ("host_a_mps2", "m/s^2", ChannelKind::F64),
    ("x_ref_m", "m", ChannelKind::F64),
    ("v_ref_mps", "m/s", ChannelKind::F64),
    ("a_hat_mps2", "m/s^2", ChannelKind::F64),
    ("a_floor_hat_mps2", "m/s^2", ChannelKind::F64),
    ("pd_m", "m", ChannelKind::F64),
    ("tau_c_s", "s", ChannelKind::F64),
    ("floor_v_hat_mps", "m/s", ChannelKind::F64),
    ("seismic_action", "", ChannelKind::U8),
    ("seat_status", "", ChannelKind::U8),
    ("lev_phase", "", ChannelKind::U8),
    ("contact_state", "", ChannelKind::U8),
    ("mains_ok", "", ChannelKind::U8),
    ("hand_active", "", ChannelKind::U8),
];

/// Every channel the recorder can produce: [`default_channels`] then the extras.
pub fn known_channels(n_magnets: usize) -> Vec<ChannelDef> {
    let mut v = default_channels(n_magnets);
    v.extend(EXTRA_CHANNELS.iter().map(|&(name, unit, kind)| ChannelDef {
        name: name.to_string(),
        unit,
        kind,
    }));
    v
}

/// Where a channel's samples come from (resolved once from its name).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Src {
    X,
    V,
    A,
    Y,
    Z,
    Roll,
    Pitch,
    Yaw,
    Gap(usize),
    Coil(usize),
    Iq,
    FLsm,
    FHand,
    FObstacle,
    FUpper,
    FRollers,
    Floor(usize),
    TCoil,
    TMag,
    CavityDp,
    VBus,
    VHold,
    PBus,
    XMeas,
    VHat,
    FHandHat,
    Residual,
    MHat,
    EKinGov,
    EPassivity,
    ForceCap,
    LevState,
    SeismicState,
    MotionMode,
    Demand,
    Level,
    SatFlags,
    Intent,
    Obstruction,
    Id,
    IAbc(usize),
    FSeat,
    FGuide,
    FPads,
    FBumper,
    FEndStop,
    FLock,
    FLatch,
    FTangential,
    AccAbs(usize),
    Omega(usize),
    MassKg,
    PayloadKg,
    EHold,
    IBus,
    EKin,
    EDrag,
    HostX,
    HostV,
    HostA,
    XRef,
    VRef,
    AHat,
    AFloorHat,
    PdM,
    TauC,
    FloorVHat,
    SeismicAction,
    SeatStatus,
    LevPhase,
    ContactState,
    MainsOk,
    HandActive,
}

/// Parses `gap_{i}_m` / `coil_{i}_a` style names.
fn indexed(name: &str, prefix: &str, suffix: &str, n: usize) -> Option<usize> {
    let i: usize = name
        .strip_prefix(prefix)?
        .strip_suffix(suffix)?
        .parse()
        .ok()?;
    (i < n.min(MAX_MAGNETS)).then_some(i)
}

fn resolve(name: &str, n_magnets: usize) -> Option<Src> {
    if let Some(i) = indexed(name, "gap_", "_m", n_magnets) {
        return Some(Src::Gap(i));
    }
    if let Some(i) = indexed(name, "coil_", "_a", n_magnets) {
        return Some(Src::Coil(i));
    }
    Some(match name {
        "x_m" => Src::X,
        "v_mps" => Src::V,
        "a_mps2" => Src::A,
        "y_m" => Src::Y,
        "z_m" => Src::Z,
        "roll_rad" => Src::Roll,
        "pitch_rad" => Src::Pitch,
        "yaw_rad" => Src::Yaw,
        "iq_a" => Src::Iq,
        "f_lsm_n" => Src::FLsm,
        "f_hand_n" => Src::FHand,
        "f_obstacle_n" => Src::FObstacle,
        "f_upper_n" => Src::FUpper,
        "f_rollers_n" => Src::FRollers,
        "floor_ax_mps2" => Src::Floor(0),
        "floor_ay_mps2" => Src::Floor(1),
        "floor_az_mps2" => Src::Floor(2),
        "t_coil_c" => Src::TCoil,
        "t_mag_c" => Src::TMag,
        "cavity_dp_pa" => Src::CavityDp,
        "v_bus_v" => Src::VBus,
        "v_hold_v" => Src::VHold,
        "p_bus_w" => Src::PBus,
        "x_meas_m" => Src::XMeas,
        "v_hat_mps" => Src::VHat,
        "f_hand_hat_n" => Src::FHandHat,
        "residual_n" => Src::Residual,
        "m_hat_kg" => Src::MHat,
        "e_kin_gov_j" => Src::EKinGov,
        "e_passivity_j" => Src::EPassivity,
        "force_cap_n" => Src::ForceCap,
        "lev_state" => Src::LevState,
        "seismic_state" => Src::SeismicState,
        "motion_mode" => Src::MotionMode,
        "demand" => Src::Demand,
        "level" => Src::Level,
        "sat_flags" => Src::SatFlags,
        "intent" => Src::Intent,
        "obstruction" => Src::Obstruction,
        "id_a" => Src::Id,
        "i_a_a" => Src::IAbc(0),
        "i_b_a" => Src::IAbc(1),
        "i_c_a" => Src::IAbc(2),
        "f_seat_n" => Src::FSeat,
        "f_guide_n" => Src::FGuide,
        "f_pads_n" => Src::FPads,
        "f_bumper_n" => Src::FBumper,
        "f_end_stop_n" => Src::FEndStop,
        "f_lock_n" => Src::FLock,
        "f_latch_n" => Src::FLatch,
        "f_tangential_n" => Src::FTangential,
        "acc_abs_x_mps2" => Src::AccAbs(0),
        "acc_abs_y_mps2" => Src::AccAbs(1),
        "acc_abs_z_mps2" => Src::AccAbs(2),
        "omega_roll_radps" => Src::Omega(0),
        "omega_pitch_radps" => Src::Omega(1),
        "omega_yaw_radps" => Src::Omega(2),
        "mass_kg" => Src::MassKg,
        "payload_kg" => Src::PayloadKg,
        "e_hold_j" => Src::EHold,
        "i_bus_a" => Src::IBus,
        "e_kin_j" => Src::EKin,
        "e_drag_j" => Src::EDrag,
        "host_x_m" => Src::HostX,
        "host_v_mps" => Src::HostV,
        "host_a_mps2" => Src::HostA,
        "x_ref_m" => Src::XRef,
        "v_ref_mps" => Src::VRef,
        "a_hat_mps2" => Src::AHat,
        "a_floor_hat_mps2" => Src::AFloorHat,
        "pd_m" => Src::PdM,
        "tau_c_s" => Src::TauC,
        "floor_v_hat_mps" => Src::FloorVHat,
        "seismic_action" => Src::SeismicAction,
        "seat_status" => Src::SeatStatus,
        "lev_phase" => Src::LevPhase,
        "contact_state" => Src::ContactState,
        "mains_ok" => Src::MainsOk,
        "hand_active" => Src::HandActive,
        _ => return None,
    })
}

/// Roll, pitch, yaw (Z-Y-X Tait–Bryan) of the unit quaternion [w, x, y, z], rad
/// (PHYSICS §5).
pub(crate) fn euler_zyx(pose: &[f64; 7]) -> [f64; 3] {
    let [_, _, _, w, x, y, z] = *pose;
    let roll = fmath::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let s = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0);
    let pitch = fmath::asin(s);
    let yaw = fmath::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    [roll, pitch, yaw]
}

/// Dissipated drag energy of the ledger (distributed eddy, rail eddy and hysteresis,
/// air, friction; PHYSICS §9), J.
pub(crate) fn drag_energy_j(t: &Truth) -> f64 {
    let e = &t.e_diss;
    e.dist_j + e.rail_eddy_j + e.rail_hyst_j + e.air_j + e.friction_j
}

/// What the recorder samples at one instant.
pub(crate) struct RecordView<'a> {
    /// Plant truth.
    pub truth: &'a Truth,
    /// Executive outputs.
    pub snap: &'a ExecSnapshot,
    /// A hand is in contact (scenario or live).
    pub hand_active: bool,
}

enum Sample {
    F(f64),
    U(u8),
}

fn sample(src: Src, v: &RecordView<'_>) -> Sample {
    use Sample::{F, U};
    let t = v.truth;
    let fs = &v.snap.fast.status;
    let sup = &v.snap.sup.status;
    let rep = &v.snap.seis.report;
    let c = &t.f_contact;
    match src {
        Src::X => F(t.x_m),
        Src::V => F(t.v_mps),
        Src::A => F(t.a_mps2),
        Src::Y => F(t.pose[1]),
        Src::Z => F(t.pose[2]),
        Src::Roll => F(euler_zyx(&t.pose)[0]),
        Src::Pitch => F(euler_zyx(&t.pose)[1]),
        Src::Yaw => F(euler_zyx(&t.pose)[2]),
        Src::Gap(i) => F(t.gaps_m.get(i).copied().unwrap_or(0.0)),
        Src::Coil(i) => F(t.coil_i_a.get(i).copied().unwrap_or(0.0)),
        Src::Iq => F(t.iq_a),
        Src::FLsm => F(t.f_lsm_n),
        Src::FHand => F(t.f_hand_n),
        Src::FObstacle => F(c.obstacle_n),
        Src::FUpper => F(c.upper_n),
        Src::FRollers => F(c.rollers_n),
        Src::Floor(i) => F(t.floor_acc_mps2.get(i).copied().unwrap_or(0.0)),
        Src::TCoil => F(t.temps_c[0]),
        Src::TMag => F(t.temps_c[1]),
        Src::CavityDp => F(t.cavity_dp_pa),
        Src::VBus => F(t.power.v_bus_v),
        Src::VHold => F(t.power.v_hold_v),
        Src::PBus => F(t.power.p_bus_w),
        Src::XMeas => F(fs.x.to_m_f64()),
        Src::VHat => F(fs.v_mps as f64),
        Src::FHandHat => F(fs.f_hand_hat_n as f64),
        Src::Residual => F(fs.residual_n as f64),
        Src::MHat => F(fs.m_hat_kg as f64),
        Src::EKinGov => F(fs.e_kin_gov_j as f64),
        Src::EPassivity => F(fs.e_passivity_j as f64),
        Src::ForceCap => F(v.snap.fast.verdict.force_cap_n as f64),
        Src::LevState => U(sup.lev.as_u8()),
        Src::SeismicState => U(rep.state.as_u8()),
        Src::MotionMode => U(sup.mode.as_u8()),
        Src::Demand => U(v.snap.fast.verdict.demand.as_u8()),
        Src::Level => U(rep.level.as_u8()),
        Src::SatFlags => U(fs.sat_flags),
        Src::Intent => U(fs.intent.as_u8()),
        Src::Obstruction => {
            U(fs.obstruction as u8 | (fs.obstruction_degraded as u8) << 1 | (fs.hand as u8) << 2)
        }
        Src::Id => F(t.id_a),
        Src::IAbc(i) => F(t.i_abc_a.get(i).copied().unwrap_or(0.0)),
        Src::FSeat => F(c.seat_n),
        Src::FGuide => F(c.guide_fin_n),
        Src::FPads => F(c.pads_n),
        Src::FBumper => F(c.bumper_n),
        Src::FEndStop => F(c.end_stop_n),
        Src::FLock => F(c.lock_n),
        Src::FLatch => F(c.latch_n),
        Src::FTangential => F(c.tangential_x_n),
        Src::AccAbs(i) => F(t.acc_abs_mps2.get(i).copied().unwrap_or(0.0)),
        Src::Omega(i) => F(t.omega_radps.get(i).copied().unwrap_or(0.0)),
        Src::MassKg => F(t.mass_kg),
        Src::PayloadKg => F(t.payload_kg),
        Src::EHold => F(t.power.e_hold_j),
        Src::IBus => F(t.power.i_bus_a),
        Src::EKin => F(t.e_diss.kinetic_j),
        Src::EDrag => F(drag_energy_j(t)),
        Src::HostX => F(t.host_x_m),
        Src::HostV => F(t.host_v_mps),
        Src::HostA => F(t.host_a_mps2),
        Src::XRef => F(fs.x_ref.to_m_f64()),
        Src::VRef => F(fs.v_ref_mps as f64),
        Src::AHat => F(fs.a_mps2 as f64),
        Src::AFloorHat => F(fs.a_floor_x_mps2 as f64),
        Src::PdM => F(rep.pd_m as f64),
        Src::TauC => F(rep.tau_c_s as f64),
        Src::FloorVHat => F(rep.floor_v_mps as f64),
        Src::SeismicAction => U(sup.action.as_u8()),
        Src::SeatStatus => U(fs.seat.as_u8()),
        Src::LevPhase => U(fs.lev.as_u8()),
        Src::ContactState => {
            let s = &t.state;
            U(s.levitated as u8
                | (s.seated as u8) << 1
                | (s.landed as u8) << 2
                | (s.upper_stop as u8) << 3
                | (s.locked as u8) << 4
                | (s.latched as u8) << 5)
        }
        Src::MainsOk => U(t.power.mains_ok as u8),
        Src::HandActive => U(v.hand_active as u8),
    }
}

/// Samples the selected channels into structure-of-arrays storage.
#[derive(Clone, Debug, Default)]
pub struct Recorder {
    srcs: Vec<Src>,
    channels: Vec<Channel>,
}

impl Recorder {
    /// A recorder for `select` on a carriage with `n_magnets` elements, reserving
    /// `capacity` samples per channel. An unknown name is `SimError::UnknownChannel`.
    pub fn new(
        select: &ChannelSelect,
        n_magnets: usize,
        capacity: usize,
    ) -> Result<Recorder, SimError> {
        let defs: Vec<ChannelDef> = match select {
            ChannelSelect::KpisOnly => Vec::new(),
            ChannelSelect::Default => default_channels(n_magnets),
            ChannelSelect::Names(names) => {
                let known = known_channels(n_magnets);
                names
                    .iter()
                    .map(|n| {
                        known
                            .iter()
                            .find(|d| &d.name == n)
                            .cloned()
                            .ok_or_else(|| SimError::UnknownChannel(n.clone()))
                    })
                    .collect::<Result<_, _>>()?
            }
        };
        let mut srcs = Vec::with_capacity(defs.len());
        let mut channels = Vec::with_capacity(defs.len());
        for d in defs {
            let src =
                resolve(&d.name, n_magnets).ok_or(SimError::UnknownChannel(d.name.clone()))?;
            srcs.push(src);
            channels.push(Channel {
                name: d.name,
                unit: d.unit.to_string(),
                data: match d.kind {
                    ChannelKind::F64 => ChannelData::F64(Vec::with_capacity(capacity)),
                    ChannelKind::U8 => ChannelData::U8(Vec::with_capacity(capacity)),
                },
            });
        }
        Ok(Recorder { srcs, channels })
    }

    /// Number of recorded channels.
    pub fn n_channels(&self) -> usize {
        self.channels.len()
    }

    /// Appends one sample of every channel.
    pub(crate) fn push(&mut self, v: &RecordView<'_>) {
        for (src, ch) in self.srcs.iter().zip(self.channels.iter_mut()) {
            match (sample(*src, v), &mut ch.data) {
                (Sample::F(x), ChannelData::F64(d)) => d.push(x),
                (Sample::U(x), ChannelData::U8(d)) => d.push(x),
                (Sample::F(x), ChannelData::U8(d)) => d.push(x as u8),
                (Sample::U(x), ChannelData::F64(d)) => d.push(x as f64),
            }
        }
    }

    /// The recorded channels.
    pub fn into_channels(self) -> Vec<Channel> {
        self.channels
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_types::{EventKind, Tick};

    #[test]
    fn channel_bytes_round_trip() {
        let c = ChannelData::F64(vec![1.5, -2.25]);
        assert_eq!(c.dtype(), "<f8");
        let b = c.to_le_bytes();
        assert_eq!(b.len(), 16);
        assert_eq!(
            f64::from_le_bytes(b[8..16].try_into().expect("8 bytes")),
            -2.25
        );
        assert_eq!(ChannelData::U8(vec![3, 4]).to_le_bytes(), vec![3, 4]);
    }

    #[test]
    fn default_channel_names_are_unique_and_suffixed() {
        let d = default_channels(4);
        let names: std::collections::BTreeSet<_> = d.iter().map(|c| c.name.clone()).collect();
        assert_eq!(names.len(), d.len());
        assert!(names.contains("gap_3_m") && !names.contains("gap_4_m"));
        for c in &d {
            if c.kind == ChannelKind::F64 {
                assert!(!c.unit.is_empty(), "{} needs a unit", c.name);
            }
        }
    }

    #[test]
    fn meta_hash_json_excludes_wall_time() {
        let mut m = RunMeta {
            version: "0.1.0".into(),
            wall_s: 1.25,
            ..RunMeta::default()
        };
        let a = m.hashed_json();
        m.wall_s = 99.0;
        assert_eq!(a, m.hashed_json());
        assert!(m.to_json().contains("wall_s") && !a.contains("wall_s"));
    }

    #[test]
    fn core_event_conversion() {
        let e = Event {
            tick: Tick(20_000),
            kind: EventKind::Landed,
            a: 0.004,
            b: 0.0,
        };
        let r = EventRec::from_core(&e);
        assert_eq!((r.t_s, r.kind.as_str()), (1.0, "Landed"));
    }

    #[test]
    fn every_known_channel_resolves_with_its_declared_type() {
        let known = known_channels(4);
        let names: std::collections::BTreeSet<_> = known.iter().map(|c| c.name.clone()).collect();
        assert_eq!(names.len(), known.len(), "unique names");
        let truth = Truth::default();
        let snap = ExecSnapshot::default();
        let view = RecordView {
            truth: &truth,
            snap: &snap,
            hand_active: true,
        };
        for d in &known {
            let src = resolve(&d.name, 4).unwrap_or_else(|| panic!("{} unresolved", d.name));
            let s = sample(src, &view);
            match (d.kind, s) {
                (ChannelKind::F64, Sample::F(_)) | (ChannelKind::U8, Sample::U(_)) => {}
                _ => panic!("{}: sample type differs from its definition", d.name),
            }
        }
        assert!(resolve("gap_4_m", 4).is_none(), "beyond n_magnets");
        assert!(resolve("gap_x_m", 4).is_none());
    }

    #[test]
    fn recorder_selects_and_samples() {
        let sel = ChannelSelect::Names(vec!["x_m".into(), "hand_active".into(), "roll_rad".into()]);
        let mut r = Recorder::new(&sel, 4, 2).expect("known names");
        assert_eq!(r.n_channels(), 3);
        let mut truth = Truth {
            x_m: 0.25,
            ..Truth::default()
        };
        // 0.1 rad roll: q = [cos(0.05), sin(0.05), 0, 0].
        truth.pose[3] = 0.05f64.cos();
        truth.pose[4] = 0.05f64.sin();
        let snap = ExecSnapshot::default();
        r.push(&RecordView {
            truth: &truth,
            snap: &snap,
            hand_active: true,
        });
        let ch = r.into_channels();
        assert_eq!(ch[0].data, ChannelData::F64(vec![0.25]));
        assert_eq!(ch[1].data, ChannelData::U8(vec![1]));
        let ChannelData::F64(roll) = &ch[2].data else {
            panic!("f64")
        };
        assert!(
            (roll[0] - 0.1).abs() < 1e-12,
            "(E) Z-Y-X roll of the quaternion"
        );
        assert!(matches!(
            Recorder::new(&ChannelSelect::Names(vec!["nope".into()]), 4, 1),
            Err(SimError::UnknownChannel(n)) if n == "nope"
        ));
        assert_eq!(
            Recorder::new(&ChannelSelect::KpisOnly, 4, 1)
                .expect("kpis only")
                .n_channels(),
            0
        );
        assert_eq!(
            Recorder::new(&ChannelSelect::Default, 4, 1)
                .expect("default")
                .n_channels(),
            default_channels(4).len()
        );
    }

    #[test]
    fn fingerprint_sees_every_part() {
        let base = RunData {
            rate_hz: 1000.0,
            channels: vec![Channel {
                name: "x_m".into(),
                unit: "m".into(),
                data: ChannelData::F64(vec![0.0, 1.0]),
            }],
            events: vec![EventRec {
                t_s: 0.5,
                kind: "Landed".into(),
                a: 0.0,
                b: 0.0,
            }],
            kpis: [("mu_eq".to_string(), 0.01)].into_iter().collect(),
            meta: RunMeta::default(),
        };
        let f0 = base.fingerprint();
        let mut d = base.clone();
        d.meta.wall_s = 12.0;
        assert_eq!(d.fingerprint(), f0, "wall_s is excluded");
        let mut d = base.clone();
        d.channels[0].data = ChannelData::F64(vec![0.0, -1.0]);
        assert_ne!(d.fingerprint(), f0);
        let mut d = base.clone();
        d.events[0].a = -0.0;
        assert_ne!(d.fingerprint(), f0, "bit patterns, not values");
        let mut d = base.clone();
        d.kpis.insert("mu_eq".into(), 0.02);
        assert_ne!(d.fingerprint(), f0);
    }
}
