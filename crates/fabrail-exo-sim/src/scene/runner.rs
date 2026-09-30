//! The building-scene loop (plan §4b): ground → building → devices, fusion, choreographer, contents, recording.
//!
//! Order of one run ([`run`]):
//! 1. The ground record (`series.ground`, channel-major LE f64, building frame) is
//!    resampled and run through the building model once (one-way coupling,
//!    [`super::building`]); every floor's absolute acceleration is kept at 1 kHz.
//! 2. Each device gets its parameter set (class preset + dotted overrides, coil mode
//!    from the scene as the single source, re-sealed and validated), its noise seed
//!    (`SeedSequence(seed).spawn(n)[i]`, [`super::seeds`]) and a `fabrail_sim::Session`
//!    (Carriage6 or Travel1d; Travel1d for every device in quick mode). The HAVEN-R
//!    plinth is the reduced `crate::platform::HavenR3Dof` driven passively (🧪).
//! 3. The loop advances in sync chunks (`BASE_TICK_HZ / sync_hz` ticks, 200 at 100 Hz).
//!    At each sync boundary, sequentially and in registry order: feeds and operator
//!    actions due, device commands, the house-level EEW fusion (`fabrail_exo::fusion`,
//!    fed with the devices' on-site detections), the choreographer (plan at the first
//!    PROTECT-level fused alert, re-plan when the confirmation or level rises), QUIET
//!    seats, obstruction (one attempt: `NotSecured`), secured/deadline bookkeeping and
//!    the one-at-a-time restore. Then every device steps to the next boundary — rayon
//!    across devices, each device independent — with the floor acceleration of its
//!    level rotated into its rail frame and its `PlanPolicy`; its contents advance in
//!    0.25 ms sub-steps. Results are gathered in registry order and the bus power is
//!    summed in index order, so the run does not depend on the worker count.
//! 4. Records, events (time order, then device order), KPIs, the plan/timeline/
//!    waterfall JSON and provenance make the [`SceneData`].
//!
//! Every alert is "synthetic/injected, not a live EEW feed". The building prediction the
//! choreographer plans against (per-device peak floor acceleration) is an **oracle**:
//! the peak of the scene's own building response; a deployed system would predict it
//! from the alert through a ground-motion model and the building transfer function. The
//! choreographer is not a rated safety function; the core `admit` filter applies to
//! every proposal.

use std::collections::BTreeMap;
use std::sync::OnceLock;
use std::time::Instant;

use fabrail_exo::choreo::plan::{
    BuildingPrediction, DeviceKinematics, HousePlan, NotSecuredReason, PlanOptions, PlanStatus,
    Plausibility, QuietAction, plan_with, quiet_action, replan,
};
use fabrail_exo::choreo::policy::{PlanPolicy, SmoothAdmissionInputs, gate_decision};
use fabrail_exo::choreo::registry::{
    ContentsSensitivity, DeviceConstraints, DeviceEntry, DeviceRegistry, MAX_DEVICES,
};
use fabrail_exo::choreo::restore::{CommandSource, RestorePlanner, RestoreRequest, check_restore};
use fabrail_exo::fusion::{Feed, Fusion, FusionCfg, LocalDetection, core_alert};
use fabrail_exo_safety::alert_gate::Confirmations;
use fabrail_exo_safety::{AnyClass, DeviceCondition, ExoClass, FusedAlert, SafetyRank};
use fabrail_params::{SystemParams, crc, derive, presets};
use fabrail_sim::recorder::{Channel, ChannelData, EventRec};
use fabrail_sim::scenario::{ChannelSelect, ScenarioSpec, decode_f64_le};
use fabrail_sim::session::Session;
use fabrail_types::{
    LevState, MotionMode, PosNm, RailClass, SafetyDemand, SeismicLevel, SeismicState, Tick,
    TravelDir, UserCommand, fmath,
};
use serde_json::{Value, json};

use super::building::{self, BUILDING_DT_S, BuildingInfo, BuildingResponse};
use super::contents::{ContentKind, DeviceContents, SWEEP_LABELS, SWEEP_POINTS, friction_cone};
use super::seeds::device_noise_seed;
use super::{
    BuildingConfig, CoilSpec, DeviceClassSpec, DeviceCmd, DeviceFidelity, DeviceRecord, DeviceSpec,
    GroundSource, OperatorKind, RecordDevices, SCENE_PROVENANCE, SceneData, SceneError, SceneEvent,
    SceneExtras, SceneMeta, SceneRunOptions, SceneSpec, invalid,
};
use crate::platform::{HavenR3Dof, PassiveVertical, PlatformCfg, guideway_radius_m};

/// Quick-mode window, s (plan §4b: Travel1d for every device, a 25 s window).
pub(crate) const QUICK_WINDOW_S: f64 = 25.0;

/// A restore step has arrived within this distance of its target, m.
const ARRIVE_TOL_M: f64 = 1.0e-3;

/// … and below this speed (the terminus arrival bound of FAB-HAP-016), m/s.
const ARRIVE_V_MPS: f64 = 1.0e-3;

/// A restore step that has not arrived after this long is abandoned (reported) and the
/// next device goes, s (design choice: longer than any class's full-travel open).
const RESTORE_STEP_TIMEOUT_S: f64 = 30.0;

/// Oracle-prediction label written into the plan JSON.
const PREDICTION_LABEL: &str = "oracle: peak floor response of this scene's own building run \
     (a deployed system would predict it from the alert through a ground-motion model and the \
     building transfer function)";

/// Channels recorded per device (`record.channels` names; f64 first, then u8 states).
pub(crate) const DEVICE_CHANNELS: [&str; 21] = [
    "x_m",
    "v_mps",
    "x_enc_m",
    "a_body_x_mps2",
    "a_body_y_mps2",
    "floor_ax_mps2",
    "floor_ay_mps2",
    "floor_az_mps2",
    "gap_m",
    "p_bus_w",
    "e_hold_j",
    "cone_util",
    "lev_state",
    "seismic_state",
    "motion_mode",
    "level",
    "demand",
    "obstruction",
    "proposal",
    "contents_sliding",
    "mains_ok",
];

/// Building channels besides `floor_<level>_{ax,ay,az}_mps2`.
pub(crate) const BUILDING_CHANNELS: [&str; 7] = [
    "t_s",
    "ground_ax_mps2",
    "ground_ay_mps2",
    "ground_az_mps2",
    "p_bus_w",
    "alert_level",
    "alert_state",
];

/// True when `name` is a channel this scene records.
fn known_channel(name: &str, levels: &[u32]) -> bool {
    DEVICE_CHANNELS.contains(&name)
        || BUILDING_CHANNELS.contains(&name)
        || levels.iter().any(|l| {
            ["ax", "ay", "az"]
                .iter()
                .any(|a| name == format!("floor_{l}_{a}_mps2"))
        })
}

/// Paths the scene does not re-seal after an override (a deliberate seal mismatch, as
/// `Params.set`).
const UNSEALED_PATHS: [&str; 6] = [
    "safety.k_f_copy_n_per_a",
    "safety.adc_a_per_code_copy",
    "safety.lsm_a_per_code_copy",
    "safety.nm_per_count_copy",
    "safety.presence_copy",
    "safety.crc",
];

// ---- Plinth (HAVEN-R reduced model, 🧪) ----------------------------------------------------

/// Villa art plinth mass, kg (plan §4b H-21: "1 t at 0.5 m/s is 125 J"; EXOTIC §X9).
const PLINTH_MASS_KG: f64 = 1000.0;
/// Plinth isolation period, s (HAVEN-R T 6 s, R = g·T²/4π² = 8.95 m; EXOTIC §1, §X9).
const PLINTH_PERIOD_S: f64 = 6.0;
/// Plinth design stroke and moat, m (\[UNCERTAIN\] villa-scale design choice: ±0.3 m
/// stroke inside a 0.35 m moat; the 50 t HAVEN-R preset has ±0.9 m; EXOTIC §X9).
const PLINTH_STROKE_M: f64 = 0.3;
const PLINTH_MOAT_M: f64 = 0.35;
/// Copper damper c/A, N·s/m per m² (EXOTIC §1 HAVEN-F: 1.48e5) and area for ζ ≈ 0.1 at
/// T 6 s (c = 2ζ·m·ω = 209 N·s/m; \[UNCERTAIN\] design choice; EXOTIC §X9).
const PLINTH_C_CU_PER_M2: f64 = 1.48e5;
const PLINTH_CU_AREA_M2: f64 = 1.41e-3;
/// FPS catcher friction (\[UNCERTAIN\], typical PTFE composite 0.05; EXOTIC §X9).
const PLINTH_FPS_MU: f64 = 0.05;

// ---- Small helpers -------------------------------------------------------------------------

/// Building-frame acceleration rotated into a rail frame whose +x points at `heading`
/// (counter-clockwise from building x): x along the rail, y cross-track (the same
/// rotation as `contents::floor_cross_track`), z unchanged.
fn to_device(a: [f64; 3], heading_rad: f64) -> [f64; 3] {
    let (s, c) = (fmath::sin(heading_rad), fmath::cos(heading_rad));
    [c * a[0] + s * a[1], -s * a[0] + c * a[1], a[2]]
}

fn t_of(k: u64, sync_hz: f64) -> f64 {
    k as f64 / sync_hz
}

/// First sync index at or after `t_s`.
fn due_index(t_s: f64, sync_hz: f64) -> u64 {
    if !(t_s.is_finite() && t_s > 0.0) {
        return 0;
    }
    (t_s * sync_hz - 1e-9).ceil().max(0.0) as u64
}

fn rail_class(c: DeviceClassSpec) -> Option<RailClass> {
    match c {
        DeviceClassSpec::Drawer => Some(RailClass::Drawer),
        DeviceClassSpec::Door => Some(RailClass::Door),
        DeviceClassSpec::Wall => Some(RailClass::Wall),
        DeviceClassSpec::Heavy => Some(RailClass::Heavy),
        DeviceClassSpec::HavenR => None,
    }
}

fn class_name(c: DeviceClassSpec) -> &'static str {
    match c {
        DeviceClassSpec::Drawer => "drawer",
        DeviceClassSpec::Door => "door",
        DeviceClassSpec::Wall => "wall",
        DeviceClassSpec::Heavy => "heavy",
        DeviceClassSpec::HavenR => "haven_r",
    }
}

/// `fabrail_rt::GOLDEN_SELFTEST_HASH`, read from a two-tick probe run of the DOOR preset
/// (fabrail-exo-sim does not depend on fabrail-rt; `RunMeta.golden` carries the
/// constant). 0 if the probe cannot run.
fn golden() -> u64 {
    static G: OnceLock<u64> = OnceLock::new();
    *G.get_or_init(|| {
        let mut p = presets::door();
        p.control.rates.coil_mode_voltage = false;
        crc::seal(&mut p);
        let s = r#"{"schema": "fabrail.scenario/1", "name": "golden_probe", "duration_s": 0.0001,
            "noise_seed": 0, "fidelity": {"model": "travel1d", "coil": "lag"}}"#;
        fabrail_sim::scenario::Scenario::from_json(s, &BTreeMap::new())
            .and_then(|sc| fabrail_sim::run(&p, &sc))
            .map_or(0, |d| d.meta.golden)
    })
}

// ---- Ground ------------------------------------------------------------------------------

/// The decoded ground record (building frame, before the topographic factor).
struct Ground {
    t0: f64,
    fs: f64,
    ch: [Option<Vec<f64>>; 3],
}

impl Ground {
    fn new(spec: &SceneSpec, series: &BTreeMap<String, Vec<u8>>) -> Result<Ground, SceneError> {
        let mut g = Ground {
            t0: 0.0,
            fs: 1.0,
            ch: [None, None, None],
        };
        if spec.ground.source == GroundSource::None {
            return Ok(g);
        }
        let Some(decl) = &spec.series.ground else {
            return Ok(g);
        };
        let bytes = series
            .get(super::SERIES_GROUND)
            .ok_or_else(|| SceneError::Series {
                name: super::SERIES_GROUND.into(),
                reason: "declared but no bytes were given".into(),
            })?;
        let data =
            decode_f64_le(bytes, decl.channels.len()).map_err(|reason| SceneError::Series {
                name: super::SERIES_GROUND.into(),
                reason,
            })?;
        g.t0 = decl.t0_s.unwrap_or(0.0);
        g.fs = decl.fs_hz;
        for (name, v) in decl.channels.iter().zip(data) {
            if v.iter().any(|x| !x.is_finite()) {
                return Err(SceneError::Series {
                    name: super::SERIES_GROUND.into(),
                    reason: format!("channel {name} has a non-finite sample"),
                });
            }
            let k = match name.as_str() {
                "ax" => 0,
                "ay" => 1,
                _ => 2,
            };
            g.ch[k] = Some(v);
        }
        Ok(g)
    }

    /// Linear interpolation; a time series reads 0 outside its span.
    fn at(&self, t_s: f64) -> [f64; 3] {
        let mut out = [0.0; 3];
        let x = (t_s - self.t0) * self.fs;
        if !(x.is_finite() && x >= 0.0) {
            return out;
        }
        let i = x.floor() as usize;
        let w = x - i as f64;
        for (k, ch) in self.ch.iter().enumerate() {
            let Some(v) = ch else {
                continue;
            };
            out[k] = match (v.get(i), v.get(i + 1)) {
                (Some(a), Some(b)) => a + w * (b - a),
                (Some(a), None) if w == 0.0 => *a,
                _ => 0.0,
            };
        }
        out
    }
}

// ---- Devices -----------------------------------------------------------------------------

/// Applies dotted overrides to the class preset, sets the coil mode, re-seals and
/// validates (the `Params.with_overrides` semantics).
fn device_params(i: usize, d: &DeviceSpec, rc: RailClass) -> Result<SystemParams, SceneError> {
    let base = presets::by_class(rc);
    let mut doc = serde_json::to_value(base).map_err(|e| SceneError::Json(e.to_string()))?;
    let mut reseal = true;
    for (path, v) in &d.overrides {
        let ptr = format!("/{}", path.replace('.', "/"));
        let at = format!("devices[{i}].overrides.{path}");
        let slot = doc
            .pointer_mut(&ptr)
            .ok_or_else(|| invalid(at.clone(), "unknown parameter"))?;
        let same = matches!(
            (&*slot, v),
            (Value::Number(_), Value::Number(_))
                | (Value::Bool(_), Value::Bool(_))
                | (Value::String(_), Value::String(_))
        );
        if !same {
            return Err(invalid(at, format!("expected a value like {slot}")));
        }
        *slot = v.clone();
        if UNSEALED_PATHS.contains(&path.as_str()) {
            reseal = false;
        }
    }
    let mut p: SystemParams = serde_json::from_value(doc)
        .map_err(|e| invalid(format!("devices[{i}].overrides"), e.to_string()))?;
    // Coil mode single source: the scene device (plan §2b item 16).
    p.control.rates.coil_mode_voltage = d.coil == CoilSpec::Voltage;
    if reseal {
        crc::seal(&mut p);
    }
    p.validate()
        .map_err(|e| invalid(format!("devices[{i}].params"), e.to_string()))?;
    Ok(p)
}

/// The device's Session scenario (no series: the floor comes from `step_to`).
fn device_scenario(
    spec: &SceneSpec,
    i: usize,
    d: &DeviceSpec,
    fidelity: DeviceFidelity,
    duration_s: f64,
    extra: &[Value],
) -> Result<ScenarioSpec, SceneError> {
    let seed = d
        .noise_seed
        .unwrap_or_else(|| device_noise_seed(spec.seed, i));
    let mut events = Vec::new();
    if let Some(pw) = &spec.power
        && !pw.ups
        && let Some(t) = pw.mains_loss_t_s
    {
        let mut e = json!({"kind": "power_loss", "t_s": t});
        if let Some(r) = pw.mains_restore_t_s.filter(|r| *r > t) {
            e["dur_s"] = json!(r - t);
        }
        events.push(e);
    }
    events.extend(extra.iter().cloned());
    events.sort_by(|a, b| {
        let t = |v: &Value| v.get("t_s").and_then(Value::as_f64).unwrap_or(0.0);
        t(a).total_cmp(&t(b))
    });
    let mut doc = json!({
        "schema": fabrail_sim::scenario::SCENARIO_SCHEMA,
        "name": format!("{}.{}", spec.name, d.id),
        "duration_s": duration_s + 1.0,
        "noise_seed": seed,
        "fidelity": {
            "model": if fidelity == DeviceFidelity::Carriage6 { "carriage6" } else { "travel1d" },
            "coil": if d.coil == CoilSpec::Voltage { "voltage" } else { "lag" },
        },
        "events": events,
    });
    if let Some(init) = &d.initial_state {
        doc["initial"] = serde_json::to_value(init).map_err(|e| SceneError::Json(e.to_string()))?;
    }
    ScenarioSpec::from_value(doc).map_err(SceneError::Sim)
}

fn plinth_cfg() -> PlatformCfg {
    PlatformCfg {
        m_kg: PLINTH_MASS_KG,
        j_z_kgm2: PLINTH_MASS_KG * 1.0 / 6.0,
        r_guideway_m: guideway_radius_m(PLINTH_PERIOD_S),
        stroke_m: PLINTH_STROKE_M,
        moat_m: PLINTH_MOAT_M,
        buffer_zone_m: PLINTH_MOAT_M - PLINTH_STROKE_M,
        c_cu_per_m2: PLINTH_C_CU_PER_M2,
        cu_area_m2: PLINTH_CU_AREA_M2,
        cu_mass_kg: 5.0,
        cu_dt_max_k: 50.0,
        fps_r_m: guideway_radius_m(PLINTH_PERIOD_S),
        fps_mu: PLINTH_FPS_MU,
        vertical: PassiveVertical::AirSpring,
    }
}

enum DevKind {
    Core {
        session: Box<Session>,
        params: Box<SystemParams>,
    },
    Plinth {
        platform: Box<HavenR3Dof>,
    },
}

/// Latest per-device state the scene logic reads (end of the last chunk).
#[derive(Clone, Copy, Debug, Default)]
struct DevState {
    lev: LevState,
    seismic: SeismicState,
    level: SeismicLevel,
    mode: MotionMode,
    demand: SafetyDemand,
    faulted: bool,
    x_enc: PosNm,
    m_hat_kg: f64,
    obstruction: bool,
    arrived: bool,
    p_bus_w: f64,
    e_hold_j: f64,
    mains_ok: bool,
    x_m: f64,
    v_mps: f64,
    floor_dev: [f64; 3],
    a_body: [f64; 2],
    gap_m: f64,
    locked: bool,
    latched: bool,
    mass_kg: f64,
    local: LocalDetection,
}

struct Dev {
    kind: DevKind,
    heading: f64,
    floor: usize,
    x_closed_m: f64,
    travel_m: f64,
    cmd: Option<UserCommand>,
    policy: PlanPolicy,
    st: DevState,
    contents: DeviceContents,
    events: Vec<EventRec>,
    err: Option<SceneError>,
}

impl Dev {
    /// Advances this device over [t0, t1] (see the module docs).
    fn step(&mut self, t0: f64, t1: f64, tick1: Tick, b: &BuildingResponse) {
        if self.err.is_some() {
            return;
        }
        let (heading, floor) = (self.heading, self.floor);
        let fd = to_device(b.floor_at(floor, t1), heading);
        let dt = t1 - t0;
        match &mut self.kind {
            DevKind::Core { session, .. } => {
                let v0 = session.truth().v_mps;
                let cmd = self.cmd.take();
                if let Err(e) = session.step_to_with_policy(tick1, &fd, cmd, &mut self.policy) {
                    self.err = Some(SceneError::Sim(e));
                    return;
                }
                let tr = *session.truth();
                // Mean relative acceleration over the chunk (∫a dt exact).
                let a_rel = if dt > 0.0 { (tr.v_mps - v0) / dt } else { 0.0 };
                self.events.extend(session.drain_events());
                self.contents.advance(t0, dt, |t| {
                    let f = to_device(b.floor_at(floor, t), heading);
                    ([f[0] + a_rel, f[1]], f[2])
                });
                let s = session.status();
                let n_gaps = tr.gaps_m.iter().filter(|g| **g > 0.0).count();
                self.st = DevState {
                    lev: s.lev,
                    seismic: s.seismic,
                    level: s.level,
                    mode: s.mode,
                    demand: s.demand,
                    faulted: !s.faults.is_empty(),
                    x_enc: s.x,
                    m_hat_kg: f64::from(s.m_hat_kg),
                    obstruction: s.obstruction,
                    arrived: s.arrived,
                    p_bus_w: s.p_bus_w,
                    e_hold_j: s.e_hold_j,
                    mains_ok: tr.power.mains_ok,
                    x_m: tr.x_m,
                    v_mps: tr.v_mps,
                    floor_dev: fd,
                    a_body: [fd[0] + a_rel, fd[1]],
                    gap_m: if n_gaps > 0 {
                        tr.gaps_m.iter().filter(|g| **g > 0.0).sum::<f64>() / n_gaps as f64
                    } else {
                        0.0
                    },
                    locked: tr.state.locked,
                    latched: tr.state.latched,
                    mass_kg: tr.mass_kg,
                    local: LocalDetection::from_report(&session.snapshot().seis.report),
                };
            }
            DevKind::Plinth { platform } => {
                let n = (dt / BUILDING_DT_S).round().max(1.0) as usize;
                let h = dt / n as f64;
                let mut rel = Vec::with_capacity(n);
                for s in 0..n {
                    let t = t0 + (s as f64 + 1.0) * h;
                    let f = b.floor_at(floor, t);
                    let v_prev = [platform.state.q[3], platform.state.q[4]];
                    platform.step([f[0], f[1]], [0.0; 3], h);
                    let q = platform.state.q;
                    rel.push([(q[3] - v_prev[0]) / h, (q[4] - v_prev[1]) / h]);
                }
                let rel_at = |t: f64| -> [f64; 2] {
                    let k = (((t - t0) / h).ceil() as usize).clamp(1, n) - 1;
                    rel.get(k).copied().unwrap_or([0.0; 2])
                };
                self.contents.advance(t0, dt, |t| {
                    let f = b.floor_at(floor, t);
                    let r = rel_at(t);
                    let a = to_device([f[0] + r[0], f[1] + r[1], f[2]], heading);
                    ([a[0], a[1]], a[2])
                });
                let q = platform.state.q;
                let r = rel.last().copied().unwrap_or([0.0; 2]);
                let fb = b.floor_at(floor, t1);
                let abs = to_device([fb[0] + r[0], fb[1] + r[1], fb[2]], heading);
                let xd = to_device([q[0], q[1], 0.0], heading);
                let vd = to_device([q[3], q[4], 0.0], heading);
                self.st = DevState {
                    lev: if platform.state.landed {
                        LevState::Landed
                    } else {
                        LevState::LevitatedIdle
                    },
                    x_m: xd[0],
                    v_mps: vd[0],
                    x_enc: PosNm::from_m_f64(xd[0]),
                    arrived: true,
                    mains_ok: true,
                    floor_dev: fd,
                    a_body: [abs[0], abs[1]],
                    mass_kg: PLINTH_MASS_KG,
                    ..DevState::default()
                };
            }
        }
    }

    fn params(&self) -> Option<&SystemParams> {
        match &self.kind {
            DevKind::Core { params, .. } => Some(params),
            DevKind::Plinth { .. } => None,
        }
    }

    fn push_alert(&mut self, a: fabrail_types::NetworkAlert) {
        if let DevKind::Core { session, .. } = &mut self.kind {
            session.push_alert(a);
        }
    }
}

/// Physical condition as the choreographer sees it.
fn condition(st: &DevState, x_closed_m: f64, tol_m: f64, plinth: bool) -> DeviceCondition {
    if plinth {
        return DeviceCondition::Hold;
    }
    let closed = st.x_m - x_closed_m <= tol_m;
    match st.lev {
        LevState::Seated => DeviceCondition::Seated,
        LevState::Landed | LevState::Manual | LevState::Fault | LevState::Boot => {
            if st.locked || st.latched {
                DeviceCondition::LandedLocked
            } else {
                DeviceCondition::Landed
            }
        }
        LevState::LevitatedIdle | LevState::Seismic | LevState::SelfTest => {
            if closed {
                DeviceCondition::ClosedLevitated
            } else if st.mode == MotionMode::Freeze {
                DeviceCondition::FrozenOpen
            } else if st.mode == MotionMode::InertialSmooth {
                DeviceCondition::Standoff
            } else if st.v_mps.abs() > 1.0e-3 {
                DeviceCondition::Moving
            } else {
                DeviceCondition::Hold
            }
        }
        _ => DeviceCondition::Moving,
    }
}

// ---- Smooth ride-through admission inputs (research tier) ----------------------------------

/// Skyhook-isolated drawer response to its floor x record (PHYSICS §16.2): relative
/// stroke and speed peaks and the peak absolute acceleration, integrated at 1 ms from
/// `from_s` on, for the admission list of plan §4b.
fn skyhook_prediction(
    b: &BuildingResponse,
    floor: usize,
    heading: f64,
    f_c_hz: f64,
    zeta: f64,
    from_s: f64,
) -> (f64, f64, f64, f64, f64) {
    let w = 2.0 * std::f64::consts::PI * f_c_hz;
    let (mut x, mut v, mut vf) = (0.0f64, 0.0f64, 0.0f64);
    let (mut s_max, mut v_max, mut a_abs, mut ay, mut az) =
        (0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64);
    let k0 = (from_s / BUILDING_DT_S).max(0.0) as usize;
    let dt = BUILDING_DT_S;
    let n = b.acc.get(floor).map_or(0, Vec::len);
    for k in k0..n {
        let f = to_device(
            b.acc
                .get(floor)
                .and_then(|v| v.get(k))
                .copied()
                .unwrap_or([0.0; 3]),
            heading,
        );
        // Relative motion x of the isolated body: ẍ + 2ζω(ẋ + v_floor) + ω²x = −a_floor.
        let acc = -f[0] - 2.0 * zeta * w * (v + vf) - w * w * x;
        v += dt * acc;
        x += dt * v;
        vf += dt * f[0];
        s_max = s_max.max(x.abs());
        v_max = v_max.max(v.abs());
        a_abs = a_abs.max((f[0] + acc).abs());
        ay = ay.max(f[1].abs());
        az = az.max(f[2].abs());
    }
    (s_max, v_max, a_abs, ay, az)
}

// ---- The run -----------------------------------------------------------------------------

/// Runs a validated scene (see the module docs).
pub(crate) fn run(
    spec: &SceneSpec,
    series: &BTreeMap<String, Vec<u8>>,
    opts: &SceneRunOptions,
    extras: &SceneExtras,
) -> Result<SceneData, SceneError> {
    #[cfg(feature = "parallel")]
    if let Some(w) = opts.workers.filter(|w| *w >= 1)
        && let Ok(pool) = rayon::ThreadPoolBuilder::new().num_threads(w).build()
    {
        return pool.install(|| run_inner(spec, series, opts, extras));
    }
    run_inner(spec, series, opts, extras)
}

fn step_all(devs: &mut [Dev], t0: f64, t1: f64, tick1: Tick, b: &BuildingResponse) {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        devs.par_iter_mut().for_each(|d| d.step(t0, t1, tick1, b));
    }
    #[cfg(not(feature = "parallel"))]
    for d in devs.iter_mut() {
        d.step(t0, t1, tick1, b);
    }
}

/// Channel recorder of one device.
#[derive(Default)]
struct Rec {
    f: Vec<(String, &'static str, Vec<f64>)>,
    u: Vec<(String, Vec<u8>)>,
}

impl Rec {
    fn push_f(&mut self, i: usize, name: &str, unit: &'static str, v: f64) {
        if let Some(c) = self.f.get_mut(i) {
            c.2.push(v);
        } else {
            self.f.push((name.to_string(), unit, vec![v]));
        }
    }
    fn push_u(&mut self, i: usize, name: &str, v: u8) {
        if let Some(c) = self.u.get_mut(i) {
            c.1.push(v);
        } else {
            self.u.push((name.to_string(), vec![v]));
        }
    }
    fn into_channels(self, select: &ChannelSelect) -> Vec<Channel> {
        let keep = |n: &str| match select {
            ChannelSelect::Default => true,
            ChannelSelect::KpisOnly => false,
            ChannelSelect::Names(v) => v.iter().any(|x| x == n),
        };
        let mut out: Vec<Channel> = self
            .f
            .into_iter()
            .filter(|c| keep(&c.0))
            .map(|(name, unit, v)| Channel {
                name,
                unit: unit.to_string(),
                data: ChannelData::F64(v),
            })
            .collect();
        out.extend(
            self.u
                .into_iter()
                .filter(|c| keep(&c.0))
                .map(|(name, v)| Channel {
                    name,
                    unit: String::new(),
                    data: ChannelData::U8(v),
                }),
        );
        out
    }
}

/// Per-device bookkeeping of the choreographed run.
#[derive(Clone, Debug, Default)]
struct Track {
    secured_s: Option<f64>,
    not_secured: Option<String>,
    quiet_done: bool,
    quiet: Option<QuietAction>,
    user_motion: Option<(TravelDir, u64)>,
    opened_episode: bool,
    drawer_opened: u32,
    peak_abs_axy: f64,
    peak_v: f64,
    p_peak_w: f64,
    e_hold_min_j: f64,
    tripped: bool,
    pfa: [f64; 3],
    /// First tick a planned motion was passed to the device confirmed (`choreo:go`).
    go_s: Option<f64>,
    /// First tick a planned motion was held for want of device confirmation.
    wait_s: Option<f64>,
}

#[allow(clippy::too_many_lines)]
fn run_inner(
    spec: &SceneSpec,
    series: &BTreeMap<String, Vec<u8>>,
    opts: &SceneRunOptions,
    extras: &SceneExtras,
) -> Result<SceneData, SceneError> {
    let wall = Instant::now();
    if let ChannelSelect::Names(names) = &spec.record.channels {
        let levels: Vec<u32> = spec.building.floors().iter().map(|f| f.level).collect();
        if let Some((k, bad)) = names
            .iter()
            .enumerate()
            .find(|(_, n)| !known_channel(n, &levels))
        {
            return Err(invalid(
                format!("record.channels[{k}]"),
                format!("unknown channel \"{bad}\""),
            ));
        }
    }
    let sync = f64::from(spec.sync_hz);
    let chunk = spec.chunk_ticks().max(1);
    let t_window = if opts.quick {
        spec.duration_s.min(QUICK_WINDOW_S)
    } else {
        spec.duration_s
    };
    let n_chunks = (t_window * sync).round().max(1.0) as u64;
    let t_end = t_of(n_chunks, sync);

    // 1. Ground and building (one-way coupling).
    let ground = Ground::new(spec, series)?;
    let bresp = building::respond(&spec.building, |t| ground.at(t), t_end);

    // 2. Devices.
    let n = spec.devices.len();
    let mut devs: Vec<Dev> = Vec::with_capacity(n);
    let mut tracks: Vec<Track> = vec![Track::default(); n];
    for (i, d) in spec.devices.iter().enumerate() {
        let floor = bresp
            .floor_index(d.level)
            .ok_or_else(|| invalid(format!("devices[{i}].level"), "names no floor"))?;
        let heading = d.heading_deg.to_radians();
        let contents = DeviceContents::new(&d.contents, spec.contents_mu_sweep_frac);
        let fidelity = if opts.quick {
            DeviceFidelity::Travel1d
        } else {
            d.fidelity
        };
        let (kind, x_closed_m, travel_m) = match rail_class(d.class) {
            Some(rc) => {
                let p = device_params(i, d, rc)?;
                // Loader rule: an explicit seat_and_latch needs a fitted latch; its capacity
                // x_retention ≥ m_rated·a_design is checked against the predicted PFA_x by
                // the choreographer (SeatAndHold + `drawer_opened` reports otherwise).
                if d.policy == fabrail_exo::choreo::policy::ChoreoPolicy::SeatAndLatch
                    && !p.design.landing.seismic_latch.enabled
                {
                    return Err(invalid(
                        format!("devices[{i}].policy"),
                        "seat_and_latch needs design.landing.seismic_latch.enabled",
                    ));
                }
                let extra = extras
                    .device_events
                    .get(&d.id)
                    .map_or(&[][..], Vec::as_slice);
                let sc = device_scenario(spec, i, d, fidelity, t_end, extra)?;
                let session = Session::new_device(&p, &sc, i as u16).map_err(SceneError::Sim)?;
                let (xc, tr) = (p.design.rail.x_closed_m, p.design.rail.travel_m);
                (
                    DevKind::Core {
                        session: Box::new(session),
                        params: Box::new(p),
                    },
                    xc,
                    tr,
                )
            }
            None => {
                let platform = HavenR3Dof::new(&plinth_cfg())
                    .map_err(|e| invalid(format!("devices[{i}]"), format!("plinth: {e:?}")))?;
                (
                    DevKind::Plinth {
                        platform: Box::new(platform),
                    },
                    0.0,
                    PLINTH_STROKE_M,
                )
            }
        };
        // Oracle prediction: this floor's peak (device frame) over the run.
        let mut pfa = [0.0f64; 3];
        for a in bresp.acc.get(floor).map_or(&[][..], Vec::as_slice) {
            let v = to_device(*a, heading);
            for k in 0..3 {
                pfa[k] = pfa[k].max(v[k].abs());
            }
        }
        tracks[i].pfa = pfa;
        tracks[i].e_hold_min_j = f64::INFINITY;
        let mut dev = Dev {
            kind,
            heading,
            floor,
            x_closed_m,
            travel_m,
            cmd: None,
            policy: PlanPolicy::default(),
            st: DevState::default(),
            contents,
            events: Vec::new(),
            err: None,
        };
        // Initial state from the session (tick 0).
        if let DevKind::Core { session, .. } = &dev.kind {
            let s = session.status();
            let tr = session.truth();
            dev.st = DevState {
                lev: s.lev,
                seismic: s.seismic,
                mode: s.mode,
                x_enc: s.x,
                m_hat_kg: f64::from(s.m_hat_kg),
                arrived: s.arrived,
                p_bus_w: s.p_bus_w,
                e_hold_j: s.e_hold_j,
                mains_ok: tr.power.mains_ok,
                x_m: tr.x_m,
                v_mps: tr.v_mps,
                locked: tr.state.locked,
                latched: tr.state.latched,
                mass_kg: tr.mass_kg,
                ..DevState::default()
            };
        } else {
            dev.st.lev = LevState::LevitatedIdle;
            dev.st.mass_kg = PLINTH_MASS_KG;
        }
        devs.push(dev);
    }

    // Fusion (house level).
    let base_cfg = devs
        .iter()
        .find_map(Dev::params)
        .map_or_else(FusionCfg::default, FusionCfg::from_params);
    let mut fusion = Fusion::new(FusionCfg {
        v_p_kmps: spec.alert.vp_kmps as f32,
        v_s_kmps: spec.alert.vs_kmps as f32,
        s_eta_sigma_s: spec.alert.s_eta_sigma_s as f32,
        ..base_cfg
    });

    // Recording.
    let decim = ((sync / spec.record.rate_hz).round() as u64).max(1);
    let rate_hz = sync / decim as f64;
    let record_dev: Vec<bool> = spec
        .devices
        .iter()
        .map(|d| match &spec.record.devices {
            RecordDevices::All => true,
            RecordDevices::List(v) => v.contains(&d.id),
        })
        .collect();
    let mut recs: Vec<Rec> = (0..n).map(|_| Rec::default()).collect();
    let mut brec = Rec::default();

    // Scene state.
    let mut scene_events: Vec<(f64, Option<usize>, EventRec)> = Vec::new();
    let mut ev = |t: f64, dev: Option<usize>, kind: &str, a: f64, b: f64| {
        scene_events.push((
            t,
            dev,
            EventRec {
                t_s: t,
                kind: kind.to_string(),
                a,
                b,
            },
        ));
    };
    let mut hp: Option<HousePlan> = None;
    let mut hp_done: Option<HousePlan> = None;
    let mut plan_t: Option<f64> = None;
    let mut replans = 0u32;
    let mut first_fa: Option<FusedAlert> = None;
    let mut last_fa: Option<FusedAlert> = None;
    let mut last_indep = false;
    let mut last_pushed: Option<(u32, SeismicLevel, bool)> = None;
    let mut restore = RestorePlanner::new();
    let mut restore_active: Option<CommandSource> = None;
    let mut restore_current: Option<(usize, PosNm, u64)> = None;
    let mut inspect_reset_done = false;
    let mut deadline_checked = false;
    let mut p_bus_peak = 0.0f64;
    let mut over_budget_s = 0.0f64;
    let mut over_episode = false;
    let budget = spec.power.as_ref().map(|p| p.p_bus_max_w);
    let mut feed_done = vec![false; spec.alert.feeds.len()];
    let mut cancel_done = vec![false; spec.alert.feeds.len()];
    let mut op_done = vec![false; spec.operator.len()];
    let mut cmd_done: Vec<Vec<bool>> = spec
        .devices
        .iter()
        .map(|d| vec![false; d.commands.len()])
        .collect();
    let mut mains_lost = false;
    let mut stale_local = vec![false; n];

    let record = |k: u64,
                  devs: &[Dev],
                  recs: &mut [Rec],
                  brec: &mut Rec,
                  fa: Option<&FusedAlert>,
                  fstate: SeismicState,
                  p_bus: f64| {
        let t = t_of(k, sync);
        let mut j = 0usize;
        brec.push_f(j, "t_s", "s", t);
        j += 1;
        let g = bresp.ground_at(t);
        for (c, name) in ["ground_ax_mps2", "ground_ay_mps2", "ground_az_mps2"]
            .iter()
            .enumerate()
        {
            brec.push_f(j, name, "m/s^2", g[c]);
            j += 1;
        }
        for (f, lvl) in bresp.levels.iter().enumerate() {
            let a = bresp.floor_at(f, t);
            for (c, ax) in ["ax", "ay", "az"].iter().enumerate() {
                brec.push_f(j, &format!("floor_{lvl}_{ax}_mps2"), "m/s^2", a[c]);
                j += 1;
            }
        }
        brec.push_f(j, "p_bus_w", "W", p_bus);
        brec.push_u(0, "alert_level", fa.map_or(0, |a| a.level.as_u8()));
        brec.push_u(1, "alert_state", fstate.as_u8());
        for (i, d) in devs.iter().enumerate() {
            if !record_dev.get(i).copied().unwrap_or(false) {
                continue;
            }
            let r = &mut recs[i];
            let s = &d.st;
            let fields: [(&str, &'static str, f64); 12] = [
                ("x_m", "m", s.x_m),
                ("v_mps", "m/s", s.v_mps),
                ("x_enc_m", "m", s.x_enc.to_m_f64()),
                ("a_body_x_mps2", "m/s^2", s.a_body[0]),
                ("a_body_y_mps2", "m/s^2", s.a_body[1]),
                ("floor_ax_mps2", "m/s^2", s.floor_dev[0]),
                ("floor_ay_mps2", "m/s^2", s.floor_dev[1]),
                ("floor_az_mps2", "m/s^2", s.floor_dev[2]),
                ("gap_m", "m", s.gap_m),
                ("p_bus_w", "W", s.p_bus_w),
                ("e_hold_j", "J", s.e_hold_j),
                ("cone_util", "", {
                    // Vector friction cone of the least-gripping nominal item right now:
                    // |a_h|/(μ_s·(g + a_z)) with a_h = (drawer a_x, floor a_y) (plan §4b).
                    let mu = spec.devices[i]
                        .contents
                        .iter()
                        .filter_map(|c| c.mu_s())
                        .fold(f64::INFINITY, f64::min);
                    let g_eff =
                        friction_cone(0.0, [0.0, 0.0], s.floor_dev[2], 0.0, 1.0, None).g_eff_mps2;
                    if !mu.is_finite() {
                        0.0
                    } else if g_eff > 0.0 {
                        fmath::hypot(s.a_body[0], s.a_body[1]) / (mu * g_eff)
                    } else {
                        f64::INFINITY
                    }
                }),
            ];
            for (c, (name, unit, v)) in fields.iter().enumerate() {
                r.push_f(c, name, unit, *v);
            }
            let flags: [(&str, u8); 9] = [
                ("lev_state", s.lev.as_u8()),
                ("seismic_state", s.seismic.as_u8()),
                ("motion_mode", s.mode.as_u8()),
                ("level", s.level.as_u8()),
                ("demand", s.demand.as_u8()),
                ("obstruction", u8::from(s.obstruction)),
                (
                    "proposal",
                    d.policy.decision.map_or(255, |p| p.action.as_u8()),
                ),
                ("contents_sliding", d.contents.sliding_now().min(255) as u8),
                ("mains_ok", u8::from(s.mains_ok)),
            ];
            for (c, (name, v)) in flags.iter().enumerate() {
                r.push_u(c, name, *v);
            }
        }
    };
    let sum_bus = |devs: &[Dev]| devs.iter().fold(0.0f64, |acc, d| acc + d.st.p_bus_w);
    record(
        0,
        &devs,
        &mut recs,
        &mut brec,
        None,
        SeismicState::Normal,
        sum_bus(&devs),
    );

    let choreo = &spec.choreo;
    for k in 0..n_chunks {
        let t = t_of(k, sync);
        let tick = Tick(k * chunk);

        // Mains (reported; the loss itself is a scenario event in every device).
        if let Some(pw) = &spec.power {
            let lost = pw
                .mains_loss_t_s
                .is_some_and(|l| t >= l && pw.mains_restore_t_s.is_none_or(|r| t < r));
            if lost != mains_lost {
                ev(
                    t,
                    None,
                    if lost {
                        "power:mains_loss"
                    } else {
                        "power:mains_restore"
                    },
                    f64::from(u8::from(pw.ups)),
                    0.0,
                );
                mains_lost = lost;
            }
        }

        // Feeds and cancellations due.
        for (fi, f) in spec.alert.feeds.iter().enumerate() {
            if !feed_done[fi] && due_index(f.t_s, sync) <= k {
                feed_done[fi] = true;
                let feed = Feed {
                    kind: f.kind,
                    id: f.id,
                    received: tick,
                    origin_offset_s: (f.origin_t_s.or(spec.ground.origin_t_s).unwrap_or(t) - t)
                        as f32,
                    magnitude: f.magnitude.or(spec.ground.magnitude).unwrap_or(0.0) as f32,
                    r_epi_km: f.r_epi_km.or(spec.ground.r_epi_km).unwrap_or(0.0) as f32,
                    depth_km: f.depth_km.or(spec.ground.depth_km).unwrap_or(0.0) as f32,
                    mmi: f.mmi.unwrap_or(0.0) as f32,
                    cancel: false,
                    injected: f.spoofed,
                };
                let ok = fusion.push(&feed);
                ev(
                    t,
                    None,
                    "alert:feed",
                    f64::from(f.id),
                    f64::from(u8::from(ok)),
                );
            }
            if !cancel_done[fi]
                && let Some(c) = f.cancel_t_s
                && due_index(c, sync) <= k
            {
                cancel_done[fi] = true;
                let _ = fusion.push(&Feed {
                    kind: f.kind,
                    id: f.id,
                    received: tick,
                    cancel: true,
                    ..Feed::default()
                });
                ev(t, None, "alert:feed_cancel", f64::from(f.id), 0.0);
            }
        }

        // Operator actions due.
        for (oi, op) in spec.operator.iter().enumerate() {
            if op_done[oi] || due_index(op.t_s, sync) > k {
                continue;
            }
            op_done[oi] = true;
            let targets: Vec<usize> = match &op.devices {
                None => (0..n).collect(),
                Some(ids) => ids
                    .iter()
                    .filter_map(|id| spec.devices.iter().position(|d| &d.id == id))
                    .collect(),
            };
            let src = f64::from(op.source as u8);
            match op.action {
                OperatorKind::InspectReset => {
                    if op.source == CommandSource::Remote {
                        ev(t, None, "operator:inspect_reset_rejected", src, 0.0);
                        continue;
                    }
                    // The core applies the manual reset only in INSPECT (it is ignored in
                    // every other state, `SeismicUnit::reset_inspect`): the scene accepts it
                    // only when every targeted core device is in INSPECT (or NORMAL).
                    let in_inspect = targets.iter().all(|&i| {
                        devs[i].params().is_none()
                            || matches!(
                                devs[i].st.seismic,
                                SeismicState::Inspect | SeismicState::Normal
                            )
                    });
                    if !in_inspect {
                        ev(t, None, "operator:inspect_reset_rejected", src, 1.0);
                        continue;
                    }
                    for &i in &targets {
                        devs[i].cmd = Some(UserCommand::Reset);
                        // Its on-site detection belongs to the finished event until the
                        // device reports NORMAL again.
                        stale_local[i] = true;
                    }
                    fusion.reset();
                    inspect_reset_done = true;
                    // The event is over: the plan is kept for the report only, and a new
                    // event plans afresh.
                    if let Some(p) = hp.take() {
                        hp_done = Some(p);
                    }
                    last_fa = None;
                    last_pushed = None;
                    last_indep = false;
                    for tr in tracks.iter_mut() {
                        tr.quiet = None;
                    }
                    ev(t, None, "operator:inspect_reset", src, targets.len() as f64);
                }
                OperatorKind::Restore => {
                    let req = restore_request(&devs, &targets, op.source, inspect_reset_done);
                    match check_restore(&req) {
                        Ok(()) => {
                            restore_active = Some(op.source);
                            ev(t, None, "operator:restore", src, restore.pending() as f64);
                        }
                        Err(r) => ev(t, None, "restore:rejected", f64::from(r as u8), src),
                    }
                }
                OperatorKind::CancelAlert => {
                    for f in &spec.alert.feeds {
                        let _ = fusion.push(&Feed {
                            kind: f.kind,
                            id: f.id,
                            received: tick,
                            cancel: true,
                            ..Feed::default()
                        });
                    }
                    ev(t, None, "operator:cancel_alert", src, 0.0);
                }
            }
        }

        // Device user commands due (quantised to the sync grid).
        for (i, d) in spec.devices.iter().enumerate() {
            for (ci, c) in d.commands.iter().enumerate() {
                if cmd_done[i][ci] || due_index(c.t_s, sync) > k {
                    continue;
                }
                cmd_done[i][ci] = true;
                let x_now = devs[i].st.x_m;
                let (uc, dir) = match c.cmd {
                    DeviceCmd::Open => (UserCommand::Open, Some(TravelDir::Open)),
                    DeviceCmd::Close => (UserCommand::Close, Some(TravelDir::Close)),
                    DeviceCmd::GoTo => {
                        let x = c.x_m.unwrap_or(x_now);
                        let dir = if x > x_now {
                            TravelDir::Open
                        } else {
                            TravelDir::Close
                        };
                        (UserCommand::GoTo(PosNm::from_m_f64(x)), Some(dir))
                    }
                    DeviceCmd::Stop => (UserCommand::Stop, None),
                    DeviceCmd::Lift => (UserCommand::Lift, None),
                    DeviceCmd::Land => (UserCommand::Land, None),
                };
                devs[i].cmd = Some(uc);
                tracks[i].user_motion = dir.map(|dd| (dd, k));
                ev(
                    t,
                    Some(i),
                    "user:command",
                    f64::from(c.cmd as u8),
                    f64::from(c.source as u8),
                );
            }
        }

        // House-level fusion with the on-site detections (earliest pick, largest level).
        for (i, d) in devs.iter().enumerate() {
            if stale_local[i] && d.st.seismic == SeismicState::Normal {
                stale_local[i] = false;
            }
        }
        let local = if spec.alert.local_p {
            devs.iter().enumerate().fold(LocalDetection::default(), |acc, (i, d)| {
                if stale_local[i] {
                    return acc;
                }
                let l = d.st.local;
                LocalDetection {
                    p_pick: match (acc.p_pick, l.p_pick) {
                        (Some(a), Some(b)) => Some(a.min(b)),
                        (a, b) => a.or(b),
                    },
                    level: acc.level.max(l.level),
                    pd_m: acc.pd_m.max(l.pd_m),
                    tau_c_s: if l.pd_m > acc.pd_m {
                        l.tau_c_s
                    } else {
                        acc.tau_c_s
                    },
                    pga_mps2: acc.pga_mps2.max(l.pga_mps2),
                }
            })
        } else {
            LocalDetection::default()
        };
        if let Some(fa) = fusion.step(tick, &local) {
            if fa.alert.cancel {
                if hp.is_some() || last_fa.is_some() {
                    ev(t, None, "alert:cancel", f64::from(fa.alert.id), 0.0);
                }
                if let Some(na) = core_alert(&fa) {
                    for d in devs.iter_mut() {
                        d.push_alert(na);
                    }
                }
                hp = None;
                last_pushed = None;
                last_fa = None;
                last_indep = false;
            } else {
                if last_fa.is_none_or(|l| l.level != fa.level) {
                    ev(
                        t,
                        None,
                        "alert:fused",
                        f64::from(fa.level.as_u8()),
                        f64::from(fa.p_damaging),
                    );
                }
                if let Some(na) = core_alert(&fa) {
                    let key = (na.id, fa.level, fa.local_p);
                    if last_pushed != Some(key) {
                        for d in devs.iter_mut() {
                            d.push_alert(na);
                        }
                        last_pushed = Some(key);
                    }
                }
                if first_fa.is_none() {
                    first_fa = Some(fa);
                }
                let indep = Confirmations::from_alert(&fa, false, true, true).independent();
                let level_up = last_fa.is_some_and(|l| fa.level > l.level);
                if choreo.enable && fa.level >= SeismicLevel::L2 {
                    let (reg, popts, pred) = planning_inputs(spec, &devs, &tracks, &bresp, tick, t);
                    let new = match &hp {
                        None => Some(plan_with(&fa, &reg, &pred, &popts)),
                        Some(prev) if (indep && !last_indep) || level_up => {
                            replans += 1;
                            Some(replan(prev, &fa, &reg, &pred, &popts))
                        }
                        Some(_) => None,
                    };
                    if let Some(p) = new {
                        let first = hp.is_none();
                        if first {
                            plan_t = Some(t);
                        }
                        let (mut n_ok, mut n_ns) = (0u32, 0u32);
                        for (i, a) in p.as_slice().iter().enumerate() {
                            let changed = hp.as_ref().and_then(|h| h.get(i)) != Some(a);
                            if a.status == PlanStatus::Planned {
                                n_ok += 1;
                            } else {
                                n_ns += 1;
                            }
                            if !changed {
                                continue;
                            }
                            if a.status == PlanStatus::Planned
                                && a.decision.target.is_some()
                                && restore.remembered(i).is_none()
                            {
                                restore.remember(i, PosNm::from_m_f64(devs[i].st.x_m));
                            }
                            ev(
                                t,
                                Some(i),
                                if a.status == PlanStatus::Planned {
                                    "choreo:planned"
                                } else {
                                    "choreo:not_secured"
                                },
                                a.start.seconds_f64(),
                                a.reason
                                    .map_or(a.finish.seconds_f64(), |r| f64::from(r as u8)),
                            );
                            tracks[i].not_secured = a.reason.map(|r| r.slug().to_string());
                        }
                        ev(
                            t,
                            None,
                            if first {
                                "choreo:plan"
                            } else {
                                "choreo:replan"
                            },
                            f64::from(n_ok),
                            f64::from(n_ns),
                        );
                        hp = Some(p);
                    }
                }
                last_indep = indep;
                last_fa = Some(fa);
            }
        }

        // Plan bookkeeping: obstruction (one attempt), secured, deadline, QUIET.
        if let Some(p) = hp.as_mut() {
            for (i, d) in devs.iter().enumerate() {
                let Some(a) = p.get(i).copied() else {
                    continue;
                };
                let tol = d
                    .params()
                    .map_or(1.0e-3, |p| p.control.motion.pos_tol_closed_m);
                let cond = condition(&d.st, d.x_closed_m, tol, d.params().is_none());
                let egress = spec.devices[i].egress;
                if a.status == PlanStatus::Planned
                    && a.decision.moves()
                    && tick >= a.start
                    && tracks[i].secured_s.is_none()
                    && d.st.obstruction
                {
                    p.mark_not_secured(i, NotSecuredReason::Obstruction);
                    tracks[i].not_secured = Some(NotSecuredReason::Obstruction.slug().to_string());
                    ev(
                        t,
                        Some(i),
                        "choreo:not_secured",
                        f64::from(NotSecuredReason::Obstruction as u8),
                        t,
                    );
                    continue;
                }
                if a.status == PlanStatus::Planned
                    && tracks[i].secured_s.is_none()
                    && tick >= a.start
                    && SafetyRank::of(cond, egress) >= a.target_rank
                    && a.decision.target.is_none_or(|tg| {
                        (d.st.x_m - tg.to_m_f64()).abs() <= tol.max(ARRIVE_TOL_M)
                            || cond == DeviceCondition::Seated
                    })
                {
                    tracks[i].secured_s = Some(t);
                    let slack = p.deadline.map_or(f64::NAN, |dl| dl.seconds_f64() - t);
                    ev(
                        t,
                        Some(i),
                        "choreo:secured",
                        t - a.start.seconds_f64(),
                        slack,
                    );
                }
            }
            if !deadline_checked && p.deadline.is_some_and(|dl| tick >= dl) {
                deadline_checked = true;
                for (i, tr) in tracks.iter_mut().enumerate().take(n) {
                    let Some(a) = p.get(i) else {
                        continue;
                    };
                    if a.status == PlanStatus::Planned && tr.secured_s.is_none() {
                        tr.not_secured = Some("deadline_missed".into());
                        ev(t, Some(i), "choreo:deadline_missed", t, 0.0);
                    }
                }
            }
            // QUIET: a drawer left away from its seat seats after the plausibility check.
            for (i, d) in devs.iter().enumerate() {
                if tracks[i].quiet_done || d.st.seismic != SeismicState::Quiet {
                    continue;
                }
                tracks[i].quiet_done = true;
                let Some(prm) = d.params() else {
                    continue;
                };
                let e = entry(spec, i, d, &tracks[i]);
                let plaus = Plausibility {
                    x: d.st.x_enc,
                    x_min: PosNm::from_m_f64(d.x_closed_m),
                    x_max: PosNm::from_m_f64(d.x_closed_m + d.travel_m),
                    tol_m: prm.control.motion.pos_tol_travel_m.max(1e-3) as f32,
                    gap_m: d.st.gap_m as f32,
                    gap_lo_m: prm.safety.gap_min_m as f32,
                    gap_hi_m: prm.safety.gap_max_m as f32,
                    sensor_fault: d.st.faulted,
                };
                let mut q = quiet_action(&e, plaus.ok());
                // The QUIET seat is a choreographer command too: the alert gate decides.
                if let (QuietAction::Seat { .. }, Some(fa)) =
                    (q, last_fa.as_ref().or(first_fa.as_ref()))
                    && let Some(dec) = q.decision()
                    && !gate_decision(fa, &e, &dec, SafetyRank::SeatedOrLocked, false).is_allowed()
                {
                    ev(t, Some(i), "choreo:quiet_gate_denied", d.st.x_m, 0.0);
                    q = QuietAction::LandAndFlag;
                }
                match q {
                    QuietAction::Seat { target, v_cap_mps } => {
                        ev(
                            t,
                            Some(i),
                            "choreo:quiet_seat",
                            target.to_m_f64(),
                            f64::from(v_cap_mps),
                        );
                    }
                    QuietAction::LandAndFlag => {
                        ev(t, Some(i), "choreo:manual_push_to_seat", d.st.x_m, 0.0);
                    }
                    QuietAction::None => {}
                }
                if q != QuietAction::None {
                    tracks[i].quiet = Some(q);
                }
            }
        }

        // Restore: one device at a time after the previous one arrived.
        if let Some(src) = restore_active {
            let mut timed_out = None;
            let arrived = restore_current.is_none_or(|(i, tg, k0)| {
                let d = &devs[i];
                // At the target and at rest (the core's `arrived` flag clears when the
                // carriage returns to idle admittance after the terminus).
                let there = k > k0
                    && (d.st.x_m - tg.to_m_f64()).abs() <= ARRIVE_TOL_M
                    && d.st.v_mps.abs() <= ARRIVE_V_MPS;
                if !there && t_of(k - k0, sync) > RESTORE_STEP_TIMEOUT_S {
                    timed_out = Some(i);
                }
                there || timed_out.is_some()
            });
            if let Some(i) = timed_out {
                ev(t, Some(i), "restore:timeout", devs[i].st.x_m, 0.0);
            }
            if arrived {
                let targets: Vec<usize> = (0..n).collect();
                let req = restore_request(&devs, &targets, src, inspect_reset_done);
                let reg = registry(spec, &devs, &tracks);
                match restore.next_step(&req, &reg, true) {
                    Some(s) => {
                        let i = usize::from(s.device);
                        devs[i].cmd = Some(UserCommand::GoTo(s.target));
                        // A local, user-initiated opening: never reversed by an alert.
                        tracks[i].user_motion = Some((TravelDir::Open, k));
                        restore_current = Some((i, s.target, k));
                        ev(
                            t,
                            Some(i),
                            "restore:step",
                            s.target.to_m_f64(),
                            f64::from(s.v_cap_mps),
                        );
                    }
                    None => {
                        if check_restore(&req).is_ok() {
                            restore_active = None;
                            restore_current = None;
                            ev(t, None, "restore:done", 0.0, 0.0);
                        }
                    }
                }
            }
        }

        // User motion ends when the device reports arrival after the command.
        for (i, d) in devs.iter().enumerate() {
            if let Some((_, k0)) = tracks[i].user_motion
                && k > k0
                && d.st.arrived
            {
                tracks[i].user_motion = None;
            }
        }

        // Policies for the next chunk. A planned motion the device core cannot yet
        // confirm (its own local P pick) is held (FREEZE) until its latest start
        // (deadline − planned duration), then the core class action stands
        // (`PlanPolicy` docs); the release and wait ticks carry over between chunks.
        for (i, d) in devs.iter_mut().enumerate() {
            let (released, waited) = (d.policy.released, d.policy.waited);
            let mut pol = if !choreo.enable {
                PlanPolicy::default()
            } else if let Some(q) = tracks[i].quiet {
                PlanPolicy::new(q.decision())
            } else {
                let a = hp.as_ref().and_then(|p| p.get(i).copied());
                let wait = hp.as_ref().and_then(|p| p.deadline).zip(a).and_then(|(dl, a)| {
                    (a.status == PlanStatus::Planned && a.decision.moves())
                        .then(|| Tick(dl.0.saturating_sub(a.finish.0.saturating_sub(a.start.0))))
                });
                PlanPolicy::with_wait(hp.as_ref().and_then(|p| p.proposal(i, tick)), wait)
            };
            pol.released = released;
            pol.waited = waited;
            d.policy = pol;
        }

        // Step every device to the next sync boundary.
        let t1 = t_of(k + 1, sync);
        step_all(&mut devs, t, t1, Tick((k + 1) * chunk), &bresp);
        for d in &mut devs {
            if let Some(e) = d.err.take() {
                return Err(e);
            }
        }

        // Device-level confirmation bookkeeping (the policy's release/wait ticks).
        for (i, d) in devs.iter().enumerate() {
            if tracks[i].wait_s.is_none()
                && let Some(w) = d.policy.waited
            {
                tracks[i].wait_s = Some(w.seconds_f64());
                ev(w.seconds_f64(), Some(i), "choreo:await_confirmation", w.seconds_f64(), 0.0);
            }
            if tracks[i].go_s.is_none()
                && let Some(r) = d.policy.released
            {
                tracks[i].go_s = Some(r.seconds_f64());
                let planned = hp
                    .as_ref()
                    .and_then(|p| p.get(i))
                    .map_or(f64::NAN, |a| a.start.seconds_f64());
                ev(r.seconds_f64(), Some(i), "choreo:go", r.seconds_f64(), planned);
            }
        }

        // Gather in index order.
        let p_bus = sum_bus(&devs);
        p_bus_peak = p_bus_peak.max(p_bus);
        if let Some(b) = budget {
            let over = p_bus > b;
            if over {
                over_budget_s += 1.0 / sync;
            }
            if over && !over_episode {
                ev(t1, None, "bus:over_budget", p_bus, b);
            }
            over_episode = over;
        }
        for (i, d) in devs.iter().enumerate() {
            let tr = &mut tracks[i];
            let s = &d.st;
            tr.peak_abs_axy = tr.peak_abs_axy.max(s.a_body[0].hypot(s.a_body[1]));
            tr.peak_v = tr.peak_v.max(s.v_mps.abs());
            tr.p_peak_w = tr.p_peak_w.max(s.p_bus_w);
            if d.params().is_some() {
                tr.e_hold_min_j = tr.e_hold_min_j.min(s.e_hold_j);
            }
            tr.tripped |= s.demand >= SafetyDemand::SafeStop;
            // Seated drawer x-retention (plan §4b): latch retention if engaged, else the
            // LSM continuous thrust holding position (the seat is a z-captive registration,
            // not an x-latch).
            if let Some(p) = d.params()
                && spec.devices[i].class == DeviceClassSpec::Drawer
                && s.lev == LevState::Seated
            {
                let ret = if s.latched || p.design.landing.seismic_latch.enabled {
                    p.design.landing.seismic_latch.x_retention_n
                } else {
                    p.design.lsm.f_cont_n
                };
                let demand = s.mass_kg * s.floor_dev[0].abs();
                let over = demand > ret;
                if over && !tr.opened_episode {
                    tr.drawer_opened += 1;
                    ev(t1, Some(i), "drawer_opened", demand, ret);
                }
                tr.opened_episode = over;
            }
        }
        if (k + 1) % decim == 0 {
            record(
                k + 1,
                &devs,
                &mut recs,
                &mut brec,
                last_fa.as_ref(),
                fusion.state(),
                p_bus,
            );
        }
    }

    // ---- Results -------------------------------------------------------------------------
    finish(FinishInputs {
        spec,
        opts,
        devs,
        tracks,
        recs,
        brec,
        bresp,
        scene_events,
        hp: hp.or(hp_done),
        plan_t,
        replans,
        first_fa,
        rate_hz,
        t_end,
        p_bus_peak,
        over_budget_s,
        wall_s: wall.elapsed().as_secs_f64(),
    })
}

/// A restore request for devices `targets` (the worst state among them).
fn restore_request(
    devs: &[Dev],
    targets: &[usize],
    source: CommandSource,
    reset: bool,
) -> RestoreRequest {
    let seis = targets
        .iter()
        .filter_map(|&i| devs.get(i))
        .filter(|d| d.params().is_some())
        .map(|d| d.st.seismic)
        .find(|s| *s != SeismicState::Normal)
        .unwrap_or(SeismicState::Normal);
    let selftest_ok = targets
        .iter()
        .filter_map(|&i| devs.get(i))
        .all(|d| !d.st.faulted && d.st.lev != LevState::Fault);
    RestoreRequest {
        source,
        seis,
        inspect_reset_done: reset,
        selftest_ok,
    }
}

/// Registry entry of device `i` from its latest state.
fn entry(spec: &SceneSpec, i: usize, d: &Dev, tr: &Track) -> DeviceEntry {
    let ds = &spec.devices[i];
    let class = match rail_class(ds.class) {
        Some(rc) => AnyClass::Core(rc),
        None => AnyClass::Exo(ExoClass::HavenR),
    };
    let contents = ds
        .contents
        .iter()
        .fold(ContentsSensitivity::None, |acc, c| {
            let s = match c.kind {
                ContentKind::Stemware => ContentsSensitivity::Stemware,
                ContentKind::Glassware | ContentKind::Porcelain | ContentKind::Custom => {
                    ContentsSensitivity::Fragile
                }
                ContentKind::Lined => ContentsSensitivity::Low,
            };
            if (s as u8) > (acc as u8) { s } else { acc }
        });
    let (constraints, tol, m_rated) = match d.params() {
        Some(p) => {
            let sf = &p.safety;
            let m_gov = sf.mass_rated_kg.max(1.05 * d.st.m_hat_kg);
            let v_cap = sf.v_close_max_mps;
            let v_u = derive::unattended_close_speed(
                v_cap,
                sf.unattended.f_pinch_n,
                m_gov,
                sf.unattended.k_series_n_per_m,
            );
            let mo = &p.control.motion;
            // Motion power at the unattended speed during the acceleration phase
            // (PHYSICS §7, §14): mechanical F·v plus LSM copper loss 1.5·R·(F/K_f)², with
            // F = m_gov·a_max; hover and electronics are the steady baseline.
            let l = &p.design.lsm;
            let f = m_gov * mo.a_max_mps2;
            let iq = if l.k_f_n_per_a > 0.0 {
                f / l.k_f_n_per_a
            } else {
                0.0
            };
            let p_move = f * v_u + 1.5 * l.r_phase_ohm * iq * iq;
            (
                DeviceConstraints {
                    v_cap_mps: v_cap as f32,
                    v_unattended_mps: v_u as f32,
                    a_brk_mps2: (mo.a_brk_ratio * mo.a_max_mps2) as f32,
                    p_move_w: p_move as f32,
                    s_min_m: mo.inertial.s_min_base_m as f32,
                    latch_fitted: p.design.landing.seismic_latch.enabled,
                    latch_retention_n: p.design.landing.seismic_latch.x_retention_n as f32,
                },
                mo.pos_tol_closed_m,
                m_gov,
            )
        }
        None => (DeviceConstraints::default(), 1.0e-3, PLINTH_MASS_KG),
    };
    DeviceEntry {
        class,
        egress: ds.egress,
        level: ds.level.min(255) as u8,
        heading_rad: d.heading as f32,
        position_m: ds
            .position_m
            .map_or([0.0; 2], |p| [p[0] as f32, p[1] as f32]),
        condition: condition(&d.st, d.x_closed_m, tol, d.params().is_none()),
        x: PosNm::from_m_f64(d.st.x_m),
        x_closed: PosNm::from_m_f64(d.x_closed_m),
        v_mps: d.st.v_mps as f32,
        m_gov_kg: m_rated as f32,
        user_motion: tr.user_motion.map(|(dir, _)| dir),
        obstruction_armed: d.params().is_some(),
        hero: ds.hero,
        policy: ds.policy,
        contents,
        constraints,
    }
}

fn registry(spec: &SceneSpec, devs: &[Dev], tracks: &[Track]) -> DeviceRegistry {
    let mut reg = DeviceRegistry::new();
    for (i, d) in devs.iter().enumerate().take(MAX_DEVICES) {
        let _ = reg.push(entry(spec, i, d, &tracks[i]));
    }
    reg
}

/// Registry, plan options and building prediction at tick `tick`.
fn planning_inputs(
    spec: &SceneSpec,
    devs: &[Dev],
    tracks: &[Track],
    b: &BuildingResponse,
    tick: Tick,
    t: f64,
) -> (DeviceRegistry, PlanOptions, BuildingPrediction) {
    let reg = registry(spec, devs, tracks);
    let mut opts = PlanOptions {
        smoothing: spec.choreo.smoothing,
        unattended_close: spec.choreo.unattended_close,
        ..PlanOptions::default()
    };
    let mut pred = BuildingPrediction {
        now: tick,
        s_eta: None,
        s_eta_sigma_s: 0.0,
        deadline_quantile: spec.choreo.deadline_quantile as f32,
        deadline_margin_s: spec.choreo.deadline_margin_s as f32,
        p_bus_max_w: spec.power.as_ref().map_or(0.0, |p| p.p_bus_max_w as f32),
        ..BuildingPrediction::default()
    };
    for (i, d) in devs.iter().enumerate().take(MAX_DEVICES) {
        pred.pfa_x_mps2[i] = tracks[i].pfa[0] as f32;
        pred.pfa_y_mps2[i] = tracks[i].pfa[1] as f32;
        let Some(p) = d.params() else {
            continue;
        };
        opts.kinematics[i] = Some(DeviceKinematics::from_params(p));
        if spec.choreo.smoothing && spec.devices[i].class == DeviceClassSpec::Drawer {
            let inr = &p.control.motion.inertial;
            let (s_max, v_rel, a_abs, ay, az) =
                skyhook_prediction(b, d.floor, d.heading, inr.f_c_hz, inr.zeta, t);
            let m = p.design.carriage.m_rated_kg;
            let e = reg.get(i).copied().unwrap_or_default();
            // Force budget m·a_LP + μ_pad·m·|a_y| + c_rail·|v_rel| + F_hyst ≤ F_cont
            // (plan §4b; PHYSICS §16.2).
            let force = m * a_abs
                + p.design.guidance.pad_mu * m * ay
                + p.design.damping.c_rail_n_s_per_m * v_rel
                + p.design.damping.f_hyst_n;
            let s = &p.design.suspension;
            let pole = derive::open_loop_pole(s.g_star_m, derive::h_prime(s.h_m_m, s.mu_rec));
            let g_float = derive::float_gap(p, m);
            let hold = derive::holdup_energy(p);
            opts.smooth[i] = Some(SmoothAdmissionInputs {
                seated: e.condition == DeviceCondition::Seated,
                x_open_m: (d.st.x_m - d.x_closed_m) as f32,
                s_min_m: (inr.s_min_base_m + s_max) as f32,
                v_rel_pred_mps: v_rel as f32,
                v_cap_mps: p.safety.v_close_max_mps as f32,
                v_rel_frac: inr.v_rel_frac as f32,
                stroke_fits: s_max <= inr.standoff_m - inr.stroke_margin_m,
                force_budget_ok: force <= p.design.lsm.f_cont_n,
                a_z_pred_mps2: az as f32,
                a_z_headroom_mps2: (inr.z_headroom_frac * pole * pole * (g_float - s.g_up_m).abs())
                    as f32,
                mains_or_ups: d.st.mains_ok || spec.power.as_ref().is_some_and(|pw| pw.ups),
                e_sc_j: d.st.e_hold_j as f32,
                e_land_j: hold.need_j as f32,
                e_seat_j: (p.design.power.p_hover_budget_w * p.control.levitation.seat_time_s)
                    as f32,
            });
        }
    }
    (reg, opts, pred)
}

// ---- Results ----------------------------------------------------------------------------

struct FinishInputs<'a> {
    spec: &'a SceneSpec,
    opts: &'a SceneRunOptions,
    devs: Vec<Dev>,
    tracks: Vec<Track>,
    recs: Vec<Rec>,
    brec: Rec,
    bresp: BuildingResponse,
    scene_events: Vec<(f64, Option<usize>, EventRec)>,
    hp: Option<HousePlan>,
    plan_t: Option<f64>,
    replans: u32,
    first_fa: Option<FusedAlert>,
    rate_hz: f64,
    t_end: f64,
    p_bus_peak: f64,
    over_budget_s: f64,
    wall_s: f64,
}

fn peak_xy(v: &[[f64; 3]]) -> (f64, f64) {
    v.iter().fold((0.0f64, 0.0f64), |(h, z), a| {
        (h.max(a[0].hypot(a[1])), z.max(a[2].abs()))
    })
}

#[allow(clippy::too_many_lines)]
fn finish(f: FinishInputs<'_>) -> Result<SceneData, SceneError> {
    let FinishInputs {
        spec,
        opts,
        mut devs,
        tracks,
        recs,
        brec,
        bresp,
        scene_events,
        hp,
        plan_t,
        replans,
        first_fa,
        rate_hz,
        t_end,
        p_bus_peak,
        over_budget_s,
        wall_s,
    } = f;
    let n = devs.len();

    // Events: device events (sessions + contents) and scene events, time then device.
    let mut all: Vec<(f64, usize, usize, SceneEvent)> = Vec::new();
    let mut seq = 0usize;
    for (t, dev, e) in scene_events {
        all.push((
            t,
            dev.map_or(0, |i| i + 1),
            seq,
            SceneEvent {
                device: dev.map(|i| spec.devices[i].id.clone()),
                event: e,
            },
        ));
        seq += 1;
    }
    for (i, d) in devs.iter_mut().enumerate() {
        for e in d.events.drain(..) {
            all.push((
                e.t_s,
                i + 1,
                seq,
                SceneEvent {
                    device: Some(spec.devices[i].id.clone()),
                    event: e,
                },
            ));
            seq += 1;
        }
        for e in &d.contents.events {
            all.push((
                e.t_s,
                i + 1,
                seq,
                SceneEvent {
                    device: Some(spec.devices[i].id.clone()),
                    event: EventRec {
                        t_s: e.t_s,
                        kind: format!("{}:{}", e.kind, e.item),
                        a: e.a,
                        b: e.b,
                    },
                },
            ));
            seq += 1;
        }
    }
    all.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    let events: Vec<SceneEvent> = all.into_iter().map(|x| x.3).collect();

    // Waterfall (plan §4b): |a_xy| and a_z peaks per stage; x-only stages labelled.
    let (g_xy, g_z) = peak_xy(&bresp.ground);
    let dev_floors: Vec<usize> = {
        let mut v: Vec<usize> = devs.iter().map(|d| d.floor).collect();
        v.sort_unstable();
        v.dedup();
        v
    };
    let (fl_xy, fl_z) = dev_floors.iter().fold((0.0f64, 0.0f64), |(h, z), &fi| {
        let (a, b) = peak_xy(bresp.acc.get(fi).map_or(&[][..], Vec::as_slice));
        (h.max(a), z.max(b))
    });
    let drawer_xy = tracks
        .iter()
        .zip(&spec.devices)
        .filter(|(_, d)| d.class == DeviceClassSpec::Drawer)
        .map(|(t, _)| t.peak_abs_axy)
        .fold(0.0f64, f64::max);
    let contents_xy = devs
        .iter()
        .map(|d| d.contents.totals(1).peak_abs_acc_mps2)
        .fold(0.0f64, f64::max);
    let mut waterfall = vec![
        json!({"stage": "ground", "axy_peak_mps2": g_xy, "az_peak_mps2": g_z, "note": "topo factor applied"}),
        json!({"stage": format!("floor ({})", spec.building.config_name()), "axy_peak_mps2": fl_xy, "az_peak_mps2": fl_z,
               "note": "device levels; vertical follows the ground (not amplified)"}),
    ];
    let mut kpis: BTreeMap<String, f64> = BTreeMap::new();
    if let BuildingConfig::TowerSentinel { .. } = &spec.building {
        // Counterfactual: the same tower with the damper locked.
        let mut locked = spec.building.clone();
        if let BuildingConfig::TowerSentinel { hmd_mode, .. } = &mut locked {
            *hmd_mode = fabrail_exo::hmd::HmdMode::LandedFps;
        }
        let rl = building::respond(
            &locked,
            |t| {
                let g = bresp.ground_at(t);
                let (_, _, _, topo) = spec.building.common();
                [g[0] / topo, g[1] / topo, g[2] / topo]
            },
            t_end,
        );
        let (lock_xy, _) = dev_floors.iter().fold((0.0f64, 0.0f64), |(h, z), &fi| {
            let (a, b) = peak_xy(rl.acc.get(fi).map_or(&[][..], Vec::as_slice));
            (h.max(a), z.max(b))
        });
        waterfall.push(
            json!({"stage": "sentinel", "axy_peak_mps2": fl_xy, "axy_peak_locked_mps2": lock_xy,
            "note": "floor with the passive tuned damper vs the damper locked (reduced model)"}),
        );
        kpis.insert(
            "sentinel_floor_axy_ratio".into(),
            if lock_xy > 0.0 {
                fl_xy / lock_xy
            } else {
                f64::NAN
            },
        );
    }
    waterfall.push(
        json!({"stage": "drawer body", "axy_peak_mps2": drawer_xy, "az_peak_mps2": fl_z,
        "note": "x-only stage: the drawer body acts along its rail; y is the floor"}),
    );
    waterfall.push(json!({"stage": "contents", "axy_peak_mps2": contents_xy,
        "note": "sliding-block item acceleration at nominal μ; toppling not modelled"}));

    // Scene KPIs.
    let info: &BuildingInfo = &bresp.info;
    let mut put = |k: &str, v: f64| {
        kpis.insert(k.to_string(), v);
    };
    put("n_devices", n as f64);
    put("t_end_s", t_end);
    put("quick", f64::from(u8::from(opts.quick)));
    put("building_period_s", info.period_s);
    put("iso_t_eff_s", info.iso_t_eff_s);
    put("iso_beta_eff", info.iso_beta_eff);
    put("iso_d_peak_m", info.iso_d_peak_m);
    put("iso_saturated_s", info.iso_saturated_s);
    put("hmd_mass_kg", info.hmd_mass_kg);
    put("hmd_stroke_peak_m", info.hmd_stroke_peak_m);
    put("waterfall_ground_axy_peak_mps2", g_xy);
    put("waterfall_ground_az_peak_mps2", g_z);
    put("waterfall_floor_axy_peak_mps2", fl_xy);
    put("waterfall_floor_az_peak_mps2", fl_z);
    put("waterfall_drawer_axy_peak_mps2", drawer_xy);
    put("waterfall_contents_axy_peak_mps2", contents_xy);
    put(
        "t_alert_s",
        first_fa.map_or(f64::NAN, |a| a.alert.received.seconds_f64()),
    );
    put("t_plan_s", plan_t.unwrap_or(f64::NAN));
    put(
        "deadline_s",
        hp.and_then(|p| p.deadline)
            .map_or(f64::NAN, Tick::seconds_f64),
    );
    put("replans", f64::from(replans));
    put("p_bus_peak_w", p_bus_peak);
    put(
        "p_bus_max_w",
        spec.power.as_ref().map_or(f64::NAN, |p| p.p_bus_max_w),
    );
    put("bus_over_budget_s", over_budget_s);
    put("plan_p_peak_w", hp.map_or(0.0, |p| f64::from(p.p_peak_w)));
    let planned = hp.map_or(0, |p| {
        p.as_slice()
            .iter()
            .filter(|a| a.status == PlanStatus::Planned)
            .count()
    });
    put("n_planned", planned as f64);
    put(
        "n_secured",
        tracks.iter().filter(|t| t.secured_s.is_some()).count() as f64,
    );
    put(
        "n_not_secured",
        tracks.iter().filter(|t| t.not_secured.is_some()).count() as f64,
    );
    put(
        "n_monitor_trips",
        tracks.iter().filter(|t| t.tripped).count() as f64,
    );
    put(
        "drawer_opened_events",
        f64::from(tracks.iter().map(|t| t.drawer_opened).sum::<u32>()),
    );
    for (k, label) in SWEEP_LABELS.iter().enumerate().take(SWEEP_POINTS) {
        let tot = devs
            .iter()
            .fold((0u32, 0u32, 0u32, 0u32, 0.0f64, 0.0f64), |acc, d| {
                let t = d.contents.totals(k);
                (
                    acc.0 + t.slip_events,
                    acc.1 + t.items_slipped,
                    acc.2 + t.impacts,
                    acc.3 + t.rock_events,
                    acc.4.max(t.max_utilisation),
                    acc.5 + t.slide_m,
                )
            });
        put(&format!("contents_slip_events_{label}"), f64::from(tot.0));
        put(&format!("contents_items_slipped_{label}"), f64::from(tot.1));
        put(&format!("contents_impacts_{label}"), f64::from(tot.2));
        put(&format!("contents_rock_events_{label}"), f64::from(tot.3));
        put(&format!("contents_max_util_{label}"), tot.4);
        put(&format!("contents_slide_m_{label}"), tot.5);
    }

    // Devices.
    let select = &spec.record.channels;
    let mut records = Vec::with_capacity(n);
    let mut plan_devices = Vec::new();
    let mut timeline = Vec::new();
    for (i, (d, (rec, tr))) in devs.iter().zip(recs.into_iter().zip(&tracks)).enumerate() {
        let ds = &spec.devices[i];
        let a = hp.as_ref().and_then(|p| p.get(i).copied());
        let policy = a.map_or_else(
            || {
                if spec.choreo.enable {
                    "core".to_string()
                } else {
                    "core (choreographer off)".to_string()
                }
            },
            |a| a.policy.slug().to_string(),
        );
        let mut k: BTreeMap<String, f64> = BTreeMap::new();
        k.insert("peak_v_mps".into(), tr.peak_v);
        k.insert("peak_body_axy_mps2".into(), tr.peak_abs_axy);
        k.insert("p_bus_peak_w".into(), tr.p_peak_w);
        k.insert(
            "e_hold_min_j".into(),
            if tr.e_hold_min_j.is_finite() {
                tr.e_hold_min_j
            } else {
                f64::NAN
            },
        );
        k.insert("pfa_x_pred_mps2".into(), tr.pfa[0]);
        k.insert("pfa_y_pred_mps2".into(), tr.pfa[1]);
        k.insert("pfa_z_pred_mps2".into(), tr.pfa[2]);
        k.insert(
            "secured".into(),
            f64::from(u8::from(tr.secured_s.is_some())),
        );
        k.insert("t_secured_s".into(), tr.secured_s.unwrap_or(f64::NAN));
        k.insert("t_go_s".into(), tr.go_s.unwrap_or(f64::NAN));
        k.insert("t_wait_s".into(), tr.wait_s.unwrap_or(f64::NAN));
        k.insert("drawer_opened_events".into(), f64::from(tr.drawer_opened));
        k.insert("monitor_trip".into(), f64::from(u8::from(tr.tripped)));
        for (s, label) in SWEEP_LABELS.iter().enumerate().take(SWEEP_POINTS) {
            let t = d.contents.totals(s);
            k.insert(
                format!("contents_slip_events_{label}"),
                f64::from(t.slip_events),
            );
            k.insert(format!("contents_impacts_{label}"), f64::from(t.impacts));
            k.insert(format!("contents_max_util_{label}"), t.max_utilisation);
            k.insert(format!("contents_slide_m_{label}"), t.slide_m);
        }
        k.insert(
            "contents_rock_events".into(),
            f64::from(d.contents.totals(1).rock_events),
        );
        k.insert(
            "contents_max_offset_m".into(),
            d.contents.totals(1).max_offset_m,
        );
        k.insert(
            "contents_max_impulse_ns".into(),
            d.contents.totals(1).max_impulse_ns,
        );
        if let Some(a) = a {
            let e = entry(spec, i, d, tr);
            k.insert(
                "v_unattended_mps".into(),
                f64::from(e.constraints.v_unattended_mps),
            );
            plan_devices.push(json!({
                "id": ds.id,
                "policy": a.policy.slug(),
                "action": format!("{:?}", a.decision.action),
                "target_m": a.decision.target.map(PosNm::to_m_f64),
                "v_cap_mps": a.decision.v_cap_mps,
                "target_rank": format!("{:?}", a.target_rank),
                "status": a.status.slug(),
                "reason": a.reason.map(NotSecuredReason::slug),
                "start_s": a.start.seconds_f64(),
                "finish_s": a.finish.seconds_f64(),
                "p_w": a.p_w,
                "secured_s": tr.secured_s,
                "outcome": tr.not_secured,
                "wait_s": tr.wait_s,
                "go_s": tr.go_s,
            }));
            timeline.push(json!({
                "id": ds.id,
                "start_s": a.start.seconds_f64(),
                "finish_pred_s": a.finish.seconds_f64(),
                "secured_s": tr.secured_s,
                "status": a.status.slug(),
                "wait_s": tr.wait_s,
                "go_s": tr.go_s,
            }));
        }
        records.push(DeviceRecord {
            id: ds.id.clone(),
            class: class_name(ds.class).to_string(),
            policy,
            channels: rec.into_channels(select),
            kpis: k,
            not_secured: tr.not_secured.clone(),
        });
    }

    let fa_json = first_fa.map(|a| {
        json!({
            "id": a.alert.id,
            "received_s": a.alert.received.seconds_f64(),
            "level": a.level.as_u8(),
            "p_damaging": a.p_damaging,
            "s_eta_s": a.s_eta.map(Tick::seconds_f64),
            "s_eta_sigma_s": a.s_eta_sigma_s,
            "n_independent": a.n_independent,
            "local_p": a.local_p,
            "injected": a.injected,
        })
    });
    let plan = json!({
        "provenance": SCENE_PROVENANCE,
        "prediction": PREDICTION_LABEL,
        "choreographer": if spec.choreo.enable { "enabled (not a rated safety function)" } else { "disabled" },
        "alert": fa_json,
        "t_plan_s": plan_t,
        "deadline_s": hp.and_then(|p| p.deadline).map(Tick::seconds_f64),
        "p_bus_max_w": spec.power.as_ref().map(|p| p.p_bus_max_w),
        "p_peak_w": hp.map(|p| p.p_peak_w),
        "replans": replans,
        "devices": plan_devices,
        "timeline": timeline,
        "waterfall": waterfall,
    });

    let meta = SceneMeta {
        version: env!("CARGO_PKG_VERSION").to_string(),
        scene: spec.name.clone(),
        seed: spec.seed,
        variant: serde_json::to_value(spec.variant)
            .ok()
            .and_then(|v| v.as_str().map(str::to_string))
            .unwrap_or_default(),
        building: spec.building.config_name().to_string(),
        provenance: SCENE_PROVENANCE.to_string(),
        spec: serde_json::to_value(spec).map_err(|e| SceneError::Json(e.to_string()))?,
        golden: golden(),
        build_profile: if cfg!(debug_assertions) {
            "debug".into()
        } else {
            "release".into()
        },
        wall_s,
    };
    Ok(SceneData {
        rate_hz,
        building: brec.into_channels(select),
        devices: records,
        events,
        kpis,
        plan,
        meta,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::run_scene;

    /// A 3-device, 2 s fixed-base smoke scene with a confirmed synthetic alert at 0.3 s.
    pub(crate) fn smoke(duration_s: f64) -> (SceneSpec, BTreeMap<String, Vec<u8>>) {
        let doc = json!({
            "schema": "fabrail.scene/1", "name": "smoke3", "seed": 42,
            "duration_s": duration_s, "sync_hz": 100, "variant": "custom",
            "building": {"config": "fixed_base", "period_s": 0.3, "zeta": 0.05, "topo_factor": 1.0,
                "floors": [{"level": 0, "name": "ground", "z_m": 0.0}, {"level": 1, "name": "upper", "z_m": 3.4}]},
            "ground": {"source": "record", "series": "series:ground", "origin_t_s": -5.0,
                "magnitude": 6.5, "r_epi_km": 30.0, "depth_km": 10.0},
            "series": {"ground": {"axis": "time", "fs_hz": 200, "t0_s": 0.0, "channels": ["ax", "ay", "az"]}},
            "devices": [
                {"id": "drawer_1", "class": "drawer", "level": 1, "heading_deg": 90, "fidelity": "carriage6",
                 "hero": true, "coil": "voltage",
                 "contents": [{"kind": "glassware", "count": 6}, {"kind": "stemware", "count": 4}],
                 "initial_state": {"lev": "levitated", "x_m": 0.2}},
                {"id": "pantry_door", "class": "door", "level": 0, "egress": "non_rated",
                 "fidelity": "travel1d", "coil": "lag", "policy": "stop_and_land"},
                {"id": "salon_wall", "class": "wall", "level": 1, "fidelity": "travel1d", "coil": "lag",
                 "policy": "land_and_lock", "contents": [{"kind": "lined", "count": 2}]}],
            "alert": {"provenance": SCENE_PROVENANCE, "local_p": true,
                "feeds": [{"kind": "network", "t_s": 0.3, "id": 1, "magnitude": 6.5, "r_epi_km": 30.0,
                           "depth_km": 10.0, "origin_t_s": -5.0, "mmi": 7},
                          {"kind": "das", "t_s": 0.35, "id": 2, "magnitude": 6.4, "r_epi_km": 30.0,
                           "depth_km": 10.0, "origin_t_s": -5.0}]},
            "power": {"p_bus_max_w": 400},
            "record": {"rate_hz": 50}
        });
        let spec = SceneSpec::from_json(&doc.to_string()).expect("smoke scene validates");
        // 0.25 g, 3 Hz x, 0.15 g 2 Hz y, 0.05 g 5 Hz z from 0.8 s.
        let fs = 200.0;
        let n = (duration_s * fs) as usize + 2;
        let g = fabrail_params::derive::G0_MPS2;
        let w = |a: f64, f: f64, t: f64| {
            if t < 0.8 {
                0.0
            } else {
                a * (2.0 * std::f64::consts::PI * f * (t - 0.8)).sin()
            }
        };
        let ch: Vec<Vec<f64>> = [(0.25 * g, 3.0), (0.15 * g, 2.0), (0.05 * g, 5.0)]
            .iter()
            .map(|&(a, f)| (0..n).map(|k| w(a, f, k as f64 / fs)).collect())
            .collect();
        let bytes = fabrail_sim::scenario::encode_f64_le(&ch);
        (spec, [("ground".to_string(), bytes)].into())
    }

    fn fingerprint(d: &SceneData) -> String {
        let mut s = String::new();
        for c in d
            .building
            .iter()
            .chain(d.devices.iter().flat_map(|x| x.channels.iter()))
        {
            s.push_str(&c.name);
            s.push_str(&format!("{:?}", c.data.to_le_bytes()));
        }
        for e in &d.events {
            s.push_str(&format!(
                "{:?}{}{:016x}{:016x}{:016x};",
                e.device,
                e.event.kind,
                e.event.t_s.to_bits(),
                e.event.a.to_bits(),
                e.event.b.to_bits()
            ));
        }
        for (k, v) in &d.kpis {
            s.push_str(&format!("{k}={:016x};", v.to_bits()));
        }
        for dv in &d.devices {
            for (k, v) in &dv.kpis {
                s.push_str(&format!("{}.{k}={:016x};", dv.id, v.to_bits()));
            }
            s.push_str(&format!("{:?}{:?}", dv.policy, dv.not_secured));
        }
        s.push_str(&d.plan.to_string());
        s.push_str(&d.meta.hashed_json());
        s
    }

    #[test]
    fn scene_smoke_worker_count_invariant() {
        // Plan §4b / FAB-EXA-016 (Rust side): results do not depend on the worker count;
        // devices are stepped in registry order and summed in index order.
        let (spec, series) = smoke(2.0);
        let one = run_scene(
            &spec,
            &series,
            &SceneRunOptions {
                workers: Some(1),
                quick: false,
            },
        )
        .expect("scene runs (1 worker)");
        let three = run_scene(
            &spec,
            &series,
            &SceneRunOptions {
                workers: Some(3),
                quick: false,
            },
        )
        .expect("scene runs (3 workers)");
        assert_eq!(
            fingerprint(&one),
            fingerprint(&three),
            "worker-count invariance"
        );
        let again = run_scene(
            &spec,
            &series,
            &SceneRunOptions {
                workers: Some(3),
                quick: false,
            },
        )
        .expect("scene runs again");
        assert_eq!(
            fingerprint(&again),
            fingerprint(&three),
            "same seed, same bytes"
        );
        // Shape: 50 Hz records over 2 s (101 samples), every device recorded, the alert
        // provenance label carried, bus power summed in index order.
        assert_eq!(one.rate_hz, 50.0);
        assert_eq!(one.devices.len(), 3);
        let n = one.building.first().map_or(0, |c| c.data.len());
        assert_eq!(n, 101);
        assert!(
            one.devices
                .iter()
                .all(|d| d.channels.iter().all(|c| c.data.len() == n))
        );
        let names: Vec<&str> = one.devices[0]
            .channels
            .iter()
            .map(|c| c.name.as_str())
            .collect();
        assert_eq!(names, DEVICE_CHANNELS, "default device channels, in order");
        assert!(one.building.iter().all(|c| known_channel(&c.name, &[0, 1])));
        assert_eq!(one.building.len(), BUILDING_CHANNELS.len() + 6);
        let mut bad = spec.clone();
        bad.record.channels = ChannelSelect::Names(vec!["x_m".into(), "nope".into()]);
        assert!(matches!(
            run_scene(&bad, &series, &SceneRunOptions::default()),
            Err(SceneError::Invalid { .. })
        ));
        assert_eq!(one.meta.provenance, SCENE_PROVENANCE);
        assert_eq!(one.plan["provenance"], SCENE_PROVENANCE);
        assert!(
            one.kpis["waterfall_floor_axy_peak_mps2"]
                > one.kpis["waterfall_ground_axy_peak_mps2"] * 0.5
        );
        let bus = one
            .building
            .iter()
            .find(|c| c.name == "p_bus_w")
            .expect("bus channel");
        let ChannelData::F64(bus) = &bus.data else {
            panic!("f64 bus power")
        };
        let per_dev: Vec<&Vec<f64>> = one
            .devices
            .iter()
            .filter_map(|d| d.channels.iter().find(|c| c.name == "p_bus_w"))
            .filter_map(|c| match &c.data {
                ChannelData::F64(v) => Some(v),
                ChannelData::U8(_) => None,
            })
            .collect();
        for (k, b) in bus.iter().enumerate() {
            let s = per_dev.iter().fold(0.0f64, |acc, v| acc + v[k]);
            assert_eq!(
                s.to_bits(),
                b.to_bits(),
                "bus power summed in index order at {k}"
            );
        }
        // Worker-invariance also holds in quick mode (Travel1d for every device).
        let q1 = run_scene(
            &spec,
            &series,
            &SceneRunOptions {
                workers: Some(1),
                quick: true,
            },
        )
        .expect("quick 1");
        let q2 = run_scene(
            &spec,
            &series,
            &SceneRunOptions {
                workers: Some(2),
                quick: true,
            },
        )
        .expect("quick 2");
        assert_eq!(fingerprint(&q1), fingerprint(&q2));
    }

    #[test]
    fn scene_restore_gating_and_spoofed_alert() {
        // Operator restore gating (plan §4b; FAB-EXA-015 at scene level): rejected before
        // the INSPECT reset and on a remote-only command; a remote INSPECT reset is
        // refused (a manual reset is local).
        let doc = json!({
            "schema": "fabrail.scene/1", "name": "restore_gate", "seed": 7, "duration_s": 1.0,
            "variant": "spoofed_alert",
            "building": {"config": "fixed_base", "period_s": 0.3,
                "floors": [{"level": 0, "name": "ground", "z_m": 0.0}]},
            "ground": {"source": "none"},
            "devices": [
                {"id": "drawer_1", "class": "drawer", "level": 0, "fidelity": "travel1d", "coil": "lag",
                 "initial_state": {"lev": "levitated", "x_m": 0.2}},
                {"id": "wall", "class": "wall", "level": 0, "fidelity": "travel1d", "coil": "lag"}],
            "alert": {"provenance": SCENE_PROVENANCE, "local_p": false,
                "feeds": [{"kind": "network", "t_s": 0.05, "id": 9, "magnitude": 7.0, "r_epi_km": 40.0,
                           "depth_km": 10.0, "origin_t_s": -3.0, "mmi": 8, "spoofed": true},
                          {"kind": "offshore_cabled", "t_s": 0.06, "id": 10, "magnitude": 7.0,
                           "r_epi_km": 40.0, "depth_km": 10.0, "origin_t_s": -3.0, "spoofed": true}]},
            "operator": [
                {"t_s": 0.1, "action": "restore", "source": "local"},
                {"t_s": 0.2, "action": "inspect_reset", "source": "remote"},
                {"t_s": 0.3, "action": "restore", "source": "local"},
                {"t_s": 0.4, "action": "inspect_reset", "source": "local"},
                {"t_s": 0.5, "action": "restore", "source": "remote"}],
            "record": {"rate_hz": 100, "channels": "kpis_only"}
        });
        let spec = SceneSpec::from_json(&doc.to_string()).expect("valid scene");
        let d = run_scene(&spec, &BTreeMap::new(), &SceneRunOptions::default()).expect("runs");
        let kinds: Vec<(f64, String, f64)> = d
            .events
            .iter()
            .filter(|e| e.device.is_none())
            .map(|e| (e.event.t_s, e.event.kind.clone(), e.event.a))
            .collect();
        let at = |t: f64| kinds.iter().filter(move |k| (k.0 - t).abs() < 1e-9);
        let code = |r: fabrail_exo::choreo::restore::RestoreReject| f64::from(r as u8);
        use fabrail_exo::choreo::restore::RestoreReject as R;
        assert!(
            at(0.1).any(|k| k.1 == "restore:rejected" && k.2 == code(R::InspectNotReset)),
            "{kinds:?}"
        );
        assert!(at(0.2).any(|k| k.1 == "operator:inspect_reset_rejected"));
        assert!(at(0.3).any(|k| k.1 == "restore:rejected" && k.2 == code(R::InspectNotReset)));
        // A local reset outside INSPECT is refused (the core ignores it there).
        assert!(at(0.4).any(|k| k.1 == "operator:inspect_reset_rejected"));
        assert!(
            at(0.5).any(|k| k.1 == "restore:rejected"),
            "remote restore refused: {kinds:?}"
        );
        assert!(!kinds.iter().any(|k| k.1 == "operator:restore"));
        // Spoofed (injected) feeds only: whatever was planned, no alert-initiated motion
        // (plan §4, §4b; bounded regret).
        if let Some(devs) = d.plan["devices"].as_array() {
            for dv in devs {
                assert!(
                    dv["target_m"].is_null(),
                    "spoofed alert moved a device: {dv}"
                );
            }
        }
        assert!(d.devices.iter().all(|x| x.channels.is_empty()), "kpis_only");
        assert!(d.building.is_empty());
    }

    #[test]
    fn ground_resampling_and_frames() {
        let (spec, series) = smoke(2.0);
        let g = Ground::new(&spec, &series).expect("ground");
        assert_eq!(g.at(0.5), [0.0; 3], "quiet before 0.8 s");
        assert_eq!(g.at(-1.0), [0.0; 3]);
        assert_eq!(
            g.at(100.0),
            [0.0; 3],
            "a time series reads 0 outside its span"
        );
        let t = 0.8 + 1.0 / 12.0; // quarter period of 3 Hz: the x peak
        let a = g.at(t);
        assert!((a[0] - 0.25 * fabrail_params::derive::G0_MPS2).abs() < 0.01 * 2.5);
        // Device frame: heading 90° maps building y onto the rail axis and building x
        // onto −cross-track (same rotation as contents::floor_cross_track).
        let d = to_device([1.0, 2.0, 3.0], std::f64::consts::FRAC_PI_2);
        assert!((d[0] - 2.0).abs() < 1e-12 && (d[1] + 1.0).abs() < 1e-12 && d[2] == 3.0);
        let c = crate::scene::contents::floor_cross_track([1.0, 2.0], std::f64::consts::FRAC_PI_2);
        assert!((c - d[1]).abs() < 1e-12);
        // Due index: an action at t runs at the first sync boundary at or after t.
        assert_eq!(due_index(0.3, 100.0), 30);
        assert_eq!(due_index(0.301, 100.0), 31);
        assert_eq!(due_index(0.0, 100.0), 0);
    }
}
