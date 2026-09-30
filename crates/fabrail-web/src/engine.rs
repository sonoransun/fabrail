//! The browser engine: one device (a class preset on a plant tier) closed through
//! fabrail-sim's steppable [`Session`], single-threaded (plan §3b). Private module; the
//! public items are re-exported at the crate root.
//!
//! Every engine is the same closed loop as a native `fabrail_sim::run`: the preset's
//! `SystemParams` (unchanged, so `Params.preset(name)` reproduces them in Python), the
//! `fabrail-rt` executive at the 20 kHz base tick (the firmware's group code), the
//! SimBoard sensor models with the in-repo RNG, and one plant tier stepped by RK4
//! (PHYSICS §5, §24). The scenario is generated here ([`scenario_json`]) and exported
//! verbatim (`fr_scenario_json`), so a native run of the same scenario plus the same
//! inputs reproduces a browser trace bit for bit (FAB-SW-014).
//!
//! Inputs are held until changed (zero-order hold):
//! * the hand is a pure force source along x on the front (grip contact: it pushes
//!   and pulls; no arm mass, damping or stiffness, so the applied force equals the
//!   input; PHYSICS §22); 0 N releases it;
//! * the floor acceleration (x, y, z) in the device rail frame is constant over each
//!   step call: the session's first-order hold is anchored at the current tick with
//!   the new value, so it never ramps from the previous one;
//! * a command is queued on the command link at the start of the next step call and,
//!   like every CAN frame in the simulation, reaches the supervisor after the link
//!   latency (`design.sensors.link_latency_s`).

use std::collections::BTreeMap;

use fabrail_params::{SystemParams, presets};
use fabrail_plant::Tier;
use fabrail_plant::interaction::HandInput;
use fabrail_sim::scenario::{
    Fidelity, FidelityCoil, FidelityModel, InitialSpec, MetricsSpec, PlantSpec, RecordSpec,
};
use fabrail_sim::{MAX_DURATION_S, SCENARIO_SCHEMA, ScenarioSpec, Session, SimError};
use fabrail_types::{MAX_MAGNETS, PosNm, RailClass, Tick, UserCommand};

use crate::{STATE_LEN, cmd, idx};

/// Preset ids (= `engine.js` `PRESETS`).
pub const PRESETS: [RailClass; 4] = [
    RailClass::Drawer,
    RailClass::Door,
    RailClass::Wall,
    RailClass::Heavy,
];

/// Plant tiers a browser engine can run, by tier id (`fabrail_create_tier`,
/// `fr_create_tier`). The Structure tier needs a building and is not offered.
pub const TIERS: [Tier; 3] = [Tier::Carriage6, Tier::Heave1, Tier::Travel1d];

/// Tier id of `fabrail_create` / `fr_create`: Carriage6, the full rigid body with the
/// real levitation, propulsion and haptic loops (the tier SIL ≡ firmware is claimed on
/// in voltage coil mode, plan §2b item 16).
pub const DEFAULT_TIER: u32 = 0;

/// Sensor-noise seed of every browser engine (the house seed; the SimBoard substreams
/// derive from it, PHYSICS §24).
pub const NOISE_SEED: u64 = 42;

/// Most base ticks one step call advances (5 s of simulated time; about 0.5 s of wall
/// time for Carriage6 in wasm), which bounds the work a single page call can queue.
pub const MAX_STEP_TICKS: u32 = 100_000;

/// Ticks per internal `Session::step_to` chunk (50 ms). Chunking is bit-identical to
/// one long call for a held floor (sim `session_chunking_matches_a_single_step`); it
/// only bounds how much work a failing step can hide.
const CHUNK_TICKS: u32 = 1_000;

/// Engine run state (`fabrail_status`, `fr_status`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum EngineStatus {
    /// Stepping normally.
    Running = 0,
    /// A simulation error (non-finite plant truth or a harness error) stopped it; the
    /// state buffer holds the last completed chunk.
    Failed = 2,
    /// The generated scenario's duration ([`MAX_DURATION_S`]) is exhausted.
    Ended = 3,
}

/// Why an engine could not be built.
#[derive(Clone, Debug, PartialEq)]
pub enum EngineError {
    /// No preset with this id.
    UnknownPreset(u32),
    /// No tier with this id.
    UnknownTier(u32),
    /// The preset, the generated scenario or the session rejected the setup.
    Sim(SimError),
}

impl core::fmt::Display for EngineError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EngineError::UnknownPreset(p) => write!(f, "preset: unknown id {p}"),
            EngineError::UnknownTier(t) => write!(f, "tier: unknown id {t}"),
            EngineError::Sim(e) => e.fmt(f),
        }
    }
}

impl From<SimError> for EngineError {
    fn from(e: SimError) -> Self {
        EngineError::Sim(e)
    }
}

/// Inputs held until the next step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Inputs {
    /// Hand force along x, N.
    pub hand_force_n: f64,
    /// Floor acceleration (x, y, z), m/s².
    pub floor_mps2: [f64; 3],
    /// Last command code set ([`cmd`]).
    pub command: u32,
}

/// The preset's class for preset id `preset`.
pub fn preset_class(preset: u32) -> Option<RailClass> {
    PRESETS.get(preset as usize).copied()
}

/// The plant tier for tier id `tier`.
pub fn tier_of(tier: u32) -> Option<Tier> {
    TIERS.get(tier as usize).copied()
}

/// The scenario (`fabrail.scenario/1`) a browser engine runs for preset `p` on `tier`:
/// no series, no events, the coil mode of the preset (the single source is the
/// scenario, and it must agree with `control.rates.coil_mode_voltage`, plan §2b item
/// 16), the class rest state, seed [`NOISE_SEED`] and the longest accepted duration.
/// Returned as the canonical JSON of the parsed spec, which is what the engine runs.
pub fn scenario_json(p: &SystemParams, tier: Tier) -> Result<String, SimError> {
    Ok(scenario_spec(p, tier)?.to_json())
}

fn scenario_spec(p: &SystemParams, tier: Tier) -> Result<ScenarioSpec, SimError> {
    let class = presets::class_name(p.design.class);
    let model = match tier {
        Tier::Carriage6 => FidelityModel::Carriage6,
        Tier::Heave1 => FidelityModel::Heave1,
        Tier::Travel1d => FidelityModel::Travel1d,
        Tier::Structure => FidelityModel::Structure,
    };
    let coil = if p.control.rates.coil_mode_voltage {
        FidelityCoil::Voltage
    } else {
        FidelityCoil::Lag
    };
    let tier_name = tier.name();
    // Built as a value, not parsed, so the wasm links no JSON parser; every block not
    // named here takes its documented `fabrail.scenario/1` default.
    let spec = ScenarioSpec {
        schema: SCENARIO_SCHEMA.into(),
        name: format!("web_{class}_{tier_name}"),
        doc: Some(format!(
            "fabrail-web browser engine: the {class} preset on {tier_name}, driven live by the page"
        )),
        duration_s: MAX_DURATION_S,
        noise_seed: NOISE_SEED,
        fidelity: Fidelity {
            model,
            coil,
            eddy_lag: true,
            kinematic_heave: true,
        },
        record: RecordSpec::default(),
        initial: InitialSpec::default(),
        plant: PlantSpec::default(),
        series: BTreeMap::new(),
        events: Vec::new(),
        metrics: MetricsSpec::default(),
    };
    spec.validate()?;
    Ok(spec)
}

/// Maps a command code ([`cmd`]) to the user command it queues (`None` for
/// [`cmd::NONE`] and unknown codes).
pub fn user_command(code: u32) -> Option<UserCommand> {
    Some(match code {
        cmd::OPEN => UserCommand::Open,
        cmd::CLOSE => UserCommand::Close,
        cmd::STOP => UserCommand::Stop,
        cmd::LIFT => UserCommand::Lift,
        cmd::LAND => UserCommand::Land,
        cmd::RESET => UserCommand::Reset,
        cmd::HOLD_TO_RUN_ON => UserCommand::HoldToRun(true),
        cmd::HOLD_TO_RUN_OFF => UserCommand::HoldToRun(false),
        cmd::MANUAL => UserCommand::Manual,
        _ => return None,
    })
}

/// The live hand for a force input: a pure force source with grip contact (PHYSICS §22,
/// `interaction::hand_force_n` with zero arm impedance), or no hand at 0 N.
fn hand_for(f_n: f64) -> Option<HandInput> {
    (f_n != 0.0).then_some(HandInput {
        f_drive_n: f_n,
        m_kg: 0.0,
        b_n_s_per_m: 0.0,
        k_n_per_m: 0.0,
        grip: true,
    })
}

/// One browser engine (a preset's controller and plant).
#[derive(Clone, Debug)]
pub struct Engine {
    class: RailClass,
    tier: Tier,
    params: SystemParams,
    session: Session,
    n_magnets: usize,
    end_tick: u64,
    inputs: Inputs,
    pending: Option<UserCommand>,
    applied_hand_n: f64,
    status: EngineStatus,
    error: Option<String>,
    state: [f64; STATE_LEN],
}

impl Engine {
    /// An engine for preset id `preset` on the default tier ([`DEFAULT_TIER`]); `None`
    /// for an unknown id or a setup the session rejects.
    pub fn new(preset: u32) -> Option<Engine> {
        Engine::with_tier(preset, DEFAULT_TIER).ok()
    }

    /// An engine for preset id `preset` on tier id `tier` ([`TIERS`]).
    pub fn with_tier(preset: u32, tier: u32) -> Result<Engine, EngineError> {
        let class = preset_class(preset).ok_or(EngineError::UnknownPreset(preset))?;
        let tier = tier_of(tier).ok_or(EngineError::UnknownTier(tier))?;
        let params = presets::by_class(class);
        let spec = scenario_spec(&params, tier)?;
        let end_tick = spec.duration_ticks();
        // Session::new validates the parameters, the scenario, their compatibility and
        // the plant step against the stiffest oscillatory element (PHYSICS §24).
        let session = Session::new(&params, &spec)?;
        let n_magnets = (params.design.suspension.n_magnets as usize).clamp(1, MAX_MAGNETS);
        let mut e = Engine {
            class,
            tier,
            params,
            session,
            n_magnets,
            end_tick,
            inputs: Inputs::default(),
            pending: None,
            applied_hand_n: 0.0,
            status: EngineStatus::Running,
            error: None,
            state: [0.0; STATE_LEN],
        };
        e.refresh();
        Ok(e)
    }

    /// The class.
    pub fn class(&self) -> RailClass {
        self.class
    }

    /// The plant tier.
    pub fn tier(&self) -> Tier {
        self.tier
    }

    /// The parameter set (the unchanged preset).
    pub fn params(&self) -> &SystemParams {
        &self.params
    }

    /// The session this engine drives.
    pub fn session(&self) -> &Session {
        &self.session
    }

    /// The canonical JSON of the scenario this engine runs.
    pub fn scenario_json(&self) -> String {
        self.session.spec().to_json()
    }

    /// Run state.
    pub fn status(&self) -> EngineStatus {
        self.status
    }

    /// The inputs held for the next steps.
    pub fn inputs(&self) -> Inputs {
        self.inputs
    }

    /// The command queued for the next step, if any.
    pub fn pending_command(&self) -> Option<UserCommand> {
        self.pending
    }

    /// Base tick of the state buffer.
    pub fn tick(&self) -> Tick {
        self.session.tick()
    }

    /// Sets the inputs for the next steps. Non-finite values read as 0. A known command
    /// code other than [`cmd::NONE`] replaces the pending command; `NONE` and unknown
    /// codes leave a pending command in place (a page may set forces every frame
    /// without cancelling a click).
    pub fn set_inputs(
        &mut self,
        hand_force_n: f64,
        floor_ax: f64,
        floor_ay: f64,
        floor_az: f64,
        command: u32,
    ) {
        let f = |x: f64| if x.is_finite() { x } else { 0.0 };
        let c = user_command(command);
        if c.is_some() {
            self.pending = c;
        }
        self.inputs = Inputs {
            hand_force_n: f(hand_force_n),
            floor_mps2: [f(floor_ax), f(floor_ay), f(floor_az)],
            command: if c.is_some() { command } else { cmd::NONE },
        };
    }

    /// Queues a move to absolute position `x_m` (m, travel axis) for the next step;
    /// false (nothing queued) for a non-finite target.
    pub fn go_to(&mut self, x_m: f64) -> bool {
        if !x_m.is_finite() {
            return false;
        }
        self.pending = Some(UserCommand::GoTo(PosNm::from_m_f64(x_m)));
        true
    }

    /// Advances up to `n_ticks` base ticks (20 kHz; at most [`MAX_STEP_TICKS`] per
    /// call and never past the scenario end); returns the ticks stepped. A stopped
    /// engine ([`EngineStatus::Failed`] or [`EngineStatus::Ended`]) steps 0.
    pub fn step(&mut self, n_ticks: u32) -> u32 {
        if self.status != EngineStatus::Running {
            return 0;
        }
        let now = self.session.tick().0;
        let left = self.end_tick.saturating_sub(now);
        let n = u64::from(n_ticks.min(MAX_STEP_TICKS)).min(left);
        if self.inputs.hand_force_n != self.applied_hand_n {
            self.session.set_hand(hand_for(self.inputs.hand_force_n));
            self.applied_hand_n = self.inputs.hand_force_n;
        }
        let floor = self.inputs.floor_mps2;
        // Anchor the first-order hold at `now` with the new floor value (zero-order hold
        // of the input) and put the pending command on the link at `now`.
        let cmd = self.pending.take();
        if let Err(e) = self.session.step_to(Tick(now), &floor, cmd) {
            return self.fail(e, 0);
        }
        let mut done = 0u64;
        while done < n {
            let chunk = (n - done).min(u64::from(CHUNK_TICKS));
            if let Err(e) = self.session.step_to(Tick(now + done + chunk), &floor, None) {
                return self.fail(e, done);
            }
            done += chunk;
        }
        self.refresh();
        if now + done >= self.end_tick {
            self.status = EngineStatus::Ended;
        }
        done as u32
    }

    fn fail(&mut self, e: SimError, done: u64) -> u32 {
        self.status = EngineStatus::Failed;
        self.error = Some(e.to_string());
        self.refresh();
        done as u32
    }

    /// The simulation error that stopped the engine, if any.
    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    /// Recomputes the state buffer from the session (plant truth and the executive's
    /// latest published outputs). Truth fields are the plant after the last completed
    /// step (t = tick/20 kHz); controller fields are the outputs of the last executed
    /// base tick (tick − 1), as a firmware telemetry frame would carry them.
    fn refresh(&mut self) {
        let st = self.session.status();
        let snap = self.session.snapshot();
        let tr = self.session.truth();
        let n = self.n_magnets;
        // Mean over the fitted elements, summed in element order.
        let mean = |v: &[f64; MAX_MAGNETS]| v[..n].iter().fold(0.0, |s, x| s + x) / n as f64;
        let s = &mut self.state;
        s[idx::TICK] = st.tick.0 as f64;
        s[idx::T_S] = st.tick.seconds_f64();
        s[idx::X_M] = tr.x_m;
        s[idx::V_MPS] = tr.v_mps;
        s[idx::A_MPS2] = tr.a_mps2;
        s[idx::GAP_M] = mean(&tr.gaps_m);
        s[idx::COIL_I_A] = mean(&tr.coil_i_a);
        s[idx::IQ_A] = tr.iq_a;
        s[idx::F_LSM_N] = tr.f_lsm_n;
        s[idx::F_HAND_N] = tr.f_hand_n;
        s[idx::F_HAND_HAT_N] = f64::from(snap.fast.status.f_hand_hat_n);
        s[idx::M_HAT_KG] = f64::from(st.m_hat_kg);
        s[idx::E_KIN_GOV_J] = f64::from(snap.fast.status.e_kin_gov_j);
        s[idx::LEV_STATE] = f64::from(st.lev.as_u8());
        s[idx::MOTION_MODE] = f64::from(st.mode.as_u8());
        s[idx::SEISMIC_STATE] = f64::from(st.seismic.as_u8());
        s[idx::SEISMIC_LEVEL] = f64::from(st.level.as_u8());
        s[idx::SAFETY_DEMAND] = f64::from(st.demand.as_u8());
        s[idx::FAULT_BITS_LO] = f64::from(st.faults.bits[0]);
        s[idx::FLOOR_AX_MPS2] = tr.floor_acc_mps2[0];
    }

    /// The state buffer.
    pub fn state(&self) -> &[f64; STATE_LEN] {
        &self.state
    }

    /// Shortens the scenario end (tests of [`EngineStatus::Ended`]).
    #[cfg(test)]
    pub(crate) fn with_end_tick(mut self, end_tick: u64) -> Engine {
        self.end_tick = end_tick;
        self
    }
}
