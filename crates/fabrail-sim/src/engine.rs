//! The shared base-tick engine behind `run`, `replay::record` and `Session` (private).
//!
//! One engine is one device: the plant tier, the `fabrail-rt` executive (the same
//! group code as the firmware), the `SimBoard`, the scenario event executor and the
//! optional recorder, metrics and HAL trace. Per base tick k (core design §B.6;
//! PHYSICS §24):
//!
//! 1. apply the scenario events due at k (discrete actions: command/alert links with
//!    the CAN latency, sensor faults, reboots, core-fault injection);
//! 2. SimBoard snapshots truth at t_k (sensed state, 1-tick transport latency, ADXL355
//!    FIFO at the ODR);
//! 3. `Executive::on_base_tick_with_policy` (LET: base first, then fast, supervisor and
//!    seismic when due);
//! 4. latch the issued commands (zero-order hold; averaged over the window for the
//!    1 ms tiers) and record the channels when k is a recording tick (truth at t_k);
//! 5. when the plant step window ends, one RK4 `plant.step` over [t₀, t₀ + h] with the
//!    floor acceleration at t₀, t₀ + h/2 and t₀ + h (Carriage6/Heave1: h = 50 µs every
//!    tick; Travel1d/Structure: h = 1 ms every 20 ticks);
//! 6. metrics at the base rate on the truth after the step.
//!
//! The executive always runs at the 20 kHz base tick, whatever the plant tier, so every
//! controller sees the sample period it was designed for.

use fabrail_core::msg::LevDirective;
use fabrail_core::seismic::action_policy::ActionPolicy;
use fabrail_params::SystemParams;
use fabrail_plant::body::Carriage6;
use fabrail_plant::heave1::Heave1;
use fabrail_plant::params::PlantParams;
use fabrail_plant::structure::Structure;
use fabrail_plant::travel1d::Travel1d;
use fabrail_plant::{
    HeaveHint, InitialState, ModelConfig, PlantError, PlantInputs, PlantModel, Tier, Truth,
};
use fabrail_rt::{ExecSnapshot, Executive};
use fabrail_types::{AuxCmd, BASE_TICK_S, MAX_MAGNETS, Tick, fmath};

use crate::events::{Discrete, EventExecutor};
use crate::metrics::{Metrics, Observation};
use crate::recorder::{EventRec, RecordView, Recorder};
use crate::replay::{ControlAction, TraceRecord, core_fault};
use crate::scenario::{ControlLossDir, ScenarioSpec, SimError};
use crate::simboard::{Actuation, SimBoard};

/// A plant tier, owned concretely so the engine (and `Session`) stay `Clone`.
#[derive(Clone, Debug)]
pub(crate) enum Plant {
    Carriage6(Box<Carriage6>),
    Heave1(Box<Heave1>),
    Travel1d(Box<Travel1d>),
    Structure(Box<Structure>),
}

impl Plant {
    /// Builds tier `cfg.tier` over `pp` (mirrors `fabrail_plant::build`).
    pub(crate) fn new(cfg: &ModelConfig, pp: &PlantParams) -> Result<Plant, SimError> {
        Ok(match cfg.tier {
            Tier::Carriage6 => Plant::Carriage6(Box::new(Carriage6::new(pp, cfg))),
            Tier::Heave1 => Plant::Heave1(Box::new(Heave1::new(pp, cfg))),
            Tier::Travel1d => Plant::Travel1d(Box::new(Travel1d::new(pp, cfg))),
            Tier::Structure => {
                let b = pp
                    .env
                    .structure
                    .as_ref()
                    .ok_or(PlantError::MissingStructure)?;
                Plant::Structure(Box::new(Structure::new(pp, b, cfg)?))
            }
        })
    }

    pub(crate) fn model(&self) -> &dyn PlantModel {
        match self {
            Plant::Carriage6(m) => m.as_ref(),
            Plant::Heave1(m) => m.as_ref(),
            Plant::Travel1d(m) => m.as_ref(),
            Plant::Structure(m) => m.as_ref(),
        }
    }

    pub(crate) fn model_mut(&mut self) -> &mut dyn PlantModel {
        match self {
            Plant::Carriage6(m) => m.as_mut(),
            Plant::Heave1(m) => m.as_mut(),
            Plant::Travel1d(m) => m.as_mut(),
            Plant::Structure(m) => m.as_mut(),
        }
    }
}

/// Floor (rail) and site acceleration as functions of time.
pub(crate) trait FloorInput {
    /// Floor acceleration (x, y, z) at `t_s` in the device rail frame, m/s².
    fn floor(&self, t_s: f64) -> [f64; 3];
    /// Site noise added to the structure accelerometer only, m/s².
    fn site(&self, t_s: f64) -> [f64; 3];
}

/// First-order hold between two floor samples (Session chunks).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct HoldFloor {
    pub t0: f64,
    pub f0: [f64; 3],
    pub t1: f64,
    pub f1: [f64; 3],
}

impl FloorInput for HoldFloor {
    fn floor(&self, t_s: f64) -> [f64; 3] {
        if self.t1 <= self.t0 {
            return self.f1;
        }
        let u = ((t_s - self.t0) / (self.t1 - self.t0)).clamp(0.0, 1.0);
        [
            self.f0[0] + (self.f1[0] - self.f0[0]) * u,
            self.f0[1] + (self.f1[1] - self.f0[1]) * u,
            self.f0[2] + (self.f1[2] - self.f0[2]) * u,
        ]
    }
    fn site(&self, _t_s: f64) -> [f64; 3] {
        [0.0; 3]
    }
}

/// What an engine keeps besides the closed loop.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct EngineOpts {
    /// Record channels.
    pub record: bool,
    /// Accumulate KPIs.
    pub metrics: bool,
    /// Log HAL traffic for replay.
    pub trace: bool,
    /// Device index (sensor substreams).
    pub device: u16,
}

/// Window accumulator of the latched actuation (the 1 ms tiers average the 20 base-tick
/// commands of their step; Carriage6/Heave1 use the tick's own command).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Window {
    n: u32,
    coil: [f64; MAX_MAGNETS],
    iq: f64,
    id: f64,
    last: Actuation,
}

/// One device's closed loop.
#[derive(Clone, Debug)]
pub(crate) struct Engine {
    p: Box<SystemParams>,
    spec: ScenarioSpec,
    tier: Tier,
    step_ticks: u64,
    link_ticks: u64,
    decim: u64,
    plant: Plant,
    pub(crate) exec: Executive,
    pub(crate) board: SimBoard,
    pub(crate) ev: EventExecutor,
    tick: Tick,
    window: Window,
    snap: ExecSnapshot,
    metrics: Option<Metrics>,
    recorder: Option<Recorder>,
    trace: Option<TraceRecord>,
    pub(crate) log: Vec<EventRec>,
    discrete: Vec<Discrete>,
    core_fault_until: Option<Tick>,
    prev_state: fabrail_plant::ContactState,
    prev_bumpers: u32,
    prev_obstacle: bool,
    prev_end_stop: bool,
    prev_z: (f64, f64),
    v_z: f64,
    warned_nonfinite: bool,
}

impl Engine {
    /// Builds the loop for parameter set `p`, scenario `spec` and plant parameters
    /// `pp` (validation is the caller's job: `preflight` / `Session::new`).
    pub(crate) fn new(
        p: &SystemParams,
        spec: &ScenarioSpec,
        pp: &PlantParams,
        opts: EngineOpts,
    ) -> Result<Engine, SimError> {
        let cfg = spec.model_config();
        let init: InitialState = spec.initial.resolve(p);
        let mut plant = Plant::new(&cfg, pp)?;
        plant.model_mut().reset(&init);
        let mut exec = Executive::new(p)?;
        // A scenario that starts levitated adopts that state instead of booting to the
        // class rest state: the supervisor warm-starts to LEVITATED_IDLE and its
        // Levitate directive catches the floating carriage (A4/A1 contract; W1.5).
        let warm = (init.lev == fabrail_plant::InitLev::Levitated
            && exec
                .sup
                .sup
                .warm_start(fabrail_types::LevState::LevitatedIdle))
        .then_some(fabrail_types::LevState::LevitatedIdle);
        let mut board = SimBoard::with_params(p, cfg.coil, spec.noise_seed, opts.device);
        board.set_initial_aux(AuxCmd {
            parking_lock: init.locked,
            seismic_latch: false,
        });
        if opts.trace {
            board.enable_trace();
        }
        let ev = EventExecutor::new(spec, p, &init);
        board.env.presence_ok = ev.presence_ok;
        let n_mag = (p.design.suspension.n_magnets as usize).min(MAX_MAGNETS);
        let decim = spec.record_decimation().max(1);
        let n_rec = (spec.duration_ticks() / decim + 1) as usize;
        let recorder = if opts.record {
            Some(Recorder::new(&spec.record.channels, n_mag, n_rec)?)
        } else {
            None
        };
        let metrics = opts
            .metrics
            .then(|| Metrics::new(p, spec.duration_s, &spec.metrics.windows));
        let truth = *plant.model().truth();
        let link_ticks = Tick::from_seconds_f64(p.design.sensors.link_latency_s.max(0.0)).0;
        Ok(Engine {
            p: Box::new(*p),
            spec: spec.clone(),
            tier: cfg.tier,
            step_ticks: cfg.tier.ticks_per_step().max(1),
            link_ticks,
            decim,
            plant,
            snap: exec.snapshot(),
            exec,
            board,
            ev,
            tick: Tick::ZERO,
            window: Window::default(),
            metrics,
            recorder,
            trace: opts.trace.then(|| TraceRecord {
                params: *p,
                control: warm
                    .map(|l| (Tick::ZERO, ControlAction::WarmStart(l.as_u8())))
                    .into_iter()
                    .collect(),
                ..TraceRecord::default()
            }),
            log: Vec::new(),
            discrete: Vec::new(),
            core_fault_until: None,
            prev_state: truth.state,
            prev_bumpers: truth.e_diss.bumper_impacts,
            prev_obstacle: false,
            prev_end_stop: false,
            prev_z: (truth.t_s, truth.pose[2]),
            v_z: 0.0,
            warned_nonfinite: false,
        })
    }

    /// The next tick to execute (= the tick the truth is at).
    pub(crate) fn tick(&self) -> Tick {
        self.tick
    }

    /// Plant truth.
    pub(crate) fn truth(&self) -> &Truth {
        self.plant.model().truth()
    }

    /// The executive's latest published outputs.
    pub(crate) fn snapshot(&self) -> &ExecSnapshot {
        &self.snap
    }

    /// The scenario.
    pub(crate) fn spec(&self) -> &ScenarioSpec {
        &self.spec
    }

    /// Queues a user command on the command link now (CAN latency applies).
    pub(crate) fn push_command(&mut self, c: fabrail_types::UserCommand) {
        self.board
            .enqueue_command(Tick(self.tick.0 + self.link_ticks), c);
    }

    /// Queues an alert on the alert link now (CAN latency applies).
    pub(crate) fn push_alert(&mut self, a: fabrail_types::NetworkAlert) {
        self.board
            .enqueue_alert(Tick(self.tick.0 + self.link_ticks), a);
    }

    /// Floor acceleration the device's sensors feel at `t_s` (Structure tier: the
    /// device storey's absolute x acceleration from the building model, the ground
    /// y and z).
    fn sensed_floor(&self, f: &dyn FloorInput, t_s: f64) -> [f64; 3] {
        let g = f.floor(t_s);
        if self.tier == Tier::Structure {
            [self.truth().host_a_mps2, g[1], g[2]]
        } else {
            g
        }
    }

    fn apply_events(&mut self, k: Tick) -> Result<(), SimError> {
        if let Some(u) = self.core_fault_until
            && k >= u
        {
            self.exec.clear_injection();
            self.board.faults.set_core(None);
            self.core_fault_until = None;
            if let Some(tr) = self.trace.as_mut() {
                tr.control.push((k, ControlAction::ClearInjection));
            }
        }
        self.discrete.clear();
        let mut acts = core::mem::take(&mut self.discrete);
        self.ev.apply_due(k, &mut acts, &mut self.log);
        for a in acts.drain(..) {
            match a {
                Discrete::Command(c) => self.board.enqueue_command(Tick(k.0 + self.link_ticks), c),
                Discrete::Alert(al) => self.board.enqueue_alert(Tick(k.0 + self.link_ticks), al),
                Discrete::SensorFault { what, ch, jump_m } => {
                    let npc = self.p.design.sensors.nm_per_count.max(1) as f64;
                    let counts = fmath::round(jump_m * 1.0e9 / npc) as i64;
                    self.board.faults.activate(what, ch, counts);
                }
                Discrete::Reboot(cause) => {
                    self.exec = Executive::new(&self.p)?;
                    self.board.reset_cause = cause.into();
                    self.core_fault_until = None;
                    if let Some(tr) = self.trace.as_mut() {
                        tr.control.push((k, ControlAction::Reboot));
                    }
                }
                Discrete::CoreFault { what, until } => {
                    self.exec.inject(core_fault(what));
                    self.board.faults.set_core(Some(what));
                    self.core_fault_until = until;
                    if let Some(tr) = self.trace.as_mut() {
                        tr.control.push((k, ControlAction::Inject(what)));
                    }
                }
            }
        }
        self.discrete = acts;
        // Continuous event state the board reads.
        self.board.faults.control_loss = self.ev.control_loss();
        let env = &mut self.board.env;
        env.presence_active = self.ev.presence_active;
        env.presence_ok = self.ev.presence_ok;
        env.hold_to_run = self.ev.hold_to_run;
        env.fire_alarm = self.ev.fire_alarm;
        env.estop = self.ev.estop;
        Ok(())
    }

    /// Travel1d kinematic heave hint from the levitation directive in force (plan §2b
    /// item 16). `Off` maps to `Hold`: Travel1d assumes ideal levitation and cannot
    /// represent the hybrid-PM divergence of idle coils (that needs Carriage6/Heave1);
    /// only a `control_loss: down` event drops the carriage.
    fn heave_hint(&self) -> HeaveHint {
        if self.ev.control_loss() == Some(ControlLossDir::Down) {
            return HeaveHint::Drop;
        }
        match self.snap.sup.directive.lev {
            LevDirective::Lift | LevDirective::Unseat => HeaveHint::Lift,
            LevDirective::Land { .. } => HeaveHint::Land,
            LevDirective::Seat => HeaveHint::Seat,
            LevDirective::Off | LevDirective::Levitate | LevDirective::Hold => HeaveHint::Hold,
        }
    }

    /// Executes one base tick against `floor`, the supervisor consulting `policy`.
    pub(crate) fn step(
        &mut self,
        floor: &dyn FloorInput,
        policy: &mut dyn ActionPolicy,
    ) -> Result<(), SimError> {
        let k = self.tick;
        let t = k.seconds_f64();
        // 1. Events due at k.
        self.apply_events(k)?;
        // 2. SimBoard snapshot at t_k.
        let fl = self.sensed_floor(floor, t);
        let site = floor.site(t);
        let structure = [fl[0] + site[0], fl[1] + site[1], fl[2] + site[2]];
        let truth = *self.plant.model().truth();
        self.board.begin_tick(k, &truth, fl, structure);
        // 3. The executive (the firmware's group code).
        self.exec.on_base_tick_with_policy(&mut self.board, policy);
        if let Some(e) = self.board.take_error() {
            return Err(SimError::Runtime(e));
        }
        self.snap = self.exec.snapshot();
        if let Some(tr) = self.trace.as_mut()
            && k.is_release(self.exec.rates.fast_decim as u32)
        {
            tr.push_fast(self.exec.last_raw);
        }
        if self.board.has_events() {
            for e in self.board.take_events() {
                self.log.push(EventRec::from_core(&e));
            }
        }
        // 4. Latch (ZOH) and record.
        let act = self.board.actuation();
        if !self.warned_nonfinite && self.board.nonfinite_commands() > 0 {
            self.warned_nonfinite = true;
            self.log.push(EventRec {
                t_s: t,
                kind: "harness:nonfinite_command".into(),
                a: self.board.nonfinite_commands() as f64,
                b: 0.0,
            });
        }
        let w = &mut self.window;
        w.n += 1;
        for (c, a) in w.coil.iter_mut().zip(act.coil.iter()) {
            *c += a;
        }
        w.iq += act.iq_ref_a;
        w.id += act.id_ref_a;
        w.last = act;
        if k.0.is_multiple_of(self.decim) {
            self.record_now(&truth);
        }
        // 5. Plant step at the end of the window.
        if (k.0 + 1).is_multiple_of(self.step_ticks) {
            self.plant_step(floor, k)?;
        }
        // 6. Metrics on the truth after the step.
        self.tick = Tick(k.0 + 1);
        if let Some(m) = self.metrics.as_mut() {
            let tr = self.plant.model().truth();
            let t1 = self.tick.seconds_f64();
            let fx = if self.tier == Tier::Structure {
                tr.host_a_mps2
            } else {
                floor.floor(t1)[0]
            };
            m.observe(&Observation {
                t_s: t1,
                truth: tr,
                snap: &self.snap,
                hand_since: self.ev.hand(t1).map(|(_, t0)| t0),
                floor_x_mps2: fx,
            });
        }
        Ok(())
    }

    fn record_now(&mut self, truth: &Truth) {
        if let Some(r) = self.recorder.as_mut() {
            let t = self.tick.seconds_f64();
            r.push(&RecordView {
                truth,
                snap: &self.snap,
                hand_active: self.ev.hand(t).is_some(),
            });
        }
    }

    fn plant_step(&mut self, floor: &dyn FloorInput, k: Tick) -> Result<(), SimError> {
        let s = self.step_ticks;
        let t0 = Tick(k.0 + 1 - s).seconds_f64();
        // Floor samples at t₀, t₀ + h/2, t₀ + h with the end point taken from the tick
        // (plant time is computed from ticks, never accumulated; core design §A.4
        // item 3), so a chunked session's first-order hold ends exactly on its value.
        let t1 = Tick(k.0 + 1).seconds_f64();
        let h = s as f64 * BASE_TICK_S;
        let w = core::mem::take(&mut self.window);
        let n = w.n.max(1) as f64;
        let mut coil = w.coil;
        for c in coil.iter_mut() {
            *c /= n;
        }
        let u = PlantInputs {
            coil_v_or_iref: coil,
            coil_enabled: w.last.coil_enabled,
            iq_ref_a: w.iq / n,
            id_ref_a: w.id / n,
            lsm_mode: w.last.lsm_mode,
            parking_lock: w.last.parking_lock,
            seismic_latch: w.last.seismic_latch,
            mains_ok: self.ev.mains_ok(k),
            payload_kg: self.ev.payload_kg(),
            t_ambient_c: self.ev.ambient_c(t0),
            air_leak_area_m2: self.ev.leak_area_m2(),
            heave_hint: self.heave_hint(),
            f_ext: self.ev.ext_forces(t0),
        };
        let fl = [
            floor.floor(t0),
            floor.floor(0.5 * (t0 + t1)),
            floor.floor(t1),
        ];
        self.plant.model_mut().step(t0, h, &u, &fl);
        self.board.inputs = u;
        let tr = *self.plant.model().truth();
        if !(tr.x_m.is_finite() && tr.v_mps.is_finite() && tr.pose[2].is_finite()) {
            return Err(SimError::Runtime(format!(
                "non-finite plant truth at t = {:.6} s (x = {}, v = {}, z = {})",
                t0 + h,
                tr.x_m,
                tr.v_mps,
                tr.pose[2]
            )));
        }
        self.plant_events(&tr);
        Ok(())
    }

    /// Plant events from contact-state edges (`"plant:<what>"`).
    fn plant_events(&mut self, tr: &Truth) {
        let t = tr.t_s;
        if t > self.prev_z.0 {
            self.v_z = (tr.pose[2] - self.prev_z.1) / (t - self.prev_z.0);
            self.prev_z = (t, tr.pose[2]);
        }
        let (p, s) = (self.prev_state, tr.state);
        let mut ev = |kind: &str, a: f64, b: f64| {
            self.log.push(EventRec {
                t_s: t,
                kind: format!("plant:{kind}"),
                a,
                b,
            });
        };
        if s.levitated && !p.levitated {
            ev("levitated", 0.0, 0.0);
        }
        if s.landed && !p.landed {
            ev("landed", self.v_z.abs(), 0.0);
        }
        if s.seated && !p.seated {
            ev("seated", self.v_z.abs(), 0.0);
        }
        if s.upper_stop && !p.upper_stop {
            ev("upper_stop", tr.f_contact.upper_n, 0.0);
        }
        if s.locked && !p.locked {
            ev("locked", 0.0, 0.0);
        }
        if s.latched && !p.latched {
            ev("latched", 0.0, 0.0);
        }
        if tr.e_diss.bumper_impacts > self.prev_bumpers {
            ev(
                "bumper",
                0.5 * tr.mass_kg * tr.v_mps * tr.v_mps,
                tr.v_mps.abs(),
            );
        }
        let obstacle = tr.f_contact.obstacle_n > 0.0;
        if obstacle && !self.prev_obstacle {
            ev("obstacle_contact", tr.v_mps.abs(), 0.0);
        }
        // Physical end stop (closed-end jamb / carriage carcass, open-end rail stop; W2
        // lead decision D3): reported on contact onset with the impact speed and the
        // position, never a fault by itself.
        let end_stop = tr.f_contact.end_stop_n > 0.0;
        if end_stop && !self.prev_end_stop {
            ev("end_stop", tr.v_mps.abs(), tr.x_m);
        }
        self.prev_end_stop = end_stop;
        self.prev_obstacle = obstacle;
        self.prev_bumpers = tr.e_diss.bumper_impacts;
        self.prev_state = s;
    }

    /// Records the final sample (t = duration) when it falls on the recording grid.
    pub(crate) fn record_final(&mut self) {
        if self.tick.0.is_multiple_of(self.decim) {
            let truth = *self.plant.model().truth();
            self.record_now(&truth);
        }
    }

    /// The recorded channels.
    pub(crate) fn take_channels(&mut self) -> Vec<crate::recorder::Channel> {
        self.recorder
            .take()
            .map(Recorder::into_channels)
            .unwrap_or_default()
    }

    /// Final KPIs.
    pub(crate) fn kpis(&self) -> std::collections::BTreeMap<String, f64> {
        self.metrics
            .as_ref()
            .map(Metrics::finish)
            .unwrap_or_default()
    }

    /// The HAL trace (inputs, outputs, control actions, fast raws).
    pub(crate) fn take_trace(&mut self) -> Option<TraceRecord> {
        let mut tr = self.trace.take()?;
        let io = self.board.take_trace();
        tr.inputs = io.inputs;
        tr.outputs = io.outputs;
        tr.ticks = self.tick.0;
        Some(tr)
    }

    /// Sets (or removes) a live hand (steppable sessions).
    pub(crate) fn set_hand(&mut self, hand: Option<fabrail_plant::interaction::HandInput>) {
        let t = self.tick.seconds_f64();
        self.ev.set_hand(hand, t);
    }
}
