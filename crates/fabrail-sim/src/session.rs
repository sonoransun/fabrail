//! Session: the steppable core API (new, step_to(tick, floor, cmds), truth, status) used by the building scenes. Owner: A7 (signatures frozen by W0-B3).
//!
//! A `Session` is one device (params + plant + the same `fabrail-rt` executive as
//! `run`, on the same engine) advanced in chunks by an external driver: the exotic
//! building scene steps every device in 200-tick chunks at 100 Hz sync, rayon across
//! devices, bus power summed in registry index order (plan §4b). Coupling is one-way:
//! the scene supplies the floor acceleration; the device never loads the building.
//!
//! Contract:
//! * `new` validates the parameters, the scenario and their compatibility (the coil
//!   mode single source included) and refuses a plant step that is too large for an
//!   oscillatory element (PHYSICS §24), exactly like `run`. The floor comes from
//!   `step_to`, so the scenario must declare no series (rail irregularity and site
//!   noise are not supported in a Session v1); its `events` still apply at their times.
//! * `step_to(tick, floor, cmd)` advances to base tick `tick` (20 kHz units; a tick at
//!   or before the current one is a no-op for time, but `cmd` is still queued).
//!   `floor_mps2` is the floor acceleration (x, y, z) in the DEVICE rail frame at
//!   `tick`; between the previous call and `tick` the session interpolates linearly from
//!   the previous value (first-order hold; the value before the first call is 0).
//!   `cmd` is put on the command link at the start of the chunk and, like every CAN
//!   frame in the simulation, reaches the supervisor after `link_latency_s` (3 ms).
//!   Non-finite floor input is rejected.
//! * `step_to_with_policy(tick, floor, cmd, policy)` is `step_to` with the supervisor
//!   consulting `policy` (a `fabrail_core::seismic::action_policy::ActionPolicy`, e.g.
//!   the exo choreographer's `PlanPolicy`) for the seismic action, exactly as
//!   `fabrail_rt::Executive::on_base_tick_with_policy`; the core `admit` filter still
//!   applies. `step_to` uses the core policy (`CorePolicy`, never overrides). The
//!   policy is borrowed per chunk, so the scene owns one policy per device and the
//!   session stays `Clone + Send`.
//! * `push_alert` delivers a (synthetic/injected) network alert on the alert link
//!   (after the link latency).
//! * `truth` and `status` are the state after the last step; `drain_events` returns the
//!   controller, scenario and plant events raised since the previous drain.
//!
//! Additions beyond the frozen surface: [`Session::new_device`] (a device index for the
//! sensor substreams, for scenes that share one noise seed), [`Session::set_hand`] (a
//! live hand, e.g. the browser engine's pointer) and [`Session::snapshot`].

use fabrail_core::seismic::action_policy::{ActionPolicy, CorePolicy};
use fabrail_params::SystemParams;
use fabrail_plant::interaction::HandInput;
use fabrail_plant::params::PlantParams;
use fabrail_plant::{InitLev, Truth};
use fabrail_rt::ExecSnapshot;
use fabrail_types::{
    FaultSet, LevState, MotionMode, NetworkAlert, PosNm, SafetyDemand, SeismicLevel, SeismicState,
    Tick, UserCommand,
};

use crate::engine::{Engine, EngineOpts, HoldFloor};
use crate::recorder::EventRec;
use crate::scenario::{ScenarioSpec, SimError, check_compat};

/// Controller-side status of a session after the last step (what the building-scene
/// choreographer sees of a device).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SessionStatus {
    /// Tick of the last completed step.
    pub tick: Tick,
    /// Levitation state machine.
    pub lev: LevState,
    /// Seismic state machine.
    pub seismic: SeismicState,
    /// Detector level.
    pub level: SeismicLevel,
    /// Motion mode in force.
    pub mode: MotionMode,
    /// Monitor demand in force.
    pub demand: SafetyDemand,
    /// Active faults (core + monitor).
    pub faults: FaultSet,
    /// Measured position (encoder).
    pub x: PosNm,
    /// Estimated velocity, m/s.
    pub v_mps: f32,
    /// Estimated mass m̂, kg.
    pub m_hat_kg: f32,
    /// Obstruction detected.
    pub obstruction: bool,
    /// The last motion command reached its target.
    pub arrived: bool,
    /// Bus power drawn by this device (truth), W.
    pub p_bus_w: f64,
    /// Usable hold-up energy remaining (truth), J.
    pub e_hold_j: f64,
}

/// One steppable device.
#[derive(Clone, Debug)]
pub struct Session {
    plant: PlantParams,
    engine: Box<Engine>,
    status: SessionStatus,
    floor: (Tick, [f64; 3]),
}

impl Session {
    /// A session for parameter set `p` running scenario `spec` (see the module docs).
    pub fn new(p: &SystemParams, spec: &ScenarioSpec) -> Result<Session, SimError> {
        Session::new_device(p, spec, 0)
    }

    /// [`Session::new`] for device `device` of a scene (the sensor-noise substreams
    /// carry the device index, `rng::substream_key`).
    pub fn new_device(
        p: &SystemParams,
        spec: &ScenarioSpec,
        device: u16,
    ) -> Result<Session, SimError> {
        p.validate()?;
        spec.validate()?;
        check_compat(p, spec)?;
        if !spec.series.is_empty() {
            return Err(SimError::Invalid {
                path: "series".into(),
                reason:
                    "a Session takes the floor acceleration from step_to; series are not supported"
                        .into(),
            });
        }
        let env = fabrail_plant::params::Environment {
            rail: None,
            structure: spec.plant.structure.as_ref().map(|s| s.to_shear_building()),
        };
        let plant = PlantParams::from_design(&p.design, &spec.plant.mismatch(), &env)
            .with_control(&p.control);
        let rep =
            fabrail_plant::stability::report_tier(&plant, spec.tier(), spec.tier().nominal_h_s());
        if !rep.ok() {
            let (element, lambda_h) = rep
                .worst_oscillatory()
                .map_or((String::new(), f64::NAN), |e| {
                    (e.element.clone(), e.lambda_h)
                });
            return Err(SimError::Unstable { element, lambda_h });
        }
        let engine = Engine::new(
            p,
            spec,
            &plant,
            EngineOpts {
                record: false,
                metrics: false,
                trace: false,
                device,
            },
        )?;
        let init = spec.initial.resolve(p);
        let truth = *engine.truth();
        let lev = match init.lev {
            InitLev::Seated => LevState::Seated,
            InitLev::Landed => LevState::Landed,
            InitLev::Levitated => LevState::LevitatedIdle,
        };
        let status = SessionStatus {
            tick: Tick::ZERO,
            lev,
            seismic: SeismicState::Normal,
            level: SeismicLevel::L0,
            mode: MotionMode::Off,
            demand: SafetyDemand::None,
            faults: FaultSet::EMPTY,
            x: PosNm::from_m_f64(init.x_m),
            v_mps: init.v_mps as f32,
            m_hat_kg: truth.mass_kg as f32,
            obstruction: false,
            arrived: true,
            p_bus_w: truth.power.p_bus_w,
            e_hold_j: truth.power.e_hold_j,
        };
        Ok(Session {
            plant,
            engine: Box::new(engine),
            status,
            floor: (Tick::ZERO, [0.0; 3]),
        })
    }

    /// Advances to base tick `tick` with the core seismic action policy (see the module
    /// docs).
    pub fn step_to(
        &mut self,
        tick: Tick,
        floor_mps2: &[f64; 3],
        cmd: Option<UserCommand>,
    ) -> Result<(), SimError> {
        self.step_to_with_policy(tick, floor_mps2, cmd, &mut CorePolicy)
    }

    /// Advances to base tick `tick`; the supervisor consults `policy` for the seismic
    /// action (see the module docs).
    pub fn step_to_with_policy(
        &mut self,
        tick: Tick,
        floor_mps2: &[f64; 3],
        cmd: Option<UserCommand>,
        policy: &mut dyn ActionPolicy,
    ) -> Result<(), SimError> {
        if !floor_mps2.iter().all(|a| a.is_finite()) {
            return Err(SimError::Runtime(format!(
                "step_to: non-finite floor acceleration {floor_mps2:?}"
            )));
        }
        if let Some(c) = cmd {
            self.engine.push_command(c);
        }
        let now = self.engine.tick();
        if tick < now {
            return Ok(());
        }
        if tick == now {
            self.floor = (tick, *floor_mps2);
            return Ok(());
        }
        let hold = HoldFloor {
            t0: self.floor.0.seconds_f64(),
            f0: self.floor.1,
            t1: tick.seconds_f64(),
            f1: *floor_mps2,
        };
        while self.engine.tick() < tick {
            self.engine.step(&hold, policy)?;
        }
        self.floor = (tick, *floor_mps2);
        self.status = self.status_now();
        Ok(())
    }

    fn status_now(&self) -> SessionStatus {
        let s = self.engine.snapshot();
        let tr = self.engine.truth();
        let fs = &s.fast.status;
        let mut faults = s.sup.status.faults;
        faults.merge(s.fast.verdict.faults);
        faults.merge(fs.faults);
        SessionStatus {
            tick: self.engine.tick(),
            lev: s.sup.status.lev,
            seismic: s.seis.report.state,
            level: s.seis.report.level,
            mode: s.sup.status.mode,
            demand: s.fast.verdict.demand,
            faults,
            x: fs.x,
            v_mps: fs.v_mps,
            m_hat_kg: fs.m_hat_kg,
            obstruction: fs.obstruction,
            arrived: fs.arrived,
            p_bus_w: tr.power.p_bus_w,
            e_hold_j: tr.power.e_hold_j,
        }
    }

    /// Queues a network alert for the alert link.
    pub fn push_alert(&mut self, alert: NetworkAlert) {
        self.engine.push_alert(alert);
    }

    /// Puts a live hand on the front from the next step (`None` releases it; scenario
    /// hand events apply only while no live hand is set).
    pub fn set_hand(&mut self, hand: Option<HandInput>) {
        self.engine.set_hand(hand);
    }

    /// Tick of the last completed step.
    pub fn tick(&self) -> Tick {
        self.status.tick
    }

    /// Plant truth after the last step.
    pub fn truth(&self) -> &Truth {
        self.engine.truth()
    }

    /// Controller status after the last step.
    pub fn status(&self) -> SessionStatus {
        self.status
    }

    /// The executive's latest published outputs (directive, statuses, report).
    pub fn snapshot(&self) -> &ExecSnapshot {
        self.engine.snapshot()
    }

    /// Events raised since the previous drain, in time order.
    pub fn drain_events(&mut self) -> Vec<EventRec> {
        let mut v = core::mem::take(&mut self.engine.log);
        v.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
        v
    }

    /// The scenario this session runs.
    pub fn spec(&self) -> &ScenarioSpec {
        self.engine.spec()
    }

    /// The plant truth parameters.
    pub fn plant_params(&self) -> &PlantParams {
        &self.plant
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> SystemParams {
        let mut p = fabrail_params::presets::door();
        p.control.rates.coil_mode_voltage = false;
        p.design.carriage.m_star_kg = 100.0;
        p.design.rail.x_closed_m = 0.0;
        p.design.rail.travel_m = 1.0;
        p.design.rest_policy = fabrail_types::RestPolicy::Hover;
        p
    }

    fn spec(extra: &str) -> ScenarioSpec {
        let s = format!(
            r#"{{"schema": "fabrail.scenario/1", "name": "s", "duration_s": 10, "noise_seed": 7,
                "fidelity": {{"model": "travel1d", "coil": "lag"}}{extra}}}"#
        );
        ScenarioSpec::from_json(&s).expect("valid spec")
    }

    #[test]
    fn session_steps_forward_and_reports_status() {
        let mut s = Session::new(&params(), &spec("")).expect("session");
        assert_eq!(s.status().lev, LevState::LevitatedIdle);
        s.step_to(Tick(200), &[0.1, 0.0, 0.0], Some(UserCommand::Close))
            .expect("step");
        assert_eq!(s.tick(), Tick(200));
        assert!((s.truth().t_s - 0.01).abs() < 1e-15);
        assert_eq!(
            s.truth().floor_acc_mps2,
            [0.1, 0.0, 0.0],
            "FOH reaches the new value"
        );
        s.step_to(Tick(100), &[0.0; 3], None)
            .expect("no-op for time");
        assert_eq!(s.tick(), Tick(200));
        assert!(s.step_to(Tick(400), &[f64::NAN, 0.0, 0.0], None).is_err());
        let _ = s.drain_events();
        assert!(s.drain_events().is_empty());
    }

    #[test]
    fn session_rejects_series_and_coil_mismatch() {
        let with_series =
            spec(r#", "series": {"floor": {"axis": "time", "fs_hz": 100, "channels": ["ax"]}}"#);
        let e = Session::new(&params(), &with_series).expect_err("series");
        assert!(e.to_string().starts_with("series:"), "{e}");
        let mut p = params();
        p.control.rates.coil_mode_voltage = true;
        assert!(matches!(
            Session::new(&p, &spec("")),
            Err(SimError::CoilModeMismatch { .. })
        ));
    }

    #[test]
    fn session_accepts_an_action_policy() {
        use fabrail_core::seismic::action_policy::{ActionContext, ActionDecision};
        struct Never(u32);
        impl ActionPolicy for Never {
            fn propose(&mut self, _: &ActionContext) -> Option<ActionDecision> {
                self.0 += 1;
                None
            }
        }
        let mut s = Session::new(&params(), &spec("")).expect("session");
        let mut pol = Never(0);
        s.step_to_with_policy(Tick(200), &[0.0; 3], None, &mut pol)
            .expect("step");
        assert_eq!(s.tick(), Tick(200));
    }

    #[test]
    fn session_is_send_and_clone_continues_identically() {
        fn assert_send<T: Send>() {}
        assert_send::<Session>();
        let mut a = Session::new(&params(), &spec("")).expect("session");
        a.step_to(Tick(400), &[0.2, 0.0, 0.0], None).expect("step");
        let mut b = a.clone();
        a.step_to(Tick(1000), &[0.0, 0.1, 0.0], Some(UserCommand::Open))
            .expect("step");
        b.step_to(Tick(1000), &[0.0, 0.1, 0.0], Some(UserCommand::Open))
            .expect("step");
        assert_eq!(a.truth(), b.truth());
        assert_eq!(a.status(), b.status());
    }

    #[test]
    fn session_chunking_matches_a_single_step() {
        // With a held floor, one long chunk and many 200-tick chunks are bit-identical
        // (the first-order hold of a constant is exact).
        let f = [0.5, 0.0, 0.1];
        let mut one = Session::new(&params(), &spec("")).expect("session");
        one.step_to(Tick(200), &f, None).expect("step");
        one.step_to(Tick(2000), &f, None).expect("step");
        let mut many = Session::new(&params(), &spec("")).expect("session");
        for c in 1..=10u64 {
            many.step_to(Tick(200 * c), &f, None).expect("step");
        }
        assert_eq!(one.truth(), many.truth());
        assert_eq!(one.status(), many.status());
    }
}
