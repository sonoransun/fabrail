//! Scenario event executor (commands, hand profiles, payload, obstacles, faults, alerts, power loss). Owner: A7.
//!
//! Events apply in `ScenarioSpec::schedule()` order (by tick, ties in document order).
//! Continuous profiles (hand, lean, smoke_dp, temp_ramp, obstacle presence, mains
//! outage) are evaluated at every plant step from their start tick; discrete events
//! (commands, alerts, faults, reboots, core faults) are returned to the engine as
//! [`Discrete`] actions at their tick. Every applied event is logged as an
//! [`EventRec`] of kind `"scenario:<kind>"` (PHYSICS §22 for the human and disturbance
//! profiles).
//!
//! Profiles (plan §2b item 11 for the disturbance shapes):
//!
//! * hand: piecewise-linear driving force over `t_pts_s` (relative to `t_s`, the first
//!   value held before the first point) through the arm impedance; released after the
//!   last point;
//! * lean: symmetric trapezoid rising over `rise_s` to `f_n`, back to 0 at
//!   `t_s + dur_s`;
//! * smoke_dp: ramps over `ramp_s`, holds, ramps down over `ramp_s` ending at
//!   `t_s + dur_s` when a duration is given;
//! * temp_ramp: the ambient moves linearly by `dt_k` over `dur_s` (0 = step);
//! * obstacle: present from `t_s` for `dur_s` (default: to the end).

use fabrail_params::SystemParams;
use fabrail_plant::interaction::{HandInput, Obstacle};
use fabrail_plant::{ExtForces, InitialState};
use fabrail_types::{NetworkAlert, PresenceSensing, Tick, UserCommand};

use crate::recorder::EventRec;
use crate::scenario::{
    ControlLossDir, CoreFaultKind, EventSpec, HandContact, ObstacleConfig, RebootCause,
    ScenarioSpec, SensorFaultKind,
};

/// Walks a scenario's schedule and yields the event indices due at each tick.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EventCursor {
    schedule: Vec<(Tick, usize)>,
    next: usize,
}

impl EventCursor {
    /// A cursor at the start of `spec`'s schedule.
    pub fn new(spec: &ScenarioSpec) -> EventCursor {
        EventCursor {
            schedule: spec.schedule(),
            next: 0,
        }
    }

    /// Indices (into `spec.events`) of every not-yet-applied event with tick ≤ `now`,
    /// in application order.
    pub fn due(&mut self, now: Tick) -> Vec<usize> {
        let mut out = Vec::new();
        while let Some(&(t, i)) = self.schedule.get(self.next) {
            if t > now {
                break;
            }
            out.push(i);
            self.next += 1;
        }
        out
    }

    /// True when the next scheduled event is due at or before `now` (allocation-free
    /// fast path for the per-tick loop).
    pub fn any_due(&self, now: Tick) -> bool {
        self.schedule.get(self.next).is_some_and(|&(t, _)| t <= now)
    }

    /// True when every event has been applied.
    pub fn finished(&self) -> bool {
        self.next >= self.schedule.len()
    }
}

/// A discrete action the engine carries out at the event's tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Discrete {
    /// A user command for the command link.
    Command(UserCommand),
    /// A (synthetic/injected) network alert or cancellation for the alert link.
    Alert(NetworkAlert),
    /// A sensor/driver fault (`jump_m` for `encoder_jump`).
    SensorFault {
        /// Fault.
        what: SensorFaultKind,
        /// Channel.
        ch: u8,
        /// Encoder jump, m.
        jump_m: f64,
    },
    /// Controller reboot.
    Reboot(RebootCause),
    /// Core-output fault injection, cleared at `until` (None = to the end).
    CoreFault {
        /// Perturbation.
        what: CoreFaultKind,
        /// Clearing tick.
        until: Option<Tick>,
    },
}

#[derive(Clone, Debug, PartialEq)]
struct HandProfile {
    t0: f64,
    t_end: f64,
    t_pts: Vec<f64>,
    f_pts: Vec<f64>,
    m_kg: f64,
    b: f64,
    k: f64,
    grip: bool,
}

impl HandProfile {
    fn force(&self, t: f64) -> f64 {
        let tau = t - self.t0;
        let (Some(&t_first), Some(&f_first)) = (self.t_pts.first(), self.f_pts.first()) else {
            return 0.0;
        };
        if tau <= t_first {
            return f_first;
        }
        for w in 0..self.t_pts.len().saturating_sub(1) {
            let (ta, tb) = (self.t_pts[w], self.t_pts[w + 1]);
            if tau <= tb {
                let (fa, fb) = (self.f_pts[w], self.f_pts[w + 1]);
                let u = if tb > ta { (tau - ta) / (tb - ta) } else { 1.0 };
                return fa + (fb - fa) * u;
            }
        }
        self.f_pts.last().copied().unwrap_or(0.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Lean {
    t0: f64,
    f_n: f64,
    dur: f64,
    rise: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Smoke {
    t0: f64,
    dp: f64,
    ramp: f64,
    dur: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Ramp {
    t0: f64,
    dt_k: f64,
    dur: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ObstacleWindow {
    t0: f64,
    t_end: f64,
    obstacle: Obstacle,
}

/// Scenario event state of one device.
#[derive(Clone, Debug)]
pub struct EventExecutor {
    cursor: EventCursor,
    events: Vec<EventSpec>,
    hands: Vec<HandProfile>,
    leans: Vec<Lean>,
    smokes: Vec<Smoke>,
    ramps: Vec<Ramp>,
    obstacles: Vec<ObstacleWindow>,
    payload_kg: f64,
    ambient0_c: f64,
    leak_area_m2: Option<f64>,
    mains_off_until: Option<u64>,
    control_loss: Option<ControlLossDir>,
    k_edge_default: f64,
    n_alerts: u32,
    alerts: Vec<NetworkAlert>,
    hand_override: Option<HandInput>,
    hand_override_since: Option<f64>,
    /// Presence-sensing zone occupied.
    pub presence_active: bool,
    /// Presence-sensing device healthy (false when no device is fitted).
    pub presence_ok: bool,
    /// Hold-to-run line actuated.
    pub hold_to_run: bool,
    /// Fire-alarm input.
    pub fire_alarm: bool,
    /// Emergency stop pressed.
    pub estop: bool,
}

/// Seconds → tick, saturating (`Tick::from_seconds_f64` rounds to the nearest tick).
fn ticks(t_s: f64) -> u64 {
    Tick::from_seconds_f64(t_s).0
}

impl EventExecutor {
    /// The executor for `spec` on parameter set `p` starting from `init`.
    pub fn new(spec: &ScenarioSpec, p: &SystemParams, init: &InitialState) -> EventExecutor {
        EventExecutor {
            cursor: EventCursor::new(spec),
            events: spec.events.clone(),
            hands: Vec::new(),
            leans: Vec::new(),
            smokes: Vec::new(),
            ramps: Vec::new(),
            obstacles: Vec::new(),
            payload_kg: init.payload_kg,
            ambient0_c: init.temp_c,
            leak_area_m2: None,
            mains_off_until: None,
            control_loss: None,
            k_edge_default: p.design.damping.k_edge_n_per_m,
            n_alerts: 0,
            alerts: Vec::new(),
            hand_override: None,
            hand_override_since: None,
            presence_active: false,
            presence_ok: p.design.presence == PresenceSensing::PlD,
            hold_to_run: false,
            fire_alarm: false,
            estop: false,
        }
    }

    /// Applies every event due at `now`: continuous profiles are registered, discrete
    /// actions are appended to `out`, and each event is logged in `log`.
    pub fn apply_due(&mut self, now: Tick, out: &mut Vec<Discrete>, log: &mut Vec<EventRec>) {
        if !self.cursor.any_due(now) {
            return;
        }
        let t = now.seconds_f64();
        for i in self.cursor.due(now) {
            let Some(e) = self.events.get(i).cloned() else {
                continue;
            };
            let (a, b) = self.apply_one(&e, now, out);
            log.push(EventRec {
                t_s: t,
                kind: format!("scenario:{}", e.kind()),
                a,
                b,
            });
        }
    }

    /// Applies one event; returns the logged (a, b) payload.
    fn apply_one(&mut self, e: &EventSpec, now: Tick, out: &mut Vec<Discrete>) -> (f64, f64) {
        let t = now.seconds_f64();
        match e {
            EventSpec::Command { cmd, x_m, .. } => {
                out.push(Discrete::Command(cmd.to_user_command(*x_m)));
                (*cmd as u8 as f64, x_m.unwrap_or(0.0))
            }
            EventSpec::Hand {
                t_pts_s,
                f_pts_n,
                impedance,
                contact,
                ..
            } => {
                let dur = t_pts_s.last().copied().unwrap_or(0.0);
                let peak = f_pts_n.iter().fold(0.0f64, |m, f| m.max(f.abs()));
                self.hands.push(HandProfile {
                    t0: t,
                    t_end: t + dur,
                    t_pts: t_pts_s.clone(),
                    f_pts: f_pts_n.clone(),
                    m_kg: impedance.m_kg,
                    b: impedance.b_n_s_per_m,
                    k: impedance.k_n_per_m,
                    grip: *contact == HandContact::Grip,
                });
                (peak, dur)
            }
            EventSpec::Payload { delta_kg, .. } => {
                self.payload_kg += delta_kg;
                (*delta_kg, self.payload_kg)
            }
            EventSpec::Obstacle {
                body,
                x_m,
                config,
                k_edge_n_per_m,
                dur_s,
                ..
            } => {
                let o = Obstacle {
                    body: (*body).into(),
                    x_m: *x_m,
                    clamped: *config == ObstacleConfig::Clamped,
                    k_edge_n_per_m: k_edge_n_per_m.unwrap_or(self.k_edge_default),
                };
                self.obstacles.push(ObstacleWindow {
                    t0: t,
                    t_end: dur_s.map_or(f64::INFINITY, |d| t + d),
                    obstacle: o,
                });
                (*x_m, o.body.k_n_per_m())
            }
            EventSpec::Lean {
                f_n, dur_s, rise_s, ..
            } => {
                self.leans.push(Lean {
                    t0: t,
                    f_n: *f_n,
                    dur: *dur_s,
                    rise: *rise_s,
                });
                (*f_n, *dur_s)
            }
            EventSpec::SmokeDp {
                dp_pa,
                ramp_s,
                dur_s,
                ..
            } => {
                self.smokes.push(Smoke {
                    t0: t,
                    dp: *dp_pa,
                    ramp: *ramp_s,
                    dur: *dur_s,
                });
                (*dp_pa, *ramp_s)
            }
            EventSpec::Piston { leak_area_m2, .. } => {
                self.leak_area_m2 = Some(*leak_area_m2);
                (*leak_area_m2, 0.0)
            }
            EventSpec::TempRamp { dt_k, dur_s, .. } => {
                self.ramps.push(Ramp {
                    t0: t,
                    dt_k: *dt_k,
                    dur: *dur_s,
                });
                (*dt_k, *dur_s)
            }
            EventSpec::PowerLoss { dur_s, .. } => {
                self.mains_off_until = Some(dur_s.map_or(u64::MAX, |d| now.0 + ticks(d)));
                (dur_s.unwrap_or(-1.0), 0.0)
            }
            EventSpec::ControlLoss { dir, .. } => {
                self.control_loss = Some(*dir);
                (
                    match dir {
                        ControlLossDir::Up => 1.0,
                        ControlLossDir::Down => -1.0,
                    },
                    0.0,
                )
            }
            EventSpec::Fault {
                what, ch, jump_m, ..
            } => {
                out.push(Discrete::SensorFault {
                    what: *what,
                    ch: *ch,
                    jump_m: jump_m.unwrap_or(crate::scenario::DEFAULT_ENCODER_JUMP_M),
                });
                (*what as u8 as f64, *ch as f64)
            }
            EventSpec::NetworkAlert { .. } => {
                self.n_alerts += 1;
                if let Some(a) = e.network_alert(self.n_alerts) {
                    self.alerts.push(a);
                    out.push(Discrete::Alert(a));
                    return (a.magnitude as f64, a.r_epi_km as f64);
                }
                (0.0, 0.0)
            }
            EventSpec::AlertCancel { id, .. } => {
                let target = match id {
                    Some(i) => self.alerts.iter().rev().find(|a| a.id == *i).copied(),
                    None => self.alerts.last().copied(),
                };
                let base = target.unwrap_or(NetworkAlert {
                    id: id.unwrap_or(0),
                    ..NetworkAlert::default()
                });
                let c = NetworkAlert {
                    received: now,
                    cancel: true,
                    ..base
                };
                out.push(Discrete::Alert(c));
                (c.id as f64, 0.0)
            }
            EventSpec::Reset { cause, .. } => {
                out.push(Discrete::Reboot(*cause));
                (*cause as u8 as f64, 0.0)
            }
            EventSpec::CoreFault { what, dur_s, .. } => {
                out.push(Discrete::CoreFault {
                    what: *what,
                    until: dur_s.map(|d| Tick(now.0 + ticks(d))),
                });
                (*what as u8 as f64, dur_s.unwrap_or(-1.0))
            }
            EventSpec::Presence { active, ok, .. } => {
                self.presence_active = *active;
                self.presence_ok = *ok;
                (*active as u8 as f64, *ok as u8 as f64)
            }
            EventSpec::HoldToRun { on, .. } => {
                self.hold_to_run = *on;
                (*on as u8 as f64, 0.0)
            }
            EventSpec::FireAlarm { active, .. } => {
                self.fire_alarm = *active;
                (*active as u8 as f64, 0.0)
            }
            EventSpec::Estop { active, .. } => {
                self.estop = *active;
                (*active as u8 as f64, 0.0)
            }
        }
    }

    /// Replaces the scenario hand with a live hand (steppable sessions; `None` hands
    /// control back to the scenario profiles). `t_s` is the time of the change.
    pub fn set_hand(&mut self, hand: Option<HandInput>, t_s: f64) {
        match (&self.hand_override, &hand) {
            (None, Some(_)) => self.hand_override_since = Some(t_s),
            (_, None) => self.hand_override_since = None,
            _ => {}
        }
        self.hand_override = hand;
    }

    /// The hand in contact at `t_s` and its contact start time.
    pub fn hand(&self, t_s: f64) -> Option<(HandInput, f64)> {
        if let (Some(h), Some(t0)) = (self.hand_override, self.hand_override_since) {
            return Some((h, t0));
        }
        // The most recently started hand whose profile covers t wins.
        self.hands
            .iter()
            .rev()
            .find(|h| t_s >= h.t0 && t_s <= h.t_end)
            .map(|h| {
                (
                    HandInput {
                        f_drive_n: h.force(t_s),
                        m_kg: h.m_kg,
                        b_n_s_per_m: h.b,
                        k_n_per_m: h.k,
                        grip: h.grip,
                    },
                    h.t0,
                )
            })
    }

    /// External loads at `t_s` (PHYSICS §22).
    pub fn ext_forces(&self, t_s: f64) -> ExtForces {
        let lean_n = self
            .leans
            .iter()
            .map(|l| {
                let tau = t_s - l.t0;
                if tau < 0.0 || tau > l.dur {
                    return 0.0;
                }
                if l.rise <= 0.0 {
                    return l.f_n;
                }
                l.f_n * (tau / l.rise).min((l.dur - tau) / l.rise).min(1.0)
            })
            .sum();
        let smoke_dp_pa = self
            .smokes
            .iter()
            .map(|s| {
                let tau = t_s - s.t0;
                if tau < 0.0 {
                    return 0.0;
                }
                let up = if s.ramp > 0.0 {
                    (tau / s.ramp).min(1.0)
                } else {
                    1.0
                };
                let down = match s.dur {
                    Some(d) if tau > d => return 0.0,
                    Some(d) if s.ramp > 0.0 => ((d - tau) / s.ramp).min(1.0),
                    _ => 1.0,
                };
                s.dp * up.min(down)
            })
            .sum();
        let obstacle = self
            .obstacles
            .iter()
            .rev()
            .find(|o| t_s >= o.t0 && t_s <= o.t_end)
            .map(|o| o.obstacle);
        ExtForces {
            hand: self.hand(t_s).map(|(h, _)| h),
            lean_n,
            smoke_dp_pa,
            obstacle,
        }
    }

    /// Payload on the body (initial + payload events), kg.
    pub fn payload_kg(&self) -> f64 {
        self.payload_kg
    }

    /// Ambient temperature at `t_s` (initial + temp_ramp events), °C.
    pub fn ambient_c(&self, t_s: f64) -> f64 {
        self.ambient0_c
            + self
                .ramps
                .iter()
                .map(|r| {
                    let tau = t_s - r.t0;
                    let frac = if tau < 0.0 {
                        0.0
                    } else if r.dur <= 0.0 {
                        1.0
                    } else {
                        (tau / r.dur).min(1.0)
                    };
                    r.dt_k * frac
                })
                .sum::<f64>()
    }

    /// Mains present at `now`.
    pub fn mains_ok(&self, now: Tick) -> bool {
        self.mains_off_until.is_none_or(|u| now.0 >= u)
    }

    /// Cavity leak-area override (piston events).
    pub fn leak_area_m2(&self) -> Option<f64> {
        self.leak_area_m2
    }

    /// Coil drivers stuck at full output.
    pub fn control_loss(&self) -> Option<ControlLossDir> {
        self.control_loss
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(events: &str) -> ScenarioSpec {
        ScenarioSpec::from_json(&format!(
            r#"{{"schema": "fabrail.scenario/1", "name": "e", "duration_s": 10, "noise_seed": 1,
                "fidelity": {{"model": "travel1d", "coil": "voltage"}}, "events": [{events}]}}"#
        ))
        .expect("valid")
    }

    #[test]
    fn cursor_yields_each_event_once_in_order() {
        let spec = ScenarioSpec::from_json(
            r#"{"schema": "fabrail.scenario/1", "name": "c", "duration_s": 2, "noise_seed": 1,
                "fidelity": {"model": "carriage6", "coil": "lag"},
                "events": [{"kind": "estop", "t_s": 1.0, "active": true},
                           {"kind": "command", "t_s": 0.5, "cmd": "open"},
                           {"kind": "estop", "t_s": 1.0, "active": false}]}"#,
        )
        .expect("valid");
        let mut c = EventCursor::new(&spec);
        assert!(c.due(Tick(9_999)).is_empty());
        assert!(c.any_due(Tick(10_000)));
        assert_eq!(c.due(Tick(10_000)), vec![1]);
        assert_eq!(c.due(Tick(30_000)), vec![0, 2]);
        assert!(c.finished() && c.due(Tick(40_000)).is_empty());
    }

    #[test]
    fn profiles_follow_their_shapes() {
        let s = spec(
            r#"{"kind": "lean", "t_s": 1, "f_n": 20, "dur_s": 4, "rise_s": 1},
               {"kind": "smoke_dp", "t_s": 1, "dp_pa": 50, "ramp_s": 2, "dur_s": 6},
               {"kind": "temp_ramp", "t_s": 0, "dt_k": 10, "dur_s": 5},
               {"kind": "hand", "t_s": 2, "t_pts_s": [0, 0.1, 0.5], "f_pts_n": [0, 8, 0]},
               {"kind": "payload", "t_s": 3, "delta_kg": 2.5},
               {"kind": "power_loss", "t_s": 4, "dur_s": 1}"#,
        );
        let p = fabrail_params::presets::door();
        let init = s.initial.resolve(&p);
        let mut ex = EventExecutor::new(&s, &p, &init);
        let mut out = Vec::new();
        let mut log = Vec::new();
        for k in 0..=Tick::from_seconds_f64(5.0).0 {
            ex.apply_due(Tick(k), &mut out, &mut log);
        }
        assert_eq!(log.len(), 6);
        assert!(log.iter().all(|e| e.kind.starts_with("scenario:")));
        let f = ex.ext_forces(1.5);
        assert!((f.lean_n - 10.0).abs() < 1e-12, "half-way up the lean ramp");
        assert!(
            (f.smoke_dp_pa - 12.5).abs() < 1e-12,
            "quarter of the smoke ramp"
        );
        assert!((ex.ext_forces(3.0).lean_n - 20.0).abs() < 1e-12);
        assert!(
            (ex.ext_forces(4.5).lean_n - 10.0).abs() < 1e-12,
            "ramping down"
        );
        assert!((ex.ext_forces(6.5).smoke_dp_pa - 12.5).abs() < 1e-12);
        assert_eq!(ex.ext_forces(7.5).smoke_dp_pa, 0.0);
        assert!((ex.ambient_c(2.5) - (init.temp_c + 5.0)).abs() < 1e-12);
        let h = ex.ext_forces(2.05).hand.expect("hand in contact");
        assert!((h.f_drive_n - 4.0).abs() < 1e-9 && h.grip);
        assert!(
            ex.ext_forces(2.6).hand.is_none(),
            "released after the last point"
        );
        assert!((ex.payload_kg() - 2.5).abs() < 1e-12);
        assert!(!ex.mains_ok(Tick::from_seconds_f64(4.5)));
        assert!(ex.mains_ok(Tick::from_seconds_f64(5.0)));
        assert!(out.is_empty(), "no discrete actions in this scenario");
    }

    #[test]
    fn discrete_actions_and_alert_cancel() {
        let s = spec(
            r#"{"kind": "command", "t_s": 0.5, "cmd": "go_to", "x_m": 0.2},
               {"kind": "network_alert", "t_s": 1, "magnitude": 7, "r_epi_km": 20,
                "depth_km": 10, "origin_t_s": -2},
               {"kind": "alert_cancel", "t_s": 2},
               {"kind": "fault", "t_s": 3, "what": "encoder_jump"},
               {"kind": "core_fault", "t_s": 4, "what": "nan", "dur_s": 0.5},
               {"kind": "reset", "t_s": 5}"#,
        );
        let p = fabrail_params::presets::door();
        let mut ex = EventExecutor::new(&s, &p, &s.initial.resolve(&p));
        let (mut out, mut log) = (Vec::new(), Vec::new());
        for k in 0..=Tick::from_seconds_f64(5.0).0 {
            ex.apply_due(Tick(k), &mut out, &mut log);
        }
        assert_eq!(out.len(), 6);
        assert!(matches!(out[0], Discrete::Command(UserCommand::GoTo(x)) if x.0 == 200_000_000));
        let Discrete::Alert(a) = out[1] else {
            panic!("alert expected")
        };
        assert_eq!((a.id, a.cancel), (1, false));
        assert!((a.origin_offset_s + 3.0).abs() < 1e-6);
        let Discrete::Alert(c) = out[2] else {
            panic!("cancel expected")
        };
        assert_eq!((c.id, c.cancel, c.received), (1, true, Tick(40_000)));
        assert!(matches!(out[3], Discrete::SensorFault { jump_m, .. } if jump_m == 1.0e-3));
        assert!(matches!(
            out[4],
            Discrete::CoreFault {
                until: Some(Tick(90_000)),
                ..
            }
        ));
        assert!(matches!(out[5], Discrete::Reboot(RebootCause::Software)));
    }
}
