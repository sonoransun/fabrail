//! ZoneState machine (Unknown, Arming, ClearVerified, Intrusion, Stopping, Stopped), ISO 13855 separation, attended jog (FAB-EXO-501..503). Owner: X1 (API frozen by W0a).
//!
//! EXOTIC §X14, §S1 (exotic design §2.6; ISO 13855 / 13849-1 / 14119 used for numeric
//! limits only, never as a compliance claim).
//!
//! | From | Condition (checked in this order) | To |
//! |---|---|---|
//! | Unknown, Stopped | motion reported | Stopping |
//! | Unknown, Stopped | `clear` | Arming |
//! | Arming | motion reported | Stopping |
//! | Arming | `clear` lost | Unknown |
//! | Arming | rising edge of the outside reset | ClearVerified (motion permitted) |
//! | ClearVerified | `clear` lost (presence or interlock trip) | Intrusion |
//! | Intrusion | moving | Stopping |
//! | Intrusion | at rest | Stopped |
//! | Stopping | stopped | Stopped |
//!
//! * `clear` = both diverse presence channels clear (Type 4 light curtain or Type 3
//!   scanner, plus radar or mats) AND the access interlocks locked (guard locking or
//!   trapped key). Either presence channel alone trips (1oo2 for the trip).
//! * `ClearVerified` needs `clear` plus a **rising edge** of the manual reset from
//!   outside the zone while in `Arming`: a reset held down (or stuck) never arms, and a
//!   press that happened before the zone became clear does not count. Motion is
//!   permitted only in `ClearVerified` (FAB-EXO-501).
//! * Any trip in `ClearVerified` enters `Intrusion` (motion withdrawn on that cycle,
//!   SS1 demanded; SS1/STO as in PHYSICS §18). A moving mover then runs the SS1 ramp
//!   (`Stopping`) and must stop within d_stop = v·t_react + v²/(2·a_brake) + v·a/(2j)
//!   ([`ss1_stop_distance_mm`]); afterwards STO on the LSM plus the class landing
//!   (`Stopped`) (FAB-EXO-502). Motion reported outside `ClearVerified` is treated the
//!   same way (SS1).
//! * **Two-person override** (BASTION, TIDEGATE; FAB-EXO-507): presence sensing wins
//!   over closure unless the class permits the override (`ZoneCfg::two_person_override`)
//!   AND it is active. The override substitutes for the presence channels only; the
//!   interlocks and the outside reset are still required, and releasing the override
//!   with presence not clear trips at once. The caller asserts the override only for a
//!   closing move.
//! * Separation S = K·(t1 + t2) + C with t2 = v/a_brake ([`iso13855_separation_mm`]):
//!   SHIELD 2.61 m, BASTION 5.01 m, ARENA 10.61 m (EXOTIC §2).
//!
//! Outside `ClearVerified` and `Stopping` the LSM is in STO; the landing follows the
//! class safe-state policy. The attended-jog limits live in [`crate::envelope`].

use fabrail_types::Tick;

use crate::types::{ZoneCfg, u8_enum};

u8_enum! {
    /// Guarded-zone state.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum ZoneState {
        /// Not verified (power-up, after a fault).
        #[default]
        Unknown = 0,
        /// Presence channels clear and interlocks locked; awaiting the outside reset.
        Arming = 1,
        /// Verified clear: motion permitted.
        ClearVerified = 2,
        /// A presence channel tripped.
        Intrusion = 3,
        /// SS1 stop in progress.
        Stopping = 4,
        /// Stopped (STO on the LSM, landed); needs a new verification cycle.
        Stopped = 5,
    }
}

/// Zone inputs sampled each safety cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct ZoneInputs {
    /// Presence channel A clear (e.g. Type 4 light curtain).
    pub presence_a_clear: bool,
    /// Presence channel B clear (diverse: scanner, radar or mats).
    pub presence_b_clear: bool,
    /// Access interlocks locked (guard locking or trapped key).
    pub interlocks_locked: bool,
    /// Manual reset pressed from outside the zone (edge).
    pub reset_outside: bool,
    /// Two-person authority override active.
    pub two_person_override: bool,
    /// The mover is in motion.
    pub motion_active: bool,
    /// The mover has come to rest after a stop.
    pub stopped: bool,
}

/// The guarded-zone state machine.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct ZoneMachine {
    cfg: ZoneCfg,
    state: ZoneState,
    /// Tick the current state was entered.
    since: Tick,
    /// Reset input on the previous cycle (rising-edge detection).
    reset_prev: bool,
    /// The two-person override substituted for presence sensing on the last cycle.
    overridden: bool,
}

impl ZoneMachine {
    /// A machine in `Unknown`.
    pub const fn new(cfg: ZoneCfg) -> ZoneMachine {
        ZoneMachine {
            cfg,
            state: ZoneState::Unknown,
            since: Tick(0),
            reset_prev: false,
            overridden: false,
        }
    }

    /// Steps at `t` with inputs `inp`; returns the new state.
    pub fn step(&mut self, t: Tick, inp: &ZoneInputs) -> ZoneState {
        let override_on = self.cfg.two_person_override && inp.two_person_override;
        let presence_clear = inp.presence_a_clear && inp.presence_b_clear;
        let clear = (presence_clear || override_on) && inp.interlocks_locked;
        let reset_edge = inp.reset_outside && !self.reset_prev;
        self.reset_prev = inp.reset_outside;
        self.overridden = override_on && !presence_clear;

        let next = match self.state {
            ZoneState::Unknown | ZoneState::Stopped => {
                if inp.motion_active {
                    ZoneState::Stopping
                } else if clear {
                    ZoneState::Arming
                } else {
                    self.state
                }
            }
            ZoneState::Arming => {
                if inp.motion_active {
                    ZoneState::Stopping
                } else if !clear {
                    ZoneState::Unknown
                } else if reset_edge {
                    ZoneState::ClearVerified
                } else {
                    ZoneState::Arming
                }
            }
            ZoneState::ClearVerified => {
                if clear {
                    ZoneState::ClearVerified
                } else {
                    ZoneState::Intrusion
                }
            }
            ZoneState::Intrusion => {
                if inp.motion_active && !inp.stopped {
                    ZoneState::Stopping
                } else {
                    ZoneState::Stopped
                }
            }
            ZoneState::Stopping => {
                if inp.stopped || !inp.motion_active {
                    ZoneState::Stopped
                } else {
                    ZoneState::Stopping
                }
            }
        };
        if next != self.state {
            self.state = next;
            self.since = t;
        }
        self.state
    }

    /// The current state.
    pub const fn state(&self) -> ZoneState {
        self.state
    }

    /// The configuration.
    pub const fn cfg(&self) -> ZoneCfg {
        self.cfg
    }

    /// Tick the current state was entered.
    pub const fn entered(&self) -> Tick {
        self.since
    }

    /// Motion is permitted only in `ClearVerified` (FAB-EXO-501).
    pub const fn motion_permitted(&self) -> bool {
        matches!(self.state, ZoneState::ClearVerified)
    }

    /// SS1 (controlled stop, then STO) is demanded: `Intrusion` or `Stopping`.
    pub const fn ss1_demanded(&self) -> bool {
        matches!(self.state, ZoneState::Intrusion | ZoneState::Stopping)
    }

    /// The two-person override is currently standing in for the presence channels.
    pub const fn override_active(&self) -> bool {
        self.overridden
    }
}

/// ISO 13855 separation S = K·(t1 + t2) + C with t2 = v/a_brake, mm (integer; t2
/// rounded down to whole ms). `cfg.a_brake_mmps2 = 0` means an unbounded stop time and
/// returns `u64::MAX`.
pub const fn iso13855_separation_mm(cfg: &ZoneCfg, v_mmps: u32) -> u64 {
    if cfg.a_brake_mmps2 == 0 {
        return u64::MAX;
    }
    let t2_ms = (v_mmps as u64 * 1000) / cfg.a_brake_mmps2 as u64;
    (cfg.k_mmps as u64 * (cfg.t1_ms as u64 + t2_ms)) / 1000 + cfg.c_mm as u64
}

/// SS1 stop distance d = v·t_react + v²/(2·a_brake) + v·a/(2j), mm (integer, rounded
/// up; `u64::MAX` for a zero deceleration or jerk). The jerk term bounds the extra
/// travel while the deceleration ramps to a = a_brake: the exact jerk-limited distance
/// is v²/(2a) + v·a/(2j) − a³/(24j²) (EXOTIC §X14; jerk-limited profiles as in
/// PHYSICS §16), so this is an upper bound.
pub const fn ss1_stop_distance_mm(
    v_mmps: u32,
    t_react_ms: u32,
    a_brake_mmps2: u32,
    a_mmps2: u32,
    j_mmps3: u32,
) -> u64 {
    if a_brake_mmps2 == 0 || j_mmps3 == 0 {
        return u64::MAX;
    }
    let v = v_mmps as u128;
    let react = (v * t_react_ms as u128).div_ceil(1000);
    let brake = (v * v).div_ceil(2 * a_brake_mmps2 as u128);
    let jerk = (v * a_mmps2 as u128).div_ceil(2 * j_mmps3 as u128);
    let d = react + brake + jerk;
    if d > u64::MAX as u128 {
        u64::MAX
    } else {
        d as u64
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::presets;
    use crate::types::{ExoClass, SafetyRegime};

    fn cfg(a_brake: u32) -> ZoneCfg {
        ZoneCfg {
            k_mmps: 1600,
            t1_ms: 100,
            c_mm: 850,
            a_brake_mmps2: a_brake,
            ..ZoneCfg::default()
        }
    }

    fn zone_cfg(c: ExoClass) -> ZoneCfg {
        match presets::preset(c).regime {
            SafetyRegime::GuardedZone(z) => z,
            r => panic!("{c:?} is not a guarded zone: {r:?}"),
        }
    }

    const CLEAR: ZoneInputs = ZoneInputs {
        presence_a_clear: true,
        presence_b_clear: true,
        interlocks_locked: true,
        reset_outside: false,
        two_person_override: false,
        motion_active: false,
        stopped: false,
    };

    /// Drives a fresh machine to ClearVerified: clear, then a reset press and release.
    fn verified(c: ZoneCfg) -> (ZoneMachine, u64) {
        let mut z = ZoneMachine::new(c);
        assert_eq!(z.step(Tick(0), &CLEAR), ZoneState::Arming);
        let press = ZoneInputs {
            reset_outside: true,
            ..CLEAR
        };
        assert_eq!(z.step(Tick(1), &press), ZoneState::ClearVerified);
        assert_eq!(z.step(Tick(2), &CLEAR), ZoneState::ClearVerified);
        (z, 3)
    }

    #[test]
    fn iso13855_anchor_distances() {
        // EXOTIC §2: SHIELD 2.61 m (0.5 m/s, t2 1 s), BASTION 5.01 m (1.2 m/s at
        // 0.48 m/s², t2 2.5 s), ARENA 10.61 m (0.3 m/s at 0.05 m/s², t2 6 s).
        assert_eq!(iso13855_separation_mm(&cfg(500), 500), 2610);
        assert_eq!(iso13855_separation_mm(&cfg(480), 1200), 5010);
        assert_eq!(iso13855_separation_mm(&cfg(50), 300), 10610);
        assert_eq!(iso13855_separation_mm(&cfg(0), 300), u64::MAX);
    }

    /// Every input combination as a `ZoneInputs` (7 booleans → 128).
    fn all_inputs() -> impl Iterator<Item = ZoneInputs> {
        (0u8..128).map(|m| ZoneInputs {
            presence_a_clear: m & 1 != 0,
            presence_b_clear: m & 2 != 0,
            interlocks_locked: m & 4 != 0,
            reset_outside: m & 8 != 0,
            two_person_override: m & 16 != 0,
            motion_active: m & 32 != 0,
            stopped: m & 64 != 0,
        })
    }

    // req: FAB-EXO-501
    #[test]
    fn motion_only_when_clear_verified() {
        // Exhaustive reachability over (machine state incl. the reset latch) × 128
        // inputs, for a class without and with the two-person override.
        for c in [zone_cfg(ExoClass::Shield), zone_cfg(ExoClass::Bastion)] {
            let mut seen = std::collections::HashSet::new();
            let mut frontier = vec![ZoneMachine::new(c)];
            let mut n_edges = 0usize;
            while let Some(m) = frontier.pop() {
                if !seen.insert((m.state, m.reset_prev)) {
                    continue;
                }
                for inp in all_inputs() {
                    let mut z = m;
                    let s = z.step(Tick(1), &inp);
                    n_edges += 1;
                    assert_eq!(
                        z.motion_permitted(),
                        s == ZoneState::ClearVerified,
                        "motion permitted only in ClearVerified"
                    );
                    let override_on = c.two_person_override && inp.two_person_override;
                    let presence = inp.presence_a_clear && inp.presence_b_clear;
                    if s == ZoneState::ClearVerified {
                        // Two diverse presence channels clear (or the permitted override),
                        // interlocks locked, and either already verified or a fresh
                        // outside reset edge out of Arming.
                        assert!(presence || override_on, "{m:?} {inp:?}");
                        assert!(inp.interlocks_locked, "{m:?} {inp:?}");
                        assert!(
                            m.state == ZoneState::ClearVerified
                                || (m.state == ZoneState::Arming
                                    && inp.reset_outside
                                    && !m.reset_prev),
                            "entered ClearVerified from {m:?} with {inp:?}"
                        );
                    }
                    if m.state == ZoneState::ClearVerified && !(presence || override_on) {
                        assert_eq!(
                            s,
                            ZoneState::Intrusion,
                            "presence trip leaves ClearVerified"
                        );
                    }
                    if m.state == ZoneState::ClearVerified && !inp.interlocks_locked {
                        assert_eq!(
                            s,
                            ZoneState::Intrusion,
                            "interlock trip leaves ClearVerified"
                        );
                    }
                    frontier.push(z);
                }
            }
            assert!(
                seen.len() >= 12,
                "reachable (state, latch) pairs: {}",
                seen.len()
            );
            assert!(n_edges >= 12 * 128);
        }

        // A reset held down through arming (stuck button) never verifies; a single
        // presence channel clear never arms; unlocked interlocks never arm.
        let c = zone_cfg(ExoClass::Arena);
        let mut z = ZoneMachine::new(c);
        let held = ZoneInputs {
            reset_outside: true,
            ..CLEAR
        };
        for t in 0..100 {
            z.step(Tick(t), &held);
            assert!(!z.motion_permitted(), "held reset must not arm");
        }
        for inp in [
            ZoneInputs {
                presence_b_clear: false,
                ..CLEAR
            },
            ZoneInputs {
                presence_a_clear: false,
                ..CLEAR
            },
            ZoneInputs {
                interlocks_locked: false,
                ..CLEAR
            },
        ] {
            let mut z = ZoneMachine::new(c);
            for t in 0..10 {
                let r = ZoneInputs {
                    reset_outside: t % 2 == 1,
                    ..inp
                };
                z.step(Tick(t), &r);
                assert!(!z.motion_permitted(), "{inp:?}");
                assert_eq!(z.state(), ZoneState::Unknown);
            }
        }
        // After a stop a full new verification cycle is required.
        let (mut z, t) = verified(c);
        let trip = ZoneInputs {
            presence_a_clear: false,
            ..CLEAR
        };
        assert_eq!(z.step(Tick(t), &trip), ZoneState::Intrusion);
        assert_eq!(z.step(Tick(t + 1), &trip), ZoneState::Stopped);
        assert_eq!(z.step(Tick(t + 2), &CLEAR), ZoneState::Arming);
        assert!(!z.motion_permitted());
    }

    // req: FAB-EXO-502
    #[test]
    fn intrusion_ss1_stop_distance() {
        // Per class: the ISO 13855 separation at v_max is reported (EXOTIC §2, exact),
        // and an intrusion at full speed stops within the analytic d_stop.
        for (class, s_expected_mm) in [
            (ExoClass::Shield, 2610),
            (ExoClass::Bastion, 5010),
            (ExoClass::Arena, 10_610),
        ] {
            let p = presets::preset(class);
            let c = zone_cfg(class);
            let v = p.kinematics.v_max_mmps;
            let s_mm = iso13855_separation_mm(&c, v);
            assert_eq!(
                s_mm, s_expected_mm,
                "{class:?}: ISO 13855 S (EXOTIC §2, exact)"
            );

            // Braking at a_brake with the class jerk limit; t_react = t1 (detection +
            // response, 100 ms).
            let (a_b, j) = (c.a_brake_mmps2 as f64, p.kinematics.j_max_mmps3 as f64);
            let d_bound = ss1_stop_distance_mm(
                v,
                c.t1_ms,
                c.a_brake_mmps2,
                c.a_brake_mmps2,
                p.kinematics.j_max_mmps3,
            );

            // Safety cycle 1 ms (20 base ticks); mover integrated in f64 mm at 0.1 ms.
            let (mut z, mut t) = verified(c);
            let mut x = 0.0f64;
            let mut vel = v as f64;
            let mut decel = 0.0f64;
            let dt = 1.0e-4;
            let t_intrusion = t + 20 * 50; // 50 ms of full-speed motion first
            let mut ss1_since: Option<u64> = None;
            let mut x_at_trip = 0.0;
            let mut stopped_state_at = None;
            while t < 20 * 60_000 {
                let moving = vel > 0.0;
                let inp = ZoneInputs {
                    presence_a_clear: t < t_intrusion,
                    motion_active: moving,
                    stopped: !moving,
                    ..CLEAR
                };
                let s = z.step(Tick(t), &inp);
                if t == t_intrusion {
                    assert_eq!(
                        s,
                        ZoneState::Intrusion,
                        "{class:?}: trip on the intrusion cycle"
                    );
                    assert!(!z.motion_permitted() && z.ss1_demanded());
                    x_at_trip = x;
                }
                if z.ss1_demanded() && ss1_since.is_none() {
                    ss1_since = Some(t);
                }
                if s == ZoneState::Stopped {
                    stopped_state_at = Some(t);
                    break;
                }
                // 20 base ticks = 1 ms per safety cycle.
                for _ in 0..10 {
                    let react_done = ss1_since
                        .is_some_and(|s0| (t - s0) as f64 / 20_000.0 >= c.t1_ms as f64 / 1000.0);
                    if react_done && vel > 0.0 {
                        decel = (decel + j * dt).min(a_b);
                        vel = (vel - decel * dt).max(0.0);
                    }
                    x += vel * dt;
                }
                t += 20;
            }
            assert!(
                stopped_state_at.is_some(),
                "{class:?}: never reached Stopped"
            );
            assert_eq!(
                ss1_since,
                Some(t_intrusion),
                "{class:?}: SS1 on the trip cycle"
            );
            let d = x - x_at_trip;
            assert!(
                d <= d_bound as f64 + 1.0,
                "{class:?}: stop distance {d:.1} mm > d_stop {d_bound} mm (+1 mm integration, EXOTIC §X14)"
            );
            // The bound is tight to the a³/(24j²) term plus the reaction-sampling slack.
            let slack = a_b.powi(3) / (24.0 * j * j) + v as f64 * 2.0e-3 + 2.0;
            assert!(
                d >= d_bound as f64 - slack,
                "{class:?}: {d:.1} mm vs bound {d_bound} mm (slack {slack:.1} mm)"
            );
        }
        assert_eq!(ss1_stop_distance_mm(0, 100, 500, 100, 500), 0);
        assert_eq!(ss1_stop_distance_mm(500, 100, 0, 100, 500), u64::MAX);
    }

    // req: FAB-EXO-507
    #[test]
    fn presence_wins_without_two_person_override() {
        let closing = ZoneInputs {
            motion_active: true,
            ..CLEAR
        };
        let person = ZoneInputs {
            presence_b_clear: false,
            ..closing
        };
        let person_override = ZoneInputs {
            two_person_override: true,
            ..person
        };

        // BASTION and TIDEGATE permit the override; SHIELD and ARENA do not.
        for (class, permits) in [
            (ExoClass::Bastion, true),
            (ExoClass::Tidegate, true),
            (ExoClass::Shield, false),
            (ExoClass::Arena, false),
        ] {
            let c = zone_cfg(class);
            assert_eq!(c.two_person_override, permits, "{class:?} preset");

            // Without the override, presence wins: a person in the zone stops the close.
            let (mut z, t) = verified(c);
            z.step(Tick(t), &closing);
            assert_eq!(
                z.step(Tick(t + 1), &person),
                ZoneState::Intrusion,
                "{class:?}"
            );
            assert!(!z.motion_permitted() && z.ss1_demanded());

            // With the override asserted: only the permitting classes keep closing.
            let (mut z, t) = verified(c);
            z.step(Tick(t), &closing);
            let s = z.step(Tick(t + 1), &person_override);
            if permits {
                assert_eq!(
                    s,
                    ZoneState::ClearVerified,
                    "{class:?}: override keeps closing"
                );
                assert!(z.motion_permitted() && z.override_active());
                // Releasing the override with the person still present trips at once.
                assert_eq!(z.step(Tick(t + 2), &person), ZoneState::Intrusion);
            } else {
                assert_eq!(s, ZoneState::Intrusion, "{class:?}: override not permitted");
                assert!(!z.override_active());
            }

            // The override covers presence only: an open interlock still trips.
            let (mut z, t) = verified(c);
            let unlocked = ZoneInputs {
                interlocks_locked: false,
                ..person_override
            };
            assert_eq!(
                z.step(Tick(t), &unlocked),
                ZoneState::Intrusion,
                "{class:?}"
            );

            // Arming with a person present needs the override plus the outside reset.
            let mut z = ZoneMachine::new(c);
            let still = ZoneInputs {
                presence_a_clear: false,
                ..CLEAR
            };
            z.step(Tick(0), &still);
            z.step(
                Tick(1),
                &ZoneInputs {
                    reset_outside: true,
                    ..still
                },
            );
            assert!(!z.motion_permitted(), "{class:?}: presence blocks arming");
            let ov = ZoneInputs {
                two_person_override: true,
                ..still
            };
            let mut z = ZoneMachine::new(c);
            z.step(Tick(0), &ov);
            let s = z.step(
                Tick(1),
                &ZoneInputs {
                    reset_outside: true,
                    ..ov
                },
            );
            assert_eq!(s == ZoneState::ClearVerified, permits, "{class:?}");
        }
    }
}
