//! LevState transition table, fault latch and refuse-to-lift on parameter CRC mismatch (PHYSICS §18; SAF-013, SAF-015). Owner: A4.
//!
//! Frozen API (W0): [`LevInputs`], [`transition`] (pure), [`rest_state`],
//! [`is_levitated`]. Main path BOOT → SELFTEST (golden hash, parameter CRC, referenced)
//! → rest state by `RestPolicy` (SEATED for DRAWER, LEVITATED_IDLE for DOOR, LANDED for
//! WALL/HEAVY); SEATED/LANDED → LIFTING → LEVITATED_IDLE ⇄ MOVING → STOPPING →
//! (SEATING | LANDING) → rest; any state → FAULT_REACTION (SS1 Safe Landing, STO on the
//! LSM only) → FAULT (latched; `Reset` clears it); side branches SEISMIC, ESCAPE, MANUAL.
//! Tests: `transition_table`, `fault_latch`, `refuse_lift_on_crc`.
//!
//! The table (the supervisor derives every input each release; `transition` is pure):
//!
//! | From | Condition (first match wins) | To |
//! |---|---|---|
//! | any but FAULT_REACTION, FAULT | demand ≥ SafeStop | FAULT_REACTION |
//! | BOOT | always | SELFTEST |
//! | SELFTEST | CRC or selftest-hash failure (refuse to lift, SAF-015) | FAULT_REACTION |
//! | SELFTEST | no fast status yet | SELFTEST |
//! | SELFTEST | fast group already levitated (warm start) | LEVITATED_IDLE |
//! | SELFTEST | SEAT: seated / not seated | SEATED / LANDED |
//! | SELFTEST | HOVER: lift permitted / not | LIFTING / LANDED |
//! | SELFTEST | LAND, LAND_LOCK | LANDED |
//! | SEATED, LANDED | re-selftest (after the INSPECT reset) | SELFTEST |
//! | SEATED, LANDED | manual request | MANUAL |
//! | LANDED | SEAT class re-seated by a push (F17) | SEATED |
//! | SEATED | seat reports Failed | LANDED |
//! | SEATED, LANDED | (lift or move request) and lift permitted | LIFTING |
//! | LIFTING | alert, land/manual request or timeout (abort) | LANDING |
//! | LIFTING | levitated (SEAT: out of the seat) | MOVING if a move is pending, else LEVITATED_IDLE |
//! | LEVITATED_IDLE, MOVING, STOPPING | alert with a class action | SEISMIC |
//! | LEVITATED_IDLE, MOVING, STOPPING | escape trigger | ESCAPE |
//! | LEVITATED_IDLE | land/manual request | SEATING (SEAT at the seat) or LANDING |
//! | LEVITATED_IDLE | move request | MOVING |
//! | MOVING | stop/land/manual request | STOPPING |
//! | MOVING | arrived | SEATING (SEAT at the seat), LANDING (LAND, LAND_LOCK), else LEVITATED_IDLE |
//! | MOVING | overdue | STOPPING |
//! | STOPPING | stopped (or timeout): land/manual → SEATING/LANDING; move → MOVING; SEAT at the seat → SEATING | else LEVITATED_IDLE |
//! | LANDING | touchdown | MANUAL (requested), SEATED (seat reports Seated) or LANDED |
//! | SEATING | seated / seat failed | SEATED (MANUAL if requested) / FAULT_REACTION |
//! | FAULT_REACTION | touchdown, seated (SEAT) or timeout | FAULT |
//! | FAULT | Reset with no demand ≥ SafeStop and a valid CRC | SELFTEST |
//! | FAULT | manual request | MANUAL |
//! | SEISMIC | escape (admitted OpenAndStop) | ESCAPE |
//! | SEISMIC | touchdown or seated (SEAT) | SEATED or LANDED |
//! | SEISMIC | back to NORMAL (cancel) | LEVITATED_IDLE (stopped) or STOPPING |
//! | ESCAPE | touchdown | LANDED |
//! | MANUAL | Reset or re-selftest | SELFTEST |
//!
//! "Touchdown" means the fast group reports `LevPhase::Off` or `Touchdown` for a
//! directive issued in the current state (`fresh`), so a status that predates the
//! state's first directive (LET delay) can never complete it.

use fabrail_types::{
    FaultSet, LevState, RestPolicy, SafetyDemand, SeismicAction, SeismicState, UserCommand,
};

use crate::msg::{LevPhase, SeatStatus};

/// Inputs to one LevState transition.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LevInputs {
    /// Command accepted this release, if any.
    pub cmd: Option<UserCommand>,
    /// Levitation phase from the fast group.
    pub phase: LevPhase,
    /// Seat status from the fast group.
    pub seat: SeatStatus,
    /// Highest demand in force (monitor and supervisor).
    pub demand: SafetyDemand,
    /// Latched faults.
    pub faults: FaultSet,
    /// Boot selftest hash matched.
    pub selftest_ok: bool,
    /// Parameter CRC and diverse copies valid (else refuse to lift, SAF-015).
    pub crc_ok: bool,
    /// Encoder referenced.
    pub referenced: bool,
    /// Seismic state.
    pub seis: SeismicState,
    /// Current segment arrived.
    pub arrived: bool,
    /// Hold-up energy sufficient (≥ 3 × landing need, SAF-009).
    pub holdup_ok: bool,
    /// A fast status has been received since boot (the fast group is running).
    pub fast_valid: bool,
    /// The fast status was produced under a directive issued in the current state.
    pub fresh: bool,
    /// Seismic action in force (after the `ActionPolicy` admission filter).
    pub action: SeismicAction,
    /// Lift requested (command, retry or hover re-lift).
    pub lift_request: bool,
    /// An accepted move (command, intent, capture) is pending.
    pub move_request: bool,
    /// Stop requested (Stop, obstruction, hold-to-run released, re-plan).
    pub stop_request: bool,
    /// Land requested (Land, hover timeout, hold-up low while levitated).
    pub land_request: bool,
    /// MANUAL requested (entered once the carriage is down).
    pub manual_request: bool,
    /// Escape trigger: fire alarm on a non-rated egress door, or admitted OpenAndStop.
    pub escape: bool,
    /// |v| below the stall speed.
    pub stopped: bool,
    /// Within the closed-position tolerance of the seat / closed stop.
    pub at_closed: bool,
    /// The current state's time limit expired.
    pub timeout: bool,
    /// Re-run SELFTEST from a rest state (after the INSPECT reset returns to NORMAL).
    pub selftest_request: bool,
}

/// The rest state for a rest policy.
pub const fn rest_state(r: RestPolicy) -> LevState {
    match r {
        RestPolicy::Seat => LevState::Seated,
        RestPolicy::Hover => LevState::LevitatedIdle,
        RestPolicy::Land | RestPolicy::LandLock => LevState::Landed,
    }
}

/// True in states where the carriage is (or is becoming) levitated.
pub const fn is_levitated(s: LevState) -> bool {
    matches!(
        s,
        LevState::Lifting
            | LevState::LevitatedIdle
            | LevState::Moving
            | LevState::Stopping
            | LevState::Landing
            | LevState::Seating
            | LevState::Seismic
            | LevState::Escape
            | LevState::FaultReaction
    )
}

/// Lift is permitted: valid CRC and selftest (SAF-015), hold-up ≥ 3× the landing need
/// (SAF-009), seismic NORMAL (seated drawers never unseat on an alert; restore only
/// after the INSPECT reset, plan §4b) and no fault reaction in force.
pub fn lift_permitted(i: &LevInputs) -> bool {
    i.crc_ok
        && i.selftest_ok
        && i.holdup_ok
        && i.seis == SeismicState::Normal
        && i.demand < SafetyDemand::SafeStop
}

/// Touchdown (or never lifted) reported under a directive of the current state.
fn down(i: &LevInputs) -> bool {
    i.fresh && matches!(i.phase, LevPhase::Off | LevPhase::Touchdown)
}

/// SEAT class reported seated under a directive of the current state (the z-seat step
/// completed, whatever phase the coils report).
fn seated(i: &LevInputs, rest: RestPolicy) -> bool {
    rest == RestPolicy::Seat && i.fresh && i.seat == SeatStatus::Seated
}

/// Where a touchdown ends: SEATED if the seat reports seated, else LANDED.
fn after_touchdown(i: &LevInputs, rest: RestPolicy) -> LevState {
    if rest == RestPolicy::Seat && i.seat == SeatStatus::Seated {
        LevState::Seated
    } else {
        LevState::Landed
    }
}

/// Seat (SEAT class at the closed position) or land in place.
fn seat_or_land(i: &LevInputs, rest: RestPolicy) -> LevState {
    if rest == RestPolicy::Seat && i.at_closed {
        LevState::Seating
    } else {
        LevState::Landing
    }
}

/// An alert with a class action takes a levitated carriage into SEISMIC.
fn seismic(i: &LevInputs) -> bool {
    i.seis != SeismicState::Normal && i.action != SeismicAction::None
}

/// One LevState transition (the table in the module docs).
pub fn transition(s: LevState, i: &LevInputs, rest: RestPolicy) -> LevState {
    use LevState as L;
    // Any state → FAULT_REACTION while a demand ≥ SafeStop is in force (SAF-013, SAF-016).
    if i.demand >= SafetyDemand::SafeStop && !matches!(s, L::FaultReaction | L::Fault) {
        return L::FaultReaction;
    }
    match s {
        L::Boot => L::SelfTest,
        L::SelfTest => {
            if !i.crc_ok || !i.selftest_ok {
                // Refuse to lift (SAF-015): the fault reaction ends in a latched FAULT.
                L::FaultReaction
            } else if !i.fast_valid {
                L::SelfTest
            } else if i.phase == LevPhase::Levitated {
                L::LevitatedIdle
            } else {
                match rest {
                    RestPolicy::Seat => after_touchdown(i, rest),
                    RestPolicy::Hover => {
                        if lift_permitted(i) {
                            L::Lifting
                        } else {
                            L::Landed
                        }
                    }
                    RestPolicy::Land | RestPolicy::LandLock => L::Landed,
                }
            }
        }
        L::Seated | L::Landed => {
            if i.selftest_request {
                L::SelfTest
            } else if i.manual_request {
                L::Manual
            } else if s == L::Landed && rest == RestPolicy::Seat && i.seat == SeatStatus::Seated {
                L::Seated
            } else if s == L::Seated && i.seat == SeatStatus::Failed {
                L::Landed
            } else if (i.lift_request || i.move_request) && lift_permitted(i) {
                L::Lifting
            } else {
                s
            }
        }
        L::Lifting => {
            if i.seis != SeismicState::Normal || i.land_request || i.manual_request || i.timeout {
                L::Landing
            } else if i.phase == LevPhase::Levitated
                && (rest != RestPolicy::Seat
                    || !matches!(i.seat, SeatStatus::Seated | SeatStatus::Unseating))
            {
                if i.move_request {
                    L::Moving
                } else {
                    L::LevitatedIdle
                }
            } else {
                L::Lifting
            }
        }
        L::LevitatedIdle => {
            if seismic(i) {
                L::Seismic
            } else if i.escape {
                L::Escape
            } else if i.land_request || i.manual_request {
                seat_or_land(i, rest)
            } else if i.move_request {
                L::Moving
            } else {
                L::LevitatedIdle
            }
        }
        L::Moving => {
            if seismic(i) {
                L::Seismic
            } else if i.escape {
                L::Escape
            } else if i.stop_request || i.land_request || i.manual_request {
                L::Stopping
            } else if i.arrived {
                match rest {
                    RestPolicy::Seat if i.at_closed => L::Seating,
                    RestPolicy::Land | RestPolicy::LandLock => L::Landing,
                    _ => L::LevitatedIdle,
                }
            } else if i.timeout {
                L::Stopping
            } else {
                L::Moving
            }
        }
        L::Stopping => {
            if seismic(i) {
                L::Seismic
            } else if i.escape {
                L::Escape
            } else if !i.stopped && !i.timeout {
                L::Stopping
            } else if i.land_request || i.manual_request {
                seat_or_land(i, rest)
            } else if i.move_request {
                L::Moving
            } else if rest == RestPolicy::Seat && i.at_closed {
                L::Seating
            } else {
                L::LevitatedIdle
            }
        }
        L::Landing => {
            if down(i) {
                if i.manual_request {
                    L::Manual
                } else {
                    after_touchdown(i, rest)
                }
            } else {
                L::Landing
            }
        }
        L::Seating => {
            if i.seat == SeatStatus::Seated {
                if i.manual_request {
                    L::Manual
                } else {
                    L::Seated
                }
            } else if i.seat == SeatStatus::Failed {
                L::FaultReaction
            } else {
                L::Seating
            }
        }
        L::FaultReaction => {
            if down(i) || i.timeout || seated(i, rest) {
                L::Fault
            } else {
                L::FaultReaction
            }
        }
        L::Fault => {
            if i.cmd == Some(UserCommand::Reset) && i.demand < SafetyDemand::SafeStop && i.crc_ok {
                L::SelfTest
            } else if i.manual_request {
                L::Manual
            } else {
                L::Fault
            }
        }
        L::Seismic => {
            if i.escape {
                L::Escape
            } else if down(i) || seated(i, rest) {
                after_touchdown(i, rest)
            } else if i.seis == SeismicState::Normal {
                if i.stopped {
                    L::LevitatedIdle
                } else {
                    L::Stopping
                }
            } else {
                L::Seismic
            }
        }
        L::Escape => {
            if down(i) {
                L::Landed
            } else {
                L::Escape
            }
        }
        L::Manual => {
            if i.cmd == Some(UserCommand::Reset) || i.selftest_request {
                L::SelfTest
            } else {
                L::Manual
            }
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::msg::{FastStatus, SeismicReport, SupInputs};
    use crate::supervisor::Supervisor;
    use fabrail_params::{SystemParams, presets};
    use fabrail_types::{
        FaultCode, LevState as L, MotionMode, PowerRaw, RailClass, SafetyVerdict, Tick,
    };

    /// Inputs of a healthy, idle, fresh release at rest.
    fn ok() -> LevInputs {
        LevInputs {
            phase: LevPhase::Off,
            selftest_ok: true,
            crc_ok: true,
            referenced: true,
            holdup_ok: true,
            fast_valid: true,
            fresh: true,
            stopped: true,
            ..LevInputs::default()
        }
    }

    #[test]
    fn rest_states_by_policy() {
        assert_eq!(rest_state(RestPolicy::Seat), LevState::Seated);
        assert_eq!(rest_state(RestPolicy::Hover), LevState::LevitatedIdle);
        assert_eq!(rest_state(RestPolicy::LandLock), LevState::Landed);
        assert!(!is_levitated(LevState::Seated));
        assert!(is_levitated(LevState::Moving));
    }

    /// The LevState table row by row (module docs; PHYSICS §18). Tolerance (E): exact
    /// state equality, source core design §B.4 and plan §2b/§4b.
    #[test]
    fn transition_table() {
        use RestPolicy as R;
        let lev = |i: LevInputs| LevInputs {
            phase: LevPhase::Levitated,
            ..i
        };
        #[rustfmt::skip]
        let rows: &[(L, LevInputs, R, L, &str)] = &[
            (L::Boot, ok(), R::Hover, L::SelfTest, "BOOT → SELFTEST"),
            (L::SelfTest, LevInputs { fast_valid: false, ..ok() }, R::Hover, L::SelfTest, "wait for the fast group"),
            (L::SelfTest, LevInputs { crc_ok: false, ..ok() }, R::Hover, L::FaultReaction, "CRC mismatch refuses lift"),
            (L::SelfTest, LevInputs { selftest_ok: false, ..ok() }, R::Land, L::FaultReaction, "golden hash mismatch"),
            (L::SelfTest, LevInputs { seat: SeatStatus::Seated, ..ok() }, R::Seat, L::Seated, "DRAWER rests SEATED"),
            (L::SelfTest, LevInputs { seat: SeatStatus::Failed, ..ok() }, R::Seat, L::Landed, "off-seat drawer on rollers"),
            (L::SelfTest, ok(), R::Hover, L::Lifting, "DOOR lifts to HOVER"),
            (L::SelfTest, LevInputs { holdup_ok: false, ..ok() }, R::Hover, L::Landed, "hold-up low: stay landed (SAF-009)"),
            (L::SelfTest, ok(), R::Land, L::Landed, "WALL rests LANDED"),
            (L::SelfTest, ok(), R::LandLock, L::Landed, "HEAVY rests LANDED"),
            (L::SelfTest, lev(ok()), R::Hover, L::LevitatedIdle, "warm start"),
            (L::Seated, LevInputs { move_request: true, seat: SeatStatus::Seated, ..ok() }, R::Seat, L::Lifting, "push-to-open unseats"),
            (L::Seated, LevInputs { move_request: true, seis: SeismicState::Protect, ..ok() }, R::Seat, L::Seated, "never unseat on an alert"),
            (L::Seated, LevInputs { move_request: true, crc_ok: false, ..ok() }, R::Seat, L::Seated, "CRC refuses lift"),
            (L::Seated, LevInputs { manual_request: true, ..ok() }, R::Seat, L::Manual, "manual"),
            (L::Seated, LevInputs { selftest_request: true, ..ok() }, R::Seat, L::SelfTest, "post-INSPECT selftest"),
            (L::Seated, LevInputs { seat: SeatStatus::Failed, ..ok() }, R::Seat, L::Landed, "seat lost"),
            (L::Landed, LevInputs { lift_request: true, ..ok() }, R::Land, L::Lifting, "lift"),
            (L::Landed, LevInputs { lift_request: true, holdup_ok: false, ..ok() }, R::Land, L::Landed, "hold-up low"),
            (L::Landed, LevInputs { lift_request: true, seis: SeismicState::Inspect, ..ok() }, R::Hover, L::Landed, "INSPECT gates lift"),
            (L::Landed, LevInputs { seat: SeatStatus::Seated, ..ok() }, R::Seat, L::Seated, "pushed back into the seat (F17)"),
            (L::Lifting, lev(ok()), R::Hover, L::LevitatedIdle, "lifted"),
            (L::Lifting, lev(LevInputs { move_request: true, ..ok() }), R::Land, L::Moving, "lifted with a pending move"),
            (L::Lifting, lev(LevInputs { seat: SeatStatus::Unseating, ..ok() }), R::Seat, L::Lifting, "still in the unseat ramp"),
            (L::Lifting, lev(LevInputs { seat: SeatStatus::InSlot, move_request: true, ..ok() }), R::Seat, L::Moving, "unseated, drive out"),
            (L::Lifting, LevInputs { phase: LevPhase::Lifting, timeout: true, ..ok() }, R::Hover, L::Landing, "lift timeout aborts"),
            (L::Lifting, LevInputs { phase: LevPhase::Lifting, seis: SeismicState::Triggered, ..ok() }, R::Hover, L::Landing, "alert aborts lift"),
            (L::LevitatedIdle, lev(LevInputs { move_request: true, ..ok() }), R::Hover, L::Moving, "move"),
            (L::LevitatedIdle, lev(LevInputs { seis: SeismicState::Triggered, action: SeismicAction::Freeze, ..ok() }), R::Hover, L::Seismic, "alert"),
            (L::LevitatedIdle, lev(LevInputs { escape: true, ..ok() }), R::Hover, L::Escape, "fire alarm fail-open"),
            (L::LevitatedIdle, lev(LevInputs { land_request: true, ..ok() }), R::Hover, L::Landing, "land"),
            (L::LevitatedIdle, lev(LevInputs { land_request: true, at_closed: true, ..ok() }), R::Seat, L::Seating, "seat"),
            (L::LevitatedIdle, lev(LevInputs { land_request: true, ..ok() }), R::Seat, L::Landing, "open drawer lands in place"),
            (L::Moving, lev(LevInputs { stopped: false, ..ok() }), R::Hover, L::Moving, "cruise"),
            (L::Moving, lev(LevInputs { arrived: true, ..ok() }), R::Hover, L::LevitatedIdle, "DOOR hovers at the target"),
            (L::Moving, lev(LevInputs { arrived: true, ..ok() }), R::Land, L::Landing, "WALL lands at park"),
            (L::Moving, lev(LevInputs { arrived: true, at_closed: true, ..ok() }), R::Seat, L::Seating, "DRAWER seats"),
            (L::Moving, lev(LevInputs { arrived: true, ..ok() }), R::Seat, L::LevitatedIdle, "DRAWER hovers open"),
            (L::Moving, lev(LevInputs { stop_request: true, stopped: false, ..ok() }), R::Hover, L::Stopping, "stop"),
            (L::Moving, lev(LevInputs { timeout: true, stopped: false, ..ok() }), R::Hover, L::Stopping, "overdue"),
            (L::Moving, lev(LevInputs { seis: SeismicState::Protect, action: SeismicAction::Land, ..ok() }), R::Hover, L::Seismic, "alert while moving"),
            (L::Stopping, lev(LevInputs { stopped: false, ..ok() }), R::Hover, L::Stopping, "braking"),
            (L::Stopping, lev(ok()), R::Hover, L::LevitatedIdle, "stopped"),
            (L::Stopping, lev(LevInputs { move_request: true, ..ok() }), R::Hover, L::Moving, "re-plan"),
            (L::Stopping, lev(LevInputs { land_request: true, ..ok() }), R::Land, L::Landing, "stop then land"),
            (L::Stopping, lev(LevInputs { at_closed: true, ..ok() }), R::Seat, L::Seating, "stopped at the seat"),
            (L::Landing, lev(ok()), R::Land, L::Landing, "descending"),
            (L::Landing, LevInputs { phase: LevPhase::Off, fresh: false, ..ok() }, R::Land, L::Landing, "stale status never completes"),
            (L::Landing, LevInputs { phase: LevPhase::Touchdown, ..ok() }, R::Land, L::Landed, "touchdown"),
            (L::Landing, LevInputs { phase: LevPhase::Touchdown, manual_request: true, ..ok() }, R::Hover, L::Manual, "then MANUAL"),
            (L::Landing, LevInputs { seat: SeatStatus::Seated, ..ok() }, R::Seat, L::Seated, "seated"),
            (L::Seating, lev(LevInputs { seat: SeatStatus::Seating, ..ok() }), R::Seat, L::Seating, "z-seat step"),
            (L::Seating, LevInputs { seat: SeatStatus::Seated, ..ok() }, R::Seat, L::Seated, "seated"),
            (L::Seating, LevInputs { seat: SeatStatus::Failed, ..ok() }, R::Seat, L::FaultReaction, "SeatFailed"),
            (L::FaultReaction, lev(ok()), R::Hover, L::FaultReaction, "SS1 in progress"),
            (L::FaultReaction, ok(), R::Hover, L::Fault, "touchdown latches FAULT"),
            (L::FaultReaction, lev(LevInputs { timeout: true, ..ok() }), R::Hover, L::Fault, "reaction timeout"),
            (L::FaultReaction, lev(LevInputs { seat: SeatStatus::Seated, ..ok() }), R::Seat, L::Fault, "seat-first SS1 seated"),
            (L::FaultReaction, lev(LevInputs { seat: SeatStatus::Seated, fresh: false, ..ok() }), R::Seat, L::FaultReaction, "stale seat status"),
            (L::Fault, ok(), R::Hover, L::Fault, "latched"),
            (L::Fault, LevInputs { cmd: Some(UserCommand::Lift), lift_request: true, ..ok() }, R::Hover, L::Fault, "only Reset clears"),
            (L::Fault, LevInputs { cmd: Some(UserCommand::Reset), ..ok() }, R::Hover, L::SelfTest, "Reset"),
            (L::Fault, LevInputs { cmd: Some(UserCommand::Reset), crc_ok: false, ..ok() }, R::Hover, L::Fault, "Reset cannot clear a CRC mismatch"),
            (L::Fault, LevInputs { manual_request: true, ..ok() }, R::Hover, L::Manual, "manual from FAULT"),
            (L::Seismic, lev(LevInputs { seis: SeismicState::Shaking, action: SeismicAction::Land, ..ok() }), R::Hover, L::Seismic, "action in progress"),
            (L::Seismic, LevInputs { phase: LevPhase::Touchdown, seis: SeismicState::Shaking, ..ok() }, R::Hover, L::Landed, "landed"),
            (L::Seismic, LevInputs { seat: SeatStatus::Seated, seis: SeismicState::Shaking, ..ok() }, R::Seat, L::Seated, "seated"),
            (L::Seismic, lev(ok()), R::Hover, L::LevitatedIdle, "cancel returns to idle"),
            (L::Seismic, lev(LevInputs { stopped: false, ..ok() }), R::Hover, L::Stopping, "cancel while moving"),
            (L::Seismic, lev(LevInputs { escape: true, seis: SeismicState::Protect, ..ok() }), R::Hover, L::Escape, "OpenAndStop"),
            (L::Escape, lev(ok()), R::Hover, L::Escape, "opening"),
            (L::Escape, LevInputs { phase: LevPhase::Touchdown, ..ok() }, R::Hover, L::Landed, "landed, manually operable"),
            (L::Manual, ok(), R::Hover, L::Manual, "manual"),
            (L::Manual, LevInputs { cmd: Some(UserCommand::Reset), ..ok() }, R::Hover, L::SelfTest, "Reset"),
        ];
        for (from, i, rest, to, why) in rows {
            assert_eq!(
                transition(*from, i, *rest),
                *to,
                "(E) {from:?} with {why} (core design §B.4 table)"
            );
        }

        // Any state but the fault states reacts to a demand ≥ SafeStop (SAF-013).
        for &s in LevState::ALL {
            let i = LevInputs {
                demand: SafetyDemand::SafeStop,
                ..ok()
            };
            let t = transition(s, &i, RestPolicy::Hover);
            if matches!(s, L::FaultReaction) {
                assert_eq!(t, L::Fault, "touchdown completes the reaction");
            } else if s == L::Fault {
                assert_eq!(t, L::Fault);
            } else {
                assert_eq!(t, L::FaultReaction, "{s:?} must react (SAF-013)");
            }
        }
        // No state ever enters LIFTING, or leaves a rest state upward, without CRC.
        for &s in LevState::ALL {
            for rest in RestPolicy::ALL {
                let i = LevInputs {
                    crc_ok: false,
                    lift_request: true,
                    move_request: true,
                    ..ok()
                };
                let t = transition(s, &i, *rest);
                assert!(
                    !(t == L::Lifting && s != L::Lifting),
                    "{s:?}/{rest:?} → LIFTING with a bad CRC (SAF-015)"
                );
            }
        }
    }

    // ---- End-to-end tests through `Supervisor::step` -------------------------------

    /// A minimal stand-in for the fast group's levitation phases (the real one is A1's
    /// `Levitation`; the supervisor only reads the phase, seat status and heartbeat).
    pub(crate) struct FakeFast {
        pub s: FastStatus,
        pub rest: RestPolicy,
        pub timer: u32,
    }

    impl FakeFast {
        pub(crate) fn new(rest: RestPolicy) -> FakeFast {
            let seat = if rest == RestPolicy::Seat {
                SeatStatus::Seated
            } else {
                SeatStatus::NoSeat
            };
            FakeFast {
                s: FastStatus {
                    referenced: true,
                    seat,
                    ..FastStatus::default()
                },
                rest,
                timer: 0,
            }
        }

        /// Applies directive `d` for one supervisor period: lifts/lands in 3 periods.
        pub(crate) fn run(&mut self, d: &crate::msg::Directive) {
            use crate::msg::LevDirective as D;
            self.s.seq = self.s.seq.wrapping_add(10);
            self.s.directive_seq = d.seq;
            self.s.mode = d.mode;
            self.timer += 1;
            match d.lev {
                D::Lift | D::Unseat => {
                    if self.s.lev != LevPhase::Levitated {
                        if self.s.lev != LevPhase::Lifting {
                            self.timer = 0;
                        }
                        self.s.lev = LevPhase::Lifting;
                        if d.lev == D::Unseat {
                            self.s.seat = SeatStatus::Unseating;
                        }
                        if self.timer >= 3 {
                            self.s.lev = LevPhase::Levitated;
                            if self.rest == RestPolicy::Seat {
                                self.s.seat = SeatStatus::InSlot;
                            }
                        }
                    }
                }
                D::Levitate | D::Hold => {}
                D::Land { .. } => {
                    if matches!(self.s.lev, LevPhase::Levitated | LevPhase::Lifting) {
                        self.s.lev = LevPhase::Landing;
                        self.timer = 0;
                    } else if self.s.lev == LevPhase::Landing && self.timer >= 3 {
                        self.s.lev = LevPhase::Touchdown;
                        if self.rest == RestPolicy::Seat {
                            self.s.seat = SeatStatus::Failed;
                        }
                    }
                }
                D::Seat => {
                    if self.s.seat != SeatStatus::Seating && self.s.seat != SeatStatus::Seated {
                        self.s.seat = SeatStatus::Seating;
                        self.timer = 0;
                    } else if self.s.seat == SeatStatus::Seating && self.timer >= 3 {
                        self.s.seat = SeatStatus::Seated;
                        self.s.lev = LevPhase::Off;
                    }
                }
                D::Off => {
                    if self.s.lev == LevPhase::Touchdown {
                        self.s.lev = LevPhase::Off;
                    }
                }
            }
        }
    }

    /// Nominal supply inputs for `p` (mains, charged supercap, 20 °C).
    pub(crate) fn nominal_inputs(p: &SystemParams, sup: &Supervisor, t: Tick) -> SupInputs {
        let (vb, vh) = sup.supply.codes_for(
            p.design.power.v_bus_v as f32,
            p.design.power.supercap_v_max_v as f32,
        );
        let tc =
            ((20.0 - p.design.sensors.temp_offset_c) / p.design.sensors.temp_c_per_code) as u16;
        SupInputs {
            tick: t,
            power: PowerRaw {
                v_bus_code: vb,
                v_hold_code: vh,
                i_bus_code: 0,
                mains_ok: true,
                temp_code: [tc, tc],
            },
            verdict: SafetyVerdict {
                force_cap_n: 1.0e3,
                ..SafetyVerdict::default()
            },
            ..SupInputs::default()
        }
    }

    /// Runs `n` supervisor releases, applying `edit` to each release's inputs.
    pub(crate) fn run(
        sup: &mut Supervisor,
        ff: &mut FakeFast,
        p: &SystemParams,
        t: &mut Tick,
        n: u32,
        mut edit: impl FnMut(u32, &mut SupInputs, &mut FastStatus),
    ) -> crate::msg::Directive {
        let mut d = crate::msg::Directive::default();
        for k in 0..n {
            let mut i = nominal_inputs(p, sup, *t);
            edit(k, &mut i, &mut ff.s);
            let (dd, _) = sup.step(&ff.s, &i, &SeismicReport::default());
            ff.run(&dd);
            d = dd;
            *t = t.plus(20);
        }
        d
    }

    /// A deadline miss (FastOverrun), a stale heartbeat (monitor HeartbeatLost/STO) and a
    /// non-finite controller value (NonFinite) each lead to FAULT_REACTION (SS1 Safe
    /// Landing: levitation LAND, never coils Off while levitated), then a latched FAULT
    /// that stays latched after the source clears and ignores every command but Reset;
    /// Reset returns through SELFTEST to the rest state. Tolerance (E): exact states,
    /// source IEC 61800-5-2 (SS1) and core design §B.4.
    // req: FAB-SAF-013
    #[test]
    fn fault_latch() {
        use crate::msg::LevDirective;
        let p = presets::door();
        for (name, code, via_verdict) in [
            ("deadline miss", FaultCode::FastOverrun, false),
            ("stale heartbeat", FaultCode::HeartbeatLost, true),
            ("non-finite value", FaultCode::NonFinite, false),
        ] {
            let Ok(mut sup) = Supervisor::new(&p) else {
                panic!("supervisor");
            };
            let mut ff = FakeFast::new(p.design.rest_policy);
            let mut t = Tick(0);
            run(&mut sup, &mut ff, &p, &mut t, 12, |_, _, _| {});
            assert_eq!(sup.lev, L::LevitatedIdle, "{name}: DOOR hovers after boot");

            // Inject the fault for one release only.
            let mut saw_reaction = false;
            for k in 0..12u32 {
                let mut i = nominal_inputs(&p, &sup, t);
                let mut s = ff.s;
                if k == 0 {
                    if via_verdict {
                        i.verdict.faults.set(code);
                        i.verdict.demand = code.demand();
                    } else {
                        s.faults.set(code);
                    }
                }
                let (d, st) = sup.step(&s, &i, &SeismicReport::default());
                if sup.lev == L::FaultReaction {
                    saw_reaction = true;
                    assert!(
                        matches!(d.lev, LevDirective::Land { .. }),
                        "{name}: SS1 lands under power, got {:?}",
                        d.lev
                    );
                    assert!(st.demand >= fabrail_types::SafetyDemand::SafeStop);
                }
                ff.run(&d);
                t = t.plus(20);
            }
            assert!(saw_reaction, "{name}: FAULT_REACTION entered");
            assert_eq!(sup.lev, L::Fault, "{name}: latched FAULT after touchdown");
            assert!(sup.faults.latched.contains(code), "{name}: code latched");

            // Other commands never clear the latch.
            for cmd in [
                UserCommand::Lift,
                UserCommand::Open,
                UserCommand::Close,
                UserCommand::Stop,
                UserCommand::Land,
            ] {
                run(&mut sup, &mut ff, &p, &mut t, 1, |_, i, _| {
                    i.cmd = Some(cmd)
                });
                assert_eq!(sup.lev, L::Fault, "{name}: {cmd:?} must not clear FAULT");
            }
            let d = run(&mut sup, &mut ff, &p, &mut t, 1, |_, _, _| {});
            assert_eq!(d.lev, LevDirective::Off);
            assert_eq!(d.mode, MotionMode::Off, "{name}: propulsion STO in FAULT");

            // Reset → SELFTEST → the rest state again.
            run(&mut sup, &mut ff, &p, &mut t, 1, |_, i, _| {
                i.cmd = Some(UserCommand::Reset)
            });
            assert_eq!(sup.lev, L::SelfTest, "{name}: Reset clears the latch");
            assert!(!sup.faults.latched.contains(code));
            run(&mut sup, &mut ff, &p, &mut t, 10, |_, _, _| {});
            assert_eq!(sup.lev, L::LevitatedIdle, "{name}: back to HOVER");
        }
    }

    /// A safety block whose CRC or diverse copy does not match refuses lift: the
    /// supervisor latches ParamCrc, never commands Lift/Unseat/Levitate, rejects Lift and
    /// Open, and a Reset cannot clear it; the sealed preset lifts normally. Tolerance
    /// (E): exact, source IEC 60730-1 Class B intent (SAF-015).
    // req: FAB-SAF-015
    #[test]
    fn refuse_lift_on_crc() {
        use crate::msg::LevDirective;
        for class in [RailClass::Drawer, RailClass::Door, RailClass::Wall] {
            let good = presets::by_class(class);
            let mut corrupt = [good; 2];
            corrupt[0].safety.e_max_j = 2.0; // edited without resealing
            corrupt[1].safety.nm_per_count_copy += 1; // diverse copy mismatch
            for bad in corrupt {
                let Ok(mut sup) = Supervisor::new(&bad) else {
                    panic!("supervisor");
                };
                assert!(!sup.crc_ok, "{class:?}: CRC check detects the edit");
                let mut ff = FakeFast::new(bad.design.rest_policy);
                let mut t = Tick(0);
                for k in 0..40u32 {
                    let mut i = nominal_inputs(&bad, &sup, t);
                    i.cmd = match k % 8 {
                        2 => Some(UserCommand::Lift),
                        4 => Some(UserCommand::Open),
                        6 => Some(UserCommand::Reset),
                        _ => None,
                    };
                    let (d, st) = sup.step(&ff.s, &i, &SeismicReport::default());
                    assert!(
                        matches!(d.lev, LevDirective::Off),
                        "{class:?} release {k}: lev {:?} with a bad CRC (SAF-015)",
                        d.lev
                    );
                    assert!(
                        !is_levitated(sup.lev) || sup.lev == L::FaultReaction,
                        "{class:?}: {:?}",
                        sup.lev
                    );
                    if k > 1 {
                        assert!(st.faults.contains(FaultCode::ParamCrc));
                    }
                    ff.run(&d);
                    t = t.plus(20);
                }
                assert_eq!(sup.lev, L::Fault, "{class:?}: latched FAULT");
            }
            // Control: the sealed preset reaches a lifted state on Lift/Open.
            let Ok(mut sup) = Supervisor::new(&good) else {
                panic!("supervisor");
            };
            assert!(sup.crc_ok);
            let mut ff = FakeFast::new(good.design.rest_policy);
            let mut t = Tick(0);
            let mut lifted = false;
            for k in 0..30u32 {
                let mut i = nominal_inputs(&good, &sup, t);
                if k == 5 {
                    i.cmd = Some(UserCommand::Lift);
                }
                let (d, _) = sup.step(&ff.s, &i, &SeismicReport::default());
                lifted |= matches!(d.lev, LevDirective::Lift | LevDirective::Unseat);
                ff.run(&d);
                t = t.plus(20);
            }
            assert!(lifted, "{class:?}: a sealed block lifts");
        }
    }
}
