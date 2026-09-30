//! Land-first class actions and egress rules (PHYSICS §21; SEI-010, SEI-011). Owner: A3.
//!
//! Frozen API (W0): [`PolicyInputs`], [`class_action`] → `ActionDecision` (the core
//! default that `seismic::action_policy` hooks may override toward safety only).
//! Class table (core design §C.2): DRAWER FREEZE → stay seated or land; DOOR FREEZE →
//! stop → land on rollers, release ASC, never lock (fire-rated never opens; non-rated
//! may open only if stopped by t_S − 1 s); WALL/HEAVY FREEZE → centre only if reachable
//! (F11: |x − x_c| ≤ v·(t_budget − t_land)) → land + lock. Level 1 means freeze only.
//! Tests: `egress_door_policy`, `class_policy_table`.
//!
//! **By state** (the seismic action is declared *not* a rated safety function, SF-08):
//!
//! | state | action |
//! |---|---|
//! | NORMAL | None |
//! | TRIGGERED (level ≤ 1, or before PROTECT) | Freeze (decelerate at a_brk and hold) |
//! | PROTECT | the class action: Center if configured and reachable (F11), an egress OpenAndStop if configured and feasible, else the landing action |
//! | SHAKING | the landing action (centring and opening are abandoned; research-tier Isolate/Damp continue if configured) |
//! | QUIET, INSPECT | the landing action (stay landed/locked until the manual reset) |
//!
//! The *landing action* is `Lock` when the tuning asks for it (`action = Lock` or
//! `lock_after_land`) **and** the opening is not on an egress path, else `Land`; a tuning
//! of `None`/`Freeze` gives Freeze. The core never selects the sovereign actions
//! (RideThrough, CompleteClose, CloseAfterShaking): configured in a core tuning they fall
//! back to the landing action; the exotic tier reaches them only through the
//! [`ActionPolicy`] hook ([`decide`]).
//!
//! **Egress** (IBC 1010.3; NFPA 80; SEI-011): never Lock on an egress path, whatever the
//! tuning; a fire-rated door never opens; a non-rated DOOR configured for `OpenAndStop`
//! opens only when (i) no non-local source contributed (`source == Local`), (ii) the
//! open target and an S-ETA are known and (iii) the move ends at rest by
//! t_S − `egress_stop_margin_s`, with t_move = d/v + v/a (trapezoid; 2√(d/a) when the
//! cruise speed is not reached; d/v when no acceleration is given) plus v₀/a to stop a
//! closing motion first. Afterwards it lands and stays manually operable (ASC released by
//! the supervisor, F9).

use fabrail_params::PolicyTuning;
use fabrail_types::{
    AlertSource, EgressRole, PosNm, RailClass, RestPolicy, SeismicAction, SeismicLevel,
    SeismicState, Tick, fmath,
};

use crate::seismic::action_policy::{
    ActionContext, ActionDecision, ActionPolicy, PolicyOutcome, resolve,
};
use crate::seismic::fusion::tick_offset;

/// What the class policy looks at.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PolicyInputs {
    /// Tick of the decision.
    pub tick: Tick,
    /// Class.
    pub class: RailClass,
    /// Rest policy.
    pub rest_policy: RestPolicy,
    /// Egress role.
    pub egress: EgressRole,
    /// Seismic state.
    pub seis: SeismicState,
    /// Detector level.
    pub level: SeismicLevel,
    /// Position.
    pub x: PosNm,
    /// Velocity, m/s.
    pub v_mps: f32,
    /// Centring target (WALL/HEAVY), if configured.
    pub x_center: Option<PosNm>,
    /// Predicted S arrival.
    pub s_eta: Option<Tick>,
    /// Time needed to land from levitation, s.
    pub t_land_s: f32,
    /// Speed available for centring, m/s.
    pub v_center_mps: f32,
    /// Decision source (`SeismicReport::source`): OpenAndStop only when `Local`.
    pub source: AlertSource,
    /// Egress open-and-stop target (non-rated DOOR configured for `OpenAndStop`), if any.
    pub x_open: Option<PosNm>,
    /// Speed for the open-and-stop move, m/s.
    pub v_open_mps: f32,
    /// Acceleration/braking used for the open-and-stop time estimate, m/s² (0: none).
    pub a_mps2: f32,
}

/// Seconds from `now` until `t` (signed).
#[inline]
fn secs_until(now: Tick, t: Tick) -> f32 {
    let d = t.0 as i64 - now.0 as i64;
    (d as f64 / fabrail_types::BASE_TICK_HZ as f64) as f32
}

/// The landing action for this opening: Lock only off egress paths (see module docs).
pub fn landing_action(i: &PolicyInputs, t: &PolicyTuning) -> SeismicAction {
    match t.action {
        SeismicAction::None | SeismicAction::Freeze => SeismicAction::Freeze,
        a => {
            let wants_lock = t.lock_after_land || a == SeismicAction::Lock;
            if wants_lock && i.egress == EgressRole::None {
                SeismicAction::Lock
            } else {
                SeismicAction::Land
            }
        }
    }
}

/// F11 centring: reachable iff |x − x_c| ≤ v·(t_budget − t_land) with t_budget the time
/// to the S-ETA; returns the centring decision (deadline t_S − t_land).
fn center_if_reachable(i: &PolicyInputs) -> Option<ActionDecision> {
    let (xc, eta) = (i.x_center?, i.s_eta?);
    let v = i.v_center_mps;
    let slack = secs_until(i.tick, eta) - fmath::fmaxf(i.t_land_s, 0.0);
    if !(v > 0.0 && slack > 0.0) {
        return None;
    }
    let d = fmath::absf(xc.diff_m(i.x));
    (d <= v * slack).then_some(ActionDecision {
        action: SeismicAction::Center,
        target: Some(xc),
        v_cap_mps: v,
        deadline: Some(tick_offset(eta, -fmath::fmaxf(i.t_land_s, 0.0))),
    })
}

/// Time to move `d_m` from rest and stop at cruise `v` with acceleration `a` (trapezoid
/// profile; triangular when v is not reached; constant speed when `a` ≤ 0), s.
pub fn move_time_s(d_m: f32, v: f32, a: f32) -> f32 {
    let d = fmath::absf(d_m);
    if v.is_nan() || v <= 0.0 {
        return f32::INFINITY;
    }
    if a.is_nan() || a <= 0.0 {
        return d / v;
    }
    if d >= v * v / a {
        d / v + v / a
    } else {
        2.0 * fmath::sqrtf(d / a)
    }
}

/// Egress open-and-stop (SEI-011): only a non-rated DOOR, only from a purely local
/// decision, and only if it is at rest at the target by t_S − margin.
fn open_and_stop_if_feasible(i: &PolicyInputs, t: &PolicyTuning) -> Option<ActionDecision> {
    if !(i.class == RailClass::Door
        && i.egress == EgressRole::NonRated
        && i.source == AlertSource::Local)
    {
        return None;
    }
    let (xo, eta) = (i.x_open?, i.s_eta?);
    let d = xo.diff_m(i.x);
    if d < 0.0 {
        return None; // the target lies on the closing side: not an opening
    }
    // A closing motion must first be stopped.
    let t_stop = if i.v_mps < 0.0 && i.a_mps2 > 0.0 {
        -i.v_mps / i.a_mps2
    } else {
        0.0
    };
    let margin = fmath::fmaxf(t.egress_stop_margin_s as f32, 0.0);
    let t_move = t_stop + move_time_s(d, i.v_open_mps, i.a_mps2);
    (t_move <= secs_until(i.tick, eta) - margin).then_some(ActionDecision {
        action: SeismicAction::OpenAndStop,
        target: Some(xo),
        v_cap_mps: i.v_open_mps,
        deadline: Some(tick_offset(eta, -margin)),
    })
}

/// The core land-first class action (see the module table).
pub fn class_action(i: &PolicyInputs, t: &PolicyTuning) -> ActionDecision {
    let landing = landing_action(i, t);
    let research = matches!(t.action, SeismicAction::Isolate | SeismicAction::Damp);
    match i.seis {
        SeismicState::Normal => ActionDecision::of(SeismicAction::None),
        SeismicState::Triggered => ActionDecision::of(SeismicAction::Freeze),
        SeismicState::Protect => {
            if t.action == SeismicAction::OpenAndStop
                && i.egress != EgressRole::FireRated
                && let Some(d) = open_and_stop_if_feasible(i, t)
            {
                return d;
            }
            if t.center_if_reachable
                && let Some(d) = center_if_reachable(i)
            {
                return d;
            }
            if research {
                ActionDecision::of(t.action)
            } else {
                ActionDecision::of(landing)
            }
        }
        SeismicState::Shaking if research => ActionDecision::of(t.action),
        SeismicState::Shaking | SeismicState::Quiet | SeismicState::Inspect => {
            ActionDecision::of(landing)
        }
    }
}

/// The class action filtered through an [`ActionPolicy`] hook: outside NORMAL the
/// policy's proposal passes `action_policy::admit` (toward safety only); in NORMAL the
/// core action (None) stands. `ctx` supplies the device and report fields; its
/// `default` and `seis` are replaced by this decision's.
pub fn decide(
    i: &PolicyInputs,
    t: &PolicyTuning,
    ctx: &ActionContext,
    policy: &mut dyn ActionPolicy,
) -> (ActionDecision, PolicyOutcome) {
    let default = class_action(i, t);
    if i.seis == SeismicState::Normal {
        return (default, PolicyOutcome::Default);
    }
    let c = ActionContext {
        default,
        seis: i.seis,
        ..*ctx
    };
    resolve(&c, policy)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::SeismicReport;
    use crate::seismic::action_policy::{CorePolicy, RejectReason};
    use fabrail_params::{SystemParams, presets};

    fn inputs(p: &SystemParams, seis: SeismicState) -> PolicyInputs {
        PolicyInputs {
            tick: Tick::from_seconds_f64(20.0),
            class: p.design.class,
            rest_policy: p.design.rest_policy,
            egress: p.design.egress,
            seis,
            level: SeismicLevel::L3,
            x: PosNm::from_m_f64(0.4),
            ..PolicyInputs::default()
        }
    }

    // req: FAB-SEI-010
    #[test]
    fn class_policy_table() {
        use SeismicAction as A;
        use SeismicState as S;
        // Expected land-first actions (core design §C.2): NORMAL, TRIGGERED, PROTECT,
        // SHAKING, QUIET, INSPECT.
        let table = [
            (
                RailClass::Drawer,
                [A::None, A::Freeze, A::Land, A::Land, A::Land, A::Land],
            ),
            (
                RailClass::Door,
                [A::None, A::Freeze, A::Land, A::Land, A::Land, A::Land],
            ),
            (
                RailClass::Wall,
                [A::None, A::Freeze, A::Lock, A::Lock, A::Lock, A::Lock],
            ),
            (
                RailClass::Heavy,
                [A::None, A::Freeze, A::Lock, A::Lock, A::Lock, A::Lock],
            ),
        ];
        for (class, want) in table {
            let p = presets::by_class(class);
            let t = p.control.seismic.policy;
            for (&s, &w) in S::ALL.iter().zip(want.iter()) {
                for level in SeismicLevel::ALL {
                    let mut i = inputs(&p, s);
                    i.level = *level;
                    let d = class_action(&i, &t);
                    assert_eq!(d.action, w, "{class:?} in {s:?} at {level:?}");
                    assert!(
                        !d.moves(),
                        "{class:?} {s:?}: land-first, no motion without a centring target"
                    );
                }
            }
        }
        // Level 1 is freeze only: TRIGGERED never lands, whatever the class.
        for &class in presets::CLASSES.iter() {
            let p = presets::by_class(class);
            let mut i = inputs(&p, S::Triggered);
            i.level = SeismicLevel::L1;
            assert_eq!(
                class_action(&i, &p.control.seismic.policy).action,
                A::Freeze
            );
        }

        // F11 centring (WALL/HEAVY): only if |x − x_c| ≤ v·(t_budget − t_land).
        for class in [RailClass::Wall, RailClass::Heavy] {
            let p = presets::by_class(class);
            let t = p.control.seismic.policy;
            assert!(
                t.center_if_reachable,
                "{class:?} centres if reachable (F11)"
            );
            let mut i = inputs(&p, S::Protect);
            i.x_center = Some(PosNm::from_m_f64(0.5)); // 0.1 m away
            i.v_center_mps = 0.04;
            i.t_land_s = 2.0;
            i.s_eta = Some(i.tick.plus(Tick::from_seconds_f64(5.0).0)); // budget 5 s − 2 s = 3 s → 0.12 m
            let d = class_action(&i, &t);
            assert_eq!(d.action, A::Center, "{class:?}: reachable (0.1 m ≤ 0.12 m)");
            assert_eq!(d.target, i.x_center);
            assert_eq!(
                d.deadline,
                Some(i.tick.plus(Tick::from_seconds_f64(3.0).0)),
                "t_S − t_land"
            );
            i.s_eta = Some(i.tick.plus(Tick::from_seconds_f64(4.0).0)); // 0.08 m < 0.1 m
            assert_eq!(
                class_action(&i, &t).action,
                A::Lock,
                "{class:?}: unreachable → land + lock"
            );
            i.s_eta = None;
            assert_eq!(
                class_action(&i, &t).action,
                A::Lock,
                "no S-ETA → no centring"
            );
            i.s_eta = Some(i.tick.plus(Tick::from_seconds_f64(5.0).0));
            i.seis = S::Shaking;
            assert_eq!(
                class_action(&i, &t).action,
                A::Lock,
                "centring abandoned in SHAKING"
            );
        }
        // DRAWER/DOOR never centre.
        for class in [RailClass::Drawer, RailClass::Door] {
            let p = presets::by_class(class);
            let mut i = inputs(&p, S::Protect);
            i.x_center = Some(PosNm::from_m_f64(0.41));
            i.v_center_mps = 1.0;
            i.s_eta = Some(i.tick.plus(200_000));
            assert_eq!(class_action(&i, &p.control.seismic.policy).action, A::Land);
        }
    }

    // req: FAB-SEI-011
    #[test]
    fn egress_door_policy() {
        use SeismicAction as A;
        use SeismicState as S;
        let p = presets::door();
        assert_eq!(
            p.design.egress,
            EgressRole::NonRated,
            "the DOOR preset is on an egress path"
        );
        let t = p.control.seismic.policy;
        // Preset: land on rollers in every protective state, never lock.
        for s in [S::Protect, S::Shaking, S::Quiet, S::Inspect] {
            assert_eq!(class_action(&inputs(&p, s), &t).action, A::Land, "{s:?}");
        }
        // Even a (mis)configured lock never locks an egress door.
        let mut lock = t;
        lock.action = A::Lock;
        lock.lock_after_land = true;
        for egress in [EgressRole::NonRated, EgressRole::FireRated] {
            let mut i = inputs(&p, S::Protect);
            i.egress = egress;
            for s in [S::Protect, S::Shaking, S::Quiet, S::Inspect] {
                i.seis = s;
                assert_ne!(
                    class_action(&i, &lock).action,
                    A::Lock,
                    "{egress:?} never locks (IBC 1010.3)"
                );
            }
        }

        // Non-rated door configured to open and stop.
        let mut open = t;
        open.action = A::OpenAndStop;
        let mut i = inputs(&p, S::Protect);
        i.x = PosNm::from_m_f64(0.2);
        i.x_open = Some(PosNm::from_m_f64(1.0));
        i.v_open_mps = 0.184;
        i.a_mps2 = 0.75;
        let t_move = move_time_s(0.8, 0.184, 0.75); // 0.8/0.184 + 0.184/0.75 = 4.593 s
        assert!((t_move - 4.593).abs() < 1e-3);
        let margin = open.egress_stop_margin_s as f32;
        assert_eq!(margin, 1.0, "stopped by t_S − 1 s (SEI-011)");
        let eta_at = |dt: f32| Some(tick_offset(i.tick, dt));
        i.s_eta = eta_at(t_move + margin + 0.01);
        let d = class_action(&i, &open);
        assert_eq!(
            d.action,
            A::OpenAndStop,
            "stops ≥ 1 s before t_S → may open"
        );
        assert_eq!(d.target, i.x_open);
        assert_eq!(
            d.deadline,
            i.s_eta.map(|e| tick_offset(e, -1.0)),
            "deadline t_S − 1 s"
        );
        i.s_eta = eta_at(t_move + margin - 0.01);
        assert_eq!(
            class_action(&i, &open).action,
            A::Land,
            "would stop < 1 s before t_S → land"
        );
        i.s_eta = None;
        assert_eq!(
            class_action(&i, &open).action,
            A::Land,
            "unknown t_S → land"
        );
        // A closing door must stop first: v₀/a added.
        i.s_eta = eta_at(t_move + margin + 0.01);
        i.v_mps = -0.15;
        assert_eq!(
            class_action(&i, &open).action,
            A::Land,
            "0.2 s to stop the closing motion first"
        );
        i.v_mps = 0.0;
        // Any non-local contribution forbids opening.
        for src in [AlertSource::Network, AlertSource::Fused] {
            i.source = src;
            assert_eq!(
                class_action(&i, &open).action,
                A::Land,
                "{src:?}: never OpenAndStop"
            );
        }
        i.source = AlertSource::Local;
        // A fire-rated door never opens, even if misconfigured.
        i.egress = EgressRole::FireRated;
        assert_eq!(
            class_action(&i, &open).action,
            A::Land,
            "fire-rated never opens (NFPA 80)"
        );
        i.egress = EgressRole::NonRated;
        // After PROTECT the door lands (ASC released by the supervisor; manually operable).
        for s in [S::Shaking, S::Quiet, S::Inspect] {
            i.seis = s;
            assert_eq!(
                class_action(&i, &open).action,
                A::Land,
                "{s:?}: land, never lock"
            );
        }
        // The hook cannot re-open it from a network alert either.
        i.seis = S::Protect;
        let ctx = ActionContext {
            egress: i.egress,
            x: i.x,
            report: SeismicReport {
                source: AlertSource::Network,
                ..SeismicReport::default()
            },
            ..ActionContext::default()
        };
        struct Open;
        impl ActionPolicy for Open {
            fn propose(&mut self, _c: &ActionContext) -> Option<ActionDecision> {
                Some(ActionDecision::of(SeismicAction::OpenAndStop))
            }
        }
        let (d, o) = decide(&i, &t, &ctx, &mut Open);
        assert_eq!(d.action, A::Land);
        assert_eq!(
            o,
            PolicyOutcome::Rejected(RejectReason::OpenAndStopNonLocal)
        );
    }

    #[test]
    fn decide_consults_the_hook_toward_safety_only() {
        let p = presets::drawer();
        let t = p.control.seismic.policy;
        let i = inputs(&p, SeismicState::Triggered);
        let ctx = ActionContext::default();
        let (d, o) = decide(&i, &t, &ctx, &mut CorePolicy);
        assert_eq!(
            (d.action, o),
            (SeismicAction::Freeze, PolicyOutcome::Default)
        );
        struct Seat;
        impl ActionPolicy for Seat {
            fn propose(&mut self, _c: &ActionContext) -> Option<ActionDecision> {
                Some(ActionDecision::of(SeismicAction::Land))
            }
        }
        let (d, o) = decide(&i, &t, &ctx, &mut Seat);
        assert_eq!(
            (d.action, o),
            (SeismicAction::Land, PolicyOutcome::Admitted),
            "freeze → land admitted"
        );
        struct Nothing;
        impl ActionPolicy for Nothing {
            fn propose(&mut self, _c: &ActionContext) -> Option<ActionDecision> {
                Some(ActionDecision::of(SeismicAction::None))
            }
        }
        let (d, _) = decide(&i, &t, &ctx, &mut Nothing);
        assert_eq!(d.action, SeismicAction::Freeze, "never less than freeze");
        let n = inputs(&p, SeismicState::Normal);
        assert_eq!(
            decide(&n, &t, &ctx, &mut Seat).1,
            PolicyOutcome::Default,
            "NORMAL: no hook"
        );
    }
}
