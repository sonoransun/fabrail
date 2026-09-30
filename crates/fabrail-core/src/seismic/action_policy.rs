//! ActionPolicy hook: an override of the class seismic action that may only move a device toward a safer state (plan §4). Owner: W0a (trait) / X3 (implementation).
//!
//! Frozen seam (W0). The supervisor computes the core land-first class action
//! (`seismic::policy::class_action`) and then asks an [`ActionPolicy`] whether to
//! override it. Exotic code (fabrail-exo EEW fusion, X3) and the house choreographer
//! (fabrail-exo `choreo`, E2) implement the trait; fabrail-core never depends on them,
//! so the context carries only core/types data. The firmware and every core preset use
//! [`CorePolicy`] (no override).
//!
//! Whatever a policy proposes passes the core-side filter [`admit`] before it takes
//! effect. The filter encodes the binding alert-motion rule (plan §4, §4b):
//!
//! * a proposal may not make things less safe: `SeismicAction::None` replacing a
//!   non-None default is rejected;
//! * `OpenAndStop` is forbidden whenever any non-local source contributed;
//! * `Lock` is forbidden on an egress path (`EgressRole` other than `None`);
//! * alert-triggered **motion** (a target, or Center / CompleteClose /
//!   CloseAfterShaking / Isolate / Damp) needs independent confirmation: a local P
//!   pick or at least `min_independent_feeds` feeds;
//! * motion may only be in the closing/seating direction (never opening), never
//!   reverses a user-initiated motion, and its speed cap is clamped to the unattended
//!   limit v ≤ min(v_cap, f_pinch/√(m_gov·k_series)) (0.23 m/s at 48 kg).
//!
//! Network-only or injected alerts can therefore cause only FREEZE, LAND, LOCK or a
//! passive ride-through (`RideThrough` without a target, SENTINEL's SEISMIC_PASSIVE),
//! and a spoofed alert has bounded regret: devices close at unattended low-energy limits.
//! The exotic tier's full per-class rank order (`fabrail_exo_safety::SafetyRank`) is
//! applied by the implementation before proposing; this filter is the core backstop.

use fabrail_types::{
    AlertSource, EgressRole, LevState, MotionMode, PosNm, RailClass, RestPolicy, SeismicAction,
    SeismicState, Tick, TravelDir, fmath,
};

use crate::msg::SeismicReport;

/// Everything a policy may look at (core and types data only).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ActionContext {
    /// Base tick of the supervisor release.
    pub tick: Tick,
    /// Product class.
    pub class: RailClass,
    /// Rest policy.
    pub rest_policy: RestPolicy,
    /// Egress role of the opening.
    pub egress: EgressRole,
    /// Seismic state (from the report).
    pub seis: SeismicState,
    /// Levitation / carriage state.
    pub lev: LevState,
    /// Motion mode currently commanded.
    pub mode: MotionMode,
    /// Measured position.
    pub x: PosNm,
    /// Measured velocity, m/s.
    pub v_mps: f32,
    /// Closed / seat position.
    pub x_closed: PosNm,
    /// Direction of a user-initiated motion in progress (never reversed), if any.
    pub user_motion: Option<TravelDir>,
    /// The seismic report this decision is based on (level, source, S-ETA, feeds).
    pub report: SeismicReport,
    /// The core land-first class action (what happens without an override).
    pub default: ActionDecision,
    /// Unattended automatic-motion speed limit, m/s (plan §4b).
    pub v_unattended_mps: f32,
    /// Independent feeds required for alert-triggered motion without a local P pick.
    pub min_independent_feeds: u8,
}

impl ActionContext {
    /// Independent confirmation: a local P pick or enough independent feeds.
    #[inline]
    pub fn confirmed(&self) -> bool {
        self.report.local_p || self.report.n_feeds >= self.min_independent_feeds
    }

    /// A non-local source (network, fused or injected) contributed to the decision.
    #[inline]
    pub fn non_local(&self) -> bool {
        self.report.source != AlertSource::Local
    }
}

/// A seismic action with its optional motion target and limits.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ActionDecision {
    /// Action.
    pub action: SeismicAction,
    /// Motion target (seat, closed stop, standoff s_min, centre), if the action moves.
    pub target: Option<PosNm>,
    /// Speed cap for the motion, m/s (clamped to the unattended limit by [`admit`]).
    pub v_cap_mps: f32,
    /// Deadline for reaching the target (S-ETA 5 % quantile − 1 s), if any.
    pub deadline: Option<Tick>,
}

impl ActionDecision {
    /// A decision with no motion target.
    #[inline]
    pub const fn of(action: SeismicAction) -> ActionDecision {
        ActionDecision {
            action,
            target: None,
            v_cap_mps: 0.0,
            deadline: None,
        }
    }

    /// True when the decision commands motion of the carriage along x.
    #[inline]
    pub fn moves(&self) -> bool {
        self.target.is_some()
            || matches!(
                self.action,
                SeismicAction::Center
                    | SeismicAction::CompleteClose
                    | SeismicAction::CloseAfterShaking
                    | SeismicAction::Isolate
                    | SeismicAction::Damp
                    | SeismicAction::OpenAndStop
            )
    }
}

/// Why a proposal was not admitted.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum RejectReason {
    /// Replacing a protective default with no action.
    NotTowardSafety = 0,
    /// `OpenAndStop` with a non-local source contributing.
    OpenAndStopNonLocal = 1,
    /// `Lock` on an egress path.
    LockOnEgress = 2,
    /// Alert-triggered motion without independent confirmation.
    Unconfirmed = 3,
    /// Motion in the opening direction.
    Opening = 4,
    /// Motion that would reverse a user-initiated motion.
    ReversesUser = 5,
    /// Motion without a target whose direction cannot be checked.
    NoTarget = 6,
}

/// Outcome of [`admit`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum PolicyOutcome {
    /// No proposal (or the proposal equals the default): the core action stands.
    #[default]
    Default,
    /// The proposal was admitted (possibly with its speed cap clamped).
    Admitted,
    /// The proposal was rejected; the core action stands.
    Rejected(RejectReason),
}

/// A seismic action policy (the exotic / choreographer seam).
pub trait ActionPolicy {
    /// Proposes an override of `ctx.default`; `None` keeps the core action. Called once
    /// per supervisor release while the seismic state is not NORMAL (and on the release
    /// that leaves NORMAL).
    fn propose(&mut self, ctx: &ActionContext) -> Option<ActionDecision>;
}

/// The core policy: never overrides (firmware, every core preset).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CorePolicy;

impl ActionPolicy for CorePolicy {
    #[inline]
    fn propose(&mut self, _ctx: &ActionContext) -> Option<ActionDecision> {
        None
    }
}

/// The core-side admission filter: returns the decision to apply and the outcome.
/// A rejected proposal yields `ctx.default` unchanged.
pub fn admit(
    ctx: &ActionContext,
    proposal: Option<ActionDecision>,
) -> (ActionDecision, PolicyOutcome) {
    let Some(mut p) = proposal else {
        return (ctx.default, PolicyOutcome::Default);
    };
    if p == ctx.default {
        return (ctx.default, PolicyOutcome::Default);
    }
    let reject = |r: RejectReason| (ctx.default, PolicyOutcome::Rejected(r));

    if p.action == SeismicAction::None && ctx.default.action != SeismicAction::None {
        return reject(RejectReason::NotTowardSafety);
    }
    if p.action == SeismicAction::OpenAndStop && ctx.non_local() {
        return reject(RejectReason::OpenAndStopNonLocal);
    }
    if p.action == SeismicAction::Lock && ctx.egress != EgressRole::None {
        return reject(RejectReason::LockOnEgress);
    }
    if p.moves() {
        if !ctx.confirmed() {
            return reject(RejectReason::Unconfirmed);
        }
        // Isolate/Damp act about the current position (research tier, opt-in); every
        // other motion must name a target so its direction can be checked. Close-type
        // actions without a target head for the closed stop.
        let target = match (p.target, p.action) {
            (Some(t), _) => Some(t),
            (None, SeismicAction::CompleteClose | SeismicAction::CloseAfterShaking) => {
                Some(ctx.x_closed)
            }
            (None, SeismicAction::Isolate | SeismicAction::Damp) => None,
            (None, _) => return reject(RejectReason::NoTarget),
        };
        if let Some(t) = target {
            // Closing is −x (PHYSICS §1): the target must not lie on the opening side.
            if t.diff_nm(ctx.x) > 0 {
                return reject(RejectReason::Opening);
            }
            let moving_close = t.diff_nm(ctx.x) < 0;
            if moving_close && ctx.user_motion == Some(TravelDir::Open) {
                return reject(RejectReason::ReversesUser);
            }
            p.target = Some(t);
        }
        let limit = fmath::fmaxf(ctx.v_unattended_mps, 0.0);
        p.v_cap_mps = if p.v_cap_mps.is_finite() && p.v_cap_mps > 0.0 {
            fmath::fminf(p.v_cap_mps, limit)
        } else {
            limit
        };
    }
    (p, PolicyOutcome::Admitted)
}

/// Asks `policy` for a proposal and filters it through [`admit`] (what the supervisor
/// calls each release).
pub fn resolve(
    ctx: &ActionContext,
    policy: &mut dyn ActionPolicy,
) -> (ActionDecision, PolicyOutcome) {
    let proposal = policy.propose(ctx);
    admit(ctx, proposal)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx() -> ActionContext {
        ActionContext {
            class: RailClass::Drawer,
            egress: EgressRole::None,
            seis: SeismicState::Protect,
            x: PosNm(200_000_000),
            x_closed: PosNm(0),
            default: ActionDecision::of(SeismicAction::Freeze),
            v_unattended_mps: 0.23,
            min_independent_feeds: 2,
            report: SeismicReport {
                source: AlertSource::Network,
                n_feeds: 1,
                ..SeismicReport::default()
            },
            ..ActionContext::default()
        }
    }

    struct Fixed(ActionDecision);
    impl ActionPolicy for Fixed {
        fn propose(&mut self, _c: &ActionContext) -> Option<ActionDecision> {
            Some(self.0)
        }
    }

    #[test]
    fn core_policy_keeps_default() {
        let c = ctx();
        let (d, o) = resolve(&c, &mut CorePolicy);
        assert_eq!(d, c.default);
        assert_eq!(o, PolicyOutcome::Default);
    }

    #[test]
    fn network_only_alert_cannot_move_or_open() {
        let c = ctx();
        let close = ActionDecision {
            action: SeismicAction::CompleteClose,
            target: Some(PosNm(0)),
            v_cap_mps: 1.0,
            deadline: None,
        };
        assert_eq!(
            admit(&c, Some(close)).1,
            PolicyOutcome::Rejected(RejectReason::Unconfirmed)
        );
        let open = ActionDecision::of(SeismicAction::OpenAndStop);
        assert_eq!(
            admit(&c, Some(open)).1,
            PolicyOutcome::Rejected(RejectReason::OpenAndStopNonLocal)
        );
        // Freeze → Land is allowed from a network-only alert.
        let land = ActionDecision::of(SeismicAction::Land);
        assert_eq!(admit(&c, Some(land)), (land, PolicyOutcome::Admitted));
        // Doing nothing instead of freezing is not toward safety.
        let none = ActionDecision::of(SeismicAction::None);
        assert_eq!(
            admit(&c, Some(none)).1,
            PolicyOutcome::Rejected(RejectReason::NotTowardSafety)
        );
    }

    #[test]
    fn confirmed_close_is_capped_and_directional() {
        let mut c = ctx();
        c.report.local_p = true;
        let close = ActionDecision {
            action: SeismicAction::CompleteClose,
            target: None,
            v_cap_mps: 1.0,
            deadline: None,
        };
        let (d, o) = admit(&c, Some(close));
        assert_eq!(o, PolicyOutcome::Admitted);
        assert_eq!(d.target, Some(PosNm(0)));
        assert_eq!(d.v_cap_mps, 0.23, "clamped to the unattended limit");
        let opening = ActionDecision {
            target: Some(PosNm(300_000_000)),
            ..close
        };
        assert_eq!(
            admit(&c, Some(opening)).1,
            PolicyOutcome::Rejected(RejectReason::Opening)
        );
        c.user_motion = Some(TravelDir::Open);
        assert_eq!(
            admit(&c, Some(close)).1,
            PolicyOutcome::Rejected(RejectReason::ReversesUser)
        );
    }

    #[test]
    fn lock_forbidden_on_egress() {
        let mut c = ctx();
        c.egress = EgressRole::NonRated;
        let lock = ActionDecision::of(SeismicAction::Lock);
        let mut pol = Fixed(lock);
        assert_eq!(
            resolve(&c, &mut pol).1,
            PolicyOutcome::Rejected(RejectReason::LockOnEgress)
        );
    }
}
