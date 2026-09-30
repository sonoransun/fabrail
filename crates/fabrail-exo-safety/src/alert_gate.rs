//! Alert gate: network-only alerts may cause only FREEZE/LAND/LOCK/SEISMIC_PASSIVE; motion needs independent confirmation (FAB-EXO-506). Owner: X1 (API frozen by W0a).
//!
//! The binding alert rule (EXOTIC §0.4, §S2; plan §4, §4b; FAB-EXO-119, 506) as a pure,
//! allocation-free check over the [`SafetyRank`] partial order, consulted by the
//! choreographer and the fusion policy before they propose (the core `admit` filter is
//! the backstop):
//!
//! * FREEZE, LAND and SEISMIC_PASSIVE are always admissible; LOCK is admissible except
//!   on an egress path (egress never locks). These four are the only actions a
//!   network-only or injected alert can cause ([`is_protective`]).
//! * Opening, unlocking, de-isolating and speeding up are never admissible on an alert.
//!   The core `OpenAndStop` class action stays admissible only while no non-local source
//!   contributed ([`open_and_stop_admissible`]).
//! * Alert-triggered **motion** must go to a strictly higher [`SafetyRank`] in the
//!   closing/seating direction, must never reverse a user-initiated motion
//!   ([`UserMotion`]), and needs independent confirmation: a local P pick or at least
//!   [`MIN_INDEPENDENT_FEEDS`] independent feeds. In addition, sovereign classes need the
//!   zone `ClearVerified`; pedestrian classes need obstruction detection armed and the
//!   motion within their low-energy (unattended) caps. Pre-positioning is motion without
//!   a checkable direction, so it is refused while any user motion is in progress.
//! * [`Confirmations::from_alert`] derives the confirmation from a [`FusedAlert`]
//!   conservatively: an injected contribution voids every feed count (only a genuine
//!   local P pick can still confirm), a local-P flag without the LocalP feed is treated
//!   as inconsistent (no confirmation at all), and a PEGS advisory alone never counts
//!   (FAB-EXO-810).
//!
//! A spoofed alert therefore has bounded regret: at worst devices freeze, land or lock,
//! or close at unattended low-energy limits with obstruction detection armed. Every
//! alert in this tier is synthetic or injected; fabrail is not an EEW provider.

use fabrail_types::{EgressRole, RailClass};

use crate::rank::{SafetyRank, alert_may_move};
use crate::types::{AlertFeedKind, AnyClass, ExoClass, FusedAlert};

/// Independent feeds needed for alert-triggered motion without a local P pick.
pub const MIN_INDEPENDENT_FEEDS: u8 = 2;

/// An action an alert (fused, network-only or injected) asks a device to take.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum AlertAction {
    /// FREEZE: decelerate and hold; no new motion.
    Freeze,
    /// LAND in place.
    Land,
    /// LAND and engage the lock.
    Lock,
    /// SEISMIC_PASSIVE: stay levitated with the controller in passive mode (SENTINEL).
    SeismicPassive,
    /// Motion from rank `from` to rank `to`; `closing` is true for the closing/seating
    /// direction.
    Move {
        /// Rank before the motion.
        from: SafetyRank,
        /// Rank after the motion.
        to: SafetyRank,
        /// Motion is in the closing or seating direction.
        closing: bool,
    },
    /// Pre-positioning / centring of an isolation platform or panel.
    PrePosition,
    /// Opening (any class).
    Open,
    /// Releasing a lock or latch.
    Unlock,
    /// Leaving isolation.
    DeIsolate,
    /// Raising a speed or force limit.
    SpeedUp,
}

/// The class the gate is asked about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GateClass {
    /// A core class in the pedestrian regime (the villa drawers, doors and walls).
    Pedestrian {
        /// Rail class.
        class: RailClass,
        /// Egress role of the opening.
        egress: EgressRole,
    },
    /// A sovereign class (guarded zone or occupied platform).
    Sovereign(ExoClass),
}

impl GateClass {
    /// The gate class of `class` with egress role `egress` (sovereign classes have no
    /// pedestrian egress role).
    pub const fn of(class: AnyClass, egress: EgressRole) -> GateClass {
        match class {
            AnyClass::Core(c) => GateClass::Pedestrian { class: c, egress },
            AnyClass::Exo(x) => GateClass::Sovereign(x),
        }
    }
}

/// A user-initiated motion in progress on the device (never reversed by an alert).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum UserMotion {
    /// No user motion.
    #[default]
    None = 0,
    /// The user is opening (+x).
    Opening = 1,
    /// The user is closing (−x).
    Closing = 2,
}

/// What confirms an alert and what the device has armed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Confirmations {
    /// A local P pick contributed.
    pub local_p: bool,
    /// Independent feeds contributing, excluding injected feeds.
    pub n_independent_feeds: u8,
    /// The zone is `ClearVerified` (sovereign classes).
    pub zone_clear_verified: bool,
    /// Obstruction detection is armed (pedestrian classes).
    pub obstruction_armed: bool,
    /// The commanded motion is within the class's low-energy (unattended) caps.
    pub within_low_energy_caps: bool,
}

impl Confirmations {
    /// Independent confirmation: a local P pick or enough independent feeds.
    pub const fn independent(&self) -> bool {
        self.local_p || self.n_independent_feeds >= MIN_INDEPENDENT_FEEDS
    }

    /// The confirmation carried by the fused alert `a`, plus the device's own flags.
    /// Conservative: a local-P flag and the LocalP feed must agree (otherwise nothing
    /// confirms); an injected contribution voids every feed count, so only a genuine
    /// local P pick can confirm; a PEGS advisory alone never counts.
    pub const fn from_alert(
        a: &FusedAlert,
        zone_clear_verified: bool,
        obstruction_armed: bool,
        within_low_energy_caps: bool,
    ) -> Confirmations {
        let has_local_feed = a.feeds.contains(AlertFeedKind::LocalP);
        let consistent = a.local_p == has_local_feed;
        let local_p = consistent && a.local_p;
        let n_independent_feeds = if !consistent || a.feeds.0 == 0 || a.feeds.pegs_only() {
            0
        } else if a.injected {
            local_p as u8
        } else {
            a.n_independent
        };
        Confirmations {
            local_p,
            n_independent_feeds,
            zone_clear_verified,
            obstruction_armed,
            within_low_energy_caps,
        }
    }
}

/// Why the gate refused an action.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum DenyReason {
    /// Opening, unlocking, de-isolating or speeding up.
    ForbiddenOnAlert = 0,
    /// LOCK on an egress path.
    LockOnEgress = 1,
    /// Motion without a strict rank gain.
    NotTowardHigherRank = 2,
    /// Motion in the opening direction.
    OpeningDirection = 3,
    /// Motion without independent confirmation.
    Unconfirmed = 4,
    /// Sovereign motion without a `ClearVerified` zone.
    ZoneNotClearVerified = 5,
    /// Pedestrian motion without armed obstruction detection.
    ObstructionNotArmed = 6,
    /// Pedestrian motion above the low-energy caps.
    AboveLowEnergyCaps = 7,
    /// Motion that would reverse a user-initiated motion (or pre-positioning while a
    /// user motion is in progress).
    ReversesUserMotion = 8,
}

/// The gate's decision.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum GateVerdict {
    /// The action may proceed.
    Allowed,
    /// The action is refused; the device keeps its core (land-first) action.
    Denied(DenyReason),
}

impl GateVerdict {
    /// True for [`GateVerdict::Allowed`].
    pub const fn is_allowed(self) -> bool {
        matches!(self, GateVerdict::Allowed)
    }
}

/// True for the four actions a network-only or injected alert may cause: FREEZE, LAND,
/// LOCK and SEISMIC_PASSIVE (LOCK still refused on an egress path by [`gate`]).
pub const fn is_protective(action: AlertAction) -> bool {
    matches!(
        action,
        AlertAction::Freeze | AlertAction::Land | AlertAction::Lock | AlertAction::SeismicPassive
    )
}

/// The core `OpenAndStop` class action (egress doors on a local detection) is
/// admissible only when no non-local source (network, offshore, DAS, PEGS or an
/// injection) contributed to the alert.
pub const fn open_and_stop_admissible(a: &FusedAlert) -> bool {
    !a.non_local()
}

/// Decides whether an alert may cause `action` on a device of `class` with no user
/// motion in progress (see [`gate_with_user`]).
pub const fn gate(action: AlertAction, c: &Confirmations, class: GateClass) -> GateVerdict {
    gate_with_user(action, c, class, UserMotion::None)
}

/// Decides whether an alert may cause `action` on a device of `class` while `user`
/// motion is in progress.
pub const fn gate_with_user(
    action: AlertAction,
    c: &Confirmations,
    class: GateClass,
    user: UserMotion,
) -> GateVerdict {
    match action {
        AlertAction::Freeze | AlertAction::Land | AlertAction::SeismicPassive => {
            GateVerdict::Allowed
        }
        AlertAction::Lock => match class {
            GateClass::Pedestrian {
                egress: EgressRole::NonRated | EgressRole::FireRated,
                ..
            } => GateVerdict::Denied(DenyReason::LockOnEgress),
            _ => GateVerdict::Allowed,
        },
        AlertAction::Open | AlertAction::Unlock | AlertAction::DeIsolate | AlertAction::SpeedUp => {
            GateVerdict::Denied(DenyReason::ForbiddenOnAlert)
        }
        AlertAction::Move { from, to, closing } => {
            if !closing {
                return GateVerdict::Denied(DenyReason::OpeningDirection);
            }
            if !alert_may_move(from, to) {
                return GateVerdict::Denied(DenyReason::NotTowardHigherRank);
            }
            if matches!(user, UserMotion::Opening) {
                return GateVerdict::Denied(DenyReason::ReversesUserMotion);
            }
            motion_checks(c, class)
        }
        AlertAction::PrePosition => {
            if !matches!(user, UserMotion::None) {
                return GateVerdict::Denied(DenyReason::ReversesUserMotion);
            }
            motion_checks(c, class)
        }
    }
}

/// Confirmation and per-regime checks shared by every alert-triggered motion.
const fn motion_checks(c: &Confirmations, class: GateClass) -> GateVerdict {
    if !c.independent() {
        return GateVerdict::Denied(DenyReason::Unconfirmed);
    }
    match class {
        GateClass::Sovereign(_) => {
            if !c.zone_clear_verified {
                return GateVerdict::Denied(DenyReason::ZoneNotClearVerified);
            }
        }
        GateClass::Pedestrian { .. } => {
            if !c.obstruction_armed {
                return GateVerdict::Denied(DenyReason::ObstructionNotArmed);
            }
            if !c.within_low_energy_caps {
                return GateVerdict::Denied(DenyReason::AboveLowEnergyCaps);
            }
        }
    }
    GateVerdict::Allowed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn network_only_alerts_cause_only_protective_actions() {
        let net_only = Confirmations {
            n_independent_feeds: 1,
            zone_clear_verified: true,
            obstruction_armed: true,
            within_low_energy_caps: true,
            ..Confirmations::default()
        };
        let drawer = GateClass::Pedestrian {
            class: RailClass::Drawer,
            egress: EgressRole::None,
        };
        for a in [
            AlertAction::Freeze,
            AlertAction::Land,
            AlertAction::Lock,
            AlertAction::SeismicPassive,
        ] {
            assert!(gate(a, &net_only, drawer).is_allowed(), "{a:?}");
        }
        let close = AlertAction::Move {
            from: SafetyRank::FrozenOpen,
            to: SafetyRank::SeatedOrLocked,
            closing: true,
        };
        assert_eq!(
            gate(close, &net_only, drawer),
            GateVerdict::Denied(DenyReason::Unconfirmed)
        );
        let confirmed = Confirmations {
            local_p: true,
            ..net_only
        };
        assert!(gate(close, &confirmed, drawer).is_allowed());
        let door = GateClass::Pedestrian {
            class: RailClass::Door,
            egress: EgressRole::NonRated,
        };
        assert_eq!(
            gate(AlertAction::Lock, &confirmed, door),
            GateVerdict::Denied(DenyReason::LockOnEgress)
        );
        for a in [
            AlertAction::Open,
            AlertAction::Unlock,
            AlertAction::DeIsolate,
            AlertAction::SpeedUp,
        ] {
            assert_eq!(
                gate(a, &confirmed, drawer),
                GateVerdict::Denied(DenyReason::ForbiddenOnAlert)
            );
        }
        let sovereign = GateClass::Sovereign(ExoClass::HavenR);
        let no_zone = Confirmations {
            zone_clear_verified: false,
            ..confirmed
        };
        assert_eq!(
            gate(AlertAction::PrePosition, &no_zone, sovereign),
            GateVerdict::Denied(DenyReason::ZoneNotClearVerified)
        );
    }

    use crate::rank::DeviceCondition;
    use crate::types::FeedSet;
    use fabrail_types::{NetworkAlert, SeismicLevel};

    const EGRESS: [EgressRole; 3] = [
        EgressRole::None,
        EgressRole::NonRated,
        EgressRole::FireRated,
    ];
    const USERS: [UserMotion; 3] = [UserMotion::None, UserMotion::Opening, UserMotion::Closing];

    /// Every gate class: the four core classes on every egress role, and the nine
    /// sovereign classes.
    fn classes() -> Vec<GateClass> {
        let mut v = Vec::new();
        for c in [
            RailClass::Drawer,
            RailClass::Door,
            RailClass::Wall,
            RailClass::Heavy,
        ] {
            for e in EGRESS {
                v.push(GateClass::of(AnyClass::Core(c), e));
            }
        }
        for &x in ExoClass::ALL {
            v.push(GateClass::of(AnyClass::Exo(x), EgressRole::None));
        }
        v
    }

    /// Every alert action: the protective four, the forbidden four, pre-positioning and
    /// every (from, to, direction) motion over the rank order.
    fn actions() -> Vec<AlertAction> {
        let mut v = vec![
            AlertAction::Freeze,
            AlertAction::Land,
            AlertAction::Lock,
            AlertAction::SeismicPassive,
            AlertAction::PrePosition,
            AlertAction::Open,
            AlertAction::Unlock,
            AlertAction::DeIsolate,
            AlertAction::SpeedUp,
        ];
        for &from in SafetyRank::ALL {
            for &to in SafetyRank::ALL {
                for closing in [false, true] {
                    v.push(AlertAction::Move { from, to, closing });
                }
            }
        }
        v
    }

    fn fused(feeds: &[AlertFeedKind], n: u8, local_p: bool, injected: bool) -> FusedAlert {
        FusedAlert {
            alert: NetworkAlert::default(),
            level: SeismicLevel::default(),
            feeds: feeds.iter().fold(FeedSet::EMPTY, |s, k| s.with(*k)),
            n_independent: n,
            local_p,
            injected,
            ..FusedAlert::default()
        }
    }

    /// The oracle, written from the spec text (EXOTIC §0.4, FAB-EXO-119) independently
    /// of `gate_with_user`.
    fn expected(
        a: AlertAction,
        confirmed: bool,
        class: GateClass,
        flags: (bool, bool, bool),
        user: UserMotion,
    ) -> bool {
        let (zone_clear, armed, caps) = flags;
        let regime_ok = match class {
            GateClass::Sovereign(_) => zone_clear,
            GateClass::Pedestrian { .. } => armed && caps,
        };
        let egress =
            matches!(class, GateClass::Pedestrian { egress, .. } if egress != EgressRole::None);
        match a {
            AlertAction::Freeze | AlertAction::Land | AlertAction::SeismicPassive => true,
            AlertAction::Lock => !egress,
            AlertAction::Open
            | AlertAction::Unlock
            | AlertAction::DeIsolate
            | AlertAction::SpeedUp => false,
            AlertAction::Move { from, to, closing } => {
                closing && to > from && user != UserMotion::Opening && confirmed && regime_ok
            }
            AlertAction::PrePosition => user == UserMotion::None && confirmed && regime_ok,
        }
    }

    // req: FAB-EXO-119
    #[test]
    fn spoof_action_matrix() {
        use AlertFeedKind::{Das, LocalP, Network, OffshoreCabled, PegsMagnitudeAdvisory as Pegs};
        // (name, alert, independently confirmed per EXOTIC §0.4).
        let sources = [
            ("local P only", fused(&[LocalP], 1, true, false), true),
            (
                "local P + network",
                fused(&[LocalP, Network], 2, true, false),
                true,
            ),
            ("network, 1 feed", fused(&[Network], 1, false, false), false),
            (
                "network, 2 providers",
                fused(&[Network], 2, false, false),
                true,
            ),
            (
                "network + offshore",
                fused(&[Network, OffshoreCabled], 2, false, false),
                true,
            ),
            (
                "offshore only",
                fused(&[OffshoreCabled], 1, false, false),
                false,
            ),
            ("DAS only", fused(&[Das], 1, false, false), false),
            ("PEGS only", fused(&[Pegs], 1, false, false), false),
            (
                "PEGS only, count inflated",
                fused(&[Pegs], 3, false, false),
                false,
            ),
            (
                "PEGS + network",
                fused(&[Pegs, Network], 2, false, false),
                true,
            ),
            ("injected network", fused(&[Network], 0, false, true), false),
            (
                "injected, count inflated",
                fused(&[Network, Das, OffshoreCabled], 3, false, true),
                false,
            ),
            (
                "injected + genuine local P",
                fused(&[LocalP, Network], 2, true, true),
                true,
            ),
            (
                "local-P flag without the feed",
                fused(&[Network], 1, true, false),
                false,
            ),
            (
                "LocalP feed without the flag",
                fused(&[LocalP, Network], 2, false, false),
                false,
            ),
            ("empty", fused(&[], 0, false, false), false),
            ("empty, count inflated", fused(&[], 5, false, false), false),
        ];
        let (classes, actions) = (classes(), actions());
        let mut n_checked = 0usize;
        let mut n_allowed_unconfirmed_motion = 0usize;
        for (name, alert, confirmed) in &sources {
            // The core OpenAndStop survives only a purely local alert.
            assert_eq!(
                open_and_stop_admissible(alert),
                !alert.non_local(),
                "{name}: OpenAndStop with a non-local source"
            );
            if alert.non_local() {
                assert!(!open_and_stop_admissible(alert), "{name}");
            }
            for flags in 0u8..8 {
                let f = (flags & 1 != 0, flags & 2 != 0, flags & 4 != 0);
                let c = Confirmations::from_alert(alert, f.0, f.1, f.2);
                assert_eq!(
                    c.independent(),
                    *confirmed,
                    "{name}: independent confirmation"
                );
                for &class in &classes {
                    for &a in &actions {
                        for user in USERS {
                            let v = gate_with_user(a, &c, class, user);
                            let want = expected(a, *confirmed, class, f, user);
                            assert_eq!(
                                v.is_allowed(),
                                want,
                                "{name} | {a:?} | {class:?} | flags {f:?} | user {user:?}: {v:?}"
                            );
                            // Network-only or injected (unconfirmed) alerts cause only
                            // FREEZE, LAND, LOCK or SEISMIC_PASSIVE (FAB-EXO-119).
                            if !confirmed && v.is_allowed() {
                                assert!(is_protective(a), "{name}: {a:?} allowed unconfirmed");
                            }
                            if !confirmed && v.is_allowed() && !is_protective(a) {
                                n_allowed_unconfirmed_motion += 1;
                            }
                            if user == UserMotion::None {
                                assert_eq!(gate(a, &c, class), v);
                            }
                            n_checked += 1;
                        }
                    }
                }
            }
        }
        assert_eq!(n_allowed_unconfirmed_motion, 0);
        assert_eq!(
            n_checked,
            sources.len() * 8 * classes.len() * actions.len() * USERS.len(),
            "exhaustive: every source × flags × class × action × user motion"
        );
        assert_eq!(classes.len(), 21);
        assert_eq!(actions.len(), 41);
    }

    // req: FAB-EXO-506
    #[test]
    fn exhaustive_transitions() {
        let full = Confirmations {
            local_p: true,
            n_independent_feeds: 2,
            zone_clear_verified: true,
            obstruction_armed: true,
            within_low_energy_caps: true,
        };
        let mut n = 0usize;
        for class in classes() {
            let egress = match class {
                GateClass::Pedestrian { egress, .. } => egress,
                GateClass::Sovereign(_) => EgressRole::None,
            };
            for &from in DeviceCondition::ALL {
                for &to in DeviceCondition::ALL {
                    let (rf, rt) = (SafetyRank::of(from, egress), SafetyRank::of(to, egress));
                    for closing in [false, true] {
                        for user in USERS {
                            let a = AlertAction::Move {
                                from: rf,
                                to: rt,
                                closing,
                            };
                            let v = gate_with_user(a, &full, class, user);
                            // Fully confirmed: an alert moves a device only toward a
                            // strictly higher rank, closing, never against the user.
                            let want = rt > rf && closing && user != UserMotion::Opening;
                            assert_eq!(
                                v.is_allowed(),
                                want,
                                "{class:?} {from:?}->{to:?} closing {closing} {user:?}"
                            );
                            let why = match v {
                                GateVerdict::Denied(r) => Some(r),
                                GateVerdict::Allowed => None,
                            };
                            if !closing {
                                assert_eq!(why, Some(DenyReason::OpeningDirection));
                            } else if rt <= rf {
                                assert_eq!(why, Some(DenyReason::NotTowardHigherRank));
                            } else if user == UserMotion::Opening {
                                assert_eq!(why, Some(DenyReason::ReversesUserMotion));
                            }
                            // Without confirmation nothing moves.
                            let none = Confirmations {
                                local_p: false,
                                n_independent_feeds: 1,
                                ..full
                            };
                            assert!(!gate_with_user(a, &none, class, user).is_allowed());
                            n += 1;
                        }
                    }
                }
            }
        }
        assert_eq!(n, 21 * 8 * 8 * 2 * 3);

        // Chains of admissible alert moves strictly increase the rank: no cycle, at most
        // three moves from Frozen-open to Seated/Locked, and nothing leaves the top rank.
        let allowed = |a: SafetyRank, b: SafetyRank| {
            gate(
                AlertAction::Move {
                    from: a,
                    to: b,
                    closing: true,
                },
                &full,
                GateClass::Sovereign(ExoClass::Shield),
            )
            .is_allowed()
        };
        let mut longest = 0;
        let mut stack: Vec<(SafetyRank, usize)> = SafetyRank::ALL.iter().map(|&r| (r, 0)).collect();
        while let Some((r, depth)) = stack.pop() {
            longest = longest.max(depth);
            assert!(depth <= 3, "alert-move chain longer than the rank order");
            for &nxt in SafetyRank::ALL {
                if allowed(r, nxt) {
                    assert!(nxt > r);
                    stack.push((nxt, depth + 1));
                }
            }
        }
        assert_eq!(longest, 3);
        for &r in SafetyRank::ALL {
            assert!(
                !allowed(SafetyRank::SeatedOrLocked, r),
                "nothing above Seated/Locked"
            );
        }
    }
}
