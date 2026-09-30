//! SafetyRank partial order: Seated or Landed+Locked > Closed-levitated > Standoff/Hold > Frozen-open (plan §4b). Owner: W0a (complete in Wave 0, W0-C1).
//!
//! "Toward safety" is this order (EXOTIC §0.4, §S2; FAB-EXO-506). An alert may command
//! motion only toward a strictly higher rank, only in the closing or seating direction
//! (checked by [`crate::alert_gate`]), and never reverses a user-initiated motion.
//!
//! A device's physical condition ([`DeviceCondition`]) maps onto a rank with
//! [`SafetyRank::of`]. The egress role matters for one condition: an egress opening
//! never locks, so landed (with ASC released after the stop) is already its top rank,
//! whereas a non-egress device that has landed but not yet locked sits one rank below
//! Seated/Landed+Locked.

use fabrail_types::EgressRole;

use crate::types::u8_enum;

u8_enum! {
    /// Safety rank, ordered: a larger value is safer.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum SafetyRank {
        /// Open and frozen (or moving): the lowest rank.
        #[default]
        FrozenOpen = 0,
        /// Held at a standoff or in position hold.
        StandoffHold = 1,
        /// Closed and levitated (or landed, not yet locked, off the egress path).
        ClosedLevitated = 2,
        /// Seated (DRAWER captive seat) or landed and locked (egress: landed).
        SeatedOrLocked = 3,
    }
}

u8_enum! {
    /// Physical condition of a device as the choreographer and the gate see it.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum DeviceCondition {
        /// In motion (user-initiated or automatic).
        #[default]
        Moving = 0,
        /// Stopped away from the closed position by a FREEZE.
        FrozenOpen = 1,
        /// Parked at a standoff (e.g. a drawer at s_min in smooth ride-through).
        Standoff = 2,
        /// Position hold, levitated, away from the closed position.
        Hold = 3,
        /// At the closed position, levitated.
        ClosedLevitated = 4,
        /// Resting on rollers/skids, not locked.
        Landed = 5,
        /// DRAWER in its captive seat.
        Seated = 6,
        /// Landed with the parking lock engaged.
        LandedLocked = 7,
    }
}

impl SafetyRank {
    /// The rank of condition `c` for an opening with egress role `egress`.
    pub const fn of(c: DeviceCondition, egress: EgressRole) -> SafetyRank {
        match c {
            DeviceCondition::Moving | DeviceCondition::FrozenOpen => SafetyRank::FrozenOpen,
            DeviceCondition::Standoff | DeviceCondition::Hold => SafetyRank::StandoffHold,
            DeviceCondition::ClosedLevitated => SafetyRank::ClosedLevitated,
            DeviceCondition::Seated | DeviceCondition::LandedLocked => SafetyRank::SeatedOrLocked,
            DeviceCondition::Landed => match egress {
                // Egress never locks: landed (ASC released) is the terminal safe state.
                EgressRole::NonRated | EgressRole::FireRated => SafetyRank::SeatedOrLocked,
                EgressRole::None => SafetyRank::ClosedLevitated,
            },
        }
    }

    /// True when `self` is strictly safer than `other`.
    pub const fn is_higher_than(self, other: SafetyRank) -> bool {
        self.as_u8() > other.as_u8()
    }
}

/// The alert-motion rank rule: an alert may command motion from `from` to `to` only
/// when `to` is strictly higher (FAB-EXO-506). Direction, confirmation and limits are
/// checked by [`crate::alert_gate::gate`].
pub const fn alert_may_move(from: SafetyRank, to: SafetyRank) -> bool {
    to.is_higher_than(from)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EGRESS: [EgressRole; 3] = [
        EgressRole::None,
        EgressRole::NonRated,
        EgressRole::FireRated,
    ];

    // req: FAB-EXO-506
    #[test]
    fn alert_moves_only_toward_higher_rank() {
        // The order itself (plan §4b): Seated/Locked > Closed-levitated > Standoff/Hold
        // > Frozen-open, total and strict.
        assert!(SafetyRank::SeatedOrLocked > SafetyRank::ClosedLevitated);
        assert!(SafetyRank::ClosedLevitated > SafetyRank::StandoffHold);
        assert!(SafetyRank::StandoffHold > SafetyRank::FrozenOpen);
        for &a in SafetyRank::ALL {
            assert!(
                !alert_may_move(a, a),
                "{a:?}: no motion without a rank gain"
            );
            for &b in SafetyRank::ALL {
                assert_eq!(alert_may_move(a, b), b > a, "{a:?} -> {b:?}");
                assert!(
                    !(alert_may_move(a, b) && alert_may_move(b, a)),
                    "antisymmetric"
                );
                for &c in SafetyRank::ALL {
                    if alert_may_move(a, b) && alert_may_move(b, c) {
                        assert!(alert_may_move(a, c), "transitive");
                    }
                }
            }
        }
        // Exhaustive over conditions and egress roles.
        for &e in &EGRESS {
            for &from in DeviceCondition::ALL {
                for &to in DeviceCondition::ALL {
                    let (rf, rt) = (SafetyRank::of(from, e), SafetyRank::of(to, e));
                    assert_eq!(alert_may_move(rf, rt), rt > rf, "{e:?}: {from:?} -> {to:?}");
                }
            }
            // Nothing is above seated / landed + locked.
            let top = SafetyRank::of(DeviceCondition::LandedLocked, e);
            assert_eq!(top, SafetyRank::SeatedOrLocked);
            for &to in DeviceCondition::ALL {
                assert!(!alert_may_move(top, SafetyRank::of(to, e)));
            }
            // A frozen-open device may be closed or seated, never "moved" sideways.
            let low = SafetyRank::of(DeviceCondition::FrozenOpen, e);
            assert!(!alert_may_move(
                low,
                SafetyRank::of(DeviceCondition::Moving, e)
            ));
            assert!(alert_may_move(
                low,
                SafetyRank::of(DeviceCondition::Seated, e)
            ));
        }
        // Egress never locks, so landed is its top rank; off the egress path, landed
        // without the lock is one rank below.
        assert_eq!(
            SafetyRank::of(DeviceCondition::Landed, EgressRole::NonRated),
            SafetyRank::SeatedOrLocked
        );
        assert_eq!(
            SafetyRank::of(DeviceCondition::Landed, EgressRole::None),
            SafetyRank::ClosedLevitated
        );
    }
}
