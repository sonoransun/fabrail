//! ChoreoPolicy and per-device policies (SmoothRideThrough admission, SeatAndHold, SeatAndLatch, StopAndLand); the alert-motion rule. Owner: E2 (API frozen by W0a).
//!
//! Plan §4b. Per-device policies:
//! * `SmoothRideThrough` (research tier): only for drawers already open by at least
//!   s_min = 25 mm + S_max,pred when the alert arrives, subject to the admission list of
//!   plan §4b ([`SmoothAdmissionInputs`], [`admit_smooth_ride_through`]); seated drawers
//!   never unseat on an alert;
//! * `SeatAndHold`; `SeatAndLatch` only with `landing.seismic_latch` fitted
//!   (x_retention_n ≥ m_rated·a_design, [`latch_holds`]);
//! * `StopAndLand` for egress doors (never lock; ASC released after the stop);
//! * `LandAndLock` off the egress path; `RideThrough`, `CompleteClose`,
//!   `CloseAfterShaking` for the sovereign classes.
//!
//! Alert-initiated motion only toward a higher `SafetyRank`, in the closing/seating
//! direction, at unattended limits, one attempt, independent confirmation, never
//! reversing a user motion. Every command the choreographer proposes is first put to
//! `fabrail_exo_safety::alert_gate` ([`gate_decision`]); the core `admit` filter is the
//! backstop. The choreographer is not a rated safety function.

use fabrail_core::seismic::action_policy::{ActionContext, ActionDecision, ActionPolicy};
use fabrail_exo_safety::alert_gate::{
    AlertAction, Confirmations, GateClass, GateVerdict, UserMotion, gate_with_user,
};
use fabrail_exo_safety::{AnyClass, ExoClass, FusedAlert, SafetyRank, presets};
use fabrail_types::{EgressRole, RailClass, SeismicAction, Tick, TravelDir};

use crate::choreo::registry::DeviceEntry;

/// Per-device choreographer policy (scene JSON `policy`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[repr(u8)]
pub enum ChoreoPolicy {
    /// By class, egress role and admission ([`resolve_auto`]).
    #[default]
    Auto = 0,
    /// Research tier: already-open drawers stop at s_min and ride through inertially.
    SmoothRideThrough = 1,
    /// Log-damped close to the seat, then hold.
    SeatAndHold = 2,
    /// Seat and engage the seismic latch (latch fitted only).
    SeatAndLatch = 3,
    /// Egress: stop, land, release ASC; never lock.
    StopAndLand = 4,
    /// Off the egress path: land and lock.
    LandAndLock = 5,
    /// Stay levitated (SENTINEL SEISMIC_PASSIVE).
    RideThrough = 6,
    /// Finish an in-progress close, else land (BASTION).
    CompleteClose = 7,
    /// Close after shaking on the external command (TIDEGATE).
    CloseAfterShaking = 8,
}

impl ChoreoPolicy {
    /// Every policy, in discriminant order.
    pub const ALL: [ChoreoPolicy; 9] = [
        ChoreoPolicy::Auto,
        ChoreoPolicy::SmoothRideThrough,
        ChoreoPolicy::SeatAndHold,
        ChoreoPolicy::SeatAndLatch,
        ChoreoPolicy::StopAndLand,
        ChoreoPolicy::LandAndLock,
        ChoreoPolicy::RideThrough,
        ChoreoPolicy::CompleteClose,
        ChoreoPolicy::CloseAfterShaking,
    ];

    /// Snake-case name (scene JSON).
    pub const fn slug(self) -> &'static str {
        match self {
            ChoreoPolicy::Auto => "auto",
            ChoreoPolicy::SmoothRideThrough => "smooth_ride_through",
            ChoreoPolicy::SeatAndHold => "seat_and_hold",
            ChoreoPolicy::SeatAndLatch => "seat_and_latch",
            ChoreoPolicy::StopAndLand => "stop_and_land",
            ChoreoPolicy::LandAndLock => "land_and_lock",
            ChoreoPolicy::RideThrough => "ride_through",
            ChoreoPolicy::CompleteClose => "complete_close",
            ChoreoPolicy::CloseAfterShaking => "close_after_shaking",
        }
    }

    /// Policies only a DRAWER may use.
    pub const fn drawer_only(self) -> bool {
        matches!(
            self,
            ChoreoPolicy::SmoothRideThrough
                | ChoreoPolicy::SeatAndHold
                | ChoreoPolicy::SeatAndLatch
        )
    }

    /// Policies that lock or latch (never on an egress path).
    pub const fn locks(self) -> bool {
        matches!(self, ChoreoPolicy::LandAndLock | ChoreoPolicy::SeatAndLatch)
    }

    /// Whether `self` may be configured for `class` with egress role `egress`.
    pub const fn allowed(self, class: AnyClass, egress: EgressRole) -> bool {
        let egress_path = !matches!(egress, EgressRole::None);
        if egress_path && self.locks() {
            return false;
        }
        if self.drawer_only() && !matches!(class, AnyClass::Core(RailClass::Drawer)) {
            return false;
        }
        true
    }

    /// The core seismic action this policy proposes (through the `ActionPolicy` hook).
    pub const fn seismic_action(self) -> SeismicAction {
        match self {
            ChoreoPolicy::Auto => SeismicAction::None,
            ChoreoPolicy::SmoothRideThrough | ChoreoPolicy::RideThrough => {
                SeismicAction::RideThrough
            }
            ChoreoPolicy::SeatAndHold => SeismicAction::Land,
            ChoreoPolicy::SeatAndLatch | ChoreoPolicy::LandAndLock => SeismicAction::Lock,
            ChoreoPolicy::StopAndLand => SeismicAction::Land,
            ChoreoPolicy::CompleteClose => SeismicAction::CompleteClose,
            ChoreoPolicy::CloseAfterShaking => SeismicAction::CloseAfterShaking,
        }
    }
}

/// The policy `Auto` resolves to for device `e` (by class, egress and latch):
/// DRAWER → SeatAndLatch with a latch fitted, else SeatAndHold; an egress DOOR/WALL →
/// StopAndLand; a non-egress DOOR → StopAndLand; WALL/HEAVY off the egress path →
/// LandAndLock; sovereign classes → their preset safe state.
pub const fn resolve_auto(e: &DeviceEntry) -> ChoreoPolicy {
    let egress_path = !matches!(e.egress, EgressRole::None);
    match e.class {
        AnyClass::Core(RailClass::Drawer) => {
            if e.constraints.latch_fitted && !egress_path {
                ChoreoPolicy::SeatAndLatch
            } else {
                ChoreoPolicy::SeatAndHold
            }
        }
        AnyClass::Core(RailClass::Door) => ChoreoPolicy::StopAndLand,
        AnyClass::Core(RailClass::Wall | RailClass::Heavy) => {
            if egress_path {
                ChoreoPolicy::StopAndLand
            } else {
                ChoreoPolicy::LandAndLock
            }
        }
        AnyClass::Exo(c) => exo_policy(c),
    }
}

/// The choreographer policy matching a sovereign class's safe state.
pub const fn exo_policy(c: ExoClass) -> ChoreoPolicy {
    use fabrail_exo_safety::SafeStatePolicy as S;
    match presets::preset(c).safe_state {
        S::LandAndLock => ChoreoPolicy::LandAndLock,
        S::IsolateThenLandFps | S::RideThrough => ChoreoPolicy::RideThrough,
        S::CompleteClose => ChoreoPolicy::CompleteClose,
        S::BrakeAndLand => ChoreoPolicy::StopAndLand,
        S::CloseAfterShaking => ChoreoPolicy::CloseAfterShaking,
    }
}

/// True when device `e`'s seismic latch holds a floor acceleration of `a_design_mps2`
/// (device x): the latch is fitted and x_retention_n ≥ m·a_design with m = m_gov
/// (≥ m_rated, the governor mass; plan §4b "x_retention_n ≥ m_rated·a_design"). With
/// the choreographer's building prediction, a_design is the predicted PFA_x.
pub fn latch_holds(e: &DeviceEntry, a_design_mps2: f32) -> bool {
    let c = &e.constraints;
    let demand = e.m_gov_kg * a_design_mps2;
    c.latch_fitted
        && c.latch_retention_n.is_finite()
        && c.latch_retention_n > 0.0
        && demand.is_finite()
        && c.latch_retention_n >= demand
}

/// Inputs of the SmoothRideThrough admission (plan §4b).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoothAdmissionInputs {
    /// The drawer is seated (seated drawers never unseat on an alert).
    pub seated: bool,
    /// Current opening from the seat, m.
    pub x_open_m: f32,
    /// Standoff s_min = 25 mm + S_max,pred, m.
    pub s_min_m: f32,
    /// Predicted relative speed, m/s (≤ `v_rel_frac`·v_cap).
    pub v_rel_pred_mps: f32,
    /// Class speed cap, m/s.
    pub v_cap_mps: f32,
    /// Admission fraction of the speed cap (`control.motion.inertial.v_rel_frac`, 0.8 in
    /// every preset; plan §4b). The default 0 admits nothing.
    pub v_rel_frac: f32,
    /// Predicted stroke fits the standoff.
    pub stroke_fits: bool,
    /// Force budget m·a_LP + μ·m·|a_y| + c_rail·|v_rel| + F_hyst ≤ F_cont holds.
    pub force_budget_ok: bool,
    /// Predicted vertical acceleration, m/s².
    pub a_z_pred_mps2: f32,
    /// z headroom 0.5·p²·(g_float − g_up), m/s².
    pub a_z_headroom_mps2: f32,
    /// Mains available (or a building UPS).
    pub mains_or_ups: bool,
    /// Supercap usable energy, J.
    pub e_sc_j: f32,
    /// One Safe Landing, J.
    pub e_land_j: f32,
    /// One seat step, J.
    pub e_seat_j: f32,
}

/// Hold-up margin of the admission energy check: E_sc ≥ 3·E_land + E_seat (plan §4b;
/// the SAF-009 3× hold-up rule, `control.supervisor` hold-up margin 3).
pub const SMOOTH_HOLDUP_MARGIN: f32 = 3.0;

/// SmoothRideThrough admission (plan §4b, PHYSICS §16.2), research tier: every one of
/// * not seated (seated drawers never unseat on an alert), and already open by at least
///   s_min = 25 mm + S_max,pred (the drawer then closes to s_min);
/// * v_rel,pred ≤ v_rel_frac·v_cap (0.8·v_cap);
/// * the predicted stroke fits the standoff, and the force budget holds;
/// * a_z,pred ≤ 0.5·p²·(g_float − g_up) (z headroom; no z-isolation claim);
/// * mains (or a building UPS), and E_sc ≥ 3·E_land + E_seat (every exit is a
///   log-damped seat on the supercap, then SS1 — seat-first).
///
/// Any non-finite input refuses (the drawer seats and holds instead).
pub fn admit_smooth_ride_through(i: &SmoothAdmissionInputs) -> bool {
    let finite = [
        i.x_open_m,
        i.s_min_m,
        i.v_rel_pred_mps,
        i.v_cap_mps,
        i.v_rel_frac,
        i.a_z_pred_mps2,
        i.a_z_headroom_mps2,
        i.e_sc_j,
        i.e_land_j,
        i.e_seat_j,
    ]
    .iter()
    .all(|x| x.is_finite());
    finite
        && !i.seated
        && i.s_min_m > 0.0
        && i.x_open_m >= i.s_min_m
        && i.v_cap_mps > 0.0
        && i.v_rel_frac > 0.0
        && i.v_rel_pred_mps <= i.v_rel_frac * i.v_cap_mps
        && i.stroke_fits
        && i.force_budget_ok
        && i.a_z_pred_mps2 <= i.a_z_headroom_mps2
        && i.mains_or_ups
        && i.e_land_j >= 0.0
        && i.e_seat_j >= 0.0
        && i.e_sc_j >= SMOOTH_HOLDUP_MARGIN * i.e_land_j + i.e_seat_j
}

/// The user motion in progress on `e`, in the gate's terms.
pub const fn user_motion_of(e: &DeviceEntry) -> UserMotion {
    match e.user_motion {
        Some(TravelDir::Open) => UserMotion::Opening,
        Some(TravelDir::Close) => UserMotion::Closing,
        None => UserMotion::None,
    }
}

/// The alert-gate action a decision `d` on device `e` amounts to, with `to` the rank
/// the decision aims for (plan §4b; EXOTIC §S2). Centring, isolation and damping are
/// pre-positioning (with or without a target) and `OpenAndStop` is an opening; any other
/// target away from the current position is a `Move` (closing when the target lies
/// below x; +x opens, PHYSICS §1); close-type actions without a target head for the
/// closed stop.
pub fn alert_action(e: &DeviceEntry, d: &ActionDecision, to: SafetyRank) -> AlertAction {
    let from = SafetyRank::of(e.condition, e.egress);
    match d.action {
        SeismicAction::Center | SeismicAction::Isolate | SeismicAction::Damp => {
            return AlertAction::PrePosition;
        }
        SeismicAction::OpenAndStop => return AlertAction::Open,
        _ => {}
    }
    if let Some(t) = d.target {
        let dx = t.diff_nm(e.x);
        if dx != 0 {
            return AlertAction::Move {
                from,
                to,
                closing: dx < 0,
            };
        }
    }
    match d.action {
        SeismicAction::None | SeismicAction::Freeze => AlertAction::Freeze,
        SeismicAction::Land => AlertAction::Land,
        SeismicAction::Lock => AlertAction::Lock,
        SeismicAction::RideThrough => AlertAction::SeismicPassive,
        SeismicAction::Center | SeismicAction::Isolate | SeismicAction::Damp => {
            AlertAction::PrePosition
        }
        SeismicAction::OpenAndStop => AlertAction::Open,
        SeismicAction::CompleteClose | SeismicAction::CloseAfterShaking => {
            if e.x.diff_nm(e.x_closed) > 0 {
                AlertAction::Move {
                    from,
                    to,
                    closing: true,
                }
            } else {
                AlertAction::Freeze
            }
        }
    }
}

/// True when the commanded speed of `d` is within device `e`'s unattended low-energy
/// limit v ≤ min(v_cap, 150 N/√(m_gov·k_series)) (plan §4b; `DeviceConstraints`).
pub fn within_unattended(e: &DeviceEntry, d: &ActionDecision) -> bool {
    let c = &e.constraints;
    let lim = if c.v_cap_mps > 0.0 {
        c.v_unattended_mps.min(c.v_cap_mps)
    } else {
        c.v_unattended_mps
    };
    lim.is_finite()
        && lim > 0.0
        && d.v_cap_mps.is_finite()
        && d.v_cap_mps > 0.0
        && d.v_cap_mps <= lim
}

/// Puts decision `d` for device `e` (aiming for rank `to`) to the exo-safety alert gate
/// with the confirmation carried by `alert` (plan §4b: consulted for every command).
/// `zone_clear_verified` is the device zone's `ClearVerified` state (sovereign classes;
/// the HAVEN plinth pre-centres only with it).
pub fn gate_decision(
    alert: &FusedAlert,
    e: &DeviceEntry,
    d: &ActionDecision,
    to: SafetyRank,
    zone_clear_verified: bool,
) -> GateVerdict {
    let c = Confirmations::from_alert(
        alert,
        zone_clear_verified,
        e.obstruction_armed,
        within_unattended(e, d),
    );
    gate_with_user(
        alert_action(e, d, to),
        &c,
        GateClass::of(e.class, e.egress),
        user_motion_of(e),
    )
}

/// A per-device `ActionPolicy` that proposes a pre-planned decision (from a
/// `HousePlan`); `None` keeps the core class action. The core `admit` filter applies.
///
/// **Device-level confirmation** (Wave 2b, E5). The house plan is confirmed by the
/// house fusion (a local P pick anywhere in the house, or ≥ 2 independent feeds), but
/// the device core's `admit` backstop confirms alert-initiated motion only with its own
/// report (`ActionContext::confirmed`: its local P pick, or `min_independent_feeds`
/// feeds, and a device sees one network link). A planned motion the device core cannot
/// yet confirm would be rejected (`Unconfirmed`) and the core class action (a DRAWER
/// lands on its rollers at the open position, H-20) would take over before the local
/// pick arrives. While the device is unconfirmed the policy therefore proposes FREEZE
/// (hold in place; admissible, it moves nothing) until `wait_until`, and after it the
/// core class action (`None`); the planned motion starts at the device's own
/// confirmation. `released` records the first tick the motion was passed on.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PlanPolicy {
    /// The planned decision for this device, if any.
    pub decision: Option<ActionDecision>,
    /// Latest tick to hold (FREEZE) a planned motion the device cannot yet confirm;
    /// `None`: no wait (the core class action stands while unconfirmed).
    pub wait_until: Option<Tick>,
    /// First tick a planned motion was passed to the device confirmed (additive, E5).
    pub released: Option<Tick>,
    /// First tick the policy held a planned motion for want of device confirmation.
    pub waited: Option<Tick>,
}

impl PlanPolicy {
    /// The policy proposing `decision` (`None`: the core class action stands).
    pub const fn new(decision: Option<ActionDecision>) -> PlanPolicy {
        PlanPolicy {
            decision,
            wait_until: None,
            released: None,
            waited: None,
        }
    }

    /// [`PlanPolicy::new`] that holds (FREEZE) an unconfirmed planned motion until
    /// `wait_until` (see the type docs).
    pub const fn with_wait(decision: Option<ActionDecision>, wait_until: Option<Tick>) -> PlanPolicy {
        PlanPolicy {
            decision,
            wait_until,
            released: None,
            waited: None,
        }
    }
}

impl ActionPolicy for PlanPolicy {
    fn propose(&mut self, ctx: &ActionContext) -> Option<ActionDecision> {
        let d = self.decision?;
        let moving = d.moves() && d.target.is_none_or(|t| t != ctx.x);
        if !moving {
            return Some(d);
        }
        if ctx.confirmed() {
            if self.released.is_none() {
                self.released = Some(ctx.tick);
            }
            return Some(d);
        }
        if self.waited.is_none() {
            self.waited = Some(ctx.tick);
        }
        match self.wait_until {
            Some(w) if ctx.tick < w => Some(ActionDecision::of(SeismicAction::Freeze)),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::choreo::plan::{
        BuildingPrediction, HousePlan, NotSecuredReason, PlanOptions, PlanStatus, plan_with, replan,
    };
    use crate::choreo::registry::{DeviceConstraints, DeviceRegistry};
    use fabrail_core::msg::SeismicReport;
    use fabrail_core::seismic::action_policy::{PolicyOutcome, admit};
    use fabrail_exo_safety::alert_gate::DenyReason;
    use fabrail_exo_safety::{AlertFeedKind, DeviceCondition, FeedSet};
    use fabrail_types::{
        AlertSource, NetworkAlert, PosNm, RestPolicy, SeismicLevel, SeismicState, Tick,
    };

    const S: u64 = 20_000;

    fn drawer(x_m: f64, cond: DeviceCondition) -> DeviceEntry {
        DeviceEntry {
            class: AnyClass::Core(RailClass::Drawer),
            egress: EgressRole::None,
            condition: cond,
            x: PosNm::from_m_f64(x_m),
            x_closed: PosNm(0),
            m_gov_kg: 48.0,
            obstruction_armed: true,
            constraints: DeviceConstraints {
                v_cap_mps: 0.2654,
                v_unattended_mps: 0.2305,
                a_brk_mps2: 3.0,
                p_move_w: 40.0,
                s_min_m: 0.05,
                latch_fitted: false,
                latch_retention_n: 0.0,
            },
            ..DeviceEntry::EMPTY
        }
    }

    /// Alerts of plan §4 by provenance.
    fn alert(kind: &str) -> FusedAlert {
        let base = FusedAlert {
            alert: NetworkAlert {
                id: 7,
                received: Tick(150_000),
                magnitude: 7.2,
                r_epi_km: 60.0,
                depth_km: 12.0,
                ..NetworkAlert::default()
            },
            level: SeismicLevel::L2,
            p_damaging: 0.9,
            s_eta: Some(Tick(350_000)),
            s_eta_sigma_s: 1.0,
            ..FusedAlert::default()
        };
        let net = FeedSet::EMPTY.with(AlertFeedKind::Network);
        match kind {
            "network_only" => FusedAlert {
                feeds: net,
                n_independent: 1,
                ..base
            },
            "local_p" => FusedAlert {
                feeds: net.with(AlertFeedKind::LocalP),
                n_independent: 2,
                local_p: true,
                ..base
            },
            "two_feeds" => FusedAlert {
                feeds: net.with(AlertFeedKind::Das),
                n_independent: 2,
                ..base
            },
            "injected" => FusedAlert {
                feeds: net.with(AlertFeedKind::OffshoreCabled),
                n_independent: 2,
                injected: true,
                ..base
            },
            "pegs_only" => FusedAlert {
                feeds: FeedSet::EMPTY.with(AlertFeedKind::PegsMagnitudeAdvisory),
                n_independent: 1,
                ..base
            },
            _ => base,
        }
    }

    fn pred(now: Tick) -> BuildingPrediction {
        BuildingPrediction {
            now,
            p_bus_max_w: 1000.0,
            ..BuildingPrediction::default()
        }
    }

    /// The core-side context a device would present for decision `d` under `a`.
    fn core_ctx(e: &DeviceEntry, a: &FusedAlert) -> ActionContext {
        let class = match e.class {
            AnyClass::Core(c) => c,
            AnyClass::Exo(_) => RailClass::Heavy,
        };
        ActionContext {
            tick: a.alert.received,
            class,
            rest_policy: RestPolicy::Seat,
            egress: e.egress,
            seis: SeismicState::Protect,
            x: e.x,
            v_mps: e.v_mps,
            x_closed: e.x_closed,
            user_motion: e.user_motion,
            report: SeismicReport {
                source: if a.non_local() {
                    AlertSource::Network
                } else {
                    AlertSource::Local
                },
                local_p: Confirmations::from_alert(a, false, true, true).local_p,
                n_feeds: Confirmations::from_alert(a, false, true, true).n_independent_feeds,
                ..SeismicReport::default()
            },
            default: ActionDecision::of(SeismicAction::Freeze),
            v_unattended_mps: e.constraints.v_unattended_mps,
            min_independent_feeds: 2,
            ..ActionContext::default()
        }
    }

    // req: FAB-EXA-006
    #[test]
    fn alert_motion_rule() {
        // (1) The gate never admits opening, unlocking, de-isolating or speeding up on
        // an alert, whatever confirms it (plan §4b; FAB-EXO-506).
        let e = drawer(0.30, DeviceCondition::FrozenOpen);
        let strong = Confirmations {
            local_p: true,
            n_independent_feeds: 3,
            zone_clear_verified: true,
            obstruction_armed: true,
            within_low_energy_caps: true,
        };
        for a in [
            AlertAction::Open,
            AlertAction::Unlock,
            AlertAction::DeIsolate,
            AlertAction::SpeedUp,
        ] {
            let v = gate_with_user(
                a,
                &strong,
                GateClass::of(e.class, e.egress),
                UserMotion::None,
            );
            assert_eq!(
                v,
                GateVerdict::Denied(DenyReason::ForbiddenOnAlert),
                "{a:?}"
            );
        }
        let open_dec = ActionDecision {
            action: SeismicAction::Land,
            target: Some(PosNm::from_m_f64(0.35)),
            v_cap_mps: 0.1,
            deadline: None,
        };
        assert_eq!(
            alert_action(&e, &open_dec, SafetyRank::SeatedOrLocked),
            AlertAction::Move {
                from: SafetyRank::FrozenOpen,
                to: SafetyRank::SeatedOrLocked,
                closing: false
            },
            "a target on the +x side is an opening"
        );
        assert!(
            !gate_decision(
                &alert("local_p"),
                &e,
                &open_dec,
                SafetyRank::SeatedOrLocked,
                true
            )
            .is_allowed()
        );

        // (2) Exhaustive over provenance × condition × user motion × egress × policy:
        // every proposal the plan makes passes the gate AND the core admit filter, never
        // targets the opening side, never exceeds the unattended limit, never locks an
        // egress opening, and moves only with independent confirmation.
        let conds = [
            DeviceCondition::Moving,
            DeviceCondition::FrozenOpen,
            DeviceCondition::Standoff,
            DeviceCondition::Hold,
            DeviceCondition::ClosedLevitated,
            DeviceCondition::Landed,
            DeviceCondition::Seated,
        ];
        let users = [None, Some(TravelDir::Open), Some(TravelDir::Close)];
        let kinds = [
            "network_only",
            "local_p",
            "two_feeds",
            "injected",
            "pegs_only",
        ];
        let classes = [
            (AnyClass::Core(RailClass::Drawer), EgressRole::None),
            (AnyClass::Core(RailClass::Door), EgressRole::NonRated),
            (AnyClass::Core(RailClass::Door), EgressRole::FireRated),
            (AnyClass::Core(RailClass::Wall), EgressRole::None),
            (AnyClass::Core(RailClass::Wall), EgressRole::NonRated),
            (AnyClass::Core(RailClass::Heavy), EgressRole::None),
            (AnyClass::Exo(ExoClass::HavenR), EgressRole::None),
        ];
        let mut moved = 0u32;
        for kind in kinds {
            let a = alert(kind);
            let independent = Confirmations::from_alert(&a, false, true, true).independent();
            for &(class, egress) in &classes {
                for &cond in &conds {
                    for &user in &users {
                        for pol in ChoreoPolicy::ALL {
                            for zone in [false, true] {
                                let mut e = drawer(0.30, cond);
                                e.class = class;
                                e.egress = egress;
                                e.user_motion = user;
                                e.policy = pol;
                                if cond == DeviceCondition::Seated {
                                    e.x = PosNm(0);
                                }
                                let mut reg = DeviceRegistry::new();
                                assert!(reg.push(e).is_ok());
                                let mut opts = PlanOptions::default();
                                opts.zone_clear_verified[0] = zone;
                                let now = a.alert.received;
                                let hp = plan_with(&a, &reg, &pred(now), &opts);
                                let act = hp.as_slice()[0];
                                assert!(act.policy.allowed(class, egress), "{act:?}");
                                assert_ne!(act.policy, ChoreoPolicy::Auto);
                                let Some(d) = act.proposal(Tick(u64::MAX)) else {
                                    continue;
                                };
                                let what = format!(
                                    "{kind} {class:?} {egress:?} {cond:?} {user:?} {pol:?} zone {zone}: {act:?}"
                                );
                                assert!(
                                    gate_decision(&a, &e, &d, act.target_rank, zone).is_allowed(),
                                    "gate refuses a planned proposal: {what}"
                                );
                                let (applied, outcome) = admit(&core_ctx(&e, &a), Some(d));
                                assert!(
                                    !matches!(outcome, PolicyOutcome::Rejected(_)),
                                    "core admit rejects a planned proposal ({outcome:?}): {what}"
                                );
                                assert!(
                                    !(d.action == SeismicAction::Lock
                                        && egress != EgressRole::None),
                                    "egress never locks: {what}"
                                );
                                assert_ne!(d.action, SeismicAction::OpenAndStop, "{what}");
                                if let Some(t) = d.target {
                                    assert!(t.diff_nm(e.x) <= 0, "never toward +x: {what}");
                                    if t != e.x {
                                        moved += 1;
                                        assert!(independent, "motion needs confirmation: {what}");
                                        assert!(
                                            d.v_cap_mps <= e.constraints.v_unattended_mps,
                                            "unattended limit: {what}"
                                        );
                                        assert_ne!(
                                            user,
                                            Some(TravelDir::Open),
                                            "a user opening is never reversed: {what}"
                                        );
                                        assert!(applied.v_cap_mps <= 0.2305 + 1e-6);
                                        if class == AnyClass::Exo(ExoClass::HavenR) {
                                            assert!(
                                                zone,
                                                "plinth pre-centres only ClearVerified: {what}"
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        assert!(moved > 0, "confirmed alerts do close open drawers");

        // (3) Concrete cases: a network-only or injected alert never moves an open
        // drawer (NotSecured by the gate, core class action stands); a local-P-confirmed
        // one closes it at the unattended limit toward the seat.
        let open = drawer(0.30, DeviceCondition::FrozenOpen);
        let mut reg = DeviceRegistry::new();
        assert!(reg.push(open).is_ok());
        for kind in ["network_only", "injected", "pegs_only"] {
            let a = alert(kind);
            let hp = plan_with(&a, &reg, &pred(a.alert.received), &PlanOptions::default());
            let act = hp.as_slice()[0];
            assert_eq!(act.status, PlanStatus::NotSecured, "{kind}");
            assert_eq!(act.reason, Some(NotSecuredReason::Gate), "{kind}");
            assert_eq!(
                act.proposal(Tick(u64::MAX)),
                None,
                "{kind}: core action stands"
            );
        }
        let a = alert("local_p");
        let hp = plan_with(&a, &reg, &pred(a.alert.received), &PlanOptions::default());
        let act = hp.as_slice()[0];
        assert_eq!(act.status, PlanStatus::Planned);
        assert_eq!(act.policy, ChoreoPolicy::SeatAndHold);
        assert_eq!(act.target_rank, SafetyRank::SeatedOrLocked);
        let d = act.proposal(act.start).expect("planned close");
        assert_eq!(d.target, Some(PosNm(0)));
        assert!(
            (d.v_cap_mps - 0.2305).abs() < 1e-6,
            "0.23 m/s at 48 kg (plan §4b)"
        );

        // (4) A user opening glide is never reversed: the gate refuses the close and the
        // core FREEZE stands (SEI-009); a user closing glide continues to the seat.
        let mut glide = drawer(0.15, DeviceCondition::Moving);
        glide.user_motion = Some(TravelDir::Open);
        let mut r2 = DeviceRegistry::new();
        assert!(r2.push(glide).is_ok());
        let hp = plan_with(&a, &r2, &pred(a.alert.received), &PlanOptions::default());
        assert_eq!(hp.as_slice()[0].reason, Some(NotSecuredReason::Gate));
        glide.user_motion = Some(TravelDir::Close);
        let mut r3 = DeviceRegistry::new();
        assert!(r3.push(glide).is_ok());
        let hp = plan_with(&a, &r3, &pred(a.alert.received), &PlanOptions::default());
        assert_eq!(hp.as_slice()[0].status, PlanStatus::Planned);

        // (5) One attempt only: after an obstruction the device is NotSecured, proposes a
        // freeze (no re-close) and a re-plan keeps it that way.
        let mut hp = plan_with(&a, &reg, &pred(a.alert.received), &PlanOptions::default());
        hp.mark_not_secured(0, NotSecuredReason::Obstruction);
        let d = hp.as_slice()[0].proposal(Tick(u64::MAX)).expect("freeze");
        assert_eq!(d, ActionDecision::of(SeismicAction::Freeze));
        let later = pred(Tick(a.alert.received.0 + S));
        let again: HousePlan = replan(&hp, &a, &reg, &later, &PlanOptions::default());
        assert_eq!(
            again.as_slice()[0].reason,
            Some(NotSecuredReason::Obstruction)
        );
        assert_eq!(
            again.as_slice()[0].proposal(Tick(u64::MAX)),
            Some(ActionDecision::of(SeismicAction::Freeze))
        );

        // (6) Egress opening: StopAndLand, never Lock, even if configured to lock.
        let mut door = drawer(0.4, DeviceCondition::Hold);
        door.class = AnyClass::Core(RailClass::Door);
        door.egress = EgressRole::NonRated;
        door.policy = ChoreoPolicy::LandAndLock;
        let mut r4 = DeviceRegistry::new();
        assert!(r4.push(door).is_ok());
        let hp = plan_with(&a, &r4, &pred(a.alert.received), &PlanOptions::default());
        let act = hp.as_slice()[0];
        assert_eq!(act.policy, ChoreoPolicy::StopAndLand);
        assert_eq!(act.decision, ActionDecision::of(SeismicAction::Land));
        assert_eq!(
            act.target_rank,
            SafetyRank::SeatedOrLocked,
            "egress landed is top"
        );

        // (7) HAVEN plinth: pre-centring only while the moat zone is ClearVerified.
        // 20 mm off centre: the pre-centring finishes before the deadline (τ 1 s).
        let mut plinth = drawer(0.02, DeviceCondition::Hold);
        plinth.class = AnyClass::Exo(ExoClass::HavenR);
        plinth.obstruction_armed = false;
        let mut r5 = DeviceRegistry::new();
        assert!(r5.push(plinth).is_ok());
        let hp = plan_with(&a, &r5, &pred(a.alert.received), &PlanOptions::default());
        assert_eq!(
            hp.as_slice()[0].decision,
            ActionDecision::of(SeismicAction::RideThrough),
            "no pre-centring without ClearVerified"
        );
        let mut opts = PlanOptions::default();
        opts.zone_clear_verified[0] = true;
        let hp = plan_with(&a, &r5, &pred(a.alert.received), &opts);
        assert_eq!(hp.as_slice()[0].decision.action, SeismicAction::Center);
    }

    #[test]
    fn smooth_admission_list() {
        let ok = SmoothAdmissionInputs {
            seated: false,
            x_open_m: 0.10,
            s_min_m: 0.04,
            v_rel_pred_mps: 0.15,
            v_cap_mps: 0.2654,
            v_rel_frac: 0.8,
            stroke_fits: true,
            force_budget_ok: true,
            a_z_pred_mps2: 0.5,
            a_z_headroom_mps2: 1.0,
            mains_or_ups: true,
            e_sc_j: 400.0,
            e_land_j: 100.0,
            e_seat_j: 50.0,
        };
        assert!(admit_smooth_ride_through(&ok));
        type Edit = fn(&mut SmoothAdmissionInputs);
        let refusals: [(&str, Edit); 10] = [
            ("seated", |i| i.seated = true),
            ("not open to s_min", |i| i.x_open_m = 0.03),
            ("v_rel > 0.8 v_cap", |i| i.v_rel_pred_mps = 0.22),
            ("stroke", |i| i.stroke_fits = false),
            ("force budget", |i| i.force_budget_ok = false),
            ("z headroom", |i| i.a_z_pred_mps2 = 1.1),
            ("mains", |i| i.mains_or_ups = false),
            ("hold-up 3·E_land + E_seat", |i| i.e_sc_j = 349.0),
            ("non-finite", |i| i.v_rel_pred_mps = f32::NAN),
            ("default frac admits nothing", |i| i.v_rel_frac = 0.0),
        ];
        for (name, edit) in refusals {
            let mut i = ok;
            edit(&mut i);
            assert!(!admit_smooth_ride_through(&i), "{name}");
        }
        assert!(!admit_smooth_ride_through(&SmoothAdmissionInputs::default()));
    }

    #[test]
    fn latch_rule_and_auto_resolution() {
        let mut e = drawer(0.0, DeviceCondition::Seated);
        assert_eq!(resolve_auto(&e), ChoreoPolicy::SeatAndHold);
        e.constraints.latch_fitted = true;
        e.constraints.latch_retention_n = 100.0;
        assert_eq!(resolve_auto(&e), ChoreoPolicy::SeatAndLatch);
        // 48 kg × 2.0 m/s² = 96 N ≤ 100 N holds; 2.2 m/s² does not.
        assert!(latch_holds(&e, 2.0));
        assert!(!latch_holds(&e, 2.2));
        e.constraints.latch_fitted = false;
        assert!(!latch_holds(&e, 0.0));
        for p in ChoreoPolicy::ALL {
            for eg in [EgressRole::NonRated, EgressRole::FireRated] {
                if p.locks() {
                    assert!(!p.allowed(AnyClass::Core(RailClass::Door), eg), "{p:?}");
                }
            }
        }
    }
}
