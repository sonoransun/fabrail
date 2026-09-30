//! Plain-data exo types: ExoClass, SafetyRegime, SafeStatePolicy, FusedAlert, AlertFeedKind and the safe-state additions. Owner: W0a (complete in Wave 0, W0-C1).
//!
//! Everything here is plain data (EXOTIC §1, §S1, §S2; plan §4):
//!
//! * [`ExoClass`]: the nine sovereign classes. The core [`RailClass`] and its pedestrian
//!   caps are never extended or changed (FAB-EXO-006); [`AnyClass`] names either.
//! * [`SafetyRegime`]: safe by separation, not by energy limit. Exotic classes leave the
//!   pedestrian regime for `GuardedZone` or `OccupiedPlatform`; [`ClassRegime::new`] is
//!   the only way to pair a class with a regime and refuses `GuardedZone` on DRAWER,
//!   DOOR or WALL and `Pedestrian` on any exotic class.
//! * [`SafeStatePolicy`] maps each class's safe state onto the core [`SeismicAction`]
//!   (including RideThrough, CompleteClose and CloseAfterShaking); land-first is not
//!   universal.
//! * [`SuspensionKind`] and [`PoleMaterial`] describe the suspension as data (FeCo is a
//!   pole-material parameter of hybrid EMS, not a kind). No research-only or frontier
//!   variant exists in Rust (FAB-EXO-210).
//! * [`AlertFeedKind`], [`FeedSet`] and [`FusedAlert`]: the fused early-warning output.
//!   `AlertFeedKind` lives here (not in fabrail-exo's fusion module) because
//!   `FusedAlert`, which the alert gate reads, names it and this crate may not depend on
//!   fabrail-exo (FAB-EXO-003).
//!
//! Every alert in this tier is synthetic or injected: fabrail is not an EEW provider.

use fabrail_types::{
    EgressRole, NetworkAlert, RailClass, RestPolicy, SeismicAction, SeismicLevel, Tick,
};

/// Defines a `#[repr(u8)]` plain-data enum with explicit discriminants plus `ALL`,
/// `as_u8`, `from_u8` and `name` (the same shape as fabrail-types' wire enums).
macro_rules! u8_enum {
    (
        $(#[$meta:meta])*
        pub enum $name:ident {
            $( $(#[$vmeta:meta])* $var:ident = $val:expr ),+ $(,)?
        }
    ) => {
        $(#[$meta])*
        #[repr(u8)]
        pub enum $name {
            $( $(#[$vmeta])* $var = $val ),+
        }

        impl $name {
            /// Every variant, in discriminant order.
            pub const ALL: &'static [$name] = &[$($name::$var),+];

            /// The wire value (telemetry, replay, PyO3).
            pub const fn as_u8(self) -> u8 {
                self as u8
            }

            /// Decodes a wire value; `None` for an unknown discriminant.
            pub const fn from_u8(v: u8) -> Option<Self> {
                match v {
                    $( x if x == $val => Some($name::$var), )+
                    _ => None,
                }
            }

            /// The variant name.
            pub const fn name(self) -> &'static str {
                match self {
                    $( $name::$var => stringify!($var), )+
                }
            }
        }
    };
}
pub(crate) use u8_enum;

u8_enum! {
    /// Sovereign/exotic product class (EXOTIC §1; plan §4 class table). Plain data: the
    /// class presets live in [`crate::presets`].
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum ExoClass {
        /// Tower hybrid mass damper, 1,000 t (research tier).
        #[default]
        Sentinel = 0,
        /// Occupied isolation platform (racks, artifacts), 50 t, T 6 s.
        HavenR = 1,
        /// Facility isolation platform, 5 kt (extrapolated).
        HavenF = 2,
        /// Radiation plug / vault door with a passive mover, 50 t.
        Shield = 3,
        /// Blast and vault plug door, 50 t, closure race.
        Bastion = 4,
        /// Field trays, roof petals, stage wagons: 8,600 t in pods.
        Arena = 5,
        /// Telescope domes, rotating floors: 6,100 t ring.
        Rotator = 6,
        /// Flood / tsunami gate, 150 t.
        Tidegate = 7,
        /// Lunar hatch in a permanently shadowed region (frontier tier; numpy only).
        Selene = 8,
    }
}

impl ExoClass {
    /// Lower-case identifier used by Python, JSON and the CLI (`"haven_r"`, …).
    pub const fn slug(self) -> &'static str {
        match self {
            ExoClass::Sentinel => "sentinel",
            ExoClass::HavenR => "haven_r",
            ExoClass::HavenF => "haven_f",
            ExoClass::Shield => "shield",
            ExoClass::Bastion => "bastion",
            ExoClass::Arena => "arena",
            ExoClass::Rotator => "rotator",
            ExoClass::Tidegate => "tidegate",
            ExoClass::Selene => "selene",
        }
    }

    /// Display name as in the EXOTIC §1 table (`"HAVEN-R"`, …).
    pub const fn display(self) -> &'static str {
        match self {
            ExoClass::Sentinel => "SENTINEL",
            ExoClass::HavenR => "HAVEN-R",
            ExoClass::HavenF => "HAVEN-F",
            ExoClass::Shield => "SHIELD",
            ExoClass::Bastion => "BASTION",
            ExoClass::Arena => "ARENA",
            ExoClass::Rotator => "ROTATOR",
            ExoClass::Tidegate => "TIDEGATE",
            ExoClass::Selene => "SELENE",
        }
    }

    /// Parses a [`slug`](Self::slug) (exact, lower case).
    pub fn from_slug(s: &str) -> Option<ExoClass> {
        ExoClass::ALL.iter().copied().find(|c| c.slug() == s)
    }
}

/// Either a core rail class or a sovereign class.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum AnyClass {
    /// A core (pedestrian-regime or infrastructure) class.
    Core(RailClass),
    /// A sovereign/exotic class.
    Exo(ExoClass),
}

u8_enum! {
    /// Capability badge: the core status (✅ V, 🔶 S, 🧪 R, 🗺️ M), replaced by 🔭 when the
    /// technology's maturity is frontier (EXOTIC §0.1). Technology maturity (TRL) is a
    /// separate column ([`Trl`]), never merged into the badge.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum Badge {
        /// ✅ verified in simulation.
        Verified = 0,
        /// 🔶 simplified model.
        Simplified = 1,
        /// 🧪 research tier.
        #[default]
        Research = 2,
        /// 🗺️ roadmap (not built).
        Roadmap = 3,
        /// 🔭 frontier: theoretical, never a product.
        Frontier = 4,
    }
}

impl Badge {
    /// The rendered badge symbol.
    pub const fn symbol(self) -> &'static str {
        match self {
            Badge::Verified => "✅",
            Badge::Simplified => "🔶",
            Badge::Research => "🧪",
            Badge::Roadmap => "🗺️",
            Badge::Frontier => "🔭",
        }
    }
}

/// Technology readiness level range (NASA/EU TRL 1–9); the maturity column.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Trl {
    /// Lowest TRL of the range.
    pub lo: u8,
    /// Highest TRL of the range.
    pub hi: u8,
}

impl Trl {
    /// A TRL range `lo..=hi`.
    pub const fn new(lo: u8, hi: u8) -> Trl {
        Trl { lo, hi }
    }
}

u8_enum! {
    /// Suspension physics kind (EXOTIC §X2, §X3; FAB-EXO-201). FeCo is a pole-material
    /// parameter of `HybridEms` ([`PoleMaterial`]), not a kind. Frontier variants never
    /// appear in Rust (FAB-EXO-210).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum SuspensionKind {
        /// PM-biased hybrid EMS under zero-power control (the core suspension).
        #[default]
        HybridEms = 0,
        /// Pure electromagnet EMS (Transrapid-style reference, 1.7 kW/t hover).
        PureEms = 1,
        /// Bulk HTS flux pinning over a PM guideway (research tier).
        HtsBulk = 2,
        /// HTS lift plus an EMS trim coil (research tier).
        HybridHtsPlusEms = 3,
        /// Non-magnetic air-pad baseline (comparison only).
        AirPad = 4,
    }
}

u8_enum! {
    /// Pole material of an EMS element (EXOTIC §X2: saturation 1.5 T for Fe, 2.3 T FeCo).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum PoleMaterial {
        /// Electrical steel / soft iron.
        #[default]
        Fe = 0,
        /// Iron-cobalt (Hiperco-class) poles.
        FeCo = 1,
        /// No iron poles (air pad; passive HTS guideway).
        None = 2,
    }
}

u8_enum! {
    /// Per-class safe state (EXOTIC §S1; plan §4 "land-first is not universal").
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum SafeStatePolicy {
        /// Land and engage the lock (SHIELD within 2 s, ARENA, SELENE).
        #[default]
        LandAndLock = 0,
        /// Isolate while levitated, then land on the FPS catcher (HAVEN).
        IsolateThenLandFps = 1,
        /// Ride through levitated in SEISMIC_PASSIVE (SENTINEL).
        RideThrough = 2,
        /// Complete an in-progress close, else land (BASTION).
        CompleteClose = 3,
        /// Brake and land on the pier (ROTATOR).
        BrakeAndLand = 4,
        /// Close only after the shaking ends, on the external decision-system command
        /// (TIDEGATE).
        CloseAfterShaking = 5,
    }
}

impl SafeStatePolicy {
    /// The core seismic action this policy maps onto (through the core `ActionPolicy`
    /// hook; FAB-EXO-102).
    pub const fn seismic_action(self) -> SeismicAction {
        match self {
            SafeStatePolicy::LandAndLock => SeismicAction::Lock,
            SafeStatePolicy::IsolateThenLandFps => SeismicAction::Isolate,
            SafeStatePolicy::RideThrough => SeismicAction::RideThrough,
            SafeStatePolicy::CompleteClose => SeismicAction::CompleteClose,
            SafeStatePolicy::BrakeAndLand => SeismicAction::Land,
            SafeStatePolicy::CloseAfterShaking => SeismicAction::CloseAfterShaking,
        }
    }
}

u8_enum! {
    /// What a class does in an earthquake (exotic design §3.4 action map).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum SeismicRole {
        /// Mass damper: SEISMIC_PASSIVE ride-through; mainly a wind device (at most about
        /// 0.7 % added damping at design-level shaking).
        #[default]
        DamperRideThrough = 0,
        /// Primary isolation of an occupied platform (research tier).
        PrimaryIsolation = 1,
        /// Stop, land and secure (lock, skids, pier).
        LandSecure = 2,
        /// Closure race against a known arrival time (t_a is an input).
        ClosureRace = 3,
        /// Freeze and land during shaking; close afterwards on external command.
        PostShakingClosure = 4,
    }
}

/// Guarded-zone configuration (EXOTIC §X14, §S1; ISO 13855 / 13849-1 / 14119 used for
/// numeric limits only, never as a compliance claim). Integer units.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ZoneCfg {
    /// Presence sensing designed toward PL e (else PL d), ISO 13849-1 target only.
    pub presence_pl_e: bool,
    /// Interlocked with a facility personnel protection system (SHIELD).
    pub pps: bool,
    /// A two-person authority override may let closure win over presence sensing
    /// (BASTION, TIDEGATE; FAB-EXO-507).
    pub two_person_override: bool,
    /// Operated remotely (SELENE).
    pub teleoperated: bool,
    /// ISO 13855 approach speed K, mm/s (1600 mm/s).
    pub k_mmps: u32,
    /// ISO 13855 detection + response time t1, ms (100 ms, exotic design §1.3).
    pub t1_ms: u32,
    /// ISO 13855 intrusion allowance C, mm (850 mm, exotic design §1.3).
    pub c_mm: u32,
    /// Braking deceleration used for t2 = v/a_brake, mm/s².
    pub a_brake_mmps2: u32,
    /// Attended-jog speed limit, mm/s (10 mm/s; FAB-EXO-503).
    pub jog_v_mmps: u32,
    /// Compliant closing-edge stiffness limit, N/mm (≤ 10 N/mm; imported from
    /// `fabrail_params::presets::EDGE_K_MAX_N_PER_M`).
    pub edge_k_n_per_mm: u32,
}

/// Occupied-platform configuration (HAVEN; EXOTIC §X9, FAB-EXO-505). The limits are
/// analysis outputs and design targets, not bounds proven for every record.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OccupantCfg {
    /// Peak platform acceleration target at DBE, mm/s² (0.1 g).
    pub a_peak_max_mmps2: u32,
    /// Jerk limit in controlled moves, mm/s³ (1 m/s³).
    pub jerk_max_mmps3: u32,
    /// Landing speed limit through the PM-bias ramp, mm/s (0.1 m/s).
    pub landing_v_max_mmps: u32,
    /// The moat band is under presence monitoring (gates controlled moves only).
    pub moat_presence: bool,
    /// Passive moat covers plus an exclusion band are fitted.
    pub moat_covers: bool,
}

/// How a class is kept safe (EXOTIC §0.2, §S1).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
pub enum SafetyRegime {
    /// Core pedestrian caps (energy, momentum, clamp), imported from
    /// `fabrail_params::presets`. Core classes only.
    #[default]
    Pedestrian,
    /// Guarded zone: presence sensing and interlocks, ISO 13855 separation.
    GuardedZone(ZoneCfg),
    /// Occupied platform: occupant comfort limits (HAVEN).
    OccupiedPlatform(OccupantCfg),
}

impl SafetyRegime {
    /// Short label as used in the EXOTIC §1 table (`"GZ"`, `"OP"`, `"Pedestrian"`).
    pub const fn label(&self) -> &'static str {
        match self {
            SafetyRegime::Pedestrian => "Pedestrian",
            SafetyRegime::GuardedZone(_) => "GZ",
            SafetyRegime::OccupiedPlatform(_) => "OP",
        }
    }
}

/// Why a class/regime pairing was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RegimeError {
    /// `GuardedZone` or `OccupiedPlatform` on DRAWER, DOOR or WALL (FAB-EXO-006).
    NotForPedestrianClass,
    /// `Pedestrian` on an exotic class: exotic classes are safe by separation.
    ExoticNeedsSeparation,
    /// `OccupiedPlatform` on a class that is not an isolation platform.
    NotAPlatform,
}

/// A class paired with an admissible regime. [`ClassRegime::new`] is the only
/// constructor, so an inadmissible pairing cannot exist (FAB-EXO-006).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ClassRegime {
    class: AnyClass,
    regime: SafetyRegime,
}

impl ClassRegime {
    /// Pairs `class` with `regime`:
    /// * DRAWER, DOOR, WALL: `Pedestrian` only;
    /// * HEAVY: `Pedestrian`, or `GuardedZone` (automatic motion above the unsensed caps
    ///   needs presence sensing);
    /// * exotic classes: never `Pedestrian`; `OccupiedPlatform` only on HAVEN-R/HAVEN-F.
    pub const fn new(class: AnyClass, regime: SafetyRegime) -> Result<ClassRegime, RegimeError> {
        let ok = match (class, &regime) {
            (AnyClass::Core(RailClass::Heavy), SafetyRegime::OccupiedPlatform(_)) => {
                Err(RegimeError::NotAPlatform)
            }
            (AnyClass::Core(RailClass::Heavy), _) => Ok(()),
            (AnyClass::Core(_), SafetyRegime::Pedestrian) => Ok(()),
            (AnyClass::Core(_), _) => Err(RegimeError::NotForPedestrianClass),
            (AnyClass::Exo(_), SafetyRegime::Pedestrian) => Err(RegimeError::ExoticNeedsSeparation),
            (
                AnyClass::Exo(ExoClass::HavenR | ExoClass::HavenF),
                SafetyRegime::OccupiedPlatform(_),
            ) => Ok(()),
            (AnyClass::Exo(_), SafetyRegime::OccupiedPlatform(_)) => Err(RegimeError::NotAPlatform),
            (AnyClass::Exo(_), SafetyRegime::GuardedZone(_)) => Ok(()),
        };
        match ok {
            Ok(()) => Ok(ClassRegime { class, regime }),
            Err(e) => Err(e),
        }
    }

    /// The class.
    pub const fn class(&self) -> AnyClass {
        self.class
    }

    /// The regime.
    pub const fn regime(&self) -> SafetyRegime {
        self.regime
    }
}

u8_enum! {
    /// A source of early-warning information (FAB-EXO-801). The fault-injection
    /// `Injector` (fabrail-exo, feature `fault-injection` only) emits `Network` feeds
    /// flagged as injected; it is not a kind.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum AlertFeedKind {
        /// On-site P-wave pick (core structure ADXL355).
        #[default]
        LocalP = 0,
        /// Network message (ShakeAlert/JMA style; synthetic in fabrail).
        Network = 1,
        /// Offshore cabled observatory (S-net-like).
        OffshoreCabled = 2,
        /// Distributed acoustic sensing.
        Das = 3,
        /// PEGS magnitude advisory (research tier). Never the sole trigger (FAB-EXO-810).
        PegsMagnitudeAdvisory = 4,
    }
}

/// A set of [`AlertFeedKind`]s (bit `k` = kind with discriminant `k`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct FeedSet(pub u8);

impl FeedSet {
    /// The empty set.
    pub const EMPTY: FeedSet = FeedSet(0);

    /// Adds `k`.
    pub const fn with(self, k: AlertFeedKind) -> FeedSet {
        FeedSet(self.0 | (1u8 << k.as_u8()))
    }

    /// True when `k` is in the set.
    pub const fn contains(self, k: AlertFeedKind) -> bool {
        self.0 & (1u8 << k.as_u8()) != 0
    }

    /// Number of kinds in the set.
    pub const fn count(self) -> u32 {
        self.0.count_ones()
    }

    /// True when only the PEGS advisory contributed (never a sufficient trigger).
    pub const fn pegs_only(self) -> bool {
        self.0 == 1u8 << AlertFeedKind::PegsMagnitudeAdvisory.as_u8()
    }
}

/// The fused early-warning output (exotic design §3.4; FAB-EXO-801, 802): a synthetic
/// core [`NetworkAlert`] plus the exotic extension. It enters the core through the
/// `ActionPolicy` hook, never as a direct actuator command.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FusedAlert {
    /// Synthetic core alert (id, received, origin offset, magnitude, distance, depth,
    /// MMI, cancel).
    pub alert: NetworkAlert,
    /// Fused level (0–3).
    pub level: SeismicLevel,
    /// P(I ≥ I_crit) from the maximum predicted intensity and capped log-odds, 0..1.
    pub p_damaging: f32,
    /// Mean predicted S-wave arrival at this site, if known.
    pub s_eta: Option<Tick>,
    /// Standard deviation of the S-wave arrival, s.
    pub s_eta_sigma_s: f32,
    /// Long-period (large-magnitude / far-field) flag.
    pub long_period: bool,
    /// Predicted tsunami arrival, if any (TIDEGATE).
    pub tsunami_eta: Option<Tick>,
    /// Feed kinds that contributed.
    pub feeds: FeedSet,
    /// Independent feeds contributing (a local P pick counts as one; injected feeds
    /// never count).
    pub n_independent: u8,
    /// A local P pick contributed.
    pub local_p: bool,
    /// Any contribution came from the fault-injection `Injector`.
    pub injected: bool,
}

impl FusedAlert {
    /// True when a non-local source (network, offshore, DAS, PEGS or an injection)
    /// contributed.
    pub const fn non_local(&self) -> bool {
        self.injected || self.feeds.0 & !(1u8 << AlertFeedKind::LocalP.as_u8()) != 0
    }
}

/// Rest behaviour helper: the core rest policy a class uses when idle.
pub const fn rest_is_levitated(r: RestPolicy) -> bool {
    matches!(r, RestPolicy::Hover)
}

/// True when the opening is on an egress path (never locks).
pub const fn is_egress(e: EgressRole) -> bool {
    !matches!(e, EgressRole::None)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gz() -> SafetyRegime {
        SafetyRegime::GuardedZone(ZoneCfg::default())
    }

    fn op() -> SafetyRegime {
        SafetyRegime::OccupiedPlatform(OccupantCfg::default())
    }

    // req: FAB-EXO-006
    #[test]
    fn guarded_zone_not_for_pedestrian_classes() {
        for c in [RailClass::Drawer, RailClass::Door, RailClass::Wall] {
            assert_eq!(
                ClassRegime::new(AnyClass::Core(c), gz()),
                Err(RegimeError::NotForPedestrianClass),
                "{c:?}: GuardedZone is not assignable (FAB-EXO-006)"
            );
            assert_eq!(
                ClassRegime::new(AnyClass::Core(c), op()),
                Err(RegimeError::NotForPedestrianClass),
                "{c:?}: OccupiedPlatform is not assignable"
            );
            let p = ClassRegime::new(AnyClass::Core(c), SafetyRegime::Pedestrian);
            assert!(p.is_ok(), "{c:?}: the pedestrian regime stays");
        }
        // HEAVY may run guarded (PL d presence sensing above the unsensed caps).
        assert!(ClassRegime::new(AnyClass::Core(RailClass::Heavy), gz()).is_ok());
        assert!(ClassRegime::new(AnyClass::Core(RailClass::Heavy), op()).is_err());
        // Exotic classes are safe by separation, never by the pedestrian caps.
        for &x in ExoClass::ALL {
            assert_eq!(
                ClassRegime::new(AnyClass::Exo(x), SafetyRegime::Pedestrian),
                Err(RegimeError::ExoticNeedsSeparation),
                "{x:?}"
            );
            assert!(ClassRegime::new(AnyClass::Exo(x), gz()).is_ok(), "{x:?}");
        }
        assert!(ClassRegime::new(AnyClass::Exo(ExoClass::HavenR), op()).is_ok());
        assert!(ClassRegime::new(AnyClass::Exo(ExoClass::Shield), op()).is_err());
    }

    #[test]
    fn wire_enums_round_trip() {
        for &c in ExoClass::ALL {
            assert_eq!(ExoClass::from_u8(c.as_u8()), Some(c));
            assert_eq!(ExoClass::from_slug(c.slug()), Some(c));
        }
        assert_eq!(ExoClass::ALL.len(), 9);
        assert_eq!(ExoClass::from_slug("Sentinel"), None);
        for &k in AlertFeedKind::ALL {
            assert_eq!(AlertFeedKind::from_u8(k.as_u8()), Some(k));
        }
        assert_eq!(AlertFeedKind::ALL.len(), 5, "FAB-EXO-801 lists five kinds");
    }

    #[test]
    fn feed_set_and_non_local() {
        let s = FeedSet::EMPTY.with(AlertFeedKind::LocalP);
        assert!(s.contains(AlertFeedKind::LocalP));
        assert_eq!(s.count(), 1);
        let mut a = FusedAlert {
            feeds: s,
            ..FusedAlert::default()
        };
        assert!(!a.non_local());
        a.feeds = s.with(AlertFeedKind::Network);
        assert!(a.non_local());
        assert!(
            FeedSet::EMPTY
                .with(AlertFeedKind::PegsMagnitudeAdvisory)
                .pegs_only()
        );
        a.feeds = s;
        a.injected = true;
        assert!(a.non_local(), "an injection is a non-local source");
    }

    #[test]
    fn safe_states_map_onto_core_actions() {
        assert_eq!(
            SafeStatePolicy::RideThrough.seismic_action(),
            SeismicAction::RideThrough
        );
        assert_eq!(
            SafeStatePolicy::CompleteClose.seismic_action(),
            SeismicAction::CompleteClose
        );
        assert_eq!(
            SafeStatePolicy::CloseAfterShaking.seismic_action(),
            SeismicAction::CloseAfterShaking
        );
        assert_eq!(
            SafeStatePolicy::LandAndLock.seismic_action(),
            SeismicAction::Lock
        );
    }
}
