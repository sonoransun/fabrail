//! Fault registry and the 128-bit fault set (core design §B.1; plan §2b item 10, §4b).
//!
//! Codes are grouped by the high byte: 0x01 sensing, 0x02 actuation, 0x03 power,
//! 0x04 execution, 0x05 levitation, 0x06 motion safety, 0x07 seismic. Codes are wire
//! values (telemetry, CAN, crash records) and are never renumbered; a retired code keeps
//! its number. Each code has a registry index (its bit in [`FaultSet`]) and a default
//! [`Severity`] that the supervisor's fault reaction starts from.

use crate::{SafetyDemand, Severity};

macro_rules! fault_codes {
    (
        $( $(#[$vmeta:meta])* $var:ident = $val:literal => $sev:ident ),+ $(,)?
    ) => {
        /// Fault code registry (`#[repr(u16)]` wire values).
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[repr(u16)]
        pub enum FaultCode {
            $( $(#[$vmeta])* $var = $val ),+
        }

        impl FaultCode {
            /// Every code in registry order; the position is the [`FaultSet`] bit index.
            pub const ALL: &'static [FaultCode] = &[$(FaultCode::$var),+];

            /// The wire value.
            #[inline]
            pub const fn as_u16(self) -> u16 {
                self as u16
            }

            /// Decodes a wire value; `None` for an unknown code.
            pub const fn from_u16(v: u16) -> Option<FaultCode> {
                match v {
                    $( $val => Some(FaultCode::$var), )+
                    _ => None,
                }
            }

            /// Default reaction class.
            pub const fn severity(self) -> Severity {
                match self {
                    $( FaultCode::$var => Severity::$sev, )+
                }
            }

            /// The variant name.
            pub const fn name(self) -> &'static str {
                match self {
                    $( FaultCode::$var => stringify!($var), )+
                }
            }
        }
    };
}

fault_codes! {
    // ---- 0x01xx sensing -------------------------------------------------------------
    /// Gap reading outside the sensor range.
    GapRange = 0x0101 => SafeLand,
    /// Gap reading frozen while the coil current changes.
    GapStuck = 0x0102 => SafeLand,
    /// Primary and redundant gap probes disagree (one channel excluded).
    GapDisagree = 0x0103 => Degrade,
    /// Encoder signal lost.
    EncoderLost = 0x0110 => SafeStop,
    /// Encoder position jump beyond physical plausibility.
    EncoderJump = 0x0111 => SafeStop,
    /// Structure accelerometer silent (seismic detection degraded).
    AccelLost = 0x0120 => Warn,
    /// Structure accelerometer output frozen.
    AccelStuck = 0x0121 => Warn,
    /// Carriage IMU lost (skyhook / inertial smoothing unavailable).
    ImuLost = 0x0122 => Degrade,
    /// Rail-mounted floor accelerometer lost (inertial smoothing unavailable).
    RailAccelLost = 0x0123 => Degrade,
    /// Coil current sense implausible.
    CurrentSense = 0x0130 => SafeLand,

    // ---- 0x02xx actuation -----------------------------------------------------------
    /// Coil current above the hardware clamp i_pulse.
    CoilOvercurrent = 0x0201 => SafeLand,
    /// Coil open circuit.
    CoilOpen = 0x0202 => SafeLand,
    /// Coil H-bridge fault.
    CoilDriver = 0x0203 => SafeLand,
    /// Coil I²t budget exhausted (i_cont continuous, i_pulse for ≤ t_pulse; plan §2b 4).
    CoilI2t = 0x0204 => SafeLand,
    /// LSM phase overcurrent.
    LsmOvercurrent = 0x0210 => Sto,
    /// LSM phase currents do not sum to zero.
    LsmPhaseSum = 0x0211 => Sto,
    /// LSM gate-driver fault.
    LsmDriver = 0x0212 => Sto,
    /// Parking lock failed to engage/release.
    LockFailed = 0x0220 => Warn,
    /// Seismic x-latch failed to engage/release.
    LatchFailed = 0x0221 => Warn,

    // ---- 0x03xx power ---------------------------------------------------------------
    /// DC bus undervoltage.
    BusUnder = 0x0301 => SafeLand,
    /// DC bus overvoltage (regeneration beyond the chopper).
    BusOver = 0x0302 => SafeStop,
    /// Mains lost: SS1 Safe Landing on the supercap (SAF-009).
    MainsLost = 0x0303 => SafeLand,
    /// Hold-up energy below 3 × landing need: refuse lift.
    HoldupLow = 0x0304 => Degrade,
    /// Coil or magnet over temperature.
    OverTemp = 0x0310 => SafeLand,

    // ---- 0x04xx execution -----------------------------------------------------------
    /// Fast group not finished by r + P (DWT on firmware).
    FastOverrun = 0x0401 => SafeLand,
    /// Supervisor group overrun.
    SupervisorOverrun = 0x0402 => SafeLand,
    /// Reset by a watchdog.
    WatchdogReset = 0x0403 => SafeLand,
    /// Parameter block CRC or diverse-copy mismatch: refuse to lift (SAF-015).
    ParamCrc = 0x0404 => SafeLand,
    /// Parameter out of validated range.
    ParamRange = 0x0405 => SafeLand,
    /// Boot selftest hash differs from GOLDEN_SELFTEST_HASH.
    SelftestHash = 0x0406 => SafeLand,
    /// Non-finite value in a controller output (SAF-013/017).
    NonFinite = 0x0407 => SafeLand,
    /// Core heartbeat stale for more than 3 periods.
    HeartbeatLost = 0x0408 => Sto,
    /// Seismic group overrun.
    SeismicOverrun = 0x0409 => Warn,

    // ---- 0x05xx levitation ----------------------------------------------------------
    /// Gap outside the controlled envelope.
    GapEnvelope = 0x0501 => SafeLand,
    /// Lift/unseat did not complete in time.
    LiftTimeout = 0x0502 => SafeLand,
    /// Landing did not complete in time.
    LandTimeout = 0x0503 => SafeStop,
    /// m̂ > 1.05·m_rated: stays levitated, motion drops to hold-to-run (plan §2b 10).
    PayloadMismatch = 0x0504 => Degrade,
    /// Coil or LSM saturation persisted.
    SaturationPersist = 0x0505 => SafeLand,
    /// DRAWER z-seat step failed (lands on the travel rollers; manual re-seat).
    SeatFailed = 0x0506 => SafeLand,
    /// Encoder not referenced: hold-to-run until homed.
    NotReferenced = 0x0507 => Degrade,

    // ---- 0x06xx motion safety -------------------------------------------------------
    /// ½·m_gov·v² above the energy cap.
    EnergyLimit = 0x0601 => SafeStop,
    /// m_gov·v above the momentum cap.
    MomentumLimit = 0x0602 => SafeStop,
    /// Directional force clamp exceeded.
    ForceLimit = 0x0603 => SafeStop,
    /// v > 1.1·v_lim for > 10 ms: STO + ASC (SAF-008).
    Overspeed = 0x0604 => Sto,
    /// Safely-limited position violated.
    PositionLimit = 0x0605 => SafeStop,
    /// Motion without a command.
    UnintendedMotion = 0x0606 => Sto,
    /// Obstruction detected (stop and reverse / release).
    Obstruction = 0x0607 => Warn,
    /// Passivity observer energy below its bound.
    PassivityViolation = 0x0608 => Degrade,
    /// Motion requested on a hold-to-run device without hold-to-run actuated.
    NotHoldToRun = 0x0609 => Warn,
    /// Obstruction threshold degraded by floor shaking beyond 2·th0 (plan §4b):
    /// SeatAndHold, no automatic motion.
    ObstructionDegraded = 0x060A => Degrade,
    /// ISOLATE mode left the explicit `IsolateLimits` envelope (SAF-020).
    IsolateLimit = 0x060B => SafeStop,

    // ---- 0x07xx seismic -------------------------------------------------------------
    /// Vertical saturation detected (0.8 g floor pulse, SEI-017): land.
    VerticalSaturation = 0x0701 => SafeLand,
    /// Centring requested but not reachable before t_S (F11).
    CenterUnreachable = 0x0702 => Warn,
}

impl FaultCode {
    /// Key-space lookup table: `(code >> 8) * 64 + (code & 0x3F)` → registry index.
    /// Built at compile time; a duplicate code or a code outside the key space
    /// (high byte ≥ 8 or low byte ≥ 64) is a compile error.
    const INDEX_LUT: [u8; 512] = {
        let mut t = [u8::MAX; 512];
        let mut i = 0usize;
        while i < FaultCode::ALL.len() {
            let c = FaultCode::ALL[i] as u16;
            let hi = (c >> 8) as usize;
            let lo = (c & 0xFF) as usize;
            assert!(
                hi < 8 && lo < 64,
                "fault code outside the FaultSet key space"
            );
            let k = hi * 64 + lo;
            assert!(t[k] == u8::MAX, "duplicate fault code");
            t[k] = i as u8;
            i += 1;
        }
        t
    };

    /// Bit index in [`FaultSet`] (registry order, < 128). O(1) table lookup.
    #[inline]
    pub const fn index(self) -> u8 {
        let c = self as u16 as usize;
        Self::INDEX_LUT[((c >> 8) & 7) * 64 + (c & 63)]
    }

    /// The demand implied by this code's default severity.
    #[inline]
    pub const fn demand(self) -> SafetyDemand {
        self.severity().demand()
    }
}

const _: () = assert!(FaultCode::ALL.len() <= 128, "FaultSet holds 128 bits");

/// Set of active faults: one bit per registry index (128 bits).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(C)]
pub struct FaultSet {
    /// Raw bits, word `i / 32`, bit `i % 32` for registry index `i`.
    pub bits: [u32; 4],
}

impl FaultSet {
    /// The empty set.
    pub const EMPTY: FaultSet = FaultSet { bits: [0; 4] };

    /// The empty set.
    #[inline]
    pub const fn new() -> FaultSet {
        FaultSet::EMPTY
    }

    #[inline]
    const fn word_bit(c: FaultCode) -> (usize, u32) {
        let i = c.index() as usize;
        (i / 32, 1u32 << (i % 32))
    }

    /// Adds `c`.
    #[inline]
    pub fn set(&mut self, c: FaultCode) {
        let (w, b) = Self::word_bit(c);
        if let Some(word) = self.bits.get_mut(w) {
            *word |= b;
        }
    }

    /// Removes `c`.
    #[inline]
    pub fn clear(&mut self, c: FaultCode) {
        let (w, b) = Self::word_bit(c);
        if let Some(word) = self.bits.get_mut(w) {
            *word &= !b;
        }
    }

    /// True when `c` is in the set.
    #[inline]
    pub fn contains(&self, c: FaultCode) -> bool {
        let (w, b) = Self::word_bit(c);
        self.bits.get(w).is_some_and(|word| word & b != 0)
    }

    /// True when no fault is set.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.bits[0] == 0 && self.bits[1] == 0 && self.bits[2] == 0 && self.bits[3] == 0
    }

    /// Number of faults set.
    #[inline]
    pub const fn count(&self) -> u32 {
        self.bits[0].count_ones()
            + self.bits[1].count_ones()
            + self.bits[2].count_ones()
            + self.bits[3].count_ones()
    }

    /// Set union.
    #[inline]
    pub const fn union(self, o: FaultSet) -> FaultSet {
        FaultSet {
            bits: [
                self.bits[0] | o.bits[0],
                self.bits[1] | o.bits[1],
                self.bits[2] | o.bits[2],
                self.bits[3] | o.bits[3],
            ],
        }
    }

    /// Adds every fault in `o`.
    #[inline]
    pub fn merge(&mut self, o: FaultSet) {
        *self = self.union(o);
    }

    /// Iterates the set in registry order (no allocation).
    #[inline]
    pub fn iter(&self) -> FaultIter {
        FaultIter {
            set: *self,
            next: 0,
        }
    }

    /// Highest default severity in the set, `None` when empty.
    pub fn max_severity(&self) -> Option<Severity> {
        self.iter().map(FaultCode::severity).max()
    }

    /// Demand implied by the highest severity (`None` when empty).
    pub fn demand(&self) -> SafetyDemand {
        match self.max_severity() {
            Some(s) => s.demand(),
            None => SafetyDemand::None,
        }
    }
}

/// Iterator over a [`FaultSet`] in registry order.
#[derive(Clone, Debug)]
pub struct FaultIter {
    set: FaultSet,
    next: usize,
}

impl Iterator for FaultIter {
    type Item = FaultCode;

    fn next(&mut self) -> Option<FaultCode> {
        while let Some(&c) = FaultCode::ALL.get(self.next) {
            self.next += 1;
            if self.set.contains(c) {
                return Some(c);
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_is_consistent() {
        for (i, &c) in FaultCode::ALL.iter().enumerate() {
            assert_eq!(c.index() as usize, i);
            assert_eq!(FaultCode::from_u16(c.as_u16()), Some(c));
            // Codes are unique.
            assert_eq!(
                FaultCode::ALL
                    .iter()
                    .filter(|&&o| o.as_u16() == c.as_u16())
                    .count(),
                1
            );
        }
        assert_eq!(FaultCode::from_u16(0x0000), None);
        assert_eq!(
            FaultCode::SeatFailed.as_u16(),
            0x0506,
            "core design §B.1 pins SeatFailed"
        );
        assert_eq!(
            FaultCode::PayloadMismatch.severity(),
            Severity::Degrade,
            "plan §2b item 10"
        );
        assert_eq!(FaultCode::Overspeed.demand(), SafetyDemand::StoLsm);
    }

    #[test]
    fn bitset_roundtrip() {
        let mut s = FaultSet::new();
        assert!(s.is_empty());
        for &c in FaultCode::ALL {
            s.set(c);
            assert!(s.contains(c));
        }
        assert_eq!(s.count() as usize, FaultCode::ALL.len());
        let collected: [Option<FaultCode>; 4] = {
            let mut it = s.iter();
            [it.next(), it.next(), it.next(), it.next()]
        };
        assert_eq!(collected[0], FaultCode::ALL.first().copied());
        assert_eq!(s.iter().count(), FaultCode::ALL.len());
        for &c in FaultCode::ALL {
            s.clear(c);
            assert!(!s.contains(c));
        }
        assert!(s.is_empty());

        let mut a = FaultSet::new();
        a.set(FaultCode::GapRange);
        let mut b = FaultSet::new();
        b.set(FaultCode::ObstructionDegraded);
        let u = a.union(b);
        assert!(u.contains(FaultCode::GapRange) && u.contains(FaultCode::ObstructionDegraded));
        assert_eq!(u.count(), 2);
        a.merge(b);
        assert_eq!(a, u);
        assert_eq!(u.max_severity(), Some(Severity::SafeLand));
        assert_eq!(u.demand(), SafetyDemand::SafeLand);
        assert_eq!(FaultSet::EMPTY.max_severity(), None);
        assert_eq!(FaultSet::EMPTY.demand(), SafetyDemand::None);
    }
}
