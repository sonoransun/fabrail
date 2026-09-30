//! Time and absolute position (PHYSICS §24; core design §A.4 items 2–3).
//!
//! * [`Tick`] counts 20 kHz base ticks. There is never an absolute f32 time; plant time
//!   is `tick / BASE_TICK_HZ` in f64, computed rather than accumulated. The MCU has no
//!   `AtomicU64`, so ticks travel only through RTIC locks or LET mailboxes.
//! * [`PosNm`] is absolute travel position in integer nanometres (±9.2e9 m range).
//!   Between 8 and 16 m, f32 values are 0.954 µm apart, which would make admittance
//!   integration at 0.025 m/s and 10 kHz (2.5 µm per tick) up to 19 % wrong per step;
//!   so all differences are taken in integers first, then cast.
//! * [`PosAccum`] integrates admittance/trajectory state as an integer base plus an f32
//!   sub-nanometre fraction, so it does not drift over a 15 m track.

use crate::{BASE_TICK_HZ, fmath};

/// Base-tick timestamp (20 kHz, h = 50 µs).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
#[repr(transparent)]
pub struct Tick(pub u64);

impl Tick {
    /// Tick zero.
    pub const ZERO: Tick = Tick(0);

    /// Time in seconds as f64: `tick / BASE_TICK_HZ` (a single correctly rounded
    /// division, exact for whole seconds; u64 → f64 is exact below 2^53 ticks).
    #[inline]
    pub fn seconds_f64(self) -> f64 {
        self.0 as f64 / BASE_TICK_HZ as f64
    }

    /// The tick nearest to `t_s` seconds; negative or NaN inputs saturate to tick 0.
    #[inline]
    pub fn from_seconds_f64(t_s: f64) -> Tick {
        // `as u64` saturates (NaN → 0), so this never panics.
        Tick(fmath::round(t_s * BASE_TICK_HZ as f64) as u64)
    }

    /// `self + n` ticks, saturating at `u64::MAX`.
    #[inline]
    pub const fn plus(self, n: u64) -> Tick {
        Tick(self.0.saturating_add(n))
    }

    /// Ticks elapsed since `earlier`, saturating at 0 if `earlier` is in the future.
    #[inline]
    pub const fn since(self, earlier: Tick) -> u64 {
        self.0.saturating_sub(earlier.0)
    }

    /// Seconds elapsed since `earlier` as f32 (relative times only; never absolute).
    #[inline]
    pub fn since_s(self, earlier: Tick) -> f32 {
        (self.since(earlier) as f64 / BASE_TICK_HZ as f64) as f32
    }

    /// True when a group with decimation `decim` is released at this tick
    /// (`tick % decim == 0`). A zero decimation never releases.
    #[inline]
    pub const fn is_release(self, decim: u32) -> bool {
        decim != 0 && self.0.is_multiple_of(decim as u64)
    }
}

/// Absolute position along the rail in nanometres (+x opens).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
#[repr(transparent)]
pub struct PosNm(pub i64);

impl PosNm {
    /// The rail origin.
    pub const ZERO: PosNm = PosNm(0);

    /// Nanometres per metre.
    pub const NM_PER_M: f64 = 1.0e9;

    /// Integer difference `self − o` in nanometres (wrapping, never panics).
    #[inline]
    pub const fn diff_nm(self, o: PosNm) -> i64 {
        self.0.wrapping_sub(o.0)
    }

    /// Difference `self − o` in metres as f32: the integer difference is taken first,
    /// then cast, so it is exact to f32 rounding at any absolute distance.
    #[inline]
    pub fn diff_m(self, o: PosNm) -> f32 {
        (self.diff_nm(o) as f64 / Self::NM_PER_M) as f32
    }

    /// This position offset by `d_m` metres (rounded to the nearest nanometre,
    /// saturating at the i64 range).
    #[inline]
    pub fn offset_m(self, d_m: f32) -> PosNm {
        let d_nm = fmath::round(d_m as f64 * Self::NM_PER_M) as i64;
        PosNm(self.0.saturating_add(d_nm))
    }

    /// The position nearest to `m` metres.
    #[inline]
    pub fn from_m_f64(m: f64) -> PosNm {
        PosNm(fmath::round(m * Self::NM_PER_M) as i64)
    }

    /// This position in metres as f64 (exact below 2^53 nm ≈ 9.0e6 m, then one rounding).
    #[inline]
    pub fn to_m_f64(self) -> f64 {
        self.0 as f64 / Self::NM_PER_M
    }
}

/// Drift-free position integrator: integer base plus an f32 fraction in (−1, 1) nm.
///
/// `advance` splits each increment into its integer part (added exactly to `base`) and
/// its fractional part (accumulated in `frac_nm`, carried when it passes ±1 nm). The
/// fraction subtraction is exact (Sterbenz), so the only accumulated error is the f32
/// rounding of a sum below 2 nm: ≤ 1.2e-7 nm per step, ≤ 0.12 nm per 1e6 steps.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PosAccum {
    /// Integer part of the integrated position.
    pub base: PosNm,
    /// Sub-nanometre remainder, always in (−1, 1).
    pub frac_nm: f32,
}

impl PosAccum {
    /// An accumulator starting exactly at `base`.
    #[inline]
    pub const fn new(base: PosNm) -> PosAccum {
        PosAccum { base, frac_nm: 0.0 }
    }

    /// Integrates `v_mps` over `dt_s`. Non-finite increments are ignored (the caller's
    /// NonFinite fault path handles them); this never panics.
    #[inline]
    pub fn advance(&mut self, v_mps: f32, dt_s: f32) {
        let d_nm = v_mps * dt_s * 1.0e9;
        if !d_nm.is_finite() {
            return;
        }
        let whole = fmath::truncf(d_nm);
        // Exact: `whole` and `d_nm` share sign and |whole| >= |d_nm| / 2 (or whole = 0).
        let rem = d_nm - whole;
        let mut frac = self.frac_nm + rem;
        let carry = fmath::truncf(frac);
        frac -= carry;
        self.base = PosNm(
            self.base
                .0
                .saturating_add(whole as i64)
                .saturating_add(carry as i64),
        );
        self.frac_nm = frac;
    }

    /// The integrated position, truncated to whole nanometres.
    #[inline]
    pub const fn pos(&self) -> PosNm {
        self.base
    }

    /// The integrated position in metres including the fraction (plots and tests only).
    #[inline]
    pub fn pos_m_f64(&self) -> f64 {
        self.base.to_m_f64() + self.frac_nm as f64 / PosNm::NM_PER_M
    }

    /// Re-anchors the accumulator exactly at `p` (e.g. after homing).
    #[inline]
    pub fn reset(&mut self, p: PosNm) {
        self.base = p;
        self.frac_nm = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 1 nm step 15 m from the origin is exact through `diff_m`, whereas absolute f32
    /// metres cannot even represent it. Tolerance (E): bit-exact, PHYSICS §24.
    #[test]
    fn posnm_diff_exact() {
        let a = PosNm(15_000_000_001);
        let b = PosNm(15_000_000_000);
        assert_eq!(
            a.diff_m(b),
            1.0e-9_f32,
            "(E) integer-first difference, PHYSICS §24"
        );
        assert_eq!(b.diff_m(a), -1.0e-9_f32);
        // Absolute f32 metres lose the step entirely (f32 spacing at 15 m is 0.95 µm).
        assert_eq!(15.000_000_001_f32 - 15.0_f32, 0.0);
        assert_eq!(a.diff_nm(b), 1);
        assert_eq!(PosNm::from_m_f64(1.234_567_891).0, 1_234_567_891);
        assert_eq!(PosNm(2_500_000_000).to_m_f64(), 2.5);
        assert_eq!(PosNm(0).offset_m(0.25).0, 250_000_000);
        assert_eq!(
            PosNm(i64::MAX).offset_m(1.0).0,
            i64::MAX,
            "saturating, no panic"
        );
    }

    // req: FAB-PRP-008
    /// 1e6 steps of 0.15 m/s × 100 µs cover 15 m; the accumulator must equal the exact
    /// f64 sum of the per-step f32 increments to within 1 nm (0 nm of drift beyond the
    /// sub-nanometre fraction). Tolerance (E) ±1 nm, PHYSICS §24.
    #[test]
    fn posaccum_no_drift_15m() {
        let (v, dt) = (0.15_f32, 1.0e-4_f32);
        let d = v * dt * 1.0e9;
        let n = 1_000_000_u32;
        let mut acc = PosAccum::new(PosNm(0));
        for _ in 0..n {
            acc.advance(v, dt);
        }
        let expected = n as f64 * d as f64;
        let got = acc.pos().0 as f64 + acc.frac_nm as f64;
        assert!(
            (got - expected).abs() <= 1.0,
            "(E) ±1 nm over 15 m, PHYSICS §24: got {got} nm, expected {expected} nm"
        );
        assert!(acc.frac_nm.abs() < 1.0);
        assert!((acc.pos().to_m_f64() - 15.0).abs() < 1.0e-3);

        // Integer increment (0.25 m/s × 100 µs = 25 000 nm in f32): exactly 0 nm drift.
        let (v2, dt2) = (0.25_f32, 1.0e-4_f32);
        let d2 = v2 * dt2 * 1.0e9;
        assert_eq!(d2, 25_000.0);
        let mut acc2 = PosAccum::new(PosNm(-7));
        for _ in 0..600_000 {
            acc2.advance(v2, dt2);
        }
        assert_eq!(acc2.pos().0, 15_000_000_000 - 7, "(E) 0 nm drift over 15 m");
        assert_eq!(acc2.frac_nm, 0.0);

        // Negative velocity walks back without bias.
        for _ in 0..600_000 {
            acc2.advance(-v2, dt2);
        }
        assert_eq!(acc2.pos().0, -7);
    }

    #[test]
    fn posaccum_ignores_non_finite() {
        let mut acc = PosAccum::new(PosNm(42));
        acc.advance(f32::NAN, 1.0e-4);
        acc.advance(f32::INFINITY, 1.0e-4);
        assert_eq!(acc.pos(), PosNm(42));
        assert_eq!(acc.frac_nm, 0.0);
    }

    /// Tick → seconds is a single division: exact for whole seconds and for one tick.
    /// Tolerance (E): bit-exact.
    #[test]
    fn tick_time_exact() {
        assert_eq!(Tick(20_000).seconds_f64(), 1.0);
        assert_eq!(Tick(1).seconds_f64(), 5.0e-5);
        assert_eq!(Tick(1).seconds_f64(), crate::BASE_TICK_S);
        assert_eq!(Tick(72_000_000).seconds_f64(), 3600.0);
        assert_eq!(Tick::from_seconds_f64(1.5), Tick(30_000));
        assert_eq!(Tick::from_seconds_f64(-2.0), Tick(0), "negative saturates");
        assert_eq!(Tick::from_seconds_f64(f64::NAN), Tick(0), "NaN saturates");
        let far = Tick(1 << 40);
        assert_eq!(far.plus(20_000).since(far), 20_000);
        assert_eq!(far.since(far.plus(1)), 0, "saturating");
        assert_eq!(far.plus(20_000).since_s(far), 1.0);
        assert!(Tick(40).is_release(2) && Tick(40).is_release(4) && Tick(40).is_release(20));
        assert!(!Tick(41).is_release(2));
        assert!(!Tick(40).is_release(0), "zero decimation never releases");
    }
}
