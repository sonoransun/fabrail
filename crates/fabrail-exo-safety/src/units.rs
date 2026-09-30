//! Integer units: mass kg (u32), velocity mm/s (i32), 2*KE in u64 uJ, momentum u64 kg*mm/s, force N (i64), with checked conversions. Owner: W0a (complete in Wave 0, W0-C1).
//!
//! EXOTIC §X16; plan §4 "Safety (integer)"; FAB-EXO-112. The integer envelope gives
//! arithmetic diversity from the f32 control channel. There is no Q16.16.
//!
//! | Quantity | Type | Unit | Exact range |
//! |---|---|---|---|
//! | mass | [`MassKg`] (u32) | kg | 0 … 4.29e9 kg (tested 1 kg … 1e8 kg) |
//! | velocity | [`VelMmps`] (i32) | mm/s | ±2.1e9 mm/s (tested ±[`V_RANGE_MMPS`]) |
//! | 2·kinetic energy | [`Ke2Uj`] (u64) | µJ (m·v² in kg·mm²/s² = 1e-6 J) | checked; exact for m ≤ 1e8 kg and abs(v) ≤ 400 m/s |
//! | momentum | [`MomKgMmps`] (u64) | kg·mm/s | total (u32 × 2³¹ < 2⁶⁴) |
//! | force | [`ForceN`] (i64) | N | total for m·a (u32 × 2³¹ mN < 2⁶³) |
//!
//! Products are formed in u128/i128 and narrowed with checked conversions, so nothing
//! wraps: the checked forms return `None` and the saturating forms clamp. The f64
//! conversions are for configuration (`new()` time) only and round to nearest.

use fabrail_params::presets::{PEDESTRIAN_E_MAX_J, PEDESTRIAN_P_MAX_KGMPS};
use fabrail_types::fmath;

/// Velocity range covered by the range proptest, mm/s (100 m/s: far above any exotic
/// class's 1.5 m/s rim speed; keeps 2·KE of 1e8 kg below 2⁶⁴ with a 18× margin).
pub const V_RANGE_MMPS: i32 = 100_000;

/// Acceleration range covered by the range proptest, mm/s² (≈ 10 g).
pub const A_RANGE_MMPS2: i32 = 100_000;

/// Mass range of FAB-EXO-112: 1 kg …
pub const MASS_RANGE_MIN_KG: u32 = 1;

/// … 1e8 kg (HAVEN-F at 100 kt, the frontier extrapolation).
pub const MASS_RANGE_MAX_KG: u32 = 100_000_000;

/// Mass, kg.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct MassKg(pub u32);

impl MassKg {
    /// Rounds `m_kg` to the nearest kilogram; `None` if negative, non-finite or above
    /// `u32::MAX`.
    pub fn from_kg_f64(m_kg: f64) -> Option<MassKg> {
        if !m_kg.is_finite() || m_kg < 0.0 {
            return None;
        }
        let r = fmath::round(m_kg);
        if r > u32::MAX as f64 {
            return None;
        }
        Some(MassKg(r as u32))
    }

    /// Mass as f64 kg (exact).
    pub fn to_kg_f64(self) -> f64 {
        self.0 as f64
    }

    /// Checked sum.
    pub const fn checked_add(self, o: MassKg) -> Option<MassKg> {
        match self.0.checked_add(o.0) {
            Some(v) => Some(MassKg(v)),
            None => None,
        }
    }

    /// Saturating sum.
    pub const fn saturating_add(self, o: MassKg) -> MassKg {
        MassKg(self.0.saturating_add(o.0))
    }
}

/// Velocity, mm/s (signed; +x opens).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct VelMmps(pub i32);

impl VelMmps {
    /// Rounds `v_mps` to the nearest mm/s; `None` if non-finite or outside i32.
    pub fn from_mps_f64(v_mps: f64) -> Option<VelMmps> {
        if !v_mps.is_finite() {
            return None;
        }
        let r = fmath::round(v_mps * 1000.0);
        if r > i32::MAX as f64 || r < i32::MIN as f64 {
            return None;
        }
        Some(VelMmps(r as i32))
    }

    /// Velocity as f64 m/s.
    pub fn to_mps_f64(self) -> f64 {
        self.0 as f64 / 1000.0
    }

    /// Speed |v| in mm/s (total: `unsigned_abs`, no overflow at `i32::MIN`).
    pub const fn speed(self) -> u32 {
        self.0.unsigned_abs()
    }
}

/// Twice the kinetic energy, µJ: m (kg) · v (mm/s)² = 2·KE·1e6 (EXOTIC §X16).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct Ke2Uj(pub u64);

impl Ke2Uj {
    /// 2·KE of mass `m` at `v`; `None` if the product exceeds `u64::MAX`.
    pub const fn of(m: MassKg, v: VelMmps) -> Option<Ke2Uj> {
        let s = v.speed() as u128;
        let p = (m.0 as u128) * s * s;
        if p > u64::MAX as u128 {
            None
        } else {
            Some(Ke2Uj(p as u64))
        }
    }

    /// 2·KE of mass `m` at `v`, saturating at `u64::MAX` (a saturated value always
    /// exceeds any configured limit, so the check fails safe).
    pub const fn of_saturating(m: MassKg, v: VelMmps) -> Ke2Uj {
        match Ke2Uj::of(m, v) {
            Some(k) => k,
            None => Ke2Uj(u64::MAX),
        }
    }

    /// From a kinetic energy in joules (rounded to the nearest µJ of 2·KE); `None` if
    /// negative, non-finite or out of range.
    pub fn from_joules_f64(e_j: f64) -> Option<Ke2Uj> {
        if !e_j.is_finite() || e_j < 0.0 {
            return None;
        }
        let r = fmath::round(2.0e6 * e_j);
        if r > u64::MAX as f64 {
            return None;
        }
        Some(Ke2Uj(r as u64))
    }

    /// Kinetic energy in joules.
    pub fn to_joules_f64(self) -> f64 {
        self.0 as f64 / 2.0e6
    }

    /// Checked sum.
    pub const fn checked_add(self, o: Ke2Uj) -> Option<Ke2Uj> {
        match self.0.checked_add(o.0) {
            Some(v) => Some(Ke2Uj(v)),
            None => None,
        }
    }

    /// Saturating sum.
    pub const fn saturating_add(self, o: Ke2Uj) -> Ke2Uj {
        Ke2Uj(self.0.saturating_add(o.0))
    }
}

/// Momentum magnitude, kg·mm/s.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct MomKgMmps(pub u64);

impl MomKgMmps {
    /// |p| = m·|v|. Total: u32::MAX · 2³¹ < 2⁶⁴, so it never overflows.
    pub const fn of(m: MassKg, v: VelMmps) -> MomKgMmps {
        MomKgMmps(m.0 as u64 * v.speed() as u64)
    }

    /// From kg·m/s (rounded to the nearest kg·mm/s); `None` if negative, non-finite or
    /// out of range.
    pub fn from_kgmps_f64(p: f64) -> Option<MomKgMmps> {
        if !p.is_finite() || p < 0.0 {
            return None;
        }
        let r = fmath::round(p * 1000.0);
        if r > u64::MAX as f64 {
            return None;
        }
        Some(MomKgMmps(r as u64))
    }

    /// Momentum in kg·m/s.
    pub fn to_kgmps_f64(self) -> f64 {
        self.0 as f64 / 1000.0
    }
}

/// Force, N (signed).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct ForceN(pub i64);

impl ForceN {
    /// F = m·a with a in mm/s², truncated toward zero to whole newtons. Total:
    /// |m·a| ≤ u32::MAX · 2³¹ mN < 2⁶³.
    pub const fn from_mass_accel(m: MassKg, a_mmps2: i32) -> ForceN {
        let mn = (m.0 as i128) * (a_mmps2 as i128);
        ForceN((mn / 1000) as i64)
    }

    /// Rounds `f_n` to the nearest newton; `None` if non-finite or outside i64.
    pub fn from_n_f64(f_n: f64) -> Option<ForceN> {
        if !f_n.is_finite() {
            return None;
        }
        let r = fmath::round(f_n);
        // i64::MAX as f64 rounds up to 2^63, so compare strictly.
        if r >= i64::MAX as f64 || r < i64::MIN as f64 {
            return None;
        }
        Some(ForceN(r as i64))
    }

    /// Force as f64 N.
    pub fn to_n_f64(self) -> f64 {
        self.0 as f64
    }

    /// Checked sum.
    pub const fn checked_add(self, o: ForceN) -> Option<ForceN> {
        match self.0.checked_add(o.0) {
            Some(v) => Some(ForceN(v)),
            None => None,
        }
    }

    /// Checked difference.
    pub const fn checked_sub(self, o: ForceN) -> Option<ForceN> {
        match self.0.checked_sub(o.0) {
            Some(v) => Some(ForceN(v)),
            None => None,
        }
    }

    /// Saturating sum.
    pub const fn saturating_add(self, o: ForceN) -> ForceN {
        ForceN(self.0.saturating_add(o.0))
    }

    /// Magnitude |F| (total).
    pub const fn magnitude(self) -> u64 {
        self.0.unsigned_abs()
    }
}

/// Integer square root ⌊√x⌋ (no floating point; `core`'s exact `u64::isqrt`).
pub const fn isqrt_u64(x: u64) -> u64 {
    x.isqrt()
}

/// Energy speed cap v = ⌊√(2KE/m)⌋ in mm/s (PHYSICS §16 low-energy cap in integer
/// units); saturates at `i32::MAX`; 0 for a zero mass.
pub const fn speed_cap_energy(m: MassKg, e2: Ke2Uj) -> VelMmps {
    if m.0 == 0 {
        return VelMmps(0);
    }
    let v = isqrt_u64(e2.0 / m.0 as u64);
    if v > i32::MAX as u64 {
        VelMmps(i32::MAX)
    } else {
        VelMmps(v as i32)
    }
}

/// Momentum speed cap v = ⌊p/m⌋ in mm/s; saturates at `i32::MAX`; 0 for a zero mass.
pub const fn speed_cap_momentum(m: MassKg, p: MomKgMmps) -> VelMmps {
    if m.0 == 0 {
        return VelMmps(0);
    }
    let v = p.0 / m.0 as u64;
    if v > i32::MAX as u64 {
        VelMmps(i32::MAX)
    } else {
        VelMmps(v as i32)
    }
}

/// The pedestrian kinetic-energy cap as 2·KE in µJ, derived from the imported
/// `fabrail_params::presets::PEDESTRIAN_E_MAX_J` (never restated; FAB-EXO-114).
/// Rounded to the nearest µJ (the f64 product sits a fraction of an ulp below it).
pub const PEDESTRIAN_KE2: Ke2Uj = Ke2Uj((2.0e6 * PEDESTRIAN_E_MAX_J + 0.5) as u64);

/// The pedestrian momentum cap in kg·mm/s, derived from the imported
/// `fabrail_params::presets::PEDESTRIAN_P_MAX_KGMPS` (FAB-EXO-114).
pub const PEDESTRIAN_MOM: MomKgMmps = MomKgMmps((1.0e3 * PEDESTRIAN_P_MAX_KGMPS + 0.5) as u64);

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    #[test]
    fn caps_are_imported_and_exact() {
        // 2 · E_max · 1e6 and p_max · 1e3 from the imported f64 constants.
        assert_eq!(
            PEDESTRIAN_KE2.0 as f64,
            (2.0e6 * PEDESTRIAN_E_MAX_J).round()
        );
        assert_eq!(
            PEDESTRIAN_MOM.0 as f64,
            (1.0e3 * PEDESTRIAN_P_MAX_KGMPS).round()
        );
        assert!((PEDESTRIAN_KE2.to_joules_f64() - PEDESTRIAN_E_MAX_J).abs() < 1e-12);
    }

    #[test]
    fn isqrt_edges() {
        for x in [
            0u64,
            1,
            2,
            3,
            4,
            15,
            16,
            17,
            99,
            100,
            u32::MAX as u64,
            u64::MAX,
        ] {
            let r = isqrt_u64(x);
            assert!(r.checked_mul(r).is_some_and(|rr| rr <= x), "x = {x}");
            let r1 = r + 1;
            assert!(r1.checked_mul(r1).is_none_or(|rr| rr > x), "x = {x}");
        }
    }

    #[test]
    fn extremes_never_wrap() {
        let m = MassKg(u32::MAX);
        let v = VelMmps(i32::MIN);
        assert_eq!(Ke2Uj::of(m, v), None, "overflow is reported, not wrapped");
        assert_eq!(Ke2Uj::of_saturating(m, v), Ke2Uj(u64::MAX));
        assert_eq!(
            MomKgMmps::of(m, v).0 as u128,
            u32::MAX as u128 * (1u128 << 31)
        );
        assert_eq!(
            ForceN::from_mass_accel(m, i32::MIN).0 as i128,
            (u32::MAX as i128 * i32::MIN as i128) / 1000
        );
        assert_eq!(speed_cap_energy(MassKg(0), Ke2Uj(5)), VelMmps(0));
        assert_eq!(
            speed_cap_energy(MassKg(1), Ke2Uj(u64::MAX)),
            VelMmps(i32::MAX)
        );
        assert_eq!(MassKg::from_kg_f64(-1.0), None);
        assert_eq!(MassKg::from_kg_f64(f64::NAN), None);
        assert_eq!(VelMmps::from_mps_f64(3.0e6), None);
        assert_eq!(ForceN::from_n_f64(1.0e19), None);
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(4096))]

        // req: FAB-EXO-112
        #[test]
        fn integer_range_1kg_to_1e8kg(
            m in MASS_RANGE_MIN_KG..=MASS_RANGE_MAX_KG,
            v in -V_RANGE_MMPS..=V_RANGE_MMPS,
            a in -A_RANGE_MMPS2..=A_RANGE_MMPS2,
        ) {
            let mass = MassKg(m);
            let vel = VelMmps(v);
            // 2·KE: exact integer product, no overflow anywhere in the range.
            let ke = Ke2Uj::of(mass, vel);
            prop_assert!(ke.is_some(), "2KE overflow at m = {} kg, v = {} mm/s", m, v);
            let ke = ke.unwrap_or_default();
            let exact = m as u128 * (v as i128 * v as i128) as u128;
            prop_assert_eq!(ke.0 as u128, exact);
            // Joules agree with the f64 reference ½·m·v² to rounding.
            let e_ref = 0.5 * m as f64 * (v as f64 / 1000.0).powi(2);
            prop_assert!((ke.to_joules_f64() - e_ref).abs() <= 1e-9 * e_ref.max(1.0));
            // Momentum: exact and never wraps.
            let p = MomKgMmps::of(mass, vel);
            prop_assert_eq!(p.0 as u128, m as u128 * v.unsigned_abs() as u128);
            // Force: exact truncation of m·a/1000.
            let f = ForceN::from_mass_accel(mass, a);
            prop_assert_eq!(f.0 as i128, (m as i128 * a as i128) / 1000);
            // Speed caps invert the products (floor semantics).
            let v_e = speed_cap_energy(mass, ke);
            prop_assert_eq!(v_e.0 as u32, v.unsigned_abs());
            let v_p = speed_cap_momentum(mass, p);
            prop_assert_eq!(v_p.0 as u32, v.unsigned_abs());
            // Sums saturate instead of wrapping.
            prop_assert!(ke.checked_add(Ke2Uj(u64::MAX)).is_none() || ke.0 == 0);
            prop_assert_eq!(ke.saturating_add(Ke2Uj(u64::MAX)), Ke2Uj(u64::MAX));
            // f64 configuration round trips.
            prop_assert_eq!(MassKg::from_kg_f64(m as f64), Some(mass));
            prop_assert_eq!(VelMmps::from_mps_f64(v as f64 / 1000.0), Some(vel));
        }
    }
}
