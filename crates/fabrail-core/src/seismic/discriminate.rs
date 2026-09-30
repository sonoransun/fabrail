//! Level matrix, Z/H polarisation, actuator veto and PGA bypass; PROTECT rule (PHYSICS §20; SEI-005). Owner: A3.
//!
//! Frozen API (W0): [`level`] (Pd/τc matrix: τc ≥ 0.6 s and Pd ≥ 0.5 cm → L3; τc only
//! → L2; Pd only → L1), [`protect`] (PROTECT fires when level ≥ 2 OR Pd ≥ 0.35 cm OR
//! PGA ≥ 80 gal; level 1 means freeze only — plan §2b item 6), [`ZhCheck`] (Z/H > 1 in
//! the first 0.5 s) and [`vetoed`] (actuator veto unless PGA ≥ 80 gal). Tests:
//! `alert_levels`, `veto_bypass_pga`.
//!
//! Thresholds are the `DetectorTuning` calibration parameters (P-Alert practice, Wu &
//! Kanamori 2005; core design §C.4); every comparison is inclusive (≥), and a non-finite
//! Pd, τc or PGA never raises a level or PROTECT on its own. The level and the Pd rule are
//! applied by `SeismicUnit` only once the short (1 s) Pd/τc window has run on a Z/H-
//! confirmed pick; the PGA rule applies at any time, trigger or not (strong motion
//! already present needs no P-wave estimate).

use fabrail_params::DetectorTuning;
use fabrail_types::SeismicLevel;

/// Detector level from Pd (m) and τc (s): the calibration matrix
/// (τc ≥ `tauc_l3_s` and Pd ≥ `pd_l3_m` → L3; τc only → L2; Pd only → L1; else L0).
pub fn level(pd_m: f32, tau_c_s: f32, t: &DetectorTuning) -> SeismicLevel {
    let long = tau_c_s >= t.tauc_l3_s as f32;
    let big = pd_m >= t.pd_l3_m as f32;
    match (long, big) {
        (true, true) => SeismicLevel::L3,
        (true, false) => SeismicLevel::L2,
        (false, true) => SeismicLevel::L1,
        (false, false) => SeismicLevel::L0,
    }
}

/// PROTECT rule: level ≥ `protect_level` (2) OR Pd ≥ `pd_alert_m` (0.35 cm) OR
/// PGA ≥ `pga_alert_mps2` (80 gal). Level 1 alone means freeze only (plan §2b item 6).
pub fn protect(level: SeismicLevel, pd_m: f32, pga_mps2: f32, t: &DetectorTuning) -> bool {
    level.as_u8() >= t.protect_level
        || pd_m >= t.pd_alert_m as f32
        || pga_mps2 >= t.pga_alert_mps2 as f32
}

/// Whether a trigger is suppressed by the actuator veto (self-actuation): true when
/// `veto` is set and PGA is below the bypass threshold `pga_veto_mps2` (80 gal).
pub fn vetoed(veto: bool, pga_mps2: f32, t: &DetectorTuning) -> bool {
    veto && (pga_mps2.is_nan() || pga_mps2 < t.pga_veto_mps2 as f32)
}

/// Z/H polarisation check over the first `zh_window_s` after the pick.
///
/// Z = vertical, H = horizontal vector magnitude of the band-passed acceleration; the
/// check passes when the amplitude ratio √(Σz²/Σ(x² + y²)) exceeds `zh_min`, evaluated
/// without a square root as Σz² > zh_min²·Σ(x² + y²). A P wave arriving at 20° from
/// vertical has Z/H = 1/tan 20° ≈ 2.7; the S phase ≈ 0.3 (physics perspective §3), and
/// door slams or bumps load the horizontals.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ZhCheck {
    /// Accumulated vertical energy.
    pub e_z: f32,
    /// Accumulated horizontal energy.
    pub e_h: f32,
    /// Samples accumulated.
    pub n: u32,
    /// Window length, samples (0: unbounded accumulation; the check is then disabled in
    /// `SeismicUnit`).
    pub n_window: u32,
}

impl ZhCheck {
    /// A check over `zh_window_s` at sample rate `fs_hz`.
    pub fn new(t: &DetectorTuning, fs_hz: f32) -> ZhCheck {
        let n = t.zh_window_s * f64::from(fs_hz);
        let n_window = if n.is_finite() && n >= 1.0 {
            fabrail_types::fmath::round(n) as u32
        } else {
            0
        };
        ZhCheck {
            n_window,
            ..ZhCheck::default()
        }
    }

    /// Accumulates one band-passed sample (x, y, z), m/s²; samples beyond the window and
    /// non-finite samples are ignored.
    pub fn push(&mut self, bp_mps2: [f32; 3]) {
        if self.done() || !bp_mps2.iter().all(|v| v.is_finite()) {
            return;
        }
        let [x, y, z] = bp_mps2;
        self.e_z += z * z;
        self.e_h += x * x + y * y;
        self.n = self.n.saturating_add(1);
    }

    /// The window has been filled.
    #[inline]
    pub fn done(&self) -> bool {
        self.n_window > 0 && self.n >= self.n_window
    }

    /// Z/H > `zh_min` (P-wave polarisation); false before any sample.
    pub fn is_p_like(&self, t: &DetectorTuning) -> bool {
        let k = t.zh_min as f32;
        self.n > 0 && self.e_z > k * k * self.e_h
    }

    /// Clears the accumulators (keeps the window length).
    pub fn reset(&mut self) {
        *self = ZhCheck {
            n_window: self.n_window,
            ..ZhCheck::default()
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    fn tuning() -> DetectorTuning {
        presets::door().control.seismic.detector
    }

    // req: FAB-SEI-005
    #[test]
    fn alert_levels() {
        let t = tuning();
        assert_eq!((t.tauc_l3_s, t.pd_l3_m), (0.6, 0.005), "core design §C.4");
        let cm = 0.01_f32;
        // The four cells of the calibration matrix, at and around the (inclusive) edges.
        assert_eq!(
            level(0.5 * cm, 0.6, &t),
            SeismicLevel::L3,
            "τc ≥ 0.6 s and Pd ≥ 0.5 cm → L3"
        );
        assert_eq!(level(2.0 * cm, 2.3, &t), SeismicLevel::L3);
        assert_eq!(level(0.49 * cm, 0.6, &t), SeismicLevel::L2, "τc only → L2");
        assert_eq!(level(0.0, 1.5, &t), SeismicLevel::L2);
        assert_eq!(level(0.5 * cm, 0.59, &t), SeismicLevel::L1, "Pd only → L1");
        assert_eq!(level(0.49 * cm, 0.59, &t), SeismicLevel::L0, "neither → L0");
        assert_eq!(level(0.0, 0.0, &t), SeismicLevel::L0);
        // Non-finite estimates never raise a level.
        assert_eq!(level(f32::NAN, f32::NAN, &t), SeismicLevel::L0);
        assert_eq!(level(f32::NAN, 0.7, &t), SeismicLevel::L2);
        // Wu–Zhao M7 R20 anchor Pd 0.7118 cm with a long τc is L3 (SEI-012 anchor).
        assert_eq!(level(0.7118 * cm, 2.0, &t), SeismicLevel::L3);
        // M5 R10: Pd 0.0643 cm, τc ≈ 0.58 s (Wu & Kanamori 2008 τc–M) → L0 (freeze only).
        assert_eq!(level(0.0643 * cm, 0.58, &t), SeismicLevel::L0);
    }

    // req: FAB-SEI-005
    #[test]
    fn protect_rule() {
        let t = tuning();
        assert_eq!(t.protect_level, 2, "plan §2b item 6");
        let cm = 0.01_f32;
        // Level ≥ 2.
        assert!(protect(SeismicLevel::L2, 0.0, 0.0, &t), "L2 → PROTECT");
        assert!(protect(SeismicLevel::L3, 0.0, 0.0, &t), "L3 → PROTECT");
        // Level 1 means freeze only.
        assert!(
            !protect(SeismicLevel::L1, 0.3 * cm, 0.7, &t),
            "L1 alone: freeze only"
        );
        assert!(!protect(SeismicLevel::L0, 0.0, 0.0, &t));
        // Pd ≥ 0.35 cm at any level (inclusive: the threshold itself triggers PROTECT, the next
        // f32 below it does not).
        let pd_thr = t.pd_alert_m as f32;
        assert_eq!(t.pd_alert_m, 0.0035, "0.35 cm (plan §2b item 6)");
        let below = |x: f32| f32::from_bits(x.to_bits() - 1);
        assert!(
            protect(SeismicLevel::L0, pd_thr, 0.0, &t),
            "Pd ≥ 0.35 cm → PROTECT"
        );
        assert!(
            protect(SeismicLevel::L1, 0.7118 * cm, 0.0, &t),
            "Wu–Zhao M7 R20 Pd"
        );
        assert!(!protect(SeismicLevel::L0, below(pd_thr), 0.0, &t));
        // PGA ≥ 80 gal at any level (inclusive).
        let pga_thr = t.pga_alert_mps2 as f32;
        assert!(
            protect(SeismicLevel::L0, 0.0, pga_thr, &t),
            "PGA ≥ 80 gal → PROTECT"
        );
        assert!(!protect(SeismicLevel::L0, 0.0, below(pga_thr), &t));
        assert!(
            !protect(SeismicLevel::L0, f32::NAN, f32::NAN, &t),
            "NaN never triggers PROTECT alone"
        );
    }

    // req: FAB-SEI-006
    #[test]
    fn veto_bypass_pga() {
        let t = tuning();
        assert_eq!(t.pga_veto_mps2, 0.8, "80 gal (core design §C.4)");
        assert!(!vetoed(false, 0.0, &t), "no veto requested");
        assert!(!vetoed(false, 5.0, &t));
        assert!(
            vetoed(true, 0.0, &t),
            "self-actuation veto suppresses the trigger"
        );
        assert!(vetoed(true, 0.79, &t), "below 80 gal the veto holds");
        assert!(
            !vetoed(true, 0.8, &t),
            "PGA ≥ 80 gal bypasses the veto (inclusive)"
        );
        assert!(!vetoed(true, 9.81, &t));
        assert!(vetoed(true, f32::NAN, &t), "an unknown PGA does not bypass");
    }

    #[test]
    fn zh_polarisation_window() {
        let t = tuning();
        let mut zh = ZhCheck::new(&t, 100.0);
        assert_eq!(zh.n_window, 50, "0.5 s at 100 Hz");
        assert!(!zh.is_p_like(&t), "no samples → not P-like");
        // P at 20° incidence: H/Z = tan 20°.
        let h = 20.0_f32.to_radians().tan();
        for k in 0..80 {
            let s = (k as f32 * 0.37).sin();
            zh.push([h * s * 0.6, h * s * 0.8, s]);
        }
        assert!(zh.done() && zh.n == 50, "accumulation stops at the window");
        assert!(zh.is_p_like(&t), "Z/H ≈ 2.7 > 1");
        zh.reset();
        assert_eq!((zh.n, zh.n_window), (0, 50));
        for k in 0..50 {
            let s = (k as f32 * 0.37).sin();
            zh.push([s, 0.5 * s, 0.3 * s]); // S-like / slam: horizontal-dominated
        }
        assert!(!zh.is_p_like(&t), "Z/H 0.27 < 1");
    }
}
