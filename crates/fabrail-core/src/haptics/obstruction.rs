//! Residual obstruction detection with persistence; stop, reverse or release; seismic threshold degradation to ObstructionDegraded (PHYSICS §17). Owner: A2.
//!
//! Frozen API (W0): [`ObstructionDetector::new`], [`ObstructionDetector::step`] →
//! [`ObstructionOut`]. Residual threshold th0 12 / 30 / 60 / 120 N with persistence
//! 20 / 30 / 35 / 35 ms (SAF-006); the residual triggers STOP, then REVERSE 50 mm
//! (DOOR/WALL) or release to 0 N (DRAWER/HEAVY). During shaking the residual is
//! corrected for the floor's inertial force and the threshold th = th0 + 0.1·m̂·|a_floor|;
//! above 2·th0 the detector reports `degraded` (ObstructionDegraded → SeatAndHold, no
//! automatic motion; plan §4b). Test: `residual_persistence`.
//!
//! **Residual** (PHYSICS §17; core design §C.4). The momentum observer's F̂ estimates the
//! external force m̂·ẍ − F_motor + b̂·ẋ − F̂_rip, i.e. the residual r = F_motor − m̂·a −
//! b̂·v − F̂_rip with the opposite sign, without differentiating velocity. On a shaking
//! floor the encoder measures x relative to the rail, so the body equation carries the
//! inertial term −m·a_floor,x (plant body model): F̂ = F_ext − m·a_floor,x. The
//! detector uses the contact force r = F̂ + m̂·a_floor,x, with a_floor,x from the rail
//! accelerometer (plan §4b writes the same correction as r = F̂ − m̂·a_floor,x for the
//! opposite sign of the floor term; the physics is identical).
//!
//! **Decision.** |r| > th for the persistence time (integer fast releases, continuous)
//! latches `detected` until [`ObstructionDetector::reset`] or disarming. A non-finite
//! residual counts as above the threshold (the conservative side: an estimator fault
//! while moving under power stops the motion). The reaction (STOP, then REVERSE or
//! release) is `fast::FastLoop`'s.

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::fmath;

/// Obstruction detector output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ObstructionOut {
    /// Obstruction confirmed (persistence met).
    pub detected: bool,
    /// Threshold degraded beyond `degraded_ratio`·th0.
    pub degraded: bool,
    /// Residual used, N.
    pub residual_n: f32,
    /// Threshold used, N.
    pub threshold_n: f32,
}

/// Residual obstruction detector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ObstructionDetector {
    /// th0, N.
    pub th0_n: f32,
    /// Persistence, s.
    pub persist_s: f32,
    /// Time above threshold, s.
    pub t_above_s: f32,
    /// Shaking threshold gain (th = th0 + gain·m̂·|a_floor|; 0.1).
    pub floor_gain: f32,
    /// Degradation ratio (2).
    pub degraded_ratio: f32,
    /// Fast period, s.
    pub dt_s: f32,
    /// Persistence, releases.
    pub persist_ticks: u32,
    /// Consecutive releases above the threshold.
    pub above_ticks: u32,
    /// Latched detection.
    pub detected: bool,
}

impl ObstructionDetector {
    /// Detector from `haptic.obstruction` (never fails).
    pub fn new(p: &SystemParams) -> Result<ObstructionDetector, ParamError> {
        let o = &p.control.haptic.obstruction;
        let dt = fabrail_params::derive::fast_period_s(p);
        let persist_ticks = if dt > 0.0 && o.persist_s > 0.0 {
            let n = fmath::round(o.persist_s / dt);
            if n >= f64::from(u32::MAX) {
                u32::MAX
            } else {
                (n as u32).max(1)
            }
        } else {
            1
        };
        Ok(ObstructionDetector {
            th0_n: o.residual_n as f32,
            persist_s: o.persist_s as f32,
            t_above_s: 0.0,
            floor_gain: o.floor_gain as f32,
            degraded_ratio: o.degraded_ratio as f32,
            dt_s: dt as f32,
            persist_ticks,
            above_ticks: 0,
            detected: false,
        })
    }

    /// One fast release from F̂ (N), the rail floor x-acceleration (m/s²) and m̂ (kg).
    /// `armed` is true while the carriage moves under power (or an unattended close);
    /// disarming clears the persistence and the latch.
    pub fn step(
        &mut self,
        f_hat_n: f32,
        a_floor_x_mps2: f32,
        m_hat_kg: f32,
        armed: bool,
    ) -> ObstructionOut {
        let a = if a_floor_x_mps2.is_finite() {
            a_floor_x_mps2
        } else {
            0.0
        };
        let m = if m_hat_kg.is_finite() && m_hat_kg > 0.0 {
            m_hat_kg
        } else {
            0.0
        };
        let r = f_hat_n + m * a;
        let th = self.th0_n + fmath::fmaxf(self.floor_gain, 0.0) * m * fmath::absf(a);
        let degraded = th > self.degraded_ratio * self.th0_n;
        if !armed {
            self.reset();
        } else {
            let above = !r.is_finite() || fmath::absf(r) > th;
            self.above_ticks = if above {
                self.above_ticks.saturating_add(1)
            } else {
                0
            };
            if self.above_ticks >= self.persist_ticks {
                self.detected = true;
            }
        }
        self.t_above_s = self.above_ticks as f32 * self.dt_s;
        ObstructionOut {
            detected: self.detected,
            degraded,
            residual_n: r,
            threshold_n: th,
        }
    }

    /// Clears the persistence timer and the latch.
    pub fn reset(&mut self) {
        self.t_above_s = 0.0;
        self.above_ticks = 0;
        self.detected = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    /// Feeds a constant residual for `n` releases; returns the release (1-based) at which
    /// the detector latched, if any.
    fn latch_at(d: &mut ObstructionDetector, f: f32, a: f32, m: f32, n: u32) -> Option<u32> {
        (1..=n).find(|_| d.step(f, a, m, true).detected)
    }

    /// Per class: a residual above th0 (12 / 30 / 60 / 120 N) is confirmed after exactly
    /// the persistence (20 / 30 / 35 / 35 ms) and not a release earlier; interrupted or
    /// sub-threshold residuals never trigger; disarming clears; the floor's inertial force
    /// is compensated and the shaking threshold th0 + 0.1·m̂·|a_floor| degrades beyond
    /// 2·th0. (T) SAF-006, plan §4b, PHYSICS §17.
    // req: FAB-SAF-006
    #[test]
    fn residual_persistence() {
        let table = [
            (presets::drawer(), 12.0, 0.020),
            (presets::door(), 30.0, 0.030),
            (presets::wall(), 60.0, 0.035),
            (presets::heavy(), 120.0, 0.035),
        ];
        for (p, th0, persist) in table {
            let class = p.design.class;
            let mut d = ObstructionDetector::new(&p).unwrap_or_default();
            assert_eq!(d.th0_n, th0, "{class:?} th0 (SAF-006)");
            let dt = f64::from(d.dt_s);
            let n_p = (persist / dt).round() as u32;
            assert_eq!(d.persist_ticks, n_p, "{class:?} persistence ticks");
            let m = p.design.carriage.m_rated_kg as f32;
            // Exactly the persistence, both signs.
            for f in [1.5 * th0, -1.5 * th0] {
                d.reset();
                assert_eq!(
                    latch_at(&mut d, f, 0.0, m, 3 * n_p),
                    Some(n_p),
                    "{class:?} F̂ = {f}"
                );
                // Latched while armed, even after the residual vanishes.
                assert!(d.step(0.0, 0.0, m, true).detected);
            }
            // One release short of the persistence, then below: never.
            d.reset();
            assert_eq!(latch_at(&mut d, 1.5 * th0, 0.0, m, n_p - 1), None);
            assert_eq!(latch_at(&mut d, 0.5 * th0, 0.0, m, 10 * n_p), None);
            // Interrupted above-threshold spells never add up.
            d.reset();
            for _ in 0..5 {
                assert_eq!(latch_at(&mut d, 2.0 * th0, 0.0, m, n_p - 1), None);
                assert_eq!(latch_at(&mut d, 0.0, 0.0, m, 1), None);
            }
            // Sub-threshold forever: never.
            d.reset();
            assert_eq!(latch_at(&mut d, 0.95 * th0, 0.0, m, 20 * n_p), None);
            // Disarmed: never, and disarming clears the latch.
            d.reset();
            assert_eq!(latch_at(&mut d, 3.0 * th0, 0.0, m, 2 * n_p), Some(n_p));
            let o = d.step(3.0 * th0, 0.0, m, false);
            assert!(!o.detected && !d.detected);
            for _ in 0..(3 * n_p) {
                assert!(!d.step(3.0 * th0, 0.0, m, false).detected);
            }
            // Shaking floor: the observer sees F̂ = −m̂·a_floor; the residual is 0.
            let th_deg = th0 / (0.1 * m); // |a_floor| at which th = 2·th0
            for a in [0.5 * th_deg, 0.9 * th_deg, 1.5 * th_deg] {
                d.reset();
                assert_eq!(
                    latch_at(&mut d, -m * a, a, m, 5 * n_p),
                    None,
                    "{class:?} floor {a}"
                );
                let o = d.step(-m * a, a, m, true);
                assert!(o.residual_n.abs() < 1e-3 * th0);
                let th = th0 + 0.1 * m * a;
                assert!(
                    (o.threshold_n - th).abs() <= 1e-4 * th,
                    "threshold {} vs {th}",
                    o.threshold_n
                );
                assert_eq!(
                    o.degraded,
                    a > th_deg,
                    "{class:?}: degraded at a = {a} (plan §4b)"
                );
                // A real obstruction on top of the floor load is still detected.
                d.reset();
                assert!(latch_at(&mut d, -m * a + 1.2 * th, a, m, 2 * n_p).is_some());
            }
            // A NaN estimate while armed is treated as an obstruction.
            d.reset();
            assert_eq!(latch_at(&mut d, f32::NAN, 0.0, m, 2 * n_p), Some(n_p));
        }
    }
}
