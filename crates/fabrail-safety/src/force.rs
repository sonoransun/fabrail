//! Directional force clamp mirror and the EN 16005 dynamic/static force-time timers (SAF-004). Owner: A5.
//!
//! The force check mirrors the core's directional clamp (PHYSICS §16) on the monitor's
//! own data: measured LSM phase-current codes (never the core's commands) against code²
//! thresholds derived in `Monitor::new` from the diverse copies K_f and A/code (plan
//! §2b item 17). [`i_sq_code`] = Σ i_abc² is the integer force proxy: with the
//! amplitude-invariant Clarke/Park transform (PHYSICS §7) a balanced set of peak I has
//! Σ i_abc² = 1.5·I² at every electrical angle and |i_q| ≤ |i_dq| = I, so
//! |F| = K_f·|i_q| ≤ K_f·√(⅔·Σ i_abc²): a cap F maps to Σ i² ≤ 1.5·(F/(K_f·A_code))²
//! (conservative when i_d ≠ 0).
//!
//! **Force-time profile** ("checked in simulation against numeric limits from EN 16005
//! (2011 draft)", never a compliance claim; SAF-004, core design §C.2 F4). A force
//! *episode* starts when the measured force rises above the lowest static level in force
//! (67 N unsensed; min(150, 80) = 80 N sensed or in the last 50 mm of closing) and ends
//! when it falls to half that level (hysteresis, so a force held at a lowered cap does
//! not re-arm the dynamic window). Within an episode the cap in force is
//! * F_drive_max for the first `drive_window` (0.75 s),
//! * then F_static (67 N unsensed; 150 N when sensed or closing in the last 50 mm),
//! * then after `final_window` (5 s) the final static force min(F_static, 80 N).
//!
//! Braking (the core commands a force opposing v, |v| above standstill) is unclamped in
//! the core (PHYSICS §16); the monitor mirrors it with the `Brake` cap K_f·i_lsm_max
//! (the inverter current limit) and does not run an episode. The cap in force goes to
//! the arbitration (`SafetyVerdict::force_cap_n`); ForceLimit is raised when the
//! measured force exceeds the cap by the monitoring tolerance (1.1×, `crate::tol_sq`),
//! within one fast period (no persistence; SAF-012), except for `grace` ticks after the
//! cap is lowered while the current follows the new limit.
#![deny(clippy::float_arithmetic)]

use fabrail_types::Tick;

use crate::tol_sq;

/// Σ i_abc² in code² (exact, no overflow: 3·32768² < 2^32).
#[inline]
pub const fn i_sq_code(i_abc: [i16; 3]) -> u64 {
    let [a, b, c] = i_abc;
    let (a, b, c) = (a as i64, b as i64, c as i64);
    (a * a + b * b + c * c) as u64
}

/// Which force cap is in force (the monitor maps it to the f32 cap in newtons it
/// precomputed in `new()`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ForceCap {
    /// Dynamic driving force F_drive_max (≤ 0.75 s).
    #[default]
    Drive = 0,
    /// Static unsensed (67 N).
    Static = 1,
    /// Static sensed or last 50 mm (150 N).
    StaticSensed = 2,
    /// Final static (< 80 N after 5 s).
    StaticFinal = 3,
    /// Braking: unclamped up to the inverter current limit K_f·i_lsm_max.
    Brake = 4,
    /// ISOLATE continuous force F_cont (SAF-020).
    Isolate = 5,
}

/// Force-timer output.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ForceOut {
    /// Cap in force for the next arbitration.
    pub cap: ForceCap,
    /// ForceLimit fault (measured force above the cap in force).
    pub fault: bool,
}

/// Dynamic/static force-time supervision on integer current codes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ForceTimer {
    /// F_drive_max as Σ i² code² threshold.
    pub drive_sq: u64,
    /// F_static (unsensed) threshold.
    pub static_sq: u64,
    /// F_static (sensed / last zone) threshold.
    pub static_sensed_sq: u64,
    /// F_static_final threshold.
    pub static_final_sq: u64,
    /// Braking threshold (inverter limit K_f·i_lsm_max).
    pub brake_sq: u64,
    /// Dynamic window, base ticks (0.75 s).
    pub drive_window_ticks: u32,
    /// Time to the final static force, base ticks (5 s).
    pub final_window_ticks: u32,
    /// Grace after a lowered cap before the new cap is enforced, base ticks.
    pub grace_ticks: u32,
    /// Tick at which the current force episode began, if one is running.
    pub episode_since: Option<Tick>,
    /// Cap in force after the last step.
    pub cap: ForceCap,
    /// The higher cap still accepted during the grace after a lowering.
    pub grace_cap: ForceCap,
    /// Tick at which the cap was last lowered.
    pub lowered_at: Option<Tick>,
}

impl ForceTimer {
    /// Threshold of a cap in code².
    #[inline]
    pub const fn cap_sq(&self, c: ForceCap) -> u64 {
        match c {
            ForceCap::Drive => self.drive_sq,
            ForceCap::Static => self.static_sq,
            ForceCap::StaticSensed => self.static_sensed_sq,
            ForceCap::StaticFinal => self.static_final_sq,
            ForceCap::Brake => self.brake_sq,
            // Not selected by the timer (the isolate envelope owns it).
            ForceCap::Isolate => self.drive_sq,
        }
    }

    /// One check at tick `t` from the measured Σ i_abc² (code²), whether the core is
    /// braking, and whether the higher static cap applies (sensed operation, or closing
    /// within the last zone).
    pub fn step(&mut self, t: Tick, i_sq: u64, braking: bool, high_static: bool) -> ForceOut {
        let static_cap = if high_static {
            ForceCap::StaticSensed
        } else {
            ForceCap::Static
        };
        // The final static force never raises the cap (min(67, 80) = 67 unsensed).
        let final_cap = if self.static_final_sq < self.cap_sq(static_cap) {
            ForceCap::StaticFinal
        } else {
            static_cap
        };
        let low_sq = self.cap_sq(final_cap);
        if braking {
            self.episode_since = None;
        } else if i_sq > low_sq {
            if self.episode_since.is_none() {
                self.episode_since = Some(t);
            }
        } else if i_sq <= low_sq / 4 {
            // Force ≤ half the lowest static level (Σ i² ∝ F²).
            self.episode_since = None;
        }
        let cap = if braking {
            ForceCap::Brake
        } else {
            match self.episode_since {
                None => ForceCap::Drive,
                Some(t0) => {
                    let age = t.since(t0);
                    if age <= self.drive_window_ticks as u64 {
                        ForceCap::Drive
                    } else if age <= self.final_window_ticks as u64 {
                        static_cap
                    } else {
                        final_cap
                    }
                }
            }
        };
        if self.cap_sq(cap) < self.cap_sq(self.cap) {
            // Lowered: keep accepting the previous (higher) cap during the grace.
            let in_grace = self
                .lowered_at
                .is_some_and(|l| t.since(l) <= self.grace_ticks as u64);
            if !in_grace {
                self.grace_cap = self.cap;
            }
            self.lowered_at = Some(t);
        }
        self.cap = cap;
        let in_grace = self
            .lowered_at
            .is_some_and(|l| t.since(l) <= self.grace_ticks as u64);
        let check_sq = if in_grace {
            let g = self.cap_sq(self.grace_cap);
            if g > self.cap_sq(cap) {
                g
            } else {
                self.cap_sq(cap)
            }
        } else {
            self.cap_sq(cap)
        };
        ForceOut {
            cap,
            fault: i_sq > tol_sq(check_sq),
        }
    }

    /// Clears the timers.
    pub fn reset(&mut self) {
        self.episode_since = None;
        self.lowered_at = None;
        self.cap = ForceCap::Drive;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Σ i² for a force F with one code per newton-per-K_f (thresholds 1.5·F²).
    const fn sq(f_n: u64) -> u64 {
        3 * f_n * f_n / 2
    }

    /// A timer with the EN 16005 numbers (400 / 67 / 150 / 80 N, 0.75 s / 5 s) and a
    /// 1 ms grace; braking up to 800 N.
    fn en16005() -> ForceTimer {
        ForceTimer {
            drive_sq: sq(400),
            static_sq: sq(67),
            static_sensed_sq: sq(150),
            static_final_sq: sq(80),
            brake_sq: sq(800),
            drive_window_ticks: 15_000,
            final_window_ticks: 100_000,
            grace_ticks: 20,
            ..ForceTimer::default()
        }
    }

    #[test]
    fn current_magnitude_proxy_is_exact() {
        assert_eq!(i_sq_code([3, -4, 1]), 26);
        assert_eq!(i_sq_code([i16::MIN; 3]), 3 * 32_768 * 32_768);
        // Balanced set of peak 1000 codes: Σ i² = 1.5·1000² at any angle (PHYSICS §7).
        let s = i_sq_code([1000, -500, -500]);
        assert_eq!(s, 1_500_000, "(E) 1.5·I² at θ = 0");
    }

    // req: FAB-SAF-004
    #[test]
    fn en16005_timer() {
        // Unsensed stall against an obstruction at 400 N, 10 kHz releases (2 ticks).
        let mut f = en16005();
        let mut t = 0u64;
        let step = |f: &mut ForceTimer, t: u64, force: u64, high: bool| {
            f.step(Tick(t), sq(force), false, high)
        };
        // ≤ 0.75 s: the dynamic cap (400 N) is in force, 400 N is accepted.
        while t <= 15_000 {
            let o = step(&mut f, t, 400, false);
            assert_eq!(
                o.cap,
                ForceCap::Drive,
                "(S) ≤ 400 N for ≤ 0.75 s, EN 16005 (2011 draft)"
            );
            assert!(!o.fault, "at the dynamic cap, no fault at t={t}");
            t += 2;
        }
        // > 0.75 s: the static unsensed cap (67 N) is in force; the current still at
        // 400 N is tolerated for the 1 ms grace, then ForceLimit.
        let o = step(&mut f, t, 400, false);
        assert_eq!(
            o.cap,
            ForceCap::Static,
            "(S) 67 N after 0.75 s unsensed, F4"
        );
        assert!(!o.fault, "grace while the current follows the lowered cap");
        let o = step(&mut f, t + 20, 400, false);
        assert!(!o.fault, "still inside the 1 ms grace");
        let o = step(&mut f, t + 22, 400, false);
        assert!(o.fault, "(S) 400 N after 0.75 s + grace exceeds 67 N · 1.1");
        // Following the cap: 67 N is accepted, 74 N is inside the 1.1 tolerance, 75 N not.
        assert!(!step(&mut f, t + 24, 67, false).fault);
        assert!(!step(&mut f, t + 26, 73, false).fault, "73 N ≤ 1.1·67 N");
        assert!(step(&mut f, t + 28, 75, false).fault, "75 N > 1.1·67 N");
        // The episode continues at the clamped level (67 N > half of 67 N): after 5 s
        // the final static force is min(67, 80) = 67 N (the final cap never raises).
        let o = step(&mut f, 100_004, 67, false);
        assert_eq!(o.cap, ForceCap::Static, "(S) unsensed final stays 67 N");
        // Releasing to ≤ 33.5 N ends the episode: the dynamic cap returns.
        let o = step(&mut f, 100_006, 30, false);
        assert_eq!(
            o.cap,
            ForceCap::Drive,
            "episode ends at ≤ half the static level"
        );

        // Sensed (or closing in the last 50 mm): 400 N ≤ 0.75 s, 150 N, < 80 N by 5 s.
        let mut f = en16005();
        let o = step(&mut f, 0, 400, true);
        assert_eq!(o.cap, ForceCap::Drive);
        let o = step(&mut f, 15_002, 150, true);
        assert_eq!(
            o.cap,
            ForceCap::StaticSensed,
            "(S) ≤ 150 N after 0.75 s, EN 16005"
        );
        assert!(!o.fault, "150 N at the sensed static cap");
        let o = step(&mut f, 50_000, 150, true);
        assert_eq!(o.cap, ForceCap::StaticSensed);
        let o = step(&mut f, 100_002, 150, true);
        assert_eq!(
            o.cap,
            ForceCap::StaticFinal,
            "(S) < 80 N within 5 s, EN 16005"
        );
        assert!(!o.fault, "grace after lowering to 80 N");
        let o = step(&mut f, 100_030, 150, true);
        assert!(o.fault, "(S) 150 N after 5 s + grace exceeds 80 N · 1.1");
        assert!(
            !step(&mut f, 100_032, 80, true).fault,
            "80 N final static accepted"
        );

        // Braking is unclamped (inverter limit): no episode, cap Brake.
        let mut f = en16005();
        for k in 0..20_000u64 {
            let o = f.step(Tick(2 * k), sq(700), true, false);
            assert_eq!(
                o.cap,
                ForceCap::Brake,
                "braking force opposing v is unclamped, PHYSICS §16"
            );
            assert!(!o.fault);
        }
        assert!(
            f.step(Tick(40_002), sq(900), true, false).fault,
            "above the inverter limit"
        );

        // A dynamic peak above F_drive_max trips within one fast period.
        let mut f = en16005();
        assert!(!step(&mut f, 0, 430, false).fault, "430 N ≤ 1.1·400 N");
        assert!(
            step(&mut f, 2, 450, false).fault,
            "(S) 450 N > 1.1·400 N trips at once"
        );
    }
}
