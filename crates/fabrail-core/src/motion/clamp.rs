//! Directional force clamp: drive <= F_drive_max for <= 0.75 s, stall <= F_static, < 80 N within 5 s, braking unclamped (PHYSICS §16; PRP-004). Owner: A2.
//!
//! Frozen API (W0): [`DirectionalClamp::new`], [`DirectionalClamp::step`] →
//! [`ClampOut`]. Driving force (same sign as velocity or from standstill) ≤ F_drive_max
//! for ≤ 0.75 s; stall (|v| < 2 mm/s for > 0.2 s) ≤ F_static (67 N unsensed, 150 N
//! sensed or in the last 50 mm; F4); < 80 N within 5 s; braking force unclamped.
//! Test: `directional_clamp_props` (proptest).
//!
//! Rules per fast release (PHYSICS §16; core design §C.4; EN 16005 (2011 draft) and
//! A156.19 numeric limits, checked in simulation only):
//!
//! * **Braking** (|v| ≥ v_stall and F·v < 0) passes unchanged: stopping is never
//!   limited.
//! * **Driving** (F·v > 0, or |v| < v_stall) is limited to F_drive = min(live
//!   `LiveLimits::f_drive_n`, `safety.f_drive_max_n`) while the dynamic allowance
//!   lasts: every release that drives above the static level F_s consumes one tick of
//!   the 0.75 s allowance (`safety.t_drive_max_s`), every release at or below F_s
//!   returns one (a leaky duty counter, integer ticks). Once spent, driving is limited to
//!   F_s.
//! * **Static level** F_s = live `f_static_n` (67 N unsensed, 150 N with PL d
//!   presence), or `f_close_last50_n` (150 N) for a closing force in the last 50 mm;
//!   never above the larger of the params' sensed and last-zone values, never above
//!   F_drive.
//! * **Stall** (|v| < v_stall for more than t_stall = 0.2 s): the force is limited to
//!   F_s, and once the stall has lasted `safety.t_static_final_s` (5 s) to
//!   0.95·`f_static_final_n` (76 N, i.e. strictly below 80 N; the 5 % margin is a design
//!   choice covering thrust-model error).
//!
//! The clamp never adds force and never flips its sign. Default `LiveLimits` (all
//! zero) allow braking only.

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::fmath;

use crate::msg::LiveLimits;

/// Final static force as a fraction of `f_static_final_n` ("< 80 N"; design choice).
const FINAL_STATIC_FRAC: f32 = 0.95;

/// Clamp result.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ClampOut {
    /// Clamped force, N.
    pub f_n: f32,
    /// The clamp limited the force.
    pub saturated: bool,
    /// Stall detected (|v| < stall_v for > stall_t).
    pub stalled: bool,
}

/// Directional force clamp with its dynamic/static timers.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DirectionalClamp {
    /// Dynamic driving allowance used, s (= `drive_ticks`·T).
    pub t_drive_s: f32,
    /// Time stalled, s (= `stall_ticks`·T).
    pub t_stall_s: f32,
    /// Stall speed threshold, m/s.
    pub stall_v_mps: f32,
    /// Stall time threshold, s.
    pub stall_t_s: f32,
    /// Fast period, s.
    pub dt_s: f32,
    /// Params cap on the driving force F_drive_max, N.
    pub f_drive_max_n: f32,
    /// Params cap on any static level (max of sensed and last-zone), N.
    pub f_static_cap_n: f32,
    /// Final static force (< 80 N), N.
    pub f_static_final_n: f32,
    /// Dynamic allowance in ticks (0.75 s).
    pub drive_max_ticks: u32,
    /// Stall detection in ticks (0.2 s).
    pub stall_min_ticks: u32,
    /// Stall duration after which the final static force applies (5 s), ticks.
    pub final_ticks: u32,
    /// Dynamic allowance used, ticks.
    pub drive_ticks: u32,
    /// Consecutive standstill releases, ticks.
    pub stall_ticks: u32,
}

/// `t/dt` rounded to whole ticks (≥ 0), saturating.
fn ticks(t_s: f64, dt_s: f64) -> u32 {
    if dt_s > 0.0 && t_s > 0.0 {
        let n = fmath::round(t_s / dt_s);
        if n >= f64::from(u32::MAX) {
            u32::MAX
        } else {
            n as u32
        }
    } else {
        0
    }
}

impl DirectionalClamp {
    /// Clamp from the safety limits and the supervisor stall tuning (never fails).
    pub fn new(p: &SystemParams) -> Result<DirectionalClamp, ParamError> {
        let sf = &p.safety;
        let sv = &p.control.supervisor;
        let dt = fabrail_params::derive::fast_period_s(p);
        Ok(DirectionalClamp {
            t_drive_s: 0.0,
            t_stall_s: 0.0,
            stall_v_mps: sv.stall_v_mps as f32,
            stall_t_s: sv.stall_t_s as f32,
            dt_s: dt as f32,
            f_drive_max_n: sf.f_drive_max_n as f32,
            f_static_cap_n: fmath::fmax(sf.f_static_sensed_n, sf.f_close_last50_n) as f32,
            f_static_final_n: FINAL_STATIC_FRAC * sf.f_static_final_n as f32,
            drive_max_ticks: ticks(sf.t_drive_max_s, dt),
            stall_min_ticks: ticks(sv.stall_t_s, dt),
            final_ticks: ticks(sf.t_static_final_s, dt),
            drive_ticks: 0,
            stall_ticks: 0,
        })
    }

    /// Static level F_s for a force of sign `f` under `limits` (module docs), N.
    fn static_level(&self, f: f32, limits: &LiveLimits, in_last_zone: bool, f_drive: f32) -> f32 {
        let base = if in_last_zone && f < 0.0 {
            fmath::fmaxf(limits.f_static_n, limits.f_close_last50_n)
        } else {
            limits.f_static_n
        };
        let mut s = fmath::fminf(fmath::fmaxf(base, 0.0), self.f_static_cap_n);
        if self.stall_ticks >= self.final_ticks {
            s = fmath::fminf(s, self.f_static_final_n);
        }
        fmath::fminf(s, f_drive)
    }

    /// One fast release: clamps the commanded force `f_cmd_n` given velocity `v_mps`,
    /// the live limits and whether the carriage is within the last closing zone.
    pub fn step(
        &mut self,
        f_cmd_n: f32,
        v_mps: f32,
        limits: &LiveLimits,
        in_last_zone: bool,
    ) -> ClampOut {
        // Standstill bookkeeping (integer ticks, deterministic); a NaN speed counts as
        // standstill (the conservative side: stall limits apply).
        let av = fmath::absf(v_mps);
        let still = av.is_nan() || av < self.stall_v_mps;
        self.stall_ticks = if still {
            self.stall_ticks.saturating_add(1)
        } else {
            0
        };
        let stalled = still && self.stall_ticks > self.stall_min_ticks;
        let f = if f_cmd_n.is_finite() { f_cmd_n } else { 0.0 };
        let braking = !still && f * v_mps < 0.0;
        let out = if braking {
            // Braking is never clamped.
            self.drive_ticks = self.drive_ticks.saturating_sub(1);
            f
        } else {
            let f_drive = fmath::fmaxf(fmath::fminf(limits.f_drive_n, self.f_drive_max_n), 0.0);
            let f_static = self.static_level(f, limits, in_last_zone, f_drive);
            let lim = if stalled || self.drive_ticks >= self.drive_max_ticks {
                f_static
            } else {
                f_drive
            };
            let o = fmath::clampf(f, -lim, lim);
            if fmath::absf(o) > f_static {
                self.drive_ticks = self.drive_ticks.saturating_add(1);
            } else {
                self.drive_ticks = self.drive_ticks.saturating_sub(1);
            }
            o
        };
        self.t_drive_s = self.drive_ticks as f32 * self.dt_s;
        self.t_stall_s = self.stall_ticks as f32 * self.dt_s;
        ClampOut {
            f_n: out,
            saturated: out != f_cmd_n,
            stalled,
        }
    }

    /// Resets the timers.
    pub fn reset(&mut self) {
        self.t_drive_s = 0.0;
        self.t_stall_s = 0.0;
        self.drive_ticks = 0;
        self.stall_ticks = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;
    use proptest::prelude::*;

    fn limits(p: &SystemParams, sensed: bool) -> LiveLimits {
        let sf = &p.safety;
        LiveLimits {
            v_open_mps: sf.v_open_max_mps as f32,
            v_close_mps: sf.v_close_max_mps as f32,
            a_max_mps2: p.control.motion.a_max_mps2 as f32,
            f_drive_n: sf.f_drive_max_n as f32,
            f_static_n: if sensed {
                sf.f_static_sensed_n
            } else {
                sf.f_static_n
            } as f32,
            f_close_last50_n: sf.f_close_last50_n as f32,
            ..LiveLimits::default()
        }
    }

    /// One constant-input run: (force command, velocity, last zone, ticks).
    fn run_strategy() -> impl Strategy<Value = Vec<(f32, f32, bool, u32)>> {
        prop::collection::vec(
            (
                -500.0f32..500.0,
                prop_oneof![Just(0.0f32), -0.0019f32..0.0019, -0.3f32..0.3],
                any::<bool>(),
                1u32..20_000,
            ),
            1..8,
        )
    }

    // req: FAB-PRP-004
    proptest! {
        #![proptest_config(ProptestConfig::with_cases(48))]
        /// Under random command/velocity histories the clamp output never exceeds the
        /// command or flips its sign, passes braking unchanged, keeps driving ≤ F_drive,
        /// uses the dynamic allowance for ≤ 0.75 s at a stretch, holds stall force ≤ 67 N
        /// unsensed (150 N sensed or closing in the last 50 mm) and < 80 N after 5 s of
        /// stall. (S) EN 16005 (2011 draft) / A156.19 numeric limits; F4; PRP-004.
        #[test]
        fn directional_clamp_props(
            class in 0usize..4,
            sensed in any::<bool>(),
            runs in run_strategy(),
        ) {
            let p = [presets::drawer(), presets::door(), presets::wall(), presets::heavy()][class];
            let sf = p.safety;
            let lim = limits(&p, sensed);
            let mut c = DirectionalClamp::new(&p).unwrap_or_default();
            let dt = f64::from(c.dt_s);
            let stall_v = f64::from(c.stall_v_mps);
            let f_static_live = if sensed { sf.f_static_sensed_n } else { sf.f_static_n };
            let max_drive_run = (sf.t_drive_max_s / dt).round() as u64 + 1;
            let (mut still_ticks, mut drive_run) = (0u64, 0u64);
            for &(f, v, zone, n) in &runs {
                for _ in 0..n {
                    let o = c.step(f, v, &lim, zone);
                    let (fo, fc, vv) = (f64::from(o.f_n), f64::from(f), f64::from(v));
                    prop_assert!(fo.abs() <= fc.abs(), "never adds force");
                    prop_assert!(fo * fc >= 0.0, "never flips sign");
                    let still = vv.abs() < stall_v;
                    still_ticks = if still { still_ticks + 1 } else { 0 };
                    if !still && fc * vv < 0.0 {
                        prop_assert_eq!(fo, fc, "braking is unclamped (PRP-004)");
                        drive_run = 0;
                        continue;
                    }
                    prop_assert!(fo.abs() <= sf.f_drive_max_n + 1e-9,
                        "drive {} > F_drive_max {} (PRP-004)", fo, sf.f_drive_max_n);
                    let f_s = if zone && fc < 0.0 {
                        f_static_live.max(sf.f_close_last50_n)
                    } else {
                        f_static_live
                    };
                    drive_run = if fo.abs() > f_s { drive_run + 1 } else { 0 };
                    prop_assert!(drive_run <= max_drive_run,
                        "dynamic force above F_static for {} ticks > 0.75 s (PRP-004)", drive_run);
                    let t_still = still_ticks as f64 * dt;
                    if t_still > sf.t_static_final_s + dt {
                        prop_assert!(fo.abs() < sf.f_static_final_n,
                            "stall force {} not < 80 N after 5 s (SAF-004, PRP-004)", fo);
                    }
                    if t_still > f64::from(c.stall_t_s) + dt {
                        prop_assert!(o.stalled);
                        prop_assert!(fo.abs() <= f_s + 1e-9,
                            "stall force {} > F_static {} (F4, PRP-004)", fo, f_s);
                        let closing_in_zone = zone && fc < 0.0;
                        if !sensed && !closing_in_zone {
                            prop_assert!(fo.abs() <= 67.0 + 1e-9, "unsensed stall > 67 N (F4)");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn default_limits_allow_braking_only() {
        let mut c = DirectionalClamp::new(&presets::door()).unwrap_or_default();
        let l = LiveLimits::default();
        assert_eq!(c.step(50.0, 0.1, &l, false).f_n, 0.0);
        assert_eq!(c.step(-50.0, 0.1, &l, false).f_n, -50.0);
        assert_eq!(c.step(f32::NAN, 0.0, &l, false).f_n, 0.0);
    }

    #[test]
    fn dynamic_allowance_expires() {
        let p = presets::door();
        let mut c = DirectionalClamp::new(&p).unwrap_or_default();
        let l = limits(&p, false);
        // Driving at 0.1 m/s with 150 N: allowed for 0.75 s, then 67 N.
        let n = (0.75 / f64::from(c.dt_s)).round() as usize;
        for _ in 0..n {
            assert_eq!(c.step(150.0, 0.1, &l, false).f_n, 150.0);
        }
        assert_eq!(c.step(150.0, 0.1, &l, false).f_n, 67.0);
    }
}
