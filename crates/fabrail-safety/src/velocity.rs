//! Integer-domain encoder velocity over a >= 10 ms window and overspeed trip v > 1.1*v_lim for > 10 ms (SAF-008). Owner: A5.
//!
//! API (W0 names; A5 bodies):
//! * [`VelocityWindow`]: tick-stamped encoder counts → µm/s over a window of at least
//!   `window_ticks` base ticks (≥ 10 ms, plan §2b item 17). The reference sample is the
//!   newest one whose age is ≥ the window, so the window never shrinks below its
//!   configured length once enough history exists; older samples are dropped as they
//!   become redundant (amortised O(1) per push, no search).
//! * [`velocity_umps`]: the exact integer conversion
//!   v = Δcounts·nm_per_count·BASE_TICK_HZ / (Δticks·1000) in µm/s, evaluated in i128
//!   (no overflow for any i64 count difference) and rounded half away from zero, so the
//!   result is within ½ µm/s of the exact rational (PHYSICS §13 encoder quantisation;
//!   PHYSICS §18 monitor units).
//! * [`SlsTimer`]: the safely-limited-speed persistence shared by the overspeed and the
//!   energy/momentum checks: "above the limit for more than `persist` ticks", with the
//!   drive-evidence exemption below.
//! * [`Overspeed`]: |v| > 1.1·v_lim in the direction of travel for more than
//!   `persist_ticks` (10 ms) → Overspeed, which the monitor maps to STO + ASC (SAF-008).
//!
//! **Drive evidence: positive drive work (Wave 2 lead decision D4).** Overspeed and KE
//! are drive hazards: the monitor trips them only while the DRIVE does positive work
//! (F·v > 0) beyond the persistence. The monitor's view of the sign of F is the sign of
//! the commanded i_q (read by comparison only) together with its own measurement that
//! the LSM carries current. A persistence interval that expires is excused (and
//! restarted) unless the LSM is energised (measured Σ i_abc² above the activity
//! threshold) AND either (a) the command does not brake (it drives along the velocity,
//! or its sign is unknown: a zero command with current flowing), or (b) the command
//! claims braking but the carriage kept speeding up for longer than
//! [`crate::BRAKE_ACCEL_SUSTAIN_TICKS`] (0.5 s): a commutation sign error makes the
//! "brake" accelerate the carriage, which is positive drive work. A drawer thrown by hand
//! past its cap, or a rail-profile disturbance, is therefore met by governor damping
//! only (the command brakes), and a brief hand push that overpowers the brake is excused,
//! while a runaway, a drive holding an unsafe speed and a persistent sign error trip
//! (core design F7: "faster only when the hand drives it").
//!
//! Integer units: counts (i64), base ticks (u64, 20 kHz), µm/s (i64). The time base is
//! the base tick, so the monitor never needs the fast-group rate. No float arithmetic
//! per tick (lint enforced).
#![deny(clippy::float_arithmetic)]

use fabrail_types::{BASE_TICK_HZ, Tick};

/// History capacity in samples (≥ 10 ms at the 20 kHz worst case is 200 samples; the
/// 10 kHz / 5 kHz fast rates need 100 / 50).
pub const VEL_WINDOW_MAX: usize = 256;

/// Shortest window the monitor accepts, base ticks: 10 ms (plan §2b item 17).
pub const VEL_WINDOW_MIN_TICKS: u32 = BASE_TICK_HZ / 100;

/// Velocity in µm/s from an encoder count difference over `d_ticks` base ticks:
/// Δcounts·nm_per_count·BASE_TICK_HZ / (Δticks·1000), rounded half away from zero,
/// saturated to i64. `d_ticks == 0` gives 0 (PHYSICS §13).
#[inline]
pub fn velocity_umps(d_counts: i128, d_ticks: u64, nm_per_count: i32) -> i64 {
    if d_ticks == 0 {
        return 0;
    }
    // |Δcounts| ≤ 2^64, nm_per_count ≤ 2^31, BASE_TICK_HZ < 2^15: the product stays
    // below 2^110, far inside i128.
    let num = d_counts
        .saturating_mul(nm_per_count as i128)
        .saturating_mul(BASE_TICK_HZ as i128);
    let den = (d_ticks as u128).saturating_mul(1000);
    let mag = num.unsigned_abs().saturating_add(den / 2) / den;
    let mag = if mag > i64::MAX as u128 {
        i64::MAX
    } else {
        mag as i64
    };
    if num < 0 { -mag } else { mag }
}

/// Windowed integer velocity from tick-stamped encoder counts.
#[derive(Clone, Debug)]
pub struct VelocityWindow {
    /// History ring of (tick, counts), oldest at `idx − len`.
    pub buf: [(Tick, i64); VEL_WINDOW_MAX],
    /// Next write index.
    pub idx: u16,
    /// Samples stored (≤ VEL_WINDOW_MAX).
    pub len: u16,
    /// Window length, base ticks.
    pub window_ticks: u32,
    /// Encoder resolution, nm/count (diverse copy).
    pub nm_per_count: i32,
}

impl Default for VelocityWindow {
    fn default() -> VelocityWindow {
        VelocityWindow::new(0, 0)
    }
}

impl VelocityWindow {
    /// A window of `window_ticks` base ticks.
    pub const fn new(window_ticks: u32, nm_per_count: i32) -> VelocityWindow {
        VelocityWindow {
            buf: [(Tick(0), 0); VEL_WINDOW_MAX],
            idx: 0,
            len: 0,
            window_ticks,
            nm_per_count,
        }
    }

    /// Ring index of the `k`-th oldest stored sample.
    #[inline]
    fn slot(&self, k: u16) -> usize {
        (self.idx as usize + VEL_WINDOW_MAX - self.len as usize + k as usize) % VEL_WINDOW_MAX
    }

    /// The `k`-th oldest stored sample.
    #[inline]
    fn sample(&self, k: u16) -> Option<(Tick, i64)> {
        if k >= self.len {
            return None;
        }
        self.buf.get(self.slot(k)).copied()
    }

    /// The newest stored sample.
    #[inline]
    pub fn newest(&self) -> Option<(Tick, i64)> {
        self.len.checked_sub(1).and_then(|k| self.sample(k))
    }

    /// Forgets the history.
    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// Velocity quantum in µm/s: one count over the configured window.
    pub fn quantum_umps(&self) -> i64 {
        velocity_umps(1, self.window_ticks.max(1) as u64, self.nm_per_count).abs()
    }

    /// Pushes the counts sampled at tick `t`; returns the windowed velocity, µm/s.
    ///
    /// A sample that does not advance time (`t` ≤ the newest tick) restarts the history
    /// and returns 0: the base clock is monotonic, so this only happens after a reset.
    pub fn push(&mut self, t: Tick, counts: i64) -> i64 {
        if let Some((t_last, _)) = self.newest()
            && t <= t_last
        {
            self.clear();
        }
        if self.len as usize >= VEL_WINDOW_MAX {
            // Full: the window cannot be longer than the history (documented limit).
            self.len -= 1;
        }
        if let Some(slot) = self.buf.get_mut(self.idx as usize) {
            *slot = (t, counts);
        }
        self.idx = ((self.idx as usize + 1) % VEL_WINDOW_MAX) as u16;
        self.len += 1;
        // Drop the oldest sample while the next one is already old enough to serve as
        // the reference (its age ≥ window).
        let w = self.window_ticks as u64;
        while let Some((t1, _)) = self.sample(1) {
            if t.since(t1) >= w {
                self.len -= 1;
            } else {
                break;
            }
        }
        match self.sample(0) {
            Some((t0, c0)) => {
                velocity_umps(counts as i128 - c0 as i128, t.since(t0), self.nm_per_count)
            }
            None => 0,
        }
    }
}

/// Evidence about who drives the motion, from the monitor's own measurements plus the
/// sign of the core's LSM command (read by comparison only).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DriveEvidence {
    /// Measured Σ i_abc² above the activity threshold: the LSM produces force.
    pub energised: bool,
    /// The core commands a force opposing the measured velocity (|v| above standstill).
    pub cmd_braking: bool,
    /// The core commands a force along the measured velocity (|v| above standstill, LSM
    /// in Run): with `energised`, positive drive work F·v > 0 (D4).
    pub cmd_driving: bool,
}

impl DriveEvidence {
    /// Evidence of positive drive work over an expired interval: the LSM is energised
    /// and the command does not brake (it drives along v, or its sign is unknown: a zero
    /// command with current flowing, a non-Run mode), or it claims braking while the
    /// carriage has kept speeding up for longer than the sustain window
    /// (`brake_accel_sustained`, a sign error). Only an energised LSM with a braking
    /// command on a carriage that is not persistently accelerating is excused as
    /// governor damping (D4).
    #[inline]
    pub const fn positive_work(&self, brake_accel_sustained: bool) -> bool {
        self.energised && (!self.cmd_braking || brake_accel_sustained)
    }
}

/// Safely-limited-speed persistence: trips when a limit is exceeded continuously for
/// more than `persist_ticks`, unless the drive evidence excuses the interval.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SlsTimer {
    /// Tick at which the current above-limit interval started.
    pub since: Option<Tick>,
    /// |v| at the start of the interval, µm/s.
    pub v_start_umps: u64,
    /// Tick since which every expired interval showed a braking command while the
    /// carriage sped up (D4 sign-error evidence).
    pub brake_accel_since: Option<Tick>,
}

impl SlsTimer {
    /// One check at `t`: `over` = the limit is exceeded now; `v_abs_umps` = |v|;
    /// `accel_tol_umps` = the speed increase over an interval that counts as "speeding
    /// up" (noise allowance). Returns true when tripped.
    pub fn step(
        &mut self,
        t: Tick,
        over: bool,
        v_abs_umps: u64,
        persist_ticks: u32,
        accel_tol_umps: u64,
        ev: DriveEvidence,
    ) -> bool {
        if !over {
            self.since = None;
            self.brake_accel_since = None;
            return false;
        }
        let Some(t0) = self.since else {
            self.since = Some(t);
            self.v_start_umps = v_abs_umps;
            return false;
        };
        if t.since(t0) <= persist_ticks as u64 {
            return false;
        }
        let speeding_up = v_abs_umps > self.v_start_umps.saturating_add(accel_tol_umps);
        let sustained = if ev.energised && ev.cmd_braking && speeding_up {
            let s0 = *self.brake_accel_since.get_or_insert(t0);
            t.since(s0) > crate::BRAKE_ACCEL_SUSTAIN_TICKS as u64
        } else {
            self.brake_accel_since = None;
            false
        };
        let excused = !ev.positive_work(sustained);
        if excused {
            // Restart the interval: the next expiry re-examines the evidence.
            self.since = Some(t);
            self.v_start_umps = v_abs_umps;
            false
        } else {
            true
        }
    }

    /// Clears the interval.
    pub fn reset(&mut self) {
        self.since = None;
        self.brake_accel_since = None;
    }
}

/// Directional overspeed persistence check (SAF-008).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Overspeed {
    /// Trip speed opening (+x), 1.1·v_open, µm/s.
    pub open_limit_umps: i64,
    /// Trip speed closing (−x), 1.1·v_close, µm/s (magnitude).
    pub close_limit_umps: i64,
    /// Persistence before tripping, base ticks (10 ms).
    pub persist_ticks: u32,
    /// Speed increase that counts as speeding up over one interval, µm/s.
    pub accel_tol_umps: u64,
    /// Persistence state.
    pub timer: SlsTimer,
}

impl Overspeed {
    /// Check with directional limits (µm/s) and persistence (base ticks).
    pub const fn new(open_limit_umps: i64, close_limit_umps: i64, persist_ticks: u32) -> Overspeed {
        Overspeed {
            open_limit_umps,
            close_limit_umps,
            persist_ticks,
            accel_tol_umps: 0,
            timer: SlsTimer {
                since: None,
                v_start_umps: 0,
                brake_accel_since: None,
            },
        }
    }

    /// True when `v_umps` exceeds the limit of its direction.
    #[inline]
    pub fn over(&self, v_umps: i64) -> bool {
        if v_umps > 0 {
            v_umps > self.open_limit_umps
        } else {
            v_umps.unsigned_abs() > self.close_limit_umps.unsigned_abs()
        }
    }

    /// One check at tick `t`; true when tripped.
    pub fn step(&mut self, t: Tick, v_umps: i64, ev: DriveEvidence) -> bool {
        let over = self.over(v_umps);
        self.timer.step(
            t,
            over,
            v_umps.unsigned_abs(),
            self.persist_ticks,
            self.accel_tol_umps,
            ev,
        )
    }
}

#[cfg(test)]
#[allow(clippy::float_arithmetic)]
mod tests {
    use super::*;

    const DRIVE: DriveEvidence = DriveEvidence {
        energised: true,
        cmd_braking: false,
        cmd_driving: true,
    };

    /// Exact rational reference, rounded half away from zero, in i128.
    fn reference(d_counts: i128, d_ticks: u64, nm: i32) -> i64 {
        let num = d_counts * nm as i128 * 20_000;
        let den = d_ticks as i128 * 1000;
        let q = (num.abs() * 2 + den) / (2 * den);
        let q = q.min(i64::MAX as i128);
        (if num < 0 { -q } else { q }) as i64
    }

    // req: FAB-SAF-008
    #[test]
    fn integer_velocity_exact() {
        // Conversion identity: 1 count of 1 µm over 200 ticks (10 ms) is 100 µm/s.
        assert_eq!(
            velocity_umps(1, 200, 1000),
            100,
            "(E) 1 µm / 10 ms, PHYSICS §13"
        );
        assert_eq!(velocity_umps(-3, 200, 1000), -300, "(E) sign, PHYSICS §13");
        assert_eq!(velocity_umps(5, 0, 1000), 0, "(E) zero interval gives 0");
        // Exact against the rational reference over a sweep of differences, intervals
        // and resolutions (0.5 µm/s rounding bound).
        for nm in [1, 50, 1000, 5000, i32::MAX] {
            for d_ticks in [1u64, 2, 3, 7, 199, 200, 201, 4000] {
                for d in [-1_000_007i128, -12_345, -1, 0, 1, 9, 18_383, 1 << 40] {
                    assert_eq!(
                        velocity_umps(d, d_ticks, nm),
                        reference(d, d_ticks, nm),
                        "(E) exact rational, rounded half away: d={d} ticks={d_ticks} nm={nm}; PHYSICS §13"
                    );
                }
            }
        }
        // No overflow for the extreme i64 difference: saturates, never panics.
        let v = velocity_umps(i64::MAX as i128 - i64::MIN as i128, 1, i32::MAX);
        assert_eq!(v, i64::MAX, "(E) saturation at the i64 range");

        // Windowed estimate of a constant 0.1838 m/s ramp (DOOR open cap) sampled at the
        // 10 kHz fast rate on a 1 µm/count encoder: after the window fills, the integer
        // velocity is within one quantum (100 µm/s) of truth, and the window is exactly
        // 200 ticks (10 ms).
        let mut w = VelocityWindow::new(200, 1000);
        assert_eq!(w.quantum_umps(), 100, "(E) quantum = 1 count / 10 ms");
        let v_true_umps = 183_848.0f64;
        for k in 0..2_000u64 {
            let t = Tick(2 * k);
            let x_um = v_true_umps * (t.0 as f64) / 20_000.0;
            let counts = fmath_floor(x_um) as i64;
            let v = w.push(t, counts);
            if t.0 >= 200 {
                assert!(
                    fabrail_types::fmath::abs(v as f64 - v_true_umps) <= 100.0,
                    "(E) within one quantum (100 µm/s) at t={}: {v}; PLAN §2b.17",
                    t.0
                );
                let (t0, _) = w.sample(0).unwrap_or((Tick(0), 0));
                assert_eq!(t.since(t0), 200, "(E) window = 10 ms exactly");
            }
        }
        // History never exceeds the window plus one sample.
        assert!(w.len <= 101, "ring holds the window only: {}", w.len);
        // Time that does not advance restarts the history.
        assert_eq!(w.push(Tick(10), 5), 0, "non-monotonic tick restarts");
    }

    fn fmath_floor(x: f64) -> f64 {
        fabrail_types::fmath::floor(x)
    }

    // req: FAB-SAF-008
    #[test]
    fn overspeed_trips() {
        // DOOR open cap 0.1838 m/s → trip at 1.1× = 202 230 µm/s; close 0.152 → 167 200.
        let mut o = Overspeed::new(202_230, 167_200, 200);
        o.accel_tol_umps = 200;
        // Below the limit: never trips.
        for k in 0..1000u64 {
            assert!(
                !o.step(Tick(2 * k), 202_230, DRIVE),
                "at the limit, not above"
            );
        }
        // Above the limit for exactly 10 ms: no trip; for > 10 ms: trip (SAF-008).
        let t0 = 10_000u64;
        assert!(!o.step(Tick(t0), 202_231, DRIVE));
        assert!(
            !o.step(Tick(t0 + 200), 202_300, DRIVE),
            "(S) 10 ms exactly is not > 10 ms, SAF-008"
        );
        assert!(
            o.step(Tick(t0 + 202), 202_300, DRIVE),
            "(S) > 10 ms trips, SAF-008"
        );
        assert!(
            o.step(Tick(t0 + 300), 250_000, DRIVE),
            "stays tripped while above"
        );
        // Dropping below clears; a brief excursion (< 10 ms) never trips.
        assert!(!o.step(Tick(t0 + 302), 100_000, DRIVE));
        assert!(!o.step(Tick(t0 + 304), 300_000, DRIVE));
        assert!(!o.step(Tick(t0 + 500), 300_000, DRIVE), "exactly 196 ticks");
        assert!(!o.step(Tick(t0 + 506), 10_000, DRIVE));
        // Closing direction uses the closing limit.
        assert!(!o.step(Tick(t0 + 600), -170_000, DRIVE));
        assert!(
            o.step(Tick(t0 + 802), -170_000, DRIVE),
            "(S) closing limit 1.1·0.152 m/s"
        );
        assert!(
            !o.step(Tick(t0 + 804), 170_000, DRIVE),
            "170 mm/s opening is below 202 mm/s"
        );

        // Hand-driven motion: LSM de-energised → excused; LSM braking while the carriage
        // slows → excused; a brief push past the brake → excused (D4); braking command
        // but speeding up for > 0.5 s (sign error: positive drive work) → trips.
        let idle = DriveEvidence::default();
        let brake = DriveEvidence {
            energised: true,
            cmd_braking: true,
            cmd_driving: false,
        };
        let mut o = Overspeed::new(202_230, 167_200, 200);
        o.accel_tol_umps = 200;
        for k in 0..400u64 {
            assert!(
                !o.step(Tick(2 * k), 400_000, idle),
                "thrown by hand, LSM idle"
            );
        }
        let mut o = Overspeed::new(202_230, 167_200, 200);
        o.accel_tol_umps = 200;
        for k in 0..400u64 {
            let v = 400_000 - 300 * k as i64;
            assert!(
                !o.step(Tick(2 * k), v, brake),
                "soft-close braking a thrown drawer"
            );
        }
        // A hand overpowering the brake for 0.3 s (speeding up 250 → 340 mm/s): excused,
        // governor damping only (D4).
        let mut o = Overspeed::new(202_230, 167_200, 200);
        o.accel_tol_umps = 200;
        for k in 0..3_000u64 {
            let v = 250_000 + 30 * k as i64;
            assert!(
                !o.step(Tick(2 * k), v, brake),
                "brief push past the brake is not drive work (D4) at {} ms",
                k / 10
            );
        }
        // A 'braking' command while the carriage keeps speeding up for > 0.5 s: a
        // commutation sign error, positive drive work → trips after the sustain window.
        let mut o = Overspeed::new(202_230, 167_200, 200);
        o.accel_tol_umps = 200;
        let mut tripped_at = None;
        for k in 0..8_000u64 {
            let v = 250_000 + 30 * k as i64;
            if o.step(Tick(2 * k), v, brake) && tripped_at.is_none() {
                tripped_at = Some(2 * k);
            }
        }
        let t_trip = tripped_at.expect("a sustained 'braking' acceleration trips (D4)");
        assert!(
            (10_000..=10_600).contains(&t_trip),
            "trips 0.5 s after the first accelerating interval (+ ≤ 3 persistence intervals): tick {t_trip}"
        );
        // Energised with a zero (or non-Run) command: the force sign is unknown, so the
        // interval counts as possible drive work and trips (conservative).
        let neutral = DriveEvidence {
            energised: true,
            cmd_braking: false,
            cmd_driving: false,
        };
        let mut o = Overspeed::new(202_230, 167_200, 200);
        o.accel_tol_umps = 200;
        let mut tripped = false;
        for k in 0..400u64 {
            tripped |= o.step(Tick(2 * k), 400_000, neutral);
        }
        assert!(
            tripped,
            "current without a braking command above the limit trips"
        );
    }
}
