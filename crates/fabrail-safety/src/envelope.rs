//! Gap envelope, coil I2t and LSM current limits from integer codes, phase-sum check, and the ISOLATE-mode IsolateLimits (SAF-020). Owner: A5.
//!
//! * [`GapEnvelope`]: every channel reporting a valid conversion must read inside the
//!   code interval of [gap_min, gap_max] (the polynomial may be decreasing, so the code
//!   pair is ordered in `Monitor::new`) → GapEnvelope (SS1); a code on an ADC rail (0 or
//!   full scale), or a channel that was valid and stopped converting → GapRange
//!   (PHYSICS §13, §15).
//! * [`CoilI2t`]: i_cont continuous, i_pulse for ≤ t_pulse (plan §2b item 4): the excess
//!   Σ(i² − i_cont²)·Δt is integrated per element in code²·ticks (recovering below
//!   i_cont) against the budget (i_pulse² − i_cont²)·t_pulse → CoilI2t; a sample above
//!   the hardware clamp i_pulse (with the monitoring tolerance) → CoilOvercurrent
//!   (PHYSICS §4). Mirrors the core's software clamp.
//! * [`LsmEnvelope`]: per-phase |i| ≤ i_lsm_max (with tolerance) → LsmOvercurrent;
//!   |Σ i_abc| ≤ i_lsm_max/10 (star winding) → LsmPhaseSum; the driver fault line →
//!   LsmDriver (PHYSICS §7).
//! * [`IsolateEnvelope`] (SAF-020): in ISOLATE the explicit stroke envelope, bumper KE
//!   and continuous force F_cont REPLACE the clamp and KE checks rather than disabling
//!   them (an R-tier deviation declared in SAFETY.md) → IsolateLimit, which the monitor
//!   escalates to STO + SS1.
//!
//! All thresholds are integer codes derived in `Monitor::new`; no float arithmetic here.
#![deny(clippy::float_arithmetic)]

use fabrail_types::{CurrentRaw, FaultCode, FaultSet, GapRaw, LsmRaw, MAX_MAGNETS, Tick};

use crate::energy::ke2_fj;
use crate::{CAP_GRACE_TICKS, tol_lin, tol_sq};

/// Gap code envelope.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GapEnvelope {
    /// Lower code bound (code of the smaller of gap_min / gap_max codes).
    pub code_lo: u16,
    /// Upper code bound.
    pub code_hi: u16,
    /// Envelope configured (a valid gap_min < gap_max and polynomial).
    pub enabled: bool,
    /// Channels that have reported a valid conversion (bit i = channel i).
    pub seen: u8,
}

impl GapEnvelope {
    /// One release.
    pub fn step(&mut self, g: &GapRaw) -> FaultSet {
        let mut f = FaultSet::EMPTY;
        for (i, &code) in g.code.iter().enumerate().take(MAX_MAGNETS.min(8)) {
            let bit = 1u8 << i;
            if g.valid & bit == 0 {
                if self.seen & bit != 0 {
                    f.set(FaultCode::GapRange);
                }
                continue;
            }
            self.seen |= bit;
            if code == 0 || code == u16::MAX {
                f.set(FaultCode::GapRange);
            } else if self.enabled && (code < self.code_lo || code > self.code_hi) {
                f.set(FaultCode::GapEnvelope);
            }
        }
        f
    }
}

/// Coil I²t budget on integer current codes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CoilI2t {
    /// i_cont² in code².
    pub cont_sq: u64,
    /// i_pulse² in code² (hardware clamp).
    pub pulse_sq: u64,
    /// Budget (i_pulse² − i_cont²)·t_pulse in code²·ticks.
    pub budget: u64,
    /// Per-element accumulated excess, code²·ticks.
    pub acc: [u64; MAX_MAGNETS],
    /// Tick of the previous release.
    pub last: Option<Tick>,
}

impl CoilI2t {
    /// Longest interval integrated in one release, base ticks (a gap in the releases
    /// longer than this is a deadline miss handled by the executive).
    pub const MAX_DT_TICKS: u64 = 400;

    /// One release at tick `t`.
    pub fn step(&mut self, t: Tick, c: &CurrentRaw) -> FaultSet {
        let dt = match self.last {
            Some(l) => t.since(l).min(CoilI2t::MAX_DT_TICKS),
            None => 0,
        };
        self.last = Some(t);
        let over_sq = tol_sq(self.pulse_sq);
        let budget = tol_lin(self.budget);
        let mut f = FaultSet::EMPTY;
        for (acc, &code) in self.acc.iter_mut().zip(c.code.iter()) {
            let i2 = (code as i64 * code as i64) as u64;
            if i2 > over_sq {
                f.set(FaultCode::CoilOvercurrent);
            }
            *acc = if i2 > self.cont_sq {
                acc.saturating_add((i2 - self.cont_sq).saturating_mul(dt))
            } else {
                acc.saturating_sub((self.cont_sq - i2).saturating_mul(dt))
            };
            if *acc > budget {
                f.set(FaultCode::CoilI2t);
            }
        }
        f
    }
}

/// LSM phase-current envelope.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LsmEnvelope {
    /// i_lsm_max in code (per phase).
    pub i_max_code: i32,
    /// Phase-sum tolerance, code.
    pub sum_tol_code: i32,
}

impl LsmEnvelope {
    /// One release.
    pub fn step(&self, l: &LsmRaw) -> FaultSet {
        let mut f = FaultSet::EMPTY;
        let lim = tol_lin(self.i_max_code.max(0) as u64);
        let mut sum = 0i64;
        for &i in l.i_abc_code.iter() {
            if (i as i64).unsigned_abs() > lim {
                f.set(FaultCode::LsmOvercurrent);
            }
            sum += i as i64;
        }
        if sum.unsigned_abs() > self.sum_tol_code.max(0) as u64 {
            f.set(FaultCode::LsmPhaseSum);
        }
        if l.fault {
            f.set(FaultCode::LsmDriver);
        }
        f
    }
}

/// ISOLATE-mode replacement envelope (SAF-020).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IsolateEnvelope {
    /// Isolation centre, encoder counts (latched when ISOLATE is entered).
    pub center_counts: i64,
    /// Stroke about the centre, counts.
    pub stroke_counts: i64,
    /// Bumper KE limit as 2·KE, fJ (at rated mass).
    pub ke2_bumper_fj: u128,
    /// Rated mass, g.
    pub m_g: u32,
    /// Continuous force as Σ i² code² threshold.
    pub f_cont_sq: u64,
    /// Limits configured (all zero = isolation not permitted).
    pub configured: bool,
    /// Tick since which the measured force has exceeded F_cont (tolerance included).
    pub force_over_since: Option<Tick>,
}

impl IsolateEnvelope {
    /// One release in ISOLATE at tick `t`: IsolateLimit when outside the envelope
    /// (stroke about the centre, 2·KE at rated mass above the bumper limit, or force
    /// above F_cont for longer than the cap grace).
    pub fn step(&mut self, t: Tick, counts: i64, v_umps: i64, i_sq: u64) -> FaultSet {
        let mut f = FaultSet::EMPTY;
        let excursion = counts.abs_diff(self.center_counts);
        if excursion > self.stroke_counts.max(0) as u64 {
            f.set(FaultCode::IsolateLimit);
        }
        if ke2_fj(self.m_g, v_umps) > self.ke2_bumper_fj {
            f.set(FaultCode::IsolateLimit);
        }
        if i_sq > tol_sq(self.f_cont_sq) {
            let since = *self.force_over_since.get_or_insert(t);
            if t.since(since) > CAP_GRACE_TICKS as u64 {
                f.set(FaultCode::IsolateLimit);
            }
        } else {
            self.force_over_since = None;
        }
        f
    }
}

#[cfg(test)]
#[allow(clippy::float_arithmetic)]
mod tests {
    use super::*;
    use crate::monitor::Monitor;
    use crate::velocity::VEL_WINDOW_MIN_TICKS;
    use fabrail_types::{CoilCmd, FastRaw, LsmCmd, LsmMode, SafetyDemand, fmath};

    #[test]
    fn gap_current_limits() {
        // Gap: DOOR envelope [3.8, 6.75] mm on the 16-bit / 20 mm sensor.
        let code = |g_m: f64| fmath::round(g_m / (0.020 / 65_535.0)) as u16;
        let mut g = GapEnvelope {
            code_lo: code(3.8e-3),
            code_hi: code(6.75e-3),
            enabled: true,
            seen: 0,
        };
        let mut raw = GapRaw {
            valid: 0x0F,
            ..GapRaw::default()
        };
        raw.code[..4].copy_from_slice(&[code(5.0e-3); 4]);
        assert!(g.step(&raw).is_empty(), "hover at g* = 5 mm is inside");
        raw.code[2] = code(3.7e-3);
        assert!(
            g.step(&raw).contains(FaultCode::GapEnvelope),
            "3.7 mm < gap_min"
        );
        raw.code[2] = code(6.8e-3);
        assert!(
            g.step(&raw).contains(FaultCode::GapEnvelope),
            "6.8 mm > gap_max"
        );
        raw.code[2] = u16::MAX;
        assert!(g.step(&raw).contains(FaultCode::GapRange), "ADC rail");
        raw.code[2] = code(5.0e-3);
        raw.valid = 0x0B;
        assert!(
            g.step(&raw).contains(FaultCode::GapRange),
            "channel 2 stopped converting"
        );
        raw.valid = 0x0F;
        assert!(
            g.step(&raw).is_empty(),
            "channels 4..7 were never valid: unfitted"
        );

        // Coil I²t: DRAWER i_cont 4 A, i_pulse 12 A, 200 ms, 15/2048 A/code, 10 kHz.
        let a = 15.0 / 2048.0;
        let c = |i: f64| fmath::round(i / a) as i16;
        let cont = c(4.0) as u64;
        let pulse = c(12.0) as u64;
        let mut i2t = CoilI2t {
            cont_sq: cont * cont,
            pulse_sq: pulse * pulse,
            budget: (pulse * pulse - cont * cont) * 4_000,
            ..CoilI2t::default()
        };
        let mut cur = CurrentRaw::default();
        // Continuous 4 A never trips.
        cur.code[0] = c(4.0);
        for k in 0..100_000u64 {
            assert!(i2t.step(Tick(2 * k), &cur).is_empty(), "i_cont continuous");
        }
        // A 12 A pulse: allowed for 200 ms (+10 % tolerance), then CoilI2t.
        cur.code[0] = c(12.0);
        let t0 = 200_000u64;
        let mut trip_at = None;
        for k in 0..3_000u64 {
            let f = i2t.step(Tick(t0 + 2 * k), &cur);
            if f.contains(FaultCode::CoilI2t) {
                trip_at = Some(2 * k);
                break;
            }
            assert!(
                !f.contains(FaultCode::CoilOvercurrent),
                "12 A is the hardware clamp"
            );
        }
        let trip = trip_at.unwrap_or(0) as f64 / 20_000.0;
        assert!(
            (0.2..=0.221).contains(&trip),
            "(S) i_pulse for t_pulse = 200 ms (+10 % tolerance), PLAN §2b.4: tripped at {trip} s"
        );
        // Recovery below i_cont drains the budget.
        cur.code[0] = 0;
        for k in 0..20_000u64 {
            i2t.step(Tick(t0 + 10_000 + 2 * k), &cur);
        }
        assert_eq!(i2t.acc[0], 0, "the excess drains below i_cont");
        cur.code[3] = c(14.0);
        assert!(
            i2t.step(Tick(t0 + 60_000), &cur)
                .contains(FaultCode::CoilOvercurrent),
            "14 A > 1.1·12 A hardware clamp"
        );

        // LSM: DRAWER i_max 6 A at 9/2048 A/code.
        let l = LsmEnvelope {
            i_max_code: fmath::round(6.0 / (9.0 / 2048.0)) as i32,
            sum_tol_code: fmath::round(0.6 / (9.0 / 2048.0)) as i32,
        };
        let ok = LsmRaw {
            i_abc_code: [1365, -683, -682],
            fault: false,
        };
        assert!(l.step(&ok).is_empty(), "balanced set at i_max");
        let over = LsmRaw {
            i_abc_code: [1600, -800, -800],
            fault: false,
        };
        assert!(
            l.step(&over).contains(FaultCode::LsmOvercurrent),
            "7 A > 1.1·6 A"
        );
        let unbalanced = LsmRaw {
            i_abc_code: [400, 0, 0],
            fault: false,
        };
        assert!(
            l.step(&unbalanced).contains(FaultCode::LsmPhaseSum),
            "Σ i = 1.76 A"
        );
        let drv = LsmRaw {
            fault: true,
            ..LsmRaw::default()
        };
        assert!(l.step(&drv).contains(FaultCode::LsmDriver));
    }

    /// A WALL monitor in unsensed operation with an energised LSM driving along +x.
    fn wall_monitor() -> (Monitor, fabrail_params::SystemParams) {
        let p = fabrail_params::presets::wall();
        (Monitor::new(&p.safety, &p.design.sensors), p)
    }

    /// Raw sample at tick `t`: encoder at `counts`, LSM phase currents for `f_n` of
    /// thrust (amplitude-invariant, θ = 0).
    fn raw(p: &fabrail_params::SystemParams, t: u64, counts: i64, f_n: f64) -> FastRaw {
        let s = &p.safety;
        let i = f_n / s.k_f_copy_n_per_a / s.lsm_a_per_code_copy;
        let a = fmath::round(i) as i16;
        let b = fmath::round(-i / 2.0) as i16;
        let mut r = FastRaw {
            tick: Tick(t),
            ..FastRaw::default()
        };
        r.enc.counts = counts;
        r.enc.referenced = true;
        r.lsm.i_abc_code = [a, b, -a - b];
        // Gap at g* on every fitted channel.
        let g_code = fmath::round(
            (p.design.suspension.g_star_m - p.design.sensors.gap_poly_m[0])
                / p.design.sensors.gap_poly_m[1],
        ) as u16;
        r.gap.valid = 0x0F;
        for c in r.gap.code.iter_mut().take(4) {
            *c = g_code;
        }
        r
    }

    // req: FAB-SAF-020
    #[test]
    fn isolate_limits_replace_clamp() {
        let (mut m, p) = wall_monitor();
        let s = p.safety;
        assert!(
            m.isolate_env.configured,
            "WALL preset configures IsolateLimits"
        );
        let drive = LsmCmd {
            iq_ref_a: 1.0,
            id_ref_a: 0.0,
            mode: LsmMode::Run,
        };
        let coil = CoilCmd::CurrentRef([0.0; MAX_MAGNETS]);
        let x0 = 3_000_000i64; // 3 m on a 1 µm/count encoder
        // Isolation ground motion: the rail moves under an inertial carriage at
        // 0.30 m/s relative (≫ the 0.0446 m/s unsensed cap) with F_cont thrust.
        let v_rel_umps = 300_000i64;
        let f_cont = s.isolate.f_cont_n;

        // Outside ISOLATE this is an overspeed + momentum + KE violation.
        let mut plain = m.clone();
        let mut t = 0u64;
        let mut x = x0;
        let mut v_out = SafetyDemand::None;
        for _ in 0..200 {
            let r = raw(&p, t, x, f_cont);
            v_out = v_out.max(plain.step_fast(Tick(t), &r, &coil, &drive, t as u32).demand);
            t += 4;
            x += v_rel_umps * 4 / 20_000;
        }
        assert!(
            v_out >= SafetyDemand::StoLsm,
            "(T) clamp/KE/overspeed active outside ISOLATE"
        );

        // In ISOLATE the same motion is inside the IsolateLimits envelope: no trip, and
        // the cap in force is F_cont (the clamp is replaced, not disabled).
        m.set_isolate(true);
        assert!(m.isolate, "configured limits enable ISOLATE");
        let mut t = 0u64;
        let mut x = x0;
        for k in 0..400 {
            let r = raw(&p, t, x, f_cont);
            let v = m.step_fast(Tick(t), &r, &coil, &drive, k);
            assert!(
                v.faults.is_empty(),
                "(T) inside the envelope at t={t}: {:?}",
                v.faults
            );
            assert_eq!(v.demand, SafetyDemand::None);
            assert_eq!(
                v.force_cap_n, f_cont as f32,
                "(T) cap = F_cont in ISOLATE, SAF-020"
            );
            let dir = if (k / 100) % 2 == 0 { 1 } else { -1 };
            t += 4;
            x += dir * v_rel_umps * 4 / 20_000;
        }
        let window = VEL_WINDOW_MIN_TICKS as u64;
        assert!(t > window, "the velocity window is full");

        // Leaving the stroke envelope: IsolateLimit → STO + SS1.
        let stroke = fmath::round(s.isolate.stroke_m * 1.0e9 / s.nm_per_count_copy as f64) as i64;
        let mut iso = m.clone();
        let r = raw(&p, t, x0 + stroke + 10, 0.0);
        let v = iso.step_fast(Tick(t), &r, &coil, &drive, 10_000);
        assert!(
            v.faults.contains(FaultCode::IsolateLimit),
            "(T) stroke envelope"
        );
        assert!(
            v.demand >= SafetyDemand::StoLsm,
            "(T) IsolateLimit demands STO + SS1"
        );

        // Bumper KE at rated mass: ½·500·v² > 0.5·563 J ⇔ v > 1.06 m/s.
        let mut iso = m.clone();
        let mut tt = t;
        let mut xx = x0;
        let mut hit = false;
        for k in 0..100 {
            let r = raw(&p, tt, xx, 0.0);
            hit |= iso
                .step_fast(Tick(tt), &r, &coil, &drive, 20_000 + k)
                .faults
                .contains(FaultCode::IsolateLimit);
            tt += 4;
            xx += 1_200_000 * 4 / 20_000;
        }
        assert!(hit, "(T) bumper KE limit at 1.2 m/s");

        // Continuous force above F_cont (beyond the grace): IsolateLimit.
        let mut iso = m.clone();
        let mut hit = false;
        for k in 0..20u32 {
            let tt = t + 4 * k as u64;
            let r = raw(&p, tt, x0, 1.5 * f_cont);
            hit |= iso
                .step_fast(Tick(tt), &r, &coil, &drive, 30_000 + k)
                .faults
                .contains(FaultCode::IsolateLimit);
        }
        assert!(hit, "(T) continuous force limit F_cont");

        // Without configured limits ISOLATE is refused and the clamp stays active.
        let mut q = Monitor::new(
            &fabrail_params::SafetyLimits {
                isolate: fabrail_params::IsolateLimits::default(),
                ..s
            },
            &p.design.sensors,
        );
        q.set_isolate(true);
        assert!(
            !q.isolate,
            "(T) never disabled: no IsolateLimits, no ISOLATE"
        );
    }
}
