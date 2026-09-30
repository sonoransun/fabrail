//! Monitor: new (floats only here), step_fast and step_slow (integer-only) composing the checks into a SafetyVerdict. Owner: A5 (signatures frozen by W0a).
//!
//! API (W0, core design §B.5; plan §2b items 5, 15, 17; PHYSICS §18):
//!
//! * [`Monitor::new`]`(&SafetyLimits, &SensorCal)`: the ONLY place with float
//!   arithmetic. Every threshold becomes an integer in the checks' own units (grams,
//!   µm/s, fJ, g·µm/s, encoder counts, current codes², base ticks), from the
//!   `SafetyLimits` diverse copies (`*_copy`); `SensorCal` supplies only what the
//!   limits block has no copy of (gap polynomial, hold-up voltage scale). Non-finite or
//!   negative limits become the most restrictive value (0), never NaN.
//! * [`Monitor::step_fast`]: integer-only per-tick checks on RAW samples (encoder
//!   counts, gap codes, coil and LSM current codes; never the core's conversions), plus
//!   the core heartbeat. The core commands are read only by comparison: finiteness
//!   (NonFinite), the LSM mode (STO feedback) and the sign of i_q* (braking vs driving
//!   for the directional clamp and the drive-evidence exemption), never by arithmetic.
//!   Returns the COMBINED verdict: its own result merged with the latest `step_slow`
//!   result (demand = max, cap = min, faults = union), which fabrail-rt arbitrates.
//! * [`Monitor::step_slow`]: supply and discrete inputs at the supervisor rate (raw
//!   `PowerRaw`/`SafetyInRaw` forwarded unchanged by fabrail-rt), latching the sensed
//!   state (PL d fitted per the sealed copy AND the presence device healthy) used by the
//!   fast caps, the E-stop (controlled stop, SS1) and the LevState used to release the
//!   STO latch after a Reset. Returns the slow-only verdict.
//! * [`Monitor::set_isolate`]: SAF-020. In ISOLATE the explicit `IsolateLimits`
//!   envelope REPLACES the clamp, KE/momentum and overspeed checks (it does not disable
//!   them); ignored unless isolate limits were configured.
//!
//! **Positive drive work (Wave 2 lead decisions D3, D4).** The cap checks (overspeed,
//! KE, momentum) and the position checks (SLP/SDI) escalate only while the drive does
//! positive work F·v > 0 (the LSM energised by the monitor's own current measurement and
//! the command driving along the measured velocity, or a sustained sign error; see
//! `velocity::DriveEvidence`). Hand-thrown or rail-disturbance excess and floor-driven
//! displacement with the LSM in STO are met by the core's governor damping and the
//! physical stops; position excursions without drive work are counted in
//! [`Monitor::pos_report`] (reported, not a trip).
//!
//! **Demands.** Faults map to demands through their severity (`FaultCode::demand`);
//! Overspeed demands STO + ASC (SAF-008); IsolateLimit demands STO + SS1 (SAF-020); any
//! STO-level demand while |v| > v_runaway becomes STO + ASC (core design F9); the
//! E-stop demands SafeStop. STO-level demands LATCH in the monitor (STO restart needs an
//! explicit reset: the latch clears only when the supervisor leaves FAULT after a
//! `UserCommand::Reset`); ASC is released to STO once the carriage is at standstill
//! (F9). Lower demands are not latched here: the supervisor's FAULT_REACTION/FAULT latch
//! and the rt demand-latch window carry them (SAF-013, SAF-016). All demands act on
//! PROPULSION only; levitation's safe state is SS1 Safe Landing.
//!
//! One instance runs in the fast group (fabrail-rt calls `step_slow` there when a new
//! supervisor publication activates), so fast and slow checks share state without a
//! cross-task lock.
#![deny(clippy::float_arithmetic)]

use fabrail_params::{SafetyLimits, SensorCal};
use fabrail_types::{
    BASE_TICK_HZ, CoilCmd, FastRaw, FaultCode, FaultSet, LevState, LsmCmd, LsmMode, PowerRaw,
    PresenceSensing, SafetyDemand, SafetyInRaw, SafetyVerdict, Tick, fmath,
};

use crate::energy::EnergyCheck;
use crate::envelope::{CoilI2t, GapEnvelope, IsolateEnvelope, LsmEnvelope};
use crate::force::{ForceCap, ForceTimer, i_sq_code};
use crate::heartbeat::Heartbeat;
use crate::position::{PositionCheck, PositionReport, StoCheck};
use crate::supply::SupplyCheck;
use crate::velocity::{
    DriveEvidence, Overspeed, VEL_WINDOW_MIN_TICKS, VelocityWindow, velocity_umps,
};
use crate::{CAP_GRACE_TICKS, LSM_ACTIVE_DIV, LSM_SUM_DIV, STANDSTILL_UMPS};

/// Force caps in newtons, precomputed in `new()` (selected per tick, never computed).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ForceCaps {
    /// Dynamic drive cap F_drive_max.
    pub drive_n: f32,
    /// Static unsensed cap (67 N).
    pub static_n: f32,
    /// Static sensed / last-zone cap (150 N; the lower of the two when they differ).
    pub static_sensed_n: f32,
    /// Final static cap (< 80 N).
    pub static_final_n: f32,
    /// Braking cap: the inverter current limit K_f·i_lsm_max (braking is unclamped).
    pub brake_n: f32,
    /// ISOLATE continuous force F_cont (SAF-020).
    pub isolate_n: f32,
}

impl ForceCaps {
    /// The cap for a selector.
    #[inline]
    pub const fn get(&self, c: ForceCap) -> f32 {
        match c {
            ForceCap::Drive => self.drive_n,
            ForceCap::Static => self.static_n,
            ForceCap::StaticSensed => self.static_sensed_n,
            ForceCap::StaticFinal => self.static_final_n,
            ForceCap::Brake => self.brake_n,
            ForceCap::Isolate => self.isolate_n,
        }
    }

    /// The largest cap (the slow verdict's "no limit").
    pub fn max_n(&self) -> f32 {
        let mut m = self.drive_n;
        for c in [self.brake_n, self.isolate_n, self.static_sensed_n] {
            if c > m {
                m = c;
            }
        }
        m
    }
}

/// The independent safety monitor.
#[derive(Clone, Debug)]
pub struct Monitor {
    /// Windowed encoder velocity.
    pub vel: VelocityWindow,
    /// Overspeed persistence.
    pub overspeed: Overspeed,
    /// Overspeed trip speeds [open, close] unsensed, µm/s (1.1 × the unsensed caps).
    pub overspeed_unsensed: [i64; 2],
    /// Overspeed trip speeds [open, close] with PL d sensing, µm/s.
    pub overspeed_sensed: [i64; 2],
    /// Energy/momentum at rated mass.
    pub energy: EnergyCheck,
    /// Force-time supervision.
    pub force: ForceTimer,
    /// Gap envelope.
    pub gap: GapEnvelope,
    /// Coil I²t.
    pub coil: CoilI2t,
    /// LSM current envelope.
    pub lsm: LsmEnvelope,
    /// ISOLATE replacement envelope (SAF-020).
    pub isolate_env: IsolateEnvelope,
    /// Position / direction.
    pub pos: PositionCheck,
    /// Reported (non-tripping) position excursions (D3).
    pub pos_report: PositionReport,
    /// STO feedback (no torque current while STO is commanded).
    pub sto: StoCheck,
    /// Core heartbeat.
    pub hb: Heartbeat,
    /// Hold-up supply.
    pub supply: SupplyCheck,
    /// Force caps, N.
    pub caps: ForceCaps,
    /// Σ i_abc² above which the LSM counts as energised, code².
    pub active_sq: u64,
    /// Encoder counts at the end of the last closing zone (x_min + last_zone).
    pub last_zone_counts: i64,
    /// ASC threshold v_runaway, µm/s (F9).
    pub v_runaway_umps: u64,
    /// ISOLATE active (set via `set_isolate`).
    pub isolate: bool,
    /// The isolation centre is latched at the next fast step.
    pub isolate_center_pending: bool,
    /// Sensed operation configured: `SafetyLimits::presence_copy` is PL d (the diverse
    /// copy of `design.presence`; SAF-003). Non-zero sensed caps alone never enable it.
    pub sensed_allowed: bool,
    /// Presence sensing configured AND healthy (latched by `step_slow`).
    pub sensed: bool,
    /// Latest LevState seen by `step_slow`.
    pub lev: LevState,
    /// Latched STO-level demand (None, StoLsm or StoLsmAsc).
    pub sto_latch: SafetyDemand,
    /// Faults that caused the STO latch (reported until the latch clears).
    pub sto_faults: FaultSet,
    /// Demand of the previous fast verdict.
    pub last_demand: SafetyDemand,
    /// Latest slow verdict (merged into every fast verdict).
    pub slow: SafetyVerdict,
}

/// Finite and non-negative, else 0 (init-time sanitising: never NaN downstream).
#[inline]
fn nonneg(x: f64) -> f64 {
    if x.is_finite() && x > 0.0 { x } else { 0.0 }
}

/// Nearest integer of a finite f64, saturating; NaN → 0 (init-time conversion only).
#[inline]
fn round_i64(x: f64) -> i64 {
    fmath::round(x) as i64
}

/// A cap in newtons as a finite, non-negative f32.
#[inline]
#[allow(clippy::float_arithmetic)]
fn cap_f32(x: f64) -> f32 {
    fmath::fmin(nonneg(x), f32::MAX as f64) as f32
}

/// Code of gap `g_m` on the polynomial g = c0 + c1·code + c2·code², if one lies in the
/// 16-bit code range.
#[allow(clippy::float_arithmetic)]
fn gap_code(poly: &[f64; 3], g_m: f64) -> Option<f64> {
    let [c0, c1, c2] = *poly;
    if !(c0.is_finite() && c1.is_finite() && c2.is_finite() && g_m.is_finite()) {
        return None;
    }
    let in_range = |c: f64| (0.0..=65_535.0).contains(&c);
    if c2 == 0.0 {
        if c1 == 0.0 {
            return None;
        }
        let c = (g_m - c0) / c1;
        return in_range(c).then_some(c);
    }
    let disc = c1 * c1 - 4.0 * c2 * (c0 - g_m);
    if disc < 0.0 {
        return None;
    }
    let s = fmath::sqrt(disc);
    let r1 = (-c1 + s) / (2.0 * c2);
    let r2 = (-c1 - s) / (2.0 * c2);
    if in_range(r1) {
        Some(r1)
    } else if in_range(r2) {
        Some(r2)
    } else {
        None
    }
}

/// True when every value of the core commands is finite (comparison only).
#[inline]
fn commands_finite(coil: &CoilCmd, lsm: &LsmCmd) -> bool {
    coil.values().iter().all(|x| x.is_finite())
        && lsm.iq_ref_a.is_finite()
        && lsm.id_ref_a.is_finite()
}

impl Monitor {
    /// Converts the limits to integer thresholds (the only float arithmetic in the
    /// monitor).
    #[allow(clippy::float_arithmetic)]
    pub fn new(l: &SafetyLimits, cal: &SensorCal) -> Monitor {
        let tick_hz = BASE_TICK_HZ as f64;
        let ticks = |s: f64| -> u32 { fmath::round(nonneg(s) * tick_hz) as u32 };
        let umps = |v: f64| -> i64 { round_i64(nonneg(v) * 1.0e6) };
        let m_g = fmath::round(nonneg(l.mass_rated_kg) * 1.0e3) as u32;
        let nm_per_count = l.nm_per_count_copy;
        let counts = |x_m: f64| -> i64 {
            if nm_per_count <= 0 || !x_m.is_finite() {
                0
            } else {
                round_i64(x_m * 1.0e9 / nm_per_count as f64)
            }
        };
        // LSM: a thrust F maps to Σ i_abc² = 1.5·(F/(K_f·A_code))² (PHYSICS §7).
        let n_per_code = nonneg(l.k_f_copy_n_per_a) * nonneg(l.lsm_a_per_code_copy);
        let force_sq = |f_n: f64| -> u64 {
            if n_per_code > 0.0 {
                let c = nonneg(f_n) / n_per_code;
                fmath::round(1.5 * c * c) as u64
            } else {
                0
            }
        };
        let lsm_a = nonneg(l.lsm_a_per_code_copy);
        let i_lsm_code = if lsm_a > 0.0 {
            fmath::round(nonneg(l.i_lsm_max_a) / lsm_a) as i64
        } else {
            0
        };
        let i_lsm_code = i_lsm_code.clamp(0, i32::MAX as i64) as i32;
        let sum_tol = i_lsm_code / LSM_SUM_DIV as i32;
        let coil_a = nonneg(l.adc_a_per_code_copy);
        let coil_sq = |i_a: f64| -> u64 {
            if coil_a > 0.0 {
                let c = nonneg(i_a) / coil_a;
                fmath::round(c * c) as u64
            } else {
                0
            }
        };

        // Velocity window ≥ 10 ms (plan §2b item 17) and its quantum.
        let window = ticks(l.vel_window_s).max(VEL_WINDOW_MIN_TICKS);
        let quantum = velocity_umps(1, window as u64, nm_per_count).unsigned_abs();
        // "Speeding up" = more than two velocity quanta over one persistence interval.
        let accel_tol = quantum.saturating_mul(2);
        let persist = ticks(l.overspeed_t_s);
        let ratio = nonneg(l.overspeed_ratio);
        let trip = |v: f64| umps(nonneg(v) * ratio);

        let mut overspeed =
            Overspeed::new(trip(l.v_open_max_mps), trip(l.v_close_max_mps), persist);
        overspeed.accel_tol_umps = accel_tol;

        let mut energy = EnergyCheck::new(
            m_g,
            fmath::round(2.0 * nonneg(l.e_max_j) * 1.0e15) as u128,
            fmath::round(nonneg(l.p_max_kgmps) * 1.0e9) as u128,
        );
        energy.persist_ticks = persist;
        energy.accel_tol_umps = accel_tol;

        // The higher static cap serves both sensed operation and the last closing zone;
        // the lower of the two limits is used when they differ (conservative).
        let high_static = fmath::fmin(nonneg(l.f_static_sensed_n), nonneg(l.f_close_last50_n));
        let brake_n = nonneg(l.k_f_copy_n_per_a) * nonneg(l.i_lsm_max_a);
        let force = ForceTimer {
            drive_sq: force_sq(l.f_drive_max_n),
            static_sq: force_sq(l.f_static_n),
            static_sensed_sq: force_sq(high_static),
            static_final_sq: force_sq(l.f_static_final_n),
            brake_sq: force_sq(brake_n),
            drive_window_ticks: ticks(l.t_drive_max_s),
            final_window_ticks: ticks(l.t_static_final_s),
            grace_ticks: CAP_GRACE_TICKS,
            ..ForceTimer::default()
        };

        // Gap envelope in codes, rounded inward (conservative by < 1 code).
        let gap = match (
            gap_code(&cal.gap_poly_m, l.gap_min_m),
            gap_code(&cal.gap_poly_m, l.gap_max_m),
        ) {
            (Some(a), Some(b)) if l.gap_min_m > 0.0 && l.gap_min_m < l.gap_max_m => {
                let lo = fmath::ceil(fmath::fmin(a, b)) as u16;
                let hi = fmath::floor(fmath::fmax(a, b)) as u16;
                GapEnvelope {
                    code_lo: lo,
                    code_hi: hi,
                    enabled: lo <= hi,
                    seen: 0,
                }
            }
            _ => GapEnvelope::default(),
        };

        let cont_sq = coil_sq(l.i_coil_cont_a);
        let pulse_sq = coil_sq(l.i_coil_pulse_a);
        let coil = CoilI2t {
            cont_sq,
            pulse_sq,
            budget: pulse_sq
                .saturating_sub(cont_sq)
                .saturating_mul(ticks(l.t_coil_pulse_s) as u64),
            ..CoilI2t::default()
        };

        let iso = l.isolate;
        let isolate_env = IsolateEnvelope {
            stroke_counts: counts(nonneg(iso.stroke_m)),
            ke2_bumper_fj: fmath::round(2.0 * nonneg(iso.bumper_ke_j) * 1.0e15) as u128,
            m_g,
            f_cont_sq: force_sq(iso.f_cont_n),
            configured: nonneg(iso.stroke_m) > 0.0
                && nonneg(iso.bumper_ke_j) > 0.0
                && nonneg(iso.f_cont_n) > 0.0,
            ..IsolateEnvelope::default()
        };

        let caps = ForceCaps {
            drive_n: cap_f32(l.f_drive_max_n),
            static_n: cap_f32(l.f_static_n),
            static_sensed_n: cap_f32(high_static),
            static_final_n: cap_f32(l.f_static_final_n),
            brake_n: cap_f32(brake_n),
            isolate_n: cap_f32(iso.f_cont_n),
        };

        // Hold-up floor code, rounded UP (never below the SAF-009 floor).
        let v_per = nonneg(cal.v_hold_v_per_code);
        let v_hold_min_code = if v_per > 0.0 {
            fmath::ceil(nonneg(l.v_hold_min_v) / v_per) as u16
        } else {
            u16::MAX
        };

        let slow = SafetyVerdict {
            demand: SafetyDemand::None,
            force_cap_n: caps.max_n(),
            faults: FaultSet::EMPTY,
        };
        Monitor {
            vel: VelocityWindow::new(window, nm_per_count),
            overspeed,
            overspeed_unsensed: [trip(l.v_open_max_mps), trip(l.v_close_max_mps)],
            overspeed_sensed: [trip(l.v_open_sensed_mps), trip(l.v_close_sensed_mps)],
            energy,
            force,
            gap,
            coil,
            lsm: LsmEnvelope {
                i_max_code: i_lsm_code,
                sum_tol_code: sum_tol,
            },
            isolate_env,
            pos: PositionCheck {
                min_counts: counts(l.x_min_m),
                max_counts: counts(l.x_max_m),
                standstill_umps: STANDSTILL_UMPS as i64,
            },
            pos_report: PositionReport::default(),
            sto: StoCheck {
                i_sq_max: 3 * (sum_tol as u64) * (sum_tol as u64) / 2,
                grace_ticks: CAP_GRACE_TICKS,
                sto_since: None,
            },
            hb: Heartbeat::new(l.heartbeat_max_ticks),
            supply: SupplyCheck { v_hold_min_code },
            caps,
            active_sq: force_sq(nonneg(l.f_drive_max_n) / LSM_ACTIVE_DIV as f64),
            last_zone_counts: counts(l.x_min_m + nonneg(l.last_zone_m)),
            v_runaway_umps: umps(l.v_runaway_mps).unsigned_abs(),
            isolate: false,
            isolate_center_pending: false,
            sensed_allowed: l.presence_copy == PresenceSensing::PlD,
            sensed: false,
            lev: LevState::Boot,
            sto_latch: SafetyDemand::None,
            sto_faults: FaultSet::EMPTY,
            last_demand: SafetyDemand::None,
            slow,
        }
    }

    /// Fast-rate checks at tick `t` on the latched raw sample, the (pre-arbitration)
    /// core commands and the core heartbeat `hb`. Returns the combined verdict.
    pub fn step_fast(
        &mut self,
        t: Tick,
        r: &FastRaw,
        coil: &CoilCmd,
        lsm: &LsmCmd,
        hb: u32,
    ) -> SafetyVerdict {
        let mut faults = FaultSet::EMPTY;
        let mut demand = SafetyDemand::None;
        if !commands_finite(coil, lsm) {
            faults.set(FaultCode::NonFinite);
        }
        if r.enc.err {
            faults.set(FaultCode::EncoderLost);
        }
        let counts = r.enc.counts;
        let v = self.vel.push(t, counts);
        let v_abs = v.unsigned_abs();
        let i_sq = i_sq_code(r.lsm.i_abc_code);
        let moving = v_abs > STANDSTILL_UMPS;
        let run = lsm.mode == LsmMode::Run;
        let cmd_braking =
            run && moving && ((v > 0 && lsm.iq_ref_a < 0.0) || (v < 0 && lsm.iq_ref_a > 0.0));
        let cmd_driving =
            run && moving && ((v > 0 && lsm.iq_ref_a > 0.0) || (v < 0 && lsm.iq_ref_a < 0.0));
        let ev = DriveEvidence {
            energised: i_sq > self.active_sq,
            cmd_braking,
            cmd_driving,
        };

        let mut limited = false;
        let own_cap = if self.isolate {
            // SAF-020: the explicit envelope replaces clamp, KE/momentum and overspeed.
            if self.isolate_center_pending {
                self.isolate_env.center_counts = counts;
                self.isolate_center_pending = false;
            }
            let fi = self.isolate_env.step(t, counts, v, i_sq);
            if !fi.is_empty() {
                demand = demand.max(SafetyDemand::StoLsm);
            }
            faults.merge(fi);
            self.caps.isolate_n
        } else {
            let [open, close] = if self.sensed {
                self.overspeed_sensed
            } else {
                self.overspeed_unsensed
            };
            self.overspeed.open_limit_umps = open;
            self.overspeed.close_limit_umps = close;
            if self.overspeed.step(t, v, ev) {
                faults.set(FaultCode::Overspeed);
                demand = demand.max(SafetyDemand::StoLsmAsc);
            }
            if self.sensed {
                self.energy.reset();
            } else {
                faults.merge(self.energy.step(t, v, ev));
            }
            let closing = v < -(STANDSTILL_UMPS as i64) || (!moving && run && lsm.iq_ref_a < 0.0);
            let in_last_zone = r.enc.referenced && counts <= self.last_zone_counts;
            let high_static = self.sensed || (closing && in_last_zone);
            let fo = self.force.step(t, i_sq, cmd_braking, high_static);
            if fo.fault {
                faults.set(FaultCode::ForceLimit);
            }
            limited = matches!(
                fo.cap,
                ForceCap::Static | ForceCap::StaticSensed | ForceCap::StaticFinal
            );
            self.caps.get(fo.cap)
        };

        faults.merge(self.gap.step(&r.gap));
        faults.merge(self.coil.step(t, &r.cur));
        faults.merge(self.lsm.step(&r.lsm));
        if r.enc.referenced {
            // D3: SLP/SDI escalate only while the drive does positive work outward; a
            // floor-driven excursion with the LSM in STO (or braking) is reported.
            let (fp, excursion) = self.pos.step(counts, v, ev);
            faults.merge(fp);
            self.pos_report.update(t, excursion, counts, &self.pos);
        }
        // STO is in force when the core commands it or when the arbitration applied the
        // monitor's own STO demand of the previous release; ASC currents are intended.
        let asc = self.sto_latch == SafetyDemand::StoLsmAsc
            || self.last_demand == SafetyDemand::StoLsmAsc;
        let sto_in_force = lsm.mode == LsmMode::Sto || self.last_demand == SafetyDemand::StoLsm;
        faults.merge(self.sto.step(t, sto_in_force && !asc, i_sq));
        if self.hb.step(t, hb) {
            faults.set(FaultCode::HeartbeatLost);
        }

        demand = demand.max(faults.demand());
        if demand >= SafetyDemand::StoLsm && v_abs > self.v_runaway_umps {
            demand = SafetyDemand::StoLsmAsc; // F9: ASC above v_runaway
        }
        // STO-level demands latch until the supervisor leaves FAULT after a Reset.
        if demand >= SafetyDemand::StoLsm {
            self.sto_latch = self.sto_latch.max(demand);
            for c in faults.iter() {
                if c.demand() >= SafetyDemand::StoLsm || c == FaultCode::IsolateLimit {
                    self.sto_faults.set(c);
                }
            }
        }
        if self.sto_latch == SafetyDemand::StoLsmAsc && !moving && demand < SafetyDemand::StoLsmAsc
        {
            self.sto_latch = SafetyDemand::StoLsm; // F9: ASC released after stop
        }
        demand = demand.max(self.sto_latch);
        faults.merge(self.sto_faults);
        if limited {
            demand = demand.max(SafetyDemand::LimitForce);
        }

        faults.merge(self.slow.faults);
        demand = demand.max(self.slow.demand);
        let cap = if own_cap < self.slow.force_cap_n {
            own_cap
        } else {
            self.slow.force_cap_n
        };
        self.last_demand = demand;
        SafetyVerdict {
            demand,
            force_cap_n: cap,
            faults,
        }
    }

    /// Supervisor-rate checks at tick `t` on the raw power and safety inputs and the
    /// current LevState. Returns the slow-only verdict (also merged into later fast
    /// verdicts).
    pub fn step_slow(
        &mut self,
        t: Tick,
        p: &PowerRaw,
        s: &SafetyInRaw,
        lev: LevState,
    ) -> SafetyVerdict {
        let _ = t;
        if self.lev == LevState::Fault && lev != LevState::Fault && lev != LevState::FaultReaction {
            // The supervisor left FAULT: only `UserCommand::Reset` does that (SAF-013).
            self.sto_latch = SafetyDemand::None;
            self.sto_faults = FaultSet::EMPTY;
        }
        self.lev = lev;
        self.sensed = self.sensed_allowed && s.presence_ok;
        let faults = self.supply.step(p);
        let mut demand = faults.demand();
        if s.estop {
            demand = demand.max(SafetyDemand::SafeStop);
        }
        self.slow = SafetyVerdict {
            demand,
            force_cap_n: self.caps.max_n(),
            faults,
        };
        self.slow
    }

    /// Enters or leaves ISOLATE (SAF-020): the isolate envelope replaces the clamp and
    /// KE checks. Ignored (stays false) when no isolate limits are configured. The
    /// isolation centre is the position at entry.
    pub fn set_isolate(&mut self, active: bool) {
        let on = active && self.isolate_env.configured;
        if on && !self.isolate {
            self.isolate_center_pending = true;
            self.isolate_env.force_over_since = None;
        }
        if !on && self.isolate {
            self.force.reset();
            self.energy.reset();
            self.overspeed.timer.reset();
        }
        self.isolate = on;
    }
}

#[cfg(test)]
#[allow(clippy::float_arithmetic)]
mod tests {
    use super::*;
    use fabrail_params::{SystemParams, presets};
    use fabrail_types::MAX_MAGNETS;

    #[test]
    fn stale_heartbeat_demands_sto() {
        let l = SafetyLimits {
            heartbeat_max_ticks: 6,
            f_drive_max_n: 90.0,
            ..SafetyLimits::default()
        };
        let mut m = Monitor::new(&l, &SensorCal::default());
        let raw = FastRaw::default();
        let v = m.step_fast(Tick(0), &raw, &CoilCmd::Off, &LsmCmd::default(), 1);
        assert_eq!(v.demand, SafetyDemand::None);
        assert_eq!(v.force_cap_n, 90.0);
        let v = m.step_fast(Tick(8), &raw, &CoilCmd::Off, &LsmCmd::default(), 1);
        assert!(v.faults.contains(FaultCode::HeartbeatLost));
        assert!(v.demand >= SafetyDemand::StoLsm);
    }

    #[test]
    fn sensed_caps_need_the_presence_copy() {
        // Presets carry non-zero sensed caps with no presence sensing fitted; only the
        // sealed diverse copy of design.presence enables sensed operation (SAF-003).
        let mut p = presets::door();
        assert!(p.safety.v_open_sensed_mps > 0.0);
        let m = Monitor::new(&p.safety, &p.design.sensors);
        assert!(!m.sensed_allowed);
        p.design.presence = PresenceSensing::PlD;
        fabrail_params::crc::seal(&mut p);
        let m = Monitor::new(&p.safety, &p.design.sensors);
        assert!(m.sensed_allowed);
    }

    #[test]
    fn isolate_requires_configured_limits() {
        let mut m = Monitor::new(&SafetyLimits::default(), &SensorCal::default());
        m.set_isolate(true);
        assert!(!m.isolate, "no IsolateLimits configured");
    }

    /// Raw sample: encoder at `counts`, LSM currents for thrust `f_n` (θ = 0), gaps at
    /// g*, coil currents zero.
    fn raw(p: &SystemParams, t: u64, counts: i64, f_n: f64) -> FastRaw {
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
        let poly = p.design.sensors.gap_poly_m;
        let g_code = fmath::round((p.design.suspension.g_star_m - poly[0]) / poly[1]) as u16;
        r.gap.valid = (1u16 << p.design.suspension.n_magnets).wrapping_sub(1) as u8;
        for c in r
            .gap
            .code
            .iter_mut()
            .take(p.design.suspension.n_magnets as usize)
        {
            *c = g_code;
        }
        r
    }

    /// Kinematic open/cruise/brake profile at rated mass with the LSM supplying m·a:
    /// accelerate at a_max to 0.99 × the unsensed cap, cruise, brake at
    /// min(2·a_max, 0.95·F_peak/m), then stand still with the LSM in STO. The monitor
    /// must stay silent (no false trips on nominal motion; W1.5 guard).
    #[test]
    fn nominal_motion_no_trip() {
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let mut m = Monitor::new(&p.safety, &p.design.sensors);
            let slow = m.step_slow(
                Tick(0),
                &PowerRaw {
                    v_hold_code: u16::MAX - 1,
                    mains_ok: true,
                    ..PowerRaw::default()
                },
                &SafetyInRaw::default(),
                LevState::Moving,
            );
            assert!(slow.faults.is_empty(), "{:?}", slow.faults);
            let mass = p.safety.mass_rated_kg;
            let a_max = p.control.motion.a_max_mps2;
            let v_cap = 0.99 * p.safety.v_open_max_mps;
            let f_peak = p.design.lsm.f_peak_n;
            let a_brk = fmath::fmin(2.0 * a_max, 0.95 * f_peak / mass);
            let decim = p.control.rates.fast_decim as u64;
            let dt = decim as f64 / 20_000.0;
            let t_cruise = fmath::fmin(1.0, 0.4 * p.design.rail.travel_m / v_cap);
            let f_drive = p.safety.f_drive_max_n;
            let (mut x, mut v) = (0.05f64, 0.0f64);
            let mut phase = 0; // 0 accel, 1 cruise, 2 brake, 3 still
            let mut t_phase = 0.0;
            let mut k = 0u32;
            while k < 200_000 {
                let t = k as u64 * decim;
                // The core clamps a force below the stall speed (2 mm/s) as driving
                // (≤ F_drive_max; PHYSICS §16), braking above it is unclamped.
                let f_brk = if v < 2.0e-3 {
                    fmath::fmin(mass * a_brk, f_drive)
                } else {
                    mass * a_brk
                };
                let (a, f, mode) = match phase {
                    0 => (a_max, mass * a_max, LsmMode::Run),
                    1 => (0.0, 0.0, LsmMode::Run),
                    2 => (-f_brk / mass, -f_brk, LsmMode::Run),
                    _ => (0.0, 0.0, LsmMode::Sto),
                };
                let counts = fmath::floor(x * 1.0e6) as i64;
                let r = raw(&p, t, counts, fmath::abs(f));
                let mut r = r;
                if f < 0.0 {
                    for c in r.lsm.i_abc_code.iter_mut() {
                        *c = -*c;
                    }
                }
                let cmd = LsmCmd {
                    iq_ref_a: (f / p.design.lsm.k_f_n_per_a) as f32,
                    id_ref_a: 0.0,
                    mode,
                };
                let coil = CoilCmd::CurrentRef([0.0; MAX_MAGNETS]);
                let vd = m.step_fast(Tick(t), &r, &coil, &cmd, k);
                assert!(
                    vd.faults.is_empty() && vd.demand <= SafetyDemand::LimitForce,
                    "{:?} phase {phase} t={t} v={v:.4} m/s: {:?} {:?}",
                    p.design.class,
                    vd.faults,
                    vd.demand
                );
                x += v * dt + 0.5 * a * dt * dt;
                v += a * dt;
                t_phase += dt;
                match phase {
                    0 if v >= v_cap => {
                        v = v_cap;
                        phase = 1;
                        t_phase = 0.0;
                    }
                    1 if t_phase >= t_cruise => {
                        phase = 2;
                        t_phase = 0.0;
                    }
                    2 if v <= 0.0 => {
                        v = 0.0;
                        phase = 3;
                        t_phase = 0.0;
                    }
                    3 if t_phase >= 0.5 => break,
                    _ => {}
                }
                k += 1;
            }
            assert_eq!(phase, 3, "{:?}: profile completed", p.design.class);
        }
    }

    #[test]
    fn nonfinite_and_sto_latch_until_reset() {
        let p = presets::door();
        let mut m = Monitor::new(&p.safety, &p.design.sensors);
        let r = raw(&p, 0, 100_000, 0.0);
        let coil = CoilCmd::CurrentRef([0.0; MAX_MAGNETS]);
        let run = LsmCmd {
            mode: LsmMode::Run,
            ..LsmCmd::default()
        };
        // A NaN in any core command: NonFinite → SS1 (SAF-013/017).
        let nan = LsmCmd {
            iq_ref_a: f32::NAN,
            ..run
        };
        let v = m.step_fast(Tick(0), &r, &coil, &nan, 1);
        assert!(v.faults.contains(FaultCode::NonFinite));
        assert_eq!(
            v.demand,
            SafetyDemand::SafeLand,
            "(T) non-finite → FAULT_REACTION"
        );
        let v = m.step_fast(Tick(2), &r, &coil, &run, 2);
        assert!(
            v.faults.is_empty(),
            "NonFinite is not latched in the monitor (SS1 via supervisor)"
        );

        // An LSM driver fault: STO, latched through a normal-looking sample.
        let mut bad = r;
        bad.lsm.fault = true;
        let v = m.step_fast(Tick(4), &bad, &coil, &run, 3);
        assert_eq!(v.demand, SafetyDemand::StoLsm);
        let v = m.step_fast(Tick(6), &r, &coil, &run, 4);
        assert_eq!(v.demand, SafetyDemand::StoLsm, "(T) STO latched");
        assert!(
            v.faults.contains(FaultCode::LsmDriver),
            "cause reported while latched"
        );
        // The supervisor enters FAULT_REACTION then FAULT; only leaving FAULT (Reset)
        // clears the latch.
        let pw = PowerRaw {
            v_hold_code: u16::MAX - 1,
            mains_ok: true,
            ..PowerRaw::default()
        };
        let si = SafetyInRaw::default();
        for lev in [LevState::FaultReaction, LevState::Fault, LevState::Fault] {
            m.step_slow(Tick(20), &pw, &si, lev);
            let v = m.step_fast(Tick(22), &r, &coil, &run, 5);
            assert_eq!(v.demand, SafetyDemand::StoLsm, "held in {lev:?}");
        }
        m.step_slow(Tick(40), &pw, &si, LevState::LevitatedIdle);
        let v = m.step_fast(Tick(42), &r, &coil, &run, 6);
        assert_eq!(
            v.demand,
            SafetyDemand::None,
            "(T) Reset clears the STO latch"
        );

        // E-stop: controlled stop (SS1), not latched in the monitor.
        let es = SafetyInRaw { estop: true, ..si };
        let v = m.step_slow(Tick(60), &pw, &es, LevState::LevitatedIdle);
        assert_eq!(v.demand, SafetyDemand::SafeStop);
        let v = m.step_fast(Tick(62), &r, &coil, &run, 7);
        assert_eq!(
            v.demand,
            SafetyDemand::SafeStop,
            "slow demand merged into the fast verdict"
        );
    }

    #[test]
    fn overspeed_demands_sto_asc_until_standstill() {
        let p = presets::door();
        let mut m = Monitor::new(&p.safety, &p.design.sensors);
        let coil = CoilCmd::CurrentRef([0.0; MAX_MAGNETS]);
        let drive = LsmCmd {
            iq_ref_a: 3.0,
            id_ref_a: 0.0,
            mode: LsmMode::Run,
        };
        // Runaway at 0.25 m/s (1.36 × the 0.1838 m/s open cap) with 90 N of thrust.
        let mut x = 0.2f64;
        let mut demand = SafetyDemand::None;
        let mut k = 0u32;
        while k < 400 {
            let r = raw(&p, 2 * k as u64, fmath::floor(x * 1.0e6) as i64, 90.0);
            demand = m.step_fast(Tick(2 * k as u64), &r, &coil, &drive, k).demand;
            if demand == SafetyDemand::StoLsmAsc {
                break;
            }
            x += 0.25 * 1.0e-4;
            k += 1;
        }
        assert_eq!(
            demand,
            SafetyDemand::StoLsmAsc,
            "(T) overspeed → STO + ASC, SAF-008"
        );
        assert!(k as u64 * 2 <= 200 + 200 + 4, "within window + 10 ms: {k}");
        // ASC brakes the carriage to standstill; ASC is then released to STO (F9).
        let sto = LsmCmd {
            mode: LsmMode::Sto,
            ..LsmCmd::default()
        };
        let x_stop = fmath::floor(x * 1.0e6) as i64;
        let mut last = SafetyDemand::None;
        for j in 1..=300u32 {
            let t = 2 * (k + j) as u64;
            let r = raw(&p, t, x_stop, 0.0);
            last = m.step_fast(Tick(t), &r, &coil, &sto, k + j).demand;
        }
        assert_eq!(
            last,
            SafetyDemand::StoLsm,
            "(T) ASC released after stop, STO held, F9"
        );
    }

    /// Wave 2 lead decisions D3/D4 at the monitor level (DOOR, 1 µm/count):
    /// (a) a hand-thrown door at 0.25 m/s (above the 0.184 m/s energy cap) with the core
    /// braking at 60 N never trips (governor damping only);
    /// (b) the drive pushing the door at 0.20 m/s (above the cap) trips EnergyLimit;
    /// (c) a floor-driven slide past x_min with the LSM in STO is reported, not tripped;
    /// (d) the drive pushing the door past x_min trips PositionLimit + UnintendedMotion.
    #[test]
    fn positive_drive_work_rule() {
        let p = presets::door();
        let slow_ok = |m: &mut Monitor| {
            m.step_slow(
                Tick(0),
                &PowerRaw {
                    v_bus_code: u16::MAX / 2,
                    v_hold_code: u16::MAX,
                    mains_ok: true,
                    ..PowerRaw::default()
                },
                &SafetyInRaw::default(),
                LevState::Moving,
            );
        };
        let coil = CoilCmd::CurrentRef([0.0; MAX_MAGNETS]);
        let cmd = |iq: f32| LsmCmd {
            iq_ref_a: iq,
            id_ref_a: 0.0,
            mode: LsmMode::Run,
        };
        let run = |m: &mut Monitor, v_mps: f64, x0: f64, f_n: f64, lsm: LsmCmd, n: u32| {
            let mut worst = SafetyVerdict::default();
            let mut x = x0;
            for k in 0..n {
                let t = 2 * u64::from(k);
                let r = raw(&p, t, fmath::floor(x * 1.0e6) as i64, f_n);
                let v = m.step_fast(Tick(t), &r, &coil, &lsm, k);
                if v.demand > worst.demand {
                    worst = v;
                }
                x += v_mps * 1.0e-4;
            }
            worst
        };
        // (a) thrown by hand, the core brakes (i_q* opposes v, measured 60 N).
        let mut m = Monitor::new(&p.safety, &p.design.sensors);
        slow_ok(&mut m);
        let w = run(&mut m, 0.25, 0.3, 60.0, cmd(-2.0), 2_000);
        assert!(
            w.demand < SafetyDemand::SafeStop,
            "(T) hand-thrown excess met by braking does not trip (D4): {:?}",
            w
        );
        // (b) the drive holds 0.20 m/s (KE 2.0 J > 1.69 J) with 30 N along v.
        let mut m = Monitor::new(&p.safety, &p.design.sensors);
        slow_ok(&mut m);
        let w = run(&mut m, 0.20, 0.3, 30.0, cmd(1.0), 2_000);
        assert!(
            w.faults.contains(FaultCode::EnergyLimit) && w.demand >= SafetyDemand::SafeStop,
            "(T) positive drive work above the energy cap trips (D4): {:?}",
            w
        );
        // (c) floor-driven slide past x_min (−10 mm) at −0.1 m/s with the LSM in STO.
        let mut m = Monitor::new(&p.safety, &p.design.sensors);
        slow_ok(&mut m);
        let sto = LsmCmd {
            mode: LsmMode::Sto,
            ..LsmCmd::default()
        };
        let w = run(&mut m, -0.1, -0.005, 0.0, sto, 3_000);
        assert!(
            !w.faults.contains(FaultCode::PositionLimit)
                && !w.faults.contains(FaultCode::UnintendedMotion),
            "(T) floor-driven excursion with the LSM in STO is not a trip (D3): {:?}",
            w
        );
        assert!(
            m.pos_report.count == 1 && m.pos_report.max_depth_counts > 10_000,
            "excursion reported (D3): {:?}",
            m.pos_report
        );
        // (d) the drive pushes the door past x_min (closing, 30 N along v).
        let mut m = Monitor::new(&p.safety, &p.design.sensors);
        slow_ok(&mut m);
        let w = run(&mut m, -0.05, -0.005, 30.0, cmd(-1.0), 3_000);
        assert!(
            w.faults.contains(FaultCode::PositionLimit)
                && w.faults.contains(FaultCode::UnintendedMotion),
            "(T) the drive pushing past x_min trips SLP/SDI (D3): {:?}",
            w
        );
    }
}
