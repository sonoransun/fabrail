//! Lift and land sequences, bumpless transfer to zero power, touchdown detection (PHYSICS §15). Owner: A1.
//!
//! Frozen API (W0): [`LiftLand`] with `start_lift`, `start_land(rate)` and `step`.
//! Lift time ≤ 0.1/0.3/1/2 s by class (LEV-012; `levitation.lift_time_max_s`); bumpless
//! transfer to zero power when |δg| < 50 µm (`bumpless_dg_m`); touchdown when |ġ| is
//! below `touchdown_gdot_mps` with a current step; landing speed 5/5/3/2 mm/s.
//!
//! Sequences (physics perspective §2.4; core design §C.4, F17):
//! - **Lift** from rest: the heave current ramps open-loop ([`LiftOff`]) until lift-off
//!   is sensed (the filtered gap leaves its rest value by > `unseat_liftoff_m`, 20 µm).
//!   The payload follows from the closed form at the lift-off current,
//!   m̂ = n·F(g_rest, i_lo)/g₀, with i_lo corrected for the detection delay (the early
//!   motion is cubic in time under a current ramp, so the delay is ≈ 3·Δg/ġ) and the
//!   current/eddy lag (τ_cl + τ_e). If the ramp reaches the current envelope without
//!   lift-off (e.g. a DRAWER above ≈ 20 kg on its 6.5 mm rollers, F17) the lift aborts
//!   (LiftTimeout) and the current ramps back down.
//! - Then a C² gap trajectory (quintic, peak acceleration [`LIFT_ACCEL_FRAC`]·g₀) moves
//!   the gap to the band-clamped zero-power gap g_zp(m̂) under the gap servo
//!   (PD + gap integral with the force feed-forward i_ff = i(g_ref, m̂·(g₀ − g̈_ref)/n)).
//!   When |g − g_zp| < 50 µm the heave loop switches bumplessly to zero power (the ZP
//!   integral is preset so the demand is continuous).
//! - **Land**: the ZP integral is frozen, the gap reference ramps at the landing speed to
//!   g_land + 0.2 mm (so the rollers take the load), and touchdown is declared when the
//!   reference is past g_land, the gap sits at the rollers and |ġ| stays below
//!   `touchdown_gdot_mps` for [`TOUCHDOWN_PERSIST_S`]; the current then ramps to zero
//!   ([`RELEASE_S`]) — the "current step".
//! - Holds and moves between servo set points serve the DRAWER seat (`seat.rs`).

use fabrail_params::derive::G0_MPS2;
use fabrail_params::{ParamError, SystemParams, derive};
use fabrail_types::{FaultCode, fmath};

use crate::filters::FirstOrderLp;
use crate::levitation::design::ElementModel;
use crate::msg::LevPhase;

/// Upper bound of the peak gap acceleration of lift/seat trajectories as a fraction of
/// g₀ (extra force ≤ 10 % of the weight; design choice).
pub const LIFT_ACCEL_FRAC: f32 = 0.10;

/// Lower bound of the peak gap acceleration, fraction of g₀ (used when the current
/// envelope leaves no headroom; the servo then saturates and the faults take over).
pub const LIFT_ACCEL_MIN_FRAC: f32 = 0.02;

/// Fraction of the envelope headroom a move may use (design choice).
pub const MOVE_HEADROOM_FRAC: f32 = 0.9;

/// Landing reference overshoot past the roller gap, m (physics perspective §2.4: "ramp
/// the gap to landing + 0.2 mm").
pub const LAND_EXTRA_M: f32 = 0.2e-3;

/// Touchdown persistence, s (design choice: 2 periods of the slowest heave closed loop
/// at ω ≈ 81 rad/s would be 0.15 s; 20 ms suffices because the gap is already at the
/// rollers and the rate test rejects transients).
pub const TOUCHDOWN_PERSIST_S: f32 = 0.02;

/// Time constant of the low-pass on the gap rate used by the touchdown test, s (design
/// choice, W2: half the touchdown persistence). The 100 Hz gap differentiator alone
/// carries ≈ 1 mm/s rms of rate noise at HEAVY's 5 µm gap noise (PHYSICS §13), which kept
/// the 1 mm/s touchdown test from persisting for 20 ms and ran a HEAVY landing into
/// SaturationPersist; low-passed at 10 ms the noise is ≈ 0.2 mm/s while a 2 mm/s descent
/// still reads above the threshold.
pub const TOUCHDOWN_RATE_TAU_S: f32 = 0.01;

/// Tolerance on "the gap sits at the rollers" for touchdown, m (design choice).
pub const TOUCHDOWN_GAP_TOL_M: f32 = 0.1e-3;

/// Current ramp-down after touchdown or at the end of a seat step, s (design choice).
pub const RELEASE_S: f32 = 0.02;

/// Time the phase reads `Transfer` after the switch to zero power, s (design choice).
pub const TRANSFER_S: f32 = 0.01;

/// Rest-gap averaging window at the start of a ramp, s (before any lift-off is
/// possible; design choice).
pub const REST_AVG_S: f32 = 0.005;

/// Hold at the ramp envelope before a lift is aborted, s (covers τ_cl + τ_e and the
/// lift-off detector's ≈ 10 ms of cubic motion; design choice).
pub const EXHAUST_HOLD_S: f32 = 0.03;

/// Ramp current envelope as a fraction of i_pulse (the hardware clamp; the margin keeps
/// the voltage-mode current overshoot below the monitor's CoilOvercurrent; design
/// choice). The DRAWER 48 kg unseat needs 11.20 A (93 %; F17).
pub const RAMP_ENVELOPE_FRAC: f64 = 0.98;

/// Lift/land time-out margin over the class limit (the requirement is tested in
/// simulation at 1×; the fault catches pathological cases; design choice).
pub const TIMEOUT_MARGIN: f32 = 2.0;

/// Extra landing time allowed beyond distance/rate before LandTimeout, s (design choice).
pub const LAND_TIMEOUT_EXTRA_S: f32 = 1.0;

/// Lift-off gap filter corner, Hz (design choice: attenuates the 0.3–5 µm in-band gap
/// noise ≈ 3× at 10/5 kHz while adding < 0.4 ms of delay).
pub const LIFTOFF_LP_HZ: f64 = 400.0;

/// What the heave loop does this release.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HeaveMode {
    /// Coils idle.
    #[default]
    Off,
    /// Open-loop current ramp (`i_ff_a` is the heave current).
    Ramp,
    /// Gap servo on (`g_ref_m`, `gdot_ref_mps`) with feed-forward `i_ff_a`.
    Servo,
    /// Zero-power control.
    Zp,
    /// Current ramp-down; `release_scale` multiplies the current at release start.
    Release,
}

/// What follows a gap move.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum After {
    /// Switch to zero power when within `bumpless_dg_m` of the target.
    #[default]
    Zp,
    /// Hold the target under the gap servo.
    Hold,
    /// Ramp the current down to zero (seat step).
    Release,
}

/// Sequencer stage.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Stage {
    /// Coils idle, at rest.
    #[default]
    Idle,
    /// Open-loop current ramp until lift-off.
    Ramp,
    /// Gap trajectory under the servo.
    Move,
    /// Servo hold at a fixed gap.
    Hold,
    /// Zero-power levitation.
    Zp,
    /// Landing ramp.
    Land,
    /// Current ramp-down.
    Release,
    /// Touched down / released; coils idle.
    Down,
}

/// Lift/land sequencer output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LiftLandOut {
    /// Phase.
    pub phase: LevPhase,
    /// Heave gap set point for this release, m.
    pub g_ref_m: f32,
    /// Feed-forward heave current, A.
    pub i_ff_a: f32,
    /// Gap-rate reference, m/s.
    pub gdot_ref_mps: f32,
    /// What the heave loop does.
    pub mode: HeaveMode,
    /// Release scale (1 → 0) in `HeaveMode::Release`.
    pub release_scale: f32,
    /// Mass from the lift-off closed form, kg (> 0 only on the detecting release).
    pub liftoff_kg: f32,
    /// Switch the heave loop to zero power this release (bumpless).
    pub transfer: bool,
    /// Bias current the zero-power band loop should start from, A (on `transfer`).
    pub transfer_bias_a: f32,
    /// Fault raised this release (LiftTimeout, LandTimeout).
    pub fault: Option<FaultCode>,
}

/// Lift-off history length (modelled force current per release): 51 ms at 10 kHz,
/// 102 ms at 5 kHz, longer than the ≈ 10–20 ms from lift-off to detection.
pub const LIFTOFF_HIST: usize = 512;

/// Open-loop current ramp with lift-off detection and the closed-form mass (F17).
///
/// The ramp command is passed through the modelled command-to-force lags (the current
/// loop τ_cl, the eddy lag τ_e, and the one-period LET delay) and recorded. When the
/// filtered gap has moved by the threshold, the lift-off current i_lo is the value for
/// which the rigid-body motion of the recorded force excess reproduces the measured
/// travel: x = (k/m)·∫(T − τ)·(i_e(τ) − i_lo)⁺ dτ, with k/m = n·k_i/m =
/// 2N·g₀/(M_pm + N·i_lo) at the lift-off force balance (PHYSICS §3; the negative
/// stiffness adds ≈ p²x ≲ 5 % of the acceleration by detection and is neglected). This
/// holds whether the ramp is still rising or already held at the envelope (the 48 kg
/// DRAWER case). The mass is then m̂ = n·F(g_rest, i_lo)/g₀.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiftOff {
    /// +1: the gap closes at lift-off (current rising); −1: the gap opens (current
    /// falling, a light DRAWER leaving the seat's upper face).
    pub dir: f32,
    /// Commanded heave current, A.
    pub i_cmd_a: f32,
    /// Ramp rate (magnitude), A/s.
    pub rate_a_per_s: f32,
    /// Current envelope (magnitude), A.
    pub i_max_a: f32,
    /// Rest gap (averaged over the first [`REST_AVG_S`]), m.
    pub g_rest_m: f32,
    /// Lift-off threshold, m.
    pub thr_m: f32,
    /// Time since the ramp started, s.
    pub t_s: f32,
    /// Rest-gap accumulator.
    pub acc: [f32; 2],
    /// Group delay of the lift-off gap filter, s.
    pub lp_delay_s: f32,
    /// Ramp start current, A.
    pub i0_a: f32,
    /// Time held at the envelope, s.
    pub t_env_s: f32,
    /// Gap low-pass.
    pub lp: FirstOrderLp,
    /// Release period, s.
    pub dt_s: f32,
    /// Per-release smoothing of the current-loop and eddy lags.
    pub a_lag: [f32; 2],
    /// Modelled coil current and force current, A.
    pub i_model: [f32; 2],
    /// Commands awaiting application (LET: [next period, the one after]).
    pub pending: [f32; 2],
    /// Magnet MMF M_pm and turns N (k/m at the force balance).
    pub m_pm_n: [f32; 2],
    /// Recorded force current, one sample per release (ring).
    pub hist: [f32; LIFTOFF_HIST],
    /// Next write index.
    pub head: u16,
    /// Valid samples.
    pub count: u16,
}

impl Default for LiftOff {
    fn default() -> LiftOff {
        LiftOff {
            dir: 1.0,
            i_cmd_a: 0.0,
            rate_a_per_s: 0.0,
            i_max_a: 0.0,
            g_rest_m: 0.0,
            thr_m: 0.0,
            t_s: 0.0,
            acc: [0.0; 2],
            lp_delay_s: 0.0,
            i0_a: 0.0,
            t_env_s: 0.0,
            lp: FirstOrderLp::default(),
            dt_s: 0.0,
            a_lag: [1.0; 2],
            i_model: [0.0; 2],
            pending: [0.0; 2],
            m_pm_n: [0.0; 2],
            hist: [0.0; LIFTOFF_HIST],
            head: 0,
            count: 0,
        }
    }
}

/// Result of one [`LiftOff::step`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LiftOffOut {
    /// Heave current command, A.
    pub i_cmd_a: f32,
    /// Lift-off detected: (rest gap m, lift-off current A).
    pub detected: Option<(f32, f32)>,
    /// The envelope was reached without lift-off.
    pub exhausted: bool,
}

/// Parameters of a [`LiftOff`] ramp.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RampSpec {
    /// Direction (+1 up, −1 down).
    pub dir: f32,
    /// Start current, A.
    pub i0_a: f32,
    /// Ramp rate, A/s.
    pub rate_a_per_s: f32,
    /// Envelope, A.
    pub i_max_a: f32,
    /// Lift-off threshold, m.
    pub thr_m: f32,
    /// Release period, s.
    pub dt_s: f32,
    /// Current-loop and eddy time constants, s.
    pub tau_s: [f32; 2],
    /// Magnet MMF M_pm and turns N.
    pub m_pm_n: [f32; 2],
}

impl LiftOff {
    /// Starts a ramp.
    pub fn start(spec: &RampSpec, lp: FirstOrderLp) -> LiftOff {
        let lag = |tau: f32| {
            if tau > 0.0 && spec.dt_s > 0.0 {
                -fmath::exp_m1f(-spec.dt_s / tau)
            } else {
                1.0
            }
        };
        LiftOff {
            dir: if spec.dir < 0.0 { -1.0 } else { 1.0 },
            i_cmd_a: spec.i0_a,
            i0_a: spec.i0_a,
            rate_a_per_s: fmath::absf(spec.rate_a_per_s),
            i_max_a: fmath::absf(spec.i_max_a),
            thr_m: spec.thr_m,
            lp: FirstOrderLp { y: 0.0, ..lp },
            lp_delay_s: lp_group_delay_s(),
            dt_s: spec.dt_s,
            a_lag: [lag(spec.tau_s[0]), lag(spec.tau_s[1])],
            i_model: [spec.i0_a; 2],
            pending: [spec.i0_a; 2],
            m_pm_n: spec.m_pm_n,
            ..LiftOff::default()
        }
    }

    /// One release: advances the ramp and tests for lift-off on the filtered gap.
    pub fn step(&mut self, gap_m: f32, gap_dot_mps: f32, dt_s: f32) -> LiftOffOut {
        let _ = gap_dot_mps;
        if self.t_s == 0.0 {
            self.lp.reset(gap_m);
        }
        let g = self.lp.step(gap_m);
        // Force-current model: the command applied over the period just ended passes
        // the current-loop lag, then the eddy lag.
        let applied = self.pending[1];
        self.i_model[0] += self.a_lag[0] * (applied - self.i_model[0]);
        self.i_model[1] += self.a_lag[1] * (self.i_model[0] - self.i_model[1]);
        if let Some(slot) = self.hist.get_mut(usize::from(self.head)) {
            *slot = self.i_model[1];
        }
        self.head = ((usize::from(self.head) + 1) % LIFTOFF_HIST) as u16;
        self.count = core::cmp::min(usize::from(self.count) + 1, LIFTOFF_HIST) as u16;
        self.t_s += dt_s;
        if self.t_s <= REST_AVG_S {
            self.acc[0] += g;
            self.acc[1] += 1.0;
            self.g_rest_m = self.acc[0] / self.acc[1];
        }
        let mut out = LiftOffOut {
            i_cmd_a: self.i_cmd_a,
            ..LiftOffOut::default()
        };
        let moved = self.dir * (self.g_rest_m - g);
        if self.t_s > REST_AVG_S && moved > self.thr_m {
            let i_lo = self.solve_liftoff(moved);
            out.detected = Some((self.g_rest_m, i_lo));
            return out;
        }
        if fmath::absf(self.i_cmd_a) >= self.i_max_a {
            // Hold at the envelope long enough for the lags and the detector, then abort.
            self.t_env_s += dt_s;
            if self.t_env_s >= EXHAUST_HOLD_S {
                out.exhausted = true;
            }
        } else {
            self.i_cmd_a = self.i_cmd_at(self.t_s);
        }
        self.pending = [self.i_cmd_a, self.pending[0]];
        out.i_cmd_a = self.i_cmd_a;
        out
    }

    /// Ramp command at time `t_s` after the start (clamped to the envelope), A.
    pub fn i_cmd_at(&self, t_s: f32) -> f32 {
        fmath::clampf(
            self.i0_a + self.dir * self.rate_a_per_s * fmath::fmaxf(t_s, 0.0),
            -self.i_max_a,
            self.i_max_a,
        )
    }

    /// Solves x = (2N·g₀/(M + N·i))·Σ(T − τ_j)·(dir·(i_e,j − i))⁺·Δt² for i (bisection
    /// over the recorded range), evaluated at T = now − filter delay. Returns the newest
    /// modelled current if the record cannot bracket the solution.
    fn solve_liftoff(&self, moved_m: f32) -> f32 {
        let n = usize::from(self.count);
        let dt = self.dt_s;
        let newest = self
            .hist
            .get((usize::from(self.head) + LIFTOFF_HIST - 1) % LIFTOFF_HIST)
            .copied()
            .unwrap_or(self.i_model[1]);
        if n < 2 || dt <= 0.0 {
            return newest;
        }
        let skip = (self.lp_delay_s / dt + 0.5) as usize;
        let [m_pm, turns] = self.m_pm_n;
        let g0 = G0_MPS2 as f32;
        // Samples oldest → newest up to T (the last `skip` samples are after T).
        let sample = |k: usize| -> f32 {
            let idx = (usize::from(self.head) + LIFTOFF_HIST - n + k) % LIFTOFF_HIST;
            self.hist.get(idx).copied().unwrap_or(0.0)
        };
        let m_used = n.saturating_sub(skip);
        let travel = |i: f32| -> f32 {
            let mut s = 0.0f32;
            for k in 0..m_used {
                let lever = (m_used - k) as f32 * dt;
                let ex = self.dir * (sample(k) - i);
                if ex > 0.0 {
                    s += lever * ex * dt;
                }
            }
            let den = m_pm + turns * i;
            if den > 0.0 {
                2.0 * turns * g0 / den * s
            } else {
                0.0
            }
        };
        let (mut lo, mut hi) = (f32::MAX, f32::MIN);
        for k in 0..m_used {
            let v = sample(k);
            lo = fmath::fminf(lo, v);
            hi = fmath::fmaxf(hi, v);
        }
        if lo.is_nan() || hi.is_nan() || lo >= hi {
            return newest;
        }
        // f(i) = travel(i) − moved is monotone in i (decreasing for dir = +1).
        let f = |i: f32| travel(i) - moved_m;
        let (f_lo, f_hi) = (f(lo), f(hi));
        if f_lo * f_hi > 0.0 {
            return newest;
        }
        for _ in 0..40 {
            let mid = 0.5 * (lo + hi);
            if f(mid) * f_lo > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }
}

/// Low-frequency group delay of the lift-off gap filter, 1/(2π·f_lp) (first-order
/// low-pass; the lift-off motion is ≫ this, so it acts as a pure delay), s.
fn lp_group_delay_s() -> f32 {
    1.0 / (core::f32::consts::TAU * LIFTOFF_LP_HZ as f32)
}

/// C² quintic gap trajectory from (g0, v0, 0) to (g1, 0, 0):
/// g = g0 + (g1 − g0)·h_p(s) + v0·T·h_v(s), s = t/T, with h_p = 10s³ − 15s⁴ + 6s⁵ and
/// h_v = s − 6s³ + 8s⁴ − 3s⁵ (h_v carries the initial velocity with zero end
/// conditions), or a constant-rate ramp (landings).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GapTraj {
    /// Start gap, m.
    pub g0_m: f32,
    /// End gap, m.
    pub g1_m: f32,
    /// Duration, s.
    pub dur_s: f32,
    /// Elapsed, s.
    pub t_s: f32,
    /// Initial rate, m/s.
    pub v0_mps: f32,
    /// Constant-rate ramp instead of the quintic.
    pub ramp: bool,
}

impl GapTraj {
    /// Quintic from `g0` to `g1` with peak |g̈| ≤ `a_max` (T = √(5.7735·|Δg|/a),
    /// 10√3/3 ≈ 5.7735 being the quintic's peak-acceleration factor), at least `t_min`.
    pub fn quintic(g0_m: f32, g1_m: f32, a_max: f32, t_min_s: f32) -> GapTraj {
        GapTraj::quintic_v0(g0_m, 0.0, g1_m, a_max, t_min_s)
    }

    /// Quintic starting with rate `v0_mps`: T also covers stopping the initial motion,
    /// T ≥ 2·|v0|/a (design choice bounding the h_v acceleration term by ≈ a).
    pub fn quintic_v0(g0_m: f32, v0_mps: f32, g1_m: f32, a_max: f32, t_min_s: f32) -> GapTraj {
        let d = fmath::absf(g1_m - g0_m);
        let t = if a_max > 0.0 {
            fmath::fmaxf(
                fmath::sqrtf(5.773_503 * d / a_max),
                2.0 * fmath::absf(v0_mps) / a_max,
            )
        } else {
            0.0
        };
        GapTraj {
            g0_m,
            g1_m,
            dur_s: fmath::fmaxf(t, t_min_s),
            t_s: 0.0,
            v0_mps,
            ramp: false,
        }
    }

    /// Constant-rate ramp from `g0` to `g1` at `rate_mps` (> 0).
    pub fn ramp(g0_m: f32, g1_m: f32, rate_mps: f32) -> GapTraj {
        let d = fmath::absf(g1_m - g0_m);
        GapTraj {
            g0_m,
            g1_m,
            dur_s: if rate_mps > 0.0 { d / rate_mps } else { 0.0 },
            t_s: 0.0,
            v0_mps: 0.0,
            ramp: true,
        }
    }

    /// A constant set point.
    pub fn hold(g_m: f32) -> GapTraj {
        GapTraj {
            g0_m: g_m,
            g1_m: g_m,
            ..GapTraj::default()
        }
    }

    /// Reference (g, ġ, g̈) at the current time.
    pub fn eval(&self) -> (f32, f32, f32) {
        if self.dur_s <= 0.0 || self.t_s >= self.dur_s {
            return (self.g1_m, 0.0, 0.0);
        }
        let t = self.dur_s;
        let s = fmath::clampf(self.t_s / t, 0.0, 1.0);
        let d = self.g1_m - self.g0_m;
        if self.ramp {
            return (self.g0_m + d * s, d / t, 0.0);
        }
        let s2 = s * s;
        let s3 = s2 * s;
        let s4 = s3 * s;
        let p = 10.0 * s3 - 15.0 * s4 + 6.0 * s4 * s;
        let dp = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / t;
        let ddp = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (t * t);
        // Velocity term v0·T·h_v(s).
        let hv = s - 6.0 * s3 + 8.0 * s4 - 3.0 * s4 * s;
        let dhv = 1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4;
        let ddhv = (-36.0 * s + 96.0 * s2 - 60.0 * s3) / t;
        let v = self.v0_mps;
        (
            self.g0_m + d * p + v * t * hv,
            d * dp + v * dhv,
            d * ddp + v * ddhv,
        )
    }

    /// Advances time.
    pub fn advance(&mut self, dt_s: f32) {
        self.t_s += dt_s;
    }

    /// True once the trajectory has ended.
    pub fn done(&self) -> bool {
        self.t_s >= self.dur_s
    }
}

/// Constants of the sequencer (f32 copies made in `new()`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LiftLandCfg {
    /// Element closed forms.
    pub elem: ElementModel,
    /// Zero-power gap g*, m.
    pub g_star_m: f32,
    /// Landing (roller) gap, m.
    pub g_land_m: f32,
    /// Modified zero-power band [g_min, g_max], m.
    pub band_m: [f32; 2],
    /// Bumpless transfer threshold, m.
    pub bumpless_dg_m: f32,
    /// Touchdown rate threshold, m/s.
    pub touchdown_gdot_mps: f32,
    /// Lift-off threshold, m.
    pub liftoff_m: f32,
    /// Lift ramp rate, A/s.
    pub ramp_a_per_s: f32,
    /// Current envelope for ramps (`RAMP_ENVELOPE_FRAC`·i_pulse), A.
    pub i_env_a: f32,
    /// Current-loop and eddy time constants [τ_cl, τ_e,nom], s.
    pub tau_lags_s: [f32; 2],
    /// Fast period, s.
    pub dt_s: f32,
    /// Trajectory peak acceleration, m/s².
    pub a_traj_mps2: f32,
    /// Default landing speed, m/s.
    pub land_speed_mps: f32,
    /// Lift-off gap filter (designed at the fast rate).
    pub lp: FirstOrderLp,
}

/// Lift/land sequencer.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LiftLand {
    /// Phase.
    pub phase: LevPhase,
    /// Elapsed time in the phase, s.
    pub t_s: f32,
    /// Landing rate, m/s.
    pub rate_mps: f32,
    /// Lift time limit, s.
    pub t_max_s: f32,
    /// Stage.
    pub stage: Stage,
    /// Constants.
    pub cfg: LiftLandCfg,
    /// Current ramp and lift-off detector.
    pub ramp: LiftOff,
    /// Gap trajectory (moves and landing).
    pub traj: GapTraj,
    /// Mass estimate used for targets and feed-forward, kg.
    pub m_hat_kg: f32,
    /// What follows the move.
    pub after: After,
    /// Hold gap, m.
    pub hold_g_m: f32,
    /// Time in the stage, s.
    pub stage_t_s: f32,
    /// Touchdown persistence timer, s.
    pub td_s: f32,
    /// Low-passed gap rate for the touchdown test ([`TOUCHDOWN_RATE_TAU_S`]), m/s.
    pub td_rate_mps: f32,
    /// Heave current applied over the last period (set by the caller before `step`), A.
    /// A landing release ramps down from it (bumpless), not from the servo feed-forward:
    /// on the rollers the feed-forward for the overshooting reference (HEAVY 12.9 A at
    /// g_land + 0.2 mm) exceeds the current actually flowing (≈ 6.5 A), and a release from
    /// it stepped the coils up by 6 A in one period (an 8 kW bus draw and BusUnder, W2).
    pub i_applied_a: f32,
    /// Landing time limit, s.
    pub land_t_max_s: f32,
    /// Heave current at release start, A.
    pub release_from_a: f32,
    /// The lift sequence targets zero power at the end (vs a seat hold).
    pub lifting: bool,
    /// Lift-off current of the last detected lift-off (delay-corrected), A.
    pub liftoff_i_a: f32,
}

impl LiftLand {
    /// Sequencer from the levitation tuning.
    pub fn new(p: &SystemParams) -> Result<LiftLand, ParamError> {
        let s = &p.design.suspension;
        let lt = &p.control.levitation;
        let t = derive::fast_period_s(p);
        if t.is_nan() || t <= 0.0 {
            return Err(ParamError::new(
                "control.rates.fast_decim",
                "fast period must be > 0",
            ));
        }
        let elem = ElementModel::new(p);
        let i_env = RAMP_ENVELOPE_FRAC * s.i_pulse_a;
        // Ramp rate: the preset value when given (DRAWER 200 A/s, F17), else reach the
        // rated-mass lift-off current at the rollers in 30 % of the lift time limit.
        let ramp = if lt.unseat_ramp_a_per_s > 0.0 {
            lt.unseat_ramp_a_per_s
        } else {
            let i_lo = fmath::abs(derive::bias_current(
                &p.design,
                s.g_land_m,
                p.design.carriage.m_rated_kg,
            ));
            let t_ramp = 0.3 * fmath::fmax(lt.lift_time_max_s, 0.05);
            fmath::fmax(i_lo, 0.1 * s.i_cont_a) / t_ramp
        };
        let lags = [
            derive::tau_current_loop_s(p) as f32,
            fmath::fmax(lt.tau_e_nominal_s, 0.0) as f32,
        ];
        Ok(LiftLand {
            t_max_s: lt.lift_time_max_s as f32,
            rate_mps: lt.land_speed_mps as f32,
            m_hat_kg: p.design.carriage.m_rated_kg as f32,
            cfg: LiftLandCfg {
                elem,
                g_star_m: s.g_star_m as f32,
                g_land_m: s.g_land_m as f32,
                band_m: [s.zp_band_m[0] as f32, s.zp_band_m[1] as f32],
                bumpless_dg_m: lt.bumpless_dg_m as f32,
                touchdown_gdot_mps: lt.touchdown_gdot_mps as f32,
                liftoff_m: lt.unseat_liftoff_m as f32,
                ramp_a_per_s: ramp as f32,
                i_env_a: i_env as f32,
                tau_lags_s: lags,
                dt_s: t as f32,
                a_traj_mps2: LIFT_ACCEL_FRAC * G0_MPS2 as f32,
                land_speed_mps: lt.land_speed_mps as f32,
                lp: FirstOrderLp::new(LIFTOFF_LP_HZ, 1.0 / t),
            },
            // Unset until the caller reports the applied current (NaN: release from the
            // feed-forward, the stand-alone sequencer behaviour).
            i_applied_a: f32::NAN,
            ..LiftLand::default()
        })
    }

    /// Band-clamped zero-power gap of the current mass estimate, m.
    pub fn zp_target_m(&self) -> f32 {
        let g = self.cfg.elem.zero_power_gap(self.m_hat_kg);
        let [lo, hi] = self.cfg.band_m;
        if lo < hi && g.is_finite() {
            fmath::clampf(g, lo, hi)
        } else if g.is_finite() {
            g
        } else {
            self.cfg.g_star_m
        }
    }

    /// Bias current that holds the current mass at gap `g` (0 inside the band), A.
    pub fn bias_at(&self, g_m: f32) -> f32 {
        let gz = self.cfg.elem.zero_power_gap(self.m_hat_kg);
        if fmath::absf(g_m - gz) < 1e-9 {
            return 0.0;
        }
        let f = self.m_hat_kg * G0_MPS2 as f32 / fmath::fmaxf(self.cfg.elem.n_lift, 1.0);
        self.cfg.elem.current_for(g_m, f)
    }

    /// Sets the mass estimate used for targets and feed-forward.
    pub fn set_mass(&mut self, m_kg: f32) {
        if m_kg.is_finite() && m_kg > 0.0 {
            self.m_hat_kg = m_kg;
        }
    }

    /// Starts a lift from the rollers (ramp up, then move to zero power).
    pub fn start_lift(&mut self) {
        self.start_ramp(1.0, 0.0, None);
    }

    /// Starts an open-loop ramp in direction `dir` from `i0_a`; after lift-off the gap
    /// moves to the zero-power target (`hold` = None) or to a servo hold at `hold`.
    pub fn start_ramp(&mut self, dir: f32, i0_a: f32, hold: Option<f32>) {
        let spec = RampSpec {
            dir,
            i0_a,
            rate_a_per_s: self.cfg.ramp_a_per_s,
            i_max_a: self.cfg.i_env_a,
            thr_m: self.cfg.liftoff_m,
            dt_s: self.cfg.dt_s,
            tau_s: self.cfg.tau_lags_s,
            m_pm_n: [self.cfg.elem.m_pm, self.cfg.elem.turns],
        };
        self.ramp = LiftOff::start(&spec, self.cfg.lp);
        self.stage = Stage::Ramp;
        self.stage_t_s = 0.0;
        self.t_s = 0.0;
        self.phase = LevPhase::Lifting;
        self.lifting = true;
        match hold {
            Some(g) => {
                self.after = After::Hold;
                self.hold_g_m = g;
            }
            None => self.after = After::Zp,
        }
    }

    /// Peak acceleration for a move from `g_from_m` to `g_to_m`: the quintic's upward
    /// acceleration peaks at s = 0.211 (after 6.7 % of an upward move, or at 93.3 % of a
    /// downward one), where the element must supply m̂(g₀ + a)/n; the move uses
    /// [`MOVE_HEADROOM_FRAC`] of the envelope headroom there, bounded to
    /// [`LIFT_ACCEL_MIN_FRAC`, `LIFT_ACCEL_FRAC`]·g₀. Short moves from a high-current
    /// hold (the DRAWER leaving its seat slot at 48 kg) thus shed current as fast as the
    /// I²t pulse budget needs (plan §2b item 4), and light loads move briskly.
    pub fn move_accel(&self, g_from_m: f32, g_to_m: f32) -> f32 {
        let g0 = G0_MPS2 as f32;
        let (a_min, a_max) = (LIFT_ACCEL_MIN_FRAC * g0, self.cfg.a_traj_mps2);
        let n = fmath::fmaxf(self.cfg.elem.n_lift, 1.0);
        let fw = self.m_hat_kg * g0 / n;
        if fw <= 0.0 {
            return a_min;
        }
        let frac = if g_to_m < g_from_m { 0.066_9 } else { 0.933_1 };
        let g_pk = g_from_m + frac * (g_to_m - g_from_m);
        let f_env = self.cfg.elem.force(g_pk, self.cfg.i_env_a);
        let a_head = g0 * (f_env / fw - 1.0);
        fmath::clampf(MOVE_HEADROOM_FRAC * a_head, a_min, a_max)
    }

    /// Starts a C² move from `g_from_m` to `g_to_m`, then `after`; `min_s` bounds the
    /// duration from below.
    pub fn start_move(&mut self, g_from_m: f32, g_to_m: f32, after: After, min_s: f32) {
        self.start_move_v0(g_from_m, 0.0, g_to_m, after, min_s);
    }

    /// [`LiftLand::start_move`] from a moving start (rate `v0_mps`), so the reference
    /// joins the actual motion (after lift-off the carriage is already rising).
    pub fn start_move_v0(
        &mut self,
        g_from_m: f32,
        v0_mps: f32,
        g_to_m: f32,
        after: After,
        min_s: f32,
    ) {
        let a = self.move_accel(g_from_m, g_to_m);
        self.traj = GapTraj::quintic_v0(g_from_m, v0_mps, g_to_m, a, min_s);
        self.after = after;
        self.hold_g_m = g_to_m;
        self.stage = Stage::Move;
        self.stage_t_s = 0.0;
    }

    /// Starts a controlled landing at `rate_mps` from the present gap (use
    /// [`LiftLand::start_land_from`]); this form keeps the frozen signature and lands
    /// from the last reference.
    pub fn start_land(&mut self, rate_mps: f32) {
        let (g, _, _) = self.traj.eval();
        let g0 = if g > 0.0 { g } else { self.cfg.g_star_m };
        self.start_land_from(g0, rate_mps);
    }

    /// Starts a controlled landing from gap `g_from_m` at `rate_mps` (≤ 0 uses the
    /// class landing speed).
    pub fn start_land_from(&mut self, g_from_m: f32, rate_mps: f32) {
        let r = if rate_mps > 0.0 {
            rate_mps
        } else {
            self.cfg.land_speed_mps
        };
        self.rate_mps = r;
        let g1 = self.cfg.g_land_m + LAND_EXTRA_M;
        self.traj = GapTraj::ramp(g_from_m, fmath::fmaxf(g1, g_from_m), r);
        self.stage = Stage::Land;
        self.stage_t_s = 0.0;
        self.t_s = 0.0;
        self.td_s = 0.0;
        // The touchdown rate filter starts at the commanded descent rate (never "at rest").
        self.td_rate_mps = r;
        self.land_t_max_s = self.traj.dur_s + LAND_TIMEOUT_EXTRA_S;
        self.phase = LevPhase::Landing;
        self.lifting = false;
    }

    /// Starts the current ramp-down from heave current `i_from_a`.
    pub fn start_release(&mut self, i_from_a: f32) {
        self.release_from_a = i_from_a;
        self.stage = Stage::Release;
        self.stage_t_s = 0.0;
    }

    /// Returns to idle (coils off).
    pub fn idle(&mut self) {
        self.stage = Stage::Idle;
        self.phase = LevPhase::Off;
        self.stage_t_s = 0.0;
        self.t_s = 0.0;
        self.lifting = false;
    }

    /// Declares zero-power levitation without a lift (caught in flight).
    pub fn set_levitated(&mut self) {
        self.stage = Stage::Zp;
        self.stage_t_s = TRANSFER_S;
        self.phase = LevPhase::Levitated;
        self.lifting = false;
    }

    /// One fast release from the heave gap (m) and its rate (m/s).
    pub fn step(&mut self, gap_m: f32, gap_dot_mps: f32, dt_s: f32) -> LiftLandOut {
        self.t_s += dt_s;
        self.stage_t_s += dt_s;
        let mut out = LiftLandOut {
            phase: self.phase,
            ..LiftLandOut::default()
        };
        match self.stage {
            Stage::Idle => {
                self.phase = LevPhase::Off;
            }
            Stage::Down => {
                // Touchdown after a landing, Off after a seat release, Timeout after an
                // aborted lift: the phase set when the release ended persists.
            }
            Stage::Ramp => {
                let r = self.ramp.step(gap_m, gap_dot_mps, dt_s);
                out.mode = HeaveMode::Ramp;
                out.i_ff_a = r.i_cmd_a;
                if let Some((g_rest, i_lo)) = r.detected {
                    self.liftoff_i_a = i_lo;
                    let m = self.cfg.elem.mass(g_rest, i_lo);
                    if m.is_finite() && m > 0.0 {
                        self.m_hat_kg = m;
                        out.liftoff_kg = m;
                    }
                    let target = match self.after {
                        After::Hold => self.hold_g_m,
                        _ => self.zp_target_m(),
                    };
                    let after = if self.after == After::Hold {
                        After::Hold
                    } else {
                        After::Zp
                    };
                    self.start_move_v0(gap_m, gap_dot_mps, target, after, 0.0);
                    return self.servo_out(out);
                } else if r.exhausted {
                    out.fault = Some(FaultCode::LiftTimeout);
                    self.start_release(r.i_cmd_a);
                    self.phase = LevPhase::Timeout;
                }
            }
            Stage::Move => {
                self.traj.advance(dt_s);
                if self.traj.done() {
                    match self.after {
                        After::Hold => {
                            self.stage = Stage::Hold;
                            if self.lifting {
                                self.phase = LevPhase::Levitated;
                                self.lifting = false;
                            }
                        }
                        After::Release => {}
                        After::Zp => {
                            let target = self.traj.g1_m;
                            if fmath::absf(gap_m - target) < self.cfg.bumpless_dg_m {
                                self.stage = Stage::Zp;
                                self.stage_t_s = 0.0;
                                // `Transfer` marks the end of a lift; a retarget of an
                                // already-levitated carriage (DRAWER slot exit) stays
                                // `Levitated` so motion is not interrupted.
                                self.phase = if self.lifting {
                                    LevPhase::Transfer
                                } else {
                                    LevPhase::Levitated
                                };
                                self.lifting = false;
                                out.transfer = true;
                                out.transfer_bias_a = self.bias_at(target);
                                out.phase = self.phase;
                                out.mode = HeaveMode::Zp;
                                out.g_ref_m = target;
                                return out;
                            }
                        }
                    }
                }
                if self.after == After::Release && self.traj.done() {
                    // The seat step's move ended: release.
                    let (g, _, _) = self.traj.eval();
                    let i = self.ff_current(g, 0.0);
                    self.start_release(i);
                    out.mode = HeaveMode::Release;
                    out.release_scale = 1.0;
                    out.i_ff_a = i;
                    out.phase = self.phase;
                    return out;
                }
                out = self.servo_out(out);
            }
            Stage::Hold => {
                self.traj = GapTraj::hold(self.hold_g_m);
                out = self.servo_out(out);
            }
            Stage::Zp => {
                if self.phase != LevPhase::Transfer || self.stage_t_s >= TRANSFER_S {
                    self.phase = LevPhase::Levitated;
                }
                out.mode = HeaveMode::Zp;
                out.g_ref_m = self.zp_target_m();
            }
            Stage::Land => {
                self.traj.advance(dt_s);
                out = self.servo_out(out);
                let past = out.g_ref_m >= self.cfg.g_land_m;
                let at_rollers = gap_m >= self.cfg.g_land_m - TOUCHDOWN_GAP_TOL_M;
                let k = fmath::fminf(dt_s / TOUCHDOWN_RATE_TAU_S, 1.0);
                if gap_dot_mps.is_finite() {
                    self.td_rate_mps += k * (gap_dot_mps - self.td_rate_mps);
                }
                if past && at_rollers && fmath::absf(self.td_rate_mps) < self.cfg.touchdown_gdot_mps
                {
                    self.td_s += dt_s;
                } else {
                    self.td_s = 0.0;
                }
                let i_now = if self.i_applied_a.is_finite() {
                    self.i_applied_a
                } else {
                    out.i_ff_a
                };
                if self.td_s >= TOUCHDOWN_PERSIST_S {
                    self.start_release(i_now);
                    self.lifting = false;
                } else if self.t_s > self.land_t_max_s {
                    out.fault = Some(FaultCode::LandTimeout);
                    self.start_release(i_now);
                }
            }
            Stage::Release => {
                let k = fmath::clampf(1.0 - self.stage_t_s / RELEASE_S, 0.0, 1.0);
                out.mode = HeaveMode::Release;
                out.release_scale = k;
                out.i_ff_a = self.release_from_a * k;
                if k <= 0.0 {
                    self.stage = Stage::Down;
                    if self.phase == LevPhase::Landing {
                        self.phase = LevPhase::Touchdown;
                    } else if self.phase != LevPhase::Timeout {
                        self.phase = LevPhase::Off;
                    }
                    out.mode = HeaveMode::Off;
                }
            }
        }
        // Lift time-out (LEV-012 limit × margin).
        if self.lifting
            && self.t_max_s > 0.0
            && self.t_s > TIMEOUT_MARGIN * self.t_max_s
            && out.fault.is_none()
        {
            out.fault = Some(FaultCode::LiftTimeout);
            self.lifting = false;
            let (g, _, _) = self.traj.eval();
            self.start_land_from(fmath::fmaxf(g, gap_m), self.cfg.land_speed_mps);
            self.phase = LevPhase::Timeout;
        }
        out.phase = self.phase;
        out
    }

    /// Feed-forward heave current for reference gap `g` and gap acceleration `gdd`:
    /// i(g, m̂·(g₀ − g̈)/n) (PHYSICS §3, §15), A.
    pub fn ff_current(&self, g_m: f32, gdd_mps2: f32) -> f32 {
        let n = fmath::fmaxf(self.cfg.elem.n_lift, 1.0);
        let f = self.m_hat_kg * (G0_MPS2 as f32 - gdd_mps2) / n;
        self.cfg.elem.current_for(g_m, f)
    }

    /// Servo output from the current trajectory.
    fn servo_out(&self, mut out: LiftLandOut) -> LiftLandOut {
        let (g, gd, gdd) = self.traj.eval();
        out.mode = HeaveMode::Servo;
        out.g_ref_m = g;
        out.gdot_ref_mps = gd;
        out.i_ff_a = self.ff_current(g, gdd);
        out.phase = self.phase;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    #[test]
    fn quintic_is_c2_and_bounded() {
        let t = GapTraj::quintic(6.25e-3, 5.0e-3, 0.49, 0.0);
        let mut tr = t;
        let mut peak_a = 0.0f32;
        let mut prev = tr.eval();
        assert_eq!(prev.0, 6.25e-3);
        while !tr.done() {
            tr.advance(1e-4);
            let e = tr.eval();
            peak_a = peak_a.max(e.2.abs());
            // Peak rate 1.875·Δg/T ≈ 19 mm/s ⇒ ≤ 2 µm per 0.1 ms step.
            assert!((e.0 - prev.0).abs() < 2.5e-6, "continuous");
            prev = e;
        }
        assert!(peak_a <= 0.49 * 1.001, "peak |g̈| ≤ a_max: {peak_a}");
        assert_eq!(tr.eval(), (5.0e-3, 0.0, 0.0));
    }

    #[test]
    fn ramp_lifts_off_and_aborts_at_envelope() {
        let p = presets::door();
        let mut ll = LiftLand::new(&p).expect("door");
        // Lift-off never sensed: the ramp reaches the envelope and aborts.
        ll.start_lift();
        let mut fault = None;
        for _ in 0..20_000 {
            let o = ll.step(6.25e-3, 0.0, 1e-4);
            if o.fault.is_some() {
                fault = o.fault;
                break;
            }
        }
        assert_eq!(fault, Some(FaultCode::LiftTimeout), "envelope abort");
        assert_eq!(ll.stage, Stage::Release);
    }

    #[test]
    fn landing_detects_touchdown() {
        let p = presets::door();
        let mut ll = LiftLand::new(&p).expect("door");
        ll.set_mass(100.0);
        ll.start_land_from(5.0e-3, 5.0e-3);
        // Kinematic "plant": follows the reference until the rollers stop it.
        let mut g = 5.0e-3f32;
        let mut phase = LevPhase::Landing;
        for _ in 0..30_000 {
            let o = ll.step(g, 0.0, 1e-4);
            if o.mode == HeaveMode::Servo {
                g = o.g_ref_m.min(6.25e-3);
            }
            phase = o.phase;
            if phase == LevPhase::Touchdown {
                break;
            }
        }
        assert_eq!(
            phase,
            LevPhase::Touchdown,
            "touchdown after the reference passes g_land"
        );
        assert!(
            ll.t_s < (1.25e-3 + 0.2e-3) / 5.0e-3 + 0.1,
            "within distance/rate + margin"
        );
    }
}
