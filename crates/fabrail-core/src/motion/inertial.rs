//! Inertial smoothing: skyhook absolute-velocity feedback above f_c from the carriage IMU, rail floor feed-forward, admission checks; research tier (PHYSICS §16.2). Owner: E1.
//!
//! Frozen API (W0): [`InertialSmoother::new`], [`InertialSmoother::admission`] →
//! [`Admission`] and [`InertialSmoother::step`] → force (N). Wave-1 additions (E1):
//! [`InertialSmoother::exit`] (the latched exit request), [`InertialSmoother::exit_plan`]
//! (the log-damped close to the seat), [`InertialSmoother::force_need_n`],
//! [`InertialSmoother::a_z_allow_mps2`], [`InertialSmoother::with_rail_feedforward`],
//! [`t_sky`], [`stroke_sine_m`] and [`residual_mps2`].
//!
//! Physics (PHYSICS §16.2; plan §4b). Along x a levitated drawer is already a free mass,
//! so smoothing means following only the low-frequency floor motion and staying
//! inertial above the crossover f_c (≈ 1 Hz). The carriage IMU drives skyhook
//! absolute-velocity feedback (leaky integral with a 0.1 Hz high-pass), c_sky =
//! 2ζ·m̂·ω_c; floor feed-forward comes only from the rail accelerometer (the structure
//! ADXL355 is used below 0.3 Hz only). Residual T_sky(f/f_c, ζ)·a_x with
//! T_sky = 1/√((1 − r²)² + (2ζr)²), plus IMU noise, g·θ_tilt, parasitic force/m and the
//! latency term 2πfL·a_HP (FAB-HAP-017…019, status R).
//!
//! **Law** (one fast release; the same form as core Isolation and HAVEN, core design
//! §C.4, exotic §3.2):
//! F = −k·x_rel − c_sky·v̂_abs + ĉ·v_rel + κ_ff·m̂·LP²(a_rail), with k = m̂·ω_c²,
//! c_sky = 2ζ·m̂·ω_c, v̂_abs ← λ·v̂_abs + T·a_imu, λ = e^(−2π·f_hp·T). For the ideal
//! loop m·ẍ_abs = −k·(x_abs − x_floor) − c_sky·ẋ_abs, so |X_abs/X_floor| = T_sky(f/f_c, ζ):
//! the drawer follows the floor below f_c and is left inertial above it. The term
//! ĉ·v_rel (ĉ = b_dist + c_rail, the levitated viscous drag of PHYSICS §9) cancels the
//! parasitic rail drag that would otherwise couple the floor into the drawer; F_hyst is
//! left in the residual. The rail feed-forward is off unless a caller sets it
//! ([`InertialSmoother::with_rail_feedforward`]; it adds an ε_sf·a_ff error term).
//!
//! **Force budget.** |F| ≤ F_cont (20 N DRAWER); bursts up to F_burst (90 N) for
//! ≤ t_burst (0.75 s), the burst allowance recovering at the rate it was used.
//!
//! **Exit.** The smoother latches an exit request ([`ExitReason`]) before any limit the
//! governor or monitor watches: stroke |x_rel| > standoff − margin, |v_rel| >
//! v_rel_frac·v_cap (0.8·v_cap), or a burst exhausted with the demand still above
//! F_cont (also: disabled, or non-finite inputs). Every exit, including mains loss, is a
//! log-damped close to the SEAT with a_brk = 2·a_max ([`InertialSmoother::exit_plan`]
//! → `TerminusPlanner::replan`), never an SS1 landing at the standoff.
//!
//! Admission to SmoothRideThrough (all must hold): the drawer is already open by at
//! least s_min = 25 mm + S_max,pred when the alert arrives (seated drawers never unseat
//! on an alert); v_rel,pred ≤ 0.8·v_cap; predicted stroke fits (∬HP(a_floor) ≤
//! standoff − margin); the force budget m·a_LP + μ_pad·m·|a_y| + c_rail·|v_rel| +
//! F_hyst ≤ F_cont (bursts ≤ 90 N for ≤ 0.75 s on DRAWER); a_z,pred ≤ 0.5·p²·(g_float −
//! g_up); mains_ok (or building UPS); E_sc ≥ 3·E_land + E_seat. The first failing check,
//! in this order, is the reported reason; any non-finite input refuses.
//!
//! Research tier: `control.motion.inertial.enable = false` in every preset, so
//! admission reports `Disabled` and `step` returns 0 N unless a study enables it. No
//! isolation or hardware performance is claimed.

use core::f64::consts::PI;

use fabrail_params::{ParamError, SystemParams, derive};
use fabrail_types::{PosNm, Tick, TravelDir, fmath};

use crate::motion::terminus::{Plan, TerminusPlanner};

/// Standard gravity for the tilt error term g·θ, m/s² (the params value; PHYSICS §16.2).
const G0_MPS2: f32 = derive::G0_MPS2 as f32;

/// Why admission was granted or refused (first failing check, in the order above).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum AdmissionReason {
    /// Every check passed.
    Admitted = 0,
    /// The mode is disabled in the parameters (every preset).
    #[default]
    Disabled = 1,
    /// Not open by at least s_min when the alert arrived (or seated).
    NotOpen = 2,
    /// v_rel,pred > 0.8·v_cap.
    Velocity = 3,
    /// Predicted stroke exceeds standoff − margin.
    Stroke = 4,
    /// Force budget exceeded (continuous or burst).
    Force = 5,
    /// z-headroom check failed.
    ZHeadroom = 6,
    /// Neither mains nor a building UPS.
    Power = 7,
    /// E_sc < 3·E_land + E_seat.
    Energy = 8,
}

/// Predicted conditions evaluated at alert time (choreographer / supervisor).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdmissionInputs {
    /// Current opening from the seat, m.
    pub open_m: f32,
    /// Predicted maximum stroke S_max,pred, m.
    pub s_max_pred_m: f32,
    /// Predicted relative speed, m/s.
    pub v_rel_pred_mps: f32,
    /// Speed cap, m/s.
    pub v_cap_mps: f32,
    /// Predicted continuous force need, N.
    pub f_need_n: f32,
    /// Predicted peak burst force, N.
    pub f_burst_pred_n: f32,
    /// Predicted burst duration, s.
    pub t_burst_pred_s: f32,
    /// Predicted vertical floor acceleration, m/s².
    pub a_z_pred_mps2: f32,
    /// Allowed vertical acceleration 0.5·p²·(g_float − g_up), m/s².
    pub a_z_allow_mps2: f32,
    /// Mains present.
    pub mains_ok: bool,
    /// Building UPS backs the mains input.
    pub ups: bool,
    /// Usable supercap energy, J.
    pub e_sc_j: f32,
    /// One landing's energy need, J.
    pub e_land_j: f32,
    /// One seat step's energy need, J.
    pub e_seat_j: f32,
}

/// Admission result.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Admission {
    /// Granted.
    pub admitted: bool,
    /// Reason (first failing check).
    pub reason: AdmissionReason,
    /// Standoff opening s_min = 25 mm + S_max,pred the drawer closes to, m.
    pub s_min_m: f32,
}

/// Why the smoother requests an exit to the log-damped seat (latched until `reset`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ExitReason {
    /// No exit requested.
    #[default]
    None = 0,
    /// The mode is disabled.
    Disabled = 1,
    /// |x_rel| beyond the stroke (standoff − margin).
    Stroke = 2,
    /// |v_rel| above v_rel_frac·v_cap.
    Velocity = 3,
    /// Burst allowance exhausted with the demand above F_cont.
    Force = 4,
    /// Non-finite input or estimated mass.
    Sensor = 5,
}

/// Skyhook inertial smoother.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InertialSmoother {
    /// Mode available.
    pub enable: bool,
    /// Crossover ω_c = 2π·f_c, rad/s.
    pub omega_c_rad_s: f32,
    /// Skyhook ζ.
    pub zeta: f32,
    /// Leaky-integrated IMU absolute velocity, m/s.
    pub v_abs_mps: f32,
    /// Burst timer, s.
    pub t_burst_s: f32,
    /// Leak factor λ = e^(−2π·f_hp·T) of the IMU integrator.
    pub lambda: f32,
    /// Fast period T, s.
    pub dt_s: f32,
    /// Continuous force limit F_cont, N.
    pub f_cont_n: f32,
    /// Burst force limit F_burst, N.
    pub f_burst_n: f32,
    /// Burst duration limit, s.
    pub t_burst_max_s: f32,
    /// Stroke about the standoff: standoff − margin, m.
    pub stroke_m: f32,
    /// Base of the admission opening s_min = base + S_max,pred, m.
    pub s_min_base_m: f32,
    /// v_rel,pred ≤ frac·v_cap (0.8).
    pub v_rel_frac: f32,
    /// Class speed cap min(v_open, v_close), m/s.
    pub v_cap_mps: f32,
    /// z-headroom fraction (0.5 in a_z ≤ 0.5·p²·(g_float − g_up)).
    pub z_headroom_frac: f32,
    /// Viscous drag compensation ĉ = b_dist + c_rail, N·s/m.
    pub c_drag_n_s_per_m: f32,
    /// Side-pad friction μ_pad (force budget).
    pub mu_pad: f32,
    /// Rail hysteresis drag F_hyst (force budget), N.
    pub f_hyst_n: f32,
    /// Sensing-to-actuation latency L (residual term 2πfL·a_HP), s.
    pub latency_s: f32,
    /// Rail-accelerometer feed-forward gain κ_ff (0 = off, the default).
    pub ff_gain: f32,
    /// Feed-forward low-pass factor α = 1 − e^(−2π·f_ff·T) (two cascaded poles).
    pub ff_alpha: f32,
    /// Feed-forward low-pass states.
    pub ff_state: [f32; 2],
    /// The last output was limited by the force budget.
    pub saturated: bool,
    /// Latched exit request.
    pub exit: ExitReason,
}

/// `x` as f32 when finite and positive, else the error for `path`.
fn positive(x: f64, path: &'static str) -> Result<f32, ParamError> {
    if x.is_finite() && x > 0.0 {
        Ok(x as f32)
    } else {
        Err(ParamError::new(path, "must be finite and > 0"))
    }
}

impl InertialSmoother {
    /// Smoother from `control.motion.inertial`, the LSM thrust limits, the drag ledger
    /// and the class speed caps. The discrete coefficients are designed here in f64
    /// (λ from the fast period). When the mode is enabled, a non-positive f_c, ζ, f_hp,
    /// fast period or F_cont, or a standoff not above its margin, is rejected; a
    /// disabled smoother is always constructible.
    pub fn new(p: &SystemParams) -> Result<InertialSmoother, ParamError> {
        let inr = &p.control.motion.inertial;
        let dt = derive::fast_period_s(p);
        let dmp = &p.design.damping;
        let fin = |x: f64| if x.is_finite() { x } else { 0.0 };
        let mut s = InertialSmoother {
            enable: inr.enable,
            omega_c_rad_s: (2.0 * PI * fin(inr.f_c_hz)) as f32,
            zeta: fin(inr.zeta) as f32,
            lambda: if dt > 0.0 && inr.hp_hz.is_finite() && inr.hp_hz > 0.0 {
                // Leaky integrator pole e^(−2π·f_hp·T) (0.1 Hz; PHYSICS §16.2).
                fmath::exp(-2.0 * PI * inr.hp_hz * dt) as f32
            } else {
                1.0
            },
            dt_s: fin(dt) as f32,
            f_cont_n: fin(p.design.lsm.f_cont_n) as f32,
            f_burst_n: fin(inr.f_burst_n) as f32,
            t_burst_max_s: fin(inr.t_burst_s) as f32,
            stroke_m: fin(inr.standoff_m - inr.stroke_margin_m) as f32,
            s_min_base_m: fin(inr.s_min_base_m) as f32,
            v_rel_frac: fin(inr.v_rel_frac) as f32,
            v_cap_mps: fin(fmath::fmin(
                p.safety.v_open_max_mps,
                p.safety.v_close_max_mps,
            )) as f32,
            z_headroom_frac: fin(inr.z_headroom_frac) as f32,
            c_drag_n_s_per_m: fin(dmp.b_dist_n_s_per_m + dmp.c_rail_n_s_per_m) as f32,
            mu_pad: fin(p.design.guidance.pad_mu) as f32,
            f_hyst_n: fin(dmp.f_hyst_n) as f32,
            latency_s: fin(inr.latency_s) as f32,
            ..InertialSmoother::default()
        };
        if s.enable {
            positive(inr.f_c_hz, "control.motion.inertial.f_c_hz")?;
            positive(inr.zeta, "control.motion.inertial.zeta")?;
            positive(inr.hp_hz, "control.motion.inertial.hp_hz")?;
            positive(dt, "control.rates.fast_decim")?;
            positive(p.design.lsm.f_cont_n, "design.lsm.f_cont_n")?;
            positive(
                inr.standoff_m - inr.stroke_margin_m,
                "control.motion.inertial.standoff_m",
            )?;
            s.f_burst_n = fmath::fmaxf(s.f_burst_n, s.f_cont_n);
        }
        Ok(s)
    }

    /// Enables the rail-accelerometer floor feed-forward F_ff = κ_ff·m̂·LP²(a_rail): two
    /// cascaded first-order low-passes at `lp_hz` (ZOH poles). Off (κ_ff = 0) by default:
    /// with it the residual gains an ε_sf·a_ff term and, above both corners, the
    /// transmissibility rises by the factor 1 + κ_ff·(f_ff/f_c)² (the feed-forward adds
    /// κ_ff·m·ω_ff² to the spring term; PHYSICS §16.2). Research option.
    pub fn with_rail_feedforward(mut self, gain: f32, lp_hz: f32) -> InertialSmoother {
        if gain.is_finite() && lp_hz.is_finite() && lp_hz > 0.0 && self.dt_s > 0.0 {
            self.ff_gain = gain;
            self.ff_alpha = 1.0 - fmath::expf(-2.0 * core::f32::consts::PI * lp_hz * self.dt_s);
        } else {
            self.ff_gain = 0.0;
            self.ff_alpha = 0.0;
        }
        self.ff_state = [0.0; 2];
        self
    }

    /// Continuous skyhook gains for mass `m_hat_kg`: (k = m̂·ω_c², c_sky = 2ζ·m̂·ω_c).
    pub fn gains(&self, m_hat_kg: f32) -> (f32, f32) {
        let w = self.omega_c_rad_s;
        (m_hat_kg * w * w, 2.0 * self.zeta * m_hat_kg * w)
    }

    /// Predicted continuous force need m·|a_LP| + μ_pad·m·|a_y| + ĉ·|v_rel| + F_hyst
    /// (plan §4b; ĉ = b_dist + c_rail), N.
    pub fn force_need_n(&self, m_kg: f32, a_lp_mps2: f32, a_y_mps2: f32, v_rel_mps: f32) -> f32 {
        m_kg * fmath::absf(a_lp_mps2)
            + self.mu_pad * m_kg * fmath::absf(a_y_mps2)
            + self.c_drag_n_s_per_m * fmath::absf(v_rel_mps)
            + self.f_hyst_n
    }

    /// Allowed vertical floor acceleration frac·p²·(g_float − g_up) (0.5·p²·Δg; plan §4b:
    /// a z-headroom check, not a z-isolation claim), m/s² (0 if the gap is not above the
    /// upper stop).
    pub fn a_z_allow_mps2(&self, pole_rad_s: f32, g_float_m: f32, g_up_m: f32) -> f32 {
        let v = self.z_headroom_frac * pole_rad_s * pole_rad_s * (g_float_m - g_up_m);
        if v.is_finite() {
            fmath::fmaxf(v, 0.0)
        } else {
            0.0
        }
    }

    /// Admission check for SmoothRideThrough (plan §4b; FAB-HAP-019): every condition in
    /// the module docs, the first failing one reported. `s_min_m` is reported even when
    /// refused.
    pub fn admission(&self, inp: &AdmissionInputs) -> Admission {
        let s_min = self.s_min_base_m + fmath::fmaxf(inp.s_max_pred_m, 0.0);
        let refuse = |reason| Admission {
            admitted: false,
            reason,
            s_min_m: s_min,
        };
        if !self.enable {
            return refuse(AdmissionReason::Disabled);
        }
        // Comparisons are written so that NaN fails them (fail closed).
        // Already open by at least s_min (a seated drawer, open ≈ 0, never unseats).
        if !(inp.open_m >= s_min && s_min.is_finite()) {
            return refuse(AdmissionReason::NotOpen);
        }
        let v_ok = fmath::absf(inp.v_rel_pred_mps) <= self.v_rel_frac * inp.v_cap_mps;
        if !v_ok {
            return refuse(AdmissionReason::Velocity);
        }
        if !(inp.s_max_pred_m <= self.stroke_m && inp.s_max_pred_m >= 0.0) {
            return refuse(AdmissionReason::Stroke);
        }
        let cont_ok = inp.f_need_n <= self.f_cont_n;
        // A predicted burst above F_cont must stay ≤ F_burst for ≤ t_burst.
        let burst_ok = inp.f_burst_pred_n <= self.f_cont_n
            || (inp.f_burst_pred_n <= self.f_burst_n && inp.t_burst_pred_s <= self.t_burst_max_s);
        if !(cont_ok && burst_ok) {
            return refuse(AdmissionReason::Force);
        }
        let z_ok = fmath::absf(inp.a_z_pred_mps2) <= inp.a_z_allow_mps2;
        if !z_ok {
            return refuse(AdmissionReason::ZHeadroom);
        }
        if !(inp.mains_ok || inp.ups) {
            return refuse(AdmissionReason::Power);
        }
        // Reserve for three landings plus the seat step (plan §4b; SAF-009 hold-up rule).
        let e_ok = inp.e_sc_j >= 3.0 * inp.e_land_j + inp.e_seat_j;
        if !e_ok {
            return refuse(AdmissionReason::Energy);
        }
        Admission {
            admitted: true,
            reason: AdmissionReason::Admitted,
            s_min_m: s_min,
        }
    }

    /// One fast release: skyhook force (N) from the carriage IMU x-acceleration, the
    /// rail floor x-acceleration, relative velocity and relative position to the
    /// standoff, with estimated mass `m_hat_kg`. Limited to F_cont (bursts to F_burst for
    /// ≤ t_burst); latches [`Self::exit`] on the exit conditions. Returns 0 N when
    /// disabled or on a non-finite input (state held).
    pub fn step(
        &mut self,
        imu_x_mps2: f32,
        rail_x_mps2: f32,
        v_rel_mps: f32,
        x_rel_m: f32,
        m_hat_kg: f32,
    ) -> f32 {
        if !self.enable {
            self.latch(ExitReason::Disabled);
            self.saturated = false;
            return 0.0;
        }
        let finite = imu_x_mps2.is_finite()
            && rail_x_mps2.is_finite()
            && v_rel_mps.is_finite()
            && x_rel_m.is_finite()
            && m_hat_kg.is_finite()
            && m_hat_kg > 0.0;
        if !finite {
            self.latch(ExitReason::Sensor);
            self.saturated = false;
            return 0.0;
        }
        // Skyhook absolute velocity: leaky integral of the IMU (0.1 Hz high-pass).
        self.v_abs_mps = self.lambda * self.v_abs_mps + self.dt_s * imu_x_mps2;
        // Optional rail feed-forward: two cascaded first-order low-passes.
        let a_ff = if self.ff_gain != 0.0 {
            let [s1, s2] = self.ff_state;
            let s1 = s1 + self.ff_alpha * (rail_x_mps2 - s1);
            let s2 = s2 + self.ff_alpha * (s1 - s2);
            self.ff_state = [s1, s2];
            s2
        } else {
            0.0
        };
        let (k, c_sky) = self.gains(m_hat_kg);
        let f_cmd = -k * x_rel_m - c_sky * self.v_abs_mps
            + self.c_drag_n_s_per_m * v_rel_mps
            + self.ff_gain * m_hat_kg * a_ff;
        // Force budget: F_cont, bursts to F_burst while the allowance lasts.
        let mag = fmath::absf(f_cmd);
        let lim = if mag > self.f_cont_n {
            if self.t_burst_s < self.t_burst_max_s {
                self.t_burst_s += self.dt_s;
                self.f_burst_n
            } else {
                self.f_cont_n
            }
        } else {
            self.t_burst_s = fmath::fmaxf(self.t_burst_s - self.dt_s, 0.0);
            self.f_cont_n
        };
        self.saturated = mag > lim;
        let f = fmath::clampf(f_cmd, -lim, lim);
        // Exit before the governor's or monitor's limits.
        if fmath::absf(x_rel_m) > self.stroke_m {
            self.latch(ExitReason::Stroke);
        } else if fmath::absf(v_rel_mps) > self.v_rel_frac * self.v_cap_mps {
            self.latch(ExitReason::Velocity);
        } else if mag > self.f_cont_n && self.t_burst_s >= self.t_burst_max_s {
            self.latch(ExitReason::Force);
        }
        f
    }

    /// Latches the first exit reason.
    fn latch(&mut self, r: ExitReason) {
        if self.exit == ExitReason::None {
            self.exit = r;
        }
    }

    /// The latched exit request ([`ExitReason::None`] while smoothing may continue).
    pub fn exit(&self) -> ExitReason {
        self.exit
    }

    /// The exit move (plan §4b): a log-damped close from the carriage state (x, v, a
    /// along +x) to `seat`, re-planned with τ_eff and a_brk = 2·a_max
    /// (`TerminusPlanner::replan`; from near rest a launch then the terminus; the same
    /// path on mains loss). A `PlanKind::Infeasible` result is handed to the caller's
    /// braking fallback.
    pub fn exit_plan(
        &self,
        planner: &TerminusPlanner,
        seat: PosNm,
        t0: Tick,
        x: PosNm,
        v_mps: f32,
        a_mps2: f32,
    ) -> Plan {
        let dx = x.diff_m(seat);
        let dir = if dx >= 0.0 {
            TravelDir::Close
        } else {
            TravelDir::Open
        };
        // Approach speed and acceleration toward the seat (along dir).
        let sg = dir.sign();
        planner.replan(seat, dir, t0, fmath::absf(dx), sg * v_mps, sg * a_mps2)
    }

    /// Clears the integrator, burst timer, feed-forward state and exit latch.
    pub fn reset(&mut self) {
        self.v_abs_mps = 0.0;
        self.t_burst_s = 0.0;
        self.ff_state = [0.0; 2];
        self.saturated = false;
        self.exit = ExitReason::None;
    }
}

/// Skyhook transmissibility T_sky = 1/√((1 − r²)² + (2ζr)²), r = f/f_c (PHYSICS §16.2,
/// §21; the f32 twin of `params::derive::t_sky`; plan §4b: ≈ 0.24–0.31 at 2 Hz for
/// ζ 0.7–0.3, ≈ 0.04 at 5 Hz, f_c = 1 Hz).
pub fn t_sky(r: f32, zeta: f32) -> f32 {
    let c = 2.0 * zeta * r;
    let a = 1.0 - r * r;
    1.0 / fmath::sqrtf(a * a + c * c)
}

/// Relative stroke amplitude for a sinusoidal floor acceleration of amplitude
/// `a_floor_mps2` at `f_hz`: |X_rel| = (a/ω²)·r·√(r² + 4ζ²)·T_sky(r, ζ) (from
/// X_rel/X_floor = −(r² − 2iζr)/(1 − r² + 2iζr) of the ideal skyhook loop; PHYSICS
/// §16.2; → a/ω² well above f_c), m.
pub fn stroke_sine_m(f_hz: f32, f_c_hz: f32, zeta: f32, a_floor_mps2: f32) -> f32 {
    if !(f_hz > 0.0 && f_c_hz > 0.0) {
        return 0.0;
    }
    let w = 2.0 * core::f32::consts::PI * f_hz;
    let r = f_hz / f_c_hz;
    fmath::absf(a_floor_mps2) / (w * w)
        * r
        * fmath::sqrtf(r * r + 4.0 * zeta * zeta)
        * t_sky(r, zeta)
}

/// Residual floor-to-drawer x acceleration budget (plan §4b; FAB-HAP-018): the
/// transmitted part plus the added error terms, summed linearly (conservative).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ResidualTerms {
    /// Floor frequency f, Hz.
    pub f_hz: f32,
    /// Crossover f_c, Hz.
    pub f_c_hz: f32,
    /// Skyhook ζ.
    pub zeta: f32,
    /// Floor x acceleration amplitude a_x, m/s².
    pub a_x_mps2: f32,
    /// IMU noise (~1 mg rms), m/s².
    pub imu_noise_mps2: f32,
    /// Carriage tilt θ (g·θ: 1 mg per mrad), rad.
    pub tilt_rad: f32,
    /// Uncompensated parasitic force (F_hyst, slip), N.
    pub f_parasitic_n: f32,
    /// Mass, kg.
    pub m_kg: f32,
    /// Sensing-to-actuation latency L, s.
    pub latency_s: f32,
    /// High-passed acceleration amplitude a_HP the loop acts on, m/s².
    pub a_hp_mps2: f32,
    /// Feed-forward scale-factor error ε_sf (0 without feed-forward).
    pub eps_sf: f32,
    /// Feed-forward acceleration amplitude a_ff, m/s².
    pub a_ff_mps2: f32,
}

/// Residual T_sky(f/f_c, ζ)·a_x + IMU noise + g·|θ| + |F_par|/m + 2πfL·a_HP + ε_sf·a_ff
/// (plan §4b), m/s².
pub fn residual_mps2(t: &ResidualTerms) -> f32 {
    let r = if t.f_c_hz > 0.0 {
        t.f_hz / t.f_c_hz
    } else {
        0.0
    };
    let par = if t.m_kg > 0.0 {
        fmath::absf(t.f_parasitic_n) / t.m_kg
    } else {
        0.0
    };
    t_sky(r, t.zeta) * fmath::absf(t.a_x_mps2)
        + fmath::absf(t.imu_noise_mps2)
        + G0_MPS2 * fmath::absf(t.tilt_rad)
        + par
        + 2.0 * core::f32::consts::PI * t.f_hz * t.latency_s * fmath::absf(t.a_hp_mps2)
        + fmath::absf(t.eps_sf * t.a_ff_mps2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::terminus::{self, PlanKind, lin};
    use fabrail_params::{derive::LevMode, presets};
    use fabrail_types::{RailClass, Segment};

    /// A preset with the research-tier mode switched on (studies only).
    fn enabled(c: RailClass) -> SystemParams {
        let mut p = presets::by_class(c);
        p.control.motion.inertial.enable = true;
        p
    }

    #[test]
    fn disabled_by_default_refuses_admission() {
        let p = SystemParams::default();
        let s = InertialSmoother::new(&p).unwrap_or_default();
        let a = s.admission(&AdmissionInputs::default());
        assert!(!a.admitted);
        // Every preset ships with the mode off (research tier; plan §4b).
        for c in presets::CLASSES {
            let mut s = InertialSmoother::new(&presets::by_class(c)).expect("preset");
            assert!(!s.enable, "{c:?}: inertial.enable = false (plan §4b)");
            assert_eq!(
                s.admission(&AdmissionInputs::default()).reason,
                AdmissionReason::Disabled
            );
            assert_eq!(
                s.step(1.0, 1.0, 0.0, 0.0, 20.0),
                0.0,
                "{c:?}: 0 N when disabled"
            );
            assert_eq!(s.exit(), ExitReason::Disabled);
        }
    }

    /// Discrete closed loop (fast period T, ZOH) of the x-axis skyhook together with the
    /// zero-power pitch loop of the design model (`params::derive`), coupled both ways:
    /// the thrust acts at the CoM height h above the pole plane (moment s_tq·h·F on
    /// pitch) and the IMU picks up tilt and rotation, a_imu = F/m − ĉ·v/m +
    /// s_imu·(g·θ + h·θ̈). States: plant [x, v, f, θ, θ̇, i, i_e] (LSM force and coil
    /// current behind the current-loop lag τ_cl, eddy lag τ_e), controller [v̂,
    /// u_x (LET-delayed force), q (ZP current integral), u_p (LET-delayed current)].
    /// The skyhook row is the implemented law with the smoother's own λ and T; `gain`
    /// scales k and c_sky (gain-margin check); the floor is still (stability only).
    fn coupled_loop(
        p: &SystemParams,
        sm: &InertialSmoother,
        m_true: f64,
        m_hat: f64,
        s_tq: f64,
        s_imu: f64,
        gain: f64,
    ) -> lin::Mat {
        let t = f64::from(sm.dt_s);
        let tau_cl = derive::tau_current_loop_s(p);
        let tau_e = p.control.levitation.tau_e_nominal_s;
        let mm = derive::mode_model(p, LevMode::Pitch);
        let p2 = mm.k_eff_n_per_m / mm.m_eff_kg;
        let b = mm.k_i_n_per_a / mm.m_eff_kg;
        let zp = derive::zp_gains_discrete(p, LevMode::Pitch);
        let cr = &p.design.carriage;
        let s = &p.design.suspension;
        let lifts: Vec<f64> = s
            .stations
            .iter()
            .take(usize::from(s.n_magnets))
            .filter(|st| st.normal_sign > 0.0)
            .map(|st| st.z_m)
            .collect();
        let h = cr.cg_m[2] - lifts.iter().sum::<f64>() / lifts.len() as f64;
        let jy = cr.j_kgm2[1];
        let c_d = f64::from(sm.c_drag_n_s_per_m);
        let g0 = derive::G0_MPS2;
        // Continuous plant [x, v, f, θ, θ̇, i, i_e | u_x, u_p], augmented for Van Loan.
        let mut a = lin::zeros(9);
        a[0][1] = 1.0;
        a[1][1] = -c_d / m_true;
        a[1][2] = 1.0 / m_true;
        a[2][2] = -1.0 / tau_cl;
        a[2][7] = 1.0 / tau_cl;
        a[3][4] = 1.0;
        a[4][3] = p2;
        a[4][2] = s_tq * h / jy;
        a[5][5] = -1.0 / tau_cl;
        a[5][8] = 1.0 / tau_cl;
        // Force from the eddy-lagged current (derive's design model).
        let i_force = if tau_e > 0.0 {
            a[6][5] = 1.0 / tau_e;
            a[6][6] = -1.0 / tau_e;
            6
        } else {
            a[6][6] = -1.0 / tau_cl;
            5
        };
        a[4][i_force] = -b;
        let at: lin::Mat = a
            .iter()
            .map(|r| r.iter().map(|v| v * t).collect())
            .collect();
        let e = lin::expm(&at);
        let mut m = lin::zeros(11);
        for i in 0..7 {
            for j in 0..7 {
                m[i][j] = e[i][j];
            }
            m[i][8] = e[i][7];
            m[i][10] = e[i][8];
        }
        // IMU row over the plant states: (f − ĉ·v)/m + s_imu·(g·θ + h·θ̈).
        let mut imu = [0.0f64; 11];
        imu[1] = -c_d / m_true;
        imu[2] = 1.0 / m_true + s_imu * h * s_tq * h / jy;
        imu[3] = s_imu * (g0 + h * p2);
        imu[i_force] += -s_imu * h * b;
        // v̂⁺ = λ·v̂ + T·a_imu.
        m[7][7] = f64::from(sm.lambda);
        for (j, v) in imu.iter().enumerate() {
            m[7][j] += t * v;
        }
        // u_x⁺ = −k·x − c_sky·v̂⁺ + ĉ·v (x_rel = x, v_rel = v with the floor still).
        let w = f64::from(sm.omega_c_rad_s);
        let k = gain * m_hat * w * w;
        let c = gain * 2.0 * f64::from(sm.zeta) * m_hat * w;
        let row7 = m[7].clone();
        for (j, v) in row7.iter().enumerate() {
            m[8][j] = -c * v;
        }
        m[8][0] += -k;
        m[8][1] += c_d;
        // Zero-power pitch: q⁺ = q + T·i, u_p⁺ = k_p·θ + k_d·θ̇ + κ·q.
        m[9][9] = 1.0;
        m[9][5] = t;
        m[10][3] = zp.kp_a_per_m;
        m[10][4] = zp.kd_a_s_per_m;
        m[10][9] = zp.kappa_per_s;
        m
    }

    // req: FAB-HAP-017
    #[test]
    fn skyhook_discrete_stable() {
        // FAB-HAP-017 (status R): the skyhook loop, c_sky = 2ζ·m̂·ω_c on the leaky-
        // integrated IMU (0.1 Hz), is stable in the fast-group discrete-pole test together
        // with the zero-power pitch loop: spectral radius ρ < 1 of the 11-state coupled
        // closed loop (Gelfand ‖M^N‖^(1/N), N = 2²⁶ ≈ 6.7e7 periods, error < 1e-6) for both
        // coupling signs, m ∈ {m_op_min, m_rated}, m̂/m ∈ {0.8, 1, 1.25} and gain ×1, ×2
        // (6 dB margin). The slowest mode must still decay at ≥ 0.5·2π·f_hp (half the
        // integrator corner). DRAWER is the requirement's class (h = +80 mm: ρ ≈ 0.999925,
        // slowest decay ≈ 0.75 rad/s in every case). The other presets are held to the
        // nominal case only (m̂/m = 1, gain 1): the top-hung DOOR and WALL (CoM ≈ 1–1.5 m
        // below the rail, so a large thrust moment and tilt pickup) are stable there but
        // lose the 6 dB margin (ρ > 1 at gain 2 with m̂/m = 1.25), one reason the mode
        // stays DRAWER-only research tier.
        for c in presets::CLASSES {
            let robust = c == RailClass::Drawer;
            let p = enabled(c);
            let sm = InertialSmoother::new(&p).expect("enabled preset");
            let rho_pitch = derive::zp_spectral_radius(
                &p,
                LevMode::Pitch,
                &derive::zp_gains_discrete(&p, LevMode::Pitch),
                p.control.levitation.tau_e_nominal_s,
            );
            assert!(
                rho_pitch < 1.0,
                "{c:?}: pitch design loop alone stable (LEV-007)"
            );
            let t = f64::from(sm.dt_s);
            let sigma_min = 0.5 * 2.0 * PI * p.control.motion.inertial.hp_hz;
            let masses = [p.design.carriage.m_op_min_kg, p.design.carriage.m_rated_kg];
            let mut worst = 0.0f64;
            let (mrs, gains): (&[f64], &[f64]) = if robust {
                (&[0.8, 1.0, 1.25], &[1.0, 2.0])
            } else {
                (&[1.0], &[1.0])
            };
            for m in masses {
                for &mr in mrs {
                    for &gain in gains {
                        for s_tq in [1.0, -1.0] {
                            for s_imu in [1.0, -1.0] {
                                let mcl = coupled_loop(&p, &sm, m, mr * m, s_tq, s_imu, gain);
                                let rho = lin::spectral_radius(&mcl, 26);
                                let sigma = -rho.ln() / t;
                                worst = worst.max(rho);
                                assert!(
                                    rho < 1.0 && sigma >= sigma_min,
                                    "{c:?} m {m} m̂/m {mr} gain {gain} signs ({s_tq}, {s_imu}): \
                                     ρ = {rho}, slowest decay {sigma} rad/s < {sigma_min} \
                                     (FAB-HAP-017; PHYSICS §16.2)"
                                );
                            }
                        }
                    }
                }
            }
            assert!(worst < 1.0, "{c:?}: worst ρ = {worst}");
        }
    }

    /// All-pass admission inputs for DRAWER (s_min = 45 mm, 0.8·v_cap = 0.212 m/s).
    fn ok_inputs(s: &InertialSmoother) -> AdmissionInputs {
        AdmissionInputs {
            open_m: 0.100,
            s_max_pred_m: 0.020,
            v_rel_pred_mps: 0.10,
            v_cap_mps: s.v_cap_mps,
            f_need_n: 10.0,
            f_burst_pred_n: 60.0,
            t_burst_pred_s: 0.5,
            a_z_pred_mps2: 0.5,
            a_z_allow_mps2: 2.0,
            mains_ok: true,
            ups: false,
            e_sc_j: 100.0,
            e_land_j: 10.0,
            e_seat_j: 5.0,
        }
    }

    // req: FAB-HAP-019
    #[test]
    fn admission_rules() {
        // Truth table of the plan §4b admission list (FAB-HAP-019, status R): each
        // condition alone refuses with its own reason, at the boundary the inclusive
        // comparisons admit, the first failing check (in the documented order) is
        // reported, NaN refuses, and seated drawers never unseat. (E) exact.
        use AdmissionReason as R;
        let s = InertialSmoother::new(&enabled(RailClass::Drawer)).expect("drawer");
        assert!((s.v_rel_frac - 0.8).abs() < 1e-6 && (s.f_cont_n - 20.0).abs() < 1e-6);
        assert!((s.f_burst_n - 90.0).abs() < 1e-6 && (s.t_burst_max_s - 0.75).abs() < 1e-6);
        let base = ok_inputs(&s);
        let ok = s.admission(&base);
        assert!(
            ok.admitted && ok.reason == R::Admitted,
            "baseline admitted: {ok:?}"
        );
        assert!(
            (ok.s_min_m - (0.025 + 0.020)).abs() < 1e-7,
            "s_min = 25 mm + S_max,pred (plan §4b): {}",
            ok.s_min_m
        );
        let v08 = 0.8 * s.v_cap_mps;
        let e_min = 3.0 * base.e_land_j + base.e_seat_j;
        type Case = (&'static str, fn(&mut AdmissionInputs, f32, f32, f32), R);
        let cases: [Case; 20] = [
            ("seated (open 0)", |i, _, _, _| i.open_m = 0.0, R::NotOpen),
            (
                "just short of s_min",
                |i, _, _, _| i.open_m = 0.045 - 1e-4,
                R::NotOpen,
            ),
            ("exactly s_min", |i, _, _, _| i.open_m = 0.045, R::Admitted),
            (
                "v_rel = 0.8·v_cap",
                |i, v08, _, _| i.v_rel_pred_mps = v08,
                R::Admitted,
            ),
            (
                "v_rel > 0.8·v_cap",
                |i, v08, _, _| i.v_rel_pred_mps = v08 * 1.01,
                R::Velocity,
            ),
            (
                "receding v_rel > 0.8·v_cap",
                |i, v08, _, _| i.v_rel_pred_mps = -v08 * 1.01,
                R::Velocity,
            ),
            (
                "stroke = standoff − margin",
                |i, _, st, _| i.s_max_pred_m = st,
                R::Admitted,
            ),
            (
                "stroke too long",
                |i, _, st, _| i.s_max_pred_m = st + 1e-3,
                R::Stroke,
            ),
            ("F_need > F_cont", |i, _, _, _| i.f_need_n = 20.5, R::Force),
            (
                "burst > F_burst",
                |i, _, _, _| i.f_burst_pred_n = 95.0,
                R::Force,
            ),
            (
                "burst too long",
                |i, _, _, _| i.t_burst_pred_s = 0.8,
                R::Force,
            ),
            (
                "no burst, long",
                |i, _, _, _| {
                    i.f_burst_pred_n = 15.0;
                    i.t_burst_pred_s = 10.0;
                },
                R::Admitted,
            ),
            (
                "a_z at headroom",
                |i, _, _, _| i.a_z_pred_mps2 = 2.0,
                R::Admitted,
            ),
            (
                "a_z over headroom",
                |i, _, _, _| i.a_z_pred_mps2 = -2.1,
                R::ZHeadroom,
            ),
            (
                "UPS without mains",
                |i, _, _, _| {
                    i.mains_ok = false;
                    i.ups = true;
                },
                R::Admitted,
            ),
            (
                "no mains, no UPS",
                |i, _, _, _| i.mains_ok = false,
                R::Power,
            ),
            (
                "E_sc = 3·E_land + E_seat",
                |i, _, _, e| i.e_sc_j = e,
                R::Admitted,
            ),
            ("E_sc short", |i, _, _, e| i.e_sc_j = e - 0.01, R::Energy),
            (
                "velocity before force",
                |i, v08, _, _| {
                    i.v_rel_pred_mps = v08 * 2.0;
                    i.f_need_n = 100.0;
                    i.mains_ok = false;
                },
                R::Velocity,
            ),
            (
                "not open before all else",
                |i, _, _, _| {
                    i.open_m = 0.0;
                    i.e_sc_j = 0.0;
                    i.f_need_n = 1e6;
                },
                R::NotOpen,
            ),
        ];
        for (name, edit, want) in cases {
            let mut i = base;
            edit(&mut i, v08, s.stroke_m, e_min);
            let a = s.admission(&i);
            assert_eq!(a.reason, want, "{name}: {a:?} (plan §4b; FAB-HAP-019)");
            assert_eq!(a.admitted, want == R::Admitted, "{name}: admitted flag");
        }
        // Stroke check against the preset: standoff 50 mm − margin 5 mm.
        assert!(
            (s.stroke_m - 0.045).abs() < 1e-7,
            "stroke = standoff − margin"
        );
        // NaN anywhere numeric refuses (fail closed).
        for k in 0..11 {
            let mut i = base;
            let f = [
                &mut i.open_m,
                &mut i.s_max_pred_m,
                &mut i.v_rel_pred_mps,
                &mut i.v_cap_mps,
                &mut i.f_need_n,
                &mut i.f_burst_pred_n,
                &mut i.a_z_pred_mps2,
                &mut i.a_z_allow_mps2,
                &mut i.e_sc_j,
                &mut i.e_land_j,
                &mut i.e_seat_j,
            ];
            if let Some(x) = f.into_iter().nth(k) {
                *x = f32::NAN;
            }
            assert!(!s.admission(&i).admitted, "NaN in field {k} must refuse");
        }
        // Disabled (every preset) wins over everything.
        let off = InertialSmoother::new(&presets::drawer()).expect("drawer");
        assert_eq!(off.admission(&base).reason, R::Disabled);
        // The z-headroom helper: 0.5·p²·(g_float − g_up), never negative.
        let pole = 70.0f32;
        let allow = s.a_z_allow_mps2(pole, 5.0e-3, 1.2e-3);
        assert!(
            (allow - 0.5 * pole * pole * 3.8e-3).abs() < 1e-4,
            "a_z allow {allow}"
        );
        assert_eq!(s.a_z_allow_mps2(pole, 1.0e-3, 1.2e-3), 0.0);
        // The force-budget helper: m·a_LP + μ·m·|a_y| + ĉ·|v_rel| + F_hyst.
        let need = s.force_need_n(48.0, 0.1, 0.5, -0.2);
        let want = 48.0 * 0.1 + 0.15 * 48.0 * 0.5 + 3.0 * 0.2 + 0.2;
        assert!(
            (need - want).abs() < 1e-4,
            "force need {need} vs {want} (plan §4b)"
        );
    }

    /// Steady-state transmissibility |A_drawer/A_floor| of the implemented smoother on a
    /// free drawer mass with rail drag, sinusoidal floor at `f_hz` (ZOH, LET delay).
    fn simulated_t(p: &SystemParams, f_hz: f64, a_floor: f64) -> f64 {
        let mut sm = InertialSmoother::new(p).expect("enabled");
        let m = p.design.carriage.m_rated_kg;
        let c_true = f64::from(sm.c_drag_n_s_per_m);
        let dt = f64::from(sm.dt_s);
        let w = 2.0 * PI * f_hz;
        let xf_amp = a_floor / (w * w);
        let n_total = (30.0 / dt) as u64;
        let n_meas = ((10.0 * f_hz).round() / f_hz / dt).round() as u64; // whole cycles
        let (mut x, mut v, mut u) = (0.0f64, 0.0f64, 0.0f64);
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for k in 0..n_total {
            let t = k as f64 * dt;
            let (xf, vf) = (xf_amp * (w * t).sin(), xf_amp * w * (w * t).cos());
            let af = -xf_amp * w * w * (w * t).sin();
            let acc = (u - c_true * (v - vf)) / m;
            let f = sm.step(
                acc as f32,
                af as f32,
                (v - vf) as f32,
                (x - xf) as f32,
                m as f32,
            );
            if k >= n_total - n_meas {
                re += acc * (w * t).cos();
                im += acc * (w * t).sin();
            }
            x += v * dt + 0.5 * acc * dt * dt;
            v += acc * dt;
            u = f64::from(f); // LET: applied from the next period
        }
        assert_eq!(sm.exit(), ExitReason::None, "no exit in the linear range");
        2.0 * (re * re + im * im).sqrt() / n_meas as f64 / a_floor
    }

    #[test]
    fn residual_follows_t_sky() {
        // The implemented discrete loop reproduces T_sky(f/f_c, ζ) within ±5 % at 2 and
        // 5 Hz for the ideal sensor (FAB-HAP-018 allows ±20 % after the error terms; the
        // 0.1 Hz leak, LET delay and ZOH account for the few %). The plan's quoted values:
        // T_sky(2) ≈ 0.24–0.31 (ζ 0.7–0.3), T_sky(5) ≈ 0.04 (plan §4b); f32 t_sky equals
        // params::derive::t_sky.
        let p = enabled(RailClass::Drawer);
        let zeta = p.control.motion.inertial.zeta as f32;
        let fc = p.control.motion.inertial.f_c_hz as f32;
        for f in [2.0f64, 5.0] {
            let got = simulated_t(&p, f, 0.5);
            let want = f64::from(t_sky(f as f32 / fc, zeta));
            assert!(
                (got / want - 1.0).abs() <= 0.05,
                "f {f} Hz: simulated |T| = {got} vs T_sky = {want} (±5 %; plan §4b, PHYSICS §16.2)"
            );
        }
        assert!((t_sky(2.0, 0.7) - 0.2437).abs() < 1e-3 && (t_sky(2.0, 0.3) - 0.3095).abs() < 1e-3);
        assert!(
            (t_sky(5.0, 0.7) - 0.0400).abs() < 1e-3,
            "plan §4b quoted 0.04 at 5 Hz"
        );
        for (r, z) in [(0.3f32, 0.7f32), (1.0, 0.7), (2.0, 0.2), (5.0, 1.0)] {
            let d = derive::t_sky(f64::from(r), f64::from(z));
            assert!(
                (f64::from(t_sky(r, z)) - d).abs() <= 1e-6 * d,
                "t_sky f32 = derive"
            );
        }
        // Residual budget: with no error terms it is T_sky·a_x; each term adds linearly.
        let base = ResidualTerms {
            f_hz: 2.0,
            f_c_hz: 1.0,
            zeta: 0.7,
            a_x_mps2: 1.0,
            m_kg: 48.0,
            ..ResidualTerms::default()
        };
        let r0 = residual_mps2(&base);
        assert!((r0 - t_sky(2.0, 0.7)).abs() < 1e-6);
        let full = ResidualTerms {
            imu_noise_mps2: 0.0098,
            tilt_rad: 1.0e-3,
            f_parasitic_n: 0.2,
            latency_s: 0.2e-3,
            a_hp_mps2: 1.0,
            eps_sf: 0.02,
            a_ff_mps2: 0.1,
            ..base
        };
        let add = 0.0098
            + G0_MPS2 * 1.0e-3
            + 0.2 / 48.0
            + 2.0 * core::f32::consts::PI * 2.0 * 0.2e-3
            + 0.002;
        assert!(
            (residual_mps2(&full) - (r0 + add)).abs() < 1e-5,
            "residual = T_sky·a_x + noise + g·θ + F/m + 2πfL·a_HP + ε·a_ff (plan §4b)"
        );
        // 1 mrad of tilt is ≈ 1 mg (plan §4b).
        assert!((G0_MPS2 * 1.0e-3 / 9.80665e-3 - 1.0).abs() < 1e-6);
        // Stroke: 0.5 g at 4 Hz needs ≈ a/ω² (7.8 mm, the review's figure) × |H_rel| ≈ 1.06.
        let st = stroke_sine_m(4.0, 1.0, 0.7, 0.5 * 9.80665);
        assert!(
            (st / 7.76e-3 - 1.06).abs() < 0.02,
            "stroke {st} m at 0.5 g, 4 Hz"
        );
    }

    #[test]
    fn force_budget_bursts_then_exits() {
        // A demand above F_cont gets F_burst for t_burst (0.75 s), then F_cont with an
        // exit request (plan §4b force budget; DRAWER 20 N / 90 N).
        let mut s = InertialSmoother::new(&enabled(RailClass::Drawer)).expect("drawer");
        let n_burst = (0.75 / f64::from(s.dt_s)).round() as i64;
        let (mut bursts, mut last) = (0i64, 0.0f32);
        for k in 0..(n_burst + 100) {
            // An absolute-velocity estimate that asks for ~60 N (inside the stroke).
            s.v_abs_mps = -60.0 / s.gains(48.0).1;
            last = s.step(0.0, 0.0, 0.0, 0.0, 48.0);
            assert!(last <= 90.0 + 1e-3, "tick {k}: |F| ≤ F_burst");
            if last > 20.0 + 1e-3 {
                bursts += 1;
            }
        }
        assert!(
            (bursts - n_burst).abs() <= 2,
            "burst lasted {bursts} fast periods vs 0.75 s = {n_burst} (±2 for f32 timer)"
        );
        assert!((last - 20.0).abs() < 1e-3, "then F_cont: {last}");
        assert!(s.saturated);
        assert_eq!(s.exit(), ExitReason::Force);
        s.reset();
        assert_eq!(s.exit(), ExitReason::None);
        // Stroke and velocity exits come before the governor/monitor limits.
        s.step(0.0, 0.0, 0.0, s.stroke_m * 1.01, 48.0);
        assert_eq!(s.exit(), ExitReason::Stroke);
        s.reset();
        s.step(0.0, 0.0, 0.81 * s.v_cap_mps, 0.0, 48.0);
        assert_eq!(s.exit(), ExitReason::Velocity);
        s.reset();
        assert_eq!(s.step(f32::NAN, 0.0, 0.0, 0.0, 48.0), 0.0);
        assert_eq!(s.exit(), ExitReason::Sensor);
        // Feed-forward off by default; when on, a constant floor acceleration settles to
        // κ·m̂·a (two unity-gain low-passes).
        assert_eq!(s.ff_gain, 0.0);
        let mut ff = InertialSmoother::new(&enabled(RailClass::Drawer))
            .expect("drawer")
            .with_rail_feedforward(1.0, 0.3);
        let mut out = 0.0;
        for _ in 0..200_000 {
            ff.v_abs_mps = 0.0;
            out = ff.step(0.0, 0.1, 0.0, 0.0, 48.0);
        }
        // f32 first-order low-passes with α ≈ 1.9e-4 stall within ~2e-4 of their input
        // (the increment drops below half an ulp): 0.2 % tolerance.
        assert!(
            (out - 4.8).abs() < 0.01,
            "κ·m̂·LP²(a) → 4.8 N (±0.2 %): {out}"
        );
    }

    #[test]
    fn exit_is_log_damped_close_to_seat() {
        // Every exit closes to the SEAT through the terminus re-plan with a_brk = 2·a_max
        // (plan §4b): from the standoff at rest (launch + terminus), moving toward the seat
        // inside d_0 (braked τ′, peak deceleration ≤ 2·a_max), and receding (quintic, no
        // overshoot). The reference always ends exactly at the seat at rest.
        let p = enabled(RailClass::Drawer);
        let sm = InertialSmoother::new(&p).expect("drawer");
        let tp = TerminusPlanner::new(&p).expect("planner");
        let seat = PosNm(1_000_000_000);
        let at = |mm: f32| seat.offset_m(mm * 1e-3);
        // At rest at the 45 mm standoff: log-damped approach.
        let pl = sm.exit_plan(&tp, seat, Tick(100), at(45.0), 0.0, 0.0);
        assert_eq!(pl.kind, PlanKind::Approach);
        let next = pl.next.expect("terminus after the launch");
        assert!(next.target == seat && next.dir == TravelDir::Close && next.d_b_m > next.d_eps_m);
        // Moving toward the seat at 0.25 m/s from 30 mm (inside d_0 ≈ 46 mm): braked.
        let pl = sm.exit_plan(&tp, seat, Tick(100), at(30.0), -0.25, 0.0);
        assert_eq!(pl.kind, PlanKind::Braked);
        let Segment::Terminus(t) = pl.seg else {
            panic!("braked plan is a terminus")
        };
        let peak = t.j_mps3 * t.t_b_s;
        assert!(
            peak > tp.a_max_mps2 && peak <= 2.0 * tp.a_max_mps2 * 1.0001,
            "peak deceleration {peak} in (a_max, 2·a_max] (plan §4b)"
        );
        // Receding at 0.05 m/s from 40 mm: quintic that never passes the seat.
        let pl = sm.exit_plan(&tp, seat, Tick(100), at(40.0), 0.05, 0.0);
        assert_eq!(pl.kind, PlanKind::Quintic);
        for plan in [
            sm.exit_plan(&tp, seat, Tick(100), at(30.0), -0.25, 0.0),
            sm.exit_plan(&tp, seat, Tick(100), at(40.0), 0.05, 0.0),
        ] {
            for k in 0..=(plan.arrive.0 - 100 + 10) {
                let (x, _, _) = tp.eval(&plan.seg, Tick(100 + k));
                assert!(x >= seat, "never past the seat (overshoot 0; FAB-HAP-016)");
            }
            let (x, v, a) = tp.eval(&plan.seg, plan.arrive);
            assert!(x == seat && v == 0.0 && a == 0.0, "(E) at the seat at rest");
            assert!(terminus::done(&plan.seg, plan.arrive));
        }
    }
}
