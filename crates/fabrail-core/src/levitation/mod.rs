//! Zero-power hybrid PM-biased EMS levitation: modal control, observer, zero-power law,
//! lift/land and the DRAWER seat (PHYSICS §15). Owner: A1 (module list frozen by W0a).
//!
//! Frozen API (W0), composed by `fast::FastLoop` (A2): [`Levitation::new`],
//! [`Levitation::step`] → [`LevOutput`]. Per controlled mode (heave, pitch; roll on
//! DRAWER and HEAVY only) the Mizuno current-integral law i* = k_p·δg + k_d·δġ + κ·q,
//! q̇ = i − i_bias, with gains from `params::derive::zp_gains_discrete` (discrete design
//! at ω = 2.5·p on the ZOH plant with the LET delay, τ_cl and nominal τ_e; plan §2b
//! item 1), allocated to magnets by `modal`, with the null-space (warp) currents
//! integrated to zero.
//!
//! One release (`step`):
//! 1. gaps → modal coordinates (`modal::Modal::decompose`, invalid channels replaced by
//!    the rigid-body prediction); coil currents → modal currents T·i;
//! 2. the directive drives the sequencers: [`liftland::LiftLand`] (lift from rest,
//!    bumpless transfer, land, touchdown) and, for `RestPolicy::Seat`, [`seat::Seat`]
//!    (unseat ramp, slot hold, slot exit, z-seat step);
//! 3. per mode ([`ModeLoop`]): the observer (`observer::GapObserver`, 4ω) corrects with
//!    the gap, then heave runs zero power, the gap servo, an open-loop ramp or a release
//!    as the sequencer says; pitch/roll run zero power when heave does and servo the level
//!    attitude while heave servos the gap (a zero-power loop must be free to move);
//! 4. allocation A (+ the warp correction), the I²t reference clamp ([`I2tClamp`]: i_pulse
//!    while the (i² − i_cont²)·t budget lasts, i_cont after; plan §2b item 4, mirrored by
//!    the monitor), the applied modal command is fed back to the observers (so they
//!    always propagate what the coils were actually asked for), m̂ is updated, and the
//!    envelope/saturation persistence faults are raised.
//!
//! Directive semantics: `Off` de-energises (only valid at rest — the supervisor's job);
//! `Hold` keeps the present support state; `Lift`/`Levitate` lift from rest (or catch an
//! already-floating carriage: a gap well away from every rest surface enters zero power
//! directly, the SIL initial-levitated case); `Unseat`/`Seat` run the DRAWER seat steps
//! (other classes treat them as `Lift`/`Land`); `Land{rate}` lands at the given speed.

pub mod design;
pub mod liftland;
pub mod modal;
pub mod observer;
pub mod seat;
pub mod zeropower;

#[cfg(test)]
mod test_plant;

use fabrail_params::derive::G0_MPS2;
use fabrail_params::{ParamError, SystemParams, derive};
use fabrail_types::{FaultCode, FaultSet, MAX_MAGNETS, PosNm, RestPolicy, fmath};

use crate::filters::Differentiator;
use crate::msg::{LevDirective, LevPhase, Measurements, SeatStatus};

use design::{ElementModel, LevGains, ModeDesign};
use liftland::{After, HeaveMode, LiftLand, Stage};
use modal::Modal;
use observer::{CoupledObserver, GapObserver, PayloadEstimator};
use seat::Seat;
use zeropower::ZeroPower;

/// Rate-filter bandwidth for the sequencers' gap rate (lift-off delay, touchdown), Hz
/// (design choice: a tenth of the 10/5 kHz fast rate keeps the 0.3–5 µm gap noise below
/// the 1 mm/s touchdown threshold).
pub const GAP_RATE_BW_HZ: f64 = 100.0;

/// Fraction of the I²t budget after which the core clamps to i_cont, so the monitor's
/// integer CoilI2t mirror (which trips at the full budget) never trips first (design
/// choice: 5 % covers the code quantisation and the one-release offset).
pub const I2T_BUDGET_FRAC: f32 = 0.95;

/// Catch-in-flight window: a gap farther than this fraction of (g_land − g*) from the
/// rollers and outside the seat faces is taken as floating (design choice).
pub const CATCH_FRAC: f32 = 0.5;

/// Thrust-moment feed-forward current plausibility window, s (design choice: 2 ms,
/// about ten current-loop time constants, short enough that a phantom feed-forward
/// cannot wind the zero-power integral up before the check trips).
pub const FF_TRACK_S: f32 = 20.0e-3;

/// Tracking-error fraction above which the feed-forward is disabled (design choice).
pub const FF_TRACK_FRAC: f32 = 0.5;

/// Commanded pitch current below which the plausibility check is not evaluated, A
/// (design choice: well above the current-sensor noise).
pub const FF_TRACK_MIN_A: f32 = 0.05;

/// Seat approach (W1.5): closing speed above which a DRAWER near its slot starts the
/// descent to the seat clearance, m/s (design choice: well above the encoder-velocity
/// noise, well below every planned close).
pub const APPROACH_V_MIN_MPS: f32 = 0.02;

/// Seat approach margin on the descent distance v·t_move, dimensionless (design choice).
pub const APPROACH_MARGIN: f32 = 1.5;

/// Fixed seat-approach allowance beyond the slot entry, m (design choice: covers the
/// quintic's settling and one supervisor period at the class speed).
pub const APPROACH_EXTRA_M: f32 = 5.0e-3;

/// Staged slot exit lead (Wave 2 lead decision D2; PHYSICS §8, §15), m: a DRAWER driving
/// out of its captive seat starts the gap transfer to zero power this far before the
/// slot end, where the captive lip's lead-in chamfer (5 mm, `plant::body::
/// SEAT_LIP_CHAMFER_M`, W1.5) already retreats upward at a slope of ≈ 0.47, far faster
/// than the gap rises. The seat-clearance hold (≈ 10.8 A per coil at 48 kg) is the
/// dominant I²t cost of the exit; ending it 5 mm earlier and closing the gap (the PM
/// force rises as the gap closes, so the current falls toward the zero-power bias)
/// keeps the exit inside the I²t budget. Must not exceed the lip chamfer. Design choice.
pub const SLOT_EXIT_LEAD_M: f32 = 5.0e-3;

/// Levitation output for one fast release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LevOutput {
    /// Coil current references, A (I²t-clamped).
    pub coil_ref_a: [f32; MAX_MAGNETS],
    /// Modal deviation [heave m, pitch rad, roll rad].
    pub modal: [f32; 3],
    /// Payload estimate m̂, kg (non-safety).
    pub m_hat_kg: f32,
    /// Levitation phase.
    pub phase: LevPhase,
    /// DRAWER seat status.
    pub seat: SeatStatus,
    /// A coil reference hit the I²t clamp this release.
    pub saturated: bool,
    /// Faults (GapEnvelope, LiftTimeout, LandTimeout, SaturationPersist, SeatFailed,
    /// PayloadMismatch is raised by the supervisor from m̂).
    pub faults: FaultSet,
    /// Coils should be energised (false in `Off`/`Touchdown`: the caller may command
    /// `CoilCmd::Off`).
    pub energised: bool,
    /// Per-element motional-EMF feed-forward −k_i·ġ̂, V (for
    /// `current::CurrentLoops::set_emf_ff` when the base group routes it).
    pub emf_ff_v: [f32; MAX_MAGNETS],
    /// Estimated heave gap at the CoM projection, m.
    pub gap_heave_m: f32,
    /// Estimated heave gap rate, m/s.
    pub gap_dot_mps: f32,
    /// Pitch attitude estimate θ_y (the gap slope ∂g/∂x from the pitch observer while
    /// the loop runs, else the rigid-body fit), rad.
    pub pitch_rad: f32,
}

/// One mode's loop: observer, zero-power law and gap servo.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModeLoop {
    /// The mode is controlled.
    pub active: bool,
    /// Velocity observer (4ω).
    pub obs: GapObserver,
    /// Zero-power law (and, for heave, the band outer loop).
    pub zp: ZeroPower,
    /// Servo gains [k_p, k_d, k_I].
    pub servo: [f32; 3],
    /// Servo integral state, A.
    pub servo_i_a: f32,
    /// Latest corrected estimate (deviation, rate).
    pub est: (f32, f32),
    /// Coupled roll–sway extension (roll on HEAVY; D1). Inactive otherwise.
    pub sway: SwayLoop,
}

/// Coupled roll–sway state of a roll loop (PHYSICS §6, §15; Wave 2 decision D1): the
/// coupled observer replaces the single-DOF gap observer, and the sway feedback
/// k_Y·Ŷ + k_Ẏ·Ŷ̇ is added to the zero-power (or servo) demand.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SwayLoop {
    /// The coupled design is in use.
    pub active: bool,
    /// Coupled observer.
    pub obs: CoupledObserver,
    /// Sway gains [k_Y, k_Ẏ].
    pub k: [f32; 2],
    /// Latest sway estimate (Ŷ m, Ŷ̇ m/s).
    pub est: (f32, f32),
}

impl SwayLoop {
    /// The sway feedback term k_Y·Ŷ + k_Ẏ·Ŷ̇, A (0 when inactive).
    #[inline]
    pub fn term(&self) -> f32 {
        if self.active {
            let v = self.k[0] * self.est.0 + self.k[1] * self.est.1;
            if v.is_finite() { v } else { 0.0 }
        } else {
            0.0
        }
    }
}

impl ModeLoop {
    /// Loop for mode design `md` at fast period `dt_s`, zero-power band `band_m`
    /// (relative, heave only) and observer damping `zeta`.
    pub fn new(md: &ModeDesign, dt_s: f32, band_m: [f32; 2], zeta: f32) -> ModeLoop {
        if !md.active {
            return ModeLoop::default();
        }
        let sway = if md.sway.active {
            SwayLoop {
                active: true,
                obs: CoupledObserver::new(&md.sway.phi, &md.sway.gam, &md.sway.l),
                k: md.sway.k,
                est: (0.0, 0.0),
            }
        } else {
            SwayLoop::default()
        };
        ModeLoop {
            active: true,
            obs: GapObserver::from_model(&md.plant, dt_s, md.observer_rad_s, zeta),
            zp: ZeroPower::new(md.zp, band_m, dt_s),
            servo: md.servo,
            servo_i_a: 0.0,
            est: (0.0, 0.0),
            sway,
        }
    }

    /// Observer correction with the modal deviation `dg` and measured modal current
    /// (the coupled roll–sway observer when the loop runs the coupled design).
    pub fn estimate(&mut self, dg: f32, i_meas_a: f32) -> (f32, f32) {
        if self.sway.active {
            let x = self.sway.obs.step(dg, i_meas_a);
            self.est = (x[0], x[1]);
            self.sway.est = (x[2], x[3]);
        } else {
            self.est = self.obs.step(dg, i_meas_a);
        }
        self.est
    }

    /// Zero-power demand from the present estimate: the Mizuno law plus the sway term.
    pub fn zp_demand(&mut self, i_meas_a: f32) -> f32 {
        let (g, gd) = self.est;
        self.zp.step(g, gd, i_meas_a) + self.sway.term()
    }

    /// Bumpless zero-power start so the demand equals `u_now` at the present estimate.
    pub fn zp_bumpless(&mut self, u_now: f32) {
        let (g, gd) = self.est;
        self.zp.bumpless(u_now - self.sway.term(), g, gd, 0.0);
    }

    /// One zero-power release (observer + law); the demand is also recorded as the
    /// applied command (override with [`ModeLoop::set_applied`] after clamping).
    pub fn step_zp(&mut self, dg: f32, i_meas_a: f32) -> f32 {
        self.estimate(dg, i_meas_a);
        let u = self.zp_demand(i_meas_a);
        self.set_applied(u);
        u
    }

    /// Gap-servo demand from the present estimate: u_ff + k_p·e + k_d·ė + x_I with the
    /// gains scaled by `scale` (gain scheduling b_nom/b), integrating when `integrate`.
    pub fn servo_demand(
        &mut self,
        e: f32,
        e_dot: f32,
        u_ff: f32,
        scale: f32,
        integrate: bool,
    ) -> f32 {
        let [kp, kd, ki] = self.servo;
        let u = u_ff + scale * (kp * e + kd * e_dot) + self.servo_i_a;
        if integrate {
            self.servo_i_a += self.zp.dt_s * scale * ki * e;
        }
        u
    }

    /// Bumpless servo start: the integral makes the demand equal `u_now`.
    pub fn servo_bumpless(&mut self, u_now: f32, u_ff: f32, e: f32, e_dot: f32, scale: f32) {
        let [kp, kd, _] = self.servo;
        self.servo_i_a = u_now - u_ff - scale * (kp * e + kd * e_dot);
        if !self.servo_i_a.is_finite() {
            self.servo_i_a = 0.0;
        }
    }

    /// Records the modal command applied over the next period.
    pub fn set_applied(&mut self, u: f32) {
        self.obs.set_command(u);
        if self.sway.active {
            self.sway.obs.set_command(u);
        }
    }

    /// Re-initialises the observer at `dg` and zeroes the integrals.
    pub fn reset(&mut self, dg: f32, i_now_a: f32) {
        self.obs.reset_full(dg, i_now_a);
        self.obs.set_command(i_now_a);
        if self.sway.active {
            self.sway.obs.reset_full(dg, i_now_a);
            self.sway.obs.set_command(i_now_a);
            self.sway.est = (0.0, 0.0);
        }
        self.est = (dg, 0.0);
        self.zp.bumpless(0.0, 0.0, 0.0, 0.0);
        self.servo_i_a = 0.0;
    }
}

/// Coil I²t reference clamp (plan §2b item 4): per element the excess
/// E = ∫(i² − i_cont²)dt (floored at 0, recovering below i_cont) is compared with the
/// budget B = (i_pulse² − i_cont²)·t_pulse; while E < `I2T_BUDGET_FRAC`·B the reference
/// is clamped to ±i_pulse, afterwards to ±i_cont. The monitor's CoilI2t mirrors the
/// same budget on integer codes (it trips at 1.0·B).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct I2tClamp {
    /// i_cont, A.
    pub i_cont_a: f32,
    /// i_pulse, A.
    pub i_pulse_a: f32,
    /// Budget B, A²·s.
    pub budget_a2s: f32,
    /// Accumulated excess per element, A²·s.
    pub acc_a2s: [f32; MAX_MAGNETS],
}

impl I2tClamp {
    /// From the suspension limits.
    pub fn new(p: &SystemParams) -> I2tClamp {
        let s = &p.design.suspension;
        let (c, pu) = (s.i_cont_a as f32, s.i_pulse_a as f32);
        I2tClamp {
            i_cont_a: c,
            i_pulse_a: pu,
            budget_a2s: fmath::fmaxf((pu * pu - c * c) * s.t_pulse_s as f32, 0.0),
            acc_a2s: [0.0; MAX_MAGNETS],
        }
    }

    /// Limit for element `i` now, A.
    pub fn limit(&self, i: usize) -> f32 {
        let acc = self.acc_a2s.get(i).copied().unwrap_or(0.0);
        if acc < I2T_BUDGET_FRAC * self.budget_a2s {
            self.i_pulse_a
        } else {
            self.i_cont_a
        }
    }

    /// Accumulates the measured currents over `dt_s`.
    pub fn accumulate(&mut self, i_meas: &[f32; MAX_MAGNETS], n: usize, dt_s: f32) {
        let c2 = self.i_cont_a * self.i_cont_a;
        for (acc, i) in self.acc_a2s.iter_mut().zip(i_meas.iter()).take(n) {
            if i.is_finite() {
                *acc = fmath::fmaxf(*acc + (i * i - c2) * dt_s, 0.0);
            }
        }
    }
}

/// Constants of the orchestrator (f32 copies made in `new()`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LevConfig {
    /// Fast period, s.
    pub dt_s: f32,
    /// Zero-power gap g*, m.
    pub g_star_m: f32,
    /// Upper stop gap, m.
    pub g_up_m: f32,
    /// Landing (roller) gap, m.
    pub g_land_m: f32,
    /// Captive seat gap, m (0 = no seat).
    pub g_seat_m: f32,
    /// Seat vertical clearance, m.
    pub seat_clear_m: f32,
    /// Rest policy.
    pub rest: RestPolicy,
    /// Saturation / envelope persistence, s.
    pub sat_persist_s: f32,
    /// Closed stop position.
    pub x_closed: PosNm,
    /// Seat slot length, nm.
    pub slot_len_nm: i64,
    /// Element closed forms.
    pub elem: ElementModel,
    /// Nominal heave b = k_i/m_eff (gain scheduling reference).
    pub b_nom: f32,
}

/// Internal stage of the orchestrator relative to the sequencers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Support {
    /// Resting on a support (seat, rollers): coils idle.
    #[default]
    Rest,
    /// Coils in use (ramping, servo, zero power, releasing).
    Active,
}

/// Levitation controller (all modes, lift/land, seat).
#[derive(Clone, Debug, Default)]
pub struct Levitation {
    /// Gain design (f32, made in `new()`).
    pub gains: design::LevGains,
    /// Seat status reported while idle.
    pub idle_seat: SeatStatus,
    /// Constants.
    pub cfg: LevConfig,
    /// Modal map, allocation and warp loop.
    pub modal: Modal,
    /// Per-mode loops (heave, pitch, roll).
    pub loops: [ModeLoop; 3],
    /// Lift/land sequencer.
    pub lift: LiftLand,
    /// DRAWER seat sequencer.
    pub seat: Seat,
    /// Payload estimator.
    pub payload: PayloadEstimator,
    /// I²t reference clamp.
    pub i2t: I2tClamp,
    /// Heave gap-rate filter (sequencer rates).
    pub gap_rate: Differentiator,
    /// Support state.
    pub support: Support,
    /// Previous heave mode (bumpless switching).
    pub prev_mode: HeaveMode,
    /// Saturation persistence timer, s.
    pub sat_s: f32,
    /// Gap-envelope persistence timer, s.
    pub env_s: f32,
    /// Use the IMU-aided heave observer when both accelerometers are healthy.
    pub imu_fusion: bool,
    /// Last applied modal command [heave, pitch, roll], A.
    pub u_applied: [f32; 3],
    /// Last directive kind seen (edge detection).
    pub last_dir: u8,
    /// Last decomposition (fills invalid channels).
    pub last_modal: [f32; 3],
    /// A lift/unseat aborted: no automatic retry until the directive changes.
    pub retry_block: bool,
    /// LSM thrust lever arm about the CoM, h = −c_z (the thrust acts at the rail
    /// reference point; PHYSICS §5, §7), m.
    pub thrust_arm_m: f32,
    /// Pitch modal current of the thrust-moment feed-forward last commanded, A.
    pub ff_pitch_a: f32,
    /// The same feed-forward as the coils carry it (first-order current-loop lag
    /// τ_cl), A: the part of the measured modal current it explains.
    pub ff_pitch_meas_a: f32,
    /// Per-release current-loop lag factor 1 − e^{−T/τ_cl}.
    pub ff_alpha: f32,
    /// Lagged total commanded pitch modal current (loop demand + feed-forward), A.
    pub ff_cmd_lag_a: f32,
    /// Low-passed |measured − lagged commanded| pitch modal current, A.
    pub ff_trk_err_a: f32,
    /// Low-passed |lagged commanded| pitch modal current, A.
    pub ff_trk_mag_a: f32,
    /// The plausibility check failed: feed-forward off until the coils are next
    /// de-energised (a failed current path does not heal by itself).
    pub ff_blocked: bool,
    /// The feed-forward may act (floating under closed-loop control this release).
    pub ff_enabled: bool,
    /// Travel speed estimate from the motion loop (previous release), m/s (+ opens).
    pub travel_v_mps: f32,
}

impl Levitation {
    /// Designs the controller.
    pub fn new(p: &SystemParams) -> Result<Levitation, ParamError> {
        let gains: LevGains = design::design(p)?;
        let s = &p.design.suspension;
        let lt = &p.control.levitation;
        let modal = Modal::new(p)?;
        let dt = gains.dt_s;
        let g_star = s.g_star_m as f32;
        let band_rel = [
            s.zp_band_m[0] as f32 - g_star,
            s.zp_band_m[1] as f32 - g_star,
        ];
        let mut loops = [ModeLoop::default(); 3];
        for (k, (lp, md)) in loops.iter_mut().zip(gains.modes.iter()).enumerate() {
            let band = if k == 0 { band_rel } else { [0.0, 0.0] };
            *lp = ModeLoop::new(md, dt, band, gains.observer_zeta);
            if k == 0 && md.active {
                // Heave: unbiased estimates away from (g*, m*) (band edge, seat hold).
                lp.obs = GapObserver::from_model_dist(
                    &md.plant,
                    f64::from(md.p2_per_s2),
                    dt,
                    md.observer_rad_s,
                    gains.observer_zeta,
                );
            }
        }
        // Modified zero-power band loop on heave (per-element k_s, k_i at g*).
        let ks = derive::k_s(s, s.g_star_m, 0.0) as f32;
        let ki = derive::k_i(s, s.g_star_m, 0.0) as f32;
        loops[0].zp = loops[0].zp.with_outer(lt.zp_outer_bw_hz as f32, ks, ki);
        let idle_seat = if p.design.rest_policy == RestPolicy::Seat {
            SeatStatus::Seated
        } else {
            SeatStatus::NoSeat
        };
        let fs = if dt > 0.0 { 1.0 / f64::from(dt) } else { 0.0 };
        Ok(Levitation {
            cfg: LevConfig {
                dt_s: dt,
                g_star_m: g_star,
                g_up_m: s.g_up_m as f32,
                g_land_m: s.g_land_m as f32,
                g_seat_m: s.g_seat_m as f32,
                seat_clear_m: p.design.landing.seat_z_clearance_m as f32,
                rest: p.design.rest_policy,
                sat_persist_s: lt.sat_persist_s as f32,
                x_closed: PosNm::from_m_f64(p.design.rail.x_closed_m),
                slot_len_nm: fmath::round(p.design.landing.slot_len_m * 1.0e9) as i64,
                elem: ElementModel::new(p),
                b_nom: gains.modes[0].b_m_per_a_s2,
            },
            modal,
            loops,
            lift: LiftLand::new(p)?,
            seat: Seat::new(p)?,
            payload: PayloadEstimator::new(p)?,
            i2t: I2tClamp::new(p),
            gap_rate: Differentiator::new(GAP_RATE_BW_HZ, fs),
            thrust_arm_m: -(p.design.carriage.cg_m[2] as f32),
            ff_alpha: if gains.tau_cl_s > 0.0 {
                1.0 - fmath::expf(-dt / gains.tau_cl_s)
            } else {
                1.0
            },
            gains,
            idle_seat,
            ..Levitation::default()
        })
    }

    /// Enables the IMU-aided heave observer (optional fusion; off by default).
    pub fn set_imu_fusion(&mut self, on: bool) {
        self.imu_fusion = on;
    }

    /// Thrust-moment feed-forward (W1.5 integration, PHYSICS §5, §7, §15): the LSM
    /// thrust F_x acts at the rail reference point, a lever h = −c_z from the CoM, so it
    /// applies the pitch torque τ_y = h·F_x. Left to the feedback loop, that torque
    /// couples the travel loop into pitch (a top-hung DOOR seen at the encoder has an
    /// effective mass 1/(1/m + h²/J_y) ≈ 0.3·m above the pitch bandwidth, which drives
    /// the admittance unstable through the momentum observer). The pitch modal current
    /// i_p = h·F_x/(n·k_i·r̄_x) cancels it (a modal current i_p produces the torque
    /// −n·k_i·r̄_x·i_p about +y; `modal` conventions), added to `refs` after the I²t clamp
    /// and subtracted from the measured modal current at the next release so the
    /// observers and the zero-power law see only net torque. Only while floating under
    /// closed-loop control with pitch active; otherwise zero.
    pub fn thrust_ff(&mut self, f_x_n: f32, gap_m: f32, refs: &mut [f32; MAX_MAGNETS]) {
        let n = usize::from(self.modal.n_magnets);
        let rb = self.modal.r_bar_m[1];
        let ki = self.cfg.elem.k_i(gap_m, 0.0);
        let nl = self.cfg.elem.n_lift;
        let ip = if self.ff_enabled && f_x_n.is_finite() && rb > 0.0 && ki > 0.0 && nl > 0.0 {
            self.thrust_arm_m * f_x_n / (nl * ki * rb)
        } else {
            0.0
        };
        self.ff_pitch_a = if ip.is_finite() { ip } else { 0.0 };
        if self.ff_pitch_a == 0.0 {
            return;
        }
        let add = self.modal.allocate(&[0.0, self.ff_pitch_a, 0.0]);
        for (i, (r, a)) in refs.iter_mut().zip(add.iter()).enumerate().take(n) {
            let lim = self.i2t.limit(i);
            *r = fmath::clampf(*r + a, -lim, lim);
        }
    }

    /// True when the DRAWER is inside its seat slot (x within slot_len of closed).
    fn in_slot(&self, x: PosNm) -> bool {
        self.cfg.rest == RestPolicy::Seat
            && self.cfg.slot_len_nm > 0
            && x.diff_nm(self.cfg.x_closed) < self.cfg.slot_len_nm
    }

    /// Seat approach (W1.5; PHYSICS §15, plan §2b item 4). The captive seat holds the
    /// drawer within [g_seat − c, g_seat] over the slot (x − x_closed < slot_len), and a
    /// drawer floating at g* (above the upper lip) cannot enter it. While the drawer
    /// closes toward the slot at zero power (seat `Clear`), it descends to the clearance
    /// middle g_seat − c/2 in `SEAT_MOVE_FRAC·seat_time` before reaching the slot entry
    /// (zone slot_len + `APPROACH_MARGIN`·|v|·t_move + `APPROACH_EXTRA_M`) and holds it
    /// under the gap servo (`InSlot`); the z-seat step on arrival then only releases the
    /// current. Turning back before the slot (v > +`APPROACH_V_MIN_MPS`) ends the
    /// approach and the ordinary slot exit returns the gap to zero power. This mirrors
    /// the unseat (lift within the clearance, then drive out).
    fn seat_approach(&mut self, x: PosNm, g_h: f32) {
        if self.cfg.rest != RestPolicy::Seat || self.cfg.slot_len_nm <= 0 {
            return;
        }
        let v = self.travel_v_mps;
        if self.seat.approaching && v > APPROACH_V_MIN_MPS {
            self.seat.approaching = false;
        }
        if self.seat.status != SeatStatus::Clear
            || !matches!(self.lift.stage, Stage::Zp | Stage::Move)
            || v.is_nan()
            || v >= -APPROACH_V_MIN_MPS
        {
            return;
        }
        let t_move = seat::SEAT_MOVE_FRAC * self.seat.seat_time_s;
        let x_open = x.diff_m(self.cfg.x_closed);
        let zone = self.cfg.slot_len_nm as f32 * 1.0e-9
            + APPROACH_MARGIN * fmath::absf(v) * t_move
            + APPROACH_EXTRA_M;
        if x_open < zone {
            self.lift
                .start_move(g_h, self.seat.g_mid_m(), After::Hold, t_move);
            self.seat.status = SeatStatus::InSlot;
            self.seat.approaching = true;
        }
    }

    /// True when `g` is clearly away from every rest surface (floating).
    fn floating(&self, g: f32) -> bool {
        let c = &self.cfg;
        let far_rollers = g < c.g_land_m - CATCH_FRAC * (c.g_land_m - c.g_star_m);
        let off_seat = c.g_seat_m <= 0.0
            || g < c.g_seat_m - c.seat_clear_m - 4.0 * self.lift.cfg.liftoff_m
            || g > c.g_seat_m + 4.0 * self.lift.cfg.liftoff_m;
        far_rollers && off_seat && g > c.g_up_m
    }

    /// One fast release.
    pub fn step(&mut self, m: &Measurements, d: &LevDirective) -> LevOutput {
        let dt = self.cfg.dt_s;
        let n = usize::from(self.modal.n_magnets);
        let mut out = LevOutput {
            m_hat_kg: self.payload.m_hat_kg,
            ..LevOutput::default()
        };

        // 1. Gaps → modal coordinates (invalid channels from the rigid-body prediction).
        let mut gaps = m.gap_m;
        let valid = m.gap_ok & self.modal.lift_mask;
        if valid != self.modal.d_mask {
            let _ = self.modal.rebuild_decomposition(valid);
        }
        for (i, g) in gaps.iter_mut().enumerate().take(n) {
            if m.gap_ok & (1u8 << i) == 0 || !g.is_finite() {
                *g = self.modal.predict_gap(&self.last_modal, i);
            }
        }
        let md = self.modal.decompose(&gaps);
        let md = if md.iter().all(|v| v.is_finite()) {
            md
        } else {
            self.last_modal
        };
        self.last_modal = md;
        let g_h = md[0];
        let mut i_modal = self.modal.modal_current(&m.coil_i_a);
        // The thrust-moment feed-forward current (`thrust_ff`) balances the LSM torque:
        // the loops see only the current that produces net modal force.
        self.ff_pitch_meas_a += self.ff_alpha * (self.ff_pitch_a - self.ff_pitch_meas_a);
        // Current plausibility for the feed-forward: the coils must carry what was
        // commanded (a failed current path, or a plant tier without coil dynamics, would
        // leave the subtracted feed-forward as a phantom current in the zero-power
        // integral). |measured − commanded| and |commanded| are low-passed over
        // `FF_TRACK_S`; a mismatch above `FF_TRACK_FRAC` disables the feed-forward.
        let cmd = self.u_applied[1] + self.ff_pitch_a;
        self.ff_cmd_lag_a += self.ff_alpha * (cmd - self.ff_cmd_lag_a);
        let beta = if FF_TRACK_S > 0.0 {
            fmath::fminf(dt / FF_TRACK_S, 1.0)
        } else {
            1.0
        };
        let meas = i_modal.get(1).copied().unwrap_or(0.0);
        if meas.is_finite() && self.ff_cmd_lag_a.is_finite() {
            self.ff_trk_err_a += beta * (fmath::absf(meas - self.ff_cmd_lag_a) - self.ff_trk_err_a);
            self.ff_trk_mag_a += beta * (fmath::absf(self.ff_cmd_lag_a) - self.ff_trk_mag_a);
        }
        if let Some(ip) = i_modal.get_mut(1) {
            *ip -= self.ff_pitch_meas_a;
        }
        let gdot_f = self.gap_rate.step(g_h);
        let i_mean = if n > 0 {
            m.coil_i_a.iter().take(n).sum::<f32>() / n as f32
        } else {
            0.0
        };
        let in_slot = self.in_slot(m.x);

        // 2. Directive → sequencers.
        self.handle_directive(d, g_h, in_slot, &mut out);
        self.lift.i_applied_a = self.u_applied[0];
        let ll = self.lift.step(g_h, gdot_f, dt);
        if let Some(f) = ll.fault {
            out.faults.set(f);
            if f == FaultCode::LiftTimeout {
                self.retry_block = true;
            }
        }
        if ll.liftoff_kg > 0.0 {
            self.payload.set(ll.liftoff_kg);
        }
        self.seat_approach(m.x, g_h);
        // D2 staged exit: while driving out (opening) the InSlot → Clear hand-over uses
        // the slot end minus SLOT_EXIT_LEAD_M.
        let in_slot_seat = if self.travel_v_mps > APPROACH_V_MIN_MPS
            && self.seat.status == SeatStatus::InSlot
            && !self.seat.approaching
        {
            in_slot
                && m.x.diff_nm(self.cfg.x_closed)
                    < self.cfg.slot_len_nm
                        - fmath::round(f64::from(SLOT_EXIT_LEAD_M) * 1.0e9) as i64
        } else {
            in_slot
        };
        let so = self.seat.update(&self.lift, &ll, in_slot_seat, g_h, dt);
        if let Some(f) = so.fault {
            out.faults.set(f);
        }
        // DRAWER slot exit: retarget from the seat hold to zero power.
        if so.exit_slot {
            let target = self.lift.zp_target_m();
            self.lift.start_move(g_h, target, After::Zp, 0.0);
        }

        // 3. Per-mode loops.
        let r_bar = self.modal.r_bar_m;
        let coords = [g_h - self.cfg.g_star_m, r_bar[1] * md[1], r_bar[2] * md[2]];
        let energised = !matches!(ll.mode, HeaveMode::Off);
        let mut u = [0.0f32; 3];
        if !energised {
            for (lp, c) in self.loops.iter_mut().zip(coords.iter()) {
                lp.reset(*c, 0.0);
            }
            self.modal.warp_i_a = [0.0; MAX_MAGNETS];
            self.u_applied = [0.0; 3];
        } else {
            // Observers (heave optionally IMU-aided).
            let a_rel = m.rail_mps2[2] - m.imu_mps2[2];
            let use_imu = self.imu_fusion && m.imu_ok && m.rail_ok && a_rel.is_finite();
            for (k, (lp, (c, im))) in self
                .loops
                .iter_mut()
                .zip(coords.iter().zip(i_modal.iter()))
                .enumerate()
            {
                if !lp.active {
                    continue;
                }
                if k == 0 && use_imu {
                    lp.est = lp.obs.step_imu(*c, *im, a_rel);
                } else {
                    lp.estimate(*c, *im);
                }
            }
            u = self.mode_demands(&ll, i_modal);
        }

        // 4. Allocation, warp, clamps.
        let mut refs = self.modal.allocate(&u);
        if energised && matches!(ll.mode, HeaveMode::Zp | HeaveMode::Servo) {
            let w = self.modal.warp(&m.coil_i_a, dt);
            for (r, wi) in refs.iter_mut().zip(w.iter()).take(n) {
                *r += wi;
            }
        }
        self.i2t.accumulate(&m.coil_i_a, n, dt);
        let mut sat = false;
        let mut clipped_torque = [0.0f32; 2];
        for (i, r) in refs.iter_mut().enumerate().take(n) {
            let lim = self.i2t.limit(i);
            if !r.is_finite() {
                *r = 0.0;
                out.faults.set(FaultCode::NonFinite);
            }
            let c = fmath::clampf(*r, -lim, lim);
            if c != *r {
                sat = true;
                let k = self.modal.k_rows.get(i).copied().unwrap_or([0.0; 3]);
                let ki = self.cfg.elem.k_i(g_h, c);
                clipped_torque[0] += ki * k[1] * (*r - c);
                clipped_torque[1] += ki * k[2] * (*r - c);
            }
            *r = c;
        }
        if self.modal.hold_mask != 0 && energised {
            let hd = self.modal.hold_down(
                clipped_torque,
                &gaps,
                self.cfg.elem.mu0_ap,
                self.cfg.elem.turns,
            );
            for (r, h) in refs.iter_mut().zip(hd.iter()).take(n) {
                if *h > 0.0 {
                    *r = fmath::fminf(*h, self.i2t.i_pulse_a);
                }
            }
        }
        if !energised {
            refs = [0.0; MAX_MAGNETS];
        }
        // Applied modal command back to the observers.
        let applied = self.modal.modal_current(&refs);
        for (lp, a) in self.loops.iter_mut().zip(applied.iter()) {
            if lp.active {
                lp.set_applied(*a);
            }
        }
        self.u_applied = applied;
        self.prev_mode = ll.mode;
        if self.ff_trk_mag_a > FF_TRACK_MIN_A
            && self.ff_trk_err_a > FF_TRACK_FRAC * self.ff_trk_mag_a
        {
            self.ff_blocked = true;
        }
        if !energised {
            self.ff_blocked = false;
            self.ff_trk_err_a = 0.0;
            self.ff_trk_mag_a = 0.0;
            self.ff_cmd_lag_a = 0.0;
        }
        self.ff_enabled = energised
            && matches!(ll.mode, HeaveMode::Zp | HeaveMode::Servo)
            && self.loops[1].active
            && !self.ff_blocked;
        if !self.ff_enabled {
            self.ff_pitch_a = 0.0;
            self.ff_pitch_meas_a = 0.0;
        }

        // Payload estimate while floating under closed-loop control.
        if matches!(ll.mode, HeaveMode::Zp | HeaveMode::Servo) && ll.phase != LevPhase::Lifting {
            out.m_hat_kg = self.payload.step(g_h, i_mean, dt);
            self.lift.set_mass(out.m_hat_kg);
        } else {
            out.m_hat_kg = self.payload.m_hat_kg;
        }

        // Persistence faults.
        if sat {
            self.sat_s += dt;
        } else {
            self.sat_s = 0.0;
        }
        if self.sat_s > self.cfg.sat_persist_s && self.cfg.sat_persist_s > 0.0 {
            out.faults.set(FaultCode::SaturationPersist);
        }
        let floating = ll.mode == HeaveMode::Zp;
        let c = &self.cfg;
        let env_lo = c.g_up_m + 0.1 * (c.g_star_m - c.g_up_m);
        let env_hi = c.g_land_m - 0.1 * (c.g_land_m - c.g_star_m);
        if floating && (g_h < env_lo || g_h > env_hi) {
            self.env_s += dt;
        } else {
            self.env_s = 0.0;
        }
        if self.env_s > self.cfg.sat_persist_s && self.cfg.sat_persist_s > 0.0 {
            out.faults.set(FaultCode::GapEnvelope);
        }

        // EMF feed-forward per element: −k_i·ġ̂_i from the modal rates.
        let (gd_h, gd_p, gd_r) = (
            self.loops[0].est.1,
            self.loops[1].est.1,
            self.loops[2].est.1,
        );
        let rb = |v: f32| if v > 0.0 { 1.0 / v } else { 0.0 };
        for ((e, r), (k, g)) in out
            .emf_ff_v
            .iter_mut()
            .zip(refs.iter())
            .zip(self.modal.k_rows.iter().zip(gaps.iter()))
            .take(n)
        {
            // Element gap rate from the modal rates (u = r̄·θ ⇒ θ̇ = u̇/r̄).
            let gd = gd_h + k[1] * gd_p * rb(r_bar[1]) + k[2] * gd_r * rb(r_bar[2]);
            *e = if energised {
                crate::current::motional_emf_v(self.cfg.elem.k_i(*g, *r), gd)
            } else {
                0.0
            };
        }

        out.coil_ref_a = refs;
        let heave_dev = if ll.g_ref_m > 0.0 {
            g_h - ll.g_ref_m
        } else {
            g_h - self.cfg.g_star_m
        };
        out.modal = [heave_dev, md[1], md[2]];
        out.phase = ll.phase;
        out.seat = so.status;
        out.saturated = sat;
        out.energised = energised;
        out.gap_heave_m = g_h;
        out.pitch_rad = if energised && self.loops[1].active && r_bar[1] > 0.0 {
            self.loops[1].est.0 / r_bar[1]
        } else {
            md[1]
        };
        out.gap_dot_mps = if energised {
            self.loops[0].est.1
        } else {
            gdot_f
        };
        self.idle_seat = so.status;
        out
    }

    /// Directive handling (edge-triggered starts; see the module docs).
    fn handle_directive(&mut self, d: &LevDirective, g_h: f32, in_slot: bool, out: &mut LevOutput) {
        let seat_class = self.cfg.rest == RestPolicy::Seat;
        let stage = self.lift.stage;
        let at_rest = matches!(stage, Stage::Idle | Stage::Down);
        let floating = matches!(stage, Stage::Zp | Stage::Move | Stage::Hold);
        match *d {
            LevDirective::Off => {
                self.lift.idle();
            }
            LevDirective::Hold => {}
            LevDirective::Lift | LevDirective::Levitate | LevDirective::Unseat => {
                if at_rest && !self.retry_block {
                    let seated = seat_class && self.seat.status == SeatStatus::Seated;
                    if seated && !self.floating(g_h) {
                        self.seat.begin_unseat(&mut self.lift, g_h);
                    } else if self.floating(g_h) {
                        // Caught in flight: zero power entered bumplessly from zero
                        // current (`mode_demands` presets every integral).
                        self.lift.set_levitated();
                        self.lift.set_mass(self.payload.m_hat_kg);
                        self.seat.caught(in_slot);
                    } else {
                        self.lift.start_lift();
                        self.seat.lifting_from_rollers();
                    }
                }
            }
            LevDirective::Land { rate_mps } => {
                if seat_class && in_slot && floating && self.seat.can_seat() {
                    // A DRAWER inside its seat slot lands INTO the seat (the rollers are
                    // below the captive seat; SS1 is seat-first, plan §4b).
                    self.seat.begin_seat(&mut self.lift, g_h);
                } else if floating || stage == Stage::Ramp {
                    let (g, _, _) = self.lift.traj.eval();
                    let from = if stage == Stage::Zp || g <= 0.0 {
                        g_h
                    } else {
                        g
                    };
                    self.lift.start_land_from(from, rate_mps);
                }
            }
            LevDirective::Seat => {
                if !seat_class {
                    if floating {
                        self.lift.start_land_from(g_h, 0.0);
                    }
                } else if floating && self.seat.can_seat() {
                    if in_slot {
                        self.seat.begin_seat(&mut self.lift, g_h);
                    } else if !self.seat.seat_refused {
                        out.faults.set(FaultCode::SeatFailed);
                        self.seat.seat_refused = true;
                    }
                }
            }
        }
        if !matches!(d, LevDirective::Seat) {
            self.seat.seat_refused = false;
        }
        if !matches!(
            d,
            LevDirective::Lift | LevDirective::Levitate | LevDirective::Unseat
        ) {
            self.retry_block = false;
        }
    }

    /// Modal current demands for this release from the sequencer output.
    fn mode_demands(&mut self, ll: &liftland::LiftLandOut, i_modal: [f32; 3]) -> [f32; 3] {
        let mut u = [0.0f32; 3];
        let g_star = self.cfg.g_star_m;
        // Pitch/roll: zero power once floating under zero power; while the heave loop
        // servos the gap (lift moves, the seat-slot hold, landing) they servo the level
        // attitude instead (PD + angle integral), because a zero-power loop whose motion
        // is blocked (a seat face, a roller) would wind its current integral up (q̇ = i
        // with no restoring motion); idle during ramps and releases.
        let prev = self.prev_mode;
        let sat = self.sat_s > 0.0;
        for (k, (lp, im)) in self
            .loops
            .iter_mut()
            .zip(i_modal.iter())
            .enumerate()
            .skip(1)
        {
            if !lp.active {
                continue;
            }
            let (g, gd) = lp.est;
            let applied = self.u_applied.get(k).copied().unwrap_or(0.0);
            // Coupled roll–sway (D1): the sway term rides on top of either law.
            let sway = lp.sway.term();
            let v = match ll.mode {
                HeaveMode::Zp => {
                    if prev != HeaveMode::Zp {
                        lp.zp_bumpless(applied);
                    }
                    lp.zp_demand(*im)
                }
                HeaveMode::Servo => {
                    if prev != HeaveMode::Servo {
                        lp.servo_bumpless(applied - sway, 0.0, g, gd, 1.0);
                    }
                    lp.servo_demand(g, gd, 0.0, 1.0, !sat) + sway
                }
                HeaveMode::Ramp | HeaveMode::Release | HeaveMode::Off => 0.0,
            };
            if let Some(slot) = u.get_mut(k) {
                *slot = v;
            }
        }
        // Heave.
        let heave = &mut self.loops[0];
        let (g, gd) = heave.est;
        u[0] = match ll.mode {
            HeaveMode::Off => 0.0,
            HeaveMode::Ramp | HeaveMode::Release => ll.i_ff_a,
            HeaveMode::Servo => {
                let e = g + g_star - ll.g_ref_m;
                let e_dot = gd - ll.gdot_ref_mps;
                // Gain scheduling: b_nom / b(g, i) with b = n·k_i/m̂.
                let ki = self.cfg.elem.k_i(ll.g_ref_m, ll.i_ff_a);
                let n = fmath::fmaxf(self.cfg.elem.n_lift, 1.0);
                let b = ki * n / fmath::fmaxf(self.lift.m_hat_kg, 1e-3);
                let scale = if b > 0.0 && self.cfg.b_nom > 0.0 {
                    fmath::clampf(self.cfg.b_nom / b, 0.5, 2.0)
                } else {
                    1.0
                };
                if self.prev_mode != HeaveMode::Servo {
                    heave.servo_bumpless(self.u_applied[0], ll.i_ff_a, e, e_dot, scale);
                }
                // Conditional integration: hold the integral while saturated.
                heave.servo_demand(e, e_dot, ll.i_ff_a, scale, self.sat_s <= 0.0)
            }
            HeaveMode::Zp => {
                if ll.transfer || self.prev_mode != HeaveMode::Zp {
                    heave
                        .zp
                        .bumpless(self.u_applied[0], g, gd, ll.transfer_bias_a);
                }
                heave.zp.step(g, gd, i_modal[0])
            }
        };
        u
    }

    /// Returns to the idle (not levitated) state.
    pub fn reset(&mut self) {
        self.lift.idle();
        for lp in self.loops.iter_mut() {
            lp.reset(0.0, 0.0);
        }
        self.modal.warp_i_a = [0.0; MAX_MAGNETS];
        self.i2t.acc_a2s = [0.0; MAX_MAGNETS];
        self.prev_mode = HeaveMode::Off;
        self.sat_s = 0.0;
        self.env_s = 0.0;
        self.u_applied = [0.0; 3];
    }

    /// Mass that zero-power levitation at gap `g_m` implies (i = 0), kg.
    pub fn zero_power_mass(&self, g_m: f32) -> f32 {
        self.cfg.elem.mass(g_m, 0.0)
    }

    /// Weight of the current estimate, N.
    pub fn weight_n(&self) -> f32 {
        self.payload.m_hat_kg * G0_MPS2 as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::levitation::test_plant::{Harness, HeavePlant};
    use fabrail_params::presets;

    /// Lift from the rollers, levitate, land (DOOR/WALL/HEAVY; LEV-012 timing, bumpless
    /// transfer, touchdown).
    #[test]
    fn lift_transfer_and_land() {
        for p in [presets::door(), presets::wall(), presets::heavy()] {
            let s = p.design.suspension;
            let m = p.design.carriage.m_star_kg;
            let plant = HeavePlant::new(&p, m, s.g_land_m);
            let mut h = Harness::new(&p, plant);
            let dt = h.h_s * h.decim as f64;
            let t_max = p.control.levitation.lift_time_max_s;
            let (mut t, mut t_lev, mut g_min) = (0.0f64, None, 1.0f64);
            for _ in 0..((2.0 * t_max / dt) as usize) {
                let o = h.release(LevDirective::Lift);
                t += dt;
                g_min = g_min.min(h.plant.g_m);
                if o.phase == LevPhase::Levitated && t_lev.is_none() {
                    t_lev = Some(t);
                }
                let names: Vec<&str> = o.faults.iter().map(|f| f.name()).collect();
                assert!(
                    o.faults.is_empty(),
                    "{:?}: no faults during the lift at t = {t:.3}: {names:?} g {} i {}",
                    p.design.class,
                    h.plant.g_m,
                    h.plant.i_a[0]
                );
            }
            let t_lev = t_lev.expect("levitated");
            assert!(
                t_lev <= t_max,
                "{:?}: lift to Levitated in {t_lev:.3} s ≤ {t_max} s (LEV-012)",
                p.design.class
            );
            assert!(
                g_min > s.g_up_m + 0.2e-3,
                "{:?}: no upper-stop approach: {g_min}",
                p.design.class
            );
            let m_hat = f64::from(h.lev.payload.m_hat_kg);
            assert!(
                (m_hat / m - 1.0).abs() < 0.05,
                "{:?}: m̂ {m_hat} within 5 %",
                p.design.class
            );
            // Zero power: current → 0 at g*.
            let _ = h.run_for(&p, 1.5, LevDirective::Levitate);
            assert!(
                h.plant.i_a[0].abs() < 5e-3,
                "{:?}: zero-power current {}",
                p.design.class,
                h.plant.i_a[0]
            );
            assert!(
                (h.plant.g_m - s.g_star_m).abs() < 5e-6,
                "{:?}: floats at g*",
                p.design.class
            );
            // Land at the class speed, then touchdown and coils idle.
            let rate = p.control.levitation.land_speed_mps as f32;
            let dist = s.g_land_m + 0.2e-3 - s.g_star_m;
            let o = h.run_for(
                &p,
                dist / f64::from(rate) + 0.3,
                LevDirective::Land { rate_mps: rate },
            );
            assert_eq!(
                o.phase,
                LevPhase::Touchdown,
                "{:?}: touchdown",
                p.design.class
            );
            assert!(!o.energised && o.coil_ref_a.iter().all(|v| *v == 0.0));
            assert!(
                (h.plant.g_m - s.g_land_m).abs() < 1e-9,
                "{:?}: on the rollers",
                p.design.class
            );
        }
    }

    /// Zero power after a load step: the current returns to zero and the gap follows
    /// m(g) (DOOR +98.1 N → 4.5967 mm; LEV-004 analytic anchor, core §E.5).
    #[test]
    fn zero_power_load_step() {
        let p = presets::door();
        let s = p.design.suspension;
        let plant = HeavePlant::new(&p, 100.0, s.g_star_m);
        let mut h = Harness::new(&p, plant);
        let _ = h.run_for(&p, 1.0, LevDirective::Levitate);
        // +98.1 N = +10 kg at 9.81 m/s² (core §E.5 anchor convention).
        h.plant.m_kg += 10.0;
        let _ = h.run_for(&p, 2.0, LevDirective::Levitate);
        assert!(
            h.plant.i_a[0].abs() < 1e-3,
            "|i_ss| < 1 mA within 2 s (LEV-004): {}",
            h.plant.i_a[0]
        );
        let want = derive::zero_power_gap(
            h.plant.m_kg,
            s.g_star_m,
            derive::h_prime(s.h_m_m, s.mu_rec),
            100.0,
        );
        assert!((want - 4.5967e-3).abs() < 1e-7, "anchor 4.5967 mm: {want}");
        assert!(
            (h.plant.g_m - want).abs() < 2e-6,
            "gap follows m(g) ±2 µm (LEV-004): {}",
            h.plant.g_m
        );
        let m_hat = f64::from(h.lev.payload.m_hat_kg);
        assert!(
            (m_hat / h.plant.m_kg - 1.0).abs() < 0.01,
            "m̂ converges: {m_hat}"
        );
    }

    /// A DRAWER that fell onto its 6.5 mm rollers re-lifts at ≤ 20 kg and aborts above
    /// (the lift-off current exceeds the i_pulse envelope; F17, LEV-019).
    #[test]
    fn drawer_relift_envelope() {
        let p = presets::drawer();
        let s = p.design.suspension;
        for (m, ok) in [(15.0, true), (26.0, false)] {
            let mut plant = HeavePlant::new(&p, m, s.g_land_m);
            plant.x_m = 0.15;
            let mut h = Harness::new(&p, plant);
            let mut faults = FaultSet::EMPTY;
            let mut phase = LevPhase::Off;
            for _ in 0..3000 {
                let o = h.release(LevDirective::Lift);
                faults.merge(o.faults);
                phase = o.phase;
            }
            if ok {
                assert_eq!(
                    phase,
                    LevPhase::Levitated,
                    "{m} kg re-lifts from the rollers"
                );
                let names: Vec<&str> = faults.iter().map(|f| f.name()).collect();
                assert!(
                    faults.is_empty(),
                    "{m} kg re-lift without faults: {names:?}"
                );
            } else {
                assert!(
                    faults.contains(FaultCode::LiftTimeout),
                    "{m} kg: envelope abort"
                );
                assert!(
                    (h.plant.g_m - s.g_land_m).abs() < 1e-9,
                    "{m} kg stays on the rollers"
                );
            }
        }
    }

    /// Lift and float with the preset gap-sensor noise (DRAWER 0.3 … HEAVY 5 µm rms,
    /// core §C.2): stays levitated, current noise bounded, no faults.
    #[test]
    fn levitates_with_sensor_noise() {
        for p in [presets::door(), presets::wall(), presets::heavy()] {
            let s = p.design.suspension;
            let mut plant = HeavePlant::new(&p, p.design.carriage.m_star_kg, s.g_land_m);
            plant.noise_m = p.design.sensors.gap_noise_m;
            let mut h = Harness::new(&p, plant);
            let t_lift = 2.0 * p.control.levitation.lift_time_max_s;
            let mut faults = FaultSet::EMPTY;
            let n = (t_lift / (h.h_s * h.decim as f64)) as usize;
            for _ in 0..n {
                faults.merge(h.release(LevDirective::Lift).faults);
            }
            let (mut i2, mut k, mut g_dev) = (0.0f64, 0.0f64, 0.0f64);
            let mut phase = LevPhase::Off;
            for _ in 0..(1.0 / (h.h_s * h.decim as f64)) as usize {
                let o = h.release(LevDirective::Levitate);
                faults.merge(o.faults);
                phase = o.phase;
                i2 += h.plant.i_a[0] * h.plant.i_a[0];
                k += 1.0;
                g_dev = g_dev.max((h.plant.g_m - s.g_star_m).abs());
            }
            let names: Vec<&str> = faults.iter().map(|f| f.name()).collect();
            assert!(
                faults.is_empty(),
                "{:?}: no faults with noise: {names:?}",
                p.design.class
            );
            assert_eq!(phase, LevPhase::Levitated, "{:?}", p.design.class);
            let i_rms = (i2 / k).sqrt();
            let lim = 0.05 * s.i_cont_a;
            assert!(
                i_rms < lim,
                "{:?}: coil current noise {i_rms:.4} A rms < 5 % of i_cont",
                p.design.class
            );
            assert!(
                g_dev < 0.1 * (s.g_star_m - s.g_up_m),
                "{:?}: gap within 10 % of the stop margin",
                p.design.class
            );
        }
    }

    #[test]
    fn off_and_nonfinite_are_safe() {
        let p = presets::door();
        let mut lev = Levitation::new(&p).expect("door");
        let mut m = Measurements {
            gap_m: [5e-3; MAX_MAGNETS],
            gap_ok: 0x0F,
            ..Measurements::default()
        };
        let o = lev.step(&m, &LevDirective::Off);
        assert_eq!(o.phase, LevPhase::Off);
        assert!(!o.energised && o.coil_ref_a.iter().all(|v| *v == 0.0));
        // Caught in flight, then a NaN gap on every channel: references stay finite.
        let _ = lev.step(&m, &LevDirective::Levitate);
        m.gap_m = [f32::NAN; MAX_MAGNETS];
        let o = lev.step(&m, &LevDirective::Levitate);
        assert!(o.coil_ref_a.iter().all(|v| v.is_finite()));
    }
}
