//! Hybrid mass damper: LQR gains, saturation stack (stroke governor, power, force, rate), DVF fallback and the mode ladder (FAB-EXO-601..609). Owner: X2 (API frozen by W0a).
//!
//! EXOTIC §X7 (SENTINEL, research tier). f32 controller; gains come from offline LQR
//! (scipy) and are injected as [`HmdGains`]; [`HmdObserver`] supplies the state
//! estimate.
//!
//! * State z = [x_s, ẋ_s, x_r, ẋ_r] with stroke x_r = x_d − x_s; u_lqr = −K·ẑ. Sign: a
//!   positive force pushes the damper toward +x_r (it acts +u on the damper, −u on the
//!   structure), so the LSM power into the stroke is u·ẋ_r.
//! * Saturation stack, in priority order (exotic design §3.1):
//!   1. stroke governor: cubic weight w = ξ³, ξ = (|x_r| − 0.7·s_max)/(0.3·s_max)
//!      clamped to [0, 1] (Shanghai-Tower-style stroke-dependent damping). Inside the
//!      zone the controller may not feed power into the stroke (a command with
//!      u·ẋ_r > 0 is scaled by 1 − w; braking is kept) and the governor adds
//!      u_sg = −w·(c_sg·ẋ_r + k_sg·x_r), with k_sg = F_max/s_max (F_max at s_max) and
//!      c_sg = F_max²/P_max (the largest linear damping that stays inside both the
//!      force and the power limit up to the corner speed P_max/F_max);
//!   2. power |u·ẋ_r| ≤ P_max; 3. force |u| ≤ F_max; 4. rate ≤ F_max per 0.5 s
//!      (bumpless mode transfer).
//!
//!   Beyond s_max (w = 1) the command therefore never feeds power into outward motion.
//!   The governor bounds what the *controller* does near the ends; it cannot arrest a
//!   1,000 t damper driven past the design basis: the ironless trim LSM (24–121 kN) is
//!   about 10 % of the guideway restoring force at full stroke, so an over-design
//!   excursion ends on the designed sliding stops and end buffers (EXOTIC §X7; the
//!   ζ_add ≤ (μ/2)(S/X) bound, FAB-EXO-603).
//! * DVF fallback u = −c_dvf·ẋ_r (collocated, passive), c_dvf = c_d,opt − c_eddy with ζ
//!   normalised to ω_s ([`crate::tuning::dvf_damping_n_s_per_m`]).
//! * Mode ladder HMD_ACTIVE → SEISMIC_PASSIVE → DVF_ONLY → PASSIVE_EDDY → LANDED_FPS
//!   (→ LOCKED for maintenance). Degradation is immediate; recovery needs
//!   [`RECOVER_QUIET_S`] of continuous better demand (SENTINEL: HMD_ACTIVE after 60 s
//!   quiet, exotic design §3.4); LANDED_FPS is latched until maintenance. On supply
//!   loss a zero-power hybrid EMS diverges in an **indeterminate** direction (below g*
//!   the PM force exceeds the weight, above it falls short) onto designed sliding stops:
//!   the upper stop (≤ 1.3 W, LEV-006) or the FPS catcher below. There is no clamp-up
//!   claim ([`supply_loss_behaviour`]; plan §4 correction, FAB-EXO-115, 605). Mass
//!   dampers are mainly wind devices (at most about 0.7 % added damping at design-level
//!   shaking).

use fabrail_types::fmath;

/// Stroke-governor saturation flag.
pub const SAT_STROKE: u8 = 1;
/// Power-limit saturation flag.
pub const SAT_POWER: u8 = 2;
/// Force-limit saturation flag.
pub const SAT_FORCE: u8 = 4;
/// Rate-limit saturation flag.
pub const SAT_RATE: u8 = 8;

/// Governor ramp start as a fraction of s_max (exotic design §3.1).
pub const GOVERNOR_START_FRAC: f32 = 0.7;

/// Rate-limit window: the force may change by at most F_max per this time, s
/// (exotic design §3.1, bumpless mode transfer).
pub const RATE_WINDOW_S: f32 = 0.5;

/// Continuous better demand needed before the ladder steps back up, s (exotic design
/// §3.4, SENTINEL: "HMD_ACTIVE after 60 s quiet").
pub const RECOVER_QUIET_S: f32 = 60.0;

/// HMD operating mode (the mode ladder), ordered by degradation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[repr(u8)]
pub enum HmdMode {
    /// Active hybrid damping (wind).
    #[default]
    HmdActive = 0,
    /// Passive eddy damping with the stroke governor armed (on alert or high floor
    /// acceleration).
    SeismicPassive = 1,
    /// Direct velocity feedback only (sensor fault).
    DvfOnly = 2,
    /// Passive eddy damping, LSM off (power loss).
    PassiveEddy = 3,
    /// On a designed sliding stop after levitation loss: the friction-pendulum catcher
    /// below or the upper sliding stop (the divergence direction is indeterminate).
    LandedFps = 4,
    /// Locked (maintenance).
    Locked = 5,
}

impl HmdMode {
    /// Every mode, in ladder order.
    pub const ALL: [HmdMode; 6] = [
        HmdMode::HmdActive,
        HmdMode::SeismicPassive,
        HmdMode::DvfOnly,
        HmdMode::PassiveEddy,
        HmdMode::LandedFps,
        HmdMode::Locked,
    ];

    /// Snake-case name (scene JSON `hmd_mode`).
    pub const fn slug(self) -> &'static str {
        match self {
            HmdMode::HmdActive => "hmd_active",
            HmdMode::SeismicPassive => "seismic_passive",
            HmdMode::DvfOnly => "dvf_only",
            HmdMode::PassiveEddy => "passive_eddy",
            HmdMode::LandedFps => "landed_fps",
            HmdMode::Locked => "locked",
        }
    }

    /// The trim LSM is energised (it commands force) in this mode.
    pub const fn lsm_enabled(self) -> bool {
        matches!(
            self,
            HmdMode::HmdActive | HmdMode::SeismicPassive | HmdMode::DvfOnly
        )
    }

    /// The damper rests on a stop (landed or locked) rather than levitating.
    pub const fn on_stops(self) -> bool {
        matches!(self, HmdMode::LandedFps | HmdMode::Locked)
    }
}

/// Controller gains (from offline LQR).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdGains {
    /// LQR gain row K on [x_s, ẋ_s, x_r, ẋ_r], N per (m, m/s, m, m/s).
    pub k_lqr: [f32; 4],
    /// DVF damping c_dvf = c_d,opt − c_eddy with c_d,opt = 2ζ_s·m_d·ω_s (ζ normalised to
    /// ω_s; [`crate::tuning::dvf_damping_n_s_per_m`]), N·s/m.
    pub c_dvf_n_s_per_m: f32,
}

/// Saturation limits.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdLimits {
    /// Stroke limit s_max, m.
    pub s_max_m: f32,
    /// Governor ramp start as a fraction of s_max (0.7).
    pub governor_start_frac: f32,
    /// Power limit, W.
    pub p_max_w: f32,
    /// Force limit, N.
    pub f_max_n: f32,
    /// Rate limit, N/s (F_max per 0.5 s).
    pub rate_n_per_s: f32,
}

impl HmdLimits {
    /// Limits for an actuator of force `f_max_n` and power `p_max_w` on a stroke of
    /// ±`s_max_m`, with the governor from 0.7·s_max and the rate F_max per 0.5 s
    /// (exotic design §3.1; SENTINEL: ±1.5 m, 24–121 kN, 33–163 kW).
    pub fn from_actuator(s_max_m: f32, p_max_w: f32, f_max_n: f32) -> HmdLimits {
        HmdLimits {
            s_max_m,
            governor_start_frac: GOVERNOR_START_FRAC,
            p_max_w,
            f_max_n,
            rate_n_per_s: f_max_n / RATE_WINDOW_S,
        }
    }
}

/// Sensor and supervisor inputs of one step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdInputs {
    /// Structure displacement estimate, m.
    pub x_s_m: f32,
    /// Structure velocity estimate, m/s.
    pub v_s_mps: f32,
    /// Stroke x_d − x_s, m.
    pub x_r_m: f32,
    /// Stroke rate, m/s.
    pub v_r_mps: f32,
    /// Floor acceleration, m/s².
    pub a_floor_mps2: f32,
    /// Triplex sensing healthy.
    pub sensors_ok: bool,
    /// Bus power available.
    pub power_ok: bool,
    /// Levitation healthy.
    pub levitation_ok: bool,
    /// Seismic alert active (or floor acceleration above threshold).
    pub seismic: bool,
    /// Maintenance lock requested.
    pub maintenance: bool,
}

/// One controller output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdOutput {
    /// LSM trim force command, N.
    pub force_n: f32,
    /// Mode in force.
    pub mode: HmdMode,
    /// Saturation flags (`SAT_*`).
    pub sat: u8,
}

/// The mode the inputs demand (the most degraded condition wins).
pub const fn next_mode(inp: &HmdInputs) -> HmdMode {
    if inp.maintenance {
        HmdMode::Locked
    } else if !inp.levitation_ok {
        HmdMode::LandedFps
    } else if !inp.power_ok {
        HmdMode::PassiveEddy
    } else if !inp.sensors_ok {
        HmdMode::DvfOnly
    } else if inp.seismic {
        HmdMode::SeismicPassive
    } else {
        HmdMode::HmdActive
    }
}

/// Which stop a diverging suspension ends on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum StopSide {
    /// The upper sliding stop.
    Upper = 0,
    /// The lower stop (FPS catcher or rollers).
    Lower = 1,
    /// Either: a zero-power hybrid EMS diverges up or down depending on the gap
    /// relative to g* at the instant of loss.
    #[default]
    Indeterminate = 2,
}

/// What the levitated damper does when its supply is lost (plan §4 correction;
/// FAB-EXO-115, 605). Mirrors the truth model's `FailureBehaviour::Diverge` in
/// fabrail-exo-sim.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SupplyLossBehaviour {
    /// The suspension diverges open-loop (zero-power hybrid EMS: yes).
    pub diverges: bool,
    /// The stop it ends on.
    pub side: StopSide,
    /// The upper stop is a designed sliding surface (contact force ≤ 1.3 W, LEV-006).
    pub upper_stop_sliding: bool,
    /// The lower stop is a designed sliding surface (FPS catcher or rollers).
    pub lower_stop_sliding: bool,
    /// Whether a "no clamp-up" property is claimed: never for a zero-power hybrid EMS.
    pub clamp_up_excluded: bool,
}

/// Supply-loss behaviour of the SENTINEL suspension (zero-power hybrid EMS): diverges
/// with an indeterminate direction onto designed sliding stops; no clamp-up claim.
pub const fn supply_loss_behaviour() -> SupplyLossBehaviour {
    SupplyLossBehaviour {
        diverges: true,
        side: StopSide::Indeterminate,
        upper_stop_sliding: true,
        lower_stop_sliding: true,
        clamp_up_excluded: false,
    }
}

/// Stroke-governor weight w = ξ³ with ξ = (|x_r| − f·s_max)/((1 − f)·s_max) clamped to
/// [0, 1] (cubic ramp from `start_frac`·s_max; exotic design §3.1). A non-positive or
/// non-finite `s_max_m` or stroke gives w = 1 (governor only).
pub fn governor_weight(x_r_m: f32, s_max_m: f32, start_frac: f32) -> f32 {
    if !(s_max_m.is_finite() && s_max_m > 0.0 && x_r_m.is_finite()) {
        return 1.0;
    }
    let frac = if start_frac.is_finite() && (0.0..1.0).contains(&start_frac) {
        start_frac
    } else {
        GOVERNOR_START_FRAC
    };
    let s0 = frac * s_max_m;
    let xi = fmath::clampf((fmath::absf(x_r_m) - s0) / (s_max_m - s0), 0.0, 1.0);
    xi * xi * xi
}

/// A limit sanitised in `new()`: non-finite or negative becomes 0 (no authority).
fn limit(x: f32) -> f32 {
    if x.is_finite() && x > 0.0 { x } else { 0.0 }
}

/// A measurement sanitised per step: non-finite becomes 0.
fn finite_or_zero(x: f32) -> f32 {
    if x.is_finite() { x } else { 0.0 }
}

/// The hybrid mass damper controller.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Hmd {
    gains: HmdGains,
    limits: HmdLimits,
    mode: HmdMode,
    f_prev_n: f32,
    /// Time the demand has continuously been better than the mode in force, s.
    t_better_s: f32,
    /// Governor stiffness at full weight k_sg = F_max/s_max, N/m.
    k_sg_n_per_m: f32,
    /// Governor damping at full weight c_sg = F_max²/P_max, N·s/m.
    c_sg_n_s_per_m: f32,
}

impl Hmd {
    /// A controller in `HmdActive` with zero output. Limits are sanitised: non-finite or
    /// negative values become 0 (no authority: the output stays passive), and a
    /// governor start outside [0, 1) becomes [`GOVERNOR_START_FRAC`].
    pub fn new(gains: HmdGains, limits: HmdLimits) -> Hmd {
        let frac = limits.governor_start_frac;
        let limits = HmdLimits {
            s_max_m: limit(limits.s_max_m),
            governor_start_frac: if frac.is_finite() && (0.0..1.0).contains(&frac) {
                frac
            } else {
                GOVERNOR_START_FRAC
            },
            p_max_w: limit(limits.p_max_w),
            f_max_n: limit(limits.f_max_n),
            rate_n_per_s: limit(limits.rate_n_per_s),
        };
        let gains = HmdGains {
            k_lqr: gains.k_lqr.map(finite_or_zero),
            c_dvf_n_s_per_m: limit(gains.c_dvf_n_s_per_m),
        };
        let k_sg = if limits.s_max_m > 0.0 {
            limits.f_max_n / limits.s_max_m
        } else {
            0.0
        };
        let c_sg = if limits.p_max_w > 0.0 {
            limits.f_max_n * limits.f_max_n / limits.p_max_w
        } else {
            0.0
        };
        Hmd {
            gains,
            limits,
            mode: HmdMode::HmdActive,
            f_prev_n: 0.0,
            t_better_s: 0.0,
            k_sg_n_per_m: k_sg,
            c_sg_n_s_per_m: c_sg,
        }
    }

    /// Steps the controller by `dt_s`.
    pub fn step(&mut self, inp: &HmdInputs, dt_s: f32) -> HmdOutput {
        let dt = if dt_s.is_finite() && dt_s > 0.0 {
            dt_s
        } else {
            0.0
        };
        self.update_mode(next_mode(inp), dt);
        if !self.mode.lsm_enabled() {
            // LSM off: passive eddy damping only; restart the rate limiter from zero so
            // a later re-entry is bumpless.
            self.f_prev_n = 0.0;
            return HmdOutput {
                force_n: 0.0,
                mode: self.mode,
                sat: 0,
            };
        }
        let x_r = finite_or_zero(inp.x_r_m);
        let v_r = finite_or_zero(inp.v_r_mps);
        let u_ctrl = match self.mode {
            HmdMode::HmdActive => {
                let k = self.gains.k_lqr;
                -(k[0] * finite_or_zero(inp.x_s_m)
                    + k[1] * finite_or_zero(inp.v_s_mps)
                    + k[2] * x_r
                    + k[3] * v_r)
            }
            HmdMode::DvfOnly => -self.gains.c_dvf_n_s_per_m * v_r,
            // SEISMIC_PASSIVE: passive eddy damping; only the governor commands force.
            _ => 0.0,
        };
        let (force_n, sat) = self.stack(u_ctrl, x_r, v_r, dt);
        HmdOutput {
            force_n,
            mode: self.mode,
            sat,
        }
    }

    /// The saturation stack: stroke governor → power → force → rate.
    fn stack(&mut self, u_ctrl: f32, x_r: f32, v_r: f32, dt: f32) -> (f32, u8) {
        let l = self.limits;
        let mut sat = 0u8;
        let mut u = finite_or_zero(u_ctrl);
        // 1. Stroke governor (cubic ramp from governor_start_frac·s_max).
        let w = governor_weight(x_r, l.s_max_m, l.governor_start_frac);
        if w > 0.0 {
            sat |= SAT_STROKE;
            if u * v_r > 0.0 {
                u *= 1.0 - w;
            }
            u -= w * (self.c_sg_n_s_per_m * v_r + self.k_sg_n_per_m * x_r);
        }
        // 2. Power |u·ẋ_r| ≤ P_max.
        let p = fmath::absf(u * v_r);
        if p > l.p_max_w {
            u = fmath::copysignf(l.p_max_w / fmath::absf(v_r), u);
            sat |= SAT_POWER;
        }
        // 3. Force |u| ≤ F_max.
        if fmath::absf(u) > l.f_max_n {
            u = fmath::copysignf(l.f_max_n, u);
            sat |= SAT_FORCE;
        }
        // 4. Rate ≤ F_max per 0.5 s.
        let du_max = l.rate_n_per_s * dt;
        let du = u - self.f_prev_n;
        if fmath::absf(du) > du_max {
            u = self.f_prev_n + fmath::copysignf(du_max, du);
            sat |= SAT_RATE;
        }
        self.f_prev_n = u;
        (u, sat)
    }

    /// Ladder update: degrade immediately; step back up only after
    /// [`RECOVER_QUIET_S`] of continuous better demand; LANDED_FPS is latched until
    /// maintenance; leaving maintenance (an operator action) goes to the demand.
    fn update_mode(&mut self, demanded: HmdMode, dt: f32) {
        if demanded >= self.mode {
            self.mode = demanded;
            self.t_better_s = 0.0;
            return;
        }
        match self.mode {
            HmdMode::LandedFps => {
                self.t_better_s = 0.0;
            }
            HmdMode::Locked => {
                self.mode = demanded;
                self.t_better_s = 0.0;
            }
            _ => {
                self.t_better_s += dt;
                if self.t_better_s >= RECOVER_QUIET_S {
                    self.mode = demanded;
                    self.t_better_s = 0.0;
                }
            }
        }
    }

    /// Mode in force.
    pub const fn mode(&self) -> HmdMode {
        self.mode
    }

    /// The force applied last step, N (fed to [`HmdObserver::step`] for anti-windup).
    pub const fn applied_force_n(&self) -> f32 {
        self.f_prev_n
    }

    /// The sanitised limits.
    pub const fn limits(&self) -> HmdLimits {
        self.limits
    }

    /// The sanitised gains.
    pub const fn gains(&self) -> HmdGains {
        self.gains
    }

    /// Supply-loss behaviour of the suspension ([`supply_loss_behaviour`]).
    pub const fn supply_loss(&self) -> SupplyLossBehaviour {
        supply_loss_behaviour()
    }
}

/// Observer configuration (SI; the passive stage as designed in `tuning`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ObserverCfg {
    /// Damper mass m_d, kg.
    pub m_d_kg: f32,
    /// Passive guideway stiffness k_d, N/m.
    pub k_d_n_per_m: f32,
    /// Passive eddy damping c_eddy, N·s/m.
    pub c_d_n_s_per_m: f32,
    /// High-pass corner of the structure velocity and displacement estimates (drift
    /// removal), Hz; well below the structure frequency (0.01 Hz for T 7 s).
    pub hp_hz: f32,
    /// Stroke observer bandwidth ω_o/2π, Hz (≈ 10× the structure frequency).
    pub bw_hz: f32,
}

/// Why an observer configuration was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum ObserverError {
    /// A non-positive or non-finite field (c_d may be 0).
    InvalidField = 0,
}

/// A state estimate ẑ = [x_s, ẋ_s, x_r, ẋ_r].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdEstimate {
    /// Structure displacement, m.
    pub x_s_m: f32,
    /// Structure velocity, m/s.
    pub v_s_mps: f32,
    /// Stroke, m.
    pub x_r_m: f32,
    /// Stroke rate, m/s.
    pub v_r_mps: f32,
}

impl HmdEstimate {
    /// Copies the estimate into the controller inputs.
    pub fn apply(&self, inp: &mut HmdInputs) {
        inp.x_s_m = self.x_s_m;
        inp.v_s_mps = self.v_s_mps;
        inp.x_r_m = self.x_r_m;
        inp.v_r_mps = self.v_r_mps;
    }
}

/// Complementary state observer (exotic design §3.1): floor acceleration, stroke
/// encoder and the *applied* LSM force (anti-windup).
///
/// * Structure: two cascaded leaky integrators of the floor acceleration,
///   v̂_s' = a_floor − ω_hp·v̂_s and x̂_s' = v̂_s − ω_hp·x̂_s (high-pass at `hp_hz`).
/// * Stroke: a model-based Luenberger observer of the damper,
///   ẍ_r = (−k_d·x_r − c_d·ẋ_r + u)/m_d − a_floor, corrected by the encoder with gains
///   l1 = 2ζ_o·ω_o − c_d/m_d and l2 = ω_o² − k_d/m_d − l1·c_d/m_d, which place the error
///   poles at s² + 2ζ_o·ω_o·s + ω_o² (ζ_o = 1/√2). The applied (saturated) u enters the
///   model, so a saturated command never winds the estimate up.
///
/// Semi-implicit Euler at the controller rate (ω_o·dt ≪ 1).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HmdObserver {
    cfg: ObserverCfg,
    w_hp: f32,
    l1: f32,
    l2: f32,
    est: HmdEstimate,
}

/// Observer damping ratio ζ_o (Butterworth).
const OBSERVER_ZETA: f32 = core::f32::consts::FRAC_1_SQRT_2;

impl HmdObserver {
    /// Validates `cfg` and places the stroke-observer poles.
    pub fn new(cfg: &ObserverCfg) -> Result<HmdObserver, ObserverError> {
        let pos = [cfg.m_d_kg, cfg.k_d_n_per_m, cfg.hp_hz, cfg.bw_hz];
        if pos.iter().any(|x| !(x.is_finite() && *x > 0.0))
            || !(cfg.c_d_n_s_per_m.is_finite() && cfg.c_d_n_s_per_m >= 0.0)
        {
            return Err(ObserverError::InvalidField);
        }
        let two_pi = 2.0 * core::f32::consts::PI;
        let w_o = two_pi * cfg.bw_hz;
        let c_m = cfg.c_d_n_s_per_m / cfg.m_d_kg;
        let l1 = 2.0 * OBSERVER_ZETA * w_o - c_m;
        let l2 = w_o * w_o - cfg.k_d_n_per_m / cfg.m_d_kg - l1 * c_m;
        Ok(HmdObserver {
            cfg: *cfg,
            w_hp: two_pi * cfg.hp_hz,
            l1,
            l2,
            est: HmdEstimate::default(),
        })
    }

    /// Steps by `dt_s` with the floor acceleration, the encoder stroke and the force
    /// applied during the last step; returns the new estimate. Non-finite measurements
    /// are ignored for this step (prediction only).
    pub fn step(
        &mut self,
        a_floor_mps2: f32,
        x_r_meas_m: f32,
        u_applied_n: f32,
        dt_s: f32,
    ) -> HmdEstimate {
        if !(dt_s.is_finite() && dt_s > 0.0) {
            return self.est;
        }
        let a = finite_or_zero(a_floor_mps2);
        let u = finite_or_zero(u_applied_n);
        let e = &mut self.est;
        e.v_s_mps += (a - self.w_hp * e.v_s_mps) * dt_s;
        e.x_s_m += (e.v_s_mps - self.w_hp * e.x_s_m) * dt_s;
        let innov = if x_r_meas_m.is_finite() {
            x_r_meas_m - e.x_r_m
        } else {
            0.0
        };
        let c = &self.cfg;
        let a_r = (-c.k_d_n_per_m * e.x_r_m - c.c_d_n_s_per_m * e.v_r_mps + u) / c.m_d_kg - a;
        e.v_r_mps += (a_r + self.l2 * innov) * dt_s;
        e.x_r_m += (e.v_r_mps + self.l1 * innov) * dt_s;
        *e
    }

    /// The current estimate.
    pub const fn estimate(&self) -> HmdEstimate {
        self.est
    }

    /// Resets the estimate to rest at the origin.
    pub fn reset(&mut self) {
        self.est = HmdEstimate::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tuning::{self, TuningForm};

    // SENTINEL-scale test plant (EXOTIC §1: 1,000 t damper, ±1.5 m, T 7 s; F_max 121 kN,
    // P_max 163 kW): host modal mass 1e8 kg (μ = 0.01), ζ_s = 1 %, passive stage tuned
    // by Warburton (force form, ζ on ω_s), eddy damping half the optimum.
    const M_S: f64 = 1.0e8;
    const T_S: f64 = 7.0;
    const ZETA_S: f64 = 0.01;
    const MU: f64 = 0.01;
    const S_MAX: f32 = 1.5;
    const F_MAX: f32 = 121.0e3;
    const P_MAX: f32 = 163.0e3;
    /// LQR row from scipy.linalg.solve_continuous_are on this plant (state
    /// [x_s, ẋ_s, x_r, ẋ_r], u on the damper), Q = diag(k_s², (k_s/ω_s)², 0, 0), R = 1;
    /// closed-loop poles −0.673 ± 1.177j, −0.197 ± 0.556j rad/s (offline, as the plan
    /// prescribes: the controller only applies K).
    const K_LQR: [f32; 4] = [-1.081_001_5e8, -2.458_416e7, 0.0, 1.415_746_1e6];

    #[derive(Clone, Copy)]
    struct Plant {
        k_s: f64,
        c_s: f64,
        m_d: f64,
        k_d: f64,
        c_eddy: f64,
        c_dvf: f64,
    }

    fn plant() -> Plant {
        let w_s = 2.0 * core::f64::consts::PI / T_S;
        let d = tuning::damper_design_for(M_S, w_s, MU, TuningForm::WarburtonForce);
        let c_eddy = 0.5 * d.c_d_n_s_per_m;
        Plant {
            k_s: M_S * w_s * w_s,
            c_s: 2.0 * ZETA_S * M_S * w_s,
            m_d: d.m_d_kg,
            k_d: d.k_d_n_per_m,
            c_eddy,
            c_dvf: tuning::dvf_damping_n_s_per_m(&d, c_eddy),
        }
    }

    /// z = [x_s, v_s, x_r, v_r]; u on the damper (+) and the structure (−); f on the
    /// structure.
    fn deriv(p: &Plant, z: [f64; 4], u: f64, f: f64) -> [f64; 4] {
        let f_d = -p.k_d * z[2] - p.c_eddy * z[3] + u;
        let a_s = (-p.k_s * z[0] - p.c_s * z[1] - f_d + f) / M_S;
        [z[1], a_s, z[3], f_d / p.m_d - a_s]
    }

    fn rk4(p: &Plant, z: [f64; 4], u: f64, f: f64, dt: f64) -> [f64; 4] {
        let add = |a: [f64; 4], b: [f64; 4], h: f64| core::array::from_fn(|i| a[i] + h * b[i]);
        let k1 = deriv(p, z, u, f);
        let k2 = deriv(p, add(z, k1, 0.5 * dt), u, f);
        let k3 = deriv(p, add(z, k2, 0.5 * dt), u, f);
        let k4 = deriv(p, add(z, k3, dt), u, f);
        core::array::from_fn(|i| z[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
    }

    struct RunOut {
        max_stroke_m: f64,
        first_sat: Option<(u8, f64)>,
        seen: u8,
    }

    /// Closed loop at 100 Hz: resonant force of amplitude `f0` on the structure, ramped
    /// linearly over the first 75 % of `t_end` (no switch-on transient). `ctl` None =
    /// LSM off (passive stage only).
    fn run(
        ctl: Option<(HmdGains, HmdLimits, HmdMode)>,
        f0: f64,
        t_end: f64,
        z0: [f64; 4],
    ) -> RunOut {
        let p = plant();
        let w_s = 2.0 * core::f64::consts::PI / T_S;
        let dt = 0.01;
        let mut hmd = ctl.map(|(g, l, _)| Hmd::new(g, l));
        let mut z = z0;
        let mut out = RunOut {
            max_stroke_m: 0.0,
            first_sat: None,
            seen: 0,
        };
        let n = (t_end / dt) as usize;
        for k in 0..n {
            let t = k as f64 * dt;
            let f = f0 * (t / (0.75 * t_end)).min(1.0) * (w_s * t).sin();
            let u = match (&mut hmd, ctl) {
                (Some(h), Some((_, _, mode))) => {
                    let inp = HmdInputs {
                        x_s_m: z[0] as f32,
                        v_s_mps: z[1] as f32,
                        x_r_m: z[2] as f32,
                        v_r_mps: z[3] as f32,
                        sensors_ok: mode != HmdMode::DvfOnly,
                        power_ok: true,
                        levitation_ok: true,
                        seismic: mode == HmdMode::SeismicPassive,
                        ..Default::default()
                    };
                    let o = h.step(&inp, dt as f32);
                    assert_eq!(o.mode, mode, "harness drives the intended mode");
                    if o.sat != 0 && out.first_sat.is_none() {
                        out.first_sat = Some((o.sat, z[2].abs()));
                    }
                    out.seen |= o.sat;
                    o.force_n as f64
                }
                _ => 0.0,
            };
            z = rk4(&p, z, u, f, dt);
            out.max_stroke_m = out.max_stroke_m.max(z[2].abs());
        }
        out
    }

    fn limits() -> HmdLimits {
        HmdLimits::from_actuator(S_MAX, P_MAX, F_MAX)
    }

    // req: FAB-EXO-604
    #[test]
    fn stroke_governor_engages_first() {
        let l = limits();
        // 1. The cubic ramp: 0 up to 0.7·s_max, ξ³ inside, 1 at and beyond s_max.
        for (x, w) in [
            (0.0, 0.0),
            (1.05, 0.0),
            (1.2, (0.15f32 / 0.45).powi(3)),
            (-1.35, (0.3f32 / 0.45).powi(3)),
            (1.5, 1.0),
            (2.0, 1.0),
        ] {
            let got = governor_weight(x, S_MAX, GOVERNOR_START_FRAC);
            assert!((got - w).abs() < 1e-6, "w({x}) = {got}, want {w}");
        }
        // C¹ at the ramp start (slope 3ξ² → 0): no bump when the governor engages.
        assert!(governor_weight(1.05 + 1e-3, S_MAX, 0.7) < 1e-7);

        // 2. Stack order on crafted states: governor first, then power, force, rate.
        let c_sg = F_MAX * F_MAX / P_MAX;
        let k_sg = F_MAX / S_MAX;
        let gains = HmdGains {
            k_lqr: [0.0, 0.0, 0.0, -50.0e3],
            c_dvf_n_s_per_m: 0.0,
        }; // u_lqr = +50 kN·s/m · v_r: feeds power into the stroke.
        let at = |x: f32, v: f32| HmdInputs {
            x_r_m: x,
            v_r_mps: v,
            sensors_ok: true,
            power_ok: true,
            levitation_ok: true,
            ..Default::default()
        };
        let big_dt = 1.0; // rate·dt = 2·F_max: the rate stage cannot bind.
        // Below 0.7·s_max: the controller command passes untouched.
        let mut h = Hmd::new(gains, l);
        let o = h.step(&at(0.5, 0.4), big_dt);
        assert_eq!((o.force_n, o.sat), (20.0e3, 0), "outside the zone");
        // Governor zone, outward: power feed scaled by 1 − w, governor braking added.
        let (x, v) = (1.35f32, 0.2f32);
        let w = governor_weight(x, S_MAX, 0.7);
        let want = (1.0 - w) * 50.0e3 * v - w * (c_sg * v + k_sg * x);
        let o = Hmd::new(gains, l).step(&at(x, v), big_dt);
        assert!(
            (o.force_n - want).abs() < 1.0 && o.sat == SAT_STROKE,
            "governor only: {o:?} vs {want}"
        );
        assert!(
            o.force_n < 0.0,
            "the governed command brakes outward motion"
        );
        // Power binds on the governed (braking, −170 kN) command, not on the raw LQR
        // push (+70 kN): the power-limited output keeps the governed sign.
        let (x, v) = (1.45f32, 1.4f32);
        let o = Hmd::new(gains, l).step(&at(x, v), big_dt);
        assert_eq!(o.sat, SAT_STROKE | SAT_POWER, "{o:?}");
        assert!((o.force_n + P_MAX / v).abs() < 1.0, "{o:?}");
        // Force binds after power (a large command at low speed).
        let hard = HmdGains {
            k_lqr: [0.0, 0.0, 0.0, -2.0e6],
            c_dvf_n_s_per_m: 0.0,
        };
        let o = Hmd::new(hard, l).step(&at(0.2, -0.1), big_dt);
        assert_eq!((o.force_n, o.sat), (-F_MAX, SAT_FORCE), "{o:?}");
        // Rate binds last: a fresh controller at 100 Hz moves by F_max/0.5 s · dt, in the
        // direction of the governed command (−108 kN; the raw LQR push is +200 kN).
        let o = Hmd::new(hard, l).step(&at(1.49, 0.1), 0.01);
        assert_eq!(o.sat, SAT_STROKE | SAT_RATE, "{o:?}");
        assert!(
            (o.force_n + 0.01 * F_MAX / RATE_WINDOW_S).abs() < 1e-2,
            "{o:?}"
        );

        // 3. At or beyond s_max the command never feeds power into outward motion, for
        //    any gain row and state (fresh controller: every stage preserves the sign).
        let mut worst = f32::NEG_INFINITY;
        for kk in [-3.0e6f32, -1.0e5, 0.0, 1.0e5, 3.0e6] {
            for ks in [-1.0e8f32, 0.0, 1.0e8] {
                let g = HmdGains {
                    k_lqr: [ks, 0.3 * ks, kk, kk],
                    c_dvf_n_s_per_m: 0.0,
                };
                for x in [1.5f32, 1.6, 2.5, -1.5, -2.0] {
                    for v in [0.01f32, 0.3, 1.0, 2.0] {
                        let v = v * x.signum(); // outward
                        for xs in [-0.2f32, 0.0, 0.2] {
                            let mut inp = at(x, v);
                            inp.x_s_m = xs;
                            inp.v_s_mps = -xs;
                            let o = Hmd::new(g, l).step(&inp, 0.01);
                            worst = worst.max(o.force_n * v);
                        }
                    }
                }
            }
        }
        assert!(
            worst <= 0.0,
            "power into outward motion beyond s_max: {worst} W"
        );

        // 4. Closed loop at the design basis (SENTINEL scale): a resonant wind-type force
        //    ramped to the level at which the passive stage alone peaks at 0.85·s_max.
        let f0 = 1.1e6;
        let passive = run(None, f0, 1200.0, [0.0; 4]);
        assert!(
            passive.max_stroke_m <= 0.85 * S_MAX as f64 + 0.01,
            "design basis: passive peak {} m",
            passive.max_stroke_m
        );
        let p = plant();
        let lqr = HmdGains {
            k_lqr: K_LQR,
            c_dvf_n_s_per_m: p.c_dvf as f32,
        };
        for (label, mode, want_max) in [
            ("hmd_active", HmdMode::HmdActive, 1.34),
            ("seismic_passive", HmdMode::SeismicPassive, 1.27),
            ("dvf_only", HmdMode::DvfOnly, 1.16),
        ] {
            let r = run(Some((lqr, l, mode)), f0, 1200.0, [0.0; 4]);
            // Reference peaks from the numpy prototype of this loop (±0.01 m).
            assert!(
                (r.max_stroke_m - want_max).abs() < 0.01,
                "{label}: peak {} m vs prototype {want_max} m",
                r.max_stroke_m
            );
            assert!(
                r.max_stroke_m <= S_MAX as f64,
                "{label}: stroke {} m exceeds s_max {S_MAX} m (FAB-EXO-604)",
                r.max_stroke_m
            );
            if let Some((sat, x)) = r.first_sat {
                assert_eq!(sat, SAT_STROKE, "{label}: first saturation {sat:#x}");
                assert!(x >= 0.7 * S_MAX as f64 - 1e-3, "{label}: engaged at {x} m");
            }
        }
        let active = run(Some((lqr, l, HmdMode::HmdActive)), f0, 1200.0, [0.0; 4]);
        assert_eq!(active.seen & SAT_STROKE, SAT_STROKE, "governor engaged");
        assert!(active.first_sat.is_some());

        // 5. A runaway controller (energy-pumping gain row, u = +1 MN·s/m · v_r): the
        //    governor removes its power feed near the ends and cuts the excursion by at
        //    least 10 %; the remaining excursion beyond s_max is the designed stops' job
        //    (reported, not bounded by the ~10 % trim LSM).
        let pump = HmdGains {
            k_lqr: [0.0, 0.0, 0.0, -1.0e6],
            c_dvf_n_s_per_m: 0.0,
        };
        let no_gov = HmdLimits {
            s_max_m: 1.0e6,
            ..l
        };
        let z0 = [0.0, 0.0, 0.05, 0.0];
        let wild = run(Some((pump, no_gov, HmdMode::HmdActive)), 0.0, 900.0, z0);
        let governed = run(Some((pump, l, HmdMode::HmdActive)), 0.0, 900.0, z0);
        assert!(
            wild.max_stroke_m > S_MAX as f64,
            "the runaway case must be a real test"
        );
        assert!(
            governed.max_stroke_m <= 0.9 * wild.max_stroke_m,
            "runaway: governed {} m vs ungoverned {} m",
            governed.max_stroke_m,
            wild.max_stroke_m
        );
    }

    // req: FAB-EXO-605
    #[test]
    fn mode_ladder_and_fail_behaviour() {
        // 1. The ladder order and the demand priority (exhaustive over the five flags).
        let order = [
            HmdMode::HmdActive,
            HmdMode::SeismicPassive,
            HmdMode::DvfOnly,
            HmdMode::PassiveEddy,
            HmdMode::LandedFps,
        ];
        assert!(order.windows(2).all(|w| w[0] < w[1]));
        assert_eq!(&HmdMode::ALL[..5], &order);
        for bits in 0u8..32 {
            let inp = HmdInputs {
                seismic: bits & 1 != 0,
                sensors_ok: bits & 2 == 0,
                power_ok: bits & 4 == 0,
                levitation_ok: bits & 8 == 0,
                maintenance: bits & 16 != 0,
                ..Default::default()
            };
            let want = if bits & 16 != 0 {
                HmdMode::Locked
            } else if bits & 8 != 0 {
                HmdMode::LandedFps
            } else if bits & 4 != 0 {
                HmdMode::PassiveEddy
            } else if bits & 2 != 0 {
                HmdMode::DvfOnly
            } else if bits & 1 != 0 {
                HmdMode::SeismicPassive
            } else {
                HmdMode::HmdActive
            };
            assert_eq!(next_mode(&inp), want, "flags {bits:05b}");
        }

        // 2. Degradation is immediate, rung by rung, with the per-mode force law.
        let gains = HmdGains {
            k_lqr: [0.0, 0.0, 0.0, 100.0e3],
            c_dvf_n_s_per_m: 40.0e3,
        };
        let l = limits();
        let mut h = Hmd::new(gains, l);
        let mut inp = HmdInputs {
            x_r_m: 0.3,
            v_r_mps: 0.2,
            sensors_ok: true,
            power_ok: true,
            levitation_ok: true,
            ..Default::default()
        };
        let dt = 1.0; // rate stage open (rate·dt = 2·F_max)
        let o = h.step(&inp, dt);
        assert_eq!((o.mode, o.force_n), (HmdMode::HmdActive, -20.0e3));
        inp.seismic = true;
        let o = h.step(&inp, dt);
        assert_eq!((o.mode, o.force_n), (HmdMode::SeismicPassive, 0.0));
        // SEISMIC_PASSIVE keeps the governor armed: inside the zone it brakes.
        let mut zone = inp;
        zone.x_r_m = 1.4;
        let o = h.step(&zone, dt);
        assert!(o.force_n < 0.0 && o.sat & SAT_STROKE != 0, "{o:?}");
        inp.sensors_ok = false;
        let o = h.step(&inp, dt);
        assert_eq!((o.mode, o.force_n), (HmdMode::DvfOnly, -8.0e3));
        inp.power_ok = false;
        let o = h.step(&inp, dt);
        assert_eq!((o.mode, o.force_n, o.sat), (HmdMode::PassiveEddy, 0.0, 0));
        inp.levitation_ok = false;
        let o = h.step(&inp, dt);
        assert_eq!((o.mode, o.force_n, o.sat), (HmdMode::LandedFps, 0.0, 0));
        // LSM off on the stops and in passive eddy, whatever the state.
        for m in HmdMode::ALL {
            assert_eq!(m.lsm_enabled(), m < HmdMode::PassiveEddy, "{}", m.slug());
            assert_eq!(m.on_stops(), m >= HmdMode::LandedFps, "{}", m.slug());
        }
        for x in [-3.0f32, 0.0, 1.49, 5.0] {
            let mut i2 = inp;
            i2.x_r_m = x;
            i2.v_r_mps = x;
            assert_eq!(h.step(&i2, dt).force_n, 0.0);
        }

        // 3. LANDED_FPS is latched: clearing every fault does not re-levitate.
        let healthy = HmdInputs {
            sensors_ok: true,
            power_ok: true,
            levitation_ok: true,
            ..Default::default()
        };
        for _ in 0..200 {
            assert_eq!(h.step(&healthy, 1.0).mode, HmdMode::LandedFps);
        }
        // Maintenance locks; releasing it (an operator action) returns to the demand,
        // and the LSM re-enters bumplessly from zero.
        let mut maint = healthy;
        maint.maintenance = true;
        assert_eq!(h.step(&maint, 0.01).mode, HmdMode::Locked);
        let mut moving = healthy;
        moving.v_r_mps = 1.0;
        let o = h.step(&moving, 0.01);
        assert_eq!(o.mode, HmdMode::HmdActive);
        assert!(
            o.force_n.abs() <= 0.01 * l.rate_n_per_s + 1e-3,
            "bumpless: {o:?}"
        );

        // 4. Recovery needs RECOVER_QUIET_S of continuous better demand; a relapse
        //    restarts the count.
        let mut h = Hmd::new(gains, l);
        let mut alert = healthy;
        alert.seismic = true;
        assert_eq!(h.step(&alert, 0.1).mode, HmdMode::SeismicPassive);
        for _ in 0..599 {
            assert_eq!(h.step(&healthy, 0.1).mode, HmdMode::SeismicPassive);
        }
        assert_eq!(h.step(&alert, 0.1).mode, HmdMode::SeismicPassive); // relapse
        let mut steps = 0;
        while h.step(&healthy, 0.1).mode != HmdMode::HmdActive {
            steps += 1;
            assert!(steps < 1000, "never recovered");
        }
        assert!(
            (598..=601).contains(&steps),
            "recovered after {steps} steps of 0.1 s (60 s quiet)"
        );
        // Power loss then return: PASSIVE_EDDY holds for the quiet time.
        let mut dark = healthy;
        dark.power_ok = false;
        assert_eq!(h.step(&dark, 0.1).mode, HmdMode::PassiveEddy);
        assert_eq!(h.step(&healthy, 30.0).mode, HmdMode::PassiveEddy);
        assert_eq!(h.step(&healthy, 30.0).mode, HmdMode::HmdActive);

        // 5. Supply loss: Diverge{Indeterminate} onto designed sliding stops, no
        //    clamp-up claim (plan §4 correction; FAB-EXO-115).
        let s = h.supply_loss();
        assert_eq!(s, supply_loss_behaviour());
        assert!(s.diverges, "zero-power hybrid EMS is open-loop unstable");
        assert_eq!(s.side, StopSide::Indeterminate);
        assert!(s.upper_stop_sliding && s.lower_stop_sliding);
        assert!(!s.clamp_up_excluded, "no clamp-up claim");

        // 6. Sanitised construction: garbage limits leave the LSM without authority.
        let mut g = Hmd::new(
            gains,
            HmdLimits {
                s_max_m: f32::NAN,
                governor_start_frac: 2.0,
                p_max_w: -1.0,
                f_max_n: f32::INFINITY,
                rate_n_per_s: 1.0e6,
            },
        );
        assert_eq!(g.limits().governor_start_frac, GOVERNOR_START_FRAC);
        let o = g.step(&moving, 0.01);
        assert_eq!(o.force_n, 0.0, "{o:?}");
    }

    #[test]
    fn observer_tracks_stroke_and_structure() {
        // Free vibration of the SENTINEL plant with a known applied force; the
        // observer sees only the floor acceleration, the encoder and the applied u.
        let p = plant();
        let cfg = ObserverCfg {
            m_d_kg: p.m_d as f32,
            k_d_n_per_m: p.k_d as f32,
            c_d_n_s_per_m: p.c_eddy as f32,
            hp_hz: 0.01,
            bw_hz: 1.5,
        };
        assert!(HmdObserver::new(&ObserverCfg { m_d_kg: 0.0, ..cfg }).is_err());
        let mut obs = HmdObserver::new(&cfg).unwrap_or_default();
        let dt = 0.01;
        let w_s = 2.0 * core::f64::consts::PI / T_S;
        let mut z = [0.0f64; 4];
        let (mut e_vr, mut n_vr, mut e_vs, mut n_vs) = (0.0, 0.0, 0.0, 0.0);
        for k in 0..60_000 {
            let t = k as f64 * dt;
            let f = 4.0e5 * (w_s * t).sin();
            let u = 60.0e3 * (0.5 * w_s * t).sin();
            let a_s = deriv(&p, z, u, f)[1];
            let est = obs.step(a_s as f32, z[2] as f32, u as f32, dt as f32);
            z = rk4(&p, z, u, f, dt);
            if t > 300.0 {
                e_vr += (est.v_r_mps as f64 - z[3]).powi(2);
                n_vr += z[3] * z[3];
                e_vs += (est.v_s_mps as f64 - z[1]).powi(2);
                n_vs += z[1] * z[1];
            }
        }
        let (r_vr, r_vs) = ((e_vr / n_vr).sqrt(), (e_vs / n_vs).sqrt());
        assert!(r_vr < 0.02, "stroke-rate RMS error {r_vr} (2 %)");
        // Leaky integration at 0.01 Hz of a 0.14 Hz mode: ~7 % phase lead is expected.
        assert!(r_vs < 0.1, "structure-velocity RMS error {r_vs} (10 %)");
    }
}
