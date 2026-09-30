//! Energy admission: E_discharge and E_absorb with T_ride = max(300 s, shaking) + landing; regeneration balance (FAB-EXO-401..405). Owner: X2 (API frozen by W0a).
//!
//! EXOTIC §X4, §X5. This module defines the [`EnergyStore`] trait **once**: the energy
//! manager here uses it, the f64 truth models in `fabrail_exo_sim::storage`
//! (flywheel, supercap, battery, hybrid) implement it, and so do the controller-side
//! f32 estimators here ([`FlywheelEstimator`], [`SupercapEstimator`],
//! [`BatteryEstimator`]).
//!
//! Numerics: the trait and the admission bookkeeping are f64 (a deliberate exception to
//! the f32 controller rule). Stored energies span 10³ J (a supercap) to 3.75e9 J (a
//! JET-class flywheel), admission runs at planning rate, and the trait is shared with
//! the f64 truth models; target builds keep i64 mJ accumulators in the safety channel
//! (plan §4). The estimators keep f32 state in a compensated (Kahan) accumulator, so a
//! 1 J step on a 3.75 GJ store is not lost to the 24-bit mantissa (≈ 256 J ulp there).
//!
//! Admission (FAB-EXO-402, 120, 122): a move starts only if
//! * the zone is `ClearVerified`;
//! * E_usable ≥ 2·(E_stop + E_land + P_hold·T_ride) with
//!   T_ride = max(300 s, shaking duration) + landing time ([`t_ride_s`]);
//! * absorption headroom holds: charge headroom + dump/eddy-brake rating ≥ 1.2·KE_max
//!   (a failing class, ROTATOR today, is reported, not hidden);
//! * the bumper rating is ≥ ½·m_rated·v_max² (rated mass, never m̂).
//!
//! Regeneration (exotic design §3.3): P_regen = η·|F_brake·v| goes to storage up to the
//! charge rating and headroom, the rest to the dump resistor or eddy brake
//! ([`split_regen`]); the balance closing within 1 % is FAB-EXO-404 (exo-sim storage).

use fabrail_types::fmath;

/// Minimum ride-through window, s (plan §4: T_ride = max(300 s, shaking) + landing).
pub const T_RIDE_MIN_S: f64 = 300.0;

/// Discharge margin on the safe-state energy (plan §4 "2× energy margin").
pub const DISCHARGE_MARGIN: f64 = 2.0;

/// Absorption margin on KE_max (FAB-EXO-122: ≥ 1.2·KE_max).
pub const ABSORB_MARGIN: f64 = 1.2;

/// An energy store as seen by the energy manager (implemented by the truth models in
/// fabrail-exo-sim). Sign convention: positive power discharges into the bus.
pub trait EnergyStore {
    /// State of charge, 0 (at the minimum usable state) … 1 (full).
    fn soc(&self) -> f64;
    /// Energy that can still be delivered above the minimum usable state, J.
    fn usable_j(&self) -> f64;
    /// Energy that can still be absorbed before the store is full, J.
    fn headroom_j(&self) -> f64;
    /// Maximum discharge power, W.
    fn p_max_dis_w(&self) -> f64;
    /// Maximum charge power, W.
    fn p_max_chg_w(&self) -> f64;
    /// Requests `p_req_w` (+ discharge, − charge) for `dt_s`; returns the power actually
    /// delivered (same sign), limited by the power ratings and the stored energy.
    fn step(&mut self, p_req_w: f64, dt_s: f64) -> f64;
}

/// Ride-through window T_ride = max(300 s, t_shaking) + t_land, s (FAB-EXO-120).
pub fn t_ride_s(t_shaking_s: f64, t_land_s: f64) -> f64 {
    fmath::fmax(T_RIDE_MIN_S, t_shaking_s) + fmath::fmax(t_land_s, 0.0)
}

/// Energy to hold for `t_ride_s` at `p_hold_w`, J (EXOTIC §2: 10 kt hybrid EMS at
/// 0.2 g, 170 kW × 300 s = 51 MJ; pure EMS 17 MW × 300 s = 5.1 GJ).
pub fn ride_through_energy_j(p_hold_w: f64, t_ride_s: f64) -> f64 {
    fmath::fmax(p_hold_w, 0.0) * fmath::fmax(t_ride_s, 0.0)
}

/// Kinetic energy ½·m·v², J (rated mass).
pub fn kinetic_energy_j(m_rated_kg: f64, v_mps: f64) -> f64 {
    0.5 * m_rated_kg * v_mps * v_mps
}

/// Ride-through hold-power budget, W (EXOTIC §X5; FAB-EXO-405): what the suspension,
/// the sensing and the propulsion draw while riding through.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HoldBudget {
    /// Levitation (hover) power, W: pure EMS ≈ 1.7 kW/t, hybrid EMS 42/170/1,060 kW per
    /// 10 kt at 0.1/0.2/0.5 g RMS (FAB-EXO-206, exo-sim suspension), passive HTS 0.
    pub levitation_w: f64,
    /// Sensing, controls and communications, W.
    pub sensors_w: f64,
    /// Propulsion (LSM) hold and trim, W.
    pub propulsion_w: f64,
}

impl HoldBudget {
    /// A passive-HTS budget: no levitation power (flux pinning; the LN₂ boil-off is a
    /// consumable budget of its own, FAB-EXO-204), only sensing and propulsion.
    pub const fn hts(sensors_w: f64, propulsion_w: f64) -> HoldBudget {
        HoldBudget {
            levitation_w: 0.0,
            sensors_w,
            propulsion_w,
        }
    }

    /// Total hold power P_hold, W (negative terms count as 0).
    pub fn total_w(&self) -> f64 {
        fmath::fmax(self.levitation_w, 0.0)
            + fmath::fmax(self.sensors_w, 0.0)
            + fmath::fmax(self.propulsion_w, 0.0)
    }
}

/// Inputs of [`admit`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdmissionInputs {
    /// The guarded zone is `ClearVerified`.
    pub zone_clear_verified: bool,
    /// Store usable energy, J ([`EnergyStore::usable_j`]).
    pub e_usable_j: f64,
    /// Store charge headroom, J ([`EnergyStore::headroom_j`]).
    pub e_headroom_j: f64,
    /// Dump resistor / eddy-brake energy rating, J.
    pub dump_rating_j: f64,
    /// Energy to stop from v_max, J.
    pub e_stop_j: f64,
    /// Energy for the energised landing, J.
    pub e_land_j: f64,
    /// Hold (levitation) power during the ride-through, W.
    pub p_hold_w: f64,
    /// Shaking duration of the class scenario, s.
    pub t_shaking_s: f64,
    /// Landing duration, s.
    pub t_land_s: f64,
    /// Rated mass, kg.
    pub m_rated_kg: f64,
    /// Speed limit of the move, m/s.
    pub v_max_mps: f64,
    /// Bumper energy rating, J.
    pub bumper_rating_j: f64,
}

impl AdmissionInputs {
    /// These inputs with the usable energy and headroom of `store`.
    pub fn with_store(self, store: &dyn EnergyStore) -> AdmissionInputs {
        AdmissionInputs {
            e_usable_j: store.usable_j(),
            e_headroom_j: store.headroom_j(),
            ..self
        }
    }
}

/// Why a move was not admitted.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum AdmissionReject {
    /// The zone is not `ClearVerified`.
    ZoneNotClear = 0,
    /// Not enough usable energy for 2·(stop + land + hold).
    Discharge = 1,
    /// Not enough absorption headroom for 1.2·KE_max.
    Absorption = 2,
    /// Bumper rating below ½·m_rated·v_max².
    Bumper = 3,
    /// A non-finite or negative input.
    InvalidInput = 4,
}

/// Result of [`admit`] (reported for every class, admitted or not).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Admission {
    /// The move may start.
    pub admitted: bool,
    /// First failed check (zone, discharge, absorption, bumper).
    pub reject: Option<AdmissionReject>,
    /// Ride-through window used, s.
    pub t_ride_s: f64,
    /// Required usable energy 2·(E_stop + E_land + P_hold·T_ride), J.
    pub e_required_j: f64,
    /// Required absorption 1.2·KE_max, J.
    pub absorb_required_j: f64,
    /// KE_max = ½·m_rated·v_max², J.
    pub ke_max_j: f64,
}

/// Safe-state reachability admission (FAB-EXO-402).
pub fn admit(i: &AdmissionInputs) -> Admission {
    let t_ride = t_ride_s(i.t_shaking_s, i.t_land_s);
    let e_required =
        DISCHARGE_MARGIN * (i.e_stop_j + i.e_land_j + ride_through_energy_j(i.p_hold_w, t_ride));
    let ke_max = kinetic_energy_j(i.m_rated_kg, i.v_max_mps);
    let absorb_required = ABSORB_MARGIN * ke_max;
    let finite = [
        i.e_usable_j,
        i.e_headroom_j,
        i.dump_rating_j,
        i.e_stop_j,
        i.e_land_j,
        i.p_hold_w,
        i.t_shaking_s,
        i.t_land_s,
        i.m_rated_kg,
        i.v_max_mps,
        i.bumper_rating_j,
    ]
    .iter()
    .all(|x| x.is_finite() && *x >= 0.0);
    let reject = if !finite {
        Some(AdmissionReject::InvalidInput)
    } else if !i.zone_clear_verified {
        Some(AdmissionReject::ZoneNotClear)
    } else if i.e_usable_j < e_required {
        Some(AdmissionReject::Discharge)
    } else if i.e_headroom_j + i.dump_rating_j < absorb_required {
        Some(AdmissionReject::Absorption)
    } else if i.bumper_rating_j < ke_max {
        Some(AdmissionReject::Bumper)
    } else {
        None
    };
    Admission {
        admitted: reject.is_none(),
        reject,
        t_ride_s: t_ride,
        e_required_j: e_required,
        absorb_required_j: absorb_required,
        ke_max_j: ke_max,
    }
}

/// Discharge margin E_usable − 2·(E_stop + E_land + P_hold·T_ride), J (negative: the
/// shortfall the report states).
pub fn discharge_margin_j(i: &AdmissionInputs) -> f64 {
    i.e_usable_j - admit(i).e_required_j
}

/// Absorption margin E_headroom + E_dump − 1.2·KE_max, J (negative: the shortfall the
/// report states; ROTATOR today).
pub fn absorption_margin_j(i: &AdmissionInputs) -> f64 {
    i.e_headroom_j + i.dump_rating_j - admit(i).absorb_required_j
}

/// Regenerated braking power P_regen = η·|F_brake·v|, W (exotic design §3.3).
pub fn regen_power_w(eta: f64, f_brake_n: f64, v_mps: f64) -> f64 {
    fmath::clamp(eta, 0.0, 1.0) * fmath::abs(f_brake_n * v_mps)
}

/// Where regenerated power goes.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RegenSplit {
    /// Power the store can take this step, W (≥ 0; pass −this to [`EnergyStore::step`]).
    pub to_store_w: f64,
    /// Power sent to the dump resistor or eddy brake, W (≥ 0).
    pub to_dump_w: f64,
}

/// Splits `p_regen_w` between `store` (up to its charge rating and its headroom over
/// `dt_s`) and the dump (the rest), exotic design §3.3.
pub fn split_regen(p_regen_w: f64, store: &dyn EnergyStore, dt_s: f64) -> RegenSplit {
    let p = if p_regen_w.is_finite() {
        fmath::fmax(p_regen_w, 0.0)
    } else {
        0.0
    };
    let by_energy = if dt_s.is_finite() && dt_s > 0.0 {
        fmath::fmax(store.headroom_j(), 0.0) / dt_s
    } else {
        0.0
    };
    let to_store = fmath::fmin(
        p,
        fmath::fmin(fmath::fmax(store.p_max_chg_w(), 0.0), by_energy),
    );
    RegenSplit {
        to_store_w: to_store,
        to_dump_w: p - to_store,
    }
}

/// Compensated (Kahan) f32 energy accumulator: value = sum − comp.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Acc {
    sum: f32,
    comp: f32,
}

impl Acc {
    fn new(v: f32) -> Acc {
        Acc { sum: v, comp: 0.0 }
    }

    fn add(&mut self, x: f32) {
        let y = x - self.comp;
        let t = self.sum + y;
        self.comp = (t - self.sum) - y;
        self.sum = t;
    }

    /// The accumulated value with the compensation applied, J.
    fn value(&self) -> f64 {
        self.sum as f64 - self.comp as f64
    }

    /// Clamps the value to [lo, hi].
    fn clamp(&mut self, lo: f32, hi: f32) {
        let v = self.value();
        if v < lo as f64 {
            *self = Acc::new(lo);
        } else if v > hi as f64 {
            *self = Acc::new(hi);
        }
    }
}

/// A non-finite or non-positive dt counts as 0.
fn dt_or_zero(dt_s: f64) -> f32 {
    if dt_s.is_finite() && dt_s > 0.0 {
        dt_s as f32
    } else {
        0.0
    }
}

/// Clamps a request to the power ratings and to the energy available over `dt`
/// (`usable`, `headroom` in J at the terminals).
fn limit_request(p_req: f64, p_dis: f32, p_chg: f32, usable: f64, headroom: f64, dt: f32) -> f32 {
    if !p_req.is_finite() || dt <= 0.0 {
        return 0.0;
    }
    let dt = dt as f64;
    let p = fmath::clamp(p_req, -(p_chg as f64), p_dis as f64);
    let p = if p > 0.0 {
        fmath::fmin(p, fmath::fmax(usable, 0.0) / dt)
    } else {
        fmath::fmax(p, -fmath::fmax(headroom, 0.0) / dt)
    };
    p as f32
}

/// Why an estimator configuration was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum StoreError {
    /// A non-positive, non-finite or inconsistent field.
    InvalidField = 0,
}

/// Flywheel estimator configuration (EXOTIC §X5; exotic design §2.5).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlywheelCfg {
    /// Rotor inertia I, kg·m².
    pub inertia_kg_m2: f32,
    /// Top speed ω_max, rad/s.
    pub omega_max_rad_s: f32,
    /// Minimum usable speed ω_min, rad/s (usable fraction 1 − (ω_min/ω_max)²).
    pub omega_min_rad_s: f32,
    /// Discharge power rating, W.
    pub p_max_dis_w: f32,
    /// Charge power rating, W.
    pub p_max_chg_w: f32,
    /// Idle loss P_loss = a + b·ω²: constant term a, W.
    pub loss_a_w: f32,
    /// Idle loss quadratic term b, W·s²/rad².
    pub loss_b_w_s2: f32,
    /// Initial speed, rad/s.
    pub omega0_rad_s: f32,
}

impl FlywheelCfg {
    /// A lossless flywheel storing `e_max_j` at `omega_max_rad_s` (I = 2E/ω²), starting
    /// full.
    pub fn from_stored_energy(
        e_max_j: f32,
        omega_max_rad_s: f32,
        omega_min_rad_s: f32,
        p_max_dis_w: f32,
        p_max_chg_w: f32,
    ) -> FlywheelCfg {
        FlywheelCfg {
            inertia_kg_m2: 2.0 * e_max_j / (omega_max_rad_s * omega_max_rad_s),
            omega_max_rad_s,
            omega_min_rad_s,
            p_max_dis_w,
            p_max_chg_w,
            loss_a_w: 0.0,
            loss_b_w_s2: 0.0,
            omega0_rad_s: omega_max_rad_s,
        }
    }
}

/// Controller-side flywheel estimator: E = ½·I·ω², usable above ω_min, SOC =
/// (ω² − ω_min²)/(ω_max² − ω_min²), idle loss a + b·ω² (EXOTIC §X5). Dead-reckons
/// between speed measurements ([`FlywheelEstimator::observe_speed`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlywheelEstimator {
    cfg: FlywheelCfg,
    e_min_j: f32,
    e_max_j: f32,
    e: Acc,
}

impl FlywheelEstimator {
    /// Validates `cfg` (0 ≤ ω_min < ω_max, positive inertia and ratings).
    pub fn new(cfg: &FlywheelCfg) -> Result<FlywheelEstimator, StoreError> {
        let pos = [cfg.inertia_kg_m2, cfg.omega_max_rad_s];
        let nonneg = [
            cfg.omega_min_rad_s,
            cfg.p_max_dis_w,
            cfg.p_max_chg_w,
            cfg.loss_a_w,
            cfg.loss_b_w_s2,
            cfg.omega0_rad_s,
        ];
        if pos.iter().any(|x| !(x.is_finite() && *x > 0.0))
            || nonneg.iter().any(|x| !(x.is_finite() && *x >= 0.0))
            || cfg.omega_min_rad_s >= cfg.omega_max_rad_s
        {
            return Err(StoreError::InvalidField);
        }
        let half_i = 0.5 * cfg.inertia_kg_m2;
        let w0 = fmath::fminf(cfg.omega0_rad_s, cfg.omega_max_rad_s);
        Ok(FlywheelEstimator {
            cfg: *cfg,
            e_min_j: half_i * cfg.omega_min_rad_s * cfg.omega_min_rad_s,
            e_max_j: half_i * cfg.omega_max_rad_s * cfg.omega_max_rad_s,
            e: Acc::new(half_i * w0 * w0),
        })
    }

    /// Replaces the estimate with a measured rotor speed, rad/s.
    pub fn observe_speed(&mut self, omega_rad_s: f32) {
        if omega_rad_s.is_finite() {
            let w = fmath::clampf(omega_rad_s, 0.0, self.cfg.omega_max_rad_s);
            self.e = Acc::new(0.5 * self.cfg.inertia_kg_m2 * w * w);
        }
    }

    /// Estimated rotor speed √(2E/I), rad/s.
    pub fn omega_rad_s(&self) -> f32 {
        fmath::sqrtf(2.0 * fmath::fmaxf(self.e.value() as f32, 0.0) / self.cfg.inertia_kg_m2)
    }

    /// Stored energy ½·I·ω², J.
    pub fn energy_j(&self) -> f64 {
        self.e.value()
    }

    /// Stored energy at ω_max, J.
    pub fn e_max_j(&self) -> f64 {
        self.e_max_j as f64
    }
}

impl EnergyStore for FlywheelEstimator {
    fn soc(&self) -> f64 {
        let span = (self.e_max_j - self.e_min_j) as f64;
        fmath::clamp((self.e.value() - self.e_min_j as f64) / span, 0.0, 1.0)
    }
    fn usable_j(&self) -> f64 {
        fmath::fmax(self.e.value() - self.e_min_j as f64, 0.0)
    }
    fn headroom_j(&self) -> f64 {
        fmath::fmax(self.e_max_j as f64 - self.e.value(), 0.0)
    }
    fn p_max_dis_w(&self) -> f64 {
        self.cfg.p_max_dis_w as f64
    }
    fn p_max_chg_w(&self) -> f64 {
        self.cfg.p_max_chg_w as f64
    }
    fn step(&mut self, p_req_w: f64, dt_s: f64) -> f64 {
        let dt = dt_or_zero(dt_s);
        let p = limit_request(
            p_req_w,
            self.cfg.p_max_dis_w,
            self.cfg.p_max_chg_w,
            self.usable_j(),
            self.headroom_j(),
            dt,
        );
        let w = self.omega_rad_s();
        let loss = self.cfg.loss_a_w + self.cfg.loss_b_w_s2 * w * w;
        self.e.add(-(p + loss) * dt);
        self.e.clamp(0.0, self.e_max_j);
        p as f64
    }
}

/// Supercapacitor estimator configuration (EXOTIC §X5).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SupercapCfg {
    /// Capacitance C, F.
    pub c_f: f32,
    /// Rated voltage V_max, V.
    pub v_max_v: f32,
    /// Minimum usable voltage V_min, V (usable ½·C·(V² − V_min²)).
    pub v_min_v: f32,
    /// Series resistance, Ω (loss I²·R with I = P/V).
    pub esr_ohm: f32,
    /// Discharge power rating, W.
    pub p_max_dis_w: f32,
    /// Charge power rating, W.
    pub p_max_chg_w: f32,
    /// Initial voltage, V.
    pub v0_v: f32,
}

/// Controller-side supercapacitor estimator: E = ½·C·V², ESR loss (P/V)²·R (EXOTIC
/// §X5). Dead-reckons between voltage measurements.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SupercapEstimator {
    cfg: SupercapCfg,
    e_min_j: f32,
    e_max_j: f32,
    e: Acc,
}

impl SupercapEstimator {
    /// Validates `cfg` (0 < V_min < V_max, positive C and ratings).
    pub fn new(cfg: &SupercapCfg) -> Result<SupercapEstimator, StoreError> {
        let pos = [cfg.c_f, cfg.v_max_v, cfg.v_min_v];
        let nonneg = [cfg.esr_ohm, cfg.p_max_dis_w, cfg.p_max_chg_w, cfg.v0_v];
        if pos.iter().any(|x| !(x.is_finite() && *x > 0.0))
            || nonneg.iter().any(|x| !(x.is_finite() && *x >= 0.0))
            || cfg.v_min_v >= cfg.v_max_v
        {
            return Err(StoreError::InvalidField);
        }
        let v0 = fmath::fminf(cfg.v0_v, cfg.v_max_v);
        Ok(SupercapEstimator {
            cfg: *cfg,
            e_min_j: 0.5 * cfg.c_f * cfg.v_min_v * cfg.v_min_v,
            e_max_j: 0.5 * cfg.c_f * cfg.v_max_v * cfg.v_max_v,
            e: Acc::new(0.5 * cfg.c_f * v0 * v0),
        })
    }

    /// Replaces the estimate with a measured open-circuit voltage, V.
    pub fn observe_voltage(&mut self, v: f32) {
        if v.is_finite() {
            let v = fmath::clampf(v, 0.0, self.cfg.v_max_v);
            self.e = Acc::new(0.5 * self.cfg.c_f * v * v);
        }
    }

    /// Estimated voltage √(2E/C), V.
    pub fn voltage_v(&self) -> f32 {
        fmath::sqrtf(2.0 * fmath::fmaxf(self.e.value() as f32, 0.0) / self.cfg.c_f)
    }
}

impl EnergyStore for SupercapEstimator {
    fn soc(&self) -> f64 {
        let span = (self.e_max_j - self.e_min_j) as f64;
        fmath::clamp((self.e.value() - self.e_min_j as f64) / span, 0.0, 1.0)
    }
    fn usable_j(&self) -> f64 {
        fmath::fmax(self.e.value() - self.e_min_j as f64, 0.0)
    }
    fn headroom_j(&self) -> f64 {
        fmath::fmax(self.e_max_j as f64 - self.e.value(), 0.0)
    }
    fn p_max_dis_w(&self) -> f64 {
        self.cfg.p_max_dis_w as f64
    }
    fn p_max_chg_w(&self) -> f64 {
        self.cfg.p_max_chg_w as f64
    }
    fn step(&mut self, p_req_w: f64, dt_s: f64) -> f64 {
        let dt = dt_or_zero(dt_s);
        let p = limit_request(
            p_req_w,
            self.cfg.p_max_dis_w,
            self.cfg.p_max_chg_w,
            self.usable_j(),
            self.headroom_j(),
            dt,
        );
        let v = fmath::fmaxf(self.voltage_v(), self.cfg.v_min_v);
        let i = p / v;
        self.e.add(-(p + i * i * self.cfg.esr_ohm) * dt);
        self.e.clamp(0.0, self.e_max_j);
        p as f64
    }
}

/// Battery estimator configuration (EXOTIC §X5: UPS class, C-rate limited).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BatteryCfg {
    /// Stored-energy capacity, J.
    pub capacity_j: f32,
    /// Reserve fraction never discharged, 0 … < 1.
    pub soc_min: f32,
    /// Discharge efficiency (terminal/stored), 0 < η ≤ 1.
    pub eta_dis: f32,
    /// Charge efficiency (stored/terminal), 0 < η ≤ 1.
    pub eta_chg: f32,
    /// Discharge power rating (C-rate), W.
    pub p_max_dis_w: f32,
    /// Charge power rating, W.
    pub p_max_chg_w: f32,
    /// Initial state of charge (absolute fraction of capacity).
    pub soc0: f32,
}

/// Controller-side battery estimator: energy counting with charge and discharge
/// efficiencies (EXOTIC §X5). Usable and headroom are terminal energies.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BatteryEstimator {
    cfg: BatteryCfg,
    e: Acc,
}

impl BatteryEstimator {
    /// Validates `cfg`.
    pub fn new(cfg: &BatteryCfg) -> Result<BatteryEstimator, StoreError> {
        let unit = |x: f32| x.is_finite() && x > 0.0 && x <= 1.0;
        let rating = |x: f32| x.is_finite() && x >= 0.0;
        let valid = cfg.capacity_j.is_finite()
            && cfg.capacity_j > 0.0
            && cfg.soc_min.is_finite()
            && (0.0..1.0).contains(&cfg.soc_min)
            && unit(cfg.eta_dis)
            && unit(cfg.eta_chg)
            && rating(cfg.p_max_dis_w)
            && rating(cfg.p_max_chg_w)
            && cfg.soc0.is_finite()
            && (0.0..=1.0).contains(&cfg.soc0);
        if !valid {
            return Err(StoreError::InvalidField);
        }
        Ok(BatteryEstimator {
            cfg: *cfg,
            e: Acc::new(cfg.soc0 * cfg.capacity_j),
        })
    }

    /// Replaces the estimate with a measured absolute state of charge (0 … 1).
    pub fn observe_soc(&mut self, soc: f32) {
        if soc.is_finite() {
            self.e = Acc::new(fmath::clampf(soc, 0.0, 1.0) * self.cfg.capacity_j);
        }
    }

    /// Stored energy, J.
    pub fn stored_j(&self) -> f64 {
        self.e.value()
    }
}

impl EnergyStore for BatteryEstimator {
    fn soc(&self) -> f64 {
        let c = &self.cfg;
        let abs = self.e.value() / c.capacity_j as f64;
        fmath::clamp(
            (abs - c.soc_min as f64) / (1.0 - c.soc_min as f64),
            0.0,
            1.0,
        )
    }
    fn usable_j(&self) -> f64 {
        let c = &self.cfg;
        fmath::fmax(
            (self.e.value() - (c.soc_min * c.capacity_j) as f64) * c.eta_dis as f64,
            0.0,
        )
    }
    fn headroom_j(&self) -> f64 {
        let c = &self.cfg;
        fmath::fmax(
            (c.capacity_j as f64 - self.e.value()) / c.eta_chg as f64,
            0.0,
        )
    }
    fn p_max_dis_w(&self) -> f64 {
        self.cfg.p_max_dis_w as f64
    }
    fn p_max_chg_w(&self) -> f64 {
        self.cfg.p_max_chg_w as f64
    }
    fn step(&mut self, p_req_w: f64, dt_s: f64) -> f64 {
        let dt = dt_or_zero(dt_s);
        let c = self.cfg;
        let p = limit_request(
            p_req_w,
            c.p_max_dis_w,
            c.p_max_chg_w,
            self.usable_j(),
            self.headroom_j(),
            dt,
        );
        let d_stored = if p > 0.0 {
            -p / c.eta_dis
        } else {
            -p * c.eta_chg
        };
        self.e.add(d_stored * dt);
        self.e.clamp(0.0, c.capacity_j);
        p as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_exo_safety::presets;

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    /// rpm → rad/s.
    fn rpm(n: f32) -> f32 {
        n * 2.0 * core::f32::consts::PI / 60.0
    }

    /// A 50 t guarded-zone move that is admitted with margin (SHIELD/BASTION scale).
    fn baseline() -> AdmissionInputs {
        AdmissionInputs {
            zone_clear_verified: true,
            e_usable_j: 2.0e6,
            e_headroom_j: 1.0e6,
            dump_rating_j: 0.0,
            e_stop_j: 20.0e3,
            e_land_j: 50.0e3,
            p_hold_w: 2.0e3,
            t_shaking_s: 60.0,
            t_land_s: 5.0,
            m_rated_kg: 50.0e3,
            v_max_mps: 1.2,
            bumper_rating_j: 50.0e3,
        }
    }

    // req: FAB-EXO-120
    #[test]
    fn t_ride_covers_shaking_duration() {
        // T_ride = max(300 s, shaking) + landing (XCRIT P1-12; the design's 120 s window
        // is superseded).
        for (shake, land, want) in [
            (0.0, 0.0, 300.0),
            (20.0, 30.0, 330.0),
            (300.0, 30.0, 330.0),
            (450.0, 30.0, 480.0), // M9 long duration (150–300 s shaking and beyond)
            (600.0, 60.0, 660.0), // basin resonance > 10 min
            (f64::NAN, 10.0, 310.0),
            (100.0, -5.0, 300.0),
        ] {
            let t = t_ride_s(shake, land);
            assert!(
                (t - want).abs() < 1e-12,
                "T_ride({shake}, {land}) = {t}, want {want}"
            );
        }
        // Monotone in the shaking duration, never below 300 s + landing.
        let mut last = 0.0;
        for k in 0..=1200 {
            let t = t_ride_s(k as f64, 40.0);
            assert!(t >= last && t >= T_RIDE_MIN_S + 40.0);
            last = t;
        }
        // Admission sizes on it: a 600 s basin scenario needs 2·P_hold·300 s more than
        // any scenario of ≤ 300 s.
        let short = admit(&AdmissionInputs {
            t_shaking_s: 100.0,
            p_hold_w: 170.0e3,
            ..baseline()
        });
        let long = admit(&AdmissionInputs {
            t_shaking_s: 600.0,
            p_hold_w: 170.0e3,
            ..baseline()
        });
        assert_eq!(short.t_ride_s, 305.0);
        assert_eq!(long.t_ride_s, 605.0);
        assert!(
            rel(
                long.e_required_j - short.e_required_j,
                2.0 * 170.0e3 * 300.0
            ) < 1e-12,
            "longer shaking must raise the required energy (exact)"
        );
    }

    // req: FAB-EXO-122
    #[test]
    fn absorption_headroom() {
        // Boundary: headroom + dump ≥ 1.2·KE_max, exactly.
        let b = baseline();
        let ke = kinetic_energy_j(b.m_rated_kg, b.v_max_mps);
        assert!(rel(ke, 36.0e3) < 1e-12);
        let at = |headroom: f64, dump: f64| {
            admit(&AdmissionInputs {
                e_headroom_j: headroom,
                dump_rating_j: dump,
                ..b
            })
        };
        assert!(at(1.2 * ke, 0.0).admitted);
        assert!(at(0.2 * ke, ke).admitted, "the dump rating counts");
        let a = at(1.2 * ke * (1.0 - 1e-9), 0.0);
        assert_eq!(a.reject, Some(AdmissionReject::Absorption));
        assert!(rel(a.absorb_required_j, 1.2 * ke) < 1e-12);

        // ROTATOR today (EXOTIC §1: "the absorption check currently fails and is
        // reported"): 6,100 t ring at 1.5 m/s rim (preset) → KE_max 6.86 MJ, 1.2× =
        // 8.24 MJ; its regen buffer is 2 kWh (exotic design §1.3, lower bound), taken
        // empty and lossless (the most favourable case), with no dump resistor rated.
        let rot = presets::ROTATOR;
        let m = rot.design_mass.0 as f64;
        let v = rot.kinematics.v_max_mmps as f64 * 1e-3;
        let buffer = BatteryEstimator::new(&BatteryCfg {
            capacity_j: 2.0 * 3.6e6,
            soc_min: 0.0,
            eta_dis: 1.0,
            eta_chg: 1.0,
            p_max_dis_w: 0.9e6 * 1.5,
            p_max_chg_w: 0.9e6 * 1.5,
            soc0: 0.0,
        })
        .unwrap_or_default();
        let ups = BatteryEstimator::new(&BatteryCfg {
            capacity_j: 50.0e6,
            soc_min: 0.2,
            eta_dis: 0.95,
            eta_chg: 0.95,
            p_max_dis_w: 1.0e6,
            p_max_chg_w: 0.0,
            soc0: 1.0,
        })
        .unwrap_or_default();
        let inp = AdmissionInputs {
            zone_clear_verified: true,
            e_usable_j: ups.usable_j() + buffer.usable_j(),
            e_headroom_j: buffer.headroom_j(),
            dump_rating_j: 0.0,
            e_stop_j: 0.5e6,
            e_land_j: 1.0e6,
            p_hold_w: 0.0, // BrakeAndLand: landed on the pier for the ride-through
            t_shaking_s: 60.0,
            t_land_s: 30.0,
            m_rated_kg: m,
            v_max_mps: v,
            bumper_rating_j: 10.0e6,
        };
        let a = admit(&inp);
        assert!(
            rel(a.ke_max_j, 6.8625e6) < 1e-12,
            "ROTATOR KE {}",
            a.ke_max_j
        );
        assert!(!a.admitted, "ROTATOR absorption must fail today (reported)");
        assert_eq!(a.reject, Some(AdmissionReject::Absorption));
        assert!(
            a.e_required_j <= inp.e_usable_j,
            "discharge passes: absorption is the gap"
        );
        let short = absorption_margin_j(&inp);
        assert!(
            rel(short, 7.2e6 - 1.2 * 6.8625e6) < 1e-9,
            "reported shortfall {short} J (−1.04 MJ)"
        );
        // The report names the fix: a dump / eddy-brake rating ≥ the shortfall admits.
        let fixed = admit(&AdmissionInputs {
            dump_rating_j: -short + 1.0,
            ..inp
        });
        assert!(fixed.admitted, "{fixed:?}");

        // Reported, not hidden, for every class: a rejected admission still carries
        // T_ride, the required energies and KE_max (rated mass × class v_max).
        for p in presets::PRESETS {
            let m = p.mass_max.0 as f64;
            let v = p.kinematics.v_max_mmps as f64 * 1e-3;
            let a = admit(&AdmissionInputs {
                zone_clear_verified: true,
                m_rated_kg: m,
                v_max_mps: v,
                ..AdmissionInputs::default()
            });
            assert!(a.ke_max_j.is_finite() && rel(a.ke_max_j + 1.0, 0.5 * m * v * v + 1.0) < 1e-12);
            assert!(rel(a.absorb_required_j + 1.0, 1.2 * a.ke_max_j + 1.0) < 1e-12);
            assert!(a.t_ride_s >= T_RIDE_MIN_S);
        }
    }

    // req: FAB-EXO-402
    #[test]
    fn admission_discharge_and_absorb() {
        let b = baseline();
        let a = admit(&b);
        assert!(a.admitted && a.reject.is_none(), "{a:?}");
        // E_required = 2·(20 kJ + 50 kJ + 2 kW × (max(300, 60) + 5) s) = 1.36 MJ.
        assert!(rel(a.e_required_j, 1.36e6) < 1e-12, "{}", a.e_required_j);

        // Each gate independently, in the order zone → discharge → absorption → bumper.
        let cases = [
            (
                AdmissionInputs {
                    zone_clear_verified: false,
                    ..b
                },
                AdmissionReject::ZoneNotClear,
            ),
            (
                AdmissionInputs {
                    e_usable_j: 1.36e6 * (1.0 - 1e-9),
                    ..b
                },
                AdmissionReject::Discharge,
            ),
            (
                AdmissionInputs {
                    e_headroom_j: 0.0,
                    ..b
                },
                AdmissionReject::Absorption,
            ),
            (
                AdmissionInputs {
                    bumper_rating_j: 36.0e3 * (1.0 - 1e-9),
                    ..b
                },
                AdmissionReject::Bumper,
            ),
            (
                AdmissionInputs {
                    zone_clear_verified: false,
                    e_usable_j: 0.0,
                    e_headroom_j: 0.0,
                    bumper_rating_j: 0.0,
                    ..b
                },
                AdmissionReject::ZoneNotClear,
            ),
        ];
        for (inp, want) in cases {
            assert_eq!(admit(&inp).reject, Some(want), "{inp:?}");
        }
        // Boundaries are inclusive.
        for inp in [
            AdmissionInputs {
                e_usable_j: 1.36e6,
                ..b
            },
            AdmissionInputs {
                bumper_rating_j: 36.0e3,
                ..b
            },
        ] {
            assert!(admit(&inp).admitted, "{inp:?}");
        }
        // The 2× margin binds: 1.5× the safe-state energy is not enough.
        let one_and_half = AdmissionInputs {
            e_usable_j: 0.75 * 1.36e6,
            ..b
        };
        assert_eq!(
            admit(&one_and_half).reject,
            Some(AdmissionReject::Discharge)
        );
        assert!(rel(discharge_margin_j(&one_and_half), -0.25 * 1.36e6) < 1e-12);
        // Non-finite or negative inputs are refused.
        for inp in [
            AdmissionInputs {
                e_usable_j: f64::NAN,
                ..b
            },
            AdmissionInputs {
                m_rated_kg: -1.0,
                ..b
            },
            AdmissionInputs {
                p_hold_w: f64::INFINITY,
                ..b
            },
        ] {
            assert_eq!(admit(&inp).reject, Some(AdmissionReject::InvalidInput));
        }

        // From a live store: a 5 MJ flywheel (ω_min = ω_max/2; a full rotor has no
        // headroom, so a 50 kJ eddy brake covers absorption) is admitted until it has
        // discharged below 1.36 MJ usable, then refused (never after).
        let b = AdmissionInputs {
            dump_rating_j: 50.0e3,
            ..b
        };
        let mut fw = FlywheelEstimator::new(&FlywheelCfg::from_stored_energy(
            5.0e6 / 0.75,
            rpm(20_000.0),
            rpm(10_000.0),
            200.0e3,
            200.0e3,
        ))
        .unwrap_or_default();
        assert!(rel(fw.usable_j(), 5.0e6) < 1e-5);
        let mut refused_at = None;
        for k in 0..400 {
            let a = admit(&b.with_store(&fw));
            match (a.admitted, refused_at) {
                (true, None) => {}
                (false, None) => {
                    assert_eq!(a.reject, Some(AdmissionReject::Discharge));
                    refused_at = Some(k);
                }
                (true, Some(_)) => panic!("re-admitted while discharging"),
                (false, Some(_)) => {}
            }
            fw.step(200.0e3, 0.1);
        }
        let k = refused_at.unwrap_or(0) as f64;
        // 5 MJ − 1.36 MJ at 200 kW = 18.2 s = 182 steps of 0.1 s.
        assert!((k - 182.0).abs() <= 1.0, "refused at step {k}");
    }

    // req: FAB-EXO-403
    #[test]
    fn ride_through_anchors() {
        // Hybrid EMS, 10 kt at 0.2 g RMS: 170 kW × 300 s = 51 MJ; 102 MJ with 2× (exact).
        let t = t_ride_s(0.0, 0.0);
        let e_hybrid = ride_through_energy_j(170.0e3, t);
        assert!(
            rel(e_hybrid, 51.0e6) < 1e-12,
            "{e_hybrid} (EXOTIC §2, exact)"
        );
        let hybrid = AdmissionInputs {
            zone_clear_verified: true,
            p_hold_w: 170.0e3,
            ..AdmissionInputs::default()
        };
        assert!(rel(admit(&hybrid).e_required_j, 102.0e6) < 1e-12);
        // Pure EMS at 1.7 kW/t: 17 MW × 300 s = 5.1 GJ (exact) ...
        let p_pure = 1.7e3 * 10_000.0;
        let e_pure = ride_through_energy_j(p_pure, t);
        assert!(rel(e_pure, 5.1e9) < 1e-12, "{e_pure} (EXOTIC §2, exact)");
        // ... flagged insufficient against one JET-class unit (3.75 GJ stored, 400 MW,
        // usable 75 % at ω_min = ω_max/2), even before the 2× margin.
        let jet = FlywheelEstimator::new(&FlywheelCfg::from_stored_energy(
            3.75e9,
            rpm(225.0),
            rpm(112.5),
            400.0e6,
            400.0e6,
        ))
        .unwrap_or_default();
        assert!(
            rel(jet.usable_j(), 0.75 * 3.75e9) < 1e-6,
            "{}",
            jet.usable_j()
        );
        assert!(jet.usable_j() < e_pure);
        let a = admit(
            &AdmissionInputs {
                p_hold_w: p_pure,
                ..hybrid
            }
            .with_store(&jet),
        );
        assert_eq!(
            a.reject,
            Some(AdmissionReject::Discharge),
            "pure EMS flagged"
        );

        // EMALS-class disk: 121 MJ delivered per cycle from 6,400 to 5,205 rpm, so
        // ≈ 357 MJ stored: E0 = 121 MJ/(1 − (5205/6400)²) = 357.4 MJ.
        let e0 = 121.0e6 / (1.0 - (5205.0f64 / 6400.0).powi(2));
        assert!(rel(e0, 357.0e6) < 2e-3, "EMALS stored {e0} (≈ 357 MJ)");
        let mut disk = FlywheelEstimator::new(&FlywheelCfg::from_stored_energy(
            e0 as f32,
            rpm(6400.0),
            rpm(5205.0),
            60.0e6,
            60.0e6,
        ))
        .unwrap_or_default();
        assert!(rel(disk.usable_j(), 121.0e6) < 1e-5, "{}", disk.usable_j());
        let mut delivered = 0.0;
        for _ in 0..10_000 {
            delivered += disk.step(60.0e6, 1.0e-3) * 1.0e-3;
        }
        assert!(
            rel(delivered, 121.0e6) < 1e-4,
            "delivered {delivered} J per cycle"
        );
        assert!((disk.omega_rad_s() / rpm(5205.0) - 1.0).abs() < 1e-4);
        // The hybrid 102 MJ fits on one EMALS-class disk used down to ω_max/2 (268 MJ).
        let disk_half = FlywheelEstimator::new(&FlywheelCfg::from_stored_energy(
            e0 as f32,
            rpm(6400.0),
            rpm(3200.0),
            60.0e6,
            60.0e6,
        ))
        .unwrap_or_default();
        assert!(rel(disk_half.usable_j(), 0.75 * e0) < 1e-5);
        let ok = admit(
            &AdmissionInputs {
                m_rated_kg: 0.0,
                ..hybrid
            }
            .with_store(&disk_half),
        );
        assert!(ok.admitted, "{ok:?}");

        // Compensated f32 bookkeeping: 1 J steps on the 3.75 GJ store are not lost.
        let mut jet = jet;
        let u0 = jet.usable_j();
        for _ in 0..10_000 {
            jet.step(1.0e3, 1.0e-3);
        }
        assert!(
            (u0 - jet.usable_j() - 10.0e3).abs() < 1.0,
            "10 kJ drawn in 1 J steps: {} J",
            u0 - jet.usable_j()
        );
    }

    // req: FAB-EXO-405
    #[test]
    fn hts_ride_through_kw() {
        // Passive HTS needs no levitation power; the ride-through draws only sensing and
        // propulsion (exotic design §2.5: "HTS: sensors and propulsion only, i.e. kW").
        // Budget: triplex sensing and controls 1 kW, LSM hold and trim 2 kW.
        let hts = HoldBudget::hts(1.0e3, 2.0e3);
        assert_eq!(hts.levitation_w, 0.0);
        let p_hts = hts.total_w();
        assert!(
            (1.0e3..10.0e3).contains(&p_hts),
            "HTS hold {p_hts} W must be kW-scale (FAB-EXO-405)"
        );
        let t = t_ride_s(60.0, 60.0);
        for tonnes in [50.0, 1_000.0, 5_000.0] {
            // Pure EMS 1.7 kW/t and hybrid 170 kW per 10 kt at 0.2 g (FAB-EXO-206).
            let pure = HoldBudget {
                levitation_w: 1.7e3 * tonnes,
                ..hts
            };
            let hybrid = HoldBudget {
                levitation_w: 170.0e3 * tonnes / 10_000.0,
                ..hts
            };
            assert!(hts.total_w() < hybrid.total_w() && hybrid.total_w() < pure.total_w());
            // HTS hold is independent of the suspended mass.
            let e_hts = ride_through_energy_j(p_hts, t);
            assert!(e_hts <= 0.05 * ride_through_energy_j(pure.total_w(), t));
            // A 5 MJ supercap bank carries the HTS ride-through with the 2× margin; the
            // pure-EMS one needs orders of magnitude more.
            let bank = SupercapEstimator::new(&SupercapCfg {
                c_f: 2.0 * 5.0e6 / (0.75 * 800.0 * 800.0),
                v_max_v: 800.0,
                v_min_v: 400.0,
                esr_ohm: 0.01,
                p_max_dis_w: 1.0e6,
                p_max_chg_w: 1.0e6,
                v0_v: 800.0,
            })
            .unwrap_or_default();
            assert!(rel(bank.usable_j(), 5.0e6) < 1e-5);
            let base = AdmissionInputs {
                zone_clear_verified: true,
                t_shaking_s: 60.0,
                t_land_s: 60.0,
                ..AdmissionInputs::default()
            };
            let a = admit(
                &AdmissionInputs {
                    p_hold_w: p_hts,
                    ..base
                }
                .with_store(&bank),
            );
            assert!(a.admitted, "{tonnes} t HTS: {a:?}");
            assert!(a.e_required_j < 3.0e6, "{}", a.e_required_j);
            let p = admit(
                &AdmissionInputs {
                    p_hold_w: pure.total_w(),
                    ..base
                }
                .with_store(&bank),
            );
            assert_eq!(
                p.reject,
                Some(AdmissionReject::Discharge),
                "{tonnes} t pure EMS"
            );
        }
    }

    #[test]
    fn estimators_and_regen_split() {
        // Flywheel: usable fraction 1 − (ω_min/ω_max)² = 75 % at ω_min = ω_max/2, SOC
        // (ω² − ω_min²)/(ω_max² − ω_min²), limits by rating and energy.
        let mut fw = FlywheelEstimator::new(&FlywheelCfg::from_stored_energy(
            4.0e6, 1000.0, 500.0, 100.0e3, 50.0e3,
        ))
        .unwrap_or_default();
        assert!(rel(fw.usable_j(), 3.0e6) < 1e-6 && fw.headroom_j() < 1.0);
        assert!((fw.soc() - 1.0).abs() < 1e-9);
        assert_eq!(fw.step(1.0e6, 1.0), 100.0e3, "discharge rating");
        fw.observe_speed(750.0);
        let want_soc =
            (750.0f64.powi(2) - 500.0f64.powi(2)) / (1000.0f64.powi(2) - 500.0f64.powi(2));
        assert!((fw.soc() - want_soc).abs() < 1e-6);
        assert_eq!(fw.step(-1.0e6, 1.0), -50.0e3, "charge rating");
        fw.observe_speed(501.0);
        let u = fw.usable_j();
        let p = fw.step(100.0e3, 1.0);
        assert!(
            (p - u).abs() < 1.0 && fw.usable_j() < 1.0,
            "energy-limited: {p} vs {u}"
        );
        assert!(
            FlywheelEstimator::new(&FlywheelCfg {
                omega_min_rad_s: 2000.0,
                ..FlywheelCfg::from_stored_energy(1.0, 1000.0, 0.0, 1.0, 1.0)
            })
            .is_err()
        );
        // Idle losses drain a + b·ω².
        let mut lossy = FlywheelEstimator::new(&FlywheelCfg {
            loss_a_w: 100.0,
            loss_b_w_s2: 1e-4,
            ..FlywheelCfg::from_stored_energy(4.0e6, 1000.0, 500.0, 100.0e3, 50.0e3)
        })
        .unwrap_or_default();
        let e0 = lossy.energy_j();
        lossy.step(0.0, 10.0);
        assert!(rel(e0 - lossy.energy_j(), 10.0 * (100.0 + 1e-4 * 1.0e6)) < 1e-3);

        // Supercap: ESR loss (P/V)²·R comes out of the store.
        let mut sc = SupercapEstimator::new(&SupercapCfg {
            c_f: 100.0,
            v_max_v: 400.0,
            v_min_v: 200.0,
            esr_ohm: 0.05,
            p_max_dis_w: 100.0e3,
            p_max_chg_w: 100.0e3,
            v0_v: 400.0,
        })
        .unwrap_or_default();
        assert!(
            rel(
                sc.usable_j(),
                0.5 * 100.0 * (400.0f64.powi(2) - 200.0f64.powi(2))
            ) < 1e-6
        );
        let e_before = sc.usable_j();
        let p = sc.step(40.0e3, 0.1);
        let loss = (40.0e3f64 / 400.0).powi(2) * 0.05 * 0.1;
        assert!(rel(e_before - sc.usable_j(), p * 0.1 + loss) < 1e-4);
        sc.observe_voltage(300.0);
        assert!((sc.voltage_v() - 300.0).abs() < 1e-3);

        // Battery: efficiencies at the terminals.
        let mut bat = BatteryEstimator::new(&BatteryCfg {
            capacity_j: 1.0e6,
            soc_min: 0.2,
            eta_dis: 0.9,
            eta_chg: 0.8,
            p_max_dis_w: 1.0e4,
            p_max_chg_w: 1.0e4,
            soc0: 0.6,
        })
        .unwrap_or_default();
        assert!(rel(bat.usable_j(), 0.4e6 * 0.9) < 1e-6);
        assert!(rel(bat.headroom_j(), 0.4e6 / 0.8) < 1e-6);
        assert!((bat.soc() - 0.5).abs() < 1e-6);
        bat.step(-1.0e4, 10.0);
        assert!(rel(bat.stored_j(), 0.6e6 + 1.0e5 * 0.8) < 1e-6);
        bat.observe_soc(0.25);
        assert!((bat.soc() - 0.0625).abs() < 1e-6);

        // Regeneration: η·|F·v| to storage up to the charge rating and headroom, the
        // rest to the dump.
        let p_regen = regen_power_w(0.9, -200.0e3, 1.0);
        assert!(rel(p_regen, 180.0e3) < 1e-12);
        let fw = FlywheelEstimator::new(&FlywheelCfg {
            omega0_rad_s: 800.0,
            ..FlywheelCfg::from_stored_energy(4.0e6, 1000.0, 500.0, 100.0e3, 50.0e3)
        })
        .unwrap_or_default();
        let s = split_regen(p_regen, &fw, 0.01);
        assert_eq!((s.to_store_w, s.to_dump_w), (50.0e3, 130.0e3));
        let full = FlywheelEstimator::new(&FlywheelCfg::from_stored_energy(
            4.0e6, 1000.0, 500.0, 100.0e3, 50.0e3,
        ))
        .unwrap_or_default();
        let s = split_regen(p_regen, &full, 0.01);
        assert!(s.to_store_w < 100.0 && (s.to_store_w + s.to_dump_w - p_regen).abs() < 1e-6);
    }
}
