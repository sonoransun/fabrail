//! EnergyStore truth models: flywheel, supercap, battery, hybrid store (FAB-EXO-401). Owner: X4 (the trait is defined once in fabrail_exo::energy, frozen by W0a).
//!
//! EXOTIC §X5 (f64 truth; PHYSICS §14 for the supercap hold-up). Each model implements
//! [`fabrail_exo::energy::EnergyStore`] and [`LossyStore`], which also reports the
//! internal loss of a step so the ledger can check the energy balance against the loss
//! model. SMES (E = ½·L·I², P ≤ V_conv·I, a quench dumps the energy) is documented only.
//!
//! | Model | Energy | Usable | Loss |
//! |---|---|---|---|
//! | [`Flywheel`] | ½·I·ω² | ½·I·(ω² − ω_min²); fraction 1 − (ω_min/ω_max)² = 75 % at ω_max/2 | a + b·ω² |
//! | [`Supercap`] | ½·C·V² | ½·C·(V² − V_min²) | I²·ESR, with p = V·I − I²·ESR at the terminals |
//! | [`Battery`] | SOC·capacity | (SOC − SOC_min)·capacity | one-way efficiency √η |
//! | [`HybridStore`] | supercap + flywheel | sum | sum |
//!
//! The hybrid store gives the supercap the high-pass share without extra state: the
//! flywheel supplies p_fly = (E_cap,target − E_cap)/τ (first-order restoring of the
//! supercap to its target energy), so for a store starting at its target the flywheel
//! carries the low-pass of the demand with time constant τ (1–5 s) and the supercap the
//! remainder.
//!
//! EMALS-class disk: 121 MJ delivered per cycle (6400 → 5205 rpm), ≈ 357 MJ stored
//! (EXOTIC §2). The regeneration/dump balance closes within 1 % ([`EnergyLedger`] in f64
//! on the host, [`EnergyLedgerMj`] in i64 mJ as on a target; FAB-EXO-404).
//!
//! Integration is explicit on energy (exact bookkeeping): each step takes the loss at
//! the start-of-step state and the delivered power, then updates the energy and
//! recomputes ω, V or SOC from it.

use fabrail_exo::energy::EnergyStore;
use fabrail_types::fmath;

/// Usable fraction of a flywheel between ω_min and ω_max: 1 − (ω_min/ω_max)²
/// (FAB-EXO-401: 0.75 at ω_min = ω_max/2).
pub fn flywheel_usable_fraction(omega_min_over_max: f64) -> f64 {
    let r = omega_min_over_max.clamp(0.0, 1.0);
    1.0 - r * r
}

/// Revolutions per minute to rad/s.
pub fn rpm_to_rad_s(rpm: f64) -> f64 {
    rpm * 2.0 * core::f64::consts::PI / 60.0
}

/// EMALS-class disk stored energy at 6400 rpm, J (≈ 357 MJ; EXOTIC §2).
pub const EMALS_STORED_J: f64 = 357.0e6;

/// EMALS-class disk top speed, rpm (EXOTIC §2).
pub const EMALS_RPM_MAX: f64 = 6400.0;

/// EMALS-class disk speed after one 121 MJ launch, rpm (EXOTIC §2).
pub const EMALS_RPM_AFTER_LAUNCH: f64 = 5205.0;

/// JET-class flywheel generator stored energy, J (3.75 GJ; exotic design §2.5).
pub const JET_STORED_J: f64 = 3.75e9;

/// JET-class flywheel generator power, W (400 MW; exotic design §2.5).
pub const JET_POWER_W: f64 = 400.0e6;

/// A store that also reports the internal loss of each step (for the ledger).
pub trait LossyStore: EnergyStore {
    /// Stored energy above the minimum usable state, J (the ledger's state variable).
    fn stored_usable_j(&self) -> f64 {
        self.usable_j()
    }
    /// Like [`EnergyStore::step`]; also returns the internal loss over the step, J.
    fn step_with_loss(&mut self, p_req_w: f64, dt_s: f64) -> (f64, f64);
}

/// Energy bookkeeping of a store over a run (f64 on host; i64 mJ on target).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EnergyLedger {
    /// Energy delivered to the bus, J.
    pub e_out_j: f64,
    /// Energy absorbed from the bus (regeneration), J.
    pub e_in_j: f64,
    /// Energy sent to the dump resistor / eddy brake, J.
    pub e_dump_j: f64,
    /// Internal losses, J.
    pub e_loss_j: f64,
    /// Stored energy at the start, J.
    pub e_start_j: f64,
    /// Stored energy at the end, J.
    pub e_end_j: f64,
}

impl EnergyLedger {
    /// A ledger opened on `store`.
    pub fn open<S: LossyStore>(store: &S) -> EnergyLedger {
        let e = store.stored_usable_j();
        EnergyLedger {
            e_start_j: e,
            e_end_j: e,
            ..EnergyLedger::default()
        }
    }

    /// Relative balance error |E_start + E_in − E_out − E_loss − E_end| / max(E_start, 1 J).
    pub fn balance_error(&self) -> f64 {
        let r = self.e_start_j + self.e_in_j - self.e_out_j - self.e_loss_j - self.e_end_j;
        r.abs() / self.e_start_j.max(1.0)
    }

    /// Relative bus-side balance: regeneration offered = absorbed + dumped
    /// (|E_offered − E_in − E_dump| / max(E_offered, 1 J)).
    pub fn bus_balance_error(&self, e_offered_j: f64) -> f64 {
        (e_offered_j - self.e_in_j - self.e_dump_j).abs() / e_offered_j.max(1.0)
    }
}

/// The same ledger in integer millijoules, as a target keeps it (FAB-EXO-404; f32 drifts
/// ≈ 1.8 % over 3e5 steps, so targets use i64 mJ).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EnergyLedgerMj {
    /// Energy delivered, mJ.
    pub e_out_mj: i64,
    /// Energy absorbed, mJ.
    pub e_in_mj: i64,
    /// Energy dumped, mJ.
    pub e_dump_mj: i64,
    /// Internal losses, mJ.
    pub e_loss_mj: i64,
    /// Stored energy at the start, mJ.
    pub e_start_mj: i64,
    /// Stored energy at the end, mJ.
    pub e_end_mj: i64,
}

/// Joules to rounded millijoules (saturating).
pub fn j_to_mj(e_j: f64) -> i64 {
    let v = fmath::round(e_j * 1.0e3);
    if v >= i64::MAX as f64 {
        i64::MAX
    } else if v <= i64::MIN as f64 {
        i64::MIN
    } else {
        v as i64
    }
}

impl EnergyLedgerMj {
    /// Relative balance error, as [`EnergyLedger::balance_error`].
    pub fn balance_error(&self) -> f64 {
        let r = i128::from(self.e_start_mj) + i128::from(self.e_in_mj)
            - i128::from(self.e_out_mj)
            - i128::from(self.e_loss_mj)
            - i128::from(self.e_end_mj);
        let r = if r < 0 { -r } else { r };
        (r as f64) / (self.e_start_mj.max(1000) as f64)
    }
}

/// Exchanges bus power with `store` for `dt_s` and books it (FAB-EXO-404): `p_bus_w` > 0
/// is a discharge demand, < 0 is regeneration offered by the braking drive; whatever the
/// store cannot absorb goes to the dump resistor / eddy brake. Both ledgers are updated
/// (the i64 one per step, rounded, as a target would). Returns the power the store
/// delivered (+) or absorbed (−), W.
pub fn exchange<S: LossyStore>(
    store: &mut S,
    p_bus_w: f64,
    dt_s: f64,
    ledger: &mut EnergyLedger,
    ledger_mj: &mut EnergyLedgerMj,
) -> f64 {
    if dt_s <= 0.0 {
        return 0.0;
    }
    let (p, loss_j) = store.step_with_loss(p_bus_w, dt_s);
    let out_j = p.max(0.0) * dt_s;
    let in_j = (-p).max(0.0) * dt_s;
    let dump_j = if p_bus_w < 0.0 {
        ((-p_bus_w) * dt_s - in_j).max(0.0)
    } else {
        0.0
    };
    ledger.e_out_j += out_j;
    ledger.e_in_j += in_j;
    ledger.e_dump_j += dump_j;
    ledger.e_loss_j += loss_j;
    ledger.e_end_j = store.stored_usable_j();
    ledger_mj.e_out_mj = ledger_mj.e_out_mj.saturating_add(j_to_mj(out_j));
    ledger_mj.e_in_mj = ledger_mj.e_in_mj.saturating_add(j_to_mj(in_j));
    ledger_mj.e_dump_mj = ledger_mj.e_dump_mj.saturating_add(j_to_mj(dump_j));
    ledger_mj.e_loss_mj = ledger_mj.e_loss_mj.saturating_add(j_to_mj(loss_j));
    ledger_mj.e_end_mj = j_to_mj(ledger.e_end_j);
    p
}

/// Limits a request to the power ratings: `p_max_dis` for discharge, `p_max_chg` for
/// charge (both taken as magnitudes).
fn rate_limit(p_req_w: f64, p_max_dis_w: f64, p_max_chg_w: f64) -> f64 {
    if !p_req_w.is_finite() {
        return 0.0;
    }
    p_req_w.clamp(-p_max_chg_w.abs(), p_max_dis_w.abs())
}

// ---- Flywheel ----------------------------------------------------------------------------

/// Flywheel: E = ½·I·ω², losses a + b·ω².
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Flywheel {
    /// Rotor inertia, kg·m².
    pub i_kgm2: f64,
    /// Speed, rad/s.
    pub omega_rad_s: f64,
    /// Minimum usable speed, rad/s.
    pub omega_min_rad_s: f64,
    /// Maximum speed, rad/s.
    pub omega_max_rad_s: f64,
    /// Constant loss a, W.
    pub loss_a_w: f64,
    /// Speed-squared loss b, W·s²/rad².
    pub loss_b: f64,
    /// Power rating (charge and discharge), W.
    pub p_max_w: f64,
}

impl Flywheel {
    /// A full flywheel storing `e_max_j` at `omega_max_rad_s`, usable down to
    /// `omega_min_rad_s`, lossless, rated `p_max_w`.
    pub fn from_energy(
        e_max_j: f64,
        omega_max_rad_s: f64,
        omega_min_rad_s: f64,
        p_max_w: f64,
    ) -> Flywheel {
        let i = if omega_max_rad_s > 0.0 {
            2.0 * e_max_j / (omega_max_rad_s * omega_max_rad_s)
        } else {
            0.0
        };
        Flywheel {
            i_kgm2: i,
            omega_rad_s: omega_max_rad_s,
            omega_min_rad_s,
            omega_max_rad_s,
            loss_a_w: 0.0,
            loss_b: 0.0,
            p_max_w,
        }
    }

    /// The EMALS-class disk: ≈ 357 MJ at 6400 rpm, usable to half speed (EXOTIC §2);
    /// rated to deliver one 121 MJ launch in ≈ 2.5 s (48 MW) \[UNCERTAIN rating\].
    pub fn emals_class() -> Flywheel {
        let w = rpm_to_rad_s(EMALS_RPM_MAX);
        Flywheel::from_energy(EMALS_STORED_J, w, 0.5 * w, 121.0e6 / 2.5)
    }

    /// A JET-class flywheel generator: 3.75 GJ, 400 MW, usable to half speed (exotic
    /// design §2.5). The 225 rpm top speed only sets the inertia scale; every energy
    /// figure depends on E and ω_min/ω_max alone \[UNCERTAIN speed\].
    pub fn jet_class() -> Flywheel {
        let w = rpm_to_rad_s(225.0);
        Flywheel::from_energy(JET_STORED_J, w, 0.5 * w, JET_POWER_W)
    }

    /// Stored kinetic energy ½·I·ω², J.
    pub fn energy_j(&self) -> f64 {
        0.5 * self.i_kgm2 * self.omega_rad_s * self.omega_rad_s
    }

    /// Internal loss at the present speed, W.
    pub fn loss_w(&self) -> f64 {
        self.loss_a_w + self.loss_b * self.omega_rad_s * self.omega_rad_s
    }
}

impl EnergyStore for Flywheel {
    fn soc(&self) -> f64 {
        let lo = self.omega_min_rad_s * self.omega_min_rad_s;
        let hi = self.omega_max_rad_s * self.omega_max_rad_s;
        if hi <= lo {
            return 0.0;
        }
        ((self.omega_rad_s * self.omega_rad_s - lo) / (hi - lo)).clamp(0.0, 1.0)
    }

    fn usable_j(&self) -> f64 {
        (0.5 * self.i_kgm2
            * ((self.omega_rad_s * self.omega_rad_s)
                - (self.omega_min_rad_s * self.omega_min_rad_s)))
            .max(0.0)
    }

    fn headroom_j(&self) -> f64 {
        (0.5 * self.i_kgm2
            * ((self.omega_max_rad_s * self.omega_max_rad_s)
                - (self.omega_rad_s * self.omega_rad_s)))
            .max(0.0)
    }

    fn p_max_dis_w(&self) -> f64 {
        self.p_max_w
    }

    fn p_max_chg_w(&self) -> f64 {
        self.p_max_w
    }

    fn step(&mut self, p_req_w: f64, dt_s: f64) -> f64 {
        self.step_with_loss(p_req_w, dt_s).0
    }
}

impl LossyStore for Flywheel {
    fn stored_usable_j(&self) -> f64 {
        // Signed: spin-down losses may take the rotor below ω_min.
        0.5 * self.i_kgm2
            * (self.omega_rad_s * self.omega_rad_s - self.omega_min_rad_s * self.omega_min_rad_s)
    }

    fn step_with_loss(&mut self, p_req_w: f64, dt_s: f64) -> (f64, f64) {
        if !(dt_s > 0.0 && self.i_kgm2 > 0.0) {
            return (0.0, 0.0);
        }
        let e = self.energy_j();
        let loss_j = (self.loss_w() * dt_s).min(e);
        let mut p = rate_limit(p_req_w, self.p_max_w, self.p_max_w);
        if p > 0.0 {
            // Discharge only from above ω_min.
            p = p.min((self.usable_j() - loss_j).max(0.0) / dt_s);
        } else if p < 0.0 {
            // Charge only up to ω_max.
            p = p.max(-(self.headroom_j() + loss_j) / dt_s);
        }
        let e_new = (e - p * dt_s - loss_j).max(0.0);
        self.omega_rad_s = fmath::sqrt(2.0 * e_new / self.i_kgm2);
        (p, loss_j)
    }
}

// ---- Supercap ----------------------------------------------------------------------------

/// Supercapacitor bank: E = ½·C·V², ESR loss I²R.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Supercap {
    /// Capacitance, F.
    pub c_f: f64,
    /// Voltage, V.
    pub v_v: f64,
    /// Minimum usable voltage, V.
    pub v_min_v: f64,
    /// Maximum voltage, V.
    pub v_max_v: f64,
    /// Series resistance, Ω.
    pub esr_ohm: f64,
    /// Power rating, W.
    pub p_max_w: f64,
}

impl Supercap {
    /// Stored energy ½·C·V², J.
    pub fn energy_j(&self) -> f64 {
        0.5 * self.c_f * self.v_v * self.v_v
    }

    /// Cell current for terminal power `p_w` (+ discharge): p = V·I − I²·R, the root with
    /// |I| smallest; `None` beyond the matched-load limit V²/(4R).
    pub fn current_for_power_a(&self, p_w: f64) -> Option<f64> {
        let v = self.v_v;
        let r = self.esr_ohm;
        if r <= 0.0 {
            return if v > 0.0 { Some(p_w / v) } else { None };
        }
        let disc = v * v - 4.0 * r * p_w;
        if disc < 0.0 {
            return None;
        }
        Some((v - fmath::sqrt(disc)) / (2.0 * r))
    }
}

impl EnergyStore for Supercap {
    fn soc(&self) -> f64 {
        let lo = self.v_min_v * self.v_min_v;
        let hi = self.v_max_v * self.v_max_v;
        if hi <= lo {
            return 0.0;
        }
        ((self.v_v * self.v_v - lo) / (hi - lo)).clamp(0.0, 1.0)
    }

    fn usable_j(&self) -> f64 {
        (0.5 * self.c_f * ((self.v_v * self.v_v) - (self.v_min_v * self.v_min_v))).max(0.0)
    }

    fn headroom_j(&self) -> f64 {
        (0.5 * self.c_f * ((self.v_max_v * self.v_max_v) - (self.v_v * self.v_v))).max(0.0)
    }

    fn p_max_dis_w(&self) -> f64 {
        self.p_max_w
    }

    fn p_max_chg_w(&self) -> f64 {
        self.p_max_w
    }

    fn step(&mut self, p_req_w: f64, dt_s: f64) -> f64 {
        self.step_with_loss(p_req_w, dt_s).0
    }
}

impl LossyStore for Supercap {
    fn stored_usable_j(&self) -> f64 {
        0.5 * self.c_f * (self.v_v * self.v_v - self.v_min_v * self.v_min_v)
    }

    fn step_with_loss(&mut self, p_req_w: f64, dt_s: f64) -> (f64, f64) {
        if !(dt_s > 0.0 && self.c_f > 0.0 && self.v_v > 0.0) {
            return (0.0, 0.0);
        }
        let mut p = rate_limit(p_req_w, self.p_max_w, self.p_max_w);
        // Matched-load ceiling (V²/4R), with a 1 % guard.
        if self.esr_ohm > 0.0 {
            p = p.min(0.99 * self.v_v * self.v_v / (4.0 * self.esr_ohm));
        }
        let mut i = self.current_for_power_a(p).unwrap_or(0.0);
        let mut internal_j = self.v_v * i * dt_s; // energy leaving the capacitance
        if internal_j > self.usable_j() {
            // Scale down to what remains above V_min.
            let scale = if internal_j > 0.0 {
                self.usable_j() / internal_j
            } else {
                0.0
            };
            i *= scale;
            internal_j = self.v_v * i * dt_s;
        } else if -internal_j > self.headroom_j() {
            let scale = self.headroom_j() / (-internal_j);
            i *= scale;
            internal_j = self.v_v * i * dt_s;
        }
        let loss_j = i * i * self.esr_ohm * dt_s;
        let p_term = (internal_j - loss_j) / dt_s;
        let e_new = (self.energy_j() - internal_j).max(0.0);
        self.v_v = fmath::sqrt(2.0 * e_new / self.c_f);
        (p_term, loss_j)
    }
}

// ---- Battery -----------------------------------------------------------------------------

/// Battery with a C-rate limit.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Battery {
    /// Capacity, J.
    pub capacity_j: f64,
    /// State of charge, 0..1.
    pub soc: f64,
    /// Minimum SOC, 0..1.
    pub soc_min: f64,
    /// C-rate limit, 1/h.
    pub c_rate_per_h: f64,
    /// Round-trip efficiency, 0..1.
    pub eta: f64,
}

impl Battery {
    /// One-way efficiency √η.
    pub fn eta_one_way(&self) -> f64 {
        fmath::sqrt(self.eta.clamp(1.0e-6, 1.0))
    }
}

impl EnergyStore for Battery {
    fn soc(&self) -> f64 {
        self.soc.clamp(0.0, 1.0)
    }

    fn usable_j(&self) -> f64 {
        ((self.soc - self.soc_min) * self.capacity_j).max(0.0)
    }

    fn headroom_j(&self) -> f64 {
        ((1.0 - self.soc) * self.capacity_j).max(0.0)
    }

    fn p_max_dis_w(&self) -> f64 {
        self.c_rate_per_h * self.capacity_j / 3600.0
    }

    fn p_max_chg_w(&self) -> f64 {
        self.c_rate_per_h * self.capacity_j / 3600.0
    }

    fn step(&mut self, p_req_w: f64, dt_s: f64) -> f64 {
        self.step_with_loss(p_req_w, dt_s).0
    }
}

impl LossyStore for Battery {
    fn stored_usable_j(&self) -> f64 {
        (self.soc - self.soc_min) * self.capacity_j
    }

    fn step_with_loss(&mut self, p_req_w: f64, dt_s: f64) -> (f64, f64) {
        if !(dt_s > 0.0 && self.capacity_j > 0.0) {
            return (0.0, 0.0);
        }
        let eta1 = self.eta_one_way();
        let mut p = rate_limit(p_req_w, self.p_max_dis_w(), self.p_max_chg_w());
        // Internal energy change per step: discharge draws p/η₁, charge stores |p|·η₁.
        let (internal_j, loss_j) = if p >= 0.0 {
            p = p.min(self.usable_j() * eta1 / dt_s);
            let drawn = p * dt_s / eta1;
            (drawn, drawn - p * dt_s)
        } else {
            p = p.max(-self.headroom_j() / (eta1 * dt_s));
            let stored = -p * dt_s * eta1;
            (-stored, -p * dt_s - stored)
        };
        self.soc -= internal_j / self.capacity_j;
        (p, loss_j)
    }
}

// ---- Hybrid store ------------------------------------------------------------------------

/// Target supercap SOC of the hybrid store (mid-window, so it can both deliver and absorb
/// transients).
pub const HYBRID_CAP_TARGET_SOC: f64 = 0.5;

/// Supercap for the high-pass share plus a flywheel for the rest.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HybridStore {
    /// Fast store.
    pub cap: Supercap,
    /// Bulk store.
    pub fly: Flywheel,
    /// Split time constant, s (1–5 s).
    pub tau_s: f64,
}

impl HybridStore {
    /// Supercap energy at the target SOC above V_min, J.
    fn cap_target_usable_j(&self) -> f64 {
        let c = &self.cap;
        HYBRID_CAP_TARGET_SOC * 0.5 * c.c_f * (c.v_max_v * c.v_max_v - c.v_min_v * c.v_min_v)
    }

    /// The flywheel's share of a demand `p_req_w`: the supercap-restoring term
    /// L = (E_target − E_cap)/τ. With dE_cap/dt = −(p − L) this gives τ·dL/dt = p − L, so
    /// L is the first-order low-pass of the demand (for a store that starts at its
    /// target) and the supercap carries the high-pass remainder. τ ≤ 0 puts everything
    /// on the flywheel.
    pub fn flywheel_share_w(&self, p_req_w: f64) -> f64 {
        if self.tau_s <= 0.0 {
            return p_req_w;
        }
        (self.cap_target_usable_j() - self.cap.stored_usable_j()) / self.tau_s
    }
}

impl EnergyStore for HybridStore {
    fn soc(&self) -> f64 {
        let total = self.cap.usable_j()
            + self.cap.headroom_j()
            + self.fly.usable_j()
            + self.fly.headroom_j();
        if total <= 0.0 {
            return 0.0;
        }
        (self.cap.usable_j() + self.fly.usable_j()) / total
    }

    fn usable_j(&self) -> f64 {
        self.cap.usable_j() + self.fly.usable_j()
    }

    fn headroom_j(&self) -> f64 {
        self.cap.headroom_j() + self.fly.headroom_j()
    }

    fn p_max_dis_w(&self) -> f64 {
        self.cap.p_max_dis_w() + self.fly.p_max_dis_w()
    }

    fn p_max_chg_w(&self) -> f64 {
        self.cap.p_max_chg_w() + self.fly.p_max_chg_w()
    }

    fn step(&mut self, p_req_w: f64, dt_s: f64) -> f64 {
        self.step_with_loss(p_req_w, dt_s).0
    }
}

impl LossyStore for HybridStore {
    fn stored_usable_j(&self) -> f64 {
        self.cap.stored_usable_j() + self.fly.stored_usable_j()
    }

    fn step_with_loss(&mut self, p_req_w: f64, dt_s: f64) -> (f64, f64) {
        if dt_s <= 0.0 {
            return (0.0, 0.0);
        }
        let p = rate_limit(p_req_w, self.p_max_dis_w(), self.p_max_chg_w());
        // The flywheel supplies its share plus whatever the supercap cannot.
        let (p_fly, l_fly) = self.fly.step_with_loss(self.flywheel_share_w(p), dt_s);
        let (p_cap, l_cap) = self.cap.step_with_loss(p - p_fly, dt_s);
        // Energy moved from the flywheel into the supercap is internal; only the net
        // terminal power reaches the bus.
        (p_fly + p_cap, l_fly + l_cap)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::suspension::{hybrid_hover_power_w, pure_ems_hover_power_w};
    use fabrail_exo::energy::{DISCHARGE_MARGIN, ride_through_energy_j, t_ride_s};
    use fabrail_params::derive::G0_MPS2;

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    // req: FAB-EXO-401
    #[test]
    fn flywheel_usable_fraction() {
        // Closed form (E): 1 − (ω_min/ω_max)² = 75 % at ω_min = ω_max/2 (FAB-EXO-401).
        assert_eq!(super::flywheel_usable_fraction(0.5), 0.75);
        assert_eq!(super::flywheel_usable_fraction(0.0), 1.0);
        assert_eq!(super::flywheel_usable_fraction(1.0), 0.0);
        // EMALS-class disk: ≈ 357 MJ at 6400 rpm; 121 MJ per launch takes it to 5205 rpm.
        let mut fw = Flywheel::emals_class();
        assert!(rel(fw.energy_j(), EMALS_STORED_J) < 1e-12);
        assert!(
            rel(fw.usable_j() / fw.energy_j(), 0.75) < 1e-12,
            "75 % usable at ω_max/2"
        );
        let w_launch = rpm_to_rad_s(EMALS_RPM_AFTER_LAUNCH);
        let per_launch = 0.5 * fw.i_kgm2 * (fw.omega_rad_s * fw.omega_rad_s - w_launch * w_launch);
        assert!(
            rel(per_launch, 121.0e6) < 5e-3,
            "121 MJ per cycle (6400 → 5205 rpm), got {per_launch:.3e}"
        );
        // Lossless constant-power discharge delivers exactly the usable energy.
        let usable0 = fw.usable_j();
        let (mut delivered, dt) = (0.0, 0.01);
        for _ in 0..1_000_000 {
            let p = fw.step(fw.p_max_w, dt);
            if p <= 0.0 {
                break;
            }
            delivered += p * dt;
        }
        assert!(
            rel(delivered, usable0) < 1e-9,
            "discharge delivers the usable energy (E)"
        );
        assert!(
            rel(fw.omega_rad_s, fw.omega_min_rad_s) < 1e-9,
            "stops at ω_min"
        );
        // Supercap and battery windows.
        let cap = Supercap {
            c_f: 10.0,
            v_v: 12.0,
            v_min_v: 6.0,
            v_max_v: 12.0,
            esr_ohm: 0.01,
            p_max_w: 500.0,
        };
        assert!(rel(cap.usable_j(), 0.5 * 10.0 * (144.0 - 36.0)) < 1e-12);
        let bat = Battery {
            capacity_j: 3.6e6,
            soc: 0.9,
            soc_min: 0.2,
            c_rate_per_h: 1.0,
            eta: 0.9,
        };
        assert!(rel(bat.usable_j(), 0.7 * 3.6e6) < 1e-12);
        // Flywheel budget (plan §4, T_ride 300 s): 10 kt hybrid at 0.2 g needs 51 MJ
        // (102 MJ with the 2× margin), inside one EMALS-class disk; pure EMS needs 5.1 GJ
        // and one JET-class unit is flagged insufficient.
        let t = t_ride_s(0.0, 0.0);
        let e_hybrid = ride_through_energy_j(hybrid_hover_power_w(10.0e6, 0.2 * G0_MPS2), t);
        let e_pure = ride_through_energy_j(pure_ems_hover_power_w(10.0e6), t);
        assert!(
            rel(e_hybrid, 51.0e6) < 1e-12 && rel(e_pure, 5.1e9) < 1e-12,
            "51 MJ / 5.1 GJ (EXOTIC §2)"
        );
        let emals = Flywheel::emals_class();
        let jet = Flywheel::jet_class();
        assert!(
            DISCHARGE_MARGIN * e_hybrid <= emals.usable_j(),
            "hybrid passes on one EMALS-class disk"
        );
        assert!(
            DISCHARGE_MARGIN * e_hybrid <= 121.0e6,
            "…even within one 121 MJ launch window"
        );
        assert!(
            DISCHARGE_MARGIN * e_pure > jet.usable_j(),
            "pure EMS flagged insufficient on one JET-class unit"
        );
        // The disk carries 170 kW for 300 s without a shortfall.
        let mut disk = Flywheel::emals_class();
        let p_hold = hybrid_hover_power_w(10.0e6, 0.2 * G0_MPS2);
        let mut e = 0.0;
        for _ in 0..3000 {
            let p = disk.step(p_hold, 0.1);
            assert!(rel(p, p_hold) < 1e-12, "no shortfall");
            e += p * 0.1;
        }
        assert!(rel(e, 51.0e6) < 1e-9 && disk.soc() > 0.5);
    }

    /// A 300 s duty of 3e5 steps: discharge pulses, regeneration above the charge rating
    /// (partly dumped) and idle, booked in f64 and i64 mJ. Returns (ledger, ledger_mj,
    /// regeneration offered).
    fn duty<S: LossyStore>(
        store: &mut S,
        p_dis: f64,
        p_regen: f64,
    ) -> (EnergyLedger, EnergyLedgerMj, f64) {
        let mut l = EnergyLedger::open(store);
        let mut lm = EnergyLedgerMj {
            e_start_mj: j_to_mj(l.e_start_j),
            e_end_mj: j_to_mj(l.e_start_j),
            ..EnergyLedgerMj::default()
        };
        let dt = 1.0e-3;
        let mut offered = 0.0;
        for k in 0..300_000_u32 {
            let t = f64::from(k) * dt;
            let phase = t % 20.0;
            let p = if phase < 6.0 {
                p_dis
            } else if phase < 9.0 {
                -p_regen
            } else {
                0.0
            };
            if p < 0.0 {
                offered += -p * dt;
            }
            exchange(store, p, dt, &mut l, &mut lm);
        }
        (l, lm, offered)
    }

    // req: FAB-EXO-404
    #[test]
    fn energy_balance_closes() {
        // Flywheel with a + b·ω² losses; regeneration above its rating is dumped.
        let mut fw = Flywheel::emals_class();
        fw.omega_rad_s *= 0.8;
        fw.loss_a_w = 20.0e3;
        fw.loss_b = 0.1;
        fw.p_max_w = 2.0e6;
        let (l, lm, offered) = duty(&mut fw, 1.5e6, 3.0e6);
        println!(
            "flywheel: out {:.3e} J, in {:.3e} J, dump {:.3e} J, loss {:.3e} J, f64 err {:.2e}, i64 mJ err {:.2e}",
            l.e_out_j,
            l.e_in_j,
            l.e_dump_j,
            l.e_loss_j,
            l.balance_error(),
            lm.balance_error()
        );
        assert!(
            l.balance_error() < 0.01,
            "f64 ledger closes within 1 % (FAB-EXO-404)"
        );
        assert!(l.balance_error() < 1e-9, "…in fact to rounding");
        assert!(
            lm.balance_error() < 0.01,
            "i64 mJ ledger closes within 1 % (FAB-EXO-404)"
        );
        assert!(
            l.bus_balance_error(offered) < 1e-9,
            "regeneration = absorbed + dumped"
        );
        assert!(
            l.e_dump_j > 0.0 && l.e_loss_j > 0.0,
            "the duty exercised dump and losses"
        );
        // The same for the supercap (ESR), battery (√η one-way) and the hybrid store.
        let mut cap = Supercap {
            c_f: 5.0e3,
            v_v: 700.0,
            v_min_v: 350.0,
            v_max_v: 800.0,
            esr_ohm: 0.02,
            p_max_w: 1.0e6,
        };
        let mut bat = Battery {
            capacity_j: 3.6e9,
            soc: 0.6,
            soc_min: 0.2,
            c_rate_per_h: 1.0,
            eta: 0.9,
        };
        let mut hyb = HybridStore {
            cap: Supercap {
                c_f: 5.0e3,
                v_v: 600.0,
                v_min_v: 350.0,
                v_max_v: 800.0,
                esr_ohm: 0.02,
                p_max_w: 1.0e6,
            },
            fly: Flywheel {
                loss_a_w: 5.0e3,
                ..Flywheel::emals_class()
            },
            tau_s: 3.0,
        };
        let results = [
            ("supercap", duty(&mut cap, 3.0e5, 1.2e6)),
            ("battery", duty(&mut bat, 5.0e5, 2.0e6)),
            ("hybrid", duty(&mut hyb, 8.0e5, 3.0e6)),
        ];
        for (name, (l, lm, offered)) in results {
            println!(
                "{name}: f64 err {:.2e}, i64 mJ err {:.2e}, dump {:.3e} J",
                l.balance_error(),
                lm.balance_error(),
                l.e_dump_j
            );
            assert!(l.balance_error() < 1e-9, "{name}: f64 ledger closes");
            assert!(
                lm.balance_error() < 0.01,
                "{name}: i64 mJ ledger closes within 1 %"
            );
            assert!(l.bus_balance_error(offered) < 1e-9, "{name}: bus balance");
        }
    }

    #[test]
    fn hybrid_store_splits_high_pass() {
        // A demand step at the target SOC: the supercap takes the step, the flywheel ramps
        // in as the first-order low-pass (τ), carrying ≈ 1 − e⁻⁵ of it after 5τ.
        let cap = Supercap {
            c_f: 5.0e3,
            v_v: 0.0,
            v_min_v: 350.0,
            v_max_v: 800.0,
            esr_ohm: 0.0,
            p_max_w: 5.0e6,
        };
        let mut h = HybridStore {
            cap,
            fly: Flywheel::emals_class(),
            tau_s: 2.0,
        };
        // Start the supercap at its target energy.
        let target = HYBRID_CAP_TARGET_SOC * 0.5 * 5.0e3 * (800.0 * 800.0 - 350.0 * 350.0);
        h.cap.v_v = ((2.0 * target / 5.0e3) + 350.0 * 350.0).sqrt();
        assert!(h.flywheel_share_w(0.0).abs() < 1e-6);
        let p = 1.0e6;
        let dt = 1.0e-3;
        let mut t = 0.0;
        let mut share_at_tau = 0.0;
        while t < 5.0 * h.tau_s {
            if (t - h.tau_s).abs() < 0.5 * dt {
                share_at_tau = h.flywheel_share_w(p) / p;
            }
            let got = h.step(p, dt);
            assert!(rel(got, p) < 1e-9, "the pair meets the demand");
            t += dt;
        }
        assert!(
            (share_at_tau - (1.0 - (-1.0_f64).exp())).abs() < 5e-3,
            "flywheel at 1 − e⁻¹ after τ: {share_at_tau}"
        );
        assert!(
            h.flywheel_share_w(p) / p > 0.99,
            "flywheel carries the low-pass after 5τ"
        );
    }
}
