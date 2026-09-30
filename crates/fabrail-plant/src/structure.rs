//! Structure tier: n-storey shear building (Rayleigh damping) with the maglev mass, and floor amplification (PHYSICS §21). Owner: A6b.
//!
//! Research tier only (AMD/TMD, SEI-021): the device's LSM force acts between the
//! maglev mass and its storey; the ground acceleration enters as `floor_mps2` (the
//! structure tier interprets the harness "floor" input as GROUND motion at the base).
//! `Truth::host_*` report the device storey; `Truth::x_m` is the maglev stroke
//! relative to that storey.
//!
//! Equations (PHYSICS §21; physics perspective §1.0 "Structure"), storey displacements
//! u relative to the ground, device stroke x relative to storey j, ground acceleration
//! a_g (x component of the floor input):
//!
//! M·ü + C·u̇ + K·u = −M·1·a_g − e_j·F_int
//! m_d·(ẍ + ü_j + a_g) = F_int
//!
//! with K the shear-building tridiagonal stiffness, C = a₀·M + a₁·K Rayleigh damping
//! fitted to ζ₁, ζ₂ at the first two modes (Chopra, Dynamics of Structures, 4th ed.,
//! Eq. 11.4.9; one storey: c = 2ζ₁√(k·m)), m_d the truth device mass (body + payload)
//! and F_int the device–storey interaction: LSM thrust (Run: K_f·i_q with the current
//! loop algebraic at the 1 ms step, |i_q| ≤ i_max; ASC: back-EMF drag; STO: 0), the
//! dissipation ledger at nominal flux (`dissipation::drag`), the end bumpers, the
//! physical end stops (`contact::end_stops`, W2 D3), the parking lock and a lean force. The maglev is assumed levitated throughout (vertical
//! dynamics and hand interaction are outside this tier). RK4 at 1 ms; the dissipation,
//! LSM and floor-work ledgers and the hold-up voltage ride in the state vector, so they
//! are integrated to RK4 accuracy.

use fabrail_types::{LsmMode, fmath};

use crate::contact::{
    BumperLog, ContactKind, bumper_force_n, bumper_penetration, closed_stop_force_n,
    contact_params, end_stop_law, end_stops, hunt_crossley_dissipation_w, lock_force_n,
    open_stop_force_n, penalty_dissipation_w,
};
use crate::dissipation::drag;
use crate::integrator::{STAGE_FLOOR_INDEX, rk4_step};
use crate::lsm::{asc_drag_n_s_per_m, k_e_from_kf};
use crate::model::{
    ContactState, InitialState, ModelConfig, PlantError, PlantInputs, PlantModel, PowerTruth, Tier,
    Truth,
};
use crate::params::PlantParams;
use crate::power::{power_flow, usable_energy_j};

/// Maximum number of storeys the Structure tier accepts (implementation limit; the AMD/TMD
/// studies use 1–5 storeys; PHYSICS §21).
pub const MAX_STOREYS: usize = 10;

/// Lumped shear building (ground → storey 1 → … → roof).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ShearBuilding {
    /// Storey masses, bottom to top, kg.
    pub storey_mass_kg: Vec<f64>,
    /// Storey shear stiffnesses, bottom to top, N/m.
    pub storey_k_n_per_m: Vec<f64>,
    /// Rayleigh damping ratios for modes 1 and 2 (2–5 %, core design §C.1). With one
    /// storey only the first is used: c = 2·ζ₁·√(k·m).
    pub rayleigh_zeta: [f64; 2],
    /// Storey carrying the device, 1-based (1 = first suspended storey).
    pub device_storey: u32,
}

impl ShearBuilding {
    /// Number of storeys.
    pub fn n_storeys(&self) -> usize {
        self.storey_mass_kg.len()
    }

    /// Range checks: 1..=MAX_STOREYS storeys, equal lengths, positive finite masses and
    /// stiffnesses, ζ in [0, 1), device storey within the building.
    pub fn validate(&self) -> Result<(), PlantError> {
        let n = self.storey_mass_kg.len();
        if n == 0 || n > MAX_STOREYS {
            return Err(PlantError::Invalid(
                "plant.structure.storey_mass_kg",
                "needs 1..=10 storeys",
            ));
        }
        if self.storey_k_n_per_m.len() != n {
            return Err(PlantError::Invalid(
                "plant.structure.storey_k_n_per_m",
                "must have one entry per storey",
            ));
        }
        let pos = |v: &f64| v.is_finite() && *v > 0.0;
        if !self.storey_mass_kg.iter().all(pos) {
            return Err(PlantError::Invalid(
                "plant.structure.storey_mass_kg",
                "must be finite and > 0",
            ));
        }
        if !self.storey_k_n_per_m.iter().all(pos) {
            return Err(PlantError::Invalid(
                "plant.structure.storey_k_n_per_m",
                "must be finite and > 0",
            ));
        }
        if !self
            .rayleigh_zeta
            .iter()
            .all(|z| z.is_finite() && (0.0..1.0).contains(z))
        {
            return Err(PlantError::Invalid(
                "plant.structure.rayleigh_zeta",
                "must be in [0, 1)",
            ));
        }
        if self.device_storey == 0 || self.device_storey as usize > n {
            return Err(PlantError::Invalid(
                "plant.structure.device_storey",
                "must be in 1..=n_storeys",
            ));
        }
        Ok(())
    }

    /// A single-storey (SDOF) building of mass `m_kg` and period `period_s` with damping
    /// ratio `zeta`: k = m·(2π/T)² (PHYSICS §21).
    pub fn sdof(m_kg: f64, period_s: f64, zeta: f64) -> ShearBuilding {
        let w = if period_s > 0.0 {
            2.0 * core::f64::consts::PI / period_s
        } else {
            0.0
        };
        ShearBuilding {
            storey_mass_kg: vec![m_kg],
            storey_k_n_per_m: vec![m_kg * w * w],
            rayleigh_zeta: [zeta, zeta],
            device_storey: 1,
        }
    }

    /// `n` identical storeys (m, k) with ζ at modes 1 and 2 and the device on
    /// `device_storey` (PHYSICS §21).
    pub fn uniform(
        n: usize,
        m_kg: f64,
        k_n_per_m: f64,
        zeta: f64,
        device_storey: u32,
    ) -> ShearBuilding {
        ShearBuilding {
            storey_mass_kg: vec![m_kg; n],
            storey_k_n_per_m: vec![k_n_per_m; n],
            rayleigh_zeta: [zeta, zeta],
            device_storey,
        }
    }

    /// Tridiagonal stiffness: diagonal k_i + k_{i+1} and super-diagonal −k_{i+1}.
    fn stiffness(&self) -> (Vec<f64>, Vec<f64>) {
        let n = self.n_storeys();
        let k = &self.storey_k_n_per_m;
        let diag = (0..n)
            .map(|i| k[i] + if i + 1 < n { k[i + 1] } else { 0.0 })
            .collect();
        let off = (0..n.saturating_sub(1)).map(|i| -k[i + 1]).collect();
        (diag, off)
    }

    /// Undamped natural frequencies ω_i, ascending, rad/s (eigenvalues of
    /// M^(−½)·K·M^(−½) by Jacobi rotations; PHYSICS §21). Empty for an invalid building.
    pub fn modal_omegas_radps(&self) -> Vec<f64> {
        if self.validate().is_err() {
            return Vec::new();
        }
        let n = self.n_storeys();
        let (diag, off) = self.stiffness();
        let m = &self.storey_mass_kg;
        let mut a = [[0.0f64; MAX_STOREYS]; MAX_STOREYS];
        for i in 0..n {
            a[i][i] = diag[i] / m[i];
            if i + 1 < n {
                let v = off[i] / fmath::sqrt(m[i] * m[i + 1]);
                a[i][i + 1] = v;
                a[i + 1][i] = v;
            }
        }
        let mut w: Vec<f64> = jacobi_eigenvalues(&mut a, n)
            .iter()
            .take(n)
            .map(|&l| fmath::sqrt(l.max(0.0)))
            .collect();
        w.sort_by(f64::total_cmp);
        w
    }

    /// Rayleigh coefficients (a₀, a₁) of C = a₀·M + a₁·K matching ζ₁, ζ₂ at the first
    /// two modes (Chopra Eq. 11.4.9: a₀ = 2ω₁ω₂(ζ₁ω₂ − ζ₂ω₁)/(ω₂² − ω₁²),
    /// a₁ = 2(ζ₂ω₂ − ζ₁ω₁)/(ω₂² − ω₁²)); one storey (or coincident modes): a₀ = ζ₁ω₁,
    /// a₁ = ζ₁/ω₁, i.e. c = 2ζ₁√(k·m). (0, 0) for an invalid building.
    pub fn rayleigh_coefficients(&self) -> (f64, f64) {
        let w = self.modal_omegas_radps();
        let [z1, z2] = self.rayleigh_zeta;
        match w.as_slice() {
            [] => (0.0, 0.0),
            [w1] => (z1 * w1, if *w1 > 0.0 { z1 / w1 } else { 0.0 }),
            [w1, w2, ..] => {
                let den = w2 * w2 - w1 * w1;
                if den.abs() <= 1e-12 * w2 * w2 || *w1 <= 0.0 {
                    (z1 * w1, z1 / w1)
                } else {
                    (
                        2.0 * w1 * w2 * (z1 * w2 - z2 * w1) / den,
                        2.0 * (z2 * w2 - z1 * w1) / den,
                    )
                }
            }
        }
    }
}

/// Eigenvalues of the symmetric leading n×n block of `a` by cyclic Jacobi rotations
/// (Numerical Recipes 3rd ed. §11.1), destroying `a`. Uses only +, −, ×, ÷ and √, so
/// native and wasm32 builds agree bit for bit (PHYSICS §24); converges quadratically,
/// ≤ 50 sweeps for n ≤ 10. Index loops mirror the matrix notation of the reference.
#[allow(clippy::needless_range_loop)]
fn jacobi_eigenvalues(a: &mut [[f64; MAX_STOREYS]; MAX_STOREYS], n: usize) -> [f64; MAX_STOREYS] {
    let n = n.min(MAX_STOREYS);
    for _sweep in 0..50 {
        let mut off = 0.0;
        let mut scale = 0.0;
        for i in 0..n {
            scale += a[i][i] * a[i][i];
            for j in (i + 1)..n {
                off += a[i][j] * a[i][j];
            }
        }
        if off <= 1e-32 * scale {
            break;
        }
        for p in 0..n {
            for q in (p + 1)..n {
                let apq = a[p][q];
                if apq == 0.0 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * apq);
                let sgn = if theta >= 0.0 { 1.0 } else { -1.0 };
                let t = sgn / (theta.abs() + fmath::sqrt(theta * theta + 1.0));
                let c = 1.0 / fmath::sqrt(t * t + 1.0);
                let sn = t * c;
                for k in 0..n {
                    let (akp, akq) = (a[k][p], a[k][q]);
                    a[k][p] = c * akp - sn * akq;
                    a[k][q] = sn * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (a[p][k], a[q][k]);
                    a[p][k] = c * apk - sn * aqk;
                    a[q][k] = sn * apk + c * aqk;
                }
            }
        }
    }
    let mut out = [0.0; MAX_STOREYS];
    for (i, o) in out.iter_mut().enumerate().take(n) {
        *o = a[i][i];
    }
    out
}

// State layout: storey displacements, storey velocities, device stroke and velocity, the
// ledger quadratures and the hold-up voltage.
const S_U: usize = 0;
const S_UD: usize = MAX_STOREYS;
const S_X: usize = 2 * MAX_STOREYS;
const S_V: usize = S_X + 1;
const S_L: usize = S_V + 1;
const L_DIST: usize = 0;
const L_EZ: usize = 1;
const L_EDDY: usize = 2;
const L_HYST: usize = 3;
const L_AIR: usize = 4;
const L_BUMPER: usize = 5;
const L_CONTACT: usize = 6;
const L_LSM_CU: usize = 7;
const L_ELEC: usize = 8;
const L_FLOOR: usize = 9;
const N_LEDGER: usize = 10;
const S_VHOLD: usize = S_L + N_LEDGER;
const NS: usize = S_VHOLD + 1;

type State = [f64; NS];

/// Step-constant inputs of one RK4 step.
#[derive(Clone, Copy)]
struct StepIn<'a> {
    u: &'a PlantInputs,
    a_g: [f64; 3],
    m_d: f64,
    lock_anchor: Option<f64>,
}

/// Loads and accelerations at one stage.
#[derive(Clone, Copy, Default)]
struct StageOut {
    f_lsm: f64,
    iq: f64,
    bumper: f64,
    stop: f64,
    lock: f64,
    udd_j: f64,
    xdd: f64,
}

/// Plant step the Structure tier's contact laws are sized for, s (the 1 ms tier step).
const STEP_S: f64 = Tier::Structure.nominal_h_s();

/// Shear building plus maglev mass.
#[derive(Clone, Debug)]
pub struct Structure {
    p: PlantParams,
    building: ShearBuilding,
    cfg: ModelConfig,
    truth: Truth,
    /// Storey masses (padded), kg.
    m: [f64; MAX_STOREYS],
    /// Tridiagonal stiffness (diagonal, super-diagonal), N/m.
    k_diag: [f64; MAX_STOREYS],
    k_off: [f64; MAX_STOREYS],
    /// Rayleigh damping (diagonal, super-diagonal), N·s/m.
    c_diag: [f64; MAX_STOREYS],
    c_off: [f64; MAX_STOREYS],
    /// Number of storeys and the device storey (0-based).
    n: usize,
    j: usize,
    y: State,
    lock_anchor: Option<f64>,
    bumper: BumperLog,
}

impl Structure {
    /// A Structure model over `p` and `building` (validated here).
    pub fn new(
        p: &PlantParams,
        building: &ShearBuilding,
        cfg: &ModelConfig,
    ) -> Result<Structure, PlantError> {
        building.validate()?;
        let n = building.n_storeys();
        let (diag, off) = building.stiffness();
        let (a0, a1) = building.rayleigh_coefficients();
        let mut s = Structure {
            p: p.clone(),
            building: building.clone(),
            cfg: *cfg,
            truth: Truth::initial(p, &InitialState::default()),
            m: [0.0; MAX_STOREYS],
            k_diag: [0.0; MAX_STOREYS],
            k_off: [0.0; MAX_STOREYS],
            c_diag: [0.0; MAX_STOREYS],
            c_off: [0.0; MAX_STOREYS],
            n,
            j: (building.device_storey as usize).saturating_sub(1),
            y: [0.0; NS],
            lock_anchor: None,
            bumper: BumperLog::default(),
        };
        for i in 0..n {
            s.m[i] = building.storey_mass_kg[i];
            s.k_diag[i] = diag[i];
            s.c_diag[i] = a0 * s.m[i] + a1 * diag[i];
            if i + 1 < n {
                s.k_off[i] = off[i];
                s.c_off[i] = a1 * off[i];
            }
        }
        s.reset(&InitialState::default());
        Ok(s)
    }

    /// The host building.
    pub fn building(&self) -> &ShearBuilding {
        &self.building
    }

    /// The construction-time options.
    pub fn config(&self) -> &ModelConfig {
        &self.cfg
    }

    /// Mechanical energy of building and device (kinetic in the ground frame plus storey
    /// strain energy), J (PHYSICS §21). Excludes the lock and bumper springs.
    pub fn mechanical_energy_j(&self) -> f64 {
        let y = &self.y;
        let mut e = 0.0;
        for i in 0..self.n {
            let ud = y[S_UD + i];
            e += 0.5 * self.m[i] * ud * ud;
            let drift = y[S_U + i] - if i > 0 { y[S_U + i - 1] } else { 0.0 };
            e += 0.5 * self.building.storey_k_n_per_m[i] * drift * drift;
        }
        let vd = y[S_UD + self.j] + y[S_V];
        e + 0.5 * self.truth.mass_kg * vd * vd
    }

    /// Device–storey loads and accelerations at a stage (PHYSICS §21).
    fn loads(&self, y: &State, si: &StepIn<'_>, udd: &mut [f64; MAX_STOREYS]) -> StageOut {
        let p = &self.p;
        let l = &p.design.lsm;
        let (x, v) = (y[S_X], y[S_V]);
        let mut o = StageOut::default();
        match si.u.lsm_mode {
            LsmMode::Run => {
                let imax = if l.i_max_a > 0.0 {
                    l.i_max_a
                } else {
                    f64::INFINITY
                };
                o.iq = si.u.iq_ref_a.clamp(-imax, imax);
                o.f_lsm = l.k_f_n_per_a * o.iq;
            }
            LsmMode::Asc => {
                let w_e = if l.tau_p_m > 0.0 {
                    core::f64::consts::PI * v / l.tau_p_m
                } else {
                    0.0
                };
                // K_e = (2/3)·K_f in the amplitude-invariant dq frame (D5; PHYSICS §7).
                o.f_lsm = -asc_drag_n_s_per_m(
                    l.k_f_n_per_a,
                    k_e_from_kf(l.k_f_n_per_a),
                    l.r_phase_ohm,
                    l.l_phase_h,
                    w_e,
                ) * v;
            }
            LsmMode::Sto => {}
        }
        let dr = drag(p, x, v, 1.0);
        o.bumper = bumper_force_n(p, x, v);
        // Physical end stops at the class geometry, as on the other tiers (W2 D3).
        o.stop = closed_stop_force_n(p, x, v, si.m_d, STEP_S)
            + open_stop_force_n(p, x, v, si.m_d, STEP_S);
        o.lock = si.lock_anchor.map_or(0.0, |a| lock_force_n(p, x, a, v));
        let f_int = o.f_lsm + dr.total_n() + o.bumper + o.stop + o.lock + si.u.f_ext.lean_n;
        let a_g = si.a_g[0];
        for i in 0..self.n {
            let mut f =
                -self.m[i] * a_g - self.k_diag[i] * y[S_U + i] - self.c_diag[i] * y[S_UD + i];
            if i + 1 < self.n {
                f -= self.k_off[i] * y[S_U + i + 1] + self.c_off[i] * y[S_UD + i + 1];
            }
            if i > 0 {
                f -= self.k_off[i - 1] * y[S_U + i - 1] + self.c_off[i - 1] * y[S_UD + i - 1];
            }
            if i == self.j {
                f -= f_int;
            }
            udd[i] = f / self.m[i];
        }
        o.udd_j = udd[self.j];
        o.xdd = f_int / si.m_d - o.udd_j - a_g;
        o
    }

    /// State derivative at a stage (PHYSICS §21, §9, §14).
    fn rates(&self, y: &State, si: &StepIn<'_>) -> (State, StageOut) {
        let p = &self.p;
        let mut udd = [0.0; MAX_STOREYS];
        let o = self.loads(y, si, &mut udd);
        let mut dy = [0.0; NS];
        let n = self.n;
        dy[S_U..S_U + n].copy_from_slice(&y[S_UD..S_UD + n]);
        dy[S_UD..S_UD + n].copy_from_slice(&udd[..n]);
        let (x, v) = (y[S_X], y[S_V]);
        dy[S_X] = v;
        dy[S_V] = o.xdd;
        // Ledger quadratures: each drag's −F·v, bumper and lock damping, LSM power.
        let dr = drag(p, x, v, 1.0);
        dy[S_L + L_DIST] = -dr.dist_n * v;
        dy[S_L + L_EZ] = -dr.end_zone_n * v;
        dy[S_L + L_EDDY] = -dr.rail_eddy_n * v;
        dy[S_L + L_HYST] = -dr.rail_hyst_n * v;
        dy[S_L + L_AIR] = -dr.air_n * v;
        let (delta, dir) = bumper_penetration(&p.design, x);
        if delta > 0.0 {
            let c = contact_params(p, ContactKind::Bumper);
            dy[S_L + L_BUMPER] =
                hunt_crossley_dissipation_w(c.k_n_per_m, c.c_n_s_per_m, delta, -dir * v);
        }
        if si.lock_anchor.is_some() {
            let c = contact_params(p, ContactKind::ParkingLock);
            dy[S_L + L_CONTACT] = c.c_n_s_per_m * v * v;
        }
        // End-stop damping (W2 D3): c·δ̇² while a stop pushes.
        let (ks, cs) = end_stop_law(p, si.m_d, STEP_S);
        let st = end_stops(&p.design);
        dy[S_L + L_CONTACT] += penalty_dissipation_w(ks, cs, x - st.open_m, v);
        if st.push_stop_m.is_none() {
            dy[S_L + L_CONTACT] += penalty_dissipation_w(ks, cs, st.closed_m - x, -v);
        }
        let l = &p.design.lsm;
        let (p_cu, p_elec) = match si.u.lsm_mode {
            LsmMode::Run => {
                let cu = 1.5 * l.r_phase_ohm.max(0.0) * o.iq * o.iq;
                (cu, o.f_lsm * v + cu)
            }
            // ASC turns the braking work into winding heat; nothing crosses the bus.
            LsmMode::Asc => (-o.f_lsm * v, 0.0),
            LsmMode::Sto => (0.0, 0.0),
        };
        dy[S_L + L_LSM_CU] = p_cu;
        dy[S_L + L_ELEC] = p_elec;
        dy[S_L + L_FLOOR] = -si.m_d * (o.udd_j + si.a_g[0]) * v;
        let pf = power_flow(
            &p.design.power,
            y[S_VHOLD],
            p_elec + p.design.power.p_electronics_w,
            si.u.mains_ok,
        );
        dy[S_VHOLD] = pf.dv_hold_dt_v_per_s;
        (dy, o)
    }
}

impl PlantModel for Structure {
    fn tier(&self) -> Tier {
        Tier::Structure
    }

    fn reset(&mut self, init: &InitialState) {
        self.truth = Truth::initial(&self.p, init);
        self.y = [0.0; NS];
        self.y[S_X] = init.x_m;
        self.y[S_V] = init.v_mps;
        self.y[S_VHOLD] = self.p.design.power.supercap_v_max_v;
        let fitted = !contact_params(&self.p, ContactKind::ParkingLock).is_inert();
        self.lock_anchor = if init.locked && fitted {
            Some(init.x_m)
        } else {
            None
        };
        self.bumper = BumperLog::default();
        self.truth.state = ContactState {
            levitated: true,
            locked: self.lock_anchor.is_some(),
            ..ContactState::default()
        };
    }

    fn step(&mut self, t: f64, h: f64, u: &PlantInputs, floor_mps2: &[[f64; 3]; 3]) {
        // Parking lock: spring-applied where it engages, released on command.
        if u.parking_lock && !contact_params(&self.p, ContactKind::ParkingLock).is_inert() {
            if self.lock_anchor.is_none() {
                self.lock_anchor = Some(self.y[S_X]);
            }
        } else {
            self.lock_anchor = None;
        }
        let m_d = (self.p.m_body_kg + u.payload_kg).max(1e-6);
        let base = StepIn {
            u,
            a_g: floor_mps2[0],
            m_d,
            lock_anchor: self.lock_anchor,
        };
        let y0 = self.y;
        let y1 = rk4_step(&y0, t, h, |stage, _, y| {
            let si = StepIn {
                a_g: floor_mps2[STAGE_FLOOR_INDEX[stage]],
                ..base
            };
            self.rates(y, &si).0
        });
        self.y = y1;
        let end = StepIn {
            a_g: floor_mps2[2],
            ..base
        };
        let (_, o) = self.rates(&self.y, &end);
        let (x, v) = (self.y[S_X], self.y[S_V]);
        let a_g = floor_mps2[2];

        let tr = &mut self.truth;
        tr.t_s = t + h;
        tr.x_m = x;
        tr.v_mps = v;
        tr.a_mps2 = o.xdd;
        tr.pose = [x, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        tr.omega_radps = [0.0; 3];
        tr.acc_abs_mps2 = [o.xdd + o.udd_j + a_g[0], a_g[1], a_g[2]];
        tr.iq_a = o.iq;
        tr.id_a = 0.0;
        tr.f_lsm_n = o.f_lsm;
        tr.f_contact.bumper_n = o.bumper.abs();
        tr.f_contact.end_stop_n = o.stop.abs();
        tr.f_contact.lock_n = o.lock.abs();
        tr.floor_acc_mps2 = [o.udd_j + a_g[0], a_g[1], a_g[2]];
        tr.temps_c = [u.t_ambient_c, u.t_ambient_c];
        tr.mass_kg = m_d;
        tr.payload_kg = u.payload_kg;
        tr.host_x_m = self.y[S_U + self.j];
        tr.host_v_mps = self.y[S_UD + self.j];
        tr.host_a_mps2 = o.udd_j + a_g[0];
        let l = &mut tr.e_diss;
        l.kinetic_j = 0.5 * m_d * v * v;
        l.dist_j = self.y[S_L + L_DIST];
        l.end_zone_j = self.y[S_L + L_EZ];
        l.rail_eddy_j = self.y[S_L + L_EDDY];
        l.rail_hyst_j = self.y[S_L + L_HYST];
        l.air_j = self.y[S_L + L_AIR];
        l.bumper_j = self.y[S_L + L_BUMPER];
        l.contact_j = self.y[S_L + L_CONTACT];
        l.lsm_copper_j = self.y[S_L + L_LSM_CU];
        l.elec_in_j = self.y[S_L + L_ELEC];
        l.floor_work_j = self.y[S_L + L_FLOOR];
        let (delta, _) = bumper_penetration(&self.p.design, x);
        self.bumper.update(&self.p, &mut tr.e_diss, delta, v, m_d);
        let pw = &self.p.design.power;
        let v_hold = self.y[S_VHOLD];
        let p_elec = match u.lsm_mode {
            LsmMode::Run => {
                o.f_lsm * v + 1.5 * self.p.design.lsm.r_phase_ohm.max(0.0) * o.iq * o.iq
            }
            _ => 0.0,
        };
        let pf = power_flow(pw, v_hold, p_elec + pw.p_electronics_w, u.mains_ok);
        tr.power = PowerTruth {
            v_bus_v: pf.v_bus_v,
            v_hold_v: v_hold,
            i_bus_a: pf.i_bus_a,
            p_bus_w: pf.p_bus_w,
            e_hold_j: usable_energy_j(pw, v_hold),
            mains_ok: u.mains_ok,
            chopper_on: pf.p_chopper_w > 0.0,
        };
        tr.state = ContactState {
            levitated: true,
            locked: self.lock_anchor.is_some(),
            ..ContactState::default()
        };
    }

    fn truth(&self) -> &Truth {
        &self.truth
    }
}

/// Peak floor acceleration amplification at relative height z/h: 1 + 2·z/h (ASCE 7-16
/// §13.3.1 Eq. 13.3-1; core design §C.5 uses z/h = 0.5 → 2.0). `z_over_h` is clamped
/// to [0, 1]; a non-finite input returns 1.
pub fn floor_amplification(z_over_h: f64) -> f64 {
    if !z_over_h.is_finite() {
        return 1.0;
    }
    1.0 + 2.0 * z_over_h.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::{Environment, Mismatch};
    use fabrail_params::DesignParams;

    /// A frictionless research device of mass `m_d` on a ±5 m stroke (no drag, no
    /// bumpers, no lock), LSM K_f 100 N/A with no current limit.
    fn device(m_d: f64) -> PlantParams {
        let mut d = DesignParams::default();
        d.carriage.m_star_kg = m_d;
        d.rail.x_closed_m = -5.0;
        d.rail.travel_m = 10.0;
        d.lsm.k_f_n_per_a = 100.0;
        PlantParams::from_design(&d, &Mismatch::default(), &Environment::default())
    }

    fn run_ground<F: FnMut(&Structure) -> f64>(
        s: &mut Structure,
        t_end: f64,
        a_g: impl Fn(f64) -> f64,
        mut iq: F,
    ) -> Vec<(f64, f64)> {
        let h = Tier::Structure.nominal_h_s();
        let n = (t_end / h).round() as u64;
        let mut out = Vec::with_capacity(n as usize);
        let mut u = PlantInputs {
            lsm_mode: LsmMode::Run,
            ..PlantInputs::default()
        };
        for k in 0..n {
            let t = k as f64 * h;
            u.iq_ref_a = iq(s);
            let floor = [
                [a_g(t), 0.0, 0.0],
                [a_g(t + 0.5 * h), 0.0, 0.0],
                [a_g(t + h), 0.0, 0.0],
            ];
            s.step(t, h, &u, &floor);
            out.push((t + h, s.truth().host_x_m));
        }
        out
    }

    #[test]
    fn floor_amplification_matches_asce7() {
        assert_eq!(floor_amplification(0.0), 1.0);
        assert_eq!(floor_amplification(0.5), 2.0);
        assert_eq!(floor_amplification(1.0), 3.0);
        assert_eq!(floor_amplification(2.0), 3.0);
        assert_eq!(floor_amplification(f64::NAN), 1.0);
    }

    #[test]
    fn building_validation() {
        let ok = ShearBuilding {
            storey_mass_kg: vec![1.0e5, 1.0e5],
            storey_k_n_per_m: vec![5.0e7, 5.0e7],
            rayleigh_zeta: [0.02, 0.05],
            device_storey: 2,
        };
        ok.validate().expect("valid");
        let mut b = ok.clone();
        b.device_storey = 3;
        assert!(b.validate().is_err());
        let mut b = ok.clone();
        b.storey_k_n_per_m.pop();
        assert!(b.validate().is_err());
        let mut b = ok;
        b.rayleigh_zeta = [1.0, 0.0];
        assert!(b.validate().is_err());
    }

    #[test]
    fn two_dof_modes_and_rayleigh_fit() {
        // Equal storeys: ω² = (k/m)·(3 ∓ √5)/2 ((E) rel 1e-12, PHYSICS §21), and the
        // Rayleigh pair reproduces ζ₁, ζ₂ at those modes (ζ_i = a₀/(2ω_i) + a₁ω_i/2).
        let (m, k) = (2.0e5, 8.0e7);
        let b = ShearBuilding {
            rayleigh_zeta: [0.02, 0.05],
            ..ShearBuilding::uniform(2, m, k, 0.0, 2)
        };
        let w = b.modal_omegas_radps();
        let s5 = 5.0f64.sqrt();
        let want = [
            (k / m * (3.0 - s5) / 2.0).sqrt(),
            (k / m * (3.0 + s5) / 2.0).sqrt(),
        ];
        for i in 0..2 {
            assert!(
                (w[i] - want[i]).abs() < 1e-12 * want[i],
                "ω{} {} vs {}",
                i + 1,
                w[i],
                want[i]
            );
        }
        let (a0, a1) = b.rayleigh_coefficients();
        for (i, z) in [0.02, 0.05].iter().enumerate() {
            let zi = a0 / (2.0 * w[i]) + a1 * w[i] / 2.0;
            assert!((zi - z).abs() < 1e-12, "mode {} ζ {zi}", i + 1);
        }
        // Uniform 5-storey building: ω_r = 2√(k/m)·sin((2r − 1)π/(2(2n + 1))) (fixed
        // base, free top; (E) rel 1e-12) — checks the Jacobi eigen-solver.
        let b5 = ShearBuilding::uniform(5, m, k, 0.02, 3);
        let w5 = b5.modal_omegas_radps();
        for (r, wr) in w5.iter().enumerate() {
            let arg = f64::from(2 * r as u32 + 1) * core::f64::consts::PI / 22.0;
            let want = 2.0 * (k / m).sqrt() * fmath::sin(arg);
            assert!(
                (wr - want).abs() < 1e-12 * want,
                "mode {}: {wr} vs {want}",
                r + 1
            );
        }
        // SDOF: c = 2ζ√(k·m).
        let sd = ShearBuilding::sdof(1.0e5, 1.0, 0.03);
        let (a0, a1) = sd.rayleigh_coefficients();
        let c = a0 * 1.0e5 + a1 * sd.storey_k_n_per_m[0];
        assert!((c - 2.0 * 0.03 * (sd.storey_k_n_per_m[0] * 1.0e5).sqrt()).abs() < 1e-6 * c);
        assert!(ShearBuilding::default().modal_omegas_radps().is_empty());
    }

    #[test]
    fn sdof_resonance_matches_dynamic_amplification() {
        // SDOF T = 1 s, ζ = 5 %, harmonic ground motion A·sin(ωt) at resonance: the
        // steady relative amplitude is A/(2ζω²) ((I) 1 %, PHYSICS §21), with a light
        // frictionless device riding along (STO, no coupling force).
        let b = ShearBuilding::sdof(1.0e5, 1.0, 0.05);
        let p = device(1.0);
        let mut s = Structure::new(
            &p,
            &b,
            &ModelConfig {
                tier: Tier::Structure,
                ..ModelConfig::default()
            },
        )
        .expect("valid");
        let w = 2.0 * core::f64::consts::PI;
        let amp = 0.5;
        let hist = run_ground(&mut s, 30.0, |t| amp * fmath::sin(w * t), |_| 0.0);
        let peak = hist
            .iter()
            .filter(|(t, _)| *t > 25.0)
            .fold(0.0f64, |m, (_, x)| m.max(x.abs()));
        let want = amp / (2.0 * 0.05 * w * w);
        assert!(
            (peak - want).abs() < 0.01 * want,
            "peak {peak} vs {want} m ((I) 1 %)"
        );
        assert!(s.truth().host_a_mps2.is_finite() && s.truth().state.levitated);
    }

    #[test]
    fn coupled_energy_balance() {
        // Undamped 2-storey building released from a displaced shape with the device
        // coupled only through distributed damping b_dist: building + device mechanical
        // energy plus the dissipated ledger stays constant ((I) rel 1e-8; PHYSICS §21, §9).
        // A sign error in the device–storey reaction breaks this balance.
        let b = ShearBuilding::uniform(2, 1.0e4, 4.0e6, 0.0, 2);
        let mut p = device(500.0);
        p.design.damping.b_dist_n_s_per_m = 2.0e3;
        let mut s = Structure::new(&p, &b, &ModelConfig::default()).expect("valid");
        s.y[S_U] = 0.01;
        s.y[S_U + 1] = 0.02;
        s.truth.mass_kg = 500.0;
        let e0 = s.mechanical_energy_j();
        let u = PlantInputs::default(); // STO: no LSM force
        let h = Tier::Structure.nominal_h_s();
        for k in 0..5000u32 {
            s.step(f64::from(k) * h, h, &u, &[[0.0; 3]; 3]);
        }
        let e1 = s.mechanical_energy_j() + s.truth().e_diss.dist_j;
        assert!(s.truth().e_diss.dist_j > 0.01 * e0, "the damper worked");
        assert!(
            (e1 - e0).abs() < 1e-8 * e0,
            "E0 {e0} vs E + dissipated {e1}"
        );
    }

    #[test]
    fn lsm_tmd_emulation_matches_the_two_dof_frequency_response() {
        // SEI-021 plumbing (research tier): the maglev mass (μ = 2 %) driven by the LSM
        // as a Den Hartog TMD (F = −k_t·x − c_t·ẋ, f = 1/(1 + μ), ζ = √(3μ/(8(1 + μ)³)))
        // on an SDOF (T 1 s, ζ 2 %) under resonant harmonic ground motion. The simulated
        // steady storey amplitude matches the closed-form 2-DOF frequency response with
        // and without the TMD within 2 % ((I), PHYSICS §21; the zero-order hold over the step is
        // compensated by a half-step prediction), and the TMD cuts it to ≈ 26 %.
        use nalgebra::Complex;
        let (m_s, t_s, z_s) = (1.0e5, 1.0, 0.02);
        let b = ShearBuilding::sdof(m_s, t_s, z_s);
        let mu = 0.02;
        let md = mu * m_s;
        let w_s = 2.0 * core::f64::consts::PI / t_s;
        let (f_opt, z_opt) = fabrail_params::derive::den_hartog(mu);
        let w_t = f_opt * w_s;
        let (k_t, c_t) = (md * w_t * w_t, 2.0 * z_opt * md * w_t);
        let amp = 0.2;
        // Closed form: [Z11 Z12; Z21 Z22]·[U; X] = −[m_s; m_d]·A at s = iω_s.
        let (k_s, c_s) = (
            b.storey_k_n_per_m[0],
            2.0 * z_s * (b.storey_k_n_per_m[0] * m_s).sqrt(),
        );
        let s = Complex::new(0.0, w_s);
        let z11 = s * s * m_s + s * c_s + k_s;
        let z12 = -(s * c_t + k_t);
        let z21 = s * s * md;
        let z22 = s * s * md + s * c_t + k_t;
        let det = z11 * z22 - z12 * z21;
        let u_tmd = ((z22 * (-m_s) - z12 * (-md)) / det).norm() * amp;
        let u_bare = (Complex::new(-m_s, 0.0) / z11).norm() * amp;
        let p = device(md);
        let a_g = |t: f64| amp * fmath::sin(w_s * t);
        let cfg = ModelConfig {
            tier: Tier::Structure,
            ..ModelConfig::default()
        };
        let h = Tier::Structure.nominal_h_s();
        let mut bare = Structure::new(&p, &b, &cfg).expect("valid");
        let hb = run_ground(&mut bare, 60.0, a_g, |_| 0.0);
        let mut tmd = Structure::new(&p, &b, &cfg).expect("valid");
        let ht = run_ground(&mut tmd, 60.0, a_g, |st| {
            let tr = st.truth();
            let x = tr.x_m + 0.5 * h * tr.v_mps;
            let v = tr.v_mps + 0.5 * h * tr.a_mps2;
            (-k_t * x - c_t * v) / 100.0
        });
        let peak = |hh: &[(f64, f64)]| {
            hh.iter()
                .filter(|(t, _)| *t > 50.0)
                .fold(0.0f64, |m, (_, x)| m.max(x.abs()))
        };
        let (pb, pt) = (peak(&hb), peak(&ht));
        assert!(
            (pb - u_bare).abs() < 0.02 * u_bare,
            "bare {pb} vs {u_bare} m"
        );
        assert!((pt - u_tmd).abs() < 0.02 * u_tmd, "TMD {pt} vs {u_tmd} m");
        assert!(pt < 0.3 * pb, "TMD {pt} vs bare {pb}");
        let l = &tmd.truth().e_diss;
        assert!(
            l.elec_in_j.is_finite() && l.lsm_copper_j == 0.0,
            "R_phase 0 in this device"
        );
    }
}
