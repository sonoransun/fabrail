//! Host structures: SDOF, 2-DOF TMD/HMD, modal shear tower; Davenport wind loading (FAB-EXO-601..609). Owner: X4 (API frozen by W0a).
//!
//! EXOTIC §X6, §X8 (f64 truth, research tier; PHYSICS §21 for the TMD closed forms,
//! §24 for RK4). The 2-DOF structure + damper model:
//!
//! M·ẍs + 2ζs·M·ωs·ẋs + M·ωs²·xs − c_d(ẋd − ẋs) − k_d(xd − xs) = −M·a_g − u + f_ext
//! m_d·ẍd + c_d(ẋd − ẋs) + k_d(xd − xs) = −m_d·a_g + u
//!
//! with u the LSM trim force (HMD) and f_ext the wind force. Beyond the stroke limit
//! s_max an elastic end-stop buffer acts between damper and structure (stiffness
//! 50·k_d, ζ 0.1 contact mode; design choice) so the truth model never exceeds the
//! stroke silently. The modal tower reduces N ≤ 150 lumped storeys with Rayleigh damping
//! to 3–5 modes (φ, ω, ζ, Γ) at construction; ω_s drift of ±10 % (cracking, occupancy)
//! is [`ModalTower::detuned`] / [`Primary::detuned`]. Wind forcing follows the Davenport
//! spectrum. Mass dampers are mainly wind devices: their seismic benefit is at most about
//! 0.7 % added damping at design level, and no stroke-limited controller beats
//! ζ_add ≤ (μ/2)(S/X).
//!
//! The plan names nalgebra `SymmetricEigen` for the modal reduction; nalgebra is not in
//! this crate's frozen manifest (it is allowed in fabrail-plant only), so the reduction
//! uses an in-crate cyclic Jacobi eigensolver on M^(−½)·K·M^(−½) (the same symmetric
//! eigenproblem; deterministic, O(n³) per sweep, run once at construction).
//!
//! Activation (EXOTIC §X8): a damper on a Coulomb-type bearing stays stuck until the
//! floor acceleration exceeds a = μ·g ([`CoulombSdof`]). Maglev removes the friction
//! threshold only on low-coercivity or homopolar rails: laminations cut eddy drag, not
//! hysteresis drag ([`RailCondition`]).

use fabrail_exo::tuning::DamperDesign;
use fabrail_params::derive::G0_MPS2;
use fabrail_sim::rng::Xoshiro256pp;
use fabrail_types::fmath;

/// End-stop buffer stiffness factor: k_stop = 50·k_d, i.e. a contact frequency √50 ≈ 7.1×
/// the damper frequency (design choice; EXOTIC §X6).
pub const STOP_K_FACTOR: f64 = 50.0;

/// End-stop buffer contact-mode damping ratio (design choice).
pub const STOP_ZETA: f64 = 0.1;

/// Buffer contact frequency when the damper has no spring (active mass damper), rad/s.
pub const STOP_OMEGA_FALLBACK_RAD_S: f64 = 2.0 * core::f64::consts::PI;

/// Davenport integral length, m (Davenport 1961: x = 1200·f/U10).
pub const DAVENPORT_L_M: f64 = 1200.0;

/// Number of harmonic components of the wind synthesis (Shinozuka sum).
pub const WIND_BINS: usize = 128;

/// Lower frequency of the wind synthesis band, Hz.
pub const WIND_F_MIN_HZ: f64 = 1.0e-3;

/// Upper frequency of the wind synthesis band, Hz.
pub const WIND_F_MAX_HZ: f64 = 2.0;

/// Coulomb coefficient of a PTFE sliding bearing (exotic design §6.1 activation table).
pub const BEARING_MU_PTFE: f64 = 0.05;

/// Coulomb coefficient of rolling bearings (exotic design §6.1; roller μ 0.003, plan §2b
/// item 19).
pub const BEARING_MU_ROLLING: f64 = 0.003;

/// Structure (primary) parameters of the 2-DOF model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Primary {
    /// Modal mass M, kg.
    pub m_kg: f64,
    /// Natural angular frequency ω_s, rad/s.
    pub omega_rad_s: f64,
    /// Damping ratio ζ_s.
    pub zeta: f64,
}

impl Primary {
    /// The same structure with ω_s scaled by `scale` (±10 % drift for cracking and
    /// occupancy; EXOTIC §X6).
    pub fn detuned(&self, scale: f64) -> Primary {
        Primary {
            omega_rad_s: self.omega_rad_s * scale,
            ..*self
        }
    }

    /// Stiffness K = M·ω_s², N/m.
    pub fn k_n_per_m(&self) -> f64 {
        self.m_kg * self.omega_rad_s * self.omega_rad_s
    }
}

/// Damper (secondary) parameters of the 2-DOF model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Damper {
    /// Damper mass m_d, kg.
    pub m_kg: f64,
    /// Stiffness k_d, N/m.
    pub k_n_per_m: f64,
    /// Damping c_d (passive eddy), N·s/m.
    pub c_n_s_per_m: f64,
    /// Stroke limit s_max, m (≤ 0: unlimited).
    pub s_max_m: f64,
}

impl Damper {
    /// A damper from a tuning design (`fabrail_exo::tuning::damper_design`) with stroke
    /// limit `s_max_m`.
    pub fn from_design(d: &DamperDesign, s_max_m: f64) -> Damper {
        Damper {
            m_kg: d.m_d_kg,
            k_n_per_m: d.k_d_n_per_m,
            c_n_s_per_m: d.c_d_n_s_per_m,
            s_max_m,
        }
    }

    /// End-stop buffer (stiffness, damping), N/m and N·s/m.
    pub fn stop_coefficients(&self) -> (f64, f64) {
        let w = if self.m_kg > 0.0 && self.k_n_per_m > 0.0 {
            fmath::sqrt(STOP_K_FACTOR * self.k_n_per_m / self.m_kg)
        } else {
            STOP_OMEGA_FALLBACK_RAD_S
        };
        let k = self.m_kg * w * w;
        (k, 2.0 * STOP_ZETA * self.m_kg * w)
    }

    /// Buffer force on the damper at stroke `x_r` and stroke rate `v_r`, N (restoring;
    /// zero inside ±s_max; the damping acts only while in contact and only compressing).
    pub fn stop_force_n(&self, x_r: f64, v_r: f64) -> f64 {
        if self.s_max_m <= 0.0 {
            return 0.0;
        }
        let pen = fmath::abs(x_r) - self.s_max_m;
        if pen <= 0.0 {
            return 0.0;
        }
        let (k, c) = self.stop_coefficients();
        let dir = fmath::copysign(1.0, x_r);
        // Damping only while compressing (v_r in the direction of x_r): no tensile force.
        let v_in = if v_r * dir > 0.0 { v_r } else { 0.0 };
        let f = -dir * k * pen - c * v_in;
        // The buffer never pulls.
        if f * dir > 0.0 { 0.0 } else { f }
    }
}

/// State [x_s, ẋ_s, x_d, ẋ_d] (absolute displacements relative to the ground).
pub type TwoDofState = [f64; 4];

/// Structure plus mass damper.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TwoDof {
    /// Structure.
    pub primary: Primary,
    /// Damper.
    pub damper: Damper,
    /// State.
    pub state: TwoDofState,
}

impl TwoDof {
    /// A structure at rest.
    pub fn new(primary: Primary, damper: Damper) -> TwoDof {
        TwoDof {
            primary,
            damper,
            state: [0.0; 4],
        }
    }

    /// Force of the spring, damper and buffer on the structure at state `y`, N (without u).
    fn link_force(&self, y: &TwoDofState) -> (f64, f64) {
        let d = &self.damper;
        let xr = y[2] - y[0];
        let vr = y[3] - y[1];
        let f_link = d.c_n_s_per_m * vr + d.k_n_per_m * xr;
        (f_link, d.stop_force_n(xr, vr))
    }

    /// State derivative under ground acceleration `a_g`, LSM force `u` (on the damper,
    /// reaction on the structure) and external force `f_ext` on the structure.
    pub fn derivative(
        &self,
        y: &TwoDofState,
        a_g_mps2: f64,
        u_n: f64,
        f_ext_n: f64,
    ) -> TwoDofState {
        let p = &self.primary;
        let d = &self.damper;
        let (f_link, f_stop) = self.link_force(y);
        let a_s = if p.m_kg > 0.0 {
            (-2.0 * p.zeta * p.m_kg * p.omega_rad_s * y[1] - p.k_n_per_m() * y[0] + f_link
                - f_stop
                - u_n
                + f_ext_n)
                / p.m_kg
                - a_g_mps2
        } else {
            0.0
        };
        let a_d = if d.m_kg > 0.0 {
            (-f_link + f_stop + u_n) / d.m_kg - a_g_mps2
        } else {
            a_s
        };
        [y[1], a_s, y[3], a_d]
    }

    /// Advances by `dt_s` under ground acceleration `a_g_mps2`, LSM force `u_n` and
    /// external (wind) force `f_ext_n` on the structure (RK4, inputs held over the step;
    /// PHYSICS §24).
    pub fn step(&mut self, a_g_mps2: f64, u_n: f64, f_ext_n: f64, dt_s: f64) {
        if dt_s <= 0.0 {
            return;
        }
        let y = self.state;
        let add = |a: &TwoDofState, b: &TwoDofState, h: f64| -> TwoDofState {
            [
                a[0] + h * b[0],
                a[1] + h * b[1],
                a[2] + h * b[2],
                a[3] + h * b[3],
            ]
        };
        let k1 = self.derivative(&y, a_g_mps2, u_n, f_ext_n);
        let k2 = self.derivative(&add(&y, &k1, 0.5 * dt_s), a_g_mps2, u_n, f_ext_n);
        let k3 = self.derivative(&add(&y, &k2, 0.5 * dt_s), a_g_mps2, u_n, f_ext_n);
        let k4 = self.derivative(&add(&y, &k3, dt_s), a_g_mps2, u_n, f_ext_n);
        for i in 0..4 {
            self.state[i] = y[i] + dt_s / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }

    /// Stroke x_d − x_s, m.
    pub fn stroke_m(&self) -> f64 {
        self.state[2] - self.state[0]
    }

    /// Total force the damper assembly (spring, eddy damping, buffer, LSM reaction)
    /// exerts on the structure at the current state, N.
    pub fn damper_force_on_structure_n(&self, u_n: f64) -> f64 {
        let (f_link, f_stop) = self.link_force(&self.state);
        f_link - f_stop - u_n
    }

    /// Absolute structure acceleration ẍs + a_g at the current state, m/s².
    pub fn structure_abs_accel_mps2(&self, a_g_mps2: f64, u_n: f64, f_ext_n: f64) -> f64 {
        self.derivative(&self.state, a_g_mps2, u_n, f_ext_n)[1] + a_g_mps2
    }

    /// Mechanical energy relative to the ground (kinetic + springs + buffer), J.
    pub fn mechanical_energy_j(&self) -> f64 {
        let p = &self.primary;
        let d = &self.damper;
        let y = &self.state;
        let xr = y[2] - y[0];
        let pen = if d.s_max_m > 0.0 {
            (fmath::abs(xr) - d.s_max_m).max(0.0)
        } else {
            0.0
        };
        let (k_stop, _) = d.stop_coefficients();
        0.5 * p.m_kg * y[1] * y[1]
            + 0.5 * d.m_kg * y[3] * y[3]
            + 0.5 * p.k_n_per_m() * y[0] * y[0]
            + 0.5 * d.k_n_per_m * xr * xr
            + 0.5 * k_stop * pen * pen
    }
}

// ---- Modal tower -------------------------------------------------------------------------

/// One retained tower mode.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Mode {
    /// Angular frequency, rad/s.
    pub omega_rad_s: f64,
    /// Damping ratio.
    pub zeta: f64,
    /// Mode-shape value at the damper floor (unit-normalised at the roof).
    pub phi_damper: f64,
    /// Modal participation factor Γ.
    pub gamma: f64,
    /// Modal mass, kg.
    pub m_modal_kg: f64,
}

/// Modal shear tower (3–5 retained modes).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ModalTower {
    /// Retained modes.
    pub modes: Vec<Mode>,
    /// Modal coordinates and rates [q_1, q̇_1, …].
    pub state: Vec<f64>,
    /// Mode shapes of the retained modes over every floor (bottom first), unit at the
    /// roof.
    pub shapes: Vec<Vec<f64>>,
    /// Floor index of the damper.
    pub damper_floor: usize,
}

/// Symmetric eigen-decomposition by cyclic Jacobi rotations of the row-major n×n matrix
/// `a` (destroyed). Returns (eigenvalues, eigenvectors as columns, row-major); unsorted.
fn jacobi_eigen(a: &mut [f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut v = vec![0.0; n * n];
    for i in 0..n {
        v[i * n + i] = 1.0;
    }
    let scale: f64 = a.iter().map(|x| x * x).sum::<f64>().max(f64::MIN_POSITIVE);
    for _sweep in 0..100 {
        let mut off = 0.0;
        for p in 0..n {
            for q in (p + 1)..n {
                off += a[p * n + q] * a[p * n + q];
            }
        }
        if off <= 1.0e-30 * scale {
            break;
        }
        for p in 0..n {
            for q in (p + 1)..n {
                let apq = a[p * n + q];
                if fmath::abs(apq) <= 1.0e-300 {
                    continue;
                }
                let app = a[p * n + p];
                let aqq = a[q * n + q];
                let theta = (aqq - app) / (2.0 * apq);
                let t = fmath::copysign(1.0, theta)
                    / (fmath::abs(theta) + fmath::sqrt(theta * theta + 1.0));
                let c = 1.0 / fmath::sqrt(t * t + 1.0);
                let s = t * c;
                for k in 0..n {
                    let akp = a[k * n + p];
                    let akq = a[k * n + q];
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let apk = a[p * n + k];
                    let aqk = a[q * n + k];
                    a[p * n + k] = c * apk - s * aqk;
                    a[q * n + k] = s * apk + c * aqk;
                }
                for k in 0..n {
                    let vkp = v[k * n + p];
                    let vkq = v[k * n + q];
                    v[k * n + p] = c * vkp - s * vkq;
                    v[k * n + q] = s * vkp + c * vkq;
                }
            }
        }
    }
    let evals = (0..n).map(|i| a[i * n + i]).collect();
    (evals, v)
}

impl ModalTower {
    /// Builds a tower from storey masses and stiffnesses (Rayleigh ζ at modes 1 and 2),
    /// retaining `n_modes`, with the damper on the roof. Invalid input (empty, mismatched
    /// or non-positive) gives a tower with no modes.
    pub fn from_storeys(
        masses_kg: &[f64],
        k_n_per_m: &[f64],
        rayleigh_zeta: [f64; 2],
        n_modes: usize,
    ) -> ModalTower {
        let top = masses_kg.len().saturating_sub(1);
        ModalTower::from_storeys_at(masses_kg, k_n_per_m, rayleigh_zeta, n_modes, top)
    }

    /// As [`ModalTower::from_storeys`] with the damper on floor `damper_floor` (0 = first
    /// suspended floor). Shear building: storey i connects floor i to floor i − 1 (the
    /// ground for i = 0); K is tridiagonal, M diagonal; the modes come from the
    /// symmetric problem M^(−½)·K·M^(−½) (EXOTIC §X6).
    pub fn from_storeys_at(
        masses_kg: &[f64],
        k_n_per_m: &[f64],
        rayleigh_zeta: [f64; 2],
        n_modes: usize,
        damper_floor: usize,
    ) -> ModalTower {
        let n = masses_kg.len();
        let valid = n > 0
            && k_n_per_m.len() == n
            && masses_kg.iter().all(|m| *m > 0.0 && m.is_finite())
            && k_n_per_m.iter().all(|k| *k > 0.0 && k.is_finite())
            && damper_floor < n;
        if !valid {
            return ModalTower::default();
        }
        let inv_sqrt_m: Vec<f64> = masses_kg.iter().map(|m| 1.0 / fmath::sqrt(*m)).collect();
        let mut a = vec![0.0; n * n];
        for i in 0..n {
            let k_up = if i + 1 < n { k_n_per_m[i + 1] } else { 0.0 };
            a[i * n + i] = (k_n_per_m[i] + k_up) * inv_sqrt_m[i] * inv_sqrt_m[i];
            if i + 1 < n {
                let off = -k_n_per_m[i + 1] * inv_sqrt_m[i] * inv_sqrt_m[i + 1];
                a[i * n + i + 1] = off;
                a[(i + 1) * n + i] = off;
            }
        }
        let (evals, evecs) = jacobi_eigen(&mut a, n);
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&i, &j| evals[i].total_cmp(&evals[j]));
        let n_keep = n_modes.clamp(1, n);
        // Mode shapes φ = M^(−½)·v, unit at the roof.
        let mut omegas = Vec::with_capacity(n_keep);
        let mut shapes = Vec::with_capacity(n_keep);
        for &j in order.iter().take(n_keep) {
            let mut phi: Vec<f64> = (0..n).map(|i| evecs[i * n + j] * inv_sqrt_m[i]).collect();
            let roof = phi[n - 1];
            let norm = if fmath::abs(roof) > 1.0e-300 {
                roof
            } else {
                phi.iter()
                    .fold(0.0_f64, |acc, x| if x.abs() > acc.abs() { *x } else { acc })
            };
            if norm != 0.0 {
                for x in phi.iter_mut() {
                    *x /= norm;
                }
            }
            omegas.push(fmath::sqrt(evals[j].max(0.0)));
            shapes.push(phi);
        }
        // Rayleigh coefficients from (ω1, ζ1), (ω2, ζ2); one mode: mass-proportional.
        let (alpha, beta) = if omegas.len() >= 2 || n >= 2 {
            let w1 = omegas[0];
            let w2 = if omegas.len() >= 2 {
                omegas[1]
            } else {
                fmath::sqrt(evals[order[1]].max(0.0))
            };
            let den = w2 * w2 - w1 * w1;
            if den > 0.0 {
                (
                    2.0 * w1 * w2 * (rayleigh_zeta[0] * w2 - rayleigh_zeta[1] * w1) / den,
                    2.0 * (rayleigh_zeta[1] * w2 - rayleigh_zeta[0] * w1) / den,
                )
            } else {
                (2.0 * rayleigh_zeta[0] * w1, 0.0)
            }
        } else {
            (2.0 * rayleigh_zeta[0] * omegas[0], 0.0)
        };
        let modes = omegas
            .iter()
            .zip(shapes.iter())
            .map(|(&w, phi)| {
                let m_modal: f64 = phi.iter().zip(masses_kg).map(|(p, m)| m * p * p).sum();
                let l: f64 = phi.iter().zip(masses_kg).map(|(p, m)| m * p).sum();
                Mode {
                    omega_rad_s: w,
                    zeta: if w > 0.0 {
                        alpha / (2.0 * w) + beta * w / 2.0
                    } else {
                        0.0
                    },
                    phi_damper: phi[damper_floor],
                    gamma: if m_modal > 0.0 { l / m_modal } else { 0.0 },
                    m_modal_kg: m_modal,
                }
            })
            .collect::<Vec<Mode>>();
        ModalTower {
            state: vec![0.0; 2 * modes.len()],
            modes,
            shapes,
            damper_floor,
        }
    }

    /// The same tower with every ω scaled by `scale` (±10 % ω_s drift; EXOTIC §X6).
    pub fn detuned(&self, scale: f64) -> ModalTower {
        let mut t = self.clone();
        for m in t.modes.iter_mut() {
            m.omega_rad_s *= scale;
        }
        t
    }

    /// Modal accelerations at modal state `q` (q̈_i + 2ζ_iω_i q̇_i + ω_i² q_i =
    /// −Γ_i·a_g + φ_i,d·f/m_i).
    fn derivative(&self, q: &[f64], a_g_mps2: f64, f_damper_n: f64, out: &mut [f64]) {
        for (i, m) in self.modes.iter().enumerate() {
            let x = q[2 * i];
            let v = q[2 * i + 1];
            let force = if m.m_modal_kg > 0.0 {
                m.phi_damper * f_damper_n / m.m_modal_kg
            } else {
                0.0
            };
            out[2 * i] = v;
            out[2 * i + 1] = -2.0 * m.zeta * m.omega_rad_s * v
                - m.omega_rad_s * m.omega_rad_s * x
                - m.gamma * a_g_mps2
                + force;
        }
    }

    /// Advances by `dt_s` under ground acceleration and a damper force at the damper floor
    /// (RK4, inputs held over the step; PHYSICS §24).
    pub fn step(&mut self, a_g_mps2: f64, f_damper_n: f64, dt_s: f64) {
        let n = self.state.len();
        if dt_s <= 0.0 || n == 0 {
            return;
        }
        let y = self.state.clone();
        let mut k1 = vec![0.0; n];
        let mut k2 = vec![0.0; n];
        let mut k3 = vec![0.0; n];
        let mut k4 = vec![0.0; n];
        let mut tmp = vec![0.0; n];
        self.derivative(&y, a_g_mps2, f_damper_n, &mut k1);
        for i in 0..n {
            tmp[i] = y[i] + 0.5 * dt_s * k1[i];
        }
        self.derivative(&tmp, a_g_mps2, f_damper_n, &mut k2);
        for i in 0..n {
            tmp[i] = y[i] + 0.5 * dt_s * k2[i];
        }
        self.derivative(&tmp, a_g_mps2, f_damper_n, &mut k3);
        for i in 0..n {
            tmp[i] = y[i] + dt_s * k3[i];
        }
        self.derivative(&tmp, a_g_mps2, f_damper_n, &mut k4);
        for i in 0..n {
            self.state[i] = y[i] + dt_s / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }

    /// Floor displacement relative to the ground, m (0 for an unknown floor).
    pub fn floor_disp_m(&self, floor: usize) -> f64 {
        self.shapes
            .iter()
            .enumerate()
            .map(|(i, phi)| phi.get(floor).copied().unwrap_or(0.0) * self.state[2 * i])
            .sum()
    }

    /// Damper-floor displacement relative to the ground, m.
    pub fn damper_disp_m(&self) -> f64 {
        self.modes
            .iter()
            .enumerate()
            .map(|(i, m)| m.phi_damper * self.state[2 * i])
            .sum()
    }

    /// Damper-floor velocity relative to the ground, m/s.
    pub fn damper_vel_mps(&self) -> f64 {
        self.modes
            .iter()
            .enumerate()
            .map(|(i, m)| m.phi_damper * self.state[2 * i + 1])
            .sum()
    }

    /// Damper-floor absolute acceleration at the current state, m/s².
    pub fn damper_abs_accel_mps2(&self, a_g_mps2: f64, f_damper_n: f64) -> f64 {
        let mut d = vec![0.0; self.state.len()];
        self.derivative(&self.state, a_g_mps2, f_damper_n, &mut d);
        let rel: f64 = self
            .modes
            .iter()
            .enumerate()
            .map(|(i, m)| m.phi_damper * d[2 * i + 1])
            .sum();
        rel + a_g_mps2
    }
}

// ---- Davenport wind (EXOTIC §X15) --------------------------------------------------------

/// Davenport along-wind force spectrum parameters.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DavenportWind {
    /// Mean wind speed at 10 m, m/s.
    pub u10_mps: f64,
    /// Surface drag coefficient κ.
    pub kappa: f64,
    /// Exposed area, m².
    pub area_m2: f64,
    /// Drag coefficient.
    pub c_d: f64,
    /// Air density, kg/m³.
    pub rho_kg_per_m3: f64,
}

/// One harmonic of the wind synthesis: (frequency Hz, amplitude m/s, phase rad).
type WindComponent = (f64, f64, f64);

impl DavenportWind {
    /// Davenport gust spectrum S_u(f) = 4·κ·U10²·x²/(f·(1 + x²)^(4/3)), x = 1200·f/U10,
    /// (m/s)²/Hz (Davenport 1961; EXOTIC §X15).
    pub fn psd_u(&self, f_hz: f64) -> f64 {
        let u = self.u10_mps;
        if !(f_hz > 0.0 && u > 0.0) {
            return 0.0;
        }
        let x = DAVENPORT_L_M * f_hz / u;
        4.0 * self.kappa * u * u * x * x / (f_hz * fmath::pow(1.0 + x * x, 4.0 / 3.0))
    }

    /// Gust standard deviation of the full spectrum √(6κ)·U10, m/s.
    pub fn sigma_u_mps(&self) -> f64 {
        fmath::sqrt(6.0 * self.kappa.max(0.0)) * self.u10_mps
    }

    /// Mean drag force ½·ρ·C_d·A·U10², N.
    pub fn mean_force_n(&self) -> f64 {
        0.5 * self.rho_kg_per_m3 * self.c_d * self.area_m2 * self.u10_mps * self.u10_mps
    }

    /// Quasi-steady force gain ∂F/∂u = ρ·C_d·A·U10, N·s/m (aerodynamic admittance 1,
    /// conservative for large faces) \[UNCERTAIN\].
    pub fn force_gain_n_s_per_m(&self) -> f64 {
        self.rho_kg_per_m3 * self.c_d * self.area_m2 * self.u10_mps
    }

    /// The harmonic components for `seed`: log-spaced bins on [1 mHz, 2 Hz], amplitude
    /// √(2·S_u(f_k)·Δf_k), phases uniform from Xoshiro256++ (Shinozuka sum).
    fn components(&self, seed: u64) -> Vec<WindComponent> {
        let mut rng = Xoshiro256pp::from_seed(seed);
        let ratio = fmath::pow(WIND_F_MAX_HZ / WIND_F_MIN_HZ, 1.0 / WIND_BINS as f64);
        let mut lo = WIND_F_MIN_HZ;
        let mut out = Vec::with_capacity(WIND_BINS);
        for _ in 0..WIND_BINS {
            let hi = lo * ratio;
            let f = fmath::sqrt(lo * hi);
            let amp = fmath::sqrt(2.0 * self.psd_u(f) * (hi - lo));
            let phase = 2.0 * core::f64::consts::PI * rng.next_f64();
            out.push((f, amp, phase));
            lo = hi;
        }
        out
    }

    /// Band variance of the synthesis Σ S_u(f_k)·Δf_k, (m/s)².
    pub fn band_variance(&self) -> f64 {
        self.components(0).iter().map(|c| 0.5 * c.1 * c.1).sum()
    }

    /// Gust speed u′ at `t_s` for seed `seed`, m/s (spot evaluation; use
    /// [`DavenportWind::series`] for runs).
    pub fn u_prime_mps(&self, t_s: f64, seed: u64) -> f64 {
        self.components(seed)
            .iter()
            .map(|(f, a, ph)| a * fmath::cos(2.0 * core::f64::consts::PI * f * t_s + ph))
            .sum()
    }

    /// Fluctuating wind force at time `t_s` for seed `seed`, N: ρ·C_d·A·U10·u′(t)
    /// (quasi-steady; the mean is [`DavenportWind::mean_force_n`]).
    pub fn force_n(&self, t_s: f64, seed: u64) -> f64 {
        self.force_gain_n_s_per_m() * self.u_prime_mps(t_s, seed)
    }

    /// Fluctuating force on a uniform grid of `n` samples at `fs_hz` (the same synthesis as
    /// [`DavenportWind::force_n`], by recursive phasor rotation re-synchronised every
    /// 4096 samples), N.
    pub fn series(&self, fs_hz: f64, n: usize, seed: u64) -> Vec<f64> {
        let mut out = vec![0.0; n];
        if !(fs_hz > 0.0 && fs_hz.is_finite()) {
            return out;
        }
        let gain = self.force_gain_n_s_per_m();
        let dt = 1.0 / fs_hz;
        for (f, a, ph) in self.components(seed) {
            let w = 2.0 * core::f64::consts::PI * f;
            let (cr, sr) = (fmath::cos(w * dt), fmath::sin(w * dt));
            let (mut c, mut s) = (0.0, 0.0);
            for (k, o) in out.iter_mut().enumerate() {
                if k % 4096 == 0 {
                    let arg = w * (k as f64) * dt + ph;
                    c = fmath::cos(arg);
                    s = fmath::sin(arg);
                }
                *o += gain * a * c;
                let c2 = c * cr - s * sr;
                s = s * cr + c * sr;
                c = c2;
            }
        }
        out
    }
}

// ---- Activation threshold (EXOTIC §X8) ---------------------------------------------------

/// Rail condition of a maglev bearing, which sets its residual (hysteresis) drag.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum RailCondition {
    /// Solid-steel rail under hybrid EMS: hysteresis drag μ_eq ≈ 1e-3.
    #[default]
    SolidSteel,
    /// Laminated rail: laminations cut eddy drag, not hysteresis drag (μ_eq ≈ 1e-3).
    Laminated,
    /// Low-coercivity rail: μ_eq ≈ 1e-4.
    LowCoercivity,
    /// Homopolar layout (no flux reversal along travel): μ_eq ≈ 1e-4.
    Homopolar,
    /// HTS over a PM guideway: μ_eq ≤ 1e-5.
    HtsPmGuideway,
}

impl RailCondition {
    /// All conditions, in table order.
    pub const ALL: [RailCondition; 5] = [
        RailCondition::SolidSteel,
        RailCondition::Laminated,
        RailCondition::LowCoercivity,
        RailCondition::Homopolar,
        RailCondition::HtsPmGuideway,
    ];

    /// Equivalent Coulomb coefficient of the residual drag (exotic design §6.1 table;
    /// magnitudes \[UNCERTAIN\]).
    pub fn mu_eq(self) -> f64 {
        match self {
            RailCondition::SolidSteel | RailCondition::Laminated => 1.0e-3,
            RailCondition::LowCoercivity | RailCondition::Homopolar => 1.0e-4,
            RailCondition::HtsPmGuideway => 1.0e-5,
        }
    }

    /// Name for reports.
    pub fn name(self) -> &'static str {
        match self {
            RailCondition::SolidSteel => "solid steel (hysteresis drag)",
            RailCondition::Laminated => "laminated (cuts eddy drag, not hysteresis drag)",
            RailCondition::LowCoercivity => "low-coercivity rail",
            RailCondition::Homopolar => "homopolar layout",
            RailCondition::HtsPmGuideway => "HTS over PM guideway",
        }
    }
}

/// Activation threshold a_act = μ·g of a Coulomb-type bearing, m/s² (EXOTIC §X8).
pub fn activation_threshold_mps2(mu: f64) -> f64 {
    mu.max(0.0) * G0_MPS2
}

/// The 1/10 rule: a damper meant to act at `target_mps2` must activate at ≤ target/10.
pub fn passes_one_tenth_rule(mu: f64, target_mps2: f64) -> bool {
    activation_threshold_mps2(mu) <= 0.1 * target_mps2
}

/// A mass on a Coulomb bearing (plus an optional spring and damper to the floor) under
/// floor acceleration: exact stick, Coulomb slip, stick detection at velocity reversal
/// (EXOTIC §X8). Coordinates are relative to the floor.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CoulombSdof {
    /// Mass, kg.
    pub m_kg: f64,
    /// Spring to the floor, N/m.
    pub k_n_per_m: f64,
    /// Viscous damper to the floor, N·s/m.
    pub c_n_s_per_m: f64,
    /// Static coefficient.
    pub mu_s: f64,
    /// Kinetic coefficient.
    pub mu_k: f64,
    /// Relative displacement, m.
    pub x_m: f64,
    /// Relative velocity, m/s.
    pub v_mps: f64,
    /// Stuck to the floor.
    pub stuck: bool,
}

impl CoulombSdof {
    /// A mass at rest and stuck, with μ_s = μ_k = `mu`.
    pub fn new(m_kg: f64, k_n_per_m: f64, c_n_s_per_m: f64, mu: f64) -> CoulombSdof {
        CoulombSdof {
            m_kg,
            k_n_per_m,
            c_n_s_per_m,
            mu_s: mu,
            mu_k: mu,
            x_m: 0.0,
            v_mps: 0.0,
            stuck: true,
        }
    }

    /// Non-friction force on the mass in the floor frame, N.
    fn applied_n(&self, a_floor_mps2: f64) -> f64 {
        -self.k_n_per_m * self.x_m - self.c_n_s_per_m * self.v_mps - self.m_kg * a_floor_mps2
    }

    /// Advances by `dt_s` under floor acceleration `a_floor_mps2` (semi-implicit Euler in
    /// slip; stick is exact).
    pub fn step(&mut self, a_floor_mps2: f64, dt_s: f64) {
        if !(dt_s > 0.0 && self.m_kg > 0.0) {
            return;
        }
        let n = self.m_kg * G0_MPS2;
        let f_app = self.applied_n(a_floor_mps2);
        if self.stuck {
            if fmath::abs(f_app) <= self.mu_s * n {
                self.v_mps = 0.0;
                return;
            }
            self.stuck = false;
        }
        let dir = if self.v_mps != 0.0 {
            fmath::copysign(1.0, self.v_mps)
        } else {
            fmath::copysign(1.0, f_app)
        };
        let a = (f_app - dir * self.mu_k * n) / self.m_kg;
        let v_new = self.v_mps + a * dt_s;
        if v_new * dir <= 0.0 {
            // Velocity reversal inside the step: stop, then stick if the bearing holds.
            self.x_m += 0.5 * self.v_mps * dt_s;
            self.v_mps = 0.0;
            self.stuck = fmath::abs(self.applied_n(a_floor_mps2)) <= self.mu_s * n;
        } else {
            self.v_mps = v_new;
            self.x_m += v_new * dt_s;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_exo::tuning::{damper_design, den_hartog, den_hartog_peak, zeta_add_bound};

    fn rel(a: f64, b: f64) -> f64 {
        ((a - b) / b).abs()
    }

    /// SENTINEL-like host: modal mass 1e7 kg, T 7 s, ζs as given.
    fn host(zeta: f64) -> Primary {
        Primary {
            m_kg: 1.0e7,
            omega_rad_s: 2.0 * core::f64::consts::PI / 7.0,
            zeta,
        }
    }

    /// Den Hartog-tuned 2-DOF of mass ratio `mu` with stroke limit `s_max`.
    fn tuned(mu: f64, zeta_s: f64, s_max: f64) -> TwoDof {
        let p = host(zeta_s);
        let d = damper_design(p.m_kg, p.omega_rad_s, mu, den_hartog(mu));
        TwoDof::new(p, Damper::from_design(&d, s_max))
    }

    /// Steady-state structure amplitude ratio X/(F0/K) under harmonic force at ratio `r`.
    fn amplitude_ratio(mu: f64, r: f64) -> f64 {
        let mut sys = tuned(mu, 0.0, 0.0);
        let k = sys.primary.k_n_per_m();
        let f0 = 1.0e-3 * k;
        let w = r * sys.primary.omega_rad_s;
        let per = 2.0 * core::f64::consts::PI / w;
        let n = 200;
        let dt = per / f64::from(n);
        let mut t = 0.0;
        let mut x_max = 0.0_f64;
        for c in 0..104 {
            for _ in 0..n {
                sys.step(0.0, 0.0, f0 * (w * (t + 0.5 * dt)).cos(), dt);
                t += dt;
                if c >= 100 {
                    x_max = x_max.max(sys.state[0].abs());
                }
            }
        }
        x_max / (f0 / k)
    }

    // req: FAB-EXO-602
    #[test]
    fn harmonic_sweep_peak() {
        for mu in [0.01, 0.02] {
            // Coarse sweep then refinement around the highest bin (simulated RK4 sweep).
            let coarse: Vec<(f64, f64)> = (0..=50)
                .map(|i| 0.9 + 0.004 * f64::from(i))
                .map(|r| (amplitude_ratio(mu, r), r))
                .collect();
            let (_, r0) = coarse
                .iter()
                .copied()
                .fold((0.0, 0.0), |a, b| if b.0 > a.0 { b } else { a });
            let peak = (-10..=10)
                .map(|i| r0 + 0.0004 * f64::from(i))
                .map(|r| amplitude_ratio(mu, r))
                .fold(0.0_f64, f64::max);
            let expect = den_hartog_peak(mu);
            println!("μ {mu}: simulated peak {peak:.3} near r = {r0:.3}; √(1 + 2/μ) = {expect:.3}");
            assert!(
                rel(peak, expect) <= 0.02,
                "peak √(1 + 2/μ) ±2 % (FAB-EXO-602, Den Hartog): {peak:.3} vs {expect:.3}"
            );
        }
    }

    /// Steady harmonic forcing at ω_s; returns (X, S, ζ_add) with
    /// ζ_add = E_removed/(2π·K·X²) per cycle.
    fn zeta_add_case(mu: f64, s_max: f64) -> (f64, f64, f64) {
        let mut sys = tuned(mu, 0.0, s_max);
        let k = sys.primary.k_n_per_m();
        let f0 = 1.0e-3 * k; // static deflection 1 mm
        let w = sys.primary.omega_rad_s;
        let per = 2.0 * core::f64::consts::PI / w;
        let n = 400;
        let dt = per / f64::from(n);
        let mut t = 0.0;
        let (mut x, mut s, mut e) = (0.0_f64, 0.0_f64, 0.0_f64);
        let meas = 10;
        for c in 0..(150 + meas) {
            for _ in 0..n {
                let p0 = sys.damper_force_on_structure_n(0.0) * sys.state[1];
                sys.step(0.0, 0.0, f0 * (w * (t + 0.5 * dt)).cos(), dt);
                t += dt;
                if c >= 150 {
                    let p1 = sys.damper_force_on_structure_n(0.0) * sys.state[1];
                    e -= 0.5 * (p0 + p1) * dt;
                    x = x.max(sys.state[0].abs());
                    s = s.max(sys.stroke_m().abs());
                }
            }
        }
        let e_cycle = e / f64::from(meas);
        (x, s, e_cycle / (2.0 * core::f64::consts::PI * k * x * x))
    }

    // req: FAB-EXO-603
    #[test]
    fn zeta_add_stroke_bound() {
        // Bound anchors (E): 3.5 % at μ 1 %, S/X 7; 0.7 % at S/X 1.4.
        assert!(rel(zeta_add_bound(0.01, 7.0), 0.035) < 1e-12);
        assert!(rel(zeta_add_bound(0.01, 1.4), 0.007) < 1e-12);
        // Simulated: free stroke, then end stops that saturate the damper progressively.
        let mut saw_saturation = false;
        for mu in [0.01, 0.02] {
            for s_max in [0.0, 0.1, 0.05, 0.03, 0.02] {
                let (x, s, z) = zeta_add_case(mu, s_max);
                let bound = zeta_add_bound(mu, s / x);
                println!(
                    "μ {mu} s_max {s_max}: X {:.4} m, S {:.4} m, S/X {:.2}, ζ_add {:.4}, bound {:.4}",
                    x,
                    s,
                    s / x,
                    z,
                    bound
                );
                assert!(
                    z <= bound + 0.002,
                    "ζ_add ≤ (μ/2)(S/X) + 0.002 (FAB-EXO-603): {z:.4} vs {bound:.4}"
                );
                if s_max > 0.0 && s / x < 2.0 {
                    saw_saturation = true;
                }
            }
        }
        assert!(
            saw_saturation,
            "the sweep reached stroke saturation (S/X < 2)"
        );
    }

    // req: FAB-EXO-609
    #[test]
    fn coulomb_activation_threshold() {
        // A 1,000 t damper on its tuned spring (T 7 s) over a floor shaking at the tower
        // period: stuck at 0.9·μg, slipping at 1.1·μg (EXOTIC §X8).
        let w = 2.0 * core::f64::consts::PI / 7.0;
        let m = 1.0e6;
        let mus = [
            ("PTFE", BEARING_MU_PTFE),
            ("rolling", BEARING_MU_ROLLING),
            (
                RailCondition::SolidSteel.name(),
                RailCondition::SolidSteel.mu_eq(),
            ),
            (
                RailCondition::LowCoercivity.name(),
                RailCondition::LowCoercivity.mu_eq(),
            ),
            (
                RailCondition::HtsPmGuideway.name(),
                RailCondition::HtsPmGuideway.mu_eq(),
            ),
        ];
        for (name, mu) in mus {
            let run = |factor: f64| -> f64 {
                let mut d = CoulombSdof::new(m, m * w * w, 0.0, mu);
                let a = factor * activation_threshold_mps2(mu);
                let dt = 1.0e-3;
                let mut x_max = 0.0_f64;
                for k in 0..70_000_u32 {
                    let t = f64::from(k) * dt;
                    d.step(a * (w * t).sin(), dt);
                    x_max = x_max.max(d.x_m.abs());
                }
                x_max
            };
            let (stick, slip) = (run(0.9), run(1.1));
            println!(
                "{name:48} μ {mu:.0e}: a_act {:.3} mg; 0.9·μg → {stick:.3e} m, 1.1·μg → {slip:.3e} m",
                mu * 1000.0
            );
            assert_eq!(stick, 0.0, "{name}: no slip at 0.9·μg (FAB-EXO-609)");
            assert!(slip > 0.0, "{name}: slip at 1.1·μg (FAB-EXO-609)");
        }
        // The 1/10 rule against a 5 mg comfort target (≤ 0.5 mg activation), with the rail
        // condition stated: laminations cut eddy drag, not hysteresis drag.
        let target = 5.0e-3 * G0_MPS2;
        assert!(!passes_one_tenth_rule(BEARING_MU_PTFE, target));
        assert!(!passes_one_tenth_rule(BEARING_MU_ROLLING, target));
        for rc in RailCondition::ALL {
            let pass = passes_one_tenth_rule(rc.mu_eq(), target);
            println!(
                "maglev variant, {:48} a_act {:.3} mg → {}",
                rc.name(),
                rc.mu_eq() * 1000.0,
                if pass { "passes" } else { "fails" }
            );
            let expect = matches!(
                rc,
                RailCondition::LowCoercivity
                    | RailCondition::Homopolar
                    | RailCondition::HtsPmGuideway
            );
            assert_eq!(pass, expect, "{}: 1/10 rule", rc.name());
        }
    }

    #[test]
    fn two_dof_conserves_energy_undamped() {
        // ζs = 0, c_d = 0, no stops: RK4 at T/200 keeps the energy within 1e-6 over 100
        // periods (PHYSICS §24).
        let p = host(0.0);
        let mut sys = TwoDof::new(
            p,
            Damper {
                m_kg: 1.0e5,
                k_n_per_m: 1.0e5 * p.omega_rad_s * p.omega_rad_s * 0.96,
                c_n_s_per_m: 0.0,
                s_max_m: 0.0,
            },
        );
        sys.state = [0.1, 0.0, 0.3, 0.0];
        let e0 = sys.mechanical_energy_j();
        let dt = 7.0 / 200.0;
        for _ in 0..20_000 {
            sys.step(0.0, 0.0, 0.0, dt);
        }
        assert!(rel(sys.mechanical_energy_j(), e0) < 1e-6);
        // Detuning scales ω_s (±10 % drift).
        assert!(rel(p.detuned(1.1).omega_rad_s, 1.1 * p.omega_rad_s) < 1e-15);
    }

    #[test]
    fn modal_tower_uniform_shear_closed_form() {
        // Uniform shear building: ω_j = 2√(k/m)·sin((2j − 1)π/(2(2N + 1))) (E, rel 1e-9).
        let n = 20;
        let (m, k) = (5.0e5, 8.0e8);
        let tower = ModalTower::from_storeys(&vec![m; n], &vec![k; n], [0.02, 0.02], 5);
        assert_eq!(tower.modes.len(), 5);
        for (j, mode) in tower.modes.iter().enumerate() {
            let jj = (j + 1) as f64;
            let w = 2.0
                * (k / m).sqrt()
                * ((2.0 * jj - 1.0) * core::f64::consts::PI / (2.0 * (2.0 * n as f64 + 1.0))).sin();
            assert!(
                rel(mode.omega_rad_s, w) < 1e-9,
                "mode {}: {} vs {w}",
                j + 1,
                mode.omega_rad_s
            );
            assert!(rel(mode.phi_damper, 1.0) < 1e-12, "unit roof shape");
        }
        // Rayleigh ζ equals the inputs at modes 1 and 2.
        assert!(rel(tower.modes[0].zeta, 0.02) < 1e-9 && rel(tower.modes[1].zeta, 0.02) < 1e-9);
        // With every mode retained, Σ Γ²·m_modal = total mass (mass participation, rel 1e-9).
        let all = ModalTower::from_storeys(&vec![m; n], &vec![k; n], [0.02, 0.02], n);
        let part: f64 = all
            .modes
            .iter()
            .map(|md| md.gamma * md.gamma * md.m_modal_kg)
            .sum();
        assert!(
            rel(part, m * n as f64) < 1e-9,
            "mass participation sums to the total mass"
        );
        // ±10 % ω drift.
        let d = tower.detuned(0.9);
        assert!(rel(d.modes[0].omega_rad_s, 0.9 * tower.modes[0].omega_rad_s) < 1e-15);
        // Static check: a constant ground acceleration settles the roof at Σ Γ·φ·a/ω².
        let mut t = tower.clone();
        let a = 0.1;
        for _ in 0..400_000 {
            t.step(a, 0.0, 1.0e-3);
        }
        let expect: f64 = tower
            .modes
            .iter()
            .map(|md| -md.gamma * a / (md.omega_rad_s * md.omega_rad_s))
            .sum();
        assert!(
            rel(t.damper_disp_m(), expect) < 1e-6,
            "static roof response {} vs {expect}",
            t.damper_disp_m()
        );
        // Invalid input yields no modes rather than a panic.
        assert!(
            ModalTower::from_storeys(&[1.0], &[1.0, 2.0], [0.02, 0.02], 3)
                .modes
                .is_empty()
        );
    }

    #[test]
    fn davenport_wind_synthesis() {
        let wind = DavenportWind {
            u10_mps: 30.0,
            kappa: 0.005,
            area_m2: 2000.0,
            c_d: 1.3,
            rho_kg_per_m3: 1.225,
        };
        // σ_u² = 6κU² over the full spectrum; the 1 mHz–2 Hz band holds ≈ 95 % of it.
        let full = wind.sigma_u_mps() * wind.sigma_u_mps();
        let band = wind.band_variance();
        assert!(rel(full, 6.0 * 0.005 * 900.0) < 1e-12);
        assert!(
            band / full > 0.9 && band / full < 1.0,
            "band fraction {:.3}",
            band / full
        );
        // Series = spot evaluation (same synthesis), and its variance matches the band.
        let fs = 5.0;
        let n = 100_000;
        let seed = 42;
        let f = wind.series(fs, n, seed);
        for k in [0_usize, 4095, 4096, 77_777] {
            let spot = wind.force_n(k as f64 / fs, seed);
            assert!(
                (f[k] - spot).abs() <= 1e-6 * wind.force_gain_n_s_per_m() * wind.sigma_u_mps(),
                "series[{k}] = force_n"
            );
        }
        let var = f.iter().map(|x| x * x).sum::<f64>() / n as f64;
        let g = wind.force_gain_n_s_per_m();
        assert!(
            rel(var / (g * g), band) < 0.1,
            "series variance ≈ band variance ±10 %"
        );
        assert_ne!(
            wind.series(fs, 16, 1),
            wind.series(fs, 16, 2),
            "seeds differ"
        );
        assert_eq!(
            wind.series(fs, 16, 1),
            wind.series(fs, 16, 1),
            "deterministic"
        );
    }
}
