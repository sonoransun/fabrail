//! Luenberger gap-velocity observer (4*omega) and the non-safety payload estimate m_hat (PHYSICS §15). Owner: A1.
//!
//! Frozen API (W0): [`GapObserver`] per controlled mode (δg, δġ from the measured modal
//! gap and the commanded modal current, bandwidth `observer_ratio`·ω), and
//! [`PayloadEstimator`] (m̂ from the steady modal current at a known gap; LEV-013:
//! within ±5 % in ≤ 1 s; non-safety — the governor uses max(m_rated, 1.05·m̂) and the
//! monitor uses rated mass only).
//!
//! Observer structure (current-estimator form; Franklin, Powell & Workman, "Digital
//! Control of Dynamic Systems", 3rd ed., §8.2): the design plant x = [δg, δġ, i, i_e]
//! is propagated exactly with its ZOH model (Φ, Γ) from the corrected mechanical
//! estimate, the MEASURED coil current i\[k\] and the LET-delayed command u_d\[k\] (known:
//! it is the previous release's output, applied over the coming period), and only the
//! mechanical states are corrected from the gap:
//!
//! ```text
//! corrected:  x̂_m = x̂_m⁻ + L_c·(y − δĝ⁻)
//! predicted:  [δĝ, δġ̂, î_e]⁻(k+1) = Φ_r·[x̂_m, i[k], î_e⁻] + Γ_r·u_d[k]
//! ```
//!
//! With an exact model the estimation error is autonomous — mechanical error poles at
//! the placed pair (ω_o = observer_ratio·ω, ζ_o = ζ), the i_e error at the eddy pole —
//! so the closed loop keeps the discrete design poles exactly (separation principle;
//! LEV-007 is tested on this f32 code). Using the measured current (not a modelled
//! current loop) keeps the estimate right when the bridge saturates. Φ is stored as
//! Φ − I so the O((pT)²) mechanical terms survive f32 rounding.
//!
//! The heave observer carries one more state, a constant acceleration disturbance d
//! (ḋ = 0, entering like gravity through Γ_d = [(cosh pT − 1)/p², sinh(pT)/p]), placed
//! at [`OBS_DIST_RATIO`]·ω_o. It absorbs the linear model's offset away from (g*, m*) —
//! a heavier payload floating at its own zero-power gap, the seat hold at 3.4 mm, the
//! band edge — so the gap and rate estimates are unbiased at every equilibrium (the
//! band clamp and the seat hold regulate the true gap). Being an observer state it
//! keeps the separation property (the design poles are unchanged).
//!
//! An optional IMU-aided (kinematic) update replaces the model acceleration by the
//! measured relative acceleration a_rel = a_rail,z − a_imu,z (both read the specific
//! force, so gravity cancels; δg̈ = a_rail − a_body), with its own gains for the same
//! bandwidth. [`GapObserver::new`] builds the purely kinematic observer (u is then an
//! acceleration).

use fabrail_params::derive::G0_MPS2;
use fabrail_params::{ParamError, SystemParams};
use fabrail_types::fmath;

use crate::levitation::design::{ElementModel, ModePlant};

/// Disturbance-state pole of the heave observer as a fraction of the observer bandwidth
/// (ω_o/4 = the design ω). Design choice checked in the LEV-020 analysis: with it the
/// heave loop tolerates τ_e up to ≈ 2.5 / 3.0 / 3.8 / 3.8 ms (DRAWER / DOOR / WALL /
/// HEAVY), versus the 2 ms requirement.
pub const OBS_DIST_RATIO: f64 = 0.25;

/// Second-order Luenberger observer for one mode.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GapObserver {
    /// Observer gains [l1, l2] (current-estimator form, model-based update).
    pub l: [f32; 2],
    /// Estimated deviation, m (or rad).
    pub x_hat: f32,
    /// Estimated rate, m/s (or rad/s).
    pub xdot_hat: f32,
    /// Period, s.
    pub dt_s: f32,
    /// Predicted [δg, δġ, i_e] for the next sample.
    pub pred: [f32; 3],
    /// Φ − I rows for δg, δġ, i_e over the columns [δg, δġ, i, i_e].
    pub dphi: [[f32; 4]; 3],
    /// Γ rows for δg, δġ, i_e.
    pub gam: [f32; 3],
    /// Command applied over the coming period u_d (the previous output), A.
    pub u_applied: f32,
    /// Model-based observer (false: kinematic double integrator only).
    pub model: bool,
    /// Kinematic (IMU-aided) gains [l1, l2].
    pub lk: [f32; 2],
    /// A first sample has initialised the estimate.
    pub primed: bool,
    /// Disturbance gain (current-estimator form; 0 without the disturbance state).
    pub ld: f32,
    /// Disturbance input column Γ_d for [δg, δġ].
    pub gdist: [f32; 2],
    /// Estimated constant acceleration disturbance d̂, m/s² (per modal coordinate).
    pub d_hat: f32,
}

/// Current-estimator gains L_c = Φ_m⁻¹·L_p for the mechanical block Φ_m (f64), placing
/// eig(Φ_m − L_p·[1 0]) at z = e^{sT} of s² + 2ζω_o·s + ω_o² (Ackermann for n = 2).
fn place2(phi_m: [[f64; 2]; 2], t: f64, w_o: f64, zeta: f64) -> [f32; 2] {
    let (a, b, c, d) = (phi_m[0][0], phi_m[0][1], phi_m[1][0], phi_m[1][1]);
    // Target z-polynomial z² − s1·z + s0.
    let (s1, s0) = if zeta < 1.0 {
        let wd = w_o * fmath::sqrt(1.0 - zeta * zeta);
        let r = fmath::exp(-zeta * w_o * t);
        (2.0 * r * fmath::cos(wd * t), r * r)
    } else {
        let q = w_o * fmath::sqrt(zeta * zeta - 1.0);
        let z1 = fmath::exp((-zeta * w_o + q) * t);
        let z2 = fmath::exp((-zeta * w_o - q) * t);
        (z1 + z2, z1 * z2)
    };
    if b == 0.0 {
        return [0.0; 2];
    }
    let lp0 = a + d - s1;
    let lp1 = (s0 - (a - lp0) * d + b * c) / b;
    // L_c = Φ_m⁻¹·L_p.
    let det = a * d - b * c;
    if det == 0.0 {
        return [0.0; 2];
    }
    [
        ((d * lp0 - b * lp1) / det) as f32,
        ((-c * lp0 + a * lp1) / det) as f32,
    ]
}

type M3 = [[f64; 3]; 3];

fn m3_mul(a: &M3, b: &M3) -> M3 {
    let mut c = [[0.0f64; 3]; 3];
    for (crow, arow) in c.iter_mut().zip(a.iter()) {
        for (aik, brow) in arow.iter().zip(b.iter()) {
            for (cij, bkj) in crow.iter_mut().zip(brow.iter()) {
                *cij += aik * bkj;
            }
        }
    }
    c
}

fn m3_inv(m: &M3) -> Option<M3> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    let k = 1.0 / det;
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * k,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * k,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * k,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * k,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * k,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * k,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * k,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * k,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * k,
        ],
    ])
}

/// Current-estimator gains for the 3-state [δg, δġ, d] observer with y = δg (Ackermann:
/// L_p = φ(Φ)·O⁻¹·e₃, O = [C; CΦ; CΦ²]; L_c = Φ⁻¹·L_p), placing the pair (ω_o, ζ) and
/// the real pole r·ω_o.
fn place3(phi: &M3, t: f64, w_o: f64, zeta: f64, r: f64) -> Option<[f32; 3]> {
    let (s1, s0) = if zeta < 1.0 {
        let wd = w_o * fmath::sqrt(1.0 - zeta * zeta);
        let q = fmath::exp(-zeta * w_o * t);
        (2.0 * q * fmath::cos(wd * t), q * q)
    } else {
        let q = w_o * fmath::sqrt(zeta * zeta - 1.0);
        let z1 = fmath::exp((-zeta * w_o + q) * t);
        let z2 = fmath::exp((-zeta * w_o - q) * t);
        (z1 + z2, z1 * z2)
    };
    let z3 = fmath::exp(-r * w_o * t);
    // (z² − s1·z + s0)(z − z3) = z³ + c2·z² + c1·z + c0.
    let (c2, c1, c0) = (-s1 - z3, s0 + s1 * z3, -s0 * z3);
    let phi2 = m3_mul(phi, phi);
    let phi3 = m3_mul(&phi2, phi);
    let mut poly = [[0.0f64; 3]; 3];
    for (r_, row) in poly.iter_mut().enumerate() {
        for (c_, v) in row.iter_mut().enumerate() {
            let id = if r_ == c_ { 1.0 } else { 0.0 };
            let at = |m: &M3| m.get(r_).and_then(|x| x.get(c_)).copied().unwrap_or(0.0);
            *v = at(&phi3) + c2 * at(&phi2) + c1 * at(phi) + c0 * id;
        }
    }
    let o: M3 = [[1.0, 0.0, 0.0], phi[0], phi2[0]];
    let o_inv = m3_inv(&o)?;
    // e3 column of O⁻¹.
    let col = [o_inv[0][2], o_inv[1][2], o_inv[2][2]];
    let lp = [
        poly[0][0] * col[0] + poly[0][1] * col[1] + poly[0][2] * col[2],
        poly[1][0] * col[0] + poly[1][1] * col[1] + poly[1][2] * col[2],
        poly[2][0] * col[0] + poly[2][1] * col[1] + poly[2][2] * col[2],
    ];
    let pi = m3_inv(phi)?;
    let lc = [
        pi[0][0] * lp[0] + pi[0][1] * lp[1] + pi[0][2] * lp[2],
        pi[1][0] * lp[0] + pi[1][1] * lp[1] + pi[1][2] * lp[2],
        pi[2][0] * lp[0] + pi[2][1] * lp[1] + pi[2][2] * lp[2],
    ];
    if lc.iter().all(|v| v.is_finite()) {
        Some([lc[0] as f32, lc[1] as f32, lc[2] as f32])
    } else {
        None
    }
}

impl GapObserver {
    /// Kinematic observer (double integrator; `step`'s `u` is an acceleration, m/s²)
    /// at `bw_rad_s` (ζ = 0.7) for sample period `dt_s`.
    pub fn new(bw_rad_s: f32, dt_s: f32) -> GapObserver {
        let t = f64::from(dt_s);
        let lk = if dt_s > 0.0 && bw_rad_s > 0.0 {
            place2([[1.0, t], [0.0, 1.0]], t, f64::from(bw_rad_s), 0.7)
        } else {
            [0.0; 2]
        };
        GapObserver {
            l: lk,
            lk,
            dt_s,
            dphi: [[0.0, dt_s, 0.0, 0.0], [0.0; 4], [0.0; 4]],
            gam: [0.0; 3],
            model: false,
            ..GapObserver::default()
        }
    }

    /// Model-based observer from the mode's sampled design plant, at bandwidth
    /// `bw_rad_s` with damping `zeta`.
    pub fn from_model(plant: &ModePlant, dt_s: f32, bw_rad_s: f32, zeta: f32) -> GapObserver {
        let t = f64::from(dt_s);
        let (w, z) = (f64::from(bw_rad_s), f64::from(zeta));
        let phi = &plant.phi;
        let l = place2([[phi[0][0], phi[0][1]], [phi[1][0], phi[1][1]]], t, w, z);
        let lk = place2([[1.0, t], [0.0, 1.0]], t, w, z);
        let row = |r: &[f64; 4], diag: usize| -> [f32; 4] {
            let mut o = [0.0f32; 4];
            for (j, (d, s)) in o.iter_mut().zip(r.iter()).enumerate() {
                *d = (if j == diag { *s - 1.0 } else { *s }) as f32;
            }
            o
        };
        GapObserver {
            l,
            lk,
            dt_s,
            dphi: [row(&phi[0], 0), row(&phi[1], 1), row(&phi[3], 3)],
            gam: [
                plant.gam[0] as f32,
                plant.gam[1] as f32,
                plant.gam[3] as f32,
            ],
            model: true,
            ..GapObserver::default()
        }
    }

    /// Model-based observer with the constant-acceleration disturbance state (heave):
    /// open-loop pole squared `p2` (1/s²) gives Γ_d; the disturbance pole sits at
    /// [`OBS_DIST_RATIO`]·ω_o. Falls back to [`GapObserver::from_model`] if the
    /// placement is singular.
    pub fn from_model_dist(
        plant: &ModePlant,
        p2: f64,
        dt_s: f32,
        bw_rad_s: f32,
        zeta: f32,
    ) -> GapObserver {
        let mut o = GapObserver::from_model(plant, dt_s, bw_rad_s, zeta);
        let t = f64::from(dt_s);
        if p2.is_nan() || p2 <= 0.0 || t <= 0.0 {
            return o;
        }
        let p = fmath::sqrt(p2);
        let gd = [(fmath::cosh(p * t) - 1.0) / p2, fmath::sinh(p * t) / p];
        let phi = &plant.phi;
        let po: M3 = [
            [phi[0][0], phi[0][1], gd[0]],
            [phi[1][0], phi[1][1], gd[1]],
            [0.0, 0.0, 1.0],
        ];
        if let Some(l) = place3(&po, t, f64::from(bw_rad_s), f64::from(zeta), OBS_DIST_RATIO) {
            o.l = [l[0], l[1]];
            o.ld = l[2];
            o.gdist = [gd[0] as f32, gd[1] as f32];
        }
        o
    }

    /// One update from the measured deviation `y` and the modal input `u`: the measured
    /// modal coil current (A) for a model-based observer, the measured acceleration
    /// (m/s²) for a kinematic one. Returns the corrected (estimate, rate).
    pub fn step(&mut self, y: f32, u: f32) -> (f32, f32) {
        if !self.primed {
            self.reset(y);
        }
        let e = y - self.pred[0];
        let g = self.pred[0] + self.l[0] * e;
        let gd = self.pred[1] + self.l[1] * e;
        let d = self.d_hat + self.ld * e;
        self.x_hat = g;
        self.xdot_hat = gd;
        if self.model {
            let x = [g, gd, u, self.pred[2]];
            self.pred = [
                g + dot4(&self.dphi[0], &x) + self.gam[0] * self.u_applied + self.gdist[0] * d,
                gd + dot4(&self.dphi[1], &x) + self.gam[1] * self.u_applied + self.gdist[1] * d,
                self.pred[2] + dot4(&self.dphi[2], &x) + self.gam[2] * self.u_applied,
            ];
            self.d_hat = d;
        } else {
            let t = self.dt_s;
            self.pred = [g + t * gd + 0.5 * t * t * u, gd + t * u, 0.0];
        }
        (g, gd)
    }

    /// IMU-aided update: corrects with the kinematic gains and propagates the mechanical
    /// states with the measured relative acceleration `a_rel` (m/s²); the eddy state
    /// still follows the measured current `i` so a switch back is seamless.
    pub fn step_imu(&mut self, y: f32, i: f32, a_rel: f32) -> (f32, f32) {
        if !self.primed {
            self.reset(y);
        }
        let e = y - self.pred[0];
        let g = self.pred[0] + self.lk[0] * e;
        let gd = self.pred[1] + self.lk[1] * e;
        self.x_hat = g;
        self.xdot_hat = gd;
        let t = self.dt_s;
        let ie = if self.model {
            let x = [g, gd, i, self.pred[2]];
            self.pred[2] + dot4(&self.dphi[2], &x) + self.gam[2] * self.u_applied
        } else {
            0.0
        };
        self.pred = [g + t * gd + 0.5 * t * t * a_rel, gd + t * a_rel, ie];
        (g, gd)
    }

    /// Records the command that will be applied over the next period (the release's
    /// output after allocation and clamping, in modal units).
    #[inline]
    pub fn set_command(&mut self, u: f32) {
        self.u_applied = if u.is_finite() { u } else { 0.0 };
    }

    /// Re-initialises at `y` with zero rate (the eddy state keeps its value).
    pub fn reset(&mut self, y: f32) {
        self.x_hat = y;
        self.xdot_hat = 0.0;
        self.pred[0] = y;
        self.pred[1] = 0.0;
        self.primed = true;
    }

    /// Re-initialises at `y` with zero rate, eddy-current state `i_e` and zero
    /// disturbance.
    pub fn reset_full(&mut self, y: f32, i_e: f32) {
        self.reset(y);
        self.pred[2] = i_e;
        self.d_hat = 0.0;
    }
}

/// Coupled roll–sway observer (PHYSICS §6, §15; Wave 2 decision D1): the current-estimator
/// Luenberger observer of the roll–sway model x = [u, u̇, Y, Ẏ, i, i_e]
/// (`derive::RollSwayModel`, u = r̄·∂g/∂y the modal roll gap, Y the CoM sway). Only the
/// roll gap is measured; the measured modal current and the LET-delayed command drive
/// the exact ZOH model, the four mechanical states are corrected from the gap
/// innovation with `derive::roll_sway_design` gains (roll pair at observer_ratio·ω, sway
/// pair at 0.7·ω_y) and the eddy state is propagated. Φ is stored as Φ − I (f32
/// precision of the O(T) terms, as in [`GapObserver`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CoupledObserver {
    /// Correction gains for [u, u̇, Y, Ẏ].
    pub l: [f32; 4],
    /// Φ − I rows for [u, u̇, Y, Ẏ, i_e] over the columns [u, u̇, Y, Ẏ, i, i_e].
    pub dphi: [[f32; 6]; 5],
    /// Γ rows for [u, u̇, Y, Ẏ, i_e].
    pub gam: [f32; 5],
    /// Predicted [u, u̇, Y, Ẏ, i_e] for the next sample.
    pub pred: [f32; 5],
    /// Corrected [u, u̇, Y, Ẏ].
    pub x_hat: [f32; 4],
    /// Command applied over the coming period u_d, A.
    pub u_applied: f32,
    /// A first sample has initialised the estimate.
    pub primed: bool,
}

impl CoupledObserver {
    /// From the sampled design plant (Φ, Γ over [u, u̇, Y, Ẏ, i, i_e]) and the gains.
    pub fn new(phi: &[[f64; 6]; 6], gam: &[f64; 6], l: &[f64; 4]) -> CoupledObserver {
        let rows = [0usize, 1, 2, 3, 5];
        let mut dphi = [[0.0f32; 6]; 5];
        let mut g = [0.0f32; 5];
        for (k, &r) in rows.iter().enumerate() {
            if let (Some(src), Some(dst), Some(gs)) = (phi.get(r), dphi.get_mut(k), g.get_mut(k)) {
                for (c, (d, v)) in dst.iter_mut().zip(src.iter()).enumerate() {
                    *d = (if c == r { *v - 1.0 } else { *v }) as f32;
                }
                *gs = gam.get(r).copied().unwrap_or(0.0) as f32;
            }
        }
        CoupledObserver {
            l: [l[0] as f32, l[1] as f32, l[2] as f32, l[3] as f32],
            dphi,
            gam: g,
            ..CoupledObserver::default()
        }
    }

    /// One update from the measured modal roll gap `y` (m) and measured modal current
    /// `i` (A). Returns the corrected [u, u̇, Y, Ẏ].
    pub fn step(&mut self, y: f32, i: f32) -> [f32; 4] {
        if !self.primed {
            self.reset(y);
        }
        let e = y - self.pred[0];
        let mut xh = [0.0f32; 4];
        for (k, (x, l)) in xh.iter_mut().zip(self.l.iter()).enumerate() {
            *x = self.pred.get(k).copied().unwrap_or(0.0) + l * e;
        }
        let ie = self.pred[4];
        let x = [xh[0], xh[1], xh[2], xh[3], i, ie];
        let base = [xh[0], xh[1], xh[2], xh[3], ie];
        let mut next = [0.0f32; 5];
        for (k, n) in next.iter_mut().enumerate() {
            let row = self.dphi.get(k).copied().unwrap_or([0.0; 6]);
            let acc: f32 = row.iter().zip(x.iter()).map(|(a, b)| a * b).sum();
            *n = base.get(k).copied().unwrap_or(0.0)
                + acc
                + self.gam.get(k).copied().unwrap_or(0.0) * self.u_applied;
        }
        if next.iter().all(|v| v.is_finite()) && xh.iter().all(|v| v.is_finite()) {
            self.pred = next;
            self.x_hat = xh;
        } else {
            self.reset(y);
        }
        self.x_hat
    }

    /// Records the command that will be applied over the next period, A.
    #[inline]
    pub fn set_command(&mut self, u: f32) {
        self.u_applied = if u.is_finite() { u } else { 0.0 };
    }

    /// Re-initialises at roll gap `y` with zero rates and zero sway (the eddy state keeps
    /// its value).
    pub fn reset(&mut self, y: f32) {
        self.pred[0] = y;
        self.pred[1] = 0.0;
        self.pred[2] = 0.0;
        self.pred[3] = 0.0;
        self.x_hat = [y, 0.0, 0.0, 0.0];
        self.primed = true;
    }

    /// Re-initialises at `y` with eddy-current state `i_e`.
    pub fn reset_full(&mut self, y: f32, i_e: f32) {
        self.reset(y);
        self.pred[4] = i_e;
    }
}

#[inline]
fn dot4(a: &[f32; 4], b: &[f32; 4]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

/// Payload estimator m̂ (non-safety).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PayloadEstimator {
    /// Current estimate, kg.
    pub m_hat_kg: f32,
    /// Filter time constant, s.
    pub tau_s: f32,
    /// Element closed forms (m = n·F(g, i)/g₀).
    pub elem: ElementModel,
    /// Lower clamp of the estimate (half the minimum operating mass), kg.
    pub m_min_kg: f32,
    /// Upper clamp of the estimate (twice the rated mass), kg.
    pub m_max_kg: f32,
}

impl PayloadEstimator {
    /// Initialises at the rated mass with the tuning time constant.
    pub fn new(p: &SystemParams) -> Result<PayloadEstimator, ParamError> {
        let c = &p.design.carriage;
        Ok(PayloadEstimator {
            m_hat_kg: c.m_rated_kg as f32,
            tau_s: p.control.levitation.m_hat_tau_s as f32,
            elem: ElementModel::new(p),
            m_min_kg: (0.5 * c.m_op_min_kg) as f32,
            m_max_kg: (2.0 * c.m_rated_kg) as f32,
        })
    }

    /// One update from the heave gap (m) and mean coil current (A) while levitated:
    /// first-order filter (time constant `tau_s`) toward the quasi-static closed form
    /// m = n·F(g, ī)/g₀ (PHYSICS §3; exact at zero power, i = 0, where it reduces to
    /// m*·((2g* + h′)/(2g + h′))²).
    pub fn step(&mut self, gap_m: f32, i_mean_a: f32, dt_s: f32) -> f32 {
        let m = self.elem.mass(gap_m, i_mean_a);
        if !m.is_finite() || dt_s <= 0.0 {
            return self.m_hat_kg;
        }
        let k = if self.tau_s > 0.0 {
            fmath::fminf(dt_s / self.tau_s, 1.0)
        } else {
            1.0
        };
        self.m_hat_kg += k * (m - self.m_hat_kg);
        if self.m_max_kg > self.m_min_kg {
            self.m_hat_kg = fmath::clampf(self.m_hat_kg, self.m_min_kg, self.m_max_kg);
        }
        self.m_hat_kg
    }

    /// Sets the estimate (e.g. from the unseat closed form, F17).
    pub fn set(&mut self, m_kg: f32) {
        if m_kg.is_finite() && m_kg > 0.0 {
            self.m_hat_kg = m_kg;
        }
    }
}

/// Weight of mass `m_kg`, N (g₀ from `derive`).
#[inline]
pub fn weight_n(m_kg: f32) -> f32 {
    m_kg * G0_MPS2 as f32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::levitation::design::{design, zoh_plant};
    use fabrail_params::presets;

    #[test]
    fn kinematic_observer_tracks_a_ramp() {
        let mut o = GapObserver::new(700.0, 1e-4);
        let mut last = (0.0, 0.0);
        for k in 0..2000 {
            let y = 1e-3 + 0.01 * (k as f32) * 1e-4; // 10 mm/s
            last = o.step(y, 0.0);
        }
        assert!((last.1 - 0.01).abs() < 1e-5, "rate of a ramp: {}", last.1);
    }

    #[test]
    fn imu_aided_update_tracks_measured_acceleration() {
        // δg(t) = A·sin(ωt) with the exact relative acceleration fed in: the IMU-aided
        // (kinematic) update tracks the rate without the plant model.
        let p = presets::door();
        let g = design(&p).expect("door");
        let md = g.modes[0];
        let mut o = GapObserver::from_model(&md.plant, g.dt_s, md.observer_rad_s, 0.7);
        let (a, w) = (50e-6f64, 2.0 * core::f64::consts::PI * 5.0);
        let dt = f64::from(g.dt_s);
        let mut err = 0.0f64;
        for k in 0..4000 {
            let t = k as f64 * dt;
            let y = a * (w * t).sin();
            let acc = -a * w * w * (w * t).sin();
            let (_, gd) = o.step_imu(y as f32, 0.0, acc as f32);
            if k > 2000 {
                err = err.max((f64::from(gd) - a * w * (w * t).cos()).abs());
            }
        }
        assert!(
            err < 0.02 * a * w,
            "IMU-aided rate error {err} (tol 2 % of amplitude)"
        );
    }

    #[test]
    fn model_observer_error_decays_at_placed_poles() {
        // Plant and observer share the model: from a wrong initial estimate the error
        // decays, and after 10/ω_o the velocity error is < 1 % of its peak.
        let p = presets::door();
        let g = design(&p).expect("door");
        let md = g.modes[0];
        let mut o = GapObserver::from_model(&md.plant, g.dt_s, md.observer_rad_s, 0.7);
        let pl = zoh_plant(
            f64::from(md.p2_per_s2),
            f64::from(md.b_m_per_a_s2),
            f64::from(g.dt_s),
            f64::from(g.tau_cl_s),
            f64::from(g.tau_e_s),
        );
        // Plant state with a steady command that balances nothing (open-loop drift is
        // fine over a short window).
        let mut x = [1e-4f64, 0.0, 0.0, 0.0];
        let u = 0.2f64;
        o.reset(0.0);
        let n = (10.0 / md.observer_rad_s / g.dt_s) as usize;
        let mut err = 0.0f64;
        let mut peak = 0.0f64;
        for _ in 0..n {
            let (_, gd) = o.step(x[0] as f32, x[2] as f32);
            err = (f64::from(gd) - x[1]).abs();
            peak = peak.max(err);
            o.set_command(u as f32);
            let mut nx = [0.0f64; 4];
            for (r, v) in nx.iter_mut().enumerate() {
                *v = (0..4).map(|c| pl.phi[r][c] * x[c]).sum::<f64>() + pl.gam[r] * u;
            }
            x = nx;
        }
        // ζ_o = 0.7 envelope after 10/ω_o: e^{−7}/√(1 − ζ²) ≈ 1.3e-3 (×4 margin for the
        // unstable open-loop drift of the plant state during the window).
        assert!(
            err < 5e-3 * peak,
            "rate error {err} after 10/ω_o vs peak {peak}"
        );
    }

    #[test]
    fn payload_estimate_is_closed_form() {
        let p = presets::drawer();
        let mut e = PayloadEstimator::new(&p).expect("drawer");
        assert_eq!(e.m_hat_kg, 48.0);
        // At zero power the mass follows m(g) (LEV-005: 5.0 mm → 9.96 kg).
        for _ in 0..20_000 {
            e.step(5.0e-3, 0.0, 1e-4);
        }
        assert!(
            (e.m_hat_kg - 9.96).abs() < 0.02,
            "m(5 mm) = 9.96 kg: {}",
            e.m_hat_kg
        );
    }
}
