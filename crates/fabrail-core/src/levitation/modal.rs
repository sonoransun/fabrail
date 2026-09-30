//! Modal decomposition and allocation A = W^-1 T^T (T W^-1 T^T)^-1 with the null-space (warp) loop (PHYSICS §15). Owner: A1.
//!
//! Frozen API (W0): [`Modal::new`], [`Modal::decompose`] (magnet gaps → modal
//! coordinates), [`Modal::allocate`] (modal current demands → magnet currents) and
//! [`Modal::warp`] (null-space component, integrated to zero by the warp loop at
//! `levitation.warp_bw_hz`; LEV-009). Tests: `allocation_identity` (1e-6 f32),
//! `nullspace_zero_net`.
//!
//! Conventions (PHYSICS §15; the per-element modal model of `derive::ModeModel`):
//! - Lever arms about the CoM, r_x,i = x_i − x_c (pitch) and r_y,i = y_i − y_c (roll),
//!   over the lift elements (PM-biased, upward normal); r̄ = √(Σr²/n) per mode.
//! - Current/force map T (modes × magnets, the controlled set only): heave row 1/n,
//!   pitch row r_x,i/(n·r̄_x), roll row r_y,i/(n·r̄_y). A modal current i_mode then
//!   produces the generalised force of the design model (heave n·k_i·i_h, pitch torque
//!   n·k_i·r̄_x·i_p), and A = W⁻¹Tᵀ(TW⁻¹Tᵀ)⁻¹ gives T·A = I with the minimum weighted
//!   current norm (for a centred pattern A_h,i = 1, A_p,i = r_x,i/r̄_x). Pure-EMS
//!   hold-down elements are excluded (infinite weight) and served by the one-sided
//!   sub-allocator [`Modal::hold_down`].
//! - Gap decomposition: the rigid-body fit δg_i = h + r_x,i·θ_p + r_y,i·θ_r
//!   (least squares over the valid lift elements, the controlled modes; roll is still
//!   reported on top-hung classes from the full fit when the pattern allows it).
//!   `decompose` returns [heave gap at the CoM projection (absolute, m), θ_p (rad, the
//!   gap slope ∂g/∂x), θ_r (rad, ∂g/∂y)]; the control coordinate of a rotation is
//!   u = r̄·θ (m).
//! - Warp: the null space of T (element currents with no net controlled wrench; on
//!   top-hung classes it includes the passive roll differential). The warp loop
//!   integrates the null-space part of the measured coil currents to zero at
//!   ω_w = 2π·warp_bw_hz (2 Hz: ≤ 5 % in 1 s is e^{−ω_w·1 s} ≈ 3.5e-6, LEV-009).

use fabrail_params::{ParamError, SystemParams, derive};
use fabrail_types::{MAX_MAGNETS, fmath};

use crate::linalg::{self, AllocMap, ModalMap, Vec3};

/// Modal map and allocation for the controlled modes.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Modal {
    /// Modal map T (rows heave, pitch, roll).
    pub t: ModalMap,
    /// Allocation A = W⁻¹Tᵀ(TW⁻¹Tᵀ)⁻¹.
    pub a: AllocMap,
    /// Active elements.
    pub n_magnets: u8,
    /// Controlled modes (2 or 3).
    pub n_modes: u8,
    /// Warp-loop integrator states, A.
    pub warp_i_a: [f32; MAX_MAGNETS],
    /// Gap decomposition rows [heave, θ_p, θ_r] (all valid lift channels).
    pub d: ModalMap,
    /// RMS lever arms [1, r̄_x, r̄_y], m (u = r̄·θ).
    pub r_bar_m: Vec3,
    /// Kinematic rows [1, r_x,i, r_y,i] per active element (lever arms about the CoM;
    /// the decomposition and T use the lift elements only).
    pub k_rows: [[f32; 3]; MAX_MAGNETS],
    /// Lift-element mask (PM-biased, upward normal).
    pub lift_mask: u8,
    /// Hold-down (pure-EMS) element mask.
    pub hold_mask: u8,
    /// Warp-loop rate ω_w = 2π·warp_bw_hz, 1/s.
    pub warp_rad_s: f32,
    /// Channel mask the decomposition `d` was built for.
    pub d_mask: u8,
    /// Roll is reported from the full fit although not controlled.
    pub roll_reported: bool,
}

impl Modal {
    /// Builds T from the magnet stations and A from `linalg::weighted_pinv`.
    ///
    /// Errors when there are no lift elements, the controlled modes are not a prefix
    /// of (heave, pitch, roll), or the allocation is singular.
    pub fn new(p: &SystemParams) -> Result<Modal, ParamError> {
        let s = &p.design.suspension;
        let c = &p.design.carriage;
        let n = usize::from(s.n_magnets);
        if n == 0 || n > MAX_MAGNETS {
            return Err(ParamError::new(
                "design.suspension.n_magnets",
                "must be 1..=MAX_MAGNETS",
            ));
        }
        let active = [
            derive::mode_active(p, derive::LevMode::Heave),
            derive::mode_active(p, derive::LevMode::Pitch),
            derive::mode_active(p, derive::LevMode::Roll),
        ];
        let n_modes: u8 = match active {
            [true, true, true] => 3,
            [true, true, false] => 2,
            [true, false, false] => 1,
            _ => {
                return Err(ParamError::new(
                    "control.levitation.roll_active",
                    "controlled modes must be heave[, pitch[, roll]]",
                ));
            }
        };
        let mut m = Modal {
            n_magnets: s.n_magnets,
            n_modes,
            warp_rad_s: (core::f64::consts::TAU * p.control.levitation.warp_bw_hz) as f32,
            ..Modal::default()
        };
        let (mut sx, mut sy, mut nl) = (0.0f64, 0.0f64, 0.0f64);
        for (i, (st, kr)) in s
            .stations
            .iter()
            .zip(m.k_rows.iter_mut())
            .enumerate()
            .take(n)
        {
            let bit = 1u8 << i;
            let (rx, ry) = (st.x_m - c.cg_m[0], st.y_m - c.cg_m[1]);
            *kr = [1.0, rx as f32, ry as f32];
            if st.normal_sign > 0.0 && st.has_pm {
                m.lift_mask |= bit;
                sx += rx * rx;
                sy += ry * ry;
                nl += 1.0;
            } else {
                m.hold_mask |= bit;
            }
        }
        if nl <= 0.0 {
            return Err(ParamError::new(
                "design.suspension.stations",
                "no lift element",
            ));
        }
        let rbx = fmath::sqrt(sx / nl);
        let rby = fmath::sqrt(sy / nl);
        m.r_bar_m = [1.0, rbx as f32, rby as f32];
        // T rows over the lift elements.
        for (i, kr) in m.k_rows.iter().enumerate().take(n) {
            if m.lift_mask & (1u8 << i) == 0 {
                continue;
            }
            let vals = [
                1.0 / nl,
                if rbx > 0.0 {
                    f64::from(kr[1]) / (nl * rbx)
                } else {
                    0.0
                },
                if rby > 0.0 {
                    f64::from(kr[2]) / (nl * rby)
                } else {
                    0.0
                },
            ];
            for (row, v) in m.t.iter_mut().zip(vals.iter()).take(usize::from(n_modes)) {
                if let Some(e) = row.get_mut(i) {
                    *e = *v as f32;
                }
            }
        }
        m.a = linalg::weighted_pinv(&m.t, &m.lift_weights(), s.n_magnets, n_modes)
            .ok_or_else(|| ParamError::new("design.suspension.stations", "allocation singular"))?;
        if !m.rebuild_decomposition(m.lift_mask) {
            return Err(ParamError::new(
                "design.suspension.stations",
                "gap decomposition singular",
            ));
        }
        Ok(m)
    }

    /// Allocation weights: 1 for lift elements, ∞ (excluded) otherwise.
    fn lift_weights(&self) -> [f32; MAX_MAGNETS] {
        let mut w = [f32::INFINITY; MAX_MAGNETS];
        for (i, wi) in w.iter_mut().enumerate() {
            if self.lift_mask & (1u8 << i) != 0 {
                *wi = 1.0;
            }
        }
        w
    }

    /// Rebuilds the gap decomposition for the valid-channel `mask` (lift elements only).
    /// Returns false (keeping the previous decomposition) when the valid channels no
    /// longer determine the controlled modes.
    pub fn rebuild_decomposition(&mut self, mask: u8) -> bool {
        let use_mask = mask & self.lift_mask;
        let mut kt: ModalMap = [[0.0; MAX_MAGNETS]; 3];
        let mut w = [f32::INFINITY; MAX_MAGNETS];
        for (i, (kr, wi)) in self.k_rows.iter().zip(w.iter_mut()).enumerate() {
            if use_mask & (1u8 << i) != 0 {
                *wi = 1.0;
                for (row, v) in kt.iter_mut().zip(kr.iter()) {
                    if let Some(e) = row.get_mut(i) {
                        *e = *v;
                    }
                }
            }
        }
        // Controlled-mode fit (heave, pitch[, roll]).
        let Some(ak) = linalg::weighted_pinv(&kt, &w, self.n_magnets, self.n_modes) else {
            return false;
        };
        let mut d: ModalMap = [[0.0; MAX_MAGNETS]; 3];
        for (i, arow) in ak.iter().enumerate() {
            for (drow, v) in d
                .iter_mut()
                .zip(arow.iter())
                .take(usize::from(self.n_modes))
            {
                if let Some(e) = drow.get_mut(i) {
                    *e = *v;
                }
            }
        }
        // Report roll from the full fit when it is not controlled but observable.
        self.roll_reported = false;
        let full = if self.n_modes < 3 {
            linalg::weighted_pinv(&kt, &w, self.n_magnets, 3)
        } else {
            None
        };
        if let Some(a3) = full {
            for (e, arow) in d[2].iter_mut().zip(a3.iter()) {
                *e = arow[2];
            }
            self.roll_reported = true;
        }
        self.d = d;
        self.d_mask = mask;
        true
    }

    /// Magnet gaps (m) → modal coordinates [heave gap at the CoM projection (absolute,
    /// m), θ_p (rad), θ_r (rad)] from the current decomposition.
    pub fn decompose(&self, gap_m: &[f32; MAX_MAGNETS]) -> [f32; 3] {
        linalg::modal_of(&self.d, gap_m, self.n_magnets)
    }

    /// Per-element gaps predicted by the rigid-body fit of `modal` (fills invalid
    /// channels).
    pub fn predict_gap(&self, modal: &[f32; 3], i: usize) -> f32 {
        self.k_rows.get(i).map_or(modal[0], |k| {
            k[0] * modal[0] + k[1] * modal[1] + k[2] * modal[2]
        })
    }

    /// Modal current demands → per-magnet current references, A.
    pub fn allocate(&self, modal_i_a: &[f32; 3]) -> [f32; MAX_MAGNETS] {
        let mut v = *modal_i_a;
        for (k, x) in v.iter_mut().enumerate() {
            if k >= usize::from(self.n_modes) {
                *x = 0.0;
            }
        }
        linalg::alloc_of(&self.a, &v, self.n_magnets)
    }

    /// Per-magnet currents → modal currents T·i, A.
    pub fn modal_current(&self, coil_i_a: &[f32; MAX_MAGNETS]) -> [f32; 3] {
        linalg::modal_of(&self.t, coil_i_a, self.n_magnets)
    }

    /// One warp-loop step: integrates the null-space part of the measured currents
    /// toward zero and returns the per-magnet correction, A.
    pub fn warp(&mut self, coil_i_a: &[f32; MAX_MAGNETS], dt_s: f32) -> [f32; MAX_MAGNETS] {
        let mut lift_i = [0.0f32; MAX_MAGNETS];
        for (i, (d, s)) in lift_i.iter_mut().zip(coil_i_a.iter()).enumerate() {
            if self.lift_mask & (1u8 << i) != 0 && s.is_finite() {
                *d = *s;
            }
        }
        let nsp = linalg::null_space_part(&self.t, &self.a, &lift_i, self.n_magnets);
        let k = self.warp_rad_s * dt_s;
        for (w, e) in self.warp_i_a.iter_mut().zip(nsp.iter()) {
            *w -= k * e;
        }
        self.warp_i_a
    }

    /// One-sided sub-allocator for pure-EMS hold-down elements (PHYSICS §15): lift
    /// elements carry the allocation; when a clamped lift element leaves a pitch or roll
    /// torque shortfall `tau_short` (N·m, sign as the demanded torque) that pulling DOWN
    /// at a hold-down element would supply, that element gets i = 2g·√(F/(μ₀A_p))/N ≥ 0
    /// (its force F = μ₀A_p·N²·i²/(4g²), M = 0, h′ = 0). Returns per-element currents
    /// (lift entries zero).
    pub fn hold_down(
        &self,
        tau_short_nm: [f32; 2],
        gap_m: &[f32; MAX_MAGNETS],
        mu0_ap: f32,
        turns: f32,
    ) -> [f32; MAX_MAGNETS] {
        let mut out = [0.0f32; MAX_MAGNETS];
        if self.hold_mask == 0 || mu0_ap <= 0.0 || turns <= 0.0 {
            return out;
        }
        // Helpful hold-down elements per torque component (a downward pull F at lever
        // arm r gives the torque −F·r, which helps when τ_short·r < 0).
        let helps = |c: usize, i: usize| -> bool {
            let r = self
                .k_rows
                .get(i)
                .map_or(0.0, |k| if c == 0 { k[1] } else { k[2] });
            let tau = if c == 0 {
                tau_short_nm[0]
            } else {
                tau_short_nm[1]
            };
            self.hold_mask & (1u8 << i) != 0 && tau * r < 0.0 && fmath::absf(r) > 1e-6
        };
        let count = |c: usize| -> f32 {
            (0..usize::from(self.n_magnets))
                .filter(|i| helps(c, *i))
                .count() as f32
        };
        let n_help = [count(0), count(1)];
        for (i, (o, (kr, g))) in out
            .iter_mut()
            .zip(self.k_rows.iter().zip(gap_m.iter()))
            .enumerate()
            .take(usize::from(self.n_magnets))
        {
            if *g <= 0.0 {
                continue;
            }
            let mut f = 0.0f32;
            for (c, (tau, r)) in tau_short_nm.iter().zip([kr[1], kr[2]].iter()).enumerate() {
                let nh = if c == 0 { n_help[0] } else { n_help[1] };
                if helps(c, i) && nh > 0.0 {
                    f = fmath::fmaxf(f, fmath::absf(*tau) / (fmath::absf(*r) * nh));
                }
            }
            if f > 0.0 {
                *o = 2.0 * g * fmath::sqrtf(f / mu0_ap) / turns;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    fn classes() -> [SystemParams; 4] {
        [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ]
    }

    // req: FAB-LEV-009
    #[test]
    fn allocation_identity() {
        for p in classes() {
            let m = Modal::new(&p).expect("preset modal map");
            let k = usize::from(m.n_modes);
            let n = usize::from(m.n_magnets);
            assert_eq!(
                k,
                if p.control.levitation.roll_active {
                    3
                } else {
                    2
                }
            );
            for r in 0..k {
                for c in 0..k {
                    let v: f32 = (0..n).map(|i| m.t[r][i] * m.a[i][c]).sum();
                    let want = if r == c { 1.0 } else { 0.0 };
                    assert!(
                        (v - want).abs() <= 1e-6,
                        "{:?}: (T·A)[{r}][{c}] = {v} (tol 1e-6 f32; LEV-009)",
                        p.design.class
                    );
                }
            }
            // Heave allocation gives every lift element the modal current (design
            // convention of derive::ModeModel).
            let i = m.allocate(&[1.0, 0.0, 0.0]);
            for v in &i[..n] {
                assert!((v - 1.0).abs() < 1e-6, "heave allocation δi = i_h");
            }
            // Decomposition recovers a rigid pose: h = 2 mm, θ_p = 1 mrad, θ_r = −0.5 mrad.
            let mut g = [0.0f32; MAX_MAGNETS];
            let (h, tp, tr) = (2.0e-3f32, 1.0e-3f32, -0.5e-3f32);
            for (i, gi) in g.iter_mut().enumerate().take(n) {
                *gi = h + m.k_rows[i][1] * tp + m.k_rows[i][2] * tr;
            }
            let d = m.decompose(&g);
            assert!((d[0] - h).abs() < 1e-8, "heave gap recovered");
            assert!((d[1] - tp).abs() < 1e-6, "pitch recovered");
            if k == 3 || m.roll_reported {
                assert!((d[2] - tr).abs() < 2e-6, "roll recovered/reported");
            }
        }
    }

    // req: FAB-LEV-009
    #[test]
    fn nullspace_zero_net() {
        for p in classes() {
            let m = Modal::new(&p).expect("preset modal map");
            let n = usize::from(m.n_magnets);
            let k_i = derive::k_i(&p.design.suspension, p.design.suspension.g_star_m, 0.0) as f32;
            // Arbitrary per-element current pattern and its null-space part.
            let mut u = [0.0f32; MAX_MAGNETS];
            for (i, v) in u.iter_mut().enumerate().take(n) {
                *v = 0.7 * (i as f32 + 1.0) - 1.3 * ((i * i) % 3) as f32;
            }
            let nsp = linalg::null_space_part(&m.t, &m.a, &u, m.n_magnets);
            // Net wrench of the null-space currents over the controlled modes: heave
            // force Σk_i·δi, pitch torque Σk_i·r_x·δi, roll torque Σk_i·r_y·δi.
            let scale: f32 = u.iter().map(|v| v.abs()).sum::<f32>() * k_i;
            let wrench = [
                (0..n).map(|i| k_i * nsp[i]).sum::<f32>(),
                (0..n).map(|i| k_i * m.k_rows[i][1] * nsp[i]).sum::<f32>(),
                (0..n).map(|i| k_i * m.k_rows[i][2] * nsp[i]).sum::<f32>(),
            ];
            for (mode, w) in wrench.iter().enumerate().take(usize::from(m.n_modes)) {
                assert!(
                    w.abs() <= 1e-6 * scale,
                    "{:?} mode {mode}: net wrench {w} of null-space currents (tol 1e-6·Σ|F|; LEV-009)",
                    p.design.class
                );
            }
            // The warp loop drives a null-space current offset to zero: with the
            // measured currents = allocation + offset + correction, the null part
            // decays at ω_w (≤ 5 % within 1 s; LEV-009).
            let mut mm = m;
            let mut i_meas = nsp;
            let dt = 1e-4f32;
            for _ in 0..10_000 {
                let w = mm.warp(&i_meas, dt);
                for (im, (off, wi)) in i_meas.iter_mut().zip(nsp.iter().zip(w.iter())) {
                    *im = off + wi;
                }
            }
            let resid = linalg::null_space_part(&mm.t, &mm.a, &i_meas, mm.n_magnets);
            let r: f32 = resid.iter().map(|v| v.abs()).sum();
            let r0: f32 = nsp.iter().map(|v| v.abs()).sum();
            assert!(
                r <= 0.05 * r0,
                "warp loop: null-space current {r} ≤ 5 % of {r0} in 1 s"
            );
        }
    }

    #[test]
    fn decomposition_survives_one_lost_channel() {
        let p = presets::drawer();
        let mut m = Modal::new(&p).expect("drawer");
        assert!(
            m.rebuild_decomposition(0b1110),
            "3 of 4 channels still determine 3 modes"
        );
        let mut g = [0.0f32; MAX_MAGNETS];
        for (i, gi) in g.iter_mut().enumerate().take(4) {
            *gi = 2.5e-3 + m.k_rows[i][1] * 2e-3 + m.k_rows[i][2] * 1e-3;
        }
        g[0] = 99.0; // garbage on the lost channel is ignored
        let d = m.decompose(&g);
        assert!((d[0] - 2.5e-3).abs() < 1e-7 && (d[1] - 2e-3).abs() < 1e-5);
        assert!(
            !m.rebuild_decomposition(0b0011),
            "2 channels cannot determine 3 modes"
        );
    }

    #[test]
    fn hold_down_is_one_sided() {
        // DRAWER variant with two rear pure-EMS hold-down elements (core §C.1 option).
        let mut p = presets::drawer();
        p.design.suspension.n_magnets = 6;
        for (k, x) in [(4usize, -0.28f64), (5usize, -0.28f64)] {
            p.design.suspension.stations[k] = fabrail_params::MagnetStation {
                x_m: x,
                y_m: if k == 4 { 0.1 } else { -0.1 },
                z_m: 0.0,
                normal_sign: -1.0,
                has_pm: false,
            };
        }
        let m = Modal::new(&p).expect("drawer with hold-downs");
        assert_eq!(m.hold_mask, 0b11_0000);
        let i = m.allocate(&[1.0, 0.5, 0.2]);
        assert_eq!(
            i[4], 0.0,
            "hold-down elements are outside the main allocation"
        );
        let g = [2.5e-3f32; MAX_MAGNETS];
        // Rear elements have r_x = −0.14 m: a downward pull there gives the torque
        // −F·r_x > 0, so only a positive pitch-torque shortfall engages them.
        let none = m.hold_down([-5.0, 0.0], &g, 4e-7, 200.0);
        let some = m.hold_down([5.0, 0.0], &g, 4e-7, 200.0);
        assert!(
            none.iter().all(|v| *v == 0.0),
            "wrong-sign shortfall: no current"
        );
        assert!(some[4] > 0.0 && some[5] > 0.0, "one-sided current i ≥ 0");
    }
}
