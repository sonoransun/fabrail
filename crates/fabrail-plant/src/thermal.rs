//! Lumped thermal network: one RC node per coil plus one magnet node (PHYSICS §12). Owner: A6b.
//!
//! Drives the tempco on B_r (−0.12 %/K) and on the coil resistance (+0.393 %/K), and the
//! thermal gap shift (−0.195 / −0.416 / −0.891 mm, core design §C.3).
//!
//! Network (PHYSICS §12; physics perspective §1.10), n active coils k and one lumped
//! magnet node m, ambient T_a (a scenario input):
//!
//! C_c·Ṫ_k = P_k − (T_k − T_a)/R_c − (T_k − T_m)/(n·R_m)
//! C_m·Ṫ_m = Σ_k (T_k − T_m)/(n·R_m) − (T_m − T_a)/R_m
//!
//! with C_c, R_c the per-coil `c_th_coil_j_per_k`, `r_th_coil_k_per_w` and C_m, R_m the
//! magnet node's `c_th_mag_j_per_k`, `r_th_mag_k_per_w`. The coil-to-magnet path reuses
//! R_m (design choice: the magnets sit between the coils and the carriage frame, so with
//! equal paths a steady coil rise ΔT_c lifts the magnets by ΔT_c/2). Values are
//! \[UNCERTAIN\] (core design §C.5; DOOR C_th 150–200 J/K, R_th 2–5 K/W). Tempcos:
//! B_r(T) = B_r,20·(1 + α_Br·(T − 20 °C)), R(T) = R_20·(1 + α_R·(T − 20 °C)).

use fabrail_params::ThermalDesign;
use fabrail_types::MAX_MAGNETS;

/// Reference temperature of the tempcos (B_r and R are specified at 20 °C; core design
/// §C.1; PHYSICS §12), °C.
pub const T_REF_C: f64 = 20.0;

/// Node temperatures (or their rates, K/s, when returned by [`thermal_rates`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ThermalState {
    /// One node per coil, °C.
    pub t_coil_c: [f64; MAX_MAGNETS],
    /// Magnet node, °C.
    pub t_mag_c: f64,
}

impl ThermalState {
    /// Every node at `t_c`.
    pub fn uniform(t_c: f64) -> ThermalState {
        ThermalState {
            t_coil_c: [t_c; MAX_MAGNETS],
            t_mag_c: t_c,
        }
    }

    /// Hottest of the first `n` coil nodes, °C (the telemetry hot-spot).
    pub fn hot_spot_c(&self, n: usize) -> f64 {
        self.t_coil_c
            .iter()
            .take(n.clamp(1, MAX_MAGNETS))
            .fold(f64::NEG_INFINITY, |a, &b| a.max(b))
    }

    /// `self + k·d` over every node (for RK4 over the thermal state).
    pub fn add_scaled(&self, k: f64, d: &ThermalState) -> ThermalState {
        let mut out = *self;
        for (o, di) in out.t_coil_c.iter_mut().zip(d.t_coil_c.iter()) {
            *o += k * di;
        }
        out.t_mag_c += k * d.t_mag_c;
        out
    }
}

impl crate::integrator::OdeState for ThermalState {
    fn add_scaled(&self, k: f64, d: &Self) -> Self {
        ThermalState::add_scaled(self, k, d)
    }
}

/// Thermal conductance 1/R, W/K (0 for a non-positive or non-finite R: adiabatic).
fn conductance(r_k_per_w: f64) -> f64 {
    if r_k_per_w.is_finite() && r_k_per_w > 0.0 {
        1.0 / r_k_per_w
    } else {
        0.0
    }
}

/// Node temperature rates, K/s, for coil copper losses `p_coil_w` (first `n` coils) at
/// ambient `t_amb_c` (PHYSICS §12). Unused coil nodes (index ≥ n) have zero rate; a node
/// with non-positive heat capacity is frozen.
pub fn thermal_rates(
    d: &ThermalDesign,
    s: &ThermalState,
    p_coil_w: &[f64; MAX_MAGNETS],
    t_amb_c: f64,
    n: usize,
) -> ThermalState {
    let n = n.min(MAX_MAGNETS);
    let g_c = conductance(d.r_th_coil_k_per_w);
    let g_m = conductance(d.r_th_mag_k_per_w);
    let g_cm = if n > 0 { g_m / n as f64 } else { 0.0 };
    let inv_cc = if d.c_th_coil_j_per_k > 0.0 {
        1.0 / d.c_th_coil_j_per_k
    } else {
        0.0
    };
    let inv_cm = if d.c_th_mag_j_per_k > 0.0 {
        1.0 / d.c_th_mag_j_per_k
    } else {
        0.0
    };
    let mut out = ThermalState::default();
    let mut q_to_mag = 0.0;
    for ((rate, &tk), &pk) in out
        .t_coil_c
        .iter_mut()
        .zip(s.t_coil_c.iter())
        .zip(p_coil_w.iter())
        .take(n)
    {
        let q_cm = g_cm * (tk - s.t_mag_c);
        q_to_mag += q_cm;
        *rate = inv_cc * (pk - g_c * (tk - t_amb_c) - q_cm);
    }
    out.t_mag_c = inv_cm * (q_to_mag - g_m * (s.t_mag_c - t_amb_c));
    out
}

/// Steady state of the network for constant losses and ambient (PHYSICS §12): solves the
/// n + 1 linear balances in closed form (each coil node is linear in T_m).
pub fn steady_state(
    d: &ThermalDesign,
    p_coil_w: &[f64; MAX_MAGNETS],
    t_amb_c: f64,
    n: usize,
) -> ThermalState {
    let n = n.min(MAX_MAGNETS);
    let g_c = conductance(d.r_th_coil_k_per_w);
    let g_m = conductance(d.r_th_mag_k_per_w);
    let g_cm = if n > 0 { g_m / n as f64 } else { 0.0 };
    let mut out = ThermalState::uniform(t_amb_c);
    let gk = g_c + g_cm;
    if gk <= 0.0 {
        return out;
    }
    // T_k = (P_k + g_c·T_a + g_cm·T_m)/gk; magnet balance Σ g_cm(T_k − T_m) = g_m(T_m − T_a).
    let sum_p: f64 = p_coil_w.iter().take(n).sum();
    let a = g_cm * (g_cm / gk - 1.0) * n as f64 - g_m; // coefficient of T_m
    let b = g_cm * (sum_p + n as f64 * g_c * t_amb_c) / gk + g_m * t_amb_c; // constant
    let t_m = if a != 0.0 { -b / a } else { t_amb_c };
    out.t_mag_c = t_m;
    for (t, &pk) in out.t_coil_c.iter_mut().zip(p_coil_w.iter()).take(n) {
        *t = (pk + g_c * t_amb_c + g_cm * t_m) / gk;
    }
    out
}

/// Remanence at magnet temperature `t_c`: B_r,20·(1 + α_Br·(T − 20)), T (PHYSICS §12;
/// NdFeB α_Br = −0.0012 /K).
pub fn br_at(b_r20_t: f64, tempco_per_k: f64, t_c: f64) -> f64 {
    b_r20_t * (1.0 + tempco_per_k * (t_c - T_REF_C))
}

/// Coil resistance at `t_c`: R_20·(1 + α_R·(T − 20)), Ω (PHYSICS §12; copper
/// α_R = +0.00393 /K).
pub fn r_at(r20_ohm: f64, tempco_per_k: f64, t_c: f64) -> f64 {
    r20_ohm * (1.0 + tempco_per_k * (t_c - T_REF_C))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::integrator::rk4_step;
    use fabrail_params::presets;

    #[test]
    fn hot_spot_ignores_unused_nodes() {
        let mut s = ThermalState::uniform(20.0);
        s.t_coil_c[1] = 35.0;
        s.t_coil_c[6] = 90.0;
        assert_eq!(s.hot_spot_c(4), 35.0);
        assert_eq!(s.hot_spot_c(0), 20.0);
    }

    #[test]
    fn energy_balance_of_the_rates() {
        // Σ C·Ṫ = Σ P − losses to ambient ((E) rel 1e-12, PHYSICS §12).
        let d = presets::door().design.thermal;
        let mut s = ThermalState::uniform(25.0);
        s.t_coil_c[..4].copy_from_slice(&[40.0, 38.0, 30.0, 45.0]);
        s.t_mag_c = 31.0;
        let mut p = [0.0; MAX_MAGNETS];
        p[..4].copy_from_slice(&[2.0, 1.5, 0.5, 3.0]);
        let r = thermal_rates(&d, &s, &p, 22.0, 4);
        let stored: f64 = r.t_coil_c[..4].iter().sum::<f64>() * d.c_th_coil_j_per_k
            + r.t_mag_c * d.c_th_mag_j_per_k;
        let out: f64 = s.t_coil_c[..4]
            .iter()
            .map(|t| (t - 22.0) / d.r_th_coil_k_per_w)
            .sum::<f64>()
            + (s.t_mag_c - 22.0) / d.r_th_mag_k_per_w;
        let want = 7.0 - out;
        assert!((stored - want).abs() < 1e-12 * 10.0, "{stored} vs {want}");
        assert_eq!(r.t_coil_c[4..], [0.0; 4], "unused nodes frozen");
    }

    #[test]
    fn integrates_to_the_steady_state_and_halves_the_magnet_rise() {
        // Uniform 1 W per coil on the DRAWER network: RK4 over 5 h reaches the closed-form
        // steady state ((I) 1e-6 K), and the magnet sits halfway between coil and ambient.
        let d = presets::drawer().design.thermal;
        let mut p = [0.0; MAX_MAGNETS];
        p[..4].fill(1.0);
        let ss = steady_state(&d, &p, 20.0, 4);
        let mut s = ThermalState::uniform(20.0);
        let h = 1.0;
        for k in 0..18_000u32 {
            s = rk4_step(&s, f64::from(k) * h, h, |_, _, st| {
                thermal_rates(&d, st, &p, 20.0, 4)
            });
        }
        for k in 0..4 {
            assert!((s.t_coil_c[k] - ss.t_coil_c[k]).abs() < 1e-6, "coil {k}");
        }
        assert!((s.t_mag_c - ss.t_mag_c).abs() < 1e-6);
        assert!(((ss.t_mag_c - 20.0) - 0.5 * (ss.t_coil_c[0] - 20.0)).abs() < 1e-9);
        // Rates vanish at the steady state.
        let r = thermal_rates(&d, &ss, &p, 20.0, 4);
        assert!(r.t_mag_c.abs() < 1e-12 && r.t_coil_c[0].abs() < 1e-12);
    }

    #[test]
    fn tempcos() {
        // B_r at 60 °C: 1.3·(1 − 0.0012·40) = 1.2376 T; R at 60 °C: 1.6·(1 + 0.00393·40)
        // = 1.85152 Ω ((E), PHYSICS §12).
        assert!((br_at(1.3, -0.0012, 60.0) - 1.2376).abs() < 1e-12);
        assert!((r_at(1.6, 0.00393, 60.0) - 1.85152).abs() < 1e-12);
        assert_eq!(br_at(1.3, -0.0012, T_REF_C), 1.3);
    }
}
