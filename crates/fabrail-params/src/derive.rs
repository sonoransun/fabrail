//! f64 closed forms (core design §B.2; plan §2b, §4b). Each function cites the PHYSICS §
//! that defines it; the analytic anchors of core design §E.5 and plan §4/§4b are asserted
//! against these (tolerance classes of core design §E.1; every assert message states its
//! tolerance and source).
//!
//! Units: SI throughout (m, kg, s, A, N, T); `r_km`/`*_kmps` where the seismology
//! convention is km. All transcendental math goes through `fabrail_types::fmath`.
//!
//! Levitation gain design (PHYSICS §15; plan §2b item 1): [`zp_gains`] is the
//! continuous-time reference; [`zp_gains_discrete`] is what the controller uses. The
//! discrete design places the three target poles of (s² + 2ζωs + ω²)(s + αω), mapped by
//! z = e^{sT}, on a six-state sampled loop: the ZOH plant (gap, gap rate, coil current
//! behind the current-loop lag τ_cl, force flux behind the eddy lag τ_e), the Mizuno
//! current integral `q[k+1] = q[k] + T·i[k]`, and one fast period of LET delay on the
//! command. The same model is exposed by [`zp_closed_loop_poles`] for LEV-007/LEV-020 and
//! the `gain_margin` study ([`zp_critical_tau_e`]).

use crate::{DesignParams, SuspensionDesign, SystemParams};
use core::f64::consts::{LN_2, PI};
use fabrail_types::{RestPolicy, fmath};

/// Standard acceleration of gravity g₀, m/s². Source: 3rd CGPM (1901); ISO 80000-3.
pub const G0_MPS2: f64 = 9.806_65;

/// Vacuum magnetic permeability μ₀, H/m. Source: CODATA 2018 recommended value.
pub const MU0_H_PER_M: f64 = 1.256_637_062_12e-6;

/// Density of dry air at 20 °C and 101.325 kPa, kg/m³. Source: ISO 2533 / CRC Handbook.
pub const RHO_AIR_KG_PER_M3: f64 = 1.204;

/// Discharge coefficient of a sharp-edged leak orifice (the cabinet reveal), used by
/// [`piston_force`]. Source: ISO 5167-2 sharp-edged orifice C ≈ 0.6; core research
/// journal item C15 (40 cm² reveal → 94 Pa → 5.6 N at 0.5 m/s).
pub const ORIFICE_CD: f64 = 0.6;

/// On-site decision time after the P arrival (trigger plus the Pd/τc window), s.
/// Source: core research journal item C7 (usable on-site time 0.119·R − 1.2 s);
/// PHYSICS §20.
pub const ONSITE_DECISION_S: f64 = 1.2;

/// Network alert latency after origin time, s. Source: core research journal item C7
/// (ShakeAlert first alert for Ridgecrest 2019 at 6.9 s after origin; 8 s used for the
/// 40.8 km crossover); PHYSICS §20.
pub const NETWORK_ALERT_S: f64 = 8.0;

/// Smallest fringing factor f(g, y) accepted by the lateral model; beyond it the
/// small-offset expansion is invalid (PHYSICS §2).
const FRINGE_F_MIN: f64 = 1.0e-3;

/// Controlled rigid-body mode for gain design (PHYSICS §15).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum LevMode {
    /// Vertical translation.
    #[default]
    Heave,
    /// Rotation about y.
    Pitch,
    /// Rotation about x (active on DRAWER and HEAVY only).
    Roll,
}

impl LevMode {
    /// Every mode, in design order.
    pub const ALL: [LevMode; 3] = [LevMode::Heave, LevMode::Pitch, LevMode::Roll];
}

/// Zero-power (Mizuno) gains per magnet for one mode (PHYSICS §15).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ZpGains {
    /// Gap gain k_p, A/m.
    pub kp_a_per_m: f64,
    /// Gap-rate gain k_d, A·s/m.
    pub kd_a_s_per_m: f64,
    /// Current-integral gain κ, 1/s.
    pub kappa_per_s: f64,
}

/// Direction of a control-loss failure (LEV-019).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum FailDir {
    /// Snap up to the upper stop g_up.
    #[default]
    Up,
    /// Drop to the landing rollers g_land.
    Down,
}

/// Result of [`size_magnets`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MagnetSize {
    /// Pole face area A_p, m².
    pub a_p_m2: f64,
    /// Magnet thickness h_m, m.
    pub h_m_m: f64,
}

/// Log-damped terminus blend (PHYSICS §16.1).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TerminusBlend {
    /// Blend duration t_b, s.
    pub t_b_s: f64,
    /// Distance-to-go at the end of the blend d_b, m.
    pub d_b_m: f64,
    /// Distance-to-go at blend start d_0, m.
    pub d_0_m: f64,
}

/// DRAWER extension envelope at a given mass (LEV-016; plan §2b item 3).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ExtensionCapacity {
    /// Maximum contactless extension as a fraction of the body length.
    pub max_extension_frac: f64,
    /// The requested (preset) travel would saturate an element.
    pub saturated: bool,
}

/// Hold-up energy budget (SAF-009; PHYSICS §14).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HoldupEnergy {
    /// Energy one Safe Landing needs (seat-first for DRAWER), J.
    pub need_j: f64,
    /// Usable supercap energy η·½C(V_max² − V_min²), J.
    pub avail_j: f64,
}

// ---- Magnetics and levitation (PHYSICS §2, §3) -----------------------------------------

/// ln cosh(u), overflow-free: |u| + ln(1 + e^{−2|u|}) − ln 2.
fn ln_cosh(u: f64) -> f64 {
    let a = fmath::abs(u);
    a + fmath::ln_1p(fmath::exp(-2.0 * a)) - LN_2
}

/// Lateral fringing factor f(g, y) = 1 − (g/w)·ln cosh(y/g), floored at 1e-3
/// (PHYSICS §2).
fn fringe_f(g_m: f64, y_m: f64, pole_w_m: f64) -> f64 {
    if g_m <= 0.0 || pole_w_m <= 0.0 {
        return 1.0;
    }
    fmath::fmax(1.0 - (g_m / pole_w_m) * ln_cosh(y_m / g_m), FRINGE_F_MIN)
}

/// Effective gap with lateral fringing: g/f(g, y), f = 1 − (g/w)·ln cosh(y/g)
/// (PHYSICS §2), m.
pub fn g_eff(g_m: f64, y_m: f64, pole_w_m: f64) -> f64 {
    g_m / fringe_f(g_m, y_m, pole_w_m)
}

/// Magnet reluctance length h′ = h_m/μ_rec (PHYSICS §2), m.
pub fn h_prime(h_m_m: f64, mu_rec: f64) -> f64 {
    if mu_rec <= 0.0 {
        return 0.0;
    }
    h_m_m / mu_rec
}

/// Magnet MMF M_pm = B_r·h_m/(μ₀·μ_rec) (PHYSICS §2), A-turns.
pub fn m_pm(b_r_t: f64, h_m_m: f64, mu_rec: f64) -> f64 {
    if mu_rec <= 0.0 {
        return 0.0;
    }
    b_r_t * h_m_m / (MU0_H_PER_M * mu_rec)
}

/// Pole face area A_p = w·l of one element, m².
fn pole_area(s: &SuspensionDesign) -> f64 {
    s.pole_w_m * s.pole_l_m
}

/// Total element MMF M_pm + N·i, A-turns.
fn mmf(s: &SuspensionDesign, i_a: f64) -> f64 {
    m_pm(s.b_r_t, s.h_m_m, s.mu_rec) + s.turns * i_a
}

/// Number of lift elements carrying the weight: active stations with a PM and an upward
/// normal (pure-EMS hold-down elements are excluded); `n_magnets` if none are flagged.
pub fn n_lift(s: &SuspensionDesign) -> f64 {
    let n = core::cmp::min(usize::from(s.n_magnets), s.stations.len());
    let count = s
        .stations
        .iter()
        .take(n)
        .filter(|st| st.normal_sign > 0.0 && st.has_pm)
        .count();
    if count > 0 {
        count as f64
    } else {
        f64::from(core::cmp::max(s.n_magnets, 1))
    }
}

/// Vertical attractive force of one element from the co-energy model at σ_leak = 0 and
/// full engagement (saturation is a plant effect) (PHYSICS §2–§3), N:
/// F_z = −∂W′/∂g with W′ = ½(M_pm + N·i)²·P, P = μ₀A_p/D, D = 2g/f(g, y) + h′, i.e.
/// F_z = ½(M_pm + N·i)²·μ₀A_p·D′/D² with D′ = 2/f − 2g·f′/f² and
/// f′ = ∂f/∂g = −(1/w)·ln cosh(y/g) + (y/(w·g))·tanh(y/g).
/// At y = 0 this is the closed form μ₀·A_p·(M_pm + N·i)²/(2g + h′)².
pub fn pm_force(s: &SuspensionDesign, g_m: f64, y_m: f64, i_a: f64) -> f64 {
    let a_p = pole_area(s);
    let hp = h_prime(s.h_m_m, s.mu_rec);
    let f_mmf = mmf(s, i_a);
    if y_m == 0.0 {
        let d = 2.0 * g_m + hp;
        return MU0_H_PER_M * a_p * f_mmf * f_mmf / (d * d);
    }
    let w = s.pole_w_m;
    let f = fringe_f(g_m, y_m, w);
    let u = y_m / g_m;
    let df = if w > 0.0 && g_m > 0.0 {
        -(1.0 / w) * ln_cosh(u) + (y_m / (w * g_m)) * fmath::tanh(u)
    } else {
        0.0
    };
    let d = 2.0 * g_m / f + hp;
    let d_prime = 2.0 / f - 2.0 * g_m * df / (f * f);
    0.5 * f_mmf * f_mmf * MU0_H_PER_M * a_p * d_prime / (d * d)
}

/// Open-loop heave pole of the zero-power element p = √(4·g₀/(2g* + h′)) (PHYSICS §3;
/// LEV-003 anchors 69.49 / 47.57 / 32.50 rad/s), rad/s.
pub fn open_loop_pole(g_star_m: f64, h_prime_m: f64) -> f64 {
    let d = 2.0 * g_star_m + h_prime_m;
    if d <= 0.0 {
        return 0.0;
    }
    fmath::sqrt(4.0 * G0_MPS2 / d)
}

/// Negative (destabilising) stiffness of one element k_s = −∂F/∂g
/// = 4·μ₀·A_p·(M_pm + N·i)²/(2g + h′)³ (PHYSICS §3), N/m.
pub fn k_s(s: &SuspensionDesign, g_m: f64, i_a: f64) -> f64 {
    let d = 2.0 * g_m + h_prime(s.h_m_m, s.mu_rec);
    let f_mmf = mmf(s, i_a);
    4.0 * MU0_H_PER_M * pole_area(s) * f_mmf * f_mmf / (d * d * d)
}

/// Current gain of one element k_i = ∂F/∂i = 2·N·μ₀·A_p·(M_pm + N·i)/(2g + h′)²
/// (PHYSICS §3; DRAWER 7.9 N/A), N/A.
pub fn k_i(s: &SuspensionDesign, g_m: f64, i_a: f64) -> f64 {
    let d = 2.0 * g_m + h_prime(s.h_m_m, s.mu_rec);
    2.0 * s.turns * MU0_H_PER_M * pole_area(s) * mmf(s, i_a) / (d * d)
}

/// Zero-power mass at gap g: m(g) = m*·((2g* + h′)/(2g + h′))² (PHYSICS §3; LEV-005
/// DRAWER 1.5 → 45.76, 5.0 → 9.96, 6.2 → 7.12 kg), kg.
pub fn zero_power_mass(gap_m: f64, g_star_m: f64, h_prime_m: f64, m_star_kg: f64) -> f64 {
    let r = (2.0 * g_star_m + h_prime_m) / (2.0 * gap_m + h_prime_m);
    m_star_kg * r * r
}

/// Zero-power gap at mass m: g = ((2g* + h′)·√(m*/m) − h′)/2 (PHYSICS §3; DOOR +10 kg
/// (98.1 N at 9.81) → 4.5967 mm), m.
pub fn zero_power_gap(m_kg: f64, g_star_m: f64, h_prime_m: f64, m_star_kg: f64) -> f64 {
    if m_kg <= 0.0 {
        return f64::INFINITY;
    }
    ((2.0 * g_star_m + h_prime_m) * fmath::sqrt(m_star_kg / m_kg) - h_prime_m) / 2.0
}

/// Bias current that holds mass m at gap g with all n lift elements equal:
/// i = ((2g + h′)·√(m·g₀/(n·μ₀·A_p)) − M_pm)/N (PHYSICS §3, §15; DRAWER seat 3.5 mm:
/// +11.20 A at 48 kg, −4.99 A at 8 kg with the balanced pole; from 6.5 mm 27.4 A is
/// infeasible, F17), A.
pub fn bias_current(d: &DesignParams, gap_m: f64, m_kg: f64) -> f64 {
    let s = &d.suspension;
    let n = n_lift(s);
    let a_p = pole_area(s);
    if a_p <= 0.0 || s.turns <= 0.0 || m_kg < 0.0 {
        return 0.0;
    }
    let hp = h_prime(s.h_m_m, s.mu_rec);
    let need = (2.0 * gap_m + hp) * fmath::sqrt(m_kg * G0_MPS2 / (n * MU0_H_PER_M * a_p));
    (need - m_pm(s.b_r_t, s.h_m_m, s.mu_rec)) / s.turns
}

/// Magnet-only force ratio n·F_pm(g, 0)/(m·g₀) (PHYSICS §3; LEV-006 stop and stay-landed
/// rules: ≤ 1.30 at g_up with m*, ≤ 0.90 at g_land with m_op_min, F18).
pub fn stop_ratio(d: &DesignParams, gap_m: f64, m_kg: f64) -> f64 {
    if m_kg <= 0.0 {
        return f64::INFINITY;
    }
    let s = &d.suspension;
    n_lift(s) * pm_force(s, gap_m, 0.0, 0.0) / (m_kg * G0_MPS2)
}

/// Sizes the magnets so F(g*) = m*·g₀/n at gap flux B_g: A_p = μ₀·m*·g₀/(n·B_g²) and
/// h_m = 2g*·B_g·μ_rec/(B_r − B_g) (PHYSICS §3; core design §C.2 row "B_g / pole / h_m").
pub fn size_magnets(
    m_star_kg: f64,
    n_magnets: f64,
    g_star_m: f64,
    b_g_t: f64,
    b_r_t: f64,
    mu_rec: f64,
) -> MagnetSize {
    if n_magnets <= 0.0 || b_g_t <= 0.0 || b_r_t <= b_g_t {
        return MagnetSize::default();
    }
    MagnetSize {
        a_p_m2: MU0_H_PER_M * m_star_kg * G0_MPS2 / (n_magnets * b_g_t * b_g_t),
        h_m_m: 2.0 * g_star_m * b_g_t * mu_rec / (b_r_t - b_g_t),
    }
}

/// Pole length that balances the PM force exactly at g* for m*:
/// l = m*·g₀·(2g* + h′)²/(n·μ₀·M_pm²·w) (PHYSICS §3). The presets use it after rounding
/// h_m to a manufacturable thickness, so zero-power equilibrium sits exactly at g* for m*
/// (LEV-004 gap tracking at ±2 µm needs the balance, not the rounded 10×32 mm pole).
pub fn balance_pole_length(s: &SuspensionDesign, m_star_kg: f64) -> f64 {
    let hp = h_prime(s.h_m_m, s.mu_rec);
    let m = m_pm(s.b_r_t, s.h_m_m, s.mu_rec);
    let n = n_lift(s);
    if m <= 0.0 || s.pole_w_m <= 0.0 {
        return 0.0;
    }
    let d = 2.0 * s.g_star_m + hp;
    m_star_kg * G0_MPS2 * d * d / (n * MU0_H_PER_M * m * m * s.pole_w_m)
}

/// Thermal zero-power gap shift for ΔT through the B_r tempco:
/// Δg = (2g* + h′)·α_Br·ΔT/2 (PHYSICS §12; LEV-014 +40 K: −0.195 / −0.416 / −0.891 mm), m.
pub fn thermal_gap_shift(p: &SystemParams, d_t_k: f64) -> f64 {
    let s = &p.design.suspension;
    (2.0 * s.g_star_m + h_prime(s.h_m_m, s.mu_rec)) * s.tempco_br_per_k * d_t_k / 2.0
}

/// PM-only work of the n lift elements between gaps g_a ≤ g_b:
/// ∫ n·F dg = n·μ₀A_p·M²/2·(1/(2g_a + h′) − 1/(2g_b + h′)) (PHYSICS §3), J.
fn pm_work(s: &SuspensionDesign, g_a: f64, g_b: f64) -> f64 {
    let hp = h_prime(s.h_m_m, s.mu_rec);
    let m = m_pm(s.b_r_t, s.h_m_m, s.mu_rec);
    0.5 * n_lift(s)
        * MU0_H_PER_M
        * pole_area(s)
        * m
        * m
        * (1.0 / (2.0 * g_a + hp) - 1.0 / (2.0 * g_b + hp))
}

/// Impact speed of a control loss (coil current 0, no lag, no dissipation) for mass m
/// starting at rest at gap g₀ (PHYSICS §15; LEV-019): up to g_up,
/// ½mv² = ∫_{g_up}^{g₀} n·F dg − m·g₀·(g₀ − g_up); down to g_land,
/// ½mv² = m·g₀·(g_land − g₀) − ∫_{g₀}^{g_land} n·F dg. Returns 0 if the stop is not
/// reached. Anchors (physics perspective A7): DOOR 50.6 / 55.6 mm/s, WALL 68.8 / 76.3,
/// DRAWER at 26 kg 109.6 / 197.3; empty DRAWER (8 kg from its 5.761 mm float) 0.38 m/s up.
pub fn impact_speed_at(p: &SystemParams, dir: FailDir, m_kg: f64, g_start_m: f64) -> f64 {
    let s = &p.design.suspension;
    if m_kg <= 0.0 {
        return 0.0;
    }
    let w = m_kg * G0_MPS2;
    let e = match dir {
        FailDir::Up => pm_work(s, s.g_up_m, g_start_m) - w * (g_start_m - s.g_up_m),
        FailDir::Down => w * (s.g_land_m - g_start_m) - pm_work(s, g_start_m, s.g_land_m),
    };
    if e <= 0.0 {
        0.0
    } else {
        fmath::sqrt(2.0 * e / m_kg)
    }
}

/// Impact speed after a control loss at g* with m* (the unstable zero-power equilibrium;
/// PHYSICS §15; LEV-019), m/s. See [`impact_speed_at`].
pub fn impact_speed(p: &SystemParams, dir: FailDir) -> f64 {
    let s = &p.design.suspension;
    impact_speed_at(p, dir, p.design.carriage.m_star_kg, s.g_star_m)
}

/// Maximum continuous lift of one fully engaged element: F at the band's smallest gap
/// with i_cont, capped at the saturation force B_sat²·A_p/μ₀ (PHYSICS §2, §15), N.
pub fn element_force_max(s: &SuspensionDesign) -> f64 {
    let g = if s.zp_band_m[0] > 0.0 {
        s.zp_band_m[0]
    } else {
        s.g_star_m
    };
    let f = pm_force(s, g, 0.0, s.i_cont_a);
    let f_sat = s.b_sat_t * s.b_sat_t * pole_area(s) / MU0_H_PER_M;
    if s.b_sat_t > 0.0 {
        fmath::fmin(f, f_sat)
    } else {
        f
    }
}

/// DRAWER contactless extension envelope at mass m (LEV-016; plan §2b item 3;
/// PHYSICS §5, §15).
///
/// Model: elements ride on the drawer; the cabinet rail spans the body length L, so at
/// extension s its front end sits at x = L/2 − s in the carriage frame (origin at the
/// body centre, +x toward the front face). An element at x_i with pole length l is
/// engaged by e_i(s) = clamp((L/2 − s − x_i + l/2)/l, 0, 1). The weight m·g₀ at the CoM
/// x_c is shared statically by the front (max x) and rear (min x) lift groups; group k
/// needs F_k per element and supplies e_k(s)·F_max ([`element_force_max`]). The maximum
/// extension is the largest s with F_k ≤ e_k(s)·F_max in both groups, i.e.
/// s_k = L/2 − x_k + l/2 − (F_k/F_max)·l; `saturated` is set when the preset travel
/// exceeds it (or no extension is feasible: CoM outside the support span, or F_k > F_max).
pub fn extension_capacity(p: &SystemParams, m_kg: f64) -> ExtensionCapacity {
    let d = &p.design;
    let s = &d.suspension;
    let len = d.carriage.length_m;
    let l = s.pole_l_m;
    let infeasible = ExtensionCapacity {
        max_extension_frac: 0.0,
        saturated: true,
    };
    if len <= 0.0 || l <= 0.0 || m_kg <= 0.0 {
        return infeasible;
    }
    let n = core::cmp::min(usize::from(s.n_magnets), s.stations.len());
    let lift = || s.stations.iter().take(n).filter(|st| st.normal_sign > 0.0);
    let x_front = lift().map(|st| st.x_m).fold(f64::NEG_INFINITY, fmath::fmax);
    let x_rear = lift().map(|st| st.x_m).fold(f64::INFINITY, fmath::fmin);
    if !x_front.is_finite() || !x_rear.is_finite() {
        return infeasible;
    }
    let n_front = lift().filter(|st| st.x_m == x_front).count() as f64;
    let n_rear = lift().filter(|st| st.x_m == x_rear).count() as f64;
    let w = m_kg * G0_MPS2;
    let x_c = d.carriage.cg_m[0];
    let (f_front, f_rear) = if x_front - x_rear > 0.0 {
        let span = x_front - x_rear;
        (
            w * (x_c - x_rear) / (span * n_front),
            w * (x_front - x_c) / (span * n_rear),
        )
    } else {
        // Every lift element at one x: a single group carries the weight.
        let per = w / n_front;
        (per, per)
    };
    let f_max = element_force_max(s);
    if f_front < 0.0 || f_rear < 0.0 || f_front > f_max || f_rear > f_max || f_max <= 0.0 {
        return infeasible;
    }
    let reach = |x_k: f64, f_k: f64| len / 2.0 - x_k + l / 2.0 - (f_k / f_max) * l;
    let s_max = fmath::clamp(
        fmath::fmin(reach(x_front, f_front), reach(x_rear, f_rear)),
        0.0,
        len,
    );
    ExtensionCapacity {
        max_extension_frac: s_max / len,
        saturated: d.rail.travel_m > s_max,
    }
}

// ---- Levitation control (PHYSICS §15) ---------------------------------------------------

/// Per-element modal quantities of one rigid-body mode (PHYSICS §15).
///
/// Modal coordinate convention: heave u = δg (all elements move together, allocation
/// δi_i = i_mode); pitch/roll u = r·θ with lever arms r_i = x_i − x_c (pitch) or
/// y_i − y_c (roll) and allocation δi_i = (r_i/r)·i_mode, r = √(Σr_i²/n). The modal
/// equation is m_eff·ü = k_eff·u − k_i·i_mode with m_eff = J/Σr_i² and
/// k_eff = (k_s·Σr_i² + m*·g₀·h_cg)/Σr_i², h_cg the CoM height above the pole-face plane
/// (a CoM below the elements, as on top-hung leaves, stabilises: DOOR roll 51 vs
/// 1030 N·m/rad). Heave: m_eff = m*/n, k_eff = k_s(g*, 0).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModeModel {
    /// Effective mass per element, kg.
    pub m_eff_kg: f64,
    /// Effective negative stiffness per element (≤ 0 means the mode is passively
    /// stable and gets no zero-power loop), N/m.
    pub k_eff_n_per_m: f64,
    /// Current gain per element k_i(g*, 0), N/A.
    pub k_i_n_per_a: f64,
}

impl ModeModel {
    /// Open-loop pole √(k_eff/m_eff), or 0 for a passive mode, rad/s.
    pub fn pole_rad_s(&self) -> f64 {
        if self.k_eff_n_per_m <= 0.0 || self.m_eff_kg <= 0.0 {
            0.0
        } else {
            fmath::sqrt(self.k_eff_n_per_m / self.m_eff_kg)
        }
    }
}

/// Modal model of `mode` at (g*, i = 0) with m* (PHYSICS §15). See [`ModeModel`].
pub fn mode_model(p: &SystemParams, mode: LevMode) -> ModeModel {
    let d = &p.design;
    let s = &d.suspension;
    let c = &d.carriage;
    let n = core::cmp::min(usize::from(s.n_magnets), s.stations.len());
    let ks = k_s(s, s.g_star_m, 0.0);
    let ki = k_i(s, s.g_star_m, 0.0);
    let n_l = n_lift(s);
    if matches!(mode, LevMode::Heave) {
        return ModeModel {
            m_eff_kg: c.m_star_kg / n_l,
            k_eff_n_per_m: ks,
            k_i_n_per_a: ki,
        };
    }
    let lift = s.stations.iter().take(n).filter(|st| st.normal_sign > 0.0);
    let (sum_r2, z_sum, count) = lift.fold((0.0, 0.0, 0.0), |(acc, zs, cnt), st| {
        let r = match mode {
            LevMode::Pitch => st.x_m - c.cg_m[0],
            _ => st.y_m - c.cg_m[1],
        };
        (acc + r * r, zs + st.z_m, cnt + 1.0)
    });
    if sum_r2 <= 0.0 || count <= 0.0 {
        return ModeModel::default();
    }
    let h_cg = c.cg_m[2] - z_sum / count;
    let j = match mode {
        LevMode::Pitch => c.j_kgm2[1],
        _ => c.j_kgm2[0],
    };
    ModeModel {
        m_eff_kg: j / sum_r2,
        k_eff_n_per_m: (ks * sum_r2 + c.m_star_kg * G0_MPS2 * h_cg) / sum_r2,
        k_i_n_per_a: ki,
    }
}

/// Open-loop pole of `mode` (0 for a passively stable mode), rad/s (PHYSICS §15).
pub fn mode_pole(p: &SystemParams, mode: LevMode) -> f64 {
    mode_model(p, mode).pole_rad_s()
}

/// True when `mode` gets a zero-power loop: heave and pitch always; roll only when
/// `levitation.roll_active` (DRAWER, HEAVY). A mode with no unstable pole gets none.
pub fn mode_active(p: &SystemParams, mode: LevMode) -> bool {
    let wanted = match mode {
        LevMode::Roll => p.control.levitation.roll_active,
        _ => true,
    };
    wanted && mode_pole(p, mode) > 0.0
}

/// Target natural frequency ω = omega_ratio·max(p_mode, p_heave) (PHYSICS §15; plan §2b
/// item 1: ratio 2.5), rad/s.
pub fn zp_omega(p: &SystemParams, mode: LevMode) -> f64 {
    let ph = mode_pole(p, LevMode::Heave);
    p.control.levitation.omega_ratio * fmath::fmax(mode_pole(p, mode), ph)
}

/// Continuous-time reference gains: pole placement of (s² + 2ζωs + ω²)(s + αω) with
/// ω = omega_ratio·max(p_mode, p_heave) (PHYSICS §15). With b = k_i/m_eff and
/// p² = k_eff/m_eff the loop polynomial is s³ + (b·k_d − κ)s² + (b·k_p − p²)s + p²κ, so
/// κ = αω³/p², k_d = ((α + 2ζ)ω + κ)/b, k_p = ((1 + 2ζα)ω² + p²)/b. Reference only; the
/// controller uses [`zp_gains_discrete`]. Passive or inactive modes return zero gains.
pub fn zp_gains(p: &SystemParams, mode: LevMode) -> ZpGains {
    if !mode_active(p, mode) {
        return ZpGains::default();
    }
    let mm = mode_model(p, mode);
    let lt = &p.control.levitation;
    let w = zp_omega(p, mode);
    let p2 = mm.k_eff_n_per_m / mm.m_eff_kg;
    let b = mm.k_i_n_per_a / mm.m_eff_kg;
    if b <= 0.0 || p2 <= 0.0 {
        return ZpGains::default();
    }
    let kappa = lt.alpha * w * w * w / p2;
    ZpGains {
        kp_a_per_m: ((1.0 + 2.0 * lt.zeta * lt.alpha) * w * w + p2) / b,
        kd_a_s_per_m: ((lt.alpha + 2.0 * lt.zeta) * w + kappa) / b,
        kappa_per_s: kappa,
    }
}

/// Fast-group period T = fast_decim/base_hz (core design §A.5), s.
pub fn fast_period_s(p: &SystemParams) -> f64 {
    let r = &p.control.rates;
    if r.base_hz == 0 {
        return 0.0;
    }
    f64::from(r.fast_decim) / f64::from(r.base_hz)
}

/// Current-loop time constant τ_cl = 1/(2π·current_bw_hz) (plan §2b item 2: 1 kHz →
/// 0.159 ms), s.
pub fn tau_current_loop_s(p: &SystemParams) -> f64 {
    let bw = p.control.levitation.current_bw_hz;
    if bw <= 0.0 {
        0.0
    } else {
        1.0 / (2.0 * PI * bw)
    }
}

/// Target discrete poles z = e^{sT} of (s² + 2ζωs + ω²)(s + αω) for `mode`, as
/// (re, im) pairs: the upper pole of the pair first, then its partner, then αω.
pub fn zp_target_poles_z(p: &SystemParams, mode: LevMode) -> [(f64, f64); 3] {
    let t = fast_period_s(p);
    let s = target_poles_s(p, mode);
    let mut out = [(0.0, 0.0); 3];
    for (o, si) in out.iter_mut().zip(s.iter()) {
        let z = si.scale(t).cexp();
        *o = (z.re, z.im);
    }
    out
}

/// Continuous target poles in s.
fn target_poles_s(p: &SystemParams, mode: LevMode) -> [lin::C; 3] {
    let lt = &p.control.levitation;
    let w = zp_omega(p, mode);
    let z = lt.zeta;
    let pair = if z < 1.0 {
        let wd = w * fmath::sqrt(1.0 - z * z);
        [lin::C::new(-z * w, wd), lin::C::new(-z * w, -wd)]
    } else {
        let r = w * fmath::sqrt(z * z - 1.0);
        [lin::C::new(-z * w + r, 0.0), lin::C::new(-z * w - r, 0.0)]
    };
    [pair[0], pair[1], lin::C::new(-lt.alpha * w, 0.0)]
}

/// Sampled design model of one mode: Φ, Γ of the 4-state ZOH plant (δg, δġ, i, i_e).
fn zoh_plant(mm: &ModeModel, t: f64, tau_cl: f64, tau_e: f64) -> ([[f64; 4]; 4], [f64; 4]) {
    let p2 = mm.k_eff_n_per_m / mm.m_eff_kg;
    let b = mm.k_i_n_per_a / mm.m_eff_kg;
    // Augmented [[A, B], [0, 0]]·T, exponentiated (Van Loan): top-right column is Γ.
    let mut a = [[0.0f64; 5]; 5];
    a[0][1] = 1.0;
    a[1][0] = p2;
    a[2][2] = -1.0 / tau_cl;
    a[2][4] = 1.0 / tau_cl;
    if tau_e > 0.0 {
        a[1][3] = -b;
        a[3][2] = 1.0 / tau_e;
        a[3][3] = -1.0 / tau_e;
    } else {
        // No eddy lag: force follows the coil current; the flux state is a decoupled,
        // fast, stable dummy.
        a[1][2] = -b;
        a[3][3] = -1.0 / tau_cl;
    }
    for row in a.iter_mut() {
        for v in row.iter_mut() {
            *v *= t;
        }
    }
    let e = lin::expm(a);
    let mut phi = [[0.0f64; 4]; 4];
    let mut gam = [0.0f64; 4];
    for (r, (prow, g)) in phi.iter_mut().zip(gam.iter_mut()).enumerate() {
        prow.copy_from_slice(&e[r][..4]);
        *g = e[r][4];
    }
    (phi, gam)
}

/// Six-state closed loop x = [δg, δġ, i, i_e, q, u_d]: plant rows Φx + Γ·u_d, the Mizuno
/// integral q⁺ = q + T·i and the LET-delayed command u_d⁺ = k_p·δg + k_d·δġ + κ·q.
fn closed_loop(phi: &[[f64; 4]; 4], gam: &[f64; 4], t: f64, k: &ZpGains) -> [[f64; 6]; 6] {
    let mut m = [[0.0f64; 6]; 6];
    for (row, (prow, g)) in m.iter_mut().zip(phi.iter().zip(gam.iter())) {
        row[..4].copy_from_slice(prow);
        row[5] = *g;
    }
    m[4][2] = t;
    m[4][4] = 1.0;
    m[5][0] = k.kp_a_per_m;
    m[5][1] = k.kd_a_s_per_m;
    m[5][4] = k.kappa_per_s;
    m
}

/// Discrete-time gains designed on the ZOH plant augmented with the LET delay, the
/// current-loop lag τ_cl and the nominal eddy lag τ_e (plan §2b item 1; LEV-007 poles
/// within 2 % of the discrete design, ζ ≥ 0.5; LEV-020 stable for τ_e ≤ 2 ms).
///
/// The closed-loop characteristic polynomial χ(z) = det(zI − M(K)) of the six-state loop
/// (see the module docs) is affine in K = (k_p, k_d, κ): expanding along the command row,
/// χ(z) = z·C₅₅(z) − k_p·C₅₀(z) − k_d·C₅₁(z) − κ·C₅₄(z) with K-free cofactors. Requiring
/// χ = 0 at the complex target pole (real and imaginary parts) and at the real one gives
/// three linear equations. The remaining three poles (current loop, eddy lag, delay) are
/// left where the plant puts them; [`zp_closed_loop_poles`] reports them. Passive or
/// inactive modes return zero gains.
pub fn zp_gains_discrete(p: &SystemParams, mode: LevMode) -> ZpGains {
    if !mode_active(p, mode) {
        return ZpGains::default();
    }
    let mm = mode_model(p, mode);
    let t = fast_period_s(p);
    let tau_cl = tau_current_loop_s(p);
    if t <= 0.0 || tau_cl <= 0.0 || mm.m_eff_kg <= 0.0 {
        return ZpGains::default();
    }
    let tau_e = fmath::fmax(p.control.levitation.tau_e_nominal_s, 0.0);
    let (phi, gam) = zoh_plant(&mm, t, tau_cl, tau_e);
    let m0 = closed_loop(&phi, &gam, t, &ZpGains::default());
    let targets = target_poles_s(p, mode);
    // Rows: Re and Im at the first target (or two real targets if ζ ≥ 1), then Re at αω.
    let mut rows = [[0.0f64; 3]; 3];
    let mut rhs = [0.0f64; 3];
    let complex_pair = targets[0].im != 0.0;
    let picks: [(usize, bool); 3] = if complex_pair {
        [(0, false), (0, true), (2, false)]
    } else {
        [(0, false), (1, false), (2, false)]
    };
    for (row, (idx, imag)) in picks.iter().enumerate() {
        let z = targets[*idx].scale(t).cexp();
        let c = cofactors_row5(&m0, z);
        let part = |v: lin::C| if *imag { v.im } else { v.re };
        rows[row] = [part(c[0]), part(c[1]), part(c[2])];
        rhs[row] = part(z.mul(c[3]));
    }
    match lin::solve3(rows, rhs) {
        Some(k) => ZpGains {
            kp_a_per_m: k[0],
            kd_a_s_per_m: k[1],
            kappa_per_s: k[2],
        },
        None => ZpGains::default(),
    }
}

/// Cofactors C₅₀, C₅₁, C₅₄, C₅₅ of (zI − M) (M's command row does not enter them).
fn cofactors_row5(m: &[[f64; 6]; 6], z: lin::C) -> [lin::C; 4] {
    let mut zi = [[lin::C::ZERO; 6]; 6];
    for (r, row) in zi.iter_mut().enumerate() {
        for (c, v) in row.iter_mut().enumerate() {
            let diag = if r == c { z } else { lin::C::ZERO };
            *v = diag.sub(lin::C::new(m[r][c], 0.0));
        }
    }
    let cof = |col: usize| -> lin::C {
        let mut minor = [[lin::C::ZERO; 5]; 5];
        for (mr, src) in minor.iter_mut().zip(zi.iter().take(5)) {
            let mut k = 0;
            for (c, v) in src.iter().enumerate() {
                if c != col {
                    mr[k] = *v;
                    k += 1;
                }
            }
        }
        let d = lin::det(minor);
        // Sign (−1)^(5 + col).
        if (5 + col).is_multiple_of(2) {
            d
        } else {
            d.scale(-1.0)
        }
    };
    [cof(0), cof(1), cof(4), cof(5)]
}

/// All six closed-loop poles (z-plane, (re, im)) of the discrete design model of `mode`
/// with gains `k` when the plant's eddy lag is `tau_e_s` (PHYSICS §15; LEV-007, LEV-020;
/// the `gain_margin` study). Zero gains on a passive mode still return the open-loop
/// poles.
pub fn zp_closed_loop_poles(
    p: &SystemParams,
    mode: LevMode,
    k: &ZpGains,
    tau_e_s: f64,
) -> [(f64, f64); 6] {
    let mm = mode_model(p, mode);
    let t = fast_period_s(p);
    let tau_cl = tau_current_loop_s(p);
    if t <= 0.0 || tau_cl <= 0.0 || mm.m_eff_kg <= 0.0 {
        return [(0.0, 0.0); 6];
    }
    let (phi, gam) = zoh_plant(&mm, t, tau_cl, fmath::fmax(tau_e_s, 0.0));
    let m = closed_loop(&phi, &gam, t, k);
    let roots = lin::roots6(lin::charpoly6(&m));
    let mut out = [(0.0, 0.0); 6];
    for (o, r) in out.iter_mut().zip(roots.iter()) {
        *o = (r.re, r.im);
    }
    out
}

/// Spectral radius max|z| of [`zp_closed_loop_poles`] (stable iff < 1).
pub fn zp_spectral_radius(p: &SystemParams, mode: LevMode, k: &ZpGains, tau_e_s: f64) -> f64 {
    zp_closed_loop_poles(p, mode, k, tau_e_s)
        .iter()
        .map(|(re, im)| fmath::hypot(*re, *im))
        .fold(0.0, fmath::fmax)
}

/// Critical eddy lag: the smallest τ_e (searched up to 50 ms, bisection to 1 µs) at which
/// the preset's discrete design loses stability; `f64::INFINITY` if it never does
/// (plan §2b item 1: the `gain_margin` study reports it; LEV-020 needs ≥ 2 ms), s.
pub fn zp_critical_tau_e(p: &SystemParams, mode: LevMode) -> f64 {
    if !mode_active(p, mode) {
        return f64::INFINITY;
    }
    let k = zp_gains_discrete(p, mode);
    let unstable = |te: f64| zp_spectral_radius(p, mode, &k, te) >= 1.0;
    let (mut lo, mut hi) = (0.0f64, 0.05f64);
    if unstable(lo) {
        return 0.0;
    }
    if !unstable(hi) {
        return f64::INFINITY;
    }
    while hi - lo > 1.0e-6 {
        let mid = 0.5 * (lo + hi);
        if unstable(mid) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    lo
}

/// Force-noise gain per element K = m_eff·(a₁ + p²) with a₁ = (1 + 2ζα)ω² the s¹
/// coefficient of the heave target polynomial (the continuous closed form; HAP-011;
/// DRAWER ≈ 0.31 N/µm at ω = 2.5·p), N/m.
pub fn noise_force_gain(p: &SystemParams) -> f64 {
    let mm = mode_model(p, LevMode::Heave);
    let lt = &p.control.levitation;
    let w = zp_omega(p, LevMode::Heave);
    let p2 = if mm.m_eff_kg > 0.0 {
        mm.k_eff_n_per_m / mm.m_eff_kg
    } else {
        0.0
    };
    mm.m_eff_kg * ((1.0 + 2.0 * lt.zeta * lt.alpha) * w * w + p2)
}

/// Force slew rate available from the bus k_i·V_bus/L with L = N²·μ₀A_p/(2g* + h′) + L_σ
/// (PWR-005; PHYSICS §4), N/s.
pub fn force_slew_available(p: &SystemParams) -> f64 {
    let s = &p.design.suspension;
    let l = coil_inductance(s);
    if l <= 0.0 {
        return 0.0;
    }
    k_i(s, s.g_star_m, 0.0) * p.design.power.v_bus_v / l
}

/// Required force slew: a full element weight share m*·g₀/n within one loop time
/// constant 1/ω of the heave design (PWR-005; PHYSICS §4, §15), N/s.
pub fn force_slew_required(p: &SystemParams) -> f64 {
    let s = &p.design.suspension;
    p.design.carriage.m_star_kg * G0_MPS2 / n_lift(s) * zp_omega(p, LevMode::Heave)
}

/// Coil inductance at g*: N²·μ₀A_p/(2g* + h′) + L_σ (PHYSICS §4; DRAWER 2.0 mH), H.
pub fn coil_inductance(s: &SuspensionDesign) -> f64 {
    let d = 2.0 * s.g_star_m + h_prime(s.h_m_m, s.mu_rec);
    if d <= 0.0 {
        return 0.0;
    }
    s.turns * s.turns * MU0_H_PER_M * pole_area(s) / d + s.l_sigma_h
}

/// Minimum rail flange thickness that carries the pole flux without saturating:
/// t ≥ B_g·w/B_sat,rail, with B_g the larger of the design flux and the PM flux at g*
/// (LEV-015; PHYSICS §10), m.
pub fn flange_min_m(p: &SystemParams) -> f64 {
    let s = &p.design.suspension;
    let r = &p.design.rail;
    if r.b_sat_rail_t <= 0.0 {
        return f64::INFINITY;
    }
    let d = 2.0 * s.g_star_m + h_prime(s.h_m_m, s.mu_rec);
    let b_pm = if d > 0.0 {
        MU0_H_PER_M * m_pm(s.b_r_t, s.h_m_m, s.mu_rec) / d
    } else {
        0.0
    };
    fmath::fmax(s.b_g_t, b_pm) * s.pole_w_m / r.b_sat_rail_t
}

// ---- Coupled roll–sway design (PHYSICS §6, §15; Wave 2 lead decision D1) ----------------

/// Minimum roll–sway coupling ratio n·k_y·h²/(k_s·Σr² + m*·g₀·h) above which an active
/// roll loop is designed on the coupled roll–sway model instead of the single-DOF roll
/// model. Design choice: HEAVY (CoM 1.25 m above the pole plane) has 1.97, DRAWER
/// (0.08 m) 0.032; the single-DOF roll loop is unstable on the coupled plant of HEAVY
/// (spectral radius 1.0019, a 2.3 Hz roll–sway oscillation growing at 9.5 s⁻¹, the W1.5
/// divergence) and neutral on DRAWER (1.000003, sway damped by the side pads).
pub const ROLL_SWAY_COUPLED_MIN: f64 = 0.25;

/// Zero-power integral pole of the coupled design as a fraction of ω (the single-DOF
/// design uses `levitation.alpha` = 0.3). Design choice from the robustness sweep of the
/// implemented structure (plant τ_e 0–2.5 ms, k_y ±25 %, c_sway 0–3×, h ±10 %): α 0.3
/// leaves the zero-power pole at 24 rad/s on top of the 15–25 rad/s roll–sway band; 0.1
/// separates them (worst spectral radius 0.99949 at 5 kHz).
pub const ROLL_SWAY_ALPHA: f64 = 0.1;

/// Sway target pole pair of the coupled design: natural frequency as a multiple of the
/// passive sway frequency ω_y = √(n·k_y/m*), and damping ratio. Design choice (same
/// sweep): keep the frequency, add damping (ζ_y 0.6) with small sway gains.
pub const ROLL_SWAY_SWAY_RATIO: f64 = 1.0;

/// Sway target damping ratio of the coupled design (see [`ROLL_SWAY_SWAY_RATIO`]).
pub const ROLL_SWAY_SWAY_ZETA: f64 = 0.6;

/// Sway observer pole pair as a multiple of ω_y (ζ 0.7). Design choice: a slow sway
/// estimate (0.7·ω_y) keeps the observer gains small, so eddy-lag and stiffness errors do
/// not leak into the sway feedback (a 4·ω_y sway observer loses stability at τ_e ≈ 0).
pub const ROLL_SWAY_OBS_RATIO: f64 = 0.7;

/// Per-element small-offset lateral reluctance stiffness k_y = F(g*, 0)/w at y = 0 (the
/// co-energy model's first-order centring, F_y ≈ −F_z·(g/w)·tanh(y/g); PHYSICS §2, §6),
/// N/m.
pub fn lateral_stiffness(s: &SuspensionDesign) -> f64 {
    if s.pole_w_m <= 0.0 {
        return 0.0;
    }
    pm_force(s, s.g_star_m, 0.0, 0.0) / s.pole_w_m
}

/// Linear roll–sway model of a rigid carriage whose lift elements (and the guide, on a
/// bottom-guided class) sit a height h below the CoM (PHYSICS §6, §15). With θ the body
/// roll about +x, Y the CoM lateral position and the lateral reluctance acting at the
/// element plane (y_m = Y + h·θ):
///
/// ```text
/// J·θ̈ = (m*·g₀·h + k_s·Σr² − n·k_y·h²)·θ − n·k_y·h·Y − c·h·(Ẏ + h·θ̇) + n·k_i·r̄·i_e
/// m*·Ÿ = −n·k_y·(Y + h·θ) − c·(Ẏ + h·θ̇)
/// ```
///
/// in the modal gap coordinate u = r̄·∂g/∂y = −r̄·θ of `levitation::modal`. With
/// k_y = c = 0 it reduces to the single-DOF roll model of [`mode_model`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RollSwayModel {
    /// Carriage mass m*, kg.
    pub m_kg: f64,
    /// Roll inertia about the CoM J_xx, kg·m².
    pub j_kgm2: f64,
    /// CoM height above the element plane h, m.
    pub h_m: f64,
    /// Total lateral reluctance stiffness n·k_y, N/m.
    pub k_y_n_per_m: f64,
    /// Sway damping c_sway at the guide (element plane), N·s/m.
    pub c_n_s_per_m: f64,
    /// Σr_y² over the lift elements, m².
    pub sum_r2_m2: f64,
    /// RMS roll lever r̄, m.
    pub r_bar_m: f64,
    /// Destabilising roll stiffness k_s·Σr² + m*·g₀·h, N·m/rad.
    pub k_neg_nm_per_rad: f64,
    /// Current gain per element k_i, N/A.
    pub k_i_n_per_a: f64,
}

impl RollSwayModel {
    /// Continuous state matrix A and input column B of x = [u, u̇, Y, Ẏ, i, i_e] with
    /// input u_cmd (the modal current command through the current-loop lag τ_cl and the
    /// eddy lag τ_e; τ_e = 0 makes the force follow i and i_e a decoupled dummy).
    pub fn continuous(&self, tau_cl_s: f64, tau_e_s: f64) -> ([[f64; 6]; 6], [f64; 6]) {
        let (m, j, h, ky, c, r) = (
            self.m_kg,
            self.j_kgm2,
            self.h_m,
            self.k_y_n_per_m,
            self.c_n_s_per_m,
            self.r_bar_m,
        );
        let mut a = [[0.0f64; 6]; 6];
        let mut b = [0.0f64; 6];
        if m <= 0.0 || j <= 0.0 || r <= 0.0 || tau_cl_s <= 0.0 {
            return (a, b);
        }
        let k_th = self.k_neg_nm_per_rad - ky * h * h;
        let bb = self.k_i_n_per_a * self.sum_r2_m2 / j; // k_i/m_eff
        a[0][1] = 1.0;
        a[1][0] = k_th / j;
        a[1][1] = -c * h * h / j;
        a[1][2] = r * ky * h / j;
        a[1][3] = r * c * h / j;
        a[2][3] = 1.0;
        a[3][0] = ky * h / (m * r);
        a[3][1] = c * h / (m * r);
        a[3][2] = -ky / m;
        a[3][3] = -c / m;
        a[4][4] = -1.0 / tau_cl_s;
        b[4] = 1.0 / tau_cl_s;
        if tau_e_s > 0.0 {
            a[1][5] = -bb;
            a[5][4] = 1.0 / tau_e_s;
            a[5][5] = -1.0 / tau_e_s;
        } else {
            a[1][4] = -bb;
            a[5][5] = -1.0 / tau_cl_s;
        }
        (a, b)
    }

    /// Exact ZOH (Φ, Γ) over period `t_s` (Van Loan).
    #[allow(clippy::needless_range_loop)]
    pub fn zoh(&self, t_s: f64, tau_cl_s: f64, tau_e_s: f64) -> ([[f64; 6]; 6], [f64; 6]) {
        let (a, b) = self.continuous(tau_cl_s, tau_e_s);
        let mut aug = [[0.0f64; 7]; 7];
        for r in 0..6 {
            for c in 0..6 {
                aug[r][c] = a[r][c] * t_s;
            }
            aug[r][6] = b[r] * t_s;
        }
        let e = lin::expm(aug);
        let mut phi = [[0.0f64; 6]; 6];
        let mut gam = [0.0f64; 6];
        for r in 0..6 {
            phi[r].copy_from_slice(&e[r][..6]);
            gam[r] = e[r][6];
        }
        (phi, gam)
    }
}

/// The roll–sway model of `p` at (g*, m*) (see [`RollSwayModel`]); `None` without lift
/// elements or a roll lever arm.
pub fn roll_sway_model(p: &SystemParams) -> Option<RollSwayModel> {
    let d = &p.design;
    let s = &d.suspension;
    let c = &d.carriage;
    let n = core::cmp::min(usize::from(s.n_magnets), s.stations.len());
    let (mut sum_r2, mut z_sum, mut cnt) = (0.0f64, 0.0f64, 0.0f64);
    for st in s.stations.iter().take(n) {
        if st.normal_sign > 0.0 && st.has_pm {
            let r = st.y_m - c.cg_m[1];
            sum_r2 += r * r;
            z_sum += st.z_m;
            cnt += 1.0;
        }
    }
    if sum_r2 <= 0.0 || cnt <= 0.0 || c.m_star_kg <= 0.0 || c.j_kgm2[0] <= 0.0 {
        return None;
    }
    let h = c.cg_m[2] - z_sum / cnt;
    Some(RollSwayModel {
        m_kg: c.m_star_kg,
        j_kgm2: c.j_kgm2[0],
        h_m: h,
        k_y_n_per_m: cnt * lateral_stiffness(s),
        c_n_s_per_m: fmath::fmax(d.guidance.c_sway_n_s_per_m, 0.0),
        sum_r2_m2: sum_r2,
        r_bar_m: fmath::sqrt(sum_r2 / cnt),
        k_neg_nm_per_rad: k_s(s, s.g_star_m, 0.0) * sum_r2 + c.m_star_kg * G0_MPS2 * h,
        k_i_n_per_a: k_i(s, s.g_star_m, 0.0),
    })
}

/// Roll–sway coupling ratio n·k_y·h²/(k_s·Σr² + m*·g₀·h): the lateral reluctance's roll
/// stiffness about the CoM relative to the destabilising roll stiffness (0 without a
/// model).
pub fn roll_sway_coupling(p: &SystemParams) -> f64 {
    match roll_sway_model(p) {
        Some(m) if m.k_neg_nm_per_rad > 0.0 => m.k_y_n_per_m * m.h_m * m.h_m / m.k_neg_nm_per_rad,
        _ => 0.0,
    }
}

/// True when the active roll loop is designed on the coupled roll–sway model: roll is
/// actively controlled and the coupling ratio is at least [`ROLL_SWAY_COUPLED_MIN`]
/// (HEAVY in the presets).
pub fn roll_sway_coupled(p: &SystemParams) -> bool {
    mode_active(p, LevMode::Roll) && roll_sway_coupling(p) >= ROLL_SWAY_COUPLED_MIN
}

/// Passive sway frequency ω_y = √(n·k_y/m*), rad/s (0 without a model).
pub fn sway_omega(p: &SystemParams) -> f64 {
    match roll_sway_model(p) {
        Some(m) if m.m_kg > 0.0 && m.k_y_n_per_m > 0.0 => fmath::sqrt(m.k_y_n_per_m / m.m_kg),
        _ => 0.0,
    }
}

/// Coupled roll–sway design (PHYSICS §15; D1).
///
/// The zero-power law gains feed back the modal roll gap, its rate, the CoM sway and its
/// rate and the current integral: u = k_u·û + k_u̇·û̇ + k_Y·Ŷ + k_Ẏ·Ŷ̇ + κ·q (q̇ = i,
/// so the coil current still returns to zero at equilibrium). They place five poles of
/// the eight-state discrete loop [x (6), q, u_d] (the ZOH roll–sway plant with τ_cl and
/// τ_e,nom, the Mizuno integral and one LET period of delay): the roll pair (ω, ζ) of the
/// single-DOF design, the zero-power pole α_rs·ω ([`ROLL_SWAY_ALPHA`]) and a sway pair
/// at ROLL_SWAY_SWAY_RATIO·ω_y with ζ_y = [`ROLL_SWAY_SWAY_ZETA`]; the characteristic
/// polynomial is affine in the gains (one command row), so five linear equations fix
/// them. The observer is the current-estimator Luenberger observer of the four
/// mechanical states from the gap alone (the measured coil current and the known
/// command drive the model; the eddy state is propagated), with the roll pair at
/// observer_ratio·ω and the sway pair at [`ROLL_SWAY_OBS_RATIO`]·ω_y.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RollSwayDesign {
    /// Gains [k_u (A/m), k_u̇ (A·s/m), k_Y (A/m), k_Ẏ (A·s/m), κ (1/s)].
    pub k: [f64; 5],
    /// Current-estimator observer gains for [u, u̇, Y, Ẏ] from the gap innovation.
    pub l: [f64; 4],
    /// Sampled design plant Φ over [u, u̇, Y, Ẏ, i, i_e].
    pub phi: [[f64; 6]; 6],
    /// Sampled design plant Γ.
    pub gam: [f64; 6],
    /// Roll target ω, rad/s.
    pub omega_rad_s: f64,
    /// Passive sway frequency ω_y, rad/s.
    pub omega_y_rad_s: f64,
    /// The model the design used.
    pub model: RollSwayModel,
}

/// Target continuous poles of the coupled design: roll pair, α_rs·ω, sway pair.
pub fn roll_sway_target_poles_s(p: &SystemParams) -> [(f64, f64); 5] {
    let lt = &p.control.levitation;
    let w = zp_omega(p, LevMode::Roll);
    let wy = ROLL_SWAY_SWAY_RATIO * sway_omega(p);
    let pair = |w: f64, z: f64| -> [(f64, f64); 2] {
        if z < 1.0 {
            let wd = w * fmath::sqrt(1.0 - z * z);
            [(-z * w, wd), (-z * w, -wd)]
        } else {
            let r = w * fmath::sqrt(z * z - 1.0);
            [(-z * w + r, 0.0), (-z * w - r, 0.0)]
        }
    };
    let r = pair(w, lt.zeta);
    let s = pair(wy, ROLL_SWAY_SWAY_ZETA);
    [r[0], r[1], (-ROLL_SWAY_ALPHA * w, 0.0), s[0], s[1]]
}

/// Eight-state design loop [x (6), q, u_d] with gains `k` in the command row.
#[allow(clippy::needless_range_loop)]
fn roll_sway_loop(phi: &[[f64; 6]; 6], gam: &[f64; 6], t: f64, k: &[f64; 5]) -> [[f64; 8]; 8] {
    let mut m = [[0.0f64; 8]; 8];
    for r in 0..6 {
        m[r][..6].copy_from_slice(&phi[r]);
        m[r][7] = gam[r];
    }
    m[6][4] = t;
    m[6][6] = 1.0;
    m[7][0] = k[0];
    m[7][1] = k[1];
    m[7][2] = k[2];
    m[7][3] = k[3];
    m[7][6] = k[4];
    m
}

#[allow(clippy::needless_range_loop)]
fn charpoly_at8(m: &[[f64; 8]; 8], z: lin::C) -> lin::C {
    let mut a = [[lin::C::ZERO; 8]; 8];
    for r in 0..8 {
        for c in 0..8 {
            let diag = if r == c { z } else { lin::C::ZERO };
            a[r][c] = diag.sub(lin::C::new(m[r][c], 0.0));
        }
    }
    lin::det(a)
}

/// The coupled roll–sway design of `p` (see [`RollSwayDesign`]); `None` when roll is not
/// on the coupled design ([`roll_sway_coupled`]) or the placement is singular.
#[allow(clippy::needless_range_loop)]
pub fn roll_sway_design(p: &SystemParams) -> Option<RollSwayDesign> {
    if !roll_sway_coupled(p) {
        return None;
    }
    let model = roll_sway_model(p)?;
    let lt = &p.control.levitation;
    let t = fast_period_s(p);
    let tau_cl = tau_current_loop_s(p);
    if t <= 0.0 || tau_cl <= 0.0 {
        return None;
    }
    let tau_e = fmath::fmax(lt.tau_e_nominal_s, 0.0);
    let (phi, gam) = model.zoh(t, tau_cl, tau_e);
    let targets = roll_sway_target_poles_s(p);
    let zero = [0.0f64; 5];
    let m0 = roll_sway_loop(&phi, &gam, t, &zero);
    // Rows: Re/Im at the upper roll pole, Re at α_rs·ω, Re/Im at the upper sway pole.
    let picks: [(usize, bool); 5] = [(0, false), (0, true), (2, false), (3, false), (3, true)];
    let mut rows = [[0.0f64; 5]; 5];
    let mut rhs = [0.0f64; 5];
    for (row, (idx, imag)) in picks.iter().enumerate() {
        let (re, im) = targets[*idx];
        let z = lin::C::new(re * t, im * t).cexp();
        let base = charpoly_at8(&m0, z);
        let part = |v: lin::C| if *imag { v.im } else { v.re };
        for (col, slot) in rows[row].iter_mut().enumerate() {
            let mut e = zero;
            e[col] = 1.0;
            let v = charpoly_at8(&roll_sway_loop(&phi, &gam, t, &e), z).sub(base);
            *slot = part(v);
        }
        rhs[row] = -part(base);
    }
    let k = lin::solve::<5>(rows, rhs)?;
    // Observer: current-estimator gains for the mechanical block (Ackermann, y = u).
    let mut pm = [[0.0f64; 4]; 4];
    for r in 0..4 {
        pm[r].copy_from_slice(&phi[r][..4]);
    }
    let w = zp_omega(p, LevMode::Roll);
    let wy = sway_omega(p);
    let obs = [
        (lt.observer_ratio * w, lt.zeta),
        (ROLL_SWAY_OBS_RATIO * wy, 0.7),
    ];
    let l = observer_current_gains4(&pm, t, &obs)?;
    Some(RollSwayDesign {
        k,
        l,
        phi,
        gam,
        omega_rad_s: w,
        omega_y_rad_s: wy,
        model,
    })
}

/// Current-estimator gains L_c = Φ⁻¹·L_p of a 4-state observer with output y = x₀,
/// placing two continuous pole pairs (ω, ζ) at z = e^{sT} (Ackermann:
/// L_p = φ(Φ)·O⁻¹·e₄, O = [C; CΦ; CΦ²; CΦ³]).
#[allow(clippy::needless_range_loop)]
fn observer_current_gains4(
    phi: &[[f64; 4]; 4],
    t: f64,
    pairs: &[(f64, f64); 2],
) -> Option<[f64; 4]> {
    // Target polynomial Π (z² − s1·z + s0) as z⁴ + c3·z³ + c2·z² + c1·z + c0.
    let mut poly = [1.0f64, 0.0, 0.0, 0.0, 0.0]; // highest first
    let mut deg = 0usize;
    for (w, z) in pairs.iter() {
        let (s1, s0) = if *z < 1.0 {
            let wd = w * fmath::sqrt(1.0 - z * z);
            let q = fmath::exp(-z * w * t);
            (2.0 * q * fmath::cos(wd * t), q * q)
        } else {
            let r = w * fmath::sqrt(z * z - 1.0);
            let z1 = fmath::exp((-z * w + r) * t);
            let z2 = fmath::exp((-z * w - r) * t);
            (z1 + z2, z1 * z2)
        };
        let q = [1.0, -s1, s0];
        let mut next = [0.0f64; 5];
        for i in 0..=deg {
            for (jq, cq) in q.iter().enumerate() {
                next[i + jq] += poly[i] * cq;
            }
        }
        poly = next;
        deg += 2;
    }
    let p2 = matmul4(phi, phi);
    let p3 = matmul4(&p2, phi);
    let p4 = matmul4(&p3, phi);
    let mut phi_p = [[0.0f64; 4]; 4];
    for r in 0..4 {
        for c in 0..4 {
            let id = if r == c { 1.0 } else { 0.0 };
            phi_p[r][c] = p4[r][c]
                + poly[1] * p3[r][c]
                + poly[2] * p2[r][c]
                + poly[3] * phi[r][c]
                + poly[4] * id;
        }
    }
    let o = [[1.0, 0.0, 0.0, 0.0], phi[0], p2[0], p3[0]];
    let o_inv = lin::inverse::<4>(o)?;
    let col = [o_inv[0][3], o_inv[1][3], o_inv[2][3], o_inv[3][3]];
    let mut lp = [0.0f64; 4];
    for r in 0..4 {
        lp[r] = (0..4).map(|c| phi_p[r][c] * col[c]).sum();
    }
    let pi = lin::inverse::<4>(*phi)?;
    let mut lc = [0.0f64; 4];
    for r in 0..4 {
        lc[r] = (0..4).map(|c| pi[r][c] * lp[c]).sum();
    }
    if lc.iter().all(|v| v.is_finite()) {
        Some(lc)
    } else {
        None
    }
}

#[allow(clippy::needless_range_loop)]
fn matmul4(a: &[[f64; 4]; 4], b: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut c = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for k in 0..4 {
            for j in 0..4 {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

// ---- LSM constants and ASC drag (PHYSICS §7; Wave 2 lead decision D5) ---------------

/// Back-EMF constant of the amplitude-invariant dq convention, K_e = ⅔·K_f (PHYSICS §7):
/// with e_q = K_e·v and the dq power P = 1.5·(v_d·i_d + v_q·i_q), the mechanical power
/// F·v = K_f·i_q·v equals 1.5·e_q·i_q exactly when K_f = 1.5·K_e (PRP-002), V·s/m.
pub fn lsm_k_e(k_f_n_per_a: f64) -> f64 {
    2.0 / 3.0 * k_f_n_per_a
}

/// Active-short-circuit drag coefficient b_e = K_f·K_e/R/(1 + (ω_e·L/R)²), N·s/m, at
/// electrical angular speed `omega_e_radps` (PHYSICS §7; D5). With the terminals shorted
/// the dq voltage equations give i_q = −e_q·R/(R² + (ω_e·L)²), so F = K_f·i_q = −b_e·v;
/// the copper loss 1.5·R·(i_d² + i_q²) equals b_e·v² (energy balance). At low speed
/// b_e = K_f·K_e/R = ⅔·K_f²/R. 0 for a non-positive or non-finite resistance.
pub fn asc_drag(
    k_f_n_per_a: f64,
    k_e_v_s_per_m: f64,
    r_ohm: f64,
    l_h: f64,
    omega_e_radps: f64,
) -> f64 {
    if r_ohm.is_nan() || r_ohm <= 0.0 || !r_ohm.is_finite() {
        return 0.0;
    }
    let x = omega_e_radps * l_h / r_ohm;
    k_f_n_per_a * k_e_v_s_per_m / r_ohm / (1.0 + x * x)
}

/// Low-speed ASC drag coefficient b_e(0) = K_f·K_e/R of a parameter set, N·s/m (PHYSICS
/// §7; D5).
pub fn asc_drag_low_speed(p: &SystemParams) -> f64 {
    let l = &p.design.lsm;
    asc_drag(
        l.k_f_n_per_a,
        l.k_e_v_s_per_m,
        l.r_phase_ohm,
        l.l_phase_h,
        0.0,
    )
}

// ---- Motion, governor, clamps, termini (PHYSICS §16, §16.1, §18) ------------------------

/// Low-energy speed cap v = √(2E/m) (PHYSICS §16; SAF-001), m/s.
pub fn v_low_energy(m_kg: f64, e_j: f64) -> f64 {
    if m_kg <= 0.0 || e_j < 0.0 {
        return 0.0;
    }
    fmath::sqrt(2.0 * e_j / m_kg)
}

/// Momentum speed cap v = p/m (PHYSICS §16; SAF-002), m/s.
pub fn momentum_cap(m_kg: f64, p_kgmps: f64) -> f64 {
    if m_kg <= 0.0 {
        return 0.0;
    }
    p_kgmps / m_kg
}

/// Unsensed speed cap min(√(2E/m), p/m) for the rated mass; a momentum cap of 0 means
/// "not applicable" (PHYSICS §16; SAF-001/002), m/s.
pub fn unsensed_speed_cap(m_kg: f64, e_j: f64, p_kgmps: f64) -> f64 {
    let v = v_low_energy(m_kg, e_j);
    if p_kgmps > 0.0 {
        fmath::fmin(v, momentum_cap(m_kg, p_kgmps))
    } else {
        v
    }
}

/// Jerk-limited stopping distance from v at a, j: v²/(2a) + v·a/(2j) when v ≥ a²/j,
/// else v·√(v/j) (PHYSICS §16; PRP-005 anchors 33.43 / 36.31 / 8.86 / 4.91 mm), m.
pub fn stop_distance(v_mps: f64, a_mps2: f64, j_mps3: f64) -> f64 {
    let v = fmath::abs(v_mps);
    if a_mps2 <= 0.0 || j_mps3 <= 0.0 {
        return f64::INFINITY;
    }
    if v >= a_mps2 * a_mps2 / j_mps3 {
        v * v / (2.0 * a_mps2) + v * a_mps2 / (2.0 * j_mps3)
    } else {
        v * fmath::sqrt(v / j_mps3)
    }
}

/// Quasi-static clamp force of a free mass into a linear body spring F = √(2E·k)
/// (PHYSICS §18; SAF-005: 1.69 J → chest 290.7 N, hand 503.5 N, with edge 173 N), N.
pub fn clamp_force(e_j: f64, k_n_per_m: f64) -> f64 {
    if e_j <= 0.0 || k_n_per_m <= 0.0 {
        return 0.0;
    }
    fmath::sqrt(2.0 * e_j * k_n_per_m)
}

/// Series stiffness k₁k₂/(k₁ + k₂) (PHYSICS §18), N/m.
pub fn series_k(k1_n_per_m: f64, k2_n_per_m: f64) -> f64 {
    let s = k1_n_per_m + k2_n_per_m;
    if s <= 0.0 {
        return 0.0;
    }
    k1_n_per_m * k2_n_per_m / s
}

/// ISO/TS 15066 transient-contact speed v = F/√(μ·k), reduced mass
/// μ = m_h·m/(m_h + m) (PHYSICS §18; ISO/TS 15066:2016 Annex A; 500 kg chest 140/280 N →
/// 0.145 / 0.291 m/s), m/s.
pub fn iso15066_speed(f_n: f64, m_h_kg: f64, m_kg: f64, k_n_per_m: f64) -> f64 {
    let mu = if m_h_kg + m_kg > 0.0 {
        m_h_kg * m_kg / (m_h_kg + m_kg)
    } else {
        0.0
    };
    if mu <= 0.0 || k_n_per_m <= 0.0 {
        return 0.0;
    }
    f_n / fmath::sqrt(mu * k_n_per_m)
}

/// Unattended automatic-close speed min(v_cap, F_pinch/√(m_gov·k_series))
/// (plan §4b: 0.23 m/s at 48 kg with 150 N and hand 75 N/mm ⊕ 10 N/mm edge), m/s.
pub fn unattended_close_speed(
    v_cap_mps: f64,
    f_pinch_n: f64,
    m_gov_kg: f64,
    k_series_n_per_m: f64,
) -> f64 {
    if m_gov_kg <= 0.0 || k_series_n_per_m <= 0.0 {
        return 0.0;
    }
    fmath::fmin(
        v_cap_mps,
        f_pinch_n / fmath::sqrt(m_gov_kg * k_series_n_per_m),
    )
}

/// Bumper energy rating ½·m·v² (PHYSICS §8; SAF-014 ≥ 24 / 113 / 563 / 5625 J), J.
pub fn bumper_rating(m_kg: f64, v_mps: f64) -> f64 {
    0.5 * m_kg * v_mps * v_mps
}

/// Manual force on the landing rollers at the rated mass: PM-relieved
/// μ·(W − n·F_pm(g_land)) or the no-PM bound μ·W (PHYSICS §8; SAF-010; F8), N.
pub fn manual_force(p: &SystemParams, relieved: bool) -> f64 {
    let d = &p.design;
    let s = &d.suspension;
    let w = d.carriage.m_rated_kg * G0_MPS2;
    let normal = if relieved {
        fmath::fmax(w - n_lift(s) * pm_force(s, s.g_land_m, 0.0, 0.0), 0.0)
    } else {
        w
    };
    d.landing.rollers_mu * normal
}

/// Terminus blend out of cruise at speed v with jerk j and time constant τ:
/// t_b = √(τ² + 2v/j) − τ, d_b = j·t_b·τ², d_0 = d_b + v·t_b − j·t_b³/6
/// (PHYSICS §16.1; DRAWER τ 0.15 s: d_0 49.8 mm, d_b 32.1 mm). Velocity and deceleration
/// are continuous at t_b: v − j·t_b²/2 = d_b/τ and j·t_b = d_b/τ².
pub fn terminus_blend(v_mps: f64, j_mps3: f64, tau_s: f64) -> TerminusBlend {
    let v = fmath::abs(v_mps);
    if j_mps3 <= 0.0 || tau_s <= 0.0 {
        return TerminusBlend::default();
    }
    let t_b = fmath::sqrt(tau_s * tau_s + 2.0 * v / j_mps3) - tau_s;
    let d_b = j_mps3 * t_b * tau_s * tau_s;
    TerminusBlend {
        t_b_s: t_b,
        d_b_m: d_b,
        d_0_m: d_b + v * t_b - j_mps3 * t_b * t_b * t_b / 6.0,
    }
}

/// Terminus arrival time t_arrive = t_b + τ·ln(d_b/d_ε) + T_q with T_q = τ
/// (PHYSICS §16.1; DRAWER 0.98 s before the +0.2 s seat step), s.
pub fn terminus_t_arrive(v_mps: f64, j_mps3: f64, tau_s: f64, d_eps_m: f64) -> f64 {
    let b = terminus_blend(v_mps, j_mps3, tau_s);
    let capture = if d_eps_m > 0.0 && b.d_b_m > d_eps_m {
        tau_s * fmath::ln(b.d_b_m / d_eps_m)
    } else {
        0.0
    };
    b.t_b_s + capture + tau_s
}

/// Minimum terminus time constant τ_min = v_app/a_max − a_max/(2j) (PHYSICS §16.1;
/// enforced by `validate`; runtime τ_eff = max(τ, τ_min(v_measured))), s.
pub fn tau_min(v_app_mps: f64, a_max_mps2: f64, j_mps3: f64) -> f64 {
    if a_max_mps2 <= 0.0 || j_mps3 <= 0.0 {
        return f64::INFINITY;
    }
    fmath::abs(v_app_mps) / a_max_mps2 - a_max_mps2 / (2.0 * j_mps3)
}

/// Thrust needed at the rated mass: m_rated·a_max + (b_dist + c_rail)·v_max + F_hyst
/// (PRP-003; PHYSICS §7, §9), N.
pub fn thrust_required(p: &SystemParams) -> f64 {
    let d = &p.design;
    let v = fmath::fmax(p.safety.v_open_max_mps, p.safety.v_close_max_mps);
    d.carriage.m_rated_kg * p.control.motion.a_max_mps2 + drag_force(p, v)
}

/// Levitated drag at speed v: (b_dist + c_rail)·|v| + F_hyst (PHYSICS §9), N.
pub fn drag_force(p: &SystemParams, v_mps: f64) -> f64 {
    let dmp = &p.design.damping;
    (dmp.b_dist_n_s_per_m + dmp.c_rail_n_s_per_m) * fmath::abs(v_mps) + dmp.f_hyst_n
}

// ---- Haptics (PHYSICS §17) --------------------------------------------------------------

/// Passivity stiffness bound K_max = 2b/(T + 2·T_d) for sample period T and delay T_d
/// (Colgate–Schenkel; plan §2b item 19; HAP-009), N/m.
pub fn passivity_k_max(b_n_s_per_m: f64, t_s: f64, t_d_s: f64) -> f64 {
    let den = t_s + 2.0 * t_d_s;
    if den <= 0.0 {
        return f64::INFINITY;
    }
    2.0 * b_n_s_per_m / den
}

// ---- Air, power (PHYSICS §11, §14) ------------------------------------------------------

/// Cavity piston force through a leak orifice: ΔP = ½ρ(v·A_p/(C_d·A_leak))², F = ΔP·A_p
/// with C_d = [`ORIFICE_CD`] (PHYSICS §11; ENV-003: 0.06 m², 0.5 m/s, 40 cm² → 5.6 N).
/// A sealed cavity (A_leak ≤ 0) is a gas spring, not an orifice: returns 0, N.
pub fn piston_force(a_p_m2: f64, v_mps: f64, a_leak_m2: f64) -> f64 {
    if a_leak_m2 <= 0.0 {
        return 0.0;
    }
    let u = v_mps * a_p_m2 / (ORIFICE_CD * a_leak_m2);
    0.5 * RHO_AIR_KG_PER_M3 * u * u * a_p_m2
}

/// Float gap of mass m: the zero-power gap clamped into the modified band (F1), m.
pub fn float_gap(p: &SystemParams, m_kg: f64) -> f64 {
    let s = &p.design.suspension;
    let g = zero_power_gap(
        m_kg,
        s.g_star_m,
        h_prime(s.h_m_m, s.mu_rec),
        p.design.carriage.m_star_kg,
    );
    fmath::clamp(g, s.zp_band_m[0], s.zp_band_m[1])
}

/// Copper energy of a controlled quasi-static descent (or ascent) of mass m from the
/// float gap to `g_rest` at speed v_land: n·R·∫i(g)²dg/v_land with i = bias current
/// (Simpson, 64 panels), J.
fn transfer_copper_energy(p: &SystemParams, m_kg: f64, g_rest_m: f64) -> f64 {
    let d = &p.design;
    let s = &d.suspension;
    let v = p.control.levitation.land_speed_mps;
    if v <= 0.0 {
        return f64::INFINITY;
    }
    let g0 = float_gap(p, m_kg);
    let (a, b) = if g0 <= g_rest_m {
        (g0, g_rest_m)
    } else {
        (g_rest_m, g0)
    };
    const PANELS: u32 = 64;
    let h = (b - a) / f64::from(PANELS);
    let i2 = |g: f64| {
        let i = bias_current(d, g, m_kg);
        i * i
    };
    let mut sum = i2(a) + i2(b);
    for k in 1..PANELS {
        let w = if k % 2 == 1 { 4.0 } else { 2.0 };
        sum += w * i2(a + h * f64::from(k));
    }
    n_lift(s) * s.r_coil_ohm * (h / 3.0) * sum / v
}

/// Hold-up energy need vs supercap availability (PHYSICS §14; SAF-009 margin ≥ 3).
///
/// need = E_copper + P_electronics·(t_stop + t_transfer + t_seat), with E_copper the
/// controlled transfer from the float gap to the rest gap (the seat for SEAT, seat-first,
/// else the landing gap) at v_land, taken at the worse of m_rated and m_op_min;
/// t_stop = v_max/(a_brk_ratio·a_max) (braking regenerates, so only electronics count).
/// avail = η·½·C·(V_max² − V_min²).
pub fn holdup_energy(p: &SystemParams) -> HoldupEnergy {
    let d = &p.design;
    let s = &d.suspension;
    let lt = &p.control.levitation;
    let mo = &p.control.motion;
    let g_rest = if matches!(d.rest_policy, RestPolicy::Seat) && s.g_seat_m > 0.0 {
        s.g_seat_m
    } else {
        s.g_land_m
    };
    let masses = [d.carriage.m_rated_kg, d.carriage.m_op_min_kg];
    let mut need = 0.0f64;
    for m in masses {
        if m <= 0.0 {
            continue;
        }
        let e_cu = transfer_copper_energy(p, m, g_rest);
        let t_transfer = if lt.land_speed_mps > 0.0 {
            fmath::abs(g_rest - float_gap(p, m)) / lt.land_speed_mps
        } else {
            f64::INFINITY
        };
        let a_brk = mo.a_brk_ratio * mo.a_max_mps2;
        let v = fmath::fmax(p.safety.v_open_max_mps, p.safety.v_close_max_mps);
        let t_stop = if a_brk > 0.0 {
            v / a_brk
        } else {
            f64::INFINITY
        };
        let e = e_cu + d.power.p_electronics_w * (t_stop + t_transfer + lt.seat_time_s);
        need = fmath::fmax(need, e);
    }
    let pw = &d.power;
    HoldupEnergy {
        need_j: need,
        avail_j: pw.converter_eff
            * 0.5
            * pw.supercap_c_f
            * (pw.supercap_v_max_v * pw.supercap_v_max_v
                - pw.supercap_v_min_v * pw.supercap_v_min_v),
    }
}

// ---- Seismic response and detection (PHYSICS §20, §21) ----------------------------------

/// Den Hartog optimum TMD tuning for mass ratio μ: (f = 1/(1 + μ),
/// ζ = √(3μ/(8(1 + μ)³))) (Den Hartog 1956; PHYSICS §21; SEI-016: μ 0.02 → 0.98039 /
/// 0.08407, μ 0.01 → 0.99010 / 0.06033).
pub fn den_hartog(mu: f64) -> (f64, f64) {
    if mu <= 0.0 {
        return (1.0, 0.0);
    }
    let q = 1.0 + mu;
    (1.0 / q, fmath::sqrt(3.0 * mu / (8.0 * q * q * q)))
}

/// Den Hartog equal-peak amplification of the undamped main mass √(1 + 2/μ)
/// (PHYSICS §21; SEI-016: μ 0.02 → 10.05, μ 0.01 → 14.177).
pub fn den_hartog_peak(mu: f64) -> f64 {
    if mu <= 0.0 {
        return f64::INFINITY;
    }
    fmath::sqrt(1.0 + 2.0 / mu)
}

/// Warburton optimum for base-acceleration excitation, white noise:
/// (f = √(1 − μ/2)/(1 + μ), ζ = √(μ(1 − μ/4)/(4(1 + μ)(1 − μ/2)))) (Warburton 1982;
/// PHYSICS §21; SEI-016: μ 0.02 → 0.97548 / 0.07019, μ 0.01 → 0.98762 / 0.04981).
pub fn warburton(mu: f64) -> (f64, f64) {
    if mu <= 0.0 {
        return (1.0, 0.0);
    }
    let q = 1.0 + mu;
    (
        fmath::sqrt(1.0 - mu / 2.0) / q,
        fmath::sqrt(mu * (1.0 - mu / 4.0) / (4.0 * q * (1.0 - mu / 2.0))),
    )
}

/// Warburton optimum for force (wind) excitation on the main mass, white noise:
/// (f = √(1 + μ/2)/(1 + μ), ζ = √(μ(1 + 3μ/4)/(4(1 + μ)(1 + μ/2)))) (Warburton 1982;
/// PHYSICS §21; plan §4 exotic anchors: f 0.99257 / 0.98528 at μ 0.01 / 0.02).
pub fn warburton_force(mu: f64) -> (f64, f64) {
    if mu <= 0.0 {
        return (1.0, 0.0);
    }
    let q = 1.0 + mu;
    (
        fmath::sqrt(1.0 + mu / 2.0) / q,
        fmath::sqrt(mu * (1.0 + 0.75 * mu) / (4.0 * q * (1.0 + mu / 2.0))),
    )
}

/// Passive isolator transmissibility T = √((1 + (2ζr)²)/((1 − r²)² + (2ζr)²))
/// (PHYSICS §21; SEI-014: T(3, 0.2) = 0.1931, T(√2) = 1, T(1, 0.2) = 2.693).
pub fn transmissibility(r: f64, zeta: f64) -> f64 {
    let c = 2.0 * zeta * r;
    let a = 1.0 - r * r;
    fmath::sqrt((1.0 + c * c) / (a * a + c * c))
}

/// Skyhook transmissibility T_sky = 1/√((1 − r²)² + (2ζr)²) (PHYSICS §16.2, §21;
/// FAB-HAP-017…019 reference).
pub fn t_sky(r: f64, zeta: f64) -> f64 {
    let c = 2.0 * zeta * r;
    let a = 1.0 - r * r;
    1.0 / fmath::sqrt(a * a + c * c)
}

/// Minimum isolation period from continuous thrust and stroke T_min = 2π·√(m·s/F_cont)
/// (the spring k ≤ F_cont/s; PHYSICS §21; plan §2b item 5: WALL 6.3 s), s.
pub fn t_min_iso(m_kg: f64, stroke_m: f64, f_cont_n: f64) -> f64 {
    if f_cont_n <= 0.0 {
        return f64::INFINITY;
    }
    2.0 * PI * fmath::sqrt(m_kg * stroke_m / f_cont_n)
}

/// Wu–Zhao peak displacement `log₁₀ Pd[cm] = −3.463 + 0.729·M − 1.374·log₁₀ R[km]`
/// (Wu & Zhao 2006; PHYSICS §20; SEI-012: M7 R20 → 0.7118 cm, M6.5 R30 → 0.1762,
/// M5 R10 → 0.0643, M7.8 R100 → 0.2987), m.
pub fn pd_wu_zhao(magnitude: f64, r_km: f64) -> f64 {
    if r_km <= 0.0 {
        return f64::INFINITY;
    }
    let log_cm = -3.463 + 0.729 * magnitude - 1.374 * fmath::log10(r_km);
    0.01 * fmath::pow(10.0, log_cm)
}

/// S−P time r/v_s − r/v_p (PHYSICS §20; SEI-012: 0.1190 s/km at 6.0 / 3.5 km/s), s.
pub fn sp_time(r_hyp_km: f64, v_p_kmps: f64, v_s_kmps: f64) -> f64 {
    if v_p_kmps <= 0.0 || v_s_kmps <= 0.0 {
        return 0.0;
    }
    r_hyp_km / v_s_kmps - r_hyp_km / v_p_kmps
}

/// Warning (lead) time at the site: S-wave arrival r/v_s minus the alert time, both
/// measured from origin (PHYSICS §20; on-site t_alert = r/v_p + t_detect), s.
pub fn lead_time(r_hyp_km: f64, v_s_kmps: f64, t_alert_s: f64) -> f64 {
    if v_s_kmps <= 0.0 {
        return 0.0;
    }
    r_hyp_km / v_s_kmps - t_alert_s
}

/// Distance where network and on-site warnings give equal lead time:
/// r/v_s − (r/v_p + t_onsite) = r/v_s − t_network ⇒ r = v_p·(t_network − t_onsite)
/// (PHYSICS §20; SEI-012: 6.0 km/s, 1.2 s, 8 s → 40.8 km), km.
pub fn lead_crossover_km(v_p_kmps: f64, t_onsite_s: f64, t_network_s: f64) -> f64 {
    v_p_kmps * (t_network_s - t_onsite_s)
}

// ---- Derived table (Params.derived(), PARAMETERS.md, docs) -----------------------------

/// Number of entries of [`derived`].
pub const DERIVED_N: usize = 40;

/// Named derived quantities of a parameter set, in SI with unit-suffixed names: the
/// single source for `Params.derived()`, the `presets` CLI table and PARAMETERS.md.
pub fn derived(p: &SystemParams) -> [(&'static str, f64); DERIVED_N] {
    let d = &p.design;
    let s = &d.suspension;
    let c = &d.carriage;
    let sf = &p.safety;
    let mo = &p.control.motion;
    let hp = h_prime(s.h_m_m, s.mu_rec);
    let heave = zp_gains_discrete(p, LevMode::Heave);
    let heave_c = zp_gains(p, LevMode::Heave);
    let v_cap = fmath::fmax(sf.v_open_max_mps, sf.v_close_max_mps);
    let tb = terminus_blend(v_cap, mo.terminus.j_mps3, mo.terminus.tau_s);
    let hold = holdup_energy(p);
    let k_hand_edge = series_k(75.0e3, d.damping.k_edge_n_per_m);
    let e_cap = bumper_rating(sf.mass_rated_kg, v_cap);
    let t_fast = fast_period_s(p);
    let seat_or_land = if s.g_seat_m > 0.0 {
        s.g_seat_m
    } else {
        s.g_land_m
    };
    [
        ("pole_rad_s", open_loop_pole(s.g_star_m, hp)),
        ("pole_pitch_rad_s", mode_pole(p, LevMode::Pitch)),
        ("pole_roll_rad_s", mode_pole(p, LevMode::Roll)),
        ("h_prime_m", hp),
        ("m_pm_at", m_pm(s.b_r_t, s.h_m_m, s.mu_rec)),
        ("pole_area_m2", pole_area(s)),
        ("f_star_n", pm_force(s, s.g_star_m, 0.0, 0.0)),
        ("k_i_n_per_a", k_i(s, s.g_star_m, 0.0)),
        ("k_s_n_per_m", k_s(s, s.g_star_m, 0.0)),
        ("coil_l_h", coil_inductance(s)),
        ("coil_tau_s", {
            let l = coil_inductance(s);
            if s.r_coil_ohm > 0.0 {
                l / s.r_coil_ohm
            } else {
                0.0
            }
        }),
        ("float_gap_rated_m", float_gap(p, c.m_rated_kg)),
        ("stop_ratio_up", stop_ratio(d, s.g_up_m, c.m_star_kg)),
        (
            "stay_landed_ratio",
            stop_ratio(d, s.g_land_m, c.m_op_min_kg),
        ),
        (
            "upper_stop_force_n",
            n_lift(s) * pm_force(s, s.g_up_m, 0.0, 0.0),
        ),
        (
            "lift_current_rated_a",
            bias_current(d, seat_or_land, c.m_rated_kg),
        ),
        (
            "lift_current_min_a",
            bias_current(d, seat_or_land, c.m_op_min_kg),
        ),
        ("thermal_gap_40k_m", thermal_gap_shift(p, 40.0)),
        ("omega_heave_rad_s", zp_omega(p, LevMode::Heave)),
        ("kp_heave_a_per_m", heave.kp_a_per_m),
        ("kd_heave_a_s_per_m", heave.kd_a_s_per_m),
        ("kappa_heave_per_s", heave.kappa_per_s),
        ("kp_heave_cont_a_per_m", heave_c.kp_a_per_m),
        ("noise_force_gain_n_per_m", noise_force_gain(p)),
        (
            "critical_tau_e_heave_s",
            zp_critical_tau_e(p, LevMode::Heave),
        ),
        ("v_open_cap_mps", sf.v_open_max_mps),
        ("v_close_cap_mps", sf.v_close_max_mps),
        ("ke_at_cap_j", e_cap),
        (
            "d_stop_m",
            stop_distance(v_cap, mo.a_max_mps2, mo.j_max_mps3),
        ),
        ("clamp_hand_edge_n", clamp_force(e_cap, k_hand_edge)),
        (
            "tau_min_s",
            tau_min(v_cap, mo.a_max_mps2, mo.terminus.j_mps3),
        ),
        ("terminus_d0_m", tb.d_0_m),
        ("terminus_db_m", tb.d_b_m),
        (
            "terminus_t_arrive_s",
            terminus_t_arrive(
                v_cap,
                mo.terminus.j_mps3,
                mo.terminus.tau_s,
                mo.terminus.d_eps_m,
            ),
        ),
        (
            "k_max_passive_n_per_m",
            passivity_k_max(d.damping.b_dist_n_s_per_m, t_fast, t_fast),
        ),
        ("holdup_need_j", hold.need_j),
        ("holdup_avail_j", hold.avail_j),
        ("manual_force_relieved_n", manual_force(p, true)),
        ("manual_force_bound_n", manual_force(p, false)),
        (
            "t_min_iso_s",
            t_min_iso(
                c.m_rated_kg,
                p.control.seismic.isolation.stroke_m,
                d.lsm.f_cont_n,
            ),
        ),
    ]
}

// ---- Small fixed-size linear algebra (no alloc) -----------------------------------------

mod lin {
    //! Complex scalars, determinants, a 3×3 solver, the matrix exponential and a degree-6
    //! root finder, all on fixed-size arrays (no alloc; fmath only).
    #![allow(clippy::needless_range_loop)]

    use fabrail_types::fmath;

    /// Complex number.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct C {
        pub re: f64,
        pub im: f64,
    }

    impl C {
        pub const ZERO: C = C { re: 0.0, im: 0.0 };
        pub const ONE: C = C { re: 1.0, im: 0.0 };

        pub const fn new(re: f64, im: f64) -> C {
            C { re, im }
        }
        pub fn add(self, o: C) -> C {
            C::new(self.re + o.re, self.im + o.im)
        }
        pub fn sub(self, o: C) -> C {
            C::new(self.re - o.re, self.im - o.im)
        }
        pub fn mul(self, o: C) -> C {
            C::new(
                self.re * o.re - self.im * o.im,
                self.re * o.im + self.im * o.re,
            )
        }
        pub fn scale(self, k: f64) -> C {
            C::new(self.re * k, self.im * k)
        }
        pub fn norm(self) -> f64 {
            fmath::hypot(self.re, self.im)
        }
        pub fn div(self, o: C) -> C {
            // Smith's algorithm.
            if fmath::abs(o.re) >= fmath::abs(o.im) {
                if o.re == 0.0 {
                    return C::new(f64::NAN, f64::NAN);
                }
                let r = o.im / o.re;
                let den = o.re + o.im * r;
                C::new((self.re + self.im * r) / den, (self.im - self.re * r) / den)
            } else {
                let r = o.re / o.im;
                let den = o.re * r + o.im;
                C::new((self.re * r + self.im) / den, (self.im * r - self.re) / den)
            }
        }
        pub fn cexp(self) -> C {
            let m = fmath::exp(self.re);
            C::new(m * fmath::cos(self.im), m * fmath::sin(self.im))
        }
    }

    /// Determinant by Gaussian elimination with partial pivoting.
    pub fn det<const N: usize>(mut a: [[C; N]; N]) -> C {
        let mut d = C::ONE;
        for col in 0..N {
            let mut piv = col;
            let mut best = a[col][col].norm();
            for r in (col + 1)..N {
                let v = a[r][col].norm();
                if v > best {
                    best = v;
                    piv = r;
                }
            }
            if best == 0.0 {
                return C::ZERO;
            }
            if piv != col {
                a.swap(piv, col);
                d = d.scale(-1.0);
            }
            let p = a[col][col];
            d = d.mul(p);
            for r in (col + 1)..N {
                let f = a[r][col].div(p);
                for c in col..N {
                    let v = a[col][c];
                    a[r][c] = a[r][c].sub(f.mul(v));
                }
            }
        }
        d
    }

    /// Solves a 3×3 real system with column equilibration and partial pivoting.
    pub fn solve3(mut a: [[f64; 3]; 3], mut b: [f64; 3]) -> Option<[f64; 3]> {
        let mut scale = [1.0f64; 3];
        for c in 0..3 {
            let m = fmath::fmax(
                fmath::fmax(fmath::abs(a[0][c]), fmath::abs(a[1][c])),
                fmath::abs(a[2][c]),
            );
            if m == 0.0 || !m.is_finite() {
                return None;
            }
            scale[c] = m;
            for r in 0..3 {
                a[r][c] /= m;
            }
        }
        for col in 0..3 {
            let mut piv = col;
            for r in (col + 1)..3 {
                if fmath::abs(a[r][col]) > fmath::abs(a[piv][col]) {
                    piv = r;
                }
            }
            if a[piv][col] == 0.0 {
                return None;
            }
            a.swap(piv, col);
            b.swap(piv, col);
            for r in (col + 1)..3 {
                let f = a[r][col] / a[col][col];
                for c in col..3 {
                    a[r][c] -= f * a[col][c];
                }
                b[r] -= f * b[col];
            }
        }
        let mut x = [0.0f64; 3];
        for r in (0..3).rev() {
            let mut s = b[r];
            for c in (r + 1)..3 {
                s -= a[r][c] * x[c];
            }
            x[r] = s / a[r][r];
        }
        for c in 0..3 {
            x[c] /= scale[c];
            if !x[c].is_finite() {
                return None;
            }
        }
        Some(x)
    }

    /// Solves an N×N real system with column equilibration and partial pivoting.
    pub fn solve<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
        let mut scale = [1.0f64; N];
        for c in 0..N {
            let m = (0..N).map(|r| fmath::abs(a[r][c])).fold(0.0, fmath::fmax);
            if m == 0.0 || !m.is_finite() {
                return None;
            }
            scale[c] = m;
            for r in 0..N {
                a[r][c] /= m;
            }
        }
        for col in 0..N {
            let mut piv = col;
            for r in (col + 1)..N {
                if fmath::abs(a[r][col]) > fmath::abs(a[piv][col]) {
                    piv = r;
                }
            }
            if a[piv][col] == 0.0 {
                return None;
            }
            a.swap(piv, col);
            b.swap(piv, col);
            for r in (col + 1)..N {
                let f = a[r][col] / a[col][col];
                for c in col..N {
                    a[r][c] -= f * a[col][c];
                }
                b[r] -= f * b[col];
            }
        }
        let mut x = [0.0f64; N];
        for r in (0..N).rev() {
            let mut s = b[r];
            for c in (r + 1)..N {
                s -= a[r][c] * x[c];
            }
            x[r] = s / a[r][r];
        }
        for c in 0..N {
            x[c] /= scale[c];
            if !x[c].is_finite() {
                return None;
            }
        }
        Some(x)
    }

    /// Inverse of an N×N real matrix (Gauss–Jordan with partial pivoting).
    pub fn inverse<const N: usize>(mut a: [[f64; N]; N]) -> Option<[[f64; N]; N]> {
        let mut inv = [[0.0f64; N]; N];
        for i in 0..N {
            inv[i][i] = 1.0;
        }
        for col in 0..N {
            let mut piv = col;
            for r in (col + 1)..N {
                if fmath::abs(a[r][col]) > fmath::abs(a[piv][col]) {
                    piv = r;
                }
            }
            if a[piv][col] == 0.0 || !a[piv][col].is_finite() {
                return None;
            }
            a.swap(piv, col);
            inv.swap(piv, col);
            let p = a[col][col];
            for c in 0..N {
                a[col][c] /= p;
                inv[col][c] /= p;
            }
            for r in 0..N {
                if r == col {
                    continue;
                }
                let f = a[r][col];
                if f == 0.0 {
                    continue;
                }
                for c in 0..N {
                    a[r][c] -= f * a[col][c];
                    inv[r][c] -= f * inv[col][c];
                }
            }
        }
        Some(inv)
    }

    fn matmul<const N: usize>(a: &[[f64; N]; N], b: &[[f64; N]; N]) -> [[f64; N]; N] {
        let mut out = [[0.0f64; N]; N];
        for i in 0..N {
            for k in 0..N {
                let aik = a[i][k];
                if aik == 0.0 {
                    continue;
                }
                for j in 0..N {
                    out[i][j] += aik * b[k][j];
                }
            }
        }
        out
    }

    /// Matrix exponential by scaling and squaring with a degree-20 Taylor series.
    pub fn expm<const N: usize>(a: [[f64; N]; N]) -> [[f64; N]; N] {
        let norm = a
            .iter()
            .map(|row| row.iter().map(|v| fmath::abs(*v)).sum::<f64>())
            .fold(0.0, fmath::fmax);
        let mut squarings = 0u32;
        let mut scaled = a;
        let mut n = norm;
        while n > 0.5 && squarings < 60 {
            n *= 0.5;
            squarings += 1;
        }
        let k = fmath::powi(0.5, squarings as i32);
        for row in scaled.iter_mut() {
            for v in row.iter_mut() {
                *v *= k;
            }
        }
        let mut result = [[0.0f64; N]; N];
        let mut term = [[0.0f64; N]; N];
        for i in 0..N {
            result[i][i] = 1.0;
            term[i][i] = 1.0;
        }
        for order in 1..=20u32 {
            term = matmul(&term, &scaled);
            let inv = 1.0 / f64::from(order);
            for row in term.iter_mut() {
                for v in row.iter_mut() {
                    *v *= inv;
                }
            }
            for i in 0..N {
                for j in 0..N {
                    result[i][j] += term[i][j];
                }
            }
        }
        for _ in 0..squarings {
            result = matmul(&result, &result);
        }
        result
    }

    /// Monic characteristic polynomial det(zI − M) of a real 6×6 matrix, coefficients
    /// c[0] + c[1]z + … + c[6]z⁶ (c[6] = 1), from exact evaluation at the 8th roots of
    /// unity and an inverse DFT.
    pub fn charpoly6(m: &[[f64; 6]; 6]) -> [f64; 7] {
        const K: usize = 8;
        let mut vals = [C::ZERO; K];
        for (k, v) in vals.iter_mut().enumerate() {
            let th = 2.0 * core::f64::consts::PI * (k as f64) / (K as f64);
            let z = C::new(fmath::cos(th), fmath::sin(th));
            let mut a = [[C::ZERO; 6]; 6];
            for r in 0..6 {
                for c in 0..6 {
                    let diag = if r == c { z } else { C::ZERO };
                    a[r][c] = diag.sub(C::new(m[r][c], 0.0));
                }
            }
            *v = det(a);
        }
        let mut coeffs = [0.0f64; 7];
        for j in 0..7 {
            let mut acc = C::ZERO;
            for (k, v) in vals.iter().enumerate() {
                let th = -2.0 * core::f64::consts::PI * ((j * k) as f64) / (K as f64);
                acc = acc.add(v.mul(C::new(fmath::cos(th), fmath::sin(th))));
            }
            coeffs[j] = acc.re / (K as f64);
        }
        coeffs[6] = 1.0;
        coeffs
    }

    fn horner(c: &[f64; 7], z: C) -> (C, C) {
        // p(z) and p'(z).
        let mut p = C::new(c[6], 0.0);
        let mut dp = C::ZERO;
        for j in (0..6).rev() {
            dp = dp.mul(z).add(p);
            p = p.mul(z).add(C::new(c[j], 0.0));
        }
        (p, dp)
    }

    /// Roots of a monic degree-6 real polynomial (Aberth–Ehrlich iteration).
    pub fn roots6(c: [f64; 7]) -> [C; 6] {
        let bound = 1.0
            + c.iter()
                .take(6)
                .map(|v| fmath::abs(*v))
                .fold(0.0, fmath::fmax);
        let r0 = fmath::fmin(bound, 2.0);
        let mut z = [C::ZERO; 6];
        for (k, zk) in z.iter_mut().enumerate() {
            let th = 2.0 * core::f64::consts::PI * (k as f64) / 6.0 + 0.4;
            *zk = C::new(r0 * fmath::cos(th), r0 * fmath::sin(th));
        }
        for _ in 0..500 {
            let mut max_step = 0.0f64;
            for k in 0..6 {
                let (p, dp) = horner(&c, z[k]);
                if p.norm() == 0.0 {
                    continue;
                }
                let ratio = p.div(dp);
                let mut sum = C::ZERO;
                for j in 0..6 {
                    if j != k {
                        let d = z[k].sub(z[j]);
                        if d.norm() > 0.0 {
                            sum = sum.add(C::ONE.div(d));
                        }
                    }
                }
                let den = C::ONE.sub(ratio.mul(sum));
                let step = ratio.div(den);
                if step.re.is_finite() && step.im.is_finite() {
                    z[k] = z[k].sub(step);
                    max_step = fmath::fmax(max_step, step.norm());
                }
            }
            if max_step < 1.0e-15 {
                break;
            }
        }
        z
    }
}

#[cfg(test)]
mod tests {
    //! Analytic anchors (core design §E.5; plan §4, §4b, §6). Every assertion message
    //! states its tolerance class (core design §E.1) and source.
    use super::*;
    use crate::presets;

    fn rel(a: f64, b: f64) -> f64 {
        fmath::abs(a - b) / fmath::abs(b)
    }

    fn hp_of(p: &SystemParams) -> f64 {
        h_prime(p.design.suspension.h_m_m, p.design.suspension.mu_rec)
    }

    // req: FAB-LEV-003
    #[test]
    fn open_loop_pole() {
        let anchors = [69.49, 47.57, 32.50, 32.50];
        for (c, want) in presets::CLASSES.iter().zip(anchors) {
            let p = presets::by_class(*c);
            let s = &p.design.suspension;
            let got = super::open_loop_pole(s.g_star_m, hp_of(&p));
            assert!(
                fmath::abs(got - want) <= 0.005,
                "{c:?}: p = {got} rad/s vs {want} (±0.005, 4 s.f.; core §E.5, LEV-003)"
            );
            let exact = fmath::sqrt(4.0 * G0_MPS2 / (2.0 * s.g_star_m + hp_of(&p)));
            assert!(
                rel(got, exact) <= 1e-12,
                "{c:?}: closed form (E) rel 1e-12, PHYSICS §3"
            );
            assert!(
                rel(mode_pole(&p, LevMode::Heave), got) <= 1e-9,
                "{c:?}: modal heave pole equals the closed form (1e-9; balanced pole, PHYSICS §3)"
            );
        }
    }

    // req: FAB-LEV-005
    #[test]
    fn zero_power_mass() {
        let p = presets::drawer();
        let s = &p.design.suspension;
        let hp = hp_of(&p);
        for (g_mm, want) in [(1.5, 45.76), (5.0, 9.96), (6.2, 7.12)] {
            let got = super::zero_power_mass(g_mm * 1e-3, s.g_star_m, hp, 26.0);
            assert!(
                fmath::abs(got - want) <= 0.005,
                "DRAWER m({g_mm} mm) = {got} kg vs {want} (±0.005 kg, 4 s.f.; core §E.5, LEV-005)"
            );
            let back = zero_power_gap(got, s.g_star_m, hp, 26.0);
            assert!(
                fmath::abs(back - g_mm * 1e-3) <= 1e-15,
                "m(g) and g(m) are inverses ((E); PHYSICS §3)"
            );
        }
        // m(g*) = m* exactly, and the modal model agrees with the balance (PHYSICS §3).
        assert_eq!(
            super::zero_power_mass(s.g_star_m, s.g_star_m, hp, 26.0),
            26.0
        );
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            let s = &p.design.suspension;
            let bal = stop_ratio(&p.design, s.g_star_m, p.design.carriage.m_star_kg);
            assert!(
                fmath::abs(bal - 1.0) <= 1e-12,
                "{c:?}: n·F_pm(g*)/(m*·g₀) = {bal} (balanced pole length, (E) 1e-12)"
            );
        }
    }

    #[test]
    fn zero_power_gap_anchors() {
        let d = presets::drawer();
        let s = &d.design.suspension;
        let hp = hp_of(&d);
        for (m, want_mm) in [(48.0, 1.427), (26.0, 2.500), (8.0, 5.761)] {
            let g = zero_power_gap(m, s.g_star_m, hp, 26.0) * 1e3;
            assert!(
                fmath::abs(g - want_mm) <= 1.0e-3,
                "DRAWER unclamped float gap at {m} kg = {g} mm vs {want_mm} (±1 µm, rounding of h′; core §E.5)"
            );
        }
        let door = presets::door();
        let s = &door.design.suspension;
        let g = zero_power_gap(110.0, s.g_star_m, hp_of(&door), 100.0);
        assert!(
            fmath::abs(g * 1e3 - 4.5967) <= 5.0e-5,
            "DOOR +10 kg (98.1 N at 9.81) → {} mm vs 4.5967 (±0.05 µm; core §E.5, LEV-004)",
            g * 1e3
        );
        let wall = presets::wall();
        let s = &wall.design.suspension;
        let g = zero_power_gap(550.0, s.g_star_m, hp_of(&wall), 500.0);
        assert!(
            fmath::abs(g * 1e3 - 9.1357) <= 5.0e-5,
            "WALL +50 kg (490.5 N at 9.81) → {} mm vs 9.1357 (±0.05 µm; core §E.5)",
            g * 1e3
        );
    }

    #[test]
    fn seat_unseat_currents() {
        let p = presets::drawer();
        let d = &p.design;
        // Core §E.5 quotes +11.23 / −4.98 / 27.45 A computed with the rounded 10×32 mm pole;
        // the preset balances the pole exactly (LEV-004), which moves them by ≤ 0.3 %.
        for (g_mm, m, want) in [(3.5, 48.0, 11.23), (3.5, 8.0, -4.98), (6.5, 48.0, 27.45)] {
            let i = bias_current(d, g_mm * 1e-3, m);
            assert!(
                rel(i, want) <= 5.0e-3,
                "DRAWER lift current at {g_mm} mm, {m} kg = {i} A vs {want} ((P) ±0.5 %, rounded pole; core §E.5, F17)"
            );
        }
        let i_pulse = d.suspension.i_pulse_a;
        assert!(
            bias_current(d, 3.5e-3, 48.0) <= i_pulse,
            "unseat within i_pulse (LEV-017)"
        );
        assert!(
            -bias_current(d, 3.5e-3, 8.0) <= i_pulse,
            "unseat within i_pulse (LEV-017)"
        );
        assert!(
            bias_current(d, d.suspension.g_land_m, 26.0) > i_pulse,
            "re-lift from 6.5 mm at m* is infeasible (F17)"
        );
        // Fail-down re-lift from the travel rollers works only up to ≈ 20 kg (plan §2b.4).
        let m20 = bias_current(d, d.suspension.g_land_m, 20.0);
        let m21 = bias_current(d, d.suspension.g_land_m, 21.0);
        assert!(
            m20 <= i_pulse && m21 > i_pulse,
            "re-lift limit ≈ 20 kg: i(20 kg) = {m20} A, i(21 kg) = {m21} A vs 12 A (plan §2b item 4)"
        );
    }

    #[test]
    fn stop_ratio_table() {
        // core §C.2 row "F/F* at stop / landing" and plan §2b item 19 (F18 0.863).
        let up = [2.16, 1.278, 1.26, 1.26];
        let land = [0.254, 0.764, 0.777, 0.777];
        let stay = [0.824, 0.849, 0.863, 0.863];
        for (k, c) in presets::CLASSES.iter().enumerate() {
            let p = presets::by_class(*c);
            let d = &p.design;
            let s = &d.suspension;
            let ms = d.carriage.m_star_kg;
            let r_up = stop_ratio(d, s.g_up_m, ms);
            let r_land = stop_ratio(d, s.g_land_m, ms);
            let r_stay = stop_ratio(d, s.g_land_m, d.carriage.m_op_min_kg);
            assert!(
                fmath::abs(r_up - up[k]) <= 0.005,
                "{c:?}: F/F* at stop {r_up} vs {} (±0.005; core §C.2)",
                up[k]
            );
            assert!(
                fmath::abs(r_land - land[k]) <= 0.0005,
                "{c:?}: F/F* at landing {r_land} vs {} (±0.0005; core §C.2)",
                land[k]
            );
            assert!(
                fmath::abs(r_stay - stay[k]) <= 0.0015,
                "{c:?}: stay-landed {r_stay} vs {} (±0.0015; F18, plan §2b.19)",
                stay[k]
            );
            let closed = {
                let r = (2.0 * s.g_star_m + hp_of(&p)) / (2.0 * s.g_up_m + hp_of(&p));
                r * r
            };
            assert!(
                rel(r_up, closed) <= 1e-12,
                "{c:?}: stop ratio = ((2g*+h′)/(2g+h′))² ((E); PHYSICS §3)"
            );
        }
    }

    #[test]
    fn k_i_and_inductance_table() {
        // core §C.2 rows "k_i" and "Coil N / L / R / τ".
        let ki = [7.9, 19.4, 41.5, 138.0];
        let l_mh = [2.0, 6.6, 13.0, 29.0];
        for (k, c) in presets::CLASSES.iter().enumerate() {
            let p = presets::by_class(*c);
            let s = &p.design.suspension;
            let got = k_i(s, s.g_star_m, 0.0);
            assert!(
                rel(got, ki[k]) <= 0.01,
                "{c:?}: k_i = {got} N/A vs {} (±1 %, 3 s.f.; core §C.2)",
                ki[k]
            );
            let l = coil_inductance(s) * 1e3;
            assert!(
                rel(l, l_mh[k]) <= 0.02,
                "{c:?}: L = {l} mH vs {} (±2 %, 2 s.f.; core §C.2)",
                l_mh[k]
            );
        }
    }

    #[test]
    fn size_magnets_matches_table() {
        // core §C.2: h_m ≈ 3.28 / 7.7 / 18 mm; pole 10×32 / 20×51 / 43×100 / 43×500 mm.
        let rows = [
            (26.0, 4.0, 2.5e-3, 0.50, 3.281_25e-3, 320.0e-6),
            (100.0, 4.0, 5.0e-3, 0.55, 7.7e-3, 20.0e-3 * 51.0e-3),
            (500.0, 4.0, 10.0e-3, 0.60, 18.0e-3, 43.0e-3 * 100.0e-3),
            (5000.0, 8.0, 10.0e-3, 0.60, 18.0e-3, 43.0e-3 * 500.0e-3),
        ];
        for (m, n, g, bg, hm, ap) in rows {
            let sz = size_magnets(m, n, g, bg, 1.3, 1.05);
            assert!(
                rel(sz.h_m_m, hm) <= 1e-9,
                "h_m = {} vs {hm} ((E) 1e-9; PHYSICS §3)",
                sz.h_m_m
            );
            assert!(
                rel(sz.a_p_m2, ap) <= 0.01,
                "A_p = {} vs {ap} (±1 %, rounded pole; core §C.2)",
                sz.a_p_m2
            );
        }
    }

    // req: FAB-LEV-007
    #[test]
    fn zp_gains_match_polynomial() {
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            let lt = &p.control.levitation;
            for mode in LevMode::ALL {
                let g = zp_gains(&p, mode);
                if !mode_active(&p, mode) {
                    assert_eq!(
                        g,
                        ZpGains::default(),
                        "{c:?} {mode:?}: passive mode has zero gains"
                    );
                    continue;
                }
                let mm = mode_model(&p, mode);
                let b = mm.k_i_n_per_a / mm.m_eff_kg;
                let p2 = mm.k_eff_n_per_m / mm.m_eff_kg;
                let w = zp_omega(&p, mode);
                // s³ + (b·k_d − κ)s² + (b·k_p − p²)s + p²κ vs the target polynomial.
                let got = [
                    b * g.kd_a_s_per_m - g.kappa_per_s,
                    b * g.kp_a_per_m - p2,
                    p2 * g.kappa_per_s,
                ];
                let want = [
                    (lt.alpha + 2.0 * lt.zeta) * w,
                    (1.0 + 2.0 * lt.zeta * lt.alpha) * w * w,
                    lt.alpha * w * w * w,
                ];
                for (gi, wi) in got.iter().zip(want.iter()) {
                    assert!(
                        rel(*gi, *wi) <= 1e-9,
                        "{c:?} {mode:?}: coefficient {gi} vs {wi} (1e-9; PHYSICS §15)"
                    );
                }
            }
            // ω = 2.5·max(p_mode, p_heave) (plan §2b item 1).
            let ph = mode_pole(&p, LevMode::Heave);
            assert!(
                rel(zp_omega(&p, LevMode::Heave), 2.5 * ph) <= 1e-12,
                "{c:?}: ω = 2.5·p (plan §2b.1)"
            );
        }
        // Old-table cross-check at ω = 3.5·p (core §C.4 DRAWER heave: 73.2 A/mm, 1077 A·s/m,
        // 894 1/s).
        let mut p = presets::drawer();
        p.control.levitation.omega_ratio = 3.5;
        let g = zp_gains(&p, LevMode::Heave);
        assert!(
            rel(g.kp_a_per_m, 73.2e3) <= 0.005,
            "k_p {} vs 73.2 A/mm (±0.5 %; core §C.4)",
            g.kp_a_per_m
        );
        assert!(
            rel(g.kd_a_s_per_m, 1077.0) <= 0.005,
            "k_d {} vs 1077 (±0.5 %; core §C.4)",
            g.kd_a_s_per_m
        );
        assert!(
            rel(g.kappa_per_s, 894.0) <= 0.005,
            "κ {} vs 894 (±0.5 %; core §C.4)",
            g.kappa_per_s
        );
    }

    fn pole_close(a: (f64, f64), b: (f64, f64)) -> f64 {
        fmath::hypot(a.0 - b.0, a.1 - b.1)
    }

    #[test]
    fn zp_discrete_places_target_poles() {
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            let t = fast_period_s(&p);
            for mode in LevMode::ALL {
                if !mode_active(&p, mode) {
                    continue;
                }
                let k = zp_gains_discrete(&p, mode);
                let poles =
                    zp_closed_loop_poles(&p, mode, &k, p.control.levitation.tau_e_nominal_s);
                for target in zp_target_poles_z(&p, mode) {
                    let best = poles
                        .iter()
                        .map(|q| pole_close(*q, target))
                        .fold(f64::INFINITY, fmath::fmin);
                    // In s: |Δz|/T within 1e-5 of ω (root conditioning of near-coincident
                    // real poles limits the reported roots to ~1e-7 in z).
                    let s_mag = zp_omega(&p, mode);
                    assert!(
                        best / t <= 1e-5 * s_mag,
                        "{c:?} {mode:?}: target {target:?} missed by {best} in z (1e-5 of ω·T; plan §2b.1)"
                    );
                }
                // LEV-007: ζ ≥ 0.5 for every closed-loop pole at the design point.
                for (re, im) in poles {
                    let s = lin::C::new(fmath::ln(fmath::hypot(re, im)), fmath::atan2(im, re))
                        .scale(1.0 / t);
                    let zeta = -s.re / s.norm();
                    assert!(
                        zeta >= 0.5,
                        "{c:?} {mode:?}: pole ζ = {zeta} < 0.5 (LEV-007)"
                    );
                }
            }
        }
    }

    #[test]
    fn zp_discrete_tends_to_continuous() {
        // Sampling, current lag and eddy lag → 0 recovers the continuous design (1e-3).
        let mut p = presets::door();
        p.control.rates.base_hz = 20_000_000;
        p.control.rates.fast_decim = 1;
        p.control.levitation.current_bw_hz = 1.0e7;
        p.control.levitation.tau_e_nominal_s = 0.0;
        let d = zp_gains_discrete(&p, LevMode::Heave);
        let c = zp_gains(&p, LevMode::Heave);
        assert!(
            rel(d.kp_a_per_m, c.kp_a_per_m) <= 1e-3,
            "k_p {} vs {} (1e-3; plan §2b.1)",
            d.kp_a_per_m,
            c.kp_a_per_m
        );
        assert!(
            rel(d.kd_a_s_per_m, c.kd_a_s_per_m) <= 1e-3,
            "k_d {} vs {} (1e-3)",
            d.kd_a_s_per_m,
            c.kd_a_s_per_m
        );
        assert!(
            rel(d.kappa_per_s, c.kappa_per_s) <= 1e-3,
            "κ {} vs {} (1e-3)",
            d.kappa_per_s,
            c.kappa_per_s
        );
    }

    #[test]
    fn zp_discrete_stable_to_2ms() {
        // LEV-020: stable for τ_e ≤ 2 ms in every class and active mode.
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            for mode in LevMode::ALL {
                if !mode_active(&p, mode) {
                    continue;
                }
                let k = zp_gains_discrete(&p, mode);
                for step in 0..=20 {
                    let te = 1.0e-4 * f64::from(step);
                    let rho = zp_spectral_radius(&p, mode, &k, te);
                    assert!(
                        rho < 1.0,
                        "{c:?} {mode:?}: ρ = {rho} at τ_e = {te} s (LEV-020: stable ≤ 2 ms)"
                    );
                }
                let crit = zp_critical_tau_e(&p, mode);
                assert!(
                    crit >= 2.0e-3,
                    "{c:?} {mode:?}: critical τ_e {crit} s < 2 ms (LEV-020)"
                );
            }
        }
    }

    #[test]
    fn zp_stable_across_float_band() {
        // Preset gains stay stable when the DRAWER mass spans 8–48 kg (the modal plant is
        // re-linearised at each float gap with its bias current).
        let p = presets::drawer();
        let k = zp_gains_discrete(&p, LevMode::Heave);
        for m in [8.0, 16.0, 26.0, 36.0, 48.0] {
            let mut q = p;
            let g = float_gap(&p, m);
            let i = bias_current(&p.design, g, m);
            // Re-express the float point as a balanced design at (g, m) with the same
            // k_i/k_s by shifting g* and m* (the modal model linearises at g*, m*).
            q.design.suspension.g_star_m = g;
            q.design.carriage.m_star_kg = m;
            q.design.suspension.b_r_t *= 1.0
                + p.design.suspension.turns * i
                    / m_pm(
                        p.design.suspension.b_r_t,
                        p.design.suspension.h_m_m,
                        p.design.suspension.mu_rec,
                    );
            let rho =
                zp_spectral_radius(&q, LevMode::Heave, &k, p.control.levitation.tau_e_nominal_s);
            assert!(
                rho < 1.0,
                "DRAWER {m} kg at {g} m: ρ = {rho} (stable across the float band, LEV-007)"
            );
        }
    }

    #[test]
    fn noise_force_gain_closed_form() {
        let p = presets::drawer();
        let k = noise_force_gain(&p);
        assert!(
            rel(k, 0.31e6) <= 0.01,
            "DRAWER K = {} N/µm vs ≈ 0.31 (±1 %; plan §2b.1, HAP-011)",
            k * 1e-6
        );
        let g = zp_gains(&p, LevMode::Heave);
        let mm = mode_model(&p, LevMode::Heave);
        assert!(
            rel(k, mm.k_i_n_per_a * g.kp_a_per_m) <= 1e-9,
            "K = k_i·k_p ((E) 1e-9; HAP-011)"
        );
    }

    #[test]
    fn thermal_gap_shift_values() {
        let want = [-0.195, -0.416, -0.891, -0.891];
        for (c, w) in presets::CLASSES.iter().zip(want) {
            let p = presets::by_class(*c);
            let got = thermal_gap_shift(&p, 40.0) * 1e3;
            assert!(
                fmath::abs(got - w) <= 5.0e-4,
                "{c:?}: +40 K Δg = {got} mm vs {w} (±0.5 µm; core §E.5, LEV-014)"
            );
        }
    }

    #[test]
    fn impact_speed_energy() {
        // Physics perspective A7 (1 %).
        let door = presets::door();
        let wall = presets::wall();
        let drawer = presets::drawer();
        for (p, up, down, name) in [
            (&door, 50.6e-3, 55.6e-3, "DOOR"),
            (&wall, 68.8e-3, 76.3e-3, "WALL"),
            (&drawer, 109.6e-3, 197.3e-3, "DRAWER@26 kg"),
        ] {
            let u = impact_speed(p, FailDir::Up);
            let d = impact_speed(p, FailDir::Down);
            assert!(
                rel(u, up) <= 0.01,
                "{name}: up {u} m/s vs {up} (±1 %; A7, LEV-019)"
            );
            assert!(
                rel(d, down) <= 0.01,
                "{name}: down {d} m/s vs {down} (±1 %; A7, LEV-019)"
            );
        }
        let s = &drawer.design.suspension;
        let g8 = zero_power_gap(8.0, s.g_star_m, hp_of(&drawer), 26.0);
        let v = impact_speed_at(&drawer, FailDir::Up, 8.0, g8);
        assert!(
            rel(v, 0.38) <= 0.02,
            "empty DRAWER snap-up {v} m/s vs 0.38 (±2 %; A7)"
        );
        // Energy identity (E): ½mv² equals the magnet work minus the potential energy.
        let m = drawer.design.carriage.m_star_kg;
        let v_up = impact_speed(&drawer, FailDir::Up);
        let e = 0.5 * m * v_up * v_up;
        let w = pm_work(s, s.g_up_m, s.g_star_m) - m * G0_MPS2 * (s.g_star_m - s.g_up_m);
        assert!(
            rel(e, w) <= 1e-12,
            "½mv² = ΔW′ − mgΔz ((E) 1e-12; PHYSICS §15)"
        );
    }

    #[test]
    fn pm_force_consistency() {
        let p = presets::door();
        let s = &p.design.suspension;
        // k_s and k_i are the exact derivatives of pm_force (finite differences, 1e-6).
        let g = s.g_star_m;
        let h = 1.0e-7;
        let fd_ks = -(pm_force(s, g + h, 0.0, 0.3) - pm_force(s, g - h, 0.0, 0.3)) / (2.0 * h);
        assert!(
            rel(fd_ks, k_s(s, g, 0.3)) <= 1e-6,
            "k_s = −∂F/∂g (FD 1e-6; PHYSICS §3)"
        );
        let fd_ki = (pm_force(s, g, 0.0, 0.3 + 1e-4) - pm_force(s, g, 0.0, 0.3 - 1e-4)) / 2e-4;
        assert!(
            rel(fd_ki, k_i(s, g, 0.3)) <= 1e-6,
            "k_i = ∂F/∂i (FD 1e-6; PHYSICS §3)"
        );
        // Lateral offset reduces the vertical force smoothly and continuously at y → 0.
        let f0 = pm_force(s, g, 0.0, 0.0);
        let fy = pm_force(s, g, 1.0e-9, 0.0);
        assert!(rel(fy, f0) <= 1e-9, "y → 0 continuity (1e-9; PHYSICS §2)");
        assert!(
            pm_force(s, g, 2.0e-3, 0.0) < f0,
            "offset reduces lift (PHYSICS §2)"
        );
        assert!(
            g_eff(g, 2.0e-3, s.pole_w_m) > g,
            "fringing lengthens the effective gap"
        );
    }

    // req: FAB-LEV-016
    #[test]
    fn extension_capacity_closed_form() {
        let p = presets::drawer();
        let at = |m: f64| extension_capacity(&p, m);
        let e48 = at(48.0);
        assert!(
            !e48.saturated,
            "48 kg at the 0.30 m preset travel is contactless (plan §2b.3)"
        );
        assert!(
            e48.max_extension_frac >= 0.5,
            "max extension at 48 kg {} ≥ 50 % (plan §2b.3)",
            e48.max_extension_frac
        );
        assert!(
            at(8.0).max_extension_frac > at(26.0).max_extension_frac,
            "capacity falls with mass"
        );
        assert!(
            at(26.0).max_extension_frac > e48.max_extension_frac,
            "capacity falls with mass"
        );
        let mut q = p;
        q.design.rail.travel_m = 0.45;
        assert!(
            extension_capacity(&q, 48.0).saturated,
            "(48 kg, 75 %) saturates (LEV-016)"
        );
        // Closed form: front group reach s = L/2 − x_A + l/2 − (F_A/F_max)·l.
        let s = &p.design.suspension;
        let x_a = s.stations[0].x_m;
        let f_a = 48.0 * G0_MPS2 / 4.0;
        let want = (0.3 - x_a + s.pole_l_m / 2.0 - f_a / element_force_max(s) * s.pole_l_m) / 0.6;
        assert!(
            rel(e48.max_extension_frac, want) <= 1e-12,
            "closed form ((E) 1e-12; PHYSICS §15)"
        );
    }

    #[test]
    fn speed_caps_and_stop_distance() {
        let caps = [(48.0, 0.2654), (100.0, 0.1838)];
        for (m, want) in caps {
            let v = v_low_energy(m, presets::PEDESTRIAN_E_MAX_J);
            assert!(
                fmath::abs(v - want) <= 5e-5,
                "√(3.38/{m}) = {v} vs {want} (±5e-5, 4 s.f.; SAF-002)"
            );
        }
        let v = momentum_cap(500.0, presets::PEDESTRIAN_P_MAX_KGMPS);
        assert!(fmath::abs(v - 0.0446) <= 1e-12, "22.3/500 = {v} ((E); F3)");
        // PRP-005 anchors use the 4-decimal caps (core §E.5).
        for (v, a, j, want_mm) in [
            (0.2654, 1.5, 20.0, 33.43),
            (0.1838, 0.75, 5.0, 36.31),
            (0.0446, 0.15, 1.5, 8.86),
            (0.025, 0.08, 1.0, 4.91),
        ] {
            let d = stop_distance(v, a, j) * 1e3;
            assert!(
                fmath::abs(d - want_mm) <= 5e-3,
                "d_stop({v}) = {d} mm vs {want_mm} (±5 µm, 4 s.f.; core §E.5)"
            );
        }
        // Branch continuity at v = a²/j (E).
        let (a, j) = (1.5, 20.0);
        let vb = a * a / j;
        let d1 = vb * vb / (2.0 * a) + vb * a / (2.0 * j);
        let d2 = vb * fmath::sqrt(vb / j);
        assert!(
            rel(d1, d2) <= 1e-12,
            "stop-distance branches meet at v = a²/j ((E); PHYSICS §16)"
        );
    }

    #[test]
    fn clamp_force_values() {
        let e = presets::PEDESTRIAN_E_MAX_J;
        for (k, want) in [(25.0e3, 290.7), (75.0e3, 503.5)] {
            let f = clamp_force(e, k);
            assert!(
                fmath::abs(f - want) <= 0.05,
                "√(2·1.69·{k}) = {f} N vs {want} (±0.05 N; core §E.5, SAF-005)"
            );
        }
        let f = clamp_force(e, series_k(75.0e3, 10.0e3));
        assert!(
            fmath::abs(f - 173.0) <= 0.5,
            "hand ⊕ 10 N/mm edge {f} N vs 173 (±0.5 N; F6)"
        );
        let wall = clamp_force(bumper_rating(500.0, 0.0446), 25.0e3);
        assert!(
            fmath::abs(wall - 157.7) <= 0.05,
            "WALL chest {wall} N vs 157.7 (±0.05 N; core §E.5)"
        );
        let heavy = clamp_force(bumper_rating(5000.0, 0.025), series_k(75.0e3, 10.0e3));
        assert!(
            fmath::abs(heavy - 166.0) <= 0.5,
            "HEAVY hand ⊕ edge {heavy} N vs 166 (±0.5 N; F6)"
        );
        // ISO/TS 15066: 500 kg chest (40 kg, 25 N/mm) at 140 / 280 N → 0.145 / 0.291 m/s.
        let v1 = iso15066_speed(140.0, 40.0, 500.0, 25.0e3);
        let v2 = iso15066_speed(280.0, 40.0, 500.0, 25.0e3);
        assert!(
            fmath::abs(v1 - 0.145) <= 5e-4 && fmath::abs(v2 - 0.291) <= 5e-4,
            "ISO/TS 15066 {v1} / {v2} m/s (±5e-4; A13)"
        );
    }

    #[test]
    fn unattended_close_speed_value() {
        let k = series_k(75.0e3, 10.0e3);
        let v = unattended_close_speed(0.2654, 150.0, 48.0, k);
        assert!(
            fmath::abs(v - 0.23) <= 5e-3,
            "unattended close at 48 kg = {v} m/s vs 0.23 (±5 mm/s, 2 s.f.; plan §4b)"
        );
        assert!(
            rel(v, 150.0 / fmath::sqrt(48.0 * k)) <= 1e-12,
            "F/√(m·k) branch binds ((E); plan §4b)"
        );
        assert_eq!(
            unattended_close_speed(0.1, 150.0, 48.0, k),
            0.1,
            "v_cap binds when lower"
        );
    }

    // req: FAB-HAP-016
    #[test]
    fn terminus_t_arrive_table() {
        // plan §4b corrected table (±1 %): (v, j, τ) → d_0, d_b [mm], t_arrive [s].
        let rows = [
            ("DRAWER", 0.2654, 20.0, 0.15, 49.8, 32.1, 0.98),
            ("cabinet slider", 0.152, 5.0, 0.2, 40.0, 23.5, 1.27),
            ("DOOR close", 0.152, 5.0, 0.25, 46.1, 31.6, 1.62),
            ("DOOR open", 0.1838, 5.0, 0.25, 57.6, 37.2, 1.68),
            ("WALL", 0.0446, 1.5, 0.6, 27.8, 25.7, 3.56),
            ("HEAVY", 0.025, 1.0, 1.0, 25.3, 24.7, 5.84),
        ];
        for (name, v, j, tau, d0, db, ta) in rows {
            let b = terminus_blend(v, j, tau);
            let t = terminus_t_arrive(v, j, tau, 0.2e-3);
            assert!(
                rel(b.d_0_m * 1e3, d0) <= 0.01,
                "{name}: d_0 = {} mm vs {d0} (±1 %; plan §4b)",
                b.d_0_m * 1e3
            );
            assert!(
                rel(b.d_b_m * 1e3, db) <= 0.01,
                "{name}: d_b = {} mm vs {db} (±1 %; plan §4b)",
                b.d_b_m * 1e3
            );
            assert!(
                rel(t, ta) <= 0.01,
                "{name}: t_arrive = {t} s vs {ta} (±1 %; plan §4b, HAP-016)"
            );
            // Continuity at the blend end (E): speed d_b/τ and deceleration d_b/τ².
            let v_end = v - j * b.t_b_s * b.t_b_s / 2.0;
            assert!(
                rel(v_end, b.d_b_m / tau) <= 1e-12,
                "{name}: C¹ at t_b ((E); PHYSICS §16.1)"
            );
            assert!(
                rel(j * b.t_b_s, b.d_b_m / (tau * tau)) <= 1e-12,
                "{name}: C² at t_b ((E); PHYSICS §16.1)"
            );
        }
        // Presets: the same closed form from the preset caps and termini.
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            let tm = &p.control.motion.terminus;
            let v = p.safety.v_close_max_mps;
            let t = terminus_t_arrive(v, tm.j_mps3, tm.tau_s, tm.d_eps_m);
            let b = terminus_blend(v, tm.j_mps3, tm.tau_s);
            let manual = b.t_b_s + tm.tau_s * fmath::ln(b.d_b_m / tm.d_eps_m) + tm.tau_s;
            assert!(
                rel(t, manual) <= 1e-12,
                "{c:?}: t_arrive closed form ((E); HAP-016)"
            );
        }
    }

    #[test]
    fn tau_min_values() {
        let taus = [0.15, 0.25, 0.6, 1.0];
        for (c, tau) in presets::CLASSES.iter().zip(taus) {
            let p = presets::by_class(*c);
            let mo = &p.control.motion;
            let v = fmath::fmax(p.safety.v_open_max_mps, p.safety.v_close_max_mps);
            let tmin = tau_min(v, mo.a_max_mps2, mo.terminus.j_mps3);
            assert!(
                tmin <= tau,
                "{c:?}: τ_min {tmin} s ≤ τ {tau} (PHYSICS §16.1)"
            );
            assert_eq!(mo.terminus.tau_s, tau, "{c:?}: preset τ (plan §4b)");
            let wv2 = PI * mo.omega_v_hz;
            assert!(wv2 >= 3.0 / tau, "{c:?}: ω_v/2 = {wv2} ≥ 3/τ (plan §4b)");
        }
    }

    // req: FAB-SAF-020
    #[test]
    fn t_min_iso_uses_f_cont() {
        let p = presets::wall();
        let t = t_min_iso(500.0, 0.2, p.design.lsm.f_cont_n);
        assert!(
            fmath::abs(t - 6.3) <= 0.05,
            "WALL T_min with F_cont = {t} s vs 6.3 (±0.05 s; plan §2b.5)"
        );
        assert!(
            rel(t, 2.0 * PI) <= 1e-12,
            "2π·√(500·0.2/100) = 2π ((E); PHYSICS §21)"
        );
        let t_peak = t_min_iso(500.0, 0.2, p.design.lsm.f_peak_n);
        assert!(
            rel(t_peak, PI) <= 1e-12,
            "with F_peak it would be π ≈ 3.14 s (the superseded core §E.5 value)"
        );
        assert!(
            p.control.seismic.isolation.t_iso_s >= 1.1 * t - 1e-9,
            "preset T_iso ≥ 1.1·T_min(F_cont) (core §C.2 isolation row)"
        );
    }

    #[test]
    fn seismic_anchors() {
        for (m, r, want_cm) in [
            (7.0, 20.0, 0.7118),
            (6.5, 30.0, 0.1762),
            (5.0, 10.0, 0.0643),
            (7.8, 100.0, 0.2987),
        ] {
            let pd = pd_wu_zhao(m, r) * 100.0;
            assert!(
                rel(pd, want_cm) <= 1e-4 * 5.0,
                "Wu–Zhao M{m} R{r} = {pd} cm vs {want_cm} (4 s.f. ≈ 5e-4; SEI-012)"
            );
        }
        let sp = sp_time(1.0, 6.0, 3.5);
        assert!(
            fmath::abs(sp - 0.1190) <= 5e-5,
            "S−P = {sp} s/km vs 0.1190 (4 s.f.; SEI-012)"
        );
        assert!(
            rel(sp, 1.0 / 3.5 - 1.0 / 6.0) <= 1e-12,
            "S−P closed form ((E) 1e-9 class; A19)"
        );
        let x = lead_crossover_km(6.0, ONSITE_DECISION_S, NETWORK_ALERT_S);
        assert!(
            fmath::abs(x - 40.8) <= 1e-9,
            "crossover {x} km vs 40.8 ((E); SEI-012)"
        );
        let on = lead_time(x, 3.5, x / 6.0 + ONSITE_DECISION_S);
        let net = lead_time(x, 3.5, NETWORK_ALERT_S);
        assert!(
            fmath::abs(on - net) <= 1e-12,
            "equal lead times at the crossover ((E); A19)"
        );
    }

    #[test]
    fn tmd_and_isolation_anchors() {
        let (f, z) = den_hartog(0.02);
        assert!(
            fmath::abs(f - 0.98039) <= 5e-6 && fmath::abs(z - 0.08407) <= 5e-6,
            "Den Hartog μ 0.02: {f} / {z} (1e-5; SEI-016)"
        );
        let (f, z) = den_hartog(0.01);
        assert!(
            fmath::abs(f - 0.99010) <= 5e-6 && fmath::abs(z - 0.06033) <= 5e-6,
            "Den Hartog μ 0.01: {f} / {z} (1e-5; plan §4)"
        );
        assert!(
            fmath::abs(den_hartog_peak(0.02) - 10.05) <= 5e-3,
            "Den Hartog peak 10.05 (4 s.f.; SEI-016)"
        );
        assert!(
            fmath::abs(den_hartog_peak(0.01) - 14.177) <= 5e-4,
            "Den Hartog peak 14.177 (plan §4)"
        );
        let (f, z) = warburton(0.02);
        assert!(
            fmath::abs(f - 0.97548) <= 5e-6 && fmath::abs(z - 0.07019) <= 5e-6,
            "Warburton base μ 0.02: {f} / {z} (1e-5; SEI-016)"
        );
        let (f, z) = warburton(0.01);
        assert!(
            fmath::abs(f - 0.98762) <= 5e-6 && fmath::abs(z - 0.04981) <= 5e-6,
            "Warburton base μ 0.01: {f} / {z} (1e-5; plan §4)"
        );
        let (f1, _) = warburton_force(0.01);
        let (f2, _) = warburton_force(0.02);
        assert!(
            fmath::abs(f1 - 0.99257) <= 5e-6 && fmath::abs(f2 - 0.98528) <= 5e-6,
            "Warburton force {f1} / {f2} (1e-5; plan §4)"
        );
        let t = transmissibility(3.0, 0.2);
        assert!(
            fmath::abs(t - 0.1931) <= 5e-5,
            "T(3, 0.2) = {t} vs 0.1931 (4 s.f.; SEI-014)"
        );
        assert!(
            fmath::abs(transmissibility(fmath::sqrt(2.0), 0.37) - 1.0) <= 1e-12,
            "T(√2) = 1 for any ζ ((E); A15)"
        );
        assert!(
            fmath::abs(transmissibility(1.0, 0.2) - 2.693) <= 5e-4,
            "T(1, 0.2) = 2.693 (A15)"
        );
        assert!(
            rel(t_sky(1.0, 0.5), 1.0) <= 1e-12,
            "T_sky(1, 0.5) = 1/(2ζ) = 1 ((E); PHYSICS §16.2)"
        );
        assert!(t_sky(5.0, 0.7) < 0.05, "T_sky ≈ 0.04 at 5·f_c (plan §4b)");
    }

    #[test]
    fn piston_force_anchor() {
        let f = piston_force(0.06, 0.5, 40.0e-4);
        assert!(
            rel(f, 5.6) <= 0.05,
            "40 cm² reveal at 0.5 m/s → {f} N vs 5.6 (±5 %; ENV-003, research C15)"
        );
        let f10 = piston_force(0.06, 0.5, 10.0e-4);
        assert!(
            rel(f10, 90.0) <= 0.05,
            "10 cm² → {f10} N vs 90 (±5 %; research C15)"
        );
    }

    #[test]
    fn passivity_and_holdup() {
        let p = presets::drawer();
        let t = fast_period_s(&p);
        assert!(
            rel(passivity_k_max(2.0, t, 0.0), 40.0e3) <= 1e-12,
            "2b/T = 40 kN/m ((E); core §C.2)"
        );
        assert!(
            rel(passivity_k_max(2.0, t, t), 40.0e3 / 3.0) <= 1e-12,
            "2b/(T + 2T_d) with T_d = T ((E); plan §2b.19)"
        );
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            let h = holdup_energy(&p);
            assert!(
                h.avail_j >= p.control.supervisor.holdup_margin * h.need_j,
                "{c:?}: hold-up {} J ≥ 3 × {} J (SAF-009)",
                h.avail_j,
                h.need_j
            );
        }
        let door = presets::door();
        let f = manual_force(&door, false);
        assert!(f <= 22.2, "DOOR no-PM manual force {f} N ≤ 22.2 (SAF-010)");
        assert!(
            manual_force(&door, true) < f,
            "PM relief lowers the manual force (F8)"
        );
    }

    #[test]
    fn derived_table_is_finite_and_named() {
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            let t = derived(&p);
            for (i, (name, v)) in t.iter().enumerate() {
                assert!(!name.is_empty(), "entry {i} has a name");
                assert!(v.is_finite(), "{c:?}: derived {name} = {v} must be finite");
                for (other, _) in t.iter().skip(i + 1) {
                    assert_ne!(name, other, "derived names are unique");
                }
            }
            let pole = t.iter().find(|(n, _)| *n == "pole_rad_s").map(|(_, v)| *v);
            assert!(pole.is_some(), "pole_rad_s present (gate G0)");
        }
        let door = derived(&presets::door());
        let pole = door
            .iter()
            .find(|(n, _)| *n == "pole_rad_s")
            .map(|(_, v)| *v)
            .unwrap_or(0.0);
        assert!(
            fmath::abs(pole - 47.57) <= 0.005,
            "gate G0: DOOR pole_rad_s ≈ 47.57 ({pole})"
        );
    }

    #[test]
    fn lsm_ke_and_asc_drag() {
        // D5 (PHYSICS §7): amplitude-invariant dq, K_e = ⅔·K_f, so F·v = 1.5·e_q·i_q, and
        // the shorted-terminal steady state gives F = −b_e·v with b_e = K_f·K_e/R/(1 + x²).
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            let l = p.design.lsm;
            assert!(
                rel(l.k_e_v_s_per_m, 2.0 / 3.0 * l.k_f_n_per_a) <= 1e-12,
                "{c:?}: K_e = ⅔·K_f (rel 1e-12; D5, PRP-002)"
            );
            for &(v, iq) in &[(0.2f64, 3.0f64), (-0.05, 1.5), (1.0, -4.0)] {
                let f = l.k_f_n_per_a * iq;
                let eq = l.k_e_v_s_per_m * v;
                assert!(
                    fmath::abs(f * v - 1.5 * eq * iq) <= 1e-12 * fmath::abs(f * v),
                    "{c:?}: F·v = 1.5·e_q·i_q (rel 1e-12; PRP-002)"
                );
            }
            let (r, lh) = (l.r_phase_ohm, l.l_phase_h);
            for &v in &[0.01f64, 0.3, 2.0] {
                let w = PI * v / l.tau_p_m;
                // Shorted dq: 0 = R·i_d − ω·L·i_q, 0 = R·i_q + ω·L·i_d + e_q.
                let eq = l.k_e_v_s_per_m * v;
                let den = r * r + (w * lh) * (w * lh);
                let iq = -eq * r / den;
                let id = w * lh / r * iq;
                let f = l.k_f_n_per_a * iq;
                let b = asc_drag(l.k_f_n_per_a, l.k_e_v_s_per_m, r, lh, w);
                assert!(
                    rel(-f, b * v) <= 1e-12,
                    "{c:?}: ASC F = −b_e·v at {v} m/s (rel 1e-12; D5)"
                );
                let p_cu = 1.5 * r * (id * id + iq * iq);
                assert!(
                    rel(p_cu, b * v * v) <= 1e-9,
                    "{c:?}: ASC copper loss = b_e·v² (rel 1e-9; energy balance, PHYSICS §7)"
                );
            }
            assert!(
                rel(
                    asc_drag_low_speed(&p),
                    2.0 / 3.0 * l.k_f_n_per_a * l.k_f_n_per_a / r
                ) <= 1e-12,
                "{c:?}: b_e(0) = ⅔·K_f²/R (rel 1e-12; D5)"
            );
        }
        assert_eq!(
            asc_drag(10.0, 5.0, 0.0, 1e-3, 0.0),
            0.0,
            "R = 0 gives no drag"
        );
        assert!(
            rel(asc_drag(15.0, 10.0, 2.0, 0.0, 0.0), 75.0) <= 1e-15,
            "DRAWER b_e(0) = 15·10/2 = 75 N·s/m (exact; D5)"
        );
    }

    /// 13-state closed loop of the coupled design on a (possibly mismatched) plant:
    /// plant x (6), u_d, q, observer predictions [u, u̇, Y, Ẏ, i_e].
    #[allow(clippy::needless_range_loop)]
    fn rs_closed_loop(
        d: &RollSwayDesign,
        t: f64,
        phi_t: &[[f64; 6]; 6],
        gam_t: &[f64; 6],
    ) -> Vec<Vec<f64>> {
        const NZ: usize = 13;
        const UD: usize = 6;
        const Q: usize = 7;
        const PM: [usize; 4] = [8, 9, 10, 11];
        const PE: usize = 12;
        let unit = |k: usize| {
            let mut v = vec![0.0; NZ];
            v[k] = 1.0;
            v
        };
        let add = |a: &[f64], b: &[f64], k: f64| {
            a.iter()
                .zip(b)
                .map(|(x, y)| x + k * y)
                .collect::<Vec<f64>>()
        };
        let mut m = vec![vec![0.0; NZ]; NZ];
        for r in 0..6 {
            for c in 0..6 {
                m[r][c] = phi_t[r][c];
            }
            m[r][UD] = gam_t[r];
        }
        let e = add(&unit(0), &unit(PM[0]), -1.0);
        let xh: Vec<Vec<f64>> = (0..4).map(|k| add(&unit(PM[k]), &e, d.l[k])).collect();
        let mut xf = xh.clone();
        xf.push(unit(4));
        xf.push(unit(PE));
        let mut cmd = vec![0.0; NZ];
        for k in 0..4 {
            cmd = add(&cmd, &xh[k], d.k[k]);
        }
        cmd = add(&cmd, &unit(Q), d.k[4]);
        m[UD] = cmd;
        m[Q] = add(&unit(Q), &unit(4), t);
        let pred = |row: usize| {
            let mut v = add(&[0.0; NZ], &unit(UD), d.gam[row]);
            for c in 0..6 {
                v = add(&v, &xf[c], d.phi[row][c]);
            }
            v
        };
        for k in 0..4 {
            m[PM[k]] = pred(k);
        }
        m[PE] = pred(5);
        m
    }

    /// Norm ratio after `n` steps from a generic initial state (stable ⇔ decays).
    fn decay_ratio(m: &[Vec<f64>], n: usize) -> f64 {
        let nz = m.len();
        let mut x: Vec<f64> = (0..nz).map(|k| 1.0 + 0.37 * k as f64).collect();
        let n0 = x.iter().map(|v| v * v).sum::<f64>().sqrt();
        let mut log_scale = 0.0f64;
        for _ in 0..n {
            let y: Vec<f64> = m
                .iter()
                .map(|row| row.iter().zip(&x).map(|(a, b)| a * b).sum())
                .collect();
            let ny = y.iter().map(|v| v * v).sum::<f64>().sqrt();
            if !ny.is_finite() || ny == 0.0 {
                return if ny == 0.0 { 0.0 } else { f64::INFINITY };
            }
            log_scale += ny.ln();
            x = y.iter().map(|v| v / ny).collect();
        }
        (log_scale - n0.ln()).exp()
    }

    #[test]
    fn roll_sway_coupled_design() {
        // D1 (PHYSICS §6, §15): HEAVY's roll loop is designed on the coupled roll–sway
        // model; DRAWER keeps the single-DOF roll design; top-hung classes have no roll loop.
        let heavy = presets::heavy();
        assert!(
            roll_sway_coupling(&heavy) > 1.5,
            "HEAVY coupling ≈ 1.97 (design value)"
        );
        assert!(
            roll_sway_coupled(&heavy),
            "HEAVY roll on the coupled design"
        );
        assert!(
            !roll_sway_coupled(&presets::drawer()),
            "DRAWER coupling 0.032 < 0.25"
        );
        assert!(!roll_sway_coupled(&presets::door()) && !roll_sway_coupled(&presets::wall()));
        let d = roll_sway_design(&heavy).expect("HEAVY coupled design");
        let t = fast_period_s(&heavy);
        // The five placed poles are roots of the eight-state loop (|χ| relative 1e-6).
        let m = roll_sway_loop(&d.phi, &d.gam, t, &d.k);
        for (re, im) in roll_sway_target_poles_s(&heavy) {
            let z = lin::C::new(re * t, im * t).cexp();
            let chi = charpoly_at8(&m, z).norm();
            let scale = charpoly_at8(&roll_sway_loop(&d.phi, &d.gam, t, &[0.0; 5]), z).norm();
            assert!(
                chi <= 1e-6 * scale,
                "target pole ({re}, {im}) placed: |χ| {chi} vs {scale}"
            );
        }
        assert!(
            d.k.iter().all(|v| v.is_finite()) && d.k[0] > 0.0 && d.k[4] > 0.0,
            "finite gains, positive gap and integral gains: {:?}",
            d.k
        );
        // Stable on the coupled plant across eddy lag 0–2 ms (LEV-020), lateral stiffness
        // ±25 %, sway damping 0–3× and CoM height ±10 % (robustness sweep, D1).
        let lt = &heavy.control.levitation;
        let tau_cl = tau_current_loop_s(&heavy);
        for &te in &[0.0, 1.0e-3, 2.0e-3] {
            for &ky in &[0.75, 1.0, 1.25] {
                for &cs in &[0.0, 1.0, 3.0] {
                    for &hs in &[0.9, 1.0, 1.1] {
                        let mut tm = d.model;
                        tm.k_y_n_per_m *= ky;
                        tm.c_n_s_per_m *= cs;
                        tm.k_neg_nm_per_rad += tm.m_kg * G0_MPS2 * tm.h_m * (hs - 1.0);
                        tm.h_m *= hs;
                        let (pt, gt) = tm.zoh(t, tau_cl, te);
                        let r = decay_ratio(&rs_closed_loop(&d, t, &pt, &gt), 100_000);
                        assert!(
                            r < 1e-6,
                            "coupled loop decays (×{r:e} over 20 s; < 1e-6) at τ_e {te}, k_y ×{ky}, c ×{cs}, h ×{hs} (LEV-020, D1)"
                        );
                    }
                }
            }
        }
        // The single-DOF roll design diverges on the same coupled plant (the W1.5 finding):
        // its closed loop grows (spectral radius 1.0019 in the Python analysis).
        let k1 = zp_gains_discrete(&heavy, LevMode::Roll);
        let d1 = RollSwayDesign {
            k: [k1.kp_a_per_m, k1.kd_a_s_per_m, 0.0, 0.0, k1.kappa_per_s],
            l: {
                // Single-DOF observer: only the roll states are corrected, and its model has
                // no sway coupling (k_y = c = 0).
                let mut m1 = d.model;
                m1.k_y_n_per_m = 0.0;
                m1.c_n_s_per_m = 0.0;
                let (p1, _) = m1.zoh(t, tau_cl, lt.tau_e_nominal_s);
                let w = zp_omega(&heavy, LevMode::Roll);
                // place2 of levitation::observer: z² − s1·z + s0 at (ω_o, ζ), L_c = Φ⁻¹·L_p.
                let (a, b, c, dd) = (p1[0][0], p1[0][1], p1[1][0], p1[1][1]);
                let (wo, z) = (lt.observer_ratio * w, lt.zeta);
                let q = fmath::exp(-z * wo * t);
                let s1 = 2.0 * q * fmath::cos(wo * fmath::sqrt(1.0 - z * z) * t);
                let s0 = q * q;
                let lp0 = a + dd - s1;
                let lp1 = (s0 - (a - lp0) * dd + b * c) / b;
                let det = a * dd - b * c;
                [
                    (dd * lp0 - b * lp1) / det,
                    (-c * lp0 + a * lp1) / det,
                    0.0,
                    0.0,
                ]
            },
            ..{
                let mut m1 = d.model;
                m1.k_y_n_per_m = 0.0;
                m1.c_n_s_per_m = 0.0;
                let (phi, gam) = m1.zoh(t, tau_cl, lt.tau_e_nominal_s);
                RollSwayDesign { phi, gam, ..d }
            }
        };
        let (pt, gt) = d.model.zoh(t, tau_cl, lt.tau_e_nominal_s);
        let r = decay_ratio(&rs_closed_loop(&d1, t, &pt, &gt), 5_000);
        assert!(
            r > 1.0,
            "single-DOF roll design grows on the coupled HEAVY plant (×{r:e} over 1 s)"
        );
    }
}
