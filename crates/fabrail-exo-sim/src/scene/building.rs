//! Building models of the scene: fixed-base shear building, linear-equivalent FPS isolation, HAVEN-F reduced platform, SENTINEL tower (plan §4b; PHYSICS §21, EXOTIC §X6, §X9).
//!
//! One-way coupling: the building is integrated once over the whole ground record before
//! the devices run (devices never load the structure), and each floor's absolute
//! acceleration (building x, y, z) is stored at [`BUILDING_DT_S`].
//!
//! **Model** (PHYSICS §21). Lumped shear building in coordinates relative to the ground,
//! M·ü + C·u̇ + K·u = −M·1·a_g(t) + f(t), the same model in x and in y (independent
//! horizontal directions); a floor's absolute acceleration is ü + a_g. Floors with
//! z_m > 0 are storeys (sorted by height, then level); floors at z_m = 0 are the ground
//! (fixed base) or the base slab above the isolation plane (isolated configurations).
//! Storey masses default to [`DEFAULT_FLOOR_MASS_KG`]; unknown storey stiffnesses share
//! one value scaled so the fixed-base fundamental period equals `period_s`
//! (bisection on the first eigenvalue). Vertical: every floor follows the ground
//! (axially rigid; vertical amplification is not modelled \[SIMPLIFIED\]). The ground
//! input is scaled by `topo_factor` in all three axes.
//!
//! * **fixed_base** (🔶): Rayleigh damping ζ at modes 1 and 2 (Chopra, *Dynamics of
//!   Structures*, 4th ed., Eq. 11.4.9); one storey: c = 2ζ√(k·m).
//! * **fps_isolated** (🔶): friction pendulum as a linear-equivalent isolator
//!   (Zayas, Low & Mahin 1990: T = 2π√(R/g), so R = g·T²/4π² with T = `iso_period_s`;
//!   Naeim & Kelly, *Design of Seismic Isolated Structures*, 1999, FPS chapter:
//!   k_eff = W/R + μW/D, β_eff = (2/π)·μ/(μ + D/R)). D starts at `iso_stroke_m` (else
//!   0.1 m) and is iterated on the record's own peak isolator displacement
//!   ([`FPS_ITERATIONS`] passes, the classical equivalent-linear iteration). The
//!   superstructure has stiffness-proportional damping ζ at its first mode (Rayleigh
//!   mass-proportional damping relative to the ground would damp the isolation mode).
//! * **haven_f** (🧪, reduced platform): linear isolation k = M_tot·(2π/T)², ζ_iso;
//!   `passive` damps the isolator relative motion, `skyhook` damps the base slab's
//!   absolute velocity (ground velocity from a leaky integral with a 0.1 Hz high-pass,
//!   as core Isolation and HAVEN; PHYSICS §16.2). A stiff end stop engages beyond
//!   `stroke_m` ([`STOP_STIFFNESS_RATIO`]·k); saturation is reported.
//! * **tower_sentinel** (🧪): the tower's storeys plus the SENTINEL damper on the top
//!   storey, mass μ·M_s, tuned with the Warburton base-excitation optimum
//!   (`fabrail_exo::tuning::damper_design_for`, EXOTIC §X7) to the fixed-base first
//!   mode. `landed_fps` locks the damper mass to the roof; every other mode is modelled
//!   as the passive tuned damper (the active HMD law of `fabrail_exo::hmd` is not run in
//!   the scene: reduced model).
//!
//! Integration: Newmark average acceleration (γ = 1/2, β = 1/4; unconditionally stable;
//! Chopra Table 5.4.2) at 1 ms with the effective stiffness factorised once; the few
//! non-linear or state-feedback forces (end stop, skyhook) are explicit.

use fabrail_exo::hmd::HmdMode;
use fabrail_exo::tuning::{TuningForm, damper_design_for};
use fabrail_params::derive::G0_MPS2;

use super::{BuildingConfig, FloorSpec, IsolationControl};

/// Building integration step, s (1 kHz, as the Structure tier; PHYSICS §24; the devices
/// sync at 100 Hz).
pub(crate) const BUILDING_DT_S: f64 = 1.0e-3;

/// Default storey mass when a floor gives none, kg. With stiffnesses scaled to
/// `period_s` the fixed-base response does not depend on it; it sets the isolation and
/// damper mass scales (\[UNCERTAIN\] design choice: a 100 t villa floor; PHYSICS §21).
pub(crate) const DEFAULT_FLOOR_MASS_KG: f64 = 1.0e5;

/// Equivalent-linear FPS passes on the record's own peak displacement (Naeim & Kelly
/// 1999 iterate until D converges; 4 passes change D by < 1 % on the villa records;
/// PHYSICS §21).
pub(crate) const FPS_ITERATIONS: usize = 4;

/// Initial FPS design displacement when `iso_stroke_m` is not given, m (\[UNCERTAIN\]
/// start value of the iteration; PHYSICS §21).
const FPS_D0_M: f64 = 0.1;

/// Smallest FPS design displacement used in the iteration, m (keeps k_eff = W/R + μW/D
/// finite; PHYSICS §21).
const FPS_D_MIN_M: f64 = 1.0e-3;

/// HAVEN-F end-stop stiffness as a multiple of the isolation stiffness (a stiff
/// buffer; \[UNCERTAIN\] design choice; EXOTIC §X9, PHYSICS §21).
pub(crate) const STOP_STIFFNESS_RATIO: f64 = 50.0;

/// Skyhook ground-velocity high-pass, Hz (core Isolation / HAVEN leaky integral, 0.1 Hz;
/// PHYSICS §16.2, §21).
const SKYHOOK_HP_HZ: f64 = 0.1;

/// Summary numbers of the building model (reported in the scene KPIs).
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct BuildingInfo {
    /// Fixed-base fundamental period of the (super)structure, s.
    pub period_s: f64,
    /// Isolation: effective period, s (0 for fixed base / tower).
    pub iso_t_eff_s: f64,
    /// Isolation: effective damping ratio.
    pub iso_beta_eff: f64,
    /// Isolation: peak isolator displacement (vector), m.
    pub iso_d_peak_m: f64,
    /// Isolation: time beyond the stroke (end stop engaged), s.
    pub iso_saturated_s: f64,
    /// SENTINEL damper mass, kg (0 otherwise).
    pub hmd_mass_kg: f64,
    /// SENTINEL damper peak stroke, m.
    pub hmd_stroke_peak_m: f64,
}

/// The building response: absolute floor accelerations (building frame x, y, z) at
/// [`BUILDING_DT_S`] from t = 0, per floor in spec order.
#[derive(Clone, Debug, Default)]
pub(crate) struct BuildingResponse {
    /// Floor levels in spec order.
    pub levels: Vec<u32>,
    /// `acc[f][k]` = floor f absolute acceleration at t = k·dt, m/s².
    pub acc: Vec<Vec<[f64; 3]>>,
    /// Ground acceleration (topo-scaled) at t = k·dt, m/s².
    pub ground: Vec<[f64; 3]>,
    /// Summary.
    pub info: BuildingInfo,
}

impl BuildingResponse {
    /// Index of floor `level`.
    pub(crate) fn floor_index(&self, level: u32) -> Option<usize> {
        self.levels.iter().position(|&l| l == level)
    }

    /// Floor `f` acceleration at time `t_s` (linear interpolation; 0 outside the run).
    pub(crate) fn floor_at(&self, f: usize, t_s: f64) -> [f64; 3] {
        interp(self.acc.get(f).map_or(&[][..], Vec::as_slice), t_s)
    }

    /// Ground acceleration at `t_s`.
    pub(crate) fn ground_at(&self, t_s: f64) -> [f64; 3] {
        interp(&self.ground, t_s)
    }
}

fn interp(v: &[[f64; 3]], t_s: f64) -> [f64; 3] {
    if v.is_empty() || !t_s.is_finite() || t_s < 0.0 {
        return [0.0; 3];
    }
    let x = t_s / BUILDING_DT_S;
    let k = x.floor() as usize;
    let Some(a) = v.get(k) else {
        return v.last().copied().unwrap_or([0.0; 3]);
    };
    let b = v.get(k + 1).unwrap_or(a);
    let w = x - k as f64;
    [
        a[0] + w * (b[0] - a[0]),
        a[1] + w * (b[1] - a[1]),
        a[2] + w * (b[2] - a[2]),
    ]
}

// ---- Small dense linear algebra --------------------------------------------------------

/// Row-major n×n matrix.
#[derive(Clone, Debug, PartialEq)]
struct Mat {
    n: usize,
    a: Vec<f64>,
}

impl Mat {
    fn zeros(n: usize) -> Mat {
        Mat {
            n,
            a: vec![0.0; n * n],
        }
    }
    fn at(&self, i: usize, j: usize) -> f64 {
        self.a[i * self.n + j]
    }
    fn add(&mut self, i: usize, j: usize, v: f64) {
        self.a[i * self.n + j] += v;
    }
    fn mul_vec(&self, x: &[f64]) -> Vec<f64> {
        (0..self.n)
            .map(|i| (0..self.n).map(|j| self.at(i, j) * x[j]).sum())
            .collect()
    }
}

/// LU factorisation with partial pivoting (Golub & Van Loan, Alg. 3.4.1).
#[derive(Clone, Debug)]
struct Lu {
    m: Mat,
    piv: Vec<usize>,
}

impl Lu {
    fn new(mut m: Mat) -> Option<Lu> {
        let n = m.n;
        let mut piv: Vec<usize> = (0..n).collect();
        for k in 0..n {
            let (p, big) = (k..n)
                .map(|i| (i, m.at(i, k).abs()))
                .fold((k, -1.0), |acc, x| if x.1 > acc.1 { x } else { acc });
            if !(big > 0.0 && big.is_finite()) {
                return None;
            }
            if p != k {
                for j in 0..n {
                    m.a.swap(k * n + j, p * n + j);
                }
                piv.swap(k, p);
            }
            let d = m.at(k, k);
            for i in k + 1..n {
                let f = m.at(i, k) / d;
                m.a[i * n + k] = f;
                for j in k + 1..n {
                    m.a[i * n + j] -= f * m.at(k, j);
                }
            }
        }
        Some(Lu { m, piv })
    }

    fn solve(&self, b: &[f64]) -> Vec<f64> {
        let n = self.m.n;
        let mut x: Vec<f64> = self.piv.iter().map(|&p| b[p]).collect();
        for i in 0..n {
            for j in 0..i {
                x[i] -= self.m.at(i, j) * x[j];
            }
        }
        for i in (0..n).rev() {
            for j in i + 1..n {
                x[i] -= self.m.at(i, j) * x[j];
            }
            x[i] /= self.m.at(i, i);
        }
        x
    }
}

/// Smallest ω² of K·φ = ω²·M·φ (M diagonal) by inverse iteration with a Rayleigh
/// quotient (converged to 1e-12 relative).
fn omega1_sq(k: &Mat, m: &[f64]) -> Option<f64> {
    let n = k.n;
    if n == 0 {
        return None;
    }
    let lu = Lu::new(k.clone())?;
    let mut x = vec![1.0; n];
    let mut lam = 0.0;
    for _ in 0..500 {
        let y = lu.solve(&x.iter().zip(m).map(|(a, b)| a * b).collect::<Vec<_>>());
        let ky = k.mul_vec(&y);
        let num: f64 = y.iter().zip(&ky).map(|(a, b)| a * b).sum();
        let den: f64 = y.iter().zip(m).map(|(a, b)| a * a * b).sum();
        let norm = den.sqrt();
        if !(norm > 0.0 && norm.is_finite()) {
            return None;
        }
        let new = num / den;
        x = y.iter().map(|v| v / norm).collect();
        if (new - lam).abs() <= 1e-12 * new.abs() {
            return Some(new);
        }
        lam = new;
    }
    Some(lam)
}

/// Second ω² (deflation of the first mode) for Rayleigh damping; `None` for one DOF.
fn omega2_sq(k: &Mat, m: &[f64], w1: f64) -> Option<f64> {
    let n = k.n;
    if n < 2 {
        return None;
    }
    // Inverse iteration on (K − σM) with σ just above ω1² finds the next mode.
    let mut shifted = k.clone();
    let sigma = 1.5 * w1;
    for (i, mi) in m.iter().enumerate() {
        shifted.add(i, i, -sigma * mi);
    }
    let lu = Lu::new(shifted)?;
    let mut x: Vec<f64> = (0..n)
        .map(|i| if i % 2 == 0 { 1.0 } else { -0.5 })
        .collect();
    let mut lam = 0.0;
    for _ in 0..500 {
        let y = lu.solve(&x.iter().zip(m).map(|(a, b)| a * b).collect::<Vec<_>>());
        let ky = k.mul_vec(&y);
        let num: f64 = y.iter().zip(&ky).map(|(a, b)| a * b).sum();
        let den: f64 = y.iter().zip(m).map(|(a, b)| a * a * b).sum();
        let norm = den.sqrt();
        if !(norm > 0.0 && norm.is_finite()) {
            return None;
        }
        let new = num / den;
        x = y.iter().map(|v| v / norm).collect();
        if (new - lam).abs() <= 1e-12 * new.abs() {
            return Some(new.max(w1));
        }
        lam = new;
    }
    Some(lam.max(w1))
}

// ---- Model assembly ----------------------------------------------------------------------

/// A spring-damper between DOF `a` and DOF `b` (`None` = the ground).
#[derive(Clone, Copy, Debug)]
struct Link {
    a: Option<usize>,
    b: usize,
    k: f64,
    c: f64,
}

/// Linear system in relative coordinates plus the explicit extras.
#[derive(Clone, Debug)]
struct System {
    m: Vec<f64>,
    links: Vec<Link>,
    /// Floor → DOF (None: the floor moves with the ground).
    floor_dof: Vec<Option<usize>>,
    /// Isolation base DOF.
    base: Option<usize>,
    /// Isolator stiffness / damping (reported and used for the stop and skyhook).
    iso_k: f64,
    iso_c: f64,
    /// Stroke beyond which the end stop engages (HAVEN-F).
    stop_m: Option<f64>,
    /// Skyhook on the base slab (HAVEN-F `skyhook`).
    skyhook: bool,
    /// Damper DOF (SENTINEL).
    tmd: Option<(usize, usize)>,
}

impl System {
    fn n(&self) -> usize {
        self.m.len()
    }

    fn matrices(&self) -> (Mat, Mat) {
        let n = self.n();
        let (mut k, mut c) = (Mat::zeros(n), Mat::zeros(n));
        for l in &self.links {
            let b = l.b;
            k.add(b, b, l.k);
            c.add(b, b, l.c);
            if let Some(a) = l.a {
                k.add(a, a, l.k);
                k.add(a, b, -l.k);
                k.add(b, a, -l.k);
                c.add(a, a, l.c);
                c.add(a, b, -l.c);
                c.add(b, a, -l.c);
            }
        }
        (k, c)
    }
}

/// Storeys (z > 0) sorted by height then level: (floor index, mass, given stiffness).
fn storeys(floors: &[FloorSpec]) -> Vec<(usize, f64, Option<f64>)> {
    let mut s: Vec<(usize, &FloorSpec)> = floors
        .iter()
        .enumerate()
        .filter(|(_, f)| f.z_m > 0.0)
        .collect();
    s.sort_by(|a, b| a.1.z_m.total_cmp(&b.1.z_m).then(a.1.level.cmp(&b.1.level)));
    s.into_iter()
        .map(|(i, f)| (i, f.mass_kg.unwrap_or(DEFAULT_FLOOR_MASS_KG), f.k_n_per_m))
        .collect()
}

/// Storey stiffnesses: given values kept; unknown ones share one value scaled so the
/// fixed-base first period is `period_s` (bisection in log space).
fn storey_stiffness(st: &[(usize, f64, Option<f64>)], period_s: f64) -> Vec<f64> {
    let given: Vec<f64> = st.iter().filter_map(|s| s.2).collect();
    if given.len() == st.len() || st.is_empty() {
        return st.iter().map(|s| s.2.unwrap_or(1.0)).collect();
    }
    let m: Vec<f64> = st.iter().map(|s| s.1).collect();
    let w_target = (2.0 * std::f64::consts::PI / period_s).powi(2);
    let ks = |x: f64| -> Vec<f64> { st.iter().map(|s| s.2.unwrap_or(x)).collect() };
    let w_of = |x: f64| -> f64 {
        let k = chain_k(&ks(x));
        omega1_sq(&k, &m).unwrap_or(f64::NAN)
    };
    // All unknown: ω² ∝ k exactly.
    if given.is_empty() {
        let w1 = w_of(1.0);
        let x = if w1 > 0.0 { w_target / w1 } else { 1.0 };
        return ks(x);
    }
    let (mut lo, mut hi) = (-12.0f64, 18.0f64);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if w_of(10f64.powf(mid)) < w_target {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    ks(10f64.powf(0.5 * (lo + hi)))
}

/// Fixed-base shear-chain stiffness matrix (storey j between j−1 and j; storey 0 to the
/// ground).
fn chain_k(k: &[f64]) -> Mat {
    let n = k.len();
    let mut m = Mat::zeros(n);
    for (j, &kj) in k.iter().enumerate() {
        m.add(j, j, kj);
        if j > 0 {
            m.add(j - 1, j - 1, kj);
            m.add(j - 1, j, -kj);
            m.add(j, j - 1, -kj);
        }
    }
    m
}

/// Assembles the system for `b`; `fps_d_m` is the FPS design displacement.
fn assemble(b: &BuildingConfig, fps_d_m: f64) -> (System, BuildingInfo) {
    let floors = b.floors();
    let (period_s, zeta, _, _) = b.common();
    let st = storeys(floors);
    let k_st = storey_stiffness(&st, period_s);
    let m_st: Vec<f64> = st.iter().map(|s| s.1).collect();
    let mut info = BuildingInfo::default();
    let k_fb = chain_k(&k_st);
    let w1 = omega1_sq(&k_fb, &m_st).unwrap_or(0.0);
    info.period_s = if w1 > 0.0 {
        2.0 * std::f64::consts::PI / w1.sqrt()
    } else {
        0.0
    };
    let isolated = matches!(
        b,
        BuildingConfig::FpsIsolated { .. } | BuildingConfig::HavenF { .. }
    );
    // Superstructure damping: Rayleigh (fixed base, tower) or stiffness-proportional at
    // mode 1 (isolated), per storey link: c = a1·k (+ a0·m to the ground, fixed base).
    let (a0, a1) = if w1 > 0.0 {
        match omega2_sq(&k_fb, &m_st, w1) {
            Some(w2) if !isolated => {
                let (o1, o2) = (w1.sqrt(), w2.sqrt());
                (zeta * 2.0 * o1 * o2 / (o1 + o2), zeta * 2.0 / (o1 + o2))
            }
            _ => (0.0, 2.0 * zeta / w1.sqrt()),
        }
    } else {
        (0.0, 0.0)
    };
    let base_mass: f64 = floors
        .iter()
        .filter(|f| f.z_m <= 0.0)
        .map(|f| f.mass_kg.unwrap_or(DEFAULT_FLOOR_MASS_KG))
        .sum::<f64>()
        .max(if isolated { DEFAULT_FLOOR_MASS_KG } else { 0.0 });
    let mut sys = System {
        m: Vec::new(),
        links: Vec::new(),
        floor_dof: vec![None; floors.len()],
        base: None,
        iso_k: 0.0,
        iso_c: 0.0,
        stop_m: None,
        skyhook: false,
        tmd: None,
    };
    if isolated {
        sys.m.push(base_mass);
        sys.base = Some(0);
        for (i, f) in floors.iter().enumerate() {
            if f.z_m <= 0.0 {
                sys.floor_dof[i] = Some(0);
            }
        }
    }
    let off = sys.m.len();
    for (j, s) in st.iter().enumerate() {
        sys.m.push(s.1);
        sys.floor_dof[s.0] = Some(off + j);
        let below = if j == 0 { sys.base } else { Some(off + j - 1) };
        sys.links.push(Link {
            a: below,
            b: off + j,
            k: k_st[j],
            c: a1 * k_st[j],
        });
        if a0 > 0.0 {
            sys.links.push(Link {
                a: None,
                b: off + j,
                k: 0.0,
                c: a0 * s.1,
            });
        }
    }
    let m_tot: f64 = sys.m.iter().sum();
    match *b {
        BuildingConfig::FixedBase { .. } => {}
        BuildingConfig::FpsIsolated {
            iso_period_s, mu, ..
        } => {
            // Zayas et al. 1990: T = 2π√(R/g); Naeim & Kelly 1999 linear equivalent.
            let r = G0_MPS2 * iso_period_s * iso_period_s / (4.0 * std::f64::consts::PI.powi(2));
            let w = m_tot * G0_MPS2;
            let d = fps_d_m.max(FPS_D_MIN_M);
            let k_eff = w / r + mu * w / d;
            let beta = (2.0 / std::f64::consts::PI) * mu / (mu + d / r);
            let c = 2.0 * beta * (k_eff * m_tot).sqrt();
            sys.iso_k = k_eff;
            sys.iso_c = c;
            sys.links.push(Link {
                a: None,
                b: 0,
                k: k_eff,
                c,
            });
            info.iso_t_eff_s = 2.0 * std::f64::consts::PI * (m_tot / k_eff).sqrt();
            info.iso_beta_eff = beta;
        }
        BuildingConfig::HavenF {
            iso_period_s,
            iso_zeta,
            stroke_m,
            control,
            ..
        } => {
            let k = m_tot * (2.0 * std::f64::consts::PI / iso_period_s).powi(2);
            let c = 2.0 * iso_zeta * (k * m_tot).sqrt();
            sys.iso_k = k;
            sys.iso_c = c;
            sys.skyhook = control == IsolationControl::Skyhook;
            sys.stop_m = Some(stroke_m);
            sys.links.push(Link {
                a: None,
                b: 0,
                k,
                c: if sys.skyhook { 0.0 } else { c },
            });
            info.iso_t_eff_s = iso_period_s;
            info.iso_beta_eff = iso_zeta;
        }
        BuildingConfig::TowerSentinel {
            hmd_mass_ratio,
            hmd_mode,
            ..
        } => {
            if let Some(top) = st.len().checked_sub(1).map(|t| off + t) {
                let m_s: f64 = m_st.iter().sum();
                let dd =
                    damper_design_for(m_s, w1.sqrt(), hmd_mass_ratio, TuningForm::WarburtonBase);
                info.hmd_mass_kg = dd.m_d_kg;
                if hmd_mode == HmdMode::LandedFps {
                    sys.m[top] += dd.m_d_kg;
                } else {
                    sys.m.push(dd.m_d_kg);
                    let d = sys.m.len() - 1;
                    sys.links.push(Link {
                        a: Some(top),
                        b: d,
                        k: dd.k_d_n_per_m,
                        c: dd.c_d_n_s_per_m,
                    });
                    sys.tmd = Some((top, d));
                }
            }
        }
    }
    (sys, info)
}

/// Integrates `sys` under `ground` (topo-scaled, building frame, at [`BUILDING_DT_S`]).
fn integrate(sys: &System, ground: &[[f64; 3]], info: &mut BuildingInfo) -> Vec<Vec<[f64; 2]>> {
    let n = sys.n();
    let steps = ground.len();
    let mut out = vec![Vec::with_capacity(steps); n];
    if n == 0 {
        return out;
    }
    let (k, c) = sys.matrices();
    let dt = BUILDING_DT_S;
    let (gamma, beta) = (0.5, 0.25);
    let (a1m, a1c) = (1.0 / (beta * dt * dt), gamma / (beta * dt));
    let mut keff = k.clone();
    for i in 0..n {
        for j in 0..n {
            keff.add(i, j, a1c * c.at(i, j));
        }
        keff.add(i, i, a1m * sys.m[i]);
    }
    let Some(lu) = Lu::new(keff) else {
        return out;
    };
    // Two horizontal directions share the factorisation.
    let mut u = [vec![0.0; n], vec![0.0; n]];
    let mut v = [vec![0.0; n], vec![0.0; n]];
    let mut a = [vec![0.0; n], vec![0.0; n]];
    let mut vg = [0.0f64; 2];
    let w_hp = 2.0 * std::f64::consts::PI * SKYHOOK_HP_HZ;
    let extra = |u: &[f64], v: &[f64], vg: f64| -> Vec<f64> {
        let mut f = vec![0.0; n];
        if let Some(b) = sys.base {
            if let Some(s) = sys.stop_m {
                let over = u[b].abs() - s;
                if over > 0.0 {
                    f[b] -= STOP_STIFFNESS_RATIO * sys.iso_k * over * u[b].signum();
                }
            }
            if sys.skyhook {
                f[b] -= sys.iso_c * (v[b] + vg);
            }
        }
        f
    };
    // Initial accelerations: M ü0 = −M·a_g(0) + f − C u̇ − K u (all zero at rest).
    for d in 0..2 {
        let ag = ground.first().map_or(0.0, |g| g[d]);
        let f = extra(&u[d], &v[d], vg[d]);
        for i in 0..n {
            a[d][i] = -ag + f[i] / sys.m[i];
        }
    }
    let mut d_peak = 0.0f64;
    let mut hmd_peak = 0.0f64;
    let mut sat_steps = 0usize;
    for (step, g) in ground.iter().enumerate() {
        if step > 0 {
            for d in 0..2 {
                // Leaky ground-velocity integral (skyhook reference).
                vg[d] += dt * (g[d] - w_hp * vg[d]);
                let f = extra(&u[d], &v[d], vg[d]);
                let mut p: Vec<f64> = (0..n).map(|i| -sys.m[i] * g[d] + f[i]).collect();
                let mu: Vec<f64> = (0..n)
                    .map(|i| {
                        sys.m[i]
                            * (a1m * u[d][i] + v[d][i] / (beta * dt) + (0.5 / beta - 1.0) * a[d][i])
                    })
                    .collect();
                let cv: Vec<f64> = (0..n)
                    .map(|i| {
                        a1c * u[d][i]
                            + (gamma / beta - 1.0) * v[d][i]
                            + dt * (gamma / (2.0 * beta) - 1.0) * a[d][i]
                    })
                    .collect();
                let cc = c.mul_vec(&cv);
                for i in 0..n {
                    p[i] += mu[i] + cc[i];
                }
                let un = lu.solve(&p);
                for i in 0..n {
                    let du = un[i] - u[d][i];
                    let vn = gamma / (beta * dt) * du
                        + (1.0 - gamma / beta) * v[d][i]
                        + dt * (1.0 - gamma / (2.0 * beta)) * a[d][i];
                    let an = a1m * du - v[d][i] / (beta * dt) - (0.5 / beta - 1.0) * a[d][i];
                    u[d][i] = un[i];
                    v[d][i] = vn;
                    a[d][i] = an;
                }
            }
        }
        for i in 0..n {
            out[i].push([a[0][i] + g[0], a[1][i] + g[1]]);
        }
        if let Some(b) = sys.base {
            let dmag = u[0][b].hypot(u[1][b]);
            d_peak = d_peak.max(dmag);
            if sys
                .stop_m
                .is_some_and(|s| u[0][b].abs() > s || u[1][b].abs() > s)
            {
                sat_steps += 1;
            }
        }
        if let Some((top, dmp)) = sys.tmd {
            hmd_peak = hmd_peak.max((u[0][dmp] - u[0][top]).hypot(u[1][dmp] - u[1][top]));
        }
    }
    info.iso_d_peak_m = d_peak;
    info.iso_saturated_s = sat_steps as f64 * dt;
    info.hmd_stroke_peak_m = hmd_peak;
    out
}

/// Runs the building `b` under the ground acceleration `ground_at(t)` (m/s², building
/// frame, before the topographic factor) for `t_end_s`.
pub(crate) fn respond(
    b: &BuildingConfig,
    ground_at: impl Fn(f64) -> [f64; 3],
    t_end_s: f64,
) -> BuildingResponse {
    let (_, _, _, topo) = b.common();
    let steps = (t_end_s / BUILDING_DT_S).ceil().max(0.0) as usize + 1;
    let ground: Vec<[f64; 3]> = (0..steps)
        .map(|k| {
            let g = ground_at(k as f64 * BUILDING_DT_S);
            [topo * g[0], topo * g[1], topo * g[2]]
        })
        .collect();
    let (mut sys, mut info) = assemble(b, 0.0);
    if let BuildingConfig::FpsIsolated { iso_stroke_m, .. } = *b {
        // Equivalent-linear iteration on the design displacement D.
        let mut d = iso_stroke_m.unwrap_or(FPS_D0_M);
        for _ in 0..FPS_ITERATIONS {
            let (s, i0) = assemble(b, d);
            let mut i1 = i0.clone();
            let _ = integrate(&s, &ground, &mut i1);
            d = i1.iso_d_peak_m.max(FPS_D_MIN_M);
        }
        let (s, i0) = assemble(b, d);
        sys = s;
        info = i0;
    }
    let dofs = integrate(&sys, &ground, &mut info);
    let floors = b.floors();
    let acc = sys
        .floor_dof
        .iter()
        .map(|dof| match dof {
            Some(i) => dofs[*i]
                .iter()
                .zip(&ground)
                .map(|(h, g)| [h[0], h[1], g[2]])
                .collect(),
            None => ground.clone(),
        })
        .collect();
    BuildingResponse {
        levels: floors.iter().map(|f| f.level).collect(),
        acc,
        ground,
        info,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn floors(zs: &[(u32, f64)]) -> Vec<FloorSpec> {
        zs.iter()
            .map(|&(level, z_m)| FloorSpec {
                level,
                name: format!("f{level}"),
                z_m,
                ..FloorSpec::default()
            })
            .collect()
    }

    fn sine(amp: f64, f_hz: f64) -> impl Fn(f64) -> [f64; 3] {
        move |t| {
            [
                amp * (2.0 * std::f64::consts::PI * f_hz * t).sin(),
                0.0,
                0.0,
            ]
        }
    }

    /// Steady-state peak over the last `tail_s` seconds.
    fn tail_peak(v: &[[f64; 3]], axis: usize, tail_s: f64) -> f64 {
        let k = (tail_s / BUILDING_DT_S) as usize;
        v.iter()
            .rev()
            .take(k)
            .map(|a| a[axis].abs())
            .fold(0.0, f64::max)
    }

    #[test]
    fn fixed_base_sdof_matches_closed_form() {
        // SDOF at T = 0.35 s, ζ = 5 %: absolute-acceleration transmissibility
        // |T(r)| = √((1 + (2ζr)²)/((1 − r²)² + (2ζr)²)) (Chopra Eq. 3.6.5); ground
        // floor = ground.
        let b = BuildingConfig::FixedBase {
            floors: floors(&[(0, 0.0), (1, 3.4)]),
            period_s: 0.35,
            zeta: 0.05,
            height_m: None,
            topo_factor: 1.0,
        };
        for f in [1.0f64, 1.0 / 0.35, 6.0] {
            let r = f * 0.35;
            let z = 0.05f64;
            let t = ((1.0 + (2.0 * z * r).powi(2))
                / ((1.0 - r * r).powi(2) + (2.0 * z * r).powi(2)))
            .sqrt();
            let resp = respond(&b, sine(1.0, f), 30.0);
            assert!(
                (resp.info.period_s - 0.35).abs() < 1e-9,
                "period scaled to 0.35 s"
            );
            let peak = tail_peak(&resp.acc[1], 0, 3.0);
            assert!(
                (peak / t - 1.0).abs() < 0.01,
                "f {f} Hz: |T| {peak} vs closed form {t} (±1 %, Newmark at 1 ms)"
            );
            assert!(
                (tail_peak(&resp.acc[0], 0, 3.0) - 1.0).abs() < 1e-3,
                "ground floor"
            );
        }
        // Topographic factor scales the input; vertical follows the ground.
        let b2 = BuildingConfig::FixedBase {
            floors: floors(&[(0, 0.0), (1, 3.4)]),
            period_s: 0.35,
            zeta: 0.05,
            height_m: None,
            topo_factor: 1.2,
        };
        let r = respond(&b2, |_| [0.0, 0.0, 0.5], 1.0);
        assert!((r.acc[1][500][2] - 0.6).abs() < 1e-12);
    }

    #[test]
    fn mdof_period_scaling_and_rayleigh() {
        // Three storeys with unknown stiffness: T1 lands on period_s.
        let b = BuildingConfig::FixedBase {
            floors: floors(&[(0, 0.0), (1, 3.0), (2, 6.0), (3, 9.0)]),
            period_s: 0.5,
            zeta: 0.05,
            height_m: None,
            topo_factor: 1.0,
        };
        let (sys, info) = assemble(&b, 0.0);
        assert!((info.period_s - 0.5).abs() < 1e-9, "{}", info.period_s);
        assert_eq!(sys.n(), 3);
        // Mixed given/unknown stiffness also hits the target period (bisection).
        let mut fl = floors(&[(0, 0.0), (1, 3.0), (2, 6.0)]);
        fl[1].k_n_per_m = Some(2.0e8);
        let b = BuildingConfig::FixedBase {
            floors: fl,
            period_s: 0.4,
            zeta: 0.05,
            height_m: None,
            topo_factor: 1.0,
        };
        let (_, info) = assemble(&b, 0.0);
        assert!((info.period_s - 0.4).abs() < 1e-6, "{}", info.period_s);
    }

    #[test]
    fn isolation_attenuates_and_reports() {
        let fl = floors(&[(0, 0.0), (1, 3.4)]);
        let fixed = BuildingConfig::FixedBase {
            floors: fl.clone(),
            period_s: 0.35,
            zeta: 0.05,
            height_m: None,
            topo_factor: 1.0,
        };
        let fps = BuildingConfig::FpsIsolated {
            floors: fl.clone(),
            period_s: 0.35,
            zeta: 0.05,
            height_m: None,
            topo_factor: 1.0,
            iso_period_s: 3.0,
            mu: 0.05,
            iso_stroke_m: None,
        };
        // A 2.5 Hz, 0.3 g ground motion: the isolated floors see far less.
        let g = sine(0.3 * G0_MPS2, 2.5);
        let rf = respond(&fixed, &g, 20.0);
        let ri = respond(&fps, &g, 20.0);
        let pf = tail_peak(&rf.acc[1], 0, 3.0);
        let pi = tail_peak(&ri.acc[1], 0, 3.0);
        assert!(pi < 0.35 * pf, "isolated {pi} vs fixed {pf}");
        assert!(ri.info.iso_t_eff_s > 1.0 && ri.info.iso_beta_eff > 0.0);
        // R = g·T²/4π² = 2.236 m at T 3 s (Zayas et al. 1990).
        let r = G0_MPS2 * 9.0 / (4.0 * std::f64::consts::PI.powi(2));
        assert!((r - 2.236).abs() < 1e-3);
        // HAVEN-F with a small stroke saturates and reports it; skyhook runs stably.
        let haven = |control| BuildingConfig::HavenF {
            floors: fl.clone(),
            period_s: 0.35,
            zeta: 0.05,
            height_m: None,
            topo_factor: 1.0,
            iso_period_s: 6.0,
            iso_zeta: 0.3,
            stroke_m: 0.02,
            control,
        };
        let slow = sine(0.2 * G0_MPS2, 0.3);
        let rp = respond(&haven(IsolationControl::Passive), &slow, 20.0);
        assert!(rp.info.iso_saturated_s > 0.0 && rp.info.iso_d_peak_m < 0.2);
        let rs = respond(&haven(IsolationControl::Skyhook), &g, 20.0);
        assert!(rs.acc[1].iter().all(|a| a[0].is_finite()));
    }

    #[test]
    fn sentinel_damper_reduces_resonant_response() {
        let tower = |mode| BuildingConfig::TowerSentinel {
            floors: floors(&[(0, 0.0), (80, 380.0)]),
            period_s: 7.0,
            zeta: 0.01,
            height_m: None,
            topo_factor: 1.0,
            zeta_s: 0.01,
            hmd_mass_ratio: 0.02,
            hmd_mode: mode,
        };
        let g = sine(0.05, 1.0 / 7.0);
        let with = respond(&tower(HmdMode::SeismicPassive), &g, 200.0);
        let locked = respond(&tower(HmdMode::LandedFps), &g, 200.0);
        let pw = tail_peak(&with.acc[1], 0, 14.0);
        let pl = tail_peak(&locked.acc[1], 0, 14.0);
        assert!(
            pw < 0.5 * pl,
            "tuned damper {pw} vs locked {pl} at resonance"
        );
        assert!(with.info.hmd_mass_kg > 0.0 && with.info.hmd_stroke_peak_m > 0.0);
    }
}
