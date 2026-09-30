//! Rigid-body kinematics in the rail frame (quaternion attitude, floor-acceleration inertial load) and the Carriage6 tier (PHYSICS §5). Owner: A6a.
//!
//! Carriage6 states (core design §C.1): p, q (unit quaternion), v, ω; per element the
//! coil current i (and Φ_eff when τ_e ≥ 2h); LSM i_d, i_q; one bristle z per friction
//! contact; the DRAWER cavity ΔP; T_mag and T_coil. RK4 at the 50 µs base tick.
//! Sign conventions (PHYSICS §1): x travel (+ opens), y cross-track, z up; the floor
//! acceleration enters as the inertial load −m·a_floor in the rail frame.
//!
//! As implemented (PHYSICS §5; physics perspective §1.1):
//! * State: CoM position r_c and velocity v_c in the rail frame, attitude q (body → rail,
//!   renormalised after every step), body-frame ω; J = diag(j_kgm2) about the CoM scaled
//!   by m/m* (uniform-density payload).
//!   m·v̇ = Σ_k F_k + F_lsm + F_contacts + F_drag + F_ext − m·g₀·ẑ − m·a_floor;
//!   J·ω̇ = Rᵀ·Σ(r × F) − ω × Jω; q̇ = ½·q ⊗ (0, ω).
//! * The reference point (rail centreline, rail underside, mid-carriage) sits at
//!   p_ref = r_c − R·c (c = `carriage.cg_m`); `Truth::pose` reports it, with z = 0 at the
//!   zero-power position (every gap at g*).
//! * Element k at body offset r_k: P_k = r_c + R·(r_k − c); gap
//!   g_k = g* + s_k·(r_kz + z_rail(x_k) − P_kz) (s_k = +1 lift, −1 hold-down under a
//!   lower flange at the same nominal gap), lateral offset y_k = P_ky − r_ky − y_rail(x_k),
//!   engagement e_k(x_k − r_kx) from `rail::engagement` with a central-difference de/dx.
//!   Its force is the co-energy gradient (magnet.rs) mapped through the gap/offset
//!   gradients (rail slopes included).
//! * Loads applied at the CoM: drag, bristle friction, hand, lean, pressure, obstacle,
//!   bumpers, the physical end stops (closed-end jamb / DRAWER carcass, open-end rail
//!   stop; W2 lead decision D3, `contact::end_stops`), lock and latch (their small
//!   moments are neglected). LSM thrust and normal
//!   force act at the reference point; the guide (fin/pads) at the guide point (bottom of
//!   a top-hung leaf, else the reference point).
//! * Penalty contacts are capped at use to k ≤ m_eff·(0.2/h)² and c ≤ 2√(k·m_eff)
//!   (PHYSICS §8, §24; physics perspective §1.6); bristle z_ss and σ1 likewise. The
//!   bristle states are held through each RK4 step and advanced exactly afterwards over
//!   the reference point's slide (`advance_bristles`; W2: an RK4-embedded z diverged
//!   when a landed body slid fast under floor shaking).
//! * The energy ledger's cumulative terms are RK4 quadrature states, so they close to
//!   integration accuracy (PHYSICS §9).

use fabrail_params::LandingKind;
use fabrail_params::derive::G0_MPS2;
use fabrail_types::{LsmMode, MAX_MAGNETS, RailClass, fmath};

use crate::air;
use crate::coil::{self, I2tBudget, LagProfile};
use crate::contact::{self, ContactKind, ContactParams};
use crate::dissipation::{self, DragForces};
use crate::friction::{self, BristleParams};
use crate::guidance;
use crate::integrator::{STAGE_FLOOR_INDEX, rk4_step};
use crate::interaction;
use crate::lsm::{self, LsmForces};
use crate::magnet::{ElementEval, ElementGeom, ElementModel, T_REF_C};
use crate::model::{
    ContactForces, ContactState, EnergyLedger, InitLev, InitialState, ModelConfig, PlantInputs,
    PlantModel, PowerTruth, Tier, Truth,
};
use crate::params::PlantParams;
use crate::power::{self, PowerFlow};
use crate::rail::{self, RailSample};
use crate::thermal::{self, ThermalState};

// ---- Vector and quaternion helpers (PHYSICS §5) ----------------------------------------

/// A 3-vector (x, y, z).
pub type Vec3 = [f64; 3];
/// A row-major 3×3 matrix.
pub type Mat3 = [[f64; 3]; 3];

/// a + b.
#[inline]
pub fn v_add(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// a − b.
#[inline]
pub fn v_sub(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// k·a.
#[inline]
pub fn v_scale(a: &Vec3, k: f64) -> Vec3 {
    [a[0] * k, a[1] * k, a[2] * k]
}

/// a · b.
#[inline]
pub fn v_dot(a: &Vec3, b: &Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// a × b.
#[inline]
pub fn v_cross(a: &Vec3, b: &Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// M·v.
#[inline]
pub fn mat_vec(m: &Mat3, v: &Vec3) -> Vec3 {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Mᵀ·v.
#[inline]
pub fn mat_t_vec(m: &Mat3, v: &Vec3) -> Vec3 {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}

/// Unit quaternion [w, x, y, z] (identity if the norm vanishes).
pub fn quat_normalize(q: &[f64; 4]) -> [f64; 4] {
    let n = fmath::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if n > 0.0 && n.is_finite() {
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    } else {
        [1.0, 0.0, 0.0, 0.0]
    }
}

/// Hamilton product a ⊗ b.
pub fn quat_mul(a: &[f64; 4], b: &[f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Rotation matrix body → rail of the (normalised) quaternion.
pub fn quat_to_mat(q: &[f64; 4]) -> Mat3 {
    let [w, x, y, z] = quat_normalize(q);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

/// Quaternion of roll (x), pitch (y), yaw (z), applied as R = R_z(yaw)·R_y(pitch)·R_x(roll).
pub fn quat_from_rpy(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (fmath::sin(0.5 * roll), fmath::cos(0.5 * roll));
    let (sp, cp) = (fmath::sin(0.5 * pitch), fmath::cos(0.5 * pitch));
    let (sy, cy) = (fmath::sin(0.5 * yaw), fmath::cos(0.5 * yaw));
    let qx = [cr, sr, 0.0, 0.0];
    let qy = [cp, 0.0, sp, 0.0];
    let qz = [cy, 0.0, 0.0, sy];
    quat_mul(&quat_mul(&qz, &qy), &qx)
}

// ---- Shared plant machinery (used by Carriage6, Heave1 and Travel1d) -------------------

/// Relative step of the central difference for de/dx of the engagement, m (PHYSICS §10).
const ENGAGE_FD_M: f64 = 1.0e-6;

/// Index of a contact kind into [`Common::cp`].
pub(crate) const fn kind_index(k: ContactKind) -> usize {
    match k {
        ContactKind::UpperStop => 0,
        ContactKind::LowerRollers => 1,
        ContactKind::Seat => 2,
        ContactKind::GuideFin => 3,
        ContactKind::SidePads => 4,
        ContactKind::Bumper => 5,
        ContactKind::ParkingLock => 6,
        ContactKind::SeismicLatch => 7,
        ContactKind::EdgeSeal => 8,
    }
}

/// One levitation element's constant data.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Station {
    /// Offset from the reference point, body frame, m.
    pub r: Vec3,
    /// +1 lift, −1 hold-down.
    pub sign: f64,
    /// Lift element (sign > 0).
    pub lift: bool,
    /// Magnetic model (truth B_r and leakage).
    pub model: ElementModel,
}

/// Constant data shared by the tiers, resolved once from the plant parameters.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Common {
    /// Active elements.
    pub n: usize,
    /// Lift elements.
    pub n_lift: usize,
    /// Element stations.
    pub st: [Station; MAX_MAGNETS],
    /// Contact laws by [`kind_index`] (raw, uncapped).
    pub cp: [ContactParams; 9],
    /// Σ Φ0² over the lift elements at g*, i = 0, 20 °C (the rail-drag flux reference).
    pub phi0_sq_sum: f64,
    /// Guide (fin/pads) point, body frame, m.
    pub guide_pt: Vec3,
}

impl Common {
    /// Resolves the constant data; the contact laws are taken at the tier step `h_s`
    /// (`contact::contact_params_at`: stiffness cap and presliding length per tier).
    pub(crate) fn new(p: &PlantParams, h_s: f64) -> Common {
        let s = &p.design.suspension;
        let n = usize::from(s.n_magnets).min(MAX_MAGNETS);
        let mut st = [Station::default(); MAX_MAGNETS];
        let mut n_lift = 0;
        let mut phi0_sq_sum = 0.0;
        for (k, slot) in st.iter_mut().enumerate().take(n) {
            let ms = &s.stations[k];
            let sign = if ms.normal_sign < 0.0 { -1.0 } else { 1.0 };
            let model = ElementModel::new(s, p.b_r_t, p.leak, ms.has_pm);
            *slot = Station {
                r: [ms.x_m, ms.y_m, ms.z_m],
                sign,
                lift: sign > 0.0,
                model,
            };
            if sign > 0.0 {
                n_lift += 1;
                let ev = model.eval(
                    &ElementGeom {
                        g_m: s.g_star_m,
                        y_m: 0.0,
                        e: 1.0,
                        de_dx_per_m: 0.0,
                    },
                    0.0,
                    T_REF_C,
                );
                phi0_sq_sum += ev.phi_wb * ev.phi_wb;
            }
        }
        let mut cp = [ContactParams::default(); 9];
        for &k in ContactKind::ALL {
            cp[kind_index(k)] = contact::contact_params_at(p, k, h_s);
        }
        let c = &p.design.carriage;
        let guide_pt = if c.top_hung {
            [c.cg_m[0], 0.0, 2.0 * c.cg_m[2]]
        } else {
            [c.cg_m[0], 0.0, 0.0]
        };
        Common {
            n,
            n_lift,
            st,
            cp,
            phi0_sq_sum,
            guide_pt,
        }
    }

    /// Model of the first lift element (Heave1 lumps the lift elements into it).
    pub(crate) fn lift_model(&self) -> ElementModel {
        self.st
            .iter()
            .take(self.n)
            .find(|s| s.lift)
            .map(|s| s.model)
            .unwrap_or_default()
    }

    /// Engagement e and de/dx of element `k` at its rail coordinate (PHYSICS §10).
    pub(crate) fn engagement(&self, p: &PlantParams, k: usize, x_travel_m: f64) -> (f64, f64) {
        let d = &p.design;
        let rx = self.st[k].r[0];
        let e = rail::engagement(&d.rail, &d.carriage, rx, x_travel_m);
        let ep = rail::engagement(&d.rail, &d.carriage, rx, x_travel_m + ENGAGE_FD_M);
        let em = rail::engagement(&d.rail, &d.carriage, rx, x_travel_m - ENGAGE_FD_M);
        (e, (ep - em) / (2.0 * ENGAGE_FD_M))
    }

    /// Mean engagement of the lift elements at travel `x_m`.
    pub(crate) fn mean_lift_engagement(&self, p: &PlantParams, x_m: f64) -> f64 {
        if self.n_lift == 0 {
            return 1.0;
        }
        let sum: f64 = (0..self.n)
            .filter(|&k| self.st[k].lift)
            .map(|k| self.engagement(p, k, x_m).0)
            .sum();
        sum / self.n_lift as f64
    }
}

/// Rail deviation at `x_m` (flat when the environment has no profile; PHYSICS §10).
pub(crate) fn rail_at(p: &PlantParams, x_m: f64) -> RailSample {
    match &p.env.rail {
        Some(r) => r.sample(x_m),
        None => RailSample::default(),
    }
}

/// Penalty stiffness and damping capped for an explicit step: k ≤ m_eff·(0.2/h)² and
/// c ≤ 2√(k·m_eff) (PHYSICS §8, §24; physics perspective §1.6).
pub(crate) fn capped(cp: &ContactParams, m_eff_kg: f64, h_s: f64) -> (f64, f64) {
    let mut k = fmath::fmax(cp.k_n_per_m, 0.0);
    if m_eff_kg > 0.0 && h_s > 0.0 {
        k = fmath::fmin(k, contact::k_max_n_per_m(m_eff_kg, h_s));
    }
    let c_max = 2.0 * fmath::sqrt(k * fmath::fmax(m_eff_kg, 0.0));
    (k, fmath::fmin(fmath::fmax(cp.c_n_s_per_m, 0.0), c_max))
}

/// Bristle parameters capped for an explicit step: the bristle frequency
/// √(F_n·μ_c/(z_ss·m)) ≤ 0.2/h for F_n ≤ 2·m·g₀, i.e. z_ss ≥ 2g₀·μ_c·(h/0.2)², and the
/// micro-damping rate 2g₀·σ1 ≤ 1/h (PHYSICS §8, §24).
pub(crate) fn capped_bristle(bp: &BristleParams, h_s: f64) -> BristleParams {
    let mut b = *bp;
    let r = h_s / 0.2;
    b.z_ss_m = fmath::fmax(b.z_ss_m, 2.0 * G0_MPS2 * fmath::fmax(b.mu_c, 0.0) * r * r);
    if h_s > 0.0 {
        b.sigma1_s_per_m = fmath::fmin(b.sigma1_s_per_m, 1.0 / (2.0 * G0_MPS2 * h_s));
    }
    b
}

/// Vertical contact loads on one lift element (PHYSICS §8).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Vc {
    /// Net force along the element normal, + closes the gap, N.
    pub f_up: f64,
    /// x force from the DRAWER seat ramp, N.
    pub fx: f64,
    /// Upper stop normal, N.
    pub upper: f64,
    /// Roller (and ramp) normal, N.
    pub rollers: f64,
    /// Seat normal (both faces), N.
    pub seat: f64,
    /// Power absorbed by the penalty contacts Σ F_n·δ̇, W.
    pub absorbed_w: f64,
}

/// Contact loads of a lift element at gap `g_m` (rate `g_dot`), body travel `x_open_m`
/// from the closed stop and x speed `v_x` (PHYSICS §8). Geometry: upper stop at g_up;
/// DRAWER (`LandingKind::Seat`): the captive seat [g_seat − c_z, g_seat] within the slot
/// (x_open ≤ slot_len), then the entry ramp rising from g_land to g_seat over ramp_len,
/// then the travel rollers at g_land; other classes: rollers (or skids) at g_land.
#[allow(clippy::too_many_arguments)]
pub(crate) fn vertical_contacts(
    p: &PlantParams,
    c: &Common,
    g_m: f64,
    g_dot: f64,
    x_open_m: f64,
    v_x: f64,
    m_eff_kg: f64,
    h_s: f64,
) -> Vc {
    let s = &p.design.suspension;
    let ld = &p.design.landing;
    let mut out = Vc::default();
    let (ku, cu) = capped(&c.cp[kind_index(ContactKind::UpperStop)], m_eff_kg, h_s);
    let fu = contact::penalty_normal_n(ku, cu, s.g_up_m - g_m, -g_dot);
    out.f_up -= fu;
    out.upper = fu;
    out.absorbed_w += fu * (-g_dot);
    let (kr, cr) = capped(&c.cp[kind_index(ContactKind::LowerRollers)], m_eff_kg, h_s);
    let seat_kind = matches!(ld.kind, LandingKind::Seat) && s.g_seat_m > 0.0;
    if seat_kind && x_open_m <= ld.slot_len_m {
        let (ks, cs) = capped(&c.cp[kind_index(ContactKind::Seat)], m_eff_kg, h_s);
        let lo = contact::penalty_normal_n(ks, cs, g_m - s.g_seat_m, g_dot);
        // Upper face (captive lip) with a lead-in chamfer at the slot entry (W1.5): over
        // the outer `SEAT_LIP_CHAMFER_M` the lip face rises linearly from the clearance
        // band to the upper stop, so a drawer entering above the band is guided down
        // (normal load plus its x component) instead of meeting a step penetration.
        let g_lip0 = s.g_seat_m - ld.seat_z_clearance_m;
        let ch = SEAT_LIP_CHAMFER_M.min(ld.slot_len_m);
        let (g_lip, slope) = if ch > 0.0 && x_open_m > ld.slot_len_m - ch {
            let sl = (g_lip0 - s.g_up_m).max(0.0) / ch;
            (g_lip0 - sl * (x_open_m - (ld.slot_len_m - ch)), sl)
        } else {
            (g_lip0, 0.0)
        };
        let rate_hi = -g_dot - slope * v_x;
        let hi = contact::penalty_normal_n(ks, cs, g_lip - g_m, rate_hi);
        out.f_up += lo - hi;
        out.fx += hi * slope;
        out.seat = lo + hi;
        out.absorbed_w += lo * g_dot + hi * rate_hi;
    } else if seat_kind && ld.ramp_len_m > 0.0 && x_open_m <= ld.slot_len_m + ld.ramp_len_m {
        let slope = ld.ramp_rise_m / ld.ramp_len_m;
        let g_r = s.g_land_m - ld.ramp_rise_m + slope * (x_open_m - ld.slot_len_m);
        let rate = g_dot - slope * v_x;
        let fr = contact::penalty_normal_n(kr, cr, g_m - g_r, rate);
        out.f_up += fr;
        out.fx += fr * slope;
        out.rollers = fr;
        out.absorbed_w += fr * rate;
    } else {
        let fr = contact::penalty_normal_n(kr, cr, g_m - s.g_land_m, g_dot);
        out.f_up += fr;
        out.rollers = fr;
        out.absorbed_w += fr * g_dot;
    }
    out
}

/// Length of the lead-in chamfer on the DRAWER seat's captive lip at the slot entry, m
/// (design choice, W1.5; PHYSICS §8).
pub const SEAT_LIP_CHAMFER_M: f64 = 5.0e-3;

/// Support gap under a lift element at `x_open_m` for the rest condition `lev`
/// (the face it rests on without penetration; PHYSICS §8).
pub(crate) fn support_gap(p: &PlantParams, lev: InitLev, x_open_m: f64) -> f64 {
    let s = &p.design.suspension;
    let ld = &p.design.landing;
    let seat_kind = matches!(ld.kind, LandingKind::Seat) && s.g_seat_m > 0.0;
    match lev {
        InitLev::Levitated => s.g_star_m,
        _ if seat_kind && x_open_m <= ld.slot_len_m => s.g_seat_m,
        _ if seat_kind && ld.ramp_len_m > 0.0 && x_open_m <= ld.slot_len_m + ld.ramp_len_m => {
            s.g_land_m - ld.ramp_rise_m
                + ld.ramp_rise_m / ld.ramp_len_m * (x_open_m - ld.slot_len_m)
        }
        _ => s.g_land_m,
    }
}

/// Bisection root of a continuous `f` on [a, b] with f(a)·f(b) ≤ 0 (to 1e-16 m).
fn bisect(mut a: f64, mut b: f64, f: impl Fn(f64) -> f64) -> f64 {
    let mut fa = f(a);
    for _ in 0..200 {
        let mid = 0.5 * (a + b);
        if (b - a).abs() <= 1e-16 || mid == a || mid == b {
            return mid;
        }
        let fm = f(mid);
        if fm == 0.0 {
            return mid;
        }
        if (fm < 0.0) == (fa < 0.0) {
            a = mid;
            fa = fm;
        } else {
            b = mid;
        }
    }
    0.5 * (a + b)
}

/// Quasi-static rest gap (all lift elements at one gap, zero current, no tilt) for mass
/// `m_kg`, extra upward force `f_extra_up_n` (iron-core LSM attraction), travel
/// `x_travel_m` and magnet temperature `t_mag_c` (PHYSICS §3, §8):
/// * `Levitated`: the zero-power gap where Σ F_pm(g) + f_extra = m·g₀ (clamped to
///   [g_up, g_land] when none exists there); inside the DRAWER seat slot, where the
///   captive seat keeps the gap within [g_seat − c_z, g_seat], the seat equilibrium;
/// * `Landed` / `Seated`: the stable contact equilibrium on the support face at x (the
///   upper seat face when the magnets out-pull the weight), or the face itself when the
///   contacts are inert.
///
/// Every lift element carries its own contacts (m_eff = m/n for the stiffness cap).
#[allow(clippy::too_many_arguments)]
pub(crate) fn rest_gap(
    p: &PlantParams,
    c: &Common,
    lev: InitLev,
    m_kg: f64,
    f_extra_up_n: f64,
    x_travel_m: f64,
    t_mag_c: f64,
    h_s: f64,
) -> f64 {
    let s = &p.design.suspension;
    let w = m_kg * G0_MPS2 - f_extra_up_n;
    let x_open = x_travel_m - p.design.rail.x_closed_m;
    let lifts: Vec<(ElementModel, f64)> = (0..c.n)
        .filter(|&k| c.st[k].lift)
        .map(|k| (c.st[k].model, c.engagement(p, k, x_travel_m).0))
        .collect();
    let f_mag = |g: f64| -> f64 {
        lifts
            .iter()
            .map(|(m, e)| {
                let ev = m.eval(
                    &ElementGeom {
                        g_m: g,
                        y_m: 0.0,
                        e: *e,
                        de_dx_per_m: 0.0,
                    },
                    0.0,
                    t_mag_c,
                );
                ev.loads(ev.phi_wb).f_attract_n
            })
            .sum::<f64>()
    };
    let n_l = lifts.len().max(1) as f64;
    let m_eff = m_kg / n_l;
    let in_slot = matches!(p.design.landing.kind, LandingKind::Seat)
        && s.g_seat_m > 0.0
        && x_open <= p.design.landing.slot_len_m;
    let lev = if in_slot { InitLev::Seated } else { lev };
    match lev {
        InitLev::Levitated => {
            let lo = s.g_up_m.max(crate::magnet::GAP_MIN_M);
            let hi = s.g_land_m.max(lo);
            let f = |g: f64| f_mag(g) - w;
            if f(lo) <= 0.0 {
                hi
            } else if f(hi) >= 0.0 {
                lo
            } else {
                bisect(lo, hi, f)
            }
        }
        InitLev::Landed | InitLev::Seated => {
            let g_s = support_gap(p, lev, x_open);
            let f = |g: f64| {
                let vc = vertical_contacts(p, c, g, 0.0, x_open, 0.0, m_eff, h_s);
                f_mag(g) + n_l * vc.f_up - w
            };
            let span = 1.0e-3;
            if f(g_s) < 0.0 {
                // Resting on the lower face: penetration below it.
                if f(g_s + span) > 0.0 {
                    bisect(g_s, g_s + span, f)
                } else {
                    g_s
                }
            } else {
                // The magnets out-pull the weight: the upper seat face (captive seat) or,
                // without one, the face itself.
                let seat_kind = matches!(p.design.landing.kind, LandingKind::Seat)
                    && s.g_seat_m > 0.0
                    && x_open <= p.design.landing.slot_len_m;
                if seat_kind {
                    let g_hi = s.g_seat_m - p.design.landing.seat_z_clearance_m;
                    if f(g_hi) > 0.0 && f(g_hi - span) < 0.0 {
                        bisect(g_hi - span, g_hi, f)
                    } else {
                        g_hi
                    }
                } else {
                    g_s
                }
            }
        }
    }
}

/// Number of x-axis bristle states: rollers, upper stop, seat, seismic latch.
pub(crate) const N_BRISTLE: usize = 4;

/// Inputs to the shared x-axis load assembly.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct XIn {
    /// Travel position (reference point), m.
    pub x: f64,
    /// Travel speed, m/s.
    pub v: f64,
    /// Body mass, kg.
    pub m: f64,
    /// Plant step, s.
    pub h: f64,
    /// Bristle states [rollers, upper stop, seat, latch], m.
    pub z: [f64; N_BRISTLE],
    /// Normal loads [rollers, upper stop, seat], N.
    pub normals: [f64; 3],
    /// Resting in the DRAWER seat (the x registration and push-stop act).
    pub seated: bool,
    /// Cavity overpressure, Pa.
    pub dp_pa: f64,
    /// ΣΦ²/ΣΦ0² for the rail drag.
    pub flux_ratio: f64,
    /// Parking-lock anchor when engaged, m.
    pub lock: Option<f64>,
    /// Seismic x-latch engaged.
    pub latched: bool,
}

/// Shared x-axis loads (everything except the hand, LSM, guide and magnets).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct XOut {
    /// Sum of the x forces below, N.
    pub fx: f64,
    /// Bristle forces [rollers, upper, seat, latch], N.
    pub f_t: [f64; N_BRISTLE],
    /// Cavity pressure rate, Pa/s.
    pub ddp: f64,
    /// Drag components (signed along x).
    pub drag: DragForces,
    /// Bumper normal (magnitude), N.
    pub bumper_n: f64,
    /// Physical end-stop normal (magnitude; W2 D3), N.
    pub end_stop_n: f64,
    /// Lock force magnitude, N.
    pub lock_n: f64,
    /// Latch force magnitude, N.
    pub latch_n: f64,
    /// Seat x force (push-stop and registration), N.
    pub seat_x_n: f64,
    /// Obstacle force (≥ 0, along +x), N.
    pub obstacle_n: f64,
    /// Power absorbed by bristle friction (rollers, upper, seat), W.
    pub friction_w: f64,
    /// Power absorbed by penalty contacts (lock, latch, seat x, obstacle), W.
    pub contact_w: f64,
    /// Power absorbed by the bumpers, W.
    pub bumper_w: f64,
    /// Power absorbed by the cavity air (throttling and compression), W.
    pub air_w: f64,
}

/// The shared x-axis loads (PHYSICS §8, §9, §11, §22). Sign conventions: drag components
/// and bristle forces are signed forces on the body along +x (the A6b seams return
/// opposing, signed values); the obstacle pushes along +x (it sits in the closing path);
/// positive pressure pushes +x. Geometry of the bumpers (one end-zone length beyond each
/// travel limit), the physical end stops (`contact::end_stops`, W2 D3), the seat
/// registration and the latch follow `contact`, identically in Carriage6 and Travel1d.
pub(crate) fn x_loads(p: &PlantParams, c: &Common, u: &PlantInputs, xi: &XIn) -> XOut {
    let d = &p.design;
    let mut o = XOut::default();
    let x_open = xi.x - d.rail.x_closed_m;
    // Bristle friction: rollers, upper stop rollers, seat (Dupont 2002; PHYSICS §8).
    let kinds = [
        ContactKind::LowerRollers,
        ContactKind::UpperStop,
        ContactKind::Seat,
    ];
    for (j, kind) in kinds.iter().enumerate() {
        let bp = capped_bristle(&c.cp[kind_index(*kind)].bristle, xi.h);
        let br = friction::bristle(&bp, xi.z[j], xi.v, fmath::fmax(xi.normals[j], 0.0));
        o.f_t[j] = br.f_t_n;
        o.fx += br.f_t_n;
        o.friction_w -= br.f_t_n * xi.v;
    }
    // Distributed damping, end zones, rail eddy/hysteresis, air drag (PHYSICS §9).
    o.drag = dissipation::drag(p, xi.x, xi.v, xi.flux_ratio);
    o.fx += o.drag.total_n();
    // End bumpers, Hunt–Crossley (PHYSICS §8; SAF-014).
    let fb = contact::bumper_force_n(p, xi.x, xi.v);
    o.fx += fb;
    o.bumper_n = fmath::abs(fb);
    o.bumper_w -= fb * xi.v;
    // Physical end stops at the class geometry (W2 lead decision D3; PHYSICS §8): the
    // open-end rail stop behind the bumper, and the closed-end jamb (DRAWER: the carcass
    // push-stop, which the seat law below applies while seated).
    let mut fs = contact::open_stop_force_n(p, xi.x, xi.v, xi.m, xi.h);
    if !xi.seated {
        fs += contact::closed_stop_force_n(p, xi.x, xi.v, xi.m, xi.h);
    }
    o.fx += fs;
    o.end_stop_n = fmath::abs(fs);
    o.contact_w -= fs * xi.v;
    // Parking lock (bilateral, PHYSICS §8).
    if let Some(a) = xi.lock {
        let f = contact::lock_force_n(p, xi.x, a, xi.v);
        o.fx += f;
        o.lock_n = fmath::abs(f);
        o.contact_w -= f * xi.v;
    }
    // Seismic x-latch: elasto-plastic up to x_retention (plan §4b; PHYSICS §8).
    if xi.latched
        && let Some((bp, f_ret)) = contact::latch_bristle(p)
    {
        let br = friction::bristle(&bp, xi.z[3], xi.v, f_ret);
        o.f_t[3] = br.f_t_n;
        o.fx += br.f_t_n;
        o.latch_n = fmath::abs(br.f_t_n);
        o.contact_w -= br.f_t_n * xi.v;
    }
    // DRAWER seat x registration and push-stop while seated (plan §2b item 4).
    if xi.seated {
        let f = contact::seat_x_force_n(p, x_open, xi.v, xi.h);
        o.fx += f;
        o.seat_x_n = f;
        o.contact_w -= f * xi.v;
    }
    // Obstacle through the closing-edge seal (PHYSICS §22).
    if let Some(ob) = u.f_ext.obstacle {
        let f = fmath::fmax(interaction::obstacle_force_n(&ob, xi.x, xi.v), 0.0);
        o.fx += f;
        o.obstacle_n = f;
        o.contact_w -= f * xi.v;
    }
    // DRAWER cavity (PHYSICS §11).
    if d.air.cavity_area_m2 > 0.0 && d.air.cavity_volume_m3 > 0.0 {
        let leak = u.air_leak_area_m2.unwrap_or(p.leak_area_m2);
        let f = air::cavity_force_n(&d.air, xi.dp_pa);
        o.fx += f;
        o.air_w -= f * xi.v;
        o.ddp = air::cavity_dp_rate(&d.air, leak, xi.dp_pa, x_open, xi.v);
    }
    // Lean and pressure disturbances (PHYSICS §11, §22; plan §2b item 11).
    o.fx += u.f_ext.lean_n;
    o.fx += if matches!(d.class, RailClass::Drawer) {
        air::cavity_force_n(&d.air, u.f_ext.smoke_dp_pa)
    } else {
        air::smoke_force_n(&d.air, u.f_ext.smoke_dp_pa)
    };
    o
}

/// Exact split update of the x-axis bristle states after an RK4 body step (PHYSICS §8,
/// §24; `friction` module docs): the tiers hold z fixed through the step (the friction
/// force uses the frozen z and the stage velocity) and then advance each state with
/// `friction::bristle_advance` over the step's slide distance `dx_m`. Unconditionally
/// stable at any sliding speed (an RK4-embedded z has λh = |v|·h/(z_ss·(1 − r_ba)), which
/// exceeds RK4's limit 2.78 above ≈ 0.04 m/s on the 1 ms tier and diverged in W2 floor-
/// shaking runs), and stick stays exact (no ratchet). The latch state advances only while
/// the latch is engaged.
pub(crate) fn advance_bristles(
    p: &PlantParams,
    c: &Common,
    z: &mut [f64],
    dx_m: f64,
    h_s: f64,
    latched: bool,
) {
    let v = if h_s > 0.0 { dx_m / h_s } else { 0.0 };
    let kinds = [
        ContactKind::LowerRollers,
        ContactKind::UpperStop,
        ContactKind::Seat,
    ];
    for (j, kind) in kinds.iter().enumerate() {
        if let Some(zj) = z.get_mut(j) {
            let bp = capped_bristle(&c.cp[kind_index(*kind)].bristle, h_s);
            *zj = friction::bristle_advance(&bp, *zj, dx_m, v);
        }
    }
    if latched
        && let Some((bp, _)) = contact::latch_bristle(p)
        && let Some(zl) = z.get_mut(3)
    {
        *zl = friction::bristle_advance(&bp, *zl, dx_m, v);
    }
}

/// Hand force with the algebraic loop through the hand's mass resolved (PHYSICS §22):
/// with F_h(a) affine in the body acceleration a, m·a = F_other + F_h(a) gives
/// a* = (F_other + F_h(0))/(m + F_h(0) − F_h(1)); the returned force is F_h(a*).
pub(crate) fn hand_force(
    h: &interaction::HandInput,
    x_rel_m: f64,
    v_mps: f64,
    m_kg: f64,
    f_other_n: f64,
) -> f64 {
    let f0 = interaction::hand_force_n(h, x_rel_m, v_mps, 0.0);
    let f1 = interaction::hand_force_n(h, x_rel_m, v_mps, 1.0);
    let m_h = f0 - f1;
    let den = m_kg + m_h;
    let a = if den > 0.0 {
        (f_other_n + f0) / den
    } else {
        f_other_n / m_kg.max(f64::MIN_POSITIVE)
    };
    interaction::hand_force_n(h, x_rel_m, v_mps, a)
}

/// LSM current profiles for one step (PHYSICS §7): Run follows the (magnitude-clamped)
/// references, STO decays to zero, both through the τ_q lag.
pub(crate) fn lsm_profiles(
    p: &PlantParams,
    u: &PlantInputs,
    iq0: f64,
    id0: f64,
) -> (LagProfile, LagProfile) {
    let l = &p.design.lsm;
    let (mut q, mut d) = match u.lsm_mode {
        LsmMode::Run => (u.iq_ref_a, u.id_ref_a),
        _ => (0.0, 0.0),
    };
    let mag = fmath::sqrt(q * q + d * d);
    if l.i_max_a > 0.0 && mag > l.i_max_a {
        q *= l.i_max_a / mag;
        d *= l.i_max_a / mag;
    }
    let mk = |i0: f64, r: f64| LagProfile {
        i0_a: i0,
        i_ref_a: r,
        tau_s: lsm::TAU_CURRENT_S,
        slew_a_per_s: f64::INFINITY,
        limit_a: f64::INFINITY,
    };
    (mk(iq0, q), mk(id0, d))
}

/// Power truth from a power flow and the hold-up voltage (PHYSICS §14).
pub(crate) fn power_truth(
    p: &PlantParams,
    pf: &PowerFlow,
    v_hold: f64,
    mains_ok: bool,
) -> PowerTruth {
    let pw = &p.design.power;
    let e_hold = power::supercap_energy_j(pw.supercap_c_f, v_hold)
        - power::supercap_energy_j(pw.supercap_c_f, pw.supercap_v_min_v);
    PowerTruth {
        v_bus_v: pf.v_bus_v,
        v_hold_v: v_hold,
        i_bus_a: pf.i_bus_a,
        p_bus_w: pf.p_bus_w,
        e_hold_j: fmath::fmax(e_hold, 0.0),
        mains_ok,
        chopper_on: pf.p_chopper_w > 0.0,
    }
}

// ---- Carriage6 state layout ---------------------------------------------------------------

const S_POS: usize = 0;
const S_VEL: usize = 3;
const S_QUAT: usize = 6;
const S_OMEGA: usize = 10;
const S_COIL: usize = 13;
const S_FLUX: usize = S_COIL + MAX_MAGNETS;
const S_BRISTLE: usize = S_FLUX + MAX_MAGNETS;
const S_CAVITY: usize = S_BRISTLE + N_BRISTLE;
const S_TCOIL: usize = S_CAVITY + 1;
const S_TMAG: usize = S_TCOIL + MAX_MAGNETS;
const S_VHOLD: usize = S_TMAG + 1;
const S_LEDGER: usize = S_VHOLD + 1;

/// Ledger quadrature slots (offsets from the ledger base) shared by the tiers.
pub(crate) const L_DIST: usize = 0;
pub(crate) const L_END_ZONE: usize = 1;
pub(crate) const L_RAIL_EDDY: usize = 2;
pub(crate) const L_RAIL_HYST: usize = 3;
pub(crate) const L_AIR: usize = 4;
pub(crate) const L_FRICTION: usize = 5;
pub(crate) const L_CONTACT: usize = 6;
pub(crate) const L_BUMPER: usize = 7;
pub(crate) const L_COIL_CU: usize = 8;
pub(crate) const L_LSM_CU: usize = 9;
pub(crate) const L_ELEC_IN: usize = 10;
pub(crate) const L_HAND: usize = 11;
pub(crate) const L_FLOOR: usize = 12;
/// Number of ledger quadratures.
pub(crate) const N_LEDGER: usize = 13;

const N_STATE: usize = S_LEDGER + N_LEDGER;

type State = [f64; N_STATE];

/// Copies ledger quadratures (base `at`) and the impact counters into an [`EnergyLedger`].
pub(crate) fn ledger_from(
    y: &[f64],
    at: usize,
    kinetic_j: f64,
    impacts: u32,
    e_max: f64,
) -> EnergyLedger {
    EnergyLedger {
        kinetic_j,
        dist_j: y[at + L_DIST],
        end_zone_j: y[at + L_END_ZONE],
        rail_eddy_j: y[at + L_RAIL_EDDY],
        rail_hyst_j: y[at + L_RAIL_HYST],
        air_j: y[at + L_AIR],
        friction_j: y[at + L_FRICTION],
        contact_j: y[at + L_CONTACT],
        bumper_j: y[at + L_BUMPER],
        coil_copper_j: y[at + L_COIL_CU],
        lsm_copper_j: y[at + L_LSM_CU],
        elec_in_j: y[at + L_ELEC_IN],
        hand_work_j: y[at + L_HAND],
        floor_work_j: y[at + L_FLOOR],
        bumper_impacts: impacts,
        bumper_e_max_j: e_max,
    }
}

/// Step-constant context of one Carriage6 step.
#[derive(Clone, Copy, Debug)]
struct Ctx {
    u: PlantInputs,
    m: f64,
    j: Vec3,
    h: f64,
    voltage: bool,
    lag_state: bool,
    coil: [LagProfile; MAX_MAGNETS],
    v_coil: [f64; MAX_MAGNETS],
    limit: [f64; MAX_MAGNETS],
    lsm_q: LagProfile,
    lsm_d: LagProfile,
    hand_anchor: f64,
    lock: Option<f64>,
    latched: bool,
}

/// Per-evaluation outputs used for the truth and the next step's set-up.
#[derive(Clone, Copy, Debug, Default)]
struct Aux {
    gaps: [f64; MAX_MAGNETS],
    y_off: [f64; MAX_MAGNETS],
    coil_i: [f64; MAX_MAGNETS],
    phi: [f64; MAX_MAGNETS],
    l_inc: [f64; MAX_MAGNETS],
    e_mot: [f64; MAX_MAGNETS],
    r_coil: [f64; MAX_MAGNETS],
    contact: ContactForces,
    lsm: LsmForces,
    f_hand: f64,
    a_ref: Vec3,
    p_ref: Vec3,
    v_ref: Vec3,
    pf: PowerFlow,
}

/// Kinematics and magnetics of one element at one evaluation.
#[derive(Clone, Copy, Debug, Default)]
struct ElemKin {
    ev: ElementEval,
    g: f64,
    g_dot: f64,
    y_off: f64,
    y_dot: f64,
    vx: f64,
    rel: Vec3,
    rail: RailSample,
}

/// Mechanical and magnetic energy terms of the Carriage6 state (PHYSICS §5, §9; the
/// conservation and power-balance anchors).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EnergyAudit {
    /// Translational kinetic energy ½·m·|v_c|², J.
    pub kinetic_trans_j: f64,
    /// Rotational kinetic energy ½·ωᵀJω, J.
    pub kinetic_rot_j: f64,
    /// Gravitational potential m·g₀·z_c (CoM height, rail frame), J.
    pub gravity_j: f64,
    /// Σ linear co-energy W′ = ½·MMF²·P, J.
    pub coenergy_j: f64,
    /// Σ stored magnetic energy ½·Φ²/P, J.
    pub stored_j: f64,
    /// Σ M_eff·Φ over the elements, J.
    pub m_phi_j: f64,
    /// Σ ½·L_σ·i², J.
    pub leakage_j: f64,
    /// LSM cogging potential, J.
    pub cogging_j: f64,
    /// Potential of the (constant) iron-core LSM normal attraction −F_n·z_ref, J.
    pub lsm_normal_j: f64,
}

/// The full rigid-body tier.
#[derive(Clone, Debug)]
pub struct Carriage6 {
    p: PlantParams,
    cfg: ModelConfig,
    com: Common,
    y: State,
    truth: Truth,
    aux: Aux,
    i2t: [I2tBudget; MAX_MAGNETS],
    lsm_iq: f64,
    lsm_id: f64,
    lock_anchor: Option<f64>,
    latched: bool,
    hand_anchor: Option<f64>,
    bumper_log: contact::BumperLog,
    bumper_counts: EnergyLedger,
    payload_kg: f64,
    h_last: f64,
}

impl Carriage6 {
    /// A Carriage6 model over `p`, reset to the default initial state.
    pub fn new(p: &PlantParams, cfg: &ModelConfig) -> Carriage6 {
        let mut c = Carriage6 {
            p: p.clone(),
            cfg: *cfg,
            com: Common::new(p, Tier::Carriage6.nominal_h_s()),
            y: [0.0; N_STATE],
            truth: Truth::initial(p, &InitialState::default()),
            aux: Aux::default(),
            i2t: [I2tBudget::default(); MAX_MAGNETS],
            lsm_iq: 0.0,
            lsm_id: 0.0,
            lock_anchor: None,
            latched: false,
            hand_anchor: None,
            bumper_log: contact::BumperLog::default(),
            bumper_counts: EnergyLedger::default(),
            payload_kg: 0.0,
            h_last: Tier::Carriage6.nominal_h_s(),
        };
        c.reset(&InitialState::default());
        c
    }

    /// The construction-time options.
    pub fn config(&self) -> &ModelConfig {
        &self.cfg
    }

    /// The plant parameters.
    pub fn params(&self) -> &PlantParams {
        &self.p
    }

    fn mass(&self) -> f64 {
        self.p.m_body_kg + self.payload_kg
    }

    fn inertia(&self, m: f64) -> Vec3 {
        let ms = self.p.design.carriage.m_star_kg;
        let k = if ms > 0.0 { m / ms } else { 1.0 };
        v_scale(&self.p.design.carriage.j_kgm2, k)
    }

    /// Eddy lag integrated as a state: enabled and τ_e ≥ 2h (core design §C.1).
    fn lag_state(&self, h: f64) -> bool {
        let tau = self.p.design.suspension.tau_eddy_s;
        self.cfg.eddy_lag && tau > 0.0 && tau >= 2.0 * h
    }

    /// Places the body at reference-point position `p_ref` (rail frame), attitude
    /// (roll, pitch, yaw), reference-point velocity `v_ref` and body rate `omega` (body
    /// frame), keeping currents, temperatures and the ledger; the eddy flux is reset
    /// to its quasi-static value. For tests and studies (initial perturbations).
    pub fn set_body_state(&mut self, p_ref: Vec3, rpy: Vec3, v_ref: Vec3, omega: Vec3) {
        let q = quat_from_rpy(rpy[0], rpy[1], rpy[2]);
        let r = quat_to_mat(&q);
        let c = self.p.design.carriage.cg_m;
        let rc = mat_vec(&r, &c);
        let pos = v_add(&p_ref, &rc);
        let w_r = mat_vec(&r, &omega);
        let vel = v_add(&v_ref, &v_cross(&w_r, &rc));
        self.y[S_POS..S_POS + 3].copy_from_slice(&pos);
        self.y[S_VEL..S_VEL + 3].copy_from_slice(&vel);
        self.y[S_QUAT..S_QUAT + 4].copy_from_slice(&q);
        self.y[S_OMEGA..S_OMEGA + 3].copy_from_slice(&omega);
        let u = PlantInputs {
            payload_kg: self.payload_kg,
            coil_enabled: true,
            coil_v_or_iref: self.truth.coil_i_a,
            ..PlantInputs::default()
        };
        self.refresh(&u);
    }

    /// Recomputes Φ_eff = Φ (quasi-static flux), the auxiliary outputs and the truth at
    /// the present state, holding the present currents.
    fn refresh(&mut self, u: &PlantInputs) {
        let ctx = self.ctx(u, self.h_last);
        let floor = self.truth.floor_acc_mps2;
        let t_s = self.truth.t_s;
        let mut aux = self.aux;
        let _ = self.eval(&ctx, 0.0, &floor, &self.y, &mut aux);
        for k in 0..self.com.n {
            self.y[S_FLUX + k] = aux.phi[k];
        }
        let _ = self.eval(&ctx, 0.0, &floor, &self.y, &mut aux);
        self.aux = aux;
        self.fill_truth(&aux, t_s, &floor);
    }

    /// Energy terms of the present state (PHYSICS §5, §9). Uses the coil currents in
    /// the state and the magnet temperature state.
    pub fn energy_audit(&self) -> EnergyAudit {
        let m = self.mass();
        let j = self.inertia(m);
        let y = &self.y;
        let vel = [y[S_VEL], y[S_VEL + 1], y[S_VEL + 2]];
        let w = [y[S_OMEGA], y[S_OMEGA + 1], y[S_OMEGA + 2]];
        let q = [y[S_QUAT], y[S_QUAT + 1], y[S_QUAT + 2], y[S_QUAT + 3]];
        let r = quat_to_mat(&q);
        let pos = [y[S_POS], y[S_POS + 1], y[S_POS + 2]];
        let c = self.p.design.carriage.cg_m;
        let t_mag = y[S_TMAG];
        let mut a = EnergyAudit {
            kinetic_trans_j: 0.5 * m * v_dot(&vel, &vel),
            kinetic_rot_j: 0.5 * (j[0] * w[0] * w[0] + j[1] * w[1] * w[1] + j[2] * w[2] * w[2]),
            gravity_j: m * G0_MPS2 * pos[2],
            ..EnergyAudit::default()
        };
        let p_ref = v_sub(&pos, &mat_vec(&r, &c));
        a.cogging_j = lsm::cogging_energy_j(&self.p.design.lsm, p_ref[0]);
        let f_n = lsm::thrust(
            &self.p.design.lsm,
            p_ref[0],
            0.0,
            0.0,
            self.lsm_id,
            LsmMode::Sto,
        )
        .fn_n;
        a.lsm_normal_j = -f_n * p_ref[2];
        for k in 0..self.com.n {
            let ev = self
                .element_kinematics(k, &r, &pos, &vel, &mat_vec(&r, &w), y[S_COIL + k], t_mag)
                .ev;
            let st = &self.com.st[k];
            a.coenergy_j += ev.coenergy_j();
            a.stored_j += ev.stored_j();
            a.m_phi_j += st.model.m_pm_at_temp(t_mag) * ev.phi_wb;
            a.leakage_j += 0.5 * st.model.l_sigma_h * y[S_COIL + k] * y[S_COIL + k];
        }
        a
    }

    /// Element k's kinematics and magnetic evaluation (PHYSICS §5).
    #[allow(clippy::too_many_arguments)]
    fn element_kinematics(
        &self,
        k: usize,
        r: &Mat3,
        pos: &Vec3,
        vel: &Vec3,
        w_r: &Vec3,
        i_a: f64,
        t_mag: f64,
    ) -> ElemKin {
        let st = &self.com.st[k];
        let c = self.p.design.carriage.cg_m;
        let rel = mat_vec(r, &v_sub(&st.r, &c));
        let pk = v_add(pos, &rel);
        let vk = v_add(vel, &v_cross(w_r, &rel));
        let rs = rail_at(&self.p, pk[0]);
        let g = self.p.design.suspension.g_star_m + st.sign * (st.r[2] + rs.z_m - pk[2]);
        let g_dot = st.sign * (rs.dz_dx * vk[0] - vk[2]);
        let yo = pk[1] - st.r[1] - rs.y_m;
        let y_dot = vk[1] - rs.dy_dx * vk[0];
        let (e, de) = self.com.engagement(&self.p, k, pk[0] - st.r[0]);
        let ev = st.model.eval(
            &ElementGeom {
                g_m: g,
                y_m: yo,
                e,
                de_dx_per_m: de,
            },
            i_a,
            t_mag,
        );
        ElemKin {
            ev,
            g,
            g_dot,
            y_off: yo,
            y_dot,
            vx: vk[0],
            rel,
            rail: rs,
        }
    }

    /// Builds the step context for inputs `u` and step `h`.
    fn ctx(&self, u: &PlantInputs, h: f64) -> Ctx {
        let s = &self.p.design.suspension;
        let voltage = matches!(self.cfg.coil, crate::model::CoilMode::Voltage);
        let v_bus = self.truth.power.v_bus_v;
        let mut coil = [LagProfile::hold(0.0); MAX_MAGNETS];
        let mut v_coil = [0.0; MAX_MAGNETS];
        let mut limit = [f64::INFINITY; MAX_MAGNETS];
        for k in 0..self.com.n {
            let i0 = self.y[S_COIL + k];
            limit[k] = self.i2t[k].limit_a(s.i_cont_a, s.i_pulse_a);
            let r = self.aux.r_coil[k];
            let l_inc = self.aux.l_inc[k];
            if voltage {
                v_coil[k] = if u.coil_enabled {
                    fmath::clamp(u.coil_v_or_iref[k], -v_bus, v_bus)
                } else {
                    0.0
                };
                coil[k] = LagProfile::hold(i0);
            } else if u.coil_enabled {
                coil[k] = LagProfile {
                    i0_a: i0,
                    i_ref_a: u.coil_v_or_iref[k],
                    tau_s: self.p.tau_cl_s,
                    slew_a_per_s: coil::slew_limit_a_per_s(v_bus, r, i0, self.aux.e_mot[k], l_inc),
                    limit_a: limit[k],
                };
            } else {
                // Freewheel decay at L/R (physics perspective §1.13).
                coil[k] = LagProfile {
                    i0_a: i0,
                    i_ref_a: 0.0,
                    tau_s: if r > 0.0 { l_inc / r } else { 0.0 },
                    slew_a_per_s: f64::INFINITY,
                    limit_a: limit[k],
                };
            }
        }
        let (lsm_q, lsm_d) = lsm_profiles(&self.p, u, self.lsm_iq, self.lsm_id);
        let m = self.p.m_body_kg + u.payload_kg;
        Ctx {
            u: *u,
            m,
            j: self.inertia(m),
            h,
            voltage,
            lag_state: self.lag_state(h),
            coil,
            v_coil,
            limit,
            lsm_q,
            lsm_d,
            hand_anchor: self.hand_anchor.unwrap_or(self.truth.x_m),
            lock: self.lock_anchor,
            latched: self.latched,
        }
    }

    /// State derivative at `s` into the step with floor acceleration `a_floor`; fills
    /// `aux` (PHYSICS §5).
    fn eval(&self, k: &Ctx, s: f64, a_floor: &Vec3, y: &State, aux: &mut Aux) -> State {
        let p = &self.p;
        let d = &p.design;
        let sus = &d.suspension;
        let mut dy = [0.0; N_STATE];
        let m = k.m;
        let pos = [y[S_POS], y[S_POS + 1], y[S_POS + 2]];
        let vel = [y[S_VEL], y[S_VEL + 1], y[S_VEL + 2]];
        let q = [y[S_QUAT], y[S_QUAT + 1], y[S_QUAT + 2], y[S_QUAT + 3]];
        let w_b = [y[S_OMEGA], y[S_OMEGA + 1], y[S_OMEGA + 2]];
        let r = quat_to_mat(&q);
        let w_r = mat_vec(&r, &w_b);
        let c = d.carriage.cg_m;
        let rc = mat_vec(&r, &c);
        let p_ref = v_sub(&pos, &rc);
        let v_ref = v_sub(&vel, &v_cross(&w_r, &rc));
        let x = p_ref[0];
        let vx = v_ref[0];
        let x_open = x - d.rail.x_closed_m;
        let t_mag = y[S_TMAG];
        let n = self.com.n;

        // Currents, copper loss and the thermal network first (Ṫ_mag feeds the EMF).
        let mut i = [0.0; MAX_MAGNETS];
        let mut r_k = [0.0; MAX_MAGNETS];
        let mut p_cu = [0.0; MAX_MAGNETS];
        let mut ts = ThermalState::uniform(t_mag);
        for kk in 0..n {
            i[kk] = if k.voltage {
                y[S_COIL + kk]
            } else {
                k.coil[kk].at(s)
            };
            ts.t_coil_c[kk] = y[S_TCOIL + kk];
            r_k[kk] = coil::r_at(sus.r_coil_ohm, sus.tempco_r_per_k, ts.t_coil_c[kk]);
            p_cu[kk] = r_k[kk] * i[kk] * i[kk];
        }
        ts.t_mag_c = t_mag;
        let tr = thermal::thermal_rates(&d.thermal, &ts, &p_cu, k.u.t_ambient_c, n);
        dy[S_TCOIL..S_TCOIL + n].copy_from_slice(&tr.t_coil_c[..n]);
        dy[S_TMAG] = tr.t_mag_c;

        let mut f = [0.0; 3];
        let mut tau = [0.0; 3];
        let mut n_rollers = 0.0;
        let mut n_upper = 0.0;
        let mut n_seat = 0.0;
        let mut absorbed = 0.0;
        let mut p_coil_in = 0.0;
        let mut phi_sq = 0.0;
        let n_l = self.com.n_lift.max(1) as f64;
        for kk in 0..n {
            let st = &self.com.st[kk];
            let ek = self.element_kinematics(kk, &r, &pos, &vel, &w_r, i[kk], t_mag);
            let (ev, g, g_dot, yo, y_dot, vkx, rel, rs) = (
                ek.ev, ek.g, ek.g_dot, ek.y_off, ek.y_dot, ek.vx, ek.rel, ek.rail,
            );
            let phi_f = if k.lag_state {
                y[S_FLUX + kk]
            } else {
                ev.phi_wb
            };
            let l = ev.loads(phi_f);
            let mut fk = [
                l.fx_n - st.sign * l.f_attract_n * rs.dz_dx - l.fy_n * rs.dy_dx,
                l.fy_n,
                st.sign * l.f_attract_n,
            ];
            if st.lift {
                let vc = vertical_contacts(p, &self.com, g, g_dot, x_open, vx, m / n_l, k.h);
                fk[2] += vc.f_up;
                fk[0] += vc.fx;
                n_rollers += vc.rollers;
                n_upper += vc.upper;
                n_seat += vc.seat;
                absorbed += vc.absorbed_w;
            } else {
                // Hold-down elements only meet the upper stop (closest approach).
                let (ku, cu) = capped(
                    &self.com.cp[kind_index(ContactKind::UpperStop)],
                    m / n_l,
                    k.h,
                );
                let fu = contact::penalty_normal_n(ku, cu, sus.g_up_m - g, -g_dot);
                fk[2] -= st.sign * fu;
                n_upper += fu;
                absorbed += fu * (-g_dot);
            }
            f = v_add(&f, &fk);
            tau = v_add(&tau, &v_cross(&rel, &fk));
            // Coil electrics (PHYSICS §4).
            let l_inc = st.model.dpsi_di_h(&ev);
            let dphi_mot = ev.dphi_dg * g_dot
                + ev.dphi_dy * y_dot
                + ev.dphi_dx * vkx
                + ev.dphi_dt * dy[S_TMAG];
            let e_mot = st.model.turns * dphi_mot;
            let di = if k.voltage {
                coil::voltage_di_dt(k.v_coil[kk], r_k[kk], i[kk], e_mot, l_inc, k.limit[kk])
            } else {
                k.coil[kk].rate(s)
            };
            if k.voltage {
                dy[S_COIL + kk] = di;
                p_coil_in += k.v_coil[kk] * i[kk];
            } else {
                p_coil_in += p_cu[kk] + i[kk] * (l_inc * di + e_mot);
            }
            dy[S_FLUX + kk] = if k.lag_state {
                (ev.phi_wb - y[S_FLUX + kk]) / sus.tau_eddy_s
            } else {
                0.0
            };
            if st.lift {
                phi_sq += phi_f * phi_f;
            }
            aux.gaps[kk] = g;
            aux.y_off[kk] = yo;
            aux.coil_i[kk] = i[kk];
            aux.phi[kk] = ev.phi_wb;
            aux.l_inc[kk] = l_inc;
            aux.e_mot[kk] = e_mot;
            aux.r_coil[kk] = r_k[kk];
        }

        // LSM at the reference point (PHYSICS §7).
        let (iq, id) = (k.lsm_q.at(s), k.lsm_d.at(s));
        let lf = lsm::thrust(&d.lsm, x, vx, iq, id, k.u.lsm_mode);
        let f_lsm = [lf.fx_n, 0.0, lf.fn_n];
        f = v_add(&f, &f_lsm);
        tau = v_add(&tau, &v_cross(&v_scale(&rc, -1.0), &f_lsm));

        // Guide fin / pads / sway damping (PHYSICS §6).
        let rel_g = mat_vec(&r, &v_sub(&self.com.guide_pt, &c));
        let pg = v_add(&pos, &rel_g);
        let vg = v_add(&vel, &v_cross(&w_r, &rel_g));
        let rsg = rail_at(p, pg[0]);
        let y_g = pg[1] - rsg.y_m;
        let yd_g = vg[1] - rsg.dy_dx * vg[0];
        let gf = guidance::guide(&d.guidance, y_g, yd_g, vx);
        let f_g = [gf.fx_n, gf.fy_n, 0.0];
        f = v_add(&f, &f_g);
        tau = v_add(&tau, &v_cross(&rel_g, &f_g));
        tau[2] += gf.mz_n_m;
        let guide_w = -(gf.fx_n * vx + gf.fy_n * yd_g + gf.mz_n_m * w_r[2]);

        // Shared x loads (friction, drag, bumpers, lock, latch, obstacle, air, lean).
        let flux_ratio = if self.com.phi0_sq_sum > 0.0 {
            phi_sq / self.com.phi0_sq_sum
        } else {
            1.0
        };
        let xo = x_loads(
            p,
            &self.com,
            &k.u,
            &XIn {
                x,
                v: vx,
                m,
                h: k.h,
                z: [
                    y[S_BRISTLE],
                    y[S_BRISTLE + 1],
                    y[S_BRISTLE + 2],
                    y[S_BRISTLE + 3],
                ],
                normals: [n_rollers, n_upper, n_seat],
                seated: n_seat > 0.0,
                dp_pa: y[S_CAVITY],
                flux_ratio,
                lock: k.lock,
                latched: k.latched,
            },
        );
        f[0] += xo.fx;
        // Bristle states are held through the RK4 step (zero rate here) and advanced
        // exactly afterwards (`advance_bristles`; PHYSICS §24).
        dy[S_CAVITY] = xo.ddp;

        // Gravity and the floor inertial load (PHYSICS §5).
        f[2] -= m * G0_MPS2;
        let f_floor = v_scale(a_floor, -m);
        f = v_add(&f, &f_floor);

        // Hand (algebraic loop through the arm mass; PHYSICS §22).
        let f_hand = match k.u.f_ext.hand {
            Some(hi) => hand_force(&hi, x - k.hand_anchor, vx, m, f[0]),
            None => 0.0,
        };
        f[0] += f_hand;

        let a_c = v_scale(&f, 1.0 / m);
        dy[S_POS..S_POS + 3].copy_from_slice(&vel);
        dy[S_VEL..S_VEL + 3].copy_from_slice(&a_c);
        // Rotation (Euler's equations in the body frame).
        let tau_b = mat_t_vec(&r, &tau);
        let jw = [k.j[0] * w_b[0], k.j[1] * w_b[1], k.j[2] * w_b[2]];
        let gyro = v_cross(&w_b, &jw);
        let w_dot = [
            if k.j[0] > 0.0 {
                (tau_b[0] - gyro[0]) / k.j[0]
            } else {
                0.0
            },
            if k.j[1] > 0.0 {
                (tau_b[1] - gyro[1]) / k.j[1]
            } else {
                0.0
            },
            if k.j[2] > 0.0 {
                (tau_b[2] - gyro[2]) / k.j[2]
            } else {
                0.0
            },
        ];
        dy[S_OMEGA..S_OMEGA + 3].copy_from_slice(&w_dot);
        let qd = quat_mul(&q, &[0.0, w_b[0], w_b[1], w_b[2]]);
        for j in 0..4 {
            dy[S_QUAT + j] = 0.5 * qd[j];
        }

        // Power (PHYSICS §14).
        let p_load = p_coil_in + lf.p_elec_w + d.power.p_electronics_w;
        let pf = power::power_flow(&d.power, y[S_VHOLD], p_load, k.u.mains_ok);
        dy[S_VHOLD] = pf.dv_hold_dt_v_per_s;

        // Ledger quadratures (PHYSICS §9).
        let lg = S_LEDGER;
        dy[lg + L_DIST] = -xo.drag.dist_n * vx;
        dy[lg + L_END_ZONE] = -xo.drag.end_zone_n * vx;
        dy[lg + L_RAIL_EDDY] = -xo.drag.rail_eddy_n * vx;
        dy[lg + L_RAIL_HYST] = -xo.drag.rail_hyst_n * vx;
        dy[lg + L_AIR] = -xo.drag.air_n * vx + xo.air_w;
        dy[lg + L_FRICTION] = xo.friction_w + guide_w;
        dy[lg + L_CONTACT] = absorbed + xo.contact_w;
        dy[lg + L_BUMPER] = xo.bumper_w;
        dy[lg + L_COIL_CU] = p_cu.iter().take(n).sum();
        dy[lg + L_LSM_CU] = lf.p_copper_w;
        dy[lg + L_ELEC_IN] = p_coil_in + lf.p_elec_w;
        dy[lg + L_HAND] = f_hand * vx;
        dy[lg + L_FLOOR] = v_dot(&f_floor, &vel);

        // Reference-point acceleration (for the truth and the IMU).
        let w_dot_r = mat_vec(&r, &w_dot);
        let arm = v_scale(&rc, -1.0);
        aux.a_ref = v_add(
            &v_add(&a_c, &v_cross(&w_dot_r, &arm)),
            &v_cross(&w_r, &v_cross(&w_r, &arm)),
        );
        aux.p_ref = p_ref;
        aux.v_ref = v_ref;
        aux.lsm = lf;
        aux.f_hand = f_hand;
        aux.pf = pf;
        let fin = d.guidance.fin_present;
        aux.contact = ContactForces {
            upper_n: n_upper,
            rollers_n: n_rollers,
            seat_n: n_seat,
            guide_fin_n: if fin { gf.normal_n } else { 0.0 },
            pads_n: if fin { 0.0 } else { gf.normal_n },
            bumper_n: xo.bumper_n,
            end_stop_n: xo.end_stop_n,
            lock_n: xo.lock_n,
            latch_n: xo.latch_n,
            obstacle_n: xo.obstacle_n,
            tangential_x_n: xo.f_t[0] + xo.f_t[1] + xo.f_t[2] + gf.fx_n,
        };
        dy
    }

    /// Writes the truth from an end-of-step evaluation.
    fn fill_truth(&mut self, aux: &Aux, t_s: f64, floor: &Vec3) {
        let y = &self.y;
        let n = self.com.n;
        let m = self.mass();
        let j = self.inertia(m);
        let q = [y[S_QUAT], y[S_QUAT + 1], y[S_QUAT + 2], y[S_QUAT + 3]];
        let r = quat_to_mat(&q);
        let w = [y[S_OMEGA], y[S_OMEGA + 1], y[S_OMEGA + 2]];
        let vel = [y[S_VEL], y[S_VEL + 1], y[S_VEL + 2]];
        let ke = 0.5 * m * v_dot(&vel, &vel)
            + 0.5 * (j[0] * w[0] * w[0] + j[1] * w[1] * w[1] + j[2] * w[2] * w[2]);
        let mut gaps = [0.0; MAX_MAGNETS];
        let mut coil_i = [0.0; MAX_MAGNETS];
        gaps[..n].copy_from_slice(&aux.gaps[..n]);
        coil_i[..n].copy_from_slice(&aux.coil_i[..n]);
        let mut ts = ThermalState::uniform(y[S_TMAG]);
        ts.t_coil_c[..n].copy_from_slice(&y[S_TCOIL..S_TCOIL + n]);
        let a_abs = mat_t_vec(&r, &v_add(&aux.a_ref, floor));
        let ct = aux.contact;
        let t = &mut self.truth;
        t.t_s = t_s;
        t.x_m = aux.p_ref[0];
        t.v_mps = aux.v_ref[0];
        t.a_mps2 = aux.a_ref[0];
        let qn = quat_normalize(&q);
        t.pose = [
            aux.p_ref[0],
            aux.p_ref[1],
            aux.p_ref[2],
            qn[0],
            qn[1],
            qn[2],
            qn[3],
        ];
        t.omega_radps = w;
        t.acc_abs_mps2 = a_abs;
        t.gaps_m = gaps;
        t.coil_i_a = coil_i;
        t.iq_a = aux.lsm.iq_a;
        t.id_a = aux.lsm.id_a;
        t.i_abc_a = lsm::dq_to_abc(&self.p.design.lsm, aux.p_ref[0], aux.lsm.iq_a, aux.lsm.id_a);
        t.f_lsm_n = aux.lsm.fx_n;
        t.f_contact = ct;
        t.f_hand_n = aux.f_hand;
        t.e_diss = ledger_from(
            y,
            S_LEDGER,
            ke,
            self.bumper_counts.bumper_impacts,
            self.bumper_counts.bumper_e_max_j,
        );
        t.floor_acc_mps2 = *floor;
        t.temps_c = [ts.hot_spot_c(n), y[S_TMAG]];
        t.cavity_dp_pa = y[S_CAVITY];
        t.power = power_truth(&self.p, &aux.pf, y[S_VHOLD], t.power.mains_ok);
        t.state = ContactState {
            levitated: ct.rollers_n <= 0.0 && ct.seat_n <= 0.0 && ct.upper_n <= 0.0,
            seated: ct.seat_n > 0.0,
            landed: ct.rollers_n > 0.0,
            upper_stop: ct.upper_n > 0.0,
            locked: self.lock_anchor.is_some(),
            latched: self.latched,
        };
        t.mass_kg = m;
        t.payload_kg = self.payload_kg;
    }
}

impl PlantModel for Carriage6 {
    fn tier(&self) -> Tier {
        Tier::Carriage6
    }

    fn reset(&mut self, init: &InitialState) {
        let p = &self.p;
        let d = &p.design;
        self.truth = Truth::initial(p, init);
        self.payload_kg = init.payload_kg;
        self.y = [0.0; N_STATE];
        self.i2t = [I2tBudget::default(); MAX_MAGNETS];
        self.lsm_iq = 0.0;
        self.lsm_id = 0.0;
        self.hand_anchor = None;
        self.latched = false;
        self.lock_anchor = if init.locked && d.landing.parking_lock {
            Some(init.x_m)
        } else {
            None
        };
        self.bumper_log = contact::BumperLog::default();
        self.bumper_counts = EnergyLedger::default();
        let m = self.mass();
        let h = self.h_last;
        // Quasi-static rest gap (PHYSICS §3, §8); the iron-core LSM attraction relieves
        // the lift elements.
        let f_n = lsm::thrust(&d.lsm, init.x_m, 0.0, 0.0, 0.0, LsmMode::Sto).fn_n;
        let gap = rest_gap(p, &self.com, init.lev, m, f_n, init.x_m, init.temp_c, h);
        let z_ref = d.suspension.g_star_m - gap;
        let c = d.carriage.cg_m;
        let pos = [init.x_m + c[0], c[1], z_ref + c[2]];
        self.y[S_POS..S_POS + 3].copy_from_slice(&pos);
        self.y[S_VEL] = init.v_mps;
        self.y[S_QUAT] = 1.0;
        for k in 0..MAX_MAGNETS {
            self.y[S_TCOIL + k] = init.temp_c;
        }
        self.y[S_TMAG] = init.temp_c;
        self.y[S_VHOLD] = d.power.supercap_v_max_v;
        // Rail profile: shift the body so the mean lift gap is the rest gap.
        if p.env.rail.is_some() && self.com.n_lift > 0 {
            let mean_z: f64 = (0..self.com.n)
                .filter(|&k| self.com.st[k].lift)
                .map(|k| rail_at(p, init.x_m + self.com.st[k].r[0]).z_m)
                .sum::<f64>()
                / self.com.n_lift as f64;
            self.y[S_POS + 2] += mean_z;
        }
        self.aux = Aux::default();
        let s = &d.suspension;
        for k in 0..self.com.n {
            let ev = self.com.st[k].model.eval(
                &ElementGeom {
                    g_m: gap,
                    y_m: 0.0,
                    e: 1.0,
                    de_dx_per_m: 0.0,
                },
                0.0,
                init.temp_c,
            );
            self.aux.l_inc[k] = self.com.st[k].model.dpsi_di_h(&ev);
            self.aux.r_coil[k] = coil::r_at(s.r_coil_ohm, s.tempco_r_per_k, init.temp_c);
        }
        self.truth.power.mains_ok = true;
        self.truth.floor_acc_mps2 = [0.0; 3];
        self.truth.t_s = 0.0;
        let u = PlantInputs {
            payload_kg: init.payload_kg,
            t_ambient_c: init.temp_c,
            ..PlantInputs::default()
        };
        self.refresh(&u);
    }

    fn step(&mut self, t: f64, h: f64, u: &PlantInputs, floor_mps2: &[[f64; 3]; 3]) {
        self.h_last = h;
        self.payload_kg = u.payload_kg;
        let d = &self.p.design;
        let x0 = self.truth.x_m;
        // Discrete engagements at the step start.
        self.hand_anchor = match (u.f_ext.hand, self.hand_anchor) {
            (Some(_), Some(a)) => Some(a),
            (Some(_), None) => Some(x0),
            (None, _) => None,
        };
        self.lock_anchor = match (u.parking_lock && d.landing.parking_lock, self.lock_anchor) {
            (true, Some(a)) => Some(a),
            (true, None) => Some(x0),
            (false, _) => None,
        };
        let latch_now = u.seismic_latch && d.landing.seismic_latch.enabled;
        if latch_now && !self.latched {
            // A latch engages undeflected.
            self.y[S_BRISTLE + 3] = 0.0;
        }
        self.latched = latch_now;
        self.truth.power.mains_ok = u.mains_ok;
        let ctx = self.ctx(u, h);
        let y0 = self.y;
        let mut scratch = Aux::default();
        let mut y1 = rk4_step(&y0, t, h, |stage, _t, y| {
            let s = [0.0, 0.5 * h, 0.5 * h, h][stage];
            self.eval(
                &ctx,
                s,
                &floor_mps2[STAGE_FLOOR_INDEX[stage]],
                y,
                &mut scratch,
            )
        });
        // Post-step: bristle states (exact split update over the reference point's slide),
        // quaternion, algebraic currents and fluxes.
        let x1 = {
            let q1 = [y1[S_QUAT], y1[S_QUAT + 1], y1[S_QUAT + 2], y1[S_QUAT + 3]];
            let r1 = quat_to_mat(&quat_normalize(&q1));
            y1[S_POS] - mat_vec(&r1, &self.p.design.carriage.cg_m)[0]
        };
        advance_bristles(
            &self.p,
            &self.com,
            &mut y1[S_BRISTLE..S_BRISTLE + N_BRISTLE],
            x1 - x0,
            h,
            ctx.latched,
        );
        let qn = quat_normalize(&[y1[S_QUAT], y1[S_QUAT + 1], y1[S_QUAT + 2], y1[S_QUAT + 3]]);
        y1[S_QUAT..S_QUAT + 4].copy_from_slice(&qn);
        let n = self.com.n;
        if !ctx.voltage {
            for k in 0..n {
                y1[S_COIL + k] = ctx.coil[k].at(h);
            }
        }
        self.y = y1;
        let mut aux = Aux::default();
        let _ = self.eval(&ctx, h, &floor_mps2[2], &self.y, &mut aux);
        if !ctx.lag_state {
            for k in 0..n {
                self.y[S_FLUX + k] = aux.phi[k];
            }
        }
        match u.lsm_mode {
            LsmMode::Asc => {
                self.lsm_iq = aux.lsm.iq_a;
                self.lsm_id = aux.lsm.id_a;
            }
            _ => {
                self.lsm_iq = ctx.lsm_q.at(h);
                self.lsm_id = ctx.lsm_d.at(h);
            }
        }
        let s = &self.p.design.suspension;
        for k in 0..n {
            self.i2t[k].update(self.y[S_COIL + k], h, s.i_cont_a, s.i_pulse_a, s.t_pulse_s);
        }
        // Bumper impacts: onsets and the largest onset kinetic energy (contact::BumperLog).
        let delta = contact::bumper_penetration(&self.p.design, aux.p_ref[0]).0;
        let (pp, v, m) = (&self.p, aux.v_ref[0], ctx.m);
        self.bumper_log
            .update(pp, &mut self.bumper_counts, delta, v, m);
        self.aux = aux;
        self.fill_truth(&aux, t + h, &floor_mps2[2]);
    }

    fn truth(&self) -> &Truth {
        &self.truth
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{CoilMode, build};
    use crate::params::{Environment, Mismatch};
    use fabrail_params::presets;

    fn plant(sp: &fabrail_params::SystemParams) -> PlantParams {
        PlantParams::from_design(&sp.design, &Mismatch::default(), &Environment::default())
    }

    #[test]
    fn quaternion_helpers() {
        let q = quat_from_rpy(0.1, -0.2, 0.3);
        let r = quat_to_mat(&q);
        // Orthonormal.
        for i in 0..3 {
            for j in 0..3 {
                let d: f64 = (0..3).map(|k| r[k][i] * r[k][j]).sum();
                assert!((d - if i == j { 1.0 } else { 0.0 }).abs() < 1e-14);
            }
        }
        // Small roll moves +y to +z (right-handed about x).
        let r = quat_to_mat(&quat_from_rpy(1e-3, 0.0, 0.0));
        let v = mat_vec(&r, &[0.0, 1.0, 0.0]);
        assert!((v[2] - 1e-3).abs() < 1e-9);
    }

    #[test]
    fn reset_levitates_at_the_zero_power_gap() {
        // m* at 20 °C floats at g* with zero current (PHYSICS §3; the pole length is solved
        // for the balance, params::presets::finish).
        for sp in [presets::drawer(), presets::door(), presets::wall()] {
            let p = plant(&sp);
            let mut c = Carriage6::new(&p, &ModelConfig::default());
            // Mid travel: the DRAWER cannot levitate at g* inside its captive seat slot.
            c.reset(&InitialState {
                x_m: 0.5 * sp.design.rail.travel_m,
                ..InitialState::default()
            });
            let t = c.truth();
            for k in 0..usize::from(sp.design.suspension.n_magnets) {
                assert!(
                    (t.gaps_m[k] - sp.design.suspension.g_star_m).abs() < 1e-9,
                    "gap {} vs g* (PHYSICS §3)",
                    t.gaps_m[k]
                );
            }
            assert!(t.state.levitated);
            assert!(t.pose[2].abs() < 1e-9);
        }
    }

    // req: FAB-SW-005
    #[test]
    fn sign_conventions() {
        // x travel (+ opens), y cross-track, z up, g > 0 (PHYSICS §1; CRIT G12).
        let sp = presets::door();
        let p = plant(&sp);
        let cfg = ModelConfig {
            tier: Tier::Carriage6,
            coil: CoilMode::Lag,
            eddy_lag: false,
            kinematic_heave: true,
        };
        let h = Tier::Carriage6.nominal_h_s();
        let floor0 = [[0.0; 3]; 3];
        // 1. +i_q thrust opens (+x) with a positive velocity.
        let mut c = Carriage6::new(&p, &cfg);
        c.reset(&InitialState {
            x_m: 0.5,
            ..InitialState::default()
        });
        let u = PlantInputs {
            coil_enabled: true,
            lsm_mode: LsmMode::Run,
            iq_ref_a: 2.0,
            ..PlantInputs::default()
        };
        for k in 0..200 {
            c.step(k as f64 * h, h, &u, &floor0);
        }
        assert!(
            c.truth().f_lsm_n > 0.0 && c.truth().v_mps > 0.0 && c.truth().x_m > 0.5,
            "+x opens"
        );
        // 2. Lifting (more current) closes the gaps and raises z; gravity with coils off
        //    opens them and lowers z; every gap stays positive.
        let mut c = Carriage6::new(&p, &cfg);
        let up = PlantInputs {
            coil_enabled: true,
            coil_v_or_iref: [0.5; MAX_MAGNETS],
            ..PlantInputs::default()
        };
        for k in 0..100 {
            c.step(k as f64 * h, h, &up, &floor0);
        }
        let t = c.truth();
        assert!(
            t.pose[2] > 0.0 && t.gaps_m[0] < sp.design.suspension.g_star_m,
            "z up closes the gap"
        );
        let mut c = Carriage6::new(&p, &cfg);
        let down = PlantInputs {
            coil_enabled: true,
            coil_v_or_iref: [-0.5; MAX_MAGNETS],
            ..PlantInputs::default()
        };
        for k in 0..100 {
            c.step(k as f64 * h, h, &down, &floor0);
        }
        let t = c.truth();
        assert!(t.pose[2] < 0.0 && t.gaps_m[0] > sp.design.suspension.g_star_m);
        assert!(t.gaps_m[..4].iter().all(|&g| g > 0.0), "g > 0");
        // 3. A +y lateral offset meets a −y reluctance restoring force.
        let mut c = Carriage6::new(&p, &cfg);
        c.set_body_state([0.0, 1.0e-3, 0.0], [0.0; 3], [0.0; 3], [0.0; 3]);
        let hold = PlantInputs {
            coil_enabled: true,
            ..PlantInputs::default()
        };
        c.step(0.0, h, &hold, &floor0);
        assert!(c.truth().pose[1] > 0.0 && c.truth().pose[1] < 1.0e-3 && c.aux.y_off[0] > 0.0);
        let ev = c.com.st[0].model.eval(
            &ElementGeom {
                g_m: c.aux.gaps[0],
                y_m: c.aux.y_off[0],
                e: 1.0,
                de_dx_per_m: 0.0,
            },
            0.0,
            20.0,
        );
        assert!(ev.loads(ev.phi_wb).fy_n < 0.0, "restoring −y");
        // 4. A +x floor acceleration is an inertial load −m·a: the body moves −x relative
        //    to the rail.
        let mut c = Carriage6::new(&p, &cfg);
        let floor = [[1.0, 0.0, 0.0]; 3];
        for k in 0..100 {
            c.step(k as f64 * h, h, &hold, &floor);
        }
        assert!(
            c.truth().v_mps < 0.0 && c.truth().a_mps2 < 0.0,
            "−m·a_floor"
        );
        // Inertially almost at rest along x: only drag, rail hysteresis and the bristle-free
        // levitated gap act (≤ 5 N on 100 kg).
        assert!(
            c.truth().acc_abs_mps2[0].abs() < 0.05,
            "inertially at rest along x"
        );
    }

    #[test]
    fn rest_states_hold_still() {
        // Every class at its rest policy (DRAWER seated, DOOR hovering, WALL landed, HEAVY
        // landed and locked) with the coils at 0 A stays put for 0.2 s: the quasi-static
        // reset is an equilibrium of the contact model (PHYSICS §8; core design §C.2 rest
        // states). Hover is open-loop unstable, so only its short-term drift is bounded.
        for sp in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let p = plant(&sp);
            let mut c = Carriage6::new(&p, &ModelConfig::default());
            let (lev, locked) = match sp.design.rest_policy {
                fabrail_types::RestPolicy::Seat => (InitLev::Seated, false),
                fabrail_types::RestPolicy::Hover => (InitLev::Levitated, false),
                fabrail_types::RestPolicy::Land => (InitLev::Landed, false),
                fabrail_types::RestPolicy::LandLock => (InitLev::Landed, true),
            };
            let x0 = if matches!(lev, InitLev::Levitated) {
                0.5 * sp.design.rail.travel_m
            } else {
                0.0
            };
            c.reset(&InitialState {
                lev,
                x_m: x0,
                locked,
                ..InitialState::default()
            });
            let z0 = c.truth().pose[2];
            let u = PlantInputs {
                coil_enabled: true,
                parking_lock: locked,
                ..PlantInputs::default()
            };
            let h = Tier::Carriage6.nominal_h_s();
            for k in 0..4000 {
                c.step(k as f64 * h, h, &u, &[[0.0; 3]; 3]);
            }
            let t = c.truth();
            let dz = (t.pose[2] - z0).abs();
            let dx = (t.x_m - x0).abs();
            match lev {
                InitLev::Seated => {
                    assert!(t.state.seated && dz < 1e-6 && dx < 1e-6, "{:?}", t.pose)
                }
                InitLev::Landed => {
                    assert!(t.state.landed && dz < 1e-6 && dx < 1e-6, "{:?}", t.pose);
                    assert_eq!(t.state.locked, locked);
                }
                InitLev::Levitated => assert!(t.state.levitated && dz < 1e-6, "hover drift {dz}"),
            }
            assert!(t.pose.iter().all(|v| v.is_finite()));
        }
    }

    #[test]
    fn every_class_steps_through_build() {
        for sp in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let p = plant(&sp);
            let mut m = build(&ModelConfig::default(), &p).expect("build");
            let init = InitialState {
                lev: match sp.design.rest_policy {
                    fabrail_types::RestPolicy::Seat => InitLev::Seated,
                    fabrail_types::RestPolicy::Hover => InitLev::Levitated,
                    _ => InitLev::Landed,
                },
                locked: matches!(sp.design.rest_policy, fabrail_types::RestPolicy::LandLock),
                ..InitialState::default()
            };
            m.reset(&init);
            let h = Tier::Carriage6.nominal_h_s();
            for k in 0..20 {
                m.step(k as f64 * h, h, &PlantInputs::default(), &[[0.0; 3]; 3]);
            }
            let t = m.truth();
            assert!(t.x_m.is_finite() && t.pose.iter().all(|v| v.is_finite()));
            assert!(t.gaps_m.iter().all(|g| g.is_finite()));
        }
    }
}
