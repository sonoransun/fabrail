//! Penalty contacts: upper stop/rollers, landing rollers, guide fin, DRAWER seat, bumpers (Hunt-Crossley), parking lock, edge seal (PHYSICS §8). Owner: A6b.
//!
//! Normal law F_n = max(0, k_c·δ + c_c·δ̇) for penetration δ > 0 (never pulls), with
//! c_c = 2ζ_c·√(k_c·m_eff), ζ_c = 0.3, and k_c = min(k_phys, m_eff·(0.2/h)²) so every
//! contact mode satisfies ω·h ≤ 0.2 (PHYSICS §8, §24; `stability::report` checks it).
//! Impact-type contacts (end bumpers, guide fin, side pads) use Hunt–Crossley
//! F = k·δ·(1 + hc·δ̇), whose dissipation vanishes at first touch and needs no mass
//! (Hunt & Crossley, J. Appl. Mech. 42(2), 1975). Tangential forces come from
//! `friction::bristle`.
//!
//! Stiffness sources (all plant truth; no `fabrail-params` field exists for them, so they
//! are module constants with their source):
//!
//! | Contact | k_phys | Friction |
//! |---|---|---|
//! | Landing / travel rollers, DRAWER seat | steel, ≥ 1e8 N/m and ≤ 5 µm at the rated load | μ 0.003 rollers (plan §2b item 19); seat UHMW μ 0.15 |
//! | Upper stop (elastomer-faced rollers) | ≥ 1e7 N/m and ≤ 50 µm at the roller rating | μ 0.003 |
//! | Guide fin, side pads | UHMW 5e6 N/m, Hunt–Crossley | μ 0.15 |
//! | End bumpers | `damping.bumper_k_n_per_m`, Hunt–Crossley | — |
//! | Parking lock / seismic latch | 1e6 N/m bilateral | latch slips at `x_retention_n` |
//! | Closing-edge seal | `damping.k_edge_n_per_m` (≤ 10 N/mm, F6) | — |
//!
//! Effective masses (the k_c cap): vertical contacts use the smallest per-support share
//! over heave, roll and pitch at the smallest truth mass; lateral contacts the smaller of
//! the translation and the yaw/roll share at the body ends; travel-axis contacts the
//! whole smallest mass. Vertical and lateral contacts are capped at the 50 µs step (they
//! exist only on Carriage6); travel-axis contacts at the 1 ms Travel1d step, so one
//! parameter set serves every tier ([`contact_params`]); [`contact_params_at`] takes an
//! explicit step (and picks the tier's presliding length, 10 µm or 50 µm).
//!
//! End zones and bumpers sit outside the nominal travel [x_closed, x_closed + travel]
//! (SAF-014: nominal termini never engage them): the progressive eddy brake spans
//! `damping.ez_len_m` beyond each limit (`dissipation::end_zone_c`) and the bumper faces
//! are at its outer edges ([`bumper_penetration`]).
//!
//! Physical end stops at the class geometry (W2 lead decision D3; [`EndStops`]): the
//! closed end is a rigid jamb [`JAMB_CLEARANCE_M`] past x_closed (DOOR, WALL, HEAVY), or
//! the DRAWER carcass (push-stop spring from x_closed, hard stop at x_closed −
//! `seat_x_compliance`); the open end is the rail's end stop behind the bumper, where
//! the bumper bottoms out at its rated stroke √(2·E_rated/k_b). Every tier (Carriage6,
//! Travel1d, Structure) applies the same geometry through [`closed_stop_force_n`] and
//! [`open_stop_force_n`], so a free or landed carriage driven by floor motion with the
//! LSM off stays between the stops; the closed-end part of the end zone and bumper lies
//! behind the jamb and is unreachable.

use fabrail_params::derive::{G0_MPS2, n_lift, pm_force};
use fabrail_params::{DesignParams, LandingKind};
use fabrail_types::{BASE_TICK_S, MAX_MAGNETS, fmath};

use crate::friction::{BristleParams, Z_SS_CARRIAGE6_M, Z_SS_TRAVEL1D_M};
use crate::model::{EnergyLedger, Tier};
use crate::params::PlantParams;

/// Contact damping ratio ζ_c (physics perspective §1.6: c_c = 2ζ_c√(k_c·m_eff), ζ_c = 0.3;
/// PHYSICS §8).
pub const ZETA_CONTACT: f64 = 0.3;

/// Static penetration budget under the rated weight, m (physics perspective §1.6: m·g/k_c
/// ≤ 10 µm, asserted; PHYSICS §8).
pub const STATIC_PENETRATION_MAX_M: f64 = 10.0e-6;

/// Steel landing/travel roller and V-seat stiffness floor, N/m (physics perspective §1.6:
/// "steel k 1e8 N/m (capped)"; PHYSICS §8).
pub const K_ROLLER_STEEL_N_PER_M: f64 = 1.0e8;

/// Design static deflection of a steel roller or seat at its share of the rated load, m
/// (design choice: rollers are sized for ≤ 5 µm, half the 10 µm penetration budget, which
/// sets k for WALL/HEAVY above the 1e8 floor; PHYSICS §8).
pub const ROLLER_DEFLECTION_M: f64 = 5.0e-6;

/// Elastomer-faced upper roller stiffness floor, N/m (physics perspective §1.6: "k 1e7";
/// PHYSICS §8).
pub const K_UPPER_ELASTOMER_N_PER_M: f64 = 1.0e7;

/// Design deflection of an elastomer upper roller at its rating share, m (design choice:
/// ≤ 50 µm, a 2 mm polyurethane face at a few % strain; PHYSICS §8).
pub const UPPER_DEFLECTION_M: f64 = 50.0e-6;

/// UHMW-PE pad / guide-fin contact stiffness, N/m \[UNCERTAIN\] (design choice: a 5 mm
/// UHMW-PE pad, E ≈ 0.7 GPa, over ≈ 35 mm² of effective contact; 20 N lateral → 4 µm;
/// PHYSICS §6, §8).
pub const K_UHMW_N_PER_M: f64 = 5.0e6;

/// Hunt–Crossley factor of the UHMW contacts, s/m \[UNCERTAIN\] (hc = 3(1 − e)/(2·v_i),
/// Hunt & Crossley 1975; e ≈ 0.55 at v_i = 0.45 m/s for UHMW-PE on steel; PHYSICS §8).
pub const HC_UHMW_S_PER_M: f64 = 1.5;

/// Parking-lock and seismic-latch bilateral stiffness, N/m (physics perspective §1.6:
/// "bilateral x spring 1e6 N/m when engaged"; PHYSICS §8).
pub const K_LOCK_N_PER_M: f64 = 1.0e6;

/// Contact elements of the plant (PHYSICS §8).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ContactKind {
    /// Upper stop / elastomer upper rollers (DRAWER ≥ 550 N rating, F2).
    UpperStop,
    /// Landing / travel rollers (μ 0.003, plan §2b item 19).
    LowerRollers,
    /// DRAWER captive seat (z both ways, x push-stop spring).
    Seat,
    /// Bottom guide fin (±2 mm clearance; DOOR/WALL/HEAVY).
    GuideFin,
    /// UHMW side pads (μ 0.15).
    SidePads,
    /// End bumpers (Hunt–Crossley).
    Bumper,
    /// Parking lock (x retention).
    ParkingLock,
    /// Seismic x-latch (x retention, `landing.seismic_latch`).
    SeismicLatch,
    /// Closing-edge seal (k_edge ≤ 10 N/mm, F6) in series with an obstacle body.
    EdgeSeal,
}

impl ContactKind {
    /// Every contact kind.
    pub const ALL: &'static [ContactKind] = &[
        ContactKind::UpperStop,
        ContactKind::LowerRollers,
        ContactKind::Seat,
        ContactKind::GuideFin,
        ContactKind::SidePads,
        ContactKind::Bumper,
        ContactKind::ParkingLock,
        ContactKind::SeismicLatch,
        ContactKind::EdgeSeal,
    ];

    /// Snake-case name (stability-report element `contact.<name>`).
    pub const fn name(self) -> &'static str {
        match self {
            ContactKind::UpperStop => "upper_stop",
            ContactKind::LowerRollers => "rollers",
            ContactKind::Seat => "seat",
            ContactKind::GuideFin => "guide_fin",
            ContactKind::SidePads => "side_pads",
            ContactKind::Bumper => "bumper",
            ContactKind::ParkingLock => "parking_lock",
            ContactKind::SeismicLatch => "seismic_latch",
            ContactKind::EdgeSeal => "edge_seal",
        }
    }

    /// The normal acts along the travel axis x (bumpers, lock, latch, edge seal): these
    /// contacts also exist on the 1 ms Travel1d tier.
    pub const fn is_travel_axis(self) -> bool {
        matches!(
            self,
            ContactKind::Bumper
                | ContactKind::ParkingLock
                | ContactKind::SeismicLatch
                | ContactKind::EdgeSeal
        )
    }

    /// The normal law is Hunt–Crossley (`ContactParams::c_n_s_per_m` is then the factor
    /// hc in s/m, not a damping coefficient).
    pub const fn is_hunt_crossley(self) -> bool {
        matches!(
            self,
            ContactKind::Bumper | ContactKind::GuideFin | ContactKind::SidePads
        )
    }
}

/// One contact element's normal and tangential law.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContactParams {
    /// Normal stiffness k_c, N/m.
    pub k_n_per_m: f64,
    /// Normal damping c_c, N·s/m (Hunt–Crossley factor hc in s/m for the kinds where
    /// [`ContactKind::is_hunt_crossley`]: bumpers, guide fin, side pads).
    pub c_n_s_per_m: f64,
    /// Tangential bristle friction.
    pub bristle: BristleParams,
}

impl ContactParams {
    /// True when the element is not fitted (zero stiffness).
    pub fn is_inert(&self) -> bool {
        self.k_n_per_m.is_nan() || self.k_n_per_m <= 0.0
    }
}

/// Penalty normal force for penetration `delta_m` (> 0 = in contact) and its rate:
/// max(0, k·δ + c·δ̇); zero out of contact. Never pulls (PHYSICS §8).
pub fn penalty_normal_n(k_n_per_m: f64, c_n_s_per_m: f64, delta_m: f64, delta_dot_mps: f64) -> f64 {
    if delta_m <= 0.0 || delta_m.is_nan() {
        return 0.0;
    }
    (k_n_per_m * delta_m + c_n_s_per_m * delta_dot_mps).max(0.0)
}

/// Hunt–Crossley bumper force k·δ·(1 + hc·δ̇) for δ > 0, clipped at 0 (Hunt & Crossley
/// 1975, linear-stiffness form; PHYSICS §8).
pub fn hunt_crossley_n(k_n_per_m: f64, hc_s_per_m: f64, delta_m: f64, delta_dot_mps: f64) -> f64 {
    if delta_m <= 0.0 || delta_m.is_nan() {
        return 0.0;
    }
    (k_n_per_m * delta_m * (1.0 + hc_s_per_m * delta_dot_mps)).max(0.0)
}

/// Power dissipated by a Hunt–Crossley contact, k·hc·δ·δ̇² while it pushes (≥ 0), W
/// (the damping part of [`hunt_crossley_n`] times δ̇; PHYSICS §8). Feeds the bumper
/// ledger.
pub fn hunt_crossley_dissipation_w(
    k_n_per_m: f64,
    hc_s_per_m: f64,
    delta_m: f64,
    delta_dot_mps: f64,
) -> f64 {
    if hunt_crossley_n(k_n_per_m, hc_s_per_m, delta_m, delta_dot_mps) <= 0.0 {
        return 0.0;
    }
    (k_n_per_m * hc_s_per_m * delta_m * delta_dot_mps * delta_dot_mps).max(0.0)
}

/// Largest penalty stiffness an explicit RK4 step `h_s` resolves on an effective mass
/// `m_eff_kg`: m_eff·(0.2/h)² (ω·h ≤ 0.2; PHYSICS §8, §24).
pub fn k_max_n_per_m(m_eff_kg: f64, h_s: f64) -> f64 {
    let r = 0.2 / h_s;
    m_eff_kg * r * r
}

/// Critical-fraction penalty damping c = 2ζ_c·√(k·m_eff), N·s/m (PHYSICS §8).
pub fn contact_damping_n_s_per_m(k_n_per_m: f64, m_eff_kg: f64) -> f64 {
    if !(k_n_per_m > 0.0 && m_eff_kg > 0.0) {
        return 0.0;
    }
    2.0 * ZETA_CONTACT * fmath::sqrt(k_n_per_m * m_eff_kg)
}

/// Smallest truth mass the plant can carry: min(m_body, m_op_min), with m_op_min ignored
/// when unset (≤ 0), kg. Payload events may lighten the body down to the empty
/// operating mass (truth-mass rule, core design §B.6).
pub fn m_min_kg(p: &PlantParams) -> f64 {
    let m_op = p.design.carriage.m_op_min_kg;
    if m_op > 0.0 && m_op.is_finite() {
        p.m_body_kg.min(m_op)
    } else {
        p.m_body_kg
    }
    .max(0.0)
}

/// Principal inertias about the CoM scaled from the m* design to mass `m_kg`, kg·m².
fn inertia_at(d: &DesignParams, m_kg: f64) -> [f64; 3] {
    let m_star = d.carriage.m_star_kg;
    let s = if m_star > 0.0 { m_kg / m_star } else { 1.0 };
    let j = d.carriage.j_kgm2;
    [j[0] * s, j[1] * s, j[2] * s]
}

/// Active element stations (the vertical support points: rollers, seat and upper stop
/// sit at the element stations, PHYSICS §8).
fn stations(d: &DesignParams) -> impl Iterator<Item = &fabrail_params::MagnetStation> {
    let n = usize::from(d.suspension.n_magnets).min(MAX_MAGNETS);
    d.suspension.stations.iter().take(n)
}

/// Effective mass of one vertical support point: the smallest of heave m/n, roll
/// J_x/Σy_k² and pitch J_y/Σx_k² (offsets from the CoM) at the smallest truth mass, kg
/// (PHYSICS §8: each support mode has ω² = k/m_eff).
pub fn m_eff_vertical_kg(p: &PlantParams) -> f64 {
    let d = &p.design;
    let m = m_min_kg(p);
    let n = stations(d).count();
    if n == 0 {
        return m;
    }
    let j = inertia_at(d, m);
    let cg = d.carriage.cg_m;
    let (mut sx2, mut sy2) = (0.0f64, 0.0f64);
    for st in stations(d) {
        sx2 += (st.x_m - cg[0]) * (st.x_m - cg[0]);
        sy2 += (st.y_m - cg[1]) * (st.y_m - cg[1]);
    }
    let mut m_eff = m / n as f64;
    if sy2 > 0.0 && j[0] > 0.0 {
        m_eff = m_eff.min(j[0] / sy2);
    }
    if sx2 > 0.0 && j[1] > 0.0 {
        m_eff = m_eff.min(j[1] / sx2);
    }
    m_eff
}

/// Effective mass at a lateral guide point: the smallest of the translation m, the yaw
/// share J_z/(L/2)² at the body ends and the roll share J_x/cg_z² at the far edge of a
/// top-hung leaf (the guide fin), at the smallest truth mass, kg (PHYSICS §6, §8).
pub fn m_eff_lateral_kg(p: &PlantParams) -> f64 {
    let d = &p.design;
    let m = m_min_kg(p);
    let j = inertia_at(d, m);
    let mut m_eff = m;
    let half_l = 0.5 * d.carriage.length_m;
    if half_l > 0.0 && j[2] > 0.0 {
        m_eff = m_eff.min(j[2] / (half_l * half_l));
    }
    let lz = d.carriage.cg_m[2].abs();
    if lz > 0.0 && j[0] > 0.0 {
        m_eff = m_eff.min(j[0] / (lz * lz));
    }
    m_eff
}

/// Rated weight share of one vertical support point, m_rated·g₀/n, N.
fn rated_share_n(d: &DesignParams) -> f64 {
    let n = stations(d).count().max(1) as f64;
    d.carriage.m_rated_kg.max(0.0) * G0_MPS2 / n
}

/// Tier step whose stability bound a contact's stiffness is capped at by
/// [`contact_params`]: 1 ms (Travel1d) for travel-axis contacts, the 50 µs base tick
/// otherwise.
pub fn default_step_s(kind: ContactKind) -> f64 {
    if kind.is_travel_axis() {
        Tier::Travel1d.nominal_h_s()
    } else {
        BASE_TICK_S
    }
}

/// Presliding length for a tier step: 10 µm on the 50 µs tiers, 50 µm on the 1 ms tier
/// (core design §C.5).
pub fn presliding_m(h_s: f64) -> f64 {
    if h_s > 2.0 * BASE_TICK_S {
        Z_SS_TRAVEL1D_M
    } else {
        Z_SS_CARRIAGE6_M
    }
}

/// The contact law for `kind` in plant `p` (PHYSICS §8): [`contact_params_at`] at
/// [`default_step_s`], so a single set is stable on every tier that uses the contact.
/// Tiers stepping at 1 ms that use roller friction (Travel1d) call
/// [`contact_params_at`] with their step for the 50 µm presliding length.
pub fn contact_params(p: &PlantParams, kind: ContactKind) -> ContactParams {
    contact_params_at(p, kind, default_step_s(kind))
}

/// The contact law for `kind` at plant step `h_s`: k = min(k_phys, m_eff·(0.2/h)²),
/// penalty damping at ζ_c = 0.3 (or the Hunt–Crossley factor), and the element's bristle
/// friction with the tier's presliding length (PHYSICS §8, §24). An element that is not
/// fitted (no seat, no fin, no lock or latch, zero bumper/edge stiffness) is inert.
pub fn contact_params_at(p: &PlantParams, kind: ContactKind, h_s: f64) -> ContactParams {
    let d = &p.design;
    if !(h_s.is_finite() && h_s > 0.0) {
        return ContactParams::default();
    }
    let n_sup = stations(d).count().max(1) as f64;
    let rollers_mu = d.landing.rollers_mu;
    let uhmw_mu = d.guidance.pad_mu;
    // (k_phys, m_eff, friction μ); Hunt–Crossley kinds take their factor below.
    let (k_phys, m_eff, mu) = match kind {
        ContactKind::UpperStop => {
            let rating_share = d.landing.upper_roller_rating_n.max(0.0) / n_sup;
            (
                K_UPPER_ELASTOMER_N_PER_M.max(rating_share / UPPER_DEFLECTION_M),
                m_eff_vertical_kg(p),
                rollers_mu,
            )
        }
        ContactKind::LowerRollers => {
            // Skids (sovereign reduced models) slide on UHMW instead of rolling.
            let mu = if matches!(d.landing.kind, LandingKind::Skids) {
                uhmw_mu
            } else {
                rollers_mu
            };
            (
                K_ROLLER_STEEL_N_PER_M.max(rated_share_n(d) / ROLLER_DEFLECTION_M),
                m_eff_vertical_kg(p),
                mu,
            )
        }
        ContactKind::Seat => {
            if !matches!(d.landing.kind, LandingKind::Seat) || d.suspension.g_seat_m <= 0.0 {
                return ContactParams::default();
            }
            (
                K_ROLLER_STEEL_N_PER_M.max(rated_share_n(d) / ROLLER_DEFLECTION_M),
                m_eff_vertical_kg(p),
                uhmw_mu,
            )
        }
        ContactKind::GuideFin => {
            if !d.guidance.fin_present {
                return ContactParams::default();
            }
            (K_UHMW_N_PER_M, m_eff_lateral_kg(p), uhmw_mu)
        }
        ContactKind::SidePads => (K_UHMW_N_PER_M, m_eff_lateral_kg(p), uhmw_mu),
        ContactKind::Bumper => (d.damping.bumper_k_n_per_m, m_min_kg(p), 0.0),
        ContactKind::ParkingLock => {
            if !d.landing.parking_lock {
                return ContactParams::default();
            }
            (K_LOCK_N_PER_M, m_min_kg(p), 0.0)
        }
        ContactKind::SeismicLatch => {
            if !d.landing.seismic_latch.enabled {
                return ContactParams::default();
            }
            (K_LOCK_N_PER_M, m_min_kg(p), 0.0)
        }
        ContactKind::EdgeSeal => (d.damping.k_edge_n_per_m, m_min_kg(p), 0.0),
    };
    if !(k_phys.is_finite() && k_phys > 0.0 && m_eff.is_finite() && m_eff > 0.0) {
        return ContactParams::default();
    }
    let k = k_phys.min(k_max_n_per_m(m_eff, h_s));
    let c = match kind {
        ContactKind::Bumper => d.damping.bumper_hc_s_per_m.max(0.0),
        ContactKind::GuideFin | ContactKind::SidePads => HC_UHMW_S_PER_M,
        _ => contact_damping_n_s_per_m(k, m_eff),
    };
    let bristle = if mu > 0.0 {
        BristleParams::new(mu, mu, presliding_m(h_s))
    } else {
        BristleParams::default()
    };
    ContactParams {
        k_n_per_m: k,
        c_n_s_per_m: c,
        bristle,
    }
}

/// Static penetration of a weight-bearing contact (rollers, seat) under its share of
/// the rated weight, m_rated·g₀/(n·k_c), m; 0 for other or unfitted kinds (PHYSICS §8:
/// budget [`STATIC_PENETRATION_MAX_M`]).
pub fn static_penetration_m(p: &PlantParams, kind: ContactKind) -> f64 {
    if !matches!(kind, ContactKind::LowerRollers | ContactKind::Seat) {
        return 0.0;
    }
    let c = contact_params(p, kind);
    if c.is_inert() {
        return 0.0;
    }
    rated_share_n(&p.design) / c.k_n_per_m
}

/// Nominal travel limits [x_closed, x_closed + travel], m (PHYSICS §1, §9).
pub fn travel_limits_m(d: &DesignParams) -> [f64; 2] {
    let x0 = d.rail.x_closed_m;
    [x0, x0 + d.rail.travel_m.max(0.0)]
}

/// Bumper penetration at travel position `x_m`: (δ ≥ 0, direction of the bumper force
/// along x). The bumper faces sit one end-zone length beyond each travel limit; the
/// closed-end bumper pushes +x, the open-end one −x (PHYSICS §8, §9; SAF-014).
pub fn bumper_penetration(d: &DesignParams, x_m: f64) -> (f64, f64) {
    let [lo, hi] = travel_limits_m(d);
    let ez = d.damping.ez_len_m.max(0.0);
    let closed_face = lo - ez;
    let open_face = hi + ez;
    if x_m < closed_face {
        (closed_face - x_m, 1.0)
    } else if x_m > open_face {
        (x_m - open_face, -1.0)
    } else {
        (0.0, 0.0)
    }
}

/// Bumper force on the body along +x at position `x_m` and velocity `v_mps`
/// (Hunt–Crossley with `bumper_k_n_per_m` capped for the step and `bumper_hc_s_per_m`),
/// N; never pulls (PHYSICS §8).
pub fn bumper_force_n(p: &PlantParams, x_m: f64, v_mps: f64) -> f64 {
    let (delta, dir) = bumper_penetration(&p.design, x_m);
    if delta <= 0.0 {
        return 0.0;
    }
    let c = contact_params(p, ContactKind::Bumper);
    // δ̇ = −dir·v: moving deeper into either bumper increases δ.
    dir * hunt_crossley_n(c.k_n_per_m, c.c_n_s_per_m, delta, -dir * v_mps)
}

/// Clearance of the closed-end jamb face beyond the closed position x_closed, m (W2 lead
/// decision D3; design choice). DOOR, WALL and HEAVY close against a rigid jamb this far
/// past the closed stop: nominal termini (overshoot ≤ 0.02 mm, FAB-HAP-013…016) never
/// touch it, it lies inside the governor's observation point x_obs = terminus +
/// overtravel (5 mm in every preset, plan §4b), and floor-driven motion against it stays
/// inside the 10 mm SLP band (presets: `safety.x_min_m` = x_closed − 10 mm) for relative
/// impact speeds up to ≈ 2 m/s at the Travel1d stiffness cap (PHYSICS §8, §18).
pub const JAMB_CLEARANCE_M: f64 = 2.0e-3;

/// Design deflection of a rigid end stop (jamb, rail end stop behind the bumper) under the
/// rated weight pushed along x (a 1 g push), m (design choice: a steel frame stop; the
/// stiffness is max([`K_LOCK_N_PER_M`], m_rated·g₀/δ), capped at m·(0.2/h)² for the
/// body's truth mass m and the step h, which on the 1 ms tiers binds first; PHYSICS §8,
/// §24). An impact at relative speed v penetrates at most v·√(m/k): ≈ 5 mm per m/s on
/// Travel1d.
pub const END_STOP_DEFLECTION_M: f64 = 0.25e-3;

/// Physical end-stop geometry of the travel axis at the class geometry (W2 lead decision
/// D3; PHYSICS §8, §9).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EndStops {
    /// Rigid closed-end face, m: the jamb at x_closed − [`JAMB_CLEARANCE_M`], or the DRAWER
    /// carcass hard stop at x_closed − `seat_x_compliance` behind its push-stop spring.
    pub closed_m: f64,
    /// Start of the DRAWER push-stop spring (x_closed), m; `None` for jamb classes.
    pub push_stop_m: Option<f64>,
    /// Bumper face at the open end, x_closed + travel + ez_len, m.
    pub open_bumper_m: f64,
    /// Rigid open-end face, m: the bumper bottomed out at its rated stroke, x_closed +
    /// travel + ez_len + √(2·E_rated/k_b).
    pub open_m: f64,
}

/// Rated stroke of the end bumpers √(2·E_rated/k_b), m: the compression at which a
/// bumper has absorbed its rating (presets: 24 J in 20 mm DRAWER, 113 J in 30 mm DOOR,
/// 563 J in 50 mm WALL, 5625 J in 100 mm HEAVY; SAF-014). 0 without a bumper.
pub fn bumper_stroke_m(d: &DesignParams) -> f64 {
    let (k, e) = (d.damping.bumper_k_n_per_m, d.damping.bumper_e_rated_j);
    if k > 0.0 && e > 0.0 && k.is_finite() && e.is_finite() {
        fmath::sqrt(2.0 * e / k)
    } else {
        0.0
    }
}

/// True when the closed end is the DRAWER carcass (push-stop spring + hard stop) rather
/// than a jamb: a seat landing with a seat gap (plan §2b item 4).
fn carcass_closed_end(d: &DesignParams) -> bool {
    matches!(d.landing.kind, LandingKind::Seat) && d.suspension.g_seat_m > 0.0
}

/// The physical end stops of design `d` (W2 lead decision D3; PHYSICS §8, §9).
pub fn end_stops(d: &DesignParams) -> EndStops {
    let [lo, hi] = travel_limits_m(d);
    let (closed_m, push_stop_m) = if carcass_closed_end(d) {
        (lo - d.landing.seat_x_compliance_m.max(0.0), Some(lo))
    } else {
        (lo - JAMB_CLEARANCE_M, None)
    };
    let open_bumper_m = hi + d.damping.ez_len_m.max(0.0);
    EndStops {
        closed_m,
        push_stop_m,
        open_bumper_m,
        open_m: open_bumper_m + bumper_stroke_m(d),
    }
}

/// Rigid end-stop law for a body of truth mass `m_kg` at plant step `h_s`: (k, c) with
/// k = min(max(K_LOCK, m_rated·g₀/δ), m·(0.2/h)²) and penalty damping c = 2ζ_c·√(k·m),
/// where m is `m_kg` but never below the smallest truth mass (PHYSICS §8, §24: ω·h ≤ 0.2
/// for the body that hits the stop; [`END_STOP_DEFLECTION_M`]).
pub fn end_stop_law(p: &PlantParams, m_kg: f64, h_s: f64) -> (f64, f64) {
    let m = m_min_kg(p).max(if m_kg.is_finite() { m_kg } else { 0.0 });
    if !(h_s.is_finite() && h_s > 0.0 && m > 0.0) {
        return (0.0, 0.0);
    }
    let k_phys =
        K_LOCK_N_PER_M.max(p.design.carriage.m_rated_kg.max(0.0) * G0_MPS2 / END_STOP_DEFLECTION_M);
    let k = k_phys.min(k_max_n_per_m(m, h_s));
    (k, contact_damping_n_s_per_m(k, m))
}

/// The DRAWER push-stop and carcass hard stop for an inward compression `comp_m` = x_closed
/// − x > 0 at speed `v_mps`, N along +x (plan §2b item 4): the spring `seat_k_x` over
/// `seat_x_compliance`, then the steel hard stop capped for a body of mass `m_cap_kg` at
/// step `h_s` (the inward branch of [`seat_x_force_n`] with m_cap = m_min).
fn push_stop_n(p: &PlantParams, comp_m: f64, v_mps: f64, m_cap_kg: f64, h_s: f64) -> f64 {
    if comp_m <= 0.0 || !comp_m.is_finite() {
        return 0.0;
    }
    let ld = &p.design.landing;
    let m = m_min_kg(p).max(if m_cap_kg.is_finite() { m_cap_kg } else { 0.0 });
    let k = K_ROLLER_STEEL_N_PER_M.min(k_max_n_per_m(m, h_s));
    let c = contact_damping_n_s_per_m(k, m);
    let spring = ld.seat_k_x_n_per_m.max(0.0) * comp_m.min(ld.seat_x_compliance_m.max(0.0));
    let beyond = comp_m - ld.seat_x_compliance_m.max(0.0);
    spring + penalty_normal_n(k, c, beyond, -v_mps)
}

/// Closed-end stop force on a body of truth mass `m_kg` along +x (≥ 0, never pulls) at
/// travel position `x_m`, speed `v_mps` and plant step `h_s`, N (W2 lead decision D3;
/// PHYSICS §8): the rigid jamb at [`EndStops::closed_m`], or on the DRAWER the push-stop
/// spring from x_closed and the carcass hard stop (the law [`seat_x_force_n`] applies
/// while seated; do not add both).
pub fn closed_stop_force_n(p: &PlantParams, x_m: f64, v_mps: f64, m_kg: f64, h_s: f64) -> f64 {
    let d = &p.design;
    if !(x_m.is_finite() && v_mps.is_finite()) {
        return 0.0;
    }
    if carcass_closed_end(d) {
        return push_stop_n(p, d.rail.x_closed_m - x_m, v_mps, m_kg, h_s);
    }
    let (k, c) = end_stop_law(p, m_kg, h_s);
    penalty_normal_n(k, c, end_stops(d).closed_m - x_m, -v_mps)
}

/// Open-end stop force on a body of truth mass `m_kg` along +x (≤ 0, never pulls) at
/// `x_m`, `v_mps` and step `h_s`, N (W2 lead decision D3; PHYSICS §8): the rail's end stop
/// where the open-end bumper bottoms out ([`EndStops::open_m`]); it acts in parallel with
/// the bumper.
pub fn open_stop_force_n(p: &PlantParams, x_m: f64, v_mps: f64, m_kg: f64, h_s: f64) -> f64 {
    if !(x_m.is_finite() && v_mps.is_finite()) {
        return 0.0;
    }
    let (k, c) = end_stop_law(p, m_kg, h_s);
    -penalty_normal_n(k, c, x_m - end_stops(&p.design).open_m, v_mps)
}

/// Power dissipated by a penalty contact's damper while it pushes, c·δ̇² (≥ 0), W; 0 out
/// of contact (PHYSICS §8, §9: the ledger share of [`penalty_normal_n`]).
pub fn penalty_dissipation_w(
    k_n_per_m: f64,
    c_n_s_per_m: f64,
    delta_m: f64,
    delta_dot_mps: f64,
) -> f64 {
    if penalty_normal_n(k_n_per_m, c_n_s_per_m, delta_m, delta_dot_mps) <= 0.0 {
        return 0.0;
    }
    (c_n_s_per_m * delta_dot_mps * delta_dot_mps).max(0.0)
}

/// Bumper overload: the elastic energy ½·k·δ² at compression `delta_m` exceeds the
/// rating `bumper_e_rated_j` (physics perspective §1.6 "overload flag when compression >
/// rated"; SAF-014, SEI-015).
pub fn bumper_overload(p: &PlantParams, delta_m: f64) -> bool {
    let c = contact_params(p, ContactKind::Bumper);
    if c.is_inert() || delta_m <= 0.0 {
        return false;
    }
    0.5 * c.k_n_per_m * delta_m * delta_m > p.design.damping.bumper_e_rated_j
}

/// Bumper impact bookkeeping for the energy ledger (physics perspective §1.6: bumpers
/// log their energy). Carried by the tier across steps.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BumperLog {
    /// In contact at the end of the last step.
    pub in_contact: bool,
    /// Compression reached at or above the energy rating since reset.
    pub overloaded: bool,
    /// Deepest compression since reset, m.
    pub delta_max_m: f64,
}

impl BumperLog {
    /// Updates the log with the end-of-step bumper compression `delta_m`, the body speed
    /// `v_mps` and mass `m_kg`: an onset (out → in contact) counts one impact in
    /// `ledger.bumper_impacts` and records its impact energy ½·m·v² in
    /// `ledger.bumper_e_max_j` when largest. Returns true on an onset.
    pub fn update(
        &mut self,
        p: &PlantParams,
        ledger: &mut EnergyLedger,
        delta_m: f64,
        v_mps: f64,
        m_kg: f64,
    ) -> bool {
        let contact = delta_m > 0.0;
        let onset = contact && !self.in_contact;
        if onset {
            ledger.bumper_impacts = ledger.bumper_impacts.saturating_add(1);
            let e = 0.5 * m_kg.max(0.0) * v_mps * v_mps;
            if e > ledger.bumper_e_max_j {
                ledger.bumper_e_max_j = e;
            }
        }
        if contact {
            self.delta_max_m = self.delta_max_m.max(delta_m);
            self.overloaded |= bumper_overload(p, delta_m);
        }
        self.in_contact = contact;
        onset
    }
}

/// DRAWER seat x constraint while seated, force along +x at `x_rel_m` = x − x_closed
/// and velocity `v_mps`, N (plan §2b item 4; PHYSICS §8): pushing in (x_rel < 0)
/// compresses the push-stop spring `seat_k_x_n_per_m` over `seat_x_compliance_m` (so
/// push-to-open works while seated), then meets the hard stop; outward the V-seat
/// registration holds (stiff penalty) within the registration notch
/// [`SEAT_REG_NOTCH_M`]; beyond it the pins ride on the flat seat face, which exerts no
/// x force (W1.5: a drawer driven out of the slot that brushed the lower seat face at
/// x_rel = 7 mm met the full outward penalty as a 7 mm step penetration and was thrown
/// at 6 m/s). Both hard parts use the seat stiffness for the step `h_s`. Zero when the
/// class has no seat.
pub fn seat_x_force_n(p: &PlantParams, x_rel_m: f64, v_mps: f64, h_s: f64) -> f64 {
    let ld = &p.design.landing;
    if !matches!(ld.kind, LandingKind::Seat) || !x_rel_m.is_finite() {
        return 0.0;
    }
    if x_rel_m < 0.0 {
        push_stop_n(p, -x_rel_m, v_mps, m_min_kg(p), h_s)
    } else if x_rel_m <= SEAT_REG_NOTCH_M {
        let m = m_min_kg(p);
        let k = K_ROLLER_STEEL_N_PER_M.min(k_max_n_per_m(m, h_s));
        -penalty_normal_n(k, contact_damping_n_s_per_m(k, m), x_rel_m, v_mps)
    } else {
        0.0
    }
}

/// Depth of the V-seat registration notch along +x, m (design choice, W1.5: the
/// registration acts only while the seat pins sit in the notch at the closed stop).
pub const SEAT_REG_NOTCH_M: f64 = 0.5e-3;

/// Parking-lock force along +x: a bilateral spring-damper about the engagement position
/// `x_anchor_m` (spring-applied, power-to-release; physics perspective §1.6), N. Zero
/// when no lock is fitted.
pub fn lock_force_n(p: &PlantParams, x_m: f64, x_anchor_m: f64, v_mps: f64) -> f64 {
    let c = contact_params(p, ContactKind::ParkingLock);
    if c.is_inert() || !(x_m.is_finite() && x_anchor_m.is_finite() && v_mps.is_finite()) {
        return 0.0;
    }
    -c.k_n_per_m * (x_m - x_anchor_m) - c.c_n_s_per_m * v_mps
}

/// The seismic x-latch as an elasto-plastic element (PHYSICS §8; plan §4b): stiffness
/// [`K_LOCK_N_PER_M`] (capped) up to the retention force `x_retention_n`, beyond which it
/// slips (the scene then reports `drawer_opened`). Returns the bristle law and the
/// "normal" load to pass to `friction::bristle` (μ = 1, F_n = x_retention_n,
/// z_ss = F_ret/k), or `None` when no latch is fitted. Below retention it holds without
/// drift (no ratchet).
pub fn latch_bristle(p: &PlantParams) -> Option<(BristleParams, f64)> {
    let c = contact_params(p, ContactKind::SeismicLatch);
    let f_ret = p.design.landing.seismic_latch.x_retention_n;
    if c.is_inert() || !(f_ret.is_finite() && f_ret > 0.0) {
        return None;
    }
    let mut b = BristleParams::new(1.0, 1.0, f_ret / c.k_n_per_m);
    // Critically damped latch spring on the whole body: σ1·F_ret = 2ζ_b·√(k·m).
    b.sigma1_s_per_m =
        2.0 * crate::friction::ZETA_BRISTLE * fmath::sqrt(c.k_n_per_m * m_min_kg(p)) / f_ret;
    Some((b, f_ret))
}

/// Magnet force pressing the carriage onto its upper stop at gap `g_up` for mass `m_kg`:
/// n·F_pm(g_up) − m·g₀ (≥ 0), N (LEV-006: F/(m*·g) ≤ 1.30 for DOOR/WALL/HEAVY). Uses
/// the design closed form (σ_leak = 0, no saturation).
pub fn upper_stop_preload_n(d: &DesignParams, m_kg: f64) -> f64 {
    let s = &d.suspension;
    (n_lift(s) * pm_force(s, s.g_up_m, 0.0, 0.0) - m_kg * G0_MPS2).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::friction::bristle;
    use crate::integrator::rk4_step;
    use crate::params::{Environment, Mismatch};
    use fabrail_params::presets;
    use proptest::prelude::*;

    fn plant(class: fabrail_types::RailClass) -> PlantParams {
        let sp = presets::by_class(class);
        PlantParams::from_design(&sp.design, &Mismatch::default(), &Environment::default())
    }

    proptest! {
        #[test]
        fn never_pulls(
            k in 0.0f64..1e9,
            c in 0.0f64..1e5,
            delta in -1e-2f64..1e-2,
            ddot in -10.0f64..10.0,
        ) {
            // The penalty and Hunt–Crossley laws are unilateral: F_n ≥ 0 always and 0
            // out of contact (PHYSICS §8).
            let f = penalty_normal_n(k, c, delta, ddot);
            prop_assert!(f >= 0.0, "penalty pulled: {f}");
            let hc = hunt_crossley_n(k, c * 1e-5, delta, ddot);
            prop_assert!(hc >= 0.0, "Hunt–Crossley pulled: {hc}");
            if delta <= 0.0 {
                prop_assert_eq!(f, 0.0);
                prop_assert_eq!(hc, 0.0);
            }
            prop_assert!(hunt_crossley_dissipation_w(k, c * 1e-5, delta, ddot) >= 0.0);
        }
    }

    #[test]
    fn normal_laws_and_bound_closed_forms() {
        assert_eq!(penalty_normal_n(1e6, 100.0, -1e-6, 0.0), 0.0);
        assert_eq!(
            penalty_normal_n(1e6, 100.0, 1e-6, -1.0),
            0.0,
            "fast separation clips to 0"
        );
        assert!((penalty_normal_n(1e6, 100.0, 1e-6, 0.001) - 1.1).abs() < 1e-12);
        assert_eq!(hunt_crossley_n(1e6, 0.5, 1e-3, -3.0), 0.0);
        assert!((hunt_crossley_n(1e6, 0.5, 1e-3, 1.0) - 1500.0).abs() < 1e-9);
        assert_eq!(penalty_normal_n(1e6, 0.0, f64::NAN, 0.0), 0.0);
        // 1 kg at 50 µs: (0.2 / 5e-5)² = 1.6e7 N/m.
        assert!((k_max_n_per_m(1.0, 5e-5) - 1.6e7).abs() < 1e-3);
    }

    #[test]
    fn drop_onto_rollers_never_pulls_and_settles_within_budget() {
        // A DRAWER support point dropped from 20 µm above its roller: the contact force
        // is never negative through bounce and settle, and the rest penetration equals
        // the static m_eff·g/k within (I) 1e-3 relative (PHYSICS §8, §24).
        let p = plant(fabrail_types::RailClass::Drawer);
        let c = contact_params(&p, ContactKind::LowerRollers);
        let m = m_eff_vertical_kg(&p);
        let h = BASE_TICK_S;
        let mut y = [20.0e-6, 0.0]; // height above contact (δ = −height), velocity up
        let mut f_min = f64::INFINITY;
        for k in 0..20_000u32 {
            y = rk4_step(&y, f64::from(k) * h, h, |_, _, s| {
                let f = penalty_normal_n(c.k_n_per_m, c.c_n_s_per_m, -s[0], -s[1]);
                [s[1], f / m - G0_MPS2]
            });
            f_min = f_min.min(penalty_normal_n(c.k_n_per_m, c.c_n_s_per_m, -y[0], -y[1]));
        }
        assert!(f_min >= 0.0);
        let want = m * G0_MPS2 / c.k_n_per_m;
        assert!(
            ((-y[0]) - want).abs() < 1e-3 * want,
            "rest penetration {} vs {want} ((I) rel 1e-3, PHYSICS §8)",
            -y[0]
        );
    }

    #[test]
    fn stiffness_caps_and_static_penetration_budget() {
        // Every fitted contact of every preset satisfies k ≤ m_eff·(0.2/h)² at its
        // default step, and the weight-bearing contacts stay within 10 µm at the rated
        // load (physics perspective §1.6; PHYSICS §8, §24).
        for &class in &presets::CLASSES {
            let p = plant(class);
            for &kind in ContactKind::ALL {
                let c = contact_params(&p, kind);
                assert!(c.k_n_per_m.is_finite() && c.k_n_per_m >= 0.0);
                assert!(c.c_n_s_per_m.is_finite() && c.c_n_s_per_m >= 0.0);
                let m_eff = match kind {
                    ContactKind::UpperStop | ContactKind::LowerRollers | ContactKind::Seat => {
                        m_eff_vertical_kg(&p)
                    }
                    ContactKind::GuideFin | ContactKind::SidePads => m_eff_lateral_kg(&p),
                    _ => m_min_kg(&p),
                };
                let k_max = k_max_n_per_m(m_eff, default_step_s(kind));
                assert!(
                    c.k_n_per_m <= k_max * (1.0 + 1e-12),
                    "{class:?} {}: k {} > {k_max}",
                    kind.name(),
                    c.k_n_per_m
                );
            }
            for kind in [ContactKind::LowerRollers, ContactKind::Seat] {
                let pen = static_penetration_m(&p, kind);
                assert!(
                    pen <= STATIC_PENETRATION_MAX_M,
                    "{class:?} {}: {pen} m > 10 µm (physics perspective §1.6)",
                    kind.name()
                );
            }
        }
        // Fitted-ness follows the presets: only the DRAWER has a seat, only the top-hung
        // and HEAVY classes a guide fin, only WALL/HEAVY a parking lock.
        let dr = plant(fabrail_types::RailClass::Drawer);
        let door = plant(fabrail_types::RailClass::Door);
        let heavy = plant(fabrail_types::RailClass::Heavy);
        assert!(!contact_params(&dr, ContactKind::Seat).is_inert());
        assert!(contact_params(&door, ContactKind::Seat).is_inert());
        assert!(contact_params(&dr, ContactKind::GuideFin).is_inert());
        assert!(!contact_params(&door, ContactKind::GuideFin).is_inert());
        assert!(contact_params(&door, ContactKind::ParkingLock).is_inert());
        assert!(!contact_params(&heavy, ContactKind::ParkingLock).is_inert());
        assert!(contact_params(&dr, ContactKind::SeismicLatch).is_inert());
        // Tier presliding: 10 µm at 50 µs, 50 µm at 1 ms (core design §C.5).
        let r1 = contact_params_at(&dr, ContactKind::LowerRollers, 1.0e-3);
        assert_eq!(r1.bristle.z_ss_m, Z_SS_TRAVEL1D_M);
        assert_eq!(
            contact_params(&dr, ContactKind::LowerRollers)
                .bristle
                .z_ss_m,
            Z_SS_CARRIAGE6_M
        );
        assert!(
            (r1.bristle.mu_c - 0.003).abs() < 1e-15,
            "roller μ 0.003 (plan §2b item 19)"
        );
    }

    /// Mass on rollers under tangential force `f(t)` with the elasto-plastic bristle,
    /// split-integrated (x, v by RK4 with z frozen; z by the exact update). Returns the
    /// position history sampled every `every` steps.
    fn slide_run(
        b: &BristleParams,
        m: f64,
        f_n: f64,
        h: f64,
        n: u32,
        every: u32,
        f: impl Fn(f64) -> f64,
    ) -> Vec<f64> {
        let mut y = [0.0f64, 0.0f64];
        let mut z = 0.0f64;
        let mut out = Vec::new();
        for k in 0..n {
            let t = f64::from(k) * h;
            let x0 = y[0];
            let zz = z;
            y = rk4_step(&y, t, h, |_, ts, s| {
                let ft = bristle(b, zz, s[1], f_n).f_t_n;
                [s[1], (f(ts) + ft) / m]
            });
            let dx = y[0] - x0;
            z = crate::friction::bristle_advance(b, z, dx, dx / h);
            if k % every == 0 {
                out.push(y[0]);
            }
        }
        out
    }

    #[test]
    fn bristle_no_ratchet() {
        // 60 s of sub-breakaway vibration with a bias on a landed 100 kg body on its
        // rollers (μ 0.003, Travel1d presliding 50 µm, 1 ms): the elasto-plastic bristle
        // stays elastic and the body does not ratchet: drift < 0.05 mm per 60 s (core
        // design §E.2), whereas a tanh-regularised Coulomb law (v_s 1 mm/s) creeps
        // millimetres under the same load (core design §A.1; physics perspective §0
        // item 4: 5.5 mm per 10 s at half breakaway).
        let m = 100.0;
        let f_n = m * G0_MPS2;
        let b = BristleParams::new(0.003, 0.003, Z_SS_TRAVEL1D_M);
        let f_ba = b.breakaway_ratio() * f_n; // 0.7·μ·F_n ≈ 2.06 N
        let force =
            |t: f64| 0.3 * f_ba + 0.5 * f_ba * fmath::sin(2.0 * core::f64::consts::PI * 2.0 * t);
        let h = 1.0e-3;
        let xs = slide_run(&b, m, f_n, h, 60_000, 1000, force);
        let drift = (xs[xs.len() - 1] - xs[0]).abs();
        let cycle_drift = (xs[xs.len() - 1] - xs[10]).abs();
        assert!(
            drift < 0.05e-3,
            "drift {drift} m over 60 s (< 0.05 mm, core design §E.2)"
        );
        assert!(
            cycle_drift < 1e-9,
            "cycle-to-cycle drift {cycle_drift} m (elastic regime exact)"
        );
        // The same load on a tanh-Coulomb law creeps (the reason for the bristle model).
        let mut y = [0.0f64, 0.0f64];
        let mu_fn = 0.003 * f_n;
        for k in 0..60_000u32 {
            y = rk4_step(&y, f64::from(k) * h, h, |_, t, s| {
                [s[1], (force(t) - mu_fn * fmath::tanh(s[1] / 1.0e-3)) / m]
            });
        }
        assert!(
            y[0].abs() > 1.0e-3,
            "tanh Coulomb creep {} m should exceed 1 mm",
            y[0]
        );
    }

    #[test]
    fn floor_vibration_below_breakaway_does_not_walk_a_landed_body() {
        // Floor-borne version: a_floor = 0.8·a_ba·sin(2π·3 t) with a 0.3·a_ba bias, as the
        // inertial load −m·a_floor in the rail frame (PHYSICS §5); drift < 0.05 mm.
        let m = 26.0;
        let f_n = m * G0_MPS2;
        let b = BristleParams::new(0.003, 0.003, Z_SS_TRAVEL1D_M);
        let a_ba = b.breakaway_ratio() * G0_MPS2;
        let xs = slide_run(&b, m, f_n, 1.0e-3, 60_000, 1000, |t| {
            -m * (0.3 * a_ba + 0.6 * a_ba * fmath::sin(2.0 * core::f64::consts::PI * 3.0 * t))
        });
        let drift = (xs[xs.len() - 1] - xs[0]).abs();
        assert!(
            drift < 0.05e-3,
            "drift {drift} m (< 0.05 mm per 60 s, core design §E.2)"
        );
    }

    #[test]
    fn sliding_force() {
        // Steady sliding at 0.1 m/s under a constant velocity: the tangential force
        // settles at −μ_c·F_n within 1 % (core design §E.2 (1 %)), on the split update
        // and on an RK4-embedded bristle at a stable step, and a pushed body slides at
        // the drag-limited balance.
        for &(mu, zss) in &[
            (0.003, Z_SS_CARRIAGE6_M),
            (0.15, Z_SS_CARRIAGE6_M),
            (0.15, Z_SS_TRAVEL1D_M),
        ] {
            let b = BristleParams::new(mu, mu, zss);
            let f_n = 250.0;
            let v = 0.1;
            // Exact update over 5 mm of slide.
            let mut z = 0.0;
            for _ in 0..1000 {
                z = crate::friction::bristle_advance(&b, z, v * 5.0e-5, v);
            }
            let f = bristle(&b, z, v, f_n).f_t_n;
            assert!(
                (f + mu * f_n).abs() <= 0.01 * mu * f_n,
                "μ {mu}: F {f} vs {} (1 %, core design §E.2)",
                -mu * f_n
            );
            // RK4-embedded at a step with relax_rate·h = 0.5.
            let hh = 0.5 / crate::friction::relax_rate_per_s(&b, v);
            let mut zr = 0.0;
            for k in 0..4000u32 {
                zr = rk4_step(&zr, f64::from(k) * hh, hh, |_, _, zz| {
                    bristle(&b, *zz, v, f_n).z_dot_mps
                });
            }
            let fr = bristle(&b, zr, v, f_n).f_t_n;
            assert!(
                (fr + mu * f_n).abs() <= 0.01 * mu * f_n,
                "RK4 μ {mu}: F {fr}"
            );
        }
        // Dynamic: a 10 kg block on UHMW pushed by 3·μ·F_n accelerates at 2·μ·g within
        // 1 % once sliding (Coulomb kinetics).
        let m = 10.0;
        let b = BristleParams::new(0.15, 0.15, Z_SS_CARRIAGE6_M);
        let f_n = m * G0_MPS2;
        let xs = slide_run(&b, m, f_n, BASE_TICK_S, 40_000, 1, |_| 3.0 * 0.15 * f_n);
        let n = xs.len();
        let tau = 1000.0 * BASE_TICK_S;
        let a = (xs[n - 1] - 2.0 * xs[n - 1001] + xs[n - 2001]) / (tau * tau);
        let want = 2.0 * 0.15 * G0_MPS2;
        assert!((a - want).abs() <= 0.01 * want, "a {a} vs {want} (1 %)");
    }

    #[test]
    fn bumper_geometry_force_and_log() {
        let p = plant(fabrail_types::RailClass::Door);
        let [lo, hi] = travel_limits_m(&p.design);
        assert_eq!((lo, hi), (0.0, 1.0));
        // Inside the travel and the end zones: no bumper.
        assert_eq!(bumper_penetration(&p.design, 1.1), (0.0, 0.0));
        // 1 cm past the open-end bumper face (1.0 + 0.15 m): pushes −x.
        let (d, dir) = bumper_penetration(&p.design, 1.16);
        assert!((d - 0.01).abs() < 1e-12 && dir == -1.0);
        let f = bumper_force_n(&p, 1.16, 0.0);
        assert!((f + 250.0e3 * 0.01).abs() < 1e-6, "static bumper force k·δ");
        assert!(
            bumper_force_n(&p, -0.16, 0.0) > 0.0,
            "closed-end bumper pushes +x"
        );
        assert!(
            bumper_force_n(&p, 1.16, -5.0) <= 0.0,
            "never pulls on rebound"
        );
        // Rating: ½·k·δ² > 113 J at δ > 30.07 mm.
        assert!(!bumper_overload(&p, 0.030) && bumper_overload(&p, 0.031));
        let mut log = BumperLog::default();
        let mut ledger = EnergyLedger::default();
        assert!(log.update(&p, &mut ledger, 1e-3, 0.5, 100.0));
        assert!(
            !log.update(&p, &mut ledger, 2e-3, 0.3, 100.0),
            "still in contact"
        );
        log.update(&p, &mut ledger, 0.0, -0.2, 100.0);
        log.update(&p, &mut ledger, 1e-4, 0.1, 100.0);
        assert_eq!(ledger.bumper_impacts, 2);
        assert!((ledger.bumper_e_max_j - 12.5).abs() < 1e-12, "½·100·0.5²");
    }

    #[test]
    fn seat_lock_and_latch_laws() {
        let dr = plant(fabrail_types::RailClass::Drawer);
        // Push-to-open: 2 mm in compresses the 2000 N/m push stop → +4 N (outward).
        let f = seat_x_force_n(&dr, -2.0e-3, 0.0, BASE_TICK_S);
        assert!((f - 4.0).abs() < 1e-12, "push stop {f}");
        // Past the 2.5 mm compliance the hard stop adds stiffly; outward the seat holds.
        assert!(seat_x_force_n(&dr, -2.6e-3, 0.0, BASE_TICK_S) > 100.0);
        assert!(seat_x_force_n(&dr, 1.0e-5, 0.0, BASE_TICK_S) < -10.0);
        assert_eq!(
            seat_x_force_n(&plant(fabrail_types::RailClass::Door), -1e-3, 0.0, 5e-5),
            0.0
        );
        // Parking lock: bilateral spring on HEAVY, nothing on DOOR.
        let hv = plant(fabrail_types::RailClass::Heavy);
        assert!((lock_force_n(&hv, 1.0e-3, 0.0, 0.0) + 1.0e3).abs() < 1e-9);
        assert_eq!(
            lock_force_n(&plant(fabrail_types::RailClass::Door), 1e-3, 0.0, 0.0),
            0.0
        );
        // Seismic latch as an elasto-plastic element that slips at retention.
        let mut d2 = dr.clone();
        d2.design.landing.seismic_latch.enabled = true;
        d2.design.landing.seismic_latch.x_retention_n = 200.0;
        assert!(latch_bristle(&dr).is_none());
        let (b, f_ret) = latch_bristle(&d2).expect("latch fitted");
        let z = crate::friction::bristle_advance(&b, 0.0, 0.05, 0.1);
        let fs = bristle(&b, z, 0.1, f_ret).f_t_n;
        assert!((fs + 200.0).abs() < 1e-6, "slips at retention: {fs}");
        // Upper-stop preload (LEV-006 form): DOOR F/(m*·g) at g_up ≈ 1.278.
        let door = plant(fabrail_types::RailClass::Door);
        let pre = upper_stop_preload_n(&door.design, 0.0) / (100.0 * G0_MPS2);
        assert!(
            (pre - 1.278).abs() < 0.01,
            "DOOR stop ratio {pre} (core design §C.2 1.278)"
        );
    }

    #[test]
    fn end_stop_geometry_and_laws() {
        // W2 lead decision D3 (PHYSICS §8, §9): the closed end is a rigid jamb 5 mm past
        // x_closed (DOOR/WALL/HEAVY) or the DRAWER carcass (push-stop spring from x_closed,
        // hard stop at −seat_x_compliance); the open end is the rail stop where the bumper
        // bottoms out at its rated stroke √(2·E/k) (preset comments: 20/30/50/100 mm). Both
        // closed faces lie inside the 10 mm SLP band (safety.x_min_m).
        let strokes = [
            (fabrail_types::RailClass::Drawer, 0.020),
            (fabrail_types::RailClass::Door, 0.030),
            (fabrail_types::RailClass::Wall, 0.050),
            (fabrail_types::RailClass::Heavy, 0.100),
        ];
        for (class, stroke) in strokes {
            let sp = presets::by_class(class);
            let p = plant(class);
            let d = &p.design;
            let st = end_stops(d);
            let m = p.m_body_kg;
            assert!(
                (bumper_stroke_m(d) - stroke).abs() < 1.0e-3,
                "{class:?}: bumper stroke {} ≈ {stroke} m (±1 mm, preset comments)",
                bumper_stroke_m(d)
            );
            let [lo, hi] = travel_limits_m(d);
            assert!(
                st.closed_m < lo && st.closed_m > sp.safety.x_min_m,
                "{class:?}: closed stop {} inside (x_min {}, x_closed {lo}) (D3)",
                st.closed_m,
                sp.safety.x_min_m
            );
            assert!(
                (st.open_bumper_m - (hi + d.damping.ez_len_m)).abs() < 1e-12
                    && (st.open_m - st.open_bumper_m - bumper_stroke_m(d)).abs() < 1e-12,
                "{class:?}: open stop at the bumper face + rated stroke"
            );
            // The bumper face used by bumper_penetration is the same face.
            assert!(
                (bumper_penetration(d, st.open_bumper_m + 1e-3).0 - 1e-3).abs() < 1e-12
                    && bumper_penetration(d, st.open_bumper_m - 1e-3).0 == 0.0,
                "{class:?}: one open-end bumper face for bumpers and stops"
            );
            for h in [BASE_TICK_S, 1.0e-3] {
                // Free between the stops, pushes back beyond them, never pulls.
                assert_eq!(closed_stop_force_n(&p, lo + 0.01, -1.0, m, h), 0.0);
                assert_eq!(open_stop_force_n(&p, st.open_m - 1e-6, 1.0, m, h), 0.0);
                let fc = closed_stop_force_n(&p, st.closed_m - 1e-4, 0.0, m, h);
                let fo = open_stop_force_n(&p, st.open_m + 1e-4, 0.0, m, h);
                assert!(
                    fc > 0.0 && fo < 0.0,
                    "{class:?} h {h}: stops push back ({fc}, {fo})"
                );
                // Never pulls: the rigid part releases (the DRAWER push-stop spring keeps
                // pushing out, ≤ seat_k_x·seat_x_compliance).
                let fr = closed_stop_force_n(&p, st.closed_m - 1e-4, 50.0, m, h);
                let spring = d.landing.seat_k_x_n_per_m * d.landing.seat_x_compliance_m;
                assert!(
                    fr >= 0.0
                        && fr
                            <= if st.push_stop_m.is_some() {
                                spring
                            } else {
                                0.0
                            },
                    "{class:?}: never pulls ({fr})"
                );
                assert_eq!(
                    open_stop_force_n(&p, st.open_m + 1e-4, -50.0, m, h),
                    0.0,
                    "never pulls"
                );
                // Stable on every tier: ω·h ≤ 0.2 at the smallest truth mass (PHYSICS §24).
                for mm in [m_min_kg(&p), p.m_body_kg, d.carriage.m_rated_kg] {
                    let (k, _) = end_stop_law(&p, mm, h);
                    assert!(
                        k * h * h <= 0.04 * mm * (1.0 + 1e-12),
                        "{class:?}: ω·h ≤ 0.2"
                    );
                }
            }
            match st.push_stop_m {
                Some(x0) => {
                    // DRAWER: the unseated carcass stop is the seated push-stop law.
                    assert_eq!(x0, lo);
                    for comp in [1e-4, 1e-3, 2.4e-3, 2.6e-3, 4e-3] {
                        let a =
                            closed_stop_force_n(&p, lo - comp, -0.01, m_min_kg(&p), BASE_TICK_S);
                        let b = seat_x_force_n(&p, -comp, -0.01, BASE_TICK_S);
                        assert!(
                            (a - b).abs() <= 1e-9 * b.abs().max(1.0),
                            "carcass = seat law"
                        );
                    }
                }
                None => {
                    assert!((lo - st.closed_m - JAMB_CLEARANCE_M).abs() < 1e-12);
                    assert_eq!(
                        closed_stop_force_n(&p, lo - 1e-3, -0.1, m, BASE_TICK_S),
                        0.0
                    );
                }
            }
        }
    }
}
