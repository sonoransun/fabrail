//! StabilityReport: max |lambda*h| per element; runs refuse above 0.2 on oscillatory elements (PHYSICS §24). Owner: A6a.
//!
//! Penalty contacts must satisfy k_c ≤ m_eff·(0.2/h)² (PHYSICS §8), i.e. ω·h ≤ 0.2 for
//! every oscillatory element, which keeps RK4's per-step phase and amplitude error far
//! below the anchors' tolerances (the RK4 stability limit on the imaginary axis is
//! 2√2). `fabrail_sim::run` refuses a scenario whose report is not [`StabilityReport::ok`].
//!
//! As implemented (PHYSICS §24; physics perspective §1.14): the tiers cap the penalty
//! contacts they integrate (upper stop, rollers, seat, bumpers, lock, latch) and the
//! bristle stiffness at use, so those entries report the capped values (≤ 0.2 by
//! construction). The guide fin, side pads and the closing-edge seal act through
//! `guidance::guide` and `interaction::obstacle_force_n` and are reported uncapped, as are
//! the closed levitation loop, the top-hung sway pendulum and the sealed DRAWER air
//! spring: these can refuse a run. First-order lags (current loops, eddy lag) are real
//! (non-oscillatory) entries; a lag with τ < 2h is integrated algebraically (flagged by
//! |λh| > 0.5, never refused). Effective masses use the lightest body
//! (min(m_body, m_op_min)); vertical contacts use its per-lift-element share.

use std::collections::BTreeMap;

use fabrail_params::derive::{self, G0_MPS2};
use fabrail_params::presets;
use fabrail_types::fmath;

use crate::air::{GAMMA_AIR, P_ATM_PA};
use crate::body::{Common, capped, capped_bristle, kind_index};
use crate::contact::ContactKind;
use crate::interaction::{ObstacleBody, series_k};
use crate::lsm;
use crate::magnet::{ElementGeom, T_REF_C};
use crate::model::Tier;
use crate::params::PlantParams;

/// Largest admissible |λ·h| on an oscillatory element (core design §B.6; PHYSICS §24).
pub const LAMBDA_H_MAX: f64 = 0.2;

/// One element's step-size figure.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StabilityEntry {
    /// Element name, e.g. "contact.upper_stop", "coil\[0\]", "bristle.rollers".
    pub element: String,
    /// |λ·h| of the element's fastest eigenvalue.
    pub lambda_h: f64,
    /// The eigenvalue is complex (oscillatory element); only these are refused.
    pub oscillatory: bool,
}

/// Per-element |λ·h| at the plant step `h_s`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StabilityReport {
    /// Plant step, s.
    pub h_s: f64,
    /// One entry per element.
    pub entries: Vec<StabilityEntry>,
}

impl StabilityReport {
    /// Largest |λ·h| over all entries (0 when empty).
    pub fn max_lambda_h(&self) -> f64 {
        self.entries.iter().map(|e| e.lambda_h).fold(0.0, f64::max)
    }

    /// The worst oscillatory entry, if any.
    pub fn worst_oscillatory(&self) -> Option<&StabilityEntry> {
        self.entries
            .iter()
            .filter(|e| e.oscillatory)
            .max_by(|a, b| a.lambda_h.total_cmp(&b.lambda_h))
    }

    /// True when no oscillatory element exceeds [`LAMBDA_H_MAX`] (NaN counts as a
    /// violation).
    pub fn ok(&self) -> bool {
        self.entries
            .iter()
            .filter(|e| e.oscillatory)
            .all(|e| e.lambda_h <= LAMBDA_H_MAX)
    }

    /// Element → |λ·h| (the Python `stability_report` dict).
    pub fn to_map(&self) -> BTreeMap<String, f64> {
        self.entries
            .iter()
            .map(|e| (e.element.clone(), e.lambda_h))
            .collect()
    }
}

/// Name of a contact kind in the report.
fn contact_name(k: ContactKind) -> &'static str {
    match k {
        ContactKind::UpperStop => "contact.upper_stop",
        ContactKind::LowerRollers => "contact.lower_rollers",
        ContactKind::Seat => "contact.seat",
        ContactKind::GuideFin => "contact.guide_fin",
        ContactKind::SidePads => "contact.side_pads",
        ContactKind::Bumper => "contact.bumper",
        ContactKind::ParkingLock => "contact.parking_lock",
        ContactKind::SeismicLatch => "contact.seismic_latch",
        ContactKind::EdgeSeal => "contact.edge_seal",
    }
}

/// |λ|max·h and the oscillatory flag of m·ẍ + c·ẋ + k·x = 0.
fn second_order(k: f64, c: f64, m: f64, h: f64) -> (f64, bool) {
    if m <= 0.0 || k < 0.0 {
        return (0.0, false);
    }
    let disc = c * c - 4.0 * k * m;
    if disc < 0.0 {
        (fmath::sqrt(k / m) * h, true)
    } else {
        ((c + fmath::sqrt(disc)) / (2.0 * m) * h, false)
    }
}

/// Stability report for the plant step `h` (core design §B.6; PHYSICS §24). The tier is
/// inferred from the step: h ≥ 1 ms is a 1 ms tier (Travel1d / Structure), shorter steps
/// are Carriage6 (see [`report_tier`]).
pub fn report(p: &PlantParams, h: f64) -> StabilityReport {
    let tier = if h >= Tier::Travel1d.nominal_h_s() {
        Tier::Travel1d
    } else {
        Tier::Carriage6
    };
    report_tier(p, tier, h)
}

/// Stability report of `tier` at step `h` (PHYSICS §24). The 1 ms tiers (Travel1d,
/// Structure) have no heave, coil, eddy, vertical-contact or lateral dynamics (levitation
/// is ideal and y ≡ 0), so only their x-axis elements are reported.
pub fn report_tier(p: &PlantParams, tier: Tier, h: f64) -> StabilityReport {
    let full = matches!(tier, Tier::Carriage6 | Tier::Heave1);
    let d = &p.design;
    let com = Common::new(p, h);
    let n_l = com.n_lift.max(1) as f64;
    let m_op = d.carriage.m_op_min_kg;
    let m_min = if m_op > 0.0 {
        p.m_body_kg.min(m_op)
    } else {
        p.m_body_kg
    };
    let mut e: Vec<StabilityEntry> = Vec::new();
    let mut push = |name: &str, lambda_h: f64, oscillatory: bool| {
        e.push(StabilityEntry {
            element: name.to_string(),
            lambda_h,
            oscillatory,
        });
    };
    push("lsm.current", h / lsm::TAU_CURRENT_S, false);
    if full {
        heave_entries(p, &com, h, &mut push);
    }
    // Penalty contacts (PHYSICS §8).
    for &kind in ContactKind::ALL {
        let vertical = matches!(
            kind,
            ContactKind::UpperStop | ContactKind::LowerRollers | ContactKind::Seat
        );
        let lateral = matches!(kind, ContactKind::GuideFin | ContactKind::SidePads);
        if !full && (vertical || lateral) {
            continue;
        }
        let cp = &com.cp[kind_index(kind)];
        let m_eff = if vertical { m_min / n_l } else { m_min };
        let (k, c) = match kind {
            ContactKind::UpperStop
            | ContactKind::LowerRollers
            | ContactKind::Seat
            | ContactKind::ParkingLock
            | ContactKind::SeismicLatch => capped(cp, m_eff, h),
            // Hunt–Crossley: stiffness capped; the damping scales with δ (not a linear c).
            ContactKind::Bumper => (capped(cp, m_eff, h).0, 0.0),
            ContactKind::GuideFin | ContactKind::SidePads | ContactKind::EdgeSeal => {
                (cp.k_n_per_m.max(0.0), cp.c_n_s_per_m.max(0.0))
            }
        };
        let (lh, osc) = second_order(k, c, m_eff, h);
        // A capped stiffness sits exactly on the limit; report the limit itself.
        let lh = if osc && lh > LAMBDA_H_MAX && lh <= LAMBDA_H_MAX * (1.0 + 1e-9) {
            LAMBDA_H_MAX
        } else {
            lh
        };
        push(contact_name(kind), lh, osc);
    }
    // Bristle presliding stiffness F_n·μ_c/z_ss with F_n ≤ 2·m·g₀ (capped; PHYSICS §8).
    for (kind, name) in [
        (ContactKind::LowerRollers, "bristle.lower_rollers"),
        (ContactKind::UpperStop, "bristle.upper_stop"),
        (ContactKind::Seat, "bristle.seat"),
    ] {
        let bp = capped_bristle(&com.cp[kind_index(kind)].bristle, h);
        let w = if bp.z_ss_m > 0.0 && bp.mu_c > 0.0 {
            fmath::sqrt(2.0 * G0_MPS2 * bp.mu_c / bp.z_ss_m)
        } else {
            0.0
        };
        push(name, (w * h).min(LAMBDA_H_MAX), true);
    }
    // Sealed DRAWER air spring k = γ·P₀·A²/V (PHYSICS §11; 17 kN/m at the defaults).
    let a = &d.air;
    if a.cavity_area_m2 > 0.0 && a.cavity_volume_m3 > 0.0 {
        let k = GAMMA_AIR * P_ATM_PA * a.cavity_area_m2 * a.cavity_area_m2 / a.cavity_volume_m3;
        push("air.cavity", second_order(k, 0.0, m_min, h).0, true);
    }
    // ISO/TS 15066 bodies through the closing-edge seal (PHYSICS §22).
    for &b in ObstacleBody::ALL {
        let k = series_k(b.k_n_per_m(), d.damping.k_edge_n_per_m);
        let m = 1.0 / (1.0 / m_min + 1.0 / b.m_eff_kg());
        let name = match b {
            ObstacleBody::Chest => "obstacle.chest",
            ObstacleBody::Hand => "obstacle.hand",
            ObstacleBody::Face => "obstacle.face",
            ObstacleBody::Leg => "obstacle.leg",
        };
        push(name, second_order(k, 0.0, m, h).0, true);
    }
    StabilityReport { h_s: h, entries: e }
}

/// Heave-side entries of the 50 µs tiers: levitation poles, coil and eddy lags, sway.
fn heave_entries(p: &PlantParams, com: &Common, h: f64, push: &mut impl FnMut(&str, f64, bool)) {
    let d = &p.design;
    let s = &d.suspension;
    // Levitation (PHYSICS §3, §15): the open-loop pole is real; the designed closed loop
    // (ω = omega_ratio·p, ζ 0.7, class preset ratio) is oscillatory.
    let hp = derive::h_prime(s.h_m_m, s.mu_rec);
    let pole = derive::open_loop_pole(s.g_star_m, hp);
    push("levitation.open_loop", pole * h, false);
    let ratio = presets::by_class(d.class).control.levitation.omega_ratio;
    push("levitation.closed_loop", ratio * pole * h, true);
    // First-order lags (PHYSICS §4, §7, §2).
    if p.tau_cl_s > 0.0 {
        push("coil.lag", h / p.tau_cl_s, false);
    }
    let model = com.lift_model();
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
    let l_inc = model.dpsi_di_h(&ev);
    if l_inc > 0.0 {
        push("coil.voltage", h * s.r_coil_ohm / l_inc, false);
    }
    if s.tau_eddy_s > 0.0 {
        push("eddy_lag", h / s.tau_eddy_s, false);
    }
    // Top-hung sway pendulum √(g₀/ℓ) (PHYSICS §6; DOOR 0.49 Hz).
    let c = &d.carriage;
    if c.top_hung && c.cg_m[2] < 0.0 {
        push("sway.pendulum", fmath::sqrt(G0_MPS2 / -c.cg_m[2]) * h, true);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verdict_only_refuses_oscillatory_elements() {
        let mut r = StabilityReport {
            h_s: 5e-5,
            entries: vec![
                StabilityEntry {
                    element: "coil[0]".into(),
                    lambda_h: 0.9,
                    oscillatory: false,
                },
                StabilityEntry {
                    element: "contact.upper_stop".into(),
                    lambda_h: 0.15,
                    oscillatory: true,
                },
            ],
        };
        assert!(r.ok());
        assert_eq!(r.max_lambda_h(), 0.9);
        r.entries[1].lambda_h = 0.25;
        assert!(!r.ok());
        assert_eq!(
            r.worst_oscillatory().map(|e| e.element.as_str()),
            Some("contact.upper_stop")
        );
        r.entries[1].lambda_h = f64::NAN;
        assert!(!r.ok(), "NaN is a violation");
        assert_eq!(r.to_map().len(), 2);
    }

    fn plant(sp: &fabrail_params::SystemParams) -> PlantParams {
        use crate::params::{Environment, Mismatch};
        PlantParams::from_design(&sp.design, &Mismatch::default(), &Environment::default())
    }

    #[test]
    fn presets_pass_at_their_tier_steps() {
        // Every preset at 50 µs (Carriage6/Heave1) and 1 ms (Travel1d) passes the ωh ≤ 0.2
        // rule on every oscillatory element (PHYSICS §24).
        for sp in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let p = plant(&sp);
            for h in [5.0e-5, 1.0e-3] {
                let r = report(&p, h);
                assert!(
                    r.ok(),
                    "{:?} at h {h}: worst {:?} (ωh ≤ 0.2, PHYSICS §24)",
                    sp.design.class,
                    r.worst_oscillatory()
                );
                // The 1 ms tiers carry no heave dynamics (ideal levitation).
                assert_eq!(
                    r.to_map().contains_key("levitation.closed_loop"),
                    h < 1.0e-3
                );
                assert!(r.entries.iter().all(|e| e.lambda_h.is_finite()));
            }
            // The DOOR closed loop at 50 µs: 2.5·47.6·5e-5 ≈ 0.006 (physics perspective §1.14).
        }
        let r = report(&plant(&presets::door()), 5.0e-5);
        let m = r.to_map();
        assert!((m["levitation.closed_loop"] - 2.5 * 47.57 * 5.0e-5).abs() < 1e-4);
        assert!((m["coil.lag"] - 5.0e-5 * 2.0 * core::f64::consts::PI * 1000.0).abs() < 1e-9);
        assert!((m["sway.pendulum"] - (G0_MPS2 / 1.05f64).sqrt() * 5.0e-5).abs() < 1e-12);
    }

    #[test]
    fn rejects_stiff() {
        // A step far too long for the closed levitation loop is refused (ωh > 0.2), while
        // a capped contact never is (PHYSICS §24).
        let p = plant(&presets::drawer());
        let r = report_tier(&p, Tier::Carriage6, 5.0e-3);
        assert!(!r.ok(), "5 ms step must be refused");
        let worst = r.worst_oscillatory().expect("oscillatory entries");
        assert!(worst.lambda_h > LAMBDA_H_MAX);
        assert!(r.to_map()["contact.upper_stop"] <= LAMBDA_H_MAX);
    }
}
