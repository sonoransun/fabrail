//! Drawer contents: sliding blocks with the vector friction cone, bristle stick-slip, stemware rocking flag; slip and contact-impulse events. Owner: E2 (friction-cone check complete in Wave 0, W0-C1).
//!
//! Plan §4b; FAB-EXA-009. A content item slides when its support's horizontal
//! acceleration leaves the static friction cone,
//!
//! |a_h| > μ_s·(g + a_z),
//!
//! with a_h = (drawer a_x, floor a_y) in the device frame: the drawer body's own
//! acceleration along its rail axis (a levitated drawer is free along x) and the floor
//! acceleration projected on the device's cross-track axis, rotated by the device
//! heading. Stemware additionally gets a rocking flag when |a_h| > g·b/h (b/h ≈
//! 0.3–0.4). μ_s defaults (glassware 0.35, porcelain 0.4, lined 0.8) are \[UNCERTAIN\] and
//! swept ±30 % by the scene. Toppling is not modelled: the only permitted wording is
//! "no Coulomb slip event in the sliding-block model".
//!
//! **Stick-slip** ([`SlidingBlock`], PHYSICS §8): each item is a sliding block on its
//! support with the core elasto-plastic bristle law (`fabrail_plant::friction`, Dupont
//! et al. 2002) generalised to the plane: relative displacement u and velocity w of the
//! item on its support, bristle deflection z (a 2-vector), normal force per unit mass
//! g_eff = g + a_z, and
//!
//! ẅ = −a_h − g_eff·(σ0·z + σ1(|w|)·ż),  ż = w − α(|z|)·|w|·z/z_ss(|w|),
//!
//! with σ0 = μ_k/z_ss, a critically damped stick mode (σ1 as `BristleParams::new`, faded
//! out above [`MICRO_DAMPING_V_MPS`]),
//! the Stribeck curve from μ_s at rest to μ_k sliding, and α = 0 (purely elastic) up to
//! z_ba = 0.7·z_ss. Steady sliding therefore carries μ_k·g_eff and the largest static
//! force is μ_s·g_eff: the block leaves stick exactly where [`friction_cone`] says
//! |a_h| > μ_s·(g + a_z). Integration is semi-implicit Euler at [`SUBSTEP_S`] (the
//! stick mode √(σ0·g) ≈ 530 rad/s is resolved with ω·h ≈ 0.13); the bristle deflection
//! is advanced along the slide direction with the exact per-step law
//! (`friction::bristle_advance`), its cross component relaxing over one z_ss of slide
//! once plastic.
//!
//! **Events** ([`ContentEvent`]; the scene appends the item index to the kind, e.g.
//! `contents:slip:0`): `contents:slip` when an item has slid more than
//! [`SLIP_EVENT_M`] from where it last stuck (10·z_ss, so elastic presliding never
//! counts; a = cone utilisation, b = μ_s); `contents:impact` when a sliding item reaches
//! the end of its free travel [`CLEARANCE_M`] and strikes the drawer wall or a
//! neighbour (inelastic with restitution [`RESTITUTION`]; a = impulse per item N·s,
//! b = impact speed m/s); `contents:rock` when a stemware item's |a_h| first exceeds
//! g·b/h (a = |a_h|/(g·b/h), b = b/h). The scene runs every item at μ·(1 − f), μ and
//! μ·(1 + f) (`contents_mu_sweep_frac` f, ±30 % by default) and emits events for the
//! nominal μ only; the sweep is reported as KPIs.

use fabrail_params::derive::G0_MPS2;
use fabrail_plant::friction::{BristleParams, Z_SS_CARRIAGE6_M, bristle_advance};
use fabrail_types::fmath;
use serde::{Deserialize, Serialize};

/// Kinetic-to-static friction ratio when an item gives no μ_k (\[UNCERTAIN\]: typical
/// glass/ceramic on wood or liner 0.7–0.9; plan §4b sweeps μ ±30 % anyway; PHYSICS §8).
pub const MU_K_OVER_MU_S: f64 = 0.8;

/// Mass of one item when none is given, kg (\[UNCERTAIN\]: a wine glass or a plate;
/// only the impact impulse depends on it; PHYSICS §8).
pub const DEFAULT_ITEM_MASS_KG: f64 = 0.3;

/// Free travel of an item before it meets the drawer wall or a neighbour, per axis, m
/// (\[UNCERTAIN\] design choice; PHYSICS §8).
pub const CLEARANCE_M: f64 = 0.02;

/// Coefficient of restitution of an item striking the wall (\[UNCERTAIN\]: glass on
/// wood/liner 0.2–0.4; Newton's impact law, PHYSICS §8).
pub const RESTITUTION: f64 = 0.3;

/// Slide from the last stick position that counts as a Coulomb slip event, m
/// (10·z_ss: elastic presliding of the bristle never counts; PHYSICS §8).
pub const SLIP_EVENT_M: f64 = 1.0e-4;

/// Relative speed below which a sliding item re-sticks (the Stribeck velocity v_s,
/// `friction::V_STRIBECK_MPS`; PHYSICS §8), m/s.
pub const V_STICK_MPS: f64 = 1.0e-3;

/// Integration sub-step of the sliding blocks, s (PHYSICS §24: ω·h ≈ 0.13 for the
/// bristle stick mode).
pub const SUBSTEP_S: f64 = 2.5e-4;

/// Rocking episode ends below this fraction of the rocking threshold (hysteresis;
/// design choice so one excursion counts once).
const ROCK_RELEASE: f64 = 0.9;

/// Velocity scale of the bristle micro-damping, m/s: σ1(v) = σ1·exp(−(v/v_d)²) keeps the
/// critically damped stick mode and removes the σ1·ż force spike when a sliding item's
/// bristle reverses at speed (Olsson, Åström, Canudas de Wit, Gäfvert & Lischinsky,
/// "Friction models and friction compensation", Eur. J. Control 4, 1998; the friction
/// force then never exceeds μ_s·g_eff; \[UNCERTAIN\] design choice, 10× the Stribeck
/// velocity; PHYSICS §8).
pub const MICRO_DAMPING_V_MPS: f64 = 0.01;

/// Content kind (scene JSON `contents[].kind`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ContentKind {
    /// Glassware (μ_s 0.35 \[UNCERTAIN\]).
    #[default]
    Glassware,
    /// Porcelain (μ_s 0.4 \[UNCERTAIN\]).
    Porcelain,
    /// Items on a high-friction liner (μ_s 0.8 \[UNCERTAIN\]).
    Lined,
    /// Stemware: glassware friction plus the rocking flag (b/h 0.35 \[UNCERTAIN\]).
    Stemware,
    /// Custom: `mu_s` required.
    Custom,
}

impl ContentKind {
    /// Every kind, in schema order.
    pub const ALL: [ContentKind; 5] = [
        ContentKind::Glassware,
        ContentKind::Porcelain,
        ContentKind::Lined,
        ContentKind::Stemware,
        ContentKind::Custom,
    ];

    /// Snake-case name.
    pub const fn slug(self) -> &'static str {
        match self {
            ContentKind::Glassware => "glassware",
            ContentKind::Porcelain => "porcelain",
            ContentKind::Lined => "lined",
            ContentKind::Stemware => "stemware",
            ContentKind::Custom => "custom",
        }
    }

    /// Default static friction coefficient (plan §4b, \[UNCERTAIN\]); `None` for custom.
    pub const fn default_mu_s(self) -> Option<f64> {
        match self {
            ContentKind::Glassware | ContentKind::Stemware => Some(0.35),
            ContentKind::Porcelain => Some(0.4),
            ContentKind::Lined => Some(0.8),
            ContentKind::Custom => None,
        }
    }

    /// Default width-to-height ratio b/h for the rocking flag (stemware 0.35, the middle
    /// of plan §4b's 0.3–0.4 \[UNCERTAIN\]); `None` when rocking is not flagged.
    pub const fn default_b_over_h(self) -> Option<f64> {
        match self {
            ContentKind::Stemware => Some(0.35),
            _ => None,
        }
    }
}

fn one() -> u32 {
    1
}

/// One content item (scene JSON `contents[]`).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContentItem {
    /// Kind.
    pub kind: ContentKind,
    /// Display name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Static friction coefficient (default by kind).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mu_s: Option<f64>,
    /// Kinetic friction coefficient (default: E2's bristle model).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mu_k: Option<f64>,
    /// Width-to-height ratio b/h for the rocking flag.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub b_over_h: Option<f64>,
    /// Mass per item, kg.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mass_kg: Option<f64>,
    /// Number of identical items.
    #[serde(default = "one")]
    pub count: u32,
}

impl ContentItem {
    /// Static friction coefficient: the item's or the kind default.
    pub fn mu_s(&self) -> Option<f64> {
        self.mu_s.or(self.kind.default_mu_s())
    }

    /// Rocking ratio b/h: the item's or the kind default.
    pub fn b_over_h(&self) -> Option<f64> {
        self.b_over_h.or(self.kind.default_b_over_h())
    }
}

/// One friction-cone evaluation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ConeSample {
    /// Horizontal support acceleration in the device frame (x along the rail, y
    /// cross-track), m/s².
    pub a_h_mps2: [f64; 2],
    /// Effective gravity g + a_z, m/s².
    pub g_eff_mps2: f64,
    /// Cone utilisation |a_h| / (μ_s·(g + a_z)); +∞ when g + a_z ≤ 0.
    pub utilisation: f64,
    /// |a_h| > μ_s·(g + a_z) (or the support is in free fall).
    pub slips: bool,
    /// |a_h| > g·b/h (only when a rocking ratio is given).
    pub rocks: bool,
}

/// Floor acceleration (building x, y) projected on the device's cross-track axis for a
/// rail whose +x points at `heading_rad` (counter-clockwise from building x), m/s².
pub fn floor_cross_track(a_floor_xy_mps2: [f64; 2], heading_rad: f64) -> f64 {
    -fmath::sin(heading_rad) * a_floor_xy_mps2[0] + fmath::cos(heading_rad) * a_floor_xy_mps2[1]
}

/// The vector friction cone of plan §4b: a_h = (a_drawer_x, floor a_y in the device
/// frame), slips iff |a_h| > μ_s·(g + a_z); rocks iff |a_h| > g·b/h.
///
/// * `a_drawer_x_mps2`: absolute acceleration of the drawer body along its rail axis;
/// * `a_floor_xy_mps2`: floor acceleration in the building frame (x, y);
/// * `a_floor_z_mps2`: floor vertical acceleration (up positive);
/// * `heading_rad`: direction of the rail's +x in the building frame;
/// * `mu_s`: static friction coefficient; `b_over_h`: rocking ratio, if flagged.
pub fn friction_cone(
    a_drawer_x_mps2: f64,
    a_floor_xy_mps2: [f64; 2],
    a_floor_z_mps2: f64,
    heading_rad: f64,
    mu_s: f64,
    b_over_h: Option<f64>,
) -> ConeSample {
    let a_h = [
        a_drawer_x_mps2,
        floor_cross_track(a_floor_xy_mps2, heading_rad),
    ];
    let mag = fmath::hypot(a_h[0], a_h[1]);
    let g_eff = G0_MPS2 + a_floor_z_mps2;
    let limit = mu_s * g_eff;
    let (utilisation, slips) = if g_eff <= 0.0 {
        (f64::INFINITY, true)
    } else if limit <= 0.0 {
        (f64::INFINITY, mag > 0.0)
    } else {
        (mag / limit, mag > limit)
    };
    let rocks = b_over_h.is_some_and(|r| mag > G0_MPS2 * r);
    ConeSample {
        a_h_mps2: a_h,
        g_eff_mps2: g_eff,
        utilisation,
        slips,
        rocks,
    }
}

/// One content event (see the module docs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContentEvent {
    /// Time, s.
    pub t_s: f64,
    /// `contents:slip`, `contents:impact` or `contents:rock`.
    pub kind: &'static str,
    /// Item index in the device's `contents` list.
    pub item: usize,
    /// First payload (see the module docs).
    pub a: f64,
    /// Second payload.
    pub b: f64,
}

/// Running statistics of one sliding block.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BlockStats {
    /// Coulomb slip events (episodes).
    pub slip_events: u32,
    /// Largest displacement from the start position, m.
    pub max_offset_m: f64,
    /// Total slide distance, m.
    pub slide_m: f64,
    /// Largest cone utilisation |a_h|/(μ_s·g_eff).
    pub max_utilisation: f64,
    /// Wall/neighbour impacts.
    pub impacts: u32,
    /// Largest impact impulse per item, N·s.
    pub max_impulse_ns: f64,
    /// Rocking episodes (stemware only).
    pub rock_events: u32,
    /// Peak horizontal absolute acceleration of the item, m/s².
    pub peak_abs_acc_mps2: f64,
}

/// One content item as a planar sliding block on its support (see the module docs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SlidingBlock {
    /// Static friction coefficient used.
    pub mu_s: f64,
    /// Kinetic friction coefficient used.
    pub mu_k: f64,
    /// Rocking ratio b/h, if flagged.
    pub b_over_h: Option<f64>,
    /// Mass of one item, kg.
    pub mass_kg: f64,
    bristle: BristleParams,
    u: [f64; 2],
    w: [f64; 2],
    z: [f64; 2],
    anchor: [f64; 2],
    sliding: bool,
    rocking: bool,
    /// Statistics.
    pub stats: BlockStats,
}

fn norm(v: [f64; 2]) -> f64 {
    fmath::hypot(v[0], v[1])
}

impl SlidingBlock {
    /// A block for `item` with its μ scaled by `mu_scale` (the ±30 % sweep); `None` when
    /// the item has no friction coefficient.
    pub fn new(item: &ContentItem, mu_scale: f64) -> Option<SlidingBlock> {
        let mu_s = item.mu_s()? * mu_scale;
        let mu_k = item
            .mu_k
            .map_or(MU_K_OVER_MU_S * mu_s, |m| m * mu_scale)
            .min(mu_s);
        if !(mu_s.is_finite() && mu_s > 0.0 && mu_k > 0.0) {
            return None;
        }
        Some(SlidingBlock {
            mu_s,
            mu_k,
            b_over_h: item.b_over_h(),
            mass_kg: item.mass_kg.unwrap_or(DEFAULT_ITEM_MASS_KG),
            bristle: BristleParams::new(mu_k, mu_s, Z_SS_CARRIAGE6_M),
            u: [0.0; 2],
            w: [0.0; 2],
            z: [0.0; 2],
            anchor: [0.0; 2],
            sliding: false,
            rocking: false,
            stats: BlockStats::default(),
        })
    }

    /// Relative displacement of the item on its support, m.
    pub fn offset_m(&self) -> [f64; 2] {
        self.u
    }

    /// True while the item slides.
    pub fn sliding(&self) -> bool {
        self.sliding
    }

    /// The bristle state rate ż (vector elasto-plastic law) at speed |w|.
    fn z_dot(&self) -> [f64; 2] {
        let s = norm(self.w);
        if s == 0.0 {
            return [0.0; 2];
        }
        let p = &self.bristle;
        let z_ss = p.z_ss_at(s);
        let z_ba = p.z_ba_ratio.clamp(0.0, 1.0) * z_ss;
        let zm = norm(self.z);
        let along = self.z[0] * self.w[0] + self.z[1] * self.w[1];
        let alpha = if zm <= z_ba || along <= 0.0 {
            0.0
        } else if zm >= z_ss || z_ss <= z_ba {
            1.0
        } else {
            z_ss * (zm - z_ba) / (zm * (z_ss - z_ba))
        };
        let k = if z_ss > 0.0 { alpha * s / z_ss } else { 0.0 };
        [self.w[0] - k * self.z[0], self.w[1] - k * self.z[1]]
    }

    /// Advances by `h` seconds at support acceleration `a_h` (device frame, m/s²) and
    /// vertical floor acceleration `a_z`; emits events for item `item` into `out` when
    /// `emit` is set.
    pub fn step(
        &mut self,
        t_s: f64,
        a_h: [f64; 2],
        a_z: f64,
        h: f64,
        item: usize,
        emit: Option<&mut Vec<ContentEvent>>,
    ) {
        let g_eff = G0_MPS2 + a_z;
        let a_mag = norm(a_h);
        let util = if g_eff > 0.0 {
            a_mag / (self.mu_s * g_eff)
        } else {
            f64::INFINITY
        };
        if util.is_finite() {
            self.stats.max_utilisation = self.stats.max_utilisation.max(util);
        }
        let p = self.bristle;
        let s0 = p.sigma0_per_m();
        let mut events: [Option<ContentEvent>; 3] = [None; 3];
        // Velocity: friction from the current bristle state (explicit), then position
        // (semi-implicit Euler).
        let f = if g_eff > 0.0 {
            let zd = self.z_dot();
            let r = norm(self.w) / MICRO_DAMPING_V_MPS;
            let s1 = p.sigma1_s_per_m * fmath::exp(-r * r);
            [
                -g_eff * (s0 * self.z[0] + s1 * zd[0]),
                -g_eff * (s0 * self.z[1] + s1 * zd[1]),
            ]
        } else {
            [0.0; 2]
        };
        let acc = [-a_h[0] + f[0], -a_h[1] + f[1]];
        self.stats.peak_abs_acc_mps2 = self
            .stats
            .peak_abs_acc_mps2
            .max(norm([a_h[0] + acc[0], a_h[1] + acc[1]]));
        for (w, a) in self.w.iter_mut().zip(acc) {
            *w += h * a;
        }
        let du = [h * self.w[0], h * self.w[1]];
        for (u, d) in self.u.iter_mut().zip(du) {
            *u += d;
        }
        let d = norm(du);
        self.stats.slide_m += if self.sliding { d } else { 0.0 };
        // Bristle: exact law along the slide direction; the cross component relaxes
        // over one z_ss of slide once plastic. Free flight unloads it.
        if g_eff <= 0.0 {
            self.z = [0.0; 2];
        } else if d > 0.0 {
            let dir = [du[0] / d, du[1] / d];
            let par = self.z[0] * dir[0] + self.z[1] * dir[1];
            let perp = [self.z[0] - par * dir[0], self.z[1] - par * dir[1]];
            let speed = norm(self.w);
            let z_ss = p.z_ss_at(speed);
            let plastic = norm(self.z) > p.z_ba_ratio.clamp(0.0, 1.0) * z_ss;
            let par_new = bristle_advance(&p, par, d, speed);
            let decay = if plastic && z_ss > 0.0 {
                fmath::exp(-d / z_ss)
            } else {
                1.0
            };
            self.z = [
                par_new * dir[0] + decay * perp[0],
                par_new * dir[1] + decay * perp[1],
            ];
        }
        // Wall / neighbour contact at the end of the free travel (per axis).
        for k in 0..2 {
            if self.u[k].abs() > CLEARANCE_M {
                let side = self.u[k].signum();
                self.u[k] = side * CLEARANCE_M;
                if self.w[k] * side > 0.0 {
                    let v_imp = self.w[k].abs();
                    let impulse = self.mass_kg * (1.0 + RESTITUTION) * v_imp;
                    self.w[k] *= -RESTITUTION;
                    self.z[k] = 0.0;
                    self.stats.impacts += 1;
                    self.stats.max_impulse_ns = self.stats.max_impulse_ns.max(impulse);
                    events[1] = Some(ContentEvent {
                        t_s,
                        kind: "contents:impact",
                        item,
                        a: impulse,
                        b: v_imp,
                    });
                }
            }
        }
        self.stats.max_offset_m = self.stats.max_offset_m.max(norm(self.u));
        // Slip episodes: onset beyond SLIP_EVENT_M from the last stick position; they end
        // when the item re-sticks inside the static cone.
        let away = norm([self.u[0] - self.anchor[0], self.u[1] - self.anchor[1]]);
        if !self.sliding && away > SLIP_EVENT_M {
            self.sliding = true;
            self.stats.slip_events += 1;
            events[0] = Some(ContentEvent {
                t_s,
                kind: "contents:slip",
                item,
                a: util,
                b: self.mu_s,
            });
        } else if self.sliding && norm(self.w) < V_STICK_MPS && util < 1.0 {
            self.sliding = false;
            self.anchor = self.u;
        }
        // Stemware rocking flag |a_h| > g·b/h (toppling is not modelled).
        if let Some(r) = self.b_over_h.filter(|r| *r > 0.0) {
            let thr = G0_MPS2 * r;
            if !self.rocking && a_mag > thr {
                self.rocking = true;
                self.stats.rock_events += 1;
                events[2] = Some(ContentEvent {
                    t_s,
                    kind: "contents:rock",
                    item,
                    a: a_mag / thr,
                    b: r,
                });
            } else if self.rocking && a_mag < ROCK_RELEASE * thr {
                self.rocking = false;
            }
        }
        if let Some(out) = emit {
            out.extend(events.into_iter().flatten());
        }
    }
}

/// The μ-sweep points of the scene: μ·(1 − f), μ, μ·(1 + f).
pub const SWEEP_POINTS: usize = 3;

/// Sweep labels (KPI suffixes).
pub const SWEEP_LABELS: [&str; SWEEP_POINTS] = ["mu_lo", "mu_nom", "mu_hi"];

/// All content items of one device, each at the three μ-sweep points.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DeviceContents {
    /// Per item: (item index, count, blocks at μ_lo, μ_nom, μ_hi).
    pub items: Vec<(usize, u32, [SlidingBlock; SWEEP_POINTS])>,
    /// Events of the nominal-μ blocks, in time order.
    pub events: Vec<ContentEvent>,
}

impl DeviceContents {
    /// Blocks for `items` swept by ±`sweep_frac`; items without μ are skipped.
    pub fn new(items: &[ContentItem], sweep_frac: f64) -> DeviceContents {
        let f = if sweep_frac.is_finite() {
            sweep_frac.clamp(0.0, 0.99)
        } else {
            0.0
        };
        let list = items
            .iter()
            .enumerate()
            .filter_map(|(i, it)| {
                let b = [
                    SlidingBlock::new(it, 1.0 - f)?,
                    SlidingBlock::new(it, 1.0)?,
                    SlidingBlock::new(it, 1.0 + f)?,
                ];
                Some((i, it.count, b))
            })
            .collect();
        DeviceContents {
            items: list,
            events: Vec::new(),
        }
    }

    /// True when the device carries contents.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Advances every block over [t0, t0 + dt) in [`SUBSTEP_S`] steps with the support
    /// acceleration `support(t)` → (a_h device frame, a_z), m/s².
    pub fn advance(&mut self, t0_s: f64, dt_s: f64, support: impl Fn(f64) -> ([f64; 2], f64)) {
        if self.items.is_empty() || dt_s.is_nan() || dt_s <= 0.0 {
            return;
        }
        let n = (dt_s / SUBSTEP_S).round().max(1.0) as usize;
        let h = dt_s / n as f64;
        for s in 0..n {
            let t = t0_s + (s as f64 + 1.0) * h;
            let (a_h, a_z) = support(t);
            for (item, _, blocks) in &mut self.items {
                for (k, b) in blocks.iter_mut().enumerate() {
                    let emit = (k == 1).then_some(&mut self.events);
                    b.step(t, a_h, a_z, h, *item, emit);
                }
            }
        }
    }

    /// Nominal-μ cone utilisation right now is not stored; the largest so far over the
    /// nominal blocks.
    pub fn max_utilisation(&self) -> f64 {
        self.items
            .iter()
            .map(|(_, _, b)| b[1].stats.max_utilisation)
            .fold(0.0, f64::max)
    }

    /// Nominal-μ items sliding now (counting identical items).
    pub fn sliding_now(&self) -> u32 {
        self.items
            .iter()
            .filter(|(_, _, b)| b[1].sliding())
            .map(|(_, c, _)| *c)
            .sum()
    }

    /// Summed statistics at sweep point `k`: (slip events, items that slipped, impacts,
    /// rock events, max offset m, max impulse N·s, max utilisation, peak item |a| m/s²).
    pub fn totals(&self, k: usize) -> ContentTotals {
        let mut t = ContentTotals::default();
        for (_, count, blocks) in &self.items {
            let Some(b) = blocks.get(k) else {
                continue;
            };
            let s = &b.stats;
            t.slip_events += s.slip_events;
            if s.slip_events > 0 {
                t.items_slipped += count;
            }
            t.impacts += s.impacts;
            t.rock_events += s.rock_events;
            t.max_offset_m = t.max_offset_m.max(s.max_offset_m);
            t.slide_m += s.slide_m;
            t.max_impulse_ns = t.max_impulse_ns.max(s.max_impulse_ns);
            t.max_utilisation = t.max_utilisation.max(s.max_utilisation);
            t.peak_abs_acc_mps2 = t.peak_abs_acc_mps2.max(s.peak_abs_acc_mps2);
        }
        t
    }
}

/// Summed content statistics of one device at one sweep point.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ContentTotals {
    /// Coulomb slip events.
    pub slip_events: u32,
    /// Items (counting identical ones) with at least one slip event.
    pub items_slipped: u32,
    /// Wall/neighbour impacts.
    pub impacts: u32,
    /// Rocking episodes.
    pub rock_events: u32,
    /// Largest displacement, m.
    pub max_offset_m: f64,
    /// Summed slide distance over the items (one per identical group), m.
    pub slide_m: f64,
    /// Largest impact impulse per item, N·s.
    pub max_impulse_ns: f64,
    /// Largest cone utilisation.
    pub max_utilisation: f64,
    /// Peak horizontal absolute acceleration of an item, m/s².
    pub peak_abs_acc_mps2: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    // req: FAB-EXA-009
    #[test]
    fn friction_cone_slip() {
        let g = G0_MPS2;
        let mu = ContentKind::Glassware.default_mu_s().unwrap_or(0.0);
        // Quiet floor, resting drawer: no slip.
        let q = friction_cone(0.0, [0.0, 0.0], 0.0, 0.0, mu, None);
        assert!(!q.slips && q.utilisation == 0.0);
        // Just inside / just outside the cone along the drawer axis.
        assert!(!friction_cone(0.99 * mu * g, [0.0, 0.0], 0.0, 0.0, mu, None).slips);
        assert!(friction_cone(1.01 * mu * g, [0.0, 0.0], 0.0, 0.0, mu, None).slips);
        // Vector sum: two components each inside the cone can slip together.
        let c = 0.8 * mu * g;
        let s = friction_cone(c, [0.0, c], 0.0, 0.0, mu, None);
        assert!(s.slips, "|a_h| = √2·0.8·μg > μg");
        assert!((s.utilisation - 0.8 * 2f64.sqrt()).abs() < 1e-12);
        // Vertical acceleration scales the cone: downward floor motion shrinks it.
        assert!(friction_cone(0.8 * mu * g, [0.0, 0.0], -0.3 * g, 0.0, mu, None).slips);
        assert!(!friction_cone(1.1 * mu * g, [0.0, 0.0], 0.3 * g, 0.0, mu, None).slips);
        assert!(
            friction_cone(0.0, [0.0, 0.0], -1.01 * g, 0.0, mu, None).slips,
            "free fall"
        );
        // Heading: at 90°, building-y floor motion is along the (free) drawer axis and
        // does not load the contents; building-x motion becomes cross-track.
        let h = core::f64::consts::FRAC_PI_2;
        assert!(!friction_cone(0.0, [0.0, 2.0 * mu * g], 0.0, h, mu, None).slips);
        let x = friction_cone(0.0, [2.0 * mu * g, 0.0], 0.0, h, mu, None);
        assert!(x.slips && (x.a_h_mps2[1] + 2.0 * mu * g).abs() < 1e-9);
        // Stemware rocking flag at |a_h| > g·b/h.
        let b = ContentKind::Stemware.default_b_over_h();
        assert!(friction_cone(0.36 * g, [0.0, 0.0], 0.0, 0.0, 0.8, b).rocks);
        assert!(!friction_cone(0.34 * g, [0.0, 0.0], 0.0, 0.0, 0.8, b).rocks);
        // μ defaults (plan §4b).
        assert_eq!(ContentKind::Porcelain.default_mu_s(), Some(0.4));
        assert_eq!(ContentKind::Lined.default_mu_s(), Some(0.8));
        assert_eq!(ContentKind::Custom.default_mu_s(), None);

        // Elasto-plastic stick-slip (plan §4b; PHYSICS §8): a held support acceleration
        // inside the cone never slips (no ratchet over 2 s of 5 Hz vibration at 0.9 of
        // the cone, the presliding stays below z_ss); just outside it slips.
        let glass = ContentItem {
            kind: ContentKind::Glassware,
            count: 1,
            ..ContentItem::default()
        };
        let run = |amp: f64, f_hz: f64, t_end: f64, heading: f64| {
            let mut dc = DeviceContents::new(std::slice::from_ref(&glass), 0.3);
            let n = (t_end / 0.01) as usize;
            for k in 0..n {
                let t0 = k as f64 * 0.01;
                dc.advance(t0, 0.01, |t| {
                    let a = amp * (2.0 * core::f64::consts::PI * f_hz * t).sin();
                    let c = friction_cone(0.0, [a, 0.0], 0.0, heading, mu, None);
                    (c.a_h_mps2, 0.0)
                });
            }
            dc
        };
        let inside = run(0.9 * mu * g, 5.0, 2.0, core::f64::consts::FRAC_PI_2);
        let nom = inside.totals(1);
        assert_eq!(nom.slip_events, 0, "0.9 of the cone: no Coulomb slip event");
        assert!(
            nom.max_offset_m < 10.0 * Z_SS_CARRIAGE6_M,
            "presliding {} m stays elastic (≤ 10·z_ss, no ratchet)",
            nom.max_offset_m
        );
        assert!((nom.max_utilisation - 0.9).abs() < 0.01);
        // The +30 % point also holds; the −30 % point (μ 0.245 < 0.9·0.35) slips.
        assert_eq!(inside.totals(2).slip_events, 0);
        assert!(
            inside.totals(0).slip_events > 0,
            "μ −30 % slips at 0.9·μ_nom·g"
        );
        let outside = run(1.3 * mu * g, 2.0, 2.0, core::f64::consts::FRAC_PI_2);
        let o = outside.totals(1);
        assert!(o.slip_events > 0 && o.items_slipped == 1, "{o:?}");
        assert!(
            outside
                .events
                .iter()
                .any(|e| e.kind == "contents:slip" && e.a > 1.0)
        );
        // Sliding carries μ_k·g: the item's own peak acceleration stays near the cone
        // (support 1.3·μ_s·g, item ≤ μ_s·g plus the bristle transient).
        assert!(
            o.peak_abs_acc_mps2 < 1.1 * mu * g,
            "item |a| {}",
            o.peak_abs_acc_mps2
        );
        // Heading: the same building-x motion along a heading-0 drawer axis is the free
        // (levitated) axis; with the drawer body at rest it does not load the contents.
        let along = run(1.3 * mu * g, 2.0, 1.0, 0.0);
        assert_eq!(
            along.totals(1).slip_events,
            0,
            "along the free axis at heading 0"
        );
        // Contact impulses: a large low-frequency push drives the item to the wall.
        let push = run(3.0 * mu * g, 0.5, 2.0, core::f64::consts::FRAC_PI_2);
        let p = push.totals(1);
        assert!(p.impacts > 0 && p.max_impulse_ns > 0.0, "{p:?}");
        assert!(
            push.events
                .iter()
                .any(|e| e.kind == "contents:impact" && e.a > 0.0)
        );
        assert!(p.max_offset_m <= CLEARANCE_M * 2f64.sqrt() + 1e-12);
        // Stemware rocking events at |a_h| > g·b/h (b/h 0.35), once per episode.
        let stem = ContentItem {
            kind: ContentKind::Stemware,
            count: 1,
            ..ContentItem::default()
        };
        let mut dc = DeviceContents::new(std::slice::from_ref(&stem), 0.0);
        dc.advance(0.0, 0.5, |_| ([0.36 * g, 0.0], 0.0));
        dc.advance(0.5, 0.5, |_| ([0.0, 0.0], 0.0));
        dc.advance(1.0, 0.5, |_| ([0.0, 0.37 * g], 0.0));
        assert_eq!(dc.totals(1).rock_events, 2);
        assert_eq!(
            dc.events
                .iter()
                .filter(|e| e.kind == "contents:rock")
                .count(),
            2
        );
        // Free fall unloads the contact: everything slips with the support.
        let mut ff = DeviceContents::new(std::slice::from_ref(&glass), 0.0);
        ff.advance(0.0, 0.1, |_| ([0.1 * g, 0.0], -1.2 * g));
        assert!(ff.totals(1).slip_events > 0);
    }
}
