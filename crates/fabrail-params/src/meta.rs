//! Per-field metadata: unit, validated range and source for every serde leaf of
//! `SystemParams` (core design §B.2). Drives `Params.describe()`, the `validate` range
//! checks and the generated docs/PARAMETERS.md; `tests/meta_complete.rs` asserts that the
//! set of struct (and serde) leaves equals the set of paths here, in declaration order,
//! and that each getter returns that leaf's value.
//!
//! Paths are dotted serde paths, e.g. `"design.suspension.g_star_m"`; array elements use
//! the index as a segment (`"design.suspension.stations.0.x_m"`). The JSON pointer used
//! by `Params.get/set` is the same path with `/` separators. Enums read as their wire
//! discriminant (`RailClass::Drawer` = 0 …) and flags as 0/1; min/max apply to that value.
//!
//! Each `fields!` entry is `path, unit, min, max, source, |p| getter;`, so metadata and
//! getter cannot drift apart. Keep entries in declaration order when adding a field.

use crate::SystemParams;

/// Metadata for one parameter leaf.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FieldMeta {
    /// Dotted serde path.
    pub path: &'static str,
    /// SI unit (`"m"`, `"kg"`, `"A"`, `"N/m"`, `"1"` for dimensionless, `""` for enums
    /// and flags).
    pub unit: &'static str,
    /// Minimum accepted value (inclusive).
    pub min: f64,
    /// Maximum accepted value (inclusive).
    pub max: f64,
    /// Source of the default values (critique row, standard, datasheet, PHYSICS §).
    pub source: &'static str,
}

/// A leaf getter: the field value as f64 (enums as discriminant, flags as 0/1).
type Getter = fn(&SystemParams) -> f64;

/// Defines `FIELDS`, `N_FIELDS` and the parallel getter table from one list.
macro_rules! fields {
    ($($path:literal, $unit:literal, $min:expr, $max:expr, $src:literal, |$p:ident| $get:expr;)*) => {
        /// Number of parameter leaves.
        pub const N_FIELDS: usize = [$($path),*].len();

        /// Every parameter leaf, in declaration (serde) order.
        pub static FIELDS: &[FieldMeta] = &[
            $(FieldMeta { path: $path, unit: $unit, min: $min, max: $max, source: $src },)*
        ];

        /// Getters parallel to `FIELDS`.
        static GETTERS: [Getter; N_FIELDS] = [$(|$p: &SystemParams| -> f64 { $get },)*];
    };
}

fields! {
    "design.class", "", 0.0, 3.0,
        "core §C.2 classes (RailClass)", |p| f64::from(p.design.class.as_u8());
    "design.rest_policy", "", 0.0, 3.0,
        "PWR-001; F17 (seat/hover/land/land_lock)", |p| f64::from(p.design.rest_policy.as_u8());
    "design.egress", "", 0.0, 2.0,
        "SEI-011; IBC 1010.3 egress role", |p| f64::from(p.design.egress.as_u8());
    "design.presence", "", 0.0, 1.0,
        "SAF-003 presence sensing (none/pl_d)", |p| f64::from(p.design.presence.as_u8());
    "design.operating_mode", "", 0.0, 2.0,
        "core §C.2 row \"Operating mode\"; F5", |p| f64::from(p.design.operating_mode as u8);
    "design.carriage.m_star_kg", "kg", 0.1, 1.0e8,
        "core §C.2 row m*; PHYSICS §3", |p| p.design.carriage.m_star_kg;
    "design.carriage.m_rated_kg", "kg", 0.1, 1.0e8,
        "core §C.2 row m_rated; F19", |p| p.design.carriage.m_rated_kg;
    "design.carriage.m_op_min_kg", "kg", 0.1, 1.0e8,
        "core §C.2 m_op range; F18", |p| p.design.carriage.m_op_min_kg;
    "design.carriage.j_kgm2.0", "kg·m²", 1.0e-6, 1.0e10,
        "principal inertia about x (box model at m*); PHYSICS §5", |p| p.design.carriage.j_kgm2[0];
    "design.carriage.j_kgm2.1", "kg·m²", 1.0e-6, 1.0e10,
        "principal inertia about y (box model at m*); PHYSICS §5", |p| p.design.carriage.j_kgm2[1];
    "design.carriage.j_kgm2.2", "kg·m²", 1.0e-6, 1.0e10,
        "principal inertia about z (box model at m*); PHYSICS §5", |p| p.design.carriage.j_kgm2[2];
    "design.carriage.cg_m.0", "m", -100.0, 100.0,
        "CoM x offset in the carriage frame; PHYSICS §5, core §C.1", |p| p.design.carriage.cg_m[0];
    "design.carriage.cg_m.1", "m", -100.0, 100.0,
        "CoM y offset in the carriage frame; PHYSICS §5, core §C.1", |p| p.design.carriage.cg_m[1];
    "design.carriage.cg_m.2", "m", -100.0, 100.0,
        "CoM z offset in the carriage frame; PHYSICS §5, core §C.1", |p| p.design.carriage.cg_m[2];
    "design.carriage.top_hung", "", 0.0, 1.0,
        "core §C.1 (DOOR/WALL top-hung)", |p| f64::from(u8::from(p.design.carriage.top_hung));
    "design.carriage.length_m", "m", 0.01, 1000.0,
        "body length along x; plan §2b item 3 (DRAWER 50 % extension)", |p| p.design.carriage.length_m;
    "design.suspension.n_magnets", "", 1.0, 8.0,
        "core §C.1 element count (≤ MAX_MAGNETS)", |p| f64::from(p.design.suspension.n_magnets);
    "design.suspension.stations.0.x_m", "m", -100.0, 100.0,
        "element station x; core §C.1", |p| p.design.suspension.stations[0].x_m;
    "design.suspension.stations.0.y_m", "m", -10.0, 10.0,
        "element station y; core §C.1", |p| p.design.suspension.stations[0].y_m;
    "design.suspension.stations.0.z_m", "m", -1.0, 1.0,
        "pole-face plane z; PHYSICS §1", |p| p.design.suspension.stations[0].z_m;
    "design.suspension.stations.0.normal_sign", "1", -1.0, 1.0,
        "+1 lift, −1 hold-down, 0 unused; core §C.1", |p| p.design.suspension.stations[0].normal_sign;
    "design.suspension.stations.0.has_pm", "", 0.0, 1.0,
        "PM bias fitted (false = pure-EMS hold-down); core §C.1", |p| f64::from(u8::from(p.design.suspension.stations[0].has_pm));
    "design.suspension.stations.1.x_m", "m", -100.0, 100.0,
        "element station x; core §C.1", |p| p.design.suspension.stations[1].x_m;
    "design.suspension.stations.1.y_m", "m", -10.0, 10.0,
        "element station y; core §C.1", |p| p.design.suspension.stations[1].y_m;
    "design.suspension.stations.1.z_m", "m", -1.0, 1.0,
        "pole-face plane z; PHYSICS §1", |p| p.design.suspension.stations[1].z_m;
    "design.suspension.stations.1.normal_sign", "1", -1.0, 1.0,
        "+1 lift, −1 hold-down, 0 unused; core §C.1", |p| p.design.suspension.stations[1].normal_sign;
    "design.suspension.stations.1.has_pm", "", 0.0, 1.0,
        "PM bias fitted (false = pure-EMS hold-down); core §C.1", |p| f64::from(u8::from(p.design.suspension.stations[1].has_pm));
    "design.suspension.stations.2.x_m", "m", -100.0, 100.0,
        "element station x; core §C.1", |p| p.design.suspension.stations[2].x_m;
    "design.suspension.stations.2.y_m", "m", -10.0, 10.0,
        "element station y; core §C.1", |p| p.design.suspension.stations[2].y_m;
    "design.suspension.stations.2.z_m", "m", -1.0, 1.0,
        "pole-face plane z; PHYSICS §1", |p| p.design.suspension.stations[2].z_m;
    "design.suspension.stations.2.normal_sign", "1", -1.0, 1.0,
        "+1 lift, −1 hold-down, 0 unused; core §C.1", |p| p.design.suspension.stations[2].normal_sign;
    "design.suspension.stations.2.has_pm", "", 0.0, 1.0,
        "PM bias fitted (false = pure-EMS hold-down); core §C.1", |p| f64::from(u8::from(p.design.suspension.stations[2].has_pm));
    "design.suspension.stations.3.x_m", "m", -100.0, 100.0,
        "element station x; core §C.1", |p| p.design.suspension.stations[3].x_m;
    "design.suspension.stations.3.y_m", "m", -10.0, 10.0,
        "element station y; core §C.1", |p| p.design.suspension.stations[3].y_m;
    "design.suspension.stations.3.z_m", "m", -1.0, 1.0,
        "pole-face plane z; PHYSICS §1", |p| p.design.suspension.stations[3].z_m;
    "design.suspension.stations.3.normal_sign", "1", -1.0, 1.0,
        "+1 lift, −1 hold-down, 0 unused; core §C.1", |p| p.design.suspension.stations[3].normal_sign;
    "design.suspension.stations.3.has_pm", "", 0.0, 1.0,
        "PM bias fitted (false = pure-EMS hold-down); core §C.1", |p| f64::from(u8::from(p.design.suspension.stations[3].has_pm));
    "design.suspension.stations.4.x_m", "m", -100.0, 100.0,
        "element station x; core §C.1", |p| p.design.suspension.stations[4].x_m;
    "design.suspension.stations.4.y_m", "m", -10.0, 10.0,
        "element station y; core §C.1", |p| p.design.suspension.stations[4].y_m;
    "design.suspension.stations.4.z_m", "m", -1.0, 1.0,
        "pole-face plane z; PHYSICS §1", |p| p.design.suspension.stations[4].z_m;
    "design.suspension.stations.4.normal_sign", "1", -1.0, 1.0,
        "+1 lift, −1 hold-down, 0 unused; core §C.1", |p| p.design.suspension.stations[4].normal_sign;
    "design.suspension.stations.4.has_pm", "", 0.0, 1.0,
        "PM bias fitted (false = pure-EMS hold-down); core §C.1", |p| f64::from(u8::from(p.design.suspension.stations[4].has_pm));
    "design.suspension.stations.5.x_m", "m", -100.0, 100.0,
        "element station x; core §C.1", |p| p.design.suspension.stations[5].x_m;
    "design.suspension.stations.5.y_m", "m", -10.0, 10.0,
        "element station y; core §C.1", |p| p.design.suspension.stations[5].y_m;
    "design.suspension.stations.5.z_m", "m", -1.0, 1.0,
        "pole-face plane z; PHYSICS §1", |p| p.design.suspension.stations[5].z_m;
    "design.suspension.stations.5.normal_sign", "1", -1.0, 1.0,
        "+1 lift, −1 hold-down, 0 unused; core §C.1", |p| p.design.suspension.stations[5].normal_sign;
    "design.suspension.stations.5.has_pm", "", 0.0, 1.0,
        "PM bias fitted (false = pure-EMS hold-down); core §C.1", |p| f64::from(u8::from(p.design.suspension.stations[5].has_pm));
    "design.suspension.stations.6.x_m", "m", -100.0, 100.0,
        "element station x; core §C.1", |p| p.design.suspension.stations[6].x_m;
    "design.suspension.stations.6.y_m", "m", -10.0, 10.0,
        "element station y; core §C.1", |p| p.design.suspension.stations[6].y_m;
    "design.suspension.stations.6.z_m", "m", -1.0, 1.0,
        "pole-face plane z; PHYSICS §1", |p| p.design.suspension.stations[6].z_m;
    "design.suspension.stations.6.normal_sign", "1", -1.0, 1.0,
        "+1 lift, −1 hold-down, 0 unused; core §C.1", |p| p.design.suspension.stations[6].normal_sign;
    "design.suspension.stations.6.has_pm", "", 0.0, 1.0,
        "PM bias fitted (false = pure-EMS hold-down); core §C.1", |p| f64::from(u8::from(p.design.suspension.stations[6].has_pm));
    "design.suspension.stations.7.x_m", "m", -100.0, 100.0,
        "element station x; core §C.1", |p| p.design.suspension.stations[7].x_m;
    "design.suspension.stations.7.y_m", "m", -10.0, 10.0,
        "element station y; core §C.1", |p| p.design.suspension.stations[7].y_m;
    "design.suspension.stations.7.z_m", "m", -1.0, 1.0,
        "pole-face plane z; PHYSICS §1", |p| p.design.suspension.stations[7].z_m;
    "design.suspension.stations.7.normal_sign", "1", -1.0, 1.0,
        "+1 lift, −1 hold-down, 0 unused; core §C.1", |p| p.design.suspension.stations[7].normal_sign;
    "design.suspension.stations.7.has_pm", "", 0.0, 1.0,
        "PM bias fitted (false = pure-EMS hold-down); core §C.1", |p| f64::from(u8::from(p.design.suspension.stations[7].has_pm));
    "design.suspension.b_r_t", "T", 0.1, 2.0,
        "sintered NdFeB B_r (N42 class 1.3 T); core §C.2 sizing", |p| p.design.suspension.b_r_t;
    "design.suspension.mu_rec", "1", 1.0, 1.5,
        "NdFeB recoil permeability 1.05; core §C.2 sizing", |p| p.design.suspension.mu_rec;
    "design.suspension.h_m_m", "m", 1.0e-4, 0.2,
        "size_magnets h_m rounded; core §C.2 row h_m", |p| p.design.suspension.h_m_m;
    "design.suspension.pole_w_m", "m", 0.001, 2.0,
        "pole width; core §C.2 row pole w×l", |p| p.design.suspension.pole_w_m;
    "design.suspension.pole_l_m", "m", 0.001, 10.0,
        "solved so n·F(g*) = m*·g₀ (derive::balance_pole_length); core §C.2", |p| p.design.suspension.pole_l_m;
    "design.suspension.b_g_t", "T", 0.05, 1.5,
        "design gap flux; core §C.2 row B_g", |p| p.design.suspension.b_g_t;
    "design.suspension.turns", "1", 1.0, 1.0e5,
        "coil turns N; core §C.2", |p| p.design.suspension.turns;
    "design.suspension.r_coil_ohm", "Ω", 1.0e-4, 1000.0,
        "coil resistance at 20 °C; core §C.2", |p| p.design.suspension.r_coil_ohm;
    "design.suspension.l_sigma_h", "H", 0.0, 1.0,
        "leakage inductance (0: core §C.2 L = N²P(g*)); PHYSICS §4", |p| p.design.suspension.l_sigma_h;
    "design.suspension.i_cont_a", "A", 0.01, 10000.0,
        "continuous current (I²t floor); core §C.2 †, plan §2b item 4", |p| p.design.suspension.i_cont_a;
    "design.suspension.i_pulse_a", "A", 0.01, 10000.0,
        "pulse current (hardware clamp); core §C.2 †, plan §2b item 4", |p| p.design.suspension.i_pulse_a;
    "design.suspension.t_pulse_s", "s", 0.0, 10.0,
        "pulse window ≤ 200 ms; plan §2b item 4", |p| p.design.suspension.t_pulse_s;
    "design.suspension.g_star_m", "m", 1.0e-4, 0.1,
        "zero-power gap g*; core §C.2", |p| p.design.suspension.g_star_m;
    "design.suspension.g_up_m", "m", 1.0e-5, 0.1,
        "upper stop gap; core §C.2, LEV-006", |p| p.design.suspension.g_up_m;
    "design.suspension.g_land_m", "m", 1.0e-4, 0.2,
        "landing (roller) gap; core §C.2, F18", |p| p.design.suspension.g_land_m;
    "design.suspension.g_seat_m", "m", 0.0, 0.1,
        "captive seat gap (DRAWER 3.5 mm); F17", |p| p.design.suspension.g_seat_m;
    "design.suspension.zp_band_m.0", "m", 1.0e-5, 0.2,
        "modified zero-power band, lower edge; F1", |p| p.design.suspension.zp_band_m[0];
    "design.suspension.zp_band_m.1", "m", 1.0e-5, 0.2,
        "modified zero-power band, upper edge; F1", |p| p.design.suspension.zp_band_m[1];
    "design.suspension.b_sat_t", "T", 0.5, 2.5,
        "iron saturation 1.5 T; core §C.1", |p| p.design.suspension.b_sat_t;
    "design.suspension.tau_eddy_s", "s", 0.0, 0.05,
        "plant eddy lag (= design point 1 ms); core §C.5, plan §2b item 1", |p| p.design.suspension.tau_eddy_s;
    "design.suspension.tempco_br_per_k", "1/K", -0.01, 0.0,
        "NdFeB −0.12 %/K; core §C.1, LEV-014", |p| p.design.suspension.tempco_br_per_k;
    "design.suspension.tempco_r_per_k", "1/K", 0.0, 0.01,
        "copper +0.393 %/K; core §C.1", |p| p.design.suspension.tempco_r_per_k;
    "design.suspension.leak", "1", 0.0, 0.9,
        "flux leakage σ (0 ideal; 0.25 studies); core §C.5", |p| p.design.suspension.leak;
    "design.suspension.stray_target_t", "T", 1.0e-6, 0.1,
        "≤ 0.5 mT at 10 cm; ENV-001", |p| p.design.suspension.stray_target_t;
    "design.suspension.shroud", "", 0.0, 1.0,
        "magnetic shroud (WALL/HEAVY); ENV-001", |p| f64::from(u8::from(p.design.suspension.shroud));
    "design.lsm.kind", "", 0.0, 3.0,
        "core §C.2 row LSM", |p| f64::from(p.design.lsm.kind as u8);
    "design.lsm.tau_p_m", "m", 0.001, 1.0,
        "pole pitch; core §C.2 row LSM", |p| p.design.lsm.tau_p_m;
    "design.lsm.k_f_n_per_a", "N/A", 0.01, 1.0e5,
        "thrust constant; design choice (F_peak/i_max), PHYSICS §7", |p| p.design.lsm.k_f_n_per_a;
    "design.lsm.k_e_v_s_per_m", "V·s/m", 0.005, 1.0e5,
        "K_e = ⅔·K_f (amplitude-invariant dq); PRP-002, PHYSICS §7", |p| p.design.lsm.k_e_v_s_per_m;
    "design.lsm.r_phase_ohm", "Ω", 1.0e-4, 1000.0,
        "phase resistance; design choice, PHYSICS §7", |p| p.design.lsm.r_phase_ohm;
    "design.lsm.l_phase_h", "H", 1.0e-6, 10.0,
        "phase inductance; design choice, PHYSICS §7", |p| p.design.lsm.l_phase_h;
    "design.lsm.f_peak_n", "N", 0.1, 1.0e8,
        "peak thrust; core §C.2, PRP-003", |p| p.design.lsm.f_peak_n;
    "design.lsm.f_cont_n", "N", 0.1, 1.0e8,
        "continuous thrust; core §C.2, PRP-003", |p| p.design.lsm.f_cont_n;
    "design.lsm.t_peak_s", "s", 0.01, 10.0,
        "peak window 0.75 s (EN 16005); core §C.2", |p| p.design.lsm.t_peak_s;
    "design.lsm.i_max_a", "A", 0.01, 1.0e5,
        "peak phase current (F_peak/K_f); design choice", |p| p.design.lsm.i_max_a;
    "design.lsm.ripple_frac", "1", 0.0, 0.2,
        "6th-order ripple 1 %; core §C.1", |p| p.design.lsm.ripple_frac;
    "design.lsm.cogging_n", "N", 0.0, 1.0e7,
        "iron-core cogging (HEAVY); core §C.1", |p| p.design.lsm.cogging_n;
    "design.lsm.f_normal_n", "N", 0.0, 1.0e8,
        "iron-core normal force (HEAVY); core §C.1", |p| p.design.lsm.f_normal_n;
    "design.lsm.segment_len_m", "m", 0.0, 1000.0,
        "stator segment (0 = continuous); core §C.1", |p| p.design.lsm.segment_len_m;
    "design.lsm.segment_overlap_m", "m", 0.0, 100.0,
        "segment hand-over overlap; core §C.1", |p| p.design.lsm.segment_overlap_m;
    "design.guidance.fin_present", "", 0.0, 1.0,
        "bottom guide fin (DOOR/WALL/HEAVY); core §C.1", |p| f64::from(u8::from(p.design.guidance.fin_present));
    "design.guidance.fin_clearance_m", "m", 0.0, 0.05,
        "fin clearance ±2 mm; core §C.1", |p| p.design.guidance.fin_clearance_m;
    "design.guidance.pad_mu", "1", 0.0, 1.0,
        "UHMW μ 0.15; core §C.5", |p| p.design.guidance.pad_mu;
    "design.guidance.pad_clearance_m", "m", 0.0, 0.05,
        "side-pad clearance; design choice", |p| p.design.guidance.pad_clearance_m;
    "design.guidance.c_sway_n_s_per_m", "N·s/m", 0.0, 1.0e8,
        "sway damping (ζ ≥ 0.05); plan §2b item 16, LEV-011", |p| p.design.guidance.c_sway_n_s_per_m;
    "design.damping.b_dist_n_s_per_m", "N·s/m", 0.01, 1.0e8,
        "distributed damping 2/5/20/100; core §C.2", |p| p.design.damping.b_dist_n_s_per_m;
    "design.damping.ez_len_m", "m", 0.0, 10.0,
        "end-zone length 150 mm; SAF-014", |p| p.design.damping.ez_len_m;
    "design.damping.ez_cmax_n_s_per_m", "N·s/m", 0.0, 1.0e9,
        "end-zone c_max; core §C.2", |p| p.design.damping.ez_cmax_n_s_per_m;
    "design.damping.bumper_k_n_per_m", "N/m", 1.0, 1.0e10,
        "bumper stiffness (rating in 20–100 mm); design choice", |p| p.design.damping.bumper_k_n_per_m;
    "design.damping.bumper_hc_s_per_m", "s/m", 0.0, 10.0,
        "Hunt–Crossley α 0.3 s/m; Hunt & Crossley 1975", |p| p.design.damping.bumper_hc_s_per_m;
    "design.damping.bumper_e_rated_j", "J", 0.0, 1.0e10,
        "≥ ½·m·v² (24/113/563/5625 J); SAF-014, F13", |p| p.design.damping.bumper_e_rated_j;
    "design.damping.k_edge_n_per_m", "N/m", 1.0, 1.0e6,
        "closing-edge seal ≤ 10 N/mm; F6", |p| p.design.damping.k_edge_n_per_m;
    "design.damping.c_rail_n_s_per_m", "N·s/m", 0.0, 1.0e8,
        "rail eddy drag [UNCERTAIN]; core §C.5", |p| p.design.damping.c_rail_n_s_per_m;
    "design.damping.f_hyst_n", "N", 0.0, 1.0e7,
        "rail hysteresis drag [UNCERTAIN]; core §C.5", |p| p.design.damping.f_hyst_n;
    "design.landing.kind", "", 0.0, 2.0,
        "seat (DRAWER) or rollers; F15, F17", |p| f64::from(p.design.landing.kind as u8);
    "design.landing.rollers_mu", "1", 0.0, 0.1,
        "roller μ 0.003 everywhere; plan §2b item 19", |p| p.design.landing.rollers_mu;
    "design.landing.upper_roller_rating_n", "N", 0.0, 1.0e9,
        "≥ n·F_pm(g_up) (DRAWER ≥ 550 N); F2, LEV-006", |p| p.design.landing.upper_roller_rating_n;
    "design.landing.seat_z_clearance_m", "m", 0.0, 0.01,
        "seat z clearance ≈ 0.15 mm; plan §2b item 4", |p| p.design.landing.seat_z_clearance_m;
    "design.landing.seat_x_compliance_m", "m", 0.0, 0.05,
        "push-stop travel ≥ 2 mm; plan §2b item 4", |p| p.design.landing.seat_x_compliance_m;
    "design.landing.seat_k_x_n_per_m", "N/m", 0.0, 1.0e8,
        "push-stop spring; design choice", |p| p.design.landing.seat_k_x_n_per_m;
    "design.landing.slot_len_m", "m", 0.0, 1.0,
        "seat slot length; plan §2b item 4", |p| p.design.landing.slot_len_m;
    "design.landing.ramp_len_m", "m", 0.0, 1.0,
        "seat entry ramp length; plan §2b item 4", |p| p.design.landing.ramp_len_m;
    "design.landing.ramp_rise_m", "m", 0.0, 0.05,
        "seat entry ramp rise (6.5 → 3.5 mm); plan §2b item 4", |p| p.design.landing.ramp_rise_m;
    "design.landing.seismic_latch.enabled", "", 0.0, 1.0,
        "x-retention latch fitted; plan §4b", |p| f64::from(u8::from(p.design.landing.seismic_latch.enabled));
    "design.landing.seismic_latch.x_retention_n", "N", 0.0, 1.0e9,
        "≥ m_rated·a_design for SeatAndLatch; plan §4b", |p| p.design.landing.seismic_latch.x_retention_n;
    "design.landing.parking_lock", "", 0.0, 1.0,
        "parking lock (WALL/HEAVY); PWR-001", |p| f64::from(u8::from(p.design.landing.parking_lock));
    "design.rail.x_closed_m", "m", -10000.0, 10000.0,
        "closed-stop position; PHYSICS §1", |p| p.design.rail.x_closed_m;
    "design.rail.travel_m", "m", 0.001, 10000.0,
        "preset travel (DRAWER 0.30 m); core §C.2, plan §2b item 3", |p| p.design.rail.travel_m;
    "design.rail.span_m", "m", 0.01, 1000.0,
        "support span for sag; PHYSICS §10", |p| p.design.rail.span_m;
    "design.rail.deflection_ratio", "1", 100.0, 1.0e6,
        "sag L/1150; core §C.1", |p| p.design.rail.deflection_ratio;
    "design.rail.joint_spacing_m", "m", 0.0, 1000.0,
        "joint spacing (0 = no joints); PHYSICS §10", |p| p.design.rail.joint_spacing_m;
    "design.rail.joint_step_m", "m", 0.0, 0.005,
        "joint step 0.1–0.3 mm; core §C.1", |p| p.design.rail.joint_step_m;
    "design.rail.lateral_step_m", "m", 0.0, 0.005,
        "lateral joint step; core §C.1", |p| p.design.rail.lateral_step_m;
    "design.rail.roughness_rms_m", "m", 0.0, 0.001,
        "roughness 20 µm rms; core §C.1", |p| p.design.rail.roughness_rms_m;
    "design.rail.flange_t_m", "m", 1.0e-4, 0.5,
        "flange ≥ B_g·w/1.5 T; LEV-015", |p| p.design.rail.flange_t_m;
    "design.rail.b_sat_rail_t", "T", 0.5, 2.5,
        "rail steel 1.5 T; LEV-015", |p| p.design.rail.b_sat_rail_t;
    "design.rail.laminated", "", 0.0, 1.0,
        "laminated/SMC rail assumed by the gain design; plan §2b item 1", |p| f64::from(u8::from(p.design.rail.laminated));
    "design.air.cavity_area_m2", "m²", 0.0, 100.0,
        "DRAWER piston area 0.06 m²; core §C.1", |p| p.design.air.cavity_area_m2;
    "design.air.cavity_volume_m3", "m³", 0.0, 1000.0,
        "DRAWER cavity 0.03 m³; core §C.1", |p| p.design.air.cavity_volume_m3;
    "design.air.leak_area_m2", "m²", 0.0, 10.0,
        "40 cm² reveal; core §C.1, ENV-003", |p| p.design.air.leak_area_m2;
    "design.air.leaf_area_m2", "m²", 0.0, 1000.0,
        "smoke-ΔP leaf area; core §C.1", |p| p.design.air.leaf_area_m2;
    "design.thermal.r_th_coil_k_per_w", "K/W", 0.001, 10000.0,
        "coil RC node; design choice, PHYSICS §12", |p| p.design.thermal.r_th_coil_k_per_w;
    "design.thermal.c_th_coil_j_per_k", "J/K", 0.1, 1.0e8,
        "coil RC node; design choice, PHYSICS §12", |p| p.design.thermal.c_th_coil_j_per_k;
    "design.thermal.r_th_mag_k_per_w", "K/W", 0.001, 10000.0,
        "magnet RC node; design choice, PHYSICS §12", |p| p.design.thermal.r_th_mag_k_per_w;
    "design.thermal.c_th_mag_j_per_k", "J/K", 0.1, 1.0e8,
        "magnet RC node; design choice, PHYSICS §12", |p| p.design.thermal.c_th_mag_j_per_k;
    "design.thermal.t_ambient_c", "°C", -40.0, 60.0,
        "reference ambient 20 °C; PHYSICS §12", |p| p.design.thermal.t_ambient_c;
    "design.thermal.t_coil_max_c", "°C", 40.0, 250.0,
        "class B 130 °C − 10 K; design choice", |p| p.design.thermal.t_coil_max_c;
    "design.thermal.t_mag_max_c", "°C", 40.0, 250.0,
        "SH-grade 80 °C proxy; ENV-004", |p| p.design.thermal.t_mag_max_c;
    "design.sensors.gap_poly_m.0", "m", -0.1, 0.1,
        "gap linearisation c0; core §C.5", |p| p.design.sensors.gap_poly_m[0];
    "design.sensors.gap_poly_m.1", "m/code", 0.0, 0.001,
        "16-bit over 20 mm; core §C.5", |p| p.design.sensors.gap_poly_m[1];
    "design.sensors.gap_poly_m.2", "m/code²", -1.0e-9, 1.0e-9,
        "gap linearisation c2; core §C.5", |p| p.design.sensors.gap_poly_m[2];
    "design.sensors.gap_noise_m", "m", 0.0, 0.001,
        "in-band gap noise 0.3/0.5/2/5 µm; core §C.2", |p| p.design.sensors.gap_noise_m;
    "design.sensors.gap_redundant", "", 0.0, 1.0,
        "dual gap probes; core §E.2 dual_gap_vote", |p| f64::from(u8::from(p.design.sensors.gap_redundant));
    "design.sensors.coil_a_per_code", "A/code", 1.0e-6, 10.0,
        "12-bit signed current ADC; core §C.5", |p| p.design.sensors.coil_a_per_code;
    "design.sensors.coil_noise_a", "A", 0.0, 10.0,
        "current noise; design choice, PHYSICS §13", |p| p.design.sensors.coil_noise_a;
    "design.sensors.lsm_a_per_code", "A/code", 1.0e-6, 10.0,
        "LSM phase ADC ±1.5·i_max; design choice", |p| p.design.sensors.lsm_a_per_code;
    "design.sensors.nm_per_count", "nm/count", 1.0, 1.0e6,
        "1 µm/count encoder; core §C.5", |p| f64::from(p.design.sensors.nm_per_count);
    "design.sensors.accel_mps2_per_lsb", "m/s²/LSB", 1.0e-9, 1.0,
        "ADXL355 3.9 µg/LSB; ADI datasheet", |p| p.design.sensors.accel_mps2_per_lsb;
    "design.sensors.accel_noise_mps2_per_rthz", "m/s²/√Hz", 0.0, 1.0,
        "ADXL355 22.5 µg/√Hz; ADI datasheet, core §C.5", |p| p.design.sensors.accel_noise_mps2_per_rthz;
    "design.sensors.accel_odr_hz", "Hz", 1.0, 1.0e5,
        "ADXL355 ODR 500 Hz; core §C.2", |p| p.design.sensors.accel_odr_hz;
    "design.sensors.accel_mount", "", 0.0, 2.0,
        "structure/rail, never carriage; SEI-001, F16", |p| f64::from(p.design.sensors.accel_mount as u8);
    "design.sensors.imu_present", "", 0.0, 1.0,
        "carriage IMU (skyhook/fusion only); F16", |p| f64::from(u8::from(p.design.sensors.imu_present));
    "design.sensors.imu_mps2_per_lsb", "m/s²/LSB", 1.0e-9, 1.0,
        "±8 g 16-bit; design choice", |p| p.design.sensors.imu_mps2_per_lsb;
    "design.sensors.imu_noise_mps2", "m/s²", 0.0, 10.0,
        "≈ 1 mg rms; plan §4b", |p| p.design.sensors.imu_noise_mps2;
    "design.sensors.rail_accel_present", "", 0.0, 1.0,
        "rail floor accelerometer; plan §4b", |p| f64::from(u8::from(p.design.sensors.rail_accel_present));
    "design.sensors.rail_accel_mps2_per_lsb", "m/s²/LSB", 1.0e-9, 1.0,
        "±8 g 16-bit; design choice", |p| p.design.sensors.rail_accel_mps2_per_lsb;
    "design.sensors.rail_accel_noise_mps2", "m/s²", 0.0, 10.0,
        "≈ 1 mg rms; plan §4b", |p| p.design.sensors.rail_accel_noise_mps2;
    "design.sensors.v_bus_v_per_code", "V/code", 1.0e-6, 10.0,
        "12-bit bus sense; design choice", |p| p.design.sensors.v_bus_v_per_code;
    "design.sensors.v_hold_v_per_code", "V/code", 1.0e-6, 10.0,
        "12-bit hold-up sense; design choice", |p| p.design.sensors.v_hold_v_per_code;
    "design.sensors.i_bus_a_per_code", "A/code", 1.0e-6, 100.0,
        "bus shunt 10 mA/code; design choice", |p| p.design.sensors.i_bus_a_per_code;
    "design.sensors.temp_c_per_code", "°C/code", 1.0e-4, 10.0,
        "12-bit temperature; design choice", |p| p.design.sensors.temp_c_per_code;
    "design.sensors.temp_offset_c", "°C", -300.0, 300.0,
        "temperature offset; design choice", |p| p.design.sensors.temp_offset_c;
    "design.sensors.handle_n_per_code", "N/code", 0.0, 100.0,
        "handle load cell (0 = not fitted); HAP-002", |p| p.design.sensors.handle_n_per_code;
    "design.sensors.handle_noise_n", "N", 0.0, 100.0,
        "load-cell σ 0.05 N; core §C.5", |p| p.design.sensors.handle_noise_n;
    "design.sensors.latency_ticks", "tick", 0.0, 100.0,
        "1-tick sensor latency; core §C.1", |p| f64::from(p.design.sensors.latency_ticks);
    "design.sensors.link_latency_s", "s", 0.0, 1.0,
        "CAN 3 ms; core §C.5", |p| p.design.sensors.link_latency_s;
    "design.power.v_bus_v", "V", 1.0, 10000.0,
        "24/48/48/300 V; core §C.2, PWR-005", |p| p.design.power.v_bus_v;
    "design.power.p_bus_max_w", "W", 0.1, 1.0e8,
        "bus power limit; design choice, PHYSICS §14", |p| p.design.power.p_bus_max_w;
    "design.power.v_chopper_v", "V", 1.0, 10000.0,
        "1.15·V_bus (≤ +16 %); PRP-010", |p| p.design.power.v_chopper_v;
    "design.power.pwm_hz", "Hz", 1000.0, 1.0e6,
        "≥ 20 kHz; ENV-002", |p| p.design.power.pwm_hz;
    "design.power.supercap_c_f", "F", 0.0, 1.0e6,
        "hold-up supercap 10 F; core §C.1", |p| p.design.power.supercap_c_f;
    "design.power.supercap_v_max_v", "V", 0.0, 10000.0,
        "supercap 12 V (HEAVY 48 V); core §C.1, SAF-009", |p| p.design.power.supercap_v_max_v;
    "design.power.supercap_v_min_v", "V", 0.0, 10000.0,
        "supercap usable floor; design choice", |p| p.design.power.supercap_v_min_v;
    "design.power.supercap_esr_ohm", "Ω", 0.0, 10.0,
        "ESR 10 mΩ; core §C.1", |p| p.design.power.supercap_esr_ohm;
    "design.power.converter_eff", "1", 0.01, 1.0,
        "η 0.9; core §C.1", |p| p.design.power.converter_eff;
    "design.power.p_electronics_w", "W", 0.0, 1.0e6,
        "standby electronics; design choice, PWR-002", |p| p.design.power.p_electronics_w;
    "design.power.p_hover_budget_w", "W", 0.0, 1.0e7,
        "hover budget 1.5/3/30/100 W; PWR-002", |p| p.design.power.p_hover_budget_w;
    "design.power.p_sleep_budget_w", "W", 0.0, 1.0e6,
        "sleep budget 0.3/0.5 W; core §C.2", |p| p.design.power.p_sleep_budget_w;
    "design.power.ups", "", 0.0, 1.0,
        "building UPS on mains; plan §4b", |p| f64::from(u8::from(p.design.power.ups));
    "control.rates.base_hz", "Hz", 1000.0, 1.0e6,
        "20 kHz base tick; core §A.5", |p| f64::from(p.control.rates.base_hz);
    "control.rates.fast_decim", "", 1.0, 1000.0,
        "fast 10/5 kHz; core §A.5", |p| f64::from(p.control.rates.fast_decim);
    "control.rates.sup_decim", "", 1.0, 10000.0,
        "supervisor 1 kHz; core §A.5", |p| f64::from(p.control.rates.sup_decim);
    "control.rates.seismic_decim", "", 1.0, 65535.0,
        "seismic 100 Hz; core §A.5", |p| f64::from(p.control.rates.seismic_decim);
    "control.rates.coil_mode_voltage", "", 0.0, 1.0,
        "voltage coil mode (SIL ≡ firmware); plan §2b item 16", |p| f64::from(u8::from(p.control.rates.coil_mode_voltage));
    "control.levitation.omega_ratio", "1", 1.0, 10.0,
        "ω = 2.5·p; plan §2b item 1", |p| p.control.levitation.omega_ratio;
    "control.levitation.zeta", "1", 0.3, 2.0,
        "ζ 0.7; core §C.4", |p| p.control.levitation.zeta;
    "control.levitation.alpha", "1", 0.01, 2.0,
        "α 0.3; core §C.4", |p| p.control.levitation.alpha;
    "control.levitation.observer_ratio", "1", 1.0, 20.0,
        "observer at 4ω; core §C.4", |p| p.control.levitation.observer_ratio;
    "control.levitation.tau_e_nominal_s", "s", 0.0, 0.02,
        "design eddy lag 1 ms (robust 0–2 ms); plan §2b item 1, LEV-020", |p| p.control.levitation.tau_e_nominal_s;
    "control.levitation.current_bw_hz", "Hz", 10.0, 1.0e5,
        "ω_c = 2π·1 kHz; plan §2b item 2", |p| p.control.levitation.current_bw_hz;
    "control.levitation.roll_active", "", 0.0, 1.0,
        "roll active on DRAWER/HEAVY; LEV-001", |p| f64::from(u8::from(p.control.levitation.roll_active));
    "control.levitation.warp_bw_hz", "Hz", 0.01, 100.0,
        "warp integrator; LEV-009, design choice", |p| p.control.levitation.warp_bw_hz;
    "control.levitation.zp_outer_bw_hz", "Hz", 0.01, 100.0,
        "modified-ZP outer PI 1 Hz; core §C.4", |p| p.control.levitation.zp_outer_bw_hz;
    "control.levitation.bumpless_dg_m", "m", 0.0, 0.001,
        "bumpless transfer < 50 µm; core §C.4", |p| p.control.levitation.bumpless_dg_m;
    "control.levitation.lift_time_max_s", "s", 0.01, 60.0,
        "lift ≤ 0.1/0.3/1/2 s; LEV-012", |p| p.control.levitation.lift_time_max_s;
    "control.levitation.land_speed_mps", "m/s", 1.0e-4, 0.1,
        "land 5/5/3/2 mm/s; core §C.4", |p| p.control.levitation.land_speed_mps;
    "control.levitation.unseat_liftoff_m", "m", 0.0, 0.001,
        "lift-off > 20 µm; F17", |p| p.control.levitation.unseat_liftoff_m;
    "control.levitation.unseat_ramp_a_per_s", "A/s", 0.0, 1.0e5,
        "unseat current ramp; F17, design choice", |p| p.control.levitation.unseat_ramp_a_per_s;
    "control.levitation.seat_time_s", "s", 0.0, 5.0,
        "+0.2 s z-seat step; plan §4b", |p| p.control.levitation.seat_time_s;
    "control.levitation.touchdown_gdot_mps", "m/s", 0.0, 0.1,
        "touchdown ġ ≈ 0; core §C.4", |p| p.control.levitation.touchdown_gdot_mps;
    "control.levitation.m_hat_tau_s", "s", 0.001, 10.0,
        "m̂ within ±5 % in ≤ 1 s; LEV-013", |p| p.control.levitation.m_hat_tau_s;
    "control.levitation.sat_persist_s", "s", 0.0, 10.0,
        "saturation persistence; design choice", |p| p.control.levitation.sat_persist_s;
    "control.motion.omega_v_hz", "Hz", 0.1, 100.0,
        "ω_v/2π 8/6/3/2 Hz; core §C.4", |p| p.control.motion.omega_v_hz;
    "control.motion.kx_ratio", "1", 0.0, 2.0,
        "K_x = ω_v/4; core §C.4", |p| p.control.motion.kx_ratio;
    "control.motion.ki_ratio", "1", 0.0, 2.0,
        "K_vi = K_v·ω_v/4; core §C.4", |p| p.control.motion.ki_ratio;
    "control.motion.a_max_mps2", "m/s²", 0.001, 20.0,
        "a_max 1.5/0.75/0.15/0.08; core §C.2, F5", |p| p.control.motion.a_max_mps2;
    "control.motion.a_max_sensed_mps2", "m/s²", 0.001, 20.0,
        "HEAVY 0.2 with PL d; core §C.2", |p| p.control.motion.a_max_sensed_mps2;
    "control.motion.j_max_mps3", "m/s³", 0.001, 1000.0,
        "jerk 20/5/1.5/1; core §C.2", |p| p.control.motion.j_max_mps3;
    "control.motion.a_brk_ratio", "1", 1.0, 10.0,
        "a_brk = 2·a_max; core §C.4", |p| p.control.motion.a_brk_ratio;
    "control.motion.t_react_fast_periods", "", 0.0, 1000.0,
        "t_react = 2 fast periods; core §C.4", |p| f64::from(p.control.motion.t_react_fast_periods);
    "control.motion.gov_margin_m", "m", 0.0, 1.0,
        "governor margin; design choice", |p| p.control.motion.gov_margin_m;
    "control.motion.pos_tol_closed_m", "m", 0.0, 0.1,
        "±0.3/0.5/1/1 mm; core §C.2, PRP-006", |p| p.control.motion.pos_tol_closed_m;
    "control.motion.pos_tol_travel_m", "m", 0.0, 0.1,
        "±1/2/5/5 mm; core §C.2, PRP-006", |p| p.control.motion.pos_tol_travel_m;
    "control.motion.ripple_ff", "", 0.0, 1.0,
        "ripple LUT feed-forward; core §C.4", |p| f64::from(u8::from(p.control.motion.ripple_ff));
    "control.motion.terminus.tau_s", "s", 0.01, 10.0,
        "τ 0.15/0.25/0.6/1.0 s; plan §4b, PHYSICS §16.1", |p| p.control.motion.terminus.tau_s;
    "control.motion.terminus.d_eps_m", "m", 1.0e-6, 0.01,
        "d_ε ≈ 0.2 mm; plan §4b", |p| p.control.motion.terminus.d_eps_m;
    "control.motion.terminus.j_mps3", "m/s³", 0.001, 1000.0,
        "blend jerk = j_max; plan §4b table", |p| p.control.motion.terminus.j_mps3;
    "control.motion.terminus.t_q_ratio", "1", 0.1, 10.0,
        "T_q = τ; plan §4b", |p| p.control.motion.terminus.t_q_ratio;
    "control.motion.terminus.overtravel_m", "m", 0.0, 1.0,
        "x_obs = terminus + overtravel; plan §4b", |p| p.control.motion.terminus.overtravel_m;
    "control.motion.terminus.replan_brk_ratio", "1", 1.0, 10.0,
        "re-plan a_brk = 2·a_max; plan §4b", |p| p.control.motion.terminus.replan_brk_ratio;
    "control.motion.inertial.enable", "", 0.0, 1.0,
        "research tier, off in every preset; plan §4b", |p| f64::from(u8::from(p.control.motion.inertial.enable));
    "control.motion.inertial.f_c_hz", "Hz", 0.01, 20.0,
        "crossover ≈ 1 Hz; plan §4b, PHYSICS §16.2", |p| p.control.motion.inertial.f_c_hz;
    "control.motion.inertial.zeta", "1", 0.05, 5.0,
        "skyhook ζ; plan §4b", |p| p.control.motion.inertial.zeta;
    "control.motion.inertial.hp_hz", "Hz", 0.001, 10.0,
        "leaky integral 0.1 Hz; plan §4b", |p| p.control.motion.inertial.hp_hz;
    "control.motion.inertial.standoff_m", "m", 0.0, 1.0,
        "ride-through standoff; plan §4b", |p| p.control.motion.inertial.standoff_m;
    "control.motion.inertial.s_min_base_m", "m", 0.0, 1.0,
        "s_min = 25 mm + S_max; plan §4b", |p| p.control.motion.inertial.s_min_base_m;
    "control.motion.inertial.stroke_margin_m", "m", 0.0, 1.0,
        "stroke margin; design choice", |p| p.control.motion.inertial.stroke_margin_m;
    "control.motion.inertial.v_rel_frac", "1", 0.0, 1.0,
        "v_rel ≤ 0.8·v_cap; plan §4b", |p| p.control.motion.inertial.v_rel_frac;
    "control.motion.inertial.f_burst_n", "N", 0.0, 1.0e8,
        "bursts ≤ F_peak (DRAWER 90 N); plan §4b", |p| p.control.motion.inertial.f_burst_n;
    "control.motion.inertial.t_burst_s", "s", 0.0, 10.0,
        "bursts ≤ 0.75 s; plan §4b", |p| p.control.motion.inertial.t_burst_s;
    "control.motion.inertial.z_headroom_frac", "1", 0.0, 1.0,
        "a_z ≤ 0.5·p²·(g_float − g_up); plan §4b", |p| p.control.motion.inertial.z_headroom_frac;
    "control.motion.inertial.structure_lp_hz", "Hz", 0.001, 10.0,
        "ADXL355 only below 0.3 Hz; plan §4b", |p| p.control.motion.inertial.structure_lp_hz;
    "control.motion.inertial.latency_s", "s", 0.0, 0.1,
        "sense-to-act latency L; design choice", |p| p.control.motion.inertial.latency_s;
    "control.haptic.mass_ratio", "1", 1.0, 20.0,
        "m/M_v 1.5/3/5/5 (≤ 2/5/20); HAP-002", |p| p.control.haptic.mass_ratio;
    "control.haptic.b_v_n_s_per_m", "N·s/m", 0.0, 1.0e8,
        "B_v 10/40/450/2000; core §C.2", |p| p.control.haptic.b_v_n_s_per_m;
    "control.haptic.k_o_hz", "Hz", 0.1, 1000.0,
        "momentum observer 20/15/8/5 Hz; core §C.4", |p| p.control.haptic.k_o_hz;
    "control.haptic.k_virtual_frac", "1", 0.0, 1.0,
        "K_v/(2b/T) ≤ 1/3 with T_d = T; plan §2b item 19, HAP-009", |p| p.control.haptic.k_virtual_frac;
    "control.haptic.passivity_alpha_ratio", "1", 0.0, 100.0,
        "α_max = 5·b_dist; core §C.4", |p| p.control.haptic.passivity_alpha_ratio;
    "control.haptic.f_hp_hz", "Hz", 0.0, 100.0,
        "F̂ high-pass 0.5 Hz; core §C.4", |p| p.control.haptic.f_hp_hz;
    "control.haptic.n_detents", "", 0.0, 4.0,
        "active detents (≤ 4); core §C.4", |p| f64::from(p.control.haptic.n_detents);
    "control.haptic.detent_x_m.0", "m", -10000.0, 10000.0,
        "detent position from the closed stop; design choice", |p| p.control.haptic.detent_x_m[0];
    "control.haptic.detent_x_m.1", "m", -10000.0, 10000.0,
        "detent position from the closed stop; design choice", |p| p.control.haptic.detent_x_m[1];
    "control.haptic.detent_x_m.2", "m", -10000.0, 10000.0,
        "detent position from the closed stop; design choice", |p| p.control.haptic.detent_x_m[2];
    "control.haptic.detent_x_m.3", "m", -10000.0, 10000.0,
        "detent position from the closed stop; design choice", |p| p.control.haptic.detent_x_m[3];
    "control.haptic.detent_f_n", "N", 0.0, 1.0e5,
        "detent peak 3/8/20 N; core §C.4", |p| p.control.haptic.detent_f_n;
    "control.haptic.detent_width_m", "m", 1.0e-4, 1.0,
        "detent width 2/3/5 mm; core §C.4", |p| p.control.haptic.detent_width_m;
    "control.haptic.soft_close_omega_hz", "Hz", 0.01, 100.0,
        "ω_sc/2π 2/1.5/0.5/0.5 Hz; core §C.4", |p| p.control.haptic.soft_close_omega_hz;
    "control.haptic.soft_close_capture_m", "m", 0.0, 10.0,
        "capture 45/65/115/150 mm; core §C.2", |p| p.control.haptic.soft_close_capture_m;
    "control.haptic.soft_close_v_final_mps", "m/s", 0.0, 1.0,
        "final ≤ 20/50/20/10 mm/s; HAP-007", |p| p.control.haptic.soft_close_v_final_mps;
    "control.haptic.hand_onset_s", "s", 0.0, 1.0,
        "hand onset 10 ms (20 ms WALL); plan §4b", |p| p.control.haptic.hand_onset_s;
    "control.haptic.obstruction.residual_n", "N", 0.1, 1.0e7,
        "residual 12/30/60/120 N; core §C.2", |p| p.control.haptic.obstruction.residual_n;
    "control.haptic.obstruction.persist_s", "s", 0.0, 1.0,
        "persistence 20/30/35/35 ms; core §C.2", |p| p.control.haptic.obstruction.persist_s;
    "control.haptic.obstruction.reverse_m", "m", 0.0, 1.0,
        "reverse 50 mm (DOOR/WALL); SAF-006", |p| p.control.haptic.obstruction.reverse_m;
    "control.haptic.obstruction.release_to_zero", "", 0.0, 1.0,
        "release to 0 N (DRAWER/HEAVY); core §C.4", |p| f64::from(u8::from(p.control.haptic.obstruction.release_to_zero));
    "control.haptic.obstruction.floor_gain", "1", 0.0, 10.0,
        "th = th0 + 0.1·m̂·|a_floor|; plan §4b", |p| p.control.haptic.obstruction.floor_gain;
    "control.haptic.obstruction.degraded_ratio", "1", 1.0, 100.0,
        "> 2·th0 → ObstructionDegraded; plan §4b", |p| p.control.haptic.obstruction.degraded_ratio;
    "control.intent.f_th_n", "N", 0.1, 1.0e6,
        "intent force 4.5/10/25/40 N; core §C.2", |p| p.control.intent.f_th_n;
    "control.intent.t_th_s", "s", 0.0, 5.0,
        "intent time 50/80/120/150 ms; core §C.2", |p| p.control.intent.t_th_s;
    "control.intent.dx_th_m", "m", 0.0, 0.1,
        "intent displacement 1.5/2/3 mm; core §C.2", |p| p.control.intent.dx_th_m;
    "control.intent.lean_v_mps", "m/s", 0.0, 0.1,
        "lean |v| < 2 mm/s; core §C.4", |p| p.control.intent.lean_v_mps;
    "control.intent.lean_t_s", "s", 0.0, 60.0,
        "lean > 1 s; core §C.4", |p| p.control.intent.lean_t_s;
    "control.intent.confirm_max_s", "s", 0.01, 5.0,
        "confirm ≤ 120/170 ms; F12, HAP-005", |p| p.control.intent.confirm_max_s;
    "control.intent.dir_consistency", "", 0.0, 1.0,
        "direction consistency; plan §2b item 11", |p| f64::from(u8::from(p.control.intent.dir_consistency));
    "control.intent.push_to_open", "", 0.0, 1.0,
        "push-to-open from the seat (DRAWER); HAP-012", |p| f64::from(u8::from(p.control.intent.push_to_open));
    "control.seismic.detector.aa_decim", "", 1.0, 1000.0,
        "500 → 100 Hz; core §C.4", |p| f64::from(p.control.seismic.detector.aa_decim);
    "control.seismic.detector.bp_lo_hz", "Hz", 0.01, 100.0,
        "band-pass 0.5 Hz; core §C.4", |p| p.control.seismic.detector.bp_lo_hz;
    "control.seismic.detector.bp_hi_hz", "Hz", 0.1, 1000.0,
        "band-pass 20 Hz; core §C.4", |p| p.control.seismic.detector.bp_hi_hz;
    "control.seismic.detector.bp_order", "", 1.0, 8.0,
        "4-pole Butterworth; core §C.4", |p| f64::from(p.control.seismic.detector.bp_order);
    "control.seismic.detector.sta_n", "sample", 1.0, 1.0e6,
        "STA 50; core §C.4", |p| f64::from(p.control.seismic.detector.sta_n);
    "control.seismic.detector.lta_n", "sample", 1.0, 1.0e7,
        "LTA 3000; core §C.4", |p| f64::from(p.control.seismic.detector.lta_n);
    "control.seismic.detector.trig_on", "1", 1.0, 100.0,
        "trigger on 4.0; core §C.4", |p| p.control.seismic.detector.trig_on;
    "control.seismic.detector.trig_off", "1", 1.0, 100.0,
        "trigger off 1.5; core §C.4", |p| p.control.seismic.detector.trig_off;
    "control.seismic.detector.pdtc_hp_hz", "Hz", 0.001, 10.0,
        "Pd/τc high-pass 0.075 Hz; core §C.4", |p| p.control.seismic.detector.pdtc_hp_hz;
    "control.seismic.detector.win_short_s", "s", 0.1, 60.0,
        "window 1 s; core §C.4", |p| p.control.seismic.detector.win_short_s;
    "control.seismic.detector.win_long_s", "s", 0.1, 60.0,
        "window 3 s; core §C.4", |p| p.control.seismic.detector.win_long_s;
    "control.seismic.detector.taup_alpha", "1", 0.0, 1.0,
        "τp_max α 0.99 [M]; core §C.4", |p| p.control.seismic.detector.taup_alpha;
    "control.seismic.detector.zh_window_s", "s", 0.05, 10.0,
        "Z/H first 0.5 s; core §C.4", |p| p.control.seismic.detector.zh_window_s;
    "control.seismic.detector.zh_min", "1", 0.0, 100.0,
        "Z/H > 1; core §C.4", |p| p.control.seismic.detector.zh_min;
    "control.seismic.detector.pga_veto_mps2", "m/s²", 0.0, 100.0,
        "actuator veto unless PGA ≥ 80 gal; core §C.4", |p| p.control.seismic.detector.pga_veto_mps2;
    "control.seismic.detector.tauc_l3_s", "s", 0.0, 100.0,
        "L3 τc ≥ 0.6 s; core §C.4", |p| p.control.seismic.detector.tauc_l3_s;
    "control.seismic.detector.pd_l3_m", "m", 0.0, 1.0,
        "L3 Pd ≥ 0.5 cm; core §C.4", |p| p.control.seismic.detector.pd_l3_m;
    "control.seismic.detector.pd_alert_m", "m", 0.0, 1.0,
        "PROTECT on Pd ≥ 0.35 cm; plan §2b item 6", |p| p.control.seismic.detector.pd_alert_m;
    "control.seismic.detector.pga_alert_mps2", "m/s²", 0.0, 100.0,
        "PROTECT on PGA ≥ 80 gal; plan §2b item 6", |p| p.control.seismic.detector.pga_alert_mps2;
    "control.seismic.detector.protect_level", "", 1.0, 3.0,
        "PROTECT on level ≥ 2; plan §2b item 6", |p| f64::from(p.control.seismic.detector.protect_level);
    "control.seismic.fusion.v_p_kmps", "km/s", 1.0, 20.0,
        "V_p 6.0 km/s; core §C.4", |p| p.control.seismic.fusion.v_p_kmps;
    "control.seismic.fusion.v_s_kmps", "km/s", 0.5, 10.0,
        "V_s 3.5 km/s; core §C.4", |p| p.control.seismic.fusion.v_s_kmps;
    "control.seismic.fusion.quiet_s", "s", 0.0, 3600.0,
        "QUIET 30 s; core §C.4", |p| p.control.seismic.fusion.quiet_s;
    "control.seismic.fusion.eta_sigma_s", "s", 0.0, 60.0,
        "S-ETA σ 1–2 s; design choice", |p| p.control.seismic.fusion.eta_sigma_s;
    "control.seismic.fusion.alert_timeout_s", "s", 1.0, 3600.0,
        "alert expiry; design choice", |p| p.control.seismic.fusion.alert_timeout_s;
    "control.seismic.fusion.min_independent_feeds", "", 1.0, 16.0,
        "≥ 2 feeds without local P; plan §4", |p| f64::from(p.control.seismic.fusion.min_independent_feeds);
    "control.seismic.policy.action", "", 0.0, 10.0,
        "class action (land-first); core §C.2, SEI-010", |p| f64::from(p.control.seismic.policy.action.as_u8());
    "control.seismic.policy.center_if_reachable", "", 0.0, 1.0,
        "centre only if reachable; F11", |p| f64::from(u8::from(p.control.seismic.policy.center_if_reachable));
    "control.seismic.policy.lock_after_land", "", 0.0, 1.0,
        "never on egress; SEI-011", |p| f64::from(u8::from(p.control.seismic.policy.lock_after_land));
    "control.seismic.policy.egress_stop_margin_s", "s", 0.0, 60.0,
        "stopped by t_S − 1 s; SEI-011", |p| p.control.seismic.policy.egress_stop_margin_s;
    "control.seismic.policy.asc_on_seismic_land", "", 0.0, 1.0,
        "ASC during seismic landing; F9", |p| f64::from(u8::from(p.control.seismic.policy.asc_on_seismic_land));
    "control.seismic.isolation.enable", "", 0.0, 1.0,
        "research tier, disabled; plan §2b item 5", |p| f64::from(u8::from(p.control.seismic.isolation.enable));
    "control.seismic.isolation.t_iso_s", "s", 0.1, 100.0,
        "T = max(2 s, 1.1·T_min(F_cont)); core §C.2, plan §2b item 5", |p| p.control.seismic.isolation.t_iso_s;
    "control.seismic.isolation.zeta", "1", 0.0, 5.0,
        "skyhook ζ 0.3; core §C.2", |p| p.control.seismic.isolation.zeta;
    "control.seismic.isolation.stroke_m", "m", 0.0, 10.0,
        "stroke ±0.2 m; core §C.2", |p| p.control.seismic.isolation.stroke_m;
    "control.seismic.isolation.hp_hz", "Hz", 0.001, 10.0,
        "IMU high-pass 0.1 Hz; core §C.4", |p| p.control.seismic.isolation.hp_hz;
    "control.seismic.amd.enable", "", 0.0, 1.0,
        "research tier, disabled; SEI-016/021", |p| f64::from(u8::from(p.control.seismic.amd.enable));
    "control.seismic.amd.c_dvf_n_s_per_m", "N·s/m", 0.0, 1.0e9,
        "direct velocity feedback; core §C.4", |p| p.control.seismic.amd.c_dvf_n_s_per_m;
    "control.seismic.amd.k_lqr.0", "SI", -1.0e12, 1.0e12,
        "LQR gains injected from Python (scipy CARE); core §C.4", |p| p.control.seismic.amd.k_lqr[0];
    "control.seismic.amd.k_lqr.1", "SI", -1.0e12, 1.0e12,
        "LQR gains injected from Python (scipy CARE); core §C.4", |p| p.control.seismic.amd.k_lqr[1];
    "control.seismic.amd.k_lqr.2", "SI", -1.0e12, 1.0e12,
        "LQR gains injected from Python (scipy CARE); core §C.4", |p| p.control.seismic.amd.k_lqr[2];
    "control.seismic.amd.k_lqr.3", "SI", -1.0e12, 1.0e12,
        "LQR gains injected from Python (scipy CARE); core §C.4", |p| p.control.seismic.amd.k_lqr[3];
    "control.seismic.amd.f_max_n", "N", 0.0, 1.0e9,
        "force limit = F_cont; core §C.4", |p| p.control.seismic.amd.f_max_n;
    "control.supervisor.watchdog_s", "s", 1.0e-4, 1.0,
        "watchdog 5 ms; core §C.4", |p| p.control.supervisor.watchdog_s;
    "control.supervisor.heartbeat_periods", "", 1.0, 100.0,
        "heartbeat 3 periods; core §C.4", |p| f64::from(p.control.supervisor.heartbeat_periods);
    "control.supervisor.payload_mismatch_ratio", "1", 1.0, 2.0,
        "PayloadMismatch at 1.05·m_rated; F18", |p| p.control.supervisor.payload_mismatch_ratio;
    "control.supervisor.m_gov_ratio", "1", 1.0, 2.0,
        "m_gov = max(m_rated, 1.05·m̂); F19", |p| p.control.supervisor.m_gov_ratio;
    "control.supervisor.holdup_margin", "1", 3.0, 100.0,
        "hold-up ≥ 3× need; SAF-009", |p| p.control.supervisor.holdup_margin;
    "control.supervisor.stall_v_mps", "m/s", 0.0, 0.1,
        "stall |v| < 2 mm/s; core §C.4", |p| p.control.supervisor.stall_v_mps;
    "control.supervisor.stall_t_s", "s", 0.0, 10.0,
        "stall > 0.2 s; core §C.4", |p| p.control.supervisor.stall_t_s;
    "control.supervisor.hover_timeout_s", "s", 0.0, 86400.0,
        "open hover ≈ 5 min on the supercap (0 = none); plan §2b item 4", |p| p.control.supervisor.hover_timeout_s;
    "control.supervisor.lift_retries", "", 0.0, 100.0,
        "lift retries; design choice", |p| f64::from(p.control.supervisor.lift_retries);
    "control.supervisor.telemetry_decim", "", 1.0, 10000.0,
        "telemetry 1 kHz; core §A.5", |p| f64::from(p.control.supervisor.telemetry_decim);
    "control.supervisor.restore_local_only", "", 0.0, 1.0,
        "restore needs a local command; plan §4b", |p| f64::from(u8::from(p.control.supervisor.restore_local_only));
    "safety.mass_rated_kg", "kg", 0.1, 1.0e8,
        "rated mass (never m̂); SAF-012", |p| p.safety.mass_rated_kg;
    "safety.e_max_j", "J", 0.0, 1.69,
        "1.69 J (A156.19, EN 16005 low energy); SAF-001", |p| p.safety.e_max_j;
    "safety.p_max_kgmps", "kg·m/s", 0.0, 22.3,
        "22.3 kg·m/s (A156.10; 0 = hold-to-run n/a); SAF-002", |p| p.safety.p_max_kgmps;
    "safety.v_open_max_mps", "m/s", 0.0, 10.0,
        "unsensed open cap; SAF-002", |p| p.safety.v_open_max_mps;
    "safety.v_close_max_mps", "m/s", 0.0, 10.0,
        "unsensed close cap; SAF-002", |p| p.safety.v_close_max_mps;
    "safety.v_open_sensed_mps", "m/s", 0.0, 10.0,
        "open cap with PL d; core §C.2, SAF-003", |p| p.safety.v_open_sensed_mps;
    "safety.v_close_sensed_mps", "m/s", 0.0, 10.0,
        "close cap with PL d; core §C.2, SAF-003", |p| p.safety.v_close_sensed_mps;
    "safety.f_drive_max_n", "N", 0.0, 400.0,
        "≤ 400 N for ≤ 0.75 s (EN 16005); SAF-004", |p| p.safety.f_drive_max_n;
    "safety.t_drive_max_s", "s", 0.0, 0.75,
        "0.75 s window; SAF-004", |p| p.safety.t_drive_max_s;
    "safety.f_static_n", "N", 0.0, 150.0,
        "67 N unsensed; F4, PRP-004", |p| p.safety.f_static_n;
    "safety.f_static_sensed_n", "N", 0.0, 150.0,
        "150 N sensed; F4", |p| p.safety.f_static_sensed_n;
    "safety.f_static_final_n", "N", 0.0, 80.0,
        "< 80 N within 5 s; SAF-004", |p| p.safety.f_static_final_n;
    "safety.t_static_final_s", "s", 0.0, 5.0,
        "5 s; SAF-004", |p| p.safety.t_static_final_s;
    "safety.f_close_last50_n", "N", 0.0, 150.0,
        "150 N in the last 50 mm; core §C.2", |p| p.safety.f_close_last50_n;
    "safety.last_zone_m", "m", 0.0, 1.0,
        "last 50 mm; core §C.2", |p| p.safety.last_zone_m;
    "safety.i_coil_cont_a", "A", 0.01, 10000.0,
        "mirrors i_cont (I²t); plan §2b item 4", |p| p.safety.i_coil_cont_a;
    "safety.i_coil_pulse_a", "A", 0.01, 10000.0,
        "mirrors i_pulse; plan §2b item 4", |p| p.safety.i_coil_pulse_a;
    "safety.t_coil_pulse_s", "s", 0.0, 10.0,
        "mirrors t_pulse; plan §2b item 4", |p| p.safety.t_coil_pulse_s;
    "safety.i_lsm_max_a", "A", 0.01, 1.0e5,
        "mirrors lsm.i_max_a; SAF-012", |p| p.safety.i_lsm_max_a;
    "safety.gap_min_m", "m", 0.0, 0.2,
        "g_up − 0.2 mm; design choice", |p| p.safety.gap_min_m;
    "safety.gap_max_m", "m", 0.0, 0.5,
        "g_land + 0.5 mm; design choice", |p| p.safety.gap_max_m;
    "safety.x_min_m", "m", -10000.0, 10000.0,
        "SLP lower bound; SAF-012", |p| p.safety.x_min_m;
    "safety.x_max_m", "m", -10000.0, 10000.0,
        "SLP upper bound; SAF-012", |p| p.safety.x_max_m;
    "safety.overspeed_ratio", "1", 1.0, 2.0,
        "1.1·v_lim; SAF-008", |p| p.safety.overspeed_ratio;
    "safety.overspeed_t_s", "s", 0.0, 1.0,
        "> 10 ms; SAF-008", |p| p.safety.overspeed_t_s;
    "safety.vel_window_s", "s", 0.01, 1.0,
        "≥ 10 ms window; plan §2b item 17", |p| p.safety.vel_window_s;
    "safety.v_runaway_mps", "m/s", 0.0, 100.0,
        "ASC above 2× cap; F9", |p| p.safety.v_runaway_mps;
    "safety.v_hold_min_v", "V", 0.0, 10000.0,
        "hold-up floor; SAF-009", |p| p.safety.v_hold_min_v;
    "safety.k_f_copy_n_per_a", "N/A", 0.01, 1.0e5,
        "diverse copy of lsm.k_f_n_per_a; SAF-015", |p| p.safety.k_f_copy_n_per_a;
    "safety.adc_a_per_code_copy", "A/code", 1.0e-6, 10.0,
        "diverse copy of sensors.coil_a_per_code; SAF-015", |p| p.safety.adc_a_per_code_copy;
    "safety.lsm_a_per_code_copy", "A/code", 1.0e-6, 10.0,
        "diverse copy of sensors.lsm_a_per_code; SAF-015", |p| p.safety.lsm_a_per_code_copy;
    "safety.nm_per_count_copy", "nm/count", 1.0, 1.0e6,
        "diverse copy of sensors.nm_per_count; SAF-015", |p| f64::from(p.safety.nm_per_count_copy);
    "safety.presence_copy", "", 0.0, 1.0,
        "diverse copy of design.presence (none/pl_d); SAF-003, SAF-015", |p| f64::from(p.safety.presence_copy.as_u8());
    "safety.watchdog_ticks", "tick", 1.0, 1.0e6,
        "watchdog_s × base_hz; core §C.4", |p| f64::from(p.safety.watchdog_ticks);
    "safety.heartbeat_max_ticks", "tick", 1.0, 1.0e6,
        "heartbeat_periods × sup_decim; core §C.4", |p| f64::from(p.safety.heartbeat_max_ticks);
    "safety.isolate.stroke_m", "m", 0.0, 10.0,
        "isolation stroke envelope; SAF-020", |p| p.safety.isolate.stroke_m;
    "safety.isolate.bumper_ke_j", "J", 0.0, 1.0e10,
        "KE at bumper ≤ rating; SAF-020", |p| p.safety.isolate.bumper_ke_j;
    "safety.isolate.f_cont_n", "N", 0.0, 1.0e8,
        "continuous force F_cont; SAF-020, plan §2b item 5", |p| p.safety.isolate.f_cont_n;
    "safety.unattended.f_pinch_n", "N", 0.0, 150.0,
        "pinch ≤ 150 N; plan §4b", |p| p.safety.unattended.f_pinch_n;
    "safety.unattended.k_series_n_per_m", "N/m", 1.0, 1.0e7,
        "hand 75 N/mm ⊕ edge 10 N/mm; plan §4b, ISO/TS 15066", |p| p.safety.unattended.k_series_n_per_m;
    "safety.unattended.detect_max_s", "s", 0.001, 1.0,
        "hand detected ≤ 20 ms; plan §4b", |p| p.safety.unattended.detect_max_s;
    "safety.unattended.max_attempts", "", 1.0, 10.0,
        "one attempt; plan §4b", |p| f64::from(p.safety.unattended.max_attempts);
    "safety.crc", "", 0.0, 4.294967295e9,
        "CRC-32/ISO-HDLC over the block (crc::seal); SAF-015", |p| f64::from(p.safety.crc);
}

/// The metadata entry for `path`, if any.
pub fn find(path: &str) -> Option<&'static FieldMeta> {
    FIELDS.iter().find(|f| f.path == path)
}

/// Index of `path` in [`FIELDS`], if any.
pub fn index_of(path: &str) -> Option<usize> {
    FIELDS.iter().position(|f| f.path == path)
}

/// The value of leaf `index` of `p` as f64 (enums as discriminant, flags as 0/1).
pub fn value_at(p: &SystemParams, index: usize) -> Option<f64> {
    GETTERS.get(index).map(|g| g(p))
}

/// The value of leaf `path` of `p` as f64 (enums as discriminant, flags as 0/1).
pub fn value(p: &SystemParams, path: &str) -> Option<f64> {
    index_of(path).and_then(|i| value_at(p, i))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::presets;

    #[test]
    fn table_is_well_formed() {
        assert_eq!(FIELDS.len(), N_FIELDS);
        for (i, f) in FIELDS.iter().enumerate() {
            assert!(f.min <= f.max, "{}: min ≤ max", f.path);
            assert!(
                !f.source.is_empty(),
                "{}: every field carries a source",
                f.path
            );
            assert!(
                f.path.starts_with("design.")
                    || f.path.starts_with("control.")
                    || f.path.starts_with("safety."),
                "{}: rooted at design/control/safety",
                f.path
            );
            for g in FIELDS.iter().skip(i + 1) {
                assert_ne!(f.path, g.path, "paths are unique");
            }
        }
    }

    #[test]
    fn getters_read_the_named_field() {
        let p = presets::drawer();
        assert_eq!(value(&p, "design.suspension.g_star_m"), Some(2.5e-3));
        assert_eq!(
            value(&p, "design.class"),
            Some(0.0),
            "RailClass::Drawer wire value"
        );
        assert_eq!(
            value(&p, "design.rest_policy"),
            Some(0.0),
            "RestPolicy::Seat wire value"
        );
        assert_eq!(value(&p, "design.carriage.top_hung"), Some(0.0));
        assert_eq!(value(&p, "design.suspension.stations.3.y_m"), Some(-0.2));
        assert_eq!(value(&p, "control.rates.base_hz"), Some(20_000.0));
        assert_eq!(value(&p, "control.motion.terminus.tau_s"), Some(0.15));
        assert_eq!(value(&p, "safety.crc"), Some(f64::from(p.safety.crc)));
        assert_eq!(value(&p, "design.nope"), None);
        assert!(find("design.suspension.g_up_m").is_some());
    }
}
