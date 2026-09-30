//! Class presets: the single source of the corrected core design §C.2 values (plan §2b)
//! for sim, Python, docs and firmware, plus the pedestrian caps.
//!
//! Every value below carries its source in a trailing comment: a core design section or
//! correction (C.2 table, C.3 F-items, C.4 controller defaults, C.5 plant defaults), a
//! plan §2b/§4b binding fix, a standard, a datasheet, or "design choice" with the reason.
//! Quantities that follow from others (the balanced pole length, speed caps, the
//! monitor's mirrors and diverse copies, isolation period) are computed in `finish` by
//! `derive`, so a change to an input re-derives consistently.
//!
//! Exotic (sovereign) code imports the pedestrian caps from here and never duplicates
//! the literals (a guard forbids them in exo code; plan §4).

use crate::derive;
use crate::{AccelMount, LandingKind, LsmKind, MagnetStation, OperatingMode, SystemParams};
use fabrail_types::{
    EgressRole, MAX_MAGNETS, PresenceSensing, RailClass, RestPolicy, SeismicAction, fmath,
};

/// Low-energy kinetic-energy cap, J. Source: ANSI/BHMA A156.19 low-energy power-operated
/// doors (1.25 lbf·ft) and EN 16005 low-energy mode; applied to every unsensed class
/// (core design F7; SAF-001).
pub const PEDESTRIAN_E_MAX_J: f64 = 1.69;

/// Momentum cap, kg·m/s. Source: ANSI/BHMA A156.10-2005 (power-operated pedestrian
/// doors); binds before 1.69 J on WALL (core design F3; SAF-002).
pub const PEDESTRIAN_P_MAX_KGMPS: f64 = 22.3;

/// Unsensed static (stall) force limit, N. Source: ANSI/BHMA A156.19 / A156.38 (15 lbf)
/// and EN 16005 low-energy mode (core design F4; PRP-004).
pub const PEDESTRIAN_F_STATIC_N: f64 = 67.0;

/// Static force limit when sensed or within the last 50 mm of closing, N.
/// Source: EN 16005 (2011 draft numeric limits; core design F4).
pub const PEDESTRIAN_F_STATIC_SENSED_N: f64 = 150.0;

/// Static force that must be reached within 5 s, N (strictly below). Source: EN 16005
/// (2011 draft numeric limits; SAF-004).
pub const PEDESTRIAN_F_STATIC_FINAL_N: f64 = 80.0;

/// Dynamic driving-force limit for at most 0.75 s, N. Source: EN 16005 (2011 draft
/// numeric limits; core design §C.2 row "F_drive_max"; SAF-004).
pub const PEDESTRIAN_F_DYNAMIC_N: f64 = 400.0;

/// Hand transient-contact body stiffness, N/m. Source: ISO/TS 15066:2016 Annex A
/// (75 N/mm; core design §C.5 human models).
pub const ISO15066_HAND_K_N_PER_M: f64 = 75.0e3;

/// Chest body stiffness, N/m. Source: ISO/TS 15066:2016 Annex A (25 N/mm; core design
/// §C.5).
pub const ISO15066_CHEST_K_N_PER_M: f64 = 25.0e3;

/// Maximum closing-edge seal stiffness, N/m. Source: core design F6 (≤ 10 N/mm brings
/// the 1.69 J hand clamp from 503 N to 173 N).
pub const EDGE_K_MAX_N_PER_M: f64 = 10.0e3;

/// Seismic bumper-rating basis speed for DRAWER, m/s: a drawer on its rollers is free
/// along x during shaking (core design F13: ½·48·1² = 24 J).
pub const SEISMIC_BUMPER_V_DRAWER_MPS: f64 = 1.0;

/// Seismic bumper-rating basis speed for DOOR/WALL/HEAVY, m/s (core design §C.2 bumper
/// row: 113 / 563 / 5625 J = ½·m·1.5²).
pub const SEISMIC_BUMPER_V_MPS: f64 = 1.5;

/// The four core classes in discriminant order.
pub const CLASSES: [RailClass; 4] = [
    RailClass::Drawer,
    RailClass::Door,
    RailClass::Wall,
    RailClass::Heavy,
];

/// Seismic bumper basis speed for `class` (SAF-014; core design F13 and §C.2), m/s.
pub const fn bumper_basis_speed(class: RailClass) -> f64 {
    match class {
        RailClass::Drawer => SEISMIC_BUMPER_V_DRAWER_MPS,
        _ => SEISMIC_BUMPER_V_MPS,
    }
}

/// A lift element with a PM bias on the pole-face plane (z = 0).
const fn lift(x_m: f64, y_m: f64) -> MagnetStation {
    MagnetStation {
        x_m,
        y_m,
        z_m: 0.0,
        normal_sign: 1.0,
        has_pm: true,
    }
}

/// An unused station slot.
const UNUSED: MagnetStation = MagnetStation {
    x_m: 0.0,
    y_m: 0.0,
    z_m: 0.0,
    normal_sign: 0.0,
    has_pm: false,
};

/// Fills the station table from `used`, leaving the rest unused.
fn stations(used: &[MagnetStation]) -> [MagnetStation; MAX_MAGNETS] {
    let mut out = [UNUSED; MAX_MAGNETS];
    for (o, s) in out.iter_mut().zip(used.iter()) {
        *o = *s;
    }
    out
}

/// Box inertia about the CoM, [J_x, J_y, J_z] for extents (x, y, z), kg·m².
fn box_inertia(m_kg: f64, lx_m: f64, ly_m: f64, lz_m: f64) -> [f64; 3] {
    [
        m_kg * (ly_m * ly_m + lz_m * lz_m) / 12.0,
        m_kg * (lx_m * lx_m + lz_m * lz_m) / 12.0,
        m_kg * (lx_m * lx_m + ly_m * ly_m) / 12.0,
    ]
}

/// Values shared by every class (core design §A.5, §C.4, §C.5; plan §2b).
fn common(class: RailClass) -> SystemParams {
    let mut p = SystemParams::default();
    p.design.class = class;
    p.design.egress = EgressRole::None;
    p.design.presence = PresenceSensing::None; // unsensed by default (SAF-003)

    let s = &mut p.design.suspension;
    s.b_r_t = 1.3; // sintered NdFeB, N42 class; core §C.2 sizing (h_m = 2g*·B_g·μ_rec/(B_r − B_g))
    s.mu_rec = 1.05; // NdFeB recoil permeability; core §C.2 sizing
    s.l_sigma_h = 0.0; // core §C.2 L equals N²·μ₀A_p/(2g* + h′); leakage neglected (σ_leak = 0)
    s.t_pulse_s = 0.2; // i_pulse window ≤ 200 ms; core §C.2 row "i_cont / i_pulse"
    s.b_sat_t = 1.5; // iron saturation 1.5 T; core §C.1 magnetics
    s.tau_eddy_s = 1.0e-3; // plant eddy lag = the design point (C.5's 0.5 ms solid-rail estimate is inside the 0–2 ms band; LEV-020)
    s.tempco_br_per_k = -0.0012; // NdFeB −0.12 %/K; core §C.1
    s.tempco_r_per_k = 0.003_93; // copper +0.393 %/K; core §C.1
    s.leak = 0.0; // σ_leak "ideal"; core §C.5 (0.25 in sensitivity studies only)
    s.stray_target_t = 0.5e-3; // ≤ 0.5 mT at 10 cm; core §C.2, ENV-001

    let dmp = &mut p.design.damping;
    dmp.ez_len_m = 0.15; // 150 mm end zones; core §C.2, SAF-014
    dmp.bumper_hc_s_per_m = 0.3; // Hunt–Crossley α (e ≈ 0.8 at 1 m/s); Hunt & Crossley 1975
    dmp.k_edge_n_per_m = EDGE_K_MAX_N_PER_M; // ≤ 10 N/mm; F6

    let ld = &mut p.design.landing;
    ld.rollers_mu = 0.003; // roller μ 0.003 everywhere; plan §2b item 19, core §C.5

    let r = &mut p.design.rail;
    r.x_closed_m = 0.0; // closed stop is the origin (PHYSICS §1)
    r.deflection_ratio = 1150.0; // sag L/1150; core §C.1 rail
    r.lateral_step_m = 0.1e-3; // lateral joint step; core §C.1 rail ("lateral steps")
    r.roughness_rms_m = 20.0e-6; // 20 µm rms; core §C.1 rail
    r.b_sat_rail_t = 1.5; // rail steel 1.5 T; LEV-015
    r.laminated = true; // the discrete design assumes a laminated/SMC rail; plan §2b item 1

    let th = &mut p.design.thermal;
    th.t_ambient_c = 20.0; // reference ambient (tempco reference 20 °C)
    th.t_coil_max_c = 120.0; // class B insulation 130 °C less 10 K margin; design choice
    th.t_mag_max_c = 80.0; // SH-grade demagnetisation proxy; ENV-004

    let sc = &mut p.design.sensors;
    sc.gap_poly_m = [0.0, 0.020 / 65_535.0, 0.0]; // 16-bit ADC over 20 mm; core §C.5
    sc.gap_redundant = true; // dual gap probes (frontend::dual_gap_vote); core §E.2
    sc.nm_per_count = 1000; // 1 µm/count encoder; core §C.5
    sc.accel_mps2_per_lsb = 3.9e-6 * derive::G0_MPS2; // ADXL355 ±2 g, 3.9 µg/LSB; ADI datasheet
    sc.accel_noise_mps2_per_rthz = 22.5e-6 * derive::G0_MPS2; // 22.5 µg/√Hz; ADI datasheet, core §C.5
    sc.accel_odr_hz = 500.0; // ADXL355 ODR 500 Hz ÷5 → 100 Hz; core §C.2
    sc.accel_mount = AccelMount::Structure; // structure-mounted; SEI-001, F16
    sc.imu_present = true; // carriage IMU for skyhook/fusion only; F16
    sc.imu_mps2_per_lsb = 8.0 * derive::G0_MPS2 / 32_768.0; // ±8 g, 16-bit; design choice
    sc.imu_noise_mps2 = 1.0e-3 * derive::G0_MPS2; // ≈ 1 mg rms; plan §4b residual terms
    sc.rail_accel_present = true; // rail floor accelerometer (obstruction residual in shaking); plan §4b
    sc.rail_accel_mps2_per_lsb = 8.0 * derive::G0_MPS2 / 32_768.0; // ±8 g, 16-bit; design choice
    sc.rail_accel_noise_mps2 = 1.0e-3 * derive::G0_MPS2; // ≈ 1 mg rms; plan §4b
    sc.i_bus_a_per_code = 0.01; // 10 mA/code bus shunt; design choice
    sc.temp_c_per_code = 0.1; // 12-bit over −50…359 °C; design choice
    sc.temp_offset_c = -50.0; // design choice (see temp_c_per_code)
    sc.handle_n_per_code = 0.0; // no handle load cell fitted (optional; HAP-002 limit 20 needs one)
    sc.handle_noise_n = 0.05; // σ 0.05 N if fitted; core §C.5
    sc.latency_ticks = 1; // 1-tick sensor latency; core §C.1 sensors
    sc.link_latency_s = 3.0e-3; // CAN 3 ms; core §C.5

    let pw = &mut p.design.power;
    pw.pwm_hz = 20_000.0; // PWM = base rate 20 kHz (≥ 20 kHz inaudible); ENV-002, core §A.5
    pw.supercap_esr_ohm = 0.01; // 10 mΩ; core §C.1 power
    pw.converter_eff = 0.9; // η 0.9; core §C.1 power
    pw.ups = false; // no building UPS assumed; plan §4b admission

    let c = &mut p.control;
    c.rates.base_hz = 20_000; // core §A.5
    c.rates.sup_decim = 20; // 1 kHz; core §A.5
    c.rates.seismic_decim = 200; // 100 Hz; core §A.5
    c.rates.coil_mode_voltage = true; // SIL ≡ firmware in voltage mode; plan §2b item 16 (firmware runs the PI)

    let lt = &mut c.levitation;
    lt.omega_ratio = 2.5; // ω = 2.5·p; plan §2b item 1
    lt.zeta = 0.7; // core §C.4
    lt.alpha = 0.3; // core §C.4
    lt.observer_ratio = 4.0; // Luenberger observer at 4ω; core §C.4
    lt.tau_e_nominal_s = 1.0e-3; // design eddy lag: centre of the 0–2 ms robust band (LEV-020); plan §2b item 1
    lt.current_bw_hz = 1000.0; // ω_c = 2π·1 kHz (τ_cl 0.159 ms); plan §2b item 2
    lt.warp_bw_hz = 2.0; // null-space integrator: ≤ 5 % within 1 s (LEV-009); design choice
    lt.zp_outer_bw_hz = 1.0; // modified-ZP outer PI 1 Hz; core §C.4
    lt.bumpless_dg_m = 50.0e-6; // |δg| < 50 µm; core §C.4
    lt.unseat_liftoff_m = 20.0e-6; // lift-off sensed > 20 µm; F17
    lt.touchdown_gdot_mps = 1.0e-3; // ġ ≈ 0 test; core §C.4 touchdown; design choice
    lt.m_hat_tau_s = 0.2; // ±5 % within 1 s (5 τ); LEV-013
    lt.sat_persist_s = 0.05; // design choice

    let mo = &mut c.motion;
    mo.kx_ratio = 0.25; // K_x = ω_v/4; core §C.4
    mo.ki_ratio = 0.25; // K_vi = K_v·ω_v/4; core §C.4
    mo.a_brk_ratio = 2.0; // a_brk = 2·a_max; core §C.4
    mo.t_react_fast_periods = 2; // t_react = 2 fast periods; core §C.4
    mo.gov_margin_m = 2.0e-3; // governor margin before x_obs; design choice
    mo.ripple_ff = true; // ripple LUT feed-forward; core §C.4
    mo.terminus.d_eps_m = 0.2e-3; // d_ε ≈ 0.2 mm; plan §4b
    mo.terminus.t_q_ratio = 1.0; // T_q = τ; plan §4b
    mo.terminus.overtravel_m = 5.0e-3; // x_obs = terminus + 5 mm > margin (tail never clipped); plan §4b
    mo.terminus.replan_brk_ratio = 2.0; // a_brk = 2·a_max on re-plan; plan §4b

    let inr = &mut mo.inertial;
    inr.enable = false; // research tier, off in every preset; plan §4b
    inr.f_c_hz = 1.0; // crossover ≈ 1 Hz; plan §4b
    inr.zeta = 0.7; // skyhook damping ratio; plan §4b (T_sky ≈ 0.24–0.31 at 2 Hz)
    inr.hp_hz = 0.1; // leaky integral 0.1 Hz; plan §4b
    inr.standoff_m = 0.05; // ride-through standoff; plan §4b (s_min = 25 mm + S_max)
    inr.s_min_base_m = 0.025; // plan §4b
    inr.stroke_margin_m = 5.0e-3; // design choice
    inr.v_rel_frac = 0.8; // v_rel,pred ≤ 0.8·v_cap; plan §4b
    inr.t_burst_s = 0.75; // bursts ≤ 0.75 s; plan §4b
    inr.z_headroom_frac = 0.5; // a_z ≤ 0.5·p²·(g_float − g_up); plan §4b
    inr.structure_lp_hz = 0.3; // ADXL355 used below 0.3 Hz only; plan §4b
    inr.latency_s = 0.2e-3; // base + fast latency (2πfL term); design choice

    let hp = &mut c.haptic;
    hp.k_virtual_frac = 0.3; // K_v ≤ 2b/(T + 2T_d) with T_d = T ⇒ ≤ 1/3 of 2b/T; plan §2b item 19
    hp.passivity_alpha_ratio = 5.0; // α_max = 5·b_dist; core §C.4
    hp.f_hp_hz = 0.5; // F̂ high-pass 0.5 Hz; core §C.4
    hp.obstruction.floor_gain = 0.1; // th = th0 + 0.1·m̂·|a_floor|; plan §4b
    hp.obstruction.degraded_ratio = 2.0; // > 2·th0 → ObstructionDegraded; plan §4b

    let it = &mut c.intent;
    it.lean_v_mps = 2.0e-3; // |v| < 2 mm/s; core §C.4 lean detection
    it.lean_t_s = 1.0; // > 1 s; core §C.4
    it.dir_consistency = true; // cabinet pressure pushes outward; plan §2b item 11

    let dt = &mut c.seismic.detector;
    dt.aa_decim = 5; // 500 → 100 Hz; core §C.4
    dt.bp_lo_hz = 0.5; // core §C.4
    dt.bp_hi_hz = 20.0; // core §C.4
    dt.bp_order = 4; // 4-pole Butterworth; core §C.4
    dt.sta_n = 50; // core §C.4
    dt.lta_n = 3000; // core §C.4
    dt.trig_on = 4.0; // core §C.4
    dt.trig_off = 1.5; // core §C.4
    dt.pdtc_hp_hz = 0.075; // core §C.4
    dt.win_short_s = 1.0; // core §C.4
    dt.win_long_s = 3.0; // core §C.4
    dt.taup_alpha = 0.99; // [M]; core §C.4
    dt.zh_window_s = 0.5; // Z/H in the first 0.5 s; core §C.4
    dt.zh_min = 1.0; // Z/H > 1; core §C.4
    dt.pga_veto_mps2 = 0.8; // 80 gal; core §C.4
    dt.tauc_l3_s = 0.6; // core §C.4
    dt.pd_l3_m = 0.005; // 0.5 cm; core §C.4
    dt.pd_alert_m = 0.0035; // 0.35 cm; plan §2b item 6
    dt.pga_alert_mps2 = 0.8; // 80 gal; plan §2b item 6
    dt.protect_level = 2; // Level ≥ 2 (Level 1 = freeze only); plan §2b item 6

    let fu = &mut c.seismic.fusion;
    fu.v_p_kmps = 6.0; // core §C.4
    fu.v_s_kmps = 3.5; // core §C.4
    fu.quiet_s = 30.0; // QUIET 30 s; core §C.4
    fu.eta_sigma_s = 1.5; // S-ETA σ 1–2 s; design choice within the stated range
    fu.alert_timeout_s = 60.0; // design choice
    fu.min_independent_feeds = 2; // ≥ 2 feeds without a local P pick; plan §4

    let po = &mut c.seismic.policy;
    po.egress_stop_margin_s = 1.0; // stopped by t_S − 1 s; SEI-011
    po.asc_on_seismic_land = true; // F9

    let iso = &mut c.seismic.isolation;
    iso.enable = false; // research tier; plan §2b item 5
    iso.zeta = 0.3; // core §C.2 isolation row
    iso.stroke_m = 0.2; // ±0.2 m; core §C.2
    iso.hp_hz = 0.1; // IMU leaky integral 0.1 Hz; core §C.4

    c.seismic.amd.enable = false; // research tier; SEI-016/021

    let sv = &mut c.supervisor;
    sv.watchdog_s = 5.0e-3; // 5 ms; core §C.4
    sv.heartbeat_periods = 3; // core §C.4
    sv.payload_mismatch_ratio = 1.05; // F18
    sv.m_gov_ratio = 1.05; // F19
    sv.holdup_margin = 3.0; // SAF-009
    sv.stall_v_mps = 2.0e-3; // core §C.4 clamp stall
    sv.stall_t_s = 0.2; // core §C.4
    sv.lift_retries = 2; // design choice
    sv.telemetry_decim = 1; // 1 kHz frames; core §A.5
    sv.restore_local_only = true; // restore needs a local command; plan §4b

    let sf = &mut p.safety;
    sf.e_max_j = PEDESTRIAN_E_MAX_J; // F7, SAF-001
    sf.t_drive_max_s = 0.75; // EN 16005 dynamic window; core §C.2
    sf.f_static_n = PEDESTRIAN_F_STATIC_N; // F4
    sf.f_static_sensed_n = PEDESTRIAN_F_STATIC_SENSED_N; // F4
    sf.f_static_final_n = PEDESTRIAN_F_STATIC_FINAL_N; // SAF-004
    sf.t_static_final_s = 5.0; // < 80 N within 5 s; SAF-004
    sf.f_close_last50_n = PEDESTRIAN_F_STATIC_SENSED_N; // 150 N in the last 50 mm; core §C.2
    sf.last_zone_m = 0.05; // last 50 mm; core §C.2
    sf.overspeed_ratio = 1.1; // SAF-008
    sf.overspeed_t_s = 0.01; // > 10 ms; SAF-008
    sf.vel_window_s = 0.01; // ≥ 10 ms window; plan §2b item 17
    sf.watchdog_ticks = 100; // 5 ms at 20 kHz; core §C.4
    sf.heartbeat_max_ticks = 60; // 3 supervisor periods at 20 ticks; core §C.4
    sf.unattended.f_pinch_n = PEDESTRIAN_F_STATIC_SENSED_N; // 150 N; plan §4b
    sf.unattended.k_series_n_per_m = derive::series_k(ISO15066_HAND_K_N_PER_M, EDGE_K_MAX_N_PER_M); // hand ⊕ edge; plan §4b
    sf.unattended.detect_max_s = 0.02; // hand detected within 20 ms; plan §4b
    sf.unattended.max_attempts = 1; // one attempt only; plan §4b
    sf.isolate.stroke_m = 0.2; // isolation stroke envelope; SAF-020, core §C.2
    p
}

/// Derived and mirrored fields, then the seal (see the module docs).
fn finish(mut p: SystemParams) -> SystemParams {
    let m_star = p.design.carriage.m_star_kg;
    let s = &mut p.design.suspension;
    // Pole length solved so n·F_pm(g*) = m*·g₀ exactly (core §C.2 "solved so
    // F(g*) = m*g/n"; PHYSICS §3).
    s.pole_l_m = derive::balance_pole_length(s, m_star);

    let d = p.design;
    // SAF-009 hold-up floor for the independent monitor (W1.5, A5 request): the usable
    // energy η·½C(V² − V_min²) is monotone in V, so E ≥ margin·E_need is the voltage
    // floor V_req = √(V_min² + 2·margin·E_need/(η·C)) (PHYSICS §14), rounded up to 1 mV.
    let v_req = {
        let pw = &d.power;
        let need = derive::holdup_energy(&p).need_j;
        let margin = p.control.supervisor.holdup_margin;
        let den = pw.converter_eff * pw.supercap_c_f;
        let v2 = pw.supercap_v_min_v * pw.supercap_v_min_v + 2.0 * margin * need / den;
        if den > 0.0 && v2.is_finite() && v2 > 0.0 {
            fmath::ceil(fmath::sqrt(v2) * 1.0e3) / 1.0e3
        } else {
            pw.supercap_v_min_v
        }
    };
    let sf = &mut p.safety;
    sf.mass_rated_kg = d.carriage.m_rated_kg;
    sf.i_coil_cont_a = d.suspension.i_cont_a; // I²t mirror; plan §2b item 4
    sf.i_coil_pulse_a = d.suspension.i_pulse_a;
    sf.t_coil_pulse_s = d.suspension.t_pulse_s;
    sf.i_lsm_max_a = d.lsm.i_max_a;
    sf.gap_min_m = round_nm(d.suspension.g_up_m - 0.2e-3); // below the upper stop ⇒ sensor fault; design choice
    sf.gap_max_m = round_nm(d.suspension.g_land_m + 0.5e-3); // above the rollers ⇒ fault; design choice
    sf.x_min_m = round_nm(d.rail.x_closed_m - 0.01); // SLP 10 mm beyond the closed stop; design choice
    sf.x_max_m = round_nm(d.rail.x_closed_m + d.rail.travel_m + 0.01); // SLP 10 mm beyond full travel
    sf.v_hold_min_v = fmath::fmax(v_req, d.power.supercap_v_min_v); // SAF-009 3× hold-up floor
    sf.isolate.f_cont_n = d.lsm.f_cont_n; // F_cont; SAF-020, plan §2b item 5
    sf.isolate.bumper_ke_j = 0.5 * d.damping.bumper_e_rated_j; // factor-2 margin on the rating; design choice
    sf.v_runaway_mps = 2.0 * fmath::fmax(sf.v_open_max_mps, sf.v_close_max_mps); // ASC above 2× the cap; F9, design choice

    let c = &mut p.control;
    let t_min = derive::t_min_iso(
        d.carriage.m_rated_kg,
        c.seismic.isolation.stroke_m,
        d.lsm.f_cont_n,
    );
    c.seismic.isolation.t_iso_s = if t_min > 2.0 / 1.1 { 1.1 * t_min } else { 2.0 }; // T = max(2 s, 1.1·T_min); core §C.2
    c.seismic.amd.f_max_n = d.lsm.f_cont_n; // continuous thrust limit
    c.motion.inertial.f_burst_n = d.lsm.f_peak_n; // bursts ≤ F_peak (DRAWER 90 N); plan §4b

    crate::crc::seal(&mut p);
    p
}

/// Rounds a derived length to 1 nm, so it prints as a short decimal (JSON, PARAMETERS.md)
/// instead of carrying the last-ulp residue of the subtraction, m.
fn round_nm(x_m: f64) -> f64 {
    fmath::round(x_m * 1.0e9) / 1.0e9
}

/// Unsensed caps from the pedestrian limits at the rated mass (SAF-001/002).
fn caps(p: &SystemParams) -> f64 {
    derive::unsensed_speed_cap(
        p.design.carriage.m_rated_kg,
        p.safety.e_max_j,
        p.safety.p_max_kgmps,
    )
}

/// Cabinet drawer: 8–48 kg, captive SEAT at 3.5 mm, 0.265 m/s, travel 0.30 m.
pub fn drawer() -> SystemParams {
    let mut p = common(RailClass::Drawer);
    let d = &mut p.design;
    d.rest_policy = RestPolicy::Seat; // F17; PWR-001
    d.operating_mode = OperatingMode::Automatic; // core §C.2
    let cr = &mut d.carriage;
    cr.m_star_kg = 26.0; // core §C.2
    cr.m_rated_kg = 48.0; // core §C.2
    cr.m_op_min_kg = 8.0; // core §C.2 (8–48 kg)
    cr.j_kgm2 = box_inertia(26.0, 0.60, 0.40, 0.20); // 0.6×0.4×0.2 m box at m*; design choice
    cr.cg_m = [-0.14, 0.0, 0.08]; // CoM at the element-pattern centre, 80 mm above the pole plane (undermount); design choice (LEV-016)
    cr.top_hung = false;
    cr.length_m = 0.60; // travel 0.30 m = 50 % extension; plan §2b item 3

    let s = &mut d.suspension;
    s.n_magnets = 4; // mid + rear × 2 sides; core §C.1
    s.stations = stations(&[
        lift(-0.02, 0.2),
        lift(-0.02, -0.2),
        lift(-0.26, 0.2),
        lift(-0.26, -0.2),
    ]); // y ±0.2 m; core §C.1
    s.h_m_m = 3.28e-3; // size_magnets 3.281 mm rounded; core §C.2 (≈3.28)
    s.pole_w_m = 10.0e-3; // core §C.2 (10×32)
    s.b_g_t = 0.50; // core §C.2
    s.turns = 200.0; // core §C.2
    s.r_coil_ohm = 1.6; // core §C.2
    s.i_cont_a = 4.0; // core §C.2 †
    s.i_pulse_a = 12.0; // core §C.2 †
    s.g_star_m = 2.5e-3; // core §C.2
    s.g_up_m = 1.2e-3; // core §C.2
    s.g_land_m = 6.5e-3; // travel rollers; core §C.2
    s.g_seat_m = 3.5e-3; // captive seat; F17
    s.zp_band_m = [1.5e-3, 6.0e-3]; // modified band; F1
    s.shroud = false; // ≤ 0.5 mT met; ENV-001

    let l = &mut d.lsm;
    l.kind = LsmKind::IronlessHalbach; // core §C.2
    l.tau_p_m = 15.0e-3; // core §C.2
    l.k_f_n_per_a = 15.0; // design choice (90 N at 6 A)
    l.k_e_v_s_per_m = derive::lsm_k_e(l.k_f_n_per_a); // K_e = ⅔·K_f (amplitude-invariant dq); PRP-002, D5
    l.r_phase_ohm = 2.0; // design choice
    l.l_phase_h = 2.0e-3; // design choice
    l.f_peak_n = 90.0; // core §C.2
    l.f_cont_n = 20.0; // core §C.2
    l.t_peak_s = 0.75; // EN 16005 window; core §C.2
    l.i_max_a = 6.0; // f_peak/K_f
    l.ripple_frac = 0.01; // 1 % 6th-order ripple; core §C.1
    l.cogging_n = 0.0; // ironless
    l.f_normal_n = 0.0; // ironless
    l.segment_len_m = 0.0; // continuous stator
    l.segment_overlap_m = 0.0;

    let g = &mut d.guidance;
    g.fin_present = false; // DRAWER: UHMW side pads only; core §C.1
    g.fin_clearance_m = 0.0;
    g.pad_mu = 0.15; // UHMW μ 0.15; core §C.5
    g.pad_clearance_m = 1.0e-3; // design choice
    g.c_sway_n_s_per_m = 20.0; // design choice (not top-hung)

    let dm = &mut d.damping;
    dm.b_dist_n_s_per_m = 2.0; // core §C.2
    dm.ez_cmax_n_s_per_m = 100.0; // core §C.2
    dm.bumper_k_n_per_m = 120.0e3; // 24 J in 20 mm; design choice
    dm.bumper_e_rated_j = 24.0; // F13; SAF-014
    dm.c_rail_n_s_per_m = 1.0; // core §C.5 [UNCERTAIN]
    dm.f_hyst_n = 0.2; // core §C.5 [UNCERTAIN]

    let ld = &mut d.landing;
    ld.kind = LandingKind::Seat; // F17
    ld.upper_roller_rating_n = 600.0; // ≥ n·F_pm(g_up) = 551.5 N (F2 "≥ 550 N")
    ld.seat_z_clearance_m = 0.15e-3; // plan §2b item 4
    ld.seat_x_compliance_m = 2.5e-3; // ≥ 2 mm push stop; plan §2b item 4
    ld.seat_k_x_n_per_m = 2000.0; // 4.5 N intent at 2.25 mm; design choice
    ld.slot_len_m = 20.0e-3; // design choice; plan §2b item 4
    ld.ramp_len_m = 30.0e-3; // design choice
    ld.ramp_rise_m = 3.0e-3; // 6.5 mm rollers → 3.5 mm seat
    ld.seismic_latch.enabled = false; // optional; plan §4b
    ld.seismic_latch.x_retention_n = 0.0;
    ld.parking_lock = false;

    let r = &mut d.rail;
    r.travel_m = 0.30; // ≤ 50 % extension; plan §2b item 3
    r.span_m = 0.60; // cabinet depth; design choice
    r.joint_spacing_m = 0.0; // single piece, no joints
    r.joint_step_m = 0.0;
    r.flange_t_m = 4.0e-3; // core §C.2

    let a = &mut d.air;
    a.cavity_area_m2 = 0.06; // core §C.1 air
    a.cavity_volume_m3 = 0.03; // core §C.1 air
    a.leak_area_m2 = 40.0e-4; // 40 cm² reveal; core §C.1, research C15
    a.leaf_area_m2 = 0.0;

    let th = &mut d.thermal;
    th.r_th_coil_k_per_w = 10.0; // design choice
    th.c_th_coil_j_per_k = 50.0; // design choice
    th.r_th_mag_k_per_w = 5.0; // design choice
    th.c_th_mag_j_per_k = 100.0; // design choice

    let sc = &mut d.sensors;
    sc.gap_noise_m = 0.3e-6; // core §C.2
    sc.coil_a_per_code = 15.0 / 2048.0; // ±15 A (V_bus/R), 12-bit signed; core §C.5
    sc.coil_noise_a = 5.0e-3; // design choice
    sc.lsm_a_per_code = 9.0 / 2048.0; // ±1.5·i_max; design choice
    sc.v_bus_v_per_code = 30.0 / 4096.0; // design choice
    sc.v_hold_v_per_code = 15.0 / 4096.0; // design choice

    let pw = &mut d.power;
    pw.v_bus_v = 24.0; // core §C.2
    // Bus sized for the coil pulse budget n·i_pulse²·R_coil = 4·12²·1.6 = 922 W (plan §2b
    // item 4; the unseat draws it for ≤ t_pulse), rounded up; design choice (W1.5: the
    // earlier 120 W sagged the bus below 0.8·V_bus during every unseat, BusUnder).
    pw.p_bus_max_w = 1000.0;
    pw.v_chopper_v = 27.6; // 1.15·V_bus (≤ +16 % bus rise); PRP-010
    pw.supercap_c_f = 10.0; // 10 F at 12 V; core §C.1 power
    pw.supercap_v_max_v = 12.0;
    pw.supercap_v_min_v = 6.0; // design choice (¾ of the energy usable)
    pw.p_electronics_w = 0.5; // design choice (≈ 5 min open hover on the supercap)
    pw.p_hover_budget_w = 1.5; // PWR-002
    pw.p_sleep_budget_w = 0.3; // core §C.2

    let c = &mut p.control;
    c.rates.fast_decim = 2; // 10 kHz; core §A.5
    let lt = &mut c.levitation;
    lt.roll_active = true; // LEV-001
    lt.lift_time_max_s = 0.1; // unseat ≤ 100 ms; LEV-012
    lt.land_speed_mps = 5.0e-3; // core §C.4
    lt.unseat_ramp_a_per_s = 200.0; // 11.2 A in 56 ms; F17, design choice
    lt.seat_time_s = 0.2; // +0.2 s z-seat step; plan §4b
    let mo = &mut c.motion;
    mo.omega_v_hz = 8.0; // core §C.4
    mo.a_max_mps2 = 1.5; // core §C.2
    mo.a_max_sensed_mps2 = 1.5; // no sensed mode
    mo.j_max_mps3 = 20.0; // core §C.2
    mo.pos_tol_closed_m = 0.3e-3; // core §C.2
    mo.pos_tol_travel_m = 1.0e-3; // core §C.2
    mo.terminus.tau_s = 0.15; // plan §4b
    mo.terminus.j_mps3 = 20.0; // j_max; plan §4b table
    let hp = &mut c.haptic;
    hp.mass_ratio = 1.5; // core §C.2
    hp.b_v_n_s_per_m = 10.0; // core §C.2
    hp.k_o_hz = 20.0; // core §C.4
    hp.n_detents = 1; // design choice
    hp.detent_x_m = [0.15, 0.0, 0.0, 0.0]; // half travel; design choice
    hp.detent_f_n = 3.0; // core §C.4
    hp.detent_width_m = 2.0e-3; // core §C.4
    hp.soft_close_omega_hz = 2.0; // core §C.4
    hp.soft_close_capture_m = 45.0e-3; // core §C.2
    hp.soft_close_v_final_mps = 20.0e-3; // HAP-007
    hp.hand_onset_s = 10.0e-3; // plan §4b
    hp.obstruction.residual_n = 12.0; // core §C.2
    hp.obstruction.persist_s = 20.0e-3; // core §C.2
    hp.obstruction.reverse_m = 0.0; // release to 0 N; core §C.4
    hp.obstruction.release_to_zero = true;
    let it = &mut c.intent;
    it.f_th_n = 4.5; // core §C.2
    it.t_th_s = 50.0e-3; // core §C.2
    it.dx_th_m = 1.5e-3; // core §C.2
    it.confirm_max_s = 120.0e-3; // F12
    it.push_to_open = true; // HAP-012
    let po = &mut c.seismic.policy;
    po.action = SeismicAction::Land; // freeze → stay seated or land (seat); core §C.2
    po.center_if_reachable = false;
    po.lock_after_land = false;
    c.supervisor.hover_timeout_s = 300.0; // ≈ 5 min open hover on the supercap; plan §2b item 4

    p.safety.p_max_kgmps = PEDESTRIAN_P_MAX_KGMPS; // binds after 1.69 J at 48 kg
    let v = caps(&p);
    let sf = &mut p.safety;
    sf.v_open_max_mps = v; // 0.2654 both ways; F7, SAF-002
    sf.v_close_max_mps = v;
    sf.v_open_sensed_mps = v; // no sensed mode ("—" in core §C.2)
    sf.v_close_sensed_mps = v;
    sf.f_drive_max_n = 90.0; // core §C.2
    finish(p)
}

/// Sliding door: 100 kg, HOVER, open 0.184 / close 0.152 m/s.
pub fn door() -> SystemParams {
    let mut p = common(RailClass::Door);
    let d = &mut p.design;
    d.rest_policy = RestPolicy::Hover; // PWR-001
    d.egress = EgressRole::NonRated; // pedestrian door on an egress path: never locks; SEI-011
    d.operating_mode = OperatingMode::LowEnergy; // core §C.2
    let cr = &mut d.carriage;
    cr.m_star_kg = 100.0; // core §C.2
    cr.m_rated_kg = 100.0;
    cr.m_op_min_kg = 90.0; // 0.9·m*; F18 †
    cr.j_kgm2 = box_inertia(100.0, 1.0, 0.04, 2.1); // 1.0 × 2.1 m leaf, 40 mm; design choice
    cr.cg_m = [0.0, 0.0, -1.05]; // CoM 1.05 m below the rail; core §C.1
    cr.top_hung = true;
    cr.length_m = 1.0;

    let s = &mut d.suspension;
    s.n_magnets = 4; // core §C.1
    s.stations = stations(&[
        lift(0.4, 0.015),
        lift(0.4, -0.015),
        lift(-0.4, 0.015),
        lift(-0.4, -0.015),
    ]); // x ±0.40, y ±15 mm; core §C.1
    s.h_m_m = 7.7e-3; // core §C.2
    s.pole_w_m = 20.0e-3; // core §C.2 (20×51)
    s.b_g_t = 0.55; // core §C.2
    s.turns = 300.0; // core §C.2
    s.r_coil_ohm = 1.7; // core §C.2
    s.i_cont_a = 6.5; // core §C.2
    s.i_pulse_a = 10.0; // core §C.2
    s.g_star_m = 5.0e-3; // core §C.2
    s.g_up_m = 4.0e-3; // core §C.2
    s.g_land_m = 6.25e-3; // core §C.2
    s.g_seat_m = 0.0; // no seat
    s.zp_band_m = [4.4e-3, 5.8e-3]; // core §C.2
    s.shroud = false;

    let l = &mut d.lsm;
    l.kind = LsmKind::Ironless; // core §C.2
    l.tau_p_m = 25.0e-3; // core §C.2
    l.k_f_n_per_a = 30.0; // design choice (150 N at 5 A)
    l.k_e_v_s_per_m = derive::lsm_k_e(l.k_f_n_per_a); // K_e = ⅔·K_f; PRP-002, D5
    l.r_phase_ohm = 1.5; // design choice
    l.l_phase_h = 5.0e-3; // design choice
    l.f_peak_n = 150.0; // core §C.2
    l.f_cont_n = 50.0; // core §C.2
    l.t_peak_s = 0.75;
    l.i_max_a = 5.0;
    l.ripple_frac = 0.01;

    let g = &mut d.guidance;
    g.fin_present = true; // bottom guide fin; core §C.1
    g.fin_clearance_m = 2.0e-3; // ±2 mm; core §C.1
    g.pad_mu = 0.15; // core §C.5
    g.pad_clearance_m = 2.0e-3; // design choice
    g.c_sway_n_s_per_m = 50.0; // sway ζ ≈ 0.08 at 0.49 Hz (≥ 0.05, LEV-011)

    let dm = &mut d.damping;
    dm.b_dist_n_s_per_m = 5.0; // core §C.2
    dm.ez_cmax_n_s_per_m = 300.0; // core §C.2
    dm.bumper_k_n_per_m = 250.0e3; // 113 J in 30 mm; design choice
    dm.bumper_e_rated_j = 113.0; // SAF-014
    dm.c_rail_n_s_per_m = 5.0; // core §C.5
    dm.f_hyst_n = 1.0; // core §C.5

    let ld = &mut d.landing;
    ld.kind = LandingKind::Rollers;
    ld.upper_roller_rating_n = 1300.0; // ≥ n·F_pm(g_up) = 1253 N
    ld.parking_lock = false; // egress: never locks

    let r = &mut d.rail;
    r.travel_m = 1.0; // core §C.2
    r.span_m = 1.2; // design choice
    r.joint_spacing_m = 2.0; // design choice
    r.joint_step_m = 0.2e-3; // 0.1–0.3 mm; core §C.1
    r.flange_t_m = 8.0e-3; // core §C.2

    d.air.leaf_area_m2 = 2.0; // smoke-control ΔP on a 2 m² leaf; core §C.1, NFPA 92 context

    let th = &mut d.thermal;
    th.r_th_coil_k_per_w = 5.0;
    th.c_th_coil_j_per_k = 150.0;
    th.r_th_mag_k_per_w = 3.0;
    th.c_th_mag_j_per_k = 300.0;

    let sc = &mut d.sensors;
    sc.gap_noise_m = 0.5e-6; // core §C.2
    sc.coil_a_per_code = 12.5 / 2048.0; // ±12.5 A; design choice
    sc.coil_noise_a = 5.0e-3;
    sc.lsm_a_per_code = 7.5 / 2048.0;
    sc.v_bus_v_per_code = 60.0 / 4096.0;
    sc.v_hold_v_per_code = 15.0 / 4096.0;

    let pw = &mut d.power;
    pw.v_bus_v = 48.0; // SELV; core §C.2
    pw.p_bus_max_w = 800.0; // ≥ n·i_pulse²·R_coil = 4·10²·1.7 = 680 W (lift pulse); design choice (W1.5)
    pw.v_chopper_v = 55.2; // 1.15·V_bus (≤ +16 %), below 60 V SELV; PRP-010
    pw.supercap_c_f = 10.0;
    pw.supercap_v_max_v = 12.0;
    pw.supercap_v_min_v = 6.0;
    pw.p_electronics_w = 1.0;
    pw.p_hover_budget_w = 3.0; // PWR-002
    pw.p_sleep_budget_w = 0.5; // core §C.2

    let c = &mut p.control;
    c.rates.fast_decim = 2; // 10 kHz
    let lt = &mut c.levitation;
    lt.roll_active = false; // gravity-stabilised (51 vs 1030 N·m/rad); LEV-001
    lt.lift_time_max_s = 0.3; // LEV-012
    lt.land_speed_mps = 5.0e-3; // core §C.4
    let mo = &mut c.motion;
    mo.omega_v_hz = 6.0;
    mo.a_max_mps2 = 0.75;
    mo.a_max_sensed_mps2 = 0.75;
    mo.j_max_mps3 = 5.0;
    mo.pos_tol_closed_m = 0.5e-3;
    mo.pos_tol_travel_m = 2.0e-3;
    mo.terminus.tau_s = 0.25; // plan §4b
    mo.terminus.j_mps3 = 5.0;
    let hp = &mut c.haptic;
    hp.mass_ratio = 3.0;
    hp.b_v_n_s_per_m = 40.0;
    hp.k_o_hz = 15.0;
    hp.n_detents = 1;
    hp.detent_x_m = [0.5, 0.0, 0.0, 0.0];
    hp.detent_f_n = 8.0;
    hp.detent_width_m = 3.0e-3;
    hp.soft_close_omega_hz = 1.5;
    hp.soft_close_capture_m = 65.0e-3;
    hp.soft_close_v_final_mps = 50.0e-3;
    hp.hand_onset_s = 10.0e-3;
    hp.obstruction.residual_n = 30.0;
    hp.obstruction.persist_s = 30.0e-3;
    hp.obstruction.reverse_m = 0.05; // stop and reverse 50 mm; SAF-006
    hp.obstruction.release_to_zero = false;
    let it = &mut c.intent;
    it.f_th_n = 10.0;
    it.t_th_s = 80.0e-3;
    it.dx_th_m = 2.0e-3;
    it.confirm_max_s = 120.0e-3; // HAP-005
    it.push_to_open = false;
    let po = &mut c.seismic.policy;
    po.action = SeismicAction::Land; // stop → land on rollers, release ASC, never lock; core §C.2
    po.center_if_reachable = false;
    po.lock_after_land = false;
    c.supervisor.hover_timeout_s = 0.0; // HOVER rest on mains: no timeout

    p.safety.p_max_kgmps = PEDESTRIAN_P_MAX_KGMPS;
    let v = caps(&p);
    let sf = &mut p.safety;
    sf.v_open_max_mps = v; // √(3.38/100) = 0.1838; SAF-002
    sf.v_close_max_mps = 0.152; // close 0.152 m/s (6 in/s; KE 1.16 J); core §C.2, SAF-002
    sf.v_open_sensed_mps = 0.70; // PL d, [unverified]; core §C.2
    sf.v_close_sensed_mps = 0.22; // core §C.2
    sf.f_drive_max_n = 150.0; // core §C.2
    finish(p)
}

/// Partition wall: 500 kg, LAND, 0.0446 m/s unsensed.
pub fn wall() -> SystemParams {
    let mut p = common(RailClass::Wall);
    let d = &mut p.design;
    d.rest_policy = RestPolicy::Land; // PWR-001
    d.operating_mode = OperatingMode::Automatic; // unsensed at the momentum cap; core §C.2
    let cr = &mut d.carriage;
    cr.m_star_kg = 500.0;
    cr.m_rated_kg = 500.0;
    cr.m_op_min_kg = 450.0; // F18 †
    cr.j_kgm2 = box_inertia(500.0, 2.0, 0.10, 3.0); // 2 × 3 m panel, 100 mm; design choice
    cr.cg_m = [0.0, 0.0, -1.5]; // top-hung; core §C.1
    cr.top_hung = true;
    cr.length_m = 2.0;

    let s = &mut d.suspension;
    s.n_magnets = 4; // 2 bogies × 2; core §C.1
    s.stations = stations(&[
        lift(0.75, 0.025),
        lift(0.75, -0.025),
        lift(-0.75, 0.025),
        lift(-0.75, -0.025),
    ]); // design choice
    s.h_m_m = 18.0e-3; // core §C.2
    s.pole_w_m = 43.0e-3; // core §C.2 (43×100)
    s.b_g_t = 0.60;
    s.turns = 300.0;
    s.r_coil_ohm = 0.7;
    s.i_cont_a = 15.0;
    s.i_pulse_a = 20.0;
    s.g_star_m = 10.0e-3;
    s.g_up_m = 8.0e-3;
    s.g_land_m = 12.5e-3;
    s.g_seat_m = 0.0;
    s.zp_band_m = [8.8e-3, 11.5e-3]; // core §C.2
    s.shroud = true; // "at risk (16 mT unshielded)"; ENV-001

    let l = &mut d.lsm;
    l.kind = LsmKind::IronlessSegmented;
    l.tau_p_m = 40.0e-3;
    l.k_f_n_per_a = 50.0; // design choice (400 N at 8 A)
    l.k_e_v_s_per_m = derive::lsm_k_e(l.k_f_n_per_a); // K_e = ⅔·K_f; PRP-002, D5
    l.r_phase_ohm = 1.0;
    l.l_phase_h = 10.0e-3;
    l.f_peak_n = 400.0;
    l.f_cont_n = 100.0;
    l.t_peak_s = 0.75;
    l.i_max_a = 8.0;
    l.ripple_frac = 0.01;
    l.segment_len_m = 1.0; // design choice
    l.segment_overlap_m = 0.1; // design choice

    let g = &mut d.guidance;
    g.fin_present = true;
    g.fin_clearance_m = 2.0e-3;
    g.pad_mu = 0.15;
    g.pad_clearance_m = 2.0e-3;
    g.c_sway_n_s_per_m = 200.0; // sway ζ ≈ 0.08 at 0.41 Hz; LEV-011

    let dm = &mut d.damping;
    dm.b_dist_n_s_per_m = 20.0;
    dm.ez_cmax_n_s_per_m = 1500.0;
    dm.bumper_k_n_per_m = 450.0e3; // 563 J in 50 mm
    dm.bumper_e_rated_j = 563.0; // SAF-014
    dm.c_rail_n_s_per_m = 20.0;
    dm.f_hyst_n = 5.0;

    let ld = &mut d.landing;
    ld.kind = LandingKind::Rollers;
    ld.upper_roller_rating_n = 6500.0; // ≥ n·F_pm(g_up) = 6158 N
    ld.parking_lock = true; // land + lock in PROTECT; core §C.2

    let r = &mut d.rail;
    r.travel_m = 6.0;
    r.span_m = 1.5;
    r.joint_spacing_m = 3.0;
    r.joint_step_m = 0.3e-3;
    r.flange_t_m = 18.0e-3;

    d.air.leaf_area_m2 = 6.0;

    let th = &mut d.thermal;
    th.r_th_coil_k_per_w = 2.0;
    th.c_th_coil_j_per_k = 800.0;
    th.r_th_mag_k_per_w = 1.5;
    th.c_th_mag_j_per_k = 1500.0;

    let sc = &mut d.sensors;
    sc.gap_noise_m = 2.0e-6; // core §C.2
    sc.coil_a_per_code = 25.0 / 2048.0;
    sc.coil_noise_a = 10.0e-3;
    sc.lsm_a_per_code = 12.0 / 2048.0;
    sc.v_bus_v_per_code = 60.0 / 4096.0;
    sc.v_hold_v_per_code = 15.0 / 4096.0;

    let pw = &mut d.power;
    pw.v_bus_v = 48.0;
    pw.p_bus_max_w = 1200.0; // ≥ n·i_pulse²·R_coil = 4·20²·0.7 = 1120 W (lift pulse); design choice (W1.5)
    pw.v_chopper_v = 55.2; // 1.15·V_bus; PRP-010
    pw.supercap_c_f = 10.0;
    pw.supercap_v_max_v = 12.0;
    pw.supercap_v_min_v = 6.0;
    pw.p_electronics_w = 3.0;
    pw.p_hover_budget_w = 30.0; // ≤ 30 W moving; core §C.2
    pw.p_sleep_budget_w = 1.0;

    let c = &mut p.control;
    c.rates.fast_decim = 4; // 5 kHz; core §A.5
    let lt = &mut c.levitation;
    lt.roll_active = false; // top-hung, gravity-stabilised; LEV-001
    lt.lift_time_max_s = 1.0;
    lt.land_speed_mps = 3.0e-3;
    let mo = &mut c.motion;
    mo.omega_v_hz = 3.0;
    mo.a_max_mps2 = 0.15;
    mo.a_max_sensed_mps2 = 0.15;
    mo.j_max_mps3 = 1.5;
    mo.pos_tol_closed_m = 1.0e-3;
    mo.pos_tol_travel_m = 5.0e-3;
    mo.terminus.tau_s = 0.6;
    mo.terminus.j_mps3 = 1.5;
    let hp = &mut c.haptic;
    hp.mass_ratio = 5.0;
    hp.b_v_n_s_per_m = 450.0;
    hp.k_o_hz = 8.0;
    hp.n_detents = 1;
    hp.detent_x_m = [3.0, 0.0, 0.0, 0.0];
    hp.detent_f_n = 20.0;
    hp.detent_width_m = 5.0e-3;
    hp.soft_close_omega_hz = 0.5;
    hp.soft_close_capture_m = 115.0e-3;
    hp.soft_close_v_final_mps = 20.0e-3;
    hp.hand_onset_s = 20.0e-3; // plan §4b (20 ms on WALL)
    hp.obstruction.residual_n = 60.0;
    hp.obstruction.persist_s = 35.0e-3;
    hp.obstruction.reverse_m = 0.05;
    hp.obstruction.release_to_zero = false;
    let it = &mut c.intent;
    it.f_th_n = 25.0;
    it.t_th_s = 120.0e-3;
    it.dx_th_m = 3.0e-3;
    it.confirm_max_s = 170.0e-3; // HAP-005
    it.push_to_open = false;
    let po = &mut c.seismic.policy;
    po.action = SeismicAction::Lock; // freeze → centre if reachable → land + lock; core §C.2
    po.center_if_reachable = true; // F11
    po.lock_after_land = true;
    c.supervisor.hover_timeout_s = 30.0; // land at park; design choice

    p.safety.p_max_kgmps = PEDESTRIAN_P_MAX_KGMPS;
    let v = caps(&p);
    let sf = &mut p.safety;
    sf.v_open_max_mps = v; // 22.3/500 = 0.0446; F3
    sf.v_close_max_mps = v;
    sf.v_open_sensed_mps = 0.15; // PL d; core §C.2
    sf.v_close_sensed_mps = 0.15;
    sf.f_drive_max_n = 400.0;
    finish(p)
}

/// Infrastructure panel: 5 t, LAND + lock, hold-to-run at 0.025 m/s unsensed.
///
/// Roll is active. The preset uses twin pole rows at y ±0.4 m: with the single-rail
/// y ±0.1 m of core design §C.1 the discrete roll loop loses stability at τ_e ≈ 1.2 ms
/// (LEV-020 needs 2 ms).
pub fn heavy() -> SystemParams {
    let mut p = common(RailClass::Heavy);
    let d = &mut p.design;
    d.rest_policy = RestPolicy::LandLock; // PWR-001
    d.operating_mode = OperatingMode::HoldToRun; // unsensed HEAVY is hold-to-run; F5, SAF-003
    let cr = &mut d.carriage;
    cr.m_star_kg = 5000.0;
    cr.m_rated_kg = 5000.0;
    cr.m_op_min_kg = 4500.0; // F18 †
    cr.j_kgm2 = box_inertia(5000.0, 3.0, 1.0, 2.5); // 3 × 1 × 2.5 m unit; design choice
    cr.cg_m = [0.0, 0.0, 1.25]; // body rides above wrap-around bogies; design choice
    cr.top_hung = false;
    cr.length_m = 3.0;

    let s = &mut d.suspension;
    s.n_magnets = 8; // 2 bogies × 4; core §C.1
    s.stations = stations(&[
        lift(1.3, 0.4),
        lift(1.3, -0.4),
        lift(0.9, 0.4),
        lift(0.9, -0.4),
        lift(-0.9, 0.4),
        lift(-0.9, -0.4),
        lift(-1.3, 0.4),
        lift(-1.3, -0.4),
    ]); // bogies at x ±1.1 m (±0.2 m), twin rows y ±0.4 m (LEV-020); design choice
    s.h_m_m = 18.0e-3;
    s.pole_w_m = 43.0e-3; // core §C.2 (43×500)
    s.b_g_t = 0.60;
    s.turns = 200.0;
    s.r_coil_ohm = 0.6;
    s.i_cont_a = 22.0;
    s.i_pulse_a = 40.0;
    s.g_star_m = 10.0e-3;
    s.g_up_m = 8.0e-3;
    s.g_land_m = 12.5e-3;
    s.g_seat_m = 0.0;
    s.zp_band_m = [8.8e-3, 11.5e-3];
    s.shroud = true; // shrouds required (80 mT); ENV-001

    let l = &mut d.lsm;
    l.kind = LsmKind::IronCoreSegmented;
    l.tau_p_m = 50.0e-3;
    l.k_f_n_per_a = 100.0; // design choice (2000 N at 20 A)
    l.k_e_v_s_per_m = derive::lsm_k_e(l.k_f_n_per_a); // K_e = ⅔·K_f; PRP-002, D5
    l.r_phase_ohm = 0.5;
    l.l_phase_h = 20.0e-3;
    l.f_peak_n = 2000.0;
    l.f_cont_n = 500.0;
    l.t_peak_s = 0.75;
    l.i_max_a = 20.0;
    l.ripple_frac = 0.01;
    l.cogging_n = 20.0; // 1 % of peak; design choice
    l.f_normal_n = 2500.0; // iron-core normal force at rated current; design choice
    l.segment_len_m = 2.0;
    l.segment_overlap_m = 0.2;

    let g = &mut d.guidance;
    g.fin_present = true;
    g.fin_clearance_m = 2.0e-3;
    g.pad_mu = 0.15;
    g.pad_clearance_m = 2.0e-3;
    g.c_sway_n_s_per_m = 1000.0;

    let dm = &mut d.damping;
    dm.b_dist_n_s_per_m = 100.0;
    dm.ez_cmax_n_s_per_m = 8000.0;
    dm.bumper_k_n_per_m = 1.125e6; // 5625 J in 100 mm
    dm.bumper_e_rated_j = 5625.0; // SAF-014
    dm.c_rail_n_s_per_m = 100.0;
    dm.f_hyst_n = 50.0;

    let ld = &mut d.landing;
    ld.kind = LandingKind::Rollers;
    ld.upper_roller_rating_n = 65_000.0; // ≥ n·F_pm(g_up) = 61.6 kN
    ld.parking_lock = true; // LAND + lock; PWR-001

    let r = &mut d.rail;
    r.travel_m = 12.0;
    r.span_m = 3.0;
    r.joint_spacing_m = 3.0;
    r.joint_step_m = 0.3e-3;
    r.flange_t_m = 18.0e-3;

    d.air.leaf_area_m2 = 7.5;

    let th = &mut d.thermal;
    th.r_th_coil_k_per_w = 1.0;
    th.c_th_coil_j_per_k = 3000.0;
    th.r_th_mag_k_per_w = 0.5;
    th.c_th_mag_j_per_k = 6000.0;

    let sc = &mut d.sensors;
    sc.gap_noise_m = 5.0e-6; // core §C.2
    sc.coil_a_per_code = 50.0 / 2048.0;
    sc.coil_noise_a = 20.0e-3;
    sc.lsm_a_per_code = 30.0 / 2048.0;
    sc.v_bus_v_per_code = 400.0 / 4096.0;
    sc.v_hold_v_per_code = 60.0 / 4096.0;

    let pw = &mut d.power;
    pw.v_bus_v = 300.0; // core §C.2 (not SELV)
    pw.p_bus_max_w = 8000.0; // ≥ n·i_pulse²·R_coil = 8·40²·0.6 = 7680 W (lift pulse); design choice (W1.5)
    pw.v_chopper_v = 345.0; // 1.15·V_bus; PRP-010
    pw.supercap_c_f = 10.0; // 48 V module: 3 × landing need; SAF-009
    pw.supercap_v_max_v = 48.0;
    pw.supercap_v_min_v = 24.0;
    pw.p_electronics_w = 10.0;
    pw.p_hover_budget_w = 100.0; // ≤ 100 W moving; core §C.2
    pw.p_sleep_budget_w = 5.0;

    let c = &mut p.control;
    c.rates.fast_decim = 4;
    let lt = &mut c.levitation;
    lt.roll_active = true; // LEV-001
    lt.lift_time_max_s = 2.0;
    lt.land_speed_mps = 2.0e-3;
    let mo = &mut c.motion;
    mo.omega_v_hz = 2.0;
    mo.a_max_mps2 = 0.08; // F5
    mo.a_max_sensed_mps2 = 0.2; // core §C.2
    mo.j_max_mps3 = 1.0;
    mo.pos_tol_closed_m = 1.0e-3;
    mo.pos_tol_travel_m = 5.0e-3;
    mo.terminus.tau_s = 1.0;
    mo.terminus.j_mps3 = 1.0;
    let hp = &mut c.haptic;
    hp.mass_ratio = 5.0;
    hp.b_v_n_s_per_m = 2000.0;
    hp.k_o_hz = 5.0;
    hp.n_detents = 0; // none
    hp.detent_x_m = [0.0; 4];
    hp.detent_f_n = 0.0;
    hp.detent_width_m = 5.0e-3;
    hp.soft_close_omega_hz = 0.5;
    hp.soft_close_capture_m = 150.0e-3;
    hp.soft_close_v_final_mps = 10.0e-3;
    hp.hand_onset_s = 20.0e-3;
    hp.obstruction.residual_n = 120.0;
    hp.obstruction.persist_s = 35.0e-3;
    hp.obstruction.reverse_m = 0.0;
    hp.obstruction.release_to_zero = true;
    let it = &mut c.intent;
    it.f_th_n = 40.0;
    it.t_th_s = 150.0e-3;
    it.dx_th_m = 0.0; // force only (plus hold-to-run); core §C.2
    it.confirm_max_s = 170.0e-3;
    it.push_to_open = false;
    let po = &mut c.seismic.policy;
    po.action = SeismicAction::Lock;
    po.center_if_reachable = true;
    po.lock_after_land = true;
    c.supervisor.hover_timeout_s = 30.0;

    let sf = &mut p.safety;
    sf.p_max_kgmps = 0.0; // hold-to-run: momentum cap not applicable (operator present); core §C.2
    sf.v_open_max_mps = 0.025; // hold-to-run 0.025 m/s (KE 1.56 J ≤ 1.69); core §C.2, SAF-002
    sf.v_close_max_mps = 0.025;
    sf.v_open_sensed_mps = 0.10; // PL d; core §C.2
    sf.v_close_sensed_mps = 0.10;
    sf.f_drive_max_n = 400.0; // caps a_max at 0.08; F5
    finish(p)
}

/// The preset for `class`.
pub fn by_class(class: RailClass) -> SystemParams {
    match class {
        RailClass::Drawer => drawer(),
        RailClass::Door => door(),
        RailClass::Wall => wall(),
        RailClass::Heavy => heavy(),
    }
}

/// Parses a lowercase class name (`"drawer" | "door" | "wall" | "heavy"`).
pub fn class_from_name(name: &str) -> Option<RailClass> {
    match name {
        "drawer" => Some(RailClass::Drawer),
        "door" => Some(RailClass::Door),
        "wall" => Some(RailClass::Wall),
        "heavy" => Some(RailClass::Heavy),
        _ => None,
    }
}

/// The lowercase name of `class` (inverse of [`class_from_name`]).
pub const fn class_name(class: RailClass) -> &'static str {
    match class {
        RailClass::Drawer => "drawer",
        RailClass::Door => "door",
        RailClass::Wall => "wall",
        RailClass::Heavy => "heavy",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::derive::{LevMode, bumper_rating, v_low_energy};

    #[test]
    fn by_class_and_names_roundtrip() {
        for c in CLASSES {
            assert_eq!(by_class(c).design.class, c);
            assert_eq!(class_from_name(class_name(c)), Some(c));
        }
        assert_eq!(class_from_name("sentinel"), None);
        assert_eq!(drawer().design.rest_policy, RestPolicy::Seat);
        assert_eq!(heavy().design.rest_policy, RestPolicy::LandLock);
    }

    // req: FAB-SW-004
    #[test]
    fn all_validate() {
        for c in CLASSES {
            let p = by_class(c);
            assert_eq!(p.validate(), Ok(()), "{c:?} preset validates (SW-004)");
            assert_eq!(
                crate::crc::verify_seal(&p),
                Ok(()),
                "{c:?} preset is sealed (SAF-015)"
            );
            assert_eq!(by_class(c), p, "{c:?} presets are deterministic");
        }
    }

    // req: FAB-SAF-002
    #[test]
    fn speed_caps() {
        // (E): the unsensed caps are the closed forms at the rated mass (core §E.5 anchors
        // 0.2654 / 0.1838 / 0.152 / 0.0446 / 0.025 m/s at 4 s.f.).
        let d = drawer();
        let v = v_low_energy(48.0, PEDESTRIAN_E_MAX_J);
        assert_eq!(
            d.safety.v_open_max_mps, v,
            "DRAWER open = √(3.38/48) ((E); F7)"
        );
        assert_eq!(d.safety.v_close_max_mps, v, "DRAWER close = open ((E); F7)");
        assert!(
            fmath::abs(v - 0.2654) <= 5e-5,
            "DRAWER {v} vs 0.2654 (4 s.f.; SAF-002)"
        );
        let o = door();
        let v = v_low_energy(100.0, PEDESTRIAN_E_MAX_J);
        assert_eq!(
            o.safety.v_open_max_mps, v,
            "DOOR open = √(3.38/100) ((E); SAF-002)"
        );
        assert!(
            fmath::abs(v - 0.1838) <= 5e-5,
            "DOOR open {v} vs 0.1838 (4 s.f.; SAF-002)"
        );
        assert_eq!(
            o.safety.v_close_max_mps, 0.152,
            "DOOR close 0.152 ((E); SAF-002)"
        );
        let w = wall();
        assert_eq!(
            w.safety.v_open_max_mps,
            PEDESTRIAN_P_MAX_KGMPS / 500.0,
            "WALL 22.3/500 ((E); F3)"
        );
        assert!(
            fmath::abs(w.safety.v_open_max_mps - 0.0446) <= 1e-15,
            "WALL 0.0446 ((E); SAF-002)"
        );
        let h = heavy();
        assert_eq!(
            h.safety.v_open_max_mps, 0.025,
            "HEAVY 0.025 hold-to-run ((E); SAF-002)"
        );
        assert!(
            bumper_rating(5000.0, 0.025) <= PEDESTRIAN_E_MAX_J,
            "HEAVY KE at cap ≤ 1.69 J (core §C.2 row 'KE at cap')"
        );
        for c in CLASSES {
            let p = by_class(c);
            let e = bumper_rating(p.design.carriage.m_rated_kg, p.safety.v_open_max_mps);
            assert!(
                e <= PEDESTRIAN_E_MAX_J * (1.0 + 1e-12),
                "{c:?}: KE at cap {e} J ≤ 1.69 (SAF-001)"
            );
        }
    }

    #[test]
    fn rest_policy_defaults() {
        let want = [
            RestPolicy::Seat,
            RestPolicy::Hover,
            RestPolicy::Land,
            RestPolicy::LandLock,
        ];
        for (c, r) in CLASSES.iter().zip(want) {
            assert_eq!(
                by_class(*c).design.rest_policy,
                r,
                "{c:?} rest policy (PWR-001)"
            );
        }
    }

    #[test]
    fn corrected_values() {
        let d = drawer();
        assert_eq!(
            d.design.rail.travel_m, 0.30,
            "DRAWER travel 0.30 m (plan §2b.3)"
        );
        assert_eq!(d.design.suspension.i_cont_a, 4.0, "i_cont (plan §2b.4)");
        assert_eq!(d.design.suspension.i_pulse_a, 12.0, "i_pulse (plan §2b.4)");
        assert_eq!(d.design.suspension.t_pulse_s, 0.2, "t_pulse (plan §2b.4)");
        assert_eq!(
            d.safety.i_coil_cont_a, 4.0,
            "monitor mirrors the I²t budget (plan §2b.4)"
        );
        assert_eq!(
            d.design.landing.seat_z_clearance_m, 0.15e-3,
            "seat z clearance (plan §2b.4)"
        );
        assert!(
            d.design.landing.seat_x_compliance_m >= 2.0e-3,
            "seat x compliance ≥ 2 mm (plan §2b.4)"
        );
        for c in CLASSES {
            let p = by_class(c);
            assert_eq!(
                p.design.landing.rollers_mu, 0.003,
                "{c:?}: roller μ 0.003 (plan §2b.19)"
            );
            assert_eq!(
                p.control.levitation.current_bw_hz, 1000.0,
                "{c:?}: ω_c = 2π·1 kHz (plan §2b.2)"
            );
            assert_eq!(
                p.control.levitation.omega_ratio, 2.5,
                "{c:?}: ω = 2.5·p (plan §2b.1)"
            );
            assert!(
                !p.control.motion.inertial.enable,
                "{c:?}: inertial off (plan §4b)"
            );
            assert!(
                !p.control.seismic.isolation.enable,
                "{c:?}: isolation off (plan §2b.5)"
            );
            assert!(!p.control.seismic.amd.enable, "{c:?}: AMD off");
            assert_eq!(
                p.design.damping.k_edge_n_per_m, 10.0e3,
                "{c:?}: k_edge 10 N/mm (F6)"
            );
            assert_eq!(
                p.design.suspension.tau_eddy_s, p.control.levitation.tau_e_nominal_s,
                "{c:?}: plant eddy lag equals the design point (LEV-007 at nominal)"
            );
            let active_roll = crate::derive::mode_active(&p, LevMode::Roll);
            assert_eq!(
                active_roll,
                matches!(c, RailClass::Drawer | RailClass::Heavy),
                "{c:?}: roll active on DRAWER/HEAVY only (LEV-001)"
            );
            let ratio = crate::derive::stop_ratio(
                &p.design,
                p.design.suspension.g_land_m,
                p.design.carriage.m_op_min_kg,
            );
            assert!(ratio <= 0.90, "{c:?}: stay-landed {ratio} ≤ 0.90 (F18)");
            let bump = bumper_rating(p.design.carriage.m_rated_kg, bumper_basis_speed(c));
            assert!(
                p.design.damping.bumper_e_rated_j >= bump,
                "{c:?}: bumper {} J ≥ ½mv² = {bump} (SAF-014)",
                p.design.damping.bumper_e_rated_j
            );
        }
        let w = wall();
        let f18 = crate::derive::stop_ratio(&w.design, w.design.suspension.g_land_m, 450.0);
        assert!(
            fmath::abs(f18 - 0.863) <= 5e-4,
            "WALL F18 {f18} vs 0.863 (plan §2b.19)"
        );
        // Balanced pole length reproduces the rounded table poles (core §C.2).
        let l_mm = [32.0, 51.0, 100.0, 500.0];
        for (c, want) in CLASSES.iter().zip(l_mm) {
            let l = by_class(*c).design.suspension.pole_l_m * 1e3;
            assert!(
                fmath::abs(l - want) / want <= 0.006,
                "{c:?}: pole l = {l} mm vs {want} (±0.6 %; core §C.2)"
            );
        }
    }
}
