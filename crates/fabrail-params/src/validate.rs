//! Parameter validation: ranges (from `meta::FIELDS`) and cross-field rules, returning
//! the first violation as `ParamError { path, reason }` (core design §B.2; SW-004).
//!
//! Order: finite values and ranges; geometry and gap ordering; stops (LEV-006, F18);
//! magnetics and currents (LEV-015, LEV-017, F1, F17); power (ENV-002, PWR-005, PRP-010,
//! SAF-009); rates; levitation modes and loop stability (LEV-001, LEV-007); motion and
//! termini (PRP-002/003, F5, PHYSICS §16.1, plan §4b); haptics (HAP-002, HAP-009); the
//! safety block (SAF-001…005, SAF-014, SAF-015 copies, SAF-020, mirrors); seismic (SEI-001,
//! SEI-011, detector and research-tier gates). Paths are the dotted `meta::FIELDS` paths.
//!
//! The CRC itself is checked by `crc::verify_seal` (boot), not here, so an edited block
//! validates before it is resealed; the diverse copies ARE checked here (they must equal
//! the design values; `crc::seal` refreshes them).

use crate::derive::{self, LevMode};
use crate::{AccelMount, LandingKind, OperatingMode, ParamError, SystemParams, meta, presets};
use fabrail_types::{
    BASE_TICK_HZ, EgressRole, PresenceSensing, RailClass, RestPolicy, SeismicAction, fmath,
};

/// Relative slack for comparisons against closed forms computed from the same inputs.
const EPS: f64 = 1.0e-9;

/// Validates `p`; `Err` carries the first violated field path and the reason.
pub fn validate(p: &SystemParams) -> Result<(), ParamError> {
    ranges(p)?;
    geometry(p)?;
    stops(p)?;
    magnetics(p)?;
    power(p)?;
    rates(p)?;
    levitation(p)?;
    motion(p)?;
    haptics(p)?;
    safety(p)?;
    seismic(p)?;
    Ok(())
}

fn err(path: &'static str, reason: &'static str) -> Result<(), ParamError> {
    Err(ParamError::new(path, reason))
}

fn check(ok: bool, path: &'static str, reason: &'static str) -> Result<(), ParamError> {
    if ok { Ok(()) } else { err(path, reason) }
}

/// a ≤ b allowing EPS relative slack.
fn le(a: f64, b: f64) -> bool {
    a <= b + EPS * fmath::fmax(fmath::abs(a), fmath::abs(b))
}

/// a == b within EPS relative.
fn close(a: f64, b: f64) -> bool {
    fmath::abs(a - b) <= EPS * fmath::fmax(fmath::abs(a), fmath::abs(b))
}

fn ranges(p: &SystemParams) -> Result<(), ParamError> {
    for (i, f) in meta::FIELDS.iter().enumerate() {
        let v = meta::value_at(p, i).unwrap_or(f64::NAN);
        if !v.is_finite() {
            return err(f.path, "not a finite number");
        }
        if v < f.min {
            return err(f.path, "below the minimum of its meta::FIELDS range");
        }
        if v > f.max {
            return err(f.path, "above the maximum of its meta::FIELDS range");
        }
    }
    Ok(())
}

const STATION_NORMAL: [&str; 8] = [
    "design.suspension.stations.0.normal_sign",
    "design.suspension.stations.1.normal_sign",
    "design.suspension.stations.2.normal_sign",
    "design.suspension.stations.3.normal_sign",
    "design.suspension.stations.4.normal_sign",
    "design.suspension.stations.5.normal_sign",
    "design.suspension.stations.6.normal_sign",
    "design.suspension.stations.7.normal_sign",
];

const DETENT_X: [&str; 4] = [
    "control.haptic.detent_x_m.0",
    "control.haptic.detent_x_m.1",
    "control.haptic.detent_x_m.2",
    "control.haptic.detent_x_m.3",
];

fn geometry(p: &SystemParams) -> Result<(), ParamError> {
    let d = &p.design;
    let s = &d.suspension;
    let c = &d.carriage;
    let n = usize::from(s.n_magnets);
    for (k, st) in s.stations.iter().enumerate().take(n) {
        check(
            st.normal_sign == 1.0 || st.normal_sign == -1.0,
            STATION_NORMAL[k],
            "an active station needs normal_sign +1 (lift) or −1 (hold-down)",
        )?;
    }
    check(
        s.stations
            .iter()
            .take(n)
            .any(|st| st.normal_sign > 0.0 && st.has_pm),
        "design.suspension.n_magnets",
        "at least one active lift element with a PM is required",
    )?;
    check(
        c.m_op_min_kg <= c.m_star_kg,
        "design.carriage.m_op_min_kg",
        "must not exceed m_star_kg",
    )?;
    check(
        c.m_star_kg <= c.m_rated_kg,
        "design.carriage.m_star_kg",
        "must not exceed m_rated_kg",
    )?;
    // The PM must balance m* at g* before any stop ratio means anything (PHYSICS §3).
    let bal = derive::stop_ratio(d, s.g_star_m, c.m_star_kg);
    check(
        fmath::abs(bal - 1.0) <= 0.05,
        "design.suspension.pole_l_m",
        "PM force at g_star_m must balance m_star_kg within 5 % (derive::balance_pole_length)",
    )?;
    // Gap ordering: g_up < band_lo ≤ g* ≤ band_hi < g_land (core §C.2; F1).
    check(
        s.g_up_m < s.zp_band_m[0],
        "design.suspension.g_up_m",
        "the upper stop must lie below the zero-power band",
    )?;
    check(
        s.zp_band_m[0] <= s.g_star_m,
        "design.suspension.zp_band_m.0",
        "band lower edge must not exceed g_star_m",
    )?;
    check(
        s.g_star_m <= s.zp_band_m[1],
        "design.suspension.zp_band_m.1",
        "band upper edge must not be below g_star_m",
    )?;
    check(
        s.zp_band_m[1] < s.g_land_m,
        "design.suspension.g_land_m",
        "the landing gap must lie above the zero-power band",
    )?;
    if matches!(d.rest_policy, RestPolicy::Seat) {
        check(
            s.g_seat_m > s.g_star_m && s.g_seat_m < s.g_land_m,
            "design.suspension.g_seat_m",
            "SEAT rest needs g_star_m < g_seat_m < g_land_m (F17)",
        )?;
        check(
            matches!(d.landing.kind, LandingKind::Seat),
            "design.landing.kind",
            "SEAT rest needs the captive seat landing kind (F17)",
        )?;
    }
    if matches!(d.rest_policy, RestPolicy::LandLock) {
        check(
            d.landing.parking_lock,
            "design.landing.parking_lock",
            "LAND + lock rest needs a parking lock (PWR-001)",
        )?;
    }
    check(
        c.top_hung == (c.cg_m[2] < 0.0),
        "design.carriage.top_hung",
        "top-hung bodies have their CoM below the pole plane (cg_m.2 < 0) and vice versa",
    )?;
    Ok(())
}

fn stops(p: &SystemParams) -> Result<(), ParamError> {
    let d = &p.design;
    let s = &d.suspension;
    let c = &d.carriage;
    // Upper stop: F/(m*·g) ≤ 1.30 except where rated upper rollers take the snap-up
    // (DRAWER, F2); every class needs rollers rated for n·F_pm(g_up) (LEV-006).
    if !matches!(d.class, RailClass::Drawer) {
        check(
            derive::stop_ratio(d, s.g_up_m, c.m_star_kg) <= 1.30,
            "design.suspension.g_up_m",
            "upper-stop magnet force exceeds 1.30·m*·g (LEV-006)",
        )?;
    }
    check(
        d.landing.upper_roller_rating_n
            >= derive::n_lift(s) * derive::pm_force(s, s.g_up_m, 0.0, 0.0),
        "design.landing.upper_roller_rating_n",
        "below the magnet snap-up force n·F_pm(g_up) (LEV-006, F2)",
    )?;
    // Stay-landed: F_pm(g_land)/(m_op_min·g) ≤ 0.90 (F18).
    check(
        derive::stop_ratio(d, s.g_land_m, c.m_op_min_kg) <= 0.90,
        "design.suspension.g_land_m",
        "stay-landed ratio F_pm(g_land)/(m_op_min·g) above 0.90 (F18)",
    )
}

fn magnetics(p: &SystemParams) -> Result<(), ParamError> {
    let d = &p.design;
    let s = &d.suspension;
    let c = &d.carriage;
    check(
        s.b_g_t < s.b_r_t,
        "design.suspension.b_g_t",
        "design gap flux must be below B_r (PHYSICS §3 sizing)",
    )?;
    check(
        d.rail.flange_t_m >= derive::flange_min_m(p) * (1.0 - EPS),
        "design.rail.flange_t_m",
        "thinner than B_g·w/B_sat,rail (LEV-015)",
    )?;
    check(
        s.i_cont_a <= s.i_pulse_a,
        "design.suspension.i_cont_a",
        "must not exceed i_pulse_a (I²t budget)",
    )?;
    check(
        s.i_pulse_a * s.r_coil_ohm <= d.power.v_bus_v,
        "design.suspension.i_pulse_a",
        "the bus cannot drive i_pulse through the coil: i_pulse·R > V_bus (F17)",
    )?;
    let lift_gap = if matches!(d.rest_policy, RestPolicy::Seat) {
        s.g_seat_m
    } else {
        s.g_land_m
    };
    for m in [c.m_rated_kg, c.m_op_min_kg] {
        check(
            fmath::abs(derive::bias_current(d, lift_gap, m)) <= s.i_pulse_a,
            "design.suspension.i_pulse_a",
            "cannot lift the rated or minimum mass from rest within i_pulse (LEV-012, LEV-017, F17)",
        )?;
        let g = derive::float_gap(p, m);
        check(
            fmath::abs(derive::bias_current(d, g, m)) <= s.i_cont_a,
            "design.suspension.zp_band_m.0",
            "band-edge bias current exceeds i_cont for the mass range (F1)",
        )?;
    }
    Ok(())
}

fn power(p: &SystemParams) -> Result<(), ParamError> {
    let d = &p.design;
    let pw = &d.power;
    check(
        pw.pwm_hz >= 20_000.0,
        "design.power.pwm_hz",
        "PWM below 20 kHz is audible (ENV-002)",
    )?;
    check(
        pw.v_bus_v < 60.0 || matches!(d.class, RailClass::Heavy),
        "design.power.v_bus_v",
        "above the 60 V SELV limit; only HEAVY may exceed it (PWR-005)",
    )?;
    check(
        derive::force_slew_available(p) >= derive::force_slew_required(p),
        "design.power.v_bus_v",
        "coil force slew k_i·V_bus/L below m*·g₀/n·ω (PWR-005)",
    )?;
    check(
        pw.v_chopper_v > pw.v_bus_v && pw.v_chopper_v <= 1.16 * pw.v_bus_v,
        "design.power.v_chopper_v",
        "the brake chopper must engage within 16 % above the bus (PRP-010)",
    )?;
    check(
        pw.v_chopper_v < 60.0 || matches!(d.class, RailClass::Heavy),
        "design.power.v_chopper_v",
        "the chopper threshold must stay below 60 V SELV (PWR-005)",
    )?;
    check(
        pw.supercap_v_min_v < pw.supercap_v_max_v,
        "design.power.supercap_v_min_v",
        "must be below supercap_v_max_v",
    )?;
    let h = derive::holdup_energy(p);
    check(
        h.avail_j >= p.control.supervisor.holdup_margin * h.need_j,
        "design.power.supercap_c_f",
        "hold-up energy below margin × one Safe Landing (SAF-009)",
    )?;
    check(
        pw.p_sleep_budget_w <= pw.p_hover_budget_w,
        "design.power.p_sleep_budget_w",
        "must not exceed p_hover_budget_w",
    )
}

fn rates(p: &SystemParams) -> Result<(), ParamError> {
    let r = &p.control.rates;
    check(
        r.base_hz == BASE_TICK_HZ,
        "control.rates.base_hz",
        "must equal fabrail_types::BASE_TICK_HZ (20 kHz; core §A.5)",
    )?;
    check(
        r.sup_decim.is_multiple_of(r.fast_decim),
        "control.rates.fast_decim",
        "must divide sup_decim (LET release alignment; core §A.5)",
    )?;
    check(
        r.seismic_decim.is_multiple_of(r.sup_decim),
        "control.rates.seismic_decim",
        "must be a multiple of sup_decim (core §A.5)",
    )?;
    check(
        f64::from(r.base_hz) <= p.design.power.pwm_hz,
        "design.power.pwm_hz",
        "the base (current-loop) rate must not exceed the PWM rate (core §A.5)",
    )
}

fn levitation(p: &SystemParams) -> Result<(), ParamError> {
    let lt = &p.control.levitation;
    let roll_pole = derive::mode_pole(p, LevMode::Roll);
    if lt.roll_active {
        check(
            roll_pole > 0.0,
            "control.levitation.roll_active",
            "roll is gravity-stabilised: an active roll loop has no unstable pole (LEV-001)",
        )?;
    } else {
        check(
            roll_pole == 0.0,
            "control.levitation.roll_active",
            "roll is magnetically unstable and must be actively controlled (LEV-001)",
        )?;
    }
    let tau_plant = p.design.suspension.tau_eddy_s;
    for mode in LevMode::ALL {
        if !derive::mode_active(p, mode) {
            continue;
        }
        let k = derive::zp_gains_discrete(p, mode);
        check(
            k.kp_a_per_m > 0.0 && k.kd_a_s_per_m > 0.0 && k.kappa_per_s > 0.0,
            "control.levitation.omega_ratio",
            "the discrete zero-power design has no positive solution (PHYSICS §15)",
        )?;
        check(
            derive::zp_spectral_radius(p, mode, &k, lt.tau_e_nominal_s) < 1.0,
            "control.levitation.omega_ratio",
            "discrete zero-power loop unstable at the design eddy lag (LEV-007)",
        )?;
        check(
            derive::zp_spectral_radius(p, mode, &k, tau_plant) < 1.0,
            "design.suspension.tau_eddy_s",
            "discrete zero-power loop unstable at the plant eddy lag (LEV-020)",
        )?;
    }
    check(
        lt.seat_time_s <= 0.0 || matches!(p.design.rest_policy, RestPolicy::Seat),
        "control.levitation.seat_time_s",
        "a seat step needs the SEAT rest policy",
    )
}

fn motion(p: &SystemParams) -> Result<(), ParamError> {
    let d = &p.design;
    let mo = &p.control.motion;
    let l = &d.lsm;
    let sf = &p.safety;
    check(
        close(l.k_e_v_s_per_m, crate::derive::lsm_k_e(l.k_f_n_per_a)),
        "design.lsm.k_e_v_s_per_m",
        "K_e must equal ⅔·K_f (amplitude-invariant dq; PRP-002, PHYSICS §7)",
    )?;
    check(
        le(l.f_peak_n, l.k_f_n_per_a * l.i_max_a),
        "design.lsm.i_max_a",
        "K_f·i_max cannot reach f_peak_n",
    )?;
    check(
        l.f_cont_n <= l.f_peak_n,
        "design.lsm.f_cont_n",
        "must not exceed f_peak_n",
    )?;
    check(
        le(d.carriage.m_rated_kg * mo.a_max_mps2, sf.f_drive_max_n),
        "control.motion.a_max_mps2",
        "m_rated·a_max exceeds the driving-force limit (F5)",
    )?;
    check(
        l.f_peak_n >= derive::thrust_required(p),
        "design.lsm.f_peak_n",
        "peak thrust below m_rated·a_max + drag (PRP-003)",
    )?;
    let v_max = fmath::fmax(sf.v_open_max_mps, sf.v_close_max_mps);
    check(
        l.f_cont_n >= derive::drag_force(p, v_max),
        "design.lsm.f_cont_n",
        "continuous thrust below the drag at the speed cap (PRP-003)",
    )?;
    check(
        mo.a_max_sensed_mps2 >= mo.a_max_mps2,
        "control.motion.a_max_sensed_mps2",
        "must not be below a_max_mps2",
    )?;
    let tm = &mo.terminus;
    check(
        tm.j_mps3 <= mo.j_max_mps3 * (1.0 + EPS),
        "control.motion.terminus.j_mps3",
        "the blend jerk exceeds j_max_mps3",
    )?;
    check(
        tm.tau_s >= derive::tau_min(v_max, mo.a_max_mps2, tm.j_mps3),
        "control.motion.terminus.tau_s",
        "below tau_min = v/a_max − a_max/(2j) (PHYSICS §16.1)",
    )?;
    check(
        core::f64::consts::PI * mo.omega_v_hz >= 3.0 / tm.tau_s,
        "control.motion.omega_v_hz",
        "ω_v/2 below 3/τ: terminus error poles slower than the exponential (plan §4b)",
    )?;
    check(
        tm.overtravel_m > mo.gov_margin_m,
        "control.motion.terminus.overtravel_m",
        "must exceed gov_margin_m so the governor never clips the exponential tail (plan §4b)",
    )
}

fn haptics(p: &SystemParams) -> Result<(), ParamError> {
    let d = &p.design;
    let hp = &p.control.haptic;
    let load_cell = d.sensors.handle_n_per_code > 0.0;
    let limit = if load_cell {
        20.0
    } else if matches!(d.class, RailClass::Drawer) {
        2.0
    } else {
        5.0
    };
    check(
        hp.mass_ratio <= limit,
        "control.haptic.mass_ratio",
        "m/M_v above the class bound: 2 DRAWER, 5 sensorless, 20 with a load cell (HAP-002)",
    )?;
    let t = derive::fast_period_s(p);
    let b = d.damping.b_dist_n_s_per_m;
    let k_max = derive::passivity_k_max(b, t, t);
    let k_v = hp.k_virtual_frac * 2.0 * b / t;
    check(
        le(k_v, k_max),
        "control.haptic.k_virtual_frac",
        "virtual stiffness above the passivity bound 2b/(T + 2T_d), T_d = T (HAP-009)",
    )?;
    if hp.n_detents > 0 {
        let slope = 2.0 * core::f64::consts::PI * hp.detent_f_n / hp.detent_width_m;
        check(
            le(slope, k_max),
            "control.haptic.detent_f_n",
            "detent stiffness 2π·F/w above the passivity bound (HAP-009)",
        )?;
    }
    let travel = d.rail.travel_m;
    for (k, x) in hp
        .detent_x_m
        .iter()
        .enumerate()
        .take(usize::from(hp.n_detents))
    {
        check(
            *x >= 0.0 && *x <= travel,
            DETENT_X[k],
            "an active detent must lie within the travel",
        )?;
    }
    Ok(())
}

fn safety(p: &SystemParams) -> Result<(), ParamError> {
    let d = &p.design;
    let s = &d.suspension;
    let sf = &p.safety;
    let sensed = matches!(d.presence, PresenceSensing::PlD);
    check(
        sensed
            || !matches!(d.class, RailClass::Heavy)
            || matches!(d.operating_mode, OperatingMode::HoldToRun),
        "design.operating_mode",
        "HEAVY automatic mode requires PL d presence sensing; use hold_to_run (SAF-003, F5)",
    )?;
    check(
        sf.mass_rated_kg == d.carriage.m_rated_kg,
        "safety.mass_rated_kg",
        "must equal design.carriage.m_rated_kg (the monitor never uses m̂; SAF-012)",
    )?;
    check(
        sf.e_max_j <= presets::PEDESTRIAN_E_MAX_J,
        "safety.e_max_j",
        "above the 1.69 J low-energy cap (SAF-001)",
    )?;
    check(
        sf.p_max_kgmps > 0.0 || matches!(d.operating_mode, OperatingMode::HoldToRun),
        "safety.p_max_kgmps",
        "the momentum cap may be waived only in hold-to-run (SAF-002)",
    )?;
    // Presence gate (SAF-003).
    if !sensed {
        let cap = derive::unsensed_speed_cap(d.carriage.m_rated_kg, sf.e_max_j, sf.p_max_kgmps);
        check(
            le(sf.v_open_max_mps, cap),
            "safety.v_open_max_mps",
            "above min(√(2E/m), p/m) without PL d presence sensing (SAF-003)",
        )?;
        check(
            le(sf.v_close_max_mps, cap),
            "safety.v_close_max_mps",
            "above min(√(2E/m), p/m) without PL d presence sensing (SAF-003)",
        )?;
        check(
            sf.f_static_n <= presets::PEDESTRIAN_F_STATIC_N,
            "safety.f_static_n",
            "above 67 N unsensed static force (PRP-004, F4)",
        )?;
    }
    check(
        sf.v_open_sensed_mps >= sf.v_open_max_mps,
        "safety.v_open_sensed_mps",
        "the sensed cap must not be below the unsensed cap",
    )?;
    check(
        sf.v_close_sensed_mps >= sf.v_close_max_mps,
        "safety.v_close_sensed_mps",
        "the sensed cap must not be below the unsensed cap",
    )?;
    check(
        sf.f_static_final_n <= sf.f_static_sensed_n,
        "safety.f_static_final_n",
        "the 5 s static force must not exceed the sensed static force (SAF-004)",
    )?;
    check(
        sf.f_drive_max_n <= presets::PEDESTRIAN_F_DYNAMIC_N,
        "safety.f_drive_max_n",
        "above the 400 N / 0.75 s dynamic limit (SAF-004)",
    )?;
    // Every unsensed cap stays ≤ 400 N through the edge seal (SAF-005).
    check(
        d.damping.k_edge_n_per_m <= presets::EDGE_K_MAX_N_PER_M,
        "design.damping.k_edge_n_per_m",
        "closing-edge seal stiffer than 10 N/mm (F6)",
    )?;
    let v_cap = fmath::fmax(sf.v_open_max_mps, sf.v_close_max_mps);
    let e_cap = derive::bumper_rating(d.carriage.m_rated_kg, v_cap);
    let k_hand_edge = derive::series_k(presets::ISO15066_HAND_K_N_PER_M, d.damping.k_edge_n_per_m);
    check(
        derive::clamp_force(e_cap, k_hand_edge) <= presets::PEDESTRIAN_F_DYNAMIC_N,
        "safety.v_close_max_mps",
        "hand clamp force through the edge seal exceeds 400 N at the cap (SAF-005)",
    )?;
    // Bumpers (SAF-014, F13).
    check(
        d.damping.bumper_e_rated_j
            >= derive::bumper_rating(d.carriage.m_rated_kg, presets::bumper_basis_speed(d.class))
                * (1.0 - EPS),
        "design.damping.bumper_e_rated_j",
        "below ½·m_rated·v² at the seismic basis speed (SAF-014, F13)",
    )?;
    // Mirrors of the design block (plan §2b item 4).
    check(
        sf.i_coil_cont_a == s.i_cont_a,
        "safety.i_coil_cont_a",
        "must mirror design.suspension.i_cont_a (I²t; plan §2b item 4)",
    )?;
    check(
        sf.i_coil_pulse_a == s.i_pulse_a,
        "safety.i_coil_pulse_a",
        "must mirror design.suspension.i_pulse_a (plan §2b item 4)",
    )?;
    check(
        sf.t_coil_pulse_s == s.t_pulse_s,
        "safety.t_coil_pulse_s",
        "must mirror design.suspension.t_pulse_s (plan §2b item 4)",
    )?;
    check(
        sf.i_lsm_max_a == d.lsm.i_max_a,
        "safety.i_lsm_max_a",
        "must mirror design.lsm.i_max_a",
    )?;
    check(
        sf.gap_min_m < s.g_up_m,
        "safety.gap_min_m",
        "must lie below the upper stop g_up_m",
    )?;
    check(
        sf.gap_max_m > s.g_land_m,
        "safety.gap_max_m",
        "must lie above the landing gap g_land_m",
    )?;
    check(
        sf.x_min_m < d.rail.x_closed_m,
        "safety.x_min_m",
        "must lie beyond the closed stop",
    )?;
    check(
        sf.x_max_m > d.rail.x_closed_m + d.rail.travel_m,
        "safety.x_max_m",
        "must lie beyond full travel",
    )?;
    check(
        sf.v_hold_min_v >= d.power.supercap_v_min_v && sf.v_hold_min_v < d.power.supercap_v_max_v,
        "safety.v_hold_min_v",
        "must lie within the supercap operating range",
    )?;
    // Diverse copies (SAF-015); crc::seal refreshes them.
    check(
        sf.k_f_copy_n_per_a.to_bits() == d.lsm.k_f_n_per_a.to_bits(),
        "safety.k_f_copy_n_per_a",
        "diverse copy differs from design.lsm.k_f_n_per_a; reseal (SAF-015)",
    )?;
    check(
        sf.adc_a_per_code_copy.to_bits() == d.sensors.coil_a_per_code.to_bits(),
        "safety.adc_a_per_code_copy",
        "diverse copy differs from design.sensors.coil_a_per_code; reseal (SAF-015)",
    )?;
    check(
        sf.lsm_a_per_code_copy.to_bits() == d.sensors.lsm_a_per_code.to_bits(),
        "safety.lsm_a_per_code_copy",
        "diverse copy differs from design.sensors.lsm_a_per_code; reseal (SAF-015)",
    )?;
    check(
        sf.nm_per_count_copy == d.sensors.nm_per_count,
        "safety.nm_per_count_copy",
        "diverse copy differs from design.sensors.nm_per_count; reseal (SAF-015)",
    )?;
    check(
        sf.presence_copy == d.presence,
        "safety.presence_copy",
        "diverse copy differs from design.presence; reseal (SAF-003, SAF-015)",
    )?;
    let base = f64::from(p.control.rates.base_hz);
    check(
        f64::from(sf.watchdog_ticks) == fmath::round(p.control.supervisor.watchdog_s * base),
        "safety.watchdog_ticks",
        "must equal control.supervisor.watchdog_s at the base rate",
    )?;
    check(
        sf.heartbeat_max_ticks
            == u32::from(p.control.supervisor.heartbeat_periods)
                * u32::from(p.control.rates.sup_decim),
        "safety.heartbeat_max_ticks",
        "must equal heartbeat_periods × sup_decim",
    )?;
    // ISOLATE replacement envelope (SAF-020).
    check(
        sf.isolate.stroke_m > 0.0 && le(sf.isolate.stroke_m, p.control.seismic.isolation.stroke_m),
        "safety.isolate.stroke_m",
        "must be positive and within the isolation design stroke (SAF-020)",
    )?;
    check(
        sf.isolate.bumper_ke_j <= d.damping.bumper_e_rated_j,
        "safety.isolate.bumper_ke_j",
        "above the bumper rating (SAF-020)",
    )?;
    check(
        sf.isolate.f_cont_n > 0.0 && sf.isolate.f_cont_n <= d.lsm.f_cont_n,
        "safety.isolate.f_cont_n",
        "must be positive and within the LSM continuous thrust (SAF-020)",
    )?;
    check(
        sf.unattended.f_pinch_n <= presets::PEDESTRIAN_F_STATIC_SENSED_N,
        "safety.unattended.f_pinch_n",
        "above 150 N (plan §4b)",
    )
}

fn seismic(p: &SystemParams) -> Result<(), ParamError> {
    let d = &p.design;
    let se = &p.control.seismic;
    check(
        !matches!(d.sensors.accel_mount, AccelMount::Carriage),
        "design.sensors.accel_mount",
        "the EEW accelerometer must be on the structure or rail, not the carriage (SEI-001)",
    )?;
    let locks = se.policy.lock_after_land || matches!(se.policy.action, SeismicAction::Lock);
    if !matches!(d.egress, EgressRole::None) {
        check(
            !se.policy.lock_after_land,
            "control.seismic.policy.lock_after_land",
            "never lock on an egress path (SEI-011)",
        )?;
        check(
            !matches!(se.policy.action, SeismicAction::Lock),
            "control.seismic.policy.action",
            "never lock on an egress path (SEI-011)",
        )?;
    }
    if matches!(d.egress, EgressRole::FireRated) {
        check(
            !matches!(se.policy.action, SeismicAction::OpenAndStop),
            "control.seismic.policy.action",
            "fire-rated doors never open on a seismic action (SEI-011)",
        )?;
    }
    if matches!(se.policy.action, SeismicAction::OpenAndStop) {
        check(
            matches!(d.egress, EgressRole::NonRated),
            "control.seismic.policy.action",
            "OpenAndStop is only for non-rated egress doors (SEI-011)",
        )?;
    }
    if locks {
        check(
            d.landing.parking_lock,
            "design.landing.parking_lock",
            "a locking seismic action needs a parking lock",
        )?;
    }
    let dt = &se.detector;
    let fs = d.sensors.accel_odr_hz / f64::from(dt.aa_decim);
    let seis_rate = f64::from(p.control.rates.base_hz) / f64::from(p.control.rates.seismic_decim);
    check(
        close(fs, seis_rate),
        "control.seismic.detector.aa_decim",
        "accel ODR / aa_decim must equal the seismic group rate (core §A.5)",
    )?;
    check(
        dt.bp_lo_hz < dt.bp_hi_hz && dt.bp_hi_hz < 0.5 * fs,
        "control.seismic.detector.bp_hi_hz",
        "band-pass edges must satisfy lo < hi < Nyquist of the decimated stream",
    )?;
    check(
        dt.trig_off < dt.trig_on,
        "control.seismic.detector.trig_off",
        "must be below trig_on",
    )?;
    check(
        dt.sta_n < dt.lta_n,
        "control.seismic.detector.sta_n",
        "must be shorter than lta_n",
    )?;
    check(
        dt.win_short_s < dt.win_long_s,
        "control.seismic.detector.win_short_s",
        "must be shorter than win_long_s",
    )?;
    check(
        dt.pd_alert_m <= dt.pd_l3_m,
        "control.seismic.detector.pd_alert_m",
        "must not exceed pd_l3_m",
    )?;
    check(
        se.fusion.v_s_kmps < se.fusion.v_p_kmps,
        "control.seismic.fusion.v_s_kmps",
        "must be below v_p_kmps",
    )?;
    let t_min = derive::t_min_iso(d.carriage.m_rated_kg, se.isolation.stroke_m, d.lsm.f_cont_n);
    check(
        se.isolation.t_iso_s >= fmath::fmax(2.0, 1.1 * t_min) * (1.0 - EPS),
        "control.seismic.isolation.t_iso_s",
        "below max(2 s, 1.1·T_min(F_cont)) (core §C.2; plan §2b item 5)",
    )?;
    if se.isolation.enable {
        check(
            d.sensors.imu_present && matches!(d.class, RailClass::Wall | RailClass::Heavy),
            "control.seismic.isolation.enable",
            "isolation (research tier) needs WALL/HEAVY and a carriage IMU (core §C.2)",
        )?;
    }
    check(
        se.amd.f_max_n <= d.lsm.f_cont_n,
        "control.seismic.amd.f_max_n",
        "above the LSM continuous thrust",
    )?;
    let inr = &p.control.motion.inertial;
    if inr.enable {
        check(
            d.sensors.imu_present
                && d.sensors.rail_accel_present
                && matches!(d.class, RailClass::Drawer),
            "control.motion.inertial.enable",
            "inertial smoothing (research tier) needs DRAWER, a carriage IMU and a rail accelerometer (plan §4b)",
        )?;
    }
    check(
        inr.f_burst_n <= d.lsm.f_peak_n,
        "control.motion.inertial.f_burst_n",
        "burst force above the LSM peak thrust (plan §4b)",
    )?;
    if d.landing.seismic_latch.enabled {
        check(
            d.landing.seismic_latch.x_retention_n > 0.0,
            "design.landing.seismic_latch.x_retention_n",
            "a fitted latch needs a positive retention force (plan §4b)",
        )?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::presets::{self, CLASSES};

    fn path_of(p: &SystemParams) -> Option<&'static str> {
        validate(p).err().map(|e| e.path)
    }

    // req: FAB-LEV-006
    #[test]
    fn rejects_bad_stops() {
        // DOOR upper stop at 3 mm: F/(m*g) = ((2g*+h′)/(2g+h′))² = 1.69 > 1.30.
        let mut p = presets::door();
        p.design.suspension.g_up_m = 3.0e-3;
        p.safety.gap_min_m = 2.8e-3;
        assert_eq!(
            path_of(&p),
            Some("design.suspension.g_up_m"),
            "LEV-006 upper stop"
        );
        // DRAWER upper rollers rated 500 N < n·F_pm(g_up) = 551 N (F2).
        let mut p = presets::drawer();
        p.design.landing.upper_roller_rating_n = 500.0;
        assert_eq!(
            path_of(&p),
            Some("design.landing.upper_roller_rating_n"),
            "F2 roller rating"
        );
        // WALL landing gap 11.7 mm: stay-landed 0.93 > 0.90 (F18).
        let mut p = presets::wall();
        p.design.suspension.g_land_m = 11.7e-3;
        p.safety.gap_max_m = 12.0e-3;
        assert_eq!(
            path_of(&p),
            Some("design.suspension.g_land_m"),
            "F18 stay-landed"
        );
        // The presets pass (F18 0.824 / 0.849 / 0.863 / 0.863).
        for c in CLASSES {
            assert_eq!(validate(&presets::by_class(c)), Ok(()));
        }
    }

    // req: FAB-ENV-002
    #[test]
    fn pwm_above_audible() {
        let mut p = presets::door();
        p.design.power.pwm_hz = 16_000.0;
        assert_eq!(
            path_of(&p),
            Some("design.power.pwm_hz"),
            "ENV-002: 16 kHz rejected"
        );
        p.design.power.pwm_hz = 20_000.0;
        assert_eq!(validate(&p), Ok(()), "ENV-002: 20 kHz accepted");
    }

    // req: FAB-HAP-014
    #[test]
    fn rejects_tau_below_tau_min() {
        // DRAWER τ_min = 0.2654/1.5 − 1.5/40 = 0.139 s.
        let mut p = presets::drawer();
        p.control.motion.terminus.tau_s = 0.12;
        assert_eq!(
            path_of(&p),
            Some("control.motion.terminus.tau_s"),
            "HAP-014: τ < τ_min rejected"
        );
        p.control.motion.terminus.tau_s = 0.14;
        assert_eq!(validate(&p), Ok(()), "τ ≥ τ_min accepted");
        // ω_v/2 ≥ 3/τ (plan §4b): WALL at 1.5 Hz gives 4.7 < 5.
        let mut p = presets::wall();
        p.control.motion.omega_v_hz = 1.5;
        assert_eq!(path_of(&p), Some("control.motion.omega_v_hz"));
    }

    // req: FAB-SW-004
    #[test]
    fn error_paths() {
        type Case = (RailClass, fn(&mut SystemParams), &'static str);
        let cases: [Case; 26] = [
            (
                RailClass::Door,
                |p| p.design.carriage.m_star_kg = f64::NAN,
                "design.carriage.m_star_kg",
            ),
            (
                RailClass::Door,
                |p| p.design.suspension.n_magnets = 9,
                "design.suspension.n_magnets",
            ),
            (
                RailClass::Door,
                |p| p.design.suspension.stations[1].normal_sign = 0.5,
                "design.suspension.stations.1.normal_sign",
            ),
            (
                RailClass::Door,
                |p| p.design.suspension.zp_band_m[0] = 5.1e-3,
                "design.suspension.zp_band_m.0",
            ),
            (
                RailClass::Drawer,
                |p| p.design.suspension.g_seat_m = 0.0,
                "design.suspension.g_seat_m",
            ),
            (
                RailClass::Door,
                |p| p.design.rail.flange_t_m = 5.0e-3,
                "design.rail.flange_t_m",
            ),
            (
                RailClass::Door,
                |p| p.design.suspension.pole_l_m *= 1.2,
                "design.suspension.pole_l_m",
            ),
            (
                RailClass::Drawer,
                |p| {
                    p.design.suspension.i_pulse_a = 10.0;
                    p.safety.i_coil_pulse_a = 10.0;
                },
                "design.suspension.i_pulse_a",
            ),
            (
                RailClass::Wall,
                |p| p.design.power.v_bus_v = 72.0,
                "design.power.v_bus_v",
            ),
            (
                RailClass::Heavy,
                |p| p.design.power.supercap_c_f = 0.05,
                "design.power.supercap_c_f",
            ),
            (
                RailClass::Door,
                |p| p.control.rates.base_hz = 40_000,
                "control.rates.base_hz",
            ),
            (
                RailClass::Door,
                |p| p.control.levitation.roll_active = true,
                "control.levitation.roll_active",
            ),
            (
                RailClass::Drawer,
                |p| p.control.levitation.roll_active = false,
                "control.levitation.roll_active",
            ),
            (
                RailClass::Door,
                |p| p.design.lsm.k_e_v_s_per_m *= 1.1,
                "design.lsm.k_e_v_s_per_m",
            ),
            (
                RailClass::Wall,
                |p| p.control.motion.a_max_mps2 = 1.0,
                "control.motion.a_max_mps2",
            ),
            (
                RailClass::Drawer,
                |p| p.control.haptic.mass_ratio = 3.0,
                "control.haptic.mass_ratio",
            ),
            (
                RailClass::Drawer,
                |p| p.control.haptic.k_virtual_frac = 0.5,
                "control.haptic.k_virtual_frac",
            ),
            (
                RailClass::Door,
                |p| p.safety.v_open_max_mps = 0.20,
                "safety.v_open_max_mps",
            ),
            (
                RailClass::Heavy,
                |p| p.design.operating_mode = OperatingMode::Automatic,
                "design.operating_mode",
            ),
            (
                RailClass::Door,
                |p| p.safety.f_static_n = 100.0,
                "safety.f_static_n",
            ),
            (
                RailClass::Wall,
                |p| p.design.damping.bumper_e_rated_j = 400.0,
                "design.damping.bumper_e_rated_j",
            ),
            (
                RailClass::Door,
                |p| p.safety.i_coil_cont_a = 6.0,
                "safety.i_coil_cont_a",
            ),
            (
                RailClass::Door,
                |p| p.design.sensors.coil_a_per_code *= 2.0,
                "safety.adc_a_per_code_copy",
            ),
            (
                RailClass::Wall,
                |p| p.design.sensors.accel_mount = AccelMount::Carriage,
                "design.sensors.accel_mount",
            ),
            (
                RailClass::Door,
                |p| p.control.seismic.policy.lock_after_land = true,
                "control.seismic.policy.lock_after_land",
            ),
            (
                RailClass::Door,
                |p| p.control.motion.inertial.enable = true,
                "control.motion.inertial.enable",
            ),
        ];
        for (c, mutate, want) in cases {
            let mut p = presets::by_class(c);
            mutate(&mut p);
            let e = validate(&p).err();
            assert_eq!(
                e.map(|e| e.path),
                Some(want),
                "{c:?}: expected rejection at {want}, got {e:?} (SW-004)"
            );
            if let Some(e) = e {
                let shown = e.to_string();
                assert!(
                    shown.starts_with(want) && shown.contains(": "),
                    "\"path: reason\" format: {shown}"
                );
                assert!(
                    meta::find(e.path).is_some(),
                    "error path {} is a meta::FIELDS path",
                    e.path
                );
            }
        }
    }

    #[test]
    fn presence_sensing_gate() {
        // With PL d presence sensing the unsensed caps no longer bind (SAF-003).
        let mut p = presets::door();
        p.safety.v_open_max_mps = 0.20;
        p.safety.v_open_sensed_mps = 0.70;
        assert_eq!(
            path_of(&p),
            Some("safety.v_open_max_mps"),
            "unsensed: rejected"
        );
        p.design.presence = PresenceSensing::PlD;
        // The monitor's diverse copy must follow (it gates the sensed caps).
        assert_eq!(
            path_of(&p),
            Some("safety.presence_copy"),
            "unsealed: rejected"
        );
        crate::crc::seal(&mut p);
        assert_eq!(validate(&p), Ok(()), "PL d: accepted");
        // HEAVY automatic with PL d is allowed.
        let mut h = presets::heavy();
        h.design.operating_mode = OperatingMode::Automatic;
        h.design.presence = PresenceSensing::PlD;
        h.safety.p_max_kgmps = presets::PEDESTRIAN_P_MAX_KGMPS;
        crate::crc::seal(&mut h);
        assert_eq!(validate(&h), Ok(()), "HEAVY automatic with PL d (SAF-003)");
    }

    #[test]
    fn assist_ratio_bounds() {
        let mut p = presets::door();
        p.control.haptic.mass_ratio = 5.0;
        assert_eq!(validate(&p), Ok(()), "≤ 5 sensorless (HAP-002)");
        p.control.haptic.mass_ratio = 6.0;
        assert_eq!(
            path_of(&p),
            Some("control.haptic.mass_ratio"),
            "> 5 sensorless rejected (HAP-002)"
        );
        p.design.sensors.handle_n_per_code = 0.01;
        assert_eq!(validate(&p), Ok(()), "≤ 20 with a load cell (HAP-002)");
        let mut d = presets::drawer();
        d.control.haptic.mass_ratio = 2.0;
        assert_eq!(validate(&d), Ok(()), "DRAWER ≤ 2 (HAP-002)");
    }

    #[test]
    fn egress_and_isolation_gates() {
        let mut p = presets::door();
        p.design.egress = EgressRole::FireRated;
        p.control.seismic.policy.action = SeismicAction::OpenAndStop;
        assert_eq!(
            path_of(&p),
            Some("control.seismic.policy.action"),
            "fire-rated never opens (SEI-011)"
        );
        let mut w = presets::wall();
        w.control.seismic.isolation.enable = true;
        assert_eq!(
            validate(&w),
            Ok(()),
            "WALL isolation may be enabled (research tier)"
        );
        let mut d = presets::door();
        d.control.seismic.isolation.enable = true;
        assert_eq!(path_of(&d), Some("control.seismic.isolation.enable"));
        let mut dr = presets::drawer();
        dr.control.motion.inertial.enable = true;
        assert_eq!(
            validate(&dr),
            Ok(()),
            "DRAWER inertial smoothing may be enabled (research tier)"
        );
    }
}
