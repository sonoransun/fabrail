//! Closed-loop log-damped termini on Carriage6 (W1.5; plan §4b, PHYSICS §16.1): the
//! real executive against the real plant in voltage coil mode, quiet floor
//! (|a_floor,x| = 0 < 0.01 g), for the DRAWER (close into the captive seat, open from the
//! seat) and the DOOR (close and open). The criteria are those of FAB-HAP-013…016,
//! measured by `metrics` on plant truth:
//!
//! * semilog fit of the distance-to-go over [t_c + t_b, t_ε]: R² ≥ 0.999 (HAP-013), with
//!   the fitted slope within 5 % of −1/τ;
//! * x − x_term never crosses zero by more than 0.02 mm (HAP-016);
//! * arrival speed on `Truth.v_mps` ≤ 1 mm/s (HAP-016);
//! * arrival within ±2 % of the reference's arrival at the same 20 µm tolerance
//!   (HAP-016; the closed form t_b + τ·ln(d_b/d_ε) + T_q is the reference reaching the
//!   target exactly, see `metrics::TERMINUS_ARRIVE_TOL_M`).
//!
//! The SPEC tokens for these requirements are the Python closed-loop tests
//! (`py:haptics::terminus_semilog_fit`, `terminus_arrival`, `terminus_hand_replan`), so
//! these Rust tests carry no `// req:` marker; they are the W1.5 closed-loop evidence the
//! Python suite builds on.

mod common;

use fabrail_params::{SystemParams, presets};
use fabrail_sim::{ChannelData, RunData, run};
use fabrail_types::MotionMode;

fn kpi(d: &RunData, k: &str) -> f64 {
    *d.kpis
        .get(k)
        .unwrap_or_else(|| panic!("KPI {k} missing: {:?}", d.kpis.keys()))
}

fn u8_channel<'a>(d: &'a RunData, name: &str) -> &'a [u8] {
    match &d.channel(name).expect("channel").data {
        ChannelData::U8(v) => v,
        ChannelData::F64(_) => panic!("{name} is not a u8 channel"),
    }
}

fn f64_channel<'a>(d: &'a RunData, name: &str) -> &'a [f64] {
    match &d.channel(name).expect("channel").data {
        ChannelData::F64(v) => v,
        ChannelData::U8(_) => panic!("{name} is not an f64 channel"),
    }
}

/// A Carriage6 voltage-mode run starting levitated (or seated) at `x_m` with one
/// command at 0.1 s and optional extra events.
fn run_move(p: &SystemParams, init: &str, x_m: f64, cmd: &str, dur_s: f64, extra: &str) -> RunData {
    let json = format!(
        r#"{{"schema": "fabrail.scenario/1", "name": "terminus_cl", "duration_s": {dur_s},
            "noise_seed": 42, "fidelity": {{"model": "carriage6", "coil": "voltage"}},
            "initial": {{"lev": "{init}", "x_m": {x_m}}},
            "record": {{"rate_hz": 200, "channels": ["x_m", "motion_mode", "demand",
                "contact_state"]}},
            "events": [{{"kind": "command", "t_s": 0.1, "cmd": "{cmd}"}}{extra}]}}"#
    );
    run(p, &common::scenario(&json, &[])).expect("run")
}

/// The FAB-HAP-013…016 criteria on one run with a single terminus.
fn assert_terminus(d: &RunData, tau_s: f64, what: &str) {
    assert_eq!(kpi(d, "terminus_count"), 1.0, "{what}: one terminus");
    let r2 = kpi(d, "terminus_r2_min");
    assert!(
        r2 >= 0.999,
        "{what}: semilog R² {r2} ≥ 0.999 (FAB-HAP-013, plan §4b)"
    );
    let tau_fit = kpi(d, "terminus_tau_fit_s");
    assert!(
        (tau_fit / tau_s - 1.0).abs() <= 0.05,
        "{what}: fitted τ {tau_fit} s within 5 % of τ = {tau_s} s (slope −1/τ, PHYSICS §16.1)"
    );
    let over = kpi(d, "terminus_overshoot_max_m");
    assert!(
        over <= 0.02e-3,
        "{what}: overshoot {over} m ≤ 0.02 mm (FAB-HAP-016)"
    );
    let v = kpi(d, "terminus_v_arrive_max_mps");
    assert!(
        v <= 1.0e-3,
        "{what}: arrival speed {v} m/s ≤ 1 mm/s on Truth.v (FAB-HAP-016)"
    );
    let err = kpi(d, "terminus_t_arrive_err_max");
    assert!(
        err <= 0.02,
        "{what}: t_arrive within ±2 % of the reference ({:.4} %; FAB-HAP-016)",
        100.0 * err
    );
    assert!(
        u8_channel(d, "demand").iter().all(|&x| x == 0),
        "{what}: no safety demand"
    );
}

#[test]
fn drawer_close_terminus_seats() {
    let p = presets::drawer();
    let tau = p.control.motion.terminus.tau_s;
    let d = run_move(&p, "levitated", 0.25, "close", 2.5, "");
    assert_terminus(&d, tau, "DRAWER close");
    // Seat approach (descent to the seat clearance before the slot), log-damped close,
    // then the z-seat step: seated at the closed stop.
    let last = *u8_channel(&d, "contact_state").last().expect("samples");
    assert_eq!(
        last & 0b11,
        0b10,
        "DRAWER seated at the end (contact_state)"
    );
    let x_end = *f64_channel(&d, "x_m").last().expect("samples");
    assert!(
        x_end.abs() <= p.control.motion.pos_tol_closed_m,
        "DRAWER at the closed stop: {x_end} m"
    );
}

#[test]
fn drawer_open_terminus_from_seat() {
    let p = presets::drawer();
    let tau = p.control.motion.terminus.tau_s;
    let d = run_move(&p, "seated", 0.0, "open", 3.5, "");
    assert_terminus(&d, tau, "DRAWER soft-open from the seat");
    let x_end = *f64_channel(&d, "x_m").last().expect("samples");
    assert!(
        (x_end - p.design.rail.travel_m).abs() <= p.control.motion.pos_tol_travel_m,
        "DRAWER at the open end: {x_end} m"
    );
}

#[test]
fn door_close_and_open_termini() {
    let p = presets::door();
    let tau = p.control.motion.terminus.tau_s;
    let d = run_move(&p, "levitated", 0.8, "close", 8.0, "");
    assert_terminus(&d, tau, "DOOR close");
    let d = run_move(&p, "levitated", 0.2, "open", 8.0, "");
    assert_terminus(&d, tau, "DOOR open");
}

/// A hand during the DOOR close capture switches to plain class admittance; on release
/// the fast group re-plans from the measured state and the door still arrives closed
/// (plan §4b hand interaction; FAB-HAP-016 hand clause, closed-loop evidence).
#[test]
fn door_hand_during_capture_replans() {
    let p = presets::door();
    let hand = r#", {"kind": "hand", "t_s": 5.45, "t_pts_s": [0, 0.03, 0.3, 0.33],
        "f_pts_n": [0, 20, 20, 0], "impedance": {"m_kg": 0.5, "b_n_s_per_m": 0, "k_n_per_m": 0}}"#;
    let d = run_move(&p, "levitated", 0.8, "close", 9.0, hand);
    // The supervisor keeps publishing the terminus; the yield is visible in the motion:
    // under plain admittance (K_v = 0, M_v = m/3) the 20 N push for 0.3 s drives the
    // door back open by centimetres, which the stiff capture would not allow (≈ 20 N /
    // (m·ω_v²/4) ≈ 0.6 mm).
    let x = f64_channel(&d, "x_m");
    let rate = d.rate_hz;
    let k_on = (5.45 * rate) as usize;
    let win = (0.5 * rate) as usize;
    let (k_min, x_min) =
        x[k_on..k_on + win]
            .iter()
            .enumerate()
            .fold(
                (0, f64::MAX),
                |(ki, xm), (k, &v)| if v < xm { (k, v) } else { (ki, xm) },
            );
    let x_back = x[k_on + k_min..k_on + 2 * win]
        .iter()
        .cloned()
        .fold(f64::MIN, f64::max);
    assert!(
        x_back - x_min > 5.0e-3,
        "the hand switched the capture to admittance: pushed back {} mm",
        1e3 * (x_back - x_min)
    );
    let modes = u8_channel(&d, "motion_mode");
    let term = MotionMode::Terminus.as_u8();
    assert!(
        modes[k_on..].contains(&term),
        "the supervisor's move stayed a terminus (the re-plan is local to the fast group)"
    );
    // Arrived closed after the re-plan (the hover door then idles in admittance).
    let x_best = x[k_on + 2 * win..]
        .iter()
        .fold(f64::MAX, |m, v| m.min(v.abs()));
    assert!(
        x_best <= p.control.motion.pos_tol_closed_m,
        "DOOR closed after the re-plan: {x_best} m"
    );
    assert!(
        u8_channel(&d, "demand").iter().all(|&x| x == 0),
        "no safety demand"
    );
}
