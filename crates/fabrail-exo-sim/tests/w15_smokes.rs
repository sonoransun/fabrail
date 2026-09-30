//! Wave-1.5 closed-loop smokes of the sovereign tier (simulation only, research tier; no
//! hardware or certification claim): the X2 controllers against the X4 truth models.
//!
//! * SENTINEL 2-DOF: a 1,000 t hybrid mass damper (μ = 1 %, Warburton base tuning,
//!   ±1.5 m, 121 kN / 163 kW, exotic design §3.1) on a T = 7 s primary with ζ_s = 1 %,
//!   under resonant base acceleration at the design basis (the passive stage peaks at
//!   0.85·s_max; X2's FAB-EXO-604 level), with the `fabrail_exo::hmd::Hmd` saturation stack
//!   in SEISMIC_PASSIVE (passive eddy damping, stroke governor armed). XS01 criterion:
//!   host peak ≤ 1.02× the uncontrolled (passive, LSM off) peak; the stroke stays within
//!   s_max plus the buffer compression, and every state stays finite.
//! * HAVEN-R 3-DOF: the 50 t T = 6 s platform (`PlatformCfg::haven_r`) with the
//!   `fabrail_exo::isolation::Isolator` skyhook under a 0.3 g, 1.5 Hz ground burst:
//!   the stroke stays inside the moat without buffer contact (XS01) and the absolute
//!   acceleration is below the ground's.
//!
//! The SPEC tokens for XS01 are X6's scenario tests (Wave 2a), so no `// req:` marker.

use fabrail_exo::hmd::{Hmd, HmdGains, HmdInputs, HmdLimits};
use fabrail_exo::isolation::{IsolationCfg, IsolationInputs, Isolator};
use fabrail_exo::tuning::{TuningForm, damper_design_for};
use fabrail_exo_sim::host::{Damper, Primary, TwoDof};
use fabrail_exo_sim::platform::{HavenR3Dof, PlatformCfg};
use fabrail_types::SeismicState;

const TAU: f64 = core::f64::consts::TAU;

/// Peak |x_s| and |stroke| over the last half of a resonant base-acceleration run.
fn sentinel_run(active: bool, a0_g: f64) -> (f64, f64, bool) {
    let mu = 0.01;
    let m_d = 1.0e6;
    let m_s = m_d / mu;
    let w_s = TAU / 7.0;
    let s_max = 1.5;
    let design = damper_design_for(m_s, w_s, mu, TuningForm::WarburtonBase);
    let mut sys = TwoDof::new(
        Primary {
            m_kg: m_s,
            omega_rad_s: w_s,
            zeta: 0.01,
        },
        Damper::from_design(&design, s_max),
    );
    let mut hmd = Hmd::new(
        HmdGains::default(),
        HmdLimits::from_actuator(s_max as f32, 163.0e3, 121.0e3),
    );
    let a0 = a0_g * 9.81;
    let dt = 0.01;
    let n = (400.0 / dt) as usize;
    let (mut xs, mut sr, mut finite) = (0.0f64, 0.0f64, true);
    for k in 0..n {
        let t = k as f64 * dt;
        let a_g = a0 * (w_s * t).sin();
        let u = if active {
            let inp = HmdInputs {
                x_s_m: sys.state[0] as f32,
                v_s_mps: sys.state[1] as f32,
                x_r_m: sys.stroke_m() as f32,
                v_r_mps: (sys.state[3] - sys.state[1]) as f32,
                a_floor_mps2: a_g as f32,
                sensors_ok: true,
                power_ok: true,
                levitation_ok: true,
                seismic: true,
                maintenance: false,
            };
            f64::from(hmd.step(&inp, dt as f32).force_n)
        } else {
            0.0
        };
        sys.step(a_g, u, 0.0, dt);
        finite &= sys.state.iter().all(|v| v.is_finite());
        if k > n / 2 {
            xs = xs.max(sys.state[0].abs());
            sr = sr.max(sys.stroke_m().abs());
        }
    }
    (xs, sr, finite)
}

#[test]
fn sentinel_two_dof_smoke() {
    // Design basis (X2, FAB-EXO-604): the resonant drive at which the passive stage
    // peaks at 0.85·s_max (the 2-DOF is linear inside the stroke, so scale a probe run).
    let (_, s_probe, _) = sentinel_run(false, 0.001);
    let a0_g = 0.001 * 0.85 * 1.5 / s_probe;
    let (x_passive, s_passive, f0) = sentinel_run(false, a0_g);
    let (x_active, s_active, f1) = sentinel_run(true, a0_g);
    assert!(f0 && f1, "every state finite");
    assert!(
        x_passive > 0.0 && s_passive > 0.0,
        "the drive excites the system"
    );
    assert!(
        x_active <= 1.02 * x_passive,
        "SENTINEL host peak {x_active} m ≤ 1.02× uncontrolled {x_passive} m (XS01)"
    );
    // End stops are elastic buffers beyond s_max (X4): allow their compression.
    assert!(
        s_active <= 1.5 * 1.05,
        "stroke {s_active} m within s_max 1.5 m plus buffer compression"
    );
}

#[test]
fn haven_r_three_dof_smoke() {
    let cfg = PlatformCfg::haven_r();
    let mut plat = HavenR3Dof::new(&cfg).expect("HAVEN-R reference config validates");
    let icfg = IsolationCfg {
        m_kg: cfg.m_kg as f32,
        f_iso_hz: (1.0 / 6.0) as f32,
        zeta_sky: 0.3,
        f_rc_hz: 0.01,
        hp_hz: 0.1,
        f_max_n: 60.0e3,
        stroke_m: cfg.stroke_m as f32,
    };
    let mut iso = Isolator::new(&icfg).expect("isolation config");
    let dt = 0.005;
    let n = (40.0 / dt) as usize;
    let (mut a_plat, mut a_ground) = (0.0f64, 0.0f64);
    let mut v_ground = 0.0f64;
    let mut x_ground = 0.0f64;
    for k in 0..n {
        let t = k as f64 * dt;
        // 0.3 g at 1.5 Hz for 10 s with a smooth envelope (a building-period burst).
        let env = if t < 10.0 {
            (core::f64::consts::PI * t / 10.0).sin().powi(2)
        } else {
            0.0
        };
        let a_g = 0.3 * 9.81 * env * (TAU * 1.5 * t).sin();
        v_ground += a_g * dt;
        x_ground += v_ground * dt;
        let q = plat.state.q;
        let a_abs = plat.abs_accel_mps2();
        let f = iso.step(
            &IsolationInputs {
                a_abs_mps2: a_abs[0] as f32,
                x_rel_m: q[0] as f32,
                v_rel_mps: q[3] as f32,
                seis: SeismicState::Shaking,
                ..IsolationInputs::default()
            },
            dt as f32,
        );
        plat.step([a_g, 0.0], [f64::from(f), 0.0, 0.0], dt);
        assert!(
            plat.state.q.iter().all(|v| v.is_finite()),
            "finite state at {t} s"
        );
        a_plat = a_plat.max(plat.abs_accel_mps2()[0].abs());
        a_ground = a_ground.max(a_g.abs());
    }
    let _ = x_ground;
    assert!(
        plat.max_radial_m() < cfg.moat_m,
        "stroke {} m inside the {} m moat (XS01)",
        plat.max_radial_m(),
        cfg.moat_m
    );
    assert!(!plat.buffer_contact(), "no buffer contact (XS01)");
    assert!(
        a_plat < a_ground,
        "isolated peak {a_plat} m/s² below the ground's {a_ground} m/s²"
    );
}
