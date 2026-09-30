//! HAVEN isolation: skyhook, semi-active low-SOC mode, slow recentring (off in SHAKING), confirmed pre-positioning (FAB-EXO-701..707). Owner: X2 (API frozen by W0a).
//!
//! EXOTIC §X9 (research tier); exotic design §3.2. f32 controller:
//!
//! F_lsm = −c_sky·v̂_abs − w_rc·k_rc·x_rel + F_pre, then |F| ≤ F_max, then (low SOC)
//! the semi-active clip.
//!
//! * v̂_abs: leaky integral of the platform absolute acceleration,
//!   v̂' = a_abs − ω_hp·v̂ (a 0.1 Hz high-pass on the velocity; ZOH-exact update), so
//!   accelerometer bias cannot integrate into a drift. c_sky = 2·ζ·m·ω_iso (Karnopp
//!   skyhook; PHYSICS §16.2, §21).
//! * Recentring: k_rc = m·(2π·f_rc)², i.e. its natural frequency is the recentring
//!   bandwidth f_rc ≤ 0.1·f_iso ([`RECENTRING_MAX_FRAC`], enforced by
//!   [`Isolator::new`]); the added stiffness is then ≤ 1 % of the isolation stiffness
//!   m·ω_iso², so the isolation band is untouched. w_rc = 0 in SHAKING (immediately) and
//!   ramps back to 1 over 1/f_rc afterwards (bumpless).
//! * Pre-positioning (centring before the S wave): a critically damped PD toward the
//!   target at [`PRE_POSITION_BW_HZ`] whose force is capped at m·0.02 g
//!   ([`PRE_POSITION_A_MAX_MPS2`]), only with independent confirmation, a
//!   `ClearVerified` moat and a state before SHAKING ([`pre_positioning_allowed`]);
//!   recentring is suspended while it runs. The target is clamped to ±stroke.
//! * Semi-active (low storage SOC): the command is clipped so that u·v_rel ≤ 0; the LSM
//!   then only brakes (regenerates) and draws no net energy.
//!
//! Vertical isolation is passive only (never maglev; FAB-EXO-705). Transmissibility
//! (FAB-EXO-702; T = 4 s, 1 Hz, ζ 0.2: passive 0.1251, skyhook 0.0663) delegates to the
//! single-source closed forms in `fabrail_params::derive`.

use fabrail_params::derive;
use fabrail_types::{SeismicState, fmath};

/// Recentring bandwidth limit as a fraction of the isolation frequency (FAB-EXO-707).
pub const RECENTRING_MAX_FRAC: f32 = 0.1;

/// Pre-positioning acceleration cap m/s², 0.02 g (exotic design §3.2: "centre with a
/// ≤ 0.02 g S-curve"; g = standard gravity, `derive::G0_MPS2`).
pub const PRE_POSITION_A_MAX_MPS2: f32 = (0.02 * derive::G0_MPS2) as f32;

/// Pre-positioning loop bandwidth, Hz: a critically damped PD at 0.2 Hz settles a
/// 0.3 m centring move in about 10 s, inside a typical S−P warning.
pub const PRE_POSITION_BW_HZ: f32 = 0.2;

/// Isolation configuration.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct IsolationCfg {
    /// Platform mass, kg.
    pub m_kg: f32,
    /// Isolation frequency f_iso = 1/T, Hz.
    pub f_iso_hz: f32,
    /// Skyhook damping ratio ζ (c_sky = 2ζ·m·ω_iso).
    pub zeta_sky: f32,
    /// Recentring bandwidth, Hz (≤ 0.1·f_iso).
    pub f_rc_hz: f32,
    /// High-pass corner of the absolute-velocity estimate, Hz (0.1 Hz).
    pub hp_hz: f32,
    /// Continuous force limit, N.
    pub f_max_n: f32,
    /// Stroke limit, m.
    pub stroke_m: f32,
}

impl IsolationCfg {
    /// Skyhook damping c_sky = 2·ζ·m·ω_iso, N·s/m.
    pub fn c_sky_n_s_per_m(&self) -> f32 {
        2.0 * self.zeta_sky * self.m_kg * 2.0 * core::f32::consts::PI * self.f_iso_hz
    }

    /// Recentring stiffness k_rc = m·(2π·f_rc)², N/m (natural frequency f_rc).
    pub fn k_rc_n_per_m(&self) -> f32 {
        let w = 2.0 * core::f32::consts::PI * self.f_rc_hz;
        self.m_kg * w * w
    }
}

/// Why a configuration was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum IsolationError {
    /// A non-positive or non-finite field.
    InvalidField = 0,
    /// Recentring faster than 0.1× the isolation frequency.
    RecentringTooFast = 1,
}

/// Inputs of one isolation step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct IsolationInputs {
    /// Platform absolute acceleration, m/s².
    pub a_abs_mps2: f32,
    /// Platform position relative to the ground, m.
    pub x_rel_m: f32,
    /// Platform velocity relative to the ground, m/s.
    pub v_rel_mps: f32,
    /// Seismic state (recentring is off in SHAKING).
    pub seis: SeismicState,
    /// Storage SOC is low: semi-active mode.
    pub low_soc: bool,
    /// Pre-positioning target, m (only with confirmation and a verified moat).
    pub pre_position_m: Option<f32>,
    /// Independent confirmation of the alert.
    pub confirmed: bool,
    /// Moat band `ClearVerified`.
    pub moat_clear_verified: bool,
}

/// Pre-positioning may run: independent confirmation, a `ClearVerified` moat and a
/// state before the shaking (NORMAL, TRIGGERED or PROTECT; never SHAKING, QUIET or
/// INSPECT).
pub const fn pre_positioning_allowed(
    seis: SeismicState,
    confirmed: bool,
    moat_clear_verified: bool,
) -> bool {
    confirmed
        && moat_clear_verified
        && matches!(
            seis,
            SeismicState::Normal | SeismicState::Triggered | SeismicState::Protect
        )
}

/// A measurement sanitised per step: non-finite becomes 0.
fn finite_or_zero(x: f32) -> f32 {
    if x.is_finite() { x } else { 0.0 }
}

/// The HAVEN isolation controller.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Isolator {
    cfg: IsolationCfg,
    v_abs_hat_mps: f32,
    /// Recentring weight w_rc ∈ [0, 1].
    w_rc: f32,
}

impl Isolator {
    /// Validates `cfg` (positive fields; recentring ≤ 0.1·f_iso).
    pub fn new(cfg: &IsolationCfg) -> Result<Isolator, IsolationError> {
        let pos = [cfg.m_kg, cfg.f_iso_hz, cfg.hp_hz, cfg.f_max_n, cfg.stroke_m];
        if pos.iter().any(|x| !(x.is_finite() && *x > 0.0))
            || !(cfg.zeta_sky.is_finite() && cfg.zeta_sky >= 0.0)
            || !(cfg.f_rc_hz.is_finite() && cfg.f_rc_hz >= 0.0)
        {
            return Err(IsolationError::InvalidField);
        }
        if cfg.f_rc_hz > RECENTRING_MAX_FRAC * cfg.f_iso_hz {
            return Err(IsolationError::RecentringTooFast);
        }
        Ok(Isolator {
            cfg: *cfg,
            v_abs_hat_mps: 0.0,
            w_rc: 1.0,
        })
    }

    /// Recentring is active outside SHAKING only.
    pub const fn recentring_enabled(seis: SeismicState) -> bool {
        !matches!(seis, SeismicState::Shaking)
    }

    /// Steps by `dt_s`; returns the LSM force command, N.
    pub fn step(&mut self, inp: &IsolationInputs, dt_s: f32) -> f32 {
        let c = self.cfg;
        let dt = if dt_s.is_finite() && dt_s > 0.0 {
            dt_s
        } else {
            0.0
        };
        let x_rel = finite_or_zero(inp.x_rel_m);
        let v_rel = finite_or_zero(inp.v_rel_mps);

        // Absolute velocity: leaky integral (ZOH-exact first-order update of
        // 1/(s + ω_hp)).
        let w_hp = 2.0 * core::f32::consts::PI * c.hp_hz;
        let alpha = fmath::expf(-w_hp * dt);
        self.v_abs_hat_mps =
            alpha * self.v_abs_hat_mps + (1.0 - alpha) / w_hp * finite_or_zero(inp.a_abs_mps2);

        // Pre-positioning (confirmed, moat ClearVerified, before the shaking).
        let target = match inp.pre_position_m {
            Some(t) if t.is_finite() => Some(fmath::clampf(t, -c.stroke_m, c.stroke_m)),
            _ => None,
        };
        let pre = if pre_positioning_allowed(inp.seis, inp.confirmed, inp.moat_clear_verified) {
            target
        } else {
            None
        };

        // Recentring weight: off in SHAKING and while pre-positioning, ramps back over
        // 1/f_rc otherwise.
        if !Self::recentring_enabled(inp.seis) || pre.is_some() {
            self.w_rc = 0.0;
        } else if c.f_rc_hz > 0.0 {
            self.w_rc = fmath::fminf(1.0, self.w_rc + c.f_rc_hz * dt);
        } else {
            self.w_rc = 1.0;
        }

        let mut f =
            -c.c_sky_n_s_per_m() * self.v_abs_hat_mps - self.w_rc * c.k_rc_n_per_m() * x_rel;
        if let Some(t) = pre {
            let w = 2.0 * core::f32::consts::PI * PRE_POSITION_BW_HZ;
            let a = -w * w * (x_rel - t) - 2.0 * w * v_rel;
            let a_cap = PRE_POSITION_A_MAX_MPS2;
            f += c.m_kg * fmath::clampf(a, -a_cap, a_cap);
        }
        f = fmath::clampf(f, -c.f_max_n, c.f_max_n);
        // Semi-active: brake only (u·v_rel ≤ 0).
        if inp.low_soc && f * v_rel > 0.0 {
            f = 0.0;
        }
        f
    }

    /// The configuration.
    pub const fn cfg(&self) -> IsolationCfg {
        self.cfg
    }

    /// Absolute-velocity estimate v̂_abs, m/s.
    pub const fn v_abs_hat_mps(&self) -> f32 {
        self.v_abs_hat_mps
    }

    /// Recentring weight w_rc ∈ [0, 1].
    pub const fn w_rc(&self) -> f32 {
        self.w_rc
    }
}

/// Passive transmissibility √((1 + (2ζr)²)/((1 − r²)² + (2ζr)²)) (PHYSICS §21).
pub fn transmissibility_passive(r: f64, zeta: f64) -> f64 {
    derive::transmissibility(r, zeta)
}

/// Skyhook transmissibility 1/√((1 − r²)² + (2ζr)²) (PHYSICS §21).
pub fn transmissibility_skyhook(r: f64, zeta: f64) -> f64 {
    derive::t_sky(r, zeta)
}

/// Skyhook transmissibility with the leaky-integrator velocity estimate of
/// [`Isolator`]: |k/(m·s² + c·s²/(s + ω_hp) + k)| at s = jω, with r = ω/ω_n and
/// h = ω_hp/ω_n (reduces to [`transmissibility_skyhook`] as h → 0; EXOTIC §X9).
pub fn transmissibility_skyhook_estimated(r: f64, zeta: f64, h: f64) -> f64 {
    // Normalised (m = k = ω_n = 1, c = 2ζ, s = jr): D = 1 − r² − 2ζr²/(h + jr)
    // = 1 − r² − 2ζr²·h/(h² + r²) + j·2ζr³/(h² + r²); |H| = 1/|D|.
    let den2 = h * h + r * r;
    let re = 1.0 - r * r - 2.0 * zeta * r * r * h / den2;
    let im = 2.0 * zeta * r * r * r / den2;
    1.0 / fmath::sqrt(re * re + im * im)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// HAVEN-R-like platform (EXOTIC §1: 50 t) on an isolation spring of period `t_iso`.
    fn cfg(t_iso: f32, f_rc_frac: f32) -> IsolationCfg {
        IsolationCfg {
            m_kg: 50.0e3,
            f_iso_hz: 1.0 / t_iso,
            zeta_sky: 0.2,
            f_rc_hz: f_rc_frac / t_iso,
            hp_hz: 0.1,
            f_max_n: 0.1 * 50.0e3 * 9.806_65,
            stroke_m: 0.9,
        }
    }

    /// Steady-state absolute-displacement amplitude ratio under harmonic ground motion
    /// x_g = X·sin(ωt), by a single-bin DFT over the last `n_meas` whole periods.
    /// `skyhook`: LSM skyhook in SHAKING (recentring off); otherwise a passive
    /// relative damper of ratio ζ (LSM off).
    fn sim_ratio(f_hz: f64, t_iso: f64, zeta: f64, skyhook: bool) -> f64 {
        let c = cfg(t_iso as f32, 0.1);
        let mut iso = Isolator::new(&c).unwrap_or_default();
        let m = c.m_kg as f64;
        let w_n = 2.0 * core::f64::consts::PI / t_iso;
        let k = m * w_n * w_n;
        let c_pas = 2.0 * zeta * m * w_n;
        let w = 2.0 * core::f64::consts::PI * f_hz;
        let xg_amp = 0.01;
        let dt = 1.0e-3;
        let (n_settle, n_meas) = (40.0 * f_hz.max(0.5), 20.0);
        let n_total = ((n_settle + n_meas) / f_hz / dt).round() as usize;
        let n_start = n_total - (n_meas / f_hz / dt).round() as usize;
        let (mut x, mut v, mut f_lsm) = (0.0f64, 0.0f64, 0.0f64);
        let (mut s_re, mut s_im) = (0.0, 0.0);
        let acc = |t: f64, x: f64, v: f64, f: f64| {
            let (xg, vg) = (xg_amp * (w * t).sin(), xg_amp * w * (w * t).cos());
            let damp = if skyhook { 0.0 } else { c_pas * (v - vg) };
            (-k * (x - xg) - damp + f) / m
        };
        for n in 0..n_total {
            let t = n as f64 * dt;
            if skyhook {
                let (xg, vg) = (xg_amp * (w * t).sin(), xg_amp * w * (w * t).cos());
                let inp = IsolationInputs {
                    a_abs_mps2: acc(t, x, v, f_lsm) as f32,
                    x_rel_m: (x - xg) as f32,
                    v_rel_mps: (v - vg) as f32,
                    seis: SeismicState::Shaking,
                    ..Default::default()
                };
                f_lsm = iso.step(&inp, dt as f32) as f64;
            }
            // RK4 on (x, v) with the force held.
            let k1 = (v, acc(t, x, v, f_lsm));
            let k2 = (
                v + 0.5 * dt * k1.1,
                acc(
                    t + 0.5 * dt,
                    x + 0.5 * dt * k1.0,
                    v + 0.5 * dt * k1.1,
                    f_lsm,
                ),
            );
            let k3 = (
                v + 0.5 * dt * k2.1,
                acc(
                    t + 0.5 * dt,
                    x + 0.5 * dt * k2.0,
                    v + 0.5 * dt * k2.1,
                    f_lsm,
                ),
            );
            let k4 = (
                v + dt * k3.1,
                acc(t + dt, x + dt * k3.0, v + dt * k3.1, f_lsm),
            );
            x += dt / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0);
            v += dt / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1);
            if n >= n_start {
                let tt = (n + 1) as f64 * dt;
                s_re += x * (w * tt).cos();
                s_im += x * (w * tt).sin();
            }
        }
        let n_m = (n_total - n_start) as f64;
        2.0 * (s_re * s_re + s_im * s_im).sqrt() / n_m / xg_amp
    }

    // req: FAB-EXO-702
    #[test]
    fn transmissibility_anchors() {
        // Closed forms at T = 4 s, ζ = 0.2 (EXOTIC §2, 4 decimals; ±2 %).
        let f_n = 0.25;
        for (f_hz, pas, sky) in [
            (0.5, 0.4125, 0.3221),
            (1.0, 0.1251, 0.0663),
            (2.0, 0.0531, 0.0159),
        ] {
            let r = f_hz / f_n;
            let (tp, ts) = (
                transmissibility_passive(r, 0.2),
                transmissibility_skyhook(r, 0.2),
            );
            assert!(
                (tp - pas).abs() < 5e-5,
                "passive T({f_hz} Hz) {tp} vs {pas} (EXOTIC §2)"
            );
            assert!(
                (ts - sky).abs() < 5e-5,
                "skyhook T({f_hz} Hz) {ts} vs {sky} (EXOTIC §2)"
            );
            assert!(
                (transmissibility_skyhook_estimated(r, 0.2, 0.0) - ts).abs() < 1e-12,
                "estimator form reduces to ideal skyhook as h → 0"
            );
        }
        // Simulated sweep vs analytic (EXOTIC §2 isolation row, ±2 %): the passive
        // harness, then the Isolator's skyhook with its 0.1 Hz leaky estimate.
        for f_hz in [0.5, 1.0, 2.0] {
            let r = f_hz / f_n;
            let sim = sim_ratio(f_hz, 4.0, 0.2, false);
            let want = transmissibility_passive(r, 0.2);
            assert!(
                ((sim - want) / want).abs() < 0.02,
                "passive sim {sim} vs {want} at {f_hz} Hz (±2 %)"
            );
        }
        for f_hz in [1.0, 2.0] {
            let sim = sim_ratio(f_hz, 4.0, 0.2, true);
            let want = transmissibility_skyhook(f_hz / f_n, 0.2);
            assert!(
                ((sim - want) / want).abs() < 0.02,
                "skyhook sim {sim} vs {want} at {f_hz} Hz (±2 %, FAB-EXO-702)"
            );
        }
        // At 0.5 Hz the 0.1 Hz high-pass costs 4.4 % against ideal skyhook; the
        // simulation matches the estimator-inclusive closed form (±2 %).
        let (r, h) = (2.0, 0.1 / f_n);
        let sim = sim_ratio(0.5, 4.0, 0.2, true);
        let want = transmissibility_skyhook_estimated(r, 0.2, h);
        assert!(
            ((sim - want) / want).abs() < 0.02,
            "0.5 Hz: sim {sim} vs {want}"
        );
        let dev = want / transmissibility_skyhook(r, 0.2) - 1.0;
        assert!(
            (dev + 0.0436).abs() < 0.002,
            "estimator deviation {dev} (−4.4 %)"
        );
        // HAVEN-R at T = 6 s isolates better than passive above √2·f_n.
        for f_hz in [0.5, 1.0, 2.0] {
            let r = f_hz * 6.0;
            assert!(transmissibility_skyhook(r, 0.2) < transmissibility_passive(r, 0.2));
        }
    }

    // req: FAB-EXO-707
    #[test]
    fn recentring_band_and_shaking_off() {
        // 1. Bandwidth rule: ≤ 0.1·f_iso accepted, above refused; bad fields refused.
        let ok = cfg(6.0, 0.1);
        assert!(Isolator::new(&ok).is_ok());
        let fast = IsolationCfg {
            f_rc_hz: ok.f_rc_hz * 1.001,
            ..ok
        };
        assert_eq!(Isolator::new(&fast), Err(IsolationError::RecentringTooFast));
        for bad in [
            IsolationCfg { m_kg: 0.0, ..ok },
            IsolationCfg {
                hp_hz: f32::NAN,
                ..ok
            },
            IsolationCfg {
                f_rc_hz: -0.001,
                ..ok
            },
            IsolationCfg {
                zeta_sky: -0.1,
                ..ok
            },
        ] {
            assert_eq!(Isolator::new(&bad), Err(IsolationError::InvalidField));
        }
        // k_rc has natural frequency f_rc, so it adds ≤ (f_rc/f_iso)² = 1 % of the
        // isolation stiffness; above resonance T_sky ∝ ω_n², so T at 1 Hz moves by
        // about 1 % (1.03 % at T 6 s), inside the ±2 % isolation tolerance.
        let w_rc = (ok.k_rc_n_per_m() / ok.m_kg).sqrt() / (2.0 * core::f32::consts::PI);
        assert!((w_rc - ok.f_rc_hz).abs() < 1e-6 * ok.f_rc_hz.max(1.0));
        let k_iso = ok.m_kg * (2.0 * core::f32::consts::PI * ok.f_iso_hz).powi(2);
        assert!(ok.k_rc_n_per_m() <= 0.0100001 * k_iso);
        let r: f64 = 6.0;
        let t0 = transmissibility_skyhook(r, 0.2);
        let t1 = transmissibility_skyhook(r / 1.01f64.sqrt(), 0.2 / 1.01f64.sqrt());
        assert!(
            ((t1 - t0) / t0).abs() < 0.0105,
            "recentring stiffening {t0} → {t1}"
        );

        // 2. Off in SHAKING, immediately; bumpless ramp back over 1/f_rc afterwards.
        let quiet = |seis| IsolationInputs {
            x_rel_m: 0.3,
            seis,
            ..Default::default()
        };
        let mut iso = Isolator::new(&ok).unwrap_or_default();
        let k_rc = ok.k_rc_n_per_m();
        let f = iso.step(&quiet(SeismicState::Normal), 0.01);
        assert!(
            (f + k_rc * 0.3).abs() < 1e-3 * k_rc,
            "NORMAL recentres: {f}"
        );
        assert!(!Isolator::recentring_enabled(SeismicState::Shaking));
        for &seis in SeismicState::ALL {
            assert_eq!(
                Isolator::recentring_enabled(seis),
                seis != SeismicState::Shaking
            );
        }
        let f = iso.step(&quiet(SeismicState::Shaking), 0.01);
        assert_eq!(
            (f, iso.w_rc()),
            (0.0, 0.0),
            "SHAKING: recentring off at once"
        );
        // In SHAKING the command is the pure skyhook for any state.
        let c_sky = ok.c_sky_n_s_per_m();
        for (a, x, v) in [(0.5, 0.4, -0.2), (-2.0, -0.8, 0.6), (0.0, 0.9, 0.0)] {
            let f = iso.step(
                &IsolationInputs {
                    a_abs_mps2: a,
                    x_rel_m: x,
                    v_rel_mps: v,
                    seis: SeismicState::Shaking,
                    ..Default::default()
                },
                0.01,
            );
            assert!(
                (f + c_sky * iso.v_abs_hat_mps()).abs() < 1e-3,
                "pure skyhook"
            );
        }
        // QUIET: the weight ramps back over 1/f_rc = 60 s (never a step).
        let mut iso = Isolator::new(&ok).unwrap_or_default();
        iso.step(&quiet(SeismicState::Shaking), 0.01);
        let mut last = 0.0f32;
        for _ in 0..5_000 {
            iso.step(&quiet(SeismicState::Quiet), 0.01);
            assert!(iso.w_rc() >= last && iso.w_rc() - last <= ok.f_rc_hz * 0.01 + 1e-6);
            last = iso.w_rc();
        }
        assert!(
            (last - 50.0 / 60.0).abs() < 0.01,
            "w_rc after 50 s: {last} (5/6)"
        );

        // 3. Pre-positioning only with confirmation, a ClearVerified moat and before the
        //    shaking (exhaustive), capped at m·0.02 g.
        let a_cap = PRE_POSITION_A_MAX_MPS2;
        assert!((a_cap - 0.196_133).abs() < 1e-5);
        for &seis in SeismicState::ALL {
            for confirmed in [false, true] {
                for moat in [false, true] {
                    let base = IsolationInputs {
                        x_rel_m: 0.4,
                        v_rel_mps: 0.05,
                        seis,
                        confirmed,
                        moat_clear_verified: moat,
                        ..Default::default()
                    };
                    let with = IsolationInputs {
                        pre_position_m: Some(0.0),
                        ..base
                    };
                    let f0 = Isolator::new(&ok).unwrap_or_default().step(&base, 0.01);
                    let f1 = Isolator::new(&ok).unwrap_or_default().step(&with, 0.01);
                    let allowed = confirmed
                        && moat
                        && matches!(
                            seis,
                            SeismicState::Normal | SeismicState::Triggered | SeismicState::Protect
                        );
                    assert_eq!(pre_positioning_allowed(seis, confirmed, moat), allowed);
                    if allowed {
                        // a_abs = 0 so v̂ = 0, and recentring is suspended: the command
                        // is F_pre alone, toward the target and capped at m·0.02 g
                        // (the PD asks for −0.76 m/s²). Without the target the fresh
                        // isolator recentres (w_rc = 1).
                        assert!((f0 + k_rc * 0.4).abs() < 1e-3 * k_rc, "{f0}");
                        assert!((f1 + ok.m_kg * a_cap).abs() < 1e-3 * ok.m_kg, "{f1}");
                    } else {
                        assert_eq!(f1, f0, "no pre-positioning: {seis:?} {confirmed} {moat}");
                    }
                }
            }
        }
        // Closed loop on a flat 1-D stage (worst case for drift): centring 0.3 m settles
        // within 20 s and the LSM acceleration never exceeds 0.02 g + skyhook.
        let mut iso = Isolator::new(&ok).unwrap_or_default();
        let (mut x, mut v, mut f) = (0.3f64, 0.0f64, 0.0f64);
        let m = ok.m_kg as f64;
        for _ in 0..20_000 {
            let inp = IsolationInputs {
                a_abs_mps2: (f / m) as f32,
                x_rel_m: x as f32,
                v_rel_mps: v as f32,
                seis: SeismicState::Protect,
                pre_position_m: Some(0.0),
                confirmed: true,
                moat_clear_verified: true,
                ..Default::default()
            };
            f = iso.step(&inp, 1.0e-3) as f64;
            v += f / m * 1.0e-3;
            x += v * 1.0e-3;
            // The centring share (total minus skyhook; recentring is suspended) stays
            // within 0.02 g.
            let f_pre = f + (c_sky * iso.v_abs_hat_mps()) as f64;
            assert!(
                f_pre.abs() <= m * a_cap as f64 * 1.0001,
                "centring accel {}",
                f_pre / m
            );
        }
        assert!(x.abs() < 0.01 && v.abs() < 0.01, "centred: x {x} v {v}");

        // 4. Semi-active (low SOC): the LSM only brakes, u·v_rel ≤ 0, for any state.
        let mut iso = Isolator::new(&ok).unwrap_or_default();
        for i in 0..2_000 {
            let s = i as f32;
            let inp = IsolationInputs {
                a_abs_mps2: 3.0 * (0.37 * s).sin(),
                x_rel_m: 0.8 * (0.11 * s).cos(),
                v_rel_mps: 0.7 * (0.23 * s).sin(),
                seis: if i % 3 == 0 {
                    SeismicState::Shaking
                } else {
                    SeismicState::Quiet
                },
                low_soc: true,
                pre_position_m: if i % 5 == 0 { Some(0.2) } else { None },
                confirmed: true,
                moat_clear_verified: true,
            };
            let f = iso.step(&inp, 0.01);
            assert!(
                f * inp.v_rel_mps <= 0.0,
                "semi-active drew power: {f} at {inp:?}"
            );
            assert!(f.abs() <= ok.f_max_n);
        }
    }
}
