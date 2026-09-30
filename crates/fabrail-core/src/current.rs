//! CurrentLoops: 20 kHz coil current PI (Kp = L*wc, Ki = R*wc, back-calculation anti-windup, motional-EMF feed-forward, I2t clamp) (PHYSICS §4). Owner: A1.
//!
//! Frozen API (W0, core design §B.4). Runs in the base task in voltage coil mode (the
//! only mode in which SIL ≡ firmware is claimed, plan §2b item 16); in lag mode the base
//! task passes the current references through and SimBoard applies the lag.
//!
//! Design (plan §2b item 2; PHYSICS §4): ω_c = 2π·`levitation.current_bw_hz` (1.0 kHz →
//! τ_cl = 0.159 ms), K_p = L̂·ω_c, K_i = R̂·ω_c with L̂ = N²·μ₀A_p/(2g* + h′) + L_σ
//! (`derive::coil_inductance`) and R̂ = `r_coil_ohm`. The PI zero cancels the coil pole
//! R/L, so the continuous loop is ω_c/s (first order at ω_c); with the ZOH and the
//! one-sample duty delay (PWM duty computed at tick k applies at k+1, core design §A.5)
//! the discrete small-signal step settles with a few % overshoot (LEV-008:
//! rise ≤ 0.3 ms, overshoot ≤ 10 %; at 1.5 kHz the delay gave ≈ 22 %, which is why the
//! review moved ω_c to 1 kHz).
//!
//! - Back-calculation anti-windup: the integrator tracks the bus-limited output with gain
//!   K_aw = ω_c (tracking time T_t = 1/ω_c ≤ T_i = L/R; Åström & Hägglund, "Advanced PID
//!   Control", 2006, §3.5), so it recovers from saturation without windup.
//! - Motional-EMF feed-forward: the element back-EMF is e = N·∂Φ/∂g·ġ = −k_i·ġ (the
//!   force/current gain equals the EMF/velocity gain by energy conservation; PHYSICS §3,
//!   §4). The fast group supplies per-element v_ff = −k̂_i·ġ̂ through
//!   [`CurrentLoops::set_emf_ff`] (a base-group input the rt owner routes; it is zero
//!   until set, and the PI integral absorbs the EMF either way).
//! - The I²t reference clamp (i_cont continuous, i_pulse for ≤ t_pulse; plan §2b item 4)
//!   is applied upstream by `levitation` on the references; the base task only limits
//!   the voltage to ±V_bus.

use core::f64::consts::PI;
use fabrail_params::{ParamError, SystemParams, derive};
use fabrail_types::{MAX_MAGNETS, fmath};

/// Per-magnet coil current PI controllers.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CurrentLoops {
    /// Active elements (≤ MAX_MAGNETS).
    pub n: u8,
    /// Proportional gain K_p = L̂·ω_c, V/A.
    pub kp_v_per_a: f32,
    /// Integral gain K_i = R̂·ω_c, V/(A·s).
    pub ki_v_per_a_s: f32,
    /// Base period h, s.
    pub dt_s: f32,
    /// Integrator states, V.
    pub integ_v: [f32; MAX_MAGNETS],
    /// Back-calculation tracking gain K_aw = ω_c, 1/s.
    pub k_aw_per_s: f32,
    /// Motional-EMF feed-forward per element, V (set by the owner of the base group).
    pub emf_ff_v: [f32; MAX_MAGNETS],
}

impl CurrentLoops {
    /// Designs the loops from the suspension and levitation tuning blocks.
    ///
    /// Errors when there are more elements than MAX_MAGNETS, the base rate is zero, or
    /// the coil or bandwidth is degenerate (L̂, R̂ or ω_c not positive) with at least one
    /// element — never a silent zero gain.
    pub fn new(p: &SystemParams) -> Result<CurrentLoops, ParamError> {
        let s = &p.design.suspension;
        let n = s.n_magnets;
        if usize::from(n) > MAX_MAGNETS {
            return Err(ParamError::new(
                "design.suspension.n_magnets",
                "exceeds MAX_MAGNETS",
            ));
        }
        let base_hz = p.control.rates.base_hz;
        if base_hz == 0 {
            return Err(ParamError::new("control.rates.base_hz", "must be > 0"));
        }
        let dt_s = (1.0 / f64::from(base_hz)) as f32;
        if n == 0 {
            return Ok(CurrentLoops {
                dt_s,
                ..CurrentLoops::default()
            });
        }
        let l_hat = derive::coil_inductance(s);
        let r_hat = s.r_coil_ohm;
        let bw = p.control.levitation.current_bw_hz;
        if not_pos(l_hat) || not_pos(r_hat) || not_pos(bw) {
            return Err(ParamError::new(
                "control.levitation.current_bw_hz",
                "coil L, R and the current bandwidth must be > 0",
            ));
        }
        let wc = 2.0 * PI * bw;
        Ok(CurrentLoops {
            n,
            kp_v_per_a: (l_hat * wc) as f32,
            ki_v_per_a_s: (r_hat * wc) as f32,
            dt_s,
            integ_v: [0.0; MAX_MAGNETS],
            k_aw_per_s: wc as f32,
            emf_ff_v: [0.0; MAX_MAGNETS],
        })
    }

    /// One base tick: bridge voltages from current references `refs` and measured
    /// currents `meas` (A), limited to ±`v_bus` (V). Elements beyond `n` get 0 V.
    /// A non-finite input zeroes that element's output and integrator (NonFinite is
    /// raised upstream by the frontend/monitor).
    pub fn step(
        &mut self,
        refs: &[f32; MAX_MAGNETS],
        meas: &[f32; MAX_MAGNETS],
        v_bus: f32,
    ) -> [f32; MAX_MAGNETS] {
        let n = usize::from(self.n);
        let vmax = if v_bus.is_finite() {
            fmath::fmaxf(v_bus, 0.0)
        } else {
            0.0
        };
        let mut out = [0.0f32; MAX_MAGNETS];
        for ((((o, integ), r), m), ff) in out
            .iter_mut()
            .zip(self.integ_v.iter_mut())
            .zip(refs.iter())
            .zip(meas.iter())
            .zip(self.emf_ff_v.iter())
            .take(n)
        {
            let e = r - m;
            let u = self.kp_v_per_a * e + *integ + ff;
            if !u.is_finite() {
                *integ = 0.0;
                *o = 0.0;
                continue;
            }
            let v = fmath::clampf(u, -vmax, vmax);
            // Back-calculation: the integrator follows the limited output.
            *integ += self.dt_s * (self.ki_v_per_a_s * e + self.k_aw_per_s * (v - u));
            *o = v;
        }
        out
    }

    /// Sets the per-element motional-EMF feed-forward v_ff = −k̂_i·ġ̂, V (see the
    /// module docs). Non-finite entries are ignored (set to 0).
    pub fn set_emf_ff(&mut self, v: &[f32; MAX_MAGNETS]) {
        for (d, s) in self.emf_ff_v.iter_mut().zip(v.iter()) {
            *d = if s.is_finite() { *s } else { 0.0 };
        }
    }

    /// Zeroes the integrators and the feed-forward (bumpless restart after Off).
    pub fn reset(&mut self) {
        self.integ_v = [0.0; MAX_MAGNETS];
        self.emf_ff_v = [0.0; MAX_MAGNETS];
    }
}

/// Motional EMF of one element e = −k_i·ġ (V) for current gain `k_i_n_per_a` (N/A) and
/// gap rate `gdot_mps` (m/s): the feed-forward the fast group passes to
/// [`CurrentLoops::set_emf_ff`] (PHYSICS §3–§4).
#[inline]
pub fn motional_emf_v(k_i_n_per_a: f32, gdot_mps: f32) -> f32 {
    -k_i_n_per_a * gdot_mps
}

/// NaN-safe "not strictly positive".
#[inline]
fn not_pos(x: f64) -> bool {
    x.is_nan() || x <= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    #[test]
    fn stub_outputs_zero_volts() {
        let p = SystemParams::default();
        let mut c = CurrentLoops::new(&p).unwrap_or_default();
        let v = c.step(&[1.0; MAX_MAGNETS], &[0.0; MAX_MAGNETS], 48.0);
        assert!(v.iter().all(|x| x.is_finite()));
    }

    /// Discrete coil plant L·di/dt = v − R·i − e, exact ZOH over one base tick, with the
    /// one-sample duty delay (v computed at k applies over [k+1, k+2]).
    struct Coil {
        a: f64,
        b: f64,
        i: f64,
        v_applied: f64,
    }

    impl Coil {
        fn new(l: f64, r: f64, dt: f64) -> Coil {
            let a = (-r / l * dt).exp();
            Coil {
                a,
                b: (1.0 - a) / r,
                i: 0.0,
                v_applied: 0.0,
            }
        }
        /// Advances one tick with the previously computed voltage, then latches `v_next`.
        fn tick(&mut self, v_next: f64, emf: f64) {
            self.i = self.a * self.i + self.b * (self.v_applied - emf);
            self.v_applied = v_next;
        }
    }

    fn step_metrics(p: &SystemParams, step_a: f32, v_bus: f32) -> (f64, f64, f64) {
        let mut c = CurrentLoops::new(p).expect("preset current loop");
        c.n = 1;
        let s = &p.design.suspension;
        let (l, r) = (derive::coil_inductance(s), s.r_coil_ohm);
        let dt = f64::from(c.dt_s);
        let mut coil = Coil::new(l, r, dt);
        let mut ys = Vec::new();
        for _ in 0..2000 {
            ys.push(coil.i);
            let mut refs = [0.0f32; MAX_MAGNETS];
            refs[0] = step_a;
            let mut meas = [0.0f32; MAX_MAGNETS];
            meas[0] = coil.i as f32;
            let v = c.step(&refs, &meas, v_bus);
            coil.tick(f64::from(v[0]), 0.0);
        }
        let target = f64::from(step_a);
        let peak = ys.iter().cloned().fold(f64::MIN, f64::max);
        let t10 = ys.iter().position(|&y| y >= 0.1 * target).unwrap_or(0) as f64 * dt;
        let t90 = ys.iter().position(|&y| y >= 0.9 * target).unwrap_or(0) as f64 * dt;
        let final_err = (ys.last().copied().unwrap_or(0.0) - target).abs() / target;
        ((peak - target) / target, t90 - t10, final_err)
    }

    // req: FAB-LEV-008
    #[test]
    fn pi_step_response() {
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let c = CurrentLoops::new(&p).expect("preset design");
            let s = &p.design.suspension;
            let wc = 2.0 * PI * p.control.levitation.current_bw_hz;
            assert!(
                (f64::from(c.kp_v_per_a) / (derive::coil_inductance(s) * wc) - 1.0).abs() < 1e-6,
                "K_p = L̂·ω_c (PHYSICS §4)"
            );
            assert!(
                (f64::from(c.ki_v_per_a_s) / (s.r_coil_ohm * wc) - 1.0).abs() < 1e-6,
                "K_i = R̂·ω_c (PHYSICS §4)"
            );
            assert_eq!(
                p.control.levitation.current_bw_hz, 1000.0,
                "ω_c = 2π·1 kHz (plan §2b.2)"
            );
            // Small-signal 0.3 A step with the one-sample duty delay (LEV-008).
            let (overshoot, rise, err) = step_metrics(&p, 0.3, p.design.power.v_bus_v as f32);
            assert!(
                overshoot <= 0.10,
                "{:?}: overshoot {:.1} % ≤ 10 % (LEV-008)",
                p.design.class,
                overshoot * 100.0
            );
            assert!(
                rise <= 0.3e-3,
                "{:?}: rise (10–90 %) {:.3} ms ≤ 0.3 ms (LEV-008)",
                p.design.class,
                rise * 1e3
            );
            assert!(
                err < 1e-4,
                "{:?}: settles to the reference (1e-4)",
                p.design.class
            );
        }
    }

    /// Saturating 0 → 10 A → 2 A sequence on the DRAWER coil (K_p·10 A ≈ 124 V ≫ 24 V
    /// bus); returns (peak on the up-step, minimum on the down-step, settle time to 1 %
    /// after the up-step, max |v|).
    fn saturating_run(k_aw: f32) -> (f64, f64, f64, f32) {
        let p = presets::drawer();
        let mut c = CurrentLoops::new(&p).expect("drawer design");
        c.n = 1;
        c.k_aw_per_s = k_aw;
        let s = &p.design.suspension;
        let (l, r) = (derive::coil_inductance(s), s.r_coil_ohm);
        let v_bus = p.design.power.v_bus_v as f32;
        let dt = f64::from(c.dt_s);
        let mut coil = Coil::new(l, r, dt);
        let (mut peak, mut low, mut settle, mut vmax) = (0.0f64, f64::MAX, f64::MAX, 0.0f32);
        let mut settled_since: Option<usize> = None;
        for k in 0..6000 {
            let target = if k < 3000 { 10.0 } else { 2.0 };
            let mut refs = [0.0f32; MAX_MAGNETS];
            refs[0] = target;
            let mut meas = [0.0f32; MAX_MAGNETS];
            meas[0] = coil.i as f32;
            let v = c.step(&refs, &meas, v_bus);
            vmax = vmax.max(v[0].abs());
            coil.tick(f64::from(v[0]), 0.0);
            if k < 3000 {
                peak = peak.max(coil.i);
                if (coil.i - 10.0).abs() < 0.1 {
                    settled_since.get_or_insert(k);
                } else {
                    settled_since = None;
                }
            } else {
                low = low.min(coil.i);
            }
        }
        if let Some(k) = settled_since {
            settle = k as f64 * dt;
        }
        (peak, low, settle, vmax)
    }

    // req: FAB-LEV-008
    #[test]
    fn antiwindup_recovers() {
        // With back-calculation the saturated up-step neither overshoots nor the
        // down-step undershoots beyond the small-signal bound (10 %, LEV-008), and the
        // loop settles; the same PI without anti-windup (K_aw = 0) winds up (> 10 %).
        let wc = (2.0 * PI * 1000.0) as f32;
        let (peak, low, settle, vmax) = saturating_run(wc);
        assert!(
            vmax <= 24.0 + 1e-4,
            "bridge voltage limited to ±V_bus: {vmax}"
        );
        assert!(
            peak <= 10.0 * 1.10,
            "saturated up-step overshoot ≤ 10 % (LEV-008): {peak}"
        );
        assert!(
            low >= 2.0 * 0.90,
            "down-step undershoot ≤ 10 % (LEV-008): {low}"
        );
        assert!(
            settle < 10e-3,
            "settles to 1 % within 10 ms after saturation: {settle} s"
        );
        let (peak0, low0, _, _) = saturating_run(0.0);
        assert!(
            peak0 > 10.0 * 1.10 || low0 < 2.0 * 0.90,
            "without anti-windup the same sequence winds up (control case): {peak0} / {low0}"
        );
    }

    #[test]
    fn emf_feedforward_cancels_motional_emf() {
        // With the EMF supplied as feed-forward the tracking error during a constant
        // gap rate is (numerically) zero; without it the PI integral absorbs it slowly.
        let p = presets::door();
        let s = &p.design.suspension;
        let (l, r) = (derive::coil_inductance(s), s.r_coil_ohm);
        let k_i = derive::k_i(s, s.g_star_m, 0.0) as f32;
        let gdot = 0.02f32; // 20 mm/s
        // Plant back-EMF e = N·∂Φ/∂g·ġ = −k_i·ġ; the feed-forward supplies exactly e.
        let emf = f64::from(motional_emf_v(k_i, gdot));
        let run = |ff: bool| -> f64 {
            let mut c = CurrentLoops::new(&p).expect("door");
            c.n = 1;
            let mut coil = Coil::new(l, r, f64::from(c.dt_s));
            let mut v_ff = [0.0f32; MAX_MAGNETS];
            if ff {
                v_ff[0] = emf as f32;
            }
            c.set_emf_ff(&v_ff);
            let mut worst = 0.0f64;
            for k in 0..400 {
                let mut refs = [0.0f32; MAX_MAGNETS];
                refs[0] = 1.0;
                let mut meas = [0.0f32; MAX_MAGNETS];
                meas[0] = coil.i as f32;
                let v = c.step(&refs, &meas, 48.0);
                coil.tick(f64::from(v[0]), emf);
                if k > 100 {
                    worst = worst.max((coil.i - 1.0).abs());
                }
            }
            worst
        };
        let (with, without) = (run(true), run(false));
        assert!(
            with < 1e-4,
            "EMF feed-forward removes the tracking error: {with}"
        );
        assert!(without > with, "without feed-forward the error is larger");
    }
}
