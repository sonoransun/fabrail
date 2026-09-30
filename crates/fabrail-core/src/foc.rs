//! Clarke/Park transforms and the exact electrical angle from PosNm by integer modulo of 2*tau_p (PHYSICS §7). Owner: A1.
//!
//! Frozen API (W0). The electrical angle is θ_e = 2π·(x_nm mod 2τ_p,nm)/(2τ_p,nm),
//! with the modulo taken in integers (`rem_euclid`) so the angle is exact at any travel
//! (core design §A.4 item 2; FOC test `angle_exact_far`). Amplitude-invariant Clarke:
//! i_α = i_a, i_β = (i_a + 2·i_b)/√3.
//!
//! [`DqPi`] is the LSM d/q current PI the firmware base task runs (core design §F);
//! in SIL the LSM is a current-controlled FOC abstraction (PHYSICS §7) and the base task
//! passes `LsmCmd` through. Design: K_p = L·ω_c, K_i = R·ω_c (the coil-loop rule of
//! PHYSICS §4 applied to the phase winding, ω_c = 2π·`levitation.current_bw_hz`),
//! cross-coupling decoupling ∓ω_e·L·i and back-EMF feed-forward e_q = (2/3)·K_f·v
//! (from F·v = 1.5·e_q·i_q with F = K_f·i_q, PRP-002), voltage vector limited to the
//! linear SVPWM range V_bus/√3 with back-calculation anti-windup (Åström & Hägglund,
//! "Advanced PID Control", 2006, §3.5).

use core::f32::consts::TAU;
use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{PosNm, fmath};

/// √3 (Clarke/SVPWM constants).
const SQRT3: f32 = 1.732_050_8;

/// Angle quantisation used by [`elec_angle`]: the pitch fraction is formed as an exact
/// integer in [0, 2²⁴) so its f32 conversion is exact and θ carries one rounding only
/// (≤ 2π/2²⁴ ≈ 3.7e-7 rad).
const ANGLE_FRAC_BITS: u32 = 24;

/// Electrical angle θ_e ∈ [0, 2π) at position `x` for pole pitch `tau_p_nm` (nm).
/// A non-positive pitch returns 0. The result depends only on x mod 2τ_p (integer), so
/// it is bit-identical at any distance (PRP-009).
pub fn elec_angle(x: PosNm, tau_p_nm: i64) -> f32 {
    if tau_p_nm <= 0 {
        return 0.0;
    }
    let period = tau_p_nm.saturating_mul(2);
    let r = x.0.rem_euclid(period); // 0 ≤ r < period (exact integer)
    // frac = ⌊r·2²⁴/period⌋ < 2²⁴ (exact in f32). u64 suffices while period < 2³⁹ nm
    // (550 m pitch; every preset is ≤ 100 mm), i128 beyond.
    let frac = if period < (1i64 << 39) {
        ((r as u64) << ANGLE_FRAC_BITS) / (period as u64)
    } else {
        ((i128::from(r) << ANGLE_FRAC_BITS) / i128::from(period)) as u64
    };
    let th = (frac as f32) * (TAU / (1u32 << ANGLE_FRAC_BITS) as f32);
    if th >= TAU { 0.0 } else { th }
}

/// Amplitude-invariant Clarke transform (a, b, c) → (α, β): α = a, β = (a + 2b)/√3
/// (assumes a + b + c = 0; c is not used).
pub fn clarke(i_abc: [f32; 3]) -> [f32; 2] {
    [i_abc[0], (i_abc[0] + 2.0 * i_abc[1]) / SQRT3]
}

/// Park transform (α, β) → (d, q) at angle `theta` (rad):
/// d = α·cos θ + β·sin θ, q = −α·sin θ + β·cos θ.
pub fn park(ab: [f32; 2], theta: f32) -> [f32; 2] {
    let (s, c) = (fmath::sinf(theta), fmath::cosf(theta));
    [ab[0] * c + ab[1] * s, -ab[0] * s + ab[1] * c]
}

/// Inverse Park (d, q) → (α, β): α = d·cos θ − q·sin θ, β = d·sin θ + q·cos θ.
pub fn inv_park(dq: [f32; 2], theta: f32) -> [f32; 2] {
    let (s, c) = (fmath::sinf(theta), fmath::cosf(theta));
    [dq[0] * c - dq[1] * s, dq[0] * s + dq[1] * c]
}

/// Inverse Clarke (α, β) → (a, b, c) (amplitude-invariant):
/// a = α, b = (−α + √3·β)/2, c = (−α − √3·β)/2.
pub fn inv_clarke(ab: [f32; 2]) -> [f32; 3] {
    let h = 0.5 * SQRT3 * ab[1];
    [ab[0], -0.5 * ab[0] + h, -0.5 * ab[0] - h]
}

/// Measured (i_d, i_q) from phase currents at position `x` (Clarke, then Park at the
/// exact electrical angle).
pub fn dq_from_abc(i_abc: [f32; 3], x: PosNm, tau_p_nm: i64) -> [f32; 2] {
    park(clarke(i_abc), elec_angle(x, tau_p_nm))
}

/// LSM d/q current PI with decoupling (firmware base task; PHYSICS §7).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DqPi {
    /// Proportional gain, V/A.
    pub kp_v_per_a: f32,
    /// Integral gain, V/(A·s).
    pub ki_v_per_a_s: f32,
    /// Base period, s.
    pub dt_s: f32,
    /// Integrator states (d, q), V.
    pub integ_v: [f32; 2],
    /// Phase inductance L, H (decoupling).
    pub l_h: f32,
    /// Back-EMF q-axis constant (2/3)·K_f, V·s/m.
    pub ke_q_v_s_per_m: f32,
    /// Pole pitch τ_p, nm.
    pub tau_p_nm: i64,
    /// Back-calculation gain, 1/s.
    pub k_aw_per_s: f32,
    /// Previous position (speed estimate for decoupling/back-EMF).
    pub x_prev: PosNm,
    /// A previous position exists.
    pub primed: bool,
}

impl DqPi {
    /// Designs the PI from `design.lsm` (L, R, τ_p, K_f) and the current bandwidth.
    pub fn new(p: &SystemParams) -> Result<DqPi, ParamError> {
        let l = &p.design.lsm;
        let base_hz = p.control.rates.base_hz;
        if base_hz == 0 {
            return Err(ParamError::new("control.rates.base_hz", "must be > 0"));
        }
        let bw = p.control.levitation.current_bw_hz;
        if [bw, l.l_phase_h, l.r_phase_ohm, l.tau_p_m]
            .iter()
            .any(|v| v.is_nan() || *v <= 0.0)
        {
            return Err(ParamError::new(
                "design.lsm",
                "L, R, tau_p and current_bw_hz must be > 0",
            ));
        }
        let wc = 2.0 * core::f64::consts::PI * bw;
        Ok(DqPi {
            kp_v_per_a: (l.l_phase_h * wc) as f32,
            ki_v_per_a_s: (l.r_phase_ohm * wc) as f32,
            dt_s: (1.0 / f64::from(base_hz)) as f32,
            integ_v: [0.0; 2],
            l_h: l.l_phase_h as f32,
            ke_q_v_s_per_m: (2.0 / 3.0 * l.k_f_n_per_a) as f32,
            tau_p_nm: fabrail_types::fmath::round(l.tau_p_m * 1.0e9) as i64,
            k_aw_per_s: wc as f32,
            x_prev: PosNm::ZERO,
            primed: false,
        })
    }

    /// One base tick: phase voltages (a, b, c) from the dq references, measured phase
    /// currents, position and bus voltage.
    pub fn step(
        &mut self,
        iq_ref_a: f32,
        id_ref_a: f32,
        i_abc_a: [f32; 3],
        x: PosNm,
        v_bus: f32,
    ) -> [f32; 3] {
        let th = elec_angle(x, self.tau_p_nm);
        let [id, iq] = park(clarke(i_abc_a), th);
        // Speed from the integer position difference (exact at any distance).
        let v = if self.primed && self.dt_s > 0.0 {
            x.diff_m(self.x_prev) / self.dt_s
        } else {
            0.0
        };
        self.x_prev = x;
        self.primed = true;
        let tau_p_m = self.tau_p_nm as f32 * 1.0e-9;
        let w_e = if tau_p_m > 0.0 {
            core::f32::consts::PI * v / tau_p_m
        } else {
            0.0
        };
        let (ed, eq) = (id_ref_a - id, iq_ref_a - iq);
        // Feed-forward: decoupling and back-EMF.
        let ff_d = -w_e * self.l_h * iq;
        let ff_q = w_e * self.l_h * id + self.ke_q_v_s_per_m * v;
        let ud = self.kp_v_per_a * ed + self.integ_v[0] + ff_d;
        let uq = self.kp_v_per_a * eq + self.integ_v[1] + ff_q;
        // Linear SVPWM range |v_dq| ≤ V_bus/√3.
        let vmax = fmath::fmaxf(v_bus, 0.0) / SQRT3;
        let mag = fmath::hypotf(ud, uq);
        let k = if mag > vmax && mag > 0.0 {
            vmax / mag
        } else {
            1.0
        };
        let (vd, vq) = (ud * k, uq * k);
        // Back-calculation anti-windup.
        self.integ_v[0] += self.dt_s * (self.ki_v_per_a_s * ed + self.k_aw_per_s * (vd - ud));
        self.integ_v[1] += self.dt_s * (self.ki_v_per_a_s * eq + self.k_aw_per_s * (vq - uq));
        let out = inv_clarke(inv_park([vd, vq], th));
        if out.iter().all(|v| v.is_finite()) {
            out
        } else {
            self.reset();
            [0.0; 3]
        }
    }

    /// Zeroes the integrators.
    pub fn reset(&mut self) {
        self.integ_v = [0.0; 2];
        self.primed = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_is_finite() {
        let th = elec_angle(PosNm(12_000_000_000), 25_000_000);
        assert!(th.is_finite());
    }

    // req: FAB-PRP-009
    #[test]
    fn angle_exact_far() {
        // HEAVY pitch 50 mm (core §C.2): period 100 mm = 1e8 nm.
        let tau = 50_000_000i64;
        let period = 2 * tau;
        for near in [
            0i64,
            1,
            12_345_678,
            49_999_999,
            50_000_000,
            99_999_999,
            -1,
            -37_000_001,
        ] {
            let th0 = elec_angle(PosNm(near), tau);
            // f64 reference of 2π·frac.
            let frac = near.rem_euclid(period) as f64 / period as f64;
            let want = 2.0 * std::f64::consts::PI * frac;
            assert!(
                (f64::from(th0) - want).abs() <= 1.0e-6,
                "θ({near}) = {th0} vs {want} (tol 1e-6 rad: 2π/2²⁴ quantisation + f32 rounding; PRP-009)"
            );
            assert!((0.0..TAU).contains(&th0), "θ in [0, 2π)");
            // Bit-identical at any distance: +k·2τ_p up to ~9e9 m of travel.
            for k in [1i64, 1_000, 10_000_000, 90_000_000_000] {
                let far = near + k * period;
                assert_eq!(
                    elec_angle(PosNm(far), tau).to_bits(),
                    th0.to_bits(),
                    "θ exact at x = {far} nm (PRP-009: integer rem_euclid)"
                );
                assert_eq!(
                    elec_angle(PosNm(near - k * period), tau).to_bits(),
                    th0.to_bits()
                );
            }
        }
        assert_eq!(elec_angle(PosNm(5), 0), 0.0, "non-positive pitch");
        assert_eq!(
            elec_angle(PosNm(i64::MIN), tau),
            elec_angle(PosNm(i64::MIN + period), tau)
        );
    }

    #[test]
    fn clarke_park_roundtrip() {
        let th = 1.234f32;
        // Balanced currents with amplitude 2 A at electrical angle th + 0.3 (q-axis
        // aligned so that (d, q) = (2·cos 0.3·…)).
        let amp = 2.0f32;
        let ph = th + 0.3;
        let abc = [
            amp * ph.cos(),
            amp * (ph - TAU / 3.0).cos(),
            amp * (ph + TAU / 3.0).cos(),
        ];
        let ab = clarke(abc);
        assert!((ab[0] - amp * ph.cos()).abs() < 1e-5);
        assert!(
            (ab[1] - amp * ph.sin()).abs() < 1e-5,
            "amplitude-invariant β"
        );
        let dq = park(ab, th);
        assert!((dq[0] - amp * 0.3f32.cos()).abs() < 1e-5);
        assert!((dq[1] - amp * 0.3f32.sin()).abs() < 1e-5);
        let back = inv_clarke(inv_park(dq, th));
        for (b, a) in back.iter().zip(abc.iter()) {
            assert!((b - a).abs() < 1e-5, "inverse transforms round-trip");
        }
    }

    #[test]
    fn dq_pi_tracks_q_current() {
        // Phase winding R–L at standstill driven by the PI; dq plant d(i)/dt = (v − R·i)/L.
        let p = fabrail_params::presets::door();
        let mut pi = DqPi::new(&p).expect("door LSM design");
        let (r, l) = (
            p.design.lsm.r_phase_ohm as f32,
            p.design.lsm.l_phase_h as f32,
        );
        let x = PosNm(123_456_789);
        let th = elec_angle(x, pi.tau_p_nm);
        let mut i_abc = [0.0f32; 3];
        let mut v_prev = [0.0f32; 3];
        let dt = pi.dt_s;
        for _ in 0..400 {
            let v = pi.step(2.0, 0.0, i_abc, x, 48.0);
            for (i, vp) in i_abc.iter_mut().zip(v_prev.iter()) {
                *i += dt * (vp - r * *i) / l;
            }
            v_prev = v;
        }
        let dq = park(clarke(i_abc), th);
        assert!((dq[1] - 2.0).abs() < 0.02, "i_q → 2 A (1 %): {}", dq[1]);
        assert!(dq[0].abs() < 0.02, "i_d → 0: {}", dq[0]);
    }
}
