//! Test-only nonlinear heave/pitch/roll plant and closed-loop harness for the levitation
//! unit tests (not the simulator: fabrail-plant is the truth model). Owner: A1.
//!
//! One rigid body in heave, pitch and roll under n hybrid elements with the co-energy
//! force F_i = μ₀A_p·(M_pm + N·i_e,i)²/(2g_i + h′)² (`derive::pm_force` at y = 0,
//! PHYSICS §2) at the element gaps g_i = g + r_x,i·θ_p + r_y,i·θ_r (lever arms about the
//! CoM), the current lag τ_cl (lag coil mode, PHYSICS §4), the eddy lag τ_e on the force,
//! gravity on the CoM height (J·θ̈ = −Σr·F + m·g₀·h_cg·θ, the `derive::ModeModel`
//! convention, inertia scaled with the mass), the upper stop, the landing rollers and the
//! DRAWER captive seat faces (active while the kinematic x is inside the slot and the gap
//! is within the seat window; a supported body is held level). RK4 at the 50 µs base
//! tick; the controller runs at the fast rate with one fast period of LET delay (a
//! command computed at release k is applied over [k+1, k+2]).
#![cfg(test)]
#![allow(clippy::needless_range_loop)]

use fabrail_params::derive::{self, G0_MPS2};
use fabrail_params::{SuspensionDesign, SystemParams};
use fabrail_types::{MAX_MAGNETS, PosNm, Tick};

use super::{LevOutput, Levitation};
use crate::msg::{LevDirective, LevPhase, Measurements, SeatStatus};

/// Plant state and constants.
#[derive(Clone, Debug)]
pub struct HeavePlant {
    pub m_kg: f64,
    pub n: usize,
    pub s: SuspensionDesign,
    /// Gap at the CoM projection, m.
    pub g_m: f64,
    pub gd_mps: f64,
    /// Pitch and roll as gap slopes ∂g/∂x, ∂g/∂y (rad) and their rates.
    pub th: [f64; 2],
    pub thd: [f64; 2],
    pub i_a: [f64; MAX_MAGNETS],
    pub ie_a: [f64; MAX_MAGNETS],
    pub tau_cl_s: f64,
    pub tau_e_s: f64,
    /// Seat faces [upper (g_seat − c), lower (g_seat)], m (None: no seat).
    pub seat: Option<(f64, f64)>,
    pub x_m: f64,
    pub slot_len_m: f64,
    /// Lever arms (r_x, r_y) per element, m.
    pub r: [[f64; 2]; MAX_MAGNETS],
    /// Gap-sensor noise rms (uniform, per sample), m; 0 = noise-free.
    pub noise_m: f64,
    /// Noise generator state (64-bit LCG, Knuth MMIX constants).
    pub rng: u64,
    /// Pitch/roll inertia at m*, kg·m² ([J_y, J_x]).
    pub j_star: [f64; 2],
    pub m_star_kg: f64,
    /// CoM height above the pole plane, m.
    pub h_cg_m: f64,
}

type St = [f64; 6 + 2 * MAX_MAGNETS];

impl HeavePlant {
    /// Free body at gap `g0_m`, level, zero current, x well outside the seat slot.
    pub fn new(p: &SystemParams, m_kg: f64, g0_m: f64) -> HeavePlant {
        let s = p.design.suspension;
        let c = &p.design.carriage;
        let seat = if s.g_seat_m > 0.0 {
            Some((s.g_seat_m - p.design.landing.seat_z_clearance_m, s.g_seat_m))
        } else {
            None
        };
        let n = usize::from(s.n_magnets);
        let mut r = [[0.0; 2]; MAX_MAGNETS];
        let mut z = 0.0;
        for k in 0..n {
            r[k] = [s.stations[k].x_m - c.cg_m[0], s.stations[k].y_m - c.cg_m[1]];
            z += s.stations[k].z_m;
        }
        HeavePlant {
            m_kg,
            n,
            s,
            g_m: g0_m,
            gd_mps: 0.0,
            th: [0.0; 2],
            thd: [0.0; 2],
            i_a: [0.0; MAX_MAGNETS],
            ie_a: [0.0; MAX_MAGNETS],
            tau_cl_s: derive::tau_current_loop_s(p),
            tau_e_s: s.tau_eddy_s,
            seat,
            x_m: 0.1,
            slot_len_m: p.design.landing.slot_len_m,
            r,
            noise_m: 0.0,
            rng: 0x2545_F491_4F6C_DD1D,
            j_star: [c.j_kgm2[1], c.j_kgm2[0]],
            m_star_kg: c.m_star_kg,
            h_cg_m: c.cg_m[2] - z / n.max(1) as f64,
        }
    }

    /// DRAWER at rest in its seat: on the lower face if heavier than the PM force there,
    /// else pulled against the upper face.
    pub fn seated(p: &SystemParams, m_kg: f64) -> HeavePlant {
        let mut h = HeavePlant::new(p, m_kg, p.design.suspension.g_seat_m);
        h.x_m = 0.0;
        let (up, lo) = h.seat.unwrap_or((h.g_m, h.g_m));
        let f_lo = h.n as f64 * derive::pm_force(&h.s, lo, 0.0, 0.0);
        h.g_m = if f_lo > m_kg * G0_MPS2 { up } else { lo };
        h
    }

    fn in_slot(&self) -> bool {
        self.x_m < self.slot_len_m
    }

    /// Element gap k for state y.
    fn gap_k(&self, y: &St, k: usize) -> f64 {
        y[0] + self.r[k][0] * y[2] + self.r[k][1] * y[4]
    }

    fn deriv(&self, y: &St, iref: &[f32; MAX_MAGNETS]) -> St {
        let mut d = [0.0; 6 + 2 * MAX_MAGNETS];
        let n = self.n;
        let (mut f_tot, mut tq) = (0.0, [0.0, 0.0]);
        for k in 0..n {
            let i = y[6 + k];
            let ie = if self.tau_e_s > 0.0 {
                y[6 + MAX_MAGNETS + k]
            } else {
                i
            };
            let f = derive::pm_force(&self.s, self.gap_k(y, k), 0.0, ie);
            f_tot += f;
            tq[0] += self.r[k][0] * f;
            tq[1] += self.r[k][1] * f;
            d[6 + k] = (f64::from(iref[k]) - i) / self.tau_cl_s;
            d[6 + MAX_MAGNETS + k] = if self.tau_e_s > 0.0 {
                (i - ie) / self.tau_e_s
            } else {
                0.0
            };
        }
        let scale = self.m_kg / self.m_star_kg;
        d[0] = y[1];
        d[1] = G0_MPS2 - f_tot / self.m_kg;
        for a in 0..2 {
            let j = self.j_star[a] * scale;
            d[2 + 2 * a] = y[3 + 2 * a];
            d[3 + 2 * a] = if j > 0.0 {
                (-tq[a] + self.m_kg * G0_MPS2 * self.h_cg_m * y[2 + 2 * a]) / j
            } else {
                0.0
            };
        }
        d
    }

    /// One base tick with per-element current references `iref`.
    pub fn tick(&mut self, iref: &[f32; MAX_MAGNETS], h: f64) {
        let mut y: St = [0.0; 6 + 2 * MAX_MAGNETS];
        y[0] = self.g_m;
        y[1] = self.gd_mps;
        y[2] = self.th[0];
        y[3] = self.thd[0];
        y[4] = self.th[1];
        y[5] = self.thd[1];
        for k in 0..MAX_MAGNETS {
            y[6 + k] = self.i_a[k];
            y[6 + MAX_MAGNETS + k] = self.ie_a[k];
        }
        let g_prev = self.g_m;
        let add = |a: &St, b: &St, k: f64| {
            let mut o = *a;
            for j in 0..o.len() {
                o[j] += k * b[j];
            }
            o
        };
        let k1 = self.deriv(&y, iref);
        let k2 = self.deriv(&add(&y, &k1, 0.5 * h), iref);
        let k3 = self.deriv(&add(&y, &k2, 0.5 * h), iref);
        let k4 = self.deriv(&add(&y, &k3, h), iref);
        for j in 0..y.len() {
            y[j] += h / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
        }
        self.g_m = y[0];
        self.gd_mps = y[1];
        self.th = [y[2], y[4]];
        self.thd = [y[3], y[5]];
        for k in 0..MAX_MAGNETS {
            self.i_a[k] = y[6 + k];
            self.ie_a[k] = y[6 + MAX_MAGNETS + k];
        }
        // Contacts (inelastic, on the CoM gap; a supported body is held level): upper
        // stop, rollers, captive seat faces.
        let (mut lo, mut hi) = (self.s.g_up_m, self.s.g_land_m);
        if let Some((up, low)) = self.seat
            && self.in_slot()
            && g_prev >= up - 1e-9
            && g_prev <= low + 1e-9
        {
            lo = up;
            hi = low;
        }
        let mut contact = false;
        if self.g_m < lo {
            self.g_m = lo;
            self.gd_mps = self.gd_mps.max(0.0);
            contact = true;
        }
        if self.g_m > hi {
            self.g_m = hi;
            self.gd_mps = self.gd_mps.min(0.0);
            contact = true;
        }
        if contact || (self.g_m - hi).abs() < 1e-12 || (self.g_m - lo).abs() < 1e-12 {
            self.th = [0.0; 2];
            self.thd = [0.0; 2];
        }
    }

    /// Uniform noise sample with rms `noise_m` (√3·σ half-width).
    fn noise(&mut self) -> f64 {
        self.rng = self
            .rng
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let u = (self.rng >> 11) as f64 / (1u64 << 53) as f64;
        (2.0 * u - 1.0) * 3f64.sqrt() * self.noise_m
    }

    /// Controller measurements (per-element gaps, optional gap noise).
    pub fn measure(&mut self, tick: u64) -> Measurements {
        let mut m = Measurements {
            tick: Tick(tick),
            x: PosNm::from_m_f64(self.x_m),
            referenced: true,
            enc_ok: true,
            ..Measurements::default()
        };
        for k in 0..self.n {
            let g = self.g_m + self.r[k][0] * self.th[0] + self.r[k][1] * self.th[1];
            let e = if self.noise_m > 0.0 {
                self.noise()
            } else {
                0.0
            };
            m.gap_m[k] = (g + e) as f32;
            m.coil_i_a[k] = self.i_a[k] as f32;
            m.gap_ok |= 1 << k;
        }
        m
    }
}

/// Unseat run summary.
#[derive(Clone, Copy, Debug, Default)]
pub struct UnseatReport {
    pub i_liftoff_a: f32,
    pub m_liftoff_kg: f32,
    pub t_in_slot_s: f32,
    pub cleared: bool,
    pub levitated: bool,
    pub saturated: bool,
    pub i2t_peak_a2s: f32,
    pub g_final_m: f32,
}

/// Levitated-run summary.
#[derive(Clone, Copy, Debug, Default)]
pub struct Trace {
    pub i_mean_last: f32,
}

/// Closed loop: Levitation at the fast rate, plant at the base rate, LET delay.
pub struct Harness {
    pub lev: Levitation,
    pub plant: HeavePlant,
    pub cmd: [f32; MAX_MAGNETS],
    pub tick: u64,
    pub decim: u64,
    pub h_s: f64,
    pub x_target_m: f64,
    pub x_accel_mps2: f64,
    pub vx_mps: f64,
}

impl Harness {
    pub fn new(p: &SystemParams, plant: HeavePlant) -> Harness {
        Harness {
            lev: Levitation::new(p).expect("preset levitation design"),
            x_target_m: plant.x_m,
            plant,
            cmd: [0.0; MAX_MAGNETS],
            tick: 0,
            decim: u64::from(p.control.rates.fast_decim),
            h_s: 1.0 / f64::from(p.control.rates.base_hz),
            x_accel_mps2: 0.0,
            vx_mps: 0.0,
        }
    }

    /// One fast release with directive `d`.
    pub fn release(&mut self, d: LevDirective) -> LevOutput {
        let m = self.plant.measure(self.tick);
        let out = self.lev.step(&m, &d);
        for _ in 0..self.decim {
            // LET: the previous release's command is applied over this period.
            let cmd = self.cmd;
            self.plant.tick(&cmd, self.h_s);
            self.tick += 1;
            // Kinematic x: accelerate toward the target (no braking), or move at 0.1 m/s.
            let h = self.h_s;
            if self.x_accel_mps2 > 0.0 && self.plant.x_m < self.x_target_m {
                self.vx_mps += self.x_accel_mps2 * h;
                self.plant.x_m = (self.plant.x_m + self.vx_mps * h).min(self.x_target_m);
            } else if (self.plant.x_m - self.x_target_m).abs() > 1e-9 {
                let step = 0.1 * h;
                let dx = (self.x_target_m - self.plant.x_m).clamp(-step, step);
                self.plant.x_m += dx;
                self.vx_mps = 0.0;
            }
        }
        self.cmd = out.coil_ref_a;
        out
    }

    /// Runs `secs` with a fixed directive; returns the last output.
    pub fn run_for(&mut self, _p: &SystemParams, secs: f64, d: LevDirective) -> LevOutput {
        let n = (secs / (self.h_s * self.decim as f64)).round() as usize;
        let mut last = LevOutput::default();
        for _ in 0..n {
            last = self.release(d);
        }
        last
    }

    /// Unseat, hold in the slot, drive out at `a_exit` m/s², run `secs` in total.
    pub fn run_unseat(&mut self, _p: &SystemParams, a_exit: f64, secs: f64) -> UnseatReport {
        let mut r = UnseatReport::default();
        let dt = self.h_s * self.decim as f64;
        let n = (secs / dt).round() as usize;
        let mut t = 0.0f64;
        let mut in_slot_seen = false;
        for _ in 0..n {
            let out = self.release(LevDirective::Unseat);
            t += dt;
            if out.saturated {
                r.saturated = true;
            }
            let peak = self.lev.i2t.acc_a2s.iter().cloned().fold(0.0f32, f32::max);
            r.i2t_peak_a2s = r.i2t_peak_a2s.max(peak);
            if !in_slot_seen && out.seat == SeatStatus::InSlot {
                in_slot_seen = true;
                r.t_in_slot_s = t as f32;
                r.m_liftoff_kg = self.lev.seat.m_liftoff_kg;
                r.i_liftoff_a = self.lev.lift.liftoff_i_a;
                // Drive out of the slot.
                self.x_target_m = 0.3;
                self.x_accel_mps2 = a_exit;
            }
            if out.seat == SeatStatus::Clear {
                r.cleared = true;
            }
            if out.phase == LevPhase::Levitated && out.seat == SeatStatus::Clear {
                r.levitated = true;
            }
        }
        r.g_final_m = self.plant.g_m as f32;
        r
    }
}

/// Catches the plant in flight with `Levitate` and runs `secs`; the trace holds the
/// mean coil current over the final second (f32 loop jitter averaged out).
pub fn run_levitated(p: &SystemParams, plant: &mut HeavePlant, secs: f64) -> (Levitation, Trace) {
    let mut h = Harness::new(p, plant.clone());
    let _ = h.run_for(p, (secs - 1.0).max(0.0), LevDirective::Levitate);
    let n = plant.n.max(1);
    let per = (1.0 / (h.h_s * h.decim as f64)).round() as usize;
    let mut acc = 0.0f64;
    for _ in 0..per {
        let _ = h.release(LevDirective::Levitate);
        acc += h.plant.i_a.iter().take(n).sum::<f64>() / n as f64;
    }
    *plant = h.plant.clone();
    (
        h.lev,
        Trace {
            i_mean_last: (acc / per.max(1) as f64) as f32,
        },
    )
}
