//! Heave1 tier: z, z', i, Phi_eff (equivalent to Carriage6 constrained heave to 1e-9) (PHYSICS §5). Owner: A6a.
//!
//! One lumped element carrying the whole weight (n_magnets elements in parallel), used
//! for the analytic anchors, gain studies, the vertical pulse and the compressed thermal
//! ramp (plan §2b item 13). x-axis inputs (hand, lean, LSM) are ignored.
//!
//! As implemented (PHYSICS §5): the n lift elements share one gap g = g* + z̄_rail − z
//! (z = heave of the reference point, up +; z̄_rail the mean rail deviation under the lift
//! stations at the fixed travel x₀), one current (the mean of the lift elements'
//! commands), one eddy-lagged flux and one coil temperature; the force is n·F(g) with the
//! same element model, contact laws (per element, m_eff = m/n) and coil electrics as
//! Carriage6, so a symmetric, untilted Carriage6 heave reproduces it to round-off
//! (anchor `tier_equivalence`, 1e-9). Hold-down elements are not modelled here.

use fabrail_params::derive::G0_MPS2;
use fabrail_types::{MAX_MAGNETS, fmath};

use crate::body::{
    Common, EnergyAudit, L_COIL_CU, L_CONTACT, L_ELEC_IN, L_FLOOR, N_LEDGER, ledger_from,
    power_truth, rail_at, rest_gap, vertical_contacts,
};
use crate::coil::{self, I2tBudget, LagProfile};
use crate::integrator::{STAGE_FLOOR_INDEX, rk4_step};
use crate::magnet::{ElementGeom, ElementModel};
use crate::model::{
    CoilMode, ContactForces, ContactState, InitialState, ModelConfig, PlantInputs, PlantModel,
    Tier, Truth,
};
use crate::params::PlantParams;
use crate::power::{self, PowerFlow};
use crate::thermal::{self, ThermalState};

const H_Z: usize = 0;
const H_ZD: usize = 1;
const H_I: usize = 2;
const H_PHI: usize = 3;
const H_TCOIL: usize = 4;
const H_TMAG: usize = 5;
const H_VHOLD: usize = 6;
const H_LEDGER: usize = 7;
const H_N: usize = H_LEDGER + N_LEDGER;

type HState = [f64; H_N];

/// Step-constant context.
#[derive(Clone, Copy, Debug)]
struct HCtx {
    m: f64,
    h: f64,
    voltage: bool,
    lag_state: bool,
    prof: LagProfile,
    v: f64,
    limit: f64,
    mains_ok: bool,
    t_amb_c: f64,
}

/// Per-evaluation outputs.
#[derive(Clone, Copy, Debug, Default)]
struct HAux {
    g: f64,
    i: f64,
    phi: f64,
    l_inc: f64,
    e_mot: f64,
    r: f64,
    zdd: f64,
    contact: ContactForces,
    pf: PowerFlow,
}

/// Heave-only tier.
#[derive(Clone, Debug)]
pub struct Heave1 {
    p: PlantParams,
    cfg: ModelConfig,
    com: Common,
    model: ElementModel,
    n_l: f64,
    e: f64,
    zr: f64,
    x0: f64,
    y: HState,
    aux: HAux,
    truth: Truth,
    i2t: I2tBudget,
    payload_kg: f64,
    h_last: f64,
}

impl Heave1 {
    /// A Heave1 model over `p`, reset to the default initial state.
    pub fn new(p: &PlantParams, cfg: &ModelConfig) -> Heave1 {
        let com = Common::new(p, Tier::Heave1.nominal_h_s());
        let model = com.lift_model();
        let n_l = com.n_lift.max(1) as f64;
        let mut h = Heave1 {
            p: p.clone(),
            cfg: *cfg,
            com,
            model,
            n_l,
            e: 1.0,
            zr: 0.0,
            x0: 0.0,
            y: [0.0; H_N],
            aux: HAux::default(),
            truth: Truth::initial(p, &InitialState::default()),
            i2t: I2tBudget::default(),
            payload_kg: 0.0,
            h_last: Tier::Heave1.nominal_h_s(),
        };
        h.reset(&InitialState::default());
        h
    }

    /// The construction-time options.
    pub fn config(&self) -> &ModelConfig {
        &self.cfg
    }

    fn mass(&self) -> f64 {
        self.p.m_body_kg + self.payload_kg
    }

    /// Places the reference point at heave `z_m` (up +, 0 at g*) with rate `z_dot_mps`,
    /// keeping the current and temperatures; the eddy flux is reset to its quasi-static
    /// value. For tests and studies (initial perturbations).
    pub fn set_heave(&mut self, z_m: f64, z_dot_mps: f64) {
        self.y[H_Z] = z_m;
        self.y[H_ZD] = z_dot_mps;
        let g = self.p.design.suspension.g_star_m + self.zr - z_m;
        let ev = self.model.eval(
            &ElementGeom {
                g_m: g,
                y_m: 0.0,
                e: self.e,
                de_dx_per_m: 0.0,
            },
            self.y[H_I],
            self.y[H_TMAG],
        );
        self.y[H_PHI] = ev.phi_wb;
        let u = PlantInputs {
            payload_kg: self.payload_kg,
            t_ambient_c: self.y[H_TMAG],
            ..PlantInputs::default()
        };
        let ctx = self.ctx(&u, self.h_last);
        let mut aux = self.aux;
        let _ = self.eval(&ctx, 0.0, &self.truth.floor_acc_mps2, &self.y, &mut aux);
        self.aux = aux;
        let (t, floor) = (self.truth.t_s, self.truth.floor_acc_mps2);
        self.fill_truth(t, &floor);
    }

    /// Heave rate ż of the reference point, m/s.
    pub fn heave_rate_mps(&self) -> f64 {
        self.y[H_ZD]
    }

    /// Mean command over the lift elements (the lumped coil's command).
    fn mean_lift(&self, v: &[f64; MAX_MAGNETS]) -> f64 {
        let sum: f64 = self
            .com
            .st
            .iter()
            .zip(v.iter())
            .take(self.com.n)
            .filter(|(st, _)| st.lift)
            .map(|(_, vk)| vk)
            .sum();
        sum / self.n_l
    }

    /// Energy terms of the present state (PHYSICS §5, §9): kinetic ½·m·ż², gravity
    /// m·g₀·z, and the n lumped elements' co-energy, stored energy, M·Φ and leakage.
    pub fn energy_audit(&self) -> EnergyAudit {
        let m = self.mass();
        let g = self.p.design.suspension.g_star_m + self.zr - self.y[H_Z];
        let i = self.y[H_I];
        let t_mag = self.y[H_TMAG];
        let ev = self.model.eval(
            &ElementGeom {
                g_m: g,
                y_m: 0.0,
                e: self.e,
                de_dx_per_m: 0.0,
            },
            i,
            t_mag,
        );
        EnergyAudit {
            kinetic_trans_j: 0.5 * m * self.y[H_ZD] * self.y[H_ZD],
            kinetic_rot_j: 0.0,
            gravity_j: m * G0_MPS2 * self.y[H_Z],
            coenergy_j: self.n_l * ev.coenergy_j(),
            stored_j: self.n_l * ev.stored_j(),
            m_phi_j: self.n_l * self.model.m_pm_at_temp(t_mag) * ev.phi_wb,
            leakage_j: self.n_l * 0.5 * self.model.l_sigma_h * i * i,
            cogging_j: 0.0,
            lsm_normal_j: 0.0,
        }
    }

    fn lag_state(&self, h: f64) -> bool {
        let tau = self.p.design.suspension.tau_eddy_s;
        self.cfg.eddy_lag && tau > 0.0 && tau >= 2.0 * h
    }

    fn ctx(&self, u: &PlantInputs, h: f64) -> HCtx {
        let s = &self.p.design.suspension;
        let voltage = matches!(self.cfg.coil, CoilMode::Voltage);
        let v_bus = self.truth.power.v_bus_v;
        let i0 = self.y[H_I];
        let limit = self.i2t.limit_a(s.i_cont_a, s.i_pulse_a);
        let (prof, v) = if voltage {
            let v = if u.coil_enabled {
                fmath::clamp(self.mean_lift(&u.coil_v_or_iref), -v_bus, v_bus)
            } else {
                0.0
            };
            (LagProfile::hold(i0), v)
        } else if u.coil_enabled {
            (
                LagProfile {
                    i0_a: i0,
                    i_ref_a: self.mean_lift(&u.coil_v_or_iref),
                    tau_s: self.p.tau_cl_s,
                    slew_a_per_s: coil::slew_limit_a_per_s(
                        v_bus,
                        self.aux.r,
                        i0,
                        self.aux.e_mot,
                        self.aux.l_inc,
                    ),
                    limit_a: limit,
                },
                0.0,
            )
        } else {
            (
                LagProfile {
                    i0_a: i0,
                    i_ref_a: 0.0,
                    tau_s: if self.aux.r > 0.0 {
                        self.aux.l_inc / self.aux.r
                    } else {
                        0.0
                    },
                    slew_a_per_s: f64::INFINITY,
                    limit_a: limit,
                },
                0.0,
            )
        };
        HCtx {
            m: self.p.m_body_kg + u.payload_kg,
            h,
            voltage,
            lag_state: self.lag_state(h),
            prof,
            v,
            limit,
            mains_ok: u.mains_ok,
            t_amb_c: u.t_ambient_c,
        }
    }

    fn eval(&self, k: &HCtx, s: f64, a_floor: &[f64; 3], y: &HState, aux: &mut HAux) -> HState {
        let p = &self.p;
        let d = &p.design;
        let sus = &d.suspension;
        let mut dy = [0.0; H_N];
        let n = self.com.n;
        let g = sus.g_star_m + self.zr - y[H_Z];
        let g_dot = -y[H_ZD];
        let i = if k.voltage { y[H_I] } else { k.prof.at(s) };
        let t_mag = y[H_TMAG];
        let r = coil::r_at(sus.r_coil_ohm, sus.tempco_r_per_k, y[H_TCOIL]);
        let p_cu = r * i * i;
        // Thermal network: every lift coil carries the lumped current (PHYSICS §12).
        let mut ts = ThermalState::uniform(t_mag);
        let mut p_coil = [0.0; MAX_MAGNETS];
        for ((tc, pc), st) in ts
            .t_coil_c
            .iter_mut()
            .zip(p_coil.iter_mut())
            .zip(self.com.st.iter())
            .take(n)
        {
            *tc = y[H_TCOIL];
            if st.lift {
                *pc = p_cu;
            }
        }
        let tr = thermal::thermal_rates(&d.thermal, &ts, &p_coil, k.t_amb_c, n);
        let lead = (0..n).find(|&kk| self.com.st[kk].lift).unwrap_or(0);
        dy[H_TCOIL] = tr.t_coil_c[lead];
        dy[H_TMAG] = tr.t_mag_c;
        // Magnet and contacts (PHYSICS §2, §8).
        let ev = self.model.eval(
            &ElementGeom {
                g_m: g,
                y_m: 0.0,
                e: self.e,
                de_dx_per_m: 0.0,
            },
            i,
            t_mag,
        );
        let phi_f = if k.lag_state { y[H_PHI] } else { ev.phi_wb };
        let l = ev.loads(phi_f);
        let x_open = self.x0 - d.rail.x_closed_m;
        let vc = vertical_contacts(p, &self.com, g, g_dot, x_open, 0.0, k.m / self.n_l, k.h);
        let f = self.n_l * (l.f_attract_n + vc.f_up) - k.m * G0_MPS2 - k.m * a_floor[2];
        let zdd = f / k.m;
        dy[H_Z] = y[H_ZD];
        dy[H_ZD] = zdd;
        // Coil electrics (PHYSICS §4).
        let l_inc = self.model.dpsi_di_h(&ev);
        let e_mot = self.model.turns * (ev.dphi_dg * g_dot + ev.dphi_dt * dy[H_TMAG]);
        let di = if k.voltage {
            coil::voltage_di_dt(k.v, r, i, e_mot, l_inc, k.limit)
        } else {
            k.prof.rate(s)
        };
        let p_in = if k.voltage {
            dy[H_I] = di;
            k.v * i
        } else {
            p_cu + i * (l_inc * di + e_mot)
        };
        dy[H_PHI] = if k.lag_state {
            (ev.phi_wb - y[H_PHI]) / sus.tau_eddy_s
        } else {
            0.0
        };
        let pf = power::power_flow(
            &d.power,
            y[H_VHOLD],
            self.n_l * p_in + d.power.p_electronics_w,
            k.mains_ok,
        );
        dy[H_VHOLD] = pf.dv_hold_dt_v_per_s;
        dy[H_LEDGER + L_CONTACT] = self.n_l * vc.absorbed_w;
        dy[H_LEDGER + L_COIL_CU] = self.n_l * p_cu;
        dy[H_LEDGER + L_ELEC_IN] = self.n_l * p_in;
        dy[H_LEDGER + L_FLOOR] = -k.m * a_floor[2] * y[H_ZD];
        *aux = HAux {
            g,
            i,
            phi: ev.phi_wb,
            l_inc,
            e_mot,
            r,
            zdd,
            contact: ContactForces {
                upper_n: self.n_l * vc.upper,
                rollers_n: self.n_l * vc.rollers,
                seat_n: self.n_l * vc.seat,
                ..ContactForces::default()
            },
            pf,
        };
        dy
    }

    fn fill_truth(&mut self, t_s: f64, floor: &[f64; 3]) {
        let aux = self.aux;
        let n = self.com.n;
        let m = self.mass();
        let y = &self.y;
        let t = &mut self.truth;
        t.t_s = t_s;
        t.x_m = self.x0;
        t.v_mps = 0.0;
        t.a_mps2 = 0.0;
        t.pose = [self.x0, 0.0, y[H_Z], 1.0, 0.0, 0.0, 0.0];
        t.omega_radps = [0.0; 3];
        t.acc_abs_mps2 = [floor[0], floor[1], aux.zdd + floor[2]];
        t.gaps_m = [0.0; MAX_MAGNETS];
        t.coil_i_a = [0.0; MAX_MAGNETS];
        for k in 0..n {
            t.gaps_m[k] = aux.g;
            t.coil_i_a[k] = aux.i;
        }
        t.f_contact = aux.contact;
        t.e_diss = ledger_from(y, H_LEDGER, 0.5 * m * y[H_ZD] * y[H_ZD], 0, 0.0);
        t.floor_acc_mps2 = *floor;
        t.temps_c = [y[H_TCOIL], y[H_TMAG]];
        t.power = power_truth(&self.p, &aux.pf, y[H_VHOLD], t.power.mains_ok);
        let ct = aux.contact;
        t.state = ContactState {
            levitated: ct.rollers_n <= 0.0 && ct.seat_n <= 0.0 && ct.upper_n <= 0.0,
            seated: ct.seat_n > 0.0,
            landed: ct.rollers_n > 0.0,
            upper_stop: ct.upper_n > 0.0,
            locked: t.state.locked,
            latched: false,
        };
        t.mass_kg = m;
        t.payload_kg = self.payload_kg;
    }
}

impl PlantModel for Heave1 {
    fn tier(&self) -> Tier {
        Tier::Heave1
    }

    fn reset(&mut self, init: &InitialState) {
        let p = &self.p;
        let d = &p.design;
        self.truth = Truth::initial(p, init);
        self.payload_kg = init.payload_kg;
        self.x0 = init.x_m;
        self.e = self.com.mean_lift_engagement(p, init.x_m);
        self.zr = if self.com.n_lift > 0 {
            (0..self.com.n)
                .filter(|&k| self.com.st[k].lift)
                .map(|k| rail_at(p, init.x_m + self.com.st[k].r[0]).z_m)
                .sum::<f64>()
                / self.com.n_lift as f64
        } else {
            0.0
        };
        self.i2t = I2tBudget::default();
        let m = self.mass();
        let gap = rest_gap(
            p,
            &self.com,
            init.lev,
            m,
            0.0,
            init.x_m,
            init.temp_c,
            self.h_last,
        );
        self.y = [0.0; H_N];
        self.y[H_Z] = d.suspension.g_star_m + self.zr - gap;
        self.y[H_TCOIL] = init.temp_c;
        self.y[H_TMAG] = init.temp_c;
        self.y[H_VHOLD] = d.power.supercap_v_max_v;
        let ev = self.model.eval(
            &ElementGeom {
                g_m: gap,
                y_m: 0.0,
                e: self.e,
                de_dx_per_m: 0.0,
            },
            0.0,
            init.temp_c,
        );
        self.y[H_PHI] = ev.phi_wb;
        self.aux = HAux {
            l_inc: self.model.dpsi_di_h(&ev),
            r: coil::r_at(
                d.suspension.r_coil_ohm,
                d.suspension.tempco_r_per_k,
                init.temp_c,
            ),
            ..HAux::default()
        };
        self.truth.power.mains_ok = true;
        self.truth.state.locked = init.locked && d.landing.parking_lock;
        let u = PlantInputs {
            payload_kg: init.payload_kg,
            t_ambient_c: init.temp_c,
            ..PlantInputs::default()
        };
        let ctx = self.ctx(&u, self.h_last);
        let mut aux = self.aux;
        let _ = self.eval(&ctx, 0.0, &[0.0; 3], &self.y, &mut aux);
        self.aux = aux;
        self.fill_truth(0.0, &[0.0; 3]);
    }

    fn step(&mut self, t: f64, h: f64, u: &PlantInputs, floor_mps2: &[[f64; 3]; 3]) {
        self.h_last = h;
        self.payload_kg = u.payload_kg;
        self.truth.power.mains_ok = u.mains_ok;
        self.truth.state.locked = u.parking_lock && self.p.design.landing.parking_lock;
        let ctx = self.ctx(u, h);
        let y0 = self.y;
        let mut scratch = HAux::default();
        let mut y1 = rk4_step(&y0, t, h, |stage, _t, y| {
            let s = [0.0, 0.5 * h, 0.5 * h, h][stage];
            self.eval(
                &ctx,
                s,
                &floor_mps2[STAGE_FLOOR_INDEX[stage]],
                y,
                &mut scratch,
            )
        });
        if !ctx.voltage {
            y1[H_I] = ctx.prof.at(h);
        }
        self.y = y1;
        let mut aux = HAux::default();
        let _ = self.eval(&ctx, h, &floor_mps2[2], &self.y, &mut aux);
        if !ctx.lag_state {
            self.y[H_PHI] = aux.phi;
        }
        let s = &self.p.design.suspension;
        self.i2t
            .update(self.y[H_I], h, s.i_cont_a, s.i_pulse_a, s.t_pulse_s);
        self.aux = aux;
        self.fill_truth(t + h, &floor_mps2[2]);
    }

    fn truth(&self) -> &Truth {
        &self.truth
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::{Environment, Mismatch};
    use fabrail_params::{derive, presets};

    #[test]
    fn open_loop_pole_fit() {
        // Physics perspective A1: from g* + 1 nm with the coils held at 0 A, ln|δ| grows at
        // p = √(4g₀/(2g* + h′)) (69.49 / 47.57 / 32.50 rad/s), fitted over three e-folds
        // (0.5 %, I + linearisation; PHYSICS §3).
        for sp in [presets::drawer(), presets::door(), presets::wall()] {
            let p =
                PlantParams::from_design(&sp.design, &Mismatch::default(), &Environment::default());
            let cfg = ModelConfig {
                tier: Tier::Heave1,
                eddy_lag: false,
                ..ModelConfig::default()
            };
            let mut m = Heave1::new(&p, &cfg);
            // Mid travel (outside the DRAWER seat slot), then 1 nm below g*.
            m.reset(&InitialState {
                x_m: 0.5 * sp.design.rail.travel_m,
                ..InitialState::default()
            });
            m.y[H_Z] -= 1e-9;
            let s = &sp.design.suspension;
            let pole = derive::open_loop_pole(s.g_star_m, derive::h_prime(s.h_m_m, s.mu_rec));
            let h = Tier::Heave1.nominal_h_s();
            let u = PlantInputs {
                coil_enabled: true,
                ..PlantInputs::default()
            };
            let dev = |m: &Heave1| (m.y[H_Z]).abs();
            let mut k = 0u32;
            while dev(&m) < 20.0e-9 {
                m.step(k as f64 * h, h, &u, &[[0.0; 3]; 3]);
                k += 1;
            }
            let (t0, d0) = (k as f64 * h, dev(&m));
            while dev(&m) < d0 * (3.0f64).exp() {
                m.step(k as f64 * h, h, &u, &[[0.0; 3]; 3]);
                k += 1;
            }
            let rate = (dev(&m) / d0).ln() / (k as f64 * h - t0);
            assert!(
                (rate - pole).abs() <= 0.005 * pole,
                "fitted p {rate} vs {pole} rad/s (0.5 %, physics perspective A1)"
            );
        }
    }
}
