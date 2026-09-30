//! Travel1d tier: x, x', bristle z, lock, levitated/landed with kinematic heave emulation for LAND (PHYSICS §5). Owner: A6a.
//!
//! Levitation is assumed ideal (the gap stays at g* while levitated). With
//! `ModelConfig::kinematic_heave` (plan §2b item 16) the gap follows
//! [`crate::model::HeaveHint`] kinematically: `Land` ramps it to g_land at the class land
//! speed (core design §C.4: 5 / 5 / 3 / 2 mm/s), `Seat` to g_seat, `Lift` back to g*
//! within the class lift time, `Drop` falls to the rollers; touchdown sets the landed
//! flag and engages roller friction, so LAND/SEAT sequences complete in seismic Monte
//! Carlo runs. Without it the flag flips instantly. RK4 at 1 ms; the SIL harness still
//! runs the full-rate executive at the 20 kHz base tick and holds the actuation over the
//! 20 ticks of each plant step (W2 lead decision D8: SIL fidelity over speed; ≈ 130×
//! real time on WALL, budget ≥ 100×).
//!
//! As implemented (PHYSICS §5): x, ẋ, the DRAWER cavity ΔP, the hold-up voltage and the
//! ledger are RK4 states; the four bristle states (rollers, upper, seat, latch) are held
//! through the RK4 step and advanced exactly afterwards (`body::advance_bristles`,
//! PHYSICS §24); the gap and the vertical flags are discrete and move once per step. Loads along x share Carriage6's
//! assembly (friction, drag, bumpers, lock, latch, obstacle, cavity, lean, pressure,
//! hand, LSM, guide fin friction at y = 0) plus the inertial load −m·a_floor,x; the
//! magnets' engagement pull along x is omitted (levitation is ideal here). Resting
//! normals are PM-relieved (core design F8): N = m·(g₀ + a_floor,z) − Σ F_pm(gap) − F_n,LSM
//! (seat: |N|, captive both ways; rollers: max(N, 0), plus the ramp's x push N·slope).
//! [`host_reaction_x_n`] is the hook the Structure tier uses to load the device storey.

use fabrail_params::LandingKind;
use fabrail_params::derive::G0_MPS2;
use fabrail_types::{LsmMode, MAX_MAGNETS, fmath};

use crate::body::{
    Common, L_AIR, L_BUMPER, L_CONTACT, L_DIST, L_ELEC_IN, L_END_ZONE, L_FLOOR, L_FRICTION, L_HAND,
    L_LSM_CU, L_RAIL_EDDY, L_RAIL_HYST, N_BRISTLE, N_LEDGER, XIn, advance_bristles, hand_force,
    ledger_from, lsm_profiles, power_truth, support_gap, x_loads,
};
use crate::coil::LagProfile;
use crate::contact;
use crate::guidance;
use crate::integrator::{STAGE_FLOOR_INDEX, rk4_step};
use crate::lsm::{self, LsmForces};
use crate::magnet::ElementGeom;
use crate::model::{
    ContactForces, ContactState, EnergyLedger, HeaveHint, InitLev, InitialState, ModelConfig,
    PlantInputs, PlantModel, Tier, Truth,
};
use crate::params::PlantParams;
use crate::power::{self, PowerFlow};

/// Kinematic drop speed toward the rollers after a control loss, m/s. Design choice: the
/// order of the LEV-019 impact speeds (50–200 mm/s; physics perspective A7; PHYSICS §5,
/// §15).
pub const DROP_SPEED_MPS: f64 = 0.1;

/// Lift-off threshold: the support releases once the gap has closed this far, m
/// (core design F17: lift-off sensed at > 20 µm; PHYSICS §15).
const LIFTOFF_M: f64 = 20.0e-6;

const T_X: usize = 0;
const T_V: usize = 1;
const T_Z: usize = 2;
const T_DP: usize = T_Z + N_BRISTLE;
const T_VHOLD: usize = T_DP + 1;
const T_LEDGER: usize = T_VHOLD + 1;
const T_N: usize = T_LEDGER + N_LEDGER;

type TState = [f64; T_N];

/// Force the device exerts on its host (rail / storey) along x, N: the reaction to every
/// interaction force on the body, −m·(a_rel + a_floor,x) (Newton's third law; the Structure
/// tier's coupling hook, PHYSICS §21).
pub fn host_reaction_x_n(t: &Truth) -> f64 {
    -t.mass_kg * (t.a_mps2 + t.floor_acc_mps2[0])
}

/// Vertical kinematic state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Vert {
    gap: f64,
    levitated: bool,
    resting: bool,
}

/// Step-constant context.
#[derive(Clone, Copy, Debug)]
struct TCtx {
    u: PlantInputs,
    m: f64,
    h: f64,
    resting: bool,
    f_pm: f64,
    f_n_lsm: f64,
    flux_ratio: f64,
    lsm_q: LagProfile,
    lsm_d: LagProfile,
    hand_anchor: f64,
    lock: Option<f64>,
    latched: bool,
}

/// Per-evaluation outputs.
#[derive(Clone, Copy, Debug, Default)]
struct TAux {
    a: f64,
    lsm: LsmForces,
    f_hand: f64,
    contact: ContactForces,
    pf: PowerFlow,
}

/// Travel-axis tier for seismic Monte Carlo, end zones and EEW timelines.
#[derive(Clone, Debug)]
pub struct Travel1d {
    p: PlantParams,
    cfg: ModelConfig,
    com: Common,
    y: TState,
    vert: Vert,
    aux: TAux,
    truth: Truth,
    lsm_iq: f64,
    lsm_id: f64,
    lock_anchor: Option<f64>,
    latched: bool,
    hand_anchor: Option<f64>,
    bumper_log: contact::BumperLog,
    bumper_counts: EnergyLedger,
    payload_kg: f64,
    h_last: f64,
}

impl Travel1d {
    /// A Travel1d model over `p`, reset to the default initial state.
    pub fn new(p: &PlantParams, cfg: &ModelConfig) -> Travel1d {
        let mut t = Travel1d {
            p: p.clone(),
            cfg: *cfg,
            com: Common::new(p, Tier::Travel1d.nominal_h_s()),
            y: [0.0; T_N],
            vert: Vert::default(),
            aux: TAux::default(),
            truth: Truth::initial(p, &InitialState::default()),
            lsm_iq: 0.0,
            lsm_id: 0.0,
            lock_anchor: None,
            latched: false,
            hand_anchor: None,
            bumper_log: contact::BumperLog::default(),
            bumper_counts: EnergyLedger::default(),
            payload_kg: 0.0,
            h_last: Tier::Travel1d.nominal_h_s(),
        };
        t.reset(&InitialState::default());
        t
    }

    /// True when heave is emulated kinematically (plan §2b item 16).
    pub fn kinematic_heave(&self) -> bool {
        self.cfg.kinematic_heave
    }

    fn mass(&self) -> f64 {
        self.p.m_body_kg + self.payload_kg
    }

    /// The support face under the carriage at `x_m` (seat within the DRAWER slot, the
    /// ramp, else the rollers).
    fn support(&self, x_m: f64, seat: bool) -> f64 {
        let lev = if seat {
            InitLev::Seated
        } else {
            InitLev::Landed
        };
        support_gap(&self.p, lev, x_m - self.p.design.rail.x_closed_m)
    }

    /// Kinematic heave for one step (plan §2b item 16).
    fn heave(&mut self, hint: HeaveHint, h: f64) {
        let s = &self.p.design.suspension;
        let x = self.y[T_X];
        let rest = self.support(x, false);
        let (target, rate) = match hint {
            HeaveHint::Hold => {
                if self.vert.resting {
                    // Resting bodies ride the support (the DRAWER ramp) as x changes.
                    self.vert.gap = rest;
                }
                return;
            }
            HeaveHint::Lift => {
                let span = fmath::abs(rest - s.g_star_m);
                let t_lift = self.p.lift_time_s;
                (
                    s.g_star_m,
                    if t_lift > 0.0 {
                        span / t_lift
                    } else {
                        f64::INFINITY
                    },
                )
            }
            HeaveHint::Land => (rest, self.p.land_speed_mps),
            HeaveHint::Seat => (self.support(x, true), self.p.land_speed_mps),
            HeaveHint::Drop => (rest, DROP_SPEED_MPS),
        };
        let g0 = self.vert.gap;
        let g1 = if self.cfg.kinematic_heave && rate.is_finite() {
            let step = rate * h;
            g0 + fmath::clamp(target - g0, -step, step)
        } else {
            target
        };
        self.vert.gap = g1;
        let at_target = fmath::abs(g1 - target) <= 1e-12;
        match hint {
            HeaveHint::Lift => {
                if self.vert.resting && g1 < rest - LIFTOFF_M {
                    self.vert.resting = false;
                }
                if !self.cfg.kinematic_heave {
                    self.vert.resting = false;
                }
                self.vert.levitated = at_target;
            }
            _ => {
                self.vert.levitated = false;
                self.vert.resting = at_target;
            }
        }
    }

    /// PM force of the lift elements at the present gap and its flux ratio.
    fn magnets(&self) -> (f64, f64) {
        let t_mag = self.truth.temps_c[1];
        let mut f = 0.0;
        let mut phi_sq = 0.0;
        for k in 0..self.com.n {
            let st = &self.com.st[k];
            if !st.lift {
                continue;
            }
            let ev = st.model.eval(
                &ElementGeom {
                    g_m: self.vert.gap,
                    y_m: 0.0,
                    e: 1.0,
                    de_dx_per_m: 0.0,
                },
                0.0,
                t_mag,
            );
            f += ev.loads(ev.phi_wb).f_attract_n;
            phi_sq += ev.phi_wb * ev.phi_wb;
        }
        let ratio = if self.com.phi0_sq_sum > 0.0 {
            phi_sq / self.com.phi0_sq_sum
        } else {
            1.0
        };
        (f, ratio)
    }

    fn ctx(&self, u: &PlantInputs, h: f64) -> TCtx {
        let (f_pm, flux_ratio) = self.magnets();
        let (lsm_q, lsm_d) = lsm_profiles(&self.p, u, self.lsm_iq, self.lsm_id);
        let f_n_lsm =
            lsm::thrust(&self.p.design.lsm, self.y[T_X], 0.0, 0.0, 0.0, LsmMode::Sto).fn_n;
        TCtx {
            u: *u,
            m: self.p.m_body_kg + u.payload_kg,
            h,
            resting: self.vert.resting,
            f_pm,
            f_n_lsm,
            flux_ratio,
            lsm_q,
            lsm_d,
            hand_anchor: self.hand_anchor.unwrap_or(self.y[T_X]),
            lock: self.lock_anchor,
            latched: self.latched,
        }
    }

    fn eval(&self, k: &TCtx, s: f64, a_floor: &[f64; 3], y: &TState, aux: &mut TAux) -> TState {
        let p = &self.p;
        let d = &p.design;
        let mut dy = [0.0; T_N];
        let (x, v, m) = (y[T_X], y[T_V], k.m);
        let x_open = x - d.rail.x_closed_m;
        // Resting normals, PM-relieved (core design F8; PHYSICS §8).
        let mut normals = [0.0; 3];
        let mut fx_ramp = 0.0;
        let seat_kind = matches!(d.landing.kind, LandingKind::Seat) && d.suspension.g_seat_m > 0.0;
        if k.resting {
            let n_net = m * (G0_MPS2 + a_floor[2]) - k.f_pm - k.f_n_lsm;
            if seat_kind && x_open <= d.landing.slot_len_m {
                normals[2] = fmath::abs(n_net);
            } else {
                normals[0] = fmath::fmax(n_net, 0.0);
                let ld = &d.landing;
                if seat_kind && ld.ramp_len_m > 0.0 && x_open <= ld.slot_len_m + ld.ramp_len_m {
                    fx_ramp = normals[0] * ld.ramp_rise_m / ld.ramp_len_m;
                }
            }
        }
        let xo = x_loads(
            p,
            &self.com,
            &k.u,
            &XIn {
                x,
                v,
                m,
                h: k.h,
                z: [y[T_Z], y[T_Z + 1], y[T_Z + 2], y[T_Z + 3]],
                normals,
                seated: k.resting && seat_kind && x_open <= d.landing.slot_len_m,
                dp_pa: y[T_DP],
                flux_ratio: k.flux_ratio,
                lock: k.lock,
                latched: k.latched,
            },
        );
        let lf = lsm::thrust(&d.lsm, x, v, k.lsm_q.at(s), k.lsm_d.at(s), k.u.lsm_mode);
        let gf = guidance::guide(&d.guidance, 0.0, 0.0, v);
        let f_floor = -m * a_floor[0];
        let mut f = xo.fx + lf.fx_n + gf.fx_n + fx_ramp + f_floor;
        let f_hand = match k.u.f_ext.hand {
            Some(hi) => hand_force(&hi, x - k.hand_anchor, v, m, f),
            None => 0.0,
        };
        f += f_hand;
        let a = f / m;
        dy[T_X] = v;
        dy[T_V] = a;
        // Bristle states are held through the RK4 step (zero rate here) and advanced
        // exactly afterwards (`body::advance_bristles`; PHYSICS §24).
        dy[T_DP] = xo.ddp;
        let pf = power::power_flow(
            &d.power,
            y[T_VHOLD],
            lf.p_elec_w + d.power.p_electronics_w,
            k.u.mains_ok,
        );
        dy[T_VHOLD] = pf.dv_hold_dt_v_per_s;
        let lg = T_LEDGER;
        dy[lg + L_DIST] = -xo.drag.dist_n * v;
        dy[lg + L_END_ZONE] = -xo.drag.end_zone_n * v;
        dy[lg + L_RAIL_EDDY] = -xo.drag.rail_eddy_n * v;
        dy[lg + L_RAIL_HYST] = -xo.drag.rail_hyst_n * v;
        dy[lg + L_AIR] = -xo.drag.air_n * v + xo.air_w;
        dy[lg + L_FRICTION] = xo.friction_w - gf.fx_n * v;
        dy[lg + L_CONTACT] = xo.contact_w - fx_ramp * v;
        dy[lg + L_BUMPER] = xo.bumper_w;
        dy[lg + L_LSM_CU] = lf.p_copper_w;
        dy[lg + L_ELEC_IN] = lf.p_elec_w;
        dy[lg + L_HAND] = f_hand * v;
        dy[lg + L_FLOOR] = f_floor * v;
        let fin = d.guidance.fin_present;
        *aux = TAux {
            a,
            lsm: lf,
            f_hand,
            contact: ContactForces {
                upper_n: 0.0,
                rollers_n: normals[0],
                seat_n: normals[2],
                guide_fin_n: if fin { gf.normal_n } else { 0.0 },
                pads_n: if fin { 0.0 } else { gf.normal_n },
                bumper_n: xo.bumper_n,
                end_stop_n: xo.end_stop_n,
                lock_n: xo.lock_n,
                latch_n: xo.latch_n,
                obstacle_n: xo.obstacle_n,
                tangential_x_n: xo.f_t[0] + xo.f_t[1] + xo.f_t[2] + gf.fx_n,
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
        let g_star = self.p.design.suspension.g_star_m;
        let x_open = y[T_X] - self.p.design.rail.x_closed_m;
        let seat_kind = matches!(self.p.design.landing.kind, LandingKind::Seat)
            && self.p.design.suspension.g_seat_m > 0.0;
        let in_seat = seat_kind && x_open <= self.p.design.landing.slot_len_m;
        let t = &mut self.truth;
        t.t_s = t_s;
        t.x_m = y[T_X];
        t.v_mps = y[T_V];
        t.a_mps2 = aux.a;
        t.pose = [y[T_X], 0.0, g_star - self.vert.gap, 1.0, 0.0, 0.0, 0.0];
        t.omega_radps = [0.0; 3];
        t.acc_abs_mps2 = [aux.a + floor[0], floor[1], floor[2]];
        t.gaps_m = [0.0; MAX_MAGNETS];
        for g in t.gaps_m.iter_mut().take(n) {
            *g = self.vert.gap;
        }
        t.coil_i_a = [0.0; MAX_MAGNETS];
        t.iq_a = aux.lsm.iq_a;
        t.id_a = aux.lsm.id_a;
        t.i_abc_a = lsm::dq_to_abc(&self.p.design.lsm, y[T_X], aux.lsm.iq_a, aux.lsm.id_a);
        t.f_lsm_n = aux.lsm.fx_n;
        t.f_contact = aux.contact;
        t.f_hand_n = aux.f_hand;
        t.e_diss = ledger_from(
            y,
            T_LEDGER,
            0.5 * m * y[T_V] * y[T_V],
            self.bumper_counts.bumper_impacts,
            self.bumper_counts.bumper_e_max_j,
        );
        t.floor_acc_mps2 = *floor;
        t.cavity_dp_pa = y[T_DP];
        t.power = power_truth(&self.p, &aux.pf, y[T_VHOLD], t.power.mains_ok);
        t.state = ContactState {
            levitated: self.vert.levitated,
            seated: self.vert.resting && in_seat,
            landed: self.vert.resting && !in_seat,
            upper_stop: false,
            locked: self.lock_anchor.is_some(),
            latched: self.latched,
        };
        t.mass_kg = m;
        t.payload_kg = self.payload_kg;
    }
}

impl PlantModel for Travel1d {
    fn tier(&self) -> Tier {
        Tier::Travel1d
    }

    fn reset(&mut self, init: &InitialState) {
        let d = &self.p.design;
        self.truth = Truth::initial(&self.p, init);
        self.payload_kg = init.payload_kg;
        self.y = [0.0; T_N];
        self.y[T_X] = init.x_m;
        self.y[T_V] = init.v_mps;
        self.y[T_VHOLD] = d.power.supercap_v_max_v;
        let x_open = init.x_m - d.rail.x_closed_m;
        self.vert = Vert {
            gap: support_gap(&self.p, init.lev, x_open),
            levitated: matches!(init.lev, InitLev::Levitated),
            resting: !matches!(init.lev, InitLev::Levitated),
        };
        self.lsm_iq = 0.0;
        self.lsm_id = 0.0;
        self.hand_anchor = None;
        self.latched = false;
        self.lock_anchor = if init.locked && d.landing.parking_lock {
            Some(init.x_m)
        } else {
            None
        };
        self.bumper_log = contact::BumperLog::default();
        self.bumper_counts = EnergyLedger::default();
        self.truth.power.mains_ok = true;
        let u = PlantInputs {
            payload_kg: init.payload_kg,
            t_ambient_c: init.temp_c,
            ..PlantInputs::default()
        };
        let ctx = self.ctx(&u, self.h_last);
        let mut aux = TAux::default();
        let _ = self.eval(&ctx, 0.0, &[0.0; 3], &self.y, &mut aux);
        self.aux = aux;
        self.fill_truth(0.0, &[0.0; 3]);
    }

    fn step(&mut self, t: f64, h: f64, u: &PlantInputs, floor_mps2: &[[f64; 3]; 3]) {
        self.h_last = h;
        self.payload_kg = u.payload_kg;
        let d = &self.p.design;
        let x0 = self.y[T_X];
        self.hand_anchor = match (u.f_ext.hand, self.hand_anchor) {
            (Some(_), Some(a)) => Some(a),
            (Some(_), None) => Some(x0),
            (None, _) => None,
        };
        self.lock_anchor = match (u.parking_lock && d.landing.parking_lock, self.lock_anchor) {
            (true, Some(a)) => Some(a),
            (true, None) => Some(x0),
            (false, _) => None,
        };
        let latch_now = u.seismic_latch && d.landing.seismic_latch.enabled;
        if latch_now && !self.latched {
            // A latch engages undeflected.
            self.y[T_Z + 3] = 0.0;
        }
        self.latched = latch_now;
        self.truth.power.mains_ok = u.mains_ok;
        self.heave(u.heave_hint, h);
        let ctx = self.ctx(u, h);
        let y0 = self.y;
        let mut scratch = TAux::default();
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
        let dx = y1[T_X] - y0[T_X];
        advance_bristles(
            &self.p,
            &self.com,
            &mut y1[T_Z..T_Z + N_BRISTLE],
            dx,
            h,
            ctx.latched,
        );
        self.y = y1;
        let mut aux = TAux::default();
        let _ = self.eval(&ctx, h, &floor_mps2[2], &self.y, &mut aux);
        match u.lsm_mode {
            LsmMode::Asc => {
                self.lsm_iq = aux.lsm.iq_a;
                self.lsm_id = aux.lsm.id_a;
            }
            _ => {
                self.lsm_iq = ctx.lsm_q.at(h);
                self.lsm_id = ctx.lsm_d.at(h);
            }
        }
        let delta = contact::bumper_penetration(&self.p.design, self.y[T_X]).0;
        let (pp, v, m) = (&self.p, self.y[T_V], ctx.m);
        self.bumper_log
            .update(pp, &mut self.bumper_counts, delta, v, m);
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
    use fabrail_params::presets;

    fn wall() -> Travel1d {
        let sp = presets::wall();
        let p = PlantParams::from_design(&sp.design, &Mismatch::default(), &Environment::default());
        Travel1d::new(
            &p,
            &ModelConfig {
                tier: Tier::Travel1d,
                ..ModelConfig::default()
            },
        )
    }

    #[test]
    fn kinematic_land_completes_at_the_class_speed() {
        // WALL lands at 3 mm/s (core design §C.4): 2.5 mm from g* to g_land in ≈ 0.83 s,
        // then the landed flag is set and the rollers carry the PM-relieved weight
        // (plan §2b item 16; F8).
        let mut m = wall();
        m.reset(&InitialState {
            x_m: 3.0,
            ..InitialState::default()
        });
        assert!(m.truth().state.levitated);
        let h = Tier::Travel1d.nominal_h_s();
        let u = PlantInputs {
            heave_hint: HeaveHint::Land,
            ..PlantInputs::default()
        };
        let mut k = 0u32;
        while !m.truth().state.landed {
            m.step(k as f64 * h, h, &u, &[[0.0; 3]; 3]);
            k += 1;
            assert!(k < 2000, "landing must complete");
        }
        let t_land = k as f64 * h;
        assert!(
            (t_land - 2.5e-3 / 3.0e-3).abs() <= 2.0 * h,
            "t_land {t_land} (core §C.4)"
        );
        let s = presets::wall().design.suspension;
        assert!((m.truth().gaps_m[0] - s.g_land_m).abs() < 1e-12);
        // F18 stay-landed ratio 0.863 at m_op_min → 0.777 of the weight at m* is relieved.
        let w = 500.0 * G0_MPS2;
        let n = m.truth().f_contact.rollers_n;
        assert!(
            n > 0.0 && n < 0.3 * w,
            "roller load {n} is PM-relieved (F8)"
        );
        // Lift returns to hover.
        let u = PlantInputs {
            heave_hint: HeaveHint::Lift,
            ..u
        };
        for _ in 0..1100 {
            m.step(k as f64 * h, h, &u, &[[0.0; 3]; 3]);
            k += 1;
        }
        assert!(m.truth().state.levitated && (m.truth().gaps_m[0] - s.g_star_m).abs() < 1e-12);
    }

    #[test]
    fn free_mass_follows_the_floor_inertially() {
        // Levitated with the LSM off, the body is a free mass along x: its relative motion
        // is −∬a_floor minus the (small) drag (PHYSICS §5, §21).
        let mut m = wall();
        m.reset(&InitialState {
            x_m: 3.0,
            ..InitialState::default()
        });
        let h = Tier::Travel1d.nominal_h_s();
        let floor = [[0.5, 0.0, 0.0]; 3];
        for k in 0..500 {
            m.step(k as f64 * h, h, &PlantInputs::default(), &floor);
        }
        let t = m.truth();
        let free = -0.5 * 0.5 * 0.5 * 0.5; // −½·a·t² at t = 0.5 s
        assert!(t.x_m - 3.0 <= 0.0 && (t.x_m - 3.0 - free).abs() <= 0.1 * free.abs());
        assert!((host_reaction_x_n(t) + 500.0 * (t.a_mps2 + 0.5)).abs() < 1e-9);
    }
}
