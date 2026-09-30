//! Intent confirmation (force x time or displacement) with direction consistency; lean and pressure rejection; push-to-open (PHYSICS §17). Owner: A2.
//!
//! Frozen API (W0): [`IntentDetector::new`], [`IntentDetector::step`] → `IntentFlag`.
//! Confirmation by force × time (4.5 N × 50 ms, 10 × 80, 25 × 120, 40 × 150 ms) or
//! displacement (1.5 / 2 / 3 mm), with direction consistency (cabinet pressure pushes
//! outward; plan §2b item 11); lean rise ≥ 1 s and smoke ramp ≥ 2 s are rejected;
//! latency: onset ≤ 10/20 ms, confirmation ≤ 120/170 ms, DRAWER first motion ≤ 160 ms
//! (F12). Disabled in SEISMIC (plan §4b). Test: `lean_rejected`.
//!
//! Detector (PHYSICS §17; core design §C.4; plan §2b item 11), per fast release:
//!
//! 1. **Pressure high-pass.** f_hp = F̂ − LP(F̂) at `haptic.f_hp_hz` (0.5 Hz, exact ZOH
//!    pole). Slow disturbances are attenuated: a 15 N lean rising over 1 s peaks at
//!    ≈ 4.6 N after it, a 50 N smoke ΔP ramped over 2 s at ≈ 8.0 N.
//! 2. **Onset.** A gesture starts when |f_hp| > f_on = F_th/4 (design choice), with its
//!    direction s = sign(f_hp) and the displacement latched.
//! 3. **Direction consistency** (`intent.dir_consistency`). At the DRAWER seat with
//!    push-to-open (`seat_mode`) the only consistent gesture is an inward push (−x),
//!    which means *open*; an outward force there is cabinet pressure (the piston effect
//!    pushes outward) and is rejected. Everywhere, a gesture whose carriage moves against
//!    the push by more than max(dx_th/4, 0.1 mm), or whose force reverses
//!    (s·f_hp < −f_on), is rejected.
//! 4. **Confirmation.** Either the displacement s·(x − x_onset) ≥ dx_th (classes with
//!    dx_th > 0) within the confirmation window (`intent.confirm_max_s`, 120 / 170 ms), or s·f_hp ≥ F_th for
//!    t_th continuously, where F_th must be reached within the window (a slow rise is a
//!    lean or pressure, not a push). Confirmed intent is held for ≥ 20 ms (two
//!    supervisor periods and margin; design choice) and while the push lasts.
//! 5. **Rejection** (window expired without the force reaching F_th, reversal,
//!    inconsistency, or a gesture that ends unconfirmed) and the end of a confirmed push
//!    enter a refractory state that re-arms only when |f_hp| < f_on and |F̂| < F_th.
//!    A release makes the high-pass swing the other way by at most its low-pass value
//!    at release; where that could reach F_th, |F̂| is still ≥ F_th at the swing's zero
//!    crossing, so the release never starts a new gesture.
//! 6. **Lean.** |F̂| > F_th with |v| < `lean_v_mps` (2 mm/s) for `lean_t_s` (1 s) is
//!    a lean: the flag is `Lean` (the caller feeds the observer's bias estimator) and no
//!    gesture arms until it ends and the high-pass settles.
//!
//! Intent is reported only while `enabled` (the supervisor clears it outside SeismicState
//! NORMAL, plan §4b). Timers count integer fast releases (deterministic).

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::fmath;

use crate::msg::IntentFlag;

/// Onset threshold as a fraction of F_th (design choice).
const ONSET_FRAC: f32 = 0.25;
/// Tolerated displacement against the push as a fraction of dx_th (design choice) …
const AGAINST_FRAC: f32 = 0.25;
/// … but at least this much (encoder noise; HEAVY has no displacement path), m.
const AGAINST_MIN_M: f32 = 0.1e-3;
/// Minimum time a confirmed intent is reported, s (design choice: ≥ 2 supervisor
/// periods at 1 kHz with margin).
const CONFIRM_HOLD_S: f64 = 0.02;

/// Gesture state machine.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum IntentPhase {
    /// Waiting for an onset.
    #[default]
    Idle,
    /// Onset seen; confirmation pending.
    Gesture,
    /// Intent confirmed.
    Confirmed,
    /// Rejected or released; waiting for |f_hp| < f_on.
    Refractory,
}

/// Intent detector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct IntentDetector {
    /// Time above threshold, s.
    pub t_above_s: f32,
    /// Displacement since onset along the gesture direction, m.
    pub dx_m: f32,
    /// Last flag.
    pub flag: IntentFlag,
    /// Force threshold F_th, N.
    pub f_th_n: f32,
    /// Onset threshold f_on, N.
    pub f_on_n: f32,
    /// Displacement threshold dx_th, m.
    pub dx_th_m: f32,
    /// Lean speed limit, m/s.
    pub lean_v_mps: f32,
    /// Require direction consistency.
    pub dir_consistency: bool,
    /// Push-to-open available (parameter gate).
    pub push_to_open: bool,
    /// The carriage is at its seat with push-to-open active (set by the caller).
    pub seat_mode: bool,
    /// High-pass low-pass factor 1 − e^(−2π·f_hp·T).
    pub hp_k: f32,
    /// High-pass low-pass state, N.
    pub lp_n: f32,
    /// Last high-passed force, N.
    pub f_hp_n: f32,
    /// Gesture phase.
    pub phase: IntentPhase,
    /// Gesture direction sign (±1).
    pub dir: f32,
    /// Displacement at onset, m (caller's reference).
    pub dx0_m: f32,
    /// The force reached F_th within the window.
    pub reached: bool,
    /// Lean active.
    pub lean: bool,
    /// Force persistence t_th, releases.
    pub t_th_ticks: u32,
    /// Confirmation window, releases.
    pub window_ticks: u32,
    /// Lean time, releases.
    pub lean_ticks_min: u32,
    /// Confirmed-intent hold, releases.
    pub hold_ticks: u32,
    /// Releases since onset / confirmation.
    pub phase_ticks: u32,
    /// Releases above F_th (continuous).
    pub above_ticks: u32,
    /// Releases meeting the lean condition (continuous).
    pub lean_ticks: u32,
    /// Fast period, s.
    pub dt_s: f32,
}

/// `t/dt` in whole releases (≥ 1 when t > 0), saturating.
fn ticks(t_s: f64, dt_s: f64) -> u32 {
    if dt_s > 0.0 && t_s > 0.0 {
        let n = fmath::ceil(t_s / dt_s - 1e-9);
        if n >= f64::from(u32::MAX) {
            u32::MAX
        } else {
            (n as u32).max(1)
        }
    } else {
        0
    }
}

impl IntentDetector {
    /// Detector from the intent tuning (never fails).
    pub fn new(p: &SystemParams) -> Result<IntentDetector, ParamError> {
        let it = &p.control.intent;
        let dt = fabrail_params::derive::fast_period_s(p);
        let w = 2.0 * core::f64::consts::PI * p.control.haptic.f_hp_hz * dt;
        Ok(IntentDetector {
            f_th_n: it.f_th_n as f32,
            f_on_n: ONSET_FRAC * it.f_th_n as f32,
            dx_th_m: it.dx_th_m as f32,
            lean_v_mps: it.lean_v_mps as f32,
            dir_consistency: it.dir_consistency,
            push_to_open: it.push_to_open,
            // Exact ZOH pole of the first-order low-pass inside the high-pass.
            hp_k: if w > 0.0 {
                (1.0 - fmath::exp(-w)) as f32
            } else {
                0.0
            },
            dir: 1.0,
            t_th_ticks: ticks(it.t_th_s, dt),
            window_ticks: ticks(it.confirm_max_s, dt),
            lean_ticks_min: ticks(it.lean_t_s, dt),
            hold_ticks: ticks(CONFIRM_HOLD_S, dt),
            dt_s: dt as f32,
            ..IntentDetector::default()
        })
    }

    /// True while no gesture is pending, confirmed or being rejected.
    pub fn is_idle(&self) -> bool {
        self.phase == IntentPhase::Idle && !self.lean
    }

    /// Intent direction flag for a gesture of sign `dir` (push-to-open maps an inward
    /// push at the seat to *open*).
    fn map(&self, confirmed: bool) -> IntentFlag {
        let open = if self.seat_mode && self.push_to_open {
            self.dir < 0.0
        } else {
            self.dir > 0.0
        };
        match (open, confirmed) {
            (true, true) => IntentFlag::Open,
            (false, true) => IntentFlag::Close,
            (true, false) => IntentFlag::PendingOpen,
            (false, false) => IntentFlag::PendingClose,
        }
    }

    /// Enters the refractory state.
    fn reject(&mut self) {
        self.phase = IntentPhase::Refractory;
        self.above_ticks = 0;
        self.reached = false;
    }

    /// One fast release from F̂_h (N, the raw observer estimate), the displacement from
    /// the caller's rest reference (m; the detector latches it at onset) and the
    /// velocity (m/s). Returns the intent flag.
    pub fn step(&mut self, f_hand_n: f32, dx_m: f32, v_mps: f32, enabled: bool) -> IntentFlag {
        let finite = f_hand_n.is_finite() && dx_m.is_finite() && v_mps.is_finite();
        if !enabled || !finite {
            self.reset();
            return IntentFlag::None;
        }
        // 1. Pressure high-pass (0.5 Hz).
        self.lp_n += self.hp_k * (f_hand_n - self.lp_n);
        let f_hp = f_hand_n - self.lp_n;
        self.f_hp_n = f_hp;
        let f_on = self.f_on_n;
        // 6. Lean: sustained raw force without motion.
        if fmath::absf(f_hand_n) > self.f_th_n && fmath::absf(v_mps) < self.lean_v_mps {
            self.lean_ticks = self.lean_ticks.saturating_add(1);
        } else {
            self.lean_ticks = 0;
        }
        let was_lean = self.lean;
        self.lean = self.lean_ticks_min > 0 && self.lean_ticks >= self.lean_ticks_min;
        if self.lean || was_lean {
            // No gesture while leaning; after it, wait for the high-pass to settle.
            self.reject();
            self.flag = if self.lean {
                IntentFlag::Lean
            } else {
                IntentFlag::None
            };
            return self.flag;
        }
        match self.phase {
            IntentPhase::Idle => {
                if fmath::absf(f_hp) > f_on {
                    self.dir = if f_hp > 0.0 { 1.0 } else { -1.0 };
                    let at_seat = self.seat_mode && self.push_to_open;
                    if self.dir_consistency && at_seat && self.dir > 0.0 {
                        // Outward force at the seat: cabinet pressure, not a push.
                        self.reject();
                    } else {
                        self.phase = IntentPhase::Gesture;
                        self.phase_ticks = 0;
                        self.above_ticks = 0;
                        self.reached = false;
                        self.dx0_m = dx_m;
                        self.dx_m = 0.0;
                    }
                }
            }
            IntentPhase::Gesture => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                let s = self.dir;
                let disp = s * (dx_m - self.dx0_m);
                self.dx_m = disp;
                let push = s * f_hp;
                let tol = fmath::fmaxf(AGAINST_FRAC * self.dx_th_m, AGAINST_MIN_M);
                let against = self.dir_consistency && disp < -tol;
                // The displacement path exists only where the class defines one
                // (dx_th > 0; HEAVY confirms by force × time plus hold-to-run).
                let moved = self.dx_th_m > 0.0 && disp >= self.dx_th_m;
                if push < -f_on || against {
                    self.reject();
                } else if moved && self.phase_ticks <= self.window_ticks {
                    self.confirm();
                } else if push >= self.f_th_n {
                    if self.phase_ticks <= self.window_ticks {
                        self.reached = true;
                    }
                    if self.reached {
                        self.above_ticks = self.above_ticks.saturating_add(1);
                        if self.above_ticks >= self.t_th_ticks {
                            self.confirm();
                        }
                    } else {
                        // F_th reached only after the window: a slow rise.
                        self.reject();
                    }
                } else {
                    self.above_ticks = 0;
                    let expired = self.phase_ticks > self.window_ticks;
                    if push < f_on || expired {
                        // Ended unconfirmed, or a slow rise (lean / pressure ramp).
                        self.reject();
                    }
                }
            }
            IntentPhase::Confirmed => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if self.phase_ticks >= self.hold_ticks && self.dir * f_hp < f_on {
                    self.reject();
                }
            }
            IntentPhase::Refractory => {
                if fmath::absf(f_hp) < f_on && fmath::absf(f_hand_n) < self.f_th_n {
                    self.phase = IntentPhase::Idle;
                }
            }
        }
        self.t_above_s = self.above_ticks as f32 * self.dt_s;
        self.flag = match self.phase {
            IntentPhase::Idle | IntentPhase::Refractory => IntentFlag::None,
            IntentPhase::Gesture => self.map(false),
            IntentPhase::Confirmed => self.map(true),
        };
        self.flag
    }

    /// Enters the confirmed state.
    fn confirm(&mut self) {
        self.phase = IntentPhase::Confirmed;
        self.phase_ticks = 0;
    }

    /// Clears the detector (configuration and `seat_mode` are kept).
    pub fn reset(&mut self) {
        self.t_above_s = 0.0;
        self.dx_m = 0.0;
        self.flag = IntentFlag::None;
        self.lp_n = 0.0;
        self.f_hp_n = 0.0;
        self.phase = IntentPhase::Idle;
        self.dir = 1.0;
        self.dx0_m = 0.0;
        self.reached = false;
        self.lean = false;
        self.phase_ticks = 0;
        self.above_ticks = 0;
        self.lean_ticks = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    /// Kinematics seen by the detector for a force profile.
    #[derive(Clone, Copy)]
    enum Mount {
        /// Held by the position loop: no motion.
        Held,
        /// DRAWER seat push stop: dx = F/k_seat within [−compliance, 0], push-to-open on.
        Seat { k: f64, travel: f64 },
        /// Carriage moving at a constant speed (m/s).
        Moving(f64),
    }

    /// Runs `force(t)` for `secs`; returns every flag and the first confirmation time.
    fn run(
        p: &SystemParams,
        mount: Mount,
        secs: f64,
        force: impl Fn(f64) -> f64,
    ) -> (Vec<IntentFlag>, Option<f64>) {
        let mut d = IntentDetector::new(p).unwrap_or_default();
        d.seat_mode = matches!(mount, Mount::Seat { .. });
        let dt = f64::from(d.dt_s);
        let n = (secs / dt).round() as usize;
        let mut flags = Vec::with_capacity(n);
        let mut first = None;
        let mut dx_prev = 0.0;
        for k in 0..n {
            let t = k as f64 * dt;
            let f = force(t);
            let dx = match mount {
                Mount::Held => 0.0,
                Mount::Seat { k, travel } => (f / k).clamp(-travel, 0.0),
                Mount::Moving(v) => v * t,
            };
            let v = (dx - dx_prev) / dt;
            dx_prev = dx;
            let fl = d.step(f as f32, dx as f32, v as f32, true);
            if fl.is_confirmed() && first.is_none() {
                first = Some(t);
            }
            flags.push(fl);
        }
        (flags, first)
    }

    /// Linear ramp to `f` over `rise` starting at `t0`, held until `t1`, released over
    /// `fall`.
    fn trapezoid(t: f64, t0: f64, rise: f64, t1: f64, fall: f64, f: f64) -> f64 {
        if t < t0 {
            0.0
        } else if t < t0 + rise {
            f * (t - t0) / rise
        } else if t < t1 {
            f
        } else if t < t1 + fall {
            f * (1.0 - (t - t1) / fall)
        } else {
            0.0
        }
    }

    /// No false intent from a 15 N lean rising over 1 s and held 5 s, a 50 N smoke ΔP
    /// ramped over 2 s on leaf classes (1.5 N on the drawer front, which has no leaf),
    /// the DRAWER cabinet piston (5.6 N outward for 50 ms at the seat,
    /// direction-inconsistent), footsteps, or a force opposing the carriage's motion, in
    /// every class; the lean is classified as `Lean`. Positive controls: a DRAWER
    /// push-to-open at the seat confirms within 60 ms (HAP-012) and a firm push
    /// confirms in every class. (T) plan §2b item 11, FAB-HAP-006, PHYSICS §17.
    // req: FAB-HAP-006
    #[test]
    fn lean_rejected() {
        for p in [
            presets::drawer(),
            presets::door(),
            presets::wall(),
            presets::heavy(),
        ] {
            let class = p.design.class;
            let f_th = p.control.intent.f_th_n;
            let ld = &p.design.landing;
            let seat = Mount::Seat {
                k: ld.seat_k_x_n_per_m,
                travel: ld.seat_x_compliance_m,
            };
            let is_drawer = p.control.intent.push_to_open;
            let mounts: Vec<Mount> = if is_drawer {
                vec![Mount::Held, seat]
            } else {
                vec![Mount::Held]
            };
            for &mount in &mounts {
                for sign in [1.0, -1.0] {
                    // Lean: 15 N, 1 s rise, 5 s hold, 0.2 s release, then 3 s quiet.
                    let (fl, first) = run(&p, mount, 9.5, |t| {
                        sign * trapezoid(t, 0.3, 1.0, 6.3, 0.2, 15.0)
                    });
                    assert!(
                        first.is_none(),
                        "{class:?} lean ({sign:+}) gave intent at {first:?} s (FAB-HAP-006)"
                    );
                    if 15.0 > f_th {
                        assert!(
                            fl.contains(&IntentFlag::Lean),
                            "{class:?}: lean not classified"
                        );
                    }
                    // Smoke ΔP ramped over 2 s, held 5 s, ramped down over 2 s: 50 N on a
                    // leaf (G8: 12.5–25 Pa × 2 m²); a drawer has no leaf, its front sees
                    // 25 Pa × the 0.06 m² cavity face = 1.5 N.
                    let f_smoke = if p.design.air.leaf_area_m2 > 0.0 {
                        50.0
                    } else {
                        25.0 * p.design.air.cavity_area_m2
                    };
                    let (_, first) = run(&p, mount, 11.0, |t| {
                        sign * trapezoid(t, 0.2, 2.0, 7.2, 2.0, f_smoke)
                    });
                    assert!(
                        first.is_none(),
                        "{class:?} smoke ΔP ({sign:+}) gave intent at {first:?} s"
                    );
                    // Footsteps: 2 N at 2 Hz impacts, 30 ms half-sine each, for 5 s.
                    let (_, first) = run(&p, mount, 5.0, |t| {
                        let ph = (t * 2.0).fract();
                        if ph < 0.03 {
                            sign * 2.0 * (core::f64::consts::PI * ph / 0.03).sin()
                        } else {
                            0.0
                        }
                    });
                    assert!(first.is_none(), "{class:?} footsteps gave intent");
                }
            }
            if is_drawer {
                // Cabinet piston at the seat: +5.6 N (outward) for 50 ms with 2 ms edges.
                let (_, first) = run(&p, seat, 1.0, |t| {
                    trapezoid(t, 0.1, 0.002, 0.15, 0.002, 5.6)
                });
                assert!(
                    first.is_none(),
                    "piston at the seat gave intent (direction-inconsistent)"
                );
                // Push-to-open: 10 N inward over 80 ms, released after 150 ms.
                let (fl, first) = run(&p, seat, 0.6, |t| {
                    trapezoid(t, 0.1, 0.08, 0.25, 0.03, -10.0)
                });
                let t_c = first.map(|t| t - 0.1);
                assert!(
                    t_c.is_some_and(|t| t <= 0.060),
                    "push-to-open detect {t_c:?} s > 60 ms (HAP-012)"
                );
                assert!(fl.contains(&IntentFlag::Open) && !fl.contains(&IntentFlag::Close));
            }
            // A force opposing the motion (closing at 0.1 m/s, pushed outward) is not intent.
            let (_, first) = run(&p, Mount::Moving(-0.1), 1.0, |t| {
                trapezoid(t, 0.1, 0.02, 0.5, 0.02, 3.0 * f_th)
            });
            assert!(
                first.is_none(),
                "{class:?}: force against the motion gave intent"
            );
            // Firm push held still (door in Hold): force × time confirms.
            let f_push = 2.5 * f_th;
            let (fl, first) = run(&p, Mount::Held, 1.0, |t| {
                trapezoid(t, 0.1, 0.05, 0.6, 0.05, f_push)
            });
            assert!(
                first.is_some(),
                "{class:?}: a {f_push} N push was not confirmed"
            );
            assert!(
                !fl.contains(&IntentFlag::Close),
                "{class:?}: push toward +x read as close"
            );
        }
    }

    #[test]
    fn disabled_reports_none() {
        let mut d = IntentDetector::new(&presets::door()).unwrap_or_default();
        for _ in 0..5_000 {
            assert_eq!(d.step(40.0, 0.01, 0.0, false), IntentFlag::None);
        }
        assert_eq!(d.step(f32::NAN, 0.0, 0.0, true), IntentFlag::None);
    }
}
