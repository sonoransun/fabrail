//! Seismic subsystem: `SeismicUnit` at 100 Hz (detector, discrimination, fusion,
//! land-first class policy, research-tier isolation and AMD) and the `ActionPolicy` hook
//! through which exotic code and the house choreographer override the class action,
//! toward safety only (PHYSICS §20, §21; plan §4). Owner: A3 (action_policy: W0a trait,
//! X3 implementation); module list frozen by W0a.
//!
//! Frozen API (W0): [`SeismicUnit::new`], [`SeismicUnit::step`] → `SeismicReport`,
//! [`SeismicUnit::reset_inspect`]. The unit drains the structure ADXL355 FIFO samples
//! of one release (≤ `ACCEL_DRAIN_MAX`), runs frontend → STA/LTA → Pd/τc →
//! discrimination → fusion, and reports. `step` takes the release tick explicitly
//! (S-ETA and QUIET dwell are absolute ticks); the INSPECT → NORMAL manual reset arrives
//! through `reset_inspect` (forwarded by fabrail-rt from `SupStatus::inspect_reset`).
//!
//! Land-first; isolation and AMD are research tier. Not an EEW provider: network
//! alerts are synthetic or injected in simulation.
//!
//! **Pipeline per decimated sample** (100 Hz; every sample of a release is stamped with
//! the release tick, 10 ms resolution; PHYSICS §20):
//!
//! 1. [`frontend`]: CIC ÷5 anti-alias, 0.5–20 Hz band-pass per axis.
//! 2. PGA = max √(bp_x² + bp_y²) (horizontal, band-passed: tilt- and gravity-free).
//! 3. [`pdtc`] on the anti-aliased vertical channel (runs continuously; its window opens
//!    at the pick); the floor-velocity estimate on x (below).
//! 4. [`stalta`] on the band-passed vertical channel. An onset is a *candidate pick*
//!    (counted in `raw_triggers`); it is **accepted** unless the actuator veto holds
//!    ([`discriminate::vetoed`]); a vetoed candidate is accepted later only if PGA
//!    reaches the 80 gal bypass while the trigger lasts. A live accepted pick is never
//!    restarted by later onsets (the S wave re-triggers; Pd/τc stay P-wave estimates).
//! 5. [`discriminate::ZhCheck`] over the first `zh_window_s` (0.5 s): Z/H ≤ 1 rejects
//!    the pick; so does a trigger that ends before the Z/H window completes (duration).
//! 6. Once the short Pd/τc window (1 s) has run on a Z/H-confirmed pick: the level
//!    matrix (latched maximum), the Pd rule and the local S-ETA ([`fusion::LocalEta`]).
//!    The PGA rule applies at any time.
//!
//! **Per release**: a trigger held longer than `alert_timeout_s` is released (the LTA
//! would otherwise stay frozen forever after a step change in ambient noise); "shaking"
//! is a sample with PGA ≥ `pga_veto_mps2` within the last 1/`bp_lo_hz` (2 s: one period
//! of the lowest detector frequency, so an oscillation's zero crossings do not end it),
//! or a live local trigger past the S-ETA; then [`fusion::Fusion::step`] applies the
//! network alert and the state machine. On the return to NORMAL (with no trigger held)
//! the event memory is cleared. `raw_triggers`
//! counts every STA/LTA onset since boot; `faults` carries AccelLost (FIFO gap, overrun,
//! or `supervisor.heartbeat_periods` consecutive releases without a sample) and
//! AccelStuck.
//!
//! **Floor velocity** (`SeismicReport::floor_v_mps`): x acceleration minus its first
//! sample (tilt offset) → baseline HP → integrating HP (0.075 Hz, as in [`pdtc`]) →
//! first-order low-pass at `inertial.structure_lp_hz` (0.3 Hz; the structure ADXL355 is
//! used below 0.3 Hz only, plan §4b).

pub mod action_policy;
pub mod amd;
pub mod discriminate;
pub mod frontend;
pub mod fusion;
pub mod isolate;
pub mod pdtc;
pub mod policy;
pub mod stalta;

use fabrail_params::{DetectorTuning, ParamError, SystemParams};
use fabrail_types::{
    AccelRaw, FaultCode, FaultSet, NetworkAlert, SeismicLevel, SeismicState, Tick, fmath,
};

use crate::filters::{Biquad, FirstOrderLp};
use crate::msg::SeismicReport;

/// Memory of the current event's pick and estimates (cleared on the return to NORMAL).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EventTrack {
    /// Tick of the candidate/accepted pick.
    pub pick: Option<Tick>,
    /// The pick was accepted (no veto, or the PGA bypass).
    pub accepted: bool,
    /// The Z/H verdict has been taken.
    pub zh_checked: bool,
    /// Z/H > zh_min: a P-like pick.
    pub zh_passed: bool,
    /// Rejected by Z/H or by a trigger shorter than the Z/H window.
    pub rejected: bool,
    /// Latched level.
    pub level: SeismicLevel,
    /// Pd of the confirmed pick, m.
    pub pd_m: f32,
    /// τc of the confirmed pick, s.
    pub tau_c_s: f32,
    /// PGA since the event began (horizontal, band-passed), m/s².
    pub pga_mps2: f32,
    /// The local PROTECT rule was met (latched).
    pub protect: bool,
    /// Local S-ETA estimate.
    pub s_eta_local: Option<Tick>,
}

/// Low-frequency floor velocity estimate along x (see the module docs).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FloorVelocity {
    /// Baseline high-pass.
    pub hp: Biquad,
    /// Integrating high-pass.
    pub ihp: Biquad,
    /// Structure-band low-pass.
    pub lp: FirstOrderLp,
    /// The low-pass is configured.
    pub lp_on: bool,
    /// The baseline has been taken.
    pub primed: bool,
    /// Baseline (first sample), m/s².
    pub a0_mps2: f32,
    /// Last estimate, m/s.
    pub v_mps: f32,
}

impl FloorVelocity {
    /// Estimator at `fs_hz` with the Pd/τc high-pass corner and the structure low-pass.
    pub fn new(hp_hz: f64, lp_hz: f64, fs_hz: f64) -> FloorVelocity {
        let lp_on = lp_hz.is_finite() && lp_hz > 0.0 && fs_hz.is_finite() && fs_hz > 0.0;
        FloorVelocity {
            hp: pdtc::highpass2(hp_hz, fs_hz),
            ihp: pdtc::integrating_highpass2(hp_hz, fs_hz),
            lp: if lp_on {
                FirstOrderLp::new(lp_hz, fs_hz)
            } else {
                FirstOrderLp::default()
            },
            lp_on,
            ..FloorVelocity::default()
        }
    }

    /// One anti-aliased x acceleration sample, m/s²; returns the velocity estimate, m/s.
    pub fn step(&mut self, a_x_mps2: f32) -> f32 {
        if !a_x_mps2.is_finite() {
            return self.v_mps;
        }
        if !self.primed {
            self.a0_mps2 = a_x_mps2;
            self.primed = true;
        }
        let v = self.ihp.step(self.hp.step(a_x_mps2 - self.a0_mps2));
        self.v_mps = if self.lp_on { self.lp.step(v) } else { v };
        self.v_mps
    }
}

/// The 100 Hz seismic unit.
#[derive(Clone, Debug, Default)]
pub struct SeismicUnit {
    /// Release sequence number.
    pub seq: u32,
    /// STA/LTA triggers since boot.
    pub raw_triggers: u32,
    /// An INSPECT reset is pending for the next release.
    pub reset_pending: bool,
    /// Detector tuning (f64 thresholds, cast at the comparisons).
    pub det: DetectorTuning,
    /// Front end.
    pub frontend: frontend::SeismicFrontend,
    /// STA/LTA on the band-passed vertical channel.
    pub stalta: stalta::StaLta,
    /// Pd/τc on the anti-aliased vertical channel.
    pub pdtc: pdtc::PdTauC,
    /// Z/H polarisation.
    pub zh: discriminate::ZhCheck,
    /// Fusion and the state machine.
    pub fusion: fusion::Fusion,
    /// Local S-ETA estimator.
    pub eta: fusion::LocalEta,
    /// Floor velocity estimate.
    pub floor: FloorVelocity,
    /// Current event.
    pub event: EventTrack,
    /// Onset of the STA/LTA trigger currently held.
    pub trig_at: Option<Tick>,
    /// A trigger held longer than this many base ticks is released (0: never).
    pub stale_ticks: u64,
    /// Consecutive releases without a FIFO sample.
    pub silent_releases: u32,
    /// AccelLost after this many silent releases.
    pub silent_limit: u32,
    /// S-ETA of the previous release.
    pub last_s_eta: Option<Tick>,
    /// Last release with a strong-motion sample (PGA ≥ `pga_veto_mps2`).
    pub strong_at: Option<Tick>,
    /// Strong-motion hold, base ticks (one period of `bp_lo_hz`).
    pub shaking_hold_ticks: u64,
}

impl SeismicUnit {
    /// Builds the unit from `design.sensors` and `control.seismic`.
    ///
    /// Degenerate parameters (`SystemParams::default()`) give an inert unit that reports
    /// NORMAL; a front-end decimation above `frontend::AA_DECIM_MAX` is rejected.
    pub fn new(p: &SystemParams) -> Result<SeismicUnit, ParamError> {
        let det = p.control.seismic.detector;
        let fe = frontend::SeismicFrontend::new(p)?;
        let fs = if fe.decim > 0 {
            p.design.sensors.accel_odr_hz / f64::from(fe.decim)
        } else {
            0.0
        };
        let mut sl = stalta::StaLta::new(&det);
        let lsb = p.design.sensors.accel_mps2_per_lsb as f32;
        sl.floor = fmath::fmaxf(lsb * lsb, f32::MIN_POSITIVE);
        let timeout = p.control.seismic.fusion.alert_timeout_s;
        let hold = if det.bp_lo_hz.is_finite() && det.bp_lo_hz > 0.0 {
            Tick::from_seconds_f64(1.0 / det.bp_lo_hz).0
        } else {
            0
        };
        Ok(SeismicUnit {
            seq: 0,
            raw_triggers: 0,
            reset_pending: false,
            det,
            frontend: fe,
            stalta: sl,
            pdtc: pdtc::PdTauC::new(&det, fs as f32),
            zh: discriminate::ZhCheck::new(&det, fs as f32),
            fusion: fusion::Fusion::new(p)?,
            eta: fusion::LocalEta::new(p),
            floor: FloorVelocity::new(
                det.pdtc_hp_hz,
                p.control.motion.inertial.structure_lp_hz,
                fs,
            ),
            event: EventTrack::default(),
            trig_at: None,
            stale_ticks: if timeout.is_finite() && timeout > 0.0 {
                Tick::from_seconds_f64(timeout).0
            } else {
                0
            },
            silent_releases: 0,
            silent_limit: u32::from(p.control.supervisor.heartbeat_periods.max(1)),
            last_s_eta: None,
            strong_at: None,
            shaking_hold_ticks: hold,
        })
    }

    /// One decimated sample (pipeline steps 2–6 of the module docs); returns its
    /// horizontal band-passed acceleration magnitude, m/s².
    fn sample(&mut self, t: Tick, o: &frontend::SeismicSample, veto: bool) -> f32 {
        let det = self.det;
        let [bx, by, bz] = o.bp_mps2;
        let [ax, _, az] = o.raw_mps2;
        let a_h = fmath::hypotf(bx, by);
        let a_h = if a_h.is_finite() { a_h } else { 0.0 };
        self.event.pga_mps2 = fmath::fmaxf(self.event.pga_mps2, a_h);
        self.floor.step(ax);
        let pd = self.pdtc.step(az);
        let was = self.stalta.triggered;
        let so = self.stalta.step(bz);
        let ev = &mut self.event;
        if so.onset {
            self.raw_triggers = self.raw_triggers.saturating_add(1);
            self.trig_at = Some(t);
            if !ev.accepted || ev.rejected {
                self.pdtc.start();
                self.zh.reset();
                ev.pick = Some(t);
                ev.accepted = !discriminate::vetoed(veto, a_h, &det);
                ev.rejected = false;
                ev.zh_checked = false;
                ev.zh_passed = false;
            }
        }
        if self.stalta.triggered && ev.pick.is_some() && !ev.rejected {
            if !ev.accepted && a_h >= det.pga_veto_mps2 as f32 {
                ev.accepted = true; // PGA bypass of the actuator veto
            }
            self.zh.push(o.bp_mps2);
        }
        if ev.accepted && !ev.rejected && !ev.zh_checked {
            if self.zh.n_window == 0 {
                ev.zh_checked = true;
                ev.zh_passed = true;
            } else if self.zh.done() {
                ev.zh_checked = true;
                ev.zh_passed = self.zh.is_p_like(&det);
                ev.rejected = !ev.zh_passed;
            } else if was && !self.stalta.triggered {
                ev.rejected = true; // shorter than the Z/H window: a transient
            }
        }
        if ev.accepted && ev.zh_passed && !ev.rejected && self.pdtc.short_done() {
            ev.pd_m = pd.pd_m;
            ev.tau_c_s = pd.tau_c_s;
            let lv = discriminate::level(pd.pd_m, pd.tau_c_s, &det);
            if lv.as_u8() > ev.level.as_u8() {
                ev.level = lv;
            }
            if discriminate::protect(ev.level, ev.pd_m, 0.0, &det) {
                ev.protect = true;
            }
            if let Some(p0) = ev.pick {
                ev.s_eta_local = self.eta.s_eta(p0, pd.pd_m, pd.tau_c_s).or(ev.s_eta_local);
            }
        }
        if det.pga_alert_mps2 > 0.0 && discriminate::protect(SeismicLevel::L0, 0.0, a_h, &det) {
            ev.protect = true; // PGA ≥ 80 gal, trigger or not
        }
        a_h
    }

    /// One seismic release at tick `t` with this release's FIFO samples `a`, an optional
    /// network alert, and the supervisor's actuator veto (self-actuation vibration;
    /// bypassed when PGA ≥ 80 gal).
    pub fn step(
        &mut self,
        t: Tick,
        a: &[AccelRaw],
        alert: Option<&NetworkAlert>,
        veto: bool,
    ) -> SeismicReport {
        self.seq = self.seq.wrapping_add(1);
        let mut faults = FaultSet::EMPTY;
        if a.is_empty() {
            self.silent_releases = self.silent_releases.saturating_add(1);
            if self.silent_releases >= self.silent_limit {
                faults.set(FaultCode::AccelLost);
            }
        } else {
            self.silent_releases = 0;
        }
        let mut pga_now = 0.0_f32;
        for s in a {
            let (o, f) = self.frontend.push(s);
            faults.merge(f);
            if let Some(o) = o {
                pga_now = fmath::fmaxf(pga_now, self.sample(t, &o, veto));
            }
        }
        if self.stalta.triggered
            && self.stale_ticks > 0
            && self
                .trig_at
                .is_some_and(|t0| t.since(t0) >= self.stale_ticks)
        {
            self.stalta.release();
        }

        let det = &self.det;
        let ev = self.event;
        let live = ev.accepted && !ev.rejected;
        let local_trigger = self.stalta.triggered && live;
        if det.pga_veto_mps2 > 0.0 && pga_now >= det.pga_veto_mps2 as f32 {
            self.strong_at = Some(t);
        }
        let strong = self
            .strong_at
            .is_some_and(|s0| t.since(s0) <= self.shaking_hold_ticks);
        let shaking = strong || (local_trigger && self.last_s_eta.is_some_and(|e| t >= e));
        let fev = fusion::FusionEvents {
            local_trigger,
            protect: ev.protect,
            shaking,
            local_p: live && ev.zh_passed,
            local_s_eta: ev.s_eta_local,
            inspect_reset: self.reset_pending,
            ..fusion::FusionEvents::default()
        };
        self.reset_pending = false;
        let fo = self.fusion.step(t, &fev, alert);
        self.last_s_eta = fo.s_eta;
        if fo.state == SeismicState::Normal && !self.stalta.triggered {
            self.event = EventTrack::default();
            self.pdtc.reset();
        }
        let ev = self.event;
        let live = ev.accepted && !ev.rejected;
        SeismicReport {
            tick: t,
            seq: self.seq,
            state: fo.state,
            level: ev.level,
            protect: fo.protect && fo.state != SeismicState::Normal,
            triggered_at: if live { ev.pick } else { None },
            pd_m: ev.pd_m,
            tau_c_s: ev.tau_c_s,
            pga_mps2: ev.pga_mps2,
            s_eta: fo.s_eta,
            source: fo.source,
            local_p: live && ev.zh_passed,
            n_feeds: fo.n_feeds,
            cancel: fo.cancel,
            floor_v_mps: self.floor.v_mps,
            raw_triggers: self.raw_triggers,
            faults,
        }
    }

    /// Requests the manual INSPECT → NORMAL reset (applied on the next release; ignored
    /// in other states).
    pub fn reset_inspect(&mut self) {
        self.reset_pending = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;
    use fabrail_types::AlertSource;

    #[test]
    fn quiet_input_reports_normal() {
        let p = SystemParams::default();
        let mut u = SeismicUnit::new(&p).unwrap_or_default();
        let r = u.step(Tick(200), &[AccelRaw::default(); 5], None, false);
        assert_eq!(r.state, SeismicState::Normal);
        assert_eq!(r.tick, Tick(200));
        assert!(!r.protect);
    }

    /// Drives a unit with structure acceleration (m/s², zero-mean: the in-crate tests do
    /// not depend on the band-pass removing gravity) at the 500 Hz ODR, one release per
    /// 200 base ticks.
    struct Rig {
        u: SeismicUnit,
        t: Tick,
        seq: u32,
        k: u64,
        lcg: u64,
        lsb: f64,
    }

    impl Rig {
        fn new(p: &SystemParams) -> Rig {
            Rig {
                u: SeismicUnit::new(p).expect("unit"),
                t: Tick(0),
                seq: 0,
                k: 0,
                lcg: 0x2545_f491_4f6c_dd1d,
                lsb: p.design.sensors.accel_mps2_per_lsb,
            }
        }

        /// ≈ N(0, σ²) from four uniforms (deterministic).
        fn noise(&mut self, sigma: f64) -> f64 {
            let mut s = 0.0;
            for _ in 0..4 {
                self.lcg = self
                    .lcg
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                s += (self.lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5;
            }
            s * sigma * 3.0_f64.sqrt()
        }

        /// Runs `dur_s` of signal `f(t_s) → [ax, ay, az]` plus 1 mg-class noise.
        fn run(
            &mut self,
            dur_s: f64,
            veto: bool,
            f: impl Fn(f64) -> [f64; 3],
            mut alert: Option<NetworkAlert>,
        ) -> Vec<SeismicReport> {
            let mut out = Vec::new();
            let n_rel = (dur_s * 100.0).round() as u64;
            for _ in 0..n_rel {
                let mut buf = [AccelRaw::default(); 5];
                for s in buf.iter_mut() {
                    let ts = self.k as f64 / 500.0;
                    let a = f(ts);
                    let mut xyz = [0_i32; 3];
                    for (c, v) in xyz.iter_mut().zip(a.iter()) {
                        *c = ((v + self.noise(1.0e-3)) / self.lsb).round() as i32;
                    }
                    self.seq = self.seq.wrapping_add(1);
                    *s = AccelRaw {
                        xyz,
                        seq: self.seq,
                        overrun: false,
                    };
                    self.k += 1;
                }
                self.t = self.t.plus(200);
                out.push(self.u.step(self.t, &buf, alert.take().as_ref(), veto));
            }
            out
        }

        fn now_s(&self) -> f64 {
            self.t.seconds_f64()
        }
    }

    /// A P-like burst: 4 Hz, vertical with H/Z = tan 20° split over x and y, from `t0`.
    fn p_burst(t0: f64, amp: f64) -> impl Fn(f64) -> [f64; 3] {
        let h = 20.0_f64.to_radians().tan();
        move |ts: f64| {
            if ts < t0 {
                return [0.0; 3];
            }
            let tau = ts - t0;
            let env = (tau / 0.3).min(1.0);
            let s = amp * env * (2.0 * std::f64::consts::PI * 4.0 * tau).sin();
            [0.6 * h * s, 0.8 * h * s, s]
        }
    }

    #[test]
    fn p_pick_freezes_fast_and_waits_for_the_short_window() {
        let p = presets::wall();
        let mut rig = Rig::new(&p);
        let quiet = rig.run(35.0, false, |_| [0.0; 3], None);
        assert!(
            quiet.iter().all(|r| r.state == SeismicState::Normal),
            "35 s of noise: NORMAL"
        );
        assert_eq!(rig.u.raw_triggers, 0, "no raw trigger on 1 mg noise");
        let t0 = rig.now_s();
        let rs = rig.run(3.0, false, p_burst(t0, 0.03), None);
        let first = rs
            .iter()
            .position(|r| r.state != SeismicState::Normal)
            .expect("the P burst triggers");
        let r = &rs[first];
        assert_eq!(
            r.state,
            SeismicState::Triggered,
            "a local pick freezes first"
        );
        let lag = r.tick.seconds_f64() - t0;
        assert!(
            lag <= 0.1,
            "pick → FREEZE {lag:.3} s ≤ 100 ms (SEI-009 detector share)"
        );
        assert_eq!(r.source, AlertSource::Local);
        assert_eq!(r.triggered_at, Some(r.tick), "the pick tick is reported");
        assert!(!r.local_p, "Z/H not yet confirmed at the pick");
        // PGA stays below 80 gal: no PROTECT before the short Pd/τc window.
        for r in rs.iter().filter(|r| r.tick.seconds_f64() < t0 + lag + 0.99) {
            assert!(
                !r.protect && r.state != SeismicState::Protect,
                "no PROTECT inside the 1 s window"
            );
        }
        let late = rs.last().expect("reports");
        assert!(late.local_p, "Z/H ≈ 2.7 confirms the pick");
        assert_eq!(
            late.level,
            discriminate::level(late.pd_m, late.tau_c_s, &p.control.seismic.detector)
        );
        assert_eq!(
            late.protect,
            discriminate::protect(
                late.level,
                late.pd_m,
                late.pga_mps2,
                &p.control.seismic.detector
            ),
            "PROTECT ⟺ the rule on the reported estimates"
        );
        assert_eq!(rig.u.raw_triggers, 1, "one onset");
    }

    #[test]
    fn horizontal_transient_is_rejected_and_self_clears() {
        let p = presets::door();
        let mut rig = Rig::new(&p);
        rig.run(31.0, false, |_| [0.0; 3], None);
        let t0 = rig.now_s();
        // A slam-like horizontal burst with a small vertical part (Z/H ≈ 0.3).
        let slam = move |ts: f64| {
            if !(t0..t0 + 1.5).contains(&ts) {
                return [0.0; 3];
            }
            let s = 0.05 * (2.0 * std::f64::consts::PI * 6.0 * (ts - t0)).sin();
            [s, 0.3 * s, 0.3 * s]
        };
        let rs = rig.run(6.0, false, slam, None);
        assert!(
            rig.u.raw_triggers >= 1,
            "the STA/LTA fires (raw trigger counted)"
        );
        assert!(
            rs.iter().all(|r| !r.protect && !r.local_p),
            "never P-like, never PROTECT"
        );
        assert_eq!(
            rs.last().map(|r| r.state),
            Some(SeismicState::Normal),
            "back to NORMAL"
        );
    }

    #[test]
    fn veto_suppresses_small_triggers_and_pga_bypasses() {
        let p = presets::wall();
        let mut rig = Rig::new(&p);
        rig.run(31.0, true, |_| [0.0; 3], None);
        let t0 = rig.now_s();
        let rs = rig.run(3.0, true, p_burst(t0, 0.03), None);
        assert!(rig.u.raw_triggers >= 1, "the onset is counted");
        assert!(
            rs.iter().all(|r| r.state == SeismicState::Normal),
            "vetoed: no event"
        );
        // Strong shaking under the same veto: PGA ≥ 80 gal bypasses it (SEI-006).
        let mut rig = Rig::new(&p);
        rig.run(31.0, true, |_| [0.0; 3], None);
        let t0 = rig.now_s();
        let strong = move |ts: f64| {
            let s = if ts < t0 {
                0.0
            } else {
                (2.0 * std::f64::consts::PI * 2.0 * (ts - t0)).sin()
            };
            [1.2 * s, 0.0, 0.3 * s]
        };
        let rs = rig.run(2.0, true, strong, None);
        let prot = rs.iter().find(|r| r.protect).expect("PGA bypass → PROTECT");
        assert!(prot.pga_mps2 >= 0.8, "PGA {} ≥ 80 gal", prot.pga_mps2);
        assert!(matches!(
            prot.state,
            SeismicState::Protect | SeismicState::Shaking
        ));
    }

    #[test]
    fn strong_shaking_runs_the_fsm_to_inspect_and_resets() {
        let p = presets::heavy();
        let mut rig = Rig::new(&p);
        rig.run(31.0, false, |_| [0.0; 3], None);
        let t0 = rig.now_s();
        let shake = move |ts: f64| {
            if !(t0..t0 + 8.0).contains(&ts) {
                return [0.0; 3];
            }
            let s = (2.0 * std::f64::consts::PI * 1.5 * (ts - t0)).sin();
            [1.0 * s, 0.5 * s, 0.2 * s]
        };
        let mut rs = rig.run(10.0, false, shake, None);
        assert!(
            rs.iter().any(|r| r.state == SeismicState::Shaking),
            "strong motion → SHAKING"
        );
        assert!(rs.iter().any(|r| r.protect), "PGA ≥ 80 gal → PROTECT");
        // The STA/LTA may hold its trigger into the coda; allow it to clear, then 30 s QUIET.
        rs.extend(rig.run(120.0, false, |_| [0.0; 3], None));
        let q = rs.iter().filter(|r| r.state == SeismicState::Quiet).count();
        assert!(
            (2999..=3001).contains(&q),
            "QUIET dwell 30 s ({q} releases)"
        );
        assert_eq!(
            rs.last().map(|r| r.state),
            Some(SeismicState::Inspect),
            "awaits the manual reset"
        );
        rig.u.reset_inspect();
        let r = rig.run(0.01, false, |_| [0.0; 3], None);
        assert_eq!(
            r[0].state,
            SeismicState::Normal,
            "INSPECT → NORMAL on reset"
        );
        assert_eq!(
            (r[0].level, r[0].pga_mps2 < 0.1),
            (SeismicLevel::L0, true),
            "event memory cleared"
        );
    }

    #[test]
    fn network_alert_eta_and_cancel_through_the_unit() {
        let p = presets::door();
        let mut rig = Rig::new(&p);
        rig.run(5.0, false, |_| [0.0; 3], None);
        let rx = rig.t.plus(200);
        let a = NetworkAlert {
            id: 3,
            received: rx,
            origin_offset_s: -4.0,
            magnitude: 7.0,
            r_epi_km: 60.0,
            depth_km: 12.0,
            mmi: 6.0,
            cancel: false,
        };
        let r = rig.run(0.01, false, |_| [0.0; 3], Some(a));
        assert_eq!(
            r[0].state,
            SeismicState::Protect,
            "network alert → PROTECT (class action before t_P)"
        );
        assert_eq!(
            (r[0].source, r[0].n_feeds, r[0].local_p),
            (AlertSource::Network, 1, false)
        );
        let want = rx.seconds_f64() - 4.0 + 60.0_f64.hypot(12.0) / 3.5;
        let eta = r[0].s_eta.expect("S-ETA").seconds_f64();
        assert!(
            (eta - want).abs() <= 1.0,
            "S-ETA {eta:.3} vs {want:.3} s (±1 s; SEI-007)"
        );
        let c = NetworkAlert { cancel: true, ..a };
        let r = rig.run(0.01, false, |_| [0.0; 3], Some(c));
        assert!(r[0].cancel);
        assert_eq!(r[0].state, SeismicState::Normal, "cancel → NORMAL");
    }

    #[test]
    fn silent_sensor_raises_accel_lost() {
        let p = presets::drawer();
        let mut u = SeismicUnit::new(&p).expect("unit");
        let mut lost = Vec::new();
        for k in 1..=5_u64 {
            lost.push(
                u.step(Tick(200 * k), &[], None, false)
                    .faults
                    .contains(FaultCode::AccelLost),
            );
        }
        assert_eq!(
            lost,
            [false, false, true, true, true],
            "3 silent releases (heartbeat periods)"
        );
    }
}
