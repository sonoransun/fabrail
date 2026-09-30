//! Local/network alert fusion, S-wave ETA and the SeismicState machine (PHYSICS §20; SEI-007, SEI-008). Owner: A3.
//!
//! Frozen API (W0): [`Fusion::new`], [`Fusion::step`] → [`FusionOut`], and the pure
//! transition function [`next_state`]. States NORMAL → TRIGGERED (freeze) → PROTECT
//! (class action) → SHAKING → QUIET (30 s) → INSPECT (manual reset) → NORMAL; a cancel
//! from any state returns to NORMAL. S-ETA from a network alert is origin + R_hyp/V_s
//! (V_p 6.0, V_s 3.5 km/s; S−P 0.1190 s/km, crossover 40.8 km); from a local pick it is
//! the pick + S−P at the distance implied by the level. fabrail is not an EEW provider:
//! every alert in simulation is synthetic or injected. Tests: `fsm_exhaustive`
//! (proptest event sequences), `eta_from_alert`.
//!
//! **Transitions** ([`next_state`], evaluated once per 100 Hz release; "active" means a
//! local trigger or strong shaking this release):
//!
//! | from | to | when |
//! |---|---|---|
//! | any | NORMAL | `cancel` |
//! | NORMAL | PROTECT / TRIGGERED | PROTECT rule met / active |
//! | TRIGGERED | PROTECT / NORMAL | PROTECT rule met / no longer active (level ≤ 1 events self-clear: freeze only) |
//! | PROTECT | SHAKING / QUIET | shaking / not active and `quiet_ticks` > 0 |
//! | SHAKING | QUIET | not active and `quiet_ticks` > 0 |
//! | QUIET | SHAKING / INSPECT | active again / `quiet_ticks` ≥ the QUIET dwell (30 s) |
//! | INSPECT | NORMAL | manual `inspect_reset` only |
//!
//! **Fusion** ([`Fusion::step`]). A non-cancel [`NetworkAlert`] (new or updated id) is
//! stored and meets the PROTECT rule while active (network alerts may cause only FREEZE,
//! LAND, LOCK or a passive ride-through; motion needs confirmation, enforced by
//! `action_policy::admit`). It expires `alert_timeout_s` after its last update. Its S-ETA
//! is received + origin_offset_s + R_hyp/V_s with R_hyp = √(r_epi² + depth²) (PHYSICS §20),
//! preferred over the local estimate. A cancel for the active id clears it and returns the
//! machine to NORMAL **only if no local evidence** (a Z/H-confirmed pick or strong shaking)
//! was seen during the event; local physics outranks a remote retraction. `quiet_ticks` is
//! recomputed here as the ticks since the last activity (local trigger, shaking or an
//! active alert). Source: Local, Network, or Fused when both contributed during the event
//! (sticky until NORMAL, so `OpenAndStop` stays forbidden once any non-local source
//! contributed); `n_feeds` counts the local pick and the network link.
//!
//! **Local S-ETA** ([`LocalEta`]). The distance implied by the detector estimates: the
//! τc magnitude log τc = 0.296·M − 1.716 (Wu & Kanamori 2008, Sensors 8:1–9 \[UNCERTAIN\]:
//! ±0.4 M) and Wu–Zhao inverted for R (coefficients taken at init from
//! `params::derive::pd_wu_zhao`, the single source), then S−P = R·(1/V_s − 1/V_p), biased
//! early by one `eta_sigma_s`. Early is the safe side: it only shrinks the budget for
//! centring or an egress open-and-stop.

use fabrail_params::{ParamError, SystemParams, derive};
use fabrail_types::{AlertSource, BASE_TICK_HZ, NetworkAlert, SeismicState, Tick, fmath};

/// Events the state machine reacts to in one seismic release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FusionEvents {
    /// A local STA/LTA trigger is active (not vetoed).
    pub local_trigger: bool,
    /// The PROTECT rule is met (local or network).
    pub protect: bool,
    /// Shaking energy above the S threshold.
    pub shaking: bool,
    /// Ticks since the last trigger or shaking sample.
    pub quiet_ticks: u64,
    /// A cancel for the active alert arrived.
    pub cancel: bool,
    /// Manual INSPECT reset requested.
    pub inspect_reset: bool,
    /// A Z/H-confirmed local P pick belongs to the current event (independent
    /// confirmation, plan §4). Input to [`Fusion::step`]; ignored by [`next_state`].
    pub local_p: bool,
    /// Local S-ETA estimate, if the detector has one. Input to [`Fusion::step`].
    pub local_s_eta: Option<Tick>,
}

/// Fusion output for one release.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FusionOut {
    /// State after this release.
    pub state: SeismicState,
    /// Decision source.
    pub source: AlertSource,
    /// Predicted S arrival.
    pub s_eta: Option<Tick>,
    /// Independent feeds contributing.
    pub n_feeds: u8,
    /// A network cancel for the active alert arrived this release.
    pub cancel: bool,
    /// The fused PROTECT rule (local rule OR an active network alert).
    pub protect: bool,
}

/// Alert fusion and the SeismicState machine.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Fusion {
    /// Current state.
    pub state: SeismicState,
    /// Active network alert, if any.
    pub alert: Option<NetworkAlert>,
    /// QUIET dwell, base ticks.
    pub quiet_ticks: u64,
    /// S-wave speed, km/s.
    pub v_s_kmps: f32,
    /// Alert expiry after its last update, base ticks (0: never).
    pub alert_timeout_ticks: u64,
    /// Tick of the active alert's last update.
    pub alert_rx: Tick,
    /// Network S-ETA of the current event (kept after expiry until NORMAL).
    pub s_eta_net: Option<Tick>,
    /// A network alert contributed to the current event.
    pub net_seen: bool,
    /// Local evidence (confirmed pick or strong shaking) seen in the current event.
    pub local_seen: bool,
    /// Tick of the last activity.
    pub last_active: Tick,
}

/// Seconds → base ticks (nearest; negative or non-finite → 0).
#[inline]
pub(crate) fn ticks_of_s(t_s: f32) -> u64 {
    let n = fmath::roundf(t_s * BASE_TICK_HZ as f32);
    if n.is_finite() && n > 0.0 {
        n as u64
    } else {
        0
    }
}

/// `t` shifted by a signed number of seconds, saturating at tick 0 (f32 seconds: 1 tick
/// resolution up to ≈ 800 s, far beyond any S−P time).
#[inline]
pub(crate) fn tick_offset(t: Tick, dt_s: f32) -> Tick {
    if dt_s >= 0.0 {
        t.plus(ticks_of_s(dt_s))
    } else {
        Tick(t.0.saturating_sub(ticks_of_s(-dt_s)))
    }
}

/// S-ETA of a network alert: received + origin_offset_s + R_hyp/V_s, with
/// R_hyp = √(r_epi² + depth²) (PHYSICS §20; SEI-007). `None` for a cancel, a non-finite
/// field or a non-positive V_s. An S arrival before tick 0 saturates to tick 0.
pub fn s_eta_from_alert(a: &NetworkAlert, v_s_kmps: f32) -> Option<Tick> {
    let r = fmath::hypotf(a.r_epi_km, a.depth_km);
    let dt = a.origin_offset_s + r / v_s_kmps;
    if a.cancel || !(v_s_kmps > 0.0 && dt.is_finite()) {
        return None;
    }
    Some(tick_offset(a.received, dt))
}

/// On-site S-ETA from the detector's Pd and τc (see the module docs).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LocalEta {
    /// Wu–Zhao log₁₀ Pd\[cm\] at M = 0, R = 1 km.
    pub c0: f32,
    /// Wu–Zhao magnitude coefficient.
    pub c_m: f32,
    /// Wu–Zhao distance coefficient (log₁₀ Pd falls by `c_r` per decade of R).
    pub c_r: f32,
    /// S−P slowness 1/V_s − 1/V_p, s/km.
    pub sp_s_per_km: f32,
    /// Early bias (one S-ETA σ), s.
    pub sigma_s: f32,
}

/// τc–magnitude regression log₁₀ τc = TAUC_M_SLOPE·M + TAUC_M_ICPT (Wu & Kanamori 2008,
/// Sensors 8:1–9; PHYSICS §20) \[UNCERTAIN\].
const TAUC_M_SLOPE: f32 = 0.296;
/// See [`TAUC_M_SLOPE`].
const TAUC_M_ICPT: f32 = -1.716;

impl LocalEta {
    /// Coefficients from `derive::pd_wu_zhao` (evaluated at init, f64) and the fusion
    /// tuning (V_p, V_s, σ).
    pub fn new(p: &SystemParams) -> LocalEta {
        let f = &p.control.seismic.fusion;
        let lg = |m: f64, r: f64| fmath::log10(100.0 * derive::pd_wu_zhao(m, r));
        let c0 = lg(0.0, 1.0);
        let sp = if f.v_s_kmps > 0.0 && f.v_p_kmps > 0.0 {
            derive::sp_time(1.0, f.v_p_kmps, f.v_s_kmps)
        } else {
            0.0
        };
        LocalEta {
            c0: c0 as f32,
            c_m: (lg(1.0, 1.0) - c0) as f32,
            c_r: (c0 - lg(0.0, 10.0)) as f32,
            sp_s_per_km: sp as f32,
            sigma_s: fmath::fmax(f.eta_sigma_s, 0.0) as f32,
        }
    }

    /// Magnitude implied by τc (Wu & Kanamori 2008) \[UNCERTAIN\].
    #[inline]
    pub fn magnitude(&self, tau_c_s: f32) -> f32 {
        (fmath::log10f(tau_c_s) - TAUC_M_ICPT) / TAUC_M_SLOPE
    }

    /// Hypocentral distance implied by Pd (m) and τc (s): Wu–Zhao inverted, km.
    pub fn r_hyp_km(&self, pd_m: f32, tau_c_s: f32) -> Option<f32> {
        if !(pd_m > 0.0 && tau_c_s > 0.0 && self.c_r > 0.0) {
            return None;
        }
        let m = self.magnitude(tau_c_s);
        let lr = (self.c0 + self.c_m * m - fmath::log10f(100.0 * pd_m)) / self.c_r;
        let r = fmath::powf(10.0, lr);
        r.is_finite().then_some(r)
    }

    /// S-ETA after a pick at `pick`: pick + max(0, S−P(R) − σ).
    pub fn s_eta(&self, pick: Tick, pd_m: f32, tau_c_s: f32) -> Option<Tick> {
        let r = self.r_hyp_km(pd_m, tau_c_s)?;
        let dt = fmath::fmaxf(r * self.sp_s_per_km - self.sigma_s, 0.0);
        Some(tick_offset(pick, dt))
    }
}

impl Fusion {
    /// Fusion from `control.seismic.fusion`.
    pub fn new(p: &SystemParams) -> Result<Fusion, ParamError> {
        let f = &p.control.seismic.fusion;
        let dwell = |s: f64| {
            if s.is_finite() && s > 0.0 {
                Tick::from_seconds_f64(s).0
            } else {
                0
            }
        };
        Ok(Fusion {
            quiet_ticks: dwell(f.quiet_s),
            v_s_kmps: f.v_s_kmps as f32,
            alert_timeout_ticks: dwell(f.alert_timeout_s),
            ..Fusion::default()
        })
    }

    /// One seismic release at tick `t` with the local detector events and an optional
    /// network alert (a new or updated alert, or a cancel).
    pub fn step(&mut self, t: Tick, ev: &FusionEvents, alert: Option<&NetworkAlert>) -> FusionOut {
        let mut net_cancel = false;
        if let Some(a) = alert {
            if a.cancel {
                if self.alert.is_some_and(|x| x.id == a.id) {
                    self.alert = None;
                    net_cancel = true;
                }
            } else {
                self.alert = Some(*a);
                self.alert_rx = t;
                self.s_eta_net = s_eta_from_alert(a, self.v_s_kmps);
                self.net_seen = true;
            }
        }
        if self.alert.is_some()
            && self.alert_timeout_ticks > 0
            && t.since(self.alert_rx) >= self.alert_timeout_ticks
        {
            self.alert = None;
        }
        let net_active = self.alert.is_some();
        if ev.local_p || ev.shaking {
            self.local_seen = true;
        }
        let active = ev.local_trigger || ev.shaking || net_active;
        if active {
            self.last_active = t;
        }
        let fe = FusionEvents {
            protect: ev.protect || net_active,
            quiet_ticks: if active { 0 } else { t.since(self.last_active) },
            cancel: ev.cancel || (net_cancel && !self.local_seen),
            ..*ev
        };
        let prev = self.state;
        self.state = next_state(prev, &fe, self.quiet_ticks);
        let out = FusionOut {
            state: self.state,
            source: match (ev.local_p, self.net_seen) {
                (true, true) => AlertSource::Fused,
                (false, true) => AlertSource::Network,
                _ => AlertSource::Local,
            },
            s_eta: self.s_eta_net.or(ev.local_s_eta),
            n_feeds: u8::from(ev.local_p) + u8::from(self.net_seen),
            cancel: net_cancel,
            protect: fe.protect,
        };
        if self.state == SeismicState::Normal && prev != SeismicState::Normal {
            self.end_event();
        }
        out
    }

    /// Clears the event-scoped memory (on the return to NORMAL): an alert still active at
    /// an INSPECT reset has been acted on and is consumed.
    fn end_event(&mut self) {
        self.alert = None;
        self.s_eta_net = None;
        self.net_seen = false;
        self.local_seen = false;
    }

    /// Returns to NORMAL with no event memory (keeps the tuning).
    pub fn reset(&mut self) {
        self.state = SeismicState::Normal;
        self.end_event();
    }
}

/// Pure SeismicState transition for one release (see the module table).
pub fn next_state(s: SeismicState, ev: &FusionEvents, quiet_dwell_ticks: u64) -> SeismicState {
    use SeismicState::{Inspect, Normal, Protect, Quiet, Shaking, Triggered};
    if ev.cancel {
        return Normal;
    }
    let active = ev.local_trigger || ev.shaking;
    let holding = active || ev.quiet_ticks == 0;
    match s {
        Normal | Triggered if ev.protect => Protect,
        Normal | Triggered if active => Triggered,
        Normal | Triggered => Normal,
        Protect if ev.shaking => Shaking,
        Protect if holding => Protect,
        Shaking if holding => Shaking,
        Protect | Shaking => Quiet,
        Quiet if active => Shaking,
        Quiet if ev.quiet_ticks >= quiet_dwell_ticks => Inspect,
        Quiet => Quiet,
        Inspect if ev.inspect_reset => Normal,
        Inspect => Inspect,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use SeismicState::{Inspect, Normal, Protect, Quiet, Shaking, Triggered};
    use fabrail_params::presets;
    use proptest::prelude::*;

    const DWELL: u64 = 600_000; // 30 s at 20 kHz

    /// The declared edge set (module table), self-loops included.
    fn edge_allowed(a: SeismicState, b: SeismicState, ev: &FusionEvents) -> bool {
        if b == Normal && ev.cancel {
            return true;
        }
        matches!(
            (a, b),
            (Normal, Normal | Triggered | Protect)
                | (Triggered, Triggered | Protect | Normal)
                | (Protect, Protect | Shaking | Quiet)
                | (Shaking, Shaking | Quiet)
                | (Quiet, Quiet | Shaking | Inspect)
                | (Inspect, Inspect)
        ) || (a == Inspect && b == Normal && ev.inspect_reset)
    }

    fn all_events() -> impl Iterator<Item = FusionEvents> {
        (0..64_u32).flat_map(|m| {
            [0, 1, DWELL - 1, DWELL]
                .into_iter()
                .map(move |q| FusionEvents {
                    local_trigger: m & 1 != 0,
                    protect: m & 2 != 0,
                    shaking: m & 4 != 0,
                    cancel: m & 8 != 0,
                    inspect_reset: m & 16 != 0,
                    local_p: m & 32 != 0,
                    quiet_ticks: q,
                    local_s_eta: None,
                })
        })
    }

    #[test]
    fn transition_table_exhaustive() {
        let mut reached = [false; 6];
        for &s in SeismicState::ALL {
            for ev in all_events() {
                let n = next_state(s, &ev, DWELL);
                assert!(
                    edge_allowed(s, n, &ev),
                    "{s:?} → {n:?} on {ev:?} is not a declared edge"
                );
                if ev.cancel {
                    assert_eq!(n, Normal, "cancel from {s:?} returns to NORMAL");
                }
                if s == Inspect && !ev.cancel {
                    assert_eq!(
                        n == Normal,
                        ev.inspect_reset,
                        "INSPECT needs the manual reset"
                    );
                }
                if s == Quiet && n == Inspect {
                    assert!(ev.quiet_ticks >= DWELL, "QUIET dwell 30 s before INSPECT");
                }
                if let Some(r) = reached.get_mut(usize::from(n.as_u8())) {
                    *r = true;
                }
            }
        }
        assert!(
            reached.iter().all(|&r| r),
            "every state is reachable: {reached:?}"
        );
    }

    fn arb_events() -> impl Strategy<Value = FusionEvents> {
        (
            any::<[bool; 5]>(),
            prop_oneof![
                Just(0_u64),
                Just(1),
                Just(DWELL - 1),
                Just(DWELL),
                0..2 * DWELL
            ],
        )
            .prop_map(|(b, q)| FusionEvents {
                local_trigger: b[0],
                protect: b[1],
                shaking: b[2],
                // Cancels and resets are rarer so long sequences explore deep states.
                cancel: b[3] && b[4] && b[0],
                inspect_reset: b[4],
                quiet_ticks: q,
                local_p: false,
                local_s_eta: None,
            })
    }

    // req: FAB-SEI-008
    proptest! {
        #[test]
        fn fsm_exhaustive(seq in prop::collection::vec(arb_events(), 1..160)) {
            let mut s = Normal;
            for ev in &seq {
                let n = next_state(s, ev, DWELL);
                prop_assert!(edge_allowed(s, n, ev), "{:?} → {:?} on {:?}", s, n, ev);
                if ev.cancel {
                    prop_assert_eq!(n, Normal, "reversible on cancel");
                }
                if s == Inspect && n == Normal {
                    prop_assert!(ev.inspect_reset || ev.cancel, "INSPECT leaves only on reset/cancel");
                }
                s = n;
            }
            // Liveness: from wherever the sequence ended, quiet then a manual reset
            // always reaches NORMAL (no stuck state).
            let quiet = FusionEvents { quiet_ticks: DWELL, ..FusionEvents::default() };
            for _ in 0..4 {
                s = next_state(s, &quiet, DWELL);
            }
            let reset = FusionEvents { inspect_reset: true, ..quiet };
            s = next_state(s, &reset, DWELL);
            prop_assert_eq!(s, Normal, "quiet + INSPECT reset reaches NORMAL");
        }
    }

    fn alert(id: u32, rx_s: f64, origin_offset_s: f32) -> NetworkAlert {
        NetworkAlert {
            id,
            received: Tick::from_seconds_f64(rx_s),
            origin_offset_s,
            magnitude: 7.0,
            r_epi_km: 20.0,
            depth_km: 10.0,
            mmi: 7.0,
            cancel: false,
        }
    }

    // req: FAB-SEI-007
    #[test]
    fn eta_from_alert() {
        let p = presets::door();
        let v_s = p.control.seismic.fusion.v_s_kmps;
        assert_eq!(v_s, 3.5, "V_s 3.5 km/s (core design §C.4)");
        // M7 at r_epi 20 km, 10 km deep, origin 2 s before reception at t = 10 s:
        // S-ETA = 10 − 2 + √(20² + 10²)/3.5 = 14.3888 s.
        let a = alert(7, 10.0, -2.0);
        let want_s = 10.0 - 2.0 + (20.0_f64.hypot(10.0)) / v_s;
        let got = s_eta_from_alert(&a, v_s as f32).expect("an ETA");
        assert!(
            (got.seconds_f64() - want_s).abs() <= 1.0e-3,
            "S-ETA {} s vs {want_s:.4} s (1 ms; SEI-007 allows ±1 s)",
            got.seconds_f64()
        );
        // An origin before tick 0 is not lost (signed offset relative to reception).
        let early = alert(8, 1.0, -5.0);
        let e = s_eta_from_alert(&early, v_s as f32).expect("an ETA");
        assert!((e.seconds_f64() - (1.0 - 5.0 + 6.388_766)).abs() <= 1.0e-3);

        let mut f = Fusion::new(&p).expect("fusion");
        let o = f.step(a.received, &FusionEvents::default(), Some(&a));
        assert_eq!(o.state, Protect, "a network alert meets the PROTECT rule");
        assert_eq!((o.source, o.n_feeds), (AlertSource::Network, 1));
        assert_eq!(o.s_eta, Some(got), "report carries the network S-ETA");
        // An update with a new distance moves the ETA.
        let mut upd = a;
        upd.received = Tick::from_seconds_f64(10.5);
        upd.r_epi_km = 40.0;
        let o = f.step(upd.received, &FusionEvents::default(), Some(&upd));
        let want = 10.5 - 2.0 + 40.0_f64.hypot(10.0) / v_s;
        assert!((o.s_eta.expect("eta").seconds_f64() - want).abs() <= 1.0e-3);
        // A cancel for another id is ignored; for the active id it returns to NORMAL.
        let t1 = Tick::from_seconds_f64(11.0);
        let other = NetworkAlert {
            id: 99,
            cancel: true,
            ..a
        };
        assert_eq!(
            f.step(t1, &FusionEvents::default(), Some(&other)).state,
            Protect
        );
        let cancel = NetworkAlert { cancel: true, ..a };
        let o = f.step(t1, &FusionEvents::default(), Some(&cancel));
        assert!(o.cancel, "cancel reported");
        assert_eq!(o.state, Normal, "a cancel returns to NORMAL (SEI-007)");
        assert_eq!(f.s_eta_net, None, "event memory cleared");

        // With local evidence the cancel is reported but does not end the event.
        let mut f = Fusion::new(&p).expect("fusion");
        f.step(a.received, &FusionEvents::default(), Some(&a));
        let local = FusionEvents {
            local_trigger: true,
            local_p: true,
            ..FusionEvents::default()
        };
        let o = f.step(t1, &local, None);
        assert_eq!(
            (o.source, o.n_feeds),
            (AlertSource::Fused, 2),
            "local P + network"
        );
        let o = f.step(t1.plus(200), &local, Some(&cancel));
        assert!(
            o.cancel && o.state == Protect,
            "local evidence outranks the retraction"
        );
        assert_eq!(
            o.source,
            AlertSource::Fused,
            "non-local contribution stays sticky"
        );
    }

    #[test]
    fn network_only_event_runs_to_inspect_and_resets() {
        let p = presets::wall();
        let mut f = Fusion::new(&p).expect("fusion");
        let a = alert(1, 5.0, -1.0);
        let mut t = a.received;
        let quiet = FusionEvents::default();
        assert_eq!(f.step(t, &quiet, Some(&a)).state, Protect);
        // Active until the alert expires (60 s), then 30 s of QUIET, then INSPECT.
        let mut states = [0_u32; 6];
        for _ in 0..(100 * 100) {
            t = t.plus(200);
            let s = f.step(t, &quiet, None).state;
            if let Some(c) = states.get_mut(usize::from(s.as_u8())) {
                *c += 1;
            }
        }
        assert_eq!(
            f.state, Inspect,
            "ends in INSPECT awaiting the manual reset"
        );
        assert!(
            states[4] >= 2999 && states[4] <= 3001,
            "QUIET lasts 30 s: {} releases",
            states[4]
        );
        let r = FusionEvents {
            inspect_reset: true,
            ..quiet
        };
        assert_eq!(f.step(t.plus(200), &r, None).state, Normal);
        assert_eq!((f.alert, f.net_seen), (None, false), "event memory cleared");
    }

    #[test]
    fn local_eta_inverts_wu_zhao() {
        let p = presets::door();
        let le = LocalEta::new(&p);
        assert!(
            (le.c0 + 3.463).abs() < 1e-5
                && (le.c_m - 0.729).abs() < 1e-5
                && (le.c_r - 1.374).abs() < 1e-5,
            "Wu–Zhao coefficients recovered from derive::pd_wu_zhao (Wu & Zhao 2006)"
        );
        assert!(
            (le.sp_s_per_km - 0.119_047_6).abs() < 1e-6,
            "S−P 0.1190 s/km (SEI-012)"
        );
        // τc of an M7 by the regression, Pd of M7 at R 20 km (0.7118 cm) → R = 20 km.
        let tau_m7 = 10.0_f32.powf(TAUC_M_SLOPE * 7.0 + TAUC_M_ICPT);
        let r = le.r_hyp_km(0.007_118, tau_m7).expect("a distance");
        assert!((r - 20.0).abs() < 0.05, "R {r} km vs 20 km");
        let pick = Tick::from_seconds_f64(3.0);
        let eta = le.s_eta(pick, 0.007_118, tau_m7).expect("an ETA");
        let want = 3.0 + 20.0 * 0.119_047_6 - 1.5;
        assert!((eta.seconds_f64() - want).abs() < 0.01, "pick + S−P − σ");
        assert_eq!(le.s_eta(pick, 0.0, tau_m7), None, "no Pd → no estimate");
    }
}
