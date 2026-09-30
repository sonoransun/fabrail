//! Degradation ladder 2oo3 -> 1oo2D -> 1oo1D -> safe state with the diverse L(g) tie-breaker (FAB-EXO-302..304). Owner: X1 (API frozen by W0a).
//!
//! EXOTIC §X13, §S3 (exotic design §2.4; IEC 61508-2 degraded-mode concepts as design
//! targets only). The ladder only ever steps down until a maintenance reset:
//!
//! | Rung | Entered when | Leaves for |
//! |---|---|---|
//! | 2oo3 | power-up / reset (3 healthy) | 1oo2D on one loss; safe state on a simultaneous double loss |
//! | 1oo2D | one channel lost | 1oo1D on a second loss once 1oo2D is established and L(g) is healthy; otherwise safe state |
//! | 1oo1D | a diagnosed second loss | safe state on a third loss or when L(g) fails |
//! | safe state | anything else | nothing (latched until [`Ladder::reset`]) |
//!
//! * **Health.** `healthy[i]` is the voter's non-exclusion AND the channel self-test
//!   ([`crate::vote`]). Voter exclusions are already persistence-filtered (latency in
//!   [t_persist, t_persist + one sample) from fault onset), so the ladder steps on the
//!   same cycle a channel is lost: the step latency from fault onset is bounded by the
//!   same t_persist window (FAB-EXO-302). Lost channels stay lost (monotone), so a
//!   channel that flickers back cannot hide a second loss.
//! * **Common-cause window.** A degraded rung counts as *established* once it has been
//!   held for `t_persist`. A further loss before that, or two channels lost on the same
//!   cycle, is treated as a dependent (common-cause) failure: the survivor shares the
//!   cause, so the ladder demands the safe state whatever the diverse channel says
//!   (FAB-EXO-303). 1oo1D is reachable only through an established 1oo2D whose second
//!   loss the diverse tie-breaker diagnosed.
//! * **Diverse channel.** L(g) acts as a tie-breaker only (FAB-EXO-304, research tier):
//!   it never provides the voted value; its health (`diverse_ok`) only decides whether
//!   the single-channel rung has diagnostics at all.
//!
//! The safe state is STO on the LSM, the brake, and the energised landing with its
//! monitored reserve (EXOTIC §S3; the core SS1 Safe Landing, PHYSICS §18); the landing
//! command is voted separately.

use fabrail_types::Tick;

use crate::types::u8_enum;

u8_enum! {
    /// Redundancy rung, ordered: a larger value is more degraded.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum Redundancy {
        /// Two-out-of-three voting.
        #[default]
        TwoOoThree = 0,
        /// One-out-of-two with diagnostics (diverse tie-breaker).
        OneOoTwoD = 1,
        /// One channel with diagnostics.
        OneOoOneD = 2,
        /// Safe state demanded (STO on the LSM, brake, energised landing).
        SafeState = 3,
    }
}

/// Channel health seen by the ladder.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct LadderInputs {
    /// Health of the three primary channels (voter not excluded, self-test OK).
    pub healthy: [bool; 3],
    /// The diverse inductance channel L(g) is healthy.
    pub diverse_ok: bool,
}

/// The degradation ladder.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Ladder {
    state: Redundancy,
    t_persist_ticks: u32,
    /// Tick the current degraded rung was entered (`None` at 2oo3).
    since: Option<Tick>,
    /// Channels lost so far (monotone until reset).
    lost: [bool; 3],
}

impl Ladder {
    /// A ladder at 2oo3 with persistence (common-cause window) `t_persist_ticks`.
    pub const fn new(t_persist_ticks: u32) -> Ladder {
        Ladder {
            state: Redundancy::TwoOoThree,
            t_persist_ticks,
            since: None,
            lost: [false; 3],
        }
    }

    /// The rung the inputs justify, without latching and without history (the
    /// common-cause rule of [`Ladder::step`] can only be more conservative).
    pub const fn rung_for(inp: &LadderInputs) -> Redundancy {
        let n = inp.healthy[0] as u8 + inp.healthy[1] as u8 + inp.healthy[2] as u8;
        match n {
            3 => Redundancy::TwoOoThree,
            2 => Redundancy::OneOoTwoD,
            1 if inp.diverse_ok => Redundancy::OneOoOneD,
            _ => Redundancy::SafeState,
        }
    }

    /// Steps at `t`; the rung never improves before [`Ladder::reset`].
    pub fn step(&mut self, t: Tick, inp: &LadderInputs) -> Redundancy {
        if self.state == Redundancy::SafeState {
            return self.state;
        }
        for (l, h) in self.lost.iter_mut().zip(inp.healthy) {
            *l = *l || !h;
        }
        let [l0, l1, l2] = self.lost;
        let n = 3 - (l0 as u8 + l1 as u8 + l2 as u8);
        let established = match self.since {
            None => true,
            Some(s) => t.since(s) >= self.t_persist_ticks as u64,
        };
        let next = match self.state {
            Redundancy::TwoOoThree => match n {
                3 => Redundancy::TwoOoThree,
                2 => Redundancy::OneOoTwoD,
                // Two or three channels lost on one cycle: common cause.
                _ => Redundancy::SafeState,
            },
            Redundancy::OneOoTwoD => match n {
                2.. => Redundancy::OneOoTwoD,
                1 if inp.diverse_ok && established => Redundancy::OneOoOneD,
                _ => Redundancy::SafeState,
            },
            Redundancy::OneOoOneD => {
                if n >= 1 && inp.diverse_ok {
                    Redundancy::OneOoOneD
                } else {
                    Redundancy::SafeState
                }
            }
            Redundancy::SafeState => Redundancy::SafeState,
        };
        if next > self.state {
            self.state = next;
            self.since = Some(t);
        }
        self.state
    }

    /// The current rung.
    pub const fn state(&self) -> Redundancy {
        self.state
    }

    /// Tick the current degraded rung was entered (`None` at 2oo3).
    pub const fn entered(&self) -> Option<Tick> {
        self.since
    }

    /// True when the safe state is demanded.
    pub const fn safe_state_demanded(&self) -> bool {
        matches!(self.state, Redundancy::SafeState)
    }

    /// Maintenance reset back to 2oo3.
    pub fn reset(&mut self) {
        self.state = Redundancy::TwoOoThree;
        self.since = None;
        self.lost = [false; 3];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vote::{VoteOut, Voter2oo3, VoterCfg};

    /// t_persist = 10 ms at the 20 kHz base tick, e_th and e_div in gap counts (1 µm):
    /// e_div = 6·e_th for an L(g) channel 5–10× noisier (exotic design §2.4).
    const P: u32 = 200;
    const E_TH: i32 = 50;
    const E_DIV: i32 = 300;

    /// The voter and the ladder composed as on the safety channel.
    struct Chain {
        voter: Voter2oo3,
        ladder: Ladder,
    }

    impl Chain {
        fn new() -> Chain {
            Chain {
                voter: Voter2oo3::with_diverse_tol(
                    VoterCfg {
                        e_th: E_TH,
                        t_persist_ticks: P,
                    },
                    E_DIV,
                ),
                ladder: Ladder::new(P),
            }
        }

        fn step(
            &mut self,
            t: u64,
            x: [i32; 3],
            self_test: [bool; 3],
            diverse: Option<i32>,
        ) -> (VoteOut, Redundancy) {
            let out = self.voter.step_diverse(Tick(t), x, diverse);
            let healthy = [0, 1, 2].map(|i| !out.excluded[i] && self_test[i]);
            let rung = self.ladder.step(
                Tick(t),
                &LadderInputs {
                    healthy,
                    diverse_ok: diverse.is_some(),
                },
            );
            (out, rung)
        }
    }

    /// Runs `chain` from `t0` until the rung changes or `t_end`; returns (tick, rung).
    fn run_until_change(
        chain: &mut Chain,
        t0: u64,
        t_end: u64,
        mut sample: impl FnMut(u64) -> ([i32; 3], [bool; 3], Option<i32>),
    ) -> (u64, Redundancy) {
        let before = chain.ladder.state();
        for t in t0..t_end {
            let (x, st, d) = sample(t);
            let (_, rung) = chain.step(t, x, st, d);
            if rung != before {
                return (t, rung);
            }
        }
        (t_end, chain.ladder.state())
    }

    const OK: [bool; 3] = [true; 3];

    // req: FAB-EXO-302
    #[test]
    fn degradation_sequence() {
        let truth = 12_000; // 12 mm gap in µm counts
        let mut c = Chain::new();
        // Healthy: 2oo3 for 1000 ticks with in-band noise (±E_TH/2) and a noisy L(g).
        for t in 0..1000u64 {
            let n = (t % 7) as i32 * 3 - 9;
            let (_, rung) = c.step(t, [truth + n, truth - n, truth], OK, Some(truth + 4 * n));
            assert_eq!(
                rung,
                Redundancy::TwoOoThree,
                "no step while healthy (t {t})"
            );
        }

        // Fault 1: channel 0 bias step at t = 1000 → 1oo2D within [P, P + 1 sample).
        let f1 = 1000u64;
        let (t1, r1) = run_until_change(&mut c, f1, f1 + 10 * P as u64, |_| {
            ([truth + 5_000, truth, truth], OK, Some(truth))
        });
        assert_eq!(r1, Redundancy::OneOoTwoD, "first loss → 1oo2D");
        assert!(
            t1 >= f1 + P as u64 && t1 < f1 + P as u64 + 1,
            "1oo2D at {t1}: must be within [onset + t_persist, + 1 sample) = [{}, {}) (FAB-EXO-302)",
            f1 + P as u64,
            f1 + P as u64 + 1
        );

        // Fault 2, after 1oo2D is established: channel 1 stuck; L(g) sides with
        // channel 2, so channel 1 is excluded → 1oo1D within the same bound.
        let f2 = t1 + 2 * P as u64;
        let (t2, r2) = run_until_change(&mut c, f2, f2 + 10 * P as u64, |_| {
            ([truth + 5_000, 0, truth], OK, Some(truth + 40))
        });
        assert_eq!(r2, Redundancy::OneOoOneD, "diagnosed second loss → 1oo1D");
        assert!(
            t2 >= f2 + P as u64 && t2 < f2 + P as u64 + 1,
            "1oo1D at {t2}: must be within [{}, {}) (FAB-EXO-302)",
            f2 + P as u64,
            f2 + P as u64 + 1
        );
        assert_eq!(c.voter.excluded(), [true, true, false]);

        // Fault 3: channel 2 drifts away from L(g) → excluded → safe state, same bound.
        let f3 = t2 + 3 * P as u64;
        let (t3, r3) = run_until_change(&mut c, f3, f3 + 10 * P as u64, |_| {
            ([truth + 5_000, 0, truth + 2_000], OK, Some(truth))
        });
        assert_eq!(r3, Redundancy::SafeState, "third loss → safe state");
        assert!(
            t3 >= f3 + P as u64 && t3 < f3 + P as u64 + 1,
            "safe state at {t3}: must be within [{}, {}) (FAB-EXO-302)",
            f3 + P as u64,
            f3 + P as u64 + 1
        );

        // The ladder never improves before a reset, whatever the inputs.
        for t in t3..t3 + 100 {
            let (_, rung) = c.step(t, [truth; 3], OK, Some(truth));
            assert_eq!(rung, Redundancy::SafeState);
        }
        c.ladder.reset();
        c.voter.reset();
        assert_eq!(c.ladder.state(), Redundancy::TwoOoThree);

        // Losing the diverse channel in 1oo1D is itself a step to the safe state
        // (no diagnostics left); it is immediate, the diverse health being its own
        // diagnosis.
        let mut l = Ladder::new(P);
        let h = |healthy, diverse_ok| LadderInputs {
            healthy,
            diverse_ok,
        };
        assert_eq!(
            l.step(Tick(0), &h([false, true, true], true)),
            Redundancy::OneOoTwoD
        );
        assert_eq!(
            l.step(Tick(P as u64), &h([false, false, true], true)),
            Redundancy::OneOoOneD
        );
        assert_eq!(
            l.step(Tick(P as u64 + 1), &h([false, false, true], false)),
            Redundancy::SafeState
        );
    }

    // req: FAB-EXO-303
    #[test]
    fn common_cause_reaches_safe_state() {
        let truth = 8_000;
        // (a) β common cause detected by the channel self-tests (dropout / saturation on
        // two channels in the same cycle): safe state on that cycle, even with a healthy
        // L(g) and a healthy third channel.
        let mut c = Chain::new();
        let (_, rung) = c.step(0, [truth; 3], OK, Some(truth));
        assert_eq!(rung, Redundancy::TwoOoThree);
        let (_, rung) = c.step(1, [0, i32::MAX, truth], [false, false, true], Some(truth));
        assert_eq!(
            rung,
            Redundancy::SafeState,
            "simultaneous double loss → safe state"
        );

        // (b) All three hit (common supply failure): safe state immediately.
        let mut c = Chain::new();
        let (_, rung) = c.step(0, [0; 3], [false; 3], Some(truth));
        assert_eq!(rung, Redundancy::SafeState);

        // (c) Two value faults with different onsets inside the common-cause window:
        // the second loss lands before 1oo2D is established → safe state, although L(g)
        // could have broken the tie.
        let mut c = Chain::new();
        let mut first = None;
        let mut safe_at = None;
        for t in 0..10 * P as u64 {
            let x0 = if t >= 100 { truth + 9_000 } else { truth };
            let x1 = if t >= 100 + P as u64 / 2 {
                truth - 9_000
            } else {
                truth
            };
            let (_, rung) = c.step(t, [x0, x1, truth], OK, Some(truth));
            if rung == Redundancy::OneOoTwoD && first.is_none() {
                first = Some(t);
            }
            if rung == Redundancy::SafeState {
                safe_at = Some(t);
                break;
            }
        }
        assert_eq!(first, Some(100 + P as u64), "first loss → 1oo2D");
        let safe_at = safe_at.expect("second loss inside the window must reach the safe state");
        assert!(
            safe_at <= 100 + 2 * P as u64,
            "safe state at {safe_at}: bound t_ladder = onset + 2·t_persist ({})",
            100 + 2 * P as u64
        );

        // (d) Value common cause: two channels jump together to the same wrong value, so
        // they outvote the healthy one. L(g) agrees with the outlier and contradicts the
        // majority → veto → every channel excluded → safe state within t_persist.
        let mut c = Chain::new();
        let mut safe_at = None;
        for t in 0..10 * P as u64 {
            let bad = if t >= 50 { truth + 2_000 } else { truth };
            let (out, rung) = c.step(t, [bad, bad, truth], OK, Some(truth + 30));
            assert_ne!(
                rung,
                Redundancy::OneOoTwoD,
                "the voter must not degrade onto the common-cause pair"
            );
            if rung == Redundancy::SafeState {
                assert_eq!(out.n_valid, 0);
                safe_at = Some(t);
                break;
            }
        }
        assert_eq!(safe_at, Some(50 + P as u64), "veto at onset + t_persist");

        // (e) Honest limit: without a healthy L(g) the value common cause is invisible to
        // voting (the β·λ_DU term of the PFH, EXOTIC §X13): the healthy channel is
        // outvoted and the ladder degrades to 1oo2D on the faulty pair.
        let mut c = Chain::new();
        let mut rung = Redundancy::TwoOoThree;
        for t in 0..3 * P as u64 {
            let bad = if t >= 50 { truth + 2_000 } else { truth };
            rung = c.step(t, [bad, bad, truth], OK, None).1;
        }
        assert_eq!(rung, Redundancy::OneOoTwoD);
        assert_eq!(c.voter.excluded(), [false, false, true]);
    }

    // req: FAB-EXO-304
    #[test]
    fn diverse_tiebreaker() {
        let truth = 10_000;
        // A distinctive L(g) value that no primary ever reads.
        let marker = truth + E_DIV / 2 + 7;

        // (1) 2oo3: L(g) never sets the output and never excludes an agreeing channel,
        // even when it is far off.
        let mut c = Chain::new();
        for t in 0..5 * P as u64 {
            let d = if t % 2 == 0 {
                Some(marker)
            } else {
                Some(-1_000_000)
            };
            let (out, rung) = c.step(t, [truth, truth + 10, truth - 10], OK, d);
            assert_eq!(out.value, truth, "2oo3 output is the MVS, not L(g)");
            assert_eq!(out.n_valid, 3);
            assert_eq!(rung, Redundancy::TwoOoThree);
        }

        // (2) 1oo2D with an agreeing pair: L(g) far off changes nothing.
        let mut c = Chain::new();
        for t in 0..=P as u64 {
            c.step(t, [truth + 9_000, truth, truth], OK, Some(truth));
        }
        assert_eq!(c.ladder.state(), Redundancy::OneOoTwoD);
        for t in P as u64 + 1..6 * P as u64 {
            let (out, rung) = c.step(t, [0, truth, truth + 20], OK, Some(-1_000_000));
            assert_eq!(out.value, truth + 10, "midpoint of the agreeing pair");
            assert_eq!(out.n_valid, 2, "agreeing pair never excluded by L(g)");
            assert_eq!(rung, Redundancy::OneOoTwoD);
        }

        // (3) 1oo2D tie: L(g) sides with channel 2; channel 1 is excluded, the output is
        // channel 2's own reading (never the L(g) value), and the ladder reaches 1oo1D.
        let t0 = 6 * P as u64;
        let mut picked = None;
        for t in t0..t0 + 3 * P as u64 {
            let (out, rung) = c.step(t, [0, truth - 3_000, truth + 20], OK, Some(marker));
            assert_ne!(out.value, marker, "L(g) is never the output (FAB-EXO-304)");
            if rung == Redundancy::OneOoOneD {
                picked = Some(out);
                break;
            }
        }
        let out = picked.expect("the tie-breaker resolves the pair");
        assert_eq!(out.excluded, [true, true, false]);
        assert_eq!(out.value, truth + 20);

        // (4) Ambiguous tie-breaker: a pair discrepancy smaller than e_div (L(g) agrees
        // with both) cannot be resolved → both excluded → safe state.
        let mut c = Chain::new();
        for t in 0..=P as u64 {
            c.step(t, [truth + 9_000, truth, truth], OK, Some(truth));
        }
        let mut rung = c.ladder.state();
        for t in P as u64 + 1..4 * P as u64 {
            rung = c
                .step(t, [0, truth, truth + 2 * E_TH], OK, Some(truth + E_TH))
                .1;
            if rung == Redundancy::SafeState {
                break;
            }
        }
        assert_eq!(rung, Redundancy::SafeState, "ambiguous L(g) → safe state");

        // (5) L(g) agreeing with neither, or unavailable → safe state.
        for d in [Some(truth + 50_000), None] {
            let mut c = Chain::new();
            for t in 0..=P as u64 {
                c.step(t, [truth + 9_000, truth, truth], OK, Some(truth));
            }
            let mut rung = c.ladder.state();
            for t in P as u64 + 1..6 * P as u64 {
                rung = c.step(t, [0, truth - 3_000, truth + 3_000], OK, d).1;
                if rung == Redundancy::SafeState {
                    break;
                }
            }
            assert_eq!(
                rung,
                Redundancy::SafeState,
                "L(g) {d:?} cannot break the tie"
            );
        }
    }
}
