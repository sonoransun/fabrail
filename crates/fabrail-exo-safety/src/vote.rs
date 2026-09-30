//! Mid-value-select 2oo3 voter with discrepancy exclusion (FAB-EXO-301). Owner: X1 (API frozen by W0a).
//!
//! EXOTIC §X13 (exotic design §2.4; IEC 61508-2 architecture concepts used as design
//! targets only, never a compliance claim). The 2oo3 vote is on the shutdown path (LSM
//! STO, brake), not on the force loop; the landing command is voted separately
//! (EXOTIC §S3).
//!
//! * **Mid-value select** mvs(a, b, c) = max(min(a, b), min(max(a, b), c)): the output
//!   always lies between two of the inputs, so a single faulty channel, whatever its
//!   value, cannot move the output outside the range of the two healthy ones. With the
//!   healthy spread within e_th the output moves by at most e_th (FAB-EXO-301).
//! * **Discrepancy exclusion.** A channel whose residual |x_i − x_mvs| exceeds `e_th` on
//!   every sample for `t_persist` is excluded (latched until [`Voter2oo3::reset`]); a
//!   residual that returns within `e_th` restarts the timer, so transients shorter than
//!   `t_persist` never exclude. Exclusion latency is therefore in [t_persist,
//!   t_persist + one sample). The degradation ladder ([`crate::ladder`]) consumes the
//!   exclusions.
//! * **Two valid channels (1oo2D).** The output is the midpoint while the pair agrees
//!   within `e_th`; while it disagrees the output holds the last agreed value (age at most
//!   `t_persist`). A survivor already off the median when the first channel was excluded
//!   keeps its onset. A disagreement that persists for `t_persist` cannot be resolved by
//!   the pair itself: the diverse inductance channel L(g) breaks the tie (the channel it
//!   agrees with, within `e_div`, stays); if it agrees with both, with neither, or is
//!   unavailable, both channels are excluded and the ladder demands the safe state.
//! * **One valid channel (1oo1D).** The diagnostics are the diverse compare: a residual
//!   |x − L(g)| above `e_div` for `t_persist` excludes the last channel.
//! * **The diverse channel is a tie-breaker only** (FAB-EXO-304, research tier): it never
//!   becomes the output value, never excludes a channel while the primaries agree, and
//!   can otherwise only pick between primaries or push toward the safe state. The one
//!   2oo3 case it acts on is a common-cause veto: when a channel is about to be excluded
//!   and L(g) agrees with that outlier but not with the majority, the majority is
//!   presumed hit by a common cause and every channel is excluded (safe state).
//!
//! L(g) = N²·μ0·A/(2g) + L_leak (the magnetic element's inductance, PHYSICS §2; sensing
//! and quantisation, PHYSICS §13) is estimated from the PWM current ripple and is about
//! 5–10× noisier than the eddy-current probe \[UNCERTAIN\] (exotic design §2.4), so
//! `e_div` is several times `e_th`. A common-cause bias on two primaries smaller than
//! `e_div` is invisible to every channel here; the PFH calculator carries it in the β
//! terms ([`crate::pfh`]).
//!
//! All arithmetic is integer: residuals are formed in i64, so no input (including
//! `i32::MIN` / `i32::MAX` from a saturated channel) can overflow.

use fabrail_types::Tick;

/// Mid-value select of three integer channels.
pub const fn mvs(a: i32, b: i32, c: i32) -> i32 {
    let lo = if a < b { a } else { b };
    let hi = if a < b { b } else { a };
    let m = if hi < c { hi } else { c };
    if lo > m { lo } else { m }
}

/// |a − b| formed in i64 (total for every i32 pair).
const fn dev(a: i32, b: i32) -> u64 {
    (a as i64 - b as i64).unsigned_abs()
}

/// Midpoint ⌊(a + b)/2⌋ of two channels, exact in i64 and always within [min, max].
const fn mid2(a: i32, b: i32) -> i32 {
    ((a as i64 + b as i64) >> 1) as i32
}

/// A non-negative threshold as u64 (a negative configured threshold acts as 0, the
/// strict direction).
const fn thr(e: i32) -> u64 {
    if e < 0 { 0 } else { e as u64 }
}

/// Channel `i` of `x` without indexing (`i` ≥ 2 selects channel 2).
const fn at(x: [i32; 3], i: usize) -> i32 {
    let [a, b, c] = x;
    match i {
        0 => a,
        1 => b,
        _ => c,
    }
}

/// Voter configuration (integer sensor units).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct VoterCfg {
    /// Discrepancy threshold e_th, sensor counts.
    pub e_th: i32,
    /// Persistence before exclusion, base ticks.
    pub t_persist_ticks: u32,
}

/// One vote.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct VoteOut {
    /// Voted value (mid value of the valid channels).
    pub value: i32,
    /// Channels excluded so far (latched until [`Voter2oo3::reset`]).
    pub excluded: [bool; 3],
    /// Channels still valid.
    pub n_valid: u8,
}

/// 2oo3 mid-value-select voter with discrepancy exclusion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Voter2oo3 {
    cfg: VoterCfg,
    excluded: [bool; 3],
    over_since: [Option<Tick>; 3],
    /// Diverse-channel agreement tolerance e_div (0 until configured: the diverse channel
    /// then resolves nothing and every tie ends in the safe state).
    e_div: i32,
    /// Onset of a 1oo2 pair discrepancy.
    pair_since: Option<Tick>,
    /// Onset of a 1oo1D primary-versus-diverse discrepancy.
    div_since: Option<Tick>,
    /// Last voted value (held while no agreed value exists).
    last: i32,
}

impl Voter2oo3 {
    /// A voter with all channels valid and no diverse tolerance (e_div = 0).
    pub const fn new(cfg: VoterCfg) -> Voter2oo3 {
        Voter2oo3::with_diverse_tol(cfg, 0)
    }

    /// A voter with all channels valid and the diverse tie-breaker tolerance `e_div`
    /// (sensor counts; L(g) is 5–10× noisier than the primaries, exotic design §2.4).
    pub const fn with_diverse_tol(cfg: VoterCfg, e_div: i32) -> Voter2oo3 {
        Voter2oo3 {
            cfg,
            excluded: [false; 3],
            over_since: [None; 3],
            e_div,
            pair_since: None,
            div_since: None,
            last: 0,
        }
    }

    /// The configuration.
    pub const fn cfg(&self) -> VoterCfg {
        self.cfg
    }

    /// The diverse tie-breaker tolerance e_div.
    pub const fn diverse_tol(&self) -> i32 {
        self.e_div
    }

    /// Channels excluded so far.
    pub const fn excluded(&self) -> [bool; 3] {
        self.excluded
    }

    /// Number of valid channels.
    pub const fn n_valid(&self) -> u8 {
        let [a, b, c] = self.excluded;
        3 - (a as u8 + b as u8 + c as u8)
    }

    /// Votes the three channel samples taken at `t` (no diverse channel available).
    pub fn step(&mut self, t: Tick, x: [i32; 3]) -> VoteOut {
        self.step_diverse(t, x, None)
    }

    /// Votes the three channel samples taken at `t`; `diverse` is the L(g) estimate in
    /// the primaries' units when the diverse channel is healthy (tie-breaker only).
    pub fn step_diverse(&mut self, t: Tick, x: [i32; 3], diverse: Option<i32>) -> VoteOut {
        match self.excluded {
            [false, false, false] => self.step3(t, x, diverse),
            [true, false, false] => self.step2(t, x, (1, 2), diverse),
            [false, true, false] => self.step2(t, x, (0, 2), diverse),
            [false, false, true] => self.step2(t, x, (0, 1), diverse),
            [false, true, true] => self.step1(t, x, 0, diverse),
            [true, false, true] => self.step1(t, x, 1, diverse),
            [true, true, false] => self.step1(t, x, 2, diverse),
            // No valid channel: hold the last value; the ladder demands the safe state.
            [true, true, true] => {}
        }
        VoteOut {
            value: self.last,
            excluded: self.excluded,
            n_valid: self.n_valid(),
        }
    }

    /// 2oo3: mid-value select, residual persistence, optional common-cause veto.
    fn step3(&mut self, t: Tick, x: [i32; 3], diverse: Option<i32>) {
        let m = mvs(at(x, 0), at(x, 1), at(x, 2));
        self.last = m;
        let e_th = thr(self.cfg.e_th);
        let persist = self.cfg.t_persist_ticks as u64;
        let mut cand = [false; 3];
        for ((since, c), xi) in self.over_since.iter_mut().zip(cand.iter_mut()).zip(x) {
            if dev(xi, m) > e_th {
                let s = *since.get_or_insert(t);
                *c = t.since(s) >= persist;
            } else {
                *since = None;
            }
        }
        if cand == [false; 3] {
            return;
        }
        // Common-cause veto (tie-breaker, not a vote): L(g) agrees with an outlier and
        // contradicts the majority, so the majority is suspect; exclude everything.
        let e_div = thr(self.e_div);
        let veto = match diverse {
            Some(d) => {
                let majority_ok = dev(m, d) <= e_div;
                let outlier_ok = cand.iter().zip(x).any(|(&c, xi)| c && dev(xi, d) <= e_div);
                !majority_ok && outlier_ok
            }
            None => false,
        };
        // A survivor that was already off the median keeps its onset as the pair
        // discrepancy onset, so a second fault's latency is still counted from its own
        // onset (and the ladder's common-cause window sees it).
        let mut inherited: Option<Tick> = None;
        for ((e, c), since) in self.excluded.iter_mut().zip(cand).zip(self.over_since) {
            *e = veto || c;
            if !*e {
                inherited = match (inherited, since) {
                    (Some(a), Some(b)) => Some(if a < b { a } else { b }),
                    (a, b) => a.or(b),
                };
            }
        }
        self.pair_since = inherited;
        self.over_since = [None; 3];
    }

    /// 1oo2D: pairwise compare; the diverse channel breaks a persistent tie.
    fn step2(&mut self, t: Tick, x: [i32; 3], (i, j): (usize, usize), diverse: Option<i32>) {
        let (xi, xj) = (at(x, i), at(x, j));
        if dev(xi, xj) <= thr(self.cfg.e_th) {
            self.pair_since = None;
            self.last = mid2(xi, xj);
            return;
        }
        // Disagreement: hold the last agreed value until it is resolved.
        let s = *self.pair_since.get_or_insert(t);
        if t.since(s) < self.cfg.t_persist_ticks as u64 {
            return;
        }
        self.pair_since = None;
        let e_div = thr(self.e_div);
        let (ok_i, ok_j) = match diverse {
            Some(d) => (dev(xi, d) <= e_div, dev(xj, d) <= e_div),
            None => (false, false),
        };
        let (drop_i, drop_j) = match (ok_i, ok_j) {
            (true, false) => (false, true),
            (false, true) => (true, false),
            // Ambiguous or no tie-breaker: neither can be trusted.
            _ => (true, true),
        };
        if drop_i {
            self.exclude(i);
        } else {
            self.last = xi;
        }
        if drop_j {
            self.exclude(j);
        } else {
            self.last = xj;
        }
    }

    /// 1oo1D: the remaining channel, diagnosed against the diverse channel.
    fn step1(&mut self, t: Tick, x: [i32; 3], k: usize, diverse: Option<i32>) {
        let xk = at(x, k);
        self.last = xk;
        match diverse {
            Some(d) if dev(xk, d) > thr(self.e_div) => {
                let s = *self.div_since.get_or_insert(t);
                if t.since(s) >= self.cfg.t_persist_ticks as u64 {
                    self.div_since = None;
                    self.exclude(k);
                }
            }
            _ => self.div_since = None,
        }
    }

    fn exclude(&mut self, i: usize) {
        if let Some(e) = self.excluded.get_mut(i) {
            *e = true;
        }
    }

    /// Re-admits every channel (maintenance reset).
    pub fn reset(&mut self) {
        self.excluded = [false; 3];
        self.over_since = [None; 3];
        self.pair_since = None;
        self.div_since = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    #[test]
    fn mvs_is_the_median() {
        for (a, b, c) in [
            (1, 2, 3),
            (3, 2, 1),
            (2, 3, 1),
            (5, 5, -9),
            (i32::MIN, 0, i32::MAX),
        ] {
            let mut v = [a, b, c];
            v.sort_unstable();
            assert_eq!(mvs(a, b, c), v[1], "{a} {b} {c}");
        }
    }

    /// A single-channel fault model (exotic design §2.4 fault list).
    #[derive(Clone, Copy, Debug)]
    enum Fault {
        StuckAt(i32),
        BiasStep(i32),
        DriftRamp(i32),
        NoiseBurst(i32),
        Dropout,
        SaturationHigh,
        SaturationLow,
    }

    impl Fault {
        fn apply(self, healthy: i32, k: u64, seed: u64) -> i32 {
            match self {
                Fault::StuckAt(v) => v,
                Fault::BiasStep(b) => healthy.saturating_add(b),
                Fault::DriftRamp(r) => healthy.saturating_add(r.saturating_mul(k as i32)),
                Fault::NoiseBurst(a) => {
                    let h = seed.wrapping_add(k).wrapping_mul(0x9E37_79B9_7F4A_7C15) >> 33;
                    healthy.saturating_add(
                        ((h % (2 * a.unsigned_abs() as u64 + 1)) as i64 - a.unsigned_abs() as i64)
                            as i32,
                    )
                }
                Fault::Dropout => 0,
                Fault::SaturationHigh => i32::MAX,
                Fault::SaturationLow => i32::MIN,
            }
        }
    }

    fn fault_strategy() -> impl Strategy<Value = Fault> {
        prop_oneof![
            any::<i32>().prop_map(Fault::StuckAt),
            any::<i32>().prop_map(Fault::BiasStep),
            (-1_000_000i32..=1_000_000).prop_map(Fault::DriftRamp),
            (0i32..=1_000_000_000).prop_map(Fault::NoiseBurst),
            Just(Fault::Dropout),
            Just(Fault::SaturationHigh),
            Just(Fault::SaturationLow),
        ]
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(512))]

        // req: FAB-EXO-301
        #[test]
        fn mvs_single_fault_bound(
            truth in -1_000_000_000i32..=1_000_000_000,
            e_th in 1i32..=10_000,
            noise in proptest::collection::vec((-1.0f64..=1.0, -1.0f64..=1.0, -1.0f64..=1.0), 64),
            faulty in 0usize..3,
            fault in fault_strategy(),
            onset in 0u64..32,
            persist in 0u32..16,
            seed in any::<u64>(),
        ) {
            // Healthy channels read truth + n with abs(n) ≤ e_th/2, so their spread is
            // within e_th (the voter's design assumption, EXOTIC §X13).
            let half = e_th / 2;
            let cfg = VoterCfg { e_th, t_persist_ticks: persist };
            let mut v = Voter2oo3::new(cfg);
            for (k, (n0, n1, n2)) in noise.iter().enumerate() {
                let k = k as u64;
                let h = [n0, n1, n2].map(|n| truth + (n * half as f64) as i32);
                let mut x = h;
                if k >= onset {
                    x[faulty] = fault.apply(h[faulty], k - onset, seed);
                }
                let out = v.step(Tick(k), x);
                // (1) Static bound: the output never leaves the healthy range, so it
                // moves by at most e_th against the fault-free vote (tolerance e_th,
                // FAB-EXO-301).
                let healthy: Vec<i32> = (0..3).filter(|&i| i != faulty).map(|i| h[i]).collect();
                let (lo, hi) = (healthy[0].min(healthy[1]), healthy[0].max(healthy[1]));
                prop_assert!(
                    out.value >= lo && out.value <= hi,
                    "tick {}: output {} outside healthy range [{}, {}] ({:?} on ch {})",
                    k, out.value, lo, hi, fault, faulty
                );
                let clean = mvs(h[0], h[1], h[2]);
                prop_assert!(
                    dev(out.value, clean) <= e_th as u64,
                    "tick {}: |vote − fault-free vote| = {} > e_th {} (FAB-EXO-301)",
                    k, dev(out.value, clean), e_th
                );
                // (2) A healthy channel is never excluded.
                for i in (0..3).filter(|&i| i != faulty) {
                    prop_assert!(!out.excluded[i], "healthy ch {} excluded at tick {}", i, k);
                }
                if out.excluded[faulty] {
                    break;
                }
            }
        }
    }

    #[test]
    fn persistent_fault_excluded_within_bound_and_transient_ignored() {
        let cfg = VoterCfg {
            e_th: 100,
            t_persist_ticks: 10,
        };
        // A burst shorter than t_persist never excludes.
        let mut v = Voter2oo3::new(cfg);
        for k in 0..100u64 {
            let bad = if (20..29).contains(&k) { 5_000 } else { 0 };
            let out = v.step(Tick(k), [bad, 0, 3]);
            assert_eq!(
                out.n_valid, 3,
                "9-tick burst < t_persist 10 excluded at {k}"
            );
        }
        // A persistent fault is excluded exactly t_persist after onset.
        let mut v = Voter2oo3::new(cfg);
        let mut excluded_at = None;
        for k in 0..100u64 {
            let bad = if k >= 20 { 5_000 } else { 0 };
            let out = v.step(Tick(k), [0, bad, 3]);
            if out.excluded[1] && excluded_at.is_none() {
                excluded_at = Some(k);
            }
        }
        assert_eq!(excluded_at, Some(30), "exclusion at onset + t_persist");
    }

    #[test]
    fn pair_discrepancy_without_tiebreaker_excludes_both() {
        let cfg = VoterCfg {
            e_th: 10,
            t_persist_ticks: 4,
        };
        let mut v = Voter2oo3::new(cfg);
        // Exclude channel 0 first.
        for k in 0..5u64 {
            v.step(Tick(k), [1_000, 0, 0]);
        }
        assert_eq!(v.excluded(), [true, false, false]);
        // The pair agrees: midpoint.
        let out = v.step(Tick(5), [0, 4, 8]);
        assert_eq!(out.value, 6);
        // The pair disagrees: the last agreed value is held, then both are excluded.
        for k in 6..10u64 {
            let out = v.step(Tick(k), [0, 0, 500]);
            assert_eq!(out.value, 6, "held while unresolved");
            assert_eq!(out.n_valid, 2);
        }
        let out = v.step(Tick(10), [0, 0, 500]);
        assert_eq!(
            out.n_valid, 0,
            "unresolvable 1oo2 discrepancy → no valid channel"
        );
        v.reset();
        assert_eq!(v.n_valid(), 3);
    }
}
