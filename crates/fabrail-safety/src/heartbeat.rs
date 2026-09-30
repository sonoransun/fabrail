//! Core heartbeat sequence and deadline supervision (SAF-013). Owner: A5.
//!
//! [`Heartbeat`]: the core's fast-group sequence number (`FastStatus::seq`) must change
//! at least every `max_ticks` base ticks (3 periods, `SafetyLimits::heartbeat_max_ticks`;
//! core design §C.4); otherwise HeartbeatLost (STO). The check is also a deadline
//! supervision: a heartbeat that does change, but only after more than `max_ticks`
//! since its previous change (the fast group missed its releases), reports stale once.
//! Integer only (PHYSICS §18).
#![deny(clippy::float_arithmetic)]

use fabrail_types::Tick;

/// Heartbeat staleness supervision.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Heartbeat {
    /// Staleness limit, base ticks.
    pub max_ticks: u32,
    /// Last heartbeat value seen.
    pub last_hb: u32,
    /// Tick at which it last changed.
    pub last_change: Tick,
    /// A heartbeat has been seen.
    pub primed: bool,
}

impl Heartbeat {
    /// Supervision with a staleness limit in base ticks.
    pub const fn new(max_ticks: u32) -> Heartbeat {
        Heartbeat {
            max_ticks,
            last_hb: 0,
            last_change: Tick(0),
            primed: false,
        }
    }

    /// One check at tick `t` with heartbeat `hb`; true when stale (unchanged for more
    /// than `max_ticks`, or changed only after a deadline miss longer than that).
    pub fn step(&mut self, t: Tick, hb: u32) -> bool {
        let late = self.max_ticks > 0 && t.since(self.last_change) > self.max_ticks as u64;
        if !self.primed || hb != self.last_hb {
            let was_primed = self.primed;
            self.primed = true;
            self.last_hb = hb;
            self.last_change = t;
            return was_primed && late;
        }
        late
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // req: FAB-SAF-013
    #[test]
    fn stale_trips() {
        // 3 supervisor periods = 60 base ticks (core design §C.4), fast rate 10 kHz.
        let mut h = Heartbeat::new(60);
        let mut hb = 0u32;
        for k in 0..1_000u64 {
            hb = hb.wrapping_add(1);
            assert!(
                !h.step(Tick(2 * k), hb),
                "a heartbeat every fast period is fresh"
            );
        }
        // Frozen heartbeat: fresh up to 60 ticks, stale beyond (T, SAF-013).
        let t0 = 2 * 999;
        assert!(
            !h.step(Tick(t0 + 60), hb),
            "60 ticks unchanged is within the limit"
        );
        assert!(
            h.step(Tick(t0 + 62), hb),
            "(T) unchanged for > 3 periods is stale, SAF-013"
        );
        assert!(h.step(Tick(t0 + 100), hb), "stays stale");
        assert!(
            h.step(Tick(t0 + 102), hb.wrapping_add(1)),
            "the late change itself is reported"
        );
        assert!(
            !h.step(Tick(t0 + 104), hb.wrapping_add(2)),
            "an on-time change clears it"
        );
        // Deadline miss: the heartbeat changes, but 80 ticks after the previous change.
        let t1 = t0 + 104;
        assert!(
            h.step(Tick(t1 + 80), hb.wrapping_add(3)),
            "(T) late heartbeat, SAF-013"
        );
        assert!(!h.step(Tick(t1 + 82), hb.wrapping_add(4)), "on time again");
        // Wrapping sequence numbers are fine.
        let mut w = Heartbeat::new(6);
        assert!(!w.step(Tick(0), u32::MAX));
        assert!(!w.step(Tick(2), 0), "wrap is a change");
        // Unconfigured limit never reports.
        let mut z = Heartbeat::new(0);
        assert!(!z.step(Tick(0), 1) && !z.step(Tick(1_000_000), 1));
    }
}
