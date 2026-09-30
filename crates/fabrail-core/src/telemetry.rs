//! Telemetry frame assembly and the fixed-capacity event ring (64 events). Owner: A4.
//!
//! Frozen API (W0): [`EventRing`] (fixed capacity [`EVENT_RING_CAP`], overwrite-oldest,
//! allocation-free) and [`assemble_frame`] (the 1 kHz `TelemetryFrame` from the latest
//! fast status, directive, supervisor status, seismic report and monitor verdict;
//! TECHNICAL_DESIGN §12, FW-011). The frame layout itself is frozen in
//! `fabrail_types::telemetry`.

use fabrail_types::{Event, RailClass, SafetyVerdict, TelemetryFrame};

use crate::msg::{Directive, FastStatus, SeismicReport, SupStatus};

/// Event ring capacity. Source: core design §B.1 / telemetry header ("the event ring
/// holds 64 entries"): at the 100 Hz telemetry drain this covers a 64-event burst.
pub const EVENT_RING_CAP: usize = 64;

/// Fixed-capacity FIFO of events; pushing into a full ring overwrites the oldest and
/// counts the loss.
#[derive(Clone, Debug)]
pub struct EventRing {
    buf: [Event; EVENT_RING_CAP],
    head: u16,
    len: u16,
    /// Events overwritten before they were drained.
    pub dropped: u32,
}

impl Default for EventRing {
    fn default() -> EventRing {
        EventRing::new()
    }
}

impl EventRing {
    /// An empty ring.
    pub const fn new() -> EventRing {
        EventRing {
            buf: [Event {
                tick: fabrail_types::Tick(0),
                kind: fabrail_types::EventKind::LevState,
                a: 0.0,
                b: 0.0,
            }; EVENT_RING_CAP],
            head: 0,
            len: 0,
            dropped: 0,
        }
    }

    /// Number of queued events.
    #[inline]
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// True when no event is queued.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Queues `e`, overwriting the oldest event when full.
    pub fn push(&mut self, e: Event) {
        let cap = EVENT_RING_CAP as u16;
        let tail = (self.head + self.len) % cap;
        if let Some(slot) = self.buf.get_mut(tail as usize) {
            *slot = e;
        }
        if self.len == cap {
            self.head = (self.head + 1) % cap;
            self.dropped = self.dropped.saturating_add(1);
        } else {
            self.len += 1;
        }
    }

    /// Dequeues the oldest event.
    pub fn pop(&mut self) -> Option<Event> {
        if self.len == 0 {
            return None;
        }
        let e = self.buf.get(self.head as usize).copied();
        self.head = (self.head + 1) % EVENT_RING_CAP as u16;
        self.len -= 1;
        e
    }
}

/// Assembles one telemetry frame. `seq` is the frame sequence number.
///
/// Demand is the higher of the monitor's and the supervisor's (the core never lowers a
/// monitor demand, SAF-016); faults are the union of the supervisor's latched set, the
/// fast group's and the monitor's; flags map hold-to-run, presence, mains, referenced,
/// obstruction and confirmed intent (`TelemetryFrame::FLAG_*`).
pub fn assemble_frame(
    seq: u32,
    class: RailClass,
    s: &FastStatus,
    d: &Directive,
    st: &SupStatus,
    r: &SeismicReport,
    v: &SafetyVerdict,
) -> TelemetryFrame {
    let mut flags = 0u8;
    if st.hold_to_run {
        flags |= TelemetryFrame::FLAG_HOLD_TO_RUN;
    }
    if st.presence {
        flags |= TelemetryFrame::FLAG_PRESENCE;
    }
    if st.mains_ok {
        flags |= TelemetryFrame::FLAG_MAINS_OK;
    }
    if s.referenced {
        flags |= TelemetryFrame::FLAG_REFERENCED;
    }
    if s.obstruction {
        flags |= TelemetryFrame::FLAG_OBSTRUCTION;
    }
    if s.intent.is_confirmed() {
        flags |= TelemetryFrame::FLAG_INTENT;
    }
    // The commanded mode reaches the frame through `st.mode` (= `d.mode`); the directive
    // is part of the frozen signature so a later frame revision can carry the planned
    // segment without an API change (the 168-byte layout is frozen in fabrail-types).
    let _ = d;
    TelemetryFrame {
        tick: st.tick,
        x: s.x,
        x_ref: s.x_ref,
        seq,
        lev: st.lev,
        seis: st.seis,
        mode: st.mode,
        demand: core::cmp::max(v.demand, st.demand),
        level: r.level,
        class,
        flags,
        sat_flags: s.sat_flags,
        v_mps: s.v_mps,
        v_ref_mps: s.v_ref_mps,
        gaps_m: s.gap_m,
        coil_i_a: s.coil_i_a,
        iq_a: s.iq_a,
        f_hand_hat_n: s.f_hand_hat_n,
        residual_n: s.residual_n,
        m_hat_kg: s.m_hat_kg,
        e_kin_gov_j: s.e_kin_gov_j,
        e_passivity_j: s.e_passivity_j,
        pd_m: r.pd_m,
        tau_c_s: r.tau_c_s,
        pga_mps2: r.pga_mps2,
        v_bus_v: st.v_bus_v,
        a_floor_x_mps2: s.a_floor_x_mps2,
        faults: st.faults.union(s.faults).union(v.faults),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_types::{EventKind, Tick};

    #[test]
    fn ring_fifo_and_overwrite() {
        let mut r = EventRing::new();
        assert!(r.is_empty());
        for k in 0..70u64 {
            r.push(Event {
                tick: Tick(k),
                kind: EventKind::Command,
                a: 0.0,
                b: 0.0,
            });
        }
        assert_eq!(r.len(), EVENT_RING_CAP);
        assert_eq!(r.dropped, 6);
        assert_eq!(
            r.pop().map(|e| e.tick),
            Some(Tick(6)),
            "oldest survivor first"
        );
        let mut n = 1;
        while r.pop().is_some() {
            n += 1;
        }
        assert_eq!(n, EVENT_RING_CAP);
    }

    /// An assembled frame maps every flag, unions the supervisor, fast-group and monitor
    /// faults, and survives the fixed-layout byte round trip (FW-011). Tolerance (E):
    /// bit-exact.
    #[test]
    fn assembled_frame_flags_faults_roundtrip() {
        use crate::msg::IntentFlag;
        use fabrail_types::{FaultCode, LevState, MotionMode, PosNm, SeismicLevel, Tick};
        let mut sf = fabrail_types::FaultSet::EMPTY;
        sf.set(FaultCode::PayloadMismatch);
        let s = FastStatus {
            x: PosNm(123_456_789),
            x_ref: PosNm(123_000_000),
            v_mps: -0.1,
            referenced: true,
            obstruction: true,
            intent: IntentFlag::Open,
            sat_flags: FastStatus::SAT_GOVERNOR,
            m_hat_kg: 48.0,
            faults: sf,
            ..FastStatus::default()
        };
        let mut stf = fabrail_types::FaultSet::EMPTY;
        stf.set(FaultCode::MainsLost);
        let st = SupStatus {
            tick: Tick(40_000),
            lev: LevState::FaultReaction,
            mode: MotionMode::Freeze,
            demand: fabrail_types::SafetyDemand::SafeLand,
            faults: stf,
            hold_to_run: true,
            presence: true,
            mains_ok: false,
            v_bus_v: 23.5,
            ..SupStatus::default()
        };
        let mut vf = fabrail_types::FaultSet::EMPTY;
        vf.set(FaultCode::Overspeed);
        let v = SafetyVerdict {
            demand: fabrail_types::SafetyDemand::StoLsmAsc,
            force_cap_n: 0.0,
            faults: vf,
        };
        let r = SeismicReport {
            level: SeismicLevel::L2,
            pd_m: 0.004,
            ..SeismicReport::default()
        };
        let f = assemble_frame(9, RailClass::Drawer, &s, &Directive::default(), &st, &r, &v);
        let want = TelemetryFrame::FLAG_HOLD_TO_RUN
            | TelemetryFrame::FLAG_PRESENCE
            | TelemetryFrame::FLAG_REFERENCED
            | TelemetryFrame::FLAG_OBSTRUCTION
            | TelemetryFrame::FLAG_INTENT;
        assert_eq!(
            f.flags, want,
            "(E) flag bits; mains lost clears FLAG_MAINS_OK"
        );
        for c in [
            FaultCode::PayloadMismatch,
            FaultCode::MainsLost,
            FaultCode::Overspeed,
        ] {
            assert!(f.faults.contains(c), "{c:?} in the union");
        }
        assert_eq!(f.demand, fabrail_types::SafetyDemand::StoLsmAsc);
        assert_eq!((f.lev, f.mode, f.level), (st.lev, st.mode, r.level));
        assert_eq!(f.sat_flags, FastStatus::SAT_GOVERNOR);
        let back = TelemetryFrame::from_bytes(&f.to_bytes());
        assert_eq!(back, Some(f), "(E) byte round trip");
    }

    #[test]
    fn frame_carries_max_demand() {
        let st = SupStatus::default();
        let v = SafetyVerdict {
            demand: fabrail_types::SafetyDemand::SafeLand,
            ..SafetyVerdict::default()
        };
        let f = assemble_frame(
            7,
            RailClass::Door,
            &FastStatus::default(),
            &Directive::default(),
            &st,
            &SeismicReport::default(),
            &v,
        );
        assert_eq!(f.seq, 7);
        assert_eq!(f.demand, fabrail_types::SafetyDemand::SafeLand);
    }
}
