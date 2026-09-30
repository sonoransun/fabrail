//! Telemetry: the fixed-size `repr(C)` frame published at 1 kHz by the supervisor group,
//! and discrete events (TECHNICAL_DESIGN §12; FW-011).
//!
//! The frame has no implicit padding: [`TelemetryFrame::to_bytes`] writes every field
//! little-endian at its `repr(C)` offset, so the byte image equals the in-memory image
//! on the little-endian targets (Cortex-M, aarch64, wasm32) and decodes on any host.

use crate::{
    FaultSet, LevState, MAX_MAGNETS, MotionMode, PosNm, RailClass, SafetyDemand, SeismicLevel,
    SeismicState, Tick,
};

/// One supervisor-rate telemetry record.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(C)]
pub struct TelemetryFrame {
    /// Base tick of the supervisor release.            offset 0
    pub tick: Tick,
    /// Measured position.                                offset 8
    pub x: PosNm,
    /// Reference position.                               offset 16
    pub x_ref: PosNm,
    /// Frame sequence number.                            offset 24
    pub seq: u32,
    /// Levitation state.                                 offset 28
    pub lev: LevState,
    /// Seismic state.                                    offset 29
    pub seis: SeismicState,
    /// Motion mode.                                      offset 30
    pub mode: MotionMode,
    /// Current monitor demand.                           offset 31
    pub demand: SafetyDemand,
    /// Detector level.                                   offset 32
    pub level: SeismicLevel,
    /// Rail class.                                       offset 33
    pub class: RailClass,
    /// `FLAG_*` bits.                                    offset 34
    pub flags: u8,
    /// Saturation flags (bit per saturated actuator).    offset 35
    pub sat_flags: u8,
    /// Measured velocity, m/s.                           offset 36
    pub v_mps: f32,
    /// Reference velocity, m/s.                          offset 40
    pub v_ref_mps: f32,
    /// Gaps, m.                                          offset 44
    pub gaps_m: [f32; MAX_MAGNETS],
    /// Coil currents, A.                                 offset 76
    pub coil_i_a: [f32; MAX_MAGNETS],
    /// LSM quadrature current, A.                        offset 108
    pub iq_a: f32,
    /// Estimated hand force F̂_h, N.                     offset 112
    pub f_hand_hat_n: f32,
    /// Obstruction residual, N.                          offset 116
    pub residual_n: f32,
    /// Payload estimate m̂, kg (non-safety).             offset 120
    pub m_hat_kg: f32,
    /// ½·m_gov·v², J.                                    offset 124
    pub e_kin_gov_j: f32,
    /// Passivity observer energy, J.                     offset 128
    pub e_passivity_j: f32,
    /// Peak displacement Pd, m.                          offset 132
    pub pd_m: f32,
    /// τc, s.                                            offset 136
    pub tau_c_s: f32,
    /// Peak ground acceleration seen, m/s².             offset 140
    pub pga_mps2: f32,
    /// Bus voltage, V.                                   offset 144
    pub v_bus_v: f32,
    /// Rail floor acceleration along x, m/s².            offset 148
    pub a_floor_x_mps2: f32,
    /// Active faults.                                    offset 152
    pub faults: FaultSet,
}

impl TelemetryFrame {
    /// Encoded size in bytes (= `size_of::<TelemetryFrame>()`, asserted in tests).
    pub const SIZE: usize = 168;

    /// Encoder referenced.
    pub const FLAG_REFERENCED: u8 = 1 << 0;
    /// Obstruction active.
    pub const FLAG_OBSTRUCTION: u8 = 1 << 1;
    /// Intent confirmed.
    pub const FLAG_INTENT: u8 = 1 << 2;
    /// Mains present.
    pub const FLAG_MAINS_OK: u8 = 1 << 3;
    /// Hold-to-run actuated.
    pub const FLAG_HOLD_TO_RUN: u8 = 1 << 4;
    /// Presence detected in the zone.
    pub const FLAG_PRESENCE: u8 = 1 << 5;

    /// Little-endian byte image at the `repr(C)` offsets.
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut w = Writer {
            buf: [0u8; Self::SIZE],
            at: 0,
        };
        w.u64(self.tick.0);
        w.i64(self.x.0);
        w.i64(self.x_ref.0);
        w.u32(self.seq);
        w.u8(self.lev.as_u8());
        w.u8(self.seis.as_u8());
        w.u8(self.mode.as_u8());
        w.u8(self.demand.as_u8());
        w.u8(self.level.as_u8());
        w.u8(self.class.as_u8());
        w.u8(self.flags);
        w.u8(self.sat_flags);
        w.f32(self.v_mps);
        w.f32(self.v_ref_mps);
        for g in self.gaps_m {
            w.f32(g);
        }
        for i in self.coil_i_a {
            w.f32(i);
        }
        w.f32(self.iq_a);
        w.f32(self.f_hand_hat_n);
        w.f32(self.residual_n);
        w.f32(self.m_hat_kg);
        w.f32(self.e_kin_gov_j);
        w.f32(self.e_passivity_j);
        w.f32(self.pd_m);
        w.f32(self.tau_c_s);
        w.f32(self.pga_mps2);
        w.f32(self.v_bus_v);
        w.f32(self.a_floor_x_mps2);
        for b in self.faults.bits {
            w.u32(b);
        }
        w.buf
    }

    /// Decodes a byte image; `None` if an enum discriminant is unknown.
    pub fn from_bytes(b: &[u8; Self::SIZE]) -> Option<TelemetryFrame> {
        let mut r = Reader { buf: b, at: 0 };
        let tick = Tick(r.u64());
        let x = PosNm(r.i64());
        let x_ref = PosNm(r.i64());
        let seq = r.u32();
        let lev = LevState::from_u8(r.u8())?;
        let seis = SeismicState::from_u8(r.u8())?;
        let mode = MotionMode::from_u8(r.u8())?;
        let demand = SafetyDemand::from_u8(r.u8())?;
        let level = SeismicLevel::from_u8(r.u8())?;
        let class = RailClass::from_u8(r.u8())?;
        let flags = r.u8();
        let sat_flags = r.u8();
        let v_mps = r.f32();
        let v_ref_mps = r.f32();
        let mut gaps_m = [0.0f32; MAX_MAGNETS];
        for g in gaps_m.iter_mut() {
            *g = r.f32();
        }
        let mut coil_i_a = [0.0f32; MAX_MAGNETS];
        for i in coil_i_a.iter_mut() {
            *i = r.f32();
        }
        let iq_a = r.f32();
        let f_hand_hat_n = r.f32();
        let residual_n = r.f32();
        let m_hat_kg = r.f32();
        let e_kin_gov_j = r.f32();
        let e_passivity_j = r.f32();
        let pd_m = r.f32();
        let tau_c_s = r.f32();
        let pga_mps2 = r.f32();
        let v_bus_v = r.f32();
        let a_floor_x_mps2 = r.f32();
        let mut faults = FaultSet::EMPTY;
        for w in faults.bits.iter_mut() {
            *w = r.u32();
        }
        Some(TelemetryFrame {
            tick,
            x,
            x_ref,
            seq,
            lev,
            seis,
            mode,
            demand,
            level,
            class,
            flags,
            sat_flags,
            v_mps,
            v_ref_mps,
            gaps_m,
            coil_i_a,
            iq_a,
            f_hand_hat_n,
            residual_n,
            m_hat_kg,
            e_kin_gov_j,
            e_passivity_j,
            pd_m,
            tau_c_s,
            pga_mps2,
            v_bus_v,
            a_floor_x_mps2,
            faults,
        })
    }
}

/// Fixed-capacity little-endian writer (no panics: writes past the end are dropped,
/// which the layout test proves never happens).
struct Writer<const N: usize> {
    buf: [u8; N],
    at: usize,
}

impl<const N: usize> Writer<N> {
    fn put<const K: usize>(&mut self, bytes: [u8; K]) {
        if let Some(dst) = self.buf.get_mut(self.at..self.at + K) {
            dst.copy_from_slice(&bytes);
        }
        self.at += K;
    }
    fn u8(&mut self, v: u8) {
        self.put([v]);
    }
    fn u32(&mut self, v: u32) {
        self.put(v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.put(v.to_le_bytes());
    }
    fn i64(&mut self, v: i64) {
        self.put(v.to_le_bytes());
    }
    fn f32(&mut self, v: f32) {
        self.put(v.to_bits().to_le_bytes());
    }
}

/// Little-endian reader over a fixed buffer (reads past the end yield zeros).
struct Reader<'a, const N: usize> {
    buf: &'a [u8; N],
    at: usize,
}

impl<const N: usize> Reader<'_, N> {
    fn take<const K: usize>(&mut self) -> [u8; K] {
        let mut out = [0u8; K];
        if let Some(src) = self.buf.get(self.at..self.at + K) {
            out.copy_from_slice(src);
        }
        self.at += K;
        out
    }
    fn u8(&mut self) -> u8 {
        let [b] = self.take::<1>();
        b
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }
    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take())
    }
    fn i64(&mut self) -> i64 {
        i64::from_le_bytes(self.take())
    }
    fn f32(&mut self) -> f32 {
        f32::from_bits(u32::from_le_bytes(self.take()))
    }
}

u8_enum! {
    /// Discrete event kinds (the event ring holds 64 entries; `a`/`b` meaning per kind).
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum EventKind {
        /// LevState change; a = new state, b = old state.
        #[default]
        LevState = 0,
        /// SeismicState change; a = new, b = old.
        SeismicState = 1,
        /// MotionMode change; a = new, b = old.
        Mode = 2,
        /// Monitor demand change; a = new, b = old.
        Demand = 3,
        /// Fault raised; a = FaultCode as u16.
        FaultRaised = 4,
        /// Fault cleared; a = FaultCode as u16.
        FaultCleared = 5,
        /// STA/LTA trigger; a = STA/LTA ratio.
        Trigger = 6,
        /// Network alert received; a = magnitude, b = r_epi_km.
        Alert = 7,
        /// Alert cancelled; a = alert id.
        AlertCancel = 8,
        /// Intent confirmed; a = F̂_h (N), b = direction sign.
        Intent = 9,
        /// Obstruction detected; a = residual (N).
        Obstruction = 10,
        /// Detent passed; a = detent index.
        Detent = 11,
        /// DRAWER seated.
        Seated = 12,
        /// DRAWER unseated; a = m̂ (kg), b = peak coil current (A).
        Unseated = 13,
        /// Touchdown; a = touchdown speed (m/s).
        Landed = 14,
        /// Lift complete; a = lift time (s).
        Lifted = 15,
        /// User command accepted; a = command index.
        Command = 16,
        /// Boot selftest result; a = 1 pass / 0 fail.
        Selftest = 17,
        /// Obstruction reversal started.
        Reversal = 18,
        /// Terminus capture entered; a = d_b (m), b = τ_eff (s).
        TerminusCapture = 19,
        /// Arrived at target; a = arrival speed (m/s).
        Arrived = 20,
        /// Seismic freeze commanded.
        Freeze = 21,
        /// PROTECT entered; a = level, b = seconds to S-ETA.
        Protect = 22,
        /// Bumper contact; a = impact energy (J).
        BumperContact = 23,
        /// Command rejected (e.g. NotHoldToRun, restore before INSPECT reset).
        Rejected = 24,
    }
}

/// One discrete event.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Event {
    /// Base tick at which the event occurred.
    pub tick: Tick,
    /// Kind.
    pub kind: EventKind,
    /// First payload value (meaning per kind).
    pub a: f32,
    /// Second payload value (meaning per kind).
    pub b: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FaultCode;
    use core::mem::{align_of, offset_of, size_of};

    // req: FAB-FW-011
    /// Fixed size, no implicit padding, byte image at the repr(C) offsets, and a lossless
    /// round trip. Tolerance (E): bit-exact.
    #[test]
    fn frame_layout() {
        assert_eq!(
            size_of::<TelemetryFrame>(),
            TelemetryFrame::SIZE,
            "(E) fixed size 168 B"
        );
        assert_eq!(align_of::<TelemetryFrame>(), 8);
        assert_eq!(offset_of!(TelemetryFrame, seq), 24);
        assert_eq!(offset_of!(TelemetryFrame, lev), 28);
        assert_eq!(offset_of!(TelemetryFrame, sat_flags), 35);
        assert_eq!(offset_of!(TelemetryFrame, v_mps), 36);
        assert_eq!(offset_of!(TelemetryFrame, gaps_m), 44);
        assert_eq!(offset_of!(TelemetryFrame, coil_i_a), 76);
        assert_eq!(offset_of!(TelemetryFrame, iq_a), 108);
        assert_eq!(offset_of!(TelemetryFrame, a_floor_x_mps2), 148);
        assert_eq!(offset_of!(TelemetryFrame, faults), 152);

        let mut faults = FaultSet::EMPTY;
        faults.set(FaultCode::SeatFailed);
        faults.set(FaultCode::VerticalSaturation);
        let mut f = TelemetryFrame {
            tick: Tick(123_456_789),
            x: PosNm(-5_000_000_001),
            x_ref: PosNm(7),
            seq: 0xDEAD_BEEF,
            lev: LevState::Seating,
            seis: SeismicState::Quiet,
            mode: MotionMode::InertialSmooth,
            demand: SafetyDemand::StoLsmAsc,
            level: SeismicLevel::L3,
            class: RailClass::Heavy,
            flags: TelemetryFrame::FLAG_MAINS_OK | TelemetryFrame::FLAG_REFERENCED,
            sat_flags: 0x81,
            v_mps: -0.265,
            v_ref_mps: 0.1838,
            iq_a: 1.5,
            f_hand_hat_n: -4.5,
            residual_n: 12.0,
            m_hat_kg: 48.0,
            e_kin_gov_j: 1.69,
            e_passivity_j: -0.001,
            pd_m: 0.007_118,
            tau_c_s: 0.6,
            pga_mps2: 0.8,
            v_bus_v: 24.0,
            a_floor_x_mps2: -1.25,
            faults,
            ..TelemetryFrame::default()
        };
        for (i, g) in f.gaps_m.iter_mut().enumerate() {
            *g = 0.0025 + i as f32 * 1.0e-6;
        }
        for (i, c) in f.coil_i_a.iter_mut().enumerate() {
            *c = -3.0 + i as f32;
        }
        let bytes = f.to_bytes();
        // Spot-check the little-endian image at the declared offsets.
        assert_eq!(&bytes[0..8], &123_456_789_u64.to_le_bytes());
        assert_eq!(bytes[28], LevState::Seating.as_u8());
        assert_eq!(&bytes[36..40], &(-0.265_f32).to_bits().to_le_bytes());
        assert_eq!(&bytes[148..152], &(-1.25_f32).to_bits().to_le_bytes());
        let back = TelemetryFrame::from_bytes(&bytes).expect("valid frame decodes");
        assert_eq!(back, f, "(E) lossless round trip");
        assert_eq!(back.to_bytes(), bytes);

        // An unknown discriminant is rejected rather than transmuted.
        let mut bad = bytes;
        bad[28] = 0xFF;
        assert!(TelemetryFrame::from_bytes(&bad).is_none());
    }

    #[test]
    fn default_frame_roundtrips() {
        let f = TelemetryFrame::default();
        assert_eq!(TelemetryFrame::from_bytes(&f.to_bytes()), Some(f));
        let e = Event {
            tick: Tick(3),
            kind: EventKind::Arrived,
            a: 0.001,
            b: 0.0,
        };
        assert_eq!(EventKind::from_u8(e.kind.as_u8()), Some(EventKind::Arrived));
    }
}
