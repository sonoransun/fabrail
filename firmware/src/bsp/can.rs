//! FDCAN1: command/alert link and telemetry (RM0440 §44; fdcan 0.2.1 driver through
//! stm32g4xx-hal's `can` feature).
//!
//! **Bus.** Kernel clock = HSE 24 MHz; nominal bit timing prescaler 1, seg1 19, seg2 4,
//! SJW 4 → 24 tq per bit = 1 Mbit/s, sample point 83 % (CiA 601-3 recommends
//! 80–87.5 %). CAN FD frames are accepted and sent without bit-rate switching; the
//! 5 Mbit/s data phase and the CiA 402 object mapping are roadmap (core design §F).
//! Receive: standard IDs 0x200–0x21F into RX FIFO 0 (interrupt line 0 → `can_rx` task,
//! FDCAN1_IT0); everything else is rejected in hardware.
//!
//! **fabrail link v0** (standard 11-bit IDs, little-endian payloads):
//!
//! | ID | Dir | Len | Payload |
//! |---|---|---|---|
//! | 0x200 | in | 1 or 12 | `UserCommand`: byte 0 = 0 Open, 1 Close, 2 GoTo (bytes 1..9 = i64 nm), 3 Stop, 4 Lift, 5 Land, 6 Reset, 7 Manual, 8 HoldToRun(false), 9 HoldToRun(true) |
//! | 0x210 | in | 32 | `NetworkAlert`: id u32, origin_offset_s f32, magnitude f32, r_epi_km f32, depth_km f32, mmi f32, flags u8 (bit 0 = cancel); `received` = base tick at reception |
//! | 0x300–0x302 | out | 64/64/48 | `TelemetryFrame::to_bytes()` bytes 0..64, 64..128, 128..168 (zero-padded) |
//! | 0x310 | out | 24 | `Event`: tick u64, kind u8, 3 pad, a f32, b f32, 4 pad |
//! | 0x320 | out | 64 | link status, sixteen u32 words: see [`encode_status`] |
//!
//! Alerts on this link are whatever the building gateway forwards; fabrail is not an
//! EEW provider and a network-only alert may cause only FREEZE/LAND/LOCK (enforced in
//! fabrail-core, not here).

use core::num::{NonZeroU8, NonZeroU16};

use fabrail_types::{Event, NetworkAlert, PosNm, TelemetryFrame, Tick, UserCommand};
use fdcan::config::{FrameTransmissionConfig, GlobalFilter, NominalBitTiming};
use fdcan::filter::{Action, FilterType, StandardFilter, StandardFilterSlot};
use fdcan::frame::{FrameFormat, TxFrameHeader};
use fdcan::id::StandardId;
use fdcan::interrupt::{Interrupt, InterruptLine};
use stm32g4xx_hal::can::Can;
use stm32g4xx_hal::rcc::Rcc;
use stm32g4xx_hal::stm32::FDCAN1;

/// FDCAN1 peripheral wrapper.
pub type CanPeriph = Can<FDCAN1>;
/// Transmit half (telemetry task).
pub type CanTx = fdcan::Tx<CanPeriph, fdcan::NormalOperationMode>;

/// Command frame ID.
pub const ID_COMMAND: u16 = 0x200;
/// Alert frame ID.
pub const ID_ALERT: u16 = 0x210;
/// First telemetry frame ID (three consecutive IDs).
pub const ID_TELEMETRY: u16 = 0x300;
/// Event frame ID.
pub const ID_EVENT: u16 = 0x310;
/// Link status frame ID.
pub const ID_STATUS: u16 = 0x320;

/// Receive half with the interrupt control (can_rx task).
pub struct CanRx {
    ctrl: fdcan::FdCanControl<CanPeriph, fdcan::NormalOperationMode>,
    rx: fdcan::Rx<CanPeriph, fdcan::NormalOperationMode, fdcan::Fifo0>,
    /// Frames lost to a full RX FIFO.
    pub overruns: u32,
}

/// Configures FDCAN1 and splits it into the receive and transmit halves.
pub fn init(fdcan1: FDCAN1, rcc: &Rcc) -> (CanRx, CanTx) {
    rcc.apb1enr1().modify(|_, w| w.fdcanen().set_bit());
    let mut can = Can::fdcan1(fdcan1);
    can.set_protocol_exception_handling(false);
    let one = NonZeroU16::MIN;
    let nz8 = |v: u8| NonZeroU8::new(v).unwrap_or(NonZeroU8::MIN);
    can.set_nominal_bit_timing(NominalBitTiming {
        prescaler: one,
        seg1: nz8(19),
        seg2: nz8(4),
        sync_jump_width: nz8(4),
    });
    can.set_frame_transmit(FrameTransmissionConfig::AllowFdCan);
    can.set_global_filter(GlobalFilter::reject_all());
    if let (Some(from), Some(to)) = (StandardId::new(0x200), StandardId::new(0x21F)) {
        can.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter {
                filter: FilterType::Range { from, to },
                action: Action::StoreInFifo0,
            },
        );
    }
    can.enable_interrupt(Interrupt::RxFifo0NewMsg);
    can.enable_interrupt_line(InterruptLine::_0, true);
    let (ctrl, tx, rx, _rx1) = can.into_normal().split();
    (
        CanRx {
            ctrl,
            rx,
            overruns: 0,
        },
        tx,
    )
}

/// A decoded inbound frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Inbound {
    /// User/building command.
    Command(UserCommand),
    /// Network alert (or cancellation).
    Alert(NetworkAlert),
}

fn f32_at(d: &[u8], at: usize) -> Option<f32> {
    let b = d.get(at..at + 4)?;
    Some(f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

fn u32_at(d: &[u8], at: usize) -> Option<u32> {
    let b = d.get(at..at + 4)?;
    Some(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

/// Decodes a link-v0 frame; None for an unknown ID or a malformed payload.
pub fn decode(id: u16, d: &[u8], now: Tick) -> Option<Inbound> {
    match id {
        ID_COMMAND => {
            let c = match *d.first()? {
                0 => UserCommand::Open,
                1 => UserCommand::Close,
                2 => {
                    let b = d.get(1..9)?;
                    let mut raw = [0u8; 8];
                    raw.copy_from_slice(b);
                    UserCommand::GoTo(PosNm(i64::from_le_bytes(raw)))
                }
                3 => UserCommand::Stop,
                4 => UserCommand::Lift,
                5 => UserCommand::Land,
                6 => UserCommand::Reset,
                7 => UserCommand::Manual,
                8 => UserCommand::HoldToRun(false),
                9 => UserCommand::HoldToRun(true),
                _ => return None,
            };
            Some(Inbound::Command(c))
        }
        ID_ALERT => Some(Inbound::Alert(NetworkAlert {
            id: u32_at(d, 0)?,
            received: now,
            origin_offset_s: f32_at(d, 4)?,
            magnitude: f32_at(d, 8)?,
            r_epi_km: f32_at(d, 12)?,
            depth_km: f32_at(d, 16)?,
            mmi: f32_at(d, 20)?,
            cancel: d.get(24)? & 1 != 0,
        })),
        _ => None,
    }
}

impl CanRx {
    /// Clears the RX-FIFO-0 interrupt and returns the next frame (ID, payload, length).
    pub fn receive(&mut self, buf: &mut [u8; 64]) -> Option<(u16, usize)> {
        self.ctrl.clear_interrupt(Interrupt::RxFifo0NewMsg);
        match self.rx.receive(buf) {
            Ok(r) => {
                if matches!(r, fdcan::ReceiveOverrun::Overrun(_)) {
                    self.overruns = self.overruns.wrapping_add(1);
                }
                let h = r.unwrap();
                let id = match h.id {
                    fdcan::id::Id::Standard(s) => s.as_raw(),
                    fdcan::id::Id::Extended(_) => return None,
                };
                Some((id, usize::from(h.len).min(64)))
            }
            Err(_) => None,
        }
    }
}

/// Queues one FD frame; false when every TX buffer is busy (the frame is dropped and
/// the caller counts it). `data.len()` must be a valid FD length (≤ 64).
pub fn send(tx: &mut CanTx, id: u16, data: &[u8]) -> bool {
    let Some(sid) = StandardId::new(id) else {
        return false;
    };
    let len = data.len().min(64) as u8;
    let header = TxFrameHeader {
        len,
        frame_format: FrameFormat::Fdcan,
        id: sid.into(),
        bit_rate_switching: false,
        marker: None,
    };
    if tx.tx_queue_is_full() {
        return false;
    }
    tx.transmit(header, data.get(..usize::from(len)).unwrap_or(data))
        .is_ok()
}

/// The three telemetry payloads (64, 64, 48 bytes) of one frame.
pub fn encode_telemetry(f: &TelemetryFrame) -> ([u8; 64], [u8; 64], [u8; 48]) {
    let b = f.to_bytes();
    let mut p0 = [0u8; 64];
    let mut p1 = [0u8; 64];
    let mut p2 = [0u8; 48];
    for (dst, src) in p0.iter_mut().zip(b.iter()) {
        *dst = *src;
    }
    for (dst, src) in p1.iter_mut().zip(b.iter().skip(64)) {
        *dst = *src;
    }
    for (dst, src) in p2.iter_mut().zip(b.iter().skip(128)) {
        *dst = *src;
    }
    (p0, p1, p2)
}

/// One event payload.
pub fn encode_event(e: &Event) -> [u8; 24] {
    let mut p = [0u8; 24];
    let fields: [(usize, &[u8]); 4] = [
        (0, &e.tick.0.to_le_bytes()),
        (8, &[e.kind.as_u8()]),
        (12, &e.a.to_bits().to_le_bytes()),
        (16, &e.b.to_bits().to_le_bytes()),
    ];
    for (at, bytes) in fields {
        if let Some(dst) = p.get_mut(at..at + bytes.len()) {
            dst.copy_from_slice(bytes);
        }
    }
    p
}

/// Link status payload (64 bytes, sixteen little-endian u32 words; the word meanings
/// are listed at `StatusWord` in `main.rs`: boot state, reset cause, crash record,
/// selftest hash, per-task maximum DWT cycles and overrun counts).
pub fn encode_status(words: [u32; 16]) -> [u8; 64] {
    let mut p = [0u8; 64];
    for (dst, w) in p.as_chunks_mut::<4>().0.iter_mut().zip(words) {
        dst.copy_from_slice(&w.to_le_bytes());
    }
    p
}
