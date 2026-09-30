//! Black channel: CRC-32, sequence counter, timeout and source ID; rejects stale, replayed and corrupted frames (FAB-EXO-306). Owner: X1 (API frozen by W0a).
//!
//! EXOTIC §X13 (exotic design §2.4; the IEC 61784-3 "black channel" concept only, never a
//! compliance claim). The safety layer adds a source ID, a sequence counter, a sender
//! timestamp and a CRC-32 (`fabrail_params::crc::crc32`, ISO-HDLC) on top of an
//! untrusted transport (the CH-A ↔ CH-B cross-link of EXOTIC §S3). The receiver rejects:
//!
//! | Transport error | Measure | Reject |
//! |---|---|---|
//! | corruption | CRC-32 over header + payload; unused payload bytes must be zero; len ≤ 32 | `Corrupt` |
//! | masquerade / misrouting | source ID | `WrongSource` |
//! | repetition, re-ordering, replay | sequence number strictly increasing | `Replay` |
//! | delay, stale data | sender timestamp within the freshness window (and not in the future) | `Stale` |
//! | loss | no good frame within the window | `Timeout` |
//!
//! CRC-32/ISO-HDLC detects every error of up to 3 bits and every burst of up to 32 bits
//! in a codeword this short (at most 376 covered bits plus the CRC, inside its
//! Hamming-distance-5 length per Koopman's CRC tables); the tests flip every single and
//! double bit of the wire frame and random bursts of up to 32 bits. A CRC is not a MAC: an
//! attacker who knows the algorithm can forge it, so any cryptographic authentication
//! belongs in the gateway, not here. A sequence number that wraps is a replay; the pair
//! must be re-keyed (receiver reset) before 2³² frames.
//!
//! Wire format ([`Frame::to_bytes`], little-endian, [`FRAME_BYTES`] = 51 bytes):
//! source_id u16 | seq u32 | tick u64 | len u8 | payload [u8; 32] | crc u32.

use fabrail_params::crc::crc32;
use fabrail_types::Tick;

/// Maximum payload bytes per frame.
pub const PAYLOAD_MAX: usize = 32;

/// Header bytes covered by the CRC: source (2) + seq (4) + tick (8) + len (1).
const HEADER_LEN: usize = 15;

/// Bytes of one frame on the wire (header + payload + CRC).
pub const FRAME_BYTES: usize = HEADER_LEN + PAYLOAD_MAX + 4;

/// A black-channel safety frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Frame {
    /// Sender identity.
    pub source_id: u16,
    /// Sequence number (strictly increasing per sender; wrap is a replay).
    pub seq: u32,
    /// Sender timestamp, base ticks.
    pub tick: Tick,
    /// Payload length in bytes (≤ [`PAYLOAD_MAX`]).
    pub len: u8,
    /// Payload (bytes beyond `len` are zero and not covered by the CRC; the receiver
    /// rejects a frame whose unused bytes are not zero).
    pub payload: [u8; PAYLOAD_MAX],
    /// CRC-32 over the header and `payload[..len]`.
    pub crc: u32,
}

/// Little-endian reader over a byte slice that never panics (missing bytes read 0).
struct Reader<'a>(&'a [u8]);

impl Reader<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        match self.0.split_first_chunk::<N>() {
            Some((head, tail)) => {
                self.0 = tail;
                *head
            }
            None => [0; N],
        }
    }
}

impl Frame {
    /// Builds and seals a frame; `None` if the payload is longer than [`PAYLOAD_MAX`].
    pub fn new(source_id: u16, seq: u32, tick: Tick, payload: &[u8]) -> Option<Frame> {
        if payload.len() > PAYLOAD_MAX {
            return None;
        }
        let mut f = Frame {
            source_id,
            seq,
            tick,
            len: payload.len() as u8,
            payload: [0; PAYLOAD_MAX],
            crc: 0,
        };
        for (d, s) in f.payload.iter_mut().zip(payload.iter()) {
            *d = *s;
        }
        f.crc = f.compute_crc();
        Some(f)
    }

    /// CRC-32 over the header and the used payload bytes.
    pub fn compute_crc(&self) -> u32 {
        let mut buf = [0u8; HEADER_LEN + PAYLOAD_MAX];
        let n = HEADER_LEN + (self.len as usize).min(PAYLOAD_MAX);
        let header = self
            .source_id
            .to_le_bytes()
            .into_iter()
            .chain(self.seq.to_le_bytes())
            .chain(self.tick.0.to_le_bytes())
            .chain([self.len]);
        for (d, s) in buf
            .iter_mut()
            .zip(header.chain(self.payload.iter().copied()))
        {
            *d = s;
        }
        crc32(buf.get(..n).unwrap_or(&[]))
    }

    /// The used payload bytes.
    pub fn data(&self) -> &[u8] {
        self.payload
            .get(..(self.len as usize).min(PAYLOAD_MAX))
            .unwrap_or(&[])
    }

    /// True when every payload byte beyond `len` is zero.
    pub fn padding_is_zero(&self) -> bool {
        self.payload
            .iter()
            .skip((self.len as usize).min(PAYLOAD_MAX))
            .all(|b| *b == 0)
    }

    /// The frame as transmitted (little-endian, [`FRAME_BYTES`] bytes).
    pub fn to_bytes(&self) -> [u8; FRAME_BYTES] {
        let mut out = [0u8; FRAME_BYTES];
        let bytes = self
            .source_id
            .to_le_bytes()
            .into_iter()
            .chain(self.seq.to_le_bytes())
            .chain(self.tick.0.to_le_bytes())
            .chain([self.len])
            .chain(self.payload)
            .chain(self.crc.to_le_bytes());
        for (d, s) in out.iter_mut().zip(bytes) {
            *d = s;
        }
        out
    }

    /// Decodes a received frame (no checking: [`Receiver::accept`] checks it).
    pub fn from_bytes(b: &[u8; FRAME_BYTES]) -> Frame {
        let mut r = Reader(b);
        Frame {
            source_id: u16::from_le_bytes(r.take()),
            seq: u32::from_le_bytes(r.take()),
            tick: Tick(u64::from_le_bytes(r.take())),
            len: u8::from_le_bytes(r.take()),
            payload: r.take(),
            crc: u32::from_le_bytes(r.take()),
        }
    }
}

/// Why a frame (or the channel) was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Reject {
    /// CRC mismatch, a length beyond [`PAYLOAD_MAX`] or non-zero unused payload bytes.
    Corrupt = 0,
    /// Not the expected sender.
    WrongSource = 1,
    /// Sequence number not greater than the last accepted one.
    Replay = 2,
    /// Sender timestamp older than the timeout (or in the future).
    Stale = 3,
    /// No good frame within the timeout.
    Timeout = 4,
}

/// Receiver side of one black channel.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Receiver {
    source_id: u16,
    timeout_ticks: u64,
    last_seq: Option<u32>,
    last_rx: Option<Tick>,
}

impl Receiver {
    /// A receiver for `source_id` with a freshness/timeout window of `timeout_ticks`.
    pub const fn new(source_id: u16, timeout_ticks: u64) -> Receiver {
        Receiver {
            source_id,
            timeout_ticks,
            last_seq: None,
            last_rx: None,
        }
    }

    /// Checks `f` received at `now`; on success updates the sequence and freshness state.
    /// A rejected frame never changes the receiver state.
    pub fn accept(&mut self, now: Tick, f: &Frame) -> Result<(), Reject> {
        if f.len as usize > PAYLOAD_MAX || !f.padding_is_zero() || f.compute_crc() != f.crc {
            return Err(Reject::Corrupt);
        }
        if f.source_id != self.source_id {
            return Err(Reject::WrongSource);
        }
        if self.last_seq.is_some_and(|s| f.seq <= s) {
            return Err(Reject::Replay);
        }
        if f.tick > now || now.since(f.tick) > self.timeout_ticks {
            return Err(Reject::Stale);
        }
        self.last_seq = Some(f.seq);
        self.last_rx = Some(now);
        Ok(())
    }

    /// `Err(Timeout)` when no good frame was accepted within the window before `now`
    /// (including never).
    pub fn check_timeout(&self, now: Tick) -> Result<(), Reject> {
        match self.last_rx {
            Some(t) if now.since(t) <= self.timeout_ticks => Ok(()),
            _ => Err(Reject::Timeout),
        }
    }

    /// The last accepted sequence number.
    pub const fn last_seq(&self) -> Option<u32> {
        self.last_seq
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SRC: u16 = 0xB2;
    /// 10 ms freshness window at the 20 kHz base tick.
    const WINDOW: u64 = 200;

    /// Offers `f` to a copy of `rx` (the original is untouched).
    fn probe(rx: Receiver, now: Tick, f: &Frame) -> Result<(), Reject> {
        let mut r = rx;
        r.accept(now, f)
    }

    fn flip(bytes: &[u8; FRAME_BYTES], bits: &[usize]) -> Frame {
        let mut b = *bytes;
        for &k in bits {
            b[k / 8] ^= 1 << (k % 8);
        }
        Frame::from_bytes(&b)
    }

    // req: FAB-EXO-306
    #[test]
    fn rejects_corrupt_stale_replay() {
        let now = Tick(1_000_000);
        let mut rx = Receiver::new(SRC, WINDOW);
        assert_eq!(
            rx.check_timeout(now),
            Err(Reject::Timeout),
            "never received"
        );

        // A good stream is accepted and round-trips through the wire format.
        for seq in 1..=5u32 {
            let f = Frame::new(SRC, seq, Tick(now.0 - 10), &[seq as u8, 0xA5, 0x5A]).unwrap();
            assert_eq!(Frame::from_bytes(&f.to_bytes()), f);
            assert_eq!(rx.accept(now, &f), Ok(()), "seq {seq}");
        }
        assert_eq!(rx.last_seq(), Some(5));
        assert_eq!(rx.check_timeout(now), Ok(()));

        // Corruption: every single-bit flip and every double-bit flip of the 51-byte
        // wire frame (header, payload incl. padding, CRC) is rejected as Corrupt
        // (CRC-32/ISO-HDLC Hamming distance ≥ 4 at this length, EXOTIC §X13).
        let good = Frame::new(SRC, 6, Tick(now.0 - 5), &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        let wire = good.to_bytes();
        let n_bits = FRAME_BYTES * 8;
        let mut trials = 0usize;
        for i in 0..n_bits {
            assert_eq!(
                probe(rx, now, &flip(&wire, &[i])),
                Err(Reject::Corrupt),
                "bit {i}"
            );
            for j in i + 1..n_bits {
                assert_eq!(
                    probe(rx, now, &flip(&wire, &[i, j])),
                    Err(Reject::Corrupt),
                    "bits {i}, {j}"
                );
                trials += 1;
            }
        }
        assert_eq!(
            trials,
            n_bits * (n_bits - 1) / 2,
            "every double-bit pattern tried"
        );
        // Random bursts of 2..=32 bits (first and last bit of the burst set): all caught.
        let mut s = 0x2545_F491_4F6C_DD1Du64;
        for _ in 0..20_000 {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            let len = 2 + (s % 31) as usize;
            let start = ((s >> 8) as usize) % (n_bits - len + 1);
            let mid = (s >> 20) as u32;
            let bits: Vec<usize> = (0..len)
                .filter(|&k| k == 0 || k == len - 1 || (mid >> (k % 32)) & 1 == 1)
                .map(|k| start + k)
                .collect();
            assert_eq!(
                probe(rx, now, &flip(&wire, &bits)),
                Err(Reject::Corrupt),
                "{bits:?}"
            );
        }
        // A length field beyond the payload is corrupt even with a matching CRC.
        let mut long = good;
        long.len = PAYLOAD_MAX as u8 + 1;
        long.crc = long.compute_crc();
        assert_eq!(probe(rx, now, &long), Err(Reject::Corrupt));
        assert!(Frame::new(SRC, 7, now, &[0; PAYLOAD_MAX + 1]).is_none());

        // Masquerade: a well-formed frame from another source.
        let foreign = Frame::new(SRC + 1, 99, now, &[1]).unwrap();
        assert_eq!(probe(rx, now, &foreign), Err(Reject::WrongSource));

        // Replay and re-ordering: the same, an older and a wrapped sequence number.
        let replay = Frame::new(SRC, 5, now, &[5, 0xA5, 0x5A]).unwrap();
        assert_eq!(probe(rx, now, &replay), Err(Reject::Replay));
        let older = Frame::new(SRC, 2, now, &[9]).unwrap();
        assert_eq!(probe(rx, now, &older), Err(Reject::Replay));
        let mut high = rx;
        let top = Frame::new(SRC, u32::MAX, now, &[]).unwrap();
        assert_eq!(high.accept(now, &top), Ok(()));
        let wrapped = Frame::new(SRC, 0, now, &[]).unwrap();
        assert_eq!(
            high.accept(now, &wrapped),
            Err(Reject::Replay),
            "wrap is a replay"
        );

        // Stale: older than the window, or stamped in the future; the boundary is fresh.
        let stale = Frame::new(SRC, 6, Tick(now.0 - WINDOW - 1), &[]).unwrap();
        assert_eq!(probe(rx, now, &stale), Err(Reject::Stale));
        let future = Frame::new(SRC, 6, Tick(now.0 + 1), &[]).unwrap();
        assert_eq!(probe(rx, now, &future), Err(Reject::Stale));
        let edge = Frame::new(SRC, 6, Tick(now.0 - WINDOW), &[]).unwrap();
        assert_eq!(probe(rx, now, &edge), Ok(()));

        // Rejected frames never advance the state: a stale frame with a high sequence
        // number does not block the next genuine frame.
        let stale_high = Frame::new(SRC, 1_000, Tick(now.0 - 10 * WINDOW), &[]).unwrap();
        assert_eq!(rx.accept(now, &stale_high), Err(Reject::Stale));
        assert_eq!(rx.last_seq(), Some(5));
        assert_eq!(rx.accept(now, &good), Ok(()), "seq 6 still accepted");

        // Timeout: fresh within the window after the last good frame, then lost.
        assert_eq!(rx.check_timeout(Tick(now.0 + WINDOW)), Ok(()));
        assert_eq!(
            rx.check_timeout(Tick(now.0 + WINDOW + 1)),
            Err(Reject::Timeout)
        );
    }
}
