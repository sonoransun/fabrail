//! Seismic front end: ADXL355 FIFO samples, anti-alias decimation by 5, 0.5-20 Hz Butterworth band-pass (PHYSICS §20). Owner: A3.
//!
//! Frozen API (W0): [`SeismicFrontend::new`], [`SeismicFrontend::push`] (one 500 Hz
//! FIFO sample → at most one 100 Hz output), [`SeismicSample`]. The ADXL355 (500 Hz ODR,
//! 22.5 µg/√Hz, ±2 g, 20-bit) is mounted on the structure or rail, never the carriage
//! (SEI-001; core design F16). Anti-alias ÷5 (`detector.aa_decim`), then a 4-pole
//! Butterworth band-pass 0.5–20 Hz, prewarped, TDF-II (`filters::butterworth_bandpass`).
//! Sequence gaps and FIFO overruns raise AccelLost; a frozen output raises AccelStuck.
//!
//! **Anti-alias decimation (PHYSICS §20).** A second-order CIC (Hogenauer 1981), i.e. the
//! triangular FIR h_k = min(k + 1, 2D − 1 − k)/D², k = 0 … 2D − 2 (D = `aa_decim`; for
//! D = 5: [1 2 3 4 5 4 3 2 1]/25), evaluated once every D input samples on the integer
//! codes. Its double zeros at multiples of ODR/D (100, 200 Hz) are exactly the bands that
//! fold onto DC; droop is −0.3 dB at 10 Hz and −1.1 dB at 20 Hz, and a 80 Hz tone (which
//! folds to 20 Hz) is attenuated by 24.5 dB (on top of the sensor's own ODR/4 digital
//! low-pass, ADI ADXL355 datasheet). The weighted sum is exact in i64 and exactly
//! representable in f32 for D ≤ 5 (≤ 2¹⁹·25 < 2²⁴), so the decimated value is one exact
//! f32 product: bit-identical on host, wasm and firmware.
//!
//! **Band-pass.** `filters::butterworth_bandpass(bp_order, bp_lo_hz, bp_hi_hz, ODR/D)` per
//! axis (the design is A1's; the numpy reference of SEI-002 mirrors it). At the first
//! output every section's TDF-II state is primed to the DC steady state of that sample
//! (y = b0·x + s0 convention, see `prime_cascade`), so gravity on z (+1 g) and any tilt
//! offset do not ring through the band-pass and inflate the STA/LTA warm-up.
//!
//! **Faults.** AccelLost on a FIFO sequence gap or overrun; AccelStuck when all three
//! codes stay identical for one short Pd/τc window of samples (`win_short_s`·ODR, 1 s):
//! with 22.5 µg/√Hz over the ODR/4 band (≈ 90 LSB rms) a live sensor never repeats a
//! triple for that long. The silent-sensor case (no samples at all) is detected per
//! release by `SeismicUnit`.
//!
//! PGA uses the band-passed horizontal magnitude (tilt-free); Pd/τc use the anti-aliased
//! vertical channel (`raw_mps2[2]`, gravity included; `pdtc` high-passes it).

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{AccelRaw, FaultCode, FaultSet};

use crate::filters::{Biquad, Cascade, butterworth_bandpass};

/// Largest supported anti-alias decimation (the CIC ring holds 2·8 − 1 = 15 samples).
pub const AA_DECIM_MAX: u16 = 8;

/// CIC ring length (2·`AA_DECIM_MAX` − 1).
const RING: usize = 2 * AA_DECIM_MAX as usize - 1;

/// ADXL355 code range: 20-bit two's complement (ADI datasheet).
const CODE_MAX: i32 = (1 << 19) - 1;
/// See [`CODE_MAX`].
const CODE_MIN: i32 = -(1 << 19);

/// One decimated seismic sample (100 Hz), m/s².
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SeismicSample {
    /// Band-passed acceleration (x, y, z) for STA/LTA, Z/H and PGA.
    pub bp_mps2: [f32; 3],
    /// Anti-aliased (not band-passed) acceleration for Pd/τc (z) and the floor-velocity
    /// estimate (x). Gravity is included on z.
    pub raw_mps2: [f32; 3],
}

/// Decimating, band-passing front end.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SeismicFrontend {
    /// m/s² per LSB.
    pub scale_mps2_per_lsb: f32,
    /// Decimation factor (0: inert, never outputs).
    pub decim: u16,
    /// Samples since the last output.
    pub count: u16,
    /// Last FIFO sequence number seen.
    pub last_seq: u32,
    /// A first sample has been seen (the CIC ring and band-pass are primed from it).
    pub seen: bool,
    /// CIC shift register, newest first (only the first 2·decim − 1 entries are used).
    pub ring: [[i32; 3]; RING],
    /// Output gain: scale / D², m/s² per weighted-code unit.
    pub gain: f32,
    /// Band-pass per axis (x, y, z).
    pub bp: [Cascade; 3],
    /// The band-pass states have been primed.
    pub bp_primed: bool,
    /// Last raw codes (stuck detection).
    pub last_xyz: [i32; 3],
    /// Consecutive samples with identical codes.
    pub same_n: u32,
    /// AccelStuck after this many identical samples (0 disables).
    pub stuck_n: u32,
}

impl SeismicFrontend {
    /// Front end from `design.sensors` and `control.seismic.detector`.
    ///
    /// Degenerate parameters (zero ODR or decimation, as in `SystemParams::default()`)
    /// give an inert front end that never outputs; a decimation above
    /// [`AA_DECIM_MAX`] is rejected.
    pub fn new(p: &SystemParams) -> Result<SeismicFrontend, ParamError> {
        let s = &p.design.sensors;
        let dt = &p.control.seismic.detector;
        if dt.aa_decim > AA_DECIM_MAX {
            return Err(ParamError::new(
                "control.seismic.detector.aa_decim",
                "above the anti-alias CIC capacity (8)",
            ));
        }
        let d = f64::from(dt.aa_decim);
        let odr = s.accel_odr_hz;
        let usable = dt.aa_decim > 0 && odr.is_finite() && odr > 0.0;
        let decim = if usable { dt.aa_decim } else { 0 };
        let gain = if usable {
            (s.accel_mps2_per_lsb / (d * d)) as f32
        } else {
            0.0
        };
        let bp1 = if usable {
            butterworth_bandpass(dt.bp_order, dt.bp_lo_hz, dt.bp_hi_hz, odr / d)
        } else {
            Cascade::default()
        };
        let stuck = dt.win_short_s * odr;
        let stuck_n = if usable && stuck.is_finite() && stuck >= 1.0 {
            stuck as u32
        } else {
            0
        };
        Ok(SeismicFrontend {
            scale_mps2_per_lsb: s.accel_mps2_per_lsb as f32,
            decim,
            count: 0,
            last_seq: 0,
            seen: false,
            ring: [[0; 3]; RING],
            gain,
            bp: [bp1; 3],
            bp_primed: false,
            last_xyz: [0; 3],
            same_n: 0,
            stuck_n,
        })
    }

    /// CIC-2 length 2·D − 1 (0 when inert).
    #[inline]
    pub fn taps(&self) -> usize {
        (2 * usize::from(self.decim)).saturating_sub(1)
    }

    /// Pushes one FIFO sample; returns the decimated sample when one is due and any
    /// faults detected (sequence gap, overrun, stuck output).
    pub fn push(&mut self, s: &AccelRaw) -> (Option<SeismicSample>, FaultSet) {
        let mut faults = FaultSet::EMPTY;
        if self.seen && s.seq != self.last_seq.wrapping_add(1) {
            faults.set(FaultCode::AccelLost);
        }
        if s.overrun {
            faults.set(FaultCode::AccelLost);
        }
        self.last_seq = s.seq;
        let xyz = s.xyz.map(|c| c.clamp(CODE_MIN, CODE_MAX));
        if self.seen && xyz == self.last_xyz {
            self.same_n = self.same_n.saturating_add(1);
        } else {
            self.same_n = 0;
        }
        self.last_xyz = xyz;
        if self.stuck_n > 0 && self.same_n >= self.stuck_n {
            faults.set(FaultCode::AccelStuck);
        }
        if self.decim == 0 {
            self.seen = true;
            return (None, faults);
        }
        if !self.seen {
            // Prime the CIC with the first sample (no start-up step through the FIR).
            self.ring = [xyz; RING];
            self.seen = true;
        }
        self.ring.copy_within(0..RING - 1, 1);
        if let Some(first) = self.ring.first_mut() {
            *first = xyz;
        }
        self.count = self.count.saturating_add(1);
        if self.count < self.decim {
            return (None, faults);
        }
        self.count = 0;

        // CIC-2: triangular weights min(k + 1, 2D − 1 − k), exact in i64.
        let taps = self.taps();
        let mut acc = [0_i64; 3];
        for (k, c) in self.ring.iter().take(taps).enumerate() {
            let w = (k + 1).min(taps - k) as i64;
            for (a, &ci) in acc.iter_mut().zip(c.iter()) {
                *a += w * i64::from(ci);
            }
        }
        let raw = acc.map(|a| a as f32 * self.gain);
        if !self.bp_primed {
            for (c, &x0) in self.bp.iter_mut().zip(raw.iter()) {
                prime_cascade(c, x0);
            }
            self.bp_primed = true;
        }
        let mut bp = [0.0_f32; 3];
        for ((y, c), &x) in bp.iter_mut().zip(self.bp.iter_mut()).zip(raw.iter()) {
            *y = c.step(x);
        }
        (
            Some(SeismicSample {
                bp_mps2: bp,
                raw_mps2: raw,
            }),
            faults,
        )
    }

    /// Clears filter state and the decimation phase (the next sample re-primes).
    pub fn reset(&mut self) {
        self.count = 0;
        self.seen = false;
        self.bp_primed = false;
        self.same_n = 0;
        for c in self.bp.iter_mut() {
            c.reset();
        }
    }
}

/// Sets a biquad's TDF-II state to the steady state of a constant input `x0` and returns
/// the steady output y = H(1)·x0 = (b0 + b1 + b2)/(1 + a1 + a2)·x0.
///
/// Convention (standard TDF-II, `filters::Biquad`): y = b0·x + s0,
/// s0 ← b1·x − a1·y + s1, s1 ← b2·x − a2·y; constant x and y give s1 = b2·x − a2·y and
/// s0 = b1·x − a1·y + s1. A section with a pole at z = 1 (1 + a1 + a2 = 0) has no steady
/// state and is left unprimed at zero.
pub(crate) fn prime_biquad(b: &mut Biquad, x0: f32) -> f32 {
    let [b0, b1, b2] = b.b;
    let [a1, a2] = b.a;
    let den = 1.0 + a1 + a2;
    if !(x0.is_finite() && den.is_finite() && den != 0.0) {
        b.s = [0.0; 2];
        return 0.0;
    }
    let y = (b0 + b1 + b2) / den * x0;
    let s1 = b2 * x0 - a2 * y;
    let s0 = b1 * x0 - a1 * y + s1;
    b.s = [s0, s1];
    y
}

/// Primes the active sections of a cascade in order (see [`prime_biquad`]); returns the
/// steady output.
pub(crate) fn prime_cascade(c: &mut Cascade, x0: f32) -> f32 {
    let n = usize::from(c.n);
    let mut u = x0;
    for s in c.sec.iter_mut().take(n) {
        u = prime_biquad(s, u);
    }
    u
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    fn sample(seq: u32, xyz: [i32; 3]) -> AccelRaw {
        AccelRaw {
            xyz,
            seq,
            overrun: false,
        }
    }

    #[test]
    fn decimates_by_five_with_exact_cic_gain() {
        let p = presets::wall();
        let mut fe = SeismicFrontend::new(&p).expect("preset front end");
        assert_eq!(fe.decim, 5, "aa_decim 5 (500 → 100 Hz; core design §C.4)");
        assert_eq!(fe.taps(), 9, "CIC-2 length 2D − 1");
        let code = [1234, -5678, 256_410];
        let mut outs = 0;
        let mut last = None;
        for k in 0..500_u32 {
            let (o, f) = fe.push(&sample(k + 1, code));
            assert!(!f.contains(FaultCode::AccelLost), "contiguous sequence");
            if let Some(o) = o {
                outs += 1;
                last = Some(o);
            }
        }
        assert_eq!(outs, 100, "500 FIFO samples → 100 outputs (÷5)");
        let o = last.expect("an output");
        // DC gain is exactly 1: Σh = D² (exact integer sum), then one f32 product.
        for (r, c) in o.raw_mps2.iter().zip(code.iter()) {
            assert_eq!(
                *r,
                (c * 25) as f32 * fe.gain,
                "CIC sum exact (tolerance 0; PHYSICS §20)"
            );
            let si = f64::from(*c) * p.design.sensors.accel_mps2_per_lsb;
            assert!(
                (f64::from(*r) - si).abs() <= 1e-6 * si.abs(),
                "DC gain 1 within f32 rounding (1e-6 relative)"
            );
        }
    }

    #[test]
    fn cic_nulls_the_alias_of_dc() {
        // A 100 Hz tone at 500 Hz ODR folds onto DC after ÷5; the CIC has a double zero
        // there, so an integer-periodic tone decimates to exactly the DC value.
        let p = presets::door();
        let mut fe = SeismicFrontend::new(&p).expect("preset front end");
        let tone = [0_i32, 3000, 900, -3000, -900]; // one 100 Hz period, zero mean
        let mut seq = 0;
        let mut last = None;
        for _ in 0..40 {
            for &t in tone.iter() {
                seq += 1;
                if let (Some(o), _) = fe.push(&sample(seq, [t, 0, 0])) {
                    last = Some(o);
                }
            }
        }
        let o = last.expect("an output");
        assert_eq!(
            o.raw_mps2[0], 0.0,
            "100 Hz alias of DC removed exactly (CIC zero; tolerance 0)"
        );
    }

    #[test]
    fn sequence_gap_and_overrun_raise_accel_lost() {
        let p = presets::drawer();
        let mut fe = SeismicFrontend::new(&p).expect("preset front end");
        let (_, f) = fe.push(&sample(10, [1, 2, 3]));
        assert!(f.is_empty(), "first sample sets the sequence");
        let (_, f) = fe.push(&sample(11, [2, 3, 4]));
        assert!(f.is_empty());
        let (_, f) = fe.push(&sample(13, [3, 4, 5]));
        assert!(f.contains(FaultCode::AccelLost), "gap 11 → 13 is FIFO loss");
        let mut s = sample(14, [4, 5, 6]);
        s.overrun = true;
        let (_, f) = fe.push(&s);
        assert!(f.contains(FaultCode::AccelLost), "overrun flag");
        let (_, f) = fe.push(&sample(u32::MAX, [5, 6, 7]));
        assert!(f.contains(FaultCode::AccelLost));
        let (_, f) = fe.push(&sample(0, [6, 7, 8]));
        assert!(f.is_empty(), "sequence wraps without a fault");
    }

    #[test]
    fn frozen_output_raises_accel_stuck() {
        let p = presets::door();
        let mut fe = SeismicFrontend::new(&p).expect("preset front end");
        assert_eq!(fe.stuck_n, 500, "one 1 s window at 500 Hz");
        let mut stuck_at = None;
        for k in 0..600_u32 {
            let (_, f) = fe.push(&sample(k, [7, 7, 7]));
            if f.contains(FaultCode::AccelStuck) && stuck_at.is_none() {
                stuck_at = Some(k);
            }
        }
        assert_eq!(
            stuck_at,
            Some(500),
            "AccelStuck after 500 identical samples"
        );
        let (_, f) = fe.push(&sample(600, [7, 8, 7]));
        assert!(
            !f.contains(FaultCode::AccelStuck),
            "a changing code clears it"
        );
    }

    #[test]
    fn default_params_are_inert_and_oversized_decimation_rejected() {
        let mut fe = SeismicFrontend::new(&SystemParams::default()).expect("inert front end");
        for k in 0..20 {
            assert_eq!(fe.push(&sample(k, [1, 2, 3])).0, None, "never outputs");
        }
        let mut p = presets::door();
        p.control.seismic.detector.aa_decim = 9;
        assert!(
            SeismicFrontend::new(&p).is_err(),
            "D > 8 exceeds the CIC ring"
        );
    }

    #[test]
    fn priming_gives_the_dc_steady_state() {
        // A band-pass-like section (zeros at ±1) and a low-pass-like one: after priming,
        // a constant input keeps a constant output (TDF-II algebra, exact up to rounding).
        let mut hp = Biquad::new([0.2, 0.0, -0.2], [-1.5, 0.6]);
        let y = prime_biquad(&mut hp, 9.8);
        assert!(y.abs() < 1e-6, "zero DC gain → zero steady output");
        let [b0, b1, b2] = hp.b;
        let [a1, a2] = hp.a;
        // One manual TDF-II step keeps the state.
        let x = 9.8_f32;
        let yy = b0 * x + hp.s[0];
        let s0 = b1 * x - a1 * yy + hp.s[1];
        let s1 = b2 * x - a2 * yy;
        assert!(
            (yy - y).abs() < 1e-5 && (s0 - hp.s[0]).abs() < 1e-5 && (s1 - hp.s[1]).abs() < 1e-5
        );
        let mut lp = Biquad::new([0.25, 0.5, 0.25], [-0.2, 0.1]);
        let y = prime_biquad(&mut lp, 2.0);
        assert!((y - 2.0 / 0.9).abs() < 1e-6, "H(1) = 1/(1 − 0.2 + 0.1)");
    }
}
