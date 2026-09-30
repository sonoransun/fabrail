//! Sensor models: noise, quantisation, bandwidth, latency and ADXL355 synthesis from floor + site + noise (PHYSICS §13). Owner: A7.
//!
//! Every conversion from f64 truth to an integer raw code happens here or in
//! `simboard` (core design §A.4 item 1). Defaults (core design §C.5; physics
//! perspective §1.12):
//!
//! | Sensor | Model |
//! |---|---|
//! | Gap (per element, primary + redundant probe) | truth → first-order anti-alias (τ 20 µs) → + bias (+ tempco drift) → + white noise σ = `gap_noise_m` (the in-band rms, added after the filter so the rms equals the parameter) → inverse of the linearisation polynomial → 16-bit code |
//! | Coil current | + white noise σ = `coil_noise_a` → 12-bit signed code |
//! | LSM phase currents | + white noise σ = 1 LSB → 12-bit signed code |
//! | Encoder | floor(x / resolution) counts (1 µm/count), no noise |
//! | Carriage IMU | inertial acceleration + tilt leakage g·θ + white noise (`imu_noise_mps2`) → 16-bit code |
//! | Rail floor accelerometer | floor acceleration + white noise (`rail_accel_noise_mps2`) → 16-bit code |
//! | Structure ADXL355 | floor + site + white noise with the data-sheet density over the Nyquist band → 20-bit code (±2.048 g clip), FIFO of 32 samples at the ODR |
//! | Handle load cell (optional) | + white noise σ = `handle_noise_n` → signed code |
//!
//! Accelerometer codes carry DYNAMIC acceleration only: the static 1 g of the vertical
//! axis is taken as removed by the commissioning offset calibration in the BSP, so the
//! core front ends (`seismic::frontend`, which computes PGA from the anti-aliased
//! signal) receive gravity-free codes. The carriage IMU still sees the tilt leakage
//! g·θ (1 mg per mrad, plan §4b), the one gravity effect that calibration cannot
//! remove.
//!
//! Sensor transport latency (`latency_ticks`, 1 by default) is applied by `simboard`
//! (a delay line on the sensed state); the actuation side is a zero-order hold over the
//! plant step (the "latch"), so the loop has exactly one base tick of transport delay
//! plus the ZOH, the delay `params::derive` designs the current and gap loops for.
//!
//! Each noise source draws from its own named substream (`rng::StreamTag`), per device
//! and channel, and draws only when the HAL samples it, so the streams are a pure
//! function of (noise seed, device, call sequence).

use std::collections::VecDeque;

use fabrail_params::SensorCal;
use fabrail_types::{AccelRaw, BASE_TICK_S, MAX_MAGNETS, fmath};

use crate::rng::{NormalStream, StreamTag, Xoshiro256pp, substream};

/// Gap probe anti-alias time constant (first order), s. Source: physics perspective
/// §1.12 ("1st-order anti-alias (20 µs)"); PHYSICS §13.
pub const GAP_ANTI_ALIAS_TAU_S: f64 = 20.0e-6;

/// LSM phase-current sense noise, LSB rms. Source: physics perspective §1.12
/// ("Coil / LSM current: 12 bit over ±I_range, σ = 1 LSB"); PHYSICS §13.
pub const LSM_NOISE_LSB: f64 = 1.0;

/// Coil and LSM current-sense code range: 12-bit signed (core design §C.5 "current
/// 12-bit").
pub const CURRENT_CODE_MIN: i16 = -2048;
/// See [`CURRENT_CODE_MIN`].
pub const CURRENT_CODE_MAX: i16 = 2047;

/// Carriage IMU and rail accelerometer code range: 16-bit signed (±8 g at
/// 8·g₀/32768 per LSB, the preset scale).
pub const ACC16_CODE_MIN: i32 = -32_768;
/// See [`ACC16_CODE_MIN`].
pub const ACC16_CODE_MAX: i32 = 32_767;

/// ADXL355 20-bit two's-complement code range (±2.048 g at 3.9 µg/LSB). Source: Analog
/// Devices ADXL354/ADXL355 data sheet Rev. D, "Data Registers".
pub const ADXL355_CODE_MIN: i32 = -(1 << 19);
/// See [`ADXL355_CODE_MIN`].
pub const ADXL355_CODE_MAX: i32 = (1 << 19) - 1;

/// ADXL355 FIFO depth in xyz samples (96 entries / 3 axes). Source: ADXL354/ADXL355
/// data sheet Rev. D, "FIFO".
pub const ADXL355_FIFO_SAMPLES: usize = 32;

/// Power-path ADC code range: 12-bit unsigned (bus, hold-up and thermistor channels;
/// the preset scales are full-scale/4096).
pub const ADC12_CODE_MAX: u16 = 4095;

/// Rounds half away from zero and saturates into [lo, hi] (non-finite → 0).
#[inline]
fn quantize(v: f64, lo: f64, hi: f64) -> f64 {
    if !v.is_finite() {
        return 0.0;
    }
    fmath::fmin(fmath::fmax(fmath::round(v), lo), hi)
}

/// Gap ADC code for gap `g_m`: the inverse of the controller linearisation
/// g = c0 + c1·code + c2·code² (PHYSICS §13), rounded and saturated to 16 bits. A zero
/// or non-finite scale gives code 0.
pub fn gap_code(poly_m: &[f64; 3], g_m: f64) -> u16 {
    let [c0, c1, c2] = *poly_m;
    let dg = g_m - c0;
    let code = if c2 == 0.0 {
        if c1 == 0.0 || !c1.is_finite() {
            return 0;
        }
        dg / c1
    } else {
        // Root of c2·x² + c1·x − dg = 0 continuous with the linear solution, in the
        // cancellation-free form x = 2·dg / (c1 + √(c1² + 4·c2·dg)).
        let disc = c1 * c1 + 4.0 * c2 * dg;
        let den = c1 + fmath::sqrt(fmath::fmax(disc, 0.0));
        if den == 0.0 || !den.is_finite() {
            return 0;
        }
        2.0 * dg / den
    };
    quantize(code, 0.0, u16::MAX as f64) as u16
}

/// Signed current code for `i_a` at `a_per_code` (12-bit, saturating; zero scale → 0).
pub fn current_code(i_a: f64, a_per_code: f64) -> i16 {
    if a_per_code == 0.0 || !a_per_code.is_finite() {
        return 0;
    }
    quantize(
        i_a / a_per_code,
        CURRENT_CODE_MIN as f64,
        CURRENT_CODE_MAX as f64,
    ) as i16
}

/// 16-bit accelerometer code (carriage IMU, rail accelerometer); zero scale → 0.
pub fn acc16_code(a_mps2: f64, mps2_per_lsb: f64) -> i32 {
    if mps2_per_lsb == 0.0 || !mps2_per_lsb.is_finite() {
        return 0;
    }
    quantize(
        a_mps2 / mps2_per_lsb,
        ACC16_CODE_MIN as f64,
        ACC16_CODE_MAX as f64,
    ) as i32
}

/// ADXL355 20-bit code for `a_mps2` at `mps2_per_lsb` (clipped at ±2.048 g; zero scale
/// → 0). Shared by `SimBoard` and `DetectorReplay` so both quantise identically.
pub fn adxl355_code(a_mps2: f64, mps2_per_lsb: f64) -> i32 {
    if mps2_per_lsb == 0.0 || !mps2_per_lsb.is_finite() {
        return 0;
    }
    quantize(
        a_mps2 / mps2_per_lsb,
        ADXL355_CODE_MIN as f64,
        ADXL355_CODE_MAX as f64,
    ) as i32
}

/// Unsigned 12-bit power-path code for `v` at `per_code` (zero scale → 0).
pub fn adc12_code(v: f64, per_code: f64) -> u16 {
    if per_code == 0.0 || !per_code.is_finite() {
        return 0;
    }
    quantize(v / per_code, 0.0, ADC12_CODE_MAX as f64) as u16
}

/// Signed 16-bit code for `v` at `per_code` (bus current, handle load cell; zero scale
/// → 0).
pub fn i16_code(v: f64, per_code: f64) -> i16 {
    if per_code == 0.0 || !per_code.is_finite() {
        return 0;
    }
    quantize(v / per_code, i16::MIN as f64, i16::MAX as f64) as i16
}

/// Encoder counts for position `x_m` at `nm_per_count` (floor, like an incremental
/// encoder's edge count; resolution ≤ 0 → 0).
pub fn encoder_counts(x_m: f64, nm_per_count: i32) -> i64 {
    if nm_per_count <= 0 || !x_m.is_finite() {
        return 0;
    }
    fmath::floor(x_m * 1.0e9 / nm_per_count as f64) as i64
}

/// White-noise σ per ADXL355 sample for noise density `n_d` (m/s²/√Hz) at `odr_hz`:
/// σ = n_d·√(ODR/2), so the one-sided PSD of the sampled noise equals the data-sheet
/// density across 0…ODR/2 (PHYSICS §13; ADXL355 22.5 µg/√Hz, core design §C.5).
pub fn adxl355_sigma(n_d_mps2_per_rthz: f64, odr_hz: f64) -> f64 {
    if !(n_d_mps2_per_rthz.is_finite() && odr_hz.is_finite() && odr_hz > 0.0) {
        return 0.0;
    }
    n_d_mps2_per_rthz * fmath::sqrt(odr_hz / 2.0)
}

/// Deterministic sensor errors beyond white noise (physics perspective §1.12). They are
/// zero by default because the scenario schema has no field for them yet; studies set
/// them programmatically.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorErrors {
    /// Static bias of each primary gap probe, m (physics §1.12 range ±10 µm).
    pub gap_bias_m: [f64; MAX_MAGNETS],
    /// Static bias of each redundant gap probe, m.
    pub gap_bias_b_m: [f64; MAX_MAGNETS],
    /// Gap-probe bias thermal drift, m/K, referred to 20 °C and driven by the magnet
    /// thermal node (the probe sits beside the pole).
    pub gap_tempco_m_per_k: f64,
}

/// Per-device sensor noise streams and filter states.
#[derive(Clone, Debug)]
pub struct SensorModels {
    /// Gap-sensor noise, one stream per magnet (primary probe; substream channel i).
    pub gap: [NormalStream; MAX_MAGNETS],
    /// Redundant gap-probe noise (substream channel 8 + i).
    pub gap_b: [NormalStream; MAX_MAGNETS],
    /// Coil-current noise, one stream per magnet.
    pub current: [NormalStream; MAX_MAGNETS],
    /// Structure accelerometer noise (x, y, z).
    pub accel: [NormalStream; 3],
    /// Carriage IMU noise (x, y, z).
    pub imu: [NormalStream; 3],
    /// Rail floor accelerometer noise (x, y, z).
    pub rail_accel: [NormalStream; 3],
    /// LSM phase-current noise (a, b, c).
    pub lsm_current: [NormalStream; 3],
    /// Handle load-cell noise.
    pub handle: NormalStream,
    /// Encoder stream (reserved for interpolation jitter; the 1 µm encoder is
    /// noise-free).
    pub encoder: Xoshiro256pp,
    /// Deterministic errors (bias, drift).
    pub errors: SensorErrors,
    gap_filt: [f64; MAX_MAGNETS],
    gap_filt_primed: bool,
    gap_alpha: f64,
}

impl SensorModels {
    /// Streams for device `device` of a run seeded with `noise_seed`.
    pub fn new(noise_seed: u64, device: u16) -> SensorModels {
        let s = |tag, ch: usize| NormalStream::substream(noise_seed, tag, device, ch as u16);
        // Exact discretisation of the first-order anti-alias at the base tick:
        // α = 1 − e^(−h/τ) (PHYSICS §13, §24).
        let gap_alpha = 1.0 - fmath::exp(-BASE_TICK_S / GAP_ANTI_ALIAS_TAU_S);
        SensorModels {
            gap: core::array::from_fn(|ch| s(StreamTag::GapNoise, ch)),
            gap_b: core::array::from_fn(|ch| s(StreamTag::GapNoise, MAX_MAGNETS + ch)),
            current: core::array::from_fn(|ch| s(StreamTag::CurrentNoise, ch)),
            accel: core::array::from_fn(|ch| s(StreamTag::Accel, ch)),
            imu: core::array::from_fn(|ch| s(StreamTag::Imu, ch)),
            rail_accel: core::array::from_fn(|ch| s(StreamTag::RailAccel, ch)),
            lsm_current: core::array::from_fn(|ch| s(StreamTag::LsmCurrent, ch)),
            handle: s(StreamTag::Handle, 0),
            encoder: substream(noise_seed, StreamTag::Encoder, device, 0),
            errors: SensorErrors::default(),
            gap_filt: [0.0; MAX_MAGNETS],
            gap_filt_primed: false,
            gap_alpha,
        }
    }

    /// Advances the gap anti-alias filters by one base tick with truth `gaps_m` and
    /// returns the filtered gaps (the first call primes the filters at the input, so
    /// there is no start-up transient).
    pub fn filter_gaps(&mut self, gaps_m: &[f64; MAX_MAGNETS]) -> [f64; MAX_MAGNETS] {
        if !self.gap_filt_primed {
            self.gap_filt = *gaps_m;
            self.gap_filt_primed = true;
        } else {
            for (f, g) in self.gap_filt.iter_mut().zip(gaps_m.iter()) {
                *f += self.gap_alpha * (g - *f);
            }
        }
        self.gap_filt
    }

    /// Primary and redundant gap codes for channel `ch` at filtered gap `g_m` and probe
    /// temperature `temp_c`.
    pub fn gap_codes(&mut self, cal: &SensorCal, ch: usize, g_m: f64, temp_c: f64) -> (u16, u16) {
        let drift = self.errors.gap_tempco_m_per_k * (temp_c - 20.0);
        let sigma = cal.gap_noise_m;
        let na = self.gap.get_mut(ch).map_or(0.0, NormalStream::next_normal);
        let bias_a = self.errors.gap_bias_m.get(ch).copied().unwrap_or(0.0);
        let a = gap_code(&cal.gap_poly_m, g_m + bias_a + drift + sigma * na);
        let b = if cal.gap_redundant {
            let nb = self
                .gap_b
                .get_mut(ch)
                .map_or(0.0, NormalStream::next_normal);
            let bias_b = self.errors.gap_bias_b_m.get(ch).copied().unwrap_or(0.0);
            gap_code(&cal.gap_poly_m, g_m + bias_b + drift + sigma * nb)
        } else {
            0
        };
        (a, b)
    }

    /// Coil current code for channel `ch` at truth current `i_a`.
    pub fn coil_code(&mut self, cal: &SensorCal, ch: usize, i_a: f64) -> i16 {
        let n = self
            .current
            .get_mut(ch)
            .map_or(0.0, NormalStream::next_normal);
        current_code(i_a + cal.coil_noise_a * n, cal.coil_a_per_code)
    }

    /// LSM phase current codes at truth phase currents `i_abc_a`.
    pub fn lsm_codes(&mut self, cal: &SensorCal, i_abc_a: &[f64; 3]) -> [i16; 3] {
        let sigma = LSM_NOISE_LSB * cal.lsm_a_per_code;
        let mut out = [0i16; 3];
        for ((o, i), s) in out.iter_mut().zip(i_abc_a).zip(self.lsm_current.iter_mut()) {
            *o = current_code(i + sigma * s.next_normal(), cal.lsm_a_per_code);
        }
        out
    }

    /// Carriage IMU codes at specific force `a_mps2` (carriage frame, gravity-free apart
    /// from tilt leakage).
    pub fn imu_codes(&mut self, cal: &SensorCal, a_mps2: &[f64; 3]) -> [i32; 3] {
        let mut out = [0i32; 3];
        for ((o, a), s) in out.iter_mut().zip(a_mps2).zip(self.imu.iter_mut()) {
            *o = acc16_code(
                a + cal.imu_noise_mps2 * s.next_normal(),
                cal.imu_mps2_per_lsb,
            );
        }
        out
    }

    /// Rail floor accelerometer codes at floor acceleration `a_mps2`.
    pub fn rail_codes(&mut self, cal: &SensorCal, a_mps2: &[f64; 3]) -> [i32; 3] {
        let mut out = [0i32; 3];
        for ((o, a), s) in out.iter_mut().zip(a_mps2).zip(self.rail_accel.iter_mut()) {
            *o = acc16_code(
                a + cal.rail_accel_noise_mps2 * s.next_normal(),
                cal.rail_accel_mps2_per_lsb,
            );
        }
        out
    }

    /// One structure ADXL355 sample at acceleration `a_mps2` (floor + site).
    pub fn adxl355_codes(&mut self, cal: &SensorCal, a_mps2: &[f64; 3]) -> [i32; 3] {
        let sigma = adxl355_sigma(cal.accel_noise_mps2_per_rthz, cal.accel_odr_hz);
        let mut out = [0i32; 3];
        for ((o, a), s) in out.iter_mut().zip(a_mps2).zip(self.accel.iter_mut()) {
            *o = adxl355_code(a + sigma * s.next_normal(), cal.accel_mps2_per_lsb);
        }
        out
    }

    /// Handle load-cell code at hand force `f_n` (0 when no cell is fitted).
    pub fn handle_code(&mut self, cal: &SensorCal, f_n: f64) -> i16 {
        if cal.handle_n_per_code <= 0.0 {
            return 0;
        }
        let n = self.handle.next_normal();
        i16_code(f_n + cal.handle_noise_n * n, cal.handle_n_per_code)
    }
}

/// The ADXL355 FIFO: 32 xyz samples; a full FIFO drops its oldest sample and flags the
/// next delivered sample as `overrun` (the seismic front end raises AccelLost on the
/// flag or a sequence gap).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Adxl355Fifo {
    q: VecDeque<AccelRaw>,
    seq: u32,
    overrun: bool,
}

impl Adxl355Fifo {
    /// An empty FIFO.
    pub fn new() -> Adxl355Fifo {
        Adxl355Fifo::default()
    }

    /// Queues one sample with the next sequence number.
    pub fn push(&mut self, xyz: [i32; 3]) {
        if self.q.len() >= ADXL355_FIFO_SAMPLES {
            self.q.pop_front();
            self.overrun = true;
        }
        let s = AccelRaw {
            xyz,
            seq: self.seq,
            overrun: core::mem::take(&mut self.overrun),
        };
        self.seq = self.seq.wrapping_add(1);
        self.q.push_back(s);
    }

    /// Moves up to `out.len()` samples into `out`, oldest first; returns the count.
    pub fn drain(&mut self, out: &mut [AccelRaw]) -> usize {
        let mut n = 0;
        for o in out.iter_mut() {
            match self.q.pop_front() {
                Some(s) => {
                    *o = s;
                    n += 1;
                }
                None => break,
            }
        }
        n
    }

    /// Queued samples.
    pub fn len(&self) -> usize {
        self.q.len()
    }

    /// True when nothing is queued.
    pub fn is_empty(&self) -> bool {
        self.q.is_empty()
    }

    /// Sequence number the next sample will carry.
    pub fn next_seq(&self) -> u32 {
        self.seq
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cal() -> SensorCal {
        fabrail_params::presets::door().design.sensors
    }

    #[test]
    fn devices_get_distinct_streams() {
        let mut a = SensorModels::new(42, 0);
        let mut b = SensorModels::new(42, 1);
        assert_ne!(a.gap[0].next_normal(), b.gap[0].next_normal());
        let mut c = SensorModels::new(42, 0);
        let mut a2 = SensorModels::new(42, 0);
        assert_eq!(c.accel[2].next_normal(), a2.accel[2].next_normal());
        let mut p = SensorModels::new(42, 0);
        assert_ne!(
            p.gap[0].next_normal(),
            p.gap_b[0].next_normal(),
            "redundant probe has its own stream"
        );
    }

    #[test]
    fn gap_code_inverts_the_linearisation() {
        // 16-bit over 20 mm (core design §C.5): 5 mm → 16383.75 → 16384.
        let lin = [0.0, 0.020 / 65_535.0, 0.0];
        assert_eq!(gap_code(&lin, 5.0e-3), 16_384);
        assert_eq!(gap_code(&lin, -1.0), 0, "saturates low");
        assert_eq!(gap_code(&lin, 1.0), u16::MAX, "saturates high");
        assert_eq!(gap_code(&[0.0, 0.0, 0.0], 1.0e-3), 0, "zero scale");
        // Quadratic: the code maps back through the polynomial to within half an LSB.
        let quad = [1.0e-3, 2.5e-7, 1.0e-12];
        let g = 7.3e-3;
        let c = gap_code(&quad, g) as f64;
        let back = quad[0] + quad[1] * c + quad[2] * c * c;
        let lsb = quad[1] + 2.0 * quad[2] * c;
        assert!(
            (back - g).abs() <= 0.5 * lsb + 1e-15,
            "(E) half an LSB, PHYSICS §13: {back} vs {g}"
        );
    }

    #[test]
    fn current_and_accel_codes_saturate() {
        let c = cal();
        assert_eq!(current_code(0.0, c.coil_a_per_code), 0);
        assert_eq!(current_code(1.0e6, c.coil_a_per_code), CURRENT_CODE_MAX);
        assert_eq!(current_code(-1.0e6, c.coil_a_per_code), CURRENT_CODE_MIN);
        assert_eq!(current_code(f64::NAN, c.coil_a_per_code), 0);
        // 1 g on the ADXL355 at 3.9 µg/LSB = 256 410 LSB (data sheet sensitivity).
        let one_g = adxl355_code(fabrail_params::derive::G0_MPS2, c.accel_mps2_per_lsb);
        assert_eq!(one_g, 256_410, "(E) 1/3.9e-6 rounded, ADXL355 Rev. D");
        assert_eq!(adxl355_code(1.0e3, c.accel_mps2_per_lsb), ADXL355_CODE_MAX);
        assert_eq!(acc16_code(-1.0e3, c.imu_mps2_per_lsb), ACC16_CODE_MIN);
        assert_eq!(encoder_counts(1.2345678e-3, 1000), 1234);
        assert_eq!(
            encoder_counts(-0.5e-6, 1000),
            -1,
            "floor, like an edge counter"
        );
    }

    #[test]
    fn adxl355_noise_density_is_preserved() {
        // σ = n_d·√(ODR/2): 22.5 µg/√Hz at 500 Hz → 355.8 µg rms (PHYSICS §13).
        let s = adxl355_sigma(22.5e-6, 500.0);
        assert!((s - 22.5e-6 * 250.0f64.sqrt()).abs() < 1e-18, "(E) {s}");
        let c = cal();
        let mut m = SensorModels::new(7, 0);
        let n = 20_000;
        let mut acc = 0.0;
        for _ in 0..n {
            let x = m.adxl355_codes(&c, &[0.0; 3])[0] as f64 * c.accel_mps2_per_lsb;
            acc += x * x;
        }
        let rms = (acc / n as f64).sqrt();
        let want = adxl355_sigma(c.accel_noise_mps2_per_rthz, c.accel_odr_hz);
        assert!(
            (rms / want - 1.0).abs() < 0.03,
            "(R) sample rms within 3 % of n_d·√(ODR/2): {rms} vs {want}"
        );
    }

    #[test]
    fn gap_filter_is_first_order_with_the_stated_tau() {
        let mut m = SensorModels::new(1, 0);
        let mut g = [0.0; MAX_MAGNETS];
        m.filter_gaps(&g);
        g[0] = 1.0e-3;
        let y = m.filter_gaps(&g)[0];
        let want = 1.0e-3 * (1.0 - (-BASE_TICK_S / GAP_ANTI_ALIAS_TAU_S).exp());
        assert!((y - want).abs() < 1e-15, "(E) exact ZOH step, PHYSICS §13");
    }

    #[test]
    fn fifo_overrun_drops_oldest_and_flags() {
        let mut f = Adxl355Fifo::new();
        for i in 0..(ADXL355_FIFO_SAMPLES as i32 + 3) {
            f.push([i, 0, 0]);
        }
        assert_eq!(f.len(), ADXL355_FIFO_SAMPLES);
        let mut out = [AccelRaw::default(); 16];
        let n = f.drain(&mut out);
        assert_eq!(n, 16);
        assert_eq!(out[0].xyz[0], 3, "three oldest dropped");
        assert_eq!(out[0].seq, 3);
        let flagged = out.iter().filter(|s| s.overrun).count();
        assert_eq!(flagged, 0, "flag rides on the sample queued after the drop");
        let mut rest = [AccelRaw::default(); 32];
        let n2 = f.drain(&mut rest);
        assert_eq!(n + n2, ADXL355_FIFO_SAMPLES);
        assert!(rest[..n2].iter().any(|s| s.overrun));
    }
}
