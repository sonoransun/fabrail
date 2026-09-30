//! Raw samples to Measurements: calibration, range/NaN checks, dual-gap vote, PosNm from encoder counts, ADXL355 decode (PHYSICS §13). Owner: A1.
//!
//! Frozen API (W0): [`convert`] (stateless, one sample), [`Frontend`] (stateful: encoder
//! jump detection, stuck-gap detection; the fast group uses this), [`FrontendCal`]
//! (f32 copies of `SensorCal` made once in `new()`, so the fast path does no f64 math)
//! and [`adxl355_decode`] (the firmware BSP decodes the SPI burst with it).
//!
//! Conversions (PHYSICS §13; core design §C.5):
//! - Gap: g = c0 + c1·code + c2·code² per probe. A code on the ADC rail (0 or 0xFFFF) or a
//!   channel without its `valid` bit is out of range. With the redundant probe fitted
//!   the two are voted: agreement within [`gap_vote_tol_m`] → their mean; disagreement
//!   → GapDisagree (Degrade) and the probe closer to the channel's previous voted value
//!   (stateless: the primary) is kept; one probe out of range → GapDisagree and the other
//!   is kept; both out → GapRange (SafeLand) and the channel is invalid.
//! - Coil current: code × A/code; a code outside the 12-bit signed range is implausible
//!   (CurrentSense).
//! - Position: `PosNm(counts × nm_per_count)` in saturating integers (PHYSICS §24); the
//!   encoder error flag gives EncoderLost; a per-release jump beyond
//!   [`ENC_V_PLAUSIBLE_MPS`] gives EncoderJump (stateful only).
//! - LSM: phase codes × A/code, then Clarke/Park at the exact electrical angle
//!   (`foc::elec_angle`, PRP-009) when the pole pitch is known ([`FrontendCal::from_params`];
//!   `new(&SensorCal)` has no pitch and reports i_d = i_q = 0).
//! - IMU and rail accelerometer: codes × scale; fitted but silent → ImuLost /
//!   RailAccelLost (Degrade).
//! - Any non-finite result (a corrupt calibration) → NonFinite and the channel invalid.
//!
//! Stuck-gap detection (GapStuck) compares consecutive codes against the coil current
//! and is only meaningful while levitated (a seated or landed carriage legitimately
//! holds its gap while the unseat/lift ramp changes the current), so it is armed by
//! [`Frontend::set_stuck_check`], which the fast-group owner drives from the
//! levitation phase.

use fabrail_params::{SensorCal, SystemParams};
use fabrail_types::{BASE_TICK_S, FastRaw, FaultCode, FaultSet, MAX_MAGNETS, PosNm, Tick, fmath};

use crate::foc;
use crate::msg::Measurements;

/// Gap ADC full-scale code (16-bit converter over the 20 mm range; core design §C.5).
pub const GAP_CODE_MAX: u16 = u16::MAX;

/// Largest magnitude of a 12-bit signed current-sense code (core design §C.5: current
/// sense 12-bit).
pub const CURRENT_CODE_ABS_MAX: i16 = 2048;

/// Largest magnitude of a 16-bit IMU / rail-accelerometer code (±8 g, 16-bit; presets).
pub const ACCEL16_CODE_ABS_MAX: i32 = 32_768;

/// Dual-probe disagreement floor, m. Source: physics perspective §1.12 — each probe
/// carries a bias of up to ±10 µm with thermal drift, so two healthy probes can differ by
/// 20 µm; 50 µm is 2.5× that worst case (design choice).
pub const GAP_VOTE_FLOOR_M: f32 = 50.0e-6;

/// Dual-probe disagreement in units of the in-band noise σ: 10σ per probe (the
/// difference has σ√2, so a false vote needs a 7σ event, p ≈ 1e-12 per sample;
/// design choice).
pub const GAP_VOTE_SIGMA: f32 = 10.0;

/// Encoder plausibility limit, m/s: a per-release position change faster than this is a
/// jump (EncoderJump). Design choice: ≥ 5× the fastest commanded motion (DOOR sensed
/// 0.70 m/s, core §C.2) and ≥ 3× the seismic bumper basis 1.5 m/s (F13).
pub const ENC_V_PLAUSIBLE_MPS: f64 = 5.0;

/// Stuck-gap window: a code frozen this long while the element current changes by more
/// than [`STUCK_DI_A`] is GapStuck. Design choice: 20 ms is ≥ 3 periods of the fastest
/// closed-loop mode (DRAWER roll ω ≈ 270 rad/s) and ≤ the monitor persistence scale.
pub const STUCK_TIME_S: f32 = 0.02;

/// Stuck-gap current change threshold, A (design choice: ≈ 12 % of the smallest
/// continuous rating, DRAWER i_cont = 4 A).
pub const STUCK_DI_A: f32 = 0.5;

/// Disagreement tolerance for the dual-probe vote: max(50 µm, 10·σ_gap), m.
pub fn gap_vote_tol_m(gap_noise_m: f64) -> f32 {
    let sigma = if gap_noise_m.is_finite() && gap_noise_m > 0.0 {
        gap_noise_m as f32
    } else {
        0.0
    };
    fmath::fmaxf(GAP_VOTE_FLOOR_M, GAP_VOTE_SIGMA * sigma)
}

/// f32 working copy of the controller calibration (made in `new()`; PHYSICS §13).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FrontendCal {
    /// Gap linearisation g = c0 + c1·code + c2·code² (m, m/code, m/code²).
    pub gap_poly_m: [f32; 3],
    /// Redundant gap probe fitted.
    pub gap_redundant: bool,
    /// Coil current scale, A/code.
    pub coil_a_per_code: f32,
    /// LSM phase current scale, A/code.
    pub lsm_a_per_code: f32,
    /// Encoder resolution, nm/count.
    pub nm_per_count: i32,
    /// Carriage IMU scale, m/s²/LSB.
    pub imu_mps2_per_lsb: f32,
    /// Rail accelerometer scale, m/s²/LSB.
    pub rail_mps2_per_lsb: f32,
    /// Dual-probe disagreement tolerance, m ([`gap_vote_tol_m`]).
    pub gap_vote_tol_m: f32,
    /// Carriage IMU fitted (a silent IMU is then ImuLost).
    pub imu_present: bool,
    /// Rail accelerometer fitted (a silent sensor is then RailAccelLost).
    pub rail_present: bool,
    /// LSM pole pitch τ_p, nm (0 = unknown: i_d = i_q = 0).
    pub tau_p_nm: i64,
    /// Active levitation elements (channels beyond are ignored); 0 = all.
    pub n_magnets: u8,
}

impl FrontendCal {
    /// Casts the f64 parameter block to f32 (the only place the conversion happens).
    pub fn new(cal: &SensorCal) -> FrontendCal {
        FrontendCal {
            gap_poly_m: [
                cal.gap_poly_m[0] as f32,
                cal.gap_poly_m[1] as f32,
                cal.gap_poly_m[2] as f32,
            ],
            gap_redundant: cal.gap_redundant,
            coil_a_per_code: cal.coil_a_per_code as f32,
            lsm_a_per_code: cal.lsm_a_per_code as f32,
            nm_per_count: cal.nm_per_count,
            imu_mps2_per_lsb: cal.imu_mps2_per_lsb as f32,
            rail_mps2_per_lsb: cal.rail_accel_mps2_per_lsb as f32,
            gap_vote_tol_m: gap_vote_tol_m(cal.gap_noise_m),
            imu_present: cal.imu_present,
            rail_present: cal.rail_accel_present,
            tau_p_nm: 0,
            n_magnets: 0,
        }
    }

    /// Calibration plus the LSM pole pitch and element count from the full parameter
    /// set (the form the fast group should use so i_d/i_q are populated).
    pub fn from_params(p: &SystemParams) -> FrontendCal {
        let tau = p.design.lsm.tau_p_m;
        FrontendCal {
            tau_p_nm: if tau.is_finite() && tau > 0.0 {
                fmath::round(tau * 1.0e9) as i64
            } else {
                0
            },
            n_magnets: p.design.suspension.n_magnets,
            ..FrontendCal::new(&p.design.sensors)
        }
    }

    /// Active channel count.
    fn channels(&self) -> usize {
        if self.n_magnets == 0 {
            MAX_MAGNETS
        } else {
            core::cmp::min(usize::from(self.n_magnets), MAX_MAGNETS)
        }
    }

    /// Gap of one probe code, or `None` when on an ADC rail or non-finite.
    fn gap_of(&self, code: u16) -> Option<f32> {
        if code == 0 || code == GAP_CODE_MAX {
            return None;
        }
        let c = f32::from(code);
        let g = self.gap_poly_m[0] + self.gap_poly_m[1] * c + self.gap_poly_m[2] * c * c;
        if g.is_finite() { Some(g) } else { None }
    }

    /// True when every scale factor is finite (a corrupt block raises NonFinite).
    fn finite(&self) -> bool {
        self.gap_poly_m.iter().all(|v| v.is_finite())
            && self.coil_a_per_code.is_finite()
            && self.lsm_a_per_code.is_finite()
            && self.imu_mps2_per_lsb.is_finite()
            && self.rail_mps2_per_lsb.is_finite()
            && self.gap_vote_tol_m.is_finite()
    }
}

/// Converts one latched raw sample (stateless: no jump or stuck detection).
///
/// Returns the measurements and the faults found (GapRange, GapDisagree, EncoderLost,
/// CurrentSense, ImuLost, RailAccelLost, NonFinite).
pub fn convert(r: &FastRaw, cal: &SensorCal) -> (Measurements, FaultSet) {
    convert_cal(r, &FrontendCal::new(cal))
}

/// [`convert`] with a precomputed [`FrontendCal`] (the fast-path form).
pub fn convert_cal(r: &FastRaw, cal: &FrontendCal) -> (Measurements, FaultSet) {
    convert_with_history(r, cal, None)
}

/// The shared conversion; `prev` holds the previous voted gaps (stateful vote).
fn convert_with_history(
    r: &FastRaw,
    cal: &FrontendCal,
    prev: Option<&[f32; MAX_MAGNETS]>,
) -> (Measurements, FaultSet) {
    let mut f = FaultSet::EMPTY;
    let mut m = Measurements {
        tick: r.tick,
        x: PosNm(r.enc.counts.saturating_mul(i64::from(cal.nm_per_count))),
        referenced: r.enc.referenced,
        enc_ok: !r.enc.err,
        ..Measurements::default()
    };
    if r.enc.err {
        f.set(FaultCode::EncoderLost);
    }
    if !cal.finite() {
        f.set(FaultCode::NonFinite);
        return (m, f);
    }
    let n = cal.channels();

    // Gap channels with the dual-probe vote.
    for (i, (((g_out, code_a), code_b), prev_g)) in m
        .gap_m
        .iter_mut()
        .zip(r.gap.code.iter())
        .zip(r.gap.code_b.iter())
        .zip(prev.map_or([0.0f32; MAX_MAGNETS], |p| *p).iter())
        .enumerate()
        .take(n)
    {
        let bit = 1u8 << i;
        let valid = r.gap.valid & bit != 0;
        if cal.n_magnets == 0 && !valid && *code_a == 0 && *code_b == 0 {
            // Element count unknown (`FrontendCal::new`): an invalid all-zero channel is
            // an unfitted element slot, not a failed probe.
            continue;
        }
        let ga = if valid { cal.gap_of(*code_a) } else { None };
        let gb = if valid && cal.gap_redundant {
            cal.gap_of(*code_b)
        } else {
            None
        };
        let voted = match (ga, gb, cal.gap_redundant) {
            (Some(a), _, false) => Some(a),
            (None, _, false) => {
                f.set(FaultCode::GapRange);
                None
            }
            (Some(a), Some(b), true) => {
                if fmath::absf(a - b) <= cal.gap_vote_tol_m {
                    Some(0.5 * (a + b))
                } else {
                    f.set(FaultCode::GapDisagree);
                    // Keep the probe closer to the previous voted value (primary when
                    // stateless or unprimed).
                    match prev {
                        Some(_) if *prev_g > 0.0 => {
                            if fmath::absf(b - prev_g) < fmath::absf(a - prev_g) {
                                Some(b)
                            } else {
                                Some(a)
                            }
                        }
                        _ => Some(a),
                    }
                }
            }
            (Some(a), None, true) => {
                f.set(FaultCode::GapDisagree);
                Some(a)
            }
            (None, Some(b), true) => {
                f.set(FaultCode::GapDisagree);
                Some(b)
            }
            (None, None, true) => {
                f.set(FaultCode::GapRange);
                None
            }
        };
        if let Some(g) = voted {
            *g_out = g;
            m.gap_ok |= bit;
        }
    }

    // Coil currents.
    for (i_out, code) in m.coil_i_a.iter_mut().zip(r.cur.code.iter()).take(n) {
        if code.unsigned_abs() > CURRENT_CODE_ABS_MAX.unsigned_abs() {
            f.set(FaultCode::CurrentSense);
            continue;
        }
        let i = f32::from(*code) * cal.coil_a_per_code;
        if i.is_finite() {
            *i_out = i;
        } else {
            f.set(FaultCode::NonFinite);
        }
    }

    // LSM phase currents → (i_d, i_q) at the exact electrical angle.
    if cal.tau_p_nm > 0 {
        let i_abc = [
            f32::from(r.lsm.i_abc_code[0]) * cal.lsm_a_per_code,
            f32::from(r.lsm.i_abc_code[1]) * cal.lsm_a_per_code,
            f32::from(r.lsm.i_abc_code[2]) * cal.lsm_a_per_code,
        ];
        let [id, iq] = foc::dq_from_abc(i_abc, m.x, cal.tau_p_nm);
        if id.is_finite() && iq.is_finite() {
            m.id_a = id;
            m.iq_a = iq;
        } else {
            f.set(FaultCode::NonFinite);
        }
    }

    // Carriage IMU and rail accelerometer.
    let (imu, imu_ok) = accel3(r.imu.xyz, r.imu.present, cal.imu_mps2_per_lsb);
    m.imu_mps2 = imu;
    m.imu_ok = imu_ok && cal.imu_present;
    if cal.imu_present && !imu_ok {
        f.set(FaultCode::ImuLost);
    }
    let (rail, rail_ok) = accel3(r.rail.xyz, r.rail.present, cal.rail_mps2_per_lsb);
    m.rail_mps2 = rail;
    m.rail_ok = rail_ok && cal.rail_present;
    if cal.rail_present && !rail_ok {
        f.set(FaultCode::RailAccelLost);
    }
    (m, f)
}

/// Scales a 16-bit 3-axis accelerometer sample; `ok` when present and in range.
fn accel3(xyz: [i32; 3], present: bool, scale: f32) -> ([f32; 3], bool) {
    if !present
        || xyz
            .iter()
            .any(|c| c.unsigned_abs() > ACCEL16_CODE_ABS_MAX.unsigned_abs())
    {
        return ([0.0; 3], false);
    }
    let a = [
        xyz[0] as f32 * scale,
        xyz[1] as f32 * scale,
        xyz[2] as f32 * scale,
    ];
    if a.iter().all(|v| v.is_finite()) {
        (a, true)
    } else {
        ([0.0; 3], false)
    }
}

/// Stateful frontend used by the fast group: adds encoder-jump (EncoderJump) and
/// stuck-gap (GapStuck) detection across releases to [`convert_cal`], and resolves
/// dual-probe disagreements against the previous voted gap.
#[derive(Clone, Copy, Debug, Default)]
pub struct Frontend {
    /// Calibration copy.
    pub cal: FrontendCal,
    /// Encoder counts at the previous release (jump detection).
    pub prev_counts: i64,
    /// A previous sample exists.
    pub primed: bool,
    /// Tick of the previous release.
    pub prev_tick: Tick,
    /// Previous voted gap per channel (0 = none), m.
    pub prev_gap_m: [f32; MAX_MAGNETS],
    /// Stuck-gap detection armed (levitated; see the module docs).
    pub stuck_check: bool,
    /// Per channel: code at the start of the current frozen run.
    pub stuck_code: [u16; MAX_MAGNETS],
    /// Per channel: tick at which the frozen run started.
    pub stuck_since: [Tick; MAX_MAGNETS],
    /// Per channel: coil current at the start of the frozen run, A.
    pub stuck_i_a: [f32; MAX_MAGNETS],
}

impl Frontend {
    /// A frontend for the given calibration (no LSM pitch: i_d = i_q = 0; see
    /// [`Frontend::from_params`]).
    pub fn new(cal: &SensorCal) -> Frontend {
        Frontend::with_cal(FrontendCal::new(cal))
    }

    /// A frontend with the LSM pole pitch and element count (populates i_d/i_q).
    pub fn from_params(p: &SystemParams) -> Frontend {
        Frontend::with_cal(FrontendCal::from_params(p))
    }

    /// A frontend for a prepared calibration.
    pub fn with_cal(cal: FrontendCal) -> Frontend {
        Frontend {
            cal,
            ..Frontend::default()
        }
    }

    /// Arms or disarms the stuck-gap check (true while levitated).
    pub fn set_stuck_check(&mut self, on: bool) {
        if on && !self.stuck_check {
            self.stuck_code = [0; MAX_MAGNETS];
            self.stuck_since = [Tick(0); MAX_MAGNETS];
        }
        self.stuck_check = on;
    }

    /// Converts one sample and runs the cross-release plausibility checks.
    pub fn step(&mut self, r: &FastRaw) -> (Measurements, FaultSet) {
        let prev = if self.primed {
            Some(self.prev_gap_m)
        } else {
            None
        };
        let (mut m, mut f) = convert_with_history(r, &self.cal, prev.as_ref());

        // Encoder jump: |Δcounts|·nm/count beyond v_plausible·Δt.
        if self.primed && !r.enc.err {
            let dt_s = (r.tick.since(self.prev_tick) as f64) * BASE_TICK_S;
            let d_nm = r
                .enc
                .counts
                .saturating_sub(self.prev_counts)
                .saturating_mul(i64::from(self.cal.nm_per_count))
                .unsigned_abs();
            // Allow at least one count of quantisation.
            let lim_nm = (ENC_V_PLAUSIBLE_MPS * dt_s * 1.0e9) as u64
                + u64::from(self.cal.nm_per_count.unsigned_abs());
            if d_nm > lim_nm {
                f.set(FaultCode::EncoderJump);
                m.enc_ok = false;
            }
        }

        // Stuck gap (armed while levitated).
        let n = self.cal.channels();
        if self.stuck_check {
            let t = r.tick;
            for (i, ((((code0, since), i0), code), i_now)) in self
                .stuck_code
                .iter_mut()
                .zip(self.stuck_since.iter_mut())
                .zip(self.stuck_i_a.iter_mut())
                .zip(r.gap.code.iter())
                .zip(m.coil_i_a.iter())
                .enumerate()
                .take(n)
            {
                if *code != *code0 || since.0 == 0 {
                    *code0 = *code;
                    *since = if t.0 == 0 { Tick(1) } else { t };
                    *i0 = *i_now;
                    continue;
                }
                let held_s = (t.since(*since) as f64 * BASE_TICK_S) as f32;
                if held_s >= STUCK_TIME_S && fmath::absf(i_now - *i0) > STUCK_DI_A {
                    f.set(FaultCode::GapStuck);
                    m.gap_ok &= !(1u8 << i);
                }
            }
        }

        for (pg, (g, i)) in self
            .prev_gap_m
            .iter_mut()
            .zip(m.gap_m.iter().zip(0u8..))
            .take(n)
        {
            if m.gap_ok & (1u8 << i) != 0 {
                *pg = *g;
            }
        }
        self.prev_counts = r.enc.counts;
        self.prev_tick = r.tick;
        self.primed = true;
        (m, f)
    }

    /// Forgets the cross-release history (after homing or a reset).
    pub fn reset(&mut self) {
        self.primed = false;
        self.prev_counts = 0;
        self.prev_gap_m = [0.0; MAX_MAGNETS];
        self.stuck_code = [0; MAX_MAGNETS];
        self.stuck_since = [Tick(0); MAX_MAGNETS];
    }
}

/// Decodes one ADXL355 XYZ burst (registers XDATA3..ZDATA1, 0x08–0x10) into
/// sign-extended 20-bit codes. Each axis is `DATA3[7:0] << 12 | DATA2[7:0] << 4 |
/// DATA1[7:4]`, two's complement (Analog Devices ADXL354/ADXL355 data sheet Rev. D,
/// "Data Registers"). At ±2 g range 1 LSB = 3.9 µg (SensorCal::accel_mps2_per_lsb).
pub fn adxl355_decode(raw: &[u8; 9]) -> [i32; 3] {
    let [x3, x2, x1, y3, y2, y1, z3, z2, z1] = *raw;
    [
        decode_axis(x3, x2, x1),
        decode_axis(y3, y2, y1),
        decode_axis(z3, z2, z1),
    ]
}

/// One ADXL355 axis: 20-bit two's complement, left-aligned in 3 bytes.
#[inline]
fn decode_axis(d3: u8, d2: u8, d1: u8) -> i32 {
    let u = ((d3 as u32) << 12) | ((d2 as u32) << 4) | ((d1 as u32) >> 4);
    // Shift the 20-bit value to the top of an i32 and arithmetic-shift back to
    // sign-extend (bit 19 is the sign bit).
    ((u << 12) as i32) >> 12
}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_params::presets;

    #[test]
    fn adxl355_decode_sign_extension() {
        // +1 LSB, -1 LSB (0xFFFFF), most negative (0x80000), most positive (0x7FFFF).
        let raw = [0x00, 0x00, 0x10, 0xFF, 0xFF, 0xF0, 0x80, 0x00, 0x00];
        assert_eq!(adxl355_decode(&raw), [1, -1, -524_288]);
        let raw = [0x7F, 0xFF, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0F];
        // The low nibble of DATA1 is reserved and ignored.
        assert_eq!(adxl355_decode(&raw), [524_287, 16, 0]);
    }

    fn code_for(cal: &SensorCal, g_m: f64) -> u16 {
        (g_m / cal.gap_poly_m[1]).round() as u16
    }

    fn raw_levitated(p: &SystemParams, g_m: f64) -> FastRaw {
        let cal = &p.design.sensors;
        let mut r = FastRaw::default();
        let c = code_for(cal, g_m);
        for i in 0..usize::from(p.design.suspension.n_magnets) {
            r.gap.code[i] = c;
            r.gap.code_b[i] = c;
            r.gap.valid |= 1 << i;
            r.cur.code[i] = 100;
        }
        r.enc.counts = 1_234;
        r.imu.present = true;
        r.imu.xyz = [0, 0, 4096];
        r.rail.present = true;
        r.rail.xyz = [10, -20, 4096];
        r
    }

    #[test]
    fn converts_calibrated_channels() {
        let p = presets::drawer();
        let cal = &p.design.sensors;
        let r = raw_levitated(&p, 2.5e-3);
        let (m, f) = convert_cal(&r, &FrontendCal::from_params(&p));
        assert!(f.is_empty(), "healthy sample has no faults: {f:?}");
        assert_eq!(m.gap_ok, 0x0F, "four voted channels");
        let lsb = cal.gap_poly_m[1];
        for g in &m.gap_m[..4] {
            assert!(
                (f64::from(*g) - 2.5e-3).abs() <= lsb,
                "gap within 1 LSB of 2.5 mm"
            );
        }
        assert_eq!(m.x, PosNm(1_234_000), "counts × nm_per_count");
        let i = 100.0 * cal.coil_a_per_code;
        assert!((f64::from(m.coil_i_a[0]) - i).abs() < 1e-6);
        assert!(m.imu_ok && m.rail_ok);
        let g1 = 4096.0 * cal.imu_mps2_per_lsb;
        assert!((f64::from(m.imu_mps2[2]) - g1).abs() < 1e-5);
    }

    #[test]
    fn unfitted_channels_are_not_faults() {
        // `Frontend::new(&SensorCal)` does not know n_magnets: the four unfitted slots of
        // a 4-element carriage read invalid and zero and must not raise GapRange.
        let p = presets::door();
        let r = raw_levitated(&p, 5.0e-3);
        let (m, f) = convert(&r, &p.design.sensors);
        assert!(f.is_empty(), "no faults from unfitted slots: {f:?}");
        assert_eq!(m.gap_ok, 0x0F);
        // With the element count known, a missing active channel is a fault.
        let mut r2 = r;
        r2.gap.valid = 0x07;
        r2.gap.code[3] = 0;
        r2.gap.code_b[3] = 0;
        let (_, f2) = convert_cal(&r2, &FrontendCal::from_params(&p));
        assert!(f2.contains(FaultCode::GapRange));
    }

    #[test]
    fn dual_gap_vote() {
        let p = presets::door();
        let fcal = FrontendCal::from_params(&p);
        let tol = f64::from(fcal.gap_vote_tol_m);
        assert!((tol - 50e-6).abs() < 1e-9, "DOOR σ 0.5 µm → floor 50 µm");
        let lsb = p.design.sensors.gap_poly_m[1];
        let mut r = raw_levitated(&p, 5.0e-3);
        // Probe B of channel 1 reads 20 µm high: agreement, mean.
        r.gap.code_b[1] = code_for(&p.design.sensors, 5.02e-3);
        let (m, f) = convert_cal(&r, &fcal);
        assert!(f.is_empty());
        assert!(
            (f64::from(m.gap_m[1]) - 5.01e-3).abs() <= lsb,
            "mean of agreeing probes"
        );
        // Probe B of channel 2 reads 0.4 mm high: disagreement, primary kept (stateless).
        r.gap.code_b[2] = code_for(&p.design.sensors, 5.4e-3);
        let (m, f) = convert_cal(&r, &fcal);
        assert!(f.contains(FaultCode::GapDisagree), "disagreement raised");
        assert!((f64::from(m.gap_m[2]) - 5.0e-3).abs() <= lsb);
        assert_eq!(m.gap_ok & 0x04, 0x04, "channel stays valid (Degrade)");
        // Stateful: primary drifts away instead; the probe matching history wins.
        let mut fe = Frontend::from_params(&p);
        let r0 = raw_levitated(&p, 5.0e-3);
        let _ = fe.step(&r0);
        let mut r1 = r0;
        r1.tick = Tick(2);
        r1.gap.code[3] = code_for(&p.design.sensors, 5.6e-3);
        let (m1, f1) = fe.step(&r1);
        assert!(f1.contains(FaultCode::GapDisagree));
        assert!(
            (f64::from(m1.gap_m[3]) - 5.0e-3).abs() <= lsb,
            "history picks probe B"
        );
        // Both probes on the rail: out of range, channel invalid.
        let mut r2 = r0;
        r2.gap.code[0] = 0;
        r2.gap.code_b[0] = GAP_CODE_MAX;
        let (m2, f2) = convert_cal(&r2, &fcal);
        assert!(f2.contains(FaultCode::GapRange));
        assert_eq!(m2.gap_ok & 1, 0);
        // Missing valid bit.
        let mut r3 = r0;
        r3.gap.valid = 0x0E;
        let (m3, f3) = convert_cal(&r3, &fcal);
        assert_eq!(m3.gap_ok & 1, 0);
        assert!(f3.contains(FaultCode::GapRange));
    }

    #[test]
    fn encoder_and_sensor_faults() {
        let p = presets::drawer();
        let mut fe = Frontend::from_params(&p);
        let mut r = raw_levitated(&p, 2.5e-3);
        r.tick = Tick(100);
        let _ = fe.step(&r);
        // 0.1 mm in one 10 kHz release (1 m/s): plausible.
        r.tick = Tick(102);
        r.enc.counts += 100;
        let (m, f) = fe.step(&r);
        assert!(!f.contains(FaultCode::EncoderJump) && m.enc_ok);
        // 1 mm in one release (10 m/s): a jump.
        r.tick = Tick(104);
        r.enc.counts += 1000;
        let (m, f) = fe.step(&r);
        assert!(f.contains(FaultCode::EncoderJump) && !m.enc_ok);
        // Encoder error flag.
        r.tick = Tick(106);
        r.enc.err = true;
        let (m, f) = fe.step(&r);
        assert!(f.contains(FaultCode::EncoderLost) && !m.enc_ok);
        // Current code beyond 12 bits, IMU silent.
        let mut r2 = raw_levitated(&p, 2.5e-3);
        r2.cur.code[0] = 4000;
        r2.imu.present = false;
        let (m2, f2) = convert(&r2, &p.design.sensors);
        assert!(f2.contains(FaultCode::CurrentSense));
        assert!(f2.contains(FaultCode::ImuLost) && !m2.imu_ok);
        // Corrupt calibration.
        let mut bad = p.design.sensors;
        bad.coil_a_per_code = f64::NAN;
        let (_, f3) = convert(&r2, &bad);
        assert!(f3.contains(FaultCode::NonFinite));
    }

    #[test]
    fn stuck_gap_only_when_armed() {
        let p = presets::drawer();
        let mut fe = Frontend::from_params(&p);
        let mut r = raw_levitated(&p, 2.5e-3);
        // Unarmed (seated/landed): frozen code with a current ramp is legitimate.
        for k in 0..600u64 {
            r.tick = Tick(2 * k + 2);
            r.cur.code[0] = (k as i16) * 2;
            let (_, f) = fe.step(&r);
            assert!(!f.contains(FaultCode::GapStuck));
        }
        // Armed (levitated): frozen code while the current moves > 0.5 A for 20 ms.
        fe.set_stuck_check(true);
        let mut hit = false;
        for k in 600..1200u64 {
            r.tick = Tick(2 * k + 2);
            r.cur.code[0] = (k as i16 % 1000) * 2;
            let (m, f) = fe.step(&r);
            if f.contains(FaultCode::GapStuck) {
                hit = true;
                assert_eq!(m.gap_ok & 1, 0, "stuck channel invalidated");
                break;
            }
        }
        assert!(hit, "GapStuck raised while armed");
    }

    #[test]
    fn lsm_dq_from_phase_codes() {
        let p = presets::door();
        let fcal = FrontendCal::from_params(&p);
        assert_eq!(fcal.tau_p_nm, 25_000_000, "DOOR τ_p 25 mm");
        let mut r = FastRaw::default();
        // Position at θ_e = 0 with a pure q-axis current of 1 A: i_a = 0, i_b = +√3/2,
        // i_c = −√3/2 (amplitude-invariant, q leads d by 90°).
        let k = 1.0 / p.design.sensors.lsm_a_per_code;
        r.lsm.i_abc_code = [
            0,
            (0.866_025 * k).round() as i16,
            (-0.866_025 * k).round() as i16,
        ];
        let (m, _) = convert_cal(&r, &fcal);
        assert!((m.iq_a - 1.0).abs() < 5e-3, "i_q = 1 A: {}", m.iq_a);
        assert!(m.id_a.abs() < 5e-3, "i_d = 0: {}", m.id_a);
    }
}
