//! SplitMix64 seeding, Xoshiro256++ streams, named substreams and Box-Muller normals (in-repo, stable). Owner: A7 (complete in Wave 0, W0-B3).
//!
//! All Rust randomness lives here (core design §A.4 item 5; PHYSICS §24). The generators are owned
//! in-repo, so their streams are stable across Rust, crate and platform versions (unlike
//! numpy `Generator` streams, NEP 19). Python owns scenario synthesis; Rust owns sensor
//! noise and fault injection.
//!
//! * [`SplitMix64`]: Steele, Lea & Flood, "Fast splittable pseudorandom number
//!   generators", OOPSLA 2014; constants as in Vigna's reference `splitmix64.c`.
//! * [`Xoshiro256pp`]: Blackman & Vigna, "Scrambled linear pseudorandom number
//!   generators", ACM TOMS 47(4), 2021 (`xoshiro256plusplus.c`). Seeded by expanding a
//!   64-bit seed through SplitMix64, as the authors recommend.
//! * Named substreams: one independent stream per (noise seed, [`StreamTag`], device,
//!   channel), seeded from `noise_seed ^ key` with
//!   `key = tag << 32 | device << 16 | channel` ([`substream_key`]). With device 0 this is
//!   exactly the core formula `noise_seed ^ (tag << 32) ^ ch`; the device field lets a
//!   building scene run many devices from one seed without stream collisions
//!   (plan §4b; scenes normally also give each device its own `SeedSequence` child).
//! * Normals: Box–Muller, 2 uniforms per 2 normals, the second cached
//!   ([`NormalStream`]). Transcendentals go through `fabrail_types::fmath` (soft-float
//!   libm) so the bits are identical on every host.

use fabrail_types::fmath;

/// The SplitMix64 increment (golden-ratio constant 2⁶⁴/φ, odd).
const SM64_GAMMA: u64 = 0x9E37_79B9_7F4A_7C15;

/// SplitMix64 generator: a 64-bit Weyl sequence through a bijective finaliser.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    /// A generator whose first output is `mix(seed + γ)`.
    #[inline]
    pub const fn new(seed: u64) -> SplitMix64 {
        SplitMix64 { state: seed }
    }

    /// Next 64-bit output.
    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(SM64_GAMMA);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}

/// Xoshiro256++ generator (period 2²⁵⁶ − 1).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Xoshiro256pp {
    s: [u64; 4],
}

impl Xoshiro256pp {
    /// A generator with the raw state `s`. An all-zero state (the one fixed point) is
    /// replaced by the SplitMix64 expansion of 0, so the generator never sticks.
    pub fn from_state(s: [u64; 4]) -> Xoshiro256pp {
        if s == [0; 4] {
            return Xoshiro256pp::from_seed(0);
        }
        Xoshiro256pp { s }
    }

    /// A generator seeded by expanding `seed` through four SplitMix64 outputs.
    pub fn from_seed(seed: u64) -> Xoshiro256pp {
        let mut sm = SplitMix64::new(seed);
        let s = [sm.next_u64(), sm.next_u64(), sm.next_u64(), sm.next_u64()];
        // SplitMix64 is a bijection of distinct Weyl states, so four consecutive
        // outputs are never all zero.
        Xoshiro256pp { s }
    }

    /// The raw state (for checkpoints and replay records).
    #[inline]
    pub const fn state(&self) -> [u64; 4] {
        self.s
    }

    /// Next 64-bit output.
    #[inline]
    pub fn next_u64(&mut self) -> u64 {
        let s = &mut self.s;
        let result = s[0].wrapping_add(s[3]).rotate_left(23).wrapping_add(s[0]);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        result
    }

    /// Uniform f64 in [0, 1): the top 53 bits times 2⁻⁵³ (exact, every value a
    /// multiple of 2⁻⁵³).
    #[inline]
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Uniform f64 in (0, 1]: `1 − next_f64()` (safe for ln).
    #[inline]
    pub fn next_f64_open0(&mut self) -> f64 {
        1.0 - self.next_f64()
    }

    /// Uniform integer in [0, n) by 128-bit multiply-high (Lemire; bias < n/2⁶⁴,
    /// negligible for fault-injection choices). Returns 0 for n = 0.
    #[inline]
    pub fn next_below(&mut self, n: u64) -> u64 {
        ((self.next_u64() as u128 * n as u128) >> 64) as u64
    }
}

/// Named randomness consumers (the substream `tag`). Discriminants are frozen: changing
/// one changes every recorded anchor.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[repr(u8)]
pub enum StreamTag {
    /// Gap-sensor noise (one channel per magnet).
    GapNoise = 1,
    /// Coil-current sense noise (one channel per magnet).
    CurrentNoise = 2,
    /// Encoder (interpolation jitter, error injection).
    Encoder = 3,
    /// Structure ADXL355 noise (channels x, y, z).
    Accel = 4,
    /// Carriage IMU noise (channels x, y, z).
    Imu = 5,
    /// Power-path sense noise (bus, hold-up, temperatures).
    Power = 6,
    /// Fault-injection choices.
    FaultInj = 7,
    /// Rail-mounted floor accelerometer noise (channels x, y, z).
    RailAccel = 8,
    /// Handle load-cell noise.
    Handle = 9,
    /// LSM phase-current sense noise (channels a, b, c).
    LsmCurrent = 10,
}

impl StreamTag {
    /// Every tag, in discriminant order.
    pub const ALL: &'static [StreamTag] = &[
        StreamTag::GapNoise,
        StreamTag::CurrentNoise,
        StreamTag::Encoder,
        StreamTag::Accel,
        StreamTag::Imu,
        StreamTag::Power,
        StreamTag::FaultInj,
        StreamTag::RailAccel,
        StreamTag::Handle,
        StreamTag::LsmCurrent,
    ];
}

/// Substream key `tag << 32 | device << 16 | channel` (bits 40–63 stay zero).
#[inline]
pub const fn substream_key(tag: StreamTag, device: u16, channel: u16) -> u64 {
    ((tag as u64) << 32) | ((device as u64) << 16) | channel as u64
}

/// The independent stream for (noise seed, tag, device, channel):
/// `Xoshiro256pp::from_seed(noise_seed ^ substream_key(..))`.
pub fn substream(noise_seed: u64, tag: StreamTag, device: u16, channel: u16) -> Xoshiro256pp {
    Xoshiro256pp::from_seed(noise_seed ^ substream_key(tag, device, channel))
}

/// Standard-normal stream by Box–Muller: each pair of uniforms (u₁ ∈ (0, 1],
/// u₂ ∈ [0, 1)) yields r·cos θ, then the cached r·sin θ, with r = √(−2 ln u₁),
/// θ = 2π·u₂.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalStream {
    rng: Xoshiro256pp,
    spare: Option<f64>,
}

impl NormalStream {
    /// A normal stream over `rng`.
    pub const fn new(rng: Xoshiro256pp) -> NormalStream {
        NormalStream { rng, spare: None }
    }

    /// The named substream as a normal stream (see [`substream`]).
    pub fn substream(noise_seed: u64, tag: StreamTag, device: u16, channel: u16) -> NormalStream {
        NormalStream::new(substream(noise_seed, tag, device, channel))
    }

    /// Next standard-normal variate.
    pub fn next_normal(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        let u1 = self.rng.next_f64_open0();
        let u2 = self.rng.next_f64();
        let r = fmath::sqrt(-2.0 * fmath::ln(u1));
        let theta = 2.0 * core::f64::consts::PI * u2;
        self.spare = Some(r * fmath::sin(theta));
        r * fmath::cos(theta)
    }

    /// Next normal variate with standard deviation `sigma` (mean 0).
    #[inline]
    pub fn next_scaled(&mut self, sigma: f64) -> f64 {
        sigma * self.next_normal()
    }

    /// The underlying uniform generator (for mixed use, e.g. fault choices).
    pub fn uniform(&mut self) -> &mut Xoshiro256pp {
        &mut self.rng
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SplitMix64 from seed 1234567: Vigna's reference `splitmix64.c` test values
    /// (also the Java `SplittableRandom` vector). Tolerance (E): bit-exact.
    #[test]
    fn splitmix64_reference_vector() {
        let mut sm = SplitMix64::new(1_234_567);
        let expected: [u64; 5] = [
            6_457_827_717_110_365_317,
            3_203_168_211_198_807_973,
            9_817_491_932_198_370_423,
            4_593_380_528_125_082_431,
            16_408_922_859_458_223_821,
        ];
        for e in expected {
            assert_eq!(sm.next_u64(), e, "(E) bit-exact, splitmix64.c reference");
        }
    }

    /// Xoshiro256++ from the raw state [1, 2, 3, 4]: the reference vector of
    /// `xoshiro256plusplus.c` (also rand_xoshiro's test). The first value is
    /// rotl(1 + 4, 23) + 1 = 41943041. Tolerance (E): bit-exact.
    #[test]
    fn xoshiro256pp_reference_vector() {
        let mut x = Xoshiro256pp::from_state([1, 2, 3, 4]);
        let expected: [u64; 8] = [
            41_943_041,
            58_720_359,
            3_588_806_011_781_223,
            3_591_011_842_654_386,
            9_228_616_714_210_784_205,
            9_973_669_472_204_895_162,
            14_011_001_112_246_962_877,
            12_406_186_145_184_390_807,
        ];
        for e in expected {
            assert_eq!(
                x.next_u64(),
                e,
                "(E) bit-exact, xoshiro256plusplus.c reference"
            );
        }
    }

    /// Stream stability: the first 8 outputs of the seeded generator are pinned
    /// (core design §E.2 `rng::stream_stability`). Reference values computed by an
    /// independent Python implementation of the same algorithms. Tolerance (E).
    #[test]
    fn stream_stability() {
        let mut x = Xoshiro256pp::from_seed(42);
        let expected: [u64; 8] = [
            15_021_278_609_987_233_951,
            5_881_210_131_331_364_753,
            18_149_643_915_985_481_100,
            12_933_668_939_759_105_464,
            14_637_574_242_682_825_331,
            10_848_501_901_068_131_965,
            2_312_344_417_745_909_078,
            11_162_538_943_635_311_430,
        ];
        for e in expected {
            assert_eq!(x.next_u64(), e, "(E) pinned stream, seed 42");
        }
        // Named substreams (device 0 reduces to the core formula).
        let mut g = substream(123, StreamTag::GapNoise, 0, 0);
        assert_eq!(g.next_u64(), 15_352_467_212_493_844_425);
        assert_eq!(g.next_u64(), 4_132_799_879_223_186_814);
        let mut a = substream(123, StreamTag::Accel, 3, 2);
        assert_eq!(a.next_u64(), 15_004_810_362_656_730_494);
        assert_eq!(a.next_u64(), 7_357_329_902_872_293_916);
    }

    /// Substreams for different tags, devices and channels start in distinct states and
    /// produce uncorrelated early outputs.
    #[test]
    fn substream_independence() {
        let mut seen = std::collections::BTreeSet::new();
        for &tag in StreamTag::ALL {
            for device in [0u16, 1, 7, 65535] {
                for ch in [0u16, 1, 2, 65535] {
                    let key = substream_key(tag, device, ch);
                    assert!(seen.insert(key), "key collision {key:#x}");
                    assert_eq!(key >> 40, 0);
                }
            }
        }
        // Early outputs of neighbouring substreams: sample correlation of the uniforms
        // over 4096 draws is small (|ρ| < 0.06, ≈ 4σ for n = 4096).
        let n = 4096;
        let mut a = substream(7, StreamTag::GapNoise, 0, 0);
        let mut b = substream(7, StreamTag::GapNoise, 0, 1);
        let mut c = substream(7, StreamTag::GapNoise, 1, 0);
        let xs: Vec<f64> = (0..n).map(|_| a.next_f64() - 0.5).collect();
        for other in [&mut b, &mut c] {
            let ys: Vec<f64> = (0..n).map(|_| other.next_f64() - 0.5).collect();
            let sxy: f64 = xs.iter().zip(&ys).map(|(x, y)| x * y).sum();
            let sxx: f64 = xs.iter().map(|x| x * x).sum();
            let syy: f64 = ys.iter().map(|y| y * y).sum();
            let rho = sxy / (sxx * syy).sqrt();
            assert!(rho.abs() < 0.06, "correlation {rho}");
        }
    }

    /// Box–Muller normals from seed 42 match the Python reference (math.log/cos/sin)
    /// within 1e-14 relative (fmath is soft-float libm, the reference is the host libm).
    #[test]
    fn box_muller_reference_and_moments() {
        let mut ns = NormalStream::new(Xoshiro256pp::from_seed(42));
        let expected = [
            -0.7689930538210061,
            1.6661184587142,
            -0.8684461074702454,
            -2.7391511556643047,
        ];
        for e in expected {
            let z = ns.next_normal();
            assert!((z - e).abs() <= 1e-14 * e.abs(), "{z} vs {e}");
        }
        let mut ns = NormalStream::substream(1, StreamTag::Imu, 0, 0);
        let n = 100_000;
        let (mut s1, mut s2) = (0.0, 0.0);
        for _ in 0..n {
            let z = ns.next_normal();
            s1 += z;
            s2 += z * z;
        }
        let mean = s1 / n as f64;
        let var = s2 / n as f64 - mean * mean;
        assert!(mean.abs() < 0.02, "mean {mean}");
        assert!((var - 1.0).abs() < 0.02, "variance {var}");
    }

    #[test]
    fn uniform_ranges() {
        let mut x = Xoshiro256pp::from_seed(9);
        for _ in 0..10_000 {
            let u = x.next_f64();
            assert!((0.0..1.0).contains(&u));
            let v = x.next_f64_open0();
            assert!(v > 0.0 && v <= 1.0);
            assert!(x.next_below(7) < 7);
        }
        assert_eq!(x.next_below(0), 0);
        let z = Xoshiro256pp::from_state([0; 4]);
        assert_ne!(z.state(), [0; 4], "zero state is replaced");
    }
}
