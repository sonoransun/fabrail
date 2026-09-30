//! Per-device noise seeds: numpy `SeedSequence(seed).spawn(n)[i]`, reproduced bit for bit (plan §4b; PHYSICS §24).
//!
//! The scene gives device `i` (registry order) the Rust sensor-noise seed that
//! `fabrail.ensemble.member_seeds(seed, n)[i].noise_seed` gives ensemble member `i`:
//! child `i` of `SeedSequence(seed)` is split as every run's seed is
//! (`ensemble.split_seed`), and its child 1 yields `generate_state(1, uint64)[0]`, i.e.
//! `SeedSequence(seed, spawn_key=(i, 1)).generate_state(1, np.uint64)[0]`. This fixes the
//! core substream key's missing device index (plan §4b, safety review item 13): no two
//! devices share a noise stream, and a device's stream does not depend on how many
//! devices the scene has.
//!
//! The algorithm is numpy's `SeedSequence` (numpy/random/bit_generator.pyx, after
//! M. E. O'Neill's `seed_seq_fe`): pool size 4, `hashmix`/`mix` with the constants below;
//! checked against numpy 2.4.3 in the tests.

/// Pool size of `SeedSequence` (numpy `DEFAULT_POOL_SIZE`).
const POOL_SIZE: usize = 4;
/// Hash constants (numpy/random/bit_generator.pyx).
const INIT_A: u32 = 0x43b0_d7e5;
const MULT_A: u32 = 0x931e_8875;
const INIT_B: u32 = 0x8b51_f9dd;
const MULT_B: u32 = 0x58f3_8ded;
const MIX_MULT_L: u32 = 0xca01_f9dd;
const MIX_MULT_R: u32 = 0x4973_f715;
const XSHIFT: u32 = 16;

fn hashmix(value: u32, hash_const: &mut u32) -> u32 {
    let mut v = value ^ *hash_const;
    *hash_const = hash_const.wrapping_mul(MULT_A);
    v = v.wrapping_mul(*hash_const);
    v ^ (v >> XSHIFT)
}

fn mix(x: u32, y: u32) -> u32 {
    let r = MIX_MULT_L
        .wrapping_mul(x)
        .wrapping_sub(MIX_MULT_R.wrapping_mul(y));
    r ^ (r >> XSHIFT)
}

/// numpy `_int_to_uint32_array`: little-endian 32-bit words, `[0]` for zero.
fn words(mut n: u64) -> Vec<u32> {
    if n == 0 {
        return vec![0];
    }
    let mut out = Vec::new();
    while n > 0 {
        out.push((n & 0xffff_ffff) as u32);
        n >>= 32;
    }
    out
}

/// A numpy `SeedSequence` with integer entropy and a spawn key.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct SeedSequence {
    pool: [u32; POOL_SIZE],
}

impl SeedSequence {
    /// `SeedSequence(entropy, spawn_key=spawn_key)`.
    pub(crate) fn new(entropy: u64, spawn_key: &[u64]) -> SeedSequence {
        let mut ent = words(entropy);
        let spawn: Vec<u32> = spawn_key.iter().flat_map(|&k| words(k)).collect();
        // numpy ≥ 1.19 (gh-16539): pad the run entropy to the pool size when spawned.
        if !spawn.is_empty() && ent.len() < POOL_SIZE {
            ent.resize(POOL_SIZE, 0);
        }
        ent.extend(spawn);
        let mut pool = [0u32; POOL_SIZE];
        let mut hc = INIT_A;
        for (i, slot) in pool.iter_mut().enumerate() {
            *slot = hashmix(ent.get(i).copied().unwrap_or(0), &mut hc);
        }
        for i_src in 0..POOL_SIZE {
            for i_dst in 0..POOL_SIZE {
                if i_src != i_dst {
                    let h = hashmix(pool[i_src], &mut hc);
                    pool[i_dst] = mix(pool[i_dst], h);
                }
            }
        }
        for &w in ent.iter().skip(POOL_SIZE) {
            for slot in pool.iter_mut() {
                let h = hashmix(w, &mut hc);
                *slot = mix(*slot, h);
            }
        }
        SeedSequence { pool }
    }

    /// `generate_state(n, np.uint64)`: 2n 32-bit words, paired little-endian.
    pub(crate) fn generate_u64(&self, n: usize) -> Vec<u64> {
        let mut hc = INIT_B;
        let w: Vec<u32> = (0..2 * n)
            .map(|i| {
                let mut v = self.pool[i % POOL_SIZE] ^ hc;
                hc = hc.wrapping_mul(MULT_B);
                v = v.wrapping_mul(hc);
                v ^ (v >> XSHIFT)
            })
            .collect();
        w.chunks_exact(2)
            .map(|p| u64::from(p[0]) | (u64::from(p[1]) << 32))
            .collect()
    }
}

/// Noise seed of device `i` of a scene seeded `scene_seed` (see the module docs).
pub(crate) fn device_noise_seed(scene_seed: u64, i: usize) -> u64 {
    SeedSequence::new(scene_seed, &[i as u64, 1])
        .generate_u64(1)
        .first()
        .copied()
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_numpy_seed_sequence() {
        // numpy 2.4.3: SeedSequence(42).pool.
        assert_eq!(
            SeedSequence::new(42, &[]).pool,
            [0x631d_3606, 0x07ae_90ae, 0x6fc4_be28, 0x2ced_5c75]
        );
        // SeedSequence(42, spawn_key=(0, 1)).pool.
        assert_eq!(
            SeedSequence::new(42, &[0, 1]).pool,
            [0xa708_42d3, 0x834e_5456, 0xf9d2_782d, 0xfba2_9e94]
        );
        // fabrail.ensemble.member_seeds(42, 3)[i].noise_seed.
        for (i, want) in [
            14_351_987_158_862_774_181u64,
            14_845_226_843_314_568_174,
            7_390_130_605_661_587_845,
        ]
        .into_iter()
        .enumerate()
        {
            assert_eq!(device_noise_seed(42, i), want, "device {i}");
        }
        // Multi-word entropy and zero: SeedSequence(2**40 + 5) and SeedSequence(0).
        assert_eq!(
            SeedSequence::new((1u64 << 40) + 5, &[]).generate_u64(1),
            vec![15_772_757_452_565_124_875]
        );
        assert_eq!(
            SeedSequence::new(0, &[]).generate_u64(1),
            vec![15_793_235_383_387_715_774]
        );
    }
}
