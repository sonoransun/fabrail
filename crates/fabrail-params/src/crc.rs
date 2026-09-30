//! CRC32 and the safety-block seal (IEC 60730 Class B style parameter integrity; SAF-015).
//!
//! `seal` refreshes the diverse calibration copies from the design block and recomputes
//! `SafetyLimits::crc` over a canonical little-endian encoding of the safety block (every
//! field except `crc`, in declaration order; f64 as IEEE-754 bits, integers at their
//! declared width). At boot the supervisor checks the seal; a CRC or copy mismatch raises
//! `ParamCrc` and the carriage refuses to lift.
//!
//! `crc32` is CRC-32/ISO-HDLC: reflected polynomial 0xEDB88320, init and xorout
//! 0xFFFFFFFF; check value of "123456789" = 0xCBF43926.

use crate::{ParamError, SafetyLimits, SystemParams};

/// Incremental CRC-32/ISO-HDLC state (bitwise, no table).
#[derive(Clone, Copy, Debug)]
struct Crc32(u32);

impl Crc32 {
    const fn new() -> Crc32 {
        Crc32(0xFFFF_FFFF)
    }

    fn update(&mut self, bytes: &[u8]) {
        let mut crc = self.0;
        for &b in bytes {
            crc ^= u32::from(b);
            for _ in 0..8 {
                let mask = (crc & 1).wrapping_neg();
                crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
            }
        }
        self.0 = crc;
    }

    fn f64(&mut self, v: f64) {
        self.update(&v.to_bits().to_le_bytes());
    }

    const fn finish(self) -> u32 {
        !self.0
    }
}

/// CRC-32/ISO-HDLC (the zlib/Ethernet CRC), bitwise, no table.
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut c = Crc32::new();
    c.update(bytes);
    c.finish()
}

/// CRC over the canonical encoding of `l` (excluding `l.crc`).
///
/// The exhaustive destructuring makes adding a field to `SafetyLimits` a compile error
/// here until the encoding covers it.
pub fn safety_crc(l: &SafetyLimits) -> u32 {
    let SafetyLimits {
        mass_rated_kg,
        e_max_j,
        p_max_kgmps,
        v_open_max_mps,
        v_close_max_mps,
        v_open_sensed_mps,
        v_close_sensed_mps,
        f_drive_max_n,
        t_drive_max_s,
        f_static_n,
        f_static_sensed_n,
        f_static_final_n,
        t_static_final_s,
        f_close_last50_n,
        last_zone_m,
        i_coil_cont_a,
        i_coil_pulse_a,
        t_coil_pulse_s,
        i_lsm_max_a,
        gap_min_m,
        gap_max_m,
        x_min_m,
        x_max_m,
        overspeed_ratio,
        overspeed_t_s,
        vel_window_s,
        v_runaway_mps,
        v_hold_min_v,
        k_f_copy_n_per_a,
        adc_a_per_code_copy,
        lsm_a_per_code_copy,
        nm_per_count_copy,
        presence_copy,
        watchdog_ticks,
        heartbeat_max_ticks,
        isolate,
        unattended,
        crc: _,
    } = *l;
    let mut c = Crc32::new();
    for v in [
        mass_rated_kg,
        e_max_j,
        p_max_kgmps,
        v_open_max_mps,
        v_close_max_mps,
        v_open_sensed_mps,
        v_close_sensed_mps,
        f_drive_max_n,
        t_drive_max_s,
        f_static_n,
        f_static_sensed_n,
        f_static_final_n,
        t_static_final_s,
        f_close_last50_n,
        last_zone_m,
        i_coil_cont_a,
        i_coil_pulse_a,
        t_coil_pulse_s,
        i_lsm_max_a,
        gap_min_m,
        gap_max_m,
        x_min_m,
        x_max_m,
        overspeed_ratio,
        overspeed_t_s,
        vel_window_s,
        v_runaway_mps,
        v_hold_min_v,
        k_f_copy_n_per_a,
        adc_a_per_code_copy,
        lsm_a_per_code_copy,
    ] {
        c.f64(v);
    }
    c.update(&nm_per_count_copy.to_le_bytes());
    c.update(&[presence_copy.as_u8()]);
    c.update(&watchdog_ticks.to_le_bytes());
    c.update(&heartbeat_max_ticks.to_le_bytes());
    let crate::IsolateLimits {
        stroke_m,
        bumper_ke_j,
        f_cont_n,
    } = isolate;
    c.f64(stroke_m);
    c.f64(bumper_ke_j);
    c.f64(f_cont_n);
    let crate::UnattendedClose {
        f_pinch_n,
        k_series_n_per_m,
        detect_max_s,
        max_attempts,
    } = unattended;
    c.f64(f_pinch_n);
    c.f64(k_series_n_per_m);
    c.f64(detect_max_s);
    c.update(&[max_attempts]);
    c.finish()
}

/// Refreshes the diverse copies from the design block and recomputes `safety.crc`.
pub fn seal(p: &mut SystemParams) {
    let d = &p.design;
    let l = &mut p.safety;
    l.k_f_copy_n_per_a = d.lsm.k_f_n_per_a;
    l.adc_a_per_code_copy = d.sensors.coil_a_per_code;
    l.lsm_a_per_code_copy = d.sensors.lsm_a_per_code;
    l.nm_per_count_copy = d.sensors.nm_per_count;
    l.presence_copy = d.presence;
    l.crc = safety_crc(l);
}

/// Bitwise f64 equality (a diverse copy must be the same bits, not merely close).
fn same(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits()
}

/// Checks the seal: the diverse copies equal the design values and `safety.crc`
/// matches. The first mismatch is returned as `path: reason` (SAF-015).
pub fn verify_seal(p: &SystemParams) -> Result<(), ParamError> {
    let d = &p.design;
    let l = &p.safety;
    if !same(l.k_f_copy_n_per_a, d.lsm.k_f_n_per_a) {
        return Err(ParamError::new(
            "safety.k_f_copy_n_per_a",
            "diverse copy differs from design.lsm.k_f_n_per_a (SAF-015)",
        ));
    }
    if !same(l.adc_a_per_code_copy, d.sensors.coil_a_per_code) {
        return Err(ParamError::new(
            "safety.adc_a_per_code_copy",
            "diverse copy differs from design.sensors.coil_a_per_code (SAF-015)",
        ));
    }
    if !same(l.lsm_a_per_code_copy, d.sensors.lsm_a_per_code) {
        return Err(ParamError::new(
            "safety.lsm_a_per_code_copy",
            "diverse copy differs from design.sensors.lsm_a_per_code (SAF-015)",
        ));
    }
    if l.nm_per_count_copy != d.sensors.nm_per_count {
        return Err(ParamError::new(
            "safety.nm_per_count_copy",
            "diverse copy differs from design.sensors.nm_per_count (SAF-015)",
        ));
    }
    if l.presence_copy != d.presence {
        return Err(ParamError::new(
            "safety.presence_copy",
            "diverse copy differs from design.presence (SAF-003, SAF-015)",
        ));
    }
    if l.crc != safety_crc(l) {
        return Err(ParamError::new(
            "safety.crc",
            "CRC32 of the safety block does not match; reseal after editing (SAF-015)",
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::presets;

    #[test]
    fn crc32_check_value() {
        assert_eq!(
            crc32(b"123456789"),
            0xCBF4_3926,
            "CRC-32/ISO-HDLC check value"
        );
        assert_eq!(crc32(b""), 0);
        assert_ne!(crc32(b"123456788"), crc32(b"123456789"));
        let mut c = Crc32::new();
        c.update(b"1234");
        c.update(b"56789");
        assert_eq!(
            c.finish(),
            0xCBF4_3926,
            "incremental update equals one-shot"
        );
    }

    /// Every safety-block field paired with a mutator that flips its value.
    type Mutator = fn(&mut SafetyLimits);

    fn mutators() -> [(&'static str, Mutator); 42] {
        fn bump(v: &mut f64) {
            *v = f64::from_bits(v.to_bits() ^ 1);
        }
        [
            ("mass_rated_kg", |l| bump(&mut l.mass_rated_kg)),
            ("e_max_j", |l| bump(&mut l.e_max_j)),
            ("p_max_kgmps", |l| bump(&mut l.p_max_kgmps)),
            ("v_open_max_mps", |l| bump(&mut l.v_open_max_mps)),
            ("v_close_max_mps", |l| bump(&mut l.v_close_max_mps)),
            ("v_open_sensed_mps", |l| bump(&mut l.v_open_sensed_mps)),
            ("v_close_sensed_mps", |l| bump(&mut l.v_close_sensed_mps)),
            ("f_drive_max_n", |l| bump(&mut l.f_drive_max_n)),
            ("t_drive_max_s", |l| bump(&mut l.t_drive_max_s)),
            ("f_static_n", |l| bump(&mut l.f_static_n)),
            ("f_static_sensed_n", |l| bump(&mut l.f_static_sensed_n)),
            ("f_static_final_n", |l| bump(&mut l.f_static_final_n)),
            ("t_static_final_s", |l| bump(&mut l.t_static_final_s)),
            ("f_close_last50_n", |l| bump(&mut l.f_close_last50_n)),
            ("last_zone_m", |l| bump(&mut l.last_zone_m)),
            ("i_coil_cont_a", |l| bump(&mut l.i_coil_cont_a)),
            ("i_coil_pulse_a", |l| bump(&mut l.i_coil_pulse_a)),
            ("t_coil_pulse_s", |l| bump(&mut l.t_coil_pulse_s)),
            ("i_lsm_max_a", |l| bump(&mut l.i_lsm_max_a)),
            ("gap_min_m", |l| bump(&mut l.gap_min_m)),
            ("gap_max_m", |l| bump(&mut l.gap_max_m)),
            ("x_min_m", |l| bump(&mut l.x_min_m)),
            ("x_max_m", |l| bump(&mut l.x_max_m)),
            ("overspeed_ratio", |l| bump(&mut l.overspeed_ratio)),
            ("overspeed_t_s", |l| bump(&mut l.overspeed_t_s)),
            ("vel_window_s", |l| bump(&mut l.vel_window_s)),
            ("v_runaway_mps", |l| bump(&mut l.v_runaway_mps)),
            ("v_hold_min_v", |l| bump(&mut l.v_hold_min_v)),
            ("k_f_copy_n_per_a", |l| bump(&mut l.k_f_copy_n_per_a)),
            ("adc_a_per_code_copy", |l| bump(&mut l.adc_a_per_code_copy)),
            ("lsm_a_per_code_copy", |l| bump(&mut l.lsm_a_per_code_copy)),
            ("nm_per_count_copy", |l| l.nm_per_count_copy ^= 1),
            ("presence_copy", |l| {
                l.presence_copy = match l.presence_copy {
                    fabrail_types::PresenceSensing::None => fabrail_types::PresenceSensing::PlD,
                    fabrail_types::PresenceSensing::PlD => fabrail_types::PresenceSensing::None,
                }
            }),
            ("watchdog_ticks", |l| l.watchdog_ticks ^= 1),
            ("heartbeat_max_ticks", |l| l.heartbeat_max_ticks ^= 1),
            ("isolate.stroke_m", |l| bump(&mut l.isolate.stroke_m)),
            ("isolate.bumper_ke_j", |l| bump(&mut l.isolate.bumper_ke_j)),
            ("isolate.f_cont_n", |l| bump(&mut l.isolate.f_cont_n)),
            ("unattended.f_pinch_n", |l| {
                bump(&mut l.unattended.f_pinch_n)
            }),
            ("unattended.k_series_n_per_m", |l| {
                bump(&mut l.unattended.k_series_n_per_m)
            }),
            ("unattended.detect_max_s", |l| {
                bump(&mut l.unattended.detect_max_s)
            }),
            ("unattended.max_attempts", |l| {
                l.unattended.max_attempts ^= 1
            }),
        ]
    }

    // req: FAB-SAF-015
    #[test]
    fn crc_detects_corruption() {
        for c in presets::CLASSES {
            let p = presets::by_class(c);
            assert_eq!(verify_seal(&p), Ok(()), "{c:?}: preset is sealed");
            // A one-bit flip in any field is detected (CRC-32 detects all single-bit errors).
            for (name, flip) in mutators() {
                let mut q = p;
                flip(&mut q.safety);
                assert!(
                    verify_seal(&q).is_err(),
                    "{c:?}: corruption of safety.{name} not detected (SAF-015)"
                );
                assert_ne!(
                    safety_crc(&q.safety),
                    p.safety.crc,
                    "{c:?}: {name} is covered by the CRC"
                );
            }
            // A corrupted stored CRC is detected.
            let mut q = p;
            q.safety.crc ^= 0x8000_0000;
            assert_eq!(verify_seal(&q).map_err(|e| e.path), Err("safety.crc"));
            // A diverse copy that disagrees with the design value is detected even with a
            // valid CRC over the corrupted block.
            let mut q = p;
            q.design.sensors.coil_a_per_code *= 1.01;
            assert_eq!(
                verify_seal(&q).map_err(|e| e.path),
                Err("safety.adc_a_per_code_copy"),
                "{c:?}: copy mismatch refuses lift (SAF-015)"
            );
            let mut q = p;
            q.design.lsm.k_f_n_per_a *= 1.01;
            assert_eq!(
                verify_seal(&q).map_err(|e| e.path),
                Err("safety.k_f_copy_n_per_a")
            );
            let mut q = p;
            q.design.sensors.lsm_a_per_code *= 1.01;
            assert_eq!(
                verify_seal(&q).map_err(|e| e.path),
                Err("safety.lsm_a_per_code_copy")
            );
            let mut q = p;
            q.design.sensors.nm_per_count += 1;
            assert_eq!(
                verify_seal(&q).map_err(|e| e.path),
                Err("safety.nm_per_count_copy")
            );
            let mut q = p;
            q.design.presence = fabrail_types::PresenceSensing::PlD;
            assert_eq!(
                verify_seal(&q).map_err(|e| e.path),
                Err("safety.presence_copy"),
                "{c:?}: fitting presence sensing needs a reseal (SAF-003)"
            );
            // Resealing repairs it; sealing is idempotent.
            seal(&mut q);
            assert_eq!(verify_seal(&q), Ok(()));
            let before = q;
            seal(&mut q);
            assert_eq!(q, before, "seal is idempotent");
        }
    }

    #[test]
    fn crc_is_order_sensitive() {
        let p = presets::door();
        let mut q = p;
        core::mem::swap(&mut q.safety.v_open_max_mps, &mut q.safety.v_close_max_mps);
        assert_ne!(
            safety_crc(&q.safety),
            safety_crc(&p.safety),
            "field order is encoded"
        );
    }
}
