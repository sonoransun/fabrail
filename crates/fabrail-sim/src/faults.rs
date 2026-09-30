//! Fault injection into sensors and (via rt fault-injection) core outputs. Owner: A7.
//!
//! Sensor/driver faults (`fault` events) are applied by SimBoard on the raw samples
//! and the actuator path; core-output faults (`core_fault` events) go through
//! `fabrail_rt`'s `fault-injection` feature, which perturbs core outputs BEFORE the
//! monitor sees them (the golden selftest hash is identical with and without the
//! feature). Random fault choices draw from the `FaultInj` substream.
//!
//! Fault semantics (scenario `fault.what`, PHYSICS §13):
//!
//! | Fault | Effect |
//! |---|---|
//! | `coil_driver` ch | bridge `ch` latches off: zero drive on that element, driver status `fault` |
//! | `lsm_driver` | inverter latches off (STO), `LsmRaw.fault` and status `fault` |
//! | `gap_stuck` ch | primary probe code freezes at its value when the fault becomes active |
//! | `gap_lost` ch | channel `ch` reports invalid conversions (valid bit cleared) |
//! | `encoder_jump` | counts jump by `jump_m` (default 1 mm) and stay offset |
//! | `encoder_lost` | signal error flag; counts freeze |
//! | `accel_stuck` ch | ADXL355 axis `ch` freezes |
//! | `accel_lost` | the FIFO stops delivering |
//! | `imu_lost` / `rail_accel_lost` | sensor reports `present = false` |
//!
//! `control_loss` events (coil drivers stuck at full output) are held here too because
//! they act on the same actuator path.

use crate::rng::{StreamTag, Xoshiro256pp, substream};
use crate::scenario::{ControlLossDir, CoreFaultKind, SensorFaultKind};
use fabrail_types::MAX_MAGNETS;

/// Active fault state of one device.
#[derive(Clone, Debug)]
pub struct FaultInjector {
    rng: Xoshiro256pp,
    sensor: Vec<(SensorFaultKind, u8)>,
    core: Option<CoreFaultKind>,
    /// Coil bridges latched off (bit per element).
    pub coil_driver_mask: u8,
    /// LSM inverter latched off.
    pub lsm_driver: bool,
    /// Primary gap probes frozen (bit per element).
    pub gap_stuck_mask: u8,
    /// Frozen codes (valid once the bit in `gap_stuck_primed` is set).
    pub gap_stuck_code: [u16; MAX_MAGNETS],
    /// Frozen codes captured.
    pub gap_stuck_primed: u8,
    /// Gap channels reporting invalid conversions (bit per element).
    pub gap_lost_mask: u8,
    /// Accumulated encoder jump, counts.
    pub encoder_offset_counts: i64,
    /// Encoder signal error.
    pub encoder_lost: bool,
    /// Counts frozen at the moment of the encoder error.
    pub encoder_frozen: Option<i64>,
    /// ADXL355 axes frozen (bit per axis) and their codes.
    pub accel_stuck_mask: u8,
    /// Frozen ADXL355 codes (valid once primed).
    pub accel_stuck_code: [i32; 3],
    /// Frozen ADXL355 codes captured (bit per axis).
    pub accel_stuck_primed: u8,
    /// ADXL355 FIFO stopped.
    pub accel_lost: bool,
    /// Carriage IMU not responding.
    pub imu_lost: bool,
    /// Rail accelerometer not responding.
    pub rail_accel_lost: bool,
    /// Coil drivers stuck at full output.
    pub control_loss: Option<ControlLossDir>,
}

impl FaultInjector {
    /// An injector for device `device` of a run seeded with `noise_seed`.
    pub fn new(noise_seed: u64, device: u16) -> FaultInjector {
        FaultInjector {
            rng: substream(noise_seed, StreamTag::FaultInj, device, 0),
            sensor: Vec::new(),
            core: None,
            coil_driver_mask: 0,
            lsm_driver: false,
            gap_stuck_mask: 0,
            gap_stuck_code: [0; MAX_MAGNETS],
            gap_stuck_primed: 0,
            gap_lost_mask: 0,
            encoder_offset_counts: 0,
            encoder_lost: false,
            encoder_frozen: None,
            accel_stuck_mask: 0,
            accel_stuck_code: [0; 3],
            accel_stuck_primed: 0,
            accel_lost: false,
            imu_lost: false,
            rail_accel_lost: false,
            control_loss: None,
        }
    }

    /// Activates a sensor/driver fault on channel `ch` (`jump_counts` is the encoder
    /// jump for `encoder_jump`, already converted to counts).
    pub fn activate(&mut self, what: SensorFaultKind, ch: u8, jump_counts: i64) {
        let bit = 1u8.checked_shl(ch as u32).unwrap_or(0);
        match what {
            SensorFaultKind::CoilDriver => self.coil_driver_mask |= bit,
            SensorFaultKind::LsmDriver => self.lsm_driver = true,
            SensorFaultKind::GapStuck => self.gap_stuck_mask |= bit,
            SensorFaultKind::GapLost => self.gap_lost_mask |= bit,
            SensorFaultKind::EncoderJump => {
                self.encoder_offset_counts = self.encoder_offset_counts.saturating_add(jump_counts)
            }
            SensorFaultKind::EncoderLost => self.encoder_lost = true,
            SensorFaultKind::AccelStuck => self.accel_stuck_mask |= bit,
            SensorFaultKind::AccelLost => self.accel_lost = true,
            SensorFaultKind::ImuLost => self.imu_lost = true,
            SensorFaultKind::RailAccelLost => self.rail_accel_lost = true,
        }
        self.activate_sensor(what, ch);
    }

    /// Records a sensor/driver fault on channel `ch` in the active list (bookkeeping
    /// only; [`FaultInjector::activate`] also applies it).
    pub fn activate_sensor(&mut self, what: SensorFaultKind, ch: u8) {
        if !self.sensor.contains(&(what, ch)) {
            self.sensor.push((what, ch));
        }
    }

    /// Sets (or clears, with `None`) the core-output fault.
    pub fn set_core(&mut self, what: Option<CoreFaultKind>) {
        self.core = what;
    }

    /// Active sensor faults.
    pub fn sensor_faults(&self) -> &[(SensorFaultKind, u8)] {
        &self.sensor
    }

    /// Active core fault.
    pub fn core_fault(&self) -> Option<CoreFaultKind> {
        self.core
    }

    /// The fault-choice random stream.
    pub fn rng(&mut self) -> &mut Xoshiro256pp {
        &mut self.rng
    }

    /// Applies gap faults to channel `ch`: returns the (possibly frozen) primary code
    /// and whether the channel is valid.
    pub fn gap(&mut self, ch: usize, code: u16) -> (u16, bool) {
        let bit = 1u8.checked_shl(ch as u32).unwrap_or(0);
        let mut out = code;
        if self.gap_stuck_mask & bit != 0 {
            if self.gap_stuck_primed & bit == 0 {
                if let Some(c) = self.gap_stuck_code.get_mut(ch) {
                    *c = code;
                }
                self.gap_stuck_primed |= bit;
            }
            out = self.gap_stuck_code.get(ch).copied().unwrap_or(code);
        }
        (out, self.gap_lost_mask & bit == 0)
    }

    /// Applies encoder faults to raw `counts`: (counts, err).
    pub fn encoder(&mut self, counts: i64) -> (i64, bool) {
        let c = counts.saturating_add(self.encoder_offset_counts);
        if self.encoder_lost {
            let frozen = *self.encoder_frozen.get_or_insert(c);
            return (frozen, true);
        }
        (c, false)
    }

    /// Applies ADXL355 axis faults to one sample.
    pub fn accel(&mut self, xyz: [i32; 3]) -> [i32; 3] {
        let mut out = xyz;
        for (ax, o) in out.iter_mut().enumerate() {
            let bit = 1u8 << ax;
            if self.accel_stuck_mask & bit != 0 {
                if self.accel_stuck_primed & bit == 0 {
                    if let Some(c) = self.accel_stuck_code.get_mut(ax) {
                        *c = *o;
                    }
                    self.accel_stuck_primed |= bit;
                }
                *o = self.accel_stuck_code.get(ax).copied().unwrap_or(*o);
            }
        }
        out
    }

    /// Coil bridge `ch` latched off.
    pub fn coil_off(&self, ch: usize) -> bool {
        1u8.checked_shl(ch as u32)
            .is_some_and(|b| self.coil_driver_mask & b != 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stuck_channels_freeze_at_the_activation_value() {
        let mut f = FaultInjector::new(1, 0);
        assert_eq!(f.gap(2, 100), (100, true));
        f.activate(SensorFaultKind::GapStuck, 2, 0);
        assert_eq!(f.gap(2, 120), (120, true), "captures the first value");
        assert_eq!(f.gap(2, 999), (120, true));
        assert_eq!(f.gap(1, 999), (999, true), "other channels untouched");
        f.activate(SensorFaultKind::GapLost, 1, 0);
        assert_eq!(f.gap(1, 5), (5, false));
        f.activate(SensorFaultKind::AccelStuck, 1, 0);
        assert_eq!(f.accel([1, 2, 3]), [1, 2, 3]);
        assert_eq!(f.accel([4, 5, 6]), [4, 2, 6]);
        assert_eq!(f.sensor_faults().len(), 3);
    }

    #[test]
    fn encoder_jump_offsets_and_lost_freezes() {
        let mut f = FaultInjector::new(1, 0);
        f.activate(SensorFaultKind::EncoderJump, 0, 1000);
        assert_eq!(f.encoder(5), (1005, false));
        f.activate(SensorFaultKind::EncoderLost, 0, 0);
        assert_eq!(f.encoder(7), (1007, true));
        assert_eq!(f.encoder(9), (1007, true), "counts freeze");
        f.activate(SensorFaultKind::CoilDriver, 3, 0);
        assert!(f.coil_off(3) && !f.coil_off(2) && !f.coil_off(200));
    }
}
