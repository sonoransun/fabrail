//! fabrail-hal: the portability contract (core design §B.3). Traits only, no logic.
//!
//! Two implementations exist: `fabrail_sim::simboard::SimBoard` (sensor models over f64
//! plant truth) and the firmware BSP (`firmware/src/hal_impl.rs`, STM32G474). The
//! `fabrail-rt` executive is generic over these traits, so SIL and firmware run the same
//! group code and produce the same bits (SIL ≡ firmware, core design §A.4 item 7).
//!
//! Group access (core design §A.5):
//! * base (20 kHz): [`BaseHal`] = coil drivers, LSM driver, gap sensors, encoder, carriage
//!   IMU, rail floor accelerometer. The base task latches `FastRaw` for the fast group,
//!   which is a pure function and touches no HAL at all.
//! * supervisor (1 kHz): [`SupervisorHal`] = power, safety inputs, aux, command link,
//!   watchdog, telemetry.
//! * seismic (100 Hz): [`SeismicHal`] = structure accelerometer FIFO, alert link.
//!
//! Several traits name their method `sample`; callers disambiguate with
//! `GapSensors::sample(board)` syntax.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use fabrail_types::{
    ACCEL_DRAIN_MAX, AccelRaw, AuxCmd, CoilCmd, CurrentRaw, DriverStatus, EncoderRaw, Event,
    GapRaw, ImuRaw, LsmCmd, LsmRaw, NetworkAlert, PowerRaw, RailAccelRaw, ResetCause, SafetyInRaw,
    TelemetryFrame, Tick, UserCommand,
};

/// Monotonic base-tick clock.
pub trait Clock {
    /// Current base tick.
    fn now(&self) -> Tick;
}

/// Levitation coil H-bridges and their current sense (base group).
pub trait CoilDrivers {
    /// Latest coil current codes.
    fn sample(&mut self) -> CurrentRaw;
    /// Applies a coil command (takes effect at the next PWM update, k + 1).
    fn command(&mut self, c: &CoilCmd);
    /// Bridge status.
    fn status(&self) -> DriverStatus;
}

/// Gap sensors (base group).
pub trait GapSensors {
    /// Latest gap codes (primary and redundant).
    fn sample(&mut self) -> GapRaw;
}

/// Linear encoder (base group).
pub trait Encoder {
    /// Latest encoder counts.
    fn sample(&mut self) -> EncoderRaw;
}

/// LSM inverter (base group).
pub trait LsmDriver {
    /// Latest phase current codes.
    fn sample(&mut self) -> LsmRaw;
    /// Applies an LSM command (STO/ASC take effect immediately in hardware).
    fn command(&mut self, c: &LsmCmd);
    /// Inverter status.
    fn status(&self) -> DriverStatus;
}

/// Carriage IMU: skyhook absolute-velocity feedback and fusion only, never EEW (F16).
pub trait CarriageImu {
    /// Latest IMU codes (`present = false` when not fitted).
    fn sample(&mut self) -> ImuRaw;
}

/// Rail-mounted floor accelerometer sampled in the base group: the only floor
/// feed-forward source for inertial smoothing (plan §4b; PHYSICS §16.2).
pub trait RailAccel {
    /// Latest rail accelerometer codes (`present = false` when not fitted).
    fn sample(&mut self) -> RailAccelRaw;
}

/// Structure-mounted ADXL355 FIFO (seismic group, 500 Hz ODR, drained at 100 Hz).
pub trait Accelerometer {
    /// Moves up to `out.len()` queued samples into `out`, oldest first; returns the
    /// number written.
    fn drain(&mut self, out: &mut [AccelRaw; ACCEL_DRAIN_MAX]) -> usize;
}

/// Power path sense (supervisor group).
pub trait PowerMonitor {
    /// Latest bus / hold-up / temperature codes.
    fn sample(&mut self) -> PowerRaw;
}

/// Safety-related discrete and analogue inputs (supervisor group).
pub trait SafetyInputs {
    /// Latest safety input sample.
    fn sample(&mut self) -> SafetyInRaw;
}

/// Auxiliary actuators (supervisor group).
pub trait Aux {
    /// Applies the auxiliary command (parking lock, seismic latch).
    fn command(&mut self, c: &AuxCmd);
}

/// User / building command link (CAN; SimBoard events).
pub trait CommandLink {
    /// Next pending command, if any.
    fn poll(&mut self) -> Option<UserCommand>;
}

/// Earthquake alert link (CAN from a gateway; simulated feeds only).
pub trait AlertLink {
    /// Next pending alert or cancellation, if any.
    fn poll(&mut self) -> Option<NetworkAlert>;
}

/// Independent watchdog (IWDG ≈ 5 ms on firmware).
pub trait Watchdog {
    /// Feeds the watchdog. The executive feeds only if every group's sequence number
    /// advanced and no overrun occurred.
    fn feed(&mut self);
    /// Cause of the last reset.
    fn reset_cause(&self) -> ResetCause;
}

/// Parameter persistence (firmware: flash A/B pages with postcard + CRC32).
pub trait ParamStore {
    /// Storage error type.
    type Error;
    /// Reads the active parameter image into `buf`; returns the number of bytes read.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;
    /// Writes a new parameter image to the inactive page and activates it.
    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;
}

/// Telemetry sink (defmt/RTT or CAN on firmware; the recorder in SIL).
pub trait Telemetry {
    /// Publishes one supervisor-rate frame.
    fn frame(&mut self, f: &TelemetryFrame);
    /// Publishes one discrete event.
    fn event(&mut self, e: &Event);
}

/// Everything the 20 kHz base task touches.
pub trait BaseHal:
    CoilDrivers + LsmDriver + GapSensors + Encoder + CarriageImu + RailAccel
{
}

impl<T> BaseHal for T where
    T: CoilDrivers + LsmDriver + GapSensors + Encoder + CarriageImu + RailAccel
{
}

/// Everything the 1 kHz supervisor group touches.
pub trait SupervisorHal:
    PowerMonitor + SafetyInputs + Aux + CommandLink + Watchdog + Telemetry
{
}

impl<T> SupervisorHal for T where
    T: PowerMonitor + SafetyInputs + Aux + CommandLink + Watchdog + Telemetry
{
}

/// Everything the 100 Hz seismic group touches.
pub trait SeismicHal: Accelerometer + AlertLink {}

impl<T> SeismicHal for T where T: Accelerometer + AlertLink {}

/// A complete board: SimBoard in SIL, the BSP on firmware.
pub trait Board: Clock + BaseHal + SupervisorHal + SeismicHal {}

impl<T> Board for T where T: Clock + BaseHal + SupervisorHal + SeismicHal {}

#[cfg(test)]
mod tests {
    use super::*;
    use fabrail_types::{LsmMode, MAX_MAGNETS};

    /// Minimal board: proves the blanket impls compose and that same-named `sample`
    /// methods are reachable through fully qualified syntax.
    #[derive(Default)]
    struct NullBoard {
        tick: u64,
        coil: Option<CoilCmd>,
        lsm: Option<LsmCmd>,
        fed: u32,
        frames: u32,
        queued_accel: usize,
    }

    impl Clock for NullBoard {
        fn now(&self) -> Tick {
            Tick(self.tick)
        }
    }
    impl CoilDrivers for NullBoard {
        fn sample(&mut self) -> CurrentRaw {
            CurrentRaw {
                code: [7; MAX_MAGNETS],
            }
        }
        fn command(&mut self, c: &CoilCmd) {
            self.coil = Some(*c);
        }
        fn status(&self) -> DriverStatus {
            DriverStatus {
                enabled: true,
                ..DriverStatus::default()
            }
        }
    }
    impl GapSensors for NullBoard {
        fn sample(&mut self) -> GapRaw {
            GapRaw {
                valid: 0x0F,
                ..GapRaw::default()
            }
        }
    }
    impl Encoder for NullBoard {
        fn sample(&mut self) -> EncoderRaw {
            EncoderRaw {
                counts: -3,
                referenced: true,
                err: false,
            }
        }
    }
    impl LsmDriver for NullBoard {
        fn sample(&mut self) -> LsmRaw {
            LsmRaw::default()
        }
        fn command(&mut self, c: &LsmCmd) {
            self.lsm = Some(*c);
        }
        fn status(&self) -> DriverStatus {
            DriverStatus::default()
        }
    }
    impl CarriageImu for NullBoard {
        fn sample(&mut self) -> ImuRaw {
            ImuRaw::default()
        }
    }
    impl RailAccel for NullBoard {
        fn sample(&mut self) -> RailAccelRaw {
            RailAccelRaw {
                xyz: [1, 2, 3],
                present: true,
            }
        }
    }
    impl Accelerometer for NullBoard {
        fn drain(&mut self, out: &mut [AccelRaw; ACCEL_DRAIN_MAX]) -> usize {
            let n = self.queued_accel.min(out.len());
            for (i, s) in out.iter_mut().take(n).enumerate() {
                *s = AccelRaw {
                    seq: i as u32,
                    ..AccelRaw::default()
                };
            }
            self.queued_accel -= n;
            n
        }
    }
    impl PowerMonitor for NullBoard {
        fn sample(&mut self) -> PowerRaw {
            PowerRaw {
                mains_ok: true,
                ..PowerRaw::default()
            }
        }
    }
    impl SafetyInputs for NullBoard {
        fn sample(&mut self) -> SafetyInRaw {
            SafetyInRaw::default()
        }
    }
    impl Aux for NullBoard {
        fn command(&mut self, _c: &AuxCmd) {}
    }
    impl CommandLink for NullBoard {
        fn poll(&mut self) -> Option<UserCommand> {
            Some(UserCommand::Open)
        }
    }
    impl AlertLink for NullBoard {
        fn poll(&mut self) -> Option<NetworkAlert> {
            None
        }
    }
    impl Watchdog for NullBoard {
        fn feed(&mut self) {
            self.fed += 1;
        }
        fn reset_cause(&self) -> ResetCause {
            ResetCause::PowerOn
        }
    }
    impl Telemetry for NullBoard {
        fn frame(&mut self, _f: &TelemetryFrame) {
            self.frames += 1;
        }
        fn event(&mut self, _e: &Event) {}
    }

    fn base_group<H: BaseHal>(h: &mut H) -> (CurrentRaw, GapRaw, RailAccelRaw) {
        let cur = CoilDrivers::sample(h);
        let gap = GapSensors::sample(h);
        let rail = RailAccel::sample(h);
        CoilDrivers::command(h, &CoilCmd::CurrentRef([0.5; MAX_MAGNETS]));
        LsmDriver::command(
            h,
            &LsmCmd {
                iq_ref_a: 1.0,
                id_ref_a: 0.0,
                mode: LsmMode::Run,
            },
        );
        (cur, gap, rail)
    }

    fn supervisor_group<H: SupervisorHal>(h: &mut H) -> Option<UserCommand> {
        let _p = PowerMonitor::sample(h);
        let _s = SafetyInputs::sample(h);
        h.feed();
        h.frame(&TelemetryFrame::default());
        CommandLink::poll(h)
    }

    fn seismic_group<H: SeismicHal>(h: &mut H) -> usize {
        let mut buf = [AccelRaw::default(); ACCEL_DRAIN_MAX];
        let n = h.drain(&mut buf);
        let _alert = AlertLink::poll(h);
        n
    }

    fn whole_board<B: Board>(b: &mut B) -> Tick {
        b.now()
    }

    #[test]
    fn blanket_impls_compose() {
        let mut b = NullBoard {
            tick: 99,
            queued_accel: 20,
            ..NullBoard::default()
        };
        let (cur, gap, rail) = base_group(&mut b);
        assert_eq!(cur.code[0], 7);
        assert_eq!(gap.valid, 0x0F);
        assert!(rail.present);
        assert!(matches!(b.coil, Some(CoilCmd::CurrentRef(_))));
        assert_eq!(b.lsm.map(|c| c.mode), Some(LsmMode::Run));
        assert_eq!(supervisor_group(&mut b), Some(UserCommand::Open));
        assert_eq!((b.fed, b.frames), (1, 1));
        assert_eq!(
            seismic_group(&mut b),
            ACCEL_DRAIN_MAX,
            "drain is capped at the buffer"
        );
        assert_eq!(seismic_group(&mut b), 4);
        assert_eq!(whole_board(&mut b), Tick(99));
        assert!(CoilDrivers::status(&b).enabled);
    }
}
