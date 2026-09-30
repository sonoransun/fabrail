//! fabrail-types: the shared vocabulary of every fabrail layer (firmware, monitor, SIL,
//! Python KPIs). No control logic lives here, so it froze cleanly in Wave 0.
//!
//! * [`units`]: `Tick` (20 kHz base ticks), `PosNm` (absolute i64 nanometres) and
//!   `PosAccum` (integer base + f32 fraction), PHYSICS §24.
//! * [`fmath`]: THE only math entry point (soft-float `libm`), identical bits on host,
//!   thumbv7em and wasm32.
//! * [`raw`]: HAL raw samples (integer ADC codes and counts) and driver status.
//! * [`cmd`]: actuator and user commands.
//! * [`state`]: classes, rest/egress policy, state machines, demands, the monitor verdict.
//! * [`segment`]: motion reference segments (S-curve, log-damped terminus, quintic).
//! * [`fault`]: the `FaultCode` registry and the 128-bit `FaultSet`.
//! * [`alert`]: network earthquake alerts.
//! * [`telemetry`]: the fixed-size `repr(C)` `TelemetryFrame` and `Event`.
//!
//! Frames and signs (PHYSICS §1, SW-005): x is travel along the rail (+x opens; this is
//! the README's "lateral propulsion"), y is cross-track ("lateral" in code), z is up.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

/// Defines a `#[repr(u8)]` plain-data enum with explicit discriminants plus
/// `ALL`, `as_u8`, `from_u8` (decoding telemetry and replay bytes) and `name`.
macro_rules! u8_enum {
    (
        $(#[$meta:meta])*
        pub enum $name:ident {
            $( $(#[$vmeta:meta])* $var:ident = $val:expr ),+ $(,)?
        }
    ) => {
        $(#[$meta])*
        #[repr(u8)]
        pub enum $name {
            $( $(#[$vmeta])* $var = $val ),+
        }

        impl $name {
            /// Every variant, in discriminant order.
            pub const ALL: &'static [$name] = &[$($name::$var),+];

            /// The wire value (telemetry, replay, PyO3).
            pub const fn as_u8(self) -> u8 {
                self as u8
            }

            /// Decodes a wire value; `None` for an unknown discriminant.
            pub const fn from_u8(v: u8) -> Option<Self> {
                match v {
                    $( x if x == $val => Some($name::$var), )+
                    _ => None,
                }
            }

            /// The variant name, e.g. for event logs and Python enums.
            pub const fn name(self) -> &'static str {
                match self {
                    $( $name::$var => stringify!($var), )+
                }
            }
        }
    };
}

pub mod alert;
pub mod cmd;
pub mod fault;
pub mod fmath;
pub mod raw;
pub mod segment;
pub mod state;
pub mod telemetry;
pub mod units;

pub use alert::*;
pub use cmd::*;
pub use fault::*;
pub use raw::*;
pub use segment::*;
pub use state::*;
pub use telemetry::*;
pub use units::*;

/// Maximum number of levitation elements (magnets) per carriage: HEAVY uses 8
/// (2 bogies × 4), the other classes 4 or 6 (core design §C.1).
pub const MAX_MAGNETS: usize = 8;

/// Base tick rate: the PWM / current-loop / plant-step rate, h = 50 µs (core design §A.5).
pub const BASE_TICK_HZ: u32 = 20_000;

/// Base tick period in seconds, `1 / BASE_TICK_HZ` (the nearest f64 to 5e-5).
pub const BASE_TICK_S: f64 = 5.0e-5;

/// Number of accelerometer samples a seismic job drains per release: the ADXL355 FIFO
/// at 500 Hz ODR delivers 5 samples per 100 Hz period; 16 leaves margin for jitter.
pub const ACCEL_DRAIN_MAX: usize = 16;
