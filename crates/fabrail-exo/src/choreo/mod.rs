//! House choreographer (plan §4b): plans every device's seismic action from a
//! FusedAlert, the S-ETA distribution, the device registry and the building config,
//! within deadlines and the bus power budget; alert-initiated motion only toward a
//! higher `SafetyRank`, in the closing/seating direction, at unattended limits. Declared
//! NOT a rated safety function. Owner: E2 (module list frozen by W0a).

pub mod plan;
pub mod policy;
pub mod registry;
pub mod restore;
