//! Network earthquake-early-warning alerts (PHYSICS §20; SEI-007).
//!
//! fabrail is not an EEW provider. Live ShakeAlert/JMA feeds are roadmap and need a
//! licence; every alert in simulation is synthetic or injected. A network-only alert may
//! cause only FREEZE, LAND, LOCK or SEISMIC_PASSIVE; alert-triggered motion needs
//! independent confirmation (plan §4).

use crate::Tick;

/// A network (or fused, re-emitted) earthquake alert as received over the alert link.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NetworkAlert {
    /// Alert identifier (updates reuse the id).
    pub id: u32,
    /// Base tick at which the alert was received.
    pub received: Tick,
    /// Estimated origin time relative to `received`, s (≤ 0: the origin precedes
    /// reception). Signed and relative so an origin before the run start (tick 0) is
    /// not lost; S-ETA = received + origin_offset_s + R_hyp/V_s (PHYSICS §20).
    pub origin_offset_s: f32,
    /// Moment magnitude estimate.
    pub magnitude: f32,
    /// Epicentral distance to this site, km.
    pub r_epi_km: f32,
    /// Hypocentral depth, km.
    pub depth_km: f32,
    /// Predicted intensity at this site (MMI).
    pub mmi: f32,
    /// This message cancels alert `id`.
    pub cancel: bool,
}

u8_enum! {
    /// Where the seismic decision came from.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
    #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
    #[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
    pub enum AlertSource {
        /// On-site P-wave detection (structure ADXL355).
        #[default]
        Local = 0,
        /// Network alert only (may cause only FREEZE/LAND/LOCK/SEISMIC_PASSIVE).
        Network = 1,
        /// Local detection confirmed by the network, or ≥ 2 independent feeds.
        Fused = 2,
    }
}
