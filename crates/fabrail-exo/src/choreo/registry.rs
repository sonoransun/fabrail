//! DeviceRegistry and Device: class, egress role, position, state, constraints, contents sensitivity, in fixed registry order. Owner: E2 (API frozen by W0a).
//!
//! Plan §4b. Devices are planned and stepped in fixed registry order (index 0 first);
//! the registry is fixed-capacity ([`MAX_DEVICES`], matching the scene schema's 64) and
//! allocation-free. Human-readable ids live in the std scene (`fabrail_exo_sim::scene`);
//! here a device is its registry index.

use fabrail_exo_safety::{AnyClass, DeviceCondition};
use fabrail_types::{EgressRole, PosNm, RailClass, TravelDir};

use crate::choreo::policy::ChoreoPolicy;

/// Maximum devices in one building (scene schema `devices.maxItems`).
pub const MAX_DEVICES: usize = 64;

/// How sensitive a device's contents are to sliding and rocking.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[repr(u8)]
pub enum ContentsSensitivity {
    /// Empty or insensitive.
    #[default]
    None = 0,
    /// Lined or high-friction contents.
    Low = 1,
    /// Glassware, porcelain.
    Fragile = 2,
    /// Stemware (rocking-prone, b/h ≈ 0.3–0.4).
    Stemware = 3,
}

/// Per-device motion and energy constraints the planner respects.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DeviceConstraints {
    /// Class speed cap, m/s.
    pub v_cap_mps: f32,
    /// Unattended automatic-motion limit min(v_cap, 150 N/√(m_gov·k_series)), m/s.
    pub v_unattended_mps: f32,
    /// Braking deceleration used for exits (2·a_max), m/s².
    pub a_brk_mps2: f32,
    /// Bus power while moving, W.
    pub p_move_w: f32,
    /// Smooth-ride-through standoff s_min = 25 mm + S_max,pred, m.
    pub s_min_m: f32,
    /// A seismic latch is fitted (`design.landing.seismic_latch`).
    pub latch_fitted: bool,
    /// Latch x-retention force, N.
    pub latch_retention_n: f32,
}

/// One device as the choreographer sees it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeviceEntry {
    /// Class.
    pub class: AnyClass,
    /// Egress role (egress devices never lock).
    pub egress: EgressRole,
    /// Floor level.
    pub level: u8,
    /// Heading of the rail's +x axis in the building frame, rad.
    pub heading_rad: f32,
    /// Plan position (display), m.
    pub position_m: [f32; 2],
    /// Physical condition.
    pub condition: DeviceCondition,
    /// Measured position.
    pub x: PosNm,
    /// Closed / seat position.
    pub x_closed: PosNm,
    /// Measured velocity, m/s.
    pub v_mps: f32,
    /// Governor mass max(m_rated, 1.05·m̂), kg.
    pub m_gov_kg: f32,
    /// Direction of a user-initiated motion in progress (never reversed).
    pub user_motion: Option<TravelDir>,
    /// Obstruction detection armed.
    pub obstruction_armed: bool,
    /// Hero device (Carriage6 fidelity; SIL ≡ firmware claimed only for these).
    pub hero: bool,
    /// Configured policy (`Auto`: by class, egress and admission).
    pub policy: ChoreoPolicy,
    /// Contents sensitivity.
    pub contents: ContentsSensitivity,
    /// Constraints.
    pub constraints: DeviceConstraints,
}

impl DeviceEntry {
    /// An empty DRAWER entry (all limits zero, so nothing moves).
    pub const EMPTY: DeviceEntry = DeviceEntry {
        class: AnyClass::Core(RailClass::Drawer),
        egress: EgressRole::None,
        level: 0,
        heading_rad: 0.0,
        position_m: [0.0; 2],
        condition: DeviceCondition::Moving,
        x: PosNm(0),
        x_closed: PosNm(0),
        v_mps: 0.0,
        m_gov_kg: 0.0,
        user_motion: None,
        obstruction_armed: false,
        hero: false,
        policy: ChoreoPolicy::Auto,
        contents: ContentsSensitivity::None,
        constraints: DeviceConstraints {
            v_cap_mps: 0.0,
            v_unattended_mps: 0.0,
            a_brk_mps2: 0.0,
            p_move_w: 0.0,
            s_min_m: 0.0,
            latch_fitted: false,
            latch_retention_n: 0.0,
        },
    };
}

impl Default for DeviceEntry {
    fn default() -> DeviceEntry {
        DeviceEntry::EMPTY
    }
}

/// The registry is full.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RegistryFull;

/// Every device of a building, in fixed order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeviceRegistry {
    entries: [DeviceEntry; MAX_DEVICES],
    len: u8,
}

impl Default for DeviceRegistry {
    fn default() -> DeviceRegistry {
        DeviceRegistry::new()
    }
}

impl DeviceRegistry {
    /// An empty registry.
    pub const fn new() -> DeviceRegistry {
        DeviceRegistry {
            entries: [DeviceEntry::EMPTY; MAX_DEVICES],
            len: 0,
        }
    }

    /// Appends `e`; returns its index.
    pub fn push(&mut self, e: DeviceEntry) -> Result<u8, RegistryFull> {
        let i = self.len;
        match self.entries.get_mut(i as usize) {
            Some(slot) => {
                *slot = e;
                self.len += 1;
                Ok(i)
            }
            None => Err(RegistryFull),
        }
    }

    /// Number of devices.
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// True when empty.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Device `i`.
    pub fn get(&self, i: usize) -> Option<&DeviceEntry> {
        self.as_slice().get(i)
    }

    /// Device `i`, mutable.
    pub fn get_mut(&mut self, i: usize) -> Option<&mut DeviceEntry> {
        let n = self.len as usize;
        self.entries.get_mut(..n).and_then(|s| s.get_mut(i))
    }

    /// The devices in registry order.
    pub fn as_slice(&self) -> &[DeviceEntry] {
        self.entries.get(..self.len as usize).unwrap_or(&[])
    }

    /// Iterates in registry order.
    pub fn iter(&self) -> impl Iterator<Item = &DeviceEntry> {
        self.as_slice().iter()
    }
}
