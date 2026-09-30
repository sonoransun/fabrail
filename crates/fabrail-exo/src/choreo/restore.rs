//! Restore after INSPECT: local confirmation only, one device at a time at unattended limits, log-damped open termini. Owner: E2 (API frozen by W0a).
//!
//! Plan §4b. Restore runs only in NORMAL, after the INSPECT manual reset and a passing
//! self-test, and only on a **local** command (never remote-only). Devices re-open one
//! at a time at unattended limits and end with log-damped open termini at their
//! remembered positions.
//!
//! [`check_restore`] is the admission rule. [`RestorePlanner`] remembers each device's
//! position before it was secured and, once a request passes the rule, hands out one
//! [`RestoreStep`] at a time in registry order, the next only after the previous device
//! has arrived. A step's speed cap is the device's unattended limit
//! min(v_cap, 150 N/√(m_gov·k_series)); the target is the remembered position, reached
//! through the log-damped open terminus (`MotionMode::SoftOpen`, PHYSICS §16.1). A
//! remembered position at or below the closed stop needs no restore and is skipped.

use fabrail_types::{PosNm, SeismicState};

use crate::choreo::registry::{DeviceRegistry, MAX_DEVICES};

/// Where a command came from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "snake_case"))]
#[repr(u8)]
pub enum CommandSource {
    /// At the device / in the house.
    #[default]
    Local = 0,
    /// Over the network.
    Remote = 1,
}

/// A restore request.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct RestoreRequest {
    /// Command source.
    pub source: CommandSource,
    /// Seismic state.
    pub seis: SeismicState,
    /// The INSPECT manual reset was performed.
    pub inspect_reset_done: bool,
    /// The self-test passed after the reset.
    pub selftest_ok: bool,
}

/// Why a restore was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum RestoreReject {
    /// Not in NORMAL.
    NotNormal = 0,
    /// The INSPECT reset has not been performed.
    InspectNotReset = 1,
    /// The self-test has not passed.
    SelftestFailed = 2,
    /// Remote-only commands never restore.
    RemoteOnly = 3,
}

impl RestoreReject {
    /// Snake-case name (scene events).
    pub const fn slug(self) -> &'static str {
        match self {
            RestoreReject::NotNormal => "not_normal",
            RestoreReject::InspectNotReset => "inspect_not_reset",
            RestoreReject::SelftestFailed => "selftest_failed",
            RestoreReject::RemoteOnly => "remote_only",
        }
    }
}

/// The restore admission rule (plan §4b).
pub const fn check_restore(r: &RestoreRequest) -> Result<(), RestoreReject> {
    if !r.inspect_reset_done {
        return Err(RestoreReject::InspectNotReset);
    }
    if !matches!(r.seis, SeismicState::Normal) {
        return Err(RestoreReject::NotNormal);
    }
    if !r.selftest_ok {
        return Err(RestoreReject::SelftestFailed);
    }
    if matches!(r.source, CommandSource::Remote) {
        return Err(RestoreReject::RemoteOnly);
    }
    Ok(())
}

/// One restore step: re-open one device.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RestoreStep {
    /// Registry index.
    pub device: u8,
    /// Remembered open position (log-damped open terminus target).
    pub target: PosNm,
    /// Speed cap (unattended limit), m/s.
    pub v_cap_mps: f32,
}

/// Sequences restores one device at a time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RestorePlanner {
    remembered: [Option<PosNm>; MAX_DEVICES],
    next: u8,
}

impl Default for RestorePlanner {
    fn default() -> RestorePlanner {
        RestorePlanner::new()
    }
}

impl RestorePlanner {
    /// A planner with no remembered positions.
    pub const fn new() -> RestorePlanner {
        RestorePlanner {
            remembered: [None; MAX_DEVICES],
            next: 0,
        }
    }

    /// Remembers device `i`'s position before it was secured.
    pub fn remember(&mut self, i: usize, x: PosNm) {
        if let Some(s) = self.remembered.get_mut(i) {
            *s = Some(x);
        }
    }

    /// The remembered position of device `i` not yet restored.
    pub fn remembered(&self, i: usize) -> Option<PosNm> {
        self.remembered.get(i).copied().flatten()
    }

    /// Number of devices still waiting for a restore step.
    pub fn pending(&self) -> usize {
        self.remembered.iter().filter(|r| r.is_some()).count()
    }

    /// The next restore step once `req` passes [`check_restore`] and the previous
    /// device has arrived: the next device (registry order) with a remembered position
    /// above its closed stop, capped at its unattended limit; its memory is consumed.
    /// `None` while the request is refused, the previous device is still moving, or
    /// nothing is left.
    pub fn next_step(
        &mut self,
        req: &RestoreRequest,
        reg: &DeviceRegistry,
        previous_arrived: bool,
    ) -> Option<RestoreStep> {
        check_restore(req).ok()?;
        if !previous_arrived {
            return None;
        }
        let n = reg.len().min(MAX_DEVICES);
        while (self.next as usize) < n {
            let i = self.next as usize;
            self.next = self.next.saturating_add(1);
            let Some(x) = self.remembered.get_mut(i).and_then(Option::take) else {
                continue;
            };
            let Some(e) = reg.get(i) else {
                continue;
            };
            if x.diff_nm(e.x_closed) <= 0 {
                continue;
            }
            let c = &e.constraints;
            let v = if c.v_cap_mps > 0.0 {
                c.v_unattended_mps.min(c.v_cap_mps)
            } else {
                c.v_unattended_mps
            };
            if !(v.is_finite() && v > 0.0) {
                continue;
            }
            return Some(RestoreStep {
                device: i as u8,
                target: x,
                v_cap_mps: v,
            });
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::choreo::registry::{DeviceConstraints, DeviceEntry};
    use fabrail_exo_safety::DeviceCondition;

    fn reg3() -> DeviceRegistry {
        let mut r = DeviceRegistry::new();
        for v_u in [0.2305f32, 0.18, 0.12] {
            let e = DeviceEntry {
                condition: DeviceCondition::Seated,
                constraints: DeviceConstraints {
                    v_cap_mps: 0.2654,
                    v_unattended_mps: v_u,
                    ..DeviceConstraints::default()
                },
                ..DeviceEntry::EMPTY
            };
            assert!(r.push(e).is_ok());
        }
        r
    }

    // req: FAB-EXA-015
    #[test]
    fn rejects_before_inspect_and_remote_only() {
        let ok = RestoreRequest {
            source: CommandSource::Local,
            seis: SeismicState::Normal,
            inspect_reset_done: true,
            selftest_ok: true,
        };
        assert_eq!(check_restore(&ok), Ok(()));
        // Before the INSPECT manual reset (in INSPECT, or even back in NORMAL): refused.
        for seis in [
            SeismicState::Inspect,
            SeismicState::Normal,
            SeismicState::Quiet,
        ] {
            let r = RestoreRequest {
                inspect_reset_done: false,
                seis,
                ..ok
            };
            assert_eq!(
                check_restore(&r),
                Err(RestoreReject::InspectNotReset),
                "{seis:?}"
            );
        }
        // Remote-only commands never restore, whatever else holds.
        let remote = RestoreRequest {
            source: CommandSource::Remote,
            ..ok
        };
        assert_eq!(check_restore(&remote), Err(RestoreReject::RemoteOnly));
        // Only in NORMAL, and only after a passing self-test.
        for seis in [
            SeismicState::Triggered,
            SeismicState::Protect,
            SeismicState::Shaking,
            SeismicState::Quiet,
            SeismicState::Inspect,
        ] {
            assert_eq!(
                check_restore(&RestoreRequest { seis, ..ok }),
                Err(RestoreReject::NotNormal)
            );
        }
        assert_eq!(
            check_restore(&RestoreRequest {
                selftest_ok: false,
                ..ok
            }),
            Err(RestoreReject::SelftestFailed)
        );

        // The planner yields nothing for a refused request and keeps its memory.
        let reg = reg3();
        let mut p = RestorePlanner::new();
        p.remember(0, PosNm::from_m_f64(0.30));
        p.remember(2, PosNm::from_m_f64(0.12));
        p.remember(1, PosNm(0)); // closed before: nothing to restore
        for bad in [
            RestoreRequest {
                inspect_reset_done: false,
                ..ok
            },
            remote,
        ] {
            assert_eq!(p.next_step(&bad, &reg, true), None);
        }
        assert_eq!(p.pending(), 3);
        // Valid local request: one device at a time, registry order, remembered
        // targets, unattended speed caps; the next only after the previous arrived.
        let s0 = p.next_step(&ok, &reg, true).expect("first device");
        assert_eq!(s0.device, 0);
        assert_eq!(s0.target, PosNm::from_m_f64(0.30));
        assert!(
            (s0.v_cap_mps - 0.2305).abs() < 1e-7,
            "unattended limit (plan §4b)"
        );
        assert_eq!(p.next_step(&ok, &reg, false), None, "previous still moving");
        let s1 = p.next_step(&ok, &reg, true).expect("second device");
        assert_eq!((s1.device, s1.target), (2, PosNm::from_m_f64(0.12)));
        assert!((s1.v_cap_mps - 0.12).abs() < 1e-7);
        assert_eq!(p.next_step(&ok, &reg, true), None, "all restored");
        assert_eq!(p.pending(), 0);
    }
}
