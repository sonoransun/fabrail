//! DRAWER captive seat: unseat current ramp to sensed lift-off (m_hat closed form) and the z-seat step (PHYSICS §15; F17). Owner: A1.
//!
//! Frozen API (W0): [`Seat`] with `start_unseat`, `start_seat` and `step`. Unseat ramps
//! current at `unseat_ramp_a_per_s` until lift-off is sensed (> `unseat_liftoff_m`,
//! 20 µm), giving m̂ from the closed form (−5.0 A at 8 kg to +11.2 A at 48 kg, within
//! i_pulse 12 A), then the LSM drives out of the slot (`landing.slot_len_m`) within the
//! I²t pulse budget (48 kg within t_pulse = 200 ms; plan §2b item 4). The z-seat step
//! takes `seat_time_s` (+0.2 s after the close terminus). Test: `unseat_sequence`.
//!
//! Geometry (plan §2b item 4; F17): the captive seat holds the drawer at g_seat (3.5 mm,
//! the lower face) with a vertical clearance c ≈ 0.15 mm (the upper face at g_seat − c).
//! A drawer heavier than the PM force at the seat (≈ 17 kg) rests on the lower face; a
//! lighter one is pulled against the upper face. The unseat ramp therefore goes UP
//! (current rising, the gap closes at lift-off) from the lower face and DOWN (current
//! falling, possibly negative, the gap opens) from the upper face — the direction is
//! read from the rest gap. After lift-off the gap servo holds the middle of the
//! clearance g_seat − c/2 (`InSlot`); when the encoder shows the drawer out of the slot
//! (x − x_closed ≥ slot_len) the gap moves to the band-clamped zero-power gap and zero
//! power takes over (`Clear`). The z-seat step moves the gap to the clearance middle in
//! 0.75·seat_time, then ramps the current to zero; the drawer settles on the face its
//! weight selects (`Seated`), or on the travel rollers if the seat was missed (`Failed`,
//! SeatFailed; manual push-to-seat, F17).

use fabrail_params::{ParamError, SystemParams};
use fabrail_types::{FaultCode, RestPolicy};

use crate::levitation::liftland::{After, HeaveMode, LiftLand, LiftLandOut, Stage};
use crate::msg::{LevPhase, SeatStatus};

/// Seat-face tolerance for "seated" after a seat step, m (design choice: 4× the 20 µm
/// lift-off threshold; the seat clearance itself is 0.15 mm).
pub const SEAT_TOL_M: f32 = 80.0e-6;

/// Fraction of `seat_time_s` spent moving to the clearance middle; the rest (≥
/// `liftland::RELEASE_S`) releases the current (design choice).
pub const SEAT_MOVE_FRAC: f32 = 0.75;

/// Seat sequencer output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SeatOut {
    /// Status.
    pub status: SeatStatus,
    /// Heave current demand during the ramp, A.
    pub i_heave_a: f32,
    /// Mass from the lift-off closed form, kg (valid once `Unseating` → `InSlot`).
    pub m_liftoff_kg: f32,
    /// Fault raised this release (SeatFailed).
    pub fault: Option<FaultCode>,
    /// The drawer left the slot this release: retarget to zero power.
    pub exit_slot: bool,
}

/// DRAWER seat sequencer.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Seat {
    /// Status.
    pub status: SeatStatus,
    /// Elapsed time in the step, s.
    pub t_s: f32,
    /// The class has a captive seat.
    pub enabled: bool,
    /// Seat (lower face) gap g_seat, m.
    pub g_seat_m: f32,
    /// Vertical clearance c, m.
    pub clearance_m: f32,
    /// z-seat step duration, s.
    pub seat_time_s: f32,
    /// Unseat requested through the frozen [`Seat::start_unseat`].
    pub pending_unseat: bool,
    /// Seat requested through the frozen [`Seat::start_seat`].
    pub pending_seat: bool,
    /// Mass from the last lift-off, kg.
    pub m_liftoff_kg: f32,
    /// A seat request outside the slot was refused (one fault per request).
    pub seat_refused: bool,
    /// Seat approach in progress (W1.5): the drawer, closing toward its slot, descends
    /// to the clearance middle before the captive lip (the slot has no lead-in for a
    /// drawer floating at g*); `InSlot` is held outside the slot until it enters or
    /// turns back.
    pub approaching: bool,
}

impl Seat {
    /// Sequencer for classes with `RestPolicy::Seat`; `NoSeat` otherwise.
    pub fn new(p: &SystemParams) -> Result<Seat, ParamError> {
        let enabled = p.design.rest_policy == RestPolicy::Seat;
        let status = if enabled {
            SeatStatus::Seated
        } else {
            SeatStatus::NoSeat
        };
        let s = &p.design.suspension;
        let ld = &p.design.landing;
        if enabled && !(s.g_seat_m > ld.seat_z_clearance_m && ld.seat_z_clearance_m > 0.0) {
            return Err(ParamError::new(
                "design.landing.seat_z_clearance_m",
                "seat gap and clearance must be positive with g_seat > clearance",
            ));
        }
        Ok(Seat {
            status,
            t_s: 0.0,
            enabled,
            g_seat_m: s.g_seat_m as f32,
            clearance_m: ld.seat_z_clearance_m as f32,
            seat_time_s: p.control.levitation.seat_time_s as f32,
            ..Seat::default()
        })
    }

    /// Middle of the seat clearance g_seat − c/2, m.
    pub fn g_mid_m(&self) -> f32 {
        self.g_seat_m - 0.5 * self.clearance_m
    }

    /// Starts the unseat ramp (flag for the orchestrator).
    pub fn start_unseat(&mut self) {
        if self.enabled {
            self.pending_unseat = true;
        }
    }

    /// Starts the z-seat step (flag for the orchestrator).
    pub fn start_seat(&mut self) {
        if self.enabled {
            self.pending_seat = true;
        }
    }

    /// True when a seat step may start (levitated, not already seating or seated).
    pub fn can_seat(&self) -> bool {
        self.enabled && matches!(self.status, SeatStatus::Clear | SeatStatus::InSlot)
    }

    /// Unseat on the sequencer: ramp direction from the rest gap `g_rest_m` (lower face →
    /// up, upper face → down), then hold the clearance middle.
    pub fn begin_unseat(&mut self, ll: &mut LiftLand, g_rest_m: f32) {
        let dir = if g_rest_m > self.g_mid_m() { 1.0 } else { -1.0 };
        ll.start_ramp(dir, 0.0, Some(self.g_mid_m()));
        self.status = SeatStatus::Unseating;
        self.t_s = 0.0;
        self.pending_unseat = false;
    }

    /// z-seat step on the sequencer from gap `g_now_m`.
    pub fn begin_seat(&mut self, ll: &mut LiftLand, g_now_m: f32) {
        let t_move = SEAT_MOVE_FRAC * self.seat_time_s;
        ll.start_move(g_now_m, self.g_mid_m(), After::Release, t_move);
        self.status = SeatStatus::Seating;
        self.t_s = 0.0;
        self.pending_seat = false;
    }

    /// Zero power entered without a lift (caught in flight).
    pub fn caught(&mut self, in_slot: bool) {
        if self.enabled {
            self.status = if in_slot {
                SeatStatus::InSlot
            } else {
                SeatStatus::Clear
            };
        }
    }

    /// A lift from the travel rollers began (fail-down re-lift, ≤ 20 kg; F17).
    pub fn lifting_from_rollers(&mut self) {
        if self.enabled {
            self.status = SeatStatus::Failed;
        }
    }

    /// Status update after the sequencer step.
    pub fn update(
        &mut self,
        ll: &LiftLand,
        out: &LiftLandOut,
        in_slot: bool,
        gap_m: f32,
        dt_s: f32,
    ) -> SeatOut {
        self.t_s += dt_s;
        let mut o = SeatOut {
            status: self.status,
            i_heave_a: if out.mode == HeaveMode::Ramp {
                out.i_ff_a
            } else {
                0.0
            },
            m_liftoff_kg: self.m_liftoff_kg,
            ..SeatOut::default()
        };
        if !self.enabled {
            o.status = SeatStatus::NoSeat;
            return o;
        }
        if out.liftoff_kg > 0.0 {
            self.m_liftoff_kg = out.liftoff_kg;
            o.m_liftoff_kg = out.liftoff_kg;
        }
        match self.status {
            SeatStatus::Unseating => {
                if matches!(ll.stage, Stage::Move | Stage::Hold) {
                    self.status = SeatStatus::InSlot;
                } else if matches!(ll.stage, Stage::Release | Stage::Down) {
                    // Envelope abort: still in the seat.
                    self.status = SeatStatus::Seated;
                }
            }
            SeatStatus::InSlot => {
                if in_slot {
                    self.approaching = false;
                }
                if self.approaching {
                    // Descending to (or holding) the clearance middle ahead of the slot.
                } else if !in_slot && ll.stage == Stage::Hold {
                    self.status = SeatStatus::Clear;
                    o.exit_slot = true;
                } else if !in_slot && matches!(ll.stage, Stage::Zp) {
                    self.status = SeatStatus::Clear;
                }
            }
            SeatStatus::Seating => {
                if ll.stage == Stage::Down {
                    let lo = self.g_seat_m - self.clearance_m - SEAT_TOL_M;
                    let hi = self.g_seat_m + SEAT_TOL_M;
                    if gap_m >= lo && gap_m <= hi {
                        self.status = SeatStatus::Seated;
                    } else {
                        self.status = SeatStatus::Failed;
                        o.fault = Some(FaultCode::SeatFailed);
                    }
                }
            }
            SeatStatus::Failed | SeatStatus::Clear => {
                // Re-lift from the rollers (Failed) reaches zero power → Clear.
                if ll.stage == Stage::Zp {
                    self.status = if in_slot {
                        SeatStatus::InSlot
                    } else {
                        SeatStatus::Clear
                    };
                }
            }
            SeatStatus::Seated | SeatStatus::NoSeat => {}
        }
        // A landing that touched down anywhere but the seat leaves the drawer on the
        // travel rollers (SS1 fail-down): manual push-to-seat (F17).
        if out.phase == LevPhase::Touchdown
            && !matches!(self.status, SeatStatus::Seated | SeatStatus::Seating)
        {
            self.status = SeatStatus::Failed;
        }
        o.status = self.status;
        o
    }

    /// One fast release from the heave gap (m) and mean coil current (A): time and
    /// status bookkeeping only (the orchestrator drives the sequence through
    /// [`Seat::update`]).
    pub fn step(&mut self, gap_m: f32, i_mean_a: f32, dt_s: f32) -> SeatOut {
        let _ = gap_m;
        self.t_s += dt_s;
        SeatOut {
            status: self.status,
            i_heave_a: if self.status == SeatStatus::Unseating {
                i_mean_a
            } else {
                0.0
            },
            m_liftoff_kg: self.m_liftoff_kg,
            ..SeatOut::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::levitation::test_plant::{Harness, HeavePlant};
    use crate::msg::LevDirective;
    use fabrail_params::{derive, presets};

    /// Rated slot-exit mass of the DRAWER preset, kg (Wave 2 lead decision D2): the
    /// largest mass whose staged slot exit stays ≥ 10 % inside the core I²t clamp level on
    /// Carriage6 in voltage mode (measured 2026-09-27: 42 kg uses 21.6 A²s on the rear
    /// coils of the 24.3 A²s clamp level, 25.6 A²s budget).
    const SLOT_EXIT_RATED_KG_DRAWER: f64 = 42.0;

    // req: FAB-LEV-017
    #[test]
    fn unseat_sequence() {
        // DRAWER captive seat (F17; plan §2b item 4): for 8, 26, 42 and 48 kg the unseat
        // ramp lifts off within i_pulse at the closed-form current (−4.99 A at 8 kg,
        // +11.20 A at 48 kg with the balanced pole), m̂ from the lift-off closed form, and
        // the drawer is held in the slot within the lift time (100 ms, LEV-012). The slot
        // exit (drive out at a_max = 1.5 m/s², float at zero power, I²t clamp never
        // engaged, within the pulse budget) is asserted up to the rated SLOT-EXIT mass
        // of 42 kg (Wave 2 lead decision D2): on Carriage6 the pitch feed-forward of the
        // rail-level thrust adds ≈ 1.3 A to the rear coils on top of the 10.8 A seat
        // hold, and a 48 kg exit needs ≈ 30 A²s against the 25.6 A²s budget (44 kg fails,
        // 43 kg uses 23.2 of the 24.3 A²s clamp level, 42 kg 21.6). This heave-only
        // harness has no pitch coupling, so it would pass at 48 kg; it is not evidence
        // for it and 48 kg is checked for the unseat only.
        let p = presets::drawer();
        let s = &p.design.suspension;
        let d = &p.design;
        let budget = (s.i_pulse_a * s.i_pulse_a - s.i_cont_a * s.i_cont_a) * s.t_pulse_s;
        for m in [8.0f64, 26.0, SLOT_EXIT_RATED_KG_DRAWER, 48.0] {
            let exit_rated = m <= SLOT_EXIT_RATED_KG_DRAWER;
            let plant = HeavePlant::seated(&p, m);
            let mut h = Harness::new(&p, plant);
            let g_rest = h.plant.g_m;
            // Closed-form lift-off current at the rest face (F17 anchors ±0.5 %).
            let i_closed = derive::bias_current(d, g_rest, m);
            let r = h.run_unseat(&p, 1.5, 1.0);
            let i_lo = f64::from(r.i_liftoff_a);
            assert!(
                i_lo.abs() <= s.i_pulse_a,
                "{m} kg: lift-off within i_pulse 12 A: {i_lo} A (LEV-017)"
            );
            assert!(
                (i_lo - i_closed).abs() <= 0.03 * s.i_cont_a,
                "{m} kg: lift-off current {i_lo} A vs closed form {i_closed} A (tol 0.12 A; F17)"
            );
            let m_hat = f64::from(r.m_liftoff_kg);
            assert!(
                (m_hat / m - 1.0).abs() < 0.05,
                "{m} kg: m̂ from the lift-off closed form {m_hat} kg (±5 %; LEV-013/F17)"
            );
            assert!(
                r.t_in_slot_s <= p.control.levitation.lift_time_max_s as f32,
                "{m} kg: unseat to InSlot in {} s ≤ 100 ms (LEV-012)",
                r.t_in_slot_s
            );
            if !exit_rated {
                continue; // unseat only above the rated slot-exit mass (D2)
            }
            assert!(r.cleared, "{m} kg: Clear after the slot exit");
            assert!(r.levitated, "{m} kg: zero-power levitation reached");
            assert!(
                !r.saturated,
                "{m} kg: the I²t clamp never engaged (LEV-017)"
            );
            assert!(
                f64::from(r.i2t_peak_a2s) < budget,
                "{m} kg: I²t excess {} A²s within the pulse budget {budget} A²s",
                r.i2t_peak_a2s
            );
            let gz = derive::zero_power_gap(
                m,
                s.g_star_m,
                derive::h_prime(s.h_m_m, s.mu_rec),
                d.carriage.m_star_kg,
            )
            .clamp(s.zp_band_m[0], s.zp_band_m[1]);
            assert!(
                (f64::from(r.g_final_m) - gz).abs() < 5e-6,
                "{m} kg: floats at the band-clamped zero-power gap {gz} m: {}",
                r.g_final_m
            );
        }
    }

    #[test]
    fn seat_step_returns_to_the_seat() {
        let p = presets::drawer();
        for m in [8.0f64, 48.0] {
            let plant = HeavePlant::seated(&p, m);
            let mut h = Harness::new(&p, plant);
            let _ = h.run_unseat(&p, 1.5, 0.6);
            // Back at the closed position (the terminus's job) while levitated, then seat.
            h.x_accel_mps2 = 0.0;
            h.x_target_m = 0.0;
            h.plant.x_m = 0.0;
            h.run_for(&p, 0.2, LevDirective::Levitate);
            let st = h.run_for(&p, 0.6, LevDirective::Seat);
            assert_eq!(
                st.seat,
                SeatStatus::Seated,
                "{m} kg: seated after the z-seat step"
            );
            assert_eq!(st.phase, LevPhase::Off, "{m} kg: coils idle when seated");
            let lo = d_lo(&p);
            assert!(h.plant.g_m >= lo && h.plant.g_m <= p.design.suspension.g_seat_m + 1e-9);
        }
    }

    fn d_lo(p: &SystemParams) -> f64 {
        p.design.suspension.g_seat_m - p.design.landing.seat_z_clearance_m - 1e-9
    }
}
