# fabrail capability matrix

This is the ledger of what fabrail can do, how well it is verified, and how mature the underlying technology is. It follows the grammar of [SPECIFICATION.md](SPECIFICATION.md) §0.5: one status badge per row, set by the weakest requirement the capability depends on, with technology maturity in its own column. The sovereign tier's own matrix is [EXOTIC.md](EXOTIC.md) §X1.1 and the frontier matrix is [FRONTIER.md](FRONTIER.md) §1; sections 3 and 4 below mirror them, and where they differ those tables win.

> **Status: Wave 2.** Statuses are the specification's own, provisional until the Wave-2 traceability gate runs every listed test (SPECIFICATION §5). Rows marked **TODO(W2.5)** carry a provisional downgrade that Wave 2.5 finalises from the outcome of a Wave-2 lead decision (D1, D2 and the items in section 6). Nothing in this ledger has run on hardware.

**Badges.**

| Badge | Status | Meaning |
|---|---|---|
| ✅ | V or V(I) | Verified in simulation or by exact analysis against the stated tolerance; V(I) rows are verified by an inspection guard (noted in the row) |
| 🔶 | S | Verified on a simplified model; the simplification is named in [PHYSICS.md](PHYSICS.md) or the row |
| 🧪 | R | Research tier: implemented and tested, not a product capability |
| 🗺️ | M | Roadmap: not built |
| 🔭 | frontier | Replaces the status badge when the technology's maturity is frontier (theoretical, never a product; FRONTIER.md) |

**Columns.**
- **Req IDs**: the requirements the capability depends on; its badge never exceeds the weakest of them.
- **Evidence**: tests named by their specification token (§0.7): `py:<stem>::<fn>` is `tests/test_<stem>.py::test_<fn>`, `rs:<crate>/<path>::<fn>` a Rust `#[test]`. A token followed by † is a test the specification lists that Wave 2 is writing and that did not yet exist when this ledger was written; every other token names a test that exists. A "gate" is an existing check script, named where no test file exists yet. Every ✅ row names at least one existing test or gate, except the rows whose Notes say *evidence pending*, which the Wave-2 traceability gate must confirm (and downgrade if the test is not green).
- **Maturity**: the maturity of the underlying technology in the world, on the TRL scale where one applies, with the source that supports it. It is never fabrail's own maturity: every fabrail row is simulation, analysis or a build check.
- **Notes**: limits, simplifications, and where a Wave-2 decision changes the row.

Words: "checked in simulation against numeric limits from [edition]" is the only wording for standards; no row claims compliance or certification (SPECIFICATION §4).

---

## 1 Core tier (DRAWER, DOOR, WALL, HEAVY)

### 1.1 Levitation

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| Active heave and pitch in every class, roll on DRAWER and HEAVY; passive y, yaw and top-hung roll | ✅ | FAB-LEV-001 | `py:physics_anchors::earnshaw_and_roll_authority`; `rs:plant/magnet::stiffness_sum_nonpositive` | Hybrid PM-biased EMS with zero-power control: TRL 4–5, laboratory demonstrators [R29], [R31], [R35]; EMS transport TRL 9 [R94] | Earnshaw: no passively stable arrangement. DOOR roll is passive: 51 against 1030 N·m/rad (PHYSICS §3) |
| Co-energy hybrid magnet model (fringing, saturation knee, eddy lag, tempco, leakage) | 🔶 | FAB-LEV-002 | `rs:plant/magnet::coenergy_force_consistency`; `rs:plant/magnet::closed_form_y0`; `rs:plant/magnet::saturation_bound` | Reluctance model; no FEM cross-check | Saturation knee at 0.7·Φ_sat and the eddy lag are \[UNCERTAIN\] design choices; rail flange saturation not modelled (PHYSICS §2, §26) |
| Open-loop pole and linearised closed forms | ✅ | FAB-LEV-003 | `rs:params/derive::open_loop_pole`; `rs:plant/heave1::open_loop_pole_fit`; `py:levitation::open_loop_pole_fit` | Analysis | p = 69.49 / 47.57 / 32.50 / 32.50 rad/s |
| Zero-power float: current returns to zero and the gap follows m(g) | ✅ | FAB-LEV-004, FAB-LEV-005 | `py:physics_anchors::zero_power_mass`; `rs:params/derive::zero_power_mass`; `rs:core/levitation::zero_power_load_step`; `py:levitation::zero_power_load_step` | TRL 4–5 [R29], [R30] | The LEV-004 closed-loop gate (DOOR +98.1 N → 4.5967 mm ±2 µm) is the Wave-2 test; the existing core test runs on the linear test plant |
| Upper-stop and stay-landed force ratios; validate rejects violations | ✅ | FAB-LEV-006 | `py:physics_anchors::stop_force_ratios`; `rs:params/validate::rejects_bad_stops` | Analysis | Exact 0.8250 / 0.8487 / 0.8631 / 0.8631 against the quoted 0.824 / 0.850 / 0.863 (±0.002) |
| Discrete zero-power gain design on the ZOH plant with LET delay, τ_cl and τ_e; stable for τ_e ≤ 2 ms | ✅ | FAB-LEV-007, FAB-LEV-020 | `rs:params/derive::zp_gains_match_polynomial`; `rs:core/tests/levitation_linear::discrete_poles_match_design`; `rs:core/tests/levitation_linear::stable_across_float_band`; `rs:core/tests/levitation_linear::stable_for_eddy_lag_to_2ms` | Analysis on the f32 code | Assumes a laminated or SMC rail; lowest critical τ_e 2.24 ms (DRAWER pitch). HEAVY roll: section 1.9 |
| Coil current PI at ω_c = 2π·1 kHz with back-calculation anti-windup | ✅ | FAB-LEV-008 | `rs:core/current::pi_step_response`; `rs:core/current::antiwindup_recovers` | Digital current control at 8–30 kHz: TRL 9 [R116] | Motional-EMF feed-forward exists but is not routed; the PI integral absorbs the EMF (PHYSICS §4) |
| Modal allocation T·A = I and the warp loop | ✅ | FAB-LEV-009 | `rs:core/levitation/modal::allocation_identity`; `rs:core/levitation/modal::nullspace_zero_net`; `py:levitation::warp_loop_equalises` | Multi-DOF zero-power control: TRL 4 [R35] | — |
| Rail irregularity tracking at v_max (sag L/1150, 0.3 mm joints, 20 µm roughness) | 🔶 | FAB-LEV-010 | `py:levitation::rail_irregularity_tracking`; `rs:plant/rail::hermite_reproduces_quadratics_and_is_c1` | Model | WALL/HEAVY traverses are slow-tier. W1.5 DRAWER and DOOR rail_traverse runs tripped EnergyLimit at the exact cap; Wave 2 decision D4 (in code) plans at 0.90 of the energy cap and trips only on positive drive work |
| Top-hung sway: guide fin engages without magnet contact, x-drag μN, sway damping | 🔶 | FAB-LEV-011 | `rs:plant/guidance::fin_pushes_back_and_drags_mu_n`; `rs:plant/guidance::reluctance_stiffness_and_sway_anchors`; `py:levitation::top_hung_sway_push` | Guide fins and UHMW pads: TRL 9 | Implemented sway ζ ≈ 0.28 (damper at the fin point) against the preset sizing 0.08; both ≥ 0.05 (PHYSICS §6) |
| Lift from rest in ≤ 0.1 / 0.3 / 1 / 2 s without upper-stop contact | ✅ | FAB-LEV-012 | `rs:core/levitation::lift_transfer_and_land`; `rs:sim/tests/sil_smoke::drawer_unseat`; `rs:sim/tests/sil_smoke::door_hover_1s`; `py:levitation::lift_latency` | TRL 4–5 | HEAVY on Carriage6: section 1.9 (D1) |
| Payload estimate m̂ (non-safety) within ±5 % in ≤ 1 s | ✅ | FAB-LEV-013 | `rs:core/levitation/observer::payload_estimate_is_closed_form`; `py:levitation::payload_estimate_converges` | Analysis | The governor uses m_gov = max(m_rated, 1.05·m̂); the monitor uses rated mass only |
| Thermal gap shift absorbed by zero power | ✅ | FAB-LEV-014 | `py:physics_anchors::thermal_gap_shift`; `rs:params/derive::thermal_gap_shift_values`; `py:levitation::thermal_drift_absorbed` | — | −0.195 / −0.416 / −0.891 mm for +40 K; dynamic check on Heave1 with a compressed ramp |
| Rail flange saturation rule in validate | ✅ | FAB-LEV-015 | `py:params::rail_saturation_rule` | Design rule | — |
| DRAWER contactless extension limited to 50 % | 🔶 | FAB-LEV-016 | `rs:params/derive::extension_capacity_closed_form`; `py:levitation::drawer_extension_envelope` | Cantilevered maglev drawer: TRL 2–3 | 52.4 % at 48 kg on the preset stations; roller support beyond 50 % is roadmap (section 5) |
| DRAWER captive seat: unseat currents, lift-off m̂, z-seat step, seat approach | ✅ | FAB-LEV-017 | `rs:core/levitation/seat::unseat_sequence`; `rs:params/derive::seat_unseat_currents`; `rs:sim/tests/sil_smoke::drawer_unseat`; `rs:sim/tests/terminus_closed_loop::drawer_close_terminus_seats`; `py:levitation::drawer_seat_unseat` | Seat concept TRL 3; mechanical push-to-open drawers TRL 9 [R108] | Rest height depends on which seat face the load selects (up to 0.15 mm); LEV-017's ±0.05 mm wording is with the spec owner. The 48 kg slot exit is the separate TODO row in section 1.9 (D2) |
| Modified zero-power band clamp | ✅ | FAB-LEV-018 | `rs:core/levitation/zeropower::bias_current_closed_form`; `py:levitation::modified_zero_power_band` | TRL 4–5 [R30] | +0.39 A at 48 kg, 1.5 mm |
| Fail direction and impact speeds; rests on rollers; DRAWER re-lift only up to 20 kg | ✅ | FAB-LEV-019 | `rs:params/derive::impact_speed_energy`; `rs:plant/tests/anchors::impact_speed_closed_form`; `rs:core/levitation::drawer_relift_envelope`; `py:fail_direction::impact_speeds`; `py:fail_direction::rests_on_rollers`; `py:fail_direction::drawer_relift_limit` | Analysis | Above 20 kg a failed-down DRAWER needs a manual push to re-seat (F17) |

### 1.2 Propulsion and motion

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| Frames and README words: "lateral propulsion" is travel along x, "lateral" is y | ✅ | FAB-PRP-001, FAB-SW-005 | `rs:plant/body::sign_conventions`; `py:docs_drift::frame_glossary` | — | FAB-PRP-001 is an inspection guard (V(I)); PHYSICS §1 repeats the SPECIFICATION §1.2 table |
| Long-stator LSM model: FOC abstraction, K_f(x), ripple, cogging, normal force, back-EMF, ASC drag | 🔶 | FAB-PRP-002 | `rs:plant/lsm::kf_ke_and_power_balance`; `rs:plant/lsm::asc_drag_low_speed_limit`; `rs:plant/lsm::ripple_segments_and_cogging` | Long-stator LSM transport: TRL 9 [R94], [R98], [R100], [R101] | Wave 2 decision D5 (in code): amplitude-invariant dq, K_e = (2/3)·K_f, ASC drag b_e = K_f·K_e/R = (2/3)·K_f²/R (PHYSICS §7). The FAB-PRP-002 text still reads "K_f = √3·K_e" until the spec owner updates it |
| Thrust sizing ≥ m_rated·a_max + drag | ✅ | FAB-PRP-003 | `py:params::thrust_sizing` | TRL 9 [R100] | — |
| Directional force clamp: 67 N unsensed stall, 150 N sensed or last 50 mm, < 80 N within 5 s, 0.75 s drive allowance, braking unclamped | ✅ | FAB-PRP-004 | `rs:core/motion/clamp::directional_clamp_props`; `rs:core/motion/clamp::dynamic_allowance_expires`; `py:safety::stall_force_clamp` † | Low-energy door operators: TRL 9 [R103] | Numeric limits from EN 16005 (2011 draft) and A156.19, checked in simulation only. HEAVY breakaway on Travel1d: section 1.9 |
| 7-segment S-curve and jerk-limited stop distance | ✅ | FAB-PRP-005 | `rs:core/tests/scurve_props::limits_and_stop_distance`; `py:physics_anchors::d_stop_values` | Jerk-limited profiles: TRL 9 | d_stop 33.42 / 36.32 / 8.861 / 4.906 mm at the exact caps |
| Position accuracy closed / travel | ✅ | FAB-PRP-006 | `rs:sim/tests/terminus_closed_loop::drawer_close_terminus_seats`; `rs:sim/tests/terminus_closed_loop::door_close_and_open_termini`; `rs:core/fast::trajectory_arrives`; `py:propulsion::position_accuracy` | 1 µm encoders: TRL 9 [R41] | Closed-loop evidence on Carriage6 DRAWER and DOOR; WALL/HEAVY by the Wave-2 test |
| Velocity cascade with m̂a + b̂v + ripple-table feed-forward; CoM motion control | ✅ | FAB-PRP-007 | `rs:core/motion/cascade::gains_follow_design`; `rs:core/motion/cascade::iq_ref_inverts_thrust`; `py:propulsion::velocity_loop_bandwidth` | TRL 9 | W1.5: the loop controls the CoM, x_c = x_enc − h·θ̂_y, and the levitation cancels the thrust pitch torque (PHYSICS §5, §15) |
| Integer absolute position, drift-free accumulation, offset invariance, exact electrical angle | ✅ | FAB-PRP-008, FAB-PRP-009 | `rs:types/units::posaccum_no_drift_15m`; `rs:core/tests/invariance::offset_invariance`; `rs:core/foc::angle_exact_far` | Software | 0 nm drift over 1e6 steps; bit-identical under x + L and tick + 2⁴⁰ |
| Regeneration and brake chopper (≤ 16 % bus rise) | 🔶 | FAB-PRP-010 | `rs:plant/power::mains_bus_limit_and_chopper`; `py:propulsion::regen_bus_rise` | TRL 9 | Quasi-static bus model (PHYSICS §14) |
| Rail thermal expansion absorbed by homing | 🔶 | FAB-PRP-011 | `py:propulsion::thermal_expansion_homing` | TRL 9 (homing) | Wave-2 test |

### 1.3 Haptics and motion feel

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| Admittance rendering: apparent mass within ±10 % of M_v; assist-ratio bounds | ✅ | FAB-HAP-001, FAB-HAP-002 | `rs:core/haptics/admittance::free_mass_follows_newton`; `rs:core/fast::admittance_renders_virtual_mass`; `py:params::assist_ratio_bounds`; `py:haptics::apparent_mass` | Admittance control in physical human–robot interaction: TRL 9 [R53], [R54]; power-assist doors TRL 9 [R103] | Rendered lightness does not reduce impact energy: the governor and monitor use m_gov and rated mass |
| Levitated drag: breakaway ≤ 1 / 5 / 20 N and μ_eq in [1e-3, 1e-2], never 0 | 🔶 | FAB-HAP-003, FAB-HAP-010 | `py:physics_anchors::equivalent_friction_band`; `rs:plant/dissipation::mu_eq_band_at_cap_speed`; `py:haptics::breakaway_force` | Passive magnetic sliders reach μ ≈ 0.004 [R111] | "Zero friction" is a direction. c_rail and F_hyst are \[UNCERTAIN\]; the Travel1d DRAWER cavity term is numerically inflated (PHYSICS §26). W1.5 DOOR breakaway 4.86 N |
| Momentum observer for the hand force | ✅ | FAB-HAP-004 | `rs:core/haptics/observer::ramp_tracking` | Force and collision observers: TRL 9 [R56] | Bandwidth-bound: ramps ≥ 66 ms on WALL and ≥ 105 ms on HEAVY |
| Intent detection with direction consistency, false-intent rejection, DRAWER push-to-open | ✅ | FAB-HAP-005, FAB-HAP-006, FAB-HAP-012 | `rs:core/haptics/intent::lean_rejected`; `py:haptics::intent_latency`; `py:haptics::false_intent_suite`; `py:haptics::drawer_push_to_open` | Push-to-open drawer drives: TRL 9 [R107], [R108] | Disabled outside SeismicState NORMAL. W1.5 grip-arm hand profiles make intent-triggered moves drag the arm (a scenario fix in Wave 2) |
| Soft-close spring and detents | ✅ | FAB-HAP-007, FAB-HAP-008 | `rs:core/haptics/effects::soft_close_is_continuous_and_restoring`; `rs:core/haptics/effects::detent_peak_and_shape`; `py:haptics::soft_close`; `py:haptics::detent_peak` | Mechanical soft-close: TRL 9 [R108] | The log-damped terminus is the primary close; the spring is HAP-007's looser bound |
| Passivity bound and passivity observer/controller | ✅ | FAB-HAP-009 | `rs:core/haptics/passivity::stiffness_bound`; `rs:core/haptics/passivity::pc_bounds_energy`; `py:haptics::passivity_observer` | Haptic passivity control: TRL 9 [R50], [R52] | K_max = 2b/(T + 2T_d) |
| Sensor-noise force gain reported | 🔶 | FAB-HAP-011 | `py:physics_anchors::sensor_noise_force_gain`; `rs:params/derive::noise_force_gain_closed_form` | Analysis | DRAWER ≈ 0.31 N/µm at ω = 2.5·p |
| Log-damped terminus for soft-open and soft-close (semilog-linear capture, C² reference, ζ ≥ 0.9, arrival ≤ 1 mm/s, ±2 % timing, hand re-plan) | ✅ | FAB-HAP-013, FAB-HAP-014, FAB-HAP-015, FAB-HAP-016 | `rs:core/motion/terminus::exponential_capture_exact`; `rs:core/motion/terminus::reference_is_c2`; `rs:core/motion/terminus::quintic_monotone`; `rs:core/motion/terminus::integrator_freeze_poles`; `rs:core/motion/governor::terminus_not_clipped`; `rs:params/derive::terminus_t_arrive_table`; `rs:params/validate::rejects_tau_below_tau_min`; `rs:sim/tests/terminus_closed_loop::drawer_close_terminus_seats`; `rs:sim/tests/terminus_closed_loop::door_close_and_open_termini`; `rs:sim/tests/terminus_closed_loop::door_hand_during_capture_replans`; `py:haptics::terminus_semilog_fit`; `py:haptics::terminus_arrival`; `py:haptics::terminus_hand_replan` | A motion-profile technique; mechanical soft-close TRL 9 [R108] | Asserted on Carriage6 DRAWER and DOOR in quiet conditions (floor below 0.01 g). W1.5 closed loop: R² 0.99989–0.99997, overshoot 0, arrival 0.56–0.94 mm/s, timing error ≤ 1.1 % |
| Inertial smoothing (skyhook above f_c ≈ 1 Hz) and SmoothRideThrough admission | 🧪 | FAB-HAP-017, FAB-HAP-018, FAB-HAP-019 | `rs:core/motion/inertial::skyhook_discrete_stable`; `rs:core/motion/inertial::admission_rules`; `rs:core/motion/inertial::residual_follows_t_sky`; `py:haptics::inertial_residual_transmissibility`; `py:haptics::inertial_exit_to_seat` | TRL 2 | Disabled in every preset; DRAWER only; every exit is a log-damped close to the seat |

### 1.4 Safety functions

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| Energy and momentum governor with m_gov = max(m_rated, 1.05·m̂); braking envelope a_brk = 2·a_max | ✅ | FAB-SAF-001, FAB-SAF-007 | `rs:core/motion/governor::rated_mass_invariance`; `rs:core/motion/governor::braking_envelope`; `rs:core/motion/governor::energy_uses_governor_mass`; `py:safety::governor_sabotage` † | Low-energy door operators: TRL 9 [R5], [R6] | Wave 2 decision D4 (in code): the envelope uses 0.90 of the KE cap (≈ 0.949 of its speed) and 0.95 of the momentum cap, and the supervisor plans at 0.98 of it; in W1.5 the monitor tripped at the exact cap on WALL throw_soft_close and obstruction_chest |
| Unsensed speed caps; presence sensing required above them; HEAVY hold-to-run | ✅ | FAB-SAF-002, FAB-SAF-003 | `rs:params/presets::speed_caps`; `py:params::presence_sensing_gate`; `rs:core/supervisor::presence_gate_and_hold_to_run` | TRL 9 | 0.2654 / 0.1838 open, 0.152 close / 0.0446 / 0.025 m/s. HEAVY's 125 kg·m/s is stated, not capped (hold-to-run is outside A156.10) |
| EN 16005 force–time profile mirrored by the monitor | 🔶 | FAB-SAF-004 | `rs:safety/force::en16005_timer`; `py:safety::en16005_force_profile` † | — | Spring-body contact model; limits from the 2011 draft, checked in simulation only |
| Clamp force F = √(2Ek) and the closing-edge seal | ✅ | FAB-SAF-005 | `py:physics_anchors::clamp_force_energy_form`; `rs:params/derive::clamp_force_values`; `py:safety::clamp_force_sim` † | Closing-edge seals: TRL 9 | ISO/TS 15066 body values used as numbers [R7] |
| Obstruction detection: stop, then reverse 50 mm or release to 0 N | ✅ | FAB-SAF-006 | `rs:core/haptics/obstruction::residual_persistence`; `rs:core/fast::obstruction_stops_then_reverses_or_releases`; `py:safety::obstruction_stop_reverse` † | Door-operator obstruction detection: TRL 9 | Wave 2 decision D6 (in code): during an unattended close a hand in the capture latches the obstruction at its onset (≤ 10 ms; 20 ms WALL/HEAVY) |
| Overspeed monitor on integer windowed velocity (STO + ASC) | ✅ | FAB-SAF-008 | `rs:safety/velocity::integer_velocity_exact`; `rs:safety/velocity::overspeed_trips`; `rs:safety/monitor::overspeed_demands_sto_asc_until_standstill`; `py:safety::overspeed_trips` † | Safe-speed drive functions: TRL 9 [R12] | Hand-driven overspeed with the LSM idle is excused by design (core design F7) |
| SS1 Safe Landing on supply loss with hold-up ≥ 3× the need | ✅ | FAB-SAF-009 | `rs:safety/supply::holdup_energy`; `rs:core/supervisor/power::presets_meet_holdup_margin`; `rs:core/supervisor::mains_loss_lands_under_power`; `rs:core/supervisor::seat_first_from_standoff_on_mains_loss`; `py:safety::safe_landing_on_supply_loss` †; `py:power::holdup_margin` † | Supercap hold-up: TRL 9 | Monitor floor V_req = 10.498 / 6.425 / 8.415 / 27.846 V (W1.5). DRAWER open hover is supercap-limited to about 5 min |
| Manual operability after landing (22.2 N) | ✅ | FAB-SAF-010 | `py:physics_anchors::impact_speeds_and_manual_force`; `py:safety::manual_force_after_landing` † | — | PM-relieved and no-PM bound both reported (0.695 / 2.942 N on DOOR) |
| ASC policy: above v_runaway or in a seismic landing, released after stop on egress classes | 🔶 | FAB-SAF-011 | `rs:safety/monitor::overspeed_demands_sto_asc_until_standstill`; `py:safety::asc_policy` † | Active short-circuit braking: TRL 9 | D5 lowers the modelled ASC drag to (2/3)·K_f²/R |
| Independent integer monitor (KE in u128, momentum, SLP, current and force codes) | ✅ | FAB-SAF-012 | `rs:safety/energy::integer_matches_float`; `rs:safety/energy::ke_no_overflow_10mps`; `rs:safety/energy::ke_trip`; `rs:safety/monitor::nominal_motion_no_trip`; `py:safety::monitor_catches_core_fault` † | "Cat 3 intent": single-MCU software diversity, not hardware redundancy | Since Wave 2 decisions D3 and D4 (in code) the cap and position checks escalate only on positive drive work (F·v > 0); floor-driven excursions are reported (PHYSICS §18) |
| Deadline, heartbeat and non-finite faults latch until Reset | ✅ | FAB-SAF-013 | `rs:rt/exec::deadline_miss_faults`; `rs:safety/heartbeat::stale_trips`; `rs:core/supervisor/levstate::fault_latch` | TRL 9 | — |
| End zones and bumper ratings; termini never engage them | ✅ | FAB-SAF-014 | `py:params::bumper_energy_rating`; `rs:plant/contact::bumper_geometry_force_and_log`; `rs:plant/dissipation::end_zones_are_progressive_and_outside_the_travel`; `rs:plant/tests/end_stops::floor_driven_motion_stays_between_the_physical_stops`; `rs:sim/tests/end_stops_closed_loop::floor_shaking_stays_inside_the_slp_band_at_the_closed_end` | TRL 9 | Wave 2 decision D3 (in code): a jamb 2 mm past x_closed (DRAWER: the carcass push-stop) and a rail end stop behind the bumper at its rated stroke (PHYSICS §8) |
| Safety parameter CRC with diverse copies; refuse lift on mismatch | ✅ | FAB-SAF-015 | `rs:params/crc::crc_detects_corruption`; `rs:core/supervisor/levstate::refuse_lift_on_crc` | Class B-style checks: in service [R13] | Designed toward, not certified |
| The core never lowers a safety demand | ✅ | FAB-SAF-016 | `rs:rt/arbitrate::arbitration_monotone` | — | — |
| Fuzzed raw inputs cause no panic and no NaN | ✅ | FAB-SAF-017 | `rs:rt/tests/fuzz::no_panic_no_nan`; `rs:safety/tests/no_panic::fuzz_raw_inputs` | — | — |
| PayloadMismatch degrades motion to hold-to-run | ✅ | FAB-SAF-018 | `rs:core/supervisor::payload_mismatch_degrades_to_hold_to_run`; `py:safety::payload_mismatch` † | — | Severity::Degrade: levitation continues |
| ISOLATE replaces (never disables) the clamp and KE checks with IsolateLimits | 🧪 | FAB-SAF-020 | `rs:safety/envelope::isolate_limits_replace_clamp`; `rs:params/derive::t_min_iso_uses_f_cont` | TRL 2–3 | Declared R-tier deviation (SAFETY.md §4) |
| Hardware design targets: PL c/d, Class B, finger gaps, captive guideway | 🗺️ | FAB-SAF-019 | — | Not built | Designed toward, not certified |

### 1.5 Seismic

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| EEW accelerometer on the structure or rail; the carriage IMU only for skyhook and fusion | ✅ | FAB-SEI-001 | `py:params::accel_mount_rejects_carriage` | MEMS on-site EEW sensing: TRL 9 [R64] | — |
| Detector front end: CIC ÷5, 0.5–20 Hz Butterworth band-pass, STA/LTA with the exact discrete recursion | ✅ | FAB-SEI-002, FAB-SEI-003 | `rs:core/filters::butterworth_prewarp`; `rs:core/seismic/stalta::step_trigger_matches_exact_recursion`; `rs:core/seismic/frontend::decimates_by_five_with_exact_cic_gain`; `py:seismic_detector::stalta_matches_reference`; `py:seismic_detector::trigger_time_step` | STA/LTA triggering: TRL 9 [R59], [R60] | Step trigger at sample 13 (0.13 s) exactly; 0.1116 s continuous estimate reported |
| Pd and τc from the first seconds of the P wave | 🔶 | FAB-SEI-004 | `rs:core/seismic/pdtc::tauc_sinusoid`; `rs:core/seismic/pdtc::pd_linear_in_amplitude`; `py:seismic_detector::pd_recovery` | On-site τc/Pd warning: in service [R61], [R64] | S arrives inside the 3 s window when S−P < 3 s (PHYSICS §26) |
| Level matrix and PROTECT rule (Level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal) | 🔶 | FAB-SEI-005 | `rs:core/seismic/discriminate::alert_levels`; `rs:core/seismic/discriminate::protect_rule` | Calibration from P-Alert practice [R64] | — |
| False-alarm resistance: 24 h of site noise, PGA bypass under actuator veto | ✅ | FAB-SEI-006 | `rs:core/seismic/discriminate::veto_bypass_pga`; `rs:core/seismic::veto_suppresses_small_triggers_and_pga_bypasses`; `py:seismic_detector::noise_24h_no_protect` (slow); `py:seismic_detector::pga_bypass_under_veto` | — | Site-noise rates and amplitudes are \[UNCERTAIN\] (PHYSICS §19) |
| Network-alert fusion, S-ETA and cancel | 🔶 | FAB-SEI-007 | `rs:core/seismic/fusion::eta_from_alert`; `rs:core/seismic::network_alert_eta_and_cancel_through_the_unit`; `py:seismic_policy::network_alert_eta_and_cancel` | Network EEW services in operation [R66]; fabrail has an interface and a synthetic injector only | Not an EEW provider; a live feed is roadmap (section 5) |
| Seismic state machine with the INSPECT manual reset | ✅ | FAB-SEI-008 | `rs:core/seismic/fusion::fsm_exhaustive`; `rs:core/seismic/fusion::transition_table_exhaustive` | — | — |
| FREEZE within 100 ms of a local pick | ✅ | FAB-SEI-009 | `rs:core/seismic::p_pick_freezes_fast_and_waits_for_the_short_window`; `py:seismic_policy::freeze_latency` | — | — |
| Land-first class actions and the M5 R10, M7 R20 and M7.8 R100 scenarios; power loss at S | 🔶 | FAB-SEI-010, FAB-SEI-019 | `rs:core/seismic/policy::class_policy_table`; `py:seismic_policy::class_actions`; `py:seismic_policy::scenario_m5_r10`; `py:seismic_policy::scenario_m7_r20`; `py:seismic_policy::scenario_m78_r100` (slow); `py:seismic_policy::power_loss_at_s` | Automated actions on early warning are in service [R66] | The seismic action is not a rated safety function (SF-08). W1.5 WALL M7 R20: PROTECT at t_P + 1.14 s, landed and locked before t_S. power_loss_at_s tripped on floor-driven sliding in W1.5; since D3 such excursions are stopped by the physical stops and reported |
| Egress door rules: never lock, fire-rated never opens, manual operability kept | ✅ | FAB-SEI-011 | `rs:core/seismic/policy::egress_door_policy`; `py:seismic_policy::egress_door_eew` | — | — |
| Lead-time and Pd closed forms (S−P 0.1190 s/km, crossover 40.8 km, Wu–Zhao) | ✅ | FAB-SEI-012 | `py:physics_anchors::lead_time_crossover`; `py:physics_anchors::wu_zhao_pd` | — | No on-site warning under about 20 km (blind zone) |
| Why land-first: a free mass drifts by the PGD; a downward pulse saturates the lift | ✅ | FAB-SEI-013, FAB-SEI-017 | `rs:plant/travel1d::free_mass_follows_the_floor_inertially`; `py:isolation::free_mass_drifts_pgd`; `py:isolation::vertical_pulse_forces_landing` | — | — |
| Contactless isolation: skyhook, stroke governor, bumper-overload Monte Carlo | 🧪 | FAB-SEI-014, FAB-SEI-015 | `rs:core/seismic/isolate::stroke_governor`; `rs:core/seismic/isolate::cfg_from_t_iso_and_limits`; `rs:params/derive::tmd_and_isolation_anchors`; `py:isolation::transmissibility`; `py:isolation::monte_carlo_bumper_overload` (slow) | Passive maglev isolators at light-structure scale: TRL 3–4 [R112]; active maglev base isolation TRL 2 | WALL isolation largely saturates (T_min 6.3 s with F_cont). Not a product feature |
| TMD and AMD closed forms; closed-loop AMD on the 2-DOF Structure model | 🧪 | FAB-SEI-016, FAB-SEI-021 | `rs:params/derive::tmd_and_isolation_anchors`; `py:physics_anchors::damper_and_isolation_anchors`; `rs:core/seismic/amd::dvf_damps_the_structure_and_saturates`; `rs:plant/structure::lsm_tmd_emulation_matches_the_two_dof_frequency_response`; `py:tmd_amd::den_hartog_frf`; `py:tmd_amd::warburton_minimum`; `py:tmd_amd::lqr_scalar`; `py:tmd_amd::closed_loop_structure_2dof` | Building TMD and AMD: TRL 9 [R83], [R144]; a rail panel as an AMD: TRL 2 | μ = 0.05 % (5 t in 10⁷ kg) is negligible; mass dampers are mainly wind devices |
| Synthetic ground motion (KT/CP, JHT, ASCE 7-16 matching) and floor amplification | ✅ | FAB-SEI-018, FAB-SEI-020 | `py:ground_motion::kt_cp_psd`; `py:ground_motion::asce_spectrum`; `py:ground_motion::deterministic`; `py:ground_motion::floor_amplification` | Stochastic ground-motion methods in engineering use [R76], [R77] | Boore S-phase PGA is low against ground-motion models and the CP surrogate's PGD is generous (PHYSICS §26) |

### 1.6 Power and environment

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| Rest policies SEAT / HOVER / LAND / LAND + lock | ✅ | FAB-PWR-001 | `py:params::rest_policy_defaults`; `rs:params/presets::rest_policy_defaults` | — | DRAWER, WALL and HEAVY rest in contact; only DOOR hovers |
| Hover and sleep power; zero-power copper loss inside the band | 🔶 | FAB-PWR-002, FAB-PWR-003 | `py:power::hover_power_budget` †; `py:power::zero_power_copper_loss` † | Drawer drives with 0.12–0.2 W standby [R107] | Evidence pending (Wave 2). Electronics power is assumed. W1.5: a DRAWER unseated but not moved holds InSlot at ≈ 87 W (26 kg); Wave 2 decision D7 re-seats it after 2 s |
| WALL/HEAVY dock energy ≥ 3× (move + landing) | 🔶 | FAB-PWR-004 | `rs:params/derive::passivity_and_holdup`; `py:power::dock_energy_margin` † | — | Analytic plus a short simulation |
| Bus voltages 24 / 48 / 48 / 300 V and force slew; bus sized for n·i_pulse²·R | ✅ | FAB-PWR-005 | `py:params::bus_voltage_and_slew`; `rs:plant/power::mains_bus_limit_and_chopper` | SELV below 60 V except HEAVY | W1.5 resized P_max to 1000 / 800 / 1200 / 8000 W (PHYSICS §14) |
| Per-cycle energy report | ✅ | FAB-PWR-006 | `rs:plant/dissipation::ledger_accumulates_dissipation`; `rs:plant/tests/anchors::power_balance_voltage_mode`; `py:power::cycle_energy_report` † | — | — |
| Stray field by dipole sum; shroud flags | 🔶 | FAB-ENV-001 | `rs:plant/stray::dipole_on_axis`; `py:environment::stray_field_budget` | — | Bare magnets, no iron: 0.22 / 1.62 / 15.9 / 79.7 mT at 10 cm; DOOR needs a housing standoff of about 5 cm for 0.5 mT; WALL and HEAVY need shrouds |
| PWM above the audible band | ✅ | FAB-ENV-002 | `rs:params/validate::pwm_above_audible` | — | No dB(A) claim |
| DRAWER cabinet piston through a 40 cm² leak | ✅ | FAB-ENV-003 | `rs:plant/air::sealed_spring_rate`; `py:physics_anchors::piston_pressure` | — | 5.64 N at 0.5 m/s; Travel1d cavity numerics limitation (PHYSICS §26) |
| Demagnetisation proxy reported | 🔶 | FAB-ENV-004 | `py:environment::demag_margin_proxy` | — | A temperature and reverse-MMF proxy, no irreversible-loss model (PHYSICS §2) |
| Acoustics and debris on hardware | 🗺️ | FAB-ENV-005 | — | Not built | — |

### 1.7 Firmware and determinism

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| Firmware target STM32G474 / thumbv7em / RTIC 2.3.1 with pinned crates | ✅ | FAB-FW-001 | `py:firmware::manifest_pins`; gate `scripts/check_firmware.sh` (W1.5: 16 PASS) | RTIC and the STM32G4 family are in service [R119], [R121]; the fabrail firmware is built and linked, never flashed | Inspection (V(I)) |
| Shared control crates no_std, forbid(unsafe), allocation-free, fmath-only; no target divergence | ✅ | FAB-FW-002, FAB-FW-003 | `py:firmware::nostd_source_guard`; `py:firmware::dependency_allowlist`; `py:repo_contracts::forbid_unsafe`; `py:firmware::no_cfg_divergence`; gate `scripts/check_nostd.sh --strict` (W1.5: 73 PASS) | — | Inspection (V(I)) |
| LET order invariance: base first, all 6 group orders give one hash | ✅ | FAB-FW-004 | `rs:rt/tests/let_order::permutation_invariant` | — | — |
| Golden selftest hash, identical with and without fault injection | ✅ | FAB-FW-005 | `rs:rt/tests/golden::selftest_hash_frozen`; `rs:rt/tests/golden::injection_hook_is_inert_when_clear`; `py:determinism::selftest_hash_matches` | — | GOLDEN_SELFTEST_HASH stays the placeholder 0 until C1 freezes it (Wave 3); until then the tests prove determinism and fault-injection identity only |
| Bit-identical replay in voltage coil mode (the SIL ≡ firmware claim) | ✅ | FAB-FW-006 | `rs:sim/tests/replay::bit_identical`; `py:site::wasm_trace_matches_native` | — | Claimed in voltage mode only and never on a chip |
| Task map matches the design; panic policy | ✅ | FAB-FW-007, FAB-FW-009 | `py:firmware::task_map_matches_design`; `py:firmware::panic_policy_lints`; gate `scripts/check_firmware.sh` (clippy −D warnings, unsafe only in `bsp/`) | — | Inspection (V(I)) |
| Schedulability ΣC/T ≤ 0.70 from cycle estimates | 🔶 | FAB-FW-008 | `py:firmware::schedulability_budget` | Estimates, not DWT measurements | 0.64 periodic, 0.66 with CAN at 1 kHz |
| Firmware builds in release, links and meets its flash and RAM budget | ✅ | FAB-FW-010 | `py:firmware::firmware_link_size`; gate `scripts/check_firmware.sh` | Never flashed | Wave 2 decision D9 (in code): release at opt-level "z", fat LTO, one codegen unit, panic = abort, no defmt; ≈ 280–282 KB flash and ≈ 22 KB static RAM per class. The 256 KiB design target (one bank of the 512 KiB G474, the other for an A/B image) is not reached; the enforced budget is 320 KiB (62.5 % of the device, ≈ 15 % headroom), with the rationale in `scripts/check_firmware.sh` and firmware/README.md |
| Telemetry frame layout (168 bytes, no padding) | ✅ | FAB-FW-011 | `rs:types/telemetry::frame_layout` | — | — |
| Real cross builds (thumbv7em; safety crates also thumbv6m and armv7r) | ✅ | FAB-FW-013 | `py:firmware::cross_check`; gate `FABRAIL_REQUIRE_CROSS=1 scripts/check_nostd.sh --strict` | — | — |
| Hardware verification: PWM-synchronous ADC, COMP→FLT, CiA 402, HIL, flashing, DWT budgets | 🗺️ | FAB-FW-012 | — | Not built | — |

### 1.8 Software, tooling and site

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| CLI subcommands with working --help | ✅ | FAB-SW-001 | `py:cli::subcommands_help` | — | `simulate` and `seismic` were stubs in W1.5 and are Wave-2 work |
| Same seed → byte-identical JSON; ensembles invariant to the worker count | ✅ | FAB-SW-002 | `rs:sim/tests/determinism::same_seed_identical`; `rs:sim/tests/determinism::batch_worker_invariance`; `py:determinism::same_seed_identical_json`; `py:determinism::ensemble_worker_invariance` | — | — |
| Seed-42 anchors within recorded tolerances | ✅ | FAB-SW-003 | `py:acceptance::seed42_anchors` | — | The test skips until C1 records tests/data/anchors_seed42.json in Wave 3: the row is provisional until then |
| Presets validate; "path: reason" errors | ✅ | FAB-SW-004 | `rs:params/presets::all_validate`; `rs:params/validate::error_paths`; `py:params::presets_validate` | — | — |
| Drift guards: stub, PHYSICS § references, PARAMETERS.md, docs data blocks | ✅ | FAB-SW-006 | `py:stub_drift::stub_matches_native`; `py:docs_drift::physics_section_refs`; `py:docs_drift::parameters_md_current`; `py:docs_drift::run_block_matches_anchors` | — | Inspection (V(I)). PARAMETERS.md is regenerated by `python -m fabrail params --markdown` after every parameter change |
| Traceability guard, bidirectional | ✅ | FAB-SW-007 | `py:traceability::bidirectional`; `py:traceability::ids_well_formed`; `py:traceability::cross_doc_refs`; gate `python -m fabrail trace --allow-missing` (W1.5: structure ok, 0 errors) | — | Inspection (V(I)) |
| Performance budgets | ✅ | FAB-SW-008 | `py:acceptance::performance_budget` (slow); `rs:sim/tests/perf::realtime_factors` (ignored, host-dependent) | — | Wave 2 decision D8 (in the perf test): the full-rate executive stays in every tier, Travel1d ≥ 100× real time (≈ 130× on WALL; the old ≥ 1000× is unreachable at ≈ 0.4 µs per base tick) and isolation Monte Carlo 200 × 60 s ≤ 60 s on 8 workers; the FAB-SW-008 text still says ≥ 1000× until the spec owner updates it |
| JSON export records every plotted number and round-trips | ✅ | FAB-SW-009 | `py:export::json_roundtrip`; `py:export::every_plotted_number` | — | — |
| README begins with the user's two paragraphs verbatim | ✅ | FAB-SW-010 | `py:docs_drift::readme_verbatim` | — | Inspection (V(I)) |
| up.sh exclusions and .gitignore entries | ✅ | FAB-SW-011 | `py:repo_contracts::up_sh_excludes`; `py:repo_contracts::gitignore_entries` | — | Inspection (V(I)) |
| Site pages: links and assets resolve, no external URLs, theme toggle, reduced motion, tier legend | ✅ | FAB-SW-012 | `py:site::links_and_assets_resolve`; `py:site::no_external_urls`; `py:site::page_chrome` | — | Inspection (V(I)) |
| Site tokens, FORMULAS fallback, generated data files and read/ pages current | ✅ | FAB-SW-013 | `py:site::css_tokens_match_plotstyle`; `py:site::formulas_match_anchors`; `py:site::data_blocks_current`; `py:site::read_pages_current` | — | Inspection (V(I)). The data blocks are xfail until Wave 3 and the read/ pages skip until `scripts/build_site.py` exists |
| Browser engine: wasm ≤ 1.5 MB, selftest hash under node, trace equal to native | ✅ | FAB-SW-014 | `py:site::wasm_size_budget`; `py:site::wasm_selftest_matches_golden`; `py:site::wasm_trace_matches_native` | — | 801,944 bytes in W1.5; the SHA-256 is recorded in Wave 3 |
| Scenario and scene schemas; one coil-mode source; egress-lock scenes rejected | ✅ | FAB-SW-015 | `rs:sim/scenario::rejects_coil_mode_mismatch`; `rs:sim/scenario::series_bytes_channel_major_le`; `py:scenarios::specs_match_schema` †; `py:scenarios::scene_rejects_egress_lock` † | — | — |

### 1.9 Class closure by tier (Wave-1.5 outcomes and Wave-2 decisions D1, D2)

SPECIFICATION §5: Carriage6 closes end to end for DOOR and DRAWER first; WALL and HEAVY acceptance runs on Heave1 and Travel1d, with Carriage6 as a second-tier smoke test; a class that does not close is downgraded here and in the specification, never left at V.

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| DOOR on Carriage6 in voltage mode: hover, lift, moves, log-damped termini, hand take-over and re-plan, bit-identical replay | ✅ | FAB-LEV-012, FAB-HAP-013, FAB-HAP-016, FAB-FW-006 | `rs:sim/tests/sil_smoke::door_hover_1s`; `rs:sim/tests/terminus_closed_loop::door_close_and_open_termini`; `rs:sim/tests/terminus_closed_loop::door_hand_during_capture_replans`; `rs:sim/tests/replay::bit_identical` | Maglev sliding door TRL 4–5; linear-motor sliding doors TRL 9 [R103] | push_open raises one Obstruction (Warn) from the gripping-arm scenario profile (a scenario fix in Wave 2, not a controller defect) |
| DRAWER on Carriage6: unseat, slot exit up to 26 kg, termini into the seat, soft-open from the seat | ✅ | FAB-LEV-017, FAB-HAP-013, FAB-HAP-016 | `rs:sim/tests/sil_smoke::drawer_unseat`; `rs:sim/tests/terminus_closed_loop::drawer_close_terminus_seats`; `rs:sim/tests/terminus_closed_loop::drawer_open_terminus_from_seat` | Maglev drawer TRL 3 | Rated 48 kg slot exit: next row |
| DRAWER unseat plus slot exit at the rated 48 kg within the I²t pulse budget (**D2**) | 🔶 TODO(W2.5) | FAB-LEV-017 | `rs:core/levitation/seat::unseat_sequence`; `py:levitation::drawer_pulse_budget` | — | W1.5: the unseat works (m̂ 47.2 kg) but during the drive-out the rear coils hit i_pulse, the rear drops onto the seat face and the run ends in Obstruction + SaturationPersist → SS1; the budget margin at a_max is ≈ 1 % (PHYSICS §15). Wave 2 decision D2 tries a staged drive-out. **W2.5 finalises:** ✅ if `py:levitation::drawer_pulse_budget` and the 48 kg closed loop pass; otherwise stays 🔶 and states the reduced rated mass for slot exit here and in FAB-LEV-017: rated slot-exit mass = **TODO kg** |
| WALL: acceptance on Heave1 and Travel1d, Carriage6 second-tier smoke | ✅ | FAB-LEV-012 | `rs:sim/tests/sil_smoke::wall_land`; `rs:web/tests::engine_matches_native_run_heave1_drawer_and_travel1d_wall` | Operable partitions TRL 9 (mechanical) [R109] | W1.5 M7 R20 on Travel1d landed and locked before t_S (the seismic scenarios themselves are FAB-SEI-010, 🔶, section 1.5). W1.5 WALL throw_soft_close and obstruction_chest tripped the monitor at the exact cap; Wave 2 decision D4 (in code) adds the planning margin and the positive-work rule, re-checked by the Wave-2 tests |
| HEAVY on Heave1 and Travel1d: land and lock, hold-to-run moves | 🔶 TODO(W2.5) | FAB-PRP-004, FAB-SAF-002, FAB-SAF-012 | `rs:core/motion/clamp::dynamic_allowance_expires`; `rs:safety/force::en16005_timer` | — | W1.5: moves keep KE within the cap (1.58 J), but the monitor trips ForceLimit during the 5 t breakaway: the core clamp's drive allowance and the monitor's force timer disagree. **W2.5 finalises** once the two timers are reconciled (✅ if the HEAVY Travel1d catalog runs without a trip) |
| HEAVY on Carriage6 (roll–sway coupling, **D1**) | 🔶 TODO(W2.5) | FAB-LEV-001, FAB-LEV-007 | `rs:params/derive::roll_sway_coupled_design`; `rs:core/tests/levitation_linear::discrete_poles_match_design` | — | W1.5: HEAVY Carriage6 rolled away within ≈ 0.6 s at hover in every catalog entry: the per-mode roll design ignored the lateral-reluctance coupling at the pole plane 1.25 m below the CoM (coupling ratio 1.97; open-loop unstable mode 10.4 rad/s against the designed 17.3 rad/s; PHYSICS §6). Wave 2 decision D1: the coupled roll–sway design is in code and verified on the linear coupled model across τ_e 0–2 ms, k_y ±25 %, c_sway 0–3× and CoM height ±10 % (PHYSICS §15). **W2.5 finalises:** ✅ if the HEAVY Carriage6 closed-loop catalog runs green; otherwise HEAVY Carriage6 stays 🔶 and HEAVY acceptance runs on Heave1/Travel1d only |

---

## 2 Example gallery and the villa (FAB-EXA)

Requirements in [SPECIFICATION_EXAMPLES.md](SPECIFICATION_EXAMPLES.md). Every alert in an example is synthetic or injected, never a live EEW feed. The villa is research tier overall with per-panel badges.

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| Gallery registry of 16 examples, quick runs, badge and label rules, vocabulary guard | ✅ | FAB-EXA-001, FAB-EXA-002, FAB-EXA-003 | `py:examples::registry_lists_sixteen` †; `py:examples::quick_runs_write_outputs` †; `py:examples::badge_not_above_weakest_requirement` †; `py:examples::reduced_model_and_synthetic_labels` †; `py:examples::vocabulary_guard` † | — | Evidence pending (Wave 2b) |
| Villa warned variant: every device reaches its SafetyRank before its deadline or is reported NotSecured | 🔶 | FAB-EXA-004 | `py:examples::villa_warned_deadlines` †; `py:examples::villa_onsite_only_fallback` † | House choreographer: research | W1.5 smoke (fixed base, 3 devices, 5 s): pantry door secured; drawer and salon wall NotSecured (deadline) |
| Choreographed motion within the house DC bus budget (**D10**) | 🔶 | FAB-EXA-005 | `py:examples::villa_bus_power_budget` † | — | W1.5 peak 847 W against a 400 W scene budget. Wave 2 decision D10: a realistic house bus budget stated in the scene, deadline-constrained parallel scheduling within it; devices that cannot finish get the core class action and are reported NotSecured with the reason |
| Alert-motion rule: toward a higher SafetyRank, closing only, unattended limits, confirmation, one attempt | ✅ | FAB-EXA-006 | `rs:exo/choreo/policy::alert_motion_rule`; `py:examples::villa_no_alert_opening_or_reversal` † | — | A spoofed alert has bounded regret: devices close at unattended low-energy limits |
| No monitor trip in any villa variant | 🔶 | FAB-EXA-007 | `py:examples::villa_no_monitor_trip` † | — | W1.5 smoke: two trips (PositionLimit / UnintendedMotion) from floor-driven sliding past x_min; Wave 2 decision D3 (in code) stops and reports such motion |
| Hero devices (Carriage6) meet the terminus criteria in quiet windows | ✅ | FAB-EXA-008 | `rs:sim/tests/terminus_closed_loop::drawer_close_terminus_seats`; `py:examples::villa_hero_termini` † | — | — |
| Contents: vector friction cone, stick-slip, rocking flag, μ sweep | 🔶 | FAB-EXA-009 | `rs:exo-sim/scene/contents::friction_cone_slip`; `py:examples::villa_contents_mu_sweep` † | — | Worded "no Coulomb slip event in the sliding-block model"; toppling not modelled |
| Paired smoothing on/off runs | 🧪 | FAB-EXA-010 | `py:examples::villa_smoothing_paired_runs` † | TRL 2 | Isolated-building no-slip is credited to the isolation |
| Unattended pinch: detected within 20 ms, ≤ 150 N, no re-close | 🔶 | FAB-EXA-011 | `py:examples::villa_unattended_pinch` † | — | W1.5 yielded within ≈ 5 ms at 42 N but flagged at hand release (≈ 150 ms); Wave 2 decision D6 (in code) latches at the hand onset |
| No false obstruction or intent on the villa floor; a hand at standoff is detected | 🔶 | FAB-EXA-012 | `py:examples::villa_no_false_obstruction_or_intent` †; `py:examples::villa_hand_at_standoff_detected` † | — | — |
| Power loss at S arrival: seat-first SS1 within the 3× hold-up check | 🔶 | FAB-EXA-013 | `rs:core/supervisor::seat_first_from_standoff_on_mains_loss`; `py:examples::villa_power_loss_seat_first` † | — | — |
| Near field: a drawer gliding open freezes within 100 ms and is never reversed | 🔶 | FAB-EXA-014 | `py:examples::villa_near_field_freeze` † | — | Results per building configuration |
| Restore: rejected before the INSPECT reset and on remote-only commands; log-damped open termini | ✅ | FAB-EXA-015 | `rs:exo/choreo/restore::rejects_before_inspect_and_remote_only`; `py:examples::villa_restore_termini` † | — | — |
| Villa determinism, worker invariance and choreographer replay | ✅ | FAB-EXA-016 | `rs:exo/choreo/plan::replay_bit_identical`; `py:examples::villa_seed42_byte_identical` †; `py:docs_drift::villa_run_block_matches_record` | — | — |

---

## 3 Sovereign tier (FAB-EXO; mirrors EXOTIC §X1.1)

Safe by separation, not by energy limit: every sovereign class declares a safety regime and a safe-state policy. Maturity follows EXOTIC §1; the villa choreographer is in section 2.

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| Tier rules, word guards, dependency closure | ✅ | FAB-EXO-001, FAB-EXO-002, FAB-EXO-003, FAB-EXO-004, FAB-EXO-005, FAB-EXO-006, FAB-EXO-116 | `py:exotic/frontier::every_output_bannered`; `rs:exo-safety/types::guarded_zone_not_for_pedestrian_classes`; `rs:exo-safety/tests/dependency_closure::closure_excludes_control_and_plant`; `py:exotic/exo_guards::one_badge_per_capability` †; `py:exotic/exo_guards::vocabulary_guard` † | Tooling | — |
| Class presets, regimes and safe-state policies as data | ✅ | FAB-EXO-101, FAB-EXO-102 | `rs:exo-safety/presets::class_table_matches_plan`; `rs:exo-safety/presets::every_class_declares_regime_and_policy` | Classes TRL 1–4 (EXOTIC §1) | — |
| Single pole-area admission and its generated tables | 🔶 | FAB-EXO-103, FAB-EXO-121, FAB-EXO-207 | `rs:exo-sim/suspension::admission_single_function`; `rs:exo-sim/suspension::feco_pressure_and_rail_cap`; `py:exotic/classes::admission_table_per_class` † | Closed form; FeCo poles TRL 5 | — |
| Stray field at accessible surfaces, or a shroud flag | 🔶 | FAB-EXO-104 | `py:exotic/classes::stray_field_or_shroud_flag` † | Dipole model, no FEM | — |
| Class kinematic and acceleration budgets | ✅ | FAB-EXO-110 | `py:exotic/classes::acceleration_budget_closed_form` † | Closed form | Evidence pending (Wave 2) |
| Curved-guideway sagitta and tilt admission | 🔶 | FAB-EXO-111 | `rs:exo-sim/platform::guideway_sagitta_tilt_admission` | Geometry; articulated pods TRL 2–3 | — |
| Integer units and the integer envelope | ✅ | FAB-EXO-112, FAB-EXO-903 | `rs:exo-safety/units::integer_range_1kg_to_1e8kg`; `rs:exo-safety/envelope::matches_f64_reference` | Software | — |
| HTS flux-pinning model (fit, loop, creep, LN₂, fade) | 🔶 | FAB-EXO-113, FAB-EXO-202, FAB-EXO-203, FAB-EXO-204, FAB-EXO-205 | `rs:exo-sim/suspension::hts_out_of_sample`; `rs:exo-sim/suspension::hts_fit_atz`; `rs:exo-sim/suspension::hts_loop_conservative_and_dissipative`; `rs:exo-sim/suspension::hts_creep_and_ln2_hold`; `rs:exo-sim/suspension::hts_ln2_landing_and_fade` | TRL 5–6 (bulk HTS demonstrators; one data set) | — |
| HTS lift with an EMS trim coil | 🧪 | FAB-EXO-208 | `rs:exo-sim/suspension::hts_ems_trim_creep_drift` | TRL 3–4 | — |
| Hover loss at scale, pure and hybrid EMS | 🔶 | FAB-EXO-206 | `rs:exo-sim/suspension::hover_loss_anchors` | Pure EMS TRL 9 [R94]; hybrid EMS at kt scale TRL 3 | — |
| No-power safe state for every suspension kind | 🔶 | FAB-EXO-115 | `rs:exo-sim/suspension::no_power_safe_state_reachable` | Stops and catchers TRL 9; the combination TRL 3 | — |
| Non-magnetic baselines (air pad, FPS) | 🔶 | FAB-EXO-209 | `rs:exo-sim/suspension::baselines_available` | TRL 9 | — |
| Storage models and the energy ledger | 🔶 | FAB-EXO-401, FAB-EXO-404 | `rs:exo-sim/storage::flywheel_usable_fraction`; `rs:exo-sim/storage::energy_balance_closes` | Flywheel, supercap, battery TRL 9; SMES documented only | — |
| Ride-through sizing with T_ride ≥ 300 s | ✅ | FAB-EXO-120, FAB-EXO-403 | `rs:exo/energy::t_ride_covers_shaking_duration`; `rs:exo/energy::ride_through_anchors` | Closed form | — |
| Energy admission with the absorption check | 🔶 | FAB-EXO-122, FAB-EXO-402, FAB-EXO-405 | `rs:exo/energy::absorption_headroom`; `rs:exo/energy::admission_discharge_and_absorb`; `rs:exo/energy::hts_ride_through_kw` | Closed form over simplified stores | — |
| 2oo3 mid-value voter, ladder, common-cause handling | ✅ | FAB-EXO-301, FAB-EXO-302, FAB-EXO-303 | `rs:exo-safety/vote::mvs_single_fault_bound`; `rs:exo-safety/ladder::degradation_sequence`; `rs:exo-safety/ladder::common_cause_reaches_safe_state` | Architecture in service elsewhere; fabrail logic only | Designed toward, not certified |
| Diverse inductance channel L(g) as tie-breaker | 🧪 | FAB-EXO-304 | `rs:exo-safety/ladder::diverse_tiebreaker` | TRL 3–4 (self-sensing) | — |
| Black channel | ✅ | FAB-EXO-306 | `rs:exo-safety/black_channel::rejects_corrupt_stale_replay` | Concept in service elsewhere | — |
| PFH calculator in two forms with a CTMC cross-check | ✅ | FAB-EXO-117, FAB-EXO-305 | `rs:exo-safety/pfh::ctmc_cross_check`; `rs:exo-safety/pfh::two_forms_anchor` | Analysis aid, placeholder rates | — |
| Guarded-zone machine, ISO 13855 distances, attended jog, bumper rating | ✅ | FAB-EXO-501, FAB-EXO-502, FAB-EXO-503, FAB-EXO-504, FAB-EXO-507 | `rs:exo-safety/zone::motion_only_when_clear_verified`; `rs:exo-safety/zone::intrusion_ss1_stop_distance`; `rs:exo-safety/envelope::attended_jog_limits`; `rs:exo-safety/envelope::bumper_rating_rated_mass`; `rs:exo-safety/zone::presence_wins_without_two_person_override`; `py:exotic/classes::iso13855_distances` † | Presence sensors and interlocks TRL 9 | — |
| Alert gate, SafetyRank and the spoof matrix | ✅ | FAB-EXO-119, FAB-EXO-506 | `rs:exo-safety/alert_gate::spoof_action_matrix`; `rs:exo-safety/rank::alert_moves_only_toward_higher_rank`; `rs:exo-safety/alert_gate::exhaustive_transitions` | Software | — |
| TMD tuning closed forms and the harmonic sweep | ✅ | FAB-EXO-601, FAB-EXO-602 | `rs:exo/tuning::den_hartog_warburton_closed_forms`; `rs:exo-sim/host::harmonic_sweep_peak` | TMD practice TRL 9 [R144] | — |
| SENTINEL hybrid mass damper (saturation, mode ladder, wind, detuning) | 🧪 | FAB-EXO-603, FAB-EXO-604, FAB-EXO-605, FAB-EXO-606, FAB-EXO-607, FAB-EXO-608 | `rs:exo-sim/host::zeta_add_stroke_bound`; `rs:exo/hmd::stroke_governor_engages_first`; `rs:exo/hmd::mode_ladder_and_fail_behaviour`; `rs:exo-sim/scenarios::near_fault_pulse_peak` †; `rs:exo-sim/scenarios::wind_rms_and_activation` †; `rs:exo-sim/scenarios::detuning_stability` † | TRL 3–4 (1,000 t maglev planar bearing); eddy damping TRL 9 | Mainly a wind device |
| Coulomb activation threshold and rail condition | 🔶 | FAB-EXO-609 | `rs:exo-sim/host::coulomb_activation_threshold` | Reduced SDOF model | — |
| HAVEN-R isolation platform, 3-DOF | 🧪 | FAB-EXO-505, FAB-EXO-701, FAB-EXO-702, FAB-EXO-703, FAB-EXO-704, FAB-EXO-705, FAB-EXO-706, FAB-EXO-707, FAB-EXO-118 | `rs:exo-sim/platform::occupied_platform_limits`; `rs:exo-sim/platform::curved_guideway_period`; `rs:exo/isolation::transmissibility_anchors`; `rs:exo-sim/platform::control_loss_fps_landing`; `rs:exo-sim/platform::rejects_no_passive_vertical`; `rs:exo-sim/platform::copper_temperature_rise`; `rs:exo/isolation::recentring_band_and_shaking_off`; `rs:exo-sim/platform::copper_sigma_temperature`; `rs:exo-sim/scenarios::stroke_within_moat_and_fling` † | TRL 2–3 | Not a product feature |
| HAVEN-F facility at 5 kt, admission only | 🧪 | FAB-EXO-103 | `rs:exo-sim/suspension::admission_single_function` | Research, extrapolated | — |
| ARENA static pod reallocation | 🧪 | — | `rs:exo/alloc::reallocation_on_pod_loss_and_quadrant_land`; `rs:exo/alloc::wls_meets_wrench_with_equal_utilisation` | TRL 3; dynamic simulation 🗺️ | No requirement row (EXOTIC §X1.1) |
| EEW fusion: maximum intensity, capped log-odds, cost–loss | 🔶 | FAB-EXO-801, FAB-EXO-802, FAB-EXO-805 | `rs:exo/fusion::feed_kinds_and_injector_gating`; `rs:exo/fusion::max_intensity_eta_logodds_costloss`; `rs:exo/fusion::cancel_returns_normal` | S-net and DAS feeds in service or early service; fusion simplified | Not an EEW provider |
| Lead-time gain and blind zone | ✅ | FAB-EXO-803 | `rs:exo-sim/eew_timing::lead_time_gain_per_km` † | Closed form | Evidence pending (Wave 2 X6) |
| Offshore warning scenario | 🔶 | FAB-EXO-804 | `rs:exo-sim/eew_timing::offshore_m8_warning` † | S-net in service; timing model simplified | — |
| PEGS advisory gating (never a sole trigger) | ✅ | FAB-EXO-810 | `rs:exo/fusion::pegs_never_sole_trigger` | PEGS magnitude research (Mw ≥ 8.3) | — |
| Air-blast closed forms and the closure race | ✅ | FAB-EXO-806, FAB-EXO-807, FAB-EXO-808 | `rs:exo/closure::close_iff_feasible`; `py:exotic/pressure::friedlander_and_reflection` †; `py:exotic/exo_guards::no_yield_distance_tables` † | Friedlander textbook; BASTION TRL 2–3 | No yield-to-distance tables |
| Tsunami timing and closure budget | 🔶 | FAB-EXO-809 | `rs:exo-sim/tsunami::timing_greens_law_budget` † | Triggering TRL 7–9; maglev gate TRL 2 | Closes only on the external decision-system command |
| no_std cross builds of the exotic crates | ✅ | FAB-EXO-901, FAB-EXO-902 | `py:firmware::cross_check`; `py:firmware::nostd_source_guard`; gate `FABRAIL_REQUIRE_CROSS=1 scripts/check_nostd.sh --strict` | Built as rlibs, never flashed | — |
| Sovereign firmware apps (1oo2D, 2oo3) and WCET | 🗺️ | FAB-EXO-904 | — | Not built | — |
| Assurance wording | ✅ | FAB-EXO-905 | `py:exotic/exo_guards::designed_toward_wording` † | — | Evidence pending (Wave 2 X7) |
| SELENE lunar hatch | 🔭 | FAB-FRN-006 | `py:exotic/frontier::selene_lunar_scaling` | TRL 1–2 | numpy frontier only |

---

## 4 Frontier tier (FAB-FRN; mirrors FRONTIER §1)

What-if studies, never product claims. Every output carries a banner, `"tier": "frontier"` and its missing orders of magnitude.

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| PEGS on-site gravity gradiometer | 🔭 | FAB-FRN-003 | `py:exotic/frontier::pegs_detection_and_gap` | Frontier; anchor 🧪 PEGS network magnitude (Mw ≥ 8.3) | Needs 1e-15/√Hz, best demonstrated 1e-10/√Hz: 5 orders short; ≈ 0 s gain against a dense P network |
| Ambient-pressure RTSC suspension | 🔭 | FAB-FRN-001 | `py:exotic/frontier::rtsc_sweep` | Frontier; anchor 🧪 HTS flux pinning | Needs Tc ≈ 358 K; the pressure ceiling stays rail-limited |
| Building-scale active metabarrier | 🔭 | FAB-FRN-004 | `py:exotic/frontier::metabarrier_mass_law` | Frontier; anchor 🧪 | 1.8 Mt of resonating mass at 1 Hz and v_s = 100 m/s |
| Structure-scale magnetic cloak or concentrator | 🔭 | FAB-FRN-007 | `py:exotic/frontier::cloak_permeability` | Frontier; anchor 🧪 cm-scale lab cloaks | Gain capped at B_sat/B0 |
| SELENE lunar hatch and cargo rail | 🔭 | FAB-FRN-006 | `py:exotic/frontier::selene_lunar_scaling` | Frontier; anchor 🧪 FLOAT concept | Pole × 0.406; nanophase-Fe dust on PM guideways |
| Trapped-field bulk magnets | 🧪 | FAB-FRN-002 | `py:exotic/frontier::trapped_field_cap` | Research (relabelled from 🔭) | Pressure capped by rail saturation, ≤ 2.1 MPa against steel |
| Diamagnetic levitation | ✅ | FAB-FRN-009 | `py:exotic/frontier::diamagnetic_pressure` | Physics deployed; FLOAT research | ✅ applies to the physics only: 179 Pa at 1 T as an upper bound |
| Quasi-zero-stiffness isolation | 🧪 | FAB-FRN-005 | `py:exotic/frontier::qzs_stiffness_and_tilt` | Deployed at instrument scale; research at building scale | — |
| Missing orders of magnitude printed by every frontier output | ✅ | FAB-FRN-008 | `py:exotic/frontier::missing_orders_printed` | — | — |
| Frontier matrix generated from the module metadata | ✅ | FAB-FRN-010 | `py:exotic/frontier::matrix_matches_code` | — | Inspection (V(I)) |

---

## 5 Roadmap (🗺️, not built)

| Capability | Status | Req IDs | Evidence | Maturity | Notes |
|---|---|---|---|---|---|
| Hardware demonstration of the safety design targets | 🗺️ | FAB-SAF-019 | — | — | Designed toward, not certified |
| Firmware on hardware: flashing, HIL, PWM-synchronous ADC, COMP→FLT latency, CiA 402, DWT cycle budgets | 🗺️ | FAB-FW-012 | — | — | The firmware builds and links only |
| Acoustics (dB(A)) and debris | 🗺️ | FAB-ENV-005 | — | — | — |
| Sovereign firmware apps and WCET | 🗺️ | FAB-EXO-904 | — | — | RTIC has no Cortex-R support |
| Live EEW feed (ShakeAlert, JMA) | 🗺️ | — | — | Services in operation [R66] | Needs a licensed partnership |
| FEM or magpylib cross-check of the magnet model | 🗺️ | — | — | — | — |
| DRAWER extension beyond 50 % on roller support | 🗺️ | — | — | — | FAB-LEV-016 caps contactless travel at 50 % |
| Multi-carriage rails, curves and switches; separate carriage and track nodes; a diverse second MCU | 🗺️ | — | — | — | — |
| Dynamic simulation of ARENA, ROTATOR, TIDEGATE, SELENE and full HAVEN-F; H∞ control; SMES | 🗺️ | — | — | — | Reduced models only (EXOTIC.md) |

---

## 6 Wave-2 lead decisions and where this ledger records them

| Decision | Affects | Recorded in | Ledger status |
|---|---|---|---|
| D1 HEAVY Carriage6 roll divergence: coupled roll–sway modal design, time-boxed; else HEAVY Carriage6 🔶 and acceptance on Heave1/Travel1d | FAB-LEV-001, FAB-LEV-007 | PHYSICS §6, §15; ledger section 1.9 | Design in code; closed-loop outcome TODO(W2.5), provisional 🔶 |
| D2 DRAWER 48 kg slot exit: staged drive-out within the I²t budget; else state the reduced rated mass | FAB-LEV-017 | PHYSICS §15; ledger section 1.9 | TODO(W2.5) row, provisional 🔶, mass TODO |
| D3 Physical closed-end and open-end stops; SLP/SDI/UnintendedMotion escalate only while the drive injects energy | FAB-SAF-012, FAB-SAF-014, FAB-SEI-019, FAB-EXA-007 | PHYSICS §8, §18 | In code (plant stops, monitor); notes on those rows |
| D4 Governor plans at ≤ 0.90 of the KE cap and ≤ 0.95 of the momentum cap; monitor trips only with positive drive work | FAB-SAF-001, FAB-SAF-012, FAB-LEV-010 | PHYSICS §16, §18 | In code (governor, monitor); notes on those rows and the WALL row in section 1.9 |
| D5 ASC convention: e_q = K_e·v, K_e = (2/3)·K_f, b_e = (2/3)·K_f²/R | FAB-PRP-002, FAB-SAF-011 | PHYSICS §7 | In code (params, plant, derive); FAB-PRP-002 text to follow |
| D6 Unattended-pinch flag at the yield instant (≤ 20 ms) | FAB-SAF-006, FAB-EXA-011 | PHYSICS §17 | In code (fast loop); notes on those rows |
| D7 DRAWER lifted but not moved re-seats after 2 s idle (or drives out if a move is pending) | FAB-PWR-002 | PHYSICS §15 | Note on the hover-power row |
| D8 Full-rate executive in every tier; Travel1d ≥ 100×; isolation MC 200 × 60 s ≤ 60 s on 8 workers | FAB-SW-008 | PHYSICS §24 | In code (perf budgets); FAB-SW-008 text to follow |
| D9 Firmware flash toward 256 KB, or a restated budget with its rationale | FAB-FW-010 | ledger section 1.7 | In code: 320 KiB budget restated with rationale (256 KiB not reached) |
| D10 Realistic house DC bus budget; deadline-constrained parallel scheduling within it | FAB-EXA-004, FAB-EXA-005 | ledger section 2 | Notes on those rows |

**Updating this ledger.** A requirement status change updates SPECIFICATION (or its companion), the test markers and this matrix in the same edit, then `python -m fabrail trace` runs. A row is never left above the weakest requirement it depends on, and a † token becomes a plain token once the test exists and passes.
