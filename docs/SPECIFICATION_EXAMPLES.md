# fabrail example gallery: acceptance specification

This document specifies the runnable example gallery and the flagship `villa_quake` scene. It uses the grammar, columns, methods, statuses and test tokens of [SPECIFICATION.md §0](SPECIFICATION.md), and the traceability guard parses it the same way. Area code: `EXA`. Tests live in `tests/test_examples.py`; quick runs are the default and full runs are marked `slow`.

> **Status: Wave 0.** IDs, tokens and statuses are frozen. The gallery prose, figures and final numbers come later (examples unit E5; EXAMPLES.md is the reader-facing gallery).

---

## §0 Wording

### §0.1 The request, quoted

The user's phrasing is kept verbatim in this quoted prompt block and is realised only through the measurable definitions below:

> "absurdly smooth", "perfectly calculated trajectory", "log dampened soft open/close"

### §0.2 Measurable definitions

| Word | Definition used in every example and test |
|---|---|
| smooth | C² reference; peak jerk ≤ j_max; arrival speed ≤ 1 mm/s; overshoot ≤ 0.02 mm (quiet conditions, abs(a_floor,x) < 0.01 g) |
| calculated | A deterministic plan computed at alert time; every deadline met or the miss reported with a reason; byte-identical replay |
| log-damped | During terminus capture the distance-to-go is d_b·e^(−t/τ), so ln d is a straight line of slope −1/τ on a semilog chart (R² ≥ 0.999; FAB-HAP-013) |

Residual errors are always reported next to the value they qualify.

### §0.3 Banned vocabulary (quoted list for the guards)

Outside quoted prompt blocks, EXAMPLES.md, villa.html and the `examples/out` story lines never use:

> `perfect`, `perfectly`, `absurdly`, `guarantee`, `earthquake-proof`, `no damage`, `protects`

The only allowed contents wording is "no Coulomb slip event in the sliding-block model".

---

## §1 Gallery

Runner: `python -m fabrail example list | run NAME [--quick]`. Each example is a thin script with this exact filename in `examples/`, writing to `examples/out/<name>/`. The badge is the core status (✅ V, 🔶 S, 🧪 R, 🗺️ M), with 🔭 overriding when maturity is frontier. Technology maturity is a separate column in EXAMPLES.md.

| # | Example | Badge | Shows, with its required caveat |
|---|---|---|---|
| 1 | `kitchen_push_open` | ✅ | DRAWER push-to-open from the seat; log-damped open and close termini; apparent mass within ±10 % of M_v across 8–48 kg (haptic lightness does not reduce impact energy) |
| 2 | `pantry_cabinet_slider` | ✅ | 8 kg sliding cabinet door derived from DRAWER (EgressRole None, latch fitted); detents; semilog terminus plot |
| 3 | `office_door_obstruction` | 🔶 | DOOR low-energy; chest obstruction checked in simulation against numeric limits from EN 16005 (2011 draft) |
| 4 | `ballroom_partition` | ✅ | WALL 500 kg rendered at M_v = m/5; sensed vs unsensed timing; seal to ±1 mm |
| 5 | `archive_heavy_shelving` | ✅ | HEAVY hold-to-run; momentum governor with a sabotaged m̂ |
| 6 | `drawer_power_loss` | ✅ | SS1 Safe Landing; manual force; manual re-seat required above 20 kg |
| 7 | `wall_pwave_m7` | ✅ | Land-first timeline against t_S (the seismic action is not a rated safety function, SF-08) |
| 8 | `villa_quake` | 🧪 overall, per-panel badges | Flagship, §2 |
| 9 | `supertall_wind_day` | 🧪 | SENTINEL (ironless LSM): activation ≤ 0.3 mg; RMS reduction reported against ζs |
| 10 | `museum_haven_quake` | 🧪 | HAVEN-R at T = 6 s; passive vs skyhook; moat covers |
| 11 | `proton_shield_pwave` | 🧪 | SHIELD beam-off; land and lock ≤ 2 s (reduced model) |
| 12 | `vault_close_race` | 🧪 | BASTION closure race; t_a is an input; feasibility needs R ≥ 2 km |
| 13 | `stadium_tray_pod_failure` | 🧪 | ARENA static WLS reallocation after losing 1 of 500 pods (dynamic simulation is 🗺️) |
| 14 | `tidegate_offshore_m8` | 🧪 | TIDEGATE fused S-net/DAS timing; closes only on the external decision-system command |
| 15 | `pegs_long_period_securing` | 🔭 | Far-field Mw 9: a PEGS magnitude advisory plus a corroborating network alert secures a stage wagon |
| 16 | `lunar_hatch_selene` | 🔭 | numpy frontier model; pole ×0.406; lunar-dust caveat |

---

## §2 Flagship: `villa_quake`, "a house that closes its own drawers"

Label: **simulated replay**. Every alert is **synthetic/injected, not a live EEW feed**.

**Setting.** A hillside villa, plus a supertall-penthouse variant with SENTINEL:
- kitchen: 6 DRAWERs (glassware, porcelain) and 2 cabinet sliders;
- a pantry sliding DOOR on the egress path (StopAndLand: stops, lands, releases ASC, never locks);
- a salon WALL off the egress path (lands and locks);
- a HAVEN-R art plinth (🧪) in the gallery room.

**Building configurations** (panel badges): fixed-base 🔶; conventional isolation at T = 3 s 🔶 (the default comparison); HAVEN-F reduced platform 🧪 (optional).

**Warned variant.** M7.2 at R_epi 60 km, 12 km deep: t_P = 10.2 s, t_S = 17.5 s. The fused alert, confirmed by a local P pick, arrives at ≈ 7.5 s. Deadlines (5 % quantile of t_S minus 1 s) fall at 13.2–14.8 s, a budget of 5.7–7.3 s after the network alert, or 1.8–3.4 s with on-site P only (PROTECT at 11.4 s).

**Near-field variant.** R_epi 15 km, 10 km deep, S−P ≈ 2.29 s (blind zone).

**Hazards** (SAFETY.md) and the requirements that cover them:

| Hazard | Description | Requirements |
|---|---|---|
| H-18 | Pinch from an unattended automatic close | FAB-EXA-006, FAB-EXA-011 |
| H-19 | A levitated front used as a handhold during shaking | FAB-EXA-012 |
| H-20 | A drawer at standoff dropped onto its travel rollers | FAB-EXA-013 |
| H-21 | Shear at the plinth moat (1 t at 0.5 m/s is 125 J) | FAB-EXA-006 |

---

## §3 Requirements (EXA)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXA-001 | The gallery registry shall list exactly the 16 named examples, and `python -m fabrail example run NAME --quick` shall write a figure and a JSON record to examples/out/NAME/ for each (full runs are slow-tier) | gallery | PLAN §4b | T | `py:examples::registry_lists_sixteen`; `py:examples::quick_runs_write_outputs` | V |
| FAB-EXA-002 | No example's badge shall exceed the weakest requirement it demonstrates; an example on a roadmap class is labelled "reduced model", and every alert shown is labelled "synthetic/injected, not a live EEW feed" | gallery | PLAN §4b | I | `py:examples::badge_not_above_weakest_requirement`; `py:examples::reduced_model_and_synthetic_labels` | V(I) |
| FAB-EXA-003 | The FAB-EXO-005 vocabulary guard shall extend to EXAMPLES.md, villa.html and the examples/out story lines using the §0.3 list, and the contents result shall be worded "no Coulomb slip event in the sliding-block model" | gallery | PLAN §4b; FAB-EXO-005 | I | `py:examples::vocabulary_guard` | V(I) |
| FAB-EXA-004 | In the warned variant every device shall reach its commanded SafetyRank before its deadline (5 % quantile of t_S minus 1 s) or be reported NotSecured with a reason; with on-site P only, devices that cannot finish fall back to freeze and seat | villa | PLAN §4b | S | `py:examples::villa_warned_deadlines`; `py:examples::villa_onsite_only_fallback` | S |
| FAB-EXA-005 | Choreographed motion shall be staggered so that the summed bus power (summed in device index order) stays within the budget at every sync step | villa | PLAN §4b | S | `py:examples::villa_bus_power_budget` | S |
| FAB-EXA-006 | An alert shall command motion only toward a higher SafetyRank and only in the closing or seating direction, within unattended limits (v ≤ min(v_cap, 150 N/√(m_gov·k_series)), 0.23 m/s at 48 kg), with obstruction detection armed, one attempt only and independent confirmation (local P or ≥ 2 feeds); there shall be no alert-initiated opening, unlocking, de-isolating or speeding up, no reversal of user-initiated motion, and the plinth pre-centres only while its moat zone is ClearVerified | villa | PLAN §4b; FAB-EXO-506 | T, S | `rs:exo/choreo/policy::alert_motion_rule`; `py:examples::villa_no_alert_opening_or_reversal` | V |
| FAB-EXA-007 | No villa variant shall trip the safety monitor; SmoothRideThrough exits before v_lim or any monitor threshold, and a monitor trip in an example counts as a failure | villa | PLAN §4b | S | `py:examples::villa_no_monitor_trip` | S |
| FAB-EXA-008 | Hero devices (Carriage6) shall meet FAB-HAP-013 to FAB-HAP-016 in quiet windows (abs(a_floor,x) < 0.01 g), including the DRAWER close of 0.30 m in ≈ 2.0 s plus the 0.2 s seat | villa | PLAN §4b; FAB-HAP-013 | S | `py:examples::villa_hero_termini` | V |
| FAB-EXA-009 | Contents shall slip by the vector friction cone abs(a_h) > μ_s·(g + a_z), with a_h rotated by each device's heading, elasto-plastic stick-slip, a rocking flag when abs(a_h) > g·b/h, μ (glassware 0.35, porcelain 0.4, lined 0.8) swept ±30 %, and slip and contact-impulse events emitted; toppling is not modelled | villa | PLAN §4b | T, S | `rs:exo-sim/scene/contents::friction_cone_slip`; `py:examples::villa_contents_mu_sweep` | S |
| FAB-EXA-010 | Paired seed-42 runs with smoothing on and off shall show an x-slip reduction on the fixed-base building, report the isolated case with the no-slip credit given to the isolation, and report y-slip, rocking and saturation | villa | PLAN §4b | S | `py:examples::villa_smoothing_paired_runs` | R |
| FAB-EXA-011 | A hand in a drawer during a choreographed close shall be detected within 20 ms with peak force ≤ 150 N and no re-close; the device is marked NotSecured and the plan continues | villa | PLAN §4b | S | `py:examples::villa_unattended_pinch` | S |
| FAB-EXA-012 | The villa floor records shall cause no false obstruction and no false intent; a hand in the gap of a drawer at standoff during shaking is detected, raises ObstructionDegraded when the shaking-adjusted threshold exceeds 2·th0, and the choreographer then forces SeatAndHold with no automatic motion | villa | PLAN §4b | S | `py:examples::villa_no_false_obstruction_or_intent`; `py:examples::villa_hand_at_standoff_detected` | S |
| FAB-EXA-013 | Power loss at S arrival during ride-through shall give a seat-first SS1 (log-damped seat on the supercap, then SS1) within the 3× hold-up check, with bumper impacts reported | villa | PLAN §4b | S | `py:examples::villa_power_loss_seat_first` | S |
| FAB-EXA-014 | In the near-field variant a drawer gliding closed shall continue to its seat under unattended limits and a drawer gliding open shall FREEZE within 100 ms and hold, never reversed; results are reported per building configuration | villa | PLAN §4b; FAB-SEI-009 | S | `py:examples::villa_near_field_freeze` | S |
| FAB-EXA-015 | Restore shall be rejected before the INSPECT manual reset and on a remote-only command, move one device at a time at unattended limits, and end with log-damped open termini at the remembered positions that meet FAB-HAP-013 to FAB-HAP-016 | villa | PLAN §4b | T, S | `rs:exo/choreo/restore::rejects_before_inspect_and_remote_only`; `py:examples::villa_restore_termini` | V |
| FAB-EXA-016 | Seed 42 shall give byte-identical villa JSON (wall_s excluded) for any worker count; a recorded FusedAlert and device status shall replay bit-identically through the choreographer; the VILLA_RUN data block shall match the record | villa | PLAN §4b | T | `rs:exo/choreo/plan::replay_bit_identical`; `py:examples::villa_seed42_byte_identical`; `py:docs_drift::villa_run_block_matches_record` | V |
