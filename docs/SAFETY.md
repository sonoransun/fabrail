# fabrail safety case

**Design evidence, not certification.** This document lists the hazards of a magnetically suspended, magnetically propelled rail, maps each to the requirements that address it, describes the safety functions and their declared design targets, records a component-level failure analysis, and states the argument that ties the evidence together. All evidence is simulation, analysis or inspection of the code. Nothing here has been demonstrated on hardware, assessed by a third party or certified, and nothing claims compliance with any standard.

Companion documents: [SPECIFICATION.md](SPECIFICATION.md) (the requirements cited as FAB IDs), [TECHNICAL_DESIGN.md](TECHNICAL_DESIGN.md) §8 (the mechanism: channels, arbitration, layers), [PHYSICS.md](PHYSICS.md) (the models the evidence rests on), [SEISMIC.md](SEISMIC.md) (the seismic readings), and [EXOTIC.md](EXOTIC.md) Section §S (the Sovereign-tier safety argument: regimes, 1oo2D/2oo3 architectures designed toward SIL targets, the PFH calculator).

> **Wording rule.** Standards are used only as sources of numbers. The only allowed wording is "checked in simulation against numeric limits from [edition]". Performance levels (PL) and safety integrity levels (SIL) are declared design targets that the design is "designed toward"; none is claimed.

Status badges follow SPECIFICATION §0.5: ✅ verified in simulation, 🔶 simplified model, 🧪 research tier, 🗺️ roadmap.

---

## §1 Scope, boundary and assumptions

### §1.1 What is covered

- The four core classes: DRAWER (8–48 kg, captive seat at rest), DOOR (100 kg, hovers at rest), WALL (500 kg, lands at park), HEAVY (5 t, lands and locks; hold-to-run unless PL d presence sensing is fitted).
- One combined node per carriage (v0.1): carriage sensing, coil drivers, LSM drive, supervisor and the safety monitor on one MCU.
- The control stack (`fabrail-core`), the integer monitor (`fabrail-safety`), the executive and arbitration (`fabrail-rt`), and the parameter block (`fabrail-params`).
- The example-gallery additions: unattended choreographed closes, drawers at an open standoff during shaking, and the research-tier isolation plinth (H-18 to H-21).

Not covered: building structure, installation, electrical installation codes, EMC, and the Sovereign-tier classes (EXOTIC §S).

### §1.2 Assumptions

The argument (§7) holds only under these assumptions. Each is a statement about the world outside the code.

| ID | Assumption |
|---|---|
| A-01 | The plant model (PHYSICS) represents the hardware within the swept uncertainties: magnet leakage and fringing, eddy lag τ_e ≤ 2 ms on a laminated or SMC rail, rail eddy and hysteresis drag |
| A-02 | A closing-edge seal with k_edge ≤ 10 N/mm is fitted on every powered closing edge |
| A-03 | m_rated is set correctly at installation and the parameter block is sealed; the installer does not raise it beyond the class range |
| A-04 | The mechanics exist as designed: upper stop as a sliding surface, rollers (μ 0.003), captive guideway, bottom guide fin, DRAWER captive seat, 150 mm end zones, bumpers at their rating |
| A-05 | The supercap and its voltage measurement are as specified (DRAWER 10 F at 12 V; HEAVY 10 F at 48 V) and are replaced before fade exceeds the hold-up margin |
| A-06 | The EEW accelerometer is mounted on the structure or the rail, never the carriage |
| A-07 | Presence sensing, where fitted, meets its declared PL d target; fabrail models only its healthy/unhealthy output |
| A-08 | The hardware overcurrent path (comparators into the TIM1 break and HRTIM fault inputs) works as designed; it is not verified (FAB-FW-012) |
| A-09 | Both channels run on one MCU: software diversity only ("Cat 3 intent"), with no protection against a common-cause MCU failure beyond the watchdogs and the mechanics |
| A-10 | Human contact follows the ISO/TS 15066 body models used by the plant (hand 75 N/mm, 0.6 kg; chest 25 N/mm, 40 kg) and the modelled arm impedance |
| A-11 | Network alerts are synthetic or injected; no live EEW feed is connected |
| A-12 | Every opening's egress role (none, non-rated, fire-rated) is declared correctly at installation |

---

## §2 Hazards

Each hazard maps to the requirements whose tests carry the evidence. The traceability guard checks that every row names at least one existing requirement (FAB-SW-007).

| ID | Hazard | Classes | Design measures | Requirements | Residual |
|---|---|---|---|---|---|
| H-01 | Crush or shear at the closing edge | all | Unsensed caps (1.69 J, 22.3 kg·m/s); directional clamp (stall ≤ 67 N unsensed, 150 N sensed or in the last 50 mm, < 80 N within 5 s); closing-edge seal; obstruction stop and reverse | FAB-SAF-002, FAB-SAF-004, FAB-SAF-005, FAB-SAF-006, FAB-PRP-004 | Clamp forces hold for the ISO/TS 15066 body models (A-10) and a fitted edge seal (A-02) |
| H-02 | Impact while opening toward the user (drawers) | DRAWER | 0.2654 m/s both ways (1.69 J at 48 kg); governor with m_gov; braking envelope at a_brk = 2·a_max | FAB-SAF-001, FAB-SAF-002, FAB-SAF-007, FAB-HAP-001 | Haptic lightness (M_v = m/1.5) does not reduce the impact energy of the real mass |
| H-03 | Drop on loss of levitation | all | SS1 Safe Landing on the supercap at ≤ 5 mm/s; stay-landed and upper-stop ratios; either fail direction ends at rest; loop stable for τ_e ≤ 2 ms | FAB-SAF-009, FAB-LEV-006, FAB-LEV-019, FAB-LEV-020 | A control loss at g* lands at `derive::impact_speed` with ≤ 2 bounces; a solid-steel rail may exceed the τ_e bound (A-01) |
| H-04 | Snap-up clamp jams an egress door | DOOR | Upper stop is a sliding surface loaded to ≤ 1.30·m*·g; the door slides by hand in either fail direction; manual force reported | FAB-LEV-006, FAB-LEV-019, FAB-SAF-010 | Manual force at the upper stop depends on the sliding-surface friction (A-04) |
| H-05 | Unintended motion or false intent | all | Intent needs force × time or displacement plus direction consistency; lean, smoke and piston profiles rejected; the monitor catches injected core faults; the core can never lower a demand | FAB-HAP-005, FAB-HAP-006, FAB-SAF-012, FAB-SAF-016 | Intent rejection is shown against modelled disturbances only |
| H-06 | Overspeed or excess kinetic energy | all | Governor with m_gov = max(m_rated, 1.05·m̂); unsensed caps; monitor overspeed → STO + ASC; integer energy and momentum checks; PayloadMismatch → hold-to-run | FAB-SAF-001, FAB-SAF-002, FAB-SAF-008, FAB-SAF-012, FAB-SAF-018 | A sealed but wrong m_rated is not detectable by software (A-03) |
| H-07 | Failure to open on an escape route | DOOR | Egress doors never lock; ASC released after stop; SS1 on power loss; manual force after landing ≤ 22.2 N | FAB-SEI-011, FAB-SEI-019, FAB-SAF-010, FAB-SAF-011 | Relies on free rollers (FMEA: seized roller) and a correct egress role (A-12) |
| H-08 | Seismic free mass, derailment or overturning | all | Land-first class actions; vertical saturation forces a landing; captive guideway; end zones and rated bumpers | FAB-SEI-010, FAB-SEI-013, FAB-SEI-017, FAB-SAF-014 | A levitated free mass drifts by ≈ PGD; bumpers are rated on a 1 m/s (DRAWER) or 1.5 m/s basis, above which the end stops take the energy |
| H-09 | Seismic false or missed alarm | all | 24 h site noise gives 0 PROTECT; PGA bypass under the actuator veto; level matrix and PROTECT rule; network-only alerts may only freeze, land or lock | FAB-SEI-005, FAB-SEI-006, FAB-SEI-007, FAB-SEI-010, FAB-EXO-119 | No useful on-site warning below ≈ 20 km; the seismic action is not a rated safety function (SF-08) |
| H-10 | Fire-rated door opening on an alert | DOOR | Fire-rated doors never open on the seismic action; OpenAndStop is forbidden whenever a non-local source contributed | FAB-SEI-011 | Depends on the declared egress role (A-12) |
| H-11 | Static field exposure for implant wearers; magnet handling | all | Dipole-sum field model against a ≤ 0.5 mT at 10 cm target; WALL/HEAVY flagged "shroud required" | FAB-ENV-001 | No measured field; WALL ≈ 16 mT and HEAVY ≈ 80 mT unshielded; magnet handling is an installation procedure, not modelled |
| H-12 | Electrical and thermal (HEAVY bus, supercap) | all | SELV (< 60 V) except HEAVY at 300 V; brake chopper at 1.15·V_bus; coil thermal model with OverTemp; I²t coil clamp | FAB-PWR-005, FAB-PRP-010, FAB-SAF-009 | HEAVY insulation, supercap handling and enclosure design are hardware items (🗺️) |
| H-13 | Finger entrapment in gaps or the rail slot | all | Powered closing is force-clamped and obstruction-monitored; finger-gap geometry and the captive guideway are declared hardware targets | FAB-SAF-019, FAB-SAF-004, FAB-SAF-006 | The geometry is roadmap and unverified |
| H-14 | Power loss mid-travel | all | SS1 on the supercap: stop at a_brk, land, STO; hold-up ≥ 3× the landing energy checked before lift; dock energy margin for WALL/HEAVY | FAB-SAF-009, FAB-SEI-019, FAB-SAF-010, FAB-PWR-004 | Supercap fade between checks (A-05) |
| H-15 | Wrong rated mass or corrupted parameters | all | CRC32 plus diverse copies, refuse lift on mismatch; validate with "path: reason"; PayloadMismatch; m_gov never below m_rated | FAB-SAF-015, FAB-SAF-018, FAB-SW-004, FAB-SAF-001 | A consistently wrong sealed value (A-03) |
| H-16 | Haptic instability or limit cycle | all | Virtual stiffness K ≤ 2b/(T + 2T_d); passivity observer and controller; validate bounds the assist ratio | FAB-HAP-009, FAB-HAP-002 | Human arm impedance is modelled (1–2 kg, 5–20 N·s/m, 200–1000 N/m) |
| H-17 | DRAWER fail-down not re-liftable | DRAWER | Seat-first SS1 from the open standoff; re-lift from the 6.5 mm rollers only up to 20 kg (i_pulse 12 A); open hover limited to ≈ 5 min by the supercap | FAB-LEV-019, FAB-LEV-017, FAB-SAF-009 | Above 20 kg the user must push the drawer closed so the seat ramp re-seats it (≈ 1.4 N on rollers) |
| H-18 | Pinch from an unattended automatic close | DRAWER/DOOR (villa) | Unattended limit v ≤ min(v_cap, 150 N/√(m_gov·k_series)), 0.23 m/s at 48 kg; obstruction armed; one attempt; the device is reported NotSecured on obstruction | FAB-EXA-006, FAB-EXA-011 | Detection ≤ 20 ms and peak ≤ 150 N are shown for the modelled hand only |
| H-19 | A levitated front used as a handhold during shaking | DRAWER | A hand at the standoff is detected; ObstructionDegraded forces SeatAndHold with no automatic motion; intent disabled outside NORMAL | FAB-EXA-012, FAB-HAP-019 | A drawer front is not a structural support; users are told so |
| H-20 | A drawer at standoff dropped onto its travel rollers | DRAWER | Admission to ride-through needs mains (or a UPS) and E_sc ≥ 3·E_land + E_seat; every exit, including mains loss, is a log-damped seat first, then SS1 | FAB-EXA-013, FAB-HAP-019, FAB-SAF-009 | Bumper impacts in this case are reported, not bounded |
| H-21 | Shear at the plinth moat (1 t at 0.5 m/s is 125 J) | HAVEN-R (villa plinth) | Pre-centring only while the moat zone is ClearVerified; passive moat covers plus exclusion; recentring off in SHAKING | FAB-EXA-006, FAB-EXO-505, FAB-EXO-707 | Research tier (🧪); the moat edge stays a shear hazard in an earthquake |

---

## §3 Safety functions

Targets are the performance levels the design is **designed toward**. They are **not certified**, not assessed and not demonstrated on hardware (FAB-SAF-019). Simulation cannot establish a PL (§8).

| SF | Function | Implementation | Declared target (not certified) | Requirements | Status |
|---|---|---|---|---|---|
| SF-01 | Safe Landing (SS1, energised) | Supervisor FAULT_REACTION: stop at a_brk, DRAWER seat-first from the standoff, controlled descent at ≤ 5 mm/s on the supercap, then STO on the LSM; hold-up checked before lift | PL d for egress doors, WALL and HEAVY; PL c for DOOR; Class B intent for DRAWER | FAB-SAF-009, FAB-SAF-013, FAB-LEV-019 | ✅ |
| SF-02 | STO / ASC on the LSM | Monitor demand → `fabrail-rt` arbitration → LSM mode STO or ASC on the same fast job; hardware comparator trip into the TIM1 break (unverified) | PL d | FAB-SAF-008, FAB-SAF-011, FAB-SAF-016 | ✅ logic; hardware path 🗺️ |
| SF-03 | Safely limited energy and momentum at rated mass | Channel 1: governor with m_gov = max(m_rated, 1.05·m̂). Channel 2: integer kinetic-energy and momentum checks from encoder counts and m_rated | PL c unsensed, PL d sensed | FAB-SAF-001, FAB-SAF-002, FAB-SAF-012, FAB-SAF-018 | ✅ |
| SF-04 | Force limit | Directional software clamp; hardware bound K_f·I_max; monitor force and current checks on integer current codes | PL c | FAB-PRP-004, FAB-SAF-004, FAB-SAF-005 | 🔶 (EN 16005 profile on a spring-body model) |
| SF-05 | Obstruction stop and reverse | Residual r = F_motor − m̂·a − b̂·v − ripple over a persistence time; STOP, then REVERSE 50 mm (DOOR/WALL) or release to 0 N (DRAWER/HEAVY) | PL c (plus external presence sensing where full energy is used) | FAB-SAF-006 | ✅ |
| SF-06 | Presence-sensing gate | Speeds above the unsensed caps need the sealed presence copy AND a healthy sensor, in both the supervisor and the monitor; HEAVY automatic mode needs it, otherwise hold-to-run | PL d | FAB-SAF-003 | ✅ logic; the sensor is modelled only as healthy/unhealthy (A-07) |
| SF-07 | Egress manual operability | Egress openings never lock; ASC released after stop; SS1 then manual operation from stored energy; manual force ≤ 22.2 N on DOOR | PL d, stored energy | FAB-SAF-010, FAB-SEI-011, FAB-SEI-019 | ✅ |
| SF-08 | Seismic protective action | Detector, PROTECT rule, fusion and land-first class actions (§5) | **Declared not a rated safety function**: the passive guideway, the landing and the mechanics carry the safety | FAB-SEI-010 | 🔶 |
| SF-09 | Position limit (SLP) | Monitor position check from integer encoder counts against x_min/x_max; 150 mm end zones and rated bumpers as the mechanical backstop | PL c | FAB-SAF-012, FAB-SAF-014 | ✅ |

**Supporting integrity measures** (not safety functions in themselves): deadline, heartbeat and non-finite supervision to a latched FAULT (FAB-SAF-013); a CRC32 and diverse copies on the safety parameter block (FAB-SAF-015); the demand latch that only rises (FAB-SAF-016); fuzzed raw inputs without panic or NaN (FAB-SAF-017); the golden selftest hash before the watchdog starts (FAB-FW-005).

**Why levitation has no STO.** A zero-power hybrid magnet cannot be made safe by switching its coil off: below g* the permanent-magnet force exceeds the weight and an unpowered carriage snaps up; above g* it drops. The failure direction is indeterminate. The safe state for levitation is therefore an energised, monitored landing (SF-01) with a mechanical backstop in both directions, and STO acts on propulsion only (TECHNICAL_DESIGN §8.2).

---

## §4 Declared deviation: ISOLATE replaces the pedestrian envelope (FAB-SAF-020, 🧪)

**What deviates.** In the research-tier ISOLATE mode (WALL/HEAVY only) the monitor no longer applies the pedestrian directional clamp, the kinetic-energy and momentum caps or the overspeed check. An isolator must let the panel move relative to a shaking floor at speeds far above the unsensed caps (WALL 0.0446 m/s), so with the pedestrian envelope in force it would trip at once.

**What replaces it.** The checks are replaced, never disabled, by an explicit `IsolateLimits` set in the sealed safety block:

| Limit | Value | Source |
|---|---|---|
| Stroke envelope about the isolation centre | ±0.2 m, never more than the isolation stroke | `safety.isolate.stroke_m`; validate |
| Kinetic energy allowed at bumper contact | half the bumper rating (WALL 563 J rating → 281.5 J) | `safety.isolate.bumper_ke_j`; design choice (factor-2 margin) |
| Continuous LSM force | ≤ F_cont (WALL 100 N, HEAVY 500 N) | `safety.isolate.f_cont_n`; validate |

Leaving any limit raises IsolateLimit and demands STO + SS1. The monitor ignores the ISOLATE request unless every limit is configured. The isolation period uses the same F_cont: T_min = 2π·√(m·s/F_cont) = 6.3 s for WALL, so isolation largely saturates (SEISMIC §4).

**Conditions of the deviation.**

- Research tier only: `isolation.enable = false` in every preset, and no example or product text presents it as a capability.
- ISOLATE has no pedestrian-safe envelope. An installation using it would need separation from people during the event (a guarded zone), which is not designed here.
- Evidence: `rs:safety/envelope::isolate_limits_replace_clamp` and `rs:params/derive::t_min_iso_uses_f_cont` (FAB-SAF-020); the Monte Carlo gate on bumper overload (FAB-SEI-015).

---

## §5 Functions declared not safety-rated

### §5.1 The seismic action (SF-08)

The land-first class actions reduce exposure: freeze, then stop, land, lock where allowed. They depend on a detector that can miss or false-trigger, and on warning time that is zero near the source (SEISMIC.md §2). They are therefore **declared not a rated safety function**. The safety of a carriage in an earthquake rests on things that do not depend on the detector: the captive guideway, the landing, the end zones and bumpers, and the egress rules that hold whatever the detector does (FAB-SEI-011).

### §5.2 The house choreographer

The choreographer (`fabrail-exo/src/choreo`, TECHNICAL_DESIGN §15.3) plans the seismic actions of every device in a building scene. Like SF-08 it is **declared not a rated safety function**. It is bounded instead of trusted:

- every proposal passes the core `admit` filter: no protective default replaced by None, no OpenAndStop from a non-local source, no lock on an egress path;
- alert-triggered motion needs independent confirmation (a local P pick or ≥ 2 feeds), moves only toward a higher SafetyRank in the closing or seating direction, never reverses a user-initiated motion, runs at unattended limits with obstruction armed, and gets one attempt;
- a spoofed alert therefore has bounded regret: devices close at unattended low-energy limits (FAB-EXA-006, FAB-EXO-119);
- restore needs the INSPECT reset and a local command (FAB-EXA-015).

Every alert shown in the examples is labelled "synthetic/injected, not a live EEW feed".

---

## §6 FMEA-lite

Detection names are `FaultCode` variants; their severity sets the reaction (TECHNICAL_DESIGN §13).

| Component | Failure mode | Effect | Detection | Reaction | Residual |
|---|---|---|---|---|---|
| Gap sensor | Stuck, noisy, offset, out of range | Wrong levitation feedback; drift toward a stop | GapStuck, GapRange; GapDisagree with the redundant probe; monitor GapEnvelope | SS1 Safe Landing (Degrade on a single-probe disagreement) | A small common offset inside the vote tolerance shifts the float gap |
| Coil driver | Short, open, stuck-on | Loss of lift authority in one element | CoilOvercurrent, CoilOpen, CoilDriver; hardware comparator into the HRTIM fault input | SS1; the other elements land the carriage | A stuck-on driver pulls toward the upper stop until its gate is disabled |
| Coil | Shorted turns | Reduced k_i; more current for the same force | CoilI2t, SaturationPersist | SS1 | Partial shorts are seen only through I²t and saturation |
| Permanent magnet | Thermal demagnetisation | Lower bias force; m̂ rises; bias current grows | PayloadMismatch; OverTemp; demagnetisation proxy reported (FAB-ENV-004) | Hold-to-run (Degrade) or SS1 | The proxy is a simplified model (🔶) |
| MCU | Hang, panic, HardFault | Loss of all control | IWDG (≈ 5 ms), WWDG; panic and HardFault handler | Outputs off, crash record, reset; boot hold until the gaps read at rest | The carriage falls to its fail direction (H-03, H-04); fast re-catch is 🧪 |
| Current sensor | Offset, gain error | Coil: equilibrium shift absorbed by the zero-power integral. LSM: force error | CurrentSense range check; monitor checks on raw codes | SS1 on implausible codes | A gain error scales the LSM force until the clamp or monitor limits it |
| Encoder | Loss, jump, slow drift | Wrong position and velocity | EncoderLost, EncoderJump; monitor's own windowed velocity from raw counts | Controlled stop; STO + ASC on overspeed | Slow drift is corrected only by homing (FAB-PRP-011) |
| LSM phase | Open, short | Loss or distortion of thrust | LsmPhaseSum, LsmOvercurrent, LsmDriver | STO; ASC above v_runaway | A runaway carriage is stopped by the end zones and bumpers |
| DC bus | Loss, overvoltage | No drive; regenerative rise | MainsLost (2 ms debounce), BusUnder, BusOver | SS1 on the supercap; brake chopper at 1.15·V_bus | Relies on the hold-up reserve (A-05) |
| Supercap | Fade, open | Insufficient landing energy | HoldupLow at the ≥ 3× check | Lift refused; hold-to-run | Fade between checks |
| Structure accelerometer | Stuck, saturated, lost | Missed or false seismic detection | AccelStuck, AccelLost | Warn; the seismic action degrades (not rated, SF-08) | The PGA bypass still fires on clipped strong motion |
| Carriage IMU, rail accelerometer | Lost | Skyhook and inertial smoothing unavailable | ImuLost, RailAccelLost | Degrade: research modes disabled | None for the core functions |
| Network alert | Loss, spoof, false alarm | Missed warning; unwanted action | Fusion confirmation rules; cancel handling | Network-only alerts may only freeze, land or lock; a cancel returns to NORMAL | Loss leaves only local P detection (≈ 0 s warning near the source) |
| Command and alert link | Loss, corruption | Stale or corrupted commands | FDCAN frame CRC; command plausibility in the supervisor | No motion without a valid command; the carriage holds or lands on its own | Link loss removes remote commands and network alerts |
| Parameter flash | Corruption | Wrong limits or calibration | CRC32 plus diverse copies (ParamCrc); A/B pages | Refuse lift; fall back to the other page or the factory preset | A consistently wrong sealed value (A-03) |
| Core software | Wrong output (gain ×3, sign flip, NaN) | Excess force, speed or energy | Monitor energy, momentum, overspeed and force checks; NonFinite | STO / STO + ASC; SS1 | Common-cause failure of the shared MCU (A-09) |
| Presence sensor | Failure or fault | Sensed speeds without protection | Healthy input monitored by supervisor and monitor | Revert to unsensed caps; HEAVY to hold-to-run | A failure that reports healthy (A-07) |
| Rail | Debris, joint step, sag | Gap disturbance; stop contact | Gap tracking (FAB-LEV-010); GapEnvelope; obstruction residual | SS1 on the envelope | Debris is not modelled (FAB-ENV-005, 🗺️) |
| Rollers | Seized | Manual force exceeds 22.2 N after landing | None in software | Periodic inspection | Undetected until manual use (H-07) |
| Parking lock, seismic latch | Fails to engage | HEAVY not held; a drawer may open under floor acceleration | LockFailed, LatchFailed | Warn; retention reported by the scene | Retention beyond m·PFA_x is reported as "drawer opened" |

---

## §7 Argument (GSN-lite)

### §7.1 Structure

```text
 G0  Within the modelled physics and scenarios, the design keeps every hazard H-01 … H-21
     inside the numeric limits cited in §2.
  ├── C1  context: SPECIFICATION.md requirements; PHYSICS.md models; presets in fabrail-params
  ├── C2  context: standards editions of §10 (numbers only, no conformity)
  ├── A   assumptions A-01 … A-12 (§1.2)
  └── S0  strategy: argue each hazard family over channel 1 (core), channel 2 (integer monitor),
          the energy reserve and the mechanics, with closed-loop simulation through the real
          control code as the evidence
       ├── G1  energy and momentum ─────────────── H-02 H-06 H-15
       ├── G2  contact forces ──────────────────── H-01 H-13 H-18
       ├── G3  loss of levitation or power ─────── H-03 H-04 H-14 H-17 H-20
       ├── G4  egress ──────────────────────────── H-07 H-10
       ├── G5  seismic ─────────────────────────── H-08 H-09 H-19 H-21
       ├── G6  software integrity ──────────────── H-05 H-15 H-16
       ├── G7  design rules outside the control loop ── H-11 H-12
       └── G8  ◇ undeveloped: the declared targets hold on hardware (FAB-SAF-019, FAB-FW-012)
```

### §7.2 Goals and evidence

| Goal | Claim | Evidence (requirements whose tests carry it) | Assumptions |
|---|---|---|---|
| G1 | In unsensed modes kinetic energy stays ≤ 1.69 J and momentum ≤ 22.3 kg·m/s at rated mass, whatever m̂ reports | FAB-SAF-001, FAB-SAF-002, FAB-SAF-007, FAB-SAF-008, FAB-SAF-012, FAB-SAF-018 | A-03 |
| G2 | Powered contact forces stay within the force–time profile, and clamp forces through the edge seal stay ≤ 400 N | FAB-PRP-004, FAB-SAF-004, FAB-SAF-005, FAB-SAF-006, FAB-EXA-011 | A-02, A-10 |
| G3 | Every loss of control or supply ends at rest in contact, captive, and manually movable | FAB-LEV-006, FAB-LEV-019, FAB-LEV-020, FAB-SAF-009, FAB-SAF-010, FAB-SEI-019, FAB-EXA-013 | A-01, A-04, A-05, A-08 |
| G4 | Egress openings never lock, fire-rated openings never open on an alert, and a landed egress door stays manually operable | FAB-SEI-011, FAB-SAF-010, FAB-SAF-011, FAB-SEI-019 | A-04, A-12 |
| G5 | Seismic actions reduce exposure without creating a new hazard, and nothing depends on the detector for safety | FAB-SEI-005, FAB-SEI-006, FAB-SEI-009, FAB-SEI-010, FAB-SEI-013, FAB-SEI-017, FAB-EXA-006, FAB-EXA-012, FAB-EXO-119 | A-06, A-11 |
| G6 | The monitor is independent of the core, deterministic, and every detected fault ends in a latched safe state that only Reset clears | FAB-SAF-012, FAB-SAF-013, FAB-SAF-015, FAB-SAF-016, FAB-SAF-017, FAB-FW-004, FAB-FW-005, FAB-FW-006, FAB-EXO-116, FAB-HAP-009 | A-09 |
| G7 | Stray field and electrical hazards are bounded by design rules and flagged where the rule is not met | FAB-ENV-001, FAB-PWR-005, FAB-PRP-010 | — |
| G8 | ◇ The declared PL targets, finger-gap geometry, captive guideway and hardware timing hold on real hardware | FAB-SAF-019, FAB-FW-012 (roadmap, no evidence) | — |

G8 is deliberately undeveloped. Until it is developed, the argument supports only the claim in G0: behaviour **within the modelled physics and scenarios**.

---

## §8 What simulation can and cannot establish

**Simulation here can establish:**

- how the control laws behave under the modelled physics, including disturbances, parameter mismatch and sensor noise;
- whether modelled bodies and scenarios stay inside the cited numeric limits;
- the outcome of injected faults (sensor faults, core faults, supply loss, control loss);
- *estimates* of diagnostic coverage for the modelled failure modes;
- determinism: the same seed gives the same bytes, and the same group code gives the same bits in simulation, in the browser and in the firmware build (voltage coil mode).

**Simulation here cannot establish:**

- PFH_D, MTTF_D, diagnostic coverage in the ISO 13849 sense, common-cause failure resistance, or any PL or SIL;
- force measured by a standard's test method, on a real closing edge;
- EMC, acoustic levels (no dB(A) figure is given), or measured stray field;
- component reliability, wear or environmental robustness;
- systematic capability: development process, independent review, tool qualification;
- type testing, installation quality or user behaviour beyond the modelled profiles;
- firmware timing on the MCU: cycle counts, PWM-synchronous sampling, comparator latency.

---

## §9 Residual risks

1. **Payload estimate tolerance.** m̂ converges within ±5 % (FAB-LEV-013) but is tagged non-safety; the governor uses m_gov ≥ m_rated. An installer who raises m_rated defeats the energy argument (A-03).
2. **Edge seal assumed.** The 173 N clamp force for a hand holds only with a ≤ 10 N/mm seal; without it the hand clamp force at 1.69 J is 503.5 N (FAB-SAF-005).
3. **Model fidelity.** Leakage and fringing (1.2–1.5× more magnet in practice), eddy lag, rail drag and hysteresis are uncertain and swept; there is no FEM cross-check. The levitation design assumes a laminated or SMC rail.
4. **DRAWER limits.** Contactless travel ≤ 50 % extension; open hover ≈ 5 min on the supercap; fail-down above 20 kg needs a manual push to re-seat.
5. **Single MCU.** Software diversity only; a common-cause MCU failure relies on the watchdogs, the fail direction and the mechanics.
6. **Hardware overcurrent path unverified** (A-08, FAB-FW-012).
7. **Seismic warning.** Near the source the on-site warning is ≈ 0 s; the seismic action is not a rated safety function.
8. **Standards numbers.** EN 16005 numbers come from the 2011 draft; IEC 60335-2-116 drawer limits are not public and are not used.
9. **Stray field.** WALL and HEAVY need shrouds; no measurement exists.
10. **Seized rollers** are not detected by software.

---

## §10 Standards and editions

fabrail takes numbers from these documents. It claims no compliance, conformity or certification with any of them. This table fixes the editions that SPECIFICATION §4 marks "to confirm"; an edition change is a requirement change and updates SPECIFICATION §4, this table and the parameter sources together.

| Document | Edition fixed here | Numbers or concepts taken | Requirements |
|---|---|---|---|
| EN 16005 | numeric limits from the 2011 draft | Force–time profile 400 N ≤ 0.75 s, 150 N, < 80 N within 5 s; 67 N low-energy stall | FAB-PRP-004, FAB-SAF-004 |
| ANSI/BHMA A156.10 | 2005 | Full-energy automatic door context; HEAVY hold-to-run is outside it | FAB-SAF-002 |
| ANSI/BHMA A156.19 | 2019 (fixed here) | Low-energy and power-assist limits: 15 lbf (67 N) stall force; the 1.25 lbf·ft (1.69 J) kinetic-energy basis | FAB-SAF-002, FAB-PRP-004 |
| ANSI/BHMA A156.38 | 2019 | Low-energy sliding doors: the 1.69 J kinetic-energy basis | FAB-SAF-002 |
| ISO/TS 15066 | 2016 | Body-region stiffness and effective mass (hand 75 N/mm, 0.6 kg; chest 25 N/mm, 40 kg) | FAB-SAF-005 |
| 2010 ADA Standards §404.2.9 | 2010 | 5 lbf (22.2 N) operating force | FAB-SAF-010, FAB-HAP-003 |
| IBC §1010.3 | 2021 (fixed here) | Egress doors manually operable and never locked by the seismic action; manual forces of §1010.3.2 (power-operated) and §1010.3.3 (horizontal sliding) | FAB-SEI-011, FAB-SEI-019 |
| NFPA 80 | 2022 (fixed here) | Fire doors are not opened by the seismic action (a rule, no number) | FAB-SEI-011 |
| ISO 13849-1 | 2023 (fixed here) | PL c/d as declared design targets; "Cat 3 intent" wording only | FAB-SAF-012, FAB-SAF-019 |
| IEC 61800-5-2 | 2016 (fixed here) | Definitions of STO, SS1, SLS and SLP; overspeed monitoring | FAB-SAF-008, FAB-SAF-013 |
| IEC 60730-1 | 2013 with amendments 1 (2015) and 2 (2020) (fixed here) | Class B-style CRC and diverse copies of safety data (intent) | FAB-SAF-015 |
| ASCE 7-16 | 2016 | Design spectrum S_DS 1.0, S_D1 0.6, T_L 8 s; floor amplification Eq. 13.3-1 | FAB-SEI-018, FAB-SEI-020 |
| IEC 60335-2-116 | — | Drawer-mover limits are not public; not used | — |
| UL 325 | — | Not used | — |

Sovereign-tier documents (IEC 61508, IEC 61784-3, ISO 13855, ISO 14119, ISO 12100) are listed with their editions in EXOTIC.md.

---

## §11 Non-claims

This safety case does **not** claim:

1. certification, conformity or compliance with any standard listed in §10, or any achieved PL, SIL, PFH_D or MTTF_D;
2. that the firmware has run on hardware, meets its cycle, memory or latency budgets on the MCU, or that the hardware overcurrent path works;
3. that haptic or rendered lightness reduces impact energy;
4. that heavy panels move fast without PL d presence sensing (HEAVY unsensed is hold-to-run at 0.025 m/s);
5. passive levitation without control;
6. measured stray-field levels, or WALL/HEAVY meeting 0.5 mT without shrouds;
7. any dB(A) figure or FEM-validated force;
8. that the seismic action or the house choreographer is a safety function, or that any installation is made safe against earthquakes;
9. that isolation, active mass damping or fast re-catch are product features (all 🧪);
10. "zero friction": the modelled equivalent friction is μ_eq ≈ 1e-3–1e-2.
