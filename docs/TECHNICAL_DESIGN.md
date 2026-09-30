# fabrail technical design

This document describes how fabrail is built: the crates and why each exists, how data and control flow through them, the state machines, the timing model, the numeric rules, the hardware boundary, the safety architecture, the firmware target and its honest verification status, and the layers added by the examples, the Sovereign tier and the browser engine.

It is the architecture companion of [SPECIFICATION.md](SPECIFICATION.md) (what the system shall do), [PHYSICS.md](PHYSICS.md) (the model and control laws exactly as implemented), [SAFETY.md](SAFETY.md) (hazards, safety functions and the argument), [SEISMIC.md](SEISMIC.md) (the three seismic readings) and [EXOTIC.md](EXOTIC.md) (the Sovereign tier). Code cites sections of this document as `TECHNICAL_DESIGN §n`; the section numbers are stable.

> **Scope, stated honestly.** Everything here is verified in simulation or by inspection, never on hardware. The firmware builds and links for the STM32G474 and has never been flashed. "Zero friction" is a direction, not a claim: the modelled equivalent friction is μ_eq ≈ 1e-3–1e-2 (FAB-HAP-010). Nothing in this repository is certified or claims compliance with any standard.

Status badges follow SPECIFICATION §0.5: ✅ verified in simulation, 🔶 simplified model, 🧪 research tier, 🗺️ roadmap, 🔭 frontier.

---

## §1 Architecture

### §1.1 Parts

fabrail is one Cargo workspace of 13 crates, a separate firmware workspace, a Python package and a static documentation site.

| Part | Kind | Role | Depends on |
|---|---|---|---|
| `fabrail-types` | no_std | Units (`Tick`, `PosNm`, `PosAccum`), `fmath` (the only math entry point: libm with `force-soft-floats`), raw HAL samples, commands, state enums, fault codes, network alerts, trajectory segments, the telemetry frame | libm |
| `fabrail-params` | no_std | `SystemParams {design, control, safety}`: presets (the single source of every class value), `derive` (closed forms and the discrete gain design), `validate`, `meta` (one `FieldMeta` per leaf), `crc` (CRC32 and diverse copies) | types |
| `fabrail-hal` | no_std | Hardware traits only: the portability contract between the controller and a board | types |
| `fabrail-core` | no_std, f32 | The function channel: frontend, current and FOC loops, levitation, motion, haptics, seismic, supervisor, telemetry; the step API in `msg.rs` | types, params |
| `fabrail-safety` | no_std, integer | The monitor channel: velocity, energy and momentum, force and current, gap and coil envelopes, position, heartbeat, supply | types, params |
| `fabrail-rt` | no_std | The executive the MCU runs: LET mailboxes, the four rate groups, safety arbitration, the golden selftest hash | types, params, hal, core, safety |
| `fabrail-plant` | std, f64 | Truth physics: magnets, coils, LSM, rigid bodies, contacts, friction, air, thermal, rail, structure; the only crate allowed nalgebra | types, params |
| `fabrail-sim` | std | Scenarios, SimBoard (sensor models, quantisation, in-repo RNG), events, fault injection, recorder, metrics, batch runs (rayon behind the `parallel` feature), replay, `DetectorReplay`, the steppable `Session` | types, params, hal, core, safety, rt, plant |
| `fabrail-py` | cdylib `_native` | PyO3 bindings; arrays cross as little-endian `bytes` | types, params, rt, plant, sim, exo-sim |
| `fabrail-exo-safety` | no_std, integer | Sovereign-tier plain data (`ExoClass`, `SafetyRegime`, `SafetyRank`, `FusedAlert`) and integer safety logic: 2oo3 voter, degradation ladder, black channel, zones, envelope, alert gate, PFH | types, params |
| `fabrail-exo` | no_std, f32 | Sovereign-tier controllers and estimators (HMD, isolation skyhook, energy admission, closure planner, pod allocation, EEW fusion) and the house choreographer | types, params, core, exo-safety |
| `fabrail-exo-sim` | std, f64 | Sovereign-tier truth models (suspension kinds, storage, host structures, isolation platform, ground-motion additions, EEW and tsunami timing) and the multi-device building scene | types, params, plant, sim, exo, exo-safety |
| `fabrail-web` | wasm32 cdylib | The browser engine: the real controller and plant behind hand-written `extern "C"` exports | types, params, core, rt, plant, sim |
| `firmware/` (`fabrail-fw`) | separate workspace | RTIC 2 application for the STM32G474RE; path dependencies on the six no_std core crates | types, params, hal, core, safety, rt |
| `python/fabrail` | Python | CLI, presets, scenario synthesis, ground motion, P+S records, human models, rail profiles, ensembles, analysis, plots, export, traceability, studies, the Sovereign layer, the example gallery | `_native` |
| `docs/` | static site | Pages, generated data blocks, the wasm engine, and these documents | — |

### §1.2 Crate graph

The direct dependencies are in the table above. Layers are the longest dependency chain from `fabrail-types`; a crate depends only on crates in lower layers.

```text
 layer 0   fabrail-types
 layer 1   fabrail-params      fabrail-hal
 layer 2   fabrail-core        fabrail-safety      fabrail-plant       fabrail-exo-safety
 layer 3   fabrail-rt          fabrail-exo
 layer 4   fabrail-sim
 layer 5   fabrail-exo-sim     fabrail-web (wasm32)
 layer 6   fabrail-py (_native)
 separate  firmware/fabrail-fw  (path deps on types, params, hal, core, safety, rt; never a host member)
```

The two safety-relevant channels meet in exactly one place:

```text
 fabrail-types ──► fabrail-params ──┬──► fabrail-core ────┐
                                    │   (function, f32)   ├──► fabrail-rt ──► fabrail-sim / firmware / fabrail-web
                                    └──► fabrail-safety ──┘    (arbitration: the monitor's
                                        (monitor, integer)      demand can only raise)
```

**Dependency rules.**

- `fabrail-safety` and `fabrail-exo-safety` depend on `fabrail-types` and `fabrail-params` only, so a defect in the control code cannot also blind the monitor through a shared dependency. A `cargo metadata` closure test enforces this transitively (FAB-EXO-003, FAB-EXO-116).
- `fabrail-core` never depends on `fabrail-safety`. The safety verdict type lives in `fabrail-types` so the core can read it without linking the monitor.
- The no_std crates (types, params, hal, core, safety, rt, exo-safety, exo) are `#![cfg_attr(not(test), no_std)]` and `#![forbid(unsafe_code)]`, use no heap, route every transcendental function through `fmath`, and spell out every dependency without `workspace = true`, so the firmware workspace can consume them by path (FAB-FW-002, FAB-EXO-902).
- nalgebra is allowed in `fabrail-plant` only. A std `num-traits` pulled in by feature unification would otherwise route host controller math through a different libm than the MCU.
- The `fault-injection` feature of `fabrail-rt` is enabled only by `fabrail-sim` (and the exotic test tooling); it never appears in the firmware manifest or lockfile (FAB-EXO-307).
- `fabrail-py` and `fabrail-web` are workspace members but not default members, so `cargo test` never links libpython or builds the wasm cdylib.

### §1.3 Why the firmware is a separate workspace

1. Cargo resolves one lockfile per workspace, including target-gated dependencies. As a member, the firmware would force every host build to resolve RTIC, the PAC and the HAL.
2. The firmware's `build.target` and its `panic = "abort"`, fat-LTO profile must not apply to host crates.
3. Its features (serde on params for the flash pages) must not unify into host builds.

The root `Cargo.toml` therefore has `exclude = ["firmware"]`, and `firmware/` has its own `[workspace]` and `Cargo.lock`.

---

## §2 Data flow

### §2.1 A simulation run

```text
fabrail_params::presets ──► validate() ──► Params            (Python: Params.preset / with_overrides; reseal)
        │
        ▼
python/fabrail: scenarios, ground, pwave, rail, human        numpy SeedSequence(seed).spawn(2):
        │                                                     child 0 → synthesis; child 1 → Rust noise_seed
        │  ScenarioSpec JSON (fabrail.scenario/1) + series bytes (channel-major little-endian f64)
        ▼
_native.run ──► fabrail_sim::run
                  check_compat (coil mode single source, rest hardware, travel, truth mass)
                  plant::build(tier)  +  SimBoard (sensor models, quantisation, RNG substreams)  +  Executive
                  per base tick k:
                    events due at k → Executive::on_base_tick(&mut SimBoard) → plant.step(RK4)
                    → metrics → recorder (at the record decimation)
        │
        ▼
RunData { channels (LE bytes, dtype, unit), events, kpis, meta (hashed JSON without wall_s) }
        │
        ▼
results.RunResult ──► analysis ──► visualization (PNG) · export (canonical JSON) · docs data blocks (RUN)
```

- The coil mode (lag or voltage) has one source, the scenario; a parameter set that disagrees is rejected (FAB-SW-015).
- Randomness: numpy owns scenario synthesis (ground motion, P+S records, rail profiles, intent jitter). Rust owns sensor noise and fault injection with in-repo SplitMix64 seeding and Xoshiro256++ streams, one named substream per (tag, device, channel). Adding a stream never perturbs another (PHYSICS §24).
- `run_batch` returns results in index order, so ensembles are identical for any worker count (FAB-SW-002).
- The same seed gives byte-identical JSON in process and in a subprocess; `wall_s` is excluded from the hashed record (FAB-SW-002).

### §2.2 Steppable sessions, scenes and the browser

`fabrail_sim::Session` advances one device to a target tick under an externally supplied floor acceleration and command, optionally through an `ActionPolicy` (`step_to_with_policy`). Three consumers use it:

| Consumer | Where | How |
|---|---|---|
| Building scene (`villa_quake`) | `fabrail-exo-sim::scene` | Building model → floor motion per room (one-way coupling); devices stepped in 200-tick chunks at 100 Hz sync, in registry order, rayon across devices; bus power summed in index order |
| Browser engine | `fabrail-web` | One device per engine, stepped single-threaded by the page (§17) |
| Sovereign scenarios | `fabrail-exo-sim::scenarios` | Host and platform models around core devices |

### §2.3 Python layer

| Module | Role |
|---|---|
| `presets`, `config` | Class factories over native `Params`; `RunConfig`; `parameters_markdown()` for PARAMETERS.md |
| `scenarios`, `ground`, `pwave`, `rail`, `human`, `records` | Scenario builders; Kanai–Tajimi/Clough–Penzien synthesis with the JHT envelope; Boore P+S records scaled to Wu–Zhao; rail irregularity; ISO/TS 15066 bodies and hand profiles; the El Centro fetch and its surrogate (PHYSICS §19, §22) |
| `ensemble`, `results`, `analysis`, `visualization`, `export`, `plotstyle` | Seeds, typed results, analysis dataclasses, figures, canonical JSON, the shared palette |
| `trace` | The bidirectional traceability guard (FAB-SW-007) |
| `studies/` | The tier-1 and tier-2 studies |
| `exotic/`, `exotic/frontier/` | The Sovereign layer; the numpy-only 🔭 frontier models |
| `gallery/` | The 16-example registry and CLI |

---

## §3 Control cascade

One carriage, one combined node (v0.1). Rates are for DRAWER/DOOR; WALL/HEAVY run the fast group at 5 kHz.

```text
 command link ─► ┌──────────────────── supervisor group (1 kHz) ────────────────────┐ ◄── seismic group (100 Hz)
 power, E-stop,  │ LevState table · ModePlanner (segments, live limits, haptic      │     ADXL355 500 Hz → ÷5 → 0.5–20 Hz BP
 presence,       │ profile) · fault manager · hold-up check · class action +        │     → STA/LTA → Pd/τc → level
 hold-to-run ──► │ ActionPolicy admission · telemetry · watchdog feed               │     → PROTECT rule → fusion → S-ETA
                 └───────────────────────────────┬──────────────────────────────────┘
                                                 │ Directive (activates at r + 20 ticks)
 FastRaw (latched by base) ──────────────────────▼──────────────────────────────────────────────────────
 ┌──────────────────────────────── fast group (10 kHz, pure function of FastRaw, FastIn) ──────────────┐
 │ frontend: gap polynomial + dual-probe vote, current codes, PosNm from counts, Clarke/Park, IMU        │
 │                                                                                                      │
 │ LEVITATION  gaps ─► modal decomposition ─► zero-power law per active mode ─► allocation + warp ─►   │──► coil i_ref[n]
 │             (heave, pitch; roll on DRAWER/HEAVY)   i* = k_p·δg + k_d·δġ + κ·q,  q̇ = i − i_bias       │
 │                                                                                                      │
 │ MOTION      encoder ─► tracking observer (v̂, â) ─► momentum observer F̂_h ─► intent / hand           │
 │             reference: S-curve │ terminus │ quintic │ admittance (detents, soft-close, passivity)    │
 │             ─► governor v_lim(x, m_gov) ─► velocity cascade ─► directional clamp ─► i_q* = F/K̂_f     │──► LSM i_q*
 │             obstruction residual ─► STOP ─► REVERSE 50 mm (DOOR/WALL) or release to 0 N              │
 │                                                                                                      │
 │ MONITOR     fabrail-safety step_fast on RAW samples (integer) ─► SafetyVerdict                       │
 │ ARBITRATION |i_q| ≤ min(core, cap)/K_f(copy) · STO/ASC on demand · demand latch (never lowered)       │
 └──────────────────────────────────────────────────┬───────────────────────────────────────────────────┘
                                                    │ FastOut (activates at r + 2 ticks)
 ┌──────────────────────────────── base task (20 kHz) ▼─────────────────────────────────────────────────┐
 │ coil current PI ×n (voltage coil mode) ─► H-bridges     LSM d/q PI (inverter driver) ─► 3-phase stage │
 │ samples currents at the PWM peak; latches FastRaw at fast releases; commands apply at k + 1 (ZOH)    │
 └──────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

| Loop | Law | Design and reference |
|---|---|---|
| Coil current | PI, K_p = L̂·ω_c, K_i = R̂·ω_c, ω_c = 2π·1.0 kHz, back-calculation anti-windup, motional-EMF feed-forward; I²t clamp (i_cont, i_pulse for t_pulse) | FAB-LEV-008; PHYSICS §4 |
| Levitation, per active mode | Mizuno zero-power current-integral law; target polynomial (s² + 2ζωs + ω²)(s + αω) with ω = 2.5·max(p_mode, p_heave), placed by a discrete design on the ZOH plant with the LET delay, the current-loop lag τ_cl and the nominal eddy lag τ_e | FAB-LEV-007, FAB-LEV-020; PHYSICS §15 |
| Allocation | A = W⁻¹Tᵀ(TW⁻¹Tᵀ)⁻¹ over the controlled set; null-space (warp) currents integrated to zero; modified zero-power band clamp outside the float band | FAB-LEV-009, FAB-LEV-018 |
| Velocity | K_v = m̂·ω_v, K_vi = K_v·ω_v/4, ω_v = 2π×{8, 6, 3, 2} Hz; feed-forward m̂a + b̂v + ripple table; integrator frozen (bumpless) during terminus capture | FAB-PRP-007, FAB-HAP-015; PHYSICS §16 |
| Governor | v ≤ min(V_mode, √(2E/m_gov), p_max/m_gov, √(2·a_brk·(x_obs − x − margin))) − a_brk·t_react, m_gov = max(m_rated, 1.05·m̂), a_brk = 2·a_max, t_react = 2 fast periods | FAB-SAF-001, FAB-SAF-007 |
| Directional clamp | Driving force ≤ F_drive_max for ≤ 0.75 s; stall ≤ 67 N unsensed (150 N sensed or in the last 50 mm); < 80 N within 5 s; braking never clamped | FAB-PRP-004 |
| Admittance | M_v·ẍ + B_v·ẋ + K_v(x − x0) = F̂_h + F_eff; K_v ≤ 2b/(T + 2T_d) with the passivity observer and controller | FAB-HAP-001, FAB-HAP-009; PHYSICS §17 |

The allocation, gains and every threshold are derived in `new()` from `fabrail-params`; nothing in the fast path is a literal.

---

## §4 State machines

The tables in code are authoritative; the diagrams below summarise them.

### §4.1 LevState (supervisor, `supervisor/levstate.rs`)

```text
 main path
   BOOT ─► SELFTEST ─► rest state by RestPolicy:
             SEATED (DRAWER)  ·  LIFTING ─► LEVITATED_IDLE (DOOR)  ·  LANDED (WALL, HEAVY)

   SEATED / LANDED ──lift or move request──► LIFTING ──levitated──► LEVITATED_IDLE ⇄ MOVING
   MOVING ──arrived──► SEATING (DRAWER at the seat)  ·  LANDING (WALL, HEAVY at park)  ·  LEVITATED_IDLE
   MOVING ──stop or land request──► STOPPING ──stopped──► SEATING  ·  LANDING  ·  LEVITATED_IDLE  ·  MOVING
   SEATING ──seated──► SEATED                    LANDING ──touchdown──► LANDED

 side branches
   LEVITATED_IDLE / MOVING / STOPPING ──PROTECT with a class action──► SEISMIC ──touchdown or seated──► LANDED / SEATED
   LEVITATED_IDLE / MOVING / STOPPING ──escape trigger──► ESCAPE (open to the travel end) ──touchdown──► LANDED (never locked)
   SEATED / LANDED / FAULT ──manual request──► MANUAL (unpowered on rollers, moved by hand) ──Reset──► SELFTEST
   LANDED ──DRAWER pushed home by hand, re-seated by the ramp (F17)──► SEATED

 fault path
   SELFTEST ──CRC or selftest-hash failure (refuse to lift)──► FAULT_REACTION
   any ──demand ≥ SafeStop──► FAULT_REACTION ──touchdown, seated or timeout──► FAULT (latched) ──Reset, valid CRC──► SELFTEST
   FAULT_REACTION = stop at a_brk → DRAWER seat-first from the standoff when only the supply failed
                    → SS1 landing on the supercap; STO on the LSM only
```

A state completes only on a fast status produced under that state's own directive, so the LET delay can never close a state early. Tested by `transition_table`, `fault_latch` and `refuse_lift_on_crc` (FAB-SAF-013, FAB-SAF-015).

### §4.2 SeismicState (seismic group, `seismic/fusion.rs`)

```text
 NORMAL ────pick or strong motion────► TRIGGERED (FREEZE)
 NORMAL, TRIGGERED ──PROTECT rule──► PROTECT (class action)
 TRIGGERED ──no longer active──► NORMAL            (level ≤ 1 events self-clear: freeze only)
 PROTECT ──shaking──► SHAKING          PROTECT, SHAKING ──quiet──► QUIET
 QUIET ──active again──► SHAKING       QUIET ──30 s without triggers──► INSPECT
 INSPECT ──manual inspect reset only──► NORMAL
 any ──cancel──► NORMAL

 PROTECT rule: Level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal
```

Exhaustive under generated event sequences (FAB-SEI-008). The detector and the class actions are described in [SEISMIC.md](SEISMIC.md).

### §4.3 MotionMode (supervisor → fast group)

| Mode | Meaning | Selected when |
|---|---|---|
| Off | Propulsion off (STO) | Rest, fault, landed |
| Hold | Position hold | Levitated idle |
| Trajectory | S-curve tracking | Commanded move |
| Admittance | Hand-driven rendering | Intent confirmed, push-to-open |
| SoftClose | Legacy soft-close spring (the looser FAB-HAP-007 bound) | Presets without a terminus |
| Terminus / SoftOpen | Log-damped approach to a close/seat or an open target (PHYSICS §16.1) | End of every commanded close or open |
| Reverse | Obstruction reversal, 50 mm | DOOR/WALL obstruction |
| Freeze | Decelerate at a_brk and hold | TRIGGERED and the start of PROTECT |
| Center | Centring, only if reachable | WALL/HEAVY class action (F11) |
| Isolate, Damp | 🧪 contactless isolation, active mass damping | Research tier, disabled in every preset |
| InertialSmooth | 🧪 follow only low-frequency floor motion | Research tier, `inertial.enable = false` |

### §4.4 Safety demand ladder (monitor → arbitration)

```text
 None < LimitForce < SafeStop < SafeLand < StoLsm < StoLsmAsc

 LimitForce  cap the LSM force at verdict.force_cap_n
 SafeStop    controlled stop, then FAULT_REACTION at the next supervisor release
 SafeLand    SS1 Safe Landing
 StoLsm      STO on the LSM (+ Safe Landing)
 StoLsmAsc   STO + active short circuit on the LSM (+ Safe Landing); ASC only above v_runaway
             or in a seismic landing, released to STO at standstill (FAB-SAF-011)
```

Demands combine by maximum. The demand level can only rise within a latch window of two supervisor periods, and the core can never lower it (FAB-SAF-016). STO-level demands latch in the monitor until the supervisor leaves FAULT after a Reset.

---

## §5 Timing and LET

### §5.1 Rate groups

| Group | DRAWER/DOOR | WALL/HEAVY | Contents | HAL access |
|---|---|---|---|---|
| base | 20 kHz (50 µs) | 20 kHz | Coil current PI ×n (voltage coil mode) or pass-through (lag mode, simulation only); LSM d/q command to the inverter; latches `FastRaw` at fast releases | Coils, LSM, gap, encoder, IMU, rail accelerometer |
| fast | 10 kHz (decimation 2) | 5 kHz (decimation 4) | Frontend, levitation, motion and haptics, governor and clamp, monitor `step_fast`, arbitration; a pure function of (FastRaw, FastIn) | none |
| supervisor | 1 kHz (decimation 20) | 1 kHz | LevState, modes and segments, intent confirmation, faults, class action, telemetry, watchdog feed | Power, safety inputs, aux, command link, watchdog, telemetry |
| seismic | 100 Hz (decimation 200) | 100 Hz | Drains ≤ 16 ADXL355 samples, detector, discrimination, fusion, floor-velocity estimate | Structure accelerometer, alert link |

The Travel1d tier (the seismic Monte Carlo) runs the plant at 1 ms with levitation assumed ideal (PHYSICS §5).

### §5.2 The LET contract

- A job released at base tick r reads mailbox values whose activation tick is ≤ r and publishes values that activate at r + P, P being its period in base ticks.
- The base task always runs first within a tick: it latches the `FastRaw` that a fast release at the same tick consumes.
- Because nothing published at r is visible before r + P, the groups due at the same tick may run in any order. The test runs all 3! = 6 orders of fast, supervisor and seismic and requires one identical output hash (FAB-FW-004).
- A PWM duty computed at tick k applies at k + 1 (zero-order hold).

This makes RTIC preemption order irrelevant to the outputs, which is what lets the simulator and the firmware run the same group code (§9.4).

### §5.3 Latency chain

| Path | Delay | Consequence |
|---|---|---|
| Gap sample → coil voltage | ≤ 1 fast period + 1 base tick: 150 µs (DRAWER/DOOR), 250 µs (WALL/HEAVY) | Included in the discrete levitation design (FAB-LEV-007); at the DRAWER heave target ω = 2.5·p = 174 rad/s the delay costs 174 × 150 µs = 0.026 rad ≈ 1.5° |
| Supervisor directive → fast group | 20 base ticks (1 ms) | Segments are planned for the activation tick |
| Accelerometer sample → freeze command | ≤ 10 ms (samples of a release are stamped with the release tick) + 10 ms (seismic LET) + ≤ 2 ms (supervisor) + ≤ 0.2 ms (fast) ≈ 22 ms | Inside the 100 ms pick-to-FREEZE budget (FAB-SEI-009) |
| Hand force → admittance onset | ≤ 10 ms (DRAWER/DOOR), ≤ 20 ms (WALL/HEAVY) | FAB-HAP-005; intent confirmation ≤ 120 / 170 ms |

### §5.4 Overruns and the watchdog

Each group checks that its release arrives within one period of the previous one. Firmware measures overruns with the DWT cycle counter and reports them through `Executive::report_overrun`.

| Miss | Fault | Severity |
|---|---|---|
| Fast job | FastOverrun | SS1 Safe Landing, sticky for the demand-latch window |
| Supervisor job | SupervisorOverrun | SS1 Safe Landing; the watchdog feed is withheld |
| Seismic job | SeismicOverrun | Warn (the seismic action is not a rated safety function) |
| Core heartbeat stale for 3 supervisor periods | HeartbeatLost | STO |
| Watchdog reset | WatchdogReset | SafeLand severity; the firmware reboots into a hold with outputs off (§9.4) |

The supervisor feeds the independent watchdog (≈ 5 ms) only when every group's sequence number advanced within its deadline and no overrun was flagged (FAB-SAF-013).

---

## §6 Numeric representation

| Quantity | Representation | Why |
|---|---|---|
| Controller values (core, rt, exo) | f32 SI | The Cortex-M4F FPU is single precision |
| Plant, `params::derive`, parameter fields | f64 SI, unit-suffixed names (`_m`, `_s`, `_n`, `_kg`, `_a`) | The 1e-12 analytic anchors; controllers cast to f32 once in `new()` |
| Monitor per-tick arithmetic | Integers only: grams, µm/s, femtojoules of 2·KE (u128, exact at 10 m/s), g·µm/s, encoder counts, current codes², base ticks | Diversity from the f32 core; `#![deny(clippy::float_arithmetic)]` in every check module; floats appear only in `Monitor::new` (FAB-SAF-012) |
| Absolute position | `PosNm(i64)` nanometres; encoder counts × nm/count exactly; differences in integers, then cast | Between 8 and 16 m, adjacent f32 values are 0.95 µm apart, which would put a 19 % per-step error on a 2.5 µm admittance step |
| Trajectory and admittance integration | `PosAccum {base: PosNm, frac_nm: f32}`; S-curves closed form from an anchor `PosNm` | No drift over 15 m (FAB-PRP-008) |
| Electrical angle | `x_nm.rem_euclid(2·τ_p)` | Exact at any distance (FAB-PRP-009) |
| Time | `Tick(u64)` at 20 kHz; plant time `tick·h` computed, never accumulated | No absolute f32 time anywhere |
| Math | `fabrail_types::fmath` only (libm, `force-soft-floats`); no `mul_add`, `num_traits`, float methods or `cfg(target_arch)` in no_std crates | The same bits on host, wasm32 and thumbv7em |
| Exotic safety integers | kg (u32), mm/s (i32), 2·KE in µJ (u64), kg·mm/s (u64), N (i64); no Q16.16 | Range 1 kg–1e8 kg without overflow (FAB-EXO-112) |
| Exotic energy stores | f64 through the `EnergyStore` trait | 1e3 J to 3.75e9 J at planning rate; i64 mJ accumulators on a target |

Rust performs no floating-point contraction and has no fast-math mode, so with one math implementation and no target-specific code the controller's outputs are bit-identical on every target. The boot selftest asserts FPSCR FZ = DN = 0 so flush-to-zero cannot silently change them (§9.4).

---

## §7 HAL boundary

The controller owns no peripheral. `fabrail-hal` defines small traits, and each rate group takes only the traits it needs:

| Group trait | Traits | Implemented by |
|---|---|---|
| `BaseHal` | `CoilDrivers`, `LsmDriver`, `GapSensors`, `Encoder`, `CarriageImu`, `RailAccel` | `SimBoard` (simulation), `BaseBoard` (firmware), the selftest LCG board |
| `SupervisorHal` | `PowerMonitor`, `SafetyInputs`, `Aux`, `CommandLink`, `Watchdog`, `Telemetry` | `SimBoard`, `SupBoard` |
| `SeismicHal` | `Accelerometer` (FIFO drain ≤ 16 samples), `AlertLink` | `SimBoard`, `SeisBoard` |
| `Board` | all of the above plus `Clock`, `ParamStore` | `SimBoard`, the replay board |

- **Raw samples cross the boundary**: ADC codes, encoder counts, current codes, accelerometer codes. `core::frontend` converts them to SI with the calibration from `fabrail-params`. The simulator quantises f64 truth into the same integer samples, with noise, bandwidth and one tick of latency (PHYSICS §13), so the controller sees the kind of bits an MCU would.
- **The monitor reads raw samples too**, with its own diverse calibration copies, never the core's converted values.
- **The base task is the only group that samples hardware at 20 kHz.** The fast group is a pure function of the latched `FastRaw` and its mailbox inputs; it has no HAL access.
- **The firmware boards do register work only.** Scaling constants come from `fabrail-params` and every decision stays in the shared crates.

---

## §8 Safety architecture

The full hazard analysis, safety functions and argument are in [SAFETY.md](SAFETY.md). This section describes the mechanism.

### §8.1 Layers

```text
 layer 5  MECHANICS        upper stop as a sliding surface (F/(m*·g) ≤ 1.30), rollers (μ 0.003), captive guideway,
                           DRAWER captive seat, bottom guide fin, 150 mm end zones, rated bumpers, edge seal ≤ 10 N/mm
 layer 4  HARDWARE         COMP/DAC overcurrent → TIM1 break / HRTIM fault inputs (< 1 µs, unverified on hardware),
                           gate enables pulled to "disabled" (any reset means STO), independent + window watchdogs
 layer 3  CHANNEL 2        fabrail-safety monitor: integer checks on raw samples with sealed diverse copies
                           → SafetyVerdict → arbitration → STO / ASC on the LSM
 layer 2  CHANNEL 1        fabrail-core: governor with m_gov, directional clamp, obstruction response, supervisor,
                           SS1 sequencing, PayloadMismatch, presence and hold-to-run gates
 layer 1  ENERGY RESERVE   supercap hold-up ≥ 3× the landing energy, checked before lift (HoldupLow)
```

- **Channel 1** is the function: it keeps every motion inside the pedestrian caps (1.69 J, 22.3 kg·m/s, 67 N static unsensed) by construction (FAB-SAF-001, FAB-SAF-002, FAB-PRP-004).
- **Channel 2** is the monitor. It shares no code with channel 1, reads raw samples, derives integer thresholds from the sealed diverse copies (`k_f`, the two current scales, `nm_per_count`, the presence fitting) and demands STO or STO + ASC on the LSM when velocity, energy, momentum, force, current, position or the core heartbeat leave their envelopes (FAB-SAF-008, FAB-SAF-012). An injected core fault (i_q gain ×3, a gap sign flip, NaN) is caught (FAB-SAF-012).
- **"Cat 3 intent" only.** Both channels run on one MCU. This is single-MCU software diversity, not hardware redundancy. A diverse second MCU running `fabrail-safety` on thumbv6m is roadmap (the crate already cross-builds for it; FAB-FW-013).
- **Arbitration** in `fabrail-rt`: |i_q| ≤ min(core limit, verdict cap)/K_f using the monitor's copy of K_f; an STO demand sets STO or ASC on the same job; the core can never lower a demand (FAB-SAF-016).

### §8.2 Why STO acts on the LSM only, and landing is energised

Zero-power hybrid EMS cannot be made safe by switching the coils off. The permanent magnets carry the weight at g*, and at any gap below g* their force exceeds the weight: an unpowered carriage snaps up to the upper stop, and above g* it drops. The failure direction is indeterminate. So:

- **STO and ASC act on propulsion only.** Removing LSM torque is a safe state for travel.
- **Levitation's safe state is SS1 Safe Landing**: a controlled, energised descent at ≤ 5 mm/s from the supercap, then STO on the LSM (FAB-SAF-009). A DRAWER at its open standoff seats first (a log-damped close to the seat on the supercap), then completes SS1.
- **Both stops are designed as backstops.** The upper stop is a sliding surface loaded to at most 1.30·m*·g (DOOR/WALL/HEAVY) so a door stuck up still slides by hand; the stay-landed rule F_pm(g_land)/(m_op_min·g) ≤ 0.90 keeps a landed carriage down (FAB-LEV-006). A control loss at g* ends at rest on rollers or the seat in either direction (FAB-LEV-019).
- **DRAWER fail-down** onto the 6.5 mm travel rollers is re-liftable only up to 20 kg (i_pulse 12 A); above that a manual push re-seats it (FAB-LEV-019).

### §8.3 ISOLATE is a declared deviation

In the research-tier ISOLATE mode the pedestrian clamp and energy checks would stop the isolator at once. The monitor therefore **replaces** them, never disables them, with an explicit `IsolateLimits` set: stroke envelope, bumper kinetic energy and continuous force F_cont. Leaving the set demands STO + SS1 (FAB-SAF-020). SAFETY.md declares this as an R-tier deviation; isolation is disabled in every preset.

---

## §9 Firmware target

### §9.1 Target and crates

| Item | Choice |
|---|---|
| MCU | STM32G474RE (Cortex-M4F, 170 MHz, 512 KB flash, 128 KB SRAM including CCM, HRTIM, 5 ADCs, 3 FDCAN, COMP/DAC), NUCLEO-G474RE as the bring-up kit |
| Target | `thumbv7em-none-eabihf` |
| Scheduler | RTIC 2.3.1 (`thumbv7-backend`), rtic-monotonics 2.2.1 (SysTick) |
| HAL and PAC | stm32g4xx-hal 0.1.0 (`stm32g474`, `hrtim`, `can`), stm32g4 0.16 through the HAL's re-export; injected ADC sequences at PAC level because the HAL has no injected-group support |
| Runtime | cortex-m 0.7.9 (`critical-section-single-core`), cortex-m-rt 0.7.7, postcard 1.1.3 for the parameter pages |
| Development only | defmt 1.1.1, defmt-rtt 1.3.0, panic-probe 1.0.0 behind the `dev` feature |
| Profile | Release: opt-level `"s"`, fat LTO, codegen-units 1, `panic = "abort"`. A `speed` profile at opt-level 3 (the core design's original choice, ≈ 400 KB) is kept for timing experiments and is not size-checked |
| Size budget | Flash ≤ 384 KiB (75 % of the device; the top 8 KiB hold the parameter pages, enforced by a `memory.x` assertion), static RAM ≤ 64 KiB (the rest of the 128 KiB is stack); measured per class image by `scripts/check_firmware.sh`. Most of the flash is init-time code that runs once: the f64 soft-float gain design, the parameter validation table and the postcard decoder |
| Class selection | features `class-drawer`, `class-door` (default), `class-wall`, `class-heavy`: the factory preset used when both parameter pages are erased |

Versions are pinned exactly (FAB-FW-001). `unsafe` code lives only in `firmware/src/bsp`, each block with a `// SAFETY:` comment (FAB-FW-002).

### §9.2 RTIC task map

NVIC with 4 priority bits. `FMAC`, `CORDIC` and `I2C4_EV` are otherwise unused vectors that the base task pends, releasing the lower-rate groups drift-free on base ticks. Software tasks are dispatched on `UCPD1` and `LPTIM1`. This table must match the `#[task]` attributes in `firmware/src/main.rs` (FAB-FW-007).

| Task | Binds | Priority | Rate | Budget (cycles) | Work |
|---|---|---|---|---|---|
| `base` | `ADC1_2` | 8 | 20 kHz | 2,500 | ADC1 injected end of sequence, triggered by TIM1_TRGO at the PWM peak: `BaseTask::run` (coil current PI, LSM d/q PI in the inverter driver); snapshots the LET inputs of every group released at this tick; pends fast, supervisor and seismic |
| `fast` | `FMAC` | 6 | 10 / 5 kHz | 4,000 / 8,000 | `FastGroup::run`: frontend, levitation, motion, haptics, monitor, arbitration |
| `accel` | `EXTI9_5` | 5 | 500 Hz | 1,000 (shared with `accel_dma`) | ADXL355 data-ready (EXTI6): starts the SPI1 DMA read |
| `accel_dma` | `DMA1_CH1` | 5 | 500 Hz | (in `accel`) | SPI1 receive complete: decodes the sample into the seismic queue |
| `supervisor` | `CORDIC` | 4 | 1 kHz | 17,000 | `SupervisorGroup::run`, watchdog feed |
| `seismic` | `I2C4_EV` | 3 | 100 Hz | 17,000 | `SeismicGroup::run` |
| `can_rx` | `FDCAN1_IT0` | 3 | event | 3,000 per frame | Commands and network alerts into queues |
| `telemetry` | software task (`UCPD1`) | 1 | 100 Hz | remainder | Frames, events and link status over FDCAN1; defmt/RTT with `dev` |
| `idle` | — | 0 | — | — | WFI |

**Schedulability.** The periodic budgets give ΣC/T = 2,500/8,500 + 4,000/17,000 + 1,000/340,000 + 17,000/170,000 + 17,000/1,700,000 = 0.294 + 0.235 + 0.003 + 0.100 + 0.010 = **0.64** of the 170 MHz core, against a design limit of 0.70 (FAB-FW-008). The fast budget scales with its period, so WALL/HEAVY at 5 kHz with 8,000 cycles give the same 0.235. `can_rx` is sporadic: at one frame per millisecond it would add 0.018 (0.66 in total). The periods are harmonic, so the rate-monotonic utilisation bound is 1.0 (Liu–Layland). These are **budgets, not measurements**; the firmware records DWT min/max cycles per task and reports them in telemetry for when hardware exists (FAB-FW-012).

Mailbox locks only copy messages; every group computation runs outside the locks, so blocking is bounded by one message copy at the ceiling priority.

### §9.3 Peripherals (BSP)

| Function | Peripheral |
|---|---|
| LSM 3-phase inverter | TIM1, centre-aligned 20 kHz, dead time; STO = MOE clear; ASC = low sides on |
| Coil H-bridges | HRTIM timers A–D, one output per bridge with locked anti-phase PWM, so eight coils fit on four timers |
| Hardware overcurrent | COMP1–7 against DAC3/DAC4 thresholds, locked after configuration: COMP1–3 → TIM1 BKIN2 (LSM phases), COMP7 → TIM1 BKIN (DC bus), COMP4–6 → HRTIM fault inputs (coil bridges 0–5); bridges 6–7 use a fault input from an external comparator, because COMP7 has no HRTIM fault path on the G474 |
| Phase and coil currents | ADC1/ADC2 dual simultaneous injected (phase currents, bus current); ADC3/ADC4 injected (coil currents); all triggered by TIM1_TRGO |
| Gap probes, supply, temperatures | ADC5 and ADC3/ADC4 regular sequences with DMA, oversampled |
| Encoder | TIM2 32-bit quadrature, extended to i64 |
| Structure accelerometer (EEW) | ADXL355 on SPI1 + DMA, data-ready on EXTI6 |
| Carriage IMU and rail accelerometer | SPI3 + DMA, polled at fast releases |
| Command, alert and telemetry link | FDCAN1 |
| Parameters | Flash A/B pages (postcard image + CRC32) |
| Watchdogs | IWDG ≈ 5 ms, WWDG |

The digital pin map fits the LQFP64 STM32G474RE; the full analog front end (about 30 ADC inputs plus the comparator inputs) needs the LQFP100/128 package of the same die, which shares the memory map, PAC and image. The NUCLEO-G474RE bring-up kit carries only a DOOR-sized subset. Register values come from RM0440 and the data sheets and are to be verified at bring-up.

### §9.4 Boot, selftest and faults

1. Gate enables low; the crash record and reset flags are read. A panic, HardFault or watchdog reset holds the outputs off.
2. Clocks (170 MHz), pins, DWT, SysTick.
3. **SELFTEST, before any watchdog starts** (about 65 ms): FPSCR FZ = DN = 0, and `fabrail_rt::selftest_hash(2000) == GOLDEN_SELFTEST_HASH` (FAB-FW-005). The hash drives the full executive with a deterministic LCG board and FNV-1a-hashes every output bit; the simulator, the browser engine and the firmware compute the same function.
4. Parameters from the A/B flash pages: decode, `crc::verify_seal`, `validate`, class check, voltage coil mode.
5. Peripherals configured with every power stage in its safe state.
6. Only if every check passed: `Executive::new`, `split()` into the tasks, ADC triggers armed, watchdogs started. Otherwise the image stays in a boot hold with outputs off and reports why over the link. A crash or watchdog hold is cleared by a Reset only while every gap reads at rest.

**Panic and HardFault handler (production):** drop the gate enables, clear the TIM1 main output, disable the HRTIM outputs, write a crash record to `.uninit`, reset. `panic-halt` is rejected: it would halt with the timers still running at the last duty cycle. Fast re-catch of a still-levitated carriage after a reset is 🧪; the mechanics make a reset benign by themselves (§8.2). The panic policy is inspected (FAB-FW-009).

### §9.5 Honest verification status

| Claim | Evidence | Status |
|---|---|---|
| The shared no_std crates build for `thumbv7em-none-eabihf`; the safety crates also for `thumbv6m-none-eabi` and `armv7r-none-eabihf` | `scripts/check_nostd.sh` tier 4 (FAB-FW-013, FAB-EXO-901) | ✅ built |
| The firmware builds in release, links with rust-lld and fits its llvm-size flash and RAM budget | `scripts/check_firmware.sh` (FAB-FW-010) | ✅ built and linked, never flashed |
| Source guards, dependency allowlist, no `cfg(target_arch)`, no sim-only control path | Inspection guards (FAB-FW-002, FAB-FW-003) | ✅ (inspection) |
| The same group code gives one output hash in any LET order | `rt/tests/let_order` (FAB-FW-004) | ✅ on the host |
| The golden selftest hash is frozen and identical with and without `fault-injection` | `rt/tests/golden` (FAB-FW-005) | ✅ on the host and in wasm32; **never measured on the MCU** |
| Recorded HAL inputs replay bit-identically (voltage coil mode) | `sim/tests/replay` (FAB-FW-006) | ✅ on the host |
| Task map matches this document | FAB-FW-007 | ✅ (inspection) |
| ΣC/T ≤ 0.70 | Budget table above (FAB-FW-008) | 🔶 estimates, not measurements |
| Cycle counts, PWM-synchronous sampling, COMP→FLT latency, CiA 402, HIL, flashing | FAB-FW-012 | 🗺️ not done |

**SIL ≡ firmware is claimed in voltage coil mode only**: the firmware runs the coil current PI, so only simulations in voltage mode execute the same code path. Lag mode is a faster, simulation-only approximation of the current loop.

---

## §10 Seismic subsystem

The design and its limits are in [SEISMIC.md](SEISMIC.md). In the architecture:

- **Sensor.** An ADXL355 (500 Hz ODR, 22.5 µg/√Hz, ±2 g, 20-bit) on the structure or rail, never on the carriage; `validate` rejects a carriage mount (FAB-SEI-001). The carriage IMU serves skyhook and fusion only.
- **Chain** (seismic group, 100 Hz): CIC anti-alias ÷5 → 4-pole Butterworth 0.5–20 Hz band-pass (prewarped, TDF-II) → recursive STA/LTA 50/3000 samples, on 4.0 / off 1.5, LTA frozen while triggered → Pd and τc over 1 s and 3 s windows by double integration with a 0.075 Hz high-pass → Z/H check and actuator veto (bypassed at PGA ≥ 80 gal) → level matrix → PROTECT rule → fusion with network alerts → S-ETA (PHYSICS §20).
- **Actions** (supervisor): land-first class actions, egress rules, and the `ActionPolicy` seam through which the Sovereign tier and the house choreographer may propose a different action, filtered by the core-side `admit` rule (§15.3).
- **Research tier.** Isolation (`seismic/isolate.rs`) and active mass damping (`seismic/amd.rs`) are 🧪 and disabled in every preset.
- **fabrail is not an EEW provider.** Network alerts in simulation are synthetic or injected; a live ShakeAlert or JMA feed is roadmap and needs a licence.

---

## §11 Parameter management

- **Single source.** `fabrail-params` holds every tunable: nominal values in the presets, truth mismatch in the plant's `Mismatch`. The simulator, Python, the docs, the browser and the firmware all read the same presets. No controller contains a tunable literal.
- **f64 fields with unit suffixes.** Controllers cast to f32 in `new()`; the monitor derives its integer thresholds in `new()`.
- **Metadata.** `meta::FIELDS` has one `FieldMeta {path, unit, min, max, source}` per leaf, including array elements. `validate` checks the ranges, then the cross-field rules (stop ratios, rail saturation, τ ≥ τ_min, PWM ≥ 20 kHz, presence and isolation gates), and reports errors as `"path: reason"` (FAB-SW-004).
- **Generated table.** `python -m fabrail params --markdown` renders the parameter table (`config.parameters_markdown()`) that becomes `docs/PARAMETERS.md`, which is generated and never hand-edited; a drift guard keeps it current (FAB-SW-006).
- **Safety block.** `SafetyLimits` carries diverse copies of `k_f`, the coil and LSM current scales, `nm_per_count` and the presence fitting, plus a CRC32 over a canonical little-endian encoding. `crc::seal` refreshes the copies and the CRC after a design edit; Python `Params.set` and `with_overrides` reseal automatically, while a direct edit of a copy is rejected. A CRC or copy mismatch refuses lift (FAB-SAF-015).
- **Python access.** `Params.get` and `Params.set` address leaves by JSON pointer (`/design/suspension/g_star_m`); JSON round trips are bit-exact.
- **Firmware storage.** Two flash pages hold postcard images with CRC32 and a sequence number; boot takes the newest valid page and falls back to the factory preset of the build class when both are erased (§9.4).

---

## §12 Telemetry

- **Frame.** `TelemetryFrame` is a fixed 168-byte `repr(C)` frame with no padding, written little-endian at its in-memory offsets, identical in simulation, the browser and firmware (FAB-FW-011). Fields: tick, x, x_ref, sequence, LevState, SeismicState, MotionMode, safety demand, seismic level, class, flags, saturation flags, v, v_ref, eight gaps, eight coil currents, i_q, F̂_h, residual, m̂, governor kinetic energy, passivity energy, Pd, τc, PGA, bus voltage, floor x acceleration and the fault set.
- **Rate.** Assembled by the supervisor group at 1 kHz from the latest fast status, directive, supervisor status, seismic report and monitor verdict.
- **Events.** A 64-entry allocation-free ring (overwrite oldest) of discrete events: state changes, faults raised and cleared, triggers, alerts, intent, obstruction, detents, seat and land events, commands, selftest, reversals.
- **Transport.** Simulation: the recorder samples the default channel set at the scenario's record rate into `RunData`. Firmware: FDCAN1 frames through the `telemetry` task, defmt/RTT in development builds. Firmware also reports DWT cycle statistics.

---

## §13 Error handling

| Layer | Rule |
|---|---|
| Construction | Every `new` returns `Result<Self, ParamError>`; `ParamError` displays as `"path: reason"` with a `meta::FIELDS` path |
| Per-tick steps | Infallible: saturating integer arithmetic, clamped floats, no `unwrap`, `expect`, `panic` or unchecked indexing in core, safety and rt (clippy-denied); a non-finite value becomes the NonFinite fault and never reaches a HAL output (FAB-SAF-017) |
| Faults | `FaultCode` (u16, grouped 0x01xx sensors … 0x07xx seismic response), each with a severity; `FaultSet` bitset; the supervisor latches faults and maps the maximum severity to a demand |
| Simulation | `SimError {Json, Schema, Invalid{path, reason}, Series, Param, CoilModeMismatch, Unstable, UnknownChannel, Plant, NotImplemented, Runtime}` |
| Python | PyO3 maps errors to `ValueError` (and `NotImplementedError`); the CLI returns exit code 1 on invalid input |
| Firmware | Boot hold with outputs off on any selftest, CRC or validation failure; the production panic handler disables the outputs, records and resets (§9.4) |

---

## §14 Deployment per class

This is the intended installation, not a built or tested product (🗺️ hardware).

| Class | Supply | Hold-up | Rest | Notes |
|---|---|---|---|---|
| DRAWER (8–48 kg) | 24 V through contacts at the captive seat | 10 F supercap at 12 V on the carriage; open hover ≈ 5 min (≈ 490 J usable at 1.5 W), then a seat or fail-down | Captive SEAT at 3.5 mm, ≤ 0.3 W | Contactless travel ≤ 50 % extension (0.30 m); roller support beyond is roadmap (FAB-LEV-016) |
| DOOR (100 kg) | 48 V (SELV) plus the link through a festoon | Supercap ≥ 3× the landing energy | HOVER at g*, ≤ 0.5 W sleep | Egress role declared per installation; never locks (FAB-SEI-011) |
| WALL (500 kg) | Dock-charged at the park position; 48 V (SELV) | ≥ 3× (move + landing) at the dock (FAB-PWR-004) | LAND at park | Stray field ≈ 16 mT unshielded: at risk against the 0.5 mT target (FAB-ENV-001) |
| HEAVY (5 t) | Dock-charged; 300 V bus (not SELV) | 10 F at 48 V (7.8 kJ), ≥ 3× (move + landing) | LAND + parking lock | Hold-to-run unless PL d presence sensing is fitted (FAB-SAF-003); shrouds required (≈ 80 mT) |

Version 0.1 is one combined node per carriage (sensing, coil drivers, LSM drive and supervisor on one MCU). Separate carriage and track nodes, multi-carriage rails, curves and switches are roadmap.

---

## §15 Terminus, inertial smoothing and the house choreographer

These three layers were added for the example gallery (plan §4b). The first is a core motion primitive; the second is a research-tier core mode; the third is a Sovereign-tier planner that is not a safety function.

### §15.1 Log-damped terminus (✅, `core/src/motion/terminus.rs`)

The reference generator for every commanded close and open (`MotionMode::Terminus`, `SoftOpen`), specified in PHYSICS §16.1:

1. A constant-jerk blend leaves the cruise S-curve: t_b = √(τ² + 2v/j) − τ, d_b = j·t_b·τ².
2. Exponential capture d(t) = d_b·e^(−t/τ): ln d is a straight line of slope −1/τ on a semilog chart, which is the measurable meaning of "log-damped".
3. At d_ε ≈ 0.2 mm a quintic from (d_ε, −d_ε/τ, d_ε/τ²) over T_q = τ lands monotonically at v = a = 0.
4. Arrival t_arrive = t_b + τ·ln(d_b/d_ε) + T_q; DRAWER adds 0.2 s for the z-seat step.

`validate` enforces τ ≥ τ_min = v_app/a_max − a_max/(2j); at runtime τ_eff = max(τ, τ_min(v_measured)), which covers a hand-thrown drawer. The velocity integrator is frozen (bumpless) during capture, giving error poles (s + ω_v/2)² with ω_v/2 ≥ 3/τ, and the governor treats the terminus plus an overtravel as its obstacle point so it never clips the exponential tail. A hand during capture switches to class admittance; release re-plans from the measured state; an obstruction always overrides the target (FAB-HAP-013, FAB-HAP-014, FAB-HAP-015, FAB-HAP-016). Nominal termini never reach the end zones or bumpers, which remain fail-safes (FAB-SAF-014).

### §15.2 Inertial smoothing (🧪, `core/src/motion/inertial.rs`)

Along x a levitated drawer is already a free mass. Smoothing makes it follow only the floor motion below a crossover f_c ≈ 1 Hz, using skyhook absolute-velocity feedback c_sky = 2ζ·m̂·ω_c from the carriage IMU (leaky integral with a 0.1 Hz high-pass); the residual follows T_sky = 1/√((1 − r²)² + (2ζr)²) (PHYSICS §16.2). It is disabled in every preset and admitted only for a drawer already open ≥ 25 mm plus the predicted stroke, with force, stroke, vertical-headroom and energy budgets met; seated drawers never unseat on an alert, and every exit is a log-damped close to the seat (FAB-HAP-017, FAB-HAP-018, FAB-HAP-019).

### §15.3 The ActionPolicy seam and the house choreographer (`fabrail-exo/src/choreo`)

The supervisor computes the core land-first class action and then asks an `ActionPolicy` whether to override it. The firmware and every core preset use `CorePolicy`, which never overrides. Exotic fusion and the house choreographer implement the trait in simulation. Whatever they propose passes the core-side `admit` filter:

- `SeismicAction::None` may not replace a protective default;
- `OpenAndStop` is forbidden whenever any non-local source contributed;
- `Lock` is forbidden on an egress path;
- alert-triggered motion needs independent confirmation (a local P pick or ≥ 2 feeds), may only close or seat, never reverses a user-initiated motion, and is capped at the unattended limit v ≤ min(v_cap, 150 N/√(m_gov·k_series)), 0.23 m/s at 48 kg.

The choreographer plans every device of a building scene from a `FusedAlert`, the S-ETA distribution, the device registry and the building configuration: deadlines at the 5 % quantile of t_S minus 1 s, staggering under the bus power budget, motion only toward a higher `SafetyRank` (Seated or Landed+Locked > Closed-levitated > Standoff/Hold > Frozen-open), one attempt, obstruction armed. It is **declared not a rated safety function**, like the core seismic action SF-08 (SAFETY.md §5). Its acceptance is FAB-EXA-004 to FAB-EXA-016; the spoof matrix is FAB-EXO-119.

---

## §16 Exotic crate layering

The Sovereign tier follows the core pattern exactly: integer safety below, f32 controllers above, f64 truth beside.

```text
 fabrail-exo-safety  (no_std, integer; types + params only)
    plain data: ExoClass, AnyClass, SafetyRegime, SafeStatePolicy, SafetyRank, FusedAlert, AlertFeedKind
    logic: 2oo3 mid-value select, ladder 2oo3 → 1oo2D → 1oo1D → safe, black channel, zone machine,
           ISO 13855 separation, integer envelope, alert gate, PFH (two forms + CTMC)
        │
        ▼
 fabrail-exo  (no_std, f32; + fabrail-core)
    HMD (LQR, saturation stack, DVF fallback, mode ladder) · TMD tuning · HAVEN skyhook · energy admission
    · closure-race planner · static pod allocation · EEW fusion (ActionPolicy) · house choreographer
        │
        ▼
 fabrail-exo-sim  (std, f64; + plant, sim)
    SuspensionModel (HybridEms adapter, PureEms, HTS, AirPad) · storage · 2-DOF and modal towers
    · HAVEN-R 3-DOF platform · ground-motion additions · EEW and tsunami timing · scenarios · building scene
    feature "py": registers the flat exo_* functions in _native
```

- The pedestrian caps are imported from `fabrail_params::presets`, never restated; exotic code never changes the core `RailClass` or its caps (FAB-EXO-006, FAB-EXO-114).
- The safety crates' dependency closure excludes every control and plant crate (FAB-EXO-116).
- Frontier (🔭) models exist only in numpy under `python/fabrail/exotic/frontier/`; no Rust crate contains one (FAB-EXO-002).
- The no_std exotic crates cross-build for thumbv7em; `fabrail-exo-safety` also for thumbv6m and armv7r (FAB-EXO-901). Sovereign firmware applications are 🗺️; the architectures are designed toward SIL 3 or SIL 4 targets and are not certified (EXOTIC §S3, §S4).

---

## §17 Browser engine

`crates/fabrail-web` compiles the real controller and plant to `wasm32-unknown-unknown` as a cdylib with hand-written `extern "C"` exports; there is no wasm-bindgen, and `docs/assets/js/engine.js` is the whole loader.

- **Engine.** One device per engine: an unchanged class preset on one plant tier, closed through `fabrail_sim::Session`, stepped single-threaded (no rayon on wasm32). The page sets a hand force, a floor acceleration and a command, steps n base ticks, and reads a 20-value f64 state buffer.
- **Proof on every page.** Each page computes `selftest_hash()` in the browser and compares it with `GOLDEN_SELFTEST_HASH`; the same wasm under node must match it, and a wasm step run must match the native `run()` trace bit for bit over the first 2,000 ticks (FAB-SW-014).
- **Budget.** The module stays ≤ 1.5 MB, with its SHA-256 recorded in the generated anchors block.
- **Fallback.** Without wasm, pages fall back to the JS FORMULAS block, which is checked against the Rust anchors under node (FAB-SW-013).
- **ABI.** Append-only and versioned; the export list and state layout are guarded against `engine.js` by a Rust test.

---

## §18 Build and verification gates

| Gate | Command | What it establishes |
|---|---|---|
| Rust tests | `cargo test --profile itest`; `cargo test --release` for the anchors and golden hash | Unit, property and closed-loop tests of every crate |
| Portability | `scripts/check_nostd.sh --strict` | Source guards, per-crate builds, the thumbv7em dependency allowlist, real cross builds, the safety-crate dependency closure |
| Firmware | `scripts/check_firmware.sh` | Release build, rust-lld link, llvm-size budget, no `fault-injection` in the manifest or lockfile |
| Python | `pytest -q` (not slow), `pytest -q -m slow` | Acceptance, seismic, examples, drift guards, the site |
| Traceability | `python -m fabrail trace` | Every requirement maps to marked tests in both directions; every cited FAB ID exists; every hazard in SAFETY.md maps to a requirement (FAB-SW-007) |
| wasm | `rustup run stable cargo build -p fabrail-web --target wasm32-unknown-unknown --release` | The browser engine and its selftest hash |

What none of these establish: hardware timing, PWM-synchronous sampling, comparator latency, EMC, acoustics, measured forces or fields, component reliability, and any performance level or SIL. See SAFETY.md §8.
