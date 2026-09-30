# Fabricated Zero Perfect Friction Rails System

fabrail implements a design using compact magnetic suspension and lateral propulsion using precision controlled magnetic field components for use in cabinetry, sliding doors, movable infrastructure.

Programmatic control provides the most tactile experience with intuition, extending into technical mastery of momentum control for large infrastructure components mitigating seismic forces through pressure wave effects.

---

**Live site:** https://sonoransun.github.io/fabrail/ &nbsp;·&nbsp; **Read the docs:** [Narrative](docs/NARRATIVE.md) · [Specification](docs/SPECIFICATION.md) · [Technical design](docs/TECHNICAL_DESIGN.md) · [Physics](docs/PHYSICS.md) · [Safety](docs/SAFETY.md) · [Seismic](docs/SEISMIC.md) · [Exotic tier](docs/EXOTIC.md) · [Frontier](docs/FRONTIER.md) · [Capability matrix](docs/capability-matrix.md)

**fabrail is a complete engineering model of a magnetically levitated, magnetically propelled rail.** It spans a 48 kg kitchen drawer up to a 5 t movable wall, and a sovereign tier reaching kiloton building dampers.

- **Suspension.** The carriage hangs a few millimetres under a steel rail on **hybrid permanent-magnet/electromagnet suspension with zero-power control**. Coil current decays to zero at equilibrium, so the permanent magnets carry the weight.
- **Propulsion.** A **long-stator linear synchronous motor** drives the carriage along the rail.
- **Control, all programmable:**
  - admittance haptics: push-to-open, rendered mass, detents, log-damped soft open and close;
  - an energy and momentum governor that uses rated mass;
  - seismic early warning on the P-wave, with a land-first policy.

**The same `no_std` Rust control core runs in three places:**
- the Python-driven simulator;
- an STM32G474 RTIC firmware image that builds and links;
- your browser, via WebAssembly.

A golden selftest hash ties the three together: every build must produce the same hash.

> [!IMPORTANT]
> **Scope, stated honestly.**
> - **Friction.** "Zero perfect friction" is a direction. The modelled equivalent friction is μ_eq ≈ 1e-3–1e-2. Contact is avoided only in nominal operation; drawers, walls and heavy panels rest in contact.
> - **"Pressure wave".** This is the seismic **P-wave**, which fabrail detects so it can act before the S-wave. No magnet exerts a "pressure-wave force", and nothing here makes a building earthquake-proof.
> - **Warning time.** On-site warning within about 20 km of the source is roughly 0 s.
> - **Research and frontier items.** Isolation and active mass damping are research tier. Frontier items (🔭) are theoretical what-ifs, never products.
> - **Standards and certification.** Nothing is certified. Standards limits (EN 16005, ANSI/BHMA A156.x, ISO/TS 15066, IBC 1010.3, …) are **checked in simulation against numeric limits from the stated edition**.
> - **Firmware.** It builds and links, but it has **never been flashed**.

---

## Contents

- [Reading the README's words](#reading-the-readmes-words)
- [What fabrail computes](#what-fabrail-computes)
- [Product classes](#product-classes)
- [Physics in brief](#physics-in-brief)
- [Control stack](#control-stack)
- [Tactile interaction and log-damped termini](#tactile-interaction-and-log-damped-termini)
- [Momentum and safety](#momentum-and-safety)
- [Seismic: three readings of "pressure wave effects"](#seismic-three-readings-of-pressure-wave-effects)
- [Sovereign / exotic tier](#sovereign--exotic-tier)
- [🔭 Frontier tier](#-frontier-tier)
- [Example gallery and the flagship `villa_quake`](#example-gallery-and-the-flagship-villa_quake)
- [Architecture](#architecture)
- [Firmware target](#firmware-target)
- [The animated site](#the-animated-site)
- [Install](#install)
- [Quickstart](#quickstart)
- [CLI reference](#cli-reference)
- [Studies](#studies)
- [Validation and traceability](#validation-and-traceability)
- [Project status](#project-status)
- [Limitations](#limitations)
- [Documentation map](#documentation-map)
- [License](#license)

---

## Reading the README's words

| README phrase | Meaning in fabrail | Where |
|---|---|---|
| "compact magnetic suspension" | Hybrid PM-biased EMS under a steel rail, with Mizuno zero-power control | [PHYSICS §2–§4](docs/PHYSICS.md) |
| "lateral propulsion" | Travel **along the rail (x)** by a long-stator LSM. In code, `lateral` always means the cross-track axis **y** | [SPEC FAB-PRP-001](docs/SPECIFICATION.md) |
| "precision controlled magnetic field components" | One deterministic schedule: a 20 kHz current loop, a 10/5 kHz gap and haptic loop, a 1 kHz supervisor and a 100 Hz seismic group | [TECHNICAL_DESIGN](docs/TECHNICAL_DESIGN.md) |
| "most tactile experience with intuition" | Admittance control, sensorless hand-force estimation, detents, passivity control, and log-damped soft open and close | [§Tactile](#tactile-interaction-and-log-damped-termini) |
| "technical mastery of momentum control" | An energy and momentum governor based on **rated** mass, jerk-limited S-curves, stop-distance envelopes and clamp-force physics | [§Momentum](#momentum-and-safety) |
| "mitigating seismic forces through pressure wave effects" | P-wave early warning, contactless isolation (🧪) and active mass damping (🧪) | [SEISMIC.md](docs/SEISMIC.md) |

## What fabrail computes

| Goal | In model terms | Provided by |
|---|---|---|
| Levitate stably at near-zero power | Co-energy magnet model; discrete zero-power gain design at ω = 2.5·p; modal allocation with a warp null-space | `fabrail-plant::magnet`, `fabrail-core::levitation` |
| Move smoothly and stop precisely | 7-segment jerk-limited S-curve, cascade with feed-forward, log-damped terminus | `fabrail-core::motion` |
| Feel right under the hand | Momentum observer, admittance, detents, Hannaford–Ryu passivity control, intent and obstruction detection | `fabrail-core::haptics` |
| Never carry dangerous energy | Governor on m_gov = max(m_rated, 1.05·m̂); directional force clamp; independent integer monitor | `fabrail-core::motion::governor`, `fabrail-safety` |
| Fail gracefully | SS1 Safe Landing on a supercap; STO/ASC on propulsion; mechanical stops on both sides of the gap | `fabrail-core::supervisor`, `fabrail-rt` |
| Act on the P-wave | STA/LTA, Pd/τc, level matrix, network fusion, class policies | `fabrail-core::seismic` |
| Generate realistic inputs | Kanai–Tajimi/Clough–Penzien ground motion, Boore stochastic P+S, rail irregularity, human intent profiles | `python/fabrail/{ground,pwave,rail,human}.py` |
| Scale to sovereign infrastructure | HTS flux pinning, tower HMD, 2oo3 voting, ride-through storage, EEW fusion | `fabrail-exo*`, `python/fabrail/exotic` |

## Product classes

Values are the corrected presets in `fabrail-params` (see [SPECIFICATION §2](docs/SPECIFICATION.md)).

| Class | Moving mass | Rest state | Nominal gap | Open-loop pole | v_max unsensed | Seismic default |
|---|---|---|---|---|---|---|
| **DRAWER** | 8–48 kg (rated 48) | captive **seat** at 3.5 mm | 2.5 mm | 69.5 rad/s | 0.265 m/s | freeze → stay seated |
| **DOOR** | 100 kg | **hover** | 5 mm | 47.6 rad/s | open 0.184 / close 0.152 m/s | freeze → stop → land on rollers, never lock |
| **WALL** | 500 kg | land at park | 10 mm | 32.5 rad/s | 0.0446 m/s | freeze → land + lock |
| **HEAVY** | 5 t | land + parking lock | 10 mm | 32.5 rad/s | 0.025 m/s, hold-to-run | freeze → land + lock |

- **Faster speeds** require PL d presence sensing; for example, WALL goes to 0.15 m/s.
- **Drawer extension.** Contactless drawer travel is limited to 50 % extension (0.30 m). Beyond that, the cantilever moment exceeds the magnets' capacity.

## Physics in brief

**Earnshaw's theorem rules out passive static levitation.** Heave and pitch are therefore actively controlled in every class, and roll is also controlled on DRAWER and HEAVY. Top-hung DOOR and WALL are roll-stable under gravity. Cross-track y is held by passive reluctance centring plus a bottom guide fin.

```text
F(g, i) = μ0·A_p·(M_pm + N·i)² / (2g + h′)²          hybrid U-core, y = 0 (co-energy form in code)
p       = √(2·g_acc / g_eff),  g_eff = g + h′/2         open-loop unstable pole, independent of mass
m(g)    = m*·((2g* + h′)/(2g + h′))²                    zero-power load ↔ gap: extra load pulls the carriage up
F_x     = K_f(x)·i_q − F_cog(x) + ripple                 long-stator LSM, amplitude-invariant dq
```

**Four fidelity tiers:**

| Tier | Model | Step |
|---|---|---|
| **Carriage6** | 6-DOF rigid body; co-energy magnets with saturation and eddy lag; coil ODE with motional EMF; dq LSM; elasto-plastic contacts | RK4, 50 µs |
| **Heave1** | Vertical axis only | 50 µs |
| **Travel1d** | Travel axis only, for seismic Monte Carlo | 1 ms |
| **Structure** | Host building, for AMD/TMD research | — |

Every equation, exactly as implemented, is in [PHYSICS.md](docs/PHYSICS.md). The source code cites it as `// PHYSICS §n`.

## Control stack

```text
 20 kHz  base      current PI ×n (ω_c = 2π·1 kHz) · LSM dq PI · latch raw samples
 10/5 kHz fast     frontend → zero-power levitation (modal, warp) → motion/haptics → governor → clamp  (+ integer safety monitor)
  1 kHz  supervisor state machines · trajectories · intent · faults · seismic policy · telemetry · watchdog
100 Hz   seismic   ADXL355 (structure-mounted) → STA/LTA → Pd/τc → levels → fusion
```

Rate groups exchange data only through **logical-execution-time mailboxes**, and the base tick always runs first. So the simulator and the firmware see identical bits regardless of preemption order. Three checks prove this:
- a permutation test;
- a replay test;
- `GOLDEN_SELFTEST_HASH`, which the firmware checks at boot.

```text
BOOT → SELFTEST → SEATED | LANDED | LEVITATED_IDLE ⇄ LIFTING → MOVING → STOPPING → SEATING | LANDING
any state → FAULT_REACTION (SS1 Safe Landing; STO on the LSM only) → FAULT (latched)
side branches: SEISMIC · ESCAPE · MANUAL
```

## Tactile interaction and log-damped termini

- **Admittance.** M_v·ẍ + B_v·ẋ = F̂_hand renders a lighter panel. The apparent-mass ratio is m/M_v ≤ 5 without a force sensor, or ≤ 20 with a handle load cell. **Rendered lightness never reduces impact energy.**
- **Intent and obstruction.**
  - Push-to-open works even while the drawer is seated.
  - Intent detection requires direction consistency, so leaning, cabinet air pressure and smoke-control pressure differentials are rejected.
  - Obstruction triggers stop-and-reverse, or a release to 0 N.
- **Passivity.** A frictionless rail has almost no physical damping. Virtual stiffness is therefore bounded by K ≤ 2b/(T + 2T_d), and a time-domain passivity observer and controller guard the rest.
- **Log-damped termini.** On soft-open and soft-close, the distance to the terminus decays as d(t) = d_b·e^(−t/τ). **On a semilog plot this is a straight line.**
  - Entry is a C² jerk-limited blend; the final approach is a quintic hand-off.
  - The carriage arrives with v = a = 0 and never touches a bumper.

  | Class | τ | Time to arrive |
  |---|---|---|
  | DRAWER | 0.15 s | 0.98 s (+0.2 s to seat) |
  | DOOR | 0.25 s | 1.62 s |
  | WALL | 0.6 s | 3.56 s |
  | HEAVY | 1.0 s | 5.84 s |

  Measured in the closed loop on the 6-DOF drawer: semilog fit R² = 0.99996, overshoot 0, arrival speed 0.94 mm/s.

## Momentum and safety

Kinetic energy depends on **physical** mass, not rendered mass. The governor therefore limits

```text
v ≤ min( √(2·1.69 J / m_gov),  22.3 kg·m/s / m_gov,  √(2·a_brk·(x_obs − x)) )   with m_gov = max(m_rated, 1.05·m̂)
F_clamp = v·√(m·k_body)   →   1.69 J gives 291 N on a chest, 503 N on a hand; a 10 N/mm edge seal brings it to 173 N
```

**Layers of protection:**
1. The core governor and directional clamps: 67 N static when unsensed, 150 N when sensed or in the last 50 mm.
2. An **independent integer-only monitor** (`fabrail-safety`). It never links the control core and uses rated mass.
3. A hardware current clamp, and STO/ASC on propulsion.
4. SS1 Safe Landing on a supercap.
5. Mechanical measures: upper and lower rollers, a captive guideway, end zones and bumpers.

[SAFETY.md](docs/SAFETY.md) lists hazards H-01 to H-21, the FMEA-lite and the safety functions. The safety-function targets are **declared, not certified**.

## Seismic: three readings of "pressure wave effects"

| Reading | What fabrail does | Tier |
|---|---|---|
| **(a) P-wave early warning** | Structure-mounted detector: STA/LTA 0.5 / 30 s, Pd and τc over 1 s and 3 s windows. PROTECT when Level ≥ 2, Pd ≥ 0.35 cm or PGA ≥ 80 gal. Network alerts are fused; the S-wave ETA comes from origin time. | ✅ simulated |
| **(b) Active or tuned mass damping** | 2-DOF host-structure studies (Den Hartog, Warburton, direct velocity feedback, LQR). Door and wall mass ratios are ≪ 1 %, so the effect on the building is negligible. | 🧪 research |
| **(c) Contactless isolation** | Virtual spring plus skyhook damping, a stroke governor, eddy-current end zones and bumpers. The WALL's 100 N continuous thrust gives T_min ≈ 6.3 s, so isolation largely saturates. | 🧪 research |

**Warning time.** The S−P interval is R·(1/V_s − 1/V_p), about 0.119 s per km of hypocentral distance, minus trigger and window time.
- Within about 20 km, usable on-site warning is ≈ 0 s.
- On-site and network warnings cross over at ≈ 41 km, which is why fabrail fuses both.

**Default policy: land first.** Magnetic lateral capacity (0.1–0.3 g) is below floor accelerations (0.3–1 g), and a free-floating mass drifts by the full peak ground displacement.
- Egress doors never lock.
- Fire-rated doors never open.

## Sovereign / exotic tier

*"Cost is no impediment."* These classes cover billionaire- and nation-state-scale infrastructure. At these masses the pedestrian energy caps would allow only mm/s, so the classes are **safe by separation** instead: guarded zones, ISO 13855 distances, triplex sensing and 2oo3 voting.

| Class | Scale | What it is | Maturity |
|---|---|---|---|
| **SENTINEL** | 1,000 t, ±1.5 m | Tower hybrid mass damper on a maglev planar bearing. It rides through levitated and has no friction threshold. Mainly a **wind** device (ζ_add ≤ (μ/2)(S/X)). | TRL 3–4 |
| **HAVEN-R / -F** | 50 t / 5 kt | Critical-payload and facility isolation, T = 6 s, with passive vertical support | TRL 2–3 |
| **SHIELD** | 5–500 t | Radiation plug doors with a passive mover. On alert: beam off, then land and lock within 2 s | TRL 3 |
| **BASTION** | 25–100 t | Blast and vault doors, 3 m in about 5.2 s. The closure race takes blast arrival time t_a as an input; there are no yield tables | TRL 2–3 |
| **ARENA** | 500–10,000 t | Stadium trays and roof petals built from pods, with static reallocation when a pod is lost | TRL 3 |
| **ROTATOR** | 6,100 t ring | Telescope domes and rotating floors | TRL 3 |
| **TIDEGATE** | 20–300 t | Flood and tsunami land-lock gates. The fused trigger stack is the real product | 2 (trigger 7–9) |
| **SELENE** | 100 t hatch | Lunar passive HTS in permanently shadowed regions | 🔭 |

**Technology models:**
- HTS flux pinning, fitted to the ATZ demonstrator: 53 kPa at 10 mm, needing 5–8× the area of EMS.
- FeCo poles, with a single admission function capped by rail saturation.
- Flywheel, supercap and battery ride-through.
- A PFH calculator in both IEC 61508-6 forms.
- Sovereign EEW fusion: seafloor sensors and fibre-optic DAS add 0.154 s of warning per km of sensor advance; PEGS contributes a magnitude advisory.

Details: [EXOTIC.md](docs/EXOTIC.md).

## 🔭 Frontier tier

*Theoretical extensions of bleeding-edge technology.* These are numpy what-if models only. Every output carries a banner and states the orders of magnitude still missing.

| What-if | Anchor |
|---|---|
| Prompt elasto-gravity signals (PEGS) beating the P-wave | Needs a gradiometer at 1e-15 /√Hz; the best demonstrated is 1e-10. Against a dense network the gain is ≈ 0 s |
| Active maglev seismic metabarrier | Mass ∝ ρ·v_s³/f³, about 1.8 Mt at 1 Hz |
| Quasi-zero-stiffness isolation (Mizuno) | Real physics at instrument scale; 🧪 at building scale |
| Room-temperature superconductor | Needs Tc ≈ 358 K; pressure stays rail-limited |
| Magnetic cloaks and concentrators, NV/atom sensors, lunar HTS, DAS along the rail, safe-RL haptics | See [FRONTIER.md](docs/FRONTIER.md) |

## Example gallery and the flagship `villa_quake`

```bash
python -m fabrail example list
python -m fabrail example run kitchen_push_open --quick
```

| # | Example | Badge |
|---|---|---|
| 1 | `kitchen_push_open`: push-to-open from the seat, log-damped termini | ✅ |
| 2 | `pantry_cabinet_slider`: 8 kg slider derived from the drawer preset, with detents | ✅ |
| 3 | `office_door_obstruction`: chest obstruction checked against the EN 16005 force–time envelope | 🔶 |
| 4 | `ballroom_partition`: 500 kg wall rendered at m/5 | ✅ |
| 5 | `archive_heavy_shelving`: HEAVY hold-to-run, governor tested with a sabotaged m̂ | ✅ |
| 6 | `drawer_power_loss`: SS1 Safe Landing and manual force | ✅ |
| 7 | `wall_pwave_m7`: land-first timeline against t_S | ✅ |
| **8** | **`villa_quake`**: see below | 🧪 |
| 9–14 | `supertall_wind_day`, `museum_haven_quake`, `proton_shield_pwave`, `vault_close_race`, `stadium_tray_pod_failure`, `tidegate_offshore_m8` | 🧪 |
| 15–16 | `pegs_long_period_securing`, `lunar_hatch_selene` | 🔭 |

### `villa_quake`: "A house that closes its own drawers"

A simulated M7.2 strikes 60 km from a hillside villa.

**The house:**
- a kitchen with six drawers of glassware and porcelain, plus two cabinet sliders;
- a pantry door on the egress path;
- a salon partition wall;
- an art plinth.

It runs in three building configurations: fixed base, conventional isolation, and levitated isolation (🧪).

**What happens:**
1. A fused, **synthetic** alert, confirmed by a local P pick, leaves 5.7–7.3 s before the planning deadline.
2. A house choreographer schedules every device in parallel within the bus power budget:
   - open drawers glide shut on **log-damped** curves at unattended, pinch-safe speeds, then seat or latch;
   - the door stops, lands and never locks;
   - the wall lands and locks;
   - the plinth pre-centres.
3. After the shaking, the system waits for the inspection reset. Then, and only on a **local** owner command, drawers re-open to their remembered positions on log-damped opening curves.

Alerts only ever move devices toward safety. They never open anything, and never reverse a motion a person started.

**Outputs:**
- a timeline against t_P and the t_S distribution;
- a compensation-stack waterfall: ground → floor → drawer body → contents;
- semilog insets whose straight lines are the log-damped signature;
- per-glass friction-cone utilisation;
- paired runs with inertial smoothing on and off. Smoothing helps on a fixed base; **on an isolated building the isolation does the work.**

The strongest claim the example makes is "no Coulomb slip event in the sliding-block model". It never claims "no damage".

## Architecture

```text
fabrail-types ─┬─> fabrail-params ─┬─> fabrail-core ────┐
 (libm only)   ├─> fabrail-hal     ├─> fabrail-safety ──┼─> fabrail-rt ─┐
               │                   │   (never core)     │                ├─> fabrail-sim ─┬─> fabrail-py  (_native, PyO3)
               └─> fabrail-plant (f64 truth physics) ───┴────────────────┘                ├─> fabrail-web (wasm32)
fabrail-exo-safety (integer, plain data) ─> fabrail-exo (f32 controllers, choreographer) ─> fabrail-exo-sim (truth, scenes)
firmware/ (separate workspace: RTIC 2 + stm32g4xx-hal) ─ path deps ─> types, params, hal, core, safety, rt
```

| Crate | Role |
|---|---|
| `fabrail-types` | Shared vocabulary: `PosNm(i64)`, `Tick(u64)` at 20 kHz, raw HAL samples, states, faults, telemetry. `fmath` is the only math entry point |
| `fabrail-params` | Single source of presets for the simulator, Python, docs and firmware. f64 fields, closed forms (`derive`), validation, field metadata, a CRC-sealed safety block |
| `fabrail-hal` | Portability contract: traits only |
| `fabrail-core` | The `no_std` f32 controller: levitation, motion, terminus, inertial smoothing, haptics, seismic, supervisor |
| `fabrail-safety` | Independent integer monitor. Builds for thumbv6m and armv7r as a diverse channel |
| `fabrail-rt` | LET executive, arbitration, golden selftest |
| `fabrail-plant` | f64 truth physics in four tiers. Uses no randomness |
| `fabrail-sim` | Scheduler, SimBoard, sensor models, metrics, recorder, replay, steppable `Session`, rayon batches |
| `fabrail-py` | Python bindings. Arrays cross as little-endian bytes |
| `fabrail-exo-*` | Sovereign tier: safety types and voting; controllers and choreographer; truth models and building scenes |
| `fabrail-web` | Hand-written `extern "C"` wasm exports (`fr_*`) used by `docs/assets/js/engine.js` |

**Determinism.**
- The controller uses no randomness.
- Sensor noise comes from in-repo SplitMix64/Xoshiro256++ substreams; scenario synthesis uses numpy `SeedSequence`.
- The same seed gives byte-identical JSON, and ensembles are invariant to the worker count.

## Firmware target

`firmware/` is an RTIC 2 application for the **STM32G474RE**.

| Task | Rate |
|---|---|
| Current loop, on the injected-ADC ISR | 20 kHz |
| Fast group | 10 / 5 kHz |
| Accelerometer (SPI + DMA) | 500 Hz |
| Supervisor | 1 kHz |
| Seismic | 100 Hz |
| CAN FD | per event |

- A production panic/HardFault handler switches outputs off, writes a crash record and resets.
- The boot SELFTEST compares the shared-core hash with `GOLDEN_SELFTEST_HASH`.
- `unsafe` appears only in `bsp/`.
- Every class variant **builds and links** for `thumbv7em-none-eabihf`, at about 310 KB flash and 20 KB RAM. [firmware/README.md](firmware/README.md) has the budgets and status.
- **Status:** never flashed. Cycle counts, ADC/PWM synchronisation and hardware-in-the-loop testing are roadmap.

```bash
scripts/check_firmware.sh          # build + link every class feature, size budget, no fault-injection in the image
```

## The animated site

`docs/` is a GitHub Pages site with no build step, no external URLs and no JS libraries. It is dark-first, honours reduced motion, and works on phones.

- The animations run the **real controller and plant in WebAssembly** (`fabrail_web.wasm`).
- Each page shows the selftest hash computed in your browser next to the firmware's golden hash.
- Without wasm, pages fall back to a closed-form `FORMULAS` block, which a test checks against the Rust anchors.

| Page | Shows |
|---|---|
| `index.html` | Hero: a floating drawer; a P-wave sweeps in and the drawer glides shut on a log-damped curve |
| `levitate.html` | Earnshaw instability with control switched off; zero-power float under a draggable payload; stop ratios |
| `glide.html` | S-curves, a live semilog terminus, the governor, a pinch-force dial against EN 16005 |
| `touch.html` | Haptic playground: drag a drawer, door or wall driven by the real admittance controller |
| `quake.html` | P and S wavefronts, the lead-time crossover, detector traces, class action timelines |
| `villa.html` | Flagship replay: house cross-section, glassware, waterfall, building toggle |
| `sovereign.html`, `frontier.html`, `examples.html` | The exotic tier, the 🔭 observatory, the gallery |
| `read/*.html` | Every Markdown document, rendered by `scripts/build_site.py` |

```bash
python -m http.server -d docs 8000      # local preview at http://localhost:8000
```

To publish: Settings → Pages → Deploy from branch, folder `/docs`.

## Install

**Requirements:**
- Python ≥ 3.10 with numpy, scipy and matplotlib
- Rust ≥ 1.89
- maturin
- Optional: rustup with the `thumbv7em-none-eabihf`, `thumbv6m-none-eabi`, `armv7r-none-eabihf` and `wasm32-unknown-unknown` targets, for cross, firmware and wasm builds
- Optional: node, for the site checks

```bash
python3 -m venv --system-site-packages .venv && source .venv/bin/activate
pip install maturin
maturin develop --release                       # builds the Rust core into fabrail._native
python -c "from fabrail import _native as n; print(n.Params.preset('door').derived()['pole_rad_s'])"   # 47.57…
```

> [!NOTE]
> If Homebrew's `rustc` comes first on PATH, point cargo at the rustup compiler for cross builds:
> `RUSTC=$(rustup which rustc --toolchain stable) rustup run stable cargo build -p fabrail-core --target thumbv7em-none-eabihf`.

## Quickstart

```bash
# The classes and their derived quantities (poles, speed caps, stop distances, …)
python -m fabrail presets

# A door push-to-open, with figures and a fabrail.run/1 JSON record
python -m fabrail simulate -c door --scenario push_open --json output/door.json

# A wall facing an M7 at 20 km: detection, PROTECT, landing before the S-wave
python -m fabrail seismic -c wall --ground pwave --magnitude 7 --distance-km 20

# Studies, sovereign simulations, the gallery
python -m fabrail study --list && python -m fabrail study terminus_semilog
python -m fabrail exo --help
python -m fabrail example run villa_quake --quick

# Tests
cargo test --workspace --profile itest          # Rust: anchors, proptests, golden hash, LET permutation, replay
python -m pytest -q                              # Python: not-slow tier (each test ≤ 15 s)
python -m pytest -q -m slow                      # Monte Carlo, 24 h detector noise, full villa matrix
scripts/check_all.sh                             # fmt, clippy, tests, no_std + cross builds, firmware, trace
```

## CLI reference

`python -m fabrail <command>`.

**Common flags:**
- `-c/--class {drawer,door,wall,heavy}`
- `-s/--seed` (default 42)
- `--set PATH=VAL` (parameter override)
- `--plant KEY=VAL` (truth mismatch)
- `-o/--output`, `--no-plots`, `--json PATH`
- `-w/--workers`
- `--fidelity {carriage6,heave1,travel1d,structure}`
- `--coil {lag,voltage}`

| Command | Purpose |
|---|---|
| `presets` | Class table with derived quantities |
| `params` | Resolved parameters; `--describe` shows field metadata; `--markdown` regenerates [PARAMETERS.md](docs/PARAMETERS.md) |
| `scenarios` | Scenario catalogue |
| `simulate` | Run one scenario |
| `compare` | Run one scenario across all classes |
| `ensemble` | Seeded ensembles with KPI distributions |
| `seismic` | Ground motion (Kanai–Tajimi, Clough–Penzien, P+S, or a record file) under a land, isolate or damp policy |
| `detect` | Replay the core detector on a recorded 3-axis trace (e.g. STEAD) |
| `study` | `--list`, a study name, or `all --tier 1\|2` |
| `anchors` | Analytic and seed-42 acceptance anchors |
| `trace` | Requirement ↔ test traceability |
| `selftest` | Golden hash plus a SIL smoke run |
| `data` | `fetch elcentro`: downloads the record with a pinned SHA-256; it is never vendored |
| `exo` | Sovereign-tier simulations, admission, PFH, EEW |
| `example` | The gallery |

## Studies

Each study answers one question and writes `output/studies/<name>.{png,json}`. The JSON holds every plotted number.

| Study | Question |
|---|---|
| `pole_map` | How do gap and PM thickness set the unstable pole and the loop rates? |
| `zero_power_band` | Which load range floats at zero current, and what bias current is needed outside it? |
| `fail_direction` | On control loss, where does each class go and how hard does it hit? |
| `drawer_envelope` | Which load and extension combinations can a drawer levitate? |
| `gain_margin` | How much eddy-current lag can the loop tolerate? What is the critical τ_e? |
| `drag_ledger` | How "zero" is zero friction? The honest μ_eq |
| `speed_energy`, `clamp_force` | What speed does mass allow? How does force over time compare with the EN 16005 envelope? |
| `haptic_zwidth` | Which stiffnesses can be rendered, given damping and sample time? |
| `terminus_semilog` | The log-damped straight lines, per class |
| `eew_leadtime`, `seismic_actions` | On-site vs network warning; which actions finish before the S-wave |
| `detector_roc` | False alarms and misses |
| `isolation_montecarlo` | Probability of overloading the WALL's bumpers in isolation (research) |
| `amd_reality` | Does a maglev mass help the building? (research) |
| `stray_field` | Accessible field against the 0.5 mT target |

## Validation and traceability

- **Requirements.** 239 requirement IDs, each naming the test that verifies it:
  - 130 core (`FAB-LEV/PRP/HAP/SAF/SEI/PWR/ENV/FW/SW-NNN`)
  - 16 examples (`FAB-EXA`)
  - 83 exotic (`FAB-EXO`)
  - 10 frontier (`FAB-FRN`)

  `python -m fabrail trace` checks the mapping **in both directions** against `@pytest.mark.req` and Rust `// req:` markers.
- **Analytic anchors.** Tests pin these values:

  | Quantity | Values |
  |---|---|
  | Open-loop poles | 69.49 / 47.57 / 32.50 rad/s |
  | DOOR +98.1 N load step | gap → 4.5967 mm |
  | Speed caps | 0.2654 / 0.1838 / 0.152 / 0.0446 / 0.025 m/s |
  | Stop distances | 33.43 / 36.31 / 8.86 / 4.91 mm |
  | Clamp forces | 290.7 / 503.5 / 173 N |
  | Wu–Zhao Pd | 0.7118 cm |
  | S−P | 0.1190 s/km; crossover 40.8 km |
  | Den Hartog | 0.98039 / 0.08407 |
  | Warburton | 0.97548 / 0.07019 |
  | Transmissibility T(3, 0.2) | 0.1931 |

- **Closed-loop evidence.** On the 6-DOF model in voltage mode: a SIL smoke test, bit-identical replay, and termini meeting FAB-HAP-013…016.
- **Guards:**
  - `.pyi` stub drift
  - docs drift (sliders ↔ presets, RUN blocks ↔ records, formulas ↔ Rust)
  - `no_std` source guards and a dependency allowlist
  - real cross builds
  - a safety-crate dependency-closure check
  - a vocabulary guard

## Project status

The project is built in waves by parallel agents.

| Wave | Contents | State |
|---|---|---|
| 0 | Contracts: workspace, types, HAL, params, schemas, spec IDs, site skeleton | ✅ done |
| 1 / 1.5 | Every module implemented; closed loop on DOOR and DRAWER; firmware links; wasm matches native | ✅ done (608 Rust + 130 Python tests green at W1.5) |
| 2a | CLI, analysis, studies, test suites, guards, PHYSICS body, exotic scenarios, core examples, site pages | ✅ done (failure lists recorded) |
| 2b / 2.5 | Exotic Python layer, sovereign/frontier examples, finishing `villa_quake`; fixing every recorded failure | ⏳ in progress |
| 3 | Freeze the golden hash and seed-42 anchors; generate site data and figures; final docs | ⏳ pending |
| 4 | Six-lens adversarial review and fixes | ⏳ pending |

**Known open items:**
- HEAVY roll–sway on the 6-DOF tier. A downgrade to 🔶 is possible; HEAVY acceptance runs on Heave1/Travel1d.
- DRAWER slot exit at the rated 48 kg.
- Floor-driven motion of free devices under strong shaking. Physical end stops are being added.

The authoritative ledger is [capability-matrix.md](docs/capability-matrix.md).

## Limitations

- **Force model.** The reluctance/co-energy model omits leakage and fringing; real designs need 1.2–1.5× more magnet. Eddy lag and rail drag are heuristic, and nothing is FEM-validated.
- **Standards editions.** Values come from the editions stated: EN 16005 from the 2011 draft, A156.10 from 2005. IEC 60335-2-116 drawer limits are not public.
- **No physical measurements.** There are no acoustic dB(A) figures and no stray-field measurements. WALL and HEAVY need shrouds to meet 0.5 mT.
- **Drawers.** They don't fit a 13 mm side-mount. After a fail-down, a drawer heavier than about 20 kg needs a manual re-seat.
- **Seismic data.**
  - There is no live ShakeAlert or JMA feed; that requires a licence.
  - Warning near the source is ≈ 0 s.
  - Real records (El Centro, STEAD) are fetched, never vendored. The OpenSees "elCentro" file is actually Borrego 1968.

## Documentation map

| Document | Contents |
|---|---|
| [NARRATIVE.md](docs/NARRATIVE.md) | The why: story beats, user journeys, the sovereign chapter, honest limits |
| [SPECIFICATION.md](docs/SPECIFICATION.md) · [SPECIFICATION_EXAMPLES.md](docs/SPECIFICATION_EXAMPLES.md) | Requirements with sources, methods and verifying tests |
| [TECHNICAL_DESIGN.md](docs/TECHNICAL_DESIGN.md) | Architecture, timing, numerics, HAL, safety architecture, firmware |
| [PHYSICS.md](docs/PHYSICS.md) | The model exactly as implemented (§1–§26) |
| [SAFETY.md](docs/SAFETY.md) | Hazards, FMEA-lite, safety functions, what simulation can and cannot establish |
| [SEISMIC.md](docs/SEISMIC.md) | The three readings, stated honestly |
| [EXOTIC.md](docs/EXOTIC.md) · [FRONTIER.md](docs/FRONTIER.md) | Sovereign tier; 🔭 what-ifs |
| [capability-matrix.md](docs/capability-matrix.md) | The honest ledger: status plus maturity |
| [PARAMETERS.md](docs/PARAMETERS.md) | Every parameter with unit, range and source (generated) |
| [references.md](docs/references.md) | Numbered sources, each noting why it matters |

## License

MIT. See [LICENSE](LICENSE).
