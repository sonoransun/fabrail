# fabrail specification

This is the authoritative requirement set for the fabrail core tier: the DRAWER, DOOR, WALL and HEAVY rail classes, the control stack, the safety monitor, the seismic subsystem, the firmware target and the tooling. Three companion documents use the same grammar and are parsed by the same traceability guard:

- [SPECIFICATION_EXAMPLES.md](SPECIFICATION_EXAMPLES.md): the example gallery and the flagship `villa_quake` acceptance (FAB-EXA-###).
- [EXOTIC.md](EXOTIC.md): the Sovereign/Exotic tier (FAB-EXO-###).
- [FRONTIER.md](FRONTIER.md): the 🔭 frontier tier (FAB-FRN-###).

Physics is specified in [PHYSICS.md](PHYSICS.md) and cited as "PHYSICS §n". The safety case is in SAFETY.md, the seismic readings in SEISMIC.md and the architecture in TECHNICAL_DESIGN.md. [NARRATIVE.md](NARRATIVE.md) tells why the requirements are what they are, for a reader who never opens the code. Bracketed tags such as [R1] cite the numbered sources in [references.md](references.md).

**Reading a row.** Each row states one "shall" with its numbers inline, the classes it binds, why it exists (Source), how it is verified (V), the tests that verify it (Verified by) and fabrail's own verification status (St.). The status describes the simulation and code in this repository, never a built rail: nothing here has run on hardware, and no row claims compliance or certification (§4).

> **Status: Wave 1.** Requirement IDs, verification methods, statuses and test tokens have been frozen since Wave 0, and the prose of §0, §1, §4 and §5 is complete. Statuses stay provisional until the traceability gate runs every listed test: a row whose tests do not pass is downgraded (for example V to S or R) in the same edit as the capability matrix, never left overstated. A change to a row changes the spec, the test markers and the capability matrix in the same edit, and is followed by `python -m fabrail trace`.

---

## §0 Conventions

### §0.1 Normative words

- **shall**: a requirement. Every "shall" is a row in a §3 table and is verified by the method in its V column.
- **should**: a design goal. It is reported but never gated.
- "Checked in simulation against numeric limits from [edition]" is the only wording used for standards. Nothing in this repository claims compliance or certification (§4).

### §0.2 Requirement ID grammar

- IDs have the form `FAB-<AREA>-<NNN>`. `<AREA>` is 2–3 capital letters and `<NNN>` is exactly three digits.
- Areas in this document, in table order: `LEV` levitation, `PRP` propulsion, `HAP` haptics, `SAF` safety, `SEI` seismic, `PWR` power, `ENV` environment, `FW` firmware, `SW` software and tooling.
- Areas in the companion documents: `EXA` (SPECIFICATION_EXAMPLES.md), `EXO` (EXOTIC.md), `FRN` (FRONTIER.md).
- IDs are never renumbered or reused. A retired row keeps its ID, strikes through its Shall text (`~~…~~`), carries no test token, and has status M with method I.
- Within an area, rows appear in ascending numeric order. Gaps are allowed.

### §0.3 Table columns

Every requirement table in the four documents has exactly these seven columns:

`ID | Shall | Class | Source | V | Verified by | St.`

| Column | Content |
|---|---|
| ID | The full ID, e.g. `FAB-LEV-001`. The row regex is `^\|\s*(FAB-[A-Z]{2,3}-\d{3})\s*\|` |
| Shall | The requirement, compressed. It contains no pipe character; magnitudes are written `abs(x)` |
| Class | Where the requirement applies: `all`, a class name, or a `/`-separated list (DRAWER/DOOR/WALL/HEAVY here; exotic class names or a safety regime such as GuardedZone in EXOTIC.md; `villa` or `gallery` in SPECIFICATION_EXAMPLES.md). `—` means not class-specific (tooling) |
| Source | Why the requirement exists (§0.6) |
| V | Verification method(s), comma-separated, from §0.4 |
| Verified by | Test tokens (§0.7), separated by `;`, or `—` when none exists (status M only) |
| St. | Status (§0.5) |

### §0.4 Verification methods

| Code | Method | Meaning |
|---|---|---|
| A | Analysis | A closed form or derivation, asserted by a test at the stated tolerance class |
| S | Simulation | A closed-loop run of the plant model through the real control code (SIL) |
| T | Test | A unit, property or logic test of the code itself |
| I | Inspection | A guard that reads source, manifests or documents (grep, AST, drift, traceability) |
| D | Demonstration | Hardware demonstration. Always roadmap in this repository |

### §0.5 Status and badges

The status is fabrail's own verification status. It renders as the house capability-matrix badge.

| Status | Badge | Meaning |
|---|---|---|
| V | ✅ | Verified in simulation (or by exact analysis) against the stated tolerance |
| V(I) | ✅ (inspection) | Verified by an inspection guard, not by simulation. Used for every method-I row |
| S | 🔶 | Verified on a simplified model; the simplification is named in PHYSICS or the row |
| R | 🧪 | Research tier: implemented and tested, but not a product capability |
| M | 🗺️ | Roadmap: not built. No test token; method D or I |

- **🔭 override.** In EXOTIC.md and FRONTIER.md the badge is the status above, except that 🔭 replaces it whenever the technology's maturity is frontier (theoretical, never a product).
- **Maturity is separate.** Technology maturity (TRL) is its own column in the class tables and the capability matrix. It is never merged into the badge.
- A capability's badge never exceeds the weakest requirement it depends on.

### §0.6 Source codes

| Code | Meaning |
|---|---|
| CRIT C#, G#, R# | Core research critique: corrections C1–C20, gaps G1–G14, risks R# |
| CRIT §3 | The critique's per-class parameter table |
| F1–F19 | Corrections applied in the core design (listed in §2 notes) |
| PLAN §2b.n | Binding review fix n of the approved implementation plan |
| PLAN §3b, §4, §4b | Site, exotic and example sections of the approved plan |
| USER | A binding user decision |
| HOUSE | House conventions shared with sibling projects |
| PHYSICS §n | This repository's physics specification |
| CTRL, SEIS, TD, PA | Control, seismic, technical-design and product-architecture research perspectives |
| Standard names | See §4 for editions and how each is used |
| `[R#]` in prose | A numbered source in references.md, e.g. [R1] for the EN 16005 draft. Not to be confused with the critique's risk codes, written `CRIT R#` |

### §0.7 Test tokens

| Token | Resolves to |
|---|---|
| `py:<stem>::<fn>` | `tests/test_<stem>.py::test_<fn>` |
| `py:<dir>/<stem>::<fn>` | `tests/<dir>/test_<stem>.py::test_<fn>` (e.g. `py:exotic/pfh::two_forms`) |
| `rs:<crate>/<path>::<fn>` | `#[test] fn <fn>` in `crates/fabrail-<crate>/src/<path>.rs`; `<crate>` may contain `-` (e.g. `exo-safety`) and `<path>` may nest modules (`core/levitation/modal`) |
| `rs:<crate>/tests/<file>::<fn>` | `#[test] fn <fn>` in `crates/fabrail-<crate>/tests/<file>.rs` |

- Python tests carry `@pytest.mark.req("FAB-…", …)`. Rust tests carry a `// req: FAB-…` line directly above `#[test]` (or within 5 lines, above `proptest!`).
- A token is extracted from the Verified-by cell with the regex `(?:py|rs):[\w./-]+::\w+`. Backticks and separators are ignored.

### §0.8 Tolerance classes

Every assert message states its value and source, e.g. `"(E) rel 1e-12, PHYSICS §3 eq 3.4"`.

| Class | Meaning |
|---|---|
| (E) | Exact algebra, relative 1e-12 |
| (I) | Integration error, bounded by O((ωh)⁴) and stated per test |
| (S) | Standard clause limit |
| (R) | Research metric |
| (MC) | Wilson 95 % upper bound over a Monte Carlo ensemble |
| (P) | Provisional: tightened after the first run and recorded in CLAUDE.md |

---

## §1 Context, frames and glossary

### §1.1 System context

- A **carriage** hangs under a steel rail on hybrid PM-biased electromagnets (EMS) and is propelled along the rail by a long-stator linear synchronous motor (LSM).
- Zero-power control drives the coil current to zero at equilibrium, so the permanent magnets carry the static weight.
- The **controller** is a `no_std` Rust core (`fabrail-core`) with an independent integer **safety monitor** (`fabrail-safety`). Both run on one executive (`fabrail-rt`) that is identical in simulation (SIL) and on the STM32G474 firmware target.
- The **plant** (`fabrail-plant`) is an f64 physics model that only the simulator uses.
- **Node roles.** Version 0.1 has one combined node: carriage sensing, coil drivers, LSM drive and supervisor share one MCU. Separate carriage and track nodes are roadmap (§5).
- **Rest in contact.** DRAWER rests on its captive seat, WALL and HEAVY rest landed. Only DOOR hovers at rest. "Contactless" describes nominal motion, never rest.

The README's own words, quoted:

> Fabricated Zero Perfect Friction Rails System
>
> fabrail implements a design using compact magnetic suspension and lateral propulsion using precision controlled magnetic field components for use in cabinetry, sliding doors, movable infrastructure.
>
> Programmatic control provides the most tactile experience with intuition, extending into technical mastery of momentum control for large infrastructure components mitigating seismic forces through pressure wave effects.

**System boundary.** The specified system is one rail with one carriage and its node.

| Direction | Signals |
|---|---|
| Inputs | Hand force on the carriage; payload; floor acceleration at the rail in x, y and z; the structure-mounted seismic accelerometer; network alerts (an interface plus a synthetic injector in this repository); supply voltage; commands (open, close, stop, reset, hold-to-run); injected faults in simulation |
| Outputs | Coil voltages or currents; LSM phase currents; STO, ASC and brake demands from the monitor; the landing and seat sequences; telemetry frames and events |
| Outside the boundary | The building and its structure, egress and fire-door design, the presence-sensing hardware (its safety-rated output is an input), and any live early-warning service |

**What is built and how it is verified.**

- **Plant tiers** (PHYSICS §5). Carriage6 is the full rigid-body model for closed-loop acceptance. Heave1 carries anchors, gain studies and thermal runs. Travel1d carries seismic Monte Carlo, end-zone and isolation runs. Structure carries the research-tier AMD/TMD studies.
- **Control and safety code.** The same `fabrail-core`, `fabrail-safety` and `fabrail-rt` code runs in simulation (SIL), in the browser engine (wasm32) and in the STM32G474 firmware build. SIL ≡ firmware is claimed in voltage coil mode only (FAB-FW-006).
- **Companion tiers.** The example gallery (SPECIFICATION_EXAMPLES.md), the sovereign tier (EXOTIC.md) and the 🔭 frontier tier (FRONTIER.md) reuse this grammar. None of them changes a core row, class or pedestrian cap (FAB-EXO-006).

### §1.2 Frames and signs

| Symbol | Meaning |
|---|---|
| x | Travel along the rail. +x opens. This is the README's "lateral propulsion" |
| y | Cross-track. In code and docs, "lateral" always means y |
| z | Up. g > 0 is gravity's magnitude |
| gap g | Distance from pole face to rail underside, > 0. A rising carriage closes the gap |
| pitch, roll, yaw | Rotations about y, x and z |
| Tick | One 20 kHz base period, 50 µs. Time is `Tick(u64)`, never an absolute f32 |
| PosNm | Absolute x position as i64 nanometres |

### §1.3 README words, read precisely

| README phrase | Meaning in this specification |
|---|---|
| `zero perfect friction` | A direction, not a claim: modelled μ_eq ≈ 1e-3–1e-2 (FAB-HAP-010) |
| `lateral propulsion` | Travel along x by the LSM (FAB-PRP-001) |
| `momentum control` | The energy and momentum governor with m_gov = max(m_rated, 1.05·m̂) (FAB-SAF-001) |
| `pressure wave effects` | The seismic P-wave, which arrives before the damaging S-wave (SEISMIC.md). It is never a force the rail exerts |
| `compact magnetic suspension` | Hybrid PM-biased EMS elements sized by `derive::size_magnets` (§2; FAB-LEV-002) |
| `precision controlled magnetic field components` | The coil current loop (ω_c = 2π·1.0 kHz on the 20 kHz tick), field-oriented LSM control and an exact electrical angle at any distance (FAB-LEV-008, FAB-PRP-009) |
| `most tactile experience with intuition` | Admittance haptics, intent detection, detents, soft-close and log-damped termini (FAB-HAP-001, FAB-HAP-005, FAB-HAP-013) |
| `large infrastructure components` | WALL (500 kg) and HEAVY (5 t) here; the sovereign classes in EXOTIC.md |
| `mitigating seismic forces` | Land-first class actions triggered by the P-wave (FAB-SEI-010). Isolation and mass damping are research tier (FAB-SEI-014, FAB-SEI-016). The rail reduces no force on the building |

NARRATIVE.md ("Reading the README's words") tells the same reading as prose.

### §1.4 Glossary

| Term | Definition |
|---|---|
| g*, m* | Zero-power gap and the payload mass the magnets balance at g* |
| g_up, g_land, g_seat | Upper stop gap, landing (roller) gap, DRAWER seat gap |
| m_rated, m_op, m̂, m_gov | Rated mass, operating range, estimated mass, governor mass max(m_rated, 1.05·m̂) |
| p | Open-loop (Earnshaw) heave pole of one magnet, rad/s |
| τ_e, τ_cl | Eddy-current lag of the force flux; closed current-loop lag |
| ZOH, LET | Zero-order hold; logical execution time (a job released at tick r publishes at r + P) |
| M_v, B_v, K_v | Rendered (virtual) mass, damping and stiffness of the admittance controller |
| F̂, residual | Estimated hand force; model residual used for obstruction detection |
| SS1, STO, ASC | Safe stop 1 (controlled stop then torque off), safe torque off, active short circuit. STO and ASC act on the LSM only |
| SLP | Safely limited position |
| PL, Cat 3 intent | ISO 13849-1 performance level; "Cat 3 intent" means single-MCU software diversity, not hardware redundancy |
| Terminus | The log-damped final approach: distance-to-go decays as d_b·e^(−t/τ), then a quintic lands at v = a = 0 (PHYSICS §16.1) |
| d_ε, τ_min | Terminus hand-over distance (≈ 0.2 mm); smallest admissible terminus time constant |
| T_sky, f_c | Skyhook transmissibility; inertial-smoothing crossover (≈ 1 Hz) (PHYSICS §16.2) |
| STA/LTA | Short-term over long-term average trigger on the band-passed acceleration |
| Pd, τc | Peak P-wave displacement and characteristic period in the first seconds |
| PGA, gal | Peak ground acceleration; 1 gal = 1 cm/s² |
| t_P, t_S, S-ETA | P and S arrival times; estimated S arrival |
| EEW | Earthquake early warning |
| SafetyRank | The exotic-tier order of device states used by the alert-motion rule (EXOTIC.md) |
| μ_eq | Equivalent friction coefficient: levitated drag divided by m·g (PHYSICS §9). Modelled, never 0 |
| PGD | Peak ground displacement; a free levitated mass drifts by about this much relative to the floor |
| Rest policy | What a class does when idle: SEAT (DRAWER), HOVER (DOOR), LAND (WALL), LAND plus parking lock (HEAVY) |
| Presence sensing (PL d) | Safety-rated sensing that stops motion before contact; required above the unsensed speed caps |
| Hold-to-run | Motion only while an operator holds the control; HEAVY's unsensed mode |
| Carriage6, Heave1, Travel1d, Structure | The plant-model tiers (PHYSICS §5) |
| SIL (in this document) | Software in the loop: the real control code run against the plant model. A safety integrity level is always written with its number, e.g. "SIL 3", and appears only in EXOTIC.md as a design target |

---

## §2 Design basis per class

Corrected values. Every number is produced by `fabrail-params` (`presets`, `derive`) and rendered into PARAMETERS.md. Where this table and code disagree, code wins and this table is regenerated.

| Parameter | DRAWER | DOOR | WALL | HEAVY |
|---|---|---|---|---|
| m* / m_rated / m_op | 26 / 48 / 8–48 kg | 100 / 100 / 90–100 kg | 500 / 500 / 450–500 kg | 5000 / 5000 / 4500–5000 kg |
| Product range (`derive::size_magnets`) | 8–48 kg | 40–150 kg | 100–1500 kg | 1.5–10 t |
| Travel (preset) | 0.30 m (contactless ≤ 50 % extension; 50–75 % roller-supported is roadmap) | 1.0 m | 6 m | 12 m |
| Rest policy | SEAT: captive, g_seat 3.5 mm, z clearance ≈ 0.15 mm, x push-stop compliance ≥ 2 mm | HOVER | LAND at park | LAND + parking lock |
| Operating mode | Automatic | Automatic, low-energy | Automatic, unsensed | Hold-to-run; automatic needs PL d presence sensing |
| Magnets / F* each | 4 / 63.6 N | 4 / 245.2 N | 4 / 1226 N | 8 / 6130 N |
| g* / upper stop / landing | 2.5 / 1.2 / 6.5 mm | 5 / 4 / 6.25 mm | 10 / 8 / 12.5 mm | 10 / 8 / 12.5 mm |
| F/F* at stop / at landing | 2.16 / 0.254 | 1.278 / 0.764 | 1.26 / 0.777 | 1.26 / 0.777 |
| Stay-landed ratio F_pm(g_land)/(m_op_min·g) ≤ 0.90 | 0.824 | 0.850 | 0.863 | 0.863 |
| Modified zero-power band | 1.5–6.0 mm (45.8–7.5 kg) | 4.4–5.8 mm | 8.8–11.5 mm | 8.8–11.5 mm |
| B_g / pole w×l / h_m | 0.50 T / 10×32 mm / ≈ 3.28 mm | 0.55 T / 20×51 mm / ≈ 7.7 mm | 0.60 T / 43×100 mm / ≈ 18 mm | 0.60 T / 43×500 mm / ≈ 18 mm |
| M_pm / h′ | 3.23 kA-t / 3.12 mm | 7.59 / 7.33 mm | 17.7 / 17.1 mm | 17.7 / 17.1 mm |
| Open-loop pole p | 69.49 rad/s | 47.57 rad/s | 32.50 rad/s | 32.50 rad/s |
| Coil N / L / R / τ | 200 / 2.0 mH / 1.6 Ω / 1.2 ms | 300 / 6.6 mH / 1.7 Ω / 3.9 ms | 300 / 13 mH / 0.7 Ω / 19 ms | 200 / 29 mH / 0.6 Ω / 48 ms |
| k_i | 7.9 N/A | 19.4 N/A | 41.5 N/A | 138 N/A |
| i_cont / i_pulse / t_pulse (I²t clamp) | 4 / 12 A / 0.2 s | 6.5 / 10 A / 0.2 s | 15 / 20 A / 0.2 s | 22 / 40 A / 0.2 s |
| V_bus | 24 V | 48 V (SELV) | 48 V (SELV) | 300 V |
| Rail flange (min) | 4 mm | 8 mm | 18 mm | 18 mm |
| Rates: current / fast / supervisor / seismic | 20 k / 10 k / 1 k / 100 Hz | 20 k / 10 k / 1 k / 100 Hz | 20 k / 5 k / 1 k / 100 Hz | 20 k / 5 k / 1 k / 100 Hz |
| Levitation design | Discrete design on the ZOH plant with LET delay, τ_cl and nominal τ_e 0.5 ms; ω = 2.5·p; stable for τ_e ≤ 2 ms; laminated or SMC rail assumed | same | same | same |
| Current PI | ω_c = 2π·1.0 kHz (τ_cl = 0.159 ms) | same | same | same |
| Noise force gain | ≈ 0.31 N/µm | `derive::noise_force_gain` | `derive::noise_force_gain` | `derive::noise_force_gain` |
| Gap-sensor noise (in band) | 0.3 µm | 0.5 µm | 2 µm | 5 µm |
| LSM | Ironless Halbach, τ_p 15 mm | Ironless, 25 mm | Ironless segmented, 40 mm | Iron-core segmented, 50 mm |
| Thrust peak / continuous | 90 / 20 N | 150 / 50 N | 400 / 100 N | 2000 / 500 N |
| F_drive_max (≤ 0.75 s) / F_static unsensed / sensed | 90 / 67 / — N | 150 / 67 / 150 N | 400 / 67 / 150 N | 400 / 67 / 150 N |
| Closing: last 50 mm / final (< 5 s) | 150 / < 80 N | 150 / < 80 N | 150 / < 80 N | 150 / < 80 N |
| v_max unsensed | 0.2654 m/s both ways | open 0.1838, close 0.152 m/s | 0.0446 m/s | 0.025 m/s (hold-to-run) |
| KE at cap | 1.69 J | 1.69 / 1.16 J | 0.50 J | 1.56 J (momentum 125 kg·m/s, not capped: hold-to-run is outside A156.10) |
| v_max with PL d sensing | — | open 0.70 [unverified], close 0.22 m/s | 0.15 m/s | 0.10 m/s |
| a_max / jerk | 1.5 m/s² / 20 m/s³ | 0.75 / 5 | 0.15 / 1.5 | 0.08 (0.2 sensed) / 1 |
| d_stop at cap | 33.43 mm | 36.31 mm | 8.86 mm | 4.91 mm |
| Clamp force at 1.69 J | chest 290.7 N, hand 503.5 N, through a 10 N/mm edge 173 N | same | WALL chest 157.7 N | same as DOOR |
| Closing-edge seal k_edge | ≤ 10 N/mm | ≤ 10 N/mm | ≤ 10 N/mm | ≤ 10 N/mm |
| Unattended close (choreographed) | v ≤ min(v_cap, 150 N/√(m_gov·k_series)) = 0.23 m/s at 48 kg, k_series = series(10, 75 N/mm) | same rule | same rule | same rule |
| Position accuracy closed / travel | ±0.3 / ±1 mm | ±0.5 / ±2 mm | ±1 / ±5 mm | ±1 / ±5 mm |
| Intent: force × time or displacement | 4.5 N × 50 ms or 1.5 mm | 10 N × 80 ms or 2 mm | 25 N × 120 ms or 3 mm | 40 N × 150 ms, plus hold-to-run |
| Obstruction residual / persistence | 12 N / 20 ms | 30 N / 30 ms | 60 N / 35 ms | 120 N / 35 ms |
| m/M_v / B_v | 1.5 / 10 N·s/m | 3 / 40 N·s/m | 5 / 450 N·s/m | 5 / 2000 N·s/m |
| b_dist / end-zone c_max (150 mm zone) | 2 / 100 N·s/m | 5 / 300 N·s/m | 20 / 1500 N·s/m | 100 / 8000 N·s/m |
| K_virtual max | 2·b_dist/(T_fast + 2·T_d) (`derive::passivity_k_max`) | same | same | same |
| Bumper rating | ≥ 24 J | ≥ 113 J | ≥ 563 J | ≥ 5625 J |
| Soft-close capture / final speed | 45 mm / ≤ 20 mm/s | 65 mm / ≤ 50 mm/s | 115 mm / ≤ 20 mm/s | 150 mm / ≤ 10 mm/s |
| Terminus τ; d_0 / d_b; t_arrive | 0.15 s; 49.8 / 32.1 mm; 0.98 s + 0.2 s seat | 0.25 s; close 46.1 / 31.6 mm, open 57.6 / 37.2 mm; 1.62 / 1.68 s | 0.6 s; 27.8 / 25.7 mm; 3.56 s | 1.0 s; 25.3 / 24.7 mm; 5.84 s |
| Terminus pole check ω_v/2 ≥ 3/τ | 25 ≥ 20 | 18.8 ≥ 12 | 9.4 ≥ 5 | 6.3 ≥ 3 |
| Inertial smoothing (🧪) | `inertial.enable = false`; f_c ≈ 1 Hz; only for drawers already open ≥ 25 mm + predicted stroke | not offered | not offered | not offered |
| Manual force on rollers (μ 0.003), no-PM bound μ·m·g | 1.41 N (PM-relieved value from `derive::manual_force`) | 2.94 N | 14.7 N | 147 N (not an egress device) |
| Hover / sleep power | ≤ 1.5 W open / ≤ 0.3 W seated | ≤ 3 W / ≤ 0.5 W | ≤ 30 W moving | ≤ 100 W moving |
| Supercap hold-up | 10 F at 12 V (ESR 10 mΩ, η 0.9); ≥ 3× landing energy; open hover ≈ 5 min (≈ 490 J usable at 1.5 W) | ≥ 3× landing energy | ≥ 3× (move + landing) at the dock | ≥ 3× (move + landing) at the dock |
| Fail-down re-lift | from the 6.5 mm rollers only up to 20 kg (i_pulse 12 A); above that a manual push re-seats it | re-lift from rollers | re-lift from rollers | re-lift from rollers |
| Seismic default action | FREEZE, then stay seated or seat | FREEZE → stop → land on rollers, release ASC, never lock; fire-rated never opens; non-rated may open only if stopped by t_S − 1 s | FREEZE → centre only if reachable → land + lock | same as WALL |
| PROTECT rule | Level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal; Level 1 = freeze only | same | same | same |
| Isolation (🧪, opt-in) | — | — | T = max(2 s, 1.1·T_min), T_min with F_cont = 6.3 s; ζ 0.3; ±0.2 m; largely saturates | same rule, computed |
| EEW accelerometer | ADXL355, 500 Hz ODR ÷5 → 100 Hz, mounted on the structure or rail | same | same | same |
| Stray-field target | ≤ 0.5 mT at 10 cm | ≤ 0.5 mT at 10 cm | at risk (16 mT unshielded) | shrouds required (80 mT) |

**Correction notes (F-numbers, as used in the Source column).** F1 DRAWER band clamp with +0.39 A bias at 48 kg. F2 DRAWER elastomer upper rollers ≥ 550 N. F3 WALL 0.0446 m/s (momentum 22.3 kg·m/s). F4 unsensed stall force 67 N everywhere; 150 N only when sensed or in the last 50 mm. F5 HEAVY unsensed is hold-to-run, a_max 0.08 m/s². F6 edge seal brings the hand clamp to 173 N. F7 DRAWER 0.265 m/s both ways. F8 manual force reported relieved and as the no-PM bound. F9 ASC policy. F10 Monte Carlo gate on bumper overload. F11 centring only if reachable. F12 latency split (onset, confirmation, first motion). F13 DRAWER bumper basis ½·48·1² = 24 J. F16 carriage IMU for skyhook and fusion only. F17 DRAWER captive seat (lifting from 6.5 mm needs 27.45 A, above V_bus/R). F18 stay-landed ratio and PayloadMismatch. F19 m_gov rule. Roller μ is 0.003 in every class (PLAN §2b.19).

---

## §3 Requirements

### §3.1 Levitation (LEV)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-LEV-001 | Heave and pitch shall be actively controlled in every class and roll actively on DRAWER and HEAVY; y and yaw are passive (reluctance plus guide); roll is passive (gravity) on top-hung DOOR and WALL | all | Earnshaw; CRIT G6 | A, T | `py:physics_anchors::earnshaw_and_roll_authority`; `rs:plant/magnet::stiffness_sum_nonpositive` | V |
| FAB-LEV-002 | The magnet force shall be the co-energy gradient F_q = −½(Φ/P)²·∂P/∂q, reduce to the closed form at y = 0 and saturate as tanh at B_sat 1.5 T | all | PHYSICS §2 | A, T | `rs:plant/magnet::coenergy_force_consistency`; `rs:plant/magnet::closed_form_y0`; `rs:plant/magnet::saturation_bound` | S |
| FAB-LEV-003 | The open-loop pole shall be p = 69.49 / 47.57 / 32.50 / 32.50 rad/s (closed form (E); simulated fit ±0.5 %) | all | CRIT §3 | A, S | `rs:params/derive::open_loop_pole`; `py:levitation::open_loop_pole_fit` | V |
| FAB-LEV-004 | Zero-power: after a load step abs(i_ss) < 1 mA within 2 s and the gap follows m(g) (DOOR +98.1 N → 4.5967 mm ±2 µm); the +10 % step also raises PayloadMismatch (Severity::Degrade: levitation continues, motion drops to hold-to-run) | all | Mizuno; PLAN §2b.10 | S | `py:levitation::zero_power_load_step` | V |
| FAB-LEV-005 | The zero-power mass shall follow m(g) = m*·((2g* + h′)/(2g + h′))²: DRAWER 1.5 → 45.76, 5.0 → 9.96, 6.2 → 7.12 kg (E) | all | CRIT C9 | A | `py:physics_anchors::zero_power_mass`; `rs:params/derive::zero_power_mass` | V |
| FAB-LEV-006 | Upper-stop force F/(m*·g) ≤ 1.30 (DOOR/WALL/HEAVY); stay-landed F_pm(g_land)/(m_op_min·g) ≤ 0.90 (0.824 / 0.850 / 0.863 / 0.863); DRAWER elastomer upper rollers rated ≥ 550 N; validate rejects violations | all | CRIT C11; F2; F18; PLAN §2b.19 | A, T | `py:physics_anchors::stop_force_ratios`; `rs:params/validate::rejects_bad_stops` | V |
| FAB-LEV-007 | Discrete closed-loop levitation poles (ZOH plant with LET delay, current-loop lag τ_cl and nominal eddy lag τ_e) shall lie within 2 % of the discrete design at ω = 2.5·p with closed-loop ζ ≥ 0.5, and stay stable across the zero-power float band | all | PLAN §2b.1 | A, T | `rs:params/derive::zp_gains_match_polynomial`; `rs:core/tests/levitation_linear::discrete_poles_match_design`; `rs:core/tests/levitation_linear::stable_across_float_band` | V |
| FAB-LEV-008 | The current PI (K_p = L̂·ω_c, K_i = R̂·ω_c, ω_c = 2π·1.0 kHz, back-calculation anti-windup, motional-EMF feedforward) shall, for a small-signal step ≤ 0.3 A with the one-sample duty delay, rise in ≤ 0.3 ms with overshoot ≤ 10 %, and recover from saturation without windup | all | PLAN §2b.2 | T | `rs:core/current::pi_step_response`; `rs:core/current::antiwindup_recovers` | V |
| FAB-LEV-009 | Allocation shall satisfy T·A = I (1e-6 in f32) with null-space currents producing zero net wrench; the warp loop equalises coil currents to ≤ 5 % within 1 s | all | CTRL A.4 | T, S | `rs:core/levitation/modal::allocation_identity`; `rs:core/levitation/modal::nullspace_zero_net`; `py:levitation::warp_loop_equalises` | V |
| FAB-LEV-010 | Under rail irregularity (L/1150 sag, 0.3 mm joint steps, 20 µm rms roughness) at v_max there shall be no stop contact and the gap error shall stay ≤ 0.3·(g* − g_up); WALL/HEAVY traverses are slow-tier | all | CRIT G3; PLAN §2b.13 | S | `py:levitation::rail_irregularity_tracking` | S |
| FAB-LEV-011 | A 20 N lateral push on a top-hung leaf shall engage the guide fin without magnet–rail contact, with x-drag = μN ±5 % and sway ζ ≥ 0.05 from `guidance.c_sway` | DOOR/WALL | CRIT G6; PLAN §2b.16 | S | `py:levitation::top_hung_sway_push` | S |
| FAB-LEV-012 | Lift to levitated shall take: DRAWER unseat ≤ 100 ms, DOOR ≤ 300 ms, WALL ≤ 1 s, HEAVY ≤ 2 s, with no upper-stop contact | all | CRIT G1 | S | `py:levitation::lift_latency` | V |
| FAB-LEV-013 | The payload estimate m̂ shall converge within ±5 % in ≤ 1 s and is tagged non-safety (the governor uses m_gov) | all | CTRL A.5 | S | `py:levitation::payload_estimate_converges` | V |
| FAB-LEV-014 | For +40 K the static gap shift shall be −0.195 / −0.416 / −0.891 mm (DRAWER / DOOR / WALL and HEAVY; rel 1e-9) and the DOOR dynamic shift within ±3 % on Heave1 with a compressed quasi-static ramp, with abs(i) < 5 mA | all | CRIT G4; PLAN §2b.13 | A, S | `py:physics_anchors::thermal_gap_shift`; `py:levitation::thermal_drift_absorbed` | V |
| FAB-LEV-015 | validate shall enforce rail flange thickness ≥ B_g·w/1.5 T | all | CRIT G4 | T | `py:params::rail_saturation_rule` | V |
| FAB-LEV-016 | DRAWER contactless travel shall be limited to ≤ 50 % extension (preset travel 0.30 m); the drawer_envelope study reports maximum contactless extension versus mass and sets the saturation flag where lift capacity is exceeded, asserting no specific feasible point; `derive::extension_capacity` supplies the closed form | DRAWER | PLAN §2b.3 | A, S | `rs:params/derive::extension_capacity_closed_form`; `py:levitation::drawer_extension_envelope` | S |
| FAB-LEV-017 | The DRAWER captive seat at 3.5 mm (z clearance ≈ 0.15 mm, x push-stop compliance ≥ 2 mm) shall allow unseating within i_pulse for 8–48 kg (−4.98 A at 8 kg to +11.23 A at 48 kg), unseat plus slot exit at 48 kg within t_pulse = 200 ms under the I²t clamp, and a rest height independent of load to ±0.05 mm | DRAWER | F17; PLAN §2b.4 | A, S, T | `rs:core/levitation/seat::unseat_sequence`; `py:levitation::drawer_seat_unseat`; `py:levitation::drawer_pulse_budget` | V |
| FAB-LEV-018 | The modified zero-power band clamp (DRAWER 1.5–6.0 mm) shall hold the gap in band with a bias current equal to the closed form ±2 % (+0.39 A at 48 kg) | all | F1 | T, S | `rs:core/levitation/zeropower::bias_current_closed_form`; `py:levitation::modified_zero_power_band` | V |
| FAB-LEV-019 | On control loss at g* the impact speed shall equal `derive::impact_speed` ±5 % with ≤ 2 bounces, and the carriage rests on rollers (or the seat) in either fail direction; a DRAWER that fails down onto its rollers is re-liftable only up to 20 kg (i_pulse 12 A) and otherwise needs a manual push to re-seat | all | CRIT C11; PLAN §2b.4 | A, S | `py:fail_direction::impact_speeds`; `py:fail_direction::rests_on_rollers`; `py:fail_direction::drawer_relift_limit` | V |
| FAB-LEV-020 | The levitation loop at preset gains shall stay stable for eddy lag τ_e ≤ 2 ms in every class; the gain_margin study reports the critical τ_e per class | all | PLAN §2b.1 | A, T | `rs:core/tests/levitation_linear::stable_for_eddy_lag_to_2ms` | V |

### §3.2 Propulsion (PRP)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-PRP-001 | The README's "lateral propulsion" shall mean travel along x, and "lateral" in code and docs shall mean y only; the glossaries in this document and PHYSICS §1 agree | all | CRIT terminology | I | `py:docs_drift::frame_glossary` | V(I) |
| FAB-PRP-002 | The LSM model shall satisfy K_f = √3·K_e and F·v = 1.5·e_q·i_q (rel 1e-9) | all | PHYSICS §7 | A, T | `rs:plant/lsm::kf_ke_and_power_balance` | S |
| FAB-PRP-003 | Thrust peak and continuous ratings shall be ≥ the §2 values and ≥ m_rated·a_max + drag | all | CRIT C4 | A, T | `py:params::thrust_sizing` | V |
| FAB-PRP-004 | The directional clamp shall limit stall force (abs(v) < 2 mm/s for > 0.2 s) to ≤ 67 N unsensed and ≤ 150 N when sensed or in the last 50 mm, reach < 80 N within 5 s, limit driving force to F_drive_max for ≤ 0.75 s, and never clamp braking force | all | EN 16005 (2011 draft); A156.19; F4 | T, S | `rs:core/motion/clamp::directional_clamp_props`; `py:safety::stall_force_clamp` | V |
| FAB-PRP-005 | The 7-segment S-curve shall respect v, a and j limits, and d_stop shall equal the closed form (rel 1e-9; 33.43 / 36.31 / 8.86 / 4.91 mm at the caps) | all | CTRL B | A, T | `rs:core/tests/scurve_props::limits_and_stop_distance`; `py:physics_anchors::d_stop_values` | V |
| FAB-PRP-006 | Position accuracy closed / travel shall be ±0.3 / ±1 mm (DRAWER), ±0.5 / ±2 mm (DOOR), ±1 / ±5 mm (WALL, HEAVY) | all | CRIT §3 | S | `py:propulsion::position_accuracy` | V |
| FAB-PRP-007 | The velocity cascade bandwidth ω_v = 2π×{8, 6, 3, 2} Hz shall be met within ±20 %, with feedforward m̂a + b̂v + ripple table | all | CTRL B | S | `py:propulsion::velocity_loop_bandwidth` | V |
| FAB-PRP-008 | Absolute x shall be `PosNm` (i64); `PosAccum` shall not drift over 15 m (0 nm over 1e6 steps); controller outputs shall be bit-identical under x + L and tick + 2⁴⁰ | all | numerics (TECHNICAL_DESIGN §6) | T | `rs:types/units::posaccum_no_drift_15m`; `rs:core/tests/invariance::offset_invariance` | V |
| FAB-PRP-009 | The electrical angle shall be exact at any distance (x_nm.rem_euclid(2·τ_p)) | all | numerics | T | `rs:core/foc::angle_exact_far` | V |
| FAB-PRP-010 | Regeneration shall raise the bus by ≤ 16 % per low-energy stop, with the brake chopper above that | all | CTRL B | S | `py:propulsion::regen_bus_rise` | S |
| FAB-PRP-011 | 15 m × 20 K of rail thermal expansion shall be absorbed by homing | WALL/HEAVY | CRIT R9 | S | `py:propulsion::thermal_expansion_homing` | S |

### §3.3 Haptics and motion feel (HAP)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-HAP-001 | Apparent mass shall be within ±10 % of M_v across the class mass range (DRAWER 8–48 kg) | all | CRIT G9 | S | `py:haptics::apparent_mass` | V |
| FAB-HAP-002 | validate shall accept m/M_v ≤ 2 (DRAWER), ≤ 5 sensorless and ≤ 20 with a handle load cell, and reject anything else | all | CRIT C5 | T | `py:params::assist_ratio_bounds` | V |
| FAB-HAP-003 | Breakaway force shall be ≤ 1 / 5 / 20 N (DRAWER / DOOR / WALL), reported against 22.2 N | DRAWER/DOOR/WALL | CRIT G9; ADA 404.2.9 | S | `py:haptics::breakaway_force` | V |
| FAB-HAP-004 | The momentum observer's F̂ error shall be ≤ 10 % of peak on 5–25 N ramps over 50–150 ms | all | CRIT G8 | T | `rs:core/haptics/observer::ramp_tracking` | V |
| FAB-HAP-005 | Admittance onset shall be ≤ 10 ms (DRAWER/DOOR) or ≤ 20 ms (WALL/HEAVY) and intent confirmation ≤ 120 / 170 ms, with the §2 thresholds | all | F12 | S | `py:haptics::intent_latency` | V |
| FAB-HAP-006 | There shall be no false intent from: the DRAWER cabinet piston (5.6 N for 50 ms, outward, direction-inconsistent), a 15 N lean rising over ≥ 1 s and held 5 s, a 50 N smoke ΔP ramped over ≥ 2 s, or footsteps; intent requires direction consistency | all | CRIT C15, G8; PLAN §2b.11 | S, T | `rs:core/haptics/intent::lean_rejected`; `py:haptics::false_intent_suite` | V |
| FAB-HAP-007 | Soft-close shall end at ≤ 20 / 50 / 20 / 10 mm/s with overshoot ≤ 1 mm and no bumper contact (the looser bound; FAB-HAP-013 to 016 tighten it for termini) | all | CRIT G9 | S | `py:haptics::soft_close` | V |
| FAB-HAP-008 | Detent peak force shall be within ±10 % of the profile (DRAWER 3 N / 2 mm, DOOR 8 N / 3 mm, WALL 20 N / 5 mm) | DRAWER/DOOR/WALL | CTRL C.3 | S | `py:haptics::detent_peak` | V |
| FAB-HAP-009 | Virtual stiffness shall be limited to K ≤ K_max = 2b/(T + 2T_d); with the passivity controller on, E_obs ≥ −1 mJ; with it off, the violation is reproduced at 3·K_max | all | Colgate; Hannaford; PLAN §2b.19 | A, T, S | `rs:core/haptics/passivity::stiffness_bound`; `rs:core/haptics/passivity::pc_bounds_energy`; `py:haptics::passivity_observer` | V |
| FAB-HAP-010 | μ_eq at 0.1 m/s shall be reported and lie within [1e-3, 1e-2] at defaults, never 0 | all | CRIT G7 | S | `py:physics_anchors::equivalent_friction_band` | S |
| FAB-HAP-011 | The sensor-noise force gain shall equal K = m(a1 + p²) (rel 1e-9) and force noise shall be reported (DRAWER ≈ 0.31 N/µm at ω = 2.5·p) | all | CRIT G10; PLAN §2b.1 | A | `py:physics_anchors::sensor_noise_force_gain` | S |
| FAB-HAP-012 | DRAWER push-to-open from the seat (using the x push-stop compliance) shall detect in ≤ 60 ms, unseat in ≤ 100 ms and show first motion in ≤ 160 ms | DRAWER | F12; F17 | S | `py:haptics::drawer_push_to_open` | V |
| FAB-HAP-013 | During terminus capture the distance-to-go shall decay as d(t) = d_b·e^(−t/τ), so a semilog fit of ln d against t over [t_c + t_b, t_ε] on plant truth or the encoder has R² ≥ 0.999; asserted on Carriage6 devices while abs(a_floor,x) < 0.01 g (PHYSICS §16.1) | all | PLAN §4b | A, S | `rs:core/motion/terminus::exponential_capture_exact`; `py:haptics::terminus_semilog_fit` | V |
| FAB-HAP-014 | The terminus reference shall be C² (constant-jerk blend, exponential capture, then a quintic from (d_ε, −d_ε/τ, d_ε/τ²) with T_q = τ, monotone to v = a = 0); validate shall enforce τ ≥ τ_min = v_app/a_max − a_max/(2j), and at runtime τ_eff = max(τ, τ_min(v_measured)) | all | PLAN §4b | A, T | `rs:core/motion/terminus::reference_is_c2`; `rs:core/motion/terminus::quintic_monotone`; `rs:params/validate::rejects_tau_below_tau_min` | V |
| FAB-HAP-015 | In terminus mode the velocity integrator shall be frozen with bumpless hold (error poles (s + ω_v/2)², ζ = 1, ω_v/2 ≥ 3/τ) and re-enabled at d_ε, giving discrete closed-loop ζ ≥ 0.9; the governor treats the commanded terminus as x_obs = terminus + overtravel and never clips the exponential tail | all | PLAN §4b | A, T | `rs:core/motion/terminus::integrator_freeze_poles`; `rs:core/motion/governor::terminus_not_clipped` | V |
| FAB-HAP-016 | At terminus arrival x − x_term shall not cross zero by more than 0.02 mm, arrival speed on Truth.v_mps shall be ≤ 1 mm/s, and t_arrive = t_b + τ·ln(d_b/d_ε) + T_q shall hold within ±2 % (DRAWER 0.98 s + 0.2 s seat, DOOR 1.62 / 1.68 s, WALL 3.56 s, HEAVY 5.84 s); a hand during capture switches to class admittance and release re-plans from the measured state, and obstruction always overrides the target | all | PLAN §4b | A, S | `rs:params/derive::terminus_t_arrive_table`; `py:haptics::terminus_arrival`; `py:haptics::terminus_hand_replan` | V |
| FAB-HAP-017 | Inertial smoothing (PHYSICS §16.2; `inertial.enable = false` in every preset) shall use skyhook absolute-velocity feedback c_sky = 2ζ·m̂·ω_c from the carriage IMU (leaky integral with a 0.1 Hz high-pass) and be stable in the fast-group discrete-pole test together with the zero-power pitch loop | DRAWER | PLAN §4b | A, T | `rs:core/motion/inertial::skyhook_discrete_stable` | R |
| FAB-HAP-018 | With smoothing on, the residual floor-to-drawer x acceleration shall lie within ±20 % of T_sky(f/f_c, ζ) = 1/√((1 − r²)² + (2ζr)²) (≈ 0.24–0.31 at 2 Hz, ≈ 0.04 at 5 Hz for f_c ≈ 1 Hz) after the stated error terms (IMU noise, g·θ_tilt, parasitic force, latency) | DRAWER | PLAN §4b | A, S | `py:haptics::inertial_residual_transmissibility` | R |
| FAB-HAP-019 | SmoothRideThrough shall be admitted only for a drawer already open ≥ s_min = 25 mm + S_max,pred with v_rel,pred ≤ 0.8·v_cap, stroke and force budgets met (F_cont 20 N; ≤ 90 N for ≤ 0.75 s), a_z,pred ≤ 0.5·p²·(g_float − g_up), mains_ok (or building UPS) and E_sc ≥ 3·E_land + E_seat; seated drawers never unseat on an alert, and every exit, including mains loss, is a log-damped close to the seat at a_brk = 2·a_max before v_lim or any monitor threshold | DRAWER | PLAN §4b | A, T, S | `rs:core/motion/inertial::admission_rules`; `py:haptics::inertial_exit_to_seat` | R |

### §3.4 Safety (SAF)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-SAF-001 | The governor shall limit v ≤ min(V_mode, √(2E/m_gov), p_max/m_gov, √(2·a_brk·(x_obs − x − margin))) − a_brk·t_react with m_gov = max(m_rated, 1.05·m̂); sabotaging m̂ to 0.5·m changes nothing | all | CRIT R3; F19 | T, S | `rs:core/motion/governor::rated_mass_invariance`; `py:safety::governor_sabotage` | V |
| FAB-SAF-002 | Unsensed speed caps shall be DRAWER 0.2654 m/s both ways, DOOR open 0.1838 and close 0.152 m/s, WALL 0.0446 m/s and HEAVY 0.025 m/s hold-to-run (E) | all | A156.19; A156.38; A156.10; F3; F7 | A, T | `rs:params/presets::speed_caps` | V |
| FAB-SAF-003 | Speeds above the unsensed caps shall require PL d presence sensing; HEAVY automatic mode requires PL d, otherwise hold-to-run | all | CRIT C2; F5 | T | `py:params::presence_sensing_gate` | V |
| FAB-SAF-004 | The force–time profile shall stay ≤ 400 N for ≤ 0.75 s, ≤ 150 N, and < 80 N within 5 s, and ≤ 67 N in low-energy (unsensed) modes | all | EN 16005 (2011 draft) | S, T | `rs:safety/force::en16005_timer`; `py:safety::en16005_force_profile` | S |
| FAB-SAF-005 | Clamp force shall follow F = √(2Ek): 290.7 N chest and 503.5 N hand at 1.69 J, 173 N through a 10 N/mm edge seal, WALL 157.7 N; simulation within ±0.5 %; every unsensed cap ≤ 400 N through the edge seal | all | CRIT C3; F6 | A, S | `py:physics_anchors::clamp_force_energy_form`; `py:safety::clamp_force_sim` | V |
| FAB-SAF-006 | An obstruction shall stop the carriage and reverse 50 mm (DOOR/WALL) or release to 0 N (DRAWER/HEAVY); DOOR reversal ≤ 20 ms after detection; residual and persistence per §2 | all | CTRL D.2 | S, T | `rs:core/haptics/obstruction::residual_persistence`; `py:safety::obstruction_stop_reverse` | V |
| FAB-SAF-007 | The braking envelope v_lim(x) shall use a_brk = 2·a_max and t_react = 2 fast periods | all | CTRL B | T | `rs:core/motion/governor::braking_envelope` | V |
| FAB-SAF-008 | The monitor shall demand STO + ASC on the LSM when v > 1.1·v_lim for > 10 ms, with velocity windowed over ≥ 10 ms from integer encoder counts | all | IEC 61800-5-2; PLAN §2b.17 | T, S | `rs:safety/velocity::integer_velocity_exact`; `rs:safety/velocity::overspeed_trips`; `py:safety::overspeed_trips` | V |
| FAB-SAF-009 | On supply loss SS1 Safe Landing shall touch down at ≤ 5 mm/s, then STO the LSM, with hold-up energy ≥ 3× the landing need; DRAWER open hover is supercap-limited (≈ 5 min, ≈ 490 J usable at 1.5 W) and reported | all | CTRL D.2; PLAN §2b.4, §2b.19 | A, S, T | `rs:safety/supply::holdup_energy`; `py:safety::safe_landing_on_supply_loss`; `py:power::holdup_margin` | V |
| FAB-SAF-010 | Manual force after landing shall be ≤ 22.2 N on DOOR; the PM-relieved value and the no-PM bound μ·m·g (roller μ 0.003) are both reported, and WALL is flagged if the bound exceeds 22.2 N | DOOR/WALL | ADA 404.2.9; F8 | A, S | `py:safety::manual_force_after_landing` | V |
| FAB-SAF-011 | ASC shall engage only above v_runaway or during a seismic landing, and be released after stop on egress classes | all | F9 | S | `py:safety::asc_policy` | S |
| FAB-SAF-012 | The integer monitor (KE in u128 or saturating arithmetic, exact at 10 m/s; momentum; SLP from the encoder; m_rated) shall trip on an injected core fault; its current and force checks on integer abs(i_abc) codes trip within 1 fast period; integer results match float within one quantum. ISO 13849 "Cat 3 intent" means single-MCU software diversity only | all | ISO 13849-1 (intent); PLAN §2b.17, §2b.19 | T, S | `rs:safety/energy::integer_matches_float`; `rs:safety/energy::ke_no_overflow_10mps`; `rs:safety/energy::ke_trip`; `py:safety::monitor_catches_core_fault` | V |
| FAB-SAF-013 | A deadline miss, stale heartbeat or non-finite value shall lead to FAULT_REACTION and then a latched FAULT that only Reset clears | all | IEC 61800-5-2 | T | `rs:rt/exec::deadline_miss_faults`; `rs:safety/heartbeat::stale_trips`; `rs:core/supervisor/levstate::fault_latch` | V |
| FAB-SAF-014 | End zones shall be 150 mm and bumpers rated ≥ 24 / 113 / 563 / 5625 J; nominal termini never engage end zones or bumpers | all | CRIT C13; F13 | A, T | `py:params::bumper_energy_rating` | V |
| FAB-SAF-015 | The safety parameter block shall carry a CRC32 plus diverse copies; a mismatch refuses lift | all | IEC 60730-1 (Class B intent) | T | `rs:params/crc::crc_detects_corruption`; `rs:core/supervisor/levstate::refuse_lift_on_crc` | V |
| FAB-SAF-016 | Core shall never lower a safety demand; the demand level only rises within a latch window | all | this specification | T | `rs:rt/arbitrate::arbitration_monotone` | V |
| FAB-SAF-017 | Fuzzed raw inputs shall cause no panic and no NaN output | all | firmware policy | T | `rs:rt/tests/fuzz::no_panic_no_nan`; `rs:safety/tests/no_panic::fuzz_raw_inputs` | V |
| FAB-SAF-018 | m̂ > 1.05·m_rated shall raise PayloadMismatch (Severity::Degrade): levitation continues and motion is hold-to-run only | all | F18; PLAN §2b.10 | S | `py:safety::payload_mismatch` | V |
| FAB-SAF-019 | Declared design targets (PL c/d, Class B, finger-gap geometry, captive guideway) shall be demonstrated on hardware; they are designed toward, not certified | all | ISO 13849-1; EN 16005 | D | — | M |
| FAB-SAF-020 | In ISOLATE the monitor shall replace (never disable) its clamp and KE checks with an explicit IsolateLimits set (stroke envelope, bumper KE, continuous force F_cont); exceeding any limit demands STO + SS1; `derive::t_min_iso` uses F_cont (WALL 6.3 s); SAFETY.md declares this an R-tier deviation | WALL/HEAVY | PLAN §2b.5 | A, T | `rs:safety/envelope::isolate_limits_replace_clamp`; `rs:params/derive::t_min_iso_uses_f_cont` | R |

### §3.5 Seismic (SEI)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-SEI-001 | The EEW accelerometer shall be mounted on the structure or rail and validate shall reject a carriage mount; the carriage IMU is used for skyhook and fusion only | all | CRIT C6; F16 | T | `py:params::accel_mount_rejects_carriage` | V |
| FAB-SEI-002 | The detector front end (÷5 anti-alias, 0.5–20 Hz 4-pole prewarped band-pass, STA/LTA 50/3000 samples, LTA frozen while triggered) shall match the f64 numpy reference to ≤ 1e-4 relative | all | Allen 1978; NMSOP | T | `rs:core/filters::butterworth_prewarp`; `py:seismic_detector::stalta_matches_reference` | V |
| FAB-SEI-003 | The step trigger time shall equal the exact discrete STA/LTA recursion (including the LTA term) to ±1 sample; the continuous estimate τ_sta·ln((K−1)/(K−θ)) = 0.1116 s at K = 16 is reported | all | Allen 1978; PLAN §2b.7 | A, T | `py:seismic_detector::trigger_time_step` | V |
| FAB-SEI-004 | τc of a sinusoid (u and u̇ fed directly over an integer number of periods) shall equal the period ±1 %, Pd shall be linear in amplitude, and Pd shall be recovered within ±20 % on synthetic M6–7 records whose P phase follows Wu–Zhao at hypocentral R | all | Wu & Kanamori; Wu–Zhao; PLAN §2b.6–7 | T, S | `rs:core/seismic/pdtc::tauc_sinusoid`; `rs:core/seismic/pdtc::pd_linear_in_amplitude`; `py:seismic_detector::pd_recovery` | S |
| FAB-SEI-005 | Levels shall follow the calibration matrix (τc ≥ 0.6 s and Pd ≥ 0.5 cm → L3; τc only → L2; Pd only → L1), and PROTECT fires when Level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal, with Level 1 meaning freeze only | all | P-Alert; Wu–Kanamori; PLAN §2b.6 | T | `rs:core/seismic/discriminate::alert_levels`; `rs:core/seismic/discriminate::protect_rule` | S |
| FAB-SEI-006 | 24 h of site noise through DetectorReplay (10-minute chunks) shall produce 0 PROTECT, with the raw-trigger count reported (slow); the PGA bypass fires under actuator veto | all | SEIS §6.1, §6.11; PLAN §2b.13 | S, T | `rs:core/seismic/discriminate::veto_bypass_pga`; `py:seismic_detector::noise_24h_no_protect`; `py:seismic_detector::pga_bypass_under_veto` | V |
| FAB-SEI-007 | On a network alert the S-ETA shall be within ±1 s, class actions shall start before t_P, and a cancel returns to NORMAL | all | SEIS §6.5–6 | T, S | `rs:core/seismic/fusion::eta_from_alert`; `py:seismic_policy::network_alert_eta_and_cancel` | S |
| FAB-SEI-008 | The seismic FSM (NORMAL → TRIGGERED → PROTECT → SHAKING → QUIET 30 s → INSPECT → NORMAL) shall be exhaustive and reversible on cancel, and INSPECT shall need a manual reset | all | SEIS §5 | T | `rs:core/seismic/fusion::fsm_exhaustive` | V |
| FAB-SEI-009 | A local pick shall reach FREEZE (deceleration at a_brk) within ≤ 100 ms | all | PA | S | `py:seismic_policy::freeze_latency` | V |
| FAB-SEI-010 | Class actions shall be land-first: M5 R10 gives no LAND or LOCK in ≥ 95 % of seeds (≤ L1, freeze only); M7 R20 gives L3, PROTECT at t_P + 1.2 ±0.2 s and freeze before t_S; M7.8 R100 (slow) gives ≥ L2 and lands ≥ 8 s before t_S | all | CRIT §3; SEIS §6; PLAN §2b.6 | T, S | `rs:core/seismic/policy::class_policy_table`; `py:seismic_policy::class_actions`; `py:seismic_policy::scenario_m5_r10`; `py:seismic_policy::scenario_m7_r20`; `py:seismic_policy::scenario_m78_r100` | S |
| FAB-SEI-011 | An egress door shall never lock; a fire-rated door never opens; a non-rated door may open only if stopped ≥ 1 s before t_S; it stays manually operable afterwards (ASC released) | DOOR | IBC 1010.3; NFPA 80 | T, S | `rs:core/seismic/policy::egress_door_policy`; `py:seismic_policy::egress_door_eew` | V |
| FAB-SEI-012 | S−P time shall be 0.1190 s/km (V_p 6.0, V_s 3.5 km/s) with a 40.8 km crossover, and Wu–Zhao Pd for M7 at R 20 km shall be 0.7118 cm (rel 1e-4) | all | CRIT C7 | A | `py:physics_anchors::lead_time_crossover`; `py:physics_anchors::wu_zhao_pd` | V |
| FAB-SEI-013 | A free (levitated, unforced) mass shall drift by ≈ PGD ±10 %, the land-first justification | all | CRIT C8 | S | `py:isolation::free_mass_drifts_pgd` | V |
| FAB-SEI-014 | Isolation transmissibility shall satisfy T(3, 0.2) = 0.1931 ±2 % and T(√2) = 1 | WALL/HEAVY | SEIS §2.1 | A, S | `py:isolation::transmissibility` | R |
| FAB-SEI-015 | The isolation Monte Carlo (slow; N = 200, KT/CP, 0.5 g, WALL ±0.2 m with end zones, under IsolateLimits) shall keep the Wilson-95 upper bound of P(bumper overload) < 5 % (k ≤ 3), and report P(end-zone entry) and the ω_f sensitivity | WALL | F10; PLAN §2b.5 | S | `py:isolation::monte_carlo_bumper_overload` | R |
| FAB-SEI-016 | TMD/AMD analysis shall reproduce Den Hartog f 0.98039, ζ 0.08407 at μ = 0.02; Warburton 0.97548 / 0.07019 as a local minimum; the LQR scalar gain K = (a + √(a² + b²q/r))/b (rel 1e-9); the mass ratio is always reported | all | Den Hartog; Warburton | A | `py:tmd_amd::den_hartog_frf`; `py:tmd_amd::warburton_minimum`; `py:tmd_amd::lqr_scalar` | R |
| FAB-SEI-017 | A downward 0.8 g floor pulse on WALL shall saturate the low side of the lift, raise VerticalSaturation, and land the carriage without upper-stop impact above rated speed | WALL | SEIS §6.10; PLAN §2b.12 | S | `py:isolation::vertical_pulse_forces_landing` | V |
| FAB-SEI-018 | Synthetic ground motion shall match the Kanai–Tajimi and Clough–Penzien PSDs within ±15 %, hit the target PGA exactly after scaling, follow the ASCE 7-16 spectrum, and be seed-deterministic | all | CRIT G11; ASCE 7-16 | T | `py:ground_motion::kt_cp_psd`; `py:ground_motion::asce_spectrum`; `py:ground_motion::deterministic` | V |
| FAB-SEI-019 | On power loss at S arrival the carriage shall land, stay captive, and keep manual force within the IBC 1010.3 limit | all | SEIS §6.9 | S | `py:seismic_policy::power_loss_at_s` | V |
| FAB-SEI-020 | Floor amplification shall be 1 + 2z/h | all | ASCE 7-16 Eq. 13.3-1 | A | `py:ground_motion::floor_amplification` | V |
| FAB-SEI-021 | A closed-loop AMD/TMD on the 2-DOF Structure model shall report response reduction against mass ratio μ while honouring force and stroke limits, and show a negligible effect at μ = 0.05 % (a 5 t panel in a 10⁷ kg building) | HEAVY | PLAN §2b.18 | S | `py:tmd_amd::closed_loop_structure_2dof` | R |

### §3.6 Power (PWR)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-PWR-001 | Rest policy defaults shall be SEAT / HOVER / LAND / LAND + lock | all | CRIT G1; F17 | T | `py:params::rest_policy_defaults` | V |
| FAB-PWR-002 | Hover power (coil plus declared electronics) shall be ≤ 1.5 W for an open DRAWER and ≤ 3 W for DOOR; sleep ≤ 0.3 / 0.5 W | DRAWER/DOOR | PA | S | `py:power::hover_power_budget` | S |
| FAB-PWR-003 | Zero-power copper loss inside the band shall be ≤ 5 % of the hover budget | all | Mizuno | S | `py:power::zero_power_copper_loss` | V |
| FAB-PWR-004 | WALL/HEAVY dock energy shall be ≥ 3× (move + landing), shown analytically plus a short simulation | WALL/HEAVY | CRIT G1; PLAN §2b.13 | A, S | `py:power::dock_energy_margin` | S |
| FAB-PWR-005 | Bus voltages shall be 24 / 48 / 48 / 300 V (SELV < 60 V except HEAVY) with slew k_i·V/L ≥ the requirement | all | CRIT C17 | A, T | `py:params::bus_voltage_and_slew` | V |
| FAB-PWR-006 | Each cycle shall report its energy (kinetic, copper, standby) | all | PHYSICS §14 | S | `py:power::cycle_energy_report` | V |

### §3.7 Environment (ENV)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-ENV-001 | The stray field (dipole sum) shall meet ≤ 0.5 mT at 10 cm as a target, with WALL/HEAVY flagged "shroud required" | all | CRIT G13 | A | `rs:plant/stray::dipole_on_axis`; `py:environment::stray_field_budget` | S |
| FAB-ENV-002 | validate shall enforce PWM ≥ 20 kHz | all | CTRL | T | `rs:params/validate::pwm_above_audible` | V |
| FAB-ENV-003 | The DRAWER cabinet piston with a 40 cm² leak shall give 5.6 N ±5 % | DRAWER | CRIT C15 | A, T | `rs:plant/air::sealed_spring_rate`; `py:physics_anchors::piston_pressure` | V |
| FAB-ENV-004 | A demagnetisation proxy (SH grade, 80 °C, −N·i_max) shall be reported | all | PHYSICS §2 | A | `py:environment::demag_margin_proxy` | S |
| FAB-ENV-005 | Acoustic dB(A) and debris behaviour shall be measured on hardware | all | PA | D | — | M |

### §3.8 Firmware (FW)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-FW-001 | The firmware target shall be STM32G474RE / thumbv7em-none-eabihf / RTIC 2.3.1 with pinned crates (rtic 2.3.1, rtic-monotonics 2.2.1, stm32g4xx-hal 0.1.0, stm32g4 0.16, cortex-m 0.7.9, cortex-m-rt 0.7.7, defmt 1.1.1, defmt-rtt 1.3.0, panic-probe 1.0.0) | all | USER; PLAN §5 | I | `py:firmware::manifest_pins` | V(I) |
| FAB-FW-002 | The shared control crates shall be no_std, forbid(unsafe_code), allocation-free and fmath-only (no mul_add, num_traits, cfg(target_arch) or float method math), with a thumbv7em dependency closure inside the allowlist; firmware unsafe code lives only in firmware/src/bsp with SAFETY comments | all | USER; PLAN §2b.15 | I | `py:firmware::nostd_source_guard`; `py:firmware::dependency_allowlist`; `py:repo_contracts::forbid_unsafe` | V(I) |
| FAB-FW-003 | SIL and firmware shall run the same fabrail-rt group code, with no cfg(target_arch) and no sim-only control path | all | USER | I | `py:firmware::no_cfg_divergence` | V(I) |
| FAB-FW-004 | LET shall activate outputs at r + P; the base tick always runs first, and all 3! = 6 orders of the fast, supervisor and seismic groups give one identical output hash | all | PLAN §2b.9 | T | `rs:rt/tests/let_order::permutation_invariant` | V |
| FAB-FW-005 | GOLDEN_SELFTEST_HASH shall be frozen and identical with and without the fault-injection feature; firmware boots SELFTEST against it before the IWDG starts | all | PLAN §2b.15 | T | `rs:rt/tests/golden::selftest_hash_frozen`; `py:determinism::selftest_hash_matches` | V |
| FAB-FW-006 | Recorded HAL inputs shall replay bit-identically in voltage coil mode (SIL ≡ firmware is claimed in voltage mode only) | all | PLAN §2b.16 | T | `rs:sim/tests/replay::bit_identical` | V |
| FAB-FW-007 | The task map in TECHNICAL_DESIGN shall match the firmware #[task] attributes (binds and priorities) | all | TD | I | `py:firmware::task_map_matches_design` | V(I) |
| FAB-FW-008 | Schedulability ΣC/T from cycle estimates shall be ≤ 0.70 (current sum 0.64) | all | Liu–Layland; PLAN §2b.19 | A | `py:firmware::schedulability_budget` | S |
| FAB-FW-009 | The panic policy shall hold: clippy denies unwrap, expect, panic and indexing in core, safety and rt; the monitor step modules deny float arithmetic; the production handler disables outputs, writes a crash record and resets | all | TD; PLAN §2b.15 | I | `py:firmware::panic_policy_lints` | V(I) |
| FAB-FW-010 | The firmware crate shall build in release for thumbv7em-none-eabihf, link with rust-lld and meet its llvm-size flash and RAM budget (built and linked; never flashed) | all | USER; PLAN §2b.14 | T | `py:firmware::firmware_link_size` | V |
| FAB-FW-011 | TelemetryFrame shall be a fixed-size repr(C) frame (168 bytes, no padding) that round-trips through bytes | all | TD | T | `rs:types/telemetry::frame_layout` | V |
| FAB-FW-012 | PWM-synchronous ADC, COMP→FLT < 1 µs, CiA 402, HIL, flashing and DWT cycle budgets shall be demonstrated on hardware | all | RM0440 | D | — | M |
| FAB-FW-013 | Real cross builds shall pass: types, params, hal, core, safety, rt, exo-safety and exo for thumbv7em-none-eabihf, and fabrail-safety and fabrail-exo-safety also for thumbv6m-none-eabi and armv7r-none-eabihf (scripts/check_nostd.sh tier 4) | all | PLAN §2b.14, §2b.18 | T | `py:firmware::cross_check` | V |

### §3.9 Software and tooling (SW)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-SW-001 | The CLI shall provide the subcommands presets, params, scenarios, simulate, compare, ensemble, seismic, detect, study, anchors, trace, selftest, data, exo and example, each with working --help | — | HOUSE | T | `py:cli::subcommands_help` | V |
| FAB-SW-002 | The same seed shall give byte-identical JSON in process and in a subprocess (SHA-256, wall_s excluded), and ensemble and batch results shall be invariant to the worker count | — | HOUSE; PLAN §2b.16 | T | `rs:sim/tests/determinism::same_seed_identical`; `rs:sim/tests/determinism::batch_worker_invariance`; `py:determinism::same_seed_identical_json`; `py:determinism::ensemble_worker_invariance` | V |
| FAB-SW-003 | Seed-42 anchors shall stay within their recorded tolerances (tests/data/anchors_seed42.json) | — | HOUSE | S | `py:acceptance::seed42_anchors` | V |
| FAB-SW-004 | Presets shall validate, and unsafe combinations shall be rejected with "path: reason" | all | CRIT G12 | T | `rs:params/presets::all_validate`; `rs:params/validate::error_paths`; `py:params::presets_validate` | V |
| FAB-SW-005 | Frames and signs shall be x travel (+ opens), y cross-track, z up, g > 0 | all | CRIT G12 | T | `rs:plant/body::sign_conventions` | V |
| FAB-SW-006 | Drift guards shall hold for the _native.pyi stub, PHYSICS § references, PARAMETERS.md and the docs data blocks against the anchors | — | HOUSE | I | `py:stub_drift::stub_matches_native`; `py:docs_drift::physics_section_refs`; `py:docs_drift::parameters_md_current`; `py:docs_drift::run_block_matches_anchors` | V(I) |
| FAB-SW-007 | The traceability guard shall pass: IDs in SPECIFICATION, SPECIFICATION_EXAMPLES, EXOTIC and FRONTIER are well-formed and unique, map bidirectionally to marked tests, and every FAB ID cited elsewhere exists | — | this specification | I | `py:traceability::bidirectional`; `py:traceability::ids_well_formed`; `py:traceability::cross_doc_refs` | V(I) |
| FAB-SW-008 | Performance budgets shall hold (Carriage6 DOOR lag ≥ 10× real time, voltage ≥ 3×, HEAVY ≥ 5×, Travel1d ≥ 1000×), and the non-slow pytest tier shall run in ≤ 120 s with every test ≤ 15 s against a release _native (BUILD_PROFILE) | — | CRIT G5; PLAN §2b.13 | T | `py:acceptance::performance_budget` | V |
| FAB-SW-009 | JSON export shall record every plotted number and round-trip | — | HOUSE | T | `py:export::json_roundtrip`; `py:export::every_plotted_number` | V |
| FAB-SW-010 | README shall begin with the user's two paragraphs verbatim | — | HOUSE | I | `py:docs_drift::readme_verbatim` | V(I) |
| FAB-SW-011 | up.sh shall exclude CLAUDE.md, up.sh, target/, firmware/target/, .venv, output/ and the root data/ while shipping docs/assets/data/, and .gitignore shall carry the house entries | — | HOUSE | I | `py:repo_contracts::up_sh_excludes`; `py:repo_contracts::gitignore_entries` | V(I) |
| FAB-SW-012 | Every site page (docs/*.html) shall resolve all internal links and assets, load no external URL, and carry the theme toggle, reduced-motion handling and the tier-badge legend; frontier content carries banners | — | PLAN §3b | I | `py:site::links_and_assets_resolve`; `py:site::no_external_urls`; `py:site::page_chrome` | V(I) |
| FAB-SW-013 | Site CSS tokens shall equal the plotstyle tokens, the JS FORMULAS fallback shall match the Rust anchors under node, and the generated data files (RUN, EXO_RUN, VILLA_RUN, ANCHORS, TRACE) and read/ pages shall be current | — | PLAN §3b | I | `py:site::css_tokens_match_plotstyle`; `py:site::formulas_match_anchors`; `py:site::data_blocks_current`; `py:site::read_pages_current` | V(I) |
| FAB-SW-014 | The browser engine (fabrail-web on wasm32-unknown-unknown) shall be ≤ 1.5 MB with its SHA-256 recorded; its selftest hash under node shall equal GOLDEN_SELFTEST_HASH, and a wasm step run shall match the native run() trace bit-for-bit over the first 2,000 ticks | — | PLAN §3b | T | `py:site::wasm_size_budget`; `py:site::wasm_selftest_matches_golden`; `py:site::wasm_trace_matches_native` | V |
| FAB-SW-015 | Scenario (fabrail.scenario/1) and scene (fabrail.scene/1) JSON shall parse and validate with "path: reason" errors against the shipped schemas; the coil mode has a single source and a mismatch is rejected; series bytes are channel-major little-endian f64; a scene that locks a device on an egress path is rejected | — | PLAN §2b.16, §4b | T | `rs:sim/scenario::rejects_coil_mode_mismatch`; `rs:sim/scenario::series_bytes_channel_major_le`; `py:scenarios::specs_match_schema`; `py:scenarios::scene_rejects_egress_lock` | V |

---

## §4 Standards applicability

fabrail borrows numeric limits from the documents below to set parameters, controller clamps and test thresholds. It claims no compliance, conformity or certification with any of them. The only wording allowed anywhere in the repository is "checked in simulation against numeric limits from [edition]".

- **Editions.** "Edition used" names the edition whose numbers fabrail uses. Several were read through a draft or a secondary summary, and the reference says which. Editions marked "fixed in SAFETY.md §10" were pinned there; an edition change updates SAFETY.md §10, this table and the parameter sources in the same edit. "To confirm" means no edition is pinned yet: until SAFETY.md §10 pins one, the numbers taken from that document cite the research source of their row (§0.6), not an edition.
- **How a limit enters the code.** A limit becomes a field in `fabrail-params` with a source comment. It is enforced by `params::validate`, a controller clamp or the monitor, and asserted by a test of tolerance class (S) whose message names the document and edition (§0.8).
- **Design targets are not claims.** Performance levels PL c/d, Class B software, finger-gap geometry and the captive guideway are declared targets for a future hardware project (FAB-SAF-019, status M). Simulation cannot establish a PL, a SIL or a PFH_d.

| Document | Edition used | What fabrail takes from it | Requirements |
|---|---|---|---|
| EN 16005 (power-operated pedestrian doorsets) [R1], [R2] | Numeric limits from the 2011 draft, cross-checked against EDSF Guideline 06 | Force–time profile: 400 N ≤ 0.75 s, 150 N, < 80 N within 5 s; 67 N low-energy stall; PL c/d as targets | FAB-PRP-004, FAB-SAF-004, FAB-SAF-019 |
| ANSI/BHMA A156.10 [R4] | 2005 | Full-energy automatic door context; V = 161/W for sliding doors over 160 lb, i.e. m·v ≤ 22.3 kg·m/s (the WALL cap). HEAVY hold-to-run is outside it | FAB-SAF-002 |
| ANSI/BHMA A156.19 [R5] | 2019, fixed in SAFETY.md §10 (numbers read through a secondary summary) | Power-assist and low-energy door limits (1.69 J, 67 N stall) | FAB-SAF-002, FAB-PRP-004 |
| ANSI/BHMA A156.38 [R6] | 2019 | Low-energy sliding doors: 1.69 J kinetic-energy basis, 15 lbf (67 N) stopping force, closing speed | FAB-SAF-002 |
| ISO/TS 15066 [R7] | 2016 | Body-region stiffness and effective mass for F = v·√(m·k) (hand 75 N/mm, 0.6 kg; chest 25 N/mm, 40 kg) | FAB-SAF-005 |
| 2010 ADA Standards §404.2.9 [R8] | 2010 | 5 lbf (22.2 N) operating force for manual operation after landing | FAB-SAF-010, FAB-HAP-003 |
| IBC §1010.3.2 and §1010.3.3 [R9] | 2021, fixed in SAFETY.md §10 | Egress doors manually operable, never locked by the seismic action | FAB-SEI-011, FAB-SEI-019 |
| NFPA 80 [R10] | 2022, fixed in SAFETY.md §10 | Fire-rated doors never open on the seismic action | FAB-SEI-011 |
| ISO 13849-1 [R11] | 2023, fixed in SAFETY.md §10 | PL c/d as design targets; "Cat 3 intent" only | FAB-SAF-012, FAB-SAF-019 |
| IEC 61800-5-2 [R12] | 2016, fixed in SAFETY.md §10 | STO, SS1 and overspeed-monitoring function definitions | FAB-SAF-008, FAB-SAF-013 |
| IEC 60730-1 [R13] | 2013 with amendments 1 (2015) and 2 (2020), fixed in SAFETY.md §10 | Class B-style CRC plus diverse copies (intent) | FAB-SAF-015 |
| ASCE/SEI 7-16 [R16] | 2016 | Design spectrum (S_DS 1.0, S_D1 0.6, T_L 8 s); floor amplification Eq. 13.3-1 | FAB-SEI-018, FAB-SEI-020 |
| ICNIRP static-field guidelines [R17] | 2009 | The 0.5 mT precaution level for implant wearers behind the ≤ 0.5 mT at 10 cm target | FAB-ENV-001 |
| ISO 14117 [R18] | To confirm (read through a secondary summary) | Implant static-field immunity levels (1 mT, 50 mT) | FAB-ENV-001 |
| EN ISO 13854 [R19] | 2019 | Minimum gaps against crushing, for the finger-gap design target | FAB-SAF-019 |
| ASTM E557 [R20] | To confirm | Operable-partition support deflection ≤ 1/8 in over 12 ft (≈ L/1150), the rail-sag amplitude | FAB-LEV-010 |
| IEC 60335-2-116 [R14] | Ed. 2 (2025), scope only | The motorised-furniture product standard; its drawer-mover limits are not public, so none is used | — |
| UL 325 [R15] | — | Entrapment-protection concepts considered; not used | — |

Exotic-tier documents (IEC 61508 [R21], IEC 61784-3 [R22], ISO 13855 [R23], ISO 14119 [R24], ISO 12100 [R25]) are listed with their editions in EXOTIC.md.

---

## §5 Open issues and out of scope

This section lists what the rows above do not settle: decisions still open, work that needs hardware, limits the rows state on purpose, and what fabrail does not attempt. NARRATIVE.md ("Scope, stated honestly") tells the same list as prose.

**Open issues (decisions pending):**
- Editions still marked "To confirm" in §4 (ISO 14117, ASTM E557) are to be pinned in SAFETY.md §10, which already fixes the others; IEC 60335-2-116 is read at Ed. 2 (2025) for scope only.
- `GOLDEN_SELFTEST_HASH` stays a placeholder until the anchors are frozen from the complete core, so FAB-FW-005 cannot pass before then. The seed-42 anchors of FAB-SW-003 are recorded at the same step, and (P) tolerances are tightened then (§0.8).
- Carriage6 closes end to end for DOOR and DRAWER first. WALL and HEAVY acceptance runs on Heave1 and Travel1d, with Carriage6 as a second-tier smoke test. A class that does not close is downgraded in this table and in the capability matrix, never left at V.
- Values flagged [UNCERTAIN] (rail drag c_rail and F_hyst, leakage, contents friction) are swept by studies. The DOOR's sensed opening speed of 0.70 m/s is marked [unverified] in §2 and no row depends on it.

**Not verified here (roadmap, status M):**
- Hardware: flashing, HIL, PWM-synchronous ADC, COMP→FLT latency, DWT cycle budgets, CiA 402 (FAB-FW-012). The firmware is required to build and link for its target within a flash and RAM budget (FAB-FW-010); it has never run on a chip, so timing, memory use and bit-identity on hardware are unmeasured.
- Acoustics (dB(A)) and debris (FAB-ENV-005).
- Certification or declared conformity of any kind, and hardware demonstration of the design targets (FAB-SAF-019).
- A live EEW feed (ShakeAlert, JMA). Only an interface and a synthetic injector exist, and a live feed needs a licensed partnership [R67].
- Multi-carriage rails, curves and switches; separate carriage and track nodes; a diverse second MCU (thumbv6m monitor).
- An FEM or magpylib cross-check of the magnet model [R46], [R47].

**Known limits stated in the rows:**
- The levitation design assumes a laminated or SMC rail. A solid-steel rail's eddy lag can exceed the 2 ms bound (FAB-LEV-020).
- DRAWER contactless travel is ≤ 50 % extension. 50–75 % needs roller support and is roadmap (FAB-LEV-016).
- DRAWER open hover is supercap-limited to about 5 minutes. After that it fails down onto rollers, and above 20 kg it needs a manual push to re-seat (FAB-SAF-009, FAB-LEV-019).
- HEAVY unsensed is hold-to-run at 0.025 m/s. Its 125 kg·m/s momentum is stated, not capped, because hold-to-run lies outside A156.10 [R4] (FAB-SAF-002, FAB-SAF-003).
- A payload above 1.05·m_rated raises PayloadMismatch and drops motion to hold-to-run (FAB-SAF-018).
- The terminus criteria hold on Carriage6 in quiet conditions only, with floor acceleration below 0.01 g (FAB-HAP-013).
- The seismic action is declared not a rated safety function (SAFETY.md, SF-08). Land-first is a policy for the rail, not a claim about the building (FAB-SEI-010).
- Isolation, AMD/TMD and inertial smoothing are research tier, and WALL isolation largely saturates (FAB-SAF-020, FAB-SEI-014/015/016/021, FAB-HAP-017).
- EN 16005 numbers come from the 2011 draft [R1]; IEC 60335-2-116 drawer limits are not public [R14].
- Model fidelity: leakage, fringing, eddy lag, rail drag and hysteresis are uncertain and swept in studies; statuses say S where that matters.

**Out of scope:** building protection of any kind, whole-building levitation [R91], pressure-wave forces, and on-site warning closer than about 20 km [R65] (see SEISMIC.md).

---

## §6 Traceability rules

These rules are implemented by `python/fabrail/trace.py` and gated by `tests/test_traceability.py` (FAB-SW-007).

**Documents scanned.** `docs/SPECIFICATION.md`, `docs/SPECIFICATION_EXAMPLES.md`, `docs/EXOTIC.md`, `docs/FRONTIER.md`.

**Row parsing.**
1. A requirement row is a line matching `^\|\s*(FAB-[A-Z]{2,3}-\d{3})\s*\|`.
2. Its cells are split on `|`. A row has exactly the seven cells of §0.3; a different count is an error.
3. Method codes come from the V cell, split on `,`; each is one of A, S, T, I, D.
4. Tokens come from the Verified-by cell with the regex `(?:py|rs):[\w./-]+::\w+`.
5. Status comes from the St. cell and is one of V, V(I), S, R, M.

**Checks.**
- IDs are well-formed and unique across all four documents. Within an area, IDs ascend.
- The token set in the documents equals the set of marked tests, per ID, in both directions:
  - Python: `@pytest.mark.req(...)` found by walking the test ASTs.
  - Rust: a `// req: FAB-…` line within 5 lines above `#[test]` or `proptest!`, attached to `fn <name>`, gives `rs:<crate>/<path>::<name>`. A marker not attached to a test fails the guard.
  - `// impl:` comments are checked only for dangling IDs.
- V, V(I), S and R rows have at least one token; M rows have none and use method D or I.
- Every method-I row has status V(I) or M.
- Every FAB ID cited in SAFETY, SEISMIC, TECHNICAL_DESIGN, capability-matrix, EXAMPLES, NARRATIVE or code exists.
- Every hazard H-01 to H-21 in SAFETY.md maps to at least one requirement.
- `[R#]` tags resolve in references.md.
- The counts in README and in `docs/assets/data/trace.js` equal the parsed counts.
- Tests that exist only in the slow tier are reported, not failed.

**Vocabulary.** Word guards (FAB-EXO-005 and its gallery extension FAB-EXA-003) skip quoted prompt blocks, meaning Markdown blockquotes and inline code, so that the banned list itself can be written down.
