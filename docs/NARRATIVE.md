# fabrail narrative

This is the "why" document, written for a reader who never opens the code. It follows the README's two paragraphs from ambition to physics to the limits that physics sets, with the numbers inline.

**How to read it.** Every number comes from `fabrail-params` (the presets and their closed forms in `derive`) or from the requirement it cites. `FAB-…` identifiers are requirements in [SPECIFICATION.md](SPECIFICATION.md) and its companions [SPECIFICATION_EXAMPLES.md](SPECIFICATION_EXAMPLES.md), [EXOTIC.md](EXOTIC.md) and [FRONTIER.md](FRONTIER.md). "PHYSICS §n" is a section of [PHYSICS.md](PHYSICS.md). Bracketed tags such as [R26] are entries in [references.md](references.md). Badges follow the capability matrix: ✅ verified in simulation, 🔶 verified on a simplified model, 🧪 research tier, 🗺️ roadmap, 🔭 frontier (a what-if study, never a product). Where the text says what a carriage does ("detected within 60 ms"), the number is the requirement's threshold; whether a threshold is met is recorded by the requirement's status in the specification, not by this prose. Each section ends with the requirements and physics it rests on, then a one-line **honest limit**.

**Contents.** A rail you cannot feel · `Zero perfect friction` is a direction, not a number · Earnshaw says no · Reading the README's words · Four classes, one control stack · Touch as the interface · Momentum is the real limit · The earthquake arrives twice · What maglev can and cannot do in an earthquake · User journeys · Scope, stated honestly.

---

## A rail you cannot feel

The README, in its own words:

> **Fabricated Zero Perfect Friction Rails System**
>
> fabrail implements a design using compact magnetic suspension and lateral propulsion using precision controlled magnetic field components for use in cabinetry, sliding doors, movable infrastructure.
>
> Programmatic control provides the most tactile experience with intuition, extending into technical mastery of momentum control for large infrastructure components mitigating seismic forces through pressure wave effects.

What fabrail builds from those two paragraphs:

- **A carriage that hangs under a steel rail** on hybrid permanent-magnet-biased electromagnets. It floats 2.5 mm below the rail (DRAWER), 5 mm (DOOR) or 10 mm (WALL, HEAVY).
- **A long-stator linear synchronous motor (LSM)** whose travelling field pushes the carriage along the rail without touching it.
- **Four rail classes:** 8–48 kg kitchen drawers, 100 kg sliding doors, 500 kg partition walls and 5 t heavy panels such as stage wagons.
- **One control stack:** a `no_std` Rust core that runs unchanged in the simulator, in the browser (wasm32) and in firmware built for an STM32G474 microcontroller. Beside it runs an independent integer-only safety monitor.
- **A physics simulator** (an f64 plant model), a seismic early-warning chain, a Python layer for scenarios, analysis and the command line, and an interactive site.

"A rail you cannot feel" is the design aim in one line: the hand should sense only what the controller chooses to render, such as a mass, a detent or a soft stop, and never the mechanism. The rest of this document is about how far physics lets that aim go.

*See:* FAB-LEV-001, FAB-PRP-001, FAB-FW-003, FAB-FW-013 · PHYSICS §1, §2, §7 · [R26]

**Honest limit.** "Contactless" describes nominal motion only: the DRAWER rests on a captive seat, WALL and HEAVY rest landed, only the DOOR hovers at rest, and none of it has run on hardware.

---

## `Zero perfect friction` is a direction, not a number

Nothing rubs when the carriage floats, but four things still take energy from it (PHYSICS §9):

- **Rail hysteresis** F_hyst: every pass re-magnetises the steel under the poles. Laminating the rail cuts eddy currents, not hysteresis (EXOTIC §X8).
- **Rail eddy drag** c_rail·v. Both rail terms scale with the square of the pole flux.
- **Deliberate damping** b_dist·v, put there on purpose (see below).
- **Air drag**, negligible at these speeds but kept in the ledger.

The ledger at the reference speed of 0.1 m/s, at the preset design mass m* (rail-drag values are the uncertain defaults that the `drag_ledger` study sweeps):

| Class (m*) | b_dist·v | c_rail·v | F_hyst | Total drag | μ_eq = F/(m*·g) |
|---|---|---|---|---|---|
| DRAWER (26 kg) | 0.20 N | 0.10 N | 0.20 N | 0.50 N | 2.0e-3 |
| DOOR (100 kg) | 0.50 N | 0.50 N | 1.0 N | 2.0 N | 2.0e-3 |
| WALL (500 kg) | 2.0 N | 2.0 N | 5.0 N | 9.0 N | 1.8e-3 |
| HEAVY (5 t) | 10 N | 10 N | 50 N | 70 N | 1.4e-3 |

The drag is set by the rail, not by the load, so a lighter carriage shows a higher μ_eq: an empty 8 kg drawer sees the same 0.50 N as μ_eq = 6.4e-3. Against that, fabrail's own landing rollers are modelled at μ = 0.003, good rollers run at 0.002–0.005, and passive permanent-magnet sliding doors are quoted at μ ≈ 0.004 [R111]. In the ledger, magnetic suspension beats good rollers by a factor of about 1.5 to 3, not by infinity. What it really buys is different: no rolling contact to wear, no stick-slip at breakaway (≤ 1 / 5 / 20 N on DRAWER / DOOR / WALL, FAB-HAP-003), and a drag that is smooth and known, so the controller can cancel it or render something else.

**Why the deliberate damping stays.** A controller renders a virtual spring or detent through a sampled loop, and the sampled loop is passive only while K ≤ 2b/(T + 2T_d) [R50]. With b_dist = 2 N·s/m and T = T_d = 100 µs, the DRAWER can render at most K ≈ 13.3 kN/m. With b → 0, K_max → 0: no stable detent, no firm virtual end stop. Driving friction toward zero and keeping touch stable pull in opposite directions, and fabrail keeps a little damping on purpose (FAB-HAP-009).

**Contact happens on purpose.** The DRAWER rests on a captive seat, the other classes land on rollers (μ 0.003), a bottom guide fin catches sideways pushes on hanging leaves, progressive eddy-current end zones over the last 150 mm and rated bumpers stop overruns, and a carriage that has landed stays landed: the magnets' pull at the landing gap is at most 0.90 of the lightest operating weight (FAB-LEV-006).

**"Zero power" is also a direction.** At equilibrium the coil current is zero and the permanent magnets carry the weight; copper loss inside the zero-power band stays ≤ 5 % of the hover budget (FAB-PWR-003). The electronics still draw power: ≤ 0.3 W for a seated DRAWER and ≤ 1.5 W while it hovers open (FAB-PWR-002).

*See:* FAB-HAP-003, FAB-HAP-009, FAB-HAP-010, FAB-LEV-006, FAB-PWR-002, FAB-PWR-003 · PHYSICS §8, §9, §17 · [R50], [R111], [R44]

**Honest limit.** μ_eq is modelled at ≈ 1e-3 to 1e-2 and never 0, from rail-drag defaults that are uncertain and swept in studies; no drag has been measured on a real rail.

---

## Earnshaw says no

In 1842 Samuel Earnshaw showed that no static arrangement of charges can hold a body stably in every direction [R26]. Braunbek showed that the argument covers paramagnetic and ferromagnetic bodies too, so only diamagnets and superconductors escape it [R27]. In modern form, the stiffnesses of a static magnetic system sum to zero or less, so at least one direction is always unstable [R28]. fabrail asserts that sum on its own magnet model (FAB-LEV-001).

**Where the instability lives.** fabrail uses attractive suspension: magnets on the carriage pull up toward the underside of a steel rail. Closer means stronger, so heave is unstable. The open-loop pole is p = 69.49 / 47.57 / 32.50 / 32.50 rad/s for DRAWER / DOOR / WALL / HEAVY (FAB-LEV-003). A disturbance grows by a factor e every 14.4, 21.0 or 30.8 ms: without control the carriage snaps up to its stop or falls onto its rollers within tens of milliseconds.

**What is controlled.**

- Heave and pitch are active in every class. Roll is active on DRAWER and HEAVY.
- The top-hung DOOR and WALL are roll-stable by gravity: the DOOR's centre of mass hangs 1.05 m below the rail.
- Sideways motion (y) and yaw are passive: magnetic reluctance centres the poles and a bottom guide fin catches larger pushes (FAB-LEV-001, FAB-LEV-011).

**Zero-power hybrid suspension.** The magnets are permanent-magnet-biased electromagnets. Mizuno's zero-power law integrates the coil current and moves the gap until the permanent magnets alone balance the weight; the coil current then settles to zero [R29]. After a load step the steady current is below 1 mA within 2 s (FAB-LEV-004).

**The price is negative stiffness in disguise** [R32]. Under zero-power control, added load moves the carriage *toward* the rail, because a smaller gap is where the magnets can carry more:

- Add 10 kg (98.1 N) to a DOOR and its gap closes from 5.000 mm to 4.5967 mm (FAB-LEV-004).
- A DRAWER floats where m(g) = m*·((2g* + h′)/(2g + h′))²: 45.76 kg at 1.5 mm, 9.96 kg at 5.0 mm, 7.12 kg at 6.2 mm (FAB-LEV-005).
- Unclamped, a 48 kg DRAWER would float at 1.427 mm, only 0.23 mm from its 1.2 mm upper stop, and a 40 K warm-up (−0.195 mm, FAB-LEV-014) would close that. So the DRAWER uses a modified zero-power band of 1.5–6.0 mm, held with a +0.39 A bias at 48 kg [R30] (FAB-LEV-018).

**How the loop is designed.** The gains come from a discrete-time design on the zero-order-hold plant. It includes the one-period output delay, the current-loop lag (τ_cl = 0.159 ms at ω_c = 2π·1.0 kHz) and a nominal eddy lag τ_e = 0.5 ms, with poles at ω = 2.5·p and ζ ≥ 0.5 (FAB-LEV-007, FAB-LEV-008). The loop stays stable for eddy lag up to 2 ms in every class (FAB-LEV-020); at preset gains the critical eddy lag is about 3.1 / 3.9 / 5.1 / 5.1 ms, and the `gain_margin` study reports it. Gap-sensor noise becomes force noise through K = m·(a1 + p²) ≈ 0.31 N/µm on the DRAWER, so 0.3 µm of sensor noise is about 0.09 N of force (FAB-HAP-011) [R40].

*See:* FAB-LEV-001, FAB-LEV-003, FAB-LEV-004, FAB-LEV-005, FAB-LEV-007, FAB-LEV-018, FAB-LEV-019, FAB-LEV-020, FAB-HAP-011 · PHYSICS §2, §3, §15 · [R26], [R27], [R29], [R32]

**Honest limit.** There is no passive levitation: on control loss the carriage lands on its rollers or seat at the speed FAB-LEV-019 bounds, and the design assumes a laminated or SMC rail, because a solid-steel rail's eddy lag can exceed 2 ms.

---

## Reading the README's words

The README is two sentences long, so each phrase carries weight. This is how fabrail reads them.

| README phrase | What fabrail builds | Where |
|---|---|---|
| `Zero Perfect Friction` | A direction: modelled μ_eq ≈ 1e-3 to 1e-2, never 0 | FAB-HAP-010; PHYSICS §9 |
| `compact magnetic suspension` | Hybrid PM-biased electromagnets: four 10 × 32 mm poles on a DRAWER (≈ 3.3 mm of magnet), eight 43 × 500 mm poles on HEAVY | FAB-LEV-002; PHYSICS §2 |
| `lateral propulsion` | Travel along the rail (x) by a long-stator LSM. In fabrail code and documents, "lateral" always means cross-track (y) | FAB-PRP-001; PHYSICS §1, §7 |
| `precision controlled magnetic field components` | A coil current loop at ω_c = 2π·1.0 kHz on a 20 kHz tick, field-oriented LSM control, and an electrical angle exact at any distance (position kept as i64 nanometres) | FAB-LEV-008, FAB-PRP-008, FAB-PRP-009 |
| `cabinetry, sliding doors, movable infrastructure` | The four classes DRAWER, DOOR, WALL and HEAVY | FAB-PWR-001; SPECIFICATION §2 |
| `most tactile experience with intuition` | Admittance haptics: rendered mass, push-to-open, detents, log-damped soft open and close | FAB-HAP-001 to 016 |
| `technical mastery of momentum control` | The energy and momentum governor, m_gov = max(m_rated, 1.05·m̂), with speed caps √(3.38/m) and 22.3/m | FAB-SAF-001, FAB-SAF-002 |
| `large infrastructure components` | WALL (500 kg) and HEAVY (5 t); beyond them, the sovereign tier | EXOTIC.md |
| `mitigating seismic forces through pressure wave effects` | The P (primary, pressure) wave travels about 1.7 times faster than the damaging S wave. fabrail detects it and acts first: freeze, land, lock | FAB-SEI-010; PHYSICS §20 |

Two words collide with engineering usage, and fabrail settles both once:

- **"Lateral."** The README's "lateral propulsion" is along-track travel. Every guidance text uses "lateral" for the cross-track axis, so fabrail writes *travel* or x for propulsion and *lateral* or y for guidance (FAB-PRP-001).
- **"Pressure."** The README's pressure wave is the seismic P wave. The air a closing drawer pushes out of its cabinet (about 5.6 N through a 40 cm² leak) is also a pressure effect, but fabrail treats it as a disturbance the intent detector must ignore (FAB-ENV-003, FAB-HAP-006).

*See:* FAB-PRP-001, FAB-HAP-010, FAB-SAF-001, FAB-SEI-010 · PHYSICS §1 · [R59], [R66]

**Honest limit.** This is fabrail's reading: where the words allow a stronger claim, such as literally zero friction or reduced forces on a building, the documents take the reading that physics supports and say so.

---

## Four classes, one control stack

Every class runs the same `fabrail-core` code on the same rate groups: a 20 kHz current tick, a 10 kHz (DRAWER, DOOR) or 5 kHz (WALL, HEAVY) fast group, a 1 kHz supervisor and a 100 Hz seismic group. The same monitor and executive run in simulation and in the firmware build. A class is a parameter set, a rest policy and an operating mode, all from one `fabrail-params` preset.

### DRAWER: kitchen drawer, 8–48 kg

- Undermount double-acting C-rail; 4 lift magnets of 63.6 N each at the 26 kg design mass; 2.5 mm float gap; 24 V bus.
- Ironless Halbach LSM, 90 N peak and 20 N continuous.
- Rest: **SEAT**, a captive registration seat at 3.5 mm. Travel 0.30 m, contactless up to 50 % extension.
- Unsensed speed 0.2654 m/s both ways (1.69 J at 48 kg). Rendered at m/1.5.
- ≤ 0.3 W seated, ≤ 1.5 W hovering open.

### DOOR: interior sliding door, 100 kg

- Top-hung leaf, centre of mass 1.05 m below the rail; 4 magnets of 245 N; 5 mm gap; 48 V (SELV).
- Ironless LSM, 150 N peak and 50 N continuous.
- Rest: **HOVER**, always levitated for an instant response, ≤ 3 W over a cable loop.
- Low-energy mode: opens at ≤ 0.1838 m/s, closes at ≤ 0.152 m/s. Rendered at m/3.

### WALL: ballroom partition, 500 kg

- Top-hung panel; 4 magnets of 1226 N; 10 mm gap; 48 V (SELV).
- Ironless segmented LSM, 400 N peak and 100 N continuous.
- Rest: **LAND** at park. It levitates only while moving (≤ 30 W) and charges at the dock.
- 0.0446 m/s unsensed, 0.15 m/s with PL d presence sensing. Rendered at m/5.

### HEAVY: stage wagon, archive shelving, hangar panel, 5 t

- Rigid body on bogies; 8 magnets of 6130 N; 10 mm gap; 300 V bus.
- Iron-core segmented LSM, 2000 N peak and 500 N continuous.
- Rest: **LAND plus a parking lock**.
- Hold-to-run at 0.025 m/s unsensed; automatic mode needs PL d presence sensing (0.10 m/s). Rendered at m/5.

**Why the carriage needs power.** The LSM stator is in the rail, but the lift coils, gap sensors and microcontroller ride on the carriage. That is why the rest policy is a class property (FAB-PWR-001). The DOOR hovers on a cable loop. The DRAWER takes power at its seat and runs on a 10 F, 12 V supercapacitor while open: ≈ 486 J usable, 4.4 times the 111 J a safe landing needs (the rule is ≥ 3×, FAB-SAF-009). WALL and HEAVY charge at the dock.

**The DRAWER seat.** The DRAWER is the hardest class.

- Lifting a 48 kg drawer off 6.5 mm landing rollers needs 27.45 A, but 24 V across its 1.6 Ω coil drives only 15 A. So the DRAWER never rests on its rollers.
- It rests on a captive seat at 3.5 mm, held in z both ways with ≈ 0.15 mm clearance, with a spring push-stop of ≥ 2 mm in x so that push-to-open still works.
- From the seat, unseating needs −4.98 A (8 kg) to +11.2 A (48 kg), inside a 12 A pulse allowed for ≤ 200 ms under the I²t clamp. The current at lift-off also gives the payload estimate m̂ from a closed form (FAB-LEV-017).
- The 6:1 load range forces the modified zero-power band (see "Earnshaw says no") and caps contactless travel at 50 % extension, 0.30 m. Beyond that a drawer needs roller support, which is roadmap (FAB-LEV-016).

*See:* FAB-PWR-001, FAB-PWR-002, FAB-LEV-016, FAB-LEV-017, FAB-LEV-019, FAB-SAF-009 · PHYSICS §3, §14, §15 · [R107], [R108], [R103], [R110], [R100]

**Honest limit.** An open DRAWER hovers for only ≈ 5 min on its supercapacitor (≈ 490 J at 1.5 W) before it must land on its travel rollers, and above 20 kg it cannot lift itself off them: a push closed re-seats it (FAB-LEV-019).

---

## Touch as the interface

**Admittance.** The controller estimates the hand's force and moves the carriage as if it were a chosen mass M_v with damping B_v and, where wanted, a spring K_v [R54] [R53]:

M_v·ẍ + B_v·ẋ + K_v·(x − x0) = F̂_h + F_eff

- The rendered ratios m/M_v are 1.5 / 3 / 5 / 5, so a 500 kg wall is felt as 100 kg.
- Apparent mass stays within ±10 % of M_v across the DRAWER's 8–48 kg (FAB-HAP-001).

**Force without a force sensor.** A generalised-momentum observer estimates the hand force from the LSM current and the known dynamics [R56]; on 5–25 N pushes it is within 10 % of the peak (FAB-HAP-004). This is also why sensorless assist stops at about 3–5×: every error in m̂, drag or force ripple is multiplied by the rendered ratio. `validate` accepts m/M_v ≤ 2 on the DRAWER, ≤ 5 without a sensor and ≤ 20 only with a handle load cell (FAB-HAP-002).

**Intent, not accident.** A push counts only if it lasts or moves the carriage in the same direction:

- DRAWER 4.5 N for 50 ms or 1.5 mm; DOOR 10 N for 80 ms or 2 mm; WALL 25 N for 120 ms or 3 mm; HEAVY 40 N for 150 ms, plus hold-to-run.
- Admittance starts within 10 ms (20 ms on WALL and HEAVY) and intent is confirmed within 120 / 170 ms (FAB-HAP-005).
- Rejected as not-intent: the cabinet air piston (5.6 N for 50 ms, outward), a 15 N lean that rises over ≥ 1 s, a 50 N smoke-pressure ramp and footsteps (FAB-HAP-006) [R43].

**Push-to-open from the seat.** The LSM works whether or not the drawer floats, so a push against the seat's spring stop shows up in the encoder and the force residual. The push is detected within 60 ms, the drawer unseats within 100 ms, and first motion follows within 160 ms (FAB-HAP-012), comparable to existing push-to-open drawer hardware [R107].

**Effects and passivity.**

- Detents: DRAWER 3 N over 2 mm, DOOR 8 N over 3 mm, WALL 20 N over 5 mm, each within ±10 % (FAB-HAP-008).
- Soft-close ends at ≤ 20 / 50 / 20 / 10 mm/s with no bumper contact (FAB-HAP-007), replacing the hydraulic dampers of manual sliding hardware [R105].
- Rendered springs must never inject energy. Stiffness stays below K ≤ 2b/(T + 2T_d) [R50]. A passivity observer and controller hold the observed energy at ≥ −1 mJ [R52]; switch them off at 3·K_max and the instability appears (FAB-HAP-009).

### The log-damped terminus

The request for the example gallery was phrased like this:

> "absurdly smooth", "perfectly calculated trajectory", "log dampened soft open/close"

fabrail turns each phrase into something a test can check (SPECIFICATION_EXAMPLES §0.2):

- **Smooth:** a C² reference, peak jerk ≤ j_max, arrival ≤ 1 mm/s and overshoot ≤ 0.02 mm in quiet conditions.
- **Calculated:** a deterministic plan in which every deadline is met or its miss is reported with a reason, and which replays byte for byte.
- **Log-damped:** the logarithm of the distance-to-go is a straight line in time.

**The law** (PHYSICS §16.1):

1. The cruise S-curve blends at constant jerk into an exponential capture, d(t) = d_b·e^(−t/τ).
2. At d_ε ≈ 0.2 mm a quintic takes over and lands at v = a = 0.
3. The arrival time is t_arrive = t_b + τ·ln(d_b/d_ε) + T_q.
4. τ is never below τ_min = v/a_max − a_max/(2j), which is 0.139 s for the DRAWER.

| Class | τ | Capture start d_0 / exponential start d_b | Distance halves every | t_arrive |
|---|---|---|---|---|
| DRAWER | 0.15 s | 49.8 / 32.1 mm | 0.10 s | 0.98 s, then 0.2 s into the seat |
| DOOR (close / open) | 0.25 s | 46.1 / 31.6 and 57.6 / 37.2 mm | 0.17 s | 1.62 / 1.68 s |
| WALL | 0.6 s | 27.8 / 25.7 mm | 0.42 s | 3.56 s |
| HEAVY | 1.0 s | 25.3 / 24.7 mm | 0.69 s | 5.84 s |

**What the tests assert** (FAB-HAP-013 to 016):

- A semilog fit with R² ≥ 0.999.
- x − x_term crosses zero by no more than 0.02 mm.
- Arrival at ≤ 1 mm/s.
- t_arrive within ±2 % of the closed form.

A hand that touches the carriage during capture switches it to plain admittance; on release the terminus is re-planned from the measured state, and a detected obstruction always overrides the target.

*See:* FAB-HAP-001 to 016 · PHYSICS §16.1, §17, §22 · [R54], [R53], [R50], [R52], [R56], [R51]

**Honest limit.** Haptic lightness changes what the hand feels, not the kinetic energy ½·m·v² of the real mass. The terminus figures hold on the full six-degree-of-freedom model in quiet conditions (floor below 0.01 g) and are measured on the plant's true velocity, because a 1 µm encoder cannot resolve a 1 mm/s arrival.

---

## Momentum is the real limit

A carriage rendered at a fifth of its mass still hits with all of it. So the governor never trusts the payload estimate alone: it uses **m_gov = max(m_rated, 1.05·m̂)**. Sabotaging m̂ to half the true mass changes nothing (FAB-SAF-001).

**Two caps apply without presence sensing:**

- **Energy.** Low-energy door practice limits kinetic energy to 1.69 J, so v ≤ √(2·1.69/m) = √(3.38/m) [R5] [R6] [R1].
- **Momentum.** A156.10 limits heavy sliding doors to V = 161/W (ft/s, lb), which is m·v ≤ 22.3 kg·m/s, so v ≤ 22.3/m [R4].

| Class | m_gov | Unsensed cap | KE | Momentum | Stop distance | With PL d sensing |
|---|---|---|---|---|---|---|
| DRAWER | 48 kg | 0.2654 m/s both ways | 1.69 J | 12.7 kg·m/s | 33.43 mm | — |
| DOOR | 100 kg | open 0.1838, close 0.152 m/s | 1.69 / 1.16 J | 18.4 / 15.2 kg·m/s | 36.31 mm | close 0.22 m/s |
| WALL | 500 kg | 0.0446 m/s | 0.50 J | 22.3 kg·m/s | 8.86 mm | 0.15 m/s |
| HEAVY | 5 t | 0.025 m/s, hold-to-run | 1.56 J | 125 kg·m/s | 4.91 mm | 0.10 m/s |

**Why WALL is slow.** At 500 kg momentum binds before energy: the energy cap would allow 0.082 m/s but the momentum cap allows 0.0446 m/s, so a 6 m run takes about 2 min 15 s. With PL d presence sensing, a safety-rated scanner or sensing edge that stops the panel before it reaches a person, it may run at 0.15 m/s, about 41 s (FAB-SAF-003).

**Why HEAVY has no momentum cap.** At 5 t the momentum cap would allow 4.46 mm/s, and 12 m would take 45 minutes. A156.10 governs automatic pedestrian doors, and a 5 t wagon is not one. So HEAVY moves only while a person holds the control:

- It runs at 0.025 m/s, where kinetic energy is 1.56 J (under 1.69 J) and momentum is 125 kg·m/s.
- Acceleration is limited to 0.08 m/s², the 400 N dynamic force limit over 5 t.
- It stops within 4.91 mm.

The research's first instinct was 0.3 m/s for a 5 t wagon: 225 J and 1,500 kg·m/s, 67 times the momentum basis.

**Clamp physics.** A body caught between carriage and frame absorbs the carriage's energy like a spring, so F = √(2E·k) = v·√(m·k) [R7]. At 1.69 J:

- A chest (25 N/mm) sees 290.7 N.
- A hand (75 N/mm) sees 503.5 N, above the 400 N transient limit.
- A 10 N/mm closing-edge seal in series brings the hand down to 173 N (FAB-SAF-005).
- A WALL at 0.50 J gives a chest 157.7 N.

**Force limits and stopping.**

- **Directional clamp.** Stall force is ≤ 67 N unsensed. 150 N is allowed only when sensed or in the last 50 mm of closing, and the force falls below 80 N within 5 s. Driving force stays ≤ F_drive_max for ≤ 0.75 s. Braking force is never clamped (FAB-PRP-004, FAB-SAF-004) [R1].
- **Braking envelope.** The braking envelope assumes 2·a_max, with a reaction time of two fast periods (FAB-SAF-007). S-curve stops match their closed form (FAB-PRP-005).
- **Obstruction.** A residual of 12 / 30 / 60 / 120 N lasting 20–35 ms stops the carriage. DOOR and WALL then reverse 50 mm; DRAWER and HEAVY release to 0 N (FAB-SAF-006).

**An independent monitor.** A separate, integer-only crate watches encoder counts, coil and motor currents and kinetic energy (in u128, exact at 10 m/s). It demands safe torque off and an active short circuit on the LSM when speed exceeds 1.1·v_lim for more than 10 ms (FAB-SAF-008, FAB-SAF-012). It shares the microcontroller with the controller, so its diversity is in software only: "Cat 3 intent" [R11].

*See:* FAB-SAF-001 to 008, FAB-SAF-012, FAB-PRP-004, FAB-PRP-005 · PHYSICS §8, §16, §18, §22 · [R4], [R5], [R6], [R7], [R1], [R12]

**Honest limit.** These limits are checked in simulation against numeric limits from the stated editions (EN 16005 2011 draft, A156.10-2005, A156.38-2019, ISO/TS 15066:2016); nothing is certified, and heavy panels do not move fast without PL d presence sensing.

---

## The earthquake arrives twice

A fault rupture sends out two body waves:

- the **P wave** (primary, compressional: the README's pressure wave) at about 6.0 km/s;
- the **S wave** (secondary, shear) at about 3.5 km/s, which carries most of the damaging horizontal shaking.

The gap between them is S−P = R·(1/3.5 − 1/6.0) = **0.1190 s per km** of hypocentral distance R (FAB-SEI-012).

**Hearing the first arrival** (PHYSICS §20):

1. **The sensor.** An ADXL355 accelerometer on the structure or the rail, never on the carriage, whose own motion would look like an earthquake (FAB-SEI-001) [R122]. It samples at 500 Hz, decimated to 100 Hz.
2. **The trigger.** A 0.5–20 Hz band-pass feeds an STA/LTA ratio over 0.5 s and 30 s, which triggers at 4.0 (FAB-SEI-002, FAB-SEI-003) [R59] [R60].
3. **The size.** The first 1–3 s of P give its peak displacement Pd and characteristic period τc [R61] [R62]. For M7 at 20 km, Wu and Zhao's relation predicts Pd = 0.7118 cm [R63].
4. **The rule.** PROTECT fires when the level is ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal [R64]. Level 1 means freeze only (FAB-SEI-005).

**Lead time.** An on-site decision lands about 1.2 s after P, giving a lead of 0.119·R − 1.2 s. A regional network alert, assumed here 8 s after origin, gives R/3.5 − 8 s.

| Hypocentral R | S−P | On-site lead | Network lead |
|---|---|---|---|
| 10 km | 1.2 s | ≈ 0 s | none (the alert comes after S) |
| 20 km | 2.4 s | 1.2 s | none |
| 40.8 km | 4.9 s | 3.7 s | 3.7 s (crossover) |
| 60 km | 7.1 s | 5.9 s | 9.1 s |
| 100 km | 11.9 s | 10.7 s | 20.6 s |

Under about 20 km the warning is a second or less: enough to freeze (≤ 100 ms, FAB-SEI-009) and little else [R65]. Beyond 40.8 km a regional network warns earlier than any sensor on site. Hence fusion: fabrail takes the earliest credible source and estimates the S arrival to ±1 s (FAB-SEI-007).

**What happens next.** The seismic state machine runs NORMAL → TRIGGERED (freeze) → PROTECT → SHAKING → QUIET (30 s) → INSPECT (manual reset) → NORMAL. A cancelled alert returns to NORMAL (FAB-SEI-008). The class actions are land-first (FAB-SEI-010, FAB-SEI-011):

- **DRAWER:** freezes, then stays seated or seats.
- **DOOR:** freezes, stops, lands on its rollers and releases the motor's short-circuit brake so that a hand can move it. It never locks. A fire-rated door never opens. A non-rated door may open only if it stops at least 1 s before S [R9] [R10].
- **WALL and HEAVY:** freeze, centre only if the centre is reachable in time, then land and lock.

**Evidence in simulation:**

- M5 at 10 km: no LAND or LOCK in ≥ 95 % of seeds.
- M7 at 20 km: level 3, PROTECT at t_P + 1.2 ± 0.2 s, and freeze before S.
- M7.8 at 100 km (slow tier): level ≥ 2, landed ≥ 8 s before S.
- 24 h of site noise: no PROTECT (FAB-SEI-006, FAB-SEI-010).

**A house that closes its own drawers** (the `villa_quake` example, 🧪 overall, a simulated replay with a synthetic alert).

- **The event.** M7.2 at 60 km epicentral distance, 12 km deep: P arrives at 10.2 s and S at 17.5 s.
- **The budget.** The fused alert, confirmed by a local P pick, arrives at ≈ 7.5 s, which leaves 5.7–7.3 s before the per-device deadlines.
- **The plan.** Open drawers close under unattended limits (≤ 0.23 m/s at 48 kg) on log-damped termini. The pantry door stops, lands and never locks. The salon wall lands and locks.
- **The rules.** An alert may move a device only toward a safer state and only in the closing or seating direction. It never opens anything and never reverses a person's motion (FAB-EXA-004, FAB-EXA-006). Close to the source (15 km) a drawer gliding open simply freezes (FAB-EXA-014).

*See:* FAB-SEI-001 to 012, FAB-EXA-004, FAB-EXA-006, FAB-EXA-014 · PHYSICS §19, §20, §21 · [R59], [R63], [R64], [R65], [R67], [R71]

**Honest limit.** There is no useful on-site warning within about 20 km, fabrail has no live ShakeAlert or JMA feed (an alert interface and a synthetic injector only, since live feeds need a licence [R67]), and the seismic action is declared not a rated safety function.

---

## What maglev can and cannot do in an earthquake

**A floating carriage is a free mass along the rail.** It barely feels the floor move beneath it, which sounds ideal until the numbers come in. An unforced free mass drifts relative to the floor by about the peak ground displacement (±10 %, FAB-SEI-013). In a large earthquake that can exceed a drawer's 0.30 m stroke or a wall's end zones.

**Magnets hold little sideways.**

- Passive reluctance and the guide fin give about 0.1–0.3 g of lateral capacity.
- Floors in strong shaking see 0.3–1 g, amplified up the building by 1 + 2z/h, up to 3 times at the roof (FAB-SEI-020) [R16].
- Vertically, a downward 0.8 g floor pulse saturates a WALL's lift. The controller raises VerticalSaturation and lands (FAB-SEI-017).

**Hence land-first.** The default action puts every carriage on its seat or rollers, and locks WALL and HEAVY. During shaking, contact with friction and locks holds a panel; end zones and rated bumpers (≥ 24 / 113 / 563 / 5625 J) are the real stops (FAB-SAF-014).

**Three readings of "mitigating seismic forces":**

1. **P-wave early warning with land-first action** is the core (FAB-SEI-001 to 012).
2. **Active or tuned mass damping** is 🧪.
   - Den Hartog's optimum at a mass ratio μ = 0.02 is a frequency ratio of 0.98039 with ζ = 0.08407 [R80].
   - The first full-scale active mass damper, in the Kyobashi Seiwa building, used about 1 % of the building's mass [R83].
   - A 5 t HEAVY panel in a 10⁷ kg building is 0.05 %, and the closed-loop two-degree-of-freedom model shows a negligible effect at that ratio (FAB-SEI-016, FAB-SEI-021). Mass dampers are mainly wind devices.
3. **Contactless isolation** is 🧪.
   - A spring-mass isolator passes T(3, 0.2) = 0.1931 of the floor acceleration at three times its natural frequency (FAB-SEI-014).
   - Stroke and continuous thrust set T_min = 2π·√(m·s/F_cont): 6.3 s for a 500 kg WALL with ±0.2 m and 100 N, so WALL isolation largely saturates.
   - A Monte Carlo run under explicit isolation limits keeps the Wilson-95 bound on bumper overload below 5 % (FAB-SEI-015, FAB-SAF-020).

**Inertial smoothing for open drawers** is 🧪 and off in every preset.

- A drawer that is already open may follow only the slow floor motion: skyhook feedback above a crossover of about 1 Hz passes ≈ 0.24–0.31 of the floor acceleration at 2 Hz and ≈ 0.04 at 5 Hz (FAB-HAP-017, FAB-HAP-018).
- It is admitted only when stroke, force, power and vertical headroom all fit. Seated drawers never unseat for it, and every exit is a log-damped close to the seat (FAB-HAP-019).
- In the villa example the contents result is worded "no Coulomb slip event in the sliding-block model", and nothing more (FAB-EXA-009, FAB-EXA-010).

**Whole buildings stay on the ground.** Levitating a building is out of scope; Arx Pax's hover foundation is a cautionary tale [R91]. The non-magnetic precedent is air lift, about 3 cm in 0.5–1 s [R90].

*See:* FAB-SEI-013 to 017, FAB-SEI-020, FAB-SEI-021, FAB-SAF-014, FAB-SAF-020, FAB-HAP-017 to 019, FAB-EXA-009, FAB-EXA-010 · PHYSICS §16.2, §21 · [R80], [R81], [R83], [R16], [R91], [R90]

**Honest limit.** fabrail makes no claim about a building's response to an earthquake: isolation, mass damping and inertial smoothing are research tier, and a rail exerts no pressure-wave force.

---

## User journeys

### DRAWER: a kitchen drawer of pans

1. **Asleep.** Closed, the drawer sits on its captive seat with the coils off, drawing ≤ 0.3 W (FAB-PWR-002).
2. **The push.** A cook presses the front: 4.5 N for 50 ms, or 1.5 mm into the spring push-stop, is enough.
3. **The lift.** The push is recognised within 60 ms and the seat lets go within 100 ms. The current needed to lift off tells the controller how much the drawer holds. By 160 ms it is moving (FAB-HAP-012, FAB-LEV-017).
4. **The glide.** It glides out at up to 0.265 m/s and feels like two-thirds of its mass (FAB-HAP-001). Heavier pans make it feel proportionally heavier, but they never let it go faster, because the governor always assumes 48 kg (FAB-SAF-001). A light detent, 3 N over 2 mm, marks a stop.
5. **The close.** Pushed shut, it is captured 49.8 mm from the seat. The remaining distance halves every 0.10 s on a straight semilog line, a quintic takes the last 0.2 mm, and the drawer arrives at ≤ 1 mm/s 0.98 s later, then steps 0.2 s down into the seat (FAB-HAP-013 to 016).
6. **A hand in the way.** A 12 N residual lasting 20 ms releases the drive to 0 N (FAB-SAF-006).
7. **Power fails while it is open.** The supercapacitor lands it at ≤ 5 mm/s on its travel rollers, where at most about 1.4 N moves it by hand. Above 20 kg it cannot re-lift itself; a push closed re-seats it (FAB-SAF-009, FAB-LEV-019).

**Honest limit.** The DRAWER is the hardest class: contactless only to 50 % extension, about 5 min of open hover on the supercapacitor, and a manual re-seat above 20 kg.

### DOOR: an office sliding door

1. **At rest.** The door hovers at its closed position on ≤ 3 W over a cable loop, so it answers at once (FAB-PWR-002).
2. **The push.** A push of 10 N for 80 ms, or 2 mm, starts assist. The 100 kg leaf is rendered at a third of its mass (FAB-HAP-001, FAB-HAP-005), and a detent (8 N over 3 mm) marks half-open.
3. **Automatic moves** stay in low-energy mode: open at ≤ 0.1838 m/s, close at ≤ 0.152 m/s (FAB-SAF-002).
4. **Someone steps into the closing leaf.** A 30 N residual for 30 ms stops the door, and it reverses 50 mm within 20 ms of detection (FAB-SAF-006). Even at the cap, a chest would see at most 290.7 N (FAB-SAF-005).
5. **The close** ends on a log-damped terminus that starts 46.1 mm out and arrives 1.62 s later.
6. **Power loss.** A safe landing onto rollers at ≤ 5 mm/s; the door then moves by hand with about 0.7 N (2.9 N if the magnets gave no relief), against the ADA's 22.2 N (FAB-SAF-009, FAB-SAF-010) [R8].
7. **A seismic alert.** It freezes, stops, lands, releases its motor brake and never locks (FAB-SEI-011).

**Honest limit.** The obstruction case is checked in simulation against numeric limits from EN 16005 (2011 draft) and is 🔶 in the gallery; no conformity is claimed.

### WALL: a ballroom partition

1. **Between uses.** The 500 kg panel rests landed at its park position and charges at the dock.
2. **The push.** An attendant pushes 25 N for 120 ms, or 3 mm. The panel lifts within 1 s (FAB-LEV-012) and moves as if it weighed 100 kg.
3. **The run.** Without presence sensing it creeps at 0.0446 m/s, the momentum cap, and its 6 m run takes about 2 min 15 s. With a PL d presence-sensing system fitted it may run at 0.15 m/s, about 41 s (FAB-SAF-003).
4. **Closed.** It seals to ±1 mm at the closed position (FAB-PRP-006), draws ≤ 30 W while moving, and lands again at park.
5. **An alert.** It freezes, centres only if it can reach the centre in time, then lands and locks.

**Honest limit.** Its unshielded stray field is about 16 mT at 10 cm, far above the 0.5 mT target, so a shroud is required (FAB-ENV-001) [R17].

### HEAVY: a stage wagon or archive carriage

1. **Moving.** The 5 t carriage moves only while someone holds the control, at 0.025 m/s: 12 m take 8 min. With PL d presence sensing, automatic mode at 0.10 m/s takes 2 min (FAB-SAF-003).
2. **The lift.** It lifts within 2 s (FAB-LEV-012). Pushed by hand, 40 N for 150 ms with the control held, it is rendered at a fifth of its mass.
3. **Stopping.** Its iron-core LSM on a 300 V bus stops it within 4.91 mm from full speed. Its bumpers are rated ≥ 5625 J (FAB-SAF-014).
4. **Parked.** It parks landed with a parking lock.
5. **An alert.** It freezes, lands and locks.

**Honest limit.** HEAVY is an attended machine, not a pedestrian door: once landed it can take up to 147 N to push by hand, and its iron-core stator needs stray-field shrouds (about 80 mT unshielded).

*See (all journeys):* FAB-HAP-001, FAB-HAP-005, FAB-HAP-012, FAB-LEV-012, FAB-SAF-002, FAB-SAF-003, FAB-SAF-006, FAB-SAF-009, FAB-SAF-010, FAB-SEI-011, FAB-ENV-001 · PHYSICS §14, §16.1, §17, §23 · [R8], [R17], [R107]

**Honest limit.** Every journey above is a simulated scenario with preset parameters; none has been walked through on a built rail.

---

<!-- The sovereign and frontier chapter (unit X9) is merged at the marker below by unit C2b. -->
<!-- CHAPTER:SOVEREIGN -->

---

## Scope, stated honestly

This mirrors the README callout and the non-claims of the specification, SAFETY.md, SEISMIC.md and the capability matrix.

- **Friction.** μ_eq is modelled at ≈ 1e-3 to 1e-2, never zero. "Contactless" holds in nominal motion only; the DRAWER rests on its seat, WALL and HEAVY rest landed (FAB-HAP-010).
- **"Zero power"** means zero coil current at equilibrium. Standby electronics dominate the energy use (FAB-PWR-002).
- **No passive levitation.** Control is mandatory (Earnshaw), and a control loss ends on rollers, a seat or a designed stop (FAB-LEV-019).
- **Seismic.** fabrail runs P-wave early warning with a land-first policy. It has an alert interface and a synthetic injector, and no live ShakeAlert or JMA feed. It gives no useful on-site warning within about 20 km. Its seismic action is not a rated safety function. It makes no claim of building protection and none of being `earthquake-proof`, and it exerts no pressure-wave force (SPECIFICATION §5).
- **Research tier (🧪), not product features:** isolation, active or tuned mass damping, inertial smoothing and fast re-catch (FAB-SEI-014, FAB-SEI-016, FAB-HAP-017).
- **Standards.** Limits are "checked in simulation against numeric limits from" the stated editions. There is no compliance, conformity or certification. PL and SIL targets are "designed toward" (FAB-SAF-019).
- **Firmware.** The STM32G474 firmware is a release build-and-link target with a flash and RAM budget (FAB-FW-010); it has never been flashed. Timing, memory use and bit-identity on hardware are unmeasured. SIL ≡ firmware is claimed in voltage coil mode only (FAB-FW-006, FAB-FW-012).
- **Haptic lightness does not reduce impact energy**, and heavy panels do not move fast without PL d presence sensing (FAB-SAF-001, FAB-SAF-003).
- **Model fidelity.** The magnet model is a reluctance model with no FEM validation; leakage and fringing may need 1.2–1.5 times more magnet. Eddy lag, rail drag and hysteresis are uncertain and swept, and a laminated or SMC rail is assumed (FAB-LEV-020).
- **Unmeasured.** Stray field is modelled, not measured, and WALL and HEAVY need shrouds. There are no dB(A) figures; acoustics and debris need hardware (FAB-ENV-001, FAB-ENV-005).
- **DRAWER.** It is undermount, not a 13 mm side-mount. It is contactless only to 50 % extension, hovers open for about 5 min on its supercapacitor, and needs a manual re-seat above 20 kg (FAB-LEV-016, FAB-LEV-019).
- **Uncorrected early values** from the research (0.40 m/s drawer opening, 0.05 m/s wall, 150 N unsensed stall force, lifting a DRAWER off 6.5 mm rollers) exceed the caps or are infeasible, and are not used.
- **The sovereign and frontier tiers** are research and what-if studies (EXOTIC.md, FRONTIER.md). 🔭 items are not real capabilities, and whole-building levitation is not proposed.
- **The villa example** is a simulated replay with a synthetic alert. Its contents result is "no Coulomb slip event in the sliding-block model" (FAB-EXA-002, FAB-EXA-003).

*See:* SPECIFICATION §4, §5 · PHYSICS §26 · [R67], [R91], [R46]

**Honest limit.** This narrative describes a simulated system: every number in it is a design value or a model output, not a measurement.
