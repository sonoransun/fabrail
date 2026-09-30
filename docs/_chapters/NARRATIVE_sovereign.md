<!-- The sovereign and frontier chapter of docs/NARRATIVE.md (docs unit X9). Unit C2b merges this file at the sovereign chapter marker, between the two horizontal rules already there, and adds its section titles to the Contents line. Links and reference tags are written relative to docs/, as they will be after the merge. -->

## When cost is no impediment: the sovereign tier

The README's second paragraph ends at "large infrastructure components". The user then widened the brief: cost is no impediment, so what would fabrail become for a billionaire's tower or a nation's shelters, stadiums and sea gates? And, separately, what lies beyond today's physics? This chapter answers both. The first answer is the Sovereign tier ([EXOTIC.md](EXOTIC.md)); the second is the 🔭 frontier tier ([FRONTIER.md](FRONTIER.md)).

**What money buys.** Better materials (FeCo poles, grain-boundary-diffused NdFeB, SmCo), triplex sensing, lockstep controllers voting in pairs or triples, flywheels and supercapacitors, and a superconducting option for ride-through. Nine new classes use them, from a 50 t vault door to an 8,600 t stadium tray. They live in their own `ExoClass` list and never change the core classes or their pedestrian caps (FAB-EXO-006).

**What money does not buy.** Five limits hold at any budget, and every sovereign number is checked against them:

- magnetic pressure stops at B²/2μ0, 2.1 MPa at the saturation of the best soft iron, and the rail saturates too (EXOTIC §X2);
- a mass damper of stroke S on a building moving X adds at most (μ/2)(S/X) of damping (EXOTIC §X6);
- moving a sensor 1 km toward the fault buys only 0.154 s of warning (EXOTIC §X10);
- vertical isolation must be passive, because an active actuator carrying the load circulates M·g·v, 294 MW for 100 kt at 0.3 m/s (EXOTIC §X9);
- reacting to a blast needs the blast to start about 2 km away or more (EXOTIC §X11).

**Safe by separation, not by lightness.** The pedestrian caps that make a 100 kg door safe (1.69 J, 22.3 kg·m/s) allow a 5 t mover 26 mm/s and a 500 t mover 2.6 mm/s, in practice nothing. So every sovereign class keeps people out of reach while it moves (a guarded zone with presence sensing, interlocks and an ISO 13855 separation distance) or carries them within comfort limits (an occupied platform). Land-first stops being universal too: a tower damper rides through levitated, a flood gate closes after the shaking, and a vault door finishes a close it can finish in time.

**Badges.** The badge is fabrail's own verification status; 🔭 replaces it for frontier items; technology maturity (TRL) is a separate column in every table. A formula verified to twelve digits can describe a machine nobody has built, and the tables say so.

*See:* FAB-EXO-001, FAB-EXO-006, FAB-EXO-102, FAB-EXO-114 · EXOTIC §0, §X1, §S1 · PHYSICS §16

**Honest limit.** Every sovereign class is research tier at best (TRL 1–4), simulated on reduced models, with no hardware; nothing in it is certified, and its safety architectures are designed toward SIL targets, not certified to them.

---

## Billionaire scale

### The quiet tower (SENTINEL, 🧪)

A residential supertall in a typhoon city sways at 7 s. Near its top a 1,000 t mass, about 1 % of the first-mode mass, rides a maglev planar bearing: two cross-stacked 1-D stages under a concave guideway of radius 12.2 m, which sets the 7 s tuning passively. On a low-coercivity or homopolar rail the bearing has no breakaway friction, so the damper starts moving at 0.3 mg or less, a tenth of the 3 mg at which residents begin to notice sway. A PTFE slider (about 50 mg) would sit still, and Citicorp's oil-film damper needs about 3 min of pumping before it floats [R145]. An ironless linear motor adds 24–121 kN of trim, and the controller can retune the damper by about ±5 % in frequency at full stroke. With the tower's own damping at 1 %, the damper cuts the wind-driven sway RMS by about 45 %; at 2 % it cuts about 30 %. The reference points are Taipei 101's 660 t pendulum and Shanghai Tower's 1,000 t eddy-current damper, which already damps with magnets over copper and no power [R144] [R147].

When an alert arrives the damper steps down to SEISMIC_PASSIVE, as the Shanghai World Financial Center's active dampers do [R148], and rides through levitated on islanded storage for at least 300 s. If the power goes, the magnet-over-copper damping stays; if the levitation goes, it settles on a friction-pendulum catcher and becomes a conventional TMD. The villa example's penthouse variant uses exactly this step.

*See:* FAB-EXO-110, FAB-EXO-601, FAB-EXO-603, FAB-EXO-605, FAB-EXO-607, FAB-EXO-609 · EXOTIC §X6, §X7, §X8 · [R144], [R147], [R148], [R151]

**Caveat.** It is mainly a wind device: at design-level shaking the stroke bound caps its added damping at about 0.7 %. The 1,000 t maglev bearing is TRL 3–4, software retuning works at wind amplitudes only, and on a supply loss the suspension may go up or down onto designed sliding stops; no "no clamp-up" property is claimed.

### The collection that floats (HAVEN-R, 🧪)

A private museum puts its most fragile pieces on a 50 t levitated plinth. A curved guideway of radius 8.95 m gives it a 6 s period and ±0.9 m of stroke; air springs carry the weight; the linear motor adds skyhook damping. At 1 Hz the plinth passes about 3 % of the floor's motion, against about 7 % for the same period with passive damping. Because nothing slides, there is no activation threshold, so it also isolates the traffic hum and small earthquakes that pass through a friction-pendulum bearing untouched. It pre-centres before the S wave only when an alert is independently confirmed and the moat around it is verified clear. The same plinth appears in the villa example's gallery room.

*See:* FAB-EXO-505, FAB-EXO-701, FAB-EXO-702, FAB-EXO-705, FAB-EXO-707 · EXOTIC §X9 · [R16], [R92], [R153]

**Caveat.** This is a scale-up of about 10⁴× from lithography stages, fully dependent on power (islanded storage, then an FPS landing at ≤ 0.1 m/s if control is lost), with no code path and no prototype tests. The 0.1 g occupant target is an analysis output for a stated site, and the moat edge is a shear hazard that covers and exclusion handle, not sensors.

### The vault that answers in five seconds (BASTION, 🧪)

A 50 t sliding plug door closes 3 m in 5.25 s (0.48 m/s², a jerk of 2 m/s³, a peak of 1.14 m/s) and lands onto wedges to seal. Walked by an attendant in jog mode it has "haptic heft": at most 10 mm/s and 1.69 J at its rated mass, so 8.2 mm/s at 50 t, with its momentum of 411 kg·m/s stated rather than hidden. Its closure planner answers one question: given the arrival time of a threat, can the door finish before it, seal included? If yes, it closes; if not, it lands in place. Cheyenne Mountain's 25 t hydraulic doors are the reference [R159].

*See:* FAB-EXO-502, FAB-EXO-503, FAB-EXO-504, FAB-EXO-507, FAB-EXO-807 · EXOTIC §X11, §X14

**Caveat.** At full speed this is a guarded machine behind 5 m of separation, not a door anyone walks through. Reacting to a blast needs the source about 2 km away or more; nearer blasts arrive in 0.1–0.5 s, so such doors stay normally closed and blast valves handle the air paths. No EMP or shock hardening is claimed, and the class is TRL 2–3.

### The private observatory (ROTATOR, 🧪)

A telescope dome turns on a ring of contactless stations instead of trolleys: no wheel wear, no rolling vibration fed into the pier, and regenerative braking of the ring's kinetic energy. The ELT's 6,100 t dome on 36 stationary trolleys is the reference scale [R137]; a private dome would be smaller.

*See:* FAB-EXO-122, FAB-EXO-504 · EXOTIC §X5, §X1.2 · [R137]

**Caveat.** Ring ovalisation and wind uplift act against a 15–20 mm gap, so mechanical hooks are required. The energy-admission check fails today for the ELT-scale ring (its 2 kWh buffer holds only 1.05× the 6.86 MJ of braking energy, below the 1.2× rule) and is reported, not hidden. Dynamic simulation of the class is roadmap.

**Honest limit.** Each of these is a research-tier study on a reduced model; none has been built, and the installations named are precedents, not fabrail products.

---

## Nation-state scale

### The door with nothing to irradiate (SHIELD, 🧪)

In a proton-therapy vault or an accelerator hall, a 50 t shielding door (5–500 t) is a passive mover: only magnets and steel move, and every coil, sensor and driver sits on the stator side, out of the beam's line of sight. The beam permit exists only while the door is landed and locked. On a P-wave alert the beam goes off in milliseconds and the door lands and locks within 2 s. Today's references are 5 t motorised vault doors that take 10–12 s and 19 t shielding doors pushed on wheels by a chain [R155] [R156].

*See:* FAB-EXO-501, FAB-EXO-502, FAB-EXO-503 · EXOTIC §S1, §X14 · [R155], [R156]

**Caveat.** Radiation ageing of the magnets and sensors is unquantified, and SmCo magnets activate to Co-60 in neutron fields. The land-and-lock timeline runs on a reduced model, and the class is TRL 3.

### The floor under a command centre (HAVEN-F, 🧪)

A national archive or command centre stands on a 5 kt floor that is always levitated: hybrid EMS on about 285 m² of pole face, air springs for the vertical load, a copper eddy damper, and a friction-pendulum layer underneath in case control is lost. It never lifts on an alert, because near the source there is no warning to lift on.

*See:* FAB-EXO-103, FAB-EXO-118, FAB-EXO-504, FAB-EXO-705 · EXOTIC §X2, §X9 · [R90], [R91], [R153]

**Caveat.** fabrail gives this class data and algebraic admission only, no dynamic simulation. It needs 10–500× the bearing area of conventional isolators, has no code path, and depends entirely on power. Whole-building levitation is not proposed; Arx Pax's "hover foundation" is the cautionary tale.

### The gate that waits for the shaking to stop (TIDEGATE, 🧪)

A 150 t land-lock gate guards a subway entrance or a seawall opening. Its real product is the trigger stack: the on-site P pick, the national network, an offshore cabled network, fibre-optic sensing (DAS) and a PEGS magnitude advisory, fused by the largest predicted intensity and a cost–loss rule. For an epicentre 100 km offshore and a city 20 km inland, seafloor stations raise the warning from about 12 s to about 23 s. During the shaking the gate is landed. Afterwards it checks its gap, then closes by maglev or on its wheels, and only on the command of an external decision system, within a tsunami budget of minutes. The Maeslantkering, whose arms close on an automatic decision system, and Japan's remotely closed coastal gates are the precedents [R140] [R143].

*See:* FAB-EXO-801, FAB-EXO-802, FAB-EXO-804, FAB-EXO-809, FAB-EXO-810 · EXOTIC §X10, §X12 · [R140], [R141], [R143], [R182], [R187]

**Caveat.** Buoyant and hydraulic gates (the Maeslantkering, MOSE, the Thames Barrier) beat maglev physically, and debris or salt in an attractive-EMS gap is unsolved. The maglev gate is TRL 2 while the triggering is TRL 7–9, and the decision path has a SIL 3 target; SIL 4 is unclaimable.

### The stadium that moves in pods (ARENA, 🧪)

A national stadium slides its 8,600 t field tray out into the sun on 500 levitated pods. Zero-power hybrid suspension is mandatory, because pure electromagnets would draw about 15 MW just to hover; the tray needs about 491 m² of pole face. When a pod fails, a weighted least-squares allocation shares its load among its neighbours, and the tray lands if any quadrant falls below 1.3× its weight share. State Farm Stadium's tray, which rolls on 542 wheel assemblies, is the reference [R138].

Roofs are the harder problem. The Mercedes-Benz Stadium's eight-panel retractable roof, each panel on two rails, was delayed by roof complexity [R162]. Per-pod force equalisation is aimed at the general difficulty of moving many bogies under one stiff structure; nothing here claims to know what caused that delay.

*See:* FAB-EXO-103, FAB-EXO-502, FAB-EXO-504 · EXOTIC §X1.2, §X2, §X14 · [R138], [R139], [R162]

**Caveat.** Floor flatness and thermal growth over 100 m and more must stay within a 15–25 mm gap, ferrous debris is a hazard, and at 0.05 m/s the energy saving is negligible. Only the static allocation is built; the dynamic simulation is roadmap.

### The hatch in the dark (SELENE, 🔭)

On the Moon, in a permanently shadowed crater at 25–40 K, a 100 t airlock hatch floats on passive superconductors over permanent magnets with no cryocooler at all. Lunar gravity scales the open-loop pole by 0.406 and lets the same magnet lift six times the mass. Moonquakes ring for about an hour, so the hatch lands and locks rather than riding them out.

*See:* FAB-FRN-006 · FRONTIER §F7 · [R214], [R215]

**Caveat.** This is a frontier what-if, not a product. Radiating heat at 40 K manages only 0.145 W/m², NASA's FLOAT is a concept with centimetre prototypes (2 orders of magnitude short of a 5 m hatch), and the nanophase iron in lunar dust collects on permanent-magnet guideways.

**Honest limit.** Nation-state use raises the stakes, not the verification: every class here is simulated on a reduced model with synthetic alerts, and the architectures are designed toward SIL targets that nothing in fabrail certifies.

---

## The README's "pressure waves," extended

The README ends on a phrase:

> mitigating seismic forces through pressure wave effects

The core reads the pressure wave as the **P wave** and acts on the warning it gives. The sovereign tier follows the phrase through every pressure wave that gives a warning, in order of the warning it gives:

1. **The P wave** (core, ✅). The compressional wave outruns the damaging S wave by 0.119 s per km of distance. On-site detection plus network fusion gives a second or less within about 20 km and tens of seconds at 100 km; the core freezes, then lands or locks. Sensors moved toward the fault add 0.154 s per km (0.270 s for intensity methods), which is why offshore cabled networks and DAS matter (EXOTIC §X10).
2. **The air blast** (BASTION, 🧪 class; the closed forms are ✅). A pressure wave in the literal sense. fabrail takes its arrival time as an input and never computes it from a charge size; a shock travels at least at the speed of sound, so a close that takes 5.85 s needs the source about 2 km away or more (EXOTIC §X11).
3. **The tsunami** (TIDEGATE, 🔶 timing). A long ocean wave that travels at √(g·h), 198 m/s over 4 km of water, and grows ×4.47 by the time it reaches 10 m. It gives minutes, not seconds, so time is not the problem; a rail bent by the earthquake that preceded it is (EXOTIC §X12).
4. **PEGS** (🔭). A rupture shifts mass, and the change in gravity arrives at the speed of light, ahead of every pressure wave. An on-site gradiometer that used it to beat the P wave would need 1e-15/√Hz, 5 orders of magnitude beyond the best demonstrated 1e-10/√Hz, and against a dense network it would gain about nothing. What exists today is a network magnitude estimate for Mw ≥ 8.3 that does not saturate; fabrail accepts it as an advisory, never as a sole trigger (FRONTIER §F4) [R195] [R196] [R199].

Two more pressure waves are named and not modelled: the piston wave a train pushes ahead of it in a tunnel (a structural load case for tunnel gates, not a control opportunity) and the atmospheric Lamb wave of a great eruption (narrative only).

*See:* FAB-SEI-012, FAB-EXO-803, FAB-EXO-806, FAB-EXO-807, FAB-EXO-809, FAB-EXO-810, FAB-FRN-003 · EXOTIC §X10, §X11, §X12 · FRONTIER §F4 · [R182], [R191], [R190], [R195]

**Honest limit.** fabrail pushes against none of these waves. It uses each as a clock, a warning of a different length, and designs for the loads it brings; "mitigating seismic forces through pressure wave effects" is read as acting on a warning, never as a pressure-wave force.

---

## Beyond today's physics: the frontier tier

Some ideas are physically grounded and published but far from demonstrated. fabrail models them in numpy only, stamps every output with a 🔭 banner, and prints how many orders of magnitude are missing (FRONTIER §F1):

- **A room-temperature superconductor** would remove the cryogenics but not the pressure ceiling: over a Halbach magnet guideway it reaches 0.51 MPa at 10 mm, and it needs a critical temperature near 358 K, 0.4 orders beyond any material at ambient pressure (FRONTIER §F2) [R209].
- **An active seismic metabarrier** around a site would need about 1.8 Mt of resonating mass to act at 1 Hz on soft soil; field tests exist only above 30 Hz, 4.4 orders away in mass (FRONTIER §F5) [R203] [R205].
- **A structure-scale magnetic cloak** could hide a rail from nearby instruments but cannot beat iron saturation; lab cloaks are centimetres across (FRONTIER §F8) [R210].

Demonstrated items were relabelled: trapped-field bulk magnets, lab-scale cloaks and quasi-zero-stiffness isolation stages are research tier (🧪), and diamagnetic levitation is textbook physics (✅) with NASA's FLOAT as research.

*See:* FAB-EXO-002, FAB-EXO-004, FAB-FRN-001, FAB-FRN-004, FAB-FRN-007, FAB-FRN-008 · FRONTIER §0, §1

**Honest limit.** Frontier items are theoretical what-if studies; none is a product, none drives a controller or a safety function, and each output says how far away it is.
