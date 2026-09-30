# fabrail seismic design: three readings, stated honestly

The README promises "mitigating seismic forces through pressure wave effects". That phrase admits three readings, and the user asked for all three:

| Reading | What it would mean | Status | Where it stands |
|---|---|---|---|
| (a) P-wave early warning | Detect the fast, weak P-wave (on site, or from a network alert) and put every carriage into a safe state before the damaging S-wave arrives | ✅ / 🔶 per requirement | Implemented and tested in simulation; the **default** |
| (b) Active or tuned mass damping | Use a heavy moving panel as a mass damper for the building | 🧪 | Implemented as research; the mass ratio makes it negligible for buildings |
| (c) Contactless isolation | Let the levitated panel stay still while the floor shakes under it | 🧪 | Implemented as research; the thrust and stroke make it saturate |

The default policy is **land-first**: freeze, then land (and lock where allowed). This document explains each reading with its numbers, why the default is what it is, and where the data comes from. Physics is in [PHYSICS.md](PHYSICS.md) §19–§21; requirements are FAB-SEI-### in [SPECIFICATION.md](SPECIFICATION.md) §3.5; the mechanism is in [TECHNICAL_DESIGN.md](TECHNICAL_DESIGN.md) §10.

> **Scope, stated honestly.** Nothing here shields a building from shaking, cancels seismic forces or exerts a pressure-wave force. There is no useful on-site warning closer than about 20 km. fabrail is not an EEW provider: every network alert in this repository is synthetic or injected, and a live ShakeAlert or JMA feed is roadmap and needs a licence. The seismic action is declared not a rated safety function (SAFETY.md, SF-08). Isolation and mass damping are research tier, disabled in every preset.

Status badges follow SPECIFICATION §0.5: ✅ verified in simulation, 🔶 simplified model, 🧪 research tier, 🗺️ roadmap, 🔭 frontier.

---

## §1 What "pressure wave" means

An earthquake radiates two body waves. The **P-wave** (primary, compressional; a pressure wave in the rock) travels at V_p ≈ 6.0 km/s and carries little damaging energy. The **S-wave** (secondary, shear) travels at V_s ≈ 3.5 km/s and carries most of the strong shaking. The earthquake therefore arrives twice, and the gap between the arrivals grows with distance:

```text
t_S − t_P = R·(1/V_s − 1/V_p) = 0.1190 s/km × R        (R hypocentral; PHYSICS §20)
```

This is the S−P anchor of FAB-SEI-012.

In fabrail, "pressure wave" means the P-wave, and "mitigating through pressure wave effects" means **acting on the P-wave's warning** before the S-wave arrives. It never means a force the rail or its magnets exert on the ground or the building.

---

## §2 Reading (a): P-wave early warning with network fusion

### §2.1 Lead time

Two sources of warning compete (PHYSICS §20; `params::derive::lead_time`, `lead_crossover_km`):

- **On site.** A structure-mounted accelerometer picks the P arrival at t_P = R/V_p; the detector needs the pick plus the short Pd/τc window, 1.2 s in total (`ONSITE_DECISION_S`), before it can decide. On-site lead = 0.1190·R − 1.2 s.
- **Network.** A regional network issues an alert a fixed latency after origin time; 8 s is used here (`NETWORK_ALERT_S`; the first ShakeAlert alert for Ridgecrest 2019 came 6.9 s after origin). Network lead = R/V_s − 8 s.

The two are equal at R = V_p·(8 − 1.2) = **40.8 km**: closer than that, the on-site pick is earlier; farther, the network alert is.

| R_hyp (km) | S − P (s) | On-site lead (s) | Network lead (s) | Best lead (s) |
|---|---|---|---|---|
| 10 | 1.19 | 0.00 | < 0 | **0** |
| 15 | 1.79 | 0.59 | < 0 | 0.59 |
| 20 | 2.38 | 1.18 | < 0 | 1.18 |
| 25 | 2.98 | 1.78 | < 0 | 1.78 |
| 30 | 3.57 | 2.37 | 0.57 | 2.37 |
| 40.8 | 4.86 | 3.66 | 3.66 | 3.66 (crossover) |
| 60 | 7.14 | 5.94 | 9.14 | 9.14 |
| 100 | 11.90 | 10.70 | 20.57 | 20.57 |
| 150 | 17.86 | 16.66 | 34.86 | 34.86 |
| 200 | 23.81 | 22.61 | 49.14 | 49.14 |

Inputs: V_p 6.0 km/s, V_s 3.5 km/s, on-site decision 1.2 s after P, network alert 8 s after origin; lead is measured to the S arrival. R is **hypocentral**: for a 10 km deep event, R ≥ 10 km even at the epicentre.

**The blind zone.** Below R ≈ 20 km the best lead is at most about 1.2 s. That is enough for FREEZE (≤ 100 ms from the pick, FAB-SEI-009) but not for a landing to complete before the S-wave, so fabrail claims **no useful on-site warning below about 20 km**. The events that hurt most are often the close ones; early warning does not help there, and the design does not pretend it does.

### §2.2 Detector chain (seismic group, 100 Hz)

| Stage | Implementation | Requirement |
|---|---|---|
| Sensor | ADXL355, 500 Hz ODR, 22.5 µg/√Hz, ±2 g, 20-bit, on the structure or rail; a carriage mount is rejected by validate | FAB-SEI-001 |
| Anti-alias | Second-order CIC ÷5 on the integer codes (exact in f32) | FAB-SEI-002 |
| Band-pass | 4-pole Butterworth 0.5–20 Hz, prewarped, TDF-II | FAB-SEI-002 |
| Trigger | Recursive STA/LTA, 50 / 3000 samples, on 4.0 / off 1.5, LTA frozen while triggered; a step in variance triggers at the time of the exact discrete recursion including the LTA term (±1 sample), and the continuous estimate τ_sta·ln((K − 1)/(K − θ)) = 0.1116 s at K = 16 is reported | FAB-SEI-002, FAB-SEI-003 |
| Magnitude proxies | Pd (peak displacement) and τc = 2π/√(∫u̇²/∫u²) over 1 s and 3 s windows, by double integration with a 0.075 Hz high-pass | FAB-SEI-004 |
| Discrimination | Z/H > 1 in the first 0.5 s; actuator veto (the rail's own motion) unless PGA ≥ 80 gal | FAB-SEI-006 |
| Level | τc ≥ 0.6 s and Pd ≥ 0.5 cm → L3; τc only → L2; Pd only → L1 | FAB-SEI-005 |
| PROTECT rule | **Level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal**; Level 1 means freeze only | FAB-SEI-005 |

The Pd anchor follows Wu and Zhao (2006): log₁₀ Pd[cm] = −3.463 + 0.729·M − 1.374·log₁₀ R[km], which gives 0.7118 cm for M7 at 20 km (FAB-SEI-012). The synthetic P+S records scale their P phase so that Pd follows this law at the hypocentral distance (PHYSICS §19), and Pd is recovered within ±20 % on synthetic M6–7 records (FAB-SEI-004; 🔶, because the records are synthetic and the τc and Pd thresholds are calibration values not validated on real records here).

### §2.3 False alarms and missed events

- **False alarms.** 24 hours of synthetic site noise (MEMS noise, footsteps, door slams, HVAC tones, the rail's own actuation) replayed through the detector in 10-minute chunks must give **0 PROTECT**; the raw-trigger count is reported (FAB-SEI-006, slow tier). A trigger without a PROTECT decision causes at most a FREEZE, which clears itself.
- **Small events.** An M5 at 10 km must not land or lock in ≥ 95 % of seeds: it stays at ≤ L1 and only freezes (FAB-SEI-010).
- **Missed events.** The actuator veto stops the rail from triggering on its own motion, but it is bypassed whenever PGA ≥ 80 gal, so strong shaking always reaches PROTECT (FAB-SEI-006).
- **Spoofed alerts.** A network-only or injected alert may cause only FREEZE, LAND, LOCK or a passive ride-through; alert-triggered motion needs independent confirmation (FAB-EXO-119).

### §2.4 Fusion and S-ETA

- A network alert (magnitude, epicentral distance, depth, origin offset) gives the S-ETA = origin + R_hyp/V_s, within ±1 s of the truth in the tests, and the class actions start before t_P (FAB-SEI-007).
- A cancel returns every state to NORMAL (FAB-SEI-007, FAB-SEI-008).
- The seismic state machine, NORMAL → TRIGGERED → PROTECT → SHAKING → QUIET (30 s) → INSPECT → NORMAL, is exhaustive under generated event sequences, and INSPECT needs a manual reset (FAB-SEI-008; TECHNICAL_DESIGN §4.2).
- The Sovereign tier adds fused feeds (offshore cabled, DAS, a PEGS magnitude advisory that is never a sole trigger) and states the value of moving sensors toward the source: 0.15385 s of warning per kilometre at V_p 6.5 km/s (EXOTIC §3.8, FAB-EXO-803).

### §2.5 Class actions and egress rules

Land-first is the core default. Each class action starts with FREEZE (decelerate at a_brk = 2·a_max and hold).

| Class | Action in PROTECT | Rest after the event |
|---|---|---|
| DRAWER | FREEZE, then stay seated, or seat | Seated; re-seat by hand if it failed down above 20 kg |
| DOOR | FREEZE → stop → land on rollers, release ASC, **never lock**; a fire-rated door never opens; a non-rated egress door may open only if it can be stopped ≥ 1 s before t_S | Landed, manually operable (≤ 22.2 N) |
| WALL | FREEZE → centre **only if reachable**, abs(x − x_c) ≤ v·(t_budget − t_land) → land + lock | Landed and locked (not on an egress path) |
| HEAVY | As WALL | Landed and locked |

Egress rules hold whatever the detector does (FAB-SEI-011, FAB-SEI-019): an egress door never locks, stays manually operable after landing, and a power loss at the S arrival still lands it captive.

### §2.6 Worked timelines

| Case | t_P | PROTECT | t_S | Result required |
|---|---|---|---|---|
| WALL, M7, R_hyp 20 km | 3.33 s | t_P + 1.2 = 4.53 s (±0.2 s) | 5.71 s | L3; PROTECT at t_P + 1.2 s; freeze complete before t_S; landing before t_S is reported, not required (FAB-SEI-010) |
| M7.8, R_hyp 100 km | 16.67 s | ≈ 17.9 s on site (a network alert at 8 s would be earlier) | 28.57 s | ≥ L2 (Pd at 100 km is only 0.30 cm, so the level rests on τc); landed ≥ 8 s before t_S (FAB-SEI-010, slow tier) |
| villa, warned, M7.2, R_epi 60 km, 12 km deep | 10.2 s | fused alert ≈ 7.5 s; on-site PROTECT 11.4 s | 17.5 s | Every device reaches its commanded rank by its deadline or is reported NotSecured (FAB-EXA-004) |

### §2.7 What reading (a) is not

- It does not reduce the shaking. It changes the state the rail is in when the shaking arrives.
- It is not a warning service. fabrail consumes alerts; it does not issue them to people.
- It is not a rated safety function (SF-08). The passive guideway, the landing and the mechanics carry the safety.

---

## §3 Reading (b): active or tuned mass damping (🧪)

**The idea.** A heavy panel moving on an LSM is a mass that can be driven against the building: a tuned mass damper (passive) or an active mass damper (driven). `fabrail-core::seismic::amd` implements direct velocity feedback and injected LQR gains; the Python layer tunes by Den Hartog and Warburton (PHYSICS §21).

**The theory reproduces** (FAB-SEI-016):

| Quantity | μ = 0.02 | Source |
|---|---|---|
| Den Hartog optimum frequency ratio f = 1/(1 + μ) | 0.98039 | Den Hartog |
| Den Hartog optimum damping ζ = √(3μ/(8(1 + μ)³)) | 0.08407 | Den Hartog |
| Warburton (base excitation) f / ζ | 0.97548 / 0.07019 | Warburton |
| LQR scalar gain K = (a + √(a² + b²q/r))/b | rel 1e-9 | — |

**The mass ratio decides.** A damper's effect scales with μ = m_damper/M_building.

- The first building with an active mass damper, Kyobashi Seiwa (Tokyo, 1989), used a mass ratio of about 1 %.
- A 5 t HEAVY panel in a 10⁷ kg building gives **μ = 0.05 %**, twenty times smaller.
- A stroke-limited damper adds at most ζ_add ≤ (μ/2)·(S/X), with S the damper stroke and X the structure's displacement (EXOTIC §2). At μ = 0.05 % that is 0.175 % for a wind-like stroke ratio S/X = 7 and 0.035 % at a design-level ratio of 1.4, against the 2–5 % damping a building already has.
- The closed-loop 2-DOF Structure model shows the reduction against μ, honours the LSM force and stroke limits, and shows a negligible effect at μ = 0.05 % (FAB-SEI-021).

**Verdict.** Mass damping by a rail panel is not a building-protection feature. Mass dampers are mainly wind devices; even the Sovereign SENTINEL (a 1,000 t damper in a supertall tower) adds only about 0.7 % damping at design-level shaking and switches to a passive seismic mode (EXOTIC §3.6).

---

## §4 Reading (c): contactless isolation (🧪)

**The idea.** A levitated panel has no friction along x, so in principle it could stay still in space while the floor moves under it: base isolation, with the LSM as a soft spring and a skyhook damper, F = −k·x_rel − c_sky·ẋ_abs (`fabrail-core::seismic::isolate`; PHYSICS §21).

### §4.1 What isolation can do in principle

Transmissibility of a base-isolated mass (FAB-SEI-014):

```text
T(r, ζ) = √((1 + (2ζr)²) / ((1 − r²)² + (2ζr)²)),   r = f/f_iso        (PHYSICS §21)
```

- T(3, 0.2) = 0.1931: shaking at three times the isolation frequency is cut to 19 %.
- T(√2) = 1 for any ζ: below √2·f_iso isolation amplifies, and near f_iso it amplifies strongly.

### §4.2 What the LSM can actually do

A free (unforced) levitated mass drifts relative to the floor by about the peak ground displacement (FAB-SEI-013). To hold it within a stroke s the LSM must supply the spring force k·s = m·ω²·s continuously. With the continuous thrust F_cont that sets a minimum isolation period (`derive::t_min_iso`; FAB-SAF-020):

```text
T_min = 2π·√(m·s/F_cont),   T_iso = max(2 s, 1.1·T_min)        (PHYSICS §21)
```

| Class | m | Stroke s | F_cont | T_min | T_iso | Force authority F_cont/m |
|---|---|---|---|---|---|---|
| WALL | 500 kg | ±0.2 m | 100 N | 6.28 s | 6.91 s | 0.20 m/s² (0.020 g) |
| HEAVY | 5000 kg | ±0.2 m | 500 N | 8.89 s | 9.77 s | 0.10 m/s² (0.010 g) |

Isolation is offered on WALL and HEAVY only; DRAWER and DOOR have no isolation mode.

**Why WALL isolation largely saturates.**

- The LSM can push the panel with only 0.02 g against floor accelerations of 0.3–1 g. Above that, the panel is effectively a free mass and its relative displacement approaches the floor displacement.
- An estimate from the ASCE 7-16 design spectrum used by the ground-motion synthesis (S_D1 = 0.6 g, T_L = 8 s): at T_iso = 6.91 s, S_a = S_D1/T = 0.087 g, so the 5 %-damped spectral displacement is S_a·g·(T/2π)² ≈ 1.03 m; with the ASCE 7-16 damping coefficient B = 1.7 for 30 % damping it is ≈ 0.61 m, about three times the ±0.2 m stroke. HEAVY (T_iso 9.77 s, beyond T_L) gives ≈ 0.70 m. These are design-spectrum estimates, not simulation results.
- In practice the stroke governor brakes at full force near the envelope, and the **end zones (150 mm) and bumpers are the real protection** of the panel and the rail.

### §4.3 Evidence and the safety deviation

- **Monte Carlo** (FAB-SEI-015, slow tier): N = 200 Kanai–Tajimi/Clough–Penzien records at 0.5 g, WALL with ±0.2 m stroke, 150 mm end zones and rated bumpers, under `IsolateLimits`. The gate is the Wilson-95 upper bound of P(bumper overload) < 5 % (k ≤ 3 of 200), not the stroke, because some end-zone entry is physically unavoidable; P(end-zone entry) and the sensitivity to the Clough–Penzien filter frequency ω_f are reported. The result is produced by the test and the `isolation_montecarlo` study, not quoted here in advance.
- **The pedestrian envelope does not apply in ISOLATE.** The monitor replaces its clamp and energy checks with the explicit `IsolateLimits` set (stroke, bumper energy, F_cont) and demands STO + SS1 when it is left. SAFETY.md §4 declares this an R-tier deviation (FAB-SAF-020).

### §4.4 Vertical: attractive EMS can only pull

An attractive suspension hangs the carriage under the rail. When the floor (and the rail) drops, following it requires the magnet force to fall below the weight: a downward 0.8 g pulse needs 80 % of the permanent-magnet force cancelled. The coil current saturates on that side first, the gap opens, and the supervisor raises VerticalSaturation and lands the carriage without an upper-stop impact above rated speed (FAB-SEI-017). **Vertical isolation by maglev is not proposed**, at any scale; the Sovereign HAVEN platforms keep passive vertical support (FAB-EXO-705).

### §4.5 Drawer-scale smoothing and the Sovereign platforms

- **Inertial smoothing** (🧪, DRAWER only, disabled in every preset) is the useful small version of reading (c): an already-open drawer follows only the floor motion below f_c ≈ 1 Hz, with a residual T_sky = 1/√((1 − r²)² + (2ζr)²) (≈ 0.24–0.31 at 2 Hz, ≈ 0.04 at 5 Hz). On a fixed-base building it attenuates the 3–5 Hz building response within an 8–25 mm stroke; ground motion below f_c still passes; on an isolated building it removes only the high-frequency edges of the isolator's motion (FAB-HAP-017, FAB-HAP-018, FAB-HAP-019).
- **HAVEN-R** (🧪, Sovereign tier) is a 50 t isolation platform at T = 6 s with a ±0.9 m stroke inside a moat, passive vertical support and a copper eddy damper (EXOTIC §3.7). It exists to show what isolation needs: metres of stroke, a long period and a guarded moat. Whole-building levitation is not proposed.

---

## §5 The default: land-first

Every class freezes first, then lands (DRAWER stays seated or seats; WALL and HEAVY also lock where allowed). The reasons are physical:

1. **Lateral capacity.** Passive reluctance guidance and the guide fin hold roughly 0.1–0.3 g across the rail (core design estimate; PHYSICS §6); design-level floor shaking is 0.3–1 g. A levitated carriage in strong shaking would load its guides and stops anyway.
2. **Vertical saturation.** Attractive EMS cannot follow a dropping floor (§4.4).
3. **Travel along x.** A levitated or rolling mass is free along x and drifts by about the floor displacement (FAB-SEI-013). Locking (WALL/HEAVY), the seat and the optional seismic latch (DRAWER, up to their retention force; beyond it the scene reports the drawer opened) or the end zones and bumpers hold it; bumpers are rated on a 1 m/s (DRAWER) or 1.5 m/s basis (FAB-SAF-014).
4. **Energy.** Landing needs little energy and completes on the supercap if the supply fails at the S arrival (FAB-SEI-019).
5. **Robust to detector error.** A false PROTECT costs a landing; a missed one leaves the mechanics to carry the event, which they must do anyway because the warning near the source is zero.

**Exceptions, and how they stay bounded.** Land-first is not universal in the Sovereign tier: SENTINEL rides through levitated, BASTION completes its close or lands, and TIDEGATE closes only after the shaking and only on an external decision-system command (EXOTIC §0.4). In the example gallery, the house choreographer may close open drawers and doors before the S-wave. All of them act through the `ActionPolicy` hook and the core `admit` filter: network-only alerts may only freeze, land, lock or ride through passively; alert-triggered motion needs a local P pick or ≥ 2 independent feeds, moves only toward a safer state in the closing direction, and never reverses a user's motion (FAB-EXA-006, FAB-EXO-119). In the near-field case a drawer already gliding open freezes within 100 ms and holds (FAB-EXA-014).

---

## §6 Data provenance

| Data | Source and licence | How fabrail uses it |
|---|---|---|
| Synthetic ground motion | Generated: Kanai–Tajimi × Clough–Penzien spectra with the Jennings–Housner–Tsai envelope, matched to the ASCE 7-16 spectrum; floor amplification 1 + 2z/h (PHYSICS §19; FAB-SEI-018, FAB-SEI-020) | Isolation Monte Carlo, class actions, scenes; seeded through numpy `SeedSequence` |
| Synthetic P+S records | Generated: Brune source (30 bar), Boore stochastic method, κ 0.04, with the P phase scaled so Pd follows Wu–Zhao at hypocentral R (PHYSICS §19) | Detector and policy tests (FAB-SEI-004, FAB-SEI-010) |
| Site noise | Generated: ADXL355 noise, footsteps, door slams, HVAC, the rail's own actuation | False-alarm test (FAB-SEI-006) |
| El Centro, 18 May 1940 (Imperial Valley) | **Fetched on request, never vendored**: `python -m fabrail data fetch elcentro` downloads the record from the CESMD distribution into the git-ignored `data/` directory and pins its SHA-256 on first use (`data/SHA256SUMS`), writing a provenance file next to it. CESMD's data policy asks for attribution, which the provenance file and the README carry. | Isolation and end-stop tests only (marked `requires_data`); not detector tests, because the digitised record captures the P onset poorly. A seeded surrogate (`elcentro_like`: medium soil, 0.34 g, 30 s) keeps every default test offline |
| The OpenSees "elCentro" example file | **Not El Centro 1940**: it is the 1968 Borrego Mountain record. | Not used |
| STEAD (Stanford Earthquake Dataset; Mousavi et al., 2019) | CC BY 4.0; downloaded by the user, never vendored; attribution required by the licence | Optional detector evaluation through `python -m fabrail detect --input FILE --fs 100` on 3-component waveforms |
| Network alerts | Synthetic or injected in scenarios and scenes, always labelled "synthetic/injected, not a live EEW feed" | Fusion and S-ETA tests; the villa example |

No recorded earthquake data ships with fabrail. Anchors that depend on numpy streams record the numpy version, because numpy does not promise identical `Generator` streams across versions (NEP 19); the Rust sensor-noise streams are in-repo and stable.

---

## §7 Non-claims

fabrail does **not** claim:

1. building protection of any kind, "cancelling" seismic forces, any "pressure-wave force", or `earthquake-proof` installations;
2. useful on-site warning below about 20 km, or any live ShakeAlert or JMA integration (only an interface and a synthetic injector exist);
3. to be an earthquake early-warning provider;
4. that active or tuned mass damping by a rail panel is a product feature (🧪; negligible at building mass ratios);
5. that contactless isolation is a product feature (🧪; it largely saturates on WALL and HEAVY);
6. maglev vertical isolation or whole-building levitation;
7. that the seismic action is a rated safety function (SF-08);
8. that the τc and Pd thresholds are validated on real records, or that the synthetic records reproduce any specific earthquake.
