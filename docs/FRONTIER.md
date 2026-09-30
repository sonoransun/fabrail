# fabrail 🔭 frontier tier

The frontier tier is a set of **theoretical what-if studies. None of it is a product claim.** It asks how the rail's physics would change if a missing material, sensor or scale existed, and it says, in numbers, how far away that is.

This document holds the frontier matrix and the FAB-FRN-### requirements. It uses the grammar, columns, methods, statuses and test tokens of [SPECIFICATION.md §0](SPECIFICATION.md) and is parsed by the same traceability guard. The Sovereign/Exotic tier is in [EXOTIC.md](EXOTIC.md).

> **Status: Wave 1.** IDs, tokens and the matrix rows are frozen. The Section §F prose describes the numpy models as implemented in Wave 1 (`python/fabrail/exotic/frontier/`); where prose and code disagree, code wins and the prose is corrected. The matrix is regenerated from the frontier module's metadata (FAB-FRN-010).

Code cites this document as `FRONTIER §Fn`.

---

## §0 The 🔭 rule

- **Where frontier code lives.** Only in `python/fabrail/exotic/frontier/`, numpy only. No Rust crate contains a frontier model (FAB-EXO-002, FAB-EXO-210).
- **What every output carries.** A 🔭 banner, `"tier":"frontier"` in JSON, a plot watermark, and the orders of magnitude still missing (FAB-EXO-004, FAB-FRN-008).
- **Badges.** The St. column is fabrail's verification status of the model code (SPECIFICATION §0.5). The rendered badge is 🔭 whenever the item's maturity (§1) is frontier. Maturity never merges into the badge.
- **Vocabulary.** Both lists of EXOTIC §0.3 apply to this document and to frontier.html.
- **What 🔭 means.** A physically grounded extrapolation backed by published theory, with no fabrail or full-scale demonstration. Its outputs are what-if numbers, never capability claims, and no frontier item is a product.
- **Anchor tier.** Every 🔭 row names the nearest real, demonstrated capability it extrapolates from, written "🔭 (anchor 🧪)", and states how many orders of magnitude separate the two (§F1).
- **Never in the safety path.** A 🔭 model never drives a controller or a safety function. The only frontier-adjacent input the fusion accepts is the research-tier network PEGS magnitude advisory, and it is never a sole trigger (FAB-EXO-810).
- **Relabelled items** (exotic critique, XCRIT §6). Trapped-field bulks, lab-scale cloaks and quasi-zero-stiffness stages are demonstrated, so they are 🧪; diamagnetic levitation is ✅ as physics, with FLOAT 🧪; a 100 kt isolation platform is a research extrapolation with no new physics (EXOTIC §X9). 🔭 is kept for the ambient-pressure RTSC, the 1e-15 gradiometer, building-scale active metabarriers, structure-scale cloaks and concentrators, and SELENE.

---

## §1 Frontier matrix

"Anchor tier" is the nearest real, demonstrated capability the item extrapolates from.

| Item | Badge | Maturity | Anchor tier | Anchor and missing orders of magnitude | Requirement |
|---|---|---|---|---|---|
| PEGS on-site gravity gradiometer | 🔭 | frontier | 🧪 (PEGS network magnitude, Mw ≥ 8.3) | Needs 1e-15/√Hz; best demonstrated 1e-10/√Hz, 5 orders short. Against a dense P network the lead-time gain is ≈ 0 s; the real value is an unsaturated magnitude | FAB-FRN-003 |
| Ambient-pressure RTSC suspension | 🔭 | frontier | 🧪 (HTS flux pinning) | Needs Tc ≈ 358 K and more pinning energy; the pressure ceiling stays rail-limited (0.51 MPa at 10 mm) | FAB-FRN-001 |
| Building-scale active metabarrier | 🔭 | frontier | 🧪 | ρλ³ at 1 Hz: 1.8 Mt of resonating mass at v_s = 100 m/s (14.4 Mt at 200 m/s) | FAB-FRN-004 |
| Structure-scale magnetic cloak or concentrator | 🔭 | frontier | 🧪 (cm-scale lab cloaks) | Lab demonstrations only; gain capped at B_sat/B0 | FAB-FRN-007 |
| SELENE lunar hatch and cargo rail | 🔭 | frontier | 🧪 (FLOAT concept) | Pole ×0.406; radiator limit 0.145 W/m² at 40 K; nanophase-Fe dust collects on PM guideways | FAB-FRN-006 |
| Trapped-field bulk magnets | 🧪 | research (relabelled from 🔭) | — | Pressure capped by rail saturation (≤ 2.1 MPa against steel) whatever B_trapped | FAB-FRN-002 |
| Diamagnetic levitation | ✅ physics | physics deployed; FLOAT research (cm-scale, ≤ 33 kg/m²) | — | 179 Pa at 1 T as an upper bound | FAB-FRN-009 |
| Quasi-zero-stiffness (QZS) isolation | 🧪 | deployed at instrument scale; research at building scale | — | Stiffness singularity at k_m = −k1; tilt error g·θ/ω² | FAB-FRN-005 |
| Lab-scale magnetic cloak | 🧪 | research | — | — | FAB-FRN-007 |
| NV-centre or atom-interferometer sensing | — | documented only | — | Noise-ASD presets are roadmap | — |
| DAS along the rail | 🧪 | documented only | — | — | — |
| Safe-RL haptics with a CBF filter | 🧪 | documented only | — | — | — |

---

## §2 Requirements (FRN)

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-FRN-001 | The RTSC what-if sweep (η 0.15–1, Tc 90–600 K, no cryogenics) shall report the required Tc ≈ 358 K and keep the pressure ceiling rail-limited (0.51 MPa at 10 mm for λ = 154 mm) | all | FT; XCRIT §1 | A, T | `py:exotic/frontier::rtsc_sweep` | R |
| FAB-FRN-002 | Trapped-field pressure p = η·B²/2μ0 shall be capped by rail saturation (≤ 2.1 MPa against steel) and by σ_mech | all | ET; XCRIT §6 | A, T | `py:exotic/frontier::trapped_field_cap` | R |
| FAB-FRN-003 | The PEGS model (κ_h = 4.6) shall detect Mw 7 at 140 km within 10 s at 1e-15/√Hz, state that the best demonstrated sensitivity is 1e-10/√Hz (5 orders short), and report a lead-time gain of ≈ 0 s against a dense P network | all | FT; XCRIT §1 | A, T | `py:exotic/frontier::pegs_detection_and_gap` | R |
| FAB-FRN-004 | The metabarrier mass law ρ·v_s³/f³ shall reproduce 1.8 Mt at 1 Hz and v_s 100 m/s (14.4 Mt at 200 m/s) within ±5 %, always stating v_s | all | FT; XCRIT §1 | A, T | `py:exotic/frontier::metabarrier_mass_law` | R |
| FAB-FRN-005 | The QZS model shall show series stiffness → ∞ at k_m = −k1 and a tilt error of g·θ/ω² | all | FT | A, T | `py:exotic/frontier::qzs_stiffness_and_tilt` | R |
| FAB-FRN-006 | The SELENE model shall give the open-loop pole scaling √(1.62/9.81) = 0.406 and the radiator limit 0.145 W/m² at 40 K, and print the nanophase-Fe dust caveat for PM guideways | SELENE | FT; XCRIT §6 | A, T | `py:exotic/frontier::selene_lunar_scaling` | R |
| FAB-FRN-007 | The cloak model shall give μ = (R2² + R1²)/(R2² − R1²) and a concentrator gain ≤ B_sat/B0, labelled 🧪 at lab scale and 🔭 at structure scale | all | FT; XCRIT §6 | A, T | `py:exotic/frontier::cloak_permeability` | R |
| FAB-FRN-008 | Every frontier output shall print its missing-orders-of-magnitude figure | all | USER 5 | T | `py:exotic/frontier::missing_orders_printed` | V |
| FAB-FRN-009 | The diamagnetic model shall give p = abs(χ)·B²/2μ0 ≈ 179 Pa at 1 T as an upper bound and check FLOAT consistency (≤ 33 kg/m²) | all | FT; XCRIT §6 | A, T | `py:exotic/frontier::diamagnetic_pressure` | V |
| FAB-FRN-010 | The §1 frontier matrix shall be generated from the frontier module's metadata (badge, maturity, anchor tier, missing orders) and drift-guarded | all | XCRIT P1-9 | I | `py:exotic/frontier::matrix_matches_code` | V(I) |

Source codes are as in EXOTIC.md: FT and ET are the frontier-theory and exotic-technology research reports, XCRIT the exotic adversarial critique, USER 5 the user's frontier-tier decision.

---

## Models (Section §F)

This part specifies the frontier models exactly as implemented in `python/fabrail/exotic/frontier/` (numpy only). Every function returns a stamped record that carries the banner, `"tier": "frontier"` and the missing-orders figure. Code cites these sections as `FRONTIER §Fn`; the physics they borrow is cited as PHYSICS §n or EXOTIC §Xn. References marked [REF-MEM] were cited from memory in the frontier research and not re-checked; values marked [UNCERTAIN] are swept, not trusted.

### Section §F1: The 🔭 rule, banner and missing-orders figure

**The banner.** Every frontier output begins with

> 🔭 FRONTIER: a theoretical what-if study, not a product claim. Missing: N orders of magnitude.

followed by a one-line note naming the item and its gap. JSON records carry `"tier": "frontier"`, the banner text, `missing_orders` (rounded to 0.1, because the figure is an estimate), `missing_orders_exact` and the note. Plot watermarks read "FRONTIER what-if, not a product claim · missing N orders of magnitude", without the 🔭 glyph, which the bundled plot fonts lack (FAB-EXO-004, FAB-FRN-008).

**The missing-orders figure** is the base-10 logarithm of the factor by which the best demonstration falls short of the need, floored at 0. It answers one question: how far is the item from its nearest real anchor? The figure for each item, as the models compute it:

| Item | Required | Best demonstrated (anchor) | Missing orders | Computed as |
|---|---|---|---|---|
| PEGS on-site gradiometer | 1e-15 /√Hz at 0.1 Hz | 1e-10 /√Hz (TOBA Phase-II, 3–8 Hz) | 5 | log10(1e-10 / 1e-15) |
| Ambient-pressure RTSC | Tc ≈ 358 K | about 138 K (Hg-1223 at ambient pressure) | 0.4 | log10(358 / 138); at 300 K no known material superconducts at all |
| Building-scale active metabarrier | a band gap near 1 Hz | field tests at ≥ 30 Hz | 4.4 at 1 Hz | 3·log10(30 Hz / f): the resonating mass grows as f⁻³ |
| Structure-scale cloak or concentrator | shells of metres | centimetre lab cylinders | 2 at R2 = 1 m | log10(R2 / 1 cm); 🧪 at R2 ≤ 1 cm |
| SELENE hatch and cargo rail | 5 m of hatch travel | about 5 cm FLOAT prototypes [UNCERTAIN] | 2 | log10(5 m / 5 cm) |
| Trapped-field bulk (🧪) | — | multi-tesla bulks exist | 0 | the pressure cap is physics, not a missing capability |
| Diamagnetic levitation (✅ physics) | EMS pressure at the same field | abs(χ) = 4.5e-4 of it | 3.3 | log10(1 / abs(χ)) |
| QZS isolation at building scale | 0.5 m of stroke | about 1 mm zero-power stages | 2.7 (0 at instrument scale) | log10(0.5 m / 1 mm) |
| NV gap sensing | 1.3 µT/√Hz | 10 nT/√Hz and better | 0 | over-specified for gap control |
| DAS along the rail | — | DAS deployed for sensing | 0 | the rail integration is research |
| Safe-RL haptics with a CBF filter | — | — | 0 | research tier; nothing is missing in orders of magnitude |

The relabelling of demonstrated items and the rule that no 🔭 model enters the safety path are stated in §0.

### Section §F2: Ambient-pressure RTSC

**Status** (Sept 2026). No room-temperature superconductor exists. LK-99 was refuted in 2023 and the Dias papers were retracted. The highest ambient-pressure Tc is about 133–138 K (Hg-1223); LaH₁₀ reaches about 250 K only near 170 GPa.

**Model** (`rtsc`; FAB-FRN-001). The HTS frozen-image model of EXOTIC §X3 with the shielding efficiency η swept from today's 0.15 to 1 and the cryogenics removed. Over a PM guideway of surface field B0 and period λ = 2π/k, ideal flux exclusion bounds the pressure at

```
P̄(h) = (B0²/μ0)·e^(−2kh)          1.15 MPa at contact, 0.51 MPa at h = 10 mm   (B0 1.2 T, λ 154 mm)
```

Today's bulk YBCO reaches η ≈ 0.15 of that image (the ATZ fit, F0 = 32.9 kN over 4 × 492 cm², against P̄(0)). The earlier design's 0.64 MPa at 10 mm implied λ = 214 mm, not the model's 154 mm (XCRIT §1).

**Temperature.** Jc(T) = Jc0·(1 − T/Tc)^n with n ≈ 1.5 [UNCERTAIN]. REBCO works at 77 K because its reduced temperature t = 77/92 = 0.84 leaves enough Jc. Matching that at 300 K needs Tc = 300 × 92/77 ≈ 358 K, and keeping the flux-creep rate S ∝ kT/U0 needs the pinning energy U0 raised by about 300/77 ≈ 3.9 (with low anisotropy to keep the irreversibility line high). The sweep covers Tc 90–600 K and uses p = min(η·j, 1)·P̄ with j = Jc(300 K, Tc)/Jc(77 K, 92 K).

**Honest reading.** More pinning never lifts p above P̄, so the pressure ceiling stays rail-limited whatever Tc is: 0.51 MPa at 10 mm is about 10× today's HTS at 10 mm and below FeCo EMS (2.1 MPa, EXOTIC §X2). An RTSC's value would be passive stability and zero hover power without cryogenics, not force density. **Tc, not pressure, is the gap:** 0.4 orders of magnitude beyond any ambient-pressure material, and nothing superconducts at 300 K today.

Sources: frontier research §3d; XCRIT §1; [R209], [R171], [R174].

### Section §F3: Trapped-field bulk magnets

**Status.** Research (🧪), relabelled from 🔭. Bulk superconductors trapping several tesla are demonstrated: 17.6 T in a reinforced GdBCO bulk pair at 26 K (Durrell et al. 2014 [REF-MEM]); the ATZ YBCO traps 3.6 T at 40 K and 6.8 T at 20 K.

**Model** (`trapped_field`; FAB-FRN-002):

```
p_raw  = η·B²/(2μ0)                                   η: fraction of B² reaching the gap
B_mech = √(2μ0·σ_mech)                                the bulk cracks when B²/2μ0 exceeds σ_mech
p      = min(η·min(B, B_mech)²/(2μ0),  B_rail²/(2μ0))
```

σ_mech = 30–100 MPa [UNCERTAIN] gives B_mech ≈ 8.7–15.9 T, because the magnetisation hoop stress scales as B²/2μ0. B_rail is the saturation of the steel the bulk pulls on: FeCo at 2.3 T gives 2.10 MPa (the default envelope), a FeCo-lined rail at 2.15 T gives 1.84 MPa and structural steel at 1.6 T gives 1.02 MPa.

**Honest reading.** Against a steel rail the pressure stays at or below 2.1 MPa whatever the trapped field is. Only bulk-to-bulk repulsion reaches multi-MPa (3.6 MPa at 3 T, 19.5 MPa at 7 T), at the price of cryogenics at 20–40 K, flux jumps and cracking. Missing orders: 0; the cap is physics.

Sources: exotic design §2.3; XCRIT §6; [R176], [R212], [R171].

### Section §F4: PEGS gravity gradiometry

**Physics.** A rupture redistributes mass, and the change in gravity travels at the speed of light, so a prompt elasto-gravity signal (PEGS) reaches a site before the P wave. On an accelerometer the gravity change and the ground acceleration it induces cancel in a homogeneous full space; the PEGS seen on seismometers come from free-surface and layering effects and are at most 1–2 nm/s², even for the largest events. A gradiometer measures the spatial derivative, which does not cancel. That is the case for an on-site gradiometer, and it needs a sensitivity nobody has built.

**Evidence.** A superconducting gravimeter at Kamioka saw about 0.9 µGal before the P wave of Tohoku 2011 (Montagner et al. 2016); broadband seismometers saw PEGS at 1,000–2,000 km (Vallée et al. 2017). PEGSNet (Licciardi et al. 2022; 74 stations, 2–30 mHz) tracks magnitude reliably for Mw ≥ 8.3 and would have alerted on Tohoku at 53 s. PEGSGraph (Arias et al. 2025) runs in real time in Peru for Mw ≥ 8.2, with estimates stabilising at about 300 s.

**Model** (`pegs`; FAB-FRN-003; the r⁵ form after Harms 2015 [REF-MEM]; PHYSICS §19–§20 for the P timing):

```
M0(Mw)  = 10^(1.5·Mw + 9.1) N·m                          (Hanks and Kanamori 1979)
Ṁ0(t)   triangle of half-duration T_h = 2.26e-6·M0^(1/3) s
I₂(t)   = second time integral of M0(t)
h(R, t) ≈ κ_h·G·I₂(t)/R⁵,   κ_h = 4.6                    (R: source–sensor distance)
t_det   = first t with h/(ASD·√B) ≥ 5,   B = 0.1 Hz
gain over on-site P       = R/v_P − t_det                 (v_P 6.5 km/s)
gain over a dense network = t_net − t_det,   t_net = R_net/v_P + latency
```

κ_h = 4.6 is a one-point fit to Shimoda et al. 2021: an Mw 7 detected within 10 s out to 140 km with a 1e-15/√Hz gradiometer [UNCERTAIN: the fit, not the paper]. The sweep (depth 10 km, v_P 6.5 km/s; each cell is t_det / gain over on-site P, in seconds):

| Detector ASD (s⁻²/√Hz) | Mw 7 at 50 km | Mw 7 at 100 km | Mw 7 at 200 km | Mw 8 at 200 km | Mw 9 at 500 km |
|---|---|---|---|---|---|
| 1e-13 | 8.9 / −1.1 | 23.7 / −8.2 | 98.9 / −68 | 37.3 / −6.5 | 87 / −10 |
| 1e-15 (the TOBA Phase-III target) | 2.8 / +5.0 | 6.6 / +8.9 | 16.3 / +14.5 | 11.7 / +19.1 | 27.6 / +49 |
| 1e-17 | 0.9 / +7.0 | 2.1 / +13.4 | 4.9 / +25.9 | 3.7 / +27.1 | 8.7 / +68 |
| 1e-19 (the final TOBA target) | 0.3 / +7.6 | 0.7 / +14.8 | 1.6 / +29.3 | 1.2 / +29.6 | 2.8 / +74 |

**Honest reading.**

- The need is 1e-15/√Hz; the best demonstrated is 1e-10/√Hz (TOBA Phase-II, 3–8 Hz): **5 orders short**.
- The fair comparison is a network alert, not on-site P. Against a dense P network (nearest station at a hypocentral distance of about 22 km, 3.3 s latency) the lead-time gain at the reference case is about 0 s. The gain is real only where no station is near the source: offshore subduction zones, or across borders.
- Even an ideal detector sees only the moment released so far, and early source growth looks alike whatever the final size (Meier et al. 2017 [REF-MEM]). PEGS can detect early but cannot know the final magnitude early. At 1e-19 the floor is probably Newtonian noise [UNCERTAIN].
- A detection later than R/v_P is flagged (`before_p` false): after the P wave the prompt-signal model no longer applies.

**What fabrail does with PEGS today.** Only the network magnitude advisory (research tier) enters fusion, as `PegsMagnitudeAdvisory`, and never as a sole trigger (FAB-EXO-810; EXOTIC §X10). Its value is an unsaturated magnitude for decisions that need tens of seconds: securing slow, heavy loads such as stage wagons and retractable roofs after a far-field great earthquake, not the land-first reflex, which on-site P already covers beyond about 20 km.

Sources: frontier research §3a; XCRIT §1; [R195], [R196], [R197], [R198], [R199], [R200], [R202].

### Section §F5: Active metabarriers

**Evidence.** Seismic metamaterials are field-tested only at 30–50 Hz and above (Brûlé et al. 2014, boreholes at 50 Hz; Colombi et al. 2016, a forest at 30–150 Hz). A scaled test gave up to 50 % reduction below 10 Hz (Palermo et al. 2016). There is no field test below 10 Hz, where buildings respond.

**Model** (`metabarrier`; FAB-FRN-004). A lumped-resonator Rayleigh metasurface, adapted from Colquitt et al. 2017 eq. 3.28–3.29 (the lumped form is an adaptation, not a published equation [UNCERTAIN]):

```
L²·ρ·v_s·(x² − 1) = √(1 − r²)·m·ω_r·x,    x = ω/ω_r,   r = v_s/v_p
Γ   = √(1 − r²)·m·ω_r / (L²·ρ·v_s)
x_u = (Γ + √(Γ² + 4))/2 ≈ 1 + Γ/2           (the gap is [f_r, x_u·f_r])
```

**The mass law.** For a fixed relative gap (Γ ≈ 0.2) the areal resonator mass m/L² scales as ρ·v_s/f. A barrier about one shear wavelength λ = v_s/f wide around a site has an area of order λ², so the resonating mass scales as ρλ³ = ρ·v_s³/f³: **1.8 Mt at 1 Hz on soil with v_s = 100 m/s** (ρ 1,800 kg/m³), or 14.4 Mt at 200 m/s. That is about 360,000 five-tonne HEAVY units. Every output states v_s.

**What active maglev units could and could not add.**

- A long-stator LSM gives each resonator a long stroke and a programmable stiffness and damping, so the band can be **retuned** in seconds to a site resonance or an alert-estimated dominant frequency. It cannot **create** momentum capacity; the mass is the limit.
- At 1 Hz and PGV 0.5 m/s a resonator strokes about 0.4 m; a pure-LSM spring would need about 79 kN per 5 t unit, so the stiffness must be physical springs with the LSM as trim.
- A capacitor-shunted LSM makes an electrical inerter that lowers the resonance but does not widen the gap. Only inertial amplification between two soil anchor points escapes the mass limit, and that is theoretical. A space-time-modulated "seismic diode" is at TRL 1–2 for seismic use.
- Active cancellation instead of reflection would have to absorb ρ·v_s·v²·A: about 0.7–9 GW across a 100 × 50 m section at 0.5–1 m/s. Out of reach.

Missing orders: 3·log10(30 Hz/f), 4.4 at 1 Hz.

Sources: frontier research §3b; XCRIT §1; [R203], [R204], [R205], [R206], [R207].

### Section §F6: Quasi-zero-stiffness isolation

**Status.** 🧪: deployed at instrument scale (Mizuno's zero-power tables; LIGO's HAM-ISI gives up to 40 dB from about 0.1 to 8 Hz with a loop gain of about 10³ at 100 mHz, Matichard et al. 2015), research at building scale.

**Series singularity** (`qzs`; FAB-FRN-005; Mizuno et al. 2007 [REF-MEM]). Zero-power control leaves a negative steady-state stiffness k_m = −k_s, the PM force gradient of PHYSICS §3. In series with a spring k1:

```
k = k1·k_m / (k1 + k_m)   →  ∞   at  k_m = −k1
```

so the payload does not sag under loads applied to it, while k1 still isolates ground motion.

**Parallel QZS.** f_n = f_p·√ε with ε = (k_p − k_n)/k_p. A WALL-class 500 kg on a spring with 20 mm of static deflection has f_p = 3.5 Hz; reaching 0.3 Hz needs ε = 0.0073, a stiffness match to 0.7 % that drift of k_n with gap and load turns into an active-trim problem.

**Tilt.** An inertial sensor reads g·θ as acceleration, so a horizontal estimate carries x_err = g·θ/ω²: 25 mm at 0.1 Hz and θ = 1e-3 rad. Rotation sensors are mandatory for long-period isolation, as LIGO found.

**Honest reading.** At building scale the stroke must cover the peak ground displacement (0.5–2 m, more near faults) while a zero-power EMS stage has a millimetre gap: 2.7 orders missing. A precision-fab or data-hall equipment stage is the credible path; whole-building QZS is not proposed.

Sources: frontier research §3c; [R32], [R208].

### Section §F7: SELENE, the lunar hatch

**Setting.** EXOTIC §1 lists SELENE: a 100 t hatch with 5–10 m of travel, passive HTS in a permanently shadowed region (PSR), teleoperated, LandAndLock. Its physics lives only here (`lunar`; FAB-FRN-006).

**Scaling** (PHYSICS §3 for the open-loop pole):

| Quantity | Law | Moon against Earth |
|---|---|---|
| EMS open-loop pole | p = √(2g/gap) | ×√(1.62/9.81) = 0.406 (44.3 → 18.0 rad/s at 10 mm) |
| Lift of the same magnet | ∝ 1/g | ×6.05 |
| FPS period | T = 2π·√(R/g) | ×2.46 |
| Radiated flux at the PSR temperature | q = ε·σ·T⁴ | 0.145 W/m² at 40 K; 0.022 W/m² at 25 K (ε = 1) |

**What the numbers mean.** PSRs sit at about 25–40 K (Diviner, Paige et al. 2010 [REF-MEM]), cold enough for REBCO without a cryocooler, but every watt dissipated there needs 7–45 m² of ideal radiator. Only passive flux-pinned HTS over permanent magnets, driven only while it moves, fits that budget: at the ATZ 53 kPa a 100 t hatch under lunar gravity needs about 3.1 m² of bulk. NdFeB undergoes a spin reorientation below about 135 K, so PSR magnets are PrFeB or SmCo. Shallow moonquakes (up to mB 5.5) ring for about an hour, so holding matters more than warning, hence LandAndLock.

**Dust caveat.** Lunar regolith carries nanophase metallic iron, which is ferromagnetic and collects on permanent-magnet guideways. fabrail does not model it.

**Anchor.** NASA/JPL FLOAT is a concept with centimetre prototypes and a claimed payload of up to 33 kg/m² at more than 0.5 m/s. Missing orders: log10(5 m hatch travel / 5 cm prototypes) = 2 [UNCERTAIN: prototype size].

Sources: frontier research §3g; XCRIT §6; [R214], [R215].

### Section §F8: Magnetic cloaks and concentrators

**Status.** 🧪 at laboratory scale (Gömöry et al. 2012: a superconducting inner shell and a ferromagnetic outer shell at 77 K in a DC field); 🔭 at structure scale.

**Model** (`cloak`; FAB-FRN-007):

```
cloak:         μ_FM = (R2² + R1²)/(R2² − R1²)        ideal SC–FM bilayer cylinder: 5.5 at R2/R1 = 1.2, 1.67 at 2
concentrator:  B_inside = (R2/R1)·B0                  (μ_ρ → ∞, μ_φ → 0; Navau et al. 2012 [REF-MEM])
               gain ≤ B_sat/B0                         (the shell saturates)
```

**Hard caps.** A concentrator cannot beat iron: the force per area stays at or below B_sat²/2μ0, 2.1 MPa for FeCo, against the 0.4–0.9 MPa of an EMS gap, a factor that pole tapering largely provides already. Superconductor AC losses rule these structures out for the travelling field of an LSM.

**Where a cloak would matter.** Making a rail magnetically invisible next to MRI, electron microscopes, quantum instruments or magnetic-anomaly-sensitive sites. Stray-field limits for people do not need one: a Halbach array's one-sided field B0·e^(−2πz/λ) falls from 1 T to 0.5 mT within about 1.21·λ (48 mm for λ = 40 mm).

Missing orders: log10(R2 / 1 cm), floored at 0; the label is 🧪 up to R2 = 1 cm and 🔭 above.

Sources: frontier research §3e; [R210], [R211].

### Section §F9: Diamagnetic levitation

**Status.** ✅ as physics (graphite levitation is textbook); FLOAT is research with centimetre prototypes.

**Model** (`diamagnetic`; FAB-FRN-009):

```
p = abs(χ)·B²/(2μ0)        179 Pa at 1 T for pyrolytic graphite (χ⊥ ≈ −4.5e-4 [REF-MEM])
```

This is an upper bound (the thick-body limit, with the whole B² acting on a diamagnet that fills the field region). The gross areal mass it lifts is p/g: 110 kg/m² under lunar gravity, which is consistent with FLOAT's claimed 33 kg/m² of payload, but only 18 kg/m² on Earth. At the same field an EMS rail pulls B²/2μ0, so diamagnetic lift is abs(χ) = 4.5e-4 of it: 3.3 orders below EMS.

Sources: frontier research §3g; XCRIT §6; [R27], [R214].

### Section §F10: Documented-only items

These have models for the record but no requirement rows (§1: "documented only").

**NV-centre and atom-interferometer sensing** (`quantum`). A field sensor in the gap reads the gap through the field slope, δg = g0·δB/B. For DRAWER (g0 2 mm, B 0.4 T, 2 kHz) the 0.3 µm noise budget allows δB ≤ 60 µT rms, about 1.3 µT/√Hz; an NV magnetometer at 10 nT/√Hz adds about 2 nm, so quantum sensing is over-specified for gap control (rail roughness and PWM ripple dominate). At 0.4 T the NV resonance moves from 2.87 GHz to about 14 GHz and off-axis fields cost contrast [UNCERTAIN], so NV belongs in fringe-field spots. The real frontier role of quantum sensors is PEGS gradiometry (§F4) and drift-free tilt and gravity references; wiring noise presets into the core gap loop is roadmap.

**DAS along the rail** (`das_rail`, 🧪). Distributed acoustic sensing turns a fibre into strain channels about 10 m long over 40–125 km. A fibre laid along a long rail is a dense array: beamforming over an aperture L resolves the apparent slowness from the moveout Δt = L/v_app (17 ms over 100 m at 6 km/s), which at 1 kHz gives back-azimuth and slowness within about a second. A DAS feed enters fusion as one feed kind, `Das`, never as a sole trigger for motion (EXOTIC §X10).

**Safe-RL haptics with a CBF filter** (`safe_rl`, 🧪). Reinforcement learning could tune the feel of a door, but only inside a control-barrier-function filter that holds the pedestrian caps whatever the policy commands (Ames et al. 2019 [REF-MEM]).

- The carriage is a 1-D mass: v⁺ = a·v + (dt/m)·u with a = 1 − b·dt/m.
- Barriers h_E = E_max − ½·m·v² and h_p = p_max − m·abs(v) must satisfy h⁺ ≥ (1 − γ)·h, which bounds abs(v⁺) ≤ v_b = min(√(v² + 2γ·h_E/m), abs(v) + γ·h_p/m); the actuator also stays within the static force cap (unsensed, or sensed when presence sensing is fitted).
- The QP min (u − u_RL)² under these linear bounds is scalar, so its exact solution is the projection u = clip(u_RL, lo, hi). With h ≥ 0 the set always contains u = 0, so the filter is always feasible and, on the model it filters, never lets energy, momentum or force exceed their caps.
- The caps are read from `fabrail_params::presets` through the native module, never restated (FAB-EXO-114). The learned part is never safety-rated; the independent integer monitor (PHYSICS §18) stays in place.

Machine learning is bounded by physics here as well: early moment-rate functions look alike whatever the final size (Meier et al. 2017 [REF-MEM]), so no model can predict a final magnitude from the first seconds; it can pick phases better and track magnitude as it grows.

**Narrative only.** Optical clocks (centimetre height resolution, useful for slow subsidence monitoring), atmospheric Lamb waves (the far-field pressure pulse of Hunga Tonga 2022) and topological seismic waveguides are mentioned in the narrative and not modelled.

Sources: frontier research §3f and §3h; [R187], [R202], [R213].
