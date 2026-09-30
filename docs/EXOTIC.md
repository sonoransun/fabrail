# fabrail Sovereign/Exotic tier

This document holds three things for the Sovereign/Exotic tier: its specification (FAB-EXO-###), its physics (Section §X) and its safety argument (Section §S). It uses the grammar, columns, methods, statuses and test tokens of [SPECIFICATION.md §0](SPECIFICATION.md) and is parsed by the same traceability guard. The theoretical 🔭 tier is in [FRONTIER.md](FRONTIER.md).

> **Status: Wave 1.** IDs, tokens, statuses, the class table and the anchors are frozen. The §X physics and §S safety prose describe the models as implemented in Wave 1; where prose and code disagree, code wins and the prose is corrected. Numbers in generated tables are regenerated from code and drift-guarded (FAB-EXO-121).

Code cites this document as `EXOTIC §Xn`, in the same way core code cites `PHYSICS §n`.

---

## §0 Rules of the tier

### §0.1 Badges and maturity

- The badge is the core status: ✅ V, 🔶 S, 🧪 R, 🗺️ M (SPECIFICATION §0.5). 🔭 replaces it whenever the technology's maturity is frontier.
- Technology maturity (TRL) is a separate column (§1). It is never merged into the badge.
- A capability's badge never exceeds the weakest requirement it depends on.

### §0.2 Safe by separation, not by energy limit

- At 500 t the pedestrian caps allow only 2.6 mm/s (energy, 1.69 J) or 0.045 mm/s (momentum, 22.3 kg·m/s). Exotic classes therefore leave the pedestrian regime.
- Each class declares a `SafetyRegime`: `GuardedZone` (PL e presence sensing and interlocks, ISO 13855 separation) or `OccupiedPlatform` (occupant comfort limits).
- The pedestrian caps are imported from `fabrail_params::presets` (`PEDESTRIAN_E_MAX_J`, `PEDESTRIAN_P_MAX_KGMPS`), never restated (FAB-EXO-114). Exotic code never changes the core `RailClass` or its caps (FAB-EXO-006).
- In the Class column of §3, a regime name (GuardedZone, OccupiedPlatform) means every class in that regime.

### §0.3 Vocabulary lists (quoted for the guards)

Research-tier and 🔭 rows never contain:

> `certified`, `guarantee`, `protects against`, `SIL 4 compliant`, `prevents collapse`

The prose of EXOTIC.md, FRONTIER.md, sovereign.html and frontier.html never contains, outside quoted prompt blocks:

> `perfect`, `perfectly`, `absurdly`, `guarantee`, `earthquake-proof`, `no damage`, `protects`

### §0.4 The alert rule

- "Toward safety" is a per-class partial order, `SafetyRank`: Seated or Landed+Locked > Closed-levitated > Standoff/Hold > Frozen-open.
- Network-only or injected alerts may cause only FREEZE, LAND, LOCK or SEISMIC_PASSIVE.
- Alert-triggered **motion** needs independent confirmation: a local P detection, or at least 2 independent feeds. In addition:
  - sovereign classes need `ClearVerified`;
  - pedestrian classes (the villa drawers and doors) move only within their low-energy caps, with obstruction detection active.
- Land-first is not universal: SENTINEL rides through levitated; TIDEGATE closes after the shaking ends, and only on command from an external decision system; BASTION completes its close or lands.

### §0.5 Non-claims (expanded in §S5)

- Nothing here is certified at any SIL or PL; architectures are "designed toward" those levels. Ferrocene qualification of a compiler target does not certify an application. One lockstep chip is SIL 3 at most.
- Mass dampers are mainly wind devices. At design-level shaking they add at most about 0.7 % damping.
- Maglev vertical isolation is not proposed. Whole-building levitation is not proposed.
- No yield-to-distance blast tables. fabrail is not an EEW provider; every alert in this tier is synthetic.
- Frontier items are theoretical and never products.
- Simulation is not validation: there is no hardware and no FEM.

---

## §1 Classes

`ExoClass` is plain data in `fabrail-exo-safety`. GZ = `GuardedZone`; OP = `OccupiedPlatform`; PPS = personnel protection system; PSR = permanently shadowed region.

| Class | Design mass | Suspension and key parameters | Regime / safe state | Maturity | This effort |
|---|---|---|---|---|---|
| SENTINEL tower HMD | 1,000 t, ±1.5 m, T 7 s | FeCo-pole EMS planar bowl as cross-stacked 1-D stages; F_max 24–121 kN (33–163 kW); ironless LSM trim; passive eddy damping | GZ / RideThrough | TRL 3–4 | Dynamic simulation: 2-DOF plus modal tower |
| HAVEN-R isolation | 50 t, T 6 s (R 8.95 m), stroke = S_D at MCE (±0.9 m) | Hybrid EMS (HTS option in 1-D stages); passive vertical support; copper damper with σ(T) | OP (a_peak ≤ 0.1 g at DBE as an analysis output; moat covers plus exclusion) / IsolateThenLandFps | TRL 2–3 | Dynamic simulation: 3-DOF planar |
| HAVEN-F facility | 5 kt | As HAVEN-R; always levitated | OP | R (extrapolated) | Data plus algebraic admission; villa building config 🧪 (reduced platform) |
| SHIELD | 50 t (5–500 t) | Hybrid EMS, passive mover | GZ + PPS / LandAndLock ≤ 2 s | TRL 3 | Data, admission, closure/land timeline |
| BASTION | 50 t; 3 m in 5.25 s (a 0.48, j 2) or 5.0 s at a 0.53 | Hybrid EMS, land-to-seal | GZ PL e / CompleteClose | TRL 2–3 | Closure-race planner (t_a is an input) |
| ARENA | 8,600 t in pods | Zero-power hybrid, FeCo | GZ / LandAndLock | TRL 3 | Data plus static WLS pod reallocation (dynamic simulation 🗺️) |
| ROTATOR | 6,100 t ring, 36 stator stations | Hybrid ring plus hooks; the absorption check currently fails and is reported | GZ / BrakeAndLand | TRL 3 | Data plus admission |
| TIDEGATE | 150 t | Hybrid plus wheels | GZ; decision path SIL 3 target (SIL 4 unclaimable) / CloseAfterShaking | TRL 2 (triggering 7–9) | Fusion, tsunami timing, closure planner |
| SELENE | 100 t hatch | Passive HTS in a PSR | Teleoperated / LandAndLock | 🔭 | numpy frontier only (FRONTIER.md) |

---

## §2 Anchors

Tests assert these values with the stated tolerance and source. Tables marked "generated" are rewritten from code.

| Quantity | Values | Tolerance |
|---|---|---|
| Den Hartog, μ 0.01 / 0.02 | f 0.99010 / 0.98039; ζ 0.06033 / 0.08407; A_max 14.177 / 10.050 (ζs = 0) | (E); sweep ±2 % |
| Warburton, base acceleration | 0.98762 / 0.04981 and 0.97548 / 0.07019 | (E) |
| Warburton, force (wind) | f 0.99257 / 0.98528 | (E) |
| Wind RMS reduction | reported against ζs (44.7 % at μ 1 %, ζs 1 %) | reported |
| Stroke bound ζ_add ≤ (μ/2)(S/X) | 3.5 % at S/X = 7; 0.7 % at 1.4 | +0.002 |
| HTS fit | 300 kgf at 20 mm and 550 kgf at 15 mm | ±1 % |
| HTS out of sample | 53 kPa at 10 mm; 5,000 N over 912 cm² at 5–8 mm | ±15 %; ±30 % |
| HTS area needed at 10 mm (generated) | SENTINEL 1,000 t: 185 m² (EMS at 0.3 MPa: 33 m²); HAVEN-R 50 t: 9.3 m²; HAVEN-F 5 kt: 925 m² | ±1 % |
| LN₂ hold | 40 h static, 30 h dynamic | ±10 % |
| Isolation at T = 4 s, ζ 0.2 | passive / skyhook: 0.5 Hz 0.4125 / 0.3221; 1 Hz 0.1251 / 0.0663; 2 Hz 0.0531 / 0.0159 | ±2 % |
| Isolation at T = 6 s | R = 8.95 m; a_peak reported as analysis | ±0.5 % |
| Ride-through, 10 kt, 300 s | 51 MJ hybrid at 0.2 g (102 MJ with 2×); pure EMS 5.1 GJ, flagged insufficient | exact |
| EMALS-class disk | 121 MJ delivered per cycle (6400 → 5205 rpm), ≈ 357 MJ stored | exact |
| Flywheel usable fraction | 75 % at ω_min = ω_max/2 | exact |
| Lead-time gain | 0.15385 s/km (V_p 6.5), 0.27027 s/km (V_s 3.7) | 1e-9 |
| Friedlander i/(Ps·td) | 0.4261 (b 0.5), e⁻¹ = 0.36788 (b 1), 0.2838 (b 2), 0.1603 (b 5) | 1e-9 |
| Reflection factor Pr/Ps | 2.008 at 1 kPa, 2.741 at 100 kPa, 7.60 at 10 MPa, → 8 | 1e-9 |
| Tsunami | c = √(g·h) = 198 m/s at 4 km; Green's law ×4.47 from 4,000 m to 10 m | 1e-9 |
| PFH, reference case | conservative 1.04e-8/h; IEC 61508-6 Ed. 2 ≈ 2.0e-10/h; CTMC within ±20 % | ±2 % |
| ISO 13855 separation S | SHIELD 2.61 m, BASTION 5.01 m, ARENA 10.61 m | exact |
| Blast feasibility | a BASTION close (5.25 s + 0.6 s) needs R ≥ 2 km | exact |
| SENTINEL kinematics, T 7 s, ±1.5 m | 1.346 m/s, 1.209 m/s², 1.085 m/s³ | (E) |
| Curved guideway sagitta L²/8R | 7.9 mm at L 0.5 m, R 3.98 m | (E) |

---

## §3 Requirements (EXO)

### §3.0 Tier rules and guards

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-001 | Every exotic capability shall carry exactly one badge (the core status V/S/R/M, replaced by 🔭 when maturity is frontier) plus a separate maturity/TRL column; frontier rows cite their anchor tier and the missing orders of magnitude | all | USER 5; XCRIT §6 | I | `py:exotic/exo_guards::one_badge_per_capability` | V(I) |
| FAB-EXO-002 | Frontier code shall exist only in python/fabrail/exotic/frontier/ (numpy only); no Rust crate contains a frontier model | all | USER 5; PLAN §4; XCRIT §4 | I | `py:exotic/exo_guards::frontier_code_numpy_only` | V(I) |
| FAB-EXO-003 | fabrail-safety and fabrail-exo-safety shall declare dependencies on fabrail-types and fabrail-params only; control crates may depend on safety types, never the reverse | all | IEC 61508-3; XCRIT P0-6 | I | `py:exotic/exo_guards::safety_manifests_types_params_only` | V(I) |
| FAB-EXO-004 | Every frontier output (stdout, JSON with "tier":"frontier", plot watermark) shall carry the 🔭 banner and its missing-orders figure | all | USER 5 | T | `py:exotic/frontier::every_output_bannered` | V |
| FAB-EXO-005 | Research-tier and 🔭 rows shall avoid the first §0.3 list, and the prose of EXOTIC.md, FRONTIER.md, sovereign.html and frontier.html shall avoid the second list outside quoted prompt blocks | all | USER 5; PLAN §4b | I | `py:exotic/exo_guards::vocabulary_guard` | V(I) |
| FAB-EXO-006 | Exotic code shall never change the core RailClass or pedestrian caps, and GuardedZone shall not be assignable to DRAWER, DOOR or WALL | all | CRIT; XCRIT P1-19 | T | `rs:exo-safety/types::guarded_zone_not_for_pedestrian_classes`; `py:exotic/exo_guards::core_caps_untouched` | V |

### §3.1 Classes, admission and cross-checks

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-101 | ExoClass presets shall reproduce the §1 class table exactly (design mass, suspension, regime, safe state, maturity) | all | SA; SS; PLAN §4 | T | `rs:exo-safety/presets::class_table_matches_plan` | V |
| FAB-EXO-102 | Each class shall declare its SafetyRegime, safe-state policy (mapped onto core SeismicAction, including RideThrough, CompleteClose and CloseAfterShaking), seismic role and rest policy | all | SA; XCRIT §3 | T | `rs:exo-safety/presets::every_class_declares_regime_and_policy` | V |
| FAB-EXO-103 | Pole area shall come from one admission function A = k·m·g/(η_fill·min(B_pole, B_rail, B_gap,max)²/2μ0), with k = 1.5 and B_gap,max ≈ 1.0–1.4 T for 10–15 mm PM-biased gaps, reported per class | all | ET; XCRIT P1-9 | A, T | `rs:exo-sim/suspension::admission_single_function`; `py:exotic/classes::admission_table_per_class` | S |
| FAB-EXO-104 | The stray field at accessible surfaces shall be ≤ 0.5 mT, or a shroud flag shall be raised | all | CRIT G13 | A | `py:exotic/classes::stray_field_or_shroud_flag` | S |
| FAB-EXO-110 | Class kinematic and acceleration budgets shall come from closed forms and be reported (SENTINEL at T 7 s, ±1.5 m: 1.346 m/s, 1.209 m/s², 1.085 m/s³; HAVEN a_peak = ω²·S_D·√(1 + 4ζ²)) | SENTINEL/HAVEN-R | XCRIT §5 | A | `py:exotic/classes::acceleration_budget_closed_form` | V |
| FAB-EXO-111 | Curved-guideway admission shall check pole sagitta L²/8R ≤ 10 % of the gap (7.9 mm at L 0.5 m, R 3.98 m) and require curved pole faces and articulated pods for tilt up to 11°; attractive 2-D travel uses cross-stacked 1-D stages | SENTINEL/HAVEN-R | XCRIT P1-10 | A, T | `rs:exo-sim/platform::guideway_sagitta_tilt_admission` | S |
| FAB-EXO-112 | Integer units (mass in kg u32, velocity in mm/s i32, 2·KE in u64 µJ, momentum in u64 kg·mm/s, force in N i64) shall neither overflow nor wrap from 1 kg to 1e8 kg (proptest) | all | XCRIT P0-5 | T | `rs:exo-safety/units::integer_range_1kg_to_1e8kg` | V |
| FAB-EXO-113 | The HTS model shall pass the out-of-sample checks: 53 kPa at 10 mm (±15 %) and 5,000 N over 912 cm² at 5–8 mm (±30 %) | SENTINEL/HAVEN-R | XCRIT P0-7 | T | `rs:exo-sim/suspension::hts_out_of_sample` | S |
| FAB-EXO-114 | Exotic code shall import the pedestrian caps from fabrail_params::presets and never restate the literals 1.69 or 22.3 | all | PLAN §4; XCRIT P1-19 | I | `py:exotic/exo_guards::pedestrian_caps_single_source` | V(I) |
| FAB-EXO-115 | For every SuspensionKind the no-power safe state shall be reachable: hybrid EMS fails as Diverge{Indeterminate} with both stops designed as sliding surfaces (upper ≤ 1.3 W, lower FPS or rollers), and landing is an energised function with a monitored reserve and a mechanical backstop | all | XCRIT P0-2, P0-3 | A, T | `rs:exo-sim/suspension::no_power_safe_state_reachable` | S |
| FAB-EXO-116 | The transitive dependency closure of fabrail-safety and fabrail-exo-safety (cargo metadata, normal and build edges) shall contain no fabrail-core, fabrail-exo, fabrail-plant, fabrail-sim or fabrail-exo-sim | all | XCRIT P0-6 | T | `rs:exo-safety/tests/dependency_closure::closure_excludes_control_and_plant` | V |
| FAB-EXO-117 | The PFH calculator shall agree with a CTMC Markov model within ±20 % | all | XCRIT P1-15 | T | `rs:exo-safety/pfh::ctmc_cross_check` | V |
| FAB-EXO-118 | The copper damper shall use σ(T) = σ0/(1 + 0.00393·ΔT), coupled into its damping coefficient | HAVEN-R/HAVEN-F | XCRIT P1-18 | T | `rs:exo-sim/platform::copper_sigma_temperature` | S |
| FAB-EXO-119 | The alert-spoof matrix shall hold: network-only or injected alerts cause only FREEZE, LAND, LOCK or SEISMIC_PASSIVE; alert-triggered motion needs independent confirmation (local P or ≥ 2 feeds) plus ClearVerified for sovereign classes, or the low-energy caps with obstruction detection for pedestrian classes | all | PLAN §4; XCRIT P0-4 | T | `rs:exo-safety/alert_gate::spoof_action_matrix` | V |
| FAB-EXO-120 | Ride-through sizing shall use T_ride = max(300 s, the class scenario's shaking duration) + landing | all | XCRIT P1-12 | T | `rs:exo/energy::t_ride_covers_shaking_duration` | V |
| FAB-EXO-121 | The admission, HTS-area and anchor tables in EXOTIC.md shall be regenerated from code and drift-guarded | all | XCRIT P1-9 | I | `py:exotic/exo_guards::doc_tables_match_code` | V(I) |
| FAB-EXO-122 | Admission shall check absorption headroom (charge headroom plus dump or eddy-brake rating ≥ 1.2·KE_max); a failing class (ROTATOR today) is reported, not hidden | all | XCRIT P1-13 | A, T | `rs:exo/energy::absorption_headroom` | S |

### §3.2 Suspension models

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-201 | The SuspensionModel trait shall expose force and stiffness in z and y, a hysteresis state, failure behaviour, hover power and tier, and be implemented by the HybridEms adapter, PureEms, HTS and AirPad (FeCo is a HybridEms parameter) | all | ET; PLAN §4 | T | `rs:exo-sim/suspension::trait_contract_all_models` | S |
| FAB-EXO-202 | The HTS frozen-image model shall fit 300 kgf at 20 mm and 550 kgf at 15 mm within ±1 % | SENTINEL/HAVEN-R | ET; XCRIT P0-7 | T | `rs:exo-sim/suspension::hts_fit_atz` | S |
| FAB-EXO-203 | The HTS play-operator loop shall be conservative at η_h = 0 (abs(∮F dz) < 1e-9 J) and dissipative at η_h > 0 (∮ > 0, return branch below the approach branch) | SENTINEL/HAVEN-R | ET | T | `rs:exo-sim/suspension::hts_loop_conservative_and_dissipative` | S |
| FAB-EXO-204 | HTS creep (quasi-static only) and the s_T(T) law shall be implemented, with LN₂ hold of 40 h static (±10 %) and 30 h dynamic at the fitted heat load | SENTINEL/HAVEN-R | ET; XCRIT P0-7 | T | `rs:exo-sim/suspension::hts_creep_and_ln2_hold` | S |
| FAB-EXO-205 | HTS shall land at 10 % LN₂ with 4 % critical, and the post-depletion fade (tens of minutes, not hours) shall be modelled | SENTINEL/HAVEN-R | ET | T | `rs:exo-sim/suspension::hts_ln2_landing_and_fade` | S |
| FAB-EXO-206 | Hover loss shall reproduce 1.7 kW/t for pure EMS and 42 / 170 / 1,060 kW for hybrid EMS at 10 kt and 0.1 / 0.2 / 0.5 g RMS | all | ET | A, T | `rs:exo-sim/suspension::hover_loss_anchors` | S |
| FAB-EXO-207 | FeCo poles shall give P(2.3 T) = 2.10 MPa, with admission capped by the rail and gap flux (the min rule of FAB-EXO-103) | all | ET; XCRIT §1 | A, T | `rs:exo-sim/suspension::feco_pressure_and_rail_cap` | S |
| FAB-EXO-208 | HTS plus EMS trim shall hold creep drift ≤ 0.5 mm over 24 h in a quasi-static creep test | SENTINEL/HAVEN-R | ET; XCRIT §4 | S | `rs:exo-sim/suspension::hts_ems_trim_creep_drift` | R |
| FAB-EXO-209 | Non-magnetic baselines (air pad, FPS) shall be available for comparison | all | SA | T | `rs:exo-sim/suspension::baselines_available` | S |
| FAB-EXO-210 | No Rust suspension kind shall model a frontier or research-only variant (RTSC, trapped-field bulk, diamagnetic); these exist only in the numpy frontier module | all | USER 5; PLAN §4 | I | `py:exotic/exo_guards::no_rust_frontier_suspension` | V(I) |

### §3.3 Redundancy

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-301 | The 2oo3 mid-value select shall keep any single-channel fault from moving the output by more than e_th (property test) | all | IEC 61508-2 | T | `rs:exo-safety/vote::mvs_single_fault_bound` | V |
| FAB-EXO-302 | The degradation ladder 2oo3 → 1oo2D → 1oo1D → safe state shall step within the t_persist bounds | all | IEC 61508-2 | T | `rs:exo-safety/ladder::degradation_sequence` | V |
| FAB-EXO-303 | A common-cause double fault shall reach the safe state | all | IEC 61508-2 | T | `rs:exo-safety/ladder::common_cause_reaches_safe_state` | V |
| FAB-EXO-304 | The diverse inductance channel L(g) shall act as a tie-breaker only | all | ET | T | `rs:exo-safety/ladder::diverse_tiebreaker` | R |
| FAB-EXO-305 | The PFH calculator shall give both forms for the reference case (λ_D 1e-6/h, DC 99 %, β 2 %, β_D 1 %, T1 8760 h, MTTR = MRT = 8 h): conservative 1.04e-8/h and IEC 61508-6 Ed. 2 ≈ 2.0e-10/h, each within ±2 % | all | ET; IEC 61508-6; XCRIT P1-15 | A, T | `rs:exo-safety/pfh::two_forms_anchor` | V |
| FAB-EXO-306 | The black channel (CRC-32, sequence counter, timeout, source ID) shall reject corrupted, stale and replayed frames | all | IEC 61784-3 | T | `rs:exo-safety/black_channel::rejects_corrupt_stale_replay` | V |
| FAB-EXO-307 | The fault-injection feature and the Injector shall be absent from every firmware feature set and lockfile | all | PLAN §4; XCRIT §3 | I | `py:firmware::no_fault_injection_in_firmware` | V(I) |

### §3.4 Energy

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-401 | Storage models (flywheel, supercap, battery; SMES documented only) shall give the flywheel usable fraction 1 − (ω_min/ω_max)² | all | ET | A, T | `rs:exo-sim/storage::flywheel_usable_fraction` | S |
| FAB-EXO-402 | A move shall start only if the zone is ClearVerified, E_usable ≥ 2·(E_stop + E_land + P_hold·T_ride), absorption headroom holds (FAB-EXO-122) and the bumper rating is ≥ ½·m_rated·v_max² | all | SS; XCRIT P1-13 | T | `rs:exo/energy::admission_discharge_and_absorb` | V |
| FAB-EXO-403 | Ride-through anchors shall hold: 10 kt hybrid EMS for 300 s at 0.2 g needs 51 MJ (102 MJ with the 2× margin); pure EMS needs 5.1 GJ and is flagged insufficient against one JET-class unit; an EMALS-class disk delivers 121 MJ per cycle (≈ 357 MJ stored) | all | ET; XCRIT P1-11, P1-12 | A, T | `rs:exo/energy::ride_through_anchors` | V |
| FAB-EXO-404 | The regeneration and dump energy balance shall close within 1 % (f64 accumulators on the host, i64 mJ on the target) | all | SS; XCRIT P0-5 | T | `rs:exo-sim/storage::energy_balance_closes` | V |
| FAB-EXO-405 | HTS ride-through shall need only kW-scale power (sensors and propulsion) | SENTINEL/HAVEN-R | ET | A | `rs:exo/energy::hts_ride_through_kw` | S |

### §3.5 Zones and regimes

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-501 | Motion shall be possible only in ClearVerified (two diverse presence channels clear, access interlocks locked, manual reset from outside the zone) | GuardedZone | ISO 13849-1; ISO 14119 | T | `rs:exo-safety/zone::motion_only_when_clear_verified` | V |
| FAB-EXO-502 | Intrusion during motion shall trigger SS1 within the analytic stop distance d_stop = v·t_react + v²/(2·a_brake) + v·a/(2j), with the ISO 13855 separation S = 2.61 / 5.01 / 10.61 m (SHIELD / BASTION / ARENA) reported | GuardedZone | ISO 13855 | A, T | `rs:exo-safety/zone::intrusion_ss1_stop_distance`; `py:exotic/classes::iso13855_distances` | V |
| FAB-EXO-503 | Attended jog shall be limited to ≤ 10 mm/s and ≤ 1.69 J at rated mass (the imported cap), with a force cap and a compliant edge (≤ 10 N/mm); momentum stays uncapped and is stated (SHIELD 411 kg·m/s) | GuardedZone | ISO 12100; XCRIT P1-19 | T | `rs:exo-safety/envelope::attended_jog_limits` | V |
| FAB-EXO-504 | Bumper rating shall be ≥ ½·m_rated·v_max² using rated mass, never m̂ (HAVEN-F included: 3.6 MJ at 1.2 m/s) | all | CRIT; XCRIT §1 | T | `rs:exo-safety/envelope::bumper_rating_rated_mass` | V |
| FAB-EXO-505 | HAVEN occupied-platform limits shall be reported as analysis outputs: a_peak = ω²·S_D·√(1 + 4ζ²) ≤ 0.1 g at DBE for the site S_D1 (T 6 s) and landing ≤ 0.1 m/s, with MCE and near-fault values reported, not bounded; the moat uses passive covers plus exclusion, and presence sensing gates controlled moves only | HAVEN-R | SA; XCRIT P0-1, P1-20 | A, S | `rs:exo-sim/platform::occupied_platform_limits` | R |
| FAB-EXO-506 | "Toward safety" shall be the SafetyRank partial order (§0.4), and an alert shall move a device only toward a higher rank, verified by an exhaustive transition test | all | SS; PLAN §4, §4b; XCRIT P0-4 | T | `rs:exo-safety/rank::alert_moves_only_toward_higher_rank`; `rs:exo-safety/alert_gate::exhaustive_transitions` | V |
| FAB-EXO-507 | Presence sensing shall win over closure unless a two-person override is active | BASTION/TIDEGATE | SA | T | `rs:exo-safety/zone::presence_wins_without_two_person_override` | V |

### §3.6 Mass dampers

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-601 | Den Hartog and Warburton tuning (base-acceleration and force forms) shall match the closed forms (E): Den Hartog f 0.99010 / 0.98039 and ζ 0.06033 / 0.08407 at μ 0.01 / 0.02; Warburton base 0.98762 / 0.04981 and 0.97548 / 0.07019; Warburton force f 0.99257 / 0.98528; ζ normalised to ω_s | SENTINEL | SS; XCRIT P1-16 | A, T | `rs:exo/tuning::den_hartog_warburton_closed_forms` | V |
| FAB-EXO-602 | A harmonic sweep of the tuned 2-DOF system with ζs = 0 shall peak at √(1 + 2/μ) ±2 % (14.177 / 10.050) | SENTINEL | SS; XCRIT P1-16 | S | `rs:exo-sim/host::harmonic_sweep_peak` | V |
| FAB-EXO-603 | Under saturation the added damping shall satisfy ζ_add ≤ (μ/2)(S/X) + 0.002 (3.5 % at S/X = 7, 0.7 % at 1.4) | SENTINEL | SS | S | `rs:exo-sim/host::zeta_add_stroke_bound` | R |
| FAB-EXO-604 | Stroke shall never exceed s_max, with the stroke governor (cubic ramp from 0.7·s_max) engaging first in the stroke → power → force → rate stack | SENTINEL | SS | T, S | `rs:exo/hmd::stroke_governor_engages_first` | R |
| FAB-EXO-605 | The mode ladder HMD_ACTIVE → SEISMIC_PASSIVE → DVF_ONLY → PASSIVE_EDDY → LANDED_FPS shall hold, and on power loss the failure behaviour shall be Diverge{Indeterminate} onto designed sliding stops, with no clamp-up claim | SENTINEL | SS; XCRIT P0-2 | T | `rs:exo/hmd::mode_ladder_and_fail_behaviour` | R |
| FAB-EXO-606 | For a near-fault pulse the controlled host peak shall be ≤ 1.02× the uncontrolled peak | SENTINEL | SS | S | `rs:exo-sim/scenarios::near_fault_pulse_peak` | R |
| FAB-EXO-607 | Under 3–7 mg wind the host RMS reduction shall be reported against ζs (44.7 % at μ 1 %, ζs 1 %), the ≥ 40 % target shall apply to active mode only, activation shall be ≤ 0.3 mg, and the SENTINEL trim LSM shall be ironless | SENTINEL | SS; XCRIT P1-17 | S | `rs:exo-sim/scenarios::wind_rms_and_activation` | R |
| FAB-EXO-608 | A ±10 % detuning of ω_s shall stay stable, with the degradation reported | SENTINEL | SS | S | `rs:exo-sim/scenarios::detuning_stability` | R |
| FAB-EXO-609 | The Coulomb activation threshold a = μ·g shall be reproduced (no slip at 0.9·μg, slip at 1.1·μg) and the maglev hysteresis-drag variant reported, with the rail condition (low-coercivity or homopolar) stated | SENTINEL | SS; CRIT G7 | S | `rs:exo-sim/host::coulomb_activation_threshold` | S |

### §3.7 Isolation

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-701 | The curved-guideway period shall satisfy T = 2π·√(R/g) ±0.5 % (HAVEN-R T 6 s, R 8.95 m) | HAVEN-R | SS; XCRIT P0-1 | A, T | `rs:exo-sim/platform::curved_guideway_period` | V |
| FAB-EXO-702 | Transmissibility at T = 4 s, 1 Hz, ζ = 0.2 shall be 0.1251 passive and 0.0663 skyhook (±2 %) | HAVEN-R | SS | A, T | `rs:exo/isolation::transmissibility_anchors` | V |
| FAB-EXO-703 | The design stroke (S_D at MCE, ±0.9 m) shall fit within the moat, the fling case shall lose no support, and bumper energy is quoted in MJ | HAVEN-R | SS; XCRIT P0-1 | S | `rs:exo-sim/scenarios::stroke_within_moat_and_fling` | R |
| FAB-EXO-704 | Control loss shall end in an FPS landing at ≤ 0.1 m/s through a PM-bias ramp | HAVEN-R | SS | S | `rs:exo-sim/platform::control_loss_fps_landing` | R |
| FAB-EXO-705 | A platform configuration without passive vertical support shall be rejected for m > 1 t (maglev vertical isolation is not proposed) | HAVEN-R/HAVEN-F | SS | T | `rs:exo-sim/platform::rejects_no_passive_vertical` | V |
| FAB-EXO-706 | The copper damper temperature rise shall be reported and kept ≤ its limit | HAVEN-R/HAVEN-F | SS | S | `rs:exo-sim/platform::copper_temperature_rise` | S |
| FAB-EXO-707 | Recentring bandwidth shall be ≤ 0.1× the isolation frequency and off in SHAKING, and pre-positioning shall happen only with independent confirmation and a ClearVerified moat | HAVEN-R | SS; PLAN §4 | T | `rs:exo/isolation::recentring_band_and_shaking_off` | R |

### §3.8 Early warning and pressure waves

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-801 | AlertFeedKind shall be {LocalP, Network, OffshoreCabled, Das, PegsMagnitudeAdvisory}; the Injector shall exist only under cfg(feature = "fault-injection"); fusion shall emit a synthetic core NetworkAlert plus an exo extension through the ActionPolicy hook | all | SS; XCRIT P1-23 | T | `rs:exo/fusion::feed_kinds_and_injector_gating` | V |
| FAB-EXO-802 | Fusion shall take the maximum predicted intensity with an S-ETA distribution, combine local detectors by capped log-odds, and act by the cost–loss rule | all | SS | T | `rs:exo/fusion::max_intensity_eta_logodds_costloss` | S |
| FAB-EXO-803 | The lead-time gain shall be ∂W/∂x = 0.15385 s/km (V_p 6.5) and 0.27027 s/km (V_s 3.7, PLUM) (rel 1e-9) with the blind-zone formula, and every output shall print its inputs | all | SS; XCRIT §1 | A, T | `rs:exo-sim/eew_timing::lead_time_gain_per_km` | V |
| FAB-EXO-804 | The offshore M8 warning time (land, S-net-like, DAS) shall be within ±1 s of the analytic value for the stated inputs | TIDEGATE | SS | S | `rs:exo-sim/eew_timing::offshore_m8_warning` | S |
| FAB-EXO-805 | A cancellation or a concurrent-quake false alarm shall return to NORMAL | all | SS | T | `rs:exo/fusion::cancel_returns_normal` | V |
| FAB-EXO-806 | The Friedlander impulse shall match its closed form (rel 1e-9; b = 1 gives e⁻¹) and the reflection factor Pr/Ps shall give 2.008 / 2.741 / 7.60 (→ 8) | BASTION | SS | A, T | `py:exotic/pressure::friedlander_and_reflection` | V |
| FAB-EXO-807 | The closure-race planner shall CLOSE iff the jerk-aware t_min + t_seal + margin ≤ t_a − t_detect, with t_a an input and the bound t_a ≤ R/c0 reported (BASTION 3 m in 5.25 s at a 0.48, j 2; a close needs R ≥ 2 km) | BASTION/TIDEGATE | SS; XCRIT P1-14, P1-22 | T | `rs:exo/closure::close_iff_feasible` | V |
| FAB-EXO-808 | No yield-to-distance table or scaled-distance blast law shall appear in code, outputs or docs | all | policy; XCRIT P1-22 | I | `py:exotic/exo_guards::no_yield_distance_tables` | V(I) |
| FAB-EXO-809 | Tsunami timing shall follow ∫dx/√(g·h) and Green's law (198 m/s at 4 km; ×4.47 from 4,000 m to 10 m) with a closure budget and a wheel fallback on rail distortion; TIDEGATE closes only after the shaking and only on the external decision-system command | TIDEGATE | SS; PLAN §4 | A, T | `rs:exo-sim/tsunami::timing_greens_law_budget` | S |
| FAB-EXO-810 | PEGS shall never be the sole trigger: only a network magnitude advisory enters fusion, and the on-site gradiometer exists only in the numpy frontier module | all | FT; XCRIT P1-23 | T | `rs:exo/fusion::pegs_never_sole_trigger` | V |

### §3.9 Firmware and assurance

| ID | Shall | Class | Source | V | Verified by | St. |
|---|---|---|---|---|---|---|
| FAB-EXO-901 | fabrail-exo-safety and fabrail-exo shall cross-build for thumbv7em-none-eabihf, and fabrail-exo-safety also for thumbv6m-none-eabi and armv7r-none-eabihf (the FAB-FW-013 gate) | all | ET; PLAN §2b.14 | T | `py:firmware::cross_check` | V |
| FAB-EXO-902 | fabrail-exo-safety and fabrail-exo shall be no_std, forbid(unsafe_code) and allocation-free | all | PLAN §2 | I | `py:firmware::nostd_source_guard` | V(I) |
| FAB-EXO-903 | The integer envelope shall equal the f64 reference within 1 LSB (Q16.16 is not used) | all | XCRIT P0-5 | T | `rs:exo-safety/envelope::matches_f64_reference` | V |
| FAB-EXO-904 | A WCET budget (MVS vote ≤ 2 µs; safety cycle ≤ 50 µs on a 400 MHz R5F) shall be measured on hardware | all | ET | D | — | M |
| FAB-EXO-905 | The architecture text shall say "designed toward, not certified" and state that Ferrocene target qualification does not certify the application, that one lockstep chip is SIL 3 at most, and that TMS570 (BE32) cannot be targeted | all | ET; XCRIT §3 | I | `py:exotic/exo_guards::designed_toward_wording` | V(I) |

**Source codes used here** (in addition to SPECIFICATION §0.6): ET, SA, SS and FT are the exotic-technology, sovereign-applications, sovereign-seismic and frontier-theory research reports; XCRIT is the exotic adversarial critique (P0-n, P1-n, §n); USER 5 is the user's frontier-tier decision.

---

## Physics (Section §X)

This part specifies the exotic models exactly as implemented: the f64 truth models in `fabrail-exo-sim`, the f32 controllers and estimators in `fabrail-exo`, the integer safety logic in `fabrail-exo-safety` and the numpy algebra in `python/fabrail/exotic/`. Code cites it as `EXOTIC §Xn`; the core physics it builds on is cited as PHYSICS §n.

- Where this text and the code disagree, the code wins and this text is corrected. Tables marked "generated" are rewritten from code and drift-guarded (FAB-EXO-121); do not edit them by hand.
- Every number carries its inputs. A value tagged [UNCERTAIN] comes from recall or from a single source; the studies sweep it instead of trusting it.
- Sources are named where they are used. XCRIT is the exotic adversarial critique, ET, SA, SS and FT the exotic-technology, sovereign-applications, sovereign-seismic and frontier-theory research reports (see the source codes under §3).

### Section §X1: Scope and tier rule

**What exists, and where.**

| Layer | Crate or package | Numerics | Contents |
|---|---|---|---|
| Integer safety | `fabrail-exo-safety` (no_std) | kg u32, mm/s i32, 2·KE in µJ u64, kg·mm/s u64, N i64 | Plain data (`ExoClass`, `SafetyRegime`, `SafeStatePolicy`, `SafetyRank`, `FusedAlert`), class presets, 2oo3 voter, degradation ladder, black channel, zone machine, envelope, alert gate, PFH calculator |
| Controllers and estimators | `fabrail-exo` (no_std) | f32; energy bookkeeping f64 | HMD with its saturation stack and mode ladder, TMD tuning, isolation skyhook, energy admission, closure-race planner, static pod allocation, EEW fusion, house choreographer |
| Truth models | `fabrail-exo-sim` (std) | f64 | Suspension kinds, storage, host structures, the HAVEN-R platform, ground-motion additions, EEW and tsunami timing, scenarios, the building scene |
| Algebra, CLI, studies | `python/fabrail/exotic/` | numpy | Class tables, admission, pressure and PFH helpers, the `fabrail exo` command |
| 🔭 frontier | `python/fabrail/exotic/frontier/` | numpy only | The what-if models of [FRONTIER.md](FRONTIER.md). No Rust crate contains a frontier model (FAB-EXO-002, FAB-EXO-210) |

The dependency rule of the core applies unchanged: `fabrail-exo-safety` depends on `fabrail-types` and `fabrail-params` only, and its transitive closure contains no control or plant crate (FAB-EXO-003, FAB-EXO-116).

**The tier rule.**

- **Badge.** The badge of a capability is the core status of the weakest requirement it depends on: ✅ V or V(I), 🔶 S, 🧪 R, 🗺️ M (SPECIFICATION §0.5). 🔭 replaces it only when the technology's maturity is frontier.
- **Maturity.** Maturity is the technology's readiness in the world (TRL), in its own column. A ✅ on a closed form says fabrail computes that closed form correctly; it says nothing about whether the machine exists. The Den Hartog tuning of SENTINEL is ✅, while the 1,000 t maglev damper that would use it is TRL 3–4.
- **Depth.** The "This effort" column of §1 states how deep fabrail goes for each class: dynamic simulation (SENTINEL: 2-DOF plus a modal tower; HAVEN-R: 3-DOF planar), algebraic admission and data (HAVEN-F, SHIELD, ARENA, ROTATOR), planners (BASTION, TIDEGATE) or numpy frontier only (SELENE). Dynamic simulation of ARENA, ROTATOR, TIDEGATE, SELENE and the full HAVEN-F is roadmap. A gallery example on a roadmap class runs on a reduced model and is labelled as such.

**Safe by separation, not by energy limit.** The pedestrian caps of the core (PHYSICS §16) allow v ≤ √(2·E/m) and v ≤ p/m:

| Mass | Energy-cap speed | Momentum-cap speed |
|---|---|---|
| 5 t | 26 mm/s | 4.5 mm/s |
| 50 t | 8.2 mm/s | 0.45 mm/s |
| 500 t | 2.6 mm/s | 0.045 mm/s |

A sovereign-class mover therefore cannot be safe by being slow and light. It is safe by keeping people out of reach while it moves (`GuardedZone`, §X14) or by carrying them within comfort limits (`OccupiedPlatform`, §X9). The caps themselves are imported from `fabrail_params::presets` and never restated in exotic code (FAB-EXO-114).

**What an unlimited budget does not buy.** The user's decision was that cost is no impediment. Money buys margin, redundancy, better materials and more storage. It does not move these bounds:

| Bound | Value | Section |
|---|---|---|
| Magnetic pressure | P ≤ min(B_pole, B_rail, B_gap,max)²/2μ0: 2.10 MPa at FeCo saturation, 1.02 MPa against a structural-steel rail, 0.57 MPa at a 1.2 T gap | §X2 |
| Mass-damper benefit | ζ_add ≤ (μ/2)(S/X): about 3.5 % in wind and 0.7 % at design-level shaking | §X6 |
| Warning time | ∂W/∂x = 1/V_p = 0.15385 s per km of sensor advance | §X10 |
| Vertical isolation | Passive only; active support circulates M·g·v_rel (294 MW at 100 kt and 0.3 m/s) | §X9 |
| Blast lead | A BASTION close needs t_a ≥ 5.85 s, so a blast source at R ≥ 2 km | §X11 |

#### §X1.1 Capability matrix

Exactly one badge per row, set by the rule above; maturity is separate. The villa house choreographer is in SPECIFICATION_EXAMPLES.md and the frontier items are in FRONTIER.md §1.

| Capability | Badge | Maturity / TRL | Where | Requirements |
|---|---|---|---|---|
| Tier rules, word guards, dependency closure | ✅ | tooling | tests/exotic guards; exo-safety closure test | FAB-EXO-001 to 006; FAB-EXO-116 |
| Class presets, regimes and safe-state policies as data | ✅ | data (classes TRL 1–4, §1) | exo-safety `presets`, `types` | FAB-EXO-101, FAB-EXO-102 |
| Single pole-area admission and its generated tables | 🔶 | closed form; FeCo poles TRL 5 | exo-sim `suspension` | FAB-EXO-103, FAB-EXO-121, FAB-EXO-207 |
| Stray field at accessible surfaces, or a shroud flag | 🔶 | dipole model, no FEM | `fabrail.exotic` | FAB-EXO-104 |
| Class kinematic and acceleration budgets | ✅ | closed form | `fabrail.exotic` | FAB-EXO-110 |
| Curved-guideway sagitta and tilt admission | 🔶 | geometry; articulated pods TRL 2–3 | exo-sim `platform` | FAB-EXO-111 |
| Integer units and the integer envelope | ✅ | software | exo-safety `units`, `envelope` | FAB-EXO-112, FAB-EXO-903 |
| HTS flux-pinning model (fit, loop, creep, LN₂, fade) | 🔶 | TRL 5–6 (ATZ demonstrators; one data set) | exo-sim `suspension` | FAB-EXO-113; FAB-EXO-202 to 205 |
| HTS lift with an EMS trim coil | 🧪 | TRL 3–4 | exo-sim `suspension` | FAB-EXO-208 |
| Hover loss at scale, pure and hybrid EMS | 🔶 | pure EMS TRL 9 (Transrapid); hybrid EMS at kt scale TRL 3 | exo-sim `suspension` | FAB-EXO-206 |
| No-power safe state for every suspension kind | 🔶 | stops and catchers TRL 9; the combination TRL 3 | exo-sim `suspension` | FAB-EXO-115 |
| Non-magnetic baselines (air pad, FPS) | 🔶 | TRL 9 | exo-sim `suspension`, `platform` | FAB-EXO-209 |
| Storage models and the energy ledger | 🔶 | flywheel, supercap, battery TRL 9; SMES documented only | exo-sim `storage` | FAB-EXO-401, FAB-EXO-404 |
| Ride-through sizing with T_ride ≥ 300 s | ✅ | closed form | exo `energy` | FAB-EXO-120, FAB-EXO-403 |
| Energy admission with the absorption check | 🔶 | closed form over simplified stores | exo `energy` | FAB-EXO-122, FAB-EXO-402, FAB-EXO-405 |
| 2oo3 mid-value voter, ladder, common-cause handling | ✅ | architecture in service elsewhere; fabrail logic only | exo-safety `vote`, `ladder` | FAB-EXO-301 to 303 |
| Diverse inductance channel L(g) as tie-breaker | 🧪 | TRL 3–4 (self-sensing) | exo-safety `vote` | FAB-EXO-304 |
| Black channel | ✅ | concept in service elsewhere | exo-safety `black_channel` | FAB-EXO-306 |
| PFH calculator in two forms with a CTMC cross-check | ✅ | analysis aid, placeholder rates | exo-safety `pfh` | FAB-EXO-117, FAB-EXO-305 |
| Guarded-zone machine, ISO 13855 distances, attended jog, bumper rating | ✅ | presence sensors and interlocks TRL 9 | exo-safety `zone`, `envelope` | FAB-EXO-501 to 504; FAB-EXO-507 |
| Alert gate, SafetyRank and the spoof matrix | ✅ | software | exo-safety `alert_gate`, `rank` | FAB-EXO-119, FAB-EXO-506 |
| TMD tuning closed forms and the harmonic sweep | ✅ | TMD practice TRL 9 | exo `tuning`; exo-sim `host` | FAB-EXO-601, FAB-EXO-602 |
| SENTINEL hybrid mass damper (saturation, mode ladder, wind, detuning) | 🧪 | TRL 3–4 (1,000 t maglev planar bearing); eddy damping TRL 9 | exo `hmd`; exo-sim `host` | FAB-EXO-603 to 608 |
| Coulomb activation threshold and rail condition | 🔶 | reduced SDOF model | exo-sim `host` | FAB-EXO-609 |
| HAVEN-R isolation platform, 3-DOF | 🧪 | TRL 2–3 | exo `isolation`; exo-sim `platform` | FAB-EXO-505; FAB-EXO-701 to 707; FAB-EXO-118 |
| HAVEN-F facility at 5 kt, admission only | 🧪 | research, extrapolated | `fabrail.exotic` | FAB-EXO-103 |
| ARENA static pod reallocation | 🧪 | TRL 3; dynamic simulation 🗺️ | exo `alloc` | — (static tests only) |
| EEW fusion: maximum intensity, capped log-odds, cost–loss | 🔶 | S-net and DAS feeds in service or early service; fusion simplified | exo `fusion` | FAB-EXO-801, FAB-EXO-802, FAB-EXO-805 |
| Lead-time gain and blind zone | ✅ | closed form | exo-sim `eew_timing` | FAB-EXO-803 |
| Offshore warning scenario | 🔶 | S-net in service; timing model simplified | exo-sim `eew_timing` | FAB-EXO-804 |
| PEGS advisory gating (never a sole trigger) | ✅ | PEGS magnitude research (Mw ≥ 8.3) | exo `fusion` | FAB-EXO-810 |
| Air-blast closed forms and the closure race | ✅ | Friedlander textbook; BASTION TRL 2–3 | exo `closure`; `fabrail.exotic` | FAB-EXO-806 to 808 |
| Tsunami timing and closure budget | 🔶 | triggering TRL 7–9; maglev gate TRL 2 | exo-sim `tsunami` | FAB-EXO-809 |
| no_std cross builds of the exotic crates | ✅ | built as rlibs, never flashed | scripts/check_nostd.sh | FAB-EXO-901, FAB-EXO-902 |
| Sovereign firmware apps (1oo2D, 2oo3) and WCET | 🗺️ | not built | — | FAB-EXO-904 |
| Assurance wording | ✅ | — | tests/exotic guards | FAB-EXO-905 |
| SELENE lunar hatch | 🔭 | TRL 1–2 | numpy frontier | FAB-FRN-006 |

#### §X1.2 Reference installations and what maglev changes

These are the movable-infrastructure precedents the classes are measured against. Figures are as reported in the sources named in the SA and SS research reports; "reported" marks a figure from a single secondary source.

| Installation | Moving mass | Bearing and drive today | Informs | What maglev changes | What it does not change |
|---|---|---|---|---|---|
| Taipei 101 tuned mass damper | 660 t steel sphere | Cable pendulum with 8 viscous dampers; about 1.0 m swing reported in Typhoon Soudelor (2015), ±1.5 m design | SENTINEL | Eddy or LSM damping replaces the hydraulics; damping becomes adjustable in software | The pendulum already has almost no threshold; the benefit is serviceability, not reach |
| Shanghai Tower damper | 1,000 t | Pendulum over a copper plate with NdFeB magnets; damping set by the magnet-to-copper gap; stroke-dependent damping limits overstroke (Lu et al. 2017); magnet count reported as 125 or about 1,800 [UNCERTAIN] | SENTINEL | Direct precedent: passive magnetic damping at 1,000 t, no power needed. fabrail's stroke governor copies its end behaviour | Eddy damping is already in service; maglev adds the planar bearing, not the damping |
| Citicorp Center (601 Lexington) damper | 366 t concrete block, about 2 % of modal mass | Sliding on pressurised oil-film bearings with springs and hydraulic actuators; the pumps lift it about 2 cm in about 3 min; it arms at 3 mg | SENTINEL | No pumps, no oil, no film stiction, and always afloat, so it can respond within an S−P window | The mass, stroke and restoring spring stay the same |
| Shanghai World Financial Center | 2 × 150 t active TMDs | Active in wind, passive in earthquakes | SENTINEL mode ladder | The SEISMIC_PASSIVE rung mirrors this practice | — |
| Kyobashi Seiwa building (1989) | 4.2 t + 1.2 t | First full-scale active mass damper; servo-hydraulic | SENTINEL | An LSM replaces the hydraulics | The mass ratio (about 1 %) still limits the benefit |
| ESO ELT dome (under construction) | 6,100 t rotating | 36 stationary trolleys on a ring; 118 seismic isolators under the pier; up to 2°/s | ROTATOR | Contactless ring bearing, no trolley wear or rolling noise, regenerative braking of about 6.9 MJ | Ring ovalisation and wind uplift against a 15–20 mm gap; the pier isolators stay |
| State Farm Stadium field tray | about 8,600 t | 542 wheel assemblies on 13 rails, 76 × 1 hp motors, about 1 h per move; μ_eff ≤ 1.6 % from motor power (upper bound) | ARENA | No wheel or rail wear, active per-pod load levelling, faster moves | The energy saving is small at 0.04 m/s; floor flatness over 150 m against a 15–25 mm gap |
| Sapporo Dome hovering stage | 8,300 t | Air lift of 7.5 cm (about 90 % of the weight) plus 34 wheels, 4 m/min | ARENA | No compressors or air noise; tray-scale load levelling | Air pads remain the cheaper contactless option |
| Mercedes-Benz Stadium roof | 8 roof panels, each on two rails (one drives, one stabilises) | Rail bogies and electric motors; delayed by roof complexity (the source says only that it was delayed three times due to the complexity of the eight-panel retractable roof) | ARENA | Per-pod force equalisation is aimed at the general problem of many bogies under one stiff structure | Nothing here says what caused that delay; fabrail makes no claim about it |
| Proton-therapy vault doors | about 5 t | Motorised sliding doors on rollers, 10–12 s; manual opening on power loss | SHIELD | Passive mover (magnets and steel only) with every electronic part outside the beam line; no roller wear | Radiation ageing of magnets and sensors, and SmCo activation to Co-60 |
| ESS target shielding doors | 19 t each | Wheels on rails with a manual chain drive | SHIELD | Contactless travel of activated shielding | Access and maintenance procedures |
| Cheyenne Mountain blast doors | 25 t | Hydraulic swing doors that two people can close by hand | BASTION | A sliding plug that closes 3 m in 5.25 s, landing to seal | Doors close on warning, not on the blast; blast valves handle air paths |
| Maeslantkering (Rotterdam) | 2 arms of about 6,800 t | Floating sector gates moved on rail locomobiles; closure decided by an automatic decision system (BOS); first automatic closure reported on 21 Dec 2023 | TIDEGATE | Little for the gates. The decision chain is the analogue of fabrail's fusion and closure planner | Buoyancy and hydraulics beat an attractive-EMS gap in water |
| MOSE (Venice), Thames Barrier | 78 flap gates; 4 × 3,700 t sector gates | Compressed-air buoyancy; hydraulic rocking beams | TIDEGATE | Nothing for the gates themselves | — |
| Japanese coastal gates after 2011 | varies | Remote or automatic closure on JMA shindo 5-upper or J-ALERT over redundant links | TIDEGATE | The fused trigger stack (S-net, DAS, local P) is the transferable part | The gates' own drives |
| CERN CMS detector elements | up to 2,000 t (YB0) | Air pads and hydraulic jacks | HEAVY, ARENA | Precise, contamination-free placement | Moves are rare; air pads suffice |
| Air Danshin houses | a house | Compressed air lifts the house about 3 cm within 0.5–1 s of detection | HAVEN | The direct analogue of lift-on-alert, which fabrail dropped: W ≈ 0 near the source | — |
| Arx Pax "hover foundation" | a building | Eddy-current lift triggered by an alert (concept) | HAVEN-F | Nothing: never validated, and a cautionary tale for whole-building levitation, which fabrail does not propose | — |
| Japanese 3-D isolation research (fast-reactor building) | a reactor building | Rolling-seal air springs with ±0.5 m vertical stroke plus laminated rubber bearings | HAVEN | Nothing vertically: a 10–25 mm EMS gap cannot give ±0.5 m | Air springs carry the vertical load (§X9) |
| Getty artifact isolators | statues and cases | Passive rollers and springs; up to about 90 % acceleration reduction reported | HAVEN-R | No activation threshold, adaptive period and damping | The passive device already does most of the job |
| Transrapid | 50–70 t sections | Pure EMS, 1.7 kW/t for levitation and guidance, long-stator LSM | §X4 reference | Hybrid PM-biased EMS removes the steady hover power | — |
| EMALS | 45 t launches | 4 disk alternators of 121 MJ each; 614 mean cycles between operational mission failures against a 4,166 requirement (FY2022) | §X5 storage | The disk is the storage benchmark | Budget does not buy reliability growth at scale |

References: Taipei 101 [R144]; Shanghai Tower [R147]; Citicorp Center [R145]; SWFC [R148]; Kyobashi Seiwa [R83]; ELT dome [R137]; State Farm Stadium [R138]; Sapporo Dome [R139]; Mercedes-Benz Stadium [R162]; proton-therapy doors [R156]; ESS doors [R155]; Cheyenne Mountain [R159]; Maeslantkering [R140]; MOSE [R141]; Thames Barrier [R142]; Japanese coastal gates [R143]; CERN CMS [R154]; Air Danshin [R90]; Arx Pax [R91]; 3-D isolation [R153]; Getty isolators [R92]; Transrapid [R94]; EMALS [R166].

**What the table says.** Only one class, the tower damper, changes the physics of its host, because only there is the mass ratio designed in (about 0.5–2 % of modal mass). Everywhere else speed is rarely the bottleneck: barrier gates close in tens of minutes on forecasts hours ahead, stadium trays move at 0.04–0.07 m/s, and blast doors must already be shut before any blast. Maglev's honest value at this scale is no contact (no wear, lubrication or rail noise), no breakaway, active per-module load levelling, millimetre positioning, a software-set spring and damper, and a clean landing on loss of control.

### Section §X2: Magnetic pressure ceilings and the single admission function

**Pressure.** A pole face pulling on iron across a gap carries at most the magnetic pressure of the flux density in the gap (PHYSICS §2):

```
P = B²/(2·μ0)
```

| B | 0.6 T | 1.0 T | 1.2 T | 1.5 T | 1.6 T | 2.0 T | 2.15 T | 2.3 T |
|---|---|---|---|---|---|---|---|---|
| P | 143 kPa | 398 kPa | 573 kPa | 895 kPa | 1.02 MPa | 1.59 MPa | 1.84 MPa | 2.10 MPa |

**Three flux limits, and the smallest governs.**

- **Pole.** Electrical-steel poles saturate at B_sat = 1.5 T (PHYSICS §2). FeCo (Hiperco-class) poles saturate at 2.3 T, the highest commercial soft-magnetic material. FeCo is a pole-material parameter of hybrid EMS, not a separate suspension kind.
- **Rail.** The rail saturates too. A structural-steel rail at 1.6 T caps the pressure at 1.02 MPa whatever the pole is made of; a FeCo-lined rail at 2.15 T caps it at 1.84 MPa (XCRIT §1). FeCo poles help only if the rail is upgraded with them.
- **Gap.** A permanent-magnet-biased circuit across a 10–15 mm gap reaches B_gap,max ≈ 1.0–1.4 T, because the magnet's remanence and the leakage around a large gap limit the gap flux [UNCERTAIN: no magnetic-circuit calculation in this repository supports more]. The default is 1.2 T.

**The single admission function** (FAB-EXO-103; `admission_area_m2`). Every pole area in this tier comes from one function:

```
A = k·m·g / (η_fill · min(B_pole, B_rail, B_gap,max)² / (2·μ0))
```

- k = 1.5 is the force headroom over the static weight (ET: a 1.5× margin on the static lift).
- η_fill = 0.3–0.6 is the fraction of the suspended footprint that is active pole face; the default is 0.45 [UNCERTAIN].
- g is standard gravity, 9.80665 m/s², as everywhere in `fabrail_params::derive`.

With the defaults the 1.2 T gap ceiling governs every EMS class, so FeCo poles buy no area until the rail and the gap flux rise as well. For SENTINEL, raising all three limits (FeCo-lined rail at 2.15 T, B_gap,max 1.4 T, η_fill 0.6) takes the pole area from 57 m² to 31 m². The earlier design tables that quoted 25–40 m² for SENTINEL or 221 m² for ARENA had left out k or η_fill; with both, ARENA needs about 491 m² (XCRIT §1).

**Generated admission table** (`admission_table(0.45, 1.6, 1.2)`: η_fill 0.45, structural-steel rail at 1.6 T, B_gap,max 1.2 T, k 1.5; HTS rows at the ATZ pressure of 53 kPa at 10 mm under Earth gravity, without k; SELENE's lunar numbers are in FRONTIER §F7):

<!-- EXO_ADMISSION:BEGIN (generated from fabrail-exo-sim suspension::admission_table; do not edit) -->
| Class | Suspension | Pole | Design mass | Governing B | Effective pressure | Area |
|---|---|---|---|---|---|---|
| SENTINEL | hybrid EMS | FeCo | 1,000 t | 1.20 T | 258 kPa | 57.1 m² |
| HAVEN-R | hybrid EMS | Fe | 50 t | 1.20 T | 258 kPa | 2.85 m² |
| HAVEN-F | hybrid EMS | FeCo | 5,000 t | 1.20 T | 258 kPa | 285 m² |
| SHIELD | hybrid EMS | Fe | 50 t | 1.20 T | 258 kPa | 2.85 m² |
| BASTION | hybrid EMS | Fe | 50 t | 1.20 T | 258 kPa | 2.85 m² |
| ARENA | hybrid EMS | FeCo | 8,600 t | 1.20 T | 258 kPa | 491 m² |
| ROTATOR | hybrid EMS | Fe | 6,100 t | 1.20 T | 258 kPa | 348 m² |
| TIDEGATE | hybrid EMS | Fe | 150 t | 1.20 T | 258 kPa | 8.56 m² |
| SELENE | HTS bulk | — | 100 t | — | 53.0 kPa | 18.5 m² |
<!-- EXO_ADMISSION:END -->

**Admission is necessary, not sufficient.** A class that passes it must also pass the stray-field check (≤ 0.5 mT at accessible surfaces, or a shroud flag; FAB-EXO-104; PHYSICS §23), the curved-guideway sagitta and tilt check (§X9; FAB-EXO-111) and the energy admission (§X5). An iron-core LSM adds a normal force of 3–10× its thrust to the suspension load; SENTINEL uses an ironless LSM partly for this reason (§X7).

**Beyond the ceiling.** An ideal room-temperature superconductor over a Halbach guideway of period 154 mm reaches only 0.51 MPa at 10 mm, and a trapped-field bulk pulling on steel stays at or below 2.1 MPa whatever field it traps. Both are in FRONTIER §F2 and §F3; neither exists in Rust (FAB-EXO-210).

Sources: ET §2.4 (pressure table, FeCo, admission with k); XCRIT §1 and P1-9 (rail saturation, η_fill, B_gap,max, one admission function); PHYSICS §2 (B_sat of the core element); [R49], [R169], [R170].

### Section §X3: HTS reduced-order model

Bulk YBCO field-cooled over a permanent-magnet guideway pins flux and levitates without power. It is passively stable in heave, cross-track, roll, pitch and yaw, and neutral along the uniform travel direction of the guideway. This escapes Earnshaw's theorem because a pinned type-II superconductor is hysteretic, not a static-permeability medium. fabrail models it as `HtsBulk` (research tier, model 🔶).

**Frozen image** (after the frozen-image model of flux pinning; ET §1.3). With guideway wavenumber k = 2π/λ, gap z and the frozen reference (z_ref, y_ref) set at field cooling:

```
F_z = s_T·s_c·F0·[e^(−2kz) − e^(−k(z + z_ref))]
k_z = s_T·s_c·F0·k·[2·e^(−2kz) − e^(−k(z + z_ref))]
F_y = −k_y·(y − y_ref),   k_y = s_T·s_c·F0·k·e^(−k(z + z_ref))
```

- **Hysteresis.** A rate-independent play operator of half-width δ moves the frozen references by η_h times its output: when abs(z − z_ref) exceeds δ, z_ref moves toward z by η_h·(abs(z − z_ref) − δ), and the same for y. At η_h = 0 the loop is conservative (abs(∮F dz) < 1e-9 J); at η_h > 0 it dissipates, with the return branch below the approach branch (FAB-EXO-203).
- **Creep** (quasi-static only): s_c(t) = 1 − S·log10(1 + t/τ0), with S = 0.02–0.05 per decade and τ0 = 1–10 s [UNCERTAIN]. The model uses S = 0.03 and τ0 = 5 s.
- **Temperature**: s_T = 2j/(1 + j) with j = [(1 − (T/Tc)²)/(1 − (77/Tc)²)]², capped at 2, and 0 at or above Tc = 92 K. This is a heuristic: the lift saturates well below the Jc gain because the guideway field bounds it [UNCERTAIN].

**Fit to the ATZ demonstrator** (Floegel-Delor et al., IEEE TAS 29(5) 3601705, 2019). Four cryostats (each 24 bulks, 492 cm²) field-cooled at 35 mm carried 300 kgf at 20 mm and 550 kgf at 15 mm (FAB-EXO-202, ±1 %).

- With η_h = 0 the two points give F0 = 32.9 kN and k = 40.85 m⁻¹, so λ = 154 mm, matching the paper's 150 mm Halbach entry.
- The default demonstrator keeps the play operator active (δ = 2 mm, η_h = 0.2) and fits the first-loading branch, on which z_ref = z_fc + η_h·(min(z_fc, z + δ) − z_fc). That gives F0 ≈ 37.2 kN and k ≈ 39.9 m⁻¹ (λ ≈ 158 mm) through the same two points.

**Out-of-sample checks** (FAB-EXO-113). The paper's measured pressure at 10 mm is 5.3 N/cm² = 53 kPa. The fitted model gives 948 kgf at 10 mm, 47.3 kPa over 4 × 492 cm², which is 11 % low and inside the ±15 % band. The authors' "more than 1,000 kg expected at 10 mm" was an expectation, not a measurement, so it is not an anchor. The second datum is the larger cryostat: 5,000 N over 912 cm² (54.8 kPa) at 5–8 mm on a mismatched rail, checked within ±30 %; the model gives 58 kPa at 8 mm and 79 kPa at 5 mm.

**Stiffness and frequencies** at 10 mm for the fitted set (948 kg): k_z ≈ 974 N/mm (f_z ≈ 5.1 Hz) and k_y ≈ 214 N/mm (f_y ≈ 2.4 Hz). Both sit inside the 1–10 Hz seismic band and HTS damping is low and hysteretic, so an eddy damper or an EMS trim loop is needed. A quasi-static extra 1 g of vertical acceleration would push the set to touchdown (3 mm at 0.3 g, 5 mm at 0.5 g), so landing skids are mandatory.

**HTS pins the lateral axis.** Because k_y is a real spring, HTS cannot carry a platform that must move freely in the plane. It is usable only in 1-D stages whose free axis is the travel axis of the guideway: cross-stacked stages for SENTINEL and HAVEN, never a planar HTS bearing.

**LN₂ reservoir and fade** (FAB-EXO-204, FAB-EXO-205).

| Quantity | Model | Value (ATZ cryostat) |
|---|---|---|
| Usable latent heat | f_use·V·ρ·h_fg = 0.9 × 2.5 L × 807 kg/m³ × 199 kJ/kg | 361 kJ |
| Static hold | 361 kJ / 2.5 W | 40.1 h (paper: about 40 h) |
| Dynamic hold | heat load fitted to "more than 30 h under dynamic conditions" | 3.35 W → 30 h |
| Landing trigger | reservoir fraction | land at 10 %; 4 % critical |
| Post-depletion fade | bulk warm-up at the static load, C_bulk = 24 × 0.155 kg × 132 J/(kg·K) = 491 J/K, to s_T = e⁻¹ at 85.2 K | about 27 min |

The fade uses the bulk heat capacity only and is therefore short [UNCERTAIN]: the reservoir is the real buffer, and after depletion the lift fades in tens of minutes, not hours. On supply loss the model reports `FailureBehaviour::SlowFade` with these figures.

**Area and logistics** (generated; `hts_area_table()`: 53 kPa at 10 mm; ATZ cryostat-250 units at 2.6 kN and 2.25 W each; boil-off at 199 kJ/kg; EMS comparison at 0.3 MPa):

<!-- EXO_HTS_AREA:BEGIN (generated from fabrail-exo-sim suspension::hts_area_table; do not edit) -->
| Class | Mass | Bulk area at 10 mm | Cryostats | Heat at 77 K | LN₂ boil-off | EMS area at 0.3 MPa |
|---|---|---|---|---|---|---|
| SENTINEL | 1,000 t | 185 m² | 3,772 | 8.49 kW | 3.68 t/day | 32.7 m² |
| HAVEN-R | 50 t | 9.25 m² | 189 | 425 W | 185 kg/day | 1.63 m² |
| HAVEN-F | 5,000 t | 925 m² | 18,859 | 42.4 kW | 18.4 t/day | 163 m² |
<!-- EXO_HTS_AREA:END -->

HTS needs 5–8× the area of EMS. A cryocooler instead of a bath costs Q̇/COP of wall-plug power with COP = 0.15 × 77/(300 − 77) ≈ 0.052 (15 % of Carnot [UNCERTAIN]), about 19 W per watt at 77 K, which is about 200 W/t for the ATZ set and inside the design's 50–250 W/t.

**HTS plus EMS trim** (`HtsEmsTrim`, FAB-EXO-208, research). An EMS trim coil adds F = k_i·i to the HTS lift; an outer loop drives the creep drift of z_ref to zero and adds active damping. The requirement is a quasi-static creep test (drift ≤ 0.5 mm over 24 h), not 86 million simulated steps.

**Honest limits.** All parameters are fitted to one four-cryostat demonstrator; extrapolation to hundreds of square metres is unvalidated. Cryogenic logistics are real: the LN₂ supply, oxygen-depletion hazard, vacuum maintenance and the field-cooling procedure interlock (jacks at the field-cooling height, cool, then lower) all apply. A high-fidelity H-formulation FEM (Quéval et al. 2018) is documented, not implemented.

Sources: ATZ 2019 (data, hold times, 50 cm module limit); ET §1 (model forms, Jc(T) from Materials 2019); XCRIT P0-7 (the corrected anchors, the area table, 1-D stages only); [R171], [R172], [R173], [R174], [R175], [R178].

### Section §X4: EMS at scale

**Pure EMS** (`PureEms`, reference only). F = K·(i/g)² with the open-loop pole p = √(2g/g0) at a fixed current (PHYSICS §3). Hover power is the Transrapid figure of 1.7 kW/t for levitation and guidance, so 10 kt draws 17 MW and an 8,600 t tray about 15 MW. On supply loss the lift decays with the coil time constant onto skids (`Immediate`).

**Hybrid PM-biased EMS** (`HybridEmsAdapter`). The adapter delegates the magnetics to the core co-energy element (PHYSICS §2–§3) and never reimplements it; its hysteresis state is the eddy lag on the force flux. The permanent magnets carry the weight at the zero-power gap g*, and the coil carries only the dynamic force m·a. For F ∝ i² the small-signal current is δi = i_W·a/(2g), so the copper loss is

```
P_hover = P_W · (a_rms / 2g)²          P_W = 1.7 kW/t × m
```

| Vertical acceleration RMS at the suspension | 0.1 g | 0.2 g | 0.5 g |
|---|---|---|---|
| Hover loss at 10 kt | 42 kW | 170 kW | 1,060 kW |

These reproduce FAB-EXO-206. The acceleration is the RMS at the suspension, not the ground PGA; with series air springs the 0.2 g case is conservative.

**Mass does not change the pole.** The open-loop pole √(2g/g_eff) does not depend on mass, so a 10 kt platform needs the same control bandwidth as a door. Only the inductance and the inverter volt-amperes scale.

**Supply loss is indeterminate.** At any gap below g* the PM force exceeds the weight, and above it the force falls short. A zero-power hybrid EMS that loses its supply therefore diverges up or down depending on where it is at that instant: `FailureBehaviour::Diverge { bias: Indeterminate }`. No "no clamp-up" property is claimed. Instead both stops are designed sliding surfaces (the upper one at a contact force ≤ 1.3 W, FAB-LEV-006; the lower one an FPS catcher or rollers), and landing is an energised safety function with a monitored reserve and a mechanical backstop (FAB-EXO-115, §S3). A PM share below one would make the upper stop clamp-free at the cost of continuous coil power (about 14–126 W/t for an assumed circuit [UNCERTAIN]); fabrail does not use it.

**Ride-through energy** (FAB-EXO-120, FAB-EXO-403; `t_ride_s`, `ride_through_energy_j`).

```
T_ride = max(300 s, t_shaking of the class scenario) + t_land
E_ride = P_hold · T_ride
```

The 300 s floor exists because the long-duration M9 scenario shakes for 150–300 s (XS03); the earlier 120 s window undersized storage (XCRIT P1-12).

| Case (10 kt, 300 s) | Hold power | Energy | With the 2× margin | Verdict |
|---|---|---|---|---|
| Hybrid EMS at 0.2 g RMS | 170 kW | 51 MJ | 102 MJ | Within one EMALS-class disk's demonstrated 121 MJ per cycle |
| Pure EMS | 17 MW | 5.1 GJ | 10.2 GJ | Flagged insufficient against one JET-class flywheel (3.75 GJ stored, about 2.8 GJ usable) |
| HTS | sensors and propulsion only | kW-scale × T_ride | — | The LN₂ reservoir, not the store, is the buffer (FAB-EXO-405) |

Sources: ET §4 and §2.1 (Transrapid, hover-loss law); XCRIT P0-2, P0-3, P1-11, P1-12 (fail direction, energised landing, EMALS, T_ride); PHYSICS §2, §3 (the core element and its pole); [R94], [R166], [R170].

### Section §X5: Energy storage

`EnergyStore` is defined once, in `fabrail_exo::energy`, and is f64: stored energies span 10³ J to 3.75e9 J and admission runs at planning rate. The truth models in `fabrail-exo-sim::storage` implement it, and so do the f32 estimators in the controller (which keep a compensated accumulator so a 1 J step on a 3.75 GJ store is not lost).

| Model | Energy | Usable | Loss | Numbers |
|---|---|---|---|---|
| Flywheel | ½·I·ω² | ½·I·(ω² − ω_min²); fraction 1 − (ω_min/ω_max)² | a + b·ω² | 75 % usable at ω_min = ω_max/2 (FAB-EXO-401) |
| Supercap | ½·C·V² | ½·C·(V² − V_min²) | I²·ESR, terminal power V·I − I²·ESR | 3–5 Wh/kg, 5–15 kW/kg at module level [UNCERTAIN] |
| Battery | SOC × capacity | (SOC − SOC_min) × capacity | one-way efficiency √η | UPS class, C-rate limited |
| Hybrid store | supercap + flywheel | sum | sum | The flywheel supplies p_fly = (E_cap,target − E_cap)/τ with τ = 1–5 s, so it carries the low-pass of the demand and the supercap the rest |
| SMES | ½·L·I², P ≤ V_conv·I | — | cryogenic overhead; a quench dumps the energy | Documented only; about 30 MJ is the largest built [UNCERTAIN]; research at kWh scale |

**Benchmarks.** An EMALS disk alternator delivers 121 MJ per launch by slowing from 6400 to 5205 rpm, which is 1 − (5205/6400)² = 33.9 % of its energy, so it stores about 357 MJ (XCRIT P1-11 corrected the earlier "90.75 MJ usable"). The JET flywheels store 3.75 GJ at up to 400 MW.

**Admission** (FAB-EXO-402, FAB-EXO-122; `admit`). A move starts only if all of these hold:

1. the zone is `ClearVerified` (§X14);
2. **discharge:** E_usable ≥ 2·(E_stop + E_land + P_hold·T_ride);
3. **absorption:** charge headroom + dump or eddy-brake rating ≥ 1.2·KE_max, because a store that is full cannot take the braking energy;
4. **bumper:** rating ≥ ½·m_rated·v_max², always with the rated mass, never the estimate m̂.

A class that fails is reported, not hidden. ROTATOR fails the absorption check today: braking 6,100 t from a 1.5 m/s rim speed releases up to 6.86 MJ (an upper bound, with I ≤ m·r²), against a 2 kWh (7.2 MJ) regeneration buffer, which is 1.05×, below the 1.2× rule.

**Regeneration and the ledger** (FAB-EXO-404; `split_regen`, `EnergyLedger`). Braking power P_regen = η·abs(F_brake·v) goes to storage up to the charge rating and the headroom; the rest goes to the dump resistor or the eddy brake. The balance of delivered, stored, dissipated and dumped energy closes within 1 %. The accumulators are f64 on the host and i64 mJ on a target; f32 accumulators would drift by about 1.8 % over 3e5 steps, more than the gate.

**What budget does not buy.** EMALS reached 614 mean cycles between operational mission failures against a 4,166-cycle requirement (FY2022). Power-electronics reliability at sovereign scale needs reliability growth, not money alone.

Sources: ET §4 (flywheel, supercap, SMES, EMALS, JET); XCRIT P1-11 and P1-13 (EMALS stored energy, absorption split, ROTATOR); exotic design §2.5 and §3.3 (models, regeneration); [R166], [R167], [R168].

### Section §X6: Host structures

**2-DOF structure plus damper** (`TwoDof`; PHYSICS §21 for the closed forms, §24 for RK4). Host mode M, ω_s, ζ_s; damper m_d, k_d, c_d; LSM trim force u; ground acceleration a_g; wind force f_ext:

```
M·ẍ_s + 2ζ_s·M·ω_s·ẋ_s + M·ω_s²·x_s − c_d(ẋ_d − ẋ_s) − k_d(x_d − x_s) = −M·a_g − u + f_ext
m_d·ẍ_d + c_d(ẋ_d − ẋ_s) + k_d(x_d − x_s) = −m_d·a_g + u
```

Beyond the stroke limit s_max an elastic end-stop buffer acts between damper and structure (stiffness 50·k_d, contact damping ζ 0.1; a design choice), so the truth model never exceeds the stroke silently.

**Modal tower** (`ModalTower`). Up to 150 lumped storeys with Rayleigh damping are reduced to 3–5 modes (φ, ω, ζ, Γ) once, at construction, by a cyclic Jacobi eigensolver on M^(−½)·K·M^(−½); nalgebra is allowed only in `fabrail-plant`, so the crate carries its own solver. Cracking and occupancy drift ω_s by up to ±10 % (`detuned`).

**Tuning** (`fabrail_exo::tuning`, delegating to `fabrail_params::derive`; FAB-EXO-601). μ = m_d/M, f = ω_d/ω_s:

| Form | Excitation | f | ζ as published | Normalised to | μ 0.01 (f / ζ) | μ 0.02 (f / ζ) |
|---|---|---|---|---|---|---|
| Den Hartog | harmonic force, undamped host | 1/(1 + μ) | √(3μ/(8(1 + μ)³)) | ω_s | 0.99010 / 0.06033 | 0.98039 / 0.08407 |
| Warburton, base | white-noise ground acceleration | √(1 − μ/2)/(1 + μ) | √(μ(1 − μ/4)/(4(1 + μ)(1 − μ/2))) | ω_d | 0.98762 / 0.04981 | 0.97548 / 0.07019 |
| Warburton, force | white-noise force on the host (wind) | √(1 + μ/2)/(1 + μ) | √(μ(1 + 3μ/4)/(4(1 + μ)(1 + μ/2))) | ω_d | 0.99257 / 0.04981 | 0.98528 / 0.07019 |

- **ζ normalisation.** The published ζ values do not share one basis. Den Hartog's is normalised to ω_s. Warburton's are exact H2 optima only as ζ_d = c_d/(2·m_d·ω_d). Every dimensional output re-expresses the damper ratio on the ω_s basis, ζ′ = f·ζ_d, so c_d = 2·ζ′·m_d·ω_s (ζ′ is 1.2 % below ζ_d at μ 0.01 and 2.5 % below at μ 0.02). Unprimed ζ_s is always the host's own damping.
- **Peak.** With ζ_s = 0 the Den Hartog fixed points give a peak amplification of √(1 + 2/μ): 14.177 at μ 0.01 and 10.050 at μ 0.02 (FAB-EXO-602, harmonic sweep ±2 %).
- **Wind uses the force form.** Wind acts on the building, not through the ground, so SENTINEL is tuned with the force form.

**The stroke bound** (FAB-EXO-603). The damper's reaction on the host is at most F = m_d·ω²·S_abs, where S_abs is its absolute stroke. The equivalent added damping of a host moving at amplitude X is

```
ζ_add ≈ F / (2·M·ω²·X) = (μ/2)·(S/X)
```

| Case | μ | S/X | ζ_add |
|---|---|---|---|
| Wind, well tuned, low amplitude | 1 % | 7 | 3.5 % |
| Design-level earthquake: roof drift near the ±1.5 m stroke | 1 % | 1.4 | 0.7 % |

No stroke-limited control law beats this, active or passive. Adding 5 % damping to a tower of 100 kt modal mass at T = 7 s with a roof velocity of 0.9 m/s would take about 8 MN, which is about 10 m of stroke on a 1,000 t damper. Mass dampers are mainly wind devices, and their seismic benefit at design level is at most about 0.7 % added damping.

**Wind response is reported against ζ_s** (FAB-EXO-607). For the passive optimum under white-noise force, the host displacement RMS falls by 44.7 % at μ 1 % and ζ_s 1 %, but by 30.0 % at ζ_s 2 %; at μ 0.5 % the figures are 37 % and 23 %, and at μ 0.3 % 32 % and 18 %. The reduction is always printed with ζ_s, and the ≥ 40 % target applies to the active mode only.

**Near-fault pulses.** A damper cannot build up before the first half-cycle of a pulse, so the requirement is only that the controlled host peak stay within 1.02× the uncontrolled peak (FAB-EXO-606).

Sources: SS §2.1 and §2.4 (2-DOF model, stroke bound, sizing); XCRIT P1-16, P1-17 (Warburton force form, ζ normalisation, ζ_s-dependent wind reduction); Connor, *Structural Motion Control* ch. 4; [R80], [R81], [R152].

### Section §X7: Active and hybrid mass damping

SENTINEL is a 1,000 t hybrid mass damper (μ ≈ 1 % of modal mass) at T = 7 s with ±1.5 m of stroke. Its kinematic budget follows from the period and stroke (FAB-EXO-110): ω = 2π/7 = 0.898 rad/s, so v = ω·S = 1.346 m/s, a = ω²·S = 1.209 m/s² and a jerk of ω³·S = 1.085 m/s³.

**Restoring force stays passive.** The damper rides a curved guideway (a concave-up bowl built as two cross-stacked 1-D stages) of radius R = g/ω² = 12.2 m. At full stroke the guideway supplies 1.18 MN of restoring force for free; an LSM supplying it would dissipate about (F/K_m)² ≈ 1.4 MW of copper loss at a motor constant of about 1,000 N/√W [UNCERTAIN]. The LSM supplies damping and trim only.

**Force and power.** The trim force is 2–10 % of the inertial force m·a = 1.21 MN, i.e. F_max = 24–121 kN, with a power of F·v = 33–163 kW (XCRIT P1-8 corrected the earlier 36–180 kN). The LSM's virtual stiffness can retune the damper by Δk/k = F_max/(m_d·ω²·s): 10 % (about ±5 % in frequency) at 1.5 m and 75 % at 0.2 m. "Software-set tuning" is true at wind amplitudes only.

**Why the LSM is ironless.** An iron-core LSM's cogging at 1–3 % of 121 kN is 0.12–0.37 mg on 1,000 t, which alone breaks the ≤ 0.3 mg activation target of §X8, and its normal force of 3–10× the thrust loads the suspension. An ironless LSM has neither.

**Controller** (`fabrail_exo::hmd`, f32).

- **State and gains.** z = [x_s, ẋ_s, x_r, ẋ_r] with stroke x_r = x_d − x_s, and u_lqr = −K·ẑ. The gains come from an offline discrete Riccati solution (scipy) with explicit weight on the stroke; the scalar anchor is K = (a + √(a² + b²·q/r))/b. The observer is fed the applied (saturated) force for anti-windup. Robustness is checked by ±10 % detuning (FAB-EXO-608); H∞ synthesis and MPC are roadmap.
- **Saturation stack**, in priority order:
  1. **Stroke governor.** Weight w = ξ³ with ξ = (abs(x_r) − 0.7·s_max)/(0.3·s_max) clamped to [0, 1]. Inside the zone the controller may not feed power into the stroke (a command with u·ẋ_r > 0 is scaled by 1 − w) and the governor adds u_sg = −w·(c_sg·ẋ_r + k_sg·x_r) with k_sg = F_max/s_max and c_sg = F_max²/P_max. This copies the stroke-dependent damping of the Shanghai Tower damper.
  2. **Power:** abs(u·ẋ_r) ≤ P_max.
  3. **Force:** abs(u) ≤ F_max.
  4. **Rate:** at most F_max per 0.5 s, for bumpless mode transfer.
- **DVF fallback.** u = −c_dvf·ẋ_r with c_dvf = c_d,opt − c_eddy (ζ normalised to ω_s). It needs only the LSM encoder and is collocated and passive.

The governor bounds what the controller does near the ends; it cannot arrest a damper driven past the design basis. The trim LSM is about 10 % of the guideway restoring force at full stroke, so an over-design excursion ends on the progressive eddy zone (the last 0.3–0.5 m) and the end buffers. The kinetic energy at 1.346 m/s is 0.91 MJ; stopping it in 0.3 m averages about 3 MN, which becomes a floor design load.

**Mode ladder** (`next_mode`; FAB-EXO-605). The most degraded condition wins. Degradation is immediate; recovery needs 60 s of continuous better demand; LANDED_FPS is latched until maintenance.

| Mode | Entered on | Force source | Precedent |
|---|---|---|---|
| HMD_ACTIVE | normal (wind) | LQR through the saturation stack | — |
| SEISMIC_PASSIVE | an alert, or floor acceleration above threshold | Passive tuning plus eddy damping; stroke governor armed | SWFC active dampers revert to passive in earthquakes |
| DVF_ONLY | a sensor fault | Collocated velocity feedback from the LSM encoder | — |
| PASSIVE_EDDY | loss of power | PM-over-copper eddy damping only | Shanghai Tower |
| LANDED_FPS | loss of levitation | The damper rests on an FPS catcher of the same radius and becomes a friction-pendulum TMD | — |
| LOCKED | maintenance | none | — |

**Fail behaviour.** On supply loss the zero-power suspension diverges in an indeterminate direction (§X4) onto designed sliding stops: the upper stop at ≤ 1.3 W (FAB-LEV-006) or the FPS catcher below. There is no clamp-up claim; the earlier design's "bias below the weight at the minimum gap" is impossible for a zero-power hybrid and was deleted (XCRIT P0-2).

**Seismic role.** SENTINEL rides through levitated (`RideThrough`): landing a 1,000 t damper mid-event would remove the damping it adds. The ride-through runs on islanded storage for T_ride (§X4); the seismic benefit is still bounded by §X6.

Sources: SS §2 (sizing, restoring, failure table, SWFC); exotic design §3.1 (controller, stack, ladder); XCRIT P0-2, P1-8, P1-17 (fail direction, F_max, ironless LSM); [R82], [R147], [R148], [R149].

### Section §X8: Activation threshold and residual drag

**Coulomb activation** (`CoulombSdof`; FAB-EXO-609). A damper on a Coulomb-type bearing stays stuck until the floor acceleration exceeds a_act = μ·g; the model reproduces no slip at 0.9·μg and slip at 1.1·μg. The industry rule is that the bearing must break away at no more than one tenth of the acceleration the damper is meant to control. Wind-comfort targets are about 3–7 mg for one-month winds, so the bearing must activate at ≤ 0.3–0.7 mg; SENTINEL uses ≤ 0.3 mg (FAB-EXO-607).

| Bearing | μ or μ_eq | a_act | Against 0.3 mg |
|---|---|---|---|
| PTFE slider | 0.05 | 50 mg | fails |
| Rolling guide | 0.003 | 3 mg | fails |
| Hybrid EMS on a solid-steel rail (hysteresis drag) | about 1e-3 | 1 mg | fails |
| Hybrid EMS on a laminated rail | about 1e-3 | 1 mg | fails |
| Hybrid EMS on a low-coercivity rail, or a homopolar layout | about 1e-4 | 0.1 mg | passes |
| HTS over a PM guideway | ≤ 1e-5 | ≤ 0.01 mg | passes |
| Sensor floor (ADXL355, 22.5 µg/√Hz over 0.1 Hz) | — | about 7 µg rms | — |

The μ_eq magnitudes are [UNCERTAIN] and are swept (`RailCondition`).

**Laminations cut eddy drag, not hysteresis drag.** A magnet moving along an iron rail loses energy two ways (PHYSICS §9):

- **Eddy drag** rises with speed. It is viscous, adds to the damper's own damping and never blocks motion. Laminations or soft-magnetic composite cut it.
- **Hysteresis drag** is the loop area of the rail material times the volume swept per unit travel. It does not depend on speed, so it acts like Coulomb friction and sets an activation threshold. Laminations do not change it. Only a low-coercivity rail material (a smaller loop) or a homopolar pole layout (no flux reversal along the travel) reduces it.

So "maglev removes the friction threshold" is true only with the right rail. The core's solid-steel and laminated rails (μ_eq ≈ 1e-3–1e-2, PHYSICS §9) do not pass the one-tenth rule at wind-comfort levels; SENTINEL's rail must be low-coercivity or homopolar.

**Always afloat matters as much as low drag.** Citicorp's oil-film bearings arm at 3 mg for two cycles, and their pumps take about 3 min to lift the mass about 2 cm. That cannot respond inside an S−P window. A zero-power maglev bearing is always levitated and responds in milliseconds.

Sources: SS §2.2 (activation table, Citicorp, 1/10 rule after Motioneering); exotic design §6.1 (activation anchors); CRIT G7; PHYSICS §9 (the core drag ledger); [R145], [R151].

### Section §X9: Isolation platform

HAVEN-R carries a 50 t payload (racks, artefacts, instruments, a small occupied room) on a levitated platform whose horizontal restoring force comes from a curved guideway. HAVEN-F scales the same idea to a 5 kt facility floor and is data plus algebraic admission only.

**Curved guideway** (FAB-EXO-701). A body on a concave guideway of radius R has k = W/R and T = 2π·√(R/g), the friction-pendulum law without the friction. HAVEN-R uses T = 6 s, so R = 8.95 m (T = 4 s would need R = 3.98 m).

**Why 6 s, not 4 s** (XCRIT P0-1). The ASCE 7-16 Ch. 17 constant-velocity branch gives the design displacement S_D = S_D1·g·T/(4π²·B), with B = 1.35 at 15 %, 1.5 at 20 % and 1.7 at 30 % damping. The platform's peak acceleration is then

```
a_peak = ω²·S_D·√(1 + 4ζ²) = (S_D1·g / (B·T))·√(1 + 4ζ²)
```

| Site (ζ 0.2, B 1.5) | T = 4 s | T = 6 s |
|---|---|---|
| DBE, S_D1 = 0.6 | S_D 0.40 m; a_peak 0.108 g (over 0.1 g) | S_D 0.60 m; a_peak 0.072 g |
| MCE, S_M1 = 0.9 | S_D 0.60 m; 0.162 g | S_D 0.89 m; 0.108 g |

So the default is T = 6 s with a stroke equal to S_D at MCE, ±0.9 m. The 0.1 g occupant target is an analysis output at DBE for the stated site; MCE and near-fault values are reported, not bounded (FAB-EXO-505). At full stroke the guideway spring alone gives 0.1 g.

**Transmissibility** (FAB-EXO-702; closed forms in `fabrail_params::derive`, PHYSICS §21). With r = f/f_iso:

```
passive:  T = √[(1 + (2ζr)²) / ((1 − r²)² + (2ζr)²)]
skyhook:  T = 1 / √[(1 − r²)² + (2ζr)²]
```

| ζ 0.2 | 0.5 Hz | 1 Hz | 2 Hz |
|---|---|---|---|
| T_iso 4 s, passive / skyhook | 0.4125 / 0.3221 | 0.1251 / 0.0663 | 0.0531 / 0.0159 |
| T_iso 6 s, passive / skyhook | 0.1931 / 0.1236 | 0.0741 / 0.0285 | 0.0343 / 0.0070 |

**Geometry admission** (FAB-EXO-111). A straight pole of length L on a guideway of radius R stands off by the sagitta L²/(8R), which must stay ≤ 10 % of the gap: 7.9 mm for L = 0.5 m at R = 3.98 m (too much for a 10–15 mm gap), 3.5 mm at R = 8.95 m. At full stroke a pod tilts by asin(S/R): 5.8° at ±0.9 m on 8.95 m, up to 10.9° at T = 4 s and 7.1° for SENTINEL. Pole faces are therefore curved and pods articulated for tilts up to 11°. Attractive EMS cannot give 2-D travel on one plane, so HAVEN uses cross-stacked 1-D stages (X then Y) like SENTINEL, which roughly doubles the suspended stage area [UNCERTAIN: layout].

**Copper damper** (FAB-EXO-118, FAB-EXO-706). PM-over-copper eddy damping gives c/A = σ·B²·t at low speed (an upper bound): 1.48e5 N·s/m per m² at B = 0.5 T and 10 mm of copper. The conductivity falls with temperature, σ(T) = σ0/(1 + 0.00393·ΔT), with ΔT = E/(m_Cu·c_p), and the damping coefficient falls with it. Example: HAVEN-F at ζ 0.2 needs about 21 m² (1.9 t) of copper; 56.5 MJ over 200 s heats it by 77 K, which lowers σ and c by 23 %.

**Controller** (`fabrail_exo::isolation`, f32).

```
F_lsm = −c_sky·v̂_abs − w_rc·k_rc·x_rel + F_pre      then abs(F) ≤ F_max, then the semi-active clip
```

- **Skyhook.** c_sky = 2ζ·m·ω_iso (Karnopp; PHYSICS §16.2). v̂_abs is the leaky integral of the platform's absolute acceleration with a 0.1 Hz high-pass, so accelerometer bias cannot integrate into a drift.
- **Recentring.** k_rc = m·(2π·f_rc)² with f_rc ≤ 0.1·f_iso, so the added stiffness is ≤ 1 % of the isolation stiffness. w_rc = 0 immediately on SHAKING and ramps back over 1/f_rc afterwards (FAB-EXO-707).
- **Pre-positioning.** A critically damped move toward the centre at 0.2 Hz with its force capped at m·0.02 g, only with independent confirmation, a `ClearVerified` moat and a state before SHAKING. Recentring is suspended while it runs.
- **Semi-active.** At low storage state of charge the command is clipped to u·v_rel ≤ 0: the LSM only brakes and regenerates, drawing no net energy.

**Vertical isolation stays passive** (FAB-EXO-705). At 1 Hz vertical isolation needs a static deflection g/ω² = 0.25 m, far beyond a 10–20 mm EMS gap. An active actuator carrying M·g through a relative motion v_rel circulates P = M·g·v_rel: 147 kW at 50 t, 29 MW at 10 kt and 294 MW at 100 kt for v_rel = 0.3 m/s. PM repulsion is too stiff as well (ω = √(g/λ) ≈ 3.5 Hz for a 20 mm decay length). So the vertical load goes to passive air springs or quasi-zero-stiffness elements, maglev is trim only, and a platform without passive vertical support is rejected above 1 t.

**Control loss** (FAB-EXO-704). The platform lands on an FPS catcher of the same radius, so the period stays continuous and the facility becomes a conventional FPS-isolated structure. A PM-bias ramp keeps the landing at ≤ 0.1 m/s; an uncontrolled drop of 10–15 mm would hit at 0.44–0.54 m/s.

**Moat.** The moat edge is a shear hazard, and presence sensing cannot stop a platform that the earthquake is driving. The moat therefore has passive covers plus exclusion; PL e presence sensing gates controlled moves only (recentring, pre-positioning, landing). Bumper energies are quoted in MJ: HAVEN-F 5 kt at 1.2 m/s stores 3.6 MJ, and a 2 m/s fling about 10 MJ (FAB-EXO-504, FAB-EXO-703).

**Always levitated.** HAVEN-F never lifts on an alert. Lift-on-alert (the Air Danshin idea) adds a lift-failure mode and gains nothing near the source, where the warning is about zero; the 15 MJ lift anchor was dropped.

**Honest comparison with LRB and FPS.** Maglev isolation has no activation threshold, so it also isolates micro-vibration and small earthquakes, and its period and damping can adapt. Against that: it needs 10–500× the bearing area, it depends entirely on power, a 10–20 mm gap makes settlement and fault offset critical, and it has no code path (no ASCE 7 Ch. 17 prototype tests). Research tier at 1–10 kt; 100 kt is an extrapolation with no new physics.

Sources: SS §3 (demand, forces, copper, vertical power, comparison); XCRIT P0-1, P1-10, P1-18, P1-20, P1-21 (T 6 s, geometry, σ(T), moat, lift-on-alert); ASCE 7-16 Ch. 17 (S_D form, B factors); [R16], [R42], [R90], [R153].

### Section §X10: EEW timing and fusion

fabrail is not an earthquake early-warning provider. Live feeds need ShakeAlert LtO or JMA licensing and are roadmap; every alert in this tier is synthetic or injected, and every timing output prints its inputs.

**Timing** (`eew_timing`; PHYSICS §20 for the core relations).

```
t_alert = R_k/V_p + τ_win + t_pkt + t_tel + t_proc + t_diss        (source algorithms; R_k: distance to the k-th station)
W       = R_site/V_s − t_alert − t_act                              (warning before the S wave)
∂W/∂x   = 1/V_p = 0.15385 s/km   (V_p 6.5 km/s)
        = 1/V_s = 0.27027 s/km   (PLUM-type intensity algorithms, V_s 3.7 km/s)
r_b     = √((V_s·t_alert)² − h²)                                    (blind-zone epicentral radius)
```

The default latency budget is τ_win 1 s, t_pkt 1 s, t_tel 0.5 s, t_proc 0.5 s and t_diss 0.3 s, 3.3 s in total [UNCERTAIN: typical-order values]. The core's own on-site relations are S−P = 0.119 s/km, a blind zone of about 20 km and a network/on-site crossover at 40.8 km (PHYSICS §20).

**Moving sensors toward the fault** (FAB-EXO-803, FAB-EXO-804). Each kilometre of sensor advance buys 0.154 s of warning for P-based algorithms and 0.270 s for intensity-based ones. A worked case (inputs: epicentre 100 km offshore at 20 km depth, city 20 km inland so the S wave arrives at 32.9 s, fourth-nearest station, 3.3 s latency):

| Sensing | Alert time | Warning at the city |
|---|---|---|
| Coastal land network (4th station about 110 km away) | 20.5 s | 12.4 s |
| S-net-like seafloor stations (about 34 km) | 9.4 s | 23.5 s |
| DAS fibre over the source (about 8 km) | 6.6 s | 26.3 s |

Densifying a network below about 10 km spacing gains little, because the fixed latency terms take over. Near the source the warning is about zero whatever the budget.

**Sources of evidence** (`AlertFeedKind`; FAB-EXO-801). `Network`, `OffshoreCabled` (S-net-like) and `Das` arrive as feeds with magnitude, distance, depth, origin time and an optional site intensity. The on-site P pick (`LocalP`) arrives only through the local detector. `PegsMagnitudeAdvisory` is research tier: an unsaturated magnitude for an event another feed already reports, never a trigger on its own (FAB-EXO-810). The `Injector` exists only under `cfg(feature = "fault-injection")`, which no firmware feature set enables (FAB-EXO-307); injected feeds never count as independent confirmation.

**Fusion** (`fabrail_exo::fusion`; FAB-EXO-802).

1. **Primary estimate (JMA-style maximum).** Each source predicts the site intensity I_k: the provider's MMI when given, else an intensity prediction equation (Atkinson and Wald 2007) at the hypocentral distance; the on-site source maps Pd to PGV to MMI. The source with the maximum I drives the event. Its S-wave arrival is t_S ~ N(t0 + R_hyp/V_s, σ) with σ = 1–2 s, and P(I ≥ I_crit) = Φ((I_max − I_crit)/σ_I) is the prior. JMA takes the maximum of its source-based (IPF) and intensity-based (PLUM) predictions in the same way.
2. **Secondary: capped log-odds** over the local detectors, logit P = logit P0 + clamp(Σ w_i·ln LR_i, ±cap). The terms are the on-site P pick confirming a remote alert, the absence of that pick 2σ after the predicted P arrival (the signature of a concurrent-quake false alarm, as on 5 Jan 2018 in Japan), a τc level ≥ 2 and PGA ≥ 80 gal. The cap stops correlated errors from compounding.
3. **Decision: cost–loss.** Act when P > C/L. Two actions with different costs give two thresholds: freeze (TRIGGERED, level 1) and protective action (PROTECT, level 2; level 3 when I_max ≥ I_crit + 1).

**States.** NORMAL → TRIGGERED or PROTECT → SHAKING → QUIET. Within an event the state and level never step down. The event returns to NORMAL when its remote evidence is withdrawn and the site saw nothing: a cancellation, a downgrade, or a no-show (no local pick or shaking by the S-ETA plus a margin; FAB-EXO-805). Local physics outranks a remote retraction: once the site picked P or shook, only the INSPECT reset ends the event.

**Output.** A `FusedAlert` is a synthetic core `NetworkAlert` plus the exotic extension (level, p_damaging, S-ETA mean and σ, long-period flag, tsunami ETA, feed set, number of independent feeds). The exotic classes reach the core only through the `ActionPolicy` hook, bounded by the alert gate (§S2) and the core `admit` filter. Machine-to-machine budget: alert to controller under 0.5 s, controller to actuator under 10 ms.

Sources: SS §1 (timing model, offshore table, fusion design, JMA precedents); Kodera 2018 (PLUM); Lior 2023 and dEPIC (DAS); XCRIT P1-23 (PEGS advisory only, Injector gating); [R67], [R70], [R182], [R183], [R186], [R187], [R195].

### Section §X11: Air blast

An air blast is the second "pressure wave" of the sovereign tier. fabrail treats it with generic closed forms whose inputs are the peak overpressure P_s, the positive-phase duration t_d and the arrival time t_a. It has no scaled-distance law and publishes no yield-to-distance table in code, outputs or docs (FAB-EXO-808). fabrail is not a protective-design or weapons-effects tool; structural response is governed by UFC 3-340-02 and is outside this repository.

**Friedlander waveform and impulse** (FAB-EXO-806; `fabrail.exotic`).

```
P(t) = P_s·(1 − t/t_d)·e^(−b·t/t_d)
i    = P_s·t_d·[1/b − (1 − e^(−b))/b²]
```

| b | 0.5 | 1 | 2 | 5 |
|---|---|---|---|---|
| i/(P_s·t_d) | 0.4261 | 0.36788 (e⁻¹) | 0.2838 | 0.1603 |

**Normal reflection** (Rankine–Hugoniot, ideal gas, γ = 1.4, P0 = 101.325 kPa):

```
P_r = 2·P_s·(7·P0 + 4·P_s) / (7·P0 + P_s)
```

P_r/P_s is 2.008 at 1 kPa, 2.741 at 100 kPa and 7.60 at 10 MPa, and tends to 8 for a strong shock.

**Arrival bound.** The shock travels at U_s = c0·√(1 + (γ + 1)/(2γ)·P_s/P0) ≥ c0 (1.04·c0 at 10 kPa, 1.36·c0 at 100 kPa, 2.29·c0 at 500 kPa), so a blast at distance R arrives no later than t_a ≤ R/c0. That bound is reported next to every closure decision; t_a itself is always an input.

**The closure race** (FAB-EXO-807; `fabrail_exo::closure`). The planner answers only "can it finish in time":

```
CLOSE  iff  t_min(d, v0, a, v_max, j) + t_seal + margin ≤ t_a − t_detect,  and the zone is clear (or a two-person override is active)
```

- t_min is the jerk-limited minimum close time (PHYSICS §16; the rest-to-rest case equals the core S-curve). A move that cannot stop within d from its current speed has no solution, because the only escape would be an opening, which an alert never commands.
- BASTION closes 3 m in 5.246 s at a = 0.48 m/s² and j = 2 m/s³ (peak 1.144 m/s), or 5.031 s at a = 0.53 m/s². With 0.6 s of seal and margin, a close needs t_a ≥ 5.85 s, so R ≥ c0·5.85 s ≈ 2.0 km.
- Otherwise the door takes its configured fallback: land in place, or hold open landed.

**What this means.** Blast lead time is usable only for km-scale events. Nearby blasts arrive in 0.1–0.5 s (a few tens to about 170 m by the weak-shock bound), and only normally closed doors and passive blast valves act in that regime. The value of a 5.25 s close is operational: less exposure while the door must be open, not a reaction to a nearby blast. The tunnel-piston calculator and the scaled-distance relations of the earlier design were dropped (XCRIT P1-22).

Sources: SS §4.1 (Friedlander, reflection, door race); XCRIT §1, P1-14, P1-22 (arrival bound, jerk-aware t_min, dropped laws); [R191], [R193].

### Section §X12: Tsunami timing

A tsunami is the third pressure wave of the tier, and fabrail models its timing only: no inundation, run-up or hydrodynamic load (`fabrail_exo_sim::tsunami`; FAB-EXO-809).

```
c(h)      = √(g·h)                           198 m/s at 4,000 m, 99 m/s at 1,000 m, 22 m/s at 50 m
t_arrival = ∫ dx / √(g·h(x))                 (trapezoid over a (distance, depth) profile)
A1/A0     = (h0/h1)^(1/4)                    Green's law: ×4.47 from 4,000 m to 10 m
budget    = t_arrival − t_shaking − t_latency
```

For example, 100 km of path over a uniform 1,000 m shelf takes about 17 min. M9 shaking lasts 1–3 min, and cabled seafloor networks detect a tsunami tens of minutes earlier than coastal gauges (reported: about 20 min for S-net), so a gate that closes in 30–120 s has minutes to spare. Time is not the problem for TIDEGATE.

**The rail after the earthquake is.** Coastal subsidence of 0.3–1.2 m, tilt, liquefaction and debris can distort the rail. TIDEGATE therefore runs a gap check after the shaking and closes by maglev only if it passes; otherwise it closes on its wheels or rack. It closes only after the shaking ends, and only on the command of an external decision system (the decision path has a SIL 3 target; SIL 4 is unclaimable). Debris and salt in an attractive-EMS gap are an unsolved objection, and buoyant or hydraulic gates beat maglev physically. The transferable product is the fused trigger stack, not the gate.

Sources: SS §4.2 (timing, Green's law, subsidence, gate fallback); SA §2G (verdict against buoyant and hydraulic gates); [R140], [R141], [R142], [R143], [R190].

### Section §X13: Redundancy, black channel and PFH

These are architecture concepts from IEC 61508 used as design targets. Nothing here is a compliance claim, and nothing establishes a PFH for hardware.

**Where the vote sits.** The 2oo3 vote is on the shutdown path (LSM STO and the brake), not on the 10 kHz force loop. The landing command is voted separately (§S3). The force loop runs one active channel with a hot standby and split magnet groups, which is firmware roadmap.

**Mid-value select** (`vote`; FAB-EXO-301). mvs(a, b, c) = max(min(a, b), min(max(a, b), c)). The output always lies between two of the inputs, so one faulty channel, whatever its value, cannot move the output outside the range of the two healthy ones; with the healthy spread within e_th the output moves by at most e_th.

- **Exclusion.** A channel whose residual abs(x_i − x_mvs) exceeds e_th on every sample for t_persist is excluded and latched until reset; a residual that returns within e_th restarts the timer.
- **Two channels (1oo2D).** The output is the midpoint while the pair agrees within e_th, and holds the last agreed value while it disagrees. A disagreement that persists for t_persist is broken by the diverse channel.
- **Diverse channel.** L(g) = N²·μ0·A/(2g) + L_leak, estimated from the PWM current ripple (PHYSICS §2, §13), is about 5–10× noisier than the eddy-current probe [UNCERTAIN], so its tolerance e_div is several times e_th. It is a tie-breaker only (FAB-EXO-304, research): it never becomes the output, never excludes a channel while the primaries agree, and otherwise can only pick between primaries or push toward the safe state.
- All arithmetic is integer, with residuals in i64, so a saturated input cannot overflow.

**Degradation ladder** (`ladder`; FAB-EXO-302, FAB-EXO-303). The ladder only steps down until a maintenance reset.

| Rung | Entered when | Leaves for |
|---|---|---|
| 2oo3 | power-up or reset with three healthy channels | 1oo2D on one loss; safe state on a simultaneous double loss |
| 1oo2D | one channel lost | 1oo1D on a second loss once 1oo2D is established and L(g) is healthy; otherwise safe state |
| 1oo1D | a diagnosed second loss | safe state on a third loss or when L(g) fails |
| Safe state | anything else | nothing (latched) |

A degraded rung is established only after it has been held for t_persist. A further loss before that, or two losses on one cycle, is treated as a common-cause failure and goes straight to the safe state, whatever the diverse channel says. The safe state is STO on the LSM, the brake, and the energised landing with its monitored reserve.

**Black channel** (`black_channel`; FAB-EXO-306; the IEC 61784-3 concept only). The CH-A ↔ CH-B cross-link carries 51-byte frames: source ID (u16), sequence (u32), sender tick (u64), length (u8), a 32-byte payload and a CRC-32 (ISO-HDLC). The receiver rejects corruption (CRC; non-zero unused payload bytes; length > 32), masquerade (source ID), repetition, reordering and replay (sequence strictly increasing), stale or future timestamps (freshness window) and loss (timeout). For a codeword this short the CRC detects every error of up to 3 bits and every burst of up to 32 bits. A CRC is not a MAC: cryptographic authentication belongs in the gateway, not the safety layer. A wrapped sequence counts as a replay, so the pair is re-keyed before 2³² frames.

**PFH for 2oo3** (`pfh`; FAB-EXO-305, FAB-EXO-117). With λ_DD = DC·λ_D, λ_DU = (1 − DC)·λ_D and the channel-equivalent down time

```
t_CE = (λ_DU/λ_D)·(T1/2 + MRT) + (λ_DD/λ_D)·MTTR

conservative:  PFH = 6·[(1 − β_D)·λ_DD + (1 − β)·λ_DU]²·t_CE + β_D·λ_DD + β·λ_DU
Ed. 2:         PFH = 6·[(1 − β_D)·λ_DD + (1 − β)·λ_DU]·(1 − β)·λ_DU·t_CE + β·λ_DU
```

The conservative form keeps detected failures dangerous until repaired; the IEC 61508-6 Ed. 2 high-demand form requires the second failure to be undetected, because a detected failure drives the safe state [UNCERTAIN: the Ed. 2 form is recalled, not re-read].

| Reference case | t_CE | Conservative | Ed. 2 |
|---|---|---|---|
| λ_D 1e-6/h, DC 99 %, β 2 %, β_D 1 %, T1 8,760 h, MTTR = MRT = 8 h | 51.8 h | 1.04e-8 /h | 2.03e-10 /h |

The two forms are about 50× apart, so the edition must always be stated. Common cause dominates: the conservative form reaches the SIL 4 band (1e-9 to 1e-8 per hour) only with β_D ≤ about 0.5 %, and a β_D below the IEC 61508-6 Annex D floor is not credible [UNCERTAIN]. A continuous-time Markov chain in the style of Annex B.5.2 (all good; one failed detected with repair at 1/MTTR; one failed undetected until the proof test; found at the proof test with repair at 1/MRT; tripped, Ed. 2 only; failed dangerous, absorbing), integrated with RK4 over two proof-test intervals, agrees with both closed forms within ±20 %. It shares their rate model, so it checks the algebra and the t_CE approximation, not the rates. The rates are placeholders; there is no FMEDA.

Sources: ET §5 (targets, worked example, SIL bands); exotic design §2.4; XCRIT P1-15 (two forms, CTMC); IEC 61508-2, -6 and IEC 61784-3 (concepts, numeric forms); [R21], [R22].

### Section §X14: Guarded-zone kinematics

**The zone machine** (`zone`; FAB-EXO-501, FAB-EXO-502). States Unknown → Arming → ClearVerified → Intrusion → Stopping → Stopped.

- **Clear** means both diverse presence channels clear (a Type 4 light curtain or Type 3 scanner, plus radar or mats) and the access interlocks locked (guard locking or a trapped key). Either presence channel alone trips.
- **ClearVerified** needs clear plus a rising edge of the manual reset from outside the zone while arming. A reset held down or stuck never arms, and a press made before the zone became clear does not count. Motion is permitted only in ClearVerified.
- **Intrusion.** Any trip in ClearVerified withdraws motion on that cycle and demands SS1. A moving mover then runs its SS1 ramp and must stop within the analytic distance d_stop below, after which the LSM goes to STO and the class lands (Stopped). Motion reported outside ClearVerified is treated the same way.
- **Two-person override** (BASTION, TIDEGATE; FAB-EXO-507). Presence sensing wins over closure unless the class permits the override and it is active. The override replaces the presence channels only; the interlocks and the outside reset are still required, and releasing it with presence not clear trips at once.

```
d_stop = v·t_react + v²/(2·a_brake) + v·a/(2j)
```

**Separation distance** (ISO 13855 numeric limits; `iso13855_separation_mm`). S = K·(t1 + t2) + C with the approach speed K = 1.6 m/s, t1 = 0.1 s for the sensing and logic, the intrusion allowance C = 0.85 m, and t2 = v/a_brake for the stop:

| Class | v_max | a_brake | t2 | S |
|---|---|---|---|---|
| SHIELD | 0.5 m/s | 0.5 m/s² | 1.0 s | 2.61 m |
| BASTION | 1.2 m/s | 0.48 m/s² | 2.5 s | 5.01 m |
| ARENA | 0.3 m/s | 0.05 m/s² | 6.0 s | 10.61 m |

**Attended jog** (`envelope`; FAB-EXO-503). Hold-to-run with a three-position enabling device at v ≤ min(10 mm/s, √(2·E_ped/m_rated)), with the pedestrian energy cap E_ped imported from `fabrail_params::presets`. The full 10 mm/s is allowed up to 33.8 t; SHIELD at 50 t jogs at 8.2 mm/s. Momentum stays uncapped and is stated: √(2·E_ped·m) = 411 kg·m/s for SHIELD. The drive force is capped at the imported sensed static limit (150 N) and the closing edge is compliant (≤ 10 N/mm), so the peak contact force F_cap + √(F_cap² + 2·k·KE) = 387 N stays within the imported dynamic limit of 400 N. (The core found that 1.69 J alone gives about 503 N on a hand without the compliant edge.)

**Bumpers and stored energy** (FAB-EXO-504). Rating ≥ ½·m_rated·v_max², always with the rated mass: SHIELD 6.25 kJ at 50 t and 0.5 m/s; BASTION 32.7 kJ at 50 t and its 1.144 m/s peak (72 kJ for a 100 t door at 1.2 m/s); ARENA 387 kJ at 0.3 m/s (momentum 2.58e6 kg·m/s); SENTINEL 0.91 MJ of overstroke; ROTATOR up to 6.86 MJ; HAVEN-F 3.6 MJ at 1.2 m/s.

Sources: exotic design §1.3 and §2.6 (zone machine, ISO 13855 inputs, jog, bumpers); XCRIT P1-19 (imported caps, force cap and compliant edge, stated momentum); ISO 13855, ISO 13849-1, ISO 14119 (numeric limits only; see "Standards used by this tier"); [R23], [R24], [R25].

### Section §X15: Ground-motion additions

These generators complement the core synthesis of PHYSICS §19 (`fabrail_exo_sim::ground`). White noise comes from the in-repo Xoshiro256++ (PHYSICS §24), so every series is a pure function of its seed on every host.

| Generator | Form | Parameters | Used by |
|---|---|---|---|
| Near-fault pulse | Mavroeidis–Papageorgiou velocity pulse v(t) = (A/2)·[1 + cos(2π·f_p·τ/γ)]·cos(2π·f_p·τ + ν) for abs(τ) ≤ γ/(2f_p) | Pulse period from ln T_p = −6.45 + 1.11·M_w (Baker 2007; σ_lnTp 0.56) [UNCERTAIN: coefficients as widely cited]: 3.4 s at M6.9, 7.3 s at M7.6; γ = 2, ν = 0 | XS01 (Kobe-type) |
| Fling step | One full cycle of sine acceleration: velocity returns to zero and the displacement ends at d | d of metres (Chi-Chi-like, 5–9 m reported [UNCERTAIN]) | XS02 |
| Basin resonance | White noise through a Kanai–Tajimi filter and a Clough–Penzien high-pass, 5 % cosine tapers, scaled to the RMS | ω_g = 2π/6, ζ_g 0.05–0.15; ω_f = 0.1·ω_g, ζ_f 0.6; longer than 10 min | XS04 |
| Long-duration M9 | Clough–Penzien filtered noise under a double Jennings–Housner–Tsai envelope (second packet at 0.3·D with 0.9 of the amplitude), scaled to the PGA | ω_g 10 rad/s, ζ_g 0.4; D 150–300 s [UNCERTAIN: envelope shape] | XS03, XS06 |
| Vertical pulse | The near-fault pulse scaled to a peak of up to 1 g | — | XS05 |
| Davenport wind | Gust spectrum S_u(f) = 4·κ·U10²·x²/(f·(1 + x²)^(4/3)), x = 1200·f/U10, synthesised as a Shinozuka sum; quasi-steady force ρ·C_d·A·U10·u′(t) with aerodynamic admittance 1 (conservative for large faces) [UNCERTAIN] | 128 log-spaced bins over 1e-3–2 Hz; U10, κ, A and C_d as inputs | XS12 |

Every scenario states its magnitude, distance, site factor and percentile, as the core does.

Sources: SS §5 (scenario synthesis); Mavroeidis and Papageorgiou 2003; Baker 2007; Davenport 1961; PHYSICS §19 (Kanai–Tajimi and Clough–Penzien in the core); [R73], [R74], [R75], [R76], [R93], [R189].

### Section §X16: Integer units and range

The exotic envelope (`units`, `envelope`) is the arithmetically diverse channel to the f32 controllers, and it must hold from a 1 kg test mass to a 1e8 kg platform (FAB-EXO-112). The core monitor's units (mass in grams as u32, at most 4,295 t) and the earlier Q16.16 format (at most 32,767) both overflow at sovereign masses, so the tier uses its own units and no Q16.16.

| Quantity | Type | Unit | Exact range |
|---|---|---|---|
| Mass | `MassKg` (u32) | kg | 0 to 4.29e9 kg (tested 1 kg to 1e8 kg) |
| Velocity | `VelMmps` (i32) | mm/s | ±2.1e9 mm/s |
| 2 × kinetic energy | `Ke2Uj` (u64) | µJ (m·v² in kg·mm²/s²) | exact for m ≤ 1e8 kg and abs(v) ≤ 400 m/s (1.6e19 < 1.8e19) |
| Momentum | `MomKgMmps` (u64) | kg·mm/s | total (u32 × 2³¹ < 2⁶⁴) |
| Force | `ForceN` (i64) | N, formed in mN | total for m·a |

- Products are formed in u128 or i128 and narrowed with checked conversions, so nothing wraps: checked forms return `None`, saturating forms clamp, and an overflow in the envelope trips (fails safe).
- The f64 conversions exist for configuration in `new()` only and round to nearest. No float arithmetic runs per cycle.
- The envelope equals an f64 reference within 1 LSB (FAB-EXO-903).
- Energy-balance accumulators are f64 on the host and i64 mJ on a target (±9.2e15 J).

Sources: XCRIT P0-5 (overflow analysis, new units, accumulators); PHYSICS §18 (the core integer monitor).

### Section §X17: Validation table and tolerance list

Tolerance classes are those of SPECIFICATION §0.8: (E) exact algebra, rel 1e-12; (I) integration error; (S) standard clause limit; (R) research metric; (MC) Monte Carlo Wilson bound; (P) provisional. Every assert message states the value, the tolerance and the source.

| Anchor (§2) | Tolerance | Requirement | Section |
|---|---|---|---|
| Den Hartog f, ζ, A_max at μ 0.01 / 0.02 | (E); harmonic sweep ±2 % | FAB-EXO-601, FAB-EXO-602 | §X6 |
| Warburton, base and force forms | (E) | FAB-EXO-601 | §X6 |
| Wind RMS reduction against ζ_s | reported (R) | FAB-EXO-607 | §X6 |
| Stroke bound ζ_add ≤ (μ/2)(S/X) | +0.002 | FAB-EXO-603 | §X6 |
| HTS fit, 300 kgf at 20 mm and 550 kgf at 15 mm | ±1 % | FAB-EXO-202 | §X3 |
| HTS out of sample, 53 kPa at 10 mm; 5,000 N over 912 cm² at 5–8 mm | ±15 %; ±30 % | FAB-EXO-113 | §X3 |
| HTS area table (generated) | ±1 % | FAB-EXO-121 | §X3 |
| LN₂ hold, 40 h static and 30 h dynamic | ±10 % | FAB-EXO-204 | §X3 |
| Hover loss, 1.7 kW/t; 42 / 170 / 1,060 kW | (E) | FAB-EXO-206 | §X4 |
| FeCo pressure, 2.10 MPa at 2.3 T | (E) | FAB-EXO-207 | §X2 |
| Isolation transmissibility at T = 4 s, ζ 0.2 | ±2 % | FAB-EXO-702 | §X9 |
| Curved guideway, T 6 s ↔ R 8.95 m | ±0.5 % | FAB-EXO-701 | §X9 |
| Sagitta, 7.9 mm at L 0.5 m and R 3.98 m | (E) | FAB-EXO-111 | §X9 |
| Ride-through, 51 MJ hybrid and 5.1 GJ pure EMS at 10 kt over 300 s | exact | FAB-EXO-403 | §X4 |
| EMALS-class disk, 121 MJ delivered and about 357 MJ stored | exact | FAB-EXO-403 | §X5 |
| Flywheel usable fraction, 75 % at ω_max/2 | exact | FAB-EXO-401 | §X5 |
| Lead-time gain, 0.15385 and 0.27027 s/km | 1e-9 | FAB-EXO-803 | §X10 |
| Offshore M8 warning | ±1 s | FAB-EXO-804 | §X10 |
| Friedlander impulse and reflection factor | 1e-9 | FAB-EXO-806 | §X11 |
| Blast feasibility, R ≥ 2 km for a BASTION close | exact | FAB-EXO-807 | §X11 |
| Tsunami, 198 m/s at 4 km; Green's law ×4.47 | 1e-9 | FAB-EXO-809 | §X12 |
| PFH, 1.04e-8 and 2.0e-10 per hour; CTMC | ±2 %; ±20 % | FAB-EXO-305, FAB-EXO-117 | §X13 |
| ISO 13855 separation, 2.61 / 5.01 / 10.61 m | exact | FAB-EXO-502 | §X14 |
| SENTINEL kinematics, 1.346 m/s, 1.209 m/s², 1.085 m/s³ | (E) | FAB-EXO-110 | §X7 |
| Integer range, 1 kg to 1e8 kg | no overflow (proptest) | FAB-EXO-112 | §X16 |

**Scenario catalogue** (`fabrail_exo_sim::scenarios`, prefixed XS and XC; each runs a reduced model of its class, labelled as such, and prints its inputs):

| Scenario | Input | Pass criterion | Requirement |
|---|---|---|---|
| XS01 | Kobe-type near-fault pulse | HAVEN stroke within the moat with no buffer contact; SENTINEL host peak ≤ 1.02× uncontrolled | FAB-EXO-606, FAB-EXO-703 |
| XS02 | Chi-Chi-like fling of metres | Exceedance expected; peak deceleration within design, bumper energy quoted in MJ; support never lost; stable landing | FAB-EXO-703 |
| XS03 | M9, 150–300 s | Energy balance within 1 %; copper ΔT within its limit; SOC > 0; no FPS fallback | FAB-EXO-404, FAB-EXO-706 |
| XS04 | Basin resonance, longer than 10 min | The stroke governor engages before the end stops; ζ_add within ±20 % of the bound | FAB-EXO-603, FAB-EXO-604 |
| XS05 | Vertical pulse up to 1 g | EMS saturation detected; the passive vertical support carries the load; supply-loss direction indeterminate onto designed stops | FAB-EXO-705, FAB-EXO-115 |
| XS06 | Grid loss at S arrival | Ride-through for T_ride on islanded storage; energised, controlled descent | FAB-EXO-120 |
| XS07 | Controller or LSM fault | Degrades to a friction-pendulum TMD or FPS isolation; impact within the buffer rating | FAB-EXO-605, FAB-EXO-704 |
| XS08 | Cancelled alert; concurrent-quake false alarm | Returns to NORMAL; cost within the cost–loss bound | FAB-EXO-805 |
| XS09 | Offshore M8: land, S-net-like, DAS | Warning within ±1 s of the analytic value | FAB-EXO-804 |
| XS10 | Generic blast with P_s, t_d and t_a as inputs | Closed before t_a, or marked infeasible | FAB-EXO-807 |
| XS11 | Tsunami with 0.3–1.2 m subsidence | Closes within the budget, or falls back to wheels | FAB-EXO-809 |
| XS12 | Wind at 3–7 mg | Activation ≤ 0.3 mg; RMS reduction reported against ζ_s | FAB-EXO-607 |
| XC-RED-1 | Single sensor faults of every kind | No loss of levitation | FAB-EXO-301 |
| XC-RED-2 | β common-cause double fault | Safe state within the ladder bound | FAB-EXO-303 |
| XC-SAF-1 | Intrusion mid-move (ARENA, BASTION) | Stop within the analytic d_stop | FAB-EXO-502 |
| XC-ARN-1 | One of 500 ARENA pods lost | Static reallocation meets the wrench, else LAND (dynamic gap error is roadmap) | — |
| XC-SHD-1 | P wave during SHIELD transit | Land and lock within 2 s; beam permit dropped (reduced model) | — |
| XC-HTS-1 | LN₂ depletion | Lands at 10 % LN₂; no free fall | FAB-EXO-205 |
| XC-ROT-1 | Wind uplift with 10 mm ring ovalisation | Hooks engage; no contact at speed (data and admission only) | — |

---

## Sovereign safety (Section §S)

This part is the safety argument of the tier. It is design evidence for architectures **designed toward** SIL 3 and SIL 4 targets; nothing in it is certified, and simulation does not establish a SIL, a PL or a PFH for hardware.

### Section §S1: Safety regimes and safe-state policies

**Regimes** (`SafetyRegime`; FAB-EXO-006, FAB-EXO-102).

- `Pedestrian`: the core caps (energy, momentum, directional force clamps; PHYSICS §16, §18). Only the core classes use it.
- `GuardedZone(ZoneCfg)`: people are kept out of reach while the mover moves (PL e presence sensing where fitted, guard locking, ISO 13855 separation; §X14).
- `OccupiedPlatform(OccupantCfg)`: people ride on it, within comfort limits (§X9).

`ClassRegime::new` is the only way to pair a class with a regime. It refuses `GuardedZone` on DRAWER, DOOR or WALL and `Pedestrian` on any exotic class. The core `RailClass` and its caps are never extended or changed.

**Per-class policy** (`presets`, `SafeStatePolicy::seismic_action`).

| Class | Regime | Safe-state policy | Core SeismicAction | Seismic role | Idle rest policy |
|---|---|---|---|---|---|
| SENTINEL | GuardedZone: locked plant room, unoccupied in operation | RideThrough | RideThrough | Damper ride-through (SEISMIC_PASSIVE) | Hover |
| HAVEN-R | OccupiedPlatform | IsolateThenLandFps | Isolate | Primary isolation | Hover |
| HAVEN-F | OccupiedPlatform, always levitated | IsolateThenLandFps | Isolate | Primary isolation | Hover |
| SHIELD | GuardedZone with PL e presence and the facility PPS | LandAndLock, within 2 s | Lock | Land and secure | LandLock |
| BASTION | GuardedZone with PL e presence; two-person override | CompleteClose, else land | CompleteClose | Closure race | LandLock |
| ARENA | GuardedZone with PL e presence (tray cleared) | LandAndLock onto skids and anchors | Lock | Land and secure | LandLock |
| ROTATOR | GuardedZone (ring zone) | BrakeAndLand onto the pier | Land | Land and secure | Land |
| TIDEGATE | GuardedZone, unmanned closure; two-person override | CloseAfterShaking, on the external command | CloseAfterShaking | Post-shaking closure | Land |
| SELENE (🔭) | GuardedZone, teleoperated | LandAndLock | Lock | Land and secure | LandLock |

**Land-first is not universal.** The core rule (freeze, then land or lock) is right for pedestrian classes, whose lateral capacity of 0.1–0.3 g is below 0.3–1 g of floor shaking. Three exotic classes need something else: SENTINEL rides through levitated, because landing would remove the damping it adds; TIDEGATE closes after the shaking ends, and only on an external decision-system command; BASTION completes a close already under way when it can finish in time (§X11), and otherwise lands.

**Occupant limits** (HAVEN; `OccupantCfg`). Controlled moves (recentring, pre-positioning, landing) stay within 0.1 g, a jerk of 1 m/s³ and a landing speed of 0.1 m/s, and need the moat band clear (`check_occupied_move`). The seismic a_peak at DBE is an analysis output (FAB-EXO-505), not a trip.

### Section §S2: The alert rule and SafetyRank

**"Toward safety" is a partial order** (`SafetyRank`; FAB-EXO-506):

> Seated or Landed+Locked > Closed-levitated > Standoff/Hold > Frozen-open

A device's physical condition maps onto a rank with `SafetyRank::of`. The egress role matters for one condition: an egress opening never locks, so landed (with ASC released after the stop) is already its top rank, whereas a non-egress device that has landed but not locked sits one rank lower.

**The gate** (`alert_gate::gate`; FAB-EXO-119).

- FREEZE, LAND and SEISMIC_PASSIVE are always admissible; LOCK is admissible except on an egress path. These four are the only actions a network-only or injected alert can cause.
- Opening, unlocking, de-isolating and speeding up are never admissible on an alert. The core `OpenAndStop` class action stays admissible only while no non-local source contributed.
- Alert-triggered **motion** must go to a strictly higher rank, in the closing or seating direction, must never reverse a user-initiated motion, and needs independent confirmation: a local P pick, or at least two independent feeds. In addition, sovereign classes need the zone `ClearVerified`, and pedestrian classes need obstruction detection armed and the motion within their unattended low-energy limits.
- Pre-positioning is motion without a checkable direction, so it is refused while any user motion is in progress.

**Confirmation is derived conservatively** (`Confirmations::from_alert`). An injected contribution voids every feed count (only a genuine local P pick can still confirm); a local-P flag without the LocalP feed is inconsistent and confirms nothing; a PEGS advisory alone never counts.

**Spoof matrix** (the test enumerates every combination; this is its summary):

| Alert evidence | Freeze, land, lock (non-egress), seismic passive | Closing move to a higher rank | Pre-positioning | Open, unlock, de-isolate, speed up |
|---|---|---|---|---|
| One network feed | allowed | denied | denied | denied |
| Injected feeds, with or without others | allowed | denied unless a genuine local P pick confirms | denied unless a genuine local P pick confirms | denied |
| PEGS advisory only | no alert is raised (never a sole trigger) | denied | denied | denied |
| Local P pick | allowed | allowed with ClearVerified (sovereign) or armed obstruction and low-energy limits (pedestrian) | allowed with a ClearVerified moat, before SHAKING | denied |
| Two or more independent feeds | allowed | as for a local P pick | as for a local P pick | denied |

A spoofed alert therefore has bounded regret: at worst devices freeze, land or lock, or close at unattended low-energy limits with obstruction detection armed. The core `admit` filter (TECHNICAL_DESIGN §15.3) is the backstop for everything the exotic policy proposes.

### Section §S3: 1oo2D and 2oo3 architectures, designed toward SIL 3 and SIL 4 targets

Both architectures are **designed toward, not certified**. There is no FMEDA, no β-factor analysis, no hardware and no WCET measurement (FAB-EXO-904 is roadmap). Only the no_std crates exist, built as rlibs for thumbv7em (control) and armv7r (safety); the firmware applications are 🗺️.

**1oo2D, designed toward a SIL 3 / PL e target.**

```
Sensors A (triplex gap set) ─> CH-A  lockstep Cortex-M7 (e.g. NXP S32K3), RTIC, thumbv7em-none-eabihf
                                     fabrail-core + fabrail-exo (control) + local plausibility
                                <── black-channel cross-link (CRC-32, sequence, timeout, source ID) ──>
Sensors B (diverse: L(g), Hall) ─> CH-B  lockstep Cortex-R5F (e.g. TI AM263x), cyclic executive on aarch32-rt, armv7r-none-eabihf
                                     fabrail-safety + fabrail-exo-safety (integer envelope, voter, ladder)
Outputs: STO-A and STO-B in series on the LSM gate drivers only; brake; each channel can trip alone.
Landing: an energised safety function with a dedicated, monitored reserve and a mechanical backstop, commanded separately.
```

**2oo3, designed toward a SIL 4 target that fabrail cannot claim.** A third channel (an AURIX TC4x with HighTec's Rust or C compiler, or an S32Z Cortex-R52 on the armv8r target, which is supported but not qualified) joins a hard-wired 2oo3 relay or contactor voter on STO and the brake only. The force loop runs one active channel with a hot standby and bumpless transfer, and split magnet groups with independent controllers.

**Corrections the architecture carries** (XCRIT P0-2, P0-3).

- STO acts on the **LSM only**. Levitation's safe state is the landing (the core SS1 Safe Landing, PHYSICS §18).
- Landing a zero-power hybrid EMS is **energise-to-land**, not de-energise-to-safe: a de-energised suspension diverges in an indeterminate direction (§X4). Landing therefore has its own monitored reserve and is voted separately from STO and the brake.
- The 2oo3 voter gates LSM STO and the brake; the landing command is voted separately, with reserve-health monitoring.

**Where diversity comes from, and where it does not.**

- Different ISA and silicon vendor; different scheduler (RTIC priority tasks on CH-A, a fixed-slot cyclic executive on CH-B); **different code** for the safety function (`fabrail-exo-safety` is not `fabrail-exo`); different arithmetic (integers against f32).
- A same-code comparison (CH-B re-running the control code) catches random hardware faults only; it is stated as such.
- Compiler diversity is weak: Ferrocene and HighTec are both LLVM-based. A C channel or a different optimisation profile adds a little.

### Section §S4: Toolchain status

Status as reported in the exotic-technology research (September 2026). Items marked [UNCERTAIN] were not confirmed from a primary source.

| Item | Status | Consequence |
|---|---|---|
| Ferrocene qualified targets | `thumbv7em-none-eabihf`, `armv7r-none-eabihf`, `aarch64-unknown-none`; `armv8r-none-eabihf` and `thumbv8m.*` are "supported" only. TÜV SÜD ISO 26262 ASIL D and IEC 61508 SIL 3 qualification of the compiler (release 26.05) | A qualified compiler for both channels of the 1oo2D sketch |
| Ferrocene `core` subset | Qualified to SIL 2 / ASIL B only | Any other use of `core`, `libm` or a crate needs the user's own justification |
| RTIC | All Cortex-M devices; no Cortex-R or Cortex-A support; not a qualified component | CH-A only; its macro-expanded code is reviewed as application code |
| aarch32-cpu / aarch32-rt | Armv7-R and Armv8-R runtime, community-maintained, not qualified, no interrupt-controller scheduler | CH-B uses a hand-written cyclic executive |
| NXP S32K3 | Single, dual or lockstep Cortex-M7; ISO 26262 up to ASIL B/D; no IEC 61508 claim found [UNCERTAIN] | Candidate for CH-A |
| TI AM263x | Lockstep Cortex-R5F, little-endian; IEC 61508 SIL 3 and ASIL D component assessment (TÜV SÜD) | Candidate for CH-B |
| TI TMS570 | BE32 big-endian | Rust cannot target it; avoid |
| Infineon AURIX | HighTec Rust compiler qualified to ISO 26262 ASIL D; no IEC 61508 claim found [UNCERTAIN] | Candidate third channel |
| PACs and HALs | svd2rust-generated, not qualified | Reviewable as application code |

References: Ferrocene [R131]; aarch32 crates [R132]; RTIC [R119]; HighTec [R133]; TMS570 [R134]; AM2634 [R135]; S32K3 [R136].

What the table does not mean, stated plainly:

- **Ferrocene target qualification does not certify the application.** A qualified compiler is one input to an assurance case, not the case.
- **One lockstep chip is SIL 3 at most** (IEC 61508-2 Annex E limits on-chip redundancy), so any SIL 4 target needs at least two diverse lockstep MCUs voting 2oo3.
- **TMS570 (BE32) cannot be targeted** by Rust or LLVM, which support only BE8 big-endian Arm.
- Every architecture here is **designed toward, not certified**.

### Section §S5: Hazards and non-claims

**Hazards of the tier**, each mapped to the requirements that address it. The core hazards H-01 to H-21 are in SAFETY.md; these extend them for the exotic classes.

| Hazard | Classes | Addressed by |
|---|---|---|
| XH1 Crushing or impact by a massive mover inside its zone | GuardedZone classes | FAB-EXO-501, FAB-EXO-502, FAB-EXO-503, FAB-EXO-504, FAB-EXO-507 |
| XH2 A spoofed, injected or false alert triggers motion | all | FAB-EXO-119, FAB-EXO-506, FAB-EXO-805, FAB-EXO-810 |
| XH3 Supply loss with a levitated heavy mass (up or down) | all hybrid EMS | FAB-EXO-115, FAB-EXO-605, FAB-EXO-704 |
| XH4 Release of stored energy (overstroke, braking into a full store, bumper overload) | all | FAB-EXO-122, FAB-EXO-402, FAB-EXO-504, FAB-EXO-604 |
| XH5 Shear at the moat of an isolated platform | HAVEN-R, HAVEN-F | FAB-EXO-505, FAB-EXO-703 |
| XH6 Occupant acceleration on an isolated platform | HAVEN-R, HAVEN-F | FAB-EXO-505, FAB-EXO-707 |
| XH7 Cryogen exhaustion and oxygen depletion | HTS options | FAB-EXO-204, FAB-EXO-205 |
| XH8 Stray field at accessible surfaces | all | FAB-EXO-104 |
| XH9 Common-cause failure of redundant channels | all | FAB-EXO-303, FAB-EXO-305, FAB-EXO-117 |
| XH10 Corrupted, stale or replayed safety messages | all | FAB-EXO-306 |
| XH11 Fault injection left enabled in a build | all | FAB-EXO-307 |
| XH12 Integer overflow in the envelope at sovereign masses | all | FAB-EXO-112, FAB-EXO-903 |
| XH13 A gate closing on a distorted rail after shaking | TIDEGATE | FAB-EXO-809 |
| XH14 Radiation ageing of magnets and sensors; SmCo activation to Co-60 | SHIELD | — (unquantified; documented only) |

**Non-claims.** These go with every exotic output, page and example.

1. **No certification.** Nothing is certified at any SIL or PL. The architectures are designed toward SIL 3 or SIL 4 targets, with no FMEDA. One lockstep chip is SIL 3 at most; Ferrocene qualification of the compiler does not certify the application; RTIC, aarch32-rt, the PACs and `libm` are not qualified.
2. **Physics bounds hold under an unlimited budget:** the pressure ceiling (§X2), the stroke bound on damping (§X6), the lead-time gain (§X10), passive-only vertical isolation (§X9) and the km-scale blast lead (§X11).
3. **Mass dampers are mainly wind devices.** At design-level shaking they add at most about 0.7 % damping, and no damper can act before the first half-cycle of a near-fault pulse.
4. **Isolation platforms** have no ASCE 7 Ch. 17 prototype tests and no code path. Maglev vertical isolation is not proposed. Whole-building levitation is not proposed (Arx Pax is a cautionary tale). The moat edge is a shear hazard in an earthquake.
5. **HTS carries cryogenic logistics** (LN₂ supply, oxygen depletion, vacuum, field-cooling interlocks, creep), and its parameters are fitted to one four-cryostat demonstrator. After LN₂ depletion the lift fades in tens of minutes.
6. **Blast.** fabrail is not a protective-design or weapons-effects tool, publishes no yield-to-distance table and has no scaled-distance law. Near-field doors must be normally closed, and only blast valves act at arrival times of 0.1–0.5 s. No EMP or shock hardening is claimed.
7. **Tsunami.** Timing only, no inundation modelling. Buoyant and hydraulic gates beat maglev physically, and debris or salt in an attractive-EMS gap is unsolved.
8. **EEW.** fabrail is not an EEW provider. Live feeds need ShakeAlert LtO or JMA licensing, every simulated feed is synthetic, and the warning near the source is about zero.
9. **"Zero friction" is not zero.** On solid-steel and laminated rails hysteresis drag (μ_eq ≈ 1e-3 [UNCERTAIN]) breaks the one-tenth activation rule; the rail material and layout must be chosen for it (§X8).
10. **SHIELD.** Radiation ageing of magnets and sensors is unquantified, and SmCo activates to Co-60.
11. **Cost being no object removes no other constraint:** NdFeB supply of 150–450 t for the largest classes, certification timelines, cryogen logistics and power-electronics reliability (EMALS: 614 against a 4,166-cycle requirement) all remain.
12. **Frontier items are theoretical and never products** ([FRONTIER.md](FRONTIER.md)).
13. **Simulation is not validation.** There is no hardware and no FEM, and every exotic parameter carries a sensitivity sweep.

---

## Standards used by this tier

The same rule as SPECIFICATION §4 applies: numeric limits only, no compliance or certification claim. The only wording used is "checked in simulation against numeric limits from [edition]". Editions marked "to confirm" are fixed here before any number from them is quoted; SAFETY.md fixes the core editions.

| Document | Edition | Use |
|---|---|---|
| IEC 61508-2, -3, -6 | Ed. 2 (2010), plus the conservative PFH form | Architecture targets, the dependency rule, the PFH calculator (§X13) |
| IEC 61784-3 | to confirm | The black-channel concept only; no numbers are taken from it (§X13) |
| ISO 13855 | 2010 (the edition whose K and C are used; any later revision to confirm) | Separation distance S = K·(t1 + t2) + C with K = 1.6 m/s and C = 0.85 m (§X14) |
| ISO 14119 | to confirm | Guard locking and interlocks, qualitatively (§X14) |
| ISO 12100 | to confirm | Risk reduction and the attended jog, qualitatively; no numbers are taken from it (§X14) |
| ISO 13849-1 | to confirm | PL d and PL e as design targets only (§X14, §S3) |
| ASCE 7-16 Ch. 17 | 2016 | The isolation displacement S_D and the B factors; no prototype tests are claimed (§X9) |
| UFC 3-340-02 | 2008 | Named as the governing reference for blast structural response, which fabrail does not model (§X11) |
