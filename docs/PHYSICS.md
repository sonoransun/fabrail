# fabrail physics

This document specifies the model exactly as implemented: the f64 plant in `fabrail-plant`, the closed forms in `fabrail_params::derive`, and the control laws in `fabrail-core` and `fabrail-safety`. Code cites it as `// PHYSICS §n` (or `§n.m`). Every constant carries a source, and every closed form has a test at the tolerance class of [SPECIFICATION.md §0.8](SPECIFICATION.md).

> **Status: Wave 2.** The numbered headings are frozen because code cites them; new material goes into the body of a section, never into a renumbering. The body describes the code as of Wave 2: the Wave-1.5 closed loop (Carriage6 for DOOR and DRAWER, voltage coil mode) and the binding Wave-2 lead decisions D1–D10. D3 (physical end stops, positive-work monitor), D4 (governor margins and damping), D5 (LSM convention), D6 (unattended pinch) and D8 (full-rate executive) are in code, as is the D1 coupled roll–sway design; D2 (DRAWER 48 kg slot exit) and D7 (re-seat after idle) were still pending when this was written and are marked *pending* with the Wave-1.5 behaviour beside them, and the closed-loop outcome of D1 is a Wave-2.5 item, so that Wave 2.5 can reconcile code and text in one edit. Exotic-tier physics is in [EXOTIC.md](EXOTIC.md) (Section §X) and frontier models are in [FRONTIER.md](FRONTIER.md) (Section §F). Parameter values are generated into [PARAMETERS.md](PARAMETERS.md); the per-class numbers quoted here come from the same presets.

**How to read an entry.** Each section names the files that implement it, gives the equations in the form the code evaluates them, lists the constants that are not parameters (module constants with their source; values marked \[UNCERTAIN\] are design choices swept by studies), and names the tests that assert it. Tolerance classes: (E) exact algebra, relative 1e-12; (I) integration error; (S) standard-clause limit; (R) research metric; (MC) Wilson 95 % bound; (P) provisional.

---

## §1 Scope and conventions

The model covers one rail with one carriage and its node: the hybrid PM-biased electromagnets (EMS) that suspend the carriage under a steel rail, the long-stator linear synchronous motor (LSM) that moves it, the contacts it can meet, the air, heat and power around it, the sensors the controller reads, and the floor it is mounted on. The building is an input (floor acceleration), except in the research-tier Structure tier (§21).

**Frames and signs.** These symbols mean the same in SPECIFICATION §1.2, in the code and here (FAB-PRP-001, FAB-SW-005; `rs:plant/body::sign_conventions`):

| Symbol | Meaning |
|---|---|
| x | Travel along the rail. +x opens. This is the README's "lateral propulsion" |
| y | Cross-track. In code and docs, "lateral" always means y |
| z | Up. g > 0 is gravity's magnitude |
| gap g | Distance from pole face to rail underside, > 0. A rising carriage closes the gap |
| pitch, roll, yaw | Rotations about y, x and z |
| Tick | One 20 kHz base period, 50 µs. Time is `Tick(u64)`, never an absolute f32 |
| PosNm | Absolute x position as i64 nanometres |

- **Origins.** The closed stop is the travel origin, `rail.x_closed_m` = 0 (+x opens toward x_closed + travel). Carriage-frame offsets (element stations, CoM) are measured from the carriage reference point: the rail centreline, mid-carriage along x, at the pole-face plane (station z = 0); its travel coordinate is the x that the encoder reads.
- **Attraction is +.** An element's force toward the rail (closing the gap) is positive. A lift element (`normal_sign` +1) attracts upward; a pure-EMS hold-down element (−1, under a lower flange) attracts downward.
- **Floor frame.** x, v and a in `Truth` are relative to the rail (floor frame). The floor acceleration enters the body as the inertial load −m·a_floor (§5). `Truth.acc_abs_mps2` is inertial.
- **Units.** SI throughout with unit-suffixed names (`_m`, `_s`, `_n`, `_kg`, `_a`, `_v`, `_t`, `_mps`, `_n_per_m`, …); seismology uses `r_km` and `*_kmps`. Standard gravity g₀ = 9.80665 m/s² (3rd CGPM 1901; `derive::G0_MPS2`); μ₀ = 1.25663706212e-6 H/m (CODATA 2018; `derive::MU0_H_PER_M`); ρ_air = 1.204 kg/m³ at 20 °C (ISO 2533).
- **Numerics.** The plant and `params::derive` are f64; the controllers (`fabrail-core`, `fabrail-exo`) are f32 and cast their parameters once in `new()`; the monitor (`fabrail-safety`) derives integer thresholds in `new()` and is integer-only per tick (§18, §24). All transcendental and rounding math goes through `fabrail_types::fmath` (soft-float libm; §24, §26).

**Model tiers** (`fabrail_plant::model::Tier`; details in §5):

| Tier | States | Step | Used for |
|---|---|---|---|
| Carriage6 | CoM position and velocity, attitude quaternion, body rates; per element coil current and eddy-lagged flux; LSM i_d, i_q; DRAWER cavity Δp; coil and magnet temperatures; hold-up voltage; 13 ledger quadratures (RK4); four bristle states (exact split update) | RK4, 50 µs | All closed-loop acceptance, haptics, obstruction, termini, SIL |
| Heave1 | z, ż, one lumped current, one eddy-lagged flux, one coil temperature | RK4, 50 µs | Analytic anchors, gain studies, vertical pulse, compressed thermal ramp |
| Travel1d | x, ẋ, DRAWER cavity Δp, hold-up voltage, ledger (RK4); bristle states (exact split update); gap and contact flags discrete (kinematic heave) | RK4, 1 ms | Seismic Monte Carlo, end zones, EEW timelines, WALL/HEAVY acceptance |
| Structure | n-storey shear building plus the carriage stroke, ledger, hold-up voltage | RK4, 1 ms | AMD/TMD research only (FAB-SEI-021) |

The controller always runs at the 20 kHz base tick, whatever the tier (§24; Wave 2 decision D8 keeps it so).

---

## §2 Magnetic element (co-energy)

**Implemented in** `crates/fabrail-plant/src/magnet.rs` (truth, f64), with the design closed form `derive::pm_force` (σ_leak = 0, no saturation, full engagement) and the controller's f32 twin `core::levitation::design::ElementModel`.

**Magnet circuit.** One element is a pole of width w (across the track) and length l (along x), area A_p = w·l, with a magnet of thickness h_m and recoil permeability μ_rec behind it, and a coil of N turns:

```text
h′      = h_m/μ_rec                                   magnet reluctance length
M_pm    = k_σ·B_r(T)·h_m/(μ₀·μ_rec)                   PM MMF, A-turns; k_σ = 1/(1 + σ_leak)
B_r(T)  = B_r·(1 + α_Br·(T_mag − 20 °C))              α_Br = −0.12 %/K (NdFeB)
MMF     = M_pm(T) + N·i
```

Pure-EMS hold-down elements have M_pm = 0 and h′ = 0, so their force is μ₀A_p·N²i²/(4g²). The truth remanence is `br_scale·B_r` and the truth leakage `leak_sigma` (plant `Mismatch`; ideal σ_leak = 0 by default, 0.25 in sensitivity studies only).

**Permeance with lateral fringing and engagement.** At gap g (floored at `GAP_MIN_M` = 1 µm), lateral offset y and engagement e ∈ [0, 1]:

```text
f(g, y) = 1 − (g/w)·ln cosh(y/g)       floored at FRINGE_F_MIN = 1e-3
D       = 2g/f + h′
P       = e·μ₀·A_p/D
∂D/∂g   = 2/f − 2g·f_g/f²,   ∂D/∂y = −2g·f_y/f²
f_g     = −(1/w)·ln cosh(u) + (u/w)·tanh(u),   f_y = −tanh(u)/w,   u = y/g
∂P/∂x   = (de/dx)·μ₀A_p/D                      (rail ends, DRAWER extension; §10)
```

ln cosh is evaluated overflow-free as |u| + ln(1 + e^(−2|u|)) − ln 2 (series u²/2 − u⁴/12 below |u| = 1e-4).

**Co-energy and forces.** W′ = ½·MMF²·P. The linear flux is Φ_lin = MMF·P. The generalised force along coordinate q is evaluated from the force-producing flux Φ_f:

```text
F_q = ½·(Φ_f/P)²·∂P/∂q
F_attract = −½(Φ_f/P)²·∂P/∂g     (+ closes the gap)
F_y       = +½(Φ_f/P)²·∂P/∂y     (restoring: opposite in sign to y)
F_x       = +½(Φ_f/P)²·∂P/∂x     (pulls into engagement)
```

With Φ_f = Φ = Φ_lin (below the saturation knee, no eddy lag) this is exactly ∂W′/∂q at constant current, so the element is conservative (`rs:plant/magnet::coenergy_force_consistency`, central differences, rel 1e-7). At y = 0 and e = 1 it reduces to the closed form

```text
F(g, i) = μ₀·A_p·(M_pm + N·i)²/(2g + h′)²          (derive::pm_force; §3)
```

(`rs:plant/magnet::closed_form_y0`, (E)). The small-offset lateral stiffness is k_y = F_z/w per element (F_y ≈ −F_z·(g/w)·tanh(y/g)); DOOR 12.3 kN/m per magnet (`guidance::reluctance_ky_n_per_m`).

**Saturation (W1.5 knee).** Saturation acts on the flux through the pole face, Φ_sat = B_sat·A_p (B_sat = 1.5 T), the same cap as `derive::element_force_max`. Below a knee K = `SAT_KNEE_FRAC`·Φ_sat the flux is exactly linear; above it it approaches Φ_sat along a tanh that is C² at the knee:

```text
Φ = Φ_lin                                               |Φ_lin| ≤ K
Φ = sign(Φ_lin)·(K + (Φ_sat − K)·tanh((|Φ_lin| − K)/(Φ_sat − K)))   otherwise
SAT_KNEE_FRAC = 0.7  [UNCERTAIN]
```

Electrical steel keeps a high permeability up to about 1.0–1.1 T (≈ 0.7·B_sat), so the knee is a design choice. A pure tanh from zero would cut the force at B_g = 0.55 T by about 8 % and move every zero-power gap by about 0.4 mm, breaking the LEV-004 and derive anchors (Wave 1.5 finding). The rail flange is sized separately (§10, FAB-LEV-015); at the minimum flange the rail already runs at 83–96 % of B_sat at nominal flux, which the pole-face cap does not model (§26). `rs:plant/magnet::saturation_bound` asserts |Φ| < Φ_sat.

**Eddy lag.** The force-producing flux lags the circuit flux through τ_e·Φ̇_eff = Φ − Φ_eff (`suspension.tau_eddy_s` = 1.0 ms in every preset, the centre of the 0–2 ms band of FAB-LEV-020; laminated or SMC rail assumed). Carriage6 and Heave1 integrate Φ_eff as a state when the eddy lag is enabled and τ_e ≥ 2h; otherwise Φ_eff = Φ. The lag is a heuristic on the force flux only; it is what the discrete levitation design is robust to (§15).

**Incremental inductance and motional EMF** (used by §4): ∂ψ/∂i = N·∂Φ/∂i + L_σ with ∂Φ/∂i = N·P·(dΦ/dΦ_lin); the coil sees the motional EMF e_mot = N·(∂Φ/∂g·ġ + ∂Φ/∂y·ẏ + ∂Φ/∂x·ẋ + ∂Φ/∂T·Ṫ_mag). L_σ = 0 in every preset.

**Demagnetisation proxy** (FAB-ENV-004, S). The model carries no irreversible-loss law. The proxy takes the SH-grade temperature limit `thermal.t_mag_max_c` = 80 °C (also the OverTemp threshold, §18) and the series circuit above with the coil driving −N·i_pulse against the magnet:

```text
B   = μ₀·(M_pm(80 °C) − N·i_pulse)/(2g + h′)        over g ∈ [g_up, g_land]
H_m = (B − B_r(80 °C))/(μ₀·μ_rec)                    magnet operating field
margin = 1 − |min H_m|/H_knee,   H_knee = 0.90·H_cJ(20 °C)·(1 − 0.006·(80 − 20))
```

with the SH-grade H_cJ(20 °C) = 1592 kA/m and β = −0.60 %/K \[UNCERTAIN\] material data held by the test (`py:environment::demag_margin_proxy`, which reports the margin per class and asserts it is positive).

**Constants** (module constants, not parameters): `FRINGE_F_MIN` 1e-3; `GAP_MIN_M` 1 µm; `SAT_KNEE_FRAC` 0.7 \[UNCERTAIN\]; `T_REF_C` 20 °C.

**Verified by** `rs:plant/magnet::{coenergy_force_consistency, closed_form_y0, saturation_bound, stiffness_sum_nonpositive, small_y_lateral, tempco_and_leakage_scale_the_mmf}` (FAB-LEV-001, FAB-LEV-002).

## §3 Linearisation and closed forms

**Implemented in** `crates/fabrail-params/src/derive.rs` (f64, the single source of every closed form; `Params.derived()` and PARAMETERS.md read the same table).

At zero current, y = 0, full engagement and σ_leak = 0, with D = 2g + h′:

```text
F(g, i)   = μ₀A_p·(M_pm + N·i)²/D²
k_s       = −∂F/∂g = 4μ₀A_p·(M_pm + N·i)²/D³          negative (destabilising) stiffness per element
k_i       =  ∂F/∂i = 2Nμ₀A_p·(M_pm + N·i)/D²          current gain per element
p         = √(n·k_s/m*) = √(4g₀/(2g* + h′))           open-loop (Earnshaw) heave pole, balanced at g*
```

The pole identity holds because the presets balance F(g*) = m*·g₀/n exactly (below).

**Zero-power mass law** (Mizuno): at i = 0 a mass floats where n·F_pm(g) = m·g₀:

```text
m(g) = m*·((2g* + h′)/(2g + h′))²
g(m) = ((2g* + h′)·√(m*/m) − h′)/2
```

**Bias current** that holds mass m at gap g with all n lift elements equal:

```text
i(g, m) = ((2g + h′)·√(m·g₀/(n·μ₀·A_p)) − M_pm)/N
```

**Stop ratios.** The magnet-only force ratio n·F_pm(g)/(m·g₀) gives the upper-stop rule (≤ 1.30 at g_up with m* for DOOR, WALL, HEAVY) and the stay-landed rule (≤ 0.90 at g_land with m_op_min, core design F18). The DRAWER upper stop is an elastomer roller rated 600 N ≥ n·F_pm(g_up) = 551.5 N (F2).

**Magnet sizing** (`size_magnets`, `balance_pole_length`): A_p = μ₀·m*·g₀/(n·B_g²) and h_m = 2g*·B_g·μ_rec/(B_r − B_g). After rounding h_m to a manufacturable thickness the presets solve the pole length l = m*·g₀·(2g* + h′)²/(n·μ₀·M_pm²·w) so that equilibrium sits exactly at g* (LEV-004 needs ±2 µm, which the rounded 10 × 32 mm pole would miss). This is why the exact DRAWER lift currents are +11.20 / −4.99 A, not the rounded core-design +11.23 / −4.98 A.

**Thermal gap shift** (§12): Δg = (2g* + h′)·α_Br·ΔT/2 (the zero-power gap scales with M_pm).

**Impact speed after a control loss** (§15, FAB-LEV-019), coil current 0, no lag or dissipation, from rest at g₀:

```text
up:    ½mv² = ∫_{g_up}^{g₀} n·F dg − m·g₀·(g₀ − g_up)
down:  ½mv² = m·g₀·(g_land − g₀) − ∫_{g₀}^{g_land} n·F dg
∫ n·F dg = ½·n·μ₀A_p·M²·(1/(2g_a + h′) − 1/(2g_b + h′))
```

**Values per class** (presets; `Params.derived()`):

| Quantity | DRAWER | DOOR | WALL | HEAVY |
|---|---|---|---|---|
| h′ / M_pm | 3.124 mm / 3232 A | 7.333 mm / 7586 A | 17.14 mm / 17734 A | 17.14 mm / 17734 A |
| A_p (w × l) | 10 × 32.06 mm | 20 × 50.92 mm | 43 × 99.51 mm | 43 × 497.6 mm |
| F* per element | 63.74 N | 245.2 N | 1225.8 N | 6129.2 N |
| k_s / k_i per element | 31.39 kN/m / 7.890 N/A | 56.58 kN/m / 19.39 N/A | 132.0 kN/m / 41.47 N/A | 660.1 kN/m / 138.2 N/A |
| p heave / pitch / roll, rad/s | 69.49 / 45.93 / 107.9 | 47.57 / 27.93 / passive | 32.50 / 23.13 / passive | 32.50 / 32.38 / 17.32 |
| Stop ratio at g_up (m*) / stay-landed (m_op_min) | 2.163 / 0.8250 | 1.278 / 0.8487 | 1.256 / 0.8631 | 1.256 / 0.8631 |
| Thermal shift for +40 K | −0.1950 mm | −0.4160 mm | −0.8914 mm | −0.8914 mm |
| Impact speed up / down at g*, m* | 109.6 / 197.3 mm/s | 50.58 / 55.59 mm/s | 68.81 / 76.27 mm/s | 68.81 / 76.27 mm/s |

The rounded stay-landed quotes in SPECIFICATION (0.824 / 0.850) differ from the exact 0.8250 / 0.8487 through the balanced pole length; tests hold them to ±0.002 (P). DRAWER: m(1.5 mm) = 45.76 kg, m(5.0 mm) = 9.963 kg, m(6.2 mm) = 7.120 kg; float gaps at 48 / 26 / 8 kg are 1.4276 / 2.500 / 5.761 mm; bias at the 3.5 mm seat +11.20 A (48 kg), +3.978 A (26 kg), −4.989 A (8 kg); from the 6.5 mm rollers 27.42 A at 48 kg, which exceeds V_bus/R = 15 A (F17, the reason for the captive seat). DOOR +10 kg (98.1 N) floats at 4.5967 mm; WALL +50 kg at 9.1357 mm. Band clamp at 48 kg and 1.5 mm: +0.391 A (F1).

**Modal quantities** (`derive::mode_model`, the per-element model the gain design uses): heave m_eff = m*/n, k_eff = k_s. Pitch and roll use lever arms r_k = x_k − x_c (pitch) or y_k − y_c (roll) over the lift elements, with m_eff = J/Σr², k_eff = (k_s·Σr² + m*·g₀·h_cg)/Σr², h_cg the CoM height above the pole plane. A CoM below the elements (top-hung leaves, h_cg < 0) makes roll passive: DOOR magnetic roll stiffness k_s·Σy² = 51 N·m/rad against the gravity term m*g₀·|h_cg| = 1030 N·m/rad (FAB-LEV-001). The mode pole is √(k_eff/m_eff); a mode with k_eff ≤ 0 gets no zero-power loop.

**Verified by** `rs:params/derive::{open_loop_pole, zero_power_mass, zero_power_gap_anchors, seat_unseat_currents, stop_ratio_table, k_i_and_inductance_table, size_magnets_matches_table, thermal_gap_shift_values, impact_speed_energy, pm_force_consistency}`, `py:physics_anchors::{earnshaw_and_roll_authority, zero_power_mass, stop_force_ratios, thermal_gap_shift, impact_speeds_and_manual_force, open_loop_pole_and_caps}`, `rs:plant/tests/anchors::{stop_ratios, impact_speed_closed_form}`.

## §4 Coil and current loop

**Implemented in** `crates/fabrail-plant/src/coil.rs` (truth), `crates/fabrail-core/src/current.rs` (controller, base task), the I²t clamps in `core::levitation::I2tClamp` and `fabrail-safety/src/envelope.rs::CoilI2t`.

**Coil mode has one source**, the scenario's `fidelity.coil`, and must equal `control.rates.coil_mode_voltage` (every preset: voltage). SIL ≡ firmware is claimed in voltage mode only (FAB-FW-006).

**Voltage mode (truth).** The coil current is an RK4 state:

```text
di/dt = (v − R(T)·i − e_mot)/(∂ψ/∂i)          |v| ≤ V_bus
R(T)  = R₂₀·(1 + α_R·(T_coil − 20 °C))        α_R = +0.393 %/K (copper)
```

At the driver limit (|i| ≥ limit) the bridge chops: di/dt is set to 0 whenever it would push |i| further. With the coils off (`CoilCmd::Off`) the current decays through the freewheel path at τ = L/R.

**Lag mode (sim only).** The current follows the exact solution of τ_cl·di/dt = i* − i within each plant step, i(s) = i* + (i₀ − i*)·e^(−s/τ_cl), with τ_cl = 1/(2π·`current_bw_hz`) = 0.159 ms. The change is slew-limited by the voltage headroom |di/dt| ≤ (V_bus − |R·i + e_mot|)/L_inc and clamped at the driver limit.

**Current PI (controller, 20 kHz base task, voltage mode).** Per element:

```text
ω_c  = 2π·current_bw_hz = 2π·1.0 kHz                      (plan §2b item 2)
K_p  = L̂·ω_c,  K_i = R̂·ω_c,  K_aw = ω_c
L̂    = N²·μ₀A_p/(2g* + h′) + L_σ                          (derive::coil_inductance)
u    = K_p·e + I + v_ff,   e = i* − i_meas
v    = clamp(u, −V_bus,nom, +V_bus,nom)
I   += T_base·(K_i·e + K_aw·(v − u))                      back-calculation anti-windup
```

The PI zero cancels the coil pole R/L, so the continuous loop is ω_c/s; with the ZOH and the one-sample duty delay (PWM computed at tick k applies at k + 1) the discrete small-signal step settles with a few percent overshoot (LEV-008: rise ≤ 0.3 ms, overshoot ≤ 10 %; at 1.5 kHz the delay gave ≈ 22 %). The limits use the nominal `design.power.v_bus_v`: `BaseHal` carries no 20 kHz bus sample. The motional-EMF feed-forward v_ff = −k̂_i·ġ̂ per element (`LevOutput.emf_ff_v`) exists but is not routed to `CurrentLoops::set_emf_ff` in Wave 1.5, so v_ff = 0 and the PI integral absorbs the EMF (`rs:core/current::emf_feedforward_cancels_motional_emf` tests the path in isolation).

**Coil values:**

| | DRAWER | DOOR | WALL | HEAVY |
|---|---|---|---|---|
| N / R₂₀ | 200 / 1.6 Ω | 300 / 1.7 Ω | 300 / 0.7 Ω | 200 / 0.6 Ω |
| L at g* / τ = L/R | 1.983 mH / 1.240 ms | 6.645 mH / 3.909 ms | 13.03 mH / 18.61 ms | 28.95 mH / 48.26 ms |
| i_cont / i_pulse / t_pulse | 4 / 12 A / 0.2 s | 6.5 / 10 A / 0.2 s | 15 / 20 A / 0.2 s | 22 / 40 A / 0.2 s |
| I²t budget B = (i_pulse² − i_cont²)·t_pulse | 25.6 A²s | 11.55 A²s | 35.0 A²s | 223.2 A²s |

**I²t clamp** (plan §2b item 4). Per element the excess E = ∫(i² − i_cont²)dt, floored at 0, is accumulated from measured currents. The plant driver folds its limit from i_pulse back to i_cont once E ≥ B and releases it when E has recovered to 0 (`coil::I2tBudget`). The core clamps its references to i_cont once E ≥ `I2T_BUDGET_FRAC`·B = 0.95·B (so the monitor never trips first) and ramps never command more than `RAMP_ENVELOPE_FRAC`·i_pulse = 0.98·i_pulse. The monitor mirrors the budget in integer code²·ticks and trips CoilI2t above 1.1·B and CoilOvercurrent above 1.1·i_pulse (§18).

**Force slew** (FAB-PWR-005): available k_i·V_bus/L at g*, required m*·g₀/n·ω_heave (a full element weight share within one loop time constant).

**Verified by** `rs:core/current::{pi_step_response, antiwindup_recovers}` (FAB-LEV-008), `rs:plant/coil::{lag_step_is_exact, voltage_mode_rate_and_chopping, i2t_budget_folds_back_and_recovers}`, `rs:plant/tests/anchors::power_balance_voltage_mode` (rel 1e-6; measured 1e-9), `py:params::bus_voltage_and_slew`.

## §5 Rigid-body tiers

**Implemented in** `crates/fabrail-plant/src/{body.rs, heave1.rs, travel1d.rs, structure.rs, model.rs, integrator.rs}`.

**Carriage6.** State: CoM position r_c and velocity v_c in the rail frame, attitude q (body → rail, renormalised after every step), body-frame ω. The inertia is J = diag(`carriage.j_kgm2`) about the CoM scaled by m/m* (uniform-density payload).

```text
m·v̇_c = Σ_k F_k + F_lsm + F_contacts + F_guide + F_drag + F_ext − m·g₀·ẑ − m·a_floor
J·ω̇   = Rᵀ·Σ(r × F) − ω × Jω
q̇     = ½·q ⊗ (0, ω)
```

- The reference point (rail centreline, rail underside, mid-carriage) sits at p_ref = r_c − R·c (c = `carriage.cg_m`); `Truth.pose` reports it, with z = 0 at the zero-power position (every gap at g*).
- Element k at body offset r_k: P_k = r_c + R·(r_k − c); gap g_k = g* + s_k·(r_kz + z_rail(x_k) − P_kz) (s_k = +1 lift, −1 hold-down); lateral offset y_k = P_ky − r_ky − y_rail(x_k); engagement e_k from `rail::engagement` with a central-difference de/dx (§10). The element force is mapped through the gap and offset gradients, rail slopes included.
- Loads applied at the CoM: drag, bristle friction, hand, lean, pressure, obstacle, bumpers, lock and latch (their small moments are neglected). The LSM thrust and the iron-core normal force act at the reference point; the guide (fin or pads) at the guide point, which is the bottom of a top-hung leaf (lever 2·|c_z| below the rail) or the reference point otherwise (§6).
- The hand force is solved with the algebraic loop through the arm mass (§22).

**LSM thrust moment and CoM motion control (W1.5).** The thrust F_x acts at the rail reference point, a lever h = −c_z from the CoM, so it applies the pitch torque τ_y = h·F_x. On a top-hung DOOR (h = 1.05 m) the encoder, which reads the rail reference point, then sees an effective mass 1/(1/m + h²/J_y) ≈ 0.3·m above the pitch bandwidth, and the W1.0 loop diverged through the momentum observer. Two changes close the loop: the levitation cancels the torque with a pitch feed-forward current (§15), and the motion loop controls the CoM, x_c = x_enc − h·θ̂_y, with θ̂_y the levitation's pitch estimate clamped to ±5 mrad (`fast::PITCH_CORR_MAX_RAD`; §16).

**Carriage geometry** (presets; principal inertias from a box model at m*, scaled by m/m* with payload):

| | DRAWER | DOOR | WALL | HEAVY |
|---|---|---|---|---|
| m* / body length | 26 kg / 0.6 m | 100 kg / 1.0 m | 500 kg / 2.0 m | 5000 kg / 3.0 m |
| J_x / J_y / J_z (kg·m²) | 0.433 / 0.867 / 1.127 | 36.76 / 45.08 / 8.35 | 375.4 / 541.7 / 167.1 | 3021 / 6354 / 4167 |
| CoM (x, y, z) from the reference point | (−0.14, 0, +0.08) m | (0, 0, −1.05) m, top-hung | (0, 0, −1.5) m, top-hung | (0, 0, +1.25) m |
| Lift stations (x; y) | −0.02, −0.26; ±0.2 m | ±0.40; ±0.015 m | ±0.75; ±0.025 m | ±1.3, ±0.9; ±0.4 m (8) |

**Heave1.** The n lift elements share one gap g = g* + z̄_rail − z (z the reference-point heave, z̄_rail the mean rail deviation under the lift stations at a fixed travel x₀), one current (the mean of the lift elements' commands), one eddy-lagged flux and one coil temperature; the force is n·F(g) with the same element model, contact laws (per element, m_eff = m/n) and coil electrics as Carriage6. A symmetric, untilted Carriage6 heave reproduces it to round-off (`rs:plant/tests/anchors::tier_equivalence`, rel 1e-9; measured 8.5e-12). Hold-down elements and x-axis inputs are not modelled.

**Travel1d.** Levitation is ideal: the gap stays at g* while levitated. x, ẋ, the DRAWER cavity Δp, the hold-up voltage and the ledger are RK4 states at 1 ms; the four bristle states (rollers, upper, seat, latch) are held through the step and advanced exactly afterwards (§8); the gap and the contact flags are discrete and move once per step. With `kinematic_heave` (plan §2b item 16) the gap follows the harness's `HeaveHint`: `Land` ramps it to the support at the class land speed (5 / 5 / 3 / 2 mm/s), `Seat` to g_seat, `Lift` back to g* within the class lift time, `Drop` falls at `DROP_SPEED_MPS` = 0.1 m/s; lift-off is at 20 µm. Resting normals are PM-relieved (core design F8):

```text
N = m·(g₀ + a_floor,z) − Σ F_pm(gap) − F_n,LSM       seat: |N| (captive both ways); rollers: max(N, 0)
```

Loads along x share Carriage6's assembly (friction, drag, bumpers, lock, latch, obstacle, cavity, lean, pressure, hand, LSM, guide-fin friction at y = 0) plus −m·a_floor,x; the magnets' engagement pull along x is omitted. `travel1d::host_reaction_x_n` = −m·(a_rel + a_floor,x) is the reaction on the host storey. Between plant steps SimBoard extrapolates the encoder kinematically (x + v·τ + ½a·τ²), so the 20 kHz controller sees a smooth position; commands are averaged over the 20-tick window (ZOH).

**Structure** (research tier) is specified in §21.

**Energy audit.** Carriage6 exposes kinetic (translational and rotational), gravitational, co-energy, stored magnetic energy, Σ M_eff·Φ, leakage, cogging and LSM-normal potentials (`body::EnergyAudit`); the conservation anchor holds E = KE + m·g₀·z − ΣW′ (plus the cogging and LSM-normal potentials) to rel 1e-8 over 50 ms of free 6-DOF motion with saturation, eddy lag, dissipation and control off at constant current (measured 1e-13).

**Verified by** `rs:plant/body::{sign_conventions, reset_levitates_at_the_zero_power_gap, rest_states_hold_still}`, `rs:plant/tests/anchors::{energy_conservation, tier_equivalence}`, `rs:plant/travel1d::{kinematic_land_completes_at_the_class_speed, free_mass_follows_the_floor_inertially}`, `rs:plant/heave1::open_loop_pole_fit`.

## §6 Guidance, sway, roll

**Implemented in** `crates/fabrail-plant/src/guidance.rs` (fin, pads, sway damping) and the magnet model (reluctance centring, §2).

- **Passive lateral reluctance.** Each PM element centres itself with k_y = F_z/w (first order of F_y ≈ −F_z·(g/w)·tanh(y/g)); passive yaw stiffness Σ k_y·(x_k − x_cg)².
- **Guide fin or side pads.** A pair of unilateral UHMW stops at ±clearance (fin 2 mm on DOOR/WALL/HEAVY; DRAWER side pads 1 mm), Hunt–Crossley F_n = k·δ·(1 + hc·δ̇) with δ = |y| − clearance, k = `K_UHMW_N_PER_M` = 5e6 N/m \[UNCERTAIN\], hc = `HC_UHMW_S_PER_M` = 1.5 s/m \[UNCERTAIN\]. Contactless in nominal operation. Friction along x at the stop: −μ·F_n·tanh(v_x/1 mm/s), μ = `guidance.pad_mu` = 0.15 (within 0.5 % of μN above 3 mm/s, inside LEV-011's ±5 %).
- **Sway damping** F_y = −c_sway·ẏ at the guide point (`guidance.c_sway_n_s_per_m` = 20 / 50 / 200 / 1000 N·s/m).
- **Top-hung sway and roll.** A top-hung leaf is a pendulum about the rail, ω = √(g₀/ℓ) with ℓ = |c_z| (DOOR 1.05 m: 0.49 Hz). Roll is passive on DOOR and WALL (§3). The presets size c_sway for ζ ≈ 0.08 treating the damper at the CoM, ζ = c_sway/(2m·√(g₀/ℓ)); as implemented the damper acts at the fin point, 2·|c_z| below the rail, so the rotational pendulum about the rail has ζ = c_sway·(2ℓ)²/(2·√(m·g₀·ℓ·(J_x + m·ℓ²))) ≈ 0.28 on DOOR. Both satisfy LEV-011 (ζ ≥ 0.05); `py:levitation::top_hung_sway_push` measures the implemented value.

**Roll–sway coupling on HEAVY (Wave 2 decision D1).** HEAVY is not top-hung: its CoM sits 1.25 m above the pole plane on twin pole rows at y = ±0.4 m, and roll is actively controlled. The lateral reluctance acts at the pole plane, 1.25 m below the CoM, so sway y_c and roll θ are coupled. With K_y = Σk_y the total lateral reluctance stiffness and k_r = k_s·Σy² + m·g₀·h the destabilising roll stiffness about the CoM (magnets plus gravity):

```text
[m  0 ] [ÿ_c]   [ K_y       −K_y·h        ] [y_c]
[0  J_x] [θ̈  ] + [−K_y·h    K_y·h² − k_r  ] [θ  ] = control and damping terms
```

The per-mode design of §15 keeps only J_x·θ̈ = k_r·θ + control, i.e. a pure roll about the CoM with pole √(k_r/J_x). For HEAVY K_y = 8 × 142.5 kN/m = 1.140 MN/m, K_y·h² = 1.78 MN·m/rad and k_r = 0.845 + 0.061 = 0.906 MN·m/rad: the coupling term is twice the stiffness the design balances. The open-loop eigen-analysis of the coupled pair gives one unstable mode at 10.4 rad/s, a rotation about a point 0.85 m below the CoM (y_c/θ = 0.845 m), and one stable sway–roll oscillation at 25.0 rad/s (4.0 Hz), against the designed pure-roll pole of 17.3 rad/s. The design therefore assumes about 3.6 times too little roll inertia for the mode it must stabilise, which is consistent with the W1.5 observation that HEAVY Carriage6 rolls away within about 0.6 s at hover (BusUnder / SaturationPersist in every HEAVY Carriage6 catalog entry). On DRAWER the same analysis moves the roll pole only from 107.9 to 106.3 rad/s (K_y·h² = 163 N·m/rad against k_r = 5042 N·m/rad), so the decoupled design is adequate there.

**Wave 2 decision D1 (coupled design in code).** `derive::roll_sway_model` carries the coupled model in the controller's own coordinates (u = r̄·∂g/∂y = −r̄·θ the modal roll gap, Y the CoM sway, the lateral reluctance and the sway damper c = `guidance.c_sway` acting at the element plane, y_m = Y + h·θ):

```text
J·θ̈ = (m*·g₀·h + k_s·Σr² − n·k_y·h²)·θ − n·k_y·h·Y − c·h·(Ẏ + h·θ̇) + n·k_i·r̄·i_e
m*·Ÿ = −n·k_y·(Y + h·θ) − c·(Ẏ + h·θ̇)
coupling ratio n·k_y·h²/(k_s·Σr² + m*·g₀·h):  HEAVY 1.97, DRAWER 0.032
```

A roll loop whose coupling ratio is at least `ROLL_SWAY_COUPLED_MIN` = 0.25 (HEAVY in the presets) is designed on this model (§15); the others keep the single-DOF design. With k_y = c = 0 the model reduces to the single-DOF roll model of §3. The single-DOF roll loop closed on the coupled HEAVY plant has spectral radius 1.0019 (a 2.3 Hz roll–sway oscillation growing at 9.5 s⁻¹, the W1.5 divergence), and is neutral on DRAWER (1.000003, sway damped by the side pads). *The coupled design and its linear robustness sweep are in code; whether HEAVY Carriage6 now runs its closed-loop catalog green is finalised by Wave 2.5 (capability matrix TODO row): if not, HEAVY Carriage6 stays 🔶 and HEAVY acceptance runs on Heave1/Travel1d.*

**Verified by** `rs:plant/guidance::{contactless_inside_the_clearance, fin_pushes_back_and_drags_mu_n, drawer_uses_side_pads, reluctance_stiffness_and_sway_anchors}`; LEV-011 by `py:levitation::top_hung_sway_push` (Wave 2).

## §7 LSM

**Implemented in** `crates/fabrail-plant/src/lsm.rs` (truth), `crates/fabrail-core/src/foc.rs` (transforms, exact angle, firmware d/q PI), `crates/fabrail-core/src/motion/cascade.rs` (ripple table).

**Convention (Wave 2 decision D5, binding).** fabrail uses the amplitude-invariant Clarke/Park transform: a balanced set of phase-peak current I gives |i_dq| = I, and Σi_abc² = 1.5·I² at every electrical angle.

```text
i_α = i_a,  i_β = (i_a + 2·i_b)/√3                     Clarke (amplitude-invariant)
P_elec = 1.5·(v_d·i_d + v_q·i_q)
F      = K_f·i_q                                       thrust constant K_f, N/A
e_q    = K_e·v,   K_e = (2/3)·K_f                       back-EMF constant, V·s/m (phase peak)
F·v    = 1.5·e_q·i_q                                    electromechanical power identity (FAB-PRP-002)
```

The identity F·v = K_f·i_q·v = 1.5·K_e·v·i_q fixes K_f = 1.5·K_e (`derive::lsm_k_e`, `lsm::KE_PER_KF` = 2/3). The plant derives K_e from K_f(x) and never reads `design.lsm.k_e_v_s_per_m`; that design field must agree with it (`params::validate`), and the presets store K_e = (2/3)·K_f: DRAWER 10.0, DOOR 20.0, WALL 33.33, HEAVY 66.67 V·s/m. The firmware d/q PI's back-EMF feed-forward uses the same e_q = (2/3)·K_f·v.

**History.** Waves 0–1.5 stored K_e = K_f/√3 and used the ASC closed form 1.5·K_f·K_e/R (≈ 0.866·K_f²/R), which contradicts F·v = 1.5·e_q·i_q: a shorted winding would brake at (2/3)·K_f²/R, about 0.77 of that. Wave 2 decision D5 made the convention above binding and the code follows it (`lsm.rs`, `structure.rs`, `derive::{lsm_k_e, asc_drag, asc_drag_low_speed}`, `validate`). SPECIFICATION's FAB-PRP-002 text still reads "K_f = √3·K_e"; the spec owner updates the prose.

**Thrust.** The inverter's d/q current loop is a first-order lag τ_q = `TAU_CURRENT_S` = 0.1 ms, applied exactly within each step; |i_dq| ≤ i_max.

```text
θ_e     = π·x/τ_p                                        (plant, f64)
K_f(x)  = K_f·w_seg(x)·(1 + r₆·cos 6θ_e)                 r₆ = ripple_frac = 1 %
F_x     = K_f(x)·i_q + F_cog(x)
F_cog   = F_c·sin 6θ_e                                    iron-core (HEAVY) only; potential U = F_c·τ_p/(6π)·cos 6θ_e
F_n     = F_n,rated·max(0, 1 + i_d/i_max)                 iron-core normal attraction toward the stator (relieves the lift)
w_seg   = 1 − (o/L_seg)·½(1 + cos(π·d/(o/2)))              within o/2 of a stator joint (C¹ raised cosine), else 1
```

The segment weight models the hand-over dip at each joint of a segmented stator with depth equal to the overlap fraction o/L_seg (10 % on WALL and HEAVY) \[UNCERTAIN\]: the hand-over geometry is not modelled in detail.

**Controller side.** The electrical angle is exact at any distance, θ_e = 2π·(x_nm mod 2τ_p,nm)/(2τ_p,nm) with the modulo taken in integers (`foc::elec_angle`, FAB-PRP-009). The cascade's ripple table inverts the same model: i_q* = (F* − F̂_cog)/(K̂_f·(1 + r₆·cos 6θ_e)), clamped to ±i_max; the modelled thrust of the measured i_q is K̂_f·i_q·(1 + r₆·cos 6θ_e) + F̂_cog (the momentum observer's actuator force). W1.5 aligned the cogging sign with the plant (+F_c·sin 6θ_e); the earlier opposite sign doubled HEAVY's cogging instead of cancelling it.

**STO and ASC** (propulsion only; §18). STO: gates off, the currents decay through the τ_q lag. ASC: the phases are shorted (v_d = v_q = 0); the steady shorted-winding solution at electrical speed ω_e = π·v/τ_p gives

```text
i_q = −(e_q/R)/(1 + x²),   i_d = x·i_q,   x = ω_e·L/R
F   = K_f·i_q = −b_e(ω_e)·v
b_e = K_f·K_e/R/(1 + x²) = (2/3)·K_f²/R/(1 + x²)          (D5)
P   = 1.5·R·(i_q² + i_d²) = b_e·v²                        booked as LSM copper loss
```

The plant applies it algebraically (the reported i_q = F/K_f(x), i_d = x·i_q), with K_e = (2/3)·K_f(x) so it carries the same w_seg·ripple factor as the thrust; the copper loss equals the braking power exactly and no bus power flows. Low-speed drag b_e(0) = (2/3)·K_f²/R (`derive::asc_drag_low_speed`): DRAWER 75, DOOR 400, WALL 1667, HEAVY 13 333 N·s/m. ASC engages only above v_runaway (2·v_cap) or during a seismic landing and is released to STO at standstill on egress classes (F9, FAB-SAF-011).

**LSM values:**

| | DRAWER | DOOR | WALL | HEAVY |
|---|---|---|---|---|
| Kind / τ_p | ironless Halbach / 15 mm | ironless / 25 mm | ironless segmented / 40 mm | iron-core segmented / 50 mm |
| K_f / R_ph / L_ph | 15 N/A / 2 Ω / 2 mH | 30 N/A / 1.5 Ω / 5 mH | 50 N/A / 1 Ω / 10 mH | 100 N/A / 0.5 Ω / 20 mH |
| F_peak / F_cont / i_max | 90 / 20 N / 6 A | 150 / 50 N / 5 A | 400 / 100 N / 8 A | 2000 / 500 N / 20 A |
| Cogging / normal force | — | — | — | 20 N / 2500 N |
| Segments (length / overlap) | — | — | 1 m / 0.1 m | 2 m / 0.2 m |

**Verified by** `rs:plant/lsm::{kf_ke_and_power_balance, asc_drag_low_speed_limit, ripple_segments_and_cogging}` (FAB-PRP-002), `rs:params/derive::lsm_ke_and_asc_drag`, `rs:core/foc::{angle_exact_far, clarke_park_roundtrip, dq_pi_tracks_q_current}` (FAB-PRP-009), `rs:core/motion/cascade::iq_ref_inverts_thrust`.

## §8 Contacts and friction

**Implemented in** `crates/fabrail-plant/src/{contact.rs, friction.rs}` and `body.rs::vertical_contacts`, `x_loads`.

**Normal laws.** Penalty contacts F_n = max(0, k_c·δ + c_c·δ̇) for penetration δ > 0 (never pull), with c_c = 2ζ_c·√(k_c·m_eff), ζ_c = 0.3. Impact-type contacts (end bumpers, guide fin, side pads) use Hunt–Crossley F = k·δ·(1 + hc·δ̇) (linear-stiffness form), whose dissipation k·hc·δ·δ̇² vanishes at first touch [R45].

**Stability cap.** Every contact is capped at use to k_c ≤ m_eff·(0.2/h)² and c ≤ 2√(k·m_eff), i.e. ω·h ≤ 0.2 on its own mode (§24). Effective masses use the lightest body (min(m_body, m_op_min)): vertical contacts the smallest per-support share over heave m/n, roll J_x/Σy² and pitch J_y/Σx²; lateral contacts the smaller of the translation, the yaw share J_z/(L/2)² and, on a top-hung leaf, the roll share J_x/c_z²; travel-axis contacts the whole mass. Vertical and lateral contacts are capped at the 50 µs step, travel-axis contacts (bumpers, lock, latch, edge seal) at the 1 ms Travel1d step, so one parameter set serves every tier.

| Contact | k_phys (before the cap) | Friction |
|---|---|---|
| Landing / travel rollers | max(1e8 N/m, rated share/5 µm) (steel) | bristle, μ 0.003 (plan §2b item 19) |
| DRAWER captive seat | as rollers | UHMW μ 0.15 |
| Upper stop (elastomer rollers) | max(1e7 N/m, rating share/50 µm) | μ 0.003 |
| Guide fin, side pads | 5e6 N/m Hunt–Crossley, hc 1.5 s/m | μ 0.15 (stateless, §6) |
| End bumpers | `damping.bumper_k_n_per_m` (120 k / 250 k / 450 k / 1.125 M N/m), hc 0.3 s/m | — |
| Parking lock, seismic latch | 1e6 N/m bilateral | latch slips at `x_retention_n` |
| Closing-edge seal | `damping.k_edge_n_per_m` 10 N/mm in series with the body (§22) | — |

**Vertical geometry** (per lift element, `vertical_contacts`). Upper stop at g_up; rollers (or skids) at g_land. DRAWER (`LandingKind::Seat`): within the slot (x_open = x − x_closed ≤ `slot_len_m` = 20 mm) the captive seat holds the gap in [g_seat − c_z, g_seat] (3.35–3.5 mm, c_z = `seat_z_clearance_m` = 0.15 mm); then an entry ramp rises from g_land − 3 mm to g_land over 30 mm; then the travel rollers at g_land. A drawer heavier than ≈ 17 kg rests on the lower seat face, a lighter one is pulled onto the upper face (so the rest height differs by up to 0.15 mm between light and heavy loads; FAB-LEV-017's "rest height independent of load to ±0.05 mm" holds only per face, a wording the spec owner has been asked to reconcile).

**Seat lip chamfer (W1.5).** Over the outer `SEAT_LIP_CHAMFER_M` = 5 mm of the slot the captive lip face rises linearly from the clearance band to the upper stop, with the x component of its normal force applied, so a drawer entering above the band is guided down instead of meeting a step penetration (a drawer floating at g* hit the lip as a 0.85 mm step and the plant produced NaN).

**Seat x constraint while seated** (`seat_x_force_n`): pushing in (x_open < 0) compresses the push-stop spring `seat_k_x_n_per_m` = 2 kN/m over `seat_x_compliance_m` = 2.5 mm (push-to-open works while seated), then meets the hard stop; outward, the V-seat registration holds with the steel seat stiffness only within the notch `SEAT_REG_NOTCH_M` = 0.5 mm (W1.5: a 48 kg drawer brushing the seat face at x_open = 7 mm had been thrown at 6 m/s by a 7 mm step penetration).

**Bumpers and end zones.** The progressive eddy end zones span `damping.ez_len_m` = 150 mm beyond each nominal travel limit [x_closed, x_closed + travel] (§9), and the bumper faces sit at their outer edges, so nominal termini never engage either (SAF-014). The bumper energy rating is ½·m·v² at the basis speed: DRAWER ½·48·1² = 24 J (F13, a drawer on rollers is free along x during shaking), DOOR 112.5 J, WALL 562.5 J, HEAVY 5625 J at 1.5 m/s. `BumperLog` counts impact onsets, the largest onset energy ½mv² and an overload flag when ½k·δ² exceeds the rating.

**Physical end stops (Wave 2 decision D3).** In Wave 1.5 nothing stopped a closed DOOR at its jamb or a WALL at its end short of the bumper, 150 mm beyond the travel; under strong floor shaking free or landed devices slid past `safety.x_min_m` (x_closed − 10 mm) and the monitor tripped PositionLimit and UnintendedMotion (villa smoke, elcentro_free, egress_eew, power_loss_at_s). D3 adds physical stops at the class geometry (`contact::end_stops`), applied on every tier with a travel axis (Carriage6, Travel1d, Structure):

```text
closed end   DOOR, WALL, HEAVY: rigid jamb at x_closed − JAMB_CLEARANCE_M (2 mm)
             DRAWER: carcass push-stop spring from x_closed, hard stop at x_closed − seat_x_compliance (2.5 mm)
open end     rail end stop behind the bumper, at x_closed + travel + ez_len + √(2·E_rated/k_b)
             (the bumper bottomed out at its rated stroke: 20 / 30 / 50 / 100 mm)
law          F = max(0, k·δ + c·δ̇),  k = min(max(1e6 N/m, m_rated·g₀/0.25 mm), m·(0.2/h)²),  c = 2·0.3·√(k·m)
```

The jamb lies inside the governor's observation point (terminus + 5 mm overtravel), so nominal termini never touch it, and the closed-end part of the end zone and bumper lies behind it and is unreachable. On the 1 ms tiers the stiffness cap binds, and an impact at relative speed v penetrates about v·√(m/k) ≈ 5 mm per m/s, which keeps floor-driven motion against the jamb inside the 10 mm SLP band for relative speeds up to ≈ 2 m/s. While a DRAWER is seated the seat law above applies instead of the carcass stop (never both). The monitor side of D3 is in §18.

**Elasto-plastic bristle friction** (Dupont, Hayward, Armstrong & Altpeter 2002 [R44]). One bristle state z per tangential contact, coefficients normalised by the normal force:

```text
F_t     = −F_n·(σ0·z + σ1·ż + σ2·v)
μ(v)    = μ_c + (μ_s − μ_c)·exp(−(v/v_s)²)              Stribeck (off: μ_s = μ_c in every contact)
σ0      = μ_c/z_ss,   z_ss(v) = μ(v)/σ0,   z_ba = r_ba·z_ss(v),  r_ba = 0.7
ż       = v·(1 − α(z, v)·z/z_ss(v))
α       = 0 for |z| ≤ z_ba or sgn z ≠ sgn v (purely elastic)
        = z_ss·(|z| − z_ba)/(|z|·(z_ss − z_ba)) on the transition,   1 for |z| ≥ z_ss
σ1      = 2ζ_b·√(σ0/g₀),  ζ_b = 1 (critically damped stick mode for F_n = m·g₀);  σ2 = 0
```

The linear-rate blend makes ż linear in z across the transition (it admits the exact per-step update `friction::bristle_advance`). Presliding length z_ss = 10 µm on the 50 µs tiers, 50 µm on Travel1d (core design §C.5). Below the breakaway ratio r_ba·μ_s the contact never slips, so floor vibration does not walk a landed body (no ratchet; `rs:plant/contact::bristle_no_ratchet`, < 0.05 mm in 60 s where a tanh-Coulomb control creeps > 1 mm). **Split update (Wave 2).** The tiers hold every bristle state fixed through the RK4 body step (the friction force uses the frozen z and the stage velocity) and then advance it exactly with `friction::bristle_advance` over the step's slide distance (`body::advance_bristles`). The rate law is autonomous in the slide distance σ and piecewise linear, so each regime has a closed form: ζ += σ while elastic, ζ = z_ss − (z_ss − ζ₀)·e^(−σ/(z_ss − z_ba)) on the transition, ζ = z_ss + (ζ₀ − z_ss)·e^(−σ/z_ss) above z_ss. This is unconditionally stable and keeps stick exact (no ratchet); an RK4-embedded z, whose transition rate |v|/(z_ss·(1 − r_ba)) exceeds RK4's real-axis limit above ≈ 0.04 m/s at 1 ms, diverged when a landed body slid fast under floor shaking. The parameters are still capped by `capped_bristle` (z_ss ≥ 2g₀·μ_c·(h/0.2)², 2g₀·σ1 ≤ 1/h).

**Parking lock and seismic latch.** The lock is a bilateral spring-damper about the engagement position (spring-applied, power-to-release). The seismic x-latch (`landing.seismic_latch`, off in every preset) is an elasto-plastic element: stiffness 1e6 N/m up to the retention force, beyond which it slips.

**Manual force after landing** (SAF-010, core design F8): PM-relieved μ·(W − n·F_pm(g_land)) and the no-PM bound μ·W at rated mass, μ = 0.003: DRAWER 1.218 / 1.412 N, DOOR 0.695 / 2.942 N, WALL 3.283 / 14.71 N, HEAVY 32.83 / 147.1 N (HEAVY is not an egress device).

**Verified by** `rs:plant/contact::{never_pulls, normal_laws_and_bound_closed_forms, drop_onto_rollers_never_pulls_and_settles_within_budget, stiffness_caps_and_static_penetration_budget, bristle_no_ratchet, floor_vibration_below_breakaway_does_not_walk_a_landed_body, sliding_force, bumper_geometry_force_and_log, seat_lock_and_latch_laws}`, `rs:plant/friction::*`, `py:params::bumper_energy_rating` (SAF-014).

## §9 Dissipation ledger and μ_eq

**Implemented in** `crates/fabrail-plant/src/dissipation.rs`; the ledger quadratures ride in the RK4 state of every tier, so they close to integration accuracy.

Each component is a signed force along +x opposing the travel velocity v:

```text
F_dist  = −b_dist·v                                      distributed eddy damping
F_ez    = −c(x)·v,   c(x) = c_max·smoothstep(depth/L_ez)  progressive end zones beyond the nominal travel
F_eddy  = −c_rail·r·v                                    rail eddy drag
F_hyst  = −F_hyst·r·tanh(v/1 mm/s)                        rail hysteresis drag (stateless sign)
F_air   = −½ρ·(C_D·A_front + C_f·2·A_leaf)·|v|·v          C_D = 1.05, C_f = 0.01
r       = ΣΦ²/ΣΦ₀²                                        flux ratio over the lift elements (1 levitated at g*)
```

smoothstep(s) = 3s² − 2s³ on s ∈ [0, 1] (C¹); the end zones run in the linear eddy regime (v ≪ v_c ≈ 9 m/s). The DRAWER front area is the cavity piston area. c_rail and F_hyst are \[UNCERTAIN\] truth values swept by the `drag_ledger` study (plant `Mismatch` overrides). The controller's viscous feed-forward uses b̂ = b_dist + c_rail (§16).

| | DRAWER | DOOR | WALL | HEAVY |
|---|---|---|---|---|
| b_dist / c_rail / F_hyst | 2 / 1 N·s/m / 0.2 N | 5 / 5 / 1 N | 20 / 20 / 5 N | 100 / 100 / 50 N |
| End-zone c_max (150 mm) | 100 N·s/m | 300 | 1500 | 8000 |
| μ_eq, closed form at 0.1 m/s, m* | 1.96e-3 | 2.04e-3 | 1.84e-3 | 1.43e-3 |

**Equivalent friction coefficient.** μ_eq = E_drag/(m·g₀·d) over a traverse, with E_drag the distributed, end-zone, rail, air and friction ledger terms; the levitated closed form at constant speed is ((b_dist + c_rail)·v + F_hyst + air)/(m·g₀). "Zero friction" is a direction: μ_eq ≈ 1e-3–1e-2, never 0 (FAB-HAP-010, S). The Travel1d coast of `py:physics_anchors::equivalent_friction_band` reports the simulated value next to the closed form; on DRAWER it also carries the cabinet-air piston force (§11).

**Verified by** `rs:plant/dissipation::{mu_eq_definition, components_oppose_motion_and_scale_with_flux, end_zones_are_progressive_and_outside_the_travel, mu_eq_band_at_cap_speed, ledger_accumulates_dissipation}`, `py:physics_anchors::equivalent_friction_band`.

## §10 Rail geometry

**Implemented in** `crates/fabrail-plant/src/rail.rs` (interpolation, engagement) and `python/fabrail/rail.py` (profile synthesis).

**Profile** (`fabrail.rail.irregularity_profile`; FAB-LEV-010), a space series on a 1 mm grid with channels z (up +) and y:

- sag half-sines between supports every `span_m`: −δ·sin(π·s/L), δ = L/`deflection_ratio` = L/1150 (the ASTM E557 partition figure [R20]);
- joint steps every `joint_spacing_m`: a bounded walk of steps U(⅓, 1)·`joint_step_m` (0.1–0.3 mm for 0.3 mm) that turns back once the offset passes ±½·`joint_step_m`, each smoothed over 1 mm with a C¹ smoothstep;
- band-limited roughness, flat wavenumber spectrum over 5–500 mm wavelengths, random phases, scaled numerically to exactly `roughness_rms_m` = 20 µm;
- lateral misalignment steps (`lateral_step_m` = 0.1 mm) at the joints on y.

**Interpolation.** Cubic Hermite on the uniform grid with Catmull–Rom node slopes (central differences, one-sided at the ends): value and slope continuous (C¹), quadratics reproduced exactly inside the grid; outside the grid the edge value is held with zero slope. The element gap and lateral offset use the rail deviation at the element's own x, and the gap rate includes the slope term (dz/dx)·v_x.

**Engagement e(x).** An element sees the fraction of its face under rail iron, with each rail end softened over ℓ_f by a linear iron-presence ramp integrated over the face (C¹ in x). The rail extent is [x_closed − L/2, x_closed + L/2] for a cabinet rail (DRAWER: not top-hung and travel ≤ body length L), else [x_closed − L, x_closed + travel + L]. As implemented the tiers call `rail::engagement`, which takes the face as ℓ_f = 2·g*(DRAWER) = 5 mm long and softened over the same 5 mm; `rail::engagement_for` (pole length l, ℓ_f = 2·g*) exists but is not wired (§26). The extension moment and the rail-end pull-back of a DRAWER emerge from e(x) and the engagement gradient force (§2). The closed-form extension envelope (`derive::extension_capacity`, FAB-LEV-016) shares the static weight between the front and rear lift groups and finds the largest extension s with F_k ≤ e_k(s)·F_max: 52.4 % at 48 kg on the preset stations, so the 0.30 m preset travel is not saturated; 75 % is.

**Flange rule** (FAB-LEV-015): t ≥ max(B_g, B_pm(g*))·w/B_sat,rail.

**Verified by** `rs:plant/rail::{profile_validation, hermite_reproduces_quadratics_and_is_c1, overlap_is_c1_monotone_and_bounded, drawer_engagement_over_the_preset_travel}`, `py:ground_motion::rail_irregularity_profile`, `rs:params/derive::extension_capacity_closed_form`, `py:params::rail_saturation_rule`.

## §11 Air and pressure disturbances

**Implemented in** `crates/fabrail-plant/src/air.rs` and `derive::piston_force`.

**DRAWER cavity** (A_p = 0.06 m², V₀ = 0.03 m³, A_leak = 40 cm²): adiabatic compression plus orifice outflow, positive Δp pushing +x (cabinet pressure pushes outward):

```text
dΔp/dt = (γ·(P₀ + Δp)/V(x))·(−A_p·ẋ − Q(Δp)),   V(x) = max(V₀ + A_p·x_open, 0.05·V₀)
Q(Δp)  = C_d·A_leak·√(2/ρ)·Δp/(Δp² + P_ε²)^¼       sharp-edged orifice, regularised at P_ε = 1 Pa
F      = Δp·A_p
γ = 1.4, P₀ = 101 325 Pa, C_d = 0.6 (ISO 5167-2), ρ = 1.204 kg/m³
```

A sealed cavity is an air spring γ·P₀·A_p²/V₀ = 17.0 kN/m; with the 40 cm² reveal the quasi-static piston force is ½ρ·(v·A_p/(C_d·A_leak))²·A_p = 5.64 N at 0.5 m/s (FAB-ENV-003, 5.6 N ±5 %). Near Δp = 0 the leak makes the state stiff: |∂(dΔp/dt)/∂Δp| ≈ 1.5e4 s⁻¹, λh = 0.73 at 50 µs (stable) but ≈ 15 at 1 ms (§26 for the Travel1d consequence).

**DOOR smoke-control pressure**: F = Δp·A_leaf (`air.leaf_area_m2` = 2 m²). **Lean and pressure profiles** are symmetric trapezoids (plan §2b item 11: lean rise ≥ 1 s, smoke ramp ≥ 2 s; §22). On DRAWER a `smoke_dp` event acts on the cavity area.

**Verified by** `rs:plant/air::{sealed_spring_rate, pressure_forces, signs_regularisation_and_stiffness}`, `py:physics_anchors::piston_pressure`.

## §12 Thermal

**Implemented in** `crates/fabrail-plant/src/thermal.rs`. One RC node per coil plus one lumped magnet node, ambient T_a a scenario input:

```text
C_c·Ṫ_k = P_k − (T_k − T_a)/R_c − (T_k − T_m)/(n·R_m)
C_m·Ṫ_m = Σ_k (T_k − T_m)/(n·R_m) − (T_m − T_a)/R_m
P_k     = R(T_k)·i_k²
```

The coil-to-magnet path reuses R_m (the magnets sit between the coils and the frame; with equal paths a steady coil rise ΔT_c lifts the magnets by ΔT_c/2). Values \[UNCERTAIN\] (DOOR C_th 150 J/K, R_th 5 K/W per coil; magnet 300 J/K, 3 K/W). Tempcos: B_r(T) = B_r,20·(1 − 0.0012·(T − 20)), R(T) = R_20·(1 + 0.00393·(T − 20)). The magnet temperature feeds M_pm (§2) and the motional EMF through ∂Φ/∂T·Ṫ_m (§4). The static zero-power gap shift for +40 K is Δg = (2g* + h′)·α_Br·ΔT/2 = −0.195 / −0.416 / −0.891 mm; zero-power control absorbs it by floating closer (FAB-LEV-014). Limits: coil 120 °C, magnet 80 °C (OverTemp, §18).

**Verified by** `rs:plant/thermal::{energy_balance_of_the_rates, integrates_to_the_steady_state_and_halves_the_magnet_rise, tempcos}`, `py:physics_anchors::thermal_gap_shift`; the dynamic shift by `py:levitation::thermal_drift_absorbed` on Heave1 with a compressed quasi-static ramp (Wave 2).

## §13 Sensors and quantisation

**Implemented in** `crates/fabrail-sim/src/{sensors.rs, simboard.rs, faults.rs}` (truth → raw codes) and `crates/fabrail-core/src/frontend.rs` (raw codes → SI). Every conversion from f64 truth to an integer code happens in the sim harness; the controllers and the monitor see only raw codes.

| Sensor | Model (SimBoard) | Default |
|---|---|---|
| Gap (primary and redundant probe per element) | truth → first-order anti-alias (τ 20 µs) → + bias (+ tempco drift) → + white noise σ = `gap_noise_m` (added after the filter, so the rms equals the parameter) → inverse linearisation polynomial → 16-bit code | σ 0.3 / 0.5 / 2 / 5 µm; 16 bit over 20 mm; bias and tempco 0 (no scenario field) |
| Coil current | + white noise σ = `coil_noise_a` → 12-bit signed code | 5 / 5 / 10 / 20 mA |
| LSM phase currents | + white noise σ = 1 LSB → 12-bit signed code | |
| Encoder | floor(x/resolution) counts, no noise | 1 µm/count |
| Carriage IMU | inertial acceleration + tilt leakage g·θ + white noise → 16-bit code | 1 mg rms |
| Rail floor accelerometer | floor acceleration + white noise → 16-bit code | 1 mg rms |
| Structure ADXL355 | floor + site + white noise at the data-sheet density over the Nyquist band → 20-bit code (±2.048 g clip), FIFO of 32 at the ODR | 22.5 µg/√Hz, 500 Hz ODR |
| Handle load cell (optional) | + white noise | 0.05 N |

- **Gravity.** Accelerometer codes carry dynamic acceleration only: the static 1 g of the vertical axis is taken as removed by the BSP's commissioning offset, so the seismic front end (which computes PGA from the anti-aliased signal) receives gravity-free codes. The carriage IMU keeps the tilt leakage g·θ (1 mg per mrad), the one gravity effect calibration cannot remove.
- **Latency.** One base tick of transport latency (`sensors.latency_ticks` = 1, a delay line on the sensed state) plus the ZOH of the actuation latch: exactly the delay `params::derive` designs the current and gap loops for. Commands and alerts reach the links after the CAN latency `link_latency_s` = 3 ms.
- **Noise streams.** Each source draws from its own named substream per (noise seed, tag, device, channel) and only when the HAL samples it (§24).
- **Front end** (`core::frontend`): gap g = c₀ + c₁·code + c₂·code² per probe; a code on the ADC rail or a channel without its valid bit is out of range; two probes agreeing within max(50 µm, 10σ) are averaged, disagreement raises GapDisagree (Degrade), both out raises GapRange (SafeLand). Position `PosNm(counts × nm_per_count)` in saturating integers; a per-release jump beyond 5 m/s gives EncoderJump. LSM phase codes go through Clarke/Park at the exact electrical angle (§7). A stuck gap (codes frozen while the current moves > 0.5 A for 20 ms) raises GapStuck, armed only while levitated.
- **Fault injection** (`faults.rs`): coil or LSM driver off, stuck or lost gap channel, encoder jump or loss, accelerometer stuck or lost, IMU or rail accelerometer lost, and control loss (coil drivers stuck at full output); core-output faults go through `fabrail-rt`'s `fault-injection` feature.

**Verified by** `rs:sim/sensors::{gap_code_inverts_the_linearisation, current_and_accel_codes_saturate, adxl355_noise_density_is_preserved, gap_filter_is_first_order_with_the_stated_tau, fifo_overrun_drops_oldest_and_flags}`, `rs:sim/simboard::sensors_quantise_truth_with_one_tick_latency`, `rs:core/frontend::{dual_gap_vote, encoder_and_sensor_faults, stuck_gap_only_when_armed, lsm_dq_from_phase_codes}`.

## §14 Power, hold-up, regeneration

**Implemented in** `crates/fabrail-plant/src/power.rs` (truth), `crates/fabrail-core/src/supervisor/power.rs` (supply monitor), `crates/fabrail-safety/src/supply.rs` (monitor floor), `derive::holdup_energy`.

**Bus** (quasi-static). `p_load` is the whole bus load: coil and LSM electrical input plus the standby electronics `p_electronics_w`.

- Mains present: the supply holds V_bus up to P_max; beyond it the bus sags like a constant-resistance load, V = V_bus·√(P_max/P). The supply cannot sink power: net regeneration first recharges the supercap (up to the charger rating `CHARGER_FRAC`·P_max = 0.1·P_max) and the rest is burned in the brake chopper, which clamps the bus at V_chopper = 1.15·V_bus (FAB-PRP-010: ≤ 16 % regen rise).
- Mains lost: the hold-up converter (η = 0.9) holds V_bus from the supercap while V_hold > V_min. The cell current solves P/η = I·(V − I·ESR): I = (V − √(V² − 4·ESR·P/η))/(2·ESR); above the maximum-power point V²/(4·ESR) the bus sags. At V_hold ≤ V_min the converter drops out and the bus collapses to 0 V.

**Bus sizing (W1.5).** `power.p_bus_max_w` is at least n·i_pulse²·R_coil so a lift or unseat pulse does not sag the bus below the BusUnder threshold (0.8·V_bus): DRAWER 4·12²·1.6 = 922 W → 1000 W; DOOR 680 → 800 W; WALL 1120 → 1200 W; HEAVY 7680 → 8000 W. The earlier 120–5000 W values sagged the bus in every unseat or lift pulse (BusUnder → SafeLand).

**Hold-up** (SAF-009). Usable energy above the converter cut-out, η·½·C·(V² − V_min²): 10 F at 12 V down to 6 V gives 486 J (HEAVY: 10 F at 48 V down to 24 V, 7776 J). The landing need (`derive::holdup_energy`) is the copper energy of a controlled quasi-static transfer from the float gap to the rest gap at the land speed, n·R·∫i(g)²dg/v_land (Simpson, 64 panels; seat-first for SEAT classes), plus the electronics over stop, transfer and seat times, at the worse of m_rated and m_op_min. Need: DRAWER 111.3 J, DOOR 7.9 J, WALL 52.2 J, HEAVY 299.0 J; every preset meets the 3× margin. The monitor's floor is the voltage equivalent of 3× the need, rounded up to 1 mV:

```text
V_req = √(V_min² + 2·3·E_need/(η·C))       DRAWER 10.498 V, DOOR 6.425 V, WALL 8.415 V, HEAVY 27.846 V
```

(`safety.v_hold_min_v`, sealed; W1.5 replaced the converter cut-out V_min as the floor). DRAWER open hover is supercap-limited: 486 J at 1.5 W is about 5.4 min (≈ 5 min, FAB-SAF-009).

**Cycle energy report** (FAB-PWR-006): kinetic, coil and LSM copper, electrical input, electronics and every ledger term of §9 per run.

**Verified by** `rs:plant/power::{supercap_energy_closed_form, mains_bus_limit_and_chopper, holdup_discharge_energy_accounting, esr_limits_the_holdup_power}`, `rs:core/supervisor/power::{usable_energy_closed_form, presets_meet_holdup_margin, seat_first_energy_only_for_seat, supply_faults}`, `rs:safety/supply::holdup_energy`, `rs:params/derive::passivity_and_holdup`.

## §15 Levitation control (including seat and unseat)

**Implemented in** `crates/fabrail-core/src/levitation/{mod.rs, design.rs, modal.rs, observer.rs, zeropower.rs, liftland.rs, seat.rs}`, gains from `derive::zp_gains_discrete`.

**Zero-power law** (Mizuno current integral [R29], [R30]), per controlled mode (heave and pitch in every class; roll on DRAWER and HEAVY; `levitation.roll_active`):

```text
u[k]   = k_p·δg[k] + k_d·δġ[k] + κ·q[k]          modal current command
q[k+1] = q[k] + T·(i[k] − i_bias)                  integral of the MEASURED modal current
```

At equilibrium the current returns to i_bias (0 inside the band) and the gap floats to the zero-power gap of the load (§3). Because q integrates the actual current, the law has no windup when the reference is clamped.

**Discrete gain design** (plan §2b item 1; `derive::zp_gains_discrete`). Six-state sampled loop x = [δg, δġ, i, i_e, q, u_d]: the ZOH plant (modal model of §3, b = k_i/m_eff, p² = k_eff/m_eff), the current behind the closed-loop lag τ_cl = 0.159 ms, the force flux behind the nominal eddy lag τ_e = `tau_e_nominal_s` = 1.0 ms, the Mizuno integral and one fast period of LET delay on the command. The characteristic polynomial χ(z) = det(zI − M(K)) is affine in K = (k_p, k_d, κ): expanding along the command row, χ(z) = z·C₅₅ − k_p·C₅₀ − k_d·C₅₁ − κ·C₅₄ with K-free cofactors. Requiring χ = 0 at the complex target pole (real and imaginary parts) and at the real one gives three linear equations for the gains. Targets: the roots of (s² + 2ζωs + ω²)(s + αω) mapped by z = e^(sT), with

```text
ω = omega_ratio·max(p_mode, p_heave) = 2.5·max(p_mode, p_heave),  ζ = 0.7,  α = 0.3
T = fast_decim/base_hz = 100 µs (DRAWER, DOOR) or 200 µs (WALL, HEAVY)
```

The remaining three poles (current loop, eddy lag, delay) stay where the plant puts them (`derive::zp_closed_loop_poles`). The continuous reference `derive::zp_gains` (κ = αω³/p², k_d = ((α + 2ζ)ω + κ)/b, k_p = ((1 + 2ζα)ω² + p²)/b) is used only for anchors and the noise gain.

| Heave, per element | DRAWER | DOOR | WALL | HEAVY |
|---|---|---|---|---|
| ω | 173.7 rad/s | 118.9 | 81.24 | 81.24 |
| k_p / k_d / κ (discrete) | 21 828 A/m / 293.0 A·s/m / 155.5 s⁻¹ | 19 186 / 371.5 / 136.3 | 23 525 / 663.1 / 107.7 | 35 287 / 994.7 / 107.7 |
| Noise force gain K = m_eff·((1 + 2ζα)ω² + p²) | 0.310 N/µm | 0.559 N/µm | 1.304 N/µm | 6.518 N/µm |
| Critical τ_e, derive state feedback | 3.05 ms | 3.87 ms | 5.14 ms | 5.14 ms |

**Robustness to eddy lag** (FAB-LEV-020). The implemented f32 loops (observer at 4ω with the heave disturbance state) stay stable for τ_e ≤ 2 ms in every class; the bisection in `levitation_linear` gives critical τ_e of DRAWER heave / pitch / roll 2.47 / 2.24 / 2.44 ms, DOOR heave / pitch 2.99 / 2.36 ms, WALL 3.82 / 3.42 ms, HEAVY heave / pitch / roll 3.82 / 4.49 / 2.62 ms. A solid-steel rail's eddy lag can exceed 2 ms; the design assumes a laminated or SMC rail (`rail.laminated`).

**Coupled roll–sway design (HEAVY roll; Wave 2 decision D1; `derive::roll_sway_design`).** The roll loop feeds back the modal roll gap, its rate, the CoM sway, its rate and the current integral,

```text
u = k_u·û + k_u̇·û̇ + k_Y·Ŷ + k_Ẏ·Ŷ̇ + κ·q,   q̇ = i        (the coil current still returns to zero at equilibrium)
```

and places five poles of the eight-state discrete loop [u, u̇, Y, Ẏ, i, i_e, q, u_d] (the ZOH roll–sway plant of §6 with τ_cl and τ_e,nom, the Mizuno integral, one LET period of delay): the roll pair (ω, ζ) of the single-DOF design, the zero-power pole at `ROLL_SWAY_ALPHA`·ω = 0.1·ω (0.3·ω would sit on top of the 15–25 rad/s roll–sway band), and a sway pair at the passive sway frequency ω_y = √(n·k_y/m*) with ζ_y = 0.6 (keep the frequency, add damping with small sway gains). The characteristic polynomial is affine in the gains, so five linear equations fix them. The observer is a current-estimator Luenberger observer of the four mechanical states from the gap alone (measured current and known command drive the model, the eddy state is propagated), with the roll pair at `observer_ratio`·ω and the sway pair at 0.7·ω_y, ζ 0.7 (a faster sway observer loses stability near τ_e = 0). The design stays stable on the coupled plant across τ_e 0–2 ms, k_y ±25 %, c_sway 0–3× and CoM height ±10 % (`rs:params/derive::roll_sway_coupled_design`, `levitation_linear`).

**Observer** (current-estimator form; Franklin, Powell & Workman §8.2). The design plant [δg, δġ, i, i_e] is propagated exactly with its ZOH model (Φ, Γ) from the corrected mechanical estimate, the measured coil current i[k] and the known LET-delayed command u_d[k]; only the mechanical states are corrected from the gap, with the error poles placed at ω_o = `observer_ratio`·ω = 4ω, ζ_o = ζ. With an exact model the estimation error is autonomous, so the closed loop keeps the discrete design poles (separation principle; LEV-007 is tested on the f32 code). Φ is stored as Φ − I so the O((pT)²) mechanical terms survive f32 rounding. The heave observer carries a constant-acceleration disturbance state (Γ_d = [(cosh pT − 1)/p², sinh(pT)/p]) at `OBS_DIST_RATIO`·ω_o = 0.25·ω_o: it absorbs the linear model's offset away from (g*, m*) (a heavier payload at its own zero-power gap, the seat hold, the band edge), so the estimates are unbiased at every equilibrium. An optional IMU-aided update replaces the model acceleration by a_rel = a_rail,z − a_imu,z (off by default).

**Modal allocation** (`modal.rs`). Lever arms about the CoM over the lift elements; r̄ = √(Σr²/n) per mode. Current/force map T: heave row 1/n, pitch row r_x,k/(n·r̄_x), roll row r_y,k/(n·r̄_y); A = W⁻¹Tᵀ(TW⁻¹Tᵀ)⁻¹ gives T·A = I with the minimum weighted current norm (FAB-LEV-009). Gaps are decomposed by the rigid-body least-squares fit δg_k = h + r_x,k·θ_p + r_y,k·θ_r over the valid lift elements (an invalid channel is replaced by the rigid-body prediction). The warp loop integrates the null-space part of the measured currents to zero at ω_w = 2π·2 Hz (≤ 5 % in 1 s). Pure-EMS hold-down elements are served by a one-sided sub-allocator.

**Modified zero-power band** (core design F1). Outside `zp_band_m` (DRAWER 1.5–6.0 mm) a one-sided outer PI sets i_bias so the gap is held at the band edge: K_I = ω_o·k_s/k_i (ω_o = 2π·1 Hz, well below the integral pole αω ≥ 24 rad/s), K_P = 0.25·k_s/k_i; it keeps regulating until its bias has bled back to zero, so the steady bias equals `derive::bias_current(g_edge, m)` (+0.39 A at 48 kg; FAB-LEV-018).

**Payload estimate** (non-safety; FAB-LEV-013). m̂ follows the quasi-static closed form m = n·F(g, ī)/g₀ through a first-order filter (τ = `m_hat_tau_s` = 0.2 s), clamped to [½·m_op_min, 2·m_rated]; at lift-off it is set from the lift-off current. The governor uses max(m_rated, 1.05·m̂) and the monitor uses rated mass only (§16, §18).

**Lift, land, touchdown** (`liftland.rs`).
- *Lift from rest*: the heave current ramps open-loop until lift-off is sensed (the gap, low-passed at 400 Hz, leaves its rest value by > `unseat_liftoff_m` = 20 µm). m̂ = n·F(g_rest, i_lo)/g₀ with i_lo corrected for the detection delay (early motion is cubic in time under a current ramp, so the delay is ≈ 3·Δg/ġ) and for τ_cl + τ_e. Reaching 0.98·i_pulse without lift-off aborts (LiftTimeout) and ramps down (a DRAWER above ≈ 20 kg on its 6.5 mm rollers, F17). Then a C² quintic gap trajectory (peak acceleration 0.1·g₀, at least 0.02·g₀) moves the gap to the band-clamped zero-power gap g_zp(m̂) under the gap servo, and the loop switches bumplessly to zero power when |g − g_zp| < 50 µm (`bumpless_dg_m`), with the ZP integral preset so the demand is continuous.
- *Gap servo*: the continuous identity between the Mizuno law and a gap PID, i* = k_p·δg + (k_d − κ/b)·δġ + (κ·p²/b)·∫δg, with the force feed-forward i_ff = i(g_ref, m̂·(g₀ − g̈_ref)/n); pitch and roll servo the level attitude while heave servos the gap.
- *Land*: the ZP integral is frozen, the gap reference ramps at the land speed (5 / 5 / 3 / 2 mm/s) to g_land + 0.2 mm, touchdown is declared when the reference is past g_land, the gap sits within 0.1 mm of the rollers and |ġ| < 1 mm/s for 20 ms; the current then ramps to zero over 20 ms.
- Lift times ≤ 0.1 / 0.3 / 1 / 2 s (FAB-LEV-012).

**DRAWER seat and unseat** (F17; plan §2b item 4; `seat.rs`). Unseat ramps the current at `unseat_ramp_a_per_s` = 200 A/s until lift-off; the direction is read from the rest face (up from the lower face, down, possibly negative, from the upper face). After lift-off the gap servo holds the clearance middle g_seat − c/2 (`InSlot`); when the encoder shows x − x_closed ≥ `slot_len_m` (20 mm) the gap moves to g_zp and zero power takes over (`Clear`). The z-seat step moves the gap to the clearance middle in 0.75·`seat_time_s` (0.2 s), then ramps the current to zero; the drawer settles on the face its weight selects (`Seated`), or on the travel rollers if the seat was missed (`Failed`, SeatFailed; manual push-to-seat). Fail-down re-lift from the rollers works only up to ≈ 20 kg (i_pulse 12 A; FAB-LEV-019).

**Seat approach (W1.5).** A drawer floating at g* is above the captive lip and cannot enter the slot. While a `Clear` drawer closes toward the slot at zero power (v < −`APPROACH_V_MIN_MPS` = −0.02 m/s) it descends to the clearance middle in 0.75·seat_time once x_open is inside slot_len + `APPROACH_MARGIN`·|v|·t_move + `APPROACH_EXTRA_M` (1.5 and 5 mm) and holds it under the gap servo (`InSlot`); the z-seat step on arrival then only releases the current. Turning back before the slot ends the approach.

**Slot exit at rated mass (Wave 2 decision D2).** The I²t budget bounds unseat plus slot exit. At 48 kg the seat hold needs ≈ 10.5–11.2 A per element against i_cont = 4 A, so the budget B = 25.6 A²s (core clamp at 0.95·B = 24.3 A²s) is spent at ≈ 109 A²/s: the 200 A/s unseat reaches InSlot at ≈ 69 ms, and a slot exit from rest at a_max = 1.5 m/s² reaches Clear at ≈ 232 ms using ≈ 24.0 A²s (≈ 215 ms and 22 A²s at F_peak = 90 N), about 1 % margin. In the W1.5 closed loop the rear coils reached i_pulse during the drive-out, the rear dropped onto the seat face and the run ended in Obstruction plus SaturationPersist → SS1; 26 kg is clean. **Wave 2 decision D2:** try a staged drive-out within the I²t budget; if it is infeasible, the honest rated mass for slot exit is reduced and stated. *Outcome: pending Wave 2; the capability matrix carries a TODO row with the number to be filled by Wave 2.5.*

**Lifted but not moved (Wave 2 decision D7).** As implemented, a DRAWER unseated without a move holds InSlot at about 3.7 A per element (≈ 87 W at 26 kg; ≈ 10.5 A at 48 kg) until the hover timeout. **Wave 2 decision D7:** the supervisor re-seats after 2 s idle in the slot, or drives out if a move is pending. *Code change pending Wave 2.*

**Thrust-moment feed-forward (W1.5).** The pitch modal current

```text
i_p,ff = h·F_x/(n·k_i·r̄_x),   h = −c_z
```

cancels the LSM torque (a modal current i_p produces the torque −n·k_i·r̄_x·i_p about +y). It is added to the references after the I²t clamp and subtracted, lagged by the current loop, from the measured modal current at the next release, so the observers and the zero-power integral see only net torque. A 20 ms plausibility gate (`FF_TRACK_S`) compares the measured and commanded pitch currents; a mismatch above 50 % of the commanded magnitude (above 0.05 A) disables the feed-forward until the coils are next de-energised. The gate is needed because Travel1d does not simulate coil current: without it the phantom feed-forward wound the zero-power integral up to SaturationPersist.

**Faults raised here**: GapEnvelope, LiftTimeout, LandTimeout, SaturationPersist (a reference at the I²t clamp for `sat_persist_s` = 50 ms), SeatFailed.

**Verified by** `rs:params/derive::{zp_gains_match_polynomial, zp_discrete_places_target_poles, zp_discrete_stable_to_2ms, zp_stable_across_float_band, noise_force_gain_closed_form, roll_sway_coupled_design}`, `rs:core/tests/levitation_linear::{discrete_poles_match_design, stable_for_eddy_lag_to_2ms, stable_across_float_band}` (FAB-LEV-007, FAB-LEV-020), `rs:core/levitation/modal::{allocation_identity, nullspace_zero_net}` (FAB-LEV-009), `rs:core/levitation/zeropower::bias_current_closed_form` (FAB-LEV-018), `rs:core/levitation/seat::unseat_sequence` (FAB-LEV-017), `rs:core/levitation::{lift_transfer_and_land, zero_power_load_step, drawer_relift_envelope}`, `rs:sim/tests/sil_smoke::{door_hover_1s, drawer_unseat, wall_land}`.

## §16 Motion, governor, clamps

**Implemented in** `crates/fabrail-core/src/motion/{scurve.rs, cascade.rs, governor.rs, clamp.rs}`, `fast.rs`, `supervisor/modes.rs`.

**7-segment S-curve** (closed form from its anchor). A rest-to-rest move of distance D under (V, A, J) is +J, 0, −J, cruise, then the mirror. Per accelerating half: trapezoidal (v_p ≥ A²/J) a_p = A, t_j = A/J, t_c = v_p/A − A/J; triangular a_p = √(v_p·J), t_j = √(v_p/J), t_c = 0; half distance d_a(v_p) = v_p·t_a/2 with t_a = 2t_j + t_c. When 2·d_a(V) > D the cruise vanishes and v_p = (A/2)·(√(A²/J² + 4D/A) − A/J) for D ≥ 2A³/J², else (D²J/4)^⅓. The first half is evaluated from the anchor and the second backwards from the end point, so the reference lands on the end exactly (§24). Jerk-limited stop distance:

```text
d_stop(v) = v²/(2a) + v·a/(2j)      v ≥ a²/j
          = v·√(v/j)                 otherwise
```

At the caps: 33.42 / 36.32 / 8.861 / 4.906 mm (SPECIFICATION quotes 33.43 / 36.31 mm at the rounded caps).

**Cascade** (per fast release, period T):

```text
x_c    = x_enc − h·θ̂_y                                  CoM position (W1.5; §5)
v_cmd  = clamp(v_ref + K_x·(x_ref − x_c), −v_lim,close, +v_lim,open)
F      = K_v·(v_cmd − v̂) + I + m̂·a_ref·[not governed] + b̂·v_ref
I     += K_vi·(v_cmd − v̂)·T        (conditional integration at ±F_peak; frozen in terminus capture)
K_v = m̂·ω_v,  K_vi = K_v·ω_v·ki_ratio,  K_x = ω_v·kx_ratio,  ki_ratio = kx_ratio = 0.25
ω_v = 2π×{8, 6, 3, 2} Hz,  b̂ = b_dist + c_rail
```

The position error is an integer `PosNm` difference, cast afterwards. The acceleration feed-forward is dropped while the governor limits the command in the direction it would push. v̂ and â come from a third-order tracking observer on the encoder, poles (s + ω_t)³, ω_t = 2π·max(5·f_v, 2·f_KO). With the integrator frozen the tracking error obeys m·ë + K_v·ė + K_v·K_x·e = 0, i.e. (s + ω_v/2)², critically damped.

**Energy and momentum governor** (SAF-001, SAF-007; core design F19):

```text
m_gov  = max(m_rated, 1.05·m̂)            m_rated = max(design m_rated, safety.mass_rated_kg)
v_lim  = min(V_mode, √(2·0.90·E/m_gov), 0.95·p_max/m_gov, √(2·a_brk·(x_obs − x − margin))) − a_brk·t_react
a_brk  = 2·a_max(live),  t_react = 2 fast periods,  margin = 2 mm
```

E = 1.69 J (A156.19/A156.38 low energy) and p_max = 22.3 kg·m/s (A156.10; 0 = not applicable, HEAVY) are the monitor's caps; at rated mass they give the speed caps √(2E/m) and p/m: DRAWER 0.2654 m/s both ways, DOOR 0.1838 open / 0.152 close, WALL 0.0446 m/s, HEAVY 0.025 m/s hold-to-run. The governor plans inside them (Wave 2 decision D4, `GOV_KE_PLAN_FRAC` = 0.90, `GOV_P_PLAN_FRAC` = 0.95): 0.90 of the energy cap is √0.90 ≈ 0.949 of its speed, so tracking overshoot, rail-profile disturbances and hand assistance meet a 10 % energy (≈ 5 % speed) margin instead of the W1.5 ≈ 0.3 mm/s. At rated mass the governed caps are DRAWER 0.2517 m/s, DOOR 0.1744 m/s open (0.152 close), WALL 0.0424 m/s (momentum-bound) and HEAVY 0.0247 m/s, less a_brk·t_react (0.6 mm/s on DRAWER). After the reaction time at v_lim the carriage brakes at a_brk; the distance used is d − a_brk·t_react²/2 ≤ d, so it stops before x_obs − margin. A commanded terminus is observed at x_obs = target + overtravel (5 mm), so the exponential tail is never clipped (§16.1). m̂ can only raise m_gov, so sabotaging m̂ to 0.5·m changes nothing. The governor limits planner references through the cascade and the admittance velocity (where limiting the virtual velocity only removes energy: under hand-driven motion the governor adds damping only).

**Planning inside the envelope (W1.5).** The supervisor plans cruise at min(class cap, 0.98·v_gov), v_gov the governor's steady cap with m_gov (`GOV_PLAN_FRAC`, 2 % headroom for the position loop); planning at the m_rated cap had left a DOOR 18–30 mm behind its reference before the terminus. The terminus then blends out of the planned speed, not the cap.

**Governor damping (D4).** In the fast loop a drive force along the velocity is scaled by k(|v|) = 1 up to `GOV_DRIVE_FADE_FRAC`·v_cap = 0.975·v_cap and falls linearly to 0 at the full cap v_cap = min(√(2E/m_gov), p_max/m_gov) (`Governor::drive_scale`); braking forces are never scaled, and both the CoM speed and the encoder-point speed (the monitor's measurement, which includes the pitch rocking of a top-hung leaf) are checked. Above the caps the drive can therefore only remove energy, which is what the monitor's positive-work rule (§18) expects. The fade band sits above the 0.949 planning cap so that tracking and rail noise around a cruise never touch it (a fade from the planning cap chattered the DRAWER drive at 0.25 m/s on the rail traverse). Research-tier direct-force modes are exempt.

**Why D4.** In Wave 1.5 the governor planned at the exact cap and the monitor tripped at it with only ≈ 0.3 mm/s of margin (WALL throw_soft_close, WALL obstruction_chest, DRAWER and DOOR rail_traverse). Since D4 the governor plans at 0.90 / 0.95 of the caps and the monitor's cap checks require positive drive work (§18); excess from a hand throw or a rail disturbance is answered by governor damping only.

**Directional clamp** (PRP-004; numeric limits from EN 16005 (2011 draft) and A156.19, checked in simulation only):
- braking (|v| ≥ 2 mm/s and F·v < 0) passes unchanged: stopping is never limited;
- driving (F·v > 0, or |v| < 2 mm/s) is limited to F_drive = min(live F_drive, `safety.f_drive_max_n`) while a leaky duty counter lasts: every release above the static level consumes one tick of the 0.75 s allowance, every release at or below it returns one; once spent, driving is limited to F_s;
- static level F_s = 67 N unsensed, 150 N with PL d presence sensing, or 150 N for a closing force in the last 50 mm (measured from `LiveLimits::x_obs_close`);
- stall (|v| < 2 mm/s for > 0.2 s): the force is limited to F_s, and after 5 s to 0.95·80 N = 76 N (strictly below 80 N; the 5 % margin covers thrust-model error).
- Back-calculation anti-windup against the clamp (W1.5): a 5 t HEAVY breaking away at 67 N had wound the velocity integral up and overshot to Overspeed.

The clamp never adds force and never flips its sign; default `LiveLimits` (all zero) allow braking only. In ISOLATE the clamp is bypassed and the monitor's IsolateLimits replace it (SAF-020, §21).

**Clamp force and pinch speed** (SAF-005). A free mass with kinetic energy E stopped by a linear body spring k reaches F = √(2E·k) = v·√(m·k): at 1.69 J the chest (25 N/mm) takes 290.7 N, the hand (75 N/mm) 503.5 N, and the hand through the 10 N/mm edge seal (series 8.82 N/mm) 172.7 N; WALL at its 0.497 J cap against the chest 157.7 N. The unattended automatic-close speed (plan §4b) is v = min(v_cap, F_pinch/√(m_gov·k_series)) = 0.2305 m/s at 48 kg with 150 N and 8.82 N/mm. ISO/TS 15066 transient speed with reduced mass μ = m_h·m/(m_h + m): v = F/√(μ·k), 500 kg against the chest at 140 / 280 N → 0.1455 / 0.2910 m/s [R7], [R58].

**Verified by** `rs:core/tests/scurve_props::limits_and_stop_distance` (FAB-PRP-005), `rs:core/motion/governor::{rated_mass_invariance, braking_envelope, terminus_not_clipped, energy_uses_governor_mass}` (FAB-SAF-001, FAB-SAF-007, FAB-HAP-015), `rs:core/motion/clamp::{directional_clamp_props, dynamic_allowance_expires}` (FAB-PRP-004), `rs:core/motion/cascade::{gains_follow_design, frozen_integrator_is_bumpless, governor_limit_bounds_command}`, `rs:params/presets::speed_caps` (FAB-SAF-002), `py:physics_anchors::{d_stop_values, clamp_force_energy_form, iso15066_speeds}`.

### §16.1 Log-damped terminus

**Implemented in** `crates/fabrail-core/src/motion/terminus.rs`, the closed forms in `derive::{terminus_blend, terminus_t_arrive, tau_min}`, wired by `fast.rs` and `supervisor/modes.rs`.

The terminus is the reference generator of every commanded close and open (`MotionMode::Terminus`, `SoftOpen`). For approach speed v, jerk j, time constant τ and capture distance d_ε:

```text
1. blend:    t_b = √(τ² + 2v/j) − τ,   d_b = j·t_b·τ²,   d_0 = d_b + v·t_b − j·t_b³/6
             (at t_b: v − j·t_b²/2 = d_b/τ and j·t_b = d_b/τ², so speed and deceleration match)
2. capture:  d(t) = d_b·e^(−t/τ)        ln d is a straight line of slope −1/τ: the "log-damped" definition
3. quintic:  from (d_ε, −d_ε/τ, d_ε/τ²) to v = a = 0 in T_q = τ, monotone for T_q/τ ∈ [1, 6]
4. arrival:  t_arrive = t_b + τ·ln(d_b/d_ε) + T_q          (DRAWER: + 0.2 s z-seat step by the supervisor)
```

The reference is C² everywhere (cruise → blend → capture → quintic → rest; the jerk steps at each joint). If d_b ≤ d_ε the capture is skipped and the quintic starts at the blend end. Distances are distance-to-go d ≥ 0; x = target − dir·d, evaluated in closed form from the target and the start tick (never a running sum). Before its t0 a terminus extrapolates the cruise it blends out of, so a supervisor can hand over from a cruising S-curve at any release before t0 without a reference step. The quintic is evaluated in factored form (1 − σ)³(α + βσ + γσ²) so it lands exactly on the target and ḋ keeps its sign near the end.

**Terminus table** (d_ε = 0.2 mm):

| Case | v | j | τ | d_0 / d_b | t_arrive |
|---|---|---|---|---|---|
| DRAWER | 0.2654 m/s | 20 m/s³ | 0.15 s | 49.9 / 32.1 mm | 0.983 s (+ 0.2 s seat) |
| Cabinet slider | 0.152 | 5 | 0.20 | 40.0 / 23.5 mm | 1.27 s |
| DOOR close | 0.152 | 5 | 0.25 | 46.1 / 31.6 mm | 1.617 s |
| DOOR open | 0.1838 | 5 | 0.25 | 57.6 / 37.1 mm | 1.675 s |
| WALL | 0.0446 | 1.5 | 0.6 | 27.8 / 25.7 mm | 3.562 s |
| HEAVY | 0.025 | 1 | 1.0 | 25.3 / 24.7 mm | 5.841 s |

**τ_min rule** (FAB-HAP-014). τ ≥ τ_min = v_app/a_max − a_max/(2j), the τ at which the blend's peak deceleration j·t_b equals a_max (DRAWER 0.139 s, DOOR 0.170 s, WALL 0.247 s, HEAVY 0.273 s), enforced by `validate`; at runtime τ_eff = max(τ, τ_min(v_measured)), which covers hand-thrown drawers.

**Integrator freeze and held-integral scaling.** During blend and capture (`terminus::integrator_frozen`) the cascade's velocity integrator is frozen with bumpless hold; with K_v = m̂·ω_v and K_x = ω_v/4 the error poles are (s + ω_v/2)², ζ = 1, and ω_v/2 ≥ 3/τ holds in every preset (DRAWER 25 ≥ 20, DOOR 18.8 ≥ 12, WALL 9.4 ≥ 5, HEAVY 6.3 ≥ 3). It is re-enabled at d_ε. **W1.5:** the held integral is scaled by |v_ref|/|v_ref at the freeze| (≤ 1; unscaled when the freeze speed is below 1 mm/s). A constant hold kept pushing with the cruise drag and air force as the reference slowed and led the exponential by F_I/(K_v·K_x) ≈ 0.1 mm on a Carriage6 DRAWER, bending the semilog line near d_ε (R² 0.9990 → 0.99996 with the scaling). The scale depends on the reference only, so the error dynamics stay (s + ω_v/2)²; release is bumpless.

**Handover from cruise.** A move to a travel end is an S-curve cruise handed to the terminus at the tick t_h where the S-curve's distance-to-go equals d_0 (bisection on the closed-form S-curve); the terminus directive is published at the first supervisor release whose activation tick r + P ≥ t_h, with t0 = t_h; in the ≤ P window the fast group tracks the cruise, which differs from the blend by j·Δt³/6 ≤ 3.3 nm on DRAWER.

**Hand interaction and re-plans.** A hand during capture (onset ≤ 10 ms, 20 ms on WALL/HEAVY) switches control to plain class admittance (K_v = 0, passivity on) when the terminus config says the hand yields (NORMAL only; outside NORMAL a hand gives Hold). On release the terminus is re-planned from the measured (d, v, a): d ≥ d_0(v, τ) → cruise then the nominal terminus; d < d_0 → τ shortened by bisection on d_0(τ′) = d down to max(τ_min(v, a_brk, j), 2/ω_v) with a_brk = 2·a_max (this relaxes the nominal ω_v/2 ≥ 3/τ to ≥ 1/τ′ for re-plans only), else `Infeasible` and the fast loop falls back to core SoftClose; nearly at rest (|v| below the class soft-close final speed) → an S-curve launch to the fastest v_app whose launch plus d_0 fits, then the terminus; inside d_ε or receding → a stand-alone quintic searched near T_q for no overshoot and |d̈| ≤ a_brk. A terminus re-plan is C¹ at the re-plan instant (it cannot carry the measured acceleration); quintic re-plans are C². Obstruction always overrides the target (§17).

**Arrival check.** A terminus has arrived only when its reference is done (`terminus::done`). The metric compares the 20 µm crossing of the truth with the reference segment's own 20 µm crossing, because the quintic tail reaches 20 µm 3.8 % before the closed-form t_arrive even under exact tracking; `terminus_t_arrive_cf_s` still reports the closed form (W1.5).

**Closed-loop results (W1.5, Carriage6, voltage mode, quiet floor):** DRAWER close into the seat R² 0.99996, τ_fit 0.151 s, overshoot 0, v_arrive 0.94 mm/s, t_arrive error 0.78 %; DRAWER soft-open from the seat R² 0.99997, error 0.67 %; DOOR close R² 0.99989, τ 0.255 s, v 0.56 mm/s, error 1.1 %; DOOR open R² 0.99990, error 1.05 %. FAB-HAP-013 to 016 hold on Carriage6 devices only while |a_floor,x| < 0.01 g.

**Verified by** `rs:core/motion/terminus::{exponential_capture_exact, reference_is_c2, quintic_monotone, integrator_freeze_poles}`, `rs:core/tests/terminus::*` (t_arrive table ±2 %, C² at every tick, semilog slope, τ_min rule, offset invariance, hand re-plans, handover, 96-case proptest), `rs:params/derive::terminus_t_arrive_table`, `rs:params/validate::rejects_tau_below_tau_min`, `rs:sim/tests/terminus_closed_loop::{drawer_close_terminus_seats, drawer_open_terminus_from_seat, door_close_and_open_termini, door_hand_during_capture_replans}`.

### §16.2 Inertial smoothing

**Implemented in** `crates/fabrail-core/src/motion/inertial.rs`. Research tier: `control.motion.inertial.enable = false` in every preset, so admission reports `Disabled` and the step returns 0 N unless a study enables it. DRAWER only (FAB-HAP-017 to 019, status R).

**Physics.** Along x a levitated drawer is already a free mass, so smoothing means following only the low-frequency floor motion and staying inertial above a crossover f_c ≈ 1 Hz. The law, per fast release:

```text
F      = −k·x_rel − c_sky·v̂_abs + ĉ·v_rel + κ_ff·m̂·LP²(a_rail)
k      = m̂·ω_c²,   c_sky = 2ζ·m̂·ω_c,   ω_c = 2π·f_c,  ζ = 0.7
v̂_abs ← λ·v̂_abs + T·a_imu,  λ = e^(−2π·f_hp·T),  f_hp = 0.1 Hz        (leaky integral of the carriage IMU)
ĉ      = b_dist + c_rail       cancels the parasitic viscous rail drag that couples the floor in
```

For the ideal loop m·ẍ_abs = −k·(x_abs − x_floor) − c_sky·ẋ_abs, so |X_abs/X_floor| = T_sky(r, ζ) = 1/√((1 − r²)² + (2ζr)²), r = f/f_c (T_sky(2) = 0.244, T_sky(5) = 0.040 at ζ 0.7). Added residual terms: IMU noise (≈ 1 mg rms), g·θ_tilt (1 mg per mrad), parasitic force/m (F_hyst is left in the residual), latency 2πfL·a_HP. The rail feed-forward is off unless a caller sets it (it adds an ε_sf·a_ff term); the structure ADXL355 is used below 0.3 Hz only.

**Budgets and exit.** |F| ≤ F_cont (20 N DRAWER), bursts up to 90 N for ≤ 0.75 s recovering at the rate used. The smoother latches an exit before any limit the governor or monitor watches: stroke |x_rel| > standoff − margin, |v_rel| > 0.8·v_cap, or a burst exhausted with the demand above F_cont. Every exit, mains loss included, is a log-damped close to the seat with a_brk = 2·a_max (`exit_plan` → `TerminusPlanner::replan`), never an SS1 landing at the standoff.

**Admission to SmoothRideThrough** (first failing check reported): open by at least s_min = 25 mm + S_max,pred (seated drawers never unseat on an alert); v_rel,pred ≤ 0.8·v_cap; predicted stroke ∬HP(a_floor) ≤ standoff − margin; force budget m·a_LP + μ_pad·m·|a_y| + (b_dist + c_rail)·|v_rel| + F_hyst ≤ F_cont (bursts as above); a_z,pred ≤ 0.5·p²·(g_float − g_up) (a z-headroom check, no z-isolation claim); mains_ok or a building UPS; E_sc ≥ 3·E_land + E_seat.

**Stability.** The skyhook loop coupled to the zero-power pitch loop (11 states: thrust moment at the CoM height, IMU tilt and rotation pickup, both signs, m ∈ {m_op_min, m_rated}, m̂/m ∈ {0.8, 1, 1.25}, gain ×1 and ×2) is stable on DRAWER with 6 dB margin (ρ ≈ 0.999925) and on HEAVY; top-hung DOOR and WALL are stable at nominal gain but not at gain ×2 with m̂/m = 1.25, which is why the mode is offered on DRAWER only.

**Verified by** `rs:core/motion/inertial::{skyhook_discrete_stable, admission_rules, residual_follows_t_sky, force_budget_bursts_then_exits, exit_is_log_damped_close_to_seat, disabled_by_default_refuses_admission}`; the closed-loop transmissibility by `py:haptics::inertial_residual_transmissibility` (Wave 2, research tier).

## §17 Haptics

**Implemented in** `crates/fabrail-core/src/haptics/{observer.rs, admittance.rs, effects.rs, passivity.rs, intent.rs, obstruction.rs}`, composed in `fast.rs`.

**Momentum observer** (De Luca and Mattone generalised momentum, extended with a disturbance-rate state). With p = m̂·v, the actuator force F_act (the modelled thrust of the measured i_q, §7) and the drag b̂·v:

```text
e     = p − p̂
p̂'    = F_act − b̂·v + F̂ + l1·e
F̂'    = Ḟ̂ + l2·e
Ḟ̂'    = l3·e
l1 = 9·K_O, l2 = 23·K_O², l3 = 15·K_O³      error poles K_O·{1, 3, 5};  K_O = 2π×{20, 15, 8, 5} Hz
```

Forward Euler at the fast period (|pole|·T ≤ 0.07). The peak error on a force ramp of slope k is ≈ 0.33·k/K_O, so FAB-HAP-004's 10 % of peak holds for ramps longer than ≈ 3.3/K_O: 26 / 35 / 66 / 105 ms (the WALL and HEAVY ramp tests start at 75 and 120 ms). A lean (|F̂| > F_th with |v| < 2 mm/s for > 1 s) feeds a bias estimator that follows the raw F̂ with τ = 1 s and relaxes to 0 otherwise, always between 0 and the raw F̂, so a released lean never pushes the carriage back. The 0.5 Hz pressure high-pass is applied inside intent detection only (applied to F̂ it would bias admittance and cost ≈ 20 % on the ramp test).

**Hand detection (W1.5).** A hand is |F̂| > F_th + 0.25·|F_cmd| for hand_onset/2 (5 ms; 10 ms on WALL/HEAVY). The thrust-proportional allowance (`HAND_TH_THRUST_FRAC`) keeps a braking terminus from reading as a hand: a 35 N DRAWER terminus brake had produced a 4.6 N F̂ bias. During powered motion an m̂ error above ≈ F_th/a_max (DRAWER ≈ 3 kg at 1.5 m/s²) could still read as a hand.

**Admittance** M_v·ẍ + B_v·ẋ + K_v·(x − x₀) = F̂_h + F_eff, M_v = m̂/assist ratio (1.5 / 3 / 5 / 5), B_v = 10 / 40 / 450 / 2000 N·s/m. Per release, with F the net force:

```text
v ← (v + T·F/M_v)/(1 + T·B_v/M_v)        implicit damper (unconditionally dissipative in B_v)
v ← clamp(v, −v_lim,close, +v_lim,open)  governor: removes energy only
x ← x + v·T                              PosAccum (integer base + sub-nm fraction)
```

Rendered lightness does not reduce impact energy: the governor and monitor act on m_gov and rated mass.

**Effects** (through F_eff, conservative or dissipative): detent F = −F_d·sin(π·(x − x_k)/w) for |x − x_k| ≤ w (DRAWER 3 N / 2 mm, DOOR 8 N / 3 mm, WALL 20 N / 5 mm; one at mid travel, none on HEAVY), a potential well of depth 2F_d·w/π; soft-close spring (the looser HAP-007 bound kept for `MotionMode::SoftClose`) F = −M_v·(ω²e + 2ωė)·s(|e|), ω = 2π×{2, 1.5, 0.5, 0.5} Hz, capture 45 / 65 / 115 / 150 mm with a smoothstep taper over the outer half; a one-sided virtual wall −K·penetration with K ≤ K_max.

**Passivity** (Colgate and Schenkel [R50]; Hannaford and Ryu [R52]). A virtual spring K rendered with period T and delay T_d on a device with physical damping b is passive iff K < 2b/(T + 2T_d); the presets render K_v = 0.3·2b/T with T_d = T, below 2b/(3T) (`validate`). K_max = 13.3 / 33.3 / 66.7 / 333 kN/m (b = b_dist). The passivity observer and controller at the port:

```text
E ← E + f·v·T
if E − W < 0 and v ≠ 0:  α = min(−(E − W)/(T·v²), α_max),  f_pc = −α·v,  E ← E + α·v²·T
α_max = 5·b_dist,  W = ½M_v·v² + ½K_v·e² (the admittance's stored energy; Ryu et al. 2005 reference)
```

With W = 0 it is the classic controller, which let E dip to −2.2 mJ at 3·K_max on DRAWER; with the stored-energy reference the generated energy is removed as it appears (FAB-HAP-009: E_obs ≥ −1 mJ).

**Intent** (per fast release; counts integer releases). (1) High-pass f_hp = F̂ − LP(F̂) at 0.5 Hz (a 15 N lean rising over 1 s peaks at ≈ 4.6 N after it; a 50 N smoke ramp over 2 s at ≈ 8.0 N). (2) Onset when |f_hp| > F_th/4, direction and displacement latched. (3) Direction consistency: at the DRAWER seat with push-to-open the only consistent gesture is an inward push (−x, meaning open); an outward force there is cabinet pressure and is rejected; anywhere, a gesture whose carriage moves against the push by more than max(dx_th/4, 0.1 mm), or whose force reverses, is rejected. (4) Confirmation by displacement ≥ dx_th (1.5 / 2 / 3 mm; none on HEAVY) within the window (120 / 170 ms), or s·f_hp ≥ F_th (4.5 / 10 / 25 / 40 N) for t_th (50 / 80 / 120 / 150 ms) continuously, with F_th reached inside the window. Confirmed intent is held ≥ 20 ms. (5) A rejected or ended gesture re-arms only when |f_hp| < F_th/4 and |F̂| < F_th. (6) Disabled outside SeismicState NORMAL.

**Obstruction.** The momentum observer's F̂ is the external force. On a shaking floor the body carries −m·a_floor,x, so F̂ = F_ext − m·a_floor,x and the detector uses the contact force r = F̂ + m̂·a_floor,x (a_floor,x from the rail accelerometer; the plan's r = F̂ − m̂·a_floor is the same physics with the opposite sign convention). Threshold th = th0 + 0.1·m̂·|a_floor,x| (th0 = 12 / 30 / 60 / 120 N), persistence 20 / 30 / 35 / 35 ms continuous; above 2·th0 the detector reports `degraded` (ObstructionDegraded → SeatAndHold, no automatic motion). The response latched by the fast loop: STOP (brake at a_brk), then REVERSE 50 mm (DOOR/WALL) or release to 0 N through LSM STO (DRAWER/HEAVY), until the supervisor changes mode or segment. A non-finite residual counts as above the threshold. A hand that held a terminus capture back harder than th0 latches an obstruction on release (no re-close into a trapped hand; W1.5).

**Unattended pinch (Wave 2 decision D6).** In Wave 1.5 a choreographed close yielded to a trapped hand within ≈ 5 ms (peak 42 N ≤ 150 N, no re-close), but the Obstruction flag was raised only at hand release (≈ 150 ms). Since D6, during an unattended close (`LiveLimits::unattended`: choreographed or alert-initiated) a hand detected in a terminus capture is a pinch at once: the obstruction latches at the hand onset (≤ 10 ms, 20 ms on WALL/HEAVY), the carriage stops, then releases to 0 N (DRAWER/HEAVY) or reverses, and never re-closes. Attended closes keep the hand yield to admittance described above.

**Verified by** `rs:core/haptics/observer::ramp_tracking` (FAB-HAP-004), `rs:core/haptics/passivity::{stiffness_bound, pc_bounds_energy}` (FAB-HAP-009), `rs:core/haptics/intent::lean_rejected` (FAB-HAP-006), `rs:core/haptics/obstruction::residual_persistence` (FAB-SAF-006), `rs:core/haptics/admittance::{free_mass_follows_newton, governor_only_removes_energy}`, `rs:core/haptics/effects::detent_peak_and_shape`, `rs:core/fast::{obstruction_stops_then_reverses_or_releases, admittance_renders_virtual_mass}`; the closed-loop suites (apparent mass, breakaway, intent latency, false intent, soft-close, detents, push-to-open) in `tests/test_haptics.py` (Wave 2).

## §18 Supervisor and safety functions

**Implemented in** `crates/fabrail-core/src/supervisor/*` (1 kHz), `crates/fabrail-safety/src/*` (independent integer monitor), `crates/fabrail-rt/src/arbitrate.rs`.

**LevState machine** (pure `levstate::transition`): BOOT → SELFTEST (golden hash, parameter CRC, referenced) → rest state by `RestPolicy` (SEATED for DRAWER, LEVITATED_IDLE for DOOR, LANDED for WALL/HEAVY); SEATED/LANDED → LIFTING → LEVITATED_IDLE ⇄ MOVING → STOPPING → SEATING or LANDING → rest; any state → FAULT_REACTION (SS1 Safe Landing, STO on the LSM only) → FAULT (latched; only Reset clears it); side branches SEISMIC, ESCAPE, MANUAL. A fast status completes a state only when it was produced under a directive of that state (LET). Time limits are derived in `new()` with a 2× margin plus a fixed allowance (lift 2·t_lift + 0.5 s; landing 2·(g_rest − g_up)/v_land + 1 s; stop 2·v_max/a_brk + 0.5 s).

**Faults and reactions.** Severity ≥ Degrade latches until Reset, except the live conditions NotReferenced and HoldupLow; Warn/Info are events. The reaction demand is max(latched severity demand, monitor demand), so the core can never lower a monitor demand (SAF-016). PayloadMismatch (m̂ > 1.05·m_rated for `m_hat_tau_s`) is Degrade: levitation continues, motion drops to hold-to-run. Seat-first SS1 runs only for SEAT classes within `inertial.standoff_m` (50 mm) of the seat when every latched fault ≥ SafeStop is MainsLost or BusUnder; its x-move energy is part of the 3× hold-up need. Supply faults (`supervisor/power.rs`): MainsLost after a 2 ms debounce (SafeLand); BusUnder V_bus < 0.8·V_bus,nom with mains (SafeLand); BusOver above 1.05·V_chopper (SafeStop); HoldupLow (Degrade, refuse lift); OverTemp coil > 120 °C or magnet > 80 °C (SafeLand).

**Integer monitor** (`fabrail-safety`; depends on types and params only). `Monitor::new` is the only place with float arithmetic; every threshold becomes an integer in the check's own units, from the sealed diverse copies:

```text
mass in grams (u32), velocity in µm/s (i64), 2·KE in fJ = m_g·v_µm/s² (u128), momentum in g·µm/s (u128)
v = Δcounts·nm_per_count·20000/(Δticks·1000)  µm/s over a window ≥ 10 ms, i128, rounded half away from zero
force proxy Σi_abc² codes: |F| = K_f·|i_q| ≤ K_f·√(⅔·Σi_abc²)  (amplitude-invariant, §7)
```

| Check | Condition | Demand |
|---|---|---|
| Overspeed (SAF-008) | abs(v) > 1.1·v_cap in the direction of travel for > 10 ms | STO + ASC |
| EnergyLimit / MomentumLimit (SAF-012) | KE or p at rated mass above the cap for > 10 ms | SafeStop |
| ForceLimit (SAF-004) | measured force > 1.1 × the cap in force (EN 16005 episode: F_drive_max for 0.75 s, then F_static, then min(F_static, 80 N) after 5 s), within one fast period | SafeStop |
| CoilI2t / CoilOvercurrent | budget above 1.1·B / sample above 1.1·i_pulse | SafeLand |
| LsmOvercurrent / LsmPhaseSum | per-phase > 1.1·i_lsm_max / abs(Σi_abc) > i_lsm_max/10 | STO |
| GapEnvelope / GapRange | valid channel outside [gap_min, gap_max] / on an ADC rail or stopped converting | SafeLand |
| PositionLimit / UnintendedMotion (SLP, SDI) | x outside [x_min, x_max] / moving further out faster than 2 mm/s | SafeStop / STO |
| StoCheck (SSM) | STO commanded but phase currents above i_lsm_max/10 after 1 ms | STO + SS1 |
| HeartbeatLost | fast sequence unchanged for 3 periods | STO |
| HoldupLow / MainsLost | V_hold code below V_req (§14) / mains lost | Degrade / SafeLand |
| IsolateLimit (SAF-020) | ISOLATE envelope exceeded (§21) | STO + SS1 |

Tolerances (crate-root constants, documented design choices): measured force and current may exceed their limit by 10 % (`MON_TOL` 11/10); a lowered force cap is enforced after a 1 ms grace (`CAP_GRACE_TICKS`, ≈ 6·τ_cl; also the STO current-decay allowance); standstill is 2 mm/s; the LSM counts as energised above F_drive_max/20.

**Positive drive work (Wave 2 decisions D3, D4).** The cap checks (overspeed, KE, momentum) and the position checks (SLP, SDI) escalate only while the drive does positive work, F·v > 0 (`velocity::DriveEvidence`). The monitor's view of the sign of F is the sign of the commanded i_q, read by comparison only, together with its own measurement that the LSM carries current (Σi_abc² above the activity threshold). A persistence interval that expires is excused and restarted unless the LSM is energised AND either the command drives along the measured velocity, or the command claims braking but the carriage kept speeding up for longer than `BRAKE_ACCEL_SUSTAIN_TICKS` (0.5 s; a commutation sign error makes the "brake" accelerate the carriage, which is positive drive work, while a hand throw lasts ≤ 0.3 s). So a drawer thrown by hand past its cap, or a rail-profile disturbance, is met by governor damping only; a brief hand push that overpowers the brake is excused; a runaway, a drive holding an unsafe speed and a persistent sign error trip (core design F7). Outside [x_min, x_max] PositionLimit and UnintendedMotion trip only while the drive pushes along an outward velocity; floor-driven displacement past a limit with the LSM in STO, idle or braking (the physical stops of §8 take it) is counted as an excursion in `Monitor::pos_report` and reported, never a new trip. StoCheck (SSM) still supervises standstill through the phase currents, never through the velocity, because hand motion of an LSM in STO is legitimate. STO-level demands latch in the monitor until the supervisor leaves FAULT after a Reset; any STO-level demand above v_runaway = 2·v_cap becomes STO + ASC, and ASC is released to STO at standstill (F9). With PL d presence sensing fitted and healthy the sensed caps replace the energy basis (SAF-003).

**Arbitration** (`fabrail-rt`): LSM force ≤ min(core limit, verdict cap) through |i_q| ≤ min(F_core, cap)/K_f,copy; an STO/ASC demand sets the inverter mode with currents zeroed; a demand ≥ SafeStop forces FAULT_REACTION at the next supervisor release (latched for two supervisor periods); demands only rise within the latch window. STO and ASC act on propulsion only; levitation's safe state is SS1 Safe Landing, an energised function on the hold-up supercap.

**Verified by** `rs:core/supervisor/levstate::{transition_table, fault_latch, refuse_lift_on_crc}`, `rs:core/supervisor::{payload_mismatch_degrades_to_hold_to_run, mains_loss_lands_under_power, seat_first_from_standoff_on_mains_loss, presence_gate_and_hold_to_run}`, `rs:safety/velocity::{integer_velocity_exact, overspeed_trips}`, `rs:safety/energy::{integer_matches_float, ke_no_overflow_10mps, ke_trip}`, `rs:safety/force::en16005_timer`, `rs:safety/envelope::{gap_current_limits, isolate_limits_replace_clamp}`, `rs:safety/position::slp_limits`, `rs:safety/heartbeat::stale_trips`, `rs:safety/supply::holdup_energy`, `rs:safety/monitor::{nominal_motion_no_trip, overspeed_demands_sto_asc_until_standstill, nonfinite_and_sto_latch_until_reset}`, `rs:rt/arbitrate::arbitration_monotone`.

## §19 Ground motion and P+S synthesis

**Implemented in** `python/fabrail/{ground.py, pwave.py, records.py}` (numpy/scipy; offline input generation only, never a controller path). Every random draw comes from the `rng` argument, so a record is a pure function of (seed, arguments).

**Stationary process** (Shinozuka and Deodatis [R76]): a(t) = Σ_k √(2·G(f_k)·Δf)·cos(2πf_k·t + φ_k), φ_k uniform, one inverse FFT, period 1/Δf ≥ the record length. Spectra (one-sided, ω = 2πf):

```text
Kanai–Tajimi     G(f) = s₀·(ω_g⁴ + 4ζ_g²ω_g²ω²)/((ω_g² − ω²)² + 4ζ_g²ω_g²ω²)
Clough–Penzien   G_CP(f) = G_KT(f)·ω⁴/((ω_f² − ω²)² + 4ζ_f²ω_f²ω²)
Site classes     firm (15 rad/s, 0.6), medium (10 rad/s, 0.4; default), soft (5 rad/s, 0.2);  ω_f = 0.1·ω_g, ζ_f = ζ_g
```

(Kanai [R73]; Clough and Penzien [R74]; Der Kiureghian and Neuenhofer tabulation.) ω_f matters: the isolation Monte Carlo outcome is sensitive to it (SEI-015). The medium-soil CP model at 0.5 g gives PGV ≈ 1 m/s and PGD ≈ 0.6–0.8 m, and the El Centro-like surrogate (medium soil, 0.34 g, 30 s) has PGD ≈ 0.76 m against ≈ 0.1–0.2 m for the real 1940 record: long-period content is generous (§26).

**Envelope and scaling.** JHT envelope e(t) = (t/t₁)² for t < t₁, 1 to t₂, then e^(−c(t − t₂)) ([R75]; 40 s reference t₁ 2 s, t₂ 12 s, c 0.2 s⁻¹, scaled to the duration). Baseline correction: zero-phase 4th-order Butterworth high-pass at 0.05 Hz with zero pads of 1.5·n/f_c (Boore 2005), then a minimum-norm acceleration adjustment weighted by the 1 s moving RMS that makes the trapezoidal end velocity and displacement exactly zero. The record is scaled numerically so max|a| = PGA exactly. Spectrum matching to the ASCE 7-16 design spectrum (S_DS 1.0, S_D1 0.6, T_L 8 s; §11.4.6 [R16]): iterative Fourier-amplitude scaling by target/PSA (log-log interpolation, clipped to [0.2, 5] per iteration, 8 iterations), response spectra from the exact Nigam–Jennings recursion.

**Floor response.** One-way coupling: a_f = a_g + Γ·φ(z)·D̈ with D the relative response of the first-mode SDOF (T₁, 5 %), φ(z) = sin(πz/2h), Γ = 4/π (uniform shear beam); the code-level amplification factor is 1 + 2z/h (ASCE 7-16 Eq. 13.3-1; FAB-SEI-020).

**P+S records** (Boore 2003 stochastic method [R77]; `pwave.boore_ps`). Per phase and component: Gaussian white noise under a Saragoni–Hart window of length 2·T_d (T_d = 1/f_c + 0.05·R), normalised, multiplied by

```text
A(f) = C·M₀·(2πf)²/(1 + (f/f_c)²) · (1/R) · exp(−πfR/(Q(f)·v)) · exp(−πκf) · S(f)
M₀ = 10^(1.5M + 16.05) dyne·cm,   f_c = 4.906e6·β·(Δσ/M₀)^⅓ (Brune [R78], Δσ 30 bar),   Q(f) = 180·f^0.45,   κ = 0.04
```

S(f) is the Kanai–Tajimi site filter for the S phase. The S phase keeps Boore's absolute level (its PGA is an output, reported in the record's metadata: ≈ 0.04 g at M7 R_epi 20 km, ≈ 0.01 g at M5 R10 and M7.8 R100, lower than typical ground-motion models; §26). The P phase uses V_p, a 20° incidence (H/Z = tan 20°) and, by default (`p_corner = "wu_kanamori"`), a corner solved per realisation so its 3 s τ_c equals the Wu–Kanamori regression; its level is rescaled so its 3 s Pd, computed by the detector's own chain, equals Wu–Zhao at the hypocentral distance (plan §2b item 6). Travel times V_p 6.0 and V_s 3.5 km/s. `p_corner = "brune"` restores a 1.5·f_c corner.

**Site noise** (false-trigger suites; rates and amplitudes \[UNCERTAIN\]): MEMS white noise (22.5 µg/√Hz; off for scenario `site` series because the simulator adds it); footstep episodes (1 per 120 s, 1.5–2.5 Hz trains of 10–30 Hz damped pulses, 1–10 mg); door slams (1 per 600 s, 5–50 mg, 8–20 Hz); HVAC tones (12–25 Hz, 0.05–0.5 mg); self-actuation (1 per 60 s, ±25–50 N LSM reaction through a 15 Hz, ζ 5 % structural mode of 2000 kg). Streamed in 10-minute chunks for the 24 h replay; the concatenation does not depend on the chunk length.

**Verified by** `py:ground_motion::{kt_cp_psd, asce_spectrum, deterministic, floor_amplification, jht_envelope, boore_corner_frequency, response_spectrum_exact, site_noise_chunks, elcentro_surrogate}` (FAB-SEI-018, FAB-SEI-020), `rs:plant/structure::floor_amplification_matches_asce7`.

## §20 Detection and fusion

**Implemented in** `crates/fabrail-core/src/seismic/{frontend.rs, stalta.rs, pdtc.rs, discriminate.rs, fusion.rs, mod.rs}` (100 Hz seismic group).

**Front end.** The structure-mounted ADXL355 (never the carriage; SEI-001) at 500 Hz ODR is decimated by 5 with a second-order CIC, the triangular FIR [1 2 3 4 5 4 3 2 1]/25 on the integer codes (exact in i64, one exact f32 product), whose double zeros at multiples of 100 Hz are the bands that fold onto DC (droop −0.3 dB at 10 Hz, −1.1 dB at 20 Hz; an 80 Hz tone is attenuated by 24.5 dB). Each axis then passes a 4-pole Butterworth band-pass 0.5–20 Hz (both edges prewarped, TDF-II sections, primed to the DC steady state of the first sample so gravity and tilt do not ring). PGA is the band-passed horizontal magnitude (tilt- and gravity-free).

**STA/LTA** on the band-passed vertical channel (Allen [R59]; recursive form), e_k = x_k², gains g(N) = 1/min(k, N):

```text
sta_k = sta_{k−1} + g(50)·(e_k − sta_{k−1})
lta_k = lta_{k−1} + g(3000)·(e_k − lta_{k−1})      (not updated while triggered)
on:  k ≥ 3000 and sta/max(lta, 1 LSB²) ≥ 4.0;   off: ratio < 1.5
```

For a variance step 1 → K the continuous estimate is τ_sta·ln((K − 1)/(K − θ)) = 0.1116 s at K = 16, θ = 4; the exact discrete recursion (with the LTA term) triggers at sample 13 (0.13 s), which `py:seismic_detector::trigger_time_step` matches to ±1 sample (SEI-003). A trigger held longer than `alert_timeout_s` (60 s) is released so a step change in ambient noise cannot freeze the LTA forever.

**Pd and τc** (Wu and Kanamori [R61]; Wu and Zhao [R63]). On the anti-aliased vertical channel, every high-pass a 2-pole Butterworth at 0.075 Hz (Tustin, prewarped):

```text
a_b = HP(a_z − a_z,0)        first sample subtracted (keeps f32 rounding out of Pd)
v   = IHP(a_b),  u = IHP(v)  IHP(s) = s/(s² + √2·ω_a·s + ω_a²): trapezoidal integrate-then-high-pass in one biquad
u̇   = HP(v)
Pd  = max|u|,   τc = 2π·√(Σu²/Σu̇²)      over 1 s (early) and 3 s windows from the pick
τp: X ← αX + u̇², D ← αD + (Δu̇/Δt)², τp = 2π√(X/D), α = 0.99 [M]
```

**Discrimination.** A pick is accepted unless the actuator veto holds (the rail's own actuation, unless PGA ≥ 80 gal); Z/H > 1 over the first 0.5 s confirms it. After the 1 s window: levels τc ≥ 0.6 s and Pd ≥ 0.5 cm → L3; τc only → L2; Pd only → L1. PROTECT fires when Level ≥ 2 OR Pd ≥ 0.35 cm OR PGA ≥ 80 gal (the PGA rule applies at any time); Level 1 means freeze only (plan §2b item 6).

**Fusion and state machine.** NORMAL → TRIGGERED (freeze) → PROTECT (class action) → SHAKING → QUIET (30 s) → INSPECT (manual reset) → NORMAL; a cancel returns to NORMAL only without local evidence. "Shaking" is a sample with PGA ≥ 80 gal within the last 2 s (1/f_lo) or a live trigger past the S-ETA. S-ETA from a network alert: received + origin offset + R_hyp/V_s, R_hyp = √(r_epi² + depth²). Local S-ETA: magnitude from log τc = 0.296·M − 1.716 (Wu and Kanamori 2008 \[UNCERTAIN\], ±0.4 M), Wu–Zhao inverted for R, then S−P = R·(1/V_s − 1/V_p), biased early by `eta_sigma_s` = 1.5 s (early is the safe side). Network or injected alerts alone may cause only FREEZE, LAND, LOCK or a passive ride-through; alert-triggered motion needs a local P pick or ≥ 2 independent feeds (`action_policy::admit`). fabrail is not an EEW provider: every alert in simulation is synthetic or injected.

**Closed forms** (`derive`): Wu–Zhao log₁₀ Pd[cm] = −3.463 + 0.729·M − 1.374·log₁₀ R[km] (M7 R20 → 0.7118 cm; M6.5 R30 → 0.1762; M5 R10 → 0.0643; M7.8 R100 → 0.2987); S−P = R·(1/3.5 − 1/6.0) = 0.1190 s/km; lead time R/V_s − t_alert; network/on-site crossover r = V_p·(t_network − t_onsite) = 6.0·(8 − 1.2) = 40.8 km.

**Verified by** `rs:core/filters::butterworth_prewarp`, `rs:core/seismic/stalta::step_trigger_matches_exact_recursion`, `rs:core/seismic/pdtc::{tauc_sinusoid, pd_linear_in_amplitude}` (FAB-SEI-004), `rs:core/seismic/discriminate::{alert_levels, protect_rule, veto_bypass_pga}` (FAB-SEI-005, FAB-SEI-006), `rs:core/seismic/fusion::{fsm_exhaustive, eta_from_alert}` (FAB-SEI-007, FAB-SEI-008), `py:physics_anchors::{lead_time_crossover, wu_zhao_pd}` (FAB-SEI-012).

## §21 Seismic response: land-first, isolation, AMD/TMD

**Implemented in** `crates/fabrail-core/src/seismic/{policy.rs, isolate.rs, amd.rs, action_policy.rs}`, `crates/fabrail-plant/src/structure.rs`, `derive::{transmissibility, t_sky, t_min_iso, den_hartog, warburton, warburton_force}`.

**Why land-first.** A levitated, unforced carriage is a free mass along x: it stays in place while the floor moves under it and drifts relative to the floor by about the ground displacement (FAB-SEI-013; `rs:plant/travel1d::free_mass_follows_the_floor_inertially`). The lateral capacity of passive reluctance guidance and the guide fin is of order 0.1–0.3 g against 0.3–1 g of design-level floor shaking, and an attractive EMS can only pull, so a downward floor pulse saturates the low side of the lift (FAB-SEI-017). The default is therefore freeze, then land (and lock where allowed).

**Class actions** (`policy::class_action`; the seismic action is declared not a rated safety function, SF-08): TRIGGERED → Freeze (decelerate at a_brk and hold); PROTECT → the class action: a seated DRAWER stays seated, an open one freezes and lands (on its travel rollers, or into the seat when it is at the seat); DOOR stops, lands on rollers, releases ASC and never locks (a fire-rated door never opens; a non-rated door may open only if the move ends at rest by t_S − 1 s with only local sources, t_move = d/v + v/a); WALL and HEAVY centre only if reachable (|x − x_c| ≤ v·(t_budget − t_land), F11), then land and lock; SHAKING, QUIET and INSPECT → the landing action. The core never selects the sovereign actions (RideThrough, CompleteClose, CloseAfterShaking); the exotic tier reaches them only through the `ActionPolicy` hook, whose proposals pass the core admission filter (toward safety only; no opening or unlocking when any non-local source contributed; alert motion only in the closing direction, capped at the unattended limit).

**Isolation** (research tier; `isolation.enable = false` everywhere). Karnopp skyhook:

```text
F      = −k·x_rel − c_sky·ẋ_abs,   k = m·ω², c_sky = 2ζ·m·ω, ω = 2π/T_iso, ζ = 0.3
T_iso  ≥ max(2 s, 1.1·T_min),   T_min = 2π·√(m·s/F_cont)        (WALL 6.28 s at 500 kg, 0.2 m, 100 N)
|F|   ≤ F_max = min(F_cont, IsolateLimits.f_cont_n);  stroke ≤ min(isolation.stroke_m, IsolateLimits.stroke_m)
T(r, ζ) = √((1 + (2ζr)²)/((1 − r²)² + (2ζr)²))                 (T(3, 0.2) = 0.1931, T(√2) = 1)
```

Stroke governor: moving outward with r_s = s − |x| left, the force to stop inside the envelope is m·v²/(2r_s); once it reaches F_max/`a_brk_ratio` the governor overrides the law with full braking −sign(v)·F_max. Floor acceleration above F_max/(2m) can still drive the carriage to the envelope, which is why WALL isolation largely saturates and the bumpers and IsolateLimits stay in force (SEI-015). In ISOLATE the monitor replaces (never disables) its clamp, KE/momentum and overspeed checks with the explicit IsolateLimits set (stroke envelope 0.2 m, bumper KE at half the bumper rating: 12 / 56.5 / 281.5 / 2812.5 J, continuous force F_cont), an R-tier deviation declared in SAFETY.md (SAF-020).

**AMD and TMD** (research tier). `Amd::step` returns the LSM force on the carriage (the reaction mass); the structure feels −F. DVF: F = +c_dvf·ẋ_s; LQR: F = −K·[x_s, ẋ_s, x_r, ẋ_r] with gains injected from Python (scipy CARE on this input sign). Clamped to ±`amd.f_max_n` (≤ F_cont) with the isolation stroke governor. Tuning closed forms: Den Hartog f = 1/(1 + μ), ζ = √(3μ/(8(1 + μ)³)), peak √(1 + 2/μ) ([R80]; μ 0.02 → 0.98039 / 0.08407 / 10.05; μ 0.01 → 0.99010 / 0.06033 / 14.18); Warburton base excitation f = √(1 − μ/2)/(1 + μ), ζ = √(μ(1 − μ/4)/(4(1 + μ)(1 − μ/2))) ([R81]; μ 0.02 → 0.97548 / 0.07019); force excitation f = √(1 + μ/2)/(1 + μ) (0.99257 / 0.98528 at μ 0.01 / 0.02). The mass ratio decides the benefit: a 5 t panel in a 10⁷ kg building is μ = 0.05 %, negligible (SEI-021). Mass dampers are mainly wind devices.

**Structure tier.** Storey displacements u relative to the ground and the device stroke x relative to storey j:

```text
M·ü + C·u̇ + K·u = −M·1·a_g − e_j·F_int
m_d·(ẍ + ü_j + a_g) = F_int
C = a₀·M + a₁·K,   a₀ = 2ω₁ω₂(ζ₁ω₂ − ζ₂ω₁)/(ω₂² − ω₁²),   a₁ = 2(ζ₂ω₂ − ζ₁ω₁)/(ω₂² − ω₁²)   (Chopra)
```

K is the shear-building tridiagonal stiffness (eigenvalues by cyclic Jacobi rotations, +−×÷√ only, so native and wasm32 agree bit for bit); one storey uses c = 2ζ₁√(k·m). F_int is the LSM thrust (Run: K_f·i_q with the current loop algebraic at 1 ms; ASC: the back-EMF drag of §7; STO: 0), the dissipation ledger at nominal flux, the bumpers, the parking lock and a lean force. The maglev is assumed levitated throughout; hand and vertical dynamics are outside this tier. An LSM-emulated Den Hartog TMD matches the closed-form 2-DOF response within 2 %.

**Verified by** `rs:core/seismic/policy::{class_policy_table, egress_door_policy}` (FAB-SEI-010, FAB-SEI-011), `rs:core/seismic/isolate::{cfg_from_t_iso_and_limits, stroke_governor}`, `rs:core/seismic/amd::{dvf_damps_the_structure_and_saturates, lqr_gains_and_stroke_governor}`, `rs:core/seismic/action_policy::{network_only_alert_cannot_move_or_open, confirmed_close_is_capped_and_directional, lock_forbidden_on_egress}`, `rs:plant/structure::{two_dof_modes_and_rayleigh_fit, sdof_resonance_matches_dynamic_amplification, coupled_energy_balance, lsm_tmd_emulation_matches_the_two_dof_frequency_response}`, `rs:params/derive::{tmd_and_isolation_anchors, t_min_iso_uses_f_cont}`, `rs:safety/envelope::isolate_limits_replace_clamp` (FAB-SAF-020).

## §22 Human and disturbance models

**Implemented in** `crates/fabrail-plant/src/interaction.rs` (truth) and `python/fabrail/human.py` (scenario events).

**Hand.** An arm impedance (m 1–2 kg, b 5–20 N·s/m, k 200–1000 N/m; default 1.5 kg, 10 N·s/m, 500 N/m) driven by a piecewise-linear force profile, riding on the body front:

```text
F_h = F_drive − b·ẋ − k·x_rel − m·ẍ          x_rel from the contact point (the arm's anchor)
```

A grip can pull; a push-only contact carries force only in the direction of F_drive and separates when the body moves away (push-to-open). The algebraic loop through the arm mass is solved exactly: with F_h affine in the body acceleration a, a* = (F_other + F_h(0))/(m + F_h(0) − F_h(1)) and the applied force is F_h(a*). Profile kinds: push, push_release, pull, throw, lean, slam, pinch (rise 50–150 ms). A W1.5 finding: grip profiles with an anchored 500 N/m arm that end abruptly make an intent-triggered automatic move drag the arm, which the obstruction detector correctly flags; the scenario builders are to use push/release profiles for automatic moves (Wave 2).

**Obstacles** (ISO/TS 15066:2016 Annex A body regions [R7]): chest 25 N/mm, 40 kg; hand 75 N/mm, 0.6 kg; face 75 N/mm, 4.4 kg \[M\]; lower leg 60 N/mm, 75 kg \[M\] \[UNCERTAIN\]; quasi-static limits reported against (140 / 140 / 65 / 130 N), never a certification claim. The closing edge (moving toward −x) meets the body surface at x_ob; the force is k_s·δ with δ = x_ob − x_edge and k_s = k_edge·k_body/(k_edge + k_body) (pure springs, as Annex A models the energy transfer). Closed forms: clamped F_max = v·√(m·k_s), free v·√(μ·k_s) with μ = (1/m_H + 1/m)⁻¹. As implemented every obstacle is clamped (a fixed surface); the free-body kinematics `FreeBody` exist but are not stepped by the tiers (§26).

**Disturbances** (plan §2b item 11). Lean and smoke-control Δp are symmetric trapezoids: lean rise ≥ 1 s, smoke ramp ≥ 2 s (rejected otherwise). Footsteps, door slams, HVAC and self-actuation are floor-noise sources of §19.

**Verified by** `rs:plant/interaction::{series_stiffness_and_body_table, clamped_hand_through_the_edge_seal_matches_f6, free_hand_transient_matches_reduced_mass_form, hand_grip_and_push_semantics, disturbance_and_hand_profiles}`, `py:ground_motion::human_events`.

## §23 Stray field

**Implemented in** `crates/fabrail-plant/src/stray.rs` (FAB-ENV-001).

Every PM element is a uniformly magnetised block of the pole face (l along x, w along y) and thickness h_m, magnetised along its normal (toward the rail for a lift element), centred h_m/2 behind the pole face, with moment m = B_r·V/μ₀ (V = w·l·h_m, B_r at 20 °C).

```text
dipole sum    B = (μ₀/4π)·(3(m·r̂)r̂ − m)/|r|³               far field; the ENV-001 target form
exact block   surface-charge method (Furlani 2001, §2.7): faces z = ±h/2 carry ±M, B = μ₀H outside
```

Both are bare magnets in free space: the rail and pole-piece iron, which close most of the flux, are not modelled, so the result is an upper-bound style estimate, and coil fields are omitted (zero-power control holds i ≈ 0). Single magnet, on axis, 10 cm from its centre (dipole): DRAWER 0.22 mT, DOOR 1.62 mT, WALL 15.9 mT ("16 mT unshielded"), HEAVY 79.7 mT ("80 mT", shrouds required). DOOR reaches 0.5 mT only at ≥ 14.8 cm from the magnet centre, so the ≤ 0.5 mT at 10 cm target needs the 10 cm measured from an accessible surface with a housing standoff of about 5 cm or more (status S; [R17], [R18]).

**Verified by** `rs:plant/stray::{dipole_on_axis, cuboid_matches_on_axis_closed_form_and_far_dipole}`; the budget by `py:environment::stray_field_budget` (Wave 2).

## §24 Numerical method (multi-rate, LET, RK4, stability rules, RNG, f32/f64)

**Implemented in** `crates/fabrail-rt/src/{exec.rs, groups.rs, mailbox.rs}`, `crates/fabrail-sim/src/{engine.rs, scheduler.rs, rng.rs}`, `crates/fabrail-plant/src/{integrator.rs, stability.rs}`, `crates/fabrail-types/src/{units.rs, fmath.rs}`.

**Rate groups** (`control.rates`): base 20 kHz (coil current PI in voltage mode, samples, latches), fast 10 kHz (DRAWER, DOOR) or 5 kHz (WALL, HEAVY; levitation, motion, haptics, monitor), supervisor 1 kHz, seismic 100 Hz.

**LET.** A job released at tick r reads mailbox values whose activation tick is ≤ r and publishes values that activate at r + P (P its period in base ticks). The base task always runs first in a tick (it latches the `FastRaw` the fast release consumes at the same tick); the other due groups may run in any order and produce identical outputs (all 3! = 6 orders tested, FAB-FW-004). The fast group is a pure function of (FastRaw, FastIn) and its own state.

**Executive rate in every tier (Wave 2 decision D8).** The executive runs every 20 kHz base tick in every plant tier, so every controller sees the sample period it was designed for (`FastLoop::dt_s` is fixed from the rates). The measured cost is ≈ 0.34 µs per tick for executive and HAL plus ≈ 0.04 µs for the harness, which bounds Travel1d to ≈ 120× real time (W1.5: 120.7× on WALL). D8 keeps the full-rate executive (SIL fidelity) and sets the Travel1d budget to ≥ 100× real time (≈ 130× on WALL in Wave 2) and the isolation Monte Carlo N = 200 × 60 s to ≤ 60 s on 8 workers (`sim/tests/perf`); SPECIFICATION's FAB-SW-008 prose follows. Carriage6 measured 11.5× (DOOR, lag), 16.2× (voltage) and 14.0× (HEAVY) in W1.5.

**Plant integration.** Classic RK4 (Butcher 0 | ½ ½ | ½ 0 ½ | 1 0 0 1 | ⅙ ⅓ ⅓ ⅙) with the inputs held (ZOH) over the step and the floor acceleration at t, t + h/2 and t + h (the harness interpolates the floor series linearly). h = 50 µs on Carriage6 and Heave1 (every base tick), 1 ms on Travel1d and Structure (every 20 ticks). The quaternion is renormalised after every step; lag-mode currents and non-state fluxes are set algebraically after the step.

**Stability rules** (`stability::report_tier`; a run whose report is not ok is refused). Every oscillatory element must satisfy |λ·h| ≤ 0.2 (RK4's imaginary-axis limit is 2√2): the tiers cap the penalty contacts (k ≤ m_eff·(0.2/h)²) and the bristle presliding stiffness at use, so those entries report ≤ 0.2 by construction; the guide fin, side pads, closing-edge seal, closed levitation loop (ω = 2.5·p), top-hung sway pendulum and sealed DRAWER air spring are reported uncapped and can refuse a run. First-order lags (current loops, eddy lag) are real entries; a lag with τ < 2h is integrated algebraically. The 1 ms tiers report only x-axis elements. The bristle states need no entry because they are advanced exactly outside RK4 (§8); the DRAWER cavity leak rate is not reported (§26).

**RNG** (in-repo, stable across Rust, crate and platform versions). SplitMix64 seeding (Steele, Lea and Flood 2014) of Xoshiro256++ streams (Blackman and Vigna 2021), one independent stream per (noise seed, tag, device, channel) with key = tag << 32 | device << 16 | channel; normals by Box–Muller with the second cached; transcendentals through fmath. Python owns scenario synthesis with numpy `SeedSequence`: one run splits `SeedSequence(seed).spawn(2)` into the synthesis generator and the Rust `noise_seed`; ensembles use `spawn(n)` and return results in index order, so they do not depend on the worker count (numpy does not promise `Generator` streams across versions, NEP 19 [R125]).

**f32 / f64 split.** The plant and `derive` are f64 (native `f64::sqrt` allowed in the std plant). Controllers are f32 with init-time gain design in f64 through fmath, cast once. The monitor is integer per tick. Absolute position is `PosNm(i64)`; every position is used as an integer difference first, then cast; trajectory and admittance states integrate in `PosAccum` (integer base plus an f32 sub-nm fraction). Time is `Tick(u64)`; a time since a segment start is the integer tick difference cast once. All transcendental and rounding math goes through `fabrail_types::fmath` (soft-float libm 0.2.16, `force-soft-floats` [R127]), and `powi` is an explicit multiply loop, so host, MCU and wasm32 produce the same bits; the selftest hash is identical native, with `fault-injection` and in wasm32.

**Verified by** `rs:rt/tests/let_order::permutation_invariant` (FAB-FW-004), `rs:rt/tests/golden::selftest_hash_frozen` (FAB-FW-005), `rs:sim/tests/replay::bit_identical` (FAB-FW-006), `rs:sim/tests/determinism::{same_seed_identical, batch_worker_invariance}`, `rs:plant/integrator::{rk4_exponential_decay_is_fourth_order, rk4_harmonic_oscillator_arrays}`, `rs:plant/stability::{rejects_stiff, presets_pass_at_their_tier_steps}`, `rs:sim/rng::{stream_stability, substream_independence}`, `rs:types/units::{posnm_diff_exact, posaccum_no_drift_15m, tick_time_exact}`, `rs:types/fmath::known_values`, `rs:core/tests/invariance::offset_invariance` (FAB-PRP-008).

## §25 Closed-form benchmarks and tolerance table

Every analytic anchor with its value, tolerance class and source. Values are computed from the presets by the functions named; SPECIFICATION quotes the rounded forms. (E) exact algebra rel 1e-12; (I) integration; (S) standard-clause limit; (R) research metric; (MC) Wilson 95 % upper bound; (P) provisional.

**Magnetics and levitation**

| Anchor | Value | Class | Source | Asserted by |
|---|---|---|---|---|
| Open-loop pole p = √(4g₀/(2g* + h′)) | 69.488 / 47.572 / 32.498 / 32.498 rad/s | (E); simulated fit ±0.5 % | core §E.5; LEV-003 | `rs:params/derive::open_loop_pole`; `py:physics_anchors::open_loop_pole_and_caps`; `rs:plant/heave1::open_loop_pole_fit` |
| m(g) DRAWER at 1.5 / 5.0 / 6.2 mm | 45.756 / 9.963 / 7.120 kg | (E) | LEV-005 | `rs:params/derive::zero_power_mass`; `py:physics_anchors::zero_power_mass` |
| DRAWER float gap at 48 / 26 / 8 kg | 1.4276 / 2.5000 / 5.7608 mm | (E); 1.427 quoted ±0.001 mm | core §E.5 | `rs:params/derive::zero_power_gap_anchors` |
| DOOR +98.1 N / WALL +490.5 N gap | 4.5967 / 9.1357 mm | (E); sim ±2 µm | LEV-004 | `rs:params/derive::zero_power_gap_anchors`; `py:levitation::zero_power_load_step` |
| DRAWER seat currents 48 / 8 kg; from 6.5 mm | +11.201 / −4.989 A; 27.42 A | ±0.5 % against the rounded-pole quotes | F17; LEV-017 | `rs:params/derive::seat_unseat_currents` |
| Band bias at 48 kg, 1.5 mm | +0.391 A | ±2 % (sim) | F1; LEV-018 | `rs:core/levitation/zeropower::bias_current_closed_form` (1e-4) |
| Stop ratio up / stay-landed | 2.163 / 0.8250; 1.278 / 0.8487; 1.256 / 0.8631 (WALL, HEAVY) | (E); quotes ±0.002 (P) | LEV-006; F18 | `rs:params/derive::stop_ratio_table`; `py:physics_anchors::stop_force_ratios`; `rs:plant/tests/anchors::stop_ratios` |
| k_i / L at g* | 7.890 / 19.39 / 41.47 / 138.2 N/A; 1.983 / 6.645 / 13.03 / 28.95 mH | (E) | core §C.2 | `rs:params/derive::k_i_and_inductance_table` |
| Thermal shift +40 K | −0.1950 / −0.4160 / −0.8914 mm | rel 1e-9; sim ±3 % | LEV-014 | `rs:params/derive::thermal_gap_shift_values`; `py:physics_anchors::thermal_gap_shift` |
| Impact speed up / down | DOOR 50.58 / 55.59 mm/s; WALL 68.81 / 76.27; DRAWER 26 kg 109.6 / 197.3 | (E); sim ±5 % | LEV-019 | `rs:params/derive::impact_speed_energy`; `rs:plant/tests/anchors::impact_speed_closed_form` (1 %) |
| Discrete ZP poles | within 2 % of the design, ζ ≥ 0.5 | (P) | LEV-007 | `rs:core/tests/levitation_linear::discrete_poles_match_design` |
| Critical eddy lag | ≥ 2 ms every mode (lowest DRAWER pitch 2.24 ms) | analysis, bisection to 1 µs | LEV-020 | `rs:core/tests/levitation_linear::stable_for_eddy_lag_to_2ms` |
| Roll–sway coupling ratio n·k_y·h²/(k_s·Σr² + m*·g₀·h) | HEAVY 1.97 (coupled design), DRAWER 0.032 (single-DOF) | (E) | D1 | `rs:params/derive::roll_sway_coupled_design` |
| Noise force gain | 0.310 / 0.559 / 1.304 / 6.518 N/µm | rel 1e-9 | HAP-011 | `rs:params/derive::noise_force_gain_closed_form`; `py:physics_anchors::sensor_noise_force_gain` |
| Current PI small-signal step | rise ≤ 0.3 ms, overshoot ≤ 10 % | (S) | LEV-008 | `rs:core/current::pi_step_response` |
| Allocation T·A = I | 1e-6 (f32) | (E) f32 | LEV-009 | `rs:core/levitation/modal::allocation_identity` |

**Plant integration anchors** (`crates/fabrail-plant/tests/anchors.rs`)

| Anchor | Tolerance | Measured (W1) | Asserted by |
|---|---|---|---|
| Energy conservation, dissipation off | rel 1e-8 | 1e-13 | `rs:plant/tests/anchors::energy_conservation` |
| Power balance, voltage mode | rel 1e-6 | 1e-9 | `rs:plant/tests/anchors::power_balance_voltage_mode` |
| Heave1 ≡ constrained Carriage6 | rel 1e-9 | 8.5e-12 | `rs:plant/tests/anchors::tier_equivalence` |
| Co-energy force vs ∂W′/∂q | rel 1e-7 (FD) | — | `rs:plant/magnet::coenergy_force_consistency` |
| F·v = 1.5·e_q·i_q | rel 1e-9 | — | `rs:plant/lsm::kf_ke_and_power_balance` |
| Bristle no-ratchet | < 0.05 mm per 60 s | — | `rs:plant/contact::bristle_no_ratchet` |
| RK4 order | fourth order on e^(−t) and a harmonic oscillator | — | `rs:plant/integrator::rk4_exponential_decay_is_fourth_order` |

**Motion, safety, haptics**

| Anchor | Value | Class | Source | Asserted by |
|---|---|---|---|---|
| Speed caps √(2E/m), p/m | 0.26536 / 0.18385 (open) / 0.152 (close) / 0.0446 / 0.025 m/s | (E) | SAF-002 | `rs:params/presets::speed_caps` |
| d_stop at the caps | 33.423 / 36.322 / 8.861 / 4.906 mm | rel 1e-9 | PRP-005 | `py:physics_anchors::d_stop_values`; `rs:core/tests/scurve_props::limits_and_stop_distance` |
| Clamp force √(2Ek) at 1.69 J | chest 290.69 N, hand 503.49 N, hand + edge 172.69 N; WALL chest 157.68 N | (E); sim ±0.5 % | SAF-005 | `rs:params/derive::clamp_force_values`; `py:physics_anchors::clamp_force_energy_form` |
| Unattended close speed | 0.2305 m/s (48 kg, 150 N, 8.82 N/mm) | (E) | plan §4b | `rs:params/derive::unattended_close_speed_value` |
| ISO/TS 15066 transient speed | 0.1455 / 0.2910 m/s (500 kg, chest 140 / 280 N) | rel 1e-9 | [R7] | `py:physics_anchors::iso15066_speeds` |
| Passivity bound 2b/(T + 2T_d) | 13.3 / 33.3 / 66.7 / 333 kN/m | (E) | HAP-009 | `rs:core/haptics/passivity::stiffness_bound` |
| Momentum observer ramp error | ≤ 10 % of peak, 5–25 N over 50–150 ms (WALL ≥ 75 ms, HEAVY ≥ 120 ms) | (S) | HAP-004 | `rs:core/haptics/observer::ramp_tracking` |
| Terminus d_0 / d_b / t_arrive | table of §16.1 | ±1 % (closed form), ±2 % closed loop | HAP-016 | `rs:params/derive::terminus_t_arrive_table`; `rs:core/tests/terminus::t_arrive_matches_closed_form_table` |
| Terminus blend continuity | v − j·t_b²/2 = d_b/τ, j·t_b = d_b/τ² | (E) | §16.1 | `rs:params/derive::terminus_t_arrive_table` |
| Semilog fit of the capture | R² ≥ 0.999 (closed loop 0.99989–0.99997) | (S) | HAP-013 | `rs:core/motion/terminus::exponential_capture_exact`; `rs:sim/tests/terminus_closed_loop::*` |
| τ_min = v/a_max − a_max/(2j) | 0.1394 / 0.1701 / 0.2473 / 0.2725 s | (E) | HAP-014 | `rs:params/derive::tau_min_values`; `rs:params/validate::rejects_tau_below_tau_min` |
| Bumper rating ½mv² | 24 / 112.5 / 562.5 / 5625 J | (E) | SAF-014; F13 | `py:params::bumper_energy_rating` |
| Manual force relieved / bound | 1.218 / 1.412; 0.695 / 2.942; 3.283 / 14.71; 32.83 / 147.1 N | (E) | SAF-010; F8 | `py:physics_anchors::impact_speeds_and_manual_force` |
| Hold-up need / available | 111.3 / 7.9 / 52.2 / 299.0 J vs 486 / 486 / 486 / 7776 J | (E) | SAF-009 | `rs:params/derive::passivity_and_holdup`; `rs:core/supervisor/power::presets_meet_holdup_margin` |
| Hold-up floor V_req | 10.498 / 6.425 / 8.415 / 27.846 V | rounded up to 1 mV | SAF-009 | `rs:safety/supply::holdup_energy` |
| Integer KE | within one quantum of float; no overflow at 10 m/s for 5 t | (E) integer | SAF-012 | `rs:safety/energy::{integer_matches_float, ke_no_overflow_10mps}` |
| Integer velocity | within ½ µm/s of the exact rational | (E) integer | SAF-008 | `rs:safety/velocity::integer_velocity_exact` |

**Air, power, environment**

| Anchor | Value | Class | Source | Asserted by |
|---|---|---|---|---|
| Piston force, 0.06 m², 0.5 m/s, 40 cm² | 5.644 N (C_d 0.6) | (E); ±5 % against 5.6 N | ENV-003 | `py:physics_anchors::piston_pressure`; `rs:plant/air::sealed_spring_rate` |
| Sealed air spring γP₀A²/V | 17.02 kN/m | (E) | ENV-003 | `rs:plant/air::sealed_spring_rate` |
| μ_eq levitated at 0.1 m/s | 1.96e-3 / 2.04e-3 / 1.84e-3 / 1.43e-3 (closed form, m*) | (S) band [1e-3, 1e-2] | HAP-010 | `py:physics_anchors::equivalent_friction_band` |
| Stray field on axis at 10 cm | 0.22 / 1.62 / 15.9 / 79.7 mT | (E) dipole | ENV-001 | `rs:plant/stray::dipole_on_axis` |
| Bus sizing n·i_pulse²·R | 922 / 680 / 1120 / 7680 W ≤ P_max | (E) | W1.5 | `rs:plant/power::mains_bus_limit_and_chopper` |

**Seismic and structural**

| Anchor | Value | Class | Source | Asserted by |
|---|---|---|---|---|
| Wu–Zhao Pd | M7 R20 0.7118 cm; M6.5 R30 0.1762; M5 R10 0.0643; M7.8 R100 0.2987 | rel 1e-4 | SEI-012; [R63] | `py:physics_anchors::wu_zhao_pd`; `rs:params/derive::seismic_anchors` |
| S−P and crossover | 0.11905 s/km; 40.8 km | (E) | SEI-012 | `py:physics_anchors::lead_time_crossover` |
| STA/LTA step trigger (K = 16) | exact discrete 13 samples (0.13 s); continuous 0.1116 s reported | ±1 sample | SEI-003 | `rs:core/seismic/stalta::step_trigger_matches_exact_recursion`; `py:seismic_detector::trigger_time_step` |
| τc of a sinusoid | the period ±1 % | (S) | SEI-004 | `rs:core/seismic/pdtc::tauc_sinusoid` |
| Butterworth corners | prewarped −3 dB at the corner | 1e-6 | SEI-002 | `rs:core/filters::butterworth_prewarp` |
| Isolation transmissibility | T(3, 0.2) = 0.19310; T(√2) = 1; T(1, 0.2) = 2.693 | (E); sim ±2 % | SEI-014 | `rs:params/derive::tmd_and_isolation_anchors`; `py:isolation::transmissibility` |
| Skyhook T_sky(r, 0.7) | 0.2437 at r = 2; 0.0400 at r = 5 | (E); sim ±20 % | HAP-018 | `rs:core/motion/inertial::residual_follows_t_sky` |
| T_min WALL (F_cont) | 6.283 s | (E) | SAF-020; plan §2b item 5 | `rs:params/derive::t_min_iso_uses_f_cont` |
| Den Hartog μ 0.01 / 0.02 | f 0.99010 / 0.98039; ζ 0.06033 / 0.08407; peak 14.177 / 10.050 | (E) | SEI-016; [R80] | `rs:params/derive::tmd_and_isolation_anchors`; `py:physics_anchors::damper_and_isolation_anchors` |
| Warburton base / force | 0.98762 / 0.04981 and 0.97548 / 0.07019; f 0.99257 / 0.98528 | (E) | SEI-016; [R81] | `py:physics_anchors::damper_and_isolation_anchors` |
| Floor amplification | 1 + 2z/h | (E) | SEI-020 | `py:ground_motion::floor_amplification`; `rs:plant/structure::floor_amplification_matches_asce7` |
| LSM TMD emulation | 2-DOF closed-form FRF within 2 % | (R) | SEI-021 | `rs:plant/structure::lsm_tmd_emulation_matches_the_two_dof_frequency_response` |

**Numerics**

| Anchor | Value | Class | Asserted by |
|---|---|---|---|
| PosAccum drift over 15 m | 0 nm over 1e6 steps | (E) | `rs:types/units::posaccum_no_drift_15m` |
| Controller offset invariance | bit-identical under x + L and tick + 2⁴⁰ | (E) | `rs:core/tests/invariance::offset_invariance` |
| LET order | one output hash for all 6 orders | (E) | `rs:rt/tests/let_order::permutation_invariant` |
| Replay | bit-identical in voltage mode | (E) | `rs:sim/tests/replay::bit_identical` |
| Selftest hash | identical native, with fault-injection and in wasm32; GOLDEN frozen by C1 | (E) | `rs:rt/tests/golden::selftest_hash_frozen`; `py:site::wasm_selftest_matches_golden` |

## §26 Precision notes and limitations

**Precision.**
- **Position.** Between 8 and 16 m, f32 values are 0.954 µm apart, which would make admittance integration at 0.025 m/s and 10 kHz (2.5 µm per tick) up to 19 % wrong per step; so all positions are `PosNm` differences first, then cast, and states integrate in `PosAccum`. The electrical angle uses an integer modulo, so it is exact at any travel.
- **Time.** `Tick(u64)` at 20 kHz; plant time is tick/20000 in f64, computed rather than accumulated. A segment's time since start is an integer tick difference cast once, so the controller is bit-identical under tick offsets up to 2⁴⁰ and more.
- **Observer rounding.** The levitation observer stores Φ − I so the O((pT)²) ≈ 5e-5 mechanical terms survive f32 rounding; the Pd chain subtracts the first sample (exact by Sterbenz) so the 0.075 Hz section carries mm/s²-scale states instead of 9.8 m/s² (with 1 g on z the f32 chain otherwise drifts ≈ 2 % from its gravity-free result).
- **libm.** Soft-float libm makes host, MCU and wasm32 agree bit for bit; the MCU's FPSCR must have flush-to-zero and default-NaN off (checked at boot). The monitor has no float arithmetic per tick.
- **Monitor quantisation.** Integer KE and momentum match float within one quantum; velocity within ½ µm/s over a ≥ 10 ms window, so a 1 µm/count encoder resolves 0.1 mm/s. Terminus arrival speeds (≤ 1 mm/s) are therefore asserted on `Truth.v_mps`, not on the encoder.

**Model limitations** (what the model does not capture, or captures with a stated simplification):
- **Magnetics.** A reluctance model with fringing and a tanh saturation knee at 0.7·Φ_sat \[UNCERTAIN\]; leakage is ideal by default and swept at 0.25 in studies; the rail flange's own saturation (83–96 % of B_sat at nominal flux on the minimum flange) is not modelled beyond the pole-face cap; the eddy lag is a first-order heuristic on the force flux. No FEM or magpylib cross-check exists (roadmap; [R46], [R47]). FAB-LEV-002 is S for these reasons.
- **Engagement.** The tiers use `rail::engagement`, whose face length and end softening are fixed at 2·g*(DRAWER) = 5 mm for every class; the pole-length form `rail::engagement_for` is not wired. On the fixed-rail classes e = 1 over the whole travel, so this matters only for the DRAWER extension moment, which the closed-form envelope (§10) bounds.
- **Friction numerics.** The bristle split update (§8) freezes z within a step, so the friction–body coupling is first order in h (the stick mode stays exact). Rail hysteresis uses a stateless tanh sign (creeps below 1 mm/s); the bristle form `rail_hyst_bristle` is not wired.
- **DRAWER cavity on Travel1d.** Travel1d integrates the cavity Δp inside RK4 at 1 ms although its linearised leak rate near Δp = 0 is ≈ 1.5e4 s⁻¹ (λh ≈ 15). The result is a numerically inflated cavity pressure: in a DRAWER soft_open at 0.1175 m/s Travel1d reports −19 Pa against the quasi-static −5.2 Pa (Carriage6 at 0.26 m/s reports −25.5 Pa, matching the quasi-static −25.4 Pa). The exact steady state `air::cavity_dp_quasi_static` exists but is not wired; Travel1d DRAWER cavity forces (and the μ_eq it contributes to) should be read with this bias until it is.
- **Obstacles.** Every obstacle is clamped; free-body (transient) contact is available in closed form and as `FreeBody` kinematics but not stepped by the tiers.
- **End stops.** The jamb, carcass and rail end stops of D3 are rigid penalty contacts capped for the step (≈ 5 mm penetration per m/s on the 1 ms tiers); their geometry (2 mm jamb clearance, bumper rated stroke) is a design choice, not a measured installation.
- **Coupled roll–sway on HEAVY.** The per-mode levitation design ignores the lateral-reluctance coupling of §6; HEAVY Carriage6 diverged at hover in Wave 1.5. The coupled design of D1 is verified on the linear coupled model; its closed-loop Carriage6 outcome is finalised in Wave 2.5. The model keeps the lateral reluctance linear (k_y = F/w at y = 0) and ignores yaw–roll coupling.
- **Current loop.** The motional-EMF feed-forward is not routed (the PI absorbs it); the bridges use the nominal bus voltage, not a measured one.
- **LSM.** A current-controlled FOC abstraction (τ_q lag) in SIL; the inverter's d/q PI runs only on firmware. Segment hand-over is a raised-cosine dip of the overlap fraction \[UNCERTAIN\]. ASC is the steady shorted-winding solution applied algebraically.
- **Ground motion.** The Boore S-phase amplitudes at 30 bar and κ 0.04 are lower than typical ground-motion models (≈ 0.04 g at M7 R20 against ≈ 0.2–0.3 g), and the Clough–Penzien medium-soil surrogate has a generous PGD (≈ 0.76 m against ≈ 0.1–0.2 m for El Centro 1940). Whenever S−P < 3 s (hypocentral R ≲ 25 km) the S arrival falls inside the 3 s Pd/τc window, which pushes some M5 R10 realisations to Level 2 (FAB-SEI-010 is S).
- **Sensors.** Gap-probe bias and tempco are implemented but default to 0 (no scenario field); the ADXL355 is modelled as white noise at the data-sheet density over the Nyquist band with a 20-bit code and ±2.048 g clip.
- **Hardware.** Nothing here has run on hardware: firmware timing, memory use and bit-identity on the chip are unmeasured, and dB(A), debris and FEM-validated forces are outside the model (FAB-FW-012, FAB-ENV-005). Standards are used only as numeric limits, "checked in simulation against numeric limits from [edition]"; no compliance or certification is claimed.
