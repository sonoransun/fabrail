//! Control tuning (core design §B.2, §C.4 as corrected by plan §2b, §4b). Values live in
//! `presets.rs`; gains are DERIVED at init by `derive` (these are design ratios and
//! bandwidths, not raw gains), so a mass or magnet change re-derives consistently.
//!
//! All physical fields are f64 SI with unit suffixes (plan §2b item 8); controllers cast
//! to f32 in `new()`.

use fabrail_types::SeismicAction;

/// Rate groups and coil mode (core design §A.5). Decimations are relative to the base.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct Rates {
    /// Base (PWM/current/plant) rate, Hz (20 000).
    pub base_hz: u32,
    /// Fast group decimation (2 → 10 kHz DRAWER/DOOR; 4 → 5 kHz WALL/HEAVY).
    pub fast_decim: u16,
    /// Supervisor decimation (20 → 1 kHz).
    pub sup_decim: u16,
    /// Seismic decimation (200 → 100 Hz).
    pub seismic_decim: u16,
    /// Voltage coil mode (current PI in the base task). SIL ≡ firmware is claimed in
    /// voltage mode only; the scenario is the single source and a mismatch is rejected
    /// (plan §2b item 16).
    pub coil_mode_voltage: bool,
}

/// Zero-power levitation design (PHYSICS §15; plan §2b items 1–2).
///
/// Mizuno current-integral law i* = k_p·δg + k_d·δġ + κ·q, q̇ = i − i_bias; target
/// polynomial (s² + 2ζωs + ω²)(s + αω) with ω = omega_ratio·max(p_mode, p_heave),
/// designed in discrete time on the ZOH plant augmented with the LET delay, the current
/// loop lag τ_cl and the nominal eddy lag τ_e (`derive::zp_gains_discrete`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct LevitationTuning {
    /// ω / p (2.5, plan §2b item 1).
    pub omega_ratio: f64,
    /// Target damping ratio ζ (0.7).
    pub zeta: f64,
    /// Integral pole ratio α (0.3).
    pub alpha: f64,
    /// Velocity observer bandwidth / ω (4).
    pub observer_ratio: f64,
    /// Nominal eddy lag used in the design τ_e,nom, s (stable for τ_e ≤ 2 ms, LEV-020).
    pub tau_e_nominal_s: f64,
    /// Current-loop bandwidth ω_c/2π (1.0 kHz → τ_cl = 0.159 ms; plan §2b item 2), Hz.
    pub current_bw_hz: f64,
    /// Roll actively controlled (DRAWER, HEAVY); gravity-stabilised on top-hung classes.
    pub roll_active: bool,
    /// Null-space (warp) current integrator bandwidth, Hz (LEV-009: ≤ 5 % within 1 s).
    pub warp_bw_hz: f64,
    /// Modified zero-power outer PI bandwidth (active outside the band only), Hz (1).
    pub zp_outer_bw_hz: f64,
    /// Bumpless transfer to zero-power when |δg| below this, m (50 µm).
    pub bumpless_dg_m: f64,
    /// Lift/unseat time limit (0.1 / 0.3 / 1 / 2 s; LEV-012), s.
    pub lift_time_max_s: f64,
    /// Landing speed (5 / 5 / 3 / 2 mm/s), m/s.
    pub land_speed_mps: f64,
    /// Unseat lift-off detection threshold (> 20 µm; F17), m.
    pub unseat_liftoff_m: f64,
    /// Unseat current ramp rate, A/s.
    pub unseat_ramp_a_per_s: f64,
    /// DRAWER z-seat step duration (+0.2 s after the terminus), s.
    pub seat_time_s: f64,
    /// Touchdown detection: |ġ| below this with a current step, m/s.
    pub touchdown_gdot_mps: f64,
    /// Payload estimator time constant (LEV-013: within ±5 % in ≤ 1 s), s.
    pub m_hat_tau_s: f64,
    /// Saturation persistence before SaturationPersist, s.
    pub sat_persist_s: f64,
}

/// Log-damped terminus (PHYSICS §16.1; plan §4b). τ ≥ τ_min = v_app/a_max − a_max/(2j)
/// is enforced by `validate`; at runtime τ_eff = max(τ, τ_min(v_measured)).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct TerminusTuning {
    /// Exponential time constant τ (0.15 / 0.25 / 0.6 / 1.0 s), s.
    pub tau_s: f64,
    /// Capture distance d_ε where the quintic takes over (≈ 0.2 mm), m.
    pub d_eps_m: f64,
    /// Blend jerk j, m/s³.
    pub j_mps3: f64,
    /// Quintic duration ratio T_q/τ (1).
    pub t_q_ratio: f64,
    /// Governor observation overtravel beyond the terminus (x_obs = terminus +
    /// overtravel, so the exponential tail is never clipped), m.
    pub overtravel_m: f64,
    /// Re-plan braking ratio a_brk/a_max after hand release inside d_0 (2).
    pub replan_brk_ratio: f64,
}

/// Inertial smoothing (PHYSICS §16.2; plan §4b). Research tier: `enable = false` in
/// every preset. Skyhook absolute-velocity feedback from the carriage IMU (leaky
/// integral, high-pass `hp_hz`), c_sky = 2ζ·m̂·ω_c.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct InertialTuning {
    /// Mode available (false in every preset).
    pub enable: bool,
    /// Crossover frequency f_c (≈ 1 Hz), Hz.
    pub f_c_hz: f64,
    /// Skyhook damping ratio ζ.
    pub zeta: f64,
    /// Leaky-integral high-pass on the IMU (0.1 Hz), Hz.
    pub hp_hz: f64,
    /// Nominal ride-through standoff from the seat, m.
    pub standoff_m: f64,
    /// Base of the admission opening s_min = 25 mm + S_max,pred, m.
    pub s_min_base_m: f64,
    /// Stroke margin: ∬HP(a_floor) ≤ standoff − margin, m.
    pub stroke_margin_m: f64,
    /// Admission: v_rel,pred ≤ v_rel_frac·v_cap (0.8).
    pub v_rel_frac: f64,
    /// Burst force allowance (90 N DRAWER), N.
    pub f_burst_n: f64,
    /// Burst duration limit (0.75 s), s.
    pub t_burst_s: f64,
    /// z-headroom admission a_z,pred ≤ frac·p²·(g_float − g_up) (0.5).
    pub z_headroom_frac: f64,
    /// Structure ADXL355 used only below this frequency (0.3 Hz), Hz.
    pub structure_lp_hz: f64,
    /// Sensing-to-actuation latency L for the 2πfL·a_HP error term, s.
    pub latency_s: f64,
}

/// Motion cascade, governor and termini (PHYSICS §16; core design §C.4).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct MotionTuning {
    /// Velocity-loop bandwidth ω_v/2π (8 / 6 / 3 / 2 Hz), Hz. K_v = m̂·ω_v.
    pub omega_v_hz: f64,
    /// Position gain ratio K_x/ω_v (0.25).
    pub kx_ratio: f64,
    /// Velocity integral ratio K_vi/(K_v·ω_v) (0.25).
    pub ki_ratio: f64,
    /// Acceleration limit a_max (1.5 / 0.75 / 0.15 / 0.08), m/s².
    pub a_max_mps2: f64,
    /// Acceleration limit with PL d presence sensing (HEAVY 0.2), m/s².
    pub a_max_sensed_mps2: f64,
    /// Jerk limit (20 / 5 / 1.5 / 1), m/s³.
    pub j_max_mps3: f64,
    /// Braking ratio a_brk/a_max (2).
    pub a_brk_ratio: f64,
    /// Governor reaction time in fast periods (2).
    pub t_react_fast_periods: u16,
    /// Governor stopping margin before x_obs, m.
    pub gov_margin_m: f64,
    /// Closed-position accuracy (±0.3 / 0.5 / 1 / 1 mm), m.
    pub pos_tol_closed_m: f64,
    /// Travel-position accuracy (±1 / 2 / 5 / 5 mm), m.
    pub pos_tol_travel_m: f64,
    /// Ripple look-up-table feed-forward enabled.
    pub ripple_ff: bool,
    /// Log-damped terminus.
    pub terminus: TerminusTuning,
    /// Inertial smoothing (research tier).
    pub inertial: InertialTuning,
}

/// Obstruction detection (PHYSICS §17; SAF-006; plan §4b seismic degradation).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct ObstructionTuning {
    /// Residual threshold th0 (12 / 30 / 60 / 120 N), N.
    pub residual_n: f64,
    /// Persistence (20 / 30 / 35 / 35 ms), s.
    pub persist_s: f64,
    /// Reversal distance after stop (50 mm DOOR/WALL; 0 = release to 0 N), m.
    pub reverse_m: f64,
    /// Release to 0 N instead of reversing (DRAWER/HEAVY).
    pub release_to_zero: bool,
    /// Shaking threshold gain: th = th0 + gain·m̂·|a_floor| (0.1).
    pub floor_gain: f64,
    /// Degraded when th > ratio·th0 (2) → ObstructionDegraded.
    pub degraded_ratio: f64,
}

/// Haptic admittance, effects and passivity (PHYSICS §17; core design §C.4).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct HapticTuning {
    /// Assist ratio m/M_v (1.5 / 3 / 5 / 5; HAP-002 bounds).
    pub mass_ratio: f64,
    /// Virtual damping B_v (10 / 40 / 450 / 2000), N·s/m.
    pub b_v_n_s_per_m: f64,
    /// Momentum-observer bandwidth K_O/2π (20 / 15 / 8 / 5 Hz), Hz.
    pub k_o_hz: f64,
    /// Virtual stiffness as a fraction of K_max = 2·b_dist/T_fast (0.3 in every preset:
    /// with the LET delay T_d = T the passive bound 2b/(T + 2T_d) is 1/3 of K_max; HAP-009).
    pub k_virtual_frac: f64,
    /// Passivity controller α_max / b_dist (5).
    pub passivity_alpha_ratio: f64,
    /// F̂ high-pass for pressure rejection (0.5 Hz), Hz.
    pub f_hp_hz: f64,
    /// Number of active detents (≤ 4).
    pub n_detents: u8,
    /// Detent positions from the closed stop, m.
    pub detent_x_m: [f64; 4],
    /// Detent peak force (3 / 8 / 20 N), N.
    pub detent_f_n: f64,
    /// Detent width (2 / 3 / 5 mm), m.
    pub detent_width_m: f64,
    /// Legacy soft-close ω_sc/2π (2 / 1.5 / 0.5 / 0.5 Hz), Hz.
    pub soft_close_omega_hz: f64,
    /// Soft-close capture distance (45 / 65 / 115 / 150 mm), m.
    pub soft_close_capture_m: f64,
    /// Soft-close final speed bound (20 / 50 / 20 / 10 mm/s; HAP-007), m/s.
    pub soft_close_v_final_mps: f64,
    /// Hand detection onset during a terminus capture (10 ms; 20 ms WALL), s.
    pub hand_onset_s: f64,
    /// Obstruction detection.
    pub obstruction: ObstructionTuning,
}

/// Intent (push-to-open, hand-guided start) and lean rejection (PHYSICS §17;
/// HAP-005/006; plan §2b item 11).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct IntentTuning {
    /// Force threshold (4.5 / 10 / 25 / 40 N), N.
    pub f_th_n: f64,
    /// Force persistence (50 / 80 / 120 / 150 ms), s.
    pub t_th_s: f64,
    /// Displacement alternative (1.5 / 2 / 3 mm), m.
    pub dx_th_m: f64,
    /// Lean detection: |v| below this … (2 mm/s), m/s.
    pub lean_v_mps: f64,
    /// … for longer than this feeds the bias estimator (1 s), s.
    pub lean_t_s: f64,
    /// Intent confirmation deadline (120 / 170 ms), s.
    pub confirm_max_s: f64,
    /// Require direction consistency (cabinet pressure pushes outward).
    pub dir_consistency: bool,
    /// Push-to-open from the seat enabled (DRAWER; HAP-012).
    pub push_to_open: bool,
}

/// On-site detector (PHYSICS §20; core design §C.4; plan §2b items 6–7).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct DetectorTuning {
    /// Anti-alias decimation from the ODR (5: 500 → 100 Hz).
    pub aa_decim: u16,
    /// Band-pass low edge (0.5 Hz), Hz.
    pub bp_lo_hz: f64,
    /// Band-pass high edge (20 Hz), Hz.
    pub bp_hi_hz: f64,
    /// Butterworth band-pass order (4).
    pub bp_order: u8,
    /// STA window, samples (50).
    pub sta_n: u32,
    /// LTA window, samples (3000).
    pub lta_n: u32,
    /// Trigger-on STA/LTA ratio (4.0).
    pub trig_on: f64,
    /// Trigger-off STA/LTA ratio (1.5).
    pub trig_off: f64,
    /// Pd/τc double-integration high-pass (0.075 Hz), Hz.
    pub pdtc_hp_hz: f64,
    /// Short Pd/τc window (1 s), s.
    pub win_short_s: f64,
    /// Long Pd/τc window (3 s), s.
    pub win_long_s: f64,
    /// τp_max smoothing α (0.99) \[M\].
    pub taup_alpha: f64,
    /// Z/H polarisation window (first 0.5 s), s.
    pub zh_window_s: f64,
    /// Z/H minimum ratio (> 1).
    pub zh_min: f64,
    /// Actuator veto unless PGA ≥ this (80 gal), m/s².
    pub pga_veto_mps2: f64,
    /// Level-3 τc threshold (0.6 s), s.
    pub tauc_l3_s: f64,
    /// Level-3 Pd threshold (0.5 cm), m.
    pub pd_l3_m: f64,
    /// PROTECT on Pd ≥ this (0.35 cm), m.
    pub pd_alert_m: f64,
    /// PROTECT on PGA ≥ this (80 gal), m/s².
    pub pga_alert_mps2: f64,
    /// PROTECT on level ≥ this (2); level 1 means freeze only.
    pub protect_level: u8,
}

/// Local/network alert fusion (PHYSICS §20).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct FusionTuning {
    /// P-wave speed (6.0 km/s), km/s.
    pub v_p_kmps: f64,
    /// S-wave speed (3.5 km/s), km/s.
    pub v_s_kmps: f64,
    /// QUIET dwell (30 s), s.
    pub quiet_s: f64,
    /// S-ETA uncertainty σ (1–2 s), s.
    pub eta_sigma_s: f64,
    /// Alert expiry without updates, s.
    pub alert_timeout_s: f64,
    /// Independent feeds required for alert-triggered motion without a local P pick (2).
    pub min_independent_feeds: u8,
}

/// Seismic class policy (PHYSICS §21; SEI-010/011; land-first).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct PolicyTuning {
    /// Class action in PROTECT (DRAWER seat/land, DOOR land, WALL/HEAVY land + lock).
    pub action: SeismicAction,
    /// Centre first, only if reachable before t_S (F11).
    pub center_if_reachable: bool,
    /// Engage the parking lock after landing (never on an egress path).
    pub lock_after_land: bool,
    /// Non-rated egress door must be stopped by t_S − margin (1 s), s.
    pub egress_stop_margin_s: f64,
    /// Use ASC during a seismic landing (F9).
    pub asc_on_seismic_land: bool,
}

/// Contactless isolation (research tier; PHYSICS §21). F = −k·x_rel − c_sky·ẋ_abs,
/// saturated at F_cont with a stroke governor; replaced limits per SAF-020.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct IsolationTuning {
    /// Mode available (false in every preset).
    pub enable: bool,
    /// Isolation period T = max(2 s, 1.1·T_min), T_min from F_cont (WALL 6.3 s), s.
    pub t_iso_s: f64,
    /// Skyhook damping ratio (0.3).
    pub zeta: f64,
    /// Design stroke (±0.2 m), m.
    pub stroke_m: f64,
    /// IMU leaky-integral high-pass (0.1 Hz), Hz.
    pub hp_hz: f64,
}

/// Active mass damping (research tier; SEI-016/021). Gains injected from Python
/// (scipy CARE); direct velocity feedback fallback.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct AmdTuning {
    /// Mode available (false in every preset).
    pub enable: bool,
    /// Direct velocity feedback gain, N·s/m.
    pub c_dvf_n_s_per_m: f64,
    /// LQR state-feedback gains on [x_s, ẋ_s, x_r, ẋ_r] (SI).
    pub k_lqr: [f64; 4],
    /// Force limit, N.
    pub f_max_n: f64,
}

/// Seismic subsystem tuning.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct SeismicTuning {
    /// On-site detector.
    pub detector: DetectorTuning,
    /// Alert fusion.
    pub fusion: FusionTuning,
    /// Class policy.
    pub policy: PolicyTuning,
    /// Isolation (research, disabled).
    pub isolation: IsolationTuning,
    /// Active mass damping (research, disabled).
    pub amd: AmdTuning,
}

/// Supervisor (PHYSICS §18; core design §C.4).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct SupervisorTuning {
    /// Watchdog period (5 ms), s.
    pub watchdog_s: f64,
    /// Heartbeat limit in periods (3).
    pub heartbeat_periods: u8,
    /// PayloadMismatch when m̂ > ratio·m_rated (1.05; F18).
    pub payload_mismatch_ratio: f64,
    /// Governor mass m_gov = max(m_rated, ratio·m̂) (1.05; F19).
    pub m_gov_ratio: f64,
    /// Hold-up margin: landing energy × margin ≤ supercap energy (3; SAF-009).
    pub holdup_margin: f64,
    /// Stall detection speed (2 mm/s), m/s.
    pub stall_v_mps: f64,
    /// Stall detection time (0.2 s), s.
    pub stall_t_s: f64,
    /// Open-hover limit on the supercap (DRAWER ≈ 5 min; plan §2b item 4), s.
    pub hover_timeout_s: f64,
    /// Lift retries before FAULT.
    pub lift_retries: u8,
    /// Telemetry frame decimation relative to the supervisor rate (1 → 1 kHz).
    pub telemetry_decim: u16,
    /// Restore after INSPECT requires a local command (never remote-only; plan §4b).
    pub restore_local_only: bool,
}

/// Top-level control block.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct ControlParams {
    /// Rate groups and coil mode.
    pub rates: Rates,
    /// Zero-power levitation.
    pub levitation: LevitationTuning,
    /// Motion cascade, governor, termini, inertial smoothing.
    pub motion: MotionTuning,
    /// Haptics and obstruction.
    pub haptic: HapticTuning,
    /// Intent and lean rejection.
    pub intent: IntentTuning,
    /// Seismic detector, fusion, policy, isolation, AMD.
    pub seismic: SeismicTuning,
    /// Supervisor.
    pub supervisor: SupervisorTuning,
}
