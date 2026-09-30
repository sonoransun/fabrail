"""Analysis: runs and ensembles reduced to the quantities the figures and tests use (core §B.8).

This is the plotting boundary: :mod:`fabrail.visualization` consumes only these
dataclasses, never raw runs, so every number a figure draws is a field here and the
JSON record of a run (``fabrail.run/1`` plus its ``analysis`` block) contains it
(FAB-SW-009). The dataclass fields are frozen in Wave 0 (additions are allowed,
removals and renames are not); every field is SI with a unit suffix, and scalars
default to NaN until computed. NaN means "not measured in this run" (the phase never
happened, or the channel was not recorded), never zero.

What each reducer computes:

- :func:`analyze_levitation`: gaps and coil currents per element, their mean, the
  Welch PSD of the mean gap over the longest levitated stretch (scipy.signal.welch,
  Hann, 50 % overlap, linear detrend), and the lift / unseat / land KPIs of the run.
- :func:`analyze_haptics`: position, speed, true and estimated hand force, the
  apparent mass from a least-squares fit F_hand = M·a + B·v over the samples in
  ``Admittance`` with a hand on (the FAB-HAP-001 definition, core §B.6), and the
  log-damped terminus (PHYSICS §16.1): distance-to-go d = sign·(x_target − x) from the
  capture event, and a semilog least-squares fit of ln d over d_ε < d ≤ d_b (the
  exponential capture; floor |a_x| < 0.01 g, plan §4b).
- :func:`analyze_safety`: governor kinetic energy against the cap, contact force and
  the directional force cap, the monitor demand timeline, faults and trips, and an
  energy ledger (bus energy drawn, truth kinetic energy, drag dissipation, hold-up
  minimum) when the ledger channels were recorded.
- :func:`analyze_seismic`: floor acceleration, detector level and the seismic and
  levitation state timelines against t_P and t_S (from the scenario synthesis
  summary, :func:`fabrail.scenarios.info`); :func:`analyze_detection` does the same
  for a ``DetectorReplay`` of a record (``fabrail detect``).
- :func:`analyze_ensemble`: KPI statistics (P50/P95) and threshold exceedances with
  the Wilson 95 % upper bound (tolerance class MC).
- :func:`compute_anchors`: the seed-42 acceptance anchors of core §E.5
  (``fabrail anchors``; ``tests/data/anchors_seed42.json``).

Plot decimation: time series are kept at every ``stride``-th sample plus the argmin
and argmax of every plotted channel (so peaks survive), capped near ``max_points``;
the plotted values are real samples, and the scalar KPIs always come from the
full-rate run (or the Rust online accumulators at the base rate).

State and fault names mirror the ``u8_enum`` registries in
``crates/fabrail-types/src/{state,fault}.rs`` (drift-checked by tests/test_cli.py).

Owner: W0-C2 (dataclasses frozen; ``wilson_upper`` complete), then B1 (Wave 2).
"""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .results import EnsembleResult, RunEvent, RunResult

__all__ = [
    "ANALYSIS_EXTRA_CHANNELS",
    "ANALYSIS_KINDS",
    "DEFAULT_MAX_POINTS",
    "DEMAND_NAMES",
    "FAULT_NAMES",
    "FAULT_SEVERITY",
    "LEVEL_NAMES",
    "LEV_STATE_NAMES",
    "MOTION_MODE_NAMES",
    "SEISMIC_STATE_NAMES",
    "SEVERITY_NAMES",
    "STATE_NAMES",
    "EnsembleAnalysis",
    "HapticAnalysis",
    "LevitationAnalysis",
    "SafetyAnalysis",
    "SeismicAnalysis",
    "analysis_channel_names",
    "analysis_from_jsonable",
    "analyze_detection",
    "analyze_ensemble",
    "analyze_haptics",
    "analyze_levitation",
    "analyze_run",
    "analyze_safety",
    "analyze_seismic",
    "compute_anchors",
    "default_channel_names",
    "fault_name",
    "fault_severity",
    "run_meta",
    "state_name",
    "wilson_upper",
]

_NAN = math.nan

#: Default cap on plotted samples per time series (see the module docstring).
DEFAULT_MAX_POINTS = 4000

#: Standard gravity, m/s² (CGPM 1901; PHYSICS §1).
_G0_MPS2 = 9.80665

#: Terminus capture ends at d_ε = 0.2 mm (plan §4b; PHYSICS §16.1).
TERMINUS_D_EPS_M = 2.0e-4

#: Terminus arrival: distance-to-go ≤ 0.02 mm (plan §4b; fabrail-sim metrics TERMINUS_ARRIVE_TOL_M).
TERMINUS_ARRIVE_M = 2.0e-5

#: Terminus criteria hold only while |a_floor,x| < 0.01 g (plan §4b).
TERMINUS_QUIET_MPS2 = 0.01 * _G0_MPS2

# ---- state and fault registries (crates/fabrail-types/src/state.rs, fault.rs) ------------

#: ``LevState`` (supervisor state; channel ``lev_state``).
LEV_STATE_NAMES: tuple[str, ...] = (
    "Boot",
    "SelfTest",
    "Seated",
    "Landed",
    "Lifting",
    "LevitatedIdle",
    "Moving",
    "Stopping",
    "Landing",
    "Seating",
    "FaultReaction",
    "Fault",
    "Seismic",
    "Escape",
    "Manual",
)
#: ``SeismicState`` (channel ``seismic_state``).
SEISMIC_STATE_NAMES: tuple[str, ...] = ("Normal", "Triggered", "Protect", "Shaking", "Quiet", "Inspect")
#: ``MotionMode`` (channel ``motion_mode``).
MOTION_MODE_NAMES: tuple[str, ...] = (
    "Off",
    "Hold",
    "Trajectory",
    "Admittance",
    "SoftClose",
    "Reverse",
    "Freeze",
    "Center",
    "Isolate",
    "Damp",
    "Terminus",
    "SoftOpen",
    "InertialSmooth",
)
#: ``SafetyDemand`` (monitor verdict; channel ``demand``).
DEMAND_NAMES: tuple[str, ...] = ("None", "LimitForce", "SafeStop", "SafeLand", "StoLsm", "StoLsmAsc")
#: ``SeismicLevel`` (channel ``level``).
LEVEL_NAMES: tuple[str, ...] = ("L0", "L1", "L2", "L3")
#: ``Severity`` in order.
SEVERITY_NAMES: tuple[str, ...] = ("Info", "Warn", "Degrade", "SafeStop", "SafeLand", "Sto")

#: State-channel name tables by channel (and by the event kind that reports changes).
STATE_NAMES: dict[str, tuple[str, ...]] = {
    "lev_state": LEV_STATE_NAMES,
    "seismic_state": SEISMIC_STATE_NAMES,
    "motion_mode": MOTION_MODE_NAMES,
    "demand": DEMAND_NAMES,
    "level": LEVEL_NAMES,
}
_EVENT_STATE_CHANNEL = {"LevState": "lev_state", "SeismicState": "seismic_state", "Mode": "motion_mode", "Demand": "demand"}

#: ``FaultCode`` → (name, severity), the wire codes of fabrail_types::FaultCode.
_FAULTS: dict[int, tuple[str, str]] = {
    0x0101: ("GapRange", "SafeLand"),
    0x0102: ("GapStuck", "SafeLand"),
    0x0103: ("GapDisagree", "Degrade"),
    0x0110: ("EncoderLost", "SafeStop"),
    0x0111: ("EncoderJump", "SafeStop"),
    0x0120: ("AccelLost", "Warn"),
    0x0121: ("AccelStuck", "Warn"),
    0x0122: ("ImuLost", "Degrade"),
    0x0123: ("RailAccelLost", "Degrade"),
    0x0130: ("CurrentSense", "SafeLand"),
    0x0201: ("CoilOvercurrent", "SafeLand"),
    0x0202: ("CoilOpen", "SafeLand"),
    0x0203: ("CoilDriver", "SafeLand"),
    0x0204: ("CoilI2t", "SafeLand"),
    0x0210: ("LsmOvercurrent", "Sto"),
    0x0211: ("LsmPhaseSum", "Sto"),
    0x0212: ("LsmDriver", "Sto"),
    0x0220: ("LockFailed", "Warn"),
    0x0221: ("LatchFailed", "Warn"),
    0x0301: ("BusUnder", "SafeLand"),
    0x0302: ("BusOver", "SafeStop"),
    0x0303: ("MainsLost", "SafeLand"),
    0x0304: ("HoldupLow", "Degrade"),
    0x0310: ("OverTemp", "SafeLand"),
    0x0401: ("FastOverrun", "SafeLand"),
    0x0402: ("SupervisorOverrun", "SafeLand"),
    0x0403: ("WatchdogReset", "SafeLand"),
    0x0404: ("ParamCrc", "SafeLand"),
    0x0405: ("ParamRange", "SafeLand"),
    0x0406: ("SelftestHash", "SafeLand"),
    0x0407: ("NonFinite", "SafeLand"),
    0x0408: ("HeartbeatLost", "Sto"),
    0x0409: ("SeismicOverrun", "Warn"),
    0x0501: ("GapEnvelope", "SafeLand"),
    0x0502: ("LiftTimeout", "SafeLand"),
    0x0503: ("LandTimeout", "SafeStop"),
    0x0504: ("PayloadMismatch", "Degrade"),
    0x0505: ("SaturationPersist", "SafeLand"),
    0x0506: ("SeatFailed", "SafeLand"),
    0x0507: ("NotReferenced", "Degrade"),
    0x0601: ("EnergyLimit", "SafeStop"),
    0x0602: ("MomentumLimit", "SafeStop"),
    0x0603: ("ForceLimit", "SafeStop"),
    0x0604: ("Overspeed", "Sto"),
    0x0605: ("PositionLimit", "SafeStop"),
    0x0606: ("UnintendedMotion", "Sto"),
    0x0607: ("Obstruction", "Warn"),
    0x0608: ("PassivityViolation", "Degrade"),
    0x0609: ("NotHoldToRun", "Warn"),
    0x060A: ("ObstructionDegraded", "Degrade"),
    0x060B: ("IsolateLimit", "SafeStop"),
    0x0701: ("VerticalSaturation", "SafeLand"),
    0x0702: ("CenterUnreachable", "Warn"),
}
#: Fault code → name.
FAULT_NAMES: dict[int, str] = {code: name for code, (name, _) in _FAULTS.items()}
#: Fault code → severity name.
FAULT_SEVERITY: dict[int, str] = {code: sev for code, (_, sev) in _FAULTS.items()}


def state_name(channel: str, code: float | int) -> str:
    """The name of state ``code`` on ``channel`` (``lev_state``, ``motion_mode``, …); ``"?<code>"`` if unknown."""
    names = STATE_NAMES.get(channel, ())
    i = int(code)
    return names[i] if 0 <= i < len(names) else f"?{i}"


def fault_name(code: float | int) -> str:
    """The ``FaultCode`` name of a wire code (e.g. 0x0607 → ``"Obstruction"``); hex if unknown."""
    i = int(code)
    return FAULT_NAMES.get(i, f"0x{i:04X}")


def fault_severity(code: float | int) -> str:
    """The severity name of a fault code (``"Warn"`` … ``"Sto"``); ``"?"`` if unknown."""
    return FAULT_SEVERITY.get(int(code), "?")


# ---- recorded channels -----------------------------------------------------------------------

#: Extra recorder channels the CLI requests on top of the default set so the analyses
#: can draw the reference, the energy ledger and the detector estimates
#: (``fabrail_sim::recorder`` EXTRA_CHANNELS).
ANALYSIS_EXTRA_CHANNELS: tuple[str, ...] = (
    "x_ref_m",
    "v_ref_mps",
    "e_kin_j",
    "e_drag_j",
    "e_hold_j",
    "f_bumper_n",
    "pd_m",
    "tau_c_s",
)


def default_channel_names(n_magnets: int) -> list[str]:
    """The recorder's default channel list for ``n_magnets`` elements (``fabrail_sim::recorder::default_channels``)."""
    n = max(0, int(n_magnets))
    return [
        "x_m",
        "v_mps",
        "a_mps2",
        "y_m",
        "z_m",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        *(f"gap_{i}_m" for i in range(n)),
        *(f"coil_{i}_a" for i in range(n)),
        "iq_a",
        "f_lsm_n",
        "f_hand_n",
        "f_obstacle_n",
        "f_upper_n",
        "f_rollers_n",
        "floor_ax_mps2",
        "floor_ay_mps2",
        "floor_az_mps2",
        "t_coil_c",
        "t_mag_c",
        "cavity_dp_pa",
        "v_bus_v",
        "v_hold_v",
        "p_bus_w",
        "x_meas_m",
        "v_hat_mps",
        "f_hand_hat_n",
        "residual_n",
        "m_hat_kg",
        "e_kin_gov_j",
        "e_passivity_j",
        "force_cap_n",
        "lev_state",
        "seismic_state",
        "motion_mode",
        "demand",
        "level",
        "sat_flags",
        "intent",
        "obstruction",
    ]


def analysis_channel_names(n_magnets: int) -> list[str]:
    """Default channels plus :data:`ANALYSIS_EXTRA_CHANNELS` (the CLI's ``--channels analysis``)."""
    return [*default_channel_names(n_magnets), *ANALYSIS_EXTRA_CHANNELS]


# ---- dataclasses (frozen fields; additions only) -------------------------------------------


def _empty() -> np.ndarray:
    return np.zeros(0, dtype=np.float64)


def _empty_u8() -> np.ndarray:
    return np.zeros(0, dtype=np.uint8)


@dataclass
class LevitationAnalysis:
    """Suspension behaviour of one run (PHYSICS §3, §15; FAB-LEV)."""

    rail_class: str = ""
    scenario: str = ""
    t_s: np.ndarray = field(default_factory=_empty)
    gap_m: np.ndarray = field(default_factory=_empty)  # (n_magnets, n)
    coil_i_a: np.ndarray = field(default_factory=_empty)  # (n_magnets, n)
    m_hat_kg_t: np.ndarray = field(default_factory=_empty)
    gap_mean_m: float = _NAN
    gap_rms_m: float = _NAN
    gap_dev_max_m: float = _NAN
    coil_i_ss_a: float = _NAN
    hover_power_w: float = _NAN
    lift_time_s: float = _NAN
    unseat_time_s: float = _NAN
    land_time_s: float = _NAN
    touchdown_v_mps: float = _NAN
    m_hat_kg: float = _NAN
    pole_fit_rad_s: float = _NAN
    psd_f_hz: np.ndarray = field(default_factory=_empty)
    gap_psd_m2_per_hz: np.ndarray = field(default_factory=_empty)
    meta: dict[str, Any] = field(default_factory=dict)
    # ---- B1 additions (Wave 2) ----
    gap_mean_t_m: np.ndarray = field(default_factory=_empty)  # mean over elements per plotted sample
    lev_state: np.ndarray = field(default_factory=_empty_u8)
    gap_limits_m: list[float] = field(default_factory=list)  # [safety.gap_min_m, safety.gap_max_m]
    g_star_m: float = _NAN  # design zero-power gap at m*
    m_star_kg: float = _NAN
    coil_i_pulse_a: float = _NAN  # safety.i_coil_pulse_a
    psd_window_s: list[float] = field(default_factory=list)  # [t0, t1] of the PSD stretch


@dataclass
class HapticAnalysis:
    """Motion feel of one run: admittance, intent, soft close, terminus (PHYSICS §16, §17; FAB-HAP)."""

    rail_class: str = ""
    scenario: str = ""
    t_s: np.ndarray = field(default_factory=_empty)
    x_m: np.ndarray = field(default_factory=_empty)
    v_mps: np.ndarray = field(default_factory=_empty)
    f_hand_n: np.ndarray = field(default_factory=_empty)
    f_hand_hat_n: np.ndarray = field(default_factory=_empty)
    breakaway_n: float = _NAN
    apparent_mass_kg: float = _NAN
    apparent_mass_err: float = _NAN
    intent_onset_s: float = _NAN
    intent_confirm_s: float = _NAN
    reverse_latency_s: float = _NAN
    soft_close_v_final_mps: float = _NAN
    overshoot_m: float = _NAN
    detent_peak_err: float = _NAN
    passivity_min_j: float = _NAN
    force_noise_rms_n: float = _NAN
    terminus_tau_fit_s: float = _NAN
    terminus_semilog_r2: float = _NAN
    t_arrive_s: float = _NAN
    arrival_v_mps: float = _NAN
    meta: dict[str, Any] = field(default_factory=dict)
    # ---- B1 additions (Wave 2) ----
    x_ref_m: np.ndarray = field(default_factory=_empty)  # empty unless x_ref_m was recorded
    motion_mode: np.ndarray = field(default_factory=_empty_u8)
    m_virtual_kg: float = _NAN  # M_v = m*/control.haptic.mass_ratio
    v_open_cap_mps: float = _NAN  # safety.v_open_max_mps
    v_close_cap_mps: float = _NAN  # safety.v_close_max_mps
    v_cap_band_mps: list[float] = field(default_factory=list)  # [−v_close_cap, +v_open_cap] as drawn
    terminus_count: int = 0
    terminus_t_capture_s: float = _NAN  # TerminusCapture event time of the analysed terminus
    terminus_target_m: float = _NAN
    terminus_d_b_m: float = _NAN  # event payload d_b
    terminus_tau_eff_s: float = _NAN  # event payload τ_eff
    terminus_t_s: np.ndarray = field(default_factory=_empty)  # time since capture, s
    terminus_d_m: np.ndarray = field(default_factory=_empty)  # distance-to-go (> 0 while approaching)
    terminus_fit_t_s: np.ndarray = field(default_factory=_empty)  # fit-window end points (time since capture)
    terminus_fit_d_m: np.ndarray = field(default_factory=_empty)  # fitted d at those points
    terminus_fit_slope_per_s: float = _NAN  # d ln d / dt of the recorded-sample fit
    terminus_fit_r2: float = _NAN  # R² of the recorded-sample fit
    terminus_fit_source: str = ""  # "kpi" (base-rate Rust accumulator) or "recorded" (this fit)


@dataclass
class SafetyAnalysis:
    """Energy, momentum, force and monitor behaviour of one run (PHYSICS §18; FAB-SAF)."""

    rail_class: str = ""
    scenario: str = ""
    t_s: np.ndarray = field(default_factory=_empty)
    e_kin_gov_j: np.ndarray = field(default_factory=_empty)
    contact_force_n: np.ndarray = field(default_factory=_empty)
    demand: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.uint8))
    ke_max_gov_j: float = _NAN
    momentum_max_kgmps: float = _NAN
    contact_force_max_n: float = _NAN
    guide_force_max_n: float = _NAN
    manual_force_n: float = _NAN
    bumper_impacts: int = 0
    bumper_energy_j: float = _NAN
    reverse_latency_s: float = _NAN
    faults: list[str] = field(default_factory=list)
    trips: list[tuple[float, str]] = field(default_factory=list)
    meta: dict[str, Any] = field(default_factory=dict)
    # ---- B1 additions (Wave 2) ----
    force_cap_n: np.ndarray = field(default_factory=_empty)  # the monitor verdict's force cap
    ke_cap_j: float = _NAN  # safety.e_max_j
    momentum_cap_kgmps: float = _NAN  # safety.p_max_kgmps (NaN where the class has none: HEAVY hold-to-run)
    fault_events: list[tuple[float, str, str]] = field(default_factory=list)  # (t, name, severity)
    e_bus_j: np.ndarray = field(default_factory=_empty)  # cumulative ∫ p_bus dt
    e_kin_j: np.ndarray = field(default_factory=_empty)  # truth kinetic energy (if recorded)
    e_drag_j: np.ndarray = field(default_factory=_empty)  # cumulative drag dissipation (if recorded)
    energy_ledger_j: dict[str, float] = field(default_factory=dict)


@dataclass
class SeismicAnalysis:
    """Detection, fusion and the class action timeline of one run (PHYSICS §19–§21; FAB-SEI)."""

    rail_class: str = ""
    scenario: str = ""
    t_s: np.ndarray = field(default_factory=_empty)
    floor_acc_mps2: np.ndarray = field(default_factory=_empty)  # (3, n)
    level: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.uint8))
    seismic_state: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.uint8))
    lev_state: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.uint8))
    t_p_s: float = _NAN
    t_s_arrival_s: float = _NAN
    trigger_t_s: float = _NAN
    freeze_t_s: float = _NAN
    protect_t_s: float = _NAN
    land_t_s: float = _NAN
    level_max: int = 0
    protect_margin_to_s_s: float = _NAN
    pd_m: float = _NAN
    tau_c_s: float = _NAN
    stroke_max_m: float = _NAN
    timeline: list[tuple[float, str]] = field(default_factory=list)
    meta: dict[str, Any] = field(default_factory=dict)
    # ---- B1 additions (Wave 2) ----
    lock_t_s: float = _NAN  # truth parking lock engaged (plant:locked)
    alert_t_s: float = _NAN  # synthetic/injected network alert (never a live EEW feed)
    alert_cancel_t_s: float = _NAN
    pga_mps2: float = _NAN  # max horizontal |a_floor| of the run
    info: dict[str, Any] = field(default_factory=dict)  # scenario synthesis summary (magnitude, R, Pd target, …)


@dataclass
class EnsembleAnalysis:
    """KPI statistics over an ensemble; Monte Carlo gates use the Wilson 95 % upper bound (tolerance class MC)."""

    scenario: str = ""
    n: int = 0
    seed: int = 0
    kpis: dict[str, np.ndarray] = field(default_factory=dict)
    p50: dict[str, float] = field(default_factory=dict)
    p95: dict[str, float] = field(default_factory=dict)
    exceedances: dict[str, int] = field(default_factory=dict)
    wilson_upper: dict[str, float] = field(default_factory=dict)
    meta: dict[str, Any] = field(default_factory=dict)
    # ---- B1 additions (Wave 2) ----
    thresholds: dict[str, float] = field(default_factory=dict)
    n_reporting: dict[str, int] = field(default_factory=dict)
    member_index: np.ndarray = field(default_factory=_empty)


#: Analysis kinds by name (CLI JSON ``analysis`` block keys).
ANALYSIS_KINDS: dict[str, type] = {
    "levitation": LevitationAnalysis,
    "haptics": HapticAnalysis,
    "safety": SafetyAnalysis,
    "seismic": SeismicAnalysis,
    "ensemble": EnsembleAnalysis,
}


def wilson_upper(k: int, n: int, z: float = 1.959963984540054) -> float:
    """Upper Wilson score bound for ``k`` successes in ``n`` trials (default 95 % two-sided z).

    (p̂ + z²/2n + z·√(p̂(1 − p̂)/n + z²/4n²)) / (1 + z²/n), with p̂ = k/n (Wilson 1927).
    """
    if n <= 0:
        raise ValueError(f"wilson_upper: n must be positive (got {n})")
    if not 0 <= k <= n:
        raise ValueError(f"wilson_upper: need 0 <= k <= n (got k={k}, n={n})")
    p = k / n
    z2 = z * z
    centre = p + z2 / (2 * n)
    spread = z * math.sqrt(p * (1.0 - p) / n + z2 / (4.0 * n * n))
    return min(1.0, (centre + spread) / (1.0 + z2 / n))


# ---- helpers -----------------------------------------------------------------------------------

#: RunMeta keys copied into every analysis ``meta`` (the run stamp; params/spec stay in the run record).
_STAMP_KEYS = ("version", "noise_seed", "fidelity", "coil", "golden", "build_profile")


def _param(result: RunResult, path: str, default: Any = _NAN) -> Any:
    """A parameter leaf from the run's recorded parameter set (``meta["params"]``)."""
    node: Any = result.meta.get("params")
    for part in path.split("."):
        if isinstance(node, Mapping) and part in node:
            node = node[part]
        elif isinstance(node, list) and part.isdigit() and int(part) < len(node):
            node = node[int(part)]
        else:
            return default
    return node


def _num(value: Any) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return _NAN
    return out


def _rail_class(result: RunResult) -> str:
    cls = _param(result, "design.class", None)
    if isinstance(cls, str) and cls:
        return cls.lower()
    name = str(result.meta.get("scenario") or "")
    head = name.split("_", 1)[0]
    return head if head in ("drawer", "door", "wall", "heavy") else ""


def _scenario_name(result: RunResult, rail_class: str) -> str:
    name = str(result.meta.get("scenario") or "")
    prefix = f"{rail_class}_"
    return name[len(prefix) :] if rail_class and name.startswith(prefix) else name


def run_meta(result: RunResult, extra: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """The analysis/plot stamp of a run: version, class, scenario, seeds, tier, coil, golden hash.

    ``extra`` (e.g. ``{"seed": 42}`` from the CLI) is merged last. ``params`` and ``spec``
    are not copied (they stay in the run record).
    """
    cls = _rail_class(result)
    meta: dict[str, Any] = {k: result.meta[k] for k in _STAMP_KEYS if k in result.meta}
    meta["rail_class"] = cls
    meta["scenario"] = _scenario_name(result, cls)
    meta["spec_name"] = str(result.meta.get("scenario") or "")
    meta["rate_hz"] = float(result.rate_hz)
    meta["n_samples"] = int(result.n_samples)
    if extra:
        meta.update(dict(extra))
    return meta


def _kpi(result: RunResult, name: str) -> float:
    return float(result.kpis.get(name, _NAN))


def _channel(result: RunResult, name: str) -> np.ndarray | None:
    arr = result.channels.get(name)
    if arr is None or np.asarray(arr).shape[0] != result.n_samples:
        return None
    return np.asarray(arr)


def _indexed_channels(result: RunResult, prefix: str, suffix: str) -> list[np.ndarray]:
    found: list[tuple[int, np.ndarray]] = []
    for name in result.channels:
        if name.startswith(prefix) and name.endswith(suffix):
            mid = name[len(prefix) : len(name) - len(suffix)]
            arr = _channel(result, name)
            if mid.isdigit() and arr is not None:
                found.append((int(mid), np.asarray(arr, dtype=np.float64)))
    return [a for _, a in sorted(found, key=lambda kv: kv[0])]


def _plot_index(n: int, max_points: int, arrays: Iterable[np.ndarray]) -> np.ndarray:
    """Sample indices to plot: every stride-th sample, the last one, and each array's argmin/argmax."""
    if n <= 0:
        return np.zeros(0, dtype=np.int64)
    if max_points <= 0 or n <= max_points:
        return np.arange(n, dtype=np.int64)
    stride = int(math.ceil(n / max_points))
    keep = [np.arange(0, n, stride, dtype=np.int64), np.array([n - 1], dtype=np.int64)]
    for a in arrays:
        x = np.asarray(a, dtype=np.float64)
        if x.ndim != 1 or x.shape[0] != n:
            continue
        finite = np.isfinite(x)
        if not finite.any():
            continue
        masked = np.where(finite, x, np.nan)
        keep.append(np.array([int(np.nanargmin(masked)), int(np.nanargmax(masked))], dtype=np.int64))
    return np.unique(np.concatenate(keep))


def _first_event(events: Sequence[RunEvent], kind: str, after: float = -math.inf) -> RunEvent | None:
    for e in events:
        if e.kind == kind and e.t_s >= after:
            return e
    return None


def _sorted_events(result: RunResult) -> list[RunEvent]:
    return sorted(result.events, key=lambda e: e.t_s)


def _longest_true_run(mask: np.ndarray) -> tuple[int, int]:
    """(start, stop) of the longest run of True in ``mask`` (stop exclusive); (0, 0) if none."""
    if mask.size == 0 or not mask.any():
        return 0, 0
    padded = np.concatenate(([False], mask.astype(bool), [False]))
    edges = np.flatnonzero(np.diff(padded.astype(np.int8)))
    starts, stops = edges[0::2], edges[1::2]
    k = int(np.argmax(stops - starts))
    return int(starts[k]), int(stops[k])


def _welch(x: np.ndarray, fs_hz: float) -> tuple[np.ndarray, np.ndarray]:
    """One-sided Welch PSD (Hann, 50 % overlap, linear detrend); the f = 0 bin is dropped."""
    from scipy.signal import welch

    n = int(x.shape[0])
    if n < 64 or not np.all(np.isfinite(x)):
        return _empty(), _empty()
    nperseg = int(min(n, 2 ** int(math.floor(math.log2(min(n, max(64.0, 2.0 * fs_hz)))))))
    f, pxx = welch(x, fs=fs_hz, window="hann", nperseg=nperseg, noverlap=nperseg // 2, detrend="linear", scaling="density")
    return np.asarray(f[1:], dtype=np.float64), np.asarray(pxx[1:], dtype=np.float64)


#: Levitated supervisor states for the gap statistics (LevitatedIdle, Moving, Stopping).
_LEVITATED = (5, 6, 7)


# ---- levitation --------------------------------------------------------------------------------


def analyze_levitation(
    result: RunResult, *, max_points: int = DEFAULT_MAX_POINTS, meta: Mapping[str, Any] | None = None
) -> LevitationAnalysis:
    """Reduce a run to :class:`LevitationAnalysis` (Welch PSD of the gap, lift/land timings)."""
    cls = _rail_class(result)
    stamp = run_meta(result, meta)
    gaps = _indexed_channels(result, "gap_", "_m")
    coils = _indexed_channels(result, "coil_", "_a")
    n = result.n_samples
    gap = np.vstack(gaps) if gaps else np.zeros((0, n))
    coil = np.vstack(coils) if coils else np.zeros((0, n))
    gap_mean_t = gap.mean(axis=0) if gaps else _empty()
    m_hat = _channel(result, "m_hat_kg")
    lev = _channel(result, "lev_state")
    idx = _plot_index(n, max_points, [*gaps, *coils, *([m_hat] if m_hat is not None else [])])

    out = LevitationAnalysis(rail_class=cls, scenario=stamp["scenario"], meta=stamp)
    out.t_s = result.t_s[idx].copy() if n else _empty()
    out.gap_m = gap[:, idx].copy() if gaps else np.zeros((0, 0))
    out.coil_i_a = coil[:, idx].copy() if coils else np.zeros((0, 0))
    out.gap_mean_t_m = gap_mean_t[idx].copy() if gaps else _empty()
    out.m_hat_kg_t = np.asarray(m_hat, dtype=np.float64)[idx].copy() if m_hat is not None else _empty()
    out.lev_state = np.asarray(lev, dtype=np.uint8)[idx].copy() if lev is not None else _empty_u8()

    out.gap_dev_max_m = _kpi(result, "gap_dev_max_m")
    out.coil_i_ss_a = _kpi(result, "coil_i_ss_a")
    out.hover_power_w = _kpi(result, "hover_power_w")
    out.lift_time_s = _kpi(result, "lift_time_s")
    out.unseat_time_s = _kpi(result, "unseat_time_s")
    out.land_time_s = _kpi(result, "land_time_s")
    out.touchdown_v_mps = _kpi(result, "touchdown_v_mps")
    out.pole_fit_rad_s = _kpi(result, "pole_fit_rad_s")
    events = _sorted_events(result)
    if math.isnan(out.lift_time_s):
        lifted = _first_event(events, "Lifted")
        if lifted is not None:
            out.lift_time_s = float(lifted.a)
    if math.isnan(out.touchdown_v_mps):
        landed = _first_event(events, "Landed")
        if landed is not None:
            out.touchdown_v_mps = float(landed.a)

    # Gap statistics and PSD over the longest levitated stretch of the full-rate record.
    if gaps and lev is not None:
        start, stop = _longest_true_run(np.isin(lev, _LEVITATED))
        seg = gap_mean_t[start:stop]
        if seg.size:
            out.psd_window_s = [float(result.t_s[start]), float(result.t_s[stop - 1])]
            out.psd_f_hz, out.gap_psd_m2_per_hz = _welch(seg, result.rate_hz)
        mean_kpi, rms_kpi = _kpi(result, "hover_gap_mean_m"), _kpi(result, "hover_gap_rms_m")
        out.gap_mean_m = mean_kpi if math.isfinite(mean_kpi) else (float(seg.mean()) if seg.size else _NAN)
        out.gap_rms_m = rms_kpi if math.isfinite(rms_kpi) else (float(seg.std()) if seg.size else _NAN)
        if m_hat is not None:
            lev_mask = np.isin(lev, _LEVITATED) & np.isfinite(m_hat)
            if lev_mask.any():
                out.m_hat_kg = float(np.asarray(m_hat)[np.flatnonzero(lev_mask)[-1]])
    else:
        out.gap_mean_m = _kpi(result, "hover_gap_mean_m")
        out.gap_rms_m = _kpi(result, "hover_gap_rms_m")
    if math.isnan(out.m_hat_kg):
        unseated = _first_event(events, "Unseated")
        if unseated is not None:
            out.m_hat_kg = float(unseated.a)

    lo, hi = _num(_param(result, "safety.gap_min_m")), _num(_param(result, "safety.gap_max_m"))
    out.gap_limits_m = [lo, hi] if math.isfinite(lo) and math.isfinite(hi) else []
    out.g_star_m = _num(_param(result, "design.suspension.g_star_m"))
    out.m_star_kg = _num(_param(result, "design.carriage.m_star_kg"))
    out.coil_i_pulse_a = _num(_param(result, "safety.i_coil_pulse_a"))
    return out


# ---- haptics -----------------------------------------------------------------------------------


def _apparent_mass(result: RunResult) -> float:
    """Least-squares M of F_hand = M·a + B·v over hand-on ``Admittance`` samples (FAB-HAP-001; core §B.6)."""
    f = _channel(result, "f_hand_n")
    a = _channel(result, "a_mps2")
    v = _channel(result, "v_mps")
    mode = _channel(result, "motion_mode")
    if f is None or a is None or v is None or mode is None:
        return _NAN
    mask = (np.asarray(mode) == 3) & (np.abs(f) > 1e-9) & np.isfinite(f) & np.isfinite(a) & np.isfinite(v)
    if int(mask.sum()) < 20:
        return _NAN
    design = np.column_stack([np.asarray(a)[mask], np.asarray(v)[mask]])
    if np.linalg.matrix_rank(design) < 2:
        return _NAN
    coef, *_ = np.linalg.lstsq(design, np.asarray(f)[mask], rcond=None)
    return float(coef[0])


def _terminus(result: RunResult, out: HapticAnalysis, max_points: int) -> None:
    """Fill the terminus fields from the last TerminusCapture event (PHYSICS §16.1)."""
    events = _sorted_events(result)
    captures = [e for e in events if e.kind == "TerminusCapture"]
    out.terminus_count = len(captures)
    count_kpi = _kpi(result, "terminus_count")
    if math.isfinite(count_kpi):
        out.terminus_count = max(out.terminus_count, int(count_kpi))
    x = _channel(result, "x_m")
    if not captures or x is None or result.n_samples == 0:
        return
    cap = captures[-1]
    t = result.t_s
    t_c, d_b, tau_eff = float(cap.t_s), float(cap.a), float(cap.b)
    arrived = _first_event(events, "Arrived", after=t_c)
    t_end = float(arrived.t_s) if arrived is not None else float(t[-1])
    i_c = int(np.searchsorted(t, t_c - 1e-12))
    i_end = min(int(np.searchsorted(t, t_end - 1e-12)), result.n_samples - 1)
    x_ref = _channel(result, "x_ref_m")
    target = float(x_ref[i_end]) if x_ref is not None else float(x[i_end])
    sign = 1.0 if target >= float(x[i_c]) else -1.0
    seg = slice(i_c, i_end + 1)
    d = sign * (target - np.asarray(x[seg], dtype=np.float64))
    ts = t[seg] - t_c
    out.terminus_t_capture_s = t_c
    out.terminus_target_m = target
    out.terminus_d_b_m = d_b
    out.terminus_tau_eff_s = tau_eff
    positive = d > 0.0
    pidx = np.flatnonzero(positive)
    keep = pidx[_plot_index(pidx.size, max_points, [d[pidx]])] if pidx.size else pidx
    out.terminus_t_s = np.asarray(ts[keep], dtype=np.float64)
    out.terminus_d_m = np.asarray(d[keep], dtype=np.float64)

    # Semilog fit over the exponential capture: first d ≤ d_b, then d_ε < d, quiet floor.
    floor = _channel(result, "floor_ax_mps2")
    quiet = np.abs(np.asarray(floor[seg])) < TERMINUS_QUIET_MPS2 if floor is not None else np.ones(d.shape, dtype=bool)
    below_db = np.flatnonzero(d <= d_b * (1.0 + 1e-9)) if math.isfinite(d_b) and d_b > 0 else np.zeros(0, dtype=np.int64)
    if below_db.size:
        k0 = int(below_db[0])
        after = np.flatnonzero(d[k0:] <= TERMINUS_D_EPS_M)
        k1 = k0 + int(after[0]) if after.size else d.shape[0]
        win = np.arange(k0, k1)
        win = win[(d[win] > TERMINUS_D_EPS_M) & quiet[win]]
        if win.size >= 5:
            tt, ln_d = ts[win], np.log(d[win])
            slope, intercept = np.polyfit(tt, ln_d, 1)
            resid = ln_d - (slope * tt + intercept)
            ss_tot = float(np.sum((ln_d - ln_d.mean()) ** 2))
            r2 = 1.0 - float(np.sum(resid**2)) / ss_tot if ss_tot > 0 else _NAN
            out.terminus_fit_slope_per_s = float(slope)
            out.terminus_fit_r2 = r2
            ends = np.array([tt[0], tt[-1]], dtype=np.float64)
            out.terminus_fit_t_s = ends
            out.terminus_fit_d_m = np.exp(slope * ends + intercept)
    arrive_idx = np.flatnonzero(d <= TERMINUS_ARRIVE_M)
    v = _channel(result, "v_mps")

    tau_kpi, r2_kpi = _kpi(result, "terminus_tau_fit_s"), _kpi(result, "terminus_r2_min")
    if math.isfinite(tau_kpi) and math.isfinite(r2_kpi):
        out.terminus_tau_fit_s, out.terminus_semilog_r2, out.terminus_fit_source = tau_kpi, r2_kpi, "kpi"
    elif math.isfinite(out.terminus_fit_slope_per_s) and out.terminus_fit_slope_per_s < 0:
        out.terminus_tau_fit_s = -1.0 / out.terminus_fit_slope_per_s
        out.terminus_semilog_r2 = out.terminus_fit_r2
        out.terminus_fit_source = "recorded"
    t_arr_kpi, v_arr_kpi = _kpi(result, "terminus_t_arrive_s"), _kpi(result, "terminus_v_arrive_max_mps")
    out.t_arrive_s = t_arr_kpi
    out.arrival_v_mps = v_arr_kpi
    if math.isnan(out.arrival_v_mps) and arrive_idx.size and v is not None:
        out.arrival_v_mps = float(abs(np.asarray(v)[i_c + int(arrive_idx[0])]))


def analyze_haptics(
    result: RunResult, *, max_points: int = DEFAULT_MAX_POINTS, meta: Mapping[str, Any] | None = None
) -> HapticAnalysis:
    """Reduce a run to :class:`HapticAnalysis` (apparent mass, intent timings, terminus semilog fit)."""
    cls = _rail_class(result)
    stamp = run_meta(result, meta)
    n = result.n_samples
    x, v = _channel(result, "x_m"), _channel(result, "v_mps")
    fh, fhh = _channel(result, "f_hand_n"), _channel(result, "f_hand_hat_n")
    xr, mode = _channel(result, "x_ref_m"), _channel(result, "motion_mode")
    present = [a for a in (x, v, fh, fhh) if a is not None]
    idx = _plot_index(n, max_points, present)

    def pick(a: np.ndarray | None) -> np.ndarray:
        return np.asarray(a, dtype=np.float64)[idx].copy() if a is not None else _empty()

    out = HapticAnalysis(rail_class=cls, scenario=stamp["scenario"], meta=stamp)
    out.t_s = result.t_s[idx].copy() if n else _empty()
    out.x_m, out.v_mps, out.f_hand_n, out.f_hand_hat_n, out.x_ref_m = pick(x), pick(v), pick(fh), pick(fhh), pick(xr)
    out.motion_mode = np.asarray(mode, dtype=np.uint8)[idx].copy() if mode is not None else _empty_u8()
    out.breakaway_n = _kpi(result, "breakaway_n")
    out.apparent_mass_err = _kpi(result, "apparent_mass_err")
    out.intent_onset_s = _kpi(result, "intent_onset_s")
    out.intent_confirm_s = _kpi(result, "intent_confirm_s")
    out.reverse_latency_s = _kpi(result, "reverse_latency_s")
    out.soft_close_v_final_mps = _kpi(result, "soft_close_v_final")
    out.overshoot_m = _kpi(result, "overshoot_m")
    out.detent_peak_err = _kpi(result, "detent_peak_err")
    out.passivity_min_j = _kpi(result, "passivity_min_j")
    out.force_noise_rms_n = _kpi(result, "force_noise_rms_n")
    out.apparent_mass_kg = _apparent_mass(result)
    m_star, ratio = _num(_param(result, "design.carriage.m_star_kg")), _num(_param(result, "control.haptic.mass_ratio"))
    out.m_virtual_kg = m_star / ratio if math.isfinite(m_star) and math.isfinite(ratio) and ratio > 0 else _NAN
    out.v_open_cap_mps = _num(_param(result, "safety.v_open_max_mps"))
    out.v_close_cap_mps = _num(_param(result, "safety.v_close_max_mps"))
    if math.isfinite(out.v_open_cap_mps) and math.isfinite(out.v_close_cap_mps):
        out.v_cap_band_mps = [-out.v_close_cap_mps, out.v_open_cap_mps]
    _terminus(result, out, max_points)
    return out


# ---- safety ------------------------------------------------------------------------------------


def _cumtrapz(y: np.ndarray, dt: float) -> np.ndarray:
    y = np.nan_to_num(np.asarray(y, dtype=np.float64))
    out = np.zeros_like(y)
    if y.size > 1:
        out[1:] = np.cumsum(0.5 * (y[1:] + y[:-1]) * dt)
    return out


def analyze_safety(
    result: RunResult, *, max_points: int = DEFAULT_MAX_POINTS, meta: Mapping[str, Any] | None = None
) -> SafetyAnalysis:
    """Reduce a run to :class:`SafetyAnalysis` (energy ledger, contact forces, monitor trips)."""
    cls = _rail_class(result)
    stamp = run_meta(result, meta)
    n = result.n_samples
    ke, fc = _channel(result, "e_kin_gov_j"), _channel(result, "f_obstacle_n")
    cap, dem = _channel(result, "force_cap_n"), _channel(result, "demand")
    p_bus, e_kin, e_drag = _channel(result, "p_bus_w"), _channel(result, "e_kin_j"), _channel(result, "e_drag_j")
    e_hold = _channel(result, "e_hold_j")
    dt = 1.0 / result.rate_hz if result.rate_hz > 0 else 0.0
    e_bus = _cumtrapz(p_bus, dt) if p_bus is not None else None
    present = [a for a in (ke, fc, cap, e_bus, e_kin, e_drag) if a is not None]
    idx = _plot_index(n, max_points, present)

    def pick(a: np.ndarray | None) -> np.ndarray:
        return np.asarray(a, dtype=np.float64)[idx].copy() if a is not None else _empty()

    out = SafetyAnalysis(rail_class=cls, scenario=stamp["scenario"], meta=stamp)
    out.t_s = result.t_s[idx].copy() if n else _empty()
    out.e_kin_gov_j, out.contact_force_n, out.force_cap_n = pick(ke), pick(fc), pick(cap)
    out.demand = np.asarray(dem, dtype=np.uint8)[idx].copy() if dem is not None else _empty_u8()
    out.e_bus_j, out.e_kin_j, out.e_drag_j = pick(e_bus), pick(e_kin), pick(e_drag)
    out.ke_max_gov_j = _kpi(result, "ke_max_gov_j")
    out.momentum_max_kgmps = _kpi(result, "momentum_max")
    out.contact_force_max_n = _kpi(result, "contact_force_max_n")
    out.guide_force_max_n = _kpi(result, "guide_force_max_n")
    out.manual_force_n = _kpi(result, "manual_force_n")
    impacts = _kpi(result, "bumper_impacts")
    out.bumper_impacts = int(impacts) if math.isfinite(impacts) else 0
    out.bumper_energy_j = _kpi(result, "bumper_energy_j")
    out.reverse_latency_s = _kpi(result, "reverse_latency_s")
    out.ke_cap_j = _num(_param(result, "safety.e_max_j"))
    p_cap = _num(_param(result, "safety.p_max_kgmps"))
    out.momentum_cap_kgmps = p_cap if math.isfinite(p_cap) and p_cap > 0 else _NAN

    for e in _sorted_events(result):
        if e.kind == "FaultRaised":
            name = fault_name(e.a)
            out.fault_events.append((float(e.t_s), name, fault_severity(e.a)))
            if name not in out.faults:
                out.faults.append(name)
        elif e.kind == "Demand" and int(e.a) != 0:
            out.trips.append((float(e.t_s), state_name("demand", e.a)))
    if not out.trips and dem is not None and np.any(dem):
        # No Demand events recorded (older runs): derive the rising edges from the channel.
        codes = np.asarray(dem, dtype=np.int64)
        rises = np.flatnonzero((codes[1:] != codes[:-1]) & (codes[1:] != 0)) + 1
        out.trips = [(float(result.t_s[i]), state_name("demand", codes[i])) for i in rises]

    ledger: dict[str, float] = {}
    if e_bus is not None and e_bus.size:
        ledger["bus_energy_drawn_j"] = float(e_bus[-1])
    if e_kin is not None and e_kin.size:
        ledger["kinetic_final_j"] = float(e_kin[-1])
        ledger["kinetic_max_j"] = float(np.nanmax(e_kin))
    if e_drag is not None and e_drag.size:
        ledger["drag_dissipated_j"] = float(e_drag[-1] - e_drag[0])
    if e_hold is not None and e_hold.size:
        ledger["holdup_min_j"] = float(np.nanmin(e_hold))
    elif math.isfinite(_kpi(result, "e_hold_min_j")):
        ledger["holdup_min_j"] = _kpi(result, "e_hold_min_j")
    out.energy_ledger_j = ledger
    return out


# ---- seismic -----------------------------------------------------------------------------------


def _timeline(events: Sequence[RunEvent]) -> list[tuple[float, str]]:
    out: list[tuple[float, str]] = []
    for e in events:
        k = e.kind
        if k in ("SeismicState", "LevState"):
            label = f"{k}: {state_name(_EVENT_STATE_CHANNEL[k], e.a)}"
        elif k == "Trigger":
            label = f"Trigger (STA/LTA {e.a:.3g})"
        elif k == "Protect":
            label = f"Protect (level {int(e.a)})"
        elif k in ("Alert", "scenario:network_alert"):
            label = f"network alert (synthetic/injected; M{e.a:.1f}, R_epi {e.b:.0f} km)"
        elif k in ("scenario:alert_cancel", "scenario:power_loss"):
            label = k.split(":", 1)[1].replace("_", " ")
        elif k in ("Freeze", "AlertCancel", "Landed", "Seated", "plant:landed", "plant:locked", "plant:seated"):
            label = k
        elif k == "FaultRaised":
            label = f"Fault {fault_name(e.a)} ({fault_severity(e.a)})"
        else:
            continue
        out.append((float(e.t_s), label))
    return out


def analyze_seismic(
    result: RunResult, *, max_points: int = DEFAULT_MAX_POINTS, meta: Mapping[str, Any] | None = None
) -> SeismicAnalysis:
    """Reduce a run to :class:`SeismicAnalysis` (trigger, freeze, protect, land against t_P and t_S)."""
    from .scenarios import info as spec_info

    cls = _rail_class(result)
    stamp = run_meta(result, meta)
    n = result.n_samples
    floors = [_channel(result, f"floor_a{ax}_mps2") for ax in "xyz"]
    level, sstate, lev = _channel(result, "level"), _channel(result, "seismic_state"), _channel(result, "lev_state")
    idx = _plot_index(n, max_points, [f for f in floors if f is not None])
    out = SeismicAnalysis(rail_class=cls, scenario=stamp["scenario"], meta=stamp)
    out.t_s = result.t_s[idx].copy() if n else _empty()
    if n:
        out.floor_acc_mps2 = np.vstack(
            [np.asarray(f, dtype=np.float64)[idx] if f is not None else np.zeros(idx.size) for f in floors]
        )
    out.level = np.asarray(level, dtype=np.uint8)[idx].copy() if level is not None else _empty_u8()
    out.seismic_state = np.asarray(sstate, dtype=np.uint8)[idx].copy() if sstate is not None else _empty_u8()
    out.lev_state = np.asarray(lev, dtype=np.uint8)[idx].copy() if lev is not None else _empty_u8()
    if floors[0] is not None and floors[1] is not None:
        out.pga_mps2 = float(np.nanmax(np.hypot(floors[0], floors[1]))) if n else _NAN

    spec = result.meta.get("spec")
    out.info = spec_info(spec) if isinstance(spec, Mapping) else {}
    out.t_p_s = _num(out.info.get("t_p_s"))
    out.t_s_arrival_s = _num(out.info.get("t_s_s"))
    events = _sorted_events(result)
    out.trigger_t_s = _kpi(result, "trigger_t_s")
    out.freeze_t_s = _kpi(result, "freeze_t_s")
    if math.isnan(out.trigger_t_s):
        e = _first_event(events, "Trigger")
        out.trigger_t_s = float(e.t_s) if e else _NAN
    if math.isnan(out.freeze_t_s):
        e = _first_event(events, "Freeze")
        out.freeze_t_s = float(e.t_s) if e else _NAN
    protect = _first_event(events, "Protect")
    out.protect_t_s = float(protect.t_s) if protect else _NAN
    t_first = min((t for t in (out.trigger_t_s, out.protect_t_s) if math.isfinite(t)), default=-math.inf)
    # Touchdown: the earlier of the truth contact (plant:landed) and the controller's Landed event.
    landed = [
        e for e in (_first_event(events, "Landed", after=t_first), _first_event(events, "plant:landed", after=t_first)) if e
    ]
    out.land_t_s = min(float(e.t_s) for e in landed) if landed else _NAN
    locked = _first_event(events, "plant:locked", after=t_first)
    out.lock_t_s = float(locked.t_s) if locked else _NAN
    alert = _first_event(events, "Alert") or _first_event(events, "scenario:network_alert")
    out.alert_t_s = float(alert.t_s) if alert else _NAN
    cancel = _first_event(events, "AlertCancel") or _first_event(events, "scenario:alert_cancel")
    out.alert_cancel_t_s = float(cancel.t_s) if cancel else _NAN
    lvl = _kpi(result, "level_max")
    out.level_max = int(lvl) if math.isfinite(lvl) else (int(np.max(level)) if level is not None and n else 0)
    out.protect_margin_to_s_s = _kpi(result, "protect_margin_to_s_s")
    out.stroke_max_m = _kpi(result, "stroke_max_m")
    pd, tc = _channel(result, "pd_m"), _channel(result, "tau_c_s")
    t_read = (
        out.protect_t_s if math.isfinite(out.protect_t_s) else (out.trigger_t_s + 3.0 if math.isfinite(out.trigger_t_s) else _NAN)
    )
    if math.isfinite(t_read) and n:
        i = min(int(np.searchsorted(result.t_s, t_read)), n - 1)
        out.pd_m = float(pd[i]) if pd is not None else _NAN
        out.tau_c_s = float(tc[i]) if tc is not None else _NAN
    out.timeline = _timeline(events)
    return out


def analyze_detection(
    t0_s: float,
    fs_hz: float,
    a_xyz_mps2: np.ndarray,
    events: Sequence[tuple[float, str, float, float]],
    report: Mapping[str, float],
    *,
    rail_class: str = "",
    name: str = "",
    max_points: int = DEFAULT_MAX_POINTS,
    meta: Mapping[str, Any] | None = None,
) -> SeismicAnalysis:
    """A :class:`SeismicAnalysis` of a ``DetectorReplay`` over a record (``fabrail detect``).

    ``a_xyz_mps2`` is (3, n) structure acceleration at ``fs_hz`` from ``t0_s``; ``events``
    and ``report`` are the replay's. The level and seismic-state traces are rebuilt as step
    functions from the ``Protect`` and ``SeismicState`` events; there is no levitation state.
    """
    from .results import RunEvent

    a = np.atleast_2d(np.asarray(a_xyz_mps2, dtype=np.float64))
    if a.shape[0] != 3:
        raise ValueError(f"analyze_detection: expected (3, n) acceleration (got {a.shape})")
    n = a.shape[1]
    t = t0_s + np.arange(n, dtype=np.float64) / float(fs_hz)
    evs = sorted((RunEvent(float(e[0]), str(e[1]), float(e[2]), float(e[3])) for e in events), key=lambda e: e.t_s)
    level = np.zeros(n, dtype=np.uint8)
    sstate = np.zeros(n, dtype=np.uint8)
    for e in evs:
        i = int(np.searchsorted(t, e.t_s))
        if e.kind == "Protect":
            level[i:] = np.maximum(level[i:], np.uint8(max(0, min(3, int(e.a)))))
        elif e.kind == "SeismicState":
            sstate[i:] = np.uint8(max(0, int(e.a)))
    idx = _plot_index(n, max_points, list(a))
    stamp: dict[str, Any] = {"rail_class": rail_class, "scenario": name, "source": "DetectorReplay", "fs_hz": float(fs_hz)}
    stamp.update(dict(meta or {}))
    out = SeismicAnalysis(rail_class=rail_class, scenario=name, meta=stamp)
    out.t_s = t[idx]
    out.floor_acc_mps2 = a[:, idx].copy()
    out.level = level[idx]
    out.seismic_state = sstate[idx]
    trig, frz, prot = _first_event(evs, "Trigger"), _first_event(evs, "Freeze"), _first_event(evs, "Protect")
    out.trigger_t_s = float(trig.t_s) if trig else _NAN
    out.freeze_t_s = float(frz.t_s) if frz else _NAN
    out.protect_t_s = float(prot.t_s) if prot else _NAN
    lvl = _num(report.get("level_max"))
    out.level_max = int(lvl) if math.isfinite(lvl) else int(level.max(initial=0))
    out.pga_mps2 = float(np.max(np.hypot(a[0], a[1]))) if n else _NAN
    out.info = {str(k): float(v) for k, v in report.items()}
    out.timeline = _timeline(evs)
    return out


def analyze_run(
    result: RunResult,
    kinds: Sequence[str] = ("levitation", "haptics", "safety", "seismic"),
    *,
    max_points: int = DEFAULT_MAX_POINTS,
    meta: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Every requested per-run analysis, keyed by kind (the CLI JSON ``analysis`` block)."""
    reducers: dict[str, Callable[..., Any]] = {
        "levitation": analyze_levitation,
        "haptics": analyze_haptics,
        "safety": analyze_safety,
        "seismic": analyze_seismic,
    }
    unknown = [k for k in kinds if k not in reducers]
    if unknown:
        raise ValueError(f"analyze_run: unknown kind(s) {unknown} (expected {', '.join(reducers)})")
    return {k: reducers[k](result, max_points=max_points, meta=meta) for k in kinds}


# ---- ensemble ----------------------------------------------------------------------------------


def analyze_ensemble(result: EnsembleResult, thresholds: dict[str, float] | None = None) -> EnsembleAnalysis:
    """Reduce an ensemble to :class:`EnsembleAnalysis`; ``thresholds`` define the exceedance counts.

    A member exceeds threshold T on KPI k when its value is strictly above T; the Wilson
    bound counts over the members that report k (a KPI whose phase never happened is
    absent, core §B.6). P50/P95 use linear interpolation over the finite values.
    """
    names = result.all_kpi_names()
    out = EnsembleAnalysis(scenario=result.scenario, n=len(result), seed=int(result.seed))
    out.meta = {"scenario": result.scenario, "seed": int(result.seed), "n": len(result), **dict(result.meta)}
    out.member_index = np.arange(len(result), dtype=np.float64)
    for name in names:
        x = result.kpi(name)
        out.kpis[name] = x
        finite = x[np.isfinite(x)]
        out.n_reporting[name] = int(finite.size)
        out.p50[name] = float(np.quantile(finite, 0.5)) if finite.size else _NAN
        out.p95[name] = float(np.quantile(finite, 0.95)) if finite.size else _NAN
    for name, thr in dict(thresholds or {}).items():
        t = float(thr)
        if not math.isfinite(t):
            raise ValueError(f"thresholds.{name}: must be finite (got {thr!r})")
        out.thresholds[name] = t
        if name not in out.kpis:
            out.exceedances[name] = 0
            continue
        k, m = result.count_exceeding(name, t)
        out.exceedances[name] = k
        if m > 0:
            out.wilson_upper[name] = wilson_upper(k, m)
    return out


# ---- JSON round trip -----------------------------------------------------------------------------


def _from_json_value(default: Any, value: Any) -> Any:
    if isinstance(default, np.ndarray):
        if value is None:
            return default.copy()
        if default.dtype == np.uint8:
            return np.asarray(value, dtype=np.uint8)
        return _nan_array(value)
    if isinstance(default, float):
        return _NAN if value is None else float(value)
    if isinstance(default, bool):
        return bool(value)
    if isinstance(default, int):
        return int(value) if value is not None else 0
    return value


def _nan_array(value: Any) -> np.ndarray:
    def fill(v: Any) -> Any:
        if isinstance(v, list):
            return [fill(u) for u in v]
        return math.nan if v is None else v

    return np.array(fill(value), dtype=np.float64)


def analysis_from_jsonable(kind: str | type, data: Mapping[str, Any]) -> Any:
    """Rebuild an analysis dataclass from its JSON form (the CLI record's ``analysis`` block).

    ``kind`` is a key of :data:`ANALYSIS_KINDS` or the class. ``null`` numbers come back as
    NaN, arrays as numpy (uint8 state traces as uint8), lists of pairs as tuples.
    """
    cls = ANALYSIS_KINDS[kind] if isinstance(kind, str) else kind
    obj = cls()
    for f in dataclasses.fields(cls):
        if f.name not in data:
            continue
        default = getattr(obj, f.name)
        value = data[f.name]
        if f.name == "kpis" and isinstance(value, Mapping):
            setattr(obj, f.name, {str(k): _nan_array(v) for k, v in value.items()})
        elif f.name in ("trips", "timeline"):
            setattr(obj, f.name, [(float(a), str(b)) for a, b in value])
        elif f.name == "fault_events":
            setattr(obj, f.name, [(float(a), str(b), str(c)) for a, b, c in value])
        elif f.name in ("p50", "p95", "wilson_upper", "thresholds", "energy_ledger_j"):
            setattr(obj, f.name, {str(k): _NAN if v is None else float(v) for k, v in dict(value).items()})
        elif f.name in ("gap_limits_m", "psd_window_s", "v_cap_band_mps"):
            setattr(obj, f.name, [_NAN if v is None else float(v) for v in value])
        else:
            setattr(obj, f.name, _from_json_value(default, value))
    return obj


# ---- seed-42 acceptance anchors (core §E.5) ------------------------------------------------------

#: Per-unit (P) absolute tolerance floors of the seed-42 anchors; the relative part is 1e-6.
_ANCHOR_REL = 1e-6
_ANCHOR_ABS = {"m": 1e-9, "s": 1e-6, "N": 1e-6, "A": 1e-6, "kg": 1e-6, "W": 1e-6, "J": 1e-6, "m/s": 1e-9, "1": 1e-9, "count": 0.0}


def _anchor_run(
    cls: str,
    scenario: str,
    *,
    fidelity: str | None = None,
    options: Mapping[str, Any] | None = None,
    overrides: Mapping[str, object] | None = None,
) -> tuple[Any, Any, Any]:
    """(params, spec dict, series) of one voltage-mode seed-42 anchor case."""
    from . import scenarios
    from .config import RunConfig

    cfg = RunConfig(rail_class=cls, seed=42, coil="voltage", overrides=dict(overrides or {}))
    params = cfg.params()
    d, series = scenarios.spec(scenario, params, cfg, fidelity=fidelity, options=options)
    return params, d, series


def compute_anchors(
    *, quick: bool = False, workers: int | None = None, progress: Callable[[str], None] | None = None
) -> tuple[dict[str, object], dict[str, dict[str, object]], list[str]]:
    """The seed-42 acceptance anchors of core §E.5 (voltage coil mode, recommended tiers).

    Returns ``(values, tolerances, missing)`` ready for :func:`fabrail.export.anchors_record`;
    ``missing`` names anchors whose KPI or event did not occur in the run (never invented).
    ``quick`` keeps the cases that run in a few seconds (DOOR push_open, DRAWER unseat at
    26 kg, DRAWER payload_step, the selftest hash); the full set adds the DRAWER 8/48 kg
    unseat, DOOR hover/throw/power-loss/chest cases, WALL pwave_m7_r20, WALL noise_1h and
    the WALL isolation_kt n = 20 ensemble (research tier, isolation enabled). Tolerances
    are class (P): rel 1e-6 plus a per-unit absolute floor; C1 re-records them.
    """
    from . import require_native
    from .results import RunResult
    from .scenarios import to_json

    native = require_native()
    say = progress or (lambda _msg: None)
    values: dict[str, object] = {}
    tols: dict[str, dict[str, object]] = {}
    missing: list[str] = []

    def put(name: str, value: float | list[int] | None, unit: str, source: str, *, abs_tol: float | None = None) -> None:
        if value is None or (isinstance(value, float) and not math.isfinite(value)):
            missing.append(name)
            return
        values[name] = value
        tols[name] = {
            "rel": 0.0 if unit == "count" else _ANCHOR_REL,
            "abs": _ANCHOR_ABS.get(unit, 1e-9) if abs_tol is None else abs_tol,
            "unit": unit,
            "source": f"{source}; seed 42, voltage coil mode; core §E.5; (P) provisional, C1 re-records",
        }

    cases: list[tuple[str, str, str, Mapping[str, Any] | None, Mapping[str, object] | None]] = [
        ("door_push_open", "door", "push_open", None, None),
        ("drawer_unseat_26kg", "drawer", "unseat", {"mass_kg": 26.0}, None),
        ("drawer_payload_step", "drawer", "payload_step", None, None),
    ]
    if not quick:
        cases += [
            ("drawer_unseat_8kg", "drawer", "unseat", {"mass_kg": 8.0}, None),
            ("drawer_unseat_48kg", "drawer", "unseat", {"mass_kg": 48.0}, None),
            ("door_hover", "door", "hover", None, None),
            ("door_throw_soft_close", "door", "throw_soft_close", None, None),
            ("door_power_loss", "door", "power_loss", None, None),
            ("door_obstruction_chest", "door", "obstruction_chest", None, None),
            ("wall_pwave_m7_r20", "wall", "pwave_m7_r20", None, None),
            ("wall_noise_1h", "wall", "noise_1h", None, None),
        ]
    jobs, specs = [], []
    for key, cls, scen, opts, ov in cases:
        params, d, series = _anchor_run(cls, scen, options=opts, overrides=ov)
        jobs.append((params, native.Scenario.from_json(to_json(d), series)))
        specs.append((key, cls, scen, d))
    say(f"running {len(jobs)} anchor cases")
    runs = [RunResult.from_native(r) for r in native.run_batch(jobs, workers)]
    by_key = {key: (run, cls, scen, d) for (key, cls, scen, d), run in zip(specs, runs, strict=True)}

    def src(key: str) -> str:
        _, cls, scen, d = by_key[key]
        return f"{cls}/{scen} ({d['fidelity']['model']})"

    def kpi(key: str, name: str) -> float:
        return float(by_key[key][0].kpis.get(name, _NAN))

    k = "door_push_open"
    for name, unit in (
        ("breakaway_n", "N"),
        ("apparent_mass_err", "1"),
        ("intent_onset_s", "s"),
        ("intent_confirm_s", "s"),
        ("hover_gap_mean_m", "m"),
        ("hover_gap_rms_m", "m"),
        ("hover_power_w", "W"),
    ):
        put(f"{k}.{name}", kpi(k, name), unit, src(k))
    unseat_keys = ["drawer_unseat_26kg"] + ([] if quick else ["drawer_unseat_8kg", "drawer_unseat_48kg"])
    for k in unseat_keys:
        put(f"{k}.unseat_time_s", kpi(k, "unseat_time_s"), "s", src(k))
        ev = by_key[k][0].first_event("Unseated")
        put(f"{k}.m_hat_kg", float(ev.a) if ev else None, "kg", src(k) + " Unseated event a")
        put(f"{k}.peak_coil_a", float(ev.b) if ev else None, "A", src(k) + " Unseated event b")
    put("drawer_payload_step.coil_i_ss_a", kpi("drawer_payload_step", "coil_i_ss_a"), "A", src("drawer_payload_step"))
    h = int(native.selftest_hash(2000))
    put("golden.selftest_hash_2000_hi_lo", [h >> 32, h & 0xFFFFFFFF], "count", "_native.selftest_hash(2000) as [hi32, lo32]")
    if not quick:
        put("door_hover.force_noise_rms_n", kpi("door_hover", "force_noise_rms_n"), "N", src("door_hover"))
        put("door_hover.hover_power_w", kpi("door_hover", "hover_power_w"), "W", src("door_hover"))
        put(
            "door_throw_soft_close.soft_close_v_final_mps",
            kpi("door_throw_soft_close", "soft_close_v_final"),
            "m/s",
            src("door_throw_soft_close"),
        )
        put("door_power_loss.touchdown_v_mps", kpi("door_power_loss", "touchdown_v_mps"), "m/s", src("door_power_loss"))
        put("door_power_loss.e_hold_min_j", kpi("door_power_loss", "e_hold_min_j"), "J", src("door_power_loss"))
        put(
            "door_obstruction_chest.contact_force_max_n",
            kpi("door_obstruction_chest", "contact_force_max_n"),
            "N",
            src("door_obstruction_chest"),
        )
        put(
            "door_obstruction_chest.reverse_latency_s",
            kpi("door_obstruction_chest", "reverse_latency_s"),
            "s",
            src("door_obstruction_chest"),
        )
        k = "wall_pwave_m7_r20"
        run = by_key[k][0]
        sei = analyze_seismic(run, max_points=10)
        put(f"{k}.trigger_after_t_p_s", sei.trigger_t_s - sei.t_p_s, "s", src(k))
        put(f"{k}.protect_after_t_p_s", sei.protect_t_s - sei.t_p_s, "s", src(k))
        put(f"{k}.land_before_t_s_s", sei.t_s_arrival_s - sei.land_t_s, "s", src(k))
        put(f"{k}.level_max", float(sei.level_max), "count", src(k))
        k = "wall_noise_1h"
        run = by_key[k][0]
        # A run emits no Trigger events (DetectorReplay does): count the Normal → Triggered transitions.
        raw = [e for e in run.events_of("SeismicState") if int(e.a) == 1]
        put(f"{k}.raw_triggers", float(len(raw)), "count", src(k) + " SeismicState → Triggered")
        put(f"{k}.protect_count", float(len(run.events_of("Protect"))), "count", src(k))
        say("running the WALL isolation_kt ensemble (n = 20, research tier)")
        from .config import RunConfig
        from .ensemble import run_ensemble

        iso_cfg = RunConfig(
            rail_class="wall",
            seed=42,
            coil="voltage",
            record_rate_hz=50.0,
            workers=workers,
            overrides={"control.seismic.isolation.enable": True, "control.seismic.policy.action": "isolate"},
        )
        ens = run_ensemble(iso_cfg.params(), "isolation_kt", iso_cfg, 20)
        ea = analyze_ensemble(ens)
        tier = str(ens.meta.get("fidelity", "?"))
        put("wall_isolation_kt_n20.stroke_max_m_p95", ea.p95.get("stroke_max_m"), "m", f"wall/isolation_kt ({tier}) P95 of 20")
        put(
            "wall_isolation_kt_n20.bumper_energy_j_p95",
            ea.p95.get("bumper_energy_j"),
            "J",
            f"wall/isolation_kt ({tier}) P95 of 20",
        )
    return values, tols, missing
