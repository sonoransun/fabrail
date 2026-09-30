//! fabrail-web: the real controller and plant in the browser (plan §3b). Owner: W1 (export list and state layout frozen by W0-C1; ABI v2 additions by W1).
//!
//! Built for `wasm32-unknown-unknown` as a cdylib with hand-written `extern "C"`
//! exports (no wasm-bindgen; the ~100-line loader is `docs/assets/js/engine.js`):
//! `create(preset)`, `set_inputs(hand force, floor accel, command)`, `step(n_ticks)`, a
//! state-buffer pointer, the analytic anchors and `selftest_hash()`, which every site
//! page compares with the firmware `GOLDEN_SELFTEST_HASH`. Depends on fabrail-sim with
//! `default-features = false` (no rayon on wasm32). Size budget ≤ 1.5 MB.
//!
//! Build: `RUSTC=$(rustup which rustc --toolchain stable) rustup run stable cargo build
//! -p fabrail-web --target wasm32-unknown-unknown --release`, then copy
//! `fabrail_web.wasm` to `docs/assets/wasm/` (Wave 3, C2c, with its SHA-256 in
//! `ANCHORS.wasm`).
//!
//! # The engine
//!
//! An engine is one device: a class preset (the unchanged `fabrail_params::presets`
//! parameter set) on one plant tier, closed through fabrail-sim's steppable `Session`
//! — the same `fabrail-rt` executive at the 20 kHz base tick as the firmware, the
//! SimBoard sensor models (seed [`NOISE_SEED`]) and an RK4 plant (PHYSICS §5, §24) —
//! stepped single-threaded. Its scenario is generated in code ([`scenario_json`], no
//! series, no events) and exported, so the native `fabrail._native.run` of that scenario
//! plus the same inputs reproduces a browser trace bit for bit (FAB-SW-014,
//! `tests/test_site.py::test_wasm_trace_matches_native`). Inputs are held until changed:
//! the hand is a pure force source along x (grip contact; 0 N releases it, PHYSICS §22),
//! the floor acceleration is constant over each step call, and a command goes on the
//! command link at the start of the next step (reaching the supervisor after the CAN
//! latency). Default tier: Carriage6 ([`DEFAULT_TIER`]).
//!
//! # ABI (version [`ABI_VERSION`] = 2; append-only)
//!
//! Two layers over one engine type. Pointers are wasm32 linear-memory byte offsets.
//! Version 1 (frozen by W0-C1) is unchanged; version 2 only adds exports (marked v2)
//! and command code 9.
//!
//! **Singleton layer** (what `docs/assets/js/engine.js` calls; `WASM_EXPORTS` there):
//!
//! | Export | Signature | Meaning |
//! |---|---|---|
//! | `memory` | Memory | linear memory |
//! | `fabrail_abi_version` | () → u32 | [`ABI_VERSION`] |
//! | `fabrail_create` | (preset: u32) → u32 | (re)creates the page engine on the default tier; 0 = ok, 1 = unknown preset, 2 = setup error |
//! | `fabrail_set_inputs` | (hand_n: f64, floor_ax: f64, floor_ay: f64, floor_az: f64, command: u32) | inputs for the next steps |
//! | `fabrail_step` | (n_ticks: u32) → u32 | advances n base ticks (20 kHz; ≤ [`MAX_STEP_TICKS`] per call); returns ticks stepped |
//! | `fabrail_state_ptr` | () → u32 | byte offset of the f64 state buffer (0 without an engine) |
//! | `fabrail_state_len` | () → u32 | [`STATE_LEN`] |
//! | `fabrail_selftest_hash` | () → u64 | `fabrail_rt::selftest::selftest_hash(2000)` (BigInt in JS) |
//! | `fabrail_anchor` | (id: u32) → f64 | analytic anchor `id` (NaN if unknown) |
//! | `fabrail_create_tier` (v2) | (preset: u32, tier: u32) → u32 | as `fabrail_create` on tier id `tier`; 3 = unknown tier |
//! | `fabrail_status` (v2) | () → u32 | 0 running, 1 no engine, 2 stopped by a simulation error, 3 scenario end reached |
//! | `fabrail_tier` (v2) | () → u32 | tier id of the page engine (`u32::MAX` without one) |
//! | `fabrail_go_to` (v2) | (x_m: f64) → u32 | queues `GoTo(x_m)` for the next step; 1 = queued, 0 = no engine / non-finite |
//!
//! **Handle layer** (several engines per page, e.g. class comparisons; no BigInt):
//!
//! | Export | Signature | Meaning |
//! |---|---|---|
//! | `fr_version` | () → u32 | [`ABI_VERSION`] |
//! | `fr_selftest_hash_lo` / `fr_selftest_hash_hi` | () → u32 | low / high 32 bits of the selftest hash |
//! | `fr_create` | (preset: u32) → u32 | new engine handle on the default tier (1 … [`MAX_ENGINES`]); 0 = error |
//! | `fr_destroy` | (h: u32) → u32 | 1 when a live engine was destroyed |
//! | `fr_set_inputs` | (h: u32, hand_force_n: f64, floor_ax: f64, floor_ay: f64, floor_az: f64, cmd: u32) → u32 | 1 = ok |
//! | `fr_step` | (h: u32, n_ticks: u32) → u32 | ticks stepped |
//! | `fr_state_ptr` | (h: u32) → *const f64 | state buffer (null for a bad handle) |
//! | `fr_state_len` | () → u32 | [`STATE_LEN`] |
//! | `fr_anchor` | (id: u32) → f64 | as `fabrail_anchor` |
//! | `fr_create_tier` (v2) | (preset: u32, tier: u32) → u32 | new engine handle on tier id `tier`; 0 = error |
//! | `fr_status` (v2) | (h: u32) → u32 | as `fabrail_status` (1 = bad handle) |
//! | `fr_tier` (v2) | (h: u32) → u32 | tier id (`u32::MAX` for a bad handle) |
//! | `fr_go_to` (v2) | (h: u32, x_m: f64) → u32 | as `fabrail_go_to` |
//! | `fr_text_ptr` (v2) | () → *const u8 | the text buffer (UTF-8) the calls below fill; valid until the next of them |
//! | `fr_scenario_json` (v2) | (preset: u32, tier: u32) → u32 | writes the canonical scenario JSON an engine of (preset, tier) runs; returns its byte length (0 = error) |
//! | `fr_anchor_name` (v2) | (id: u32) → u32 | writes anchor `id`'s `Params.derived()` key; byte length (0 = unknown) |
//! | `fr_last_error` (v2) | () → u32 | writes the last create/step error message; byte length (0 = none) |
//! | analytic `fr_*` | f64 → f64 | closed forms of `fabrail_params::derive` (below) |
//!
//! Analytic exports (all f64, the single-source closed forms): `fr_open_loop_pole(g*,
//! h′)`, `fr_zero_power_mass(g, g*, h′, m*)`, `fr_v_low_energy(m, E)`,
//! `fr_momentum_cap(m, p)`, `fr_stop_distance(v, a, j)`, `fr_clamp_force(E, k)`,
//! `fr_series_k(k1, k2)`, `fr_unattended_close_speed(v_cap, F, m_gov, k_series)`,
//! `fr_terminus_t_arrive(v, j, τ, d_ε)`, `fr_terminus_d0(v, j, τ)`,
//! `fr_terminus_db(v, j, τ)`, `fr_terminus_tb(v, j, τ)` (v2), `fr_sp_time(r, v_p, v_s)`,
//! `fr_transmissibility(r, ζ)`, `fr_t_sky(r, ζ)`, `fr_den_hartog_f(μ)`,
//! `fr_den_hartog_zeta(μ)`, `fr_warburton_f(μ)`, `fr_warburton_zeta(μ)`. They mirror
//! the JS `FORMULAS` block for node cross-checks.
//!
//! **Presets** (`preset: u32`, = `engine.js` `PRESETS` index): 0 drawer, 1 door,
//! 2 wall, 3 heavy ([`PRESETS`]).
//!
//! **Tiers** (`tier: u32`, v2): 0 Carriage6 (default), 1 Heave1, 2 Travel1d ([`TIERS`]).
//! Heave1 ignores x-axis inputs (hand, LSM); Travel1d assumes ideal levitation with
//! kinematic heave and steps the plant every 1 ms.
//!
//! **Commands** (`command`/`cmd`, = `engine.js` `COMMANDS`, mirroring
//! `fabrail_types::UserCommand`; [`cmd`]): 0 none, 1 open, 2 close, 3 stop, 4 lift,
//! 5 land, 6 reset, 7 hold_to_run_on, 8 hold_to_run_off, 9 manual (v2). A command is
//! consumed by the next step; code 0 (and an unknown code) leaves a pending command in
//! place, so a page may set forces every frame without cancelling a click.
//!
//! **Anchors** (`id`): `id = 64·preset + k`, k indexing
//! `fabrail_params::derive::derived()` (the `Params.derived()` table, e.g. `pole_rad_s`)
//! of that preset ([`anchor`], [`anchor_name`]).
//!
//! **State buffer** ([`STATE_LEN`] = 20 little-endian f64, = `engine.js`
//! `STATE_FIELDS`, append-only; [`idx`]). Truth fields are the plant after the last
//! completed step; controller fields are the executive's outputs of the last executed
//! base tick (tick − 1), as a telemetry frame carries them:
//!
//! | Index | Name | Quantity |
//! |---|---|---|
//! | 0 | `tick` | base tick (20 kHz) since create |
//! | 1 | `t_s` | time since create, s |
//! | 2 | `x_m` | travel position (truth), m |
//! | 3 | `v_mps` | travel velocity (truth), m/s |
//! | 4 | `a_mps2` | travel acceleration (truth), m/s² |
//! | 5 | `gap_m` | mean magnetic gap over the fitted elements (truth), m |
//! | 6 | `coil_i_a` | mean coil current over the fitted elements (truth), A |
//! | 7 | `iq_a` | LSM q-axis current (truth), A |
//! | 8 | `f_lsm_n` | LSM thrust (truth), N |
//! | 9 | `f_hand_n` | hand force applied to the front (truth; = the input for this pure force source), N |
//! | 10 | `f_hand_hat_n` | momentum-observer hand-force estimate, N |
//! | 11 | `m_hat_kg` | estimated mass m̂, kg |
//! | 12 | `e_kin_gov_j` | governor kinetic energy ½·m_gov·v², J |
//! | 13 | `lev_state` | `LevState` wire value |
//! | 14 | `motion_mode` | `MotionMode` wire value |
//! | 15 | `seismic_state` | `SeismicState` wire value |
//! | 16 | `seismic_level` | `SeismicLevel` wire value |
//! | 17 | `safety_demand` | `SafetyDemand` wire value (monitor verdict) |
//! | 18 | `fault_bits_lo` | `FaultSet.bits[0]` of the core ∪ monitor faults (u32, exact in f64) |
//! | 19 | `floor_ax_mps2` | floor acceleration x applied to the plant (truth), m/s² |
//!
//! Before the first step the controller fields hold the rest state (the class rest
//! `LevState`, m̂ = the body mass).

use std::sync::{Mutex, MutexGuard, OnceLock};

use fabrail_params::derive::{self, DERIVED_N};
use fabrail_params::presets;

mod engine;

pub use engine::{
    DEFAULT_TIER, Engine, EngineError, EngineStatus, Inputs, MAX_STEP_TICKS, NOISE_SEED, PRESETS,
    TIERS, preset_class, scenario_json, tier_of, user_command,
};

/// ABI version (bump only when the ABI gains exports; never renumber). 2 = W1 additions.
pub const ABI_VERSION: u32 = 2;

/// Number of f64 values in the state buffer.
pub const STATE_LEN: usize = 20;

/// State field names, in buffer order (= `engine.js` `STATE_FIELDS`).
pub const STATE_FIELDS: [&str; STATE_LEN] = [
    "tick",
    "t_s",
    "x_m",
    "v_mps",
    "a_mps2",
    "gap_m",
    "coil_i_a",
    "iq_a",
    "f_lsm_n",
    "f_hand_n",
    "f_hand_hat_n",
    "m_hat_kg",
    "e_kin_gov_j",
    "lev_state",
    "motion_mode",
    "seismic_state",
    "seismic_level",
    "safety_demand",
    "fault_bits_lo",
    "floor_ax_mps2",
];

/// State buffer indices.
pub mod idx {
    /// Base tick.
    pub const TICK: usize = 0;
    /// Time, s.
    pub const T_S: usize = 1;
    /// Position, m.
    pub const X_M: usize = 2;
    /// Velocity, m/s.
    pub const V_MPS: usize = 3;
    /// Acceleration, m/s².
    pub const A_MPS2: usize = 4;
    /// Mean gap, m.
    pub const GAP_M: usize = 5;
    /// Mean coil current, A.
    pub const COIL_I_A: usize = 6;
    /// LSM q current, A.
    pub const IQ_A: usize = 7;
    /// LSM thrust, N.
    pub const F_LSM_N: usize = 8;
    /// Hand force, N.
    pub const F_HAND_N: usize = 9;
    /// Hand-force estimate, N.
    pub const F_HAND_HAT_N: usize = 10;
    /// Mass estimate, kg.
    pub const M_HAT_KG: usize = 11;
    /// Governor kinetic energy, J.
    pub const E_KIN_GOV_J: usize = 12;
    /// LevState.
    pub const LEV_STATE: usize = 13;
    /// MotionMode.
    pub const MOTION_MODE: usize = 14;
    /// SeismicState.
    pub const SEISMIC_STATE: usize = 15;
    /// SeismicLevel.
    pub const SEISMIC_LEVEL: usize = 16;
    /// SafetyDemand.
    pub const SAFETY_DEMAND: usize = 17;
    /// `FaultSet.bits[0]`.
    pub const FAULT_BITS_LO: usize = 18;
    /// Floor acceleration x, m/s².
    pub const FLOOR_AX_MPS2: usize = 19;
}

/// Command codes (= `engine.js` `COMMANDS`; 9 is a v2 addition).
pub mod cmd {
    /// No command.
    pub const NONE: u32 = 0;
    /// Open.
    pub const OPEN: u32 = 1;
    /// Close.
    pub const CLOSE: u32 = 2;
    /// Stop.
    pub const STOP: u32 = 3;
    /// Lift.
    pub const LIFT: u32 = 4;
    /// Land.
    pub const LAND: u32 = 5;
    /// Reset (FAULT / INSPECT).
    pub const RESET: u32 = 6;
    /// Hold-to-run pressed.
    pub const HOLD_TO_RUN_ON: u32 = 7;
    /// Hold-to-run released.
    pub const HOLD_TO_RUN_OFF: u32 = 8;
    /// Enter MANUAL (unpowered, on rollers) (v2).
    pub const MANUAL: u32 = 9;
    /// Highest valid code.
    pub const MAX: u32 = 9;
}

/// Engines available through the handle layer.
pub const MAX_ENGINES: usize = 8;

/// Anchor id stride per preset (`id = ANCHOR_STRIDE·preset + k`).
pub const ANCHOR_STRIDE: u32 = 64;

/// `fabrail_status` / `fr_status` code without an engine (or for a bad handle).
pub const STATUS_NO_ENGINE: u32 = 1;

/// `fabrail_tier` / `fr_tier` without an engine.
pub const TIER_NONE: u32 = u32::MAX;

/// The selftest hash computed in this runtime (cached; `fabrail_rt` golden run).
pub fn selftest_hash() -> u64 {
    static HASH: OnceLock<u64> = OnceLock::new();
    *HASH.get_or_init(|| fabrail_rt::selftest::selftest_hash(fabrail_rt::selftest::SELFTEST_TICKS))
}

fn derived_tables() -> &'static [[(&'static str, f64); DERIVED_N]; 4] {
    static TABLES: OnceLock<[[(&'static str, f64); DERIVED_N]; 4]> = OnceLock::new();
    TABLES.get_or_init(|| PRESETS.map(|c| derive::derived(&presets::by_class(c))))
}

/// Analytic anchor `id = 64·preset + k` (k < `DERIVED_N`): entry k of
/// `fabrail_params::derive::derived()` of that preset; NaN when unknown.
pub fn anchor(id: u32) -> f64 {
    let (p, k) = ((id / ANCHOR_STRIDE) as usize, (id % ANCHOR_STRIDE) as usize);
    derived_tables()
        .get(p)
        .and_then(|t| t.get(k))
        .map_or(f64::NAN, |(_, v)| *v)
}

/// Name of anchor `id` (the `Params.derived()` key), if any.
pub fn anchor_name(id: u32) -> Option<&'static str> {
    let (p, k) = ((id / ANCHOR_STRIDE) as usize, (id % ANCHOR_STRIDE) as usize);
    derived_tables()
        .get(p)
        .and_then(|t| t.get(k))
        .map(|(n, _)| *n)
}

/// Tier id of an engine.
fn tier_id(e: &Engine) -> u32 {
    TIERS
        .iter()
        .position(|t| *t == e.tier())
        .map_or(TIER_NONE, |i| i as u32)
}

// ---- engine and text storage -----------------------------------------------------------

type Slots = [Option<Box<Engine>>; MAX_ENGINES];

static ENGINES: Mutex<Slots> = Mutex::new([const { None }; MAX_ENGINES]);
static DEFAULT_ENGINE: Mutex<Option<Box<Engine>>> = Mutex::new(None);
static TEXT: Mutex<Vec<u8>> = Mutex::new(Vec::new());
static LAST_ERROR: Mutex<String> = Mutex::new(String::new());

fn lock<T>(m: &'static Mutex<T>) -> MutexGuard<'static, T> {
    match m.lock() {
        Ok(g) => g,
        Err(p) => p.into_inner(),
    }
}

fn with_handle<R>(h: u32, f: impl FnOnce(&mut Engine) -> R) -> Option<R> {
    let mut g = lock(&ENGINES);
    let slot = g.get_mut((h as usize).checked_sub(1)?)?;
    slot.as_deref_mut().map(f)
}

fn with_default<R>(f: impl FnOnce(&mut Engine) -> R) -> Option<R> {
    lock(&DEFAULT_ENGINE).as_deref_mut().map(f)
}

/// Replaces the text buffer; returns its byte length.
fn put_text(s: &str) -> u32 {
    let mut t = lock(&TEXT);
    t.clear();
    t.extend_from_slice(s.as_bytes());
    t.len() as u32
}

fn set_error(msg: String) {
    *lock(&LAST_ERROR) = msg;
}

/// Builds an engine, recording the reason of a failure for `fr_last_error`.
fn build(preset: u32, tier: u32) -> Result<Engine, EngineError> {
    Engine::with_tier(preset, tier).inspect_err(|e| set_error(e.to_string()))
}

/// Steps an engine, recording a simulation error for `fr_last_error`.
fn step_engine(e: &mut Engine, n_ticks: u32) -> u32 {
    let n = e.step(n_ticks);
    if let Some(msg) = e.error() {
        set_error(msg.to_string());
    }
    n
}

// ---- singleton layer (engine.js) -------------------------------------------------------

/// ABI version.
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_abi_version() -> u32 {
    ABI_VERSION
}

/// (Re)creates the page engine on tier id `tier`: 0 = ok, 1 = unknown preset, 2 = setup
/// error, 3 = unknown tier (v2). The previous page engine is kept on error.
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_create_tier(preset: u32, tier: u32) -> u32 {
    match build(preset, tier) {
        Ok(e) => {
            *lock(&DEFAULT_ENGINE) = Some(Box::new(e));
            0
        }
        Err(EngineError::UnknownPreset(_)) => 1,
        Err(EngineError::Sim(_)) => 2,
        Err(EngineError::UnknownTier(_)) => 3,
    }
}

/// (Re)creates the page engine on the default tier: 0 = ok, 1 = unknown preset, 2 =
/// setup error.
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_create(preset: u32) -> u32 {
    fabrail_create_tier(preset, DEFAULT_TIER)
}

/// Sets the page engine's inputs.
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_set_inputs(
    hand_n: f64,
    floor_ax: f64,
    floor_ay: f64,
    floor_az: f64,
    command: u32,
) {
    let _ = with_default(|e| e.set_inputs(hand_n, floor_ax, floor_ay, floor_az, command));
}

/// Steps the page engine; returns ticks stepped (0 without an engine).
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_step(n_ticks: u32) -> u32 {
    with_default(|e| step_engine(e, n_ticks)).unwrap_or(0)
}

/// Byte offset of the page engine's state buffer (0 without an engine). The value is a
/// wasm32 linear-memory address; on 64-bit hosts it is truncated and meaningless.
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_state_ptr() -> u32 {
    with_default(|e| e.state().as_ptr() as usize as u32).unwrap_or(0)
}

/// Number of f64 values in the state buffer.
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_state_len() -> u32 {
    STATE_LEN as u32
}

/// The selftest hash computed in this runtime.
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_selftest_hash() -> u64 {
    selftest_hash()
}

/// Analytic anchor `id` (NaN if unknown).
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_anchor(id: u32) -> f64 {
    anchor(id)
}

/// Page engine status (v2): 0 running, 1 no engine, 2 failed, 3 ended.
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_status() -> u32 {
    with_default(|e| e.status() as u32).unwrap_or(STATUS_NO_ENGINE)
}

/// Page engine tier id (v2; `u32::MAX` without an engine).
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_tier() -> u32 {
    with_default(|e| tier_id(e)).unwrap_or(TIER_NONE)
}

/// Queues `GoTo(x_m)` on the page engine for the next step (v2); 1 = queued.
#[unsafe(no_mangle)]
pub extern "C" fn fabrail_go_to(x_m: f64) -> u32 {
    with_default(|e| u32::from(e.go_to(x_m))).unwrap_or(0)
}

// ---- handle layer -----------------------------------------------------------------------

/// ABI version.
#[unsafe(no_mangle)]
pub extern "C" fn fr_version() -> u32 {
    ABI_VERSION
}

/// Low 32 bits of the selftest hash.
#[unsafe(no_mangle)]
pub extern "C" fn fr_selftest_hash_lo() -> u32 {
    selftest_hash() as u32
}

/// High 32 bits of the selftest hash.
#[unsafe(no_mangle)]
pub extern "C" fn fr_selftest_hash_hi() -> u32 {
    (selftest_hash() >> 32) as u32
}

/// Creates an engine on tier id `tier` (v2); returns its handle (1-based) or 0 on error
/// / no free slot.
#[unsafe(no_mangle)]
pub extern "C" fn fr_create_tier(preset: u32, tier: u32) -> u32 {
    let Ok(e) = build(preset, tier) else {
        return 0;
    };
    let mut g = lock(&ENGINES);
    match g.iter_mut().enumerate().find(|(_, s)| s.is_none()) {
        Some((i, slot)) => {
            *slot = Some(Box::new(e));
            i as u32 + 1
        }
        None => {
            set_error(format!(
                "fr_create: all {MAX_ENGINES} engine slots are in use"
            ));
            0
        }
    }
}

/// Creates an engine on the default tier; returns its handle (1-based) or 0 on error /
/// no free slot.
#[unsafe(no_mangle)]
pub extern "C" fn fr_create(preset: u32) -> u32 {
    fr_create_tier(preset, DEFAULT_TIER)
}

/// Destroys engine `h`; 1 when a live engine was destroyed.
#[unsafe(no_mangle)]
pub extern "C" fn fr_destroy(h: u32) -> u32 {
    let mut g = lock(&ENGINES);
    match (h as usize).checked_sub(1).and_then(|i| g.get_mut(i)) {
        Some(slot @ Some(_)) => {
            *slot = None;
            1
        }
        _ => 0,
    }
}

/// Sets engine `h`'s inputs; 1 = ok, 0 = bad handle.
#[unsafe(no_mangle)]
pub extern "C" fn fr_set_inputs(
    h: u32,
    hand_force_n: f64,
    floor_ax: f64,
    floor_ay: f64,
    floor_az: f64,
    cmd: u32,
) -> u32 {
    with_handle(h, |e| {
        e.set_inputs(hand_force_n, floor_ax, floor_ay, floor_az, cmd)
    })
    .map_or(0, |_| 1)
}

/// Steps engine `h`; returns ticks stepped (0 for a bad handle).
#[unsafe(no_mangle)]
pub extern "C" fn fr_step(h: u32, n_ticks: u32) -> u32 {
    with_handle(h, |e| step_engine(e, n_ticks)).unwrap_or(0)
}

/// Engine `h`'s state buffer (null for a bad handle).
#[unsafe(no_mangle)]
pub extern "C" fn fr_state_ptr(h: u32) -> *const f64 {
    with_handle(h, |e| e.state().as_ptr()).unwrap_or(std::ptr::null())
}

/// Number of f64 values in the state buffer.
#[unsafe(no_mangle)]
pub extern "C" fn fr_state_len() -> u32 {
    STATE_LEN as u32
}

/// Analytic anchor `id` (NaN if unknown).
#[unsafe(no_mangle)]
pub extern "C" fn fr_anchor(id: u32) -> f64 {
    anchor(id)
}

/// Engine `h`'s status (v2): 0 running, 1 bad handle, 2 failed, 3 ended.
#[unsafe(no_mangle)]
pub extern "C" fn fr_status(h: u32) -> u32 {
    with_handle(h, |e| e.status() as u32).unwrap_or(STATUS_NO_ENGINE)
}

/// Engine `h`'s tier id (v2; `u32::MAX` for a bad handle).
#[unsafe(no_mangle)]
pub extern "C" fn fr_tier(h: u32) -> u32 {
    with_handle(h, |e| tier_id(e)).unwrap_or(TIER_NONE)
}

/// Queues `GoTo(x_m)` on engine `h` for the next step (v2); 1 = queued.
#[unsafe(no_mangle)]
pub extern "C" fn fr_go_to(h: u32, x_m: f64) -> u32 {
    with_handle(h, |e| u32::from(e.go_to(x_m))).unwrap_or(0)
}

/// The text buffer filled by `fr_scenario_json`, `fr_anchor_name` and `fr_last_error`
/// (v2). Valid until the next of those calls.
#[unsafe(no_mangle)]
pub extern "C" fn fr_text_ptr() -> *const u8 {
    lock(&TEXT).as_ptr()
}

/// Writes the canonical scenario JSON an engine of (preset, tier) runs into the text
/// buffer (v2); returns its byte length, 0 for an unknown preset or tier.
#[unsafe(no_mangle)]
pub extern "C" fn fr_scenario_json(preset: u32, tier: u32) -> u32 {
    let (Some(class), Some(tier)) = (preset_class(preset), tier_of(tier)) else {
        return 0;
    };
    match scenario_json(&presets::by_class(class), tier) {
        Ok(s) => put_text(&s),
        Err(e) => {
            set_error(e.to_string());
            0
        }
    }
}

/// Writes anchor `id`'s name into the text buffer (v2); returns its byte length, 0 when
/// unknown.
#[unsafe(no_mangle)]
pub extern "C" fn fr_anchor_name(id: u32) -> u32 {
    anchor_name(id).map_or(0, put_text)
}

/// Writes the last create/step error message into the text buffer (v2); returns its
/// byte length, 0 when there was none.
#[unsafe(no_mangle)]
pub extern "C" fn fr_last_error() -> u32 {
    let msg = lock(&LAST_ERROR).clone();
    if msg.is_empty() { 0 } else { put_text(&msg) }
}

// ---- analytic exports (single-source closed forms) --------------------------------------

/// Open-loop heave pole √(4g₀/(2g* + h′)), rad/s (PHYSICS §3).
#[unsafe(no_mangle)]
pub extern "C" fn fr_open_loop_pole(g_star_m: f64, h_prime_m: f64) -> f64 {
    derive::open_loop_pole(g_star_m, h_prime_m)
}

/// Zero-power mass m*·((2g* + h′)/(2g + h′))², kg (PHYSICS §3).
#[unsafe(no_mangle)]
pub extern "C" fn fr_zero_power_mass(
    gap_m: f64,
    g_star_m: f64,
    h_prime_m: f64,
    m_star_kg: f64,
) -> f64 {
    derive::zero_power_mass(gap_m, g_star_m, h_prime_m, m_star_kg)
}

/// Low-energy speed cap √(2E/m), m/s (PHYSICS §16).
#[unsafe(no_mangle)]
pub extern "C" fn fr_v_low_energy(m_kg: f64, e_j: f64) -> f64 {
    derive::v_low_energy(m_kg, e_j)
}

/// Momentum speed cap p/m, m/s (PHYSICS §16).
#[unsafe(no_mangle)]
pub extern "C" fn fr_momentum_cap(m_kg: f64, p_kgmps: f64) -> f64 {
    derive::momentum_cap(m_kg, p_kgmps)
}

/// Jerk-limited stop distance, m (PHYSICS §16).
#[unsafe(no_mangle)]
pub extern "C" fn fr_stop_distance(v_mps: f64, a_mps2: f64, j_mps3: f64) -> f64 {
    derive::stop_distance(v_mps, a_mps2, j_mps3)
}

/// Clamp force √(2E·k), N (PHYSICS §22).
#[unsafe(no_mangle)]
pub extern "C" fn fr_clamp_force(e_j: f64, k_n_per_m: f64) -> f64 {
    derive::clamp_force(e_j, k_n_per_m)
}

/// Springs in series k1·k2/(k1 + k2), N/m (PHYSICS §22).
#[unsafe(no_mangle)]
pub extern "C" fn fr_series_k(k1_n_per_m: f64, k2_n_per_m: f64) -> f64 {
    derive::series_k(k1_n_per_m, k2_n_per_m)
}

/// Unattended close speed min(v_cap, F/√(m_gov·k)), m/s (plan §4b).
#[unsafe(no_mangle)]
pub extern "C" fn fr_unattended_close_speed(
    v_cap_mps: f64,
    f_pinch_n: f64,
    m_gov_kg: f64,
    k_series_n_per_m: f64,
) -> f64 {
    derive::unattended_close_speed(v_cap_mps, f_pinch_n, m_gov_kg, k_series_n_per_m)
}

/// Terminus arrival time t_b + τ·ln(d_b/d_ε) + τ, s (PHYSICS §16.1).
#[unsafe(no_mangle)]
pub extern "C" fn fr_terminus_t_arrive(v_mps: f64, j_mps3: f64, tau_s: f64, d_eps_m: f64) -> f64 {
    derive::terminus_t_arrive(v_mps, j_mps3, tau_s, d_eps_m)
}

/// Terminus blend start distance d_0, m (PHYSICS §16.1).
#[unsafe(no_mangle)]
pub extern "C" fn fr_terminus_d0(v_mps: f64, j_mps3: f64, tau_s: f64) -> f64 {
    derive::terminus_blend(v_mps, j_mps3, tau_s).d_0_m
}

/// Terminus capture distance d_b, m (PHYSICS §16.1).
#[unsafe(no_mangle)]
pub extern "C" fn fr_terminus_db(v_mps: f64, j_mps3: f64, tau_s: f64) -> f64 {
    derive::terminus_blend(v_mps, j_mps3, tau_s).d_b_m
}

/// Terminus blend duration t_b = √(τ² + 2v/j) − τ, s (PHYSICS §16.1; v2).
#[unsafe(no_mangle)]
pub extern "C" fn fr_terminus_tb(v_mps: f64, j_mps3: f64, tau_s: f64) -> f64 {
    derive::terminus_blend(v_mps, j_mps3, tau_s).t_b_s
}

/// S−P time r/v_s − r/v_p, s (PHYSICS §20).
#[unsafe(no_mangle)]
pub extern "C" fn fr_sp_time(r_hyp_km: f64, v_p_kmps: f64, v_s_kmps: f64) -> f64 {
    derive::sp_time(r_hyp_km, v_p_kmps, v_s_kmps)
}

/// Passive transmissibility (PHYSICS §21).
#[unsafe(no_mangle)]
pub extern "C" fn fr_transmissibility(r: f64, zeta: f64) -> f64 {
    derive::transmissibility(r, zeta)
}

/// Skyhook transmissibility (PHYSICS §16.2).
#[unsafe(no_mangle)]
pub extern "C" fn fr_t_sky(r: f64, zeta: f64) -> f64 {
    derive::t_sky(r, zeta)
}

/// Den Hartog frequency ratio (PHYSICS §21).
#[unsafe(no_mangle)]
pub extern "C" fn fr_den_hartog_f(mu: f64) -> f64 {
    derive::den_hartog(mu).0
}

/// Den Hartog damping ratio (PHYSICS §21).
#[unsafe(no_mangle)]
pub extern "C" fn fr_den_hartog_zeta(mu: f64) -> f64 {
    derive::den_hartog(mu).1
}

/// Warburton (base acceleration) frequency ratio (PHYSICS §21).
#[unsafe(no_mangle)]
pub extern "C" fn fr_warburton_f(mu: f64) -> f64 {
    derive::warburton(mu).0
}

/// Warburton (base acceleration) damping ratio (PHYSICS §21).
#[unsafe(no_mangle)]
pub extern "C" fn fr_warburton_zeta(mu: f64) -> f64 {
    derive::warburton(mu).1
}

#[cfg(test)]
mod tests;
