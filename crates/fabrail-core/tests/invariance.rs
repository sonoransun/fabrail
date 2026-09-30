//! Controller invariance under x → x + L and tick → tick + offset (PHYSICS §24;
//! FAB-PRP-008; core design §A.4 items 2–3). Owner: A2.
//!
//! The fast loop uses absolute positions only as integer `PosNm` differences and time
//! only as tick differences, so shifting every position (measurements, segment
//! anchors/targets, observation points, spring anchor, detents, isolation centre,
//! standoff, and the parameter-defined rail origin `rail.x_closed_m` with the safety
//! position limits, which fix the DRAWER seat) by L ≤ 15 m and every tick by up to 2⁴⁰
//! must give bit-identical outputs.
//! With the ripple table on, L is a multiple of the electrical period 2τ_p (the
//! electrical angle is an exact integer modulo); with it off, L is arbitrary.
//!
//! The measurement stream is synthesised open-loop in integer nanometres (a seeded
//! random walk) so the shifted stream is exact; the loop's internal state evolves
//! through every mode family (hold, planned S-curve, terminus, quintic, admittance,
//! soft-close, freeze) with mode changes mid-run.

use fabrail_core::fast::FastLoop;
use fabrail_core::motion::scurve;
use fabrail_core::msg::{
    Directive, FastOutputs, FastStatus, HapticProfile, LevDirective, LiveLimits, Measurements,
    TerminusCfg,
};
use fabrail_params::{SystemParams, presets};
use fabrail_types::{
    CoilCmd, MAX_MAGNETS, MotionMode, PosNm, QuinticSeg, Segment, TerminusSeg, Tick, TravelDir,
};
use proptest::prelude::*;

/// Deterministic generator for the synthetic measurement stream.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0 >> 11
    }

    /// Uniform in [−1, 1).
    fn unit(&mut self) -> f32 {
        (self.next() as f64 / (1u64 << 53) as f64 * 2.0 - 1.0) as f32
    }
}

fn class(i: usize) -> SystemParams {
    [
        presets::drawer(),
        presets::door(),
        presets::wall(),
        presets::heavy(),
    ][i]
}

/// The directive for mode family `k` around the base position `x0`, starting at `t0`.
fn directive(p: &SystemParams, k: u8, x0: PosNm, t0: Tick, unattended: bool) -> Directive {
    let mo = &p.control.motion;
    let tm = &mo.terminus;
    let sf = &p.safety;
    let travel = p.design.rail.travel_m as f32;
    let limits = LiveLimits {
        v_open_mps: sf.v_open_max_mps as f32,
        v_close_mps: sf.v_close_max_mps as f32,
        a_max_mps2: mo.a_max_mps2 as f32,
        f_drive_n: sf.f_drive_max_n as f32,
        f_static_n: sf.f_static_n as f32,
        f_close_last50_n: sf.f_close_last50_n as f32,
        x_obs_open: x0.offset_m(travel),
        x_obs_close: x0.offset_m(-0.2 * travel),
        hold_to_run: false,
        unattended,
    };
    let m_v = (p.design.carriage.m_rated_kg / p.control.haptic.mass_ratio) as f32;
    let dt = fabrail_params::derive::fast_period_s(p);
    let k_v =
        (p.control.haptic.k_virtual_frac * 2.0 * p.design.damping.b_dist_n_s_per_m / dt) as f32;
    let haptic = HapticProfile {
        m_v_kg: m_v,
        b_v_n_s_per_m: p.control.haptic.b_v_n_s_per_m as f32,
        k_v_n_per_m: if k == 5 { 0.0 } else { 0.1 * k_v },
        x0: x0.offset_m(0.01),
        n_detents: 1,
        detents: [x0.offset_m(0.02), PosNm::ZERO, PosNm::ZERO, PosNm::ZERO],
        detent_f_n: p.control.haptic.detent_f_n as f32,
        detent_width_m: p.control.haptic.detent_width_m as f32,
        soft_close: k == 5,
        passivity: true,
        intent_enable: true,
        push_to_open: p.control.intent.push_to_open,
    };
    let terminus = TerminusCfg {
        tau_s: tm.tau_s as f32,
        d_eps_m: tm.d_eps_m as f32,
        j_mps3: tm.j_mps3 as f32,
        t_q_ratio: tm.t_q_ratio as f32,
        overtravel_m: tm.overtravel_m as f32,
        a_brk_replan_mps2: (tm.replan_brk_ratio * mo.a_max_mps2) as f32,
        hand_onset_s: p.control.haptic.hand_onset_s as f32,
        hand_yields: true,
        seat_on_arrival: false,
        freeze_integrator: true,
    };
    let v_cap = sf.v_open_max_mps.min(sf.v_close_max_mps) as f32;
    let (mode, segment) = match k {
        0 => (
            MotionMode::Hold,
            Segment::SCurve(scurve::plan(x0, t0, x0, 0.1, 1.0, 1.0)),
        ),
        1 => (
            MotionMode::Trajectory,
            Segment::SCurve(scurve::plan(
                x0,
                t0,
                x0.offset_m(0.3 * travel),
                v_cap,
                mo.a_max_mps2 as f32,
                mo.j_max_mps3 as f32,
            )),
        ),
        2 => (
            MotionMode::Terminus,
            Segment::Terminus(TerminusSeg {
                target: x0.offset_m(-0.05),
                t0,
                dir: TravelDir::Close,
                v_app_mps: 0.5 * v_cap,
                j_mps3: tm.j_mps3 as f32,
                tau_s: tm.tau_s as f32,
                t_b_s: 0.02,
                d_b_m: 0.03,
                d_0_m: 0.04,
                d_eps_m: tm.d_eps_m as f32,
                t_q_s: tm.tau_s as f32,
            }),
        ),
        3 => (
            MotionMode::SoftOpen,
            Segment::Quintic(QuinticSeg {
                target: x0.offset_m(0.004),
                t0,
                dir: TravelDir::Open,
                d0_m: 0.004,
                v0_mps: -0.01,
                a0_mps2: 0.02,
                t_q_s: tm.tau_s as f32,
            }),
        ),
        4 => (MotionMode::Admittance, Segment::default()),
        5 => (
            MotionMode::SoftClose,
            Segment::SCurve(scurve::plan(x0, t0, x0, 0.1, 1.0, 1.0)),
        ),
        _ => (MotionMode::Freeze, Segment::default()),
    };
    Directive {
        seq: u32::from(k),
        lev: LevDirective::Levitate,
        mode,
        segment,
        terminus,
        haptic,
        limits,
        ..Directive::default()
    }
}

/// Shifts every absolute position in `d` by `l_nm` and every tick by `off`.
fn shift(d: &Directive, l_nm: i64, off: u64) -> Directive {
    let sx = |x: PosNm| PosNm(x.0 + l_nm);
    let st = |t: Tick| Tick(t.0 + off);
    let mut o = *d;
    o.segment = match d.segment {
        Segment::SCurve(mut s) => {
            s.anchor = sx(s.anchor);
            s.t0 = st(s.t0);
            Segment::SCurve(s)
        }
        Segment::Terminus(mut s) => {
            s.target = sx(s.target);
            s.t0 = st(s.t0);
            Segment::Terminus(s)
        }
        Segment::Quintic(mut s) => {
            s.target = sx(s.target);
            s.t0 = st(s.t0);
            Segment::Quintic(s)
        }
    };
    o.limits.x_obs_open = sx(d.limits.x_obs_open);
    o.limits.x_obs_close = sx(d.limits.x_obs_close);
    o.haptic.x0 = sx(d.haptic.x0);
    for (a, b) in o.haptic.detents.iter_mut().zip(d.haptic.detents.iter()) {
        *a = sx(*b);
    }
    o.isolate.center = sx(d.isolate.center);
    o.inertial.standoff = sx(d.inertial.standoff);
    o
}

fn coil_bits(c: &CoilCmd) -> (u8, [u32; MAX_MAGNETS]) {
    let tag = match c {
        CoilCmd::Off => 0,
        CoilCmd::CurrentRef(_) => 1,
        CoilCmd::Voltage(_) => 2,
    };
    (tag, c.values().map(f32::to_bits))
}

/// Everything in (outputs, status) that must be bit-identical, with positions taken
/// relative to the run's base.
fn fingerprint(o: &FastOutputs, s: &FastStatus, base: i64) -> Vec<u64> {
    let (tag, coil) = coil_bits(&o.coil);
    let mut v: Vec<u64> = vec![
        u64::from(tag),
        u64::from(o.lsm.iq_ref_a.to_bits()),
        u64::from(o.lsm.id_ref_a.to_bits()),
        u64::from(o.lsm.mode.as_u8()),
        u64::from(s.seq),
        u64::from(s.directive_seq),
        u64::from(s.mode.as_u8()),
        (s.x.0 - base) as u64,
        (s.x_ref.0 - base) as u64,
        u64::from(s.v_mps.to_bits()),
        u64::from(s.a_mps2.to_bits()),
        u64::from(s.v_ref_mps.to_bits()),
        u64::from(s.iq_a.to_bits()),
        u64::from(s.f_hand_hat_n.to_bits()),
        u64::from(s.residual_n.to_bits()),
        u64::from(s.m_hat_kg.to_bits()),
        u64::from(s.e_kin_gov_j.to_bits()),
        u64::from(s.e_passivity_j.to_bits()),
        u64::from(s.a_floor_x_mps2.to_bits()),
        u64::from(s.intent.as_u8()),
        u64::from(s.hand),
        u64::from(s.obstruction),
        u64::from(s.obstruction_degraded),
        u64::from(s.arrived),
        u64::from(s.lev.as_u8()),
        u64::from(s.seat.as_u8()),
        u64::from(s.sat_flags),
        u64::from(s.referenced),
    ];
    v.extend(coil.iter().map(|&b| u64::from(b)));
    v.extend(s.modal.iter().map(|m| u64::from(m.to_bits())));
    v.extend(s.faults.bits.iter().map(|&b| u64::from(b)));
    v
}

/// Runs the phases on a fresh loop; returns one fingerprint per release.
#[allow(clippy::too_many_arguments)]
fn run(
    p: &SystemParams,
    phases: &[(u8, u16)],
    seed: u64,
    base_nm: i64,
    l_nm: i64,
    off: u64,
    unattended: bool,
) -> Vec<Vec<u64>> {
    // The whole world moves: the rail origin (seat, closed stop) shifts with it.
    let mut p = *p;
    let l_m = l_nm as f64 * 1e-9;
    p.design.rail.x_closed_m += l_m;
    p.safety.x_min_m += l_m;
    p.safety.x_max_m += l_m;
    fabrail_params::crc::seal(&mut p);
    let p = &p;
    let mut f = FastLoop::new(p).unwrap_or_default();
    let decim = u64::from(p.control.rates.fast_decim.max(1));
    let g_star = p.design.suspension.g_star_m as f32;
    let i_max = p.design.lsm.i_max_a as f32;
    let mut rng = Lcg(seed);
    let mut x_nm = base_nm;
    let mut tick = 0u64;
    let mut out = Vec::new();
    for &(k, n) in phases {
        let d0 = directive(p, k, PosNm(base_nm), Tick(tick), unattended);
        let d = shift(&d0, l_nm, off);
        for _ in 0..n {
            // Random walk: ±3 µm per release with a slow drift, occasional 20 µm jumps.
            let r = rng.unit();
            x_nm += (r * 3_000.0) as i64 + 400;
            if rng.next().is_multiple_of(997) {
                x_nm += 20_000;
            }
            let m = Measurements {
                tick: Tick(tick + off),
                gap_m: [g_star; MAX_MAGNETS],
                gap_ok: 0xff,
                coil_i_a: [0.1 * rng.unit(); MAX_MAGNETS],
                x: PosNm(x_nm + l_nm),
                referenced: true,
                enc_ok: !rng.next().is_multiple_of(1_000),
                iq_a: 0.5 * i_max * rng.unit(),
                id_a: 0.0,
                imu_mps2: [0.5 * rng.unit(), 0.1 * rng.unit(), 9.81],
                imu_ok: true,
                rail_mps2: [0.5 * rng.unit(), 0.1 * rng.unit(), 0.0],
                rail_ok: true,
            };
            let (o, s) = f.step(&m, &d);
            out.push(fingerprint(&o, &s, base_nm + l_nm));
            tick += decim;
        }
    }
    out
}

// req: FAB-PRP-008
proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]
    /// Every output and status field is bit-identical when all positions shift by
    /// L ≤ 15 m (a multiple of 2τ_p with the ripple table on, arbitrary with it off)
    /// and all ticks by up to 2⁴⁰, across mode changes. (E) bit-exact, PHYSICS §24.
    #[test]
    fn offset_invariance(
        cls in 0usize..4,
        ripple in any::<bool>(),
        l_frac in 0.0f64..1.0,
        off in 0u64..=(1u64 << 40),
        seed in any::<u64>(),
        base_mm in -2_000i64..2_000,
        unattended in any::<bool>(),
        phases in prop::collection::vec((0u8..7, 50u16..600), 1..4),
    ) {
        let mut p = class(cls);
        p.control.motion.ripple_ff = ripple;
        let period_nm = (2.0 * p.design.lsm.tau_p_m * 1e9).round() as i64;
        let l_max = 15_000_000_000i64;
        let l_nm = if ripple {
            let k = ((l_frac * l_max as f64) as i64) / period_nm;
            k * period_nm
        } else {
            (l_frac * l_max as f64) as i64
        };
        let base = base_mm * 1_000_000;
        let a = run(&p, &phases, seed, base, 0, 0, unattended);
        let b = run(&p, &phases, seed, base, l_nm, off, unattended);
        prop_assert_eq!(a.len(), b.len());
        for (k, (fa, fb)) in a.iter().zip(b.iter()).enumerate() {
            prop_assert_eq!(fa, fb,
                "release {} differs under x + {} nm, tick + {} ((E) bit-exact, FAB-PRP-008)",
                k, l_nm, off);
        }
    }
}

/// Maximal shifts (15 m, 2⁴⁰ ticks) on every class and mode family, ripple table on.
#[test]
fn offset_invariance_extremes() {
    for cls in 0..4 {
        let p = class(cls);
        let period_nm = (2.0 * p.design.lsm.tau_p_m * 1e9).round() as i64;
        let l_nm = (15_000_000_000 / period_nm) * period_nm;
        let phases: Vec<(u8, u16)> = (0u8..7).map(|k| (k, 400)).collect();
        let a = run(&p, &phases, 42, 100_000_000, 0, 0, false);
        let b = run(&p, &phases, 42, 100_000_000, l_nm, 1u64 << 40, false);
        assert_eq!(
            a, b,
            "class {cls}: not invariant under 15 m and 2^40 ticks (FAB-PRP-008)"
        );
    }
}
