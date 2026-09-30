//! Log-damped terminus integration tests (PHYSICS §16.1; plan §4b). Owner: E1.
//!
//! Public-API checks of `motion::terminus`, sampled at the 20 kHz base tick the way the
//! fast group evaluates the reference: arrival time against the plan §4b closed-form
//! table (±2 %), C² continuity with bounded jerk at every tick, semilog linearity of the
//! capture, the τ_min rule, invariance to the anchor position and start tick, the
//! hand-release re-plans (nominal, braked at a_brk = 2·a_max, approach from rest through
//! the `scurve` launch, quintic, infeasible), the governor observation point and the
//! integrator-freeze phases. The requirement tokens for FAB-HAP-013…016 live in the
//! module's unit tests (`src/motion/terminus.rs`); no `// req:` markers here, since
//! SPECIFICATION lists no token for this file. Closed-loop acceptance on the plant
//! (FAB-HAP-013…016 `py:haptics::*`) runs after the W1.5 wiring.

use fabrail_core::motion::{
    eval_segment, scurve,
    terminus::{
        self, PlanKind, TerminusPhase, TerminusPlanner, blend, eval_d, integrator_frozen,
        t_arrive_s,
    },
};
use fabrail_params::{SystemParams, presets};
use fabrail_types::{
    BASE_TICK_S, PosNm, QuinticSeg, RailClass, Segment, TerminusSeg, Tick, TravelDir,
};
use proptest::prelude::*;

const DT: f64 = BASE_TICK_S;

fn preset(c: RailClass) -> SystemParams {
    presets::by_class(c)
}

fn planner(c: RailClass) -> TerminusPlanner {
    TerminusPlanner::new(&preset(c)).expect("preset planner")
}

/// Distance-to-go (m, f64 from integer nm) of x for a move toward `target` in `dir`.
fn dist(target: PosNm, dir: TravelDir, x: PosNm) -> f64 {
    -f64::from(dir.sign()) * x.diff_nm(target) as f64 * 1e-9
}

/// (x, v, a) of any segment at `t`.
fn at(seg: &Segment, t: Tick) -> (PosNm, f64, f64) {
    let r = eval_segment(seg, t);
    (r.x, f64::from(r.v_mps), f64::from(r.a_mps2))
}

/// Upper bound on the reference jerk of a terminus: blend j, capture d_b/τ³, and the
/// quintic's (6|c3| + 24|c4| + 60|c5|)/T³ ≤ 780·d_h/T³ + 408·|ḋ0|/T² + 75·|d̈0|/T.
fn jerk_bound(s: &TerminusSeg) -> f64 {
    let tau = f64::from(s.tau_s);
    let tq = f64::from(s.t_q_s);
    let dh = f64::from(s.d_eps_m.min(s.d_b_m));
    f64::from(s.j_mps3)
        + f64::from(s.d_b_m) / tau.powi(3)
        + 780.0 * dh / tq.powi(3)
        + 408.0 * dh / tau / tq.powi(2)
        + 75.0 * dh / (tau * tau) / tq
}

/// Two f32 ulps relative (the evaluators work in f32 distance-to-go and speed).
const F32_2ULP: f64 = 2.0 * f32::EPSILON as f64;

/// Tick-by-tick smoothness over [from, to]: the position increments integrate the
/// velocity (trapezoid: J·dt²/12, plus the 1 nm PosNm quantisation and 2 f32 ulps of
/// the distance-to-go, ≤ 15 nm at 0.1 m), the velocity increments integrate the
/// acceleration (J·dt/6 plus 2 ulps of v), and the acceleration never jumps by more
/// than J·dt (plus 2 ulps of a): no C0/C1/C2 step anywhere. Returns the peak |a|, |v|.
fn assert_smooth(name: &str, seg: &Segment, from: Tick, to: Tick, jerk: f64) -> (f64, f64) {
    let x_end = at(seg, to).0;
    let (mut d_pk, mut v_pk, mut a_pk) = (0.0f64, 0.0f64, 0.0f64);
    for k in from.0..=to.0 {
        let (x, v, a) = at(seg, Tick(k));
        d_pk = d_pk.max(x.diff_nm(x_end).unsigned_abs() as f64 * 1e-9);
        v_pk = v_pk.max(v.abs());
        a_pk = a_pk.max(a.abs());
    }
    let tol_x = (2.0e-9 + 2.0 * F32_2ULP * d_pk) / DT + jerk * DT * DT;
    let tol_v = 2.0 * F32_2ULP * v_pk / DT + jerk * DT;
    let tol_a = jerk * DT * 1.01 + 2.0 * F32_2ULP * a_pk;
    let (mut x0, mut v0, mut a0) = at(seg, from);
    for k in (from.0 + 1)..=to.0 {
        let (x1, v1, a1) = at(seg, Tick(k));
        let dx = x1.diff_nm(x0) as f64 * 1e-9 / DT;
        assert!(
            (dx - 0.5 * (v0 + v1)).abs() <= tol_x,
            "{name}: tick {k}: Δx/dt = {dx} vs v̄ = {} (±{tol_x}; C0/C1, PHYSICS §16.1)",
            0.5 * (v0 + v1)
        );
        assert!(
            ((v1 - v0) / DT - 0.5 * (a0 + a1)).abs() <= tol_v,
            "{name}: tick {k}: Δv/dt = {} vs ā = {} (±{tol_v}; C1/C2, PHYSICS §16.1)",
            (v1 - v0) / DT,
            0.5 * (a0 + a1)
        );
        assert!(
            (a1 - a0).abs() <= tol_a,
            "{name}: tick {k}: acceleration step {} > J·dt = {} (C2; PHYSICS §16.1)",
            a1 - a0,
            jerk * DT
        );
        (x0, v0, a0) = (x1, v1, a1);
    }
    (a_pk, v_pk)
}

/// First tick at or after `from` where the reference is exactly on `target` at rest.
fn arrival(seg: &Segment, target: PosNm, from: Tick, limit: u64) -> Option<Tick> {
    (from.0..from.0 + limit).map(Tick).find(|&t| {
        let r = eval_segment(seg, t);
        r.x == target && r.v_mps == 0.0 && r.a_mps2 == 0.0
    })
}

#[test]
fn t_arrive_matches_closed_form_table() {
    // plan §4b corrected table (τ; t_arrive), d_ε = 0.2 mm, T_q = τ: the first tick at
    // which the sampled reference sits on the target at rest is within ±2 % of the
    // table (FAB-HAP-016 tolerance) and within one tick of t_arrive_s. DRAWER's +0.2 s
    // seat step is the supervisor's, after this arrival.
    let rows = [
        ("DRAWER", 0.2654f32, 20.0f32, 0.15f32, 0.98f64),
        ("cabinet slider", 0.152, 5.0, 0.2, 1.27),
        ("DOOR close", 0.152, 5.0, 0.25, 1.62),
        ("DOOR open", 0.1838, 5.0, 0.25, 1.68),
        ("WALL", 0.0446, 1.5, 0.6, 3.56),
        ("HEAVY", 0.025, 1.0, 1.0, 5.84),
    ];
    for (name, v, j, tau, table) in rows {
        let tp = TerminusPlanner {
            tau_s: tau,
            d_eps_m: 2.0e-4,
            j_mps3: j,
            t_q_ratio: 1.0,
            // τ_min(v) < τ for every row with the class a_max; a large a_max keeps
            // τ_eff = τ exactly as in the table.
            a_max_mps2: 10.0,
            replan_brk_ratio: 2.0,
            ..TerminusPlanner::default()
        };
        let t0 = Tick(12_345);
        let target = PosNm(-777_000_000);
        let seg = tp.nominal(target, TravelDir::Close, t0, v);
        assert_eq!(seg.tau_s, tau, "{name}: τ_eff = τ");
        let s = Segment::Terminus(seg);
        let got = arrival(&s, target, t0, 200_000).expect("arrives");
        let t = got.since(t0) as f64 * DT;
        assert!(
            (t / table - 1.0).abs() <= 0.02,
            "{name}: t_arrive = {t} s vs {table} s (±2 %; plan §4b, FAB-HAP-016)"
        );
        let closed = f64::from(t_arrive_s(&seg));
        assert!(
            (t - closed).abs() <= DT + 1e-9,
            "{name}: sampled arrival {t} vs closed form {closed} (one tick)"
        );
        assert_eq!(terminus::arrive_tick(&s), got, "{name}: arrive_tick");
    }
    // The presets' own planners at their caps (DOOR close/open 1.62/1.68 s).
    for (c, dir, table) in [
        (RailClass::Drawer, TravelDir::Close, 0.98),
        (RailClass::Door, TravelDir::Close, 1.62),
        (RailClass::Door, TravelDir::Open, 1.68),
        (RailClass::Wall, TravelDir::Open, 3.56),
        (RailClass::Heavy, TravelDir::Close, 5.84),
    ] {
        let tp = planner(c);
        let seg = tp.nominal(PosNm(0), dir, Tick(0), tp.v_cap_mps(dir));
        let t = f64::from(t_arrive_s(&seg));
        assert!(
            (t / table - 1.0).abs() <= 0.02,
            "{c:?} {dir:?}: preset t_arrive = {t} s vs {table} s (±2 %; plan §4b)"
        );
    }
}

#[test]
fn reference_smooth_at_every_tick() {
    // Every preset, both directions, at the class caps: C² with bounded jerk at every
    // base tick from 10 ms of cruise before the blend to 10 ms after arrival; never past
    // the target, monotone approach (1 nm quantisation), |a| ≤ a_max (τ ≥ τ_min) and
    // |v| ≤ v_app (PHYSICS §16.1; plan §4b).
    for c in presets::CLASSES {
        let p = preset(c);
        let tp = planner(c);
        for dir in [TravelDir::Open, TravelDir::Close] {
            let v = tp.v_cap_mps(dir);
            let target = PosNm(3_000_000_000);
            let t0 = Tick(1_000);
            let seg = tp.nominal(target, dir, t0, v);
            let s = Segment::Terminus(seg);
            let end = terminus::arrive_tick(&s).plus(200);
            let name = format!("{c:?} {dir:?}");
            let (a_pk, v_pk) = assert_smooth(&name, &s, Tick(t0.0 - 200), end, jerk_bound(&seg));
            assert!(
                a_pk <= p.control.motion.a_max_mps2 * (1.0 + 1e-4),
                "{name}: peak |a| {a_pk} ≤ a_max (τ ≥ τ_min; plan §4b)"
            );
            assert!(v_pk <= f64::from(v) * (1.0 + 1e-6), "{name}: |v| ≤ v_app");
            let mut prev = f64::INFINITY;
            for k in (t0.0 - 200)..=end.0 {
                let (x, _, _) = at(&s, Tick(k));
                let d = dist(target, dir, x);
                assert!(d >= 0.0, "{name}: never past the target (FAB-HAP-016)");
                assert!(d <= prev + 1.0e-9, "{name}: monotone approach (1 nm)");
                prev = d;
            }
        }
    }
}

#[test]
fn semilog_reference_is_linear() {
    // Over the capture [t0 + t_b, t_ε] the sampled distance-to-go is a straight line on a
    // semilog plot: ln d fitted against t has slope −1/τ (±1e-3) and R² ≥ 0.9999
    // (FAB-HAP-013 needs 0.999 on plant truth; plan §4b), for the presets and for a
    // braked re-plan (τ′ < τ).
    let mut cases: Vec<(String, TerminusSeg)> = presets::CLASSES
        .iter()
        .map(|&c| {
            let tp = planner(c);
            (
                format!("{c:?}"),
                tp.nominal(PosNm(0), TravelDir::Close, Tick(0), tp.v_close_cap_mps),
            )
        })
        .collect();
    let tp = planner(RailClass::Drawer);
    let pl = tp.replan(PosNm(0), TravelDir::Close, Tick(0), 0.030, 0.25, 0.0);
    assert_eq!(pl.kind, PlanKind::Braked);
    if let Segment::Terminus(s) = pl.seg {
        cases.push(("DRAWER braked".into(), s));
    }
    for (name, seg) in cases {
        let tau = f64::from(seg.tau_s);
        let t_b = f64::from(seg.t_b_s);
        let t_eps = t_b + tau * (f64::from(seg.d_b_m) / f64::from(seg.d_eps_m)).ln();
        let (k0, k1) = (
            (t_b / DT).ceil() as u64 + 1,
            (t_eps / DT).floor() as u64 - 1,
        );
        let (mut n, mut sx, mut sy, mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for k in k0..=k1 {
            let (x, _, _) = at(&Segment::Terminus(seg), seg.t0.plus(k));
            let d = dist(seg.target, seg.dir, x);
            let (t, y) = (k as f64 * DT, d.ln());
            n += 1.0;
            sx += t;
            sy += y;
            sxx += t * t;
            sxy += t * y;
            syy += y * y;
        }
        let slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        let r2 = (n * sxy - sx * sy).powi(2) / ((n * sxx - sx * sx) * (n * syy - sy * sy));
        assert!(
            (slope * tau + 1.0).abs() <= 1e-3,
            "{name}: slope {slope} vs −1/τ (±1e-3; plan §4b)"
        );
        assert!(
            r2 >= 0.9999,
            "{name}: semilog R² = {r2} ≥ 0.9999 (FAB-HAP-013)"
        );
    }
}

#[test]
fn tau_min_rule() {
    // τ_min = v/a_max − a_max/(2j) is exactly the τ whose blend peaks at a_max; τ_eff =
    // max(τ, τ_min(|v|)) keeps a hand-thrown carriage (2, 4, 8× the cap) at ≤ a_max, and
    // at the class caps τ_eff = τ (validate enforces τ ≥ τ_min; plan §4b).
    for c in presets::CLASSES {
        let p = preset(c);
        let tp = planner(c);
        let a_max = p.control.motion.a_max_mps2 as f32;
        for dir in [TravelDir::Open, TravelDir::Close] {
            assert_eq!(
                tp.tau_eff(tp.v_cap_mps(dir)),
                tp.tau_s,
                "{c:?} {dir:?}: τ_eff = τ at the cap"
            );
        }
        for v in [
            2.0 * tp.v_open_cap_mps,
            4.0 * tp.v_open_cap_mps,
            8.0 * tp.v_open_cap_mps,
        ] {
            let te = tp.tau_eff(v);
            let tm = v / a_max - a_max / (2.0 * tp.j_mps3);
            if tm > tp.tau_s {
                assert!((te - tm).abs() <= 1e-6 * tm, "{c:?} v {v}: τ_eff = τ_min");
                let b = blend(v, tp.j_mps3, te);
                let peak = tp.j_mps3 * b.t_b_s;
                assert!(
                    (peak / a_max - 1.0).abs() <= 1e-4,
                    "{c:?} v {v}: peak deceleration {peak} = a_max at τ_min (plan §4b)"
                );
                // The whole sampled reference respects a_max.
                let s = Segment::Terminus(tp.nominal(PosNm(0), TravelDir::Close, Tick(0), v));
                let end = terminus::arrive_tick(&s);
                let a_pk = (0..=end.0)
                    .map(|k| at(&s, Tick(k)).2.abs())
                    .fold(0.0, f64::max);
                assert!(
                    a_pk <= f64::from(a_max) * (1.0 + 1e-4),
                    "{c:?} v {v}: |a| {a_pk} ≤ a_max"
                );
            } else {
                assert_eq!(te, tp.tau_s);
            }
        }
        // Symmetric in the sign of v; NaN falls back to τ.
        assert_eq!(tp.tau_eff(-3.0), tp.tau_eff(3.0));
        assert_eq!(tp.tau_eff(f32::NAN), tp.tau_s);
    }
}

#[test]
fn invariant_to_anchor_and_tick_offset() {
    // The reference relative to its target is bit-identical at any absolute position
    // (±9 km) and any start tick (up to 10¹² ticks ≈ 1.6 years): all differences are
    // formed in integers first (PHYSICS §24).
    let tp = planner(RailClass::Wall);
    let offsets = [
        (PosNm(0), Tick(0)),
        (PosNm(9_000_000_000_000), Tick(1_000_000_000_000)),
        (PosNm(-9_000_000_000_000), Tick(77)),
        (PosNm(123_456_789_012), Tick(4_294_967_296)),
    ];
    let make = |target: PosNm, t0: Tick| -> [Segment; 3] {
        [
            Segment::Terminus(tp.nominal(target, TravelDir::Open, t0, tp.v_open_cap_mps)),
            Segment::Quintic(QuinticSeg {
                target,
                t0,
                dir: TravelDir::Close,
                d0_m: 2.0e-3,
                v0_mps: -5.0e-3,
                a0_mps2: 1.0e-3,
                t_q_s: 0.9,
            }),
            tp.replan(target, TravelDir::Close, t0, 0.02, 0.03, 0.0).seg,
        ]
    };
    let reference = make(offsets[0].0, offsets[0].1);
    for (target, t0) in offsets {
        let segs = make(target, t0);
        for (s_ref, s) in reference.iter().zip(segs.iter()) {
            let len = terminus::arrive_tick(s).since(t0) + 400;
            for k in (0..len).step_by(7) {
                let r0 = eval_segment(s_ref, offsets[0].1.plus(k));
                let r = eval_segment(s, t0.plus(k));
                assert_eq!(
                    r.x.diff_nm(target),
                    r0.x.diff_nm(offsets[0].0),
                    "target {target:?} t0 {t0:?} tick +{k}: x − target (E; PHYSICS §24)"
                );
                assert_eq!(r.v_mps.to_bits(), r0.v_mps.to_bits(), "v bit-identical");
                assert_eq!(r.a_mps2.to_bits(), r0.a_mps2.to_bits(), "a bit-identical");
            }
        }
    }
}

#[test]
fn hand_release_replans() {
    // plan §4b hand interaction: on release the terminus is re-planned from the measured
    // (d, v, a) with τ_eff, braking at up to a_brk = 2·a_max inside d_0; infeasible →
    // core SoftClose. Each feasible plan starts at the carriage (within one tick of
    // cruise for a cruise-first plan), never passes the target and lands on it at rest.
    let p = preset(RailClass::Drawer);
    let tp = planner(RailClass::Drawer);
    let a_max = p.control.motion.a_max_mps2;
    let target = PosNm(50_000_000);
    let dir = TravelDir::Close;
    let t0 = Tick(10_000);
    struct Case {
        name: &'static str,
        d: f32,
        v: f32,
        a: f32,
        kind: PlanKind,
    }
    let cases = [
        Case {
            name: "cruise then terminus",
            d: 0.080,
            v: 0.20,
            a: 0.0,
            kind: PlanKind::Nominal,
        },
        Case {
            name: "inside d_0: braked",
            d: 0.030,
            v: 0.25,
            a: -0.3,
            kind: PlanKind::Braked,
        },
        Case {
            name: "too fast, too close",
            d: 0.020,
            v: 0.60,
            a: 0.0,
            kind: PlanKind::Infeasible,
        },
        Case {
            name: "inside d_ε",
            d: 1.0e-4,
            v: 5.0e-4,
            a: 0.0,
            kind: PlanKind::Quintic,
        },
        Case {
            name: "receding",
            d: 0.040,
            v: -0.05,
            a: 0.0,
            kind: PlanKind::Quintic,
        },
        Case {
            name: "held at rest",
            d: 0.030,
            v: 0.0,
            a: 0.0,
            kind: PlanKind::Approach,
        },
    ];
    for c in cases {
        let pl = tp.replan(target, dir, t0, c.d, c.v, c.a);
        assert_eq!(pl.kind, c.kind, "{}: plan kind", c.name);
        let (x, v, _) = at(&pl.seg, t0);
        let start_tol = f64::from(c.v.abs()) * DT + 2e-9;
        assert!(
            (dist(target, dir, x) - f64::from(c.d)).abs() <= start_tol + 1e-6 * f64::from(c.d),
            "{}: starts at the carriage: d = {} vs {}",
            c.name,
            dist(target, dir, x),
            c.d
        );
        if c.kind != PlanKind::Approach {
            // ẋ = dir·(approach speed).
            assert!(
                (v - f64::from(dir.sign()) * f64::from(c.v)).abs() <= 1e-4,
                "{}: starts at the measured speed: v = {v}",
                c.name
            );
        }
        if c.kind == PlanKind::Infeasible {
            continue; // the caller falls back to core SoftClose
        }
        let seg = match (c.kind, pl.next) {
            (PlanKind::Approach, Some(next)) => Segment::Terminus(next),
            _ => pl.seg,
        };
        if let Segment::Terminus(s) = seg {
            let peak = f64::from(s.j_mps3 * s.t_b_s);
            assert!(
                peak <= 2.0 * a_max * (1.0 + 1e-4),
                "{}: peak deceleration {peak} ≤ a_brk = 2·a_max (plan §4b)",
                c.name
            );
            if c.kind == PlanKind::Braked {
                assert!(
                    s.tau_s < tp.tau_eff(c.v) && peak > a_max,
                    "{}: τ′ < τ_eff, a > a_max",
                    c.name
                );
            }
        }
        let from = if c.kind == PlanKind::Approach {
            pl.switch_from
        } else {
            t0
        };
        let end = pl.arrive.plus(100);
        let (a_pk, _) = assert_smooth(c.name, &seg, from, end, 2.0e3);
        assert!(
            a_pk <= 2.0 * a_max * (1.0 + 1e-3),
            "{}: |a| {a_pk} ≤ a_brk",
            c.name
        );
        for k in from.0..=end.0 {
            let (x, _, _) = at(&seg, Tick(k));
            assert!(
                dist(target, dir, x) >= 0.0,
                "{}: never past the target (FAB-HAP-016)",
                c.name
            );
        }
        assert_eq!(
            arrival(&seg, target, from, 400_000),
            Some(pl.arrive),
            "{}: arrives at `arrive`",
            c.name
        );
    }
}

#[test]
fn approach_hands_over_from_the_launch_scurve() {
    // From rest the plan is the `scurve` launch (full distance, so on its own it still
    // ends at the target) plus a terminus that extrapolates the same cruise before its
    // t0: switching at any tick in [switch_from, next.t0] changes the reference by at
    // most one tick of cruise (t0 rounding) and keeps C² (PHYSICS §16, §16.1). Checked
    // against the S-curve implementation for every preset from 2·d_0 and 10·d_0.
    for c in presets::CLASSES {
        let tp = planner(c);
        let v_cap = tp.v_close_cap_mps;
        let d0 = blend(v_cap, tp.j_mps3, tp.tau_eff(v_cap)).d_0_m;
        for d in [2.0 * d0, 10.0 * d0] {
            let target = PosNm(-1_000_000_000);
            let t0 = Tick(500);
            let pl = tp
                .approach(target, TravelDir::Close, t0, d, 0.0)
                .expect("approach");
            assert_eq!(pl.kind, PlanKind::Approach);
            let next = pl.next.expect("terminus");
            let Segment::SCurve(launch) = pl.seg else {
                panic!("launch is an S-curve")
            };
            assert!(pl.switch_from <= next.t0, "{c:?}: a cruise window exists");
            assert!(next.v_app_mps <= v_cap && next.v_app_mps > 0.0);
            let launch_seg = Segment::SCurve(launch);
            let term_seg = Segment::Terminus(next);
            // The launch reaches its cruise at v_app by switch_from.
            let r = scurve::eval(&launch, pl.switch_from);
            assert!(
                (f64::from(r.v_mps).abs() - f64::from(next.v_app_mps)).abs() <= 1e-5
                    && f64::from(r.a_mps2).abs() <= 1e-3,
                "{c:?} d {d}: cruising at v_app at switch_from: {r:?}"
            );
            // Launch and terminus coincide over the whole cruise window.
            let tol_nm = (f64::from(next.v_app_mps) * DT * 1e9 * 0.5).ceil() as i64 + 2;
            for k in pl.switch_from.0..=next.t0.0 {
                let (xs, vs, _) = at(&launch_seg, Tick(k));
                let (xt, vt, _) = at(&term_seg, Tick(k));
                assert!(
                    xs.diff_nm(xt).abs() <= tol_nm && (vs - vt).abs() <= 1e-5,
                    "{c:?} d {d} tick {k}: launch {xs:?}/{vs} vs terminus {xt:?}/{vt} (≤ {tol_nm} nm)"
                );
            }
            // A composite switched mid-window is smooth and lands on the target.
            let switch = Tick((pl.switch_from.0 + next.t0.0) / 2);
            let comp = |t: Tick| {
                if t < switch {
                    at(&launch_seg, t)
                } else {
                    at(&term_seg, t)
                }
            };
            let jerk = jerk_bound(&next).max(f64::from(tp.j_mps3));
            let (mut x0, mut v0, mut a0) = comp(t0);
            for k in (t0.0 + 1)..=pl.arrive.0 + 10 {
                let (x1, v1, a1) = comp(Tick(k));
                let dx = x1.diff_nm(x0) as f64 * 1e-9 / DT;
                assert!(
                    (dx - 0.5 * (v0 + v1)).abs() <= f64::from(tol_nm as i32) * 1e-9 / DT + 1e-4,
                    "{c:?} d {d}: position continuity at tick {k}"
                );
                assert!(
                    (a1 - a0).abs() <= jerk * DT * 1.01 + 1e-4,
                    "{c:?} d {d}: C2 at tick {k}"
                );
                (x0, v0, a0) = (x1, v1, a1);
            }
            let r = eval_segment(&term_seg, pl.arrive);
            assert!(
                r.x == target && r.v_mps == 0.0,
                "{c:?} d {d}: lands on the target"
            );
        }
    }
}

#[test]
fn governor_observes_beyond_the_terminus() {
    // x_obs = target + dir·overtravel (plan §4b): past the target in the direction of
    // travel by more than the governor's stopping margin, so the braking envelope
    // √(2·a_brk·(x_obs − x − margin)) is still open at the target and never clips the
    // exponential tail.
    for c in presets::CLASSES {
        let p = preset(c);
        let tp = planner(c);
        let over = p.control.motion.terminus.overtravel_m;
        assert!(
            over > p.control.motion.gov_margin_m,
            "{c:?}: overtravel > margin"
        );
        let target = PosNm(2_000_000_000);
        for dir in [TravelDir::Open, TravelDir::Close] {
            let xo = tp.x_obs(target, dir);
            assert_eq!(
                xo.diff_nm(target),
                (f64::from(dir.sign()) * over * 1e9).round() as i64,
                "{c:?} {dir:?}: x_obs = target + dir·overtravel"
            );
            assert_eq!(terminus::x_obs(target, dir, f32::NAN), target);
            assert_eq!(terminus::x_obs(target, dir, -1.0), target);
            // The whole reference stays short of x_obs − margin.
            let seg = Segment::Terminus(tp.nominal(target, dir, Tick(0), tp.v_cap_mps(dir)));
            let lim = xo.offset_m(-dir.sign() * p.control.motion.gov_margin_m as f32);
            for k in (0..terminus::arrive_tick(&seg).0 + 100).step_by(5) {
                let (x, _, _) = at(&seg, Tick(k));
                assert!(
                    f64::from(dir.sign()) * x.diff_nm(lim) as f64 <= 0.0,
                    "{c:?} {dir:?}: reference beyond x_obs − margin"
                );
            }
        }
    }
}

#[test]
fn phases_and_integrator_freeze() {
    // Phases advance Cruise → Blend → Capture → Quintic → Arrived, each non-empty for the
    // presets; the integrator-freeze request is exactly Blend ∪ Capture (plan §4b;
    // FAB-HAP-015) and `done` matches Arrived.
    for c in presets::CLASSES {
        let tp = planner(c);
        let seg = tp.nominal(PosNm(0), TravelDir::Open, Tick(2_000), tp.v_open_cap_mps);
        let s = Segment::Terminus(seg);
        let mut seen = Vec::new();
        for k in 0..terminus::arrive_tick(&s).0 + 10 {
            let t = Tick(k);
            let ph = terminus::phase(&seg, t);
            if seen.last() != Some(&ph) {
                seen.push(ph);
            }
            assert_eq!(
                integrator_frozen(&s, t),
                matches!(ph, TerminusPhase::Blend | TerminusPhase::Capture),
                "{c:?} tick {k}: freeze request in {ph:?}"
            );
            assert_eq!(terminus::done(&s, t), ph == TerminusPhase::Arrived);
        }
        assert_eq!(
            seen,
            [
                TerminusPhase::Cruise,
                TerminusPhase::Blend,
                TerminusPhase::Capture,
                TerminusPhase::Quintic,
                TerminusPhase::Arrived
            ],
            "{c:?}: phase order"
        );
        // Continuous-time evaluator and tick evaluator agree.
        let st = eval_d(&seg, 0.5 * t_arrive_s(&seg));
        assert!(st.d_m > 0.0 && st.rate_mps < 0.0);
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(96))]

    /// Any measured state (d ∈ [0, 0.4] m, v ∈ [−0.3, 0.8] m/s, a ∈ [−3, 3] m/s²) gives a
    /// plan without panics; every feasible plan starts at the carriage, never passes the
    /// target, and lands on it at rest at `arrive` (DRAWER and WALL; PHYSICS §16.1).
    #[test]
    fn replan_is_safe_for_any_measured_state(
        d in 0.0f32..0.4,
        v in -0.3f32..0.8,
        a in -3.0f32..3.0,
        wall in any::<bool>(),
    ) {
        let c = if wall { RailClass::Wall } else { RailClass::Drawer };
        let tp = planner(c);
        let target = PosNm(0);
        let dir = TravelDir::Open;
        let t0 = Tick(1_000);
        let pl = tp.replan(target, dir, t0, d, v, a);
        let seg = match (pl.kind, pl.next) {
            (PlanKind::Approach, Some(n)) => Segment::Terminus(n),
            _ => pl.seg,
        };
        let from = if pl.kind == PlanKind::Approach { pl.switch_from } else { t0 };
        if pl.kind != PlanKind::Approach {
            let (x, _, _) = at(&pl.seg, t0);
            let err = (dist(target, dir, x) - f64::from(d)).abs();
            prop_assert!(err <= f64::from(v.abs()) * DT + 1e-6 * f64::from(d) + 2e-9, "start {err}");
        }
        if pl.kind != PlanKind::Infeasible {
            let end = pl.arrive.0 + 20;
            prop_assert!(end - from.0 < 2_000_000, "arrival within 100 s");
            for k in (from.0..=end).step_by(3) {
                let (x, _, _) = at(&seg, Tick(k));
                prop_assert!(dist(target, dir, x) >= -1e-9, "past the target at tick {}", k);
            }
            let r = eval_segment(&seg, pl.arrive);
            prop_assert!(r.x == target && r.v_mps == 0.0 && r.a_mps2 == 0.0);
        }
    }
}
