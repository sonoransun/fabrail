//! Linear closed-loop checks of the IMPLEMENTED levitation loops (LEV-007, LEV-020).
//! Owner: A1.
//!
//! Method (PHYSICS §15; plan §2b item 1). For every class and every controlled mode the
//! per-mode loop that `Levitation::new` builds (the f32 observer at 4ω plus the Mizuno
//! zero-power law, exactly the code `Levitation::step` runs in zero power) is probed into
//! a discrete state-space model (A_c, B_c, C_c, D_c) — the loop is linear inside the
//! zero-power band, so unit-probe columns are exact to f32 rounding. It is closed with
//! the exact ZOH design plant x = [δg, δġ, i, i_e] (`design::zoh_plant`: the current
//! loop lag τ_cl = 1/(2π·1 kHz), the eddy lag τ_e) and one fast period of LET delay on
//! the command (u applied over [k+1, k+2]). Eigenvalues come from a self-contained
//! Hessenberg QR (the core crate carries no linear-algebra dependency; Press et al.,
//! "Numerical Recipes in C", 2nd ed., §11.5–11.6: balanc, elmhes, hqr).
//!
//! - `discrete_poles_match_design` (LEV-007): at the nominal τ_e every pole of the
//!   discrete design (`derive::zp_closed_loop_poles`, including the three placed targets
//!   of (s² + 2ζωs + ω²)(s + αω) at ω = 2.5·p) appears in the implemented loop within 2 %
//!   (|s − s_d|/|s_d| with s = ln z/T), and every complex closed-loop pole has ζ ≥ 0.5.
//! - `stable_for_eddy_lag_to_2ms` (LEV-020): with the gains and observer unchanged, the
//!   loop stays stable (spectral radius < 1) for every plant τ_e in 0…2 ms.
//! - `stable_across_float_band` (LEV-007): with the plant linearised at the zero-power
//!   equilibrium of every gap in the modified band (mass m(g) = m*·((2g* + h′)/(2g + h′))²,
//!   inertia scaled with the mass), the loop stays stable.

// Dense numerical kernels (balance, Hessenberg, QR) read best with explicit indices.
#![allow(clippy::needless_range_loop, clippy::manual_memcpy)]

use fabrail_core::levitation::design::zoh_plant;
use fabrail_core::levitation::{Levitation, ModeLoop};
use fabrail_params::derive::{self, G0_MPS2, LevMode};
use fabrail_params::{SystemParams, presets};

type Mat = Vec<Vec<f64>>;

/// A z-plane pole (re, im).
type Zc = (f64, f64);

/// Controller state order: [pred δg, pred δġ, pred i_e, d̂, u_applied, q].
const NC: usize = 6;

fn classes() -> Vec<(&'static str, SystemParams)> {
    vec![
        ("drawer", presets::drawer()),
        ("door", presets::door()),
        ("wall", presets::wall()),
        ("heavy", presets::heavy()),
    ]
}

fn get_state(l: &ModeLoop) -> [f64; NC] {
    [
        f64::from(l.obs.pred[0]),
        f64::from(l.obs.pred[1]),
        f64::from(l.obs.pred[2]),
        f64::from(l.obs.d_hat),
        f64::from(l.obs.u_applied),
        f64::from(l.zp.q_a_s),
    ]
}

fn set_state(l: &mut ModeLoop, x: &[f64; NC]) {
    l.obs.pred = [x[0] as f32, x[1] as f32, x[2] as f32];
    l.obs.d_hat = x[3] as f32;
    l.obs.u_applied = x[4] as f32;
    l.zp.q_a_s = x[5] as f32;
    l.obs.primed = true;
    l.zp.x_outer_a = 0.0;
    l.zp.i_bias_a = 0.0;
}

/// Probes the loop into (A_c, B_c, C_c, D_c); inputs [δg, i].
fn probe(template: &ModeLoop) -> (Mat, Mat, Vec<f64>, [f64; 2]) {
    // Probe scales: small enough to stay inside the zero-power band (the band outer loop
    // stays inactive), large enough to keep f32 rounding at ≈ 1e-7 relative.
    let scale = [1e-5, 1e-3, 1e-2, 1e-2, 1e-2, 1e-4];
    let mut a = vec![vec![0.0; NC]; NC];
    let mut b = vec![vec![0.0; 2]; NC];
    let mut c = vec![0.0; NC];
    let mut d = [0.0; 2];
    for j in 0..NC {
        let mut l = *template;
        let mut x = [0.0; NC];
        x[j] = scale[j];
        set_state(&mut l, &x);
        let u = f64::from(l.step_zp(0.0, 0.0));
        let nx = get_state(&l);
        for i in 0..NC {
            a[i][j] = nx[i] / scale[j];
        }
        c[j] = u / scale[j];
    }
    for (k, s) in [1e-5f64, 1e-2].iter().enumerate() {
        let mut l = *template;
        set_state(&mut l, &[0.0; NC]);
        let (y, i) = if k == 0 { (*s, 0.0) } else { (0.0, *s) };
        let u = f64::from(l.step_zp(y as f32, i as f32));
        let nx = get_state(&l);
        for r in 0..NC {
            b[r][k] = nx[r] / s;
        }
        d[k] = u / s;
    }
    (a, b, c, d)
}

/// Closed-loop matrix: plant [δg, δġ, i, i_e], delayed command u_d, controller (NC).
fn closed_loop(
    ctrl: &(Mat, Mat, Vec<f64>, [f64; 2]),
    p2: f64,
    bb: f64,
    t: f64,
    tau_cl: f64,
    tau_e: f64,
) -> Mat {
    let (ac, bc, cc, dc) = ctrl;
    let pl = zoh_plant(p2, bb, t, tau_cl, tau_e);
    let n = 5 + NC;
    let mut m = vec![vec![0.0; n]; n];
    for r in 0..4 {
        for c in 0..4 {
            m[r][c] = pl.phi[r][c];
        }
        m[r][4] = pl.gam[r];
    }
    // u_d⁺ = u = C_c x_c + D_c [δg; i].
    for j in 0..NC {
        m[4][5 + j] = cc[j];
    }
    m[4][0] += dc[0];
    m[4][2] += dc[1];
    // x_c⁺ = A_c x_c + B_c [δg; i].
    for r in 0..NC {
        for j in 0..NC {
            m[5 + r][5 + j] = ac[r][j];
        }
        m[5 + r][0] += bc[r][0];
        m[5 + r][2] += bc[r][1];
    }
    m
}

// ---- eigenvalues: balanc + elmhes + hqr (Numerical Recipes in C, 2nd ed.) ----------

fn balance(a: &mut Mat) {
    let n = a.len();
    let radix = 2.0f64;
    let sqrdx = radix * radix;
    loop {
        let mut done = true;
        for i in 0..n {
            let (mut c, mut r) = (0.0f64, 0.0f64);
            for j in 0..n {
                if j != i {
                    c += a[j][i].abs();
                    r += a[i][j].abs();
                }
            }
            if c != 0.0 && r != 0.0 {
                let mut g = r / radix;
                let mut f = 1.0;
                let s = c + r;
                while c < g {
                    f *= radix;
                    c *= sqrdx;
                }
                g = r * radix;
                while c > g {
                    f /= radix;
                    c /= sqrdx;
                }
                if (c + r) / f < 0.95 * s {
                    done = false;
                    let g = 1.0 / f;
                    for j in 0..n {
                        a[i][j] *= g;
                    }
                    for j in 0..n {
                        a[j][i] *= f;
                    }
                }
            }
        }
        if done {
            break;
        }
    }
}

fn hessenberg(a: &mut Mat) {
    let n = a.len();
    if n < 3 {
        return;
    }
    for m in 1..n - 1 {
        let mut x = 0.0f64;
        let mut i = m;
        for j in m..n {
            if a[j][m - 1].abs() > x.abs() {
                x = a[j][m - 1];
                i = j;
            }
        }
        if i != m {
            for j in (m - 1)..n {
                let tmp = a[i][j];
                a[i][j] = a[m][j];
                a[m][j] = tmp;
            }
            for row in a.iter_mut() {
                row.swap(i, m);
            }
        }
        if x != 0.0 {
            for i in (m + 1)..n {
                let mut y = a[i][m - 1];
                if y != 0.0 {
                    y /= x;
                    a[i][m - 1] = y;
                    for j in m..n {
                        a[i][j] -= y * a[m][j];
                    }
                    for j in 0..n {
                        a[j][m] += y * a[j][i];
                    }
                }
            }
        }
    }
    for i in 0..n {
        for j in 0..n {
            if i > j + 1 {
                a[i][j] = 0.0;
            }
        }
    }
}

/// Eigenvalues of an upper Hessenberg matrix (NR hqr, ported with 1-based indices).
#[allow(unused_assignments, clippy::needless_range_loop)]
fn hqr(h: &Mat) -> Vec<(f64, f64)> {
    let n = h.len();
    let mut a = vec![vec![0.0f64; n + 1]; n + 1];
    for i in 0..n {
        for j in 0..n {
            a[i + 1][j + 1] = h[i][j];
        }
    }
    let mut wr = vec![0.0f64; n + 1];
    let mut wi = vec![0.0f64; n + 1];
    let sign = |a: f64, b: f64| if b >= 0.0 { a.abs() } else { -a.abs() };
    let mut anorm = 0.0;
    for i in 1..=n {
        for j in (i.max(2) - 1)..=n {
            anorm += a[i][j].abs();
        }
    }
    let mut nn = n as i64;
    let mut t = 0.0f64;
    let (mut p, mut q, mut r, mut s, mut w, mut x, mut y, mut z) = (
        0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64, 0.0f64,
    );
    while nn >= 1 {
        let mut its = 0;
        loop {
            let nnu = nn as usize;
            let mut l = nnu;
            while l >= 2 {
                s = a[l - 1][l - 1].abs() + a[l][l].abs();
                if s == 0.0 {
                    s = anorm;
                }
                if a[l][l - 1].abs() + s == s {
                    a[l][l - 1] = 0.0;
                    break;
                }
                l -= 1;
            }
            x = a[nnu][nnu];
            if l == nnu {
                wr[nnu] = x + t;
                wi[nnu] = 0.0;
                nn -= 1;
            } else {
                y = a[nnu - 1][nnu - 1];
                w = a[nnu][nnu - 1] * a[nnu - 1][nnu];
                if l == nnu - 1 {
                    p = 0.5 * (y - x);
                    q = p * p + w;
                    z = q.abs().sqrt();
                    x += t;
                    if q >= 0.0 {
                        z = p + sign(z, p);
                        wr[nnu - 1] = x + z;
                        wr[nnu] = x + z;
                        if z != 0.0 {
                            wr[nnu] = x - w / z;
                        }
                        wi[nnu - 1] = 0.0;
                        wi[nnu] = 0.0;
                    } else {
                        wr[nnu - 1] = x + p;
                        wr[nnu] = x + p;
                        wi[nnu - 1] = -z;
                        wi[nnu] = z;
                    }
                    nn -= 2;
                } else {
                    assert!(its < 60, "hqr: too many iterations");
                    if its == 10 || its == 20 {
                        t += x;
                        for i in 1..=nnu {
                            a[i][i] -= x;
                        }
                        s = a[nnu][nnu - 1].abs() + a[nnu - 1][nnu - 2].abs();
                        x = 0.75 * s;
                        y = x;
                        w = -0.4375 * s * s;
                    }
                    its += 1;
                    let mut m = nnu - 2;
                    loop {
                        z = a[m][m];
                        r = x - z;
                        s = y - z;
                        p = (r * s - w) / a[m + 1][m] + a[m][m + 1];
                        q = a[m + 1][m + 1] - z - r - s;
                        r = a[m + 2][m + 1];
                        s = p.abs() + q.abs() + r.abs();
                        p /= s;
                        q /= s;
                        r /= s;
                        if m == l {
                            break;
                        }
                        let u = a[m][m - 1].abs() * (q.abs() + r.abs());
                        let v = p.abs() * (a[m - 1][m - 1].abs() + z.abs() + a[m + 1][m + 1].abs());
                        if u + v == v {
                            break;
                        }
                        m -= 1;
                    }
                    for i in (m + 2)..=nnu {
                        a[i][i - 2] = 0.0;
                        if i != m + 2 {
                            a[i][i - 3] = 0.0;
                        }
                    }
                    let mut k = m;
                    while k < nnu {
                        if k != m {
                            p = a[k][k - 1];
                            q = a[k + 1][k - 1];
                            r = 0.0;
                            if k != nnu - 1 {
                                r = a[k + 2][k - 1];
                            }
                            x = p.abs() + q.abs() + r.abs();
                            if x != 0.0 {
                                p /= x;
                                q /= x;
                                r /= x;
                            }
                        }
                        s = sign((p * p + q * q + r * r).sqrt(), p);
                        if s != 0.0 {
                            if k == m {
                                if l != m {
                                    a[k][k - 1] = -a[k][k - 1];
                                }
                            } else {
                                a[k][k - 1] = -s * x;
                            }
                            p += s;
                            x = p / s;
                            y = q / s;
                            z = r / s;
                            q /= p;
                            r /= p;
                            for j in k..=nnu {
                                p = a[k][j] + q * a[k + 1][j];
                                if k != nnu - 1 {
                                    p += r * a[k + 2][j];
                                    a[k + 2][j] -= p * z;
                                }
                                a[k + 1][j] -= p * y;
                                a[k][j] -= p * x;
                            }
                            let mmin = if nnu < k + 3 { nnu } else { k + 3 };
                            for i in l..=mmin {
                                p = x * a[i][k] + y * a[i][k + 1];
                                if k != nnu - 1 {
                                    p += z * a[i][k + 2];
                                    a[i][k + 2] -= p * r;
                                }
                                a[i][k + 1] -= p * q;
                                a[i][k] -= p;
                            }
                        }
                        k += 1;
                    }
                }
            }
            if nn < 1 || l as i64 >= nn - 1 {
                break;
            }
        }
    }
    (1..=n).map(|i| (wr[i], wi[i])).collect()
}

fn eigenvalues(m: &Mat) -> Vec<(f64, f64)> {
    let mut a = m.clone();
    balance(&mut a);
    hessenberg(&mut a);
    hqr(&a)
}

fn radius(ev: &[(f64, f64)]) -> f64 {
    ev.iter().map(|(re, im)| re.hypot(*im)).fold(0.0, f64::max)
}

/// z → s = ln z / T as (re, im).
fn to_s(z: (f64, f64), t: f64) -> (f64, f64) {
    let (re, im) = z;
    ((re.hypot(im)).ln() / t, im.atan2(re) / t)
}

/// Per-mode linear model (p², b) at the zero-power equilibrium of gap `g` (mass m(g),
/// inertia scaled with the mass; PHYSICS §3, §15 and the `derive::ModeModel` convention).
fn mode_at(p: &SystemParams, mode: LevMode, g: f64) -> (f64, f64) {
    let d = &p.design;
    let s = &d.suspension;
    let c = &d.carriage;
    let hp = derive::h_prime(s.h_m_m, s.mu_rec);
    let m = derive::zero_power_mass(g, s.g_star_m, hp, c.m_star_kg);
    let ks = derive::k_s(s, g, 0.0);
    let ki = derive::k_i(s, g, 0.0);
    let n_l = derive::n_lift(s);
    if mode == LevMode::Heave {
        let me = m / n_l;
        return (ks / me, ki / me);
    }
    let n = usize::from(s.n_magnets);
    let lift = s
        .stations
        .iter()
        .take(n)
        .filter(|st| st.normal_sign > 0.0 && st.has_pm);
    let (sum_r2, z_sum, cnt) = lift.fold((0.0, 0.0, 0.0), |(a, zs, k), st| {
        let r = if mode == LevMode::Pitch {
            st.x_m - c.cg_m[0]
        } else {
            st.y_m - c.cg_m[1]
        };
        (a + r * r, zs + st.z_m, k + 1.0)
    });
    let h_cg = c.cg_m[2] - z_sum / cnt;
    let j = if mode == LevMode::Pitch {
        c.j_kgm2[1]
    } else {
        c.j_kgm2[0]
    } * m
        / c.m_star_kg;
    let me = j / sum_r2;
    let ke = (ks * sum_r2 + m * G0_MPS2 * h_cg) / sum_r2;
    (ke / me, ki / me)
}

// ---- Coupled roll–sway loop (HEAVY roll; Wave 2 decision D1) ------------------------

/// Coupled controller state order: [pred u, pred u̇, pred Y, pred Ẏ, pred i_e, u_applied, q].
const NCS: usize = 7;

fn get_state_rs(l: &ModeLoop) -> [f64; NCS] {
    let o = &l.sway.obs;
    [
        f64::from(o.pred[0]),
        f64::from(o.pred[1]),
        f64::from(o.pred[2]),
        f64::from(o.pred[3]),
        f64::from(o.pred[4]),
        f64::from(o.u_applied),
        f64::from(l.zp.q_a_s),
    ]
}

fn set_state_rs(l: &mut ModeLoop, x: &[f64; NCS]) {
    let o = &mut l.sway.obs;
    o.pred = [
        x[0] as f32,
        x[1] as f32,
        x[2] as f32,
        x[3] as f32,
        x[4] as f32,
    ];
    o.u_applied = x[5] as f32;
    o.primed = true;
    l.zp.q_a_s = x[6] as f32;
    l.zp.x_outer_a = 0.0;
    l.zp.i_bias_a = 0.0;
}

/// Probes the coupled roll–sway loop into (A_c, B_c, C_c, D_c); inputs [u, i].
fn probe_rs(template: &ModeLoop) -> (Mat, Mat, Vec<f64>, [f64; 2]) {
    let scale = [1e-5, 1e-3, 1e-5, 1e-3, 1e-2, 1e-2, 1e-4];
    let mut a = vec![vec![0.0; NCS]; NCS];
    let mut b = vec![vec![0.0; 2]; NCS];
    let mut c = vec![0.0; NCS];
    let mut d = [0.0; 2];
    for j in 0..NCS {
        let mut l = *template;
        let mut x = [0.0; NCS];
        x[j] = scale[j];
        set_state_rs(&mut l, &x);
        let u = f64::from(l.step_zp(0.0, 0.0));
        let nx = get_state_rs(&l);
        for i in 0..NCS {
            a[i][j] = nx[i] / scale[j];
        }
        c[j] = u / scale[j];
    }
    for (k, sc) in [1e-5f64, 1e-2].iter().enumerate() {
        let mut l = *template;
        set_state_rs(&mut l, &[0.0; NCS]);
        let (y, i) = if k == 0 { (*sc, 0.0) } else { (0.0, *sc) };
        let u = f64::from(l.step_zp(y as f32, i as f32));
        let nx = get_state_rs(&l);
        for r in 0..NCS {
            b[r][k] = nx[r] / sc;
        }
        d[k] = u / sc;
    }
    (a, b, c, d)
}

/// The roll–sway model at the zero-power equilibrium of gap `g` (mass m(g), inertia
/// scaled with the mass, k_s/k_i at g, lateral stiffness n·F(g)/w with F = m·g₀/n).
fn rs_model_at(p: &SystemParams, g: f64) -> derive::RollSwayModel {
    let s = &p.design.suspension;
    let c = &p.design.carriage;
    let base = derive::roll_sway_model(p).expect("roll–sway model");
    let hp = derive::h_prime(s.h_m_m, s.mu_rec);
    let m = derive::zero_power_mass(g, s.g_star_m, hp, c.m_star_kg);
    let n_l = derive::n_lift(s);
    let mut out = base;
    out.m_kg = m;
    out.j_kgm2 = base.j_kgm2 * m / c.m_star_kg;
    out.k_y_n_per_m = n_l * (m * G0_MPS2 / n_l) / s.pole_w_m;
    out.k_neg_nm_per_rad = derive::k_s(s, g, 0.0) * base.sum_r2_m2 + m * G0_MPS2 * base.h_m;
    out.k_i_n_per_a = derive::k_i(s, g, 0.0);
    out
}

/// Closed loop of the probed coupled controller with the roll–sway plant `model` at eddy
/// lag `tau_e`: plant [u, u̇, Y, Ẏ, i, i_e], delayed command u_d, controller (NCS).
fn closed_loop_rs(
    ctrl: &(Mat, Mat, Vec<f64>, [f64; 2]),
    model: &derive::RollSwayModel,
    t: f64,
    tau_cl: f64,
    tau_e: f64,
) -> Mat {
    let (ac, bc, cc, dc) = ctrl;
    let (phi, gam) = model.zoh(t, tau_cl, tau_e);
    let n = 7 + NCS;
    let mut m = vec![vec![0.0; n]; n];
    for r in 0..6 {
        for c in 0..6 {
            m[r][c] = phi[r][c];
        }
        m[r][6] = gam[r];
    }
    for j in 0..NCS {
        m[6][7 + j] = cc[j];
    }
    m[6][0] += dc[0];
    m[6][4] += dc[1];
    for r in 0..NCS {
        for j in 0..NCS {
            m[7 + r][7 + j] = ac[r][j];
        }
        m[7 + r][0] += bc[r][0];
        m[7 + r][4] += bc[r][1];
    }
    m
}

/// Eigenvalues of the eight-state coupled design loop [x (6), q, u_d] (f64 gains, state
/// feedback; `derive::roll_sway_design`).
fn rs_design_poles(p: &SystemParams) -> Vec<(f64, f64)> {
    let d = derive::roll_sway_design(p).expect("coupled design");
    let t = derive::fast_period_s(p);
    let mut m = vec![vec![0.0; 8]; 8];
    for r in 0..6 {
        for c in 0..6 {
            m[r][c] = d.phi[r][c];
        }
        m[r][7] = d.gam[r];
    }
    m[6][4] = t;
    m[6][6] = 1.0;
    m[7][0] = d.k[0];
    m[7][1] = d.k[1];
    m[7][2] = d.k[2];
    m[7][3] = d.k[3];
    m[7][6] = d.k[4];
    eigenvalues(&m)
}

fn loops(p: &SystemParams) -> Levitation {
    Levitation::new(p).expect("preset levitation design")
}

// req: FAB-LEV-007
#[test]
fn discrete_poles_match_design() {
    for (name, p) in classes() {
        let lev = loops(&p);
        let t = derive::fast_period_s(&p);
        let tau_cl = derive::tau_current_loop_s(&p);
        let tau_e = p.control.levitation.tau_e_nominal_s;
        assert_eq!(
            p.control.levitation.omega_ratio, 2.5,
            "ω = 2.5·p (plan §2b.1)"
        );
        for (k, mode) in LevMode::ALL.iter().enumerate() {
            let lp = &lev.loops[k];
            assert_eq!(
                lp.active,
                derive::mode_active(&p, *mode),
                "{name} {mode:?} active set"
            );
            if !lp.active {
                continue;
            }
            // D1: the coupled roll–sway loop is closed with the roll–sway plant and
            // compared with the eight-state coupled design (its five placed targets
            // included: roll pair at ω = 2.5·p, α_rs·ω, the sway pair).
            let (ev, design): (Vec<Zc>, Vec<Zc>) = if lp.sway.active {
                let ctrl = probe_rs(lp);
                let model = derive::roll_sway_model(&p).expect("model");
                let ev = eigenvalues(&closed_loop_rs(&ctrl, &model, t, tau_cl, tau_e));
                let mut design = rs_design_poles(&p);
                for (re, im) in derive::roll_sway_target_poles_s(&p) {
                    let r = (re * t).exp();
                    design.push((r * (im * t).cos(), r * (im * t).sin()));
                }
                (ev, design)
            } else {
                let ctrl = probe(lp);
                let (p2, bb) = mode_at(&p, *mode, p.design.suspension.g_star_m);
                let ev = eigenvalues(&closed_loop(&ctrl, p2, bb, t, tau_cl, tau_e));
                let kd = derive::zp_gains_discrete(&p, *mode);
                let mut design: Vec<(f64, f64)> =
                    derive::zp_closed_loop_poles(&p, *mode, &kd, tau_e).to_vec();
                design.extend(derive::zp_target_poles_z(&p, *mode));
                (ev, design)
            };
            // Every discrete design pole (f64 gains, state feedback) is present.
            for zd in design.iter() {
                let sd = to_s(*zd, t);
                let best = ev
                    .iter()
                    .map(|z| {
                        let s = to_s(*z, t);
                        (s.0 - sd.0).hypot(s.1 - sd.1) / sd.0.hypot(sd.1)
                    })
                    .fold(f64::MAX, f64::min);
                assert!(
                    best <= 0.02,
                    "{name} {mode:?}: design pole z = {zd:?} matched within {:.3} % (tol 2 %; LEV-007)",
                    best * 100.0
                );
            }
            // ζ ≥ 0.5 for every complex closed-loop pole (design pair, observer pair).
            for z in &ev {
                if z.1.abs() > 1e-9 {
                    let s = to_s(*z, t);
                    let zeta = -s.0 / s.0.hypot(s.1);
                    assert!(
                        zeta >= 0.5,
                        "{name} {mode:?}: closed-loop pole {z:?} has ζ = {zeta:.3} (≥ 0.5; LEV-007)"
                    );
                }
            }
            assert!(
                radius(&ev) < 1.0,
                "{name} {mode:?}: stable at the design point"
            );
        }
    }
}

// req: FAB-LEV-020
#[test]
fn stable_for_eddy_lag_to_2ms() {
    for (name, p) in classes() {
        let lev = loops(&p);
        let t = derive::fast_period_s(&p);
        let tau_cl = derive::tau_current_loop_s(&p);
        for (k, mode) in LevMode::ALL.iter().enumerate() {
            let lp = &lev.loops[k];
            if !lp.active {
                continue;
            }
            // D1: the coupled roll–sway loop against the coupled plant.
            let rs = lp.sway.active;
            let ctrl = if rs { probe_rs(lp) } else { probe(lp) };
            let (p2, bb) = mode_at(&p, *mode, p.design.suspension.g_star_m);
            let model = derive::roll_sway_model(&p).unwrap_or_default();
            let cl = |tau_e: f64| {
                if rs {
                    closed_loop_rs(&ctrl, &model, t, tau_cl, tau_e)
                } else {
                    closed_loop(&ctrl, p2, bb, t, tau_cl, tau_e)
                }
            };
            let mut worst = 0.0f64;
            for step in 0..=40 {
                let tau_e = 2.0e-3 * f64::from(step) / 40.0;
                let rho = radius(&eigenvalues(&cl(tau_e)));
                worst = worst.max(rho);
                assert!(
                    rho < 1.0,
                    "{name} {mode:?}: unstable at τ_e = {:.2} ms (ρ = {rho:.6}); LEV-020 needs τ_e ≤ 2 ms",
                    tau_e * 1e3
                );
            }
            assert!(
                worst < 1.0,
                "{name} {mode:?}: worst ρ over 0–2 ms = {worst}"
            );
            // Critical τ_e of the implemented loop (bisection to 1 µs; reported by the
            // gain_margin study through derive::zp_critical_tau_e for state feedback).
            let (mut lo, mut hi) = (0.0f64, 0.02f64);
            while hi - lo > 1e-6 {
                let mid = 0.5 * (lo + hi);
                let rho = radius(&eigenvalues(&cl(mid)));
                if rho >= 1.0 {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            eprintln!(
                "{name} {mode:?}: implemented critical τ_e = {:.3} ms",
                lo * 1e3
            );
            assert!(
                lo > 2.0e-3,
                "{name} {mode:?}: critical τ_e {lo} s > 2 ms (LEV-020)"
            );
        }
    }
}

#[test]
fn pole_check_detects_detuning() {
    // Control case for the LEV-007 check: scaling the implemented command by 1.1 moves
    // the closed-loop poles far outside the 2 % window, so the test has power.
    let p = presets::drawer();
    let lev = loops(&p);
    let t = derive::fast_period_s(&p);
    let tau_cl = derive::tau_current_loop_s(&p);
    let tau_e = p.control.levitation.tau_e_nominal_s;
    let mut ctrl = probe(&lev.loops[0]);
    for c in ctrl.2.iter_mut() {
        *c *= 1.1;
    }
    ctrl.3[0] *= 1.1;
    ctrl.3[1] *= 1.1;
    let (p2, bb) = mode_at(&p, LevMode::Heave, p.design.suspension.g_star_m);
    let ev = eigenvalues(&closed_loop(&ctrl, p2, bb, t, tau_cl, tau_e));
    let kd = derive::zp_gains_discrete(&p, LevMode::Heave);
    let worst = derive::zp_closed_loop_poles(&p, LevMode::Heave, &kd, tau_e)
        .iter()
        .map(|zd| {
            let sd = to_s(*zd, t);
            ev.iter()
                .map(|z| {
                    let s = to_s(*z, t);
                    (s.0 - sd.0).hypot(s.1 - sd.1) / sd.0.hypot(sd.1)
                })
                .fold(f64::MAX, f64::min)
        })
        .fold(0.0, f64::max);
    assert!(worst > 0.05, "a 10 % gain error must be visible: {worst}");
}

// req: FAB-LEV-007
#[test]
fn stable_across_float_band() {
    for (name, p) in classes() {
        let lev = loops(&p);
        let t = derive::fast_period_s(&p);
        let tau_cl = derive::tau_current_loop_s(&p);
        let tau_e = p.control.levitation.tau_e_nominal_s;
        let [g_lo, g_hi] = p.design.suspension.zp_band_m;
        for (k, mode) in LevMode::ALL.iter().enumerate() {
            let lp = &lev.loops[k];
            if !lp.active {
                continue;
            }
            let rs = lp.sway.active;
            let ctrl = if rs { probe_rs(lp) } else { probe(lp) };
            for step in 0..=20 {
                let g = g_lo + (g_hi - g_lo) * f64::from(step) / 20.0;
                let ev = if rs {
                    // D1: coupled roll–sway plant at the zero-power equilibrium of g.
                    eigenvalues(&closed_loop_rs(
                        &ctrl,
                        &rs_model_at(&p, g),
                        t,
                        tau_cl,
                        tau_e,
                    ))
                } else {
                    let (p2, bb) = mode_at(&p, *mode, g);
                    eigenvalues(&closed_loop(&ctrl, p2, bb, t, tau_cl, tau_e))
                };
                let rho = radius(&ev);
                assert!(
                    rho < 1.0,
                    "{name} {mode:?}: unstable at the zero-power gap {:.3} mm (ρ = {rho:.6}; LEV-007)",
                    g * 1e3
                );
            }
        }
    }
}

#[test]
fn eigen_solver_self_check() {
    // Companion matrix of (z − 0.5)(z − 0.9)(z² − 1.6z + 0.73) (pair 0.8 ± 0.3j).
    let c = [
        -(0.5 * 0.9 * 0.73),
        0.5 * 0.9 * 1.6 + 0.73 * 1.4,
        -(0.45 + 1.4 * 1.6 + 0.73),
        1.6 + 1.4,
    ];
    let m: Mat = vec![
        vec![0.0, 1.0, 0.0, 0.0],
        vec![0.0, 0.0, 1.0, 0.0],
        vec![0.0, 0.0, 0.0, 1.0],
        vec![c[0], c[1], c[2], c[3]],
    ];
    let mut ev = eigenvalues(&m);
    ev.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap()
            .then(a.1.partial_cmp(&b.1).unwrap())
    });
    let want = [(0.5, 0.0), (0.8, -0.3), (0.8, 0.3), (0.9, 0.0)];
    for (g, w) in ev.iter().zip(want.iter()) {
        assert!(
            (g.0 - w.0).abs() < 1e-9 && (g.1 - w.1).abs() < 1e-9,
            "{g:?} vs {w:?}"
        );
    }
}
