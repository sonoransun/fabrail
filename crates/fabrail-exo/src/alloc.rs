//! Static pod force allocation (module name alloc = allocation; no heap): weighted least squares with active-set clamping and reallocation on pod loss. Owner: X2 (API frozen by W0a).
//!
//! Exotic design §3.3 (ARENA, ROTATOR). A rigid-body heave/roll/pitch wrench is
//! allocated to N pods by weighted least squares with active-set clamping; a failed pod
//! triggers reallocation (call [`allocate`] again with it marked unhealthy); LAND is
//! declared when residual capacity in any quadrant falls below 1.3× its weight share.
//! Static only: the dynamic ARENA simulation is roadmap. Fixed-size, const-generic
//! arrays (no heap; the module name is "allocation", not the `alloc` crate).
//!
//! Problem (z up, moments about the origin of the pod coordinates):
//!
//! minimise Σ w_i·(f_i − f̄_i)²  subject to  Σ f_i = F_z, Σ f_i·y_i = M_x,
//! −Σ f_i·x_i = M_y, 0 ≤ f_i ≤ f_max,i.
//!
//! * Gap equalisation: the preferred force f̄_i = F_z·f_max,i/Σf_max is an equal
//!   utilisation; identical zero-power pods carrying equal utilisation settle at equal
//!   gaps, because the zero-power gap is set by the load (PHYSICS §3, §15).
//! * Solution on the free set: f = f̄ + W⁻¹Bᵀλ with (B·W⁻¹·Bᵀ)·λ = τ − B·f̄ (3×3, lever
//!   arms scaled by the layout length for conditioning; completely pivoted, so a
//!   direction the free pods cannot produce, e.g. roll with every pod on one line, is
//!   left in the residual instead of blowing up).
//! * Active set: pods that leave [0, f_max] are clamped to the bound and the rest is
//!   re-solved (redistributed pseudo-inverse) until no bound is violated.
//! * LAND rule: the weight share of a quadrant (by the signs of x and y) is what it
//!   carries in the all-pods-healthy allocation of the same wrench; LAND when the
//!   healthy capacity Σf_max of any quadrant is below 1.3× that share, when the wrench
//!   is not met, or when no pod is healthy.

use fabrail_types::fmath;

/// LAND when residual quadrant capacity falls below this multiple of its weight share.
pub const QUADRANT_CAPACITY_MIN: f32 = 1.3;

/// Relative tolerance on the achieved wrench (f32 active-set solution).
pub const WRENCH_REL_TOL: f32 = 1.0e-4;

/// Active-set iteration cap (each pass clamps at least one pod).
const MAX_PASSES: usize = 64;

/// One pod.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pod {
    /// Position along the structure, m (pitch lever arm).
    pub x_m: f32,
    /// Position across the structure, m (roll lever arm).
    pub y_m: f32,
    /// Maximum lift force, N.
    pub f_max_n: f32,
    /// WLS weight (larger = more expensive).
    pub weight: f32,
    /// The pod is healthy.
    pub healthy: bool,
}

impl Pod {
    /// The pod is structurally usable: finite position, positive finite capacity.
    fn valid(&self) -> bool {
        self.x_m.is_finite()
            && self.y_m.is_finite()
            && self.f_max_n.is_finite()
            && self.f_max_n > 0.0
    }

    /// The WLS weight, 1 when unset or invalid.
    fn w(&self) -> f32 {
        if self.weight.is_finite() && self.weight > 0.0 {
            self.weight
        } else {
            1.0
        }
    }

    /// Quadrant index by the signs of x and y (0 … 3).
    fn quadrant(&self) -> usize {
        usize::from(self.x_m >= 0.0) * 2 + usize::from(self.y_m >= 0.0)
    }
}

/// A rigid-body wrench to allocate (z up).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Wrench {
    /// Vertical force, N.
    pub fz_n: f32,
    /// Moment about x (roll), N·m.
    pub mx_nm: f32,
    /// Moment about y (pitch), N·m.
    pub my_nm: f32,
}

/// Allocation outcome.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AllocStatus {
    /// The wrench was met within tolerance.
    pub ok: bool,
    /// LAND demanded (capacity rule).
    pub land: bool,
    /// Healthy pods.
    pub n_healthy: u32,
    /// Unallocated wrench (demand minus achieved).
    pub residual: Wrench,
}

/// Solves the symmetric positive semi-definite 3×3 system `m·x = r` by Gaussian
/// elimination with complete pivoting; directions whose pivot falls below 1e-6 of the
/// largest entry are left at 0 (not controllable by the free pods).
fn solve3(m: [[f32; 3]; 3], r: [f32; 3]) -> [f32; 3] {
    let mut a = m;
    let mut b = r;
    let mut perm = [0usize, 1, 2];
    let scale = a
        .iter()
        .flat_map(|row| row.iter())
        .fold(0.0f32, |s, v| fmath::fmaxf(s, fmath::absf(*v)));
    if !(scale.is_finite() && scale > 0.0) {
        return [0.0; 3];
    }
    let tol = 1.0e-6 * scale;
    let mut rank = 0usize;
    for k in 0..3 {
        let (mut pi, mut pj, mut best) = (k, k, 0.0f32);
        for (i, row) in a.iter().enumerate().skip(k) {
            for (j, v) in row.iter().enumerate().skip(k) {
                if fmath::absf(*v) > best {
                    (pi, pj, best) = (i, j, fmath::absf(*v));
                }
            }
        }
        if best < tol {
            break;
        }
        a.swap(k, pi);
        b.swap(k, pi);
        for row in a.iter_mut() {
            row.swap(k, pj);
        }
        perm.swap(k, pj);
        let pivot_row = a[k];
        let pivot_b = b[k];
        for (row, bi) in a.iter_mut().zip(b.iter_mut()).skip(k + 1) {
            let f = row[k] / pivot_row[k];
            for (c, p) in row.iter_mut().zip(pivot_row.iter()).skip(k) {
                *c -= f * p;
            }
            *bi -= f * pivot_b;
        }
        rank += 1;
    }
    let mut y = [0.0f32; 3];
    for k in (0..rank).rev() {
        let mut s = b[k];
        for (akc, yc) in a[k].iter().zip(y.iter()).take(rank).skip(k + 1) {
            s -= akc * yc;
        }
        y[k] = s / a[k][k];
    }
    let mut x = [0.0f32; 3];
    for (k, p) in perm.iter().enumerate() {
        x[*p] = y[k];
    }
    x
}

/// WLS with active-set clamping over the pods selected by `use_pod`; writes the forces
/// (0 for unselected pods) and returns the achieved wrench.
fn solve_wls<const N: usize>(
    w: &Wrench,
    pods: &[Pod; N],
    use_pod: &[bool; N],
    len_m: f32,
    out: &mut [f32; N],
) -> Wrench {
    let row = |p: &Pod| [1.0f32, p.y_m / len_m, -p.x_m / len_m];
    let tau = [w.fz_n, w.mx_nm / len_m, w.my_nm / len_m];
    let cap: f32 = pods
        .iter()
        .zip(use_pod.iter())
        .filter(|(_, u)| **u)
        .map(|(p, _)| p.f_max_n)
        .sum();
    let pref = |p: &Pod| {
        if cap > 0.0 {
            w.fz_n * p.f_max_n / cap
        } else {
            0.0
        }
    };
    let mut free = *use_pod;
    for (f, u) in out.iter_mut().zip(use_pod.iter()) {
        if !*u {
            *f = 0.0;
        }
    }
    for _ in 0..MAX_PASSES {
        // r = τ − B_fixed·f_fixed − B_free·f̄_free; M = Σ_free B_iB_iᵀ/w_i.
        let mut r = tau;
        let mut m = [[0.0f32; 3]; 3];
        for ((p, u), (fr, f)) in pods
            .iter()
            .zip(use_pod.iter())
            .zip(free.iter().zip(out.iter()))
        {
            if !*u {
                continue;
            }
            let bi = row(p);
            let fi = if *fr { pref(p) } else { *f };
            for (rk, bk) in r.iter_mut().zip(bi.iter()) {
                *rk -= bk * fi;
            }
            if *fr {
                let inv_w = 1.0 / p.w();
                for (mrow, bk) in m.iter_mut().zip(bi.iter()) {
                    for (mkl, bl) in mrow.iter_mut().zip(bi.iter()) {
                        *mkl += inv_w * bk * bl;
                    }
                }
            }
        }
        let lambda = solve3(m, r);
        let mut clamped = false;
        for ((p, fr), f) in pods.iter().zip(free.iter_mut()).zip(out.iter_mut()) {
            if !*fr {
                continue;
            }
            let bi = row(p);
            let v = pref(p) + (bi[0] * lambda[0] + bi[1] * lambda[1] + bi[2] * lambda[2]) / p.w();
            if !v.is_finite() || v < 0.0 {
                *f = 0.0;
                *fr = false;
                clamped = true;
            } else if v > p.f_max_n {
                *f = p.f_max_n;
                *fr = false;
                clamped = true;
            } else {
                *f = v;
            }
        }
        if !clamped {
            break;
        }
    }
    let mut achieved = Wrench::default();
    for ((p, f), u) in pods.iter().zip(out.iter()).zip(use_pod.iter()) {
        if !*u {
            continue;
        }
        achieved.fz_n += *f;
        achieved.mx_nm += *f * p.y_m;
        achieved.my_nm -= *f * p.x_m;
    }
    achieved
}

/// Allocates `w` over `pods`, writing each pod's force into `out` (0 for failed pods).
pub fn allocate<const N: usize>(w: &Wrench, pods: &[Pod; N], out: &mut [f32; N]) -> AllocStatus {
    let n_healthy = pods.iter().filter(|p| p.healthy).count() as u32;
    let valid: [bool; N] = core::array::from_fn(|i| pods[i].valid());
    let usable: [bool; N] = core::array::from_fn(|i| valid[i] && pods[i].healthy);
    let wrench_ok = w.fz_n.is_finite() && w.mx_nm.is_finite() && w.my_nm.is_finite();
    // Layout length scale for the moment rows (and the moment tolerance).
    let len_m = pods.iter().filter(|p| p.valid()).fold(1.0e-3f32, |l, p| {
        fmath::fmaxf(l, fmath::fmaxf(fmath::absf(p.x_m), fmath::absf(p.y_m)))
    });
    if !wrench_ok || !usable.iter().any(|u| *u) {
        out.iter_mut().for_each(|f| *f = 0.0);
        return AllocStatus {
            ok: false,
            land: true,
            n_healthy,
            residual: if wrench_ok { *w } else { Wrench::default() },
        };
    }
    let achieved = solve_wls(w, pods, &usable, len_m, out);
    let residual = Wrench {
        fz_n: w.fz_n - achieved.fz_n,
        mx_nm: w.mx_nm - achieved.mx_nm,
        my_nm: w.my_nm - achieved.my_nm,
    };
    let scale_n = fmath::fmaxf(
        fmath::absf(w.fz_n) + (fmath::absf(w.mx_nm) + fmath::absf(w.my_nm)) / len_m,
        1.0,
    );
    let tol = WRENCH_REL_TOL * scale_n;
    let ok = fmath::absf(residual.fz_n) <= tol
        && fmath::absf(residual.mx_nm) <= tol * len_m
        && fmath::absf(residual.my_nm) <= tol * len_m;

    // Quadrant capacity rule against the all-pods-healthy weight shares.
    let mut design = [0.0f32; N];
    solve_wls(w, pods, &valid, len_m, &mut design);
    let mut share = [0.0f32; 4];
    let mut capacity = [0.0f32; 4];
    for ((p, d), u) in pods.iter().zip(design.iter()).zip(usable.iter()) {
        if !p.valid() {
            continue;
        }
        let q = p.quadrant();
        share[q] += *d;
        if *u {
            capacity[q] += p.f_max_n;
        }
    }
    let short = share
        .iter()
        .zip(capacity.iter())
        .any(|(s, c)| *s > 0.0 && *c < QUADRANT_CAPACITY_MIN * *s);
    AllocStatus {
        ok,
        land: !ok || short,
        n_healthy,
        residual,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ARENA-like tray (EXOTIC §1: 8,600 t on pods): a 4 × 4 grid at 20 m pitch,
    /// each pod rated at twice its equal share (50 % utilisation).
    const M_T: f32 = 8_600.0e3 * 9.806_65;

    fn grid() -> [Pod; 16] {
        core::array::from_fn(|k| Pod {
            x_m: -30.0 + 20.0 * (k % 4) as f32,
            y_m: -30.0 + 20.0 * (k / 4) as f32,
            f_max_n: 2.0 * M_T / 16.0,
            weight: 1.0,
            healthy: true,
        })
    }

    fn achieved(pods: &[Pod], f: &[f32]) -> Wrench {
        let mut a = Wrench::default();
        for (p, fi) in pods.iter().zip(f.iter()) {
            a.fz_n += fi;
            a.mx_nm += fi * p.y_m;
            a.my_nm -= fi * p.x_m;
        }
        a
    }

    fn close(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn wls_meets_wrench_with_equal_utilisation() {
        let pods = grid();
        let mut f = [0.0; 16];
        let w = Wrench {
            fz_n: M_T,
            ..Wrench::default()
        };
        let s = allocate(&w, &pods, &mut f);
        assert!(s.ok && !s.land && s.n_healthy == 16, "{s:?}");
        for fi in f {
            assert!(close(fi, M_T / 16.0, 1e-4 * M_T), "equal shares: {fi}");
        }
        // Roll and pitch moments (wind, 1 % offset of the centre of mass): the WLS
        // solution meets the wrench and equals the closed-form minimum-norm deviation
        // Δf_i = λ·B_i (symmetric grid: Σy² = Σx² = 16·500 m²).
        let w = Wrench {
            fz_n: M_T,
            mx_nm: 0.01 * M_T * 30.0,
            my_nm: -0.01 * M_T * 20.0,
        };
        let s = allocate(&w, &pods, &mut f);
        assert!(s.ok && !s.land, "{s:?}");
        let a = achieved(&pods, &f);
        assert!(close(a.fz_n, w.fz_n, 1e-4 * M_T));
        assert!(close(a.mx_nm, w.mx_nm, 1e-4 * M_T * 30.0));
        assert!(close(a.my_nm, w.my_nm, 1e-4 * M_T * 30.0));
        for (p, fi) in pods.iter().zip(f.iter()) {
            let want = M_T / 16.0 + w.mx_nm * p.y_m / 8000.0 - w.my_nm * p.x_m / 8000.0;
            assert!(close(*fi, want, 1e-4 * M_T), "{fi} vs {want}");
        }
        // Weights: a costlier pod deviates less from its preferred force.
        let mut heavy = pods;
        heavy[15].weight = 10.0;
        let mut g = [0.0; 16];
        allocate(&w, &heavy, &mut g);
        assert!((g[15] - M_T / 16.0).abs() < (f[15] - M_T / 16.0).abs());
    }

    #[test]
    fn reallocation_on_pod_loss_and_quadrant_land() {
        let w = Wrench {
            fz_n: M_T,
            ..Wrench::default()
        };
        let mut pods = grid();
        let mut f = [0.0; 16];
        // One interior pod lost: reallocated, wrench met, neighbours pick up, no LAND
        // (quadrant capacity 3·f_max ≥ 1.3 × its share 2·f_max).
        pods[5].healthy = false;
        let s = allocate(&w, &pods, &mut f);
        assert!(s.ok && !s.land && s.n_healthy == 15, "{s:?}");
        assert_eq!(f[5], 0.0);
        let a = achieved(&pods, &f);
        assert!(close(a.fz_n, M_T, 1e-4 * M_T) && close(a.mx_nm, 0.0, 1e-4 * M_T * 30.0));
        assert!(f[4] > M_T / 16.0 && f[6] > M_T / 16.0 && f[1] > M_T / 16.0);
        assert!(
            f.iter()
                .all(|x| *x >= 0.0 && *x <= 2.0 * M_T / 16.0 * 1.0001)
        );
        // A second pod in the same quadrant: capacity 2·f_max < 1.3 × 2·f_max → LAND,
        // even though the wrench can still be met.
        pods[0].healthy = false;
        let s = allocate(&w, &pods, &mut f);
        assert!(s.ok, "still allocatable: {s:?}");
        assert!(s.land, "quadrant capacity below 1.3× its share → LAND");
        // The same two losses in different quadrants keep flying.
        let mut spread = grid();
        spread[5].healthy = false;
        spread[10].healthy = false;
        let s = allocate(&w, &spread, &mut f);
        assert!(s.ok && !s.land, "{s:?}");
    }

    #[test]
    fn active_set_clamping_and_infeasible_residual() {
        let pods = grid();
        let mut f = [0.0; 16];
        // A pitch moment the unconstrained WLS would meet only with negative forces on
        // the light side and > f_max on the heavy side (|M_y| > 16.7·M_T·1 m): clamped,
        // still met (the layout's limit is 20·M_T·1 m).
        let w = Wrench {
            fz_n: M_T,
            mx_nm: 0.0,
            my_nm: -18.0 * M_T,
        };
        let s = allocate(&w, &pods, &mut f);
        let f_max = 2.0 * M_T / 16.0;
        assert!(f.iter().all(|x| *x >= 0.0 && *x <= f_max * 1.0001), "{f:?}");
        assert!(
            f.iter().any(|x| close(*x, f_max, 1e-3 * f_max)),
            "a pod at f_max"
        );
        assert!(f.contains(&0.0), "a pod clamped at 0");
        assert!(s.ok, "{s:?}");
        let a = achieved(&pods, &f);
        assert!(close(a.my_nm, w.my_nm, 1e-4 * M_T * 30.0));
        // Beyond total capacity: every pod at f_max, residual reported, LAND.
        let w = Wrench {
            fz_n: 2.5 * M_T,
            ..Wrench::default()
        };
        let s = allocate(&w, &pods, &mut f);
        assert!(!s.ok && s.land);
        assert!(close(s.residual.fz_n, 0.5 * M_T, 1e-4 * M_T), "{s:?}");
        assert!(f.iter().all(|x| close(*x, f_max, 1e-4 * f_max)));
    }

    #[test]
    fn degenerate_layouts_never_nan() {
        // Pods on one line (y = 0): a roll moment cannot be produced and is reported.
        let line: [Pod; 4] = core::array::from_fn(|k| Pod {
            x_m: -15.0 + 10.0 * k as f32,
            y_m: 0.0,
            f_max_n: 1.0e6,
            weight: 1.0,
            healthy: true,
        });
        let mut f = [0.0; 4];
        let w = Wrench {
            fz_n: 1.0e6,
            mx_nm: 5.0e5,
            my_nm: 0.0,
        };
        let s = allocate(&w, &line, &mut f);
        assert!(f.iter().all(|x| x.is_finite()));
        assert!(!s.ok && s.land);
        assert!(close(s.residual.mx_nm, 5.0e5, 1.0) && close(s.residual.fz_n, 0.0, 1.0));
        // No healthy pod: LAND, zero forces, the whole wrench as residual.
        let mut dead = line;
        dead.iter_mut().for_each(|p| p.healthy = false);
        let s = allocate(&w, &dead, &mut f);
        assert_eq!((s.ok, s.land, s.n_healthy, s.residual), (false, true, 0, w));
        assert!(f.iter().all(|x| *x == 0.0));
        // Non-finite demand: refused, not propagated.
        let s = allocate(
            &Wrench {
                fz_n: f32::NAN,
                ..w
            },
            &line,
            &mut f,
        );
        assert!(!s.ok && s.land && f.iter().all(|x| *x == 0.0));
        // Invalid pod data (NaN position, zero capacity) is ignored like a failed pod.
        let mut odd = line;
        odd[1].x_m = f32::NAN;
        odd[2].f_max_n = 0.0;
        let w = Wrench {
            fz_n: 1.0e6,
            ..Wrench::default()
        };
        let s = allocate(&w, &odd, &mut f);
        assert!(f.iter().all(|x| x.is_finite()) && f[1] == 0.0 && f[2] == 0.0);
        assert!(s.ok, "{s:?}");
    }
}
