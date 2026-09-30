//! Small fixed-size f32 linear algebra: Vec3/Mat3, 3x3 inverse, allocation pseudo-inverse. Owner: A1.
//!
//! Frozen API (W0). Everything is fixed-size and allocation-free; no external linear-algebra crate
//! in the controller crates (feature-unification trap, core design §A.1). The allocation map is
//! the weighted right pseudo-inverse A = W⁻¹Tᵀ(TW⁻¹Tᵀ)⁻¹ of the modal map T (modes ×
//! magnets), used by `levitation::modal` (PHYSICS §15).
//!
//! Inverses return `None` when the matrix is (numerically) singular; callers treat that as
//! "design failed" (a `ParamError` at `new()`), never as a silent zero gain. All loops
//! use iterators or constant indices (the crate denies `clippy::indexing_slicing`).

use fabrail_types::{MAX_MAGNETS, fmath};

/// 3-vector.
pub type Vec3 = [f32; 3];

/// 3×3 matrix, row-major.
pub type Mat3 = [[f32; 3]; 3];

/// Modal map T: row k maps magnet gaps/forces to mode k (heave, pitch, roll).
pub type ModalMap = [[f32; MAX_MAGNETS]; 3];

/// Allocation map A: row i maps the 3 modal demands to magnet i.
pub type AllocMap = [[f32; 3]; MAX_MAGNETS];

/// The 3×3 identity.
pub const MAT3_IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Relative singularity threshold for [`mat3_inv`]: |det| ≤ `SING_REL`·‖M‖∞³ is treated
/// as singular. Source: f32 unit roundoff 2⁻²⁴ ≈ 6e-8 times a 100× safety factor for the
/// cofactor cancellation (Golub & Van Loan, "Matrix Computations", 4th ed., §2.7).
const SING_REL: f32 = 1.0e-6;

/// Dot product.
#[inline]
pub fn dot3(a: &Vec3, b: &Vec3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// m·v.
pub fn mat3_mul_vec(m: &Mat3, v: &Vec3) -> Vec3 {
    [dot3(&m[0], v), dot3(&m[1], v), dot3(&m[2], v)]
}

/// Transpose.
pub fn mat3_transpose(m: &Mat3) -> Mat3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

/// a·b.
pub fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let bt = mat3_transpose(b);
    [
        [
            dot3(&a[0], &bt[0]),
            dot3(&a[0], &bt[1]),
            dot3(&a[0], &bt[2]),
        ],
        [
            dot3(&a[1], &bt[0]),
            dot3(&a[1], &bt[1]),
            dot3(&a[1], &bt[2]),
        ],
        [
            dot3(&a[2], &bt[0]),
            dot3(&a[2], &bt[1]),
            dot3(&a[2], &bt[2]),
        ],
    ]
}

/// Determinant (cofactor expansion along the first row).
pub fn mat3_det(m: &Mat3) -> f32 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

/// Infinity norm (maximum absolute row sum).
fn mat3_norm_inf(m: &Mat3) -> f32 {
    m.iter()
        .map(|r| fmath::absf(r[0]) + fmath::absf(r[1]) + fmath::absf(r[2]))
        .fold(0.0, fmath::fmaxf)
}

/// Inverse by the adjugate; `None` when |det| ≤ 1e-6·‖M‖∞³ (numerically singular) or
/// any entry is non-finite.
pub fn mat3_inv(m: &Mat3) -> Option<Mat3> {
    let det = mat3_det(m);
    let n = mat3_norm_inf(m);
    if !det.is_finite() || !n.is_finite() || n <= 0.0 || fmath::absf(det) <= SING_REL * n * n * n {
        return None;
    }
    let k = 1.0 / det;
    let inv = [
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * k,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * k,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * k,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * k,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * k,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * k,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * k,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * k,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * k,
        ],
    ];
    if inv.iter().flatten().all(|v| v.is_finite()) {
        Some(inv)
    } else {
        None
    }
}

/// Weighted right pseudo-inverse A = W⁻¹Tᵀ(TW⁻¹Tᵀ)⁻¹ over the first `n_magnets`
/// columns and `n_modes` (1–3) rows of `t`, with per-magnet weights `w` (> 0). Rows of
/// A beyond `n_magnets` and columns beyond `n_modes` are zero. `None` when TW⁻¹Tᵀ is
/// singular or an argument is out of range (PHYSICS §15; LEV-009 allocation identity).
///
/// The Gram matrix G = TW⁻¹Tᵀ is formed on the active block and padded with the identity
/// on unused modes, so one 3×3 inverse serves 1–3 modes. A weight of `f32::INFINITY`
/// removes a magnet from the allocation (its row of A is zero).
pub fn weighted_pinv(
    t: &ModalMap,
    w: &[f32; MAX_MAGNETS],
    n_magnets: u8,
    n_modes: u8,
) -> Option<AllocMap> {
    let n = usize::from(n_magnets);
    let k = usize::from(n_modes);
    if n == 0 || n > MAX_MAGNETS || k == 0 || k > 3 {
        return None;
    }
    // W⁻¹ on the active magnets (0 on unused ones and for infinite weights).
    let mut w_inv = [0.0f32; MAX_MAGNETS];
    for (wi, wv) in w_inv.iter_mut().zip(w.iter()).take(n) {
        if wv.is_nan() || *wv <= 0.0 {
            return None;
        }
        *wi = if wv.is_finite() { 1.0 / *wv } else { 0.0 };
    }
    // G = T W⁻¹ Tᵀ (k×k active block, identity elsewhere).
    let mut g = MAT3_IDENTITY;
    for (r, (grow, trow)) in g.iter_mut().zip(t.iter()).enumerate() {
        for (c, (gv, tcol)) in grow.iter_mut().zip(t.iter()).enumerate() {
            if r < k && c < k {
                *gv = trow
                    .iter()
                    .zip(tcol.iter())
                    .zip(w_inv.iter())
                    .take(n)
                    .map(|((a, b), wi)| a * b * wi)
                    .sum();
            }
        }
    }
    let g_inv = mat3_inv(&g)?;
    // A = W⁻¹ Tᵀ G⁻¹: row i of A is w_inv[i]·(column i of T)ᵀ·G⁻¹.
    let mut a = [[0.0f32; 3]; MAX_MAGNETS];
    for (i, (arow, wi)) in a.iter_mut().zip(w_inv.iter()).enumerate().take(n) {
        let tcol: Vec3 = [
            if k > 0 { col(t, 0, i) } else { 0.0 },
            if k > 1 { col(t, 1, i) } else { 0.0 },
            if k > 2 { col(t, 2, i) } else { 0.0 },
        ];
        let gt = mat3_transpose(&g_inv);
        for (av, gcol) in arow.iter_mut().zip(gt.iter()).take(k) {
            *av = wi * dot3(&tcol, gcol);
        }
    }
    if a.iter().flatten().all(|v| v.is_finite()) {
        Some(a)
    } else {
        None
    }
}

/// Element `t[row][i]` (0 when out of range).
#[inline]
fn col(t: &ModalMap, row: usize, i: usize) -> f32 {
    t.get(row).and_then(|r| r.get(i)).copied().unwrap_or(0.0)
}

/// T·u for the first `n_magnets` entries (the modal content of a per-magnet vector).
pub fn modal_of(t: &ModalMap, u: &[f32; MAX_MAGNETS], n_magnets: u8) -> Vec3 {
    let n = usize::from(n_magnets);
    let mut out = [0.0f32; 3];
    for (o, trow) in out.iter_mut().zip(t.iter()) {
        *o = trow.iter().zip(u.iter()).take(n).map(|(a, b)| a * b).sum();
    }
    out
}

/// A·v for the first `n_magnets` rows (per-magnet vector from modal demands).
pub fn alloc_of(a: &AllocMap, v: &Vec3, n_magnets: u8) -> [f32; MAX_MAGNETS] {
    let n = usize::from(n_magnets);
    let mut out = [0.0f32; MAX_MAGNETS];
    for (o, arow) in out.iter_mut().zip(a.iter()).take(n) {
        *o = dot3(arow, v);
    }
    out
}

/// Null-space projection of a per-magnet vector: u − A·(T·u) (the warp component that
/// produces no net modal force; PHYSICS §15). Entries beyond `n_magnets` are zero.
pub fn null_space_part(
    t: &ModalMap,
    a: &AllocMap,
    u: &[f32; MAX_MAGNETS],
    n_magnets: u8,
) -> [f32; MAX_MAGNETS] {
    let n = usize::from(n_magnets);
    let m = modal_of(t, u, n_magnets);
    let r = alloc_of(a, &m, n_magnets);
    let mut out = [0.0f32; MAX_MAGNETS];
    for (o, (ui, ri)) in out.iter_mut().zip(u.iter().zip(r.iter())).take(n) {
        *o = ui - ri;
    }
    out
}

#[cfg(test)]
#[allow(clippy::needless_range_loop)]
mod tests {
    use super::*;

    #[test]
    fn zero_vector_maps_to_zero() {
        assert_eq!(mat3_mul_vec(&MAT3_IDENTITY, &[0.0; 3]), [0.0; 3]);
    }

    #[test]
    fn inverse_roundtrip_and_singular() {
        let m: Mat3 = [[4.0, 1.0, 0.5], [1.0, 3.0, -0.2], [0.5, -0.2, 2.0]];
        let inv = mat3_inv(&m).expect("regular matrix");
        let p = mat3_mul(&m, &inv);
        for (r, row) in p.iter().enumerate() {
            for (c, v) in row.iter().enumerate() {
                let want = if r == c { 1.0 } else { 0.0 };
                assert!((v - want).abs() < 1e-6, "M·M⁻¹ = I to 1e-6 (f32 roundoff)");
            }
        }
        let s: Mat3 = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 1.0, 1.0]];
        assert!(mat3_inv(&s).is_none(), "rank-2 matrix is rejected");
        assert!(
            (mat3_det(&m) - 20.89).abs() < 1e-4,
            "det by hand: 4·5.96 − 2.1 − 0.85 = 20.89"
        );
    }

    #[test]
    fn pinv_is_right_inverse_with_weights() {
        let mut t: ModalMap = [[0.0; MAX_MAGNETS]; 3];
        let xs = [0.3f32, 0.3, -0.3, -0.3, 0.1];
        let ys = [0.2f32, -0.2, 0.2, -0.2, 0.0];
        for i in 0..5 {
            t[0][i] = 1.0;
            t[1][i] = xs[i];
            t[2][i] = ys[i];
        }
        let mut w = [1.0f32; MAX_MAGNETS];
        w[4] = 3.0;
        let a = weighted_pinv(&t, &w, 5, 3).expect("full rank");
        for r in 0..3 {
            for c in 0..3 {
                let v: f32 = (0..5).map(|i| t[r][i] * a[i][c]).sum();
                let want = if r == c { 1.0 } else { 0.0 };
                assert!((v - want).abs() < 1e-6, "T·A = I to 1e-6 (LEV-009)");
            }
        }
        for row in a.iter().skip(5) {
            assert_eq!(*row, [0.0; 3], "unused magnets get no allocation");
        }
        // Out-of-range arguments.
        assert!(weighted_pinv(&t, &w, 0, 3).is_none());
        assert!(weighted_pinv(&t, &w, 5, 4).is_none());
        let mut wz = w;
        wz[1] = 0.0;
        assert!(
            weighted_pinv(&t, &wz, 5, 3).is_none(),
            "zero weight rejected"
        );
    }

    #[test]
    fn null_space_has_no_modal_content() {
        let mut t: ModalMap = [[0.0; MAX_MAGNETS]; 3];
        for i in 0..4 {
            t[0][i] = 0.25;
            t[1][i] = if i < 2 { 0.25 } else { -0.25 };
        }
        let a = weighted_pinv(&t, &[1.0; MAX_MAGNETS], 4, 2).expect("rank 2");
        let u = [1.0, -2.0, 0.5, 3.0, 9.0, 9.0, 9.0, 9.0];
        let nsp = null_space_part(&t, &a, &u, 4);
        let m = modal_of(&t, &nsp, 4);
        assert!(m[0].abs() < 1e-6 && m[1].abs() < 1e-6, "T·null = 0 to 1e-6");
        assert_eq!(nsp[4..], [0.0; 4], "inactive magnets are zero");
    }
}
