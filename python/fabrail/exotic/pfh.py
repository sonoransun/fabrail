"""PFH calculator for the 2oo3 architecture, both forms, with a CTMC cross-check (EXOTIC §X13).

The Rust implementation is ``fabrail_exo_safety::pfh`` (called through
``_native.exo_pfh_2oo3``). This module restates the closed forms in numpy and adds an
independent continuous-time Markov chain, so the three can be compared:

    λ_DD = DC·λ_D,  λ_DU = (1 − DC)·λ_D
    t_CE = (λ_DU/λ_D)·(T1/2 + MRT) + (λ_DD/λ_D)·MTTR
    conservative:  PFH = 6·[(1 − β_D)·λ_DD + (1 − β)·λ_DU]²·t_CE + β_D·λ_DD + β·λ_DU
    Ed. 2:         PFH = 6·[(1 − β_D)·λ_DD + (1 − β)·λ_DU]·(1 − β)·λ_DU·t_CE + β·λ_DU

Reference case (FAB-EXO-305): λ_D 1e-6/h, DC 99 %, β 2 %, β_D 1 %, T1 8,760 h,
MTTR = MRT = 8 h gives t_CE 51.8 h, 1.04e-8/h (conservative) and ≈ 2.0e-10/h (Ed. 2).
The two forms are about 50× apart, so the edition is always stated. Common cause
dominates.

These are architecture concepts used as design targets. The rates are placeholders
(there is no FMEDA), a CTMC sharing the closed forms' rate model checks the algebra and
the t_CE approximation, not the rates, and nothing here establishes a PFH or a SIL for
hardware. :func:`sil_target_band` names the numeric band a value falls in and nothing
more.

Owner: X7 (Wave 2).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

from ._util import native

__all__ = [
    "REFERENCE",
    "PfhInputs",
    "PfhResult",
    "beta_d_sweep",
    "ctmc_pfh",
    "pfh_2oo3",
    "pfh_conservative",
    "pfh_ed2",
    "sil_target_band",
    "t_ce_h",
]


@dataclass(frozen=True)
class PfhInputs:
    """Inputs of the 2oo3 PFH (rates per hour, times in hours)."""

    lambda_d_per_h: float = 1e-6
    dc: float = 0.99
    beta: float = 0.02
    beta_d: float = 0.01
    t1_h: float = 8760.0
    mttr_h: float = 8.0
    mrt_h: float = 8.0

    def __post_init__(self) -> None:
        for name in ("lambda_d_per_h", "t1_h", "mttr_h", "mrt_h"):
            v = getattr(self, name)
            if not (math.isfinite(v) and v > 0):
                raise ValueError(f"{name} must be finite and > 0 (got {v!r})")
        for name in ("dc", "beta", "beta_d"):
            v = getattr(self, name)
            if not (math.isfinite(v) and 0.0 <= v <= 1.0):
                raise ValueError(f"{name} must lie in [0, 1] (got {v!r})")

    @property
    def lambda_dd(self) -> float:
        """Dangerous detected rate DC·λ_D, 1/h."""
        return self.dc * self.lambda_d_per_h

    @property
    def lambda_du(self) -> float:
        """Dangerous undetected rate (1 − DC)·λ_D, 1/h."""
        return (1.0 - self.dc) * self.lambda_d_per_h


#: The EXOTIC §X13 reference case (FAB-EXO-305).
REFERENCE = PfhInputs()


def t_ce_h(p: PfhInputs = REFERENCE) -> float:
    """Channel-equivalent down time t_CE = (λ_DU/λ_D)·(T1/2 + MRT) + (λ_DD/λ_D)·MTTR, h."""
    return p.lambda_du / p.lambda_d_per_h * (p.t1_h / 2.0 + p.mrt_h) + p.lambda_dd / p.lambda_d_per_h * p.mttr_h


def pfh_conservative(p: PfhInputs = REFERENCE) -> float:
    """Conservative 2oo3 PFH (detected failures stay dangerous until repaired), 1/h."""
    x = (1.0 - p.beta_d) * p.lambda_dd + (1.0 - p.beta) * p.lambda_du
    return 6.0 * x * x * t_ce_h(p) + p.beta_d * p.lambda_dd + p.beta * p.lambda_du


def pfh_ed2(p: PfhInputs = REFERENCE) -> float:
    """IEC 61508-6 Ed. 2 high-demand 2oo3 PFH (the second failure must be undetected), 1/h [UNCERTAIN: recalled form]."""
    x = (1.0 - p.beta_d) * p.lambda_dd + (1.0 - p.beta) * p.lambda_du
    return 6.0 * x * (1.0 - p.beta) * p.lambda_du * t_ce_h(p) + p.beta * p.lambda_du


# CTMC states (Annex B.5.2 style): all good; one failed detected (repair at 1/MTTR); one
# failed undetected (found at the proof test); one found at the proof test (repair at
# 1/MRT); tripped (Ed. 2 only, repaired at 1/MTTR); failed dangerous (absorbing).
_OK, _D1, _U1, _R1, _TRIP, _FD = range(6)


def _generator(p: PfhInputs, form: str) -> np.ndarray:
    a = (1.0 - p.beta_d) * p.lambda_dd  # independent detected, per channel
    u = (1.0 - p.beta) * p.lambda_du  # independent undetected, per channel
    ccf_dd, ccf_du = p.beta_d * p.lambda_dd, p.beta * p.lambda_du
    q = np.zeros((6, 6))
    mu, mu_r = 1.0 / p.mttr_h, 1.0 / p.mrt_h
    q[_OK, _D1] = 3.0 * a
    q[_OK, _U1] = 3.0 * u
    if form == "conservative":
        q[_OK, _FD] = ccf_dd + ccf_du
        for s in (_D1, _U1, _R1):
            q[s, _FD] = 2.0 * (a + u) + ccf_dd + ccf_du
    elif form == "ed2":
        q[_OK, _FD] = ccf_du
        q[_OK, _TRIP] = ccf_dd
        for s in (_D1, _U1, _R1):
            q[s, _FD] = 2.0 * u + ccf_du
            q[s, _TRIP] = 2.0 * a + ccf_dd
        q[_TRIP, _OK] = mu
    else:
        raise ValueError(f"form must be 'conservative' or 'ed2' (got {form!r})")
    q[_D1, _OK] = mu
    q[_R1, _OK] = mu_r
    np.fill_diagonal(q, -q.sum(axis=1))
    return q


def ctmc_pfh(p: PfhInputs = REFERENCE, form: str = "conservative", n_intervals: int = 2) -> float:
    """Average PFH of the 2oo3 CTMC over ``n_intervals`` proof-test intervals, 1/h.

    The chain is integrated exactly between proof tests (matrix exponential,
    ``scipy.linalg.expm``); at each proof test every undetected single failure becomes
    known and enters repair at 1/MRT. The average dangerous-failure frequency is
    P(failed dangerous at n·T1)/(n·T1). Independent of the Rust CTMC (same rate model).
    """
    from scipy.linalg import expm

    if n_intervals < 1:
        raise ValueError("n_intervals must be >= 1")
    q = _generator(p, form)
    step = expm(q * p.t1_h)
    x = np.zeros(6)
    x[_OK] = 1.0
    for _ in range(n_intervals):
        x = x @ step
        x[_R1] += x[_U1]  # the proof test reveals the undetected failure
        x[_U1] = 0.0
    return float(x[_FD] / (n_intervals * p.t1_h))


@dataclass(frozen=True)
class PfhResult:
    """Both PFH forms from the Rust calculator and from this module, with the CTMC cross-checks (1/h)."""

    inputs: PfhInputs
    t_ce_h: float
    conservative: float
    ed2: float
    ctmc: float
    ctmc_ed2: float | None
    py_conservative: float
    py_ed2: float
    py_ctmc: float
    py_ctmc_ed2: float
    ratio_forms: float

    def as_dict(self) -> dict[str, object]:
        """JSON-ready dict, with the inputs and the stated edition of each form."""
        d = asdict(self)
        d["forms"] = {"conservative": "detected failures dangerous until repaired", "ed2": "IEC 61508-6 Ed. 2 (2010) high-demand form [UNCERTAIN: recalled]"}
        d["note"] = "Placeholder rates, no FMEDA: the values check the algebra, not a hardware PFH; nothing is certified."
        return d


def pfh_2oo3(p: PfhInputs = REFERENCE, n_intervals: int = 2) -> PfhResult:
    """Both forms from ``_native.exo_pfh_2oo3`` plus this module's closed forms and CTMC (FAB-EXO-305, FAB-EXO-117)."""
    r = native().exo_pfh_2oo3(p.lambda_d_per_h, p.dc, p.beta, p.beta_d, p.t1_h, p.mttr_h, p.mrt_h)
    ce = float(r["conservative"])
    e2 = float(r["ed2"])
    ctmc_ed2 = r.get("ctmc_ed2")
    return PfhResult(
        inputs=p,
        t_ce_h=float(r["t_ce_h"]),
        conservative=ce,
        ed2=e2,
        ctmc=float(r["ctmc"]),
        ctmc_ed2=None if ctmc_ed2 is None else float(ctmc_ed2),
        py_conservative=pfh_conservative(p),
        py_ed2=pfh_ed2(p),
        py_ctmc=ctmc_pfh(p, "conservative", n_intervals),
        py_ctmc_ed2=ctmc_pfh(p, "ed2", n_intervals),
        ratio_forms=ce / e2 if e2 > 0 else math.inf,
    )


#: Numeric PFH bands of the high-demand/continuous mode (IEC 61508-1 Table 3), 1/h.
_SIL_BANDS = ((4, 1e-9, 1e-8), (3, 1e-8, 1e-7), (2, 1e-7, 1e-6), (1, 1e-6, 1e-5))


def sil_target_band(pfh_per_h: float) -> str:
    """The numeric SIL target band a PFH value falls in, e.g. "SIL 3 band (1e-8 to 1e-7 /h)".

    A band name only: a computed value inside a band does not establish that SIL, and the
    architectures are designed toward their targets, not certified (EXOTIC §S3).
    """
    if not math.isfinite(pfh_per_h) or pfh_per_h < 0:
        raise ValueError(f"PFH must be finite and >= 0 (got {pfh_per_h!r})")
    for sil, lo, hi in _SIL_BANDS:
        if lo <= pfh_per_h < hi:
            return f"SIL {sil} band ({lo:g} to {hi:g} /h)"
    return "below the SIL 4 band" if pfh_per_h < 1e-9 else "above the SIL 1 band"


def beta_d_sweep(beta_d_values: tuple[float, ...] = (0.001, 0.002, 0.005, 0.01, 0.02), base: PfhInputs = REFERENCE) -> list[dict[str, float | str]]:
    """PFH (both forms) against β_D, showing that common cause sets the floor (EXOTIC §X13)."""
    out: list[dict[str, float | str]] = []
    for bd in beta_d_values:
        p = PfhInputs(base.lambda_d_per_h, base.dc, base.beta, bd, base.t1_h, base.mttr_h, base.mrt_h)
        c = pfh_conservative(p)
        out.append({"beta_d": bd, "conservative_per_h": c, "ed2_per_h": pfh_ed2(p), "conservative_band": sil_target_band(c)})
    return out
