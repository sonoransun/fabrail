"""Seed splitting and Monte Carlo ensembles (core §A.4 rule 5, §B.8).

Seeds:

- One run: ``SeedSequence(seed).spawn(2)``. Child 0 seeds the numpy synthesis
  generator; child 1 gives ``generate_state(1, np.uint64)[0]``, the Rust
  ``noise_seed`` (sensor noise and fault injection, ``fabrail_sim::rng``).
- Ensembles: ``SeedSequence(seed).spawn(n)``; member ``i`` splits child ``i`` the same
  way. ``run_batch`` returns results in index order, so ensembles do not depend on
  ``--workers``. A single ``simulate`` run is not ensemble member 0 (documented).

numpy does not promise ``Generator`` streams across versions (NEP 19): anchors are
recorded with numpy 2.4.3 and re-recorded if numpy changes (CLAUDE.md).

:func:`run_ensemble` re-synthesises every member's scenario inputs (ground motion,
hand-profile jitter, rail profile) from that member's generator, so the members are
independent draws; the Rust sensor noise follows each member's ``noise_seed``.

Owner: W0-C2 (stub; the seed split is complete), then A9 (Wave 1).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ._native import Params
    from .config import RunConfig
    from .results import EnsembleResult

__all__ = ["MemberSeed", "member_seeds", "run_ensemble", "split_seed"]


def _child(ss: np.random.SeedSequence, k: int) -> np.random.SeedSequence:
    """Child ``k`` of ``ss`` exactly as ``ss.spawn`` would make it, without mutating ``ss``."""
    return np.random.SeedSequence(ss.entropy, spawn_key=(*ss.spawn_key, k), pool_size=ss.pool_size)


def split_seed(seed: int | np.random.SeedSequence) -> tuple[np.random.SeedSequence, int]:
    """Split one run's seed into (synthesis SeedSequence, Rust ``noise_seed``).

    Pure: the same input always gives the same output (a SeedSequence argument is not
    mutated). Use ``np.random.default_rng(synth)`` for the synthesis generator.
    """
    ss = seed if isinstance(seed, np.random.SeedSequence) else np.random.SeedSequence(int(seed))
    synth = _child(ss, 0)
    noise = _child(ss, 1)
    return synth, int(noise.generate_state(1, np.uint64)[0])


@dataclass(frozen=True)
class MemberSeed:
    """Seeds of ensemble member ``index``."""

    index: int
    synth: np.random.SeedSequence
    noise_seed: int

    def rng(self) -> np.random.Generator:
        """A fresh synthesis generator for this member."""
        return np.random.default_rng(self.synth)


def member_seeds(seed: int, n: int) -> list[MemberSeed]:
    """Seeds for ``n`` ensemble members from master ``seed`` (index order)."""
    if n < 0:
        raise ValueError(f"member_seeds: n must be >= 0 (got {n})")
    root = np.random.SeedSequence(int(seed))
    out = []
    for i in range(n):
        synth, noise_seed = split_seed(_child(root, i))
        out.append(MemberSeed(i, synth, noise_seed))
    return out


def run_ensemble(
    params: Params,
    scenario: str,
    cfg: RunConfig,
    n: int,
    *,
    fidelity: str | None = None,
    options: dict[str, object] | None = None,
) -> EnsembleResult:
    """Run ``n`` members of catalog scenario ``scenario`` through ``_native.run_batch``.

    Member ``i`` is built by :func:`fabrail.scenarios.build` from ``member_seeds(cfg.seed,
    n)[i]`` (its own synthesis generator and ``noise_seed``); the batch runs on
    ``cfg.workers`` threads and returns in index order, so the result does not depend on
    the worker count (FAB-SW determinism). ``fidelity`` and ``options`` pass through to
    the scenario builder.
    """
    from . import require_native, scenarios
    from .results import EnsembleResult, RunResult

    if n < 1:
        raise ValueError(f"run_ensemble: n must be >= 1 (got {n})")
    members = member_seeds(cfg.seed, n)
    jobs = [(params, scenarios.build(scenario, params, cfg, fidelity=fidelity, member=m, options=options)) for m in members]
    data = require_native().run_batch(jobs, cfg.workers)
    results = [RunResult.from_native(d) for d in data]
    meta: dict[str, object] = {
        "n": n,
        "seed": cfg.seed,
        "rail_class": params.rail_class.key,
        "scenario": scenario,
        "fidelity": scenarios.resolve_fidelity(scenario, cfg, fidelity),
        "coil": cfg.coil,
        "seed_rule": "SeedSequence(seed).spawn(n)[i] -> (numpy synthesis, Rust noise_seed)",
        "numpy": np.__version__,
    }
    return EnsembleResult(
        seed=cfg.seed, noise_seeds=[m.noise_seed for m in members], members=results, scenario=scenario, meta=meta
    )
