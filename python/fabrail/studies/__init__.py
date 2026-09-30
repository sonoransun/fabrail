"""Exploratory studies: runnable, plotted answers to specific questions (core §C.6; plan S4).

Each study writes ``output/studies/<name>.png`` and a ``fabrail.study/1`` JSON record
holding every plotted number::

    python -m fabrail study --list
    python -m fabrail study pole_map
    python -m fabrail study all --tier 1

Studies are discovered with :mod:`pkgutil`: every module in this package with a
module-level ``STUDY = StudySpec(...)`` is one (see :mod:`fabrail.studies.runner`).

Control studies (B2, tier 1; each backs a specific claim):

- ``pole_map``: open- and closed-loop levitation poles per class and mode (LEV-003, LEV-007).
- ``zero_power_band``: the zero-power float gap versus mass, and the band clamp (LEV-004/005/018).
- ``fail_direction``: which way a carriage goes on control loss and how hard it lands (LEV-006, LEV-019).
- ``drawer_envelope``: DRAWER contactless extension, unseat, re-lift and slot exit versus mass (LEV-016/017/019).
- ``gain_margin``: the critical eddy lag τ_e per class and mode (LEV-020).
- ``drag_ledger``: the levitated drag ledger and μ_eq over the uncertain rail-drag range (HAP-010).
- ``speed_energy``: which pedestrian limit sets each speed cap, and governed-move margins (SAF-001/002).
- ``clamp_force``: F = √(2Ek) against simulated obstructions (SAF-004/005/006).
- ``haptic_zwidth``: the passive stiffness range (Z-width) and the passivity observer (HAP-009).
- ``terminus_semilog``: the log-damped termini as straight semilog capture lines (HAP-013…016).

Seismic and structure studies are B3's modules in the same package.
"""

from __future__ import annotations

from .runner import StudyResult, StudySpec, discover, execute, quick, run_study

__all__ = ["StudyResult", "StudySpec", "discover", "execute", "names", "quick", "run_study"]


def names(tier: int | None = None) -> list[str]:
    """Discovered study names, optionally restricted to one tier."""
    return [n for n, s in discover().items() if tier is None or s.tier == tier]
