"""Run results as numpy (core §B.8).

:meth:`RunResult.from_native` turns a ``fabrail._native.RunData`` into numpy arrays:
channels cross PyO3 as little-endian bytes (``"<f8"`` for physical channels, ``"u1"``
for state channels) and are read with ``np.frombuffer`` (no rust-numpy; core §A.1).
:class:`EnsembleResult` collects the members of an ensemble in index order, so it does
not depend on the worker count.

``meta`` is the Rust ``RunMeta`` (version, scenario, noise_seed, fidelity, coil,
params, spec, golden, build_profile, wall_s). ``wall_s`` is the only volatile key:
:meth:`RunResult.stable_meta` drops it, and the JSON export keeps it out of the hashed
payload (plan §2b item 16; ``fabrail.export``).

Owner: W0-C2 (stub; ``from_native`` implemented against the core §B.7 RunData
surface), then A8 (Wave 1).
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, NamedTuple

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ._native import RunData

__all__ = ["VOLATILE_META_KEYS", "EnsembleResult", "RunEvent", "RunResult"]

#: Metadata keys that differ between identical runs (wall-clock timing); never hashed.
VOLATILE_META_KEYS: frozenset[str] = frozenset({"wall_s"})

#: numpy dtypes of the native channel buffers (``RunData.channel`` dtype strings).
_NATIVE_DTYPES: Mapping[str, np.dtype] = {"<f8": np.dtype("<f8"), "u1": np.dtype("u1")}


class RunEvent(NamedTuple):
    """One recorded event: time, kind (a core ``EventKind`` name) and two payload values."""

    t_s: float
    kind: str
    a: float
    b: float


@dataclass
class RunResult:
    """One run: the time base, every recorded channel, events, KPIs and metadata.

    Attributes:
        rate_hz: recording rate, Hz.
        t_s: sample times, s (``arange(n) / rate_hz``).
        channels: channel name → samples (float64, or uint8 for state channels), in
            recording order.
        units: channel name → unit string.
        events: recorded events in time order.
        kpis: online KPIs (core §B.6 names; a KPI whose phase never happened is absent).
        meta: ``RunMeta`` (version, scenario, noise_seed, fidelity, coil, params,
            spec, golden, build_profile, wall_s).
    """

    rate_hz: float
    t_s: np.ndarray
    channels: dict[str, np.ndarray] = field(default_factory=dict)
    units: dict[str, str] = field(default_factory=dict)
    events: list[RunEvent] = field(default_factory=list)
    kpis: dict[str, float] = field(default_factory=dict)
    meta: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_native(cls, data: RunData) -> RunResult:
        """Decode a native ``RunData``.

        Each channel's little-endian buffer is read with ``np.frombuffer`` in its
        declared dtype and copied (the arrays are writable and own their memory).
        Raises ``ValueError`` for an unknown dtype string or ragged channels.
        """
        channels: dict[str, np.ndarray] = {}
        units: dict[str, str] = {}
        for name in data.channel_names():
            buf, dtype, unit = data.channel(name)
            if dtype not in _NATIVE_DTYPES:
                raise ValueError(f"{name}: unknown channel dtype {dtype!r} (expected '<f8' or 'u1')")
            dt = _NATIVE_DTYPES[dtype]
            arr = np.frombuffer(buf, dtype=dt).copy()
            channels[name] = arr.astype(np.float64 if dt.kind == "f" else np.uint8, copy=False)
            units[name] = str(unit)
        lengths = {len(v) for v in channels.values()}
        if len(lengths) > 1:
            raise ValueError(f"channels: ragged channel lengths {sorted(lengths)}")
        n = lengths.pop() if lengths else int(data.n_samples())
        rate = float(data.rate_hz)
        t = np.arange(n, dtype=np.float64) / rate if rate > 0 else np.zeros(n)
        events = [RunEvent(float(t_s), str(kind), float(a), float(b)) for t_s, kind, a, b in data.events()]
        return cls(
            rate_hz=rate,
            t_s=t,
            channels=channels,
            units=units,
            events=events,
            kpis={str(k): float(v) for k, v in data.kpis().items()},
            meta=json.loads(data.meta_json()),
        )

    def __getitem__(self, name: str) -> np.ndarray:
        return self.channels[name]

    def __contains__(self, name: object) -> bool:
        return name in self.channels

    @property
    def n_samples(self) -> int:
        return int(self.t_s.shape[0])

    @property
    def duration_s(self) -> float:
        """Recorded span n/rate, s (0 for an empty record)."""
        return self.n_samples / self.rate_hz if self.rate_hz > 0 else 0.0

    def channel_names(self) -> list[str]:
        """Channel names in recording order."""
        return list(self.channels)

    def events_of(self, kind: str) -> list[RunEvent]:
        """Events of one kind, in time order."""
        return [e for e in self.events if e.kind == kind]

    def first_event(self, kind: str) -> RunEvent | None:
        """The earliest event of ``kind``, or None."""
        found = self.events_of(kind)
        return min(found, key=lambda e: e.t_s) if found else None

    def window(self, t0_s: float, t1_s: float) -> RunResult:
        """The samples with t0_s ≤ t < t1_s and the events in the same window.

        KPIs are whole-run values and are carried unchanged; the time base keeps its
        absolute times.
        """
        if not (t1_s >= t0_s):
            raise ValueError(f"window: t1_s ({t1_s!r}) must not precede t0_s ({t0_s!r})")
        mask = (self.t_s >= t0_s) & (self.t_s < t1_s)
        return RunResult(
            rate_hz=self.rate_hz,
            t_s=self.t_s[mask].copy(),
            channels={k: v[mask].copy() for k, v in self.channels.items()},
            units=dict(self.units),
            events=[e for e in self.events if t0_s <= e.t_s < t1_s],
            kpis=dict(self.kpis),
            meta=dict(self.meta),
        )

    def at(self, name: str, t_s: float) -> float:
        """Channel ``name`` linearly interpolated at ``t_s`` (held at the ends)."""
        if self.n_samples == 0:
            raise ValueError(f"{name}: the run recorded no samples")
        return float(np.interp(t_s, self.t_s, self.channels[name].astype(np.float64)))

    def stable_meta(self) -> dict[str, Any]:
        """``meta`` without the volatile keys (:data:`VOLATILE_META_KEYS`)."""
        return {k: v for k, v in self.meta.items() if k not in VOLATILE_META_KEYS}

    def channel_summary(self) -> dict[str, dict[str, float]]:
        """Per channel: min, max, mean, first and last sample (NaN-aware; empty → NaN)."""
        out: dict[str, dict[str, float]] = {}
        for name, values in self.channels.items():
            x = np.asarray(values, dtype=np.float64)
            finite = x[np.isfinite(x)]
            if finite.size == 0:
                out[name] = {k: math.nan for k in ("min", "max", "mean", "first", "last")}
                continue
            scale = float(np.abs(finite).max())
            mean = scale * float(np.mean(finite / scale)) if scale > 0 else 0.0  # overflow-safe near ±f64::MAX
            out[name] = {
                "min": float(finite.min()),
                "max": float(finite.max()),
                "mean": mean,
                "first": float(x[0]),
                "last": float(x[-1]),
            }
        return out


@dataclass
class EnsembleResult:
    """An ensemble in member-index order (independent of ``--workers``).

    Attributes:
        seed: master seed.
        noise_seeds: per-member Rust ``noise_seed`` (``fabrail.ensemble.member_seeds``).
        members: per-member results.
        scenario: scenario name.
        meta: ensemble-level metadata.
    """

    seed: int
    noise_seeds: list[int] = field(default_factory=list)
    members: list[RunResult] = field(default_factory=list)
    scenario: str = ""
    meta: dict[str, Any] = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.members)

    def __iter__(self) -> Iterator[RunResult]:
        return iter(self.members)

    def __getitem__(self, index: int) -> RunResult:
        return self.members[index]

    @classmethod
    def from_members(
        cls,
        seed: int,
        members: Sequence[RunResult],
        noise_seeds: Sequence[int] | None = None,
        scenario: str | None = None,
        meta: Mapping[str, Any] | None = None,
    ) -> EnsembleResult:
        """Assemble an ensemble; missing noise seeds and scenario are read from member meta."""
        members = list(members)
        if noise_seeds is None:
            noise_seeds = [int(m.meta.get("noise_seed", 0)) for m in members]
        if len(noise_seeds) != len(members):
            raise ValueError(f"noise_seeds: {len(noise_seeds)} seeds for {len(members)} members")
        if scenario is None:
            scenario = str(members[0].meta.get("scenario", "")) if members else ""
        return cls(seed=int(seed), noise_seeds=[int(s) for s in noise_seeds], members=members, scenario=scenario, meta=dict(meta or {}))

    def kpi_names(self) -> list[str]:
        """KPI names present in every member."""
        if not self.members:
            return []
        common = set(self.members[0].kpis)
        for m in self.members[1:]:
            common &= set(m.kpis)
        return sorted(common)

    def all_kpi_names(self) -> list[str]:
        """KPI names present in at least one member."""
        names: set[str] = set()
        for m in self.members:
            names |= set(m.kpis)
        return sorted(names)

    def kpi(self, name: str) -> np.ndarray:
        """KPI ``name`` across members (NaN where a member lacks it)."""
        return np.array([m.kpis.get(name, np.nan) for m in self.members], dtype=np.float64)

    def kpi_table(self, names: Sequence[str] | None = None) -> Mapping[str, np.ndarray]:
        """``{name: kpi(name)}`` for ``names`` (default: :meth:`kpi_names`)."""
        return {n: self.kpi(n) for n in (names if names is not None else self.kpi_names())}

    def kpi_summary(
        self, names: Sequence[str] | None = None, quantiles: Sequence[float] = (0.05, 0.5, 0.95)
    ) -> dict[str, dict[str, float]]:
        """Summary statistics per KPI over the members that report it.

        Keys ``n`` (members with a finite value), ``mean``, ``std`` (population),
        ``min``, ``max`` and ``p<q·100>`` for each quantile (linear interpolation,
        e.g. ``p05``, ``p50``, ``p95``). A KPI no member reports gives n = 0 and NaNs.
        """
        out: dict[str, dict[str, float]] = {}
        for name in names if names is not None else self.all_kpi_names():
            x = self.kpi(name)
            x = x[np.isfinite(x)]
            row: dict[str, float] = {"n": float(x.size)}
            stats = ("mean", "std", "min", "max")
            if x.size:
                row.update(mean=float(x.mean()), std=float(x.std()), min=float(x.min()), max=float(x.max()))
            else:
                row.update({k: math.nan for k in stats})
            for q in quantiles:
                if not 0.0 <= q <= 1.0:
                    raise ValueError(f"quantiles: {q!r} is outside [0, 1]")
                row[f"p{round(q * 100):02d}"] = float(np.quantile(x, q)) if x.size else math.nan
            out[name] = row
        return out

    def count_exceeding(self, name: str, threshold: float, *, above: bool = True) -> tuple[int, int]:
        """(k, n): members whose KPI ``name`` exceeds ``threshold`` (strictly; ``above=False``
        counts values strictly below it) and members reporting the KPI. Feed the pair to
        ``fabrail.analysis.wilson_upper`` for the (MC) tolerance class."""
        x = self.kpi(name)
        x = x[np.isfinite(x)]
        k = int(np.count_nonzero(x > threshold if above else x < threshold))
        return k, int(x.size)
