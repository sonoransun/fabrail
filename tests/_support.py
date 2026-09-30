"""Shared helpers for the pytest suite (plan §2b item 13; core design §E.3). Owner: B6.

Import it from any test module as ``import _support`` (``tests/`` is on ``sys.path``
through ``conftest.py``). Nothing here is a test; pytest does not collect this file.

Contents:

- **Paths**: :data:`ROOT`, :data:`DOCS`, :data:`TESTS`, :data:`TEST_DATA`,
  :data:`ANCHORS_SEED42` (written by C1 in Wave 3, never by hand),
  :data:`README_ORIGINAL` (the user's original README, byte for byte).
- **Per-test (fidelity, duration) table**: :data:`TEST_PLAN` maps a test token
  (``py:<stem>::<fn>``) to the plant tier, simulated duration, coil mode and tier
  (fast or ``slow``) that keeps it inside the 15 s budget of a not-slow test (plan §2b
  item 13). ``conftest.py`` warns when a test the table lists as slow is not marked
  ``slow``, and prints the slowest tests with their row at the end of a session.
- **Session run cache**: :func:`cached_run` runs a catalog scenario once per session
  and key (class, scenario, seed, coil, tier, duration, record rate, overrides, plant,
  options) and hands every later caller the same :class:`CachedRun`. Runs are
  deterministic (FAB-SW-002), so sharing them between tests changes no result; it only
  removes repeated work. The cache is bounded (least recently used first out).
- **Seed-42 anchors**: :func:`load_seed42_anchors` / :func:`seed42_anchors_or_skip`
  (skip with the owner and command until C1 writes the file).
- **Docs helpers**: :func:`data_block` (the ``/* NAME:BEGIN */ var NAME = {…};
  /* NAME:END */`` JSON of a generated docs data file), :func:`md_prose` (Markdown lines
  with blockquotes and inline code removed, for vocabulary guards),
  :func:`physics_sections` (the ``§n`` headings of PHYSICS.md).
- **Tools**: :func:`node_bin`, :func:`cargo_bin`, :func:`rustup_stable_rustc`,
  :func:`run_script`.

Default coil mode for :func:`cached_run` is ``"voltage"``: SIL ≡ firmware is claimed
in voltage coil mode only (plan §2b item 16; FAB-FW-006), and W1.5 closed Carriage6 in
that mode. Pass ``coil="lag"`` for the faster simulation-only approximation.
"""

from __future__ import annotations

import collections
import json
import os
import re
import shutil
import subprocess
import time
from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
TESTS = ROOT / "tests"
TEST_DATA = TESTS / "data"
#: Seed-42 acceptance anchors (fabrail.anchors/1). Written by C1 in Wave 3 with
#: ``python -m fabrail anchors --write-tests``; never hand-edited (core design §E.5).
ANCHORS_SEED42 = TEST_DATA / "anchors_seed42.json"
#: The user's README as it stood before this repository was built (470 bytes).
README_ORIGINAL = TEST_DATA / "readme_original.md"
#: SHA-256 of :data:`README_ORIGINAL`, recorded from the session's first read of
#: README.md (2026-09-26); guards the fixture itself against edits.
README_ORIGINAL_SHA256 = "581ef25f8a731c902337045c572c7a363b9dc2c378b1f63144752d6699356423"
README_ORIGINAL_BYTES = 470

#: Wall-time limit of one not-slow test's call phase, s (plan §2b item 13; FAB-SW-008).
TEST_TIME_LIMIT_S = 15.0
#: Wall-time limit of the whole not-slow tier, s (FAB-SW-008; core design §E.6).
TIER_TIME_LIMIT_S = 120.0

_BUILD_HINT = "build it with `maturin develop --release` (see CLAUDE.md)"


# --------------------------------------------------------------------------- native


def native_or_none() -> Any:
    """``fabrail._native`` or None when it is not built."""
    try:
        from fabrail import _native
    except ImportError:
        return None
    return _native


def require_native() -> Any:
    """``fabrail._native``; skips the calling test when the extension is not built."""
    import pytest

    mod = native_or_none()
    if mod is None:
        pytest.skip(f"fabrail._native is not built; {_BUILD_HINT}")
    return mod


# --------------------------------------------------------------------------- test plan


@dataclass(frozen=True)
class PlanRow:
    """How one expensive test stays inside its budget.

    ``fidelity`` is the plant tier (carriage6, heave1, travel1d, structure; "detector"
    for DetectorReplay-only tests, "analytic" for closed forms), ``duration_s`` the
    simulated time per run (None: the catalog default), ``coil`` the coil mode, ``slow``
    whether the test belongs in the ``slow`` tier, and ``note`` the reason.
    """

    __test__ = False  # not a pytest class

    fidelity: str
    duration_s: float | None
    coil: str = "voltage"
    slow: bool = False
    note: str = ""


#: The per-test (fidelity, duration) table of plan §2b item 13. Keys are test tokens
#: (``py:<stem>::<fn>``, SPECIFICATION §0.7). Rows follow the core design §E.3 catalog
#: and the plan's runtime decisions: thermal on Heave1 with a compressed ramp; the
#: WALL/HEAVY rail traverse and M7.8 in the slow tier; the 24 h noise run through
#: DetectorReplay in 10-minute chunks; the isolation Monte Carlo N = 200 slow.
#: Measured Wave-1.5 costs (release, M-series): Carriage6 DOOR voltage ≈ 16× real
#: time, Carriage6 HEAVY ≈ 14×, Travel1d WALL ≈ 120× (the executive runs at 20 kHz in
#: every tier, lead decision D8).
TEST_PLAN: dict[str, PlanRow] = {
    # levitation / fail direction / propulsion (B4)
    "py:levitation::thermal_drift_absorbed": PlanRow("heave1", 20.0, note="compressed thermal ramp on Heave1 (plan §2b item 13)"),
    "py:levitation::rail_irregularity_tracking": PlanRow(
        "carriage6", None, note="DRAWER/DOOR on Carriage6; the WALL/HEAVY traverse is slow"
    ),
    "py:levitation::rail_irregularity_tracking_heavy_classes": PlanRow(
        "carriage6", None, slow=True, note="WALL/HEAVY rail traverse (plan §2b item 13)"
    ),
    "py:levitation::drawer_pulse_budget": PlanRow("carriage6", 3.0, note="unseat + slot exit, 48 kg"),
    "py:levitation::payload_estimate_converges": PlanRow("carriage6", 5.0),
    "py:propulsion::thermal_expansion_homing": PlanRow("travel1d", None),
    # haptics (B4)
    "py:haptics::false_intent_suite": PlanRow("carriage6", None, note="several short disturbance runs; cache them"),
    "py:haptics::terminus_semilog_fit": PlanRow("carriage6", None, note="Carriage6 only (FAB-HAP-013)"),
    "py:haptics::inertial_residual_transmissibility": PlanRow(
        "carriage6", 20.0, note="research tier; a 2 Hz and a 5 Hz sine window"
    ),
    # safety / power (B4)
    "py:safety::safe_landing_on_supply_loss": PlanRow("carriage6", 4.0),
    "py:power::cycle_energy_report": PlanRow("travel1d", None),
    # seismic (B5)
    "py:seismic_detector::noise_24h_no_protect": PlanRow(
        "detector", 86_400.0, slow=True, note="DetectorReplay streamed in 10-minute chunks"
    ),
    "py:seismic_policy::scenario_m5_r10": PlanRow("travel1d", None, note="ensemble; keep n small in the fast tier"),
    "py:seismic_policy::scenario_m7_r20": PlanRow("travel1d", None),
    "py:seismic_policy::scenario_m78_r100": PlanRow("travel1d", None, slow=True, note="long record (plan §2b item 13)"),
    "py:isolation::monte_carlo_bumper_overload": PlanRow(
        "travel1d", 60.0, slow=True, note="N = 200 × 60 s on 8 workers (lead decision D8)"
    ),
    "py:tmd_amd::closed_loop_structure_2dof": PlanRow("structure", None),
    # guards (B6)
    "py:acceptance::performance_budget": PlanRow("carriage6", 1.0, slow=True, note="real-time factors, MC, tier time"),
    "py:acceptance::seed42_anchors": PlanRow(
        "carriage6", None, note="compute_anchors(quick=True): push_open, unseat 26 kg, payload_step (≈ 2 s)"
    ),
    "py:acceptance::seed42_anchors_full": PlanRow(
        "travel1d", None, slow=True, note="every anchor incl. noise_1h and the n = 20 isolation ensemble (≈ 40 s)"
    ),
    "py:determinism::same_seed_identical_json": PlanRow("carriage6", 1.0, note="plus Travel1d pwave_m7_r20"),
    "py:determinism::ensemble_worker_invariance": PlanRow("travel1d", None, note="4 members × 46 s"),
    "py:firmware::firmware_link_size": PlanRow("analytic", None, slow=True, note="full firmware build and link"),
    "py:firmware::cross_check": PlanRow("analytic", None, slow=True, note="real cross builds (tier 4)"),
    # examples (E5)
    "py:examples::villa_full_matrix": PlanRow("carriage6", None, slow=True, note="every variant × building"),
}


def plan_for(token: str) -> PlanRow | None:
    """The :data:`TEST_PLAN` row of a test token, or None."""
    return TEST_PLAN.get(token)


def token_of(nodeid: str) -> str | None:
    """The SPECIFICATION §0.7 token of a pytest node id (parametrisation stripped).

    ``tests/test_haptics.py::test_soft_close[door]`` → ``py:haptics::soft_close``;
    ``tests/exotic/test_pfh.py::test_x`` → ``py:exotic/pfh::x``.
    """
    path, _, rest = nodeid.partition("::")
    fn = rest.split("::")[-1].split("[", 1)[0]
    p = Path(path)
    parts = list(p.parts)
    if parts and parts[0] == "tests":
        parts = parts[1:]
    if not parts or not parts[-1].startswith("test_") or not fn.startswith("test_"):
        return None
    stem = parts[-1][len("test_") :].removesuffix(".py")
    return "py:" + "/".join((*parts[:-1], stem)) + "::" + fn[len("test_") :]


# --------------------------------------------------------------------------- run cache


def _freeze(value: Any) -> Any:
    """A hashable, order-independent form of a JSON-like value (for cache keys)."""
    if isinstance(value, Mapping):
        return tuple(sorted((str(k), _freeze(v)) for k, v in value.items()))
    if isinstance(value, list | tuple):
        return tuple(_freeze(v) for v in value)
    if isinstance(value, float) and value != value:  # NaN never equals itself
        return ("nan",)
    return value


@dataclass(frozen=True)
class RunKey:
    """Everything that determines a catalog run (FAB-SW-002: same key → same bytes)."""

    rail_class: str
    scenario: str
    seed: int = 42
    coil: str = "voltage"
    fidelity: str | None = None
    duration_s: float | None = None
    record_rate_hz: float = 1000.0
    overrides: tuple = ()
    plant: tuple = ()
    options: tuple = ()


@dataclass
class CachedRun:
    """One finished catalog run, shared by every test that asks for the same key.

    Attributes:
        key: the cache key.
        params: the resolved ``_native.Params`` (overrides and coil applied).
        spec: the ``fabrail.scenario/1`` dict the run used.
        data: the native ``RunData``.
        wall_s: wall time of ``_native.run``, s (measured here, never hashed).
    """

    key: RunKey
    params: Any
    spec: dict[str, Any]
    data: Any
    wall_s: float
    _result: Any = field(default=None, repr=False)

    @property
    def result(self) -> Any:
        """The run as a :class:`fabrail.results.RunResult` (decoded once)."""
        if self._result is None:
            from fabrail.results import RunResult

            self._result = RunResult.from_native(self.data)
        return self._result

    @property
    def kpis(self) -> dict[str, float]:
        return dict(self.data.kpis())

    @property
    def events(self) -> list[tuple[float, str, float, float]]:
        return list(self.data.events())

    @property
    def meta(self) -> dict[str, Any]:
        return json.loads(self.data.meta_json())

    def record(self, include_channels: bool = False) -> dict[str, Any]:
        """``fabrail.export.run_record`` of this run (fresh dict; ``timing`` excluded from ``sha256``)."""
        from fabrail import export

        return export.run_record(self.result, include_channels=include_channels)

    @property
    def realtime_factor(self) -> float:
        """Simulated seconds per wall second of ``_native.run``."""
        dur = float(self.spec.get("duration_s", 0.0))
        return dur / self.wall_s if self.wall_s > 0 else float("inf")


class RunCache:
    """Session-scoped cache of catalog runs (least recently used out beyond ``max_entries``)."""

    def __init__(self, max_entries: int = 48) -> None:
        self.max_entries = int(max_entries)
        self._runs: collections.OrderedDict[RunKey, CachedRun] = collections.OrderedDict()
        self.hits = 0
        self.misses = 0
        self.wall_s = 0.0

    def __len__(self) -> int:
        return len(self._runs)

    def __contains__(self, key: object) -> bool:
        return key in self._runs

    def clear(self) -> None:
        self._runs.clear()

    def stats(self) -> dict[str, float]:
        """Hits, misses, entries and the total wall time spent running, s."""
        return {"hits": self.hits, "misses": self.misses, "entries": len(self._runs), "wall_s": self.wall_s}

    def get(self, key: RunKey) -> CachedRun:
        """The run for ``key``, computing it on the first request."""
        hit = self._runs.get(key)
        if hit is not None:
            self._runs.move_to_end(key)
            self.hits += 1
            return hit
        self.misses += 1
        run = _execute(key)
        self.wall_s += run.wall_s
        self._runs[key] = run
        while len(self._runs) > self.max_entries:
            self._runs.popitem(last=False)
        return run


def _execute(key: RunKey) -> CachedRun:
    from fabrail import scenarios
    from fabrail.config import RunConfig

    native = require_native()
    cfg = RunConfig(
        rail_class=key.rail_class,
        seed=key.seed,
        duration_s=key.duration_s,
        coil=key.coil,
        record_rate_hz=key.record_rate_hz,
        overrides=dict(key.overrides),
        plant=dict(key.plant),
    )
    params = cfg.params()
    spec_d, series = scenarios.spec(key.scenario, params, cfg, fidelity=key.fidelity, options=dict(key.options) or None)
    scenario = native.Scenario.from_json(scenarios.to_json(spec_d), series)
    t0 = time.perf_counter()
    data = native.run(params, scenario)
    wall = time.perf_counter() - t0
    return CachedRun(key=key, params=params, spec=spec_d, data=data, wall_s=wall)


#: The session's run cache (a module global lives exactly one pytest session).
RUN_CACHE = RunCache()


def run_key(
    rail_class: str,
    scenario: str,
    *,
    seed: int = 42,
    coil: str = "voltage",
    fidelity: str | None = None,
    duration_s: float | None = None,
    record_rate_hz: float = 1000.0,
    overrides: Mapping[str, Any] | None = None,
    plant: Mapping[str, Any] | None = None,
    options: Mapping[str, Any] | None = None,
) -> RunKey:
    """The :class:`RunKey` for these arguments (overrides, plant keys and options unordered)."""
    return RunKey(
        rail_class=str(rail_class),
        scenario=str(scenario),
        seed=int(seed),
        coil=str(coil),
        fidelity=fidelity,
        duration_s=None if duration_s is None else float(duration_s),
        record_rate_hz=float(record_rate_hz),
        overrides=_freeze(dict(overrides or {})),
        plant=_freeze(dict(plant or {})),
        options=_freeze(dict(options or {})),
    )


def cached_run(rail_class: str, scenario: str, **kwargs: Any) -> CachedRun:
    """Run catalog ``scenario`` for ``rail_class`` once per session (see :func:`run_key`).

    Keyword arguments: ``seed`` (42), ``coil`` ("voltage"), ``fidelity`` (None = the
    catalog's recommended tier), ``duration_s`` (None = catalog default),
    ``record_rate_hz`` (1000), ``overrides`` (dotted ``--set`` paths), ``plant``
    (``--plant`` keys), ``options`` (builder options). Treat the returned run as
    read-only: it is shared.
    """
    return RUN_CACHE.get(run_key(rail_class, scenario, **kwargs))


# --------------------------------------------------------------------------- anchors


def load_seed42_anchors() -> dict[str, Any] | None:
    """The recorded seed-42 anchors (fabrail.anchors/1), or None before C1 writes them."""
    if not ANCHORS_SEED42.is_file():
        return None
    return json.loads(ANCHORS_SEED42.read_text(encoding="utf-8"))


def seed42_anchors_or_skip() -> dict[str, Any]:
    """The recorded seed-42 anchors; skips the calling test while the file is absent."""
    import pytest

    doc = load_seed42_anchors()
    if doc is None:
        pytest.skip(
            "tests/data/anchors_seed42.json is not recorded yet: C1 (Wave 3) writes it with "
            "`python -m fabrail anchors --write-tests` (core design §E.5; FAB-SW-003)"
        )
    return doc


# --------------------------------------------------------------------------- docs


def data_block(path: Path, name: str) -> dict[str, Any]:
    """The JSON object between ``/* NAME:BEGIN */ var NAME =`` and ``; /* NAME:END */``.

    Raises ``AssertionError`` with the file name when the block is absent or not JSON.
    """
    text = path.read_text(encoding="utf-8")
    m = re.search(
        rf"/\*\s*{name}:BEGIN\s*\*/\s*var\s+{name}\s*=\s*(\{{.*\}})\s*;\s*/\*\s*{name}:END\s*\*/",
        text,
        re.S,
    )
    assert m, f"{path.relative_to(ROOT)}: no `/* {name}:BEGIN */ var {name} = {{…}}; /* {name}:END */` block"
    try:
        return json.loads(m.group(1))
    except json.JSONDecodeError as e:  # pragma: no cover - reported by the assertion
        raise AssertionError(f"{path.relative_to(ROOT)}: the {name} block is not JSON ({e})") from e


_INLINE_CODE = re.compile(r"(`+)(?:(?!\1).)+?\1")
_FENCE = re.compile(r"^\s{0,3}(```|~~~)")


def md_prose(text: str, *, skip_fenced: bool = False) -> Iterator[tuple[int, str]]:
    """Markdown lines as (line number, text) with blockquotes and inline code removed.

    "Quoted prompt blocks" in the vocabulary rule (SPECIFICATION §6; FAB-EXO-005,
    FAB-EXA-003) are Markdown blockquotes and inline code. Fenced code blocks are
    kept unless ``skip_fenced`` (they are prose to the vocabulary rule).
    """
    fenced = False
    for lineno, line in enumerate(text.splitlines(), 1):
        if _FENCE.match(line):
            fenced = not fenced
            if skip_fenced:
                continue
        if fenced and skip_fenced:
            continue
        if line.lstrip().startswith(">"):
            continue
        yield lineno, _INLINE_CODE.sub(" ", line)


_PHYS_HEADING = re.compile(r"^#{2,4}\s*§\s*(\d+(?:\.\d+)*)\b", re.M)


def physics_sections(path: Path | None = None) -> set[str]:
    """Section numbers with a heading in docs/PHYSICS.md (``## §7 …`` → "7", ``### §16.1`` → "16.1")."""
    p = path or (DOCS / "PHYSICS.md")
    return set(_PHYS_HEADING.findall(p.read_text(encoding="utf-8")))


# --------------------------------------------------------------------------- tools


def node_bin() -> str | None:
    """Path of ``node`` or None."""
    return shutil.which("node")


def cargo_bin() -> str | None:
    """Path of the host ``cargo`` or None."""
    return shutil.which("cargo")


def rustup_stable_rustc() -> str | None:
    """The rustup stable toolchain's rustc (the cross/firmware compiler), or None."""
    rustup = shutil.which("rustup")
    if rustup is None:
        return None
    try:
        out = subprocess.run(
            [rustup, "which", "rustc", "--toolchain", "stable"],
            capture_output=True,
            text=True,
            timeout=30,
            env={**os.environ, "RUSTUP_AUTO_INSTALL": "0"},
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    path = out.stdout.strip()
    return path if out.returncode == 0 and path and Path(path).exists() else None


def pytest_target_dir(name: str) -> Path:
    """A persistent cargo target dir for test-driven builds (``target/pytest-<name>``).

    Kept apart from the agents' and the developer's target dirs (CLAUDE.md: one target
    dir per build owner avoids lock contention) and reused across sessions (warm).
    """
    return ROOT / "target" / f"pytest-{name}"


@dataclass(frozen=True)
class ScriptResult:
    """Outcome of :func:`run_script`."""

    returncode: int
    stdout: str
    stderr: str
    wall_s: float

    @property
    def tail(self) -> str:
        """The last 60 lines of stdout and stderr (for assertion messages)."""
        lines = (self.stdout + "\n" + self.stderr).strip().splitlines()
        return "\n".join(lines[-60:])


def run_script(
    args: Sequence[str],
    *,
    env: Mapping[str, str] | None = None,
    timeout: float = 600.0,
    cwd: Path | None = None,
) -> ScriptResult:
    """Run a repository script or tool; never raises on a non-zero exit."""
    full_env = {**os.environ, **dict(env or {})}
    t0 = time.perf_counter()
    r = subprocess.run(list(args), capture_output=True, text=True, timeout=timeout, cwd=str(cwd or ROOT), env=full_env)
    return ScriptResult(r.returncode, r.stdout, r.stderr, time.perf_counter() - t0)


def rel(path: Path) -> str:
    """``path`` relative to the repository root (posix)."""
    try:
        return path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        return path.as_posix()
