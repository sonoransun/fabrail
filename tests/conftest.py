"""Shared pytest configuration (plan §2b item 13; core design §E.3). Owner: B6.

What it does:

- **Headless plots.** ``MPLBACKEND=Agg`` is set before anything imports matplotlib.
- **Import paths.** ``python/`` (so ``import fabrail`` works on a fresh checkout; after
  ``maturin develop`` it also holds ``fabrail/_native*.so``) and ``tests/`` (for
  ``import _support``) are on ``sys.path``.
- **Release build only.** The session stops (usage error) when
  ``fabrail._native.BUILD_PROFILE`` is not ``"release"``: the anchors and the time
  budgets assume a release build (plan §2b item 13; FAB-SW-008).
  ``FABRAIL_ALLOW_DEBUG_NATIVE=1`` overrides it for local debugging.
- **15 s per not-slow test.** A passing test without the ``slow`` marker whose call
  phase takes longer than ``FABRAIL_TEST_TIME_LIMIT_S`` (default 15 s) is turned into
  a failure. ``0`` disables the guard while profiling.
- **120 s for the not-slow tier.** When the whole default tier runs (``-m 'not slow'``
  over ``tests/``, no ``-k``), a session longer than ``FABRAIL_TIER_TIME_LIMIT_S``
  (default 120 s) exits with status 1 (FAB-SW-008; ``0`` disables).
- **Per-test (fidelity, duration) table.** ``_support.TEST_PLAN``: a test the table
  puts in the slow tier but that lacks the ``slow`` marker gets a warning, and the
  terminal summary lists the slowest tests with their table row.
  ``FABRAIL_TEST_TIMES=<path>`` also writes every test's phase durations as JSON.
- **Session run cache.** Fixture ``run_cache`` (and ``_support.cached_run``): each
  (class, scenario, seed, coil, tier, …) runs once per session.
- **Tool markers.** ``requires_cargo`` tests skip when ``cargo`` is not on PATH and
  ``node`` tests skip when ``node`` is not.

Fixtures: ``repo_root``, ``native`` (skips when the extension is not built),
``run_cache``, ``node_bin_path``, ``seed42_anchors``.
"""

from __future__ import annotations

import importlib
import json
import os
import sys
import time
import warnings
from pathlib import Path

os.environ["MPLBACKEND"] = "Agg"  # before any matplotlib import (headless; plan §2b item 13)

ROOT = Path(__file__).resolve().parents[1]
_TESTS = str(ROOT / "tests")
_PY_SRC = str(ROOT / "python")
for _p in (_PY_SRC, _TESTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import pytest  # noqa: E402

import _support  # noqa: E402

#: Wall-time limit for one not-slow test's call phase, s (plan §2b item 13; 0 disables).
TEST_TIME_LIMIT_S = float(os.environ.get("FABRAIL_TEST_TIME_LIMIT_S", str(_support.TEST_TIME_LIMIT_S)))
#: Wall-time limit for the whole not-slow tier, s (FAB-SW-008; 0 disables).
TIER_TIME_LIMIT_S = float(os.environ.get("FABRAIL_TIER_TIME_LIMIT_S", str(_support.TIER_TIME_LIMIT_S)))

_BUILD_HINT = "build it with `maturin develop --release` (see CLAUDE.md)"

#: nodeid -> {"setup": s, "call": s, "teardown": s, "slow": bool, "outcome": str}
_DURATIONS: dict[str, dict[str, object]] = {}
_SESSION_T0: list[float] = []
_TIER_FAILURE: list[str] = []


def _load_native():
    try:
        return importlib.import_module("fabrail._native")
    except ImportError:
        return None


# ---------------------------------------------------------------------------- fixtures


@pytest.fixture(scope="session")
def repo_root() -> Path:
    """The repository root."""
    return ROOT


@pytest.fixture(scope="session")
def native():
    """The compiled ``fabrail._native`` module; skips the test when it is not built."""
    mod = _load_native()
    if mod is None:
        pytest.skip(f"fabrail._native is not built; {_BUILD_HINT}")
    return mod


@pytest.fixture(scope="session")
def run_cache() -> _support.RunCache:
    """The session run cache (``_support.RUN_CACHE``): ``run_cache.get(_support.run_key(...))``."""
    return _support.RUN_CACHE


@pytest.fixture(scope="session")
def node_bin_path() -> str:
    """Path of ``node``; skips the test when node is not installed."""
    node = _support.node_bin()
    if node is None:
        pytest.skip("node is not installed (needed for the JS/wasm checks)")
    return node


@pytest.fixture(scope="session")
def seed42_anchors() -> dict:
    """The recorded seed-42 anchors; skips until C1 writes tests/data/anchors_seed42.json."""
    return _support.seed42_anchors_or_skip()


# ---------------------------------------------------------------------------- session


def pytest_configure(config: pytest.Config) -> None:
    _SESSION_T0[:] = [time.perf_counter()]
    _DURATIONS.clear()
    _TIER_FAILURE.clear()


def pytest_sessionstart(session: pytest.Session) -> None:
    """Refuse a non-release ``_native`` (plan §2b item 13; FAB-SW-008)."""
    mod = _load_native()
    if mod is None:
        return
    profile = getattr(mod, "BUILD_PROFILE", "unknown")
    if profile != "release" and os.environ.get("FABRAIL_ALLOW_DEBUG_NATIVE") != "1":
        pytest.exit(
            f"fabrail._native is a {profile!r} build; the anchors and time budgets assume release: "
            f"{_BUILD_HINT} (or set FABRAIL_ALLOW_DEBUG_NATIVE=1 for local debugging)",
            returncode=pytest.ExitCode.USAGE_ERROR,
        )


def _stale_native(mod) -> str:
    """Why the loaded extension may predate the Rust sources ('' when it does not)."""
    so = getattr(mod, "__file__", None)
    if not so:
        return ""
    try:
        built = Path(so).stat().st_mtime
    except OSError:
        return ""
    newest, newest_path = 0.0, None
    for crate in (ROOT / "crates").glob("*"):
        for sub in ("src", "Cargo.toml"):
            base = crate / sub
            paths = [base] if base.is_file() else base.rglob("*.rs") if base.is_dir() else []
            for p in paths:
                try:
                    m = p.stat().st_mtime
                except OSError:
                    continue
                if m > newest:
                    newest, newest_path = m, p
    if newest > built and newest_path is not None:
        return f"{_support.rel(newest_path)} is newer than the built extension ({(newest - built) / 60:.0f} min)"
    return ""


def pytest_report_header(config: pytest.Config) -> list[str]:
    mod = _load_native()
    if mod is None:
        return [f"fabrail._native: not built ({_BUILD_HINT})"]
    lines = []
    try:
        h = mod.selftest_hash(2000)
        golden = int(getattr(mod, "GOLDEN_SELFTEST_HASH", 0))
        lines.append(
            f"fabrail._native: {getattr(mod, 'BUILD_PROFILE', '?')} build, selftest_hash(2000) = 0x{h:016x}, "
            f"GOLDEN_SELFTEST_HASH = {f'0x{golden:016x}' if golden else 'not frozen (C1, Wave 3)'}"
        )
    except Exception as e:  # noqa: BLE001 - a header must never break the session
        lines.append(f"fabrail._native: selftest failed: {e}")
    stale = _stale_native(mod)
    if stale:
        lines.append(f"fabrail._native may be stale: {stale}; rebuild with `maturin develop --release`")
    return lines


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    """Skip tool-dependent tests without the tool; warn on TEST_PLAN slow-tier mismatches."""
    have_cargo = _support.cargo_bin() is not None
    have_node = _support.node_bin() is not None
    for item in items:
        if item.get_closest_marker("requires_cargo") and not have_cargo:
            item.add_marker(pytest.mark.skip(reason="needs cargo on PATH (requires_cargo)"))
        if item.get_closest_marker("node") and not have_node:
            item.add_marker(pytest.mark.skip(reason="needs node on PATH (node marker)"))
        token = _support.token_of(item.nodeid)
        row = _support.plan_for(token) if token else None
        if row is not None and row.slow and item.get_closest_marker("slow") is None:
            warnings.warn(
                f"{item.nodeid}: tests/_support.py TEST_PLAN puts {token} in the slow tier "
                f"({row.note or row.fidelity}); mark it @pytest.mark.slow",
                stacklevel=1,
            )


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item: pytest.Item, call: pytest.CallInfo):
    """Fail a passing not-slow test whose call phase exceeded ``TEST_TIME_LIMIT_S``; record durations."""
    outcome = yield
    report = outcome.get_result()
    slow = item.get_closest_marker("slow") is not None
    rec = _DURATIONS.setdefault(item.nodeid, {"slow": slow})
    rec[report.when] = round(float(call.duration), 6)
    if report.when == "call":
        rec["outcome"] = report.outcome
    if report.when == "call" and report.passed and TEST_TIME_LIMIT_S > 0 and not slow and call.duration > TEST_TIME_LIMIT_S:
        report.outcome = "failed"
        report.longrepr = (
            f"not-slow test took {call.duration:.1f} s > {TEST_TIME_LIMIT_S:g} s (plan §2b item 13; "
            "FAB-SW-008): shorten it (tests/_support.py TEST_PLAN, the run cache) or mark it @pytest.mark.slow"
        )
        rec["outcome"] = "failed (time)"


def _full_default_tier(config: pytest.Config) -> bool:
    """True when this session is the whole not-slow tier over tests/ (no -k, no node ids)."""
    if (config.option.markexpr or "").strip() != "not slow":
        return False
    if getattr(config.option, "keyword", ""):
        return False
    if getattr(config.option, "deselect", None):
        return False
    args = [a for a in (config.args or []) if a]
    if not args:
        return True
    tests_dir = (ROOT / "tests").resolve()
    for a in args:
        if "::" in a:
            return False
        p = Path(a)
        p = (Path(config.invocation_params.dir) / p) if not p.is_absolute() else p
        if p.resolve() != tests_dir:
            return False
    return True


def pytest_sessionfinish(session: pytest.Session, exitstatus: int) -> None:
    """Enforce the not-slow tier budget and write FABRAIL_TEST_TIMES."""
    wall = time.perf_counter() - (_SESSION_T0[0] if _SESSION_T0 else time.perf_counter())
    config = session.config
    out = os.environ.get("FABRAIL_TEST_TIMES")
    if out:
        try:
            Path(out).parent.mkdir(parents=True, exist_ok=True)
            Path(out).write_text(
                json.dumps({"session_wall_s": round(wall, 3), "tests": _DURATIONS}, indent=1, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        except OSError:
            pass
    if TIER_TIME_LIMIT_S > 0 and _full_default_tier(config) and wall > TIER_TIME_LIMIT_S and session.testscollected:
        _TIER_FAILURE.append(
            f"the not-slow tier took {wall:.1f} s > {TIER_TIME_LIMIT_S:g} s (FAB-SW-008; core design §E.6): "
            "move long tests to the slow tier or shorten them (tests/_support.py TEST_PLAN)"
        )
        if session.exitstatus == pytest.ExitCode.OK:
            session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter, exitstatus: int, config: pytest.Config) -> None:
    """Slowest tests with their TEST_PLAN row, run-cache statistics and the tier verdict."""
    if _DURATIONS:
        rows = sorted(
            ((float(r.get("setup", 0.0)) + float(r.get("call", 0.0)), nid, r) for nid, r in _DURATIONS.items()),
            reverse=True,
        )[:8]
        if rows and rows[0][0] >= 1.0:
            terminalreporter.section("fabrail: slowest tests (setup + call)", sep="-")
            for total, nid, r in rows:
                if total < 1.0:
                    break
                token = _support.token_of(nid)
                row = _support.plan_for(token) if token else None
                plan = f"  [{row.fidelity}, {row.duration_s or 'default'} s{', slow' if row.slow else ''}]" if row else ""
                tag = " slow" if r.get("slow") else ""
                terminalreporter.write_line(f"{total:7.2f} s{tag:5s} {nid}{plan}")
    stats = _support.RUN_CACHE.stats()
    if stats["misses"]:
        terminalreporter.write_line(
            f"fabrail run cache: {stats['misses']} runs, {stats['hits']} reuses, {stats['wall_s']:.1f} s in _native.run"
        )
    for msg in _TIER_FAILURE:
        terminalreporter.write_line(f"FAILED (tier budget): {msg}", red=True, bold=True)
