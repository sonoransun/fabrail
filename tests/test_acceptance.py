"""Seed-42 acceptance anchors and performance budgets (FAB-SW-003, FAB-SW-008).

Contract:

- ``seed42_anchors``: ``tests/data/anchors_seed42.json`` (fabrail.anchors/1, written by
  C1 in Wave 3 with ``python -m fabrail anchors --write-tests``; core design §E.5) is
  well formed (its ``sha256`` is the payload hash, every anchor carries a finite value,
  a tolerance and a source), and a fresh seed-42 computation by the producer the CLI
  uses (``fabrail.analysis.compute_anchors``) reproduces every anchor of the quick set
  (DOOR push_open, DRAWER unseat 26 kg, DRAWER payload_step, the selftest hash; ≈ 2 s)
  within its recorded tolerance (``fabrail.export.check_anchors``). The full set
  (≈ 40 s: WALL P-wave, 1 h noise, the n = 20 isolation ensemble, …) is checked by the
  slow companion ``seed42_anchors_full``. Until C1 writes the file both are skipped
  with that reason.
- ``performance_budget`` (slow): real-time factors of the SIL harness (Carriage6 DOOR
  lag coil ≥ 10×, Carriage6 DOOR voltage ≥ 3×, Carriage6 HEAVY ≥ 5×, Travel1d WALL
  ≥ 100×), the isolation Monte Carlo N = 200 × 60 s including the Python synthesis on 8
  workers ≤ 60 s, and the not-slow pytest tier ≤ 120 s with every test ≤ 15 s against a
  release ``_native``. Budgets: FAB-SW-008 and core design §E.6, with the Travel1d and
  Monte Carlo budgets of lead decision D8 (Wave 2): the executive keeps its full
  20 kHz rate in every tier for SIL fidelity, so Travel1d is budgeted at ≥ 100× real
  time instead of the original ≥ 1000×. Each factor is the best of three runs (the host
  is shared); every measured value is printed.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time

import pytest

import _support

#: Real-time budgets: (label, preset, tier, coil, simulated s, minimum factor, source).
RTF_BUDGETS = (
    ("carriage6 DOOR lag", "door", "carriage6", "lag", 1.0, 10.0, "FAB-SW-008; core §E.6"),
    ("carriage6 DOOR voltage", "door", "carriage6", "voltage", 1.0, 3.0, "FAB-SW-008; core §E.6"),
    ("carriage6 HEAVY voltage", "heavy", "carriage6", "voltage", 1.0, 5.0, "FAB-SW-008; core §E.6"),
    ("travel1d WALL voltage", "wall", "travel1d", "voltage", 10.0, 100.0, "lead decision D8 (was 1000×)"),
)
#: Isolation Monte Carlo: members, simulated seconds each, workers, wall budget (s).
MC_MEMBERS, MC_DURATION_S, MC_WORKERS, MC_BUDGET_S = 200, 60.0, 8, 60.0
#: Not-slow pytest tier: total and per-test wall budgets, s (FAB-SW-008; plan §2b item 13).
TIER_BUDGET_S, TEST_BUDGET_S = _support.TIER_TIME_LIMIT_S, _support.TEST_TIME_LIMIT_S


# ---------------------------------------------------------------------------- FAB-SW-003


def _check_anchor_doc(doc: dict) -> list[str]:
    from fabrail import export

    bad = []
    if doc.get("schema") != export.ANCHORS_SCHEMA:
        bad.append(f"schema {doc.get('schema')!r} != {export.ANCHORS_SCHEMA!r}")
    if doc.get("sha256") != export.payload_sha256(doc):
        bad.append("sha256 is not the payload hash: the file was edited by hand (re-record with the CLI)")
    anchors = doc.get("anchors") or {}
    if not anchors:
        bad.append("no anchors recorded")
    for name, spec in sorted(anchors.items()):
        values = spec.get("value")
        flat = values if isinstance(values, list) else [values]
        if not flat or not all(isinstance(v, int | float) and not isinstance(v, bool) and math.isfinite(v) for v in flat):
            bad.append(f"{name}: value {values!r} is not a finite number or list of numbers")
        if "rel" not in spec and "abs" not in spec:
            bad.append(f"{name}: no tolerance (rel/abs)")
        if not str(spec.get("source", "")).strip():
            bad.append(f"{name}: no source (core §E.5: re-record after touching RNG, plant or control; never invent)")
    return bad


def _fresh_anchors(quick: bool) -> tuple[dict[str, object], list[str]]:
    """Seed-42 anchors computed now by the CLI's producer: (name → value, names not measured)."""
    try:
        from fabrail.analysis import compute_anchors
    except ImportError as e:  # pragma: no cover - reported as a failure
        pytest.fail(f"fabrail.analysis.compute_anchors (B1, the `fabrail anchors` producer) is unavailable: {e}")
    values, _tols, missing = compute_anchors(quick=quick)
    return dict(values), list(missing)


def _compare_anchors(doc: dict, quick: bool) -> list[str]:
    """Failures of a fresh computation against the recorded document (empty = pass)."""
    from fabrail import export

    measured, missing = _fresh_anchors(quick)
    recorded = doc["anchors"]
    subset = {"schema": doc["schema"], "anchors": {k: v for k, v in recorded.items() if k in measured}}
    lines = export.check_anchors(subset, measured)
    lines += [f"{k}: recorded but not measured now (its phase did not occur)" for k in sorted(set(missing) & set(recorded))]
    lines += [f"{k}: computed now but not recorded (re-record with C1)" for k in sorted(set(measured) - set(recorded))]
    if not quick:
        gone = sorted(set(recorded) - set(measured) - set(missing))
        lines += [f"{k}: recorded but no longer computed by the producer" for k in gone]
    elif not subset["anchors"]:
        lines.append("no recorded anchor belongs to the quick set (the quick check would be vacuous)")
    return lines


def _anchors_or_fail_structure() -> dict:
    doc = _support.seed42_anchors_or_skip()
    bad = _check_anchor_doc(doc)
    assert not bad, "tests/data/anchors_seed42.json is malformed:\n" + "\n".join(bad)
    return doc


@pytest.mark.req("FAB-SW-003")
def test_seed42_anchors():
    """The recorded seed-42 anchors are well formed, and a fresh seed-42 computation of
    the quick set reproduces each within its recorded tolerance and source (FAB-SW-003;
    core §E.5). Skipped until C1 (Wave 3) records tests/data/anchors_seed42.json."""
    doc = _anchors_or_fail_structure()
    failures = _compare_anchors(doc, quick=True)
    assert not failures, (
        "seed-42 anchors (quick set) outside their recorded tolerance (rel/abs and source per anchor in the JSON; "
        "FAB-SW-003):\n" + "\n".join(failures)
    )


@pytest.mark.slow
def test_seed42_anchors_full():
    """Slow companion of ``seed42_anchors``: every recorded anchor, including the WALL
    P-wave timeline, the 1 h noise run and the n = 20 isolation ensemble, is reproduced
    within tolerance and nothing is recorded that the producer no longer computes."""
    doc = _anchors_or_fail_structure()
    failures = _compare_anchors(doc, quick=False)
    assert not failures, "seed-42 anchors (full set) outside tolerance (FAB-SW-003):\n" + "\n".join(failures)


def test_anchor_loader_contract(tmp_path, monkeypatch):
    """The loader returns None (and the fixture skips) without the file, and a document
    written by ``export.anchors_record`` passes the structure check, while a hand edit
    (stale ``sha256``) or a missing source is reported."""
    from fabrail import export

    monkeypatch.setattr(_support, "ANCHORS_SEED42", tmp_path / "absent.json")
    assert _support.load_seed42_anchors() is None
    with pytest.raises(pytest.skip.Exception):
        _support.seed42_anchors_or_skip()
    good = export.anchors_record(
        {"a": 1.0, "b": [1.0, 2.0]}, {"a": {"rel": 0.01, "source": "test"}, "b": {"abs": 0.1, "source": "t"}}
    )
    assert _check_anchor_doc(good) == []
    edited = json.loads(json.dumps(good))
    edited["anchors"]["a"]["value"] = 2.0
    assert any("sha256" in m for m in _check_anchor_doc(edited))
    nosrc = export.anchors_record({"a": 1.0}, {"a": 0.01})
    assert any("no source" in m for m in _check_anchor_doc(nosrc))


# ---------------------------------------------------------------------------- FAB-SW-008


def _perf_scenario(native, tier: str, coil: str, duration_s: float):
    spec = {
        "schema": "fabrail.scenario/1",
        "name": "perf",
        "duration_s": duration_s,
        "noise_seed": 1,
        "fidelity": {"model": tier, "coil": coil},
        "record": {"rate_hz": 1000},
    }
    return native.Scenario.from_json(json.dumps(spec))


def _rtf(native, preset: str, tier: str, coil: str, duration_s: float, reps: int = 3) -> list[float]:
    p = native.Params.preset(preset).with_overrides({"control.rates.coil_mode_voltage": coil == "voltage"})
    s = _perf_scenario(native, tier, coil, duration_s)
    out = []
    for _ in range(reps):
        t0 = time.perf_counter()
        native.run(p, s)
        out.append(duration_s / (time.perf_counter() - t0))
    return out


@pytest.mark.slow
@pytest.mark.req("FAB-SW-008")
def test_performance_budget(native, tmp_path):
    """Real-time factors, the isolation Monte Carlo and the not-slow pytest tier meet
    their budgets on a release build (FAB-SW-008; core §E.6; lead decision D8)."""
    from fabrail.config import RunConfig
    from fabrail.ensemble import run_ensemble

    assert native.BUILD_PROFILE == "release", f"_native is a {native.BUILD_PROFILE!r} build (budgets assume release)"
    report, misses = [], []
    for label, preset, tier, coil, dur, budget, src in RTF_BUDGETS:
        factors = _rtf(native, preset, tier, coil, dur)
        best = max(factors)
        report.append(
            f"{label}: {best:.1f}× real time (runs {', '.join(f'{f:.1f}' for f in factors)}; budget ≥ {budget:g}×, {src})"
        )
        if best < budget:
            misses.append(report[-1])

    cfg = RunConfig(
        rail_class="wall",
        seed=42,
        coil="voltage",
        duration_s=MC_DURATION_S,
        workers=MC_WORKERS,
        overrides={"control.seismic.isolation.enable": True},
    )
    from fabrail import scenarios

    probe = scenarios.build_spec("isolation_kt", cfg.params(), cfg)
    assert probe["duration_s"] == MC_DURATION_S and probe["fidelity"]["model"] == "travel1d", (
        f"isolation MC members must be {MC_DURATION_S:g} s on Travel1d (got {probe['duration_s']} s, {probe['fidelity']})"
    )
    t0 = time.perf_counter()
    ens = run_ensemble(cfg.params(), "isolation_kt", cfg, MC_MEMBERS)
    mc_wall = time.perf_counter() - t0
    report.append(
        f"isolation MC {MC_MEMBERS} × {MC_DURATION_S:g} s on {MC_WORKERS} workers (with synthesis): {mc_wall:.1f} s "
        f"(budget ≤ {MC_BUDGET_S:g} s, lead decision D8)"
    )
    assert len(ens.members) == MC_MEMBERS
    if mc_wall > MC_BUDGET_S:
        misses.append(report[-1])

    times = tmp_path / "tier_times.json"
    env = {
        **os.environ,
        "FABRAIL_TEST_TIMES": str(times),
        "FABRAIL_TIER_TIME_LIMIT_S": "0",  # measured here, not by the child's own guard
        "PYTHONPATH": str(_support.ROOT / "python"),
    }
    t0 = time.perf_counter()
    r = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "-m", "not slow", "-p", "no:cacheprovider", "tests"],
        capture_output=True,
        text=True,
        timeout=900,
        cwd=str(_support.ROOT),
        env=env,
    )
    tier_wall = time.perf_counter() - t0
    report.append(f"not-slow pytest tier: {tier_wall:.1f} s (budget ≤ {TIER_BUDGET_S:g} s; exit {r.returncode})")
    if tier_wall > TIER_BUDGET_S:
        misses.append(report[-1])
    if times.is_file():
        per = json.loads(times.read_text(encoding="utf-8"))["tests"]
        over = sorted(
            (float(v.get("call", 0.0)), k)
            for k, v in per.items()
            if not v.get("slow") and float(v.get("call", 0.0)) > TEST_BUDGET_S
        )
        for dur, nid in over:
            misses.append(f"not-slow test {nid}: {dur:.1f} s > {TEST_BUDGET_S:g} s")
        longest = max((float(v.get("call", 0.0)), k) for k, v in per.items()) if per else (0.0, "-")
        report.append(f"longest test call: {longest[0]:.1f} s ({longest[1]})")
    else:
        misses.append("the child session wrote no FABRAIL_TEST_TIMES file")
    print("\n".join(report))
    assert not misses, "(S) performance budgets missed (FAB-SW-008; core §E.6; D8):\n" + "\n".join(misses)
