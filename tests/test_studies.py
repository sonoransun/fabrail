"""Study runner and tier-1 study smoke tests (core §E test list: ``test_studies``).

Contracts checked here:

- ``tier1_smoke``: every tier-1 study runs with reduced settings (``quick=True``), returns a
  :class:`~fabrail.studies.runner.StudyResult` with a figure and a record, and
  :func:`~fabrail.studies.runner.run_study` writes ``<name>.png`` and a valid
  ``fabrail.study/1`` JSON document whose ``sha256`` matches its payload. Studies whose
  module declares ``QUICK_AWARE = True`` run in the non-slow tier (each ≤ 15 s, plan §2b
  item 13); any other tier-1 study is smoke-tested in the ``slow`` tier.
- ``json_records_numbers`` (FAB-SW-009 contract for studies): every number drawn in a
  study figure is present in its record, in the plotted unit, and survives the canonical
  JSON round trip (``export.missing_plotted``, rtol 1e-9).
- ``tier1_full_budget`` (slow): each tier-1 study at full settings finishes in < 30 s
  (core §C.6).

No SPECIFICATION token points into this file, so the tests carry no ``req`` marker
(CLAUDE.md traceability rule). The studies report physics findings in their records;
these tests check the study machinery, not the physics verdicts.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import pytest

pytest.importorskip("matplotlib")

from fabrail.config import RunConfig  # noqa: E402
from fabrail.export import STUDY_SCHEMA, missing_plotted, payload_sha256, to_jsonable  # noqa: E402
from fabrail.studies import runner  # noqa: E402

#: The control studies B2 owns (plan S4; core §C.6 as trimmed for Wave 2).
B2_STUDIES = (
    "clamp_force",
    "drag_ledger",
    "drawer_envelope",
    "fail_direction",
    "gain_margin",
    "haptic_zwidth",
    "pole_map",
    "speed_energy",
    "terminus_semilog",
    "zero_power_band",
)

QUICK_BUDGET_S = 15.0  # plan §2b item 13 (non-slow test limit)
FULL_BUDGET_S = 30.0  # core §C.6 (tier-1 study at full settings)
RTOL = 1e-9  # plotted value vs recorded number (float64 round trip is exact; rtol covers unit scaling)


def _tier1_params() -> list[Any]:
    """Tier-1 study names at collection time; non-quick-aware studies go to the slow tier."""
    try:
        specs = runner.discover()
    except Exception as exc:  # noqa: BLE001 - surface a broken study module as one failing case
        return [pytest.param(f"<discovery failed: {type(exc).__name__}: {exc}>", id="discovery")]
    out = []
    for name, spec in specs.items():
        if spec.tier != 1:
            continue
        marks = [] if runner.quick_aware(name) else [pytest.mark.slow]
        out.append(pytest.param(name, id=name, marks=marks))
    return out


_CACHE: dict[str, dict[str, Any]] = {}


def _quick_run(name: str, out_dir: Path) -> dict[str, Any]:
    """Run study ``name`` once with reduced settings; cache timing, plotted-number checks and the written paths."""
    if name.startswith("<discovery failed"):
        pytest.fail(name)
    if name in _CACHE:
        return _CACHE[name]
    cfg = RunConfig(output=out_dir, seed=42)
    spec = runner.discover()[name]
    t0 = time.perf_counter()
    result = runner.execute(spec, cfg, quick=True)
    wall = time.perf_counter() - t0
    assert isinstance(result, runner.StudyResult), f"{name}: run() must return a StudyResult (runner contract)"
    assert result.fig is not None, f"{name}: a tier-1 study makes a figure (core §C.6)"
    record = dict(result.record)
    missing_mem = missing_plotted(result.fig, record, rtol=RTOL)
    roundtrip = json.loads(json.dumps(to_jsonable(record), allow_nan=False))
    missing_json = missing_plotted(result.fig, roundtrip, rtol=RTOL)
    paths = runner.write_result(spec, result, cfg)
    _CACHE[name] = {
        "wall_s": wall,
        "missing_mem": missing_mem,
        "missing_json": missing_json,
        "paths": paths,
        "spec": spec,
        "record": record,
    }
    return _CACHE[name]


@pytest.fixture(scope="module")
def study_out(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("studies_out")


def test_discovery_lists_b2_studies() -> None:
    """Every B2 control study is discovered by pkgutil as a tier-1 ``StudySpec`` with a question, and reduced-settings aware."""
    specs = runner.discover()
    for name in B2_STUDIES:
        assert name in specs, f"{name}: not discovered in fabrail.studies (runner.discover, core §B.8)"
        spec = specs[name]
        assert spec.tier == 1, f"{name}: tier {spec.tier}, expected 1 (core §C.6: tier 1 < 30 s)"
        assert spec.question.strip().endswith("?"), f"{name}: the question must be a question (StudySpec contract)"
        assert runner.quick_aware(name), f"{name}: QUICK_AWARE must be True so the smoke test stays in the non-slow tier"
    tiers = [s.tier for s in specs.values()]
    assert tiers == sorted(tiers), "discover() orders studies by (tier, name) (runner contract)"


def test_unknown_study_raises(study_out: Path) -> None:
    """``run_study`` names the available studies when asked for an unknown one."""
    with pytest.raises(KeyError, match="unknown study"):
        runner.run_study("no_such_study", RunConfig(output=study_out))


def test_quick_mode_is_scoped() -> None:
    """``quick()`` is True only inside ``quick_mode`` and resets afterwards (reduced-settings contract)."""
    assert runner.quick() is False, "quick() defaults to False"
    with runner.quick_mode():
        assert runner.quick() is True, "quick() inside quick_mode()"
        with runner.quick_mode(False):
            assert runner.quick() is False, "nested quick_mode(False)"
        assert runner.quick() is True, "outer mode restored"
    assert runner.quick() is False, "quick() reset after the context"


@pytest.mark.parametrize("name", _tier1_params())
def test_tier1_smoke(name: str, study_out: Path) -> None:
    """A tier-1 study runs with reduced settings and writes a PNG plus a valid, sealed ``fabrail.study/1`` record."""
    got = _quick_run(name, study_out)
    if runner.quick_aware(name):
        assert got["wall_s"] <= QUICK_BUDGET_S, f"{name}: quick run {got['wall_s']:.1f} s > {QUICK_BUDGET_S:g} s (plan §2b item 13)"
    paths = {p.suffix: p for p in got["paths"]}
    assert ".png" in paths and paths[".png"].stat().st_size > 10_000, f"{name}: no PNG written (run_study contract; cfg.plots = True)"
    assert paths[".png"].name == f"{name}.png", f"{name}: PNG path {paths['.png']} (output/studies/<name>.png)"
    doc = json.loads(paths[".json"].read_text(encoding="utf-8"))
    assert doc["schema"] == STUDY_SCHEMA, f"{name}: schema {doc['schema']!r} != {STUDY_SCHEMA!r} (export contract)"
    assert doc["name"] == name and doc["tier"] == got["spec"].tier, f"{name}: name/tier in the record differ from the StudySpec"
    assert doc["question"] == got["spec"].question, f"{name}: question differs from the StudySpec"
    assert doc["sha256"] == payload_sha256(doc), f"{name}: sha256 does not match the canonical payload (export.payload_sha256)"
    rec = doc["record"]
    assert isinstance(rec, dict) and rec, f"{name}: empty record"
    if runner.quick_aware(name):
        assert rec.get("quick") is True, f"{name}: a quick-aware study records quick = true in reduced mode (runner contract)"
    for key in ("caption", "title"):
        if name in B2_STUDIES:
            assert isinstance(rec.get(key), str) and rec[key].strip(), f"{name}: record lacks its {key} (honest caption, B2 contract)"
    if name in B2_STUDIES:
        banned = ("perfect", "absurdly", "guarantee", "earthquake-proof", "no damage", "protects")
        text = (rec["caption"] + " " + rec["title"]).lower()
        hits = [w for w in banned if w in text]
        assert not hits, f"{name}: caption uses banned vocabulary {hits} (plan §7 / FAB-EXO-005 vocabulary guard)"


@pytest.mark.parametrize("name", _tier1_params())
def test_json_records_numbers(name: str, study_out: Path) -> None:
    """Every number a study plots is in its record, in the plotted unit, before and after the JSON round trip (rtol 1e-9)."""
    got = _quick_run(name, study_out)
    assert not got["missing_mem"], f"{name}: plotted values missing from the record (FAB-SW-009, rtol {RTOL:g}): {got['missing_mem'][:5]}"
    assert not got["missing_json"], f"{name}: plotted values lost in the JSON round trip (rtol {RTOL:g}): {got['missing_json'][:5]}"


@pytest.mark.slow
@pytest.mark.parametrize("name", _tier1_params())
def test_tier1_full_budget(name: str, tmp_path: Path) -> None:
    """A tier-1 study at full settings finishes in under 30 s and leaves every plotted number in its record (core §C.6)."""
    if name.startswith("<discovery failed"):
        pytest.fail(name)
    cfg = RunConfig(output=tmp_path, seed=42)
    t0 = time.perf_counter()
    result = runner.execute(name, cfg)
    wall = time.perf_counter() - t0
    missing = missing_plotted(result.fig, dict(result.record), rtol=RTOL)
    runner.write_result(runner.discover()[name], result, cfg)
    assert wall < FULL_BUDGET_S, f"{name}: full run {wall:.1f} s ≥ {FULL_BUDGET_S:g} s (core §C.6 tier-1 budget)"
    assert not missing, f"{name}: plotted values missing from the full record (FAB-SW-009): {missing[:5]}"
