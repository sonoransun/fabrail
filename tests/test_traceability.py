"""Requirement ⇄ test traceability gate (FAB-SW-007; SPECIFICATION §6; core design §E.4).

The guard itself is ``fabrail.trace`` (stdlib only); these tests turn its report into
the Wave-2 integration gate (plan §2b item 16):

- ``ids_well_formed``: every requirement row of SPECIFICATION, SPECIFICATION_EXAMPLES,
  EXOTIC and FRONTIER is well formed (7 cells, known methods and status, tokens only in
  Verified-by, the status/method rules), IDs are unique, in their document's areas,
  ascending, with the areas in SPECIFICATION order, and every token is grammatical.
- ``bidirectional``: per ID, the listed tokens equal the marked tests in both
  directions. A listed token without a marked test is a failure (``no_file``,
  ``no_test`` or ``unmarked``), as is a marker the row does not list or a marker naming
  an unknown ID. While Wave-2 test files are still being written in parallel this test
  fails by listing them; that is its job (W2.5 closes them). Requirements whose tests
  are all slow are reported, never failed.
- ``cross_doc_refs``: every FAB ID cited anywhere else (docs, README, code, tests,
  scripts, examples) exists; every hazard H-01 to H-21 in SAFETY.md maps to a
  requirement; every ``[R#]`` tag resolves in references.md.
- Unmarked companions: ``counts`` (the docs TRACE block and the README TRACE block equal
  the parsed counts, the counts are internally consistent, and ``python -m fabrail
  trace --json`` reports the same numbers without the native extension) and
  ``guard_detects_seeded_defects`` (a synthetic repository with one defect of each kind
  is caught; IDs there are assembled at run time so this file cites none).
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import textwrap
from collections import defaultdict

import pytest

import _support
from fabrail import trace

ROOT = _support.ROOT

STRUCTURE_CHECKS = frozenset(
    {
        "missing_doc",
        "row_format",
        "unknown_area",
        "wrong_document",
        "area_order",
        "order",
        "duplicate_id",
        "bad_token",
        "status_rule",
        "retired_row",
    }
)
MARKER_CHECKS = frozenset({"unlisted_marker", "dangling_marker", "unattached_marker", "marker_syntax", "syntax", "unreadable"})
CITATION_CHECKS = frozenset({"dangling_citation", "hazard", "reference"})


@pytest.fixture(scope="module")
def report() -> trace.TraceReport:
    return trace.check(ROOT)


def _fmt(issues) -> str:
    return "\n".join(f"  {i}" for i in issues)


@pytest.mark.req("FAB-SW-007")
def test_ids_well_formed(report):
    """Requirement rows are well formed and IDs unique, ordered and in their document's
    areas across the four requirement documents (FAB-SW-007; SPECIFICATION §0, §6)."""
    bad = [e for e in report.errors if e.check in STRUCTURE_CHECKS]
    assert not bad, f"{len(bad)} requirement-table problem(s) (FAB-SW-007):\n{_fmt(bad)}"
    ids = [r.id for r in report.requirements]
    assert ids, "no requirement rows parsed"
    assert len(ids) == len(set(ids)), "duplicate IDs"
    assert all(trace.ID_FULL_RE.match(i) for i in ids), [i for i in ids if not trace.ID_FULL_RE.match(i)]
    for doc, n in report.by_doc.items():
        assert n > 0, f"docs/{doc}.md defines no requirement"
    for r in report.requirements:
        assert r.area in trace.DOC_AREAS[r.doc], f"{r.id} in {r.doc}"
        for t in r.tokens:
            files, fn, problem = trace.token_path(ROOT, t)
            assert problem is None and files and fn, f"{r.id}: token {t!r} does not resolve ({problem})"


@pytest.mark.req("FAB-SW-007")
def test_bidirectional(report):
    """For every ID the listed test tokens equal the marked tests in both directions:
    no listed test is missing or unmarked, no marked test is unlisted, no marker names an
    unknown ID or floats free of a test (FAB-SW-007). Slow-only requirements are
    reported, not failed."""
    marker_errors = [e for e in report.errors if e.check in MARKER_CHECKS]
    groups: dict[str, list[str]] = defaultdict(list)
    for m in report.missing:
        groups[m.path].append(f"{m.token.rsplit('::', 1)[-1]} ({m.id}, {m.reason})")
    lines = []
    if marker_errors:
        lines.append(f"{len(marker_errors)} marker error(s):\n{_fmt(marker_errors)}")
    if report.missing:
        by_reason = report.missing_by_reason()
        lines.append(
            f"{len(report.missing)} listed test(s) without a marked test "
            f"(no file {by_reason['no_file']}, no test {by_reason['no_test']}, unmarked {by_reason['unmarked']}):"
        )
        for path in sorted(groups):
            lines.append(f"  {path} ({len(groups[path])}): {', '.join(sorted(groups[path]))}")
    if report.slow_only:
        print(f"slow-only requirements (reported, not failed): {', '.join(report.slow_only)}")
    assert not lines, "traceability is not bidirectional (FAB-SW-007):\n" + "\n".join(lines)


@pytest.mark.req("FAB-SW-007")
def test_cross_doc_refs(report):
    """Every FAB ID cited outside the requirement tables exists, every hazard H-01 to
    H-21 maps to a requirement, and every [R#] tag resolves (FAB-SW-007)."""
    for doc in ("SAFETY.md", "references.md"):
        assert (ROOT / "docs" / doc).is_file(), f"docs/{doc} is missing, so its cross-references cannot be checked"
    bad = [e for e in report.errors if e.check in CITATION_CHECKS]
    assert not bad, f"{len(bad)} dangling cross-reference(s) (FAB-SW-007):\n{_fmt(bad)}"


def test_counts(report):
    """The TRACE data block and the README TRACE block (once generated) equal the parsed
    counts; the counts are consistent; the CLI reports the same numbers and runs without
    the native extension (stdlib only)."""
    bad = [e for e in report.errors if e.check == "counts"]
    assert not bad, f"stale counts (regenerate with scripts/update_docs_run.py / README):\n{_fmt(bad)}"
    assert report.total == sum(report.by_doc.values()) == sum(report.by_area.values()) == sum(report.by_status.values()), (
        report.by_doc,
        report.by_area,
        report.by_status,
    )
    n_tokens = len({trace.canonical_token(t) for r in report.requirements for t in r.tokens})
    assert report.tokens["py"] + report.tokens["rs"] == n_tokens
    env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
    code = (
        "import sys, builtins; real = builtins.__import__\n"
        "def guard(name, *a, **k):\n"
        "    if name.startswith('fabrail._native') or name in ('numpy', '_native'): raise ImportError('blocked: ' + name)\n"
        "    return real(name, *a, **k)\n"
        "builtins.__import__ = guard\n"
        f"sys.path.insert(0, {str(ROOT / 'python')!r})\n"
        "from fabrail.trace import main\n"
        "sys.exit(main(['--json', '--allow-missing', '--root', " + repr(str(ROOT)) + "]))\n"
    )
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=120, env=env)
    assert r.returncode in (0, 1), f"trace CLI crashed without _native/numpy:\n{r.stderr[-2000:]}"
    cli = json.loads(r.stdout)
    fresh = report.to_dict(full=False)
    for key in ("total", "by_doc", "by_area", "by_status", "tokens", "marked", "n_errors", "n_missing"):
        assert cli[key] == fresh[key], f"trace --json {key} = {cli[key]!r}, in-process {fresh[key]!r}"
    assert cli["schema"] == trace.SCHEMA


# ---------------------------------------------------------------------------- guard self-test

_TABLE_HEAD = "| ID | Shall | Class | Source | V | Verified by | St. |\n|---|---|---|---|---|---|---|\n"


def _rid(area: str, n: int) -> str:
    """A requirement ID assembled at run time (so this file cites no ID)."""
    return "-".join(("FAB", area, f"{n:03d}"))


def _write_repo(tmp_path, spec_rows: list[str], test_src: str, extra_doc: str = "") -> None:
    docs = tmp_path / "docs"
    docs.mkdir()
    (docs / "SPECIFICATION.md").write_text("# Spec\n\n" + _TABLE_HEAD + "\n".join(spec_rows) + "\n" + extra_doc, encoding="utf-8")
    for d in ("SPECIFICATION_EXAMPLES", "EXOTIC", "FRONTIER"):
        (docs / f"{d}.md").write_text(f"# {d}\n", encoding="utf-8")
    tests = tmp_path / "tests"
    tests.mkdir()
    (tests / "test_demo.py").write_text(test_src, encoding="utf-8")


def test_guard_detects_seeded_defects(tmp_path):
    """A synthetic repository with one defect of each kind: a listed test that does not
    exist, a marked test its row does not list, a marker naming an undefined ID, a
    duplicate ID, a descending ID, an M row with a token, and a dangling citation — the
    guard reports each; the clean variant passes."""
    a1, a2, a3, a9 = _rid("LEV", 1), _rid("LEV", 2), _rid("LEV", 3), _rid("LEV", 9)
    clean_rows = [
        f"| {a1} | ok | all | src | T | `py:demo::one` | V |",
        f"| {a2} | ok | all | src | D | — | M |",
    ]
    clean_src = textwrap.dedent(
        f"""
        import pytest

        @pytest.mark.req("{a1}")
        def test_one():
            pass
        """
    )
    clean = tmp_path / "clean"
    clean.mkdir()
    _write_repo(clean, clean_rows, clean_src)
    rep = trace.check(clean)
    assert rep.ok, [str(e) for e in rep.errors] + [m.token for m in rep.missing]

    broken = tmp_path / "broken"
    broken.mkdir()
    rows = [
        f"| {a1} | ok | all | src | T | `py:demo::one`; `py:demo::absent` | V |",
        f"| {a3} | ok | all | src | T | `py:demo::three` | V |",
        f"| {a2} | ok | all | src | D | `py:demo::one` | M |",
        f"| {a3} | dup | all | src | T | `py:demo::three` | V |",
    ]
    src = textwrap.dedent(
        f"""
        import pytest

        @pytest.mark.req("{a1}")
        def test_one():
            pass

        @pytest.mark.req("{a3}", "{a1}")
        def test_three():
            pass

        @pytest.mark.req("{a9}")
        def test_orphan():
            pass
        """
    )
    _write_repo(broken, rows, src, extra_doc=f"\nSee {_rid('PRP', 42)} for details.\n")
    rep = trace.check(broken)
    kinds = {e.check for e in rep.errors}
    for kind in ("duplicate_id", "order", "status_rule", "dangling_marker", "unlisted_marker", "dangling_citation"):
        assert kind in kinds, f"guard missed a seeded {kind}: {sorted(kinds)}"
    assert any(m.token == "py:demo::absent" and m.reason == "no_test" for m in rep.missing), rep.missing
    assert not rep.ok and not rep.structure_ok
