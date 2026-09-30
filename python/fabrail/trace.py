"""Requirement ⇄ test traceability guard (SPECIFICATION §6; core design §E.4; FAB-SW-007).

Stdlib only: it never imports ``fabrail._native`` or numpy, so it runs on a fresh
checkout (``python -m fabrail trace``, ``python -m fabrail.trace``).

**Documents.** docs/SPECIFICATION.md, SPECIFICATION_EXAMPLES.md, EXOTIC.md and
FRONTIER.md. A requirement row is a line matching ``^\\|\\s*(FAB-[A-Z]{2,3}-\\d{3})\\s*\\|``
with exactly seven cells ``ID | Shall | Class | Source | V | Verified by | St.``
(SPECIFICATION §0.3). Methods come from the V cell (A, S, T, I, D), tokens from the
Verified-by cell (``(?:py|rs):[\\w./-]+::\\w+``), the status from St. (V, V(I), S, R, M).

**Test tokens** (SPECIFICATION §0.7):

- ``py:<stem>::<fn>`` → ``tests/test_<stem>.py::test_<fn>``;
  ``py:<dir>/<stem>::<fn>`` → ``tests/<dir>/test_<stem>.py::test_<fn>``.
- ``rs:<crate>/<path>::<fn>`` → ``#[test] fn <fn>`` in ``crates/fabrail-<crate>/src/<path>.rs``
  (``<path>/mod.rs`` is written ``<path>``); ``rs:<crate>/tests/<file>::<fn>`` →
  ``crates/fabrail-<crate>/tests/<file>.rs``. The pseudo-crate ``fw`` is the firmware
  workspace: ``rs:fw/<path>::<fn>`` → ``firmware/src/<path>.rs``.

**Markers.** Python tests carry ``@pytest.mark.req("FAB-…", …)`` (on the function, its
class, or a module-level ``pytestmark``), found by walking the test ASTs. Rust tests
carry a ``// req: FAB-…`` line at most five lines above ``#[test]`` or ``proptest!``;
it attaches to the next ``fn``. A marker that attaches to no test is an error.
``// impl: FAB-…`` comments are only checked for dangling IDs.

**Checks.**

- IDs are well-formed, unique across the four documents, in their document's areas,
  ascending within an area; SPECIFICATION's areas appear in the order LEV, PRP, HAP,
  SAF, SEI, PWR, ENV, FW, SW.
- Rows have seven cells, valid methods and status, and nothing but tokens in
  Verified-by (or ``—``).
- V, V(I), S and R rows have at least one token; M rows have none and use D or I.
  A row with method I is V(I) or M, and a V(I) row uses method I. Retired rows
  (struck-through Shall) are M with method I and no token.
- Bidirectional: per ID, the tokens in the documents equal the marked tests. A
  listed token without a marked test is *missing* (``no_file``, ``no_test`` or
  ``unmarked``); a marked test the row does not list is an error, as is a marker
  naming an unknown ID.
- Every FAB ID cited anywhere else (docs, README, code, tests, scripts; ``A/B/C``
  lists and ``X to NNN`` ranges included) exists. Put ``trace: ignore`` on a line to
  exempt it (for deliberate negative examples).
- When docs/SAFETY.md exists, hazards H-01 to H-21 are all present and each maps to a
  requirement. When docs/references.md exists, every ``[R#]`` tag resolves.
- The counts in docs/assets/data/trace.js (unless ``placeholder``) and in a README
  ``<!-- TRACE:BEGIN -->…<!-- TRACE:END -->`` block equal the parsed counts.
- Requirements whose marked tests are all slow (``pytest.mark.slow`` or Rust
  ``#[ignore]``) are reported in ``slow_only``, never failed.

``TraceReport.ok`` is true when there are no errors and no missing tests;
``structure_ok`` ignores missing tests (Wave-0/1 bring-up, ``--allow-missing``).

Owner: W0-C2 (complete). Gated by tests/test_traceability.py (B6).
"""

from __future__ import annotations

import argparse
import ast
import json
import re
import sys
from collections import defaultdict
from collections.abc import Iterable, Iterator, Sequence
from dataclasses import dataclass, field
from pathlib import Path

__all__ = [
    "AREAS",
    "DOC_AREAS",
    "Issue",
    "Marker",
    "MissingTest",
    "Requirement",
    "SPEC_DOCS",
    "TestLoc",
    "TraceReport",
    "add_arguments",
    "canonical_token",
    "check",
    "cli",
    "collect_py_markers",
    "collect_py_tests",
    "collect_rs_markers",
    "collect_rs_tests",
    "find_root",
    "main",
    "parse_spec",
    "token_path",
]

SCHEMA = "fabrail.trace/1"

#: The four documents the guard parses, in order.
SPEC_DOCS: tuple[str, ...] = ("SPECIFICATION", "SPECIFICATION_EXAMPLES", "EXOTIC", "FRONTIER")

#: Areas each document may define, in table order (SPECIFICATION §0.2).
DOC_AREAS: dict[str, tuple[str, ...]] = {
    "SPECIFICATION": ("LEV", "PRP", "HAP", "SAF", "SEI", "PWR", "ENV", "FW", "SW"),
    "SPECIFICATION_EXAMPLES": ("EXA",),
    "EXOTIC": ("EXO",),
    "FRONTIER": ("FRN",),
}
AREAS: tuple[str, ...] = tuple(a for d in SPEC_DOCS for a in DOC_AREAS[d])
METHODS: tuple[str, ...] = ("A", "S", "T", "I", "D")
STATUSES: tuple[str, ...] = ("V", "V(I)", "S", "R", "M")
N_CELLS = 7
#: Hazards SAFETY.md must list, each mapped to at least one requirement.
HAZARDS: tuple[str, ...] = tuple(f"H-{i:02d}" for i in range(1, 22))
#: A ``// req:`` line attaches to a ``#[test]``/``proptest!`` at most this many lines below.
MARKER_WINDOW = 5

ROW_RE = re.compile(r"^\|\s*(FAB-[A-Z]{2,3}-\d{3})\s*\|")
LOOSE_ROW_RE = re.compile(r"^\|\s*(?:~~)?\s*FAB-")
TOKEN_RE = re.compile(r"(?:py|rs):[\w./-]+::\w+")
ID_FULL_RE = re.compile(r"^FAB-([A-Z]{2,3})-(\d{3})$")
ID_RE = re.compile(r"\bFAB-([A-Z]{2,3})-(\d{3})\b")
# A bare continuation number followed by a unit ("FAB-SAF-005 to 150 N") is a quantity, not an ID.
_UNIT_AFTER = (
    r"(?![.,]\d)(?!\s*(?:(?:N|kN|ms|s|kg|g|t|m|mm|µm|um|cm|km|Hz|kHz|A|mA|V|W|kW|J|kJ|Pa|kPa|K|rad|mg|gal)\b|[%°]))"
)
CONT_RE = re.compile(
    r"\bFAB-([A-Z]{2,3})-(\d{3})\b("
    r"(?:\s*(?:/|…|\.\.\.?|–|—)\s*\d{3}\b" + _UNIT_AFTER + r")+"
    r"|\s+(?:to|through)\s+\d{3}\b" + _UNIT_AFTER + r")"
)
BARE_NUM_RE = re.compile(r"\d{3}")
HAZARD_ROW_RE = re.compile(r"^\|\s*(H-\d{2})\s*\|")
HAZARD_RE = re.compile(r"\bH-(\d{2})\b")
RTAG_RE = re.compile(r"\[(R\d+(?:\s*[,;]\s*R\d+)*)\]")
RDEF_ROW_RE = re.compile(r"^\|\s*\[?(R\d+)\]?\s*\|")

RS_REQ_RE = re.compile(r"^\s*//+!?\s*req:\s*(.*)$")
RS_TEST_RE = re.compile(r"^\s*#\[test\]")
RS_PROPTEST_RE = re.compile(r"\bproptest!\s*[{(\[]?")
RS_FN_RE = re.compile(r"\bfn\s+([A-Za-z_]\w*)\s*[(<]")
RS_IGNORE_RE = re.compile(r"#\[ignore\b")

_SKIP_DIRS = frozenset(
    {".git", ".hypothesis", ".pytest_cache", ".venv", "__pycache__", "node_modules", "out", "output", "target", "venv"}
)
_CITE_EXTS = frozenset({".css", ".html", ".js", ".json", ".md", ".py", ".pyi", ".rs", ".sh", ".toml", ".txt"})
_CITE_ROOTS: tuple[str, ...] = ("docs", "python", "tests", "crates", "firmware", "scripts", "examples")
_CITE_FILES: tuple[str, ...] = ("README.md", "CLAUDE.md", "run_analysis.py")
_IGNORE_PRAGMA = "trace: ignore"


# --------------------------------------------------------------------------- records


@dataclass(frozen=True)
class Requirement:
    """One parsed requirement row."""

    id: str
    doc: str
    path: str
    line: int
    area: str
    number: int
    shall: str
    cls: str
    source: str
    methods: tuple[str, ...]
    tokens: tuple[str, ...]
    status: str
    retired: bool
    problems: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.id,
            "doc": self.doc,
            "line": self.line,
            "area": self.area,
            "class": self.cls,
            "source": self.source,
            "methods": list(self.methods),
            "tokens": list(self.tokens),
            "status": self.status,
            "retired": self.retired,
        }


@dataclass(frozen=True)
class Marker:
    """A ``req`` marker attached to one test (``kind`` is "py" or "rs")."""

    ids: tuple[str, ...]
    token: str
    path: str
    line: int
    kind: str
    slow: bool = False

    def to_dict(self) -> dict[str, object]:
        return {"ids": list(self.ids), "token": self.token, "path": self.path, "line": self.line, "slow": self.slow}


@dataclass(frozen=True)
class TestLoc:
    """Where a test function lives (marked or not)."""

    __test__ = False  # not a pytest class

    token: str
    path: str
    line: int
    slow: bool = False


@dataclass(frozen=True)
class Issue:
    """A failed check. ``where`` is ``path:line`` relative to the root (may be empty)."""

    check: str
    message: str
    where: str = ""

    def to_dict(self) -> dict[str, str]:
        return {"check": self.check, "where": self.where, "message": self.message}

    def __str__(self) -> str:
        loc = f"{self.where}: " if self.where else ""
        return f"[{self.check}] {loc}{self.message}"


@dataclass(frozen=True)
class MissingTest:
    """A token the documents list for ``id`` without a marked test.

    ``reason``: ``no_file`` (the test file does not exist), ``no_test`` (the file
    exists, the test function does not) or ``unmarked`` (the test exists but does
    not carry this ID).
    """

    id: str
    token: str
    reason: str
    path: str

    def to_dict(self) -> dict[str, str]:
        return {"id": self.id, "token": self.token, "reason": self.reason, "path": self.path}


@dataclass
class _Scan:
    tests: dict[str, TestLoc] = field(default_factory=dict)
    markers: list[Marker] = field(default_factory=list)
    errors: list[Issue] = field(default_factory=list)


@dataclass
class TraceReport:
    """The result of :func:`check`."""

    root: str
    requirements: list[Requirement] = field(default_factory=list)
    markers: list[Marker] = field(default_factory=list)
    tests: dict[str, TestLoc] = field(default_factory=dict)
    errors: list[Issue] = field(default_factory=list)
    missing: list[MissingTest] = field(default_factory=list)
    slow_only: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    # -- verdicts ---------------------------------------------------------------
    @property
    def structure_ok(self) -> bool:
        """No structural, marker or cross-reference errors (missing tests ignored)."""
        return not self.errors

    @property
    def ok(self) -> bool:
        """The full bidirectional guard passes."""
        return not self.errors and not self.missing

    # -- counts (the docs TRACE block, schema fabrail.docs.trace/1) ------------
    def _valid(self) -> Iterator[Requirement]:
        return _unique(r for r in self.requirements if r.area != "?")

    @property
    def ids(self) -> list[str]:
        return [r.id for r in self._valid()]

    @property
    def total(self) -> int:
        return sum(1 for _ in self._valid())

    @property
    def by_doc(self) -> dict[str, int]:
        out = dict.fromkeys(SPEC_DOCS, 0)
        for r in self._valid():
            out[r.doc] = out.get(r.doc, 0) + 1
        return out

    @property
    def by_area(self) -> dict[str, int]:
        out = dict.fromkeys(AREAS, 0)
        for r in self._valid():
            if r.area in out:
                out[r.area] += 1
        return out

    @property
    def by_status(self) -> dict[str, int]:
        out = dict.fromkeys(STATUSES, 0)
        for r in self._valid():
            if r.status in out:
                out[r.status] += 1
        return out

    @property
    def tokens(self) -> dict[str, int]:
        """Distinct canonical tokens listed in the documents, by kind."""
        seen = {canonical_token(t) for r in self._valid() for t in r.tokens}
        return {"py": sum(t.startswith("py:") for t in seen), "rs": sum(t.startswith("rs:") for t in seen)}

    @property
    def marked(self) -> dict[str, int]:
        """Distinct marked tests found, by kind."""
        seen = {m.token for m in self.markers}
        return {"py": sum(t.startswith("py:") for t in seen), "rs": sum(t.startswith("rs:") for t in seen)}

    def missing_by_reason(self) -> dict[str, int]:
        out = {"no_file": 0, "no_test": 0, "unmarked": 0}
        for m in self.missing:
            out[m.reason] = out.get(m.reason, 0) + 1
        return out

    # -- output -------------------------------------------------------------------
    def to_dict(self, full: bool = True) -> dict[str, object]:
        """JSON-ready dict. The top-level count keys match ``fabrail.docs.trace/1``."""
        out: dict[str, object] = {
            "schema": SCHEMA,
            "fabrail_version": _version(),
            "ok": self.ok,
            "structure_ok": self.structure_ok,
            "total": self.total,
            "by_doc": self.by_doc,
            "by_area": self.by_area,
            "by_status": self.by_status,
            "tokens": self.tokens,
            "marked": self.marked,
            "slow_only": len(self.slow_only),
            "n_errors": len(self.errors),
            "n_missing": len(self.missing),
            "missing_by_reason": self.missing_by_reason(),
            "errors": [e.to_dict() for e in self.errors],
            "missing": [m.to_dict() for m in self.missing],
            "slow_only_ids": list(self.slow_only),
            "notes": list(self.notes),
        }
        if full:
            out["requirements"] = [r.to_dict() for r in self.requirements]
            out["markers"] = [m.to_dict() for m in self.markers]
        return out

    def to_json(self, indent: int | None = 2, full: bool = True) -> str:
        return json.dumps(self.to_dict(full=full), indent=indent, ensure_ascii=False)

    def summary(self, verbose: bool = False) -> str:
        """Human-readable report."""
        docs = ", ".join(f"{d} {n}" for d, n in self.by_doc.items())
        status = " · ".join(f"{s} {n}" for s, n in self.by_status.items())
        tok = self.tokens
        mk = self.marked
        lines = [
            f"fabrail trace: {self.total} requirements ({docs})",
            f"  status:  {status}",
            f"  tokens:  {tok['py'] + tok['rs']} distinct (py {tok['py']}, rs {tok['rs']}); "
            f"marked tests found: py {mk['py']}, rs {mk['rs']}",
            f"  structure: {'OK' if self.structure_ok else f'{len(self.errors)} error(s)'}",
        ]
        mr = self.missing_by_reason()
        lines.append(
            f"  missing tests: {len(self.missing)} "
            f"(no file {mr['no_file']}, no test {mr['no_test']}, unmarked {mr['unmarked']})"
        )
        if self.slow_only:
            lines.append(f"  slow-only requirements (reported, not failed): {', '.join(self.slow_only)}")
        for note in self.notes:
            lines.append(f"  note: {note}")
        if self.errors:
            lines.append("errors:")
            lines += [f"  {e}" for e in self.errors]
        if self.missing:
            lines.append("missing tests:")
            if verbose:
                lines += [f"  {m.id}  {m.token}  [{m.reason}]  {m.path}" for m in self.missing]
            else:
                groups: dict[str, list[MissingTest]] = defaultdict(list)
                for m in self.missing:
                    groups[m.path].append(m)
                for path in sorted(groups):
                    items = groups[path]
                    names = sorted({m.token.rsplit("::", 1)[-1] for m in items})
                    shown = ", ".join(names[:8]) + (", …" if len(names) > 8 else "")
                    lines.append(f"  {path} ({len(items)}): {shown}")
                lines.append("  (use --verbose or --json for every item)")
        verdict = "PASS" if self.ok else ("PASS (structure only; tests missing)" if self.structure_ok else "FAIL")
        lines.append(f"RESULT: {verdict}")
        return "\n".join(lines)


def _unique(reqs: Iterable[Requirement]) -> Iterator[Requirement]:
    seen: set[str] = set()
    for r in reqs:
        if r.id not in seen:
            seen.add(r.id)
            yield r


def _version() -> str:
    try:
        from . import __version__
    except ImportError:  # run as a plain script
        return "unknown"
    return __version__


# --------------------------------------------------------------------------- helpers


def find_root(start: str | Path | None = None) -> Path:
    """The repository root: the nearest ancestor holding docs/SPECIFICATION.md.

    Searches upward from ``start`` (default: the current directory), then from this
    file's location. Falls back to ``start`` itself.
    """
    candidates: list[Path] = []
    if start is not None:
        candidates.append(Path(start).resolve())
    else:
        candidates.append(Path.cwd().resolve())
        candidates.append(Path(__file__).resolve().parent)
    for base in candidates:
        for d in (base, *base.parents):
            if (d / "docs" / "SPECIFICATION.md").is_file():
                return d
    return candidates[0]


def canonical_token(token: str) -> str:
    """Canonical form of a test token: a Rust ``…/mod`` path is written without ``/mod``."""
    head, sep, fn = token.partition("::")
    if head.startswith("rs:") and head.endswith("/mod"):
        head = head[: -len("/mod")]
    return f"{head}{sep}{fn}"


def token_path(root: Path, token: str) -> tuple[list[Path], str, str | None]:
    """Resolve ``token`` to candidate files, the test function name and a grammar problem.

    Returns ``(files, function, problem)``; ``problem`` is None for a well-formed token.
    """
    head, _, fn = canonical_token(token).partition("::")
    if head.startswith("py:"):
        rel = head[3:]
        parts = [p for p in rel.split("/") if p]
        if not parts or any(p in (".", "..") for p in parts):
            return [], "", f"malformed py token {token!r}"
        path = root / "tests" / Path(*parts[:-1]) / f"test_{parts[-1]}.py"
        return [path], f"test_{fn}", None
    if head.startswith("rs:"):
        rel = head[3:]
        crate, _, sub = rel.partition("/")
        if not crate or not sub:
            return [], fn, f"rs token needs rs:<crate>/<path>::<fn> (got {token!r})"
        crate_dir = root / "firmware" if crate == "fw" else root / "crates" / f"fabrail-{crate}"
        if not crate_dir.is_dir():
            return [], fn, f"unknown crate {crate!r} in {token!r} (no {crate_dir.relative_to(root).as_posix()}/)"
        if sub.startswith("tests/"):
            base = crate_dir / "tests" / sub[len("tests/") :]
        else:
            base = crate_dir / "src" / sub
        return [base.with_name(base.name + ".rs"), base / "mod.rs"], fn, None
    return [], "", f"unknown token kind {token!r}"


def _rel(root: Path, path: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def _read(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError):
        return None


def _skip(rel_parts: Sequence[str]) -> bool:
    return any(p in _SKIP_DIRS for p in rel_parts)


# --------------------------------------------------------------------------- spec parsing


def _cells(line: str) -> list[str]:
    body = line.strip()
    if body.startswith("|"):
        body = body[1:]
    if body.endswith("|"):
        body = body[:-1]
    return [c.strip() for c in body.split("|")]


def parse_spec(path: str | Path, doc: str | None = None, root: str | Path | None = None) -> list[Requirement]:
    """Parse the requirement rows of one document.

    Rows with problems (wrong cell count, bad method or status, text outside tokens
    in Verified-by) are still returned, with ``problems`` filled in; :func:`check`
    reports them. A line that starts like a row but has a malformed ID is returned
    as a Requirement with area ``"?"`` and a problem.
    """
    path = Path(path)
    doc = doc or path.stem
    root_p = Path(root) if root is not None else path.parent.parent
    rel = _rel(root_p, path)
    text = _read(path)
    if text is None:
        return []
    out: list[Requirement] = []
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = raw.rstrip("\n")
        m = ROW_RE.match(line)
        if not m:
            if LOOSE_ROW_RE.match(line):
                first = _cells(line)[0] if _cells(line) else line
                out.append(
                    Requirement(
                        id=first, doc=doc, path=rel, line=lineno, area="?", number=-1, shall="", cls="",
                        source="", methods=(), tokens=(), status="", retired=False,
                        problems=(f"malformed requirement ID {first!r} (expected FAB-<AREA>-<NNN>)",),
                    )
                )
            continue
        rid = m.group(1)
        idm = ID_FULL_RE.match(rid)
        area, number = (idm.group(1), int(idm.group(2))) if idm else ("?", -1)
        cells = _cells(line)
        problems: list[str] = []
        if len(cells) != N_CELLS:
            problems.append(f"{len(cells)} cells (expected {N_CELLS}: ID | Shall | Class | Source | V | Verified by | St.)")
        cells = (cells + [""] * N_CELLS)[:N_CELLS] if len(cells) < N_CELLS else cells
        shall, cls, source, v_cell, by_cell, status = cells[1], cells[2], cells[3], cells[4], cells[5], cells[-1]
        methods = tuple(x.strip() for x in v_cell.split(",") if x.strip())
        if not methods:
            problems.append("empty V (method) cell")
        bad = [x for x in methods if x not in METHODS]
        if bad:
            problems.append(f"unknown method(s) {', '.join(bad)} (allowed: {', '.join(METHODS)})")
        if len(set(methods)) != len(methods):
            problems.append(f"repeated method in {v_cell!r}")
        if status not in STATUSES:
            problems.append(f"unknown status {status!r} (allowed: {', '.join(STATUSES)})")
        tokens = tuple(dict.fromkeys(TOKEN_RE.findall(by_cell)))
        leftover = TOKEN_RE.sub("", by_cell)
        leftover = re.sub(r"[`;,\s]", "", leftover)
        if leftover and leftover not in ("—", "-", "–"):
            problems.append(f"Verified-by has text outside tokens: {leftover!r}")
        retired = shall.startswith("~~")
        out.append(
            Requirement(
                id=rid, doc=doc, path=rel, line=lineno, area=area, number=number, shall=shall, cls=cls,
                source=source, methods=methods, tokens=tokens, status=status, retired=retired,
                problems=tuple(problems),
            )
        )
    return out


# --------------------------------------------------------------------------- python markers


def _mark(node: ast.expr) -> tuple[str, ast.Call | None] | None:
    """``pytest.mark.<name>`` / ``mark.<name>`` (called or not) → (name, call)."""
    call = node if isinstance(node, ast.Call) else None
    target = node.func if call is not None else node
    if isinstance(target, ast.Attribute):
        base = target.value
        if (isinstance(base, ast.Attribute) and base.attr == "mark") or (isinstance(base, ast.Name) and base.id == "mark"):
            return target.attr, call
    return None


def _marks(nodes: Iterable[ast.expr]) -> list[tuple[str, ast.Call | None, int]]:
    out = []
    for node in nodes:
        m = _mark(node)
        if m is not None:
            out.append((m[0], m[1], getattr(node, "lineno", 0)))
    return out


def _module_marks(tree: ast.Module) -> list[tuple[str, ast.Call | None, int]]:
    out: list[tuple[str, ast.Call | None, int]] = []
    for node in tree.body:
        value: ast.expr | None = None
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "pytestmark" for t in node.targets):
            value = node.value
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.target.id == "pytestmark":
            value = node.value
        if value is None:
            continue
        items = value.elts if isinstance(value, (ast.List, ast.Tuple)) else [value]
        out += _marks(items)
    return out


def _req_ids(call: ast.Call | None, rel: str, lineno: int, errors: list[Issue]) -> tuple[str, ...]:
    if call is None or not call.args:
        errors.append(Issue("marker_syntax", "pytest.mark.req needs at least one FAB ID", f"{rel}:{lineno}"))
        return ()
    ids: list[str] = []
    for arg in call.args:
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            if ID_FULL_RE.match(arg.value):
                ids.append(arg.value)
            else:
                errors.append(Issue("marker_syntax", f"malformed ID {arg.value!r} in pytest.mark.req", f"{rel}:{lineno}"))
        else:
            errors.append(Issue("marker_syntax", "pytest.mark.req arguments must be string literals", f"{rel}:{lineno}"))
    return tuple(dict.fromkeys(ids))


def _scan_py(root: Path) -> _Scan:
    scan = _Scan()
    tests_dir = root / "tests"
    if not tests_dir.is_dir():
        return scan
    for path in sorted(tests_dir.rglob("test_*.py")):
        rel_parts = path.relative_to(root).parts
        if _skip(rel_parts):
            continue
        rel = _rel(root, path)
        text = _read(path)
        if text is None:
            scan.errors.append(Issue("unreadable", "cannot read test file", rel))
            continue
        try:
            tree = ast.parse(text, filename=rel)
        except SyntaxError as exc:
            scan.errors.append(Issue("syntax", f"cannot parse: {exc.msg}", f"{rel}:{exc.lineno or 0}"))
            continue
        sub = path.relative_to(tests_dir).with_suffix("").parts
        prefix = "py:" + "/".join((*sub[:-1], sub[-1][len("test_") :]))
        module_marks = _module_marks(tree)

        def visit(fn: ast.FunctionDef | ast.AsyncFunctionDef, outer: list[tuple[str, ast.Call | None, int]]) -> None:
            own = _marks(fn.decorator_list)
            allm = outer + own
            is_test = fn.name.startswith("test_")
            slow = any(name == "slow" for name, _, _ in allm)
            token = f"{prefix}::{fn.name[len('test_'):]}" if is_test else ""
            if is_test:
                scan.tests[token] = TestLoc(token, rel, fn.lineno, slow)
            ids: list[str] = []
            for name, call, lineno in allm:
                if name == "req":
                    ids += _req_ids(call, rel, lineno or fn.lineno, scan.errors)
            if not ids:
                return
            if not is_test:
                scan.errors.append(
                    Issue("unattached_marker", f"req marker on {fn.name!r}, which is not a test_ function", f"{rel}:{fn.lineno}")
                )
                return
            scan.markers.append(Marker(tuple(dict.fromkeys(ids)), token, rel, fn.lineno, "py", slow))

        for node in tree.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                visit(node, module_marks)
            elif isinstance(node, ast.ClassDef):
                cls_marks = module_marks + _marks(node.decorator_list)
                for item in node.body:
                    if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                        visit(item, cls_marks)
    return scan


def collect_py_tests(root: str | Path | None = None) -> dict[str, TestLoc]:
    """Every pytest function under tests/, keyed by its token."""
    return _scan_py(find_root(root)).tests


def collect_py_markers(root: str | Path | None = None) -> list[Marker]:
    """``@pytest.mark.req`` markers under tests/ (AST walk; nothing is imported)."""
    return _scan_py(find_root(root)).markers


# --------------------------------------------------------------------------- rust markers


def _rs_files(root: Path) -> Iterator[tuple[Path, str]]:
    """(file, token prefix) for every Rust source the guard reads."""
    crates = root / "crates"
    if crates.is_dir():
        for crate_dir in sorted(p for p in crates.iterdir() if p.is_dir()):
            name = crate_dir.name.removeprefix("fabrail-")
            for sub in ("src", "tests"):
                base = crate_dir / sub
                if not base.is_dir():
                    continue
                for f in sorted(base.rglob("*.rs")):
                    if _skip(f.relative_to(root).parts):
                        continue
                    rel = f.relative_to(base).with_suffix("").as_posix()
                    if sub == "tests":
                        rel = "tests/" + rel
                    yield f, canonical_token(f"rs:{name}/{rel}::x").rsplit("::", 1)[0]
    fw = root / "firmware" / "src"
    if fw.is_dir():
        for f in sorted(fw.rglob("*.rs")):
            if _skip(f.relative_to(root).parts):
                continue
            rel = f.relative_to(fw).with_suffix("").as_posix()
            yield f, canonical_token(f"rs:fw/{rel}::x").rsplit("::", 1)[0]


def _fn_after(lines: Sequence[str], k: int, limit: int = 40) -> tuple[str | None, int, bool]:
    """The first ``fn`` at or after line ``k`` (skipping attributes); (name, index, ignored)."""
    ignored = False
    m = RS_FN_RE.search(lines[k].split("]", 1)[-1]) if RS_TEST_RE.match(lines[k]) else None
    if m:
        return m.group(1), k, ignored
    for j in range(k + 1, min(len(lines), k + 1 + limit)):
        s = lines[j].strip()
        if RS_IGNORE_RE.search(s):
            ignored = True
        m = RS_FN_RE.search(s)
        if m and not s.startswith("//"):
            return m.group(1), j, ignored
    return None, k, ignored


def _ignored_above(lines: Sequence[str], k: int) -> bool:
    """``#[ignore]`` among the attribute/comment lines directly above line ``k``."""
    for j in range(k - 1, max(-1, k - 1 - MARKER_WINDOW), -1):
        s = lines[j].strip()
        if RS_IGNORE_RE.search(s):
            return True
        if not (s.startswith("#[") or s.startswith("//")):
            return False
    return False


def _scan_rs_file(path: Path, prefix: str, rel: str, scan: _Scan) -> None:
    text = _read(path)
    if text is None:
        scan.errors.append(Issue("unreadable", "cannot read Rust file", rel))
        return
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if RS_TEST_RE.match(line):
            name, j, ignored = _fn_after(lines, i)
            if name:
                token = f"{prefix}::{name}"
                scan.tests[token] = TestLoc(token, rel, j + 1, ignored or _ignored_above(lines, i))
    for i, line in enumerate(lines):
        m = RS_REQ_RE.match(line)
        if not m:
            continue
        where = f"{rel}:{i + 1}"
        ids = tuple(dict.fromkeys(f"FAB-{a}-{n}" for a, n in ID_RE.findall(m.group(1))))
        if not ids:
            scan.errors.append(Issue("marker_syntax", f"`// req:` line without a FAB ID: {line.strip()!r}", where))
            continue
        anchor: int | None = None
        for j in range(i + 1, min(len(lines), i + 1 + MARKER_WINDOW)):
            s = lines[j].strip()
            if RS_TEST_RE.match(lines[j]) or (RS_PROPTEST_RE.search(s) and not s.startswith("//")):
                anchor = j
                break
            if not s or s.startswith("//") or s.startswith("#[") or s.startswith("#!["):
                continue
            break
        if anchor is None:
            scan.errors.append(
                Issue("unattached_marker", f"`// req:` for {', '.join(ids)} is not within {MARKER_WINDOW} lines above #[test] or proptest!", where)
            )
            continue
        name, j, fn_ignored = _fn_after(lines, anchor)
        if name is None:
            scan.errors.append(Issue("unattached_marker", f"no fn after the test attribute for {', '.join(ids)}", where))
            continue
        token = f"{prefix}::{name}"
        slow = fn_ignored or _ignored_above(lines, anchor)
        loc = scan.tests.setdefault(token, TestLoc(token, rel, j + 1, slow))
        scan.markers.append(Marker(ids, token, rel, i + 1, "rs", loc.slow or slow))


def _scan_rs(root: Path) -> _Scan:
    scan = _Scan()
    for path, prefix in _rs_files(root):
        _scan_rs_file(path, prefix, _rel(root, path), scan)
    return scan


def collect_rs_tests(root: str | Path | None = None) -> dict[str, TestLoc]:
    """Every Rust ``#[test]`` function (crates/*/src, crates/*/tests, firmware/src), keyed by token."""
    return _scan_rs(find_root(root)).tests


def collect_rs_markers(root: str | Path | None = None) -> list[Marker]:
    """``// req:`` markers attached to Rust tests."""
    return _scan_rs(find_root(root)).markers


# --------------------------------------------------------------------------- citations


def _cite_files(root: Path) -> Iterator[Path]:
    for name in _CITE_FILES:
        p = root / name
        if p.is_file():
            yield p
    for top in _CITE_ROOTS:
        base = root / top
        if not base.is_dir():
            continue
        for p in sorted(base.rglob("*")):
            if p.is_file() and p.suffix in _CITE_EXTS and not _skip(p.relative_to(root).parts):
                yield p


def _citations(line: str) -> set[str]:
    if _IGNORE_PRAGMA in line:
        return set()
    ids = {f"FAB-{a}-{n}" for a, n in ID_RE.findall(line)}
    for m in CONT_RE.finditer(line):
        area = m.group(1)
        ids |= {f"FAB-{area}-{n}" for n in BARE_NUM_RE.findall(m.group(3))}
    return ids


# --------------------------------------------------------------------------- the guard


def check(root: str | Path | None = None) -> TraceReport:
    """Run every traceability check under ``root`` (default: :func:`find_root`)."""
    root_p = find_root(root)
    report = TraceReport(root=str(root_p))
    err = report.errors.append

    # ---- documents ---------------------------------------------------------
    for doc in SPEC_DOCS:
        path = root_p / "docs" / f"{doc}.md"
        if not path.is_file():
            err(Issue("missing_doc", f"docs/{doc}.md not found"))
            continue
        rows = parse_spec(path, doc=doc, root=root_p)
        report.requirements += rows
        area_first: list[str] = []
        last_number: dict[str, int] = {}
        for r in rows:
            where = f"{r.path}:{r.line}"
            for p in r.problems:
                err(Issue("row_format", f"{r.id}: {p}", where))
            if r.area == "?":
                continue
            if r.area not in AREAS:
                err(Issue("unknown_area", f"{r.id}: unknown area {r.area!r}", where))
            elif r.area not in DOC_AREAS[doc]:
                err(Issue("wrong_document", f"{r.id}: area {r.area} belongs in another document, not {doc}.md", where))
            if r.area in DOC_AREAS[doc]:  # contiguity is tracked over this document's own areas
                if r.area not in area_first:
                    area_first.append(r.area)
                elif area_first[-1] != r.area:
                    err(Issue("area_order", f"{r.id}: area {r.area} is reopened after {area_first[-1]} rows", where))
                    area_first.append(r.area)
            if r.area in last_number and r.number <= last_number[r.area]:
                err(Issue("order", f"{r.id} does not ascend within {r.area} (after FAB-{r.area}-{last_number[r.area]:03d})", where))
            last_number[r.area] = max(r.number, last_number.get(r.area, -1))
            _check_row_rules(r, err)
        present = list(dict.fromkeys(area_first))
        expected = [a for a in DOC_AREAS[doc] if a in present]
        if present != expected:
            err(Issue("area_order", f"areas appear as {', '.join(present)}; expected {', '.join(expected)}", f"docs/{doc}.md"))

    reqs = [r for r in report.requirements if r.area != "?"]
    by_id: dict[str, Requirement] = {}
    for r in reqs:
        if r.id in by_id:
            first = by_id[r.id]
            err(Issue("duplicate_id", f"{r.id} is already defined at {first.path}:{first.line}", f"{r.path}:{r.line}"))
        else:
            by_id[r.id] = r

    # ---- tokens: grammar and crates ------------------------------------------
    for r in by_id.values():
        for t in r.tokens:
            _, _, problem = token_path(root_p, t)
            if problem:
                err(Issue("bad_token", f"{r.id}: {problem}", f"{r.path}:{r.line}"))

    # ---- markers ------------------------------------------------------------
    py = _scan_py(root_p)
    rs = _scan_rs(root_p)
    report.errors += py.errors + rs.errors
    report.markers = py.markers + rs.markers
    report.tests = {**py.tests, **rs.tests}

    marked: dict[str, dict[str, Marker]] = defaultdict(dict)
    for mk in report.markers:
        for rid in mk.ids:
            if rid not in by_id:
                err(Issue("dangling_marker", f"test {mk.token} carries {rid}, which no document defines", f"{mk.path}:{mk.line}"))
                continue
            marked[rid][mk.token] = mk

    # ---- bidirectional ------------------------------------------------------
    for rid, r in by_id.items():
        listed = {canonical_token(t) for t in r.tokens}
        have = marked.get(rid, {})
        for t in sorted(set(have) - listed):
            mk = have[t]
            err(Issue("unlisted_marker", f"test {t} is marked {rid}, but the {rid} row does not list it", f"{mk.path}:{mk.line}"))
        for t in sorted(listed - set(have)):
            files, _, problem = token_path(root_p, t)
            if problem:
                continue  # already reported as bad_token
            existing = [f for f in files if f.is_file()]
            if t in report.tests:
                reason, path = "unmarked", report.tests[t].path
            elif existing:
                reason, path = "no_test", _rel(root_p, existing[0])
            else:
                reason, path = "no_file", _rel(root_p, files[0])
            report.missing.append(MissingTest(rid, t, reason, path))
        if have and all(mk.slow for mk in have.values()):
            report.slow_only.append(rid)

    # ---- citations -----------------------------------------------------------
    known = set(by_id)
    seen_cites: set[tuple[str, str]] = set()
    for path in _cite_files(root_p):
        text = _read(path)
        if text is None:
            continue
        rel = _rel(root_p, path)
        for lineno, line in enumerate(text.splitlines(), 1):
            for cid in sorted(_citations(line) - known):
                if (rel, cid) in seen_cites:
                    continue
                seen_cites.add((rel, cid))
                err(Issue("dangling_citation", f"{cid} is cited but not defined in any requirement document", f"{rel}:{lineno}"))

    _check_hazards(root_p, by_id, report)
    _check_references(root_p, report)
    _check_counts(root_p, report)
    return report


def _check_row_rules(r: Requirement, err) -> None:  # noqa: ANN001 - list.append
    where = f"{r.path}:{r.line}"
    methods = set(r.methods)
    if r.retired:
        if r.status != "M" or r.methods != ("I",) or r.tokens:
            err(Issue("retired_row", f"{r.id}: a retired row is status M, method I, with no token", where))
        return
    if r.status in ("V", "V(I)", "S", "R") and not r.tokens:
        err(Issue("status_rule", f"{r.id}: status {r.status} needs at least one test token", where))
    if r.status == "M":
        if r.tokens:
            err(Issue("status_rule", f"{r.id}: an M (roadmap) row carries no test token", where))
        if not methods or not methods <= {"D", "I"}:
            err(Issue("status_rule", f"{r.id}: an M (roadmap) row uses method D or I", where))
    if "I" in methods and r.status not in ("V(I)", "M"):
        err(Issue("status_rule", f"{r.id}: a method-I row has status V(I) or M (got {r.status})", where))
    if r.status == "V(I)" and "I" not in methods:
        err(Issue("status_rule", f"{r.id}: status V(I) needs method I", where))


def _check_hazards(root: Path, by_id: dict[str, Requirement], report: TraceReport) -> None:
    safety = root / "docs" / "SAFETY.md"
    text = _read(safety) if safety.is_file() else None
    if text is None:
        report.notes.append("docs/SAFETY.md absent: hazard mapping (H-01 to H-21) not checked yet")
        return
    rows: dict[str, tuple[int, str]] = {}
    for lineno, line in enumerate(text.splitlines(), 1):
        m = HAZARD_ROW_RE.match(line)
        if m:
            rows.setdefault(m.group(1), (lineno, line))
    cited_by_spec: set[str] = set()
    for r in by_id.values():
        cited_by_spec |= {f"H-{n}" for n in HAZARD_RE.findall(" ".join((r.shall, r.source)))}
    for h in HAZARDS:
        if h not in rows:
            report.errors.append(Issue("hazard", f"{h} is not a hazard row in SAFETY.md", "docs/SAFETY.md"))
            continue
        lineno, line = rows[h]
        mapped = {f"FAB-{a}-{n}" for a, n in ID_RE.findall(line)} & set(by_id)
        if not mapped and h not in cited_by_spec:
            report.errors.append(Issue("hazard", f"{h} maps to no requirement", f"docs/SAFETY.md:{lineno}"))


def _check_references(root: Path, report: TraceReport) -> None:
    refs = root / "docs" / "references.md"
    cited: dict[str, str] = {}
    docs = root / "docs"
    if docs.is_dir():
        for p in sorted(docs.glob("*.md")):
            if p.name == "references.md":
                continue
            text = _read(p) or ""
            for lineno, line in enumerate(text.splitlines(), 1):
                for group in RTAG_RE.findall(line):
                    for tag in re.findall(r"R\d+", group):
                        cited.setdefault(tag, f"{_rel(root, p)}:{lineno}")
    if not refs.is_file():
        if cited:
            report.notes.append(f"docs/references.md absent: {len(cited)} [R#] tag(s) not checked yet")
        return
    text = _read(refs) or ""
    defined: set[str] = set()
    for line in text.splitlines():
        for group in RTAG_RE.findall(line):
            defined |= set(re.findall(r"R\d+", group))
        m = RDEF_ROW_RE.match(line)
        if m:
            defined.add(m.group(1))
    for tag in sorted(set(cited) - defined, key=lambda s: int(s[1:])):
        report.errors.append(Issue("reference", f"[{tag}] does not resolve in docs/references.md", cited[tag]))


def _check_counts(root: Path, report: TraceReport) -> None:
    js = root / "docs" / "assets" / "data" / "trace.js"
    text = _read(js) if js.is_file() else None
    if text is None:
        report.notes.append("docs/assets/data/trace.js absent: site counts not checked")
    else:
        m = re.search(r"/\*\s*TRACE:BEGIN\s*\*/(.*?)/\*\s*TRACE:END\s*\*/", text, re.S)
        data: dict[str, object] | None = None
        if m:
            body = re.sub(r"^\s*var\s+TRACE\s*=\s*", "", m.group(1).strip()).rstrip().rstrip(";")
            try:
                loaded = json.loads(body)
                data = loaded if isinstance(loaded, dict) else None
            except ValueError:
                data = None
        if data is None:
            report.errors.append(Issue("counts", "cannot read the TRACE block", "docs/assets/data/trace.js"))
        elif data.get("placeholder"):
            report.notes.append("docs/assets/data/trace.js is a placeholder: site counts not checked yet")
        else:
            expected = {
                "total": report.total,
                "by_doc": report.by_doc,
                "by_area": report.by_area,
                "by_status": report.by_status,
                "tokens": report.tokens,
            }
            for key, value in expected.items():
                if data.get(key) != value:
                    report.errors.append(
                        Issue("counts", f"TRACE.{key} = {data.get(key)!r}, parsed {value!r} (regenerate with scripts/update_docs_run.py)", "docs/assets/data/trace.js")
                    )
    readme = root / "README.md"
    text = _read(readme) if readme.is_file() else None
    block = re.search(r"<!--\s*TRACE:BEGIN\s*-->(.*?)<!--\s*TRACE:END\s*-->", text or "", re.S)
    if not block:
        report.notes.append("README.md has no <!-- TRACE:BEGIN --> block: README counts not checked")
        return
    for n in re.findall(r"(\d+)\s+requirements\b", block.group(1)):
        if int(n) != report.total:
            report.errors.append(Issue("counts", f"README says {n} requirements, parsed {report.total}", "README.md"))


# --------------------------------------------------------------------------- CLI


def add_arguments(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Add the ``trace`` flags (shared by ``python -m fabrail trace`` and ``python -m fabrail.trace``)."""
    parser.add_argument(
        "--json", nargs="?", const="-", default=None, metavar="PATH", dest="json_path",
        help="write the report as JSON to PATH, or to stdout when PATH is omitted",
    )
    parser.add_argument("--root", type=Path, default=None, help="repository root (default: found from the current directory)")
    parser.add_argument(
        "--allow-missing", action="store_true",
        help="exit 0 when only missing tests fail (bring-up waves); structure errors still fail",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="list every missing test")
    return parser


def cli(args: argparse.Namespace) -> int:
    """Run the guard for parsed ``args``; returns the exit status (0 pass, 1 fail)."""
    report = check(args.root)
    json_path = getattr(args, "json_path", None)
    if json_path == "-":
        sys.stdout.write(report.to_json() + "\n")
    else:
        if json_path:
            out = Path(json_path)
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(report.to_json() + "\n", encoding="utf-8")
        print(report.summary(verbose=getattr(args, "verbose", False)))
        if json_path:
            print(f"JSON report written to {json_path}")
    if report.ok or (getattr(args, "allow_missing", False) and report.structure_ok):
        return 0
    return 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = add_arguments(
        argparse.ArgumentParser(prog="python -m fabrail.trace", description="Requirement ⇄ test traceability guard (FAB-SW-007).")
    )
    return cli(parser.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
