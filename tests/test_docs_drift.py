"""Documentation drift guards (FAB-PRP-001, FAB-SW-006, FAB-SW-010, FAB-EXA-016).

Contract:

- ``frame_glossary``: SPECIFICATION §1.2/§1.3 and PHYSICS §1 agree on the frames — x is
  travel along the rail (+x opens; the README's "lateral propulsion"), y is cross-track
  ("lateral"), z is up — and no code identifier ties "lateral" to x, travel or
  propulsion (FAB-PRP-001).
- ``readme_verbatim``: README.md begins with the user's original README byte for byte
  (470 bytes, SHA-256 recorded in ``_support``; FAB-SW-010).
- ``run_block_matches_anchors``: the generated RUN and ANCHORS data blocks
  (docs/assets/data, written by scripts/update_docs_run.py in Wave 3) quote the recorded
  seed-42 anchors and the frozen golden hash, and every RUN entry's KPIs and sampled
  channels equal a fresh seed-42 run of the same class, scenario, tier and coil mode.
  Skipped while the block is a placeholder or the anchors are unrecorded (FAB-SW-006).
- ``villa_run_block_matches_record``: the VILLA_RUN block equals the villa_quake record
  in examples/out/villa_quake/ on every scalar the fabrail.docs.villa_run/1 schema
  shares with it. Skipped while the block is a placeholder (FAB-EXA-016).
- ``physics_section_refs``: every ``PHYSICS §n`` citation in code, tests, scripts and
  docs names a section heading that exists in docs/PHYSICS.md (FAB-SW-006).
- ``parameters_md_current``: docs/PARAMETERS.md is exactly
  ``python -m fabrail params --markdown`` (FAB-SW-006).
- Unmarked companions (core design §E.3): ``sliders_match_presets``,
  ``formulas_match_rust`` (node), ``plotstyle_matches_css`` and
  ``no_external_resources``.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
from html.parser import HTMLParser
from pathlib import Path

import pytest

import _support

ROOT = _support.ROOT
DOCS = _support.DOCS
DATA = DOCS / "assets" / "data"
SPEC = DOCS / "SPECIFICATION.md"
PHYSICS = DOCS / "PHYSICS.md"


def _read(p: Path) -> str:
    return p.read_text(encoding="utf-8")


def _section(text: str, heading_re: str) -> str:
    m = re.search(heading_re, text, re.M)
    assert m, f"heading {heading_re!r} not found"
    rest = text[m.end() :]
    level = len(re.match(r"#+", m.group(0)).group(0))
    nxt = re.search(rf"^#{{1,{level}}}\s", rest, re.M)
    return rest[: nxt.start()] if nxt else rest


def _table(section: str) -> dict[str, str]:
    """First-column key (code marks stripped) → the rest of the row, for a 2+-column table."""
    out = {}
    for line in section.splitlines():
        if not line.strip().startswith("|") or set(line.replace("|", "").strip()) <= {"-", " ", ":"}:
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) >= 2:
            out[cells[0].strip("`").strip().lower()] = " | ".join(cells[1:])
    return out


# ---------------------------------------------------------------------------- FAB-PRP-001

_BAD_LATERAL_IDENT = re.compile(
    r"\b\w*(lateral_x|x_lateral|lateral_travel|travel_lateral|lateral_propuls\w*|lateral_thrust|lateral_drive)\w*\b",
    re.I,
)


@pytest.mark.req("FAB-PRP-001")
def test_frame_glossary():
    """x = travel (+x opens; the README's "lateral propulsion"), y = cross-track
    ("lateral"), z = up in SPECIFICATION §1.2 and PHYSICS §1; SPECIFICATION §1.3 reads
    "lateral propulsion" as travel along x; no identifier in code ties "lateral" to x,
    travel or propulsion (FAB-PRP-001)."""
    spec = _read(SPEC)
    frames = _table(_section(spec, r"^###\s*§1\.2\b.*$"))
    bad = []
    x, y, z = frames.get("x", ""), frames.get("y", ""), frames.get("z", "")
    if not (re.search(r"travel", x, re.I) and "+x opens" in x and "lateral propulsion" in x):
        bad.append(f'SPECIFICATION §1.2 x row: {x!r} (needs travel, +x opens, the README\'s "lateral propulsion")')
    if not (re.search(r"cross-track", y, re.I) and "lateral" in y):
        bad.append(f'SPECIFICATION §1.2 y row: {y!r} (needs cross-track and "lateral")')
    if not re.search(r"\bup\b", z, re.I):
        bad.append(f"SPECIFICATION §1.2 z row: {z!r} (needs up)")
    words = _table(_section(spec, r"^###\s*§1\.3\b.*$"))
    lp = words.get("lateral propulsion", "")
    if not re.search(r"travel along x", lp, re.I):
        bad.append(f"SPECIFICATION §1.3 'lateral propulsion': {lp!r} (needs 'travel along x')")
    phys = _section(_read(PHYSICS), r"^##\s*§1\b.*$")
    flat = re.sub(r"[`*_]", "", phys)
    checks = {
        "x is travel": r"\bx\b[^.\n]{0,60}travel|travel[^.\n]{0,40}\bx\b",
        "+x opens": r"\+x opens",
        "y is cross-track": r"\by\b[^.\n]{0,40}cross-track|cross-track[^.\n]{0,20}\by\b",
        "'lateral' means y": r"lateral",
        "z is up": r"\bz\b[^.\n]{0,20}\bup\b",
    }
    for what, pat in checks.items():
        if not re.search(pat, flat, re.I):
            bad.append(f"PHYSICS §1 does not state {what}")
    for base in (ROOT / "crates", ROOT / "firmware" / "src", ROOT / "python" / "fabrail"):
        for f in sorted(list(base.rglob("*.rs")) + list(base.rglob("*.py"))):
            if "target" in f.parts:
                continue
            for i, line in enumerate(_read(f).splitlines(), 1):
                m = _BAD_LATERAL_IDENT.search(line)
                if m:
                    bad.append(f"{_support.rel(f)}:{i}: identifier {m.group(0)!r} ties 'lateral' to x/travel")
    assert not bad, "frame and glossary drift (FAB-PRP-001):\n" + "\n".join(bad)


# ---------------------------------------------------------------------------- FAB-SW-010


@pytest.mark.req("FAB-SW-010")
def test_readme_verbatim():
    """README.md begins with the user's two paragraphs verbatim: its first 470 bytes equal
    tests/data/readme_original.md (whose SHA-256 is pinned) (FAB-SW-010)."""
    original = _support.README_ORIGINAL.read_bytes()
    assert len(original) == _support.README_ORIGINAL_BYTES, f"fixture is {len(original)} bytes, expected 470"
    assert hashlib.sha256(original).hexdigest() == _support.README_ORIGINAL_SHA256, (
        "tests/data/readme_original.md was edited (SHA-256 differs from the recorded original)"
    )
    readme = (ROOT / "README.md").read_bytes()
    head = readme[: len(original)]
    if head != original:
        k = next((i for i, (a, b) in enumerate(zip(head, original, strict=False)) if a != b), min(len(head), len(original)))
        pytest.fail(
            f"README.md differs from the user's original at byte {k}: "
            f"{head[max(0, k - 30) : k + 30]!r} vs {original[max(0, k - 30) : k + 30]!r} (FAB-SW-010)"
        )
    text = original.decode("utf-8")
    paragraphs = [p for p in text.split("\n\n") if p.strip() and not p.startswith("#")]
    assert len(paragraphs) == 2, f"the original README has {len(paragraphs)} paragraphs, expected 2"


# ---------------------------------------------------------------------------- FAB-SW-006: data blocks


def _golden(native) -> int:
    return int(native.GOLDEN_SELFTEST_HASH)


def _close(a, b, rel=1e-12, abs_tol=0.0) -> bool:
    if isinstance(a, list) or isinstance(b, list):
        return (
            isinstance(a, list)
            and isinstance(b, list)
            and len(a) == len(b)
            and all(_close(x, y, rel, abs_tol) for x, y in zip(a, b, strict=True))
        )
    if a is None or b is None:
        return a is None and b is None
    try:
        fa, fb = float(a), float(b)
    except (TypeError, ValueError):
        return a == b
    if math.isnan(fa) or math.isnan(fb):
        return math.isnan(fa) and math.isnan(fb)
    return fa == fb or math.isclose(fa, fb, rel_tol=rel, abs_tol=abs_tol)


@pytest.mark.req("FAB-SW-006")
def test_run_block_matches_anchors(native):
    """RUN and ANCHORS (docs/assets/data) quote the recorded seed-42 anchors and the
    frozen golden hash, and each RUN entry's KPIs and sampled channels equal a fresh
    seed-42 run (KPIs rel 1e-12; samples rel 1e-9) (FAB-SW-006). Skipped until Wave 3."""
    import fabrail

    run = _support.data_block(DATA / "run.js", "RUN")
    anchors_blk = _support.data_block(DATA / "anchors.js", "ANCHORS")
    assert run.get("schema") == "fabrail.docs.run/1", run.get("schema")
    if run.get("placeholder"):
        pytest.skip("docs/assets/data/run.js is the Wave-0 placeholder: C2a (Wave 3) runs scripts/update_docs_run.py")
    recorded = _support.seed42_anchors_or_skip()
    bad = []
    golden = _golden(native)
    for name, blk in (("RUN", run), ("ANCHORS", anchors_blk)):
        if blk.get("placeholder"):
            bad.append(f"{name} is still a placeholder while RUN is generated")
            continue
        if blk.get("fabrail_version") != fabrail.__version__:
            bad.append(f"{name}.fabrail_version {blk.get('fabrail_version')!r} != {fabrail.__version__}")
        if golden and blk.get("golden_selftest_hash") != f"0x{golden:016x}":
            bad.append(f"{name}.golden_selftest_hash {blk.get('golden_selftest_hash')} != 0x{golden:016x}")
    if run.get("seed") != 42:
        bad.append(f"RUN.seed = {run.get('seed')}, expected 42")
    rec = recorded.get("anchors", {})
    got = anchors_blk.get("seed42") or {}
    for name in sorted(set(rec) | set(got)):
        if name not in got:
            bad.append(f"ANCHORS.seed42 lacks recorded anchor {name!r}")
        elif name not in rec:
            bad.append(f"ANCHORS.seed42.{name} is not in tests/data/anchors_seed42.json")
        elif not _close(got[name].get("value"), rec[name].get("value")):
            bad.append(f"ANCHORS.seed42.{name} = {got[name].get('value')!r} != recorded {rec[name].get('value')!r}")
    for key, entry in sorted((run.get("runs") or {}).items()):
        fresh = _support.cached_run(
            entry["class"], entry["scenario"], seed=run["seed"], coil=entry["coil"], fidelity=entry["fidelity"]
        )
        kp = fresh.kpis
        for k, v in (entry.get("kpis") or {}).items():
            if k not in kp or not _close(v, kp[k]):
                bad.append(f"RUN.runs.{key}.kpis.{k} = {v!r}, fresh run {kp.get(k)!r}")
        res = fresh.result
        t = entry.get("t_s") or []
        for ch, blk in (entry.get("channels") or {}).items():
            data = blk.get("data") or []
            if len(data) != len(t):
                bad.append(f"RUN.runs.{key}.channels.{ch}: {len(data)} samples for {len(t)} times")
                continue
            if ch not in res.channels:
                bad.append(f"RUN.runs.{key}.channels.{ch}: not a channel of the fresh run")
                continue
            arr = res.channels[ch]
            for k in sorted({0, len(t) // 2, len(t) - 1}) if t else []:
                idx = int(round(float(t[k]) * res.rate_hz))
                if not (0 <= idx < len(arr)) or not _close(data[k], float(arr[idx]), rel=1e-9, abs_tol=1e-12):
                    fresh_v = arr[idx] if 0 <= idx < len(arr) else None
                    bad.append(f"RUN.runs.{key}.channels.{ch}[{k}] (t={t[k]}) = {data[k]!r}, fresh {fresh_v!r}")
                    break
    assert not bad, "stale RUN/ANCHORS blocks (rerun scripts/update_docs_run.py; FAB-SW-006):\n" + "\n".join(bad)


# ---------------------------------------------------------------------------- FAB-EXA-016

_VILLA_DIR = ROOT / "examples" / "out" / "villa_quake"
_VILLA_TOP = ("seed", "variant", "building")
#: Compared only when the record carries them (static wording, also checked on the page).
_VILLA_TOP_OPTIONAL = ("label", "alert_label")
_VILLA_DEVICE = (
    "class",
    "room",
    "egress",
    "policy",
    "badge",
    "fidelity",
    "rank_target",
    "deadline_s",
    "t_secured_s",
    "status",
    "reason",
)
_VILLA_CONTENTS = ("mu_s", "slip_events", "rocking_flag", "cone_utilisation_max")


def _villa_record() -> tuple[Path, dict]:
    cands = sorted(_VILLA_DIR.glob("*.json")) if _VILLA_DIR.is_dir() else []
    preferred = [p for p in cands if p.stem in ("villa_quake", "record", "villa_run")]
    for p in preferred + [p for p in cands if p not in preferred]:
        try:
            doc = json.loads(_read(p))
        except ValueError:
            continue
        if isinstance(doc, dict) and ("devices" in doc or any(isinstance(doc.get(k), dict) and "devices" in doc[k] for k in doc)):
            return p, doc
    pytest.fail(
        f"no villa_quake record with a 'devices' list under {_support.rel(_VILLA_DIR)}/ "
        "(run `python -m fabrail example run villa_quake`; E5 owns the record)"
    )


def _roots(doc: dict) -> list[dict]:
    out = [doc]
    for k in ("villa_run", "docs", "record", "result"):
        if isinstance(doc.get(k), dict):
            out.append(doc[k])
    return out


def _find(roots: list[dict], *path: str):
    for r in roots:
        cur = r
        for p in path:
            if not isinstance(cur, dict) or p not in cur:
                break
            cur = cur[p]
        else:
            return True, cur
    return False, None


@pytest.mark.req("FAB-EXA-016")
def test_villa_run_block_matches_record():
    """VILLA_RUN (docs/assets/data/villa_run.js) equals the villa_quake record in
    examples/out/villa_quake/ on every shared scalar: seed, variant, building, labels,
    event, per-device policy/status/reason/deadline/secured time, contents slip counts,
    the smoothing ablation and the bus budget (FAB-EXA-016). Skipped until Wave 3."""
    blk = _support.data_block(DATA / "villa_run.js", "VILLA_RUN")
    assert blk.get("schema") == "fabrail.docs.villa_run/1", blk.get("schema")
    if blk.get("placeholder"):
        pytest.skip("docs/assets/data/villa_run.js is the Wave-0 placeholder: C2a (Wave 3) generates it from E5's record")
    path, doc = _villa_record()
    roots = _roots(doc)
    bad = []

    def cmp(label: str, want, *where: str, optional: bool = False) -> None:
        ok, got = _find(roots, *where)
        if not ok:
            if not optional:
                bad.append(f"{label}: absent from {_support.rel(path)}")
        elif not _close(want, got, rel=1e-12):
            bad.append(f"{label}: block {want!r} vs record {got!r}")

    for k in _VILLA_TOP:
        if k in blk:
            cmp(k, blk[k], k)
    for k in _VILLA_TOP_OPTIONAL:
        if k in blk:
            cmp(k, blk[k], k, optional=True)
    for k, v in (blk.get("event") or {}).items():
        cmp(f"event.{k}", v, "event", k)
    ok, rec_devices = _find(roots, "devices")
    rec_by_id = {d.get("id"): d for d in (rec_devices or []) if isinstance(d, dict)}
    for d in blk.get("devices") or []:
        rd = rec_by_id.get(d.get("id"))
        if rd is None:
            bad.append(f"device {d.get('id')!r} absent from the record")
            continue
        for k in _VILLA_DEVICE:
            if k in d and not _close(d[k], rd.get(k)):
                bad.append(f"devices[{d['id']}].{k}: block {d[k]!r} vs record {rd.get(k)!r}")
    ok, rec_contents = _find(roots, "contents")
    rc = {(c.get("device"), c.get("kind")): c for c in (rec_contents or []) if isinstance(c, dict)}
    for c in blk.get("contents") or []:
        r = rc.get((c.get("device"), c.get("kind")))
        if r is None:
            bad.append(f"contents ({c.get('device')}, {c.get('kind')}) absent from the record")
            continue
        for k in _VILLA_CONTENTS:
            if k in c and not _close(c[k], r.get(k)):
                bad.append(f"contents ({c['device']}, {c['kind']}).{k}: block {c[k]!r} vs record {r.get(k)!r}")
    for bld, vals in (blk.get("smoothing_ablation") or {}).items():
        for k, v in (vals or {}).items():
            cmp(f"smoothing_ablation.{bld}.{k}", v, "smoothing_ablation", bld, k)
    if isinstance(blk.get("bus_power"), dict) and "budget_w" in blk["bus_power"]:
        cmp("bus_power.budget_w", blk["bus_power"]["budget_w"], "bus_power", "budget_w")
    assert not bad, f"VILLA_RUN vs {_support.rel(path)} (FAB-EXA-016):\n" + "\n".join(bad)


# ---------------------------------------------------------------------------- FAB-SW-006: § refs

_PHYS_CITE = re.compile(
    r"PHYSICS(?:\.md)?\s*§\s*(\d+(?:\.\d+)?)((?:\s*(?:,|;|/|and|or|&|–|—|-|to)\s*§?\s*\d+(?:\.\d+)?(?![\d.]*\s*(?:mm|ms|s|Hz|kg|N|m)\b))*)"
)
_CITE_GLOBS = (
    "crates/**/*.rs",
    "firmware/src/**/*.rs",
    "python/**/*.py",
    "python/**/*.pyi",
    "tests/**/*.py",
    "scripts/*",
    "examples/*.py",
    "docs/*.md",
    "docs/*.html",
    "docs/assets/js/*.js",
    "README.md",
)


def physics_citations() -> list[tuple[str, int, str]]:
    """(file, line, section) for every ``PHYSICS §n`` citation (with ``, §m`` / ``–§m`` chains)."""
    out = []
    seen: set[Path] = set()
    for pattern in _CITE_GLOBS:
        for f in sorted(ROOT.glob(pattern)):
            if f in seen or not f.is_file() or "target" in f.relative_to(ROOT).parts or f.name == "PHYSICS.md":
                continue
            seen.add(f)
            try:
                text = _read(f)
            except UnicodeDecodeError:
                continue
            for i, line in enumerate(text.splitlines(), 1):
                for s in cites_in_line(line):
                    out.append((_support.rel(f), i, s))
    return out


def cites_in_line(line: str) -> list[str]:
    """PHYSICS section numbers cited in one line, in order, without duplicates."""
    secs: list[str] = []
    for m in _PHYS_CITE.finditer(line):
        secs.append(m.group(1))
        secs += re.findall(r"§?\s*(\d+(?:\.\d+)?)", m.group(2))
    return list(dict.fromkeys(secs))


def test_physics_citation_parser():
    """The citation scanner reads chains and ranges and ignores quantities after a comma."""
    cases = {
        "// PHYSICS §8, §24": ["8", "24"],
        "(PHYSICS §2–§3)": ["2", "3"],
        "PHYSICS §16.1 terminus": ["16.1"],
        "(E) rel 1e-12, PHYSICS §3 eq 3.4": ["3"],
        "PHYSICS §5, 20 ms later": ["5"],
        "PHYSICS §9, §14, §16.1": ["9", "14", "16.1"],
        "PHYSICS.md §21 and §16.2": ["21", "16.2"],
        "EXOTIC §X3 and core §C.2": [],
    }
    for line, want in cases.items():
        assert cites_in_line(line) == want, f"{line!r}: {cites_in_line(line)} != {want}"


@pytest.mark.req("FAB-SW-006")
def test_physics_section_refs():
    """Every ``PHYSICS §n`` (and chained ``, §m`` / ``–§m``) cited in code, tests,
    scripts, examples and docs names a ``§`` heading of docs/PHYSICS.md (FAB-SW-006)."""
    sections = _support.physics_sections()
    assert {str(n) for n in range(1, 27)} <= sections, (
        f"PHYSICS.md lacks frozen headings: {sorted(set(map(str, range(1, 27))) - sections)}"
    )
    assert {"16.1", "16.2"} <= sections, "PHYSICS.md lacks §16.1 / §16.2"
    cites = physics_citations()
    assert len(cites) > 100, f"only {len(cites)} PHYSICS § citations found (the scanner is broken?)"
    missing = sorted({(s, f"{f}:{i}") for f, i, s in cites if s not in sections})
    first: dict[str, str] = {}
    for s, where in missing:
        first.setdefault(s, where)
    assert not first, "PHYSICS § citations with no such section heading (FAB-SW-006):\n" + "\n".join(
        f"§{s} (first at {w}; {sum(1 for x, _ in missing if x == s)} places)" for s, w in sorted(first.items())
    )


# ---------------------------------------------------------------------------- FAB-SW-006: PARAMETERS.md


@pytest.mark.req("FAB-SW-006")
def test_parameters_md_current(native):
    """docs/PARAMETERS.md is exactly the output of ``python -m fabrail params
    --markdown`` (config.parameters_markdown(); generated, never hand-edited;
    FAB-SW-006)."""
    from fabrail.config import parameters_markdown

    path = DOCS / "PARAMETERS.md"
    assert path.is_file(), (
        "docs/PARAMETERS.md is missing: generate it with `python -m fabrail params --markdown > docs/PARAMETERS.md`"
    )
    want = parameters_markdown()
    got = _read(path)
    if got != want:
        a, b = got.splitlines(), want.splitlines()
        k = next((i for i, (x, y) in enumerate(zip(a, b, strict=False)) if x != y), min(len(a), len(b)))
        pytest.fail(
            f"docs/PARAMETERS.md is stale at line {k + 1}: {a[k] if k < len(a) else '<EOF>'!r} vs generated "
            f"{b[k] if k < len(b) else '<EOF>'!r} (regenerate with `python -m fabrail params --markdown`)"
        )


# ---------------------------------------------------------------------------- unmarked companions


class _Inputs(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.ranges: list[dict[str, str | None]] = []

    def handle_starttag(self, tag, attrs):  # noqa: ANN001
        a = dict(attrs)
        if tag == "input" and (a.get("type") or "").lower() == "range":
            self.ranges.append({**a, "_line": str(self.getpos()[0])})


def _json_pointer(doc, pointer: str):
    cur = doc
    for part in pointer.strip("/").split("/"):
        cur = cur[int(part)] if isinstance(cur, list) else cur[part]
    return cur


_NODE_FORMULAS = r"""
const fs = require("fs"), vm = require("vm");
const m = /\/\*\s*FORMULAS:BEGIN\s*\*\/([\s\S]*?)\/\*\s*FORMULAS:END\s*\*\//.exec(fs.readFileSync(process.argv[2], "utf8"));
const F = vm.runInNewContext(m[1] + "\n;FORMULAS", {});
const out = { classes: F.CLASSES, fields: F.CLASS_FIELDS, cases: [], derived: {} };
for (const c of F.CASES || []) {
  let v = null, err = null;
  try { v = F[c.js].apply(null, c.args); } catch (e) { err = String(e); }
  out.cases.push({ id: c.id, py: c.py, args: c.args, out: c.out || null, value: v, error: err });
}
for (const k of Object.keys(F.CLASSES || {})) out.derived[k] = F.derivedFor(k);
process.stdout.write(JSON.stringify(out));
"""


def _formulas(node: str, tmp_path: Path) -> dict:
    drv = tmp_path / "formulas_driver.js"
    drv.write_text(_NODE_FORMULAS, encoding="utf-8")
    r = subprocess.run([node, str(drv), str(DOCS / "assets" / "js" / "formulas.js")], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, f"node could not evaluate the FORMULAS block:\n{r.stderr[-2000:]}"
    return json.loads(r.stdout)


def test_sliders_match_presets(native, tmp_path):
    """Every range slider on a site page has a consistent min ≤ value ≤ max with a
    positive step; a slider that names a preset (``data-preset`` + ``data-path``) starts
    at that preset's value; and — under node — the FORMULAS.CLASSES inputs every page
    initialises its sliders from equal ``Params.preset(cls)`` at their JSON pointers."""
    bad = []
    n_sliders = 0
    for page in sorted(DOCS.glob("*.html")):
        p = _Inputs()
        p.feed(_read(page))
        for s in p.ranges:
            n_sliders += 1
            where = f"{page.name}:{s['_line']} ({s.get('data-param') or s.get('id') or '?'})"
            try:
                lo, hi = float(s.get("min") or 0), float(s.get("max") or 100)
                val = float(s.get("value") or (lo + hi) / 2)
                step = s.get("step")
                scale = float(s.get("data-scale") or 1)
            except ValueError as e:
                bad.append(f"{where}: non-numeric attribute ({e})")
                continue
            if not lo < hi:
                bad.append(f"{where}: min {lo} ≥ max {hi}")
            if not lo <= val <= hi:
                bad.append(f"{where}: value {val} outside [{lo}, {hi}]")
            if step not in (None, "any") and not float(step) > 0:
                bad.append(f"{where}: step {step} ≤ 0")
            if not scale > 0:
                bad.append(f"{where}: data-scale {scale} ≤ 0")
            cls, path = s.get("data-preset"), s.get("data-path")
            if cls and path:
                shown = 10**val if "data-log" in s else val
                want = float(native.Params.preset(cls).get(path))
                tol = (float(step) / scale if step not in (None, "any") else 0.0) / 2 + 1e-9 * abs(want)
                if abs(shown / scale - want) > tol:
                    bad.append(f"{where}: starts at {shown / scale} but preset {cls} {path} = {want}")
    node = _support.node_bin()
    if node is not None:
        out = _formulas(node, tmp_path)
        for cls, inputs in (out.get("classes") or {}).items():
            doc = json.loads(native.Params.preset(cls).to_json())
            for field, pointer in (out.get("fields") or {}).items():
                if field not in inputs:
                    bad.append(f"FORMULAS.CLASSES.{cls} lacks {field}")
                    continue
                want = _json_pointer(doc, pointer)
                if not _close(inputs[field], want, rel=1e-15):
                    bad.append(f"FORMULAS.CLASSES.{cls}.{field} = {inputs[field]!r} != preset {pointer} = {want!r}")
    print(f"{n_sliders} range sliders checked; FORMULAS.CLASSES {'checked' if node else 'not checked (no node)'}")
    assert not bad, "sliders vs presets:\n" + "\n".join(bad)


@pytest.mark.node
def test_formulas_match_rust(native, node_bin_path, tmp_path):
    """Under node, every FORMULAS.CASES row (the pages' JS fallback) equals the Rust
    closed form it mirrors (rel 1e-12: both are IEEE f64; only libm vs V8 elementary
    functions differ), and derivedFor(cls) equals Params.derived() on shared keys."""
    out = _formulas(node_bin_path, tmp_path)
    bad = []
    assert out["cases"], "FORMULAS.CASES is empty"
    for c in out["cases"]:
        tag = f"{c['id']}: JS vs _native.{c['py']}{tuple(c['args'])}"
        if c["error"]:
            bad.append(f"{tag}: JS raised {c['error']}")
            continue
        fn = getattr(native, c["py"] or "", None)
        if fn is None:
            bad.append(f"{tag}: no such native function")
            continue
        ref = fn(*c["args"])
        pairs = [((c["value"] or {}).get(jk), ref[nk]) for jk, nk in c["out"].items()] if c["out"] else [(c["value"], ref)]
        for js_v, rs_v in pairs:
            if not _close(js_v, rs_v, rel=1e-12):
                bad.append(f"{tag}: JS {js_v!r} vs Rust {rs_v!r}")
    for cls, d in (out.get("derived") or {}).items():
        ref = native.Params.preset(cls).derived()
        shared = set(d or {}) & set(ref)
        if not shared:
            bad.append(f"derivedFor({cls!r}) shares no key with Params.derived()")
        for k in sorted(shared):
            if not _close(d[k], ref[k], rel=1e-12):
                bad.append(f"derivedFor({cls!r}).{k} = {d[k]!r} vs Rust {ref[k]!r}")
    assert not bad, "FORMULAS vs Rust:\n" + "\n".join(bad)


def test_plotstyle_matches_css():
    """The site.css PLOTSTYLE block and fabrail.plotstyle.COLORS hold the same names and
    values in both directions, and the ``--class-*`` tokens map the classes to the
    series slots of plotstyle.CLASS_COLORS."""
    from fabrail import plotstyle

    css = _read(DOCS / "assets" / "css" / "site.css")
    tokens = plotstyle.css_palette(css)
    bad = list(plotstyle.palette_drift(css))
    for name in sorted(set(plotstyle.COLORS) - set(tokens)):
        bad.append(f"plotstyle.COLORS[{name!r}] has no --ps-{name} token")
    slot_of = {v.lower(): k for k, v in plotstyle.COLORS.items() if re.fullmatch(r"s\d", k)}
    for cls, color in plotstyle.CLASS_COLORS.items():
        m = re.search(rf"--class-{cls}\s*:\s*var\(--(s\d)\)", css)
        want = slot_of.get(color.lower())
        if not m:
            bad.append(f"site.css: --class-{cls} is not a var(--sN) series slot")
        elif want and m.group(1) != want:
            bad.append(f"--class-{cls} uses --{m.group(1)}, plotstyle.CLASS_COLORS[{cls!r}] is slot {want}")
    assert not bad, "plotstyle vs site.css:\n" + "\n".join(bad)


class _Resources(HTMLParser):
    """Resources an HTML page loads (not navigation links)."""

    LOADING = {
        "script": "src",
        "img": "src",
        "iframe": "src",
        "source": "src",
        "video": "src",
        "audio": "src",
        "embed": "src",
        "track": "src",
        "object": "data",
        "input": "src",
    }

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.urls: list[tuple[int, str]] = []
        self.inline: list[tuple[int, str]] = []
        self._in: str | None = None

    def handle_starttag(self, tag, attrs):  # noqa: ANN001
        a = dict(attrs)
        line = self.getpos()[0]
        attr = self.LOADING.get(tag)
        if attr and a.get(attr):
            self.urls.append((line, a[attr] or ""))
        if tag == "link" and a.get("href") and (a.get("rel") or "").lower() not in ("canonical", "alternate"):
            self.urls.append((line, a["href"] or ""))
        for k in ("srcset", "poster", "background"):
            if a.get(k):
                self.urls.append((line, a[k] or ""))
        if a.get("style"):
            self.inline.append((line, a["style"] or ""))
        self._in = tag if tag in ("script", "style") else None

    def handle_data(self, data):  # noqa: ANN001
        if self._in:
            self.inline.append((self.getpos()[0], data))

    def handle_endtag(self, tag):  # noqa: ANN001
        self._in = None


_EXTERNAL = re.compile(r"^\s*(?:https?:)?//", re.I)
_CODE_LOADS = re.compile(
    r"""(?:url\(\s*['"]?|@import\s+(?:url\()?\s*['"]?|fetch\(\s*['"`]|import\(\s*['"`]|new\s+(?:Worker|EventSource|WebSocket)\(\s*['"`]|\.src\s*=\s*['"`]|XMLHttpRequest[\s\S]{0,80}?open\([^,]*,\s*['"`])((?:https?:|wss?:)?//[^'"`)\s]+)""",
    re.I,
)


def test_no_external_resources():
    """No site page, stylesheet or script loads anything from another origin: script,
    img, iframe, media and link hrefs, CSS url()/@import, and JS fetch/import/Worker/
    WebSocket/src assignments are all relative (the site works from file:// and offline;
    plan §3b). Plain navigation links to references are allowed."""
    bad = []
    for page in sorted(DOCS.rglob("*.html")):
        p = _Resources()
        p.feed(_read(page))
        for line, url in p.urls:
            for part in url.split(","):
                if _EXTERNAL.match(part):
                    bad.append(f"{_support.rel(page)}:{line}: loads {part.strip()!r}")
        for line, text in p.inline:
            for m in _CODE_LOADS.finditer(text):
                bad.append(f"{_support.rel(page)}:~{line}: inline code loads {m.group(1)!r}")
    for f in sorted(list((DOCS / "assets").rglob("*.js")) + list((DOCS / "assets").rglob("*.css"))):
        for i, line in enumerate(_read(f).splitlines(), 1):
            for m in _CODE_LOADS.finditer(line):
                bad.append(f"{_support.rel(f)}:{i}: loads {m.group(1)!r}")
    assert not bad, "external resources on the site (plan §3b):\n" + "\n".join(bad)
