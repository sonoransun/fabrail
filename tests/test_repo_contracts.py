"""Repository contracts: publishing, ignores, licence, unsafe policy, vocabulary.

Contract:

- ``up_sh_excludes`` (FAB-SW-011): ``up.sh`` run against a synthetic tree that holds
  every excluded and every must-ship path, and against this repository, publishes
  neither CLAUDE.md, up.sh, target/, firmware/target/, .venv, output/, the root data/,
  compiled ``*.so`` nor caches, while docs/assets/data/, tests/data/ and
  python/fabrail/data/ (and nested files that merely share an excluded name) ship.
  Nothing is written into this repository; the destinations are temporary.
- ``gitignore_entries`` (FAB-SW-011): .gitignore carries the house entries (bytecode,
  venvs, output, Rust targets, the compiled extension, test caches, the root-anchored
  ``/data/``, OS files), and no pattern ignores a tracked data directory.
- ``forbid_unsafe`` (FAB-FW-002): every no_std crate and the std plant/sim crates forbid
  ``unsafe`` unconditionally; fabrail-exo-sim forbids it unless its PyO3 feature is on;
  fabrail-py writes no ``unsafe`` itself (PyO3 macros only); fabrail-web uses it only in
  ``#[unsafe(no_mangle)]`` export attributes; the firmware denies it outside
  firmware/src/bsp.
- Unmarked companions: ``license_mit`` (LICENSE is the MIT text; pyproject, every crate
  and the firmware declare MIT) and ``vocabulary_guard`` (plan §4b, §7: the banned words
  "perfect(ly)", "absurdly", "guarantee", "earthquake-proof", "no damage" and "protects"
  never appear in docs/**, the examples/out story files, the gallery or the example
  scripts outside quoted prompt blocks — Markdown blockquotes and inline code, HTML
  blockquote/code/pre; SPECIFICATION §6).
"""

from __future__ import annotations

import ast
import os
import re
import shutil
import subprocess
import tomllib
from html.parser import HTMLParser
from pathlib import Path

import pytest

import _support

ROOT = _support.ROOT

#: Excluded from publication (FAB-SW-011; up.sh header).
MUST_NOT_SHIP = (
    "CLAUDE.md",
    "up.sh",
    "target/debug/x.rlib",
    "firmware/target/thumbv7em-none-eabihf/x.elf",
    ".venv/bin/python",
    "output/run.json",
    "data/elcentro.at2",
    "python/fabrail/_native.cpython-314-darwin.so",
    "python/fabrail/__pycache__/x.cpython-314.pyc",
    ".pytest_cache/v/cache",
    ".hypothesis/examples/x",
    "crates/fabrail-web/target/x",
)
#: Must ship even though a component shares an excluded root name.
MUST_SHIP = (
    "README.md",
    "LICENSE",
    "docs/assets/data/run.js",
    "tests/data/readme_original.md",
    "python/fabrail/data/README.md",
    "docs/output/figure.png",
    "docs/CLAUDE.md",
    "crates/fabrail-core/src/lib.rs",
)


def _run_up(src_up: Path, dest: Path) -> subprocess.CompletedProcess:
    return subprocess.run(["bash", str(src_up), str(dest)], capture_output=True, text=True, timeout=300)


def _published(dest: Path) -> set[str]:
    return {p.relative_to(dest).as_posix() for p in dest.rglob("*") if p.is_file()}


@pytest.mark.req("FAB-SW-011")
def test_up_sh_excludes(tmp_path):
    """up.sh publishes neither CLAUDE.md, up.sh, target/, firmware/target/, .venv,
    output/, the root data/, *.so nor caches, and ships docs/assets/data/, tests/data/,
    python/fabrail/data/ and nested same-named files (FAB-SW-011): checked on a synthetic
    tree seeded with each case and on this repository (both into temporary targets)."""
    src = tmp_path / "src"
    for rel in MUST_NOT_SHIP + MUST_SHIP:
        p = src / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(rel, encoding="utf-8")
    shutil.copy2(ROOT / "up.sh", src / "up.sh")
    dest = tmp_path / "dest"
    dest.mkdir()
    r = _run_up(src / "up.sh", dest)
    assert r.returncode == 0, f"up.sh failed: {r.stderr}"
    got = _published(dest)
    leaked = [p for p in MUST_NOT_SHIP if p in got]
    lost = [p for p in MUST_SHIP if p not in got]
    assert not leaked, f"up.sh published excluded paths: {leaked} (FAB-SW-011)"
    assert not lost, f"up.sh dropped paths that must ship: {lost} (FAB-SW-011)"

    missing_dest = subprocess.run(["bash", str(ROOT / "up.sh"), str(tmp_path / "nope")], capture_output=True, text=True)
    assert missing_dest.returncode != 0, "up.sh must refuse a destination that does not exist"

    real = tmp_path / "real"
    real.mkdir()
    r = _run_up(ROOT / "up.sh", real)
    assert r.returncode == 0, f"up.sh on the repository failed: {r.stderr}"
    got = _published(real)
    for bad in ("CLAUDE.md", "up.sh"):
        assert bad not in got, f"{bad} was published"
    for prefix in ("target/", "firmware/target/", ".venv/", "output/", "data/"):
        assert not any(p.startswith(prefix) for p in got), f"{prefix} was published"
    assert not any(p.endswith(".so") or "/__pycache__/" in p for p in got), "compiled extension or bytecode published"
    for need in ("README.md", "LICENSE", "docs/SPECIFICATION.md", "docs/assets/data/run.js", "tests/data/readme_original.md"):
        assert need in got, f"{need} was not published"


def _gitignore_patterns() -> list[str]:
    lines = (ROOT / ".gitignore").read_text(encoding="utf-8").splitlines()
    return [ln.strip() for ln in lines if ln.strip() and not ln.lstrip().startswith("#")]


def _glob(name: str, pat: str) -> bool:
    """gitignore glob: ``*`` and ``?`` never cross a slash, ``**`` does, ``[...]`` classes."""
    rx, i = [], 0
    while i < len(pat):
        if pat.startswith("**", i):
            rx.append(".*")
            i += 2
        elif pat[i] == "*":
            rx.append("[^/]*")
            i += 1
        elif pat[i] == "?":
            rx.append("[^/]")
            i += 1
        elif pat[i] == "[":
            j = pat.find("]", i + 1)
            if j < 0:
                rx.append(re.escape(pat[i]))
                i += 1
            else:
                rx.append("[" + pat[i + 1 : j].replace("!", "^", 1) + "]")
                i = j + 1
        else:
            rx.append(re.escape(pat[i]))
            i += 1
    return re.fullmatch("".join(rx), name) is not None


def _ignored(path: str, patterns: list[str], is_dir: bool = False) -> bool:
    """Minimal gitignore semantics: anchored (leading or inner slash) vs basename patterns,
    trailing slash = directories (and everything below), negation with '!'."""
    parts = path.strip("/").split("/")
    result = False
    for pat in patterns:
        neg = pat.startswith("!")
        p = pat[1:] if neg else pat
        dir_only = p.endswith("/")
        p = p.rstrip("/")
        anchored = p.startswith("/") or "/" in p
        p = p.lstrip("/")
        hit = False
        for k in range(1, len(parts) + 1):
            prefix = "/".join(parts[:k])
            leaf_is_dir = k < len(parts) or is_dir
            if dir_only and not leaf_is_dir:
                continue
            hit = _glob(prefix, p) if anchored else _glob(parts[k - 1], p)
            if hit:
                break
        if hit:
            result = not neg
    return result


@pytest.mark.req("FAB-SW-011")
def test_gitignore_entries():
    """.gitignore ignores bytecode, venvs, output/, examples/out/, Rust targets, the
    compiled extension, test caches, the ROOT data/ (anchored) and OS files, and ignores
    none of docs/assets/data/, tests/data/, python/fabrail/data/ or the sources
    (FAB-SW-011)."""
    pats = _gitignore_patterns()
    must_ignore = [
        ("python/fabrail/__pycache__/x.pyc", False),
        ("tests/x.pyc", False),
        (".venv", True),
        ("output", True),
        ("target", True),
        ("firmware/target", True),
        ("python/fabrail/_native.cpython-314-darwin.so", False),
        (".pytest_cache", True),
        (".hypothesis", True),
        ("data", True),
        ("data/elcentro.at2", False),
        (".DS_Store", False),
    ]
    must_track = [
        ("docs/assets/data/run.js", False),
        ("docs/assets/data", True),
        ("tests/data/readme_original.md", False),
        ("tests/data", True),
        ("python/fabrail/data/README.md", False),
        ("python/fabrail/data", True),
        ("README.md", False),
        ("LICENSE", False),
        ("Cargo.lock", False),
        ("firmware/Cargo.lock", False),
        ("docs/assets/wasm/fabrail_web.wasm", False),
        ("crates/fabrail-core/src/lib.rs", False),
        ("up.sh", False),
    ]
    bad = [f"{p} is not ignored" for p, d in must_ignore if not _ignored(p, pats, d)]
    bad += [f"{p} would be ignored" for p, d in must_track if _ignored(p, pats, d)]
    if "/data/" not in pats and "/data" not in pats:
        bad.append("the root data/ entry must be anchored (/data/): an unanchored data/ ignores docs/assets/data")
    assert not bad, ".gitignore house entries (FAB-SW-011):\n" + "\n".join(bad)


def test_gitignore_matcher():
    """Control for the matcher above (gitignore semantics the checks rely on)."""
    assert _ignored("docs/assets/data/x.js", ["data/"])
    assert not _ignored("docs/assets/data/x.js", ["/data/"])
    assert _ignored("data/x", ["/data/"])
    assert _ignored("a/b/__pycache__/x.pyc", ["__pycache__/"])
    assert _ignored("python/fabrail/_x.so", ["python/fabrail/*.so"])
    assert not _ignored("python/fabrail/sub/_x.so", ["python/fabrail/*.so"])
    assert not _ignored("keep.txt", ["*.txt", "!keep.txt"])


def test_license_mit():
    """LICENSE is the MIT text, and pyproject, every crate manifest and the firmware
    declare MIT (core design §D.1 item 23)."""
    lic = (ROOT / "LICENSE").read_text(encoding="utf-8")
    assert lic.startswith("MIT License"), "LICENSE does not start with 'MIT License'"
    for phrase in (
        "Permission is hereby granted, free of charge",
        "The above copyright notice and this permission notice shall be included",
        'THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND',
    ):
        assert phrase in lic, f"LICENSE lacks the MIT clause {phrase!r}"
    py = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    assert py["project"]["license"] in ({"text": "MIT"}, "MIT"), py["project"]["license"]
    ws = tomllib.loads((ROOT / "Cargo.toml").read_text(encoding="utf-8"))
    assert ws["workspace"]["package"]["license"] == "MIT"
    for manifest in sorted((ROOT / "crates").glob("*/Cargo.toml")) + [ROOT / "firmware" / "Cargo.toml"]:
        pkg = tomllib.loads(manifest.read_text(encoding="utf-8"))["package"]
        lic_field = pkg.get("license")
        assert lic_field == "MIT" or lic_field == {"workspace": True}, f"{_support.rel(manifest)}: license {lic_field!r}"


# ---------------------------------------------------------------------------- FAB-FW-002


def _code(path: Path) -> str:
    text = re.sub(r"/\*.*?\*/", "", path.read_text(encoding="utf-8"), flags=re.S)
    return "\n".join(ln.split("//", 1)[0] for ln in text.splitlines())


@pytest.mark.req("FAB-FW-002")
def test_forbid_unsafe():
    """Unsafe code is forbidden in every shared and simulation crate, confined to PyO3
    macro expansion in fabrail-py, to ``#[unsafe(no_mangle)]`` exports in fabrail-web and
    to firmware/src/bsp in the firmware (FAB-FW-002)."""
    bad = []
    forbid_always = ("types", "params", "hal", "core", "safety", "rt", "exo-safety", "exo", "plant", "sim")
    for c in forbid_always:
        lib = (ROOT / "crates" / f"fabrail-{c}" / "src" / "lib.rs").read_text(encoding="utf-8")
        if not re.search(r"^\s*#!\[forbid\(unsafe_code\)\]", lib, re.M):
            bad.append(f"fabrail-{c}: no unconditional #![forbid(unsafe_code)]")
    exo_sim = (ROOT / "crates" / "fabrail-exo-sim" / "src" / "lib.rs").read_text(encoding="utf-8")
    if not re.search(r'#!\[cfg_attr\(not\(feature\s*=\s*"py"\),\s*forbid\(unsafe_code\)\)\]', exo_sim):
        bad.append('fabrail-exo-sim: no #![cfg_attr(not(feature = "py"), forbid(unsafe_code))]')
    for f in sorted((ROOT / "crates" / "fabrail-py" / "src").rglob("*.rs")):
        if re.search(r"\bunsafe\b", _code(f)):
            bad.append(f"{_support.rel(f)}: hand-written unsafe (only PyO3 macro expansion is allowed)")
    for f in sorted((ROOT / "crates" / "fabrail-web" / "src").rglob("*.rs")):
        for i, line in enumerate(_code(f).splitlines(), 1):
            rest = re.sub(r"#\[\s*unsafe\(\s*no_mangle\s*\)\s*\]", "", line)
            if re.search(r"\bunsafe\b", rest):
                bad.append(f"{_support.rel(f)}:{i}: unsafe beyond #[unsafe(no_mangle)]")
    fw = ROOT / "firmware" / "src"
    main_rs = (fw / "main.rs").read_text(encoding="utf-8")
    if "#![deny(unsafe_code)]" not in main_rs:
        bad.append("firmware/src/main.rs lacks #![deny(unsafe_code)]")
    for f in sorted(fw.rglob("*.rs")):
        rel = f.relative_to(fw).as_posix()
        if rel.startswith("bsp/"):
            continue
        for i, line in enumerate(_code(f).splitlines(), 1):
            if re.search(r"\bunsafe\b", line) and not re.search(r"(allow|deny|forbid)\(unsafe_code\)", line):
                bad.append(f"firmware/src/{rel}:{i}: unsafe outside bsp/")
    bsp_mod = (fw / "bsp" / "mod.rs").read_text(encoding="utf-8")
    if "allow(unsafe_code)" not in bsp_mod:
        bad.append("firmware/src/bsp/mod.rs: the unsafe allowance should be scoped to bsp/ explicitly")
    assert not bad, "unsafe policy (FAB-FW-002):\n" + "\n".join(bad)


# ---------------------------------------------------------------------------- vocabulary

BANNED = re.compile(r"\b(perfect(?:ly)?|absurdly|guarantee\w*|earthquake-proof|no damage|protects)\b", re.I)


class _HtmlText(HTMLParser):
    """Visible prose of an HTML page (outside blockquote/code/pre/kbd/samp/q), plus inline scripts."""

    QUOTED = {"blockquote", "code", "pre", "kbd", "samp", "q"}

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.stack: list[str] = []
        self.text: list[tuple[int, str]] = []
        self.scripts: list[tuple[int, str]] = []

    def handle_starttag(self, tag, attrs):  # noqa: ANN001
        if tag not in ("br", "img", "input", "meta", "link", "hr", "source", "wbr"):
            self.stack.append(tag)
        for k, v in attrs:
            if k in ("title", "alt", "aria-label", "content", "placeholder") and v and not (set(self.stack) & self.QUOTED):
                self.text.append((self.getpos()[0], v))

    def handle_endtag(self, tag):  # noqa: ANN001
        if tag in self.stack:
            while self.stack and self.stack.pop() != tag:
                pass

    def handle_data(self, data):  # noqa: ANN001
        line = self.getpos()[0]
        if self.stack and self.stack[-1] == "script":
            self.scripts.append((line, data))
        elif self.stack and self.stack[-1] == "style":
            return
        elif not (set(self.stack) & self.QUOTED):
            self.text.append((line, data))


def js_strings(src: str) -> list[tuple[int, str]]:
    """String and template literals of JavaScript source, with their line numbers.

    A small tokenizer: comments are skipped; a ``/`` after an operator or keyword starts
    a regular expression literal, which is skipped.
    """
    out: list[tuple[int, str]] = []
    i, n, line = 0, len(src), 1
    prev = ""  # last significant character
    while i < n:
        ch = src[i]
        if ch == "\n":
            line += 1
            i += 1
            continue
        if ch.isspace():
            i += 1
            continue
        if src.startswith("//", i):
            j = src.find("\n", i)
            i = n if j < 0 else j
            continue
        if src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            line += src.count("\n", i, j)
            i = j
            continue
        if ch in "'\"`":
            j, buf, start_line = i + 1, [], line
            while j < n and src[j] != ch:
                if src[j] == "\\" and j + 1 < n:
                    buf.append(src[j : j + 2])
                    j += 2
                    continue
                if src[j] == "\n":
                    line += 1
                buf.append(src[j])
                j += 1
            out.append((start_line, "".join(buf)))
            i = j + 1
            prev = ch
            continue
        if ch == "/" and (prev == "" or prev in "(,=:[!&|?{};+-*%<>~^" or src[max(0, i - 7) : i].rstrip().endswith("return")):
            j, in_class = i + 1, False
            while j < n and src[j] != "\n":
                if src[j] == "\\":
                    j += 2
                    continue
                if src[j] == "[":
                    in_class = True
                elif src[j] == "]":
                    in_class = False
                elif src[j] == "/" and not in_class:
                    break
                j += 1
            i = j + 1
            prev = "/"
            continue
        prev = ch
        i += 1
    return out


def _py_strings(path: Path) -> list[tuple[int, str]]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return [(n.lineno, n.value) for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)]


def _strip_inline_code(s: str) -> str:
    return re.sub(r"(`+)(?:(?!\1).)+?\1", " ", s)


def _prose_hits(where: str, items: list[tuple[int, str]]) -> list[str]:
    hits = []
    for line, text in items:
        for chunk in _strip_inline_code(text).splitlines():
            if chunk.lstrip().startswith(">"):
                continue  # a quoted prompt line inside a literal
            for m in BANNED.finditer(chunk):
                hits.append(f"{where}:{line}: {m.group(0)!r} in {chunk.strip()[:100]!r}")
    return hits


def vocabulary_hits(root: Path = ROOT) -> list[str]:
    """Every banned word outside quoted prompt blocks in the guarded files."""
    hits: list[str] = []
    docs = root / "docs"
    for f in sorted(docs.rglob("*.md")):
        for lineno, text in _support.md_prose(f.read_text(encoding="utf-8")):
            for m in BANNED.finditer(text):
                hits.append(f"{_support.rel(f)}:{lineno}: {m.group(0)!r} in {text.strip()[:100]!r}")
    for f in sorted(docs.rglob("*.html")):
        p = _HtmlText()
        p.feed(f.read_text(encoding="utf-8"))
        hits += _prose_hits(_support.rel(f), p.text)
        for line, src in p.scripts:
            hits += _prose_hits(f"{_support.rel(f)} <script>", [(line + k - 1, s) for k, s in js_strings(src)])
    for f in sorted((docs / "assets").rglob("*.js")):
        hits += _prose_hits(_support.rel(f), js_strings(f.read_text(encoding="utf-8")))
    out_dir = root / "examples" / "out"
    if out_dir.is_dir():
        for f in sorted(out_dir.rglob("*")):
            if f.suffix in (".md", ".txt"):
                for lineno, text in _support.md_prose(f.read_text(encoding="utf-8")):
                    for m in BANNED.finditer(text):
                        hits.append(f"{_support.rel(f)}:{lineno}: {m.group(0)!r}")
            elif f.suffix == ".json":
                import json as _json

                def walk(v, path=""):  # noqa: ANN001
                    if isinstance(v, str):
                        yield path, v
                    elif isinstance(v, dict):
                        for k, x in v.items():
                            yield from walk(x, f"{path}.{k}")
                    elif isinstance(v, list):
                        for k, x in enumerate(v):
                            yield from walk(x, f"{path}[{k}]")

                try:
                    doc = _json.loads(f.read_text(encoding="utf-8"))
                except ValueError:
                    continue
                for path, s in walk(doc):
                    hits += _prose_hits(f"{_support.rel(f)}{path}", [(0, s)])
    py_files = sorted((root / "python" / "fabrail" / "gallery").rglob("*.py"))
    py_files += sorted((root / "examples").glob("*.py")) if (root / "examples").is_dir() else []
    for f in py_files:
        hits += _prose_hits(_support.rel(f), _py_strings(f))
    return hits


def test_vocabulary_guard():
    """Outside quoted prompt blocks (Markdown blockquotes and inline code; HTML
    blockquote/code/pre), docs/**, the examples/out story files, the gallery package and
    the example scripts never say "perfect(ly)", "absurdly", "guarantee",
    "earthquake-proof", "no damage" or "protects" (plan §4b, §7; SPECIFICATION §6)."""
    hits = vocabulary_hits()
    assert not hits, "banned vocabulary outside quoted prompt blocks (plan §7):\n" + "\n".join(hits[:60])


def test_vocabulary_scanner(tmp_path):
    """Control for the guard: prose hits are found, quoted and code forms are not, and
    the JS tokenizer reads strings but skips comments and regex literals."""
    root = tmp_path
    (root / "docs" / "assets" / "js").mkdir(parents=True)
    (root / "docs" / "a.md").write_text(
        "The drawer closes absurdly well.\n> perfectly quoted\nUse `guarantee` as a word.\n", encoding="utf-8"
    )
    (root / "docs" / "p.html").write_text(
        "<p>This protects you.</p><blockquote>no damage</blockquote><code>perfect</code>"
        "<script>var s = 'earthquake-proof'; // perfect\n</script>",
        encoding="utf-8",
    )
    (root / "docs" / "assets" / "js" / "x.js").write_text(
        "/* guarantee */ var re = /perfect/; var t = `no damage here`; // absurdly\n", encoding="utf-8"
    )
    hits = vocabulary_hits(root)
    joined = "\n".join(hits)
    assert "absurdly" in joined and "protects" in joined and "earthquake-proof" in joined, hits
    assert sum("no damage" in h for h in hits) == 1 and "x.js" in joined, hits
    assert "perfectly" not in joined and "guarantee" not in joined, hits
    assert not any("perfect'" in h and "a.md" in h for h in hits)
    strings = [s for _, s in js_strings("var a = x / 2, b = 'y'; var r = /a'b/g; `c${d}`")]
    assert strings == ["y", "c${d}"], strings


def test_support_env_is_clean():
    """The suite runs headless (MPLBACKEND=Agg) with tests/ and python/ importable."""
    assert os.environ.get("MPLBACKEND") == "Agg"
    import fabrail

    assert Path(fabrail.__file__).resolve().is_relative_to(ROOT / "python")
