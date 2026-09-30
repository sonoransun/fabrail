"""The GitHub Pages site and its browser engine (plan §3b; FAB-SW-012, -013, -014).

Owner: W1 (the wasm/node checks and the page checks); C2c re-runs the file after the
Wave-3 site build.

What is checked:

- every page under ``docs/`` resolves its internal links, fragments and assets, and no
  site HTML/JS/CSS carries an external URL (the only exemption is the W3C XML
  namespace identifiers, which are names, never fetched);
- every top-level page carries the shared chrome: ``data-page``, the static navigation
  equal to ``nav.js`` ``PAGES``, the theme toggle with ``theme.js`` loaded synchronously
  in ``<head>``, reduced-motion CSS, the tier-badge legend, the selftest badge, and a
  frontier banner wherever frontier content appears;
- the CSS ``--ps-*`` tokens equal ``fabrail.plotstyle.COLORS`` and the ``--class-*``
  tokens use the plotstyle series slots;
- under node, the JS ``FORMULAS`` fallback (``assets/js/formulas.js``: its ``CASES``
  table, ``CLASSES`` and ``derivedFor``) matches the native Rust closed forms and
  presets, and the wasm analytic exports and anchors equal the native ones;
- the generated data blocks and ``read/`` pages are current (placeholders until the
  Wave-3 generators run);
- the wasm engine (``crates/fabrail-web``) is within its 1.5 MB budget, imports
  nothing, and its SHA-256 is recorded in ``ANCHORS.wasm`` once shipped;
- under node, the wasm selftest hash equals the native ``selftest_hash(2000)`` and the
  frozen ``GOLDEN_SELFTEST_HASH``, and a wasm step run matches the native ``run()``
  trace bit for bit over the first 2,000 ticks.

The wasm under test is ``$FABRAIL_WASM`` when set, else the shipped
``docs/assets/wasm/fabrail_web.wasm``, else the newest
``target/*/wasm32-unknown-unknown/release/fabrail_web.wasm`` (build command in
CLAUDE.md). Node-dependent tests carry the ``node`` marker and skip without node. A
comparison between two builds that fails while either build is older than the Rust
sources it depends on is reported as an expected failure naming the stale artifact:
the two builds are then of different source snapshots, which says nothing about
wasm/native determinism.
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys
from html.parser import HTMLParser
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
ENGINE_JS = DOCS / "assets" / "js" / "engine.js"
FORMULAS_JS = DOCS / "assets" / "js" / "formulas.js"
NAV_JS = DOCS / "assets" / "js" / "nav.js"
SITE_CSS = DOCS / "assets" / "css" / "site.css"
DATA_DIR = DOCS / "assets" / "data"
SHIPPED_WASM = DOCS / "assets" / "wasm" / "fabrail_web.wasm"
SELFTEST_RS = ROOT / "crates" / "fabrail-rt" / "src" / "selftest.rs"

#: Browser-engine size budget, bytes: "≤ 1.5 MB" (plan §3b; FAB-SW-014), read in SI
#: units so the check is the stricter of the two readings.
WASM_BUDGET_BYTES = 1_500_000

#: Ticks of the bit-exact wasm/native trace comparison (FAB-SW-014).
TRACE_TICKS = 2_000

#: Base tick rate, Hz (fabrail_types::BASE_TICK_HZ; PHYSICS §24).
BASE_TICK_HZ = 20_000

#: Selftest length, base ticks (fabrail_rt::selftest::SELFTEST_TICKS).
SELFTEST_TICKS = 2_000

#: Relative tolerance of JS FORMULAS against the Rust closed forms. Both evaluate the
#: same expressions in IEEE f64; only the elementary functions differ (V8's fdlibm
#: ports vs the Rust libm crate, each within 1 ulp), so agreement is ~1e-15. 1e-12
#: leaves margin for a few chained roundings while any formula drift (≥ 1e-6 for a
#: changed constant or term) still fails.
FORMULA_REL_TOL = 1e-12

#: Engine preset ids and plant tier ids (crates/fabrail-web/src/lib.rs ABI v2).
PRESET_IDS = {"drawer": 0, "door": 1, "wall": 2, "heavy": 3}
TIER_IDS = {"carriage6": 0, "heave1": 1, "travel1d": 2}
#: Command codes (engine.js COMMANDS; fabrail-web ``cmd``).
CMD = {"none": 0, "open": 1, "close": 2, "stop": 3, "lift": 4, "land": 5, "reset": 6}

#: State-buffer fields (engine.js STATE_FIELDS, 20 LE f64).
STATE_FIELDS = (
    "tick", "t_s", "x_m", "v_mps", "a_mps2", "gap_m", "coil_i_a", "iq_a",
    "f_lsm_n", "f_hand_n", "f_hand_hat_n", "m_hat_kg", "e_kin_gov_j",
    "lev_state", "motion_mode", "seismic_state", "seismic_level", "safety_demand",
    "fault_bits_lo", "floor_ax_mps2",
)  # fmt: skip

#: W3C XML namespace identifiers: names in xmlns attributes and createElementNS calls,
#: never fetched, so they are not external resources.
XML_NAMESPACES = (
    "http://www.w3.org/2000/svg",
    "http://www.w3.org/1999/xlink",
    "http://www.w3.org/1999/xhtml",
    "http://www.w3.org/XML/1998/namespace",
    "http://www.w3.org/1998/Math/MathML",
)

#: Rust crates the browser engine and the native module compile from (staleness).
ENGINE_CRATES = ("types", "params", "hal", "core", "safety", "rt", "plant", "sim")


# ---------------------------------------------------------------------------- helpers


def _site_pages() -> list[Path]:
    """Top-level site pages (docs/*.html)."""
    return sorted(DOCS.glob("*.html"))


def _all_html() -> list[Path]:
    """Every HTML file of the site, including generated read/ pages."""
    return sorted(p for p in DOCS.rglob("*.html") if p.is_file())


def _site_text_files() -> list[Path]:
    """Every HTML, JS and CSS file under docs/."""
    return sorted(p for p in DOCS.rglob("*") if p.is_file() and p.suffix in {".html", ".js", ".css"})


def _rel(p: Path) -> str:
    return p.relative_to(ROOT).as_posix()


def _classes(attrs: dict[str, str | None]) -> set[str]:
    return set((attrs.get("class") or "").split())


class _Page(HTMLParser):
    """What the checks need from one HTML page (stdlib parser; no DOM)."""

    #: (tag, attribute) pairs whose value is a URL the browser may fetch or navigate.
    URL_ATTRS = {
        "href", "src", "poster", "data", "action", "formaction", "xlink:href", "background",
    }  # fmt: skip

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.refs: list[tuple[str, str, int]] = []  # (attribute, value, line)
        self.ids: set[str] = set()
        self.body_attrs: dict[str, str | None] | None = None
        self.nav_links: list[tuple[str, str]] = []
        self.nav_found = False
        self.theme_toggle = False
        self.selftest_in_footer = False
        self.tier_legend = False
        self.frontier_banner = False
        self.frontier_outside_legend = False
        self.styles: list[str] = []
        self.stylesheets: list[str] = []
        self.head_scripts: list[tuple[str, bool]] = []  # (src, synchronous)
        self._open: dict[str, list[str]] = {}  # region -> [tag, depth]
        self._depth: dict[str, int] = {}
        self._in_style = False
        self._nav_href: str | None = None
        self._nav_text: list[str] = []
        self._in_head = False

    # Region tracking by tag-name depth (robust to implicitly closed elements).
    def _enter(self, region: str, tag: str) -> None:
        if region not in self._open:
            self._open[region] = [tag]
            self._depth[region] = 0

    def _inside(self, region: str) -> bool:
        return region in self._open

    def handle_starttag(self, tag: str, attrs_list: list[tuple[str, str | None]]) -> None:
        attrs = dict(attrs_list)
        line = self.getpos()[0]
        for region, (rtag,) in list(self._open.items()):
            if tag == rtag:
                self._depth[region] += 1
        cls = _classes(attrs)
        if tag == "head":
            self._in_head = True
        if tag == "body":
            self.body_attrs = attrs
        if attrs.get("id"):
            self.ids.add(attrs["id"] or "")
        if tag == "a" and attrs.get("name"):
            self.ids.add(attrs["name"] or "")
        for name, value in attrs_list:
            if value is None:
                continue
            if name in self.URL_ATTRS:
                self.refs.append((f"{tag}[{name}]", value.strip(), line))
            elif name == "srcset":
                for part in value.split(","):
                    url = part.strip().split(" ")[0]
                    if url:
                        self.refs.append((f"{tag}[srcset]", url, line))
            elif name == "style":
                for url in _css_urls(value):
                    self.refs.append((f"{tag}[style] url()", url, line))
            elif name == "content" and (attrs.get("http-equiv") or "").lower() == "refresh":
                m = re.search(r"url\s*=\s*(\S+)", value, re.I)
                if m:
                    self.refs.append(("meta[refresh]", m.group(1).strip("'\""), line))
        if tag == "link" and "stylesheet" in (attrs.get("rel") or "").split():
            self.stylesheets.append(attrs.get("href") or "")
        if tag == "script" and self._in_head and attrs.get("src"):
            sync = "async" not in attrs and "defer" not in attrs and attrs.get("type") != "module"
            self.head_scripts.append((attrs.get("src") or "", sync))
        if tag == "style":
            self._in_style = True
        if tag == "nav" and "data-site-nav" in attrs:
            self.nav_found = True
            self._enter("nav", tag)
        if tag == "footer":
            self._enter("footer", tag)
        if "tier-legend" in cls:
            self.tier_legend = True
            self._enter("legend", tag)
        if "data-theme-toggle" in attrs:
            self.theme_toggle = True
        if "data-selftest" in attrs and self._inside("footer"):
            self.selftest_in_footer = True
        if "banner" in cls and "frontier" in cls:
            self.frontier_banner = True
        if "badge--f" in cls and not self._inside("legend") and not self._inside("flink"):
            self.frontier_outside_legend = True
        if tag == "a" and self._inside("nav"):
            self._nav_href = attrs.get("href") or ""
            self._nav_text = []
        if tag == "a" and re.search(r"(^|/)frontier\.html(#|$)", attrs.get("href") or ""):
            self._enter("flink", tag)

    def handle_startendtag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self.handle_starttag(tag, attrs)
        self.handle_endtag(tag)

    def handle_endtag(self, tag: str) -> None:
        if tag == "head":
            self._in_head = False
        if tag == "style":
            self._in_style = False
        if tag == "a" and self._nav_href is not None:
            self.nav_links.append((self._nav_href, " ".join("".join(self._nav_text).split())))
            self._nav_href = None
        for region, (rtag,) in list(self._open.items()):
            if tag == rtag:
                if self._depth[region] == 0:
                    del self._open[region]
                    del self._depth[region]
                else:
                    self._depth[region] -= 1

    def handle_data(self, data: str) -> None:
        if self._in_style:
            self.styles.append(data)
        if self._nav_href is not None:
            self._nav_text.append(data)
        if "\N{TELESCOPE}" in data and not any(self._inside(r) for r in ("legend", "nav", "flink")):
            self.frontier_outside_legend = True


_PAGE_CACHE: dict[Path, _Page] = {}


def _parse(path: Path) -> _Page:
    if path not in _PAGE_CACHE:
        p = _Page()
        p.feed(path.read_text(encoding="utf-8"))
        p.close()
        _PAGE_CACHE[path] = p
    return _PAGE_CACHE[path]


def _css_urls(css: str) -> list[str]:
    """URLs referenced by ``url(...)`` and ``@import`` in CSS text (comments removed)."""
    css = re.sub(r"/\*.*?\*/", "", css, flags=re.S)
    urls = [m.group(2).strip() for m in re.finditer(r"url\(\s*(['\"]?)(.*?)\1\s*\)", css)]
    urls += [m.group(2) for m in re.finditer(r"@import\s+(['\"])(.*?)\1", css)]
    return [u for u in urls if u]


def _is_external(url: str) -> bool:
    return bool(re.match(r"^(?:[a-z][a-z0-9+.-]*:)?//", url, re.I))


def _is_non_file(url: str) -> bool:
    """Schemes and forms that are not files of the site (checked elsewhere or inert)."""
    return bool(re.match(r"^(?:mailto|tel|data|blob|javascript|about):", url, re.I))


def _resolve(base: Path, url: str) -> tuple[Path | None, str]:
    """(file the URL names, fragment); file None for a same-page fragment."""
    path, _, frag = url.partition("#")
    path = path.split("?", 1)[0]
    if not path:
        return None, frag
    from urllib.parse import unquote

    target = (base.parent / unquote(path)).resolve()
    if target.is_dir():
        target = target / "index.html"
    return target, frag


def _golden() -> int:
    """GOLDEN_SELFTEST_HASH from the Rust source (0 until C1 freezes it)."""
    m = re.search(
        r"GOLDEN_SELFTEST_HASH\s*:\s*u64\s*=\s*(0x[0-9a-fA-F_]+|[0-9_]+)\s*;",
        SELFTEST_RS.read_text(encoding="utf-8"),
    )
    assert m, f"{_rel(SELFTEST_RS)} defines GOLDEN_SELFTEST_HASH"
    return int(m.group(1).replace("_", ""), 0)


def _built_wasms() -> list[Path]:
    """Built fabrail_web.wasm artifacts under target/, newest first."""
    found = [
        p
        for p in (ROOT / "target").glob("*/wasm32-unknown-unknown/release/fabrail_web.wasm")
        if p.is_file()
    ]
    return sorted(found, key=lambda p: p.stat().st_mtime, reverse=True)


def _wasm_under_test() -> Path | None:
    env = os.environ.get("FABRAIL_WASM")
    if env:
        p = Path(env)
        assert p.is_file(), f"FABRAIL_WASM={env} does not exist"
        return p
    if SHIPPED_WASM.is_file():
        return SHIPPED_WASM
    built = _built_wasms()
    return built[0] if built else None


@pytest.fixture(scope="module")
def wasm_path() -> Path:
    p = _wasm_under_test()
    if p is None:
        pytest.skip(
            "no fabrail_web.wasm: build it with `RUSTC=$(rustup which rustc --toolchain stable) "
            "rustup run stable cargo build -p fabrail-web --target wasm32-unknown-unknown --release` "
            "(CLAUDE.md), or set FABRAIL_WASM"
        )
    return p


@pytest.fixture(scope="module")
def node_bin() -> str:
    node = shutil.which("node")
    if node is None:
        pytest.skip("node is not installed")
    return node


def _stale_sources(artifact: Path, crates: tuple[str, ...]) -> list[str]:
    """Rust sources (src/**, Cargo.toml) of ``crates`` newer than ``artifact``."""
    t = artifact.stat().st_mtime
    newer = []
    for c in crates:
        d = ROOT / "crates" / f"fabrail-{c}"
        for f in [d / "Cargo.toml", *d.glob("src/**/*.rs")]:
            if f.is_file() and f.stat().st_mtime > t:
                newer.append(_rel(f))
    return newer


def _staleness(wasm: Path, native) -> str:
    """Why two builds may be of different source snapshots ('' when both are current)."""
    notes = []
    w = _stale_sources(wasm, (*ENGINE_CRATES, "web"))
    if w:
        name = _rel(wasm) if wasm.is_relative_to(ROOT) else str(wasm)
        notes.append(f"{name} is older than {len(w)} source(s), e.g. {w[0]}: rebuild the wasm")
    if native is not None:
        n = _stale_sources(Path(native.__file__), (*ENGINE_CRATES, "py"))
        if n:
            notes.append(
                f"the native module is older than {len(n)} source(s), e.g. {n[0]}: "
                "rerun maturin develop --release"
            )
    return "; ".join(notes)


def _native_or_none():
    try:
        from fabrail import _native
    except ImportError:
        return None
    return _native


# ---------------------------------------------------------------------------- node driver

_NODE_DRIVER = r"""
"use strict";
const fs = require("fs");
const vm = require("vm");
const [mode, cfgPath] = process.argv.slice(2);
const cfg = JSON.parse(fs.readFileSync(cfgPath, "utf8"));

function instantiate(path) {
  const mod = new WebAssembly.Module(fs.readFileSync(path));
  return { mod: mod, ex: new WebAssembly.Instance(mod, {}).exports };
}
function text(ex, len) {
  return Buffer.from(new Uint8Array(ex.memory.buffer, ex.fr_text_ptr() >>> 0, len)).toString("utf8");
}

async function selftest() {
  // The page path: engine.js loads the same bytes from window.FABRAIL_WASM_B64.
  globalThis.FABRAIL_WASM_B64 = fs.readFileSync(cfg.wasm).toString("base64");
  const E = require(cfg.engine_js);
  const out = { load: await E.load() };
  out.hex = E.selftestHex();
  const w = instantiate(cfg.wasm);
  out.imports = WebAssembly.Module.imports(w.mod).map((i) => i.module + "." + i.name);
  out.exports = WebAssembly.Module.exports(w.mod).map((e) => e.name);
  out.abi = w.ex.fabrail_abi_version();
  out.lo = w.ex.fr_selftest_hash_lo() >>> 0;
  out.hi = w.ex.fr_selftest_hash_hi() >>> 0;
  // The frozen JS API end to end on the page engine.
  out.create = E.create("door");
  E.setInputs({ handN: 0, floorAx: 0, command: "open" });
  out.stepped = E.step(200);
  out.state = E.state();
  return out;
}

function trace() {
  const ex = instantiate(cfg.wasm).ex;
  const out = [];
  for (const c of cfg.cases) {
    const h = ex.fr_create_tier(c.preset, c.tier);
    if (h === 0) {
      const n = ex.fr_last_error();
      throw new Error("fr_create_tier(" + c.preset + ", " + c.tier + "): " + (n ? text(ex, n) : "?"));
    }
    const scenario = text(ex, ex.fr_scenario_json(c.preset, c.tier));
    ex.fr_set_inputs(h, c.hand_n, c.floor[0], c.floor[1], c.floor[2], c.cmd);
    const len = ex.fr_state_len();
    const buf = new Float64Array((c.ticks + 1) * len);
    const read = (k) => buf.set(new Float64Array(ex.memory.buffer, ex.fr_state_ptr(h) >>> 0, len), k * len);
    read(0);
    for (let k = 1; k <= c.ticks; k++) {
      if (ex.fr_step(h, 1) !== 1) {
        const n = ex.fr_last_error();
        throw new Error("fr_step failed at tick " + k + ": " + (n ? text(ex, n) : "?"));
      }
      read(k);
    }
    out.push({ scenario: scenario, len: len, status: ex.fr_status(h),
               states: Buffer.from(buf.buffer).toString("base64") });
    ex.fr_destroy(h);
  }
  return out;
}

function formulas() {
  const m = /\/\*\s*FORMULAS:BEGIN\s*\*\/([\s\S]*?)\/\*\s*FORMULAS:END\s*\*\//.exec(
    fs.readFileSync(cfg.formulas_js, "utf8"));
  if (!m) throw new Error(cfg.formulas_js + " has no /* FORMULAS:BEGIN */ … /* FORMULAS:END */ block");
  // The block must be self-contained: evaluate it alone in a fresh context.
  const F = vm.runInNewContext(m[1] + "\n;FORMULAS", {});
  const CASES = F.CASES || [];
  const out = { functions: {}, cases: [], classes: F.CLASSES || null,
                class_fields: F.CLASS_FIELDS || null, derived: {}, extra: {},
                wasm: null, anchors: null };
  for (const k of Object.keys(F)) {
    if (typeof F[k] !== "function") continue;
    out.functions[k] = CASES.some((c) => c.js === k || F[c.js] === F[k]);
  }
  for (const c of CASES) {
    const r = { id: c.id, js: c.js, py: c.py, args: c.args, out: c.out || null,
                expect: c.expect === undefined ? null : c.expect,
                tol: c.tol === undefined ? null : c.tol, value: null, error: null };
    try { r.value = F[c.js].apply(null, c.args); } catch (e) { r.error = String(e); }
    out.cases.push(r);
  }
  if (typeof F.derivedFor === "function" && F.CLASSES) {
    for (const cls of Object.keys(F.CLASSES)) out.derived[cls] = F.derivedFor(cls);
  }
  for (const c of cfg.extra) out.extra[c.id] = F[c.fn] ? F[c.fn].apply(null, c.args) : null;
  if (cfg.wasm) {
    const ex = instantiate(cfg.wasm).ex;
    out.wasm = {};
    for (const c of cfg.wasm_cases) out.wasm[c.id] = ex[c.fn].apply(null, c.args);
    out.anchors = [];
    for (let p = 0; p < 4; p++) {
      for (let k = 0; k < 64; k++) {
        const id = 64 * p + k;
        const n = ex.fr_anchor_name(id);
        if (n) out.anchors.push({ preset: p, name: text(ex, n), value: ex.fabrail_anchor(id) });
      }
    }
  }
  return out;
}

const modes = { selftest: selftest, trace: trace, formulas: formulas };
Promise.resolve()
  .then(() => modes[mode]())
  .then((r) => process.stdout.write(JSON.stringify(r)))
  .catch((e) => { process.stderr.write(String(e && e.stack ? e.stack : e)); process.exit(1); });
"""


def _node(node_bin: str, tmp_path: Path, mode: str, cfg: dict) -> object:
    driver = tmp_path / "site_driver.js"
    driver.write_text(_NODE_DRIVER, encoding="utf-8")
    cfg_path = tmp_path / f"{mode}.json"
    cfg_path.write_text(json.dumps(cfg), encoding="utf-8")
    r = subprocess.run(
        [node_bin, str(driver), mode, str(cfg_path)],
        capture_output=True,
        text=True,
        timeout=120,
        cwd=ROOT,
    )
    assert r.returncode == 0, f"node {mode} failed:\n{r.stderr[-4000:]}"
    return json.loads(r.stdout)


def _data_block(path: Path, name: str) -> dict:
    """The JSON object between ``/* NAME:BEGIN */`` and ``/* NAME:END */``."""
    text = path.read_text(encoding="utf-8")
    m = re.search(
        rf"/\*\s*{name}:BEGIN\s*\*/\s*var\s+{name}\s*=\s*(\{{.*\}})\s*;\s*/\*\s*{name}:END\s*\*/",
        text,
        re.S,
    )
    assert m, f"{_rel(path)}: no `/* {name}:BEGIN */ var {name} = {{…}}; /* {name}:END */` block"
    try:
        return json.loads(m.group(1))
    except json.JSONDecodeError as e:
        pytest.fail(f"{_rel(path)}: the {name} block is not JSON ({e})")


def _anchors_block() -> dict:
    return _data_block(DATA_DIR / "anchors.js", "ANCHORS")


# ---------------------------------------------------------------------------- FAB-SW-012


@pytest.mark.req("FAB-SW-012")
def test_links_and_assets_resolve():
    """Every internal href/src/url() of every site page names an existing file inside
    docs/, and every fragment names an id of its target page (FAB-SW-012)."""
    pages = _all_html()
    assert pages, "docs/ has HTML pages"
    docs = DOCS.resolve()
    bad: list[str] = []
    checked = 0
    for page in pages:
        p = _parse(page)
        refs = list(p.refs) + [("style url()", u, 0) for s in p.styles for u in _css_urls(s)]
        for where, url, line in refs:
            if not url or _is_external(url) or _is_non_file(url):
                continue  # external URLs fail test_no_external_urls
            target, frag = _resolve(page, url)
            loc = f"{_rel(page)}:{line} {where}={url!r}"
            if target is None:
                if frag and frag not in p.ids:
                    bad.append(f"{loc}: no id {frag!r} on this page")
                checked += 1
                continue
            if not target.is_relative_to(docs):
                bad.append(f"{loc}: leaves docs/ (the published root)")
            elif not target.is_file():
                bad.append(f"{loc}: {target.relative_to(docs)} does not exist")
            elif frag and target.suffix == ".html" and frag not in _parse(target).ids:
                bad.append(f"{loc}: no id {frag!r} in {target.relative_to(docs)}")
            checked += 1
    for css in sorted(DOCS.rglob("*.css")):
        for url in _css_urls(css.read_text(encoding="utf-8")):
            if _is_external(url) or _is_non_file(url):
                continue
            target, _ = _resolve(css, url)
            if target is not None and not target.is_file():
                bad.append(f"{_rel(css)} url({url!r}): does not exist")
            checked += 1
    assert checked > 0, "no internal references found"
    assert not bad, "unresolved internal references (plan §3b; FAB-SW-012):\n" + "\n".join(bad)


@pytest.mark.req("FAB-SW-012")
def test_no_external_urls():
    """No site HTML, JS or CSS carries an external URL: the site works from file:// and
    GitHub Pages with no third-party request (plan §3b; FAB-SW-012). W3C XML namespace
    identifiers (xmlns values, createElementNS) are names, not resources."""
    files = _site_text_files()
    assert files, "docs/ has HTML/JS/CSS files"
    url_re = re.compile(r"""(?i)\b(?:https?|ftps?|wss?)://[^\s"'<>)\\]*""")
    rel_re = re.compile(
        r"""(?ix)
        (?:\b(?:src|href|action|poster|data|srcset|xlink:href)\s*=\s*["']?\s*//[^/\s"'>]
          | url\(\s*["']?\s*//[^/\s"')]
          | @import\s+["']//[^/\s"']
          | \b(?:fetch|import|importScripts)\s*\(\s*["']//[^/\s"']
        )"""
    )
    bad: list[str] = []
    for f in files:
        for n, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            for m in url_re.finditer(line):
                url = m.group(0).rstrip(".,;")
                if url.rstrip("/") in XML_NAMESPACES:
                    continue
                bad.append(f"{_rel(f)}:{n}: {url}")
            for m in rel_re.finditer(line):
                bad.append(f"{_rel(f)}:{n}: protocol-relative URL {m.group(0).strip()!r}")
    assert not bad, (
        "external URLs in the site (plan §3b: no external URLs or JS libraries; FAB-SW-012):\n"
        + "\n".join(bad)
    )


def _nav_pages() -> list[tuple[str, str, str]]:
    """nav.js PAGES as (key, href, label)."""
    js = NAV_JS.read_text(encoding="utf-8")
    body = js[js.index("var PAGES = [") :]
    body = body[: body.index("];")]
    pages = re.findall(r'key:\s*"([^"]+)",\s*href:\s*"([^"]+)",\s*label:\s*"([^"]+)"', body)
    assert pages, f"{_rel(NAV_JS)} declares PAGES"
    return pages


def _page_css(page: Path, p: _Page) -> str:
    """Inline styles plus the linked local stylesheets of a page."""
    css = list(p.styles)
    for href in p.stylesheets:
        target, _ = _resolve(page, href)
        if target is not None and target.is_file():
            css.append(target.read_text(encoding="utf-8"))
    return "\n".join(css)


@pytest.mark.req("FAB-SW-012")
def test_page_chrome():
    """Every top-level page carries the shared chrome (W0 site contract; FAB-SW-012):
    ``<body data-page>`` naming the page, the static nav links equal to nav.js PAGES,
    the theme toggle with theme.js loaded synchronously in <head>, reduced-motion CSS,
    the tier-badge legend, the footer selftest badge, and a frontier banner on
    frontier.html and on any page with frontier (🔭) content outside the legend (a 🔭
    badge on a card linking to frontier.html labels a pointer, not content)."""
    nav = _nav_pages()
    keys = [k for k, _, _ in nav]
    page_files = _site_pages()
    stems = [p.stem for p in page_files]
    assert sorted(stems) == sorted(keys), (
        f"docs/*.html {sorted(stems)} must equal nav.js PAGES {sorted(keys)}"
    )
    for _, href, _ in nav:
        assert (DOCS / href).is_file(), f"nav.js PAGES href {href} exists"
    bad: list[str] = []
    for page in page_files:
        p = _parse(page)
        name = _rel(page)
        if p.body_attrs is None or p.body_attrs.get("data-page") != page.stem:
            bad.append(f"{name}: <body data-page={page.stem!r}> missing")
        if not p.nav_found:
            bad.append(f"{name}: no <nav data-site-nav>")
        else:
            want = [(href, label) for _, href, label in nav]
            if p.nav_links != want:
                bad.append(f"{name}: static nav links {p.nav_links} != nav.js PAGES {want}")
        if not p.theme_toggle:
            bad.append(f"{name}: no [data-theme-toggle] control")
        theme = [(src, sync) for src, sync in p.head_scripts if src.endswith("theme.js")]
        if not theme or not all(sync for _, sync in theme):
            bad.append(f"{name}: theme.js must load synchronously in <head> (no flash of the wrong theme)")
        css = _page_css(page, p)
        if not re.search(r"@media\s*\(\s*prefers-reduced-motion\s*:\s*reduce\s*\)", css):
            bad.append(f"{name}: no prefers-reduced-motion: reduce CSS")
        if not p.tier_legend:
            bad.append(f"{name}: no .tier-legend")
        if not p.selftest_in_footer:
            bad.append(f"{name}: no [data-selftest] badge in the footer")
        needs_banner = page.stem == "frontier" or p.frontier_outside_legend
        if needs_banner and not p.frontier_banner:
            bad.append(f"{name}: frontier content without a .banner.frontier")
    assert not bad, "site chrome (plan §3b; FAB-SW-012):\n" + "\n".join(bad)


# ---------------------------------------------------------------------------- FAB-SW-013


@pytest.mark.req("FAB-SW-013")
def test_css_tokens_match_plotstyle():
    """site.css ``--ps-*`` tokens equal plotstyle.COLORS, and the ``--class-*`` tokens use
    the series slots plotstyle.CLASS_COLORS assigns (FAB-SW-013)."""
    from fabrail import plotstyle

    css = SITE_CSS.read_text(encoding="utf-8")
    m = re.search(r"/\*\s*PLOTSTYLE:BEGIN(.*?)PLOTSTYLE:END\s*\*/", css, re.S)
    assert m, f"{_rel(SITE_CSS)} has a PLOTSTYLE:BEGIN … PLOTSTYLE:END block"
    tokens = dict(re.findall(r"--ps-([\w-]+)\s*:\s*([^;]+);", m.group(1)))
    assert tokens, "the PLOTSTYLE block declares --ps-* tokens"
    expected = ("ink", "ink2", "muted", "grid", "surface", "page", "s1", "s2", "s3", "s4", "s5")
    missing = [k for k in expected if k not in tokens]
    assert not missing, f"PLOTSTYLE block lacks --ps-{missing}"
    diff = {
        k: (v.strip().lower(), plotstyle.COLORS.get(k, "<absent>").lower())
        for k, v in tokens.items()
        if v.strip().lower() != plotstyle.COLORS.get(k, "").lower()
    }
    assert not diff, f"--ps-* != plotstyle.COLORS (css, python): {diff}"
    slot_of = {plotstyle.COLORS[f"s{i}"].lower(): f"s{i}" for i in range(1, 6)}
    for cls, color in plotstyle.CLASS_COLORS.items():
        m2 = re.search(rf"--class-{cls}\s*:\s*var\(\s*--(s\d)\s*\)", css)
        assert m2, f"site.css declares --class-{cls}: var(--sN)"
        assert slot_of.get(color.lower()) == m2.group(1), (
            f"--class-{cls} uses --{m2.group(1)} but plotstyle.CLASS_COLORS[{cls!r}] = {color} "
            f"is slot {slot_of.get(color.lower())}"
        )


#: FORMULAS helpers that are not mirrors of a derive:: closed form (time profiles for
#: plots and animation, and the class-table evaluator checked through its outputs).
_FORMULA_HELPERS = frozenset({"derivedFor", "jerkPath", "scurve", "terminusProfile", "closeProfile"})


def _composite_checks(n) -> list[tuple[str, str, list, float]]:
    """(id, FORMULAS function, args, native value) for mirrors whose Rust function has no
    Python binding: derive::unsensed_speed_cap = min(v_low_energy, momentum_cap) with
    the pedestrian caps (PHYSICS §16)."""
    return [
        (f"unsensed_{m}", "unsensedSpeedCap", [m], min(n.v_low_energy(m), n.momentum_speed_cap(m)))
        for m in (8.0, 48.0, 100.0, 500.0, 5000.0)
    ]


def _json_pointer(doc, pointer: str):
    for part in pointer.strip("/").split("/"):
        doc = doc[int(part)] if isinstance(doc, list) else doc[part]
    return doc


def _wasm_analytic_cases(n) -> list[tuple[str, str, list, float]]:
    """(id, wasm export, args, native value) for the analytic fr_* exports."""
    b = n.terminus_blend(0.184, 1.0, 0.25)
    return [
        ("open_loop_pole", "fr_open_loop_pole", [0.005, 0.012], n.open_loop_pole(0.005, 0.012)),
        ("zero_power_mass", "fr_zero_power_mass", [0.0046, 0.005, 0.012, 100.0],
         n.zero_power_mass(0.0046, 0.005, 0.012, 100.0)),
        ("v_low_energy", "fr_v_low_energy", [48.0, 1.69], n.v_low_energy(48.0, 1.69)),
        ("momentum_cap", "fr_momentum_cap", [500.0, 22.3], n.momentum_speed_cap(500.0, 22.3)),
        ("stop_distance", "fr_stop_distance", [0.3, 0.5, 2.0], n.stop_distance(0.3, 0.5, 2.0)),
        ("clamp_force", "fr_clamp_force", [1.69, 25e3], n.clamp_force(1.69, 25e3)),
        ("series_k", "fr_series_k", [75e3, 10e3], n.series_stiffness(75e3, 10e3)),
        ("unattended", "fr_unattended_close_speed", [0.2654, 150.0, 48.0, 8.82e3],
         n.unattended_close_speed(0.2654, 150.0, 48.0, 8.82e3)),
        ("t_arrive", "fr_terminus_t_arrive", [0.184, 1.0, 0.25, 2e-4],
         n.terminus_t_arrive(0.184, 1.0, 0.25, 2e-4)),
        ("d0", "fr_terminus_d0", [0.184, 1.0, 0.25], b["d_0_m"]),
        ("db", "fr_terminus_db", [0.184, 1.0, 0.25], b["d_b_m"]),
        ("tb", "fr_terminus_tb", [0.184, 1.0, 0.25], b["t_b_s"]),
        ("sp_time", "fr_sp_time", [61.2, 6.0, 3.5], n.sp_time(61.2, 6.0, 3.5)),
        ("transmissibility", "fr_transmissibility", [3.0, 0.2], n.transmissibility(3.0, 0.2)),
        ("t_sky", "fr_t_sky", [2.0, 0.7], n.skyhook_transmissibility(2.0, 0.7)),
        ("den_hartog_f", "fr_den_hartog_f", [0.02], n.den_hartog(0.02)[0]),
        ("den_hartog_zeta", "fr_den_hartog_zeta", [0.02], n.den_hartog(0.02)[1]),
        ("warburton_f", "fr_warburton_f", [0.02], n.warburton(0.02)[0]),
        ("warburton_zeta", "fr_warburton_zeta", [0.02], n.warburton(0.02)[1]),
    ]  # fmt: skip


@pytest.mark.node
@pytest.mark.req("FAB-SW-013")
def test_formulas_match_anchors(node_bin, native, tmp_path):
    """Under node, the self-contained FORMULAS block (assets/js/formulas.js, the pages'
    fallback when wasm is unavailable) matches the native Rust closed forms: every
    ``CASES`` row equals its ``_native`` function to 1e-12 relative and its published
    anchor to the stated tolerance, every closed-form mirror is covered, ``CLASSES``
    equals the presets exactly and ``derivedFor`` equals ``Params.derived()``; the
    ANCHORS "analytic" block, once generated, agrees too. When a wasm build exists its
    analytic exports and its Params.derived() anchor table equal the native values
    exactly (same Rust code, soft-float libm on both targets) (FAB-SW-013)."""
    assert FORMULAS_JS.is_file(), f"{_rel(FORMULAS_JS)} exists"
    wasm = _wasm_under_test()
    wcases = _wasm_analytic_cases(native)
    extra = _composite_checks(native)
    cfg = {
        "formulas_js": str(FORMULAS_JS),
        "extra": [{"id": i, "fn": fn, "args": args} for i, fn, args, _ in extra],
        "wasm": str(wasm) if wasm else None,
        "wasm_cases": [{"id": i, "fn": fn, "args": args} for i, fn, args, _ in wcases],
    }
    out = _node(node_bin, tmp_path, "formulas", cfg)
    bad: list[str] = []

    def close(a, b) -> bool:
        return isinstance(a, (int, float)) and isinstance(b, (int, float)) and (
            a == b or math.isclose(a, b, rel_tol=FORMULA_REL_TOL, abs_tol=0.0)
        )

    # CASES: JS ↔ _native with the same (resolved) arguments, and the published anchors.
    assert out["cases"], "formulas.js CASES is non-empty"
    for c in out["cases"]:
        tag = f"{c['id']}: {c['js']}{tuple(c['args'])} vs _native.{c['py']}"
        if c["error"]:
            bad.append(f"{tag}: JS raised {c['error']}")
            continue
        fn = getattr(native, c["py"] or "", None)
        if fn is None:
            bad.append(f"{tag}: _native has no {c['py']!r}")
            continue
        ref = fn(*c["args"])
        pairs = (
            [(c["value"], ref)]
            if c["out"] is None
            else [((c["value"] or {}).get(jk), ref[nk]) for jk, nk in c["out"].items()]
        )
        for js_v, rs_v in pairs:
            if not close(js_v, rs_v):
                bad.append(f"{tag}: JS {js_v!r} vs Rust {rs_v!r} (rel {FORMULA_REL_TOL:g})")
        if c["expect"] is not None:
            expects = c["expect"] if isinstance(c["expect"], list) else [c["expect"]]
            for (_, rs_v), want in zip(pairs, expects, strict=True):
                if not abs(rs_v - want) <= c["tol"]:
                    bad.append(f"{tag}: Rust {rs_v!r} vs published anchor {want} ± {c['tol']}")
    # Coverage: every closed-form mirror has a CASES row or a composite check here.
    exempt = _FORMULA_HELPERS | {fn for _, fn, _, _ in extra} | {"hPrime"}  # hPrime: derived h_prime_m
    uncovered = sorted(k for k, cov in out["functions"].items() if not cov and k not in exempt)
    if uncovered:
        bad.append(f"FORMULAS {uncovered} have no CASES row (formulas.js) or composite check (this file)")
    for cid, fn, args, ref in extra:
        if not close(out["extra"].get(cid), ref):
            bad.append(f"{fn}{tuple(args)}: JS {out['extra'].get(cid)!r} vs Rust {ref!r}")
    # CLASSES mirror the presets exactly; derivedFor mirrors Params.derived().
    classes, fields = out["classes"], out["class_fields"]
    assert classes and fields, "formulas.js exposes CLASSES and CLASS_FIELDS"
    assert sorted(classes) == sorted(PRESET_IDS), f"CLASSES {sorted(classes)} = the four presets"
    for cls, row in classes.items():
        preset = native.Params.preset(cls)
        pj = json.loads(preset.to_json())
        for field, pointer in fields.items():
            want = _json_pointer(pj, pointer)
            if row.get(field) != want:
                bad.append(f"CLASSES.{cls}.{field} = {row.get(field)!r} but the preset has {pointer} = {want!r}")
        derived = preset.derived()
        for key, js_v in (out["derived"].get(cls) or {}).items():
            if key not in derived:
                bad.append(f"derivedFor({cls!r}).{key}: not a Params.derived() key")
            elif not close(js_v, derived[key]):
                bad.append(f"derivedFor({cls!r}).{key}: JS {js_v!r} vs Rust {derived[key]!r}")
        if "h_prime_m" not in (out["derived"].get(cls) or {}):
            bad.append(f"derivedFor({cls!r}) lacks h_prime_m (the hPrime cross-check)")
    # The generated ANCHORS "analytic" entries (Wave 3) agree with derivedFor.
    anchors = _anchors_block()
    if not anchors.get("placeholder"):
        for name, a in (anchors.get("analytic") or {}).items():
            cls, key = a.get("class"), a.get("key", name)
            js_v = (out["derived"].get(cls) or {}).get(key) if cls else None
            if js_v is not None and not close(js_v, a.get("value")):
                bad.append(f"ANCHORS.analytic.{name}: {a.get('value')!r} vs derivedFor({cls!r}).{key} {js_v!r}")
    assert not bad, (
        "FORMULAS drift from fabrail_params::derive / presets (formulas.js vs _native; "
        "FAB-SW-013):\n" + "\n".join(bad)
    )
    if out["wasm"] is None:
        return
    mismatches = [
        f"{fn}{tuple(args)}: wasm {out['wasm'][cid]!r} vs native {ref!r}"
        for cid, fn, args, ref in wcases
        if out["wasm"][cid] != ref
    ]
    derived = {name: native.Params.preset(name).derived() for name in PRESET_IDS}
    by_id = {v: k for k, v in PRESET_IDS.items()}
    for a in out["anchors"]:
        cls = by_id[a["preset"]]
        ref = derived[cls].get(a["name"])
        if ref is None or a["value"] != ref:
            mismatches.append(f"anchor {cls}.{a['name']}: wasm {a['value']!r} vs native {ref!r}")
    if len(out["anchors"]) != sum(len(d) for d in derived.values()):
        mismatches.append(
            f"the wasm anchor table has {len(out['anchors'])} entries; Params.derived() has "
            f"{sum(len(d) for d in derived.values())} over the four presets"
        )
    if mismatches:
        stale = _staleness(wasm, native)
        if stale:
            pytest.xfail(f"wasm/native analytic mismatch with stale builds ({stale})")
    assert not mismatches, "wasm analytic exports != native (exact):\n" + "\n".join(mismatches)


_DATA_BLOCKS = (
    ("run.js", "RUN", "run"),
    ("exo_run.js", "EXO_RUN", "exo_run"),
    ("villa_run.js", "VILLA_RUN", "villa_run"),
    ("anchors.js", "ANCHORS", "anchors"),
    ("trace.js", "TRACE", "trace"),
)


@pytest.mark.req("FAB-SW-013")
def test_data_blocks_current():
    """The generated data blocks parse, carry their schema, and — once generated
    (Wave 3, scripts/update_docs_run.py) — are current: the golden hash they quote is
    the frozen one, ANCHORS.wasm is the shipped wasm's size and SHA-256, and TRACE
    equals a fresh traceability check (FAB-SW-013)."""
    import fabrail
    from fabrail import trace as fabtrace

    blocks = {}
    for fname, name, schema in _DATA_BLOCKS:
        b = _data_block(DATA_DIR / fname, name)
        assert b.get("schema") == f"fabrail.docs.{schema}/1", f"{fname}: schema"
        assert isinstance(b.get("placeholder"), bool), f"{fname}: placeholder flag"
        blocks[name] = b
    placeholders = sorted(k for k, b in blocks.items() if b["placeholder"])
    golden_hex = f"0x{_golden():016x}"
    bad = []
    for name, b in blocks.items():
        if b["placeholder"]:
            continue
        if b.get("fabrail_version") != fabrail.__version__:
            bad.append(f"{name}.fabrail_version {b.get('fabrail_version')!r} != {fabrail.__version__}")
        if "golden_selftest_hash" in b and _golden() != 0 and b["golden_selftest_hash"] != golden_hex:
            bad.append(f"{name}.golden_selftest_hash {b['golden_selftest_hash']} != {golden_hex}")
    anchors = blocks["ANCHORS"]
    if not anchors["placeholder"] and anchors.get("wasm"):
        w = anchors["wasm"]
        f = DOCS / w.get("file", "")
        if not f.is_file():
            bad.append(f"ANCHORS.wasm.file {w.get('file')!r} is not shipped")
        else:
            data = f.read_bytes()
            if w.get("bytes") != len(data):
                bad.append(f"ANCHORS.wasm.bytes {w.get('bytes')} != {len(data)}")
            if w.get("sha256") != hashlib.sha256(data).hexdigest():
                bad.append("ANCHORS.wasm.sha256 != sha256 of the shipped wasm")
    tr = blocks["TRACE"]
    if not tr["placeholder"]:
        fresh = fabtrace.check(ROOT).to_dict()
        for key in ("ok", "total", "by_doc", "by_area", "by_status", "tokens", "slow_only"):
            if tr.get(key) != fresh.get(key):
                bad.append(f"TRACE.{key} {tr.get(key)!r} != fresh {fresh.get(key)!r}")
    assert not bad, "stale docs data blocks (rerun scripts/update_docs_run.py; FAB-SW-013):\n" + "\n".join(bad)
    if placeholders:
        pytest.xfail(f"placeholders until Wave 3 (C2a, scripts/update_docs_run.py): {placeholders}")


@pytest.mark.req("FAB-SW-013")
def test_read_pages_current():
    """docs/read/*.html are the current rendering of docs/*.md: scripts/build_site.py
    --check exits 0 (Wave 2a, W5) (FAB-SW-013)."""
    script = ROOT / "scripts" / "build_site.py"
    if not script.is_file():
        pytest.skip("scripts/build_site.py (W5, Wave 2a) not written yet")
    if "--check" not in script.read_text(encoding="utf-8"):
        pytest.xfail("scripts/build_site.py has no --check mode yet (contract: exit 0 iff docs/read/ is current)")
    r = subprocess.run(
        [sys.executable, str(script), "--check"],
        capture_output=True,
        text=True,
        timeout=60,
        cwd=ROOT,
    )
    assert r.returncode == 0, f"docs/read/ is stale (rerun scripts/build_site.py):\n{r.stdout}\n{r.stderr}"
    stems = {p.stem.lower() for p in DOCS.glob("*.md")}
    for page in sorted((DOCS / "read").glob("*.html")):
        if page.stem != "index":
            assert page.stem.lower() in stems, f"{_rel(page)} has no docs/{page.stem}.md source"


# ---------------------------------------------------------------------------- FAB-SW-014


def _wasm_sections(data: bytes) -> list[tuple[int, str, int]]:
    """(section id, custom name, size) of a wasm binary."""

    def leb(i: int) -> tuple[int, int]:
        r = s = 0
        while True:
            x = data[i]
            i += 1
            r |= (x & 0x7F) << s
            s += 7
            if x < 0x80:
                return r, i

    assert data[:4] == b"\0asm", "wasm magic"
    out, i = [], 8
    while i < len(data):
        sid = data[i]
        n, i = leb(i + 1)
        name = ""
        if sid == 0:
            ln, j = leb(i)
            name = data[j : j + ln].decode("utf-8", "replace")
        out.append((sid, name, n))
        i += n
    return out


@pytest.mark.req("FAB-SW-014")
def test_wasm_size_budget():
    """Every fabrail_web.wasm present (shipped and newest build) is ≤ 1.5 MB and imports
    nothing (engine.js instantiates it with an empty import object); the shipped one has
    its size and SHA-256 recorded in ANCHORS.wasm (FAB-SW-014)."""
    artifacts = []
    if SHIPPED_WASM.is_file():
        artifacts.append(SHIPPED_WASM)
    built = _built_wasms()
    if built:
        artifacts.append(built[0])
    env = os.environ.get("FABRAIL_WASM")
    if env and Path(env).is_file():
        artifacts.append(Path(env))
    if not artifacts:
        pytest.skip("no fabrail_web.wasm built or shipped yet")
    for a in artifacts:
        data = a.read_bytes()
        assert len(data) <= WASM_BUDGET_BYTES, (
            f"{a}: {len(data):,} bytes > {WASM_BUDGET_BYTES:,} (plan §3b budget 1.5 MB)"
        )
        imports = [s for s in _wasm_sections(data) if s[0] == 2]
        assert not imports, f"{a}: has an import section; engine.js instantiates with {{}}"
    if not SHIPPED_WASM.is_file():
        pytest.skip(
            f"size ok ({built[0].stat().st_size:,} bytes); the SHA-256 is recorded when C2c "
            "ships docs/assets/wasm/fabrail_web.wasm (Wave 3)"
        )
    anchors = _anchors_block()
    w = anchors.get("wasm")
    assert w, "ANCHORS.wasm records the shipped wasm (scripts/update_docs_run.py)"
    data = SHIPPED_WASM.read_bytes()
    assert w.get("file") == "assets/wasm/fabrail_web.wasm"
    assert w.get("bytes") == len(data), "ANCHORS.wasm.bytes = shipped size"
    assert w.get("sha256") == hashlib.sha256(data).hexdigest(), "ANCHORS.wasm.sha256 = shipped SHA-256"


@pytest.mark.node
@pytest.mark.req("FAB-SW-014")
def test_wasm_selftest_matches_golden(node_bin, wasm_path, tmp_path):
    """Under node, engine.js loads the wasm exactly as a page does (base64 path, frozen
    export list), and the selftest hash it computes equals the native
    ``selftest_hash(2000)`` and, once C1 freezes it, ``GOLDEN_SELFTEST_HASH``
    (FAB-SW-014)."""
    out = _node(node_bin, tmp_path, "selftest", {"wasm": str(wasm_path), "engine_js": str(ENGINE_JS)})
    assert out["load"]["mode"] == "wasm", f"engine.js fell back: {out['load']['reason']}"
    assert out["imports"] == [], f"the wasm imports {out['imports']}"
    js = ENGINE_JS.read_text(encoding="utf-8")
    body = js[js.index("var WASM_EXPORTS") :]
    frozen = re.findall(r"^\s*(fabrail_\w+|memory):", body[: body.index("};")], re.M)
    assert frozen and set(frozen) <= set(out["exports"]), "every engine.js WASM_EXPORTS name is exported"
    assert out["abi"] >= 1
    wasm_hash = int(out["hex"], 16)
    assert wasm_hash == (out["hi"] << 32) | out["lo"], "fr_selftest_hash_lo/hi = fabrail_selftest_hash"
    assert wasm_hash not in (0, 2**64 - 1), f"selftest ran (0x{wasm_hash:016x} is a sentinel)"
    assert out["create"] is True and out["stepped"] == 200, "engine.js create/setInputs/step"
    st = out["state"]
    assert list(st) == list(STATE_FIELDS) and st["tick"] == 200, f"engine.js state(): {st}"
    golden = _golden()
    native = _native_or_none()
    stale = _staleness(wasm_path, native)
    if native is not None:
        native_hash = native.selftest_hash(SELFTEST_TICKS)
        if wasm_hash != native_hash and stale:
            pytest.xfail(f"wasm 0x{wasm_hash:016x} vs native 0x{native_hash:016x} with stale builds ({stale})")
        assert wasm_hash == native_hash, (
            f"wasm selftest 0x{wasm_hash:016x} != native 0x{native_hash:016x} "
            "(same executive, same LCG board: must be bit-identical; FAB-SW-014)"
        )
    if golden != 0:
        if wasm_hash != golden and stale:
            pytest.xfail(f"wasm 0x{wasm_hash:016x} vs golden 0x{golden:016x} with a stale build ({stale})")
        assert wasm_hash == golden, f"wasm selftest 0x{wasm_hash:016x} != GOLDEN_SELFTEST_HASH 0x{golden:016x}"
    elif native is None:
        pytest.skip("no reference: _native not built and GOLDEN_SELFTEST_HASH not frozen (C1)")


#: (preset, tier, floor (x, y, z) m/s², command, native record rate Hz): a moving
#: floor and a command on each tier; Travel1d records on its 1 ms plant grid.
_TRACE_CASES = (
    ("door", "carriage6", (0.3, -0.05, 0.02), "open", 20_000.0),
    ("drawer", "heave1", (0.0, 0.0, 0.5), "lift", 20_000.0),
    ("wall", "travel1d", (0.4, 0.0, 0.0), "lift", 1_000.0),
)


def _native_run(native, preset: str, scenario_json: str, floor, command: str, rate_hz: float):
    """The native run of the engine's own scenario plus the same inputs as scenario data:
    a constant floor series (equal samples interpolate exactly) and a command at t = 0."""
    spec = json.loads(scenario_json)
    spec["duration_s"] = TRACE_TICKS / BASE_TICK_HZ
    spec["record"] = {"rate_hz": rate_hz, "channels": "default"}
    spec["series"] = {"floor": {"axis": "time", "fs_hz": 5.0, "t0_s": 0.0, "channels": ["ax", "ay", "az"]}}
    spec["events"] = [{"kind": "command", "t_s": 0.0, "cmd": command}]
    series = {"floor": b"".join(struct.pack("<3d", a, a, a) for a in floor)}
    scen = native.Scenario.from_json(json.dumps(spec), series)
    return native.run(native.Params.preset(preset), scen)


def _channel(run, name: str) -> list[float]:
    buf, dtype, _unit = run.channel(name)
    if dtype == "u1":
        return [float(b) for b in buf]
    assert dtype == "<f8", f"{name}: dtype {dtype}"
    return list(struct.unpack(f"<{len(buf) // 8}d", buf))


def _bits(x: float) -> int:
    return struct.unpack("<Q", struct.pack("<d", x))[0]


def _compare_trace(states: list[list[float]], run, decim: int, n_mag: int) -> list[str]:
    """Bit-exact comparison (see crates/fabrail-web/src/tests.rs): recorded sample j is
    the truth at tick j·decim (= the engine after j·decim steps) and the executive
    outputs of that tick (= the engine after j·decim + 1 steps)."""
    idx = {f: i for i, f in enumerate(STATE_FIELDS)}
    truth = ("x_m", "v_mps", "a_mps2", "iq_a", "f_lsm_n", "f_hand_n", "floor_ax_mps2")
    ctrl = {
        "f_hand_hat_n": "f_hand_hat_n", "m_hat_kg": "m_hat_kg", "e_kin_gov_j": "e_kin_gov_j",
        "lev_state": "lev_state", "motion_mode": "motion_mode", "seismic_state": "seismic_state",
        "level": "seismic_level", "demand": "safety_demand",
    }  # fmt: skip
    ch = {name: _channel(run, name) for name in (*truth, *ctrl)}
    gaps = [_channel(run, f"gap_{i}_m") for i in range(n_mag)]
    coils = [_channel(run, f"coil_{i}_a") for i in range(n_mag)]

    def mean(chs: list[list[float]], j: int) -> float:
        s = 0.0
        for c in chs:  # element order, like the Rust fold (math.fsum/sum would differ)
            s += c[j]
        return s / n_mag

    n = run.n_samples()
    errs = []
    if (n - 1) * decim != len(states) - 1:
        return [f"sample grid: {n} samples every {decim} ticks vs {len(states) - 1} ticks"]
    for j in range(n):
        k = j * decim
        st = states[k]
        for name in truth:
            if _bits(st[idx[name]]) != _bits(ch[name][j]):
                errs.append(f"{name} at tick {k}: wasm {st[idx[name]]!r} vs native {ch[name][j]!r}")
        for name, chs in (("gap_m", gaps), ("coil_i_a", coils)):
            if _bits(st[idx[name]]) != _bits(mean(chs, j)):
                errs.append(f"{name} (mean) at tick {k}: wasm {st[idx[name]]!r} vs native {mean(chs, j)!r}")
        if k + 1 < len(states):
            sc = states[k + 1]
            for cname, field in ctrl.items():
                if _bits(sc[idx[field]]) != _bits(ch[cname][j]):
                    errs.append(f"{field} of tick {k}: wasm {sc[idx[field]]!r} vs native {ch[cname][j]!r}")
        if len(errs) > 20:
            break
    return errs


@pytest.mark.node
@pytest.mark.req("FAB-SW-014")
def test_wasm_trace_matches_native(node_bin, native, wasm_path, tmp_path):
    """A wasm step run (one base tick per call, floor and command inputs held) matches
    the native ``run()`` of the scenario the engine exports, bit for bit, over the first
    2,000 ticks on Carriage6, Heave1 and Travel1d (FAB-SW-014)."""
    cfg_cases = [
        {"preset": PRESET_IDS[p], "tier": TIER_IDS[t], "floor": list(f), "cmd": CMD[c],
         "hand_n": 0.0, "ticks": TRACE_TICKS}
        for p, t, f, c, _ in _TRACE_CASES
    ]  # fmt: skip
    out = _node(node_bin, tmp_path, "trace", {"wasm": str(wasm_path), "cases": cfg_cases})
    errors = []
    for (preset, tier, floor, command, rate), res in zip(_TRACE_CASES, out, strict=True):
        assert res["status"] == 0, f"{preset}/{tier}: engine stopped (status {res['status']})"
        raw = base64.b64decode(res["states"])
        vals = struct.unpack(f"<{len(raw) // 8}d", raw)
        n = res["len"]
        assert n == len(STATE_FIELDS)
        states = [list(vals[i : i + n]) for i in range(0, len(vals), n)]
        assert len(states) == TRACE_TICKS + 1
        assert json.loads(res["scenario"])["fidelity"]["model"] == tier
        try:
            run = _native_run(native, preset, res["scenario"], floor, command, rate)
        except NotImplementedError as e:
            pytest.skip(f"native run() not available in this build: {e}")
        n_mag = int(native.Params.preset(preset).get("/design/suspension/n_magnets"))
        decim = round(BASE_TICK_HZ / rate)
        errs = _compare_trace(states, run, decim, n_mag)
        errors += [f"{preset}/{tier}: {e}" for e in errs]
    if errors:
        stale = _staleness(wasm_path, native)
        if stale:
            pytest.xfail(f"wasm/native trace mismatch with stale builds ({stale}): {errors[0]}")
    assert not errors, "wasm trace != native run() (bit-exact, FAB-SW-014):\n" + "\n".join(errors[:40])
