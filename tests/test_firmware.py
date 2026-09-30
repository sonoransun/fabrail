"""Firmware-target guards (FAB-FW-001/002/003/007/008/009/010/013, FAB-EXO-307/901/902).

These are inspection guards over the sources plus two real toolchain gates. Nothing
here runs on hardware: the firmware is built and linked, never flashed (FAB-FW-012).

Contract:

- ``manifest_pins``: firmware/Cargo.toml pins the crates FAB-FW-001 names with exact
  (``=``) requirements, Cargo.lock resolves exactly those versions, and the build
  targets thumbv7em-none-eabihf on the STM32G474 (``stm32g474`` feature, RTIC
  ``thumbv7-backend``, 512K/128K memory map). The expected pins are parsed from the
  FAB-FW-001 row itself, so the specification and the manifest cannot drift apart.
- ``nostd_source_guard``: the eight no_std crates carry ``#![cfg_attr(not(test),
  no_std)]`` and ``#![forbid(unsafe_code)]``, use no alloc and no workspace
  inheritance, and pass the tier-1 source guards of scripts/check_nostd.sh (fmath-only
  math, no mul_add, num_traits, nalgebra or cfg(target_arch)); firmware ``unsafe``
  lives only in firmware/src/bsp with ``// SAFETY:`` comments.
- ``dependency_allowlist`` (cargo): tiers 3 and 5 of scripts/check_nostd.sh (thumbv7em
  dependency allowlist, no std/alloc features, safety-crate closure).
- ``no_cfg_divergence``: the shared crates contain no target/OS/std/sim ``cfg``; the only
  feature gates are ``serde`` and fabrail-rt's simulation-only ``fault-injection``; the
  firmware runs the fabrail-rt groups from ``Executive::split`` — the same group types
  ``Executive::on_base_tick`` drives in SIL — and depends on no plant or sim crate.
- ``task_map_matches_design``: the TECHNICAL_DESIGN §9.2 table equals the ``#[task]``,
  ``#[idle]`` and dispatcher attributes of firmware/src/main.rs (binds and priorities).
- ``schedulability_budget``: ΣC/T of the §9.2 periodic budgets at the §9.1 core clock
  is ≤ 0.70 and equals the sums stated in TECHNICAL_DESIGN and FAB-FW-008.
- ``panic_policy_lints``: core, safety and rt deny unwrap/expect/panic/indexing; every
  monitor step module denies float arithmetic (allowed only in tests and in
  ``Monitor::new``'s threshold derivation); the production panic handler disables the
  outputs, writes a crash record and resets; ``panic = "abort"``.
- ``firmware_link_size`` (slow, cargo): scripts/check_firmware.sh builds and links every
  class image and each meets the flash/RAM budget TECHNICAL_DESIGN §9.1 states.
- ``no_fault_injection_in_firmware``: the simulation-only injector appears nowhere in
  the firmware manifest, lockfile, sources or resolved feature graph.
- ``cross_check`` (slow, cargo): real cross builds, scripts/check_nostd.sh tier 4
  ``--strict`` (thumbv7em for the control crates; thumbv6m and armv7r for the safety
  crates).
"""

from __future__ import annotations

import json
import re
import tomllib
from pathlib import Path

import pytest

import _support

ROOT = _support.ROOT
FW = ROOT / "firmware"
CRATES = ROOT / "crates"
SPEC = ROOT / "docs" / "SPECIFICATION.md"
TD = ROOT / "docs" / "TECHNICAL_DESIGN.md"
CHECK_NOSTD = ROOT / "scripts" / "check_nostd.sh"
CHECK_FIRMWARE = ROOT / "scripts" / "check_firmware.sh"

#: The no_std crates (CLAUDE.md "no_std crates"; FAB-FW-002, FAB-EXO-902).
NOSTD_CRATES = ("types", "params", "hal", "core", "safety", "rt", "exo-safety", "exo")
#: The shared control crates the firmware links (FAB-FW-003).
SHARED_CRATES = ("types", "params", "hal", "core", "safety", "rt")
#: Feature gates allowed in shared-crate code, with the reason.
ALLOWED_FEATURE_CFGS = {
    "serde": "derive-only serialisation of plain data",
    "fault-injection": "simulation-only injector; the golden hash is identical with and without it (FAB-FW-005)",
}
FAULT_INJECTION_RE = re.compile(r"fault[-_]injection", re.I)


def _read(p: Path) -> str:
    return p.read_text(encoding="utf-8")


def _strip_comments(text: str) -> str:
    """Rust source without ``//`` and ``/* */`` comments (string literals are rare here)."""
    text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def _spec_row(rid: str) -> str:
    for line in _read(SPEC).splitlines():
        if re.match(rf"^\|\s*{re.escape(rid)}\s*\|", line):
            return line
    raise AssertionError(f"{rid} row not found in docs/SPECIFICATION.md")


# ---------------------------------------------------------------------------- FW-001


def _pins_from_spec() -> dict[str, str]:
    row = _spec_row("FAB-FW-001")
    m = re.search(r"pinned crates \(([^)]*)\)", row)
    assert m, "FAB-FW-001 row: no 'pinned crates (…)' list"
    pins = dict(re.findall(r"([a-z0-9][a-z0-9_-]*)\s+(\d+\.\d+(?:\.\d+)?)", m.group(1)))
    assert len(pins) >= 9, f"FAB-FW-001 lists {len(pins)} pins: {pins}"
    return pins


def _lock_versions(lock_path: Path) -> dict[str, list[str]]:
    lock = tomllib.loads(_read(lock_path))
    out: dict[str, list[str]] = {}
    for pkg in lock.get("package", []):
        out.setdefault(pkg["name"], []).append(pkg["version"])
    return out


def _dep_version(spec: object) -> str:
    return spec if isinstance(spec, str) else str(dict(spec).get("version", ""))  # type: ignore[arg-type]


@pytest.mark.req("FAB-FW-001")
def test_manifest_pins():
    """firmware/Cargo.toml pins every FAB-FW-001 crate exactly, Cargo.lock resolves the
    same versions, and the build targets thumbv7em-none-eabihf on the STM32G474
    (FAB-FW-001; TECHNICAL_DESIGN §9.1)."""
    pins = _pins_from_spec()
    manifest = tomllib.loads(_read(FW / "Cargo.toml"))
    deps = manifest.get("dependencies", {})
    locked = _lock_versions(FW / "Cargo.lock")
    bad = []
    for name, want in sorted(pins.items()):
        if name not in deps:
            bad.append(f"{name}: not a firmware dependency")
            continue
        req = _dep_version(deps[name])
        if not req.startswith("="):
            bad.append(f"{name}: requirement {req!r} is not an exact '=' pin")
        got = req.lstrip("=").strip()
        if not (got == want or got.startswith(want + ".")):
            bad.append(f"{name}: manifest pins {got!r}, FAB-FW-001 says {want}")
        versions = locked.get(name, [])
        if len(versions) != 1 or not (versions[0] == want or versions[0].startswith(want + ".")):
            bad.append(f"{name}: Cargo.lock resolves {versions}, FAB-FW-001 says {want}")
    for dev_only in ("defmt", "defmt-rtt", "panic-probe"):
        if dev_only in deps and not (isinstance(deps[dev_only], dict) and deps[dev_only].get("optional")):
            bad.append(f"{dev_only}: must be optional (dev feature only; production uses the crash handler)")
    feats = lambda n: set(dict(deps.get(n, {})).get("features", [])) if isinstance(deps.get(n), dict) else set()  # noqa: E731
    if "thumbv7-backend" not in feats("rtic"):
        bad.append("rtic: feature thumbv7-backend missing (Cortex-M4F BASEPRI backend)")
    if "stm32g474" not in feats("stm32g4xx-hal"):
        bad.append("stm32g4xx-hal: feature stm32g474 missing")
    if manifest.get("profile", {}).get("release", {}).get("panic") != "abort":
        bad.append("[profile.release] panic must be 'abort'")
    cfg = tomllib.loads(_read(FW / ".cargo" / "config.toml"))
    if cfg.get("build", {}).get("target") != "thumbv7em-none-eabihf":
        bad.append(f"firmware/.cargo/config.toml build.target = {cfg.get('build', {}).get('target')!r}")
    tc = tomllib.loads(_read(FW / "rust-toolchain.toml"))
    if "thumbv7em-none-eabihf" not in tc.get("toolchain", {}).get("targets", []):
        bad.append("firmware/rust-toolchain.toml does not list thumbv7em-none-eabihf")
    mem = _read(FW / "memory.x")
    if not re.search(r"FLASH\s*:\s*ORIGIN\s*=\s*0x08000000\s*,\s*LENGTH\s*=\s*512K", mem):
        bad.append("memory.x: FLASH is not 512K at 0x08000000 (STM32G474RE)")
    if not re.search(r"RAM\s*:\s*ORIGIN\s*=\s*0x20000000\s*,\s*LENGTH\s*=\s*128K", mem):
        bad.append("memory.x: RAM is not 128K at 0x20000000 (STM32G474RE)")
    if "firmware" not in tomllib.loads(_read(ROOT / "Cargo.toml"))["workspace"].get("exclude", []):
        bad.append("the host workspace must exclude firmware/ (separate workspace)")
    assert not bad, "firmware manifest pins (FAB-FW-001):\n" + "\n".join(bad)


# ---------------------------------------------------------------------------- FW-002


def _firmware_unsafe_violations() -> list[str]:
    src = FW / "src"
    bad = []
    for f in sorted(src.rglob("*.rs")):
        rel = f.relative_to(src).as_posix()
        lines = _read(f).splitlines()
        for i, line in enumerate(lines):
            code = line.split("//", 1)[0]
            if not re.search(r"\bunsafe\b", code):
                continue
            if re.search(r"#!?\[\s*(allow|deny|forbid|warn)\(\s*unsafe_code\s*\)", code):
                continue
            if not rel.startswith("bsp/"):
                bad.append(f"firmware/src/{rel}:{i + 1}: unsafe outside bsp/")
                continue
            if re.search(r"#\[\s*unsafe\(", code):
                continue
            back = lines[max(0, i - 12) : i + 1]
            if re.search(r"\bunsafe\s+(extern\s+\"C\"\s+)?fn\b", code):
                if not any("# Safety" in b or "SAFETY:" in b for b in back):
                    bad.append(f"firmware/src/{rel}:{i + 1}: unsafe fn without a '# Safety' section")
            elif not any("SAFETY:" in b for b in back[-8:]):
                bad.append(f"firmware/src/{rel}:{i + 1}: unsafe block without a '// SAFETY:' comment")
    main_rs = _read(src / "main.rs")
    if "#![deny(unsafe_code)]" not in main_rs:
        bad.append("firmware/src/main.rs lacks #![deny(unsafe_code)]")
    return bad


def _non_test_code(path: Path) -> str:
    """Rust code of ``path`` without comments and without ``#[cfg(test)] mod … { … }`` blocks."""
    text = _strip_comments(_read(path))
    out, i = [], 0
    for m in re.finditer(r"#\[cfg\(test\)\]\s*(?:#\[[^\]]*\]\s*)*(?:pub(?:\([^)]*\))?\s+)?mod\s+\w+\s*\{", text):
        if m.start() < i:
            continue
        out.append(text[i : m.start()])
        depth, j = 1, m.end()
        while j < len(text) and depth:
            depth += {"{": 1, "}": -1}.get(text[j], 0)
            j += 1
        i = j
    out.append(text[i:])
    return "".join(out)


@pytest.mark.req("FAB-FW-002", "FAB-EXO-902")
def test_nostd_source_guard():
    """The no_std crates are no_std, forbid(unsafe_code), allocation-free, fmath-only and
    free of workspace inheritance (scripts/check_nostd.sh tier 1 plus an independent
    re-check here); firmware unsafe code lives only in firmware/src/bsp with SAFETY
    comments (FAB-FW-002, FAB-EXO-902)."""
    bad = []
    for c in NOSTD_CRATES:
        crate = CRATES / f"fabrail-{c}"
        lib = _read(crate / "src" / "lib.rs")
        if not re.search(r"^\s*#!\[cfg_attr\(not\(test\),\s*no_std\)\]", lib, re.M):
            bad.append(f"fabrail-{c}: lib.rs lacks #![cfg_attr(not(test), no_std)]")
        if not re.search(r"^\s*#!\[forbid\(unsafe_code\)\]", lib, re.M):
            bad.append(f"fabrail-{c}: lib.rs lacks an unconditional #![forbid(unsafe_code)]")
        if re.search(r"(^|\.)workspace\s*=\s*true", _read(crate / "Cargo.toml"), re.M):
            bad.append(f"fabrail-{c}: Cargo.toml uses workspace inheritance (firmware consumes it by path)")
        manifest = tomllib.loads(_read(crate / "Cargo.toml"))
        for dep in ("std", "alloc", "nalgebra", "num-traits", "rayon"):
            if dep in manifest.get("dependencies", {}):
                bad.append(f"fabrail-{c}: depends on {dep}")
        for f in sorted((crate / "src").rglob("*.rs")):
            code = _non_test_code(f)
            rel = _support.rel(f)
            for pat, what in (
                (r"\bextern\s+crate\s+(std|alloc)\b", "extern crate std/alloc"),
                (r"(^|[^\w:])(use\s+)?alloc::", "alloc:: path"),
                (r"(^|[^\w:])std::", "std:: path"),
                (r"\b(Vec|String|Box)\s*<", "heap type"),
                (r"\bmul_add\b", "mul_add"),
                (r"cfg\([^)]*target_arch", "cfg(target_arch)"),
            ):
                for m in re.finditer(pat, code, re.M):
                    if what == "heap type" and (
                        "heapless" in code[max(0, m.start() - 20) : m.start()]
                        or re.search(rf"use\s+heapless::(?:{m.group(1)}\b|\{{[^}}]*\b{m.group(1)}\b)", code)
                    ):
                        continue
                    line = code.count("\n", 0, m.start()) + 1
                    bad.append(f"{rel}:~{line}: {what}")
                    break
    r = _support.run_script(["bash", str(CHECK_NOSTD), "--tiers", "1"], timeout=120)
    if r.returncode != 0:
        bad.append("scripts/check_nostd.sh --tiers 1 failed:\n" + r.tail)
    bad += _firmware_unsafe_violations()
    assert not bad, "no_std source guards (FAB-FW-002, FAB-EXO-902):\n" + "\n".join(bad)


@pytest.mark.requires_cargo
@pytest.mark.req("FAB-FW-002")
def test_dependency_allowlist():
    """The thumbv7em dependency closure of every no_std crate stays inside the allowlist
    with no std/alloc/libm-arch features, and the safety crates reach only
    fabrail-types and fabrail-params (scripts/check_nostd.sh tiers 3 and 5; FAB-FW-002)."""

    def once() -> _support.ScriptResult:
        return _support.run_script(
            ["bash", str(CHECK_NOSTD), "--tiers", "3 5"],
            env={"CARGO_TARGET_DIR": str(_support.pytest_target_dir("nostd"))},
            timeout=300,
        )

    r = once()
    if r.returncode != 0:
        # `cargo tree` output is parsed with stderr merged: while another build holds the
        # package-cache lock ("Blocking waiting for file lock …") or rewrites Cargo.lock the
        # check fails transiently. A genuine allowlist violation fails both attempts.
        first = r.tail
        r = once()
        if r.returncode != 0:
            r = _support.ScriptResult(r.returncode, first + "\n--- retry ---\n" + r.stdout, r.stderr, r.wall_s)
    assert r.returncode == 0, f"check_nostd.sh --tiers '3 5' failed (FAB-FW-002):\n{r.tail}"
    n_pass = len(re.findall(r"^\s*PASS\s", r.stdout, re.M))
    assert n_pass >= 2 * len(NOSTD_CRATES) + 2, f"only {n_pass} checks passed (expected ≥ {2 * len(NOSTD_CRATES) + 2}):\n{r.tail}"


# ---------------------------------------------------------------------------- FW-003


@pytest.mark.req("FAB-FW-003")
def test_no_cfg_divergence():
    """SIL and firmware run the same fabrail-rt group code: no target/OS/std/sim cfg in
    the shared crates, only the serde and simulation-only fault-injection feature gates,
    the firmware takes its groups from Executive::split (the types on_base_tick drives
    in SIL), and the firmware links no plant or sim crate (FAB-FW-003)."""
    bad = []
    for c in SHARED_CRATES:
        for f in sorted((CRATES / f"fabrail-{c}" / "src").rglob("*.rs")):
            code = _strip_comments(_read(f))
            rel = _support.rel(f)
            for m in re.finditer(r"cfg(?:_attr)?\s*\(", code):
                seg = code[m.end() : m.end() + 200]
                depth, j = 1, 0
                while j < len(seg) and depth:
                    depth += {"(": 1, ")": -1}.get(seg[j], 0)
                    j += 1
                cond = seg[: j - 1]
                line = code.count("\n", 0, m.start()) + 1
                for key in ("target_arch", "target_os", "target_family", "target_pointer_width", "target_env", "target_vendor"):
                    if key in cond:
                        bad.append(f"{rel}:{line}: cfg on {key}")
                for feat in re.findall(r'feature\s*=\s*"([^"]+)"', cond):
                    if feat not in ALLOWED_FEATURE_CFGS:
                        bad.append(f"{rel}:{line}: cfg(feature = {feat!r}) is not an allowed gate")
                    if feat == "fault-injection" and c != "rt":
                        bad.append(f"{rel}:{line}: fault-injection gate outside fabrail-rt")
    for c in SHARED_CRATES:
        feats = tomllib.loads(_read(CRATES / f"fabrail-{c}" / "Cargo.toml")).get("features", {})
        for f in feats:
            if f not in ("default", *ALLOWED_FEATURE_CFGS):
                bad.append(f"fabrail-{c}: feature {f!r} (possible sim-only control path)")
    exec_rs = _read(CRATES / "fabrail-rt" / "src" / "exec.rs")
    for group in ("BaseTask", "FastGroup", "SupervisorGroup", "SeismicGroup"):
        if not re.search(rf"\bfn\s+split\b[\s\S]*?\b{group}\b", exec_rs):
            bad.append(f"fabrail-rt Executive::split does not hand out {group}")
    if not re.search(r"\bfn\s+on_base_tick\b", exec_rs):
        bad.append("fabrail-rt has no Executive::on_base_tick (the SIL entry)")
    main_rs = _strip_comments(_read(FW / "src" / "main.rs"))
    if not re.search(r"\.split\(\)", main_rs):
        bad.append("firmware/src/main.rs does not take its groups from Executive::split()")
    for group in ("BaseTask", "FastGroup", "SupervisorGroup", "SeismicGroup", "Executive"):
        if not re.search(rf"\b{group}\b", main_rs):
            bad.append(f"firmware/src/main.rs does not use fabrail_rt::{group}")
    engine = "\n".join(_read(p) for p in (CRATES / "fabrail-sim" / "src").rglob("*.rs"))
    if "fabrail_rt::" not in engine or "Executive" not in engine or "on_base_tick" not in engine:
        bad.append("fabrail-sim does not drive fabrail_rt::Executive::on_base_tick")
    fw_deps = tomllib.loads(_read(FW / "Cargo.toml")).get("dependencies", {})
    for d in fw_deps:
        if d in ("fabrail-plant", "fabrail-sim", "fabrail-py", "fabrail-exo-sim", "fabrail-web"):
            bad.append(f"firmware depends on {d} (a host/simulation crate)")
    shared = {f"fabrail-{c}" for c in SHARED_CRATES}
    fab = {d for d in fw_deps if d.startswith("fabrail-")}
    if fab != shared:
        bad.append(f"firmware fabrail dependencies {sorted(fab)} != the shared crates {sorted(shared)}")
    assert not bad, "SIL/firmware divergence (FAB-FW-003):\n" + "\n".join(bad)


# ---------------------------------------------------------------------------- FW-007/008


def _section(text: str, heading_re: str) -> str:
    m = re.search(heading_re, text, re.M)
    assert m, f"heading {heading_re!r} not found"
    rest = text[m.end() :]
    nxt = re.search(r"^#{1,3}\s", rest, re.M)
    return rest[: nxt.start()] if nxt else rest


def _md_table(section: str) -> list[dict[str, str]]:
    lines = [ln for ln in section.splitlines() if ln.strip().startswith("|")]
    assert len(lines) >= 3, "no Markdown table in the section"
    head = [c.strip() for c in lines[0].strip().strip("|").split("|")]
    rows = []
    for line in lines[2:]:
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        rows.append(dict(zip(head, cells, strict=False)))
    return rows


def _task_table() -> list[dict[str, str]]:
    return _md_table(_section(_read(TD), r"^###\s*§9\.2\b.*$"))


def _code_names(cell: str) -> list[str]:
    return re.findall(r"`([^`]+)`", cell)


def _firmware_tasks() -> tuple[dict[str, dict[str, object]], list[str]]:
    """{task: {binds, priority, kind}} from firmware/src/main.rs, and the dispatchers."""
    text = _strip_comments(_read(FW / "src" / "main.rs"))
    app = re.search(r"#\[rtic::app\((.*?)\)\]\s*(?:pub\s+)?mod\b", text, re.S)
    assert app, "firmware/src/main.rs: no #[rtic::app(...)]"
    disp = re.search(r"dispatchers\s*=\s*\[([^\]]*)\]", app.group(1))
    dispatchers = [d.strip() for d in (disp.group(1).split(",") if disp else []) if d.strip()]
    tasks: dict[str, dict[str, object]] = {}
    for m in re.finditer(r"#\[(task|idle|init)(?:\((.*?)\))?\]\s*(?:pub\s+)?(?:async\s+)?fn\s+(\w+)", text, re.S):
        kind, args, name = m.group(1), m.group(2) or "", m.group(3)
        if kind == "init":
            continue
        binds = re.search(r"binds\s*=\s*(\w+)", args)
        prio = re.search(r"priority\s*=\s*(\d+)", args)
        tasks[name] = {
            "kind": kind,
            "binds": binds.group(1) if binds else None,
            "priority": int(prio.group(1)) if prio else (0 if kind == "idle" else 1),
        }
    return tasks, dispatchers


@pytest.mark.req("FAB-FW-007")
def test_task_map_matches_design():
    """Every TECHNICAL_DESIGN §9.2 row names a firmware task with the same binding and
    priority, and every #[task]/#[idle] in firmware/src/main.rs has a row (FAB-FW-007)."""
    rows = _task_table()
    tasks, dispatchers = _firmware_tasks()
    bad = []
    seen = set()
    for row in rows:
        names = _code_names(row.get("Task", ""))
        assert names, f"§9.2 row without a `task` name: {row}"
        name = names[0]
        seen.add(name)
        fw = tasks.get(name)
        if fw is None:
            bad.append(f"§9.2 task {name!r} has no #[task]/#[idle] fn in firmware/src/main.rs")
            continue
        binds_cell = row.get("Binds", "")
        prio_cell = row.get("Priority", "").strip()
        want_prio = int(prio_cell) if prio_cell.isdigit() else None
        if want_prio is not None and fw["priority"] != want_prio:
            bad.append(f"{name}: priority {fw['priority']} in main.rs, {want_prio} in §9.2")
        if fw["kind"] == "idle":
            if want_prio not in (None, 0):
                bad.append(f"{name}: #[idle] runs at priority 0, §9.2 says {want_prio}")
            continue
        binds = _code_names(binds_cell)
        if fw["binds"] is None:  # software task: dispatched on one of the RTIC dispatchers
            if "software" not in binds_cell.lower():
                bad.append(f"{name}: a software task in main.rs, but §9.2 binds it to {binds_cell!r}")
            for d in binds:
                if d not in dispatchers:
                    bad.append(f"{name}: §9.2 names dispatcher {d}, main.rs dispatchers are {dispatchers}")
        elif binds != [fw["binds"]]:
            bad.append(f"{name}: binds {fw['binds']} in main.rs, {binds_cell!r} in §9.2")
    for name in sorted(set(tasks) - seen):
        bad.append(f"firmware task {name!r} ({tasks[name]}) has no §9.2 row")
    assert not bad, "task map vs TECHNICAL_DESIGN §9.2 (FAB-FW-007):\n" + "\n".join(bad)


_RATE_RE = re.compile(r"(\d[\d.]*(?:\s*/\s*\d[\d.]*)*)\s*(k?Hz)")


def _num(s: str) -> float:
    return float(s.replace(",", "").replace(" ", "").strip())


def _utilisation() -> tuple[float, list[str]]:
    """ΣC/T of the §9.2 periodic tasks at the §9.1 clock, and the per-task terms."""
    td = _read(TD)
    clock = re.search(r"STM32G474\w*\s*\(Cortex-M4F,\s*([\d.]+)\s*MHz", td)
    assert clock, "TECHNICAL_DESIGN §9.1: core clock 'Cortex-M4F, <n> MHz' not found"
    f_cpu = float(clock.group(1)) * 1e6
    total, terms = 0.0, []
    for row in _task_table():
        name = (_code_names(row.get("Task", "")) or ["?"])[0]
        budget, rate = row.get("Budget (cycles)", ""), row.get("Rate", "")
        budgets = [_num(b) for b in re.findall(r"\d[\d,]*", budget.split("(")[0])]
        rates = [float(v) * (1e3 if u == "kHz" else 1.0) for vs, u in _RATE_RE.findall(rate) for v in re.split(r"\s*/\s*", vs)]
        if not budgets or not rates:
            continue  # sporadic, remainder or shared budget
        # Paired alternatives ("10 / 5 kHz" with "4,000 / 8,000"): take the worst term.
        pairs = list(zip(budgets, rates, strict=True)) if len(budgets) == len(rates) else [(budgets[0], rates[0])]
        u = max(c * r / f_cpu for c, r in pairs)
        total += u
        terms.append(f"{name} {u:.3f}")
    return total, terms


@pytest.mark.req("FAB-FW-008")
def test_schedulability_budget():
    """ΣC/T of the §9.2 periodic cycle budgets at the §9.1 clock is ≤ 0.70 (Liu–Layland,
    harmonic periods) and equals the sum TECHNICAL_DESIGN §9.2 and FAB-FW-008 state
    (± 0.005). These are budgets, not measurements (FAB-FW-012)."""
    total, terms = _utilisation()
    assert terms, "no periodic task budgets parsed from TECHNICAL_DESIGN §9.2"
    assert total <= 0.70, f"(A) ΣC/T = {total:.3f} > 0.70 design limit (FAB-FW-008): {terms}"
    stated_td = re.search(r"=\s*\*\*([\d.]+)\*\*\s*of the", _section(_read(TD), r"^###\s*§9\.2\b.*$"))
    assert stated_td, "TECHNICAL_DESIGN §9.2: no '= **0.xx** of the …' schedulability sum"
    stated_spec = re.search(r"current sum ([\d.]+)", _spec_row("FAB-FW-008"))
    assert stated_spec, "FAB-FW-008 row: no 'current sum 0.xx'"
    for where, val in (("TECHNICAL_DESIGN §9.2", stated_td.group(1)), ("FAB-FW-008", stated_spec.group(1))):
        assert abs(float(val) - total) <= 0.005, f"(A) {where} states {val}, the table gives {total:.3f} ({terms})"


# ---------------------------------------------------------------------------- FW-009


def _fn_body_span(text: str, start: int) -> tuple[int, int]:
    """(start, end) of the braced body of the item beginning at ``start``."""
    i = text.index("{", start)
    depth, j = 1, i + 1
    while j < len(text) and depth:
        depth += {"{": 1, "}": -1}.get(text[j], 0)
        j += 1
    return i, j


@pytest.mark.req("FAB-FW-009")
def test_panic_policy_lints():
    """core, safety and rt deny unwrap/expect/panic/indexing outside tests; every
    fabrail-safety module denies float arithmetic, with allowances only on test modules
    and on Monitor::new's threshold derivation (and helpers only it calls); the
    production panic handler disables outputs, writes a crash record and resets; the
    release profile aborts on panic (FAB-FW-009; TECHNICAL_DESIGN §9.4)."""
    bad = []
    lints = ("clippy::unwrap_used", "clippy::expect_used", "clippy::panic", "clippy::indexing_slicing")
    for c in ("core", "safety", "rt"):
        lib = _read(CRATES / f"fabrail-{c}" / "src" / "lib.rs")
        m = re.search(r"#!\[cfg_attr\(\s*not\(test\)\s*,\s*deny\(([^)]*)\)\s*\)\]", lib, re.S)
        denied = {x.strip() for x in m.group(1).split(",")} if m else set()
        for lint in lints:
            if lint not in denied:
                bad.append(f"fabrail-{c}/src/lib.rs does not deny {lint} outside tests")
    safety_src = CRATES / "fabrail-safety" / "src"
    for f in sorted(safety_src.glob("*.rs")):
        if f.name == "lib.rs":
            continue
        text = _read(f)
        if "#![deny(clippy::float_arithmetic)]" not in text:
            bad.append(f"{_support.rel(f)}: no #![deny(clippy::float_arithmetic)]")
        allowed_fns: list[str] = []
        for m in re.finditer(r"#\[allow\(clippy::float_arithmetic\)\]", text):
            after = text[m.end() : m.end() + 300]
            item = re.match(r"\s*(?:#\[[^\]]*\]\s*)*(?:pub(?:\([^)]*\))?\s+)?(mod\s+(\w+)|fn\s+(\w+))", after)
            before = text[max(0, m.start() - 120) : m.start()]
            if item and item.group(2) == "tests" and "#[cfg(test)]" in before:
                continue
            if item and item.group(3):
                allowed_fns.append(item.group(3))
                continue
            line = text.count("\n", 0, m.start()) + 1
            bad.append(f"{_support.rel(f)}:{line}: float arithmetic allowed on a non-fn, non-test item")
        if allowed_fns:
            code = _non_test_code(f)
            if "new" not in allowed_fns:
                bad.append(f"{_support.rel(f)}: float arithmetic allowed in {allowed_fns} outside Monitor::new")
                continue
            news = [m.start() for m in re.finditer(r"\bfn\s+new\b", code)]
            spans = [_fn_body_span(code, s) for s in news]
            helper_spans = {
                h: _fn_body_span(code, code.index(f"fn {h}")) for h in allowed_fns if h != "new" and f"fn {h}" in code
            }
            for h in helper_spans:
                for call in re.finditer(rf"\b{h}\s*\(", code):
                    pos = call.start()
                    if code[max(0, pos - 3) : pos] == "fn ":
                        continue
                    inside = any(s <= pos < e for s, e in spans) or any(
                        s <= pos < e for k, (s, e) in helper_spans.items() if k != h
                    )
                    if not inside:
                        line = code.count("\n", 0, pos) + 1
                        bad.append(f"{_support.rel(f)}:~{line}: float helper {h}() called outside Monitor::new")
    crash = _read(FW / "src" / "bsp" / "crash.rs")
    ph = re.search(r"#\[cfg\(not\(feature\s*=\s*\"dev\"\)\)\]\s*#\[panic_handler\]\s*fn\s+(\w+)", crash)
    if not ph:
        bad.append('firmware: no production #[panic_handler] gated on not(feature = "dev") in bsp/crash.rs')
    else:
        s, e = _fn_body_span(crash, ph.start())
        body = crash[s:e]
        called = re.findall(r"\b(\w+)\s*\(", body)
        chain = body + "".join(
            crash[slice(*_fn_body_span(crash, crash.index(f"fn {c}(")))] for c in called if f"fn {c}(" in crash
        )
        for what, pat in (
            ("disables the outputs", r"outputs_off\s*\("),
            ("writes a crash record", r"\brecord\s*\("),
            ("resets", r"sys_reset\s*\("),
        ):
            if not re.search(pat, chain):
                bad.append(f"firmware panic handler never {what} ({pat})")
    main_rs = _read(FW / "src" / "main.rs")
    for lint in ("clippy::unwrap_used", "clippy::expect_used", "clippy::panic"):
        if lint not in main_rs:
            bad.append(f"firmware/src/main.rs does not deny {lint}")
    fw_manifest = tomllib.loads(_read(FW / "Cargo.toml"))
    if "panic-halt" in fw_manifest.get("dependencies", {}):
        bad.append("firmware depends on panic-halt (halts with the timers running; rejected in TD §9.4)")
    if fw_manifest.get("profile", {}).get("release", {}).get("panic") != "abort":
        bad.append("firmware [profile.release] panic is not 'abort'")
    assert not bad, "panic policy (FAB-FW-009):\n" + "\n".join(bad)


# ---------------------------------------------------------------------------- FW-010


def _documented_budget_kib() -> tuple[int, int]:
    row = next(
        (r for r in _md_table(_section(_read(TD), r"^###\s*§9\.1\b.*$")) if r.get("Item", "").startswith("Size budget")),
        None,
    )
    assert row, "TECHNICAL_DESIGN §9.1: no 'Size budget' row"
    text = row.get("Choice", "")
    flash = re.search(r"Flash\s*≤\s*(\d+)\s*KiB", text)
    ram = re.search(r"RAM\s*≤\s*(\d+)\s*KiB", text)
    assert flash and ram, f"TECHNICAL_DESIGN §9.1 size budget not parseable: {text!r}"
    return int(flash.group(1)), int(ram.group(1))


def _script_budget_kib() -> tuple[int, int]:
    text = _read(CHECK_FIRMWARE)
    flash = re.search(r"FABRAIL_FW_FLASH_BUDGET_KB:-(\d+)", text)
    ram = re.search(r"FABRAIL_FW_RAM_BUDGET_KB:-(\d+)", text)
    assert flash and ram, "scripts/check_firmware.sh: default budgets not found"
    return int(flash.group(1)), int(ram.group(1))


def test_firmware_budget_documented():
    """The firmware size budget scripts/check_firmware.sh enforces is the one
    TECHNICAL_DESIGN §9.1 states, and it leaves the 8 KiB parameter pages of the 512 KiB
    flash free (FAB-FW-010; lead decision D9: any budget change is stated, not hidden)."""
    doc, script = _documented_budget_kib(), _script_budget_kib()
    assert doc == script, f"TECHNICAL_DESIGN §9.1 says flash/RAM ≤ {doc} KiB, check_firmware.sh enforces {script} KiB"
    assert script[0] <= 512 - 8, f"flash budget {script[0]} KiB overlaps the parameter pages (≤ 504 KiB)"
    assert script[1] <= 128, f"RAM budget {script[1]} KiB exceeds the 128 KiB SRAM"


@pytest.mark.slow
@pytest.mark.requires_cargo
@pytest.mark.req("FAB-FW-010")
def test_firmware_link_size(tmp_path):
    """scripts/check_firmware.sh builds every class image in release for
    thumbv7em-none-eabihf, links it with rust-lld, and each image meets the flash and
    RAM budget stated in TECHNICAL_DESIGN §9.1 (built and linked; never flashed;
    FAB-FW-010)."""
    if _support.rustup_stable_rustc() is None:
        pytest.skip("rustup stable toolchain with thumbv7em-none-eabihf not installed (CLAUDE.md toolchain notes)")
    out = tmp_path / "firmware_size.json"
    r = _support.run_script(
        ["bash", str(CHECK_FIRMWARE), "--no-lint", "--json", str(out)],
        env={"CARGO_TARGET_DIR": str(_support.pytest_target_dir("firmware")), "CARGO_BUILD_JOBS": "3"},
        timeout=1800,
    )
    assert r.returncode == 0, f"check_firmware.sh failed (FAB-FW-010):\n{r.tail}"
    doc = json.loads(out.read_text(encoding="utf-8"))
    flash_kib, ram_kib = _documented_budget_kib()
    assert doc["schema"] == "fabrail.firmware_size/1"
    assert doc["flash_budget"] == flash_kib * 1024 and doc["ram_budget"] == ram_kib * 1024, doc
    classes = {img["class"] for img in doc["images"]}
    assert classes == {"door", "drawer", "wall", "heavy"}, f"images built: {sorted(classes)}"
    lines = []
    for img in doc["images"]:
        lines.append(f"{img['class']}: flash {img['flash']} B ({img['flash'] / 1024:.0f} KiB), RAM {img['ram']} B")
        assert img["ok"] and img["flash"] <= flash_kib * 1024 and img["ram"] <= ram_kib * 1024, (
            f"(T) {img['class']} image over budget: flash {img['flash']} B > {flash_kib} KiB or RAM {img['ram']} B > "
            f"{ram_kib} KiB (TECHNICAL_DESIGN §9.1; FAB-FW-010)"
        )
    print("\n".join(lines))


# ---------------------------------------------------------------------------- EXO-307


@pytest.mark.req("FAB-EXO-307")
def test_no_fault_injection_in_firmware():
    """The simulation-only fault injector is absent from the firmware manifest (outside
    comments), lockfile, sources and — when cargo is available — the resolved feature
    graph of every class build (FAB-EXO-307)."""
    bad = []
    manifest = "\n".join(line.split("#", 1)[0] for line in _read(FW / "Cargo.toml").splitlines())
    if FAULT_INJECTION_RE.search(manifest):
        bad.append("firmware/Cargo.toml names the fault-injection feature")
    if FAULT_INJECTION_RE.search(_read(FW / "Cargo.lock")):
        bad.append("firmware/Cargo.lock mentions fault-injection")
    for f in sorted((FW / "src").rglob("*.rs")):
        code = _strip_comments(_read(f))
        if FAULT_INJECTION_RE.search(code) or re.search(r"\b(Injector|CoreFault|inject)\b\s*[(:{]", code):
            bad.append(f"{_support.rel(f)}: uses the fault injector")
    deps = tomllib.loads(_read(FW / "Cargo.toml")).get("dependencies", {})
    for name, spec in deps.items():
        if isinstance(spec, dict) and any(FAULT_INJECTION_RE.search(x) for x in spec.get("features", [])):
            bad.append(f"firmware dependency {name} enables {spec['features']}")
    assert not bad, "fault injector in the firmware (FAB-EXO-307):\n" + "\n".join(bad)
    cargo = _support.cargo_bin()
    if cargo is None:
        pytest.skip("static checks passed; cargo not on PATH for the resolved-feature check")
    for cls in ("door", "drawer", "wall", "heavy"):
        feats = [] if cls == "door" else ["--no-default-features", "--features", f"class-{cls}"]
        r = _support.run_script(
            [
                cargo,
                "tree",
                "--manifest-path",
                str(FW / "Cargo.toml"),
                "-e",
                "features",
                "--target",
                "thumbv7em-none-eabihf",
                "--locked",
                "--offline",
                *feats,
            ],
            timeout=120,
        )
        assert r.returncode == 0, f"cargo tree (class-{cls}) failed:\n{r.tail}"
        assert "fabrail-rt" in r.stdout, f"cargo tree (class-{cls}) does not show fabrail-rt"
        assert not FAULT_INJECTION_RE.search(r.stdout), f"class-{cls}: resolved features include fault-injection"


# ---------------------------------------------------------------------------- FW-013


@pytest.mark.slow
@pytest.mark.requires_cargo
@pytest.mark.req("FAB-FW-013", "FAB-EXO-901")
def test_cross_check():
    """Real cross builds pass: every no_std crate for thumbv7em-none-eabihf (default and
    all features), fabrail-safety and fabrail-exo-safety also for thumbv6m-none-eabi and
    armv7r-none-eabihf (scripts/check_nostd.sh tier 4 --strict: a missing toolchain or
    target fails instead of skipping; FAB-FW-013, FAB-EXO-901)."""
    script = _read(CHECK_NOSTD)
    for c in NOSTD_CRATES:
        assert f"fabrail-{c}" in script.split("THUMBV7EM_CRATES")[0], f"check_nostd.sh NOSTD_CRATES lacks fabrail-{c}"
    assert re.search(r"SAFETY_CRATES=\(fabrail-safety fabrail-exo-safety\)", script)
    assert re.search(r"SAFETY_TARGETS=\(thumbv6m-none-eabi armv7r-none-eabihf\)", script)
    r = _support.run_script(
        ["bash", str(CHECK_NOSTD), "--tiers", "4", "--strict"],
        env={"CARGO_TARGET_DIR": str(_support.pytest_target_dir("nostd")), "FABRAIL_REQUIRE_CROSS": "1"},
        timeout=1800,
    )
    assert r.returncode == 0, f"(T) real cross builds failed (FAB-FW-013, FAB-EXO-901):\n{r.tail}"
    passed = re.findall(r"^\s*PASS\s+(.*)$", r.stdout, re.M)
    for target in ("thumbv7em-none-eabihf", "thumbv6m-none-eabi", "armv7r-none-eabihf"):
        assert any(target in p for p in passed), f"no passing cross build for {target}:\n{r.tail}"
    assert not re.search(r"^\s*SKIP\s", r.stdout, re.M), f"a cross build was skipped:\n{r.tail}"
