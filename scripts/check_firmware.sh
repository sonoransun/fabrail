#!/usr/bin/env bash
# scripts/check_firmware.sh: firmware build, link and size gate.
#
# Verifies FAB-FW-010 (release build for thumbv7em-none-eabihf, linked by rust-lld,
# llvm-size flash/RAM budget), FAB-EXO-307 (the simulation-only fault injector is absent
# from the firmware manifest, lockfile and resolved feature set) and the firmware half of
# FAB-FW-002 (`unsafe` only in firmware/src/bsp, each block with a SAFETY comment). The
# firmware is built and linked here; it has never been flashed (FAB-FW-012).
#
# Steps
#   1  Toolchain: rustup stable with the thumbv7em-none-eabihf target. Homebrew rustc
#      comes first on PATH and has no thumbv7em sysroot, so RUSTC is set to the
#      toolchain's rustc and cargo runs through `rustup run stable` (CLAUDE.md).
#   2  Source policy: no `unsafe` outside firmware/src/bsp (also enforced by
#      #![deny(unsafe_code)] in main.rs), a `// SAFETY:` comment before every unsafe
#      block and a `# Safety` section on every unsafe fn in bsp/.
#   3  Feature hygiene: fabrail-rt's fault-injection feature appears neither in
#      firmware/Cargo.toml (outside comments) nor in firmware/Cargo.lock nor in the
#      resolved feature graph (`cargo tree -e features`).
#   4  rustfmt --check and clippy -D warnings on the firmware crate (skip: --no-lint).
#   5  `cargo build --release` for every class feature (door is the default; drawer,
#      wall and heavy with --no-default-features); each ELF is copied to
#      <target>/thumbv7em-none-eabihf/release/fabrail-fw-<class>.elf. memory.x asserts at
#      link time that the image ends below the parameter A/B pages. The `dev` image
#      (defmt/RTT, panic-probe) is linked as well, without a size gate.
#   6  Size: flash = text + data, RAM = data + bss (Berkeley llvm-size; .bss includes
#      the RTIC resources and the crash record in .uninit). Tools tried in order:
#      $LLVM_SIZE, Homebrew llvm-size, llvm-size / rust-size on PATH, the toolchain's
#      llvm-tools; otherwise the ELF section table is summed with python3 (same rule).
#
# Budgets (override with FABRAIL_FW_FLASH_BUDGET_KB / FABRAIL_FW_RAM_BUDGET_KB):
#   flash ≤ 320 KiB  (W2 lead decision D9; firmware/README.md "Size"). The release image
#                    is built at opt-level "z": ≈ 280 KB door / ≈ 282 KB other classes
#                    (2026-09-27; "s" would be ≈ 320 KB). The design-stage 256 KiB (one
#                    256 KiB bank of the 512 KiB G474RE, leaving the other bank for an
#                    A/B image) is not reached: the rest is init-time code of the shared
#                    crates (f64 soft-float gain design, 368-field validation table and
#                    strings, postcard decoder). 320 KiB = 62.5 % of the device, ≈ 15 %
#                    headroom over the measured image for Wave 2/3 growth (the shared
#                    crates grew ≈ 12 KB during Wave 2 alone). Hard limit 504 KiB: the
#                    top 8 KiB hold the parameter pages (memory.x ASSERT).
#   RAM   ≤  64 KiB  static RAM; the remaining ~108 KiB of the 128 KiB SRAM is stack.
#
# Usage
#   scripts/check_firmware.sh [--classes "door drawer wall heavy"] [--no-lint] [--json PATH] [-h]
#
# Environment
#   CARGO_TARGET_DIR   default: target/firmware (separate from host builds: other rustc)
#   CARGO_BUILD_JOBS   default 3
#
# Output: one "FIRMWARE_SIZE class=… flash=… ram=… flash_budget=… ram_budget=… ok=…"
# line per class (parsed by tests), a summary, exit 0 only if every step passed.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FW="$ROOT/firmware"
TARGET=thumbv7em-none-eabihf

CLASSES="door drawer wall heavy"
LINT=1
JSON_OUT=""
FLASH_BUDGET_KB="${FABRAIL_FW_FLASH_BUDGET_KB:-320}"
RAM_BUDGET_KB="${FABRAIL_FW_RAM_BUDGET_KB:-64}"
export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-3}"
TD="${CARGO_TARGET_DIR:-$ROOT/target/firmware}"
export CARGO_TARGET_DIR="$TD"
mkdir -p "$TD"

usage() { sed -n '2,52p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --classes) CLASSES="${2:?--classes needs a list}"; shift 2 ;;
        --classes=*) CLASSES="${1#--classes=}"; shift ;;
        --no-lint) LINT=0; shift ;;
        --json) JSON_OUT="${2:?--json needs a path}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

FAILS=0
pass() { printf '  PASS  %s\n' "$*"; }
fail() { printf '  FAIL  %s\n' "$*"; FAILS=$((FAILS + 1)); }
step() { printf '\n== %s\n' "$*"; }

# ---------------------------------------------------------------------------------------
step "1  toolchain"
if ! command -v rustup >/dev/null 2>&1; then
    fail "rustup not found (brew install rustup; see CLAUDE.md)"
    echo; echo "check_firmware: FAILED (no toolchain)"; exit 1
fi
RUSTC_TC="$(rustup which rustc --toolchain stable 2>/dev/null || true)"
if [ -z "$RUSTC_TC" ] || [ ! -x "$RUSTC_TC" ]; then
    fail "rustup stable toolchain missing (rustup toolchain install stable)"
    echo; echo "check_firmware: FAILED (no toolchain)"; exit 1
fi
export RUSTC="$RUSTC_TC"
export RUSTUP_AUTO_INSTALL=0
SYSROOT="$("$RUSTC" --print sysroot)"
if [ -d "$SYSROOT/lib/rustlib/$TARGET/lib" ]; then
    pass "rustc $("$RUSTC" --version | awk '{print $2}') with $TARGET"
else
    fail "$TARGET not installed (rustup target add $TARGET)"
    echo; echo "check_firmware: FAILED (no target)"; exit 1
fi
TC_BIN="$(dirname "$RUSTC")"
cargo_tc() { (cd "$FW" && rustup run stable cargo "$@"); }

# ---------------------------------------------------------------------------------------
step "2  unsafe policy (firmware/src)"
if python3 - "$FW/src" <<'PY'
import pathlib, re, sys
src = pathlib.Path(sys.argv[1])
bad = []
for f in sorted(src.rglob("*.rs")):
    rel = f.relative_to(src).as_posix()
    lines = f.read_text().splitlines()
    for i, line in enumerate(lines):
        code = line.split("//", 1)[0]
        if not re.search(r"\bunsafe\b", code):
            continue
        if re.search(r"#!?\[\s*(allow|deny|forbid|warn)\(unsafe_code\)", code):
            continue
        if not rel.startswith("bsp/"):
            bad.append(f"{rel}:{i+1}: unsafe outside bsp/")
            continue
        # Attributes such as #[unsafe(link_section = ...)] need no comment of their own.
        if re.search(r"#\[\s*unsafe\(", code):
            continue
        back = lines[max(0, i - 12):i + 1]
        if re.search(r"\bunsafe\s+(extern\s+\"C\"\s+)?fn\b", code):
            if not any("# Safety" in l or "SAFETY:" in l for l in back):
                bad.append(f"{rel}:{i+1}: unsafe fn without a '# Safety' section")
        elif not any("SAFETY:" in l for l in back[-8:]):
            bad.append(f"{rel}:{i+1}: unsafe block without a '// SAFETY:' comment")
for b in bad:
    print("   ", b)
sys.exit(1 if bad else 0)
PY
then
    pass "unsafe only in bsp/, every block/fn justified"
else
    fail "unsafe policy"
fi

# ---------------------------------------------------------------------------------------
step "3  simulation-only fault injector absent"
FI='fault[-_]injection'
if grep -v '^[[:space:]]*#' "$FW/Cargo.toml" | grep -Eq "$FI"; then
    fail "firmware/Cargo.toml enables or names the fault injector"
else
    pass "firmware/Cargo.toml"
fi
if [ -f "$FW/Cargo.lock" ] && grep -Eq "$FI" "$FW/Cargo.lock"; then
    fail "firmware/Cargo.lock mentions the fault injector"
else
    pass "firmware/Cargo.lock"
fi
TREE_OK=1
for c in $CLASSES; do
    if [ "$c" = door ]; then feats=(); else feats=(--no-default-features --features "class-$c"); fi
    if ! tree="$(cargo_tc tree -e features --target "$TARGET" ${feats[@]+"${feats[@]}"} 2>&1)"; then
        echo "$tree" | tail -5
        fail "cargo tree ($c)"; TREE_OK=0; continue
    fi
    if echo "$tree" | grep -Eq "$FI"; then
        fail "resolved features of class-$c include the fault injector"; TREE_OK=0
    fi
done
[ "$TREE_OK" = 1 ] && pass "resolved feature graph (classes: $CLASSES)"

# ---------------------------------------------------------------------------------------
if [ "$LINT" = 1 ]; then
    step "4  rustfmt + clippy (firmware)"
    if cargo_tc fmt --check >/dev/null 2>&1; then pass "rustfmt"; else fail "rustfmt (run: cd firmware && rustup run stable cargo fmt)"; fi
    # clippy-driver must come from the toolchain (PATH first), not Homebrew.
    if (cd "$FW" && rustup run stable env PATH="$TC_BIN:$PATH" cargo clippy --release -- -D warnings) \
        >"$TD/clippy.log" 2>&1; then
        pass "clippy -D warnings (class-door)"
    else
        tail -20 "$TD/clippy.log" || true; fail "clippy"
    fi
else
    step "4  lint skipped (--no-lint)"
fi

# ---------------------------------------------------------------------------------------
step "5  release build + link per class"
REL="$TD/$TARGET/release"
BUILT=()
for c in $CLASSES; do
    if [ "$c" = door ]; then feats=(); else feats=(--no-default-features --features "class-$c"); fi
    if cargo_tc build --release ${feats[@]+"${feats[@]}"} >"$TD/build-$c.log" 2>&1; then
        cp "$REL/fabrail-fw" "$REL/fabrail-fw-$c.elf"
        BUILT+=("$c")
        pass "class-$c linked → $(basename "$REL")/fabrail-fw-$c.elf"
    else
        grep -E '^(error|warning)' -A 6 "$TD/build-$c.log" | head -30
        fail "class-$c build/link (log: $TD/build-$c.log)"
    fi
done
# The development image (defmt over RTT + panic-probe) must link too; not size-gated.
if cargo_tc build --release --features dev >"$TD/build-dev.log" 2>&1; then
    cp "$REL/fabrail-fw" "$REL/fabrail-fw-door-dev.elf"
    pass "class-door + dev linked (defmt.x via build.rs)"
else
    grep -E '^(error|warning)' -A 6 "$TD/build-dev.log" | head -30
    fail "dev build/link (log: $TD/build-dev.log)"
fi

# ---------------------------------------------------------------------------------------
step "6  size budget (flash ≤ ${FLASH_BUDGET_KB} KiB, RAM ≤ ${RAM_BUDGET_KB} KiB)"
SIZE_TOOL=""
for cand in "${LLVM_SIZE:-}" /opt/homebrew/opt/llvm/bin/llvm-size "$(command -v llvm-size || true)" \
            "$(command -v rust-size || true)" "$(ls "$SYSROOT"/lib/rustlib/*/bin/llvm-size 2>/dev/null | head -1)"; do
    if [ -n "$cand" ] && [ -x "$cand" ]; then SIZE_TOOL="$cand"; break; fi
done
berkeley() {
    # prints "text data bss" for an ELF
    if [ -n "$SIZE_TOOL" ]; then
        "$SIZE_TOOL" "$1" | awk 'NR==2 {print $1, $2, $3}'
    else
        python3 - "$1" <<'PY'
import struct, sys
b = open(sys.argv[1], "rb").read()
assert b[:4] == b"\x7fELF" and b[4] == 1, "ELF32 expected"
shoff, = struct.unpack_from("<I", b, 0x20)
shentsize, shnum = struct.unpack_from("<HH", b, 0x2E)
text = data = bss = 0
for k in range(shnum):
    name, typ, flags, addr, off, size = struct.unpack_from("<IIIIII", b, shoff + k * shentsize)
    if not flags & 0x2:          # SHF_ALLOC
        continue
    if typ == 8:                 # SHT_NOBITS
        bss += size
    elif flags & 0x1:            # SHF_WRITE
        data += size
    else:
        text += size
print(text, data, bss)
PY
    fi
}
echo "  size tool: ${SIZE_TOOL:-python3 ELF section sum}"
JSON_ROWS=""
for c in ${BUILT[@]+"${BUILT[@]}"}; do
    read -r text data bss < <(berkeley "$REL/fabrail-fw-$c.elf")
    flash=$((text + data)); ram=$((data + bss))
    fb=$((FLASH_BUDGET_KB * 1024)); rb=$((RAM_BUDGET_KB * 1024))
    ok=1
    [ "$flash" -le "$fb" ] || ok=0
    [ "$ram" -le "$rb" ] || ok=0
    printf 'FIRMWARE_SIZE class=%s flash=%d ram=%d flash_budget=%d ram_budget=%d ok=%d\n' \
        "$c" "$flash" "$ram" "$fb" "$rb" "$ok"
    msg="class-$c flash $flash B ($((flash * 100 / fb)) % of budget), RAM $ram B ($((ram * 100 / rb)) %)"
    if [ "$ok" = 1 ]; then pass "$msg"; else fail "$msg"; fi
    JSON_ROWS="$JSON_ROWS{\"class\":\"$c\",\"flash\":$flash,\"ram\":$ram,\"text\":$text,\"data\":$data,\"bss\":$bss,\"ok\":$([ $ok = 1 ] && echo true || echo false)},"
done
if [ ${#BUILT[@]} -eq 0 ]; then fail "no image to measure"; fi

if [ -n "$JSON_OUT" ]; then
    mkdir -p "$(dirname "$JSON_OUT")"
    printf '{"schema":"fabrail.firmware_size/1","target":"%s","flash_budget":%d,"ram_budget":%d,"images":[%s]}\n' \
        "$TARGET" "$((FLASH_BUDGET_KB * 1024))" "$((RAM_BUDGET_KB * 1024))" "${JSON_ROWS%,}" >"$JSON_OUT"
    echo "  wrote $JSON_OUT"
fi

echo
if [ "$FAILS" -eq 0 ]; then
    echo "check_firmware: OK (built and linked; never flashed)"
    exit 0
fi
echo "check_firmware: $FAILS check(s) FAILED"
exit 1
