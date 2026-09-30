#!/usr/bin/env bash
# scripts/check_nostd.sh: portability gate for the no_std crates.
#
# Verifies FAB-FW-002 (source guards, dependency allowlist), FAB-FW-013 / FAB-EXO-901
# (real cross builds), FAB-EXO-902 (no_std + forbid(unsafe_code)) and FAB-EXO-116
# (transitive dependency closure of the safety crates). See docs/SPECIFICATION.md.
#
# Tiers
#   1  Source guards (grep, best effort): no std, no alloc, no num_traits or nalgebra,
#      no cfg(target_arch), no mul_add, no direct libm and no float method math
#      (.sin( .exp( .tanh( .sqrt( …) outside fabrail-types/src/fmath.rs. #[cfg(test)]
#      items and comments are stripped first. Also checks the crate attributes and that
#      no manifest uses `workspace = true`. A user-defined method that shares a float
#      method's name (e.g. a Complex::exp built on fmath) is exempted from the math
#      guards, and only those, by a trailing `// nostd-allow: <reason>` on its line;
#      prefer a distinct name (cexp) where possible.
#   2  Per-crate isolated host builds (each crate alone; default and all features).
#   3  thumbv7em dependency allowlist (`cargo tree -e normal`), and no dependency built
#      with a "std", "alloc" or libm "arch" feature (the host proc-macro chain is exempt).
#   4  Real cross builds with the rustup stable toolchain: control crates for
#      thumbv7em-none-eabihf; fabrail-safety and fabrail-exo-safety also for
#      thumbv6m-none-eabi and armv7r-none-eabihf. Fallback when rustup or a target is
#      missing: FABRAIL_BUILD_STD=1 uses Homebrew rust-src with RUSTC_BOOTSTRAP=1
#      -Zbuild-std=core; otherwise the build is SKIPped with a reason.
#   5  Transitive dependency closure (cargo metadata, normal + build edges): the safety
#      crates may reach fabrail-types and fabrail-params only.
#
# Usage
#   scripts/check_nostd.sh [--tiers "1 2 3 4 5"] [--strict] [-h]
#     --tiers   run only the listed tiers (default: all)
#     --strict  a SKIP in tier 4 counts as a failure (same as FABRAIL_REQUIRE_CROSS=1)
#
# Environment
#   CARGO_TARGET_DIR   base target dir (default: target/nostd); tiers use subdirectories
#   CARGO_BUILD_JOBS   default 3
#   FABRAIL_REQUIRE_CROSS=1, FABRAIL_BUILD_STD=1   see above
#
# Toolchain note (CLAUDE.md): Homebrew rustc comes first on PATH and has only the host
# sysroot, so `rustup run stable cargo` alone fails with "can't find crate for core".
# Tier 4 therefore sets RUSTC to the toolchain's rustc and uses its own target dir.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# ---------------------------------------------------------------------------------------
# configuration
# ---------------------------------------------------------------------------------------

NOSTD_CRATES=(
    fabrail-types fabrail-params fabrail-hal fabrail-core
    fabrail-safety fabrail-rt fabrail-exo-safety fabrail-exo
)
# Control crates built for the Cortex-M4F/M7 target.
THUMBV7EM_CRATES=("${NOSTD_CRATES[@]}")
# Safety crates also built for Cortex-M0+ (diverse monitor MCU) and Cortex-R5F (CH-B).
SAFETY_CRATES=(fabrail-safety fabrail-exo-safety)
SAFETY_TARGETS=(thumbv6m-none-eabi armv7r-none-eabihf)

# Tier 3 allowlist (normal-edge dependency names for thumbv7em).
ALLOW_RE='^(fabrail-(types|params|hal|core|safety|rt|exo-safety|exo)|libm|serde|serde_core|serde_derive|proc-macro2|quote|syn|unicode-ident|heapless|hash32|stable_deref_trait|byteorder)$'
# Host-only proc-macro chain: may carry std features (runs on the build machine).
PROC_MACRO_RE='^(proc-macro2|quote|syn|unicode-ident|serde_derive)$'
# Tier 5: the only fabrail crates the safety crates may reach.
SAFETY_ALLOWED_FAB='fabrail-types fabrail-params'

TIERS="1 2 3 4 5"
STRICT="${FABRAIL_REQUIRE_CROSS:-0}"
BUILD_STD="${FABRAIL_BUILD_STD:-0}"
export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-3}"
BASE_TARGET_DIR="${CARGO_TARGET_DIR:-$ROOT/target/nostd}"
unset CARGO_TARGET_DIR   # set per tier below

usage() { sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --tiers) TIERS="${2:?--tiers needs a list}"; shift 2 ;;
        --tiers=*) TIERS="${1#--tiers=}"; shift ;;
        --strict) STRICT=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "check_nostd: unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

want_tier() { case " $TIERS " in *" $1 "*) return 0 ;; *) return 1 ;; esac; }

# ---------------------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------------------

N_PASS=0; N_FAIL=0; N_SKIP=0
SUMMARY=()

pass() { N_PASS=$((N_PASS + 1)); SUMMARY+=("PASS  $1"); echo "  PASS  $1"; }
fail() { N_FAIL=$((N_FAIL + 1)); SUMMARY+=("FAIL  $1"); echo "  FAIL  $1"; }
skip() {
    if [ "$STRICT" = "1" ]; then
        fail "$1 (SKIP not allowed with --strict: $2)"
    else
        N_SKIP=$((N_SKIP + 1)); SUMMARY+=("SKIP  $1 ($2)"); echo "  SKIP  $1 ($2)"
    fi
}
header() { echo; echo "== tier $1: $2"; }

# Run a command quietly; on failure print the last 40 lines of its output.
run_quiet() {
    local log
    log="$(mktemp -t fabrail_nostd.XXXXXX)"
    if "$@" >"$log" 2>&1; then
        rm -f "$log"; return 0
    else
        local rc=$?
        echo "    \$ $*" >&2
        tail -n 40 "$log" | sed 's/^/    | /' >&2
        rm -f "$log"; return "$rc"
    fi
}

# ---------------------------------------------------------------------------------------
# tier 1: source guards
# ---------------------------------------------------------------------------------------

# Emits "file:line:code" for every code line outside #[cfg(test)] items, with // and /* */
# comments removed. Best effort: braces inside string or char literals can confuse it.
read -r -d '' STRIP_AWK <<'AWK' || true
FNR == 1 { skip = 0; depth = 0; seen_open = 0; pending = 0; inblock = 0; skipfile = 0 }
skipfile { next }
{
    line = $0
    allow = (index($0, "nostd-allow:") > 0)
    if (inblock) {
        e = index(line, "*/")
        if (e == 0) next
        line = substr(line, e + 2); inblock = 0
    }
    while (match(line, /\/\*/)) {
        pre = substr(line, 1, RSTART - 1); rest = substr(line, RSTART + 2)
        e = index(rest, "*/")
        if (e) { line = pre substr(rest, e + 2) } else { line = pre; inblock = 1 }
    }
    sub(/\/\/.*$/, "", line)
    if (line ~ /#!\[cfg\(test\)\]/) { skipfile = 1; next }
    if (skip) {
        o = gsub(/\{/, "{", line); c = gsub(/\}/, "}", line)
        depth += o - c
        if (o > 0) seen_open = 1
        if (seen_open && depth <= 0) skip = 0
        else if (!seen_open && line ~ /;[[:space:]]*$/) skip = 0
        next
    }
    if (line ~ /#\[cfg\((all\()?test[,)]/) {
        pending = 1
        sub(/#\[cfg\([^]]*\]/, "", line)
        if (line ~ /^[[:space:]]*$/) next
    }
    if (pending) {
        if (line ~ /^[[:space:]]*(#\[.*)?$/) next
        pending = 0
        o = gsub(/\{/, "{", line); c = gsub(/\}/, "}", line)
        if (o > c) { skip = 1; depth = o - c; seen_open = 1 }
        else if (o == 0 && line !~ /;[[:space:]]*$/) { skip = 1; depth = 0; seen_open = 0 }
        next
    }
    if (line ~ /^[[:space:]]*$/) next
    if (allow) line = line " /*nostd-allow*/"
    print FILENAME ":" FNR ":" line
}
AWK

# Collect the stripped non-test code of every no_std crate once.
tier1() {
    header 1 "source guards (no_std crates)"
    local code_all code_math
    code_all="$(mktemp -t fabrail_nostd_code.XXXXXX)"
    code_math="$(mktemp -t fabrail_nostd_math.XXXXXX)"

    local crate dir lib manifest files=()
    for crate in "${NOSTD_CRATES[@]}"; do
        dir="crates/$crate"
        lib="$dir/src/lib.rs"
        manifest="$dir/Cargo.toml"
        if [ ! -f "$lib" ]; then fail "$crate: $lib missing"; continue; fi

        if grep -Eq '^[[:space:]]*#!\[cfg_attr\(not\(test\),[[:space:]]*no_std\)\]' "$lib"; then
            pass "$crate: #![cfg_attr(not(test), no_std)]"
        else
            fail "$crate: lib.rs lacks #![cfg_attr(not(test), no_std)]"
        fi
        if grep -Fq '#![forbid(unsafe_code)]' "$lib"; then
            pass "$crate: #![forbid(unsafe_code)]"
        else
            fail "$crate: lib.rs lacks #![forbid(unsafe_code)]"
        fi
        if grep -Eq '(^|\.)workspace[[:space:]]*=[[:space:]]*true' "$manifest"; then
            fail "$crate: Cargo.toml uses workspace inheritance (firmware consumes it by path)"
        else
            pass "$crate: no workspace inheritance"
        fi

        # Source files outside test-only conventions (tests.rs, test_*.rs, src/**/tests/).
        while IFS= read -r f; do files+=("$f"); done < <(
            find "$dir/src" -name '*.rs' \
                ! -name 'tests.rs' ! -name 'test_*.rs' ! -path '*/tests/*' | LC_ALL=C sort
        )
    done

    if [ "${#files[@]}" -eq 0 ]; then
        fail "no no_std sources found"; rm -f "$code_all" "$code_math"; return
    fi
    awk "$STRIP_AWK" "${files[@]}" >"$code_all"
    # fmath.rs is the one place libm and float method names are allowed. Lines marked
    # `// nostd-allow: <reason>` are exempt from the math guards only (never from std/alloc).
    grep -v '^crates/fabrail-types/src/fmath\.rs:' "$code_all" \
        | grep -v '/\*nostd-allow\*/$' >"$code_math" || true

    guard() {  # guard <label> <file> <ERE>
        local label="$1" file="$2" re="$3" hits
        hits="$(grep -En -- "$re" "$file" | cut -d: -f2- || true)"
        if [ -z "$hits" ]; then
            pass "$label"
        else
            fail "$label"
            echo "$hits" | head -n 20 | sed 's/^/    > /'
            local n; n="$(printf '%s\n' "$hits" | wc -l | tr -d ' ')"
            if [ "$n" -gt 20 ]; then echo "    > … $((n - 20)) more"; fi
        fi
        return 0
    }

    guard "no std paths (use std:: / std::…)"      "$code_all"  '(^|[^[:alnum:]_:])std::'
    guard "no extern crate std / alloc"             "$code_all"  'extern[[:space:]]+crate[[:space:]]+(std|alloc)'
    guard "no use alloc::"                          "$code_all"  '(^|[^[:alnum:]_:])use[[:space:]]+alloc::'
    guard "no std-only macros (print!, dbg!, format!, vec!)" "$code_all" '(^|[^[:alnum:]_])(e?println|e?print|dbg|format|vec)![[:space:]]*[([]'
    guard "no num_traits"                           "$code_all"  'num_traits'
    guard "no nalgebra"                             "$code_all"  'nalgebra'
    guard "no cfg(target_arch)"                     "$code_all"  'cfg\([^)]*target_arch'
    guard "no mul_add"                              "$code_all"  'mul_add'
    guard "no direct libm outside fmath.rs"         "$code_math" '(^|[^[:alnum:]_])libm::'
    guard "no float method math outside fmath.rs"   "$code_math" '\.(sin|cos|tan|asin|acos|atan|atan2|sin_cos|sinh|cosh|tanh|asinh|acosh|atanh|exp|exp2|exp_m1|ln|ln_1p|log2|log10|powi|powf|sqrt|cbrt|hypot|floor|ceil|round|round_ties_even|trunc|fract|rem_euclid_f)[[:space:]]*\('
    guard "no UFCS float math outside fmath.rs"     "$code_math" '(^|[^[:alnum:]_])f(32|64)::(sin|cos|tan|asin|acos|atan|atan2|sinh|cosh|tanh|exp|exp2|exp_m1|ln|ln_1p|log2|log10|powi|powf|sqrt|cbrt|hypot|floor|ceil|round|trunc|fract|mul_add)[[:space:]]*\('

    rm -f "$code_all" "$code_math"
}

# ---------------------------------------------------------------------------------------
# tier 2: per-crate isolated host builds
# ---------------------------------------------------------------------------------------

tier2() {
    header 2 "per-crate isolated host builds"
    local crate
    for crate in "${NOSTD_CRATES[@]}"; do
        if run_quiet env CARGO_TARGET_DIR="$BASE_TARGET_DIR/host" \
                cargo build --locked -p "$crate" --lib; then
            pass "$crate (default features)"
        else
            fail "$crate (default features)"
        fi
        if run_quiet env CARGO_TARGET_DIR="$BASE_TARGET_DIR/host" \
                cargo build --locked -p "$crate" --lib --all-features; then
            pass "$crate (all features)"
        else
            fail "$crate (all features)"
        fi
    done
}

# ---------------------------------------------------------------------------------------
# tier 3: thumbv7em dependency allowlist and feature check
# ---------------------------------------------------------------------------------------

tier3() {
    header 3 "thumbv7em-none-eabihf dependency allowlist"
    local crate deps bad feats badf
    for crate in "${NOSTD_CRATES[@]}"; do
        if ! deps="$(cargo tree --locked -p "$crate" -e normal --all-features \
                --target thumbv7em-none-eabihf --prefix none --format '{p}' 2>&1)"; then
            fail "$crate: cargo tree failed"; echo "$deps" | tail -n 10 | sed 's/^/    | /'; continue
        fi
        bad="$(printf '%s\n' "$deps" | awk 'NF { print $1 }' | LC_ALL=C sort -u \
               | grep -Ev "$ALLOW_RE" || true)"
        if [ -z "$bad" ]; then
            pass "$crate: dependencies within allowlist"
        else
            fail "$crate: dependencies outside allowlist: $(echo "$bad" | tr '\n' ' ')"
        fi

        if ! feats="$(cargo tree --locked -p "$crate" -e normal,features --all-features \
                --target thumbv7em-none-eabihf --prefix none 2>&1)"; then
            fail "$crate: cargo tree -e features failed"; continue
        fi
        # Lines look like: serde feature "std"
        badf="$(printf '%s\n' "$feats" | awk -v pm="$PROC_MACRO_RE" '
            $2 == "feature" {
                f = $3; gsub(/"/, "", f)
                if ($1 == "libm" && f == "arch") { print $1 " feature " f; next }
                if ((f == "std" || f == "alloc") && $1 !~ pm) print $1 " feature " f
            }' | LC_ALL=C sort -u)"
        if [ -z "$badf" ]; then
            pass "$crate: no std/alloc/libm-arch feature on target dependencies"
        else
            fail "$crate: forbidden features: $(echo "$badf" | tr '\n' ';')"
        fi
    done
}

# ---------------------------------------------------------------------------------------
# tier 4: real cross builds
# ---------------------------------------------------------------------------------------

find_rustup() {
    local c
    if command -v brew >/dev/null 2>&1; then
        c="$(brew --prefix rustup 2>/dev/null || true)/bin/rustup"
        if [ -x "$c" ]; then echo "$c"; return 0; fi
    fi
    c="$(command -v rustup 2>/dev/null || true)"
    if [ -n "$c" ]; then echo "$c"; return 0; fi
    if [ -x "$HOME/.cargo/bin/rustup" ]; then echo "$HOME/.cargo/bin/rustup"; return 0; fi
    return 1
}

# cross_build <label> <target> <extra cargo args...> -- <crates...>
cross_build() {
    local label="$1" target="$2"; shift 2
    local extra=() crates=()
    while [ $# -gt 0 ] && [ "$1" != "--" ]; do extra+=("$1"); shift; done
    if [ $# -gt 0 ]; then shift; fi
    crates=("$@")
    local pargs=() c
    for c in "${crates[@]}"; do pargs+=(-p "$c"); done

    if [ -n "${TC_RUSTC:-}" ] && [ -d "$TC_SYSROOT/lib/rustlib/$target/lib" ]; then
        if run_quiet env RUSTUP_AUTO_INSTALL=0 RUSTC="$TC_RUSTC" \
                CARGO_TARGET_DIR="$BASE_TARGET_DIR/cross" \
                "$RUSTUP" run stable cargo build --locked --lib --target "$target" \
                ${extra[@]+"${extra[@]}"} "${pargs[@]}"; then
            pass "$label [$target, rustup stable]"
        else
            fail "$label [$target, rustup stable]"
        fi
        return
    fi

    local reason
    if [ -z "${TC_RUSTC:-}" ]; then
        reason="rustup stable toolchain not found"
    else
        reason="target $target not installed (rustup target add $target --toolchain stable)"
    fi

    if [ "$BUILD_STD" = "1" ]; then
        local src
        src="$(rustc --print sysroot 2>/dev/null)/lib/rustlib/src/rust/library"
        if [ -d "$src" ]; then
            if run_quiet env RUSTC_BOOTSTRAP=1 CARGO_TARGET_DIR="$BASE_TARGET_DIR/buildstd" \
                    cargo build --locked --lib -Zbuild-std=core --target "$target" \
                    ${extra[@]+"${extra[@]}"} "${pargs[@]}"; then
                pass "$label [$target, -Zbuild-std=core fallback]"
            else
                fail "$label [$target, -Zbuild-std=core fallback]"
            fi
            return
        fi
        reason="$reason; FABRAIL_BUILD_STD=1 but no rust-src at $src"
    else
        reason="$reason; set FABRAIL_BUILD_STD=1 to try RUSTC_BOOTSTRAP=1 -Zbuild-std=core"
    fi
    skip "$label [$target]" "$reason"
}

tier4() {
    header 4 "real cross builds"
    TC_RUSTC=""; TC_SYSROOT=""; RUSTUP=""
    if RUSTUP="$(find_rustup)"; then
        if TC_RUSTC="$(RUSTUP_AUTO_INSTALL=0 "$RUSTUP" which rustc --toolchain stable 2>/dev/null)" \
                && [ -x "$TC_RUSTC" ]; then
            TC_SYSROOT="$("$TC_RUSTC" --print sysroot)"
            echo "  rustup: $RUSTUP; toolchain rustc: $TC_RUSTC ($("$TC_RUSTC" --version))"
        else
            TC_RUSTC=""
        fi
    fi

    cross_build "control crates" thumbv7em-none-eabihf -- "${THUMBV7EM_CRATES[@]}"
    cross_build "control crates, all features" thumbv7em-none-eabihf --all-features -- "${THUMBV7EM_CRATES[@]}"
    local t
    for t in "${SAFETY_TARGETS[@]}"; do
        cross_build "safety crates" "$t" -- "${SAFETY_CRATES[@]}"
    done
}

# ---------------------------------------------------------------------------------------
# tier 5: transitive dependency closure of the safety crates
# ---------------------------------------------------------------------------------------

tier5() {
    header 5 "safety-crate dependency closure (cargo metadata)"
    local meta out
    meta="$(mktemp -t fabrail_nostd_meta.XXXXXX)"
    if ! cargo metadata --locked --format-version 1 >"$meta" 2>/dev/null; then
        fail "cargo metadata failed"; rm -f "$meta"; return
    fi
    if command -v python3 >/dev/null 2>&1; then
        if out="$(python3 - "$meta" "$SAFETY_ALLOWED_FAB" "${SAFETY_CRATES[@]}" <<'PY'
import json, sys

meta = json.load(open(sys.argv[1]))
allowed = set(sys.argv[2].split())
roots = sys.argv[3:]
names = {p["id"]: p["name"] for p in meta["packages"]}
nodes = {n["id"]: n for n in meta["resolve"]["nodes"]}
status = 0
for root in roots:
    ids = [i for i in nodes if names[i] == root]
    if not ids:
        print(f"FAIL {root}: not in the workspace resolve graph")
        status = 1
        continue
    seen, stack = set(), list(ids)
    while stack:
        cur = stack.pop()
        for dep in nodes[cur]["deps"]:
            kinds = {k.get("kind") for k in dep.get("dep_kinds", [])}
            if kinds & {None, "build"} and dep["pkg"] not in seen:
                seen.add(dep["pkg"])
                stack.append(dep["pkg"])
    fab = sorted({names[i] for i in seen if names[i].startswith("fabrail-")})
    bad = [n for n in fab if n not in allowed]
    if bad:
        print(f"FAIL {root}: closure reaches {', '.join(bad)}")
        status = 1
    else:
        print(f"PASS {root}: fabrail closure = {', '.join(fab) or '(none)'}")
sys.exit(status)
PY
        )"; then :; fi
    else
        # Fallback without python3: cargo tree over normal + build edges.
        local r lines
        out=""
        for r in "${SAFETY_CRATES[@]}"; do
            lines="$(cargo tree --locked -p "$r" -e normal,build --prefix none --format '{p}' \
                     | awk '{ print $1 }' | grep '^fabrail-' | grep -vx "$r" | LC_ALL=C sort -u || true)"
            local bad=""
            for l in $lines; do
                case " $SAFETY_ALLOWED_FAB " in *" $l "*) ;; *) bad="$bad $l" ;; esac
            done
            if [ -z "$bad" ]; then out+="PASS $r: fabrail closure ok (cargo tree)"$'\n'
            else out+="FAIL $r: closure reaches$bad"$'\n'; fi
        done
    fi
    rm -f "$meta"
    local line
    while IFS= read -r line; do
        [ -z "$line" ] && continue
        case "$line" in
            PASS\ *) pass "${line#PASS }" ;;
            *) fail "${line#FAIL }" ;;
        esac
    done <<<"$out"
}

# ---------------------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------------------

echo "check_nostd: tiers [$TIERS], strict=$STRICT, target dir $BASE_TARGET_DIR, jobs $CARGO_BUILD_JOBS"
want_tier 1 && tier1
want_tier 2 && tier2
want_tier 3 && tier3
want_tier 4 && tier4
want_tier 5 && tier5

echo
echo "== summary: $N_PASS passed, $N_FAIL failed, $N_SKIP skipped"
if [ "$N_FAIL" -gt 0 ]; then
    printf '%s\n' "${SUMMARY[@]}" | grep '^FAIL' | sed 's/^/  /'
    exit 1
fi
if [ "$N_SKIP" -gt 0 ]; then
    printf '%s\n' "${SUMMARY[@]}" | grep '^SKIP' | sed 's/^/  /'
fi
exit 0
