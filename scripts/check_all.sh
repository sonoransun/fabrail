#!/usr/bin/env bash
# scripts/check_all.sh: the full local gate (plan §5 C3 and §6 "Verification").
#
# Runs, in order, and reports one PASS/FAIL line per step:
#   fmt        cargo fmt --all --check                                  (host workspace)
#   clippy     cargo clippy --workspace --all-targets --all-features -- -D warnings
#   test       cargo test --profile itest                               (default members)
#   nostd      scripts/check_nostd.sh --strict  (FABRAIL_REQUIRE_CROSS=1: real cross
#              builds are a hard gate, plan G0)
#   firmware   scripts/check_firmware.sh        (build + link + size; never flashed)
#   maturin    maturin develop --release --offline into .venv            (skip: --no-maturin)
#   pytest     .venv/bin/python -m pytest -q                              (not-slow tier)
#   slow       .venv/bin/python -m pytest -q -m slow                      (only with --slow)
#   trace      .venv/bin/python -m fabrail trace --allow-missing          (structure gate)
#
# Usage
#   scripts/check_all.sh [--slow] [--no-maturin] [--fail-fast] [--only "fmt clippy …"] [-h]
#     --slow        also run the slow pytest tier
#     --no-maturin  reuse the installed _native (e.g. while another agent owns maturin)
#     --fail-fast   stop at the first failing step (default: run all, then summarise)
#     --only LIST   run only the named steps
#
# Environment
#   CARGO_TARGET_DIR   base directory; steps use <base>/checkall-{host,nostd,firmware,maturin}
#                      (default target/). Host, cross and firmware builds use different
#                      compilers, so they never share a target dir.
#   CARGO_BUILD_JOBS   default 3
#
# Exit status: 0 only if every selected step passed. Commands and their measured
# durations are in CLAUDE.md.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SLOW=0
MATURIN=1
FAIL_FAST=0
ONLY=""
while [ $# -gt 0 ]; do
    case "$1" in
        --slow) SLOW=1; shift ;;
        --no-maturin) MATURIN=0; shift ;;
        --fail-fast) FAIL_FAST=1; shift ;;
        --only) ONLY="${2:?--only needs a list}"; shift 2 ;;
        --only=*) ONLY="${1#--only=}"; shift ;;
        -h|--help) sed -n '2,32p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

BASE_TD="${CARGO_TARGET_DIR:-$ROOT/target}"
unset CARGO_TARGET_DIR
export CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-3}"
PY="$ROOT/.venv/bin/python"

RESULTS=()
FAILED=0

selected() {
    [ -z "$ONLY" ] && return 0
    case " $ONLY " in *" $1 "*) return 0 ;; *) return 1 ;; esac
}

run() {
    # run NAME CMD...
    local name="$1"; shift
    selected "$name" || return 0
    printf '\n== %s: %s\n' "$name" "$*"
    local t0=$SECONDS
    if "$@"; then
        RESULTS+=("PASS  $name ($((SECONDS - t0)) s)")
    else
        RESULTS+=("FAIL  $name ($((SECONDS - t0)) s)")
        FAILED=$((FAILED + 1))
        if [ "$FAIL_FAST" = 1 ]; then summary; exit 1; fi
    fi
}

skip() {
    selected "$1" || return 0
    RESULTS+=("SKIP  $1 ($2)")
}

summary() {
    echo
    echo "== check_all summary"
    for r in ${RESULTS[@]+"${RESULTS[@]}"}; do echo "  $r"; done
    if [ "$FAILED" -eq 0 ]; then echo "check_all: OK"; else echo "check_all: $FAILED step(s) FAILED"; fi
}

host() { CARGO_TARGET_DIR="$BASE_TD/checkall-host" "$@"; }

run fmt host cargo fmt --all --check
run clippy host cargo clippy --workspace --all-targets --all-features -- -D warnings
run test host cargo test --profile itest
run nostd env FABRAIL_REQUIRE_CROSS=1 CARGO_TARGET_DIR="$BASE_TD/checkall-nostd" \
    scripts/check_nostd.sh --strict
run firmware env CARGO_TARGET_DIR="$BASE_TD/checkall-firmware" scripts/check_firmware.sh

if [ "$MATURIN" = 1 ]; then
    if [ -x "$PY" ] && command -v maturin >/dev/null 2>&1; then
        run maturin env VIRTUAL_ENV="$ROOT/.venv" PATH="$ROOT/.venv/bin:$PATH" \
            CARGO_TARGET_DIR="$BASE_TD/maturin" maturin develop --release --offline
    else
        run maturin sh -c 'echo "missing .venv or maturin (see CLAUDE.md: python3 -m venv --system-site-packages .venv)"; exit 1'
    fi
else
    skip maturin "--no-maturin"
fi

if [ -x "$PY" ]; then
    run pytest "$PY" -m pytest -q
    if [ "$SLOW" = 1 ]; then
        run slow "$PY" -m pytest -q -m slow
    else
        skip slow "use --slow"
    fi
    run trace "$PY" -m fabrail trace --allow-missing
else
    run pytest sh -c 'echo "missing .venv/bin/python"; exit 1'
    skip slow "no .venv"
    skip trace "no .venv"
fi

summary
[ "$FAILED" -eq 0 ]
