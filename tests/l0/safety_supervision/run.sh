#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-SUP: CPU supervision on a bare Spin through an ST-Link (ADR 0009, issue luizvilla/Core#16).
##
## Builds env SAFETY_SUP, flashes it over the ST-Link and runs every case (run.py). It REFLASHES the board, so
## it runs only when asked: CORE_TARGET=1 (tests/run_all.sh --l0 --target sets it) or --run. Otherwise it
## reports BLOCKED and touches nothing. Afterwards the board holds the SAFETY_SUP firmware: flash env USB again
## before a bench session.
## @return 0 when every criterion passes or the test is skipped.
##
## @par Usage
##   bash tests/l0/safety_supervision/run.sh --run              # build, flash, run (campaign of its own)
##   bash tests/l0/safety_supervision/run.sh --run --no-flash   # board already flashed
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
run=${CORE_TARGET:-0}
args=()
for arg in "$@"; do
    case "$arg" in
        --run) run=1 ;;
        --no-flash) args+=(--no-flash) ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
blocked() {
    for n in 01 02 03 04 05 06 07 08 09; do
        printf '@@\tL0-SUP-%s\tBLOCKED\tCPU supervision on the Spin\ton target\t%s\n' "$n" "$1"
    done
}
if [ "$run" != 1 ]; then
    blocked "not requested: reflashes the board; run tests/run_all.sh --l0 --target or $0 --run"
    exit 0
fi
if [ -z "${CORE_TEST_OUT:-}" ]; then
    # Standalone: a campaign of its own, so the record, the review page and the control center show it.
    sha=$(git -C "$repo" rev-parse --short=7 HEAD)
    [ -z "$(git -C "$repo" status --porcelain)" ] || sha="$sha-dirty"
    mkdir -p "$repo/tests/out"
    campaign=$(mktemp -d "$repo/tests/out/$(date +%F)_${sha}_$(date +%H%M%S)_XXXXXX")
    CORE_TEST_OUT="$campaign" CORE_TARGET=1 bash "$0" "$@" 2>&1 | tee "$campaign/l0_safety_supervision.log"
    rc=${PIPESTATUS[0]}
    python3 "$repo/tests/common/make_record.py" --out "$campaign" --levels L0 "$campaign/l0_safety_supervision.log" >/dev/null
    py=python3; [ -x "$repo/.venv/bin/python" ] && py="$repo/.venv/bin/python"
    "$py" "$repo/tests/common/review_report.py" "$campaign"
    python3 "$repo/tests/common/control_center.py" --out "$repo/tests/out"
    exit "$rc"
fi
if ! bash "$repo/tests/common/target_preflight.sh" --env=SAFETY_SUP; then
    blocked "preflight failed, see the MISSING lines above (bash tests/common/target_preflight.sh --install)"
    exit 0
fi
python3 "$here/run.py" --out "$CORE_TEST_OUT/l0_sup" "${args[@]}"
