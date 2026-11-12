#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-TIM: on-target timing tests on the Spin through an ST-Link (ADR 0005).
##
## Builds the PlatformIO env TIMING, flashes it over the ST-Link, waits for the harness, reads g_timing_report
## over SWD and prints the result lines. It REFLASHES the board, so it runs only when asked: CORE_TARGET=1
## (tests/run_all.sh --l0 --target sets it) or --run. Otherwise it reports BLOCKED and touches nothing.
## After the run the board holds the TIMING firmware: flash the USB env again before a bench session.
## @return 0 when every criterion passes or the test is skipped.
##
## @par Usage
##   bash tests/l0/timing/run.sh --run                       # build, flash, measure, compare with baseline.json
##   bash tests/l0/timing/run.sh --run --update-baseline     # same, then write baseline.json from this run
##   bash tests/l0/timing/run.sh --run --no-flash            # read the board as it is (already flashed)
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
PIO=${PIO:-$HOME/.platformio/penv/bin/pio}
run=${CORE_TARGET:-0}
update=0
flash=1
for arg in "$@"; do
    case "$arg" in
        --run) run=1 ;;
        --update-baseline) update=1 ;;
        --no-flash) flash=0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
blocked() { printf '@@\tL0-TIM-01\tBLOCKED\tCycle cost per controlStep section on the Spin\ton target\t%s\n' "$1"; }

if [ "$run" != 1 ]; then
    blocked "not requested: reflashes the board; run tests/run_all.sh --l0 --target or tests/l0/timing/run.sh --run"
    exit 0
fi
if [ -z "${CORE_TEST_OUT:-}" ]; then
    # Standalone: run as a campaign of its own, so the record, the review page and the control center show it.
    sha=$(git -C "$repo" rev-parse --short=7 HEAD)
    [ -z "$(git -C "$repo" status --porcelain)" ] || sha="$sha-dirty"
    mkdir -p "$repo/tests/out"
    campaign=$(mktemp -d "$repo/tests/out/$(date +%F)_${sha}_$(date +%H%M%S)_XXXXXX")
    CORE_TEST_OUT="$campaign" CORE_TARGET=1 bash "$0" "$@" 2>&1 | tee "$campaign/l0_timing.log"
    rc=${PIPESTATUS[0]}
    python3 "$repo/tests/common/make_record.py" --out "$campaign" --levels L0 "$campaign/l0_timing.log" >/dev/null
    py=python3; [ -x "$repo/.venv/bin/python" ] && py="$repo/.venv/bin/python"
    "$py" "$repo/tests/common/review_report.py" "$campaign"
    python3 "$repo/tests/common/control_center.py" --out "$repo/tests/out"
    exit "$rc"
fi
if ! bash "$here/preflight.sh"; then
    blocked "preflight failed, see the MISSING lines above (bash tests/l0/timing/preflight.sh --install)"
    exit 0
fi
campaign=$CORE_TEST_OUT
# Test-case folder: target_manifest.json + timing_report.json, rendered by tests/common/review_report.py.
out="$campaign/l0_tim01"
mkdir -p "$out"
cd "$repo" || exit 1
if [ $flash -eq 1 ]; then
    echo "== build and flash env TIMING"
    "$PIO" run -e TIMING -t upload >"$out/timing_build.txt" 2>&1 || {
        tail -30 "$out/timing_build.txt"
        printf '@@\tL0-TIM-01\tFAIL\tTIMING build or flash\texit 0\tsee %s\n' "$out/timing_build.txt"
        exit 1
    }
fi
echo "== read g_timing_report over SWD"
python3 "$here/read_report.py" --elf "$repo/.pio/build/TIMING/firmware.elf" --out "$out/timing_report.json" --timeout 120
args=("$out/timing_report.json" --baseline "$here/baseline.json" --out "$out")
[ $update -eq 1 ] && args+=(--update-baseline "$here/baseline.json")
python3 "$here/evaluate.py" "${args[@]}"
rc=$?
echo "Table: $out/timing_tim01.md; review page: $campaign/report.html"
echo "The board now runs the TIMING firmware (no tasks, no PWM): flash env USB before using it on the bench."
exit $rc
