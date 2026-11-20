#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-TRP: every protection path of the manifest trips at its threshold, on the SPOC bench (ADR 0009,
##        issue luizvilla/Core#23, C11).
##
## Runs only when asked: CORE_TARGET=1 (tests/run_all.sh --l0 --target sets it) or --run; otherwise BLOCKED.
## Builds and flashes the test build on the CUT through the ST-Link (it stays there: the CUT runs the safety test
## firmware between campaigns), drives the MUX (Core-MUX mux-injector) and reads the MHO984.
## Wiring: TP1-TP3, MUX pins 2 and 4 to TP5 (L0-CAL README); TP5-TP7 for the cases that hold two pins.
## @return 0 when every criterion passes or the test is skipped.
##
## @par Usage
##   bash tests/l0/spoc_trip/run.sh --run                    # campaign of its own (Twist 1.4.2)
##   bash tests/l0/spoc_trip/run.sh --run --board=ownverter  # Ownverter 1.1.0
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
run=${CORE_TARGET:-0}
board=twist
for arg in "$@"; do
    case "$arg" in
        --run) run=1 ;;
        --board=*) board=${arg#--board=} ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
blocked() {
    for n in 01 02 03; do
        printf '@@\tL0-TRP-%s\tBLOCKED\tProtection paths trip at their thresholds\ton the SPOC bench\t%s\n' "$n" "$1"
    done
}
if [ "$run" != 1 ]; then
    blocked "not requested: reflashes the CUT; run tests/run_all.sh --l0 --target or $0 --run"
    exit 0
fi
case "$board" in
    twist) env=SAFETY_L0_TWIST; shield=twist:1_4_2 ;;
    ownverter) env=SAFETY_L0_OWNVERTER; shield=ownverter:1_1_0 ;;
    *) echo "unknown board: $board (twist, ownverter)" >&2; exit 2 ;;
esac
py=python3; [ -x "$repo/.venv/bin/python" ] && py="$repo/.venv/bin/python"
if [ -z "${CORE_TEST_OUT:-}" ]; then
    sha=$(git -C "$repo" rev-parse --short=7 HEAD)
    [ -z "$(git -C "$repo" status --porcelain --untracked-files=no)" ] || sha="$sha-dirty"
    mkdir -p "$repo/tests/out"
    campaign=$(mktemp -d "$repo/tests/out/$(date +%F)_${sha}_$(date +%H%M%S)_XXXXXX")
    CORE_TEST_OUT="$campaign" CORE_TARGET=1 bash "$0" "$@" 2>&1 | tee "$campaign/l0_spoc_trip.log"
    rc=${PIPESTATUS[0]}
    python3 "$repo/tests/common/make_record.py" --out "$campaign" --levels L0 "$campaign/l0_spoc_trip.log" >/dev/null
    "$py" "$repo/tests/common/review_report.py" "$campaign"
    python3 "$repo/tests/common/control_center.py" --out "$repo/tests/out"
    exit "$rc"
fi
if ! bash "$repo/tests/common/spoc_preflight.sh" || ! bash "$repo/tests/common/target_preflight.sh" --env=$env; then
    blocked "preflight failed, see the MISSING lines above"
    exit 0
fi
"$py" "$here/run.py" --out "$CORE_TEST_OUT/l0_trip_$board" --env $env --shield $shield
