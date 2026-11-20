#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-ADC: offset of the CUT ADC at the low end, on the SPOC bench
##        (issue luizvilla/Core#23, finding of 2026-11-20).
##
## Runs only when asked: CORE_TARGET=1 (tests/run_all.sh --l0 --target sets it) or --run; otherwise BLOCKED.
## Builds and flashes SAFETY_L0_TWIST on the CUT through the ST-Link (it stays there), resets it twice, routes the
## MUX. Wiring as L0-TRP (TP1-TP3; the injector on TP5, for the armed reading).
## @return 0 when every criterion passes or the test is skipped.
##
## @par Usage
##   bash tests/l0/spoc_adc/run.sh --run      # campaign of its own (Twist 1.4.2)
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
run=${CORE_TARGET:-0}
for arg in "$@"; do
    case "$arg" in
        --run) run=1 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
blocked() {
    for n in 00 01 02 03 04 05 06; do
        printf '@@\tL0-ADC-%s\tBLOCKED\tOffset of the CUT ADC\ton the SPOC bench\t%s\n' "$n" "$1"
    done
}
if [ "$run" != 1 ]; then
    blocked "not requested: reflashes the CUT; run tests/run_all.sh --l0 --target or $0 --run"
    exit 0
fi
py=python3; [ -x "$repo/.venv/bin/python" ] && py="$repo/.venv/bin/python"
if [ -z "${CORE_TEST_OUT:-}" ]; then
    sha=$(git -C "$repo" rev-parse --short=7 HEAD)
    [ -z "$(git -C "$repo" status --porcelain --untracked-files=no)" ] || sha="$sha-dirty"
    mkdir -p "$repo/tests/out"
    campaign=$(mktemp -d "$repo/tests/out/$(date +%F)_${sha}_$(date +%H%M%S)_XXXXXX")
    CORE_TEST_OUT="$campaign" CORE_TARGET=1 bash "$0" "$@" 2>&1 | tee "$campaign/l0_spoc_adc.log"
    rc=${PIPESTATUS[0]}
    python3 "$repo/tests/common/make_record.py" --out "$campaign" --levels L0 "$campaign/l0_spoc_adc.log" >/dev/null
    "$py" "$repo/tests/common/review_report.py" "$campaign"
    python3 "$repo/tests/common/control_center.py" --out "$repo/tests/out"
    exit "$rc"
fi
if ! bash "$repo/tests/common/spoc_preflight.sh" || ! bash "$repo/tests/common/target_preflight.sh" --env=SAFETY_L0_TWIST; then
    blocked "preflight failed, see the MISSING lines above"
    exit 0
fi
"$py" "$here/run.py" --out "$CORE_TEST_OUT/l0_adc"
