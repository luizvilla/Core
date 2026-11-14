#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-API: one Safety API (ADR 0009, issue luizvilla/Core#22).
##
## L0-API-01 (build_mode.py, no board, about 3 min): development-only calls do not compile in a production build.
## L0-API-02..06 (run.py): builds envs SAFETY_API and SAFETY_API_DEV, flashes each over the ST-Link and runs every
## case. That part REFLASHES the board, so it runs only when asked: CORE_TARGET=1 (tests/run_all.sh --l0 --target
## sets it) or --run; otherwise it reports BLOCKED. Afterwards the board holds the SAFETY_API_DEV firmware: flash env
## USB again before a bench session.
## @return 0 when the scripts complete (verdicts are in the result lines).
##
## @par Usage
##   bash tests/l0/safety_api/run.sh --run              # build test + on target (campaign of its own)
##   bash tests/l0/safety_api/run.sh --run --no-flash   # boards already flashed (not useful: two builds)
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
if [ -z "${CORE_TEST_OUT:-}" ]; then
    # Standalone: a campaign of its own, so the record, the review page and the control center show it.
    sha=$(git -C "$repo" rev-parse --short=7 HEAD)
    [ -z "$(git -C "$repo" status --porcelain --untracked-files=no)" ] || sha="$sha-dirty"
    mkdir -p "$repo/tests/out"
    campaign=$(mktemp -d "$repo/tests/out/$(date +%F)_${sha}_$(date +%H%M%S)_XXXXXX")
    CORE_TEST_OUT="$campaign" CORE_TARGET=$run bash "$0" "$@" 2>&1 | tee "$campaign/l0_safety_api.log"
    rc=${PIPESTATUS[0]}
    python3 "$repo/tests/common/make_record.py" --out "$campaign" --levels L0 "$campaign/l0_safety_api.log" >/dev/null
    py=python3; [ -x "$repo/.venv/bin/python" ] && py="$repo/.venv/bin/python"
    "$py" "$repo/tests/common/review_report.py" "$campaign"
    python3 "$repo/tests/common/control_center.py" --out "$repo/tests/out"
    exit "$rc"
fi
out="$CORE_TEST_OUT/l0_api"
mkdir -p "$out"
python3 "$here/build_mode.py" --out "$out" || exit 1
blocked() {
    for n in 02 03 04 05 06; do
        printf '@@\tL0-API-%s\tBLOCKED\tOne Safety API on the Spin\ton target\t%s\n' "$n" "$1"
    done
}
if [ "$run" != 1 ]; then
    blocked "not requested: reflashes the board; run tests/run_all.sh --l0 --target or $0 --run"
    exit 0
fi
if ! bash "$repo/tests/common/target_preflight.sh" --env=SAFETY_API; then
    blocked "preflight failed, see the MISSING lines above (bash tests/common/target_preflight.sh --install)"
    exit 0
fi
python3 "$here/run.py" --out "$out" "${args[@]}"
