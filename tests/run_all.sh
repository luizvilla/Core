#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run_all.sh
## @brief Runs a test campaign (L0 and/or L1) and writes its test record.
##
## Runs every tests/l0/*/run.sh and, when present, tests/l1/run.sh, logging
## each one to the campaign folder. A script that exits non-zero without
## reporting a FAIL itself (build error, crash, sanitizer abort) gets a FAIL
## criterion added, so it cannot go unnoticed. tests/common/make_record.py then
## writes record.md, then report.html (the review page, procedure §8.5).
##
## Record folder: tests/out/<YYYY-MM-DD>_<sha7>[-dirty]/ (git-ignored).
## @return 0 when the campaign passes; 1 on any FAIL or when no criterion was
## reported; 2 on a bad option.
## @see docs/testing/test-procedure.md §7, tests/README.md
##
## @par Usage
##   tests/run_all.sh          # L0 then L1
##   tests/run_all.sh --l0     # L0 only
##   tests/run_all.sh --l1     # L1 only
##   tests/run_all.sh --l0 --target   # also the on-target L0-TIM tests: REFLASHES the Spin over the ST-Link
set -uo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)
levels=()
for arg in "$@"; do
    case "$arg" in
        --l0) levels+=(L0) ;;
        --l1) levels+=(L1) ;;
        --target) export CORE_TARGET=1 ;;
        -h|--help) sed -n '/^## @brief/,/^##   .*--target/s/^## \{0,1\}//p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
[ ${#levels[@]} -eq 0 ] && levels=(L0 L1)

sha=$(git -C "$repo" rev-parse --short=7 HEAD)
[ -z "$(git -C "$repo" status --porcelain)" ] || sha="$sha-dirty"
mkdir -p "$repo/tests/out"
# Keep repeated campaigns/profiles instead of deleting earlier evidence.
out=$(mktemp -d "$repo/tests/out/$(date +%F)_${sha}_$(date +%H%M%S)_XXXXXX")
export CORE_TEST_OUT="$out"

# Object code used when a test binary dies before reporting its own failures.
object_code() {
    case "$1" in
        boost) echo BST ;;
        grid_observer) echo GOB ;;
        inverter_following) echo GFL ;;
        app) echo APP ;;
        fast_average) echo FAV ;;
        timing) echo TIM-01 ;;
        safety_supervision) echo SUP-00 ;;
        l1) echo INT-00 ;;
        *) echo "$1" | tr '[:lower:]' '[:upper:]' ;;
    esac
}

# Runs one test script, logging its output. If it exits non-zero without
# reporting a FAIL itself (build error, crash, sanitizer abort), a FAIL
# criterion is added so the failure cannot go unnoticed.
run_script() {
    local level=$1 script=$2 name log rc
    name=$(basename "$(dirname "$script")")
    log="$out/${level,,}_$name.log"
    echo "== $level $name"
    bash "$script" >"$log" 2>&1
    rc=$?
    if [ $rc -ne 0 ] && ! grep -q $'^@@\t[^\t]*\tFAIL\t' "$log"; then
        printf '@@\t%s-%s\tFAIL\t%s did not complete\texit code 0\texit code %d, see %s\n' \
            "$level" "$(object_code "$name")" "$name" "$rc" "$(basename "$log")" >>"$log"
    fi
}

for level in "${levels[@]}"; do
    dir="$repo/tests/${level,,}"
    if [ "$level" = L1 ] && [ -x "$dir/run.sh" ]; then
        run_script L1 "$dir/run.sh"
        continue
    fi
    shopt -s nullglob
    scripts=("$dir"/*/run.sh)
    shopt -u nullglob
    if [ ${#scripts[@]} -eq 0 ]; then
        echo "== $level: no test cases yet"
        continue
    fi
    for script in "${scripts[@]}"; do run_script "$level" "$script"; done
done

shopt -s nullglob
logs=("$out"/*.log)
shopt -u nullglob
echo
python3 "$repo/tests/common/make_record.py" --out "$out" \
    --levels "$(IFS=,; echo "${levels[*]}")" "${logs[@]}" && rc=0 || rc=$?
# The review page needs matplotlib, which lives in the test environment.
if [ -x "$repo/.venv/bin/python" ]; then
    "$repo/.venv/bin/python" "$repo/tests/common/review_report.py" "$out" ||
        echo "Review page not written (see the error above); record.md is complete."
else
    echo "Review page skipped: no .venv (tests/l1/setup.sh creates it)."
fi
# Where the whole validation stands, across campaigns (standard library only).
python3 "$repo/tests/common/control_center.py" --out "$(dirname "$out")" ||
    echo "Control center not updated (see the error above)."
exit $rc
