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
## writes record.md.
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
set -uo pipefail

repo=$(cd "$(dirname "$0")/.." && pwd)
levels=()
for arg in "$@"; do
    case "$arg" in
        --l0) levels+=(L0) ;;
        --l1) levels+=(L1) ;;
        -h|--help) sed -n '/^## @brief/,/^##   .*--l1/s/^## \{0,1\}//p' "$0"; exit 0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
[ ${#levels[@]} -eq 0 ] && levels=(L0 L1)

sha=$(git -C "$repo" rev-parse --short=7 HEAD)
git -C "$repo" diff --quiet HEAD -- 2>/dev/null || sha="$sha-dirty"
out="$repo/tests/out/$(date +%F)_$sha"
rm -rf "$out"
mkdir -p "$out"

# Object code used when a test binary dies before reporting its own failures.
object_code() {
    case "$1" in
        boost) echo BST ;;
        grid_observer) echo GOB ;;
        inverter_following) echo GFL ;;
        app) echo APP ;;
        l1) echo FWB ;;
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
    if [ "$level" = L1 ] && [ -f "$dir/run.sh" ]; then
        L1_OUT_DIR="$out" run_script L1 "$dir/run.sh"
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
    --levels "$(IFS=,; echo "${levels[*]}")" "${logs[@]}"
