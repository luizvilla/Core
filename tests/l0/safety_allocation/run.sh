#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-ALC: protection allocation (ADR 0009, issue luizvilla/Core#17). No hardware.
##
## L0-ALC-01..10: the allocation on synthetic boards (test_allocation.py, instant). L0-ALC-11/12: every shield
## devicetree builds with the allocation and matches its committed protection table (build_check.py, about 20 s
## per shield). Writes the test-case folder l0_alc/ for the review page. Standalone (--run outside a campaign), it
## writes a campaign of its own.
## @return 0 when the scripts complete (verdicts are in the result lines).
##
## @par Usage
##   bash tests/l0/safety_allocation/run.sh            # inside tests/run_all.sh, or standalone campaign
##   bash tests/l0/safety_allocation/run.sh --update-docs   # rewrite docs/safety/boards/ from the build
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
args=()
for arg in "$@"; do
    case "$arg" in
        --update-docs) args+=(--update-docs) ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done
if [ -z "${CORE_TEST_OUT:-}" ]; then
    sha=$(git -C "$repo" rev-parse --short=7 HEAD)
    [ -z "$(git -C "$repo" status --porcelain --untracked-files=no)" ] || sha="$sha-dirty"
    mkdir -p "$repo/tests/out"
    campaign=$(mktemp -d "$repo/tests/out/$(date +%F)_${sha}_$(date +%H%M%S)_XXXXXX")
    CORE_TEST_OUT="$campaign" bash "$0" "$@" 2>&1 | tee "$campaign/l0_safety_allocation.log"
    rc=${PIPESTATUS[0]}
    python3 "$repo/tests/common/make_record.py" --out "$campaign" --levels L0 "$campaign/l0_safety_allocation.log" >/dev/null
    py=python3; [ -x "$repo/.venv/bin/python" ] && py="$repo/.venv/bin/python"
    "$py" "$repo/tests/common/review_report.py" "$campaign"
    python3 "$repo/tests/common/control_center.py" --out "$repo/tests/out"
    exit "$rc"
fi
out="$CORE_TEST_OUT/l0_alc"
mkdir -p "$out"
python3 "$here/test_allocation.py" || exit 1
python3 "$here/build_check.py" --out "$out" "${args[@]}" || exit 1
python3 - "$out" <<'PY'
import json, sys, os
out = sys.argv[1]
man = {'title': 'L0-ALC · Protection allocation on every shield',
       'purpose': 'Checks that the build assigns each limit the strongest hardware path the board offers '
                  '(comparator or ADC watchdog to an HRTIM fault input), refuses a hardware limit with no path '
                  'unless the shield states an exception, and that the committed protection tables match the build '
                  '(ADR 0009).',
       'criteria': ['L0-ALC-'],
       'conditions': [['Synthetic boards', 'L0-ALC-01..10: test_allocation.py on hand-made models using real Spin pins'],
                      ['Shields', 'L0-ALC-11/12: every zephyr/boards/shields/*/*.overlay, env SAFETY_CHECK, target '
                                  'idedata (devicetree + allocation, no compile), strict mode'],
                      ['Routing tables', 'RM0440 Rev 8 Tables 82, 196, 197, 220']],
       'shields': json.load(open(os.path.join(out, 'shields.json')))}
json.dump(man, open(os.path.join(out, 'allocation_manifest.json'), 'w'), indent=1)
PY
