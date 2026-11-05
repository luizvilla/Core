#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief Runs every L1 test case: builds the firmware block, then runs each case in Pulsim.
##
## Called by tests/run_all.sh (--l1). Needs the project virtual environment
## (.venv) with tests/l1/requirements.txt installed. When the environment or
## the pinned Pulsim version is missing, every L1 case is reported BLOCKED
## instead of failing, so L0 campaigns stay usable on machines without it.
##
## Each case script receives the firmware block library and a CSV path in
## L1_OUT_DIR (the campaign folder when run by run_all.sh).
## @return 0 when every criterion passes or is BLOCKED/INFO, 1 otherwise.
## @see tests/l1/README.md
##
## @par Usage
##   bash tests/l1/run.sh
set -uo pipefail
L1_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_ROOT=$(cd "$L1_DIR/../.." && pwd)
PY="$REPO_ROOT/.venv/bin/python"
PULSIM_PIN=$(sed -n 's/^pulsim==//p' "$L1_DIR/requirements.txt")
OUT_DIR=${L1_OUT_DIR:-$(mktemp -d)}

blocked_all() {
    printf '@@\tL1-FWB-01\tBLOCKED\tFirmware block runs inside Pulsim\t—\t%s\n' "$1"
}

if [ ! -x "$PY" ]; then
    blocked_all "no .venv: python3 -m venv .venv && .venv/bin/pip install -r tests/l1/requirements.txt"
    exit 0
fi
found=$("$PY" -I -c "import pulsim; print(pulsim.__version__)" 2>/dev/null || true)
if [ "$found" != "$PULSIM_PIN" ]; then
    blocked_all "Pulsim ${found:-not installed}, pinned $PULSIM_PIN: .venv/bin/pip install -r tests/l1/requirements.txt"
    exit 0
fi

SO="$OUT_DIR/firmware_block.so"
if ! bash "$L1_DIR/firmware_block/build.sh" "$SO" >"$OUT_DIR/firmware_block_build.log" 2>&1; then
    cat "$OUT_DIR/firmware_block_build.log"
    printf '@@\tL1-FWB-01\tFAIL\tFirmware block builds\texit code 0\tbuild failed, see firmware_block_build.log\n'
    exit 1
fi

rc=0
"$PY" -I "$L1_DIR/firmware_block/test_firmware_block.py" "$SO" "$OUT_DIR/L1-FWB-01.csv" || rc=1
exit $rc
