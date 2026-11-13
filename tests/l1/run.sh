#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## @file run.sh
## @brief Run L1 integration checks and emit explicit blocked formal catalogue cases.
## @return Nonzero if any implemented integration criterion fails.
## @par Usage
##   tests/run_all.sh --l1
set -uo pipefail
repo=$(cd "$(dirname "$0")/../.." && pwd)
py="$repo/.venv/bin/python"
if [ ! -x "$py" ]; then echo 'Missing Core/.venv; see tests/l1/README.md' >&2; exit 1; fi
bash "$repo/tests/l1/firmware_block/build.sh" || exit 1
build="$repo/tests/l1/firmware_block/build"
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
 -I"$repo/tests/common/shim" -I"$repo/tests/common" -I"$repo/src" -I"$repo/tests/l1/firmware_block" \
 -I"${SINGLE_PHASE_INVERTER_DIR:-$repo/owntech/lib/USB/single_phase_inverter}/src" -I"$repo/owntech/lib/USB/control_library/src" \
 "$repo/tests/l1/integration/test_wrapper.cpp" -L"$build" -lcore_control -Wl,-rpath,"$build" -o "$build/test_wrapper" || exit 1
out="${CORE_TEST_OUT:-$repo/tests/out/l1-development}"
mkdir -p "$out"
cp "$build/manifest.json" "$out/firmware-manifest.json"
cp "$repo/tests/l1/contract.json" "$out/contract.json"
"$py" "$repo/tests/l1/campaign_manifest.py" "$out"
rc=0
"$build/test_wrapper" || rc=1
"$py" "$repo/tests/l1/integration/test_rc.py" || rc=1
"$py" "$repo/tests/l1/integration/test_model.py" || rc=1
"$py" "$repo/tests/l1/integration/test_pv.py" || rc=1
"$py" "$repo/tests/l1/integration/test_pv_lambertw.py" || rc=1
"$py" "$repo/tests/l1/application/test_switched.py" || rc=1
"$py" "$repo/tests/l1/application/test_settle.py" || rc=1  # L1-APP-06: BLOCKED unless L1_LONG=1 (about one hour)
"$py" "$repo/tests/l1/blocked.py"
exit "$rc"
