#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file build.sh
## @brief Builds the firmware block: controlStep() and its controllers as a Pulsim C-Block shared library.
##
## Compiles firmware_block.cpp and the firmware sources behind controlStep()
## (src/app, src/boost_converter, src/grid_observer) with -Wall -Wextra
## -Werror, and the single_phase_inverter and control_library sources without
## -Werror, all with -fPIC -O2, then links one shared library. No sanitizer:
## the library is loaded into the Python process by ctypes.
##
## Uses the same include paths and library override (SINGLE_PHASE_INVERTER_DIR)
## as the L0 builds; see tests/common/l0_build.sh.
## @param $1 Output path of the shared library (default: firmware_block.so in
## a temporary folder printed on stdout).
## @return 0 on success; the path of the library is printed on the last line.
## @see README.md in this folder
##
## @par Usage
##   bash tests/l1/firmware_block/build.sh /tmp/firmware_block.so
set -euo pipefail
OUT_PATH=${1:-}
source "$(dirname "$0")/../../common/l0_build.sh"
[ -n "$OUT_PATH" ] || OUT_PATH="$(mktemp -d)/firmware_block.so"
INV="$INV_LIB_DIR/src"
CTL="$LIB_ROOT/control_library/src"
FLAGS=(-std=c++17 -O2 -fPIC)
objects=()
compile() {
    local mode=$1 src=$2 obj
    obj="$L0_BIN/$(echo "$src" | tr '/' '_').o"
    if [ "$mode" = strict ]; then
        "$L0_CXX" "${FLAGS[@]}" -Wall -Wextra -Werror "${L0_INCLUDES[@]}" -c "$src" -o "$obj"
    else
        "$L0_CXX" "${FLAGS[@]}" -w "${L0_INCLUDES[@]}" -c "$src" -o "$obj"
    fi
    objects+=("$obj")
}
for src in "$REPO_ROOT/tests/l1/firmware_block/firmware_block.cpp" \
           "$REPO_ROOT/src/app/control_step.cpp" \
           "$REPO_ROOT/src/boost_converter/boost.cpp" \
           "$REPO_ROOT/src/grid_observer/gridObserver.cpp"; do
    compile strict "$src"
done
for src in "$INV/singlePhaseInverterFollowing.cpp" "$INV/sogi.cpp" \
           "$CTL/pid.cpp" "$CTL/transform.cpp" "$CTL/trigo.cpp"; do
    compile lib "$src"
done
mkdir -p "$(dirname "$OUT_PATH")"
"$L0_CXX" -shared "${objects[@]}" -lm -o "$OUT_PATH"
echo "$OUT_PATH"
