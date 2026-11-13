#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## @file build.sh
## @brief Build unchanged Core application/controller sources behind the Pulsim ABI.
## @details Mirrors L0 source/override selection. Core and wrapper use -Werror;
## third-party sources do not. Never falls back to the older workspace firmware.
## @return Nonzero on missing dependencies, compile or link failure.
## @par Usage
##   tests/l1/firmware_block/build.sh
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../.." && pwd)
libs="$repo/owntech/lib/USB"
inv="${SINGLE_PHASE_INVERTER_DIR:-$libs/single_phase_inverter}"
for file in "$inv/src/singlePhaseInverterFollowing.cpp" "$libs/control_library/src/pid.cpp"; do
    if [ ! -f "$file" ]; then echo "Missing dependency: $file. See tests/l1/README.md." >&2; exit 1; fi
done
out="${L1_BUILD_DIR:-$repo/tests/l1/firmware_block/build}"  # a study variant builds elsewhere
mkdir -p "$out"
work=$(mktemp -d "$out/objects.XXXXXX")
trap 'rm -rf "$work"' EXIT
# L1_APP_INCLUDE: a directory holding app/control_step.h with study tuning, searched before src/
includes=(${L1_APP_INCLUDE:+-I"$L1_APP_INCLUDE"} -I"$repo/tests/common/shim" -I"$repo/src" -I"$repo/tests/l1/firmware_block" -I"$inv/src" -I"$libs/control_library/src")
flags=(-std=c++20 -O1 -g -fPIC -Wall -Wextra -fsanitize=undefined -fno-sanitize-recover=undefined)
# L1_APP_SRC: a study copy of control_step.cpp (same interface), compiled instead of src/app/control_step.cpp
app_src="${L1_APP_SRC:-$repo/src/app/control_step.cpp}"
strict=("$repo/tests/l1/firmware_block/wrapper.cpp" "$app_src" "$repo/src/boost_converter/boost.cpp" "$repo/src/grid_observer/gridObserver.cpp")
third=("$inv/src/singlePhaseInverterFollowing.cpp" "$inv/src/sogi.cpp" "$inv/src/apf.cpp" "$libs/control_library/src/pid.cpp" "$libs/control_library/src/transform.cpp" "$libs/control_library/src/trigo.cpp" "$libs/control_library/src/filters.cpp" "$libs/control_library/src/pr.cpp" "$libs/control_library/src/fir.cpp")
objects=()
for src in "${strict[@]}" "${third[@]}"; do
    obj="$work/$(basename "$src").o"
    extra=()
    for own in "${strict[@]}"; do if [ "$src" = "$own" ]; then extra=(-Werror); fi; done
    "${CXX:-g++}" "${flags[@]}" "${extra[@]}" "${includes[@]}" -MMD -MF "$obj.d" -c "$src" -o "$obj"
    objects+=("$obj")
done
"${CXX:-g++}" -shared -fsanitize=undefined "${objects[@]}" -o "$work/libcore_control.so"
mv "$work/libcore_control.so" "$out/libcore_control.so"
python3 "$repo/tests/l1/firmware_block/manifest.py" "$inv" "$work"
