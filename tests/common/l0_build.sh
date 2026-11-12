#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file l0_build.sh
## @brief Build helper sourced by every tests/l0/*/run.sh.
##
## Defines REPO_ROOT, LIB_ROOT (libraries fetched by PlatformIO for the USB
## environment), L0_BIN (temporary build folder, removed on exit) and
## l0_build().
##
## Our sources (tests and src/) build with -Wall -Wextra -Werror; third-party
## library sources (owntech/lib/...) build without -Werror, because their
## warnings are not ours to fix here. Everything builds with UBSan, which aborts
## on undefined behaviour. Include paths: hardware stand-ins, check.h, src/ and
## its subfolders, control_library and single_phase_inverter.
##
## @par SINGLE_PHASE_INVERTER_DIR
## Optional path to another single_phase_inverter checkout (e.g. a fix branch)
## to test instead of the PlatformIO copy; exported as INV_LIB_DIR. The test
## record stamps that checkout's commit.
##
## @par l0_build NAME --strict SOURCES... --lib SOURCES...
## Compiles each source and links $L0_BIN/NAME.
## @see tests/common/README.md
##
## @par Usage
##   source "$(dirname "$0")/../../common/l0_build.sh"
##   l0_build test_x --strict "$REPO_ROOT/tests/l0/x/test_x.cpp" --lib "$LIB_ROOT/..."
##   "$L0_BIN/test_x"
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
LIB_ROOT="$REPO_ROOT/owntech/lib/USB"
# single_phase_inverter checkout under test; override to regression-test a branch.
INV_LIB_DIR=$(cd "${SINGLE_PHASE_INVERTER_DIR:-$LIB_ROOT/single_phase_inverter}" 2>/dev/null && pwd || echo "${SINGLE_PHASE_INVERTER_DIR:-$LIB_ROOT/single_phase_inverter}")
L0_BIN=$(mktemp -d)
trap 'rm -rf "$L0_BIN"' EXIT

L0_CXX=${CXX:-g++}
# Match firmware: control_library's filters.cpp uses C++20 aggregate initialization.
L0_FLAGS=(-std=c++20 -O1 -g -fsanitize=undefined -fno-sanitize-recover=undefined)
L0_INCLUDES=(
    -I"$REPO_ROOT/tests/common/shim"
    -I"$REPO_ROOT/tests/common"
    -I"$REPO_ROOT/src"
    -I"$REPO_ROOT/src/boost_converter"
    -I"$REPO_ROOT/src/grid_observer"
    -I"$LIB_ROOT/control_library/src"
    -I"$INV_LIB_DIR/src"
)

l0_build() {
    local name=$1; shift
    local mode=strict objects=() src obj
    for src in "$@"; do
        case "$src" in
            --strict) mode=strict; continue ;;
            --lib) mode=lib; continue ;;
        esac
        obj="$L0_BIN/$(echo "$src" | tr '/' '_').o"
        if [ "$mode" = strict ]; then
            "$L0_CXX" "${L0_FLAGS[@]}" -Wall -Wextra -Werror "${L0_INCLUDES[@]}" -c "$src" -o "$obj"
        else
            "$L0_CXX" "${L0_FLAGS[@]}" -w "${L0_INCLUDES[@]}" -c "$src" -o "$obj"
        fi
        objects+=("$obj")
    done
    "$L0_CXX" "${L0_FLAGS[@]}" "${objects[@]}" -lm -o "$L0_BIN/$name"
}
