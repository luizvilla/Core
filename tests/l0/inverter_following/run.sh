#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-GFL: builds and runs the grid-following inverter library tests on the host.
##
## Builds against the single_phase_inverter copy that PlatformIO fetched for the
## USB environment (jalinei main) and prints its commit first. Library and
## control_library sources build without -Werror.
## @return 1 if the library was never fetched, otherwise the test binary's exit
## code: 0 when every criterion passes.
## Set SINGLE_PHASE_INVERTER_DIR to test another library checkout.
## @see README.md in this folder
##
## @par Usage
##   bash tests/l0/inverter_following/run.sh
source "$(dirname "$0")/../../common/l0_build.sh"
INV="$INV_LIB_DIR/src"
CTL="$LIB_ROOT/control_library/src"
if [ ! -f "$INV/singlePhaseInverterFollowing.cpp" ]; then
    echo "single_phase_inverter not fetched: run a PlatformIO build first" >&2
    exit 1
fi
printf 'library: %s @ %s\n' "$INV_LIB_DIR" "$(git -C "$INV" rev-parse --short HEAD 2>/dev/null || echo unknown)"
l0_build test_inverter_following \
    --strict "$REPO_ROOT/tests/l0/inverter_following/test_inverter_following.cpp" \
    --lib    "$INV/singlePhaseInverterFollowing.cpp" "$INV/sogi.cpp" "$INV/apf.cpp" \
             "$CTL/pid.cpp" "$CTL/transform.cpp" "$CTL/trigo.cpp"
"$L0_BIN/test_inverter_following"
