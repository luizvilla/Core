#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-APP: builds and runs the application control step tests on the host.
##
## Compiles test_app.cpp and the firmware sources behind controlStep()
## (src/app/control_step.cpp, boost.cpp, gridObserver.cpp) with -Werror, and the
## single_phase_inverter and control_library sources without it, then runs
## the binary.
## @return 1 if the libraries were never fetched, otherwise the test binary's
## exit code: 0 when every criterion passes.
## Set SINGLE_PHASE_INVERTER_DIR to test another library checkout.
## @see README.md in this folder
##
## @par Usage
##   bash tests/l0/app/run.sh
source "$(dirname "$0")/../../common/l0_build.sh"
INV="$INV_LIB_DIR/src"
CTL="$LIB_ROOT/control_library/src"
if [ ! -f "$INV/singlePhaseInverterFollowing.cpp" ]; then
    echo "single_phase_inverter not fetched: run a PlatformIO build first" >&2
    exit 1
fi
printf 'library: %s @ %s\n' "$INV_LIB_DIR" "$(git -C "$INV" rev-parse --short HEAD 2>/dev/null || echo unknown)"
l0_build test_app \
    --strict "$REPO_ROOT/tests/l0/app/test_app.cpp" \
             "$REPO_ROOT/src/app/control_step.cpp" \
             "$REPO_ROOT/src/boost_converter/boost.cpp" \
             "$REPO_ROOT/src/grid_observer/gridObserver.cpp" \
    --lib    "$INV/singlePhaseInverterFollowing.cpp" "$INV/sogi.cpp" \
             "$CTL/pid.cpp" "$CTL/transform.cpp" "$CTL/trigo.cpp" \
             "$CTL/filters.cpp" "$CTL/pr.cpp" "$CTL/fir.cpp"
"$L0_BIN/test_app"
