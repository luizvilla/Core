#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-BST: builds and runs the boost converter controller tests on the host.
##
## Compiles test_boost.cpp and src/boost_converter/boost.cpp with -Werror, and
## control_library sources without it, then runs the binary.
## @return The test binary's exit code: 0 when every criterion passes.
## @see README.md in this folder
##
## @par Usage
##   bash tests/l0/boost/run.sh
source "$(dirname "$0")/../../common/l0_build.sh"
l0_build test_boost \
    --strict "$REPO_ROOT/tests/l0/boost/test_boost.cpp" \
             "$REPO_ROOT/src/boost_converter/boost.cpp" \
    --lib    "$LIB_ROOT/control_library/src/pid.cpp" \
             "$LIB_ROOT/control_library/src/filters.cpp" \
             "$LIB_ROOT/control_library/src/pr.cpp" \
             "$LIB_ROOT/control_library/src/fir.cpp" \
             "$LIB_ROOT/control_library/src/trigo.cpp"
rc=0
python3 "$REPO_ROOT/tests/l0/boost/test_burst_registers.py" || rc=1
"$L0_BIN/test_boost" || rc=1
exit "$rc"
