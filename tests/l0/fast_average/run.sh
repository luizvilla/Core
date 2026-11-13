#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file run.sh
## @brief L0-FAV: builds and runs the tests of the DMA averaging arithmetic on the host.
##
## Compiles test_fast_average.cpp with -Werror; the code under test is the header
## zephyr/modules/owntech_spin_api/zephyr/src/data/data_fast_average.h (no hardware access).
## @return The test binary's exit code: 0 when every criterion passes.
## @see README.md in this folder
##
## @par Usage
##   bash tests/l0/fast_average/run.sh
source "$(dirname "$0")/../../common/l0_build.sh"
L0_INCLUDES+=(-I"$REPO_ROOT/zephyr/modules/owntech_spin_api/zephyr/src/data")
l0_build test_fast_average \
    --strict "$REPO_ROOT/tests/l0/fast_average/test_fast_average.cpp"
"$L0_BIN/test_fast_average"
