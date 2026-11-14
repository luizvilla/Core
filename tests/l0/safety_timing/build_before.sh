#!/usr/bin/env bash
## SPDX-License-Identifier: LGPL-2.1
## Copyright (c) 2026 OWNTECH TECHNOLOGIES
##
## @file build_before.sh
## @brief Builds (and optionally flashes) B0, the "before" build of the Safety API timing test (issue #27):
##        tests/firmware/safety_timing on Core main at BEFORE_COMMIT, the commit the safety work started from.
##
## Core main has no OWNTECH_APP_DIR hook and no pio_tests.ini, so the firmware is built in a git worktree of that
## commit (.pio/before_worktree, ignored by git): main.cpp goes to its src/, the test overlay without the idle-band
## lines (unknown to main's bindings) to src/app.overlay, and an env TOV_BEFORE is appended to its platformio.ini.
## The ELF is copied to .pio/build/TOV_BEFORE/firmware.elf, where the harness looks for symbols.
## @return the build (or upload) exit code.
##
## @par Usage
##   bash tests/l0/safety_timing/build_before.sh [--flash] > build_TOV_BEFORE.txt
set -u
BEFORE_COMMIT=8111a0dc
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
pio=${PIO:-$HOME/.platformio/penv/bin/pio}
wt="$repo/.pio/before_worktree"
target=()
[ "${1:-}" = --flash ] && target=(-t upload)

if [ ! -d "$wt" ] || [ "$(git -C "$wt" rev-parse --short=8 HEAD 2>/dev/null)" != "$BEFORE_COMMIT" ]; then
    git -C "$repo" worktree remove --force "$wt" 2>/dev/null
    rm -rf "$wt"
    git -C "$repo" worktree add --detach "$wt" "$BEFORE_COMMIT" || exit 1
fi
git -C "$wt" checkout -q -- . && git -C "$wt" clean -qfd src
cp "$repo/tests/firmware/safety_timing/main.cpp" "$wt/src/main.cpp"
grep -v 'idle-band' "$repo/tests/firmware/safety_timing/app.overlay" > "$wt/src/app.overlay"
cat >> "$wt/platformio.ini" <<'INI'

# Safety API timing test (issue #27), before build B0: added by tests/l0/safety_timing/build_before.sh
[env:TOV_BEFORE]
extends = bootloader
board_shield = twist
board_shield_version = 1_4_1
board_build.zephyr.cmake_extra_args = -DBUILD_ENV_NAME=TOV_BEFORE
upload_protocol = custom
upload_command = ${platformio.packages_dir}/tool-openocd/bin/openocd -d1 -s ${platformio.packages_dir}/tool-openocd/openocd/scripts -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program $SOURCE 0x8047800 verify reset exit"
INI
echo "== B0: Core main at $BEFORE_COMMIT, worktree $wt"
"$pio" run -d "$wt" -e TOV_BEFORE "${target[@]}"
rc=$?
if [ $rc -eq 0 ]; then
    mkdir -p "$repo/.pio/build/TOV_BEFORE"
    cp "$wt/.pio/build/TOV_BEFORE/firmware.elf" "$repo/.pio/build/TOV_BEFORE/firmware.elf"
fi
exit $rc
